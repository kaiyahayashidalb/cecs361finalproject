`timescale 1ns / 1ps
//==============================================================================
// Simple High-Level I2C Master for Pmod CMPS2 (MMC34160PJ)
// 
// Clean, well-documented design optimized for easy verification.
// Supports register read/write with multi-byte transfers.
//==============================================================================
module i2c_master #(
    parameter CLK_HZ = 100_000_000,
    parameter I2C_HZ = 400_000
)(
    input  wire        clk,
    input  wire        rst,
    
    // Simple transaction interface
    input  wire        start,           // Pulse to begin transaction
    input  wire [6:0]  device_addr,     // 7-bit I2C address (0x30 for CMPS2)
    input  wire [7:0]  reg_addr,        // Register address
    input  wire        rw,              // 0=write, 1=read
    input  wire [2:0]  num_bytes,       // Bytes to transfer (1-7)
    input  wire [7:0]  wr_data,         // Data to write
    
    output reg  [7:0]  rd_data,         // Received data byte
    output reg         rd_valid,        // Pulse when rd_data valid
    output reg         busy,
    output reg         done,
    output reg         error,           // NACK received
    
    // Debug output
    output wire [3:0]  debug_state,      // Current FSM state for debugging
    
    // I2C bus
    inout  wire        sda,
    inout  wire        scl
);

    assign debug_state = state;

    //==========================================================================
    // Clock divider for I2C timing
    // Each I2C bit has 4 phases: setup, high1, high2, low
    //==========================================================================
    localparam integer CLKS_PER_QUARTER = CLK_HZ / (I2C_HZ * 4);
    
    reg [$clog2(CLKS_PER_QUARTER)-1:0] clk_div;
    reg [1:0] quarter;  // 0,1,2,3 within each bit period
    wire quarter_tick = (clk_div == 0);
    
    // Use state instead of busy to control clock divider
    // This avoids timing issues when busy transitions
    wire running = (state != ST_IDLE);
    
    always @(posedge clk) begin
        if (rst || !running) begin
            clk_div <= 0;  // Start at 0 so first tick happens immediately
            quarter <= 0;
        end else if (clk_div == 0) begin
            clk_div <= CLKS_PER_QUARTER - 1;
            quarter <= quarter + 1;
        end else begin
            clk_div <= clk_div - 1;
        end
    end

    //==========================================================================
    // I2C open-drain signals
    //==========================================================================
    reg sda_oe, scl_oe;  // Output enable (active high = drive low)
    
    // Open-drain: drive low when oe=1, else release to high-Z (pulled high externally)
    assign sda = sda_oe ? 1'b0 : 1'bz;
    assign scl = scl_oe ? 1'b0 : 1'bz;
    
    // Read SDA - just read the pin directly (pull-up ensures high when released)
    wire sda_in = sda;

    //==========================================================================
    // State machine
    //==========================================================================
    localparam [3:0]
        ST_IDLE      = 4'd0,
        ST_START     = 4'd1,   // Generate START condition
        ST_ADDR_W    = 4'd2,   // Send device address + write bit
        ST_REG       = 4'd3,   // Send register address
        ST_RESTART   = 4'd4,   // Generate repeated START for reads
        ST_ADDR_R    = 4'd5,   // Send device address + read bit
        ST_WRITE     = 4'd6,   // Write data byte
        ST_READ      = 4'd7,   // Read data byte
        ST_ACK_CHECK = 4'd8,   // Check slave ACK after we send
        ST_ACK_SEND  = 4'd9,   // Send ACK/NACK after we receive
        ST_STOP      = 4'd10;  // Generate STOP condition
    
    reg [3:0] state, next_state;
    reg [7:0] shift_out;       // Byte being sent
    reg [7:0] shift_in;        // Byte being received
    reg [2:0] bit_cnt;         // Bit counter (7 down to 0)
    reg [2:0] bytes_left;      // Bytes remaining
    reg       is_read_op;      // Stored read/write flag
    reg [6:0] stored_addr;     // Stored device address
    reg [7:0] stored_reg;      // Stored register address
    reg [7:0] stored_data;     // Stored write data
    reg       ack_bit;         // Sampled ACK value

    //==========================================================================
    // Main FSM
    //==========================================================================
    always @(posedge clk) begin
        if (rst) begin
            state <= ST_IDLE;
            busy <= 0;
            done <= 0;
            error <= 0;
            rd_valid <= 0;
            rd_data <= 0;
            sda_oe <= 0;
            scl_oe <= 0;
            shift_out <= 0;
            shift_in <= 0;
            bit_cnt <= 0;
            bytes_left <= 0;
            is_read_op <= 0;
            stored_addr <= 0;
            stored_reg <= 0;
            stored_data <= 0;
            ack_bit <= 0;
            next_state <= ST_IDLE;
        end else begin
            // Default: clear pulses
            done <= 0;
            rd_valid <= 0;
            
            case (state)
                //==============================================================
                ST_IDLE: begin
                    busy <= 0;
                    sda_oe <= 0;  // Release
                    scl_oe <= 0;  // Release
                    
                    if (start) begin
                        busy <= 1;
                        error <= 0;
                        stored_addr <= device_addr;
                        stored_reg <= reg_addr;
                        stored_data <= wr_data;
                        is_read_op <= rw;
                        bytes_left <= num_bytes;
                        state <= ST_START;
                    end
                end
                
                //==============================================================
                // START: SDA goes low while SCL is high
                ST_START: begin
                    if (quarter_tick) begin
                        case (quarter)
                            0: begin sda_oe <= 0; scl_oe <= 0; end  // Both high
                            1: begin sda_oe <= 1; end                   // SDA low (START)
                            2: begin scl_oe <= 1; end                   // SCL low
                            3: begin
                                // Prepare to send address + W
                                shift_out <= {stored_addr, 1'b0};
                                bit_cnt <= 7;
                                state <= ST_ADDR_W;
                            end
                        endcase
                    end
                end
                
                //==============================================================
                // Send address + write bit
                ST_ADDR_W: begin
                    if (quarter_tick) begin
                        case (quarter)
                            0: begin sda_oe <= ~shift_out[7]; scl_oe <= 1; end
                            1: begin scl_oe <= 0; end
                            2: begin /* hold */ end
                            3: begin
                                scl_oe <= 1;
                                shift_out <= {shift_out[6:0], 1'b0};
                                if (bit_cnt == 0) begin
                                    next_state <= ST_REG;  // After addr+W, send register
                                    state <= ST_ACK_CHECK;
                                end else begin
                                    bit_cnt <= bit_cnt - 1;
                                end
                            end
                        endcase
                    end
                end
                
                //==============================================================
                // Send register address
                ST_REG: begin
                    if (quarter_tick) begin
                        case (quarter)
                            0: begin sda_oe <= ~shift_out[7]; scl_oe <= 1; end
                            1: begin scl_oe <= 0; end
                            2: begin /* hold */ end
                            3: begin
                                scl_oe <= 1;
                                shift_out <= {shift_out[6:0], 1'b0};
                                if (bit_cnt == 0) begin
                                    next_state <= is_read_op ? ST_RESTART : ST_WRITE;
                                    state <= ST_ACK_CHECK;
                                end else begin
                                    bit_cnt <= bit_cnt - 1;
                                end
                            end
                        endcase
                    end
                end
                
                //==============================================================
                // Send address + read bit
                ST_ADDR_R: begin
                    if (quarter_tick) begin
                        case (quarter)
                            0: begin sda_oe <= ~shift_out[7]; scl_oe <= 1; end
                            1: begin scl_oe <= 0; end
                            2: begin /* hold */ end
                            3: begin
                                scl_oe <= 1;
                                shift_out <= {shift_out[6:0], 1'b0};
                                if (bit_cnt == 0) begin
                                    next_state <= ST_READ;  // After addr+R, start reading
                                    state <= ST_ACK_CHECK;
                                end else begin
                                    bit_cnt <= bit_cnt - 1;
                                end
                            end
                        endcase
                    end
                end
                
                //==============================================================
                // Write data byte
                ST_WRITE: begin
                    if (quarter_tick) begin
                        case (quarter)
                            0: begin sda_oe <= ~shift_out[7]; scl_oe <= 1; end
                            1: begin scl_oe <= 0; end
                            2: begin /* hold */ end
                            3: begin
                                scl_oe <= 1;
                                shift_out <= {shift_out[6:0], 1'b0};
                                if (bit_cnt == 0) begin
                                    next_state <= (bytes_left <= 1) ? ST_STOP : ST_WRITE;
                                    state <= ST_ACK_CHECK;
                                end else begin
                                    bit_cnt <= bit_cnt - 1;
                                end
                            end
                        endcase
                    end
                end
                
                //==============================================================
                // Check ACK from slave
                ST_ACK_CHECK: begin
                    if (quarter_tick) begin
                        case (quarter)
                            0: begin
                                sda_oe <= 0;  // Release SDA for slave ACK
                                scl_oe <= 1;  // SCL low
                            end
                            1: begin
                                scl_oe <= 0;  // SCL high
                            end
                            2: begin
                                // Hold SCL high, sample will be in quarter 3
                            end
                            3: begin
                                scl_oe <= 1;  // SCL low
                                
                                // Sample SDA NOW (while SCL just went low, data still valid)
                                if (sda_in) begin
                                    // NACK - abort
                                    error <= 1;
                                    state <= ST_STOP;
                                end else begin
                                    // ACK received - continue to next state
                                    state <= next_state;
                                    
                                    // Setup for next state
                                    case (next_state)
                                        ST_REG: begin
                                            shift_out <= stored_reg;
                                            bit_cnt <= 7;
                                        end
                                        ST_RESTART: begin
                                            // No setup needed
                                        end
                                        ST_WRITE: begin
                                            shift_out <= stored_data;
                                            bit_cnt <= 7;
                                            bytes_left <= bytes_left - 1;
                                        end
                                        ST_READ: begin
                                            bit_cnt <= 7;
                                            shift_in <= 0;
                                        end
                                        ST_STOP: begin
                                            // No setup needed
                                        end
                                        default: ;
                                    endcase
                                end
                            end
                        endcase
                    end
                end
                
                //==============================================================
                // RESTART: SDA high, then low while SCL high
                ST_RESTART: begin
                    if (quarter_tick) begin
                        case (quarter)
                            0: begin sda_oe <= 0; scl_oe <= 1; end  // SDA high, SCL low
                            1: begin scl_oe <= 0; end                   // SCL high
                            2: begin sda_oe <= 1; end                   // SDA low (START)
                            3: begin
                                scl_oe <= 1;  // SCL low
                                shift_out <= {stored_addr, 1'b1};  // Address + R
                                bit_cnt <= 7;
                                state <= ST_ADDR_R;
                            end
                        endcase
                    end
                end
                
                //==============================================================
                // Read a byte from slave
                ST_READ: begin
                    if (quarter_tick) begin
                        case (quarter)
                            0: begin
                                sda_oe <= 0;  // Release SDA - slave drives
                                scl_oe <= 1;  // SCL low
                            end
                            1: begin
                                scl_oe <= 0;  // SCL high
                            end
                            2: begin
                                // Sample SDA and shift into register
                                shift_in <= {shift_in[6:0], sda_in};
                            end
                            3: begin
                                scl_oe <= 1;  // SCL low
                                
                                if (bit_cnt == 0) begin
                                    // Byte complete - use the UPDATED shift_in with last bit
                                    rd_data <= {shift_in[6:0], sda_in};
                                    rd_valid <= 1;
                                    bytes_left <= bytes_left - 1;
                                    state <= ST_ACK_SEND;
                                end else begin
                                    bit_cnt <= bit_cnt - 1;
                                end
                            end
                        endcase
                    end
                end
                
                //==============================================================
                // Send ACK (more bytes) or NACK (last byte)
                ST_ACK_SEND: begin
                    if (quarter_tick) begin
                        case (quarter)
                            0: begin
                                // ACK=drive low (more data), NACK=release (last byte)
                                sda_oe <= (bytes_left != 0) ? 1 : 0;
                                scl_oe <= 1;
                            end
                            1: begin
                                scl_oe <= 0;  // SCL high
                            end
                            2: begin
                                // Hold
                            end
                            3: begin
                                scl_oe <= 1;  // SCL low
                                
                                if (bytes_left == 0) begin
                                    state <= ST_STOP;
                                end else begin
                                    bit_cnt <= 7;
                                    shift_in <= 0;
                                    state <= ST_READ;
                                end
                            end
                        endcase
                    end
                end
                
                //==============================================================
                // STOP: SDA goes high while SCL is high
                ST_STOP: begin
                    if (quarter_tick) begin
                        case (quarter)
                            0: begin sda_oe <= 1; scl_oe <= 1; end  // SDA low, SCL low
                            1: begin scl_oe <= 0; end                   // SCL high
                            2: begin sda_oe <= 0; end                   // SDA high (STOP)
                            3: begin
                                done <= 1;
                                state <= ST_IDLE;
                            end
                        endcase
                    end
                end
                
                default: state <= ST_IDLE;
            endcase
        end
    end
    
endmodule
