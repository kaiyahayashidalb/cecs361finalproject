`timescale 1ns / 1ps
//==============================================================================
// Magnetometer Driver for Pmod CMPS2 (MMC34160PJ)
// Uses high-level i2c_master transaction interface
//==============================================================================
module magnetometer_driver #(
    parameter CLK_HZ = 100_000_000,  // 100 MHz system clock
    parameter SIM_MODE = 0           // 1 = reduce delays for simulation
)(
    input  wire clk,
    input  wire rst,
    
    // Control interface
    input  wire start_read,          // Pulse high to start a magnetometer reading
    output reg  data_valid,          // High for one cycle when new data is ready
    output reg  busy,                // High while operation in progress
    output reg  error,               // High if NACK received
    
    // Magnetometer data outputs (16-bit signed)
    output reg signed [15:0] mag_x,
    output reg signed [15:0] mag_y,
    output reg signed [15:0] mag_z,
    
    // Debug output - count rd_valid pulses
    output reg [7:0] debug_byte,
    
    // I2C bus
    inout  wire sda,
    inout  wire scl
);

    //==========================================================================
    // Pmod CMPS2 (MMC34160PJ) Constants - Per Digilent Reference Manual
    //==========================================================================
    localparam [6:0] I2C_ADDR = 7'h30;    // 7-bit I2C address (0x60 >> 1)
    
    // Register addresses (from Digilent Pmod CMPS2 Reference Manual)
    localparam [7:0] REG_XOUT_LSB   = 8'h00;  // X axis output LSB
    localparam [7:0] REG_STATUS     = 8'h03;  // Status register
    localparam [7:0] REG_CTRL0      = 8'h07;  // Internal Control Register 0 (NOT 0x08!)
    localparam [7:0] REG_CTRL1      = 8'h08;  // Internal Control Register 1
    localparam [7:0] REG_PRODUCT_ID = 8'h20;  // Product ID (should read 0x06)
    
    // Control Register 0 commands (from Digilent Reference Manual)
    localparam [7:0] CMD_REFILL_CAP = 8'h80;  // Refill capacitor (required before SET/RESET)
    localparam [7:0] CMD_SET        = 8'h20;  // SET action (magnetize sensing resistors)
    localparam [7:0] CMD_RESET      = 8'h40;  // RESET action (reverse magnetization)
    localparam [7:0] CMD_TM_M       = 8'h01;  // Take Measurement (magnetic)

    //==========================================================================
    // I2C Master Interface
    //==========================================================================
    reg         i2c_start;
    reg  [6:0]  i2c_device_addr;
    reg  [7:0]  i2c_reg_addr;
    reg         i2c_rw;
    reg  [2:0]  i2c_num_bytes;
    reg  [7:0]  i2c_wr_data;
    
    wire [7:0]  i2c_rd_data;
    wire        i2c_rd_valid;
    wire        i2c_busy;
    wire        i2c_done;
    wire        i2c_error;
    
    i2c_master #(
        .CLK_HZ(CLK_HZ),
        .I2C_HZ(100_000)   // 100kHz I2C
    ) i2c (
        .clk(clk),
        .rst(rst),
        .start_txn(i2c_start),
        .device_addr(i2c_device_addr),
        .reg_addr(i2c_reg_addr),
        .rw(i2c_rw),
        .num_bytes(i2c_num_bytes),
        .wr_data(i2c_wr_data),
        .rd_data(i2c_rd_data),
        .rd_valid(i2c_rd_valid),
        .busy(i2c_busy),
        .done(i2c_done),
        .error(i2c_error),
        .start_cal(1'b0),
        .sda(sda),
        .scl(scl)
    );

    //==========================================================================
    // FSM States - Following Digilent Pmod CMPS2 Reference Manual
    //==========================================================================
    localparam [4:0]
        S_IDLE           = 5'd0,
        S_REFILL_CAP     = 5'd1,   // Write 0x80 to CTRL0 (refill capacitor)
        S_REFILL_WAIT    = 5'd2,
        S_REFILL_DELAY   = 5'd3,   // Wait 50ms for capacitor
        S_SET_CMD        = 5'd4,   // Write 0x20 to CTRL0 (SET action)
        S_SET_WAIT       = 5'd5,
        S_SET_DELAY      = 5'd6,   // Wait 1ms after SET
        S_TRIG_MEAS      = 5'd7,   // Write 0x01 to CTRL0 (take measurement)
        S_TRIG_WAIT      = 5'd8,
        S_MEAS_DELAY     = 5'd9,   // Wait 8ms for measurement
        S_READ_START     = 5'd10,  // Read 6 bytes from 0x00
        S_READ_WAIT      = 5'd11;
    
    reg [4:0] state;
    reg [2:0] byte_cnt;
    reg [7:0] data_buf [0:5];   // 6 bytes: X_LSB, X_MSB, Y_LSB, Y_MSB, Z_LSB, Z_MSB
    reg       initialized;
    reg [23:0] delay_cnt;       // Counter for delays (need up to 50ms = 5,000,000 cycles)

    //==========================================================================
    // Debug: show byte 0 (X_LSB) or byte 1 (X_MSB) from last read
    // We latch bytes 0 and 1 specifically for X-axis debug
    //==========================================================================
    reg [7:0] debug_x_lsb;
    reg [7:0] debug_x_msb;
    
    always @(posedge clk) begin
        if (rst) begin
            debug_byte <= 0;
            debug_x_lsb <= 0;
            debug_x_msb <= 0;
        end else if (i2c_rd_valid && state == S_READ_WAIT) begin
            // Capture X-axis bytes specifically
            if (byte_cnt == 0) debug_x_lsb <= i2c_rd_data;  // X_LSB
            if (byte_cnt == 1) debug_x_msb <= i2c_rd_data;  // X_MSB
            // Show current byte being received
            debug_byte <= i2c_rd_data;
        end else if (state == S_IDLE && !busy) begin
            // When idle, show X_MSB (more likely to have non-zero bits)
            debug_byte <= debug_x_msb;
        end
    end

    //==========================================================================
    // Capture incoming read bytes into data_buf
    //==========================================================================
    always @(posedge clk) begin
        if (rst) begin
            byte_cnt <= 0;
        end else if (state == S_READ_START) begin
            byte_cnt <= 0;
        end else if (i2c_rd_valid && state == S_READ_WAIT) begin
            data_buf[byte_cnt] <= i2c_rd_data;
            byte_cnt <= byte_cnt + 1;
        end
    end

    //==========================================================================
    // Main FSM - Following Digilent Pmod CMPS2 Initialization Sequence
    //==========================================================================
    always @(posedge clk) begin
        if (rst) begin
            state <= S_IDLE;
            busy <= 0;
            data_valid <= 0;
            error <= 0;
            initialized <= 0;
            mag_x <= 0;
            mag_y <= 0;
            mag_z <= 0;
            i2c_start <= 0;
            i2c_device_addr <= I2C_ADDR;
            i2c_reg_addr <= 0;
            i2c_rw <= 0;
            i2c_num_bytes <= 1;
            i2c_wr_data <= 0;
            delay_cnt <= 0;
        end else begin
            data_valid <= 0;
            i2c_start <= 0;
            
            case (state)
                //==============================================================
                // IDLE: Wait for start_read pulse
                //==============================================================
                S_IDLE: begin
                    if (start_read) begin
                        busy <= 1;
                        error <= 0;
                        if (initialized) begin
                            // Already initialized, just take a measurement
                            state <= S_TRIG_MEAS;
                        end else begin
                            // First time: do full initialization sequence
                            // Step 1: Refill capacitor (write 0x80 to CTRL0)
                            state <= S_REFILL_CAP;
                        end
                    end else begin
                        busy <= 0;
                    end
                end
                
                //==============================================================
                // INIT STEP 1: Refill capacitor (write 0x80 to register 0x07)
                //==============================================================
                S_REFILL_CAP: begin
                    i2c_device_addr <= I2C_ADDR;
                    i2c_reg_addr <= REG_CTRL0;      // 0x07
                    i2c_rw <= 0;                     // Write
                    i2c_num_bytes <= 1;
                    i2c_wr_data <= CMD_REFILL_CAP;  // 0x80
                    i2c_start <= 1;
                    state <= S_REFILL_WAIT;
                end
                
                S_REFILL_WAIT: begin
                    if (i2c_done) begin
                        if (i2c_error) begin
                            error <= 1;
                            busy <= 0;
                            state <= S_IDLE;
                        end else begin
                            delay_cnt <= 0;
                            state <= S_REFILL_DELAY;
                        end
                    end
                end
                
                // Wait 50ms for capacitor to refill (50ms * 100MHz = 5,000,000 cycles)
                // In SIM_MODE, reduce to 100 cycles
                S_REFILL_DELAY: begin
                    if (delay_cnt >= (SIM_MODE ? 24'd100 : 24'd5_000_000)) begin
                        state <= S_SET_CMD;
                    end else begin
                        delay_cnt <= delay_cnt + 1;
                    end
                end
                
                //==============================================================
                // INIT STEP 2: SET action (write 0x20 to register 0x07)
                //==============================================================
                S_SET_CMD: begin
                    i2c_device_addr <= I2C_ADDR;
                    i2c_reg_addr <= REG_CTRL0;      // 0x07
                    i2c_rw <= 0;                     // Write
                    i2c_num_bytes <= 1;
                    i2c_wr_data <= CMD_SET;         // 0x20
                    i2c_start <= 1;
                    state <= S_SET_WAIT;
                end
                
                S_SET_WAIT: begin
                    if (i2c_done) begin
                        if (i2c_error) begin
                            error <= 1;
                            busy <= 0;
                            state <= S_IDLE;
                        end else begin
                            delay_cnt <= 0;
                            state <= S_SET_DELAY;
                        end
                    end
                end
                
                // Wait 1ms after SET action (1ms * 100MHz = 100,000 cycles)
                // In SIM_MODE, reduce to 100 cycles
                S_SET_DELAY: begin
                    if (delay_cnt >= (SIM_MODE ? 24'd100 : 24'd100_000)) begin
                        initialized <= 1;
                        state <= S_TRIG_MEAS;
                    end else begin
                        delay_cnt <= delay_cnt + 1;
                    end
                end
                
                //==============================================================
                // MEASUREMENT: Trigger measurement (write 0x01 to register 0x07)
                //==============================================================
                S_TRIG_MEAS: begin
                    i2c_device_addr <= I2C_ADDR;
                    i2c_reg_addr <= REG_CTRL0;      // 0x07
                    i2c_rw <= 0;                     // Write
                    i2c_num_bytes <= 1;
                    i2c_wr_data <= CMD_TM_M;        // 0x01
                    i2c_start <= 1;
                    state <= S_TRIG_WAIT;
                end
                
                S_TRIG_WAIT: begin
                    if (i2c_done) begin
                        if (i2c_error) begin
                            error <= 1;
                            busy <= 0;
                            state <= S_IDLE;
                        end else begin
                            delay_cnt <= 0;
                            state <= S_MEAS_DELAY;
                        end
                    end
                end
                
                // Wait 8ms for measurement (8ms * 100MHz = 800,000 cycles)
                // In SIM_MODE, reduce to 100 cycles
                S_MEAS_DELAY: begin
                    if (delay_cnt >= (SIM_MODE ? 24'd100 : 24'd800_000)) begin
                        state <= S_READ_START;
                    end else begin
                        delay_cnt <= delay_cnt + 1;
                    end
                end
                
                //==============================================================
                // READ: Read 6 bytes starting from register 0x00
                //==============================================================
                S_READ_START: begin
                    i2c_device_addr <= I2C_ADDR;
                    i2c_reg_addr <= REG_XOUT_LSB;   // 0x00
                    i2c_rw <= 1;                     // Read
                    i2c_num_bytes <= 3'd6;
                    i2c_start <= 1;
                    state <= S_READ_WAIT;
                end
                
                S_READ_WAIT: begin
                    if (i2c_done) begin
                        if (i2c_error) begin
                            error <= 1;
                            busy <= 0;
                            state <= S_IDLE;
                        end else begin
                            // Assemble 16-bit values from LSB, MSB pairs
                            mag_x <= {data_buf[1], data_buf[0]};  // X_MSB, X_LSB
                            mag_y <= {data_buf[3], data_buf[2]};  // Y_MSB, Y_LSB
                            mag_z <= {data_buf[5], data_buf[4]};  // Z_MSB, Z_LSB
                            data_valid <= 1;
                            busy <= 0;
                            state <= S_IDLE;
                        end
                    end
                end
                
                default: begin
                    state <= S_IDLE;
                    busy <= 0;
                end
            endcase
        end
    end

endmodule
