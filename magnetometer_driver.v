`timescale 1ns / 1ps

module magnetometer_driver #(
    parameter CLK_HZ = 100_000_000   // 100 MHz system clock
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
    
    // I2C bus
    inout  wire sda,
    inout  wire scl
);

    //==========================================================================
    // MMC34160PJ Constants
    //==========================================================================
    localparam [6:0] I2C_ADDR = 7'h30;    // 7-bit I2C address
    
    // Register addresses
    localparam [7:0] REG_XOUT_LSB = 8'h00;  // X axis output LSB
    localparam [7:0] REG_CTRL0    = 8'h08;  // Internal Control 0
    localparam [7:0] REG_CTRL1    = 8'h09;  // Internal Control 1
    
    // Control register bits
    localparam [7:0] CTRL0_TM_M   = 8'h01;  // Take Measurement (magnetic)
    localparam [7:0] CTRL1_SW_RST = 8'h80;  // Software reset
    localparam [7:0] CTRL1_CM_FREQ_1HZ = 8'h01;  // Continuous mode 1Hz

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
        .I2C_HZ(400_000)
    ) i2c (
        .clk(clk),
        .rst(rst),
        .start(i2c_start),
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
        .sda(sda),
        .scl(scl)
    );

    //==========================================================================
    // FSM States
    //==========================================================================
    localparam [2:0]
        S_IDLE        = 3'd0,
        S_INIT_START  = 3'd1,   // Start initialization write
        S_INIT_WAIT   = 3'd2,   // Wait for init to complete
        S_TRIG_START  = 3'd3,   // Start trigger measurement write
        S_TRIG_WAIT   = 3'd4,   // Wait for trigger to complete
        S_READ_START  = 3'd5,   // Start 6-byte read
        S_READ_WAIT   = 3'd6,   // Wait for read to complete
        S_DONE        = 3'd7;
    
    reg [2:0] state;
    reg [2:0] byte_cnt;
    reg [7:0] data_buf [0:5];   // 6 bytes: X_LSB, X_MSB, Y_LSB, Y_MSB, Z_LSB, Z_MSB
    reg       initialized;

    //==========================================================================
    // Capture incoming read bytes
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
    // Main FSM
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
        end else begin
            // Default: clear pulses
            data_valid <= 0;
            i2c_start <= 0;
            
            case (state)
                //--------------------------------------------------------------
                S_IDLE: begin
                    if (start_read) begin
                        busy <= 1;
                        error <= 0;
                        if (initialized) begin
                            state <= S_TRIG_START;
                        end else begin
                            state <= S_INIT_START;
                        end
                    end else begin
                        busy <= 0;
                    end
                end
                
                //--------------------------------------------------------------
                // INIT: Write to CTRL1 to enable continuous mode
                //--------------------------------------------------------------
                S_INIT_START: begin
                    i2c_device_addr <= I2C_ADDR;
                    i2c_reg_addr <= REG_CTRL1;
                    i2c_rw <= 0;  // Write
                    i2c_num_bytes <= 1;
                    i2c_wr_data <= CTRL1_CM_FREQ_1HZ;
                    i2c_start <= 1;
                    state <= S_INIT_WAIT;
                end
                
                S_INIT_WAIT: begin
                    if (i2c_done) begin
                        if (i2c_error) begin
                            error <= 1;
                            state <= S_IDLE;
                        end else begin
                            initialized <= 1;
                            state <= S_TRIG_START;
                        end
                    end
                end
                
                //--------------------------------------------------------------
                // TRIGGER: Write TM_M bit to CTRL0 to take measurement
                //--------------------------------------------------------------
                S_TRIG_START: begin
                    i2c_device_addr <= I2C_ADDR;
                    i2c_reg_addr <= REG_CTRL0;
                    i2c_rw <= 0;  // Write
                    i2c_num_bytes <= 1;
                    i2c_wr_data <= CTRL0_TM_M;
                    i2c_start <= 1;
                    state <= S_TRIG_WAIT;
                end
                
                S_TRIG_WAIT: begin
                    if (i2c_done) begin
                        if (i2c_error) begin
                            error <= 1;
                            state <= S_IDLE;
                        end else begin
                            state <= S_READ_START;
                        end
                    end
                end
                
                //--------------------------------------------------------------
                // READ: Read 6 bytes from REG_XOUT_LSB
                //--------------------------------------------------------------
                S_READ_START: begin
                    i2c_device_addr <= I2C_ADDR;
                    i2c_reg_addr <= REG_XOUT_LSB;
                    i2c_rw <= 1;  // Read
                    i2c_num_bytes <= 3'd6;
                    i2c_start <= 1;
                    state <= S_READ_WAIT;
                end
                
                S_READ_WAIT: begin
                    if (i2c_done) begin
                        if (i2c_error) begin
                            error <= 1;
                            state <= S_IDLE;
                        end else begin
                            state <= S_DONE;
                        end
                    end
                end
                
                //--------------------------------------------------------------
                // DONE: Assemble 16-bit values and signal completion
                //--------------------------------------------------------------
                S_DONE: begin
                    // MMC34160PJ outputs LSB first: [LSB, MSB]
                    mag_x <= {data_buf[1], data_buf[0]};  // X_MSB, X_LSB
                    mag_y <= {data_buf[3], data_buf[2]};  // Y_MSB, Y_LSB
                    mag_z <= {data_buf[5], data_buf[4]};  // Z_MSB, Z_LSB
                    data_valid <= 1;
                    state <= S_IDLE;
                end
                
                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
