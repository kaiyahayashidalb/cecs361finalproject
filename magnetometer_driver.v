`timescale 1ns / 1ps

module magnetometer_driver #(
    parameter CLK_HZ = 100_000_000,  // 100 MHz system clock
    parameter TESTBENCH_MODE = 0     // Set to 1 to expose I2C master interface for testing
)(
    input  wire clk,
    input  wire rst,
    
    // Control interface
    input  wire start_read,        // Pulse high to start a magnetometer reading
    output reg  data_valid,        // High for one cycle when new data is ready
    output reg  busy,              // High while operation in progress
    
    // Magnetometer data outputs (signed 16-bit values)
    output reg signed [15:0] mag_x,
    output reg signed [15:0] mag_y,
    output reg signed [15:0] mag_z,
    
    // I2C bus (only used when TESTBENCH_MODE = 0)
    inout  wire sda,
    inout  wire scl,
    
    // I2C master interface (only used when TESTBENCH_MODE = 1)
    output wire [6:0] i2c_dev_addr,
    output wire [7:0] i2c_reg_addr,
    output wire i2c_rw,
    output wire [7:0] i2c_wr_data,
    output wire [7:0] i2c_rd_len,
    output wire i2c_start,
    input  wire i2c_done,
    input  wire i2c_nack,
    input  wire i2c_rd_valid,
    input  wire [7:0] i2c_rd_data
);

    // MMC34160PJ I2C address and registers (from datasheet)
    localparam I2C_ADDR = 7'h30;              // 7-bit I2C address
    
    // Register addresses
    localparam REG_XOUT_0 = 8'h00;            // X-axis output [17:10] (MSB byte)
    localparam REG_XOUT_1 = 8'h01;            // X-axis output [9:2] (LSB byte)
    localparam REG_YOUT_0 = 8'h02;            // Y-axis output [17:10] (MSB byte)
    localparam REG_YOUT_1 = 8'h03;            // Y-axis output [9:2] (LSB byte)
    localparam REG_ZOUT_0 = 8'h04;            // Z-axis output [17:10] (MSB byte)
    localparam REG_ZOUT_1 = 8'h05;            // Z-axis output [9:2] (LSB byte)
    localparam REG_XOUT_2 = 8'h06;            // X-axis bits [1:0], Y-axis bits [1:0] in upper nibble
    localparam REG_YOUT_2 = 8'h07;            // Y-axis bits [1:0], Z-axis bits [1:0] in upper nibble
    localparam REG_ZOUT_2 = 8'h08;            // Z-axis bits [1:0]
    localparam REG_STATUS = 8'h18;            // Status register
    localparam REG_CONTROL_0 = 8'h1B;         // Internal Control 0
    localparam REG_CONTROL_1 = 8'h1C;         // Internal Control 1
    localparam REG_CONTROL_2 = 8'h1D;         // Internal Control 2
    localparam REG_PRODUCT_ID = 8'h2F;        // Product ID (should read 0x10)
    
    // Control register 0 bit definitions
    localparam CTRL0_TAKE_MEAS_M = 8'h01;     // TM_M: Take measurement
    localparam CTRL0_TAKE_MEAS_T = 8'h02;     // TM_T: Take temperature measurement
    localparam CTRL0_START_MDT = 8'h04;       // Start motion detection
    localparam CTRL0_DO_SET = 8'h08;          // Do SET operation
    localparam CTRL0_DO_RESET = 8'h10;        // Do RESET operation
    localparam CTRL0_AUTO_SR_EN = 8'h20;      // Enable automatic set/reset
    
    // Control register 1 bit definitions
    localparam CTRL1_BW_100HZ = 8'h00;        // Bandwidth 100Hz
    localparam CTRL1_BW_200HZ = 8'h01;        // Bandwidth 200Hz
    localparam CTRL1_BW_400HZ = 8'h02;        // Bandwidth 400Hz
    localparam CTRL1_BW_800HZ = 8'h03;        // Bandwidth 800Hz
    
    // Control register 2 bit definitions
    localparam CTRL2_CMM_EN = 8'h10;          // Enable continuous measurement mode
    localparam CTRL2_EN_PRD_SET = 8'h08;      // Enable periodic SET
    localparam CTRL2_CMM_FREQ_10HZ = 8'h00;   // 10Hz continuous mode
    localparam CTRL2_CMM_FREQ_50HZ = 8'h01;   // 50Hz continuous mode
    localparam CTRL2_CMM_FREQ_100HZ = 8'h02;  // 100Hz continuous mode
    localparam CTRL2_CMM_FREQ_200HZ = 8'h03;  // 200Hz continuous mode
    
    // I2C master signals (internal or exposed based on TESTBENCH_MODE)
    reg  i2c_start_int;
    reg  [6:0] i2c_dev_addr_int;
    reg  [7:0] i2c_reg_addr_int;
    reg  i2c_rw_int;
    reg  [7:0] i2c_wr_data_int;
    reg  [7:0] i2c_rd_len_int;
    wire i2c_rd_valid_int;
    wire [7:0] i2c_rd_data_int;
    reg  i2c_rd_ready;
    wire i2c_busy;
    wire i2c_done_int;
    wire i2c_nack_int;
    
    // Conditional connection based on mode
    generate
        if (TESTBENCH_MODE == 1) begin
            // Testbench mode: expose internal signals as outputs/inputs
            assign i2c_dev_addr = i2c_dev_addr_int;
            assign i2c_reg_addr = i2c_reg_addr_int;
            assign i2c_rw = i2c_rw_int;
            assign i2c_wr_data = i2c_wr_data_int;
            assign i2c_rd_len = i2c_rd_len_int;
            assign i2c_start = i2c_start_int;
            assign i2c_done_int = i2c_done;
            assign i2c_nack_int = i2c_nack;
            assign i2c_rd_valid_int = i2c_rd_valid;
            assign i2c_rd_data_int = i2c_rd_data;
        end else begin
            // Normal mode: instantiate I2C master
            i2c_master #(
                .CLK_HZ(CLK_HZ),
                .I2C_HZ(400_000)  // 400 kHz fast mode
            ) i2c_inst (
                .clk(clk),
                .rst(rst),
                .start(i2c_start_int),
                .dev_addr(i2c_dev_addr_int),
                .reg_addr(i2c_reg_addr_int),
                .rw(i2c_rw_int),
                .wr_data(i2c_wr_data_int),
                .rd_len(i2c_rd_len_int),
                .rd_valid(i2c_rd_valid_int),
                .rd_data(i2c_rd_data_int),
                .rd_ready(i2c_rd_ready),
                .busy(i2c_busy),
                .done(i2c_done_int),
                .nack_seen(i2c_nack_int),
                .sda(sda),
                .scl(scl)
            );
        end
    endgenerate;
    
    // FSM states
    localparam 
        IDLE           = 4'd0,
        INIT_START     = 4'd1,
        INIT_WAIT      = 4'd2,
        READ_START     = 4'd3,
        READ_WAIT      = 4'd4,
        READ_COLLECT   = 4'd5,
        DONE           = 4'd6;
    
    reg [3:0] state;
    
    // Data collection buffer (6 bytes: X_LSB, X_MSB, Y_LSB, Y_MSB, Z_LSB, Z_MSB)
    reg [7:0] data_buf[0:5];
    reg [2:0] byte_count;
    
    // Initialization flag
    reg initialized;
    
    // Main FSM
    always @(posedge clk) begin
        if (rst) begin
            state <= IDLE;
            busy <= 0;
            data_valid <= 0;
            initialized <= 0;
            i2c_start_int <= 0;
            i2c_rd_ready <= 1;
            byte_count <= 0;
            mag_x <= 0;
            mag_y <= 0;
            mag_z <= 0;
        end else begin
            // Default values
            i2c_start_int <= 0;
            data_valid <= 0;
            
            case (state)
                IDLE: begin
                    busy <= 0;
                    if (start_read) begin
                        busy <= 1;
                        byte_count <= 0;
                        
                        if (!initialized) begin
                            // Need to initialize sensor first
                            state <= INIT_START;
                        end else begin
                            // Already initialized, just read data
                            state <= READ_START;
                        end
                    end
                end
                
                // ========== INITIALIZATION ==========
                INIT_START: begin
                    // Write to Control Register 2 to enable continuous measurement mode
                    // CMM_EN + 100Hz frequency
                    i2c_dev_addr_int <= I2C_ADDR;
                    i2c_reg_addr_int <= REG_CONTROL_2;
                    i2c_rw_int <= 0;  // Write
                    i2c_wr_data_int <= CTRL2_CMM_EN | CTRL2_CMM_FREQ_100HZ;  // Continuous mode at 100Hz
                    i2c_start_int <= 1;
                    state <= INIT_WAIT;
                end
                
                INIT_WAIT: begin
                    if (i2c_done_int) begin
                        if (i2c_nack_int) begin
                            // NACK received - initialization failed
                            // Could add error handling here
                            state <= IDLE;
                            busy <= 0;
                        end else begin
                            // Initialization successful
                            initialized <= 1;
                            state <= READ_START;
                        end
                    end
                end
                
                // ========== READ DATA ==========
                READ_START: begin
                    // Read 6 bytes starting from XOUT_0 register (0x00)
                    // Order: XOUT_0 (MSB), XOUT_1 (LSB), YOUT_0 (MSB), YOUT_1 (LSB), ZOUT_0 (MSB), ZOUT_1 (LSB)
                    // Note: Each axis has 18 bits total, but we only read the 16 MSB bits (registers 0 and 1)
                    i2c_dev_addr_int <= I2C_ADDR;
                    i2c_reg_addr_int <= REG_XOUT_0;  // Start at 0x00
                    i2c_rw_int <= 1;  // Read
                    i2c_rd_len_int <= 8'd6;  // Read 6 bytes
                    i2c_start_int <= 1;
                    byte_count <= 0;
                    state <= READ_WAIT;
                end
                
                READ_WAIT: begin
                    if (i2c_rd_valid_int) begin
                        // Store received byte
                        data_buf[byte_count] <= i2c_rd_data_int;
                        byte_count <= byte_count + 1;
                        
                        if (byte_count == 5) begin
                            // All 6 bytes received, move to processing
                            state <= READ_COLLECT;
                        end
                    end else if (i2c_done_int) begin
                        // Transaction complete
                        if (i2c_nack_int) begin
                            // NACK received - read failed
                            state <= IDLE;
                            busy <= 0;
                        end else if (byte_count == 6) begin
                            // All data collected successfully
                            state <= READ_COLLECT;
                        end
                    end
                end
                
                READ_COLLECT: begin
                    // Combine bytes into 16-bit signed values
                    // MMC34160PJ outputs data as MSB first, then LSB
                    // Reading: XOUT_0[17:10], XOUT_1[9:2], YOUT_0[17:10], YOUT_1[9:2], ZOUT_0[17:10], ZOUT_1[9:2]
                    // Note: This gives us the top 16 bits of the 18-bit value
                    
                    mag_x <= {data_buf[0], data_buf[1]};  // MSB first: XOUT_0, XOUT_1
                    mag_y <= {data_buf[2], data_buf[3]};  // YOUT_0, YOUT_1
                    mag_z <= {data_buf[4], data_buf[5]};  // ZOUT_0, ZOUT_1
                    
                    data_valid <= 1;
                    state <= DONE;
                end
                
                DONE: begin
                    busy <= 0;
                    state <= IDLE;
                end
                
                default: state <= IDLE;
            endcase
        end
    end

endmodule
