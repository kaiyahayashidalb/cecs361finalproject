`timescale 1ns / 1ps

module magnetometer_driver #(
    parameter CLK_HZ = 100_000_000   // 100 MHz system clock
)(
    input  wire clk,
    input  wire rst,
    
    // Control interface
    input  wire start_read,          // Pulse high to start a magnetometer reading
    input  wire start_calibrate,     // Pulse high to start calibration sequence
    output reg  data_valid,          // High for one cycle when new data is ready
    output reg  calibration_done,    // High for one cycle when calibration complete
    output reg  busy,                // High while operation in progress
    output reg  error,               // High if NACK received
    
    // Magnetometer data outputs (16-bit signed, calibrated)
    output reg signed [15:0] mag_x,
    output reg signed [15:0] mag_y,
    output reg signed [15:0] mag_z,
    
    // Calibration offset outputs (can be saved for future use)
    output reg signed [15:0] offset_x,
    output reg signed [15:0] offset_y,
    output reg signed [15:0] offset_z,
    
    // Debug outputs
    output wire [3:0] dbg_i2c_state, // I2C master state
    output wire [4:0] dbg_mag_state, // Magnetometer driver state
    
    // I2C bus
    inout  wire sda,
    inout  wire scl
);

    //==========================================================================
    // MMC34160PJ Constants - FIXED PER DATASHEET EXAMPLES
    //==========================================================================
    localparam [6:0] I2C_ADDR = 7'h30;    // 7-bit I2C address (binary 0110000)
    
    // Register addresses - Following datasheet Quick Start pseudocode (page 6)
    localparam [7:0] REG_XOUT_LSB = 8'h00;  // X axis output LSB
    localparam [7:0] REG_CTRL0    = 8'h07;  // Internal Control 0 (per datasheet pg 6)
    localparam [7:0] REG_STATUS   = 8'h07;  // Status register (same as CTRL0? datasheet unclear)
    localparam [7:0] REG_CTRL1    = 8'h09;  // Internal Control 1
    
    // Control register 0 bits
    localparam [7:0] CTRL0_REFILL = 8'h80;  // Refill capacitor
    localparam [7:0] CTRL0_RESET  = 8'h40;  // Reset sensor
    localparam [7:0] CTRL0_SET    = 8'h20;  // Set sensor
    localparam [7:0] CTRL0_TM_M   = 8'h01;  // Take Measurement
    
    // Control register 1 bits
    localparam [7:0] CTRL1_SW_RST = 8'h80;  // Software reset

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
    wire [3:0]  i2c_dbg_state;
    
    i2c_master #(
        .CLK_HZ(CLK_HZ),
        .I2C_HZ(100_000)    // 100kHz for reliability
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
        .debug_state(i2c_dbg_state),
        .sda(sda),
        .scl(scl)
    );
    
    // Debug outputs
    assign dbg_i2c_state = i2c_dbg_state;
    assign dbg_mag_state = state;

    //==========================================================================
    // FSM States - SIMPLIFIED (No Status Check)
    //==========================================================================
    localparam [4:0]
        S_IDLE           = 5'd0,
        S_TRIG_START     = 5'd1,   // Start trigger measurement write
        S_TRIG_WAIT      = 5'd2,   // Wait for trigger to complete
        S_MEAS_DELAY     = 5'd3,   // Wait for measurement to complete (10ms)
        S_READ_START     = 5'd4,   // Start 6-byte read
        S_READ_WAIT      = 5'd5,   // Wait for read to complete
        S_DONE           = 5'd6;   // Process and output data
    
    reg [4:0] state;
    reg [2:0] byte_cnt;
    reg [7:0] byte0, byte1, byte2, byte3, byte4, byte5;
    reg [25:0] delay_cnt;       // Counter for delays
    
    // Calibration not implemented in this simplified version
    reg calibrated;
    
    // MMC34160PJ zero-field offset (sensor outputs 32768 at zero field)
    localparam [15:0] SENSOR_MIDPOINT = 16'd32768;

    //==========================================================================
    // Capture incoming read bytes
    //==========================================================================
    always @(posedge clk) begin
        if (rst) begin
            byte_cnt <= 0;
            byte0 <= 0; byte1 <= 0; byte2 <= 0;
            byte3 <= 0; byte4 <= 0; byte5 <= 0;
        end else if (state == S_READ_START) begin
            byte_cnt <= 0;
        end else if (i2c_rd_valid && state == S_READ_WAIT) begin
            case (byte_cnt)
                3'd0: byte0 <= i2c_rd_data;
                3'd1: byte1 <= i2c_rd_data;
                3'd2: byte2 <= i2c_rd_data;
                3'd3: byte3 <= i2c_rd_data;
                3'd4: byte4 <= i2c_rd_data;
                3'd5: byte5 <= i2c_rd_data;
                default: ;
            endcase
            byte_cnt <= byte_cnt + 1;
        end
    end

    //==========================================================================
    // Main FSM - SIMPLIFIED
    //==========================================================================
    always @(posedge clk) begin
        if (rst) begin
            state <= S_IDLE;
            busy <= 0;
            data_valid <= 0;
            calibration_done <= 0;
            error <= 0;
            mag_x <= 0;
            mag_y <= 0;
            mag_z <= 0;
            offset_x <= 0;
            offset_y <= 0;
            offset_z <= 0;
            calibrated <= 0;
            i2c_start <= 0;
            i2c_device_addr <= I2C_ADDR;
            i2c_reg_addr <= 0;
            i2c_rw <= 0;
            i2c_num_bytes <= 1;
            i2c_wr_data <= 0;
            delay_cnt <= 0;
        end else begin
            // Default: clear pulses
            data_valid <= 0;
            calibration_done <= 0;
            i2c_start <= 0;
            
            case (state)
                //--------------------------------------------------------------
                S_IDLE: begin
                    busy <= 0;
                    if (start_read) begin
                        busy <= 1;
                        error <= 0;  // Clear error on new read
                        state <= S_TRIG_START;
                    end
                end
                
                //==============================================================
                // SIMPLIFIED MEASUREMENT SEQUENCE - Per Datasheet Quick Start
                //==============================================================
                S_TRIG_START: begin
                    i2c_device_addr <= I2C_ADDR;
                    i2c_reg_addr <= REG_CTRL0;  // Now 0x07 per datasheet
                    i2c_rw <= 0;  // Write
                    i2c_num_bytes <= 1;
                    i2c_wr_data <= CTRL0_TM_M;  // 0x01 to trigger measurement
                    i2c_start <= 1;
                    state <= S_TRIG_WAIT;
                end
                
                S_TRIG_WAIT: begin
                    if (i2c_done) begin
                        if (i2c_error) begin
                            error <= 1;
                            state <= S_IDLE;
                        end else begin
                            delay_cnt <= 0;
                            state <= S_MEAS_DELAY;
                        end
                    end
                end
                
                S_MEAS_DELAY: begin
                    // Wait 10ms for measurement to complete (1,000,000 cycles at 100MHz)
                    // Datasheet says minimum 7.92ms for 16-bit resolution
                    if (delay_cnt >= 26'd1_000_000) begin
                        state <= S_READ_START;
                    end else begin
                        delay_cnt <= delay_cnt + 1;
                    end
                end
                
                S_READ_START: begin
                    i2c_device_addr <= I2C_ADDR;
                    i2c_reg_addr <= REG_XOUT_LSB;  // 0x00
                    i2c_rw <= 1;  // Read
                    i2c_num_bytes <= 3'd6;  // Read all 6 bytes (X,Y,Z)
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
                
                S_DONE: begin
                    // Assemble raw 16-bit unsigned values and convert to signed
                    // MMC34160PJ outputs unsigned values: 0=max negative, 32768=zero, 65535=max positive
                    // Subtract sensor midpoint (32768) to get signed value
                    mag_x <= $signed({1'b0, byte1, byte0}) - $signed(17'd32768);
                    mag_y <= $signed({1'b0, byte3, byte2}) - $signed(17'd32768);
                    mag_z <= $signed({1'b0, byte5, byte4}) - $signed(17'd32768);
                    data_valid <= 1;
                    state <= S_IDLE;
                end
                
                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
