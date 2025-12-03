`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// CECS 361 Final Project - Digital Compass
// Engineer: Nathan Sarkozy
// 
// Module: tilt_compensation (SIMPLIFIED - No Tilt Compensation)
// Description: Sensor driver wrapper module.
//              - Reads accelerometer via SPI (ADXL362) - for future use
//              - Reads magnetometer via I2C (MMC34160PJ)
//              - Outputs raw magnetometer X/Y for heading calculation
//              - Tilt compensation disabled for stability
// 
// Dependencies: SPI_master.v, magnetometer_driver.v
//////////////////////////////////////////////////////////////////////////////////

module tilt_compensation (
    // Clocks and reset
    input wire clk,                       // 100 MHz system clock
    input wire iclk,                      // 4 MHz clock for SPI
    input wire reset,                     // Active high reset
    
    // SPI interface (accelerometer)
    input wire miso,
    output wire sclk,
    output wire mosi,
    output wire cs,
    
    // I2C interface (magnetometer)
    inout wire sda,
    inout wire scl,
    
    // Test inputs (directly inject sensor data for simulation)
    input wire [14:0] acl_data,           // Direct accelerometer data input (unused)
    input wire signed [15:0] mag_x,       // Direct magnetometer X input
    input wire signed [15:0] mag_y,       // Direct magnetometer Y input
    input wire signed [15:0] mag_z,       // Direct magnetometer Z input
    
    // Outputs
    output wire signed [15:0] mag_x_comp, // Outputs raw magnetometer X (no tilt compensation)
    output wire signed [15:0] mag_y_comp, // Outputs raw magnetometer Y (no tilt compensation)
    output wire signed [15:0] mag_x_raw,  // Raw magnetometer X 
    output wire signed [15:0] mag_y_raw,  // Raw magnetometer Y
    output wire signed [8:0] pitch,       // Not computed (always 0)
    output wire signed [8:0] roll,        // Not computed (always 0)
    output reg data_valid,                // New data ready flag
    
    // Debug outputs
    output wire mag_error_out,            // I2C error flag
    output wire mag_busy_out,             // I2C busy flag
    output wire [7:0] mag_x_debug         // I2C debug byte
);

    // ========== Debug Signal Routing ==========
    assign mag_error_out = mag_error;
    assign mag_busy_out = mag_busy;
    assign mag_x_debug = i2c_debug_byte;
    
    // ========== Raw Magnetometer Outputs ==========
    // Route raw signed magnetometer data directly to outputs
    assign mag_x_raw = mag_x_signed;
    assign mag_y_raw = mag_y_signed;
    assign mag_x_comp = mag_x_signed;  // Same as raw (no tilt compensation)
    assign mag_y_comp = mag_y_signed;  // Same as raw (no tilt compensation)
    
    // Pitch/Roll not computed (tilt compensation disabled)
    assign pitch = 9'sd0;
    assign roll = 9'sd0;

    // ========== SPI Accelerometer (ADXL362) ==========
    // Kept for potential future use, but data not used for heading calculation
    wire [14:0] acl_data_spi;
    
    SPI_master spi_accel (
        .iclk(iclk),
        .miso(miso),
        .sclk(sclk),
        .mosi(mosi),
        .cs(cs),
        .acl_data(acl_data_spi)
    );

    // ========== I2C Magnetometer (MMC34160PJ) ==========
    wire signed [15:0] mag_x_i2c, mag_y_i2c, mag_z_i2c;
    wire mag_busy;
    wire mag_error;
    wire [7:0] i2c_debug_byte;
    reg start_mag_read;
    reg [19:0] mag_timer;
    
    magnetometer_driver #(
        .CLK_HZ(100_000_000)
    ) mag_inst (
        .clk(clk),
        .rst(reset),
        .start_read(start_mag_read),
        .data_valid(),  // Not used - we poll periodically
        .busy(mag_busy),
        .error(mag_error),
        .mag_x(mag_x_i2c),
        .mag_y(mag_y_i2c),
        .mag_z(mag_z_i2c),
        .debug_byte(i2c_debug_byte),
        .sda(sda),
        .scl(scl)
    );
    
    // ========== Sensor Data Selection ==========
    // Use direct inputs if provided (non-zero), otherwise use hardware I2C driver
    wire use_test_data = (mag_x != 0 || mag_y != 0 || mag_z != 0);
    wire signed [15:0] mag_x_sel_raw = use_test_data ? mag_x : mag_x_i2c;
    wire signed [15:0] mag_y_sel_raw = use_test_data ? mag_y : mag_y_i2c;
    
    // ========== Magnetometer Unsigned to Signed Conversion ==========
    // The MMC34160PJ outputs UNSIGNED 16-bit values where:
    //   0 = -8 Gauss, 32768 = 0 Gauss, 65535 = +8 Gauss
    // Convert to signed by flipping the MSB (offset binary to two's complement)
    wire signed [15:0] mag_x_signed = {~mag_x_sel_raw[15], mag_x_sel_raw[14:0]};
    wire signed [15:0] mag_y_signed = {~mag_y_sel_raw[15], mag_y_sel_raw[14:0]};
    
    // ========== Magnetometer Polling Timer ==========
    // Trigger magnetometer read every ~10ms (1,000,000 cycles at 100MHz)
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            mag_timer <= 0;
            start_mag_read <= 0;
        end else begin
            start_mag_read <= 0;
            if (mag_timer >= 20'd1_000_000) begin
                mag_timer <= 0;
                if (!mag_busy)
                    start_mag_read <= 1;
            end else begin
                mag_timer <= mag_timer + 1;
            end
        end
    end

    // ========== Data Valid Generation ==========
    // Generate data_valid pulse every ~10ms (40000 cycles at 4MHz)
    reg [15:0] valid_counter;
    
    always @(posedge iclk or posedge reset) begin
        if (reset) begin
            valid_counter <= 0;
            data_valid <= 0;
        end else begin
            data_valid <= 0;
            valid_counter <= valid_counter + 1;
            
            // Pulse data_valid every ~10ms
            if (valid_counter >= 16'd40000) begin
                valid_counter <= 0;
                data_valid <= 1;
            end
        end
    end

endmodule
