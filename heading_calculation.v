`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: CECS 361 Final Project
// Engineer: 
// 
// Create Date: 12/01/2025
// Design Name: heading_calculation
// Module Name: heading_calculation
// Project Name: Compass
// Description: Complete heading calculation module that:
//              1. Instantiates tilt_compensation (which includes SPI & I2C drivers)
//              2. Calculates compass heading from tilt-compensated magnetometer data
//              Uses quadrant-aware atan2 approximation
// 
// Inputs: 
//   - clk: 100 MHz system clock
//   - iclk: 4 MHz clock for SPI
//   - reset: Active high reset
//   - SPI pins: miso, sclk, mosi, cs (accelerometer)
//   - I2C pins: sda, scl (magnetometer)
//
// Outputs:
//   - heading: Compass heading in degrees (0-359, where 0=North, 90=East)
//   - pitch, roll: Tilt angles in degrees
//   - data_valid: Pulse indicating new heading is ready
//
//////////////////////////////////////////////////////////////////////////////////

module heading_calculation (
    // Clock and reset
    input wire clk,                        // 100 MHz system clock
    input wire iclk,                       // 4 MHz clock for SPI
    input wire reset,                      // Active high reset
    
    // SPI interface (accelerometer)
    input wire miso,
    output wire sclk,
    output wire mosi,
    output wire cs,
    
    // I2C interface (magnetometer)
    inout wire sda,
    inout wire scl,
    
    // Test inputs (directly inject sensor data for simulation)
    input wire [14:0] acl_data,            // Direct accelerometer data input
    input wire signed [15:0] mag_x,        // Direct magnetometer X input
    input wire signed [15:0] mag_y,        // Direct magnetometer Y input
    input wire signed [15:0] mag_z,        // Direct magnetometer Z input
    
    // Outputs
    output reg [8:0] heading,              // Heading in degrees (0-359)
    output wire signed [8:0] pitch,        // Pitch angle from tilt compensation
    output wire signed [8:0] roll,         // Roll angle from tilt compensation
    output reg data_valid                  // Output data valid pulse
);

    // ========== Tilt Compensation Instance (includes SPI & I2C drivers) ==========
    wire signed [15:0] mag_x_comp;         // Tilt-compensated mag X
    wire signed [15:0] mag_y_comp;         // Tilt-compensated mag Y
    wire tilt_data_valid;                  // Tilt compensation complete
    
    tilt_compensation tilt_inst (
        .clk(clk),
        .iclk(iclk),
        .reset(reset),
        .miso(miso),
        .sclk(sclk),
        .mosi(mosi),
        .cs(cs),
        .sda(sda),
        .scl(scl),
        // Test inputs - pass through from module ports
        .acl_data(acl_data),
        .mag_x(mag_x),
        .mag_y(mag_y),
        .mag_z(mag_z),
        // Outputs
        .mag_x_comp(mag_x_comp),
        .mag_y_comp(mag_y_comp),
        .pitch(pitch),
        .roll(roll),
        .data_valid(tilt_data_valid)
    );

    // ========== Quadrant Detection ==========
    wire x_pos = (mag_x_comp > 0);
    wire x_neg = (mag_x_comp < 0);
    wire y_pos = (mag_y_comp > 0);
    wire y_neg = (mag_y_comp < 0);
    wire x_zero = (mag_x_comp == 0);
    wire y_zero = (mag_y_comp == 0);
    
    // Absolute values for ratio calculation
    wire [15:0] abs_x = x_neg ? -mag_x_comp : mag_x_comp;
    wire [15:0] abs_y = y_neg ? -mag_y_comp : mag_y_comp;
    
    // ========== Atan2 Approximation ==========
    // Returns angle within quadrant (0-90 degrees)
    reg [6:0] base_angle;
    
    // Calculate base angle from ratio |Y|/|X| or |X|/|Y|
    always @(*) begin
        if (abs_x == 0 && abs_y == 0) begin
            base_angle = 7'd0;
        end else if (abs_y <= abs_x) begin
            // |Y|/|X| <= 1, angle is 0-45 degrees
            if (abs_y == 0)
                base_angle = 7'd0;
            else if (abs_y <= (abs_x >>> 3))      // ratio < 0.125 ? ~7°
                base_angle = 7'd7;
            else if (abs_y <= (abs_x >>> 2))      // ratio < 0.25 ? ~14°
                base_angle = 7'd14;
            else if (abs_y <= (abs_x >>> 1))      // ratio < 0.5 ? ~27°
                base_angle = 7'd27;
            else if (abs_y <= abs_x - (abs_x >>> 2))  // ratio < 0.75 ? ~37°
                base_angle = 7'd37;
            else
                base_angle = 7'd45;               // ratio ? 1 ? 45°
        end else begin
            // |Y|/|X| > 1, angle is 45-90 degrees
            if (abs_x == 0)
                base_angle = 7'd90;
            else if (abs_x <= (abs_y >>> 3))      // ratio < 0.125 ? ~83°
                base_angle = 7'd83;
            else if (abs_x <= (abs_y >>> 2))      // ratio < 0.25 ? ~76°
                base_angle = 7'd76;
            else if (abs_x <= (abs_y >>> 1))      // ratio < 0.5 ? ~63°
                base_angle = 7'd63;
            else if (abs_x <= abs_y - (abs_y >>> 2))  // ratio < 0.75 ? ~53°
                base_angle = 7'd53;
            else
                base_angle = 7'd45;               // ratio ? 1 ? 45°
        end
    end
    
    // ========== Heading Calculation ==========
    // Compass heading convention:
    //   0° = North (+X direction)
    //   90° = East (+Y direction)
    //   180° = South (-X direction)
    //   270° = West (-Y direction)
    reg [8:0] heading_calc;
    
    always @(*) begin
        if (x_zero && y_zero) begin
            heading_calc = 9'd0;
        end else if (x_pos && y_zero) begin
            // Pure North
            heading_calc = 9'd0;
        end else if (x_zero && y_pos) begin
            // Pure East
            heading_calc = 9'd90;
        end else if (x_neg && y_zero) begin
            // Pure South
            heading_calc = 9'd180;
        end else if (x_zero && y_neg) begin
            // Pure West
            heading_calc = 9'd270;
        end else if (x_pos && y_pos) begin
            // Quadrant 1: 0-90° (NE)
            heading_calc = {2'b00, base_angle};
        end else if (x_neg && y_pos) begin
            // Quadrant 2: 90-180° (SE)
            heading_calc = 9'd180 - {2'b00, base_angle};
        end else if (x_neg && y_neg) begin
            // Quadrant 3: 180-270° (SW)
            heading_calc = 9'd180 + {2'b00, base_angle};
        end else begin
            // Quadrant 4: 270-360° (NW)
            heading_calc = 9'd360 - {2'b00, base_angle};
        end
    end
    
    // ========== Register Output ==========
    always @(posedge iclk) begin
        data_valid <= 1'b0;
        
        if (tilt_data_valid) begin
            // Normalize heading to 0-359 range
            if (heading_calc >= 9'd360)
                heading <= heading_calc - 9'd360;
            else
                heading <= heading_calc;
            data_valid <= 1'b1;
        end
    end

endmodule
