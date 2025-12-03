`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// CECS 361 Final Project - Digital Compass
// 
// Module: heading_calculation
// Description: Compass heading calculation module that:
//              1. Instantiates tilt_compensation (sensor driver wrapper)
//              2. Calculates compass heading from raw magnetometer X/Y data
//              3. Uses quadrant-aware atan2 approximation for heading
//              4. Applies 138° calibration offset to align with true North
//              5. Updates display every 64 samples (~640ms) for stability
// 
// Inputs: 
//   - clk: 100 MHz system clock
//   - iclk: 4 MHz clock for SPI
//   - reset: Active high reset
//   - SPI pins: miso, sclk, mosi, cs (accelerometer - hardware present, not used)
//   - I2C pins: sda, scl (magnetometer - Pmod CMPS2)
//
// Outputs:
//   - heading: Compass heading in degrees (0-359, where 0=North, 90=East)
//   - pitch, roll: Always 0 (tilt compensation disabled for stability)
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
    output reg data_valid,                 // Output data valid pulse
    
    // Debug outputs
    output wire mag_error,                 // I2C error flag
    output wire mag_busy,                  // I2C busy flag
    output wire [7:0] mag_x_debug          // Raw mag X for debug
);

    // ========== Sensor Driver Instance (includes SPI & I2C drivers) ==========
    wire signed [15:0] mag_x_comp;         // Raw mag X (tilt comp disabled)
    wire signed [15:0] mag_y_comp;         // Raw mag Y (tilt comp disabled)
    wire tilt_data_valid;                  // Data ready flag
    
    // Raw magnetometer data (same as comp when tilt compensation disabled)
    wire signed [15:0] mag_x_raw;
    wire signed [15:0] mag_y_raw;
    
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
        .mag_x_raw(mag_x_raw),      // Get raw magnetometer data
        .mag_y_raw(mag_y_raw),      // Get raw magnetometer data
        .pitch(pitch),
        .roll(roll),
        .data_valid(tilt_data_valid),
        .mag_error_out(mag_error),
        .mag_busy_out(mag_busy),
        .mag_x_debug(mag_x_debug)
    );

    // ========== Quadrant Detection ==========
    // Using raw magnetometer data for stable heading when board is held level.
    // Tilt compensation was disabled due to accelerometer noise causing
    // heading drift even when stationary.
    
    // Use raw magnetometer data for heading calculation
    wire signed [15:0] mag_x_use = mag_x_raw;
    wire signed [15:0] mag_y_use = mag_y_raw;
    
    // Deadband to reduce noise near zero
    localparam signed [15:0] DEADBAND = 16'sd50;
    
    wire x_pos = (mag_x_use > DEADBAND);
    wire x_neg = (mag_x_use < -DEADBAND);
    wire y_pos = (mag_y_use > DEADBAND);
    wire y_neg = (mag_y_use < -DEADBAND);
    wire x_zero = !x_pos && !x_neg;
    wire y_zero = !y_pos && !y_neg;
    
    // Absolute values for ratio calculation
    wire [15:0] abs_x = mag_x_use[15] ? (-mag_x_use) : mag_x_use;
    wire [15:0] abs_y = mag_y_use[15] ? (-mag_y_use) : mag_y_use;
    
    // ========== Atan2 Approximation ==========
    // Returns angle within quadrant (0-90 degrees)
    // Using more comparison points for better accuracy
    reg [6:0] base_angle;
    
    // Precompute shifted values to avoid repeated shifting
    wire [15:0] abs_x_div2 = abs_x >> 1;    // abs_x * 0.5
    wire [15:0] abs_x_div4 = abs_x >> 2;    // abs_x * 0.25
    wire [15:0] abs_x_div8 = abs_x >> 3;    // abs_x * 0.125
    wire [15:0] abs_y_div2 = abs_y >> 1;    // abs_y * 0.5
    wire [15:0] abs_y_div4 = abs_y >> 2;    // abs_y * 0.25
    wire [15:0] abs_y_div8 = abs_y >> 3;    // abs_y * 0.125
    
    // Calculate base angle from ratio |Y|/|X| or |X|/|Y|
    // atan(0.125) ≈ 7°, atan(0.25) ≈ 14°, atan(0.5) ≈ 27°, atan(0.75) ≈ 37°, atan(1) = 45°
    always @(*) begin
        if (abs_x == 0 && abs_y == 0) begin
            base_angle = 7'd0;
        end else if (abs_x >= abs_y) begin
            // |Y|/|X| <= 1, angle is 0-45 degrees
            // Compare Y against fractions of X
            if (abs_y == 0)
                base_angle = 7'd0;
            else if (abs_y < abs_x_div8)           // Y/X < 0.125 → ~4°
                base_angle = 7'd4;
            else if (abs_y < abs_x_div4)           // Y/X < 0.25 → ~11°
                base_angle = 7'd11;
            else if (abs_y < abs_x_div2)           // Y/X < 0.5 → ~22°
                base_angle = 7'd22;
            else if (abs_y < abs_x - abs_x_div4)   // Y/X < 0.75 → ~34°
                base_angle = 7'd34;
            else if (abs_y < abs_x)                // Y/X < 1.0 → ~42°
                base_angle = 7'd42;
            else
                base_angle = 7'd45;                // Y/X = 1.0 → 45°
        end else begin
            // |Y|/|X| > 1, angle is 45-90 degrees
            // Compare X against fractions of Y
            if (abs_x == 0)
                base_angle = 7'd90;
            else if (abs_x < abs_y_div8)           // X/Y < 0.125 → ~86°
                base_angle = 7'd86;
            else if (abs_x < abs_y_div4)           // X/Y < 0.25 → ~79°
                base_angle = 7'd79;
            else if (abs_x < abs_y_div2)           // X/Y < 0.5 → ~68°
                base_angle = 7'd68;
            else if (abs_x < abs_y - abs_y_div4)   // X/Y < 0.75 → ~56°
                base_angle = 7'd56;
            else if (abs_x < abs_y)                // X/Y < 1.0 → ~48°
                base_angle = 7'd48;
            else
                base_angle = 7'd45;                // X/Y = 1.0 → 45°
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
            // Quadrant 1: 0-90� (NE)
            heading_calc = {2'b00, base_angle};
        end else if (x_neg && y_pos) begin
            // Quadrant 2: 90-180� (SE)
            heading_calc = 9'd180 - {2'b00, base_angle};
        end else if (x_neg && y_neg) begin
            // Quadrant 3: 180-270� (SW)
            heading_calc = 9'd180 + {2'b00, base_angle};
        end else begin
            // Quadrant 4: 270-360� (NW)
            heading_calc = 9'd360 - {2'b00, base_angle};
        end
    end
    
    // ========== Register Output with Calibration Offset ==========
    // Calibration: Board reads 222° when pointing North, so add 138° offset
    // (360° - 222° = 138°) to align with true North
    localparam [8:0] HEADING_OFFSET = 9'd138;
    wire [9:0] adjusted_heading = {1'b0, heading_calc} + {1'b0, HEADING_OFFSET};
    wire [8:0] normalized_heading = (adjusted_heading >= 10'd360) ? 
                                    (adjusted_heading[8:0] - 9'd360) : adjusted_heading[8:0];
    
    // Update display only every N samples to reduce flicker
    reg [6:0] update_count;
    localparam [6:0] UPDATE_INTERVAL = 7'd64;  // Update every 64 samples (~640ms)
    
    always @(posedge iclk or posedge reset) begin
        if (reset) begin
            heading <= 9'd0;
            update_count <= 0;
            data_valid <= 1'b0;
        end else begin
            data_valid <= 1'b0;
            
            if (tilt_data_valid) begin
                update_count <= update_count + 1;
                
                // Only update display periodically
                if (update_count >= UPDATE_INTERVAL) begin
                    update_count <= 0;
                    heading <= normalized_heading;
                end
                
                data_valid <= 1'b1;
            end
        end
    end

endmodule
