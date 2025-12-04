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
    output wire [7:0] mag_x_debug,         // Raw mag X for debug
    output wire signed [15:0] mag_x_raw,   // NEW: Full raw X value
    output wire signed [15:0] mag_y_raw    // NEW: Full raw Y value
);

    // ========== Sensor Driver Instance (includes SPI & I2C drivers) ==========
    wire signed [15:0] mag_x_comp;         // Raw mag X (tilt comp disabled)
    wire signed [15:0] mag_y_comp;         // Raw mag Y (tilt comp disabled)
    wire tilt_data_valid;                  // Data ready flag
    
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

    // ========== 8-DIRECTION COMPASS CALCULATION ==========
    // Maps magnetometer readings to 8 cardinal/intercardinal directions
    // N=0°, NE=45°, E=90°, SE=135°, S=180°, SW=225°, W=270°, NW=315°
    
    // AXIS CORRECTION: Your sensor is rotated 225° (or -135°) from standard
    // We need to rotate the coordinate system BEFORE calculating sectors
    // Rotation by -135° (or +225°):
    //   X_new = X*cos(-135°) - Y*sin(-135°) = -0.707*X + 0.707*Y = (Y-X)/sqrt(2)
    //   Y_new = X*sin(-135°) + Y*cos(-135°) = -0.707*X - 0.707*Y = -(X+Y)/sqrt(2)
    //
    // Simplified (ignoring sqrt(2) scaling since we only care about ratios):
    //   X_new ≈ Y - X  (North-South axis after rotation)
    //   Y_new ≈ -(X + Y)  (East-West axis after rotation)
    
    wire signed [16:0] mag_x_extended = {mag_x_raw[15], mag_x_raw};
    wire signed [16:0] mag_y_extended = {mag_y_raw[15], mag_y_raw};
    
    wire signed [16:0] mag_x_rotated = mag_y_extended - mag_x_extended;  // Y - X
    wire signed [16:0] mag_y_rotated = -(mag_x_extended + mag_y_extended); // -(X + Y)
    
    // Truncate back to 16 bits (keep upper bits for better range)
    wire signed [15:0] mag_x_use = mag_x_rotated[15:0];
    wire signed [15:0] mag_y_use = mag_y_rotated[15:0];
    
    // Deadband threshold
    localparam signed [15:0] DEADBAND = 16'sd10;
    
    // Sign detection
    wire x_pos = (mag_x_use > DEADBAND);
    wire x_neg = (mag_x_use < -DEADBAND);
    wire y_pos = (mag_y_use > DEADBAND);
    wire y_neg = (mag_y_use < -DEADBAND);
    wire x_zero = !x_pos && !x_neg;
    wire y_zero = !y_pos && !y_neg;
    
    // Absolute values
    wire [15:0] abs_x = mag_x_use[15] ? (-mag_x_use) : mag_x_use;
    wire [15:0] abs_y = mag_y_use[15] ? (-mag_y_use) : mag_y_use;
    
    // ========== SECTOR DETERMINATION ==========
    // Divide the compass into 16 sectors of 22.5° each
    // Sector boundaries at: 11.25°, 33.75°, 56.25°, 78.75°, etc.
    //
    // 16 directions: N, NNE, NE, ENE, E, ESE, SE, SSE, S, SSW, SW, WSW, W, WNW, NW, NNW
    // Degrees:       0, 22.5, 45, 67.5, 90, 112.5, 135, 157.5, 180, 202.5, 225, 247.5, 270, 292.5, 315, 337.5
    //
    // To determine sector, we check Y/X ratio against tan values:
    //   tan(11.25°) ≈ 0.199 ≈ 1/5
    //   tan(33.75°) ≈ 0.668 ≈ 2/3
    //   tan(56.25°) ≈ 1.497 ≈ 3/2
    //   tan(78.75°) ≈ 5.027 ≈ 5/1
    
    // Multiply to avoid division: compare abs_y * denom vs abs_x * numer
    wire [31:0] y_times_5 = {16'b0, abs_y} * 32'd5;
    wire [31:0] x_times_1 = {16'b0, abs_x};
    wire [31:0] y_times_3 = {16'b0, abs_y} * 32'd3;
    wire [31:0] x_times_2 = {16'b0, abs_x} * 32'd2;
    wire [31:0] y_times_2 = {16'b0, abs_y} * 32'd2;
    wire [31:0] x_times_3 = {16'b0, abs_x} * 32'd3;
    wire [31:0] y_times_1 = {16'b0, abs_y};
    wire [31:0] x_times_5 = {16'b0, abs_x} * 32'd5;
    
    // Ratio comparisons (Y/X compared to threshold)
    wire ratio_lt_0p2 = (y_times_5 < x_times_1);           // Y/X < 1/5 (< 11.25°)
    wire ratio_lt_0p67 = (y_times_3 < x_times_2);          // Y/X < 2/3 (< 33.75°)
    wire ratio_lt_1p5 = (y_times_2 < x_times_3);           // Y/X < 3/2 (< 56.25°)
    wire ratio_lt_5 = (y_times_1 < x_times_5);             // Y/X < 5/1 (< 78.75°)
    
    // Determine which of 5 zones we're in (for one quadrant)
    // Zone 0: 0° to 11.25° (cardinal)
    // Zone 1: 11.25° to 33.75° (secondary intercardinal)
    // Zone 2: 33.75° to 56.25° (primary intercardinal)
    // Zone 3: 56.25° to 78.75° (secondary intercardinal)
    // Zone 4: 78.75° to 90° (cardinal)
    
    wire [2:0] angle_zone;
    assign angle_zone = ratio_lt_0p2  ? 3'd0 :   // Very close to X axis
                        ratio_lt_0p67 ? 3'd1 :   // Between cardinal and diagonal
                        ratio_lt_1p5  ? 3'd2 :   // Close to 45° diagonal
                        ratio_lt_5    ? 3'd3 :   // Between diagonal and Y axis
                                        3'd4;    // Very close to Y axis
    
    // ========== 16-DIRECTION HEADING CALCULATION ==========
    reg [8:0] heading_calc;
    
    always @(*) begin
        // Default to North if signal too weak
        if (x_zero && y_zero) begin
            heading_calc = 9'd0;
        end
        // ===== QUADRANT 1: X+, Y+ (N to E, 0° to 90°) =====
        else if (x_pos && y_pos) begin
            case (angle_zone)
                3'd0: heading_calc = 9'd0;    // N (0°)
                3'd1: heading_calc = 9'd22;   // NNE (22.5° → 22°)
                3'd2: heading_calc = 9'd45;   // NE (45°)
                3'd3: heading_calc = 9'd68;   // ENE (67.5° → 68°)
                3'd4: heading_calc = 9'd90;   // E (90°)
                default: heading_calc = 9'd45;
            endcase
        end
        // ===== QUADRANT 2: X-, Y+ (E to S, 90° to 180°) =====
        else if (x_neg && y_pos) begin
            case (angle_zone)
                3'd0: heading_calc = 9'd180;  // S (180°)
                3'd1: heading_calc = 9'd158;  // SSE (157.5° → 158°)
                3'd2: heading_calc = 9'd135;  // SE (135°)
                3'd3: heading_calc = 9'd112;  // ESE (112.5° → 112°)
                3'd4: heading_calc = 9'd90;   // E (90°)
                default: heading_calc = 9'd135;
            endcase
        end
        // ===== QUADRANT 3: X-, Y- (S to W, 180° to 270°) =====
        else if (x_neg && y_neg) begin
            case (angle_zone)
                3'd0: heading_calc = 9'd180;  // S (180°)
                3'd1: heading_calc = 9'd202;  // SSW (202.5° → 202°)
                3'd2: heading_calc = 9'd225;  // SW (225°)
                3'd3: heading_calc = 9'd248;  // WSW (247.5° → 248°)
                3'd4: heading_calc = 9'd270;  // W (270°)
                default: heading_calc = 9'd225;
            endcase
        end
        // ===== QUADRANT 4: X+, Y- (W to N, 270° to 360°) =====
        else if (x_pos && y_neg) begin
            case (angle_zone)
                3'd0: heading_calc = 9'd0;    // N (0°/360°)
                3'd1: heading_calc = 9'd338;  // NNW (337.5° → 338°)
                3'd2: heading_calc = 9'd315;  // NW (315°)
                3'd3: heading_calc = 9'd292;  // WNW (292.5° → 292°)
                3'd4: heading_calc = 9'd270;  // W (270°)
                default: heading_calc = 9'd315;
            endcase
        end
        // ===== PURE CARDINAL AXES =====
        else if (x_pos && y_zero) begin
            heading_calc = 9'd0;    // Pure North
        end
        else if (x_neg && y_zero) begin
            heading_calc = 9'd180;  // Pure South
        end
        else if (x_zero && y_pos) begin
            heading_calc = 9'd90;   // Pure East
        end
        else if (x_zero && y_neg) begin
            heading_calc = 9'd270;  // Pure West
        end
        // Default fallback
        else begin
            heading_calc = 9'd0;
        end
    end
    
    // ========== Register Output with Calibration Offset ==========
    // TEMPORARY: Offset disabled for debugging - set to 0
    // Original offset was 138° - you need to calibrate for YOUR board
    // To calibrate: Point board North, note the reading, then set offset = 360 - reading
    localparam [8:0] HEADING_OFFSET = 9'd0;  // Changed from 138 to 0 for debugging
    wire [9:0] adjusted_heading = {1'b0, heading_calc} + {1'b0, HEADING_OFFSET};
    wire [8:0] normalized_heading = (adjusted_heading >= 10'd360) ? 
                                    (adjusted_heading[8:0] - 9'd360) : adjusted_heading[8:0];
    
    // Update display only every N samples to reduce flicker
    reg [6:0] update_count;
    localparam [6:0] UPDATE_INTERVAL = 7'd64;  // Update every 64 samples (~640ms)
    
    always @(posedge clk or posedge reset) begin
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
