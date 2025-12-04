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

    // ========== 16-DIRECTION COMPASS CALCULATION ==========
    // Maps magnetometer readings to 16 directions (22.5° resolution)
    // N=0°, NNE=22.5°, NE=45°, ENE=67.5°, E=90°, ESE=112.5°, SE=135°, SSE=157.5°,
    // S=180°, SSW=202.5°, SW=225°, WSW=247.5°, W=270°, WNW=292.5°, NW=315°, NNW=337.5°
    
    // AXIS CORRECTION: Your sensor is rotated 225° (or -135°) from standard
    wire signed [16:0] mag_x_extended = {mag_x_raw[15], mag_x_raw};
    wire signed [16:0] mag_y_extended = {mag_y_raw[15], mag_y_raw};
    
    wire signed [16:0] mag_x_rotated = mag_y_extended - mag_x_extended;  // Y - X
    wire signed [16:0] mag_y_rotated = -(mag_x_extended + mag_y_extended); // -(X + Y)
    
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
    
    // ========== 16-SECTOR DETERMINATION ==========
    // Divide compass into 16 equal sectors of 22.5° each
    // Sector boundaries at: 11.25°, 33.75°, 56.25°, 78.75° (and equivalents in other quadrants)
    //
    // For equal sectors, we need: tan(11.25°) ≈ 0.199, tan(33.75°) ≈ 0.668, tan(56.25°) ≈ 1.496, tan(78.75°) ≈ 5.027
    //
    // Using bit-shifts for hardware efficiency:
    // tan(11.25°) ≈ 0.199 ≈ 51/256 ≈ X>>2 - X>>4 (0.1875, close enough)
    // tan(33.75°) ≈ 0.668 ≈ 171/256 ≈ 2/3
    // tan(56.25°) ≈ 1.496 ≈ 3/2
    // tan(78.75°) ≈ 5.027 ≈ 5
    
    wire [16:0] abs_x_17 = {1'b0, abs_x};
    wire [16:0] abs_y_17 = {1'b0, abs_y};
    
    // Precise sector boundaries for equal 22.5° sectors
    // Boundary 1: 11.25° → tan = 0.199 ≈ 1/5
    wire [17:0] boundary1_x = {abs_x_17, 1'b0} + (abs_x_17 << 1);  // 5X for comparison
    wire sector1 = (abs_y_17 * 5'd5) < boundary1_x;                 // Y/X < 0.2 (±11.25°)
    
    // Boundary 2: 33.75° → tan = 0.668 ≈ 2/3
    wire [17:0] boundary2_x = abs_x_17 + (abs_x_17 >> 1);          // 1.5X
    wire sector2 = (abs_y_17 + (abs_y_17 >> 1)) < boundary2_x;     // Y*1.5 < X*1.5 → Y < X*2/3
    
    // Boundary 3: 56.25° → tan = 1.496 ≈ 3/2
    wire [17:0] boundary3_y = abs_y_17 + (abs_y_17 >> 1);          // 1.5Y
    wire sector3 = abs_x_17 < boundary3_y;                          // X < 1.5Y → Y/X > 2/3
    
    // Boundary 4: 78.75° → tan = 5.027 ≈ 5
    wire [18:0] boundary4_y = (abs_y_17 << 2) + abs_y_17;          // 5Y
    wire sector4 = abs_x_17 < boundary4_y;                          // X < 5Y → Y/X > 1/5
    
    // ========== 16-DIRECTION HEADING CALCULATION ==========
    // Each sector covers exactly 22.5° (360°/16)
    // Sectors centered at: 0°, 22.5°, 45°, 67.5°, 90°, 112.5°, 135°, 157.5°, 180°, 202.5°, 225°, 247.5°, 270°, 292.5°, 315°, 337.5°
    // Boundaries at: ±11.25° from center
    
    reg [8:0] heading_calc;
    
    always @(*) begin
        if (x_zero && y_zero) begin
            heading_calc = 9'd0;  // No signal
        end
        // ===== QUADRANT 1: X+, Y+ (348.75° to 90°) =====
        else if (x_pos && y_pos) begin
            if (sector1)
                heading_calc = 9'd0;      // N (348.75° to 11.25°)
            else if (sector2)
                heading_calc = 9'd23;     // NNE (11.25° to 33.75°)
            else if (sector3)
                heading_calc = 9'd45;     // NE (33.75° to 56.25°)
            else if (sector4)
                heading_calc = 9'd68;     // ENE (56.25° to 78.75°)
            else
                heading_calc = 9'd90;     // E (78.75° to 101.25°)
        end
        // ===== QUADRANT 2: X-, Y+ (78.75° to 180°) =====
        else if (x_neg && y_pos) begin
            if (sector4)
                heading_calc = 9'd90;     // E (78.75° to 101.25°)
            else if (sector3)
                heading_calc = 9'd113;    // ESE (101.25° to 123.75°)
            else if (sector2)
                heading_calc = 9'd135;    // SE (123.75° to 146.25°)
            else if (sector1)
                heading_calc = 9'd158;    // SSE (146.25° to 168.75°)
            else
                heading_calc = 9'd180;    // S (168.75° to 191.25°)
        end
        // ===== QUADRANT 3: X-, Y- (168.75° to 270°) =====
        else if (x_neg && y_neg) begin
            if (sector1)
                heading_calc = 9'd180;    // S (168.75° to 191.25°)
            else if (sector2)
                heading_calc = 9'd203;    // SSW (191.25° to 213.75°)
            else if (sector3)
                heading_calc = 9'd225;    // SW (213.75° to 236.25°)
            else if (sector4)
                heading_calc = 9'd248;    // WSW (236.25° to 258.75°)
            else
                heading_calc = 9'd270;    // W (258.75° to 281.25°)
        end
        // ===== QUADRANT 4: X+, Y- (258.75° to 360°) =====
        else if (x_pos && y_neg) begin
            if (sector4)
                heading_calc = 9'd270;    // W (258.75° to 281.25°)
            else if (sector3)
                heading_calc = 9'd293;    // WNW (281.25° to 303.75°)
            else if (sector2)
                heading_calc = 9'd315;    // NW (303.75° to 326.25°)
            else if (sector1)
                heading_calc = 9'd338;    // NNW (326.25° to 348.75°)
            else
                heading_calc = 9'd0;      // N (348.75° to 11.25°)
        end
        // ===== PURE AXES =====
        else if (x_pos)
            heading_calc = 9'd0;          // N
        else if (x_neg)
            heading_calc = 9'd180;        // S
        else if (y_pos)
            heading_calc = 9'd90;         // E
        else if (y_neg)
            heading_calc = 9'd270;        // W
        else
            heading_calc = 9'd0;          // Default
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
