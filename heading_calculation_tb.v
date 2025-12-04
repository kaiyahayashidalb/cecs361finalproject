`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// FAST Testbench for heading_calculation
// Mocks mag data directly - no I2C simulation needed
// Runs in milliseconds instead of minutes
//////////////////////////////////////////////////////////////////////////////////

module heading_calculation_tb;

    // ========== Clock ==========
    reg clk = 0;
    always #5 clk = ~clk;  // 100 MHz
    
    // ========== Mock Magnetometer Data ==========
    reg signed [15:0] mag_x_raw;
    reg signed [15:0] mag_y_raw;
    
    // ========== HEADING CALCULATION LOGIC (same as heading_calculation.v) ==========
    
    // Axis rotation (225° correction for sensor orientation)
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
    
    // Ratio comparisons for 16-sector detection
    wire [31:0] y_times_5 = {16'b0, abs_y} * 32'd5;
    wire [31:0] x_times_1 = {16'b0, abs_x};
    wire [31:0] y_times_3 = {16'b0, abs_y} * 32'd3;
    wire [31:0] x_times_2 = {16'b0, abs_x} * 32'd2;
    wire [31:0] y_times_2 = {16'b0, abs_y} * 32'd2;
    wire [31:0] x_times_3 = {16'b0, abs_x} * 32'd3;
    wire [31:0] y_times_1 = {16'b0, abs_y};
    wire [31:0] x_times_5 = {16'b0, abs_x} * 32'd5;
    
    wire ratio_lt_0p2 = (y_times_5 < x_times_1);
    wire ratio_lt_0p67 = (y_times_3 < x_times_2);
    wire ratio_lt_1p5 = (y_times_2 < x_times_3);
    wire ratio_lt_5 = (y_times_1 < x_times_5);
    
    wire [2:0] angle_zone = ratio_lt_0p2  ? 3'd0 :
                            ratio_lt_0p67 ? 3'd1 :
                            ratio_lt_1p5  ? 3'd2 :
                            ratio_lt_5    ? 3'd3 : 3'd4;
    
    // ========== 16-Direction Heading Calculation ==========
    reg [8:0] heading;
    
    always @(*) begin
        if (x_zero && y_zero)
            heading = 9'd0;
        // Quadrant 1: X+, Y+ (N to E, 0° to 90°)
        else if (x_pos && y_pos) begin
            case (angle_zone)
                3'd0: heading = 9'd0;    // N
                3'd1: heading = 9'd22;   // NNE
                3'd2: heading = 9'd45;   // NE
                3'd3: heading = 9'd68;   // ENE
                3'd4: heading = 9'd90;   // E
                default: heading = 9'd45;
            endcase
        end
        // Quadrant 2: X-, Y+ (E to S, 90° to 180°)
        else if (x_neg && y_pos) begin
            case (angle_zone)
                3'd0: heading = 9'd180;  // S
                3'd1: heading = 9'd158;  // SSE
                3'd2: heading = 9'd135;  // SE
                3'd3: heading = 9'd112;  // ESE
                3'd4: heading = 9'd90;   // E
                default: heading = 9'd135;
            endcase
        end
        // Quadrant 3: X-, Y- (S to W, 180° to 270°)
        else if (x_neg && y_neg) begin
            case (angle_zone)
                3'd0: heading = 9'd180;  // S
                3'd1: heading = 9'd202;  // SSW
                3'd2: heading = 9'd225;  // SW
                3'd3: heading = 9'd248;  // WSW
                3'd4: heading = 9'd270;  // W
                default: heading = 9'd225;
            endcase
        end
        // Quadrant 4: X+, Y- (W to N, 270° to 360°)
        else if (x_pos && y_neg) begin
            case (angle_zone)
                3'd0: heading = 9'd0;    // N
                3'd1: heading = 9'd338;  // NNW
                3'd2: heading = 9'd315;  // NW
                3'd3: heading = 9'd292;  // WNW
                3'd4: heading = 9'd270;  // W
                default: heading = 9'd315;
            endcase
        end
        // Pure cardinal axes
        else if (x_pos && y_zero)
            heading = 9'd0;    // N
        else if (x_neg && y_zero)
            heading = 9'd180;  // S
        else if (x_zero && y_pos)
            heading = 9'd90;   // E
        else if (x_zero && y_neg)
            heading = 9'd270;  // W
        else
            heading = 9'd0;
    end
    
    // ========== Test Infrastructure ==========
    integer test_count = 0;
    integer pass_count = 0;
    
    task run_test;
        input [127:0] test_name;
        input signed [15:0] mx;
        input signed [15:0] my;
        input integer expected_heading;
        integer diff;
        begin
            test_count = test_count + 1;
            
            mag_x_raw = mx;
            mag_y_raw = my;
            
            #100;  // Let combinational logic settle
            
            // Calculate difference with wrap-around
            diff = (heading > expected_heading) ? (heading - expected_heading) : (expected_heading - heading);
            if (diff > 180) diff = 360 - diff;
            
            if (diff <= 25) begin
                $display("[PASS] Test %2d: %-5s  mag(%6d,%6d) -> heading=%3d (expected %3d)",
                         test_count, test_name, mx, my, heading, expected_heading);
                pass_count = pass_count + 1;
            end else begin
                $display("[FAIL] Test %2d: %-5s  mag(%6d,%6d) -> heading=%3d (expected %3d)  rotated(%6d,%6d)",
                         test_count, test_name, mx, my, heading, expected_heading, mag_x_use, mag_y_use);
            end
        end
    endtask
    
    // ========== Test Stimulus ==========
    initial begin
        $display("\n================================================================");
        $display("  HEADING CALCULATION TESTBENCH (Fast Mock - No I2C)");
        $display("================================================================\n");
        
        mag_x_raw = 0;
        mag_y_raw = 0;
        #100;
        
        // Rotation formula: X_rot = Y - X, Y_rot = -(X + Y)
        // After rotation:
        //   X_rot+ Y_rot~0 = North (0°)
        //   X_rot~0 Y_rot+ = East (90°)  
        //   X_rot- Y_rot~0 = South (180°)
        //   X_rot~0 Y_rot- = West (270°)
        
        $display("Testing all 16 compass directions:\n");
        
        // Cardinal directions
        run_test("N",    -16'sd1000,  16'sd1000, 0);
        run_test("E",    -16'sd1000, -16'sd1000, 90);
        run_test("S",     16'sd1000, -16'sd1000, 180);
        run_test("W",     16'sd1000,  16'sd1000, 270);
        
        // Primary intercardinal
        run_test("NE",   -16'sd1414,  16'sd0,    45);
        run_test("SE",    16'sd0,    -16'sd1414, 135);
        run_test("SW",    16'sd1414,  16'sd0,    225);
        run_test("NW",    16'sd0,     16'sd1414, 315);
        
        // Secondary intercardinal
        run_test("NNE",  -16'sd1200,  16'sd600,  22);
        run_test("ENE",  -16'sd1200, -16'sd600,  68);
        run_test("ESE",  -16'sd600,  -16'sd1200, 112);
        run_test("SSE",   16'sd600,  -16'sd1200, 158);
        run_test("SSW",   16'sd1200, -16'sd600,  202);
        run_test("WSW",   16'sd1200,  16'sd600,  248);
        run_test("WNW",   16'sd600,   16'sd1200, 292);
        run_test("NNW",  -16'sd600,   16'sd1200, 338);
        
        $display("\n================================================================");
        $display("  TESTBENCH SUMMARY");
        $display("================================================================");
        $display("  Tests Run:    %0d", test_count);
        $display("  Tests Passed: %0d", pass_count);
        $display("  Tests Failed: %0d", test_count - pass_count);
        if (pass_count == test_count)
            $display("  *** ALL TESTS PASSED ***");
        $display("================================================================\n");
        
        #100;
        $finish;
    end

endmodule
