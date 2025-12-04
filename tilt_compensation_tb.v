`timescale 1ns / 1ps
//==============================================================================
// Tilt Compensation Testbench
// Tests the sensor driver wrapper module with various magnetometer inputs
//==============================================================================
module tilt_compensation_tb;

    //==========================================================================
    // Parameters
    //==========================================================================
    parameter CLK_PERIOD = 10;      // 100 MHz system clock (10ns)
    parameter ICLK_PERIOD = 250;    // 4 MHz clock (250ns)
    
    //==========================================================================
    // DUT Signals
    //==========================================================================
    reg clk = 0;
    reg iclk = 0;
    reg reset = 1;
    
    // SPI interface (directly driven for test)
    reg miso = 1;
    wire sclk;
    wire mosi;
    wire cs;
    
    // I2C interface
    tri1 sda, scl;
    
    // Test inputs
    reg [14:0] acl_data = 0;
    reg signed [15:0] mag_x = 0;
    reg signed [15:0] mag_y = 0;
    reg signed [15:0] mag_z = 0;
    
    // Outputs
    wire signed [15:0] mag_x_comp;
    wire signed [15:0] mag_y_comp;
    wire signed [15:0] mag_x_raw;
    wire signed [15:0] mag_y_raw;
    wire signed [8:0] pitch;
    wire signed [8:0] roll;
    wire data_valid;
    wire mag_error_out;
    wire mag_busy_out;
    wire [7:0] mag_x_debug;
    
    //==========================================================================
    // Test Tracking
    //==========================================================================
    integer tests_passed = 0;
    integer tests_failed = 0;
    
    //==========================================================================
    // DUT Instantiation
    //==========================================================================
    tilt_compensation dut (
        .clk(clk),
        .iclk(iclk),
        .reset(reset),
        .miso(miso),
        .sclk(sclk),
        .mosi(mosi),
        .cs(cs),
        .sda(sda),
        .scl(scl),
        .acl_data(acl_data),
        .mag_x(mag_x),
        .mag_y(mag_y),
        .mag_z(mag_z),
        .mag_x_comp(mag_x_comp),
        .mag_y_comp(mag_y_comp),
        .mag_x_raw(mag_x_raw),
        .mag_y_raw(mag_y_raw),
        .pitch(pitch),
        .roll(roll),
        .data_valid(data_valid),
        .mag_error_out(mag_error_out),
        .mag_busy_out(mag_busy_out),
        .mag_x_debug(mag_x_debug)
    );
    
    //==========================================================================
    // Clock Generation
    //==========================================================================
    always #(CLK_PERIOD/2) clk = ~clk;
    always #(ICLK_PERIOD/2) iclk = ~iclk;
    
    //==========================================================================
    // Test Tasks
    //==========================================================================
    task check(input [511:0] name, input condition);
    begin
        if (condition) begin
            $display("  PASS: %0s", name);
            tests_passed = tests_passed + 1;
        end else begin
            $display("  FAIL: %0s", name);
            tests_failed = tests_failed + 1;
        end
    end
    endtask
    
    // Task to test magnetometer input and verify output conversion
    // Input: unsigned offset-binary (32768 = 0 Gauss)
    // Expected output: signed two's complement
    task test_mag_conversion(
        input signed [15:0] in_x,
        input signed [15:0] in_y,
        input signed [15:0] in_z,
        input signed [15:0] exp_x,
        input signed [15:0] exp_y,
        input [511:0] name
    );
    begin
        mag_x = in_x;
        mag_y = in_y;
        mag_z = in_z;
        #100;  // Wait for combinational logic to settle
        
        if (mag_x_raw == exp_x && mag_y_raw == exp_y) begin
            $display("  PASS: %0s", name);
            $display("        Input:  X=%04X Y=%04X", in_x[15:0], in_y[15:0]);
            $display("        Output: X=%04X (%0d) Y=%04X (%0d)", 
                     mag_x_raw[15:0], mag_x_raw, mag_y_raw[15:0], mag_y_raw);
            tests_passed = tests_passed + 1;
        end else begin
            $display("  FAIL: %0s", name);
            $display("        Input:    X=%04X Y=%04X", in_x[15:0], in_y[15:0]);
            $display("        Expected: X=%04X (%0d) Y=%04X (%0d)", 
                     exp_x[15:0], exp_x, exp_y[15:0], exp_y);
            $display("        Got:      X=%04X (%0d) Y=%04X (%0d)", 
                     mag_x_raw[15:0], mag_x_raw, mag_y_raw[15:0], mag_y_raw);
            tests_failed = tests_failed + 1;
        end
    end
    endtask
    
    //==========================================================================
    // Main Test Sequence
    //==========================================================================
    initial begin
        $display("\n============================================");
        $display("Tilt Compensation Testbench");
        $display("============================================\n");
        
        // Reset sequence
        reset = 1;
        mag_x = 0;
        mag_y = 0;
        mag_z = 0;
        #200;
        reset = 0;
        #200;
        
        //----------------------------------------------------------------------
        $display("TEST 1: Reset State");
        //----------------------------------------------------------------------
        check("pitch = 0 after reset", pitch == 0);
        check("roll = 0 after reset", roll == 0);
        
        //----------------------------------------------------------------------
        $display("\nTEST 2: Magnetometer Data Pass-Through");
        //----------------------------------------------------------------------
        // The module converts unsigned offset-binary to signed two's complement
        // by flipping the MSB: signed = {~unsigned[15], unsigned[14:0]}
        //
        // Offset-binary:  0x0000 = -max, 0x8000 = 0, 0xFFFF = +max
        // Two's comp:     0x8000 = -max, 0x0000 = 0, 0x7FFF = +max
        //
        // Conversion: flip MSB
        // 0x8000 (unsigned 0) -> 0x0000 (signed 0)
        // 0x0000 (unsigned -max) -> 0x8000 (signed -32768)
        // 0xFFFF (unsigned +max) -> 0x7FFF (signed +32767)
        
        // Test zero point: 0x8000 -> 0x0000
        test_mag_conversion(16'h8000, 16'h8000, 16'h8000, 
                            16'h0000, 16'h0000, "Zero point (0x8000 -> 0)");
        
        // Test positive max: 0xFFFF -> 0x7FFF (+32767)
        test_mag_conversion(16'hFFFF, 16'hFFFF, 16'hFFFF, 
                            16'h7FFF, 16'h7FFF, "Positive max (0xFFFF -> +32767)");
        
        // Test negative max: 0x0000 -> 0x8000 (-32768)
        test_mag_conversion(16'h0000, 16'h0000, 16'h0000, 
                            16'h8000, 16'h8000, "Negative max (0x0000 -> -32768)");
        
        // Test positive value: 0xC000 -> 0x4000 (+16384)
        test_mag_conversion(16'hC000, 16'hC000, 16'hC000, 
                            16'h4000, 16'h4000, "Positive value (0xC000 -> +16384)");
        
        // Test negative value: 0x4000 -> 0xC000 (-16384)
        test_mag_conversion(16'h4000, 16'h4000, 16'h4000, 
                            16'hC000, 16'hC000, "Negative value (0x4000 -> -16384)");
        
        // Test small positive: 0x8001 -> 0x0001 (+1)
        test_mag_conversion(16'h8001, 16'h8001, 16'h8001, 
                            16'h0001, 16'h0001, "Small positive (0x8001 -> +1)");
        
        // Test small negative: 0x7FFF -> 0xFFFF (-1)
        test_mag_conversion(16'h7FFF, 16'h7FFF, 16'h7FFF, 
                            16'hFFFF, 16'hFFFF, "Small negative (0x7FFF -> -1)");
        
        //----------------------------------------------------------------------
        $display("\nTEST 3: Raw vs Compensated (No Tilt Compensation)");
        //----------------------------------------------------------------------
        // Since tilt compensation is disabled, raw and comp should be identical
        mag_x = 16'hABCD;
        mag_y = 16'h1234;
        mag_z = 16'h5678;
        #100;
        check("mag_x_comp == mag_x_raw", mag_x_comp == mag_x_raw);
        check("mag_y_comp == mag_y_raw", mag_y_comp == mag_y_raw);
        
        //----------------------------------------------------------------------
        $display("\nTEST 4: Cardinal Directions (Simulated)");
        //----------------------------------------------------------------------
        // Simulate compass pointing in different directions
        // North: +Y dominant (X~0, Y=max positive)
        test_mag_conversion(16'h8000, 16'hFFFF, 16'h8000, 
                            16'h0000, 16'h7FFF, "North (+Y)");
        
        // South: -Y dominant (X~0, Y=max negative)
        test_mag_conversion(16'h8000, 16'h0000, 16'h8000, 
                            16'h0000, 16'h8000, "South (-Y)");
        
        // East: +X dominant (X=max positive, Y~0)
        test_mag_conversion(16'hFFFF, 16'h8000, 16'h8000, 
                            16'h7FFF, 16'h0000, "East (+X)");
        
        // West: -X dominant (X=max negative, Y~0)
        test_mag_conversion(16'h0000, 16'h8000, 16'h8000, 
                            16'h8000, 16'h0000, "West (-X)");
        
        // Northeast: +X, +Y
        test_mag_conversion(16'hC000, 16'hC000, 16'h8000, 
                            16'h4000, 16'h4000, "Northeast (+X, +Y)");
        
        // Southwest: -X, -Y
        test_mag_conversion(16'h4000, 16'h4000, 16'h8000, 
                            16'hC000, 16'hC000, "Southwest (-X, -Y)");
        
        //----------------------------------------------------------------------
        $display("\nTEST 5: Test Data Selection");
        //----------------------------------------------------------------------
        // When test inputs are non-zero, they should override I2C data
        mag_x = 16'hAAAA;
        mag_y = 16'h5555;
        mag_z = 16'h1234;
        #100;
        check("Test data used when non-zero", mag_x_raw != 16'h0000 || mag_y_raw != 16'h0000);
        
        // When all test inputs are zero, I2C data would be used (but we can't test that without I2C slave)
        // This at least verifies the test data override works
        
        //----------------------------------------------------------------------
        $display("\nTEST 6: Data Valid Pulse Generation");
        //----------------------------------------------------------------------
        // Wait for at least one data_valid pulse (should occur every ~10ms at 4MHz)
        // 10ms at 4MHz = 40,000 cycles = 40,000 * 250ns = 10ms
        // We'll wait a bit longer to catch it
        begin : data_valid_test
            integer timeout;
            integer pulse_count;
            pulse_count = 0;
            timeout = 0;
            
            // Wait for up to 15ms worth of cycles
            while (timeout < 60000 && pulse_count < 1) begin
                @(posedge iclk);
                timeout = timeout + 1;
                if (data_valid) pulse_count = pulse_count + 1;
            end
            
            check("data_valid pulses generated", pulse_count >= 1);
        end
        
        //----------------------------------------------------------------------
        $display("\nTEST 7: Alternating Bit Patterns");
        //----------------------------------------------------------------------
        // Test with alternating bits to catch any bit-level issues
        test_mag_conversion(16'hAAAA, 16'h5555, 16'h0000, 
                            16'h2AAA, 16'hD555, "Alternating bits (0xAAAA, 0x5555)");
        
        test_mag_conversion(16'h5555, 16'hAAAA, 16'h0000, 
                            16'hD555, 16'h2AAA, "Alternating bits reversed");
        
        //----------------------------------------------------------------------
        $display("\n============================================");
        $display("RESULTS: %0d PASS, %0d FAIL", tests_passed, tests_failed);
        $display("============================================\n");
        
        #1000;
        $finish;
    end
    
    //==========================================================================
    // Timeout
    //==========================================================================
    initial begin
        #100_000_000;  // 100ms timeout
        $display("\n*** TIMEOUT ***");
        $finish;
    end

endmodule
