`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: CECS 361 Final Project
// Engineer: Nathan Sarkozy
// 
// Create Date: 11/30/2025 05:54:44 PM
// Design Name: SPI_master_tb
// Module Name: SPI_master_tb
// Project Name: Compass
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

module SPI_master_tb;

    // Inputs
    reg iclk;
    wire cs;
    wire sclk;
    reg miso;
    wire mosi;
    wire [14:0] acl_data;
    
    // Testbench variables
    reg [15:0] x_data = 16'h0000;
    reg [15:0] y_data = 16'h0000;
    reg [15:0] z_data = 16'h0000;
    
    // Test tracking
    integer test_case = 0;
    integer tests_passed = 0;
    integer tests_failed = 0;
    
    // Expected results storage
    reg [14:0] expected_combined;
    reg [4:0] expected_x, expected_y, expected_z;
    
    // Instantiate the Unit Under Test (UUT)
    SPI_master uut (
        .iclk(iclk),
        .cs(cs),
        .sclk(sclk),
        .mosi(mosi),
        .miso(miso),
        .acl_data(acl_data)
    );
    
    // Clock generation (4 MHz)
    always begin
        iclk = 0;
        #125;
        iclk = 1;
        #125;
    end
    
    // Function to extract expected 5-bit value from 16-bit input
    function [4:0] extract_5bit;
        input [15:0] data;
        begin
            extract_5bit = data[11:7];  // Bits [11:7] as per design
        end
    endfunction
    
    // MISO simulation - accelerometer response
    always @(negedge sclk or posedge cs) begin
        if (cs) begin
            miso <= 1'b0;
        end else begin
            // During RECEIVE_DATA state (4'h9)
            if (uut.state_reg == 4'h9) begin
                case (uut.byte_counter)
                    3'd0: miso <= x_data[uut.bit_counter];        // X LSB
                    3'd1: miso <= x_data[8 + uut.bit_counter];    // X MSB
                    3'd2: miso <= y_data[uut.bit_counter];        // Y LSB
                    3'd3: miso <= y_data[8 + uut.bit_counter];    // Y MSB
                    3'd4: miso <= z_data[uut.bit_counter];        // Z LSB
                    3'd5: miso <= z_data[8 + uut.bit_counter];    // Z MSB
                    default: miso <= 1'b0;
                endcase
            end else begin
                miso <= 1'b0;
            end
        end
    end
    
    // Enhanced test task with realistic accelerometer values
    task run_orientation_test;
        input signed [15:0] x_accel;  // X-axis 12-bit signed value in 16-bit format
        input signed [15:0] y_accel;  // Y-axis 12-bit signed value in 16-bit format
        input signed [15:0] z_accel;  // Z-axis 12-bit signed value in 16-bit format
        input [8*80:1] test_description;
        
        reg [4:0] x_5bit;
        reg [4:0] y_5bit;
        reg [4:0] z_5bit;
        reg [3:0] x_4bit;  // Actual 4-bit LED display value
        reg [3:0] y_4bit;
        reg [3:0] z_4bit;
        reg [14:0] combined_15bit;
        
        begin
            test_case = test_case + 1;
            
            $display("TEST %0d: %s", test_case, test_description);
            
           
            
        
            
            // Extract bits [11:7] (5 bits as acl_data stores them)
            x_5bit = x_accel[11:7];
            y_5bit = y_accel[11:7];
            z_5bit = z_accel[11:7];
            
       
            
            $display("bits [11:7] from sensor:");
            $display("  X [11:7]: 0b%05b ", x_5bit, x_5bit);
            $display("  Y [11:7]: 0b%05b ", y_5bit, y_5bit);
            $display("  Z [11:7]: 0b%05b ", z_5bit, z_5bit);
            
          
            
            // Combine into 15-bit orientation value (as stored internally)
            combined_15bit = {x_5bit, y_5bit, z_5bit};
            
            $display("\nCOMBINED 15-BIT OUTPUT (acl_data[14:0]):");
            $display("  Full value: 0b%015b (0x%04h)", combined_15bit, combined_15bit);
            
            // Set test data for simulation
            x_data = x_accel;
            y_data = y_accel;
            z_data = z_accel;
            
            // Calculate expected results
            expected_x = x_5bit;
            expected_y = y_5bit;
            expected_z = z_5bit;
            expected_combined = combined_15bit;
            
            // Wait for data acquisition to complete
            wait(uut.state_reg == 4'hA && uut.counter == 32'd258);  // END_SPI state
            @(posedge iclk);
            @(posedge iclk);
            
            // Verify results
            $display("\n--- VERIFICATION ---");
            $display("Component          Expected        Actual          Status");
            $display("-------------------------------------------------------------");
            $display("acl_data[14:0]     0x%04h          0x%04h          %s", 
                    expected_combined, acl_data,
                    (expected_combined === acl_data) ? "PASS" : "FAIL");
            $display("X [14:10]          0b%05b         0b%05b         %s", 
                    expected_x, acl_data[14:10],
                    (expected_x === acl_data[14:10]) ? "PASS" : "FAIL");
            $display("Y [9:5]            0b%05b         0b%05b         %s", 
                    expected_y, acl_data[9:5],
                    (expected_y === acl_data[9:5]) ? "PASS" : "FAIL");
            $display("Z [4:0]            0b%05b         0b%05b         %s", 
                    expected_z, acl_data[4:0],
                    (expected_z === acl_data[4:0]) ? "PASS" : "FAIL");
            
       
            
            // Check overall test result
            if ((expected_combined === acl_data) &&
                (expected_x === acl_data[14:10]) &&
                (expected_y === acl_data[9:5]) &&
                (expected_z === acl_data[4:0])) begin
                tests_passed = tests_passed + 1;
                $display(" TEST RESULT: PASS ");
            end else begin
                tests_failed = tests_failed + 1;
                $display(" TEST RESULT: FAIL");
            end
        end
    endtask
    
    // Main test sequence
    initial begin
        // Initialize
        iclk = 0;
        miso = 0;
        tests_passed = 0;
        tests_failed = 0;
        
        #100;
        
      //MISO STRESS TEST
        
        // Test Case 1: FLAT ON TABLE (0g, 0g, +1g)
        run_orientation_test(
            16'b0101010101010101,  // X = 0x5555
            16'b0101010101010101,  // Y = 0x5555  
            16'b0101010101010101,  // Z = 0x5555
            "ALTERNATING BITS"
        );
        
        // Test Case 2: FLAT ON TABLE (0g, 0g, +1g)        
        run_orientation_test(
            16'b1010101010101010,  // X = 0xAAAA
            16'b1010101010101010,  // Y = 0xAAAA
            16'b1010101010101010,  // Z = 0xAAAA
            "INVERSE ALTERNATING"
        );

        // Test Case 3: FLAT ON TABLE (0g, 0g, +1g)        
       run_orientation_test(
            16'b0101010101010101,  // X = 0x5555
            16'b1010101010101010,  // Y = 0xAAAA (inverse of X)
            16'b0101010101010101,  // Z = 0x5555
            "MIX ALTERNATION: X/Z = 0101, Y = 1010"
        );
        
       //acl_data in
        
        // Test Case 4: FLAT ON TABLE 
        run_orientation_test(
            16'h0000, 
            16'h0000, 
            16'h0100,  
            "FLAT ON TABLE (switches at bottom: X=0, Y=0, Z=8"
        );
        
        // Test Case 5: TOP ROTATED 90° - X-AXIS DOWN 
        run_orientation_test(
            16'h00E0, 
            16'h0000,  
            16'h0000,
            "TOP ROTATED 90 (X-axis down): X=7, Y=0, Z=0"
        );
        
        // Test Case 6: SIDE ROTATED 90° - Y-AXIS DOWN (0g, +1g, 0g)
        run_orientation_test(
            16'h0000,  
            16'h00E0, 
            16'h0000,  
            "SIDE ROTATED 90: X=0, Y=7, Z=0"
        );
        
        // Final summary

        $display("Total Tests:     %0d", (tests_passed + tests_failed));
        $display("Tests Passed:    %0d", tests_passed);
        $display("Tests Failed:    %0d", tests_failed);
        
      
        #10000;
        $finish;
    end

endmodule