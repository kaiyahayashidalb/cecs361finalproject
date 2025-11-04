`timescale 1ns / 1ps

module top_tb;

    // Testbench signals
    reg clk;
    reg reset;
    reg [8:0] switches;
    wire [3:0] hundreds, tens, ones;  // Observe the BCD outputs

    // Instantiate the top module (we expose BCD outputs via the converter)
    binary_to_bcd converter(
        .binary({1'b0, switches}),
        .hundreds(hundreds),
        .tens(tens),
        .ones(ones)
    );

    // Clock generation (if needed by top module)
    initial begin
        clk = 0;
        forever #10 clk = ~clk; // 50MHz clock
    end

    // Test procedure
    initial begin
        reset = 1;
        switches = 0;
        #20;
        reset = 0;

        // Test cases
        switches = 9'd0;   #20;
        switches = 9'd1;   #20; 
        switches = 9'd5;   #20; 
        switches = 9'd9;   #20;
        switches = 9'd12;  #20; 
        switches = 9'd45;  #20; 
        switches = 9'd359; #20; 

        $finish;
    end

endmodule
