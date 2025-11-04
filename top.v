`timescale 1ns / 1ps

module top(
    input clk,
    input reset,
    input [8:0] switches,   // 9 switches (0-511)
    output reg [6:0] seg,
    output reg [7:0] an
);

    // Convert 9-bit binary to BCD digits
    wire [3:0] hundreds, tens, ones;
    
    binary_to_bcd converter(
        .binary({1'b0, switches}),
        .hundreds(hundreds),
        .tens(tens),
        .ones(ones)
    );

    // 7-segment decoders
    wire [6:0] seg0, seg1, seg2, seg3, seg4, seg5, seg6, seg7;
    bcd_to_7seg dec0(.bcd(ones),     .seg(seg0));
    bcd_to_7seg dec1(.bcd(tens),     .seg(seg1));
    bcd_to_7seg dec2(.bcd(hundreds), .seg(seg2));

    // Multiplexing counter
    reg [19:0] refresh_counter = 0;
    wire [1:0] current_digit = refresh_counter[19:18];

    always @(posedge clk or posedge reset) begin
        if (reset)
            refresh_counter <= 0;
        else
            refresh_counter <= refresh_counter + 1;
    end

    // Drive segments & anodes
  always @(*) begin
        case (current_digit)
            2'd0: begin seg = seg0; an = 8'b11111110; end  // Ones
            2'd1: begin seg = seg1; an = 8'b11111101; end  // Tens
            2'd2: begin seg = seg2; an = 8'b11111011; end  // Hundreds
            2'd3: begin seg = seg3; an = 8'b11111111; end  // Off
            2'd4: begin seg = seg4; an = 8'b11111111; end  // Off
            2'd5: begin seg = seg5; an = 8'b11111111; end  // Off
            2'd6: begin seg = seg6; an = 8'b11111111; end  // Off
            2'd7: begin seg = seg7; an = 8'b11111111; end  // Off
            default: begin seg = 7'b1111111; an = 8'b11111111; end
        endcase
    end


endmodule
