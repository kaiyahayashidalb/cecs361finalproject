`timescale 1ns / 1ps

module top(
    input clk,              // System clock (100 MHz)
    input reset,            // SW[0] - reset switch
    
    // SPI interface for accelerometer (directly on Nexys A7)
    input ACL_MISO,
    output ACL_SCLK,
    output ACL_MOSI,
    output ACL_CSN,
    
    // I2C interface for magnetometer (directly on Pmod or external)
    inout sda,
    inout scl,
    
    // Display outputs
    output reg [6:0] seg,
    output reg [7:0] an
);

    // ========== Clock Generation ==========
    // Generate 4 MHz clock from 100 MHz for SPI master
    reg [4:0] clk_div = 0;
    reg clk_4mhz = 0;
    
    always @(posedge clk) begin
        if (clk_div == 5'd12) begin  // 100MHz / 25 = 4MHz
            clk_div <= 0;
            clk_4mhz <= ~clk_4mhz;
        end else begin
            clk_div <= clk_div + 1;
        end
    end

    // ========== Heading Calculation (includes Tilt Compensation, SPI, I2C) ==========
    wire [8:0] heading;
    wire signed [8:0] pitch, roll;
    wire heading_valid;
    
    heading_calculation heading_inst (
        .clk(clk),
        .iclk(clk_4mhz),
        .reset(reset),
        .miso(ACL_MISO),
        .sclk(ACL_SCLK),
        .mosi(ACL_MOSI),
        .cs(ACL_CSN),
        .sda(sda),
        .scl(scl),
        // Test inputs tied to 0 - use real hardware drivers
        .acl_data(15'd0),
        .mag_x(16'sd0),
        .mag_y(16'sd0),
        .mag_z(16'sd0),
        // Outputs
        .heading(heading),
        .pitch(pitch),
        .roll(roll),
        .data_valid(heading_valid)
    );

    // ========== Display Heading on 7-Segment ==========
    // Convert heading (0-359) to BCD digits
    wire [3:0] hundreds, tens, ones;
    
    binary_to_bcd converter(
        .binary({1'b0, heading}),
        .hundreds(hundreds),
        .tens(tens),
        .ones(ones)
    );

    // 7-segment decoders
    wire [6:0] seg0, seg1, seg2;
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
            default: begin seg = 7'b1111111; an = 8'b11111111; end  // Off
        endcase
    end

endmodule
