module binary_to_bcd(
    input [9:0] binary,        // 0-511
    output reg [3:0] hundreds, // 0-5 (for 0-511)
    output reg [3:0] tens,     // 0-9
    output reg [3:0] ones      // 0-9
);
    always @(*) begin
        if (binary > 511) begin
            hundreds = 5;
            tens     = 1;
            ones     = 1;
        end else begin
            hundreds = binary / 100;
            tens     = (binary % 100) / 10;
            ones     = binary % 10;
        end
    end
endmodule