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


module SPI_master (
    input  wire iclk,            // 4 MHz input clock
    input  wire miso,            // Master In Slave Out
    output wire sclk,            // SPI clock (1 MHz)
    output reg  mosi = 1'b0,     // Master Out Slave In, initialized to 0
    output reg  cs = 1'b1,       // Chip Select, initialized to high (inactive)
    output wire [14:0] acl_data  // 15-bit accelerometer data (5 bits per axis)
);

    // Control signal for sclk output
    reg sclk_control = 1'b0;

    // Internal registers for clock division
    reg clk_counter = 1'b0;
    reg clk_reg     = 1'b1;

    // Clock division to generate 1 MHz sclk from 4 MHz iclk
    always @(posedge iclk) begin
        clk_counter <= clk_counter + 1;
        if (clk_counter == 1'b1)
            clk_reg <= ~clk_reg;    // Toggle to generate 1 MHz clock
    end

    // SPI communication parameters
    reg [7:0] write_instr   = 8'h0A;    // Write instruction for sensor
    reg [7:0] mode_reg_addr = 8'h2D;    // Mode register address
    reg [7:0] mode_wr_data  = 8'h02;    // Data to set measurement mode
    reg [7:0] read_instr    = 8'h0B;    // Read instruction for sensor
    reg [7:0] x_LSB_addr    = 8'h0E;    // X data LSB address (auto-increments)
    reg [14:0] temp_DATA    = 15'b0;    // Temporary 15-bit data buffer
    reg [15:0] X = 15'b0;               // X-axis data (MSB and LSB)
    reg [15:0] Y = 15'b0;               // Y-axis data (MSB and LSB)
    reg [15:0] Z = 15'b0;               // Z-axis data (MSB and LSB)
    reg [31:0] counter = 32'b0;         // State machine sync counter
    wire latch_data;

    // Bit counter for byte transmission/reception
    reg [3:0] bit_counter = 4'd0;
    
    // Byte counter for multi-byte operations
    reg [2:0] byte_counter = 3'd0;

    // State machine parameters (reduced from 93 to 11 states)
    localparam [3:0] POWER_UP       = 4'h0,  // Wait 6ms, CS = 0, SCLK = idle
                     BEGIN_SPIW     = 4'h1,  // Start SPI write
                     SEND_WCMD      = 4'h2,  // Send write command byte
                     SEND_WADDR     = 4'h3,  // Send write address byte
                     SEND_WDATA     = 4'h4,  // Send write data byte
                     WAIT           = 4'h5,  // Wait 40ms for valid data
                     BEGIN_SPIR     = 4'h6,  // Start SPI read
                     SEND_RCMD      = 4'h7,  // Send read command byte
                     SEND_RADDR     = 4'h8,  // Send read address byte
                     RECEIVE_DATA   = 4'h9,  // Receive 6 bytes of data
                     END_SPI        = 4'hA;  // Wait 10ms, loop back

    // State register initialization
    reg [3:0] state_reg = POWER_UP;
    
    // Base counter value for calculating bit timing
    reg [31:0] base_counter = 32'd0;

    // State machine logic triggered on positive edge of iclk
    always @(posedge iclk) begin
        counter <= counter + 1;    // Increment state machine sync counter
        
        case (state_reg)
            POWER_UP : begin
                if (counter == 32'd23999) begin   // Wait for 6ms
                    state_reg <= BEGIN_SPIW;
                end
            end

            BEGIN_SPIW : begin                  // Start SPI write
                if (counter == 32'd24001) begin
                    state_reg    <= SEND_WCMD;
                    cs           <= 1'b0;       // Activate chip select
                    sclk_control <= 1'b1;       // Enable sclk for CPHA = 0
                    bit_counter  <= 4'd7;       // Start with MSB
                    base_counter <= 32'd24001;  // Store base for timing
                end
            end

            SEND_WCMD : begin                   // Send write command byte
                mosi <= write_instr[bit_counter];
                if (counter == base_counter + 4 + ((7 - bit_counter) * 4)) begin
                    if (bit_counter == 4'd0) begin
                        state_reg    <= SEND_WADDR;
                        bit_counter  <= 4'd7;
                        base_counter <= counter;
                    end else begin
                        bit_counter <= bit_counter - 1;
                    end
                end
            end

            SEND_WADDR : begin                  // Send write address byte
                mosi <= mode_reg_addr[bit_counter];
                if (counter == base_counter + 4 + ((7 - bit_counter) * 4)) begin
                    if (bit_counter == 4'd0) begin
                        state_reg    <= SEND_WDATA;
                        bit_counter  <= 4'd7;
                        base_counter <= counter;
                    end else begin
                        bit_counter <= bit_counter - 1;
                    end
                end
            end

            SEND_WDATA : begin                  // Send write data byte
                mosi <= mode_wr_data[bit_counter];
                if (counter == base_counter + 4 + ((7 - bit_counter) * 4)) begin
                    if (bit_counter == 4'd0) begin
                        state_reg    <= WAIT;
                        counter      <= 32'd0;      // Reset counter
                        cs           <= 1'b1;       // Deactivate chip select
                        sclk_control <= 1'b0;       // Deactivate sclk
                    end else begin
                        bit_counter <= bit_counter - 1;
                    end
                end
            end

            WAIT : begin                         // Wait 40ms for valid data
                if (counter == 32'd400) begin
                    counter   <= 32'd0;          // Reset counter
                    state_reg <= BEGIN_SPIR;
                end
            end

            BEGIN_SPIR : begin                   // Start SPI read
                if (counter == 32'd1) begin
                    state_reg    <= SEND_RCMD;
                    cs           <= 1'b0;        // Activate chip select
                    sclk_control <= 1'b1;        // Enable sclk
                    bit_counter  <= 4'd7;        // Start with MSB
                    base_counter <= 32'd0;       // Base counter for read
                end
            end

            SEND_RCMD : begin                    // Send read command byte
                mosi <= read_instr[bit_counter];
                if (counter == 4 + ((7 - bit_counter) * 4)) begin
                    if (bit_counter == 4'd0) begin
                        state_reg    <= SEND_RADDR;
                        bit_counter  <= 4'd7;
                        base_counter <= counter;
                    end else begin
                        bit_counter <= bit_counter - 1;
                    end
                end
            end

            SEND_RADDR : begin                   // Send read address byte
                mosi <= x_LSB_addr[bit_counter];
                if (counter == base_counter + 4 + ((7 - bit_counter) * 4)) begin
                    if (bit_counter == 4'd0) begin
                        state_reg    <= RECEIVE_DATA;
                        bit_counter  <= 4'd7;
                        byte_counter <= 3'd0;
                        base_counter <= counter;
                    end else begin
                        bit_counter <= bit_counter - 1;
                    end
                end
            end

            RECEIVE_DATA : begin                 // Receive 6 bytes of data
                if (counter == base_counter + 4 + ((7 - bit_counter) * 4)) begin
                    // Sample MISO data
                    case (byte_counter)
                        3'd0: X[bit_counter] <= miso;       // X LSB
                        3'd1: X[8 + bit_counter] <= miso;   // X MSB
                        3'd2: Y[bit_counter] <= miso;       // Y LSB
                        3'd3: Y[8 + bit_counter] <= miso;   // Y MSB
                        3'd4: Z[bit_counter] <= miso;       // Z LSB
                        3'd5: Z[8 + bit_counter] <= miso;   // Z MSB
                    endcase
                    
                    if (bit_counter == 4'd0) begin
                        if (byte_counter == 3'd5) begin
                            // All 6 bytes received
                            cs           <= 1'b1;    // Deactivate chip select
                            sclk_control <= 1'b0;    // Deactivate sclk
                            state_reg    <= END_SPI;
                        end else begin
                            // Move to next byte
                            byte_counter <= byte_counter + 1;
                            bit_counter  <= 4'd7;
                            base_counter <= counter;
                        end
                    end else begin
                        bit_counter <= bit_counter - 1;
                    end
                end
            end

            END_SPI : begin                      // End SPI read, wait 10ms
                if (counter == 32'd400) begin
                    counter   <= 32'd0;          // Reset counter
                    state_reg <= BEGIN_SPIR;     // Loop back to initiate another read
                end
            end
        endcase
    end

    // Data buffer logic triggered on negative edge of iclk
    always @(negedge iclk)
        if (latch_data) begin                    // Latch data 1.5 ticks after entering END_SPI
            temp_DATA <= {X[11:7], Y[11:7], Z[11:7]}; // Latch sign bit + 4 data bits per axis
        end

    // Output accelerometer data
    assign acl_data = temp_DATA;

    assign latch_data = ((state_reg == END_SPI) && (counter == 32'd258)) ? 1 : 0;
    assign sclk = (sclk_control) ? clk_reg : 0;

endmodule
