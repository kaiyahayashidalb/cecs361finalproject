`timescale 1ns / 1ps
// Digital Compass Testbench
module top_tb;

    // Clock and Reset
    reg clk = 0;
    reg reset;
    always #5 clk = ~clk;  // 100 MHz

    // Interfaces
    reg ACL_MISO = 0;
    wire ACL_SCLK, ACL_MOSI, ACL_CSN;
    wire sda, scl;
    wire [6:0] seg;
    wire [7:0] an;
    wire [15:0] LED;

    // Test counters
    integer tests_passed = 0;
    integer tests_failed = 0;
    integer test_num = 0;

    // Test stimulus
    reg signed [15:0] sim_mag_x = 0, sim_mag_y = 0, sim_mag_z = 0;

    // I2C Slave Model (Magnetometer @ 0x30)
    localparam [6:0] MAG_ADDR = 7'h30;
    
    // State machine
    localparam S_IDLE = 0, S_ADDR = 1, S_ADDR_ACK = 2, S_REG = 3, S_REG_ACK = 4,
               S_DATA_OUT = 5, S_DATA_ACK = 6, S_DATA_IN = 7, S_DATA_IN_ACK = 8;
    
    reg [3:0] i2c_state = S_IDLE;
    reg [7:0] i2c_shift = 0;
    reg [3:0] i2c_bit = 0;
    reg [7:0] i2c_reg_addr = 0;
    reg i2c_rw = 0;
    reg i2c_sda_out = 1;
    reg i2c_driving = 0;

    assign sda = i2c_driving ? i2c_sda_out : 1'bz;
    pullup(sda); 
    pullup(scl);

    // Edge detection
    reg [3:0] scl_sync = 4'hF, sda_sync = 4'hF;
    reg scl_rise, scl_fall, sda_fall, sda_rise, scl_high, sda_val;
    reg i2c_start_det, i2c_stop_det;
    
    always @(posedge clk) begin
        scl_sync <= {scl_sync[2:0], scl};
        sda_sync <= {sda_sync[2:0], sda};
        
        scl_rise <= (scl_sync[2:1] == 2'b01);
        scl_fall <= (scl_sync[2:1] == 2'b10);
        sda_fall <= (sda_sync[2:1] == 2'b10);
        sda_rise <= (sda_sync[2:1] == 2'b01);
        scl_high <= scl_sync[1];
        sda_val  <= sda_sync[1];
        
        i2c_start_det <= sda_fall && scl_high;
        i2c_stop_det  <= sda_rise && scl_high;
    end

    // Convert SIGNED magnetometer values to UNSIGNED for sensor output
    function [7:0] get_mag_data(input [7:0] addr);
        reg [15:0] unsigned_x, unsigned_y, unsigned_z;
        begin
            unsigned_x = sim_mag_x + 16'sd32768;
            unsigned_y = sim_mag_y + 16'sd32768;
            unsigned_z = sim_mag_z + 16'sd32768;
            
            case(addr)
                8'h00: get_mag_data = unsigned_x[7:0];
                8'h01: get_mag_data = unsigned_x[15:8];
                8'h02: get_mag_data = unsigned_y[7:0];
                8'h03: get_mag_data = unsigned_y[15:8];
                8'h04: get_mag_data = unsigned_z[7:0];
                8'h05: get_mag_data = unsigned_z[15:8];
                default: get_mag_data = 8'h00;
            endcase
        end
    endfunction

    // I2C Slave FSM
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            i2c_state <= S_IDLE;
            i2c_driving <= 0;
            i2c_sda_out <= 1;
            i2c_shift <= 0;
            i2c_bit <= 0;
            i2c_reg_addr <= 0;
            i2c_rw <= 0;
        end else begin
            if (i2c_start_det) begin
                i2c_state <= S_ADDR;
                i2c_bit <= 0;
                i2c_shift <= 0;
                i2c_driving <= 0;
            end
            else if (i2c_stop_det) begin
                i2c_state <= S_IDLE;
                i2c_driving <= 0;
            end
            else begin
                case (i2c_state)
                    S_IDLE: i2c_driving <= 0;
                    
                    S_ADDR: begin
                        if (scl_rise) begin
                            i2c_shift <= {i2c_shift[6:0], sda_val};
                            i2c_bit <= i2c_bit + 1;
                        end
                        if (scl_fall && i2c_bit == 8) begin
                            i2c_rw <= i2c_shift[0];
                            if (i2c_shift[7:1] == MAG_ADDR) begin
                                i2c_state <= S_ADDR_ACK;
                                i2c_driving <= 1;
                                i2c_sda_out <= 0;
                            end else begin
                                i2c_state <= S_IDLE;
                            end
                        end
                    end
                    
                    S_ADDR_ACK: begin
                        if (scl_fall) begin
                            i2c_bit <= 0;
                            if (i2c_rw) begin
                                i2c_state <= S_DATA_OUT;
                                i2c_shift <= get_mag_data(i2c_reg_addr);
                                i2c_sda_out <= get_mag_data(i2c_reg_addr) >> 7;
                            end else begin
                                i2c_state <= S_REG;
                                i2c_driving <= 0;
                                i2c_shift <= 0;
                            end
                        end
                    end
                    
                    S_REG: begin
                        if (scl_rise) begin
                            i2c_shift <= {i2c_shift[6:0], sda_val};
                            i2c_bit <= i2c_bit + 1;
                        end
                        if (scl_fall && i2c_bit == 8) begin
                            i2c_reg_addr <= i2c_shift;
                            i2c_state <= S_REG_ACK;
                            i2c_driving <= 1;
                            i2c_sda_out <= 0;
                        end
                    end
                    
                    S_REG_ACK: begin
                        if (scl_fall) begin
                            i2c_bit <= 0;
                            i2c_shift <= 0;
                            i2c_state <= S_DATA_IN;
                            i2c_driving <= 0;
                        end
                    end
                    
                    S_DATA_IN: begin
                        if (scl_rise) begin
                            i2c_shift <= {i2c_shift[6:0], sda_val};
                            i2c_bit <= i2c_bit + 1;
                        end
                        if (scl_fall && i2c_bit == 8) begin
                            i2c_state <= S_DATA_IN_ACK;
                            i2c_driving <= 1;
                            i2c_sda_out <= 0;
                            i2c_bit <= 0;
                        end
                    end
                    
                    S_DATA_IN_ACK: begin
                        if (scl_fall) begin
                            i2c_reg_addr <= i2c_reg_addr + 1;
                            i2c_state <= S_DATA_IN;
                            i2c_driving <= 0;
                            i2c_shift <= 0;
                        end
                    end
                    
                    S_DATA_OUT: begin
                        if (scl_fall) begin
                            i2c_bit <= i2c_bit + 1;
                            if (i2c_bit < 7) begin
                                i2c_shift <= {i2c_shift[6:0], 1'b0};
                                i2c_sda_out <= i2c_shift[6];
                            end else begin
                                i2c_driving <= 0;
                                i2c_state <= S_DATA_ACK;
                                i2c_bit <= 0;
                            end
                        end
                    end
                    
                    S_DATA_ACK: begin
                        if (scl_fall) begin
                            if (!sda_val) begin
                                i2c_reg_addr <= i2c_reg_addr + 1;
                                i2c_shift <= get_mag_data(i2c_reg_addr + 1);
                                i2c_state <= S_DATA_OUT;
                                i2c_driving <= 1;
                                i2c_sda_out <= get_mag_data(i2c_reg_addr + 1) >> 7;
                            end else begin
                                i2c_state <= S_IDLE;
                            end
                        end
                    end
                    
                    default: i2c_state <= S_IDLE;
                endcase
            end
        end
    end

    // Device Under Test
    top uut(
        .clk(clk),
        .reset(reset),
        .ACL_MISO(ACL_MISO),
        .ACL_SCLK(ACL_SCLK),
        .ACL_MOSI(ACL_MOSI),
        .ACL_CSN(ACL_CSN),
        .sda(sda),
        .scl(scl),
        .seg(seg),
        .an(an),
        .LED(LED)
    );

    // Debug signals
    reg signed [15:0] dbg_mag_x_raw = uut.heading_inst.mag_x_raw;
    reg signed [15:0] dbg_mag_y_raw = uut.heading_inst.mag_y_raw;
    reg dbg_mag_err = uut.mag_error;
    reg dbg_mag_busy = uut.mag_busy;
    reg [8:0] dbg_heading = uut.heading;

    // Test tasks
    task check_heading(input [8:0] expected, input [8:0] tolerance, input [255:0] dir_name);
        reg [8:0] actual, diff;
        begin
            test_num = test_num + 1;
            actual = uut.heading;
            diff = (actual > expected) ? (actual - expected) : (expected - actual);
            // Handle wrap-around for 0/360 boundary
            if (diff > 180) diff = 360 - diff;
            
            if (diff <= tolerance) begin
                tests_passed = tests_passed + 1;
                $display("[PASS] Test %0d: %s  Heading=%0d° (expected=%0d°±%0d°)", 
                         test_num, dir_name, actual, expected, tolerance);
            end else begin
                tests_failed = tests_failed + 1;
                $display("[FAIL] Test %0d: %s  Heading=%0d° (expected=%0d°±%0d°) [sim_mag: x=%0d y=%0d] [raw: x=%0d y=%0d]", 
                         test_num, dir_name, actual, expected, tolerance,
                         $signed(sim_mag_x), $signed(sim_mag_y),
                         $signed(dbg_mag_x_raw), $signed(dbg_mag_y_raw));
            end
        end
    endtask

    task wait_for_stable_heading;
        integer stable_count;
        reg [8:0] last_heading;
        integer timeout;
        begin
            stable_count = 0;
            timeout = 0;
            last_heading = uut.heading;
            
            while (stable_count < 2_000_000 && timeout < 100_000_000) begin
                @(posedge clk);
                timeout = timeout + 1;
                if (uut.heading == last_heading) begin
                    stable_count = stable_count + 1;
                end else begin
                    stable_count = 0;
                    last_heading = uut.heading;
                end
            end
            
            if (timeout >= 100_000_000) begin
                $display("[WARNING] Timeout waiting for stable heading, using current value: %0d°", uut.heading);
            end
            
            #10_000_000;
        end
    endtask

    // Main test sequence
    initial begin
        $display("\nDigital Compass Testbench");

        reset = 1; 
        #1000; 
        reset = 0;
        
        #50_000_000;

        // Test all 16 compass directions
        
        // North (0°)
        sim_mag_x = -16'sd1400; sim_mag_y = 16'sd1600;
        wait_for_stable_heading;
        check_heading(9'd0, 9'd5, "0 - N (Q1 zone 0)");

        // NNE (22°)
        sim_mag_x = -16'sd2100; sim_mag_y = 16'sd900;
        wait_for_stable_heading;
        check_heading(9'd22, 9'd5, "22 -  (Q1 zone 1)");

        // NE (45°)
        sim_mag_x = -16'sd3000; sim_mag_y = 16'sd0;
        wait_for_stable_heading;
        check_heading(9'd45, 9'd5, "45 - NE (Q1 zone 2)");

        // ENE (68°)
        sim_mag_x = -16'sd1750; sim_mag_y = -16'sd750;
        wait_for_stable_heading;
        check_heading(9'd68, 9'd5, "68 -  (Q1 zone 3)");

        // E (90°)
        sim_mag_x = -16'sd1400; sim_mag_y = -16'sd1600;
        wait_for_stable_heading;
        check_heading(9'd90, 9'd5, "90 - E (Q1 zone 4)");

        // ESE (112°)
        sim_mag_x = -16'sd750; sim_mag_y = -16'sd1750;
        wait_for_stable_heading;
        check_heading(9'd112, 9'd5, "112 -  (Q2 zone 3)");

        // SE (135°)
        sim_mag_x = 16'sd0; sim_mag_y = -16'sd3000;
        wait_for_stable_heading;
        check_heading(9'd135, 9'd5, "135 -  (Q2 zone 2)");

        // SSE (158°)
        sim_mag_x = 16'sd900; sim_mag_y = -16'sd2100;
        wait_for_stable_heading;
        check_heading(9'd158, 9'd5, "158 -  (Q2 zone 1)");

        // S (180°)
        sim_mag_x = 16'sd1600; sim_mag_y = -16'sd1400;
        wait_for_stable_heading;
        check_heading(9'd180, 9'd5, "180 - S (Q2 zone 0)");

        // SSW (202°)
        sim_mag_x = 16'sd2100; sim_mag_y = -16'sd900;
        wait_for_stable_heading;
        check_heading(9'd202, 9'd5, "202 -  (Q3 zone 1)");

        // SW (225°)
        sim_mag_x = 16'sd3000; sim_mag_y = 16'sd0;
        wait_for_stable_heading;
        check_heading(9'd225, 9'd5, "225 - SW (Q3 zone 2)");

        // WSW (248°)
        sim_mag_x = 16'sd1750; sim_mag_y = 16'sd750;
        wait_for_stable_heading;
        check_heading(9'd248, 9'd5, "248 -  (Q3 zone 3)");

        // W (270°)
        sim_mag_x = 16'sd1400; sim_mag_y = 16'sd1600;
        wait_for_stable_heading;
        check_heading(9'd270, 9'd5, "270 - W (Q3 zone 4)");

        // WNW (292°)
        sim_mag_x = 16'sd750; sim_mag_y = 16'sd1750;
        wait_for_stable_heading;
        check_heading(9'd292, 9'd5, "292 -  (Q4 zone 3)");

        // NW (315°)
        sim_mag_x = 16'sd0; sim_mag_y = 16'sd3000;
        wait_for_stable_heading;
        check_heading(9'd315, 9'd5, "315 - NW (Q4 zone 2)");

        // NNW (338°)
        sim_mag_x = -16'sd900; sim_mag_y = 16'sd2100;
        wait_for_stable_heading;
        check_heading(9'd338, 9'd5, "338 -  (Q4 zone 1)");

        // Results
        $display("\n================================");
        $display("Test Results Summary");
        $display("================================");
        $display("  Total Tests: %0d", tests_passed + tests_failed);
        $display("  Passed:      %0d", tests_passed);
        $display("  Failed:      %0d", tests_failed);
        if (tests_failed == 0)
            $display("All tests passed");
        else
            $display("Some tests failed");

        #10000;
        $finish;
    end

    // Timeout watchdog
    initial begin
        #2000_000_000;
        $display("\n[ERROR] Testbench timeout!");
        $finish;
    end

endmodule
