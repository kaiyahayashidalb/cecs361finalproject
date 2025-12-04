`timescale 1ns / 1ps
//==============================================================================
// Magnetometer Driver Testbench - Simplified
// Simple I2C slave that ACKs everything and returns programmed data
//==============================================================================
module mag_driver_tb;

    parameter CLK_HZ = 100_000_000;
    parameter CLK_PERIOD = 10;  // 100MHz
    
    //==========================================================================
    // DUT signals
    //==========================================================================
    reg clk = 0;
    reg rst = 1;
    reg start_read = 0;
    
    wire data_valid;
    wire busy;
    wire error;
    wire signed [15:0] mag_x, mag_y, mag_z;
    wire [7:0] debug_byte;
    
    // I2C bus - use tri1 for proper open-drain with pullup
    tri1 sda, scl;
    
    //==========================================================================
    // Test tracking
    //==========================================================================
    integer tests_passed = 0;
    integer tests_failed = 0;
    
    //==========================================================================
    // Slave test data (6 bytes for read)
    //==========================================================================
    reg [7:0] slave_read_data [0:5];
    
    //==========================================================================
    // DUT
    //==========================================================================
    magnetometer_driver #(.CLK_HZ(CLK_HZ), .SIM_MODE(1)) dut (
        .clk(clk),
        .rst(rst),
        .start_read(start_read),
        .data_valid(data_valid),
        .busy(busy),
        .error(error),
        .mag_x(mag_x),
        .mag_y(mag_y),
        .mag_z(mag_z),
        .debug_byte(debug_byte),
        .sda(sda),
        .scl(scl)
    );
    
    //==========================================================================
    // Clock generation
    //==========================================================================
    always #(CLK_PERIOD/2) clk = ~clk;
    
    //==========================================================================
    // Simple I2C Slave Model (from working i2c_master_tb)
    //==========================================================================
    
    // Slave enable (disable to test NACK)
    reg slave_enable = 1;
    
    // Slave drives SDA
    reg sda_slave = 1;
    assign sda = (slave_enable && !sda_slave) ? 1'b0 : 1'bz;
    
    // Edge detection for slave
    reg scl_prev = 1;
    wire scl_rise = scl & ~scl_prev;
    wire scl_fall = ~scl & scl_prev;
    always @(posedge clk) scl_prev <= scl;
    
    // Slave state machine
    localparam S_IDLE = 0, S_ADDR = 1, S_ACK = 2, S_DATA = 3, S_WAIT_MACK = 4;
    reg [2:0] state = S_IDLE;
    reg is_read = 0;
    reg [3:0] bit_cnt = 0;
    reg [7:0] shift_reg = 0;
    reg master_ack = 0;
    reg [3:0] byte_cnt = 0;
    reg [2:0] read_idx = 0;  // Index into slave_read_data
    
    // Start/stop detection
    reg sda_prev = 1;
    reg sda_slave_prev = 1;
    always @(posedge clk) begin
        sda_prev <= sda;
        sda_slave_prev <= sda_slave;
    end
    wire start_cond = sda_prev & ~sda & scl;
    wire stop_cond = ~sda_prev & sda & scl & sda_slave & sda_slave_prev;
    
    // Simple slave behavior
    always @(posedge clk) begin
        if (rst || !slave_enable) begin
            state <= S_IDLE;
            sda_slave <= 1;
            bit_cnt <= 0;
            is_read <= 0;
            byte_cnt <= 0;
            read_idx <= 0;
        end
        else if (start_cond) begin
            state <= S_ADDR;
            bit_cnt <= 0;
            sda_slave <= 1;
            is_read <= 0;
            byte_cnt <= 0;
            read_idx <= 0;
        end
        else if (stop_cond && state != S_IDLE) begin
            state <= S_IDLE;
            sda_slave <= 1;
            is_read <= 0;
            byte_cnt <= 0;
        end
        else if (scl_rise) begin
            case (state)
                S_ADDR: begin
                    shift_reg <= {shift_reg[6:0], sda};
                    bit_cnt <= bit_cnt + 1;
                end
                S_WAIT_MACK: begin
                    master_ack <= ~sda;
                end
            endcase
        end
        else if (scl_fall) begin
            case (state)
                S_ADDR: begin
                    if (bit_cnt == 8) begin
                        if (byte_cnt == 0)
                            is_read <= shift_reg[0];
                        sda_slave <= 0;  // ACK
                        state <= S_ACK;
                        bit_cnt <= 0;
                        byte_cnt <= byte_cnt + 1;
                    end
                end
                S_ACK: begin
                    if (is_read) begin
                        sda_slave <= slave_read_data[read_idx][7];
                        shift_reg <= {slave_read_data[read_idx][6:0], 1'b0};
                        bit_cnt <= 1;
                        state <= S_DATA;
                    end else begin
                        sda_slave <= 1;
                        bit_cnt <= 0;
                        state <= S_ADDR;
                    end
                end
                S_DATA: begin
                    if (bit_cnt == 8) begin
                        sda_slave <= 1;
                        state <= S_WAIT_MACK;
                        bit_cnt <= 0;
                        read_idx <= read_idx + 1;
                    end else begin
                        sda_slave <= shift_reg[7];
                        shift_reg <= {shift_reg[6:0], 1'b0};
                        bit_cnt <= bit_cnt + 1;
                    end
                end
                S_WAIT_MACK: begin
                    if (master_ack) begin
                        sda_slave <= slave_read_data[read_idx][7];
                        shift_reg <= {slave_read_data[read_idx][6:0], 1'b0};
                        bit_cnt <= 1;
                        state <= S_DATA;
                    end else begin
                        sda_slave <= 1;
                        state <= S_IDLE;
                        is_read <= 0;
                    end
                end
            endcase
        end
    end

    //==========================================================================
    // Test Tasks
    //==========================================================================
    
    task check(input [255:0] name, input condition);
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
    
    task do_read(
        input [15:0] exp_x, exp_y, exp_z,
        input [255:0] name
    );
    begin
        // Pulse start_read for a few cycles
        start_read = 1;
        @(posedge clk);
        @(posedge clk);
        start_read = 0;
        
        // Wait for busy to go high
        @(posedge clk);
        if (!busy) begin
            $display("  FAIL: %0s - busy never went high", name);
            tests_failed = tests_failed + 1;
        end else begin
            // Wait for completion
            wait(!busy);
            @(posedge clk);
            @(posedge clk);
            
            if (error) begin
                $display("  FAIL: %0s - error flag set", name);
                tests_failed = tests_failed + 1;
            end else if (mag_x == exp_x && mag_y == exp_y && mag_z == exp_z) begin
                $display("  PASS: %0s (X=%04X Y=%04X Z=%04X)", name, mag_x, mag_y, mag_z);
                tests_passed = tests_passed + 1;
            end else begin
                $display("  FAIL: %0s", name);
                $display("    Expected: X=%04X Y=%04X Z=%04X", exp_x, exp_y, exp_z);
                $display("    Got:      X=%04X Y=%04X Z=%04X", mag_x, mag_y, mag_z);
                tests_failed = tests_failed + 1;
            end
        end
        #1000;
    end
    endtask

    //==========================================================================
    // Main Test
    //==========================================================================
    integer i;
    
    initial begin
        $dumpfile("mag_driver_tb.vcd");
        $dumpvars(0, mag_driver_tb);
        
        // Initialize slave data
        for (i = 0; i < 6; i = i + 1)
            slave_read_data[i] = 8'h00;
        
        // Reset
        rst = 1;
        #100;
        rst = 0;
        #100;
        
        $display("\n============================================");
        $display("Magnetometer Driver Testbench");
        $display("============================================\n");
        
        //----------------------------------------------------------------------
        $display("TEST 1: Reset state");
        check("busy=0 after reset", busy == 0);
        check("error=0 after reset", error == 0);
        
        //----------------------------------------------------------------------
        $display("\nTEST 2: First read (init + read)");
        // X=0x1234, Y=0x5678, Z=0x9ABC
        slave_read_data[0] = 8'h34;  // X_LSB
        slave_read_data[1] = 8'h12;  // X_MSB
        slave_read_data[2] = 8'h78;  // Y_LSB
        slave_read_data[3] = 8'h56;  // Y_MSB
        slave_read_data[4] = 8'hBC;  // Z_LSB
        slave_read_data[5] = 8'h9A;  // Z_MSB
        do_read(16'h1234, 16'h5678, 16'h9ABC, "Init + Read");
        
        //----------------------------------------------------------------------
        $display("\nTEST 3: Second read (no init)");
        slave_read_data[0] = 8'hBB;
        slave_read_data[1] = 8'hAA;
        slave_read_data[2] = 8'hDD;
        slave_read_data[3] = 8'hCC;
        slave_read_data[4] = 8'hFF;
        slave_read_data[5] = 8'hEE;
        do_read(16'hAABB, 16'hCCDD, 16'hEEFF, "Read AABB/CCDD/EEFF");
        
        //----------------------------------------------------------------------
        $display("\nTEST 4: All zeros");
        for (i = 0; i < 6; i = i + 1)
            slave_read_data[i] = 8'h00;
        do_read(16'h0000, 16'h0000, 16'h0000, "Read zeros");
        
        //----------------------------------------------------------------------
        $display("\nTEST 5: All ones");
        for (i = 0; i < 6; i = i + 1)
            slave_read_data[i] = 8'hFF;
        do_read(16'hFFFF, 16'hFFFF, 16'hFFFF, "Read all 1s");
        
        //----------------------------------------------------------------------
        $display("\nTEST 6: NACK test");
        slave_enable = 0;
        start_read = 1;
        @(posedge clk);
        @(posedge clk);
        start_read = 0;
        @(posedge clk);
        if (busy) begin
            wait(!busy);
            @(posedge clk);
        end
        check("NACK sets error", error == 1);
        slave_enable = 1;
        
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
        #50_000_000;
        $display("\n*** TIMEOUT ***");
        $display("DEBUG: state=%d, i2c_busy=%b, i2c_done=%b, error=%b", 
                 dut.state, dut.i2c_busy, dut.i2c_done, dut.i2c_error);
        $display("DEBUG: slv_state=%d, bit_cnt=%d, byte_cnt=%d", state, bit_cnt, byte_cnt);
        $finish;
    end

endmodule
