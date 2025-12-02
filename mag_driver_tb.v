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
    wire sda, scl;
    
    //==========================================================================
    // Test tracking
    //==========================================================================
    integer tests_passed = 0;
    integer tests_failed = 0;
    
    // Test data storage (6 bytes: X_LSB, X_MSB, Y_LSB, Y_MSB, Z_LSB, Z_MSB)
    reg [7:0] test_data [0:7];
    
    //==========================================================================
    // DUT
    //==========================================================================
    magnetometer_driver #(.CLK_HZ(CLK_HZ)) dut (
        .clk(clk),
        .rst(rst),
        .start_read(start_read),
        .data_valid(data_valid),
        .busy(busy),
        .error(error),
        .mag_x(mag_x),
        .mag_y(mag_y),
        .mag_z(mag_z),
        .sda(sda),
        .scl(scl)
    );
    
    //==========================================================================
    // Clock generation
    //==========================================================================
    always #(CLK_PERIOD/2) clk = ~clk;
    
    //==========================================================================
    // I2C SLAVE MODEL (from working i2c_master_tb)
    //==========================================================================
    
    // Slave configuration
    reg         slave_enable = 1;
    reg  [6:0]  slave_addr = 7'h30;
    reg  [7:0]  slave_read_data [0:7];
    reg  [7:0]  slave_write_data [0:7];
    reg  [7:0]  slave_reg_addr;
    integer     slave_write_count = 0;
    
    // Slave internal state
    reg         slv_sda = 1;
    reg  [3:0]  slv_state = 0;
    reg  [7:0]  slv_shifter = 0;
    reg  [3:0]  slv_bitcnt = 0;
    reg         slv_rw_bit = 0;
    reg  [2:0]  slv_byte_num = 0;
    reg         slv_got_addr = 0;
    reg         slv_tx_bit = 1;
    reg         slv_ack_bit = 0;
    
    // Edge detection
    wire        scl_val = (scl === 0) ? 0 : 1;
    wire        sda_val = (sda === 0) ? 0 : 1;
    
    reg         scl_prev = 1, sda_prev = 1;
    wire        scl_rise_now = !scl_prev && scl_val;
    wire        scl_fall_now = scl_prev && !scl_val;
    wire        start_cond = sda_prev && !sda_val && scl_val;
    wire        stop_cond = !sda_prev && sda_val && scl_val;
    
    // Slave states
    localparam SLV_IDLE     = 0;
    localparam SLV_GET_BYTE = 1;
    localparam SLV_SEND_ACK = 2;
    localparam SLV_PUT_BYTE = 3;
    localparam SLV_GET_ACK  = 4;
    
    // SDA driver
    assign sda = (!slave_enable) ? 1'bz :
                 (slv_state == SLV_PUT_BYTE) ? (slv_tx_bit ? 1'bz : 1'b0) :
                 (slv_sda == 0) ? 1'b0 : 1'bz;
    
    always @(posedge clk) begin
        scl_prev <= scl_val;
        sda_prev <= sda_val;
        
        if (rst || !slave_enable) begin
            slv_state <= SLV_IDLE;
            slv_sda <= 1;
            slv_shifter <= 0;
            slv_bitcnt <= 0;
            slv_rw_bit <= 0;
            slv_byte_num <= 0;
            slv_got_addr <= 0;
            slv_tx_bit <= 1;
            slv_ack_bit <= 0;
        end else begin
            if (start_cond) begin
                slv_state <= SLV_GET_BYTE;
                slv_bitcnt <= 0;
                slv_shifter <= 0;
                slv_sda <= 1;
                slv_got_addr <= 0;
                slv_byte_num <= 0;
                slv_tx_bit <= 1;
                slv_rw_bit <= 0;
                slv_ack_bit <= 0;
            end
            else if (stop_cond) begin
                slv_state <= SLV_IDLE;
                slv_sda <= 1;
                slv_bitcnt <= 0;
                slv_shifter <= 0;
                slv_got_addr <= 0;
                slv_byte_num <= 0;
                slv_tx_bit <= 1;
                slv_rw_bit <= 0;
                slv_ack_bit <= 0;
            end
            else begin
                case (slv_state)
                    SLV_IDLE: begin
                        slv_sda <= 1;
                    end
                    
                    SLV_GET_BYTE: begin
                        if (scl_rise_now) begin
                            slv_shifter <= {slv_shifter[6:0], sda_val};
                            slv_bitcnt <= slv_bitcnt + 1;
                        end
                        
                        if (scl_fall_now && slv_bitcnt == 8) begin
                            if (!slv_got_addr) begin
                                slv_got_addr <= 1;
                                slv_rw_bit <= slv_shifter[0];
                                if (slv_shifter[7:1] == slave_addr) begin
                                    slv_sda <= 0;  // ACK
                                end else begin
                                    slv_sda <= 1;  // NACK
                                end
                            end else begin
                                if (slv_byte_num == 0) begin
                                    slave_reg_addr <= slv_shifter;
                                end else begin
                                    slave_write_data[slv_byte_num-1] <= slv_shifter;
                                    slave_write_count <= slv_byte_num;
                                end
                                slv_byte_num <= slv_byte_num + 1;
                                slv_sda <= 0;  // ACK
                            end
                            slv_state <= SLV_SEND_ACK;
                        end
                    end
                    
                    SLV_SEND_ACK: begin
                        if (scl_fall_now) begin
                            if (slv_rw_bit && slv_got_addr) begin
                                slv_state <= SLV_PUT_BYTE;
                                slv_shifter <= slave_read_data[0];
                                slv_bitcnt <= 0;
                                slv_byte_num <= 0;
                                slv_tx_bit <= slave_read_data[0][7];
                            end else begin
                                slv_sda <= 1;
                                slv_shifter <= 0;
                                slv_bitcnt <= 0;
                                slv_state <= SLV_GET_BYTE;
                            end
                        end
                    end
                    
                    SLV_PUT_BYTE: begin
                        if (scl_fall_now) begin
                            if (slv_bitcnt == 7) begin
                                slv_sda <= 1;
                                slv_state <= SLV_GET_ACK;
                                slv_bitcnt <= 0;
                            end else begin
                                slv_bitcnt <= slv_bitcnt + 1;
                                slv_tx_bit <= slv_shifter[6 - slv_bitcnt];
                            end
                        end
                    end
                    
                    SLV_GET_ACK: begin
                        if (scl_rise_now) begin
                            slv_ack_bit <= sda_val;
                        end
                        if (scl_fall_now) begin
                            if (slv_ack_bit) begin
                                slv_state <= SLV_IDLE;
                                slv_sda <= 1;
                                slv_tx_bit <= 1;
                            end else begin
                                slv_shifter <= slave_read_data[slv_byte_num + 1];
                                slv_tx_bit <= slave_read_data[slv_byte_num + 1][7];
                                slv_byte_num <= slv_byte_num + 1;
                                slv_bitcnt <= 0;
                                slv_state <= SLV_PUT_BYTE;
                            end
                        end
                    end
                endcase
            end
        end
    end

    //==========================================================================
    // Test helper tasks
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
    
    task test_read(
        input [255:0] name,
        input [15:0] exp_x, exp_y, exp_z
    );
    integer timeout;
    begin
        start_read = 1;
        @(posedge clk);
        #1;
        start_read = 0;
        
        // Wait for busy with timeout
        timeout = 0;
        while (!busy && timeout < 100) begin
            @(posedge clk);
            timeout = timeout + 1;
        end
        
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
    // Main test sequence
    //==========================================================================
    integer i;
    
    initial begin
        $dumpfile("mag_driver_tb.vcd");
        $dumpvars(0, mag_driver_tb);
        
        // Initialize
        for (i = 0; i < 8; i = i + 1) begin
            slave_read_data[i] = 8'h00;
            slave_write_data[i] = 8'h00;
        end
        
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
        //----------------------------------------------------------------------
        check("busy=0", busy == 0);
        check("error=0", error == 0);
        
        //----------------------------------------------------------------------
        $display("\nTEST 2: First read (triggers initialization)");
        // Data order: X_LSB, X_MSB, Y_LSB, Y_MSB, Z_LSB, Z_MSB
        //----------------------------------------------------------------------
        slave_read_data[0] = 8'h34;  // X_LSB
        slave_read_data[1] = 8'h12;  // X_MSB -> X = 0x1234
        slave_read_data[2] = 8'h78;  // Y_LSB
        slave_read_data[3] = 8'h56;  // Y_MSB -> Y = 0x5678
        slave_read_data[4] = 8'hBC;  // Z_LSB
        slave_read_data[5] = 8'h9A;  // Z_MSB -> Z = 0x9ABC
        test_read("Init+Read", 16'h1234, 16'h5678, 16'h9ABC);
        
        //----------------------------------------------------------------------
        $display("\nTEST 3: Second read (no init needed)");
        //----------------------------------------------------------------------
        slave_read_data[0] = 8'hBB;  // X_LSB
        slave_read_data[1] = 8'hAA;  // X_MSB -> X = 0xAABB
        slave_read_data[2] = 8'hDD;  // Y_LSB
        slave_read_data[3] = 8'hCC;  // Y_MSB -> Y = 0xCCDD
        slave_read_data[4] = 8'hFF;  // Z_LSB
        slave_read_data[5] = 8'hEE;  // Z_MSB -> Z = 0xEEFF
        test_read("Read AABB/CCDD/EEFF", 16'hAABB, 16'hCCDD, 16'hEEFF);
        
        //----------------------------------------------------------------------
        $display("\nTEST 4: All zeros");
        //----------------------------------------------------------------------
        slave_read_data[0] = 8'h00;
        slave_read_data[1] = 8'h00;
        slave_read_data[2] = 8'h00;
        slave_read_data[3] = 8'h00;
        slave_read_data[4] = 8'h00;
        slave_read_data[5] = 8'h00;
        test_read("Read 0000/0000/0000", 16'h0000, 16'h0000, 16'h0000);
        
        //----------------------------------------------------------------------
        $display("\nTEST 5: All ones");
        //----------------------------------------------------------------------
        slave_read_data[0] = 8'hFF;
        slave_read_data[1] = 8'hFF;
        slave_read_data[2] = 8'hFF;
        slave_read_data[3] = 8'hFF;
        slave_read_data[4] = 8'hFF;
        slave_read_data[5] = 8'hFF;
        test_read("Read FFFF/FFFF/FFFF", 16'hFFFF, 16'hFFFF, 16'hFFFF);
        
        //----------------------------------------------------------------------
        $display("\nTEST 6: Alternating pattern");
        //----------------------------------------------------------------------
        slave_read_data[0] = 8'h55;  // X_LSB
        slave_read_data[1] = 8'hAA;  // X_MSB -> X = 0xAA55
        slave_read_data[2] = 8'hAA;  // Y_LSB
        slave_read_data[3] = 8'h55;  // Y_MSB -> Y = 0x55AA
        slave_read_data[4] = 8'h5A;  // Z_LSB
        slave_read_data[5] = 8'hA5;  // Z_MSB -> Z = 0xA55A
        test_read("Read AA55/55AA/A55A", 16'hAA55, 16'h55AA, 16'hA55A);
        
        //----------------------------------------------------------------------
        $display("\nTEST 7: Back-to-back reads");
        //----------------------------------------------------------------------
        slave_read_data[0] = 8'h22;
        slave_read_data[1] = 8'h11;
        slave_read_data[2] = 8'h44;
        slave_read_data[3] = 8'h33;
        slave_read_data[4] = 8'h66;
        slave_read_data[5] = 8'h55;
        test_read("B2B Read 1", 16'h1122, 16'h3344, 16'h5566);
        
        slave_read_data[0] = 8'h88;
        slave_read_data[1] = 8'h77;
        slave_read_data[2] = 8'hAA;
        slave_read_data[3] = 8'h99;
        slave_read_data[4] = 8'hCC;
        slave_read_data[5] = 8'hBB;
        test_read("B2B Read 2", 16'h7788, 16'h99AA, 16'hBBCC);
        
        //----------------------------------------------------------------------
        $display("\nTEST 8: NACK detection (slave disabled)");
        //----------------------------------------------------------------------
        slave_enable = 0;
        start_read = 1;
        @(posedge clk);
        #1;
        start_read = 0;
        @(posedge clk);
        #1;
        // Wait for transaction to complete
        wait(!busy);
        @(posedge clk);
        check("NACK: error=1", error == 1);
        slave_enable = 1;
        #1000;
        
        //----------------------------------------------------------------------
        // Summary
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
        $display("RESULTS: %0d PASS, %0d FAIL (incomplete)", tests_passed, tests_failed);
        $finish;
    end

endmodule
