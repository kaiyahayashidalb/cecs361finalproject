module i2c_master_tb;

    //==========================================================================
    // Parameters
    //==========================================================================
    parameter CLK_PERIOD = 10;  // 100MHz
    
    //==========================================================================
    // Signals
    //==========================================================================
    reg         clk = 0;
    reg         rst = 1;
    reg         start = 0;
    reg  [6:0]  device_addr = 7'h30;
    reg  [7:0]  reg_addr = 0;
    reg         rw = 0;
    reg  [2:0]  num_bytes = 1;
    reg  [7:0]  wr_data = 0;
    
    wire [7:0]  rd_data;
    wire        rd_valid;
    wire        busy;
    wire        done;
    wire        error;
    wire        sda, scl;
    
    //==========================================================================
    // Test tracking
    //==========================================================================
    integer tests_passed = 0;
    integer tests_failed = 0;
    reg [7:0] captured_bytes [0:7];
    integer capture_count = 0;
    
    //==========================================================================
    // DUT
    //==========================================================================
    i2c_master #(
        .CLK_HZ(100_000_000),
        .I2C_HZ(400_000)
    ) dut (
        .clk(clk),
        .rst(rst),
        .start(start),
        .device_addr(device_addr),
        .reg_addr(reg_addr),
        .rw(rw),
        .num_bytes(num_bytes),
        .wr_data(wr_data),
        .rd_data(rd_data),
        .rd_valid(rd_valid),
        .busy(busy),
        .done(done),
        .error(error),
        .sda(sda),
        .scl(scl)
    );
    
    //==========================================================================
    // Clock generation
    //==========================================================================
    always #(CLK_PERIOD/2) clk = ~clk;
    
    //==========================================================================
    // Capture received bytes
    //==========================================================================
    always @(posedge clk) begin
        if (rst) begin
            capture_count <= 0;
        end else if (rd_valid) begin
            captured_bytes[capture_count] <= rd_data;
            capture_count <= capture_count + 1;
            $display("  -> Received byte[%0d] = 0x%02X", capture_count, rd_data);
        end else if (start) begin
            capture_count <= 0;
        end
    end

    //==========================================================================
    // SIMPLE I2C SLAVE MODEL
    //
    // Key design: responds immediately and predictably
    // - Configurable address
    // - Configurable read data (set before each test)
    // - Records written data for verification
    //==========================================================================
    
    // Slave configuration
    reg         slave_enable = 1;
    reg  [6:0]  slave_addr = 7'h30;
    reg  [7:0]  slave_read_data [0:7];  // What slave will return on reads
    reg  [7:0]  slave_write_data [0:7]; // What slave received on writes
    reg  [7:0]  slave_reg_addr;         // Register address received
    integer     slave_write_count = 0;
    
    // Slave internal state
    reg         slv_sda = 1;            // 1=release, 0=pull low
    reg  [3:0]  slv_state = 0;
    reg  [7:0]  slv_shifter = 0;
    reg  [3:0]  slv_bitcnt = 0;
    reg         slv_rw_bit = 0;
    reg  [2:0]  slv_byte_num = 0;
    reg         slv_got_addr = 0;
    reg         slv_tx_bit = 1;         // Registered TX bit for PUT_BYTE
    reg         slv_ack_bit = 0;        // Sampled ACK/NACK from master
    
    // Edge detection - directly on wire values (no registration delay)
    wire        scl_val = (scl === 0) ? 0 : 1;
    wire        sda_val = (sda === 0) ? 0 : 1;
    
    // Registered previous values for edge detection
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
    
    // SDA driver - use registered slv_tx_bit for PUT_BYTE state
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
            // START condition resets slave
            if (start_cond) begin
                slv_state <= SLV_GET_BYTE;
                slv_bitcnt <= 0;
                slv_shifter <= 0;
                slv_sda <= 1;
                slv_got_addr <= 0;
                slv_byte_num <= 0;
                slv_tx_bit <= 1;  // Release SDA
                slv_rw_bit <= 0;  // Reset R/W bit
                slv_ack_bit <= 0;
            end
            // STOP condition - fully reset slave state
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
                    
                    //----------------------------------------------------------
                    // Receive a byte (address or data)
                    //----------------------------------------------------------
                    SLV_GET_BYTE: begin
                        if (scl_rise_now) begin
                            slv_shifter <= {slv_shifter[6:0], sda_val};
                            slv_bitcnt <= slv_bitcnt + 1;
                        end
                        
                        if (scl_fall_now && slv_bitcnt == 8) begin
                            // Got 8 bits - decide whether to ACK
                            if (!slv_got_addr) begin
                                // This is address byte
                                slv_got_addr <= 1;
                                slv_rw_bit <= slv_shifter[0];
                                if (slv_shifter[7:1] == slave_addr) begin
                                    slv_sda <= 0;  // ACK
                                end else begin
                                    slv_sda <= 1;  // NACK - wrong address
                                end
                            end else begin
                                // This is data byte (register addr or write data)
                                // slv_byte_num tracks data bytes: 0=reg_addr, 1+=write_data
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
                    
                    //----------------------------------------------------------
                    // Hold ACK, then transition
                    //----------------------------------------------------------
                    SLV_SEND_ACK: begin
                        if (scl_fall_now) begin
                            if (slv_rw_bit && slv_got_addr) begin
                                // Read mode after address - send data
                                // Pre-load the first bit (bit 7) NOW so it's ready
                                slv_state <= SLV_PUT_BYTE;
                                slv_shifter <= slave_read_data[0];
                                slv_bitcnt <= 0;
                                slv_byte_num <= 0;  // Reset for read byte tracking
                                slv_tx_bit <= slave_read_data[0][7];  // Pre-load bit 7!
                            end else begin
                                // Write mode - receive more data (keep slv_byte_num as is)
                                slv_sda <= 1;  // Release
                                slv_shifter <= 0;
                                slv_bitcnt <= 0;
                                slv_state <= SLV_GET_BYTE;
                            end
                        end
                    end
                    
                    //----------------------------------------------------------
                    // Send a byte to master
                    // slv_tx_bit is registered and updated on SCL fall
                    // bitcnt=0 means bit 7 is on SDA, bitcnt=7 means bit 0
                    //----------------------------------------------------------
                    SLV_PUT_BYTE: begin
                        if (scl_fall_now) begin
                            if (slv_bitcnt == 7) begin
                                // All 8 bits sent (0-7), release for master ACK
                                slv_sda <= 1;
                                slv_state <= SLV_GET_ACK;
                                slv_bitcnt <= 0;
                            end else begin
                                // Advance to next bit - pre-load next bit into slv_tx_bit
                                slv_bitcnt <= slv_bitcnt + 1;
                                slv_tx_bit <= slv_shifter[6 - slv_bitcnt];  // Next bit (7-bitcnt-1 = 6-bitcnt)
                            end
                        end
                    end
                    
                    //----------------------------------------------------------
                    // Wait for master ACK/NACK
                    // ACK = SDA low (master wants more data)
                    // NACK = SDA high (master done, will send STOP)
                    //----------------------------------------------------------
                    SLV_GET_ACK: begin
                        if (scl_rise_now) begin
                            // Sample ACK/NACK on rising edge
                            slv_ack_bit <= sda_val;  // 0=ACK, 1=NACK
                        end
                        if (scl_fall_now) begin
                            if (slv_ack_bit) begin
                                // NACK - master is done, go to idle and wait for STOP
                                slv_state <= SLV_IDLE;
                                slv_sda <= 1;
                                slv_tx_bit <= 1;
                            end else begin
                                // ACK - master wants more data
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
    
    task do_write(input [6:0] addr, input [7:0] regad, input [7:0] data);
    begin
        @(posedge clk);
        device_addr = addr;
        reg_addr = regad;
        wr_data = data;
        rw = 0;
        num_bytes = 1;
        start = 1;
        @(posedge clk);
        start = 0;
        wait(done);
        @(posedge clk);
    end
    endtask
    
    task do_read(input [6:0] addr, input [7:0] regad, input [2:0] nbytes);
        integer j;
    begin
        // Clear captured bytes before each read
        for (j = 0; j < 8; j = j + 1) begin
            captured_bytes[j] = 8'hXX;
        end
        @(posedge clk);
        device_addr = addr;
        reg_addr = regad;
        rw = 1;
        num_bytes = nbytes;
        start = 1;
        @(posedge clk);
        start = 0;
        wait(done);
        @(posedge clk);
    end
    endtask

    //==========================================================================
    // Main test sequence
    //==========================================================================
    integer i;
    
    initial begin
        $dumpfile("i2c_master_tb.vcd");
        $dumpvars(0, i2c_master_tb);
        
        // Initialize
        for (i = 0; i < 8; i = i + 1) begin
            slave_read_data[i] = 8'h00;
            slave_write_data[i] = 8'h00;
            captured_bytes[i] = 8'h00;
        end
        
        // Reset
        rst = 1;
        #100;
        rst = 0;
        #100;
        
        $display("\n============================================");
        $display("I2C Master Testbench");
        $display("============================================\n");
        
        //----------------------------------------------------------------------
        $display("TEST 1: Reset state");
        //----------------------------------------------------------------------
        check("busy=0", busy == 0);
        check("done=0", done == 0);
        check("error=0", error == 0);
        
        //----------------------------------------------------------------------
        $display("\nTEST 2: Single byte write");
        //----------------------------------------------------------------------
        slave_addr = 7'h30;
        do_write(7'h30, 8'h09, 8'hAB);
        check("no error", error == 0);
        check("reg_addr=0x09", slave_reg_addr == 8'h09);
        check("data=0xAB", slave_write_data[0] == 8'hAB);
        #200;
        
        //----------------------------------------------------------------------
        $display("\nTEST 3: Single byte read");
        //----------------------------------------------------------------------
        slave_read_data[0] = 8'h42;
        do_read(7'h30, 8'h00, 3'd1);
        check("no error", error == 0);
        check("read 0x42", captured_bytes[0] == 8'h42);
        #200;
        
        //----------------------------------------------------------------------
        $display("\nTEST 4: Multi-byte read (6 bytes - magnetometer XYZ)");
        //----------------------------------------------------------------------
        slave_read_data[0] = 8'hDE;
        slave_read_data[1] = 8'hAD;
        slave_read_data[2] = 8'hBE;
        slave_read_data[3] = 8'hEF;
        slave_read_data[4] = 8'hCA;
        slave_read_data[5] = 8'hFE;
        do_read(7'h30, 8'h00, 3'd6);
        check("no error", error == 0);
        check("byte0=0xDE", captured_bytes[0] == 8'hDE);
        check("byte1=0xAD", captured_bytes[1] == 8'hAD);
        check("byte2=0xBE", captured_bytes[2] == 8'hBE);
        check("byte3=0xEF", captured_bytes[3] == 8'hEF);
        check("byte4=0xCA", captured_bytes[4] == 8'hCA);
        check("byte5=0xFE", captured_bytes[5] == 8'hFE);
        #200;
        
        //----------------------------------------------------------------------
        $display("\nTEST 5: NACK detection (wrong address)");
        //----------------------------------------------------------------------
        slave_addr = 7'h30;
        do_write(7'h50, 8'h00, 8'h00);  // Wrong address
        check("error=1", error == 1);
        #200;
        
        //----------------------------------------------------------------------
        $display("\nTEST 6: NACK detection (slave disabled)");
        //----------------------------------------------------------------------
        slave_enable = 0;
        do_read(7'h30, 8'h00, 3'd1);
        check("error=1", error == 1);
        slave_enable = 1;
        #200;
        
        //----------------------------------------------------------------------
        $display("\nTEST 7: All-zeros pattern");
        //----------------------------------------------------------------------
        slave_read_data[0] = 8'h00;
        slave_read_data[1] = 8'h00;
        do_read(7'h30, 8'h00, 3'd2);
        check("no error", error == 0);
        check("byte0=0x00", captured_bytes[0] == 8'h00);
        check("byte1=0x00", captured_bytes[1] == 8'h00);
        #200;
        
        //----------------------------------------------------------------------
        $display("\nTEST 8: All-ones pattern");
        //----------------------------------------------------------------------
        slave_read_data[0] = 8'hFF;
        slave_read_data[1] = 8'hFF;
        do_read(7'h30, 8'h00, 3'd2);
        check("no error", error == 0);
        check("byte0=0xFF", captured_bytes[0] == 8'hFF);
        check("byte1=0xFF", captured_bytes[1] == 8'hFF);
        #200;
        
        //----------------------------------------------------------------------
        $display("\nTEST 9: Alternating pattern");
        //----------------------------------------------------------------------
        slave_read_data[0] = 8'hAA;
        slave_read_data[1] = 8'h55;
        slave_read_data[2] = 8'hAA;
        do_read(7'h30, 8'h00, 3'd3);
        check("no error", error == 0);
        check("byte0=0xAA", captured_bytes[0] == 8'hAA);
        check("byte1=0x55", captured_bytes[1] == 8'h55);
        check("byte2=0xAA", captured_bytes[2] == 8'hAA);
        #200;
        
        //----------------------------------------------------------------------
        $display("\nTEST 10: Different slave address");
        //----------------------------------------------------------------------
        slave_addr = 7'h50;
        slave_read_data[0] = 8'h77;
        do_read(7'h50, 8'h00, 3'd1);
        check("no error", error == 0);
        check("byte0=0x77", captured_bytes[0] == 8'h77);
        slave_addr = 7'h30;
        #200;
        
        //----------------------------------------------------------------------
        $display("\nTEST 11: Back-to-back transactions");
        //----------------------------------------------------------------------
        slave_read_data[0] = 8'h11;
        do_read(7'h30, 8'h00, 3'd1);
        check("txn1 ok", error == 0 && captured_bytes[0] == 8'h11);
        
        slave_read_data[0] = 8'h22;
        do_read(7'h30, 8'h00, 3'd1);
        check("txn2 ok", error == 0 && captured_bytes[0] == 8'h22);
        
        slave_read_data[0] = 8'h33;
        do_read(7'h30, 8'h00, 3'd1);
        check("txn3 ok", error == 0 && captured_bytes[0] == 8'h33);
        #200;
        
        //----------------------------------------------------------------------
        $display("\nTEST 12: Max byte count (7)");
        //----------------------------------------------------------------------
        slave_read_data[0] = 8'h01;
        slave_read_data[1] = 8'h02;
        slave_read_data[2] = 8'h03;
        slave_read_data[3] = 8'h04;
        slave_read_data[4] = 8'h05;
        slave_read_data[5] = 8'h06;
        slave_read_data[6] = 8'h07;
        do_read(7'h30, 8'h00, 3'd7);
        check("no error", error == 0);
        check("byte0=0x01", captured_bytes[0] == 8'h01);
        check("byte6=0x07", captured_bytes[6] == 8'h07);
        #200;
        
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
