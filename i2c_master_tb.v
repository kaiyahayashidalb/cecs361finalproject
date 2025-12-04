`timescale 1ns/1ps

// Simple I2C Master Testbench

module i2c_master_tb;

    // Clock and reset
    reg clk = 0;
    reg rst = 1;
    always #5 clk = ~clk;  // 100MHz
    
    // Master control signals
    reg start_txn = 0;
    reg [6:0] device_addr = 7'h30;
    reg [7:0] reg_addr = 8'h00;
    reg rw = 0;
    reg [2:0] num_bytes = 1;
    reg [7:0] wr_data = 8'h00;
    
    wire [7:0] rd_data;
    wire rd_valid, done, error, busy;
    
    // I2C bus - use tri1 for proper open-drain with pullup
    tri1 sda, scl;
    
    // Slave drives SDA
    reg sda_slave = 1;
    assign sda = sda_slave ? 1'bz : 1'b0;
    
    // DUT
    i2c_master #(.CLK_HZ(100_000_000), .I2C_HZ(400_000)) uut (
        .clk(clk),
        .rst(rst),
        .start_txn(start_txn),
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
        .start_cal(1'b0),
        .sda(sda),
        .scl(scl)
    );
    
    // Edge detection for slave
    reg scl_prev = 1;
    wire scl_rise = scl & ~scl_prev;
    wire scl_fall = ~scl & scl_prev;
    always @(posedge clk) scl_prev <= scl;
    
    // Slave state machine
    localparam S_IDLE = 0, S_ADDR = 1, S_ACK = 2, S_DATA = 3, S_WAIT_MACK = 4;
    reg [2:0] state = S_IDLE;
    reg is_read = 0;
    reg [7:0] read_data = 8'hA5;
    reg [3:0] bit_cnt = 0;
    reg [7:0] shift_reg = 0;
    reg master_ack = 0;
    reg [3:0] byte_cnt = 0;  // Track bytes received to know when restart might come
    
    // Start/stop detection - need to filter out false stops caused by slave releasing SDA
    reg sda_prev = 1;
    reg sda_slave_prev = 1;
    always @(posedge clk) begin
        sda_prev <= sda;
        sda_slave_prev <= sda_slave;
    end
    wire start_cond = sda_prev & ~sda & scl;
    // Only detect STOP if slave isn't the one causing the SDA transition
    wire stop_cond = ~sda_prev & sda & scl & sda_slave & sda_slave_prev;
    
    // Simple slave behavior
    always @(posedge clk) begin
        if (rst) begin
            state <= S_IDLE;
            sda_slave <= 1;
            bit_cnt <= 0;
            is_read <= 0;
            byte_cnt <= 0;
        end
        else if (start_cond) begin
            state <= S_ADDR;
            bit_cnt <= 0;
            sda_slave <= 1;
            is_read <= 0;
            byte_cnt <= 0;
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
                    // Sample master's ACK/NACK
                    master_ack <= ~sda;  // ACK=0 (low), NACK=1 (high)
                end
            endcase
        end
        else if (scl_fall) begin
            case (state)
                S_ADDR: begin
                    if (bit_cnt == 8) begin
                        // Only first byte after START contains R/W bit
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
                        // For read: output bit 7 immediately after ACK
                        // The master will sample this on the next SCL high
                        sda_slave <= read_data[7];
                        shift_reg <= {read_data[6:0], 1'b0};
                        bit_cnt <= 1;
                        state <= S_DATA;
                    end else begin
                        // After ACK for write byte, release SDA to allow restart detection
                        sda_slave <= 1;
                        bit_cnt <= 0;
                        state <= S_ADDR;
                    end
                end
                S_DATA: begin
                    if (bit_cnt == 8) begin
                        // All 8 bits have been sampled by master, release SDA for master ACK/NACK
                        sda_slave <= 1;
                        state <= S_WAIT_MACK;
                        bit_cnt <= 0;
                        read_data <= read_data + 1;
                    end else if (bit_cnt < 8) begin
                        // Output next bit from shift register
                        sda_slave <= shift_reg[7];
                        shift_reg <= {shift_reg[6:0], 1'b0};
                        bit_cnt <= bit_cnt + 1;
                    end
                end
                S_WAIT_MACK: begin
                    if (master_ack) begin
                        // Master sent ACK, output first bit of next byte immediately
                        sda_slave <= read_data[7];
                        shift_reg <= {read_data[6:0], 1'b0};
                        bit_cnt <= 1;
                        state <= S_DATA;
                    end else begin
                        // Master sent NACK, done with read
                        sda_slave <= 1;
                        state <= S_IDLE;
                        is_read <= 0;
                    end
                end
            endcase
        end
    end
    
    // Test sequence
    initial begin
        $display("=== I2C Master Test ===");
        
        #100 rst = 0;
        #100;
        
        // Test 1: Single byte write
        $display("\n[TEST 1] Write 0xBE to reg 0x10");
        device_addr = 7'h30;
        reg_addr = 8'h10;
        wr_data = 8'hBE;
        rw = 0;
        num_bytes = 1;
        start_txn = 1;
        @(posedge clk) start_txn = 0;
        
        wait(done || error);
        if (error) $display("  FAIL: Error during write");
        else $display("  PASS: Write completed");
        #1000;
        
        // Test 2: Single byte read
        $display("\n[TEST 2] Read from reg 0x00");
        read_data = 8'hA5;
        reg_addr = 8'h00;
        rw = 1;
        num_bytes = 1;
        start_txn = 1;
        @(posedge clk) start_txn = 0;
        
        wait(done || error);
        if (error) $display("  FAIL: Error during read");
        else if (rd_data == 8'hA5) $display("  PASS: Read 0x%02X", rd_data);
        else $display("  FAIL: Expected 0xA5, got 0x%02X", rd_data);
        #1000;
        
        // Test 3: Read with different data pattern (0x00)
        $display("\n[TEST 3] Read 0x00 pattern");
        read_data = 8'h00;
        reg_addr = 8'h01;
        rw = 1;
        num_bytes = 1;
        start_txn = 1;
        @(posedge clk) start_txn = 0;
        
        wait(done || error);
        if (error) $display("  FAIL: Error during read");
        else if (rd_data == 8'h00) $display("  PASS: Read 0x%02X", rd_data);
        else $display("  FAIL: Expected 0x00, got 0x%02X", rd_data);
        #1000;
        
        // Test 4: Read with different data pattern (0xFF)
        $display("\n[TEST 4] Read 0xFF pattern");
        read_data = 8'hFF;
        reg_addr = 8'h02;
        rw = 1;
        num_bytes = 1;
        start_txn = 1;
        @(posedge clk) start_txn = 0;
        
        wait(done || error);
        if (error) $display("  FAIL: Error during read");
        else if (rd_data == 8'hFF) $display("  PASS: Read 0x%02X", rd_data);
        else $display("  FAIL: Expected 0xFF, got 0x%02X", rd_data);
        #1000;
        
        // Test 5: Read alternating pattern (0x55)
        $display("\n[TEST 5] Read 0x55 pattern");
        read_data = 8'h55;
        reg_addr = 8'h03;
        rw = 1;
        num_bytes = 1;
        start_txn = 1;
        @(posedge clk) start_txn = 0;
        
        wait(done || error);
        if (error) $display("  FAIL: Error during read");
        else if (rd_data == 8'h55) $display("  PASS: Read 0x%02X", rd_data);
        else $display("  FAIL: Expected 0x55, got 0x%02X", rd_data);
        #1000;
        
        // Test 6: Read alternating pattern (0xAA)
        $display("\n[TEST 6] Read 0xAA pattern");
        read_data = 8'hAA;
        reg_addr = 8'h04;
        rw = 1;
        num_bytes = 1;
        start_txn = 1;
        @(posedge clk) start_txn = 0;
        
        wait(done || error);
        if (error) $display("  FAIL: Error during read");
        else if (rd_data == 8'hAA) $display("  PASS: Read 0x%02X", rd_data);
        else $display("  FAIL: Expected 0xAA, got 0x%02X", rd_data);
        #1000;
        
        // Test 7: Back-to-back transactions
        $display("\n[TEST 7] Back-to-back write then read");
        // Write first
        read_data = 8'h42;
        reg_addr = 8'h20;
        wr_data = 8'hCD;
        rw = 0;
        num_bytes = 1;
        start_txn = 1;
        @(posedge clk) start_txn = 0;
        wait(done || error);
        #100;  // Small delay for master to return to idle
        if (error) begin
            $display("  FAIL: Error during write");
        end else begin
            // Now read
            rw = 1;
            start_txn = 1;
            @(posedge clk) start_txn = 0;
            wait(done || error);
            if (error) $display("  FAIL: Error during read");
            else if (rd_data == 8'h42) $display("  PASS: Back-to-back OK");
            else $display("  FAIL: Expected 0x42, got 0x%02X", rd_data);
        end
        #1000;
        
        $display("\n=== All Tests Complete ===");
        $finish;
    end
    
    // Timeout
    initial begin
        #5000000;  // 5ms timeout
        $display("TIMEOUT!");
        $finish;
    end

endmodule
