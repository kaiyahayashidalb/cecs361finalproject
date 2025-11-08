`timescale 1ns / 1ps

module SPI_master_tb();

  // Testbench signals
  parameter CLK_PERIOD = 10; // 100MHz clock
  parameter SPI_MODE = 0;
  parameter CLKS_PER_HALF_BIT = 1; // Faster for simulation (was 2)
  
  // Clock and reset
  reg r_Clk = 0;
  reg r_Rst_L = 1;
  
  // Master signals
  reg [7:0] r_Master_TX_Byte;
  reg r_Master_TX_DV;
  wire w_Master_TX_Ready;
  wire w_Master_RX_DV;
  wire [7:0] w_Master_RX_Byte;
  
  // SPI Interface wires
  wire w_SPI_Clk;
  wire w_SPI_MOSI;
  reg w_SPI_MISO = 0;
  
  // Clock generation
  always #(CLK_PERIOD/2) r_Clk = ~r_Clk;
  
  // Instantiate SPI Master
  SPI_Master #(
    .SPI_MODE(SPI_MODE),
    .CLKS_PER_HALF_BIT(CLKS_PER_HALF_BIT)
  ) UUT_SPI_Master (
    // Control/Data Signals
    .i_Rst_L(r_Rst_L),
    .i_Clk(r_Clk),
    
    // TX (MOSI) Signals
    .i_TX_Byte(r_Master_TX_Byte),
    .i_TX_DV(r_Master_TX_DV),
    .o_TX_Ready(w_Master_TX_Ready),
    
    // RX (MISO) Signals
    .o_RX_DV(w_Master_RX_DV),
    .o_RX_Byte(w_Master_RX_Byte),

    // SPI Interface
    .o_SPI_Clk(w_SPI_Clk),
    .i_SPI_MISO(w_SPI_MISO),
    .o_SPI_MOSI(w_SPI_MOSI)
  );


  // Test tracking variables
  integer test_count;
  integer pass_count;
  integer fail_count;
  integer i; // Loop variable
  
  // Expected data patterns for verification
  reg [7:0] expected_rx_data [0:7];
  reg [7:0] test_tx_data [0:7];
  integer test_index;
  
  // Waveform monitoring signals
  reg [7:0] current_tx_data;
  reg [7:0] current_expected_rx;
  reg [7:0] current_actual_rx;
  reg [7:0] current_test_number;
  reg test_passed;
  reg test_in_progress;
  
  // Initialize test vectors
  initial begin
    test_count = 0;
    pass_count = 0;
    fail_count = 0;
    test_index = 0;
    
    // Initialize monitoring signals
    current_tx_data = 8'h00;
    current_expected_rx = 8'h00;
    current_actual_rx = 8'h00;
    current_test_number = 8'h00;
    test_passed = 1'b0;
    test_in_progress = 1'b0;
    
    // Test data patterns
    test_tx_data[0] = 8'hC1;  expected_rx_data[0] = 8'hAA;
    test_tx_data[1] = 8'hBE;  expected_rx_data[1] = 8'hAB;
    test_tx_data[2] = 8'hEF;  expected_rx_data[2] = 8'hAC;
    test_tx_data[3] = 8'h0B;  expected_rx_data[3] = 8'hAD;
    test_tx_data[4] = 8'h08;  expected_rx_data[4] = 8'hAE;
    test_tx_data[5] = 8'h5A;  expected_rx_data[5] = 8'hAF;
    test_tx_data[6] = 8'h3C;  expected_rx_data[6] = 8'hB0;
    test_tx_data[7] = 8'hFF;  expected_rx_data[7] = 8'hB1;
  end
  
  // MISO data generator with predictable pattern
  reg [7:0] miso_data = 8'hAA;
  reg [2:0] bit_counter = 0;
  
  always @(negedge w_SPI_Clk) begin
    w_SPI_MISO <= miso_data[7-bit_counter];
    bit_counter <= bit_counter + 1;
    if (bit_counter == 7) begin
      miso_data <= miso_data + 1;
    end
  end

  // Task for sending and verifying SPI data
  task SendAndVerify(input [7:0] tx_data, input [7:0] expected_rx);
    reg [7:0] actual_rx;
    begin
      // Update test tracking signals
      test_count = test_count + 1;
      current_test_number = test_count;
      current_tx_data = tx_data;
      current_expected_rx = expected_rx;
      test_in_progress = 1'b1;
      
      // Wait for ready
      while (!w_Master_TX_Ready) @(posedge r_Clk);
      
      // Send byte
      @(posedge r_Clk);
      r_Master_TX_Byte <= tx_data;
      r_Master_TX_DV   <= 1'b1;
      @(posedge r_Clk);
      r_Master_TX_DV <= 1'b0;
      
      // Wait for transmission to complete
      while (w_Master_TX_Ready) @(posedge r_Clk);
      while (!w_Master_TX_Ready) @(posedge r_Clk);
      
      // Check received data
      actual_rx = w_Master_RX_Byte;
      current_actual_rx = actual_rx;
      test_in_progress = 1'b0;
      
      if (actual_rx == expected_rx) begin
        $display("Test %0d: TX=0x%02X, RX=0x%02X - PASSED", test_count, tx_data, actual_rx);
        pass_count = pass_count + 1;
        test_passed = 1'b1;
      end else begin
        $display("Test %0d: TX=0x%02X, RX=0x%02X (Expected 0x%02X) - FAILED", 
                test_count, tx_data, actual_rx, expected_rx);
        fail_count = fail_count + 1;
        test_passed = 1'b0;
      end
    end
  endtask

  // Main test sequence
  initial begin
    // Initialize signals
    r_Master_TX_Byte = 8'h00;
    r_Master_TX_DV = 1'b0;
    
    $display("=== SPI Master Test ===");
    
    // Reset sequence
    r_Rst_L = 1'b0;
    repeat(5) @(posedge r_Clk);
    r_Rst_L = 1'b1;
    repeat(3) @(posedge r_Clk);
    
    $display("Starting tests...");
    
    // Exhaustive test loop
    for (i = 0; i < 8; i = i + 1) begin
      repeat(3) @(posedge r_Clk);
      SendAndVerify(test_tx_data[i], expected_rx_data[i]);
    end
    
    // Edge case tests
    $display("\nTesting edge cases:");
    SendAndVerify(8'h00, 8'hB2);  // All zeros
    SendAndVerify(8'hFF, 8'hB3);  // All ones
    SendAndVerify(8'h55, 8'hB4);  // Alternating pattern 1
    SendAndVerify(8'hAA, 8'hB5);  // Alternating pattern 2
    
    // Back-to-back transmission test
    $display("\nTesting back-to-back transmissions:");
    for (i = 0; i < 4; i = i + 1) begin
      SendAndVerify(8'h10 + i, 8'hB6 + i);
    end
    
    repeat(10) @(posedge r_Clk);
    
    // Results summary
    $display("\n=== Test Results ===");
    $display("Total Tests: %0d", test_count);
    $display("Passed: %0d", pass_count);
    $display("Failed: %0d", fail_count);
    
    if (fail_count == 0) begin
      $display("ALL TESTS PASSED");
    end else begin
      $display("TESTS FAILED - %0d test(s) failed", fail_count);
    end
    
    $display("Simulation time: %0t ns", $time);
    $finish;
  end

endmodule