module i2c_master #(
  parameter CLK_HZ = 100_000_000,
  parameter I2C_HZ = 400_000
)(
  input  wire clk,
  input  wire rst,
  
  // Simple transaction interface
  input  wire start,              // Start a transaction
  input  wire [7:0] tx_data,      // Data to send (address or data byte)
  input  wire tx_valid,           // tx_data is valid
  output reg  tx_ready,           // Ready to accept tx_data
  
  output reg  [7:0] rx_data,      // Received data byte
  output reg  rx_valid,           // rx_data is valid
  
  input  wire gen_start_cond,     // Generate START condition before next byte
  input  wire gen_stop_cond,      // Generate STOP condition after current byte
  input  wire read_mode,          // 1 = read byte from slave, 0 = write byte to slave
  input  wire send_ack,           // When reading: 1 = send ACK, 0 = send NACK
  
  output reg  busy,               // Transaction in progress
  output reg  nack_received,      // Slave sent NACK
  
  // I2C bus
  inout  wire sda,
  inout  wire scl
);

  // Clock divider for I2C timing (4 phases per bit)
  localparam TPH = CLK_HZ / (I2C_HZ * 4);
  localparam TPH_W = $clog2(TPH);
  
  reg [TPH_W-1:0] tcnt;
  reg [1:0] phase;
  wire tick = (tcnt == 0);
  
  always @(posedge clk) begin
    if (rst) begin
      tcnt <= 0;
      phase <= 0;
    end else if (tcnt == 0) begin
      tcnt <= TPH - 1;
      phase <= phase + 1'b1;
    end else begin
      tcnt <= tcnt - 1'b1;
    end
  end

  // Open-drain I2C outputs
  reg sda_out, scl_out;
  assign sda = sda_out ? 1'b0 : 1'bz;
  assign scl = scl_out ? 1'b0 : 1'bz;
  
  // Read SDA with pullup behavior (high-Z = 1)
  wire sda_in = (sda === 1'b0) ? 1'b0 : 1'b1;

  // Shift register and bit counter
  reg [7:0] shift_reg;
  reg [2:0] bit_cnt;
  
  // Latched control signals
  reg do_start, do_stop, is_read, ack_bit;
  
  // FSM states
  localparam IDLE = 0;
  localparam START = 1;
  localparam TX_BIT = 2;
  localparam TX_ACK = 3;
  localparam RX_BIT = 4;
  localparam RX_ACK = 5;
  localparam STOP = 6;
  
  reg [2:0] state;

  always @(posedge clk) begin
    if (rst) begin
      state <= IDLE;
      busy <= 0;
      tx_ready <= 1;
      rx_valid <= 0;
      nack_received <= 0;
      sda_out <= 0;
      scl_out <= 0;
      shift_reg <= 0;
      bit_cnt <= 0;
      do_start <= 0;
      do_stop <= 0;
      is_read <= 0;
      ack_bit <= 0;
    end else begin
      rx_valid <= 0;
      
      case (state)
        IDLE: begin
          busy <= 0;
          tx_ready <= 1;
          sda_out <= 0;
          scl_out <= 0;
          
          if (start && tx_valid) begin
            shift_reg <= tx_data;
            do_start <= gen_start_cond;
            do_stop <= gen_stop_cond;
            is_read <= read_mode;
            ack_bit <= send_ack;
            bit_cnt <= 7;
            busy <= 1;
            tx_ready <= 0;
            
            if (gen_start_cond) begin
              state <= START;
            end else if (read_mode) begin
              state <= RX_BIT;
            end else begin
              state <= TX_BIT;
            end
          end
        end

        // Generate START condition: SDA falls while SCL is high
        START: begin
          if (tick && phase == 0) begin
            sda_out <= 0;  // SDA = high
            scl_out <= 0;  // SCL = high
          end else if (tick && phase == 1) begin
            sda_out <= 1;  // Pull SDA low (START)
          end else if (tick && phase == 2) begin
            scl_out <= 1;  // Pull SCL low
          end else if (tick && phase == 3) begin
            if (is_read) begin
              state <= RX_BIT;
              shift_reg <= 0;
            end else begin
              state <= TX_BIT;
            end
          end
        end

        // Transmit data bit
        TX_BIT: begin
          if (tick && phase == 0) begin
            sda_out <= ~shift_reg[7];  // Set data bit (inverted for open-drain)
            scl_out <= 1;              // SCL low
          end else if (tick && phase == 1) begin
            scl_out <= 0;              // SCL high
          end else if (tick && phase == 3) begin
            scl_out <= 1;              // SCL low
            shift_reg <= {shift_reg[6:0], 1'b0};
            
            if (bit_cnt == 0) begin
              state <= TX_ACK;
            end else begin
              bit_cnt <= bit_cnt - 1;
            end
          end
        end

        // Receive ACK/NACK from slave
        TX_ACK: begin
          if (tick && phase == 0) begin
            sda_out <= 0;  // Release SDA
            scl_out <= 1;  // SCL low
          end else if (tick && phase == 1) begin
            scl_out <= 0;  // SCL high
          end else if (tick && phase == 2) begin
            nack_received <= sda_in;  // Sample ACK/NACK
          end else if (tick && phase == 3) begin
            scl_out <= 1;  // SCL low
            
            if (do_stop) begin
              state <= STOP;
            end else begin
              state <= IDLE;
            end
          end
        end

        // Receive data bit from slave
        RX_BIT: begin
          if (tick && phase == 0) begin
            sda_out <= 0;  // Release SDA
            scl_out <= 1;  // SCL low
          end else if (tick && phase == 1) begin
            scl_out <= 0;  // SCL high
          end else if (tick && phase == 2) begin
            shift_reg <= {shift_reg[6:0], sda_in};  // Sample data bit
          end else if (tick && phase == 3) begin
            scl_out <= 1;  // SCL low
            
            if (bit_cnt == 0) begin
              state <= RX_ACK;
            end else begin
              bit_cnt <= bit_cnt - 1;
            end
          end
        end

        // Send ACK/NACK to slave
        RX_ACK: begin
          if (tick && phase == 0) begin
            sda_out <= ~ack_bit;  // ACK=0 (pull low), NACK=1 (release)
            scl_out <= 1;         // SCL low
          end else if (tick && phase == 1) begin
            scl_out <= 0;  // SCL high
          end else if (tick && phase == 2) begin
            rx_data <= shift_reg;
            rx_valid <= 1;
          end else if (tick && phase == 3) begin
            scl_out <= 1;  // SCL low
            
            if (do_stop) begin
              state <= STOP;
            end else begin
              state <= IDLE;
            end
          end
        end

        // Generate STOP condition: SDA rises while SCL is high
        STOP: begin
          if (tick && phase == 0) begin
            sda_out <= 1;  // SDA low
            scl_out <= 1;  // SCL low
          end else if (tick && phase == 1) begin
            scl_out <= 0;  // SCL high
          end else if (tick && phase == 2) begin
            sda_out <= 0;  // SDA high (STOP)
          end else if (tick && phase == 3) begin
            state <= IDLE;
          end
        end

        default: state <= IDLE;
      endcase
    end
  end
endmodule
