module i2c_master #(
  parameter CLK_HZ = 100_000_000,
  parameter I2C_HZ = 400_000  // Updated for MMC34160PJ fast mode
)(
  input  wire clk, rst,
  input  wire start,                // start transaction
  input  wire [6:0] dev_addr,       // 7-bit I2C address
  input  wire [7:0] reg_addr,       // register address
  input  wire rw,                   // 0=write, 1=read
  input  wire [7:0] wr_data,        // data for write
  input  wire [7:0] rd_len,         // number of bytes to read
  output reg  rd_valid,
  output reg  [7:0] rd_data,
  input  wire rd_ready,
  output reg  busy, done, nack_seen,
  inout  wire sda, scl
);

  // Clock divider (4 phases per I2C bit)
  localparam TPH = CLK_HZ / (I2C_HZ * 4);
  localparam TPH_W = $clog2(TPH);
  reg [TPH_W-1:0] tcnt; reg [1:0] phase;
  wire tick = (tcnt == 0);

  always @(posedge clk)
    if (rst) begin tcnt<=0; phase<=0; end
    else if (tcnt==0) begin tcnt<=TPH-1; phase<=phase+1'b1; end
    else tcnt<=tcnt-1'b1;

  // Open-drain control
  reg sda_drv0, scl_drv0;
  assign sda = sda_drv0 ? 1'b0 : 1'bz;
  assign scl = scl_drv0 ? 1'b0 : 1'bz;
  wire sda_in = sda;

  // Shifter + counters
  reg [7:0] sh;
  reg [2:0] bit_idx;
  reg [7:0] bytes_left;

  // Latched command
  reg [6:0] a_dev; reg [7:0] a_reg, a_wdat, a_rlen; reg a_rw;

  // FSM states
  localparam
    IDLE=0, START0=1, SAW0=2, SAW1=3,
    REG0=4, REG1=5, WDAT0=6, WDAT1=7,
    STOP0=8, STOP1=9, RSTA0=10, SAR0=11, SAR1=12,
    RBIT=13, RACK=14, DONE_STATE=15;
  reg [4:0] st;

  // Main FSM
  always @(posedge clk) begin
    if (rst) begin
      st<=IDLE; busy<=0; done<=0; nack_seen<=0;
      rd_valid<=0; rd_data<=0; sda_drv0<=0; scl_drv0<=0;
      sh<=0; bit_idx<=7; bytes_left<=0;
    end else begin
      done<=0; rd_valid<=0;

      case (st)
        IDLE: begin
          busy<=0; nack_seen<=0; sda_drv0<=0; scl_drv0<=0;
          if (start) begin
            a_dev<=dev_addr; a_reg<=reg_addr; a_wdat<=wr_data;
            a_rlen<=rd_len; a_rw<=rw;
            busy<=1; bit_idx<=7; st<=START0;
          end
        end

        START0: if (tick&&phase==0) begin 
                  sda_drv0<=0; scl_drv0<=0; // Both HIGH initially
                end
                else if (tick&&phase==1) begin
                  sda_drv0<=1; // Pull SDA LOW while SCL HIGH (START condition)
                end
                else if (tick&&phase==2) begin
                  scl_drv0<=1; // Pull SCL LOW
                  sh<={a_dev,1'b0}; bit_idx<=7; st<=SAW0;
                end

        SAW0: if (tick&&phase==0) begin
                sda_drv0 <= ~sh[7]; scl_drv0 <= 1;  // Set data (inverted), SCL low
              end
              else if (tick&&phase==1) scl_drv0 <= 0;  // SCL high
              else if (tick&&phase==3) begin
                scl_drv0 <= 1;  // SCL low
                sh<={sh[6:0],1'b0};
                if (bit_idx==0) st<=SAW1; else bit_idx<=bit_idx-1;
              end

        SAW1: if (tick&&phase==0) begin 
                sda_drv0<=0; scl_drv0<=1;  // Release SDA for ACK, SCL low
              end
              else if (tick&&phase==1) scl_drv0 <= 0;  // SCL high
              else if (tick&&phase==2) begin
                if (sda_in) nack_seen<=1;
              end
              else if (tick&&phase==3) begin
                scl_drv0 <= 1;  // SCL low
                sh<=a_reg; bit_idx<=7; st<=REG0;
              end

        REG0: if (tick&&phase==0) begin
                sda_drv0 <= ~sh[7]; scl_drv0 <= 1;
              end
              else if (tick&&phase==1) scl_drv0 <= 0;
              else if (tick&&phase==3) begin
                scl_drv0 <= 1;
                sh<={sh[6:0],1'b0};
                if (bit_idx==0) st<=REG1; else bit_idx<=bit_idx-1;
              end

        REG1: if (tick&&phase==0) begin 
                sda_drv0<=0; scl_drv0<=1;
              end
              else if (tick&&phase==1) scl_drv0 <= 0;
              else if (tick&&phase==2) begin
                if (sda_in) nack_seen<=1;
              end
              else if (tick&&phase==3) begin
                scl_drv0 <= 1;
                if (a_rw==0) begin sh<=a_wdat; bit_idx<=7; st<=WDAT0; end
                else st<=RSTA0;
              end

        WDAT0: if (tick&&phase==0) begin
                 sda_drv0 <= ~sh[7]; scl_drv0 <= 1;
               end
               else if (tick&&phase==1) scl_drv0 <= 0;
               else if (tick&&phase==3) begin
                 scl_drv0 <= 1;
                 sh<={sh[6:0],1'b0};
                 if (bit_idx==0) st<=WDAT1; else bit_idx<=bit_idx-1;
               end

        WDAT1: if (tick&&phase==0) begin 
                 sda_drv0<=0; scl_drv0<=1;
               end
               else if (tick&&phase==1) scl_drv0 <= 0;
               else if (tick&&phase==2) begin
                 if (sda_in) nack_seen<=1;
               end
               else if (tick&&phase==3) begin
                 scl_drv0 <= 1;
                 st<=STOP0;
               end

        STOP0: if (tick&&phase==0) begin 
                 sda_drv0<=1'b1; scl_drv0<=1;  // SDA low, SCL low
               end
               else if (tick&&phase==1) scl_drv0 <= 0;  // SCL high
               else if (tick&&phase==2) sda_drv0<=0;  // SDA high (STOP)
               else if (tick&&phase==3) st<=STOP1;

        STOP1: begin done<=1; st<=DONE_STATE; end
        DONE_STATE: st<=IDLE;

        // -------- READ path --------
        // repeated START: SDA goes low while SCL is high, then send SLA+R
        RSTA0: if (tick && phase==2'd0) begin
                 scl_drv0 <= 0;   // SCL HIGH
                 sda_drv0 <= 0;   // SDA HIGH
               end
               else if (tick && phase==2'd1) begin
                 sda_drv0 <= 1;   // Pull SDA LOW (START condition while SCL HIGH)
               end
               else if (tick && phase==2'd2) begin
                 scl_drv0 <= 1;   // Pull SCL LOW
               end
               else if (tick && phase==2'd3) begin
                 sh <= {a_dev,1'b1}; // SLA+R
                 bit_idx <= 3'd7;
                 st <= SAR0;
               end

        SAR0:  if (tick&&phase==0) begin
                 sda_drv0 <= ~sh[7]; scl_drv0 <= 1;
               end
               else if (tick&&phase==1) scl_drv0 <= 0;
               else if (tick&&phase==3) begin
                 scl_drv0 <= 1;
                 sh<={sh[6:0],1'b0};
                 if (bit_idx==0) st<=SAR1; else bit_idx<=bit_idx-1;
               end

        SAR1:  if (tick&&phase==0) begin 
                 sda_drv0<=0; scl_drv0<=1;
               end
               else if (tick&&phase==1) scl_drv0 <= 0;
               else if (tick&&phase==2) begin
                 if (sda_in) nack_seen<=1;
                 bytes_left<= (a_rlen==0)?1:a_rlen;
               end
               else if (tick&&phase==3) begin
                 scl_drv0 <= 1;
                 bit_idx<=7; sh<=0; sda_drv0<=0; st<=RBIT;
               end

        RBIT:  if (tick&&phase==0) begin
                 sda_drv0 <= 0; scl_drv0 <= 1;  // Release SDA, SCL low
               end
               else if (tick&&phase==1) scl_drv0 <= 0;  // SCL high
               else if (tick&&phase==2) sh<={sh[6:0],sda_in};
               else if (tick&&phase==3) begin
                 scl_drv0 <= 1;  // SCL low
                 if (bit_idx==0) st<=RACK; 
                 else bit_idx<=bit_idx-1;
               end

        RACK:  if (tick&&phase==0) begin
                 scl_drv0 <= 1;  // SCL low
                 if (bytes_left>1) sda_drv0<=1; else sda_drv0<=0;  // ACK (pull low) if more bytes, NACK (release) if last
               end
               else if (tick&&phase==1) scl_drv0 <= 0;  // SCL high
               else if (tick&&phase==2) begin
                 rd_data<=sh; rd_valid<=1;
               end
               else if (tick&&phase==3) begin
                 scl_drv0 <= 1;  // SCL low
                 if (bytes_left==1) st<=STOP0;
                 else begin bytes_left<=bytes_left-1; bit_idx<=7; sh<=0; st<=RBIT; end
               end
      endcase
    end
  end
endmodule