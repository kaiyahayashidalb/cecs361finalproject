`timescale 1ns / 1ps
//==============================================================================
// I2C Master with CMPS2 Calibration
// 
// Two operating modes:
// 1. Manual mode: User controls transactions via start_txn
// 2. Calibration mode: Automatic calibration sequence via start_cal
//==============================================================================

module i2c_master #(
    parameter CLK_HZ = 100_000_000,
    parameter I2C_HZ = 400_000
)(
    input  wire        clk,
    input  wire        rst,
    
    // Manual transaction interface
    input  wire        start_txn,
    input  wire [6:0]  device_addr,
    input  wire [7:0]  reg_addr,
    input  wire        rw,
    input  wire [2:0]  num_bytes,
    input  wire [7:0]  wr_data,
    
    output reg  [7:0]  rd_data,
    output reg         rd_valid,
    output reg         busy,
    output reg         done,
    output reg         error,
    
    // Calibration interface
    input  wire        start_cal,
    output reg         cal_busy,
    output reg         cal_done,
    output reg         cal_error,
    output reg signed [15:0] x_offset,
    output reg signed [15:0] y_offset,
    output reg signed [15:0] z_offset,
    
    // I2C bus
    inout  wire        sda,
    inout  wire        scl
);

    //==========================================================================
    // CMPS2 Constants
    //==========================================================================
    localparam [6:0] CMPS2_ADDR = 7'h30;
    localparam [7:0] REG_XOUT_LSB = 8'h00;
    localparam [7:0] REG_STATUS = 8'h03;  // Status register (bit 0 = measurement done)
    localparam [7:0] REG_CTRL0 = 8'h07;
    localparam [7:0] CMD_REFILL = 8'h80;
    localparam [7:0] CMD_SET = 8'h20;
    localparam [7:0] CMD_RESET = 8'h40;
    localparam [7:0] CMD_MEAS = 8'h01;
    
    localparam integer DELAY_10MS = CLK_HZ / 100;
    localparam integer DELAY_50MS = CLK_HZ / 20;
    localparam integer DELAY_1MS = CLK_HZ / 1000;
    localparam integer DELAY_8MS = (CLK_HZ * 8) / 1000;
    
    //==========================================================================
    // I2C Timing
    //==========================================================================
    localparam integer CLKS_PER_QUARTER = CLK_HZ / (I2C_HZ * 4);
    reg [$clog2(CLKS_PER_QUARTER)-1:0] clk_div;
    reg [1:0] quarter;
    wire quarter_tick = (clk_div == CLKS_PER_QUARTER - 1);
    
    always @(posedge clk) begin
        if (rst || i2c_state == I2C_IDLE) begin
            clk_div <= 0;
            quarter <= 0;
        end else if (quarter_tick) begin
            clk_div <= 0;
            quarter <= quarter + 1;
        end else begin
            clk_div <= clk_div + 1;
        end
    end
    
    //==========================================================================
    // I2C Signals
    //==========================================================================
    reg scl_out, sda_out;
    assign scl = scl_out ? 1'bz : 1'b0;
    assign sda = sda_out ? 1'bz : 1'b0;
    
    //==========================================================================
    // I2C State Machine
    //==========================================================================
    localparam [3:0]
        I2C_IDLE = 0,
        I2C_START = 1,
        I2C_ADDR = 2,
        I2C_ADDR_ACK = 3,
        I2C_REG = 4,
        I2C_REG_ACK = 5,
        I2C_DATA_W = 6,
        I2C_DATA_W_ACK = 7,
        I2C_RESTART = 8,
        I2C_DATA_R = 9,
        I2C_MASTER_ACK = 10,
        I2C_STOP = 11;
    
    reg [3:0] i2c_state;
    reg [7:0] i2c_shift;
    reg [2:0] i2c_bit_cnt;
    reg [2:0] i2c_byte_cnt;
    reg [1:0] i2c_is_read;  // 0=write, 1=read, 2=read after restart
    reg [6:0] i2c_addr;
    reg [7:0] i2c_reg;
    reg [7:0] i2c_wdata;
    
    //==========================================================================
    // Calibration State Machine
    //==========================================================================
    localparam [4:0]
        CAL_IDLE = 0,
        CAL_INIT = 1,
        CAL_SET_REFILL = 2,
        CAL_SET_REFILL_WAIT = 3,
        CAL_SET_CMD = 4,
        CAL_SET_WAIT = 5,
        CAL_SET_MEAS = 6,
        CAL_SET_MEAS_WAIT = 7,
        CAL_SET_STATUS = 8,
        CAL_SET_STATUS_WAIT = 9,
        CAL_SET_DATA = 10,
        CAL_SET_DATA_WAIT = 11,
        CAL_RST_REFILL = 12,
        CAL_RST_REFILL_WAIT = 13,
        CAL_RST_CMD = 14,
        CAL_RST_WAIT = 15,
        CAL_RST_MEAS = 16,
        CAL_RST_MEAS_WAIT = 17,
        CAL_RST_STATUS = 18,
        CAL_RST_STATUS_WAIT = 19,
        CAL_RST_DATA = 20,
        CAL_RST_DATA_WAIT = 21,
        CAL_CALC = 22;
    
    reg [4:0] cal_state;
    reg [31:0] cal_timer;
    reg [7:0] cal_data_buf [0:5];
    reg [2:0] cal_data_idx;
    reg signed [15:0] x_set, y_set, z_set;
    reg signed [15:0] x_rst, y_rst, z_rst;
    reg cal_start_txn;
    reg [6:0] cal_addr;
    reg [7:0] cal_reg;
    reg cal_rw;
    reg [2:0] cal_bytes;
    reg [7:0] cal_wdata;
    
    wire cal_timer_done = (cal_timer == 0);
    
    always @(posedge clk) begin
        if (rst || cal_state == CAL_IDLE)
            cal_timer <= 0;
        else if (cal_timer > 0)
            cal_timer <= cal_timer - 1;
    end
    
    // Capture read data during calibration
    always @(posedge clk) begin
        if (rd_valid && cal_busy) begin
            cal_data_buf[cal_data_idx] <= rd_data;
            cal_data_idx <= cal_data_idx + 1;
        end
    end
    
    //==========================================================================
    // I2C Transaction Controller - uses muxed signals
    //==========================================================================
    wire i2c_start = cal_busy ? cal_start_txn : start_txn;
    wire [6:0] active_addr = cal_busy ? cal_addr : device_addr;
    wire [7:0] active_reg = cal_busy ? cal_reg : reg_addr;
    wire active_rw = cal_busy ? cal_rw : rw;
    wire [2:0] active_bytes = cal_busy ? cal_bytes : num_bytes;
    wire [7:0] active_wdata = cal_busy ? cal_wdata : wr_data;
    
    always @(posedge clk) begin
        if (rst) begin
            i2c_state <= I2C_IDLE;
            busy <= 0;
            done <= 0;
            error <= 0;
            rd_valid <= 0;
            scl_out <= 1;
            sda_out <= 1;
            i2c_shift <= 0;
            i2c_bit_cnt <= 0;
            i2c_byte_cnt <= 0;
            i2c_is_read <= 0;
            rd_data <= 0;
            i2c_addr <= 0;
            i2c_reg <= 0;
            i2c_wdata <= 0;
        end else begin
            done <= 0;
            rd_valid <= 0;
            
            case (i2c_state)
                I2C_IDLE: begin
                    busy <= 0;
                    scl_out <= 1;
                    sda_out <= 1;
                    error <= 0;
                    
                    if (i2c_start) begin
                        busy <= 1;
                        i2c_addr <= active_addr;
                        i2c_reg <= active_reg;
                        i2c_is_read <= active_rw;
                        i2c_byte_cnt <= active_bytes;
                        i2c_wdata <= active_wdata;
                        i2c_state <= I2C_START;
                    end
                end
                
                I2C_START: begin
                    if (quarter_tick) begin
                        case (quarter)
                            2'd0: begin sda_out <= 1; scl_out <= 1; end
                            2'd1: sda_out <= 0;
                            2'd2: scl_out <= 0;
                            2'd3: begin
                                i2c_shift <= {i2c_addr, 1'b0};
                                i2c_bit_cnt <= 7;
                                i2c_state <= I2C_ADDR;
                            end
                        endcase
                    end
                end
                
                I2C_ADDR: begin
                    if (quarter_tick) begin
                        case (quarter)
                            2'd0: begin scl_out <= 0; sda_out <= i2c_shift[7]; end
                            2'd1: scl_out <= 1;
                            2'd2: ;
                            2'd3: begin
                                scl_out <= 0;
                                i2c_shift <= {i2c_shift[6:0], 1'b0};
                                if (i2c_bit_cnt == 0) begin
                                    i2c_state <= I2C_ADDR_ACK;
                                    sda_out <= 1;
                                end else
                                    i2c_bit_cnt <= i2c_bit_cnt - 1;
                            end
                        endcase
                    end
                end
                
                I2C_ADDR_ACK: begin
                    if (quarter_tick) begin
                        case (quarter)
                            2'd0: begin scl_out <= 0; sda_out <= 1; end
                            2'd1: scl_out <= 1;
                            2'd2: if (sda && i2c_is_read != 2) error <= 1;
                            2'd3: begin
                                scl_out <= 0;
                                if (error)
                                    i2c_state <= I2C_STOP;
                                else if (i2c_is_read == 2) begin
                                    // After restart for read: go directly to DATA_R
                                    i2c_bit_cnt <= 7;
                                    i2c_state <= I2C_DATA_R;
                                    i2c_is_read <= 1;
                                end else begin
                                    i2c_shift <= i2c_reg;
                                    i2c_bit_cnt <= 7;
                                    i2c_state <= I2C_REG;
                                end
                            end
                        endcase
                    end
                end
                
                I2C_REG: begin
                    if (quarter_tick) begin
                        case (quarter)
                            2'd0: begin scl_out <= 0; sda_out <= i2c_shift[7]; end
                            2'd1: scl_out <= 1;
                            2'd2: ;
                            2'd3: begin
                                scl_out <= 0;
                                i2c_shift <= {i2c_shift[6:0], 1'b0};
                                if (i2c_bit_cnt == 0) begin
                                    i2c_state <= I2C_REG_ACK;
                                    sda_out <= 1;
                                end else
                                    i2c_bit_cnt <= i2c_bit_cnt - 1;
                            end
                        endcase
                    end
                end
                
                I2C_REG_ACK: begin
                    if (quarter_tick) begin
                        case (quarter)
                            2'd0: begin scl_out <= 0; sda_out <= 1; end
                            2'd1: scl_out <= 1;
                            2'd2: if (sda) error <= 1;
                            2'd3: begin
                                scl_out <= 0;
                                if (error)
                                    i2c_state <= I2C_STOP;
                                else if (i2c_is_read)
                                    i2c_state <= I2C_RESTART;
                                else begin
                                    i2c_shift <= i2c_wdata;
                                    i2c_bit_cnt <= 7;
                                    i2c_state <= I2C_DATA_W;
                                end
                            end
                        endcase
                    end
                end
                
                I2C_DATA_W: begin
                    if (quarter_tick) begin
                        case (quarter)
                            2'd0: begin scl_out <= 0; sda_out <= i2c_shift[7]; end
                            2'd1: scl_out <= 1;
                            2'd2: ;
                            2'd3: begin
                                scl_out <= 0;
                                i2c_shift <= {i2c_shift[6:0], 1'b0};
                                if (i2c_bit_cnt == 0) begin
                                    i2c_state <= I2C_DATA_W_ACK;
                                    sda_out <= 1;
                                end else
                                    i2c_bit_cnt <= i2c_bit_cnt - 1;
                            end
                        endcase
                    end
                end
                
                I2C_DATA_W_ACK: begin
                    if (quarter_tick) begin
                        case (quarter)
                            2'd0: begin scl_out <= 0; sda_out <= 1; end
                            2'd1: scl_out <= 1;
                            2'd2: if (sda) error <= 1;
                            2'd3: begin
                                scl_out <= 0;
                                i2c_state <= I2C_STOP;
                            end
                        endcase
                    end
                end
                
                I2C_RESTART: begin
                    if (quarter_tick) begin
                        case (quarter)
                            2'd0: begin sda_out <= 1; scl_out <= 0; end
                            2'd1: scl_out <= 1;
                            2'd2: sda_out <= 0;
                            2'd3: begin
                                scl_out <= 0;
                                i2c_shift <= {i2c_addr, 1'b1};
                                i2c_bit_cnt <= 7;
                                i2c_state <= I2C_ADDR;
                                i2c_is_read <= 2;
                            end
                        endcase
                    end
                end
                
                I2C_DATA_R: begin
                    if (quarter_tick) begin
                        case (quarter)
                            2'd0: begin scl_out <= 0; sda_out <= 1; end
                            2'd1: scl_out <= 1;
                            2'd2: begin
                                // Sample bit and shift into register
                                i2c_shift <= {i2c_shift[6:0], sda};
                            end
                            2'd3: begin
                                scl_out <= 0;
                                if (i2c_bit_cnt == 0) begin
                                    // Capture complete byte from shift register (updated at Q2)
                                    rd_data <= i2c_shift;
                                    rd_valid <= 1;
                                    i2c_byte_cnt <= i2c_byte_cnt - 1;
                                    i2c_state <= I2C_MASTER_ACK;
                                end else
                                    i2c_bit_cnt <= i2c_bit_cnt - 1;
                            end
                        endcase
                    end
                end
                
                I2C_MASTER_ACK: begin
                    if (quarter_tick) begin
                        case (quarter)
                            2'd0: begin
                                scl_out <= 0;
                                sda_out <= (i2c_byte_cnt == 0);
                            end
                            2'd1: scl_out <= 1;
                            2'd2: ;
                            2'd3: begin
                                scl_out <= 0;
                                if (i2c_byte_cnt == 0)
                                    i2c_state <= I2C_STOP;
                                else begin
                                    i2c_bit_cnt <= 7;
                                    i2c_state <= I2C_DATA_R;
                                end
                            end
                        endcase
                    end
                end
                
                I2C_STOP: begin
                    if (quarter_tick) begin
                        case (quarter)
                            2'd0: begin sda_out <= 0; scl_out <= 0; end
                            2'd1: scl_out <= 1;
                            2'd2: sda_out <= 1;
                            2'd3: begin
                                done <= 1;
                                i2c_state <= I2C_IDLE;
                            end
                        endcase
                    end
                end
            endcase
        end
    end
    
    //==========================================================================
    // Calibration Controller
    //==========================================================================
    always @(posedge clk) begin
        if (rst) begin
            cal_state <= CAL_IDLE;
            cal_busy <= 0;
            cal_done <= 0;
            cal_error <= 0;
            x_offset <= 0;
            y_offset <= 0;
            z_offset <= 0;
            x_set <= 0;
            y_set <= 0;
            z_set <= 0;
            x_rst <= 0;
            y_rst <= 0;
            z_rst <= 0;
            cal_data_idx <= 0;
            cal_start_txn <= 0;
            cal_addr <= CMPS2_ADDR;
            cal_reg <= 0;
            cal_rw <= 0;
            cal_bytes <= 1;
            cal_wdata <= 0;
        end else begin
            cal_done <= 0;
            cal_start_txn <= 0;
            
            case (cal_state)
                CAL_IDLE: begin
                    cal_busy <= 0;
                    if (start_cal) begin
                        cal_busy <= 1;
                        cal_timer <= DELAY_10MS;
                        cal_state <= CAL_INIT;
                    end
                end
                
                CAL_INIT: begin
                    if (cal_timer_done) begin
                        cal_reg <= REG_CTRL0;
                        cal_rw <= 0;
                        cal_bytes <= 1;
                        cal_wdata <= CMD_REFILL;
                        cal_start_txn <= 1;
                        cal_state <= CAL_SET_REFILL;
                    end
                end
                
                CAL_SET_REFILL: begin
                    if (busy) cal_state <= CAL_SET_REFILL_WAIT;
                end
                
                CAL_SET_REFILL_WAIT: begin
                    if (done && !error) begin
                        cal_timer <= DELAY_50MS;
                        cal_wdata <= CMD_SET;
                        cal_state <= CAL_SET_CMD;
                    end else if (error) begin
                        cal_error <= 1;
                        cal_state <= CAL_IDLE;
                    end
                end
                
                CAL_SET_CMD: begin
                    if (cal_timer_done) begin
                        cal_start_txn <= 1;
                        cal_state <= CAL_SET_WAIT;
                    end
                end
                
                CAL_SET_WAIT: begin
                    if (busy) begin
                        if (done && !error) begin
                            cal_timer <= DELAY_1MS;
                            cal_wdata <= CMD_MEAS;
                            cal_state <= CAL_SET_MEAS;
                        end else if (error) begin
                            cal_error <= 1;
                            cal_state <= CAL_IDLE;
                        end
                    end
                end
                
                CAL_SET_MEAS: begin
                    if (cal_timer_done) begin
                        cal_start_txn <= 1;
                        cal_state <= CAL_SET_MEAS_WAIT;
                    end
                end
                
                CAL_SET_MEAS_WAIT: begin
                    if (busy) begin
                        if (done && !error) begin
                            cal_timer <= DELAY_8MS;
                            cal_reg <= REG_STATUS;
                            cal_rw <= 1;
                            cal_state <= CAL_SET_STATUS;
                        end else if (error) begin
                            cal_error <= 1;
                            cal_state <= CAL_IDLE;
                        end
                    end
                end
                
                CAL_SET_STATUS: begin
                    if (cal_timer_done) begin
                        cal_start_txn <= 1;
                        cal_state <= CAL_SET_STATUS_WAIT;
                    end
                end
                
                CAL_SET_STATUS_WAIT: begin
                    if (busy) begin
                        if (done && !error) begin
                            if (rd_data[0]) begin
                                cal_reg <= REG_XOUT_LSB;
                                cal_bytes <= 6;
                                cal_data_idx <= 0;
                                cal_state <= CAL_SET_DATA;
                            end else begin
                                cal_timer <= DELAY_1MS;
                                cal_state <= CAL_SET_STATUS;
                            end
                        end else if (error) begin
                            cal_error <= 1;
                            cal_state <= CAL_IDLE;
                        end
                    end
                end
                
                CAL_SET_DATA: begin
                    cal_start_txn <= 1;
                    cal_state <= CAL_SET_DATA_WAIT;
                end
                
                CAL_SET_DATA_WAIT: begin
                    if (busy) begin
                        if (done && !error) begin
                            x_set <= {cal_data_buf[1], cal_data_buf[0]};
                            y_set <= {cal_data_buf[3], cal_data_buf[2]};
                            z_set <= {cal_data_buf[5], cal_data_buf[4]};
                            cal_reg <= REG_CTRL0;
                            cal_rw <= 0;
                            cal_bytes <= 1;
                            cal_wdata <= CMD_REFILL;
                            cal_state <= CAL_RST_REFILL;
                        end else if (error) begin
                            cal_error <= 1;
                            cal_state <= CAL_IDLE;
                        end
                    end
                end
                
                // RESET sequence
                CAL_RST_REFILL: begin
                    cal_start_txn <= 1;
                    cal_state <= CAL_RST_REFILL_WAIT;
                end
                
                CAL_RST_REFILL_WAIT: begin
                    if (busy) begin
                        if (done && !error) begin
                            cal_timer <= DELAY_50MS;
                            cal_wdata <= CMD_RESET;
                            cal_state <= CAL_RST_CMD;
                        end else if (error) begin
                            cal_error <= 1;
                            cal_state <= CAL_IDLE;
                        end
                    end
                end
                
                CAL_RST_CMD: begin
                    if (cal_timer_done) begin
                        cal_start_txn <= 1;
                        cal_state <= CAL_RST_WAIT;
                    end
                end
                
                CAL_RST_WAIT: begin
                    if (busy) begin
                        if (done && !error) begin
                            cal_timer <= DELAY_1MS;
                            cal_wdata <= CMD_MEAS;
                            cal_state <= CAL_RST_MEAS;
                        end else if (error) begin
                            cal_error <= 1;
                            cal_state <= CAL_IDLE;
                        end
                    end
                end
                
                CAL_RST_MEAS: begin
                    if (cal_timer_done) begin
                        cal_start_txn <= 1;
                        cal_state <= CAL_RST_MEAS_WAIT;
                    end
                end
                
                CAL_RST_MEAS_WAIT: begin
                    if (busy) begin
                        if (done && !error) begin
                            cal_timer <= DELAY_8MS;
                            cal_reg <= REG_STATUS;
                            cal_rw <= 1;
                            cal_bytes <= 1;
                            cal_state <= CAL_RST_STATUS;
                        end else if (error) begin
                            cal_error <= 1;
                            cal_state <= CAL_IDLE;
                        end
                    end
                end
                
                CAL_RST_STATUS: begin
                    if (cal_timer_done) begin
                        cal_start_txn <= 1;
                        cal_state <= CAL_RST_STATUS_WAIT;
                    end
                end
                
                CAL_RST_STATUS_WAIT: begin
                    if (busy) begin
                        if (done && !error) begin
                            if (rd_data[0]) begin
                                cal_reg <= REG_XOUT_LSB;
                                cal_rw <= 1;
                                cal_bytes <= 6;
                                cal_data_idx <= 0;
                                cal_state <= CAL_RST_DATA;
                            end else begin
                                cal_timer <= DELAY_1MS;
                                cal_state <= CAL_RST_STATUS;
                            end
                        end else if (error) begin
                            cal_error <= 1;
                            cal_state <= CAL_IDLE;
                        end
                    end
                end
                
                CAL_RST_DATA: begin
                    cal_start_txn <= 1;
                    cal_state <= CAL_RST_DATA_WAIT;
                end
                
                CAL_RST_DATA_WAIT: begin
                    if (busy) begin
                        if (done && !error) begin
                            x_rst <= {cal_data_buf[1], cal_data_buf[0]};
                            y_rst <= {cal_data_buf[3], cal_data_buf[2]};
                            z_rst <= {cal_data_buf[5], cal_data_buf[4]};
                            cal_state <= CAL_CALC;
                        end else if (error) begin
                            cal_error <= 1;
                            cal_state <= CAL_IDLE;
                        end
                    end
                end
                
                CAL_CALC: begin
                    x_offset <= (x_set + x_rst) >>> 1;
                    y_offset <= (y_set + y_rst) >>> 1;
                    z_offset <= (z_set + z_rst) >>> 1;
                    cal_done <= 1;
                    cal_state <= CAL_IDLE;
                end
            endcase
        end
    end

endmodule
