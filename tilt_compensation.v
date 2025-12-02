`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// CECS 361 Final Project - Digital Compass
// Engineer: Nathan Sarkozy
// 
// Module: tilt_compensation
// Description: Tilt compensation module with integrated sensor drivers.
//              - Reads accelerometer via SPI (ADXL362)
//              - Reads magnetometer via I2C (MMC34160PJ)
//              - Applies tilt compensation to magnetometer readings
//              - Outputs corrected X/Y magnetic field for heading calculation
// 
// Dependencies: SPI_master.v, magnetometer_driver.v
//////////////////////////////////////////////////////////////////////////////////

module tilt_compensation (
    // Clocks and reset
    input wire clk,                       // 100 MHz system clock
    input wire iclk,                      // 4 MHz clock for SPI
    input wire reset,                     // Active high reset
    
    // SPI interface (accelerometer)
    input wire miso,
    output wire sclk,
    output wire mosi,
    output wire cs,
    
    // I2C interface (magnetometer)
    inout wire sda,
    inout wire scl,
    
    // Test inputs (directly inject sensor data for simulation)
    input wire [14:0] acl_data,           // Direct accelerometer data input
    input wire signed [15:0] mag_x,       // Direct magnetometer X input
    input wire signed [15:0] mag_y,       // Direct magnetometer Y input
    input wire signed [15:0] mag_z,       // Direct magnetometer Z input
    
    // Outputs
    output reg signed [15:0] mag_x_comp,  // Tilt-compensated X
    output reg signed [15:0] mag_y_comp,  // Tilt-compensated Y
    output reg signed [8:0] pitch,        // Pitch angle (degrees)
    output reg signed [8:0] roll,         // Roll angle (degrees)
    output reg data_valid                 // New data ready flag
);

    // ========== SPI Accelerometer (ADXL362) ==========
    wire [14:0] acl_data_spi;
    
    SPI_master spi_accel (
        .iclk(iclk),
        .miso(miso),
        .sclk(sclk),
        .mosi(mosi),
        .cs(cs),
        .acl_data(acl_data_spi)
    );

    // ========== I2C Magnetometer (MMC34160PJ) ==========
    wire signed [15:0] mag_x_i2c, mag_y_i2c, mag_z_i2c;
    wire mag_busy;
    wire mag_error;
    reg start_mag_read;
    reg [19:0] mag_timer;
    
    magnetometer_driver #(
        .CLK_HZ(100_000_000)
    ) mag_inst (
        .clk(clk),
        .rst(reset),
        .start_read(start_mag_read),
        .data_valid(),  // Not used - we poll periodically
        .busy(mag_busy),
        .error(mag_error),
        .mag_x(mag_x_i2c),
        .mag_y(mag_y_i2c),
        .mag_z(mag_z_i2c),
        .sda(sda),
        .scl(scl)
    );
    
    // ========== Sensor Data Selection ==========
    // Use direct inputs if provided (non-zero), otherwise use hardware drivers
    wire [14:0] acl_data_sel = (acl_data != 0) ? acl_data : acl_data_spi;
    wire signed [15:0] mag_x_sel = (mag_x != 0 || mag_y != 0 || mag_z != 0) ? mag_x : mag_x_i2c;
    wire signed [15:0] mag_y_sel = (mag_x != 0 || mag_y != 0 || mag_z != 0) ? mag_y : mag_y_i2c;
    wire signed [15:0] mag_z_sel = (mag_x != 0 || mag_y != 0 || mag_z != 0) ? mag_z : mag_z_i2c;
    
    // Trigger magnetometer read every ~10ms
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            mag_timer <= 0;
            start_mag_read <= 0;
        end else begin
            start_mag_read <= 0;
            if (mag_timer >= 20'd1_000_000) begin
                mag_timer <= 0;
                if (!mag_busy)
                    start_mag_read <= 1;
            end else begin
                mag_timer <= mag_timer + 1;
            end
        end
    end

    // ========== Accelerometer Data Extraction ==========
    // Sign-extend 5-bit values to 9-bit signed
    wire signed [8:0] accel_x = {{4{acl_data_sel[14]}}, acl_data_sel[14:10]};
    wire signed [8:0] accel_y = {{4{acl_data_sel[9]}}, acl_data_sel[9:5]};
    wire signed [8:0] accel_z = {{4{acl_data_sel[4]}}, acl_data_sel[4:0]};

    // ========== Tilt Compensation State Machine ==========
    localparam IDLE      = 4'd0,
               CALC_ANG  = 4'd1,
               CALC_TRIG = 4'd2,
               COMP_X1   = 4'd3,
               COMP_X2   = 4'd4,
               COMP_X3   = 4'd5,
               COMP_Y1   = 4'd6,
               COMP_Y2   = 4'd7,
               COMP_Y3   = 4'd8,
               COMP_Y4   = 4'd9,
               COMP_Y5   = 4'd10,
               COMP_Y6   = 4'd11,
               DONE      = 4'd12;
    
    reg [3:0] state;
    reg [15:0] delay_counter;
    
    // Calculated angles
    reg signed [8:0] pitch_calc;
    reg signed [8:0] roll_calc;
    
    // Trig values (Q8.8 fixed point: 256 = 1.0)
    reg signed [15:0] sin_pitch, cos_pitch;
    reg signed [15:0] sin_roll, cos_roll;
    
    // Intermediate calculation registers
    reg signed [31:0] temp_mult;
    reg signed [15:0] x_term1, x_term2;
    reg signed [15:0] y_term1, y_term2, y_term3;
    
    // Latched sensor values
    reg signed [15:0] mag_x_reg, mag_y_reg, mag_z_reg;
    reg signed [8:0] ax_reg, ay_reg, az_reg;
    
    // ========== Main Processing FSM ==========
    always @(posedge iclk or posedge reset) begin
        if (reset) begin
            state <= IDLE;
            delay_counter <= 0;
            data_valid <= 0;
            pitch <= 0;
            roll <= 0;
            mag_x_comp <= 0;
            mag_y_comp <= 0;
        end else begin
            case (state)
                IDLE: begin
                    data_valid <= 0;
                    delay_counter <= delay_counter + 1;
                    
                    // Process every ~10ms (40000 cycles at 4MHz)
                    if (delay_counter >= 16'd40000) begin
                        delay_counter <= 0;
                        // Latch sensor values (using selected source)
                        mag_x_reg <= mag_x_sel;
                        mag_y_reg <= mag_y_sel;
                        mag_z_reg <= mag_z_sel;
                        ax_reg <= accel_x;
                        ay_reg <= accel_y;
                        az_reg <= accel_z;
                        state <= CALC_ANG;
                    end
                end
                
                // Calculate pitch and roll angles
                CALC_ANG: begin
                    if (az_reg != 0) begin
                        // atan(x/z) ? (x/z) * 57.3° for small angles
                        pitch_calc <= (ax_reg * 9'sd57) / az_reg;
                        roll_calc <= (ay_reg * 9'sd57) / az_reg;
                    end else begin
                        // Handle vertical orientation
                        pitch_calc <= (ax_reg > 0) ? 9'sd90 : -9'sd90;
                        roll_calc <= (ay_reg > 0) ? 9'sd90 : -9'sd90;
                    end
                    state <= CALC_TRIG;
                end
                
                // Calculate sin/cos using approximations
                CALC_TRIG: begin
                    pitch <= pitch_calc;
                    roll <= roll_calc;
                    
                    // sin(?) ? ? * ?/180 * 256 = ? * 4.47
                    sin_pitch <= (pitch_calc * 16'sd1144) >>> 8;
                    sin_roll <= (roll_calc * 16'sd1144) >>> 8;
                    
                    // cos(?) ? 1 - ?²/2, scaled to Q8.8
                    cos_pitch <= 16'sd256 - ((pitch_calc * pitch_calc) >>> 7);
                    cos_roll <= 16'sd256 - ((roll_calc * roll_calc) >>> 7);
                    
                    state <= COMP_X1;
                end
                
                // X compensation: X_comp = X*cos(pitch) + Z*sin(pitch)
                COMP_X1: begin
                    temp_mult <= mag_x_reg * cos_pitch;
                    state <= COMP_X2;
                end
                
                COMP_X2: begin
                    x_term1 <= temp_mult[23:8];
                    temp_mult <= mag_z_reg * sin_pitch;
                    state <= COMP_X3;
                end
                
                COMP_X3: begin
                    mag_x_comp <= x_term1 + temp_mult[23:8];
                    state <= COMP_Y1;
                end
                
                // Y compensation: Y_comp = X*sin(roll)*sin(pitch) + Y*cos(roll) - Z*sin(roll)*cos(pitch)
                COMP_Y1: begin
                    temp_mult <= mag_x_reg * sin_roll;
                    state <= COMP_Y2;
                end
                
                COMP_Y2: begin
                    temp_mult <= $signed(temp_mult[23:8]) * sin_pitch;
                    state <= COMP_Y3;
                end
                
                COMP_Y3: begin
                    y_term1 <= temp_mult[23:8];
                    temp_mult <= mag_y_reg * cos_roll;
                    state <= COMP_Y4;
                end
                
                COMP_Y4: begin
                    y_term2 <= temp_mult[23:8];
                    temp_mult <= mag_z_reg * sin_roll;
                    state <= COMP_Y5;
                end
                
                COMP_Y5: begin
                    temp_mult <= $signed(temp_mult[23:8]) * cos_pitch;
                    state <= COMP_Y6;
                end
                
                COMP_Y6: begin
                    mag_y_comp <= y_term1 + y_term2 - temp_mult[23:8];
                    state <= DONE;
                end
                
                DONE: begin
                    data_valid <= 1;
                    state <= IDLE;
                end
                
                default: state <= IDLE;
            endcase
        end
    end

endmodule