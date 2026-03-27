/*
 * Copyright (c) 2024 Your Name
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

module dlatch(
    input wire [7:0] data,
    input wire en,
    input wire reset,
    output reg [7:0] q
);

always @* begin
    if (reset) q <= 0;
    else if (en) q <= data;
end

endmodule

module dp(
    input clk,
    input reset,
    input tx_done,
    input [7:0] tx_out,
    output [7:0] rx_data
);

dlatch d1(
    .data(tx_out),
    .en(tx_done),
    .reset(reset),
    .q(rx_data)
);

endmodule

module baud_rate_gen(
    input wire clk,
    input wire reset,
    output wire rxclk_en,
    output wire txclk_en
);

parameter CLK_FREQ = 50_000_000;
parameter BAUD_RATE = 115200;
localparam RX_ACC_MAX = CLK_FREQ/(BAUD_RATE*16);
localparam TX_ACC_MAX = CLK_FREQ/BAUD_RATE;

reg [15:0] rx_acc = 0;
reg [15:0] tx_acc = 0;

always @(posedge clk or posedge reset) begin
    if (reset) begin
        rx_acc <= 0;
        tx_acc <= 0;
    end else begin
        rx_acc <= (rx_acc >= RX_ACC_MAX-1) ? 0 : rx_acc + 1;
        tx_acc <= (tx_acc >= TX_ACC_MAX-1) ? 0 : tx_acc + 1;
    end
end

assign rxclk_en = (rx_acc == RX_ACC_MAX/2);
assign txclk_en = (tx_acc == TX_ACC_MAX/2);

endmodule

module tx_fsm(
    input wire clk,
    input wire reset,
    input wire [7:0] tx_data,
    input wire tx_start,
    input wire txclk_en,
    output reg tx_out,
    output reg tx_busy,
    output reg tx_done
);

localparam [3:0]
    IDLE  = 4'd0,
    START = 4'd1,
    DATA  = 4'd2,
    STOP  = 4'd3;

reg [3:0] state;
reg [2:0] bit_index;
reg [7:0] data_reg;

always @(posedge clk or posedge reset) begin
    if (reset) begin
        state <= IDLE;
        tx_out <= 1'b1;
        tx_busy <= 1'b0;
        tx_done <= 1'b0;
    end else begin
        case(state)
            IDLE: begin
                tx_out <= 1'b1;
                tx_done <= 1'b0;
                if (tx_start) begin
                    state <= START;
                    data_reg <= tx_data;
                    tx_busy <= 1'b1;
                end
            end
            
            START: if (txclk_en) begin
                tx_out <= 1'b0;
                state <= DATA;
                bit_index <= 0;
            end
            
            DATA: if (txclk_en) begin
                tx_out <= data_reg[bit_index];
                if (bit_index == 7) begin
                    state <= STOP;
                end else begin
                    bit_index <= bit_index + 1;
                end
            end
            
            STOP: if (txclk_en) begin
                tx_out <= 1'b1;
                tx_busy <= 1'b0;
                tx_done <= 1'b1;
                state <= IDLE;
            end
        endcase
    end
end

endmodule

module rx_fsm(
    input wire clk,
    input wire reset,
    input wire rx_in,
    input wire rxclk_en,
    output reg [7:0] rx_data,
    output reg rx_done,
    output reg rx_error
);

localparam [3:0]
    IDLE   = 4'd0,
    START  = 4'd1,
    DATA   = 4'd2,
    STOP   = 4'd3;

reg [3:0] state;
reg [2:0] bit_index;
reg [7:0] data_reg;
reg [3:0] sample_cnt;

always @(posedge clk or posedge reset) begin
    if (reset) begin
        state <= IDLE;
        rx_data <= 8'h00;
        rx_done <= 1'b0;
        rx_error <= 1'b0;
    end else begin
        case(state)
            IDLE: begin
                rx_done <= 1'b0;
                if (!rx_in) begin  // Start bit detection
                    state <= START;
                    sample_cnt <= 0;
                end
            end
            
            START: if (rxclk_en) begin
                if (sample_cnt == 7) begin  // Sample mid-point of start bit
                    if (!rx_in) begin
                        state <= DATA;
                        bit_index <= 0;
                        sample_cnt <= 0;
                    end else begin
                        state <= IDLE;  // False start
                    end
                end else begin
                    sample_cnt <= sample_cnt + 1;
                end
            end
            
            DATA: if (rxclk_en) begin
                if (sample_cnt == 15) begin  // Sample mid-point of data bit
                    data_reg[bit_index] <= rx_in;
                    if (bit_index == 7) begin
                        state <= STOP;
                    end else begin
                        bit_index <= bit_index + 1;
                    end
                    sample_cnt <= 0;
                end else begin
                    sample_cnt <= sample_cnt + 1;
                end
            end
            
            STOP: if (rxclk_en) begin
                if (!rx_in) begin
                    rx_error <= 1'b1;  // Missing stop bit
                end
                rx_data <= data_reg;
                rx_done <= 1'b1;
                state <= IDLE;
            end
        endcase
    end
end

endmodule

module tt_um_vlsi (
    input  wire [7:0] ui_in,    // Dedicated inputs
    output wire [7:0] uo_out,   // Dedicated outputs
    input  wire [7:0] uio_in,   // IOs: Input path
    output wire [7:0] uio_out,  // IOs: Output path
    output wire [7:0] uio_oe,   // IO enable
    input  wire       ena,
    input  wire       clk,
    input  wire       rst_n     // Active-low reset
);

wire txclk_en, rxclk_en;

// Baud rate generator with active-high reset
baud_rate_gen brg(
    .clk(clk),
    .reset(!rst_n),  // Invert reset polarity
    .rxclk_en(rxclk_en),
    .txclk_en(txclk_en)
);

// Transmitter FSM
tx_fsm transmitter(
    .clk(clk),
    .reset(!rst_n),
    .tx_data(ui_in),
    .tx_start(uio_in[0]),
    .txclk_en(txclk_en),
    .tx_out(uio_out[0]),
    .tx_busy(uio_out[1]),
    .tx_done(uio_out[2])
);

// Receiver FSM
rx_fsm receiver(
    .clk(clk),
    .reset(!rst_n),
    .rx_in(uio_in[1]),
    .rxclk_en(rxclk_en),
    .rx_data(uo_out),
    .rx_done(uio_out[3]),
    .rx_error(uio_out[4])
);

// Output enable configuration
assign uio_oe = 8'b00011111;  // Enable first 5 bits as outputs

// Tie remaining outputs to 0
assign uio_out[7:5] = 3'b0;
// List all unused inputs to prevent warnings
wire _unused = &{ena, clk, rst_n, 1'b0};

endmodule
