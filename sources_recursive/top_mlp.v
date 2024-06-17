`timescale 1ns / 1ps

module top_mlp #(
    parameter IN_IMG_NUM = 10,
	parameter FP_BW = 32,
	parameter INT_BW = 8,
    // parameter X_BUF_DATA_WIDTH = INT_BW*IN_IMG_NUM,  	// add in 2024-04-17 / if you try INT8 Streamline , you should change X_BUF_DATA_WIDTH to this line
	parameter X_BUF_DATA_WIDTH = FP_BW*IN_IMG_NUM,
	parameter X_BUF_DEPTH = 784*IN_IMG_NUM,
    // parameter W_BUF_DATA_WIDTH = INT_BW *IN_IMG_NUM,		// add in 2024-04-17 / if you try INT8 Streamline , you should change W_BUF_DATA_WIDTH to this line
	parameter W_BUF_DATA_WIDTH = FP_BW *IN_IMG_NUM, 	
	parameter W_BUF_DEPTH = 784*IN_IMG_NUM,
    parameter Y_BUF_DATA_WIDTH = 32,
	parameter Y_BUF_ADDR_WIDTH = 32,  							// add in 2023-05-10
    parameter Y_BUF_DEPTH = 10*IN_IMG_NUM * 4 					// modify in 2024-04-17, y_buf_addr has to increase +4 -> 0 - 396
)(
    //system interface
    input   wire                            clk,
    input   wire                            rst_n,
    input   wire                            start_i,
    output  wire                            done_intr_o,
    output  wire                            done_led_o,

    //output buffer interface
    output  wire                            y_buf_en,
    output  wire                            y_buf_wr_en,
    output  wire [Y_BUF_ADDR_WIDTH-1:0]     y_buf_addr,			// modify in 2023-05-10, [$clog2(Y_BUF_DEPTH)-1:0] -> [Y_BUF_ADDR_WIDTH-1:0]
    output  wire [Y_BUF_DATA_WIDTH-1:0]     y_buf_data
);

assign  y_buf_en = y_buf_wr_en;

parameter RBAW = $clog2(IN_IMG_NUM*10);
wire    [RBAW-1:0] resultbuf_addr;
wire    [Y_BUF_ADDR_WIDTH-1:0] resultbuf_addr_output = {{(Y_BUF_ADDR_WIDTH - RBAW - 2){1'b0}}, resultbuf_addr, 2'b00};
assign  y_buf_addr = resultbuf_addr_output;

DSDMNIST #(
    .IMGNUM(IN_IMG_NUM), .FPW(32),
    .ROMPATH("C:/Users/kiki1/Desktop/dsd2024/DSD24_Termprj_Provided_Materials/00_RTL_Skeleton/dsd_mlp.srcs/sources_1/new/roms/"),
    .ROMHEX("IMGROM10.txt")
) main (
    .i_CLK                      (clk                        ),
    .i_RST_n                    (rst_n                      ),
    .i_START                    (start_i                    ),
    .o_IRQ_DONE                 (done_intr_o                ),
    .o_LED_DONE                 (done_led_o                 ),
    
    .o_OUTBUF_WE                (y_buf_wr_en                ),
    .o_OUTBUF_ADDR              (resultbuf_addr             ),
    .o_OUTBUF_DATA              (y_buf_data                 )
);
    
endmodule
