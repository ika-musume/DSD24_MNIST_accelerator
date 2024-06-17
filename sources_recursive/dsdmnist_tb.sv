`timescale 1ns / 1ns

module DSDMNIST_tb;

reg     clk = 1'b1;
reg     rst_n = 1'b0;
reg     start = 1'b0;

always #1 clk = ~clk;

parameter IMGNUM = 10;
parameter OAW = $clog2(IMGNUM * 10);
reg [31:0] outbuf [0:2**OAW-1];
wire                outbuf_we;
wire    [OAW-1:0]   outbuf_addr;
wire    [31:0]      outbuf_data;
always @(posedge clk) begin
    if(outbuf_we) outbuf[outbuf_addr] <= outbuf_data;
end



DSDMNIST #(
    .IMGNUM(IMGNUM), .FPW(32),
    .ROMPATH("C:/Users/kiki1/Desktop/dsd2024/DSD24_Termprj_Provided_Materials/00_RTL_Skeleton/dsd_mlp.srcs/sources_1/new/roms/"),
    .ROMHEX("IMGROM10.txt")
) dut (
    .i_CLK                      (clk                        ),
    .i_RST_n                    (rst_n                      ),
    .i_START                    (start                      ),
    .o_IRQ_DONE                 (                           ),
    .o_LED_DONE                 (                           ),

    .o_OUTBUF_WE                (outbuf_we                  ),
    .o_OUTBUF_ADDR              (outbuf_addr                ),
    .o_OUTBUF_DATA              (outbuf_data                )
);





initial begin
    #40 rst_n = 1'b1;
    #10 start = 1'b1;
    #40 start = 1'b0;
    #5000 start = 1'b1;
    #40 start = 1'b0;
end

endmodule