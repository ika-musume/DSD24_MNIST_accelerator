module DSDMNIST #(
    //basic information
    parameter IMGNUM = 0,
    parameter FPW = 0,
    parameter ROMPATH = "",
    parameter ROMHEX  = "",

    //address/data width
    parameter  IAW = $clog2(IMGNUM * 784), //input image ROM address width
    parameter  IDW = FPW, //input image ROM address width
    parameter  OAW = $clog2(IMGNUM * 10), //output RAM address width
    parameter  ODW = FPW //output RAM data width
)(
    //system interface
    input   wire                i_CLK,
    input   wire                i_RST_n,
    input   wire                i_START,
    output  wire                o_IRQ_DONE,
    output  wire                o_LED_DONE,

    //output buffer interface
    output  wire                o_OUTBUF_WE,
    output  wire    [OAW-1:0]   o_OUTBUF_ADDR,
    output  wire    [ODW-1:0]   o_OUTBUF_DATA
);



///////////////////////////////////////////////////////////
//////  INTERCONNECTIONS
////

//control wires
wire                imgrom_rd;
wire    [IAW-2:0]   imgrom_addr;
wire                wrom_rd;
wire    [8:0]       wrom_addr;
wire    [4:0]       bc_datasel;
wire                mac_rst, mac_relu_en, mac_shft, mac_add;
wire    [1:0]       acc_addr;
wire                acc_rst, acc_rd, acc_wr;
wire                bufwr_start, bufwr_done;

//data wires
wire    [2*IDW-1:0]     imgrom_do;
wire    [18*128-1:0]    wrom_do; //weight rom output 
wire    [24*8-1:0]      mac_data_row; //mac data input(layer buffer)
wire    [24*16-1:0]     acc_data_col; //acc data output 16*4

//FUCK YOU
`define PACK_ARRAY(PK_W, PK_H, PK_S, PK_D, pk_iter) \
genvar pk_iter; \
generate \
    for(pk_iter=0; pk_iter<PK_H; pk_iter=pk_iter+1) begin \
        assign PK_D[(PK_W*pk_iter) +: PK_W] = PK_S[pk_iter][(PK_W-1):0]; \
    end \
endgenerate

`define UNPACK_ARRAY(UNPK_W, UNPK_H, UNPK_S, UNPK_D, unpk_iter) \
genvar unpk_iter; \
generate \
    for(unpk_iter=0; unpk_iter<UNPK_H; unpk_iter=unpk_iter+1) begin \
        assign UNPK_D[unpk_iter][(UNPK_W-1):0] = UNPK_S[(UNPK_W*unpk_iter) +: UNPK_W]; \
    end \
endgenerate



///////////////////////////////////////////////////////////
//////  IMAGE ROM
////

DSDMNIST_sequencer #(
    .IMGNUM(IMGNUM), .IAW(IAW)
) sequencer_main (
    .i_CLK                      (i_CLK                      ),
    .i_RST_n                    (i_RST_n                    ),
    .i_START                    (i_START                    ),
    .o_IRQ_DONE                 (o_IRQ_DONE                 ),
    .o_LED_DONE                 (o_LED_DONE                 ),


    .o_IMGROM_RD                (imgrom_rd                  ),
    .o_IMGROM_ADDR              (imgrom_addr                ),
    .o_WROM_RD                  (wrom_rd                    ),
    .o_WROM_ADDR                (wrom_addr                  ),
    .o_BC_DATASEL               (bc_datasel                 ),
    .o_MAC_RST                  (mac_rst                    ),
    .o_MAC_RELU_EN              (mac_relu_en                ),
    .o_MAC_SHFT                 (mac_shft                   ),
    .o_MAC_ADD                  (mac_add                    ),    
    .o_ACC_RST                  (acc_rst                    ),
    .o_ACC_RD                   (acc_rd                     ),
    .o_ACC_WR                   (acc_wr                     ),
    .o_BUFWR_START              (bufwr_start                ),
    .i_BUFWR_DONE               (bufwr_done                 )
);



///////////////////////////////////////////////////////////
//////  IMAGE ROM
////

DSDMNIST_imgrom #(
    .IAW(IAW), .IDW(IDW),
    .ROMPATH(ROMPATH),
    .ROMHEX(ROMHEX)
) imgrom_main (
    .i_CLK                      (i_CLK                      ),

    .i_RD                       (imgrom_rd                  ),
    .i_ADDR                     (imgrom_addr                ),
    .o_DO0                      (imgrom_do[IDW*0+:IDW]      ),
    .o_DO1                      (imgrom_do[IDW*1+:IDW]      )
);



///////////////////////////////////////////////////////////
//////  WEIGHT ROM
////

//`define DSDMNIST_SIMULATION
DSDMNIST_wrom #(
    .ROMPATH(ROMPATH)
) wrom_main (
    .i_CLK                      (i_CLK                      ),

    .i_RD                       (wrom_rd                    ),
    .i_ADDR                     (wrom_addr                  ),
    .o_DO_PACKED                (wrom_do                    )
);



///////////////////////////////////////////////////////////
//////  BROADCASTER
////


DSDMNIST_broadcaster bc_main (
    .i_CLK                      (i_CLK                      ),
    .i_ACC_WR                   (acc_wr                     ),
    .i_DATASEL                  (bc_datasel                 ),
    .o_ACC_ADDR                 (acc_addr                   ),

    .i_IMGROM_PACKED            ({imgrom_do[(IDW*1+8)+:8], imgrom_do[(IDW*0+8)+:8]}), //intended truncation
    .i_BUFDATA_PACKED           (acc_data_col               ),
    .o_MAC_DATA_ROW_PACKED      (mac_data_row               )
);



///////////////////////////////////////////////////////////
//////  OPERATOR
////

DSDMNIST_operator op_main (
    .i_CLK                      (i_CLK                      ),

    .i_MAC_RST                  (mac_rst                    ),

    .i_MAC_RELU_EN              (mac_relu_en                ),
    .i_MAC_SHFT                 (mac_shft                   ),
    .i_MAC_ADD                  (mac_add                    ),

    .i_ACC_ADDR                 (acc_addr                   ),
    .i_ACC_RST                  (acc_rst                    ),
    .i_ACC_RD                   (acc_rd                     ),
    .i_ACC_WR                   (acc_wr                     ),

    .i_MAC_WEIGHT_PACKED        (wrom_do                    ),
    .i_MAC_DATA_ROW_PACKED      (mac_data_row               ),
    .o_ACC_DATA_COL_PACKED      (acc_data_col               )
);



///////////////////////////////////////////////////////////
//////  BUFFER I/F
////

DSDMNIST_bufif #(.OAW(OAW)) bufif_main (
    .i_CLK                      (i_CLK                      ),
    .i_RST_n                    (i_RST_n                    ),
    .i_BUFWR_START              (bufwr_start                ),

    .i_ACC_DATA_PACKED          (acc_data_col               ),
    .o_BUF_ADDR                 (o_OUTBUF_ADDR              ),
    .o_BUF_DATA                 (o_OUTBUF_DATA              ),
    .o_BUF_EN                   (o_OUTBUF_WE                ),
    .o_BUFWR_DONE               (bufwr_done                 )
);

endmodule

module DSDMNIST_bufif #(
    parameter OAW = 0
) (
    input   wire                i_CLK,
    input   wire                i_RST_n,
    input   wire                i_BUFWR_START,

    input   wire    [24*16-1:0] i_ACC_DATA_PACKED,
    output  wire    [OAW-1:0]   o_BUF_ADDR,
    output  wire    [31:0]      o_BUF_DATA,
    output  wire                o_BUF_EN,
    output  wire                o_BUFWR_DONE
);

wire signed [23:0]  i_ACC_DATA[0:15];
`UNPACK_ARRAY(24, 16, i_ACC_DATA_PACKED, i_ACC_DATA, unpk_acc_data)

integer i;
reg     [9:0]       en_pipeline;
reg     [31:0]      scratchpad[0:9];
assign  o_BUF_DATA = scratchpad[0];
assign  o_BUFWR_DONE = en_pipeline[9];
always @(posedge i_CLK) begin
    en_pipeline[0] <= i_BUFWR_START;
    en_pipeline[9:1] <= en_pipeline[8:0];

    if(i_BUFWR_START) begin
        for(i=0; i<10; i=i+1) begin
            scratchpad[i] <= $unsigned({{8{i_ACC_DATA[i][23]}}, i_ACC_DATA[i]}); //sign extension
        end
    end
    else begin
        for(i=0; i<9; i=i+1) begin
            scratchpad[i] <= scratchpad[i+1];
        end
        scratchpad[9] <= 32'd0;
    end
end

reg                 buf_addr_cnt;
reg     [OAW-1:0]   buf_addr;
assign  o_BUF_EN = buf_addr_cnt;
assign  o_BUF_ADDR = buf_addr;
always @(posedge i_CLK) begin
    if(!i_RST_n) begin
        buf_addr_cnt <= 1'b0;
        buf_addr <= {OAW{1'b0}};
    end
    else begin
        if(i_BUFWR_START) buf_addr_cnt <=  1'b1;
        else begin
            if(en_pipeline[9]) buf_addr_cnt <= 1'b0;
        end

        if(buf_addr_cnt) begin
            buf_addr <= buf_addr + {{(OAW-1){1'b0}}, 1'b1};
        end
    end
end


endmodule

module DSDMNIST_sequencer #(
    parameter IMGNUM = 0,
    parameter IAW = 0
) (
    input   wire                i_CLK,
    input   wire                i_RST_n,
    input   wire                i_START,
    output  wire                o_IRQ_DONE,
    output  wire                o_LED_DONE,

    output  wire                o_IMGROM_RD,
    output  wire    [IAW-2:0]   o_IMGROM_ADDR,
    output  wire                o_WROM_RD,
    output  wire    [8:0]       o_WROM_ADDR,
    output  wire    [4:0]       o_BC_DATASEL,
    output  wire                o_MAC_RST, o_MAC_RELU_EN, o_MAC_SHFT, o_MAC_ADD,
    output  wire                o_ACC_RST, o_ACC_RD, o_ACC_WR,
    output  wire                o_BUFWR_START,
    input   wire                i_BUFWR_DONE
);


/*
    INPUT SWITCH PEDGE DETECTOR
*/
reg     [2:0]   sw_syncchain;
wire            sw_pdet = ~sw_syncchain[2] & sw_syncchain[1];
always @(posedge i_CLK) begin
    if(!i_RST_n) sw_syncchain <= 3'b000;
    else begin
        sw_syncchain[2:1] <= sw_syncchain[1:0];
        sw_syncchain[0] <= i_START;
    end
end


/*
    FLOW SEQUENCER
*/
parameter   ICW = $clog2(IMGNUM);
reg     [ICW:0]     img_cntr;
reg     [2:0]       layer_status;
reg                 layer_calc_start;
reg                 layer_calc_done;
always @(posedge i_CLK) begin
    if(!i_RST_n) begin
        img_cntr <= {(ICW+1){1'b0}};
        layer_status <= 3'd0;
        layer_calc_start <= 1'b0;
    end
    else begin
        if(layer_status == 3'd0) begin
            if(sw_pdet) begin
                layer_status <= 3'd1;
                layer_calc_start <= 1'b1;
                img_cntr <= img_cntr + {{(ICW){1'b0}}, 1'b1}; //++
            end
            else layer_calc_start <= 1'b0;
        end
        else begin
            if(layer_calc_done) begin
                if(layer_status == 3'd6) begin
                    if(img_cntr != IMGNUM) begin
                        layer_status <= 3'd1;
                        layer_calc_start <= 1'b1;
                        img_cntr <= img_cntr + {{(ICW){1'b0}}, 1'b1}; //++
                    end
                    else begin
                        layer_status <= 3'd0;
                        img_cntr <= {(ICW+1){1'b0}}; //reset image counter
                    end
                end
                else begin
                    layer_status <= layer_status + 3'd1;
                    layer_calc_start <= 1'b1;
                end
            end
            else layer_calc_start <= 1'b0;
        end
    end
end


/*
    LAYER CALCULATION SEQUENCER
*/

//master counter
reg                 layer_run;
reg     [8:0]       layer_cntr;
reg                 mac_done;
//reg     [2:0]       mac_done_dlyd;
always @(posedge i_CLK) begin
    //mac_done_dlyd[0] <= mac_done;
    //mac_done_dlyd[2:1] <= mac_done_dlyd[1:0];

    if(!i_RST_n) begin
        layer_run <= 1'b0;
        layer_cntr <= 9'd511;
        mac_done <= 1'b0;
    end
    else begin
        if(!layer_run) begin
            if(layer_calc_start) begin
                layer_run <= 1'b1;
                layer_cntr <= layer_cntr + 9'd1;
            end
            mac_done <= 1'b0;
        end
        else begin
            //layer 1: 392 cycles
            if(layer_status == 3'd1) begin
                if(layer_cntr != 9'd391) begin
                    layer_cntr <= layer_cntr + 9'd1;
                    mac_done <= 1'b0;
                end
                else begin
                    layer_run <= 1'b0;
                    layer_cntr <= 9'd511;
                    mac_done <= 1'b1;
                end
            end
            //layer 2: 16 cycles
            else if(layer_status == 3'd2) begin
                if(layer_cntr != 9'd15) begin
                    layer_cntr <= layer_cntr + 9'd1;
                    mac_done <= 1'b0;
                end
                else begin
                    layer_run <= 1'b0;
                    layer_cntr <= 9'd511;
                    mac_done <= 1'b1;
                end
            end
            //layer 3: 8 cycles
            else if(layer_status == 3'd3) begin
                if(layer_cntr != 9'd7) begin
                    layer_cntr <= layer_cntr + 9'd1;
                    mac_done <= 1'b0;
                end
                else begin
                    layer_run <= 1'b0;
                    layer_cntr <= 9'd511;
                    mac_done <= 1'b1;
                end
            end
            //layer 4: 4 cycles
            else if(layer_status == 3'd4) begin
                if(layer_cntr != 9'd3) begin
                    layer_cntr <= layer_cntr + 9'd1;
                    mac_done <= 1'b0;
                end
                else begin
                    layer_run <= 1'b0;
                    layer_cntr <= 9'd511;
                    mac_done <= 1'b1;
                end
            end
            //layer 5: 2 cycles
            else if(layer_status == 3'd5) begin
                if(layer_cntr != 9'd1) begin
                    layer_cntr <= layer_cntr + 9'd1;
                    mac_done <= 1'b0;
                end
                else begin
                    layer_run <= 1'b0;
                    layer_cntr <= 9'd511;
                    mac_done <= 1'b1;
                end
            end
            //final output: 11 cycles
            else if(layer_status == 3'd6) begin
                if(layer_cntr != 9'd0) begin
                    layer_cntr <= layer_cntr + 9'd1;
                    mac_done <= 1'b0;
                end
                else begin
                    layer_run <= 1'b0;
                    layer_cntr <= 9'd511;
                    mac_done <= 1'b1;
                end
            end
        end
    end
end

//imgrom and weightrom address counter
wire                imgrom_rd = layer_run;
reg     [IAW-2:0]   imgrom_addr;
wire                wrom_rd = layer_run;
reg     [8:0]       wrom_addr;
wire                acc_rd = layer_run && (layer_status != 3'd1);
assign  o_IMGROM_RD = imgrom_rd;
assign  o_IMGROM_ADDR = imgrom_addr;
assign  o_WROM_RD = wrom_rd;
assign  o_WROM_ADDR = wrom_addr;
assign  o_ACC_RD = acc_rd;
always @(posedge i_CLK) begin
    if(!i_RST_n) begin
        imgrom_addr <= {(IAW-1){1'b0}};
        wrom_addr <= 9'd0;
    end
    else begin
        if(layer_run && layer_status == 3'd1) imgrom_addr <= imgrom_addr + {{(IAW-2){1'b0}}, 1'b1};
        if(layer_calc_done && layer_status == 3'd6) wrom_addr <= 9'd0;
        else begin
            if(layer_run) wrom_addr <= wrom_addr + 9'd1;
        end
    end
end

//layer buffer address control
reg     [4:0]       acc_ctrl;
reg     [4:0]       bc_datasel;
assign  o_BC_DATASEL = bc_datasel;
always @(*) begin
    if(layer_run) begin
             if(layer_status == 3'd1) bc_datasel = 5'b00100;
        else if(layer_status == 3'd2) bc_datasel = {1'b1, layer_cntr[3:0]};
        else if(layer_status == 3'd3) bc_datasel = {1'b1, layer_cntr[3:0]};
        else if(layer_status == 3'd4) bc_datasel = {2'b01, layer_cntr[2:0]};
        else if(layer_status == 3'd5) bc_datasel = {2'b01, layer_cntr[2:0]};
        else if(layer_status == 3'd6) bc_datasel = 5'b00100;
        else                          bc_datasel = 5'b00100;
    end
    else begin
        bc_datasel = {3'b001, acc_ctrl[4:3]};
    end
end

//mac accumulation control
reg                 mac_rst, mac_relu_en, mac_add;
assign  o_MAC_RST = mac_rst;
assign  o_MAC_RELU_EN = mac_relu_en;
assign  o_MAC_ADD = mac_add;
always @(posedge i_CLK) begin
    if(!i_RST_n) begin
        mac_rst <= 1'b1;
        mac_relu_en <= 1'b0;
        mac_add <= 1'b0;
    end
    else begin
        mac_rst <= 1'b0;

        if(layer_run) begin
            mac_relu_en <= 1'b1;
            mac_add <= 1'b1;
        end
        else begin
            mac_relu_en <= 1'b0;
            mac_add <= 1'b0;
        end
    end
end

assign  o_MAC_SHFT = acc_ctrl[2];
assign  o_ACC_WR = acc_ctrl[1];
assign  o_ACC_RST = acc_ctrl[0];
reg     [3:0]       acc_op_cntr;
reg                 acc_op_run;
always @(posedge i_CLK) begin
    if(!i_RST_n) begin
        acc_op_cntr <= 4'd15;
        acc_op_run <= 1'b0;
        layer_calc_done <= 1'b0;
    end
    else begin
        if(!acc_op_run) begin 
            acc_op_cntr <= 4'd15;
            if(mac_done) begin
                if(layer_status == 3'd6) layer_calc_done <= 1'b1;
                else begin
                    acc_op_cntr <= 4'd0;
                    acc_op_run <= 1'b1;
                end
            end
            else begin
                layer_calc_done <= 1'b0;
            end
        end
        else begin
            if(acc_op_cntr == 4'd11) begin
                acc_op_cntr <= 4'd15;
                acc_op_run <= 1'b0;
                layer_calc_done <= 1'b1;
            end
            else begin
                acc_op_cntr <= acc_op_cntr + 4'd1;
                layer_calc_done <= 1'b0;
            end
        end
    end

    if(layer_status == 3'd1) case(acc_op_cntr)
        //                 bank  shft   wr    rst
        4'h0: acc_ctrl <= {2'd0, 1'b0, 1'b1, 1'b1};
        4'h1: acc_ctrl <= {2'd1, 1'b0, 1'b1, 1'b1};
        4'h2: acc_ctrl <= {2'd2, 1'b0, 1'b1, 1'b1};
        4'h3: acc_ctrl <= {2'd3, 1'b0, 1'b1, 1'b1};
        4'h4: acc_ctrl <= {2'd3, 1'b1, 1'b1, 1'b0};
        4'h5: acc_ctrl <= {2'd2, 1'b1, 1'b1, 1'b0};
        4'h6: acc_ctrl <= {2'd1, 1'b1, 1'b1, 1'b0};
        4'h7: acc_ctrl <= {2'd0, 1'b1, 1'b1, 1'b0};
        4'h8: acc_ctrl <= {2'd3, 1'b1, 1'b1, 1'b0};
        4'h9: acc_ctrl <= {2'd2, 1'b1, 1'b1, 1'b0};
        4'hA: acc_ctrl <= {2'd1, 1'b1, 1'b1, 1'b0};
        4'hB: acc_ctrl <= {2'd0, 1'b1, 1'b1, 1'b0};
        4'hF: acc_ctrl <= {2'd0, 1'b0, 1'b0, 1'b0};
        default: acc_ctrl <= {2'd0, 1'b0, 1'b0, 1'b0};
    endcase
    else if(layer_status == 3'd2 || layer_status == 3'd3) case(acc_op_cntr)
        //                 bank  shft   wr    rst
        4'h0: acc_ctrl <= {2'd0, 1'b0, 1'b1, 1'b1};
        4'h1: acc_ctrl <= {2'd1, 1'b0, 1'b1, 1'b1};
        4'h2: acc_ctrl <= {2'd2, 1'b0, 1'b1, 1'b1};
        4'h3: acc_ctrl <= {2'd3, 1'b0, 1'b1, 1'b1};
        4'h4: acc_ctrl <= {2'd1, 1'b1, 1'b1, 1'b0};
        4'h5: acc_ctrl <= {2'd0, 1'b1, 1'b1, 1'b0};
        4'h6: acc_ctrl <= {2'd1, 1'b1, 1'b1, 1'b0};
        4'h7: acc_ctrl <= {2'd0, 1'b1, 1'b1, 1'b0};
        4'h8: acc_ctrl <= {2'd1, 1'b1, 1'b1, 1'b0};
        4'h9: acc_ctrl <= {2'd0, 1'b1, 1'b1, 1'b0};
        4'hA: acc_ctrl <= {2'd1, 1'b1, 1'b1, 1'b0};
        4'hB: acc_ctrl <= {2'd0, 1'b1, 1'b1, 1'b0};
        4'hF: acc_ctrl <= {2'd0, 1'b0, 1'b0, 1'b0};
        default: acc_ctrl <= {2'd0, 1'b0, 1'b0, 1'b0};
    endcase
    else if(layer_status == 3'd4 || layer_status == 3'd5) case(acc_op_cntr)
        //                 bank  shft   wr    rst
        4'h0: acc_ctrl <= {2'd0, 1'b0, 1'b1, 1'b1};
        4'h1: acc_ctrl <= {2'd1, 1'b0, 1'b1, 1'b1};
        4'h2: acc_ctrl <= {2'd2, 1'b0, 1'b1, 1'b1};
        4'h3: acc_ctrl <= {2'd3, 1'b0, 1'b1, 1'b1};
        4'h4: acc_ctrl <= {2'd0, 1'b1, 1'b1, 1'b0};
        4'h5: acc_ctrl <= {2'd0, 1'b1, 1'b1, 1'b0};
        4'h6: acc_ctrl <= {2'd0, 1'b1, 1'b1, 1'b0};
        4'h7: acc_ctrl <= {2'd0, 1'b1, 1'b1, 1'b0};
        4'h8: acc_ctrl <= {2'd0, 1'b1, 1'b1, 1'b0};
        4'h9: acc_ctrl <= {2'd0, 1'b1, 1'b1, 1'b0};
        4'hA: acc_ctrl <= {2'd0, 1'b1, 1'b1, 1'b0};
        4'hB: acc_ctrl <= {2'd0, 1'b1, 1'b1, 1'b0};
        4'hF: acc_ctrl <= {2'd0, 1'b0, 1'b0, 1'b0};
        default: acc_ctrl <= {2'd0, 1'b0, 1'b0, 1'b0};
    endcase
    else acc_ctrl <= {2'd0, 1'b0, 1'b0, 1'b0};
end

assign  o_BUFWR_START = layer_calc_done && (layer_status == 3'd6);

//hold irq pulse for 6 cycles
reg     [5:0]   irq_sr;
reg             led;
assign  o_IRQ_DONE = |{irq_sr};
assign  o_LED_DONE = led;
always @(posedge i_CLK) begin
    irq_sr[0] <= i_BUFWR_DONE && layer_status == 3'd0;
    irq_sr[5:1] <= irq_sr[4:0];

    if(!i_RST_n) led <= 1'b0;
    else begin
        if(o_IRQ_DONE) led <= 1'b1;
    end
end

endmodule


module DSDMNIST_imgrom #(
    parameter IAW = 0,
    parameter IDW = 0,
    parameter ROMPATH = "",
    parameter ROMHEX = ""
) (
    input   wire                i_CLK,

    input   wire                i_RD,

    input   wire    [IAW-2:0]   i_ADDR,
    output  wire    [IDW-1:0]   o_DO0,
    output  wire    [IDW-1:0]   o_DO1
);

reg     [IDW-1:0]   imgrom[0:(2**IAW)-1];
reg     [IDW-1:0]   imgrom_dout0, imgrom_dout1;
assign  o_DO0 = imgrom_dout0;
assign  o_DO1 = imgrom_dout1;

always @(posedge i_CLK) begin
    if(i_RD) imgrom_dout0 <= imgrom[{i_ADDR, 1'b0}];
    if(i_RD) imgrom_dout1 <= imgrom[{i_ADDR, 1'b1}];
end

//initialize bram
initial begin
    $readmemh({ROMPATH, ROMHEX}, imgrom); //load
end

endmodule

module DSDMNIST_operator (
    input   wire                    i_CLK,
    
    //MAC control
    input   wire                    i_MAC_RST,
    input   wire                    i_MAC_RELU_EN,
    input   wire                    i_MAC_SHFT,
    input   wire                    i_MAC_ADD,

    //ACC control
    input   wire    [1:0]           i_ACC_ADDR,
    input   wire                    i_ACC_RST,
    input   wire                    i_ACC_RD, i_ACC_WR,

    //MAC weight
    input   wire    [18*128-1:0]    i_MAC_WEIGHT_PACKED,

    //MAC data
    input   wire    [24*8-1:0]      i_MAC_DATA_ROW_PACKED,

    //ACC output
    output  wire    [24*16-1:0]     o_ACC_DATA_COL_PACKED
);

wire signed [17:0]  i_MAC_WEIGHT[0:128];
`UNPACK_ARRAY(18, 128, i_MAC_WEIGHT_PACKED, i_MAC_WEIGHT, unpk_wrom)

wire signed [23:0]  i_MAC_DATA_ROW[0:7];
`UNPACK_ARRAY(24, 8, i_MAC_DATA_ROW_PACKED, i_MAC_DATA_ROW, unpk_data_row)

wire signed [23:0]  o_ACC_DATA_COL[0:15];
`PACK_ARRAY(24, 16, o_ACC_DATA_COL, o_ACC_DATA_COL_PACKED, pk_acc_data_col)

//declare MAC interconnections
wire signed [41:0]  mac_interconnection[0:8][0:15];

genvar zeros;
generate
for(zeros=0; zeros<16; zeros=zeros+1) begin
    assign  mac_interconnection[0][zeros] = 42'sd0;
end
endgenerate


//declare ACC output
wire signed [47:0]  acc_out[0:15];

genvar i, j;
generate
for(i=0; i<16; i=i+1) begin : mac_column
    for(j=0; j<9; j=j+1) begin : mac_row
        if(j == 8) begin
            DSDMNIST_acc ACC_ELEMENT (
                .i_CLK                      (i_CLK                      ),
                .i_ACC_ADDR                 (i_ACC_ADDR                 ),
                .i_ACC_RST                  (i_ACC_RST                  ),
                .i_ACC_RD                   (i_ACC_RD                   ),
                .i_ACC_WR                   (i_ACC_WR                   ),

                .i_ACC_IN                   (mac_interconnection[j][i]  ),
                .o_ACC_OUT                  (acc_out[i]                 )
            );

            assign  o_ACC_DATA_COL[i] = acc_out[i][39:16]; //truncation
        end
        else begin
            DSDMNIST_mac MAC_ELEMENT (
                .i_CLK                      (i_CLK                      ),
                .i_MAC_RST                  (i_MAC_RST                  ),
                
                .i_MAC_RELU_EN              (i_MAC_RELU_EN              ),
                .i_MAC_SHFT                 (i_MAC_SHFT                 ),
                .i_MAC_ADD                  (i_MAC_ADD                  ),

                .i_MUL_OP1                  (i_MAC_WEIGHT[16*j+i]       ),
                .i_MUL_OP2                  (i_MAC_DATA_ROW[j]          ),
                .i_ACC_IN                   (mac_interconnection[j][i]  ),
                .o_ACC_OUT                  (mac_interconnection[j+1][i])
            );
        end
    end
end
endgenerate

endmodule

module DSDMNIST_broadcaster (
    input   wire                i_CLK,
    input   wire                i_ACC_WR, //acc write mode
    input   wire    [4:0]       i_DATASEL, //acc data sel
    output  reg     [1:0]       o_ACC_ADDR, //acc mem addr output

    input   wire    [15:0]      i_IMGROM_PACKED, //imgrom data
    input   wire    [24*16-1:0] i_BUFDATA_PACKED, //acc data array(24bit*16)

    output  wire    [24*8-1:0]  o_MAC_DATA_ROW_PACKED //mac data
);

wire        [7:0]   i_IMGROM[0:1];
`UNPACK_ARRAY(8, 2, i_IMGROM_PACKED, i_IMGROM, unpk_imgrom)

reg signed  [23:0]  o_MAC_DATA_ROW[0:7];
`PACK_ARRAY(24, 8, o_MAC_DATA_ROW, o_MAC_DATA_ROW_PACKED, pk_mac_data_row)

wire signed [23:0]  i_BUFDATA[0:15];
`UNPACK_ARRAY(24, 16, i_BUFDATA_PACKED, i_BUFDATA, unpk_bufdata)


/*
    boradcaster

    1XXXX - broadcasting mode 2/2/2/2
     ||||> COLSEL[0]
     |||>  COLSEL[1]
     ||>   ROW[0]
     |>    ROW[1]

    01XXX - broadcasting mode 1/1/1/1/1/1/1/1
      |||> COLSEL[0]
      ||>  ROW[0]
      |>   ROW[1]

    001XX - broadcasting mode 4/4(imgrom)
*/

reg     [4:0]           datasel_z;
always @(posedge i_CLK) begin
    datasel_z <= i_DATASEL[4:0];
end

integer i;

always @(*) begin
    if(i_ACC_WR) begin
        o_ACC_ADDR = i_DATASEL[1:0];
        for(i=0; i<8; i=i+1) o_MAC_DATA_ROW[i] = 24'sd0; //force output zero
    end
    else begin
        if(datasel_z[4] == 1'b1) begin
            //broadcast four values to 2/2/2/2 rows of MACs
            o_ACC_ADDR = i_DATASEL[3:2];
            o_MAC_DATA_ROW[0] = i_BUFDATA[datasel_z[1:0]*4 + 0]; //VALUE 0
            o_MAC_DATA_ROW[1] = i_BUFDATA[datasel_z[1:0]*4 + 0]; //|
            o_MAC_DATA_ROW[2] = i_BUFDATA[datasel_z[1:0]*4 + 1]; //VALUE 1
            o_MAC_DATA_ROW[3] = i_BUFDATA[datasel_z[1:0]*4 + 1]; //|
            o_MAC_DATA_ROW[4] = i_BUFDATA[datasel_z[1:0]*4 + 2]; //VALUE 2
            o_MAC_DATA_ROW[5] = i_BUFDATA[datasel_z[1:0]*4 + 2]; //|
            o_MAC_DATA_ROW[6] = i_BUFDATA[datasel_z[1:0]*4 + 3]; //VALUE 3
            o_MAC_DATA_ROW[7] = i_BUFDATA[datasel_z[1:0]*4 + 3]; //|
        end
        else begin
            if(datasel_z[3] == 1'b1) begin
                //broadcast eight values to 1/1/1/1/1/1/1/1 rows of MACs
                o_ACC_ADDR = i_DATASEL[2:1];
                o_MAC_DATA_ROW[0] = i_BUFDATA[datasel_z[0]*8 + 0];
                o_MAC_DATA_ROW[1] = i_BUFDATA[datasel_z[0]*8 + 1];
                o_MAC_DATA_ROW[2] = i_BUFDATA[datasel_z[0]*8 + 2];
                o_MAC_DATA_ROW[3] = i_BUFDATA[datasel_z[0]*8 + 3];
                o_MAC_DATA_ROW[4] = i_BUFDATA[datasel_z[0]*8 + 4];
                o_MAC_DATA_ROW[5] = i_BUFDATA[datasel_z[0]*8 + 5];
                o_MAC_DATA_ROW[6] = i_BUFDATA[datasel_z[0]*8 + 6];
                o_MAC_DATA_ROW[7] = i_BUFDATA[datasel_z[0]*8 + 7];
            end
            else begin
                if(datasel_z[2] == 1'b1) begin
                    //broadcast image input to 4/4 rows of MACs
                    o_ACC_ADDR = i_DATASEL[1:0];
                    o_MAC_DATA_ROW[0] = $signed({8'h00, i_IMGROM[0], 8'h00});
                    o_MAC_DATA_ROW[1] = $signed({8'h00, i_IMGROM[0], 8'h00});
                    o_MAC_DATA_ROW[2] = $signed({8'h00, i_IMGROM[0], 8'h00});
                    o_MAC_DATA_ROW[3] = $signed({8'h00, i_IMGROM[0], 8'h00});
                    o_MAC_DATA_ROW[4] = $signed({8'h00, i_IMGROM[1], 8'h00});
                    o_MAC_DATA_ROW[5] = $signed({8'h00, i_IMGROM[1], 8'h00});
                    o_MAC_DATA_ROW[6] = $signed({8'h00, i_IMGROM[1], 8'h00});
                    o_MAC_DATA_ROW[7] = $signed({8'h00, i_IMGROM[1], 8'h00});
                end
                else begin
                    o_ACC_ADDR = i_DATASEL[1:0];
                    for(i=0; i<8; i=i+1) o_MAC_DATA_ROW[i] = 24'sd0; //force output zero
                end
            end
        end
    end
end

endmodule

module DSDMNIST_acc (
    input   wire                i_CLK,
    
    input   wire        [1:0]   i_ACC_ADDR,
    input   wire                i_ACC_RST,
    input   wire                i_ACC_WR, i_ACC_RD,
    input   wire signed [41:0]  i_ACC_IN,
    output  reg  signed [47:0]  o_ACC_OUT
);

//declare addressable accumulator using LUTRAM
reg signed  [47:0]  acc[0:3];

always @(posedge i_CLK) begin
    if(i_ACC_RST) begin
        acc[i_ACC_ADDR] <= 48'sd0;
    end
    else begin
        if(i_ACC_WR) acc[i_ACC_ADDR] <= acc[i_ACC_ADDR] + i_ACC_IN;
        else begin
            if(i_ACC_RD) o_ACC_OUT <= acc[i_ACC_ADDR];
        end
    end
end

endmodule

module DSDMNIST_mac (
    input   wire                i_CLK,
    input   wire                i_MAC_RST,
    
    input   wire                i_MAC_RELU_EN, //0 pipeline delay
    input   wire                i_MAC_SHFT, //0 pipeline delay
    input   wire                i_MAC_ADD, //2 pipeline delay

    input   wire signed [17:0]  i_MUL_OP1, //weights, Q2.16
    input   wire signed [23:0]  i_MUL_OP2, //intermediate values, Q8.16
    input   wire signed [41:0]  i_ACC_IN,

    output  wire signed [41:0]  o_ACC_OUT //Q10.32
);

reg                 mac_add_z, mac_add_zz;
always @(posedge i_CLK) begin
    mac_add_z <= i_MAC_ADD;
    mac_add_zz <= mac_add_z;
end

reg signed  [17:0] op1;
reg signed  [23:0] op2;
(* use_dsp = "yes" *) reg signed  [41:0]  mul;
(* use_dsp = "yes" *) reg signed  [41:0]  acc;
assign  o_ACC_OUT = acc;

always @(posedge i_CLK) begin
    if(i_MAC_RST) begin
        op1 <= 18'sd0; op2 <= 24'sd0;
        acc <= 42'sd0;
    end
    else begin
        op1 <= i_MUL_OP1;

        if(i_MAC_RELU_EN) op2 <= i_MUL_OP2[23] ? 24'sd0 : i_MUL_OP2;
        else op2 <= i_MUL_OP2;

        mul <= op1 * op2; 

        if(mac_add_zz) acc <= acc + mul;
        else begin
            if(i_MAC_SHFT) acc <= i_ACC_IN;
        end
    end
end

endmodule

module DSDMNIST_wrom #(parameter ROMPATH = "") (
    input   wire                    i_CLK,

    input   wire                    i_RD,
    input   wire    [8:0]           i_ADDR,
    output  wire    [18*128-1:0]    o_DO_PACKED
);



genvar rom;
generate
for(rom=0; rom<64; rom=rom+1) begin : wrom

//declare 18bit*1024 dual-port BRAM(RAMB18E1)
(* ram_style = "block" *) reg     [31:0]  wrom [0:1023];
reg     [31:0]  wrom_out0, wrom_out1;
assign  o_DO_PACKED[(rom*36)   +:18] = wrom_out0[17:0];
assign  o_DO_PACKED[(rom*36)+18+:18] = wrom_out1[17:0];

always @(posedge i_CLK) begin
    if(i_RD) wrom_out0 <= wrom[{1'b0, i_ADDR}]; //even MAC(0), lower half
    if(i_RD) wrom_out1 <= wrom[{1'b1, i_ADDR}]; //odd MAC(1), higher half
end
/*
reg     [31:0]  wrom0[0:511];
reg     [31:0]  wrom1[0:511];
reg     [31:0]  wrom_out0, wrom_out1;
always @(posedge i_CLK) begin
    if(i_RD) wrom_out0 <= wrom0[i_ADDR]; //even MAC(0), lower half
    if(i_RD) wrom_out1 <= wrom1[i_ADDR]; //odd MAC(1), higher half
end
*/

//initialize bram
initial begin
    //string                  filename;
    //$sformat(filename, , rom); //specify a filename
    $readmemh({ROMPATH, "MACWEIGHT_", (8'h30+(rom/10)), (8'h30+(rom%10)),".txt"}, wrom); //load
end

end
endgenerate



`ifdef DSDMNIST_SIMULATION
//make ROM initializer file
//string                  filename;
int     i, j, k, m;
reg     [128*32-1:0]    wrombuf[0:511];
reg     [31:0]          w1buf[0:(784*64)-1];
reg     [31:0]          w2buf[0:(64*32)-1];
reg     [31:0]          w3buf[0:(32*32)-1];
reg     [31:0]          w4buf[0:(32*16)-1];
reg     [31:0]          w5buf[0:(16*10)-1];
reg     [128*32-1:0]    writebuf;
reg     [31:0]          macbuf[0:1023];
initial begin
    if(ROMPATH != "") $readmemh({ROMPATH, "fixed_point_layer1_hex.txt"}, w1buf);
    if(ROMPATH != "") $readmemh({ROMPATH, "fixed_point_layer2_hex.txt"}, w2buf);
    if(ROMPATH != "") $readmemh({ROMPATH, "fixed_point_layer3_hex.txt"}, w3buf);
    if(ROMPATH != "") $readmemh({ROMPATH, "fixed_point_layer4_hex.txt"}, w4buf);
    if(ROMPATH != "") $readmemh({ROMPATH, "fixed_point_layer5_hex.txt"}, w5buf);

    /*
        ADDR(DEC)   128*32 VECTOR

        421     |10|10|10|10|10|10|10|10| layer 5 weight
        420     |10|10|10|10|10|10|10|10| layer 5 weight / 0001/1011/1100
        419     |16|16|16|16|16|16|16|16| layer 4 weight
                    ...........
        416     |16|16|16|16|16|16|16|16| layer 4 weight / 0001 1011 1000
                    ...........
        415     |-32--|--32-|-32--|--32-| layer 3 weight
                    ...........
        408     |-32--|--32-|-32--|--32-| layer 3 weight / 0001/1011/0000
                    ...........
        407     |-32--|--32-|-32--|--32-| layer 2 weight
                    ...........
        392     |-32--|--32-|-32--|--32-| layer 2 weight / 0001_1010_0000
                    ...........
        391     |----64-----|-----64----| layer 1 weight
                    ...........
          0     |----64-----|-----64----| layer 1 weight
               MSB                     LSB
    */

    //initialize weightbuf
    for(i=0; i<512; i=i+1) begin
        wrombuf[i] = 4096'd0;
    end

    for(i=0; i<1024; i=i+1) begin
        macbuf[i] = 32'd0;
    end

    //convert weight 1 buffer
    writebuf = 4096'd0;
    for(i=0; i<392; i=i+1) begin
        for(j=0; j<2; j=j+1) begin
            for(k=0; k<64; k=k+1) begin
                m = ((k*784) + ((i*2) + j));
                writebuf = writebuf | ({4064'd0, w1buf[m]} << ((j*64) + k)*32);
            end
        end
        wrombuf[i] = writebuf;
        writebuf = 4096'd0;
    end

    //convert weight 2 buffer
    writebuf = 4096'd0;
    for(i=0; i<16; i=i+1) begin
        for(j=0; j<4; j=j+1) begin
            for(k=0; k<32; k=k+1) begin
                m = ((k*64) + ((i*4) + j));
                writebuf = writebuf | ({4064'd0, w2buf[m]} << ((j*32) + k)*32);
            end
        end
        wrombuf[392+i] = writebuf;
        writebuf = 4096'd0;
    end

    //convert weight 3 buffer
    writebuf = 4096'd0;
    for(i=0; i<8; i=i+1) begin
        for(j=0; j<4; j=j+1) begin
            for(k=0; k<32; k=k+1) begin
                m = ((k*32) + ((i*4) + j));
                writebuf = writebuf | ({4064'd0, w3buf[m]} << ((j*32) + k)*32);
            end
        end
        wrombuf[408+i] = writebuf;
        writebuf = 4096'd0;
    end

    //convert weight 4 buffer
    writebuf = 4096'd0;
    for(i=0; i<4; i=i+1) begin
        for(j=0; j<8; j=j+1) begin
            for(k=0; k<16; k=k+1) begin
                m = ((k*32) + ((i*8) + j));
                writebuf = writebuf | ({4064'd0, w4buf[m]} << ((j*16) + k)*32);
            end
        end
        wrombuf[416+i] = writebuf;
        writebuf = 4096'd0;
    end

    //convert weight 5 buffer
    writebuf = 4096'd0;
    for(i=0; i<2; i=i+1) begin
        for(j=0; j<8; j=j+1) begin
            for(k=0; k<10; k=k+1) begin
                m = ((k*16) + ((i*8) + j));
                writebuf = writebuf | ({4064'd0, w5buf[m]} << ((j*16) + k)*32);
            end
        end
        wrombuf[420+i] = writebuf;
        writebuf = 4096'd0;
    end

    //split into 32-bit(effective width is 18-bit MAC buffer), odd+even column(to fill 18k bram completely)
    for(i=0; i<64; i=i+1) begin
        for(j=0; j<2; j=j+1) begin
            for(k=0; k<512; k=k+1) begin
                macbuf[j*512+k] = {wrombuf[k]}[(i*2+j)*32+:32];
            end
        end
        $sformat(filename, "MACWEIGHT_%0d.txt", i);
        $writememh({ROMPATH, filename}, macbuf);
    end

    //write
    //$writememh({ROMPATH, "WEIGHTROM.txt"}, wrombuf);

    //read
    //$readmemh({ROMPATH, "WEIGHTROM.txt"}, wrombuf);
    
end
`endif

endmodule