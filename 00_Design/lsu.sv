`include "data_memory.sv"
`include "load_unit.sv"
`include "mux3_1.sv"
module lsu(
  /////////- input -///////////
    input   logic       i_clk,
    input   logic       i_reset,

    input   logic [31:0]i_addr,
    input   logic [31:0]i_wdata,
    input   logic       i_wren,
    input   logic [2:0]  i_load_type,  // Chọn kiểu load
    
    output  logic [31:0] o_load_result
);

    logic [31:0] data_read_mem;

    data_memory data_memory_lsu (
        .i_clk(i_clk),
        .i_reset(i_reset),
        .i_addr(i_addr),
        .i_wdata(i_wdata),
        .i_wren(i_wren),
        .o_rdata(data_read_mem));

    load_unit load_unit_lsu(
        .i_load_data(data_read_mem),
        .i_load_type(i_load_type),
        .o_load_result(o_load_result));


endmodule
module lsu_2(
  //input
  input logic         i_clk,
  input logic         i_rst,
  input logic  [31:0] i_st_data,
  input logic  [31:0] i_lsu_addr,
  input logic         i_lsu_wren,
  input logic  [31:0] i_io_sw,
//  input logic [3:0] i_io_btn,
  input logic [2:0] slt_sl,
  //sram
 /* output logic o_stall,
  output logic [17:0]   o_SRAM_ADDR,
  inout  wire [15:0]   o_SRAM_DQ  ,
  output logic          o_SRAM_CE_N,
  output logic          o_SRAM_WE_N,
  output logic          o_SRAM_LB_N,
  output logic          o_SRAM_UB_N,
  output  logic          o_SRAM_OE_N, 
  */
  //out put
  output logic [31:0] o_ld_data ,
  output logic [31:0] o_io_ledr, 
  output logic [31:0] o_io_ledg ,
  output logic [6:0] o_io_hex0, 
  output logic [6:0] o_io_hex1, 
  output logic [6:0] o_io_hex2,   
  output logic [6:0] o_io_hex3, 
  output logic [6:0] o_io_hex4, 
  output logic [6:0] o_io_hex5, 
  output logic [6:0] o_io_hex6,   
  output logic [6:0] o_io_hex7, 
  output logic [31:0] o_io_lcd
  );
  logic en_datamem;
  logic en_op_buf;
  logic [31:0] data_out_1,data_out_2,io_sw;
  logic [31:0] INPUT;
  logic ACK;
always_ff @(posedge i_clk) begin
        INPUT <= i_io_sw; 
     end
assign io_sw = i_rst ? 32'd0: INPUT; 

  demux_sel_mem demux_1 (
    .i_lsu_addr(i_lsu_addr[15:0]),
    .en_datamem(en_datamem),
    .en_op_buf(en_op_buf)  
  );
  /*
  logic read_signal;
  logic write_signal;
  assign write_signal = ((slt_sl == 3'b010) & (~ACK)) ? 1'b1: 1'b0;
  assign read_signal = ((slt_sl == 3'b101) & (~ACK)) ? 1'b1: 1'b0; 
  assign o_stall = (en_datamem & ((slt_sl == 3'b101)|(slt_sl == 3'b010)) & (~ACK)) ? 1'b1: 1'b0;
  */
datamem mem (
    .i_clk(i_clk),
    .i_rst(i_rst),
    .i_wren(i_lsu_wren),
    .i_enb(en_datamem),
    .i_addr(i_lsu_addr[12:0]),
    .i_data(i_st_data),
    .o_data(data_out_1)
);

  /*
 data_mem mem_sdram
(
    .in_CLK(i_clk),
    .in_CSn(i_rst),              // CHIP SELECT (active low)
    .in_write_en(i_lsu_wren),         // 1 = Write, 0 = Read
    .in_CASn(1'b1),             // Column Address Strobe (active low)
    .in_RASn(~en_datamem),             // Row Address Strobe (active low)
    .in_bank_select(i_lsu_addr[13:12]),      // Bank selection (2-bit)
    .in_sdram_addr(i_lsu_addr[12:0]),      // Row address (13-bit)
    .in_sdram_write_data(i_st_data),
    .out_sdram_read_data(data_out_1),
    .o_stall(o_stall) 
);
*/					
/*  sram_IS61WV25616_controller_32b_3lr sram_mem (
    .i_ADDR (i_lsu_addr[17:0]),
    .i_WDATA (i_st_data),
    .i_BMASK (4'b1111),
    .i_WREN  (write_signal),
    .i_RDEN  (read_signal) ,
    .o_RDATA (data_out_1) ,
    .o_ACK   (ACK) ,
    .SRAM_ADDR(o_SRAM_ADDR),
    .SRAM_DQ  (o_SRAM_DQ),
    .SRAM_CE_N (o_SRAM_CE_N),
    .SRAM_WE_N (o_SRAM_WE_N),
    .SRAM_LB_N (o_SRAM_LB_N),
    .SRAM_UB_N (o_SRAM_UB_N),
    .SRAM_OE_N (o_SRAM_OE_N),
    .i_clk (i_clk),
    .i_reset (en_datamem)
);
*/
  output_buffer  outputperiph (
    .slt_sl (slt_sl),
    .st_data_2_i   (i_st_data), 
    .addr_2_i      (i_lsu_addr[15:0]),
    .en_bf         (en_op_buf), 
    .st_en_2_i     (i_lsu_wren),
    .i_clk         (i_clk), 
    .i_rst         (i_rst),
    .data_out_2_o  (data_out_2), 
    .io_lcd_o      (o_io_lcd), 
    .io_ledg_o     (o_io_ledg), 
    .io_ledr_o     (o_io_ledr), 
    .io_hex0_o     (o_io_hex0), 
    .io_hex1_o     (o_io_hex1), 
    .io_hex2_o     (o_io_hex2), 
    .io_hex3_o     (o_io_hex3), 
    .io_hex4_o     (o_io_hex4), 
    .io_hex5_o     (o_io_hex5), 
    .io_hex6_o     (o_io_hex6), 
    .io_hex7_o     (o_io_hex7)
	  );
  mux_3_1_lsu mux31  (
    .in_data_3_i(data_out_1), 
    .in_data_2_i(data_out_2), 
    .in_data_1_i(io_sw), 
    .i_lsu_addr(i_lsu_addr[15:0]),
    .o_ld_data(o_ld_data)
    );					
endmodule

module datamem (
    input logic i_clk,
    input logic i_rst,
    input logic i_wren,
    input logic i_enb,
    input logic [12:0] i_addr,
    input logic [31:0] i_data,
    output logic [31:0] o_data
);
logic [31:0] data_mem [2048:0];
always_ff @ (posedge i_clk) begin
    if (i_enb && i_wren) begin
        data_mem[i_addr[12:2]] <= i_data;
     end
    end
assign o_data = (i_rst == 1'b1) ? 32'h0: data_mem[i_addr[12:2]];
endmodule

module demux_sel_mem (
    input logic [15:0] i_lsu_addr,
    output logic en_datamem,
    output logic en_op_buf  
);
parameter start_datamem = 16'h2000;
parameter end_datamem = 16'h4000;
parameter start_op_bf = 16'h7000;
parameter end_op_bf = 16'h7040;
always_comb begin 
    if ((i_lsu_addr >= start_datamem) && (i_lsu_addr < end_datamem)) begin
        en_datamem = 1'b1;
        en_op_buf = 1'b0;
    end
    else if ((i_lsu_addr >= start_op_bf) && (i_lsu_addr < end_op_bf)) begin
            en_datamem = 1'b0;
            en_op_buf = 1'b1;
    end
    else begin
        en_datamem = 1'b0;
        en_op_buf = 1'b0;
    end
end
endmodule

module output_buffer(
     input logic [2:0] slt_sl,
     input logic [31:0] st_data_2_i  ,
	 input logic [15:0] addr_2_i ,
     input logic en_bf, 
	 input logic st_en_2_i           ,
	 input logic i_clk               ,
     input logic i_rst              ,
	 output logic [31:0] data_out_2_o,
	 output logic [31:0] io_lcd_o    ,
	 output logic [31:0] io_ledg_o   ,
	 output logic [31:0] io_ledr_o   ,
	 output logic [6:0] io_hex0_o   ,
	 output logic [6:0] io_hex1_o   ,
	 output logic [6:0] io_hex2_o   ,
	 output logic [6:0] io_hex3_o   ,
	 output logic [6:0] io_hex4_o   ,
	 output logic [6:0] io_hex5_o   ,
	 output logic [6:0] io_hex6_o   ,
	 output logic [6:0] io_hex7_o
	 );
     logic [31:0] data_bs,data_tmp;
     logic [31:0] MEMBF [0:4];
     always_comb begin
        case(addr_2_i[15:4])
        12'h700: data_bs = MEMBF[0]; // LED Red
        12'h701: data_bs = MEMBF[1]; // led green
        12'h702: begin
            if (addr_2_i[2] == 1'b0) data_bs = MEMBF[2];// HEX 0-3
            else data_bs = MEMBF[3]; //HEX 4-7
        end
        12'h703: data_bs = MEMBF[4]; //lcd
        default data_bs = 32'h0;
        endcase
     end
    data_trsf trsf_st (
        .slt_sl(slt_sl),
        .addr_sp(addr_2_i[1:0]),
        .wr_en(st_en_2_i),
        .data_bf(st_data_2_i),
        .data_bs(data_bs),
        .data_af(data_tmp)
    );

	 always_ff @(posedge i_clk, posedge i_clk) begin
        if (i_rst) begin
            MEMBF[0] <= '0;
            MEMBF[1] <= '0;
            MEMBF[2] <= '0;
            MEMBF[3] <= '0;
            MEMBF[4] <= '0;
        end
        else if (en_bf) begin
           if (st_en_2_i) begin
        case(addr_2_i[15:4])
        12'h700: MEMBF[0] <= data_tmp; // LED Red
        12'h701: MEMBF[1] <= data_tmp; // led green
        12'h702: begin
            if (addr_2_i[2] == 1'b0) MEMBF[2] <= data_tmp;// HEX 0-3
            else MEMBF[3] <= data_tmp; //HEX 4-7
        end
        12'h703: MEMBF[4]<= data_tmp;
        default begin
             MEMBF[0] <= MEMBF[0];
             MEMBF[1] <= MEMBF[1];
             MEMBF[2] <= MEMBF[2];
             MEMBF[3] <= MEMBF[3];
             MEMBF[4] <= MEMBF[4];
            end
        endcase
           end          
        end
     end
    assign data_out_2_o = (i_rst == 1'b1) ? 32'h0: data_tmp;
    assign  io_ledr_o =  MEMBF[0];
    assign  io_ledg_o =  MEMBF[1];
    assign  io_hex0_o =  MEMBF[2][6:0];  
    assign  io_hex1_o =  MEMBF[2][14:8];
    assign  io_hex2_o =  MEMBF[2][22:16]; 
    assign  io_hex3_o =  MEMBF[2][30:24];
    assign  io_hex4_o =  MEMBF[3][6:0];
    assign  io_hex5_o =  MEMBF[3][14:8]; 
    assign  io_hex6_o =  MEMBF[3][22:16];
    assign  io_hex7_o =  MEMBF[3][30:24];    
    assign	io_lcd_o  =  MEMBF[4];               
  endmodule
  module  data_trsf
(
    input logic [2:0] slt_sl,
    input logic [1:0] addr_sp,
    input logic wr_en,
    input logic [31:0] data_bf,
    input logic [31:0] data_bs,
    output logic [31:0] data_af
);
    // Định nghĩa giá trị SW, SB, SH, LW, LB, LH, LBU, LHU
    localparam SW = 3'b010, SB = 3'b000, SH = 3'b001;
    localparam LW = 3'b101, LB = 3'b011, LH = 3'b100;
    localparam LBU = 3'b110, LHU = 3'b111;
logic [31:0] memb_tmp,memh_tmp;
always_comb begin 
  case (addr_sp) 
  2'b00: begin
  memb_tmp = (data_bs & 32'h000000ff);
  memh_tmp = (data_bs & 32'h0000ffff);
  end
  2'b01: begin
  memb_tmp = (data_bs & 32'h0000ff00);
  memh_tmp = (data_bs & 32'h00ffff00);
  end
  2'b10: begin
  memb_tmp = (data_bs & 32'h00ff0000);
  memh_tmp = (data_bs & 32'hffff0000);
  end
  2'b11: begin
  memb_tmp = (data_bs & 32'hff000000);
  memh_tmp = (data_bs & 32'hffff0000);
  end
  endcase
end
always_comb begin
 if(wr_en) begin
   case (slt_sl) 
    SW: data_af = data_bf;
    SB: begin
    case(addr_sp)
    2'b00: data_af = (data_bf & 32'h000000ff) | (data_bs & 32'hffffff00);
    2'b01: data_af = ((data_bf & 32'h000000ff) << 8) | (data_bs & 32'hffff00ff);
    2'b10: data_af = ((data_bf & 32'h000000ff) << 16)| (data_bs & 32'hff00ffff);
    2'b11: data_af = ((data_bf & 32'h000000ff) << 24)| (data_bs & 32'h00ffffff);
    endcase
    end
    SH: begin
    case (addr_sp)
    2'b00: data_af = (data_bf & 32'h0000ffff) | (data_bs & 32'hffff0000); 
    2'b01: data_af = ((data_bf & 32'h0000ffff) << 8) | (data_bs & 32'hff0000ff); 
    2'b10: data_af = ((data_bf & 32'h0000ffff) << 16) | (data_bs & 32'hffff0000);
    2'b11: data_af = ((data_bf & 32'h0000ffff) << 16) | (data_bs & 32'hffff0000); 
    endcase 
    end
	 default: data_af = data_bf;
	 endcase
 end
 else begin
  case (slt_sl) 
   LW: data_af = data_bs;
   LB: data_af = memb_tmp;
   LH: data_af = memh_tmp;
   LBU: data_af = (memb_tmp[7] == 1)? (memb_tmp | 32'hffffff00): memb_tmp;
   LHU: data_af = (memh_tmp[13] == 1)? (memh_tmp | 32'hffff0000): memh_tmp;
	default: data_af = 32'h00000000;
  endcase
 end
end
endmodule

/* verilator lint_off UNUSED */
module mux_3_1_lsu(
  input logic [31:0] in_data_1_i,
  input logic [31:0] in_data_2_i,
  input logic [31:0] in_data_3_i,
  input logic [15:0] i_lsu_addr,
  output logic [31:0] o_ld_data
);
  logic [1:0] addr_sel ;
 
always_comb begin
  case (i_lsu_addr[15:4])
     12'h780:  addr_sel  =  2'b00; // SW
     12'h703:  addr_sel  =  2'b01; //LCD
     12'h702:  addr_sel  =  2'b01; //7-seg
     12'h701:  addr_sel  =  2'b01; //GRL
     12'h700:  addr_sel  =  2'b01; //RL
     default addr_sel = 2'b10; // MEM
  endcase
end
always_comb begin
  case(addr_sel)
  2'b00:o_ld_data = in_data_1_i; //input
  2'b01:o_ld_data = in_data_2_i; // op_bf
  2'b10:o_ld_data = in_data_3_i; // mem
  default: o_ld_data = 32'd0;    
  endcase
  end
endmodule

  

