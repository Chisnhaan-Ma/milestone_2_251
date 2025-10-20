module singlecycle
(   input   logic  i_clk              ,
	input   logic   i_rst             ,
    input   logic  [31:0]    i_io_sw  ,
 //   input   logic  [3:0]     i_io_btn,
    output  logic            o_insn_vld,
    output  logic  [31:0]    o_pc_debug,
	output  logic  [31:0]    o_io_lcd ,
	output  logic  [31:0]    o_io_ledg,
	output  logic  [31:0]    o_io_ledr,
	output  logic  [6:0]    o_io_hex0, 
	output  logic  [6:0]    o_io_hex1, 
	output  logic  [6:0]    o_io_hex2, 
	output  logic  [6:0]    o_io_hex3,
	output  logic  [6:0]    o_io_hex4,
	output  logic  [6:0]    o_io_hex5,
	output  logic  [6:0]    o_io_hex6,
	output  logic  [6:0]    o_io_hex7
/*   //sram_control
    output logic [17:0]   o_SRAM_ADDR,
    inout  wire [15:0]   o_SRAM_DQ  ,
    output logic          o_SRAM_CE_N,
    output logic          o_SRAM_WE_N,
    output logic          o_SRAM_LB_N,
    output logic          o_SRAM_UB_N,
    output  logic          o_SRAM_OE_N 
 */   
);
    
	logic  [31:0]  alu_data   ; 
	logic  [31:0]  wb_data    ; 
	logic  [31:0]  pc_four    ;   
	logic  [31:0]  nxt_pc     ; 
	logic  [31:0]  pc         ;  
	logic  [31:0]  instr      ;
	logic  [31:0]  rs1_data   ; 
	logic  [31:0]  rs2_data   ; 
	logic  [31:0]  operand_a  ;
	logic  [31:0]  operand_b  ; 
	logic  [31:0]  imm        ;
	logic  [31:0]  ld_data    ;
	logic  [1:0 ]  wb_sel     ;
	logic  [3:0]   alu_op     ;
	logic  [2:0 ]  imm_sel    ;
	logic          br_sel     ;
	logic          rd_wren    ;
	logic          op_a_sel   ;
	logic          op_b_sel   ;
	logic 	       mem_wren   ;
	logic          br_unsigned;
	logic          br_less    ;
	logic          br_equal	  ;
    logic [2:0]    slt_sl    ;
    logic insn_vld;
//    logic          stall     ;
always_ff @ (posedge i_clk) begin
    o_pc_debug <= (i_rst == 1'b0) ? 32'h0: pc;
    o_insn_vld <= (i_rst == 1'b0) ? 1'b0: insn_vld;
end
  mux_2_1 block_mux_pc (
      .data_1_i    (alu_data), 
      .data_0_i    (pc_four ), 
      .sel_i       (br_sel  ), 
      .data_out_o  (nxt_pc  )
      );
      

  pc block_pc (
      .i_clk       (i_clk   ), 
      .i_rst      (!i_rst  ), 
//      .i_stall     (stall),
      .i_next_pc   (nxt_pc  ), 
      .o_pc_current (pc)
      );

  pc_plus4 block_pcplus4 (
      .pc_current_i(pc      ),
      .i_rst(!i_rst), 
      .pc_plus4_o  (pc_four )
      );

  inst_memory block_imem (
      .i_addr      (pc[13:0]), 
      .i_rst      (!i_rst ),
      .o_rdata     (instr   )
      );	
 
  immgen block_immgen (
      .i_inst      (instr   ), 
      .i_immsel    (imm_sel ),
      .o_imm       (imm     )
      );	

  regfile block_regfile (
      .i_clk         (i_clk       ),
      .i_rst         (!i_rst)       ,
//      .i_stall        (stall)      ,
      .i_rs1_addr    (instr[19:15]), 
      .i_rs2_addr    (instr[24:20]), 
      .i_rd_addr     (instr[11:7] ), 
      .i_rd_data     (wb_data     ), 
      .i_rd_wren     (rd_wren     ), 
      .o_rs1_data    (rs1_data    ), 
      .o_rs2_data    (rs2_data    )
      );
	
  mux_2_1 block_mux_a (
      .data_1_i      (pc          ), 
      .data_0_i      (rs1_data    ), 
      .sel_i         (op_a_sel    ), 
      .data_out_o    (operand_a   )
      ); 	

  mux_2_1 block_mux_b (
      .data_1_i      (imm         ), 
      .data_0_i      (rs2_data    ), 
      .sel_i         (op_b_sel    ), 
      .data_out_o    (operand_b   )
      );

  alu block_alu (
      .i_operand_a   (operand_a   ), 
      .i_operand_b   (operand_b   ), 
      .i_alu_op      (alu_op      ),  
      .o_alu_data    (alu_data    )
      );

  mux_3_1 block_mux_wb (
      .data_0_i      (alu_data    ), 
      .data_1_i      (ld_data    ), 
      .data_2_i      (pc_four     ), 
      .sel_i         (wb_sel      ), 
      .data_out_o    (wb_data     )
      );	 
	
  brc block_brc(
      .i_rs1_data    (rs1_data    ), 
      .i_rs2_data    (rs2_data    ), 
      .i_br_un       (br_unsigned ), 
      .o_br_less     (br_less     ), 
      .o_br_equal    (br_equal    )
      );	
	
  ctrl_unit block_ctrl_unit(
      .i_instr       (instr       ), 
      .i_br_equal    (br_equal    ), 
      .i_br_less     (br_less     ),
      .o_insn_vld   (insn_vld),
      .o_slt_sl      (slt_sl)     ,
      .o_pc_sel      (br_sel      ), 
      .o_br_unsigned (br_unsigned ), 
      .o_rd_wren     (rd_wren     ), 
      .o_op_a_sel    (op_a_sel    ), 
      .o_op_b_sel    (op_b_sel    ), 
	  .o_alu_op      (alu_op      ), 
	  .o_mem_wren    (mem_wren    ), 
	  .o_wb_sel      (wb_sel      ),
	  .o_imm_sel     (imm_sel     )
	  );
	
  lsu block_lsu (
      .i_clk         (i_clk       ),
      .i_rst         (!i_rst      ), 
      .i_st_data     (rs2_data    ), 
      .i_lsu_addr        (alu_data    ), 
      .i_lsu_wren       (mem_wren    ),
      .slt_sl        (slt_sl)     ,
      .i_io_sw       (i_io_sw     ),
 //     .i_io_btn      (i_io_btn)   ,
/*      .o_stall        (stall),
      .o_SRAM_ADDR (o_SRAM_ADDR),
      .o_SRAM_DQ  (o_SRAM_DQ ),
      .o_SRAM_CE_N (o_SRAM_CE_N),
      .o_SRAM_WE_N (o_SRAM_WE_N),
      .o_SRAM_LB_N (o_SRAM_LB_N),
      .o_SRAM_UB_N (o_SRAM_UB_N),
      .o_SRAM_OE_N (o_SRAM_OE_N), 
      */ 
      .o_ld_data     (ld_data     ),
      .o_io_lcd      (o_io_lcd    ), 
      .o_io_ledg     (o_io_ledg   ), 
      .o_io_ledr     (o_io_ledr   ), 
      .o_io_hex0     (o_io_hex0   ), 
      .o_io_hex1     (o_io_hex1   ), 
      .o_io_hex2     (o_io_hex2   ), 
      .o_io_hex3     (o_io_hex3   ),
      .o_io_hex4     (o_io_hex4   ), 
      .o_io_hex5     (o_io_hex5   ), 
      .o_io_hex6     (o_io_hex6   ), 
      .o_io_hex7     (o_io_hex7   )
      );		
endmodule

