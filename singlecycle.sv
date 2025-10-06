`include "alu.sv"
`include "brc.sv"
`include "control_unit.sv"
`include "data_memory.sv"
`include "imm_gen.sv"
`include "inst_memory.sv"
`include "load_unit"
`include "mux2_1.sv"
`include "pc.sv"
`include "regfile.sv"
module singlecycle(
    input   logic         i_clk,
    input   logic         i_reset
    /*
    input   logic         i_io_sw,
    output  logic [31:0]    o_pc_debug,
    output  logic           o_insn_vld,
    output  logic [31:0]    o_io_ledr,
    output  logic [31:0]    o_io_ledg,
    output  logic  [6:0]    o_io_hex0, 
	output  logic  [6:0]    o_io_hex1, 
	output  logic  [6:0]    o_io_hex2, 
	output  logic  [6:0]    o_io_hex3,
	output  logic  [6:0]    o_io_hex4,
	output  logic  [6:0]    o_io_hex5,
	output  logic  [6:0]    o_io_hex6,
	output  logic  [6:0]    o_io_hex7
    */
);
	///- processor internal signal -///

    //For pc
	logic [31:0] pc_data_in;
	logic [31:0] pc_data_out;
    logic [31:0] pc_add4_out
    logic        pc_sel;

    //For inst memory
	logic [31:0] inst;

    //For regfile
    logic [31:0] rs1_data;
	logic [31:0] rs2_data;
    logic        rd_wren;
    logic [4:0] rd_addr;
    logic [4:0] rs1_addr;
    logic [4:0] rs2_addr; 

    //For brach control
    logic br_equal;
	logic br_less;
	logic br_un;
	logic Asel;
	logic Bsel;

    //For alu
    logic [31:0] operand_a;
	logic [31:0] operand_b;
    logic [3:0]  alu_op;
	logic [31:0] alu_data;

    //For immediate generator
    logic [31:0] imm_out;
    logic [2:0] imm_sel;

    //For data memory
    logic [31:0] rdata;
    logic [2:0] load_type;
	logic [31:0]load_result;

    //For write back
    logic wb_sel;
    logic [31:0]wb_data;

    assign rs1_addr[4:0] = inst[19:15];
    assign rs2_addr[4:0] = inst[24:20];
    assign rd_addr[4:0] = inst[11:7];
    
    pc pc_top (
        .i_clk(i_clk),
        .i_reset(i_reset),
        .i_pc_data_in(pc_data_in),
        .o_pc_data_out(pc_data_out));

    inst_memory inst_memory_top (
        .i_reset(i_reset),
        .i_addr(pc_data_out),
        .o_rdata(inst));

    
endmodule