`ifndef KIT_SINGLE_CYCLE
`define KIT_SINGLE_CYCLE

//`include "alu.sv"
//`include "brc.sv"
//`include "control_unit.sv"
//`include "imm_gen.sv"
//`include "inst_memory.sv"
//`include "lsu.sv"
//`include "mux3_1.sv"
//`include "mux2_1.sv"
//`include "pc.sv"
//`include "regfile.sv"
module Test_pc_debug(
    input   logic         i_clk,
    input   logic         i_reset,
    output  logic [31:0]  o_pc_debug,
    output  logic         o_inst_vld,

    input   logic  [31:0]    i_io_sw,

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
);
	///- processor internal signal -///

    //For pc
	logic [31:0] pc_data_in;
	logic [31:0] pc_data_out;
    logic [31:0] pc_add4_out;
    logic        pc_sel;

    //For inst memory
	logic [31:0] inst;

    //For regfile
    logic [31:0] rs1_data;
	logic [31:0] rs2_data;
    logic [31:0] rd_data;
    logic        rd_wren;

    
    logic [4:0] rd_addr;
    logic [4:0] rs1_addr;
    logic [4:0] rs2_addr; 

    //For brach control
    logic br_equal;
	logic br_less;
	logic br_un;

    //For operand selection
	logic asel;
	logic bsel;

    //For alu
    logic [31:0] operand_a;
	logic [31:0] operand_b;
    logic [3:0]  alu_op;
	logic [31:0] alu_data;

    //For immediate generator
    logic [31:0] imm_out;
    logic [2:0]  imm_sel;

    //For lsu
    logic [31:0] rdata;
    //logic [2:0]  load_type;
	//logic [31:0] load_result;
    logic [31:0] ld_data;
    logic [2:0]  slt_sl;
    logic        wren;

    //For write back
    logic [1:0] wb_sel;
    //logic [31:0]wb_data;

    logic inst_vld;

    assign rs1_addr[4:0] = inst[19:15];
    assign rs2_addr[4:0] = inst[24:20];
    assign rd_addr[4:0] =  inst[11:7];
    

    pc pc_top (
        .i_clk(i_clk),
        .i_reset(i_reset),
        .i_pc_data_in(pc_data_in),
        .o_pc_data_out(pc_data_out)
        );

    add_sub_32_bit pc_add4 (
        .A(pc_data_out),
        .B(32'd4),
        .Sel(1'b0),
        .Result(pc_add4_out));

    inst_memory inst_memory_top (
        .i_addr(pc_data_out),
        .o_rdata(inst)
        );
    
    imm_gen imm_gen_top(
        .i_imm_sel(imm_sel),
        .i_inst(inst),
        .o_imm_out(imm_out));

    alu alu_top(
        .i_operand_a(operand_a),
        .i_operand_b(operand_b),
        .i_alu_op(alu_op),
        .o_alu_data(alu_data));

    lsu lsu_top(
        .i_clk(i_clk),
        .i_reset(i_reset),

        .i_lsu_wren(wren),
        .i_lsu_addr(alu_data),
        .i_st_data(rs2_data),

        .slt_sl(slt_sl),

        .i_io_sw (i_io_sw),

        .o_ld_data     (ld_data),
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

    mux_3_1 mux3_1_top_writeback(
        .sel_i(wb_sel),
        .data_0_i(ld_data),
        .data_1_i(alu_data),
        .data_2_i(pc_add4_out),
        .data_out_o(rd_data));

    brc brc_top(
        .i_rs1_data(rs1_data),
        .i_rs2_data(rs2_data),
        .i_br_un(br_un),
        
        .o_br_equal(br_equal),
        .o_br_less(br_less));

    control_unit control_unit_top(
        .i_inst(inst),
        .i_br_equal(br_equal),
        .i_br_less(br_less),

        .o_inst_vld_ctrl(insn_vld),
        .o_pc_sel(pc_sel),
        .o_imm_sel(imm_sel),
        .o_rd_wren(rd_wren),
        .o_br_un(br_un),
        .o_asel(asel),
        .o_bsel(bsel),
        .o_alu_op(alu_op),
        .o_wren(wren),
        .o_slt_sl(slt_sl),
        .o_wb_sel(wb_sel)
    );
    
    regfile regfile_top(
        .i_clk(i_clk),
        .i_reset(i_reset),

        .i_rs1_addr(rs1_addr),
        .i_rs2_addr(rs2_addr),

        .i_rd_addr(rd_addr),
        .i_rd_data(rd_data),
        .i_rd_wren(rd_wren),

        .o_rs1_data(rs1_data),
        .o_rs2_data(rs2_data)
    );
    mux_2_1 mux2_1_asel_top(
        .data_0_i(rs1_data),
        .data_1_i(pc_data_out),
        .sel_i(asel),
        .data_out_o(operand_a));
    
    mux_2_1 mux2_1_bsel_top(
        .data_0_i(rs2_data),
        .data_1_i(imm_out),
        .sel_i(bsel),
        .data_out_o(operand_b));

    mux_2_1 mux2_1_pc_sel(
        .data_0_i(pc_add4_out),
        .data_1_i(alu_data),
        .sel_i(pc_sel),
        .data_out_o(pc_data_in));

    always_ff @( posedge i_clk or posedge i_reset ) begin
        if(i_reset) begin    
            o_inst_vld <= insn_vld;
            o_pc_debug <= 32'b0;
        end
        else begin
            o_inst_vld <= insn_vld;
            o_pc_debug <= pc_data_out;
        end
    end
endmodule
`endif
`ifndef INST_MEMORY
`define INST_MEMORY
module inst_memory (
  output logic [31:0] o_rdata,
  input  logic [31:0] i_addr
);

  logic [31:0] imem [0:2048];
  initial begin
    $readmemh("D:/HCMUT/Year_2025_2026/251/Conmputer_Organization/milestone_2/00_src/lcd.dump",imem);
  end
  always @(*) begin
      o_rdata = imem[i_addr[31:2]];  
  end
endmodule
`endif
//`include "full_adder.sv"
`ifndef ADD_SUB_32_BIT
`define ADD_SUB_32_BIT
module add_sub_32_bit (
    input  logic [31:0] A, B,  // Input A, B
    input  logic Sel,          // 0 = ADD, 1 = SUB
    output logic [31:0] Result,// Kết quả phép cộng 
    output logic Cout);          // Carry-out

    logic [31:0] B_mod;        // 
    logic Cin;                 // Carry-in
    logic [31:0] carry;  // Carry signals
    assign B_mod = (Sel) ? ~B : B;  // Bù 2 của B

    full_adder FA0(
	 A[0],
	 B_mod[0],
	 Sel,
	 Result[0],
	 carry[0]);

    // Generate 31 more full adders
    genvar i;
    generate
        for (i = 1; i < 32; i = i + 1) begin :adder_32
            full_adder FA (A[i], B_mod[i], carry[i-1], Result[i], carry[i]);
        end
    endgenerate

    assign Cout = carry[31];

endmodule
`endif

`ifndef ALU
`define ALU

//`include "add_sub_32_bit.sv"
//`include "shift_right_logical.sv"
//`include "shift_left_logical.sv"
//`include "shift_right_arithmetic.sv"
//`include "slt_sltu.sv"

module alu(
	//  input
	input  logic  [31:0]  i_operand_a,
	input  logic  [31:0]  i_operand_b,
    input  logic   [3:0]  i_alu_op,
	//  output
	output  logic  [31:0]  o_alu_data
);
    logic [31:0] add_sub_out, sll_out, srl_out, sra_out;
    logic [31:0] slt_out, sltu_out;

    // Instance các module cần dùng
	 // ADD, SUB
    add_sub_32_bit add_sub_alu( 
	 .A(i_operand_a),
	 .B(i_operand_b),
	 .Sel(i_alu_op[3]),
	 .Result(add_sub_out)); 
	 
	 // SLL
    shift_left_logical sll_alu(
	 .data_in(i_operand_a),
	 .shift_amt(i_operand_b[4:0]),
	 .data_out(sll_out)); // SLL

	 // SRL
    shift_right_logical srl_alu(
	 .data_in(i_operand_a), 
	 .shift_amt(i_operand_b[4:0]),
	 .data_out(srl_out)); 
	 
	 // SRA
    shift_right_arithmetic sra_alu(
	 .data_in(i_operand_a), 
	 .shift_amt(i_operand_b[4:0]),
	 .data_out(sra_out)); 
	 
	 // SLT
    slt_sltu slt_alu(
	 .A(i_operand_a), 
	 .B(i_operand_b), 
	 .Sel(1'b0),
	 .Result(slt_out));  
	 
	 // SLT
    slt_sltu sltu_alu(
	 .A(i_operand_a),
	 .B(i_operand_b), 
	 .Sel(1'b1), 
	 .Result(sltu_out));  

    always @(*) begin
        case (i_alu_op)
            4'b0000: o_alu_data = add_sub_out;  // ADD
            4'b1000: o_alu_data = add_sub_out;  // SUB
            4'b0001: o_alu_data = sll_out;      // SLL
            4'b0010: o_alu_data = slt_out;  // SLT (1 bit)
            4'b0011: o_alu_data = sltu_out; // SLTU (1 bit)
            4'b0100: o_alu_data = i_operand_a ^ i_operand_b;  // XOR
            4'b0101: o_alu_data = srl_out; // SRL
            4'b1101: o_alu_data = sra_out; // SRA
            4'b0110: o_alu_data = i_operand_a | i_operand_b;  // OR
            4'b0111: o_alu_data = i_operand_a & i_operand_b;  // AND
			4'b1111: o_alu_data = i_operand_b; //Cho lệnh LUI
            default: o_alu_data = 32'bz;  
        endcase
    end
endmodule
`endif
`ifndef BRC
`define BRC
//`include "add_sub_32_bit.sv"
module  brc (
	//  input
	input  logic  [31:0]  i_rs1_data,
	input  logic  [31:0]  i_rs2_data,
	input  logic          i_br_un,
	//  output
	output  logic  o_br_less,
	output  logic  o_br_equal
);
 logic [31:0] Diff;  // Kết quả phép trừ rs1 - rs2
    logic Cout;         // Carry-out từ add_sub_32_bit

    add_sub_32_bit subtractor (
        .A(i_rs1_data),
        .B(i_rs2_data),
        .Sel(1'b1),   // SUB
        .Result(Diff),
        .Cout(Cout)
    );

    // o_br_equal
    always @(*) begin
        if (Diff == 32'b0)
            o_br_equal = 1'b1;
        else
            o_br_equal = 1'b0;
    end

    // o_br_less
    always @(*) begin
        if (i_br_un)  // Unsigned
            o_br_less = ~Cout;
        else       // Signed
            o_br_less = Diff[31];
    end

endmodule
`endif 
`ifndef CONTROL_UNIT
`define CONTROL_UNIT
///Control Unit//////
module control_unit(
	input logic [31:0]i_inst,
	input logic i_br_equal,
	input logic i_br_less,

	output logic o_inst_vld_ctrl,
	output logic o_pc_sel,
	output logic [2:0]o_imm_sel,
	output logic o_rd_wren,
	output logic o_br_un,
	output logic o_bsel,
	output logic o_asel,
	output logic [3:0]o_alu_op,
	output logic o_wren,
	output logic [2:0] o_slt_sl,
	output logic [2:0]o_load_type,
	output logic [1:0]o_wb_sel);
	
	logic [6:0] opcode;
    logic [2:0] funct3;
    logic [6:0] funct7;

   assign opcode = i_inst[6:0];
   assign funct3 = i_inst[14:12];
	
    always @(*) begin
    	// Giá trị mặc định
        o_pc_sel   = 1'b0;
        o_imm_sel = 3'b000;
		o_rd_wren  = 1'b0;
        o_br_un    = 1'b0;
        o_bsel    = 1'b0;
        o_asel    = 1'b0;
        o_alu_op = 4'b0000;
        o_wren   = 1'b0;
        o_wb_sel   = 2'b00;
		o_load_type = 3'b0;
		o_slt_sl = 3'b0;
		case (opcode)
			7'b0110011: begin  // R-type 
				o_inst_vld_ctrl = 1'b1; 
				o_rd_wren  = 1'b1;//cho phép ghi reg file
				o_wb_sel   = 2'b1;// chọn write back từ ngõ ra ALU
				o_asel    = 1'b0;// chọn A từ rs1
				o_bsel    = 1'b0;//chọn B từ rs2s
				o_load_type = 3'b0; //không load
				o_imm_sel = 3'b000;// Imm_Gen theo I type
				o_br_un    = 1'b0;
				o_wren   = 1'b0;
				o_slt_sl = 3'b0;
				case(funct3)
					3'b000: o_alu_op = (i_inst[30]) ? 4'b1000 : 4'b0000; // SUB nếu i_inst[30] = 1, ADD nếu i_inst[30] = 0
					3'b001: o_alu_op = 4'b0001; // SLL
					3'b010: o_alu_op = 4'b0010; // SLT
					3'b011: o_alu_op = 4'b0011; // SLTU
					3'b100: o_alu_op = 4'b0100; // XOR
					3'b101: o_alu_op = (i_inst[30]) ? 4'b1101 : 4'b0101; // SRA nếu i_inst[30] = 1, SRL nếu i_inst[30] = 0
					3'b110: o_alu_op = 4'b0110; // OR
					3'b111: o_alu_op = 4'b0111; // AND
					default: begin
						o_inst_vld_ctrl =  1'b0;
						o_alu_op = 4'b0000; // Mặc định là ADD
					end
				endcase
			end
			

			7'b0010011: begin  // I-type
				o_slt_sl = 3'b000; 
				o_inst_vld_ctrl = 1'b1; 
				o_rd_wren  = 1'b1;//cho phép ghi reg file
				o_wb_sel   = 2'b1;// chọn write back từ ngõ ra ALU
				o_asel    = 1'b0;// chọn A từ rs1
				o_bsel    = 1'b1;//chọn B từ rs2
				o_imm_sel = 3'b000;// Imm_Gen theo I type
				o_wren   = 1'b0;	// Cho phép đọc đọc DMEM
				o_load_type = 3'b0; // không load
				o_br_un    = 1'b0;
				case(funct3)
					3'b000: o_alu_op = 4'b0000; // ADDI
					3'b001: o_alu_op = 4'b0001; // SLLI
					3'b010: o_alu_op = 4'b0010; // SLTI
					3'b011: o_alu_op = 4'b0011; // SLTUI
					3'b100: o_alu_op = 4'b0100; // XORI
					3'b101: o_alu_op = (i_inst[30]) ? 4'b1101 : 4'b0101; // SRAI nếu i_inst[30] = 1, SRLI nếu i_inst[30] = 0
					3'b110: o_alu_op = 4'b0110; // ORI
					3'b111: o_alu_op = 4'b0111; // ANDI
					default: begin
						o_inst_vld_ctrl =  1'b0;
						o_alu_op = 4'b0000; // Mặc định là ADD
					end
				endcase																							
			end
				
			7'b0000011: begin  // Load
				//o_slt_sl = 3'b0;
				o_inst_vld_ctrl = 1'b1; 
				o_rd_wren  = 1'b1; // Cho phép ghi lại vào regfile
				o_wb_sel   = 2'b0; // Lấy dữ liệu từ DMEM
				o_asel    = 1'b0; // Chọn Rs1 + Imm_Gen
				o_bsel    = 1'b1; // Chọn Imm_Gen
				o_imm_sel = 3'b000;	// Imm_Gen theo I type
				o_alu_op = 4'b0000;	// Thực hiện phép cộng
				o_wren   = 1'b0;	// Cho phép đọc đọc DMEM
				o_br_un    = 1'b0;
				case(funct3)
					3'b000: o_slt_sl = 3'b011; // lb
					3'b001: o_slt_sl = 3'b100; // lh
					3'b010: o_slt_sl = 3'b101; // lw
					3'b100: o_slt_sl = 3'b110; // lbu
					3'b101: o_slt_sl = 3'b111; // lhu	
					default: begin
						o_inst_vld_ctrl =  1'b0;
						o_load_type = 3'bz;
						o_slt_sl = 3'b101; 
					end
				endcase
			end
			
			7'b0100011: begin //	S-type
				o_inst_vld_ctrl = 1'b1; 
				o_imm_sel = 3'b001;	// Imm_Gen theo S type
				o_rd_wren = 1'b0; //không cho ghi lại vào reg file
				o_asel = 1'b0; //chọn A là rs1 
				o_bsel = 1'b1; //chọn B là Imm_Gen theo S type
				o_wren = 1'b1; //cho phép đọc và ghi DMEM
				o_alu_op = 4'b0000;	// Thực hiện phép cộng
				o_wb_sel = 2'b11; //write back là tùy định vì không ghi ngược lại vào regfile
				o_load_type = 3'b000; 
				o_br_un    = 1'b0;
				case (funct3)
					3'b000: o_slt_sl = 3'b000; // sb
					3'b001: o_slt_sl = 3'b001; // sh
					3'b010: o_slt_sl = 3'b010; // sw
					default begin
						o_slt_sl = 3'b010; // sw
						o_inst_vld_ctrl =  1'b0;
            		end
          		endcase
			end
			
			7'b1100011: begin	// B-type
				o_inst_vld_ctrl = 1'b1; 
				o_imm_sel = 3'b010;	//Imm_Gen theo B type
				o_rd_wren = 1'b0;	//Không ghi lại vào reg file
				o_asel = 1'b1;	// A chọn PC hiện hành
				o_bsel = 1'b1;	// B chọn imm_Gen theo B type
				o_alu_op = 4'b0000;	//ALU thực hiện phép cộng
				o_wren = 1'b0;	// Cho phép đọc DMEM
				o_wb_sel = 2'b11; // write back là tùy định vì không ghi ngược lại vào regfile
				o_load_type = 3'b000; 
				o_slt_sl = 3'b0;
				case(funct3)
					3'b000: begin o_br_un = 1'b1; o_pc_sel = i_br_equal;	end//BEQ
					3'b001: begin o_br_un = 1'b1; o_pc_sel = ~i_br_equal;	end//BNE
					3'b100: begin o_br_un = 1'b0;	o_pc_sel = i_br_less;	end//BLT
					3'b101: begin o_br_un = 1'b0; o_pc_sel = ~i_br_less; end//BGE
					3'b110: begin o_br_un = 1'b1;	o_pc_sel = i_br_less;	end//BLTU
					3'b111: begin o_br_un = 1'b1;	o_pc_sel = ~i_br_less;	end//BGEU
					default: begin
						o_pc_sel = 1'b0; // Không nhảy nếu opcode không hợp lệ
						o_inst_vld_ctrl =  1'b0;
					end
				endcase
			end
			
			7'b1101111: begin // J-type JAL
				o_inst_vld_ctrl = 1'b1; 
				o_pc_sel = 1'b1;	//o_pc_sel chọn luôn nhảy
				o_imm_sel = 3'b011; // Imm_Gen theo J Type
				o_rd_wren = 1'b1;	// cho phép ghi ngược vào regfile
				o_bsel = 1'b1;	// B chọn Imm_Gen theo J type
				o_asel  =1'b1;	// A chọn PC hiện hành
				o_alu_op = 4'b0000;	// ALU thực hiện phép cộng
				o_wren = 1'b0;	// Cho phép đọc DMEM
				o_wb_sel = 2'b10;	// Write back chọn PC+4 để return
				o_load_type = 3'b000; 
				o_br_un    = 1'b0;
				o_slt_sl = 3'b000;
			end
			
			7'b1100111: begin // I-type JALR
				o_inst_vld_ctrl = 1'b1; 
				o_pc_sel = 1'b1;	//o_pc_sel chọn luôn nhảy
				o_imm_sel = 3'b000; // Imm_Gen theo I Type
				o_rd_wren = 1'b1;	// cho phép ghi ngược vào regfile
				o_bsel = 1'b1;	// B chọn Imm_Gen theo I type
				o_asel = 1'b0;	// A chọn rs1
				o_alu_op = 4'b0000;	// ALU thực hiện phép cộng
				o_wren = 1'b0;	// Cho phép đọc DMEM
				o_wb_sel = 2'b10; // Write back chọn PC+4 để return
				o_load_type = 3'b000; 
				o_br_un    = 1'b0;
				o_slt_sl = 3'b000;
			end
			7'b0110111: begin // U-type LUI
				o_inst_vld_ctrl = 1'b1; 
				o_pc_sel = 1'b0;	// o_pc_sel chọn không nhảy
				o_imm_sel = 3'b100; //Imm_Gen theo U type LUI
				o_rd_wren = 1'b1; //cho phép ghi vào reg file
				o_bsel = 1'b1;	// B chọn Imm_Gen theo U type LUI
				o_asel = 1'b0;	// A tùy định
				o_alu_op = 4'b1111; //ALU dẫn B ra ALU_out
				o_wren = 1'b0; // Cho phép đọc DMEM
				o_wb_sel = 2'b01; // chọn write back từ ALU_out
				o_load_type = 3'b000; 
				o_br_un    = 1'b0;
				o_slt_sl = 3'b000;
			end
			7'b0010111: begin // U-type AUIPC
				o_inst_vld_ctrl = 1'b1; 
				o_pc_sel = 1'b0;	// o_pc_sel chọn không nhảy
				o_imm_sel = 3'b101; //Imm_Gen theo U type AUIPC
				o_rd_wren = 1'b1; //cho phép ghi vào reg file
				o_bsel = 1'b1;	// B chọn Imm_Gen theo U type LUI
				o_asel = 1'b1;	// A tùy lấy PC hiện hành
				o_alu_op = 4'b0000; //ALU thực hiện phép cộng
				o_wren = 1'b0; // Cho phép đọc DMEM
				o_wb_sel = 2'b01; // chọn write back từ ALU_out
				o_load_type = 3'b000; 
				o_br_un    = 1'b0;
				o_slt_sl = 3'b000;
			end
			default begin
				o_inst_vld_ctrl = 1'b0;
				o_pc_sel   = 1'b0;
				o_imm_sel = 3'b000;
				o_rd_wren  = 1'b0;
				o_br_un    = 1'b0;
				o_bsel    = 1'b0;
				o_asel    = 1'b0;
				o_alu_op = 4'b0000;
				o_wren   = 1'b0;
				o_wb_sel   = 2'b00;
				o_load_type = 3'b000;
				o_slt_sl = 3'b000; 
			end
		endcase
	end
endmodule
`endif
`ifndef FULL_ADDER
`define FULL_ADDER
module full_adder (
    input  logic A,
	input logic B,
	input logic Cin,
    output logic Sum,
	output logic Cout);
	 
    assign Sum  = A ^ B ^ Cin;
    assign Cout = (A & B) | (Cin & (A ^ B));
endmodule
`endif
`ifndef IMM_GEN
`define IMM_GEN
/////////Immediate generator///////////////
module imm_gen(	
	input logic [2:0] i_imm_sel, // Chọn kiểu generate
	input logic [31:0] i_inst,	// i_instruction
	output logic [31:0] o_imm_out); // Kết quả
	// I type = 000	7'b0010011
	// S type = 001	7'b0100011
	// B type = 010	7'b1100011
	// J type = 011	7'b1101111	JAL
	// U type = 100	7'b0110111 	LUI
	// U type = 101	7'b0010111	AUIPC
always @(*) begin
	case(i_imm_sel) 
		3'b000: begin //I typte
          o_imm_out = {{20{i_inst[31]}}, i_inst[31:20]};
		end
		3'b001: begin // S type
          o_imm_out = {{20{i_inst[31]}}, i_inst[31:25], i_inst[11:7]};
		end
		3'b010: begin //B type
          o_imm_out = {{20{i_inst[31]}}, i_inst[7],i_inst[30:25], i_inst[11:8],1'b0};
		end
		3'b011: begin // J type JAL
          o_imm_out = {{11{i_inst[31]}}, i_inst[31], i_inst[19:12], i_inst[20], i_inst[30:21], 1'b0};
		end
		3'b100: begin // U type LUI
			o_imm_out = {i_inst[31:12], 12'b0};
		end
		3'b101: begin // U type AUIPC
			o_imm_out = {i_inst[31:12], 12'b0};
		end
		default: o_imm_out = 32'bz;
	endcase
end
endmodule
`endif
`ifndef LSU
`define LSU
/*------------------------------------------------------------*/


/*------------------------------------------------------------*/

module lsu (
    input logic i_clk, i_reset, i_lsu_wren,

    input logic [31:0] i_lsu_addr,
    input logic [31:0] i_st_data,  
    output logic [31:0] o_ld_data, 
    input logic [2:0] slt_sl,

    output logic [31:0] o_io_ledr, //
    output logic [31:0] o_io_ledg,  
    output logic [6:0] o_io_hex0, 
    output logic [6:0] o_io_hex1, 
    output logic [6:0] o_io_hex2,   
    output logic [6:0] o_io_hex3,  // output buffer
    output logic [6:0] o_io_hex4, 
    output logic [6:0] o_io_hex5, 
    output logic [6:0] o_io_hex6,   
    output logic [6:0] o_io_hex7, 
    output logic [31:0] o_io_lcd, //
    
    input logic [31:0] i_io_sw // switch -> input buffer
  );

  localparam RESERVED_1  = 32'h1001_1000;
  localparam SWITCH      = 32'h1001_0000;
  localparam RESERVED_2  = 32'h1000_5000;
  localparam LCD_CONTROL = 32'h1000_4FFF;
  localparam SEG7LEDS_74 = 32'h1000_3FFF;
  localparam SEG7LEDS_30 = 32'h1000_2FFF;
  localparam GREEN_LEDS  = 32'h1000_1FFF;
  localparam RED_LEDS    = 32'h1000_0FFF;
  localparam RESERVED_3  = 32'h0FFF_FFFF;
  localparam MEM_2KB     = 32'h0000_07FF;
  
  /*---------------------------------*/

  logic [31:0] data_out_1, data_out_2, data_out_3; // IO_switchs - peripheral registers - Data (SRAM)
  logic en_datamem;
  logic en_op_buf;
  //logic ACK;
/*-------- Input buffer --------*/
  logic [31:0] INPUT;
  always_ff @(posedge i_clk) begin // ghi đồng bộ
    INPUT <= i_io_sw; 
  end
  assign data_out_1 = i_reset ? 32'd0: INPUT; // đọc bất đồng bộ

/*-------- Data mem --------*/
  datamem mem (
    .i_clk(i_clk),
    .i_reset(i_reset),
    .i_wren(i_lsu_wren),
    .slt_sl(slt_sl),
    .i_enb(en_datamem),
    .i_addr(i_lsu_addr[10:0]), // 21 bits cao giong nhau -> xet 11 bit thap
    .i_data(i_st_data),
    .o_data(data_out_3)
  );

/*-------- DEMUX --------*/
  demux_sel_mem demux_1 (
    .i_lsu_addr(i_lsu_addr[31:0]),
    .en_datamem(en_datamem),
    .en_op_buf(en_op_buf)  
  );

/*-------- Output buffer --------*/
  output_buffer  outputperiph (
    .slt_sl (slt_sl),
    .st_data_2_i   (i_st_data), 
    .addr_2_i      (i_lsu_addr[31:0]),
    .en_bf         (en_op_buf), 
    .st_en_2_i     (i_lsu_wren),
    .i_clk         (i_clk), 
    .i_reset       (i_reset),
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
/*-------- MUX --------*/
  mux_3_1_lsu mux31  (
    .in_data_3_i(data_out_3), 
    .in_data_2_i(data_out_2), 
    .in_data_1_i(data_out_1), 
    .i_lsu_addr(i_lsu_addr[31:0]),
    .o_ld_data(o_ld_data)
    );	


endmodule

module datamem (
  input logic i_clk,
  input logic i_reset,
  input logic i_wren,
  input logic i_enb,
  input logic [2:0] slt_sl,
  input logic [10:0] i_addr,
  input logic [31:0] i_data,
  output logic [31:0] o_data
);
  logic [31:0] data_mem [2047:0];
  logic [31:0] data_bs,data_tmp;

  data_trsf trsf_st (
    .slt_sl(slt_sl),
    .addr_sp(i_addr[1:0]),
    .wr_en(i_wren),
    .data_bf(i_data),
    .data_bs(data_bs),
    .data_af(data_tmp)
  );

  assign o_data = (i_reset == 1'b1) ? 32'h0: data_tmp;

  always @(*) begin
    data_bs = data_mem [i_addr[10:2]];
  end

  always_ff @(posedge i_clk or posedge i_reset) begin
    if (i_reset) begin
      for (int i=0;i<2048; i++) begin
        data_mem[i] <= 0;
      end
    end
    else if (i_enb) begin
      if (i_wren) begin
        data_mem[i_addr[10:2]] <= data_tmp;
      end
      else 
        data_mem[i_addr[10:2]] <= data_mem[i_addr[10:2]];
    end
  end
endmodule

module demux_sel_mem (
    input logic [31:0] i_lsu_addr,
    output logic en_datamem,
    output logic en_op_buf  
  );
  parameter START_DATAMEM = 32'h0000_0000;
  parameter END_DATAMEM = 32'h0000_0800;
  parameter START_OP_BF = 32'h1000_0000;
  parameter END_OP_BF = 32'h1000_5000;

  always @(*) begin
    en_datamem = 1'b0;
    en_op_buf = 1'b0;
    if ((i_lsu_addr >= START_DATAMEM) && (i_lsu_addr < END_DATAMEM)) begin
      en_datamem = 1'b1;
      en_op_buf = 1'b0;
    end
    else if ((i_lsu_addr >= START_OP_BF) && (i_lsu_addr < END_OP_BF)) begin
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
  input logic [2:0] slt_sl, // chọn store/load kiểu gì (W H B HU BU)
    /* 
       SW = 3'b010, SB = 3'b000, SH = 3'b001;
       LW = 3'b101, LB = 3'b011, LH = 3'b100;
       LBU = 3'b110, LHU = 3'b111;
    */
  input logic [31:0] st_data_2_i, // data
	input logic [31:0] addr_2_i, // address
  input logic en_bf, // en
	input logic st_en_2_i, //write enable
	input logic i_clk,
  input logic i_reset,

	output logic [31:0] data_out_2_o, // data out -> mux
	output logic [31:0] io_lcd_o,
	output logic [31:0] io_ledg_o,
	output logic [31:0] io_ledr_o,
	output logic [6:0] io_hex0_o,
	output logic [6:0] io_hex1_o,
	output logic [6:0] io_hex2_o,
	output logic [6:0] io_hex3_o,
	output logic [6:0] io_hex4_o,
	output logic [6:0] io_hex5_o,
	output logic [6:0] io_hex6_o,
	output logic [6:0] io_hex7_o
	 );

  logic [31:0] data_bs,data_tmp;
  logic [31:0] MEMBF [0:4];

  assign data_out_2_o = (i_reset == 1'b1) ? 32'h0: data_tmp;
  assign io_ledr_o =  MEMBF[0];
  assign io_ledg_o =  MEMBF[1];
  assign io_hex0_o =  MEMBF[2][6:0];  
  assign io_hex1_o =  MEMBF[2][14:8];
  assign io_hex2_o =  MEMBF[2][22:16]; 
  assign io_hex3_o =  MEMBF[2][30:24];
  assign io_hex4_o =  MEMBF[3][6:0];
  assign io_hex5_o =  MEMBF[3][14:8]; 
  assign io_hex6_o =  MEMBF[3][22:16];
  assign io_hex7_o =  MEMBF[3][30:24];    
  assign io_lcd_o  =  MEMBF[4];  

  data_trsf trsf_st (
    .slt_sl(slt_sl),
    .addr_sp(addr_2_i[1:0]),
    .wr_en(st_en_2_i),
    .data_bf(st_data_2_i),
    .data_bs(data_bs),
    .data_af(data_tmp)
  );

  always @(*) begin
    data_bs = 32'h0;
    case(addr_2_i[15:12])
      4'h0: data_bs = MEMBF[0]; // LED Red
      4'h1: data_bs = MEMBF[1]; // led green
      4'h2: data_bs = MEMBF[2];// HEX 0-3
      4'h3: data_bs = MEMBF[3];// HEX 4-7
      4'h4: data_bs = MEMBF[4]; //lcd
      default data_bs = 32'h0;
    endcase
  end

  always_ff @(posedge i_clk or posedge i_reset) begin
    if (i_reset) begin
      MEMBF[0] <= 32'h0;
      MEMBF[1] <= 32'h0;
      MEMBF[2] <= 32'h0;
      MEMBF[3] <= 32'h0;
      MEMBF[4] <= 32'h0;
    end
    else if (en_bf) begin
      if (st_en_2_i) begin
        case(addr_2_i[15:12])
          4'h0: MEMBF[0] <= data_tmp; // LED Red
          4'h1: MEMBF[1] <= data_tmp; // led green
          4'h2: MEMBF[2] <= data_tmp; // HEX 0-3
          4'h3: MEMBF[3] <= data_tmp; //HEX 4-7
          4'h4: MEMBF[4] <= data_tmp; //lcd
          default: begin
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

  always @(*) begin
    memb_tmp = 32'h0;
    memh_tmp = 32'h0;
    case (addr_sp) 
      2'b00: begin
      memb_tmp = (data_bs & 32'h000000ff);
      memh_tmp = (data_bs & 32'h0000ffff);
      end
      2'b01: begin
      memb_tmp = (data_bs & 32'h0000ff00);
      memh_tmp = (data_bs & 32'h0000ffff);
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

  always @(*) begin
    data_af = 32'h0;
    if(wr_en) begin
      case (slt_sl) 
        SW: data_af = data_bf;
        SB: begin
          case(addr_sp)
            2'b00: data_af = {data_bs[31:8],data_bf[7:0]};               //(data_bf & 32'h000000ff) | (data_bs & 32'hffffff00);
            2'b01: data_af = {data_bs[31:16],data_bf[7:0],data_bs[7:0]}; //((data_bf & 32'h000000ff) << 8) | (data_bs & 32'hffff00ff);
            2'b10: data_af = {data_bs[31:24],data_bf[7:0],data_bs[15:0]};//((data_bf & 32'h000000ff) << 16)| (data_bs & 32'hff00ffff);
            2'b11: data_af = {data_bf[7:0],data_bs[23:0]};               //((data_bf & 32'h000000ff) << 24)| (data_bs & 32'h00ffffff);
          endcase
        end
        SH: begin
          case (addr_sp)
            2'b00: data_af = {data_bs[31:16],data_bf[15:0]};  //(data_bf & 32'h0000ffff) | (data_bs & 32'hffff0000); 
            2'b01: data_af = {data_bs[31:16],data_bf[15:0]};  //((data_bf & 32'h0000ffff)) | (data_bs & 32'hffff0000); 
            2'b10: data_af = {data_bf[15:0], data_bs[15:0]};  //((data_bf & 32'h0000ffff) << 16) | (data_bs & 32'h0000ffff);
            2'b11: data_af = {data_bf[15:0], data_bs[15:0]};  //((data_bf & 32'h0000ffff) << 16)  | (data_bs & 32'h0000ffff); 
          endcase 
        end
        default: data_af = data_bf;
      endcase
    end
    else begin
      case (slt_sl) 
        LW: data_af = data_bs;
        LBU: begin
          case (addr_sp) 
            2'b00: data_af = memb_tmp;
            2'b01: data_af = {8'b0, memb_tmp[31:8]};  //(memb_tmp >>8);
            2'b10: data_af = {16'b0,memb_tmp[31:16]}; //(memb_tmp >>16);
            2'b11: data_af = {23'b0,memb_tmp[31:24]}; //(memb_tmp >>24);
          endcase
        end
        LHU: begin
          case(addr_sp)
            2'b00: data_af = memh_tmp;
            2'b01: data_af = memh_tmp;
            2'b10: data_af = {16'b0,memh_tmp[31:16]}; //memh_tmp >> 16;
            2'b11: data_af = {16'b0,memh_tmp[31:16]}; //memh_tmp >> 16;
          endcase
        end
        LB: begin
          case (addr_sp)
            2'b00: data_af = (memb_tmp[7])  ? {24'hffffff, memb_tmp[7:0]}   : {24'h000000, memb_tmp[7:0]};
            2'b01: data_af = (memb_tmp[15]) ? {24'hffffff, memb_tmp[15:8]}  : {24'h000000, memb_tmp[15:8]};
            2'b10: data_af = (memb_tmp[23]) ? {24'hffffff, memb_tmp[23:16]} : {24'h000000, memb_tmp[23:16]};
            2'b11: data_af = (memb_tmp[31]) ? {24'hffffff, memb_tmp[31:24]} : {24'h000000, memb_tmp[31:24]};
          endcase
          /*case(addr_sp)
            2'b00: data_af = (memb_tmp[7] == 1) ? (memb_tmp | 32'hffffff00): memb_tmp;
            2'b01: data_af = (memb_tmp[15] == 1)? ((memb_tmp >> 8) | 32'hffffff00) : (memb_tmp >>8);
            2'b10: data_af = (memb_tmp[23] == 1)? ((memb_tmp >> 16) | 32'hffffff00) : (memb_tmp >>16);
            2'b11: data_af = (memb_tmp[31] == 1)? ((memb_tmp >> 24) | 32'hffffff00) : (memb_tmp >>24);
          endcase*/
        end
        
        LH: begin
          case (addr_sp)
            2'b00: data_af = (memh_tmp[15]) ? {16'hffff, memh_tmp[15:0]}   : {16'h0000, memh_tmp[15:0]};
            2'b01: data_af = (memh_tmp[15]) ? {16'hffff, memh_tmp[15:0]}   : {16'h0000, memh_tmp[15:0]};
            2'b10: data_af = (memh_tmp[31]) ? {16'hffff, memh_tmp[31:16]}  : {16'h0000, memh_tmp[31:16]};
            2'b11: data_af = (memh_tmp[31]) ? {16'hffff, memh_tmp[31:16]}  : {16'h0000, memh_tmp[31:16]};
          endcase
          /*case(addr_sp)
            2'b00: data_af = (memh_tmp[15] == 1)? (memh_tmp | 32'hffff0000): memh_tmp;
            2'b01: data_af = (memh_tmp[15] == 1)? (memh_tmp | 32'hffff0000): memh_tmp;
            2'b10: data_af = (memh_tmp[31] == 1)? ((memh_tmp >> 16) | 32'hffff0000) : (memh_tmp >>16);
            2'b11: data_af = (memh_tmp[31] == 1)? ((memh_tmp >> 16) | 32'hffff0000) : (memh_tmp >>16);
          endcase*/
        end
        default: data_af = 32'h0;
      endcase
    end
  end
endmodule

module mux_3_1_lsu(
  input logic [31:0] in_data_1_i,
  input logic [31:0] in_data_2_i,
  input logic [31:0] in_data_3_i,
  input logic [31:0] i_lsu_addr,
  output logic [31:0] o_ld_data
);
  logic [1:0] addr_sel ;
 
  always @(*) begin
    addr_sel = 2'b11;
    case (i_lsu_addr[31:12])
      20'h1001_0:  addr_sel  =  2'b00; // SW
      20'h1000_4:  addr_sel  =  2'b01; //LCD
      20'h1000_3:  addr_sel  =  2'b01; //7-seg
      20'h1000_2:  addr_sel  =  2'b01; //7-seg
      20'h1000_1:  addr_sel  =  2'b01; //GRL
      20'h1000_0:  addr_sel  =  2'b01; //RL
      20'h0000_0:  addr_sel  =  2'b10; //MEM
      default: addr_sel = 2'b11; 
    endcase
  end
  always @(*) begin
    o_ld_data = 32'h0;
    case(addr_sel)
    2'b00:o_ld_data = in_data_1_i; // input_bf
    2'b01:o_ld_data = in_data_2_i; // output_bf
    2'b10:o_ld_data = in_data_3_i; // mem
    default: o_ld_data = 32'h0;    
    endcase
  end
endmodule

/*------------------------------------------------------------*/
`endif



`ifndef MUX_2_1
`define MUX_2_1
module mux_2_1 (
  input logic [31:0] data_1_i    ,
  input logic [31:0] data_0_i    ,
  input logic        sel_i       ,
  output logic [31:0] data_out_o		
  );
  
  always_comb begin
    case (sel_i)
      1'b0 : data_out_o = data_0_i;
      1'b1 : data_out_o = data_1_i;
      default: data_out_o = 32'b0;
    endcase
  end
endmodule 
`endif
`ifndef MUX_3_1
`define MUX_3_1
module mux_3_1 (
  input logic [31:0] data_0_i,
  input logic [31:0] data_1_i,
  input logic [31:0] data_2_i,
  input logic [1:0] sel_i,
  output logic [31:0] data_out_o		
  );
  
  always_comb begin
    case (sel_i)
      2'b00 : data_out_o = data_0_i;
      2'b01 : data_out_o = data_1_i;
      2'b10 : data_out_o = data_2_i;
      default : data_out_o = 32'b0;		
    endcase
  end
endmodule
`endif
`ifndef PC
`define PC
/////////Program Counter//////////////////////
module pc (
  input logic i_clk,
  input logic i_reset,
  input logic [31:0] i_pc_data_in,
  output logic [31:0] o_pc_data_out);
  always_ff @(posedge i_clk or posedge i_reset) begin
    if (i_reset) begin
      o_pc_data_out <= 32'b0;
    end
    else 
      o_pc_data_out <= i_pc_data_in;
  end
endmodule
`endif
`ifndef REGFILE
`define REGFILE
module regfile (
  input  logic        i_clk,
  input  logic        i_reset,

  input  logic [4:0]  i_rs1_addr, 
  input  logic [4:0]  i_rs2_addr,

  input  logic [4:0]  i_rd_addr,
  input  logic [31:0] i_rd_data,
  input  logic        i_rd_wren,

  output logic [31:0] o_rs1_data, 
  output logic [31:0] o_rs2_data
);

  logic [31:0] Reg [31:0];

  // --- RESET + WRITE LOGIC ---
  always_ff @(posedge i_clk or posedge i_reset) begin
    if (i_reset) begin
      // Reset tất cả 32 thanh ghi về 0
      for (int i = 0; i < 32; i++)
        Reg[i] <= 32'h0;
    end else begin
      // Chỉ ghi nếu có enable và không ghi vào x0 (r0)
      if (i_rd_wren && (i_rd_addr != 5'd0))
        Reg[i_rd_addr] <= i_rd_data;
    end
  end

  // --- READ LOGIC ---
  assign o_rs1_data = (i_rs1_addr == 5'd0) ? 32'h0 : Reg[i_rs1_addr];
  assign o_rs2_data = (i_rs2_addr == 5'd0) ? 32'h0 : Reg[i_rs2_addr];

endmodule
`endif
`ifndef SHIFT_LEFT_LOGICAL
`define SHIFT_LEFT_LOGICAL
module shift_left_logical (
    input  logic [31:0] data_in,   // Data
    input  logic [4:0]  shift_amt, // Số bit cần dịch
    output logic [31:0] data_out);   // Kết quả


    always @(*) begin
        case (shift_amt)
            5'd0:  data_out = data_in;
            5'd1:  data_out = {data_in[30:0], 1'b0};
            5'd2:  data_out = {data_in[29:0], 2'b0};
            5'd3:  data_out = {data_in[28:0], 3'b0};
            5'd4:  data_out = {data_in[27:0], 4'b0};
            5'd5:  data_out = {data_in[26:0], 5'b0};
            5'd6:  data_out = {data_in[25:0], 6'b0};
            5'd7:  data_out = {data_in[24:0], 7'b0};
            5'd8:  data_out = {data_in[23:0], 8'b0};
            5'd9:  data_out = {data_in[22:0], 9'b0};
            5'd10: data_out = {data_in[21:0], 10'b0};
            5'd11: data_out = {data_in[20:0], 11'b0};
            5'd12: data_out = {data_in[19:0], 12'b0};
            5'd13: data_out = {data_in[18:0], 13'b0};
            5'd14: data_out = {data_in[17:0], 14'b0};
            5'd15: data_out = {data_in[16:0], 15'b0};
            5'd16: data_out = {data_in[15:0], 16'b0};
            5'd17: data_out = {data_in[14:0], 17'b0};
            5'd18: data_out = {data_in[13:0], 18'b0};
            5'd19: data_out = {data_in[12:0], 19'b0};
            5'd20: data_out = {data_in[11:0], 20'b0};
            5'd21: data_out = {data_in[10:0], 21'b0};
            5'd22: data_out = {data_in[9:0], 22'b0};
            5'd23: data_out = {data_in[8:0], 23'b0};
            5'd24: data_out = {data_in[7:0], 24'b0};
            5'd25: data_out = {data_in[6:0], 25'b0};
            5'd26: data_out = {data_in[5:0], 26'b0};
            5'd27: data_out = {data_in[4:0], 27'b0};
            5'd28: data_out = {data_in[3:0], 28'b0};
            5'd29: data_out = {data_in[2:0], 29'b0};
            5'd30: data_out = {data_in[1:0], 30'b0};
            5'd31: data_out = {data_in[0], 31'b0};
            default: data_out = 32'bz;
        endcase
    end

endmodule
`endif
`ifndef SHIFT_RIGHT_ARITHMETIC
`define SHIFT_RIGHT_ARITHMETIC

module shift_right_arithmetic (
    input  logic [31:0] data_in, // Data
    input  logic [4:0] shift_amt, // Số bit cần dịch
    output logic [31:0] data_out); //Kết quả

    always @(*) begin
        case (shift_amt)
            5'd0:  data_out = data_in;
            5'd1:  data_out = {data_in[31], data_in[31:1]};
            5'd2:  data_out = {{2{data_in[31]}}, data_in[31:2]};
            5'd3:  data_out = {{3{data_in[31]}}, data_in[31:3]};
            5'd4:  data_out = {{4{data_in[31]}}, data_in[31:4]};
            5'd5:  data_out = {{5{data_in[31]}}, data_in[31:5]};
            5'd6:  data_out = {{6{data_in[31]}}, data_in[31:6]};
            5'd7:  data_out = {{7{data_in[31]}}, data_in[31:7]};
            5'd8:  data_out = {{8{data_in[31]}}, data_in[31:8]};
            5'd9:  data_out = {{9{data_in[31]}}, data_in[31:9]};
            5'd10: data_out = {{10{data_in[31]}}, data_in[31:10]};
            5'd11: data_out = {{11{data_in[31]}}, data_in[31:11]};
            5'd12: data_out = {{12{data_in[31]}}, data_in[31:12]};
            5'd13: data_out = {{13{data_in[31]}}, data_in[31:13]};
            5'd14: data_out = {{14{data_in[31]}}, data_in[31:14]};
            5'd15: data_out = {{15{data_in[31]}}, data_in[31:15]};
            5'd16: data_out = {{16{data_in[31]}}, data_in[31:16]};
            5'd17: data_out = {{17{data_in[31]}}, data_in[31:17]};
            5'd18: data_out = {{18{data_in[31]}}, data_in[31:18]};
            5'd19: data_out = {{19{data_in[31]}}, data_in[31:19]};
            5'd20: data_out = {{20{data_in[31]}}, data_in[31:20]};
            5'd21: data_out = {{21{data_in[31]}}, data_in[31:21]};
            5'd22: data_out = {{22{data_in[31]}}, data_in[31:22]};
            5'd23: data_out = {{23{data_in[31]}}, data_in[31:23]};
            5'd24: data_out = {{24{data_in[31]}}, data_in[31:24]};
            5'd25: data_out = {{25{data_in[31]}}, data_in[31:25]};
            5'd26: data_out = {{26{data_in[31]}}, data_in[31:26]};
            5'd27: data_out = {{27{data_in[31]}}, data_in[31:27]};
            5'd28: data_out = {{28{data_in[31]}}, data_in[31:28]};
            5'd29: data_out = {{29{data_in[31]}}, data_in[31:29]};
            5'd30: data_out = {{30{data_in[31]}}, data_in[31:30]};
            5'd31: data_out = {{31{data_in[31]}}, data_in[31]};
            default: data_out = 32'bz;
        endcase
    end
endmodule
	
`endif 
`ifndef SHIFT_RIGHT_LOGICAL
`define SHIFT_RIGHT_LOGICAL

module shift_right_logical (
    input  logic [31:0] data_in,   // Data
    input  logic [4:0]  shift_amt, // Số bit cần dịch
    output logic [31:0] data_out);   // Kết quả 


    always @(*) begin
        case (shift_amt)
            5'd0:  data_out = data_in;
            5'd1:  data_out = {1'b0, data_in[31:1]};
            5'd2:  data_out = {2'b0, data_in[31:2]};
            5'd3:  data_out = {3'b0, data_in[31:3]};
            5'd4:  data_out = {4'b0, data_in[31:4]};
            5'd5:  data_out = {5'b0, data_in[31:5]};
            5'd6:  data_out = {6'b0, data_in[31:6]};
            5'd7:  data_out = {7'b0, data_in[31:7]};
            5'd8:  data_out = {8'b0, data_in[31:8]};
            5'd9:  data_out = {9'b0, data_in[31:9]};
            5'd10: data_out = {10'b0, data_in[31:10]};
            5'd11: data_out = {11'b0, data_in[31:11]};
            5'd12: data_out = {12'b0, data_in[31:12]};
            5'd13: data_out = {13'b0, data_in[31:13]};
            5'd14: data_out = {14'b0, data_in[31:14]};
            5'd15: data_out = {15'b0, data_in[31:15]};
            5'd16: data_out = {16'b0, data_in[31:16]};
            5'd17: data_out = {17'b0, data_in[31:17]};
            5'd18: data_out = {18'b0, data_in[31:18]};
            5'd19: data_out = {19'b0, data_in[31:19]};
            5'd20: data_out = {20'b0, data_in[31:20]};
            5'd21: data_out = {21'b0, data_in[31:21]};
            5'd22: data_out = {22'b0, data_in[31:22]};
            5'd23: data_out = {23'b0, data_in[31:23]};
            5'd24: data_out = {24'b0, data_in[31:24]};
            5'd25: data_out = {25'b0, data_in[31:25]};
            5'd26: data_out = {26'b0, data_in[31:26]};
            5'd27: data_out = {27'b0, data_in[31:27]};
            5'd28: data_out = {28'b0, data_in[31:28]};
            5'd29: data_out = {29'b0, data_in[31:29]};
            5'd30: data_out = {30'b0, data_in[31:30]};
            5'd31: data_out = {31'b0, data_in[31]};
            default: data_out = 32'bz;
        endcase
    end

endmodule
`endif 
`ifndef SLT_SLTU
`define SLT_SLTU 
//`include "add_sub_32_bit.sv"
module slt_sltu (
    input  logic [31:0] A, B,  // Input A, B
    input  logic Sel,          // 0 = SLT (có dấu), 1 = SLTU (không dấu)
    output logic [31:0] Result); // Kết quả

    logic [31:0] diff_out;  // Kết quả phép trừ A - B
    logic carry_out;        // Carry/Borrow từ phép trừ

    add_sub_32_bit SUB(
        .A(A),
        .B(B), 
        .Sel(1'b1), 
        .Result(diff_out),
        .Cout(carry_out)
    );

    // So sánh
    always @(*) begin
        if (Sel == 1'b0) begin // SLT
            Result = {31'd0,diff_out[31]};  
        end else begin //SLTU
            Result = {31'd0,~carry_out};
        end
    end

endmodule
`endif 