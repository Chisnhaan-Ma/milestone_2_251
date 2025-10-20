module RISC_single_cycle(
	input logic clk,
	input logic reset);
	
	///CPU internal signal///
	logic[31:0]PC_in;
	logic[31:0]PC_out;
	logic [31:0]inst;
	logic [4:0]inst_rsw;
	logic [4:0]inst_rs1;
	logic [4:0]inst_rs2;
	logic [31:0]data_1;
	logic [31:0]data_2;
	logic [31:0]ALU_A;
	logic [31:0]ALU_B;
	logic [31:0]ALU_out;
	logic [31:0]Imm_out;
	logic [31:0]dataR_DMEM;
	logic [31:0] PC_add4_out;
	logic [31:0] WBSel_out;
	logic BrEq;
	logic BrLt;
	logic BrUn;
	logic Asel;
	logic Bsel;
	logic [3:0]ALU_sel;
	logic MemRW;
	logic [1:0]WBSel;
	logic regWEn;
	logic [2:0]Imm_Sel;
	logic PCSel;
	logic [2:0]load_type;
	logic [31:0]load_result;
	
	//Module instance//
	PC PC_instance (
	.clk(clk),
	.reset(reset), 
	.data_in(PC_in),
	.data_out(PC_out));
	
	IMEM IMEM_instance( 
	.addr(PC_out), 
	.inst(inst),
	.inst_rsw(inst_rsw),
	.inst_rs1(inst_rs1),
	.inst_rs2(inst_rs2));
  
	regfile regfile_instance(
	.clk(clk),
	.data_W(WBSel_out),
	.rsW(inst_rsw),
	.rs1(inst_rs1),
	.rs2(inst_rs2),
	.data_1(data_1),
	.data_2(data_2),
	.regWEn(regWEn),
	.reset(reset));
	
	ALU ALU_instance(
	.A(ALU_A),
	.B(ALU_B),
	.ALU_out(ALU_out),
	.ALU_sel(ALU_sel));
	
	Imm_Gen Imm_Gen_instance(
	.Imm_out(Imm_out),
	.Imm_Sel(Imm_Sel),
	.inst(inst));
	
	DMEM DMEM_instance(
	.addr(ALU_out),
	.clk(clk),
	.MemRW(MemRW),
	.dataW(data_2),
	.dataR(dataR_DMEM));
	
	Add_Sub_32bit PC_add4(
	.A(PC_out),
	.B(32'd4),
	.Sel(1'b0),
	.Result(PC_add4_out));
	
   MUX4to1 Writeback(
	.sel(WBSel),
	.in0(load_result),
	.in1(ALU_out),
	.in2(PC_add4_out),
	.in3(32'b0),
	.out(WBSel_out));
	
	brc brc_instance(
	.BrUn(BrUn),
	.data_1(data_1),
	.data_2(data_2),
	.BrLt(BrLt),
	.BrEq(BrEq));
	
	Control_unit Control_unit_instance(
	.inst(inst),
	.BrEq(BrEq),
	.BrLt(BrLt),
	.PCSel(PCSel),
	.Imm_Sel(Imm_Sel),
	.regWEn(regWEn),
	.BrUn(BrUn),
	.Asel(Asel),
	.Bsel(Bsel),
	.ALU_sel(ALU_sel),
	.MemRW(MemRW),
	.WBSel(WBSel),
	.load_type(load_type));
	
	Load_encode Load_encode_instance(
	.load_data(dataR_DMEM),
	.load_type(load_type),
	.load_result(load_result));
	
	assign ALU_A = Asel ? PC_out: data_1;
	assign ALU_B = Bsel ? Imm_out: data_2;
	assign PC_in = PCSel ? ALU_out: PC_add4_out;	
	
endmodule


///Control Unit//////
module Control_unit(
	input logic [31:0]inst,
	input logic BrEq,
	input logic BrLt,
	output logic PCSel,
	output logic [2:0]Imm_Sel,
	output logic regWEn,
	output logic BrUn,
	output logic Bsel,
	output logic Asel,
	output logic [3:0]ALU_sel,
	output logic MemRW,
	output logic [2:0]load_type,
	output logic [1:0]WBSel);
	
	logic [6:0] opcode;
    logic [2:0] funct3;
    logic [6:0] funct7;

   assign opcode = inst[6:0];
   assign funct3 = inst[14:12];
	
    always @(*) begin
        // Giá trị mặc định
        PCSel   = 1'b0;
        Imm_Sel = 3'b000;
		regWEn  = 1'b0;
        BrUn    = 1'b0;
        Bsel    = 1'b0;
        Asel    = 1'b0;
        ALU_sel = 4'b0000;
        MemRW   = 1'b0;
        WBSel   = 2'b00;
		load_type = 3'b0;
		case (opcode)

			7'b0110011: begin  // R-type 
            regWEn  = 1'b1;//cho phép ghi reg file
            WBSel   = 2'b1;// chọn write back từ ngõ ra ALU
            Asel    = 1'b0;// chọn A từ rs1
            Bsel    = 1'b0;//chọn B từ rs2
			load_type = 3'b0; //không load
				case(funct3)
					3'b000: ALU_sel = (inst[30]) ? 4'b1000 : 4'b0000; // SUB nếu inst[30] = 1, ADD nếu inst[30] = 0
					3'b001: ALU_sel = 4'b0001; // SLL
					3'b010: ALU_sel = 4'b0010; // SLT
					3'b011: ALU_sel = 4'b0011; // SLTU
					3'b100: ALU_sel = 4'b0100; // XOR
					3'b101: ALU_sel = (inst[30]) ? 4'b1101 : 4'b0101; // SRA nếu inst[30] = 1, SRL nếu inst[30] = 0
					3'b110: ALU_sel = 4'b0110; // OR
					3'b111: ALU_sel = 4'b0111; // AND
					default: ALU_sel = 4'b0000; // Mặc định là ADD
				endcase
			end
			

			7'b0010011: begin  // I-type 
			regWEn  = 1'b1;//cho phép ghi reg file
            WBSel   = 2'b1;// chọn write back từ ngõ ra ALU
            Asel    = 1'b0;// chọn A từ rs1
            Bsel    = 1'b1;//chọn B từ rs2
				Imm_Sel = 3'b000;// Imm_Gen theo I type
				MemRW   = 1'b0;	// Cho phép đọc đọc DMEM
				load_type = 3'b0; // không load
				case(funct3)
					3'b000: ALU_sel = 4'b0000; // ADDI
					3'b001: ALU_sel = 4'b0001; // SLLI
					3'b010: ALU_sel = 4'b0010; // SLTI
					3'b011: ALU_sel = 4'b0011; // SLTUI
					3'b100: ALU_sel = 4'b0100; // XORI
					3'b101: ALU_sel = (inst[30]) ? 4'b1101 : 4'b0101; // SRAI nếu inst[30] = 1, SRLI nếu inst[30] = 0
					3'b110: ALU_sel = 4'b0110; // ORI
					3'b111: ALU_sel = 4'b0111; // ANDI
					default: ALU_sel = 4'b0000; // Mặc định là ADD
				endcase
				end
				
			7'b0000011: begin  // Load
			regWEn  = 1'b1; // Cho phép ghi lại vào regfile
            WBSel   = 2'b0; // Lấy dữ liệu từ DMEM
            Asel    = 1'b0; // Chọn Rs1 + Imm_Gen
            Bsel    = 1'b1; // Chọn Imm_Gen
				Imm_Sel = 3'b000;	// Imm_Gen theo I type
				ALU_sel = 4'b0000;	// Thực hiện phép cộng
				MemRW   = 1'b0;	// Cho phép đọc đọc DMEM
				case(funct3)
					3'b000: load_type = 3'b000; //LB
					3'b001: load_type = 3'b001; //LH
					3'b010: load_type = 3'b010; //LW
					3'b100: load_type = 3'b100; //LBU					
					3'b101: load_type = 3'b101; //LHU			
					default: load_type = 3'b111; 
				endcase
			end
			
			7'b0100011: begin //	S-type
				Imm_Sel = 3'b001;	// Imm_Gen theo S type
				regWEn = 1'b0; //không cho ghi lại vào reg file
				Asel = 1'b0; //chọn A là rs1 
				Bsel = 1'b1; //chọn B là Imm_Gen theo S type
				MemRW = 1'b1; //cho phép đọc và ghi DMEM
				ALU_sel = 4'b0000;	// Thực hiện phép cộng
				WBSel = 2'b11; //write back là tùy định vì không ghi ngược lại vào regfile
			end
			
			7'b1100011: begin	// B-type
				Imm_Sel = 3'b010;	//Imm_Gen theo B type
				regWEn = 1'b0;	//Không ghi lại vào reg file
				Asel = 1'b1;	// A chọn PC hiện hành
				Bsel = 1'b1;	// B chọn imm_Gen theo B type
				ALU_sel = 4'b0000;	//ALU thực hiện phép cộng
				MemRW = 1'b0;	// Cho phép đọc DMEM
				WBSel = 2'b11; // write back là tùy định vì không ghi ngược lại vào regfile
				case(funct3)
					3'b000: begin BrUn = 1'b1; PCSel = BrEq;	end//BEQ
					3'b001: begin BrUn = 1'b1; PCSel = ~BrEq;	end//BNE
					3'b100: begin BrUn = 1'b0;	PCSel = BrLt;	end//BLT
					3'b101: begin BrUn = 1'b0; PCSel = ~BrLt; end//BGE
					3'b110: begin BrUn = 1'b1;	PCSel = BrLt;	end//BLTU
					3'b111: begin BrUn = 1'b1;	PCSel = ~BrLt;	end//BGEU
					default: PCSel = 1'b0; // Không nhảy nếu opcode không hợp lệ
				endcase
			end
			
			7'b1101111: begin // J-type JAL
				PCSel = 1'b1;	//PCSel chọn luôn nhảy
				Imm_Sel = 3'b011; // Imm_Gen theo J Type
				regWEn = 1'b1;	// cho phép ghi ngược vào regfile
				Bsel = 1'b1;	// B chọn Imm_Gen theo J type
				Asel  =1'b1;	// A chọn PC hiện hành
				ALU_sel = 4'b0000;	// ALU thực hiện phép cộng
				MemRW = 1'b0;	// Cho phép đọc DMEM
				WBSel = 2'b10;	// Write back chọn PC+4 để return
			end
			
			7'b1100111: begin // I-type JALR
				PCSel = 1'b1;	//PCSel chọn luôn nhảy
				Imm_Sel = 3'b000; // Imm_Gen theo I Type
				regWEn = 1'b1;	// cho phép ghi ngược vào regfile
				Bsel = 1'b1;	// B chọn Imm_Gen theo I type
				Asel = 1'b0;	// A chọn rs1
				ALU_sel = 4'b0000;	// ALU thực hiện phép cộng
				MemRW = 1'b0;	// Cho phép đọc DMEM
				WBSel = 2'b10; // Write back chọn PC+4 để return
			end
			7'b0110111: begin // U-type LUI
				PCSel = 1'b0;	// PCSel chọn không nhảy
				Imm_Sel = 3'b100; //Imm_Gen theo U type LUI
				regWEn = 1'b1; //cho phép ghi vào reg file
				Bsel = 1'b1;	// B chọn Imm_Gen theo U type LUI
				Asel = 1'b0;	// A tùy định
				ALU_sel = 4'b1111; //ALU dẫn B ra ALU_out
				MemRW = 1'b0; // Cho phép đọc DMEM
				WBSel = 2'b01; // chọn write back từ ALU_out
			end
			7'b0010111: begin // U-type AUIPC
				PCSel = 1'b0;	// PCSel chọn không nhảy
				Imm_Sel = 3'b101; //Imm_Gen theo U type AUIPC
				regWEn = 1'b1; //cho phép ghi vào reg file
				Bsel = 1'b1;	// B chọn Imm_Gen theo U type LUI
				Asel = 1'b1;	// A tùy lấy PC hiện hành
				ALU_sel = 4'b0000; //ALU thực hiện phép cộng
				MemRW = 1'b0; // Cho phép đọc DMEM
				WBSel = 2'b01; // chọn write back từ ALU_out
			end
			default begin
				PCSel   = 1'b0;
				Imm_Sel = 3'b000;
				regWEn  = 1'b0;
				BrUn    = 1'b0;
				Bsel    = 1'b0;
				Asel    = 1'b0;
				ALU_sel = 4'b0000;
				MemRW   = 1'b0;
				WBSel   = 2'b00;
			end
		endcase
	end
endmodule
/////////Program Counter//////////////////////
module PC (
  input logic clk,
  input logic reset,
  input logic [31:0] data_in,
  output logic [31:0] data_out);
  always_ff @(posedge clk or posedge reset) begin
    if (reset)
      data_out <= 0;
    else 
      data_out <= data_in;
  end
endmodule


//////////Register File ///////////////////////
module regfile(
	input logic clk, regWEn, reset,
	input logic [31:0]data_W,
	input logic [4:0]rsW,
	input logic [4:0]rs1,
	input logic [4:0]rs2,
	output logic [31:0]data_1,
	output logic [31:0]data_2);
	
	logic [31:0]reg_mem[31:0];
	
	always_ff @(posedge clk or posedge reset) begin
      if (reset) begin
            for (int i = 1; i < 32; i++) begin
                reg_mem[i] <= 32'b0;  //reset regfile = 0
            end
		end
		else if(regWEn && (rsW != 5'h0)) reg_mem[rsW] <= data_W;
		else reg_mem[rsW] <= reg_mem[rsW];
		
	end
	assign data_1 = (rs1 == 5'd0) ? 32'b0 : reg_mem[rs1];
	assign data_2 = (rs2 == 5'd0) ? 32'b0 : reg_mem[rs2];
	
	
endmodule

///////Instruction Memory ////////////////////////	
module IMEM (
	input logic [31:0]addr,
	output logic [31:0]inst,
	output logic [4:0]inst_rsw,
	output logic [4:0]inst_rs1,
	output logic [4:0]inst_rs2);
	
	logic [31:0] memory [0:256];
    initial begin
     $readmemh("D:/HCMUT/Year_2025_2026/251/Conmputer_Organization/milestone_2/design/mem.dump", memory);
    end
	assign inst = memory[addr[31:2]];
	assign inst_rsw = memory[addr[31:0]][11:7];
	assign inst_rs1 = memory[addr[31:0]][19:15];
	assign inst_rs2 = memory[addr[31:0]][24:20];	
endmodule

///////Data Memory ////////////////////////
module DMEM(
	input logic [31:0]addr,
	input logic [31:0]dataW,
	output logic [31:0]dataR,
	input logic clk, MemRW);
	
	logic [31:0]memory[0:255];

	always_ff @ (posedge clk) begin
		if(MemRW) memory[addr] <= dataW;	// MemRW = 1 để write
	end
	assign dataR = memory[addr];	//luôn ở trạng thái READ
endmodule	

////////Load Encoding/////
module Load_encode(
    input  logic [31:0] load_data, // Data từ DMEM
    input  logic [2:0]  load_type,  // Chọn kiểu load
    output logic [31:0] load_result); 
	 
    always @(*) begin
        case (load_type)
            3'b000: load_result = {{24{load_data[7]}}, load_data[7:0]};  // Load byte mở rộng dấu
            3'b001: load_result = {{16{load_data[15]}}, load_data[15:0]}; // Load haftword mở rộng dấu
            3'b010: load_result = load_data;  // LW 
            3'b100: load_result = {24'b0, load_data[7:0]};  // LBU 
            3'b101: load_result = {16'b0, load_data[15:0]}; // LHU 
            default: load_result = 32'b0;  
        endcase
    end

endmodule
/////////Immediate generator///////////////
module Imm_Gen(	
	input logic [2:0] Imm_Sel, // Chọn kiểu generate
	input logic [31:0] inst,	// Instruction
	output logic [31:0] Imm_out); // Kết quả
	// I type = 000	7'b0010011
	// S type = 001	7'b0100011
	// B type = 010	7'b1100011
	// J type = 011	7'b1101111	JAL
	// U type = 100	7'b0110111 	LUI
	// U type = 101	7'b0010111	AUIPC
always @(*) begin
	case(Imm_Sel) 
		3'b000: begin //I typte
          Imm_out = {{20{inst[31]}}, inst[31:20]};
		end
		3'b001: begin // S type
          Imm_out = {{20{inst[31]}}, inst[31:25], inst[11:7]};
		end
		3'b010: begin //B type
          Imm_out = {{20{inst[31]}}, inst[7],inst[30:25], inst[11:8],1'b0};
		end
		3'b011: begin // J type JAL
          Imm_out = {{11{inst[31]}}, inst[31], inst[19:12], inst[20], inst[30:21], 1'b0};
		end
		3'b100: begin // U type LUI
			Imm_out = {inst[31:12], 12'b0};
		end
		3'b101: begin // U type AUIPC
			Imm_out = {inst[31:12], 12'b0};
		end
		default: Imm_out = 32'b0;
	endcase
end
endmodule
	

module Shift_Left_Logical (
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
            default: data_out = 32'b0;
        endcase
    end

endmodule

module Shift_Right_Logical (
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
            default: data_out = 32'b0;
        endcase
    end

endmodule

module Shift_Right_Arithmetic (
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
            default: data_out = 32'b0;
        endcase
    end
endmodule

module Add_Sub_32bit (
    input  logic [31:0] A, B,  // Input A, B
    input  logic Sel,          // 0 = ADD, 1 = SUB
    output logic [31:0] Result,// Kết quả phép cộng 
    output logic Cout);          // Carry-out

    logic [31:0] B_mod;        // 
    logic Cin;                 // Carry-in
    logic [31:0] carry;  // Carry signals
    assign B_mod = (Sel) ? ~B : B;  // Bù 2 của B

    Full_Adder FA0(
	 A[0],
	 B_mod[0],
	 Sel,
	 Result[0],
	 carry[0]);

    // Generate 31 more full adders
    genvar i;
    generate
        for (i = 1; i < 32; i = i + 1) begin :adder_32
            Full_Adder FA (A[i], B_mod[i], carry[i-1], Result[i], carry[i]);
        end
    endgenerate

    assign Cout = carry[31];

endmodule

	
module Full_Adder (
    input  logic A,
	 input logic B,
	 input logic Cin,
    output logic Sum,
	 output logic Cout);
	 
    assign Sum  = A ^ B ^ Cin;
    assign Cout = (A & B) | (Cin & (A ^ B));

endmodule
	
module SLT_SLTU (
    input  logic [31:0] A, B,  // Input A, B
    input  logic Sel,          // 0 = SLT (có dấu), 1 = SLTU (không dấu)
    output logic [31:0] Result); // Kết quả

    logic [31:0] diff_out;  // Kết quả phép trừ A - B
    logic carry_out;        // Carry/Borrow từ phép trừ

    Add_Sub_32bit SUB(
        .A(A),
        .B(B), 
        .Sel(1'b1), 
        .Result(diff_out),
        .Cout(carry_out)
    );

    // So sánh
    always @(*) begin
        if (Sel == 1'b0) begin // SLT
            Result = diff_out[31];  
        end else begin //SLTU
            Result = ~carry_out;
        end
    end

endmodule

	
module ALU(
    input  logic [3:0]  ALU_sel, // Chọn phép tính
    input  logic [31:0] A,	// Toán hạng 1
    input  logic [31:0] B, // Toán hạng 2
    output logic [31:0] ALU_out); // Kết quả
	 
    logic [31:0] add_sub_out, sll_out, srl_out, sra_out;
    logic [31:0] slt_out, sltu_out;

    // Instance các module cần dùng
	 
	 // ADD, SUB
    Add_Sub_32bit ADD_SUB( 
	 .A(A),
	 .B(B),
	 .Sel(ALU_sel[3]),
	 .Result(add_sub_out)); 
	 
	 // SLL
    Shift_Left_Logical SLL(
	 .data_in(A),
	 .shift_amt(B[4:0]),
	 .data_out(sll_out)); // SLL

	 // SRL
    Shift_Right_Logical SRL(
	 .data_in(A), 
	 .shift_amt(B[4:0]),
	 .data_out(srl_out)); 
	 
	 // SRA
    Shift_Right_Arithmetic SRA(
	 .data_in(A), 
	 .shift_amt(B[4:0]),
	 .data_out(sra_out)); 
	 
	 // SLT
    SLT_SLTU SLT_MODULE(
	 .A(A), 
	 .B(B), 
	 .Sel(1'b0),
	 .Result(slt_out));  
	 
	 // SLT
    SLT_SLTU SLTU_MODULE(
	 .A(A),
	 .B(B), 
	 .Sel(1'b1), 
	 .Result(sltu_out));  

    always @(*) begin
        case (ALU_sel)
            4'b0000: ALU_out = add_sub_out;  // ADD
            4'b1000: ALU_out = add_sub_out;  // SUB
            4'b0001: ALU_out = sll_out;      // SLL
            4'b0010: ALU_out = slt_out;  // SLT (1 bit)
            4'b0011: ALU_out = sltu_out; // SLTU (1 bit)
            4'b0100: ALU_out = A ^ B;  // XOR
            4'b0101: ALU_out = srl_out; // SRL
            4'b1101: ALU_out = sra_out; // SRA
            4'b0110: ALU_out = A | B;  // OR
            4'b0111: ALU_out = A & B;  // AND
			4'b1111: ALU_out = B; //Cho lệnh LUI
            default: ALU_out = 32'b0;  
        endcase
    end
endmodule	
	
module MUX4to1 (
    input logic [1:0] sel, // Chọn nguồn writeback
    input logic [31:0] in0, // Writeback từ DMEM qua Load
	 input logic [31:0] in1, // Writeback từ ALU
	 input logic [31:0] in2, // Writeback PC + 4 cho lệnh JALR
	 input logic [31:0]	in3, // Trống
    output logic [31:0] out); // Ngõ ra writeback
	 
    always @(*) begin
        case (sel)
            2'b00: out = in0;
            2'b01: out = in1;
            2'b10: out = in2;
            2'b11: out = in3;
            default: out = 32'd0; 
        endcase
    end
endmodule
	
module brc (
    input logic BrUn, // 1: so sánh unsigned, 0: so sánh signed
    input logic [31:0] data_1,  // Toán hạng 1
	 input logic [31:0] data_2,  // Toán hạng 2
    output logic BrEq,	// 1: Bằng, 0: Không bằng 
	 output logic BrLt); // 1: (1)<(2) , 0: (1) >= 2;

    logic [31:0] Diff;  // Kết quả phép trừ rs1 - rs2
    logic Cout;         // Carry-out từ Add_Sub_32bit

    Add_Sub_32bit subtractor (
        .A(data_1),
        .B(data_2),
        .Sel(1'b1),   // SUB
        .Result(Diff),
        .Cout(Cout)
    );

    // BrEq
    always @(*) begin
        if (Diff == 32'b0)
            BrEq = 1'b1;
        else
            BrEq = 1'b0;
    end

    // BrLt
    always @(*) begin
        if (BrUn)  // Unsigned
            BrLt = ~Cout;
        else       // Signed
            BrLt = Diff[31];
    end

endmodule

module RISC_single_cycle_tb;

  logic clk;
  logic reset;

  // Instantiate DUT
  RISC_single_cycle abc(.clk(clk),.reset(reset));

  // Clock generation
  always #5 clk = ~clk;
	
  initial begin
    $dumpfile("RISC_single_cycle_tb.vcd");
    $dumpvars(0, RISC_single_cycle_tb);
    clk = 1;
    reset = 1;    // Reset để PC = 0
    #3;
    reset = 0;   
    #50;
    $finish;          
  end

endmodule
	
	
	
	
	
	
	
	