/////////Immediate generator///////////////
module imm_gen(	
	input logic [2:0] i_imm_sel, // Chọn kiểu generate
	input logic [31:0] i_inst,	// i_instruction
	output logic [31:0] i_imm_out); // Kết quả
	// I type = 000	7'b0010011
	// S type = 001	7'b0100011
	// B type = 010	7'b1100011
	// J type = 011	7'b1101111	JAL
	// U type = 100	7'b0110111 	LUI
	// U type = 101	7'b0010111	AUIPC
always @(*) begin
	case(i_imm_sel) 
		3'b000: begin //I typte
          i_imm_out = {{20{i_inst[31]}}, i_inst[31:20]};
		end
		3'b001: begin // S type
          i_imm_out = {{20{i_inst[31]}}, i_inst[31:25], i_inst[11:7]};
		end
		3'b010: begin //B type
          i_imm_out = {{20{i_inst[31]}}, i_inst[7],i_inst[30:25], i_inst[11:8],1'b0};
		end
		3'b011: begin // J type JAL
          i_imm_out = {{11{i_inst[31]}}, i_inst[31], i_inst[19:12], i_inst[20], i_inst[30:21], 1'b0};
		end
		3'b100: begin // U type LUI
			i_imm_out = {i_inst[31:12], 12'b0};
		end
		3'b101: begin // U type AUIPC
			i_imm_out = {i_inst[31:12], 12'b0};
		end
		default: i_imm_out = 32'bz;
	endcase
end
endmodule