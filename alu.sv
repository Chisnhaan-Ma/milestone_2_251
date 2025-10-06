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
    Add_Sub_32bit ADD_SUB( 
	 .A(i_operand_a),
	 .B(i_operand_b),
	 .Sel(i_alu_op[3]),
	 .Result(add_sub_out)); 
	 
	 // SLL
    Shift_Left_Logical SLL(
	 .data_in(i_operand_a),
	 .shift_amt(i_operand_b[4:0]),
	 .data_out(sll_out)); // SLL

	 // SRL
    Shift_Right_Logical SRL(
	 .data_in(i_operand_a), 
	 .shift_amt(i_operand_b[4:0]),
	 .data_out(srl_out)); 
	 
	 // SRA
    Shift_Right_Arithmetic SRA(
	 .data_in(i_operand_a), 
	 .shift_amt(i_operand_b[4:0]),
	 .data_out(sra_out)); 
	 
	 // SLT
    SLT_SLTU SLT_MODULE(
	 .A(i_operand_a), 
	 .B(i_operand_b), 
	 .Sel(1'b0),
	 .Result(slt_out));  
	 
	 // SLT
    SLT_SLTU SLTU_MODULE(
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
            default: data_out = 32'bz;
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
            default: data_out = 32'bz;
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
            default: data_out = 32'bz;
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