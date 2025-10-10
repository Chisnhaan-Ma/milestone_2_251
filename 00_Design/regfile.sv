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
  always_ff @(posedge i_clk or negedge i_reset) begin
    if (!i_reset) begin
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
