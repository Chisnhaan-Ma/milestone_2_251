/////////Program Counter//////////////////////
module pc (
  input logic i_clk,
  input logic i_reset,
  input logic [31:0] i_pc_data_in,
  output logic [31:0] o_pc_data_out);
  always_ff @(posedge i_clk or negedge i_reset) begin
    if (i_reset==0) begin
      o_pc_data_out <= 32'b0;
    end
    else 
      o_pc_data_out <= i_pc_data_in;
  end
endmodule