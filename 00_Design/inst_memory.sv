module inst_memory 
  (
  output logic [31:0] o_rdata,
  input  logic [31:0] i_addr ,
  input  logic        i_reset
);

  logic [31:0] imem [0:2048];
  initial begin
    $readmemh("D:/HCMUT/Year_2025_2026/251/Conmputer_Organization/milestone_2/00_Design/mem.dump",imem);
    $$display("Read mem.dump to instruction memory");
  end
  always_comb begin
    if (i_reset==0) begin
      o_rdata = 32'h0;  
    end else begin
      o_rdata = imem[i_addr[31:2]];  
    end
  end
endmodule
