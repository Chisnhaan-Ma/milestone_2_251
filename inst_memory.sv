module inst_memory 
  (
  output logic [31:0] o_rdata,
  input  logic [13:0] i_addr ,
  input  logic        i_reset
);

  logic [31:0] imem [2048:0];
  initial begin
    $readmemh("/mnt/d/RTL/Milestone2_submit/02_test/dump/mem.dump", imem);
  end
always_comb begin
    if (i_reset) begin
      o_rdata = 32'h0;  
    end else begin
      o_rdata = imem[i_addr[13:2]];  
    end
  end
endmodule
