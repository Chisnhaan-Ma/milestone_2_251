`include "singlecycle.sv"
module Singlecycle_Tb();

    logic clk;
    logic reset;

    singlecycle singlecycle_test_top(
        .i_clk(clk),
        .i_reset(reset)
    );
      // Clock generation
    always #5 clk = ~clk;
	
    initial begin
        $dumpfile("wave.vcd");      // ✅ tên file VCD sẽ sinh ra
        $dumpvars(0, Singlecycle_Tb); // ✅ tên module testbench top-level
        //force singlecycle_test_top.regfile_top.i_rd_wren = 1'b1;
        clk = 0;
        reset = 1;    // Reset để PC = 0
        #3;reset = 0; 
        #500;
        $finish;  
    end
endmodule
