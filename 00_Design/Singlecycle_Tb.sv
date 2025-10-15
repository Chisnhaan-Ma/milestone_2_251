`ifndef SINGLE_CYCLE_TB
`define SINGLE_CYCLE_TB
`include "singlecycle.sv"
`timescale 1ps/1ps
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
        $dumpfile("wave.vcd");      // file VCD sẽ sinh ra
        $dumpvars(0, Singlecycle_Tb); //tên module testbench top-level
        clk = 0;
        reset = 1;    // Reset để PC = 0
        #3ps;reset = 0; 
        #1000ps;
        $finish;  
    end
endmodule
`endif