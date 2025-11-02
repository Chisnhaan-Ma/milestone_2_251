`ifndef SINGLE_CYCLE_TB
`define SINGLE_CYCLE_TB
`include "singlecycle.sv"
`timescale 1ps/1ps
module Singlecycle_Tb();

    logic clk;
    logic reset;
    logic [31:0] io_sw;
    singlecycle singlecycle_test_top(
        .i_clk(clk),
        .i_reset(reset),
        .i_io_sw(io_sw)
    );
      // Clock generation
    always #5 clk = ~clk;
	
    initial begin
        $dumpfile("wave.vcd");      // file VCD sẽ sinh ra
        $dumpvars(0, Singlecycle_Tb); //tên module testbench top-level
        clk = 0;
        reset = 1;    // Reset để PC = 0
        #3ps;
        force reset = 0; 
        force io_sw = 32'h0c;
        #2000;
        force io_sw = 32'h9;
        #20000;
        $finish;  
    end
endmodule
`endif