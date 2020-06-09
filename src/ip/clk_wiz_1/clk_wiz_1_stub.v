// Copyright 1986-2019 Xilinx, Inc. All Rights Reserved.
// --------------------------------------------------------------------------------
// Tool Version: Vivado v.2019.2 (lin64) Build 2708876 Wed Nov  6 21:39:14 MST 2019
// Date        : Tue May 19 14:01:45 2020
// Host        : john-Inspiron-7570 running 64-bit Ubuntu 20.04 LTS
// Command     : write_verilog -force -mode synth_stub
//               /home/john/code/fpga/projects/ice-flight/src/ip/clk_wiz_1/clk_wiz_1_stub.v
// Design      : clk_wiz_1
// Purpose     : Stub declaration of top-level module interface
// Device      : xc7a35tcpg236-1
// --------------------------------------------------------------------------------

// This empty module with port declaration file causes synthesis tools to infer a black box for IP.
// The synthesis directives are for Synopsys Synplify support to prevent IO buffer insertion.
// Please paste the declaration into a Verilog source file or add the file as an additional source.
module clk_wiz_1(clk_100mhz, reset, locked, clk_in1)
/* synthesis syn_black_box black_box_pad_pin="clk_100mhz,reset,locked,clk_in1" */;
  output clk_100mhz;
  input reset;
  output locked;
  input clk_in1;
endmodule
