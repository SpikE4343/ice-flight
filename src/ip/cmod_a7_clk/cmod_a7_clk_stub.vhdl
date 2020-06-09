-- Copyright 1986-2019 Xilinx, Inc. All Rights Reserved.
-- --------------------------------------------------------------------------------
-- Tool Version: Vivado v.2019.2 (lin64) Build 2708876 Wed Nov  6 21:39:14 MST 2019
-- Date        : Tue May 19 14:26:35 2020
-- Host        : john-Inspiron-7570 running 64-bit Ubuntu 20.04 LTS
-- Command     : write_vhdl -force -mode synth_stub
--               /home/john/code/fpga/projects/ice-flight/src/ip/cmod_a7_clk/cmod_a7_clk_stub.vhdl
-- Design      : cmod_a7_clk
-- Purpose     : Stub declaration of top-level module interface
-- Device      : xc7a35tcpg236-1
-- --------------------------------------------------------------------------------
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

entity cmod_a7_clk is
  Port ( 
    clk_out1 : out STD_LOGIC;
    reset : in STD_LOGIC;
    locked : out STD_LOGIC;
    clk_in1 : in STD_LOGIC
  );

end cmod_a7_clk;

architecture stub of cmod_a7_clk is
attribute syn_black_box : boolean;
attribute black_box_pad_pin : string;
attribute syn_black_box of stub : architecture is true;
attribute black_box_pad_pin of stub : architecture is "clk_out1,reset,locked,clk_in1";
begin
end;
