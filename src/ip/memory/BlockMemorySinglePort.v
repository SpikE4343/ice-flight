
module BlockMemorySinglePort
#(
  parameter DEPTH = 8,
  parameter DATA_WIDTH = 8
)
(
  input clk,
  input reset,
  input write,
  input [$clog2(DEPTH)-1:0] address,
  input [DATA_WIDTH-1:0] value,
  output [DATA_WIDTH-1:0] data
);
    
  reg [DATA_WIDTH-1:0] mem [DEPTH-1:0];  

  always @(posedge clk) begin
    if(write) 
      mem[address] <= value;
  end

  assign data = mem[address];


endmodule
