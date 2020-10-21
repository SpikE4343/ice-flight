
module BlockMemoryDualPort
#(
  parameter DEPTH = 8,
  parameter DATA_WIDTH = 8
)(
  input reset,

  input p1_clk,
  input p1_write,
  input p1_read,
  input [$clog2(DEPTH)-1:0] p1_address,
  input [DATA_WIDTH-1:0] p1_write_data,
  output [DATA_WIDTH-1:0] p1_read_data,

  input p2_clk,
  input p2_write,
  input p2_read,
  input [$clog2(DEPTH)-1:0] p2_address,
  input [DATA_WIDTH-1:0] p2_write_data,
  output [DATA_WIDTH-1:0] p2_read_data
);
  
  
  reg [DATA_WIDTH-1:0] mem [DEPTH-1:0];  

  reg [DATA_WIDTH-1:0] p1_read_value;
  reg [DATA_WIDTH-1:0] p2_read_value;

  // First port
  always @(posedge p1_clk) begin
    if(p1_write) 
      mem[p1_address] <= p1_write_data;

    if(p1_read)
      p1_read_value <= mem[p1_address];

  end

  assign p1_read_data = p1_read_value; 

  // Second port
  always @(posedge p2_clk) begin
    if(p2_write) 
      mem[p2_address] <= p2_write_data;

    if(p2_read)
      p2_read_value <= mem[p2_address];
  end

  assign p2_read_data = p2_read_value;


endmodule
