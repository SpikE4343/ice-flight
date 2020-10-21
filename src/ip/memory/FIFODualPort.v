
// TODO: Some how to force DEPTH to be a power of two?
module FIFODualPort
#(
  parameter DEPTH = 8,
  parameter DATA_WIDTH = 8
)
(
  input reset,

  input write_clk,
  input write,
  input [DATA_WIDTH-1:0] write_data,

  input read_clk,
  input read,
  output [DATA_WIDTH-1:0] read_data,

  output dataAvailable
);

  `define BIT_WIDTH $clog2(DEPTH)-1
  
  localparam ONE = `BIT_WIDTH'h01;

  reg [`BIT_WIDTH:0] write_address;
  reg [`BIT_WIDTH:0] read_address;

  assign dataAvailable = write_address != read_address;
  
  BlockMemoryDualPort #(
    .DEPTH(DEPTH),
    .DATA_WIDTH(DATA_WIDTH)
  )  memory ( 
    .reset(reset),
    // Write Port
    .p2_clk(write_clk),
    .p2_write_data(write_data),
    .p2_write(write),
    .p2_address(write_address),

    // Read Port
    .p1_clk(read_clk),
    .p1_read(read),
    .p1_write(1'b0),
    .p1_write_data(8'h00),
    .p1_read_data(read_data),
    .p1_address(read_address)
  );

  initial begin
    write_address = 0;
    read_address = 0;
  end

  // First port
  always @(posedge write_clk) begin
    if(write) 
      write_address <= write_address + 1;
  end

  // Second port
  always @(posedge read_clk) begin
    if(read) 
      read_address <= read_address + 1;
  end


endmodule
