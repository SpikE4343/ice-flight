
`timescale 1 ns / 1 ps

module BlockMemorySinglePort_TestBench;

  localparam BASE_FREQ = 10_000_000;

 reg clk;
 reg reset;
 
 reg write;
 reg read;
 reg [7:0] mem_address;
 wire [7:0] mem_read_data;
 reg [7:0] mem_write_data;

 
 //assign mem_read_data_wire = mem_read_data;

 BlockMemorySinglePort #(

 ) block_mem (
   .clk(clk),
   .reset(reset),
   .write(write),
   //.read(read),
   .address(mem_address),
   .data(mem_read_data),
   .value(mem_write_data)
 );

 always
  #1 clk = ~clk;
  
  integer idx;
  initial
  begin

    $dumpfile("BlockMemorySinglePort_TestBench.vcd");
    $dumpvars();

    clk = 1'b0;
    
    for(idx=0; idx <= 8'd255; idx = idx + 8'd1) begin
      #1 mem_address = idx;
      mem_write_data = idx;
      write = 1;
      #1 write = 0;
    end
    
    #10
    
    for(idx=0; idx <= 8'd255; idx = idx + 8'd1) begin
      #1 mem_address = idx;
      read = 1;
      #1 read = 0; 
      #1 if(mem_read_data != idx ) begin
        $display("Invalid mem block", idx, mem_read_data );
      end
    end
 
    
    $finish;
  end


endmodule
