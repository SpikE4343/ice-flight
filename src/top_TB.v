//-------------------------------------------------
// File: cnt16_tb.v
// Purpose: Verilog Simulation Example
// Test Bench
//-----------------------------------------------------------
`timescale 1 ns / 10 ps


module flight__tb;
 //---------------------------------------------------------
 // inputs to the DUT are reg type
 //localparam FILESIZE=8536;
 //localparam FILESIZE=1024;
 //localparam FILESIZE=1040;
 //localparam FILESIZE=76240;
 //localparam FILESIZE=10208;
 localparam FILESIZE=1408;
 //localparam FILESIZE=30;

 //---------------------------------------------------------
 // inputs to the DUT are reg type
 reg clk_50;
 reg rst;
 //--------------------------------------------------------
 // outputs from the DUT are wire type
 reg [16:0] dataIndex;
 reg [7:0] msgData[0:FILESIZE-1];
 reg dataReady;
 
// Gyro SPI
  wire MISO;
  wire MOSI;
  wire SCLK;
  wire CS;


  wire rcRxIn;
  reg [7:0] rxData;

    // Motor outputs
  wire motor1;
  wire motor2;
  wire motor3;
  wire motor4;

  uart_tx rc_tx (
    .clock(clk_50),
    .send(dataReady),
    .txIn(rxData),
    .txOut(rcRxIn)
    );


 top #(
   .FIXED_WIDTH_BIT(`FIXED_WIDTH_BIT)
 ) dut (
   .CLK(clk_50),
   .PIN_3(~rcRxIn),
   
   .PIN_21(motor1),
   .PIN_22(motor2),
   .PIN_23(motor3),
   .PIN_24(motor4),

   .PIN_4(SCLK),
   .PIN_5(MOSI),
   .PIN_6(MISO),
   .PIN_7(CS)
    
 );

  assign MISO = ~MOSI; 
 always
  #10 clk_50 = ~clk_50;
  
  integer idx;
  initial
  begin

    $dumpfile("top_TB.vcd");
    $dumpvars();
    

    // for (idx = 0; idx < 32; idx = idx + 1) 
    //   $dumpvars(0, dut.fportfport_decode.controlPacket[idx]);

    clk_50 = 1'b0;


    dataIndex = 0;
    dataReady = 0;
    
    

    //$readmemh("rxsr-fport-data-1024.txt", msgData);
    //$readmemh("rxsr-throttle-10208.txt", msgData);
    //$readmemh("rxsr-throttle.txt", msgData);
    //$readmemh("rxsr-throttle-1040.txt", msgData);
    $readmemh("../captures/rxsr-throttle-1408-hr.txt", msgData);

    clk_50 = 1'b0;
    while (dataIndex < FILESIZE) begin
      
      #78000 rxData = msgData[dataIndex];
           dataIndex = dataIndex + 1;
           dataReady = 1;
      #1   dataReady = 0;

    end
    // at time 0
    
    #16000000 $finish;
    // call the load_count task
  end


endmodule
