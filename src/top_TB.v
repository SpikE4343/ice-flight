//-------------------------------------------------
// File: cnt16_tb.v
// Purpose: Verilog Simulation Example
// Test Bench
//-----------------------------------------------------------
`timescale 1 ns / 1 ps
`define gyro_test_data 1

module flight__tb;
 //---------------------------------------------------------
 // inputs to the DUT are reg type
 //localparam FILESIZE=8536;
 //localparam FILESIZE=1024;
 localparam FILESIZE=1040;
 //localparam FILESIZE=76240;
 //localparam FILESIZE=10208;
 //localparam FILESIZE=1396;
 //localparam FILESIZE=16'd15888;
 //localparam FILESIZE=30;



 //---------------------------------------------------------
 // inputs to the DUT are reg type
 reg clk_50;
 reg rst;
 //--------------------------------------------------------
 // outputs from the DUT are wire type
 reg [15:0] dataIndex;
 reg [7:0] msgData[0:FILESIZE-1];
 reg dataReady;
 
 localparam BASE_FREQ = 10_000_000;
 
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
  wire rx_send_done;

  uart_tx #(
    .CLKS_PER_BIT(BASE_FREQ/115200)
  )rc_tx (
    .clock(clk_50),
    .send(dataReady),
    .txIn(rxData),
    .txOut(rcRxIn),
    .sendComplete(rx_send_done)
    );

  wire debug_signal;
  wire [7:0] debug_byte;
  wire debug_data_ready;
  uart_rx #(
    .CLKS_PER_BIT(BASE_FREQ/400000)
  ) debug_rx (
    .clock(clk_50),
    .rxIn(debug_signal),
    .rxDataReady(debug_data_ready),
    .rxData(debug_byte)
    );

 flight #(
    .BASE_FREQ(BASE_FREQ)
 ) dut (
   .CLK(clk_50),
   .RX_IN(~rcRxIn),
   
   .MOTOR_1(motor1),
   .MOTOR_2(motor2),
   .MOTOR_3(motor3),
   .MOTOR_4(motor4),

   .IMU_SCLK(SCLK),
   .IMU_MOSI(MOSI),
   .IMU_MISO(MISO),
   .IMU_CS(CS),

   .DEBUG_UART_TX(debug_signal)
    
 );

  assign MISO = 0; 
 always
  #1 clk_50 = ~clk_50;
  
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
    //$readmemh("../captures/rxsr-throttle-1408-hr.txt", msgData);
    //$readmemh("rxsr-throttle-1408-hr.txt", msgData);
    $readmemh("../captures/rxsr-throttle-1040.txt", msgData);
    
    clk_50 = 1'b0;
    while (dataIndex < FILESIZE) begin
      
      //#78000 rxData = msgData[dataIndex];
      #100 rxData = msgData[dataIndex];
       dataIndex = dataIndex + 1;
       dataReady = 1;
           
       #5 while(rx_send_done != 0) 
        #1 dataReady = 0;
        
        // started sending
       while(rx_send_done == 0) 
        #1 dataReady = 0;
        
      
     

    end
    // at time 0
    
    //#100_000_000 $finish;
    $finish;
    // call the load_count task
  end


endmodule
