// spi master tx/rx


module spi_master  
#(
  parameter CLKS_PER_BIT = 16,
  parameter WORDBITS = 8,
  parameter BUFFER_DEPTH = 8
)
(
  input clock,
  input [WORDBITS-1:0] txData,
  input send,
  input inMISO,
  output reg [WORDBITS-1:0] rxData,
  output reg outCLK,
  output reg outMOSI,
  output reg outCS,
  output reg complete
);

  localparam LISTEN       = 4'd0;
  localparam STARTBIT     = 4'd1; 
  localparam DATA_TX_BITS = 4'd2;
  localparam STOPBIT      = 4'd3;
  localparam RESTART      = 4'd4;


  reg [3:0] state;
  reg [WORDBITS-1:0] data;
  reg [$clog2(WORDBITS)-1:0] bitCount;
  reg [15:0] timerCount;

  initial begin
    state <= LISTEN;
    complete <= 1'b0;

    // TODO: polarity parameter
    outMOSI <= 1'b0;
  end

  always @(posedge clock) begin
    case(state)

    // ======================================
    // wait for send bit set
    // ______
    //|
    // _   _
    //| |_| ....
    LISTEN: begin
      timerCount <= 0;
      bitCount <= (WORDBITS-1);
      data <= 1'b0;
      outMOSI <= 1'b0;
      outCS <= 1'b1;
      outCLK <= 1'b1;
      
      if( send == 1'b1 ) begin
        data <= txData;
        complete <= 1'b0; 
        state <= STARTBIT;
      end
    end
    
    // ======================================
    // send start bit of 0
    STARTBIT: begin
      outMOSI <= 1'b0;
      outCS <= 1'b0;

      if( timerCount < CLKS_PER_BIT-1) begin
        // wait until middle of signal 
        timerCount <= timerCount + 1;
      end else begin
        // signal is set so we have data
        timerCount <= 0;
        state <= DATA_TX_BITS;
      end
    end

    // ======================================
    // clock out each bit
    DATA_TX_BITS:  begin
      outMOSI <= data[bitCount];

      if(timerCount < CLKS_PER_BIT/2) 
      begin
        timerCount <= timerCount + 1;
        outCLK <= 1'b0;
      end else if( timerCount < CLKS_PER_BIT-1) 
      begin
        outCLK <= 1'b1;
        if(timerCount == CLKS_PER_BIT/2) begin
          rxData[bitCount] <= inMISO;
        end
        timerCount <= timerCount + 1;
      end else begin
        timerCount <= 0;
        if( bitCount > 0 ) begin
          // more bits to read
          bitCount <= bitCount - 1;
        end else begin
          // all bits read, send stop bit
          bitCount <= 0;
          state <= STOPBIT;
        end
      end
    end

    // ======================================
    // clock out stop bit (1)
    STOPBIT:  begin
      outMOSI <= 1'b0;
      outCLK <= 1'b1;
      

      // wait until middle of bit
      if(timerCount < CLKS_PER_BIT-1) begin
        timerCount <= timerCount + 1;
      end else begin
        // middle of bit. Sample input
        timerCount <= 0;
        complete <= 1;
        outCS <= 1'b1;
        state <= RESTART;
      end
    end

    // ======================================
    // reset data ready
    RESTART: begin
      if( send == 0 ) state <= LISTEN;
      complete <= 0;
      outMOSI <= 1'b0;

      end
    endcase
  end

endmodule
