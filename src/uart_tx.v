
// LSB - no parity
module uart_tx  
#(
  parameter CLKS_PER_BIT = 139,
  parameter WORDBITS = 8,
  parameter STOPBITS = 1
)
(
  input clock,
  input [7:0] txIn,
  input send,
  output sendComplete,
  output txOut
);

  localparam LISTEN     = 4'd0;
  localparam STARTBIT   = 4'd1; 
  localparam DATA       = 4'd2;
  localparam STOPBIT    = 4'd3;
  localparam RESTART    = 4'd4;

  reg sending;
  reg [3:0] state;
  reg [7:0] data;
  reg [4:0] bitCount;
  reg [$clog2(CLKS_PER_BIT)-1:0] timerCount;
  reg outputBit;
  reg sendingComplete;

  assign txOut = outputBit;
  assign sendComplete = sendingComplete;

  initial begin
    state <= LISTEN;
    outputBit <= 1'b1;
    sending <= 0;
    data <= 8'h00;
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
      data <= 0;
      outputBit <= 1'b1;
      sendingComplete <= 0;

      if( send && !sending) begin
        data <= txIn[7 -:8];
        state <= STARTBIT;
        sending <= 1;
      end
    end
    
    // ======================================
    // send start bit of 0
    STARTBIT: begin
      outputBit <= 1'b0;

      if( timerCount < CLKS_PER_BIT-1) begin
        // wait until middle of signal 
        timerCount <= timerCount + 1;
      end else begin
        // signal is set so we have data
        timerCount <= 0;
        state <= DATA;
      end
    end

    // ======================================
    // clock out each bit
    DATA:  begin
      outputBit <= data[(WORDBITS-1) - bitCount];
      // wait until middle of bit
      if(timerCount < CLKS_PER_BIT-1) begin
        timerCount <= timerCount + 1;
      end else begin
        // middle of bit. Sample input
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
      outputBit <= 1'b1;
      // wait until middle of bit
      if(timerCount < CLKS_PER_BIT-1) begin
        timerCount <= timerCount + 1;
      end else begin
        // middle of bit. Sample input
        timerCount <= 0;
        state <= RESTART;
      end
    end

    // ======================================
    // reset data ready
    RESTART: begin
      if( send == 0 ) state <= LISTEN;
      outputBit <= 1'b1;
      sending <= 0;
      sendingComplete <= 1;
      end
    endcase
  end

endmodule
