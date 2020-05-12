
// LSB - no parity
module uart_tx  
#(
  parameter CLKS_PER_BIT = 139,
  parameter WORDBITS = 8,
  parameter STOPBITS = 1,
  parameter INPUT_BUFFER_DEPTH = 16,
  parameter INPUT_BITS=8
)
(
  input clock,
  input [INPUT_BITS-1:0] txIn,
  input send,
  output txOut
);

  localparam LISTEN     = 4'd0;
  localparam STARTBIT   = 4'd1; 
  localparam DATA       = 4'd2;
  localparam STOPBIT    = 4'd3;
  localparam RESTART    = 4'd4;

  reg [$clog2(INPUT_BUFFER_DEPTH)-1:0] read_addr;
  reg [$clog2(INPUT_BUFFER_DEPTH)-1:0] write_addr;
  reg [7:0] inputBuffer [INPUT_BUFFER_DEPTH-1:0]; 

  reg [3:0] state;
  reg [7:0] data;
  reg [4:0] bitCount;
  reg [$clog2(CLKS_PER_BIT)-1:0] timerCount;
  reg outputBit;

  assign txOut = outputBit;

  initial begin
    state <= LISTEN;
    outputBit <= 1'b1;
    write_addr <= 0;
    read_addr <= 0;
  end

  integer i;
  integer b;
  always @(posedge send) begin
    b=INPUT_BITS-1;
    for(i=0; i < INPUT_BITS / 8; i=i+1) begin

      inputBuffer[write_addr+i] <= txIn[b -:8];
      b = b - 8;
    end
    write_addr <= write_addr + (INPUT_BITS / 8);
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
      
      if( read_addr != write_addr ) begin
        data <= inputBuffer[read_addr];
        read_addr <= read_addr + 1;
        state <= STARTBIT;
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
      end
    endcase
  end

endmodule
