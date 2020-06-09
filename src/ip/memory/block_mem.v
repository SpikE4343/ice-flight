
module block_mem
#(
  parameter CLKS_PER_BIT = 139,
  parameter WORDBITS = 8,
  parameter STOPBITS = 1
)
(
  input clock,
  input rxIn,
  output reg rxDataReady,
  output reg [7:0] rxData
);

  localparam LISTEN     = 4'd0;
  localparam STARTBIT   = 4'd1; 
  localparam DATA       = 4'd2;
  localparam STOPBIT    = 4'd3;
  localparam RESTART    = 4'd4;

  (* mark_debug = "true" *) reg [3:0] state;
  reg [7:0] data;
  reg [4:0] bitCount;
  reg [$clog2(CLKS_PER_BIT)-1:0] timerCount;

  initial begin
    state <= LISTEN;
    rxDataReady <= 0;
    rxData <= 0;
  end

  always @(posedge clock) begin
    case(state)

    // ======================================
    // wait for input signal to drop
    LISTEN: begin
      timerCount <= 0;
      bitCount <= 0;
      data <= 0;
      rxDataReady <= 0;
      //rxData <= 0;

      if( rxIn == 1'b0 ) 
        state <= STARTBIT;
    end
    
    // ======================================
    // wait for start bit to complete
    STARTBIT: begin
      if( timerCount < (CLKS_PER_BIT-1)) begin
        // wait until middle of signal 
        timerCount <= timerCount + 1;
        if(rxIn != 1'b0 && timerCount == (CLKS_PER_BIT-1)/2)
          state <= LISTEN;
      end else begin
        // signal is set so we have data
        timerCount <= 0;
        bitCount <= WORDBITS-1;
        rxData <= 0;
        state <= DATA;
      end 
    end

    // ======================================
    // clock in each bit
    DATA:  begin
      // wait until middle of bit
      if(timerCount < CLKS_PER_BIT-1) begin
        timerCount <= timerCount + 1;
      
        if(timerCount == (CLKS_PER_BIT-1)/2) begin
          rxData[7 - bitCount] <= rxIn;
        end
      end else begin
        // middle of bit. Sample input
        timerCount <= 0;
        
        if( bitCount > 0 ) begin
          // sample data
          // more bits to read
          bitCount <= bitCount - 1;
        end else begin
          // all bits read, wait for stop bit
          bitCount <= 0;
          state <= STOPBIT;
        end
      end
    end

    // ======================================
    // clock in each bit
    STOPBIT:  begin
      // wait until middle of bit
      if(timerCount < CLKS_PER_BIT-1) begin
        timerCount <= timerCount + 1;
      end else begin
        // middle of bit. Sample input
        timerCount <= 0;        
        rxDataReady <= 1;
        state <= RESTART;
      end
    end

    // ======================================
    // reset data ready
    RESTART: begin
      state <= LISTEN;
      //rxDataReady <= 0;
      end
    endcase
  end

endmodule
