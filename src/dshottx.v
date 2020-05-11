
// Combinational module to build dshot 16-bit packet from
// command, telemetryReq
module DShotPacketEncoder (
  input [10:0] command,
  input telemetryReq,
  output [15:0] packet
);

wire [11:0] data = { command, telemetryReq };
wire [3:0] crc = data[11:8] ^ data[7:4] ^ data[3:0];

assign packet = {data, crc};  

endmodule

// #define MOTOR_BIT_0           7
// #define MOTOR_BIT_1           14



module DShotTx
#(
  parameter CLKS_PER_BIT = 16
)   
(
  input clock,
  input [15:0] command,
  input load,
  output outValue
);

  reg [7:0] alarm;
  reg [4:0] bitsCount;
  reg [7:0] timerCount;
  reg [15:0] data;
  reg outputBit;

  reg state;

  //assign sending = state;
  assign outValue = outputBit;

  always @(posedge clock) begin
    if( load ) begin
      state <= 1;
      bitsCount <= 5'd16;
      timerCount <= 0;
      data <= command;
    end

    if(state) begin
      // end/start of single bit timer
      //$display("sending, ", sending);
      if(timerCount == 0) begin
        if( bitsCount == 0) begin
          // both single bit and all bits counters are zero 
          // so transmission is complete
          state <= 0;
        end else begin
          // set output high time
          alarm <= data[bitsCount-1] ? (CLKS_PER_BIT / 3) : (2 *CLKS_PER_BIT / 3);
          timerCount <= CLKS_PER_BIT;
          bitsCount <= bitsCount-1;
          state <= 1;
        end
      end else begin
        // sending bit
        timerCount <= timerCount -1;
        state <= 1;
      end

      outputBit <= state && (timerCount > alarm); 
    end else begin
      outputBit <= 0;
    end
  end

endmodule
