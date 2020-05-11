// un buffered uart transceiver

module uart
#(
  parameter CLKS_PER_BIT = 139,
  parameter WORDBITS = 8,
  parameter STOPBITS = 1,
  parameter INPUT_BITS=8
)
(
  input       clock,
  input       rxIn,
  input [INPUT_BITS-1:0] txIn,
  input       txSend,

  output rxDataReady,
  output [7:0] rxData,
  output txOut
);

 //---------------------------------------------------------
  uart_tx #(
    .CLKS_PER_BIT(CLKS_PER_BIT),
    .WORDBITS(WORDBITS),
    .STOPBITS(STOPBITS),
    .INPUT_BITS(INPUT_BITS)
    ) tx (
    .clock(clock),
    .txIn(txIn),
    .send(txSend),
    .txOut(txOut)  
    );

 //---------------------------------------------------------
  uart_rx #(
    .CLKS_PER_BIT(CLKS_PER_BIT),
    .WORDBITS(WORDBITS),
    .STOPBITS(STOPBITS)
    ) rx (
    .clock(clock),
    .rxIn(rxIn),
    .rxDataReady(rxDataReady),
    .rxData(rxData)
  );
 
endmodule
