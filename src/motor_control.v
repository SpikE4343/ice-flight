
module motor_control #(
  parameter BASE_FREQ=16000000,
  parameter DSHOT_FREQ=600000
)
(
    input clock,    
    input [10:0] command,
    input send,
    output txOut
);
    wire [15:0] packet;
    
    DShotPacketEncoder packet_encoder (
      .command(command),
      .telemetryReq(1'b0),
      .packet(packet)
    );

    DShotTx #(
      .CLKS_PER_BIT(BASE_FREQ/DSHOT_FREQ)
    ) dshot_tx (
      .clock (clock),
      .command(packet),
      .load(send),
      .outValue(txOut)
    );
endmodule



