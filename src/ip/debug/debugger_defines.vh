`define MSG_MARKER 8'h7f

`define MSG_TYPE_TELEMETRY_STATUS       8'd5
`define MSG_LEN_TELEMETRY_STATUS        8'd6

`define MSG_TYPE_TELEMETRY_GYRO         8'd4
`define MSG_LEN_TELEMETRY_GYRO          8'd6

`define MSG_TYPE_TELEMETRY_RX           8'd3
`define MSG_LEN_TELEMETRY_RX            8'd12

`define MSG_TYPE_TELEMETRY_MOTOR        8'd2
`define MSG_LEN_TELEMETRY_MOTOR         8'd16

`define MSG_TYPE_TELEMETRY_ATTITUDE     8'sd1
`define MSG_LEN_TELEMETRY_ATTITUDE      8'd10

`define MSG_TELEMETRY_COUNT 8'h04