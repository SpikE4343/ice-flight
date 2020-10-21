`define MSG_MARKER 8'h7f

`define MSG_TYPE_TELEMETRY_SETPOINT     8'd6
`define MSG_LEN_TELEMETRY_SETPOINT      8'd20

`define MSG_TYPE_TELEMETRY_STATUS       8'd5
`define MSG_LEN_TELEMETRY_STATUS        8'd7

`define MSG_TYPE_TELEMETRY_GYRO         8'd4
`define MSG_LEN_TELEMETRY_GYRO          8'd12

`define MSG_TYPE_TELEMETRY_RX           8'd3
`define MSG_LEN_TELEMETRY_RX            8'd12

`define MSG_TYPE_TELEMETRY_MOTOR        8'd2
`define MSG_LEN_TELEMETRY_MOTOR         8'd16

`define MSG_TYPE_TELEMETRY_ATTITUDE     8'd1
`define MSG_LEN_TELEMETRY_ATTITUDE      8'd20


`define MSG_TELEMETRY_COUNT 8'd06

`define MSG_BUFFER_SIZE 2048