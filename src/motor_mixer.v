//`include "debugger_defines.vh"

module motor_mixer 
#(
  parameter MOTOR_INDEX=0,
  parameter ROLL_MIX=0,
  parameter PITCH_MIX=0,
  parameter YAW_MIX=0,
  parameter THROTTLE_MIX=0
)(  
  input armed,
  input failsafe,
  input signed [31:0] inputs_roll,
  input signed [31:0] inputs_pitch,
  input signed [31:0] inputs_yaw,
  input signed [31:0] inputs_throttle,
  
  output signed [31:0] mixedThrottle
);

  localparam INPUT_THROTTLE = 0;
  localparam INPUT_ROLL = 1;
  localparam INPUT_PITCH = 2;
  localparam INPUT_YAW = 3;
  localparam INPUT_ARM = 4;

  wire signed [31:0] mixed[3:0];
  wire signed [31:0] mixedSum;
  wire signed [31:0] clamped;
  reg signed [31:0] motorThrottle;

  
  // Motor index layout, up is forward
  //  3 ^ 1
  //    X
  //  2   0

  // Roll
  axis_scalar #(
    .AXIS_INDEX(INPUT_ROLL),
    .MIX_SCALAR(ROLL_MIX)
  ) rollScalar (
    .inputs(inputs_roll),
    .mixedValue(mixed[INPUT_ROLL])
  );

  // Pitch
  axis_scalar #(
    .AXIS_INDEX(INPUT_PITCH),
    .MIX_SCALAR(PITCH_MIX)
  ) pitchScalar (
    .inputs(inputs_pitch),
    .mixedValue(mixed[INPUT_PITCH])
  );

  // Yaw
  axis_scalar #(
    .AXIS_INDEX(INPUT_YAW),
    .MIX_SCALAR(YAW_MIX)
  ) yawScalar (
    .inputs(inputs_yaw),
    .mixedValue(mixed[INPUT_YAW])
  );

  // Throttle
  axis_scalar #(
    .AXIS_INDEX(INPUT_THROTTLE),
    .MIX_SCALAR(THROTTLE_MIX)
  ) throttleScalar (
    .inputs(inputs_throttle),
    .mixedValue(mixed[INPUT_THROTTLE])
  );

   assign mixedSum = mixed[INPUT_ROLL] 
                   +  mixed[INPUT_PITCH] 
                   +  mixed[INPUT_YAW] 
                   +  mixed[INPUT_THROTTLE];

  // ((mixedSum + 1 ) / 2) 
  assign mixedThrottle = (armed && ~failsafe) ? mixedSum : 32'sd0;

  
endmodule


module axis_scalar 
#(
  parameter AXIS_INDEX=0,
  parameter MIX_SCALAR=32'd0
)(
  input signed [31:0] inputs,
  output signed [31:0] mixedValue
);
  wire signed [63:0] temp;

  assign temp = (MIX_SCALAR) * inputs;
  assign mixedValue = (temp >>> 28);
endmodule