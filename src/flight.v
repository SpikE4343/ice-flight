`define SBUS_INPUT_MIN 192
`define SBUS_INPUT_MID 992
`define SBUS_INPUT_MAX 1792
`define SBUS_HALF_RANGE 792
`define SBUS_INPUT_RANGE `SBUS_INPUT_MAX - `SBUS_INPUT_MIN

`include "debugger_defines.vh"

module flight 
#(
  parameter BASE_FREQ=16_000_000,
  
  parameter FPORT_UART_BAUD=115200,
  
  parameter MOTOR_UPDATE_HZ=1000,
  
  parameter DSHOT_150_FREQ=150_000,
  parameter DSHOT_300_FREQ=300_000,
  parameter DSHOT_600_FREQ=600_000,

  parameter GYRO_UPDATE_HZ=32_000,
  parameter GYRO_SPI_REG_FREQ=1_000_000,
  parameter GYRO_SPI_UPDATE_FREQ=4_000_000,
  
  parameter FIXED_WIDTH_BIT=31,
  parameter DEBUG_UART_BAUD=2_000_000,
  parameter DEBUG_UPDATE_HZ=100
)(
    input CLK,    // 16MHz clock

    // RC Control input
    input RX_IN,
    
    // Gyro SPI master
    output IMU_SCLK, // SCLK
    output IMU_MOSI, // MOSI
    input  IMU_MISO, // MISO
    output IMU_CS, // CS

    // Indicator LEDs
    output LED_STATUS,
    output LED_ARMED,

    // Debug UART pins
    output DEBUG_UART_TX,
    input DEBUG_UART_RX,

    // Motor outputs
    output MOTOR_1,
    output MOTOR_2,
    output MOTOR_3,
    output MOTOR_4,
    
    input btn0,
    input btn1
);

  localparam MOTOR_DSHOT_FREQ = DSHOT_600_FREQ;
  localparam UPDATE_CLK_TICKS = BASE_FREQ / MOTOR_UPDATE_HZ;
  localparam DEBUG_SEND_CLK_TICKS = MOTOR_UPDATE_HZ / DEBUG_UPDATE_HZ;
  localparam INPUT_THROTTLE = 0;
  localparam INPUT_ROLL = 1;
  localparam INPUT_PITCH = 2;
  localparam INPUT_YAW = 3;
  localparam INPUT_ARM = 4;

  wire reset;
  wire clk;
  
  // Motors
  wire motorOutputs[3:0];
  reg motor_send;
  
  reg signed [31:0] motorThrottle[3:0];

  wire [10:0] controlInputs[4:0];

  reg signed [31:0] setpoint[4:0];
  reg signed [31:0] inputs[4:0];
  reg signed [31:0] attitude[4:0];

  // First index should be motor count define
  reg signed [31:0] motorMix[3:0][3:0];

  reg signed [31:0] pid_gains[3:0][3:0];


  wire signed [15:0] gyro_rates_raw[2:0];
  reg signed [31:0] gyro_rates_sampled[2:0];

  // RX Controls
  wire [7:0] rx_data_byte;
  wire rx_data_ready;
  wire controls_ready;
  
  wire armed;
  wire failsafe;
  wire rxFrameLoss;
  wire [7:0] rssi;

  // Gyro SPI
  wire gyroSampleReady;

  wire w_debug_msg_send;
  wire [7:0] w_debug_msg_type;
  wire [7:0] w_debug_msg_length;
  wire [7:0] w_debug_msg_data;
  wire [7:0] w_debug_msg_data_index;
  wire w_debug_msg_data_load;

  wire w_debug_send_complete;

  wire [7:0] w_debug_uart_byte;
  wire w_debug_uart_load;
  wire w_debug_uart_complete;

  reg debug_uart_load;
  reg debug_msg_send;
  reg [7:0] debug_msg_type;
  reg [7:0] debug_msg_length;
  reg [7:0] debug_msg_data;

  reg [7:0] attitudeUpdateCounter; 

  assign w_debug_msg_send   = debug_msg_send;
  assign w_debug_msg_type   = debug_msg_type;
  assign w_debug_msg_length = debug_msg_length;
  assign w_debug_msg_data   = debug_msg_data;

  localparam DEBUG_ST_RESET  = 0;
  localparam DEBUG_ST_WAIT   = 1;
  localparam DEBUG_ST_CHECK  = 2;
  localparam DEBUG_ST_SEND   = 3;
  localparam DEBUG_ST_LOAD   = 4;

  localparam DEBUG_SEND_STATUS   = 5;
  localparam DEBUG_SEND_GYRO     = 4;
  localparam DEBUG_SEND_RX       = 3;
  localparam DEBUG_SEND_MOTOR    = 2;
  localparam DEBUG_SEND_ATTITUDE = 1;
  localparam DEBUG_SEND_COMPLETE = 0;

  localparam P_TERM = 0;
  localparam I_TERM = 1;
  localparam D_TERM = 2;
  localparam F_TERM = 3;

  reg [4:0] debugState;
  reg [4:0] debugSendState;
  reg [31:0] debugUpdateCount;
  
  reg [7:0] fc_state;
  reg update_attitude;
  reg change_update_state;


  //   static uint16_t sbusChannelsReadRawRC(const rxRuntimeState_t *rxRuntimeState, uint8_t chan)
  // {
      // Linear fitting values read from OpenTX-ppmus and comparing with values received by X4R
      // http://www.wolframalpha.com/input/?i=linear+fit+%7B173%2C+988%7D%2C+%7B1812%2C+2012%7D%2C+%7B993%2C+1500%7D
  //     return (5 * rxRuntimeState->channelData[chan] / 8) + 880;
  //}

  // sbus/fport rc range is from 192 - 1792 
  // output range is -2000 - 2000
  function automatic signed [31:0] RxRangeInput;
    input [10:0] inputValue;
    begin
      RxRangeInput = ((32'd5 * inputValue >>> 3) - 32'd620) <<< 19;
    end
  endfunction

  function automatic signed [31:0] MotorThrottleValue;
    input [3:0] index;
    reg signed [31:0] throttle;
    reg signed [63:0] temp;
    begin
      throttle = 32'd0;
      if( armed ) begin
        temp = (motorMix[index][INPUT_ROLL]) * inputs[INPUT_ROLL];
        throttle = throttle + (temp >>> 28);

        temp = (motorMix[index][INPUT_PITCH]) * inputs[INPUT_PITCH];
        throttle = throttle + (temp >>> 28);

        temp = (motorMix[index][INPUT_YAW]) * inputs[INPUT_YAW];
        throttle = throttle + (temp >>> 28);

        temp = (motorMix[index][INPUT_THROTTLE]) * inputs[INPUT_THROTTLE];
        throttle = throttle + (temp >>> 28);
        
        // (throttle + 1) / 2
        throttle = (throttle + 31'h10_00_00_00) >> 1;


        
        // temp = throttle <<< 11;
        // throttle = throttle >>> 28;
        // 28 - 11 = 17
        throttle = throttle >>> 17;

      end else begin
        throttle = 0;
      end

      MotorThrottleValue = throttle;
    end
  endfunction

  function automatic signed [31:0] PID;
    input [3:0] index;
    reg signed [31:0] error;
    reg signed [63:0] mul;
    begin
      error = gyro_rates_sampled[index-1] - setpoint[index];
      mul = error * pid_gains[index][P_TERM];
      attitude[index] =  mul >>> 28;
      inputs[index] = attitude[index];
      PID = inputs[index];
    end
  endfunction
  
  function automatic [3:0] MaxThrottle;
    input v;
    reg [3:0] a;
    reg [3:0] b;
    begin
      a = (motorThrottle[0] > motorThrottle[1]) ? 0 : 1;
      b = motorThrottle[2] > motorThrottle[3] ? 2 : 3;
      MaxThrottle = motorThrottle[a] > motorThrottle[b] ? a : b;
    end
  endfunction

  function automatic [3:0] MinThrottle;
    input v;
    reg [3:0] a;
    reg [3:0] b;
    begin
      a = (motorThrottle[0] < motorThrottle[1]) ? 0 : 1;
      b = motorThrottle[2] < motorThrottle[3] ? 2 : 3;
      MinThrottle = motorThrottle[a] < motorThrottle[b] ? a : b;
    end
  endfunction
  
  // Motor index layout, up is forward
  //  3 ^ 1
  //    X
  //  2   0

  initial begin
    motorMix[0][INPUT_THROTTLE] = 1 <<< 28;
    motorMix[0][INPUT_ROLL] = 1 <<< 28;
    motorMix[0][INPUT_PITCH] = -1 <<< 28;
    motorMix[0][INPUT_YAW] = -1 <<< 28;

    motorMix[1][INPUT_THROTTLE] = 1 <<< 28;
    motorMix[1][INPUT_ROLL] = 1 <<< 28;
    motorMix[1][INPUT_PITCH] = 1 <<< 28;
    motorMix[1][INPUT_YAW] = 1 <<< 28;

    motorMix[2][INPUT_THROTTLE] = 1 <<< 28;
    motorMix[2][INPUT_ROLL] = -1 <<< 28;
    motorMix[2][INPUT_PITCH]= -1 <<< 28;
    motorMix[2][INPUT_YAW] =1 <<< 28;

    motorMix[3][INPUT_THROTTLE] = 1 <<< 28;
    motorMix[3][INPUT_ROLL] = -1 <<< 28;
    motorMix[3][INPUT_PITCH] = 1 <<< 28;
    motorMix[3][INPUT_YAW] = -1 <<< 28;

    pid_gains[INPUT_ROLL][P_TERM] = 1 <<< 28;
    pid_gains[INPUT_ROLL][I_TERM] = 0 <<< 28;
    pid_gains[INPUT_ROLL][D_TERM] = 0 <<< 28;
    pid_gains[INPUT_ROLL][F_TERM] = 0 <<< 28;

    pid_gains[INPUT_PITCH][P_TERM] = 1 <<< 28;
    pid_gains[INPUT_PITCH][I_TERM] = 0 <<< 28;
    pid_gains[INPUT_PITCH][D_TERM] = 0 <<< 28;
    pid_gains[INPUT_PITCH][F_TERM] = 0 <<< 28;

    pid_gains[INPUT_YAW][P_TERM] = 1 <<< 28;
    pid_gains[INPUT_YAW][I_TERM] = 0 <<< 28;
    pid_gains[INPUT_YAW][D_TERM] = 0 <<< 28;
    pid_gains[INPUT_YAW][F_TERM] = 0 <<< 28;

    debugState = DEBUG_ST_RESET;
    debugUpdateCount = DEBUG_SEND_CLK_TICKS;
    gyro_rates_sampled[0] = 16'h0;
    gyro_rates_sampled[1] = 16'h0;
    gyro_rates_sampled[2] = 16'h0;
    
    
    
    setpoint[INPUT_THROTTLE] = -32'd2000;
    setpoint[INPUT_ROLL] = 32'd0;
    setpoint[INPUT_PITCH] = 32'd0;
    setpoint[INPUT_YAW] = 32'd0;
    setpoint[4] = -32'd2000;

    inputs[0] = 32'h0;
    inputs[1] = 32'h0;
    inputs[2] = 32'h0;
    inputs[3] = 32'h0;
    inputs[4] = 32'h0;
    
    fc_state = ST_RESET;
    change_update_state = 1;
    update_attitude = 0;
    attitudeUpdateCounter = 8'h0;
  end

  // PINS
  assign clk = CLK;
  assign MOTOR_1 = motorOutputs[0];
  assign MOTOR_2 = motorOutputs[1];
  assign MOTOR_3 = motorOutputs[2];
  assign MOTOR_4 = motorOutputs[3];

  assign armed = 1;//controlInputs[0] > 0;
  assign LED_ARMED = controls_ready | rx_data_ready;
  assign LED_STATUS = failsafe | rxFrameLoss;
  
  assign reset = btn1;
  
  

  localparam ST_RESET = 0;
  localparam ST_WAIT = 1;
  localparam ST_UPDATE_ATTITUDE = 2;
  localparam ST_UPDATE_MOTOR_MIX = 3;
  localparam ST_MOTOR_SATURATION = 4;
  localparam ST_CLEAN_MOTOR_OUTPUTS = 5;
  localparam ST_APPLY_UPDATES = 6;
  

  // * Sample Gyro State
  always @(posedge gyroSampleReady) begin
    gyro_rates_sampled[0] <= gyro_rates_raw[0];// + gyro_rates_sampled[0]) >> 1;
    gyro_rates_sampled[1] <= gyro_rates_raw[1];// + gyro_rates_sampled[1]) >> 1;
    gyro_rates_sampled[2] <= gyro_rates_raw[2];// + gyro_rates_sampled[2]) >> 1;

    
    if(attitudeUpdateCounter == 8'h0) begin
      attitudeUpdateCounter <= (GYRO_UPDATE_HZ / MOTOR_UPDATE_HZ) -1;
      update_attitude <= ~update_attitude;
    end else begin
      attitudeUpdateCounter <= attitudeUpdateCounter - 1;
    end
  end

  // * Main Loop: process gyro, rx, and attitude data to update motor controls
  always @(posedge clk) begin
  
    if(btn0) begin 
        fc_state <= ST_RESET;
    end
  
    case(fc_state)

      ST_RESET: begin
        
        motor_send <= 0;
        fc_state <= ST_WAIT;
        motorThrottle[0] <= 0;
        motorThrottle[1] <= 0;
        motorThrottle[2] <= 0;
        motorThrottle[3] <= 0;
      end

      ST_WAIT: begin
        motor_send <= 0;
        if(change_update_state != update_attitude) begin
            change_update_state <= update_attitude;
            fc_state <= ST_UPDATE_ATTITUDE;
        end
      end

      ST_UPDATE_ATTITUDE: begin
        inputs[INPUT_ROLL]  <= PID(INPUT_ROLL);
        inputs[INPUT_PITCH] <= PID(INPUT_PITCH);
        inputs[INPUT_YAW]   <= PID(INPUT_YAW);
        inputs[INPUT_THROTTLE] <= setpoint[INPUT_THROTTLE];
        fc_state <= ST_UPDATE_MOTOR_MIX;
      end

      ST_UPDATE_MOTOR_MIX: begin
        motorThrottle[0] <= MotorThrottleValue(0);
        motorThrottle[1] <= MotorThrottleValue(1);
        motorThrottle[2] <= MotorThrottleValue(2);
        motorThrottle[3] <= MotorThrottleValue(3);
        
        fc_state <= ST_MOTOR_SATURATION;
      end

      ST_MOTOR_SATURATION: begin
          
        fc_state <= ST_CLEAN_MOTOR_OUTPUTS;
      end

      ST_CLEAN_MOTOR_OUTPUTS: begin
        // rescale saturated motors
        // motorThrottle[0] <= motorThrottle[0] < 32'sd0 ? 32'sd0 : (motorThrottle[0] > 32'sd2047 ? 32'sd2047 : motorThrottle[0]);
        // motorThrottle[1] <= motorThrottle[1] < 32'sd0 ? 32'sd0 : (motorThrottle[1] > 32'sd2047 ? 32'sd2047 : motorThrottle[1]);
        // motorThrottle[2] <= motorThrottle[2] < 32'sd0 ? 32'sd0 : (motorThrottle[2] > 32'sd2047 ? 32'sd2047 : motorThrottle[2]);
        // motorThrottle[3] <= motorThrottle[3] < 32'sd0 ? 32'sd0 : (motorThrottle[3] > 32'sd2047 ? 32'sd2047 : motorThrottle[3]);
      
        fc_state <= ST_APPLY_UPDATES;
      end

      ST_APPLY_UPDATES: begin
        motor_send <= 1;
        fc_state <= ST_WAIT;
        debugUpdateCount <= debugUpdateCount - 1;
        
        if( debugUpdateCount == 32'b0 && (debugState == DEBUG_ST_WAIT || debugState == DEBUG_ST_RESET))
            debugState <= DEBUG_ST_SEND;
      end
    endcase
    
     case(debugState)
      DEBUG_ST_RESET: begin
        debugState <= DEBUG_ST_WAIT;
        debugUpdateCount <= DEBUG_SEND_CLK_TICKS;
      end

      DEBUG_ST_WAIT: 
      begin
        debugSendState <= `MSG_TELEMETRY_COUNT;
      end

      DEBUG_ST_LOAD: 
      begin
        debug_msg_send <= 0;
        if( w_debug_send_complete) begin
          debugSendState <= debugSendState - 1;
          debugState <= DEBUG_ST_CHECK;
        end
      end

      DEBUG_ST_CHECK:
      begin
      if( debugSendState == 0)
        debugState <= DEBUG_ST_RESET;
      else 
        debugState <= DEBUG_ST_SEND;
      end

      DEBUG_ST_SEND: 
      begin
        case (debugSendState)
          DEBUG_SEND_GYRO: 
          begin
            debug_msg_type <= `MSG_TYPE_TELEMETRY_GYRO;
            debug_msg_length <= `MSG_LEN_TELEMETRY_GYRO;
          end

          DEBUG_SEND_RX: 
          begin
            debug_msg_type <= `MSG_TYPE_TELEMETRY_RX;
            debug_msg_length <= `MSG_LEN_TELEMETRY_RX;
          end

          DEBUG_SEND_MOTOR: 
          begin
            debug_msg_type <= `MSG_TYPE_TELEMETRY_MOTOR;
            debug_msg_length <= `MSG_LEN_TELEMETRY_MOTOR;
          end

          DEBUG_SEND_ATTITUDE: 
          begin
            debug_msg_type <= `MSG_TYPE_TELEMETRY_ATTITUDE;
            debug_msg_length <= `MSG_LEN_TELEMETRY_ATTITUDE;
          end

          DEBUG_SEND_STATUS: 
          begin
            debug_msg_type <= `MSG_TYPE_TELEMETRY_STATUS;
            debug_msg_length <= `MSG_LEN_TELEMETRY_STATUS;
          end
        endcase

        debug_msg_send <= 1;
        debugState <= DEBUG_ST_LOAD;
      end
    endcase
  end

  // * Update RX control inputs
  always @(posedge controls_ready) begin
    setpoint[INPUT_THROTTLE] <= RxRangeInput(controlInputs[INPUT_THROTTLE]);
    setpoint[INPUT_ROLL] <= -1 * RxRangeInput(controlInputs[INPUT_ROLL]);
    setpoint[INPUT_PITCH] <= -1 * RxRangeInput(controlInputs[INPUT_PITCH]);
    setpoint[INPUT_YAW] <= RxRangeInput(controlInputs[INPUT_YAW]);
  end

  // * Debug Protocol message data loader
  always @(posedge w_debug_msg_data_load) begin
    case (debugSendState)
       DEBUG_SEND_STATUS: 
      begin
        case(w_debug_msg_data_index)
          8'd0: debug_msg_data <= fc_state;
          8'd1: debug_msg_data <= armed;
          8'd2: debug_msg_data <= failsafe;
          8'd3: debug_msg_data <= rxFrameLoss;

        endcase
      end

      DEBUG_SEND_GYRO: 
      begin
        case(w_debug_msg_data_index)
          // * Roll
          8'd0: debug_msg_data <= gyro_rates_sampled[0][7:0];
          8'd1: debug_msg_data <= gyro_rates_sampled[0][15:8];

          // * Pitch
          8'd2: debug_msg_data <= gyro_rates_sampled[1][7:0];
          8'd3: debug_msg_data <= gyro_rates_sampled[1][15:8];

          // * Yaw
          8'd4: debug_msg_data <= gyro_rates_sampled[2][7:0];
          8'd5: debug_msg_data <= gyro_rates_sampled[2][15:8];

        endcase
      end

      DEBUG_SEND_RX: 
      begin
        case(w_debug_msg_data_index)
          // * Throttle
          8'd0: debug_msg_data <= controlInputs[0][7:0];
          8'd1: debug_msg_data <= { 5'h0, controlInputs[0][10:8]};

          // * Pitch
          8'd2: debug_msg_data <= controlInputs[1][7:0];
          8'd3: debug_msg_data <= {5'h0, controlInputs[1][10:8]};

          // * Yaw
          8'd4: debug_msg_data <= controlInputs[2][7:0];
          8'd5: debug_msg_data <= {5'h0, controlInputs[2][10:8]};

          // * Roll
          8'd6: debug_msg_data <= controlInputs[3][7:0];
          8'd7: debug_msg_data <= { 5'h0, controlInputs[3][10:8]};

          // * Aux1
          8'd8: debug_msg_data <= controlInputs[4][7:0];
          8'd9: debug_msg_data <= { 5'h0, controlInputs[4][10:8]};

          // * RSSI
          8'd10: debug_msg_data <= rssi;
          8'd11: debug_msg_data <= {7'd0, failsafe};

        endcase
      end

      DEBUG_SEND_MOTOR: 
      begin
        case(w_debug_msg_data_index)
          // * Motor1
          8'd0: debug_msg_data <= motorThrottle[0][7:0];
          8'd1: debug_msg_data <= motorThrottle[0][15:8];
          8'd2: debug_msg_data <= motorThrottle[0][23:16];
          8'd3: debug_msg_data <= motorThrottle[0][31:24];

          // * Motor 2
          8'd4: debug_msg_data <= motorThrottle[1][7:0];
          8'd5: debug_msg_data <= motorThrottle[1][15:8];
          8'd6: debug_msg_data <= motorThrottle[1][23:16];
          8'd7: debug_msg_data <= motorThrottle[1][31:24];

          // * Motor 3
          8'd8: debug_msg_data <= motorThrottle[2][7:0];
          8'd9: debug_msg_data <= motorThrottle[2][15:8];
          8'd10: debug_msg_data <= motorThrottle[2][23:16];
          8'd11: debug_msg_data <= motorThrottle[2][31:24];

          // * Motor 4
          8'd12: debug_msg_data <= motorThrottle[3][7:0];
          8'd13: debug_msg_data <= motorThrottle[3][15:8];
          8'd14: debug_msg_data <= motorThrottle[3][23:16];
          8'd15: debug_msg_data <= motorThrottle[3][31:24];   
        endcase
      end

      DEBUG_SEND_ATTITUDE: begin
        case(w_debug_msg_data_index)
          // * Control Inputs with rates applied
          8'd0: debug_msg_data <= inputs[0][7:0];
          8'd1: debug_msg_data <= inputs[0][15:8];

          8'd2: debug_msg_data <= inputs[1][7:0];
          8'd3: debug_msg_data <= inputs[1][15:8];

          8'd4: debug_msg_data <= inputs[2][7:0];
          8'd5: debug_msg_data <= inputs[2][15:8];

          8'd6: debug_msg_data <= inputs[3][7:0];
          8'd7: debug_msg_data <= inputs[3][15:8];

          8'd6: debug_msg_data <= inputs[4][7:0];
          8'd7: debug_msg_data <= inputs[4][15:8];
        endcase
      end
    endcase
  end

  //========================================
  // * Debug UART
  //========================================
  uart #(
    .CLKS_PER_BIT(BASE_FREQ/DEBUG_UART_BAUD)
  ) debug (
    .clock(clk),
    .txIn(w_debug_uart_byte),
    .txOut(DEBUG_UART_TX),
    .txSend(w_debug_uart_load),
    .txSendComplete(w_debug_uart_complete),
    .rxIn(DEBUG_UART_RX)
  );

  //========================================
  // * Debug Protocol
  //========================================
  debug_protocol debug_proto (
    .clock(clk),
    .txMsgType(w_debug_msg_type),
    .txMsgLen(w_debug_msg_length),
    
    .txMsgData(w_debug_msg_data),
    .txMsgDataIndex(w_debug_msg_data_index),
    .txMsgDataLoad(w_debug_msg_data_load),

    .send(w_debug_msg_send),
    .sendComplete(w_debug_send_complete),

    .uart_in(w_debug_uart_byte),
    .uartSend(w_debug_uart_load),
    .uartSendComplete(w_debug_uart_complete)
  );

  //========================================
  // * IMU 
  //========================================
  imu_mpu_9250 #(
    .BASE_FREQ(BASE_FREQ),
    .GYRO_UPDATE_HZ(GYRO_UPDATE_HZ),
    .GYRO_SPI_REG_FREQ(GYRO_SPI_REG_FREQ),
    .GYRO_SPI_UPDATE_FREQ(GYRO_SPI_UPDATE_FREQ)
  ) imu (
    .CLK(clk),
    .SCLK(IMU_SCLK),
    .MOSI(IMU_MOSI),
    .MISO(IMU_MISO),
    .CS(IMU_CS),
    .rates_raw_roll(gyro_rates_raw[0]),
    .rates_raw_pitch(gyro_rates_raw[1]),
    .rates_raw_yaw(gyro_rates_raw[2]),
    .sampleReady(gyroSampleReady)
  );

  //========================================
  // * RC Control Input 
  //========================================
  uart_rx #(
    .CLKS_PER_BIT(BASE_FREQ/FPORT_UART_BAUD)
  ) control_rx (
    .clock(clk),
    .rxIn(~RX_IN),
    .rxDataReady(rx_data_ready),
    .rxData(rx_data_byte)
  );

  //========================================
  // * FPORT Protocol Decoder
  //========================================
  fport_rx_decoder #(
    .BASE_FREQ(BASE_FREQ)
  ) fport_decode(
    .reset(reset),
    .clock(clk),
    .rxData(rx_data_byte),
    .rxDataAvail(rx_data_ready),
    .controlFrameReady(controls_ready),
    .rssi(rssi),
    .failsafe(failsafe),
    .rxFrameLoss(rxFrameLoss),
    .controls0(controlInputs[0]),
    .controls1(controlInputs[1]),
    .controls2(controlInputs[2]),
    .controls3(controlInputs[3]),
    .controls4(controlInputs[4])
  );
  
  // * TODO: merge motor control into single 
  // *  module with multiple channels, one for each motor.
  // * This should save some register space since all outputs 
  // *  can be managed with a single counter/alarm pair
  //========================================
  // * Motor 1
  //========================================
  motor_control #(
    .BASE_FREQ(BASE_FREQ),
    .DSHOT_FREQ(MOTOR_DSHOT_FREQ)
  ) motor1 (
    .clock(clk),
    .command(motorThrottle[0][10:0]),
    .send(motor_send),
    .txOut(motorOutputs[0])
  );

  //========================================
  // * Motor 2
  //========================================
  motor_control #(
    .BASE_FREQ(BASE_FREQ),
    .DSHOT_FREQ(MOTOR_DSHOT_FREQ)
  ) motor2 (
    .clock(clk),
    .command((motorThrottle[1][10:0])),
    .send(motor_send),
    .txOut(motorOutputs[1])
  );

  //========================================
  // * Motor 3
  //========================================
  motor_control #(
    .BASE_FREQ(BASE_FREQ),
    .DSHOT_FREQ(MOTOR_DSHOT_FREQ)
  ) motor3 (
    .clock(clk),
    .command((motorThrottle[2][10:0])),
    .send(motor_send),
    .txOut(motorOutputs[2])
  );

  //========================================
  // * Motor 4
  //========================================
  motor_control #(
    .BASE_FREQ(BASE_FREQ),
    .DSHOT_FREQ(MOTOR_DSHOT_FREQ)
  ) motor4 (
    .clock(clk),
    .command((motorThrottle[3][10:0])),
    .send(motor_send),
    .txOut(motorOutputs[3])
  );
endmodule
