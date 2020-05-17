`define SBUS_INPUT_MIN 192
`define SBUS_INPUT_MID 992
`define SBUS_INPUT_MAX 1792
`define SBUS_HALF_RANGE 792
`define SBUS_INPUT_RANGE `SBUS_INPUT_MAX - `SBUS_INPUT_MIN

`include "debugger_defines.vh"

// look in pins.pcf for all the pin names on the TinyFPGA BX board
module top 
#(
  parameter BASE_FREQ=16000000,
  
  parameter FPORT_UART_BAUD=115200,
  
  parameter MOTOR_UPDATE_HZ=4_000,
  
  parameter DSHOT_150_FREQ=150_000,
  parameter DSHOT_300_FREQ=300_000,
  parameter DSHOT_600_FREQ=600_000,

  parameter GYRO_UPDATE_HZ=8_000,
  parameter GYRO_SPI_REG_FREQ=1_000_000,
  parameter GYRO_SPI_UPDATE_FREQ=4_000_000,
  
  parameter FIXED_WIDTH_BIT=31,
  parameter DEBUG_UART_BAUD=400000,
  parameter DEBUG_UPDATE_HZ=100
)
(
    input CLK,    // 16MHz clock
    output LED,   // User/boot LED next to power LED
    output USBPU,  // USB pull-up resistor

    // RC Control input
    input PIN_3,
    
    // Gyro SPI master
    output PIN_4, // SCLK
    output PIN_5, // MOSI
    input  PIN_6, // MISO
    output PIN_7, // CS


    input PIN_10,
    output PIN_11,
    output PIN_12,
    input PIN_13,

    // Motor outputs
    output PIN_21,
    output PIN_22,
    output PIN_23,
    output PIN_24
);

  localparam MOTOR_DSHOT_FREQ = DSHOT_600_FREQ;
  localparam UPDATE_CLK_TICKS = BASE_FREQ / MOTOR_UPDATE_HZ;
  localparam DEBUG_SEND_CLK_TICKS = BASE_FREQ / DEBUG_UPDATE_HZ;
  localparam INPUT_THROTTLE = 0;
  localparam INPUT_ROLL = 1;
  localparam INPUT_PITCH = 2;
  localparam INPUT_YAW = 3;
  localparam INPUT_ARM = 4;

  wire clk;
  
  // Motors
  wire motorOutputs[3:0];
  reg [15:0] motorUpdateCount;
  reg motor_send;
  
  reg signed [31:0] motorThrottle[3:0];

  wire [10:0] controlInputs[4:0];

  reg signed [31:0] inputs[4:0];

  // First index should be motor count define
  reg signed [7:0] motorMix[3:0][3:0];

  wire signed [15:0] gyro_rates_raw[2:0];
  reg signed [15:0] gyro_rates_sampled[2:0];

  // RX Controls
  wire rx_in;
  wire [7:0] rx_data_byte;
  wire rx_data_ready;
  wire controls_ready;
  reg frame_ready;
  
  wire armed;
  wire failsafe;
  wire rxFrameLoss;
  wire [7:0] rssi;

  // Gyro SPI
  wire GYRO_MISO;
  wire GYRO_MOSI;
  wire GYRO_SCLK;
  wire GYRO_CS;
  wire gyroSampleReady;


  wire w_debug_uart_tx_out;

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

  wire fast_clk;

  reg debug_uart_load;
  reg debug_msg_send;
  reg [7:0] debug_msg_type;
  reg [7:0] debug_msg_length;
  reg [7:0] debug_msg_data;

  assign w_debug_msg_send   = debug_msg_send;
  assign w_debug_msg_type   = debug_msg_type;
  assign w_debug_msg_length = debug_msg_length;
  assign w_debug_msg_data   = debug_msg_data;

  localparam DEBUG_ST_RESET  = 0;
  localparam DEBUG_ST_WAIT   = 1;
  localparam DEBUG_ST_CHECK  = 2;
  localparam DEBUG_ST_SEND   = 3;
  localparam DEBUG_ST_LOAD   = 4;

  localparam DEBUG_SEND_GYRO     = 4;
  localparam DEBUG_SEND_RX       = 3;
  localparam DEBUG_SEND_MOTOR    = 2;
  localparam DEBUG_SEND_ATTITUDE = 1;
  localparam DEBUG_SEND_COMPLETE = 0;

  reg [4:0] debugState;
  reg [4:0] debugSendState;
  reg [31:0] debugUpdateCount;


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
      RxRangeInput = (((5 * inputValue / 8) + 880) - 1500) * 4;
    end
  endfunction

  function automatic signed [31:0] MotorThrottleValue;
    input [3:0] index;
    begin
      MotorThrottleValue = armed ? 
           ( motorMix[index][INPUT_THROTTLE] * inputs[INPUT_THROTTLE]
           + motorMix[index][INPUT_ROLL]    * inputs[INPUT_ROLL] 
           + motorMix[index][INPUT_PITCH]   * inputs[INPUT_PITCH]
           + motorMix[index][INPUT_YAW]     * inputs[INPUT_YAW])// + 2000)/2
        : 0;
    end
  endfunction
  

  // Motor index layout, up is forward
  //  3 ^ 1
  //    X
  //  2   0

  initial begin
    motorMix[0][INPUT_THROTTLE] = 1;
    motorMix[0][INPUT_ROLL] = 1;
    motorMix[0][INPUT_PITCH] = -1;
    motorMix[0][INPUT_YAW] = -1;

    motorMix[1][INPUT_THROTTLE] = 1;
    motorMix[1][INPUT_ROLL] = 1;
    motorMix[1][INPUT_PITCH] = 1;
    motorMix[1][INPUT_YAW] = 1;

    motorMix[2][INPUT_THROTTLE] = 1;
    motorMix[2][INPUT_ROLL] = -1;
    motorMix[2][INPUT_PITCH]= -1;
    motorMix[2][INPUT_YAW] =1;

    motorMix[3][INPUT_THROTTLE] = 1;
    motorMix[3][INPUT_ROLL] = -1;
    motorMix[3][INPUT_PITCH] = 1;
    motorMix[3][INPUT_YAW] = -1;

    debugState = DEBUG_ST_RESET;
    debugUpdateCount = DEBUG_SEND_CLK_TICKS;
    gyro_rates_sampled[0] = 16'h0;
    gyro_rates_sampled[1] = 16'h0;
    gyro_rates_sampled[2] = 16'h0;

    inputs[0] = 32'h0;
    inputs[1] = 32'h0;
    inputs[2] = 32'h0;
    inputs[3] = 32'h0;
    inputs[4] = 32'h0;
  end

  /*  Generate a 50 MHz internal clock from 16 MHz input clock  */
// SB_PLL40_CORE pll_inst (
//    .REFERENCECLK(CLK),
//    .PLLOUTCORE(fast_clk),
//    .PLLOUTGLOBAL(),
//    .EXTFEEDBACK(),
//    .DYNAMICDELAY(),
//    .RESETB(1'b1),
//    .BYPASS(1'b0),
//    .LATCHINPUTVALUE(),
//    .LOCK(),
//    .SDI(),
//    .SDO(),
//    .SCLK()
//  );  

//    defparam pll_inst.DIVR = 4'b0000;
//  defparam pll_inst.DIVF = 7'b0101111;
//  defparam pll_inst.DIVQ = 3'b100;
//  defparam pll_inst.FILTER_RANGE = 3'b001;
//  defparam pll_inst.FEEDBACK_PATH = "SIMPLE";
//  defparam pll_inst.DELAY_ADJUSTMENT_MODE_FEEDBACK = "FIXED";
//  defparam pll_inst.FDA_FEEDBACK = 4'b0000;
//  defparam pll_inst.DELAY_ADJUSTMENT_MODE_RELATIVE = "FIXED";
//  defparam pll_inst.FDA_RELATIVE = 4'b0000;
//  defparam pll_inst.SHIFTREG_DIV_MODE = 2'b00;
//  defparam pll_inst.PLLOUT_SELECT = "GENCLK";
//  defparam pll_inst.ENABLE_ICEGATE = 1'b0;

  // PINS
  assign clk = CLK;

  // drive USB pull-up resistor to '0' to disable USB
  assign USBPU = 0;
  
  assign rx_in = ~PIN_3;

  assign PIN_12 = w_debug_uart_tx_out;

  assign PIN_21 = motorOutputs[0];
  assign PIN_22 = motorOutputs[1];
  assign PIN_23 = motorOutputs[2];
  assign PIN_24 = motorOutputs[3];

  assign GYRO_SCLK = PIN_4;
  assign GYRO_MOSI = PIN_5;
  assign GYRO_MISO = PIN_6;
  assign GYRO_CS = PIN_7;
  
  assign armed = controlInputs[0] > 0;

  // * Sample Gyro State
  always @(posedge gyroSampleReady) begin
    gyro_rates_sampled[0] <= gyro_rates_raw[0];
    gyro_rates_sampled[1] <= gyro_rates_raw[1];
    gyro_rates_sampled[2] <= gyro_rates_raw[2];
  end

  // * Main Loop: process gyro, rx, and attitude data to update motor controls
  always @(posedge clk) begin
    if( motorUpdateCount > 0) begin
      motorUpdateCount <= motorUpdateCount - 1;
      motor_send <= 0;
    end else begin
      motorUpdateCount <= UPDATE_CLK_TICKS;
      motor_send <= 1;

      motorThrottle[0] <= MotorThrottleValue(0);
      motorThrottle[1] <= MotorThrottleValue(1);
      motorThrottle[2] <= MotorThrottleValue(2);
      motorThrottle[3] <= MotorThrottleValue(3);
    end
  end

  // * Update RX control inputs
  always @(posedge controls_ready) begin
    inputs[INPUT_THROTTLE] <= RxRangeInput(controlInputs[INPUT_THROTTLE]);
    inputs[INPUT_ROLL] <= -1 * RxRangeInput(controlInputs[INPUT_ROLL]);
    inputs[INPUT_PITCH] <= -1 * RxRangeInput(controlInputs[INPUT_PITCH]);
    inputs[INPUT_YAW] <= RxRangeInput(controlInputs[INPUT_YAW]);
  end

  // * Debug protocol
  always @(posedge clk) begin
    case(debugState)
      DEBUG_ST_RESET: begin
        debugState <= DEBUG_ST_WAIT;
      end

      DEBUG_ST_WAIT: 
      begin
        if( debugUpdateCount > 0) begin
          debugUpdateCount <= debugUpdateCount - 1;
        end else begin
          debugUpdateCount <= DEBUG_SEND_CLK_TICKS;
          debugState <= DEBUG_ST_SEND;
          debugSendState <= `MSG_TELEMETRY_COUNT;
        end
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
        debugState <= DEBUG_ST_WAIT;
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
        endcase

        debug_msg_send <= 1;
        debugState <= DEBUG_ST_LOAD;
      end
    endcase
  end

  // * Debug Protocol message data loader
  always @(posedge w_debug_msg_data_load) begin
    case (debugSendState)
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
          // * Throttle
          8'd0: debug_msg_data <= motorThrottle[0][7:0];
          8'd1: debug_msg_data <= motorThrottle[0][15:8];

          8'd2: debug_msg_data <= motorThrottle[1][7:0];
          8'd3: debug_msg_data <= motorThrottle[1][15:8];

          8'd4: debug_msg_data <= motorThrottle[2][7:0];
          8'd5: debug_msg_data <= motorThrottle[2][15:8];

          8'd6: debug_msg_data <= motorThrottle[3][7:0];
          8'd7: debug_msg_data <= motorThrottle[3][15:8];
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
    .txOut(w_debug_uart_tx_out),
    .txSend(w_debug_uart_load),
    .txSendComplete(w_debug_uart_complete),
    .rxIn(PIN_13)
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
    .GYRO_UPDATE_HZ(GYRO_UPDATE_HZ),
    .GYRO_SPI_REG_FREQ(GYRO_SPI_REG_FREQ),
    .GYRO_SPI_UPDATE_FREQ(GYRO_SPI_UPDATE_FREQ)
  ) imu (
    .CLK(clk),
    .SCLK(GYRO_SCLK),
    .MOSI(GYRO_MOSI),
    .MISO(GYRO_MISO),
    .CS(GYRO_CS),
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
    .rxIn(rx_in),
    .rxDataReady(rx_data_ready),
    .rxData(rx_data_byte)
  );

  //========================================
  // * FPORT Protocol Decoder
  //========================================
  fport_rx_decoder fport_decode(
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
    .DSHOT_FREQ(MOTOR_DSHOT_FREQ)
  ) motor4 (
    .clock(clk),
    .command((motorThrottle[3][10:0])),
    .send(motor_send),
    .txOut(motorOutputs[3])
  );
endmodule
