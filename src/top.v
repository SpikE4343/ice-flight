`define SBUS_INPUT_MIN 192
`define SBUS_INPUT_MID 992
`define SBUS_INPUT_MAX 1792
`define SBUS_HALF_RANGE 792
`define SBUS_INPUT_RANGE `SBUS_INPUT_MAX - `SBUS_INPUT_MIN

`define MSG_MARKER 8'h7f
`define MSG_TYPE_TELEMETRY 8'h01

// look in pins.pcf for all the pin names on the TinyFPGA BX board
module top 
#(
  parameter BASE_FREQ=16000000,
  parameter FPORT_UART_BAUD=115200,
  parameter MOTOR_UPDATE_HZ=1_000,
  parameter DSHOT_150_FREQ=150_000,
  parameter DSHOT_300_FREQ=300_000,
  parameter DSHOT_600_FREQ=600_000,
  parameter DSHOT_1200_FREQ=1_200_000,
  parameter GYRO_UPDATE_HZ=1_000,
  parameter GYRO_SPI_REG_FREQ=1_000_000,
  parameter GYRO_SPI_UPDATE_FREQ=4_000_000,
  parameter FIXED_WIDTH_BIT=31,
  parameter TELEMETRY_HZ=1,
  parameter DEBUG_BITS=72,
  parameter DEBUG_UART_BAUD=115200
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
  
  reg signed [31:0] motorThrottle1;
  reg signed [31:0] motorThrottle2;
  reg signed [31:0] motorThrottle3;
  reg signed [31:0] motorThrottle4;

  wire [10:0] controlInputs1;
  wire [10:0] controlInputs2;
  wire [10:0] controlInputs3;
  wire [10:0] controlInputs4;
  wire [10:0] controlInputs5;

  // First index should be motor count define
  reg signed [31:0] motorMix[3:0][3:0];

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


  reg [DEBUG_BITS-1:0] debug_byte;
  reg debug_load;
  wire dbg_tx_out;

  wire dbg_load;
  wire [DEBUG_BITS-1:0] dbg_byte;

  wire fast_clk;

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
            motorMix[index][INPUT_THROTTLE] * (RxRangeInput(controlInputs1)+2000)/4
           + motorMix[index][INPUT_ROLL]     * RxRangeInput(controlInputs2)
           + motorMix[index][INPUT_PITCH]    * RxRangeInput(controlInputs3) 
           + motorMix[index][INPUT_YAW]      * RxRangeInput(controlInputs4)
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

  assign dbg_byte = debug_byte;
  assign dbg_load = debug_load;

  assign PIN_12 = dbg_tx_out;

  assign PIN_21 = motorOutputs[0];
  assign PIN_22 = motorOutputs[1];
  assign PIN_23 = motorOutputs[2];
  assign PIN_24 = motorOutputs[3];

  assign GYRO_SCLK = PIN_4;
  assign GYRO_MOSI = PIN_5;
  assign GYRO_MISO = PIN_6;
  assign GYRO_CS = PIN_7;
  
  assign armed = controlInputs1 > 0;

  assign PIN_11 = gyroSampleReady;

  always @(posedge gyroSampleReady) begin
    gyro_rates_sampled[0] <= gyro_rates_raw[0];
    gyro_rates_sampled[1] <= gyro_rates_raw[1];
    gyro_rates_sampled[2] <= gyro_rates_raw[2];
  end

  always @(posedge clk) begin
    if( motorUpdateCount > 0) begin
      motorUpdateCount <= motorUpdateCount - 1;
      motor_send <= 0;
      debug_load <= 0;
    end else begin
      motorUpdateCount <= UPDATE_CLK_TICKS;
      motor_send <= 1;

      motorThrottle1 <= MotorThrottleValue(0);
      motorThrottle2 <= MotorThrottleValue(1);
      motorThrottle3 <= MotorThrottleValue(2);
      motorThrottle4 <= MotorThrottleValue(3);

      // update gyro state
      debug_byte <= {
        `MSG_MARKER,
        `MSG_TYPE_TELEMETRY,
        8'd6,
        // little endian
        gyro_rates_sampled[0][7:0],
        gyro_rates_sampled[0][15:8],
        gyro_rates_sampled[1][7:0],
        gyro_rates_sampled[1][15:8],
        gyro_rates_sampled[2][7:0],
        gyro_rates_sampled[2][15:8]
        };

      debug_load <= 1;
      //debug_load = 0;
      //debug_byte = controlInputs1[10:8];
      //debug_load = 1;
      //debug_load = 0;
    end
  end

  uart #(
    .CLKS_PER_BIT(BASE_FREQ/DEBUG_UART_BAUD),
    .INPUT_BITS(DEBUG_BITS)
  ) debug (
    .clock(clk),
    .txIn(dbg_byte),
    .txOut(dbg_tx_out),
    .txSend(dbg_load),
    .rxIn(PIN_13)
  );

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

  // RC control
  uart_rx #(
    .CLKS_PER_BIT(BASE_FREQ/FPORT_UART_BAUD)
  ) control_rx (
    .clock(clk),
    .rxIn(rx_in),
    .rxDataReady(rx_data_ready),
    .rxData(rx_data_byte)
  );

  fport_rx_decoder fport_decode(
    .clock(clk),
    .rxData(rx_data_byte),
    .rxDataAvail(rx_data_ready),
    .controlFrameReady(controls_ready),
    .rssi(rssi),
    .failsafe(failsafe),
    .rxFrameLoss(rxFrameLoss),
    .controls0(controlInputs1),
    .controls1(controlInputs2),
    .controls2(controlInputs3),
    .controls3(controlInputs4),
    .controls4(controlInputs5)
  );
  
   // Motor Outputs
  motor_control #(
    .DSHOT_FREQ(MOTOR_DSHOT_FREQ)
  ) motor1 (
    .clock(clk),
    .command(motorThrottle1[10:0]),
    .send(motor_send),
    .txOut(motorOutputs[0])
  );

  motor_control #(
    .DSHOT_FREQ(MOTOR_DSHOT_FREQ)
  ) motor2 (
    .clock(clk),
    .command((motorThrottle2[10:0])),
    .send(motor_send),
    .txOut(motorOutputs[1])
  );

  motor_control #(
    .DSHOT_FREQ(MOTOR_DSHOT_FREQ)
  ) motor3 (
    .clock(clk),
    .command((motorThrottle3[10:0])),
    .send(motor_send),
    .txOut(motorOutputs[2])
  );

  motor_control #(
    .DSHOT_FREQ(MOTOR_DSHOT_FREQ)
  ) motor4 (
    .clock(clk),
    .command((motorThrottle4[10:0])),
    .send(motor_send),
    .txOut(motorOutputs[3])
  );
endmodule
