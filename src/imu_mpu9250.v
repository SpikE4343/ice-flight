// look in pins.pcf for all the pin names on the TinyFPGA BX board

`include "imu_defines.vh"

module imu_mpu_9250 
#(
  parameter BASE_FREQ=16_000_000,
  parameter GYRO_UPDATE_HZ=16_000,
  parameter GYRO_SPI_REG_FREQ=1_000_000,
  parameter GYRO_SPI_UPDATE_FREQ=4_000_000,
  parameter FIXED_PRECISION=1000,
  parameter SPI_REG_WORDBITS=16,
  parameter SPI_GYRO_WORDBITS=16*3 + 8,
  parameter FIXED_WIDTH_BIT=31
)
(
    input CLK,    // 16MHz clock
    
    // Gyro SPI master
    output SCLK, // SCLK
    output MOSI, // MOSI
    input  MISO, // MISO
    output CS, // CS

    output reg signed [15:0] rates_raw_roll,
    output reg signed [15:0] rates_raw_pitch,
    output reg signed [15:0] rates_raw_yaw,
    output reg sampleReady
);

  localparam UPDATE_CLK_TICKS = BASE_FREQ / GYRO_UPDATE_HZ;
  
  wire clk;
  
  // Gyro update
  reg [15:0] gyroUpdateCounter;
  reg gyro_update;

  //Debug UART
  reg debug_send;
  reg [7:0] debug_data_byte;

  
  reg [31:0] counter1;

  // Hish speed readonly
  reg [SPI_GYRO_WORDBITS-1:0] spiTxData;
  reg spiTx;

  wire [SPI_GYRO_WORDBITS-1:0] spiRxData;
  wire spiRx;

  // Configuration Registers
  reg [SPI_REG_WORDBITS-1:0] spiRegTxData;
  reg spiRegTx;

  wire [SPI_REG_WORDBITS-1:0] spiRegRxData;
  wire spiRegRx;


  reg [15:0] gyroConfigCmdCount;
  reg [15:0] gyroConfigCmdSentCount;
  reg [15:0] gyroConfigCmds[8:0];

  localparam IMU_ST_RESET=0;
  localparam IMU_ST_STARTUP=1;
  localparam IMU_ST_FAIL=-1;
  localparam IMU_ST_CALIBRATION=2;
  localparam IMU_ST_ACTIVE=3;
  

  localparam IMU_ST_CFG_TRANSMIT_WAIT=0;
  localparam IMU_ST_CFG_RESET=1;
  localparam IMU_ST_CFG_RESET_WAIT=2;
  localparam IMU_ST_CFG_NEXT_COMMAND=3;
  localparam IMU_ST_CFG_SEND_COMMAND=4;
  localparam IMU_ST_CFG_COMPLETE=5;
  
  localparam IMU_ST_ACTIVE_IDLE=0;
  localparam IMU_ST_ACTIVE_READ_GYRO=1;
  localparam IMU_ST_ACTIVE_READ_GYRO_WAIT=2;
  localparam IMU_ST_FILTER_RATES=3;
  


  reg [7:0] imu_state;
  reg [7:0] imu_cfg_state;
  reg [7:0] imu_active_state;

  // Angular Velocity deg/sec 
  // reg signed [15:0] gyro_rates_raw[2:0];
  
  // assign rates_raw_roll = gyro_rates_raw[0];
  // assign rates_raw_pitch = gyro_rates_raw[1];
  // assign rates_raw_yaw = gyro_rates_raw[2];

  initial begin
    gyroConfigCmds[0] = { `MPU_RA_PWR_MGMT_1, `INV_CLK_PLL };
    gyroConfigCmds[1] = { `MPU_RA_GYRO_CONFIG, `INV_FSR_1000DPS << 3};
    gyroConfigCmds[2] = { `MPU_RA_CONFIG, 8'h07 };
    gyroConfigCmds[3] = { `MPU_RA_SMPLRT_DIV, 8'h00 };
    gyroConfigCmds[4] = { `MPU_RA_ACCEL_CONFIG, `INV_FSR_16G << 3 };
    gyroConfigCmds[5] = { `MPU_RA_INT_PIN_CFG, 8'b00010010};
    gyroConfigCmdCount = 6;

    imu_state = IMU_ST_RESET;
    imu_cfg_state = IMU_ST_CFG_RESET;
    imu_active_state = IMU_ST_ACTIVE_IDLE;
    counter1 = BASE_FREQ / 1000;
  end 

   


  // PINS
  assign clk = CLK;

  wire fastOutSCLK;
  wire fastOutMOSI;
  wire fastOutCS;

  wire regOutSCLK;
  wire regOutMOSI;
  wire regOutCS;

  assign SCLK = spi_out_fast ? fastOutSCLK : regOutSCLK;
  assign MOSI = spi_out_fast ? fastOutMOSI : regOutMOSI;
  assign CS = spi_out_fast ? fastOutCS : regOutCS;
  

  reg debug_;
  reg spi_out_fast;

  always @(posedge clk) 
  begin
    if( gyroUpdateCounter > 0) 
    begin
      gyroUpdateCounter <= gyroUpdateCounter - 1;
      gyro_update <= 0;
    end else begin
      gyroUpdateCounter <= UPDATE_CLK_TICKS;
      gyro_update <= 1;
    end
  end

  // uart_tx debug (
  //   .clock(clk),
  //   .txIn(debug_data_byte),
  //   .txOut(PIN_4),
  //   .send(debug_send)
  // );
  spi_master #(
    .WORDBITS(SPI_REG_WORDBITS),
    .CLKS_PER_BIT(BASE_FREQ/GYRO_SPI_REG_FREQ)
  ) spi_registers(
    .clock(clk),

    .inMISO(MISO),
    .outCLK(regOutSCLK),
    .outMOSI(regOutMOSI),
    .outCS(regOutCS),

    .complete(spiRegRx),
    .send(spiRegTx),

    .txData(spiRegTxData),
    .rxData(spiRegRxData)
  );


  spi_master #(
    .WORDBITS(SPI_GYRO_WORDBITS),
    .CLKS_PER_BIT(BASE_FREQ/GYRO_SPI_UPDATE_FREQ)
  ) spi_fast (
    .clock(clk),

    .inMISO(MISO),
    .outCLK(fastOutSCLK),
    .outMOSI(fastOutMOSI),
    .outCS(fastOutCS),

    .complete(spiRx),
    .send(spiTx),

    .txData(spiTxData),
    .rxData(spiRxData)
  );

  reg [7:0] gyro_update_state;
  reg data_ready;
  

  always @(posedge spiRx) begin
    data_ready <= 1;
  end

  always @(posedge clk) begin
    case (imu_state)
      IMU_ST_RESET:begin
        if(counter1 == 0)
          imu_state <= IMU_ST_STARTUP;

        counter1 <= counter1 - 1;
      end
      // Start Up
      IMU_ST_STARTUP: begin
        case(imu_cfg_state)
          
          IMU_ST_CFG_RESET: begin // 1 
            gyroConfigCmdSentCount <= 0;
            spi_out_fast <= 0;
            spiRegTxData <= { `MPU_RA_PWR_MGMT_1, `MPU9250_BIT_RESET };
            spiRegTx <= 1; 
            imu_cfg_state <= IMU_ST_CFG_RESET_WAIT;
            counter1 <= BASE_FREQ / 2000000;
          end

          // wait for power up time (50 ms)
          IMU_ST_CFG_RESET_WAIT: begin // 2
            if(counter1 > 0)
              counter1 <= counter1 - 1;
            else 
              imu_cfg_state <= IMU_ST_CFG_NEXT_COMMAND;
          end
          
          IMU_ST_CFG_TRANSMIT_WAIT: begin // 0
            spiRegTx <= 0; 
            if(spiRegRx)
              imu_cfg_state <= IMU_ST_CFG_NEXT_COMMAND;
          end

          IMU_ST_CFG_SEND_COMMAND: begin // 4
            spiRegTxData <= gyroConfigCmds[gyroConfigCmdSentCount];
            spiRegTx <= 1; 
            imu_cfg_state <= IMU_ST_CFG_TRANSMIT_WAIT;
          end

          IMU_ST_CFG_NEXT_COMMAND: begin // 3
            if(gyroConfigCmdSentCount >= gyroConfigCmdCount-1)
              imu_cfg_state = IMU_ST_CFG_COMPLETE;
            else begin
              gyroConfigCmdSentCount <= gyroConfigCmdSentCount + 1;
              imu_cfg_state <= IMU_ST_CFG_SEND_COMMAND;
            end
          end

          IMU_ST_CFG_COMPLETE: begin // 5
            imu_state <= IMU_ST_CALIBRATION;
          end
        endcase
      end

      IMU_ST_FAIL: 
      begin
      end

      IMU_ST_CALIBRATION: 
      begin
        imu_state <= IMU_ST_ACTIVE;
      end

      IMU_ST_ACTIVE: 
      begin
        
        case(imu_active_state)

          IMU_ST_ACTIVE_IDLE: 
          begin
            sampleReady <= 0;
            if(gyro_update) begin
              spi_out_fast <= 1;
              spiTxData = { 
                `MPU_RA_GYRO_XOUT_H | `MPU_RA_READ_FLAG, 
                
                // X Axis
                `MPU_NULL_BYTE,
                `MPU_NULL_BYTE,

                // Y Axis
                `MPU_NULL_BYTE,
                `MPU_NULL_BYTE,
                
                // Z axis
                `MPU_NULL_BYTE,
                `MPU_NULL_BYTE 
              };
              spiTx <= 1; 
              imu_active_state <= IMU_ST_ACTIVE_READ_GYRO_WAIT;
            end else begin
              spiTx <= 0;
            end
              
          end

          IMU_ST_ACTIVE_READ_GYRO_WAIT: 
          begin
            if(spiRx) begin
              imu_active_state <= IMU_ST_ACTIVE_IDLE;

              rates_raw_yaw <= spiRxData[15:0] * -1;// / `GYRO_RATE_SCALAR;
              rates_raw_pitch <= spiRxData[31:16];// / `GYRO_RATE_SCALAR;
              rates_raw_roll <= spiRxData[47:32] * -1;// / `GYRO_RATE_SCALAR;
              sampleReady <= 1;
            end
          end
          
          // IMU_ST_FILTER_RATES: 
          // begin
          //   if(spiRx) begin
          //     gyro_rates_filtered[0] <= (gyro_rates_filtered[0] + gyro_rates_raw[0]) >>> 2;
          //     gyro_rates_filtered[1] <= (gyro_rates_filtered[1] + gyro_rates_raw[1]) >>> 2;
          //     gyro_rates_filtered[2] <= (gyro_rates_filtered[2] + gyro_rates_raw[2]) >>> 2;
          //   end
          //   imu_active_state <= IMU_ST_ACTIVE_IDLE;
          // end
        endcase
      end
    endcase
  end
endmodule
