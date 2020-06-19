// look in pins.pcf for all the pin names on the TinyFPGA BX board

`include "ip/peripherals/imu/imu_defines.vh"

module imu_mpu_9250 
#(
  parameter BASE_FREQ=16_000_000,
  parameter GYRO_UPDATE_HZ=16_000,
  parameter GYRO_SPI_REG_FREQ=1_000_000,
  parameter GYRO_SPI_UPDATE_FREQ=20_000_000,
  parameter FIXED_PRECISION=1000,
  parameter SPI_REG_WORDBITS=16,
  parameter SPI_GYRO_WORDBITS=16*3 + 8,
  parameter FIXED_WIDTH_BIT=31,
  parameter SAMPLE_DEPTH=64
)
(
    input CLK, 
    
    // Gyro SPI master
    output SCLK, // SCLK
    output MOSI, // MOSI
    input  MISO, // MISO
    output CS, // CS

    output reg signed [31:0] rates_raw_roll,
    output reg signed [31:0] rates_raw_pitch,
    output reg signed [31:0] rates_raw_yaw,
    output reg sampleReady
);

  localparam UPDATE_CLK_TICKS = BASE_FREQ / GYRO_UPDATE_HZ;
  
  wire clk;
  
  // Gyro update
  reg [15:0] gyroUpdateCounter;
  reg gyro_update;
  reg [15:0] gyroOffsets[3:0];


  // generic 32 bit counter to use
  reg [31:0] counter1;

  // High speed readonly
  reg [SPI_GYRO_WORDBITS-1:0] spiTxData;
  reg spiTx;

  wire [SPI_GYRO_WORDBITS-1:0] spiRxData;
  wire spiRx;

  // Configuration Registers
  reg [SPI_REG_WORDBITS-1:0] spiRegTxData;
  reg spiRegTx;

  wire [SPI_REG_WORDBITS-1:0] spiRegRxData;
  wire spiRegRx;

  //reg signed [31:0] gyroSampleBuffer [SAMPLE_DEPTH-1:0];
  reg [$clog2(SAMPLE_DEPTH)-1:0] sample_write;
  reg [$clog2(SAMPLE_DEPTH)-1:0] sample_read;

  reg [15:0] gyroConfigCmdCount;
  reg [15:0] gyroConfigCmdSentCount;
  reg [15:0] gyroConfigCmds[8:0];

  // * Main IMU states
  localparam IMU_ST_RESET=0;
  localparam IMU_ST_STARTUP=1;
  localparam IMU_ST_FAIL=-1;
  localparam IMU_ST_CALIB_START=2;
  localparam IMU_ST_CALIBRATION=3;
  localparam IMU_ST_CALIB_SUCCESS =4;
  localparam IMU_ST_ACTIVE=5;
  
  // * IMU Configuration states
  localparam IMU_ST_CFG_TRANSMIT_WAIT=0;
  localparam IMU_ST_CFG_RESET=1;
  localparam IMU_ST_CFG_RESET_WAIT=2;
  localparam IMU_ST_CFG_NEXT_COMMAND=3;
  localparam IMU_ST_CFG_SEND_COMMAND=4;
  localparam IMU_ST_CFG_COMPLETE=5;
  
  // * IMU Sampling system states
  localparam IMU_ST_SAMPLE_STOPPED=0;
  localparam IMU_ST_SAMPLE_IDLE=1;
  localparam IMU_ST_SAMPLE_READ_GYRO=2;
  localparam IMU_ST_SAMPLE_READ_GYRO_WAIT=3;
  
  reg [7:0] imu_state;
  reg [7:0] imu_cfg_state;
  reg [7:0] imu_calibration_state;
  reg [7:0] imu_sample_state;
  integer i;

  localparam SAMPLE_WORDS=744;
  `ifdef gyro_test_data
      reg signed [15:0] testSampleData[0:SAMPLE_WORDS-1];
      reg [8:0] testSampleRead;
  `endif
  
  initial begin
    gyroConfigCmds[0] = { `MPU_RA_PWR_MGMT_1, `INV_CLK_PLL };
    gyroConfigCmds[1] = { `MPU_RA_GYRO_CONFIG, `INV_FSR_2000DPS << 3};
    gyroConfigCmds[2] = { `MPU_RA_CONFIG, 8'h00 };//8'h07 };
    gyroConfigCmds[3] = { `MPU_RA_SMPLRT_DIV, 8'h00 };
    gyroConfigCmds[4] = { `MPU_RA_ACCEL_CONFIG, `INV_FSR_16G << 3 };
    gyroConfigCmds[5] = { `MPU_RA_INT_PIN_CFG, 8'b00010010};
    gyroConfigCmdCount = 6;

    rates_raw_roll = 32'h0;
    rates_raw_pitch = 32'h0;
    rates_raw_yaw = 32'h0;

    gyroOffsets[0] = 16'h0000;
    gyroOffsets[1] = 16'h0000;
    gyroOffsets[2] = 16'h0000;

    imu_state = IMU_ST_RESET;
    imu_cfg_state = IMU_ST_CFG_RESET;
    imu_sample_state = IMU_ST_SAMPLE_STOPPED;
    imu_calibration_state = IMU_ST_CALIB_START;
    counter1 = 0;//BASE_FREQ / 1000;

    sample_read = 7'd0;
    sample_write = 7'd0;

//    for(i=0; i < SAMPLE_DEPTH; i=i+1) begin
//      gyroSampleBuffer[i] = 32'd0;
//    end

    
    `ifdef gyro_test_data
       $readmemh("mem-gyro-1024.txt", testSampleData);
       testSampleRead = 0;
    `endif
  end 

  // PINS
  assign clk = CLK;

  wire fastOutSCLK;
  wire fastOutMOSI;
  wire fastOutCS;

  wire regOutSCLK;
  wire regOutMOSI;
  wire regOutCS;

  reg spi_out_fast;
  
  assign SCLK = fastOutSCLK;//spi_out_fast ? fastOutSCLK : regOutSCLK;
  assign MOSI = spi_out_fast ? fastOutMOSI : regOutMOSI;
  assign CS = spi_out_fast ? fastOutCS : regOutCS;
  
  
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


  always @(posedge clk) begin
    
    
    if( gyroUpdateCounter > 0) 
    begin
      gyroUpdateCounter <= gyroUpdateCounter - 1;
      gyro_update <= 0;
    end else begin
      gyroUpdateCounter <= UPDATE_CLK_TICKS;
      gyro_update <= 1;
    end
    
    case (imu_state)
      IMU_ST_RESET:begin
        if(counter1 == 0)
          imu_state <= IMU_ST_STARTUP;

        counter1 <= counter1 - 1;
      end
      // * Write imu configuration registers
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

      // * Startup, configuration or calibration has failed, disable sampling
      IMU_ST_FAIL: 
      begin
        
        imu_sample_state <= IMU_ST_SAMPLE_STOPPED;
      end

      // * Sample imu and calculate stddev of each axis, if all are less than a given
      // * threashold, then store averaged values as dc offsets
      // * When data is too noisy to find offsets, restart calibration process
      IMU_ST_CALIBRATION: 
      begin
          
        case (imu_calibration_state)
          IMU_ST_CALIB_START: 
          begin
            if(imu_sample_state == IMU_ST_SAMPLE_STOPPED)
              imu_sample_state <= IMU_ST_SAMPLE_IDLE;
          end

          IMU_ST_CALIB_SUCCESS:
          begin
             imu_state <= IMU_ST_ACTIVE;
          end
        endcase
       
      end

      IMU_ST_ACTIVE: 
      begin
        
        // start pulling samples from the imu
        if(imu_sample_state == IMU_ST_SAMPLE_STOPPED)
          imu_sample_state <= IMU_ST_SAMPLE_IDLE;
      end
    endcase

    // * Sample imu independently of main imu state
    case(imu_sample_state)
      IMU_ST_SAMPLE_STOPPED:
      begin
      end

      IMU_ST_SAMPLE_IDLE: 
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
          imu_sample_state <= IMU_ST_SAMPLE_READ_GYRO_WAIT;
        end else begin
          spiTx <= 0;
        end
          
      end

      // TODO: ring buffer for raw sample data
      IMU_ST_SAMPLE_READ_GYRO_WAIT: 
      begin
        if(spiRx) begin
          //imu_sample_state <= IMU_ST_SAMPLE_IDLE;


          // 7FFF
          //  111 1111 1111 1111.
          //  111 1.111 1111 1111
          //  1111.1111 1111 1110
          
          // 111 1.111 1111 1111
          // 1111.1111 11111110 00000000 00000000


          // 0000 . 1111 | 1111 1111 | 1110 0000 | 0000 0000
          // 
          // 16.4 

          //  F.FF E0 00 0
          // 0.F FF E0 00 
          // { 4'b0, sample, }
          `ifdef gyro_test_data
            // gyroSampleBuffer[sample_write] <= -testSampleData[testSampleRead];// / 2000 / 16  - gyroOffsets[2]) * -1;// / `GYRO_RATE_SCALAR;

            // // Pitch
            // gyroSampleBuffer[sample_write+9'd1] <= testSampleData[testSampleRead+9'd1];//  - gyroOffsets[1]);// / `GYRO_RATE_SCALAR;
            
            
            // gyroSampleBuffer[sample_write+9'd2] <= -testSampleData[testSampleRead+9'd2];


            rates_raw_roll <= testSampleData[testSampleRead] <<< 13;
            rates_raw_pitch <=testSampleData[testSampleRead+9'd1] <<< 13;
            rates_raw_yaw <= testSampleData[testSampleRead+9'd2] <<< 13;

            testSampleRead <= testSampleRead + 9'd3;
          `else
            rates_raw_roll <= { spiRxData[47], spiRxData[47], spiRxData[47],  spiRxData[47], spiRxData[47:32], 12'h0};
            rates_raw_pitch <= { spiRxData[31], spiRxData[31], spiRxData[31], spiRxData[31], spiRxData[31:16], 12'h0};
            rates_raw_yaw <= { spiRxData[15], spiRxData[15], 3'h0, spiRxData[15:0], 12'h0};

            // rates_raw_roll <= spiRxData[47:32] <<< 13;
            // rates_raw_pitch <= spiRxData[31:16] <<< 13;
            // rates_raw_yaw <= spiRxData[15:0] <<< 13;
            // Yaw
            // gyroSampleBuffer[sample_write+9'd2] <= spiRxData[15:0];// / 2000 / 16  - gyroOffsets[2]) * -1;// / `GYRO_RATE_SCALAR;

            // // Pitch
            // gyroSampleBuffer[sample_write+9'd1] <= spiRxData[31:16];//  - gyroOffsets[1]);// / `GYRO_RATE_SCALAR;
            
            // // Roll
            // gyroSampleBuffer[sample_write]      <= spiRxData[47:32];//  - gyroOffsets[0])* -1;// / `GYRO_RATE_SCALAR;
          `endif
          sample_write <= sample_write + 9'd3;
          imu_sample_state <= 8'd4; // * signal ready
        end
      end

      4: // * IMU_ST_SAMPLE_READY:
      begin
        sample_read <= sample_read + 9'd3;
        counter1 <= 8;
        imu_sample_state  <= 5;
        sampleReady <= 1;
      end

       5: // * IMU_ST_SAMPLE_READY:
      begin        
        if(counter1 == 0) begin
          imu_sample_state  <= IMU_ST_SAMPLE_IDLE;
        end

        counter1 = counter1 - 1;
        sampleReady <= 1;
      end
    endcase
  end
endmodule
