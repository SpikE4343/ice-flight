`define MPU9250_BIT_RESET      8'h80
`define MPU_RA_XG_OFFS_TC      8'h00    //[7] PWR_MODE, [6:1] XG_OFFS_TC, [0] OTP_BNK_VLD
`define MPU_RA_YG_OFFS_TC      8'h01    //[7] PWR_MODE, [6:1] YG_OFFS_TC, [0] OTP_BNK_VLD
`define MPU_RA_ZG_OFFS_TC      8'h02    //[7] PWR_MODE, [6:1] ZG_OFFS_TC, [0] OTP_BNK_VLD
`define MPU_RA_X_FINE_GAIN     8'h03    //[7:0] X_FINE_GAIN
`define MPU_RA_Y_FINE_GAIN     8'h04    //[7:0] Y_FINE_GAIN
`define MPU_RA_Z_FINE_GAIN     8'h05    //[7:0] Z_FINE_GAIN
`define MPU_RA_XA_OFFS_H       8'h06    //[15:0] XA_OFFS
`define MPU_RA_XA_OFFS_L_TC    8'h07
`define MPU_RA_YA_OFFS_H       8'h08    //[15:0] YA_OFFS
`define MPU_RA_YA_OFFS_L_TC    8'h09
`define MPU_RA_ZA_OFFS_H       8'h0A    //[15:0] ZA_OFFS
`define MPU_RA_ZA_OFFS_L_TC    8'h0B
`define MPU_RA_PRODUCT_ID      8'h0C    // Product ID Register
`define MPU_RA_XG_OFFS_USRH    8'h13    //[15:0] XG_OFFS_USR
`define MPU_RA_XG_OFFS_USRL    8'h14
`define MPU_RA_YG_OFFS_USRH    8'h15    //[15:0] YG_OFFS_USR
`define MPU_RA_YG_OFFS_USRL    8'h16
`define MPU_RA_ZG_OFFS_USRH    8'h17    //[15:0] ZG_OFFS_USR
`define MPU_RA_ZG_OFFS_USRL    8'h18
`define MPU_RA_SMPLRT_DIV      8'h19
`define MPU_RA_CONFIG          8'h1A
`define MPU_RA_GYRO_CONFIG     8'h1B
`define MPU_RA_ACCEL_CONFIG    8'h1C
`define MPU_RA_FF_THR          8'h1D
`define MPU_RA_FF_DUR          8'h1E
`define MPU_RA_MOT_THR         8'h1F
`define MPU_RA_MOT_DUR         8'h20
`define MPU_RA_ZRMOT_THR       8'h21
`define MPU_RA_ZRMOT_DUR       8'h22
`define MPU_RA_FIFO_EN         8'h23
`define MPU_RA_I2C_MST_CTRL    8'h24
`define MPU_RA_I2C_SLV0_ADDR   8'h25
`define MPU_RA_I2C_SLV0_REG    8'h26
`define MPU_RA_I2C_SLV0_CTRL   8'h27
`define MPU_RA_I2C_SLV1_ADDR   8'h28
`define MPU_RA_I2C_SLV1_REG    8'h29
`define MPU_RA_I2C_SLV1_CTRL   8'h2A
`define MPU_RA_I2C_SLV2_ADDR   8'h2B
`define MPU_RA_I2C_SLV2_REG    8'h2C
`define MPU_RA_I2C_SLV2_CTRL   8'h2D
`define MPU_RA_I2C_SLV3_ADDR   8'h2E
`define MPU_RA_I2C_SLV3_REG    8'h2F
`define MPU_RA_I2C_SLV3_CTRL   8'h30
`define MPU_RA_I2C_SLV4_ADDR   8'h31
`define MPU_RA_I2C_SLV4_REG    8'h32
`define MPU_RA_I2C_SLV4_DO     8'h33
`define MPU_RA_I2C_SLV4_CTRL   8'h34
`define MPU_RA_I2C_SLV4_DI     8'h35
`define MPU_RA_I2C_MST_STATUS  8'h36
`define MPU_RA_INT_PIN_CFG     8'h37
`define MPU_RA_INT_ENABLE      8'h38
`define MPU_RA_DMP_INT_STATUS  8'h39
`define MPU_RA_INT_STATUS      8'h3A
`define MPU_RA_ACCEL_XOUT_H    8'h3B
`define MPU_RA_ACCEL_XOUT_L    8'h3C
`define MPU_RA_ACCEL_YOUT_H    8'h3D
`define MPU_RA_ACCEL_YOUT_L    8'h3E
`define MPU_RA_ACCEL_ZOUT_H    8'h3F
`define MPU_RA_ACCEL_ZOUT_L    8'h40
`define MPU_RA_TEMP_OUT_H      8'h41
`define MPU_RA_TEMP_OUT_L      8'h42
`define MPU_RA_GYRO_XOUT_H     8'h43
`define MPU_RA_GYRO_XOUT_L     8'h44
`define MPU_RA_GYRO_YOUT_H     8'h45
`define MPU_RA_GYRO_YOUT_L     8'h46
`define MPU_RA_GYRO_ZOUT_H     8'h47
`define MPU_RA_GYRO_ZOUT_L      8'h48
`define MPU_RA_EXT_SENS_DATA_00 8'h49
`define MPU_RA_MOT_DETECT_STATUS   8'h61
`define MPU_RA_I2C_SLV0_DO     8'h63
`define MPU_RA_I2C_SLV1_DO     8'h64
`define MPU_RA_I2C_SLV2_DO     8'h65
`define MPU_RA_I2C_SLV3_DO     8'h66
`define MPU_RA_I2C_MST_DELAY_CTRL  8'h67
`define MPU_RA_SIGNAL_PATH_RESET   8'h68
`define MPU_RA_MOT_DETECT_CTRL     8'h69
`define MPU_RA_USER_CTRL       8'h6A
`define MPU_RA_PWR_MGMT_1      8'h6B
`define MPU_RA_PWR_MGMT_2      8'h6C
`define MPU_RA_BANK_SEL        8'h6D
`define MPU_RA_MEM_START_ADDR  8'h6E
`define MPU_RA_MEM_R_W         8'h6F
`define MPU_RA_DMP_CFG_1       8'h70
`define MPU_RA_DMP_CFG_2       8'h71
`define MPU_RA_FIFO_COUNTH     8'h72
`define MPU_RA_FIFO_COUNTL     8'h73
`define MPU_RA_FIFO_R_W        8'h74
`define MPU_RA_WHO_AM_I        8'h75

`define MPU_RA_READ_FLAG       8'h80

`define MPU_NULL_BYTE          8'h00

// gyro_fsr
`define INV_FSR_250DPS   8'h00
`define INV_FSR_500DPS   8'h01
`define INV_FSR_1000DPS  8'h02
`define INV_FSR_2000DPS  8'h03

// F CHOICE B
`define FCB_DISABLED 8'h00
`define FCB_8800_32  8'h01
`define FCB_3600_32  8'h02

// clock_sel_e {
`define INV_CLK_INTERNAL 8'h00
`define INV_CLK_PLL 8'h01

// accel_fsr_e {
`define INV_FSR_2G 8'h00
`define INV_FSR_4G 8'h01
`define INV_FSR_8G 8'h02
`define INV_FSR_16G 8'h03

// gyroOverflow_e
`define GYRO_OVERFLOW_NONE  8'h00
`define GYRO_OVERFLOW_Xv    8'h01
`define GYRO_OVERFLOW_Y     8'h02
`define GYRO_OVERFLOW_Z     8'h03




`define MAX_GYRO_RATE 16'd1000
`define GYRO_RATE_SCALAR 16'd32767 / `MAX_GYRO_RATE
`define FIXED_SCALE_BITS 12
`define FIXED_WIDTH_BIT 64

`define TO_FIXED(val) (val << `FIXED_SCALE_BITS)
`define FROM_FIXED(val) (val >>> `FIXED_SCALE_BITS)

`define FIXED_MULT_NON_FIXED(a, b) `FROM_FIXED((a * `TO_FIXED(b))) 

`define FIXED_MULT( a, b) `FROM_FIXED((a) * (b))

`define FIXED_LITERAL(val)  (`FIXED_WIDTH_BIT'val)
