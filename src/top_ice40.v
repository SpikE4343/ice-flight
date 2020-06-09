

// look in pins.pcf for all the pin names on the TinyFPGA BX board
module top 
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


flight #(

) flight_core (
  .CLK(CLK),

  .RX_IN(PIN_3),

  .IMU_CLK(PIN_4),
  .IMU_MOSI(PIN_5),
  .IMU_MISO(PIN_6),
  .IMU_CS(PIN_7),

  .DEBUG_UART_TX(PIN_12),
  .DEBUG_UART_RX(PIN_13),

  .MOTOR_1(PIN_21),
  .MOTOR_2(PIN_22),
  .MOTOR_3(PIN_23),
  .MOTOR_4(PIN_24)
  
);
  
endmodule
