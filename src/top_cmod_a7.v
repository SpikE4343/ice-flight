`timescale 1 ns / 10 ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 05/18/2020 02:07:01 PM
// Design Name: 
// Module Name: top_cmod_a7
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module top_cmod_a7(
    input sysclk,
    input pio1,
    
    output pio2,
    output pio3,
    input pio4,
    output pio5,
    
    output pio6,
    output pio7,
    output pio8,
    output pio9,
    output pio10,
    input pio11,
    
    output led0,
    output led1,
    
    input btn0,
    input btn1
    );
    
  wire clk_100mhz;
  
  cmod_a7_clk main_clock
  (
  .clk_out1(clk_100mhz),           
  .reset(reset), 
  .locked(locked),
  .clk_in1(sysclk)
  );
  
    
 flight #(
    .BASE_FREQ(100_000_000)
    ) fc (
    .CLK(clk_100mhz),
    .RX_IN(pio1),
    .IMU_SCLK(pio2),
    .IMU_MOSI(pio3),
    .IMU_MISO(pio4),
    .IMU_CS(pio5),
    
    .MOTOR_1(pio6),
    .MOTOR_2(pio7),
    .MOTOR_3(pio8),
    .MOTOR_4(pio9),
    
    .DEBUG_UART_TX(pio10),
    .DEBUG_UART_RX(pio11),

    .LED_STATUS(led0),
    .LED_ARMED(led1),
    
    .btn0(btn0),
    .btn1(btn1)
 );
endmodule
