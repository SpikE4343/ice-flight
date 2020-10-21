// * F-Port RC protocol decoder

//-----------------------------------------------

// * C struct for fport Control Frame
// typedef struct
// {
//   uint16_t channel0 : 11;
//   uint16_t channel1 : 11;
//   uint16_t channel2 : 11;
//   uint16_t channel3 : 11;
//   uint16_t channel4 : 11;
//   uint16_t channel5 : 11;
//   uint16_t channel6 : 11;
//   uint16_t channel7 : 11;
//   uint16_t channel8 : 11;
//   uint16_t channel9 : 11;
//   uint16_t channel10 : 11;
//   uint16_t channel11 : 11;
//   uint16_t channel12 : 11;
//   uint16_t channel13 : 11;
//   uint16_t channel14 : 11;
//   uint16_t channel15 : 11;
//   uint8_t flags;
//   uint8_t rssi;
// } __attribute__((__packed__)) SbusChannels_t;

// #define SBUS_MIN 192
// #define SBUS_MAX 1792
    
//-----------------------------------------------

//* TODO: 

// * CRC validation and roll back
// * Handle uplink and downlink frames
// * Parse all controls (only first 5 currently)

//----------------------------------------------- 

//* DONE:

//  * Block Ram for input storage
//  * Parse packet payload as read from buffer

//-----------------------------------------------


`define FPORT_PACKET_HEADER 8'h7E
`define FPORT_PACKET_FOOTER 8'h7E

module fport_rx_decoder
#(
  parameter BASE_FREQ=16000000,
  parameter DEPTH = 64,
  parameter TIMEOUT_COUNT=BASE_FREQ
)
(
  input reset,
  input       clock,
  input [7:0] rxData,
  input rxDataAvail,
  output reg controlFrameReady,
  output reg frameRecv,
  output reg [10:0] controls0,
  output reg [10:0] controls1,
  output reg [10:0] controls2,
  output reg [10:0] controls3,
  output reg [10:0] controls4,

  output reg [7:0] rssi,
  output reg failsafe,
  output reg rxFrameLoss,

  output [3:0] state_debug
);

  localparam DEPTH_BIT = $clog2(DEPTH)-1;
  
 
  localparam ST_FIND_MARKER_START = 4'd2;
  localparam ST_FIND_MARKER_END = 4'd3;
  localparam ST_READ_HEADER_LENGTH = 4'd4;
  localparam ST_READ_HEADER_TYPE = 4'd5;
  localparam ST_READ_PAYLOAD  = 4'd6;
  localparam ST_READ_CRC  = 4'd7;

  localparam ST_CONTROL_FRAME_READY = 4'd1;
  localparam ST_RESET = 4'd0;

  localparam LEN_OFFSET = 5'd1;
  localparam TYPE_OFFSET = 5'd2;

  // Control Frame
  localparam FRAME_TYPE_CONTROL = 8'h00;
  localparam FRAME_LEN_CONTROL = 8'h19;

  // Downlink Frame
  localparam FRAME_TYPE_DOWNLINK = 8'h01;
  localparam FRAME_LEN_DOWNLINK = 8'h08;

  // Uplink Frame
  localparam FRAME_TYPE_UPLINK = 8'h81;
  localparam FRAME_LEN_UPLINK = 8'h19;

  localparam HEADER_LEN = 5'd3;
  
  


  (* mark_debug = "true" *) reg [3:0] state;
  
  reg [7:0] packet_len;
  reg [7:0] packet_type;
  reg [7:0] packet_crc;
  
  reg [31:0] timeout_counter;

  reg [3:0] waitDelay;
  reg unmask;

  reg [7:0] packet_data_read;
  wire [7:0] fifo_write_data;
  wire [7:0] fifo_read_data;

  reg fifo_read;
  reg fifo_write;
  wire fifo_has_data;


  FIFODualPort
 #(
    .DEPTH(DEPTH),
    .DATA_WIDTH(8)
 ) inputFIFO (
   .reset(reset),
   .write_clk(rxDataAvail),
   .write(rxDataAvail),
   .write_data(fifo_write_data),

   .read_clk(clock),
   .read(fifo_read),
   .read_data(fifo_read_data),

   .dataAvailable(fifo_has_data)
 );


  assign fifo_write_data = unmask ? rxData ^ 8'h20 : rxData;
  assign state_debug = state;


  initial begin
    state = ST_RESET;
    controlFrameReady = 0;
    frameRecv = 0;
    controls0 = 0;
    controls1 = 0;
    controls2 = 0;
    controls3 = 0;
    controls4 = 0;
    rssi = 0;

    rxFrameLoss = 1;
    failsafe = 1;
    unmask = 0;
    timeout_counter = TIMEOUT_COUNT;
    waitDelay = 0;
    fifo_read = 0;
    fifo_write = 0;
    packet_data_read = 0;
  end



  always @(posedge rxDataAvail) begin
      
      if( (state == ST_READ_PAYLOAD  || state == ST_READ_CRC )&& (rxData == 'h7D || rxData == 'h7E) ) begin
        // next byte is the actual data but xor with 8'h20 and 
        // we don't want this byte to count as valid
        unmask <= 1;
      end else begin
        unmask <= 0;
      end
  end
    

  always @(posedge clock) begin
    if(reset) begin
        state <= ST_RESET;
        timeout_counter <= TIMEOUT_COUNT;
        fifo_read <= 0;
    end else begin
      if( timeout_counter == 0 ) begin
          state <= ST_RESET;
          // Signal failsafe if connection to rx has timed out
          rssi <= 0;
          failsafe <= 1;
          rxFrameLoss <= 1;
      end else begin
          timeout_counter <= timeout_counter - 1;
      end

        if(fifo_read == 1)
          fifo_read <= 0;

      case(state)
      

        // ======================================
        // Read from fifo, until read data is PACKET_HEADER value
        //
        ST_FIND_MARKER_START: begin
          if( fifo_has_data && ~fifo_read) begin
            timeout_counter <= TIMEOUT_COUNT;
            if( fifo_read_data == `FPORT_PACKET_HEADER ) begin
              state <= ST_FIND_MARKER_END;
            end

            fifo_read <= 1;
          end
        end

        // ======================================
        // Read from fifo, until read data is not PACKET_HEADER value
        //
        ST_FIND_MARKER_END: begin
          if( fifo_has_data && ~fifo_read) begin
            timeout_counter <= TIMEOUT_COUNT;
            if( fifo_read_data == `FPORT_PACKET_HEADER ) begin
              // keep skipping packet header byte until next byte is not 
              // the header
              fifo_read <= 1;
            end else begin 
              // don't pop fifo so next state can do it
              state <= ST_READ_HEADER_LENGTH;
            end 
          end
        end

        // ======================================
        // Read from fifo, until read data is not PACKET_HEADER value
        //
        ST_READ_HEADER_LENGTH: begin
          if( fifo_has_data && ~fifo_read ) begin
            timeout_counter <= TIMEOUT_COUNT;
            packet_len <= fifo_read_data;
            fifo_read <= 1;
            state <= ST_READ_HEADER_TYPE;
          end
        end

        // ======================================
        // Read packet type byte
        //
        ST_READ_HEADER_TYPE: begin
          if( fifo_has_data && ~fifo_read ) begin
            timeout_counter <= TIMEOUT_COUNT;
            fifo_read <= 1;

            // Only care about control frames for now. 
            if( fifo_read_data != FRAME_TYPE_CONTROL) begin
              state <= ST_RESET;
              frameRecv <= 1;
            end else begin
              packet_type <= fifo_read_data;
              packet_data_read <= 0;
              state <= ST_READ_PAYLOAD;
            end
            
          end
        end
  
        // ======================================
        // Read and parse all packed data
        //
        ST_READ_PAYLOAD: begin
          if(packet_data_read > packet_len) begin
              // done reading all payload data now read crc byte
              state <= ST_READ_CRC;
          end else if( fifo_has_data && ~fifo_read ) begin
            timeout_counter <= TIMEOUT_COUNT;

            packet_data_read <= packet_data_read + 1;
            
            case(packet_data_read)
              8'd00:
                begin
                  // lower byte
                  controls0[7:0] <= fifo_read_data;
                end

              8'd01:
                begin
                  // upper 2 bits of controls0
                  controls0[10:8] <= fifo_read_data[2:0];
                  
                  // lower 5 bits of controls1
                  controls1[4:0] <= fifo_read_data[7:3];
                end

              8'd02:
                begin
                  // upper 5 bits of controls1
                  controls1[10:5] <= fifo_read_data[5:0];
                  
                  // lower 2 bits of controls2
                  controls2[1:0] <= fifo_read_data[7:6];
                end

              8'd03:
                begin
                  // middle 8 bits of controls2
                  controls2[9:2] <= fifo_read_data;
                end

              8'd04:
                begin
                  // upper 1 bit of controls2
                  controls2[10] <= fifo_read_data[0];
                  controls3[6:0] <= fifo_read_data[7:1];
                end

              8'd05:
                begin
                  // upper 4 bit of controls3
                  controls3[10:7] <= fifo_read_data[3:0];
                  controls4[3:0] <= fifo_read_data[7:4];
                end

              8'd06:
                begin
                  // upper 4 bit of controls3
                  controls4[10:4] <= fifo_read_data[6:0];
                end

              // TODO: add rest of controls
              8'd22:
                begin
                  // upper 4 bit of controls3
                  rxFrameLoss <= fifo_read_data[2];
                  failsafe <= fifo_read_data[3];
                end

              8'd23:
                begin
                  rssi <= fifo_read_data;
                end

              default: 
                begin
                  // skip
                end
            endcase

            fifo_read <= 1;
          end
        end
        
        // ======================================
        // Read crc byte
        //
        ST_READ_CRC: begin
          if( fifo_has_data && ~fifo_read ) begin
            timeout_counter <= TIMEOUT_COUNT;
            packet_crc <= fifo_read_data;
            // TODO: reset if crc doesn't match
            fifo_read <= 1;
            state <= ST_CONTROL_FRAME_READY;
            controlFrameReady <= 1;
            frameRecv <= 1;
            waitDelay <= 16'd1;
          end
        end
  
        // ======================================
        // Signal new control data available
        // 
        ST_CONTROL_FRAME_READY: begin
          if(waitDelay == 16'd0 ) begin
              state <= ST_RESET;
          end
          waitDelay <= waitDelay - 16'd1;
        end
  
        // ======================================
        // Start the whole thing over
        //
        ST_RESET: begin
          state <= ST_FIND_MARKER_START;
          packet_len <= 0;
          packet_type <= 0;
          controlFrameReady <= 0;
          frameRecv <= 0;
          timeout_counter <= TIMEOUT_COUNT;
          packet_data_read = 0;
        end
      endcase
    end
  end
 
endmodule
