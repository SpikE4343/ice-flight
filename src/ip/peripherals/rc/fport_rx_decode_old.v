// * F-Port RC protocol decoder

//===============================================
// C struct for fport Control Frame
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
    
//===============================================    

// TODO: 
// * Block Ram for input storage
// * CRC validation and roll back
// * Parse packet payload as read from buffer

//===============================================

module fport_rx_decoder_old
#(
  parameter BASE_FREQ=16000000,
  parameter DEPTH = 32
)
(
  input reset,
  input       clock,
  input [7:0] rxData,
  input rxDataAvail,
  output reg controlFrameReady,
  output reg frameRecv,

  output debug_out,

  output reg [10:0] controls0,
  output reg [10:0] controls1,
  output reg [10:0] controls2,
  output reg [10:0] controls3,
  output reg [10:0] controls4,

  output reg [7:0] rssi,
  output reg failsafe,
  output reg rxFrameLoss
);

  localparam DEPTH_BIT = $clog2(DEPTH)-1;
  localparam PACKET_HEADER = 8'h7E;
  localparam PACKET_FOOTER = 8'h7E;
 
  localparam ST_FIND_HEADER = 4'd1;
  localparam ST_READ_HEADER = 4'd2;
  localparam ST_PARSE_HEADER = 4'd3;
  localparam ST_FIND_FOOTER = 4'd4;
  localparam ST_PARSE_DATA  = 4'd5;
  localparam ST_CONTROL_FRAME_READY = 4'd6;
  localparam ST_RESET = 4'd0;
  
  //packet_type != 8'b0 || packet_len != 8'h19

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
  
  localparam TIMEOUT_COUNT = BASE_FREQ;
  
  // TODO: Use ram instead?
  reg [7:0] controlPacket [DEPTH-1:0]; 

  reg [DEPTH_BIT:0] read_addr;
  reg [DEPTH_BIT:0] write_addr;
  reg [DEPTH_BIT:0] fifo_size;
  (* mark_debug = "true" *) reg [3:0] state;
  reg [3:0] postReadState;
  reg [7:0] data;
  reg [DEPTH_BIT:0] packet_start;
  reg [DEPTH_BIT:0] packet_end;
  reg [7:0] packet_len;
  reg [7:0] packet_type;
  reg [7:0] packet_crc;
  reg [DEPTH_BIT:0] packet_data_start;
  reg [7:0] data_read_len;
  
  reg [31:0] timeout_counter;

  reg [3:0] waitDelay;
  reg unmask;

  
  function automatic [DEPTH_BIT:0] PacketData;
    input [DEPTH_BIT:0] offset;
    begin
      PacketData = packet_data_start + offset;
    end
  endfunction

  function automatic [10:0] ReverseControlValue;
    input [10:0] value;
    begin
      ReverseControlValue[0] = value[10];
      ReverseControlValue[1] = value[9];
      ReverseControlValue[2] = value[8];
      ReverseControlValue[3] = value[7];
      ReverseControlValue[4] = value[6];
      ReverseControlValue[5] = value[5];
      ReverseControlValue[6] = value[4];
      ReverseControlValue[7] = value[3];
      ReverseControlValue[8] = value[2];
      ReverseControlValue[9] = value[1];
      ReverseControlValue[10] = value[0];
    end
  endfunction


  initial begin
    read_addr = 0;
    write_addr = 0;
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
    failsafe = 0;
    unmask = 0;
    timeout_counter = TIMEOUT_COUNT;
    waitDelay = 0;
  end



  always @(posedge rxDataAvail) begin
      if(unmask) begin
        unmask <= 0;
        controlPacket[write_addr] <= rxData ^ 8'h20;
        write_addr <= write_addr + 1;
      end else if( (data_read_len-HEADER_LEN) < packet_len  && state == ST_FIND_FOOTER && (rxData == 'h7D || rxData == 'h7E) ) begin
        // next byte is the actual data but bit masked with 8'h20 and 
        // we don't want this byte to count as valid
        unmask <= 1;
      end else begin
        unmask <= 0;
        controlPacket[write_addr] <= rxData;
        write_addr <= write_addr + 1;
      end
  end
    
  always @(posedge clock) begin
    if(reset) begin
        state <= ST_RESET;
        timeout_counter <= TIMEOUT_COUNT;
    end else begin
    
        if( timeout_counter == 0 ) begin
            state <= ST_RESET;
        end else begin
            timeout_counter <= timeout_counter - 1;
        end
        case(state)
          // ======================================
          ST_FIND_HEADER: begin
            if( read_addr != write_addr ) begin
              timeout_counter <= TIMEOUT_COUNT;
              if( controlPacket[read_addr] == PACKET_HEADER ) begin
                // keep skipping packet header byte until next byte is not 
                // the header
                packet_start <= read_addr;
                packet_data_start <= read_addr + HEADER_LEN;
                data_read_len <= 1;
              end else if( data_read_len == 1) begin 
                state <= ST_READ_HEADER;
                data_read_len <= data_read_len + 1;
              end 
    
              read_addr <= read_addr + 1;
            end
          end
    
          // ======================================
          ST_READ_HEADER: begin
            if( read_addr != write_addr ) begin
              timeout_counter <= TIMEOUT_COUNT;
              if( read_addr == packet_data_start - 5'd1 ) begin
                state <= ST_PARSE_HEADER;
              end
    
              data_read_len <= data_read_len + 1;
              read_addr <= read_addr + 1;
            end
          end
          
          // ======================================
          ST_FIND_FOOTER: begin
            if( read_addr != write_addr ) begin
              timeout_counter <= TIMEOUT_COUNT;
              if( controlPacket[read_addr] == PACKET_HEADER ) begin
                packet_end <= read_addr;
                state <= ST_PARSE_DATA;
              end else if( data_read_len > packet_len + HEADER_LEN * 2) begin
                state <= ST_RESET;
              end
    
              data_read_len <= data_read_len + 1;
              read_addr <= read_addr + 1;
    
            end
          end
          
          // ======================================
          ST_PARSE_HEADER: begin
            packet_len <= controlPacket[packet_start + LEN_OFFSET];
            packet_type <= controlPacket[packet_start + TYPE_OFFSET];
            //packet_crc <= controlPacket[ packet_start + TYPE_OFFSET + controlPacket[packet_start + LEN_OFFSET] + 1];
            state <= ST_FIND_FOOTER;
            
          end
    
          // ======================================
          ST_PARSE_DATA: begin
            waitDelay <= 4'd8;
            case(packet_type)
              FRAME_TYPE_CONTROL: begin
                if(packet_len != FRAME_LEN_CONTROL) 
                begin
                  state <= ST_RESET;
                end else begin
                  // TODO: look up generator/for - loop? method for unpacking little endian c struct:
                  
                  // Controls 1-16
                  controls0 <= 
                    { 
                      controlPacket[PacketData(1)][2:0],
                      controlPacket[PacketData(0)]
                  };
                  
                  controls1 <= { 
                    controlPacket[PacketData(2)][5:0],
                    controlPacket[PacketData(1)][7:3]
                  };
    
                  controls2 <= {
                    controlPacket[PacketData(4)][0:0],
                    controlPacket[PacketData(3)],
                    controlPacket[PacketData(2)][7:6]  
                  };
                    
                  controls3 <={ 
                    controlPacket[PacketData(5)][3:0],  
                    controlPacket[PacketData(4)][7:1]
                  };
    
                  controls4 <= { 
                    controlPacket[PacketData(6)][6:0],
                    controlPacket[PacketData(5)][7:4]
                  };
    
                    
                  // [22] Flags
                  rxFrameLoss <= controlPacket[PacketData(22)][2:2];
                  failsafe <= controlPacket[PacketData(22)][3:3];
                  
                  // [23] RSSI 
                  rssi <= controlPacket[PacketData(23)];  
                  
                  state <= ST_CONTROL_FRAME_READY;
                  controlFrameReady <= 1;
                end
              end
              
              FRAME_TYPE_DOWNLINK: begin
                state <= ST_CONTROL_FRAME_READY;
              end
    
              FRAME_TYPE_UPLINK: begin
                state <= ST_CONTROL_FRAME_READY;
              end
    
              default : begin
                state <= ST_RESET;
              end
            endcase
          end

          // ======================================
          ST_CONTROL_FRAME_READY: begin
            waitDelay <= waitDelay - 1;
            if(waitDelay == 0 ) begin
                state <= ST_RESET;
            end
          end
    
          // ======================================
          ST_RESET: begin
            state <= ST_FIND_HEADER;
            data_read_len <= 0;
            packet_len <= 0;
            packet_start <= 0;
            packet_end <= 0;
            packet_type <= 0;
            packet_data_start <= 0;
            controlFrameReady <= 0;
            frameRecv <= 0;
            timeout_counter <= TIMEOUT_COUNT;
          end
        endcase
    end
  end
 
endmodule
