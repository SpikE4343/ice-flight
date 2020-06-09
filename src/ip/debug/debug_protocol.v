

`include "debugger_defines.vh"

module debug_protocol  
(
  input clock,

  // * debug protocol message type byte to write
  input [7:0] txMsgType,

  // * debug protocol message length byte
  input [7:0] txMsgLen,
  
  // * message payload byte at index to send next
  input [7:0] txMsgData,

  // * message payload data index to load
  output [7:0] txMsgDataIndex,

  // * signal next data byte to load
  // * txMsgDataIndex: will be set to byte needed
  // * txMsgData should be set to the required byte in one clock cycle
  output txMsgDataLoad,
  
  // * raise send to 1 to begin message transmission
  input send,

  // * raise to 1 when message transmission is done
  output sendComplete,

  // * data byte out to uart
  output [7:0] uart_in,

  // * raised to load uart_in into uart
  output uartSend,

  // * listen for uart transmission completion
  input uartSendComplete
);

  localparam ST_RESET        = 0;
  localparam ST_WAIT         = 1;
  localparam ST_UART_WAIT    = 2;
  localparam ST_UART_COMPLETE_WAIT = 3;
  localparam ST_SEND_MARKER  = 4;
  localparam ST_SEND_TYPE    = 5;
  localparam ST_SEND_LENGTH  = 6;
  localparam ST_SEND_DATA    = 7;
  localparam ST_SEND_DATA_LOAD_START = 8;
  localparam ST_SEND_DATA_LOAD = 9;
  localparam ST_COMPLETE     = 100;
  
  

  reg [7:0] state;
  reg [7:0] nextState;
  reg [7:0] sentDataCount;
  reg msgComplete;
  reg msgDataLoad;
  reg uart_send;
  reg [7:0] msgDataIndex;
  reg [7:0] uart_data_in;
  reg [2:0] msgDataLoadCount;

  assign sendComplete = msgComplete;
  assign txMsgDataLoad = msgDataLoad;
  assign txMsgDataIndex = msgDataIndex;
  assign uartSend = uart_send;
  assign uart_in = uart_data_in;

  initial begin
    state = ST_RESET;
  end

   always @(posedge clock) begin
    case(state)
      // ======================================
      ST_RESET: begin
        msgComplete <= 0;
        sentDataCount <= 0;
        nextState <= 0;
        state <= ST_WAIT;
        msgDataLoad <= 0;
        msgComplete <= 0;
        msgDataIndex <= 0;
        uart_data_in <= 0;
      end

      // ======================================
      ST_WAIT: begin
        if(send)
          state <= ST_SEND_MARKER;
      end
      
      // ======================================
      ST_UART_WAIT:
      begin
        uart_send <= 0;
        if(uartSendComplete)
          state <= nextState;
      end

      ST_UART_COMPLETE_WAIT:
      begin
        uart_send <= 0;
        if(!uartSendComplete)
          state <= nextState;
      end

      // ======================================
      ST_SEND_MARKER: 
      begin
        uart_data_in <= `MSG_MARKER;
        uart_send <= 1;
        nextState <= ST_SEND_TYPE;
        state <= ST_UART_WAIT;
      end

      // ======================================
      ST_SEND_TYPE: 
      begin
        uart_data_in <= txMsgType;
        uart_send <= 1;
        nextState <= ST_SEND_LENGTH;
        state <= ST_UART_WAIT;
      end

      // ======================================
      ST_SEND_LENGTH:
      begin
        uart_data_in <= txMsgLen;
        sentDataCount <= txMsgLen;
        uart_send <= 1;
        nextState <= ST_SEND_DATA;
        state <= ST_UART_WAIT;
      end

      // ======================================
      ST_SEND_DATA:
      begin
        if( sentDataCount <= 0 ) begin
          state <= ST_COMPLETE;
          msgDataLoad <= 0;
        end else begin
          state <= ST_SEND_DATA_LOAD_START;
          msgDataIndex <= txMsgLen - sentDataCount;
        end
      end

      ST_SEND_DATA_LOAD_START:
      begin
        msgDataLoad <= 1;
        state <= ST_SEND_DATA_LOAD;
      end

      ST_SEND_DATA_LOAD:
      begin
        uart_data_in <= txMsgData;
        sentDataCount <= sentDataCount - 1;
        msgDataLoad <= 0;
        uart_send <= 1;
        nextState <= ST_SEND_DATA;
        state <= ST_UART_WAIT;
      end

      // ======================================
      ST_COMPLETE:
      begin
        msgComplete <= 1;
        msgDataLoad <= 0;
        msgDataIndex <= 0;
        state <= ST_RESET;
      end

    endcase
  end

endmodule
