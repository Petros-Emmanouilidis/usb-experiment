`default_nettype none

// module tb();

//     // DUT instantiation
//     logic clock, reset_n, read, write;
//     logic [1:0] wires_in, wires_out, status;
//     logic [3:0] data_in, data_out, mode, data_indx;

//     USBHost DUT(.*);


//     // Testbench transmitter instantiation
//     logic send_packet, send_done;
//     logic [3:0] send_PID, send_ENDP;
//     logic [6:0] send_Addr;
//     logic [63:0] send_Payload;

//     PACKET_SEND test_transmitter(.PID(send_PID), .ENDP(send_ENDP), 
//                                  .Addr(send_Addr), .Payload(send_Payload), 
//                                  .wires_out(wires_in), .*);


//     // Testbench receiver instantiation
//     logic expect_packet, receive_done, had_errors;
//     logic [3:0] recd_PID, recd_ENDP;
//     logic [6:0] recd_Addr;
//     logic [63:0] recd_Payload;

//     PACKET_RECEIVE test_receiver(.PID(recd_PID), .ENDP(recd_ENDP), 
//                                  .Addr(recd_Addr), .Payload(recd_Payload), 
//                                  .wires_in(wires_out), .*);


//     initial forever #5 clock = ~clock;

//     initial begin

//         // Setup
//         clock = 0; reset_n = 1; read = 0; write = 0;
//         mode = '0; data_in = '0;
//         expect_packet = 0; send_packet = 0;

//         // Reset device
//         reset_n <= 0;
//         #1 reset_n <= 1;

//         // Input data

//         // Ensure data is correct

//         // Initiate write

//         /*
//         EXPECT *OUT*
//         PID: 1, Addr: 5, ENDP: 4, Payload: x, CRC5: 10, CRC16: x

//         EXPECT *DATA0*
//         PID: 3, Addr: x, ENDP: x, Payload: hf21000000000000, CRC5: x, CRC16: a0e7

//         SEND *ACK*
//         PID: 2, Addr: x, ENDP: x, Payload: x, CRC5: x, CRC16: x

//         EXPECT *OUT*
//         PID: 1, Addr: 5, ENDP: 8, Payload: x, CRC5: he, CRC16: x

//         EXPECT *DATA0*
//         PID: 3, Addr: x, ENDP: x, Payload: h40aa11b7682df6d8, CRC5: x, CRC16: 544a

//         SEND *ACK*
//         PID: 2, Addr: x, ENDP: x, Payload: x, CRC5: x, CRC16: x
//         */

//         // Completed write


//         // Initiate read

//         /*
//         EXPECT *OUT*
//         PID: 1, Addr: 5, ENDP: 4, Payload: x, CRC5: 10, CRC16: x

//         EXPECT *DATA0*
//         PID: 3, Addr: x, ENDP: x, Payload: hf21000000000000, CRC5: x, CRC16: a0e7

//         SEND *ACK*
//         PID: 2, Addr: x, ENDP: x, Payload: x, CRC5: x, CRC16: x

//         EXPECT *IN*
//         PID: 1, Addr: 9, ENDP: 8, Payload: x, CRC5: 0e, CRC16: x

//         SEND *DATA0*
//         PID: 3, Addr: x, ENDP: x, Payload: h40aa11b7682df6d8, CRC5: x, CRC16: 544a 

//         EXPECT *ACK*
//         PID: 2, Addr: x, ENDP: x, Payload: x, CRC5: x, CRC16: x

//         */

//         // Output received data

//         $finish(0);

//     end

// endmodule: tb

typedef enum logic [3:0] {
        PID_OUT = 4'b0001, PID_IN = 4'b1001, PID_DATA0 = 4'b0011,
        PID_ACK = 4'b0010, PID_NAK = 4'b1010, PID_X = 4'bxxxx
    } pid_t;

module USBHost (
  input  logic clock, reset_n,
  input  logic read, write, 
  input  logic [3:0] mode, data_in,
  input  logic [1:0] wires_in,

  output logic [1:0] wires_out,
  output logic [1:0] status,
  output logic [3:0] data_indx, data_out
);



    logic send_packet, expect_packet;
    logic send_done, receive_done, had_errors, addr_or_data;

    logic [3:0] send_PID, send_ENDP;
    logic [6:0] send_Addr;
    logic [63:0] send_payload;

    logic [3:0] received_PID, received_ENDP;
    logic [6:0] received_Addr;
    logic [63:0] received_payload; 

    logic make_in, make_out, do_read, do_write;
    logic txn_done, txn_error;

    logic finished, finished_with_error;

    logic [63:0] final_data, out_data, in_data, memory_data;
    logic [15:0] memory_address;


    logic is_idle, data_received, en_interface, no_transaction;
    logic [3:0] data_ENDP, addr_ENDP;


    always_comb begin
        data_received = finished;
        en_interface  = no_transaction;

        if (is_idle && no_transaction)              status = 2'b00;
        else if (finished && finished_with_error)   status = 2'b01;
        else if (finished && ~finished_with_error)  status = 2'b10;
        else                                        status = 2'b11;

        do_read     = is_idle && no_transaction && read;
        do_write    = is_idle && no_transaction && write;
    end

    HOST_INTERFACE inter(.en(en_interface), .mempage(memory_address), .memdata(memory_data), .*);

    PACKET_SEND transmitter(.PID(send_PID), .ENDP(send_ENDP), 
                            .Addr(send_Addr), .Payload(send_payload), .*);

    PACKET_RECEIVE receiver(.PID(received_PID), .ENDP(received_ENDP),
                            .Addr(received_Addr), .Payload(received_payload), .*);

    IN_OUT_HANDLER io_fsm(.make_read(make_in), .make_write(make_out),
                          .received_data(received_payload), .send_data(out_data), 
                          .completed_transaction(txn_done), .ended_with_errors(txn_error),
                          .final_data(in_data), 
                          .PID_to_sender(send_PID), .data_to_sender(send_payload), .ENDP_to_sender(send_ENDP), .*); 

    READ_WRITE_HANDLER rw_fsm(.mempage(memory_address), .memdata(memory_data), .*);
endmodule : USBHost


module HOST_INTERFACE
    (input  logic clock, reset_n,

     input  logic [3:0] mode, data_in,
     output logic is_idle,
     output logic [3:0] data_indx, data_out,


     input  logic data_received, en,
     input  logic [63:0] final_data,
     output logic [3:0] data_ENDP, addr_ENDP,
     output logic [6:0] send_Addr,
     output logic [15:0] mempage,
     output logic [63:0] memdata   
);

    logic [7:0]  ENDP_reg, ENDP_reg_next;
    logic [7:0]  Addr_reg, Addr_reg_next;
    logic [15:0] mempage_reg, mempage_reg_next;
    logic [63:0] data_in_reg, data_out_reg, data_out_reg_next;

    logic [3:0]  count, count_next;

    logic [3:0]  data_in_hb, data_out_hb, misc_hb;


    // ENUMERATION:
    /*
        0 -> Do nothing
        1 -> Set Address
        2 -> Set ENDP
        3 -> Set Memory Address
        4 -> Set Data to write
        5 -> Output received data
        6 -> Output data to send
        7 -> Output {Addr, ENDP of Addr, ENDP of Data, Mempage}
    */

    enum logic [3:0] {
        IDLE         = 4'd0,    SET_ADDR    = 4'd1, 
        SET_ENDP     = 4'd2,    SET_MEMPAGE = 4'd3,
        SET_DATA     = 4'd4,    OUT_DATA_IN = 4'd5,
        OUT_DATA_OUT = 4'd6,    OUT_ALL     = 4'd7
        
    } cur_state, next_state;

        always_comb begin
        data_ENDP = ENDP_reg[3:0];
        addr_ENDP = ENDP_reg[7:4];
        send_Addr = Addr_reg[6:0];
        mempage   = mempage_reg;
        memdata   = data_out_reg;
        is_idle   = cur_state == IDLE;
    end


    HALFBYTE_PICKER_64 in_hb(.in_reg(data_in_reg), .index(count), .halfbyte(data_in_hb));
    HALFBYTE_PICKER_64 out_hb(.in_reg(data_out_reg), .index(count), .halfbyte(data_out_hb));
    HALFBYTE_PICKER_32 msc_hb(.in_reg({Addr_reg, ENDP_reg, mempage_reg}), .index(count), .halfbyte(misc_hb)); 

    always_comb begin

        ENDP_reg_next = ENDP_reg;
        Addr_reg_next = Addr_reg;
        mempage_reg_next = mempage_reg;
        data_out_reg_next = data_out_reg;

        data_indx = '0;
        data_out = '0;

        case (cur_state) 
            IDLE: begin
                count_next = '0;
                if (en) begin
                    case (mode)
                        4'd0: next_state = IDLE;
                        4'd1: next_state = SET_ADDR;
                        4'd2: next_state = SET_ENDP;
                        4'd3: next_state = SET_MEMPAGE;
                        4'd4: next_state = SET_DATA;
                        4'd5: next_state = OUT_DATA_IN;
                        4'd6: next_state = OUT_DATA_OUT;
                        4'd7: next_state = OUT_ALL;
                        default: next_state = IDLE;
                    endcase
                end
                else next_state = IDLE;
            end
            SET_ENDP: begin
                count_next = (count == 4'd1) ? 4'd0 : count + 1;
                next_state = (count == 4'd1) ? IDLE : SET_ENDP;
                ENDP_reg_next = (ENDP_reg << 4) + data_in;
            end
            SET_ADDR: begin
                count_next = (count == 4'd1) ? 4'd0 : count + 1;
                next_state = (count == 4'd1) ? IDLE : SET_ADDR;
                Addr_reg_next = (Addr_reg << 4) + data_in;
            end
            SET_MEMPAGE: begin
                count_next = (count == 4'd3) ? 4'd0 : count + 1;
                next_state = (count == 4'd3) ? IDLE : SET_MEMPAGE;
                mempage_reg_next = (mempage_reg << 4) + data_in;
            end
            SET_DATA: begin
                count_next = (count == 4'd15) ? 4'd0 : count + 1;
                next_state = (count == 4'd15) ? IDLE : SET_DATA;
                data_out_reg_next = (data_out_reg << 4) + data_in;

            end
            OUT_DATA_IN: begin
                count_next = (count == 4'd15) ? 4'd0 : count + 1;
                next_state = (count == 4'd15) ? IDLE : OUT_DATA_IN;
                data_indx  = count;
                data_out   = data_in_hb;       
            end
            OUT_DATA_OUT: begin
                count_next = (count == 4'd15) ? 4'd0 : count + 1;
                next_state = (count == 4'd15) ? IDLE : OUT_DATA_OUT;
                data_indx  = count;
                data_out   = data_out_hb;
            end
            OUT_ALL: begin
                count_next = (count == 4'd7) ? 4'd0 : count + 1;
                next_state = (count == 4'd7) ? IDLE : OUT_ALL;
                data_indx  = count;
                data_out   = misc_hb;
            end
            default: begin
                count_next = '0;
                next_state = IDLE;
                data_indx  = count;
                data_out   = '0;
            end
        endcase
    end
    
    
    always_ff @(posedge clock, negedge reset_n) begin
        if (~reset_n) cur_state <= IDLE;
        else          cur_state <= next_state;
    end

    always_ff @(posedge clock, negedge reset_n) begin
        if (~reset_n) begin
            ENDP_reg <= '0;
            Addr_reg <= '0;
            mempage_reg <= '0;
            data_out_reg <= '0;
        end
        else begin
            ENDP_reg <= ENDP_reg_next;
            Addr_reg <= Addr_reg_next;
            mempage_reg <= mempage_reg_next;
            data_out_reg <= data_out_reg_next;
        end
    end

    always_ff @(posedge clock, negedge reset_n) begin
        if (~reset_n) data_in_reg <= '0;
        else data_in_reg <= (data_received) ? final_data : data_in_reg;
    end

    always_ff @(posedge clock, negedge reset_n) begin
        if (~reset_n) count <= '0;
        else count <= count_next;
    end
endmodule: HOST_INTERFACE

module HALFBYTE_PICKER_64 
(
        input  logic [63:0] in_reg,
        input  logic [3:0] index,
        output logic [3:0] halfbyte
);

    always_comb begin
        case (index) 
            4'd0:  halfbyte = in_reg[3:0];
            4'd1:  halfbyte = in_reg[7:4];
            4'd2:  halfbyte = in_reg[11:8];
            4'd3:  halfbyte = in_reg[15:12];
            4'd4:  halfbyte = in_reg[19:16];
            4'd5:  halfbyte = in_reg[23:20];
            4'd6:  halfbyte = in_reg[27:24];
            4'd7:  halfbyte = in_reg[31:28];
            4'd8:  halfbyte = in_reg[35:32];
            4'd9:  halfbyte = in_reg[39:36];
            4'd10: halfbyte = in_reg[43:40];
            4'd11: halfbyte = in_reg[47:44];
            4'd12: halfbyte = in_reg[51:48];
            4'd13: halfbyte = in_reg[55:52];
            4'd14: halfbyte = in_reg[59:56];
            4'd15: halfbyte = in_reg[63:60];
            default: halfbyte = '0;
        endcase
    end
endmodule: HALFBYTE_PICKER_64


module HALFBYTE_PICKER_32
(
        input  logic [31:0] in_reg,
        input  logic [3:0] index,
        output logic [3:0] halfbyte
);

    always_comb begin
        case (index) 
            4'd0:  halfbyte = in_reg[3:0];
            4'd1:  halfbyte = in_reg[7:4];
            4'd2:  halfbyte = in_reg[11:8];
            4'd3:  halfbyte = in_reg[15:12];
            4'd4:  halfbyte = in_reg[19:16];
            4'd5:  halfbyte = in_reg[23:20];
            4'd6:  halfbyte = in_reg[27:24];
            4'd7:  halfbyte = in_reg[31:28];
            default: halfbyte = '0;
        endcase
    end
endmodule: HALFBYTE_PICKER_32

// Main hardware thread for sending packets.
// All submodules are instantiated and connected in here
module PACKET_SEND (
    input  logic clock, reset_n,
    input  logic send_packet,
    input  logic [3:0] PID, ENDP,
    input  logic [6:0] Addr,
    input  logic [63:0] Payload,
    output logic send_done,

    output logic [1:0] wires_out
);

    // A bunch of status and control points the Packet Send Manager
    // fsm takes in an uses to orchestrate packet sending
    logic pause, encoder_or_crc, bit_to_stuff;
    logic crc_select, crc_init, stuff_en, nrzi_en;
    logic wire_en, is_eop, packet_pause, crc_pause, hard_init;
    logic packet_out, crc_out, stuff_out, nrzi_out, nrzi_init; 

    logic [6:0] crc_index, packet_index;
    logic [1:0] packet_phase;

    assign bit_to_stuff = encoder_or_crc ? packet_out : crc_out;

    // Quite literally every module needed to send a packet
    PACKET_SEND_MANAGER fsm(.*);

    PACKET_ENCODER encoder(.pause(packet_pause), .index(packet_index), 
                           .phase(packet_phase), .bit_out(packet_out), .*);

    CRC_DRIVER crc(.bit_in(packet_out), .select_crc(crc_select), .init(crc_init), 
                   .pause(crc_pause), .index(crc_index), .bit_out(crc_out), .*);

    BIT_STUFFER bit_stuff(.bit_in(bit_to_stuff), .en(stuff_en), .bit_out(stuff_out), 
                          .is_stuffing(pause), .*);

    NRZI nrzi(.bit_in(stuff_out), .en(nrzi_en), .init(nrzi_init), .bit_out(nrzi_out), .*);

    WIRE_DRIVER out_wire(.bit_in(nrzi_out), .en(wire_en), .*);
endmodule: PACKET_SEND


// Module used for CRC5
module CRC5 (
  input  logic clock, reset_n,
  input  logic bit_in, en, init,
  output logic [4:0] crc5_out,
  output logic residue_match
);
  
  logic x1; 
  assign x1 = bit_in ^ crc5_out[4];

  // Used when receiving packets
  assign residue_match = (crc5_out == 5'b01100);

  // Logic based on lecture
  always_ff @(posedge clock, negedge reset_n) begin
    if (~reset_n) for (int i = 0; i < 5; i++) crc5_out[i] <= '1;
    else begin
        if (init) for (int i = 0; i < 5; i++) crc5_out[i] <= '1;
        else begin
            crc5_out[0] <= en ? x1 : crc5_out[0];
            crc5_out[2] <= en ? crc5_out[1] ^ x1 : crc5_out[2];

            crc5_out[1] <= en ? crc5_out[0] : crc5_out[1];
            crc5_out[3] <= en ? crc5_out[2] : crc5_out[3];
            crc5_out[4] <= en ? crc5_out[3] : crc5_out[4];
        end
    end
  end
endmodule: CRC5


// Very similar to CRC5, this is a
// CRC16 module
module CRC16 (
    input  logic clock, reset_n,
    input  logic bit_in, en, init,
    output logic [15:0] crc16_out,
    output logic residue_match 
);

    logic x1;
    assign x1 = bit_in ^ crc16_out[15];

    assign residue_match = (crc16_out == 16'h800D);

    // Logic is extremely similar to that of CRC5. XORs are
    // placed in appropriate positions given the polynomial.
    always_ff @(posedge clock, negedge reset_n) begin
        if (~reset_n) for (int i = 0; i < 16; i++) crc16_out[i] <= '1;  
        else begin
            if (init) for (int i = 0; i < 16; i++) crc16_out[i] <= '1;
            else begin
                crc16_out[0] <= en ? x1 : crc16_out[0];
                crc16_out[2] <= en ? crc16_out[1] ^ x1 : crc16_out[2];
                crc16_out[15] <= en ? crc16_out[14] ^ x1 : crc16_out[15];

                crc16_out[1] <= en ? crc16_out[0] : crc16_out[1];
                crc16_out[3] <= en ? crc16_out[2] : crc16_out[3];
                crc16_out[4] <= en ? crc16_out[3] : crc16_out[4];
                crc16_out[5] <= en ? crc16_out[4] : crc16_out[5];
                crc16_out[6] <= en ? crc16_out[5] : crc16_out[6];
                crc16_out[7] <= en ? crc16_out[6] : crc16_out[7];
                crc16_out[8] <= en ? crc16_out[7] : crc16_out[8];
                crc16_out[9] <= en ? crc16_out[8] : crc16_out[9];
                crc16_out[10] <= en ? crc16_out[9] : crc16_out[10];
                crc16_out[11] <= en ? crc16_out[10] : crc16_out[11];
                crc16_out[12] <= en ? crc16_out[11] : crc16_out[12];
                crc16_out[13] <= en ? crc16_out[12] : crc16_out[13];
                crc16_out[14] <= en ? crc16_out[13] : crc16_out[14];
            end
        end
    end
endmodule: CRC16


// A wrapper module that abstracts the process of 
// CRC encoding. Used when sending packages
module CRC_DRIVER (
    input  logic clock, reset_n,
    input  logic select_crc, init, pause, bit_in,
    input  logic [6:0] index,
    output logic bit_out
);

    logic en, residue_match5, residue_match16;
    assign en = ~pause;

    logic [4:0] crc5_out, crc5_out_inv;
    logic [15:0] crc16_out, crc16_out_inv;

    always_comb begin
        for (int i = 0; i < 5; i++) crc5_out_inv[i] = ~crc5_out[i];
        for (int j = 0; j < 16; j++) crc16_out_inv[j] = ~crc16_out[j];
    end

    // The driver selects between CRC5 and CRC16 based on the FSM's needs
    CRC5  crc5(.residue_match(residue_match5), .*);
    CRC16 crc16(.residue_match(residue_match16), .*);

    // The driver takes in an index from the FSM in order to select which CRC bit it will
    // output. 
    assign bit_out = select_crc ? crc5_out_inv[4 - index] : crc16_out_inv[15 - index];
endmodule: CRC_DRIVER


// The overarching FSM used to send a packet. This guy
// orchestrates the movements of all other modules used in 
// the process. 
module PACKET_SEND_MANAGER (
    input  logic clock, reset_n,
    input  logic pause, send_packet,
    input  logic [3:0] PID,
    output logic encoder_or_crc,
    output logic crc_select, crc_init, hard_init, 
    output logic stuff_en, nrzi_en, nrzi_init, wire_en, is_eop,
    output logic packet_pause, crc_pause,
    output logic [6:0] packet_index, crc_index,
    output logic [1:0] packet_phase,
    output logic send_done
);

    // This dude's states represent the different segments of a USB
    // packet. That includes SYNC, PID, Address and ENDP, CRC5 or CRC16,
    // Data Payload, and EOP. An IDLE state is included since we are not always
    // sending a packet 
    enum logic [2:0] {IDLE = 3'b000, SYNC = 3'b001, 
                      PID_STATE = 3'd2, ADDR_ENDP = 3'd3, 
                      CRC5 = 3'd4, PAYLOAD = 3'd5, CRC16 = 3'd6, EOP = 3'd7} cur_state, next_state;

    logic [6:0] count, count_next;



    // Next state combinational logic
    always_comb begin
        case (cur_state)

            // We only escape idle when asked to send a packet
            IDLE: begin
                next_state = send_packet ? SYNC : IDLE;
                count_next = '0;
            end
            // We always transmit PID after sync
            SYNC: begin
                next_state = count == 7'd7 ? PID_STATE : SYNC;
                count_next = count == 7'd7 ? '0 : count + 1;
            end
            // After PID, we can transition to many states
            // depending on what the value of that PID actually was.
            PID_STATE: begin
                if (count == 7'd7) begin
                    count_next = '0;
                    if (PID == PID_IN || PID == PID_OUT) next_state = ADDR_ENDP;
                    else if (PID == PID_DATA0) next_state = PAYLOAD;
                    else next_state = EOP;
                end
                else begin
                    next_state = PID_STATE;
                    count_next = count + 1;
                end
            end

            // The following state transitions are fairly linear:
            // Address and Endpoint are followed by CRC5 and EOP
            // Payload is followed by CRC16 and then EOP

            // Make sure, however, that we don't transition while paused.
            // This can lead to many many bugs. 
            ADDR_ENDP: begin
                next_state = count == 7'd10 && ~pause ? CRC5 : ADDR_ENDP;
                if (pause) count_next = count;
                else count_next = count == 7'd10 ? '0 : count + 1;
            end
            CRC5: begin
                next_state = count == 7'd4 && ~pause ? EOP : CRC5;
                if (pause) count_next = count;
                else count_next = count == 7'd4 ? '0 : count + 1;
            end
            CRC16: begin
                next_state = count == 7'd15 && ~pause ? EOP : CRC16;
                if (pause) count_next = count;
                else count_next = count == 7'd15 ? '0 : count + 1;
            end
            PAYLOAD: begin
                next_state = count == 7'd63 && ~pause ? CRC16 : PAYLOAD;
                if (pause) count_next = count;
                else count_next = count == 7'd63 ? '0 : count + 1;
            end
            // After EOP we return to idle
            EOP: begin
                next_state = count == 7'd2 ? IDLE : EOP;
                if (pause) count_next = count;
                else count_next = count == 7'd2 ? '0 : count + 1;
            end
        endcase
    end

    // Combinational logic used to flicker and read
    // control and status points respectively. 
    always_comb begin

        // Default values for controls
        encoder_or_crc = '1;
        packet_index = '0; packet_phase = '0;
        crc_index = '0; crc_select = '0; crc_init = '1;
        stuff_en = '0; nrzi_en = '0; nrzi_init = '0;
        wire_en = '0; is_eop = '0; 
        send_done = '0; hard_init = '0;

        packet_pause = pause;
        crc_pause = pause;

        case (cur_state)
            // When IDLE, everything should be off. We can keep our CRC FFs at
            // 1 (inited)
            IDLE: begin
                packet_index = '0; 
                crc_index = '0; crc_select = '0; crc_init = '1;
                stuff_en = '0; nrzi_en = '0; wire_en = '0; packet_phase = '0;
                nrzi_init = '1;
                hard_init = '1;
            end
            // During SYNC, ask the encoder for some SYNC bits. Bit stuffing
            // should be off and CRC should not listen to the SYNC bits
            SYNC: begin
                packet_index = count; packet_phase = 2'b11; 
                encoder_or_crc = '1;

                crc_pause = '1; crc_init = '1;
                stuff_en = '0; nrzi_en = '1; wire_en = '1;
            end
            // During PID, ask encoder for some PID bits. We don't bit
            // stuff this guy either, and the CRC also ignores him
            PID_STATE: begin
                packet_index = count; packet_phase = '0;
                encoder_or_crc = '1;

                crc_pause = '1; crc_init = '1;
                stuff_en = '0; nrzi_en = '1; wire_en = '1;
            end
            // Ok, this time we want CRC to start taking in values;
            // we also want those bits to be potentially bit stuffed
            ADDR_ENDP: begin
                packet_index = count; packet_phase = 2'b01;
                encoder_or_crc = '1; crc_init = '0;

                stuff_en = '1; nrzi_en = '1; wire_en = '1;
            end
            // Same here
            PAYLOAD: begin
                packet_index = count; packet_phase = 2'b10;
                encoder_or_crc = '1; crc_init = '0;

                stuff_en = '1; nrzi_en = '1; wire_en = '1;
            end
            // Now it is time for the CRC driver to speak and 
            // the packet encoder to shut up (his time in the
            // spotlight is over). Choose the CRC driver over
            // the Packet Encoder and make sure to choose
            // CRC5
            CRC5: begin
                crc_index = count; 
                encoder_or_crc = '0; crc_init = '0; 
                crc_pause = '1; crc_select = '1;

                stuff_en = '1; nrzi_en = '1; wire_en = '1;

            end
            // Same as previous state, this time we need to choose
            // CRC16 instead.
            CRC16: begin
                crc_index = count;
                encoder_or_crc = '0; crc_init = '0; 
                crc_pause = '1; crc_select = '0;

                stuff_en = '1; nrzi_en = '1; wire_en = '1;
            end
            // EOP is handled by the wire driver itself. As a result, we activate
            // that guy and switch off everything else
            EOP: begin
                wire_en = '1; is_eop = ~pause; 
                stuff_en = count == '0;
                send_done = count == 7'd2;
                crc_init = '1;
                hard_init = '1;
            end
        endcase
    end


    // State update
    always_ff @(posedge clock, negedge reset_n) begin
        if (~reset_n) cur_state <= IDLE;
        else cur_state <= next_state;
    end

    // Counter update
    always_ff @(posedge clock, negedge reset_n) begin
        if (~reset_n) count <= '0;
        else count <= count_next;
    end
endmodule: PACKET_SEND_MANAGER


// This guy holds the answers combinationally for the entire packet. It takes in
// field values and encodes them in appropriate format.

// The phase input is used to select between fields
// Encoding for phase:
//
// 00 -> PID 
// 01 -> ADDR_ENDP 
// 10 -> PAYLOAD
// 11 -> SYNC
module PACKET_ENCODER (
    input  logic clock, reset_n,
    input  logic pause, 
    input  logic [3:0] PID,
    input  logic [6:0] Addr,
    input  logic [3:0] ENDP,
    input  logic [63:0] Payload,
    input  logic [6:0] index,
    input  logic [1:0] phase,  
    output logic bit_out
);

    logic [95:0] data_register;
    logic [15:0] acknak_register;
    logic [26:0] inout_register;

    logic [3:0] PID_lsb, PID_lsb_inv, ENDP_lsb;
    logic [6:0] Addr_lsb;
    logic [63:0] Payload_lsb;

    logic [7:0] SYNC, PID_full;
    logic [10:0] Addr_Endp_register;

    // Just some combinational formatting of packet fields.
    always_comb begin
        for (int i = 0; i < 4; i++) begin
            PID_lsb[i] = PID[3-i]; 
            PID_lsb_inv[i] = ~PID[3-i];
            ENDP_lsb[i] = ENDP[3-i];
        end
        for (int j = 0; j < 7; j++) Addr_lsb[j] = Addr[6-j];
        for (int k = 0; k < 64; k++) Payload_lsb[k] = Payload[63-k];

        SYNC = 8'b0000_0001;
        PID_full = {PID_lsb, PID_lsb_inv};
        Addr_Endp_register = {Addr_lsb, ENDP_lsb};
    end

    // This is here for debugging, in case I want to look at the entire fields
    always_comb begin
        inout_register = '0; data_register = '0; acknak_register = '0;

        if (PID == PID_IN || PID == PID_OUT) begin
            inout_register = {SYNC, PID_lsb, PID_lsb_inv, Addr_lsb, ENDP_lsb};
        end
        else if (PID == PID_DATA0) begin
            data_register = {SYNC, PID_lsb, PID_lsb_inv, Payload_lsb};
        end
        else if (PID == PID_ACK || PID == PID_NAK) begin
            acknak_register = {SYNC, PID_lsb, PID_lsb_inv};
        end
        else begin
            inout_register = '0;
            data_register = '0;
            acknak_register = '0;
        end
    end

    // Combinational logic to select between packet fields
    // and output bit for selected field
    always_comb begin
        if (phase == 2'b00) begin
            bit_out = PID_full[7-index];
        end
        else if (phase == 2'b01) begin
            bit_out = Addr_Endp_register[10-index];
        end
        else if (phase == 2'b10) begin
            bit_out = Payload_lsb[63-index];
        end
        else begin
            bit_out = SYNC[7-index];
        end
    end
endmodule: PACKET_ENCODER


// This guy stuffs in a zero everytime he spots 
// 6 consequtive 1s. He lets others know when he is
// outputting stuffing

// When other modules see that this guy is throwing out
// stuffing, the bit stream pauses for one clock
// cycle so that no bit is lost during the stuffing
// period
module BIT_STUFFER (
    input  logic bit_in, en, clock, reset_n, hard_init,
    output logic bit_out, is_stuffing
);

    logic [2:0] count, count_next;

    // Combinational logic for next state of counter
    // and output of bit stuffer
    always_comb begin
        if (count == 3'd6) begin
            bit_out     = '0;
            count_next  = '0;
            is_stuffing = '1;
        end
        else begin
            bit_out     = bit_in;
            count_next  = bit_in ? count + 1 : '0;
            is_stuffing = '0;
        end
    end

    // Really the bitstuffer is just a counter
    always_ff @(posedge clock, negedge reset_n) begin
        if (~reset_n) count <= '0;
        else if (hard_init) count <= '0;
        else count <= en ? count_next : count;
    end
endmodule: BIT_STUFFER


// Again, a very simple design
// Takes in a bit and outputs either its internal
// state or its compliment based on the value of
// bit_in. Outputs the same value when disabled
module NRZI (
    input  logic bit_in, en, init, clock, reset_n,
    output logic bit_out
);

    logic cur_value, cur_value_next;

    assign bit_out = bit_in ? cur_value : ~cur_value;
    assign cur_value_next = bit_out;

    // Initial value of internal state starts out as a 1
    always_ff @(posedge clock, negedge reset_n) begin
        if (~reset_n) cur_value <= '1;
        else if (init) cur_value <= '1;
        else cur_value <= en ? cur_value_next : cur_value;
    end
endmodule: NRZI

// Last module in the packet centipede, this guy
// converts every bit he receives into a tri-state
// output to be send away. He also gets an internal 
// FSM because he handles EOP by himself like a big boy

// The en input is used to convey idleness while the is_eop
// input initiates the start of an EOP
module WIRE_DRIVER (
    output logic [1:0] wires_out,

    input  logic clock, reset_n, 
    input  logic bit_in, is_eop, en
);

    enum logic [1:0] {IDLE = '0, SENDING = 2'b01, EOP1 = 2'b10, EOP2 = '1} cur_state, next_state;
    logic drive_dp, drive_dm;

    // Wires out logic
    assign wires_out[1] = en ? drive_dp : '1;
    assign wires_out[0] = en ? drive_dm : '0;

    // Next state transition logic
    // We use EOP1 and EOP2 transitions to replicate the 
    // SE0 SE0 J convention
    always_comb begin
        if (~en) begin
            next_state = IDLE;
        end
        else begin
            if      (cur_state == IDLE) next_state = SENDING;
            else if (cur_state == SENDING) next_state = is_eop ? EOP1 : SENDING;
            else if (cur_state == EOP1) next_state = EOP2;
            else next_state = IDLE;
        end
    end

    // Setting values of DP and DM
    // according to state. 
    always_comb begin
        case (cur_state)
            IDLE: begin
                if (en) begin
                    drive_dp = '0;
                    drive_dm = '1;
                end
                else begin
                    drive_dp = '1;
                    drive_dm = '1;
                end
            end
            SENDING: begin
                if (~is_eop) begin
                    if (bit_in) begin
                        drive_dp = '1;
                        drive_dm = '0;
                    end
                    else begin
                        drive_dp = '0;
                        drive_dm = '1;
                    end
                end
                else begin
                    drive_dp = '0;
                    drive_dm = '0;
                end
            end
            EOP1: begin
                drive_dp = '0;
                drive_dm = '0;
            end
            EOP2: begin
                drive_dp = '1;
                drive_dm = '0;
            end
            default: begin
                drive_dp = '1;
                drive_dm = '1;
            end
        endcase
    end

    // State transition FF
    always_ff @(posedge clock, negedge reset_n) begin
        if (~reset_n) cur_state <= IDLE;
        else cur_state <= en ? next_state : cur_state;
    end
endmodule: WIRE_DRIVER


// This is a big boy FSM that handles USB IN and OUT
// requests. It gets a bunch of signals coming from the 
// sender and receiver that let it know when packets have
// been sent received and what was received.

// It also commands the transmitte and receiver in order
// to actually send the appropriate packages. 

// Its state logic encapsulates the way an IN or OUT 
// transaction is meant to go about, error correction
// included. 

// It reports the results of a finished transaction to a 
// bigger boy FSM
module IN_OUT_HANDLER 
    (input  logic clock, reset_n,
     input  logic make_read, make_write,
     input  logic send_done, receive_done,
     input  logic had_errors,
     input  logic [3:0] received_PID, addr_ENDP, data_ENDP,
     input  logic [63:0] received_data, send_data,
     input  logic addr_or_data,

     output logic completed_transaction, ended_with_errors,
     output logic send_packet, expect_packet,
     output logic [63:0] final_data,
     output logic [3:0] PID_to_sender, ENDP_to_sender,
     output logic [63:0] data_to_sender
);

    enum logic [3:0] {IDLE = '0, SEND_IN = 4'd1, SEND_OUT = 4'd2,
                      WAIT_DIN = 4'd3, SEND_DOUT = 4'd4, 
                      ACK_IN = 4'd5, NAK_IN = 4'd6, 
                      WAIT_OUT_RESPONSE = 4'd7} cur_state, next_state; 

    logic [3:0] error_counter, error_counter_nxt, timeout_counter, timeout_counter_nxt;
    logic [8:0] timer, timer_nxt;

    logic timeout, end_transaction;

    // Timeout occurs when we get 255 clock ticks from the timer
    assign timeout = timer == 9'd255;

    // We need to end the transaction if we have counted 8 errors or 8 timeouts
    assign end_transaction = (error_counter == 4'd6 && had_errors) || (timeout_counter == 4'd6 && timeout);

    // Next state transition logic
    always_comb begin
        case (cur_state)

            // If asked to read then begin IN transaction
            // If asked to write then begin OUT transaction
            // Else remain Idle
            IDLE: begin
                if (make_read) next_state = SEND_IN;
                else if (make_write) next_state = SEND_OUT;
                else next_state = IDLE;
            end

            // Proceed to waiting for a response once sending has
            // finished.
            // Otherwise, keep on sending
            SEND_IN: next_state = send_done ? WAIT_DIN : SEND_IN;


            // Proceed to sending Data once out packet has finished
            // sending
            // Otherwise, keep on sending
            SEND_OUT: next_state = send_done ? SEND_DOUT : SEND_OUT;

            WAIT_DIN: begin

                // If a packet is received, check if it had errors.
                // If it had errors, then check whether you need to end the transaction; 
                // send a NAK if you are allowed to continue or end transaction with error if not

                // If it had no errors, then all is good and you can send an ACK
                if (receive_done) begin
                    if (had_errors) next_state = end_transaction ? IDLE : NAK_IN;
                    else next_state = ACK_IN;
                end

                // If nothing was received and a timeout occured, check if you can end the transaction
                // If not send a NAK, otherwise end transaction with error and go back to idle
                else if (timeout) next_state = end_transaction ? IDLE : NAK_IN;

                // If nothing was received but we haven't timed out yet, just keep on waiting
                else next_state = WAIT_DIN;
            end

            // Stay here until DATA0 package has finished sending. Once it's been sent,
            // go on waiting for device's response
            SEND_DOUT: next_state = send_done ? WAIT_OUT_RESPONSE : SEND_DOUT;

            // Stay here until ACK package has finished sending. Once it's been sent,
            // finish transaction with no errors (all went well :) ) 
            ACK_IN: next_state = send_done ? IDLE : ACK_IN;

            // If we are sending a nak, then stay here until we are done sending it and 
            // go back to waiting for device's repeat response
            NAK_IN: next_state = send_done ? WAIT_DIN : NAK_IN;

            // If we received a packet, check if it was a NAK (or had errors)
            // If it was a NAK (oh no) check if we should end the transaction or repeat the 
            // DATA0 package. If we need to end transaction then go Idle cold turkey. Otherwise,
            // go back to sending DATA0

            // If we didn't receive a NAK and the packet was accurate, then we are good and we can
            // go end the transaction with a correct idle

            // Now if we get a timeout, end or repeat transaction depending on how many times this has occured.
            // Otherwise, we probably haven't received the packet yet so we can go on waiting here.
            WAIT_OUT_RESPONSE: begin
                if (receive_done) begin
                    if (received_PID == PID_NAK || received_PID != PID_ACK) next_state = end_transaction ? IDLE : SEND_DOUT;
                    else next_state = IDLE;
                end
                else begin
                    if (timeout) next_state = end_transaction ? IDLE : SEND_DOUT;
                    else next_state = WAIT_OUT_RESPONSE;
                end
            end
            default: next_state = IDLE;
        endcase
    end

    // Counter combinational logic + some other stuff control stuff
    // To much effort to fully explain, the general trend is that
    // send and expect packet signals are sent in the appropriate times
    // (with one clock cycle gap for turnaround) and that the PID and data
    // to send are prepared and asserted before we actually start sending
    always_comb begin
        ENDP_to_sender = addr_or_data ? addr_ENDP : data_ENDP;
        case (cur_state) 
            IDLE: begin
                error_counter_nxt = '0;
                timeout_counter_nxt = '0;
                timer_nxt = '0;

                send_packet = next_state != IDLE;
                expect_packet = '0;
                PID_to_sender = next_state == SEND_IN ? PID_IN : PID_OUT;
                data_to_sender = '0;

            end
            SEND_IN: begin
                error_counter_nxt = '0;
                timeout_counter_nxt = '0;
                timer_nxt = '0;  

                send_packet = '0;
                expect_packet = next_state != SEND_IN;
                PID_to_sender = PID_IN;
                data_to_sender = '0;
            end
            SEND_OUT: begin
                error_counter_nxt = '0;
                timeout_counter_nxt = '0;
                timer_nxt = '0;

                send_packet = '1;
                expect_packet = '0;
                PID_to_sender = (next_state != SEND_OUT) ? PID_DATA0 : PID_OUT;
                data_to_sender = send_data;

            end
            WAIT_DIN: begin
                error_counter_nxt = had_errors ? error_counter + 1 : error_counter;
                timeout_counter_nxt = timeout ? timeout_counter + 1 : timeout_counter;
                timer_nxt = timer + 1;

                // send_packet = (next_state == ACK_IN || next_state == NAK_IN);
                send_packet = '0;
                expect_packet = next_state == WAIT_DIN;
                if (next_state == ACK_IN) PID_to_sender = PID_ACK;
                else if (next_state == NAK_IN) PID_to_sender = PID_NAK;
                else PID_to_sender = '0;
                data_to_sender = '0;
            end
            SEND_DOUT: begin
                error_counter_nxt = error_counter; 
                timeout_counter_nxt = timeout_counter;
                timer_nxt = '0;

                send_packet = next_state == SEND_DOUT;
                expect_packet = next_state != SEND_DOUT;
                data_to_sender = send_data;
                PID_to_sender = PID_DATA0;
            end
            ACK_IN: begin
                error_counter_nxt = error_counter;
                timeout_counter_nxt = timeout_counter;
                timer_nxt = '0;

                send_packet = next_state == ACK_IN;
                expect_packet = '0;
                data_to_sender = '0;
                PID_to_sender = PID_ACK;
            end
            NAK_IN: begin
                error_counter_nxt = error_counter;
                timeout_counter_nxt = timeout_counter;
                timer_nxt = '0;

                send_packet = next_state == NAK_IN;
                expect_packet = next_state == WAIT_DIN;
                data_to_sender = '0;
                PID_to_sender = PID_NAK;
            end
            WAIT_OUT_RESPONSE: begin
                error_counter_nxt = had_errors ? error_counter + 1 : error_counter;
                timeout_counter_nxt = timeout ? timeout_counter + 1 : timeout_counter;
                timer_nxt = timer + 1;

                // send_packet = next_state == SEND_DOUT;
                send_packet = '0;
                expect_packet = next_state == WAIT_OUT_RESPONSE;
                data_to_sender = next_state == SEND_DOUT ? send_data : '0;
                PID_to_sender = next_state == SEND_DOUT ? PID_DATA0 : '0;
            end
            default: begin
                error_counter_nxt   = '0;
                timeout_counter_nxt = '0;
                timer_nxt           = '0;

                send_packet         = '0;
                expect_packet       = '0;
                PID_to_sender       = '0;
                data_to_sender      = '0;
            end
        endcase
    end

    // Other combinational logic
    always_comb begin

        // We have completed a transaction when we are about to go back to idle
        completed_transaction = (cur_state == WAIT_DIN || 
                                 cur_state == WAIT_OUT_RESPONSE || 
                                 cur_state == ACK_IN) &&
                                 next_state == IDLE;

        // We ended with issues if we are completing a transaction on a timeout
        // or corrupt data error
        ended_with_errors = completed_transaction && 
                            (timeout || (had_errors && received_PID != PID_ACK));
    end


    // We latch the data from any incoming transaction, when it finishes
    // We know that the last incoming transaction is going to be a data packet,
    // unless there was an error in which case we don't care about the final data.
    always_ff @(posedge clock, negedge reset_n) begin
        if (~reset_n) final_data <= '0;
        else final_data <= receive_done ? received_data : final_data;
    end


    always_ff @(posedge clock, negedge reset_n) begin
        if (~reset_n) cur_state <= IDLE;
        else cur_state <= next_state;
    end

    always_ff @(posedge clock, negedge reset_n) begin
        if (~reset_n) begin
            error_counter <= '0;
            timeout_counter <= '0;
            timer <= '0;
        end
        else begin
            error_counter <= error_counter_nxt;
            timeout_counter <= timeout_counter_nxt;
            timer <= timer_nxt;
        end
    end
endmodule: IN_OUT_HANDLER

// This is the bigger boy FSM. He is way more abstract,
// because he handles read and write operations. He doesn't 
// interface with the transmitter and receiver at all. Instead, 
// only knows about the IN and OUT transactions the IO Handler
// FSM performs. 
// He takes in info about the results of IN/OUT transactions
// and determines which transaction should occur first. It also
// determines the Data that is being passed around in those transactions,
// that being the DATA0 (Are we sending an address or data), and the ENDP
// 4 or 8?
module READ_WRITE_HANDLER (
    input  logic clock, reset_n,
    input  logic do_read, do_write,
    input  logic txn_done, txn_error,
    input  logic [63:0] in_data,

    input  logic [15:0] mempage, 
    input  logic [63:0] memdata,

    output logic make_in, make_out,
    output logic addr_or_data,

    output logic finished, finished_with_error, no_transaction,
    output logic [63:0] out_data, final_data
);

    enum logic [2:0] {IDLE = '0, ADDR_READ = 3'd1, DATA_READ = 3'd2,
                      ADDR_WRITE = 3'd3, DATA_WRITE = 3'd4} cur_state, next_state;

    logic [63:0] page_data;
    assign page_data = {mempage, 48'd0};
    
    
    assign no_transaction = cur_state == IDLE;

    // Next state logic for this FSM.
    // When we evade IDLE, we either write and address and expect
    // data or we write an address and then write data.
    always_comb begin
        case (cur_state) 
            IDLE: begin
                if (do_read) next_state = ADDR_READ;
                else if (do_write) next_state = ADDR_WRITE;
                else next_state = IDLE;
            end
            ADDR_READ: begin
                if (txn_done && txn_error) next_state = IDLE;
                else if (txn_done) next_state = DATA_READ;
                else next_state = ADDR_READ;
            end
            DATA_READ: begin
                next_state = txn_done ? IDLE : DATA_READ;
            end
            ADDR_WRITE: begin
                if (txn_done && txn_error) next_state = IDLE;
                else if (txn_done) next_state = DATA_WRITE;
                else next_state = ADDR_WRITE;
            end
            DATA_WRITE: begin
                next_state = txn_done ? IDLE : DATA_WRITE;
            end
            default: begin
                next_state = IDLE;
            end
        endcase
    end

    // Some control / status point logic. Mostly handles some basic 
    // outputs relating to transactions finishing and choosing between
    // the data that needs to be sent for the next IN/OUT transaction that
    // will occur.
    always_comb begin
        case(cur_state) 
            IDLE: begin
                finished            = '0;
                finished_with_error = '0;
                make_out            = next_state != IDLE;
                make_in             = '0;
                out_data            = next_state != IDLE ? page_data : '0;
                addr_or_data        = next_state != IDLE;
            end
            ADDR_READ: begin
                finished_with_error = next_state == IDLE && txn_error;
                finished            = next_state == IDLE;
                make_out            = txn_done && !txn_error;
                make_in             = '0;
                out_data            = txn_done ? memdata : page_data;
                addr_or_data        = ~txn_done;
            end
            DATA_READ: begin
                finished_with_error = txn_error;
                finished            = txn_done;
                make_out            = '0;
                make_in             = next_state != IDLE;
                out_data            = memdata;
                addr_or_data        = '0;
            end
            ADDR_WRITE: begin
                finished_with_error = next_state == IDLE && txn_error;
                finished            = next_state == IDLE;
                make_out            = txn_done && !txn_error;
                make_in             = '0;
                out_data            = txn_done ? memdata : page_data;
                addr_or_data        = ~txn_done;
            end
            DATA_WRITE: begin
                finished_with_error = txn_error;
                finished            = txn_done;
                make_out            = next_state != IDLE;
                make_in             = '0;
                out_data            = memdata;
                addr_or_data        = '0;
            end
            default: begin
                finished_with_error = '0;
                finished            = '0;
                make_out            = '0;
                make_in             = '0;
                out_data            = '0;
                addr_or_data        = '0;
            end
        endcase
    end

    // Final data is given by the IN/OUT handler
    // and is passed unchanged through here for the sake of cleanness
    // (I sure do enjoy it when all the top level signals come out of the
    // top level module)
    assign final_data = in_data;

    // Regular next state update
    always_ff @(posedge clock, negedge reset_n) begin
        if (~reset_n) cur_state <= IDLE;
        else cur_state <= next_state;
    end
endmodule: READ_WRITE_HANDLER

// Man just listens to the wires. It
// listens to the waves rippling through USBWires
// and returns the equivalent bits. He can also
// tell you when he saw an EOP.
module WIRE_LISTENER (
    input  logic clock, reset_n,
    input  logic [1:0] wires_in,
    output logic bit_out, invalid, saw_eop
);

    logic dp, dm;
    assign dp = wires_in[1];
    assign dm = wires_in[0];

    logic [2:0] dp_log, dm_log;

    always_ff @(posedge clock, negedge reset_n) begin
        if (!reset_n) begin
            dp_log <= '0;
            dm_log <= '0;
        end
        else begin
            dp_log <= (dp_log << 1) + dp;
            dm_log <= (dm_log << 1) + dm;
        end
    end

    // ENCODING SCHEME:
    // bit_out = 1 and invalid = 0 --> J
    // bit_out = 0 and invalid = 0 --> K
    // bit_out = 0 and invalid = 1 --> SEO
    // bit_out = 1 and invalid = 1 --> UNDEFINED
    always_comb begin
        case ({dp, dm})
            2'b00: begin
                bit_out = '0;
                invalid = '1;
            end
            2'b01: begin
                bit_out = '0;
                invalid = '0;
            end
            2'b10: begin
                bit_out = '1;
                invalid = '0;
            end
            2'b11: begin
                bit_out = '1;
                invalid = '1;
            end
        endcase
    end

    assign saw_eop = (dp_log == 3'b001) & (dm_log == 3'b000);
endmodule: WIRE_LISTENER

// This guy listens to encoded binary input
// and let's everyone know if it detects
// a sync pattern
module SYNC_DETECTOR (
    input  logic clock, reset_n,
    input  logic bit_in,
    output logic saw_sync
);
    logic [7:0] log;

    always_ff @(posedge clock, negedge reset_n) begin
        if (!reset_n) log <= '0;
        else log <= (log << 1) + bit_in;
    end

    assign saw_sync = (log[6:0] == 7'b0101_010) & (bit_in == '0);
endmodule: SYNC_DETECTOR


// Very similar to the NRZI module.
// Only difference is that this guy also has
// an init control that forces stored value into
// a zero
module NRZI_DECODER (
    input  logic clock, reset_n,
    input  logic bit_in, en, init,
    output logic bit_out
);

    logic cur_value, cur_value_next;

    assign bit_out = cur_value == bit_in;
    assign cur_value_next = bit_in;

    // Initial value of internal state starts out as a 1
    always_ff @(posedge clock, negedge reset_n) begin
        if (~reset_n) cur_value <= '1;
        else begin
            if (en) begin
                cur_value <= init ? '0 : cur_value_next;
            end
            else cur_value <= cur_value;
        end
    end
endmodule: NRZI_DECODER


// Honestly, this is just reused bit stuffer code.
// The functionality here is the same, detect a sequence
// of 6 consecutive 1s. It doesn't matter what we output
// during the stuff clock as long as we signal to others
// that our output is stuffing and not data. 

// I renamed the module for the sake of convenience.
module BIT_UNSTUFFER (
    input  logic clock, reset_n, en, bit_in, hard_init,
    output logic bit_out, is_stuffing
);
    logic [2:0] count, count_next;

    // Combinational logic for next state of counter
    // and output of bit stuffer
    always_comb begin
        if (count == 3'd6) begin
            bit_out     = '0;
            count_next  = '0;
            is_stuffing = '1;
        end
        else begin
            bit_out     = bit_in;
            count_next  = bit_in ? count + 1 : '0;
            is_stuffing = '0;
        end
    end

    // Really the bitstuffer is just a counter
    always_ff @(posedge clock, negedge reset_n) begin
        if (~reset_n) count <= '0;
        else if (hard_init) count <= '0;
        else count <= en ? count_next : count;
    end
endmodule: BIT_UNSTUFFER

// This guy is really just a big boy shift register that 
// just sort of accumulates all the incoming bits in the 
// right bucket
module PACKET_RECEIVER (
    input  logic clock, reset_n,
    input  logic bit_in,
    input  logic [1:0] phase,

    output logic [3:0] pid, pid_to_fsm,
    output logic [10:0] addr_endp,
    output logic [63:0] payload,
    output logic PID_ERROR
);

    // This is a terrible lazy way to define an enumeration
    // I absolutely hate it. There is a better way to do this but
    // I don't really care :)
    enum logic [1:0] {IDLE_RECEIVE = '0, PID_RECEIVING = 2'd1, 
                      ADDR_ENDP = 2'd2, PAYLOAD_RECEIVE = 2'd3} temp;


    logic [7:0] PID_accum;
    logic [10:0] ADDR_ENDP_accum;
    logic [63:0] PAYLOAD_accum;
    logic [3:0] pid_inv;

    // Just setting up the outputs
    assign pid = PID_accum[3:0];
    assign pid_inv = PID_accum[7:4];
    assign addr_endp = ADDR_ENDP_accum;
    assign payload = PAYLOAD_accum;

    // This guy gave me so many nightmares. Apparently, the 
    // moment when my FSM makes a state change choice based on PID, 
    // the PID is still one bit off (it fully accumulates the moment 
    // the choice has already been done). As a result, to make the choice in the
    // first place I check with a shifted version of the PID.
    assign pid_to_fsm = PID_accum[4:1];

    // Apparently PIDs are always correct, so I never actually do anything 
    // with this guy lmao. It's also probably wrong
    assign PID_ERROR = PID_accum[3:0] != PID_accum[7:4];

    // Basically accumulates the appropriate fields in the
    // appropriate buckets.
    // Bits are pushed in trough the MSB towards the LSB
    always_ff @(posedge clock, negedge reset_n) begin
        if (!reset_n) begin
            PID_accum <= '0;
            ADDR_ENDP_accum <= '0;
            PAYLOAD_accum <= '0;
        end
        else begin
            case (phase)
                2'd0: begin
                    PID_accum <= PID_accum;
                    ADDR_ENDP_accum <= ADDR_ENDP_accum;
                    PAYLOAD_accum <= PAYLOAD_accum;
                end
                2'd1: begin
                    PID_accum <= {bit_in, PID_accum[7:1]};
                    ADDR_ENDP_accum <= ADDR_ENDP_accum;
                    PAYLOAD_accum <= PAYLOAD_accum;         
                end
                2'd2: begin
                    PID_accum <= PID_accum;
                    ADDR_ENDP_accum <= {bit_in, ADDR_ENDP_accum[10:1]};
                    PAYLOAD_accum <= PAYLOAD_accum;           
                end
                2'd3: begin
                    PID_accum <= PID_accum;
                    ADDR_ENDP_accum <= ADDR_ENDP_accum;
                    PAYLOAD_accum <= {bit_in, PAYLOAD_accum[63:1]};
                end
            endcase
        end
    end
endmodule: PACKET_RECEIVER

// Packet receiving FSM. This guy is the PACKET_SEND_MANAGER
// equivalent. Basically just parse through different segments
// of the incoming package and accumulates all those juicy juicy
// bits after decoding them.
module PACKET_RECEIVE_MANAGER (
    input  logic clock, reset_n,
    input  logic saw_sync, saw_eop, pause, enable, 
    input  logic [3:0] PID,

    output logic [1:0] packet_phase,
    output logic crc_select, crc_init, crc_pause,
    output logic stuff_en, nrzi_en, nrzi_init, hard_init,
    output logic send_done
);

    enum logic [2:0] {IDLE = '0, PID_STATE = 3'd1, ADDR_ENDP = 3'd2,
                    PAYLOAD = 3'd3, CRC5 = 3'd4, CRC16 = 3'd5,
                    EOP = 3'd6, ERROR_WAIT = 3'd7} cur_state, next_state;

    logic [6:0] count, count_next;

    // Next state logic
    always_comb begin
        case (cur_state)
            // Slightly modified, we skip the sync
            // and go straight to pid when we've detected a sync
            IDLE: begin
                next_state = saw_sync & enable ? PID_STATE : IDLE;
                count_next = '0;
            end
            // Ultimate branching state, we go to different
            // states based on PID value.
            // This time we also have an error default state
            // for incorrect PIDs
            PID_STATE: begin
                if (count == 7'd7) begin
                    count_next = '0;
                    if (PID == PID_IN || PID == PID_OUT) next_state = ADDR_ENDP;
                    else if (PID == PID_DATA0) next_state = PAYLOAD;
                    else if (PID == PID_NAK || PID == PID_ACK) next_state = EOP;
                    else next_state = ERROR_WAIT;
                end
                else begin
                    next_state = PID_STATE;
                    count_next = count + 1;
                end
            end
            // Same as PACKET_SEND_MANAGER
            ADDR_ENDP: begin
                next_state = count == 7'd10 && ~pause ? CRC5 : ADDR_ENDP;
                if (pause) count_next = count;
                else count_next = count == 7'd10 ? '0 : count + 1;
            end
            PAYLOAD: begin
                next_state = count == 7'd63 && ~pause ? CRC16 : PAYLOAD;
                if (pause) count_next = count;
                else count_next = count == 7'd63 ? '0 : count + 1;
            end
            CRC5: begin
                next_state = count == 7'd4 && ~pause ? EOP : CRC5;
                if (pause) count_next = count;
                else count_next = count == 7'd4 ? '0 : count + 1;
            end
            CRC16: begin
                next_state = count == 7'd15 && ~pause ? EOP : CRC16;
                if (pause) count_next = count;
                else count_next = count == 7'd15 ? '0 : count + 1;
            end
            EOP: begin
                next_state = count == 7'd2 ? IDLE : EOP;
                if (pause) count_next = count;
                else count_next = count == 7'd2 ? '0 : count + 1;
            end
            // If we see an incorrect PID, we wait in error until
            // sender finishes yapping
            ERROR_WAIT: begin
                next_state = saw_eop ? IDLE : ERROR_WAIT;
                count_next = saw_eop ? '0 : count + 1; // Here for debugging purposes
            end
        endcase
    end


    // Status / Control point logic
    // There's a bunch of stuff going on here,
    // I don't really care to go into much detail.
    // Basically this blob of code just makes sure every submodule
    // gets turned on at the right time

    // Compared to previous iterations, I am also hard reseting 
    // crc and bit_stuff after a package is sent. I wasn't doing
    // that in the prelab because I was only sending one packet and
    // my foolish negligence came back to bite me :)
    always_comb begin
        packet_phase = '0; 
        crc_select = '0; crc_init = '1;
        stuff_en = '0; nrzi_en = '1; nrzi_init = '1;
        send_done = '0; hard_init = '0;

        crc_pause = pause;

        case (cur_state) 
            IDLE: begin
                packet_phase = '0;
                crc_select = '0; crc_init = '1;
                stuff_en = '0; nrzi_en = '1; nrzi_init = ~(saw_sync & enable);
                send_done = '0;
                hard_init = '1;
            end
            PID_STATE: begin
                packet_phase = pause ? '0 : 2'd1;
                crc_select = '0; crc_init = '1;
                stuff_en = '0; nrzi_en = '1; nrzi_init = '0;
                send_done = '0; crc_pause = pause;
            end
            ADDR_ENDP: begin
                packet_phase = pause ? '0 : 2'd2;
                crc_select = '0; crc_init = '0;
                stuff_en = '1; nrzi_en = '1; nrzi_init = '0;
                send_done = '0; crc_pause = pause;
            end
            PAYLOAD: begin
                packet_phase = pause ? '0 : 2'd3;
                crc_select = '0; crc_init = '0;
                stuff_en = '1; nrzi_en = '1; nrzi_init = '0;
                send_done = '0; crc_pause = pause;
            end
            CRC5: begin
                packet_phase = '0;
                crc_select = '0; crc_init = '0;
                stuff_en = '1; nrzi_en = '1; nrzi_init = '0;
                send_done = '0; crc_pause = pause;
            end
            CRC16: begin
                packet_phase = '0;
                crc_select = '0; crc_init = '0;
                stuff_en = '1; nrzi_en = '1; nrzi_init = '0;
                send_done = '0; crc_pause = pause;
            end
            EOP: begin
                packet_phase = '0;
                crc_select = '0; crc_init = '0;
                stuff_en = '0; nrzi_en = '1; nrzi_init = '0;
                send_done = count == 7'd2; crc_pause = '1;
                hard_init = '1;

            end
            ERROR_WAIT: begin
                packet_phase = '0;
                crc_select = '0; crc_init = '0;
                stuff_en = '0; nrzi_en = '1; nrzi_init = '0;
                crc_pause = '1;
                send_done = saw_eop;
            end
        endcase
    end

    // Same as PACKET_SEND_MANAGER

    // State update
    always_ff @(posedge clock, negedge reset_n) begin
        if (~reset_n) cur_state <= IDLE;
        else cur_state <= next_state;
    end

    // Counter update
    always_ff @(posedge clock, negedge reset_n) begin
        if (~reset_n) count <= '0;
        else count <= count_next;
    end
endmodule: PACKET_RECEIVE_MANAGER


// Same thing as bit unstuffer, this guy is just
// the regular driver with some additional outputs.
// Made a separate module so that I don't get confused
module CRC_DRIVER_RECEIVE (
    input  logic clock, reset_n,
    input  logic select_crc, init, pause, bit_in,
    input  logic [6:0] index,
    output logic bit_out,
    output logic residue_match5, residue_match16
);

    logic en;
    assign en = ~pause;

    logic [4:0] crc5_out, crc5_out_inv;
    logic [15:0] crc16_out, crc16_out_inv;

    always_comb begin
        for (int i = 0; i < 5; i++) crc5_out_inv[i] = ~crc5_out[i];
        for (int j = 0; j < 16; j++) crc16_out_inv[j] = ~crc16_out[j];
    end

    // The driver selects between CRC5 and CRC16 based on the FSM's needs
    CRC5  crc5(.residue_match(residue_match5), .*);
    CRC16 crc16(.residue_match(residue_match16), .*);

    // The driver takes in an index from the FSM in order to select which CRC bit it will
    // output. 
    assign bit_out = select_crc ? crc5_out_inv[4 - index] : crc16_out_inv[15 - index];
endmodule: CRC_DRIVER_RECEIVE


// Main hardware thread for receiving packets. This
// guy just instantiates all smaller submodules and wires
// them together. It starts listening on the wires once
// expect packet is asserted and it returns you the packet
// it saw + whether it was correct. 
module PACKET_RECEIVE (
    input  logic clock, reset_n,
    input  logic expect_packet,

    output logic [3:0] PID, ENDP,
    output logic [6:0] Addr,
    output logic [63:0] Payload,

    output logic receive_done, had_errors,

    input  logic [1:0] wires_in
);

    // Instantiating all the smaller boys 
    logic wire_out, saw_eop;
    WIRE_LISTENER wire_in(.bit_out(wire_out), .invalid(), .*);

    logic saw_sync;
    SYNC_DETECTOR find_sync(.bit_in(wire_out), .*);

    logic nrzi_out, nrzi_en, nrzi_init;
    NRZI_DECODER nrzi(.bit_in(wire_out), .bit_out(nrzi_out), 
                      .en(nrzi_en), .init(nrzi_init), .*);

    logic stuff_out, stuff_en, pause, hard_init;
    BIT_UNSTUFFER bit_unstuff(.bit_in(nrzi_out), .bit_out(stuff_out), 
                              .en(stuff_en), .is_stuffing(pause), .*);

    logic [1:0] phase;
    logic [3:0] pid, pid_to_fsm;
    logic [10:0] addr_endp;
    logic [63:0] payload;

    logic PID_ERROR;

    PACKET_RECEIVER packet_decode(.bit_in(stuff_out), .*);

    logic crc_select, crc_init, residue_match5, residue_match16, crc_pause;
    CRC_DRIVER_RECEIVE crc(.bit_in(stuff_out), .bit_out(), .index('0),
                           .select_crc(crc_select), .init(crc_init), 
                           .pause(crc_pause), .*);

    PACKET_RECEIVE_MANAGER fsm(.enable(expect_packet), .PID(pid_to_fsm), 
                               .send_done(receive_done), .packet_phase(phase), .*);

    assign PID = pid;
    assign Addr = addr_endp[6:0];
    assign ENDP = addr_endp[10:7];
    assign Payload = payload;

    // Error occurs when we had no match from either
    // crc residue
    assign had_errors =  (~residue_match5 & ~residue_match16 & receive_done);
endmodule: PACKET_RECEIVE