module USBHost (
	clock,
	reset_n,
	read,
	write,
	mode,
	data_in,
	wires_in,
	wires_out,
	status,
	data_indx,
	data_out
);
	input wire clock;
	input wire reset_n;
	input wire read;
	input wire write;
	input wire [3:0] mode;
	input wire [3:0] data_in;
	input wire [1:0] wires_in;
	output wire [1:0] wires_out;
	output reg [1:0] status;
	output wire [3:0] data_indx;
	output wire [3:0] data_out;
	wire send_packet;
	wire expect_packet;
	wire send_done;
	wire receive_done;
	wire had_errors;
	wire addr_or_data;
	wire [3:0] send_PID;
	wire [3:0] send_ENDP;
	wire [6:0] send_Addr;
	wire [63:0] send_payload;
	wire [3:0] received_PID;
	wire [3:0] received_ENDP;
	wire [6:0] received_Addr;
	wire [63:0] received_payload;
	wire make_in;
	wire make_out;
	reg do_read;
	reg do_write;
	wire txn_done;
	wire txn_error;
	wire finished;
	wire finished_with_error;
	wire [63:0] final_data;
	wire [63:0] out_data;
	wire [63:0] in_data;
	wire [63:0] memory_data;
	wire [15:0] memory_address;
	wire is_idle;
	reg data_received;
	reg en_interface;
	wire no_transaction;
	wire [3:0] data_ENDP;
	wire [3:0] addr_ENDP;
	always @(*) begin
		data_received = finished;
		en_interface = no_transaction;
		if (is_idle && no_transaction)
			status = 2'b00;
		else if (finished && finished_with_error)
			status = 2'b01;
		else if (finished && ~finished_with_error)
			status = 2'b10;
		else
			status = 2'b11;
		do_read = (is_idle && no_transaction) && read;
		do_write = (is_idle && no_transaction) && write;
	end
	HOST_INTERFACE inter(
		.en(en_interface),
		.mempage(memory_address),
		.memdata(memory_data),
		.clock(clock),
		.reset_n(reset_n),
		.mode(mode),
		.data_in(data_in),
		.is_idle(is_idle),
		.data_indx(data_indx),
		.data_out(data_out),
		.data_received(data_received),
		.final_data(final_data),
		.data_ENDP(data_ENDP),
		.addr_ENDP(addr_ENDP),
		.send_Addr(send_Addr)
	);
	PACKET_SEND transmitter(
		.PID(send_PID),
		.ENDP(send_ENDP),
		.Addr(send_Addr),
		.Payload(send_payload),
		.clock(clock),
		.reset_n(reset_n),
		.send_packet(send_packet),
		.send_done(send_done),
		.wires_out(wires_out)
	);
	PACKET_RECEIVE receiver(
		.PID(received_PID),
		.ENDP(received_ENDP),
		.Addr(received_Addr),
		.Payload(received_payload),
		.clock(clock),
		.reset_n(reset_n),
		.expect_packet(expect_packet),
		.receive_done(receive_done),
		.had_errors(had_errors),
		.wires_in(wires_in)
	);
	IN_OUT_HANDLER io_fsm(
		.make_read(make_in),
		.make_write(make_out),
		.received_data(received_payload),
		.send_data(out_data),
		.completed_transaction(txn_done),
		.ended_with_errors(txn_error),
		.final_data(in_data),
		.PID_to_sender(send_PID),
		.data_to_sender(send_payload),
		.ENDP_to_sender(send_ENDP),
		.clock(clock),
		.reset_n(reset_n),
		.send_done(send_done),
		.receive_done(receive_done),
		.had_errors(had_errors),
		.received_PID(received_PID),
		.addr_ENDP(addr_ENDP),
		.data_ENDP(data_ENDP),
		.addr_or_data(addr_or_data),
		.send_packet(send_packet),
		.expect_packet(expect_packet)
	);
	READ_WRITE_HANDLER rw_fsm(
		.mempage(memory_address),
		.memdata(memory_data),
		.clock(clock),
		.reset_n(reset_n),
		.do_read(do_read),
		.do_write(do_write),
		.txn_done(txn_done),
		.txn_error(txn_error),
		.in_data(in_data),
		.make_in(make_in),
		.make_out(make_out),
		.addr_or_data(addr_or_data),
		.finished(finished),
		.finished_with_error(finished_with_error),
		.no_transaction(no_transaction),
		.out_data(out_data),
		.final_data(final_data)
	);
endmodule
module HOST_INTERFACE (
	clock,
	reset_n,
	mode,
	data_in,
	is_idle,
	data_indx,
	data_out,
	data_received,
	en,
	final_data,
	data_ENDP,
	addr_ENDP,
	send_Addr,
	mempage,
	memdata
);
	input wire clock;
	input wire reset_n;
	input wire [3:0] mode;
	input wire [3:0] data_in;
	output reg is_idle;
	output reg [3:0] data_indx;
	output reg [3:0] data_out;
	input wire data_received;
	input wire en;
	input wire [63:0] final_data;
	output reg [3:0] data_ENDP;
	output reg [3:0] addr_ENDP;
	output reg [6:0] send_Addr;
	output reg [15:0] mempage;
	output reg [63:0] memdata;
	reg [7:0] ENDP_reg;
	reg [7:0] ENDP_reg_next;
	reg [7:0] Addr_reg;
	reg [7:0] Addr_reg_next;
	reg [15:0] mempage_reg;
	reg [15:0] mempage_reg_next;
	reg [63:0] data_in_reg;
	reg [63:0] data_out_reg;
	reg [63:0] data_out_reg_next;
	reg [3:0] count;
	reg [3:0] count_next;
	wire [3:0] data_in_hb;
	wire [3:0] data_out_hb;
	wire [3:0] misc_hb;
	reg [3:0] cur_state;
	reg [3:0] next_state;
	always @(*) begin
		data_ENDP = ENDP_reg[3:0];
		addr_ENDP = ENDP_reg[7:4];
		send_Addr = Addr_reg[6:0];
		mempage = mempage_reg;
		memdata = data_out_reg;
		is_idle = cur_state == 4'd0;
	end
	HALFBYTE_PICKER_64 in_hb(
		.in_reg(data_in_reg),
		.index(count),
		.halfbyte(data_in_hb)
	);
	HALFBYTE_PICKER_64 out_hb(
		.in_reg(data_out_reg),
		.index(count),
		.halfbyte(data_out_hb)
	);
	HALFBYTE_PICKER_32 msc_hb(
		.in_reg({Addr_reg, ENDP_reg, mempage_reg}),
		.index(count),
		.halfbyte(misc_hb)
	);
	always @(*) begin
		ENDP_reg_next = ENDP_reg;
		Addr_reg_next = Addr_reg;
		mempage_reg_next = mempage_reg;
		data_out_reg_next = data_out_reg;
		data_indx = 1'sb0;
		data_out = 1'sb0;
		case (cur_state)
			4'd0: begin
				count_next = 1'sb0;
				if (en)
					case (mode)
						4'd0: next_state = 4'd0;
						4'd1: next_state = 4'd1;
						4'd2: next_state = 4'd2;
						4'd3: next_state = 4'd3;
						4'd4: next_state = 4'd4;
						4'd5: next_state = 4'd5;
						4'd6: next_state = 4'd6;
						4'd7: next_state = 4'd7;
						default: next_state = 4'd0;
					endcase
				else
					next_state = 4'd0;
			end
			4'd2: begin
				count_next = (count == 4'd1 ? 4'd0 : count + 1);
				next_state = (count == 4'd1 ? 4'd0 : 4'd2);
				ENDP_reg_next = (ENDP_reg << 4) + data_in;
			end
			4'd1: begin
				count_next = (count == 4'd1 ? 4'd0 : count + 1);
				next_state = (count == 4'd1 ? 4'd0 : 4'd1);
				Addr_reg_next = (Addr_reg << 4) + data_in;
			end
			4'd3: begin
				count_next = (count == 4'd3 ? 4'd0 : count + 1);
				next_state = (count == 4'd3 ? 4'd0 : 4'd3);
				mempage_reg_next = (mempage_reg << 4) + data_in;
			end
			4'd4: begin
				count_next = (count == 4'd15 ? 4'd0 : count + 1);
				next_state = (count == 4'd15 ? 4'd0 : 4'd4);
				data_out_reg_next = (data_out_reg << 4) + data_in;
			end
			4'd5: begin
				count_next = (count == 4'd15 ? 4'd0 : count + 1);
				next_state = (count == 4'd15 ? 4'd0 : 4'd5);
				data_indx = count;
				data_out = data_in_hb;
			end
			4'd6: begin
				count_next = (count == 4'd15 ? 4'd0 : count + 1);
				next_state = (count == 4'd15 ? 4'd0 : 4'd6);
				data_indx = count;
				data_out = data_out_hb;
			end
			4'd7: begin
				count_next = (count == 4'd7 ? 4'd0 : count + 1);
				next_state = (count == 4'd7 ? 4'd0 : 4'd7);
				data_indx = count;
				data_out = misc_hb;
			end
			default: begin
				count_next = 1'sb0;
				next_state = 4'd0;
				data_indx = count;
				data_out = 1'sb0;
			end
		endcase
	end
	always @(posedge clock or negedge reset_n)
		if (~reset_n)
			cur_state <= 4'd0;
		else
			cur_state <= next_state;
	always @(posedge clock or negedge reset_n)
		if (~reset_n) begin
			ENDP_reg <= 1'sb0;
			Addr_reg <= 1'sb0;
			mempage_reg <= 1'sb0;
			data_out_reg <= 1'sb0;
		end
		else begin
			ENDP_reg <= ENDP_reg_next;
			Addr_reg <= Addr_reg_next;
			mempage_reg <= mempage_reg_next;
			data_out_reg <= data_out_reg_next;
		end
	always @(posedge clock or negedge reset_n)
		if (~reset_n)
			data_in_reg <= 1'sb0;
		else
			data_in_reg <= (data_received ? final_data : data_in_reg);
	always @(posedge clock or negedge reset_n)
		if (~reset_n)
			count <= 1'sb0;
		else
			count <= count_next;
endmodule
module HALFBYTE_PICKER_64 (
	in_reg,
	index,
	halfbyte
);
	input wire [63:0] in_reg;
	input wire [3:0] index;
	output reg [3:0] halfbyte;
	always @(*)
		case (index)
			4'd0: halfbyte = in_reg[3:0];
			4'd1: halfbyte = in_reg[7:4];
			4'd2: halfbyte = in_reg[11:8];
			4'd3: halfbyte = in_reg[15:12];
			4'd4: halfbyte = in_reg[19:16];
			4'd5: halfbyte = in_reg[23:20];
			4'd6: halfbyte = in_reg[27:24];
			4'd7: halfbyte = in_reg[31:28];
			4'd8: halfbyte = in_reg[35:32];
			4'd9: halfbyte = in_reg[39:36];
			4'd10: halfbyte = in_reg[43:40];
			4'd11: halfbyte = in_reg[47:44];
			4'd12: halfbyte = in_reg[51:48];
			4'd13: halfbyte = in_reg[55:52];
			4'd14: halfbyte = in_reg[59:56];
			4'd15: halfbyte = in_reg[63:60];
			default: halfbyte = 1'sb0;
		endcase
endmodule
module HALFBYTE_PICKER_32 (
	in_reg,
	index,
	halfbyte
);
	input wire [31:0] in_reg;
	input wire [3:0] index;
	output reg [3:0] halfbyte;
	always @(*)
		case (index)
			4'd0: halfbyte = in_reg[3:0];
			4'd1: halfbyte = in_reg[7:4];
			4'd2: halfbyte = in_reg[11:8];
			4'd3: halfbyte = in_reg[15:12];
			4'd4: halfbyte = in_reg[19:16];
			4'd5: halfbyte = in_reg[23:20];
			4'd6: halfbyte = in_reg[27:24];
			4'd7: halfbyte = in_reg[31:28];
			default: halfbyte = 1'sb0;
		endcase
endmodule
module PACKET_SEND (
	clock,
	reset_n,
	send_packet,
	PID,
	ENDP,
	Addr,
	Payload,
	send_done,
	wires_out
);
	input wire clock;
	input wire reset_n;
	input wire send_packet;
	input wire [3:0] PID;
	input wire [3:0] ENDP;
	input wire [6:0] Addr;
	input wire [63:0] Payload;
	output wire send_done;
	output wire [1:0] wires_out;
	wire pause;
	wire encoder_or_crc;
	wire bit_to_stuff;
	wire crc_select;
	wire crc_init;
	wire stuff_en;
	wire nrzi_en;
	wire wire_en;
	wire is_eop;
	wire packet_pause;
	wire crc_pause;
	wire hard_init;
	wire packet_out;
	wire crc_out;
	wire stuff_out;
	wire nrzi_out;
	wire nrzi_init;
	wire [6:0] crc_index;
	wire [6:0] packet_index;
	wire [1:0] packet_phase;
	assign bit_to_stuff = (encoder_or_crc ? packet_out : crc_out);
	PACKET_SEND_MANAGER fsm(
		.clock(clock),
		.reset_n(reset_n),
		.pause(pause),
		.send_packet(send_packet),
		.PID(PID),
		.encoder_or_crc(encoder_or_crc),
		.crc_select(crc_select),
		.crc_init(crc_init),
		.hard_init(hard_init),
		.stuff_en(stuff_en),
		.nrzi_en(nrzi_en),
		.nrzi_init(nrzi_init),
		.wire_en(wire_en),
		.is_eop(is_eop),
		.packet_pause(packet_pause),
		.crc_pause(crc_pause),
		.packet_index(packet_index),
		.crc_index(crc_index),
		.packet_phase(packet_phase),
		.send_done(send_done)
	);
	PACKET_ENCODER encoder(
		.pause(packet_pause),
		.index(packet_index),
		.phase(packet_phase),
		.bit_out(packet_out),
		.clock(clock),
		.reset_n(reset_n),
		.PID(PID),
		.Addr(Addr),
		.ENDP(ENDP),
		.Payload(Payload)
	);
	CRC_DRIVER crc(
		.bit_in(packet_out),
		.select_crc(crc_select),
		.init(crc_init),
		.pause(crc_pause),
		.index(crc_index),
		.bit_out(crc_out),
		.clock(clock),
		.reset_n(reset_n)
	);
	BIT_STUFFER bit_stuff(
		.bit_in(bit_to_stuff),
		.en(stuff_en),
		.bit_out(stuff_out),
		.is_stuffing(pause),
		.clock(clock),
		.reset_n(reset_n),
		.hard_init(hard_init)
	);
	NRZI nrzi(
		.bit_in(stuff_out),
		.en(nrzi_en),
		.init(nrzi_init),
		.bit_out(nrzi_out),
		.clock(clock),
		.reset_n(reset_n)
	);
	WIRE_DRIVER out_wire(
		.bit_in(nrzi_out),
		.en(wire_en),
		.wires_out(wires_out),
		.clock(clock),
		.reset_n(reset_n),
		.is_eop(is_eop)
	);
endmodule
module CRC5 (
	clock,
	reset_n,
	bit_in,
	en,
	init,
	crc5_out,
	residue_match
);
	input wire clock;
	input wire reset_n;
	input wire bit_in;
	input wire en;
	input wire init;
	output reg [4:0] crc5_out;
	output wire residue_match;
	wire x1;
	assign x1 = bit_in ^ crc5_out[4];
	assign residue_match = crc5_out == 5'b01100;
	always @(posedge clock or negedge reset_n)
		if (~reset_n) begin : sv2v_autoblock_1
			reg signed [31:0] i;
			for (i = 0; i < 5; i = i + 1)
				crc5_out[i] <= 1'sb1;
		end
		else if (init) begin : sv2v_autoblock_2
			reg signed [31:0] i;
			for (i = 0; i < 5; i = i + 1)
				crc5_out[i] <= 1'sb1;
		end
		else begin
			crc5_out[0] <= (en ? x1 : crc5_out[0]);
			crc5_out[2] <= (en ? crc5_out[1] ^ x1 : crc5_out[2]);
			crc5_out[1] <= (en ? crc5_out[0] : crc5_out[1]);
			crc5_out[3] <= (en ? crc5_out[2] : crc5_out[3]);
			crc5_out[4] <= (en ? crc5_out[3] : crc5_out[4]);
		end
endmodule
module CRC16 (
	clock,
	reset_n,
	bit_in,
	en,
	init,
	crc16_out,
	residue_match
);
	input wire clock;
	input wire reset_n;
	input wire bit_in;
	input wire en;
	input wire init;
	output reg [15:0] crc16_out;
	output wire residue_match;
	wire x1;
	assign x1 = bit_in ^ crc16_out[15];
	assign residue_match = crc16_out == 16'h800d;
	always @(posedge clock or negedge reset_n)
		if (~reset_n) begin : sv2v_autoblock_1
			reg signed [31:0] i;
			for (i = 0; i < 16; i = i + 1)
				crc16_out[i] <= 1'sb1;
		end
		else if (init) begin : sv2v_autoblock_2
			reg signed [31:0] i;
			for (i = 0; i < 16; i = i + 1)
				crc16_out[i] <= 1'sb1;
		end
		else begin
			crc16_out[0] <= (en ? x1 : crc16_out[0]);
			crc16_out[2] <= (en ? crc16_out[1] ^ x1 : crc16_out[2]);
			crc16_out[15] <= (en ? crc16_out[14] ^ x1 : crc16_out[15]);
			crc16_out[1] <= (en ? crc16_out[0] : crc16_out[1]);
			crc16_out[3] <= (en ? crc16_out[2] : crc16_out[3]);
			crc16_out[4] <= (en ? crc16_out[3] : crc16_out[4]);
			crc16_out[5] <= (en ? crc16_out[4] : crc16_out[5]);
			crc16_out[6] <= (en ? crc16_out[5] : crc16_out[6]);
			crc16_out[7] <= (en ? crc16_out[6] : crc16_out[7]);
			crc16_out[8] <= (en ? crc16_out[7] : crc16_out[8]);
			crc16_out[9] <= (en ? crc16_out[8] : crc16_out[9]);
			crc16_out[10] <= (en ? crc16_out[9] : crc16_out[10]);
			crc16_out[11] <= (en ? crc16_out[10] : crc16_out[11]);
			crc16_out[12] <= (en ? crc16_out[11] : crc16_out[12]);
			crc16_out[13] <= (en ? crc16_out[12] : crc16_out[13]);
			crc16_out[14] <= (en ? crc16_out[13] : crc16_out[14]);
		end
endmodule
module CRC_DRIVER (
	clock,
	reset_n,
	select_crc,
	init,
	pause,
	bit_in,
	index,
	bit_out
);
	input wire clock;
	input wire reset_n;
	input wire select_crc;
	input wire init;
	input wire pause;
	input wire bit_in;
	input wire [6:0] index;
	output wire bit_out;
	wire en;
	wire residue_match5;
	wire residue_match16;
	assign en = ~pause;
	wire [4:0] crc5_out;
	reg [4:0] crc5_out_inv;
	wire [15:0] crc16_out;
	reg [15:0] crc16_out_inv;
	always @(*) begin
		begin : sv2v_autoblock_1
			reg signed [31:0] i;
			for (i = 0; i < 5; i = i + 1)
				crc5_out_inv[i] = ~crc5_out[i];
		end
		begin : sv2v_autoblock_2
			reg signed [31:0] j;
			for (j = 0; j < 16; j = j + 1)
				crc16_out_inv[j] = ~crc16_out[j];
		end
	end
	CRC5 crc5(
		.residue_match(residue_match5),
		.clock(clock),
		.reset_n(reset_n),
		.bit_in(bit_in),
		.en(en),
		.init(init),
		.crc5_out(crc5_out)
	);
	CRC16 crc16(
		.residue_match(residue_match16),
		.clock(clock),
		.reset_n(reset_n),
		.bit_in(bit_in),
		.en(en),
		.init(init),
		.crc16_out(crc16_out)
	);
	assign bit_out = (select_crc ? crc5_out_inv[4 - index] : crc16_out_inv[15 - index]);
endmodule
module PACKET_SEND_MANAGER (
	clock,
	reset_n,
	pause,
	send_packet,
	PID,
	encoder_or_crc,
	crc_select,
	crc_init,
	hard_init,
	stuff_en,
	nrzi_en,
	nrzi_init,
	wire_en,
	is_eop,
	packet_pause,
	crc_pause,
	packet_index,
	crc_index,
	packet_phase,
	send_done
);
	input wire clock;
	input wire reset_n;
	input wire pause;
	input wire send_packet;
	input wire [3:0] PID;
	output reg encoder_or_crc;
	output reg crc_select;
	output reg crc_init;
	output reg hard_init;
	output reg stuff_en;
	output reg nrzi_en;
	output reg nrzi_init;
	output reg wire_en;
	output reg is_eop;
	output reg packet_pause;
	output reg crc_pause;
	output reg [6:0] packet_index;
	output reg [6:0] crc_index;
	output reg [1:0] packet_phase;
	output reg send_done;
	reg [2:0] cur_state;
	reg [2:0] next_state;
	reg [6:0] count;
	reg [6:0] count_next;
	always @(*)
		case (cur_state)
			3'b000: begin
				next_state = (send_packet ? 3'b001 : 3'b000);
				count_next = 1'sb0;
			end
			3'b001: begin
				next_state = (count == 7'd7 ? 3'd2 : 3'b001);
				count_next = (count == 7'd7 ? {7 {1'sb0}} : count + 1);
			end
			3'd2:
				if (count == 7'd7) begin
					count_next = 1'sb0;
					if ((PID == 4'b1001) || (PID == 4'b0001))
						next_state = 3'd3;
					else if (PID == 4'b0011)
						next_state = 3'd5;
					else
						next_state = 3'd7;
				end
				else begin
					next_state = 3'd2;
					count_next = count + 1;
				end
			3'd3: begin
				next_state = ((count == 7'd10) && ~pause ? 3'd4 : 3'd3);
				if (pause)
					count_next = count;
				else
					count_next = (count == 7'd10 ? {7 {1'sb0}} : count + 1);
			end
			3'd4: begin
				next_state = ((count == 7'd4) && ~pause ? 3'd7 : 3'd4);
				if (pause)
					count_next = count;
				else
					count_next = (count == 7'd4 ? {7 {1'sb0}} : count + 1);
			end
			3'd6: begin
				next_state = ((count == 7'd15) && ~pause ? 3'd7 : 3'd6);
				if (pause)
					count_next = count;
				else
					count_next = (count == 7'd15 ? {7 {1'sb0}} : count + 1);
			end
			3'd5: begin
				next_state = ((count == 7'd63) && ~pause ? 3'd6 : 3'd5);
				if (pause)
					count_next = count;
				else
					count_next = (count == 7'd63 ? {7 {1'sb0}} : count + 1);
			end
			3'd7: begin
				next_state = (count == 7'd2 ? 3'b000 : 3'd7);
				if (pause)
					count_next = count;
				else
					count_next = (count == 7'd2 ? {7 {1'sb0}} : count + 1);
			end
		endcase
	always @(*) begin
		encoder_or_crc = 1'sb1;
		packet_index = 1'sb0;
		packet_phase = 1'sb0;
		crc_index = 1'sb0;
		crc_select = 1'sb0;
		crc_init = 1'sb1;
		stuff_en = 1'sb0;
		nrzi_en = 1'sb0;
		nrzi_init = 1'sb0;
		wire_en = 1'sb0;
		is_eop = 1'sb0;
		send_done = 1'sb0;
		hard_init = 1'sb0;
		packet_pause = pause;
		crc_pause = pause;
		case (cur_state)
			3'b000: begin
				packet_index = 1'sb0;
				crc_index = 1'sb0;
				crc_select = 1'sb0;
				crc_init = 1'sb1;
				stuff_en = 1'sb0;
				nrzi_en = 1'sb0;
				wire_en = 1'sb0;
				packet_phase = 1'sb0;
				nrzi_init = 1'sb1;
				hard_init = 1'sb1;
			end
			3'b001: begin
				packet_index = count;
				packet_phase = 2'b11;
				encoder_or_crc = 1'sb1;
				crc_pause = 1'sb1;
				crc_init = 1'sb1;
				stuff_en = 1'sb0;
				nrzi_en = 1'sb1;
				wire_en = 1'sb1;
			end
			3'd2: begin
				packet_index = count;
				packet_phase = 1'sb0;
				encoder_or_crc = 1'sb1;
				crc_pause = 1'sb1;
				crc_init = 1'sb1;
				stuff_en = 1'sb0;
				nrzi_en = 1'sb1;
				wire_en = 1'sb1;
			end
			3'd3: begin
				packet_index = count;
				packet_phase = 2'b01;
				encoder_or_crc = 1'sb1;
				crc_init = 1'sb0;
				stuff_en = 1'sb1;
				nrzi_en = 1'sb1;
				wire_en = 1'sb1;
			end
			3'd5: begin
				packet_index = count;
				packet_phase = 2'b10;
				encoder_or_crc = 1'sb1;
				crc_init = 1'sb0;
				stuff_en = 1'sb1;
				nrzi_en = 1'sb1;
				wire_en = 1'sb1;
			end
			3'd4: begin
				crc_index = count;
				encoder_or_crc = 1'sb0;
				crc_init = 1'sb0;
				crc_pause = 1'sb1;
				crc_select = 1'sb1;
				stuff_en = 1'sb1;
				nrzi_en = 1'sb1;
				wire_en = 1'sb1;
			end
			3'd6: begin
				crc_index = count;
				encoder_or_crc = 1'sb0;
				crc_init = 1'sb0;
				crc_pause = 1'sb1;
				crc_select = 1'sb0;
				stuff_en = 1'sb1;
				nrzi_en = 1'sb1;
				wire_en = 1'sb1;
			end
			3'd7: begin
				wire_en = 1'sb1;
				is_eop = ~pause;
				stuff_en = count == {7 {1'sb0}};
				send_done = count == 7'd2;
				crc_init = 1'sb1;
				hard_init = 1'sb1;
			end
		endcase
	end
	always @(posedge clock or negedge reset_n)
		if (~reset_n)
			cur_state <= 3'b000;
		else
			cur_state <= next_state;
	always @(posedge clock or negedge reset_n)
		if (~reset_n)
			count <= 1'sb0;
		else
			count <= count_next;
endmodule
module PACKET_ENCODER (
	clock,
	reset_n,
	pause,
	PID,
	Addr,
	ENDP,
	Payload,
	index,
	phase,
	bit_out
);
	input wire clock;
	input wire reset_n;
	input wire pause;
	input wire [3:0] PID;
	input wire [6:0] Addr;
	input wire [3:0] ENDP;
	input wire [63:0] Payload;
	input wire [6:0] index;
	input wire [1:0] phase;
	output reg bit_out;
	reg [95:0] data_register;
	reg [15:0] acknak_register;
	reg [26:0] inout_register;
	reg [3:0] PID_lsb;
	reg [3:0] PID_lsb_inv;
	reg [3:0] ENDP_lsb;
	reg [6:0] Addr_lsb;
	reg [63:0] Payload_lsb;
	reg [7:0] SYNC;
	reg [7:0] PID_full;
	reg [10:0] Addr_Endp_register;
	always @(*) begin
		begin : sv2v_autoblock_1
			reg signed [31:0] i;
			for (i = 0; i < 4; i = i + 1)
				begin
					PID_lsb[i] = PID[3 - i];
					PID_lsb_inv[i] = ~PID[3 - i];
					ENDP_lsb[i] = ENDP[3 - i];
				end
		end
		begin : sv2v_autoblock_2
			reg signed [31:0] j;
			for (j = 0; j < 7; j = j + 1)
				Addr_lsb[j] = Addr[6 - j];
		end
		begin : sv2v_autoblock_3
			reg signed [31:0] k;
			for (k = 0; k < 64; k = k + 1)
				Payload_lsb[k] = Payload[63 - k];
		end
		SYNC = 8'b00000001;
		PID_full = {PID_lsb, PID_lsb_inv};
		Addr_Endp_register = {Addr_lsb, ENDP_lsb};
	end
	always @(*) begin
		inout_register = 1'sb0;
		data_register = 1'sb0;
		acknak_register = 1'sb0;
		if ((PID == 4'b1001) || (PID == 4'b0001))
			inout_register = {SYNC, PID_lsb, PID_lsb_inv, Addr_lsb, ENDP_lsb};
		else if (PID == 4'b0011)
			data_register = {SYNC, PID_lsb, PID_lsb_inv, Payload_lsb};
		else if ((PID == 4'b0010) || (PID == 4'b1010))
			acknak_register = {SYNC, PID_lsb, PID_lsb_inv};
		else begin
			inout_register = 1'sb0;
			data_register = 1'sb0;
			acknak_register = 1'sb0;
		end
	end
	always @(*)
		if (phase == 2'b00)
			bit_out = PID_full[7 - index];
		else if (phase == 2'b01)
			bit_out = Addr_Endp_register[10 - index];
		else if (phase == 2'b10)
			bit_out = Payload_lsb[63 - index];
		else
			bit_out = SYNC[7 - index];
endmodule
module BIT_STUFFER (
	bit_in,
	en,
	clock,
	reset_n,
	hard_init,
	bit_out,
	is_stuffing
);
	input wire bit_in;
	input wire en;
	input wire clock;
	input wire reset_n;
	input wire hard_init;
	output reg bit_out;
	output reg is_stuffing;
	reg [2:0] count;
	reg [2:0] count_next;
	always @(*)
		if (count == 3'd6) begin
			bit_out = 1'sb0;
			count_next = 1'sb0;
			is_stuffing = 1'sb1;
		end
		else begin
			bit_out = bit_in;
			count_next = (bit_in ? count + 1 : {3 {1'sb0}});
			is_stuffing = 1'sb0;
		end
	always @(posedge clock or negedge reset_n)
		if (~reset_n)
			count <= 1'sb0;
		else if (hard_init)
			count <= 1'sb0;
		else
			count <= (en ? count_next : count);
endmodule
module NRZI (
	bit_in,
	en,
	init,
	clock,
	reset_n,
	bit_out
);
	input wire bit_in;
	input wire en;
	input wire init;
	input wire clock;
	input wire reset_n;
	output wire bit_out;
	reg cur_value;
	wire cur_value_next;
	assign bit_out = (bit_in ? cur_value : ~cur_value);
	assign cur_value_next = bit_out;
	always @(posedge clock or negedge reset_n)
		if (~reset_n)
			cur_value <= 1'sb1;
		else if (init)
			cur_value <= 1'sb1;
		else
			cur_value <= (en ? cur_value_next : cur_value);
endmodule
module WIRE_DRIVER (
	wires_out,
	clock,
	reset_n,
	bit_in,
	is_eop,
	en
);
	output wire [1:0] wires_out;
	input wire clock;
	input wire reset_n;
	input wire bit_in;
	input wire is_eop;
	input wire en;
	reg [1:0] cur_state;
	reg [1:0] next_state;
	reg drive_dp;
	reg drive_dm;
	assign wires_out[1] = (en ? drive_dp : 1'b1);
	assign wires_out[0] = (en ? drive_dm : 1'b0);
	always @(*)
		if (~en)
			next_state = 2'b00;
		else if (cur_state == 2'b00)
			next_state = 2'b01;
		else if (cur_state == 2'b01)
			next_state = (is_eop ? 2'b10 : 2'b01);
		else if (cur_state == 2'b10)
			next_state = 2'b11;
		else
			next_state = 2'b00;
	always @(*)
		case (cur_state)
			2'b00:
				if (en) begin
					drive_dp = 1'sb0;
					drive_dm = 1'sb1;
				end
				else begin
					drive_dp = 1'sb1;
					drive_dm = 1'sb1;
				end
			2'b01:
				if (~is_eop) begin
					if (bit_in) begin
						drive_dp = 1'sb1;
						drive_dm = 1'sb0;
					end
					else begin
						drive_dp = 1'sb0;
						drive_dm = 1'sb1;
					end
				end
				else begin
					drive_dp = 1'sb0;
					drive_dm = 1'sb0;
				end
			2'b10: begin
				drive_dp = 1'sb0;
				drive_dm = 1'sb0;
			end
			2'b11: begin
				drive_dp = 1'sb1;
				drive_dm = 1'sb0;
			end
			default: begin
				drive_dp = 1'sb1;
				drive_dm = 1'sb1;
			end
		endcase
	always @(posedge clock or negedge reset_n)
		if (~reset_n)
			cur_state <= 2'b00;
		else
			cur_state <= (en ? next_state : cur_state);
endmodule
module IN_OUT_HANDLER (
	clock,
	reset_n,
	make_read,
	make_write,
	send_done,
	receive_done,
	had_errors,
	received_PID,
	addr_ENDP,
	data_ENDP,
	received_data,
	send_data,
	addr_or_data,
	completed_transaction,
	ended_with_errors,
	send_packet,
	expect_packet,
	final_data,
	PID_to_sender,
	ENDP_to_sender,
	data_to_sender
);
	input wire clock;
	input wire reset_n;
	input wire make_read;
	input wire make_write;
	input wire send_done;
	input wire receive_done;
	input wire had_errors;
	input wire [3:0] received_PID;
	input wire [3:0] addr_ENDP;
	input wire [3:0] data_ENDP;
	input wire [63:0] received_data;
	input wire [63:0] send_data;
	input wire addr_or_data;
	output reg completed_transaction;
	output reg ended_with_errors;
	output reg send_packet;
	output reg expect_packet;
	output reg [63:0] final_data;
	output reg [3:0] PID_to_sender;
	output reg [3:0] ENDP_to_sender;
	output reg [63:0] data_to_sender;
	reg [3:0] cur_state;
	reg [3:0] next_state;
	reg [3:0] error_counter;
	reg [3:0] error_counter_nxt;
	reg [3:0] timeout_counter;
	reg [3:0] timeout_counter_nxt;
	reg [8:0] timer;
	reg [8:0] timer_nxt;
	wire timeout;
	wire end_transaction;
	assign timeout = timer == 9'd255;
	assign end_transaction = ((error_counter == 4'd6) && had_errors) || ((timeout_counter == 4'd6) && timeout);
	always @(*)
		case (cur_state)
			4'b0000:
				if (make_read)
					next_state = 4'd1;
				else if (make_write)
					next_state = 4'd2;
				else
					next_state = 4'b0000;
			4'd1: next_state = (send_done ? 4'd3 : 4'd1);
			4'd2: next_state = (send_done ? 4'd4 : 4'd2);
			4'd3:
				if (receive_done) begin
					if (had_errors)
						next_state = (end_transaction ? 4'b0000 : 4'd6);
					else
						next_state = 4'd5;
				end
				else if (timeout)
					next_state = (end_transaction ? 4'b0000 : 4'd6);
				else
					next_state = 4'd3;
			4'd4: next_state = (send_done ? 4'd7 : 4'd4);
			4'd5: next_state = (send_done ? 4'b0000 : 4'd5);
			4'd6: next_state = (send_done ? 4'd3 : 4'd6);
			4'd7:
				if (receive_done) begin
					if ((received_PID == 4'b1010) || (received_PID != 4'b0010))
						next_state = (end_transaction ? 4'b0000 : 4'd4);
					else
						next_state = 4'b0000;
				end
				else if (timeout)
					next_state = (end_transaction ? 4'b0000 : 4'd4);
				else
					next_state = 4'd7;
			default: next_state = 4'b0000;
		endcase
	always @(*) begin
		ENDP_to_sender = (addr_or_data ? addr_ENDP : data_ENDP);
		case (cur_state)
			4'b0000: begin
				error_counter_nxt = 1'sb0;
				timeout_counter_nxt = 1'sb0;
				timer_nxt = 1'sb0;
				send_packet = next_state != 4'b0000;
				expect_packet = 1'sb0;
				PID_to_sender = (next_state == 4'd1 ? 4'b1001 : 4'b0001);
				data_to_sender = 1'sb0;
			end
			4'd1: begin
				error_counter_nxt = 1'sb0;
				timeout_counter_nxt = 1'sb0;
				timer_nxt = 1'sb0;
				send_packet = 1'sb0;
				expect_packet = next_state != 4'd1;
				PID_to_sender = 4'b1001;
				data_to_sender = 1'sb0;
			end
			4'd2: begin
				error_counter_nxt = 1'sb0;
				timeout_counter_nxt = 1'sb0;
				timer_nxt = 1'sb0;
				send_packet = 1'sb1;
				expect_packet = 1'sb0;
				PID_to_sender = (next_state != 4'd2 ? 4'b0011 : 4'b0001);
				data_to_sender = send_data;
			end
			4'd3: begin
				error_counter_nxt = (had_errors ? error_counter + 1 : error_counter);
				timeout_counter_nxt = (timeout ? timeout_counter + 1 : timeout_counter);
				timer_nxt = timer + 1;
				send_packet = 1'sb0;
				expect_packet = next_state == 4'd3;
				if (next_state == 4'd5)
					PID_to_sender = 4'b0010;
				else if (next_state == 4'd6)
					PID_to_sender = 4'b1010;
				else
					PID_to_sender = 1'sb0;
				data_to_sender = 1'sb0;
			end
			4'd4: begin
				error_counter_nxt = error_counter;
				timeout_counter_nxt = timeout_counter;
				timer_nxt = 1'sb0;
				send_packet = next_state == 4'd4;
				expect_packet = next_state != 4'd4;
				data_to_sender = send_data;
				PID_to_sender = 4'b0011;
			end
			4'd5: begin
				error_counter_nxt = error_counter;
				timeout_counter_nxt = timeout_counter;
				timer_nxt = 1'sb0;
				send_packet = next_state == 4'd5;
				expect_packet = 1'sb0;
				data_to_sender = 1'sb0;
				PID_to_sender = 4'b0010;
			end
			4'd6: begin
				error_counter_nxt = error_counter;
				timeout_counter_nxt = timeout_counter;
				timer_nxt = 1'sb0;
				send_packet = next_state == 4'd6;
				expect_packet = next_state == 4'd3;
				data_to_sender = 1'sb0;
				PID_to_sender = 4'b1010;
			end
			4'd7: begin
				error_counter_nxt = (had_errors ? error_counter + 1 : error_counter);
				timeout_counter_nxt = (timeout ? timeout_counter + 1 : timeout_counter);
				timer_nxt = timer + 1;
				send_packet = 1'sb0;
				expect_packet = next_state == 4'd7;
				data_to_sender = (next_state == 4'd4 ? send_data : {64 {1'sb0}});
				PID_to_sender = (next_state == 4'd4 ? 4'b0011 : {4 {1'sb0}});
			end
			default: begin
				error_counter_nxt = 1'sb0;
				timeout_counter_nxt = 1'sb0;
				timer_nxt = 1'sb0;
				send_packet = 1'sb0;
				expect_packet = 1'sb0;
				PID_to_sender = 1'sb0;
				data_to_sender = 1'sb0;
			end
		endcase
	end
	always @(*) begin
		completed_transaction = (((cur_state == 4'd3) || (cur_state == 4'd7)) || (cur_state == 4'd5)) && (next_state == 4'b0000);
		ended_with_errors = completed_transaction && (timeout || (had_errors && (received_PID != 4'b0010)));
	end
	always @(posedge clock or negedge reset_n)
		if (~reset_n)
			final_data <= 1'sb0;
		else
			final_data <= (receive_done ? received_data : final_data);
	always @(posedge clock or negedge reset_n)
		if (~reset_n)
			cur_state <= 4'b0000;
		else
			cur_state <= next_state;
	always @(posedge clock or negedge reset_n)
		if (~reset_n) begin
			error_counter <= 1'sb0;
			timeout_counter <= 1'sb0;
			timer <= 1'sb0;
		end
		else begin
			error_counter <= error_counter_nxt;
			timeout_counter <= timeout_counter_nxt;
			timer <= timer_nxt;
		end
endmodule
module READ_WRITE_HANDLER (
	clock,
	reset_n,
	do_read,
	do_write,
	txn_done,
	txn_error,
	in_data,
	mempage,
	memdata,
	make_in,
	make_out,
	addr_or_data,
	finished,
	finished_with_error,
	no_transaction,
	out_data,
	final_data
);
	input wire clock;
	input wire reset_n;
	input wire do_read;
	input wire do_write;
	input wire txn_done;
	input wire txn_error;
	input wire [63:0] in_data;
	input wire [15:0] mempage;
	input wire [63:0] memdata;
	output reg make_in;
	output reg make_out;
	output reg addr_or_data;
	output reg finished;
	output reg finished_with_error;
	output wire no_transaction;
	output reg [63:0] out_data;
	output wire [63:0] final_data;
	reg [2:0] cur_state;
	reg [2:0] next_state;
	wire [63:0] page_data;
	assign page_data = {mempage, 48'd0};
	assign no_transaction = cur_state == 3'b000;
	always @(*)
		case (cur_state)
			3'b000:
				if (do_read)
					next_state = 3'd1;
				else if (do_write)
					next_state = 3'd3;
				else
					next_state = 3'b000;
			3'd1:
				if (txn_done && txn_error)
					next_state = 3'b000;
				else if (txn_done)
					next_state = 3'd2;
				else
					next_state = 3'd1;
			3'd2: next_state = (txn_done ? 3'b000 : 3'd2);
			3'd3:
				if (txn_done && txn_error)
					next_state = 3'b000;
				else if (txn_done)
					next_state = 3'd4;
				else
					next_state = 3'd3;
			3'd4: next_state = (txn_done ? 3'b000 : 3'd4);
			default: next_state = 3'b000;
		endcase
	always @(*)
		case (cur_state)
			3'b000: begin
				finished = 1'sb0;
				finished_with_error = 1'sb0;
				make_out = next_state != 3'b000;
				make_in = 1'sb0;
				out_data = (next_state != 3'b000 ? page_data : {64 {1'sb0}});
				addr_or_data = next_state != 3'b000;
			end
			3'd1: begin
				finished_with_error = (next_state == 3'b000) && txn_error;
				finished = next_state == 3'b000;
				make_out = txn_done && !txn_error;
				make_in = 1'sb0;
				out_data = (txn_done ? memdata : page_data);
				addr_or_data = ~txn_done;
			end
			3'd2: begin
				finished_with_error = txn_error;
				finished = txn_done;
				make_out = 1'sb0;
				make_in = next_state != 3'b000;
				out_data = memdata;
				addr_or_data = 1'sb0;
			end
			3'd3: begin
				finished_with_error = (next_state == 3'b000) && txn_error;
				finished = next_state == 3'b000;
				make_out = txn_done && !txn_error;
				make_in = 1'sb0;
				out_data = (txn_done ? memdata : page_data);
				addr_or_data = ~txn_done;
			end
			3'd4: begin
				finished_with_error = txn_error;
				finished = txn_done;
				make_out = next_state != 3'b000;
				make_in = 1'sb0;
				out_data = memdata;
				addr_or_data = 1'sb0;
			end
			default: begin
				finished_with_error = 1'sb0;
				finished = 1'sb0;
				make_out = 1'sb0;
				make_in = 1'sb0;
				out_data = 1'sb0;
				addr_or_data = 1'sb0;
			end
		endcase
	assign final_data = in_data;
	always @(posedge clock or negedge reset_n)
		if (~reset_n)
			cur_state <= 3'b000;
		else
			cur_state <= next_state;
endmodule
module WIRE_LISTENER (
	clock,
	reset_n,
	wires_in,
	bit_out,
	invalid,
	saw_eop
);
	input wire clock;
	input wire reset_n;
	input wire [1:0] wires_in;
	output reg bit_out;
	output reg invalid;
	output wire saw_eop;
	wire dp;
	wire dm;
	assign dp = wires_in[1];
	assign dm = wires_in[0];
	reg [2:0] dp_log;
	reg [2:0] dm_log;
	always @(posedge clock or negedge reset_n)
		if (!reset_n) begin
			dp_log <= 1'sb0;
			dm_log <= 1'sb0;
		end
		else begin
			dp_log <= (dp_log << 1) + dp;
			dm_log <= (dm_log << 1) + dm;
		end
	always @(*)
		case ({dp, dm})
			2'b00: begin
				bit_out = 1'sb0;
				invalid = 1'sb1;
			end
			2'b01: begin
				bit_out = 1'sb0;
				invalid = 1'sb0;
			end
			2'b10: begin
				bit_out = 1'sb1;
				invalid = 1'sb0;
			end
			2'b11: begin
				bit_out = 1'sb1;
				invalid = 1'sb1;
			end
		endcase
	assign saw_eop = (dp_log == 3'b001) & (dm_log == 3'b000);
endmodule
module SYNC_DETECTOR (
	clock,
	reset_n,
	bit_in,
	saw_sync
);
	input wire clock;
	input wire reset_n;
	input wire bit_in;
	output wire saw_sync;
	reg [7:0] log;
	always @(posedge clock or negedge reset_n)
		if (!reset_n)
			log <= 1'sb0;
		else
			log <= (log << 1) + bit_in;
	assign saw_sync = (log[6:0] == 7'b0101010) & (bit_in == 1'b0);
endmodule
module NRZI_DECODER (
	clock,
	reset_n,
	bit_in,
	en,
	init,
	bit_out
);
	input wire clock;
	input wire reset_n;
	input wire bit_in;
	input wire en;
	input wire init;
	output wire bit_out;
	reg cur_value;
	wire cur_value_next;
	assign bit_out = cur_value == bit_in;
	assign cur_value_next = bit_in;
	always @(posedge clock or negedge reset_n)
		if (~reset_n)
			cur_value <= 1'sb1;
		else if (en)
			cur_value <= (init ? 1'b0 : cur_value_next);
		else
			cur_value <= cur_value;
endmodule
module BIT_UNSTUFFER (
	clock,
	reset_n,
	en,
	bit_in,
	hard_init,
	bit_out,
	is_stuffing
);
	input wire clock;
	input wire reset_n;
	input wire en;
	input wire bit_in;
	input wire hard_init;
	output reg bit_out;
	output reg is_stuffing;
	reg [2:0] count;
	reg [2:0] count_next;
	always @(*)
		if (count == 3'd6) begin
			bit_out = 1'sb0;
			count_next = 1'sb0;
			is_stuffing = 1'sb1;
		end
		else begin
			bit_out = bit_in;
			count_next = (bit_in ? count + 1 : {3 {1'sb0}});
			is_stuffing = 1'sb0;
		end
	always @(posedge clock or negedge reset_n)
		if (~reset_n)
			count <= 1'sb0;
		else if (hard_init)
			count <= 1'sb0;
		else
			count <= (en ? count_next : count);
endmodule
module PACKET_RECEIVER (
	clock,
	reset_n,
	bit_in,
	phase,
	pid,
	pid_to_fsm,
	addr_endp,
	payload,
	PID_ERROR
);
	input wire clock;
	input wire reset_n;
	input wire bit_in;
	input wire [1:0] phase;
	output wire [3:0] pid;
	output wire [3:0] pid_to_fsm;
	output wire [10:0] addr_endp;
	output wire [63:0] payload;
	output wire PID_ERROR;
	wire [1:0] temp;
	reg [7:0] PID_accum;
	reg [10:0] ADDR_ENDP_accum;
	reg [63:0] PAYLOAD_accum;
	wire [3:0] pid_inv;
	assign pid = PID_accum[3:0];
	assign pid_inv = PID_accum[7:4];
	assign addr_endp = ADDR_ENDP_accum;
	assign payload = PAYLOAD_accum;
	assign pid_to_fsm = PID_accum[4:1];
	assign PID_ERROR = PID_accum[3:0] != PID_accum[7:4];
	always @(posedge clock or negedge reset_n)
		if (!reset_n) begin
			PID_accum <= 1'sb0;
			ADDR_ENDP_accum <= 1'sb0;
			PAYLOAD_accum <= 1'sb0;
		end
		else
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
endmodule
module PACKET_RECEIVE_MANAGER (
	clock,
	reset_n,
	saw_sync,
	saw_eop,
	pause,
	enable,
	PID,
	packet_phase,
	crc_select,
	crc_init,
	crc_pause,
	stuff_en,
	nrzi_en,
	nrzi_init,
	hard_init,
	send_done
);
	input wire clock;
	input wire reset_n;
	input wire saw_sync;
	input wire saw_eop;
	input wire pause;
	input wire enable;
	input wire [3:0] PID;
	output reg [1:0] packet_phase;
	output reg crc_select;
	output reg crc_init;
	output reg crc_pause;
	output reg stuff_en;
	output reg nrzi_en;
	output reg nrzi_init;
	output reg hard_init;
	output reg send_done;
	reg [2:0] cur_state;
	reg [2:0] next_state;
	reg [6:0] count;
	reg [6:0] count_next;
	always @(*)
		case (cur_state)
			3'b000: begin
				next_state = (saw_sync & enable ? 3'd1 : 3'b000);
				count_next = 1'sb0;
			end
			3'd1:
				if (count == 7'd7) begin
					count_next = 1'sb0;
					if ((PID == 4'b1001) || (PID == 4'b0001))
						next_state = 3'd2;
					else if (PID == 4'b0011)
						next_state = 3'd3;
					else if ((PID == 4'b1010) || (PID == 4'b0010))
						next_state = 3'd6;
					else
						next_state = 3'd7;
				end
				else begin
					next_state = 3'd1;
					count_next = count + 1;
				end
			3'd2: begin
				next_state = ((count == 7'd10) && ~pause ? 3'd4 : 3'd2);
				if (pause)
					count_next = count;
				else
					count_next = (count == 7'd10 ? {7 {1'sb0}} : count + 1);
			end
			3'd3: begin
				next_state = ((count == 7'd63) && ~pause ? 3'd5 : 3'd3);
				if (pause)
					count_next = count;
				else
					count_next = (count == 7'd63 ? {7 {1'sb0}} : count + 1);
			end
			3'd4: begin
				next_state = ((count == 7'd4) && ~pause ? 3'd6 : 3'd4);
				if (pause)
					count_next = count;
				else
					count_next = (count == 7'd4 ? {7 {1'sb0}} : count + 1);
			end
			3'd5: begin
				next_state = ((count == 7'd15) && ~pause ? 3'd6 : 3'd5);
				if (pause)
					count_next = count;
				else
					count_next = (count == 7'd15 ? {7 {1'sb0}} : count + 1);
			end
			3'd6: begin
				next_state = (count == 7'd2 ? 3'b000 : 3'd6);
				if (pause)
					count_next = count;
				else
					count_next = (count == 7'd2 ? {7 {1'sb0}} : count + 1);
			end
			3'd7: begin
				next_state = (saw_eop ? 3'b000 : 3'd7);
				count_next = (saw_eop ? {7 {1'sb0}} : count + 1);
			end
		endcase
	always @(*) begin
		packet_phase = 1'sb0;
		crc_select = 1'sb0;
		crc_init = 1'sb1;
		stuff_en = 1'sb0;
		nrzi_en = 1'sb1;
		nrzi_init = 1'sb1;
		send_done = 1'sb0;
		hard_init = 1'sb0;
		crc_pause = pause;
		case (cur_state)
			3'b000: begin
				packet_phase = 1'sb0;
				crc_select = 1'sb0;
				crc_init = 1'sb1;
				stuff_en = 1'sb0;
				nrzi_en = 1'sb1;
				nrzi_init = ~(saw_sync & enable);
				send_done = 1'sb0;
				hard_init = 1'sb1;
			end
			3'd1: begin
				packet_phase = (pause ? {2 {1'sb0}} : 2'd1);
				crc_select = 1'sb0;
				crc_init = 1'sb1;
				stuff_en = 1'sb0;
				nrzi_en = 1'sb1;
				nrzi_init = 1'sb0;
				send_done = 1'sb0;
				crc_pause = pause;
			end
			3'd2: begin
				packet_phase = (pause ? {2 {1'sb0}} : 2'd2);
				crc_select = 1'sb0;
				crc_init = 1'sb0;
				stuff_en = 1'sb1;
				nrzi_en = 1'sb1;
				nrzi_init = 1'sb0;
				send_done = 1'sb0;
				crc_pause = pause;
			end
			3'd3: begin
				packet_phase = (pause ? {2 {1'sb0}} : 2'd3);
				crc_select = 1'sb0;
				crc_init = 1'sb0;
				stuff_en = 1'sb1;
				nrzi_en = 1'sb1;
				nrzi_init = 1'sb0;
				send_done = 1'sb0;
				crc_pause = pause;
			end
			3'd4: begin
				packet_phase = 1'sb0;
				crc_select = 1'sb0;
				crc_init = 1'sb0;
				stuff_en = 1'sb1;
				nrzi_en = 1'sb1;
				nrzi_init = 1'sb0;
				send_done = 1'sb0;
				crc_pause = pause;
			end
			3'd5: begin
				packet_phase = 1'sb0;
				crc_select = 1'sb0;
				crc_init = 1'sb0;
				stuff_en = 1'sb1;
				nrzi_en = 1'sb1;
				nrzi_init = 1'sb0;
				send_done = 1'sb0;
				crc_pause = pause;
			end
			3'd6: begin
				packet_phase = 1'sb0;
				crc_select = 1'sb0;
				crc_init = 1'sb0;
				stuff_en = 1'sb0;
				nrzi_en = 1'sb1;
				nrzi_init = 1'sb0;
				send_done = count == 7'd2;
				crc_pause = 1'sb1;
				hard_init = 1'sb1;
			end
			3'd7: begin
				packet_phase = 1'sb0;
				crc_select = 1'sb0;
				crc_init = 1'sb0;
				stuff_en = 1'sb0;
				nrzi_en = 1'sb1;
				nrzi_init = 1'sb0;
				crc_pause = 1'sb1;
				send_done = saw_eop;
			end
		endcase
	end
	always @(posedge clock or negedge reset_n)
		if (~reset_n)
			cur_state <= 3'b000;
		else
			cur_state <= next_state;
	always @(posedge clock or negedge reset_n)
		if (~reset_n)
			count <= 1'sb0;
		else
			count <= count_next;
endmodule
module CRC_DRIVER_RECEIVE (
	clock,
	reset_n,
	select_crc,
	init,
	pause,
	bit_in,
	index,
	bit_out,
	residue_match5,
	residue_match16
);
	input wire clock;
	input wire reset_n;
	input wire select_crc;
	input wire init;
	input wire pause;
	input wire bit_in;
	input wire [6:0] index;
	output wire bit_out;
	output wire residue_match5;
	output wire residue_match16;
	wire en;
	assign en = ~pause;
	wire [4:0] crc5_out;
	reg [4:0] crc5_out_inv;
	wire [15:0] crc16_out;
	reg [15:0] crc16_out_inv;
	always @(*) begin
		begin : sv2v_autoblock_1
			reg signed [31:0] i;
			for (i = 0; i < 5; i = i + 1)
				crc5_out_inv[i] = ~crc5_out[i];
		end
		begin : sv2v_autoblock_2
			reg signed [31:0] j;
			for (j = 0; j < 16; j = j + 1)
				crc16_out_inv[j] = ~crc16_out[j];
		end
	end
	CRC5 crc5(
		.residue_match(residue_match5),
		.clock(clock),
		.reset_n(reset_n),
		.bit_in(bit_in),
		.en(en),
		.init(init),
		.crc5_out(crc5_out)
	);
	CRC16 crc16(
		.residue_match(residue_match16),
		.clock(clock),
		.reset_n(reset_n),
		.bit_in(bit_in),
		.en(en),
		.init(init),
		.crc16_out(crc16_out)
	);
	assign bit_out = (select_crc ? crc5_out_inv[4 - index] : crc16_out_inv[15 - index]);
endmodule
module PACKET_RECEIVE (
	clock,
	reset_n,
	expect_packet,
	PID,
	ENDP,
	Addr,
	Payload,
	receive_done,
	had_errors,
	wires_in
);
	input wire clock;
	input wire reset_n;
	input wire expect_packet;
	output wire [3:0] PID;
	output wire [3:0] ENDP;
	output wire [6:0] Addr;
	output wire [63:0] Payload;
	output wire receive_done;
	output wire had_errors;
	input wire [1:0] wires_in;
	wire wire_out;
	wire saw_eop;
	WIRE_LISTENER wire_in(
		.bit_out(wire_out),
		.invalid(),
		.clock(clock),
		.reset_n(reset_n),
		.wires_in(wires_in),
		.saw_eop(saw_eop)
	);
	wire saw_sync;
	SYNC_DETECTOR find_sync(
		.bit_in(wire_out),
		.clock(clock),
		.reset_n(reset_n),
		.saw_sync(saw_sync)
	);
	wire nrzi_out;
	wire nrzi_en;
	wire nrzi_init;
	NRZI_DECODER nrzi(
		.bit_in(wire_out),
		.bit_out(nrzi_out),
		.en(nrzi_en),
		.init(nrzi_init),
		.clock(clock),
		.reset_n(reset_n)
	);
	wire stuff_out;
	wire stuff_en;
	wire pause;
	wire hard_init;
	BIT_UNSTUFFER bit_unstuff(
		.bit_in(nrzi_out),
		.bit_out(stuff_out),
		.en(stuff_en),
		.is_stuffing(pause),
		.clock(clock),
		.reset_n(reset_n),
		.hard_init(hard_init)
	);
	wire [1:0] phase;
	wire [3:0] pid;
	wire [3:0] pid_to_fsm;
	wire [10:0] addr_endp;
	wire [63:0] payload;
	wire PID_ERROR;
	PACKET_RECEIVER packet_decode(
		.bit_in(stuff_out),
		.clock(clock),
		.reset_n(reset_n),
		.phase(phase),
		.pid(pid),
		.pid_to_fsm(pid_to_fsm),
		.addr_endp(addr_endp),
		.payload(payload),
		.PID_ERROR(PID_ERROR)
	);
	wire crc_select;
	wire crc_init;
	wire residue_match5;
	wire residue_match16;
	wire crc_pause;
	localparam [6:0] sv2v_uu_crc_ext_index_0 = 1'sb0;
	CRC_DRIVER_RECEIVE crc(
		.bit_in(stuff_out),
		.bit_out(),
		.index(sv2v_uu_crc_ext_index_0),
		.select_crc(crc_select),
		.init(crc_init),
		.pause(crc_pause),
		.clock(clock),
		.reset_n(reset_n),
		.residue_match5(residue_match5),
		.residue_match16(residue_match16)
	);
	PACKET_RECEIVE_MANAGER fsm(
		.enable(expect_packet),
		.PID(pid_to_fsm),
		.send_done(receive_done),
		.packet_phase(phase),
		.clock(clock),
		.reset_n(reset_n),
		.saw_sync(saw_sync),
		.saw_eop(saw_eop),
		.pause(pause),
		.crc_select(crc_select),
		.crc_init(crc_init),
		.crc_pause(crc_pause),
		.stuff_en(stuff_en),
		.nrzi_en(nrzi_en),
		.nrzi_init(nrzi_init),
		.hard_init(hard_init)
	);
	assign PID = pid;
	assign Addr = addr_endp[6:0];
	assign ENDP = addr_endp[10:7];
	assign Payload = payload;
	assign had_errors = (~residue_match5 & ~residue_match16) & receive_done;
endmodule