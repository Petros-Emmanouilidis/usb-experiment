`default_nettype none

module my_chip (
    input logic [11:0] io_in, // Inputs to your chip
    output logic [11:0] io_out, // Outputs from your chip
    input logic clock,
    input logic reset // Important: Reset is ACTIVE-HIGH
);
    
    USBHost design(.clock(clock), .reset_n(~reset),
                   .read(io_in[11]), .write(io_in[10]), .mode(io_in[9:6]), .data_in(io_in[5:2]), .wires_in(io_in[1:0]),
                   .status(io_out[11:10]), .data_indx(io_out[9:6]), .data_out(io_out[5:2]), .wires_out(io_out[1:0]));

endmodule
