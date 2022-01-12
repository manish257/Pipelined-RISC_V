`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/14/2021 11:29:08 AM
// Design Name: 
// Module Name: Mux3x1
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


module Mux3x1(out, in1, in2, in3, sel);

input [31:0] in1, in2, in3; //replace n with number of bits-1
input [2:0] sel;
output [31:0]out; //replace n with number of bits-1

assign out = 	(sel == 2'b00) ? in1 :
		(sel == 2'b01) ? in2 : 
		(sel == 2'b10) ? in3 : 31'bx; //replace n with number of bits

endmodule
