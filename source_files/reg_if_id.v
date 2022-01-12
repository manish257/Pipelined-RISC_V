`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/13/2021 09:46:28 PM
// Design Name: 
// Module Name: reg_if_id
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


module reg_if_id(
	input  wire clk,
	input  wire rst,
	input  wire [31:0] pc,
	input  wire [31:0]instruction,
	input  wire [31:0] PCPlus4,
	//input  wire br,
	output reg  [31:0] pcD,
	output reg  [31:0] instructionD,
	output reg  [31:0] PCPlus4D
);

	always @ (posedge clk) begin
		if (rst) begin
			pcD   <= 0;
			instructionD <= 0;
			PCPlus4D <= 0;
		end 
		else 
		begin
			pcD   <= pc;
			instructionD <= instruction;
			PCPlus4D <= PCPlus4;
		end
	end
endmodule
