`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 10/20/2021 11:20:20 AM
// Design Name: 
// Module Name: address_gen
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


module address_gen (output reg [31:0] pc, input [31:0] PCNext,input clk,rst); 
 // Write your code here. This is the part of Phase-I
 

 
 always@(posedge clk)
    begin
	if (rst)
		pc =0;
	else
	   pc = PCNext;
	
    end
endmodule
