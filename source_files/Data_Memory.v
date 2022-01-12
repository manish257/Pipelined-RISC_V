`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 10/20/2021 11:48:58 AM
// Design Name: 
// Module Name: Data_Memory
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

module Data_Memory(output [31:0] Data_Out, 
input [31:0] Data_In, input [31:0] D_Addr, 
input wr, rst, input clk);
 reg [31:0] Mem [255:0]; 
 integer i;
assign Data_Out=Mem[D_Addr]; 
 always @(posedge clk) begin
 if(rst) begin
 for(i=0;i<256;i=i+1)
 Mem[i]=32'h0;
 end else if (wr) begin
 Mem[D_Addr]<=Data_In;
 end
 end
endmodule