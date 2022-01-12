`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 10/20/2021 11:02:55 AM
// Design Name: 
// Module Name: instruction_memory
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


module instruction_memory (input [31:0] pc,output reg [31:0] instruction);

 always@(*) begin
 case(pc)
 
 32'h0: instruction =  32'h00C00413;  	//addi x8 , x0 , 12
 32'h4: instruction =  32'h00900493; 	//addi x9 , x0 , 9
// //gcd :
// 32'h8: instruction =  32'h00940C63; 	//beq x8 , x9 , stop	
// 32'hC: instruction =  32'h00944663; 	//blt x8 , x9 , less  
// 32'h10: instruction = 32'h40940433; 	//sub x8 , x8 , x9	
// 32'h14: instruction = 32'hFE000AE3; 	//beq x0 , x0 , gcd
// //less :
// 32'h18: instruction = 32'h408484B3; 	//sub x9 , x9 , x8	
// 32'h1C: instruction = 32'hFE0006E3; 	//beq x0 , x0 , gcd	
// //stop :
// 32'h20: instruction = 32'h00000063; 	//beq x0 , x0 , stop	

 endcase
 end
endmodule


