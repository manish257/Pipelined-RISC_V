`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/14/2021 11:08:51 AM
// Design Name: 
// Module Name: reg_mem_wb
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


module reg_mem_wb(
	input  wire clk,
	input  wire rst,
	input  wire [31:0] Data_Out,
	input  wire [31:0] ALUResultM,
    input  wire [4:0] RdM,     
    input  wire [31:0] PCPlus4M,
	input  wire ResultSrcM,  
    input  wire RegWriteM,
	
	
	output reg [31:0] ReadDataW,
	output reg [31:0] ALUResultW,
    output reg [31:0]PCPlus4W,
    output reg [4:0] RdW,
    output reg ResultSrcW,
	output reg RegWriteW
	
);

	always @ (posedge clk) begin
		if(rst) begin
			ReadDataW<=0;  
			ALUResultW<=0;
			RdW<=0;
			PCPlus4W<=0; 
			ResultSrcW<=0;   
			RegWriteW<=0;          
			
		end else
		 begin
			ReadDataW<=Data_Out;  
            ALUResultW<=ALUResultM;
            RdW<=RdM;
            PCPlus4W<=PCPlus4M; 
            ResultSrcW<=ResultSrcM;   
            RegWriteW<=RegWriteM; 
		end
	end
endmodule
