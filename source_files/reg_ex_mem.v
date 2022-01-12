`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/13/2021 11:11:10 PM
// Design Name: 
// Module Name: reg_ex_mem
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


module reg_ex_mem(
	input  wire clk,
	input  wire rst,
	input  wire [31:0] ALU_Out,
	input  wire [31:0]RD2E,
	input  wire [4:0] RdE,
	input  wire [31:0] PCPlus4E,
	input  wire ResultSrcE,
	input  wire RegWriteE,
	input  wire MemWriteE,
	
	output reg [31:0] ALUResultM,
    output reg [31:0] WriteDataM,
    output reg [4:0] RdM,
    output reg [31:0] PCPlus4M,
	output reg ResultSrcM,
    output reg RegWriteM,
    output reg MemWriteM
);

	always @ (posedge clk) begin
		if(rst) begin
			ALUResultM <= 0;
            WriteDataM <= 0;
            RdM <= 0;
            PCPlus4M <= 0;
            ResultSrcM <= 0;
            RegWriteM <= 0;
            MemWriteM <=0;
		end else begin
			ALUResultM <= ALU_Out;
            WriteDataM <= RD2E;
            RdM <= RdE;
            PCPlus4M <= PCPlus4E;
            ResultSrcM <= ResultSrcE;
            RegWriteM <= RegWriteE;
            MemWriteM <=MemWriteE;
		end
	end

endmodule
