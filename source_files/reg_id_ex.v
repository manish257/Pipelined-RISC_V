`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/13/2021 10:07:26 PM
// Design Name: 
// Module Name: reg_id_ex
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


module reg_id_ex (

	input  wire clk,
    input  wire rst,
	input  wire [31:0] Port_A, Port_B,
	input  wire [31:0] pcD,
	input  wire [31:0] PCPlus4D,
	input  wire [31:0] instructionD,
	input  wire [31:0] immext,
	input  wire PCSrc, MemWrite, ALUSrc, RegWrite,
	input  wire [3:0] ALUControl,
	input  wire ResultSrc,
	input  wire [6:0] opcode,
	input  wire [2:0] func3,
	
	output reg [31:0] RD1E, RD2E,
	output reg [31:0] pcE,
	output reg [31:0] PCPlus4E,
	output reg [4:0] RdE,
	output reg [31:0] ImmExtE,
	output reg PCSrcE, MemWriteE, ALUSrcE, RegWriteE,
	output reg [3:0] ALUControlE,
	output reg ResultSrcE,
    output reg [6:0] opcodeE,
    output reg [2:0] func3E
);

	always @ (posedge clk) begin
		if (rst) begin
			//ex_aluop      <= `EXE_NOP_OP;<= 0;
			//ex_alusel     <= `EXE_RES_NOP;
			RD1E<=0;
			RD2E<=0;
            pcE<=0;       
            PCPlus4E<=0; 
            RdE<=0;       
            ImmExtE<=0;   
			PCSrcE <=0;
			MemWriteE <=0;
			ALUSrcE <=0;
			RegWriteE <=0;
            ALUControlE <=0;
            ResultSrcE <=0;
            opcodeE <=0;
            func3E <=0;    
		end 
		else
		 begin
		    RD1E<=Port_A;
            RD2E<=Port_B;
            pcE<=pcD;       
            PCPlus4E<=PCPlus4D; 
            RdE<= instructionD[11:7];       
            ImmExtE<=0;
			PCSrcE <=PCSrc;
            MemWriteE <=MemWrite;
            ALUSrcE <=ALUSrc;
            RegWriteE <=RegWrite;
            ALUControlE <=ALUControl;
            ResultSrcE <=ResultSrc;    
			opcodeE <= opcode;
			func3E <= func3;
			
		end
	end

endmodule
