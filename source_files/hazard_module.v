`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 01/12/2022 09:33:42 PM
// Design Name: 
// Module Name: hazard_module
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


module hazard_module(
           input [4:0] rs1D,rs2D,rs1E,rs2E,rdE,rdM,rdW,
           input RegWriteM,RegWriteW,
           input ResultSrcE0,stopped_interrupt,interrupt_en,
           input PCSrcE,
           output StallF,StallD,FlushE,FlushD,FlushM,
           output reg [1:0] ForwardAE,ForwardBE);
           
    always @(*)
    begin
        if(((rs1E == rdM) & RegWriteM) & (rs1E != 0))
            ForwardAE = 2'b10;
        else if(((rs1E == rdW) & RegWriteW) & (rs1E != 0))
            ForwardAE = 2'b01;
        else
            ForwardAE = 2'b00;
		if(((rs2E == rdM) & RegWriteM) & (rs2E != 0))
            ForwardBE = 2'b10;
        else if(((rs2E == rdW) & RegWriteW) & (rs2E != 0))
            ForwardBE = 2'b01;
        else
            ForwardBE = 2'b00;
    end
	
    wire lwStall;
    assign lwStall = ResultSrcE0 & ((rs1D == rdE) | (rs2D == rdE));
	assign StallF = lwStall;
	assign StallD = lwStall;
	
	assign FlushD = PCSrcE | stopped_interrupt | interrupt_en;
	assign FlushE = lwStall | PCSrcE | stopped_interrupt;
	assign FlushM = stopped_interrupt;
endmodule
