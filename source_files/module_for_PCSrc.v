`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/14/2021 09:04:46 PM
// Design Name: 
// Module Name: module_for_PCSrc
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


module module_for_PCSrc(
    input ZeroOut,
    input CarryOut,
    input [6:0] opcodeE,
    input [2:0] func3E,
    
    output reg PCSrc
    );

    always @(*)
            begin
                casex({opcodeE,func3E})
                    10'b1101111xxx, 10'b1100111xxx: PCSrc = 1;   //J type instruction
                    10'b1100011000: PCSrc = ZeroOut; //BEQ
                    10'b1100011001: PCSrc = ~ZeroOut; //BNE
                    10'b1100011101, 10'b1100011111 : PCSrc = (~CarryOut) || ZeroOut; //BGE
                    10'b1100011100, 10'b1100011111: PCSrc =CarryOut;
                    default: PCSrc=0;
                endcase
    
            end
endmodule
