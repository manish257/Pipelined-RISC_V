`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 10/20/2021 11:30:02 AM
// Design Name: 
// Module Name: register_file
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

module register_file(Port_A, Port_B, hardwire, RESULT, Addr_A, Addr_B, Addr_Wr, wr, clk,rst);
output reg [31:0] Port_A, Port_B,hardwire; 
input [31:0] RESULT; 
input [4:0] Addr_A, Addr_B, Addr_Wr; 
input wr, clk,rst; 
reg [31:0] Reg_File [31:0]; 
always @ (*) begin
Reg_File[0] <= 0;
Port_A <= Reg_File[Addr_A[4:0]];
Port_B <= Reg_File[Addr_B[4:0]];
hardwire= Reg_File[8];
end
 always @(negedge clk) begin
 if(wr)
    Reg_File[Addr_Wr]=RESULT[31:0];
 end
endmodule
/*

module register_file(input [4:0]Addr_A, Addr_B, Addr_Wr,input [31:0]RESULT,input wr,clk,rst,output reg [31:0]Port_A, Port_B);
reg [31:0] Reg_Fie [31:0];
integer i;
always@ (*)begin
Port_A= Reg_Fie[Addr_A];
Port_A= Reg_Fie[Addr_B];
 end
always @ (posedge clk) 
begin
    if (rst)
        begin
            for (i=0;i<=31;i=i+1)
                Reg_Fie[i]<= 32'b0; 
        end
    else if (wr)
        begin
            Reg_Fie[Addr_Wr]<= RESULT;
        end
end
endmodule

*/