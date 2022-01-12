`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 01/02/2022 09:57:45 PM
// Design Name: 
// Module Name: CSR
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


module CSR(input [31:0] pcE,
           input interrupt,mret,clk,
           input [31:0] operand,
           output reg [31:0] epc,r_data,
           output stopped_interrupt,interrupt_en);

    reg delay;
    reg [31:0] ret;
    reg [31:0] CSR_IP;
    assign interrupt_en = mret | stopped_interrupt;
    always @(posedge clk)
        delay = interrupt; 
    assign stopped_interrupt = ~delay & interrupt;
    always @(*)
    begin
        if(stopped_interrupt)
        begin
            epc = 32'd52;  //ISR address
            ret = pcE;
        end
        if(mret)
            epc = ret;
    end
endmodule