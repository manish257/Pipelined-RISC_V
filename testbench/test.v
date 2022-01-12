`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 10/28/2021 12:46:00 PM
// Design Name: 
// Module Name: test
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


module test;
reg rst, clk;						// 1 button to reset, clock signal as input
wire alu_z;						// An LED turned ON if result is zero
wire[7:0] Anode_Activate;		// Anodes to control 7-segments
wire [6:0] LED_out;

main mainfile(.rst(rst), .clk(clk), .alu_z(alu_z), .Anode_Activate(Anode_Activate), .LED_out(LED_out));


//always begin
//clk=0;
//forever #10 clk=~clk;
//end

initial begin
clk=0; rst=1;
#5 clk=1; rst=1;
#5 clk=0; rst=0;

#5 clk=1; rst=0;
#5 clk=0; rst=0;

#5 clk=1; rst=0;
#5 clk=0; rst=0;

#5 clk=1; rst=0;
#5 clk=0; rst=0;

//#5 clk=1; rst=0;
//#5 clk=0; rst=0;

//#5 clk=1; rst=0;
//#5 clk=0; rst=0;

//#5 clk=1; rst=0;
//#5 clk=0; rst=0;

//#5 clk=1; rst=0;
//#5 clk=0; rst=0;

//#5 clk=1; rst=0;
//#5 clk=0; rst=0;

//#5 clk=1; rst=0;
//#5 clk=0; rst=0;

//#5 clk=1; rst=0;
//#5 clk=0; rst=0;

//#5 clk=1; rst=0;
//#5 clk=0; rst=0;

//#5 clk=1; rst=0;
//#5 clk=0; rst=0;

//#5 clk=1; rst=0;
//#5 clk=0; rst=0;

//#5 clk=1; rst=0;
//#5 clk=0; rst=0;

//#5 clk=1; rst=0;
//#5 clk=0; rst=0;


//#5 clk=1; rst=0;
//#5 clk=0; rst=0;


#5 $stop;
end
endmodule
