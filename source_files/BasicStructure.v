`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    12:49:01 01/10/2021 
// Design Name: 
// Module Name:    Basic RISC-V single cycle implementation on Verilog 
// Project Name: 
// Target Devices: 
// Tool versions: 
// Description: 
//
// Dependencies: 
//
// Revision: 
// Revision 0.01 - File Created
// Additional Comments: 
//
//////////////////////////////////////////////////////////////////////////////////
module main(rst, clk, alu_z, Anode_Activate, LED_out);
input rst, clk;						// 1 button to reset, clock signal as input
output alu_z;						// An LED turned ON if result is zero
output reg[7:0] Anode_Activate;		// Anodes to control 7-segments
output reg [6:0] LED_out;			// output result to be sent on 7-segments
wire [31:0] pc;

wire [31:0] Result;
	// ALL modules will be called in this file. Code will be executed and results will be shown on 7-segment display
// Code segment for BCD to 7-segment Decoder. Keep this code as it is
reg [31:0] counter;		// A 32 bit flashing counter
reg toggle = 0;			// A variable to toggle between two 7-segments 

always @(posedge clk)
    begin
            if(counter>=100000) begin
                 counter <= 0;
				 toggle = ~toggle; end
            else begin
                counter <= counter + 1;
				end
    end 
    // anode activating signals for 8 segments, digit period of 1ms
    // decoder to generate anode signals 
    always @(*)
    begin
        case(toggle)
        1'b0: begin
            Anode_Activate = 8'b01111111; 
            // activate SEGMENT1 and Deactivate all others
              end
        1'b1: begin
            Anode_Activate = 8'b10111111; 
            // activate LED2 and Deactivate all others    
               end
        endcase
    end
    // Cathode patterns of the 7-segment 1 LED display 
    wire [31:0]hardwire;
    assign Result= hardwire;
    
    always @(*)
    begin
	if (toggle) begin
        case(Result)				// First 4 bits of Result from ALU will be displayed on 1st segment
        32'b0000: LED_out = 7'b0000001; // "0"     
        32'b0001: LED_out = 7'b1001111; // "1" 
        32'b0010: LED_out = 7'b0010010; // "2" 
        32'b0011: LED_out = 7'b0000110; // "3" 
        32'b0100: LED_out = 7'b1001100; // "4" 
        32'b0101: LED_out = 7'b0100100; // "5" 
        32'b0110: LED_out = 7'b0100000; // "6" 
        32'b0111: LED_out = 7'b0001111; // "7" 
        32'b1000: LED_out = 7'b0000000; // "8"     
        32'b1001: LED_out = 7'b0000100; // "9"
		32'b1010: LED_out = 7'b0001000; // "A"     
        32'b1011: LED_out = 7'b1100000; // "b"     
        32'b1100: LED_out = 7'b0110001; // "C"     
        32'b1101: LED_out = 7'b1000010; // "d"     
        32'b1110: LED_out = 7'b0110000; // "E"     
        32'b1111: LED_out = 7'b0111000; // "F"     
        
        default: LED_out = 7'b0000001; // "0"
        endcase
		end
    

	// Cathode patterns of the 7-segment 2 LED display
if(!toggle) begin	
        case(Result[7:4])			// Next 4 bits of Result from ALU will be displayed on 2nd segment
        4'b0000: LED_out = 7'b0000001; // "0"     
        4'b0001: LED_out = 7'b1001111; // "1" 
        4'b0010: LED_out = 7'b0010010; // "2" 
        4'b0011: LED_out = 7'b0000110; // "3" 
        4'b0100: LED_out = 7'b1001100; // "4" 
        4'b0101: LED_out = 7'b0100100; // "5" 
        4'b0110: LED_out = 7'b0100000; // "6" 
        4'b0111: LED_out = 7'b0001111; // "7" 
        4'b1000: LED_out = 7'b0000000; // "8"     
        4'b1001: LED_out = 7'b0000100; // "9"
		4'b1010: LED_out = 7'b0001000; // "A"     
        4'b1011: LED_out = 7'b1100000; // "b"     
        4'b1100: LED_out = 7'b0110001; // "C"     
        4'b1101: LED_out = 7'b1000010; // "d"     
        4'b1110: LED_out = 7'b0110000; // "E"     
        4'b1111: LED_out = 7'b0111000; // "F"     
        
        default: LED_out = 7'b0000001; // "0"
        endcase
    end
end
    
    
	// Keep writing your code (calling lower module) here!
	wire [31:0] PCNext,PCPlus4,PCTarget,immext;
	wire [6:0] opcode;
	wire [2:0] func3;
	wire [6:0] func7;
    wire PCSrc, MemWrite, ALUSrc, RegWrite, Btype,ZeroOut, CarryOut;
    wire [2:0] ResultSrc;
    wire [3:0] ALUControl;
    wire [1:0]  immsrc;
    wire [31:0] Port_A, Port_B; 
    wire [31:0] Din; 
    wire [4:0] Addr_A, Addr_B, Addr_Wr; 
    wire wr, clk, rst; 
    wire [31:0] RESULT;
    wire [31:0] A,B;
    wire [3:0] ALU_Sel;
    wire [31:0] ALU_Out;
    wire [31:0] Data_Out;
    wire [31:0] Data_In;
    wire [31:0] D_Addr;
    wire [31:0] instruction;
    
    wire [31:0] pcD;
    wire [31:0] instructionD;
    wire [31:0] PCPlus4D;
    
    wire [31:0] RD1E, RD2E;
    wire [31:0] pcE;
    wire [31:0] PCPlus4E;
    wire [4:0] RdE;
    wire [31:0] ImmExtE;
    wire PCSrcE, MemWriteE, ALUSrcE, RegWriteE;
    wire [3:0] ALUControlE;
    wire [2:0] ResultSrcE;
    //wire [1:0] ImmSrcE;
    wire [2:0] func3E;
    wire [6:0] opcodeE;
    
    wire [31:0] ALUResultM;
    wire [31:0] WriteDataM;
    wire [4:0] RdM;
    wire [31:0] PCPlus4M;
    wire ResultSrcM;
    wire RegWriteM;
    wire MemWriteM;
    
    wire [31:0] ReadDataW;
    wire [31:0] ALUResultW;
    wire [31:0]PCPlus4W;
    wire [4:0] RdW;
    wire ResultSrcW;
    wire RegWriteW;
    
    
    reg_if_id IF_ID (.clk(clk),.rst(rst),.pc(pc),.instruction(instruction),.PCPlus4(PCPlus4),
                .pcD(pcD),.instructionD(instructionD),.PCPlus4D(PCPlus4D));
     
    reg_id_ex ID_EX(.clk(clk),.rst(rst),.Port_A(Port_A), .Port_B(Port_B),.pcD(pcD),.PCPlus4D(PCPlus4D),.instructionD(instructionD),
               .immext(immext),.PCSrc(PCSrc), .MemWrite(MemWrite), .ALUSrc(ALUSrc), 
               .RegWrite(RegWrite),.ALUControl(ALUControl),.ResultSrc(ResultSrc),
               .RD1E(RD1E), .RD2E(RD2E),.pcE(pcE),.PCPlus4E(PCPlus4E),.RdE(RdE),.ImmExtE(ImmExtE),.PCSrcE(PCSrcE), .MemWriteE(MemWriteE),
               .ALUSrcE(ALUSrcE), .RegWriteE(RegWriteE),
               .ALUControlE(ALUControlE),.ResultSrcE(ResultSrcE),.func3(instructionD[14:12]),.opcode(instructionD[6:0]),
               .func3E(func3E),.opcodeE(opcodeE));

    
    reg_ex_mem EX_MEM(.clk(clk),.rst(rst),.ALU_Out(ALU_Out),.RD2E(RD2E),.RdE(RdE),.PCPlus4E(PCPlus4E),.ResultSrcE(ResultSrcE),.RegWriteE(RegWriteE),
               .MemWriteE(MemWriteE),
               .ALUResultM(ALUResultM), .WriteDataM(WriteDataM), .RdM(RdM), .PCPlus4M(PCPlus4M), .ResultSrcM(ResultSrcM), 
               .RegWriteM(RegWriteM),
               .MemWriteM(MemWriteM));
    
    reg_mem_wb MEM_WB(.clk(clk),.rst(rst),.Data_Out(Data_Out),.ALUResultM(ALUResultM),.RdM(RdM),.PCPlus4M(PCPlus4M),
                 .ResultSrcM(ResultSrcM),.RegWriteM(RegWriteM),.ReadDataW(ReadDataW),.ALUResultW(ALUResultW),.PCPlus4W(PCPlus4W),
                 .RdW(RdW),.ResultSrcW(ResultSrcW),.RegWriteW(RegWriteW));
    
    Control_Unit controller(.opcode(instructionD[6:0]) , .func3(instructionD[14:12]) , .func7(instructionD[31:25]),
                                         .ResultSrc(ResultSrc), .MemWrite(MemWrite), .ALUSrc(ALUSrc), .RegWrite(RegWrite),
                                         .ALUControl(ALUControl),
                                         .ImmSrc(immsrc));
    
	CSR control_status_register(.pcE(PCPlus4D),.epc(epc),.interrupt(hardwire),.mret(mret),.interrupt_en(interrupt_en),.clk(clk),
				.stopped_interrupt(stopped_interrupt));

    module_for_PCSrc PCSrc_module(.ZeroOut(ZeroOut),.CarryOut(CarryOut),.opcodeE(opcodeE),.func3E(func3E),.PCSrc(PCSrc));                                           
                                             
	adder pcplus4ins(.a(pc),.b(32'h4),.y(PCPlus4));
	
	adder pctargetins(.a(pcE),.b(ImmExtE),.y(PCTarget));
	
	//mux2x1 pcmux(.sel(PCSrc),.b(PCPlus4),.a(PCTarget),.o(PCNext));
	
	instruction_memory instructionmemory(.pc(pc),.instruction(instruction));
	
	hazard_module hazard(.rs1D(rs1),.rs2D(rs2),.rs1E(rs1E),.rs2E(rs2E),.rdE(RdE),.interrupt_en(interrupt_en),
						.rdM(RdE),.rdW(RdM),.ResultSrcE0(ResultSrc[0]),.RegWriteM(RegWriteE),
						.RegWriteW(RegWrite_out),.StallF(StallF),.StallD(StallD),.FlushE(FlushE),.FlushD(FlushD),
						.ForwardAE(ForwardAE),.ForwardBE(ForwardBE),.PCSrcE(PCSrc),.FlushM(FlushM),.stopped_interrupt(stopped_interrupt));
	
	
	mux2x1 middle(.sel(ALUSrcE),.b(RD2E),.a(ImmExtE),.o(B));
	                  
    extend ext(.instruction(instructionD[31:7]), .immsrc(immsrc), .immext(immext));
     
    register_file registerfile(.Port_A(Port_A), .Port_B(Port_B),.hardwire(hardwire), .RESULT(RESULT), 
                                .Addr_A(instructionD[19:15]), .Addr_B(instructionD[24:20]), .Addr_Wr(RdW), 
                                .wr(RegWriteW), .clk(clk),.rst(rst));
     
    alu alu( .A(RD1E),.B(B),.ALU_Sel(ALUControlE),.ALU_Out(ALU_Out),.CarryOut(CarryOut), .ZeroOut(ZeroOut));
     
    //address_gen AddressGenerator(.pc(pc),.PCNext(PCNext),.clk(clk),.rst(rst));
    address_gen AddressGenerator(.pc(pc),.PCNext(PCPlus4),.clk(clk),.rst(rst));
    
    Mux3x1 muxatend( .in1(ALUResultW),.in2(ReadDataW),.in3(PCPlus4W), .sel(ResultSrcW),.out(RESULT));
    
    Data_Memory datamemory(.Data_Out(Data_Out),.Data_In(WriteDataM),.D_Addr(ALUResultM), .wr(MemWriteM), .rst(rst),.clk(clk));
     
endmodule







module Control_Unit(input [6:0] opcode , input [2:0] func3 , input [6:0] func7,
					output reg  MemWrite, ALUSrc, RegWrite, 
					output reg [3:0] ALUControl,
					output reg  [2:0] ResultSrc,
					output [1:0] ImmSrc,
					output reg [2:0] instruction_type);			
	// This is the part of Phase-II
	
	always @ (*)
	begin
	case (opcode)
	7'b0010011: instruction_type = 3'd0; //i-type
	7'b1100111: instruction_type = 3'd0; //i-type jalr
	7'b0000011: instruction_type = 3'd0; //lw i-type
	7'b0100011: instruction_type = 3'd1; //sw s-type
	7'b1100011: instruction_type = 3'd2; //b-type
	7'b1101111: instruction_type = 3'd3; //j-type jal
	7'b0110011: instruction_type = 3'd4; //r-type
	7'b0110111: instruction_type = 3'd5; //u-type lui
	7'b0010111: instruction_type = 3'd5; //u-type auipc
	default: instruction_type = 3'd0; 
	endcase
	end
	assign ImmSrc = instruction_type[1:0];
	always @(*)
	begin
	if (instruction_type == 0 || instruction_type == 3'd4 || instruction_type == 3'd5 || instruction_type == 3'd3)
        RegWrite=1;
     else
        RegWrite =0;
            
    if (instruction_type==1)
        MemWrite=1;
    else
        MemWrite=0;
        
    if(opcode == 7'b0000011)
        ResultSrc = 2'b01;
    if (instruction_type==3'd3 || opcode == 7'b1100111)
        ResultSrc =2'b10;    
    else
        ResultSrc =2'b00;
        
    if ( instruction_type== 3'd2 || instruction_type == 3'd4)
        ALUSrc = 0;
    else
        ALUSrc =1;
    end
	
	always @(*)
    begin
        casex({opcode,func3,func7})
        17'b01100110000000000: ALUControl = 4'b0000; //add 
        17'b01100110000100000: ALUControl = 4'b0001; //sub
        17'b01100110000000001: ALUControl = 4'b0010; //mul
        17'b01100111000000001: ALUControl = 4'b0011; //div
        17'b0010011000xxxxxxx: ALUControl = 4'b0000;   //addI 
        17'b1100011000xxxxxxx: ALUControl = 4'b0001;       //beq
        17'b1100011001xxxxxxx: ALUControl = 4'b0001;       //bneq
        17'b1100011100xxxxxxx: ALUControl = 4'b0001; //blt
            
        endcase
    end
	
endmodule



module mux2x1(output [31:0]o,		// 32 bit output
					input[31:0]a,b,		// 32 bit inputs
					input sel			// Selection Signal
			);
			
	assign o = sel ? a:b;
endmodule


