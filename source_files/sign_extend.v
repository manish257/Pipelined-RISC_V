module extend(input  		[31:7] instruction,
              input  		[1:0]  immsrc,
              output reg 	[31:0] immext);
    always@(*)
    case(immsrc)
         // I−type
    2'b00:     immext = {{20{instruction[31]}}, instruction[31:20]};
		 // S−type (stores)
    2'b01:     immext = {{20{instruction[31]}}, instruction[31:25], instruction[11:7]};
         // B−type (branches)
    2'b10:      immext = {{20{instruction[31]}}, instruction[7],  instruction[30:25], instruction[11:8], 1'b0};                          // J−type (jal)
		// J−type (branches)
	2'b11:      immext = {{12{instruction[31]}}, instruction[19:12],  instruction[20], instruction[30:21], 1'b0};
           
	default: 	immext = 32'bx; // undefined
    endcase
endmodule