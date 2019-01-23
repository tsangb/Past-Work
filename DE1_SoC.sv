// Top-level module that defines the I/Os for the DE-1 SoC board

module DE1_SoC (HEX0, HEX1, HEX2, HEX3, HEX4, HEX5, KEY, LEDR, SW, CLOCK_50);
 output logic [6:0] HEX0, HEX1, HEX2, HEX3, HEX4, HEX5;
 output logic [9:0] LEDR;
 input logic [3:0] KEY;
 input logic [9:0] SW;
 input logic CLOCK_50;
 // Default values, turns off the HEX displays
 assign HEX0 = 7'b1111111;
 assign HEX1 = 7'b1111111;
 assign HEX2 = 7'b1111111;
 assign HEX3 = 7'b1111111;
 assign HEX4 = 7'b1111111;
 assign HEX5 = 7'b1111111;

	logic			rst;
	
	//assign rst = SW[9];
	
	reg clk;
	
	assign clk = CLOCK_50;
	
	
	
	
	//output	[31:0]	o;			// test variable

	
	genvar			index;		// for creating muxes
	
	wire			regWE, sramWE;						// memory controls
	wire			z, v, c, n;							// alu flags
	wire			regDst, branch, memToReg, aluSrc;	// controls value into reg
	wire			bSel;								// branch is taken or not
	wire			jump, jumpReg;						// jump/jump reg is taken or not
	wire			flushIf, flushId;					// flush values from ifid and idex
	wire			stall;								// reset pc address + 1 / stall 1 cycle
	wire			ctrlCtrl;							// control for control mux
	
	// pc wires
	wire	[31:0]	jAddress;	// jump address
	wire	[31:0]	pcOut;		// output of pc
	wire	[31:0]	pcP1;		// pc + 1
	wire	[31:0]	pcP1IfId;	// pcP1 out of ifid register
	wire	[31:0]	pcBranch;	// output of branch adder (pcAdder)
	wire	[31:0]	branchYN;	// output of branch mux
	wire	[31:0]	jumpYN;		// output of jump mux
	wire	[31:0]	jrYN;		// output of jump register mux
	wire	[31:0]	rstYN;		// output of reset mux
	wire	[31:0]	stallYN;	// output of stall mux
	wire	[31:0]	pcIn;		// output of next mux
	wire			bPredict;	// pre-sets pc based on prediction
	
	// pre-IfId
	wire	[31:0]	iMemOut;	// output of instruction memory
	
	// ifId wires
	// wbReg = regWE, memToReg
	// mReg = sramWE, branch, jumpSel, jr
	// exReg = regDst, aluSrc, aluOp[1:0]
	wire	[3:0]	mCtrl;		// memory control bits from control
	wire	[1:0]	wbCtrl;		// write back control bits from control
	wire	[3:0]	exCtrl;		// execute bits from control
	
	wire	[31:0]	ifIdBus;	// output of ifId instructions
	wire	[31:0]	immediate;	// output of extender, 32 immediate bits
	wire	[31:0]	read0Out;	// output of read reg 0
	wire	[31:0]	read1Out;	// output of read reg 1
	wire	[31:0]	reg1Out;	// output of register 1 or wb
	wire	[31:0]	reg2Out;	// output of register 2 or wb
	wire	[1:0]	aluOp;		// output of control
	wire			match1;		// if write address matches with read register 0
	wire			match2;		// if write address matches with read register 1
	wire			controlMuxIn;// controls values into idex control
	
	// idEx wires
	wire	[3:0]	mIdEx;		// memory control bits from idEx
	wire	[1:0]	wbIdEx;		// write back control bits from idEx
	wire	[3:0]	exBits;		// execution bits after IdEx
	
	wire	[31:0]	idExR1;		// read data 1 from idEx
	wire	[31:0]	idExR2;		// read data 2 from idEx
	wire	[31:0]	idExImm;	// immediate bits from idEx
	wire	[31:0]	idExPc;		// pc from idEx
	wire	[4:0]	idExRegRW;	// read/write address from idEx (20:16)
	wire	[4:0]	idExRegW;	// write address from idEx (15:11)
	wire	[4:0]	idExRegR;	// read address from idEx (25:21)
	
	wire	[31:0]	aluA;		// output of 3 to 1 mux for ALU input 1
	wire	[31:0]	aluB;		// output of 2 to 1 mux between immediate and reg value
	wire	[31:0]	valB2;		// output of 3 to 1 mux for ALU input 2
	
	wire	[1:0]	aluMuxCtrl1;// controls valA
	wire	[1:0]	aluMuxCtrl2;// controls valB
	
	wire	[4:0]	regWAddress;// register address chosen between 20:16 and 15:11
	wire	[2:0]	aluOpOut;	// alu operation from alu control
	wire	[31:0]	aluOut;		// output of ALU
	wire			bpOut;		// bPredict out of idEx
	wire			pcCtrl;		// controls pc mux if we have to reset due to bad branch
	wire	[31:0]	stallAddr;	// pc stall address
	
	// exMem wires
	wire	[31:0]	exMemAlu;	// exMem alu output
	wire	[4:0]	exMemW;		// exMem write address for reg
	wire	[1:0]	wbExMem;	// exMem write back control bits
	wire	[3:0]	mExMem;		// exMem memory bits
	wire	[15:0]	sramData;	// holder for inout for sram
	wire	[31:0]	sramOut;	// output of SRAM
	wire	[3:0]	flags;		// ALU flags
	wire	[10:0]	exMemB;		// sram address from exMem
	
	// memWb wires
	wire	[31:0]	memWbAlu;	// memWb alu output
	wire	[31:0]	memWbSram;	// memWb sram output
	wire	[4:0]	memWbW;		// memWb write address for reg
	wire	[1:0]	wbMemWb;	// memWb write back control bits
	wire	[31:0]	dataWb;		// data to be written back
	
	wire jOut;
	
	// test variable
	//assign o = {pcOut[7:0], 1'bz, aluOut[23:0]};
	//iMemOut[31:26], 1'bz, exMemB[3:0], 1'bz, exMemAlu[3:0], 1'bz, mExMem[3], 1'bz, sramData[3:0], 1'bz, sramOut[3:0]
	
	// immediate jump address
	assign jAddress = {6'b0, ifIdBus[25:0]};
	
	// flush registers
	assign flushId = pcCtrl;
	assign flushIf = pcCtrl | jump | bPredict;
	
	and				branchControl (bSel, flags[3], mIdEx[2]);
	or				jumpGate (jOut, jump, jumpReg);
	
	
	
	// program counter
	pc				myPc (.pcOut(pcOut), .pcIn(pcIn), .clk(clk), .rst(rst));
	// program counter incrementer
	add1			myadd1 (.addOut(pcP1), .pc(pcOut));
	// branch address calculator
	pcAdder			myPcAdder (.addOut(pcBranch), .pc(pcP1IfId), .shiftImmediate(immediate));
	// 16 to 32 bit extender
	extender		myExtender (.exOut(immediate), .in(ifIdBus[15:0]));
	// registers for instructions
	instructionMem	myInstructionMem (.code(iMemOut), .address(pcOut[6:0]));
	
	// ifId to idEx
	ifId			myIfId (.iOut(ifIdBus), .pcOut(pcP1IfId), .iIn(iMemOut), .pcIn(pcP1), .flush(flushIf), .clk(clk));
	
	// controls control signals
	control			myControl (.regDst(regDst), .branch(branch), .sramWE(sramWE), .memToReg(memToReg), .jump(jump), .jumpReg(jumpReg), 
							   .aluOp(aluOp), .aluSrc(aluSrc), .regWE(regWE), .opCode(ifIdBus[31:26]), .funct(ifIdBus[5:0]));
	// register memory
	regMem			myRegMem (.read0Out(read0Out), .read1Out(read1Out), .read0(ifIdBus[25:21]), .read1(ifIdBus[20:16]), 
							  .writeAddress(memWbW), .dataIn(dataWb), .clk(clk), .WE(wbMemWb[1]), .rst(rst));
	// deals with branch and stall hazards
	hazardUnit		myHazardUnit (.bPredict(bPredict), .stall(stall), .addrOut(stallAddr), .rw(regWAddress), 
								  .idExM(mIdEx[3]), .instructionIn(ifIdBus), .bTaken(bSel), .aluOpIdEx(exBits[1:0]), .aluOpCurr(aluOp), .jump(jOut), .addrIn(idExPc), .clk(clk));
	
	or			removeControl (controlMuxIn, stall, pcCtrl);	// sets NOP in case of bad branch or stall
	
	and			pcCMux (pcCtrl, !bSel, bpOut, mIdEx[2]);		// if we predict a branch, but do not actually branch, we reset
	
	// if we experience a stall, we need to set the operation going into control to NOP
	mux2To1		hazardMuxWb (.muxOut(wbCtrl[1]), .muxIn1(1'b1), .muxIn0(regWE), .ctrl(controlMuxIn));	// reg will not save value
	mux2To1		hazardMuxM3 (.muxOut(mCtrl[3]), .muxIn1(1'b1), .muxIn0(sramWE), .ctrl(controlMuxIn));	// sram will not save
	mux2To1		hazardMuxM2 (.muxOut(mCtrl[2]), .muxIn1(1'b0), .muxIn0(branch), .ctrl(controlMuxIn));	// do not branch
	mux2To1		hazardMuxM1 (.muxOut(mCtrl[1]), .muxIn1(1'b0), .muxIn0(jump), .ctrl(controlMuxIn));		// do not jump
	mux2To1		hazardMuxM0 (.muxOut(mCtrl[0]), .muxIn1(1'b0), .muxIn0(jumpReg), .ctrl(controlMuxIn));	// do not jump
	
	// assign execution control bits
	assign exCtrl = {regDst, aluSrc, aluOp};
	// assign write-back control bit
	assign wbCtrl[0] = memToReg;
	// assign match bits
	assign match1 = ((memWbW == ifIdBus[25:21]) && (ifIdBus[31:26] == 6'b000000));
	assign match2 = ((memWbW == ifIdBus[20:16]) && (ifIdBus[31:26] == 6'b000000));
	
	// idEx to exMem
	idEx			myIdEx (.r1Out(idExR1), .r2Out(idExR2), .extendOut(idExImm), .pcOut(idExPc), .rwRegOut(idExRegRW), 
							.rRegOut(idExRegR), .wRegOut(idExRegW), .wbOut(wbIdEx), .mOut(mIdEx), .exOut(exBits), .bpOut(bpOut),
							.r1In(reg1Out), .r2In(reg2Out), .extendIn(immediate), .pcIn(pcP1IfId), .rwRegIn(ifIdBus[20:16]),
							.rRegIn(ifIdBus[25:21]), .wRegIn(ifIdBus[15:11]), .wbIn(wbCtrl), .mIn(mCtrl), .exIn(exCtrl), .flush(flushId), .bpIn(bPredict), .clk(clk));
	
	ALU				myALU (.dataOut(aluOut), .z(z), .v(v), .c(c), .n(n), .valA(aluA), .valB(aluB), .control(aluOpOut));
	aluControl		myALUControl (.aluOpOut(aluOpOut), .aluOp(exBits[1:0]), .funct(idExImm[5:0]));
	fwdUnit			myFwdUnit (.aluMuxCtrl1(aluMuxCtrl1), .aluMuxCtrl2(aluMuxCtrl2), .rReg(idExRegR), .rw(idExRegRW), .wAdr1(exMemW), .wAdr2(memWbW), .wb1(wbExMem), .wb2(wbMemWb));
	
	generate
	for (index = 0; index < 32; index = index + 1)
	begin: muxCreation1
		// chooses value into alu input A
		mux3To1		valA (.out(aluA[index]), .in2(exMemAlu[index]), .in1(dataWb[index]), .in0(idExR1[index]), .ctrl(aluMuxCtrl1));
		// chooses value into alu input B
		mux3To1		valB (.out(valB2[index]), .in2(exMemAlu[index]), .in1(dataWb[index]), .in0(idExR2[index]), .ctrl(aluMuxCtrl2));
		
		// chooses value into alu input B between reg/calculated value and immediate
		mux2To1		immediateMux(.muxOut(aluB[index]), .muxIn1(idExImm[index]), .muxIn0(valB2[index]), .ctrl(exBits[2]));

		// if we predict branch and it is a branch we branch
		mux2To1		branchMux(.muxOut(branchYN[index]), .muxIn1(pcBranch[index]), .muxIn0(pcP1[index]), .ctrl(bPredict));
		// if we jump, we jump
		mux2To1		jumpMux(.muxOut(jumpYN[index]), .muxIn1(jAddress[index]), .muxIn0(branchYN[index]), .ctrl(jump));
		// if we jump reg, we jump reg
		mux2To1		jrMux(.muxOut(jrYN[index]), .muxIn1(reg1Out[index]), .muxIn0(jumpYN[index]), .ctrl(jumpReg));
		// if we predicted wrong, we go back to addr + 1 after branch operation
		mux2To1		resetMux(.muxOut(rstYN[index]), .muxIn1(idExPc[index]), .muxIn0(jrYN[index]), .ctrl(pcCtrl));
		// we have to stall one cycle, so we set it equal to itself
		mux2To1		stallMux(.muxOut(pcIn[index]), .muxIn1(stallAddr[index]), .muxIn0(rstYN[index]), .ctrl(stall));
		
		// we need to check if wb is the same as the input read addresses to forward the value
		mux2To1		aCheck(.muxOut(reg1Out[index]), .muxIn1(dataWb[index]), .muxIn0(read0Out[index]), .ctrl(match1));
		mux2To1		bCheck(.muxOut(reg2Out[index]), .muxIn1(dataWb[index]), .muxIn0(read1Out[index]), .ctrl(match2));
	end
	endgenerate
	
	generate
	for (index = 0; index < 5; index = index + 1)
	begin: muxCreation2
		mux2To1		rAddr (.muxOut(regWAddress[index]), .muxIn1(idExRegW[index]), .muxIn0(idExRegRW[index]), .ctrl(exBits[3]));
	end
	endgenerate
	
	// exMem to memWb
	exMem			myExMem (.aluOut(exMemAlu), .regWOut(exMemW), .wbOut(wbExMem), .mOut(mExMem), .flagOut(flags), .exMemB(exMemB),
							 .aluIn(aluOut), .regWIn(regWAddress), .wbIn(wbIdEx), .mIn(mIdEx), .flagIn({z, v, c, n}), .exMemBIn(aluB[10:0]), .clk(clk));
	
	// assign sram inout based on if read/write or other
	
	assign sramOut = {16'b0, sramData};
	
	SRAM			mySRAM (.dataOut(sramData), .dataIn(exMemAlu[15:0]), .address(exMemB[10:0]), .WE(mExMem[3]), .clk(clk));
	
	// memWb
	memWb			myMemWb (.sramOut(memWbSram), .aluOut(memWbAlu), .regWOut(memWbW), .wbOut(wbMemWb),
							 .sramInEx(sramOut), .aluIn(exMemAlu), .regWIn(exMemW), .wbIn(wbExMem), .clk(clk));
	
	generate
	for (index = 0; index < 32; index = index + 1)
	begin: muxCreation3
		mux2To1		sramALUMux(.muxOut(dataWb[index]), .muxIn1(memWbSram[index]), .muxIn0(memWbAlu[index]), .ctrl(wbMemWb[0]));
	end
	endgenerate
	
endmodule

/**************************************************************
*	This module defines alu control
**************************************************************/
//  This module controls the alu opcode

module aluControl(aluOpOut, aluOp, funct);

	output reg	[2:0]	aluOpOut;
	
	input	[5:0]	funct; // instruction [5:0] indicating function field 
	input	[1:0]	aluOp; // ALUop outputs from control system 
	
	//parameter[1:0] 
	
	
	//function field based on MIPS reference 
	parameter [5:0] ADD = 6'b100000 , SUB = 6'b100010, AND = 6'b100100,
					OR = 6'b100101, XOR = 6'b100110,  SLT = 6'b101010, SLL = 6'b000000;

	always@ (*)
		case(aluOp)
			2'b00:		// lw, sw
				begin
					aluOpOut = 3'b001;
				end
			2'b01:		// bgt
				begin
					aluOpOut = 3'b000; // NOP 
				end
			2'b10:		// R-type
				begin
					if(funct == ADD)
						aluOpOut = 3'b001;
					else if(funct == SUB)
						aluOpOut = 3'b010;
					else if(funct == AND)
						aluOpOut = 3'b011;
					else if(funct == OR)
						aluOpOut = 3'b100;
					else if(funct == XOR)
						aluOpOut = 3'b101; 
					else if(funct == SLT)
						aluOpOut = 3'b110; 
					else if(funct == SLL)
						aluOpOut = 3'b111; 
					else
						aluOpOut = 3'b000; // otherwise no operation 
					//aluOpcode = instructions[2:0];
				end
			2'b11:		// addi
				begin
					aluOpOut = 3'b001;
				end
	endcase

endmodule

/**************************************************************
*	This module defines the instruction decoder
**************************************************************/
//  The module converts our code into control signals
//  These control signals include all CS, OE, WE, immediate/not immediate, etc.

module control(regDst, branch, sramWE,memToReg, jump, jumpReg, aluOp, aluSrc, regWE, opCode, funct);
	
	input	[5:0]	opCode, funct;
	
	output					regDst;
	output					branch;
	output					sramWE;
	output					regWE;
	output	reg [1:0]		aluOp;	// 00 : lw, sw, 01 : bgt, 10; r				// high true
	output					aluSrc;
	output					memToReg;
	output	reg				jump;
	output  reg				jumpReg;
	
	reg			regDst;
	reg			branch;
	reg			sramWE;
	reg			regWE;
	reg			aluSrc;
	reg			memToReg;
	
	// 00 no op
	// 10 r format
	// 01 bgt
	
	initial
	begin
		sramWE = 1'b1;
		regDst = 1'bx;
		branch = 1'b0;
		regWE = 1'b1;
		aluOp = 2'b00; 
		aluSrc = 1'b1;
		memToReg = 1'b1;
		jump = 1'b0;
		jumpReg = 1'b0;
	end

	//opCode
	parameter [5:0] rType = 6'b000000, lw = 6'b100011, sw = 6'b101011, bgt = 6'b000100,
					j = 6'b000010, addi = 6'b001000;
					/*immLW = 6'b100001, immAdd = 6'b111001,
					immSub = 6'b111010, immAnd = 6'b111011, immOr = 6'b111100, 
					immXor = 6'b111101, immSLT = 6'b111110, immSLL = 6'b111111,
					jr = 6'b010000;*/
					
	//funct
	parameter [5:0] jr = 6'b001000;  // other function field will be covered in aluControl.v 
									
	always@ (*)
		case(opCode)
			default:
				begin
					sramWE <= 1'b1;
					regDst <= 1'b1;
					branch <= 1'b0;
					regWE <= 1'b1;
					aluOp <= 2'b00; 
					aluSrc <= 1'b1;
					memToReg <= 1'b1;
					jump <= 1'b0;
					jumpReg <= 1'b0;
				end
			rType:
				begin
					if (funct == jr) // jump register occurs from R type instructions
						begin
							sramWE <= 1'b1;
							regDst <= 1'bx;
							branch <= 1'bx;
							regWE <= 1'b1;
							aluOp <= 2'bx; 
							aluSrc <= 1'bx;
							memToReg <= 1'bx;
							jump <= 1'b0;
							jumpReg <= 1'b1;
						end
					else
						begin 
							sramWE <= 1'b1;
							regDst <= 1'b1;
							branch <= 1'b0;
							regWE <= 1'b0;
							aluOp <= 2'b10; 
							aluSrc <= 1'b0;
							memToReg <= 1'b0;
							jump <= 1'b0;
							jumpReg <= 1'b0;
						end
				end
			lw:
				begin
					sramWE <= 1'b1;
					regDst <= 1'b0;
					branch <= 1'b0;
					regWE <= 1'b0;
					aluOp <= 2'b00; 
					aluSrc <= 1'b1;
					memToReg <= 1'b1;
					jump <= 1'b0;
					jumpReg <= 1'b0;
				end
			sw:
				begin
					sramWE <= 1'b0;
					regDst <= 1'b1;
					branch <= 1'b0;
					regWE <= 1'b1;
					aluOp <= 2'b00; 
					aluSrc <= 1'b1;
					memToReg <= 1'bx;
					jump <= 1'b0;
					jumpReg <= 1'b0;
				end		
			bgt:
				begin
					sramWE <= 1'b1;
					regDst <= 1'bx;
					branch <= 1'b1;
					regWE <= 1'b1;
					aluOp <= 2'b01; 
					aluSrc <= 1'bx;
					memToReg <= 1'bx;
					jump <= 1'b0;
					jumpReg <= 1'b0;
				end	
			j:
				begin
					sramWE <= 1'b1;
					regDst <= 1'bx;
					branch <= 1'b0;
					regWE <= 1'bx;
					aluOp <= 2'b00; 
					aluSrc <= 1'bx;
					memToReg <= 1'bx;
					jump <= 1'b1;
					jumpReg <= 1'b0;
				end			
			addi:
				begin
					sramWE <= 1'b1;
					regDst <= 1'b0;
					branch <= 1'b0;
					regWE <= 1'b0;
					aluOp <= 2'b11; 
					aluSrc <= 1'b1;
					memToReg <= 1'b0;
					jump <= 1'b0;
					jumpReg <= 1'b0;
				end
		endcase
endmodule

/**************************************************************
*	This module defines a D flip-flop
**************************************************************/
//  The module is a D flip-flop
//  Code provided by lab 1.

module DFlipFlop(q, clk, D, rst);
	input D, clk, rst;
	output q;
	reg q;

	always@ (negedge rst or posedge clk)
	begin
		if(!rst)
			q = 0;
		else
			q = D;
		end
endmodule 

/**************************************************************
*	This module defines the regMem module
**************************************************************/
//  The module is a block of memory with an regMem design
//  There are 32 bits in a register

module register(dataOut, dataIn, writeReg, clk, rst);
					
	input  	 [31:0]		dataIn;
	input	 			writeReg;
	input				clk;
	input				rst;
	
	output	 [31:0]		dataOut;
	
	wire	 [31:0]		andStatement0;
	wire	 [31:0]		andStatement1;
	wire	 			notWriteReg;
	wire	 [31:0]		data;

	not	 				not0(notWriteReg, writeReg);
	
	// If we are writing, then input dataIn into the flip-flops
	// Otherwise, input the previous value of the flip-flops back into the flip-flops
	// AB + (~A)C, where A is writing to this register, B is data in, and C is data out.

	and	 		data0[31:0](andStatement0[31:0], notWriteReg, dataOut[31:0]);		// We are not writing
	and	 		data1[31:0](andStatement1[31:0], writeReg, dataIn[31:0]);			// We are writing
	or	 		data2[31:0](data[31:0], andStatement0[31:0], andStatement1[31:0]);
	DFlipFlop	bits[31:0](.q(dataOut[31:0]), .clk(clk), .D(data[31:0]), .rst(rst));


endmodule

/**************************************************************
*	This module defines a decoder
**************************************************************/
//  The module is a 2-to-4 decoder

module decoder2To4(decoderOut, bitsIn);
					
	input  	 [1:0]		bitsIn;
	
	output	 [3:0]		decoderOut;
	
	wire	 [1:0]		bitsInNot;

	not		 			not0[1:0](bitsInNot[1:0], bitsIn[1:0]);
	
	// Truth table
	and 				twoToFour0(decoderOut[0], bitsInNot[1], bitsInNot[0]);	// 00
	and					twoToFour1(decoderOut[1], bitsInNot[1], bitsIn[0]);		// 01
	and 				twoToFour2(decoderOut[2], bitsIn[1], bitsInNot[0]);		// 10
	and 				twoToFour3(decoderOut[3], bitsIn[1], bitsIn[0]);		// 11
	
endmodule

/**************************************************************
*	This module defines a decoder
**************************************************************/
//  The module is a 3-to-8 decoder

module decoder3To8(decoderOut, bitsIn, enable);
					
	input  	 [2:0]		bitsIn;
	input				enable;
	
	output	 [7:0]		decoderOut;
	
	wire	 [2:0]		bitsInNot;

	not		 			not0[2:0](bitsInNot[2:0], bitsIn[2:0]);

	// Truth table
	and 				threeToEight0(decoderOut[0], bitsInNot[2], bitsInNot[1], bitsInNot[0], enable);	// 000
	and 				threeToEight1(decoderOut[1], bitsInNot[2], bitsInNot[1], bitsIn[0], enable);	// 001
	and 				threeToEight2(decoderOut[2], bitsInNot[2], bitsIn[1], bitsInNot[0], enable);	// 010
	and 				threeToEight3(decoderOut[3], bitsInNot[2], bitsIn[1], bitsIn[0], enable);		// 011
	and 				threeToEight4(decoderOut[4], bitsIn[2], bitsInNot[1], bitsInNot[0], enable);	// 100
	and 				threeToEight5(decoderOut[5], bitsIn[2], bitsInNot[1], bitsIn[0], enable);		// 101
	and 				threeToEight6(decoderOut[6], bitsIn[2], bitsIn[1], bitsInNot[0], enable);		// 110
	and 				threeToEight7(decoderOut[7], bitsIn[2], bitsIn[1], bitsIn[0], enable);			// 111
	
endmodule	

/**************************************************************
*	This module defines a mux
**************************************************************/
//  The module is a 32-to-1 mux

module mux32To1(bitOut, bitsIn, readSel);
					
	input  	 [31:0]		bitsIn;
	input	 [4:0]		readSel;
	
	output	 			bitOut;
	
	wire	 [4:0]		readSelNot;
	wire	 [31:0]		regBit;
	wire	 [31:0]		regAddress;
	

	not		 			not0[4:0](readSelNot[4:0], readSel[4:0]);


	and					and0(regAddress[0], readSelNot[4], readSelNot[3], readSelNot[2], readSelNot[1], readSelNot[0]);	// 00000
	and					and1(regAddress[1], readSelNot[4], readSelNot[3], readSelNot[2], readSelNot[1], readSel[0]);	// 00001
	and					and2(regAddress[2], readSelNot[4], readSelNot[3], readSelNot[2], readSel[1], readSelNot[0]);	// 00010
	and					and3(regAddress[3], readSelNot[4], readSelNot[3], readSelNot[2], readSel[1], readSel[0]);		// 00011
	and					and4(regAddress[4], readSelNot[4], readSelNot[3], readSel[2], readSelNot[1], readSelNot[0]);	// 00100
	and					and5(regAddress[5], readSelNot[4], readSelNot[3], readSel[2], readSelNot[1], readSel[0]);		// 00101
	and					and6(regAddress[6], readSelNot[4], readSelNot[3], readSel[2], readSel[1], readSelNot[0]);		// 00110
	and					and7(regAddress[7], readSelNot[4], readSelNot[3], readSel[2], readSel[1], readSel[0]);			// 00111
	and					and8(regAddress[8], readSelNot[4], readSel[3], readSelNot[2], readSelNot[1], readSelNot[0]);	// 01000
	and					and9(regAddress[9], readSelNot[4], readSel[3], readSelNot[2], readSelNot[1], readSel[0]);		// 01001
	and					and10(regAddress[10], readSelNot[4], readSel[3], readSelNot[2], readSel[1], readSelNot[0]);		// 01010
	and					and11(regAddress[11], readSelNot[4], readSel[3], readSelNot[2], readSel[1], readSel[0]);		// 01011
	and					and12(regAddress[12], readSelNot[4], readSel[3], readSel[2], readSelNot[1], readSelNot[0]);		// 01100
	and					and13(regAddress[13], readSelNot[4], readSel[3], readSel[2], readSelNot[1], readSel[0]);		// 01101
	and					and14(regAddress[14], readSelNot[4], readSel[3], readSel[2], readSel[1], readSelNot[0]);		// 01110
	and					and15(regAddress[15], readSelNot[4], readSel[3], readSel[2], readSel[1], readSel[0]);			// 01111
	and					and16(regAddress[16], readSel[4], readSelNot[3], readSelNot[2], readSelNot[1], readSelNot[0]);	// 10000
	and					and17(regAddress[17], readSel[4], readSelNot[3], readSelNot[2], readSelNot[1], readSel[0]);		// 10001
	and					and18(regAddress[18], readSel[4], readSelNot[3], readSelNot[2], readSel[1], readSelNot[0]);		// 10010
	and					and19(regAddress[19], readSel[4], readSelNot[3], readSelNot[2], readSel[1], readSel[0]);		// 10011
	and					and20(regAddress[20], readSel[4], readSelNot[3], readSel[2], readSelNot[1], readSelNot[0]);		// 10100
	and					and21(regAddress[21], readSel[4], readSelNot[3], readSel[2], readSelNot[1], readSel[0]);		// 10101
	and					and22(regAddress[22], readSel[4], readSelNot[3], readSel[2], readSel[1], readSelNot[0]);		// 10110
	and					and23(regAddress[23], readSel[4], readSelNot[3], readSel[2], readSel[1], readSel[0]);			// 10111
	and					and24(regAddress[24], readSel[4], readSel[3], readSelNot[2], readSelNot[1], readSelNot[0]);		// 11000
	and					and25(regAddress[25], readSel[4], readSel[3], readSelNot[2], readSelNot[1], readSel[0]);		// 11001
	and					and26(regAddress[26], readSel[4], readSel[3], readSelNot[2], readSel[1], readSelNot[0]);		// 11010
	and					and27(regAddress[27], readSel[4], readSel[3], readSelNot[2], readSel[1], readSel[0]);			// 11011
	and					and28(regAddress[28], readSel[4], readSel[3], readSel[2], readSelNot[1], readSelNot[0]);		// 11100
	and					and29(regAddress[29], readSel[4], readSel[3], readSel[2], readSelNot[1], readSel[0]);			// 11101
	and					and30(regAddress[30], readSel[4], readSel[3], readSel[2], readSel[1], readSelNot[0]);			// 11110
	and					and31(regAddress[31], readSel[4], readSel[3], readSel[2], readSel[1], readSel[0]);				// 11111

	and		 			and32[31:0](regBit[31:0], regAddress[31:0], bitsIn[31:0]);

	or					or0(bitOut, regBit[31], regBit[30], regBit[29], regBit[28], regBit[27],
							regBit[26], regBit[25], regBit[24], regBit[23], regBit[22], regBit[21], 
							regBit[20], regBit[19], regBit[18], regBit[17], regBit[16], regBit[15],
							regBit[14], regBit[13], regBit[12], regBit[11], regBit[10], regBit[9],
							regBit[8], regBit[7], regBit[6], regBit[5], regBit[4], regBit[3], regBit[2],
							regBit[1], regBit[0]);
	
endmodule

/**************************************************************
*	This module defines the regMem module
**************************************************************/
//  The module is a block of 32 registers
//  This is a 32 x 32 architecture

module regMem(read0Out, read1Out, read0, read1, writeAddress, dataIn, clk, WE, rst);
			
	input    [4:0]   	read0;		
	input    [4:0]   	read1;
	input    [4:0]   	writeAddress;			
	input  	 [31:0]		dataIn;
	input				clk;
	input				WE;			// WE = write enable, active low:  1 -> not write; 0 -> write
	input				rst;
	
	output	 [31:0]		read0Out;
	output 	 [31:0]		read1Out;
	
	wire 	 [31:0]		decoderOut;
	wire	 [3:0]		enable;
	wire	 [31:0]		writeReg;
	wire	 [31:0]		regOut[31:0];
	wire	 [31:0]		getBit[31:0];
	wire				notWE;
	wire	[31:0]		preRead0Out;
	wire	[31:0]		preRead1Out;
	
	wire isZero0, isZero1;
	
	genvar				index;
	genvar				index2;
	
	not 				notWrite(notWE, WE);
	
	// Writing in data
	and					writeGate[31:0](writeReg[31:0], notWE, decoderOut[31:0]);

	
	// Create 32 registers
	generate
		for (index = 0; index < 32; index = index + 1)
		begin: makeReg
			register		myReg(regOut[index][31:0], dataIn[31:0], writeReg[index], clk, rst);
		end
	endgenerate

	// 2-to-4 decoder, enables output from 3-to-8 decoders
	decoder2To4			twoToFour(enable[3:0], writeAddress[4:3]);
	
	// 3-to-8 decoders
	// controls registers 0 to 7
	decoder3To8			zeroToSeven(decoderOut[7:0], writeAddress[2:0], enable[0]);
	
	// registers 8 to 15
	decoder3To8			eightToFifteen(decoderOut[15:8], writeAddress[2:0], enable[1]);
	
	// registers 16 to 23
	decoder3To8			sixteenToTwentyThree(decoderOut[23:16], writeAddress[2:0], enable[2]);
	
	// registers 24 to 31
	decoder3To8			twentyFourToThirtyOne(decoderOut[31:24], writeAddress[2:0], enable[3]);
	
	// flips 2d array so we can access by column instead of row
	generate
		for (index = 0; index < 32; index = index + 1)
		begin: gettingBits
			for (index2 = 0; index2 < 32; index2 = index2 + 1)
			begin: gettingBits2
				and			flip(getBit[index][index2], 1'b1, regOut[index2][index]);
			end
		end
	endgenerate
	
	and				isZeroGate0(isZero0, !read0[4], !read0[3], !read0[2], !read0[1], !read0[0]);
	and				isZeroGate1(isZero1, !read1[4], !read1[3], !read1[2], !read1[1], !read1[0]);
	
	// Take in 5 read bits and data from register, read out
	generate
		for (index = 0; index < 32; index = index + 1)
		begin: Muxes
			mux32To1		readMux0(preRead0Out[index], getBit[index][31:0], read0[4:0]);
			mux32To1		readMux1(preRead1Out[index], getBit[index][31:0], read1[4:0]);
		end
	endgenerate
	
	assign read0Out = (read0 == 5'b0) ? 32'b0 : preRead0Out;
	assign read1Out = (read1 == 5'b0) ? 32'b0 : preRead1Out;
	
endmodule

/**************************************************************
*	This module defines the SRAM module
**************************************************************/
//  The module is the SRAM.
//  It is 16 x 2048.

module SRAM(dataOut, dataIn, address, WE, clk);

	output	[15:0]	dataOut;
	
	input   [15:0]  dataIn;		
	input  	[10:0]	address;
	input			clk;
	input			WE;			// WE = write enable, active low:  1 -> read; 0 -> write
	
	reg 	[15:0]	myMemory[2047:0];
	
	integer 			index;
	
	initial
	begin
		for (index = 0; index < 2048; index = index + 1)
		begin
			myMemory[index] = 0;
		end
	end
	
	always@ (posedge clk)
	begin
		if (WE == 1'b0)
		begin
			myMemory[address] <= dataIn;
		end
		
	end
	
	assign dataOut = myMemory[address];
	
endmodule

/**************************************************************
*	This module defines the sign extender
**************************************************************/
//  The module extends 16 bits to 32 bit by
//  propagating the sign bit

module extender(exOut, in);
	
	input	[15:0]	in;
	
	output	[31:0]	exOut;
	
	wire			sign;
	
	assign sign = in[15];
	
	assign exOut = sign ? {16'b1111111111111111, in} : {16'b0, in};
	
endmodule

/**************************************************************
*	This module defines the ALU
**************************************************************/
//  The module does arithmetic and logical functions.
//  Specifically, add, subtract, and, or, xor, slt, and sll
//  slt (set less than), sil (shift left logical)

module ALU(dataOut, z, v, c, n, valA, valB, control);
	
	output	[31:0]	dataOut;
	output 	z, v, c, n;
	
	input	[31:0]	valA;		// input 1
	input	[31:0]	valB;		// input 2
	input	[2:0]	control;	// controls operation
	
	wire	[15:0]	mathOut; 	// adder output
	wire	[15:0]	andOut;		// and operation output
	wire	[15:0]	orOut;		// or operation output
	wire	[15:0]	xorOut;		// xor operation output
	wire	[15:0]	shiftOut;	// barrel shifter output
	wire	[15:0]	mathProp;	// propagate sign bit
	wire			sltOut;		// set less than output
	
	wire	[15:0]	bitsInB;	// input bits B, with or without 2's complement
	wire	[15:0]  mathOutput;	// Holder variable
	wire			overflow;	// carry-out of 16th bit
	
	genvar			index;
	
	assign mathOut = ((!control[2] & !control[1] & control[0]) | 								// add
					 (!control[2] & control[1] & !control[0])) | 								// sub
					 (control[2] & control[1] & !control[0]) ? mathOutput : 16'b0;				// slt
	
	generate
	for (index = 0; index < 16; index = index + 1)
	begin: propSign
		assign mathProp[index] = mathOut[15];
	end
	endgenerate
	
	assign sltOut = mathOut[15];
	
	adder			myAdder(mathOutput, overflow, valA[15:0], bitsInB[15:0]);
	andOp			myAnd(andOut, valA[15:0], valB[15:0]);
	orOp			myOr(orOut, valA[15:0], valB[15:0]);
	xorOp			myXor(xorOut, valA[15:0], valB[15:0]);
	barrelShifter	myShift(shiftOut, valA[15:0], valB[1:0]);
	
	assign bitsInB = (!control[2] & !control[1] & control[0]) ? valB[15:0] : 					// add
					 ((!control[2] & control[1] & !control[0]) |								// sub
					 (control[2] & control[1] & !control[0])) ? (~valB[15:0] + 16'b1) : 16'bz;	// set less
	
	assign dataOut = (!control[2] & !control[1] & !control[0]) ? 32'bz : 					// 000 NOP
					 (!control[2] & !control[1] & control[0]) ? {mathProp, mathOut} :		// 001 ADD
					 (!control[2] & control[1] & !control[0]) ? {mathProp, mathOut} :		// 010 SUB
					 (!control[2] & control[1] & control[0]) ? {16'b0, andOut} :			// 011 AND
					 (control[2] & !control[1] & !control[0]) ? {16'b0, orOut} :			// 100 OR
					 (control[2] & !control[1] & control[0]) ? {16'b0, xorOut} :			// 101 XOR
					 (control[2] & control[1] & !control[0]) & !(valA[15:0] == valB[15:0]) ? {31'b0, sltOut} :			// 110 SLT
					 (control[2] & control[1] & !control[0]) & (valA[15:0] == valB[15:0]) ? {32'b1} :
					 {16'b0, shiftOut};															// 111 SLL
					 
	assign z = (dataOut == 32'b0);
	assign v = ((valA[15] & valB[15] & !mathOut[15] & !overflow) | 								// 2 positive inputs
				(!valA[15] & !valB[15] & mathOut[15] & overflow)) & !z;							// 2 negative inputs
	assign c = (control[2] & control[1] & control[0]) & (valB[0] | valB[1]);
	assign n = mathOut[15];
	
endmodule

/**************************************************************
*	This module defines an adder
**************************************************************/
//  This module adds up to 16 bits at a time.

module adder(out, overflow, bitsInA, bitsInB);
	
	output	[15:0]	out;
	output			overflow;
	
	input	[15:0]	bitsInA;
	input	[15:0]	bitsInB;
	
	wire	[3:0]	carry;
	wire	[3:0]	prop;
	wire	[3:0]	gen;
	
	CLA		bits0To3(out[3:0], gen[0], prop[0], bitsInA[3:0], bitsInB[3:0], 1'b0);
	CLA		bits4To7(out[7:4], gen[1], prop[1], bitsInA[7:4], bitsInB[7:4], carry[0]);
	CLA		bits8To11(out[11:8], gen[2], prop[2], bitsInA[11:8], bitsInB[11:8], carry[1]);
	CLA		bits12To15(out[15:12], gen[3], prop[3], bitsInA[15:12], bitsInB[15:12], carry[2]);
	
	LCU		myLCU(carry, prop, gen, 1'b0);
	
	assign overflow = carry[3];

endmodule

/**************************************************************
*	This module defines a Look-ahead Carry Unit
**************************************************************/
//  This module combines four CLAs to be able to calculate up to 16 bits at a time.

module LCU(cOut, propIn, genIn, cIn);
	
	output	[3:0]	cOut;
	
	input	[3:0]	propIn;
	input	[3:0]	genIn;
	input			cIn;
	
	assign cOut[0] = genIn[0] | (propIn[0] & cIn); 		// carry-out 4
	assign cOut[1] = genIn[1] | (propIn[1] & cOut[0]); 	// carry-out 8
	assign cOut[2] = genIn[2] | (propIn[2] & cOut[1]); 	// carry-out 12
	assign cOut[3] = genIn[3] | (propIn[3] & cOut[2]); 	// carry-out 16

endmodule

/**************************************************************
*	This module defines a carry look-ahead adder
**************************************************************/
//  The module does arithmetic.  It is 4 bits wide.

module CLA(out, genOut, propOut, bitsInA, bitsInB, cIn);
	
	output	[3:0]	out;
	output			propOut;
	output			genOut;
	
	input	[3:0]	bitsInA;
	input	[3:0]	bitsInB;
	input			cIn;
	
	wire	[3:0]	gen;
	wire	[3:0]	prop;
	wire	[3:0]	carry;
	
	assign gen = bitsInA & bitsInB;
	assign prop = bitsInA ^ bitsInB;
	
	assign carry[0] = cIn;
	assign carry[1] = carry[0] & prop[0] | gen[0]; 
	assign carry[2] = carry[1] & prop[1] | gen[1];
	assign carry[3] = carry[2] & prop[2] | gen[2];
	
	assign out = prop ^ carry;
	assign propOut = prop[3] & prop[2] & prop[1] & prop[0];
	assign genOut = gen[3] | (prop[3] & gen[2]) | (prop[3] & prop[2] & gen[1]) | (prop[3] & prop[2] & prop[1] & gen[0]);

endmodule

/**************************************************************
*	This module defines a barrel shifter
**************************************************************/
//  This is a barrel shifter. It shifts 3 to 0 bits to the left.
//  Any bits that fall off the left side are discarded.
//  New bits are 0.

module barrelShifter(shiftOut, shiftIn, shiftBy);
	
	output	[15:0]	shiftOut;
	
	input	[15:0]	shiftIn;
	input	[1:0]	shiftBy;
	
	wire	[15:0]	layer0;
	wire	[15:0]	layer1;
	wire			layer0Ctrl;
	wire			layer1Ctrl;
	wire			layer2Ctrl;
	
	genvar			index;
	
	// These will go into each layer of muxes.
	assign layer0Ctrl = shiftBy[1] | shiftBy[0];	// Shift by at least 1 if true, else 0
	assign layer1Ctrl = shiftBy[1];					// Shift by at least 2 if true, else 1
	assign layer2Ctrl = shiftBy[1] & shiftBy[0];	// Shift by 3 if true, else 2.
	
	mux2To1			startMux0(layer0[0], 1'b0, shiftIn[0], layer0Ctrl);
	mux2To1			startMux1(layer1[0], 1'b0, layer0[0], layer1Ctrl);
	mux2To1			startMux2(shiftOut[0], 1'b0, layer1[0], layer2Ctrl);
	
	generate
	for (index = 1; index < 16; index = index + 1)
	begin: shiftLayers
		mux2To1 	muxLayer0(layer0[index], shiftIn[index - 1], shiftIn[index], layer0Ctrl);
		mux2To1 	muxLayer1(layer1[index], layer0[index - 1], layer0[index], layer1Ctrl);
		mux2To1 	muxLayer2(shiftOut[index], layer1[index - 1], layer1[index], layer2Ctrl);
	end
	endgenerate
	
endmodule

/**************************************************************
*	This module defines a 2 to 1 mux
**************************************************************/
//  This is a 2 to 1 mux

module mux2To1(muxOut, muxIn1, muxIn0, ctrl);
	
	output			muxOut;
	
	input			muxIn1;
	input			muxIn0;
	input			ctrl;
	
	assign muxOut = ctrl ? muxIn1 : muxIn0;

endmodule

/**************************************************************
*	This module defines the AND operation
**************************************************************/
//  This compares inputs with an AND gate, with each corresponding bit

module andOp(andOut, bitsInA, bitsInB);
	
	output	[15:0]	andOut;
	
	input	[15:0]	bitsInA;
	input	[15:0]	bitsInB;
	
	genvar index;
	
	generate
	for (index = 0; index < 16; index = index + 1)
	begin: andCreate
		assign andOut[index] = bitsInA[index] & bitsInB[index];
	end
	endgenerate

endmodule

/**************************************************************
*	This module defines the OR operation
**************************************************************/
//  This compares inputs with an OR gate, with each corresponding bit

module orOp(orOut, bitsInA, bitsInB);
	
	output	[15:0]	orOut;
	
	input	[15:0]	bitsInA;
	input	[15:0]	bitsInB;
	
	genvar index;
	
	generate
	for (index = 0; index < 16; index = index + 1)
	begin: orCreate
		assign orOut[index] = bitsInA[index] | bitsInB[index];
	end
	endgenerate

endmodule

/**************************************************************
*	This module defines the XOR operation
**************************************************************/
//  This compares inputs with an XOR gate, with each corresponding bit

module xorOp(xorOut, bitsInA, bitsInB);
	
	output	[15:0]	xorOut;
	
	input	[15:0]	bitsInA;
	input	[15:0]	bitsInB;
	
	genvar index;
	
	generate
	for (index = 0; index < 16; index = index + 1)
	begin: xorCreate
		assign xorOut[index] = bitsInA[index] ^ bitsInB[index];
	end
	endgenerate

endmodule

/**************************************************************
*	This module defines the pc
**************************************************************/
//  This is the program counter

module pc(pcOut, pcIn, clk, rst);
	
	output	[31:0]	pcOut;
	
	input rst;
	input	[31:0]	pcIn;
	input			clk;
	
	reg		[31:0]	programCounter;
	
	assign pcOut = programCounter;
	
	initial
	begin
		programCounter = 32'b0;
	end
	
	always@ (posedge clk)
	begin
		if(rst)
			programCounter <= 32'b0;
		else
			programCounter <= pcIn;
	end
	
endmodule

/**************************************************************
*	This module defines the pc adder
**************************************************************/
//  This adds the branch offset to the pc
module pcAdder(addOut, pc, shiftImmediate);
	
	output	[31:0]	addOut;
	
	wire			overflow;
	wire	[15:0]	mathOutput;
	
	input	[31:0]	pc;
	input	[31:0]	shiftImmediate;
	
	adder			myAdder(mathOutput, overflow, pc[15:0], shiftImmediate[15:0]);
	extender		myExtender(addOut, mathOutput);
	
endmodule

/**************************************************************
*	This module defines the instruction memory
**************************************************************/
//  The module contains the instructions we want to run
//  This is 128x32 architecture

module instructionMem(code, address);
	
	// read-something command in verilog lets you read directly from text file
	input		[6:0]	address;
	
	output		[31:0]	code;
	
	reg			[31:0]	registers[127:0];

	initial
	begin
		$readmemb("lab5demo1.txt", registers, 0, 127);
	end
	
	// Reading out from insMem
	assign code = registers[address];
	
endmodule

/**************************************************************
*	This module defines the pc adder
**************************************************************/
//  This module adds one to the pc

module add1(addOut, pc);
	
	output	[31:0]	addOut;
	
	wire			overflow;
	wire	[15:0]	mathOutput;
	
	input	[31:0]	pc;
	
	adder			myAdder(mathOutput, overflow, pc[15:0], 16'b1);
	extender		myExtender(addOut, mathOutput);
	
endmodule

/**************************************************************
*	This module defines the instruction register
**************************************************************/
//  It holds the instructions responsible for fetching and decoding

module ifId(iOut, pcOut, iIn, pcIn, flush, clk);

	output	[31:0]	iOut;
	output	[31:0]	pcOut;

	input	[31:0]	iIn;
	input	[31:0]	pcIn;
	input			clk;
	input			flush;
	
	reg		[31:0]	iReg;
	reg		[31:0]	pcReg;
	
	initial
	begin
		iReg = {6'b011100, 26'b0};
		pcReg = 32'b0;
	end
	
	always@ (posedge clk)
	begin
		if (!flush)
		begin
			iReg <= iIn;
			pcReg <= pcIn;
		end
		else
		begin
			iReg <= {6'b011100, 26'b0};
			pcReg <= 32'b0;
		end
	end
	
	assign iOut = iReg;
	assign pcOut = pcReg;

endmodule

/**************************************************************
*	This module defines the execution register
**************************************************************/
//  This module holds the values from the register to be passed into
//  execution

module idEx(r1Out, r2Out, extendOut, pcOut, rwRegOut, rRegOut, wRegOut, wbOut, mOut, exOut, bpOut,
			r1In, r2In, extendIn, pcIn, rwRegIn, rRegIn, wRegIn, wbIn, mIn, exIn, flush, bpIn, clk);

	output	[31:0]	r1Out;
	output	[31:0]	r2Out;
	output	[31:0]	extendOut;
	output	[31:0]	pcOut;
	output			bpOut;
	
	// instructions 20:16, 15:11
	// we need to take these values with us for each stage
	output	[4:0]	rwRegOut;	// 20:16
	output	[4:0]	wRegOut;	// 15:11
	output	[4:0]	rRegOut;	// 25:21
	
	// control bits
	output	[1:0]	wbOut;		// regWE, memToReg
	output	[3:0]	mOut;		// sramWE, branch, jumpSel, jr
	output	[3:0]	exOut;		// regDst, aluSrc, aluOp[1:0]
	
	// define inputs
	input	[31:0]	r1In;
	input	[31:0]	r2In;
	input	[31:0]	extendIn;
	input	[31:0]	pcIn;
	
	input	[4:0]	rwRegIn;
	input	[4:0]	wRegIn;
	input	[4:0]	rRegIn;
	
	input	[1:0]	wbIn;		// regWE, memToReg
	input	[3:0]	mIn;		// sramWE, branch, jumpSel, jr
	input	[3:0]	exIn;		// regDst, aluSrc, aluOp[1:0]
	
	input			bpIn;
	input			clk;
	input			flush;
	
	// registers
	reg		[31:0]	r1Reg;
	reg		[31:0]	r2Reg;
	reg		[31:0]	extendReg;
	reg		[31:0]	pcReg;
	
	reg		[4:0]	rwReg;
	reg		[4:0]	wReg;
	reg		[4:0]	rReg;
	
	reg		[1:0]	wbReg;		// regWE, memToReg
	reg		[3:0]	mReg;		// sramWE, branch, jumpSel, jr
	reg		[3:0]	exReg;		// regDst, aluSrc, aluOp[1:0]
	reg				bpReg;
	
	initial
	begin
		r1Reg = 32'b0;
		r2Reg = 32'b0;
		extendReg = 32'b0;
		pcReg = 32'b0;
		rwReg = 5'b0;
		wReg = 5'b0;
		wbReg = 2'b10;
		mReg = 4'b1000;
		exReg = 4'b0010;
		rReg = 5'b0;
		bpReg = 1'b0;
	end
	
	always@ (posedge clk)
	begin
		if (!flush)
		begin
			r1Reg <= r1In;
			r2Reg <= r2In;
			extendReg <= extendIn;
			pcReg <= pcIn;
			rwReg <= rwRegIn;
			wReg <= wRegIn;
			wbReg <= wbIn;
			mReg <= mIn;
			exReg <= exIn;
			rReg <= rRegIn;
			bpReg <= bpIn;
		end
		else
		begin
			r1Reg <= 32'b0;
			r2Reg <= 32'b0;
			extendReg <= 32'b0;
			pcReg <= 32'b0;
			rwReg <= 5'b0;
			wReg <= 5'b0;
			wbReg <= 2'b10;
			mReg <= 4'b1000;
			exReg <= 4'b0010;
			rReg <= 5'b0;
			bpReg <= 1'b0;
		end
	end	
	
	assign r1Out = r1Reg;
	assign r2Out = r2Reg;
	assign extendOut = extendReg;
	assign pcOut = pcReg;
	assign rwRegOut = rwReg;
	assign wRegOut = wReg;
	assign wbOut = wbReg;
	assign mOut = mReg;
	assign exOut = exReg;
	assign rRegOut = rReg;
	assign bpOut = bpReg;

endmodule

/**************************************************************
*	This module defines the memory register
**************************************************************/
//  This module contains the results from execution to be stored in
//  SRAM or written back

module exMem(aluOut, regWOut, wbOut, mOut, flagOut, exMemB, aluIn, regWIn, wbIn, mIn, flagIn, exMemBIn, clk);

	output	[31:0]	aluOut;
	output	[4:0]	regWOut;
	output	[3:0]	flagOut;
	output	[10:0]	exMemB;
	
	output	[1:0]	wbOut;		// regWE, memToReg
	output	[3:0]	mOut;		// sramWE, branch, jumpSel, jr
	
	input	[31:0]	aluIn;
	input	[4:0]	regWIn;
	input	[10:0]	exMemBIn;
	
	input	[1:0]	wbIn;		// regWE, memToReg
	input	[3:0]	mIn;		// sramWE, branch, jumpSel, jr
	input	[3:0]	flagIn;
	
	input			clk;
	
	reg		[31:0]	aluReg;
	reg		[4:0]	regWReg;
	reg		[10:0]	bReg;
	
	reg		[1:0]	wbReg;		// regWE, memToReg
	reg		[3:0]	mReg;		// sramWE, branch, jumpSel, jr
	reg		[3:0]	flagReg;
	
	initial
	begin
		aluReg = 32'b0;
		regWReg = 5'b0;
		wbReg = 2'b10;
		mReg = 4'b1000;
		flagReg = 4'b0;
		bReg = 11'b0;
	end	
	
	always@ (posedge clk)
	begin
		aluReg <= aluIn;
		regWReg <= regWIn;
		wbReg <= wbIn;
		mReg <= mIn;
		flagReg <= flagIn;
		bReg <= exMemBIn;
	end	
	
	assign aluOut = aluReg;
	assign regWOut = regWReg;
	assign wbOut = wbReg;
	assign mOut = mReg;
	assign flagOut = flagReg;
	assign exMemB = bReg;

endmodule

/**************************************************************
*	This module defines the write-back register
**************************************************************/
//  This module contains the values to be written back into
//  registers

module memWb(sramOut, aluOut, regWOut, wbOut, sramInEx, aluIn, regWIn, wbIn, clk);

	output	[31:0]	sramOut;
	output	[31:0]	aluOut;
	output	[4:0]	regWOut;
	output	[1:0]	wbOut;		// regWE, memToReg
	
	input	[31:0]	sramInEx;
	input	[31:0]	aluIn;
	input	[4:0]	regWIn;
	input	[1:0]	wbIn;
	
	input			clk;
	
	reg		[31:0]	sramRegEx;
	reg		[31:0]	aluReg;
	reg		[4:0]	regWReg;
	reg		[1:0]	wbReg;
	
	initial
	begin
		sramRegEx = 32'b0;
		aluReg = 32'b0;
		regWReg = 5'b0;
		wbReg = 2'b10;
	end	
	
	always@ (posedge clk)
	begin
		sramRegEx <= sramInEx;
		aluReg <= aluIn;
		regWReg <= regWIn;
		wbReg <= wbIn;
	end	
	
	assign sramOut = sramRegEx;
	assign aluOut = aluReg;
	assign regWOut = regWReg;
	assign wbOut = wbReg;
	
endmodule

/**************************************************************
*	This module defines the forwarding module
**************************************************************/
//  When we calculate a value and try to save it back, because it is in a pipeline,
//  sometimes the value has not been saved before it is called again.  Therefore, this
//  module checks the write addresses of execution to memory and memory to writeback
//  and compares them to the read address and the read/write address of decode to execution. 
//  If they match, then we must replace the input to the ALU with one of the output values.

module fwdUnit(aluMuxCtrl1, aluMuxCtrl2, rReg, rw, wAdr1, wAdr2, wb1, wb2);
	
	output	[1:0]	aluMuxCtrl1;
	output	[1:0]	aluMuxCtrl2;
	
	input	[4:0]	rReg;		// read register, 25:21
	input	[4:0]	rw;			// read write register 20:16
	input	[4:0]	wAdr1;		// write address of data in execution to memory
	input	[4:0]	wAdr2;		// write address of data in memory to writeback
	
	input	[1:0]	wb1;		// wb from execution to memory control bits
	input	[1:0]	wb2;		// wb from memory to writeback control bits
	
	reg		[1:0]	aluMuxCtrl1;
	reg		[1:0]	aluMuxCtrl2;
	
	// alu mux control cases:
	// 00 is values from register
	// 01 result is from mem/wb
	// 10 result is from ex/mem
	
	// if wb1, we want the most recent values, or 01
	// if wb2, we want the next set of values, or 10
	
	// rReg is the top mux aka valA
	// rw is the bottom mux aka valB
	
	always@ (*)
	begin
		if (wb1[1] == 1'b0 && wAdr1 == rReg)	// exMem = 25:21
		begin
			aluMuxCtrl1 = 2'b10;				// select exMem
		end
		else if (wb2[1] == 1'b0 && wAdr2 == rReg)	// memWb = 25:21
		begin
			aluMuxCtrl1 = 2'b01;				// select memWb
		end
		else
		begin
			aluMuxCtrl1 = 2'b00;				// select input
		end

		if (wb1[1] == 1'b0 && wAdr1 == rw)		// control val b
		begin
			aluMuxCtrl2 = 2'b10;
		end
		else if (wb2[1] == 1'b0 && wAdr2 == rw)
		begin
			aluMuxCtrl2 = 2'b01;
		end
		else
		begin
			aluMuxCtrl2 = 2'b00;
		end
	end
	
endmodule

/**************************************************************
*	This module is a 3-to-1 mux
**************************************************************/
//  This is a 3-to-1 mux

module mux3To1(out, in2, in1, in0, ctrl);

	output			out;

	input			in2;
	input			in1;
	input			in0;
	
	input	[1:0]	ctrl;
	
	wire			muxToMux0;
	wire			muxToMux1;
	
	// msb CBA lsb
	mux2To1			mux1 (muxToMux0, in2, in0, ctrl[1]);
	mux2To1			mux2 (muxToMux1, 1'b0, in1, ctrl[1]);
	mux2To1			mux3 (out, muxToMux1, muxToMux0, ctrl[0]);

endmodule

/**************************************************************
*	This module defines the hazard module
**************************************************************/
//  If we are loading, we need to let the load operation go one cycle
//  so we can forward the answer.  This module also deals with branching.
//  If it takes branches often, then it is more likely to predict taking the branch
//  and thus preloading the values.  If wrong, flushes out and returns to the branch address
//  If we do not branch often, we predict not taking the branch and continue as normal.
//  If wrong, we flush and the calculated address goes to the pc.

module hazardUnit(bPredict, stall, addrOut, rw, idExM, instructionIn, bTaken, aluOpIdEx, aluOpCurr, jump, addrIn, clk);
	
	output			stall;		// controls stall mux and control mux, high true
	output			bPredict;	// predicts based on branchP case whether or not we will branch
	output	[31:0]	addrOut;	// saved stall address
	
	input	[4:0]	rw;
	input			idExM;		// sramWE
	input	[31:0]	instructionIn;
	input			bTaken;		// did we take the branch?
	input	[1:0]	aluOpIdEx;	// alu operation from idex
	input	[1:0]	aluOpCurr;	// alu operation from ifid
	input			jump;		// includes both jr and jump
	input	[31:0]	addrIn;		// input stall address
	input			clk;
	
	reg		[1:0]	branchP;	// cases
	reg		[1:0]	stallReg;	// wait 1 cycle, things going into control -> NOP, hold pc at current value
	reg				pcCtrl;		// 1 is reset, else continue as before
	reg				bPredict;	// predict if branch or not
	reg		[31:0]	address;	// register for stall address
	
	assign stall = 0;
	assign addrOut = address;
	
	initial
	begin
		branchP = 2'b00;
		bPredict = 1'b1;
		stallReg = 2'b00;
	end
	
	always@ (*)
	begin
		if (aluOpCurr == 2'b01)
		begin
			bPredict <= !branchP[1];
		end
		else
		begin
			bPredict <= 1'b0;
		end
	end
	
	// branch hazard
	always@ (*)
	begin
		if (aluOpIdEx == 2'b01)		// 01 is bgt
		begin
			case(branchP)
				2'b00:		// strongly taken
					begin
						if (bTaken)
						begin
							branchP <= 2'b00;
						end
						else
						begin
							branchP <= 2'b01;
						end
					end
				2'b01:		// taken
					begin
						if (bTaken)
						begin
							branchP <= 2'b00;
						end
						else
						begin
							branchP <= 2'b10;
						end
					end
				2'b10:		// not taken
					begin
					if (bTaken)
						begin
							branchP <= 2'b01;
						end
						else
						begin
							branchP <= 2'b11;
						end
					end
				2'b11:		// strongly not taken
					begin
						if (bTaken)
						begin
							branchP <= 2'b10;
						end
						else
						begin
							branchP <= 2'b11;
						end
					end
			endcase
		end
	end
	
	// data hazards
	always@ (posedge clk)
	begin
		if ((idExM == 1'b0) && (instructionIn[31:26] == 6'b000000) && stallReg == 2'b00)		// lw and then ALU op
		begin
			if (rw == instructionIn[25:21] || rw == instructionIn[20:16])
			begin
				stallReg <= 2'b11;		// we want to stall, so read only, meaning sramWE, regWE are 1.
				address <= addrIn;
			end
			else
			begin
				stallReg <= 2'b00;
			end
		end
		else if (stallReg == 2'b11)
		begin
			stallReg <= 2'b01;
		end
		else
		begin
			stallReg <= 2'b00;
		end	
	end
	
endmodule

/**************************************************************
*	This module defines the test bench
**************************************************************/
//  This module is for testing

module testBench;

	wire			rst, clk;
	wire	[31:0]	o;

	integration	myIntegration (o, rst, clk);
	TestModule  myTester (rst, clk, o);

	initial
	begin
	  $dumpfile("int0.vcd");
	  $dumpvars(1, myIntegration);
	end

endmodule

/**************************************************************
*	Define the tester module.
**************************************************************/

module TestModule (rst, clk, o);
	//	Declare variables:

	output				rst;
	output				clk;
	
	input		[31:0]	o;
	
	reg 				clk;
	reg					rst;
	
	integer				counter;
	
	parameter			stimDelay = 10;

	initial
	begin
		clk = 0;
		rst = 0;
	end
	
	always #5 clk = ~clk;
	
	initial                      			// this initial block will apply the test vectors
	
    begin                       			// begin INITIAL loop
		
		// 	obtain graphical waveform shiftOutput of signals
		$display("\tPIPELINE\t\tTIME");
		$display("addr----output---clk");  
		$monitor("%b\t%b\t%d", o, clk, counter);

		// begin a second block to perform the simulation 
		begin
			for(counter = 1; counter < 80; counter = counter + 1)
				begin
					#stimDelay rst = 1;
				end
		end
				
		#(2*stimDelay);		        		// needed to see END of simulation
		$stop;								// temporarily stops simulation, goto 
											// Verilog Interactive mode 
											// need to type 1.1 or '$finish;' 
		$finish;							// finish simulation 

	end		    							// close second initial loop

endmodule									// close test-module