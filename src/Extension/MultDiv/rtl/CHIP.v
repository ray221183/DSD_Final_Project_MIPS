// Top module of your design, you cannot modify this module!!
// `include "./MIPS_Pipeline_MultDiv.v" //
// `include "./MIPS_Pipeline_hasHazard.v" //
// `include "./cache_dm.v"

module CHIP (	clk,
				rst_n,
//----------for slow_memD------------
				mem_read_D,
				mem_write_D,
				mem_addr_D,
				mem_wdata_D,
				mem_rdata_D,
				mem_ready_D,
//----------for slow_memI------------
				mem_read_I,
				mem_write_I,
				mem_addr_I,
				mem_wdata_I,
				mem_rdata_I,
				mem_ready_I,
//----------for TestBed--------------				
				DCACHE_addr, 
				DCACHE_wdata,
				DCACHE_wen   
			);
input			clk, rst_n;
//--------------------------

output			mem_read_D;
output			mem_write_D;
output	[31:4]	mem_addr_D;
output	[127:0]	mem_wdata_D;
input	[127:0]	mem_rdata_D;
input			mem_ready_D;
//--------------------------
output			mem_read_I;
output			mem_write_I;
output	[31:4]	mem_addr_I;
output	[127:0]	mem_wdata_I;
input	[127:0]	mem_rdata_I;
input			mem_ready_I;
//----------for TestBed--------------
output	[29:0]	DCACHE_addr;
output	[31:0]	DCACHE_wdata;
output			DCACHE_wen;
//--------------------------

// wire declaration
wire        ICACHE_ren;
wire        ICACHE_wen;
wire [29:0] ICACHE_addr;
wire [31:0] ICACHE_wdata;
wire        ICACHE_stall;
wire [31:0] ICACHE_rdata;

wire        DCACHE_ren;
wire        DCACHE_wen;
wire [29:0] DCACHE_addr;
wire [31:0] DCACHE_wdata;
wire        DCACHE_stall;
wire [31:0] DCACHE_rdata;

//=========================================
	// Note that the overall design of your MIPS includes:
	// 1. pipelined MIPS processor
	// 2. data cache
	// 3. instruction cache


	MIPS_Pipeline i_MIPS(
		// control interface
		.clk            (clk)           , 
		.rst_n          (rst_n)         ,
//----------I cache interface-------		
		.ICACHE_ren     (ICACHE_ren)    ,
		.ICACHE_wen     (ICACHE_wen)    ,
		.ICACHE_addr    (ICACHE_addr)   ,
		.ICACHE_wdata   (ICACHE_wdata)  ,
		.ICACHE_stall   (ICACHE_stall)  ,
		.ICACHE_rdata   (ICACHE_rdata)  ,
//----------D cache interface-------
		.DCACHE_ren     (DCACHE_ren)    ,
		.DCACHE_wen     (DCACHE_wen)    ,
		.DCACHE_addr    (DCACHE_addr)   ,
		.DCACHE_wdata   (DCACHE_wdata)  ,
		.DCACHE_stall   (DCACHE_stall)  ,
		.DCACHE_rdata   (DCACHE_rdata)
	);
	
	cache D_cache(
        .clk        (clk)         ,
        .proc_reset (~rst_n)      ,
        .proc_read  (DCACHE_ren)  ,
        .proc_write (DCACHE_wen)  ,
        .proc_addr  (DCACHE_addr) ,
        .proc_rdata (DCACHE_rdata),
        .proc_wdata (DCACHE_wdata),
        .proc_stall (DCACHE_stall),
        .mem_read   (mem_read_D)  ,
        .mem_write  (mem_write_D) ,
        .mem_addr   (mem_addr_D)  ,
        .mem_wdata  (mem_wdata_D) ,
        .mem_rdata  (mem_rdata_D) ,
        .mem_ready  (mem_ready_D)
	);

	cache I_cache(
        .clk        (clk)         ,
        .proc_reset (~rst_n)      ,
        .proc_read  (ICACHE_ren)  ,
        .proc_write (ICACHE_wen)  ,
        .proc_addr  (ICACHE_addr) ,
        .proc_rdata (ICACHE_rdata),
        .proc_wdata (ICACHE_wdata),
        .proc_stall (ICACHE_stall),
        .mem_read   (mem_read_I)  ,
        .mem_write  (mem_write_I) ,
        .mem_addr   (mem_addr_I)  ,
        .mem_wdata  (mem_wdata_I) ,
        .mem_rdata  (mem_rdata_I) ,
        .mem_ready  (mem_ready_I)
	);
endmodule


//cache dm v3
module cache(
    clk,
    proc_reset,
    proc_read,
    proc_write,
    proc_addr,
    proc_rdata,
    proc_wdata,
    proc_stall,
    mem_read,
    mem_write,
    mem_addr,
    mem_rdata,
    mem_wdata,
    mem_ready
);
    
//==== input/output definition ============================
    input          clk;
    // processor interface
    input          proc_reset;
    input          proc_read, proc_write;
    input   [29:0] proc_addr;
    input   [31:0] proc_wdata;
    output         proc_stall;
    output  [31:0] proc_rdata;
    // memory interface
    input  [127:0] mem_rdata;
    input          mem_ready;
    output         mem_read, mem_write;
    output  [27:0] mem_addr;
    output [127:0] mem_wdata;
    
//==== wire/reg definition ================================
    // internal
    reg         valid     [0:7];
    reg         valid_nxt [0:7];
    reg         dirty     [0:7];
    reg         dirty_nxt [0:7];
    reg [24:0]  tag       [0:7];
    reg [24:0]  tag_nxt   [0:7];
    reg [127:0] data      [0:7];
    reg [127:0] data_nxt  [0:7];
    reg hit;
    // output
    reg proc_stall_c;
    reg [31:0] proc_rdata_c;
    reg mem_read_c;
    reg mem_write_c;
    reg [27:0] mem_addr_c;
    reg [127:0] mem_wdata_c;
    reg test;

    wire askToDo;
    assign askToDo = proc_read || proc_write;
    
    integer i;

//==== combinational circuit ==============================
    //assign
    assign proc_stall = proc_stall_c;
    assign proc_rdata = proc_rdata_c;
    assign mem_read = mem_read_c;
    assign mem_write = mem_write_c;
    assign mem_addr = mem_addr_c;
    assign mem_wdata = mem_wdata_c;
    
    always @(*) begin
        hit = ( valid[proc_addr[4:2]] && ( proc_addr[29:5] == tag[proc_addr[4:2]] ) );
        //revise
        if(!(proc_read || proc_write) || hit) begin
            proc_stall_c = 0;
        end 
        else begin
            proc_stall_c = 1;
        end   
    end

    always @(*) begin //output

        for(i=0;i<8;i=i+1) begin
            valid_nxt[i] = valid[i];
            dirty_nxt[i] = dirty[i];
            tag_nxt[i]   = tag[i];
            data_nxt[i]  = data[i];
        end
        proc_rdata_c = 0;
        mem_read_c = 0;
        mem_write_c = 0;
        mem_addr_c = 0;
        mem_wdata_c = 0;

        if(hit) begin
            if(proc_read) begin
                case (proc_addr[1:0])
                    2'd0: proc_rdata_c = data[proc_addr[4:2]][31:0];
                    2'd1: proc_rdata_c = data[proc_addr[4:2]][63:32];
                    2'd2: proc_rdata_c = data[proc_addr[4:2]][95:64];
                    2'd3: proc_rdata_c = data[proc_addr[4:2]][127:96];
                    default: proc_rdata_c = 32'd0;
                endcase
                mem_read_c = 0;
                mem_write_c = 0;
            end
            if(proc_write) begin
                case (proc_addr[1:0])
                    2'd0: data_nxt[proc_addr[4:2]] = {data[proc_addr[4:2]][127:32], proc_wdata};
                    2'd1: data_nxt[proc_addr[4:2]] = {data[proc_addr[4:2]][127:64], proc_wdata, data[proc_addr[4:2]][31:0]};
                    2'd2: data_nxt[proc_addr[4:2]] = {data[proc_addr[4:2]][127:96], proc_wdata, data[proc_addr[4:2]][63:0]};
                    2'd3: data_nxt[proc_addr[4:2]] = {proc_wdata, data[proc_addr[4:2]][95:0]};
                    default: data_nxt[proc_addr[4:2]] = data[proc_addr[4:2]];
                endcase
                dirty_nxt[proc_addr[4:2]] = 1;
            end
        end
        else begin //miss
            if(dirty[proc_addr[4:2]]) begin //write back
                if(mem_ready) begin
                    if(askToDo) begin
                        mem_read_c = 0;
                        mem_write_c = 0;
                        dirty_nxt[proc_addr[4:2]] = 0;
                    end
                end
                else begin
                    if(askToDo) begin
                        proc_rdata_c = 0;
                        mem_read_c = 0;
                        mem_write_c = 1;
                        mem_addr_c = {tag[proc_addr[4:2]], proc_addr[4:2]};
                        mem_wdata_c = data[proc_addr[4:2]];
                    end
                end
            end
            else begin //read mem
                if(mem_ready) begin
                    if(askToDo) begin
                        mem_read_c = 0;
                        mem_write_c = 0;
                        valid_nxt[proc_addr[4:2]] = 1;
                        dirty_nxt[proc_addr[4:2]] = 0;
                        tag_nxt[proc_addr[4:2]] = proc_addr[29:5];
                        data_nxt[proc_addr[4:2]] = mem_rdata;
                    end
                end
                else begin
                    if(askToDo) begin
                        mem_read_c = 1;
                        mem_write_c = 0;
                        mem_addr_c = proc_addr[29:2];
                    end
                end
            end
        end       
    end

    

//==== sequential circuit =================================
    always@( posedge clk ) begin
        if( proc_reset ) begin
            for(i=0;i<8;i=i+1) begin
                valid[i] <= 0;
                dirty[i] <= 0; 
            end
        end
        else begin
            for(i=0;i<8;i=i+1) begin
                valid[i] <= valid_nxt[i];
                dirty[i] <= dirty_nxt[i];
            end
        end
    end

    always @( posedge clk ) begin
        for(i=0;i<8;i=i+1) begin
            tag[i] <= tag_nxt[i];
            data[i] <= data_nxt[i];
        end
    end

endmodule


//MultDiv
module MIPS_Pipeline(
// control interface
		input clk, 
		input rst_n,
//----------I cache interface-------		
		output        ICACHE_wen,   //0
		output        ICACHE_ren,   //(stall) ? 1'b0 : 1'b1;
		output [29:0] ICACHE_addr,  //pc[31:2]
		output [31:0] ICACHE_wdata, //32'd0
		input         ICACHE_stall, //stall all register
		input  [31:0] ICACHE_rdata,
//----------D cache interface-------
		output        DCACHE_ren,
	    output        DCACHE_wen,
		output [29:0] DCACHE_addr,
		output [31:0] DCACHE_wdata,
		input         DCACHE_stall,
		input  [31:0] DCACHE_rdata
);
//
integer i;
//reg wire
//IF
reg  [31:0] pc_br;
reg  [31:0] pc, pc_next, pc_IF;
wire [31:0] pcPlusFour;
reg  [31:0] reg_file [0:31];
reg  [31:0] reg_file_next [0:31];
reg  [31:0] inst_IF;
wire [31:0] inst_IF_next;

//ID
wire [31:0] ReadData1;
reg  [31:0] ReadData1_ID;
wire [31:0] ReadData2;
reg  [31:0] ReadData2_ID;
reg  [31:0] WMemData_ID; 
reg  [4:0]  rs_ID;
reg  [4:0]  rt_ID;
wire [4:0]  WBAddr;
reg  [4:0]  WBAddr_ID;
reg  [31:0] branch1;
reg  [31:0] branch2;
reg  [4:0]  shamt_ID;
//EX
reg  [31:0] ALUout_EX;
reg  [31:0] WMemData_EX, WMemData_EX_next;
reg  [4:0]  WBAddr_EX;

//MW
wire [31:0] WBData_MW; //write back data
reg  [31:0] DMemRData;
wire [31:0] DMemRData_next;
reg  [31:0] ALUout_MW;
reg  [31:0] ALUout_MW_next;
reg  [4:0]  WBAddr_MW; //write back address


/////////////////////////////////////////////////////
////////////// name could be changed ////////////////
/////////////////////////////////////////////////////
//hazard////////////
wire stallAll; //from D cache and I cache
///////////////////
//data hazard
//forward                //from forward unit
wire [1:0] ForwardA;
wire [1:0] ForwardB;
wire [1:0] ForwardC;
wire [1:0] ForwardA_br;
wire [1:0] ForwardB_br;
//stall                  //from stall unit
wire instFlush;
wire stall;
wire IMemNoread;
///////////////////
//control hazard         //branch handle
wire compareEqual;
///////////////////
//control
// wire       jump; //?
// wire       branch; //?
// wire       jal; //?
wire       maystall;
wire [1:0] pcDst, pcDst_o;
wire       WrDst, WrDst_o;
reg        WrDst_ID, WrDst_EX, WrDst_MW; //?
wire [1:0] RegDst, RegDst_o;
wire       ALUsrc, ALUsrc_o;
reg        ALUsrc_ID;
wire [3:0] ALUop, ALUop_o; 
reg  [3:0] ALUop_ID;
wire       MemWrite, MemWrite_o;
reg        MemWrite_ID, MemWrite_EX;
wire       MemRead, MemRead_o;
reg        MemRead_ID, MemRead_EX;
wire       RegWrite, RegWrite_o;
reg        RegWrite_ID, RegWrite_EX, RegWrite_MW;
wire       Mem2Reg, Mem2Reg_o;
reg        Mem2Reg_ID, Mem2Reg_EX, Mem2Reg_MW;
wire       regHiLo;
reg        regHiLo_ID;
wire       MulOrDiv; 
reg        MulOrDiv_ID; // 0 Mul | 1 Div
wire [1:0] fromHiLo;
reg  [1:0] fromHiLo_ID, fromHiLo_EX;


wire [35:0] ctrl_out;
assign ctrl_out = {3'd0, Mem2Reg, 3'd0, RegWrite, 3'd0, MemRead, 3'd0, MemWrite, ALUop, 3'd0, ALUsrc, 2'd0, RegDst, 3'd0, WrDst, 2'd0, pcDst};
//ALU
reg   [31:0] ALU1;
reg   [31:0] ALU2;
reg  [31:0] ALUout;
//MultDiv;
wire        stall_MD;

reg  [5:0]  counter, counter_next;
reg  [63:0] HILO, HILO_next;
reg  [31:0] ALU1_MUL, ALU1_next;
reg  [31:0] remainder, remainder_next;
reg  [31:0] quotient, quotient_next;
wire [31:0] difference;
wire        complete;
wire [32:0] partial_sum;

assign complete = (counter_next == 32) ? 1 : 0;
assign stall_MD = (regHiLo_ID && !complete) ? 1 : 0;
assign partial_sum = (ALU1_next[0]) ? 
                        (counter == 0) ? ALU2 : 
                                         $signed(ALU2) + $signed({HILO[63], HILO[63:32]}) : 
                        (counter == 0) ? 0 : 
                                         {HILO[63], HILO[63:32]};
assign difference = (counter == 0) ? {31'd0, ALU1[31]} - ALU2 : remainder - ALU2;
always@(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        counter <= 6'd0;
        HILO <= 64'd0;
        ALU1_MUL <= 32'd0;
        remainder <= 32'd0;
        quotient <= 32'd0;
    end
    else begin
        counter <= (complete) ? 6'd0 : counter_next;
        HILO <= (regHiLo_ID) ? HILO_next : HILO;
        ALU1_MUL <= ALU1_next;
        remainder <= remainder_next;
        quotient <= quotient_next;
    end
end
always@(*) begin 
    counter_next = counter;
    HILO_next = HILO;
    ALU1_next = (counter == 0) ? ALU1 : {1'd0, ALU1_MUL[31:1]};
    remainder_next = remainder;
    quotient_next = quotient;
    if(regHiLo_ID && !MulOrDiv_ID) begin //Multiply
        if(counter == 0) HILO_next[30:0] = 31'd0;
        else HILO_next[30:0] = HILO[31:1];
        HILO_next[63:31] = partial_sum;
        counter_next = counter + 1;
    end
    if(regHiLo_ID && MulOrDiv_ID) begin //Div
        if(counter == 0) quotient_next = {ALU1[29:0], ~difference[31], 1'd0};
        else quotient_next = {quotient[30:0], ~difference[31]};
        if(counter == 0) begin
            if(difference[31]) remainder_next = {30'd0, ALU1[31:30]};
            else remainder_next = {difference[30:0], ALU1[30]};
        end
        else begin
            if(difference[31]) remainder_next = {remainder[30:0], quotient[31]};
            else remainder_next = {difference[30:0], quotient[31]};
        end
        counter_next = counter + 1;
        HILO_next = (complete) ? {remainder_next, quotient_next} : 0;
    end
end
/////////////////////////////////////////////////////
/////////////////////////////////////////////////////
/////////////////////////////////////////////////////

assign pcDst = pcDst_o;
assign WrDst = (stall) ? 0 : WrDst_o;
assign RegDst = RegDst_o;
assign ALUsrc = ALUsrc_o;
assign ALUop = (stall) ? 0 : ALUop_o;
assign MemWrite = (stall) ? 0 : MemWrite_o;
assign MemRead = (stall) ? 0 : MemRead_o;
assign RegWrite = (stall) ? 0  : RegWrite_o;
assign Mem2Reg = Mem2Reg_o;



//IF
assign ICACHE_addr  = pc[31:2];
assign ICACHE_wen   = 0;
assign ICACHE_ren   = 1;//(IMemNoread) ? 1'b0 : 1'b1;
assign ICACHE_wdata = 32'd0;
assign pcPlusFour = pc + 4;
assign stallAll = (ICACHE_stall | DCACHE_stall | stall_MD);
assign inst_IF_next = (stall | stallAll) ? inst_IF : 
                      (instFlush) ? 32'd0 : 
                      ICACHE_rdata;
always@(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        pc <= 32'd0;
        pc_IF <= 32'd0;
        inst_IF <= 32'd0;
    end
    else begin
        pc <= (stall | stallAll) ? pc : pc_next;
        pc_IF <= (stall | stallAll) ? pc_IF : pcPlusFour;
        inst_IF <= inst_IF_next;
    end
end
always@(*) begin
    pc_br = {{14{inst_IF[15]}}, inst_IF[15:0], 2'd0} + pc_IF;
    case(pcDst)
        0: pc_next = pcPlusFour; 
        1: begin //branch
            pc_next = pc_br; 
        end
        2: pc_next = {4'd0, inst_IF[25:0], 2'd0}; //j jal
        3: pc_next = branch1; //jr jalr
        default: pc_next = pc;
    endcase
end
//ID
assign ReadData1 = (WrDst) ? pc_IF : reg_file_next[inst_IF[25:21]];
assign ReadData2 = (WrDst) ? 32'd0 : 
                   (ALUsrc) ? { {16{inst_IF[15]}}, inst_IF[15:0] } :
                   reg_file_next[inst_IF[20:16]];
assign  WBAddr   = (RegDst == 0) ? inst_IF[20:16] :
                   (RegDst == 1) ? inst_IF[15:11] :
                   5'd31;
assign compareEqual = &(branch1 ~^ branch2);
always@(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        for(i=0;i<32;i=i+1) begin
            reg_file[i] <= 32'd0;
        end
        ReadData1_ID <= 32'd0;
        ReadData2_ID <= 32'd0;
        WMemData_ID  <= 32'd0;
        rs_ID        <= 5'd0;
        rt_ID        <= 5'd0;
        WBAddr_ID    <= 5'd0;
        //control
        RegWrite_ID <= 1'd0;
        Mem2Reg_ID <= 1'd0;
        MemWrite_ID <= 1'd0;
        MemRead_ID <= 1'd0;
        ALUop_ID <= 4'd0;
        ALUsrc_ID <= 1'd0;
        shamt_ID <= 5'd0;
        WrDst_ID <= 1'd0;
        regHiLo_ID <= 1'd0;
        MulOrDiv_ID <= 1'd0;
        fromHiLo_ID <= 2'd0;
    end
    else begin
        for(i=0;i<32;i=i+1) begin
            reg_file[i] <= reg_file_next[i];
        end
        ReadData1_ID <= (stallAll)? ReadData1_ID : ReadData1;
        ReadData2_ID <= (stallAll)? ReadData2_ID : ReadData2;
        WMemData_ID  <= (stallAll)? WMemData_ID : reg_file_next[inst_IF[20:16]];
        rs_ID <= (stallAll)? rs_ID : inst_IF[25:21];
        rt_ID <= (stallAll)? rt_ID : inst_IF[20:16];
        shamt_ID <= (stallAll)? shamt_ID : inst_IF[10:6];
        WBAddr_ID <= (stallAll)? WBAddr_ID : WBAddr;
        //control
        RegWrite_ID <= (stallAll)? RegWrite_ID : RegWrite;
        Mem2Reg_ID <= (stallAll)? Mem2Reg_ID : Mem2Reg;
        MemWrite_ID <= (stallAll)? MemWrite_ID : MemWrite;
        MemRead_ID <= (stallAll)? MemRead_ID : MemRead;
        ALUop_ID <= (stallAll)? ALUop_ID : ALUop;
        ALUsrc_ID <= (stallAll)? ALUsrc_ID : ALUsrc;
        WrDst_ID <= (stallAll)? WrDst_ID : WrDst;
        regHiLo_ID <= (stallAll)? regHiLo_ID : regHiLo;
        MulOrDiv_ID <= (stallAll)? MulOrDiv_ID : MulOrDiv;
        fromHiLo_ID <= (stallAll)? fromHiLo_ID : fromHiLo;
    end
end
always@(*) begin
    for(i=0;i<32;i=i+1) begin
        reg_file_next[i] = reg_file[i];
    end
    if(RegWrite_MW) begin
        reg_file_next[WBAddr_MW] = WBData_MW;
    end
    // branch compare/////////////////////////////////
    case(ForwardA_br)
        0: branch1 = reg_file_next[inst_IF[25:21]];
        1: branch1 = ALUout_EX;
        2: branch1 = WBData_MW;
        default: branch1 = 32'd0;
    endcase
    case(ForwardB_br)
        0: branch2 = reg_file_next[inst_IF[20:16]];
        1: branch2 = ALUout_EX;
        2: branch2 = WBData_MW;
        default: branch2 = 32'd0;
    endcase
end

// control signal/////////////////////////////////
//EX
always@(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        ALUout_EX <= 32'd0;
        WMemData_EX <= 32'd0;
        WBAddr_EX <= 5'd0;
        RegWrite_EX <= 1'd0;
        Mem2Reg_EX <= 1'd0;
        MemWrite_EX <= 1'd0;
        MemRead_EX <= 1'd0;
        fromHiLo_EX <= 2'd0;
    end
    else begin
        ALUout_EX <= (stallAll)? ALUout_EX : ALUout;
        WMemData_EX <= (stallAll)? WMemData_EX : WMemData_EX_next;
        WBAddr_EX <= (stallAll)? WBAddr_EX : WBAddr_ID;
        RegWrite_EX <= (stallAll)? RegWrite_EX : RegWrite_ID;
        Mem2Reg_EX <= (stallAll)? Mem2Reg_EX : Mem2Reg_ID;
        MemWrite_EX <= (stallAll)? MemWrite_EX : MemWrite_ID;
        MemRead_EX <= (stallAll)? MemRead_EX : MemRead_ID;
        fromHiLo_EX <= (stallAll)? fromHiLo_EX : fromHiLo_ID;
    end
end
always@(*) begin
    case(ForwardA)
        0: ALU1 = ReadData1_ID;
        1: ALU1 = (WrDst_ID) ? ReadData1_ID : ALUout_EX;
        2: ALU1 = (WrDst_ID) ? ReadData1_ID : WBData_MW;
        default: ALU1 = 32'd0;
    endcase
    case(ForwardB)
        0: ALU2 = ReadData2_ID;
        1: ALU2 = (WrDst_ID || ALUsrc_ID) ? ReadData2_ID : ALUout_EX;
        2: ALU2 = (WrDst_ID  || ALUsrc_ID) ? ReadData2_ID : WBData_MW;
        default: ALU2 = 32'd0;
    endcase
    case(ForwardC)
        0: WMemData_EX_next = WMemData_ID;
        1: WMemData_EX_next = ALUout_EX;
        2: WMemData_EX_next = WBData_MW;
        default: WMemData_EX_next = 32'd0;
    endcase
    ///////ALU///////
    ALUout = 0;
    if(ALUop_ID == 4'b0000) ALUout = $signed(ALU1) + $signed(ALU2); //add addi lw sw j jal jr jalr beq bne
    if(ALUop_ID == 4'b0001 &&  ALUsrc_ID) ALUout = ALU1 & {16'd0, ALU2[15:0]}; //andi
    if(ALUop_ID == 4'b0001 && !ALUsrc_ID) ALUout = ALU1 & ALU2; //and
    if(ALUop_ID == 4'b0010 &&  ALUsrc_ID) ALUout = ALU1 | {16'd0, ALU2[15:0]}; //ori
    if(ALUop_ID == 4'b0010 && !ALUsrc_ID) ALUout = ALU1 | ALU2; //or
    if(ALUop_ID == 4'b0011 &&  ALUsrc_ID) ALUout = ALU1 ^ {16'd0, ALU2[15:0]}; //xori
    if(ALUop_ID == 4'b0011 && !ALUsrc_ID) ALUout = ALU1 ^ ALU2; //xor
    if(ALUop_ID == 4'b0100) ALUout = ($signed(ALU1)<$signed(ALU2)) ? 1 : 0; //slt
    if(ALUop_ID == 4'b0101) ALUout = $signed(ALU1) - $signed(ALU2); //sub
    if(ALUop_ID == 4'b0110) ALUout = ~(ALU1 | ALU2); //nor
    if(ALUop_ID == 4'b0111) ALUout = ALU2 << shamt_ID; //sll
    if(ALUop_ID == 4'b1000) ALUout = $signed(ALU2) >>> shamt_ID; //sra
    if(ALUop_ID == 4'b1001) ALUout = ALU2 >> shamt_ID; //srl
    /////////////////
end
//MW
assign DCACHE_ren = MemRead_EX;
assign DCACHE_wen = MemWrite_EX;
assign DCACHE_addr = ALUout_EX[31:2];
assign DCACHE_wdata = WMemData_EX;
assign DMemRData_next = DCACHE_rdata;
assign WBData_MW = (Mem2Reg_MW) ? DMemRData : ALUout_MW;
always@(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        DMemRData <= 32'd0;
        ALUout_MW <= 32'd0;
        WBAddr_MW <= 5'd0;
        RegWrite_MW <= 1'd0;
        Mem2Reg_MW <= 1'd0;
    end
    else begin
        DMemRData <= (stallAll) ? DMemRData : DMemRData_next;
        ALUout_MW <= (stallAll) ? ALUout_MW : ALUout_MW_next;
        WBAddr_MW <= (stallAll) ? WBAddr_MW : WBAddr_EX;
        RegWrite_MW <= (stallAll) ? RegWrite_MW : RegWrite_EX;
        Mem2Reg_MW <= (stallAll)? Mem2Reg_MW : Mem2Reg_EX;
    end
end
always@(*) begin
    case(fromHiLo_EX)
        0: ALUout_MW_next = ALUout_EX;
        1: ALUout_MW_next = HILO[31:0];
        2: ALUout_MW_next = HILO[63:32];
        default: ALUout_MW_next = ALUout_MW;
    endcase
end

// ////ctrl unit
wire beq; //stall
wire bne; //stall
wire j;
wire jal;
wire jr; //stall
wire jalr; //stall
wire lw;
wire sw;
wire mflo;
wire mfhi;


reg [3:0] a;
reg [3:0] b;
// ================= combinational =========================
//assign IMemNoread  = ((beq && compareEqual) | (bne && !compareEqual) | jr | jalr | j | jal);
assign regHiLo = (a[3:1] == 3'b110);
assign MulOrDiv = a[0];
assign mflo = (a == 4'b1110);
assign mfhi = (a == 4'b1111);
assign fromHiLo = (mfhi) ? 2 :
                  (mflo) ? 1 :
                  0;
assign maystall = (beq | bne | jr | jalr);
assign beq = (inst_IF[31:26] == 6'd4);
assign bne = (inst_IF[31:26] == 6'd5);
assign j = (inst_IF[31:26] == 6'd2);
assign jal = (inst_IF[31:26] == 6'd3);
assign jr = (inst_IF[31:26] == 6'd0) && (inst_IF[5:0] == 6'd8);
assign jalr = (inst_IF[31:26] == 6'd0) && (inst_IF[5:0] == 6'd9);
assign lw = (inst_IF[31:26] == 6'd35);
assign sw = (inst_IF[31:26] == 6'd43);
always @(*) begin
    case (inst_IF[5:0])
        6'b100000: b = 4'b0000;
        6'b100010: b = 4'b0101;
        6'b100100: b = 4'b0001;
        6'b100101: b = 4'b0010;
        6'b100110: b = 4'b0011;
        6'b100111: b = 4'b0110;
        6'b000000: b = 4'b0111;
        6'b000011: b = 4'b1000;
        6'b000010: b = 4'b1001;
        6'b101010: b = 4'b0100;
        6'b001001: b = 4'b0000;
        6'b011000: b = 4'b1100; //mult
        6'b011010: b = 4'b1101; //div
        6'b010010: b = 4'b1110; //mflo
        6'b010000: b = 4'b1111; //mfhi
        default: b = 4'b0000;
    endcase
    case (inst_IF[31:26])
        6'b001000: a = 4'b0000;
        6'b001100: a = 4'b0001;
        6'b001101: a = 4'b0010;
        6'b001110: a = 4'b0011;
        6'b001010: a = 4'b0100;
        6'b000000: a = b;
        default: a = 4'b0000;
    endcase
end

// ==================== ctrl ==================================
//ID
assign pcDst_o = ( jr||jalr )? 2'd3:
                ( j||jal )? 2'd2:
                ( (compareEqual && beq) || (!compareEqual && bne) )? 2'd1:2'd0;
assign WrDst_o = jal||jalr;
assign RegDst_o = (jal)? 2'd2:
                (inst_IF[31:26] == 6'd0)? 2'd1: 2'd0;
assign ALUsrc_o = !(inst_IF[29:27] == 3'd0);
//EX
assign ALUop_o = (inst_IF[31:26] != 0)? a : b;
//MEM
assign MemWrite_o = sw;
assign MemRead_o = lw;
//WB
assign RegWrite_o = !(beq||bne||sw||j||jr||regHiLo);
assign Mem2Reg_o = lw;
// //////////////////////////////////////////////////////////
////stall unit
wire   checkstall;
assign checkstall = ((maystall && RegWrite_ID) || MemRead_ID);
assign stall = checkstall && ((WBAddr_ID == inst_IF[25:21])||(WBAddr_ID == inst_IF[20:16]));
assign instFlush = (pcDst_o)? 1'b1: 1'b0;
//////////////////////////////////////////////////////////
////forward unit
assign ForwardA = (RegWrite_EX && WBAddr_EX && (WBAddr_EX == rs_ID))? 2'b01:
                    (RegWrite_MW && WBAddr_MW && (WBAddr_MW == rs_ID))? 2'b10: 2'b00;

assign ForwardB = (RegWrite_EX && WBAddr_EX && (WBAddr_EX == rt_ID))? 2'b01:
                    (RegWrite_MW && WBAddr_MW && (WBAddr_MW == rt_ID))? 2'b10: 2'b00;

assign ForwardC = ForwardB;

assign ForwardA_br = (RegWrite_EX && WBAddr_EX && (WBAddr_EX == inst_IF[25:21]))? 2'b01:
                    (RegWrite_MW && WBAddr_MW && (WBAddr_MW == inst_IF[25:21]))? 2'b10: 2'b00;

assign ForwardB_br = (RegWrite_EX && WBAddr_EX && (WBAddr_EX == inst_IF[20:16]))? 2'b01:
                    (RegWrite_MW && WBAddr_MW && (WBAddr_MW == inst_IF[20:16]))? 2'b10: 2'b00;



endmodule
