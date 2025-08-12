module datapath(

    input wire clk,
    input wire reset,
    output wire [31:0] alu_out,
    output wire [31:0] pc_out,
    output wire [31:0] instruction,
    output wire [31:0] datawrite_mux_out
);
    // --------------------------------------------------
    // 1) PC Wires
    // --------------------------------------------------
       wire [31:0] pc_mux_out;
       //wire [31:0] pc_out;  // holds current PC from program_counter
       wire [31:0] pc4; 
       //wire [31:0] instruction; 
       wire  [31:0] data1;
       wire  [31:0] data2; 
       wire [31:0] imm_val;
       wire [31:0] mux1_out;
      // wire [31:0]alu_out;
       wire [31:0] dmem_out;
       //wire [31:0] datawrite_mux_out;
       wire [31:0]jmux_out;
       
       
       wire [2:0] ImmSel;
       wire regwen;
       wire bsel;
       wire [3:0] ALUSel;
       wire MEMRW;
       wire  [2:0]  Func3;
       wire [1:0] WBSel;
       wire PCSel;
       wire ASel;
       wire BrEq;
       wire BrLT;
       wire BrUn;

      
       

       program_counter pc (
           .clk    (clk),         // system clock
           .reset  (reset),       // synchronous reset
           .pc_in  (pc_mux_out),    // next PC (PC + 4)
           .pc_out (pc_out)  // current PC value
       );
   

       pcplus4 pc_4 (
           .pc_out  (pc_out), // current PC in
           .PCPlus4 (pc4)     // next PC out (pc_out_wire + 4)
       );
    
    
    imem instr(
        .addr(pc_out),
        .dataR(instruction)
    );
    
    
    regfile RegFile(
        .dataW(datawrite_mux_out),
        .clk(clk),
        .reset(reset),
        .RegWEn(regwen),
        .rsW(instruction[11:7]),
        .rs1(instruction[19:15]),
        .rs2(instruction[24:20]),
        .data1(data1),
        .data2(data2)
            );
            
       Imm_Gen IMM (
                .instruction   (instruction),
                .ImmSel  (ImmSel),
                .ImmExt (imm_val)
            );
            
        mux_for_imm M1(
                    .d2         (data2),
                    .imm        (imm_val),
                    .Bsel       (bsel),
                    .imm_mux_out(mux1_out)
                );
                
         alu ALU(
         .alu_in1(jmux_out),
         .alu_in2(mux1_out),
         .alusel(ALUSel),
         .alu_out(alu_out)
         );
    
            data_memory dmem (
                     .clk(clk),
                     .MemRW(MEMRW),         // Memory read/write control
                     .funct3(Func3),        // should be [2:0], from instruction[14:12]
                     .addr(alu_out),        // effective memory address from ALU
                     .WD(data2),            // data to write (rs2 value)
                     .RD(dmem_out)          // output data (read result)
         );

            
            mux_for_datawrite MuxWrite(
            
                  .WBSel(WBSel),
                  .dmem (dmem_out),
                  .alu (alu_out),
                  .PCPlus4(pc4),
                  .dataW(datawrite_mux_out)
            );
            
            
            
            control_unit CU (
                .opcode  (instruction[6:0]),
                .funct3  (instruction[14:12]),
                .funct7  (instruction[31:25]),
                .regwen  (regwen),
                .bsel    (bsel),
                .asel    (ASel),
                .pcsel   (PCSel),
                .ALUSel  (ALUSel),
                .MEMRW   (MEMRW),
                .ImmSel  (ImmSel),
                .WBSel   (WBSel),
                .Func3   (Func3),
                .BrEq    (BrEq),
                .BrLT    (BrLT),
                .BrUn    (BrUn)
            );

            
            muxx_for_pc PC(
                            .PCPlus4(pc4),
                            .alu_out(alu_out),
                            .PCSel(PCSel),
                            .pc(pc_mux_out)                  
                       );
           Mux_for_jump MPC(
                       .in1(data1),
                       .in2(pc_out),
                       .ASel(ASel),
                       .out(jmux_out)
           );

            branch_comparator BCMP (
                .rs1  (data1),
                .rs2  (data2),
                .BrUn (BrUn),
                .BrEq (BrEq),
                .BrLT (BrLT)
            );

            
            
    endmodule
    
    
    
