module riscvpipeline (
    input         clk,
    input         reset,
    input         turbo,     // Interruptor do Modo Turbo
    output [31:0] PC,
    input  [31:0] Instr,
    output [31:0] Address,  
    output [31:0] WriteData, 
    output        MemWrite,  
    input  [31:0] ReadData);

   // --- Funções Auxiliares ---
   function [6:0] opcode; input [31:0] i; opcode = i[6:0]; endfunction
   function [4:0] rdId;   input [31:0] i; rdId   = i[11:7]; endfunction
   function [4:0] rs1Id;  input [31:0] i; rs1Id  = i[19:15]; endfunction
   function [4:0] rs2Id;  input [31:0] i; rs2Id  = i[24:20]; endfunction
   function [2:0] funct3; input [31:0] i; funct3 = i[14:12]; endfunction
   function [6:0] funct7; input [31:0] i; funct7 = i[31:25]; endfunction
   function [4:0] shamt;  input [31:0] i; shamt  = i[24:20]; endfunction

   function isALUreg; input [31:0] i; isALUreg = (opcode(i) == 7'b0110011); endfunction
   function isALUimm; input [31:0] i; isALUimm = (opcode(i) == 7'b0010011); endfunction
   function isBranch; input [31:0] i; isBranch = (opcode(i) == 7'b1100011); endfunction
   function isJALR;   input [31:0] i; isJALR   = (opcode(i) == 7'b1100111); endfunction
   function isJAL;    input [31:0] i; isJAL    = (opcode(i) == 7'b1101111); endfunction
   function isLoad;   input [31:0] i; isLoad   = (opcode(i) == 7'b0000011); endfunction
   function isStore;  input [31:0] i; isStore  = (opcode(i) == 7'b0100011); endfunction
   function isLUI;    input [31:0] i; isLUI    = (opcode(i) == 7'b0110111); endfunction
   function isAUIPC;  input [31:0] i; isAUIPC  = (opcode(i) == 7'b0010111); endfunction
   function isEBREAK; input [31:0] i; isEBREAK = (i == 32'b00000000000100000000000001110011); endfunction

   function [31:0] Iimm; input [31:0] i; Iimm = {{20{i[31]}}, i[31:20]}; endfunction
   function [31:0] Simm; input [31:0] i; Simm = {{20{i[31]}}, i[31:25], i[11:7]}; endfunction
   function [31:0] Bimm; input [31:0] i; Bimm = {{19{i[31]}}, i[31], i[7], i[30:25], i[11:8], 1'b0}; endfunction
   function [31:0] Uimm; input [31:0] i; Uimm = {i[31:12], 12'b0}; endfunction
   function [31:0] Jimm; input [31:0] i; Jimm = {{11{i[31]}}, i[31], i[19:12], i[20], i[30:21], 1'b0}; endfunction

   function writesRd; input [31:0] i;
      writesRd = isALUreg(i) | isALUimm(i) | isLUI(i) | isAUIPC(i) | isLoad(i) | isJAL(i) | isJALR(i);
   endfunction
   function readsRs1; input [31:0] i;
      readsRs1 = isALUreg(i) | isALUimm(i) | isBranch(i) | isJALR(i) | isLoad(i) | isStore(i);
   endfunction
   function readsRs2; input [31:0] i;
      readsRs2 = isALUreg(i) | isBranch(i) | isStore(i);
   endfunction

   // ========================================================================
   // ESTRUTURAS DO BTB e BHT
   // ========================================================================
   reg [1:0] bht [0:63]; 
   reg [31:0] btb_target [0:63];
   reg [31:0] btb_tag [0:63];
   integer k; 
   
   wire        update_enable;
   wire [31:0] update_pc;
   wire [31:0] update_target;
   wire        update_taken;

   // ========================================================================
   /********************** F: Instruction Fetch *********************************/
   localparam NOP = 32'b0000000_00000_00000_000_00000_0110011;
   reg [31:0] F_PC;
   reg [31:0] FD_PC;
   reg [31:0] FD_instr;
   reg        FD_nop;
   reg        FD_predictedTaken; 
   assign PC = F_PC;

   wire        stall;
   wire        flush_ex; 

   // Lógica de Predição
   wire [5:0]  f_index = F_PC[7:2];
   wire [1:0]  f_bht_state = bht[f_index];
   wire [31:0] f_btb_target = btb_target[f_index];
   wire [31:0] f_btb_tag    = btb_tag[f_index];
   
   // SÓ PREDIZ SE O TURBO ESTIVER LIGADO
   wire f_predictTaken = turbo && (f_bht_state[1] == 1'b1) && (f_btb_tag == F_PC);

   wire [31:0] correct_pc_from_ex;

   // BLOCO 1: Pipeline Fetch e Lógica de PC
   always @(posedge clk) begin
      if (reset) begin
         F_PC <= 0;
         FD_nop <= 1'b1;
         FD_instr <= NOP;
         FD_predictedTaken <= 1'b0;
         // NOTA: A limpeza do BHT foi removida daqui para evitar conflito!
      end else begin
         if (!stall) begin
            FD_instr <= Instr;
            FD_PC    <= F_PC;
            FD_predictedTaken <= f_predictTaken;
            
            if (flush_ex) begin
               F_PC <= correct_pc_from_ex; 
            end else if (f_predictTaken) begin
               F_PC <= f_btb_target;
            end else begin
               F_PC <= F_PC + 4;
            end
         end
         
         if (flush_ex) begin
            FD_nop <= 1'b1; 
            FD_predictedTaken <= 1'b0;
         end else if (!stall) begin
            FD_nop <= 1'b0;
         end
      end
   end

   // BLOCO 2: Gerenciamento do BHT/BTB (Reset E Update juntos aqui)
   always @(posedge clk) begin
      if (reset) begin
          // Limpeza das tabelas quando aperta Reset
          for(k=0; k<64; k=k+1) begin
             bht[k] <= 2'b10;
             btb_target[k] <= 0;
             btb_tag[k] <= 32'hFFFFFFFF;
          end
      end else if (update_enable) begin
          // Atualização das tabelas durante execução
          if (update_taken) begin
             if (bht[update_pc[7:2]] != 2'b11) bht[update_pc[7:2]] <= bht[update_pc[7:2]] + 1;
          end else begin
             if (bht[update_pc[7:2]] != 2'b00) bht[update_pc[7:2]] <= bht[update_pc[7:2]] - 1;
          end
          
          if (update_taken) begin
             btb_target[update_pc[7:2]] <= update_target;
             btb_tag[update_pc[7:2]]    <= update_pc; 
          end
      end
   end

   /************************ D: Instruction Decode *******************************/
   reg [31:0] DE_PC;
   reg [31:0] DE_instr;
   reg [31:0] DE_rs1;
   reg [31:0] DE_rs2;
   reg        DE_predictedTaken; 

   reg [31:0] EM_instr; 
   reg [31:0] MW_instr; 
   reg [31:0] EM_Eresult;
   wire        writeBackEn;
   wire [31:0] writeBackData;
   wire [4:0]  wbRdId;

   wire [31:0] forward_rs1;
   wire [31:0] forward_rs2;
   wire [1:0]  forwardA, forwardB;

   reg [31:0] RegisterBank [0:31];

   assign forwardA = ((rs1Id(DE_instr) != 0) && (rs1Id(DE_instr) == rdId(EM_instr)) && writesRd(EM_instr)) ? 2'b10 :
                     ((rs1Id(DE_instr) != 0) && (rs1Id(DE_instr) == rdId(MW_instr)) && writesRd(MW_instr)) ? 2'b01 : 2'b00;
   
   assign forwardB = ((rs2Id(DE_instr) != 0) && (rs2Id(DE_instr) == rdId(EM_instr)) && writesRd(EM_instr)) ? 2'b10 :
                     ((rs2Id(DE_instr) != 0) && (rs2Id(DE_instr) == rdId(MW_instr)) && writesRd(MW_instr)) ? 2'b01 : 2'b00;

   assign forward_rs1 = (forwardA == 2'b10) ? EM_Eresult : (forwardA == 2'b01) ? writeBackData : DE_rs1;
   assign forward_rs2 = (forwardB == 2'b10) ? EM_Eresult : (forwardB == 2'b01) ? writeBackData : DE_rs2;

   always @(posedge clk) begin
      if (!stall) begin
         DE_PC    <= FD_PC;
         DE_predictedTaken <= FD_predictedTaken; 
         DE_instr <= (FD_nop || flush_ex) ? NOP : FD_instr;
      end else begin
         DE_instr <= NOP;
      end
      
      DE_rs1 <= (rs1Id(FD_instr) != 0 && writeBackEn && (rs1Id(FD_instr) == wbRdId)) ? writeBackData : (rs1Id(FD_instr) ? RegisterBank[rs1Id(FD_instr)] : 32'b0);
      DE_rs2 <= (rs2Id(FD_instr) != 0 && writeBackEn && (rs2Id(FD_instr) == wbRdId)) ? writeBackData : (rs2Id(FD_instr) ? RegisterBank[rs2Id(FD_instr)] : 32'b0);
                
      if (writeBackEn) 
         RegisterBank[wbRdId] <= writeBackData;
   end

   /************************ E: Execute *****************************************/
   reg [31:0] EM_PC;
   reg [31:0] EM_rs2;
   reg [31:0] EM_addr;
   
   wire [31:0] E_aluIn1 = forward_rs1;
   wire [31:0] E_aluIn2 = (isALUreg(DE_instr) | isBranch(DE_instr)) ? forward_rs2 : Iimm(DE_instr);
   wire E_minus = DE_instr[30] & isALUreg(DE_instr);
   wire E_arith_shift = DE_instr[30];

   wire [31:0] E_aluPlus = E_aluIn1 + E_aluIn2;
   wire [32:0] E_aluMinus = {1'b1, ~E_aluIn2} + {1'b0,E_aluIn1} + 33'b1;
   wire        E_LT  = (E_aluIn1[31] ^ E_aluIn2[31]) ? E_aluIn1[31] : E_aluMinus[32];
   wire        E_LTU = E_aluMinus[32];
   wire        E_EQ  = (E_aluMinus[31:0] == 0);

   function [31:0] flip32; input [31:0] x; flip32 = {x[0], x[1], x[2], x[3], x[4], x[5], x[6], x[7], x[8], x[9], x[10], x[11], x[12], x[13], x[14], x[15], x[16], x[17], x[18], x[19], x[20], x[21], x[22], x[23], x[24], x[25], x[26], x[27], x[28], x[29], x[30], x[31]}; endfunction

   wire [31:0] E_shifter_in = funct3(DE_instr) == 3'b001 ? flip32(E_aluIn1) : E_aluIn1;
   wire [31:0] E_shifter = $signed({E_arith_shift & E_aluIn1[31], E_shifter_in}) >>> E_aluIn2[4:0];
   wire [31:0] E_leftshift = flip32(E_shifter);

   reg [31:0] E_aluOut;
   always @(*) begin
      case(funct3(DE_instr))
         3'b000: E_aluOut = E_minus ? E_aluMinus[31:0] : E_aluPlus;
         3'b001: E_aluOut = E_leftshift;
         3'b010: E_aluOut = {31'b0, E_LT};
         3'b011: E_aluOut = {31'b0, E_LTU};
         3'b100: E_aluOut = E_aluIn1 ^ E_aluIn2;
         3'b101: E_aluOut = E_shifter;
         3'b110: E_aluOut = E_aluIn1 | E_aluIn2;
         3'b111: E_aluOut = E_aluIn1 & E_aluIn2;
      endcase
   end

   reg E_takeBranch;
   always @(*) begin
      case (funct3(DE_instr))
         3'b000: E_takeBranch = E_EQ;
         3'b001: E_takeBranch = !E_EQ;
         3'b100: E_takeBranch = E_LT;
         3'b101: E_takeBranch = !E_LT;
         3'b110: E_takeBranch = E_LTU;
         3'b111: E_takeBranch = !E_LTU;
         default: E_takeBranch = 1'b0;
      endcase
   end

   wire E_ActualTaken = (isBranch(DE_instr) && E_takeBranch);
   wire [31:0] E_TargetAddr = isBranch(DE_instr) ? DE_PC + Bimm(DE_instr) : 
                              isJAL(DE_instr) ? DE_PC + Jimm(DE_instr) : {E_aluPlus[31:1],1'b0};

   wire prediction_was_wrong = (E_ActualTaken != DE_predictedTaken);
   wire unconditional_jump = isJAL(DE_instr) || isJALR(DE_instr);

   assign flush_ex = (isBranch(DE_instr) && prediction_was_wrong) || unconditional_jump;
   assign correct_pc_from_ex = (E_ActualTaken || unconditional_jump) ? E_TargetAddr : (DE_PC + 4);

   // SINAIS PARA ATUALIZAÇÃO SÓ SE TURBO ESTIVER LIGADO
   assign update_enable = (isBranch(DE_instr) && !stall);
   assign update_pc     = DE_PC;
   assign update_target = E_TargetAddr;
   assign update_taken  = E_ActualTaken;

   wire [31:0] E_result = (isJAL(DE_instr) | isJALR(DE_instr)) ? DE_PC+4 :
                          isLUI(DE_instr) ? Uimm(DE_instr) :
                          isAUIPC(DE_instr) ? DE_PC + Uimm(DE_instr) : E_aluOut;
   
   always @(posedge clk) begin
      if (flush_ex || reset) begin
         EM_instr   <= NOP;
         EM_PC      <= 0;
         EM_Eresult <= 0;
         EM_addr    <= 0;
         EM_rs2     <= 0;
      end else begin
         EM_PC      <= DE_PC;
         EM_instr   <= DE_instr;
         EM_rs2     <= forward_rs2; 
         EM_Eresult <= E_result;
         EM_addr    <= isStore(DE_instr) ? forward_rs1 + Simm(DE_instr) : forward_rs1 + Iimm(DE_instr);
      end
   end

/************************ M: Memory *******************************************/
   reg [31:0] MW_PC;
   reg [31:0] MW_Eresult;
   reg [31:0] MW_Mdata;
   reg [31:0] MW_addr;

   wire [2:0] M_funct3 = funct3(EM_instr);
   wire halt = !reset & isEBREAK(MW_instr); // Definido como wire para evitar aviso

   assign Address  = EM_addr;
   assign MemWrite = isStore(EM_instr);
   assign WriteData = EM_rs2;
   always @(posedge clk) begin
      MW_PC        <= EM_PC;
      MW_instr     <= EM_instr;
      MW_Eresult   <= EM_Eresult;
      MW_Mdata     <= ReadData;
      MW_addr      <= EM_addr;
   end

/************************ W: WriteBack ****************************************/
   wire [2:0] W_funct3 = funct3(MW_instr);
   assign writeBackData = isLoad(MW_instr) ? MW_Mdata : MW_Eresult;
   assign writeBackEn = writesRd(MW_instr) && rdId(MW_instr) != 0;
   assign wbRdId = rdId(MW_instr);

   wire load_use_hazard = (isLoad(DE_instr) && 
                          ((readsRs1(FD_instr) && (rs1Id(FD_instr) == rdId(DE_instr))) ||
                           (readsRs2(FD_instr) && (rs2Id(FD_instr) == rdId(DE_instr))))
                          && (rdId(DE_instr) != 0));
   
   assign stall = load_use_hazard;
   assign flush = flush_ex; 

   always @(posedge clk) begin
      if (halt) begin
         $writememh("regs.out", RegisterBank);
         $finish();
      end
   end
endmodule
