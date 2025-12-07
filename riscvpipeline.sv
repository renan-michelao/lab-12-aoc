module riscvpipeline (
    input         clk,
    input         reset,
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

   /********************** F: Instruction Fetch *********************************/
   localparam NOP = 32'b0000000_00000_00000_000_00000_0110011;
   reg [31:0] F_PC;
   reg [31:0] FD_PC;
   reg [31:0] FD_instr;
   reg        FD_nop;
   assign PC = F_PC;

   wire [31:0] jumpOrBranchAddress;
   wire        jumpOrBranch;
   wire        stall;
   wire        flush;

   always @(posedge clk) begin
      if (reset) begin
         F_PC <= 0;
         FD_nop <= 1'b1;
         FD_instr <= NOP;
      end else begin
         if (!stall) begin
            FD_instr <= Instr;
            FD_PC    <= F_PC;
            if (jumpOrBranch)
               F_PC <= jumpOrBranchAddress;
            else
               F_PC <= F_PC + 4;
         end
         
         // Se houver flush, invalidamos o que acabou de ser buscado (próximo ciclo vira NOP no Decode)
         if (flush) begin
            FD_nop <= 1'b1;
         end else if (!stall) begin
            FD_nop <= 1'b0;
         end
         // Se stall, FD_nop mantém o estado
      end
   end

   /************************ D: Instruction Decode *******************************/
   reg [31:0] DE_PC;
   reg [31:0] DE_instr;
   reg [31:0] DE_rs1;
   reg [31:0] DE_rs2;

   reg [31:0] EM_instr; // Forward declaration para visibilidade
   reg [31:0] MW_instr; // Forward declaration
   reg [31:0] EM_Eresult;

   wire        writeBackEn;
   wire [31:0] writeBackData;
   wire [4:0]  wbRdId;

   wire [31:0] forward_rs1;
   wire [31:0] forward_rs2;
   wire [1:0]  forwardA, forwardB;

   reg [31:0] RegisterBank [0:31];

   // Lógica de Forwarding (Mem->Ex e WB->Ex)
   assign forwardA = ((rs1Id(DE_instr) != 0) && (rs1Id(DE_instr) == rdId(EM_instr)) && writesRd(EM_instr)) ? 2'b10 :
                     ((rs1Id(DE_instr) != 0) && (rs1Id(DE_instr) == rdId(MW_instr)) && writesRd(MW_instr)) ? 2'b01 : 2'b00;
   
   assign forwardB = ((rs2Id(DE_instr) != 0) && (rs2Id(DE_instr) == rdId(EM_instr)) && writesRd(EM_instr)) ? 2'b10 :
                     ((rs2Id(DE_instr) != 0) && (rs2Id(DE_instr) == rdId(MW_instr)) && writesRd(MW_instr)) ? 2'b01 : 2'b00;

   assign forward_rs1 = (forwardA == 2'b10) ? EM_Eresult :
                        (forwardA == 2'b01) ? writeBackData : DE_rs1;
   
   assign forward_rs2 = (forwardB == 2'b10) ? EM_Eresult :
                        (forwardB == 2'b01) ? writeBackData : DE_rs2;

   always @(posedge clk) begin
      // CORREÇÃO STALL:
      // Se não houver stall, o pipeline flui normal.
      // Se houver stall, inserimos uma BOLHA (NOP) no estágio Execute (DE_instr)
      // para que a instrução de Load possa completar no próximo ciclo sem conflito.
      if (!stall) begin
         DE_PC    <= FD_PC;
         DE_instr <= (FD_nop || flush) ? NOP : FD_instr;
      end else begin
         // Inserção da Bolha durante o Stall
         DE_instr <= NOP;
         // DE_PC mantém o valor ou recebe lixo, não importa pois é NOP
      end
      
      // Leitura do Register Bank com Bypass Interno (WB -> ID no mesmo ciclo)
      DE_rs1 <= (rs1Id(FD_instr) != 0 && writeBackEn && (rs1Id(FD_instr) == wbRdId)) ? 
                writeBackData : 
                (rs1Id(FD_instr) ? RegisterBank[rs1Id(FD_instr)] : 32'b0);
                
      DE_rs2 <= (rs2Id(FD_instr) != 0 && writeBackEn && (rs2Id(FD_instr) == wbRdId)) ? 
                writeBackData : 
                (rs2Id(FD_instr) ? RegisterBank[rs2Id(FD_instr)] : 32'b0);
                
      if (writeBackEn) // WB independe de stall do front-end
         RegisterBank[wbRdId] <= writeBackData;
   end

   /************************ E: Execute *****************************************/
   reg [31:0] EM_PC;
   // EM_instr declarado acima
   reg [31:0] EM_rs2;
   // EM_Eresult declarado acima
   reg [31:0] EM_addr;
   
   wire [31:0] E_aluIn1 = forward_rs1;
   wire [31:0] E_aluIn2 = (isALUreg(DE_instr) | isBranch(DE_instr)) ? forward_rs2 : Iimm(DE_instr);
   wire [4:0]  E_shamt  = isALUreg(DE_instr) ? forward_rs2[4:0] : shamt(DE_instr);
   wire E_minus = DE_instr[30] & isALUreg(DE_instr);
   wire E_arith_shift = DE_instr[30];

   wire [31:0] E_aluPlus = E_aluIn1 + E_aluIn2;
   wire [32:0] E_aluMinus = {1'b1, ~E_aluIn2} + {1'b0,E_aluIn1} + 33'b1;
   wire        E_LT  = (E_aluIn1[31] ^ E_aluIn2[31]) ? E_aluIn1[31] : E_aluMinus[32];
   wire        E_LTU = E_aluMinus[32];
   wire        E_EQ  = (E_aluMinus[31:0] == 0);

   function [31:0] flip32;
      input [31:0] x;
      flip32 = {x[ 0], x[ 1], x[ 2], x[ 3], x[ 4], x[ 5], x[ 6], x[ 7],
            x[ 8], x[ 9], x[10], x[11], x[12], x[13], x[14], x[15],
            x[16], x[17], x[18], x[19], x[20], x[21], x[22], x[23],
            x[24], x[25], x[26], x[27], x[28], x[29], x[30], x[31]};
   endfunction

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

   wire E_JumpOrBranch = (isJAL(DE_instr) || isJALR(DE_instr) || (isBranch(DE_instr) && E_takeBranch));
   wire [31:0] E_JumpOrBranchAddr = isBranch(DE_instr) ? DE_PC + Bimm(DE_instr) :
                                    isJAL(DE_instr)    ? DE_PC + Jimm(DE_instr) :
                                    {E_aluPlus[31:1],1'b0} ;

   wire [31:0] E_result = (isJAL(DE_instr) | isJALR(DE_instr)) ? DE_PC+4 :
                          isLUI(DE_instr) ? Uimm(DE_instr) :
                          isAUIPC(DE_instr) ? DE_PC + Uimm(DE_instr) : E_aluOut;

   always @(posedge clk) begin
      // Execução e Memória continuam mesmo durante Stall (para liberar o Load)
      // Se flush (branch taken), anulamos a instrução atual
      if (flush || reset) begin
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
   // MW_instr declarado acima
   reg [31:0] MW_Eresult;
   reg [31:0] MW_Mdata;
   reg [31:0] MW_addr;

   wire [2:0] M_funct3 = funct3(EM_instr);
   assign halt = !reset & isEBREAK(MW_instr);

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

   assign jumpOrBranchAddress = E_JumpOrBranchAddr;
   assign jumpOrBranch        = E_JumpOrBranch;

   /*************** Hazard Detection Unit CORRIGIDO ****************/
   
   // Load-Use Hazard:
   // Condição: Instrução no estágio EXECUTE (DE_instr) é um Load
   // E a instrução no estágio DECODE (FD_instr) precisa desse valor.
   wire load_use_hazard = (isLoad(DE_instr) && 
                          ((readsRs1(FD_instr) && (rs1Id(FD_instr) == rdId(DE_instr))) ||
                           (readsRs2(FD_instr) && (rs2Id(FD_instr) == rdId(DE_instr))))
                          && (rdId(DE_instr) != 0)); // Otimização: não stall se dest for x0
   
   assign stall = load_use_hazard;
   assign flush = E_JumpOrBranch;

/******************************************************************************/

   always @(posedge clk) begin
      if (halt) begin
         $writememh("regs.out", RegisterBank);
         $finish();
      end
   end
endmodule