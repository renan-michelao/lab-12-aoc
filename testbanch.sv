module testbench();
    logic         clk, reset, memwrite;
    logic [31:0]  pc, instr;
    logic [31:0]  writedata, addr, readdata;
    
    // Contador de ciclos
    integer ciclos;
    
    // Instancia o processador
    riscvpipeline cpu(clk, reset, pc, instr, addr, writedata, memwrite, readdata);

    // Memória de Instruções (Carrega o arquivo text.hex)
    mem #("text.hex") instr_mem(.clk(clk), .a(pc), .rd(instr));

    // Memória de Dados (Carrega o arquivo data.hex)
    mem #("data.hex") data_mem(clk, memwrite, addr, writedata, readdata);

    // Inicialização
    initial begin
      $dumpfile("dump.vcd"); 
      $dumpvars(0);
      
      ciclos = 0;
      reset <= 1; 
      #15 reset <= 0; // Solta o reset
      
      // Espera o programa terminar (timeout de segurança de 5000 ciclos)
      // O processador deve parar sozinho quando encontrar um EBREAK
    end

    // Gerador de Clock e Contador
    always begin
      clk <= 1; #5; 
      clk <= 0; #5; 
      
      // Conta ciclos apenas se não estiver em reset
      if (!reset) begin
          ciclos = ciclos + 1;
          
          // Debug opcional: Mostra o que está acontecendo a cada ciclo
          // $display("Ciclo %d: PC=%h Instr=%h", ciclos, pc, instr);
      end
      
      // Segurança: Se demorar demais, para.
      if (ciclos > 5000) begin
          $display("ERRO: Simulação demorou demais! Loop infinito?");
          $finish;
      end
    end
    
    // Monitora o sinal de halt (parada) do processador
    // O seu processador tem um wire 'halt' interno? 
    // Se não tiver saída no módulo, monitoramos a instrução EBREAK (0x00100073)
    always @(posedge clk) begin
        if (instr == 32'h00100073) begin // EBREAK
            $display("--------------------------------------------------");
            $display("FIM DO PROGRAMA (EBREAK encontrado)");
            $display("Total de Ciclos de Clock: %d", ciclos);
            $display("--------------------------------------------------");
            $writememh("regs.out", cpu.RegisterBank); // Salva os registradores
            $finish;
        end
    end

endmodule