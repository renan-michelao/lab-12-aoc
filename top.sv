module top (
    input        CLOCK_50, // Clock de 50MHz da placa
    input  [9:0] SW,       // Switches (SW[0] será o TURBO)
    input  [3:0] KEY,      // Botões (KEY[0] será o Reset)
    output [6:0] HEX0, HEX1, HEX2, HEX3, HEX4, HEX5, // Displays
    output [9:0] LEDR      // LEDs vermelhos
);

    wire clk = CLOCK_50;
    wire reset = ~KEY[0]; 

    // Sinais do Processador
    wire [31:0] pc, instr, readdata, writedata, dataadr;
    wire memwrite;

    // --- Instancia o Processador ---
    // Agora ligamos o SW[0] na entrada 'turbo'
    riscvpipeline cpu(
        .clk(clk),
        .reset(reset),
        .turbo(SW[0]),  // <--- LIGAÇÃO DO MODO TURBO
        .PC(pc),
        .Instr(instr),
        .MemWrite(memwrite),
        .Address(dataadr),
        .WriteData(writedata),
        .ReadData(readdata)
    );

    // Memória de Instruções
    mem #("riscv.hex") imem (
        .clk(clk),
        .we(1'b0),     
        .a(pc),
        .wd(32'b0),
        .rd(instr)
    );

    // Memória de Dados
    mem #("data.hex") dmem (
        .clk(clk),
        .we(memwrite), 
        .a(dataadr),
        .wd(writedata),
        .rd(readdata)
    );

    // --- Contador de Performance ---
    reg [31:0] cycle_counter;
    reg program_finished;
    wire is_ebreak = (instr == 32'h00100073);

    always @(posedge clk) begin
        if (reset) begin
            cycle_counter <= 0;
            program_finished <= 0;
        end else begin
            if (is_ebreak) program_finished <= 1;
            if (!program_finished) cycle_counter <= cycle_counter + 1;
        end
    end

    // --- Saída Visual (EM DECIMAL) ---
    hex7seg d0 (cycle_counter % 10, HEX0);
    hex7seg d1 ((cycle_counter / 10) % 10, HEX1);
    hex7seg d2 ((cycle_counter / 100) % 10, HEX2);
    hex7seg d3 ((cycle_counter / 1000) % 10, HEX3);
    hex7seg d4 ((cycle_counter / 10000) % 10, HEX4);
    hex7seg d5 ((cycle_counter / 100000) % 10, HEX5);

    assign LEDR = pc[11:2]; 
endmodule