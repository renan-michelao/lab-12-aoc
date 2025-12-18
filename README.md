# Implementação de Predição de Desvio Dinâmica (BHT + BTB) em RISC-V

Este projeto consiste na implementação e validação em hardware (FPGA) de um **mecanismo de predição de desvio dinâmica** em um processador RISC-V de 5 estágios.

O sistema utiliza uma **Tabela de Histórico de Desvios (BHT)** de 2 bits e um **Buffer de Destino de Desvio (BTB)** para eliminar penalidades de controle (stalls/flushes) causadas por instruções de desvio (*branches*).

---

## 🚀 Resultados Obtidos

Utilizando o algoritmo **Bubble Sort** (ordenando um vetor de 10 inteiros) como benchmark, obtivemos os seguintes resultados na placa **DE0-CV (Cyclone V)**:

| Modo | Configuração | Ciclos Totais | Penalidade por Desvio |
| :--- | :--- | :--- | :--- |
| **Base (Original)** | Sem Predição (Turbo OFF) | ~133 Ciclos | 2 Ciclos (Flush no Execute) |
| **Otimizado (Turbo)** | Com BHT + BTB (Turbo ON) | **117 Ciclos** | **0 Cycles** (Em caso de acerto) |

> **Ganho de Performance:** Redução de aproximadamente **12%** no tempo de execução total para este algoritmo, sem alteração na frequência de clock.

---

## 🛠️ Detalhes da Implementação

### 1. Arquitetura do Preditor (Estágio Fetch)
A lógica de predição foi movida do estágio de *Execute* para o estágio de *Fetch* para antecipar o salto.

* **BHT (Branch History Table):** Tabela de 64 entradas com contadores de saturação de 2 bits (*Forte Não, Fraco Não, Fraco Sim, Forte Sim*). Isso proporciona histerese, evitando que o preditor mude de opinião erraticamente dentro de loops.
* **BTB (Branch Target Buffer):** Armazena o endereço de destino do salto. Se a predição for "Tomada", o PC é atualizado imediatamente para o alvo do BTB.
* **Correção de Erro:** No estágio de *Execute*, o processador valida a predição. Se houver erro (misprediction), realiza o *flush* do pipeline, corrige o PC e atualiza as tabelas de histórico.

### 2. Funcionalidades de Hardware (FPGA)
* **Modo Turbo (Switch 0):** Implementação de um interruptor físico (`SW[0]`) que permite ativar ou desativar a lógica do preditor em tempo real.
* **Visualização em Decimal:** O módulo de display de 7 segmentos foi programado com lógica matemática (`% 10`, `/ 10`) para exibir a contagem de ciclos diretamente em decimal (ex: `117`) ao invés de hexadecimal.
* **Reset Síncrono de Tabelas:** Solução de hardware para limpar as tabelas BHT/BTB ao pressionar o Reset (`KEY[0]`). Isso corrige erros de predição causados por dados "sujos" de execuções anteriores (*Cold Start*).

---

## 📂 Arquivos do Projeto

* `riscvpipeline.sv` – Núcleo do processador (Pipeline 5 estágios + Lógica BHT/BTB).
* `top.sv` – Módulo Top-Level (Integração CPU, Memórias, Displays e Switches).
* `mem.sv` – Memória RAM simulada (Instruções e Dados com suporte a .hex).
* `hex7seg.sv` – Decodificador para displays de 7 segmentos.
* `riscv.hex` – Código de máquina do Bubble Sort.
* `data.hex` – Vetor de dados desordenados para teste.
* `testbanch.sv` – Testbench para validação via simulação (ModelSim/Icarus).

---

## 🎮 Como Testar na Placa (DE0-CV)

1.  Carregue o projeto na FPGA utilizando o **Quartus Programmer**.
2.  **Teste Base (Sem Predição):**
    * Posicione a chave `SW[0]` para **BAIXO**.
    * Pressione `KEY[0]` (Reset).
    * O display exibirá o resultado base: **~133**.
3.  **Teste Turbo (Com Predição):**
    * Posicione a chave `SW[0]` para **CIMA**.
    * **Importante:** Clique em "Start" no Quartus Programmer (para recarregar a memória RAM desordenada).
    * Pressione `KEY[0]` (Reset).
    * O display exibirá o resultado otimizado: **117**.

---

## Conclusão

O projeto demonstra que o custo de hardware adicional para manter tabelas de histórico e destino (BHT/BTB) é amplamente justificado pelo ganho de **IPC (Instruções por Ciclo)** em algoritmos com fluxo de controle complexo.

---
**Autores:** Grupo D1
**Disciplina:** Arquitetura e Organização de Computadores
