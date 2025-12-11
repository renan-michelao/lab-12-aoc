.data
# Array de 10 números inteiros desordenados
array:  .word 90, 5, 12, 3, 44, 1, 8, 100, 25, 2
size:   .word 10

.text
.globl main

main:
    # --- Configuração Inicial ---
    la   x10, array      # x10 = Endereço base do vetor (array)
    la   x11, size       # Carrega endereço do tamanho
    lw   x11, 0(x11)     # x11 = N (Tamanho do vetor = 10)
    
    # --- Início do Bubble Sort ---
    add  x5, x0, x0      # x5 (i) = 0

loop_externo:
    bge  x5, x11, fim    # Se i >= N, terminou a ordenação
    
    add  x6, x0, x0      # x6 (j) = 0
    sub  x31, x11, x5    # x31 = N - i
    addi x31, x31, -1    # x31 = N - i - 1 (Limite do loop interno)

loop_interno:
    bge  x6, x31, proximo_i # Se j >= N - i - 1, vai para próximo i
    
    # Cálculo do endereço: array[j]
    slli x7, x6, 2       # x7 = j * 4 (offset em bytes)
    add  x7, x7, x10     # x7 = Endereço de array[j]
    
    # Carrega os valores
    lw   x28, 0(x7)      # x28 = array[j]
    lw   x29, 4(x7)      # x29 = array[j+1]
    
    # Compara
    ble  x28, x29, nao_troca # Se array[j] <= array[j+1], não faz nada
    
    # Troca (Swap)
    sw   x29, 0(x7)      # array[j] = antigo array[j+1]
    sw   x28, 4(x7)      # array[j+1] = antigo array[j]

nao_troca:
    addi x6, x6, 1       # j++
    beq  x0, x0, loop_interno # Volta incondicionalmente para loop interno

proximo_i:
    addi x5, x5, 1       # i++
    beq  x0, x0, loop_externo # Volta incondicionalmente para loop externo

fim:
    ebreak               # Para a simulação (Instrução especial do RARS/RISC-V)
    beq x0, x0, fim      # Trava de segurança caso ebreak não funcione