; ------------------------------------------------------------
; Arquivo: Mod.nasm
; Curso: Elementos de Sistemas
; Criado por: Luciano Soares
; Data: 27/03/2017
;
; Calcula o resto da divisão (modulus) entre RAM[0] por RAM[1]
; e armazena o resultado na RAM[2].
;
; 4  % 3 = 1
; 10 % 7 = 3
; ------------------------------------------------------------

    leaw $R2, %A
    movw $0, (%A)

    leaw $R3, %A
    movw $0, (%A)

LOOP:
    leaw $R2, %A
    movw (%A), %D
    incw %D
    movw %D, (%A)

    leaw $R2, %A
    movw (%A), %D
    leaw $R1, %A
    movw (%A), %A
    subw %A, %D, %D

    leaw $RESET_REMAINDER, %A
    je %D
    nop

CONTINUE_INCREMENT:
    leaw $R3, %A
    movw (%A), %D
    incw %D
    movw %D, (%A)

    leaw $R3, %A
    movw (%A), %D
    leaw $R0, %A
    movw (%A), %A
    subw %A, %D, %D

    leaw $END, %A
    je %D
    nop

    leaw $LOOP, %A
    jmp
    nop

RESET_REMAINDER:
    leaw $R2, %A
    movw $0, (%A)

    leaw $CONTINUE_INCREMENT, %A
    jmp
    nop

END: