; Arquivo: Factorial.nasm
; Curso: Elementos de Sistemas
; Criado por: Luciano Soares
; Data: 27/03/2017

; Calcula o fatorial do número em R0 e armazena o valor em R1.

; REG[0] -> VALOR DO FATORIAL
; REG[1] -> RESPOSTA, OU SEJA ONDE SERA SOMADO
; REG[2] -> CONTADOR MULTIPLICACAO
; REG[3] -> CONTADOR(FATORIAL-1)


    ;SALVANDO O VALOR DO FATORIAL:
leaw $0,%A ;0 -> RegA
movw (%A),%D ; RAM[0] -> RegD 


    ;SALVANDO O VALOR DO CONTADOR DO FATORIAL
leaw $3, %A 
movw %D, (%A) ; ATUALIZOU CONTADOR DO FATORIAL



FATORIAL:

    ; if RAM[0] == 0!:
leaw $SET_UM, %A
jle %D ; caso nao pule, vai seguir a linha de codigo normalmente
nop

    ;DECRESCENDO O VALOR DO CONTADOR NA RAM[3]
leaw $3, %A 
movw (%A), %D
decw %D ; RegD -1
movw %D, (%A) ; ATUALIZOU CONTADOR

    ;ATUALIZANDO O CONTADOR DA MULTIPLICAO NA RAM[2]
leaw $2, %A 
movw %D, (%A) ; ATUALIZOU CONTADOR DA MULTIPLICACAO


    ;INICIANDO A MULTIPLICACAO
    ;RAM[0] -> RESULTADO A SER MULTIPLICADO
    ;RAM[1] -> RESULTADO ACUMULADO
    ;RAM[2] -> CONTADOR DA MULTIPLICACAO
    ;RAM[3] -> CONTADOR DO FATORIAL


MULTIPLICACAO: ; label so para usarmos depois para fazer o jump

leaw $0,%A
movw (%A),%D; pega o valor da RAM[0] e armazena no RegD - > NUMERO A SER SOMADO
leaw $1, %A ;armazena 1 no regA
addw %D,(%A),%D ;soma os valores do RegD e armazena a soma na RAM[1]
movw %D,(%A)


    ;ATUALIZANDO CONTADOR DA MULTIPLICACAO -> RAM[2]:

leaw $2, %A ; armazena 1 no regA
movw (%A), %D ; pega o valor do RAM[3] e armazena no regD
decw %D ; subtrai 1 do valor da RAM[3], isso pq o regA esta com 1, para que contabilize o número de somas --CONTADOR RAM[1]
movw %D,(%A) ; armazena o valor do RegD na RAM[2], que é o CONTADOR

    ;armazenar o valor da Ram[0] no %D para fazer o if.
    ;LEMBRAR: %A -> PARA GUARDAR POSIÇÕES// %D -> PARA GUARDAR VALORES
     
; if contador < 0:
leaw $END_MULTIPLICACAO, %A ; associa o valor da linha END_LOOP no regA, para usar no "if"(jge) - PRECISA SEMPRE GUARDAR NO regA
jle %D ;se o valor da RAM[1], que é o contador(que foi transferido p o regD) for menor ou igual a zero, ele vai voltar uma linha e vai ir para o endereço da linha END_LOOP, jle - jump lower equal than
nop ;usado apenas para dar o tempo de fazer o jump


; else: (ou seja se o contador ainda é positivo), ele ira seguir o codigo 
leaw $MULTIPLICACAO, %A  ; se isso acontece ele ja volta direto para a linha do loop
jmp ; faz o jump para a linha anterior
nop ; da o tempo para realizar o jump

END_MULTIPLICACAO: ; vem para essa linha quando encerra o loop.



    ;ATUALIZANDO O VALOR DO CONTADOR DO FATORIAL NA RAM[3]
leaw $3, %A 
movw (%A), %D



    ;ATUALIZANDO O VALOR DO CONTADOR DA MULTIPLICACAO NA RAM[2]:
leaw $2, %A
movw %D, (%A) 

    ;ATUALIZANDO O VALOR DA RAM[0] PARA SER IGUAL AO DA RAM[1]:
;RAM[1]:
leaw $1,%A 
movw (%A), %D

;RAM[0]:
leaw $0, %A
movw %D, (%A)


    ;ZERANDO A RAM[1] para nao dar conflito
leaw $1, %A
movw $0 , (%A)

    ;if RAM[3] != 0:
leaw $3, %A
movw (%A), %D
leaw $FATORIAL, %A 
jg %D
nop

    ;else:
leaw $0, %A
movw (%A), %D
leaw $1, %A
movw %D, (%A)
leaw $END_FATORIAL, %A
jmp
nop

    ;SETANDO O RESULTADO DO FATORIAL CASO SEJA 1:
SET_UM: 
leaw $1,%A
movw %A,%D
movw %D,(%A) ; nao precisa pular pro END_FATORIAL pq ja vai executar a linha


END_FATORIAL:



