; Arquivo: Abs.nasm
; Curso: Elementos de Sistemas
; Criado por: Luciano Soares
; Data: 27/03/2017

; Multiplica o valor de RAM[1] com RAM[0] salvando em RAM[3]

;RAM1 -> CONTADOR
;RAM[0] -> NUMERO A SER SOMADO


LOOP: ; label so para usarmos depois para fazer o jump
leaw $0,%A
movw (%A),%D; pega o valor da RAM[0] e armazena no RegD
leaw $3, %A ;armazena 3 no regA
addw %D,(%A),%D ;soma os valores do RegD e armazena a soma na RAM[3]
movw %D,(%A)

leaw $1, %A ; armazena 1 no regA
movw (%A), %D ; pega o valor do RAM[1] e armazena no regD
decw %D ; subtrai 1 do valor da RAM[1], isso pq o regD esta com 1, para que contabilize o número de somas --CONTADOR RAM[1]
movw %D,(%A) ; armazena o valor do RegD na RAM[1], que é o CONTADOR

    ;armazenar o valor da Ram[0] no %D para fazer o if.
    ;LEMBRAR: %A -> PARA GUARDAR POSIÇÕES// %D -> PARA GUARDAR VALORES
     
; if contador < 0:
leaw $END_LOOP, %A ; associa o valor da linha END_LOOP no regA, para usar no "if"(jge) - PRECISA SEMPRE GUARDAR NO regA
jle %D ;se o valor da RAM[1], que é o contador(que foi transferido p o regD) for menor ou igual a zero, ele vai voltar uma linha e vai ir para o endereço da linha END_LOOP, jle - jump lower equal than
nop ;usado apenas para dar o tempo de fazer o jump


; else: (ou seja se o contador ainda é positivo), ele ira seguir o codigo 
leaw $LOOP, %A  ; se isso acontece ele ja volta direto para a linha do loop
jmp ; faz o jump para a linha anterior
nop ; da o tempo para realizar o jump

END_LOOP: ; vem para essa linha quando encerra o loop.





