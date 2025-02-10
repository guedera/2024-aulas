; ------------------------------------
; Calcule a média dos valores de um vetor
; que possui inicio em RAM[5] e tamanho
; defindo em RAM[4],
;
; 1. Salve a soma em RAM[1]
; 2. Salve a média em RAM[0]
; 
; ------------------------------------
; antes       | depois
;             |
; RAM[0]:     | RAM[0]:  2  : média 
; RAM[1]:     | RAM[1]:  8  : soma
; RAM[2]:     | RAM[2]:  
; RAM[3]:     | RAM[3]:  
; RAM[4]:  4  | RAM[4]:  4 
; RAM[5]:  1  | RAM[5]:  1 - 
; RAM[6]:  2  | RAM[6]:  2 | vetor
; RAM[7]:  1  | RAM[7]:  1 |
; RAM[8]:  4  | RAM[8]:  4 -
; ------------------------------------


leaw $4, %A             ; endereço 4 e coloca em %A
movw (%A), %D           ; valor em RAM[4] e joga em %D
leaw $3, %A             ; Agora %A vai pra 3 (RAM[3])
movw %D, (%A)           ; Move o valor de %D para o endereço %A

leaw $5, %A             ; Pega o endereço 5 e joga em %A
movw %A, %D             ; Copia o valor de %A pra %D
leaw $2, %A             ; Agora %A vai pra 2 (RAM[2])
movw %D, (%A)           ; Salva o valor de %D em RAM[2]


SUM:
leaw $3, %A            
movw (%A), %D           ; Move o valor no endereço %A para %D

leaw $MEAN, %A         
je %D                   ; Salta para o endereço em %A se a flag zero estiver definida
nop                     ; Não faz nada (no operation)


leaw $2, %A            
movw (%A), %A           
movw (%A), %D           
leaw $1, %A            
addw (%A), %D, %D       ; Adiciona o valor no endereço %A a %D e armazena em %D
movw %D, (%A)           ; Move o valor de %D para o endereço %A

leaw %2, %A           
movw (%A), %D           
incw %D                
movw %D, (%A)           

leaw %3, %A            
movw (%A), %D          
decw %D                 
movw %D, (%A)          

leaw $SUM, %A           ; Carrega o endereço do rótulo SUM em %A
jmp                     ; Salta para o endereço em %A
nop                     ; Não faz nada (no operation)

MEAN:
leaw $1, %A             ; Carrega o endereço 1 em %A
movw (%A), %D           ; Move o valor no endereço %A para %D
leaw $2, %A             ; Carrega o endereço 2 em %A
movw %D, (%A)           ; Move o valor de %D para o endereço %A


SUB:
leaw $2, %A             ; Carrega o endereço 2 em %A
movw (%A), %D           ; Move o valor no endereço %A para %D
leaw %4, %A             ; Carrega o endereço 4 em %A
subw %D, (%A), %D       ; Subtrai o valor no endereço %A de %D e armazena em %D
leaw $MAIORQUEZERO, %A  ; Carrega o endereço do rótulo MAIORQUEZERO em %A
jge %D                  ; Salta para o endereço em %A se %D for maior ou igual a zero
nop                     ; Não faz nada (no operation)

leaw $END, %A           ; Carrega o endereço do rótulo END em %A
jmp                     ; Salta para o endereço em %A
nop                     ; Não faz nada (no operation)

MAIORQUEZERO:
leaw $2, %A             ; Carrega o endereço 2 em %A
movw %D, (%A)           ; Move o valor de %D para o endereço %A
leaw $0, %A             ; Carrega o endereço 0 em %A
movw (%A), %D           ; Move o valor no endereço %A para %D
incw %D                 ; Incrementa %D
movw %D, (%A)           ; Move o valor de %D para o endereço %A
leaw %SUB, %A           ; Carrega o endereço do rótulo SUB em %A
jmp                     ; Salta para o endereço em %A
nop                     ; Não faz nada (no operation)

END:
