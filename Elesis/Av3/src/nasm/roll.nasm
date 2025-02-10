;-------------------------------------
; roll.nasm
; Leia o README.md para detalhes
;-------------------------------------

;R2
leaw $19, %A
movw (%A), %D
leaw $21, %A
movw %D, (%A)

;R3
leaw $18, %A
movw (%A), %D
leaw $19, %A
movw %D, (%A)

;R4
leaw $17, %A
movw (%A), %D
leaw $18, %A
movw %D, (%A)

;R5
leaw $16, %A
movw (%A), %D
leaw $17, %A
movw %D, (%A)

;R6
leaw $15, %A
movw (%A), %D
leaw $16, %A
movw %D, (%A)

;R1
leaw $21, %A
movw (%A), %D
leaw $15, %A
movw %D, (%A)

;passou no teste!!!