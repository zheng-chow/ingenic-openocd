.section .text,"ax",@progbits
.set noat
.set noreorder
L1:
	li	$8,0xff
	li	$9,0x1f
	addu	$10,$8,$9
	la	$11,data
	la	$12,L1
	jr 	$12
	nop	

.section .data,"a",@progbits
.set noat
.set noreorder

data:
.word	0x0000ffff
.word 	0x0000000f
.word 	0xffffffff		
