	.file	"target0.c"
	.section	.rodata
.LC0:
	.string	"Hi %s! Your grade is %s.\n"
	.text
	.globl	_main
	.type	_main, @function
_main:
.LFB2:
	.cfi_startproc
	pushl	%ebp
	.cfi_def_cfa_offset 8
	.cfi_offset 5, -8
	movl	%esp, %ebp
	.cfi_def_cfa_register 5
	subl	$56, %esp
	movl	12(%ebp), %eax
	movl	%eax, -44(%ebp)
	movl	%gs:20, %eax
	movl	%eax, -12(%ebp)
	xorl	%eax, %eax
	leal	-27(%ebp), %eax
	movl	$7104878, (%eax)
	subl	$12, %esp
	leal	-22(%ebp), %eax
	pushl	%eax
	call	gets
	addl	$16, %esp
	subl	$4, %esp
	leal	-27(%ebp), %eax
	pushl	%eax
	leal	-22(%ebp), %eax
	pushl	%eax
	pushl	$.LC0
	call	printf
	addl	$16, %esp
	subl	$12, %esp
	pushl	$0
	call	exit
	.cfi_endproc
.LFE2:
	.size	_main, .-_main
	.ident	"GCC: (Ubuntu 5.4.0-6ubuntu1~16.04.4) 5.4.0 20160609"
	.section	.note.GNU-stack,"",@progbits
