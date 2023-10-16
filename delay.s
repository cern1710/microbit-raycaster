; Output for delay() in -O1 or above

	.section	__TEXT,__text,regular,pure_instructions
	.build_version macos, 12, 0	sdk_version 13, 1
	.globl	_delay                          ; -- Begin function delay
	.p2align	2
_delay:                                 ; @delay
	.cfi_startproc
; %bb.0:
	sub	sp, sp, #16
	.cfi_def_cfa_offset 16
	mov	w8, #10000
	mul	w8, w0, w8
	str	w8, [sp, #12]
LBB0_1:                                 ; =>This Inner Loop Header: Depth=1
	ldr	w8, [sp, #12]
	sub	w9, w8, #1
	str	w9, [sp, #12]
	cbnz	w8, LBB0_1
; %bb.2:
	add	sp, sp, #16
	ret
	.cfi_endproc
                                        ; -- End function
.subsections_via_symbols
