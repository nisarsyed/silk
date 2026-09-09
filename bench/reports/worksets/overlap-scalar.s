	.build_version macos, 26, 0	sdk_version 26, 5
	.section	__TEXT,__text,regular,pure_instructions
	.globl	_inspect_overlap                ; -- Begin function inspect_overlap
	.p2align	2
_inspect_overlap:                       ; @inspect_overlap
	.cfi_startproc
; %bb.0:
	mov	w0, #0                          ; =0x0
	fmov	w8, s1
	fmov	w9, s0
	and	w9, w9, #0x7fffffff
	and	w8, w8, #0x7fffffff
	mov	w10, #2139095040                ; =0x7f800000
	cmp	w8, w10
	cset	w8, lt
	mov	w10, #2139095039                ; =0x7f7fffff
	cmp	w9, w10
	b.gt	LBB0_13
; %bb.1:
	cbz	w8, LBB0_13
; %bb.2:
	mov	w0, #0                          ; =0x0
	fmov	w8, s2
	and	w9, w8, #0x7fffffff
	fmov	w8, s3
	and	w8, w8, #0x7fffffff
	mov	w10, #2139095040                ; =0x7f800000
	cmp	w8, w10
	cset	w8, lt
	mov	w10, #2139095039                ; =0x7f7fffff
	cmp	w9, w10
	b.gt	LBB0_13
; %bb.3:
	cbz	w8, LBB0_13
; %bb.4:
	fcmp	s0, s2
	b.hi	LBB0_13
; %bb.5:
	fcmp	s1, s3
	b.hi	LBB0_13
; %bb.6:
	mov	w0, #0                          ; =0x0
	fmov	w8, s4
	and	w9, w8, #0x7fffffff
	fmov	w8, s5
	and	w8, w8, #0x7fffffff
	mov	w10, #2139095040                ; =0x7f800000
	cmp	w8, w10
	cset	w8, lt
	mov	w10, #2139095039                ; =0x7f7fffff
	cmp	w9, w10
	b.gt	LBB0_13
; %bb.7:
	cbz	w8, LBB0_13
; %bb.8:
	mov	w0, #0                          ; =0x0
	fmov	w8, s6
	and	w9, w8, #0x7fffffff
	fmov	w8, s7
	and	w8, w8, #0x7fffffff
	mov	w10, #2139095040                ; =0x7f800000
	cmp	w8, w10
	cset	w8, lt
	mov	w10, #2139095039                ; =0x7f7fffff
	cmp	w9, w10
	b.gt	LBB0_13
; %bb.9:
	cbz	w8, LBB0_13
; %bb.10:
	fcmp	s4, s6
	b.hi	LBB0_13
; %bb.11:
	fcmp	s5, s7
	b.hi	LBB0_13
; %bb.12:
	fcmp	s2, s4
	cset	w8, ge
	fcmp	s1, s7
	fccmp	s0, s6, #2, ls
	cset	w9, ls
	and	w8, w9, w8
	fcmp	s3, s5
	csel	w0, wzr, w8, lt
LBB0_13:
	ret
	.cfi_endproc
                                        ; -- End function
.subsections_via_symbols
