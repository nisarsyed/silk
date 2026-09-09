	.build_version macos, 26, 0	sdk_version 26, 5
	.section	__TEXT,__text,regular,pure_instructions
	.globl	_inspect_overlap                ; -- Begin function inspect_overlap
	.p2align	2
_inspect_overlap:                       ; @inspect_overlap
	.cfi_startproc
; %bb.0:
                                        ; kill: def $s7 killed $s7 def $q7
                                        ; kill: def $s6 killed $s6 def $q6
                                        ; kill: def $s5 killed $s5 def $q5
                                        ; kill: def $s4 killed $s4 def $q4
                                        ; kill: def $s3 killed $s3 def $q3
                                        ; kill: def $s2 killed $s2 def $q2
                                        ; kill: def $s1 killed $s1 def $q1
                                        ; kill: def $s0 killed $s0 def $q0
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
	mov.s	v0[1], v4[0]
	mov.s	v0[2], v1[0]
	mov.s	v0[3], v5[0]
	mov.s	v6[1], v2[0]
	mov.s	v6[2], v7[0]
	mov.s	v6[3], v3[0]
	fcmge.4s	v0, v6, v0
	uminv.4s	s0, v0
	fmov	w8, s0
	cmp	w8, #0
	cset	w0, ne
LBB0_13:
	ret
	.cfi_endproc
                                        ; -- End function
.subsections_via_symbols
