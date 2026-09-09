	.build_version macos, 26, 0	sdk_version 26, 5
	.section	__TEXT,__literal16,16byte_literals
	.p2align	4, 0x0                          ; -- Begin function sl_world_step
lCPI0_0:
	.long	0x3f800000                      ; float 1
	.long	0x00000000                      ; float 0
	.long	0x3f800000                      ; float 1
	.long	0x00000000                      ; float 0
	.section	__TEXT,__literal8,8byte_literals
	.p2align	3, 0x0
lCPI0_1:
	.long	0x3f800000                      ; float 1
	.long	0x00000000                      ; float 0
	.section	__TEXT,__text,regular,pure_instructions
	.globl	_sl_world_step
	.p2align	2
_sl_world_step:                         ; @sl_world_step
	.cfi_startproc
; %bb.0:
	cbz	x0, LBB0_95
; %bb.1:
	sub	sp, sp, #224
	stp	d15, d14, [sp, #64]             ; 16-byte Folded Spill
	stp	d13, d12, [sp, #80]             ; 16-byte Folded Spill
	stp	d11, d10, [sp, #96]             ; 16-byte Folded Spill
	stp	d9, d8, [sp, #112]              ; 16-byte Folded Spill
	stp	x28, x27, [sp, #128]            ; 16-byte Folded Spill
	stp	x26, x25, [sp, #144]            ; 16-byte Folded Spill
	stp	x24, x23, [sp, #160]            ; 16-byte Folded Spill
	stp	x22, x21, [sp, #176]            ; 16-byte Folded Spill
	stp	x20, x19, [sp, #192]            ; 16-byte Folded Spill
	stp	x29, x30, [sp, #208]            ; 16-byte Folded Spill
	add	x29, sp, #208
	.cfi_def_cfa w29, 16
	.cfi_offset w30, -8
	.cfi_offset w29, -16
	.cfi_offset w19, -24
	.cfi_offset w20, -32
	.cfi_offset w21, -40
	.cfi_offset w22, -48
	.cfi_offset w23, -56
	.cfi_offset w24, -64
	.cfi_offset w25, -72
	.cfi_offset w26, -80
	.cfi_offset w27, -88
	.cfi_offset w28, -96
	.cfi_offset b8, -104
	.cfi_offset b9, -112
	.cfi_offset b10, -120
	.cfi_offset b11, -128
	.cfi_offset b12, -136
	.cfi_offset b13, -144
	.cfi_offset b14, -152
	.cfi_offset b15, -160
	ldr	x19, [x0]
	cbz	x19, LBB0_94
; %bb.2:
	ldr	w8, [x19, #204]
	sub	w9, w8, #1
	cmp	w9, #7
	b.hi	LBB0_94
; %bb.3:
	ldr	w9, [x19, #200]
	sub	w11, w9, #1
	and	w10, w9, #0x7fffffff
	sub	w10, w10, #2048, lsl #12        ; =8388608
	lsr	w10, w10, #24
	cmp	w9, #0
	mov	w9, #126                        ; =0x7e
	ccmp	w10, w9, #2, ge
	mov	w10, #8388606                   ; =0x7ffffe
	ccmp	w11, w10, #0, hi
	b.hi	LBB0_94
; %bb.4:
	ldr	w11, [x19, #208]
	sub	w12, w11, #1
	and	w13, w11, #0x7fffffff
	sub	w13, w13, #2048, lsl #12        ; =8388608
	lsr	w13, w13, #24
	cmp	w11, #0
	ccmp	w13, w9, #2, ge
	ccmp	w12, w10, #0, hi
	b.hi	LBB0_94
; %bb.5:
	ldr	w9, [x19, #212]
	sub	w11, w9, #1
	and	w10, w9, #0x7fffffff
	sub	w10, w10, #2048, lsl #12        ; =8388608
	lsr	w10, w10, #24
	cmp	w9, #0
	mov	w9, #126                        ; =0x7e
	ccmp	w10, w9, #2, ge
	mov	w10, #8388606                   ; =0x7ffffe
	ccmp	w11, w10, #0, hi
	b.hi	LBB0_94
; %bb.6:
	ldr	w11, [x19, #216]
	sub	w12, w11, #1
	and	w13, w11, #0x7fffffff
	sub	w13, w13, #2048, lsl #12        ; =8388608
	lsr	w13, w13, #24
	cmp	w11, #0
	ccmp	w13, w9, #2, ge
	ccmp	w12, w10, #0, hi
	b.hi	LBB0_94
; %bb.7:
	ldr	w9, [x19, #220]
	sub	w11, w9, #1
	and	w10, w9, #0x7fffffff
	sub	w10, w10, #2048, lsl #12        ; =8388608
	lsr	w10, w10, #24
	cmp	w9, #0
	mov	w9, #126                        ; =0x7e
	ccmp	w10, w9, #2, ge
	mov	w10, #8388606                   ; =0x7ffffe
	ccmp	w11, w10, #0, hi
	b.hi	LBB0_94
; %bb.8:
	ldr	w11, [x19, #224]
	sub	w12, w11, #1
	and	w13, w11, #0x7fffffff
	sub	w13, w13, #2048, lsl #12        ; =8388608
	lsr	w13, w13, #24
	cmp	w11, #0
	ccmp	w13, w9, #2, ge
	ccmp	w12, w10, #0, hi
	b.hi	LBB0_94
; %bb.9:
	ldr	w9, [x19, #228]
	sub	w11, w9, #1
	and	w10, w9, #0x7fffffff
	sub	w10, w10, #2048, lsl #12        ; =8388608
	lsr	w10, w10, #24
	cmp	w9, #0
	mov	w9, #126                        ; =0x7e
	ccmp	w10, w9, #2, ge
	mov	w10, #8388606                   ; =0x7ffffe
	ccmp	w11, w10, #0, hi
	b.hi	LBB0_94
; %bb.10:
	fmov	w11, s0
	sub	w12, w11, #1
	and	w13, w11, #0x7fffffff
	sub	w13, w13, #2048, lsl #12        ; =8388608
	lsr	w13, w13, #24
	cmp	w11, #0
	ccmp	w13, w9, #2, ge
	ccmp	w12, w10, #0, hi
	b.hi	LBB0_94
; %bb.11:
	ucvtf	s1, w8
	fdiv	s1, s0, s1
	str	q1, [sp, #48]                   ; 16-byte Folded Spill
	fmov	w8, s1
	sub	w10, w8, #1
	and	w9, w8, #0x7fffffff
	sub	w9, w9, #2048, lsl #12          ; =8388608
	lsr	w9, w9, #24
	cmp	w8, #0
	mov	w8, #126                        ; =0x7e
	ccmp	w9, w8, #2, ge
	mov	w9, #8388606                    ; =0x7ffffe
	ccmp	w10, w9, #0, hi
	b.hi	LBB0_94
; %bb.12:
	fmov	s1, #1.00000000
	fdiv	s8, s1, s0
	fmov	w10, s8
	sub	w11, w10, #1
	and	w12, w10, #0x7fffffff
	sub	w12, w12, #2048, lsl #12        ; =8388608
	lsr	w12, w12, #24
	cmp	w10, #0
	ccmp	w12, w8, #2, ge
	ccmp	w11, w9, #0, hi
	b.hi	LBB0_94
; %bb.13:
	ldr	q2, [sp, #48]                   ; 16-byte Folded Reload
	fdiv	s11, s1, s2
	fmov	w8, s11
	sub	w9, w8, #1
	and	w10, w8, #0x7fffffff
	sub	w10, w10, #2048, lsl #12        ; =8388608
	lsr	w10, w10, #24
	cmp	w8, #0
	mov	w8, #126                        ; =0x7e
	ccmp	w10, w8, #2, ge
	mov	w8, #8388606                    ; =0x7ffffe
	ccmp	w9, w8, #0, hi
	b.hi	LBB0_94
; %bb.14:
	ldp	s9, s10, [x19, #192]
	add	x8, x19, #856
	str	xzr, [x19, #952]
	movi.2d	v1, #0000000000000000
	stp	q1, q1, [x8]
	stp	q1, q1, [x8, #32]
	stp	q1, q1, [x8, #64]
	str	x8, [sp]                        ; 8-byte Folded Spill
	mov	w8, #1                          ; =0x1
	strb	w8, [x19, #972]
	mov	x0, x19
	bl	_sl_sleep_step_begin
	mov	x0, x19
	bl	_sl_contact_step_begin
	mov	x0, x19
	bl	_sl_wake_finish
	mov	x0, x19
	bl	_sl_islands_build
	mov	x0, x19
	bl	_sl_sleep_graph_ready
	mov	x0, x19
	ldr	q0, [sp, #48]                   ; 16-byte Folded Reload
                                        ; kill: def $s0 killed $s0 killed $q0
	mov.16b	v1, v11
	bl	_sl_joint_prepare
	mov	x0, x19
	ldr	q0, [sp, #48]                   ; 16-byte Folded Reload
                                        ; kill: def $s0 killed $s0 killed $q0
	str	s11, [sp, #44]                  ; 4-byte Folded Spill
	mov.16b	v1, v11
	bl	_sl_solver_prepare
	ldr	w8, [x19, #136]
	cbz	w8, LBB0_21
; %bb.15:
	ldr	x10, [x19, #296]
	ldr	x9, [x19, #312]
	cmp	w8, #3
	b.ls	LBB0_18
; %bb.16:
	lsl	x11, x8, #3
	add	x12, x9, x11
	cmp	x10, x12
	b.hs	LBB0_83
; %bb.17:
	add	x11, x10, x11
	cmp	x9, x11
	b.hs	LBB0_83
LBB0_18:
	mov	x11, #0                         ; =0x0
	adrp	x13, lCPI0_1@PAGE
LBB0_19:
	lsl	x12, x11, #3
	add	x10, x10, x12
	add	x9, x9, x12
	sub	x8, x8, x11
	ldr	d0, [x13, lCPI0_1@PAGEOFF]
LBB0_20:                                ; =>This Inner Loop Header: Depth=1
	str	xzr, [x10], #8
	str	d0, [x9], #8
	subs	x8, x8, #1
	b.ne	LBB0_20
LBB0_21:
	ldr	w8, [x19, #204]
	cbz	w8, LBB0_54
; %bb.22:
	mov	w22, #0                         ; =0x0
	fmov	s0, #1.00000000
	mov	w8, #4059                       ; =0xfdb
	movk	w8, #16201, lsl #16
	ldr	q2, [sp, #48]                   ; 16-byte Folded Reload
	fmadd	s1, s2, s9, s0
	fmadd	s2, s2, s10, s0
	fmov	s3, w8
	fdiv	s1, s0, s1
	str	q1, [sp, #16]                   ; 16-byte Folded Spill
	fmul	s13, s8, s3
	fneg	s14, s13
	movi.2s	v15, #70, lsl #24
	fdiv	s0, s0, s2
	str	s0, [sp, #12]                   ; 4-byte Folded Spill
	mvni.2s	v0, #127, msl #16
	fneg.2s	v8, v0
	b	LBB0_24
LBB0_23:                                ;   in Loop: Header=BB0_24 Depth=1
	mov	x0, x19
	mov	w1, #0                          ; =0x0
	bl	_sl_joint_solve
	mov	x0, x19
	ldr	s0, [sp, #44]                   ; 4-byte Folded Reload
	mov	w1, #0                          ; =0x0
	bl	_sl_solver_solve
	add	w22, w22, #1
	ldr	w8, [x19, #204]
	cmp	w22, w8
	b.hs	LBB0_54
LBB0_24:                                ; =>This Loop Header: Depth=1
                                        ;     Child Loop BB0_28 Depth 2
                                        ;     Child Loop BB0_37 Depth 2
	ldr	w20, [x19, #68]
	ldr	w8, [x19, #136]
	adds	w25, w8, w20
	b.eq	LBB0_34
; %bb.25:                               ;   in Loop: Header=BB0_24 Depth=1
	mov	x26, #0                         ; =0x0
	ldr	x27, [x19, #368]
	neg	w28, w20
	b	LBB0_28
LBB0_26:                                ;   in Loop: Header=BB0_28 Depth=2
	ldr	s2, [x19, #200]
	mov	s1, v0[1]
                                        ; kill: def $s0 killed $s0 killed $q0
	bl	_linear_velocity_cap
	stp	s0, s1, [x23]
	fcmp	s9, s14
	fcsel	s0, s9, s14, gt
	fcmp	s0, s13
	fcsel	s0, s0, s13, mi
	str	s0, [x24, x21, lsl #2]
LBB0_27:                                ;   in Loop: Header=BB0_28 Depth=2
	add	x26, x26, #1
	add	w28, w28, #1
	cmp	x25, x26
	b.eq	LBB0_34
LBB0_28:                                ;   Parent Loop BB0_24 Depth=1
                                        ; =>  This Inner Loop Header: Depth=2
	mov	x8, x28
	cmp	x26, x20
	b.hs	LBB0_30
; %bb.29:                               ;   in Loop: Header=BB0_28 Depth=2
	ldr	x8, [x19, #232]
	ldr	x9, [x19, #104]
	ldr	w9, [x9, x26, lsl #2]
	lsl	x9, x9, #3
	ldr	w8, [x8, x9]
LBB0_30:                                ;   in Loop: Header=BB0_28 Depth=2
	mov	w21, w8
	ldrb	w8, [x27, x21]
	ands	w8, w8, #0xff
	ccmp	x26, x20, #0, eq
	ccmp	w8, #1, #2, lo
	b.hi	LBB0_27
; %bb.31:                               ;   in Loop: Header=BB0_28 Depth=2
	ldr	x9, [x19, #48]
	ldrb	w9, [x9, x21]
	cbnz	w9, LBB0_27
; %bb.32:                               ;   in Loop: Header=BB0_28 Depth=2
	ldr	x9, [x19, #264]
	add	x23, x9, x21, lsl #3
	ldr	d0, [x23]
	ldr	x24, [x19, #320]
	ldr	s9, [x24, x21, lsl #2]
	cbnz	w8, LBB0_26
; %bb.33:                               ;   in Loop: Header=BB0_28 Depth=2
	ldr	x8, [x19, #280]
	ldr	s1, [x8, x21, lsl #2]
	ldr	x8, [x19, #328]
	ldr	s2, [x8, x21, lsl #2]
	ldr	x8, [x19, #288]
	ldr	x9, [x19, #344]
	ldr	s3, [x9, x21, lsl #2]
	ldr	d4, [x8, x21, lsl #3]
	fmul	s2, s2, s3
	fmul.2s	v1, v4, v1[0]
	ldr	d3, [x19, #184]
	fadd.2s	v1, v1, v3
	mov.16b	v3, v0
	ldr	q5, [sp, #48]                   ; 16-byte Folded Reload
	fmla.2s	v3, v1, v5[0]
	ldr	q1, [sp, #16]                   ; 16-byte Folded Reload
	fmul.2s	v1, v3, v1[0]
	fabs.2s	v3, v1
	fcmgt.2s	v4, v3, v8
	fcmgt.2s	v3, v8, v3
	orr.8b	v3, v3, v4
	bit.8b	v0, v1, v3
	fmadd	s1, s2, s5, s9
	ldr	s2, [sp, #12]                   ; 4-byte Folded Reload
	fmul	s1, s2, s1
	fmov	w8, s1
	and	w8, w8, #0x7fffffff
	mov	w9, #2139095040                 ; =0x7f800000
	cmp	w8, w9
	fcsel	s9, s1, s9, lt
	b	LBB0_26
LBB0_34:                                ;   in Loop: Header=BB0_24 Depth=1
	mov	x0, x19
	bl	_sl_joint_warm_start
	mov	x0, x19
	bl	_sl_solver_warm_start
	mov	x0, x19
	mov	w1, #1                          ; =0x1
	bl	_sl_joint_solve
	mov	x0, x19
	ldr	s0, [sp, #44]                   ; 4-byte Folded Reload
	mov	w1, #1                          ; =0x1
	bl	_sl_solver_solve
	ldr	w25, [x19, #68]
	ldr	w27, [x19, #136]
	cmn	w27, w25
	mov	w21, #2139095039                ; =0x7f7fffff
	b.eq	LBB0_23
; %bb.35:                               ;   in Loop: Header=BB0_24 Depth=1
	mov	x26, #0                         ; =0x0
	b	LBB0_37
LBB0_36:                                ;   in Loop: Header=BB0_37 Depth=2
	add	x26, x26, #1
	add	w8, w27, w25
	cmp	x26, x8
	b.hs	LBB0_23
LBB0_37:                                ;   Parent Loop BB0_24 Depth=1
                                        ; =>  This Inner Loop Header: Depth=2
	mov	w9, w25
	cmp	x26, x9
	b.hs	LBB0_39
; %bb.38:                               ;   in Loop: Header=BB0_37 Depth=2
	ldr	x8, [x19, #232]
	ldr	x10, [x19, #104]
	ldr	w10, [x10, x26, lsl #2]
	lsl	x10, x10, #3
	ldr	w8, [x8, x10]
	b	LBB0_40
LBB0_39:                                ;   in Loop: Header=BB0_37 Depth=2
	sub	w8, w26, w25
LBB0_40:                                ;   in Loop: Header=BB0_37 Depth=2
	ldr	x10, [x19, #368]
	mov	w20, w8
	ldrb	w8, [x10, x20]
	ands	w8, w8, #0xff
	ccmp	x26, x9, #0, eq
	ccmp	w8, #1, #2, lo
	b.hi	LBB0_36
; %bb.41:                               ;   in Loop: Header=BB0_37 Depth=2
	ldr	x9, [x19, #48]
	ldrb	w9, [x9, x20]
	cbnz	w9, LBB0_36
; %bb.42:                               ;   in Loop: Header=BB0_37 Depth=2
	ldr	x9, [x19, #296]
	lsl	x10, x20, #3
	add	x13, x9, x10
	ldp	s9, s10, [x13]
	ldr	x11, [x19, #312]
	add	x28, x11, x10
	ldp	s11, s12, [x28]
	ldp	x11, x12, [x19, #256]
	add	x14, x12, x10
	ldr	s0, [x14]
	ldr	q1, [sp, #48]                   ; 16-byte Folded Reload
	fmadd	s0, s0, s1, s9
	fmov	w15, s0
	and	w15, w15, #0x7fffffff
	cmp	w15, w21
	b.gt	LBB0_44
; %bb.43:                               ;   in Loop: Header=BB0_37 Depth=2
	ldr	s1, [x11, x10]
	fadd	s1, s0, s1
	fabs	s1, s1
	fcmp	s1, s15
	b.ls	LBB0_46
LBB0_44:                                ;   in Loop: Header=BB0_37 Depth=2
	mov.16b	v0, v10
	cbnz	w8, LBB0_47
; %bb.45:                               ;   in Loop: Header=BB0_37 Depth=2
	str	wzr, [x14]
	add	x13, x9, x20, lsl #3
	ldr	s0, [x13, #4]
	b	LBB0_47
LBB0_46:                                ;   in Loop: Header=BB0_37 Depth=2
	str	s0, [x13]
	mov.16b	v0, v10
LBB0_47:                                ;   in Loop: Header=BB0_37 Depth=2
	add	x12, x12, x10
	ldr	s1, [x12, #4]!
	ldr	q2, [sp, #48]                   ; 16-byte Folded Reload
	fmadd	s0, s1, s2, s0
	fmov	w13, s0
	and	w13, w13, #0x7fffffff
	cmp	w13, w21
	b.gt	LBB0_49
; %bb.48:                               ;   in Loop: Header=BB0_37 Depth=2
	add	x11, x11, x20, lsl #3
	ldr	s1, [x11, #4]
	fadd	s1, s0, s1
	fabs	s1, s1
	fcmp	s1, s15
	b.ls	LBB0_51
LBB0_49:                                ;   in Loop: Header=BB0_37 Depth=2
	cbnz	w8, LBB0_52
; %bb.50:                               ;   in Loop: Header=BB0_37 Depth=2
	str	wzr, [x12]
	b	LBB0_52
LBB0_51:                                ;   in Loop: Header=BB0_37 Depth=2
	add	x8, x9, x10
	str	s0, [x8, #4]!
LBB0_52:                                ;   in Loop: Header=BB0_37 Depth=2
	ldr	x8, [x19, #320]
	ldr	s0, [x8, x20, lsl #2]
	ldr	q1, [sp, #48]                   ; 16-byte Folded Reload
	fmul	s2, s1, s0
	ldp	s0, s1, [x28]
	bl	_sl_rotation_integrate
	stp	s0, s1, [x28]
	ldrb	w8, [x19]
	cmp	w8, #1
	b.ne	LBB0_36
; %bb.53:                               ;   in Loop: Header=BB0_37 Depth=2
	mov	x0, x19
	mov	x1, x20
	mov.16b	v0, v9
	mov.16b	v1, v10
	mov.16b	v2, v11
	mov.16b	v3, v12
	bl	_sl_sleep_travel
	ldr	w25, [x19, #68]
	ldr	w27, [x19, #136]
	b	LBB0_36
LBB0_54:
	mov	x0, x19
	bl	_sl_solver_restitution
	mov	x0, x19
	bl	_sl_joint_store
	mov	x0, x19
	bl	_sl_solver_store
	ldr	w8, [x19, #136]
	cbz	w8, LBB0_59
; %bb.55:
	mov	x23, #0                         ; =0x0
	mov	w22, #0                         ; =0x0
	mov	w20, #0                         ; =0x0
	ldr	x24, [x19, #368]
	ldr	x21, [x19, #48]
	lsl	x25, x8, #3
	b	LBB0_57
LBB0_56:                                ;   in Loop: Header=BB0_57 Depth=1
	add	x23, x23, #8
	cmp	x25, x23
	b.eq	LBB0_60
LBB0_57:                                ; =>This Inner Loop Header: Depth=1
	ldrb	w8, [x24], #1
	ldrb	w9, [x21], #1
	orr	w10, w8, w9
	tst	w10, #0xff
	cinc	w22, w22, eq
	cmp	w8, #1
	cinc	w20, w20, eq
	cbnz	w9, LBB0_56
; %bb.58:                               ;   in Loop: Header=BB0_57 Depth=1
	ldp	x8, x9, [x19, #296]
	ldr	d0, [x8, x23]
	ldr	x8, [x19, #256]
	ldr	d1, [x8, x23]
	fadd.2s	v0, v1, v0
	str	d0, [x8, x23]
	add	x26, x9, x23
	ldr	x8, [x19, #312]
	add	x8, x8, x23
	ldp	s1, s2, [x8]
	ldp	s3, s4, [x26]
	fnmul	s0, s4, s2
	fmadd	s0, s1, s3, s0
	fmul	s1, s1, s4
	fmadd	s1, s2, s3, s1
	bl	_sl_rotation_normalize
	stp	s0, s1, [x26]
	b	LBB0_56
LBB0_59:
	mov	w20, #0                         ; =0x0
	mov	w22, #0                         ; =0x0
LBB0_60:
	stp	w22, w20, [sp, #44]             ; 8-byte Folded Spill
	ldr	w8, [x19, #64]
	cbz	w8, LBB0_63
; %bb.61:
	mov	w20, #0                         ; =0x0
	mov	w22, #0                         ; =0x0
LBB0_62:                                ; =>This Inner Loop Header: Depth=1
	mov	x0, x19
	mov	x1, x20
	bl	_sl_island_is_sleeping
	add	w22, w22, w0
	add	w20, w20, #1
	ldr	w23, [x19, #64]
	cmp	w20, w23
	b.lo	LBB0_62
	b	LBB0_64
LBB0_63:
	mov	w22, #0                         ; =0x0
	mov	w23, #0                         ; =0x0
LBB0_64:
	mov	x0, x19
	bl	_sl_sleep_step_end
	ldr	w8, [x19, #136]
	cbz	w8, LBB0_69
; %bb.65:
	ldr	x10, [x19, #312]
	ldp	x11, x12, [x19, #288]
	ldr	x9, [x19, #328]
	cmp	w8, #3
	b.hi	LBB0_73
; %bb.66:
	mov	x13, #0                         ; =0x0
LBB0_67:
	lsl	x14, x13, #3
	add	x12, x12, x14
	add	x10, x10, x14
	add	x11, x11, x14
	add	x9, x9, x13, lsl #2
	sub	x8, x8, x13
	movi	d0, #0000000000000000
Lloh0:
	adrp	x13, lCPI0_1@PAGE
Lloh1:
	ldr	d1, [x13, lCPI0_1@PAGEOFF]
LBB0_68:                                ; =>This Inner Loop Header: Depth=1
	str	d0, [x12], #8
	str	d1, [x10], #8
	str	d0, [x11], #8
	str	wzr, [x9], #4
	subs	x8, x8, #1
	b.ne	LBB0_68
LBB0_69:
	sub	w20, w23, w22
	mov	x0, x19
	bl	_sl_contact_step_end
	strb	wzr, [x19, #972]
	ldr	w8, [x19, #64]
	cbz	w8, LBB0_72
; %bb.70:
	ldr	x9, [x19, #112]
	cmp	w8, #4
	b.hi	LBB0_81
; %bb.71:
	mov	x10, #0                         ; =0x0
	mov	w11, #0                         ; =0x0
	b	LBB0_91
LBB0_72:
	mov	w11, #0                         ; =0x0
	b	LBB0_93
LBB0_73:
	mov	x13, #0                         ; =0x0
	lsl	x14, x8, #3
	add	x1, x12, x14
	add	x2, x10, x14
	add	x0, x9, x8, lsl #2
	add	x3, x11, x14
	cmp	x12, x3
	ccmp	x11, x1, #2, lo
	cset	w14, lo
	cmp	x12, x0
	ccmp	x9, x1, #2, lo
	cset	w15, lo
	cmp	x10, x3
	ccmp	x11, x2, #2, lo
	cset	w16, lo
	cmp	x10, x0
	ccmp	x9, x2, #2, lo
	cset	w17, lo
	cmp	x11, x0
	ccmp	x9, x3, #2, lo
	cset	w0, lo
	cmp	x10, x1
	ccmp	x12, x2, #2, lo
	b.lo	LBB0_67
; %bb.74:
	tbnz	w14, #0, LBB0_67
; %bb.75:
	tbnz	w15, #0, LBB0_67
; %bb.76:
	tbnz	w16, #0, LBB0_67
; %bb.77:
	tbnz	w17, #0, LBB0_67
; %bb.78:
	tbnz	w0, #0, LBB0_67
; %bb.79:
	cmp	w8, #16
	b.hs	LBB0_103
; %bb.80:
	mov	x13, #0                         ; =0x0
	b	LBB0_107
LBB0_81:
	cmp	w8, #17
	b.hs	LBB0_85
; %bb.82:
	mov	x10, #0                         ; =0x0
	mov	w11, #0                         ; =0x0
	b	LBB0_88
LBB0_83:
	adrp	x12, lCPI0_0@PAGE
	cmp	w8, #16
	b.hs	LBB0_96
; %bb.84:
	mov	x11, #0                         ; =0x0
	b	LBB0_100
LBB0_85:
	ands	x10, x8, #0xf
	mov	w11, #16                        ; =0x10
	csel	x12, x11, x10, eq
	sub	x10, x8, x12
	add	x11, x9, #232
	movi.2d	v0, #0000000000000000
	mov	x13, x8
	movi.2d	v1, #0000000000000000
	movi.2d	v2, #0000000000000000
	movi.2d	v3, #0000000000000000
LBB0_86:                                ; =>This Inner Loop Header: Depth=1
	sub	x14, x11, #196
	sub	x15, x11, #168
	sub	x16, x11, #140
	sub	x17, x11, #84
	sub	x0, x11, #56
	sub	x1, x11, #28
	add	x2, x11, #28
	add	x3, x11, #56
	ldur	s4, [x11, #-224]
	ld1.s	{ v4 }[1], [x14]
	add	x14, x11, #84
	ld1.s	{ v4 }[2], [x15]
	ld1.s	{ v4 }[3], [x16]
	ldur	s5, [x11, #-112]
	ld1.s	{ v5 }[1], [x17]
	ld1.s	{ v5 }[2], [x0]
	ld1.s	{ v5 }[3], [x1]
	add	x15, x11, #140
	ldr	s6, [x11]
	ld1.s	{ v6 }[1], [x2]
	ld1.s	{ v6 }[2], [x3]
	add	x16, x11, #168
	ld1.s	{ v6 }[3], [x14]
	ldr	s7, [x11, #112]
	ld1.s	{ v7 }[1], [x15]
	add	x14, x11, #196
	ld1.s	{ v7 }[2], [x16]
	ld1.s	{ v7 }[3], [x14]
	umax.4s	v0, v4, v0
	umax.4s	v1, v5, v1
	umax.4s	v2, v6, v2
	umax.4s	v3, v7, v3
	sub	x13, x13, #16
	add	x11, x11, #448
	cmp	x12, x13
	b.ne	LBB0_86
; %bb.87:
	umax.4s	v0, v0, v1
	umax.4s	v0, v0, v2
	umax.4s	v0, v0, v3
	umaxv.4s	s0, v0
	fmov	w11, s0
	cmp	x12, #5
	b.lo	LBB0_91
LBB0_88:
	mov	x12, x10
	ands	x10, x8, #0x3
	mov	w13, #4                         ; =0x4
	csel	x13, x13, x10, eq
	mov	w10, #28                        ; =0x1c
	madd	x14, x12, x10, x9
	sub	x10, x8, x13
	dup.4s	v0, w11
	add	x11, x14, #64
	add	x12, x13, x12
	sub	x12, x12, x8
LBB0_89:                                ; =>This Inner Loop Header: Depth=1
	sub	x13, x11, #28
	ldur	s1, [x11, #-56]
	ld1.s	{ v1 }[1], [x13]
	ld1.s	{ v1 }[2], [x11]
	add	x13, x11, #28
	ld1.s	{ v1 }[3], [x13]
	umax.4s	v0, v1, v0
	add	x11, x11, #112
	adds	x12, x12, #4
	b.ne	LBB0_89
; %bb.90:
	umaxv.4s	s0, v0
	fmov	w11, s0
LBB0_91:
	mov	w12, #28                        ; =0x1c
	madd	x9, x10, x12, x9
	add	x9, x9, #8
	sub	x10, x8, x10
LBB0_92:                                ; =>This Inner Loop Header: Depth=1
	ldr	w12, [x9], #28
	cmp	w12, w11
	csel	w11, w12, w11, hi
	subs	x10, x10, #1
	b.ne	LBB0_92
LBB0_93:
	ldr	w10, [x19, #204]
	ldr	w12, [x19, #456]
	ldr	w13, [x19, #180]
	ldr	x14, [sp]                       ; 8-byte Folded Reload
	ldp	q0, q1, [x14, #64]
	stp	q0, q1, [x19, #672]
	ldr	x9, [x14, #96]
	str	x9, [x19, #704]
	ldp	q0, q1, [x14]
	stp	q0, q1, [x19, #608]
	ldp	q1, q0, [x14, #32]
	stp	q1, q0, [x19, #640]
	ldr	w9, [sp, #44]                   ; 4-byte Folded Reload
	str	w9, [x19, #712]
	ldr	w9, [sp, #48]                   ; 4-byte Folded Reload
	str	w9, [x19, #716]
	str	w12, [x19, #720]
	str	w13, [x19, #724]
	str	w20, [x19, #728]
	str	w22, [x19, #732]
	str	w8, [x19, #736]
	str	w11, [x19, #740]
	str	w10, [x19, #744]
	str	wzr, [x19, #748]
LBB0_94:
	ldp	x29, x30, [sp, #208]            ; 16-byte Folded Reload
	ldp	x20, x19, [sp, #192]            ; 16-byte Folded Reload
	ldp	x22, x21, [sp, #176]            ; 16-byte Folded Reload
	ldp	x24, x23, [sp, #160]            ; 16-byte Folded Reload
	ldp	x26, x25, [sp, #144]            ; 16-byte Folded Reload
	ldp	x28, x27, [sp, #128]            ; 16-byte Folded Reload
	ldp	d9, d8, [sp, #112]              ; 16-byte Folded Reload
	ldp	d11, d10, [sp, #96]             ; 16-byte Folded Reload
	ldp	d13, d12, [sp, #80]             ; 16-byte Folded Reload
	ldp	d15, d14, [sp, #64]             ; 16-byte Folded Reload
	add	sp, sp, #224
LBB0_95:
	ret
LBB0_96:
	and	x11, x8, #0xfffffff0
	add	x13, x9, #64
	add	x14, x10, #64
	movi.2d	v0, #0000000000000000
	ldr	q1, [x12, lCPI0_0@PAGEOFF]
	mov	x15, x11
LBB0_97:                                ; =>This Inner Loop Header: Depth=1
	stp	q0, q0, [x14, #-64]
	stp	q0, q0, [x14, #-32]
	stp	q0, q0, [x14]
	stp	q0, q0, [x14, #32]
	stp	q1, q1, [x13, #-64]
	stp	q1, q1, [x13, #-32]
	stp	q1, q1, [x13]
	stp	q1, q1, [x13, #32]
	add	x13, x13, #128
	add	x14, x14, #128
	subs	x15, x15, #16
	b.ne	LBB0_97
; %bb.98:
	cmp	x11, x8
	adrp	x13, lCPI0_1@PAGE
	b.eq	LBB0_21
; %bb.99:
	tst	x8, #0xc
	b.eq	LBB0_19
LBB0_100:
	mov	x14, x11
	and	x11, x8, #0xfffffffc
	sub	x13, x14, x11
	lsl	x15, x14, #3
	add	x14, x9, x15
	add	x15, x10, x15
	movi.2d	v0, #0000000000000000
	ldr	q1, [x12, lCPI0_0@PAGEOFF]
LBB0_101:                               ; =>This Inner Loop Header: Depth=1
	stp	q0, q0, [x15], #32
	stp	q1, q1, [x14], #32
	adds	x13, x13, #4
	b.ne	LBB0_101
; %bb.102:
	cmp	x11, x8
	adrp	x13, lCPI0_1@PAGE
	b.ne	LBB0_19
	b	LBB0_21
LBB0_103:
	and	x13, x8, #0xfffffff0
	add	x14, x9, #32
	add	x15, x11, #64
	add	x16, x12, #64
	add	x17, x10, #64
	movi.2d	v0, #0000000000000000
Lloh2:
	adrp	x0, lCPI0_0@PAGE
Lloh3:
	ldr	q1, [x0, lCPI0_0@PAGEOFF]
	mov	x0, x13
LBB0_104:                               ; =>This Inner Loop Header: Depth=1
	stp	q0, q0, [x16, #-64]
	stp	q0, q0, [x16, #-32]
	stp	q0, q0, [x16]
	stp	q0, q0, [x16, #32]
	stp	q1, q1, [x17, #-64]
	stp	q1, q1, [x17, #-32]
	stp	q1, q1, [x17]
	stp	q1, q1, [x17, #32]
	stp	q0, q0, [x15, #-64]
	stp	q0, q0, [x15, #-32]
	stp	q0, q0, [x15]
	stp	q0, q0, [x15, #32]
	stp	q0, q0, [x14, #-32]
	stp	q0, q0, [x14], #64
	add	x15, x15, #128
	add	x16, x16, #128
	add	x17, x17, #128
	subs	x0, x0, #16
	b.ne	LBB0_104
; %bb.105:
	cmp	x13, x8
	b.eq	LBB0_69
; %bb.106:
	tst	x8, #0xc
	b.eq	LBB0_67
LBB0_107:
	mov	x16, x13
	and	x13, x8, #0xfffffffc
	sub	x14, x16, x13
	add	x15, x9, x16, lsl #2
	lsl	x0, x16, #3
	add	x16, x11, x0
	add	x17, x10, x0
	add	x0, x12, x0
	movi.2d	v0, #0000000000000000
Lloh4:
	adrp	x1, lCPI0_0@PAGE
Lloh5:
	ldr	q1, [x1, lCPI0_0@PAGEOFF]
LBB0_108:                               ; =>This Inner Loop Header: Depth=1
	stp	q0, q0, [x0], #32
	stp	q1, q1, [x17], #32
	stp	q0, q0, [x16], #32
	stp	xzr, xzr, [x15], #16
	adds	x14, x14, #4
	b.ne	LBB0_108
; %bb.109:
	cmp	x13, x8
	b.ne	LBB0_67
	b	LBB0_69
	.loh AdrpLdr	Lloh0, Lloh1
	.loh AdrpLdr	Lloh2, Lloh3
	.loh AdrpLdr	Lloh4, Lloh5
	.cfi_endproc
                                        ; -- End function
	.p2align	2                               ; -- Begin function linear_velocity_cap
_linear_velocity_cap:                   ; @linear_velocity_cap
	.cfi_startproc
; %bb.0:
	fabs	s3, s0
	fabs	s4, s1
	fcmp	s3, s4
	fcsel	s3, s3, s4, gt
	fcmp	s3, #0.0
	b.ne	LBB1_2
LBB1_1:
	ret
LBB1_2:
	fmov	s4, #1.00000000
	fdiv	s4, s4, s3
	fmul	s5, s0, s4
	fmul	s4, s1, s4
	fmul	s4, s4, s4
	fmadd	s4, s5, s5, s4
	fsqrt	s4, s4
	fdiv	s5, s2, s4
	fcmp	s3, s5
	b.ls	LBB1_1
; %bb.3:
	fdiv	s2, s2, s3
	fdiv	s2, s2, s4
	fmul	s0, s0, s2
	fmul	s1, s1, s2
	ret
	.cfi_endproc
                                        ; -- End function
	.p2align	2                               ; -- Begin function sl_rotation_integrate
_sl_rotation_integrate:                 ; @sl_rotation_integrate
	.cfi_startproc
; %bb.0:
	fmov	w8, s2
	and	w8, w8, #0x7fffffff
	mov	w9, #2139095039                 ; =0x7f7fffff
	cmp	w8, w9
	b.gt	LBB2_4
; %bb.1:
	fabs	s3, s0
	fabs	s4, s1
	fcmp	s3, s4
	fcsel	s5, s3, s4, gt
	movi.2d	v4, #0000000000000000
	fmov	s3, #1.00000000
	mov	w8, #14269                      ; =0x37bd
	movk	w8, #13702, lsl #16
	fmov	s6, w8
	fcmp	s5, s6
	b.ls	LBB2_5
; %bb.2:
	mov	w8, #2139095040                 ; =0x7f800000
	fmov	s4, w8
	fcmp	s5, s4
	movi.2d	v4, #0000000000000000
	b.eq	LBB2_5
	b.vs	LBB2_5
; %bb.3:
	fcmp	s2, #0.0
	b.ne	LBB2_6
LBB2_4:
	mov.16b	v3, v0
	mov.16b	v4, v1
LBB2_5:
	mov.16b	v0, v3
	mov.16b	v1, v4
	ret
LBB2_6:
	fabs	s3, s2
	fmov	s4, #1.00000000
	fcmp	s3, s4
	fcsel	s3, s4, s3, mi
	fdiv	s3, s4, s3
	fmul	s4, s2, s3
	fnmul	s2, s4, s1
	fmadd	s2, s0, s3, s2
	fmov	w8, s2
	fmul	s4, s0, s4
	fmadd	s5, s1, s3, s4
	fmov	w9, s5
	fabs	s3, s2
	fabs	s4, s5
	fcmp	s3, s4
	fcsel	s6, s3, s4, gt
	and	w10, w8, #0x7fffffff
	and	w9, w9, #0x7fffffff
	mov	w8, #2139095040                 ; =0x7f800000
	cmp	w9, w8
	cset	w9, lt
	mov	w11, #2139095039                ; =0x7f7fffff
	cmp	w10, w11
	ccmp	w9, #0, #4, le
	mov	w9, #14269                      ; =0x37bd
	movk	w9, #13702, lsl #16
	fmov	s3, w9
	fccmp	s6, s3, #0, ne
	mov.16b	v3, v0
	mov.16b	v4, v1
	b.ls	LBB2_5
; %bb.7:
	fdiv	s0, s2, s6
	fdiv	s1, s5, s6
	fmul	s2, s1, s1
	fmadd	s2, s0, s0, s2
	fsqrt	s2, s2
	fmov	w9, s2
	and	w9, w9, #0x7fffffff
	cmp	w9, w8
	cset	w8, lt
	mov	w9, #14269                      ; =0x37bd
	movk	w9, #13702, lsl #16
	fmov	s3, w9
	fcmp	s2, s3
	ccmp	w8, #0, #4, hi
	fmov	s3, #1.00000000
	movi.2d	v4, #0000000000000000
	b.eq	LBB2_5
; %bb.8:
	fdiv	s3, s0, s2
	fdiv	s4, s1, s2
	mov.16b	v0, v3
	mov.16b	v1, v4
	ret
	.cfi_endproc
                                        ; -- End function
	.p2align	2                               ; -- Begin function sl_rotation_normalize
_sl_rotation_normalize:                 ; @sl_rotation_normalize
	.cfi_startproc
; %bb.0:
	mov.16b	v2, v1
	mov.16b	v3, v0
	fabs	s0, s0
	fabs	s1, s1
	fcmp	s0, s1
	fcsel	s4, s0, s1, gt
	movi.2d	v1, #0000000000000000
	fmov	s0, #1.00000000
	mov	w8, #14269                      ; =0x37bd
	movk	w8, #13702, lsl #16
	fmov	s5, w8
	fcmp	s4, s5
	b.ls	LBB3_4
; %bb.1:
	mov	w8, #2139095040                 ; =0x7f800000
	fmov	s5, w8
	fcmp	s4, s5
	b.eq	LBB3_4
	b.vs	LBB3_4
; %bb.2:
	fdiv	s3, s3, s4
	fdiv	s2, s2, s4
	fmul	s4, s2, s2
	fmadd	s4, s3, s3, s4
	fsqrt	s4, s4
	fmov	w9, s4
	and	w9, w9, #0x7fffffff
	cmp	w9, w8
	cset	w8, lt
	mov	w9, #14269                      ; =0x37bd
	movk	w9, #13702, lsl #16
	fmov	s5, w9
	fcmp	s4, s5
	ccmp	w8, #0, #4, hi
	b.eq	LBB3_4
; %bb.3:
	fdiv	s0, s3, s4
	fdiv	s1, s2, s4
LBB3_4:
	ret
	.cfi_endproc
                                        ; -- End function
	.globl	_sl_stepper_init                ; -- Begin function sl_stepper_init
	.p2align	2
_sl_stepper_init:                       ; @sl_stepper_init
	.cfi_startproc
; %bb.0:
	fmov	w8, s0
	sub	w9, w8, #1
	and	w10, w8, #0x7fffffff
	sub	w10, w10, #2048, lsl #12        ; =8388608
	lsr	w10, w10, #24
	cmp	w10, #127
	ccmn	w8, #1, #4, lo
	mov	w8, #8388607                    ; =0x7fffff
	ccmp	w9, w8, #0, le
	cset	w8, lo
	b.hs	LBB4_2
; %bb.1:
	str	s0, [x0]
	str	wzr, [x0, #4]
	mov	x0, x8
	ret
LBB4_2:
	str	xzr, [x0]
	mov	x0, x8
	ret
	.cfi_endproc
                                        ; -- End function
	.globl	_sl_world_advance               ; -- Begin function sl_world_advance
	.p2align	2
_sl_world_advance:                      ; @sl_world_advance
	.cfi_startproc
; %bb.0:
	stp	d9, d8, [sp, #-48]!             ; 16-byte Folded Spill
	stp	x20, x19, [sp, #16]             ; 16-byte Folded Spill
	stp	x29, x30, [sp, #32]             ; 16-byte Folded Spill
	add	x29, sp, #32
	.cfi_def_cfa w29, 16
	.cfi_offset w30, -8
	.cfi_offset w29, -16
	.cfi_offset w19, -24
	.cfi_offset w20, -32
	.cfi_offset b8, -40
	.cfi_offset b9, -48
	mov.16b	v1, v0
	mov	x19, x1
	ldp	s0, s2, [x1]
	fadd	s3, s1, s2
	fmov	w8, s1
	sub	w9, w8, #1
	and	w10, w8, #0x7fffffff
	sub	w10, w10, #2048, lsl #12        ; =8388608
	lsr	w10, w10, #24
	cmn	w8, #1
	mov	w8, #127                        ; =0x7f
	ccmp	w10, w8, #2, gt
	mov	w8, #8388607                    ; =0x7fffff
	ccmp	w9, w8, #0, hs
	fcsel	s8, s3, s2, lo
	fcmp	s8, s0
	b.ge	LBB5_2
; %bb.1:
	mov	w0, #0                          ; =0x0
	str	s8, [x19, #4]
	ldp	x29, x30, [sp, #32]             ; 16-byte Folded Reload
	ldp	x20, x19, [sp, #16]             ; 16-byte Folded Reload
	ldp	d9, d8, [sp], #48               ; 16-byte Folded Reload
	ret
LBB5_2:
	mov	x20, x0
	bl	_sl_world_step
	ldr	s0, [x19]
	fsub	s8, s8, s0
	fcmp	s8, s0
	b.ge	LBB5_4
; %bb.3:
	mov	w0, #1                          ; =0x1
	str	s8, [x19, #4]
	ldp	x29, x30, [sp, #32]             ; 16-byte Folded Reload
	ldp	x20, x19, [sp, #16]             ; 16-byte Folded Reload
	ldp	d9, d8, [sp], #48               ; 16-byte Folded Reload
	ret
LBB5_4:
	mov	x0, x20
	bl	_sl_world_step
	ldr	s0, [x19]
	fsub	s8, s8, s0
	fcmp	s8, s0
	b.ge	LBB5_6
; %bb.5:
	mov	w0, #2                          ; =0x2
	str	s8, [x19, #4]
	ldp	x29, x30, [sp, #32]             ; 16-byte Folded Reload
	ldp	x20, x19, [sp, #16]             ; 16-byte Folded Reload
	ldp	d9, d8, [sp], #48               ; 16-byte Folded Reload
	ret
LBB5_6:
	mov	x0, x20
	bl	_sl_world_step
	ldr	s0, [x19]
	fsub	s8, s8, s0
	fcmp	s8, s0
	b.ge	LBB5_8
; %bb.7:
	mov	w0, #3                          ; =0x3
	str	s8, [x19, #4]
	ldp	x29, x30, [sp, #32]             ; 16-byte Folded Reload
	ldp	x20, x19, [sp, #16]             ; 16-byte Folded Reload
	ldp	d9, d8, [sp], #48               ; 16-byte Folded Reload
	ret
LBB5_8:
	mov	x0, x20
	bl	_sl_world_step
	ldr	s0, [x19]
	fsub	s8, s8, s0
	fcmp	s8, s0
	b.ge	LBB5_10
; %bb.9:
	mov	w0, #4                          ; =0x4
	str	s8, [x19, #4]
	ldp	x29, x30, [sp, #32]             ; 16-byte Folded Reload
	ldp	x20, x19, [sp, #16]             ; 16-byte Folded Reload
	ldp	d9, d8, [sp], #48               ; 16-byte Folded Reload
	ret
LBB5_10:
	mov	x0, x20
	bl	_sl_world_step
	ldr	s0, [x19]
	fsub	s8, s8, s0
	fcmp	s8, s0
	b.ge	LBB5_12
; %bb.11:
	mov	w0, #5                          ; =0x5
	str	s8, [x19, #4]
	ldp	x29, x30, [sp, #32]             ; 16-byte Folded Reload
	ldp	x20, x19, [sp, #16]             ; 16-byte Folded Reload
	ldp	d9, d8, [sp], #48               ; 16-byte Folded Reload
	ret
LBB5_12:
	mov	x0, x20
	bl	_sl_world_step
	ldr	s0, [x19]
	fsub	s8, s8, s0
	fcmp	s8, s0
	b.ge	LBB5_14
; %bb.13:
	mov	w0, #6                          ; =0x6
	str	s8, [x19, #4]
	ldp	x29, x30, [sp, #32]             ; 16-byte Folded Reload
	ldp	x20, x19, [sp, #16]             ; 16-byte Folded Reload
	ldp	d9, d8, [sp], #48               ; 16-byte Folded Reload
	ret
LBB5_14:
	mov	x0, x20
	bl	_sl_world_step
	ldr	s0, [x19]
	fsub	s8, s8, s0
	fcmp	s8, s0
	b.ge	LBB5_16
; %bb.15:
	mov	w0, #7                          ; =0x7
	str	s8, [x19, #4]
	ldp	x29, x30, [sp, #32]             ; 16-byte Folded Reload
	ldp	x20, x19, [sp, #16]             ; 16-byte Folded Reload
	ldp	d9, d8, [sp], #48               ; 16-byte Folded Reload
	ret
LBB5_16:
	mov	x0, x20
	bl	_sl_world_step
	movi.2d	v8, #0000000000000000
	mov	w0, #8                          ; =0x8
	str	s8, [x19, #4]
	ldp	x29, x30, [sp, #32]             ; 16-byte Folded Reload
	ldp	x20, x19, [sp, #16]             ; 16-byte Folded Reload
	ldp	d9, d8, [sp], #48               ; 16-byte Folded Reload
	ret
	.cfi_endproc
                                        ; -- End function
.subsections_via_symbols
