/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Hexagon event context saved by the assembly EVENT_ENTRY macro.
 *
 * This structure must match the register save layout in event_handlers.S
 * exactly.  It is used by both the C event handler (irq_manage.c) and the
 * GDB stub (gdbstub.c).  Defining it in one place prevents silent
 * divergence between the two users.
 *
 * The offsets below correspond to the EVENT_CTX_* constants in
 * event_handlers.S:
 *
 *   0x00  r0_r1
 *   0x08  r2_r3
 *   0x10  r4_r5
 *   0x18  r6_r7
 *   0x20  r8_r9
 *   0x28  r10_r11
 *   0x30  r12_r13
 *   0x38  r14_r15
 *   0x40  pred_regs
 *   0x44  link_reg
 *   0x48  gelr
 *   0x4c  gsr
 *   0x50  sa0
 *   0x54  lc0
 *   0x58  sa1
 *   0x5c  lc1
 *   0x60  m0
 *   0x64  m1
 *   0x68  usr
 *   0x6c  r28
 *   0x70  scratch
 *   -- 0x78 (EVENT_CTX_SIZE without CONFIG_GDBSTUB) --
 *
 * With CONFIG_GDBSTUB, EVENT_ENTRY additionally saves the callee-saved
 * GPRs and GP/UGP so the debug stub can present the full register file
 * (see the field comment below for why):
 *
 *   0x78  r16_r17
 *   0x80  r18_r19
 *   0x88  r20_r21
 *   0x90  r22_r23
 *   0x98  r24_r25
 *   0xa0  r26_r27
 *   0xa8  gp
 *   0xac  ugp
 *   -- total size 0xb0 (EVENT_CTX_SIZE with CONFIG_GDBSTUB) --
 */

#ifndef ZEPHYR_ARCH_HEXAGON_INCLUDE_EVENT_CONTEXT_H_
#define ZEPHYR_ARCH_HEXAGON_INCLUDE_EVENT_CONTEXT_H_

#ifndef _ASMLANGUAGE

#include <zephyr/types.h>

/** @brief Volatile register context saved on the stack by EVENT_ENTRY. */
struct event_context {
	uint32_t r0_r1[2];         /**< r0, r1 */
	uint32_t r2_r3[2];         /**< r2, r3 */
	uint32_t r4_r5[2];         /**< r4, r5 */
	uint32_t r6_r7[2];         /**< r6 (syscall num), r7 */
	uint32_t r8_r9[2];         /**< r8, r9 */
	uint32_t r10_r11[2];       /**< r10, r11 */
	uint32_t r12_r13[2];       /**< r12, r13 */
	uint32_t r14_r15[2];       /**< r14, r15 */
	uint32_t pred_regs;        /**< P3:0 packed as one 32-bit word */
	uint32_t link_reg;         /**< R31 (LR) */
	uint32_t gelr;             /**< GELR: return PC (G0 at event entry) */
	uint32_t gsr;              /**< GSR: guest status (G1 at event entry) */
	uint32_t sa0;              /**< hardware loop SA0 */
	uint32_t lc0;              /**< hardware loop LC0 */
	uint32_t sa1;              /**< hardware loop SA1 */
	uint32_t lc1;              /**< hardware loop LC1 */
	uint32_t m0;               /**< modifier register M0 */
	uint32_t m1;               /**< modifier register M1 */
	uint32_t usr;              /**< user status register USR */
	uint32_t r28;              /**< R28 (caller-saved, not in r0-r15 pairs) */
	uint32_t scratch;          /**< temporary slot used during EVENT_EXIT */
#ifdef CONFIG_GDBSTUB
	/*
	 * Callee-saved GPRs and GP/UGP, saved only for CONFIG_GDBSTUB.
	 *
	 * Every other event handler leaves r16-r27 alone: they are true
	 * ABI callee-saved registers, so any C function the handler calls
	 * already preserves the interrupted code's values in its own
	 * prologue/epilogue without this file's help. The debug stub is
	 * different -- it needs to *read* those values from deep inside
	 * the C handler chain (irq_manage.c -> gdbstub.c), and by then a
	 * callee may already have repurposed the physical register for
	 * its own locals; the interrupted value only still exists in that
	 * callee's own stack slot, which nothing outside it can address.
	 * EVENT_ENTRY runs before any C code does, so this is the only
	 * point they are guaranteed to still hold the trap-time values.
	 */
	uint32_t r16_r17[2];       /**< r16, r17 */
	uint32_t r18_r19[2];       /**< r18, r19 */
	uint32_t r20_r21[2];       /**< r20, r21 */
	uint32_t r22_r23[2];       /**< r22, r23 */
	uint32_t r24_r25[2];       /**< r24, r25 */
	uint32_t r26_r27[2];       /**< r26, r27 */
	uint32_t gp;               /**< global pointer GP */
	uint32_t ugp;              /**< user global pointer UGP */
#endif
};

/** @brief Size of struct event_context in bytes (must equal EVENT_CTX_SIZE). */
#define EVENT_CTX_SIZE_C sizeof(struct event_context)

/*
 * EVENT_CTX_SIZE as a numeric constant matching the assembler definition in
 * event_handlers.S.  Used by gdbstub.c to locate the pre-exception SP and FP
 * above the saved context frame.
 *
 * The assembly frame body is EVENT_CTX_SIZE bytes, then allocframe appends
 * 8 bytes for FP:LR (the frame chain).  struct event_context occupies the
 * first sizeof(struct event_context) bytes of the frame body; the remaining
 * bytes (if any) are unused padding for alignment.
 *
 * If struct event_context grows beyond EVENT_CTX_SIZE, update EVENT_CTX_SIZE
 * here and in event_handlers.S (keep 8-byte aligned).
 */
#ifdef CONFIG_GDBSTUB
#define EVENT_CTX_SIZE 0xb0
#else
#define EVENT_CTX_SIZE 0x78
#endif

/*
 * Overhead appended by allocframe above EVENT_CTX_SIZE: the old FP and LR
 * are stored as a pair (8 bytes) immediately above the event context body.
 * Pre-exception SP = (uint8_t *)ctx + EVENT_CTX_SIZE + EVENT_ENTRY_ALLOCFRAME_OVERHEAD.
 * If EVENT_ENTRY ever changes its frame layout, update this constant and
 * add a BUILD_ASSERT in gdbstub.c to catch the mismatch.
 */
#define EVENT_ENTRY_ALLOCFRAME_OVERHEAD 8

BUILD_ASSERT(EVENT_CTX_SIZE_C <= EVENT_CTX_SIZE,
	     "struct event_context exceeds EVENT_CTX_SIZE; "
	     "update both event_context.h and event_handlers.S");

#endif /* _ASMLANGUAGE */

#endif /* ZEPHYR_ARCH_HEXAGON_INCLUDE_EVENT_CONTEXT_H_ */
