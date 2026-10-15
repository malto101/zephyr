/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Native v73 event vectors.
 *
 * The native table has sixteen architectural event slots followed by the
 * thirty-two interrupt slots used by the v66-and-later Hexagon ABI.  On v73,
 * interrupt-relative slot 2 is the L2VIC entry; the other slots are native
 * kernel or miscellaneous events and must not be treated as L2VIC sources.
 */

#include <zephyr/toolchain.h>
#include <zephyr/linker/sections.h>
#include <native_event_context.h>

GTEXT(__start)
GTEXT(__reset_handler)
GTEXT(_native_fatal_event)
GTEXT(_native_interrupt_handler)
GTEXT(_hang)
GTEXT(z_prep_c)
GTEXT(z_hexagon_fatal_error)
GTEXT(z_hexagon_native_interrupt_dispatch)
GTEXT(z_lemans_native_early_startup)
GTEXT(z_hexagon_native_record_fault)

/*
 * The first sixteen entries are the architectural event slots.  The next
 * thirty-two entries are the native interrupt slots; slot 2 in this group is
 * the L2VIC input on v73 and all slots use the same Zephyr demultiplexer for
 * now.
 */
SECTION_FUNC(event_vectors, _event_vector_table)
	.p2align 8
	/* 0-15: architectural events. */
	jump #__reset_handler
	jump #_native_fatal_event       /* architectural event 1 */
	jump #_native_fatal_event       /* architectural event 2 */
	jump #_native_fatal_event       /* architectural event 3 */
	jump #_native_fatal_event       /* architectural event 4 */
	jump #_native_fatal_event       /* architectural event 5 */
	jump #_native_fatal_event       /* architectural event 6 */
	jump #_native_fatal_event       /* architectural event 7 */
	jump #_native_fatal_event       /* architectural event 8 */
	jump #_native_fatal_event       /* architectural event 9 */
	jump #_native_fatal_event       /* architectural event 10 */
	jump #_native_fatal_event       /* architectural event 11 */
	jump #_native_fatal_event       /* architectural event 12 */
	jump #_native_fatal_event       /* architectural event 13 */
	jump #_native_fatal_event       /* architectural event 14 */
	jump #_native_fatal_event       /* architectural event 15 */

	/* 16-47: v66-v79 native interrupt events. */
	jump #_native_fatal_event       /* 0: miscellaneous */
	jump #_native_fatal_event       /* 1: miscellaneous */
	jump #_native_interrupt_handler /* 2: L2VIC */
	jump #_native_fatal_event       /* 3: miscellaneous */
	jump #_native_fatal_event       /* 4: miscellaneous */
	jump #_native_fatal_event       /* 5: miscellaneous */
	jump #_native_fatal_event       /* 6: miscellaneous */
	jump #_native_fatal_event       /* 7: IPI, unsupported in the first image */
	jump #_native_fatal_event       /* 8: IPI, unsupported in the first image */
	jump #_native_fatal_event       /* 9: IPI, unsupported in the first image */
	jump #_native_fatal_event       /* 10: IPI, unsupported in the first image */
	jump #_native_fatal_event       /* 11: IPI, unsupported in the first image */
	jump #_native_fatal_event       /* 12: IPI, unsupported in the first image */
	jump #_native_fatal_event       /* 13: miscellaneous */
	jump #_native_fatal_event       /* 14: miscellaneous */
	jump #_native_fatal_event       /* 15: reschedule, unsupported in the first image */
	jump #_native_fatal_event       /* 16: miscellaneous */
	jump #_native_fatal_event       /* 17: miscellaneous */
	jump #_native_fatal_event       /* 18: miscellaneous */
	jump #_native_fatal_event       /* 19: miscellaneous */
	jump #_native_fatal_event       /* 20: miscellaneous */
	jump #_native_fatal_event       /* 21: miscellaneous */
	jump #_native_fatal_event       /* 22: miscellaneous */
	jump #_native_fatal_event       /* 23: miscellaneous */
	jump #_native_fatal_event       /* 24: miscellaneous */
	jump #_native_fatal_event       /* 25: miscellaneous */
	jump #_native_fatal_event       /* 26: miscellaneous */
	jump #_native_fatal_event       /* 27: miscellaneous */
	jump #_native_fatal_event       /* 28: miscellaneous */
	jump #_native_fatal_event       /* 29: miscellaneous */
	jump #_native_fatal_event       /* 30: miscellaneous */
	jump #_native_fatal_event       /* 31: miscellaneous */

/* Native reset entry. */
SECTION_FUNC(reset, __start)
	.p2align 2
	jump #__reset_handler

SECTION_FUNC(TEXT, __reset_handler)
	/* Install the native event-vector base before enabling any interrupt. */
	r0 = ##_event_vector_table
	evb = r0

	/* Start with native exceptions and interrupts disabled. */
	r0 = #0
	ssr = r0
	isync

	/* Establish the stack before using a call or touching BSS. */
	r29 = ##_interrupt_stack_end
	r30 = #0

	/* Disable the hardware watchdog before the lengthy C startup path. */
	call ##z_lemans_native_early_startup

	/* Clear BSS before entering C code. */
	r0 = ##__bss_start
	r1 = ##__bss_end
	r2 = #0
	r3 = sub(r1, r0)
	r3 = lsr(r3, #2)
	p0 = cmp.gt(r3, #0)
	if (!p0) jump #_bss_done

	loop0(_bss_loop, r3)
_bss_loop:
	{
		memw(r0++#4) = r2
	} :endloop0

_bss_done:
	call ##z_prep_c

_hang:
	jump #_hang

/* Events not yet supported by the native backend are fatal. */
SECTION_FUNC(TEXT, _native_fatal_event)
	/* SSR.CAUSE identifies the architectural event that reached this path. */
	r0 = ssr
	r0 = and(r0, #0xff)
	call ##z_hexagon_native_record_fault
	r0 = #0xff
	jump ##z_hexagon_fatal_error

/*
 * Native interrupt entry.
 *
 * allocframe preserves the interrupted frame pointer and link register.  The
 * remaining caller-saved state and native return registers are saved here so
 * the C L2VIC dispatcher can freely use the normal C ABI.  The saved SSR and
 * ELR are restored before rte returns to the interrupted context.
 */
SECTION_FUNC(TEXT, _native_interrupt_handler)
	allocframe(#NATIVE_EVENT_CTX_SIZE)

	memd(r29+#NATIVE_EVENT_CTX_R0_R1) = r1:0
	memd(r29+#NATIVE_EVENT_CTX_R2_R3) = r3:2
	memd(r29+#NATIVE_EVENT_CTX_R4_R5) = r5:4
	memd(r29+#NATIVE_EVENT_CTX_R6_R7) = r7:6
	memd(r29+#NATIVE_EVENT_CTX_R8_R9) = r9:8
	memd(r29+#NATIVE_EVENT_CTX_R10_R11) = r11:10
	memd(r29+#NATIVE_EVENT_CTX_R12_R13) = r13:12
	memd(r29+#NATIVE_EVENT_CTX_R14_R15) = r15:14

	r0 = p3:0
	memw(r29+#NATIVE_EVENT_CTX_PRED) = r0
	r0 = r31
	memw(r29+#NATIVE_EVENT_CTX_LR) = r0
	r0 = r28
	memw(r29+#NATIVE_EVENT_CTX_R28) = r0
	r0 = ssr
	memw(r29+#NATIVE_EVENT_CTX_SSR) = r0
	/* Keep the native dispatcher non-nestable until exit semantics exist. */
	r1 = clrbit(r0, #18)
	ssr = r1
	isync
	r0 = elr
	memw(r29+#NATIVE_EVENT_CTX_ELR) = r0
	r0 = sa0
	memw(r29+#NATIVE_EVENT_CTX_SA0) = r0
	r0 = lc0
	memw(r29+#NATIVE_EVENT_CTX_LC0) = r0
	r0 = sa1
	memw(r29+#NATIVE_EVENT_CTX_SA1) = r0
	r0 = lc1
	memw(r29+#NATIVE_EVENT_CTX_LC1) = r0
	r0 = m0
	memw(r29+#NATIVE_EVENT_CTX_M0) = r0
	r0 = m1
	memw(r29+#NATIVE_EVENT_CTX_M1) = r0
	r0 = usr
	memw(r29+#NATIVE_EVENT_CTX_USR) = r0

	/* Save callee-saved GPRs and native global state. */
	memd(r29+#NATIVE_EVENT_CTX_R16_R17) = r17:16
	memd(r29+#NATIVE_EVENT_CTX_R18_R19) = r19:18
	memd(r29+#NATIVE_EVENT_CTX_R20_R21) = r21:20
	memd(r29+#NATIVE_EVENT_CTX_R22_R23) = r23:22
	memd(r29+#NATIVE_EVENT_CTX_R24_R25) = r25:24
	memd(r29+#NATIVE_EVENT_CTX_R26_R27) = r27:26
	r0 = g2
	memw(r29+#NATIVE_EVENT_CTX_G2) = r0
	r0 = gp
	memw(r29+#NATIVE_EVENT_CTX_GP) = r0
	r0 = ugp
	memw(r29+#NATIVE_EVENT_CTX_UGP) = r0

	call ##z_hexagon_native_interrupt_dispatch

	/* Restore native global state before restoring the GPR pairs. */
	r0 = memw(r29+#NATIVE_EVENT_CTX_G2)
	g2 = r0
	r0 = memw(r29+#NATIVE_EVENT_CTX_GP)
	gp = r0
	r0 = memw(r29+#NATIVE_EVENT_CTX_UGP)
	ugp = r0
	r27:26 = memd(r29+#NATIVE_EVENT_CTX_R26_R27)
	r25:24 = memd(r29+#NATIVE_EVENT_CTX_R24_R25)
	r23:22 = memd(r29+#NATIVE_EVENT_CTX_R22_R23)
	r21:20 = memd(r29+#NATIVE_EVENT_CTX_R20_R21)
	r19:18 = memd(r29+#NATIVE_EVENT_CTX_R18_R19)
	r17:16 = memd(r29+#NATIVE_EVENT_CTX_R16_R17)

	r0 = memw(r29+#NATIVE_EVENT_CTX_SA0)
	sa0 = r0
	r0 = memw(r29+#NATIVE_EVENT_CTX_LC0)
	lc0 = r0
	r0 = memw(r29+#NATIVE_EVENT_CTX_SA1)
	sa1 = r0
	r0 = memw(r29+#NATIVE_EVENT_CTX_LC1)
	lc1 = r0
	r0 = memw(r29+#NATIVE_EVENT_CTX_M0)
	m0 = r0
	r0 = memw(r29+#NATIVE_EVENT_CTX_M1)
	m1 = r0
	r0 = memw(r29+#NATIVE_EVENT_CTX_USR)
	usr = r0

	r28 = memw(r29+#NATIVE_EVENT_CTX_R28)
	r0 = memw(r29+#NATIVE_EVENT_CTX_PRED)
	p3:0 = r0
	r31 = memw(r29+#NATIVE_EVENT_CTX_LR)

	r15:14 = memd(r29+#NATIVE_EVENT_CTX_R14_R15)
	r13:12 = memd(r29+#NATIVE_EVENT_CTX_R12_R13)
	r11:10 = memd(r29+#NATIVE_EVENT_CTX_R10_R11)
	r9:8 = memd(r29+#NATIVE_EVENT_CTX_R8_R9)
	r7:6 = memd(r29+#NATIVE_EVENT_CTX_R6_R7)
	r5:4 = memd(r29+#NATIVE_EVENT_CTX_R4_R5)
	r3:2 = memd(r29+#NATIVE_EVENT_CTX_R2_R3)
	r1:0 = memd(r29+#NATIVE_EVENT_CTX_R0_R1)

	/* Restore native return state only after all other state is restored. */
	r0 = memw(r29+#NATIVE_EVENT_CTX_SSR)
	ssr = r0
	r0 = memw(r29+#NATIVE_EVENT_CTX_ELR)
	elr = r0

	deallocframe
	rte
