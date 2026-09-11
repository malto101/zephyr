/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * HVM guest-mode event vectors.
 *
 * Zephyr runs as a guest OS under the Hexagon Virtual Machine (HVM).
 * The hypervisor dispatches events to the guest via the Guest Event
 * Vector Base (GEVB) table set by the vmsetvec trap1 call.
 *
 * GEVB layout (offsets from base):
 *   0  Reset
 *   1  Machine check
 *   2  General exception
 *   3  Debug
 *   5  Trap0 (syscall)
 *   7  Interrupt
 */

#include <zephyr/toolchain.h>
#include <zephyr/linker/sections.h>
#include <hexagon_vm.h>

GTEXT(__start)
GTEXT(__reset_handler)
GTEXT(_machine_check_handler)
GTEXT(_general_exception_handler)
GTEXT(_trap0_handler)
GTEXT(_interrupt_handler)
#ifdef CONFIG_MMU
GTEXT(hexagon_mmu_init)
#endif

/*
 * Guest Event Vector Base (GEVB) table.
 *
 * The hypervisor dispatches to PC = GEVB + offset.  Each entry is a
 * single 4-byte jump instruction (packet).
 *
 *   Offset  Event
 *   ------  -----
 *   0       Reset
 *   1       Machine check
 *   2       General exception
 *   3       Debug
 *   4       (reserved)
 *   5       Trap0 (syscall)
 *   6       (reserved)
 *   7       Interrupt
 */
SECTION_FUNC(reset, __start)
	.p2align 2
	/* 0: Reset */
	jump #__reset_handler
	/* 1: Machine check */
	jump #_machine_check_handler
	/* 2: General exception */
	jump #_general_exception_handler
	/* 3: Debug */
	jump #_unhandled_event
	/* 4: Reserved */
	jump #_unhandled_event
	/* 5: Trap0 (syscall) */
	jump #_trap0_handler
	/* 6: Reserved */
	jump #_unhandled_event
	/* 7: Interrupt */
	jump #_interrupt_handler


/* Main reset handler */
SECTION_FUNC(TEXT, __reset_handler)
	/*
	 * Set the guest event vector base via the HVM vmsetvec call.
	 * Under H2, trap1 is handled by the hypervisor which sets GEVB.
	 */
	r0 = ##__start
	trap1(#HEXAGON_VM_vmsetvec)

	/*
	 * Set up initial stack pointer -- must be done before calling
	 * hexagon_mmu_init() below, which is a C function and needs a
	 * valid (guest-accessible) stack for its own prologue.  Under H2,
	 * r29 initially points to the boot-VM's stack which is outside the
	 * guest's fence range.
	 */
	r29 = ##_interrupt_stack_end

	/* Clear frame pointer */
	r30 = #0

	/*
	 * Initialize BSS section BEFORE page table setup.
	 * The page table (hexagon_pgd/hexagon_l2 in mem_manage.c) lives in
	 * BSS, so we must zero it first, then let hexagon_mmu_init() write
	 * the PDE/PTE entries and call vmnewmap. This runs under the
	 * initial linear mapping provided by H2.
	 */
	r0 = ##__bss_start
	r1 = ##__bss_end
	r2 = #0

	/* BSS clear using word stores (BSS end is ALIGN(4)) */
	r3 = sub(r1, r0)
	r3 = lsr(r3, #2)        /* number of 4-byte words */
	p0 = cmp.gt(r3, #0)
	if (!p0) jump #_bss_done

	loop0(_bss_loop, r3)
_bss_loop:
	{
		memw(r0++#4) = r2
	} :endloop0

_bss_done:

#ifdef CONFIG_MMU
	/* MMU mappings setup -- must follow BSS init (page table is in BSS) */
	call ##hexagon_mmu_init
#endif

	/* Jump to C initialization */
	call ##z_prep_c

	/* Should never reach here */
	jump #_hang


/* Event vector table marker */
SECTION_FUNC(event_vectors, _event_vector_table)

SECTION_FUNC(TEXT, _unhandled_event)
	r0 = #0xFF
	jump ##z_hexagon_fatal_error

/*
 * Route through the enhanced handler, like every other real event below,
 * so a machine check gets the same full volatile-context save as any
 * other event before z_hexagon_event_handler() calls z_hexagon_fatal_error()
 * (irq_manage.c) -- jumping to z_hexagon_fatal_error() directly here, as
 * an earlier version of this file did, left the interrupted register
 * state nowhere for a postmortem dump or gdbstub to read back.
 */
SECTION_FUNC(TEXT, _machine_check_handler)
	jump ##z_hexagon_enhanced_machine_check

/*
 * General exception handler.
 * Route through the enhanced handler which saves full volatile context,
 * calls z_hexagon_exception_handler(), and handles preemption on exit.
 */
SECTION_FUNC(TEXT, _general_exception_handler)
	jump ##z_hexagon_enhanced_exception

/*
 * Trap0 (syscall) handler.
 * Route through the enhanced handler so that trap0 events get full
 * context save/restore.  The C handler z_hexagon_trap0_handler()
 * dispatches based on r6 (syscall number).
 */
SECTION_FUNC(TEXT, _trap0_handler)
	jump ##z_hexagon_enhanced_trap0

SECTION_FUNC(TEXT, _interrupt_handler)
	jump ##z_hexagon_enhanced_interrupt

_hang:
	jump #_hang
