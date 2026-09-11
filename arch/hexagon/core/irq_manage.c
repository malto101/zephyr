/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
#include <zephyr/sw_isr_table.h>
#include <zephyr/arch/hexagon/exception.h>
#include <hexagon_vm.h>
#include <hexagon_intc.h>
#include <irq.h>
#include <event_context.h>
#ifdef CONFIG_GDBSTUB
#include <zephyr/arch/hexagon/gdbstub.h>
#endif
#ifdef CONFIG_USERSPACE
extern void z_hexagon_syscall_handler(struct arch_esf *esf);
extern void z_hexagon_user_mode_sync(void);
#endif

LOG_MODULE_DECLARE(os, CONFIG_KERNEL_LOG_LEVEL);

/* ISR nesting counter -- read by arch_is_in_isr() in arch.h */
uint32_t z_hexagon_isr_nesting;

/* Forward declarations for handlers defined below */
static void z_hexagon_exception_handler(struct event_context *ctx);
static void z_hexagon_trap0_handler(struct event_context *ctx);
static void z_hexagon_interrupt_handler(struct event_context *ctx);

/* Main event handler called from assembly */
void z_hexagon_event_handler(unsigned int event_num, struct event_context *ctx)
{
#ifdef CONFIG_USERSPACE
	/*
	 * We are now in kernel mode (H2 disabled guest interrupts on
	 * event entry).  Clear the user-mode flag so that any kernel
	 * code called from this handler sees arch_is_user_context()=false.
	 */
	_hexagon_user_mode_active = 0;
#endif

	/*
	 * Re-enable guest interrupts for syscall (trap0) handling.
	 * H2 disables IE on event entry, but kernel syscall code
	 * (k_sem_take, k_msgq_get, etc.) expects arch_irq_lock() to
	 * return key=1 (IE was enabled) so that subsequent z_swap()
	 * calls pass the SPIN_VALIDATE assertion.  Nested interrupts
	 * are safe because EVENT_ENTRY saves all volatile state.
	 */
	if (event_num == HEXAGON_EVENT_TRAP0) {
		hexagon_vm_setie(VM_INT_ENABLE);
#ifdef CONFIG_USERSPACE
		/*
		 * Mark this thread's own trap0 handling in flight so a
		 * nested event's z_hexagon_event_exit_user_sync() knows it
		 * is resuming this thread's still-kernel-mode handler, not
		 * real user-mode code -- see arch.trap0_active's own comment
		 * (thread.h) for why this is safe even if that handling
		 * later blocks (legitimately resumed here again much later)
		 * or is abandoned by a fault (never cleared, but only ever
		 * misleads this same thread, and only until it is reused).
		 */
		_current->arch.trap0_active = 1;
#endif
	}

	switch (event_num) {
	case HEXAGON_EVENT_MACHINE_CHECK:
		z_hexagon_fatal_error(K_ERR_CPU_EXCEPTION);
		break;

	case HEXAGON_EVENT_GENERAL_EXCEPTION:
		z_hexagon_exception_handler(ctx);
		break;

	case HEXAGON_EVENT_DEBUG:
#ifdef CONFIG_GDBSTUB
		/* Handled entirely by z_hexagon_enhanced_debug asm stub */
		break;
#else
		z_hexagon_fatal_error(K_ERR_CPU_EXCEPTION);
		break;
#endif

	case HEXAGON_EVENT_TRAP0:
		z_hexagon_trap0_handler(ctx);
		break;

	case HEXAGON_EVENT_INTERRUPT:
		z_hexagon_interrupt_handler(ctx);
		break;

	default:
		z_hexagon_fatal_error(K_ERR_SPURIOUS_IRQ);
		break;
	}

#ifdef CONFIG_USERSPACE
	if (event_num == HEXAGON_EVENT_TRAP0) {
		/* This thread's own trap0 handling has finished normally. */
		_current->arch.trap0_active = 0;
	}
#endif

	/*
	 * Disable interrupts before returning to the EVENT_EXIT assembly
	 * path.  EVENT_EXIT expects IE=0 for the preemption check and
	 * vmrte sequence.
	 */
	hexagon_vm_setie(VM_INT_DISABLE);
}

#ifdef CONFIG_USERSPACE
/*
 * Called from EVENT_EXIT assembly AFTER the context switch (or when no
 * switch occurred).  At this point _current is the thread that will
 * actually resume, so both the sync decision and z_hexagon_user_mode_sync()
 * itself read the correct thread.
 *
 * Only sync when _current->arch.trap0_active is clear, i.e. the thread
 * about to resume is not itself in the middle of its own trap0 handling.
 * That handling deliberately re-enables guest interrupts (see
 * z_hexagon_event_handler() above), so a hardware interrupt -- or, in
 * principle, an exception -- can fire while it is suspended and, if the
 * scheduler's preemption check decides not to switch away, resume back
 * into it directly. That resumption point is this thread's own
 * still-kernel-mode C code, never real user-mode code, no matter what
 * arch.priv_level says.
 *
 * This used to be checked against z_hexagon_isr_nesting (which only
 * counts HEXAGON_EVENT_INTERRUPT and is always back to 0 by the time any
 * event's own EVENT_EXIT runs, since it is incremented and decremented
 * entirely inside z_hexagon_interrupt_handler()) and, briefly, against a
 * global H2-event nesting counter and against this event's own saved
 * GSR IE bit; both replacements were tried and reverted -- the counter
 * leaked permanently upward whenever z_hexagon_fatal_error() aborted a
 * thread mid-trap0 (a z_swap() that never returns to that event's own
 * EVENT_EXIT to decrement it), and the GSR IE bit is set for perfectly
 * ordinary user-mode code too, not just nested trap0 handling, so it
 * could not tell the two apart at all. arch.trap0_active fixes both:
 * it is scoped to the one thread whose trap0 handling it tracks (see its
 * own comment in <zephyr/arch/hexagon/thread.h>), so an abandoned trap0
 * can only ever mislead that same thread's own future resumption, never
 * any other thread's -- and arch_new_thread() resets it, so even that is
 * bounded by the thread's own next reuse.
 */
void z_hexagon_event_exit_user_sync(void)
{
	if (!_current->arch.trap0_active) {
		z_hexagon_user_mode_sync();
	}
}
#endif

#ifdef CONFIG_GDBSTUB
/*
 * Save callee-saved GPRs (r16-r28) and the special registers EVENT_ENTRY
 * does not save, hand off to the GDB stub, then restore them (in case
 * GDB modified them). Shared by the general-exception and trap0 software-
 * breakpoint paths below: both reach a breakpoint the same way and need
 * the same extra register set live for z_hexagon_gdb_entry() to present
 * the full register file to GDB.
 */
static void hexagon_gdb_breakpoint_entry(struct event_context *ctx)
{
	/* Save callee-saved GPRs */
	__asm__ volatile(
		"memd(%0+#%1) = r17:16\n"
		"memd(%0+#%2) = r19:18\n"
		"memd(%0+#%3) = r21:20\n"
		"memd(%0+#%4) = r23:22\n"
		"memd(%0+#%5) = r25:24\n"
		"memd(%0+#%6) = r27:26\n"
		"memw(%0+#%7) = r28\n"
		:
		: "r"(_gdb_callee_regs),
		  "i"(GDB_CALLEE_R16_OFF), "i"(GDB_CALLEE_R18_OFF),
		  "i"(GDB_CALLEE_R20_OFF), "i"(GDB_CALLEE_R22_OFF),
		  "i"(GDB_CALLEE_R24_OFF), "i"(GDB_CALLEE_R26_OFF),
		  "i"(GDB_CALLEE_R28_OFF)
		: "memory");
	/* Save special registers */
	__asm__ volatile(
		"{ r0 = usr }\n memw(%0+#%1) = r0\n"
		"{ r0 = gp }\n  memw(%0+#%2) = r0\n"
		"{ r0 = ugp }\n memw(%0+#%3) = r0\n"
		"{ r0 = lc0 }\n memw(%0+#%4) = r0\n"
		"{ r0 = lc1 }\n memw(%0+#%5) = r0\n"
		"{ r0 = sa0 }\n memw(%0+#%6) = r0\n"
		"{ r0 = sa1 }\n memw(%0+#%7) = r0\n"
		:
		: "r"(_gdb_callee_regs),
		  "i"(GDB_CALLEE_USR_OFF), "i"(GDB_CALLEE_GP_OFF),
		  "i"(GDB_CALLEE_UGP_OFF), "i"(GDB_CALLEE_LC0_OFF),
		  "i"(GDB_CALLEE_LC1_OFF), "i"(GDB_CALLEE_SA0_OFF),
		  "i"(GDB_CALLEE_SA1_OFF)
		: "r0", "memory");

	z_hexagon_gdb_entry(ctx);

	/* Restore callee-saved GPRs (in case GDB modified them) */
	__asm__ volatile(
		"r17:16 = memd(%0+#%1)\n"
		"r19:18 = memd(%0+#%2)\n"
		"r21:20 = memd(%0+#%3)\n"
		"r23:22 = memd(%0+#%4)\n"
		"r25:24 = memd(%0+#%5)\n"
		"r27:26 = memd(%0+#%6)\n"
		"r28 = memw(%0+#%7)\n"
		:
		: "r"(_gdb_callee_regs),
		  "i"(GDB_CALLEE_R16_OFF), "i"(GDB_CALLEE_R18_OFF),
		  "i"(GDB_CALLEE_R20_OFF), "i"(GDB_CALLEE_R22_OFF),
		  "i"(GDB_CALLEE_R24_OFF), "i"(GDB_CALLEE_R26_OFF),
		  "i"(GDB_CALLEE_R28_OFF)
		: "memory");
	/* Restore special registers */
	__asm__ volatile(
		"r0 = memw(%0+#%1)\n usr = r0\n"
		"r0 = memw(%0+#%2)\n gp = r0\n"
		"r0 = memw(%0+#%3)\n ugp = r0\n"
		"r0 = memw(%0+#%4)\n lc0 = r0\n"
		"r0 = memw(%0+#%5)\n lc1 = r0\n"
		"r0 = memw(%0+#%6)\n sa0 = r0\n"
		"r0 = memw(%0+#%7)\n sa1 = r0\n"
		:
		: "r"(_gdb_callee_regs),
		  "i"(GDB_CALLEE_USR_OFF), "i"(GDB_CALLEE_GP_OFF),
		  "i"(GDB_CALLEE_UGP_OFF), "i"(GDB_CALLEE_LC0_OFF),
		  "i"(GDB_CALLEE_LC1_OFF), "i"(GDB_CALLEE_SA0_OFF),
		  "i"(GDB_CALLEE_SA1_OFF)
		: "r0", "memory");
}
#endif /* CONFIG_GDBSTUB */

/* Handle general exceptions */
#define GSR_CAUSE_MASK 0xFF

static void z_hexagon_exception_handler(struct event_context *ctx)
{
	uint32_t cause = ctx->gsr & GSR_CAUSE_MASK;
	uint32_t pc = ctx->gelr;

	LOG_ERR("exception: cause=0x%x pc=0x%x", cause, pc);

#ifdef CONFIG_GDBSTUB
	/* Check if the faulting instruction is a breakpoint.
	 *
	 * Guard against a misaligned or zero PC (e.g., from stack corruption
	 * or a fault before the first instruction) before dereferencing.
	 * Hexagon instructions are always 4-byte aligned; a non-aligned PC
	 * cannot be a valid breakpoint.
	 *
	 * When called from the general exception path (EVENT_ENTRY),
	 * callee-saved registers (r16-r28) are NOT saved by the assembly
	 * stub.  hexagon_gdb_breakpoint_entry() saves them so
	 * z_hexagon_gdb_entry() can present the full register file to GDB.
	 */
	if ((pc != 0U) && ((pc & 3U) == 0U) &&
	    *(uint32_t *)pc == HEXAGON_BREAK_INSN) {
		hexagon_gdb_breakpoint_entry(ctx);
		return;
	}
#endif

	/* Fatal error for now */
	z_hexagon_fatal_error(K_ERR_CPU_EXCEPTION);
}

/* Handle trap0 (syscall) events.
 *
 * Under H2, trap0 generates GEVB entry 5.  GELR points to the instruction
 * AFTER the trap0, so the trap0 itself is at GELR-4.
 *
 * The event_context r6_r7[0] holds r6 which is the syscall number by
 * convention.
 */
static void z_hexagon_trap0_handler(struct event_context *ctx)
{
#ifdef CONFIG_GDBSTUB
	{
		/* Check if the trap0 instruction is a GDB breakpoint.
		 * GELR = instruction after trap0, so breakpoint is at GELR-4.
		 * Guard against a zero or misaligned GELR (e.g. on reset fault).
		 */
		uint32_t trap_pc = ctx->gelr - 4;

		if ((trap_pc != 0U) && ((trap_pc & 3U) == 0U) &&
		    *(uint32_t *)trap_pc == HEXAGON_BREAK_INSN) {
			/* Point GELR back to the breakpoint address so that
			 * when GDB removes the breakpoint and continues,
			 * execution resumes at the original instruction.
			 */
			ctx->gelr = trap_pc;

			hexagon_gdb_breakpoint_entry(ctx);
			return;
		}
	}
#endif

#ifdef CONFIG_USERSPACE
	{
		/*
		 * Build a minimal arch_esf so that z_hexagon_syscall_handler
		 * can read argument registers and write the return value back.
		 * Only the argument and syscall-number registers matter here;
		 * the remaining fields are left zeroed.
		 */
		struct arch_esf esf = { 0 };

		esf.r0 = ctx->r0_r1[0];
		esf.r1 = ctx->r0_r1[1];
		esf.r2 = ctx->r2_r3[0];
		esf.r3 = ctx->r2_r3[1];
		esf.r4 = ctx->r4_r5[0];
		esf.r5 = ctx->r4_r5[1];
		esf.r6 = ctx->r6_r7[0]; /* syscall number */
		esf.r7 = ctx->r6_r7[1];

		z_hexagon_syscall_handler(&esf);

		/* Write return value back to the event context */
		ctx->r0_r1[0] = esf.r0;
	}
#else
	/*
	 * With CONFIG_USERSPACE disabled, trap0 should never fire from
	 * application code.  Treat any unexpected trap0 as a fatal error.
	 */
	ARG_UNUSED(ctx);
	z_hexagon_fatal_error(K_ERR_CPU_EXCEPTION);
#endif
}

/* Handle interrupts */
static void z_hexagon_interrupt_handler(struct event_context *ctx)
{
	uint32_t irq_num;
	uint32_t cause = ctx->gsr & GSR_CAUSE_MASK;

	/*
	 * GSR.CAUSE (bits 7:0) contains the virtual IRQ number
	 * delivered by H2 (e.g. 12 for the timer).  The interrupt
	 * has already been consumed from the pending bitmap, so
	 * vmintop GET would not find it.
	 */
	irq_num = cause;

	if (irq_num >= ARCH_IRQ_COUNT) {
		return; /* Out of range -- spurious */
	}

	/* Track ISR nesting so arch_is_in_isr() works correctly */
	z_hexagon_isr_nesting++;

	/* Call ISR from SW ISR table */
	const struct _isr_table_entry *entry = &_sw_isr_table[irq_num];

	entry->isr(entry->arg);

	z_hexagon_isr_nesting--;

	/* Re-enable the interrupt (H2 disables it on delivery) */
	hexagon_intc_ack(irq_num);
}

/* Enable an IRQ */
void arch_irq_enable(unsigned int irq)
{
	if (irq >= ARCH_IRQ_COUNT) {
		return;
	}

	/* Use interrupt controller abstraction */
	hexagon_intc_enable(irq);
}

/* Disable an IRQ */
void arch_irq_disable(unsigned int irq)
{
	if (irq >= ARCH_IRQ_COUNT) {
		return;
	}

	/* Use interrupt controller abstraction */
	hexagon_intc_disable(irq);
}

/* Check if IRQ is enabled */
int arch_irq_is_enabled(unsigned int irq)
{
	uint32_t status;

	if (irq >= ARCH_IRQ_COUNT) {
		return 0;
	}

	/* Use vmintop to query interrupt state */
	status = hexagon_vm_intop_status(irq);

	return status & 1;
}

/* Connect IRQ at runtime */
int arch_irq_connect_dynamic(unsigned int irq, unsigned int priority,
			     void (*routine)(const void *parameter), const void *parameter,
			     uint32_t flags)
{
	if (irq >= ARCH_IRQ_COUNT) {
		return -EINVAL;
	}

#ifdef CONFIG_DYNAMIC_INTERRUPTS
	/* Set up SW ISR table entry atomically with respect to the IRQ */
	unsigned int key = arch_irq_lock();

	_sw_isr_table[irq].isr = routine;
	_sw_isr_table[irq].arg = parameter;
	arch_irq_unlock(key);
#endif

	/* Set interrupt priority */
	hexagon_irq_priority_set(irq, priority);

	return 0;
}

/* Set interrupt priority */
void hexagon_irq_priority_set(unsigned int irq, unsigned int priority)
{
	if (irq >= ARCH_IRQ_COUNT || priority > HEXAGON_IRQ_PRIORITY_LOWEST) {
		return;
	}

	/* Use interrupt controller abstraction */
	hexagon_intc_set_priority(irq, priority);
}

/* Spurious interrupt handler */
FUNC_NORETURN void z_irq_spurious(const void *unused)
{
	ARG_UNUSED(unused);

	LOG_ERR("Spurious interrupt detected!");
	z_hexagon_fatal_error(K_ERR_SPURIOUS_IRQ);

	CODE_UNREACHABLE;
}
