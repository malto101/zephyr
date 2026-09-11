/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Per-arch thread definition
 */

#ifndef ZEPHYR_INCLUDE_ARCH_HEXAGON_THREAD_H_
#define ZEPHYR_INCLUDE_ARCH_HEXAGON_THREAD_H_

/* Hexagon requires 8-byte stack alignment. Defined ahead of the structs
 * below so it can size/align the priv_stack member of _thread_arch.
 */
#define ARCH_STACK_PTR_ALIGN 8

#ifndef _ASMLANGUAGE
#include <zephyr/types.h>

/* Forward declaration to avoid including hvx.h from thread.h */
struct hvx_context;

/**
 * @brief Callee-saved register context for cooperative context switching.
 */
struct _callee_saved {
	/** General-purpose register R16 (callee-saved). */
	uint32_t r16;
	/** General-purpose register R17 (callee-saved). */
	uint32_t r17;
	/** General-purpose register R18 (callee-saved). */
	uint32_t r18;
	/** General-purpose register R19 (callee-saved). */
	uint32_t r19;
	/** General-purpose register R20 (callee-saved). */
	uint32_t r20;
	/** General-purpose register R21 (callee-saved). */
	uint32_t r21;
	/** General-purpose register R22 (callee-saved). */
	uint32_t r22;
	/** General-purpose register R23 (callee-saved). */
	uint32_t r23;
	/** General-purpose register R24 (callee-saved). */
	uint32_t r24;
	/** General-purpose register R25 (callee-saved). */
	uint32_t r25;
	/** General-purpose register R26 (callee-saved). */
	uint32_t r26;
	/** General-purpose register R27 (callee-saved). */
	uint32_t r27;

	/** Stack pointer (R29). */
	uint32_t r29_sp;
	/** Frame pointer (R30). */
	uint32_t r30_fp;

	/** Link register (R31). */
	uint32_t r31_lr;
};

typedef struct _callee_saved _callee_saved_t;

/* Thread flags */
#define HEXAGON_THREAD_FLAG_ABORT      0x01
#define HEXAGON_THREAD_FLAG_FP_USED    0x02
#define HEXAGON_THREAD_FLAG_STACK_PROT 0x04

/**
 * @brief Architecture-specific thread data.
 */
struct _thread_arch {
	/** Return value from arch_switch. */
	uint32_t swap_return_value;

	/* Thread privilege level */
	uint8_t priv_level;

	/*
	 * Nonzero while this thread's own trap0 (syscall) handling is
	 * in flight with guest interrupts deliberately re-enabled (see
	 * z_hexagon_event_handler(), irq_manage.c) -- including while that
	 * handling is suspended mid-syscall (a blocking call z_swap()ed
	 * away) or, if a fault aborts this thread without ever clearing it,
	 * permanently until arch_new_thread() resets it for whichever
	 * thread next reuses this k_thread object.
	 *
	 * Read by z_hexagon_event_exit_user_sync() -- via _current, always
	 * the thread about to actually resume -- to tell a hardware
	 * interrupt or exception nested inside this thread's own re-enabled
	 * IE window (resuming into this thread's still-kernel-mode trap0
	 * handler) from a genuine top-level event (resuming into whatever
	 * this thread's arch.priv_level says it should). Deliberately
	 * per-thread rather than a global counter or flag: a global would
	 * leak state across threads if abandoned by the fault case above;
	 * this can only ever mislead the same thread it belongs to, and
	 * only until that thread is next reinitialized.
	 */
	uint8_t trap0_active;

	/* Flags */
	uint8_t flags;

	/* Hardware thread ID (-1 if not a hardware thread) */
	int8_t hw_thread_id;

	/* Thread-local storage pointer */
	void *tls_ptr;

	/* User global pointer (UGP) for TLS */
	uint32_t ugp;

#ifdef CONFIG_HW_STACK_PROTECTION
	/* Stack protection FRAMELIMIT value */
	uint32_t framelimit;
#endif

#ifdef CONFIG_USERSPACE
	/* Original entry point and arguments for K_USER threads */
	void (*user_entry)(void *, void *, void *);
	void *user_p1;
	void *user_p2;
	void *user_p3;

	/*
	 * Dedicated kernel-side stack for this thread's own event/syscall
	 * handling while it runs in user mode (see arch_user_mode_enter()
	 * and EVENT_ENTRY in event_handlers.S). Living in the thread
	 * control block keeps it entirely separate from the thread's own,
	 * user-visible declared stack: a K_USER thread's declared stack
	 * size no longer has to also cover kernel-side event-handling
	 * headroom, unlike an earlier version of this port that carved a
	 * fixed reserve out of the top of the declared stack instead (that
	 * silently broke any K_USER thread whose declared stack was close
	 * to or smaller than the reserve, e.g. CONFIG_DYNAMIC_THREAD_STACK_SIZE's
	 * 1024-byte default).
	 */
	uint8_t priv_stack[CONFIG_PRIVILEGED_STACK_SIZE] __aligned(ARCH_STACK_PTR_ALIGN);
#endif

#ifdef CONFIG_HEXAGON_HVX
	/*
	 * Per-thread HVX context pointer.  Using a dedicated field here
	 * (rather than k_thread_custom_data) avoids conflicting with
	 * application use of the custom-data slot.
	 *
	 * NULL means this thread has not allocated an HVX context.
	 */
	struct hvx_context *hvx_ctx;
#endif
};

typedef struct _thread_arch _thread_arch_t;

#endif /* _ASMLANGUAGE */

#endif /* ZEPHYR_INCLUDE_ARCH_HEXAGON_THREAD_H_ */
