/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/kernel_structs.h>
#include <zephyr/logging/log.h>
#include <switch_frame.h>

LOG_MODULE_DECLARE(os, CONFIG_KERNEL_LOG_LEVEL);

#define MAX_STACK_FRAMES CONFIG_ARCH_STACKWALK_MAX_FRAMES

/*
 * Hexagon allocframe creates a frame header at FP:
 *   [FP+0] = saved FP (previous frame pointer)
 *   [FP+4] = saved LR (return address)
 */
struct stackframe {
	uintptr_t fp;
	uintptr_t lr;
};

static inline bool in_stack_bound(uintptr_t addr,
				  const struct k_thread *const thread)
{
#ifdef CONFIG_THREAD_STACK_INFO
	uintptr_t start = thread->stack_info.start;
	uintptr_t end = start + thread->stack_info.size;

	return (addr >= start) && (addr < end);
#else
	ARG_UNUSED(addr);
	ARG_UNUSED(thread);
	return true;
#endif
}

static void walk_stackframe(stack_trace_callback_fn cb, void *cookie,
			    const struct k_thread *thread,
			    uintptr_t fp, uintptr_t lr)
{
	int i;

	for (i = 0; i < MAX_STACK_FRAMES; i++) {
		if (lr == 0U) {
			break;
		}

		if (!cb(cookie, lr)) {
			break;
		}

		if (fp == 0U) {
			break;
		}

		if (!in_stack_bound(fp, thread)) {
			break;
		}

		/* Follow the frame pointer chain */
		struct stackframe *frame = (struct stackframe *)fp;

		lr = frame->lr;
		fp = frame->fp;
	}
}

void arch_stack_walk(stack_trace_callback_fn callback_fn, void *cookie,
		     const struct k_thread *thread,
		     const struct arch_esf *esf)
{
	uintptr_t fp;
	uintptr_t lr;

	if (esf != NULL) {
		fp = esf->r30_fp;
		lr = esf->pc;
	} else if ((thread == NULL) || (thread == _current)) {
		/* Walk current thread's stack */
		__asm__ volatile("%[fp] = r30" : [fp] "=r"(fp));
		__asm__ volatile("%[lr] = r31" : [lr] "=r"(lr));
	} else {
		/*
		 * For a non-current thread, switch_handle is the saved SP
		 * (the base of the switch frame pushed by z_hexagon_arch_switch).
		 * The frame layout is defined in switch_frame.h:
		 *   [SP + SWITCH_FP_LR + 0] = saved r30 (FP)
		 *   [SP + SWITCH_FP_LR + 4] = saved r31 (LR)
		 */
		const uint32_t *frame = (const uint32_t *)thread->switch_handle;

		if (frame == NULL) {
			return;
		}
		fp = frame[SWITCH_FP_LR / 4];       /* r30 */
		lr = frame[SWITCH_FP_LR / 4 + 1];   /* r31 */
	}

	walk_stackframe(callback_fn, cookie, thread, fp, lr);
}
