/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Hexagon hardware stack protection via FRAMELIMIT (c16)
 */

#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <hexagon_arch.h>
#include <kernel_internal.h>
#include <zephyr/arch/hexagon/thread.h>

#ifdef CONFIG_HW_STACK_PROTECTION

BUILD_ASSERT(HEXAGON_THREAD_FLAG_STACK_PROT == 0x04,
	     "arch/hexagon/core/switch.S hardcodes this bit position "
	     "(tstbit #2) -- update both if this value changes");

void z_arch_stack_protection_setup(struct k_thread *thread)
{
	if (thread == NULL) {
		return;
	}

	uintptr_t stack_limit = thread->stack_info.start;

	/* FRAMELIMIT is loaded when the thread is switched in. */
	thread->arch.framelimit = stack_limit;
	thread->arch.flags |= HEXAGON_THREAD_FLAG_STACK_PROT;
}

#endif /* CONFIG_HW_STACK_PROTECTION */
