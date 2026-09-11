/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Hexagon user mode state tracking
 *
 * arch_is_user_context() cannot read struct k_thread directly because
 * syscall.h is processed before struct k_thread is fully defined in the
 * kernel include chain.  Instead, a per-system volatile flag is kept in
 * sync with the current thread's privilege level:
 *
 *  - Set to 1 by arch_user_mode_enter() before calling vmrte.
 *  - Cleared to 0 by the trap0 handler on every kernel re-entry, and
 *    restored to 1 when returning to a user thread.
 *
 * This gives correct results as long as:
 *  - There is at most one user-mode thread active at a time (true today).
 *  - The clear/restore happens atomically with the mode switch (true
 *    because guest interrupts are disabled during event handling).
 */

#include <zephyr/kernel.h>
#include <zephyr/arch/hexagon/arch.h>

#ifdef CONFIG_USERSPACE

BUILD_ASSERT(!IS_ENABLED(CONFIG_SMP),
	     "Hexagon user mode state uses a global flag: SMP is not supported");

/* Nonzero when the current thread is executing in user mode */
volatile uint32_t _hexagon_user_mode_active;

/**
 * @brief Synchronise the global flag with the current thread's priv_level.
 *
 * Called from the trap0/exception handler after the C handler returns,
 * just before vmrte.  Reads the current thread's arch.priv_level and
 * updates _hexagon_user_mode_active accordingly so that arch_is_user_context()
 * returns the correct value for the thread that is about to run.
 */
void z_hexagon_user_mode_sync(void)
{
	_hexagon_user_mode_active = (_current->arch.priv_level != 0) ? 1U : 0U;
}

#endif /* CONFIG_USERSPACE */
