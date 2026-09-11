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
#include <zephyr/linker/section_tags.h>

#ifdef CONFIG_USERSPACE

BUILD_ASSERT(!IS_ENABLED(CONFIG_SMP),
	     "Hexagon user mode state uses a global flag: SMP is not supported");

/*
 * Nonzero when the current thread is executing in user mode.
 *
 * Placed in the dedicated .hex_user_readable section (see
 * soc/qemu/hexagon/linker.ld) rather than ordinary .bss: hexagon_mmu_init()
 * maps that section R+W+U while it maps the rest of kernel RAM R+W with no
 * U, and this flag is exactly the case that split has to make an exception
 * for -- arch_is_user_context() (syscall.h) reads it directly, without a
 * syscall, from whichever mode is currently running, user included. Unlike
 * ordinary .bss, that section is never automatically zeroed at boot;
 * hexagon_mmu_init() does it explicitly since it already necessarily runs
 * before anything could read this flag.
 */
volatile uint32_t _hexagon_user_mode_active Z_GENERIC_SECTION(.hex_user_readable);

/*
 * Mirrors _current for the same reason _hexagon_user_mode_active mirrors
 * arch_is_user_context()'s answer: hexagon_user_thread_exit()
 * (userspace.c), the raw-asm return address arch_user_mode_enter() sets
 * up for when a user thread's entry function returns, needs the current
 * thread pointer to call k_thread_abort(self) -- and, being __naked with
 * no established stack frame, can't safely call k_current_get() (or any
 * other real C function) to get it. Reading _kernel.cpus[0].current
 * directly used to be this mirror; that struct is ordinary kernel .bss,
 * now mapped without U by hexagon_mmu_init()'s kernel/user RAM split, so
 * it faulted from a real user thread's return path the first time any
 * test's user thread actually ran off the end of its entry function.
 */
struct k_thread *_hexagon_current_thread_user_visible Z_GENERIC_SECTION(.hex_user_readable);

/**
 * @brief Synchronise the global flags with the current thread.
 *
 * Called from the trap0/exception handler after the C handler returns,
 * just before vmrte.  Reads the current thread (and its arch.priv_level)
 * and updates _hexagon_user_mode_active/_hexagon_current_thread_user_visible
 * accordingly, so both reflect the thread that is about to run.
 */
void z_hexagon_user_mode_sync(void)
{
	_hexagon_user_mode_active = (_current->arch.priv_level != 0) ? 1U : 0U;
	_hexagon_current_thread_user_visible = _current;
}

#endif /* CONFIG_USERSPACE */
