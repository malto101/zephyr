/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Hexagon specific syscall header
 *
 * This header contains the Hexagon specific syscall interface.  It is
 * included by the syscall interface architecture-abstraction header
 * (include/arch/syscall.h)
 */

#ifndef ZEPHYR_INCLUDE_ARCH_HEXAGON_SYSCALL_H_
#define ZEPHYR_INCLUDE_ARCH_HEXAGON_SYSCALL_H_

#ifndef _ASMLANGUAGE

#include <zephyr/types.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Syscall invocation macros. hexagon-specific machine constraints used to ensure
 * args land in the proper registers.
 */
static inline uintptr_t arch_syscall_invoke6(uintptr_t arg1, uintptr_t arg2, uintptr_t arg3,
					     uintptr_t arg4, uintptr_t arg5, uintptr_t arg6,
					     uintptr_t call_id)
{
	register uint32_t r6 __asm__("r6") = call_id;
	register uint32_t r0 __asm__("r0") = arg1;
	register uint32_t r1 __asm__("r1") = arg2;
	register uint32_t r2 __asm__("r2") = arg3;
	register uint32_t r3 __asm__("r3") = arg4;
	register uint32_t r4 __asm__("r4") = arg5;
	register uint32_t r5 __asm__("r5") = arg6;

	__asm__ __volatile__("trap0(#1)" : "=r"(r0) : "r"(r6), "0"(r0), "r"(r1), "r"(r2),
			     "r"(r3), "r"(r4), "r"(r5) : "memory");
	return r0;
}

static inline uintptr_t arch_syscall_invoke5(uintptr_t arg1, uintptr_t arg2, uintptr_t arg3,
					     uintptr_t arg4, uintptr_t arg5, uintptr_t call_id)
{
	register uint32_t r6 __asm__("r6") = call_id;
	register uint32_t r0 __asm__("r0") = arg1;
	register uint32_t r1 __asm__("r1") = arg2;
	register uint32_t r2 __asm__("r2") = arg3;
	register uint32_t r3 __asm__("r3") = arg4;
	register uint32_t r4 __asm__("r4") = arg5;

	__asm__ __volatile__("trap0(#1)" : "=r"(r0) : "r"(r6), "0"(r0), "r"(r1), "r"(r2),
			     "r"(r3), "r"(r4) : "memory");
	return r0;
}

static inline uintptr_t arch_syscall_invoke4(uintptr_t arg1, uintptr_t arg2, uintptr_t arg3,
					     uintptr_t arg4, uintptr_t call_id)
{
	register uint32_t r6 __asm__("r6") = call_id;
	register uint32_t r0 __asm__("r0") = arg1;
	register uint32_t r1 __asm__("r1") = arg2;
	register uint32_t r2 __asm__("r2") = arg3;
	register uint32_t r3 __asm__("r3") = arg4;

	__asm__ __volatile__("trap0(#1)" : "=r"(r0) : "r"(r6), "0"(r0), "r"(r1), "r"(r2),
			     "r"(r3) : "memory");
	return r0;
}

static inline uintptr_t arch_syscall_invoke3(uintptr_t arg1, uintptr_t arg2, uintptr_t arg3,
					     uintptr_t call_id)
{
	register uint32_t r6 __asm__("r6") = call_id;
	register uint32_t r0 __asm__("r0") = arg1;
	register uint32_t r1 __asm__("r1") = arg2;
	register uint32_t r2 __asm__("r2") = arg3;

	__asm__ __volatile__("trap0(#1)" : "=r"(r0) : "r"(r6), "0"(r0), "r"(r1),
			     "r"(r2) : "memory");
	return r0;
}

static inline uintptr_t arch_syscall_invoke2(uintptr_t arg1, uintptr_t arg2, uintptr_t call_id)
{
	register uint32_t r6 __asm__("r6") = call_id;
	register uint32_t r0 __asm__("r0") = arg1;
	register uint32_t r1 __asm__("r1") = arg2;

	__asm__ __volatile__("trap0(#1)" : "=r"(r0) : "r"(r6), "0"(r0), "r"(r1) : "memory");
	return r0;
}

static inline uintptr_t arch_syscall_invoke1(uintptr_t arg1, uintptr_t call_id)
{
	register uint32_t r6 __asm__("r6") = call_id;
	register uint32_t r0 __asm__("r0") = arg1;

	__asm__ __volatile__("trap0(#1)" : "=r"(r0) : "r"(r6), "0"(r0) : "memory");
	return r0;
}

static inline uintptr_t arch_syscall_invoke0(uintptr_t call_id)
{
	register uint32_t r6 __asm__("r6") = call_id;
	register uint32_t r0 __asm__("r0");

	__asm__ __volatile__("trap0(#1)" : "=r"(r0) : "r"(r6) : "memory");
	return r0;
}

#ifdef CONFIG_USERSPACE
/*
 * _hexagon_user_mode_active is set to 1 by arch_user_mode_enter() before
 * issuing vmrte, and cleared back to 0 when a trap0/exception returns to
 * kernel mode.  It must be read atomically (volatile) since it is written
 * and read across the user/kernel boundary.
 */
extern volatile uint32_t _hexagon_user_mode_active;

static inline bool arch_is_user_context(void)
{
	return _hexagon_user_mode_active != 0;
}

/*
 * Mirrors _current, kept in sync alongside _hexagon_user_mode_active
 * (arch/hexagon/core/user_mode_state.c) for the same reason: code that
 * needs the current thread pointer but can't safely make a real function
 * call to get it -- currently just hexagon_user_thread_exit() (a __naked
 * function with no stack frame, used as the return address for a user
 * thread whose entry function returns) -- reads this instead of
 * _kernel.cpus[0].current, which hexagon_mmu_init() maps without U like
 * any other ordinary kernel .bss. struct k_thread is forward-declared
 * only (not defined) at this point in the include chain, but that's
 * enough for a pointer declaration.
 */
struct k_thread;
extern struct k_thread *_hexagon_current_thread_user_visible;
#endif

#ifdef __cplusplus
}
#endif

#endif /* _ASMLANGUAGE */
#endif /* ZEPHYR_INCLUDE_ARCH_HEXAGON_SYSCALL_H_ */
