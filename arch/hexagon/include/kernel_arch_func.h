/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_ARCH_HEXAGON_INCLUDE_KERNEL_ARCH_FUNC_H_
#define ZEPHYR_ARCH_HEXAGON_INCLUDE_KERNEL_ARCH_FUNC_H_

#ifndef _ASMLANGUAGE

#include <zephyr/kernel_structs.h>
#include <zephyr/types.h>
#include <zephyr/sys/util.h>
#include <zephyr/toolchain.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Stack protection functions */
#ifdef CONFIG_HW_STACK_PROTECTION
extern void z_arch_stack_protection_setup(struct k_thread *thread);
#ifdef CONFIG_USERSPACE
extern void z_hexagon_stack_protect_user_enter(struct k_thread *thread);
#endif
#endif

#ifdef CONFIG_SMP
/* Point GP at this CPU's block */
void z_hexagon_cpu_local_init(struct z_hexagon_cpu_local *local);

#ifdef CONFIG_USERSPACE
/* Drop this CPU's installed memory map if another CPU asked for it */
void z_hexagon_mem_domain_evict_poll(void);
#endif
#endif

/* This CPU's ISR nesting count; only stable with interrupts locked */
static ALWAYS_INLINE uint32_t *z_hexagon_nested(void)
{
#ifdef CONFIG_SMP
	struct z_hexagon_cpu_local *local;

	__asm__ volatile("%0 = gp" : "=r"(local));
	return &local->nested;
#else
	return &_kernel.cpus[0].nested;
#endif
}

static inline bool arch_is_in_isr(void)
{
#ifdef CONFIG_SMP
	uint32_t nested;

	/* A single GP-relative load cannot race with migration */
	__asm__ volatile("%0 = memw(gp+#8)" : "=r"(nested));
	return nested != 0U;
#else
	return _kernel.cpus[0].nested != 0U;
#endif
}

/* Thread context switching */
extern void z_hexagon_arch_switch(void *switch_to, void **switched_from);
static ALWAYS_INLINE void arch_switch(void *switch_to, void **switched_from)
{
	z_hexagon_arch_switch(switch_to, switched_from);
}

/* Thread creation */
extern void arch_new_thread(struct k_thread *thread, k_thread_stack_t *stack, char *stack_ptr,
			   k_thread_entry_t entry, void *p1, void *p2, void *p3);

/* Thread abortion */
/* arch_thread_return_value_set is only needed for !CONFIG_USE_SWITCH */

/* Stack pointer manipulation */
extern char *arch_k_thread_stack_buffer(k_thread_stack_t *stack);

#ifdef __cplusplus
}
#endif

#endif /* _ASMLANGUAGE */

#endif /* ZEPHYR_ARCH_HEXAGON_INCLUDE_KERNEL_ARCH_FUNC_H_ */
