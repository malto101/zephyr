/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Hexagon architecture inline function implementations
 */

#ifndef ZEPHYR_INCLUDE_ARCH_HEXAGON_ARCH_INLINES_H_
#define ZEPHYR_INCLUDE_ARCH_HEXAGON_ARCH_INLINES_H_

#include <zephyr/kernel_structs.h>
#include <hexagon_vm.h>

#ifdef CONFIG_SMP
/*
 * Per-CPU block that GP points to. GP is unused with -G0, HVM keeps it per
 * VCPU, and its low 6 bits are read-only, hence the alignment. User code
 * can write GP, so USERSPACE on SMP would need it reloaded on event entry.
 */
struct z_hexagon_cpu_local {
	_cpu_t *cpu;
	struct k_thread *current;
	/* ISR nesting count, kept here so one load reads it without a race */
	uint32_t nested;
} __aligned(64);

extern struct z_hexagon_cpu_local z_hexagon_cpu_local[CONFIG_MP_MAX_NUM_CPUS];
#endif

/**
 * @brief Return the current CPU's kernel data.
 *
 * On SMP it is read from the GP block.
 *
 * @return Pointer to the current CPU's _cpu_t.
 */
static ALWAYS_INLINE _cpu_t *arch_curr_cpu(void)
{
#ifdef CONFIG_SMP
	_cpu_t *cpu;

	__asm__ volatile("%0 = memw(gp+#0)" : "=r"(cpu));
	return cpu;
#else
	return &_kernel.cpus[0];
#endif
}

#ifdef CONFIG_ARCH_HAS_CUSTOM_CURRENT_IMPL
/* A single GP-relative load cannot race with migration. */
static ALWAYS_INLINE struct k_thread *arch_current_thread(void)
{
	struct k_thread *thread;

	__asm__ volatile("%0 = memw(gp+#4)" : "=r"(thread));
	return thread;
}

static ALWAYS_INLINE void arch_current_thread_set(struct k_thread *thread)
{
	__asm__ volatile("memw(gp+#4) = %0" : : "r"(thread) : "memory");
}
#endif

/**
 * @brief Return the ID of the current CPU.
 *
 * @return Zephyr CPU number, always 0 without SMP.
 */
static ALWAYS_INLINE uint32_t arch_proc_id(void)
{
	return IS_ENABLED(CONFIG_SMP) ? arch_curr_cpu()->id : 0U;
}

/**
 * @brief Return the number of CPUs.
 *
 * @return Number of CPUs on the system.
 */
static ALWAYS_INLINE unsigned int arch_num_cpus(void)
{
	return CONFIG_MP_MAX_NUM_CPUS;
}

#endif /* ZEPHYR_INCLUDE_ARCH_HEXAGON_ARCH_INLINES_H_ */
