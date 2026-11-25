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
/* Zephyr CPU running on each HVM VCPU slot, set as each CPU starts */
extern _cpu_t *z_hexagon_slot_cpu[HVM_MAX_VCPUS];
#endif

/**
 * @brief Return the current CPU's kernel data.
 *
 * On SMP each CPU is an HVM VCPU, found through its vmvpid slot.
 *
 * @return Pointer to the current CPU's _cpu_t.
 */
static ALWAYS_INLINE _cpu_t *arch_curr_cpu(void)
{
#ifdef CONFIG_SMP
	return z_hexagon_slot_cpu[HVM_ID_CPUIDX(hexagon_vm_vpid())];
#else
	return &_kernel.cpus[0];
#endif
}

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
