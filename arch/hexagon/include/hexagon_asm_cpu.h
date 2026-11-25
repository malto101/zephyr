/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_ARCH_HEXAGON_INCLUDE_HEXAGON_ASM_CPU_H_
#define ZEPHYR_ARCH_HEXAGON_INCLUDE_HEXAGON_ASM_CPU_H_

#ifdef _ASMLANGUAGE

#include <hexagon_vm.h>
#include <offsets_short.h>

/*
 * rd = this CPU's _cpu_t. On SMP this clobbers r0 (vmvpid), so r0 must be
 * dead and rd must not be r0.
 */
.macro LOAD_CURRENT_CPU rd
#ifdef CONFIG_SMP
	trap1(#HEXAGON_VM_vmvpid)
	{
		r0 = extractu(r0, #HVM_ID_CPUIDX_WIDTH, #HVM_ID_CPUIDX_SHIFT)
		\rd = ##z_hexagon_slot_cpu
	}
	\rd = memw(\rd + r0 << #2)
#else
	\rd = ##_kernel
	\rd = add(\rd, #___kernel_t_cpus_OFFSET)
#endif
.endm

#endif /* _ASMLANGUAGE */

#endif /* ZEPHYR_ARCH_HEXAGON_INCLUDE_HEXAGON_ASM_CPU_H_ */
