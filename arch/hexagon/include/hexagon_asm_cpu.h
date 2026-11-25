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

/* rd = this CPU's _cpu_t */
.macro LOAD_CURRENT_CPU rd
#ifdef CONFIG_SMP
	\rd = memw(gp+#0)
#else
	\rd = ##_kernel
	\rd = add(\rd, #___kernel_t_cpus_OFFSET)
#endif
.endm

#endif /* _ASMLANGUAGE */

#endif /* ZEPHYR_ARCH_HEXAGON_INCLUDE_HEXAGON_ASM_CPU_H_ */
