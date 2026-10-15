/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/tracing/tracing.h>

#include <irq.h>

void arch_cpu_idle(void)
{
	sys_trace_idle();
	__asm__ volatile("wait(r0)" : : "r"(0));
	hexagon_native_irq_enable();
}

void arch_cpu_atomic_idle(unsigned int key)
{
	sys_trace_idle();
	__asm__ volatile("wait(r0)" : : "r"(0));
	hexagon_native_ssr_write(key);
}
