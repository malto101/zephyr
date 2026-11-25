/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/irq_offload.h>
#include <zephyr/irq.h>

/*
 * IRQ 0 is used as the software trigger for irq_offload.  The board DTS
 * assigns the lowest hardware IRQ at 0x0c (timer), so IRQ 0 does not
 * conflict with any real device on this platform.
 */
#define IRQ_OFFLOAD_LINE 0

/* Per CPU: each CPU posts the line to itself */
static irq_offload_routine_t offload_routine[CONFIG_MP_MAX_NUM_CPUS];
static const void *offload_param[CONFIG_MP_MAX_NUM_CPUS];

static void irq_offload_isr(const void *param)
{
	ARG_UNUSED(param);

	uint32_t cpu = arch_proc_id();
	irq_offload_routine_t tmp = offload_routine[cpu];

	offload_routine[cpu] = NULL;
	if (tmp != NULL) {
		tmp(offload_param[cpu]);
	}
}

void arch_irq_offload(irq_offload_routine_t routine, const void *parameter)
{
	unsigned int key = arch_irq_lock();
	uint32_t cpu = arch_proc_id();

	/*
	 * ISRs run with IE off, so a posted line would only fire after
	 * this ISR returns. Run the routine synchronously instead.
	 */
	if (k_is_in_isr()) {
		routine(parameter);
		arch_irq_unlock(key);
		return;
	}

	offload_routine[cpu] = routine;
	offload_param[cpu] = parameter;

	/* The line is per-VCPU, enable it on whichever CPU this is */
	irq_enable(IRQ_OFFLOAD_LINE);

	/* Fires on this CPU once IE is back on */
	hexagon_irq_trigger(IRQ_OFFLOAD_LINE);

	arch_irq_unlock(key);
}

void arch_irq_offload_init(void)
{
	IRQ_CONNECT(IRQ_OFFLOAD_LINE, 0, irq_offload_isr, NULL, 0);
	irq_enable(IRQ_OFFLOAD_LINE);
}
