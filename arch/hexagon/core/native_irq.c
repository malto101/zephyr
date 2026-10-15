/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>

#include <zephyr/kernel.h>
#include <zephyr/sw_isr_table.h>
#include <zephyr/irq.h>
#include <zephyr/toolchain.h>

#include <hexagon_intc.h>
#include <irq.h>

extern uint32_t z_hexagon_isr_nesting;

void arch_irq_enable(unsigned int irq)
{
	if (irq < ARCH_IRQ_COUNT) {
		hexagon_intc_enable(irq);
	}
}

void arch_irq_disable(unsigned int irq)
{
	if (irq < ARCH_IRQ_COUNT) {
		hexagon_intc_disable(irq);
	}
}

int arch_irq_is_enabled(unsigned int irq)
{
	uint32_t word;
	uint32_t mask;

	if (irq >= ARCH_IRQ_COUNT) {
		return 0;
	}

	word = irq >> 5U;
	mask = BIT(irq & 0x1fU);
	return (hexagon_intc_enable_status(word) & mask) != 0U;
}

void z_irq_spurious(const void *unused)
{
	ARG_UNUSED(unused);
	k_fatal_halt(K_ERR_SPURIOUS_IRQ);
	CODE_UNREACHABLE;
}

void z_hexagon_native_interrupt_dispatch(void)
{
	for (uint32_t word = 0U; word < (ARCH_IRQ_COUNT / 32U); word++) {
		uint32_t pending = hexagon_intc_status(word) & hexagon_intc_enable_status(word);

		while (pending != 0U) {
			unsigned int bit = (unsigned int)__builtin_ctz(pending);
			unsigned int irq = (word * 32U) + bit;
			const struct _isr_table_entry *entry = &_sw_isr_table[irq];

			pending &= ~BIT(bit);
			hexagon_intc_ack(irq);
			z_hexagon_isr_nesting++;
			entry->isr(entry->arg);
			z_hexagon_isr_nesting--;
		}
	}
}
