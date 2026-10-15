/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <errno.h>

#include <zephyr/devicetree.h>
#include <zephyr/init.h>
#include <zephyr/toolchain.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/sys_io.h>

#include <hexagon_intc.h>
#include <irq.h>

#define L2VIC_BASE DT_REG_ADDR(DT_NODELABEL(l2vic))

#define L2VIC_ENABLE        0x100U
#define L2VIC_ENABLE_CLEAR  0x180U
#define L2VIC_ENABLE_SET    0x200U
#define L2VIC_TYPE          0x280U
#define L2VIC_STATUS        0x380U
#define L2VIC_CLEAR         0x400U
#define L2VIC_SOFTWARE_INT  0x480U

#define L2VIC_WORD(irq) ((irq) >> 5U)
#define L2VIC_MASK(irq) BIT((irq) & 0x1fU)

static inline uintptr_t l2vic_reg(uint32_t offset, uint32_t word)
{
	return (uintptr_t)L2VIC_BASE + offset + (word * sizeof(uint32_t));
}

static inline void l2vic_write(uint32_t offset, uint32_t word, uint32_t value)
{
	sys_write32(value, l2vic_reg(offset, word));
	/* Command registers are write-only; read status to complete the write. */
	(void)sys_read32(l2vic_reg(L2VIC_STATUS, word));
}

void hexagon_intc_ack(uint32_t irq)
{
	if (irq < ARCH_IRQ_COUNT) {
		l2vic_write(L2VIC_CLEAR, L2VIC_WORD(irq), L2VIC_MASK(irq));
	}
}

void hexagon_intc_set_pending(uint32_t irq)
{
	if (irq < ARCH_IRQ_COUNT) {
		l2vic_write(L2VIC_SOFTWARE_INT, L2VIC_WORD(irq), L2VIC_MASK(irq));
	}
}

void hexagon_intc_enable(uint32_t irq)
{
	if (irq < ARCH_IRQ_COUNT) {
		l2vic_write(L2VIC_ENABLE_SET, L2VIC_WORD(irq), L2VIC_MASK(irq));
	}
}

void hexagon_intc_disable(uint32_t irq)
{
	if (irq < ARCH_IRQ_COUNT) {
		l2vic_write(L2VIC_ENABLE_CLEAR, L2VIC_WORD(irq), L2VIC_MASK(irq));
	}
}

void hexagon_intc_set_priority(uint32_t irq, uint32_t priority)
{
	ARG_UNUSED(irq);
	ARG_UNUSED(priority);
}

static int hexagon_intc_init_native(void)
{
	for (uint32_t word = 0U; word < (ARCH_IRQ_COUNT / 32U); word++) {
		/* Disable all sources before changing their trigger configuration. */
		l2vic_write(L2VIC_ENABLE_CLEAR, word, UINT32_MAX);

		/* QuRT uses zero for level-sensitive sources on this controller. */
		l2vic_write(L2VIC_TYPE, word, 0U);

		/* Drop reset-era pending state before any source is enabled. */
		l2vic_write(L2VIC_CLEAR, word, UINT32_MAX);
	}

	return 0;
}

void hexagon_intc_init(void)
{
	(void)hexagon_intc_init_native();
}

SYS_INIT(hexagon_intc_init_native, PRE_KERNEL_1, CONFIG_INTC_INIT_PRIORITY);

int hexagon_irq_trigger(unsigned int irq)
{
	if (irq >= ARCH_IRQ_COUNT) {
		return -EINVAL;
	}

	hexagon_intc_set_pending(irq);
	return 0;
}

uint32_t hexagon_intc_status(uint32_t word)
{
	if (word >= (ARCH_IRQ_COUNT / 32U)) {
		return 0U;
	}

	return sys_read32(l2vic_reg(L2VIC_STATUS, word));
}

uint32_t hexagon_intc_enable_status(uint32_t word)
{
	if (word >= (ARCH_IRQ_COUNT / 32U)) {
		return 0U;
	}

	return sys_read32(l2vic_reg(L2VIC_ENABLE, word));
}
