/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>

#include <zephyr/devicetree.h>
#include <zephyr/drivers/timer/system_timer.h>
#include <zephyr/init.h>
#include <zephyr/irq.h>
#include <zephyr/sys/clock.h>
#include <zephyr/sys/sys_io.h>

#define QTIMER_NODE DT_NODELABEL(qtimer)
#define QTIMER_AC_BASE DT_REG_ADDR_BY_NAME(QTIMER_NODE, ac)
#define QTIMER_FRAME_BASE DT_REG_ADDR_BY_NAME(QTIMER_NODE, frame)
#define QTIMER_FREQ DT_PROP(QTIMER_NODE, clock_frequency)
#define QTIMER_IRQ DT_IRQN(QTIMER_NODE)

#define QTIMER_AC_CNTFRQ  0x00U
#define QTIMER_AC_CNTACR0 0x40U
#define QTIMER_CVAL_LO    0x20U
#define QTIMER_CVAL_HI    0x24U
#define QTIMER_TVAL       0x28U
#define QTIMER_CTL        0x2cU
#define QTIMER_CTL_ENABLE BIT(0)
#define QTIMER_CTL_IMASK  BIT(1)
#define QTIMER_CTL_ISTATUS BIT(2)

/* QuRT keeps a 192-tick guard between the current count and a match. */
#define TIMER_CORE_BACKEND_COMPARE_EXACT
#define TIMER_CORE_ALARM_LEAD_CYCLES 192U
#define TIMER_CORE_COUNTER_WIDTH 58

BUILD_ASSERT(QTIMER_IRQ == 2, "Lemans QTimer must use L2VIC IRQ 2");
BUILD_ASSERT(QTIMER_FREQ == 19200000, "Lemans QTimer frequency must be 19.2 MHz");
BUILD_ASSERT(DT_REG_SIZE_BY_NAME(QTIMER_NODE, ac) >= (QTIMER_AC_CNTACR0 + 4U),
	"QTimer access-control range is too small");
BUILD_ASSERT(DT_REG_SIZE_BY_NAME(QTIMER_NODE, frame) >= (QTIMER_CTL + 4U),
	"QTimer frame range is too small");

static inline uint64_t qtimer_read_counter(void)
{
	uint32_t hi;
	uint32_t lo;
	uint32_t hi_check;

	do {
		hi = sys_read32(QTIMER_FRAME_BASE + 0x04U);
		lo = sys_read32(QTIMER_FRAME_BASE + 0x00U);
		hi_check = sys_read32(QTIMER_FRAME_BASE + 0x04U);
	} while (hi != hi_check);

	return ((uint64_t)hi << 32) | lo;
}

static inline uint64_t timer_driver_cycle_get(void)
{
	return qtimer_read_counter();
}

static inline void timer_driver_set_compare(uint64_t deadline)
{
	/* The low word is written first, as in the QuRT physical-timer path. */
	sys_write32((uint32_t)deadline, QTIMER_FRAME_BASE + QTIMER_CVAL_LO);
	sys_write32((uint32_t)(deadline >> 32), QTIMER_FRAME_BASE + QTIMER_CVAL_HI);
	sys_write32(QTIMER_CTL_ENABLE, QTIMER_FRAME_BASE + QTIMER_CTL);
}

#include "system_timer_generic.h"

static void qtimer_isr(const void *arg)
{
	ARG_UNUSED(arg);
	timer_core_announce();
}

static int qtimer_init(void)
{
	/* Permit the native supervisor to access the physical counter frame. */
	sys_write32(0x3fU, QTIMER_AC_BASE + QTIMER_AC_CNTACR0);
	sys_write32(QTIMER_FREQ, QTIMER_AC_BASE + QTIMER_AC_CNTFRQ);

	/* Mask the source while the initial compare value is installed. */
	sys_write32(QTIMER_CTL_IMASK, QTIMER_FRAME_BASE + QTIMER_CTL);
	IRQ_CONNECT(QTIMER_IRQ, 0, qtimer_isr, NULL, 0);
	irq_enable(QTIMER_IRQ);
	timer_core_init();
	return 0;
}

SYS_INIT(qtimer_init, PRE_KERNEL_2, CONFIG_SYSTEM_CLOCK_INIT_PRIORITY);

void sys_clock_disable(void)
{
	sys_write32(QTIMER_CTL_IMASK, QTIMER_FRAME_BASE + QTIMER_CTL);
	irq_disable(QTIMER_IRQ);
}

void arch_busy_wait(uint32_t usec_to_wait)
{
	uint64_t start;
	uint64_t delay;

	if (usec_to_wait == 0U) {
		return;
	}

	start = qtimer_read_counter();
	delay = ((uint64_t)QTIMER_FREQ * usec_to_wait) / 1000000ULL;
	while ((qtimer_read_counter() - start) < delay) {
	}
}
