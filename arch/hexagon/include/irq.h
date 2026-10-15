/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_ARCH_HEXAGON_INCLUDE_IRQ_H_
#define ZEPHYR_ARCH_HEXAGON_INCLUDE_IRQ_H_

#include <zephyr/irq.h>
#include <zephyr/sw_isr_table.h>

#if defined(CONFIG_HEXAGON_HVM_GUEST)
#include <hexagon_vm.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Number of interrupts matches the Kconfig-defined NUM_IRQS */
#define ARCH_IRQ_COUNT CONFIG_NUM_IRQS

/* Event numbers */
#define HEXAGON_EVENT_MACHINE_CHECK     1
#define HEXAGON_EVENT_GENERAL_EXCEPTION 2
#define HEXAGON_EVENT_DEBUG             3
#define HEXAGON_EVENT_TRAP0             5
#define HEXAGON_EVENT_INTERRUPT         7

/* Interrupt priority levels (0-63, 0 is highest) */
#define HEXAGON_IRQ_PRIORITY_HIGHEST 0
#define HEXAGON_IRQ_PRIORITY_LOWEST  63

#ifndef _ASMLANGUAGE

#if defined(CONFIG_HEXAGON_HVM_GUEST)
/* Interrupt lock/unlock through the H2 vmsetie/vmgetie calls. */
static ALWAYS_INLINE unsigned int arch_irq_lock(void)
{
	return hexagon_vm_setie(VM_INT_DISABLE);
}

static ALWAYS_INLINE void arch_irq_unlock(unsigned int key)
{
	hexagon_vm_setie(key);
}

static ALWAYS_INLINE bool arch_irq_unlocked(unsigned int key)
{
	return key != 0U;
}

static ALWAYS_INLINE bool arch_cpu_irqs_are_enabled(void)
{
	return hexagon_vm_getie() != VM_INT_DISABLE;
}
#else
/* Native Hexagon stores the interrupt-enable state in SSR bit 18. */
#define HEXAGON_SSR_IE BIT(18)

static ALWAYS_INLINE unsigned int hexagon_native_ssr_read(void)
{
	unsigned int ssr;

	__asm__ volatile("%0 = ssr" : "=r"(ssr));
	return ssr;
}

static ALWAYS_INLINE void hexagon_native_ssr_write(unsigned int ssr)
{
	__asm__ volatile("ssr = %0\n\tisync" : : "r"(ssr));
}

static ALWAYS_INLINE unsigned int arch_irq_lock(void)
{
	unsigned int ssr = hexagon_native_ssr_read();

	hexagon_native_ssr_write(ssr & ~HEXAGON_SSR_IE);
	return ssr;
}

static ALWAYS_INLINE void arch_irq_unlock(unsigned int key)
{
	hexagon_native_ssr_write(key);
}

static ALWAYS_INLINE bool arch_irq_unlocked(unsigned int key)
{
	return (key & HEXAGON_SSR_IE) != 0U;
}

static ALWAYS_INLINE bool arch_cpu_irqs_are_enabled(void)
{
	return (hexagon_native_ssr_read() & HEXAGON_SSR_IE) != 0U;
}

static ALWAYS_INLINE void hexagon_native_irq_enable(void)
{
	unsigned int ssr = hexagon_native_ssr_read();

	hexagon_native_ssr_write(ssr | HEXAGON_SSR_IE);
}

static ALWAYS_INLINE void hexagon_native_irq_disable(void)
{
	unsigned int ssr = hexagon_native_ssr_read();

	hexagon_native_ssr_write(ssr & ~HEXAGON_SSR_IE);
}

static ALWAYS_INLINE void hexagon_native_irq_sync(void)
{
	__asm__ volatile("isync");
}

void z_hexagon_native_interrupt_dispatch(void);
#endif

/* Enable specific IRQ */
void arch_irq_enable(unsigned int irq);

/* Disable specific IRQ */
void arch_irq_disable(unsigned int irq);

/* Check if IRQ is enabled */
int arch_irq_is_enabled(unsigned int irq);

#ifdef CONFIG_DYNAMIC_INTERRUPTS
/* Connect IRQ at runtime */
int arch_irq_connect_dynamic(unsigned int irq, unsigned int priority,
			     void (*routine)(const void *parameter), const void *parameter,
			     uint32_t flags);
#endif

/* Hexagon-specific interrupt operations */
void hexagon_irq_priority_set(unsigned int irq, unsigned int priority);
int hexagon_irq_trigger(unsigned int irq);

#endif /* _ASMLANGUAGE */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_ARCH_HEXAGON_INCLUDE_IRQ_H_ */
