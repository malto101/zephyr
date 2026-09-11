/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Hexagon public kernel interface
 */

#ifndef ZEPHYR_INCLUDE_ARCH_HEXAGON_ARCH_H_
#define ZEPHYR_INCLUDE_ARCH_HEXAGON_ARCH_H_

#include <zephyr/devicetree.h>
#include <zephyr/arch/hexagon/thread.h>
#include <zephyr/arch/hexagon/exception.h>
#include <zephyr/arch/hexagon/error.h>
#include <zephyr/arch/common/sys_bitops.h>
#include <zephyr/arch/common/ffs.h>
#include <zephyr/arch/common/sys_io.h>
#include <zephyr/sys/util.h>
#include <zephyr/kernel/mm.h>
#include <hexagon_vm.h>
#include <irq.h>
#include <irq_connect.h>
#include <zephyr/sw_isr_table.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Connect a direct ISR to an interrupt line.
 *
 * @param irq_p IRQ number.
 * @param priority_p Interrupt priority (unused on Hexagon).
 * @param isr_p ISR function pointer.
 * @param flags_p IRQ flags (unused on Hexagon).
 */
#define ARCH_IRQ_DIRECT_CONNECT(irq_p, priority_p, isr_p, flags_p) \
	Z_ISR_DECLARE_DIRECT(irq_p, ISR_FLAG_DIRECT, isr_p)

/** @brief Invoke the direct ISR header. */
#define ARCH_ISR_DIRECT_HEADER() arch_isr_direct_header()

/** @brief Invoke the direct ISR footer. */
#define ARCH_ISR_DIRECT_FOOTER(swap) arch_isr_direct_footer(swap)

#ifdef CONFIG_PM
extern void arch_isr_direct_pm(void);
/** @brief Perform power management work at direct ISR exit. */
#define ARCH_ISR_DIRECT_PM() arch_isr_direct_pm()
#else
/** @brief No-op when power management is disabled. */
#define ARCH_ISR_DIRECT_PM() do { } while (false)
#endif

/**
 * @brief Direct ISR header handler.
 *
 * Called at entry of a direct ISR to perform tracing if enabled.
 */
static inline void arch_isr_direct_header(void)
{
#ifdef CONFIG_TRACING_ISR
	extern void sys_trace_isr_enter(void);
	sys_trace_isr_enter();
#endif
}

/**
 * @brief Direct ISR footer handler.
 *
 * Called at exit of a direct ISR to perform tracing if enabled.
 *
 * @param maybe_swap Unused on Hexagon.
 */
static inline void arch_isr_direct_footer(int maybe_swap)
{
#ifdef CONFIG_TRACING_ISR
	extern void sys_trace_isr_exit(void);
	sys_trace_isr_exit();
#endif
	ARG_UNUSED(maybe_swap);
}

/**
 * @brief Declare a direct ISR.
 *
 * @param name Name of the ISR function to declare.
 */
#define ARCH_ISR_DIRECT_DECLARE(name) \
	static inline int name##_body(void); \
	void name(void) \
	{ \
		int check_reschedule; \
		ISR_DIRECT_HEADER(); \
		check_reschedule = name##_body(); \
		ISR_DIRECT_FOOTER(check_reschedule); \
	} \
	static inline int name##_body(void)

/**
 * @brief Architecture-specific kernel initialization.
 *
 * No additional initialization needed for Hexagon.
 */
static inline void arch_kernel_init(void)
{
}

/**
 * @brief Get the current system clock cycle count as a 32-bit value.
 *
 * @return Current 32-bit cycle count.
 */
extern uint32_t sys_clock_cycle_get_32(void);

/**
 * @brief Get the current cycle count (32-bit).
 *
 * @return Current 32-bit hardware cycle count.
 */
static inline uint32_t arch_k_cycle_get_32(void)
{
	return sys_clock_cycle_get_32();
}

/**
 * @brief Get the current system clock cycle count as a 64-bit value.
 *
 * @return Current 64-bit cycle count.
 */
extern uint64_t sys_clock_cycle_get_64(void);

/**
 * @brief Get the current cycle count (64-bit).
 *
 * @return Current 64-bit hardware cycle count.
 */
static inline uint64_t arch_k_cycle_get_64(void)
{
	return sys_clock_cycle_get_64();
}

/**
 * @brief Execute a no-op instruction.
 */
static inline void arch_nop(void)
{
	__asm__ volatile("nop");
}

/* ISR nesting counter -- incremented/decremented in irq_manage.c */
extern uint32_t z_hexagon_isr_nesting;

/**
 * @brief Check if currently executing in interrupt context.
 *
 * @retval true if in ISR context.
 * @retval false if in thread context.
 */
static inline bool arch_is_in_isr(void)
{
	return z_hexagon_isr_nesting != 0U;
}

#ifdef CONFIG_USERSPACE
/* Memory partition attributes for userspace support */
typedef uint32_t k_mem_partition_attr_t;

/*
 * Memory partition permission attribute bit encoding:
 *   bit 0: user-readable
 *   bit 1: user-writable
 *   bit 2: executable (both priv and user)
 *   bit 3: priv-writable (implies priv-readable)
 *   bit 4: priv-readable only (no user access)
 *
 * These values are placeholders until arch_mem_domain_partition_add()
 * is implemented to enforce them via the H2 page table permission bits.
 */
#define K_MEM_PARTITION_P_RW_U_RW  ((k_mem_partition_attr_t)0x0b) /* priv RW + user RW */
#define K_MEM_PARTITION_P_RW_U_RO  ((k_mem_partition_attr_t)0x09) /* priv RW + user RO */
#define K_MEM_PARTITION_P_RW_U_NA  ((k_mem_partition_attr_t)0x08) /* priv RW, no user */
#define K_MEM_PARTITION_P_RO_U_RO  ((k_mem_partition_attr_t)0x11) /* priv RO + user RO */
#define K_MEM_PARTITION_P_RO_U_NA  ((k_mem_partition_attr_t)0x10) /* priv RO, no user */
#define K_MEM_PARTITION_P_NA_U_NA  ((k_mem_partition_attr_t)0x00) /* no access */
#define K_MEM_PARTITION_P_RWX_U_RWX ((k_mem_partition_attr_t)0x0f) /* priv RWX + user RWX */
#define K_MEM_PARTITION_P_RWX_U_RX ((k_mem_partition_attr_t)0x0d) /* priv RWX, user RX */
#define K_MEM_PARTITION_P_RX_U_RX  ((k_mem_partition_attr_t)0x15) /* priv RX, user RX */

#define K_MEM_PARTITION_IS_WRITABLE(attr)   ((attr) & 0x02)
#define K_MEM_PARTITION_IS_EXECUTABLE(attr) ((attr) & 0x04)
#endif

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_ARCH_HEXAGON_ARCH_H_ */
