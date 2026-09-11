/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_ARCH_HEXAGON_INCLUDE_HEXAGON_ARCH_H_
#define ZEPHYR_ARCH_HEXAGON_INCLUDE_HEXAGON_ARCH_H_

/**
 * @file
 * @brief Hexagon control registers and interrupt controller interface
 */

#ifndef _ASMLANGUAGE
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

static inline uintptr_t hexagon_get_framelimit(void)
{
	uintptr_t result;

	__asm__ volatile("%[result] = c16" : [result] "=r"(result) : : "memory");
	return result;
}

static inline void hexagon_set_framelimit(uintptr_t limit)
{
	__asm__ volatile("c16 = %[limit]" : : [limit] "r"(limit) : "memory");
}

static inline uintptr_t hexagon_get_stack_pointer(void)
{
	uintptr_t result;

	__asm__ volatile("%[result] = r29" : [result] "=r"(result));
	return result;
}

static inline uintptr_t hexagon_get_frame_pointer(void)
{
	uintptr_t result;

	__asm__ volatile("%[result] = r30" : [result] "=r"(result));
	return result;
}

static inline uint32_t hexagon_get_usr(void)
{
	uint32_t result;

	__asm__ volatile("%[result] = usr" : [result] "=r"(result));
	return result;
}

static inline void hexagon_set_usr(uint32_t usr_value)
{
	__asm__ volatile("usr = %[usr_value]" : : [usr_value] "r"(usr_value));
}

/*
 * FRAMEKEY: return address scrambling for stack smashing protection.
 * allocframe XOR-scrambles LR with FRAMEKEY before storing on stack;
 * deallocframe unscrambles before loading to LR.  Default 0 = disabled.
 */
static inline uint32_t hexagon_get_framekey(void)
{
	uint32_t result;

	__asm__ volatile("%[result] = framekey" : [result] "=r"(result) : : "memory");
	return result;
}

static inline void hexagon_set_framekey(uint32_t key)
{
	__asm__ volatile("framekey = %[key]" : : [key] "r"(key) : "memory");
}

/* Interrupt controller (VM hypercall-based) */
int hexagon_irq_trigger(unsigned int irq);
void hexagon_intc_init(void);
void hexagon_intc_enable(uint32_t irq);
void hexagon_intc_disable(uint32_t irq);
void hexagon_intc_set_priority(uint32_t irq, uint32_t priority);
void hexagon_intc_ack(uint32_t irq);

#ifdef __cplusplus
}
#endif

#endif /* _ASMLANGUAGE */

#endif /* ZEPHYR_ARCH_HEXAGON_INCLUDE_HEXAGON_ARCH_H_ */
