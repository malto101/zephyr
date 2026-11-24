/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * HVX context management
 *
 * The hardware keeps a full set of HVX registers per context. SSR.XA picks
 * the context and SSR.XE turns HVX on. Every thread that uses HVX gets a
 * context of its own, so a context switch only has to flip XA/XE with the
 * HVM extbits hwconfig call.
 *
 * Threads start with XE=0. The first HVX instruction traps with cause 0x16
 * and z_hexagon_hvx_trap() hands out a context. If they are all taken, one
 * owner is pushed out to a save area and gets its state back the next time
 * it touches HVX. Contexts are zeroed before they change hands.
 */

#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/arch/hexagon/hvx.h>
#include <hexagon_vm.h>
#include <hexagon_hvx.h>

LOG_MODULE_DECLARE(os, CONFIG_KERNEL_LOG_LEVEL);

BUILD_ASSERT(!IS_ENABLED(CONFIG_SMP));

/* SSR.XA is 3 bits wide */
#define HVX_MAX_CONTEXTS 8U

#define HWCONFIG_EXTBITS  3U
#define HWCONFIG_EXTPOWER 5U

/* One spare area, a thread being restored holds its own while it evicts another */
K_MEM_SLAB_DEFINE_STATIC(hvx_save_slab, HEXAGON_HVX_SAVE_SIZE, CONFIG_HEXAGON_HVX_SAVE_AREAS + 1,
			 HEXAGON_HVX_VLEN);

/*
 * Owner of each context. Like the pole star, a context stays put with its
 * thread until that thread lets go.
 */
static struct k_thread *hvx_owner[HVX_MAX_CONTEXTS];
static uint32_t hvx_num_contexts;
static uint32_t hvx_next_victim;

/* What XA/XE are set to right now */
static uint32_t hvx_hw_xa;
static bool hvx_hw_xe;

static void hvx_set(uint32_t xa, bool xe)
{
	if ((xe == hvx_hw_xe) && (!xe || (xa == hvx_hw_xa))) {
		return;
	}

	(void)hexagon_vm_hwconfig(HWCONFIG_EXTBITS, 0U, xa, xe ? 1U : 0U);
	hvx_hw_xa = xa;
	hvx_hw_xe = xe;
}

static bool hvx_owns(const struct k_thread *thread)
{
	int8_t ctx = thread->arch.hvx_ctx;

	return (ctx >= 0) && ((uint32_t)ctx < hvx_num_contexts) && (hvx_owner[ctx] == thread);
}

/*
 * Runs at POST_KERNEL because a hwconfig call outside the VM's trap mask
 * comes back as a trap0 event, and that exit path needs the scheduler.
 */
static int hvx_init(void)
{
	uint32_t contexts = hexagon_vm_getinfo(vm_info_hvx_contexts);

	/*
	 * If HVM switches HVX contexts on its own (hvx_switch != 0) it ignores
	 * our XA/XE, and per-thread contexts are off the table.
	 */
	if ((contexts == 0U) || (hexagon_vm_getinfo(vm_info_hvx_vlength) != HEXAGON_HVX_VLEN) ||
	    (hexagon_vm_getinfo(vm_info_hvx_switch) != 0U)) {
		return 0;
	}

	if ((hexagon_vm_hwconfig(HWCONFIG_EXTPOWER, 0U, 1U, 0U) != 0) ||
	    (hexagon_vm_hwconfig(HWCONFIG_EXTBITS, 0U, 0U, 0U) != 0)) {
		LOG_WRN("HVX: hwconfig not allowed for this VM");
		return 0;
	}
	hvx_hw_xa = 0U;
	hvx_hw_xe = false;
	hvx_num_contexts = MIN(contexts, HVX_MAX_CONTEXTS);

	return 0;
}

SYS_INIT(hvx_init, POST_KERNEL, 0);

void z_hexagon_hvx_switch(void)
{
	if (hvx_owns(_current)) {
		hvx_set((uint32_t)_current->arch.hvx_ctx, true);
	} else {
		hvx_set(hvx_hw_xa, false);
	}
}

static int hvx_take_context(void)
{
	struct k_thread *victim;
	void *buf;
	uint32_t ctx;

	for (ctx = 0U; ctx < hvx_num_contexts; ctx++) {
		if (hvx_owner[ctx] == NULL) {
			return (int)ctx;
		}
	}

	if (k_mem_slab_alloc(&hvx_save_slab, &buf, K_NO_WAIT) != 0) {
		LOG_ERR("HVX: all %u contexts and %u save areas in use", hvx_num_contexts,
			CONFIG_HEXAGON_HVX_SAVE_AREAS);
		return -ENOMEM;
	}

	ctx = hvx_next_victim;
	hvx_next_victim = (ctx + 1U) % hvx_num_contexts;
	victim = hvx_owner[ctx];

	hvx_set(ctx, true);
	z_hexagon_hvx_save(buf);
	victim->arch.hvx_save = buf;
	victim->arch.hvx_ctx = -1;
	hvx_owner[ctx] = NULL;

	return (int)ctx;
}

bool z_hexagon_hvx_trap(void)
{
	struct k_thread *thread = _current;
	int ctx;

	if (hvx_num_contexts == 0U) {
		LOG_ERR("HVX: not available");
		return false;
	}

	if (arch_is_in_isr()) {
		LOG_ERR("HVX: used in ISR");
		return false;
	}

	/* XE was already on, so this is a real coprocessor error */
	if (hvx_hw_xe) {
		LOG_ERR("HVX: coprocessor error, XA=%u", hvx_hw_xa);
		return false;
	}

	if (hvx_owns(thread)) {
		hvx_set((uint32_t)thread->arch.hvx_ctx, true);
		return true;
	}

	ctx = hvx_take_context();
	if (ctx < 0) {
		return false;
	}

	hvx_owner[ctx] = thread;
	thread->arch.hvx_ctx = (int8_t)ctx;
	hvx_set((uint32_t)ctx, true);

	if (thread->arch.hvx_save != NULL) {
		z_hexagon_hvx_restore(thread->arch.hvx_save);
		k_mem_slab_free(&hvx_save_slab, thread->arch.hvx_save);
		thread->arch.hvx_save = NULL;
	} else {
		z_hexagon_hvx_zero();
	}

	return true;
}

void z_hexagon_hvx_thread_release(struct k_thread *thread)
{
	if (hvx_owns(thread)) {
		hvx_owner[thread->arch.hvx_ctx] = NULL;
	}
	thread->arch.hvx_ctx = -1;

	if (thread->arch.hvx_save != NULL) {
		k_mem_slab_free(&hvx_save_slab, thread->arch.hvx_save);
		thread->arch.hvx_save = NULL;
	}

	if (thread == _current) {
		hvx_set(hvx_hw_xa, false);
	}
}

uint32_t hexagon_hvx_contexts(void)
{
	return hvx_num_contexts;
}

int hexagon_hvx_release(void)
{
	unsigned int key;

	if (arch_is_in_isr()) {
		return -EPERM;
	}

	key = arch_irq_lock();
	z_hexagon_hvx_thread_release(_current);
	arch_irq_unlock(key);

	return 0;
}
