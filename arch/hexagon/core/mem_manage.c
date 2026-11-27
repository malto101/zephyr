/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Hexagon runtime memory mappings
 *
 * The kernel image, console and HVM windows are mapped once at boot by
 * mmu.c. Mappings made later (k_mem_map(), k_mem_map_phys_bare(),
 * device MMIO) go to one linear list that every installed map reaches
 * through hexagon_dyn_link, ahead of hexagon_fixed_tail:
 *
 *   kernel map / domain list --> hexagon_dyn_link --> dyn list --> fixed tail
 *
 * HVM walks the installed lists on every STLB miss, so the list is never
 * edited in place: each change fills the other buffer and then swaps
 * hexagon_dyn_link to it. Entries stay disjoint from each other and from
 * the boot-time map, because ctlbw refuses a TLB entry overlapping one
 * already present and the miss then repeats forever.
 *
 * A new translation needs no TLB maintenance: HVM caches no misses.
 * Removing or changing one does. vmclrmap() only flushes the installed
 * map's ASID, so hexagon_map_flushed() makes every other map flush its
 * ASID on its next install.
 */

#include <zephyr/kernel.h>
#include <zephyr/kernel/mm.h>
#include <zephyr/spinlock.h>
#include <zephyr/sys/barrier.h>
#include <kernel_arch_interface.h>
#include <hexagon_vm.h>

#ifdef CONFIG_MMU

#include <hexagon_mmu.h>

/*
 * Entries are disjoint and each covers at least one page, so one per
 * KERNEL_VM page is enough there: the kernel treats a failed
 * arch_mem_map() as fatal.
 */
#define HEXAGON_DYN_ENTRIES                                                                        \
	(CONFIG_KERNEL_VM_SIZE / CONFIG_MMU_PAGE_SIZE + CONFIG_HEXAGON_MMU_DIRECT_MAP_ENTRIES)

struct hexagon_linear_entry hexagon_dyn_link[1] __aligned(8);

/* Two buffers: the one hexagon_dyn_link points at, and the next one. */
static struct hexagon_linear_entry hexagon_dyn_lists[2][HEXAGON_DYN_ENTRIES + 1] __aligned(8);
static uint32_t hexagon_dyn_active;
static struct k_spinlock hexagon_dyn_lock;

static inline uintptr_t hexagon_entry_va(const struct hexagon_linear_entry *entry)
{
	return (uintptr_t)(entry->high & 0xFFFFFU) << 12;
}

static inline uintptr_t hexagon_entry_pa(const struct hexagon_linear_entry *entry)
{
	return (uintptr_t)(entry->low & 0xFFFFFFU) << 12;
}

static inline bool hexagon_entry_is_mapping(const struct hexagon_linear_entry *entry)
{
	return (entry->high & __HVM_LINEAR_CHAIN) == 0U &&
	       (entry->low != 0U || entry->high != 0U);
}

void hexagon_dyn_map_init(void)
{
	k_spinlock_key_t key = k_spin_lock(&hexagon_dyn_lock);

	hexagon_linear_entry_set_chain(&hexagon_dyn_lists[hexagon_dyn_active][0],
				       hexagon_fixed_tail);
	hexagon_linear_entry_set_chain(hexagon_dyn_link, hexagon_dyn_lists[hexagon_dyn_active]);
	k_spin_unlock(&hexagon_dyn_lock, key);
}

/*
 * Publish the active list minus [va, va + size), plus va -> pa over that
 * range if map is true. On success *flush_start and *flush_end bound every
 * entry removed or cut, empty if none was.
 */
static int hexagon_dyn_update(uintptr_t va, size_t size, bool map, uintptr_t pa, uint32_t xwru,
			      uint32_t cache_attr, uint32_t extra_high, uintptr_t *flush_start,
			      uintptr_t *flush_end)
{
	const struct hexagon_linear_entry *src = hexagon_dyn_lists[hexagon_dyn_active];
	struct hexagon_linear_entry *dst = hexagon_dyn_lists[hexagon_dyn_active ^ 1U];
	uintptr_t end = va + size;
	uint32_t n = 0;
	int ret;

	*flush_start = UINTPTR_MAX;
	*flush_end = 0;

	for (; hexagon_entry_is_mapping(src); src++) {
		uintptr_t e_va = hexagon_entry_va(src);
		uintptr_t e_end = hexagon_linear_entry_end(src);
		uintptr_t e_pa = hexagon_entry_pa(src);
		uint32_t e_xwru = hexagon_linear_entry_xwru(src);
		uint32_t e_cache = (src->low >> 24) & 0x7U;
		uint32_t e_extra = src->high & __HVM_LINEAR_SHARED;

		if (e_end <= va || e_va >= end) {
			if (n >= HEXAGON_DYN_ENTRIES) {
				return -ENOMEM;
			}
			dst[n++] = *src;
			continue;
		}

		*flush_start = MIN(*flush_start, e_va);
		*flush_end = MAX(*flush_end, e_end);

		if (e_va < va) {
			ret = hexagon_decompose_mapping(&dst[n], HEXAGON_DYN_ENTRIES - n, e_va,
							e_pa, va - e_va, e_xwru, e_cache, e_extra);
			if (ret < 0) {
				return ret;
			}
			n += ret;
		}
		if (e_end > end) {
			ret = hexagon_decompose_mapping(&dst[n], HEXAGON_DYN_ENTRIES - n, end,
							e_pa + (end - e_va), e_end - end, e_xwru,
							e_cache, e_extra);
			if (ret < 0) {
				return ret;
			}
			n += ret;
		}
	}

	if (map) {
		ret = hexagon_decompose_mapping(&dst[n], HEXAGON_DYN_ENTRIES - n, va, pa, size,
						xwru, cache_attr, extra_high);
		if (ret < 0) {
			return ret;
		}
		n += ret;
	}

	hexagon_linear_entry_set_chain(&dst[n], hexagon_fixed_tail);

	/* The new list must be complete before HVM can reach it. */
	barrier_dmem_fence_full();
	*(volatile uint32_t *)&hexagon_dyn_link[0].low = (uint32_t)(uintptr_t)dst;
	barrier_dmem_fence_full();
	hexagon_dyn_active ^= 1U;

	return 0;
}

static void hexagon_dyn_flush(uintptr_t start, uintptr_t end)
{
	if (start >= end) {
		return;
	}

	/*
	 * One page per call: vmclrmap() with a length above 1 flushes the
	 * whole ASID from the TLB, not just the range.
	 */
	for (uintptr_t va = start; va < end; va += CONFIG_MMU_PAGE_SIZE) {
		(void)hexagon_vm_clrmap((void *)va, 1);
	}
#ifdef CONFIG_USERSPACE
	hexagon_map_flushed();
#endif
}

int arch_mem_map(void *virt, uintptr_t phys, size_t size, uint32_t flags)
{
	uintptr_t va = (uintptr_t)virt;
	uint32_t xwru = __HVM_LINEAR_R;
	uint32_t cache_attr;
	uint32_t extra_high = 0;
	uintptr_t flush_start;
	uintptr_t flush_end;
	k_spinlock_key_t key;
	int ret;

	if (size == 0) {
		return 0;
	}
	if (((va | phys | size) & (CONFIG_MMU_PAGE_SIZE - 1)) != 0) {
		return -EINVAL;
	}

	/* The boot-time map already covers it, and nothing may overlap it. */
	ret = hexagon_fixed_map_contains(va, size);
	if (ret != 0) {
		return (ret > 0 && va == phys) ? 0 : -EINVAL;
	}

	if ((flags & K_MEM_PERM_RW) != 0U) {
		xwru |= __HVM_LINEAR_W;
	}
	if ((flags & K_MEM_PERM_EXEC) != 0U) {
		xwru |= __HVM_LINEAR_X;
	}
	if ((flags & K_MEM_PERM_USER) != 0U) {
		xwru |= __HVM_LINEAR_U;
	}

	switch (flags & K_MEM_CACHE_MASK) {
	case K_MEM_CACHE_NONE:
		/* As the console window: device memory, no guestmap. */
		cache_attr = __HEXAGON_C_DEV;
		extra_high = __HVM_LINEAR_SHARED;
		break;
	case K_MEM_CACHE_WT:
		cache_attr = __HEXAGON_C_WT_L2;
		break;
	default:
		cache_attr = __HEXAGON_C_WB_L2;
		break;
	}

	key = k_spin_lock(&hexagon_dyn_lock);
	ret = hexagon_dyn_update(va, size, true, phys, xwru, cache_attr, extra_high, &flush_start,
				 &flush_end);
	if (ret == 0) {
		hexagon_dyn_flush(flush_start, flush_end);
	}
	k_spin_unlock(&hexagon_dyn_lock, key);

	return ret;
}

int arch_mem_unmap(void *addr, size_t size)
{
	uintptr_t flush_start;
	uintptr_t flush_end;
	k_spinlock_key_t key;
	int ret;

	if (size == 0) {
		return 0;
	}

	key = k_spin_lock(&hexagon_dyn_lock);
	ret = hexagon_dyn_update((uintptr_t)addr, size, false, 0, 0, 0, 0, &flush_start,
				 &flush_end);
	if (ret == 0) {
		hexagon_dyn_flush(flush_start, flush_end);
	}
	k_spin_unlock(&hexagon_dyn_lock, key);

	return ret;
}

int arch_page_phys_get(void *virt, uintptr_t *phys)
{
	uintptr_t va = ROUND_DOWN((uintptr_t)virt, CONFIG_MMU_PAGE_SIZE);
	const struct hexagon_linear_entry *entry;
	uintptr_t pa = 0;
	int ret = -EFAULT;
	k_spinlock_key_t key;

	if (hexagon_fixed_map_contains(va, CONFIG_MMU_PAGE_SIZE) > 0) {
		pa = va;
		ret = 0;
	} else {
		key = k_spin_lock(&hexagon_dyn_lock);
		for (entry = hexagon_dyn_lists[hexagon_dyn_active]; hexagon_entry_is_mapping(entry);
		     entry++) {
			if (va >= hexagon_entry_va(entry) && va < hexagon_linear_entry_end(entry)) {
				pa = hexagon_entry_pa(entry) + (va - hexagon_entry_va(entry));
				ret = 0;
				break;
			}
		}
		k_spin_unlock(&hexagon_dyn_lock, key);
	}

	if (ret == 0 && phys != NULL) {
		*phys = pa + ((uintptr_t)virt - va);
	}

	return ret;
}

#endif /* CONFIG_MMU */

/* Cache operations using HVM vmcache hypercall */

void arch_dcache_flush_all(void)
{
	hexagon_vm_cache(hvmc_dccleaninva, 0, 0);
}

void arch_dcache_invd_all(void)
{
	hexagon_vm_cache(hvmc_dckill, 0, 0);
}

void arch_dcache_flush_and_invd_all(void)
{
	hexagon_vm_cache(hvmc_dccleaninva, 0, 0);
}
