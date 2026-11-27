/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Hexagon runtime memory mappings
 *
 * k_mem_map(), k_mem_map_phys_bare() and device MMIO write 4KB PTEs into
 * the kernel page table, see mmu.c. KERNEL_VM pages are copied into every
 * memory domain's own L2 tables; elsewhere the L2 tables come from a pool
 * and are shared by every domain.
 *
 * A new translation needs no TLB maintenance: HVM caches no misses.
 * Removing or changing one does. vmclrmap() only flushes the installed
 * table's ASID, so hexagon_map_flushed() makes every other table flush
 * its ASID on its next install.
 */

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/kernel/mm.h>
#include <zephyr/spinlock.h>
#include <zephyr/sys/barrier.h>
#include <kernel_arch_interface.h>
#include <hexagon_vm.h>

#ifdef CONFIG_MMU

#include <hexagon_mmu.h>

#ifdef CONFIG_USERSPACE
#include <kuser.h>
/* Domain tables are updated together with the kernel's. */
#define hexagon_mmu_lock z_mem_domain_lock
#else
static struct k_spinlock hexagon_mmu_lock;
#endif

static uint32_t hexagon_l2_pool[CONFIG_HEXAGON_MMU_L2_TABLES][HEXAGON_MMU_PT_ENTRIES]
	__aligned(HEXAGON_MMU_PT_ALIGN);
static uint32_t hexagon_l2_pool_used;

static bool hexagon_slot_is_invalid(uint32_t slot)
{
	return (hexagon_kernel_l1[slot] & __HVM_PDE_S_MASK) == __HVM_PDE_S_INVALID;
}

/* L2 tables [va, end) still needs, or -EINVAL if it touches a block. */
static int hexagon_l2_tables_needed(uintptr_t va, uintptr_t end)
{
	int needed = 0;

	for (uintptr_t slot_va = ROUND_DOWN(va, BIT(HEXAGON_MMU_SLOT_SHIFT)); slot_va < end;
	     slot_va += BIT(HEXAGON_MMU_SLOT_SHIFT)) {
		uint32_t slot = slot_va >> HEXAGON_MMU_SLOT_SHIFT;

		if (hexagon_slot_is_invalid(slot)) {
			needed++;
		} else if (hexagon_pde_l2(hexagon_kernel_l1[slot]) == NULL) {
			return -EINVAL;
		}
		if (slot_va + BIT(HEXAGON_MMU_SLOT_SHIFT) == 0U) {
			break;
		}
	}

	return needed;
}

static uint32_t *hexagon_kernel_pte_alloc(uintptr_t va)
{
	uint32_t slot = va >> HEXAGON_MMU_SLOT_SHIFT;

	if (hexagon_slot_is_invalid(slot)) {
		uint32_t *l2 = hexagon_l2_pool[hexagon_l2_pool_used++];

		(void)memset(l2, 0, sizeof(hexagon_l2_pool[0]));
		barrier_dmem_fence_full();
		hexagon_kernel_l1[slot] = (uint32_t)(uintptr_t)l2 | __HVM_PDE_S_4KB;
#ifdef CONFIG_USERSPACE
		hexagon_mem_domain_sync_l1(slot);
#endif
	}

	return hexagon_kernel_pte(va);
}

/* Write pte for each page of [va, va + size), pa advancing unless pte is 0. */
static void hexagon_kernel_pte_update(uintptr_t va, size_t size, uint32_t pte)
{
	bool flushed = false;

	for (size_t off = 0; off < size; off += CONFIG_MMU_PAGE_SIZE) {
		uintptr_t page = va + off;
		uint32_t *slot = hexagon_kernel_pte_alloc(page);
		uint32_t old = *slot;
		uint32_t new = (pte == 0U) ? 0U : pte + off;

		if (old == new) {
			continue;
		}
		*slot = new;
#ifdef CONFIG_USERSPACE
		if (hexagon_va_in_vm(page)) {
			hexagon_mem_domain_sync_page(page);
		}
#endif
		if (old != 0U) {
			barrier_dmem_fence_full();
			(void)hexagon_vm_clrmap((void *)page, 1);
			flushed = true;
		}
	}

#ifdef CONFIG_USERSPACE
	if (flushed) {
		hexagon_map_flushed();
	}
#else
	ARG_UNUSED(flushed);
#endif
}

int arch_mem_map(void *virt, uintptr_t phys, size_t size, uint32_t flags)
{
	uintptr_t va = (uintptr_t)virt;
	uint32_t perms = __HVM_PTE_R;
	uint32_t cache_attr;
	k_spinlock_key_t key;
	int ret;

	if (size == 0) {
		return 0;
	}
	if (((va | phys | size) & (CONFIG_MMU_PAGE_SIZE - 1)) != 0 || va + size < va) {
		return -EINVAL;
	}

	/* The boot-time map already covers it, and nothing may overlap it. */
	ret = hexagon_fixed_map_contains(va, size);
	if (ret != 0) {
		return (ret > 0 && va == phys) ? 0 : -EINVAL;
	}

	if ((flags & K_MEM_PERM_RW) != 0U) {
		perms |= __HVM_PTE_W;
	}
	if ((flags & K_MEM_PERM_EXEC) != 0U) {
		perms |= __HVM_PTE_X;
	}
	if ((flags & K_MEM_PERM_USER) != 0U) {
		perms |= __HVM_PTE_U;
	}

	switch (flags & K_MEM_CACHE_MASK) {
	case K_MEM_CACHE_NONE:
		/* As the console window: device memory, no guestmap. */
		cache_attr = __HEXAGON_C_DEV;
		perms |= __HVM_PTE_SHARED;
		break;
	case K_MEM_CACHE_WT:
		cache_attr = __HEXAGON_C_WT_L2;
		break;
	default:
		cache_attr = __HEXAGON_C_WB_L2;
		break;
	}

	key = k_spin_lock(&hexagon_mmu_lock);
	ret = hexagon_l2_tables_needed(va, va + size);
	if (ret > (int)(CONFIG_HEXAGON_MMU_L2_TABLES - hexagon_l2_pool_used)) {
		ret = -ENOMEM;
	}
	if (ret >= 0) {
		hexagon_kernel_pte_update(va, size, hexagon_pte(phys, perms, cache_attr));
		ret = 0;
	}
	k_spin_unlock(&hexagon_mmu_lock, key);

	return ret;
}

int arch_mem_unmap(void *addr, size_t size)
{
	uintptr_t va = ROUND_DOWN((uintptr_t)addr, CONFIG_MMU_PAGE_SIZE);
	k_spinlock_key_t key;

	size = ROUND_UP((uintptr_t)addr + size, CONFIG_MMU_PAGE_SIZE) - va;
	if (size == 0 || hexagon_fixed_map_contains(va, size) != 0) {
		return 0;
	}

	key = k_spin_lock(&hexagon_mmu_lock);
	for (size_t off = 0; off < size; off += CONFIG_MMU_PAGE_SIZE) {
		if (hexagon_kernel_pte(va + off) != NULL) {
			hexagon_kernel_pte_update(va + off, CONFIG_MMU_PAGE_SIZE, 0U);
		}
	}
	k_spin_unlock(&hexagon_mmu_lock, key);

	return 0;
}

int arch_page_phys_get(void *virt, uintptr_t *phys)
{
	uintptr_t va = (uintptr_t)virt;
	uint32_t pde = hexagon_kernel_l1[va >> HEXAGON_MMU_SLOT_SHIFT];
	uint32_t pte = hexagon_kernel_pte_get(va);
	uintptr_t page_mask;

	if (pte == 0U) {
		return -EFAULT;
	}

	page_mask = (hexagon_pde_l2(pde) != NULL) ? (CONFIG_MMU_PAGE_SIZE - 1U)
						  : (BIT(HEXAGON_MMU_SLOT_SHIFT) - 1U);
	if (phys != NULL) {
		*phys = (pte & ~page_mask) | (va & page_mask);
	}

	return 0;
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
