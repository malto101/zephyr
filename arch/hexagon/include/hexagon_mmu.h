/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_ARCH_HEXAGON_INCLUDE_HEXAGON_MMU_H_
#define ZEPHYR_ARCH_HEXAGON_INCLUDE_HEXAGON_MMU_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <zephyr/arch/hexagon/arch.h>
#include <hexagon_vm.h>

#define HEXAGON_PTE_PERM_MASK (__HVM_PTE_R | __HVM_PTE_W | __HVM_PTE_X | __HVM_PTE_U)

#define HEXAGON_MMU_VM_START ((uintptr_t)CONFIG_KERNEL_VM_BASE)
#define HEXAGON_MMU_VM_END   (HEXAGON_MMU_VM_START + (uintptr_t)CONFIG_KERNEL_VM_SIZE)

/* Kernel L1 table, installed while no memory domain's is. */
extern uint32_t hexagon_kernel_l1[HEXAGON_MMU_PT_ENTRIES];

static inline uint32_t hexagon_pte(uintptr_t pa, uint32_t perms, uint32_t cache_attr)
{
	return ((uint32_t)pa & ~(CONFIG_MMU_PAGE_SIZE - 1U)) | perms | (cache_attr << 6);
}

static inline bool hexagon_va_in_vm(uintptr_t va)
{
	return va >= HEXAGON_MMU_VM_START && va < HEXAGON_MMU_VM_END;
}

static inline uint32_t hexagon_pt_index(uintptr_t va)
{
	return (va >> 12) & (HEXAGON_MMU_PT_ENTRIES - 1U);
}

/* L2 table an L1 entry points to, NULL for a block or invalid entry. */
static inline uint32_t *hexagon_pde_l2(uint32_t pde)
{
	if ((pde & __HVM_PDE_S_MASK) != __HVM_PDE_S_4KB) {
		return NULL;
	}

	return (uint32_t *)(uintptr_t)(pde & ~(CONFIG_MMU_PAGE_SIZE - 1U));
}

/* Kernel PTE slot for va, NULL if no L2 table covers it. */
static inline uint32_t *hexagon_kernel_pte(uintptr_t va)
{
	uint32_t *l2 = hexagon_pde_l2(hexagon_kernel_l1[va >> HEXAGON_MMU_SLOT_SHIFT]);

	return (l2 == NULL) ? NULL : &l2[hexagon_pt_index(va)];
}

/* Kernel translation of va: an L2 PTE, a block L1 entry, or 0 if unmapped. */
static inline uint32_t hexagon_kernel_pte_get(uintptr_t va)
{
	uint32_t pde = hexagon_kernel_l1[va >> HEXAGON_MMU_SLOT_SHIFT];
	uint32_t *l2 = hexagon_pde_l2(pde);

	if (l2 != NULL) {
		return l2[hexagon_pt_index(va)];
	}

	return ((pde & __HVM_PDE_S_MASK) == __HVM_PDE_S_INVALID) ? 0U : pde;
}

/*
 * 1 if the page-aligned [va, va + size) lies inside one identity range
 * mapped at boot (image, console window, HVM kernel), 0 if it is disjoint
 * from all of them, -1 if it partially overlaps one.
 */
int hexagon_fixed_map_contains(uintptr_t va, size_t size);

#ifdef CONFIG_USERSPACE
/*
 * Note that vmclrmap() flushed the installed table's ASID after a
 * translation was removed or changed: every other table must flush on
 * its next install. See userspace.c.
 */
void hexagon_map_flushed(void);

/* Copy a new kernel L1 entry outside KERNEL_VM to every domain. */
void hexagon_mem_domain_sync_l1(uint32_t slot);

/* Recompute every domain's PTE for a changed kernel page in KERNEL_VM. */
void hexagon_mem_domain_sync_page(uintptr_t va);
#endif

#endif /* ZEPHYR_ARCH_HEXAGON_INCLUDE_HEXAGON_MMU_H_ */
