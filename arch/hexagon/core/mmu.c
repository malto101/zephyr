/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Hexagon kernel page table
 *
 * Replaces the boot table, which is RWX everywhere, with a
 * VM_TRANS_TYPE_TABLE map that enforces W^X on the kernel image:
 *
 *                     U   R   W   X   Granule
 *   Text              1   1   0   1   4KB
 *   Rodata            1   1   0   0   4KB
 *   Image RAM         0   1   1   0   4KB
 *   Console device    1   1   1   0   4MB block (if configured)
 *   HVM kernel image  0   1   1   1   4MB blocks
 *
 * KERNEL_VM has dedicated L2 tables; the rest of KERNEL_VM is left
 * unmapped for k_mem_map() and friends, see mem_manage.c. Guest mode
 * enforces R/W/X but ignores U, so kernel code reaches U=0 pages; user
 * mode only reaches U=1 ones. Console output runs at user tier during
 * trap0 dispatch, hence U on the console.
 */

#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/init.h>
#include <zephyr/linker/linker-defs.h>
#include <hexagon_vm.h>
#include <hexagon_mmu.h>

BUILD_ASSERT(CONFIG_MMU_PAGE_SIZE == 0x1000U, "HVM page tables use 4KB pages");
BUILD_ASSERT((CONFIG_KERNEL_VM_BASE % 0x400000U) == 0U,
	     "CONFIG_KERNEL_VM_BASE must be 4MB-aligned");
BUILD_ASSERT((CONFIG_HEXAGON_H2_KERNEL_BASE % 0x400000U) == 0U,
	     "CONFIG_HEXAGON_H2_KERNEL_BASE must be 4MB-aligned");
BUILD_ASSERT((CONFIG_HEXAGON_H2_KERNEL_SIZE % 0x400000U) == 0U &&
	     CONFIG_HEXAGON_H2_KERNEL_SIZE > 0U,
	     "CONFIG_HEXAGON_H2_KERNEL_SIZE must be a non-zero multiple of 4MB");

uint32_t hexagon_kernel_l1[HEXAGON_MMU_PT_ENTRIES] __aligned(HEXAGON_MMU_PT_ALIGN);

static uint32_t hexagon_kernel_vm_l2[HEXAGON_MMU_VM_SLOTS][HEXAGON_MMU_PT_ENTRIES]
	__aligned(HEXAGON_MMU_PT_ALIGN);

static void hexagon_kernel_map_pages(uintptr_t start, uintptr_t end, uint32_t perms)
{
	for (uintptr_t va = ROUND_DOWN(start, CONFIG_MMU_PAGE_SIZE); va < end;
	     va += CONFIG_MMU_PAGE_SIZE) {
		uint32_t *pte = hexagon_kernel_pte(va);

		/* A page shared by text and rodata stays executable. */
		if (*pte == 0U) {
			*pte = hexagon_pte(va, perms, __HEXAGON_C_WB_L2);
		}
	}
}

static void hexagon_kernel_map_block(uintptr_t base, uint32_t perms, uint32_t cache_attr)
{
	uint32_t slot = base >> HEXAGON_MMU_SLOT_SHIFT;

	if (hexagon_pde_l2(hexagon_kernel_l1[slot]) != NULL) {
		k_panic();
	}
	hexagon_kernel_l1[slot] = hexagon_pte(base, perms, cache_attr) | __HVM_PDE_S_4MB;
}

/* EARLY: memory domains copy this table before PRE_KERNEL_1. */
static int hexagon_kernel_map_init(void)
{
	uintptr_t text_start = (uintptr_t)__text_region_start;
	uintptr_t text_end = (uintptr_t)__text_region_end;
	uintptr_t ram_start = ROUND_UP(MAX(text_end, (uintptr_t)__rodata_region_end),
				       CONFIG_MMU_PAGE_SIZE);
	uintptr_t ram_end = (uintptr_t)_image_ram_end;

	if (text_start < HEXAGON_MMU_VM_START || ram_end > HEXAGON_MMU_VM_END) {
		k_panic();
	}

	for (uint32_t i = 0; i < HEXAGON_MMU_PT_ENTRIES; i++) {
		hexagon_kernel_l1[i] = __HVM_PDE_S_INVALID;
	}
	for (uint32_t i = 0; i < HEXAGON_MMU_VM_SLOTS; i++) {
		hexagon_kernel_l1[HEXAGON_MMU_VM_SLOT_FIRST + i] =
			(uint32_t)(uintptr_t)hexagon_kernel_vm_l2[i] | __HVM_PDE_S_4KB;
	}

	hexagon_kernel_map_pages(text_start, text_end, __HVM_PTE_R | __HVM_PTE_X | __HVM_PTE_U);
	hexagon_kernel_map_pages((uintptr_t)__rodata_region_start, (uintptr_t)__rodata_region_end,
				 __HVM_PTE_R | __HVM_PTE_U);
	hexagon_kernel_map_pages(ram_start, ram_end, __HVM_PTE_R | __HVM_PTE_W);

#if DT_HAS_CHOSEN(zephyr_console)
	hexagon_kernel_map_block(HVM_CONSOLE_MAP_BASE,
				 __HVM_PTE_R | __HVM_PTE_W | __HVM_PTE_U | __HVM_PTE_SHARED,
				 __HEXAGON_C_DEV);
#endif

	for (uintptr_t base = CONFIG_HEXAGON_H2_KERNEL_BASE;
	     base < CONFIG_HEXAGON_H2_KERNEL_BASE + CONFIG_HEXAGON_H2_KERNEL_SIZE;
	     base += 0x400000U) {
		hexagon_kernel_map_block(base, __HVM_PTE_R | __HVM_PTE_W | __HVM_PTE_X,
					 __HEXAGON_C_WB_L2);
	}

	if (hexagon_vm_newmap(hexagon_kernel_l1, VM_TRANS_TYPE_TABLE, VM_TLB_INVALIDATE_TRUE) !=
	    0) {
		k_panic();
	}

	return 0;
}

SYS_INIT(hexagon_kernel_map_init, EARLY, 0);

int hexagon_fixed_map_contains(uintptr_t va, size_t size)
{
	const uintptr_t ranges[][2] = {
		{ ROUND_DOWN((uintptr_t)__text_region_start, CONFIG_MMU_PAGE_SIZE),
		  ROUND_UP((uintptr_t)_image_ram_end, CONFIG_MMU_PAGE_SIZE) },
#if DT_HAS_CHOSEN(zephyr_console)
		{ HVM_CONSOLE_MAP_BASE, HVM_CONSOLE_MAP_BASE + 0x400000U },
#endif
		{ CONFIG_HEXAGON_H2_KERNEL_BASE,
		  CONFIG_HEXAGON_H2_KERNEL_BASE + CONFIG_HEXAGON_H2_KERNEL_SIZE },
	};
	uintptr_t end = va + size;

	for (size_t i = 0; i < ARRAY_SIZE(ranges); i++) {
		if (va >= ranges[i][0] && end <= ranges[i][1]) {
			return 1;
		}
		if (va < ranges[i][1] && end > ranges[i][0]) {
			return -1;
		}
	}

	return 0;
}
