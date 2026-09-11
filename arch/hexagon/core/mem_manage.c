/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Hexagon MMU management via the H2 hypervisor page table interface.
 *
 * hexagon_mmu_init() builds a 2-level page table (a 1024-entry L1 "PGD"
 * plus a pool of 64KB-granularity L2 tables, see HEX_PAGE_SIZE below for
 * why 64KB) and activates it via the vmnewmap hypercall, replacing the
 * single-level, 4MB-superpage table that an earlier version of this port
 * used (a flat identity map with
 * one uniform R+W+X+U permission set covering all of code, rodata, data
 * and every thread's stack -- there was no way to tell the MMU that
 * .rodata/.text should not be writable, or that a stack should not be
 * executable).
 *
 * The code and rodata region (__rom_region_start/end) and the data/bss/
 * stacks region (from there through _image_ram_end) are mapped through
 * the L2 tables with different permissions: R+X (no W) for the former,
 * R+W (no X) for the latter. The device MMIO window and H2's own kernel
 * region don't need per-page granularity -- they keep the same direct,
 * single L1-entry 4MB mappings the previous flat map used, just built in
 * C instead of assembly.
 *
 * This *is* the only page table ever installed with H2 (built once, at
 * boot, before z_prep_c() runs) -- there is deliberately no second,
 * disjoint table: an earlier version of this file built and activated
 * one from arch_mem_map()/arch_mem_unmap(), which left the CPU walking
 * the boot table while vmclrmap() invalidated TLB state for a virtual
 * range whose only "mapping" lived in that unused second table,
 * corrupting unrelated code-page translations. CONFIG_KERNEL_DIRECT_MAP
 * still makes every arch_mem_map() request resolve virt == phys, and
 * the range asked for is always already covered by this same table, so
 * arch_mem_map()/arch_mem_unmap() remain no-ops.
 *
 * Known limitation: the page tables themselves (hexagon_pgd/hexagon_l2)
 * live in the ordinary R+W+U data/bss mapping, not a supervisor-only
 * one, so user-mode code could in principle overwrite its own page
 * tables directly rather than through a syscall. Not addressed here;
 * would need its own linker section mapped without __HVM_PTE_U.
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/linker/linker-defs.h>
#include <hexagon_vm.h>

/* L1 (PGD) geometry: 1024 entries, each spanning 4MB of address space. */
#define HEX_PGD_ENTRIES  1024
#define HEX_PGDIR_SHIFT  22
#define HEX_PGDIR_SIZE   (1U << HEX_PGDIR_SHIFT)

/*
 * L2 page size for the ROM/RAM split. Must match the alignment the linker
 * script (soc/qemu/hexagon/linker.ld) actually uses for the boundary
 * between them -- HEXAGON_ROM_RAM_ALIGN there, kept in sync with the
 * 0x10000 here by the runtime check in hexagon_mmu_init() below, since a
 * BUILD_ASSERT can't see the linker script's value directly.
 *
 * This is deliberately its own constant, not CONFIG_MMU_PAGE_SIZE (4KB):
 * an earlier version of this file used that directly, which is correct
 * but not efficient. A single H2 second-level page table uses one page
 * size for every entry across the whole 4MB PGD entry it covers (HVM
 * spec S9.4), and this port's images are typically well under 4MB, so
 * ROM and RAM share one such table; at 4KB granularity that means a TLB
 * fill for every 4KB of code or data ever touched. Under heavy host CPU
 * contention (e.g. many qemu-system-hexagon instances running at once in
 * a test sweep) that was enough extra trap-and-refill overhead to make
 * some larger images time out that ran fine standalone -- reliably
 * reproduced at 4KB granularity, gone at 64KB. See HEXAGON_ROM_RAM_ALIGN
 * in the linker script for the full explanation.
 */
#define HEX_PAGE_SIZE       0x10000
#define HEX_PAGE_SHIFT      16
#define HEX_L2_ENTRIES      (HEX_PGDIR_SIZE / HEX_PAGE_SIZE)
#define HEX_L2_TABLE_BYTES  (HEX_L2_ENTRIES * sizeof(uint32_t))

BUILD_ASSERT(HEX_PAGE_SIZE == (1 << HEX_PAGE_SHIFT), "HEX_PAGE_SHIFT out of sync");
#define HEX_PDE_S __HVM_PDE_S_64KB /* keep in sync with HEX_PAGE_SIZE above */

/* PDE (Figure 9-3 in the HVM spec): L2 table address | S field only --
 * no permission bits at this level. L2 tables must be aligned to their
 * own size (256 bytes for 64 4-byte entries, at the 64KB HEX_PAGE_SIZE
 * above).
 */
#define HEX_PDE_ADDR_MASK   (~(HEX_L2_TABLE_BYTES - 1))

/*
 * Number of L2 tables to pre-allocate. Each covers 4MB of ROM+RAM. This
 * port's only board has 16MB of RAM total, so 8 (32MB) is already more
 * than could ever be needed; hexagon_mmu_init() asserts rather than
 * silently overrunning the pool if that ever changes.
 */
#define HEX_MAX_L2_TABLES 8

static uint32_t hexagon_pgd[HEX_PGD_ENTRIES] __aligned(4096);
static uint32_t hexagon_l2[HEX_MAX_L2_TABLES][HEX_L2_ENTRIES] __aligned(HEX_L2_TABLE_BYTES);
static uint32_t hexagon_l2_next;

static uint32_t *hexagon_l2_alloc(void)
{
	__ASSERT(hexagon_l2_next < HEX_MAX_L2_TABLES,
		 "hexagon L2 page table pool exhausted (%u/%u)",
		 hexagon_l2_next, HEX_MAX_L2_TABLES);

	return hexagon_l2[hexagon_l2_next++];
}

/* Get (allocating if necessary) the L2 table backing the 4MB PGD entry
 * that virt falls in.
 */
static uint32_t *hexagon_l2_table_for(uintptr_t virt)
{
	uint32_t pgd_idx = virt >> HEX_PGDIR_SHIFT;
	uint32_t pde = hexagon_pgd[pgd_idx];
	uint32_t *l2;

	if (pde == 0) {
		l2 = hexagon_l2_alloc();
		memset(l2, 0, HEX_L2_TABLE_BYTES);
		hexagon_pgd[pgd_idx] = (((uint32_t)(uintptr_t)l2) & HEX_PDE_ADDR_MASK) |
				       HEX_PDE_S;
	} else {
		l2 = (uint32_t *)(uintptr_t)(pde & HEX_PDE_ADDR_MASK);
	}

	return l2;
}

/* Map [virt_start, virt_end) (both HEX_PAGE_SIZE-aligned) identity
 * (phys == virt, per CONFIG_KERNEL_DIRECT_MAP) through the L2 tables
 * with the given permission and cache-attribute bits.
 */
static void hexagon_l2_map_range(uintptr_t virt_start, uintptr_t virt_end,
				 uint32_t pte_perm_bits, uint32_t cache_attr)
{
	uint32_t pte_flags = pte_perm_bits | (cache_attr << 6);
	uintptr_t va;

	__ASSERT((virt_start & (HEX_PAGE_SIZE - 1)) == 0,
		 "unaligned range start 0x%lx", (unsigned long)virt_start);
	__ASSERT((virt_end & (HEX_PAGE_SIZE - 1)) == 0,
		 "unaligned range end 0x%lx", (unsigned long)virt_end);

	for (va = virt_start; va < virt_end; va += HEX_PAGE_SIZE) {
		uint32_t *l2 = hexagon_l2_table_for(va);
		uint32_t idx = (va >> HEX_PAGE_SHIFT) & (HEX_L2_ENTRIES - 1);

		l2[idx] = (uint32_t)va | pte_flags;
	}
}

/* Set a single direct L1 PTE (Figure 9-4: S=4MB, permissions and cache
 * attributes embedded in the entry itself, no L2 table). phys must be
 * 4MB-aligned. Used for the device MMIO window and H2's own kernel
 * region, which don't need per-page permission splits.
 */
static void hexagon_l1_map_4mb(uintptr_t phys, uint32_t pte_perm_bits, uint32_t cache_attr)
{
	__ASSERT((phys & (HEX_PGDIR_SIZE - 1)) == 0,
		 "unaligned 4MB mapping 0x%lx", (unsigned long)phys);

	hexagon_pgd[phys >> HEX_PGDIR_SHIFT] =
		(uint32_t)phys | pte_perm_bits | (cache_attr << 6) | __HVM_PDE_S_4MB;
}

void hexagon_mmu_init(void)
{
	uintptr_t rom_start = (uintptr_t)__rom_region_start;
	uintptr_t rom_end = ROUND_UP((uintptr_t)__rom_region_end, HEX_PAGE_SIZE);
	uintptr_t ram_end = ROUND_UP((uintptr_t)_image_ram_end, HEX_PAGE_SIZE);
	int32_t ret;

	/*
	 * rom_start is __start, the very first thing linked into RAM --
	 * its address is the board's SRAM origin, fixed by devicetree/
	 * hardware, not something the linker script itself aligns. Every
	 * board this port targets happens to have a page-aligned (indeed
	 * far more than page-aligned) SRAM origin; assert rather than
	 * silently mis-map if that's ever not true.
	 */
	__ASSERT((rom_start & (HEX_PAGE_SIZE - 1)) == 0,
		 "SRAM origin 0x%lx is not page-aligned", (unsigned long)rom_start);

	/*
	 * rom_end must land exactly on _image_ram_start: this file's
	 * HEX_PAGE_SIZE and the linker script's HEXAGON_ROM_RAM_ALIGN have
	 * to agree for the same reason neither can be CONFIG_MMU_PAGE_SIZE
	 * alone (see the comments on both). A silent mismatch here either
	 * folds real writable data into the read-only ROM mapping or opens
	 * an unintended gap -- exactly the class of bug this assert exists
	 * to catch instead of a hard-to-diagnose fault deep in unrelated
	 * code the first time something touches the misplaced boundary.
	 */
	__ASSERT(rom_end == (uintptr_t)_image_ram_start,
		 "HEX_PAGE_SIZE (0x%x) and linker.ld's HEXAGON_ROM_RAM_ALIGN disagree: "
		 "rom_end=0x%lx != _image_ram_start=0x%lx",
		 HEX_PAGE_SIZE, (unsigned long)rom_end, (unsigned long)_image_ram_start);

	memset(hexagon_pgd, 0, sizeof(hexagon_pgd));
	hexagon_l2_next = 0;

	/* .text + .rodata: readable and executable, never writable. */
	hexagon_l2_map_range(rom_start, rom_end,
			     __HVM_PTE_R | __HVM_PTE_X | __HVM_PTE_U, __HEXAGON_C_WB_L2);

	/*
	 * .data/.bss/.noinit/stacks/heap: readable and writable, never
	 * executable. rom_end is exactly where this region starts: the
	 * linker script MMU-page-aligns the RAM section's first byte
	 * (soc/qemu/hexagon/linker.ld), with nothing but that alignment
	 * gap -- itself harmlessly covered by the ROM mapping above --
	 * between __rom_region_end and it.
	 */
	hexagon_l2_map_range(rom_end, ram_end,
			     __HVM_PTE_R | __HVM_PTE_W | __HVM_PTE_U, __HEXAGON_C_WB_L2);

	/*
	 * PL011 UART at 0x10000000: supervisor-only (no U), matching the
	 * previous flat map. SHARED bypasses H2's guestmap translation so
	 * the guest physical address is used directly as the host address.
	 */
	hexagon_l1_map_4mb(0x10000000, __HVM_PTE_R | __HVM_PTE_W | __HVM_PTE_SHARED,
			   __HEXAGON_C_DEV);

	/* H2's own kernel, two 4MB regions: supervisor-only RWX, matching
	 * the previous flat map.
	 */
	hexagon_l1_map_4mb(0x9b800000, __HVM_PTE_R | __HVM_PTE_W | __HVM_PTE_X,
			   __HEXAGON_C_WB_L2);
	hexagon_l1_map_4mb(0x9bc00000, __HVM_PTE_R | __HVM_PTE_W | __HVM_PTE_X,
			   __HEXAGON_C_WB_L2);

	/* Flush so H2's table walker sees the freshly written entries. */
	hexagon_vm_cache(hvmc_dccleaninva, 0, 0);

	ret = hexagon_vm_newmap(hexagon_pgd, VM_TRANS_TYPE_TABLE, VM_TLB_INVALIDATE_TRUE);
	if (ret != 0) {
		hexagon_vm_stop(VM_STOP_HALT);
	}
}

#ifdef CONFIG_MMU

/*
 * CONFIG_KERNEL_DIRECT_MAP makes every arch_mem_map() request (device
 * MMIO via DEVICE_MMIO_MAP(), or an explicit K_MEM_DIRECT_MAP caller)
 * resolve virt == phys, so the range asked for is always already
 * covered by the table hexagon_mmu_init() installed. Nothing left to do
 * here; see the file header for why a second, dynamically-modified
 * table is deliberately not built.
 */
int arch_mem_map(void *virt, uintptr_t phys, size_t size, uint32_t flags)
{
	ARG_UNUSED(flags);

	__ASSERT((uintptr_t)virt == phys,
		 "hexagon requires CONFIG_KERNEL_DIRECT_MAP (virt %p != phys 0x%lx)",
		 virt, phys);
	ARG_UNUSED(size);

	return (uintptr_t)virt == phys ? 0 : -EINVAL;
}

int arch_mem_unmap(void *addr, size_t size)
{
	ARG_UNUSED(addr);
	ARG_UNUSED(size);

	return 0;
}

int arch_page_phys_get(void *virt, uintptr_t *phys)
{
	if (phys != NULL) {
		*phys = (uintptr_t)virt;
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
