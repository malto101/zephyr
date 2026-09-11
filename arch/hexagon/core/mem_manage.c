/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Hexagon MMU management via the H2 hypervisor page table interface.
 *
 * The boot-time reset handler (hvm_event_vectors.S:_setup_page_table)
 * builds a single-level, 4MB-superpage table that identity-maps all of
 * SRAM, RAM and the device MMIO window, then activates it via the
 * vmnewmap hypercall before z_prep_c() runs. That table is the only one
 * ever installed with H2: it stays resident and covers every physical
 * address Zephyr uses on this board.
 *
 * CONFIG_KERNEL_DIRECT_MAP therefore makes every arch_mem_map() request
 * (device MMIO via DEVICE_MMIO_MAP(), or an explicit K_MEM_DIRECT_MAP
 * caller) resolve virt == phys, so the range asked for is always already
 * covered by that identity map. arch_mem_map()/arch_mem_unmap() have
 * nothing left to do: building and activating a second, disjoint page
 * table here -- as an earlier version of this file did -- left the CPU
 * walking the boot table while vmclrmap() invalidated TLB state for a
 * virtual range whose only "mapping" lived in the unused second table,
 * corrupting unrelated code-page translations.
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/__assert.h>

#ifdef CONFIG_MMU

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

#include <hexagon_vm.h>

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
