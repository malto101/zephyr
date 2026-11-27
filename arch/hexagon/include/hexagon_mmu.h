/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_ARCH_HEXAGON_INCLUDE_HEXAGON_MMU_H_
#define ZEPHYR_ARCH_HEXAGON_INCLUDE_HEXAGON_MMU_H_

#include <stddef.h>
#include <stdint.h>
#include <hexagon_vm.h>

/* Linear list every installed map ends with, see mmu.c. */
extern struct hexagon_linear_entry hexagon_fixed_tail[];

/* Map installed while no user thread's map is. */
extern struct hexagon_linear_entry hexagon_kernel_map[];

/*
 * Chain entry to the runtime mappings list, which chains on to
 * hexagon_fixed_tail. Every installed map reaches the tail through it.
 */
extern struct hexagon_linear_entry hexagon_dyn_link[];

/* Point hexagon_dyn_link at the runtime mappings, see mem_manage.c. */
void hexagon_dyn_map_init(void);

/*
 * Note that vmclrmap() flushed the installed map's ASID after a
 * translation was removed: every other map must flush on its next
 * install. Defined with CONFIG_USERSPACE, see userspace.c.
 */
void hexagon_map_flushed(void);

/*
 * 1 if the page-aligned [va, va + size) lies inside one identity range
 * of the boot-time map (image, console window, HVM kernel), 0 if it is
 * disjoint from all of them, -1 if it partially overlaps one.
 */
int hexagon_fixed_map_contains(uintptr_t va, size_t size);

/* Round [*addr, *addr + *size) outward to whole 4KB pages. */
void hexagon_align_region_to_page(uintptr_t *addr, size_t *size);

/*
 * Map the page-aligned [addr, addr + size) 1:1 with at most max_entries
 * linear entries written to entries[]. Return the number used.
 */
uint32_t hexagon_decompose_region(struct hexagon_linear_entry *entries, uint32_t max_entries,
				  uintptr_t addr, size_t size, uint32_t xwru, uint32_t cache_attr);

/*
 * Map the page-aligned [va, va + size) to pa with at most max_entries
 * linear entries written to entries[]. Return the number used, or
 * -ENOMEM if they cannot cover the whole range.
 */
int hexagon_decompose_mapping(struct hexagon_linear_entry *entries, uint32_t max_entries,
			      uintptr_t va, uintptr_t pa, size_t size, uint32_t xwru,
			      uint32_t cache_attr, uint32_t extra_high);

#endif /* ZEPHYR_ARCH_HEXAGON_INCLUDE_HEXAGON_MMU_H_ */
