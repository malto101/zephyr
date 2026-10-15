/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stddef.h>
#include <stdint.h>

#include <zephyr/sys/util.h>

#include <lemans_mmu.h>

#define LEMANS_RAM_MAPPING(offset) \
	{ \
		.virtual_address = LEMANS_CARVEOUT_BASE + (offset), \
		.physical_address = LEMANS_CARVEOUT_BASE + (offset), \
		.size = 0x00100000U, \
		.page_count = LEMANS_TLB_PAGES_1M, \
		.tlb_hi = LEMANS_TLB_HI(LEMANS_CARVEOUT_BASE + (offset), \
					LEMANS_CARVEOUT_BASE + (offset)), \
		.tlb_lo = LEMANS_TLB_LO(LEMANS_CARVEOUT_BASE + (offset), \
					LEMANS_TLB_SIZE_BIT_1M, \
					LEMANS_TLB_PERM_READ | LEMANS_TLB_PERM_WRITE | \
					LEMANS_TLB_PERM_EXECUTE, \
					LEMANS_TLB_CACHE_WRITEBACK), \
		.permissions = LEMANS_TLB_PERM_READ | LEMANS_TLB_PERM_WRITE | \
			LEMANS_TLB_PERM_EXECUTE, \
		.cacheability = LEMANS_TLB_CACHE_WRITEBACK, \
	}

#define LEMANS_DEVICE_RANGE_MAPPING(address, mapping_size, size_bit, pages) \
	{ \
		.virtual_address = (address), \
		.physical_address = (address), \
		.size = (mapping_size), \
		.page_count = (pages), \
		.tlb_hi = LEMANS_TLB_HI((address), (address)), \
		.tlb_lo = LEMANS_TLB_LO((address), (size_bit), \
					LEMANS_TLB_PERM_READ | LEMANS_TLB_PERM_WRITE, \
					LEMANS_TLB_CACHE_DEVICE), \
		.permissions = LEMANS_TLB_PERM_READ | LEMANS_TLB_PERM_WRITE, \
		.cacheability = LEMANS_TLB_CACHE_DEVICE, \
	}

#define LEMANS_DEVICE_MAPPING(address) \
	LEMANS_DEVICE_RANGE_MAPPING(address, LEMANS_TLB_PAGE_SIZE, \
				    LEMANS_TLB_SIZE_BIT_4K, 1U)

#define LEMANS_DEVICE_256K_MAPPING(address) \
	LEMANS_DEVICE_RANGE_MAPPING(address, 0x00040000U, \
				    LEMANS_TLB_SIZE_BIT_256K, LEMANS_TLB_PAGES_256K)

#define LEMANS_DEVICE_1M_MAPPING(address) \
	LEMANS_DEVICE_RANGE_MAPPING(address, 0x00100000U, \
				    LEMANS_TLB_SIZE_BIT_1M, LEMANS_TLB_PAGES_1M)

#define LEMANS_SHARED_MAPPING(address) \
	{ \
		.virtual_address = (address), \
		.physical_address = (address), \
		.size = 0x00100000U, \
		.page_count = LEMANS_TLB_PAGES_1M, \
		.tlb_hi = LEMANS_TLB_HI((address), (address)), \
		.tlb_lo = LEMANS_TLB_LO((address), LEMANS_TLB_SIZE_BIT_1M, \
					LEMANS_TLB_PERM_READ | LEMANS_TLB_PERM_WRITE, \
					LEMANS_TLB_CACHE_NONE_SHARED), \
		.permissions = LEMANS_TLB_PERM_READ | LEMANS_TLB_PERM_WRITE, \
		.cacheability = LEMANS_TLB_CACHE_NONE_SHARED, \
	}

BUILD_ASSERT(LEMANS_TLB_PAGE_SIZE == 0x1000U,
	"Lemans native TLB page size must be 4 KiB");
BUILD_ASSERT(LEMANS_CARVEOUT_BASE <= UINT32_MAX - LEMANS_CARVEOUT_SIZE,
	"Lemans carveout exceeds the 32-bit direct-map contract");
BUILD_ASSERT(LEMANS_CARVEOUT_SIZE == 0x01e00000U,
	"Lemans native mapping table expects a 30 MiB carveout");
BUILD_ASSERT((LEMANS_CARVEOUT_BASE & 0x000fffffU) == 0U,
	"Lemans carveout must be 1 MiB aligned for the first mapping table");
BUILD_ASSERT((LEMANS_SMEM_BASE & 0x000fffffU) == 0U,
	"Lemans SMEM must be 1 MiB aligned for shared mappings");
BUILD_ASSERT((LEMANS_SMEM_SIZE & 0x000fffffU) == 0U,
	"Lemans SMEM size must be 1 MiB aligned");
BUILD_ASSERT((LEMANS_IPCC_BASE & 0x0003ffffU) == 0U,
	"Lemans IPCC must be 256 KiB aligned");
BUILD_ASSERT(LEMANS_QDSP6SS_SIZE >= 0x1000U,
	"Lemans QDSP6SS window is too small");
BUILD_ASSERT(LEMANS_L2VIC_SIZE >= 0x1000U,
	"Lemans L2VIC window is too small");
BUILD_ASSERT(LEMANS_QTIMER_AC_SIZE >= 0x44U,
	"Lemans QTimer access-control window is too small");
BUILD_ASSERT(LEMANS_QTIMER_FRAME_SIZE >= 0x30U,
	"Lemans QTimer frame window is too small");
BUILD_ASSERT(LEMANS_WDOG_SIZE >= 0x10U,
	"Lemans watchdog window is too small");
BUILD_ASSERT(LEMANS_IPCC_SIZE >= 0x40000U,
	"Lemans IPCC window is too small");
BUILD_ASSERT(LEMANS_TCSR_SIZE >= 0x1000U,
	"Lemans TCSR window is too small");
BUILD_ASSERT((LEMANS_TCSR_SMEM_TARG_INFO_LO & LEMANS_TLB_PAGE_MASK) == 0U,
	"Lemans SMEM target-info register must be page aligned");
BUILD_ASSERT(LEMANS_TCSR_SMEM_TARG_INFO_HI ==
		LEMANS_TCSR_SMEM_TARG_INFO_LO + 4U,
	"Lemans SMEM target-info registers must be adjacent");

/*
 * These entries describe, but do not program, the first direct-boot map.
 * They are split at the hardware-supported page sizes so each tlb_lo value
 * has an unambiguous size encoding. The reset path must not consume them
 * until the v73QA_1 TLB-match workaround and MMU transition are validated.
 */
const struct lemans_native_mapping z_lemans_native_mappings[] = {
	LEMANS_RAM_MAPPING(0x000000U),
	LEMANS_RAM_MAPPING(0x100000U),
	LEMANS_RAM_MAPPING(0x200000U),
	LEMANS_RAM_MAPPING(0x300000U),
	LEMANS_RAM_MAPPING(0x400000U),
	LEMANS_RAM_MAPPING(0x500000U),
	LEMANS_RAM_MAPPING(0x600000U),
	LEMANS_RAM_MAPPING(0x700000U),
	LEMANS_RAM_MAPPING(0x800000U),
	LEMANS_RAM_MAPPING(0x900000U),
	LEMANS_RAM_MAPPING(0xa00000U),
	LEMANS_RAM_MAPPING(0xb00000U),
	LEMANS_RAM_MAPPING(0xc00000U),
	LEMANS_RAM_MAPPING(0xd00000U),
	LEMANS_RAM_MAPPING(0xe00000U),
	LEMANS_RAM_MAPPING(0xf00000U),
	LEMANS_RAM_MAPPING(0x1000000U),
	LEMANS_RAM_MAPPING(0x1100000U),
	LEMANS_RAM_MAPPING(0x1200000U),
	LEMANS_RAM_MAPPING(0x1300000U),
	LEMANS_RAM_MAPPING(0x1400000U),
	LEMANS_RAM_MAPPING(0x1500000U),
	LEMANS_RAM_MAPPING(0x1600000U),
	LEMANS_RAM_MAPPING(0x1700000U),
	LEMANS_RAM_MAPPING(0x1800000U),
	LEMANS_RAM_MAPPING(0x1900000U),
	LEMANS_RAM_MAPPING(0x1a00000U),
	LEMANS_RAM_MAPPING(0x1b00000U),
	LEMANS_RAM_MAPPING(0x1c00000U),
	LEMANS_RAM_MAPPING(0x1d00000U),

	LEMANS_DEVICE_MAPPING(LEMANS_QDSP6SS_BASE),
	LEMANS_DEVICE_MAPPING(LEMANS_L2VIC_BASE),
	LEMANS_DEVICE_MAPPING(LEMANS_QTIMER_AC_BASE),
	LEMANS_DEVICE_MAPPING(LEMANS_QTIMER_FRAME_BASE),
	LEMANS_DEVICE_MAPPING(LEMANS_WDOG_BASE),
	LEMANS_DEVICE_256K_MAPPING(LEMANS_IPCC_BASE),
	LEMANS_DEVICE_MAPPING(LEMANS_TCSR_SMEM_TARG_INFO_LO &
				      ~LEMANS_TLB_PAGE_MASK),
	LEMANS_SHARED_MAPPING(LEMANS_SMEM_BASE),
	LEMANS_SHARED_MAPPING(LEMANS_SMEM_BASE + 0x00100000U),
};

const size_t z_lemans_native_mapping_count =
	ARRAY_SIZE(z_lemans_native_mappings);
