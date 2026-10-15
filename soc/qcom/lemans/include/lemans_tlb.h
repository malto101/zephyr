/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_SOC_QCOM_LEMANS_INCLUDE_LEMANS_TLB_H_
#define ZEPHYR_SOC_QCOM_LEMANS_INCLUDE_LEMANS_TLB_H_

#include <stdint.h>

#include <zephyr/sys/util.h>

/* Native v73 32-bit virtual address/38-bit physical address format. */
#define LEMANS_TLB_PAGE_SHIFT 12U
#define LEMANS_TLB_PAGE_SIZE  (1U << LEMANS_TLB_PAGE_SHIFT)
#define LEMANS_TLB_PAGE_MASK  (LEMANS_TLB_PAGE_SIZE - 1U)
#define LEMANS_TLB_ENTRY_COUNT 128U

#define LEMANS_TLB_VPAGE_BITS 20U
#define LEMANS_TLB_VPAGE_MASK ((1U << LEMANS_TLB_VPAGE_BITS) - 1U)
#define LEMANS_TLB_PPAGE_BITS 23U
#define LEMANS_TLB_PPAGE_MASK ((UINT64_C(1) << LEMANS_TLB_PPAGE_BITS) - UINT64_C(1))
#define LEMANS_TLB_PHYS_ADDR_BITS 38U
#define LEMANS_TLB_PHYS_ADDR_MASK \
	((UINT64_C(1) << LEMANS_TLB_PHYS_ADDR_BITS) - UINT64_C(1))

#define LEMANS_TLB_PPAGE_SHIFT 1U
#define LEMANS_TLB_PPAGE_HIGH_BIT_POS 27U
#define LEMANS_TLB_PPAGE_HIGH_OFFSET 24U
#define LEMANS_TLB_PPAGE_EP_BIT_POS 29U
#define LEMANS_TLB_PPAGE_EP_OFFSET 23U
#define LEMANS_TLB_ASID_BITS 7U
#define LEMANS_TLB_ASID_POS 20U
#define LEMANS_TLB_GLOBAL_BIT_POS 30U
#define LEMANS_TLB_VALID_BIT_POS 31U

/* XWRU is stored in tlb_lo[31:28], with U at the least-significant bit. */
#define LEMANS_TLB_PERM_USER    0x1U
#define LEMANS_TLB_PERM_READ    0x2U
#define LEMANS_TLB_PERM_WRITE   0x4U
#define LEMANS_TLB_PERM_EXECUTE 0x8U
#define LEMANS_TLB_PERM_MASK    0xfU
#define LEMANS_TLB_PERM_SUPERVISOR 0U
#define LEMANS_TLB_PERM_OFFSET  28U
#define LEMANS_TLB_PERM_XWRU_MASK \
	(LEMANS_TLB_PERM_MASK << LEMANS_TLB_PERM_OFFSET)

BUILD_ASSERT(LEMANS_TLB_PERM_READ == BIT(1),
	"Lemans TLB read permission must encode XWRU.R");
BUILD_ASSERT(LEMANS_TLB_PERM_WRITE == BIT(2),
	"Lemans TLB write permission must encode XWRU.W");
BUILD_ASSERT(LEMANS_TLB_PERM_EXECUTE == BIT(3),
	"Lemans TLB execute permission must encode XWRU.X");
BUILD_ASSERT(LEMANS_TLB_PERM_USER == BIT(0),
	"Lemans TLB user permission must encode XWRU.U");
BUILD_ASSERT(LEMANS_TLB_PERM_XWRU_MASK == UINT32_C(0xf0000000),
	"Lemans TLB XWRU field must occupy tlb_lo[31:28]");

/* CCCC values from the v73 32VA/38PA format. */
#define LEMANS_TLB_CACHE_DEVICE       0x4U
#define LEMANS_TLB_CACHE_NONE_SHARED  0x6U
#define LEMANS_TLB_CACHE_WRITEBACK    0x7U
#define LEMANS_TLB_CACHE_MASK         0xfU
#define LEMANS_TLB_CACHE_OFFSET       24U

/* The page-size field is a one-hot bit number, not a page shift. */
#define LEMANS_TLB_SIZE_BIT_4K   0U
#define LEMANS_TLB_SIZE_BIT_16K  1U
#define LEMANS_TLB_SIZE_BIT_64K  2U
#define LEMANS_TLB_SIZE_BIT_256K 3U
#define LEMANS_TLB_SIZE_BIT_1M   4U
#define LEMANS_TLB_SIZE_BIT_4M   5U
#define LEMANS_TLB_SIZE_BIT_16M  6U
#define LEMANS_TLB_SIZE_BIT_64M  7U
#define LEMANS_TLB_SIZE_BIT_256M 8U
#define LEMANS_TLB_SIZE_BIT_1G   9U

#define LEMANS_TLB_PAGES_1M  256U
#define LEMANS_TLB_PAGES_4M  1024U
#define LEMANS_TLB_PAGES_16M 4096U
#define LEMANS_TLB_PAGES_256K 64U

BUILD_ASSERT(LEMANS_TLB_SIZE_BIT_4K == 0U,
	"Lemans TLB 4 KiB size encoding changed");
BUILD_ASSERT(LEMANS_TLB_SIZE_BIT_1M == 4U,
	"Lemans TLB 1 MiB size encoding changed");
BUILD_ASSERT(LEMANS_TLB_SIZE_BIT_4M == 5U,
	"Lemans TLB 4 MiB size encoding changed");
BUILD_ASSERT(LEMANS_TLB_SIZE_BIT_16M == 6U,
	"Lemans TLB 16 MiB size encoding changed");
BUILD_ASSERT(LEMANS_TLB_SIZE_BIT_1G == 9U,
	"Lemans TLB 1 GiB size encoding changed");

/*
 * Encode a virtual page, physical page, global/valid mapping, and ASID zero.
 * The mapping description uses global entries, so the ASID field is zero and
 * the global bit makes it irrelevant to matching.
 */
#define LEMANS_TLB_HI(va, pa) \
	(((((uint32_t)(va) >> LEMANS_TLB_PAGE_SHIFT) & \
	   LEMANS_TLB_VPAGE_MASK)) | \
	 ((((uint64_t)(pa) >> (LEMANS_TLB_PAGE_SHIFT + LEMANS_TLB_PPAGE_BITS)) & \
	   0x1ULL) << LEMANS_TLB_PPAGE_EP_BIT_POS) | \
	 ((((uint64_t)(pa) >> (LEMANS_TLB_PAGE_SHIFT + LEMANS_TLB_PPAGE_HIGH_OFFSET)) & \
	   0x3ULL) << LEMANS_TLB_PPAGE_HIGH_BIT_POS) | \
	 (1U << LEMANS_TLB_GLOBAL_BIT_POS) | \
	 (1U << LEMANS_TLB_VALID_BIT_POS))

#define LEMANS_TLB_LO(pa, size_bit, perms, cache) \
	((uint32_t)(((((uint64_t)(pa) >> LEMANS_TLB_PAGE_SHIFT) & \
		      LEMANS_TLB_PPAGE_MASK) << LEMANS_TLB_PPAGE_SHIFT) | \
		    (1U << (size_bit)) | \
		    ((((uint32_t)(cache) & LEMANS_TLB_CACHE_MASK) << \
		      LEMANS_TLB_CACHE_OFFSET)) | \
		    ((((uint32_t)(perms) & LEMANS_TLB_PERM_MASK) << \
		      LEMANS_TLB_PERM_OFFSET))))

#endif /* ZEPHYR_SOC_QCOM_LEMANS_INCLUDE_LEMANS_TLB_H_ */
