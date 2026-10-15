/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/ztest.h>

#include <lemans_tlb.h>

/* Independent constants for the 4 KiB reference mapping. */
#define TEST_TLB_HI_REFERENCE \
	(UINT32_C(0xc0000000) | UINT32_C(0x00081234))
#define TEST_TLB_LO_REFERENCE \
	(UINT32_C(0xe0000000) | UINT32_C(0x04000000) | \
	 UINT32_C(0x00000001) | UINT32_C(0x00136a00))

#define TEST_RANGES_OVERLAP(first_base, first_size, second_base, second_size) \
	(((uint64_t)(first_base) < ((uint64_t)(second_base) + (second_size))) && \
	 ((uint64_t)(second_base) < ((uint64_t)(first_base) + (first_size))))

struct test_range {
	uint32_t base;
	uint32_t size;
};

static bool test_range_is_valid(const struct test_range *range)
{
	uint64_t end = (uint64_t)range->base + range->size;

	return range->size != 0U &&
	       (range->base & LEMANS_TLB_PAGE_MASK) == 0U &&
	       (range->size & LEMANS_TLB_PAGE_MASK) == 0U &&
	       end <= (UINT64_C(1) << 32);
}

static bool test_ranges_overlap(const struct test_range *first,
				const struct test_range *second)
{
	return TEST_RANGES_OVERLAP(first->base, first->size,
				   second->base, second->size);
}

BUILD_ASSERT(LEMANS_TLB_HI(0x81234000U, UINT64_C(0x9b500000)) ==
		     TEST_TLB_HI_REFERENCE, "TLB high-word reference mismatch");
BUILD_ASSERT(LEMANS_TLB_LO(UINT64_C(0x9b500000), LEMANS_TLB_SIZE_BIT_4K,
			    LEMANS_TLB_PERM_READ | LEMANS_TLB_PERM_WRITE |
			    LEMANS_TLB_PERM_EXECUTE, LEMANS_TLB_CACHE_DEVICE) ==
		     TEST_TLB_LO_REFERENCE, "TLB low-word reference mismatch");
BUILD_ASSERT(!TEST_RANGES_OVERLAP(UINT32_C(0x9b500000), UINT32_C(0x100000),
				  UINT32_C(0x9b600000), UINT32_C(0x100000)),
		     "non-overlapping TLB ranges were rejected");
BUILD_ASSERT(TEST_RANGES_OVERLAP(UINT32_C(0x9b500000), UINT32_C(0x100000),
				 UINT32_C(0x9b5ff000), UINT32_C(0x1000)),
		     "overlapping TLB ranges were not detected");
BUILD_ASSERT((UINT64_C(0xfffff000) + UINT32_C(0x2000)) > UINT32_MAX,
		     "32-bit VA wraparound test is ineffective");

static uint64_t test_decode_physical_page(uint32_t hi, uint32_t lo)
{
	return (((uint64_t)((lo >> LEMANS_TLB_PPAGE_SHIFT) &
			    LEMANS_TLB_PPAGE_MASK)) << LEMANS_TLB_PAGE_SHIFT) |
	       (((uint64_t)((hi >> LEMANS_TLB_PPAGE_EP_BIT_POS) & 1U)) << 35U) |
	       (((uint64_t)((hi >> LEMANS_TLB_PPAGE_HIGH_BIT_POS) & 0x3U)) << 36U);
}

ZTEST(lemans_tlb, test_4k_encoding)
{
	uint64_t pa = UINT64_C(0x9b500000);
	uint32_t hi = LEMANS_TLB_HI(UINT32_C(0x81234000), pa);
	uint32_t lo = LEMANS_TLB_LO(pa, LEMANS_TLB_SIZE_BIT_4K,
				    LEMANS_TLB_PERM_READ | LEMANS_TLB_PERM_WRITE |
				    LEMANS_TLB_PERM_EXECUTE,
				    LEMANS_TLB_CACHE_DEVICE);

	zassert_equal(hi, TEST_TLB_HI_REFERENCE, "unexpected tlb_hi: 0x%08x", hi);
	zassert_equal(lo, TEST_TLB_LO_REFERENCE, "unexpected tlb_lo: 0x%08x", lo);
	zassert_equal((lo >> LEMANS_TLB_PERM_OFFSET) & LEMANS_TLB_PERM_MASK,
			      LEMANS_TLB_PERM_READ | LEMANS_TLB_PERM_WRITE |
			      LEMANS_TLB_PERM_EXECUTE,
			      "unexpected supervisor XWRU field");
}

ZTEST(lemans_tlb, test_1m_encoding_with_high_physical_bits)
{
	uint64_t pa = UINT64_C(0x2812000000);
	uint32_t hi = LEMANS_TLB_HI(UINT32_C(0x81234000), pa);
	uint32_t lo = LEMANS_TLB_LO(pa, LEMANS_TLB_SIZE_BIT_1M,
				    LEMANS_TLB_PERM_READ | LEMANS_TLB_PERM_WRITE |
				    LEMANS_TLB_PERM_EXECUTE,
				    LEMANS_TLB_CACHE_WRITEBACK);

	zassert_equal(hi, UINT32_C(0xf0081234), "unexpected tlb_hi: 0x%08x", hi);
	zassert_equal(lo, UINT32_C(0xe7024010), "unexpected tlb_lo: 0x%08x", lo);
	zassert_equal((lo >> LEMANS_TLB_SIZE_BIT_1M) & 1U, 1U,
			      "1 MiB size bit is not set");
	zassert_equal((lo >> LEMANS_TLB_SIZE_BIT_4M) & 1U, 0U,
			      "4 MiB size bit is unexpectedly set");
}

ZTEST(lemans_tlb, test_size_encoding)
{
	zassert_equal(LEMANS_TLB_SIZE_BIT_4K, 0U, "4 KiB size bit");
	zassert_equal(LEMANS_TLB_SIZE_BIT_256K, 3U, "256 KiB size bit");
	zassert_equal(LEMANS_TLB_SIZE_BIT_1M, 4U, "1 MiB size bit");
	zassert_equal(LEMANS_TLB_SIZE_BIT_4M, 5U, "4 MiB size bit");
	zassert_equal(LEMANS_TLB_SIZE_BIT_16M, 6U, "16 MiB size bit");
	zassert_equal(LEMANS_TLB_SIZE_BIT_1G, 9U, "1 GiB size bit");
}

ZTEST(lemans_tlb, test_mapping_ranges)
{
	const struct test_range carveout = {
		.base = UINT32_C(0x9b500000),
		.size = UINT32_C(0x01e00000),
	};
	const struct test_range adjacent = {
		.base = UINT32_C(0x9d300000),
		.size = UINT32_C(0x00100000),
	};
	const struct test_range overlap = {
		.base = UINT32_C(0x9b5ff000),
		.size = UINT32_C(0x00001000),
	};
	const struct test_range wraps = {
		.base = UINT32_C(0xfffff000),
		.size = UINT32_C(0x00002000),
	};

	zassert_true(test_range_is_valid(&carveout), "carveout range is invalid");
	zassert_true(test_range_is_valid(&adjacent), "adjacent range is invalid");
	zassert_true(test_range_is_valid(&overlap), "overlap range is invalid");
	zassert_false(test_range_is_valid(&wraps), "VA wraparound was accepted");
	zassert_false(test_ranges_overlap(&carveout, &adjacent),
			     "adjacent ranges overlap");
	zassert_true(test_ranges_overlap(&carveout, &overlap),
			    "overlapping ranges were not detected");
}

ZTEST(lemans_tlb, test_physical_page_split)
{
	uint64_t pa = UINT64_C(0x3abcde000);
	uint32_t hi = LEMANS_TLB_HI(UINT32_C(0x81234000), pa);
	uint32_t lo = LEMANS_TLB_LO(pa, LEMANS_TLB_SIZE_BIT_4K,
				    LEMANS_TLB_PERM_READ,
				    LEMANS_TLB_CACHE_DEVICE);

	zassert_equal(test_decode_physical_page(hi, lo), pa,
			      "physical page was not split/recombined");
}

ZTEST_SUITE(lemans_tlb, NULL, NULL, NULL, NULL, NULL);
