/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_SOC_QCOM_LEMANS_INCLUDE_LEMANS_MMU_H_
#define ZEPHYR_SOC_QCOM_LEMANS_INCLUDE_LEMANS_MMU_H_

#include <stdint.h>
#include <stddef.h>

#include <zephyr/sys/util.h>

#include <lemans_runtime.h>
#include <lemans_tlb.h>

struct lemans_native_mapping {
	uint32_t virtual_address;
	uint64_t physical_address;
	uint32_t size;
	uint32_t page_count;
	uint32_t tlb_hi;
	uint32_t tlb_lo;
	uint8_t permissions;
	uint8_t cacheability;
};

/*
 * This table is a description of the first identity-map contract only. It is
 * intentionally not consumed by reset code yet: programming tlbw and enabling
 * SYSCFG/MMU require a separately validated v73QA_1 transition sequence.
 */
extern const struct lemans_native_mapping z_lemans_native_mappings[];
extern const size_t z_lemans_native_mapping_count;

#endif /* ZEPHYR_SOC_QCOM_LEMANS_INCLUDE_LEMANS_MMU_H_ */
