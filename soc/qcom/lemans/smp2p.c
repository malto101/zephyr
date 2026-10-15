/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <errno.h>

#include <zephyr/init.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>
#include <zephyr/arch/common/sys_io.h>

#include <lemans_runtime.h>

#define LEMANS_SMEM_TARG_INFO_LO         LEMANS_TCSR_SMEM_TARG_INFO_LO
#define LEMANS_SMEM_TARG_INFO_HI         LEMANS_TCSR_SMEM_TARG_INFO_HI

#define LEMANS_SMP2P_ITEM_SIZE           0x154U

#define LEMANS_SMEM_LOCAL_HOST           5U
#define LEMANS_SMEM_REMOTE_HOST          0U
#define LEMANS_SMP2P_ITEM                94U

#define LEMANS_SMEM_TARG_INFO_MAGIC      0x49494953U
#define LEMANS_SMEM_TOC_MAGIC            0x434F5424U
#define LEMANS_SMEM_PARTITION_MAGIC      0x54525024U
#define LEMANS_SMEM_ALLOC_CANARY         0xA5A5U

#define LEMANS_SMEM_TOC_SIZE             4096U
#define LEMANS_SMEM_TOC_HEADER_SIZE      32U
#define LEMANS_SMEM_TOC_ENTRY_SIZE       48U
#define LEMANS_SMEM_TOC_MAX_ENTRIES      60U
#define LEMANS_SMEM_PARTITION_HEADER_SIZE 32U
#define LEMANS_SMEM_ALLOC_HEADER_SIZE    16U

#define LEMANS_SMP2P_MAGIC               0x504D5324U
#define LEMANS_SMP2P_VERSION             1U
#define LEMANS_SMP2P_FEATURES            1U
#define LEMANS_SMP2P_ENTRY_COUNT         16U
#define LEMANS_SMP2P_ENTRY_SIZE          20U
#define LEMANS_SMP2P_HEADER_SIZE         20U
#define LEMANS_SMP2P_SIZE \
	(LEMANS_SMP2P_HEADER_SIZE + (LEMANS_SMP2P_ENTRY_COUNT * LEMANS_SMP2P_ENTRY_SIZE))
#define LEMANS_SMP2P_ENTRY_NAME          "slave-kernel"
#define LEMANS_SMP2P_READY               0x00000002U
#define LEMANS_SMP2P_HANDOVER            0x00000006U

BUILD_ASSERT(LEMANS_SMP2P_SIZE == LEMANS_SMP2P_ITEM_SIZE,
	"Lemans SMP2P item size is inconsistent");
BUILD_ASSERT(LEMANS_IPCC_SEND == (LEMANS_IPCC_BASE + (6U * 0x1000U) + 0x0CU),
	"Lemans CDSP0 IPCC client register is inconsistent");
BUILD_ASSERT(LEMANS_IPCC_SMP2P_PAYLOAD == ((8U << 16) | 2U),
	"Lemans SMP2P IPCC payload is inconsistent");

#define TOC_ENTRY_OFFSET                 0U
#define TOC_ENTRY_SIZE                   4U
#define TOC_ENTRY_HOST0                  12U
#define TOC_ENTRY_HOST1                  14U
#define TOC_ENTRY_CACHELINE              16U
#define TOC_ENTRY_EXCLUSION_BASE         32U
#define TOC_ENTRY_EXCLUSION_COUNT        4U
#define SMEM_TOC_VERSION                 1U

#define PARTITION_IDENTIFIER             0U
#define PARTITION_HOST0                  4U
#define PARTITION_HOST1                  6U
#define PARTITION_SIZE                   8U
#define PARTITION_FREE_UNCACHED          12U
#define PARTITION_FREE_CACHED            16U

#define ALLOC_CANARY                     0U
#define ALLOC_ITEM                       2U
#define ALLOC_SIZE                       4U
#define ALLOC_PADDING_DATA               8U
#define ALLOC_PADDING_HEADER             10U

struct lemans_smem_partition {
	uintptr_t base;
	uint32_t size;
	uint32_t cacheline;
};

struct lemans_smem_item {
	uintptr_t address;
	size_t size;
};

static inline uint16_t smem_read16(uintptr_t address)
{
	return sys_read16((mm_reg_t)address);
}

static inline uint32_t smem_read32(uintptr_t address)
{
	return sys_read32((mm_reg_t)address);
}

static inline void smem_write16(uintptr_t address, uint16_t value)
{
	sys_write16(value, (mm_reg_t)address);
}

static inline void smem_write32(uintptr_t address, uint32_t value)
{
	sys_write32(value, (mm_reg_t)address);
}

static inline void smem_write8(uintptr_t address, uint8_t value)
{
	sys_write8(value, (mm_reg_t)address);
}

static inline void lemans_smp2p_barrier(void)
{
	__asm__ volatile("barrier" ::: "memory");
}

static uint32_t smem_align_up(uint32_t value, uint32_t alignment)
{
	uint32_t remainder;

	if (alignment == 0U) {
		return 0U;
	}

	remainder = value % alignment;
	if (remainder != 0U) {
		value += alignment - remainder;
	}

	return value;
}

static bool smem_range_valid(uint32_t offset, uint32_t length, uint32_t limit)
{
	return offset <= limit && length <= (limit - offset);
}

static bool smem_host_pair_matches(uint16_t host0, uint16_t host1,
					   uint16_t local, uint16_t remote)
{
	return (host0 == local && host1 == remote) ||
		       (host0 == remote && host1 == local);
}

static int lemans_smem_target_info(uintptr_t *base, uint32_t *size,
					   uint16_t *max_items)
{
	uint64_t info_address;
	uint32_t smem_base;
	uint32_t smem_size;
	uint16_t items;

	info_address = (uint64_t)smem_read32(LEMANS_SMEM_TARG_INFO_LO);
	info_address |= (uint64_t)smem_read32(LEMANS_SMEM_TARG_INFO_HI) << 32;
	if ((info_address >> 32) != 0U || info_address == 0U) {
		return -ERANGE;
	}

	if (smem_read32((uintptr_t)info_address) != LEMANS_SMEM_TARG_INFO_MAGIC) {
		return -EINVAL;
	}
	smem_size = smem_read32((uintptr_t)info_address + 4U);
	smem_base = smem_read32((uintptr_t)info_address + 8U);
	items = smem_read16((uintptr_t)info_address + 16U);

	if (smem_size < LEMANS_SMEM_TOC_SIZE ||
	    smem_base > (UINT32_MAX - smem_size) || items <= LEMANS_SMP2P_ITEM) {
		return -EINVAL;
	}

	*base = (uintptr_t)smem_base;
	*size = smem_size;
	*max_items = items;
	return 0;
}

static int lemans_smem_partition(struct lemans_smem_partition *partition)
{
	uintptr_t smem_base;
	uintptr_t toc_base;
	uint32_t smem_size;
	uint16_t max_items;
	uint32_t entry_count;
	uint32_t index;

	if (lemans_smem_target_info(&smem_base, &smem_size, &max_items) != 0) {
		return -EINVAL;
	}
	ARG_UNUSED(max_items);

	toc_base = smem_base + smem_size - LEMANS_SMEM_TOC_SIZE;
	if (smem_read32(toc_base) != LEMANS_SMEM_TOC_MAGIC) {
		return -EINVAL;
	}

	if (smem_read32(toc_base + 4U) != SMEM_TOC_VERSION) {
		return -EINVAL;
	}
	entry_count = smem_read32(toc_base + 8U);
	if (entry_count > LEMANS_SMEM_TOC_MAX_ENTRIES ||
	    !smem_range_valid(LEMANS_SMEM_TOC_HEADER_SIZE,
			       entry_count * LEMANS_SMEM_TOC_ENTRY_SIZE,
			       LEMANS_SMEM_TOC_SIZE)) {
		return -EINVAL;
	}

	for (index = 0U; index < entry_count; index++) {
		uintptr_t entry = toc_base + LEMANS_SMEM_TOC_HEADER_SIZE +
			(index * LEMANS_SMEM_TOC_ENTRY_SIZE);
		uint32_t offset = smem_read32(entry + TOC_ENTRY_OFFSET);
		uint32_t size = smem_read32(entry + TOC_ENTRY_SIZE);
		uint16_t host0 = smem_read16(entry + TOC_ENTRY_HOST0);
		uint16_t host1 = smem_read16(entry + TOC_ENTRY_HOST1);

		if (!smem_host_pair_matches(host0, host1,
					    LEMANS_SMEM_LOCAL_HOST,
					    LEMANS_SMEM_REMOTE_HOST)) {
			continue;
		}
		if (size < LEMANS_SMEM_PARTITION_HEADER_SIZE ||
		    !smem_range_valid(offset, size, smem_size - LEMANS_SMEM_TOC_SIZE)) {
			return -EINVAL;
		}
		for (uint32_t exclusion = 0U;
		     exclusion < TOC_ENTRY_EXCLUSION_COUNT; exclusion++) {
			uint32_t excluded = smem_read32(entry + TOC_ENTRY_EXCLUSION_BASE +
						       (exclusion * sizeof(uint32_t)));

			if (excluded > size) {
				return -EINVAL;
			}
			size -= excluded;
		}
		if (size < LEMANS_SMEM_PARTITION_HEADER_SIZE) {
			return -EINVAL;
		}

		partition->base = smem_base + offset;
		partition->size = size;
		partition->cacheline = smem_read32(entry + TOC_ENTRY_CACHELINE);
		return partition->cacheline == 0U ? -EINVAL : 0;
	}

	return -ENOENT;
}

static int lemans_smem_scan_uncached(const struct lemans_smem_partition *partition,
					     uint32_t *free_uncached,
					     struct lemans_smem_item *item)
{
	uint32_t offset = LEMANS_SMEM_PARTITION_HEADER_SIZE;
	uint32_t limit = *free_uncached;

	while (offset < limit) {
		uint32_t size;
		uint32_t padding_data;
		uint32_t padding_header;
		uint32_t total;
		uint16_t canary;
		uint16_t smem_type;
		uintptr_t header;

		if (!smem_range_valid(offset, LEMANS_SMEM_ALLOC_HEADER_SIZE, limit)) {
			return -EINVAL;
		}

		header = partition->base + offset;
		canary = smem_read16(header + ALLOC_CANARY);
		smem_type = smem_read16(header + ALLOC_ITEM);
		size = smem_read32(header + ALLOC_SIZE);
		padding_data = smem_read16(header + ALLOC_PADDING_DATA);
		padding_header = smem_read16(header + ALLOC_PADDING_HEADER);
		if (canary != LEMANS_SMEM_ALLOC_CANARY || size == 0U ||
		    padding_data > size) {
			return -EINVAL;
		}
		if (size > UINT32_MAX - LEMANS_SMEM_ALLOC_HEADER_SIZE - padding_header) {
			return -EINVAL;
		}
		total = LEMANS_SMEM_ALLOC_HEADER_SIZE + padding_header + size;
		if (!smem_range_valid(offset, total, limit) ||
		    !smem_range_valid(offset + LEMANS_SMEM_ALLOC_HEADER_SIZE +
				      padding_header, size, partition->size)) {
			return -EINVAL;
		}

		if (smem_type == LEMANS_SMP2P_ITEM) {
			item->address = header + LEMANS_SMEM_ALLOC_HEADER_SIZE +
					padding_header;
			item->size = size - padding_data;
			return 0;
		}

		offset += total;
	}

	return offset == limit ? -ENOENT : -EINVAL;
}

static int lemans_smem_scan_cached(const struct lemans_smem_partition *partition,
					   uint32_t free_cached,
					   struct lemans_smem_item *item)
{
	uint32_t header_padding = smem_align_up(LEMANS_SMEM_ALLOC_HEADER_SIZE,
						partition->cacheline) -
						 LEMANS_SMEM_ALLOC_HEADER_SIZE;
	uint32_t offset;

	if (partition->size < LEMANS_SMEM_ALLOC_HEADER_SIZE + header_padding ||
	    free_cached > partition->size) {
		return -EINVAL;
	}

	offset = partition->size - LEMANS_SMEM_ALLOC_HEADER_SIZE - header_padding;
	while (offset >= free_cached) {
		uint32_t size;
		uint32_t padding_data;
		uint32_t total;
		uint16_t canary;
		uint16_t smem_type;
		uintptr_t header = partition->base + offset;

		if (!smem_range_valid(offset, LEMANS_SMEM_ALLOC_HEADER_SIZE,
				      partition->size)) {
			return -EINVAL;
		}

		canary = smem_read16(header + ALLOC_CANARY);
		smem_type = smem_read16(header + ALLOC_ITEM);
		size = smem_read32(header + ALLOC_SIZE);
		padding_data = smem_read16(header + ALLOC_PADDING_DATA);
		if (canary != LEMANS_SMEM_ALLOC_CANARY || size == 0U ||
		    padding_data > size || size > offset) {
			return -EINVAL;
		}
		if (size > UINT32_MAX - LEMANS_SMEM_ALLOC_HEADER_SIZE - header_padding) {
			return -EINVAL;
		}
		total = size + LEMANS_SMEM_ALLOC_HEADER_SIZE + header_padding;
		if (total > offset) {
			return -EINVAL;
		}
		if (!smem_range_valid(offset - size, size, partition->size)) {
			return -EINVAL;
		}

		if (smem_type == LEMANS_SMP2P_ITEM) {
			item->address = header - size;
			item->size = size - padding_data;
			return 0;
		}
		if (offset - total < free_cached) {
			break;
		}

		offset -= total;
	}

	return -ENOENT;
}

static int lemans_smem_get(struct lemans_smem_item *item)
{
	struct lemans_smem_partition partition;
	uint32_t free_uncached;
	uint32_t free_cached;
	int ret;

	ret = lemans_smem_partition(&partition);
	if (ret != 0) {
		return ret;
	}

	if (smem_read32(partition.base + PARTITION_IDENTIFIER) !=
		LEMANS_SMEM_PARTITION_MAGIC ||
	    !smem_host_pair_matches(smem_read16(partition.base + PARTITION_HOST0),
				     smem_read16(partition.base + PARTITION_HOST1),
				     LEMANS_SMEM_LOCAL_HOST,
				     LEMANS_SMEM_REMOTE_HOST) ||
	    smem_read32(partition.base + PARTITION_SIZE) != partition.size) {
		return -EINVAL;
	}

	free_uncached = smem_read32(partition.base + PARTITION_FREE_UNCACHED);
	free_cached = smem_read32(partition.base + PARTITION_FREE_CACHED);
	if (free_uncached < LEMANS_SMEM_PARTITION_HEADER_SIZE ||
	    free_uncached > free_cached || free_cached > partition.size) {
		return -EINVAL;
	}

	ret = lemans_smem_scan_uncached(&partition, &free_uncached, item);
	if (ret == 0) {
		return 0;
	}
	if (ret != -ENOENT) {
		return ret;
	}

	return lemans_smem_scan_cached(&partition, free_cached, item);
}

static void lemans_smp2p_kick(void)
{
	lemans_smp2p_barrier();
	smem_write32(LEMANS_IPCC_SEND, LEMANS_IPCC_SMP2P_PAYLOAD);
}

static int lemans_smp2p_publish(void)
{
	struct lemans_smem_item item;
	uintptr_t address;
	uint32_t version_features;
	uint32_t index;
	int ret;

	ret = lemans_smem_get(&item);
	if (ret != 0 || item.size < LEMANS_SMP2P_SIZE) {
		return ret != 0 ? ret : -EINVAL;
	}

	address = item.address;
	for (index = 0U; index < LEMANS_SMP2P_SIZE; index += 4U) {
		smem_write32(address + index, 0U);
	}

	smem_write32(address + 0U, LEMANS_SMP2P_MAGIC);
	smem_write16(address + 8U, LEMANS_SMEM_LOCAL_HOST);
	smem_write16(address + 10U, LEMANS_SMEM_REMOTE_HOST);
	smem_write16(address + 12U, LEMANS_SMP2P_ENTRY_COUNT);
	smem_write16(address + 14U, 0U);
	smem_write32(address + 16U, 0U);
	lemans_smp2p_barrier();

	version_features = LEMANS_SMP2P_VERSION | (LEMANS_SMP2P_FEATURES << 8);
	smem_write32(address + 4U, version_features);
	lemans_smp2p_kick();

	for (index = 0U; index < sizeof(LEMANS_SMP2P_ENTRY_NAME); index++) {
		smem_write8(address + LEMANS_SMP2P_HEADER_SIZE + index,
			    (uint8_t)LEMANS_SMP2P_ENTRY_NAME[index]);
	}
	lemans_smp2p_barrier();
	smem_write16(address + 14U, 1U);
	lemans_smp2p_kick();

	smem_write32(address + LEMANS_SMP2P_HEADER_SIZE + 16U,
		    LEMANS_SMP2P_READY);
	lemans_smp2p_kick();
	smem_write32(address + LEMANS_SMP2P_HEADER_SIZE + 16U,
		    LEMANS_SMP2P_HANDOVER);
	lemans_smp2p_kick();

	return 0;
}

static int lemans_smp2p_init(void)
{
	return lemans_smp2p_publish();
}

SYS_INIT(lemans_smp2p_init, PRE_KERNEL_1, 30);
