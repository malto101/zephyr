/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Boot handshake with Linux remoteproc (qcom_q6v5_pas): allocate this
 * processor's outbound SMP2P item in the SMEM partition shared with the
 * host, publish a "slave-kernel" entry with the ready and handover bits
 * set, and ring the host over IPCC.
 */

#include <string.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/hwspinlock.h>
#include <zephyr/drivers/mbox.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/barrier.h>
#include <zephyr/sys/device_mmio.h>
#include <zephyr/sys/sys_io.h>

LOG_MODULE_REGISTER(qcom_smp2p, LOG_LEVEL_INF);

#define SMP2P_NODE DT_COMPAT_GET_ANY_STATUS_OKAY(qcom_smp2p)
#define SMEM_NODE  DT_COMPAT_GET_ANY_STATUS_OKAY(qcom_smem)

#define SMP2P_LOCAL_PID  DT_PROP(SMP2P_NODE, qcom_local_pid)
#define SMP2P_REMOTE_PID DT_PROP(SMP2P_NODE, qcom_remote_pid)
#define SMP2P_OUT_ITEM   DT_PROP_BY_IDX(SMP2P_NODE, qcom_smem, 0)

/* SMEM table of contents, last 4K of the region */
#define SMEM_TOC_SIZE      0x1000U
#define SMEM_TOC_MAGIC     0x434f5424U /* "$TOC" */
#define SMEM_TOC_COUNT     0x08U
#define SMEM_TOC_ENTRIES   0x20U
#define SMEM_TOC_ENTRY_SZ  0x30U
#define SMEM_TOC_E_OFFSET  0x00U
#define SMEM_TOC_E_SIZE    0x04U
#define SMEM_TOC_E_HOSTS   0x0cU

/* Partition header */
#define SMEM_PRT_MAGIC     0x54525024U /* "$PRT" */
#define SMEM_PRT_HOSTS     0x04U
#define SMEM_PRT_FREE_UNC  0x0cU
#define SMEM_PRT_FREE_C    0x10U
#define SMEM_PRT_HDR_SZ    0x20U

/* Uncached allocation header */
#define SMEM_ENT_CANARY    0xa5a5U
#define SMEM_ENT_TAG       0x00U /* canary | item << 16 */
#define SMEM_ENT_SIZE      0x04U
#define SMEM_ENT_PADDING   0x08U /* padding_data | padding_hdr << 16 */
#define SMEM_ENT_HDR_SZ    0x10U

/* SMP2P item */
#define SMP2P_MAGIC        0x504d5324U /* "$SMP" */
#define SMP2P_VERSION      1U
#define SMP2P_MAX_ENTRY    16U
#define SMP2P_NAME_LEN     16U
#define SMP2P_HDR_SZ       0x14U
#define SMP2P_ENTRY_SZ     (SMP2P_NAME_LEN + 4U)
#define SMP2P_ITEM_SZ      (SMP2P_HDR_SZ + SMP2P_MAX_ENTRY * SMP2P_ENTRY_SZ)

#define SMP2P_BIT_READY    BIT(1)
#define SMP2P_BIT_HANDOVER BIT(2)

static const struct mbox_dt_spec smp2p_mbox = MBOX_DT_SPEC_GET(SMP2P_NODE, tx);
static const struct hwspinlock_dt_spec smem_lock = HWSPINLOCK_DT_SPEC_GET(SMEM_NODE);

static bool smem_hosts_match(uint32_t hosts)
{
	uint16_t h0 = hosts & 0xffffU;
	uint16_t h1 = hosts >> 16;

	return (h0 == SMP2P_LOCAL_PID && h1 == SMP2P_REMOTE_PID) ||
	       (h0 == SMP2P_REMOTE_PID && h1 == SMP2P_LOCAL_PID);
}

static mm_reg_t smem_partition(mm_reg_t smem, size_t smem_size)
{
	mm_reg_t toc = smem + smem_size - SMEM_TOC_SIZE;
	uint32_t count;

	if (sys_read32(toc) != SMEM_TOC_MAGIC) {
		return 0;
	}

	count = MIN(sys_read32(toc + SMEM_TOC_COUNT),
		    (SMEM_TOC_SIZE - SMEM_TOC_ENTRIES) / SMEM_TOC_ENTRY_SZ);

	for (uint32_t i = 0; i < count; i++) {
		mm_reg_t e = toc + SMEM_TOC_ENTRIES + i * SMEM_TOC_ENTRY_SZ;
		uint32_t off = sys_read32(e + SMEM_TOC_E_OFFSET);
		uint32_t size = sys_read32(e + SMEM_TOC_E_SIZE);
		mm_reg_t prt = smem + off;

		if (!smem_hosts_match(sys_read32(e + SMEM_TOC_E_HOSTS)) || size == 0U ||
		    off > smem_size || size > smem_size - off) {
			continue;
		}
		if (sys_read32(prt) == SMEM_PRT_MAGIC &&
		    smem_hosts_match(sys_read32(prt + SMEM_PRT_HOSTS))) {
			return prt;
		}
	}

	return 0;
}

/* Find or allocate an uncached item, as Linux qcom_smem_alloc_private() */
static mm_reg_t smem_alloc(mm_reg_t prt, uint16_t item, uint32_t size)
{
	uint32_t free_unc = sys_read32(prt + SMEM_PRT_FREE_UNC);
	uint32_t free_c = sys_read32(prt + SMEM_PRT_FREE_C);
	uint32_t asize = ROUND_UP(size, 8U);
	uint32_t off = SMEM_PRT_HDR_SZ;

	while (off < free_unc) {
		mm_reg_t e = prt + off;
		uint32_t tag = sys_read32(e + SMEM_ENT_TAG);
		uint32_t esize = sys_read32(e + SMEM_ENT_SIZE);
		uint32_t pad_hdr = sys_read32(e + SMEM_ENT_PADDING) >> 16;

		if ((tag & 0xffffU) != SMEM_ENT_CANARY) {
			LOG_ERR("SMEM: bad canary at %#x", off);
			return 0;
		}
		if ((tag >> 16) == item) {
			return e + SMEM_ENT_HDR_SZ + pad_hdr;
		}
		off += SMEM_ENT_HDR_SZ + pad_hdr + esize;
	}

	if (off != free_unc || off + SMEM_ENT_HDR_SZ + asize > free_c) {
		LOG_ERR("SMEM: no room for item %u", item);
		return 0;
	}

	sys_write32(SMEM_ENT_CANARY | ((uint32_t)item << 16), prt + off + SMEM_ENT_TAG);
	sys_write32(asize, prt + off + SMEM_ENT_SIZE);
	sys_write32(asize - size, prt + off + SMEM_ENT_PADDING);
	sys_write32(0, prt + off + SMEM_ENT_PADDING + 4U);
	barrier_dmem_fence_full();
	sys_write32(off + SMEM_ENT_HDR_SZ + asize, prt + SMEM_PRT_FREE_UNC);

	return prt + off + SMEM_ENT_HDR_SZ;
}

static void smp2p_write_name(mm_reg_t dst, const char *name)
{
	uint32_t w[SMP2P_NAME_LEN / 4U] = {0};

	memcpy(w, name, MIN(strlen(name), SMP2P_NAME_LEN - 1U));
	for (size_t i = 0; i < ARRAY_SIZE(w); i++) {
		sys_write32(w[i], dst + i * 4U);
	}
}

static int smp2p_ready_init(void)
{
	mm_reg_t smem, prt, out, value;
	k_spinlock_key_t key;

	if (!device_is_ready(smp2p_mbox.dev) || !device_is_ready(smem_lock.dev)) {
		return -ENODEV;
	}

	device_map(&smem, DT_REG_ADDR(SMEM_NODE), DT_REG_SIZE(SMEM_NODE),
		   K_MEM_CACHE_NONE | K_MEM_DIRECT_MAP);

	prt = smem_partition(smem, DT_REG_SIZE(SMEM_NODE));
	if (prt == 0) {
		LOG_ERR("SMEM: no partition for hosts %u/%u", SMP2P_LOCAL_PID, SMP2P_REMOTE_PID);
		return -ENOENT;
	}

	key = hw_spin_lock(smem_lock.dev, smem_lock.id);
	out = smem_alloc(prt, SMP2P_OUT_ITEM, SMP2P_ITEM_SZ);
	hw_spin_unlock(smem_lock.dev, smem_lock.id, key);
	if (out == 0) {
		return -ENOMEM;
	}

	/* An item left by an earlier boot is reset, value 0 first */
	value = out + SMP2P_HDR_SZ + SMP2P_NAME_LEN;
	sys_write32(0, value);
	smp2p_write_name(out + SMP2P_HDR_SZ, "slave-kernel");
	sys_write32(SMP2P_LOCAL_PID | (SMP2P_REMOTE_PID << 16), out + 0x08);
	sys_write32(SMP2P_MAX_ENTRY | (1U << 16), out + 0x0c);
	sys_write32(0, out + 0x10);
	sys_write32(SMP2P_VERSION, out + 0x04); /* no features */
	sys_write32(SMP2P_MAGIC, out);
	mbox_send_dt(&smp2p_mbox, NULL);

	k_busy_wait(1000);
	sys_write32(SMP2P_BIT_READY | SMP2P_BIT_HANDOVER, value);
	mbox_send_dt(&smp2p_mbox, NULL);

	LOG_INF("SMP2P item %u at %#lx: ready", SMP2P_OUT_ITEM, (unsigned long)out);

	return 0;
}

SYS_INIT(smp2p_ready_init, POST_KERNEL, 99);
