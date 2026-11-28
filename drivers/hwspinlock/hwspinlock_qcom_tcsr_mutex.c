/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Qualcomm TCSR mutex: one register per lock. Writing the owner's
 * processor ID takes a free lock, reading it back tells who holds it,
 * writing 0 releases it.
 */

#define DT_DRV_COMPAT qcom_tcsr_mutex

#include <zephyr/device.h>
#include <zephyr/drivers/hwspinlock.h>
#include <zephyr/sys/device_mmio.h>
#include <zephyr/sys/sys_io.h>

struct tcsr_mutex_config {
	struct hwspinlock_driver_config common;
	DEVICE_MMIO_NAMED_ROM(base);
	uint32_t stride;
	uint32_t proc_id;
};

struct tcsr_mutex_data {
	DEVICE_MMIO_NAMED_RAM(base);
};

#define DEV_CFG(dev)  ((const struct tcsr_mutex_config *)(dev)->config)
#define DEV_DATA(dev) ((struct tcsr_mutex_data *)(dev)->data)

static mm_reg_t tcsr_mutex_reg(const struct device *dev, uint32_t id)
{
	const struct tcsr_mutex_config *cfg = dev->config;

	return DEVICE_MMIO_NAMED_GET(dev, base) + id * cfg->stride;
}

static int tcsr_mutex_trylock(const struct device *dev, uint32_t id)
{
	const struct tcsr_mutex_config *cfg = dev->config;
	mm_reg_t reg = tcsr_mutex_reg(dev, id);

	sys_write32(cfg->proc_id, reg);

	return sys_read32(reg) == cfg->proc_id ? 0 : -EBUSY;
}

static void tcsr_mutex_lock(const struct device *dev, uint32_t id)
{
	while (tcsr_mutex_trylock(dev, id) != 0) {
		arch_spin_relax();
	}
}

static void tcsr_mutex_unlock(const struct device *dev, uint32_t id)
{
	sys_write32(0, tcsr_mutex_reg(dev, id));
}

static uint32_t tcsr_mutex_get_max_id(const struct device *dev)
{
	const struct tcsr_mutex_config *cfg = dev->config;

	return cfg->common.num_locks - 1U;
}

static int tcsr_mutex_init(const struct device *dev)
{
	DEVICE_MMIO_NAMED_MAP(dev, base, K_MEM_CACHE_NONE);

	return 0;
}

static DEVICE_API(hwspinlock, tcsr_mutex_api) = {
	.trylock = tcsr_mutex_trylock,
	.lock = tcsr_mutex_lock,
	.unlock = tcsr_mutex_unlock,
	.get_max_id = tcsr_mutex_get_max_id,
};

#define TCSR_MUTEX_INIT(n)                                                                         \
	BUILD_ASSERT(DT_INST_PROP(n, num_locks) * DT_INST_PROP(n, stride) <=                       \
		     DT_INST_REG_SIZE(n));                                                         \
	HWSPINLOCK_SPINLOCK_ARRAY_DT_INST_DEFINE(n);                                               \
	static struct tcsr_mutex_data tcsr_mutex_data_##n;                                         \
	static const struct tcsr_mutex_config tcsr_mutex_config_##n = {                            \
		.common = HWSPINLOCK_COMMON_CONFIG_FROM_DT_INST(n),                                \
		DEVICE_MMIO_NAMED_ROM_INIT(base, DT_DRV_INST(n)),                                  \
		.stride = DT_INST_PROP(n, stride),                                                 \
		.proc_id = DT_INST_PROP(n, proc_id),                                               \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(n, tcsr_mutex_init, NULL, &tcsr_mutex_data_##n,                      \
			      &tcsr_mutex_config_##n, PRE_KERNEL_1,                                \
			      CONFIG_HWSPINLOCK_INIT_PRIORITY, &tcsr_mutex_api);

DT_INST_FOREACH_STATUS_OKAY(TCSR_MUTEX_INIT)
