/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Qualcomm IPCC, as seen by one client through its own register frame.
 * A channel is (client << 16) | signal: the destination when sending, the
 * source when receiving.
 */

#define DT_DRV_COMPAT qcom_ipcc

#include <zephyr/device.h>
#include <zephyr/drivers/mbox.h>
#include <zephyr/irq.h>
#include <zephyr/spinlock.h>
#include <zephyr/sys/barrier.h>
#include <zephyr/sys/device_mmio.h>
#include <zephyr/sys/sys_io.h>

#define IPCC_CONFIG              0x08
#define IPCC_SEND_ID             0x0c
#define IPCC_RECV_ID             0x10
#define IPCC_RECV_SIGNAL_ENABLE  0x14
#define IPCC_RECV_SIGNAL_DISABLE 0x18
#define IPCC_RECV_SIGNAL_CLEAR   0x1c

#define IPCC_CLEAR_ON_RECV_RD BIT(0)
#define IPCC_NO_PENDING       0xffffffffU

struct ipcc_rx {
	mbox_channel_id_t channel;
	mbox_callback_t cb;
	void *user_data;
};

struct ipcc_config {
	DEVICE_MMIO_ROM;
	void (*irq_config)(void);
};

struct ipcc_data {
	DEVICE_MMIO_RAM;
	struct k_spinlock lock;
	struct ipcc_rx rx[CONFIG_MBOX_QCOM_IPCC_RX_CHANNELS];
};

static struct ipcc_rx *ipcc_rx_find(struct ipcc_data *data, mbox_channel_id_t channel)
{
	for (size_t i = 0; i < ARRAY_SIZE(data->rx); i++) {
		if (data->rx[i].cb != NULL && data->rx[i].channel == channel) {
			return &data->rx[i];
		}
	}

	return NULL;
}

static int ipcc_send(const struct device *dev, mbox_channel_id_t channel,
		     const struct mbox_msg *msg)
{
	if (msg != NULL && msg->size != 0) {
		return -EMSGSIZE;
	}

	/* Order shared memory writes before the doorbell */
	barrier_dsync_fence_full();
	sys_write32(channel, DEVICE_MMIO_GET(dev) + IPCC_SEND_ID);

	return 0;
}

static int ipcc_register_callback(const struct device *dev, mbox_channel_id_t channel,
				  mbox_callback_t cb, void *user_data)
{
	struct ipcc_data *data = dev->data;
	k_spinlock_key_t key = k_spin_lock(&data->lock);
	struct ipcc_rx *rx = ipcc_rx_find(data, channel);
	int ret = 0;

	for (size_t i = 0; rx == NULL && i < ARRAY_SIZE(data->rx); i++) {
		if (data->rx[i].cb == NULL) {
			rx = &data->rx[i];
		}
	}

	if (rx == NULL) {
		ret = -ENOMEM;
	} else {
		rx->channel = channel;
		rx->user_data = user_data;
		rx->cb = cb;
	}

	k_spin_unlock(&data->lock, key);

	return ret;
}

static int ipcc_mtu_get(const struct device *dev)
{
	ARG_UNUSED(dev);

	return 0;
}

static uint32_t ipcc_max_channels_get(const struct device *dev)
{
	ARG_UNUSED(dev);

	return UINT32_MAX;
}

static int ipcc_set_enabled(const struct device *dev, mbox_channel_id_t channel, bool enabled)
{
	sys_write32(channel, DEVICE_MMIO_GET(dev) +
		    (enabled ? IPCC_RECV_SIGNAL_ENABLE : IPCC_RECV_SIGNAL_DISABLE));

	return 0;
}

static void ipcc_isr(const struct device *dev)
{
	struct ipcc_data *data = dev->data;
	mm_reg_t base = DEVICE_MMIO_GET(dev);
	uint32_t id;

	while ((id = sys_read32(base + IPCC_RECV_ID)) != IPCC_NO_PENDING) {
		struct ipcc_rx *rx;

		sys_write32(id, base + IPCC_RECV_SIGNAL_CLEAR);

		rx = ipcc_rx_find(data, id);
		if (rx != NULL) {
			rx->cb(dev, id, rx->user_data, NULL);
		}
	}
}

static int ipcc_init(const struct device *dev)
{
	const struct ipcc_config *cfg = dev->config;
	mm_reg_t base;

	DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);
	base = DEVICE_MMIO_GET(dev);

	/* The ISR clears each signal itself */
	sys_write32(sys_read32(base + IPCC_CONFIG) & ~IPCC_CLEAR_ON_RECV_RD, base + IPCC_CONFIG);

	cfg->irq_config();

	return 0;
}

static DEVICE_API(mbox, ipcc_api) = {
	.send = ipcc_send,
	.register_callback = ipcc_register_callback,
	.mtu_get = ipcc_mtu_get,
	.max_channels_get = ipcc_max_channels_get,
	.set_enabled = ipcc_set_enabled,
};

#define IPCC_INIT(n)                                                                               \
	static void ipcc_irq_config_##n(void)                                                      \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), ipcc_isr,                   \
			    DEVICE_DT_INST_GET(n), 0);                                             \
		irq_enable(DT_INST_IRQN(n));                                                       \
	}                                                                                          \
                                                                                                   \
	static const struct ipcc_config ipcc_config_##n = {                                        \
		DEVICE_MMIO_ROM_INIT(DT_DRV_INST(n)),                                              \
		.irq_config = ipcc_irq_config_##n,                                                 \
	};                                                                                         \
                                                                                                   \
	static struct ipcc_data ipcc_data_##n;                                                     \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, ipcc_init, NULL, &ipcc_data_##n, &ipcc_config_##n, PRE_KERNEL_1,  \
			      CONFIG_MBOX_INIT_PRIORITY, &ipcc_api);

DT_INST_FOREACH_STATUS_OKAY(IPCC_INIT)
