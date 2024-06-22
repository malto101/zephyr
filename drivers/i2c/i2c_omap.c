#define DT_DRV_COMPAT ti_omap_i2c

#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
#include <i2c-priv.h>
#include <i2c.h>

LOG_MODULE_REGISTER(i2c_omap);

typedef void (*init_func_t)(const struct device *dev);

#define I2C_REVNB_LO        0x0
#define I2C_REVNB_HI        0x4
#define I2C_SYSC            0x10
#define I2C_EOI             0x20
#define I2C_IRQSTATUS_RAW   0x24
#define I2C_IRQSTATUS       0x28
#define I2C_IRQENABLE_SET   0x2c
#define I2C_IRQENABLE_CLR   0x30
#define I2C_WE              0x34
#define I2C_DMARXENABLE_SET 0x38
#define I2C_DMATXENABLE_SET 0x3c
#define I2C_DMARXENABLE_CLR 0x40
#define I2C_DMATXENABLE_CLR 0x44
#define I2C_DMARXWAKE_EN    0x48
#define I2C_DMATXWAKE_EN    0x4c
#define I2C_IE              0x84
#define I2C_STAT            0x88
#define I2C_SYSS            0x90
#define I2C_BUF             0x94
#define I2C_CNT             0x98
#define I2C_DATA            0x9c
#define I2C_CON             0xa4
#define I2C_OA              0xa8
#define I2C_SA              0xac
#define I2C_PSC             0xb0
#define I2C_SCLL            0xb4
#define I2C_SCH             0xb8
#define I2C_SYSTEST         0xbc
#define I2C_BUFSTAT         0xc0
#define I2C_OA1             0xc4
#define I2C_OA2             0xc8
#define I2C_OA3             0xcc
#define I2C_ACTOA           0xd0
#define I2C_SBLOCK          0xd4

LOG_MODULE_REGISTER(omap_i2c, CONFIG_I2C_LOG_LEVEL);

struct omap_i2c_config {
	uint32_t base;
	uint32_t irq;
};

struct omap_i2c_data {
	struct k_sem lock;
	// Additional runtime data
};

static int omap_i2c_configure(const struct device *dev, uint32_t dev_config)
{
	// Configuration implementation
	return 0;
}

static int omap_i2c_transfer(const struct device *dev, struct i2c_msg *msgs, uint8_t num_msgs,
			     uint16_t addr)
{
	// Transfer implementation
	return 0;
}

static int omap_i2c_init(const struct device *dev)
{
	const struct omap_i2c_config *config = dev->config;
	struct omap_i2c_data *data = dev->data;

	k_sem_init(&data->lock, 1, 1);

	// Initialize the hardware

	return 0;
}

static const struct i2c_driver_api omap_i2c_driver_api = {
	.configure = omap_i2c_configure,
	.transfer = omap_i2c_transfer,
};

#define OMAP_I2C_INIT(inst)                                                                        \
	static struct omap_i2c_data omap_i2c_data_##inst;                                          \
	static const struct omap_i2c_config omap_i2c_config_##inst = {                             \
		.base = DT_INST_REG_ADDR(inst),                                                    \
		.irq = DT_INST_IRQN(inst),                                                         \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(inst, &omap_i2c_init, NULL, &omap_i2c_data_##inst,                   \
			      &omap_i2c_config_##inst, POST_KERNEL, CONFIG_I2C_INIT_PRIORITY,      \
			      &omap_i2c_driver_api);

DT_INST_FOREACH_STATUS_OKAY(OMAP_I2C_INIT)