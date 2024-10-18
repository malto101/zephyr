/* Copyright (C) 2024 BeagleBoard.org Foundation
* Copyright (C) 2024 Dhruv Menon
*
* SPDX-License-Identifier: Apache-2.0
*/

#define DT_DRV_COMPAT ti_omap_i2c
#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
#include <zephyr/drivers/i2c.h>
#include "i2c_bitbang.h"

LOG_MODULE_REGISTER(omap_i2c, CONFIG_OMAP_I2C_LOG_LEVEL);


#define OMAP_I2C_TIMEOUT 100U
/* OCP_SYSSTATUS bit definitions */
#define SYSS_RESETDONE_MASK BIT(0)

#define I2C_BITRATE_FAST 400000
#define I2C_BITRATE_STANDARD 100000

/* I2C Registers */
#define I2C_REVNB_LO		0x0		/* Revision number low */
#define I2C_REVNB_HI		0x4		/* Revision number high */
#define I2C_SYSC			0x10	/* System configuration */
#define I2C_EOI				0x20	/* End of interrupt */
#define I2C_IRQSTATUS_RAW	0x24	/* Per-event raw interrupt status vector */
#define I2C_IRQSTATUS		0x28	/* Per-event enabled interrupt status vector */
#define I2C_IRQENABLE_SET	0x2c	/* Per-event interrupt enable bit set */
#define I2C_IRQENABLE_CLR	0x30	/* Per-event interrupt clear bit clear */
#define I2C_WE				0x34	/* Wakeup enable */
#define I2C_DMARXENABLE_SET	0x38	/* Per-event DMA RX enable set */
#define I2C_DMATXENABLE_SET	0x3c	/* Per-event DMA TX enable set */
#define I2C_DMARXENABLE_CLR	0x40	/* Per-event DMA RX enable clear */
#define I2C_DMATXENABLE_CLR	0x44	/* Per-event DMA TX enable clear */
#define I2C_DMARXWAKE_EN	0x48	/* DMA RX wake enable */
#define I2C_DMATXWAKE_EN	0x4c	/* DMA TX wake enable */
#define I2C_IE				0x84	/* Interrupt enable(legacy) */
#define I2C_STAT			0x88	/* Status */
#define I2C_SYSS			0x90	/* System status */
#define I2C_BUF				0x94	/* Buffer */
#define I2C_CNT				0x98	/* Data count */
#define I2C_DATA			0x9c	/* Data access */
#define I2C_CON				0xa4	/* Configuration */
#define I2C_OA				0xa8	/* Own address */
#define I2C_SA				0xac	/* Slave Address */
#define I2C_PSC				0xb0	/* Clock prescaler */
#define I2C_SCLL			0xb4	/* SCL low time */
#define I2C_SCLH			0xb8	/* SCL high time */
#define I2C_SYSTEST			0xbc	/* System test */
#define I2C_BUFSTAT			0xc0	/* Buffer status */
#define I2C_OA1				0xc4	/* Own address 1 */
#define I2C_OA2				0xc8	/* Own address 2 */
#define I2C_OA3				0xcc	/* Own address 3 */
#define I2C_ACTOA			0xd0	/* Active own address */
#define I2C_SBLOCK			0xd4	/* Clock Blocking Enable */

/* I2C Interrupt Enable Register (OMAP_I2C_IE) */
#define I2C_IE_XDR		BIT(14)		/* TX Buffer drain interrupt enable */
#define I2C_IE_RDR		BIT(13)		/* RX Buffer drain interrupt enable */
#define I2C_IE_XRDY		BIT(4)		/* TX data ready interrupt enable */
#define I2C_IE_RRDY		BIT(3)		/* RX data ready interrupt enable */
#define I2C_IE_ARDY		BIT(2)		/* Access ready interrupt enable */
#define I2C_IE_NACK		BIT(1)		/* No acknowledgment interrupt enable */
#define OMAP_I2C_IE_AL	BIT(0)		/* Arbitration lost interrupt enable */

/* I2C Configuration Register (OMAP_I2C_CON) */
#define I2C_CON_EN			BIT(15)  /* I2C module enable */
#define I2C_CON_BE			BIT(14)  /* Big endian mode */
#define I2C_CON_OPMODE_HS	BIT(12)  /* High Speed support */
#define I2C_CON_STB			BIT(11)  /* Start byte mode (master) */
#define I2C_CON_MST			BIT(10)  /* Master/slave mode */
#define I2C_CON_TRX			BIT(9)   /* TX/RX mode (master only) */
#define I2C_CON_XA			BIT(8)   /* Expand address */
#define I2C_CON_STP			BIT(1)   /* Stop condition (master only) */
#define I2C_CON_STT			BIT(0)   /* Start condition (master) */

/* I2C WE wakeup enable register */
#define I2C_WE_XDR_WE	BIT(14)		/* TX drain wakeup */
#define I2C_WE_RDR_WE	BIT(13)		/* RX drain wakeup */
#define I2C_WE_AAS_WE	BIT(9)		/* Address as slave wakeup */
#define I2C_WE_BF_WE	BIT(8)		/* Bus free wakeup */
#define I2C_WE_STC_WE	BIT(6)		/* Start condition wakeup */
#define I2C_WE_GC_WE	BIT(5)		/* General call wakeup */
#define I2C_WE_DRDY_WE	BIT(3)		/* TX/RX data ready wakeup */
#define I2C_WE_ARDY_WE	BIT(2)		/* Register access ready wakeup */
#define I2C_WE_NACK_WE	BIT(1)		/* No acknowledgment wakeup */
#define I2C_WE_AL_WE	BIT(0)		/* Arbitration lost wakeup */

#define I2C_WE_ALL		(I2C_WE_XDR_WE | I2C_WE_RDR_WE | \
				I2C_WE_AAS_WE | I2C_WE_BF_WE | \
				I2C_WE_STC_WE | I2C_WE_GC_WE | \
				I2C_WE_DRDY_WE | I2C_WE_ARDY_WE | \
				I2C_WE_NACK_WE | I2C_WE_AL_WE)

/* I2C Buffer Configuration Register (OMAP_I2C_BUF): */
#define I2C_BUF_RDMA_EN 	BIT(15) /* RX DMA channel enable */
#define I2C_BUF_RXFIF_CLR	BIT(14) /* RX FIFO Clear */
#define I2C_BUF_XDMA_EN		BIT(7)  /* TX DMA channel enable */
#define I2C_BUF_TXFIF_CLR	BIT(6)  /* TX FIFO Clear */

/* I2C Status Register (OMAP_I2C_STAT): */
#define I2C_STAT_XDR  BIT(14) /* TX Buffer draining */
#define I2C_STAT_RDR  BIT(13) /* RX Buffer draining */
#define I2C_STAT_BB   BIT(12) /* Bus busy */
#define I2C_STAT_ROVR BIT(11) /* Receive overrun */
#define I2C_STAT_XUDF BIT(10) /* Transmit underflow */
#define I2C_STAT_AAS  BIT(9)  /* Address as slave */
#define I2C_STAT_BF   BIT(8)  /* Bus Free */
#define I2C_STAT_XRDY BIT(4)  /* Transmit data ready */
#define I2C_STAT_RRDY BIT(3)  /* Receive data ready */
#define I2C_STAT_ARDY BIT(2)  /* Register access ready */
#define I2C_STAT_NACK BIT(1)  /* No ack interrupt enable */
#define I2C_STAT_AL   BIT(0)  /* Arbitration lost */

/* I2C System Test Register (OMAP_I2C_SYSTEST): */
#define I2C_SYSTEST_ST_EN       BIT(15) /* System test enable */
#define I2C_SYSTEST_FREE        BIT(14) /* Free running mode */
#define I2C_SYSTEST_TMODE_MASK  (3 << 12) /* Test mode select mask */
#define I2C_SYSTEST_TMODE_SHIFT (12)      /* Test mode select shift */

/* Functional mode */
#define I2C_SYSTEST_SCL_I_FUNC  BIT(8)  /* SCL line input value */
#define I2C_SYSTEST_SCL_O_FUNC  BIT(7)  /* SCL line output value */
#define I2C_SYSTEST_SDA_I_FUNC  BIT(6)  /* SDA line input value */
#define I2C_SYSTEST_SDA_O_FUNC  BIT(5)  /* SDA line output value */

/* SDA/SCL IO mode */
#define I2C_SYSTEST_SCL_I       BIT(3)  /* SCL line sense in */
#define I2C_SYSTEST_SCL_O       BIT(2)  /* SCL line drive out */
#define I2C_SYSTEST_SDA_I       BIT(1)  /* SDA line sense in */
#define I2C_SYSTEST_SDA_O       BIT(0)  /* SDA line drive out */

typedef void (*init_func_t)(const struct device *dev);

struct i2c_omap_cfg {
    uint32_t base;
    uint32_t irq;
	uint32_t speed;
	init_func_t init_func;

};

enum i2c_omap_speed {
    I2C_OMAP_SPEED_STANDARD,
    I2C_OMAP_SPEED_FAST,
    I2C_OMAP_SPEED_FAST_PLUS,
};

struct i2c_omap_speed_config {
    uint32_t pscstate;
    uint32_t scllstate;
    uint32_t sclhstate;
};

struct i2c_omap_data {
	enum i2c_omap_speed speed;
    struct i2c_omap_speed_config speed_config;
    struct i2c_msg current_msg;
    struct k_sem cmd_complete;
    int cmd_err;
    bool receiver;
    bool bb_valid;
};
/**
 * Writes a value to the register of the OMAP I2C controller.
 *
 * @param cfg The configuration structure for the OMAP I2C controller.
 * @param reg The register to write the value to.
 * @param value The value to write to the register.
 */
static inline void omap_i2c_write_reg(const struct i2c_omap_cfg *cfg,
									  uint32_t reg, uint32_t value)
{
	sys_write16(value, cfg->base + reg);
}

/**
 * @brief Reads a register value from the OMAP I2C controller.
 *
 * This function reads the value of a register from the OMAP I2C controller
 * specified by the given configuration and register address.
 *
 * @param cfg Pointer to the I2C OMAP configuration structure.
 * @param reg The register address to read from.
 * @return The value read from the register.
 */
static inline uint16_t omap_i2c_read_reg(const struct i2c_omap_cfg *cfg, int reg)
{
	int read_reg_val = sys_read16(cfg->base + reg);
	return read_reg_val;
}

/**
 * @brief Acknowledge the I2C status by writing to the I2C_STAT register.
 *
 * This function is used to acknowledge the I2C status by writing the provided
 * status value to the I2C_STAT register. It is an inline function that is
 * specific to the OMAP I2C driver.
 *
 * @param dev Pointer to the device structure.
 * @param stat The status value to be written to the I2C_STAT register.
 */
static inline void omap_i2c_ack_stat(const struct device *dev, uint16_t stat)
{
	const struct i2c_omap_cfg *cfg = dev->config;
	omap_i2c_write_reg(cfg, I2C_STAT, stat);
}

/**
 * @brief Initializes the OMAP I2C driver.
 *
 * This function is responsible for initializing the OMAP I2C driver.
 *
 * @param dev Pointer to the device structure for the I2C driver instance.
 */
static void __omap_i2c_init(const struct device *dev)
{
	const struct i2c_omap_cfg *cfg = dev->config;
	struct i2c_omap_data *data = dev->data;

	omap_i2c_write_reg(cfg, I2C_CON, 0);

	omap_i2c_write_reg(cfg, I2C_PSC, data->speed_config.pscstate);

	omap_i2c_write_reg(cfg, I2C_SCLL, data->speed_config.scllstate);
	omap_i2c_write_reg(cfg, I2C_SCLH, data->speed_config.sclhstate);

	omap_i2c_write_reg(cfg, I2C_WE, I2C_WE_ALL);

	omap_i2c_write_reg(cfg, I2C_CON, I2C_CON_EN);

	k_busy_wait(1);
	uint16_t iestate = (I2C_IE_XRDY | I2C_IE_RRDY |
			I2C_IE_ARDY | I2C_IE_NACK |
			OMAP_I2C_IE_AL)  | ((data->current_msg.len) ?
				(I2C_IE_RDR | I2C_IE_XDR) : 0);
	omap_i2c_write_reg(cfg, I2C_IRQENABLE_SET, iestate);
}

/**
 * @brief Reset the OMAP I2C controller.
 *
 * This function resets the OMAP I2C controller specified by the device pointer.
 *
 * @param dev Pointer to the device structure for the I2C controller.
 * @return 0 on success, negative errno code on failure.
 */
static int omap_i2c_reset(const struct device *dev)
{
	const struct i2c_omap_cfg *cfg = dev->config;
	struct i2c_omap_data *data = dev->data;
	uint64_t timeout;
	uint16_t sysc;

	sysc = omap_i2c_read_reg(cfg, I2C_SYSC);

	// Disable the I2C controller by clearing the enable bit in the CON register
	omap_i2c_write_reg(cfg, I2C_CON, omap_i2c_read_reg(cfg, I2C_CON) & ~(I2C_CON_EN));

	// Set the timeout value to 1000 milliseconds from the current uptime
	timeout = k_uptime_get() + OMAP_I2C_TIMEOUT;

	// Re-enable the I2C controller
	omap_i2c_write_reg(cfg, I2C_CON, I2C_CON_EN);

	// Wait for the reset to complete by polling the SYSS register
	while (!(omap_i2c_read_reg(cfg, I2C_SYSS) & SYSS_RESETDONE_MASK)) {
		if (k_uptime_get() > timeout) {
			LOG_WRN("timeout waiting for controller reset");
			return -ETIMEDOUT;
		}
		k_msleep(1);
	}

	// The SYSC register is cleared by the reset; rewrite its original value
	omap_i2c_write_reg(cfg, I2C_SYSC, sysc);

	// Mark the bus monitoring as invalid, to be updated on the next transfer
	data->bb_valid = 0;
	return 0;
}

/**
 * @brief Set the speed of the OMAP I2C controller.
 *
 * This function sets the speed of the OMAP I2C controller based on the
 * specified speed parameter. The speed can be set to either Fast mode or
 * Standard mode.
 *
 * @param dev The pointer to the device structure.
 * @param speed The desired speed for the I2C controller.
 *
 * @return 0 on success, negative error code on failure.
 */
static int omap_i2c_set_speed(const struct device *dev, uint32_t speed)
{
	struct i2c_omap_data *data = dev->data;

	/* If configured for High Speed */
	switch (speed) {
	case I2C_BITRATE_FAST + 1 ... UINT32_MAX:
		return -ENOTSUP;
	case I2C_BITRATE_FAST:
		/* Fast mode */
		data->speed_config = (struct i2c_omap_speed_config){
			.pscstate = 9,
			.scllstate = 7,
			.sclhstate = 5,
		};
		break;
	case I2C_BITRATE_STANDARD:
		/* Standard mode */
		data->speed_config = (struct i2c_omap_speed_config){
			.pscstate = 23,
			.scllstate = 13,
			.sclhstate = 15,
		};
		break;
	default:
		return -ERANGE;
	}

	return 0;
}

/**
 * @brief Initialize the OMAP I2C controller.
 *
 * This function initializes the OMAP I2C controller by setting the speed and
 * performing any necessary initialization steps.
 *
 * @param dev Pointer to the device structure for the I2C controller.
 * @return 0 if successful, negative error code otherwise.
 */
static int omap_i2c_init(const struct device *dev)
{
	struct i2c_omap_data *data = dev->data;
	const struct i2c_omap_cfg *cfg = dev->config;

	k_sem_init(&data->cmd_complete, 0, 1);

	// Set the speed for I2C 
	if (omap_i2c_set_speed(dev, cfg->speed)) {
		LOG_ERR("Failed to set speed");
		return -ENOTSUP;
	}

	__omap_i2c_init(dev);

	return 0;
}

/**
 * @brief Configure the OMAP I2C controller with the specified device configuration.
 *
 * This function configures the OMAP I2C controller with the specified device configuration.
 *
 * @param dev The pointer to the device structure.
 * @param dev_config The device configuration to be applied.
 *
 * @return 0 on success, negative error code on failure.
 */
static int omap_i2c_configure(const struct device *dev, uint32_t dev_config)
{
	const struct i2c_omap_cfg *cfg = dev->config;
	uint32_t speed_cfg;

	switch (I2C_SPEED_GET(dev_config))
	{
		case I2C_SPEED_STANDARD:
			speed_cfg = I2C_BITRATE_STANDARD;
			goto out;
		case I2C_SPEED_FAST:
			speed_cfg = I2C_BITRATE_FAST;
			goto out;
		default:
			return -ENOTSUP;
	}

	if ((dev_config & I2C_MODE_CONTROLLER) != I2C_MODE_CONTROLLER)
	{
		return -ENOTSUP;
	}

	if ((dev_config & I2C_MSG_ADDR_10_BITS) != I2C_MSG_ADDR_10_BITS)
	{
		return -ENOTSUP;
	}

out:
	// Take the I2C module out of reset
	omap_i2c_write_reg(cfg, I2C_CON, 0);
	omap_i2c_set_speed(dev, speed_cfg);
	omap_i2c_reset(dev);
	__omap_i2c_init(dev);

	return 0;
}

/**
 * @brief Transmit or receive data over I2C bus
 *
 * This function transmits or receives data over the I2C bus using the OMAP I2C controller.
 *
 * @param dev Pointer to the I2C device structure
 * @param num_bytes Number of bytes to transmit or receive
 */
static void omap_i2c_transmit_receive_data(const struct device *dev, uint8_t num_bytes)
{
	const struct i2c_omap_cfg *cfg = dev->config;
	struct i2c_omap_data *data = dev->data;

	while (num_bytes--) {
		if (data->receiver) {
			*data->current_msg.buf++ = omap_i2c_read_reg(cfg, I2C_DATA);
		} else {
			omap_i2c_write_reg(cfg, I2C_DATA, *(data->current_msg.buf++));
		}
	}
}

/**
 * @brief Resize the FIFO buffer for the OMAP I2C controller.
 *
 * This function resizes the FIFO buffer for the OMAP I2C controller based on the specified size.
 * It clears the RX threshold and sets the new size for the receiver, or clears the TX threshold
 * and sets the new size for the transmitter.
 *
 * @param dev Pointer to the device structure.
 * @param size The new size of the FIFO buffer.
 */
static void omap_i2c_resize_fifo(const struct device *dev, uint8_t size)
{
	const struct i2c_omap_cfg *cfg = dev->config;
	struct i2c_omap_data *data = dev->data;
	uint16_t buf_cfg;

	buf_cfg = omap_i2c_read_reg(cfg, I2C_BUF);
	if (data->receiver) {
		// Clear RX threshold
		buf_cfg &= ~(0x3f << 8);
		buf_cfg |= ((size) << 8) | I2C_BUF_RXFIF_CLR;
	} else {
		// Clear TX threshold
		buf_cfg &= ~0x3f;
		buf_cfg |= (size) | I2C_BUF_TXFIF_CLR ;
	}
	omap_i2c_write_reg(cfg, I2C_BUF, buf_cfg);
}

/**
 * @brief Get the state of the SCL line.
 *
 * This function retrieves the state of the SCL (clock) line for the OMAP I2C controller.
 *
 * @param io_context The I2C context.
 * @return The state of the SCL line.
 */
static int __maybe_unused omap_i2c_get_scl(void *io_context)
{
	const struct i2c_omap_cfg *cfg = io_context;
	uint32_t reg;

	reg = omap_i2c_read_reg(cfg, I2C_SYSTEST);

	return reg & I2C_SYSTEST_SCL_I_FUNC;
}

/**
 * @brief Get the state of the SDA line.
 *
 * This function retrieves the state of the SDA (data) line for the OMAP I2C controller.
 *
 * @param io_context The I2C context.
 * @return The state of the SDA line.
 */
static int omap_i2c_get_sda(void *io_context)
{
	const struct i2c_omap_cfg *cfg = io_context;
	uint32_t reg;

	reg = omap_i2c_read_reg(cfg, I2C_SYSTEST);

	return reg & I2C_SYSTEST_SDA_I_FUNC;
}

/**
 * @brief Set the state of the SDA line.
 *
 * This function sets the state of the SDA (data) line for the OMAP I2C controller.
 *
 * @param io_context The I2C context.
 * @param state The state to set (0 for low, 1 for high).
 */
static void omap_i2c_set_sda(void *io_context, int state)
{
	const struct i2c_omap_cfg *cfg = io_context;
	uint32_t reg;

	reg = omap_i2c_read_reg(cfg, I2C_SYSTEST);

	if (state) {
		reg |= I2C_SYSTEST_SDA_O;
	} else {
		reg &= ~I2C_SYSTEST_SDA_O;
	}

	omap_i2c_write_reg(cfg, I2C_SYSTEST, reg);
}

/**
 * @brief Set the state of the SCL line.
 *
 * This function sets the state of the SCL (clock) line for the OMAP I2C controller.
 *
 * @param io_context The I2C context.
 * @param state The state to set (0 for low, 1 for high).
 */
static void omap_i2c_set_scl(void *io_context, int state)
{
	const struct i2c_omap_cfg *cfg = io_context;
	uint32_t reg;

	reg = omap_i2c_read_reg(cfg, I2C_SYSTEST);

	if (state) {
		reg |= I2C_SYSTEST_SCL_O;
	} else {
		reg &= ~I2C_SYSTEST_SCL_O;
	}

	omap_i2c_write_reg(cfg, I2C_SYSTEST, reg);
}
/**
 * @brief Recovers the I2C bus using the OMAP I2C controller.
 *
 * This function attempts to recover the I2C bus by performing a bus recovery
 * sequence using the OMAP I2C controller. It uses the provided device
 * configuration and bit-banging operations to recover the bus.
 *
 * @param dev Pointer to the device structure.
 * @return 0 on success, negative error code on failure.
 */

static int omap_i2c_recover_bus(const struct device *dev)
{
	const struct i2c_omap_cfg *cfg = dev->config;
	struct i2c_bitbang bitbang_omap;
	struct i2c_bitbang_io bitbang_omap_io = {
		.get_sda = omap_i2c_get_sda,
		.set_scl = omap_i2c_set_scl,
		.set_sda = omap_i2c_set_sda,
	};
	uint32_t reg;
	int error = 0;
	reg = omap_i2c_read_reg(cfg, I2C_SYSTEST);
	reg |= I2C_SYSTEST_ST_EN | 3 << I2C_SYSTEST_TMODE_SHIFT | I2C_SYSTEST_SCL_O | I2C_SYSTEST_SDA_O;
	omap_i2c_write_reg(cfg, I2C_SYSTEST, reg);

	i2c_bitbang_init(&bitbang_omap, &bitbang_omap_io, (void *)cfg);

	error = i2c_bitbang_recover_bus(&bitbang_omap);
	if (error != 0) {
		LOG_ERR("failed to recover bus (err %d)", error);
		goto restore;
	}

restore:
	reg = omap_i2c_read_reg(cfg, I2C_SYSTEST);
	reg &= ~I2C_SYSTEST_ST_EN & ~I2C_SYSTEST_TMODE_MASK & ~I2C_SYSTEST_SCL_O & ~I2C_SYSTEST_SDA_O;
	omap_i2c_write_reg(cfg, I2C_SYSTEST, reg);
	return error;
}


/**
 * @brief Wait for the bus to become free (no longer busy).
 *
 * This function waits for the bus to become free by continuously checking the
 * status register of the OMAP I2C controller. If the bus remains busy for a
 * certain timeout period, the function attempts to recover the bus by calling
 * omap_i2c_recover_bus().
 *
 * @param dev The I2C device structure.
 * @return 0 if the bus becomes free, or a negative error code if the bus cannot
 * be recovered.
 */
static int omap_i2c_wait_for_bb(const struct device *dev)
{
	const struct i2c_omap_cfg *cfg = dev->config;
	uint32_t timeout;
	timeout = k_uptime_get_32() + OMAP_I2C_TIMEOUT;
	while (omap_i2c_read_reg(cfg, I2C_STAT) & I2C_STAT_BB) {
		if (k_uptime_get_32() > timeout){
			return omap_i2c_recover_bus(dev);
		} 
		k_msleep(1);
	}
	return 0;
}

/**
 * @brief Performs data transfer for the OMAP I2C driver.
 *
 * This function is responsible for handling the data transfer logic for the OMAP I2C driver.
 * It reads the status register and performs the necessary actions based on the status flags.
 * It handles both receive and transmit logic, and also handles error conditions such as NACK,
 * arbitration lost, receive overrun, and transmit underflow.
 *
 * @param dev Pointer to the device structure.
 *
 * @return Returns 0 on success, or a negative error code on failure.
 */
static int omap_i2c_xfer_data(const struct device *dev)
{
	const struct i2c_omap_cfg *cfg = dev->config;
	struct i2c_omap_data *data = dev->data;
	uint16_t bits, stat;
	int err = 0;
	do {
		bits = omap_i2c_read_reg(cfg, I2C_IE);
		stat = omap_i2c_read_reg(cfg, I2C_STAT);
		stat |= bits;

		if (data->receiver) {
			stat &= ~(I2C_STAT_XDR | I2C_STAT_XRDY);
		} else {
			stat &= ~(I2C_STAT_RDR | I2C_STAT_RRDY);
		}

		if (stat & I2C_STAT_NACK) {
			err |= I2C_STAT_NACK;
			omap_i2c_ack_stat(dev, I2C_STAT_NACK);
		}

		if (stat & I2C_STAT_AL) {
			err |= I2C_STAT_AL;
			omap_i2c_ack_stat(dev, I2C_STAT_AL);
		}

		if (stat & I2C_STAT_ARDY) {
			omap_i2c_ack_stat(dev, I2C_STAT_ARDY);
		}

		if (stat & (I2C_STAT_ARDY | I2C_STAT_NACK | I2C_STAT_AL)) {
			omap_i2c_ack_stat(dev, I2C_STAT_RRDY | I2C_STAT_RDR |
						       I2C_STAT_XRDY | I2C_STAT_XDR |
						       I2C_STAT_ARDY);
			break;
		}

		// Handle receive logic
		if (stat & (I2C_STAT_RRDY | I2C_STAT_RDR)) {
			int buffer = omap_i2c_read_reg(
				cfg, (stat & I2C_STAT_RRDY) ? I2C_BUF : I2C_BUFSTAT);
			omap_i2c_transmit_receive_data(dev, buffer);
			omap_i2c_ack_stat(dev, stat & I2C_STAT_RRDY ? I2C_STAT_RRDY
									 : I2C_STAT_RDR);
			continue;
		}

		// Handle transmit logic
		if (stat & (I2C_STAT_XRDY | I2C_STAT_XDR)) {
			int buffer = omap_i2c_read_reg(
				cfg, (stat & I2C_STAT_XRDY) ? I2C_BUF : I2C_BUFSTAT);
			omap_i2c_transmit_receive_data(dev, buffer);
			omap_i2c_ack_stat(dev, (stat & I2C_STAT_XRDY) ? I2C_STAT_XRDY
									   : I2C_STAT_XDR);
			continue;
		}
		

		if (stat & I2C_STAT_ROVR) {
			LOG_INF("Receive overrun");
			err |= I2C_STAT_ROVR;
			omap_i2c_ack_stat(dev, I2C_STAT_ROVR);
			break;
		}

		if (stat & I2C_STAT_XUDF) {
			LOG_ERR("Transmit underflow");
			err |= I2C_STAT_XUDF;
			omap_i2c_ack_stat(dev, I2C_STAT_XUDF);
			break;
		

		}
		if (!stat) {
			data->cmd_err |= -EAGAIN;
			return -EAGAIN;
		}
		data->cmd_err = err;
	} while (stat);
	return err;
}

/**
 * @brief Performs an I2C transfer of a single message.
 *
 * This function is responsible for performing an I2C transfer of a single message.
 * It sets up the necessary configurations, writes the target device address,
 * sets the buffer and buffer length, and handles various error conditions.
 *
 * @param dev The I2C device structure.
 * @param msg Pointer to the I2C message structure.
 * @param polling Flag indicating whether to use polling mode or not.
 * @param addr The target device address.
 *
 * @return 0 on success, negative error code on failure.
 *         Possible error codes include:
 *         - ETIMEDOUT: Timeout occurred during the transfer.
 *         - EIO: I/O error due to receiver overrun or transmit underflow.
 *         - EAGAIN: Arbitration lost error, try again.
 *         - ENOMSG: Message error due to NACK.
 */
static int omap_i2c_xfer_msg(const struct device *dev, struct i2c_msg *msg, bool polling,
							 uint16_t addr)
{
	const struct i2c_omap_cfg *cfg = dev->config;
	struct i2c_omap_data *data = dev->data;

	unsigned long time_left;
	uint16_t control_reg;
	int ret;

	// Set receiver flag based on message direction
	data->receiver = (msg->flags & I2C_MSG_READ) ? true : false;

	// Resize FIFO to accommodate message length
	omap_i2c_resize_fifo(dev, msg->len);

	// Write target device address to I2C slave address register
	omap_i2c_write_reg(cfg, I2C_SA, addr);
	// Set buffer and buffer length for the message
	data->current_msg = *msg;

	// Ensure memory ordering with compiler barrier
	compiler_barrier();

	// Write buffer length to I2C count register
	omap_i2c_write_reg(cfg, I2C_CNT, msg->len);
	// Clear FIFO buffers in the I2C controller
	control_reg = omap_i2c_read_reg(cfg, I2C_BUF);
	control_reg |= I2C_BUF_RXFIF_CLR | I2C_BUF_TXFIF_CLR;
	omap_i2c_write_reg(cfg, I2C_BUF, control_reg);
	// If not using polling, reset command completion semaphore
	if (!polling) {
		k_sem_reset(&data->cmd_complete);
	}

	// Reset command error flag
	data->cmd_err = 0;
	// Configure control register bits for I2C operation
	control_reg = I2C_CON_EN | I2C_CON_MST | I2C_CON_STT;

	// Set high-speed mode if configured speed is greater than 400 kHz
	if (data->speed > I2C_BITRATE_FAST) {
		control_reg |= I2C_CON_OPMODE_HS;
	}
	// Set STOP condition flag if specified in message flags
	if (msg->flags & I2C_MSG_STOP) {
		control_reg |= I2C_CON_STP;
	}
	// Set TRX (transmit/receive) flag based on message direction
	if (!(msg->flags & I2C_MSG_READ)) {
		control_reg |= I2C_CON_TRX;
	}

	// Write control register settings to I2C control register
	omap_i2c_write_reg(cfg, I2C_CON, control_reg);
	// If not using polling, wait for completion of command
	if (!polling) {
	int result = k_sem_take(&data->cmd_complete, K_MSEC(OMAP_I2C_TIMEOUT));
	if (result == -EAGAIN) {
		// Timeout occurred, handle it accordingly
		time_left = -ETIMEDOUT; // or any other timeout handling
	}
	} else {
	// Polling mode: loop until data transfer is complete
		do {
			for (uint8_t count = 0; count < 5; count++) {
				if (omap_i2c_read_reg(cfg, I2C_STAT)) {
					break;
				}
				ret = -EAGAIN;
			}
			ret = omap_i2c_xfer_data(dev);
		} while (ret == -EAGAIN);
		time_left = !ret; 
		
	}
	// Handle timeout and specific error conditions
	if (time_left == 0 || (data->cmd_err & (I2C_STAT_ROVR | I2C_STAT_XUDF))) {
		omap_i2c_reset(dev);
		__omap_i2c_init(dev);

		// Return appropriate error code based on condition
		if (time_left == 0) {
			return -ETIMEDOUT;
		} else {
			return -EIO; // I/O error due to receiver overrun or transmit underflow
		}
	}
	// Handle arbitration lost, NACK, and command errors
	if (data->cmd_err & (I2C_STAT_AL | I2C_STAT_NACK)) {

		// Arbitration lost error, try again
		if (data->cmd_err & I2C_STAT_AL) {
			return -EAGAIN;
		}

		// NACK error handling
		if (data->cmd_err & I2C_STAT_NACK) {
			if (msg->flags) {
				return 0; // Non-essential message, return success
			}

			// Send STOP condition on NACK error
			omap_i2c_write_reg(cfg, I2C_CON, omap_i2c_read_reg(cfg, I2C_CON) | I2C_CON_STP);
			return -ENOMSG; // Message error due to NACK
		}
	}

	// If no command error, return success
	if (!data->cmd_err) {
		return 0;
	}
	return -EIO; // General I/O error if none of the specific error conditions match
}

/**
 * @brief Performs a common transfer operation for OMAP I2C devices.
 *
 * This function is responsible for transferring multiple I2C messages in a common way
 * for OMAP I2C devices. It waits for the bus to be idle, then iterates through each
 * message in the provided array and transfers them one by one using the omap_i2c_xfer_msg()
 * function. After all messages have been transferred, it waits for the bus to be idle again
 * before returning.
 *
 * @param dev The pointer to the I2C device structure.
 * @param msg An array of I2C messages to be transferred.
 * @param num The number of messages in the array.
 * @param polling Specifies whether to use polling or interrupt-based transfer.
 * @param addr The I2C slave address.
 * @return 0 on success, or a negative error code on failure.
 */
static int omap_i2c_xfer_common(const struct device *dev, struct i2c_msg msg[], int num, bool polling, uint16_t addr)
{
	int ret, msg_idx = 0;
	ret = omap_i2c_wait_for_bb(dev);
	if (ret < 0) {
		return ret;
	}
	while (msg_idx < num) {
		ret = omap_i2c_xfer_msg(dev, &msg[msg_idx], polling, addr);
		if (ret < 0) {
			break;
		}
		msg_idx++;
	}
	omap_i2c_wait_for_bb(dev);
	return ret;
}

/**
 * @brief OMAP I2C ISR function.
 *
 * This function is the interrupt service routine for OMAP I2C.
 * It checks the status and interrupt enable registers to determine if the
 * transfer is complete, and signals the completion using a semaphore.
 *
 * @param dev Pointer to the I2C device structure.
 */
static void omap_i2c_isr(const struct device *dev)
{
	struct i2c_omap_data *data = dev->data;
	if (omap_i2c_read_reg(dev->config, I2C_STAT) & omap_i2c_read_reg(dev->config, I2C_IE) & ~I2C_STAT_NACK) {
		k_sem_give(&data->cmd_complete);
	}
}

/**
 * @brief OMAP I2C transfer function using IRQ.
 *
 * This function performs the I2C transfer using the OMAP I2C controller
 * in interrupt-driven mode. It calls the common transfer function with the
 * specified messages, number of messages, and slave address.
 *
 * @param dev Pointer to the I2C device structure.
 * @param msgs Array of I2C messages to be transferred.
 * @param num_msgs Number of I2C messages in the array.
 * @param addr Slave address.
 * @return 0 on success, negative error code on failure.
 */
static int __maybe_unused omap_i2c_transfer_irq(const struct device *dev, struct i2c_msg msgs[], uint8_t num_msgs, uint16_t addr)
{
	return omap_i2c_xfer_common(dev, msgs, num_msgs, false, addr);
}

/**
 * @brief OMAP I2C transfer function using polling.
 *
 * This function performs the I2C transfer using the OMAP I2C controller
 * in polling mode. It calls the common transfer function with the
 * specified messages, number of messages, and slave address.
 *
 * @param dev Pointer to the I2C device structure.
 * @param msgs Array of I2C messages to be transferred.
 * @param num_msgs Number of I2C messages in the array.
 * @param addr Slave address.
 * @return 0 on success, negative error code on failure.
 */
static int __maybe_unused omap_i2c_transfer_polling(const struct device *dev, struct i2c_msg msgs[], uint8_t num_msgs, uint16_t addr)
{
	return omap_i2c_xfer_common(dev, msgs, num_msgs, true, addr);
}

static const struct i2c_driver_api omap_i2c_api = {
	.transfer = omap_i2c_transfer_polling,
	.configure = omap_i2c_configure,
	.recover_bus = omap_i2c_recover_bus,
};

#define I2C_OMAP_INIT(inst)																\
	LOG_INSTANCE_REGISTER(omap_i2c,inst,4);												\
	static void i2c_omap_##inst##_init(const struct device *dev);						\
	static const struct i2c_omap_cfg i2c_omap_cfg_##inst = {							\
		.base = DT_INST_REG_ADDR(inst),													\
		.irq = DT_INST_IRQN(inst),														\
		.init_func = i2c_omap_##inst##_init,											\
		.speed = DT_INST_PROP(inst, clock_frequency),									\
	};																					\
																						\
	static struct i2c_omap_data i2c_omap_data_##inst;									\
																						\
	I2C_DEVICE_DT_INST_DEFINE(inst,														\
				omap_i2c_init,															\
				NULL,																	\
				&i2c_omap_data_##inst,													\
				&i2c_omap_cfg_##inst,													\
				POST_KERNEL,															\
				CONFIG_I2C_INIT_PRIORITY,												\
				&omap_i2c_api);															\
																						\
	static void i2c_omap_##inst##_init(const struct device *dev)						\
	{																					\
		IRQ_CONNECT(DT_INST_IRQN(inst),													\
			0,																			\
			omap_i2c_isr,																\
			DEVICE_DT_INST_GET(inst), 0);												\
																						\
		irq_enable(DT_INST_IRQN(inst));													\
	};
DT_INST_FOREACH_STATUS_OKAY(I2C_OMAP_INIT)