#define DT_DRV_COMPAT ti_omap_i2c

#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/dt-bindings/i2c/i2c.h>
#include "i2c_bitbang.h"

LOG_MODULE_REGISTER(omap_i2c, CONFIG_OMAP_I2C_LOG_LEVEL);

typedef void (*init_func_t)(const struct device *dev);

/**
 * @brief Configuration structure for OMAP I2C driver
 */
struct i2c_omap_cfg {
	mm_reg_t base;           /**< Base address of the I2C controller */
	int irq;                 /**< IRQ number for the I2C controller */
	uint32_t speed;          /**< Speed of the I2C bus */
	init_func_t init_func;   /**< Initialization function for the I2C controller */
};


/**
 * @brief Structure representing the data for the OMAP I2C driver.
 */
struct i2c_omap_data {
	struct k_sem cmd_complete; 		/**< Semaphore for command completion */
	uint16_t cmd_err; 				/**< Command error status */
	uint16_t iestate; 				/**< I2C state */
	uint16_t scllstate; 			/**< SCL low state */
	uint16_t sclhstate; 			/**< SCL high state */
	uint16_t pscstate; 				/**< PSC state */
	uint8_t *buf; 					/**< Buffer for data transfer */
	size_t buf_len; 				/**< Length of the buffer */
	uint32_t flags; 				/**< Flags for I2C configuration */
	uint16_t westate; 				/**< Wait state */
	uint8_t fifo_size; 				/**< FIFO size */
	unsigned receiver: 1; 			/**< Receiver flag */
	unsigned b_hw: 1; 				/**< Bad hardware fixes flag */
	unsigned bb_valid: 1; 			/**< BB valid flag */
	uint32_t latency; 				/**< Latency */
	uint8_t threshold; 				/**< Threshold */
	void (*set_mpu_wkup_lat)(init_func_t dev, uint32_t latency); /**< Function pointer for setting MPU wakeup latency */
};

#define OMAP_I2C_FLAG_SIMPLE_CLOCK        BIT(1)
#define OMAP_I2C_FLAG_FORCE_19200_INT_CLK BIT(6)

/* I2C Interrupt Enable Register (OMAP_I2C_IE): */
#define OMAP_I2C_IE_XDR  BIT(14) /* TX Buffer drain int enable */
#define OMAP_I2C_IE_RDR  BIT(13) /* RX Buffer drain int enable */
#define OMAP_I2C_IE_XRDY BIT(4)  /* TX data ready int enable */
#define OMAP_I2C_IE_RRDY BIT(3)  /* RX data ready int enable */
#define OMAP_I2C_IE_ARDY BIT(2)  /* Access ready int enable */
#define OMAP_I2C_IE_NACK BIT(1)  /* No ack interrupt enable */
#define OMAP_I2C_IE_AL   BIT(0)  /* Arbitration lost int ena */

/* I2C Status Register (OMAP_I2C_STAT): */
#define OMAP_I2C_STAT_XDR  BIT(14) /* TX Buffer draining */
#define OMAP_I2C_STAT_RDR  BIT(13) /* RX Buffer draining */
#define OMAP_I2C_STAT_BB   BIT(12) /* Bus busy */
#define OMAP_I2C_STAT_ROVR BIT(11) /* Receive overrun */
#define OMAP_I2C_STAT_XUDF BIT(10) /* Transmit underflow */
#define OMAP_I2C_STAT_AAS  BIT(9)  /* Address as slave */
#define OMAP_I2C_STAT_BF   BIT(8)  /* Bus Free */
#define OMAP_I2C_STAT_XRDY BIT(4)  /* Transmit data ready */
#define OMAP_I2C_STAT_RRDY BIT(3)  /* Receive data ready */
#define OMAP_I2C_STAT_ARDY BIT(2)  /* Register access ready */
#define OMAP_I2C_STAT_NACK BIT(1)  /* No ack interrupt enable */
#define OMAP_I2C_STAT_AL   BIT(0)  /* Arbitration lost int ena */

/* I2C WE wakeup enable register */
#define OMAP_I2C_WE_XDR_WE  BIT(14) /* TX drain wakeup */
#define OMAP_I2C_WE_RDR_WE  BIT(13) /* RX drain wakeup */
#define OMAP_I2C_WE_AAS_WE  BIT(9)  /* Address as slave wakeup */
#define OMAP_I2C_WE_BF_WE   BIT(8)  /* Bus free wakeup */
#define OMAP_I2C_WE_STC_WE  BIT(6)  /* Start condition wakeup */
#define OMAP_I2C_WE_GC_WE   BIT(5)  /* General call wakeup */
#define OMAP_I2C_WE_DRDY_WE BIT(3)  /* TX/RX data ready wakeup */
#define OMAP_I2C_WE_ARDY_WE BIT(2)  /* Register access ready wakeup */
#define OMAP_I2C_WE_NACK_WE BIT(1)  /* No acknowledgment wakeup */
#define OMAP_I2C_WE_AL_WE   BIT(0)  /* Arbitration lost wakeup */

#define OMAP_I2C_WE_ALL                                                                            \
	(OMAP_I2C_WE_XDR_WE | OMAP_I2C_WE_RDR_WE | OMAP_I2C_WE_AAS_WE | OMAP_I2C_WE_BF_WE |        \
	 OMAP_I2C_WE_STC_WE | OMAP_I2C_WE_GC_WE | OMAP_I2C_WE_DRDY_WE | OMAP_I2C_WE_ARDY_WE |      \
	 OMAP_I2C_WE_NACK_WE | OMAP_I2C_WE_AL_WE)

/* I2C Buffer Configuration Register (OMAP_I2C_BUF): */
#define OMAP_I2C_BUF_RDMA_EN   BIT(15) /* RX DMA channel enable */
#define OMAP_I2C_BUF_RXFIF_CLR BIT(14) /* RX FIFO Clear */
#define OMAP_I2C_BUF_XDMA_EN   BIT(7)  /* TX DMA channel enable */
#define OMAP_I2C_BUF_TXFIF_CLR BIT(6)  /* TX FIFO Clear */

/* I2C Configuration Register (OMAP_I2C_CON): */
#define OMAP_I2C_CON_EN        BIT(15) /* I2C module enable */
#define OMAP_I2C_CON_BE        BIT(14) /* Big endian mode */
#define OMAP_I2C_CON_OPMODE_HS BIT(12) /* High Speed support */
#define OMAP_I2C_CON_STB       BIT(11) /* Start byte mode (master) */
#define OMAP_I2C_CON_MST       BIT(10) /* Master/slave mode */
#define OMAP_I2C_CON_TRX       BIT(9)  /* TX/RX mode (master only) */
#define OMAP_I2C_CON_XA        BIT(8)  /* Expand address */
#define OMAP_I2C_CON_RM        BIT(2)  /* Repeat mode (master only) */
#define OMAP_I2C_CON_STP       BIT(1)  /* Stop cond (master only) */
#define OMAP_I2C_CON_STT       BIT(0)  /* Start condition (master) */

/* I2C SCL time value when Master */
#define OMAP_I2C_SCLL_HSSCLL 8
#define OMAP_I2C_SCLH_HSSCLH 8

/* I2C System Test Register (OMAP_I2C_SYSTEST): */
#define OMAP_I2C_SYSTEST_ST_EN       BIT(15) /* System test enable */
#define OMAP_I2C_SYSTEST_FREE        BIT(14) /* Free running mode */
#define OMAP_I2C_SYSTEST_TMODE_MASK  (3 << 12) /* Test mode select mask */
#define OMAP_I2C_SYSTEST_TMODE_SHIFT (12)      /* Test mode select shift */

/* Functional mode */
#define OMAP_I2C_SYSTEST_SCL_I_FUNC  BIT(8)  /* SCL line input value */
#define OMAP_I2C_SYSTEST_SCL_O_FUNC  BIT(7)  /* SCL line output value */
#define OMAP_I2C_SYSTEST_SDA_I_FUNC  BIT(6)  /* SDA line input value */
#define OMAP_I2C_SYSTEST_SDA_O_FUNC  BIT(5)  /* SDA line output value */

/* SDA/SCL IO mode */
#define OMAP_I2C_SYSTEST_SCL_I       BIT(3)  /* SCL line sense in */
#define OMAP_I2C_SYSTEST_SCL_O       BIT(2)  /* SCL line drive out */
#define OMAP_I2C_SYSTEST_SDA_I       BIT(1)  /* SDA line sense in */
#define OMAP_I2C_SYSTEST_SDA_O       BIT(0)  /* SDA line drive out */
/* OCP_SYSSTATUS bit definitions */
#define SYSS_RESETDONE_MASK BIT(0)

/* struct omap_i2c_bus_platform_data .flags meanings */

#define OMAP_I2C_FLAG_NO_FIFO             BIT(0)
#define OMAP_I2C_FLAG_SIMPLE_CLOCK        BIT(1)
#define OMAP_I2C_FLAG_16BIT_DATA_REG      BIT(2)
#define OMAP_I2C_FLAG_ALWAYS_ARMXOR_CLK   BIT(5)
#define OMAP_I2C_FLAG_FORCE_19200_INT_CLK BIT(6)

/* OCP_SYSCONFIG bit definitions */
#define SYSC_CLOCKACTIVITY_MASK (0x3 << 8)
#define SYSC_SIDLEMODE_MASK     (0x3 << 3)
#define SYSC_ENAWAKEUP_MASK     BIT(2)
#define SYSC_SOFTRESET_MASK     BIT(1)
#define SYSC_AUTOIDLE_MASK      BIT(0)

#define SYSC_IDLEMODE_SMART     0x2
#define SYSC_CLOCKACTIVITY_FCLK 0x2

/* Errata definitions */
#define I2C_OMAP_ERRATA_I207 BIT(0)
#define I2C_OMAP_ERRATA_I462 BIT(1)

#define OMAP_I2C_IP_V2_INTERRUPTS_MASK 0x6FFF

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
#define I2C_SCLH            0xb8
#define I2C_SYSTEST         0xbc
#define I2C_BUFSTAT         0xc0
#define I2C_OA1             0xc4
#define I2C_OA2             0xc8
#define I2C_OA3             0xcc
#define I2C_ACTOA           0xd0
#define I2C_SBLOCK          0xd4


/* Define the bus free timeout */
#define OMAP_I2C_BUS_FREE_TIMEOUT 10U

/* timeout waiting for the controller to respond */
#define OMAP_I2C_TIMEOUT 1000U

static int omap_i2c_xfer_data(const struct device *dev);

/**
 * @brief Writes a 16-bit value to a register in the I2C peripheral.
 *
 * This function writes a 16-bit value to a specific register in the I2C peripheral.
 *
 * @param cfg Pointer to the configuration structure for the I2C peripheral.
 * @param data Pointer to the data structure for the I2C peripheral.
 * @param reg The register address to write the value to.
 * @param val The 16-bit value to write to the register.
 */
static inline void omap_i2c_write_reg(const struct i2c_omap_cfg *cfg, int reg, uint16_t val)
{
	int write_reg=cfg->base + reg;
	LOG_DBG("Write 0x%04x to register 0x%02x (base: 0x%lx, reg= 0x%x)", val, write_reg, cfg->base, reg);
	sys_write16(val, write_reg);
}

/**
 * @brief Reads a 16-bit value from a register in the I2C peripheral.
 *
 * This function reads a 16-bit value from a register in the I2C peripheral
 *
 * @param cfg Pointer to the I2C OMAP configuration structure.
 * @param data Pointer to the I2C OMAP data structure.
 * @param reg The register address to read from.
 * @return The 16-bit value read from the register.
 */
static inline uint16_t omap_i2c_read_reg(const struct i2c_omap_cfg *cfg, int reg)
{
	int read_reg = cfg->base + reg;
	int read_reg_val = sys_read16(read_reg);
	LOG_DBG("0x%x was read from register 0x%02x, config->base: %lx, reg= 0x%x",read_reg_val, read_reg, cfg->base, reg);
	return read_reg_val;
}

/**
 * @brief Initializes the OMAP I2C controller.
 *
 * This function initializes the OMAP I2C controller by performing the following steps:
 * - Resets the I2C module by writing to the I2C_CON register.
 * - Sets the SCL low and high time values by writing to the I2C_SCLL and I2C_SCLH registers.
 * - Takes the I2C module out of reset by writing to the I2C_CON register.
 * - Adds a delay to allow the bus to settle.
 * - Conditionally writes to the IE register if iestate is set.
 *
 * @param dev Pointer to the device structure.
 */
static void __omap_i2c_init(const struct device *dev)
{
	LOG_INF("Initializing I2C controller");
	const struct i2c_omap_cfg *cfg = dev->config;
	struct i2c_omap_data *data = dev->data;

	// Write to I2C_CON to reset I2C module
	LOG_DBG("Writing to I2C_CON register");
	omap_i2c_write_reg(cfg, I2C_CON, 0);

	// Setup clock prescaler to obtain approx 96MHz I2C module clock:
	LOG_DBG("Writing to I2C_PSC register");
	omap_i2c_write_reg(cfg, I2C_PSC, data->pscstate);

	// SCL low and high time values
	LOG_DBG("Writing to I2C_SCLL and I2C_SCLH registers");
	omap_i2c_write_reg(cfg, I2C_SCLL, data->scllstate);
	omap_i2c_write_reg(cfg, I2C_SCLH, data->sclhstate);

	// Take the I2C module out of reset
	LOG_DBG("Writing to I2C_CON register");
	omap_i2c_write_reg(cfg, I2C_CON, OMAP_I2C_CON_EN);

	// Delay to allow the bus to settle
	k_busy_wait(1);

	// Conditionally write to IE register if iestate is set
	if (data->iestate) {
		LOG_DBG("Writing to IE register");
		omap_i2c_write_reg(cfg, I2C_IRQENABLE_SET, data->iestate);
	}
}

/**
 * @brief Soft reset the I2C controller
 *
 * This function performs a soft reset of the I2C controller by disabling it,
 * waiting for the reset to complete, and then re-enabling it.
 *
 * @param dev Pointer to the device structure for the I2C controller
 * @return 0 on success, negative error code on failure
 */
static int omap_i2c_reset(const struct device *dev)
{
	LOG_INF("Resetting I2C controller");

	const struct i2c_omap_cfg *cfg = dev->config;
	struct i2c_omap_data *data = dev->data;
	uint64_t timeout;
	uint16_t sysc;

	sysc = omap_i2c_read_reg(cfg, I2C_SYSC);

	// Disable the I2C controller by clearing the enable bit in the CON register
	omap_i2c_write_reg(cfg, I2C_CON, omap_i2c_read_reg(cfg, I2C_CON) & ~(OMAP_I2C_CON_EN));

	// Set the timeout value to 1000 milliseconds from the current uptime
	timeout = k_uptime_get() + OMAP_I2C_TIMEOUT;

	// Re-enable the I2C controller
	omap_i2c_write_reg(cfg, I2C_CON, OMAP_I2C_CON_EN);

	// Wait for the reset to complete by polling the SYSS register
	while (!(omap_i2c_read_reg(cfg, I2C_SYSS) & SYSS_RESETDONE_MASK)) {
		// Check if the current uptime exceeds the timeout value
		if (k_uptime_get() > timeout) {
			// Log a warning message if the reset timeout occurs
			LOG_WRN("timeout waiting for controller reset");
			// Return a timeout error
			return -ETIMEDOUT;
		}
		// Sleep for 1 millisecond before polling again
		k_msleep(1);
	}

	// The SYSC register is cleared by the reset; rewrite its original value
	omap_i2c_write_reg(cfg, I2C_SYSC, sysc);

	// Mark the bus monitoring as invalid, to be updated on the next transfer
	data->bb_valid = 0;
	LOG_DBG("I2C controller reset complete");
	// Return success
	return 0;
}
/**
 * @brief Initialize the OMAP I2C controller.
 *
 * This function initializes the OMAP I2C controller by configuring the speed, base address, and IRQ.
 * It also initializes the command completion semaphore and sets the flags for simple clock configuration.
 *
 * @param dev Pointer to the device structure.
 * @return 0 on success, negative error code on failure.
 */
static int omap_i2c_init(const struct device *dev)
{
	LOG_INF("Initializing I2C controller");
	struct i2c_omap_data *data = dev->data;
	const struct i2c_omap_cfg *cfg = dev->config;

	// Print the speed, base, and irq
	LOG_INF("Speed: %d Kbps", cfg->speed);
	LOG_INF("Base: 0x%lx", cfg->base);
	LOG_INF("IRQ: %d", cfg->irq);

	k_sem_init(&data->cmd_complete, 0, 1);
	data->flags |= OMAP_I2C_FLAG_SIMPLE_CLOCK;
	uint16_t psc = 0, scll = 0, sclh = 0;
	uint16_t fsscll = 0, fssclh = 0;
	unsigned long fclk_rate = 96000000;
	unsigned long internal_clk = 0;
	unsigned long speed = cfg->speed;

	data->westate = OMAP_I2C_WE_ALL;

	// if (omap->flags & OMAP_I2C_FLAG_ALWAYS_ARMXOR_CLK) {
	// 	clk_dev = device_get_binding("CLOCK_CONTROL");
	// 	if (!clk_dev) {
	// 		LOG_ERR("Failed to get clock device binding");
	// 		return -ENODEV;
	// 	}

	// 	if (clock_control_get_rate(clk_dev, CLOCK_CONTROL_SUBSYS_BASE, &fclk_rate)) {
	// 		LOG_ERR("Could not get fck rate");
	// 		return -EIO;
	// 	}

	// 	if (fclk_rate > 12000000) {
	// 		psc = fclk_rate / 12000000;
	// 	}
	// }

	// if (!(data->flags & OMAP_I2C_FLAG_SIMPLE_CLOCK)) {
		 if (cfg->speed > I2C_BITRATE_STANDARD) {
			LOG_DBG("Speed > 100 Kbps");
			internal_clk = 9600;
		} else {
			internal_clk = 4000;
		}

	// 	clk_dev = device_get_binding("CLOCK_CONTROL");
	// 	if (!clk_dev) {
	// 		LOG_ERR("Failed to get clock device binding");
	// 		return -ENODEV;
	// 	}

	// 	if (clock_control_get_rate(clk_dev, CLOCK_CONTROL_SUBSYS_BASE, &fclk_rate)) {
	// 		LOG_ERR("Could not get fck rate");
	// 		return -EIO;
	// 	}
		fclk_rate /= 1000;
		speed /= 1000;
		psc = fclk_rate / internal_clk;
		psc = psc - 1;

		if (cfg->speed > I2C_BITRATE_FAST) {
			LOG_ERR("Speed > 400 Kbps not supported");
			// unsigned long scl;

			// scl = internal_clk / 400;
			// fsscll = scl - (scl / 3) - 7;
			// fssclh = (scl / 3) - 5;

			// scl = fclk_rate / cfg->speed;
			// hsscll = scl - (scl / 3) - 7;
			// hssclh = (scl / 3) - 5;
		} else {
			LOG_DBG("Speed <= 400 Kbps");
			fsscll = internal_clk / (speed * 2) - 7;
			fssclh = internal_clk / (speed * 2) - 5;
		}

		scll = fsscll;
		sclh = fssclh;
	// } else {
	// 	LOG_DBG("Simple clock configuration");
	// 	fclk_rate /= (psc + 1) * 1000;
	// 	if (psc > 2) {
	// 		psc = 2;
	// 	}
	// 	scll = fclk_rate / (speed * 2) - 7 + psc;
	// 	sclh = fclk_rate / (speed * 2) - 7 + psc;
	// }


	// As there is no clock control module, we will be fixing it to 100 Kbps
	// For simplicity, always set the internal clock to 4000 kHz for 100 kbps operation
	// internal_clk = 4000;

	// Prescaler Calculation and clock periods for 100 kbps
	// psc = 23;  // 23
	// scll = 13;  //13
	// sclh = 15;  //15

	data->iestate = (OMAP_I2C_IE_XRDY | OMAP_I2C_IE_RRDY | OMAP_I2C_IE_ARDY | OMAP_I2C_IE_NACK |
			 OMAP_I2C_IE_AL) |
			((data->fifo_size) ? (OMAP_I2C_IE_RDR | OMAP_I2C_IE_XDR) : 0);

	data->pscstate = psc;
	data->scllstate = scll;
	data->sclhstate = sclh;
	LOG_DBG("PSC: %d, SCLL: %d, SCLH: %d", psc, scll, sclh);

	uint16_t rev_low = omap_i2c_read_reg(cfg, I2C_REVNB_LO);
	uint16_t rev_high = omap_i2c_read_reg(cfg, I2C_REVNB_HI);

	LOG_DBG("I2C OMAP init called, REV_LO: 0x%08x, REV_HI: 0x%08x", rev_low, rev_high);

	__omap_i2c_init(dev);

	return 0;
}

/**
 * @brief Interrupt service routine for the OMAP I2C driver.
 *
 * This function handles the interrupt generated by the I2C controller.
 * It reads the status register and checks for any enabled events.
 * If an event is detected, it performs the necessary actions to handle the interrupt.
 *
 * @param dev Pointer to the device structure.
 */
static void omap_i2c_isr(const struct device *dev)
{
	LOG_INF("I2C interrupt service routine");
	const struct i2c_omap_cfg *cfg = dev->config;
	uint16_t stat;
	uint16_t mask;

	stat = omap_i2c_read_reg(cfg, I2C_STAT);
	mask = omap_i2c_read_reg(cfg, I2C_IE) & ~OMAP_I2C_STAT_NACK;

	if (stat & mask) {
		LOG_DBG("I2C event detected");
		// Handle the interrupt (wake the thread, etc.)
	}
}
/**
 * @brief Configures the I2C controller.
 *
 * This function configures the I2C controller based on the provided device configuration.
 *
 * @param dev The pointer to the I2C device structure.
 * @param dev_config The device configuration.
 *
 * @retval 0 If successful.
 * @retval -ENOTSUP If the I2C controller does not support the requested configuration.
 */
static int omap_i2c_configure(const struct device *dev, uint32_t dev_config)
{
	LOG_INF("Configuring I2C controller");

	switch (I2C_SPEED_GET(dev_config))
	{
		case I2C_SPEED_STANDARD:
			/* Use recommended value for 100 kHz bus */
			LOG_DBG("I2C controller Speed: I2C_SPEED_STANDARD(100 Kbps)");
			goto out;
		case I2C_SPEED_FAST:
			LOG_DBG("I2C controller Speed: I2C_SPEED_FAST(400 Kbps)");
			goto out;
		default:
			return -ENOTSUP;
	}

	// const struct i2c_omap_cfg *cfg = dev->config;)
	if ((dev_config & I2C_MODE_CONTROLLER) != I2C_MODE_CONTROLLER)
	{
		LOG_DBG("I2C controller does not support slave mode");
		return -ENOTSUP;
	}

	if ((dev_config & I2C_MSG_ADDR_10_BITS) != I2C_MSG_ADDR_10_BITS)
	{
		LOG_DBG("I2C controller does not support 10-bit addressing");
		return -ENOTSUP;
	}


out:
	omap_i2c_reset(dev);
	__omap_i2c_init(dev);

	return 0;
}

/**
 * @brief Transmit data over I2C bus
 *
 * This function transmits a specified number of bytes over the I2C bus.
 *
 * @param dev Pointer to the I2C device structure
 * @param num_bytes Number of bytes to transmit
 * @return 0 on success, negative error code on failure
 */
static int omap_i2c_transmit_data(const struct device *dev, uint8_t num_bytes)
{
	LOG_INF("Transmitting %d bytes", num_bytes);
	const struct i2c_omap_cfg *cfg = dev->config;
	struct i2c_omap_data *data = dev->data;
	uint8_t buffer_data;

	while (num_bytes--) {
		buffer_data = *data->buf++;
		data->buf_len--;

		omap_i2c_write_reg(cfg, I2C_DATA, buffer_data);
	}
	LOG_DBG("Data transmitted successfully");
	return 0;
}

/**
 * @brief Receive data from the I2C bus.
 *
 * This function receives a specified number of bytes from the I2C bus.
 *
 * @param dev The I2C device structure.
 * @param num_bytes The number of bytes to receive.
 */
static void omap_i2c_receive_data(const struct device *dev, uint8_t num_bytes)
{
	LOG_INF("Receiving %d bytes", num_bytes);
	const struct i2c_omap_cfg *cfg = dev->config;
	struct i2c_omap_data *data = dev->data;
	uint16_t buffer_data;
	while (num_bytes--) {
		buffer_data = omap_i2c_read_reg(cfg, I2C_DATA);
		*data->buf++ = buffer_data;
		data->buf_len--;
		LOG_DBG("Receiving byte: %x", buffer_data);
	}
	LOG_DBG("Data received successfully");
}

/**
 * @brief Resizes the FIFO for the I2C controller.
 *
 * This function resizes the FIFO for either the RX or TX direction based on the specified size.
 * It also sets up the notification threshold based on the message size.
 * If the FIFO is not supported, the function returns without making any changes.
 * After resizing the FIFO, it calculates the wakeup latency constraint for the MPU (Microprocessor Unit).
 *
 * @param dev The I2C device structure.
 * @param size The size of the FIFO to be set.
 * @param is_rx A boolean indicating whether the FIFO is for RX (true) or TX (false).
 *
 * @return None.
 */
static void omap_i2c_resize_fifo(const struct device *dev, uint8_t size, bool is_rx)
{
	LOG_INF("Resizing FIFO for %s %d bytes", is_rx ? "RX" : "TX", size);
	const struct i2c_omap_cfg *cfg = dev->config;
	struct i2c_omap_data *data = dev->data;
	uint16_t buf;
	if (data->flags & OMAP_I2C_FLAG_NO_FIFO) {
		LOG_DBG("FIFO not supported");
		return;
	}

	// Set up notification threshold based on message size
	data->threshold = CLAMP(size, (uint8_t)1, data->fifo_size);
	buf = omap_i2c_read_reg(cfg, I2C_BUF);
	LOG_DBG("Current buffer value: 0x%04x", buf);
	if (is_rx) {
		// Clear RX threshold
		LOG_DBG("Clearing RX threshold");	
		buf &= ~(0x3f << 8);
		buf |= ((data->threshold - 1) << 8) | OMAP_I2C_BUF_RXFIF_CLR;
	} else {
		// Clear TX threshold
		LOG_DBG("Clearing TX threshold");
		buf &= ~0x3f;
		buf |= (data->threshold - 1) | OMAP_I2C_BUF_TXFIF_CLR;
	}
	LOG_DBG("Writing buffer value: 0x%04x", buf);
	omap_i2c_write_reg(cfg, I2C_BUF, buf);
	data->fifo_size = buf;

	// Calculate wakeup latency constraint for MPU
	if (data->set_mpu_wkup_lat != NULL) {
		LOG_DBG("Calculating wakeup latency constraint");
		data->latency = (1000000 * data->threshold) / (1000 * cfg->speed / 8);
		data->set_mpu_wkup_lat(cfg->init_func, data->latency);
	}
	LOG_DBG("FIFO resized successfully");
}
/**
 * @brief Waits for an I2C event by polling the status register.
 *
 * This function waits for an I2C event by continuously polling the status register
 * until an event occurs or a maximum number of attempts is reached.
 *
 * @param dev Pointer to the I2C device structure.
 * @return 0 if an event occurs, or -EAGAIN if the maximum number of attempts is reached.
 */
static int omap_i2c_wait(const struct device *dev)
{
	LOG_INF("Waiting for I2C event");
	const struct i2c_omap_cfg *cfg = dev->config;
	uint16_t stat;
	//	uint16_t mask = omap_i2c_read_reg(cfg, data, I2C_IE);
	int count = 0;

	for (count = 0; count < 5; count++) {
		stat = omap_i2c_read_reg(cfg, I2C_STAT);
		if (stat){
			LOG_DBG("I2C event detected");
			return 0;
		}
	}
	LOG_DBG("Maximum number of attempts reached");
	return -EAGAIN;
}

/**
 * @brief Get the status of the SCL line for the OMAP I2C controller.
 *
 * This function reads the value of the SCL line from the I2C_SYSTEST register
 * and returns the status of the SCL line.
 *
 * @param dev Pointer to the device structure for the OMAP I2C controller.
 * @return The status of the SCL line. Returns a non-zero value if the SCL line
 *         is functional, and zero otherwise.
 */

static int __maybe_unused omap_i2c_get_scl(void *io_context)
{
	const struct i2c_omap_cfg *cfg = io_context;
	uint32_t reg;

	reg = omap_i2c_read_reg(cfg, I2C_SYSTEST);

	return reg & OMAP_I2C_SYSTEST_SCL_I_FUNC;
}

/**
 * @brief Get the status of the SDA (Serial Data Line) pin for the OMAP I2C controller.
 *
 * This function retrieves the status of the SDA pin for the specified I2C device.
 *
 * @param dev Pointer to the device structure.
 * @return The status of the SDA pin. Returns a non-zero value if the SDA pin is functional,
 *         and returns zero if the SDA pin is not functional.
 */
static int omap_i2c_get_sda(void *io_context)
{
	
	const struct i2c_omap_cfg *cfg = io_context;
	uint32_t reg;

	reg = omap_i2c_read_reg(cfg, I2C_SYSTEST);

	return reg & OMAP_I2C_SYSTEST_SDA_I_FUNC;
}

static void omap_i2c_set_sda(void *io_context, int state)
{
	const struct i2c_omap_cfg *cfg = io_context;
	uint32_t reg;

	reg = omap_i2c_read_reg(cfg, I2C_SYSTEST);

	if (state) {
		reg |= OMAP_I2C_SYSTEST_SDA_O;
	} else {
		reg &= ~OMAP_I2C_SYSTEST_SDA_O;
	}

	omap_i2c_write_reg(cfg, I2C_SYSTEST, reg);
}

static void omap_i2c_set_scl(void *io_context, int state)
{
	const struct i2c_omap_cfg *cfg = io_context;
	uint32_t reg;

	reg = omap_i2c_read_reg(cfg, I2C_SYSTEST);

	if (state) {
		reg |= OMAP_I2C_SYSTEST_SCL_O;
	} else {
		reg &= ~OMAP_I2C_SYSTEST_SCL_O;
	}

	omap_i2c_write_reg(cfg, I2C_SYSTEST, reg);
}

/**
 * @brief Try bus recovery, but only if SDA is actually low.
 *
 * This function attempts to recover the I2C bus if it is in a stuck state.
 * It checks the status of the SDA and SCL lines using the I2C_SYSTEST register.
 * If SDA is not stuck low, the bus is considered to be already fine and no recovery is needed.
 * If SCL is stuck low, recovery would not fix the issue and an error is returned.
 * Otherwise, the function calls the i2c_recover_bus() function to perform the bus recovery.
 *
 * @param dev Pointer to the I2C device structure.
 * @return 0 if the bus is already fine or recovery is successful, negative error code otherwise.
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
	reg |= OMAP_I2C_SYSTEST_ST_EN | 3 << OMAP_I2C_SYSTEST_TMODE_SHIFT | OMAP_I2C_SYSTEST_SCL_O | OMAP_I2C_SYSTEST_SDA_O;
	omap_i2c_write_reg(cfg, I2C_SYSTEST, reg);

	i2c_bitbang_init(&bitbang_omap, &bitbang_omap_io, (void *)cfg);

	error = i2c_bitbang_recover_bus(&bitbang_omap);
	if (error != 0) {
		LOG_ERR("failed to recover bus (err %d)", error);
		goto restore;
	}


restore:
	reg = omap_i2c_read_reg(cfg, I2C_SYSTEST);
	reg &= ~OMAP_I2C_SYSTEST_ST_EN & ~OMAP_I2C_SYSTEST_TMODE_MASK & ~OMAP_I2C_SYSTEST_SCL_O & ~OMAP_I2C_SYSTEST_SDA_O;
	omap_i2c_write_reg(cfg, I2C_SYSTEST, reg);
	return error;
}

/**
 * @brief Wait for the I2C bus to become idle.
 *
 * This function waits for the I2C bus to become idle by checking the BUS
 * busy status bit in the I2C status register. If the bus remains busy for
 * longer than the specified timeout, the function attempts to recover the
 * bus by calling i2c_recover_bus().
 *
 * @param dev Pointer to the I2C device structure.
 * @return 0 on success, negative errno code on failure.
 */
static int omap_i2c_wait_for_bb(const struct device *dev)
{
	LOG_INF("Waiting for BUS busy to clear");
	const struct i2c_omap_cfg *cfg = dev->config;
	uint32_t timeout;

	timeout = k_uptime_get_32() + OMAP_I2C_TIMEOUT;
	while (omap_i2c_read_reg(cfg, I2C_STAT) & OMAP_I2C_STAT_BB) {
		LOG_DBG("Waiting for BUS busy to clear");
		if (k_uptime_get_32() > timeout){
			LOG_DBG("timeout waiting for bus ready");
			return omap_i2c_recover_bus(dev);
		} 
		k_msleep(1);
	}
	LOG_DBG("BUS busy cleared");
	return 0;
}

/*
 * Wait while BB-bit doesn't reflect the I2C bus state
 *
 * In a multimaster environment, after IP software reset, BB-bit value doesn't
 * correspond to the current bus state. It may happen what BB-bit will be 0,
 * while the bus is busy due to another I2C master activity.
 * Here are BB-bit values after reset:
 *     SDA   SCL   BB   NOTES
 *       0     0    0   1, 2
 *       1     0    0   1, 2
 *       0     1    1
 *       1     1    0   3
 * Later, if IP detect SDA=0 and SCL=1 (ACK) or SDA 1->0 while SCL=1 (START)
 * combinations on the bus, it set BB-bit to 1.
 * If IP detect SDA 0->1 while SCL=1 (STOP) combination on the bus,
 * it set BB-bit to 0 and BF to 1.
 * BB and BF bits correctly tracks the bus state while IP is suspended
 * BB bit became valid on the next FCLK clock after CON_EN bit set
 *
 * NOTES:
 * 1. Any transfer started when BB=0 and bus is busy wouldn't be
 *    completed by IP and results in controller timeout.
 * 2. Any transfer started when BB=0 and SCL=0 results in IP
 *    starting to drive SDA low. In that case IP corrupt data
 *    on the bus.
 * 3. Any transfer started in the middle of another master's transfer
 *    results in unpredictable results and data corruption
 */

static int __maybe_unused omap_i2c_wait_for_bb_valid(const struct device *dev)
{
	LOG_INF("Waiting for BB-bit to become valid");
	uint32_t bus_free_timeout = 0;
	uint32_t timeout;
	int bus_free = 0;
	uint16_t stat, systest;
	struct i2c_omap_data *data = dev->data;
	const struct i2c_omap_cfg *cfg = dev->config;

	if (data->bb_valid)
		{
			LOG_DBG("BB-bit is already valid");
		return 0;
		}
	timeout = k_uptime_get_32() + OMAP_I2C_TIMEOUT;
	while (1) {
		LOG_DBG("Waiting for BB-bit to become valid");
	stat = omap_i2c_read_reg(cfg, I2C_STAT);

	// We will see BB or BF event in a case IP had detected any
	// activity on the I2C bus. Now IP correctly tracks the bus
	// state. BB-bit value is valid.
	if (stat & (OMAP_I2C_STAT_BB | OMAP_I2C_STAT_BF)){
			LOG_DBG("BB or BF event detected");
		break;
		}
	// Otherwise, we must look signals on the bus to make the right decision.
	systest = omap_i2c_read_reg(cfg, I2C_SYSTEST);
	if ((systest & OMAP_I2C_SYSTEST_SCL_I_FUNC) &&
		(systest & OMAP_I2C_SYSTEST_SDA_I_FUNC)) {
		if (!bus_free) {
			LOG_DBG("Bus free detected");
			bus_free_timeout = k_uptime_get_32() + OMAP_I2C_BUS_FREE_TIMEOUT;
			bus_free = 1;
		}

		// SDA and SCL lines were high for 10 ms without bus activity detected.
		// The bus is free. Consider BB-bit value is valid.
		if (k_uptime_get_32() > bus_free_timeout) {
			LOG_DBG("Bus free timeout");
			break;
		}
		} else {
			// SDA or SCL line is low. The bus is busy. Reset the bus free timeout.
			LOG_DBG("Resetting bus free timeout");
		bus_free = 0;
	}

	if (k_uptime_get_32() > timeout) {
		// SDA or SCL were low for the entire timeout without any activity detected.
		// Most likely, a slave is locking up the bus with no master driving the clock.
		LOG_WRN("timeout waiting for bus ready");
		return i2c_recover_bus(dev);
	}

	k_msleep(1);
	}

	data->bb_valid = 1;
	return 0;
}

/**
 * @brief Performs a low-level master read/write transaction on the I2C bus.
 *
 * This function is responsible for handling the low-level details of an I2C transfer,
 * including setting up the target device address, buffer length, control register settings,
 * and handling various error conditions. It supports both polling and non-polling modes.
 *
 * @param dev The I2C device structure.
 * @param msg Pointer to the I2C message structure containing the transfer details.
 * @param stop Flag indicating whether to send a STOP condition after the transfer.
 * @param polling Flag indicating whether to use polling mode for the transfer.
 * @param addr The target device address.
 * @return 0 on success, negative error code on failure.
 */
static int omap_i2c_xfer_msg(const struct device *dev, struct i2c_msg *msg, int stop, bool polling,
							 uint16_t addr)
{
	LOG_INF("Starting I2C transfer message");
	const struct i2c_omap_cfg *cfg = dev->config;
	struct i2c_omap_data *data = dev->data;

	unsigned long time_left;
	uint16_t control_reg;
	int ret;

	LOG_DBG("addr: 0x%04x, len: %d, flags: 0x%x, stop: %d", addr, msg->len, msg->flags, stop);

	data->receiver = !!(msg->flags & I2C_MSG_READ);
	omap_i2c_resize_fifo(dev, msg->len, data->receiver);

	// Write target device address to I2C slave address register
	LOG_DBG("Writing target device address 0x%x to I2C slave address register", addr);
	omap_i2c_write_reg(cfg, I2C_SA, addr);
	// Set buffer and buffer length for the message
	data->buf = msg->buf;
	/* make sure writes to data->buf_len are ordered */

	data->buf_len = msg->len;

	// Ensure memory ordering with compiler barrier
	compiler_barrier();

	// Write buffer length to I2C count register
	omap_i2c_write_reg(cfg, I2C_CNT, data->buf_len);
	LOG_DBG("Writing buffer length %d to I2C count register", data->buf_len);
	// Clear FIFO buffers in the I2C controller
	LOG_DBG("Clearing FIFO buffers in the I2C controller");
	control_reg = omap_i2c_read_reg(cfg, I2C_BUF);
	control_reg |= OMAP_I2C_BUF_RXFIF_CLR | OMAP_I2C_BUF_TXFIF_CLR;
	omap_i2c_write_reg(cfg, I2C_BUF, control_reg);

	// If not using polling, reset command completion semaphore
	if (!polling) {
		LOG_DBG("Resetting command completion semaphore");
		k_sem_reset(&data->cmd_complete);
	}

	// Reset command error flag
	data->cmd_err = 0;
	// Configure control register bits for I2C operation
	control_reg = OMAP_I2C_CON_EN | OMAP_I2C_CON_MST | OMAP_I2C_CON_STT;

	// Set high-speed mode if configured speed is greater than 400 kHz
	if (cfg->speed > I2C_BITRATE_FAST) {
		LOG_DBG("Setting high-speed mode");
		control_reg |= OMAP_I2C_CON_OPMODE_HS;
	}
	// Set STOP condition flag if specified in message flags
	if (msg->flags & I2C_MSG_STOP) {
		LOG_DBG("Setting stop condition flag");
		stop = 1;
	}
	// Set TRX (transmit/receive) flag based on message direction
	if (!(msg->flags & I2C_MSG_READ)) {
		LOG_DBG("Setting TRX flag for transmit");
		control_reg |= OMAP_I2C_CON_TRX;
	}
	// Set STOP condition flag in non-hardware mode with stop condition specified
	if (!data->b_hw && stop) {
		LOG_DBG("Setting stop condition in non-hardware mode");
		control_reg |= OMAP_I2C_CON_STP;
	}


	// Write control register settings to I2C control register
	LOG_DBG("Writing control register settings to I2C control register");
	omap_i2c_write_reg(cfg, I2C_CON, control_reg);

	// Handle specific cases for non-hardware mode with stop condition
	if (data->b_hw && stop) {
		LOG_DBG("Handling non-hardware mode with stop condition");
		uint64_t current_time = k_uptime_get(); // Get current uptime in milliseconds
		uint64_t delay = current_time + 1000;   // Add timeout duration in ms
		uint16_t con = omap_i2c_read_reg(cfg, I2C_CON);
		while (con & OMAP_I2C_CON_STT) {
			con = omap_i2c_read_reg(cfg, I2C_CON);
			// Timeout handling if start condition doesn't finish within timeout period
			if (k_uptime_get() > delay) {
				LOG_ERR("controller timed out waiting for start condition to finish");
				return -ETIMEDOUT;
			}
			k_sleep(K_MSEC(1)); // Yield to avoid busy looping
		}
		control_reg |= OMAP_I2C_CON_STP;
		control_reg &= OMAP_I2C_CON_STT;
		omap_i2c_write_reg(cfg, I2C_CON, control_reg);
	}

	// If not using polling, wait for completion of command
	if (!polling) {
	LOG_DBG("Waiting for command completion");
	int result = k_sem_take(&data->cmd_complete, K_MSEC(OMAP_I2C_TIMEOUT));
	LOG_DBG("Result: %d", result);
	if (result == -EAGAIN) {
		LOG_DBG("Timeout occurred");
		// Timeout occurred, handle it accordingly
		time_left = -ETIMEDOUT; // or any other timeout handling
	}
	} else {
	// Polling mode: loop until data transfer is complete
		LOG_DBG("Polling mode");
		do {
			omap_i2c_wait(dev);
			ret = omap_i2c_xfer_data(dev);
		} while (ret == -EAGAIN);
		time_left = !ret; 
		
	}
	// Handle timeout scenario: reset and re-initialize I2C controller
	if (time_left == 0) {
		omap_i2c_reset(dev);
		__omap_i2c_init(dev);
		LOG_DBG("I2C controller reset and re-initialized");
		return -ETIMEDOUT;
	}

	// Handle command errors
	if (likely(!data->cmd_err)) {
		LOG_DBG("No command errors");
		LOG_DBG("Exiting at line %d", __LINE__);
		return 0; // No error, return success
	}

	// Handle specific error conditions
	if (data->cmd_err & (OMAP_I2C_STAT_ROVR | OMAP_I2C_STAT_XUDF)) {
		omap_i2c_reset(dev);
		__omap_i2c_init(dev);
		LOG_DBG("I/O error due to receiver overrun or transmit underflow");
		return -EIO; // I/O error due to receiver overrun or transmit underflow
	}
	if (data->cmd_err & OMAP_I2C_STAT_AL) {
		LOG_DBG("Arbitration lost error");
		return -EAGAIN; // Arbitration lost error, try again
	}
	if (data->cmd_err & OMAP_I2C_STAT_NACK) {
		LOG_DBG("NACK received");
		// NACK error handling: return 0 for non-essential messages, otherwise remote error
		if (msg->flags) {
			LOG_DBG("NACK received");
			return 0;
		}
		control_reg = omap_i2c_read_reg(cfg, I2C_CON);
		control_reg |= OMAP_I2C_CON_STP;
		omap_i2c_write_reg(cfg, I2C_CON, control_reg);
		return -EREMOTE;
	}
	LOG_DBG("Exiting at line %d", __LINE__);
	return -EIO; // General I/O error if none of the specific error conditions match
}

/**
 * @brief Writes a given status value to the I2C status register to acknowledge or clear specific status bits.
 *
 * This function acknowledges or clears specific status bits in the I2C status register by writing the provided status value.
 *
 * @param dev Pointer to the device structure for the I2C controller.
 * @param stat The status value to be written to the I2C status register.
 *
 * @return None.
 */
static inline void omap_i2c_ack_stat(const struct device *dev, uint16_t stat)
{
	LOG_INF("Acknowledge status 0x%04x", stat);
	const struct i2c_omap_cfg *cfg = dev->config;
	omap_i2c_write_reg(cfg, I2C_STAT, stat);
	LOG_DBG("Status acknowledged");
}

/**
 * @brief Handles data transfer for the I2C controller.
 *
 * This function handles the data transfer for the I2C controller. It reads the interrupt enable bits and status register,
 * masks the status register with the interrupt enable bits, and handles specific status bits and errors. It also handles
 * the receive and transmit data ready status, receive and transmit ready status, receive overrun error, and transmit
 * underflow error. The function continues processing while there are status bits set and returns the command error.
 *
 * @param dev The pointer to the I2C device structure.
 * @return The command error.
 */
static int omap_i2c_xfer_data(const struct device *dev)
{
	LOG_INF("Handling I2C data transfer");
	const struct i2c_omap_cfg *cfg = dev->config;
	struct i2c_omap_data *data = dev->data;

	uint16_t bits, stat; 
	int count = 0; 
	int err = 0;

	do {
		// Read the interrupt enable bits and status register
		bits = omap_i2c_read_reg(cfg, I2C_IE);
		LOG_DBG("bits: 0x%04x\n", bits);
		stat = omap_i2c_read_reg(cfg, I2C_STAT);
		LOG_DBG("stat: 0x%04x\n", stat);
		// Mask the status register with the interrupt enable bits
		// stat &= bits;
		// stat &= ~OMAP_I2C_STAT_BF; //Bus free status bit

		// If in receiver mode, ignore transmit-related status bits else, ignore receive-related status bits
		if (data->receiver) {
			LOG_DBG("Receiver mode");
			stat &= ~(OMAP_I2C_STAT_XDR | OMAP_I2C_STAT_XRDY);
		} else {
			LOG_DBG("Transmitter mode");
			stat &= ~(OMAP_I2C_STAT_RDR | OMAP_I2C_STAT_RRDY);
		}

		// If no relevant status bits are set, exit with an EAGAIN error
		if (!stat) {
			LOG_ERR("No relevant status bits set");
			LOG_DBG("STAT empty");
			err = -EAGAIN;
			return err;
		}

		LOG_DBG("IRQ (ISR = 0x%04x)", stat);

		if (count++ == 10) {
			LOG_WRN("Too much work in one IRQ");
			err = -ETIMEDOUT;
			return err;
		}

		// Handle specific status bits and errors
		if (stat & OMAP_I2C_STAT_NACK) {
			LOG_ERR("NACK received"); // Log NACK error
			err |= OMAP_I2C_STAT_NACK; // Set NACK error flag
			omap_i2c_ack_stat(dev, OMAP_I2C_STAT_NACK); // Acknowledge the NACK status
		}

		if (stat & OMAP_I2C_STAT_AL) {
			LOG_ERR("Arbitration lost"); // Log arbitration loss error
			err |= OMAP_I2C_STAT_AL; // Set arbitration lost error flag
			omap_i2c_ack_stat(dev, OMAP_I2C_STAT_AL); // Acknowledge the arbitration lost status
		}

		if (stat & OMAP_I2C_STAT_ARDY) {
			LOG_DBG("Access ready"); 
			omap_i2c_ack_stat(dev, OMAP_I2C_STAT_ARDY); // Acknowledge the ARDY status
		}

		// If any of the major status bits are set, clear the FIFO and exit
		if (stat & (OMAP_I2C_STAT_ARDY | OMAP_I2C_STAT_NACK | OMAP_I2C_STAT_AL)) {
			LOG_DBG("Clearing FIFO and exiting");
			omap_i2c_ack_stat(dev, (OMAP_I2C_STAT_RRDY | OMAP_I2C_STAT_RDR |
						OMAP_I2C_STAT_XRDY | OMAP_I2C_STAT_XDR |
						OMAP_I2C_STAT_ARDY));
			return 0;
		}

		// Handle the receive data ready status
		if (stat & OMAP_I2C_STAT_RDR) {
			uint8_t num_bytes = 1;
			LOG_DBG("Receive data ready status");
			// If FIFO size is set, determine the number of bytes to read
			if (data->fifo_size) {
				num_bytes = data->buf_len;
				LOG_DBG("Receiving %d bytes", num_bytes);
			}
			omap_i2c_receive_data(dev, num_bytes); // Receive data
			omap_i2c_ack_stat(dev, OMAP_I2C_STAT_RDR); // Acknowledge the RDR status
			continue;
		}

		// Handle the receive ready status
		if (stat & OMAP_I2C_STAT_RRDY) {
			uint8_t num_bytes = 1;
			LOG_DBG("Receive ready status");
			// If threshold is set, determine the number of bytes to read
			if (data->threshold) {
				num_bytes = data->threshold;
			}

			omap_i2c_receive_data(dev, num_bytes); // Receive data
			omap_i2c_ack_stat(dev, OMAP_I2C_STAT_RRDY); // Acknowledge the RRDY status
			continue;
		}

		// Handle the transmit data ready status
		if (stat & OMAP_I2C_STAT_XDR) {
			uint8_t num_bytes = 1;
			int ret;
			LOG_DBG("Transmit data ready status");
			// If FIFO size is set, determine the number of bytes to transmit
			if (data->fifo_size) {
				num_bytes = data->buf_len;
				LOG_DBG("Transmitting %d bytes", num_bytes);
			}

			ret = omap_i2c_transmit_data(dev, num_bytes); // Transmit data
			if (ret < 0) {
				break;
			}

			omap_i2c_ack_stat(dev, OMAP_I2C_STAT_XDR); // Acknowledge the XDR status
			continue;
		}

		// Handle the transmit ready status
		if (stat & OMAP_I2C_STAT_XRDY) {
			uint8_t num_bytes = 1;
			int ret;
			LOG_DBG("Transmit ready status");
			// If threshold is set, determine the number of bytes to transmit
			if (data->threshold) {
				LOG_DBG("Transmitting %d bytes", data->threshold);
				num_bytes = data->threshold;
			}

			ret = omap_i2c_transmit_data(dev, num_bytes); // Transmit data
			if (ret < 0) {
				break;
			}

			omap_i2c_ack_stat(dev, OMAP_I2C_STAT_XRDY); // Acknowledge the XRDY status
			continue;
		}

		// Handle receive overrun error
		if (stat & OMAP_I2C_STAT_ROVR) {
			LOG_ERR("Receive overrun"); // Log receive overrun error
			err |= OMAP_I2C_STAT_ROVR; // Set receive overrun error flag
			omap_i2c_ack_stat(dev, OMAP_I2C_STAT_ROVR); // Acknowledge the ROVR status
			break;
		}

		// Handle transmit underflow error
		if (stat & OMAP_I2C_STAT_XUDF) {
			LOG_ERR("Transmit underflow"); // Log transmit underflow error
			err |= OMAP_I2C_STAT_XUDF; // Set transmit underflow error flag
			omap_i2c_ack_stat(dev, OMAP_I2C_STAT_XUDF); // Acknowledge the XUDF status
			break;
		}
		LOG_DBG("Redo at line %d", __LINE__);
	} while (stat); // Continue processing while there are status bits set
	LOG_DBG("Exiting at line %d", __LINE__);
	return err; 
}

/**
 * @brief Prepare controller for a transaction and call omap_i2c_xfer_msg to do the work during IRQ processing.
 *
 * This function prepares the I2C controller for a transaction and then calls omap_i2c_xfer_msg to perform the actual data transfer.
 * It waits for the bus busy condition to clear, sets the MPU wakeup latency if available, and then transfers each message in the array.
 * If a message transfer fails, the function exits the loop and returns the error code.
 * If all message transfers are successful, the function returns the number of messages transferred and marks the I2C transfer as completed.
 * If an error occurs during the bus busy wait, the function returns the error code and marks the I2C transfer as completed.
 *
 * @param dev The I2C device structure.
 * @param msg An array of I2C messages to be transferred.
 * @param num The number of messages in the array.
 * @param polling Flag indicating whether to use polling mode or not.
 * @param addr The I2C slave address.
 * @return 0 if successful, otherwise a negative error code.
 */
static int omap_i2c_xfer_common(const struct device *dev, struct i2c_msg msg[], int num,
				bool polling, uint16_t addr)
{
	LOG_INF("Starting I2C transfer common");
	struct i2c_omap_data *data = dev->data; 
	const struct i2c_omap_cfg *cfg = dev->config;
	int i, r = 0; 

	r = omap_i2c_wait_for_bb(dev); // Wait for bus busy condition to clear
	if (r < 0) {
		LOG_DBG("omap_i2c_wait_for_bb failed with error %d", r);
		goto out; // Exit if an error occurred during bus busy wait
	}
	if (data->set_mpu_wkup_lat != NULL) {
		LOG_DBG("Setting MPU wakeup latency to %d", data->latency);
		data->set_mpu_wkup_lat(cfg->init_func, data->latency);
	}

	// Loop through each message in the array and transfer it
	for (i = 0; i < num; i++) {
		LOG_DBG("Transferring message %d of %d", i + 1, num);
		r = omap_i2c_xfer_msg(dev, &msg[i], (i == (num - 1)), polling, addr);
		if (r != 0) {
			LOG_DBG("omap_i2c_xfer_msg failed with error %d", r);
			break; // Exit the loop if a message transfer fails
		}
	}

	if (r == 0) {
		LOG_DBG("I2C transfer completed successfully");
		r = num; 
		k_sem_give(&data->cmd_complete);
		return 0; // Return success if all message transfers are successful
	}

	omap_i2c_wait_for_bb(dev); // Wait for bus busy condition to clear again

	if (data->set_mpu_wkup_lat != NULL) {
		LOG_DBG("Setting MPU wakeup latency to -1");
		data->set_mpu_wkup_lat(cfg->init_func, -1);
	}
	LOG_DBG("Exiting at line %d", __LINE__);
out:
	// Error handling path
	// Mark the device as busy or suspend it based on implementation
	// Return the error code
	LOG_DBG("omap_i2c_xfer_common failed with error %d", r);
	k_sem_give(&data->cmd_complete);
	return r;
}

/**
 * @brief Transfer I2C messages using interrupt-driven mode.
 *
 * This function transfers I2C messages using interrupt-driven mode. It takes an array of I2C messages,
 * the number of messages in the array, and the address of the I2C device. It calls the omap_i2c_xfer_common()
 * function to perform the actual transfer.
 *
 * @param dev The I2C device.
 * @param msgs An array of I2C messages.
 * @param num_msgs The number of messages in the array.
 * @param addr The address of the I2C device.
 * @return 0 on success, negative errno code on failure.
 */
static int __maybe_unused omap_i2c_transfer_irq(const struct device *dev, struct i2c_msg msgs[], uint8_t num_msgs, uint16_t addr)
{
	LOG_INF("Transfering I2C messages using IRQ");
	return omap_i2c_xfer_common(dev, msgs, num_msgs, false, addr);
}

/**
 * @brief Transfer I2C messages using polling mode.
 *
 * This function transfers I2C messages using polling mode. It takes an array of
 * I2C messages, the number of messages in the array, and the address of the
 * I2C device. It logs a message indicating the transfer mode and calls the
 * omap_i2c_xfer_common() function to perform the actual transfer.
 *
 * @param dev The I2C device.
 * @param msgs An array of I2C messages.
 * @param num_msgs The number of messages in the array.
 * @param addr The address of the I2C device.
 * @return 0 on success, negative errno code on failure.
 */
static int omap_i2c_transfer_polling(const struct device *dev, struct i2c_msg msgs[], uint8_t num_msgs, uint16_t addr)
{
	LOG_INF("Transfering I2C messages using Polling");
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
