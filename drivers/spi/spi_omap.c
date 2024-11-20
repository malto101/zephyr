#define DT_DRV_COMPAT ti_omap_spi

#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
#include <zephyr/drivers/spi.h>
#include "spi_context.h"

LOG_MODULE_REGISTER(spi_omap, CONFIG_SPI_LOG_LEVEL);

#define DEV_CFG(dev)          ((const struct spi_omap_config *)((dev)->config))
#define DEV_DATA(dev)         ((struct spi_omap_data *)(dev)->data)
#define DEV_SPI_CFG_BASE(dev) ((struct spi_omap_reg_t *)DEVICE_MMIO_NAMED_GET(dev, port_base))

/* SPI trnasfer status */
#define MCSPI_TRANSFER_COMPLETED        (0U)
#define MCSPI_TRANSFER_STARTED          (1U)
#define MCSPI_TRANSFER_CANCELLED        (2U)
#define MCSPI_TRANSFER_FAILED           (3U)
#define MCSPI_TRANSFER_CSN_DEASSERT     (4U)
#define MCSPI_TRANSFER_TIMEOUT          (5U)

/* SPI register offsets */

typedef struct {
	__IO uint32_t MCSPI_HL_REV;       /* 0x00 */
	__IO uint32_t MCSPI_HL_HWINFO;    /* 0x04 */
	uint8_t RESERVED_0[0x2];          /* 0x08 - 0x09 */
	__IO uint32_t MCSPI_HL_SYSCONFIG; /* 0x10 */
	uuint8_t RESERVED_1[0xEC];        /* 0x14 - 0x99 */
	__IO uint32_t MCSPI_REVISION;     /* 0x100 */
	uint8_t RESERVED_2[0x6];          /* 0x104 - 0x109 */
	__IO uint32_t MCSPI_SYSCONFIG;    /* 0x110 */
	__IO uint32_t MCSPI_SYSSTATUS;    /* 0x114 */
	__IO uint32_t MCSPI_IRQSTATUS;    /* 0x118 */
	__IO uint32_t MCSPI_IRQENABLE;    /* 0x11C */
	__IO uint32_t MCSPI_WAKEUPENABLE; /* 0x120 */
	__IO uint32_t MCSPI_SYST;         /* 0x124 */
	__IO uint32_t MCSPI_MODULCTRL;    /* 0x128 */
	__IO uint32_t MCSPI_CH0CONF;      /* 0x12C */
	__IO uint32_t MCSPI_CH0STAT;      /* 0x130 */
	__IO uint32_t MCSPI_CH0CTRL;      /* 0x134 */
	__IO uint32_t MCSPI_TX0;          /* 0x138 */
	__IO uint32_t MCSPI_RX0;          /* 0x13C */
	__IO uint32_t MCSPI_CH1CONF;      /* 0x140 */
	__IO uint32_t MCSPI_CH1STAT;      /* 0x144 */
	__IO uint32_t MCSPI_CH1CTRL;      /* 0x148 */
	__IO uint32_t MCSPI_TX1;          /* 0x14C */
	__IO uint32_t MCSPI_RX1;          /* 0x150 */
	__IO uint32_t MCSPI_CH2CONF;      /* 0x154 */
	__IO uint32_t MCSPI_CH2STAT;      /* 0x158 */
	__IO uint32_t MCSPI_CH2CTRL;      /* 0x15C */
	__IO uint32_t MCSPI_TX2;          /* 0x160 */
	__IO uint32_t MCSPI_RX2;          /* 0x164 */
	__IO uint32_t MCSPI_CH3CONF;      /* 0x168 */
	__IO uint32_t MCSPI_CH3STAT;      /* 0x16C */
	__IO uint32_t MCSPI_CH3CTRL;      /* 0x170 */
	__IO uint32_t MCSPI_TX3;          /* 0x174 */
	__IO uint32_t MCSPI_RX3;          /* 0x178 */
	__IO uint32_t MCSPI_XFERLEVEL;    /* 0x17C */
	__IO uint32_t MCSPI_DAFTX;        /* 0x180 */
	uint8_t RESERVED_3[0x1C];         /* 0x184 - 0x19F */
	__IO uint32_t MCSPI_DAFRX;        /* 0x1A0 */
} spi_omap_reg_t;

/* MCSPI_CH(0/1/2/3)CONF register bits */
#define OMAP_MCSPI_CHXCONF_PHA                                                                     \
	BIT(0) /* Clock phase -> Data are latched on even-numbered edges of SPICLK*/
#define OMAP_MCSPI_CHXCONF_POL         BIT(1)       /* Clock polarity -> SPICLK is high when idle */
#define OMAP_MCSPI_CHXCONF_CLKD_MASK   (0x0f << 2)  /* Clock divider mask*/
#define OMAP_MCSPI_CHXCONF_EPOL        BIT(6)       /* Enable polarity -> Active low*/
#define OMAP_MCSPI_CHXCONF_WL_MASK     (0x1f << 7)  /* Word length mask*/
#define OMAP_MCSPI_CHXCONF_TRM_RX_ONLY BIT(12)      /* Transfer mode -> Receive only*/
#define OMAP_MCSPI_CHXCONF_TRM_TX_ONLY BIT(13)      /* Transfer mode -> Transmit only*/
#define OMAP_MCSPI_CHXCONF_TRM_MASK    (0x03 << 12) /* Transfer mode mask*/
#define OMAP_MCSPI_CHXCONF_DMAW        BIT(14)      /* DMA enable for transmit*/
#define OMAP_MCSPI_CHXCONF_DMAR        BIT(15)      /* DMA enable for receive*/
#define OMAP_MCSPI_CHXCONF_DPE0        BIT(16)      /* No Transmission for data line 0*/
#define OMAP_MCSPI_CHXCONF_DPE1        BIT(17)      /* No Transmission for data line 1*/
#define OMAP_MCSPI_CHXCONF_IS          BIT(18)      /* Input selection*/
#define OMAP_MCSPI_CHXCONF_TURBO       BIT(19)      /* Turbo mode Enable*/
#define OMAP_MCSPI_CHXCONF_FORCE       BIT(20)      /* Force mode*/
#define OMAP_MCSPI_CHXCONF_FFET        BIT(27)      /* FIFO enable for transmit*/
#define OMAP_MCSPI_CHXCONF_FFER        BIT(28)      /* FIFO enable for receive*/
#define OMAP_MCSPI_CHXCONF_CLKG        BIT(29) /* Clock divider granularity ->1 clock cycle granularity*/

/* MCSPI_CH(0/1/2/3)CTRL register bits */
#define OMAP_MCSPI_CHXCTRL_EN           BIT(0) /* Channel enable*/

/* MCSPI_MODULCTRL register bits */
#define OMAP_MCSPI_MODULCTRL_SINGLE BIT(0) /* Single or multi-channel mode*/
#define OMAP_MCSPI_MODULCTRL_MS     BIT(2) /* Enable slave mode*/
#define OMAP_MCSPI_MODULCTRL_STEST  BIT(3) /* Enable System test mode*/

/* MCSPI_CH(0/1/2/3)STAT register bits */
#define OMAP_MCSPI_CHSTAT_RXS		BIT(0)	/* RX shift*/
#define OMAP_MCSPI_CHSTAT_TXS		BIT(1)	/* TX shift*/
#define OMAP_MCSPI_CHSTAT_EOT		BIT(2)	/* End of transfer*/
#define OMAP_MCSPI_CHSTAT_TXFFE		BIT(3)	/* TX FIFO empty*/

struct spi_cs_omap_config {
	uint8_t chip_select;
	uint32_t frequency;
	uint32_t cs_delay;
};

struct spi_omap_config {
	DEVICE_MMIO_NAMED_ROM(base);
	uint32_t irq;
	struct spi_cs_omap_config cs_config[4];
};

struct spi_cs_omap_data (
	uint32_t chconf, chctrl;
);

struct spi_omap_data {
	DEVICE_MMIO_NAMED_RAM(base);
	struct spi_context ctx;
	uint8_t xfer_status;
	struct spi_cs_omap_data cs_data;
};

static int
omap_spi_configure(const struct device *dev, const struct spi_config *config)
{
	const struct spi_omap_config *omap_cfg = DEV_CFG(dev);
	struct spi_omap_data *omap_data = DEV_DATA(dev);
	struct spi_context *ctx = &omap_data->ctx;
	struct spi_omap_reg_t *SPI_OMAP_REG = DEV_SPI_CFG_BASE(dev);
	uint32_t word_size = SPI_WORD_SIZE_GET(config->operation);

	if (spi_context_configured(ctx, config)) {
		return 0;
	}

	if (config->operation & SPI_HALF_DUPLEX) {
		LOG_ERR("Half-duplex not supported");
		return -ENOTSUP;
	}

	if (word_size != 8 && word_size != 16 && word_size != 32) {
		LOG_ERR("Only 8, 16, and 32-bit word sizes are supported");
		return -ENOTSUP;
	}

	if (config->operation & SPI_TRANSFER_LSB) {
		LOG_ERR("Transfer LSB first mode is not supported");
		return -EINVAL;
	}

	if (config->operation & SPI_MODE_CPOL) {
		SPI_OMAP_REG->MCSPI_CH0CONF |= OMAP_MCSPI_CHXCONF_POL
	}

	if (config->operation & SPI_MODE_CPHA) {
		SPI_OMAP_REG->MCSPI_CH0CONF |= OMAP_MCSPI_CHXCONF_PHA
	}

	ctx->config = config;

	return 0;
}

static int omap_mcspi_controller_init(const struct device *dev)
{
	const struct spi_omap_config *omap_cfg = DEV_CFG(dev);
	struct spi_omap_data *omap_data = DEV_DATA(dev);
	struct spi_context *ctx = &omap_data->ctx;
	struct spi_omap_reg_t *SPI_OMAP_REG = DEV_SPI_CFG_BASE(dev);
	uint32_t reg_val;

	SPI_OMAP_REG->MCSPI_WAKEUPENABLE |= 0;
	/*Choose between host and target*/
	reg_val = SPI_OMAP_REG->MCSPI_MODULCTRL;
	reg_val &= ~OMAP_MCSPI_MODULCTRL_STEST;

	if (SPI_OP_MODE_GET(config->operation) != SPI_OP_MODE_MASTER) {
		LOG_ERR("Slave mode is not supported");
		return -ENOTSUP;
	} else {
		SPI_OMAP_REG->MCSPI_MODULCTRL &= ~OMAP_MCSPI_MODULCTRL_MS;
	}

	if (IS_ENABLED(CONFIG_SPI_EXTENDED_MODES) &&
	    (config->operation & SPI_LINES_MASK) != SPI_LINES_SINGLE) {
		LOG_ERR("Multiple lines are not supported");
		return -EINVAL;
	} else {
		SPI_OMAP_REG->MCSPI_CH0CONF |= OMAP_MCSPI_MODULCTRL_SINGLE;
	}

	return 0;
}

static void omap_mcspi_set_cs_enable(const struct device *dev, int ChannelNum, bool enable)
{
	struct spi_omap_reg_t *SPI_OMAP_REG = DEV_SPI_CFG_BASE(dev);
	struct spi_omap_data *data = DEV_DATA(dev);

	volatile uint32_t *ctrl_reg = &SPI_OMAP_REG->MCSPI_CH0CTRL + (ChannelNum << 2);

	if (enable) {
        *ctrl_reg |= OMAP_MCSPI_CHXCTRL_EN;  // Enable the channel
	} else {
        *ctrl_reg &= ~OMAP_MCSPI_CHXCTRL_EN; // Disable the channel
	}
	SPI_OMAP_REG->MCSPI_CH0CTRL = *ctrl_reg;
	data->cs_data.chctrl = SPI_OMAP_REG->MCSPI_CH0CTRL;
}

static uint32_t omap_mcspi_chcfg_read(const struct device *dev, int ChannelNum)
{
	struct spi_omap_reg_t *SPI_OMAP_REG = DEV_SPI_CFG_BASE(dev);
	struct spi_omap_data *data = DEV_DATA(dev);

	volatile uint32_t *conf_reg = &SPI_OMAP_REG->MCSPI_CH0CONF + (ChannelNum << 2);
	data->cs_data.chconf = *conf_reg;
	return data->cs_data.chconf;
}

static void omap_mcspi_chcfg_write(const struct device *dev, int ChannelNum, uint32_t val)
{
	struct spi_omap_reg_t *SPI_OMAP_REG = DEV_SPI_CFG_BASE(dev);
	struct spi_omap_data *data = DEV_DATA(dev);

	volatile uint32_t *conf_reg = &SPI_OMAP_REG->MCSPI_CH0CONF + (ChannelNum << 2);
	*conf_reg = val;
	data->cs_data.chconf = *conf_reg;
}
	

static int poll_reg(volatile uint32_t *reg, unsigned long bit )
{
	uint32_t timeout_ms = 1000;
    uint64_t timeout = k_uptime_get() + timeout_ms;

    while (!(*reg & bit)) {
        if (k_uptime_get() > timeout) {
            if (!(*reg & bit)) {
                return -ETIMEDOUT;
            } else {
                return 0;
            }
        }
        k_busy_wait(1); // Small delay to prevent tight busy-wait loops
    }
    return 0;
}

static int omap_mcspi_txrx_pio(const struct device *dev, const struct spi_config *config,
                               const struct spi_buf_set *tx_bufs,
                               const struct spi_buf_set *rx_bufs)
{
    const struct spi_omap_config *cfg = DEV_CFG(dev);
    struct spi_omap_data *data = DEV_DATA(dev);
    struct spi_context *ctx = &data->ctx;
    struct spi_omap_reg_t *SPI_OMAP_REG = DEV_SPI_CFG_BASE(dev);
    int count = tx_bufs->count;
    int word_len = SPI_WORD_SIZE_GET(config->operation);
    int ChannelNum = config->slave;

    // Calculate pointers for the selected channel
	volatile uint32_t *conf_reg = &SPI_OMAP_REG->MCSPI_CH0CONF + (ChannelNum << 2);
	volatile uint32_t *stat_reg = &SPI_OMAP_REG->MCSPI_CH0STAT + (ChannelNum << 2);
	volatile uint32_t *ctrl_reg = &SPI_OMAP_REG->MCSPI_CH0CTRL + (ChannelNum << 2);
	volatile uint32_t *tx_reg = &SPI_OMAP_REG->MCSPI_TX0 + (ChannelNum << 2);
	volatile uint32_t *rx_reg = &SPI_OMAP_REG->MCSPI_RX0 + (ChannelNum << 2);

    do {
        count -= word_len / 8;
        if (tx_bufs) {
            // Wait for TXS (Transmit buffer ready)
            if (poll_reg(stat_reg, OMAP_MCSPI_CHSTAT_TXS) < 0) {
                LOG_ERR("TXS timeout\n");
                goto out;
            }
            // Write data to TX register
            *tx_reg = *(uint32_t *)tx_bufs->buffers->buf++;
        }
        if (rx_bufs) {
            // Wait for RXS (Receive buffer ready)
            if (poll_reg(stat_reg, OMAP_MCSPI_CHSTAT_RXS) < 0) {
                LOG_ERR("RXS timeout\n");
                goto out;
            }
            if (count == (word_len / 8) && tx_bufs == NULL && (*conf_reg & OMAP_MCSPI_CHXCONF_TURBO)) {
                omap_mcspi_set_cs_enable(dev, config->slave, false);
                *(uint32_t *)rx_bufs->buffers->buf++ = *rx_reg;

                // Wait for EOT (End of Transfer) if in Turbo mode
                if (poll_reg(stat_reg, OMAP_MCSPI_CHSTAT_EOT) < 0) {
                    LOG_ERR("EOT timeout\n");
                    goto out;
                }
                count = 0;
            } else if (count == 0 && tx_bufs == NULL) {
                omap_mcspi_set_cs_enable(dev, config->slave, false);
            }
            *(uint32_t *)rx_bufs->buffers->buf++ = *rx_reg;
        }
        k_busy_wait(ctx->config->cs.delay);
    } while (count >= (word_len / 8));

    if (rx_bufs == NULL) {
        // Final checks if only TX was performed
        if (poll_reg(stat_reg, OMAP_MCSPI_CHSTAT_TXS) < 0) {
            LOG_ERR("TXS timeout\n");
        } else if (poll_reg(stat_reg, OMAP_MCSPI_CHSTAT_EOT) < 0) {
            LOG_ERR("EOT timeout\n");
        }
        omap_mcspi_set_cs_enable(dev, config->slave, false);
    }

out:
    omap_mcspi_set_cs_enable(dev, config->slave, true);
    return tx_bufs->count - count;
}

static int omap_mcspi_transceive(const struct device *dev, const struct spi_config *config,
				      const struct spi_buf_set *tx_bufs,
				      const struct spi_buf_set *rx_bufs)
{
	const struct spi_omap_config *cfg = DEV_CFG(dev);
	struct spi_omap_data *data = DEV_DATA(dev);
	struct spi_context *ctx = &data->ctx;
	struct spi_omap_reg_t *SPI_OMAP_REG = DEV_SPI_CFG_BASE(dev);
	uint32_t tx_data, rx_data;
    int err;

	spi_context_lock(ctx, false, NULL, NULL, config);
	err = omap_spi_configure(dev, config);
	if (err) {
		goto out;
	}	
	omap_mcspi_set_cs_enable(dev, config->slave, false);
	cfg->cs_configs[config->slave].frequency = config->frequency;
	cfg->cs_configs[config->slave].mode = SPI_MODE_GET(config->operation);
	cfg->cs_configs[config->slave].word_size = SPI_WORD_SIZE_GET(config->operation);
	data->cs_data.chconf = omap_mcspi_chcfg_read(dev, config->slave);
	data->cs_data.chconf &= ~OMAP_MCSPI_CHXCONF_TRM_MASK;
	data->cs_data.chconf &= ~OMAP_MCSPI_CHXCONF_TURBO;

	if (tx_bufs && (rx_bufs->count == 0) ) {
		data->cs_data.chconf |= OMAP_MCSPI_CHXCONF_TRM_TX_ONLY;
	} else if (rx_bufs && (tx_bufs->count == 0)) {
		data->cs_data.chconf |= OMAP_MCSPI_CHXCONF_TRM_RX_ONLY;
	}
	omap_mcspi_chcfg_write(dev, config->slave, data->cs_data.chconf);

	if (SPI_WORD_SIZE_GET(config->operation)) {
		unsigned count;
		omap_mcspi_set_cs_enable(dev, config->slave, true);
		/* RX_ONLY mode needs dummy data in TX reg*/
		if (tx_bufs == NULL) {
			SPI_OMAP_REG->MCSPI_TX0 = 0x00;
		} else {
			count = omap_mcspi_txrx_pio(dev, config, tx_bufs, rx_bufs);
		}
		if (count == 0) {
			err = -EIO;
			goto out;
		}
	}
	
out:
    spi_context_release(ctx, err);
	omap_mcspi_set_cs_enable(dev, config->slave, false);
    return err;
}

static int omap_mcspi_release(const struct device *dev, const struct spi_config *config)
{
	struct spi_omap_reg_t *SPI_OMAP_REG = DEV_SPI_CFG_BASE(dev);
	omap_mcspi_set_cs_enable(dev, config->slave, false);
	spi_context_unlock_unconditionally(&data->ctx);
	return 0;
}
static const struct spi_driver_api spi_omap_driver_api = {
	.transceive = omap_mcspi_transceive,
	.release = omap_mcspi_release,
};

#define SPI_OMAP_DEVICE_INIT(n)                                                                    \
	static void spi_omap_config_func_##n(const struct device *dev);                            \
                                                                                                   \
	static const struct spi_omap_config spi_omap_config_##n =                                  \
		{                                                                                  \
			DEVICE_MMIO_NAMED_INIT(base, DT_DRV_INST(n)),                              \
			.irq = DT_INST_IRQN(n),                                                    \
		},                                                                                         \
                                                                                                   \
					    static struct spi_omap_data spi_omap_data_##n;         \
                                                                                                   \
	DEVICE_DT_DEFINE(DT_DRV_INST(n), spi_omap_config_func_##n, NULL, &spi_omap_data_##n,       \
			 &spi_omap_config_##n, POST_KERNEL, CONFIG_SPI_INIT_PRIORITY,              \
			 &spi_omap_driver_api);                                                    \
                                                                                                   \
	DT_INST_FOREACH_STATUS_OKAY(SPI_OMAP_DEVICE_INIT)