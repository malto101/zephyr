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

/* MCSPI_MODULCTRL register bits */
#define OMAP_MCSPI_MODULCTRL_SINGLE BIT(0) /* Single or multi-channel mode*/
#define OMAP_MCSPI_MODULCTRL_MS     BIT(2) /* Enable slave mode*/
#define OMAP_MCSPI_MODULCTRL_STEST  BIT(3) /* Enable System test mode*/

struct spi_omap_config {
	DEVICE_MMIO_NAMED_ROM(base);
	struct spi_config spi_cfg;
	uint32_t irq;
	uint32_t frequency;
};

struct spi_omap_data {
	DEVICE_MMIO_NAMED_RAM(base);
	struct spi_context ctx;
}

static int
spi_omap_configure(const struct device *dev, const struct spi_config *config)
{
	const struct spi_omap_config *omap_cfg = DEV_CFG(dev);
	struct spi_omap_data *omap_data = DEV_DATA(dev);
	struct spi_context *ctx = &omap_data->ctx;
	struct spi_omap_reg_t *SPI_OMAP_REG = DEV_SPI_CFG_BASE(dev);

	if (spi_context_configured(ctx, config)) {
		return 0;
	}

	if (config->operation & SPI_HALF_DUPLEX) {
		LOG_ERR("Half-duplex not supported");
		return -ENOTSUP;
	}

	if (SPI_WORD_SIZE_GET(config->operation) != 8) {
		LOG_ERR("Word sizes other than 8 bits are not supported");
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

	SPI_OMAP_REG->MCSPI_WAKEUPENABLE |= 1;
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

static unsigned omap_mcspi_transceive(const struct device *dev, const struct spi_config *config,
				      const struct spi_buf_set *tx_bufs,
				      const struct spi_buf_set *rx_bufs)
{
	const struct spi_omap_config *omap_cfg = DEV_CFG(dev);
	struct spi_omap_data *omap_data = DEV_DATA(dev);
	struct spi_context *ctx = &omap_data->ctx;
	struct spi_omap_reg_t *SPI_OMAP_REG = DEV_SPI_CFG_BASE(dev);
	uint32_t tx_data, rx_data;
    int err;

	err = spi_omap_configure(dev, config);
	if (err) {
		goto out;
	}

	spi_context_buffers_setup(ctx, tx_bufs, rx_bufs, 1);
	if (spi_context_cs_control(ctx, true)) {
        goto out;
    }

	do {
		if (spi_context_tx_on(ctx)) {
			tx_data = *ctx->tx_buf;
		} else {
			tx_data = 0U;
		}
	}
}

static const struct spi_driver_api spi_omap_driver_api = {
	.transceive = spi_omap_transceive,
	.release = spi_omap_release,
};

#define SPI_OMAP_DEVICE_INIT(n)                                                                    \
	static void spi_omap_config_func_##n(const struct device *dev);                            \
                                                                                                   \
	static const struct spi_omap_config spi_omap_config_##n =                                  \
		{                                                                                  \
			DEVICE_MMIO_NAMED_INIT(base, DT_DRV_INST(n)),                              \
			.spi_cfg =                                                                 \
				{                                                                  \
					.frequency = DT_INST_PROP(n, spi_max_frequency),           \
					.operation = SPI_OP_MODE_MASTER | SPI_WORD_SET(8) |        \
						     SPI_TRANSFER_MSB,                             \
				},                                                                 \
	},                                                                                         \
                                                                                                   \
					    static struct spi_omap_data spi_omap_data_##n;         \
                                                                                                   \
	DEVICE_DT_DEFINE(DT_DRV_INST(n), spi_omap_config_func_##n, NULL, &spi_omap_data_##n,       \
			 &spi_omap_config_##n, POST_KERNEL, CONFIG_SPI_INIT_PRIORITY,              \
			 &spi_omap_driver_api);                                                    \
                                                                                                   \
	DT_INST_FOREACH_STATUS_OKAY(SPI_OMAP_DEVICE_INIT)