#define DT_DRV_COMPAT ti_omap_i2c

#include <zephyr/errno.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
#include <zephyr/drivers/i2c.h>

LOG_MODULE_REGISTER(i2c_omap);

struct omap_i2c_device {
	struct device *dev;
	mm_reg_t base;
	int irq;
	int reg_shift;
	uint16_t iestate;
	uint16_t scllstate;
	uint16_t sclhstate;
	uint16_t pscstate;
	uint32_t rev;
	uint8_t *buf;
	uint8_t *regs;
	size_t buf_len;
	uint32_t flags;
	uint16_t westate;
}

#define OMAP_I2C_FLAG_SIMPLE_CLOCK        BIT(1)
#define OMAP_I2C_FLAG_FORCE_19200_INT_CLK BIT(6)

/* I2C Interrupt Enable Register (OMAP_I2C_IE): */
#define OMAP_I2C_IE_XDR  (1 << 14) /* TX Buffer drain int enable */
#define OMAP_I2C_IE_RDR  (1 << 13) /* RX Buffer drain int enable */
#define OMAP_I2C_IE_XRDY (1 << 4)  /* TX data ready int enable */
#define OMAP_I2C_IE_RRDY (1 << 3)  /* RX data ready int enable */
#define OMAP_I2C_IE_ARDY (1 << 2)  /* Access ready int enable */
#define OMAP_I2C_IE_NACK (1 << 1)  /* No ack interrupt enable */
#define OMAP_I2C_IE_AL   (1 << 0)  /* Arbitration lost int ena */

/* I2C Status Register (OMAP_I2C_STAT): */
#define OMAP_I2C_STAT_XDR  (1 << 14) /* TX Buffer draining */
#define OMAP_I2C_STAT_RDR  (1 << 13) /* RX Buffer draining */
#define OMAP_I2C_STAT_BB   (1 << 12) /* Bus busy */
#define OMAP_I2C_STAT_ROVR (1 << 11) /* Receive overrun */
#define OMAP_I2C_STAT_XUDF (1 << 10) /* Transmit underflow */
#define OMAP_I2C_STAT_AAS  (1 << 9)  /* Address as slave */
#define OMAP_I2C_STAT_BF   (1 << 8)  /* Bus Free */
#define OMAP_I2C_STAT_XRDY (1 << 4)  /* Transmit data ready */
#define OMAP_I2C_STAT_RRDY (1 << 3)  /* Receive data ready */
#define OMAP_I2C_STAT_ARDY (1 << 2)  /* Register access ready */
#define OMAP_I2C_STAT_NACK (1 << 1)  /* No ack interrupt enable */
#define OMAP_I2C_STAT_AL   (1 << 0)  /* Arbitration lost int ena */

/* I2C WE wakeup enable register */
#define OMAP_I2C_WE_XDR_WE  (1 << 14) /* TX drain wakup */
#define OMAP_I2C_WE_RDR_WE  (1 << 13) /* RX drain wakeup */
#define OMAP_I2C_WE_AAS_WE  (1 << 9)  /* Address as slave wakeup*/
#define OMAP_I2C_WE_BF_WE   (1 << 8)  /* Bus free wakeup */
#define OMAP_I2C_WE_STC_WE  (1 << 6)  /* Start condition wakeup */
#define OMAP_I2C_WE_GC_WE   (1 << 5)  /* General call wakeup */
#define OMAP_I2C_WE_DRDY_WE (1 << 3)  /* TX/RX data ready wakeup */
#define OMAP_I2C_WE_ARDY_WE (1 << 2)  /* Reg access ready wakeup */
#define OMAP_I2C_WE_NACK_WE (1 << 1)  /* No acknowledgment wakeup */
#define OMAP_I2C_WE_AL_WE   (1 << 0)  /* Arbitration lost wakeup */

#define OMAP_I2C_WE_ALL
(OMAP_I2C_WE_XDR_WE | OMAP_I2C_WE_RDR_WE | OMAP_I2C_WE_AAS_WE | OMAP_I2C_WE_BF_WE |
 OMAP_I2C_WE_STC_WE | OMAP_I2C_WE_GC_WE | OMAP_I2C_WE_DRDY_WE | OMAP_I2C_WE_ARDY_WE |
 OMAP_I2C_WE_NACK_WE | OMAP_I2C_WE_AL_WE)

/* I2C Configuration Register (OMAP_I2C_CON): */
#define OMAP_I2C_CON_EN        (1 << 15) /* I2C module enable */
#define OMAP_I2C_CON_BE        (1 << 14) /* Big endian mode */
#define OMAP_I2C_CON_OPMODE_HS (1 << 12) /* High Speed support */
#define OMAP_I2C_CON_STB       (1 << 11) /* Start byte mode (master) */
#define OMAP_I2C_CON_MST       (1 << 10) /* Master/slave mode */
#define OMAP_I2C_CON_TRX       (1 << 9)  /* TX/RX mode (master only) */
#define OMAP_I2C_CON_XA        (1 << 8)  /* Expand address */
#define OMAP_I2C_CON_RM        (1 << 2)  /* Repeat mode (master only) */
#define OMAP_I2C_CON_STP       (1 << 1)  /* Stop cond (master only) */
#define OMAP_I2C_CON_STT       (1 << 0)  /* Start condition (master) */

/* I2C SCL time value when Master */
#define OMAP_I2C_SCLL_HSSCLL 8
#define OMAP_I2C_SCLH_HSSCLH 8

/* I2C System Test Register (OMAP_I2C_SYSTEST): */
#define OMAP_I2C_SYSTEST_ST_EN       (1 << 15) /* System test enable */
#define OMAP_I2C_SYSTEST_FREE        (1 << 14) /* Free running mode */
#define OMAP_I2C_SYSTEST_TMODE_MASK  (3 << 12) /* Test mode select */
#define OMAP_I2C_SYSTEST_TMODE_SHIFT (12)      /* Test mode select */
/* Functional mode */
#define OMAP_I2C_SYSTEST_SCL_I_FUNC  (1 << 8) /* SCL line input value */
#define OMAP_I2C_SYSTEST_SCL_O_FUNC  (1 << 7) /* SCL line output value */
#define OMAP_I2C_SYSTEST_SDA_I_FUNC  (1 << 6) /* SDA line input value */
#define OMAP_I2C_SYSTEST_SDA_O_FUNC  (1 << 5) /* SDA line output value */
/* SDA/SCL IO mode */
#define OMAP_I2C_SYSTEST_SCL_I       (1 << 3) /* SCL line sense in */
#define OMAP_I2C_SYSTEST_SCL_O       (1 << 2) /* SCL line drive out */
#define OMAP_I2C_SYSTEST_SDA_I       (1 << 1) /* SDA line sense in */
#define OMAP_I2C_SYSTEST_SDA_O       (1 << 0) /* SDA line drive out */

/* OCP_SYSSTATUS bit definitions */
#define SYSS_RESETDONE_MASK (1 << 0)

/* OCP_SYSCONFIG bit definitions */
#define SYSC_CLOCKACTIVITY_MASK (0x3 << 8)
#define SYSC_SIDLEMODE_MASK     (0x3 << 3)
#define SYSC_ENAWAKEUP_MASK     (1 << 2)
#define SYSC_SOFTRESET_MASK     (1 << 1)
#define SYSC_AUTOIDLE_MASK      (1 << 0)

#define SYSC_IDLEMODE_SMART     0x2
#define SYSC_CLOCKACTIVITY_FCLK 0x2

/* Errata definitions */
#define I2C_OMAP_ERRATA_I207 (1 << 0)
#define I2C_OMAP_ERRATA_I462 (1 << 1)

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
#define I2C_SCH             0xb8
#define I2C_SYSTEST         0xbc
#define I2C_BUFSTAT         0xc0
#define I2C_OA1             0xc4
#define I2C_OA2             0xc8
#define I2C_OA3             0xcc
#define I2C_ACTOA           0xd0
#define I2C_SBLOCK          0xd4

	LOG_MODULE_REGISTER(omap_i2c, CONFIG_I2C_LOG_LEVEL);

static inline void omap_i2c_write_reg(struct omap_i2c_dev *omap, int reg, u16 val)
{
	sys_write16(val, omap->base + (omap->regs[reg] << omap->reg_shift));
}

static void __omap_i2c_init(const struct device *dev)
{
	struct omap_i2c_dev *omap = dev->data;

	// Write to I2C_CON to reset I2C module
	omap_i2c_write_reg(omap, I2C_CON, 0);
	omap_i2c_write_reg(omap, I2C_PSC, omap->pscstate);

	/* SCL low and high time values */
	omap_i2c_write_reg(omap, I2C_SCLL, omap->scllstate);
	omap_i2c_write_reg(omap, I2C_SCLH, omap->sclhstate);
	// Take the I2C module out of reset
	omap_i2c_write_reg(omap, I2C_CON, OMAP_I2C_CON_EN);

	// Delay to allow the bus to settle
	k_busy_wait(1);

	// Conditionally write to IE register if iestate is set
	if (omap->iestate) {
		omap_i2c_write_reg(omap, I2C_IRQENABLE_SET, omap->iestate);
	}
}

static int omap_i2c_init(const struct device *dev)
{
	struct omap_i2c_dev *omap = dev->data;
	uint16_t psc = 0, scll = 0, sclh = 0;
	uint16_t fsscll = 0, fssclh = 0, hsscll = 0, hssclh = 0;
	unsigned long fclk_rate = 12000000;
	unsigned long internal_clk = 0;
	const struct device *clk_dev;
	int error;

	omap->westate = OMAP_I2C_WE_ALL;

	if (omap->flags & OMAP_I2C_FLAG_ALWAYS_ARMXOR_CLK) {
		clk_dev = device_get_binding("CLOCK_CONTROL");
		if (!clk_dev) {
			LOG_ERR("Failed to get clock device binding");
			return -ENODEV;
		}

		if (clock_control_get_rate(clk_dev, CLOCK_CONTROL_SUBSYS_BASE, &fclk_rate)) {
			LOG_ERR("Could not get fck rate");
			return -EIO;
		}

		if (fclk_rate > 12000000) {
			psc = fclk_rate / 12000000;
		}
	}

	if (!(omap->flags & OMAP_I2C_FLAG_SIMPLE_CLOCK)) {
		if (omap->speed > 400 || omap->flags & OMAP_I2C_FLAG_FORCE_19200_INT_CLK) {
			internal_clk = 19200;
		} else if (omap->speed > 100) {
			internal_clk = 9600;
		} else {
			internal_clk = 4000;
		}

		clk_dev = device_get_binding("CLOCK_CONTROL");
		if (!clk_dev) {
			LOG_ERR("Failed to get clock device binding");
			return -ENODEV;
		}

		if (clock_control_get_rate(clk_dev, CLOCK_CONTROL_SUBSYS_BASE, &fclk_rate)) {
			LOG_ERR("Could not get fck rate");
			return -EIO;
		}
		fclk_rate /= 1000;

		psc = fclk_rate / internal_clk;
		psc = psc - 1;

		if (omap->speed > 400) {
			unsigned long scl;

			scl = internal_clk / 400;
			fsscll = scl - (scl / 3) - 7;
			fssclh = (scl / 3) - 5;

			scl = fclk_rate / omap->speed;
			hsscll = scl - (scl / 3) - 7;
			hssclh = (scl / 3) - 5;
		} else if (omap->speed > 100) {
			unsigned long scl;

			scl = internal_clk / omap->speed;
			fsscll = scl - (scl / 3) - 7;
			fssclh = (scl / 3) - 5;
		} else {
			fsscll = internal_clk / (omap->speed * 2) - 7;
			fssclh = internal_clk / (omap->speed * 2) - 5;
		}
		scll = (hsscll << OMAP_I2C_SCLL_HSSCLL) | fsscll;
		sclh = (hssclh << OMAP_I2C_SCLH_HSSCLH) | fssclh;
	} else {
		fclk_rate /= (psc + 1) * 1000;
		if (psc > 2) {
			psc = 2;
		}
		scll = fclk_rate / (omap->speed * 2) - 7 + psc;
		sclh = fclk_rate / (omap->speed * 2) - 7 + psc;
	}

	omap->iestate = (OMAP_I2C_IE_XRDY | OMAP_I2C_IE_RRDY | OMAP_I2C_IE_ARDY | OMAP_I2C_IE_NACK |
			 OMAP_I2C_IE_AL) |
			((omap->fifo_size) ? (OMAP_I2C_IE_RDR | OMAP_I2C_IE_XDR) : 0);

	omap->pscstate = psc;
	omap->scllstate = scll;
	omap->sclhstate = sclh;

	__omap_i2c_init(omap);

	return 0;
}

static int omap_i2c_transmit_data(struct omap_i2c_dev *omap, uint8_t num_bytes, bool is_xdr)
{
	uint16_t w;

	while (num_bytes--) {
		w = *omap->buf++;
		omap->buf_len--;

		omap_i2c_write_reg(omap, OMAP_I2C_DATA_REG, w);
	}

	return 0;
}

static const struct i2c_driver_api omap_i2c_driver_api = {
	.init = omap_i2c_init,
};

#define OMAP_I2C_INIT(inst)
static struct omap_i2c_data omap_i2c_data_##inst;
static const struct omap_i2c_config omap_i2c_config_##inst = {
	.base = DT_INST_REG_ADDR(inst),
	.irq = DT_INST_IRQN(inst),
};
DEVICE_DT_INST_DEFINE(inst, &omap_i2c_init, NULL, &omap_i2c_data_##inst, &omap_i2c_config_##inst,
		      POST_KERNEL, CONFIG_I2C_INIT_PRIORITY, &omap_i2c_driver_api);

DT_INST_FOREACH_STATUS_OKAY(OMAP_I2C_INIT)