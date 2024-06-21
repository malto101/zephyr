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

struct omap_i2c_dev {
	const struct device *dev;
	mm_reg_t base; /* memory-mapped register base address */
	int irq;
	int reg_shift; /* bit shift for I2C register addresses */
	struct k_sem cmd_complete;
	const struct device *ioarea;
	uint32_t latency; /* maximum mpu wkup latency */
	void (*set_mpu_wkup_lat)(const struct device *dev, long latency);
	uint32_t speed; /* Speed of bus in kHz */
	uint32_t flags;
	uint16_t scheme;
	uint16_t cmd_err;
	uint8_t *buf;
	uint8_t *regs;
	size_t buf_len;
	struct i2c_driver_api *api;
	uint8_t threshold;
	uint8_t fifo_size; /* use as flag and value
			    * fifo_size==0 implies no fifo
			    * if set, should be trsh+1
			    */
	uint32_t rev;
	bool b_hw;        /* bad h/w fixes */
	bool bb_valid;    /* true when BB-bit reflects the I2C bus state */
	bool receiver;    /* true when we're in receiver mode */
	uint16_t iestate; /* Saved interrupt register */
	uint16_t pscstate;
	uint16_t scllstate;
	uint16_t sclhstate;
	uint16_t syscstate;
	uint16_t westate;
	uint16_t errata;
};

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

static inline void omap_i2c_write_reg(struct omap_i2c_dev *omap, int reg, u16 val)
{
	writew_relaxed(val, omap->base + (omap->regs[reg] << omap->reg_shift));
}
