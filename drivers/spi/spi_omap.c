#define DT_DRV_COMPAT ti_omap_spi

#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
#include <zephyr/drivers/spi.h>

LOG_MODULE_REGISTER(spi_omap, CONFIG_SPI_LOG_LEVEL);

#include "spi_context.h"

/* SPI register offsets */

typedef struct {
    __IO uint32_t MCSPI_HL_REV;      /* 0x00 */
    __IO uint32_t MCSPI_HL_HWINFO;   /* 0x04 */
    uint8_t RESERVED_0[0x2];       /* 0x08 - 0x09 */
    __IO uint32_t MCSPI_HL_SYSCONFIG;    /* 0x10 */
    uuint8_t RESERVED_1[0xEC];      /* 0x14 - 0x99 */
    __IO uint32_t MCSPI_REVISION;    /* 0x100 */
    uint8_t RESERVED_2[0x6];    /* 0x104 - 0x109 */
    __IO uint32_t MCSPI_SYSCONFIG;   /* 0x110 */
    __IO uint32_t MCSPI_SYSSTATUS;   /* 0x114 */
    __IO uint32_t MCSPI_IRQSTATUS;   /* 0x118 */
    __IO uint32_t MCSPI_IRQENABLE;   /* 0x11C */
    __IO uint32_t MCSPI_WAKEUPENABLE;    /* 0x120 */
    __IO uint32_t MCSPI_SYST;        /* 0x124 */
    __IO uint32_t MCSPI_MODULCTRL;   /* 0x128 */
    __IO uint32_t MCSPI_CH0CONF;     /* 0x12C */
    __IO uint32_t MCSPI_CH0STAT;     /* 0x130 */
    __IO uint32_t MCSPI_CH0CTRL;     /* 0x134 */
    __IO uint32_t MCSPI_TX0;         /* 0x138 */
    __IO uint32_t MCSPI_RX0;         /* 0x13C */
    __IO uint32_t MCSPI_CH1CONF;     /* 0x140 */
    __IO uint32_t MCSPI_CH1STAT;     /* 0x144 */
    __IO uint32_t MCSPI_CH1CTRL;     /* 0x148 */
    __IO uint32_t MCSPI_TX1;         /* 0x14C */
    __IO uint32_t MCSPI_RX1;         /* 0x150 */
    __IO uint32_t MCSPI_CH2CONF;     /* 0x154 */
    __IO uint32_t MCSPI_CH2STAT;     /* 0x158 */
    __IO uint32_t MCSPI_CH2CTRL;     /* 0x15C */
    __IO uint32_t MCSPI_TX2;         /* 0x160 */
    __IO uint32_t MCSPI_RX2;         /* 0x164 */
    __IO uint32_t MCSPI_CH3CONF;     /* 0x168 */
    __IO uint32_t MCSPI_CH3STAT;     /* 0x16C */
    __IO uint32_t MCSPI_CH3CTRL;     /* 0x170 */
    __IO uint32_t MCSPI_TX3;         /* 0x174 */
    __IO uint32_t MCSPI_RX3;         /* 0x178 */
    __IO uint32_t MCSPI_XFERLEVEL;   /* 0x17C */
    __IO uint32_t MCSPI_DAFTX;       /* 0x180 */
    uint8_t RESERVED_3[0x1C];   /* 0x184 - 0x19F */
    __IO uint32_t MCSPI_DAFRX;       /* 0x1A0 */
} SPI_OMAP_REG, *SPI_OMAP_REG_PTR;

struct spi_omap_config {
    DEVICE_MMIO_NAMED_ROM(base);
    
};