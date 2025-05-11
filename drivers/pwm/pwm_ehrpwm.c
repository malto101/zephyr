/*
 * Copyright (c) 2025 Dhruv Menon <dhruvmenon1104@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ti_ehrpwm_pwm

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(pwm_ehrpwm, CONFIG_PWM_LOG_LEVEL);

#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/pinctrl.h>


#define DEFAULT_CLK_RATE 100000000U /* 100 MHz,*/

/* EHRPWM register offsets */
#define TBCTL           0x00
#define TBPRD           0x0A
#define CMPA            0x12
#define CMPB            0x14
#define AQCTLA          0x16
#define AQCTLB          0x18
#define AQSFRC          0x1A
#define AQCSFRC         0x1C

/* TBCTL bits */
#define TBCTL_PRDLD_SHDW    0
#define TBCTL_PRDLD_IMDT    BIT(3)
#define TBCTL_CLKDIV_MASK   (BIT(12) | BIT(11) | BIT(10) | BIT(9) | BIT(8) | BIT(7))
#define TBCTL_CTRMODE_MASK  (BIT(1) | BIT(0))
#define TBCTL_CTRMODE_UP    0
#define TBCTL_HSPCLKDIV_SHIFT 7
#define TBCTL_CLKDIV_SHIFT  10
#define TBCTL_PRDLD_MASK    BIT(3)

/* AQCTL bits */
#define AQCTL_CBU_FRCLOW    BIT(8)
#define AQCTL_CBU_FRCHIGH   BIT(9)
#define AQCTL_CAU_FRCLOW    BIT(4)
#define AQCTL_CAU_FRCHIGH   BIT(5)
#define AQCTL_PRD_FRCLOW    BIT(2)
#define AQCTL_PRD_FRCHIGH   BIT(3)
#define AQCTL_ZRO_FRCLOW    BIT(0)
#define AQCTL_ZRO_FRCHIGH   BIT(1)

#define NUM_CHANNELS        2
#define PERIOD_MAX          0xFFFF
#define CLKDIV_MAX          7
#define HSPCLKDIV_MAX       7

#define DEV_CFG(dev) ((const struct pwm_ehrpwm_config *)(dev)->config)
#define DEV_DATA(dev) ((struct pwm_ehrpwm_data *)(dev)->data)
#define EHRPWM_BASE(dev) (DEVICE_MMIO_GET(dev))
struct pwm_ehrpwm_config {
    DEVICE_MMIO_ROM;
	const struct pinctrl_dev_config *pcfg;

};

struct pwm_ehrpwm_data {
    DEVICE_MMIO_RAM;
    uint32_t period_cycles[NUM_CHANNELS];
    pwm_flags_t polarity[NUM_CHANNELS];
    uint32_t clk_rate;
};

static inline uint16_t ehrpwm_read(const struct device *dev, uint32_t offset)
{
    uint16_t val = sys_read16(EHRPWM_BASE(dev) + offset);
    LOG_DBG("ehrpwm_read: base=0x%08lx offset=0x%02x val=0x%04x", (unsigned long)EHRPWM_BASE(dev), offset, val);
    return val;
}

static inline void ehrpwm_write(const struct device *dev, uint32_t offset, uint16_t value)
{
    LOG_DBG("ehrpwm_write: base=0x%08lx offset=0x%02x value=0x%04x", (unsigned long)EHRPWM_BASE(dev), offset, value);
    sys_write16(value, EHRPWM_BASE(dev) + offset);
}

static inline void ehrpwm_modify(const struct device *dev, uint32_t offset, 
                                uint16_t mask, uint16_t value)
{
    uint16_t val = ehrpwm_read(dev, offset);
    LOG_DBG("ehrpwm_modify: before val=0x%04x mask=0x%04x value=0x%04x", val, mask, value);
    val &= ~mask;
    val |= (value & mask);
    LOG_DBG("ehrpwm_modify: after val=0x%04x", val);
    ehrpwm_write(dev, offset, val);
}

static int set_prescale_div(uint32_t period_cycles, uint16_t *prescale_div,
                           uint16_t *tb_clk_div)
{
    LOG_DBG("set_prescale_div: period_cycles=%u", period_cycles);
    for (uint32_t clkdiv = 0; clkdiv <= CLKDIV_MAX; clkdiv++) {
        for (uint32_t hspclkdiv = 0; hspclkdiv <= HSPCLKDIV_MAX; hspclkdiv++) {
            *prescale_div = (1 << clkdiv) * (hspclkdiv ? (hspclkdiv * 2) : 1);
            LOG_DBG("  clkdiv=%u hspclkdiv=%u prescale_div=%u", clkdiv, hspclkdiv, *prescale_div);
            if (period_cycles / *prescale_div <= PERIOD_MAX) {
                *tb_clk_div = (clkdiv << TBCTL_CLKDIV_SHIFT) |
                            (hspclkdiv << TBCTL_HSPCLKDIV_SHIFT);
                LOG_DBG("  selected tb_clk_div=0x%04x", *tb_clk_div);
                return 0;
            }
        }
    }
    LOG_ERR("set_prescale_div: No valid prescaler found");
    return -EINVAL;
}

static void configure_polarity(const struct device *dev, uint32_t channel)
{
    struct pwm_ehrpwm_data *data = DEV_DATA(dev);
    uint16_t aqctl_val;
    uint32_t aqctl_reg;
    LOG_DBG("configure_polarity: channel=%u polarity=0x%08x", channel, data->polarity[channel]);
    if (channel == 1) {
        aqctl_reg = AQCTLB;
        aqctl_val = (data->polarity[channel] & PWM_POLARITY_INVERTED) ?
                    (AQCTL_CBU_FRCHIGH | AQCTL_PRD_FRCLOW | AQCTL_ZRO_FRCLOW) :
                    (AQCTL_CBU_FRCLOW | AQCTL_PRD_FRCHIGH | AQCTL_ZRO_FRCHIGH);
    } else {
        aqctl_reg = AQCTLA;
        aqctl_val = (data->polarity[channel] & PWM_POLARITY_INVERTED) ?
                    (AQCTL_CAU_FRCHIGH | AQCTL_PRD_FRCLOW | AQCTL_ZRO_FRCLOW) :
                    (AQCTL_CAU_FRCLOW | AQCTL_PRD_FRCHIGH | AQCTL_ZRO_FRCHIGH);
    }
    LOG_DBG("configure_polarity: aqctl_reg=0x%02x aqctl_val=0x%04x", aqctl_reg, aqctl_val);
    ehrpwm_write(dev, aqctl_reg, aqctl_val);
}

static int pwm_ehrpwm_set_cycles(const struct device *dev, uint32_t channel,
                                uint32_t period_cycles, uint32_t pulse_cycles,
                                pwm_flags_t flags)
{
    LOG_DBG("pwm_ehrpwm_set_cycles: channel=%u period_cycles=%u pulse_cycles=%u flags=0x%08x", channel, period_cycles, pulse_cycles, flags);
    if (channel >= NUM_CHANNELS) {
        LOG_ERR("Invalid channel: %u", channel);
        return -EINVAL;
    }

    struct pwm_ehrpwm_data *data = DEV_DATA(dev);
    uint16_t prescale_div, tb_clk_div;

    /* Check if period conflicts with other channels */
    for (uint32_t i = 0; i < NUM_CHANNELS; i++) {
        if (i != channel && data->period_cycles[i] && 
            data->period_cycles[i] != period_cycles) {
            LOG_ERR("Period conflict with channel %u", i);
            return -EINVAL;
        }
    }

    /* Calculate prescaler */
    if (set_prescale_div(period_cycles, &prescale_div, &tb_clk_div)) {
        LOG_ERR("Unsupported period value");
        return -EINVAL;
    }

    /* Store configuration */
    data->period_cycles[channel] = period_cycles;
    data->polarity[channel] = flags;

    /* Configure hardware */
    LOG_DBG("Configuring hardware: tb_clk_div=0x%04x prescale_div=%u", tb_clk_div, prescale_div);
    ehrpwm_modify(dev, TBCTL, TBCTL_CLKDIV_MASK, tb_clk_div);
    ehrpwm_modify(dev, TBCTL, TBCTL_PRDLD_MASK, TBCTL_PRDLD_SHDW);
    ehrpwm_write(dev, TBPRD, period_cycles / prescale_div);
    ehrpwm_modify(dev, TBCTL, TBCTL_CTRMODE_MASK, TBCTL_CTRMODE_UP);

    /* Set duty cycle */
    uint16_t duty = pulse_cycles / prescale_div;
    LOG_DBG("Setting duty cycle: duty=0x%04x", duty);
    ehrpwm_write(dev, (channel == 1) ? CMPB : CMPA, duty);

    /* Configure polarity */
    configure_polarity(dev, channel);

    return 0;
}

static int pwm_ehrpwm_get_cycles_per_sec(const struct device *dev, 
                                        uint32_t channel, uint64_t *cycles)
{
    LOG_DBG("pwm_ehrpwm_get_cycles_per_sec: channel=%u", channel);
    if (channel >= NUM_CHANNELS) {
        LOG_ERR("Invalid channel: %u", channel);
        return -EINVAL;
    }

    struct pwm_ehrpwm_data *data = DEV_DATA(dev);
    *cycles = data->clk_rate;
    LOG_DBG("clk_rate=%u", data->clk_rate);
    return 0;
}

static int pwm_ehrpwm_init(const struct device *dev)
{
    printf("pwm_ehrpwm_init called\n");
    struct pwm_ehrpwm_data *data = DEV_DATA(dev);
    const struct pwm_ehrpwm_config *cfg = DEV_CFG(dev);
    LOG_DBG("pwm_ehrpwm_init: base=0x%08lx", (unsigned long)EHRPWM_BASE(dev));
    /* ideally Get clock rate, but as clock subsystem isnt ready yet,
     we will use a default value */
    data->clk_rate = DEFAULT_CLK_RATE;

    int ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		LOG_ERR("failed to apply pinctrl");
		return ret;
	}

    LOG_DBG("pwm_ehrpwm_init: done");
    return 0;
}

static DEVICE_API(pwm, pwm_ehrpwm_driver_api) = {
    .set_cycles = pwm_ehrpwm_set_cycles,
    .get_cycles_per_sec = pwm_ehrpwm_get_cycles_per_sec,
};

#define PWM_EHRPWM_INIT(n) \
	PINCTRL_DT_INST_DEFINE(n);\
    static const struct pwm_ehrpwm_config pwm_ehrpwm_config_##n = { \
		DEVICE_MMIO_ROM_INIT( DT_DRV_INST(n)),\
        .pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n)\
    }; \
    static struct pwm_ehrpwm_data pwm_ehrpwm_data_##n; \
    DEVICE_DT_INST_DEFINE(n, \
            pwm_ehrpwm_init, \
            NULL, \
            &pwm_ehrpwm_data_##n, \
            &pwm_ehrpwm_config_##n, \
            POST_KERNEL, \
            CONFIG_PWM_INIT_PRIORITY, \
            &pwm_ehrpwm_driver_api);

DT_INST_FOREACH_STATUS_OKAY(PWM_EHRPWM_INIT)
