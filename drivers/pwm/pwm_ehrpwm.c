/*
 * Copyright (c) 2025 Dhruv Menon <dhruvmenon1104@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ti_ehrpwm_pwm

#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(pwm_ehrpwm, CONFIG_PWM_LOG_LEVEL);

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

struct pwm_ehrpwm_config {
    mm_reg_t base;
    const struct device *clock_dev;
    clock_control_subsys_t clock_subsys;
};

struct pwm_ehrpwm_data {
    uint32_t period_cycles[NUM_CHANNELS];
    pwm_flags_t polarity[NUM_CHANNELS];
    uint32_t clk_rate;
};

static inline uint16_t ehrpwm_read(const struct device *dev, uint32_t offset)
{
    const struct pwm_ehrpwm_config *config = dev->config;
    return sys_read16(config->base + offset);
}

static inline void ehrpwm_write(const struct device *dev, uint32_t offset, uint16_t value)
{
    const struct pwm_ehrpwm_config *config = dev->config;
    sys_write16(value, config->base + offset);
}

static inline void ehrpwm_modify(const struct device *dev, uint32_t offset, 
                                uint16_t mask, uint16_t value)
{
    uint16_t val = ehrpwm_read(dev, offset);
    val &= ~mask;
    val |= (value & mask);
    ehrpwm_write(dev, offset, val);
}

static int set_prescale_div(uint32_t period_cycles, uint16_t *prescale_div,
                           uint16_t *tb_clk_div)
{
    for (uint32_t clkdiv = 0; clkdiv <= CLKDIV_MAX; clkdiv++) {
        for (uint32_t hspclkdiv = 0; hspclkdiv <= HSPCLKDIV_MAX; hspclkdiv++) {
            *prescale_div = (1 << clkdiv) * (hspclkdiv ? (hspclkdiv * 2) : 1);
            if (period_cycles / *prescale_div <= PERIOD_MAX) {
                *tb_clk_div = (clkdiv << TBCTL_CLKDIV_SHIFT) |
                            (hspclkdiv << TBCTL_HSPCLKDIV_SHIFT);
                return 0;
            }
        }
    }
    return -EINVAL;
}

static void configure_polarity(const struct device *dev, uint32_t channel)
{
    struct pwm_ehrpwm_data *data = dev->data;
    uint16_t aqctl_val;
    uint32_t aqctl_reg;

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

    ehrpwm_write(dev, aqctl_reg, aqctl_val);
}

static int pwm_ehrpwm_set_cycles(const struct device *dev, uint32_t channel,
                                uint32_t period_cycles, uint32_t pulse_cycles,
                                pwm_flags_t flags)
{
    if (channel >= NUM_CHANNELS) {
        return -EINVAL;
    }

    struct pwm_ehrpwm_data *data = dev->data;
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
    ehrpwm_modify(dev, TBCTL, TBCTL_CLKDIV_MASK, tb_clk_div);
    ehrpwm_modify(dev, TBCTL, TBCTL_PRDLD_MASK, TBCTL_PRDLD_SHDW);
    ehrpwm_write(dev, TBPRD, period_cycles / prescale_div);
    ehrpwm_modify(dev, TBCTL, TBCTL_CTRMODE_MASK, TBCTL_CTRMODE_UP);

    /* Set duty cycle */
    uint16_t duty = pulse_cycles / prescale_div;
    ehrpwm_write(dev, (channel == 1) ? CMPB : CMPA, duty);

    /* Configure polarity */
    configure_polarity(dev, channel);

    return 0;
}

static int pwm_ehrpwm_get_cycles_per_sec(const struct device *dev, 
                                        uint32_t channel, uint64_t *cycles)
{
    if (channel >= NUM_CHANNELS) {
        return -EINVAL;
    }

    struct pwm_ehrpwm_data *data = dev->data;
    *cycles = data->clk_rate;
    return 0;
}

static int pwm_ehrpwm_init(const struct device *dev)
{
    const struct pwm_ehrpwm_config *config = dev->config;
    struct pwm_ehrpwm_data *data = dev->data;
    int ret;

    /* Get clock rate */
    ret = clock_control_get_rate(config->clock_dev, config->clock_subsys, 
                                &data->clk_rate);
    if (ret < 0) {
        LOG_ERR("Failed to get clock rate: %d", ret);
        return ret;
    }

    /* Enable clock */
    ret = clock_control_on(config->clock_dev, config->clock_subsys);
    if (ret < 0) {
        LOG_ERR("Failed to enable clock: %d", ret);
        return ret;
    }

    return 0;
}

static DEVICE_API(pwm,pwm_ehrpwm_driver_api) = {
    .set_cycles = pwm_ehrpwm_set_cycles,
    .get_cycles_per_sec = pwm_ehrpwm_get_cycles_per_sec,
};

#define PWM_EHRPWM_INIT(n) \
    static struct pwm_ehrpwm_data pwm_ehrpwm_data_##n; \
    static const struct pwm_ehrpwm_config pwm_ehrpwm_config_##n = { \
        .base = DT_INST_REG_ADDR(n), \
        .clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)), \
        .clock_subsys = (clock_control_subsys_t)DT_INST_CLOCKS_CELL(n, bits), \
    }; \
    DEVICE_DT_INST_DEFINE(n, \
            pwm_ehrpwm_init, \
            NULL, \
            &pwm_ehrpwm_data_##n, \
            &pwm_ehrpwm_config_##n, \
            POST_KERNEL, \
            CONFIG_KERNEL_INIT_PRIORITY_DEVICE, \
            &pwm_ehrpwm_driver_api);

DT_INST_FOREACH_STATUS_OKAY(PWM_EHRPWM_INIT)