/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_SOC_QCOM_LEMANS_INCLUDE_LEMANS_RUNTIME_H_
#define ZEPHYR_SOC_QCOM_LEMANS_INCLUDE_LEMANS_RUNTIME_H_

#include <stdint.h>

#include <zephyr/devicetree.h>
#include <zephyr/sys/util.h>

#define LEMANS_SRAM_NODE DT_NODELABEL(sram0)
#define LEMANS_L2VIC_NODE DT_NODELABEL(l2vic)
#define LEMANS_QTIMER_NODE DT_NODELABEL(qtimer)
#define LEMANS_WDOG_NODE DT_NODELABEL(wdog)
#define LEMANS_IPCC_NODE DT_NODELABEL(ipcc)
#define LEMANS_SMEM_NODE DT_NODELABEL(smem)

/*
 * The first native image is identity-mapped. These values describe the
 * physical direct-boot contract; they are not a QuRT virtual-address map.
 */
#define LEMANS_CARVEOUT_BASE DT_REG_ADDR(LEMANS_SRAM_NODE)
#define LEMANS_CARVEOUT_SIZE DT_REG_SIZE(LEMANS_SRAM_NODE)
#define LEMANS_CARVEOUT_END (LEMANS_CARVEOUT_BASE + LEMANS_CARVEOUT_SIZE)

#define LEMANS_QDSP6SS_BASE DT_REG_ADDR(DT_NODELABEL(qdsp6ss))
#define LEMANS_L2VIC_BASE DT_REG_ADDR(LEMANS_L2VIC_NODE)
#define LEMANS_QTIMER_AC_BASE \
	DT_REG_ADDR_BY_NAME(LEMANS_QTIMER_NODE, ac)
#define LEMANS_QTIMER_FRAME_BASE \
	DT_REG_ADDR_BY_NAME(LEMANS_QTIMER_NODE, frame)
#define LEMANS_WDOG_BASE DT_REG_ADDR(LEMANS_WDOG_NODE)
#define LEMANS_IPCC_BASE DT_REG_ADDR(LEMANS_IPCC_NODE)
#define LEMANS_TCSR_BASE DT_REG_ADDR(DT_NODELABEL(tcsr))
#define LEMANS_SMEM_BASE DT_REG_ADDR(LEMANS_SMEM_NODE)
#define LEMANS_SMEM_SIZE DT_REG_SIZE(LEMANS_SMEM_NODE)
#define LEMANS_IPCC_SIZE DT_REG_SIZE(LEMANS_IPCC_NODE)
#define LEMANS_TCSR_SIZE DT_REG_SIZE(DT_NODELABEL(tcsr))
#define LEMANS_QDSP6SS_SIZE DT_REG_SIZE(DT_NODELABEL(qdsp6ss))
#define LEMANS_L2VIC_SIZE DT_REG_SIZE(LEMANS_L2VIC_NODE)
#define LEMANS_WDOG_SIZE DT_REG_SIZE(LEMANS_WDOG_NODE)
#define LEMANS_QTIMER_AC_SIZE DT_REG_SIZE_BY_NAME(LEMANS_QTIMER_NODE, ac)
#define LEMANS_QTIMER_FRAME_SIZE DT_REG_SIZE_BY_NAME(LEMANS_QTIMER_NODE, frame)

BUILD_ASSERT(LEMANS_QDSP6SS_BASE == 0x26380000U,
	"Unexpected Lemans QDSP6SS base");
BUILD_ASSERT(LEMANS_L2VIC_BASE == (LEMANS_QDSP6SS_BASE + 0x10000U),
	"Lemans L2VIC is not relative to QDSP6SS");
BUILD_ASSERT(LEMANS_QTIMER_AC_BASE == (LEMANS_QDSP6SS_BASE + 0x20000U),
	"Lemans QTimer AC is not relative to QDSP6SS");
BUILD_ASSERT(LEMANS_QTIMER_FRAME_BASE == (LEMANS_QTIMER_AC_BASE + 0x1000U),
	"Lemans QTimer frame is not F0");
BUILD_ASSERT(LEMANS_WDOG_BASE == (LEMANS_QDSP6SS_BASE + 0x4000U),
	"Lemans watchdog is not relative to QDSP6SS");
BUILD_ASSERT(LEMANS_IPCC_BASE == 0x00400000U,
	"Unexpected Lemans IPCC base");
BUILD_ASSERT(LEMANS_TCSR_BASE == 0x01F00000U,
	"Unexpected Lemans TCSR base");
BUILD_ASSERT(LEMANS_SMEM_BASE == 0x90900000U,
	"Unexpected Lemans SMEM base");
BUILD_ASSERT(LEMANS_SMEM_SIZE == 0x00200000U,
	"Unexpected Lemans SMEM size");

#define LEMANS_IPCC_SEND \
	(LEMANS_IPCC_BASE + (6U * 0x1000U) + 0x0CU)
#define LEMANS_IPCC_SMP2P_PAYLOAD ((8U << 16) | 2U)

#define LEMANS_TCSR_SMEM_TARG_INFO_LO \
	(LEMANS_TCSR_BASE + 0x000D4000U)
#define LEMANS_TCSR_SMEM_TARG_INFO_HI \
	(LEMANS_TCSR_BASE + 0x000D4004U)

#define LEMANS_QTIMER_FREQ 19200000U
#define LEMANS_QTIMER_IRQ 2U
#define LEMANS_WDOG_IRQ 58U
#define LEMANS_IPCC_IRQ 30U

BUILD_ASSERT(DT_PROP(LEMANS_QTIMER_NODE, clock_frequency) == LEMANS_QTIMER_FREQ,
	"Unexpected Lemans QTimer frequency");
BUILD_ASSERT(DT_IRQN(LEMANS_QTIMER_NODE) == LEMANS_QTIMER_IRQ,
	"Unexpected Lemans QTimer IRQ");
BUILD_ASSERT(DT_IRQN(LEMANS_WDOG_NODE) == LEMANS_WDOG_IRQ,
	"Unexpected Lemans watchdog IRQ");
BUILD_ASSERT(DT_IRQN(LEMANS_IPCC_NODE) == LEMANS_IPCC_IRQ,
	"Unexpected Lemans IPCC IRQ");

#define LEMANS_WDOG_CTL (LEMANS_WDOG_BASE + 0x04U)
#define LEMANS_WDOG_STATUS (LEMANS_WDOG_BASE + 0x08U)
#define LEMANS_WDOG_ENABLE BIT(0)

#define LEMANS_DIAG_MAGIC 0x4C4D4447U

#define LEMANS_DIAG_ERROR_NONE 0U
#define LEMANS_DIAG_ERROR_WATCHDOG 1U
#define LEMANS_DIAG_ERROR_TLB 2U
#define LEMANS_DIAG_ERROR_CACHE 3U
#define LEMANS_DIAG_ERROR_EVENT 4U
#define LEMANS_DIAG_ERROR_INTC 5U
#define LEMANS_DIAG_ERROR_TIMER 6U
#define LEMANS_DIAG_ERROR_SMP2P 7U

#define LEMANS_DIAG_STAGE_RESET 1U
#define LEMANS_DIAG_STAGE_WDOG 2U
#define LEMANS_DIAG_STAGE_CSTART 3U
#define LEMANS_DIAG_STAGE_INTC 4U
#define LEMANS_DIAG_STAGE_TIMER 5U
#define LEMANS_DIAG_STAGE_SMP2P 6U
#define LEMANS_DIAG_STAGE_HELLO 7U
#define LEMANS_DIAG_STAGE_EVENT 8U

struct lemans_native_diag {
	uint32_t magic;
	uint32_t boot_count;
	uint32_t stage;
	uint32_t last_error;
	uint32_t watchdog_ctl_before;
	uint32_t watchdog_ctl_after;
	uint32_t watchdog_status;
	uint32_t carveout_base;
	uint32_t carveout_size;
	uint32_t tlb_index;
	uint32_t tlb_hi;
	uint32_t tlb_lo;
	uint32_t syscfg_before;
	uint32_t syscfg_after;
	uint32_t ssr;
	uint32_t elr;
	uint32_t cause;
	uint32_t bad_address;
	uint32_t event_slot;
	uint32_t failure_stage;
};

BUILD_ASSERT(sizeof(struct lemans_native_diag) <= 0x1000U,
	"Lemans native diagnostic record exceeds its reserved section");

extern volatile struct lemans_native_diag z_lemans_native_diag;

static ALWAYS_INLINE void z_lemans_native_diag_fail(uint32_t stage,
							    uint32_t error)
{
	z_lemans_native_diag.stage = stage;
	z_lemans_native_diag.failure_stage = stage;
	z_lemans_native_diag.last_error = error;
}

static ALWAYS_INLINE void z_lemans_native_diag_record_event(uint32_t ssr,
							    uint32_t elr,
							    uint32_t cause,
							    uint32_t bad_address,
							    uint32_t event_slot)
{
	z_lemans_native_diag.ssr = ssr;
	z_lemans_native_diag.elr = elr;
	z_lemans_native_diag.cause = cause;
	z_lemans_native_diag.bad_address = bad_address;
	z_lemans_native_diag.event_slot = event_slot;
}

void z_lemans_native_early_startup(void);
void z_lemans_native_diag_stage(uint32_t stage);

BUILD_ASSERT((LEMANS_CARVEOUT_BASE & 0xFFFU) == 0U,
	"Lemans carveout base must be page aligned");
BUILD_ASSERT((LEMANS_CARVEOUT_SIZE & 0xFFFU) == 0U,
	"Lemans carveout size must be page aligned");
BUILD_ASSERT(LEMANS_CARVEOUT_SIZE >= 0x10000U,
	"Lemans carveout is too small for native startup");
BUILD_ASSERT((LEMANS_L2VIC_BASE & 0xFFFU) == 0U,
	"Lemans L2VIC base must be page aligned");
BUILD_ASSERT((LEMANS_QTIMER_AC_BASE & 0xFFFU) == 0U,
	"Lemans QTimer base must be page aligned");
BUILD_ASSERT((LEMANS_QTIMER_FRAME_BASE & 0xFFFU) == 0U,
	"Lemans QTimer frame must be page aligned");
BUILD_ASSERT((LEMANS_WDOG_BASE & 0xFFFU) == 0U,
	"Lemans watchdog base must be page aligned");
BUILD_ASSERT((LEMANS_IPCC_BASE & 0xFFFU) == 0U,
	"Lemans IPCC base must be page aligned");
BUILD_ASSERT((LEMANS_TCSR_BASE & 0xFFFU) == 0U,
	"Lemans TCSR base must be page aligned");
BUILD_ASSERT((LEMANS_SMEM_BASE & 0xFFFU) == 0U,
	"Lemans SMEM base must be page aligned");
BUILD_ASSERT((LEMANS_SMEM_SIZE & 0xFFFU) == 0U,
	"Lemans SMEM size must be page aligned");

#endif /* ZEPHYR_SOC_QCOM_LEMANS_INCLUDE_LEMANS_RUNTIME_H_ */
