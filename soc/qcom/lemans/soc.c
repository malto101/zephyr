/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>

#include <zephyr/init.h>
#include <zephyr/linker/section_tags.h>
#include <zephyr/arch/common/sys_io.h>
#include <zephyr/sys/sys_io.h>

#include <lemans_runtime.h>

/* The linker reserves this NOLOAD section outside ordinary noinit. */
volatile struct lemans_native_diag z_lemans_native_diag
	__attribute__((section("lemans_native_diag")));

void z_lemans_native_diag_stage(uint32_t stage)
{
	z_lemans_native_diag.stage = stage;
}

void z_lemans_native_early_startup(void)
{
	uint32_t watchdog_ctl;

	if (z_lemans_native_diag.magic == LEMANS_DIAG_MAGIC) {
		z_lemans_native_diag.boot_count++;
	} else {
		z_lemans_native_diag.boot_count = 1U;
		z_lemans_native_diag.last_error = 0U;
	}

	z_lemans_native_diag.magic = LEMANS_DIAG_MAGIC;
	z_lemans_native_diag.stage = LEMANS_DIAG_STAGE_RESET;
	z_lemans_native_diag.carveout_base = LEMANS_CARVEOUT_BASE;
	z_lemans_native_diag.carveout_size = LEMANS_CARVEOUT_SIZE;

	/* The reference Lemans watchdog disables the counter through CTL.ENABLE. */
	watchdog_ctl = sys_read32((mm_reg_t)LEMANS_WDOG_CTL);
	z_lemans_native_diag.watchdog_ctl_before = watchdog_ctl;
	z_lemans_native_diag.watchdog_status =
		sys_read32((mm_reg_t)LEMANS_WDOG_STATUS);
	sys_write32(watchdog_ctl & ~LEMANS_WDOG_ENABLE,
		    (mm_reg_t)LEMANS_WDOG_CTL);
	z_lemans_native_diag.watchdog_ctl_after =
		sys_read32((mm_reg_t)LEMANS_WDOG_CTL);
	z_lemans_native_diag.stage = LEMANS_DIAG_STAGE_WDOG;
}

static int lemans_soc_init(void)
{
	z_lemans_native_diag_stage(LEMANS_DIAG_STAGE_CSTART);
	return 0;
}

SYS_INIT(lemans_soc_init, PRE_KERNEL_1, 0);
