/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>

#include <zephyr/toolchain.h>

#ifdef CONFIG_SOC_LEMANS_CDSP0
#include <lemans_runtime.h>
#endif

uint32_t z_hexagon_isr_nesting;

void z_hexagon_native_record_fault(uint32_t event_slot)
{
#ifdef CONFIG_SOC_LEMANS_CDSP0
	uint32_t ssr;
	uint32_t elr;
	uint32_t badva;

	__asm__ volatile("%0 = ssr" : "=r"(ssr));
	__asm__ volatile("%0 = elr" : "=r"(elr));
	__asm__ volatile("%0 = badva" : "=r"(badva));

	z_lemans_native_diag_record_event(ssr, elr, ssr & 0xffU, badva,
					  event_slot);
	z_lemans_native_diag_fail(LEMANS_DIAG_STAGE_EVENT,
					  LEMANS_DIAG_ERROR_EVENT);
#else
	ARG_UNUSED(event_slot);
#endif
}
