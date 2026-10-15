/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/toolchain.h>
#include <zephyr/linker/sections.h>
#include <switch_frame.h>

GTEXT(z_hexagon_arch_switch)
GTEXT(z_hexagon_thread_start)

SECTION_FUNC(TEXT, z_hexagon_arch_switch)
	r29 = add(r29, #-SWITCH_FRAME_SIZE)
	memd(r29+#SWITCH_R1716) = r17:16
	memd(r29+#SWITCH_R1918) = r19:18
	memd(r29+#SWITCH_R2120) = r21:20
	memd(r29+#SWITCH_R2322) = r23:22
	memd(r29+#SWITCH_R2524) = r25:24
	memd(r29+#SWITCH_R2726) = r27:26
	memd(r29+#SWITCH_FP_LR) = r31:30

#ifdef CONFIG_THREAD_LOCAL_STORAGE
	r2 = ugp
	memw(r29+#SWITCH_UGP) = r2
#endif

	memw(r1) = r29
	r29 = r0

	{
		r17:16 = memd(r29+#SWITCH_R1716)
		r19:18 = memd(r29+#SWITCH_R1918)
	}
	{
		r21:20 = memd(r29+#SWITCH_R2120)
		r23:22 = memd(r29+#SWITCH_R2322)
	}
	{
		r25:24 = memd(r29+#SWITCH_R2524)
	}
	{
		r27:26 = memd(r29+#SWITCH_R2726)
	}
	{
		r31:30 = memd(r29+#SWITCH_FP_LR)
	}

#ifdef CONFIG_THREAD_LOCAL_STORAGE
	r2 = memw(r29+#SWITCH_UGP)
	ugp = r2
#endif

	r29 = add(r29, #SWITCH_FRAME_SIZE)
	jumpr r31

SECTION_FUNC(TEXT, z_hexagon_thread_start)
	r0 = ssr
	r0 = or(r0, #0x40000)
	ssr = r0
	isync
	{
		r0 = r16
		r1 = r17
		r2 = r18
		r3 = r19
	}
	jump ##z_thread_entry
