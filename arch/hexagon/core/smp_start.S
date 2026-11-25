/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/toolchain.h>
#include <zephyr/linker/sections.h>

GTEXT(z_hexagon_secondary_entry)
GTEXT(z_hexagon_secondary_c)

/*
 * vmstart entry: SP is set, IE is off, GEVB and the page table are
 * inherited from the starting VCPU. FRAMELIMIT and FRAMEKEY are
 * undefined, clear them before the first allocframe.
 */
SECTION_FUNC(TEXT, z_hexagon_secondary_entry)
	{
		r0 = #0
		r30 = #0
		r31 = #0
	}
	c16 = r0
	c17 = r0
	jump z_hexagon_secondary_c
