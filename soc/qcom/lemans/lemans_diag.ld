/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Keep the early native diagnostic record in RAM and out of the image's
 * initialized data.  Its address is reported by the record itself; the
 * section is not a hardware console or a persistence guarantee.
 */
SECTION_PROLOGUE(lemans_native_diag, (NOLOAD), ALIGN(8))
{
	__lemans_native_diag_start = .;
	KEEP(*(lemans_native_diag))
	. = ALIGN(8);
	ASSERT(. <= __lemans_native_diag_start + 0x1000,
		"Lemans diagnostic record exceeds its reserved 4 KiB")
	. = __lemans_native_diag_start + 0x1000;
	__lemans_native_diag_end = .;
} > LEMANS_DIAG

__lemans_native_diag_size = __lemans_native_diag_end -
	__lemans_native_diag_start;

ASSERT(__lemans_native_diag_start == ORIGIN(LEMANS_DIAG),
	"Lemans diagnostic section is not at carveout end")
ASSERT(__lemans_native_diag_end == ORIGIN(LEMANS_DIAG) + LENGTH(LEMANS_DIAG),
	"Lemans diagnostic section does not fill reserved region")
ASSERT(__lemans_native_diag_size == 0x1000,
	"Lemans diagnostic section is not 4 KiB")
ASSERT(_image_ram_end <= ORIGIN(LEMANS_DIAG),
	"Ordinary RAM allocations overlap Lemans diagnostic region")
ASSERT(_interrupt_stack_end <= ORIGIN(LEMANS_DIAG),
	"Interrupt stack overlaps Lemans diagnostic region")
ASSERT(_end <= ORIGIN(LEMANS_DIAG),
	"Image end overlaps Lemans diagnostic region")
