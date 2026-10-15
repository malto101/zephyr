# SPDX-License-Identifier: Apache-2.0
# Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.

zephyr_library()
zephyr_include_directories(include)
zephyr_library_sources_ifdef(CONFIG_SOC_LEMANS_CDSP0
	soc.c
	mmu.c
)
zephyr_library_sources_ifdef(CONFIG_LEMANS_SMP2P smp2p.c)

set(SOC_LINKER_SCRIPT "${CMAKE_CURRENT_SOURCE_DIR}/linker.ld" CACHE INTERNAL "")

if(CONFIG_SOC_LEMANS_CDSP0 AND CONFIG_HEXAGON_NATIVE)
	zephyr_linker_sources(RAM_SECTIONS lemans_diag.ld)
	set_property(GLOBAL APPEND PROPERTY extra_post_build_commands
		COMMAND ${PYTHON_EXECUTABLE}
			${CMAKE_CURRENT_SOURCE_DIR}/validate_native_elf.py
			${PROJECT_BINARY_DIR}/${KERNEL_ELF_NAME}
	)
endif()

if(CONFIG_SOC_LEMANS_CDSP0)
	zephyr_compile_options(-mv73)
	zephyr_link_libraries(-mv73)
endif()
