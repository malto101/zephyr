# Copyright (C) 2023 BeagleBoard.org Foundation
# Copyright (C) 2023 S Prashanth
#
# SPDX-License-Identifier: Apache-2.0

zephyr_include_directories(.)

if(CONFIG_SOC_J721E_R5 OR CONFIG_SOC_J722S_R5_MAIN)
  zephyr_sources(r5/soc.c)

  zephyr_include_directories(r5/)

  if(CONFIG_OPENAMP_RSC_TABLE)
    zephyr_linker_section(NAME .resource_table GROUP ROM_REGION NOINPUT)
    zephyr_linker_section_configure(SECTION .resource_table KEEP INPUT ".resource_table*")
  endif()

  set(SOC_LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/r5/linker.ld CACHE INTERNAL "")
endif()
