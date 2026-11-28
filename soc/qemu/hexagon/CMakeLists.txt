# SPDX-License-Identifier: Apache-2.0
# Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.

zephyr_library()
zephyr_library_sources_ifdef(CONFIG_SOC_QEMU_HEXAGON_VIRT soc.c)

# Common Hexagon HVM guest linker script
set(SOC_LINKER_SCRIPT ${ZEPHYR_BASE}/include/zephyr/arch/hexagon/linker.ld CACHE INTERNAL "")
