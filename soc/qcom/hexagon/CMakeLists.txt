# Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
# SPDX-License-Identifier: Apache-2.0

zephyr_library()
zephyr_library_sources(soc.c)
zephyr_library_sources_ifdef(CONFIG_QCOM_HEXAGON_SMP2P_READY smp2p.c)

set(SOC_LINKER_SCRIPT ${ZEPHYR_BASE}/include/zephyr/arch/hexagon/linker.ld CACHE INTERNAL "")

# Remoteproc loads HVM and Zephyr as one ELF. Given an HVM loadlinux linked
# for this DSP, write zephyr/zephyr_hvm.elf for signing.
zephyr_get(HEXAGON_H2_LOADLINUX)
if(HEXAGON_H2_LOADLINUX)
  set(fw ${ZEPHYR_BINARY_DIR}/${KERNEL_NAME}_hvm.elf)
  set_property(GLOBAL APPEND PROPERTY extra_post_build_commands
    COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_LIST_DIR}/merge_elf.py
      --hvm ${HEXAGON_H2_LOADLINUX}
      --guest ${ZEPHYR_BINARY_DIR}/${KERNEL_ELF_NAME}
      -o ${fw}
  )
  set_property(GLOBAL APPEND PROPERTY extra_post_build_byproducts ${fw})
endif()
