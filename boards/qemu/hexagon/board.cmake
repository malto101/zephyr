# SPDX-License-Identifier: Apache-2.0
# Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.

set(SUPPORTED_EMU_PLATFORMS qemu)
set(QEMU_BINARY_SUFFIX hexagon)

set(QEMU_BOARD_FLAGS
  -machine virt
  -m 4G
)

# Hexagon boots Zephyr as an H2 hypervisor guest.  QEMU loads H2's
# "loadlinux" as -kernel; H2 then boots the Zephyr ELF placed at the
# guest load address via -device loader.
#
# loadlinux is a full Hexagon ELF, so it has to come in through -kernel:
# the virt machine's -bios path is load_image_targphys() of a raw blob at
# the reset vector, capped at 64 KiB, and rejects an ELF this size with
# "Could not load BIOS".
#
# Point HEXAGON_H2_LOADLINUX (env var or cmake -D) at an H2 build, e.g. from
# https://github.com/qualcomm/hexagon-hypervisor. Without it QEMU is left to
# find its own firmware, which only works on a QEMU that bundles one.
if(DEFINED ENV{HEXAGON_H2_LOADLINUX})
  set(HEXAGON_H2_LOADLINUX $ENV{HEXAGON_H2_LOADLINUX})
endif()

if(HEXAGON_H2_LOADLINUX)
  get_filename_component(HEXAGON_H2_LOADLINUX "${HEXAGON_H2_LOADLINUX}" ABSOLUTE)

  if(NOT EXISTS "${HEXAGON_H2_LOADLINUX}")
    message(WARNING
      "HEXAGON_H2_LOADLINUX set but not found at: ${HEXAGON_H2_LOADLINUX}\n"
      "Falling back to QEMU's bundled H2 loadlinux firmware."
    )
    unset(HEXAGON_H2_LOADLINUX)
  endif()
endif()

# Pass zephyr.elf via device loader below instead of -kernel (the generic
# loader parses ELF program headers and places segments at their physical
# addresses, so no explicit addr= or objcopy-to-bin is needed). Explicitly
# clear QEMU_KERNEL_OPTION so cmake/emu/qemu.cmake doesn't fall back to its
# own "-kernel <zephyr.elf>" default, which would boot Zephyr directly
# instead of through the H2 hypervisor.
if(HEXAGON_H2_LOADLINUX)
  set(QEMU_KERNEL_OPTION "-kernel;${HEXAGON_H2_LOADLINUX}")
else()
  set(QEMU_KERNEL_OPTION "")
endif()

list(APPEND QEMU_EXTRA_FLAGS
  "-device;loader,file=${ZEPHYR_BINARY_DIR}/${KERNEL_ELF_NAME}"
)

board_set_debugger_ifnset(qemu)
