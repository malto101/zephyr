# SPDX-License-Identifier: Apache-2.0
# Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.

zephyr_library()

zephyr_library_sources(
  prep_c.c
  thread.c
)

zephyr_library_sources_ifdef(CONFIG_HEXAGON_HVM_GUEST
  hvm_event_vectors.S
  event_handlers.S
  switch.S
  cpu_idle.c
  fatal.c
  irq_manage.c
  intc.c
  mem_manage.c
)

zephyr_library_sources_ifdef(CONFIG_HEXAGON_NATIVE
  fatal.c
  native_event_vectors.S
  native_intc.c
  native_irq.c
  native_status.c
  native_switch.S
  native_cpu_idle.c
)


zephyr_library_sources_ifdef(CONFIG_THREAD_LOCAL_STORAGE tls.c)
zephyr_library_sources_ifdef(CONFIG_IRQ_OFFLOAD irq_offload.c)

zephyr_library_sources_ifdef(CONFIG_HW_STACK_PROTECTION stack_protect.c)
if(CONFIG_PM AND CONFIG_HEXAGON_NATIVE)
  message(FATAL_ERROR
    "CONFIG_PM is not supported by the native Hexagon backend; use a native idle implementation")
endif()

zephyr_library_sources_ifdef(CONFIG_PM power.c)
zephyr_library_sources_ifdef(CONFIG_ARCH_STACKWALK stacktrace.c)
zephyr_library_sources_ifdef(CONFIG_DEBUG_COREDUMP coredump.c)
zephyr_library_sources_ifdef(CONFIG_GDBSTUB gdbstub.c)

zephyr_library_sources_ifdef(CONFIG_MMU userspace.c)
zephyr_library_sources_ifdef(CONFIG_USERSPACE user_mode_state.c)
