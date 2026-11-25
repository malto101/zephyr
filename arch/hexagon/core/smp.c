/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <hexagon_vm.h>

BUILD_ASSERT(CONFIG_MP_MAX_NUM_CPUS <= HVM_MAX_VCPUS, "more CPUs than HVM VCPU slots");

_cpu_t *z_hexagon_slot_cpu[HVM_MAX_VCPUS];
