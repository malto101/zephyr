/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/ztest.h>
#include <zephyr/sys/util.h>
#include <hexagon_vm.h>

ZTEST(smp_hvm, test_vpid_boot_cpu)
{
	uint32_t id = hexagon_vm_vpid();

	TC_PRINT("boot VCPU id 0x%08x cpuidx %u\n", id, HVM_ID_CPUIDX(id));
	zassert_true(HVM_ID_CPUIDX(id) < HVM_MAX_VCPUS, "boot VCPU cpuidx %u", HVM_ID_CPUIDX(id));
	zassert_equal(hexagon_vm_vpid(), id, "vpid not stable");
}

static volatile uint32_t probe_vpid;

void probe_entry_c(void)
{
	probe_vpid = hexagon_vm_vpid();
	hexagon_vm_stop(VM_STOP_NONE);
}

/* A new VCPU's FRAMELIMIT and FRAMEKEY are undefined, clear them first */
void probe_entry(void);
__asm__(".pushsection .text\n"
	".p2align 2\n"
	"probe_entry:\n"
	"	r0 = #0\n"
	"	r30 = #0\n"
	"	c16 = r0\n"
	"	c17 = r0\n"
	"	jump probe_entry_c\n"
	".popsection\n");

ZTEST(smp_hvm, test_vmstart_runs_second_vcpu)
{
	static uint64_t probe_stack[64];
	int32_t id;

	id = hexagon_vm_start(probe_entry, &probe_stack[ARRAY_SIZE(probe_stack)]);
	zassert_not_equal(id, -1, "vmstart failed");
	TC_PRINT("second VCPU id 0x%08x\n", (uint32_t)id);
	zassert_true(HVM_ID_CPUIDX((uint32_t)id) < HVM_MAX_VCPUS, "new VCPU cpuidx out of range");
	zassert_not_equal(HVM_ID_CPUIDX((uint32_t)id), HVM_ID_CPUIDX(hexagon_vm_vpid()),
			  "new VCPU has the caller's cpuidx");

	zassert_true(WAIT_FOR(probe_vpid != 0U, 1000000, k_busy_wait(10)),
		     "second VCPU did not run");
	zassert_equal(probe_vpid, (uint32_t)id, "secondary saw vpid 0x%x, vmstart returned 0x%x",
		      probe_vpid, (uint32_t)id);
}

ZTEST_SUITE(smp_hvm, NULL, NULL, NULL, NULL, NULL);
