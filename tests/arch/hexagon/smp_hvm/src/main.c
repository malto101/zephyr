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

ZTEST(smp_hvm, test_cpu_lookup)
{
	unsigned int key = arch_irq_lock();
	_cpu_t *cpu = arch_curr_cpu();
	uint32_t id = arch_proc_id();

	arch_irq_unlock(key);

	zassert_true(id < arch_num_cpus(), "CPU id %u", id);
	zassert_equal_ptr(cpu, &_kernel.cpus[id], "CPU %u has the wrong _cpu_t", id);
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

#if defined(CONFIG_SMP) && defined(CONFIG_SCHED_CPU_MASK) && defined(CONFIG_IRQ_OFFLOAD)
#include <zephyr/irq_offload.h>

static volatile int32_t offload_cpu;

static void offload_fn(const void *arg)
{
	ARG_UNUSED(arg);
	offload_cpu = (int32_t)arch_proc_id();
}

static void offload_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	offload_cpu = -1;
	irq_offload(offload_fn, NULL);
	zassert_equal(offload_cpu, (int32_t)(uintptr_t)p1, "offload ran on CPU %d",
		      offload_cpu);
}

ZTEST(smp_hvm, test_irq_offload_each_cpu)
{
	static K_THREAD_STACK_DEFINE(stack, 1024);
	static struct k_thread thread;

	for (unsigned int cpu = 0U; cpu < arch_num_cpus(); cpu++) {
		k_tid_t tid = k_thread_create(&thread, stack, K_THREAD_STACK_SIZEOF(stack),
					      offload_thread, (void *)(uintptr_t)cpu, NULL, NULL,
					      K_PRIO_PREEMPT(0), 0, K_FOREVER);

		zassert_ok(k_thread_cpu_pin(tid, (int)cpu));
		k_thread_start(tid);
		zassert_ok(k_thread_join(tid, K_SECONDS(5)), "offload on CPU %u hung", cpu);
	}
}
#endif

ZTEST_SUITE(smp_hvm, NULL, NULL, NULL, NULL, NULL);
