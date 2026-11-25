/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/barrier.h>
#include <kernel_internal.h>
#include <ipi.h>
#include <hexagon_vm.h>
#include <irq.h>

#ifdef CONFIG_THREAD_LOCAL_STORAGE
#include <kernel_tls.h>
#endif

BUILD_ASSERT(CONFIG_MP_MAX_NUM_CPUS <= HVM_MAX_VCPUS, "more CPUs than HVM VCPU slots");

_cpu_t *z_hexagon_slot_cpu[HVM_MAX_VCPUS];

/* The kernel starts one CPU at a time and waits for it to come up */
static struct {
	arch_cpustart_t fn;
	void *arg;
	char *stack;
} start_args;

static atomic_t slot_published;

void z_hexagon_secondary_entry(void);

#ifdef CONFIG_SMP
static void ipi_isr(const void *arg)
{
	ARG_UNUSED(arg);

	z_sched_ipi();
}

/* A CPU's IPI is enabled on that CPU only, so a post reaches just it */
static void ipi_init_local(void)
{
	int32_t irq = HEXAGON_IPI_IRQ_BASE + (int32_t)arch_proc_id();

	hexagon_vm_intop_locen(irq);
	hexagon_vm_intop_globen(irq);
}

void arch_sched_directed_ipi(uint32_t cpu_bitmap)
{
	unsigned int num_cpus = arch_num_cpus();

	for (unsigned int i = 0U; i < num_cpus; i++) {
		if ((cpu_bitmap & BIT(i)) != 0U) {
			hexagon_vm_intop_post(HEXAGON_IPI_IRQ_BASE + (int32_t)i, 0);
		}
	}
}

void arch_sched_broadcast_ipi(void)
{
	arch_sched_directed_ipi(IPI_ALL_CPUS_MASK & ~BIT(arch_proc_id()));
}

/* HVM asks guests to yield a VCPU that spins, so the lock holder can run */
void arch_spin_relax(void)
{
	hexagon_vm_yield();
}

#define IPI_CONNECT(n, _) IRQ_CONNECT(HEXAGON_IPI_IRQ_BASE + (n), 0, ipi_isr, NULL, 0)

int arch_smp_init(void)
{
	LISTIFY(CONFIG_MP_MAX_NUM_CPUS, IPI_CONNECT, (;));
	ipi_init_local();

	return 0;
}
#endif

void arch_cpu_start(int cpu_num, k_thread_stack_t *stack, int sz, arch_cpustart_t fn, void *arg)
{
	char *top = (char *)ROUND_DOWN((uintptr_t)K_KERNEL_STACK_BUFFER(stack) + sz, 8);
	int32_t id;

	start_args.fn = fn;
	start_args.arg = arg;
	start_args.stack = K_KERNEL_STACK_BUFFER(stack);
	atomic_clear(&slot_published);
	barrier_dmem_fence_full();

	id = hexagon_vm_start(z_hexagon_secondary_entry, top);
	__ASSERT(id != -1, "vmstart failed for CPU %d", cpu_num);
	if (id == -1) {
		return;
	}

	/* The new CPU cannot find its _cpu_t until this is published */
	z_hexagon_slot_cpu[HVM_ID_CPUIDX((uint32_t)id)] = &_kernel.cpus[cpu_num];
	barrier_dmem_fence_full();
	atomic_set(&slot_published, 1);
}

FUNC_NORETURN FUNC_NO_STACK_PROTECTOR void z_hexagon_secondary_c(void)
{
#ifdef CONFIG_THREAD_LOCAL_STORAGE
	/* Boot TLS block at the bottom of the start stack, as in z_prep_c() */
	char *block = (char *)ROUND_UP((uintptr_t)start_args.stack, 8);

	z_tls_copy(block);
	__asm__ volatile("ugp = %0" : : "r"(block + ROUND_UP(z_tls_data_size(), 8)));
#endif

	while (atomic_get(&slot_published) == 0) {
		hexagon_vm_yield();
	}

#ifdef CONFIG_SMP
	ipi_init_local();
#endif
	start_args.fn(start_args.arg);
	CODE_UNREACHABLE;
}
