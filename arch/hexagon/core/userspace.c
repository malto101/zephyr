/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Hexagon userspace support
 */

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/arch/hexagon/arch.h>
#include <zephyr/internal/syscall_handler.h>
#include <zephyr/linker/linker-defs.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/barrier.h>
#include <zephyr/sys/slist.h>
#include <kernel_internal.h>
#include <hexagon_vm.h>
#include <hexagon_mmu.h>

LOG_MODULE_DECLARE(os, CONFIG_KERNEL_LOG_LEVEL);

#ifdef CONFIG_USERSPACE

/*
 * Defined near the arch_mem_domain_* hooks below. Called from here,
 * switch.S, and event_handlers.S.
 */
void z_hexagon_mem_domain_switch(void);
static uint32_t hexagon_domain_pte(const struct k_mem_domain *domain,
				   const struct k_thread *owner, uintptr_t va, int skip_id);

/* [start, start + size) inside [base, base + len), overflow-safe. */
static inline bool hexagon_range_within(uintptr_t start, size_t size, uintptr_t base, size_t len)
{
	return start >= base && size <= len && (start - base) <= (len - size);
}

/*
 * A supervisor thread has no installed user map. Allow access to its
 * own stack, plus reads from global text/rodata.
 */
static bool hexagon_supervisor_buffer_ok(const struct k_thread *thread, uintptr_t start,
					 size_t size, int write)
{
	if (hexagon_range_within(start, size, thread->stack_info.start, thread->stack_info.size)) {
		return true;
	}

	if (write != 0) {
		return false;
	}

	if (hexagon_range_within(start, size, (uintptr_t)__rom_region_start,
				 (uintptr_t)__rom_region_end - (uintptr_t)__rom_region_start)) {
		return true;
	}

	return hexagon_range_within(
		start, size, (uintptr_t)__rodata_region_start,
		(uintptr_t)__rodata_region_end - (uintptr_t)__rodata_region_start);
}

/*
 * Check the calling thread's domain translation, the same one HVM
 * enforces on its user-mode accesses: its stack, its domain's mappable
 * partitions and the kernel's U=1 text, rodata and console pages.
 */
int arch_buffer_validate(const void *addr, size_t size, int write)
{
	struct k_thread *thread = k_current_get();
	uint32_t need = __HVM_PTE_U | ((write != 0) ? __HVM_PTE_W : __HVM_PTE_R);
	uintptr_t start = (uintptr_t)addr;
	uintptr_t last;
	k_spinlock_key_t key;
	int ret = 0;

	if (size > (UINTPTR_MAX - start)) {
		return -EPERM;
	}
	/* Treat a zero-length request like a one-byte one: still confirm addr's page is mapped. */
	last = ROUND_DOWN(start + ((size != 0U) ? (size - 1U) : 0U), CONFIG_MMU_PAGE_SIZE);

	/*
	 * Hold z_mem_domain_lock across the walk: trap0 re-enables guest
	 * interrupts, so a partition add/remove could otherwise change the
	 * domain under it.
	 */
	key = k_spin_lock(&z_mem_domain_lock);

	if (thread->arch.priv_level == 0 || thread->mem_domain_info.mem_domain == NULL) {
		ret = hexagon_supervisor_buffer_ok(thread, start, size, write) ? 0 : -EPERM;
		goto out;
	}

	struct k_mem_domain *domain = thread->mem_domain_info.mem_domain;

	for (uintptr_t va = ROUND_DOWN(start, CONFIG_MMU_PAGE_SIZE);; va += CONFIG_MMU_PAGE_SIZE) {
		uint32_t pte = hexagon_domain_pte(domain, thread, va, -1);

		if ((pte & need) != need) {
			ret = -EPERM;
			break;
		}
		if (va == last) {
			break;
		}
	}

out:
	k_spin_unlock(&z_mem_domain_lock, key);
	return ret;
}

size_t arch_user_string_nlen(const char *s, size_t maxsize, int *err_arg)
{
	/*
	 * A zero-length request must never touch *s, matching
	 * strnlen(s, 0)'s contract of accepting an arbitrary pointer.
	 */
	if (maxsize == 0) {
		*err_arg = 0;
		return 0;
	}

	if (arch_buffer_validate(s, maxsize, 0)) {
		*err_arg = -1;
		return 0;
	}

	*err_arg = 0;
	return strnlen(s, maxsize);
}

/*
 * __naked comes from picolibc's sys/cdefs.h, so it silently disappears on
 * any other libc (e.g. CONFIG_MINIMAL_LIBC). Define it locally instead.
 */
#ifndef __naked
#define __naked __attribute__((naked))
#endif

/*
 * Guest register usage for vmrte:
 *   G0 = GELR (entry point)
 *   G1 = GSR (bit 31 = user mode, bit 30 = IE)
 *   G2 = GOSP (user stack pointer)
 *   G3 = GBADVA (0 for normal entry)
 */
static void __used __naked hexagon_user_thread_exit(void)
{
	/*
	 * User function returned -- call k_thread_abort(self) via explicit
	 * trap0 syscalls; a C wrapper's user-mode check could be optimized
	 * away. Syscall convention: r0 = arg/return value, r6 = syscall number.
	 *
	 * This runs in user mode, so ask the kernel for the current thread
	 * rather than reading _kernel directly.
	 */
	__asm__ volatile(
		/* syscall: r0 = k_sched_current_thread_query() */
		"r6 = #%[query_id]\n\t"
		"trap0(#0x1)\n\t"
		/* syscall: k_thread_abort(r0) */
		"r6 = #%[abort_id]\n\t"
		"trap0(#0x1)\n\t"
		/* should not return -- loop as backstop */
		"1: jump 1b\n\t"
		:
		: [query_id] "i"(K_SYSCALL_K_SCHED_CURRENT_THREAD_QUERY),
		  [abort_id] "i"(K_SYSCALL_K_THREAD_ABORT)
		:
	);
}

void arch_user_mode_enter(k_thread_entry_t user_entry, void *p1, void *p2, void *p3)
{
	/*
	 * Not k_current_get(): k_thread_user_mode_enter() just called
	 * arch_tls_stack_setup(), which zeroes z_tls_current along with the
	 * rest of .tbss, and ugp is not reloaded until the vmrte below.
	 */
	struct k_thread *thread = k_sched_current_thread_query();

	/*
	 * Events taken from user mode run on priv_stack, so the user stack
	 * never needs its reserved area. Hand it back to the stack buffer:
	 * the map rounds the stack to whole pages, so leaving it out of
	 * stack_info would not keep user mode out of it anyway.
	 */
	thread->stack_info.start -= K_THREAD_STACK_RESERVED;
	thread->stack_info.size += K_THREAD_STACK_RESERVED;

	/*
	 * stack_info.delta reserves the TLS block at the top of the stack
	 * buffer; without subtracting it, user_sp would land inside the TLS
	 * block arch_tls_stack_setup() just populated.
	 */
	uintptr_t user_sp = thread->stack_info.start + thread->stack_info.size -
			    thread->stack_info.delta;

	user_sp = ROUND_DOWN(user_sp, ARCH_STACK_PTR_ALIGN);

	__ASSERT(user_sp >= thread->stack_info.start,
		 "declared stack (%zu bytes) too small to hold TLS/headroom (%zu bytes)",
		 thread->stack_info.size, thread->stack_info.delta);

	/*
	 * GOSP needs its own per-thread stack, separate from both the
	 * current call frame (overwritten almost immediately) and a single
	 * shared address (a preempted thread can resume onto it mid-syscall,
	 * reproduced with test_syscall_switch_stress). priv_stack is a fixed
	 * CONFIG_PRIVILEGED_STACK_SIZE-byte array in the TCB, the same
	 * approach RISC-V/ARM/ARC/x86/xtensa use for their privileged
	 * stacks.
	 */
	uintptr_t kernel_sp = ROUND_DOWN((uintptr_t)thread->arch.priv_stack +
					 sizeof(thread->arch.priv_stack),
					 ARCH_STACK_PTR_ALIGN);

	/*
	 * Zero only the unused portion below the live C call chain, on this
	 * thread's own stack. Stop 8 bytes short of SP: memset() opens its
	 * own leaf frame there before writing anything.
	 */
	uintptr_t stack_ptr;

	__asm__ volatile("%0 = r29" : "=r"(stack_ptr));

	/*
	 * Guard against underflow for an unusually small stack combined
	 * with a deep call chain before this point; skip the clear instead
	 * of memset() past the stack buffer.
	 */
	if (stack_ptr > thread->stack_info.start + 8) {
		memset((void *)thread->stack_info.start, 0,
		       stack_ptr - 8 - thread->stack_info.start);
	}

	/*
	 * arch_tls_stack_setup() zeroed z_tls_current along with the rest
	 * of .tbss; ugp keeps pointing at this block, so restore it now for
	 * this thread's first syscall onward.
	 */
#ifdef CONFIG_CURRENT_THREAD_USE_TLS
	extern Z_THREAD_LOCAL k_tid_t z_tls_current;

	z_tls_current = thread;
#endif

	/*
	 * Record the drop to user mode so z_hexagon_user_mode_sync() keeps
	 * re-deriving _hexagon_user_mode_active as 1 on every later event
	 * exit, even one unrelated to this thread.
	 */
	thread->arch.priv_level = 1;

	/*
	 * Install this thread's memory domain table before the privilege
	 * drop below: the first user-mode access must already be bound by
	 * it.
	 */
	z_hexagon_mem_domain_switch();

	/*
	 * Set the thread-local flag now so arch_is_user_context() is already
	 * correct right after vmrte, before the first trap0 fires.
	 */
	_hexagon_user_mode_active = 1;

#ifdef CONFIG_HW_STACK_PROTECTION
	/*
	 * Once this thread's FRAMELIMIT is priv_stack's, a switch away and
	 * back would load it while SP is still on this stack; keep
	 * interrupts off until vmrte, which re-enables them from GSR.IE.
	 */
	(void)arch_irq_lock();
	z_hexagon_stack_protect_user_enter(thread);
#endif

	/*
	 * H2 vmrte with GSSR.UM swaps r29 <-> GOSP.  To end up with
	 * r29=user_sp in user mode, set:
	 *   r29 = kernel_sp (will become gosp after swap)
	 *   GOSP (g2) = user_sp (will become r29 after swap)
	 */
	__asm__ volatile(
		"r4 = %[entry]\n\t"
		"r5 = %[vmest]\n\t"
		"r6 = %[stack]\n\t"      /* GOSP = user SP (becomes r29) */
		"r7 = #0\n\t"
		"g0 = r4\n\t"            /* GELR = user entry */
		"g1 = r5\n\t"            /* GSR = UM + IE */
		"g2 = r6\n\t"            /* GOSP = user SP */
		"g3 = r7\n\t"            /* GBADVA = 0 */
		"r0 = %[p1]\n\t"
		"r1 = %[p2]\n\t"
		"r2 = %[p3]\n\t"
		"r29 = %[ksp]\n\t"       /* kernel SP (becomes gosp) */
		"r31 = %[exit_fn]\n\t"   /* LR = user thread exit stub */
		"r30 = #0\n\t"           /* FP = 0 (no parent frame) */
		"trap1(#1)\n\t"          /* vmrte */
		:
		: [entry] "r"((uintptr_t)user_entry),
		  [vmest] "r"((uint32_t)0xC0000000), /* User mode + IE */
		  [ksp] "r"(kernel_sp),
		  [stack] "r"(user_sp),
		  [p1] "r"(p1),
		  [p2] "r"(p2),
		  [p3] "r"(p3),
		  [exit_fn] "r"((uintptr_t)hexagon_user_thread_exit)
		: "r0", "r1", "r2", "r4", "r5", "r6", "r7",
		  "r29", "r30", "r31", "memory"
	);

	CODE_UNREACHABLE;
}

void arch_syscall_invoke(uint32_t syscall_id, uint32_t arg1, uint32_t arg2, uint32_t arg3,
			 uint32_t arg4, uint32_t arg5, uint32_t arg6, struct arch_esf *esf)
{
	if (syscall_id >= K_SYSCALL_LIMIT) {
		esf->r0 = -ENOSYS;
		return;
	}

	esf->r0 = (uint32_t)_k_syscall_table[syscall_id](
		arg1, arg2, arg3, arg4, arg5, arg6, esf);
}

void z_hexagon_syscall_handler(struct arch_esf *esf)
{
	uint32_t syscall_id = esf->r6;

	arch_syscall_invoke(syscall_id, esf->r0, esf->r1, esf->r2, esf->r3, esf->r4, esf->r5, esf);
}

FUNC_NORETURN void arch_syscall_oops(void *ssf)
{
	struct arch_esf *esf = (struct arch_esf *)ssf;

	z_fatal_error(K_ERR_KERNEL_OOPS, esf);
	CODE_UNREACHABLE;
}

void z_impl_hexagon_user_fault(unsigned int reason)
{
	if (reason != K_ERR_STACK_CHK_FAIL) {
		reason = K_ERR_KERNEL_OOPS;
	}
	z_fatal_error(reason, NULL);
	CODE_UNREACHABLE;
}

static void z_vrfy_hexagon_user_fault(unsigned int reason)
{
	z_impl_hexagon_user_fault(reason);
}
#include <zephyr/syscalls/hexagon_user_fault_mrsh.c>

int arch_mem_domain_max_partitions_get(void)
{
	return CONFIG_MAX_DOMAIN_PARTITIONS;
}

/*
 * A partition, unlike a stack, is never rounded out: that would grant
 * user access to whatever shares its first and last pages. It must also
 * lie in KERNEL_VM, the only range with per-domain L2 tables.
 */
static bool hexagon_partition_is_placed(const struct k_mem_partition *part)
{
	return ((part->start | part->size) & (CONFIG_MMU_PAGE_SIZE - 1U)) == 0U &&
	       part->start >= HEXAGON_MMU_VM_START &&
	       part->size <= HEXAGON_MMU_VM_END - part->start;
}

#define HEXAGON_PARTITION_IS_PRIV_WRITABLE(attr) ((attr) & 0x08U) /* bit 3 */
#define HEXAGON_PARTITION_IS_PRIV_READABLE(attr) ((attr) & 0x18U) /* bits 3-4 */

/*
 * A user-accessible partition's PTE binds the kernel too, see
 * hexagon_partition_perms(), so the kernel cannot be given write access
 * the user lacks. Such a partition is rejected and never mapped.
 */
static bool hexagon_partition_attr_is_supported(k_mem_partition_attr_t attr)
{
	if (!K_MEM_PARTITION_IS_READABLE(attr) && !K_MEM_PARTITION_IS_WRITABLE(attr)) {
		return true;
	}

	return !HEXAGON_PARTITION_IS_PRIV_WRITABLE(attr) || K_MEM_PARTITION_IS_WRITABLE(attr);
}

static bool hexagon_partition_is_mappable(const struct k_mem_partition *part)
{
	return hexagon_partition_is_placed(part) &&
	       hexagon_partition_attr_is_supported(part->attr);
}

/*
 * HVM has one R/W/X set per page plus U. Guest mode ignores U but enforces
 * R/W/X, so ISR and syscall accesses obey the same permissions.
 *
 * Partition permissions map as follows (X always follows the partition):
 *
 *   Partition type     U   R/W bits come from
 *   User-accessible    1   User permissions
 *   No user access     0   Privileged permissions
 */
static uint32_t hexagon_partition_perms(k_mem_partition_attr_t attr)
{
	uint32_t perms = 0U;

	if (K_MEM_PARTITION_IS_READABLE(attr) || K_MEM_PARTITION_IS_WRITABLE(attr)) {
		perms |= __HVM_PTE_U;
		if (K_MEM_PARTITION_IS_READABLE(attr)) {
			perms |= __HVM_PTE_R;
		}
		if (K_MEM_PARTITION_IS_WRITABLE(attr)) {
			perms |= __HVM_PTE_W;
		}
	} else {
		if (HEXAGON_PARTITION_IS_PRIV_READABLE(attr)) {
			perms |= __HVM_PTE_R;
		}
		if (HEXAGON_PARTITION_IS_PRIV_WRITABLE(attr)) {
			perms |= __HVM_PTE_W;
		}
	}
	if (K_MEM_PARTITION_IS_EXECUTABLE(attr)) {
		perms |= __HVM_PTE_X;
	}

	return perms;
}

BUILD_ASSERT(!IS_ENABLED(CONFIG_SMP),
	     "Hexagon tracks one installed page table and stack owner: SMP is not supported");

/* Domain whose table is installed. NULL means hexagon_kernel_l1 is. */
static struct k_mem_domain *hexagon_installed_domain;

/* Every initialized domain, to propagate kernel table changes. */
static sys_slist_t hexagon_domains;

/*
 * Bumped whenever a translation is removed or changed. vmclrmap() only
 * flushes the installed table's ASID; a table whose generation lags
 * behind invalidates its ASID on its next install.
 */
static uint32_t hexagon_map_gen;
static uint32_t hexagon_kernel_map_gen;

void hexagon_map_flushed(void)
{
	hexagon_map_gen++;
	if (hexagon_installed_domain == NULL) {
		hexagon_kernel_map_gen = hexagon_map_gen;
	} else {
		hexagon_installed_domain->arch.gen = hexagon_map_gen;
	}
}

static void hexagon_table_install(uint32_t *l1, uint32_t *gen, bool stale)
{
	stale = stale || *gen != hexagon_map_gen;
	if (hexagon_vm_newmap(l1, VM_TRANS_TYPE_TABLE,
			      stale ? VM_TLB_INVALIDATE_TRUE : VM_TLB_INVALIDATE_FALSE) != 0) {
		k_panic();
	}
	*gen = hexagon_map_gen;
}

static void hexagon_kernel_table_install(void)
{
	if (hexagon_installed_domain != NULL) {
		hexagon_table_install(hexagon_kernel_l1, &hexagon_kernel_map_gen, false);
		hexagon_installed_domain = NULL;
	}
}

static void hexagon_domain_table_install(struct k_mem_domain *domain)
{
	hexagon_table_install(domain->arch.l1, &domain->arch.gen, domain->arch.stale);
	domain->arch.stale = false;
	hexagon_installed_domain = domain;
}

static bool hexagon_thread_stack_contains(const struct k_thread *thread, uintptr_t va)
{
	uintptr_t start = ROUND_DOWN(thread->stack_info.start, CONFIG_MMU_PAGE_SIZE);
	uintptr_t end = ROUND_UP(thread->stack_info.start + thread->stack_info.size,
				 CONFIG_MMU_PAGE_SIZE);

	return va >= start && va < end;
}

/*
 * What domain's table maps va to while owner runs in user mode: owner's
 * stack, else the first mappable partition other than skip_id, else the
 * kernel's PTE. Overrides keep the kernel's frame and cache attributes,
 * so a partition over unmapped memory stays unmapped.
 */
static uint32_t hexagon_domain_pte(const struct k_mem_domain *domain,
				   const struct k_thread *owner, uintptr_t va, int skip_id)
{
	uint32_t pte = hexagon_kernel_pte_get(va);
	int remaining = domain->num_partitions;

	if (pte == 0U || !hexagon_va_in_vm(va)) {
		return pte;
	}

	if (owner != NULL && hexagon_thread_stack_contains(owner, va)) {
		return (pte & ~HEXAGON_PTE_PERM_MASK) | __HVM_PTE_R | __HVM_PTE_W | __HVM_PTE_U;
	}

	for (int i = 0; remaining > 0 && i < CONFIG_MAX_DOMAIN_PARTITIONS; i++) {
		const struct k_mem_partition *part = &domain->partitions[i];

		if (part->size == 0U) {
			continue;
		}
		remaining--;
		if (i == skip_id || !hexagon_partition_is_mappable(part) || va < part->start ||
		    va - part->start >= part->size) {
			continue;
		}

		return (pte & ~HEXAGON_PTE_PERM_MASK) | hexagon_partition_perms(part->attr);
	}

	return pte;
}

/*
 * Recompute domain's PTEs over the KERNEL_VM part of [start, start +
 * size). A changed translation leaves the TLB now if the table is
 * installed, else on its next install.
 *
 * Called with z_mem_domain_lock held.
 */
static void hexagon_domain_update(struct k_mem_domain *domain, uintptr_t start, size_t size,
				  int skip_id)
{
	uintptr_t va = MAX(ROUND_DOWN(start, CONFIG_MMU_PAGE_SIZE), HEXAGON_MMU_VM_START);
	uintptr_t end = HEXAGON_MMU_VM_END;

	if (start < end && size < end - start) {
		end = ROUND_UP(start + size, CONFIG_MMU_PAGE_SIZE);
	}

	for (; va < end; va += CONFIG_MMU_PAGE_SIZE) {
		uint32_t *pte = &domain->arch.l2[(va - HEXAGON_MMU_VM_START) >>
						  HEXAGON_MMU_SLOT_SHIFT][hexagon_pt_index(va)];
		uint32_t new = hexagon_domain_pte(domain, domain->arch.stack_owner, va, skip_id);
		uint32_t old = *pte;

		if (new == old) {
			continue;
		}
		*pte = new;
		if (old == 0U) {
			continue;
		}
		if (hexagon_installed_domain == domain) {
			barrier_dmem_fence_full();
			(void)hexagon_vm_clrmap((void *)va, 1);
		} else {
			domain->arch.stale = true;
		}
	}
}

static void hexagon_domain_update_stack(struct k_mem_domain *domain,
					const struct k_thread *thread)
{
	hexagon_domain_update(domain, thread->stack_info.start, thread->stack_info.size, -1);
}

void hexagon_mem_domain_sync_l1(uint32_t slot)
{
	struct arch_mem_domain *arch;

	SYS_SLIST_FOR_EACH_CONTAINER(&hexagon_domains, arch, node) {
		arch->l1[slot] = hexagon_kernel_l1[slot];
	}
}

void hexagon_mem_domain_sync_page(uintptr_t va)
{
	struct arch_mem_domain *arch;

	SYS_SLIST_FOR_EACH_CONTAINER(&hexagon_domains, arch, node) {
		hexagon_domain_update(CONTAINER_OF(arch, struct k_mem_domain, arch), va,
				      CONFIG_MMU_PAGE_SIZE, -1);
	}
}

/*
 * Copy the kernel table: share its L2 tables and blocks outside
 * KERNEL_VM, and give KERNEL_VM private L2 tables. The core adds the
 * initial partitions next.
 */
int arch_mem_domain_init(struct k_mem_domain *domain)
{
	struct arch_mem_domain *arch = &domain->arch;

	(void)sys_slist_find_and_remove(&hexagon_domains, &arch->node);
	if (hexagon_installed_domain == domain) {
		hexagon_kernel_table_install();
	}

	(void)memcpy(arch->l1, hexagon_kernel_l1, sizeof(arch->l1));
	for (uint32_t i = 0; i < HEXAGON_MMU_VM_SLOTS; i++) {
		uint32_t slot = HEXAGON_MMU_VM_SLOT_FIRST + i;

		(void)memcpy(arch->l2[i], hexagon_pde_l2(hexagon_kernel_l1[slot]),
			     sizeof(arch->l2[i]));
		arch->l1[slot] = (uint32_t)(uintptr_t)arch->l2[i] | __HVM_PDE_S_4KB;
	}
	arch->stack_owner = NULL;
	/* HVM keys ASIDs on the table address: drop any earlier use of it. */
	arch->stale = true;

	sys_slist_append(&hexagon_domains, &arch->node);
	return 0;
}

int arch_mem_domain_deinit(struct k_mem_domain *domain)
{
	if (hexagon_installed_domain == domain) {
		hexagon_kernel_table_install();
	}
	(void)sys_slist_find_and_remove(&hexagon_domains, &domain->arch.node);
	return 0;
}

int arch_mem_domain_partition_add(struct k_mem_domain *domain, uint32_t partition_id)
{
	const struct k_mem_partition *part = &domain->partitions[partition_id];

	/* The core has already recorded it. Updates skip it from now on. */
	if (!hexagon_partition_is_placed(part)) {
		LOG_ERR("mem domain: partition %#lx/%#zx is not page-aligned in KERNEL_VM",
			(unsigned long)part->start, part->size);
		__ASSERT(false, "partition is not page-aligned in KERNEL_VM");
		return -EINVAL;
	}

	if (!hexagon_partition_attr_is_supported(part->attr)) {
		LOG_ERR("mem domain: partition %#lx/%#zx attr %#x gives priv more than user",
			(unsigned long)part->start, part->size, part->attr);
		__ASSERT(false, "partition attr is not supported");
		return -EINVAL;
	}

	hexagon_domain_update(domain, part->start, part->size, -1);
	return 0;
}

int arch_mem_domain_partition_remove(struct k_mem_domain *domain, uint32_t partition_id)
{
	const struct k_mem_partition *part = &domain->partitions[partition_id];

	/* Called before the core clears partitions[partition_id]. */
	if (hexagon_partition_is_mappable(part)) {
		hexagon_domain_update(domain, part->start, part->size, (int)partition_id);
	}
	return 0;
}

/* z_hexagon_mem_domain_switch() maps the stack once the thread runs. */
int arch_mem_domain_thread_add(struct k_thread *thread)
{
	ARG_UNUSED(thread);
	return 0;
}

/* thread->mem_domain_info.mem_domain still names the domain it leaves. */
int arch_mem_domain_thread_remove(struct k_thread *thread)
{
	struct k_mem_domain *domain = thread->mem_domain_info.mem_domain;

	if (domain != NULL && domain->arch.stack_owner == thread) {
		domain->arch.stack_owner = NULL;
		hexagon_domain_update_stack(domain, thread);
	}
	return 0;
}

/*
 * Install the page table matching _current: called at every point
 * Hexagon already reloads other per-thread hardware state on switch-in
 * (z_hexagon_thread_start, EVENT_EXIT) and from arch_user_mode_enter()
 * for the first drop to user mode.
 *
 * Threads of one domain share its table, and only the stack of the one
 * that runs is mapped U=1: switching between them moves that mapping.
 */
void z_hexagon_mem_domain_switch(void)
{
	struct k_thread *thread = k_sched_current_thread_query();
	struct k_mem_domain *domain = thread->mem_domain_info.mem_domain;
	struct k_thread *old_owner;
	k_spinlock_key_t key;

	if (thread->arch.priv_level == 0 || domain == NULL) {
		hexagon_kernel_table_install();
		return;
	}

	if (hexagon_installed_domain == domain && domain->arch.stack_owner == thread) {
		return;
	}

	key = k_spin_lock(&z_mem_domain_lock);
	if (hexagon_installed_domain != domain) {
		hexagon_domain_table_install(domain);
	}

	old_owner = domain->arch.stack_owner;
	if (old_owner != thread) {
		domain->arch.stack_owner = thread;
		if (old_owner != NULL) {
			hexagon_domain_update_stack(domain, old_owner);
		}
		hexagon_domain_update_stack(domain, thread);
	}
	k_spin_unlock(&z_mem_domain_lock, key);
}

#endif /* CONFIG_USERSPACE */
