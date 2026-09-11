/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Hexagon userspace support
 */

#include <zephyr/kernel.h>
#include <zephyr/arch/hexagon/arch.h>
#include <zephyr/internal/syscall_handler.h>
#include <zephyr/linker/linker-defs.h>
#include <kernel_internal.h>
#include <hexagon_vm.h>
#include <offsets_short.h>

#ifdef CONFIG_USERSPACE

int arch_buffer_validate(const void *addr, size_t size, int write)
{
	struct k_thread *thread = k_current_get();
	uintptr_t start = (uintptr_t)addr;
	uintptr_t end;

	/*
	 * Deliberately no arch_is_user_context() shortcut here: every real
	 * caller -- K_SYSCALL_MEMORY_WRITE/READ from a z_vrfy_ handler, or
	 * check_perms()'s own z_impl_ -- runs from inside
	 * z_hexagon_event_handler()'s trap0 path, which unconditionally
	 * clears _hexagon_user_mode_active on entry so kernel code sees
	 * itself as kernel. A prior version of this function returned 0
	 * ("allowed") whenever that read false, which given the above is
	 * every single call -- it happened to look correct for in-bounds
	 * accesses and silently skipped real validation for everything
	 * else. RISC-V's and ARM's arch_buffer_validate() have no such
	 * shortcut either: this always validates against the calling
	 * (_current) thread's own bounds, regardless of what mode the CPU
	 * is flagged as right now.
	 */
	if (size > (UINTPTR_MAX - start)) {
		return -EPERM;
	}
	end = start + size;

	/* Check thread stack */
	if (start >= thread->stack_info.start &&
	    end <= thread->stack_info.start + thread->stack_info.size) {
		return 0; /* Within thread stack */
	}

	/*
	 * Global read-only data (rodata/text) is not part of any thread's
	 * stack or memory-domain partition, but string literals and other
	 * const data passed to syscalls (e.g. k_usermode_string_copy()) come
	 * from there constantly and must be readable without an explicit
	 * grant. Matches RISC-V's and ARM's arch_buffer_validate().
	 */
	if (!write) {
		uintptr_t ro_start = (uintptr_t)__rom_region_start;
		uintptr_t ro_end = (uintptr_t)__rom_region_end;

		if (ro_end > ro_start && start >= ro_start && end <= ro_end) {
			return 0;
		}

		uintptr_t rodata_start = (uintptr_t)__rodata_region_start;
		uintptr_t rodata_end = (uintptr_t)__rodata_region_end;

		if (rodata_end > rodata_start && start >= rodata_start && end <= rodata_end) {
			return 0;
		}
	}

	/* Check memory domain partitions */
	if (thread->mem_domain_info.mem_domain != NULL) {
		struct k_mem_domain *domain = thread->mem_domain_info.mem_domain;
		int remaining_partitions;
		k_spinlock_key_t key;

		/*
		 * z_mem_domain_lock also guards domain->partitions[]/
		 * num_partitions against k_mem_domain_add_partition()/
		 * remove_partition() (kernel/userspace/mem_domain.c): trap0
		 * handling re-enables guest interrupts, so a timer tick can
		 * preempt this scan mid-loop and let another thread mutate
		 * the same domain before it resumes.
		 */
		key = k_spin_lock(&z_mem_domain_lock);
		remaining_partitions = domain->num_partitions;

		/*
		 * partitions[] can have unused holes (size == 0) left behind
		 * by a removed partition -- num_partitions counts only the
		 * active ones, so scan the whole array and skip holes rather
		 * than assuming the first num_partitions entries are all
		 * valid. Matches RISC-V's arch_buffer_validate().
		 */
		for (int i = 0; remaining_partitions > 0 && i < CONFIG_MAX_DOMAIN_PARTITIONS;
		     i++) {
			const struct k_mem_partition *part = &domain->partitions[i];

			if (part->size == 0) {
				continue;
			}
			remaining_partitions--;

			uintptr_t part_start = part->start;
			uintptr_t part_end = part_start + part->size;

			if (start < part_start || end > part_end) {
				continue;
			}

			/* For a write access, the partition must be writable */
			if (write && !K_MEM_PARTITION_IS_WRITABLE(part->attr)) {
				continue;
			}

			k_spin_unlock(&z_mem_domain_lock, key);
			return 0; /* Within a valid partition */
		}

		k_spin_unlock(&z_mem_domain_lock, key);
	}

	return -EPERM;
}

size_t arch_user_string_nlen(const char *s, size_t maxsize, int *err_arg)
{
	/*
	 * A zero-length request must never touch *s -- callers rely on this
	 * to probe with an arbitrary (possibly invalid) pointer, matching
	 * strnlen(s, 0)'s contract. arch_buffer_validate(s, 0, 0) would
	 * otherwise still reject an address outside the thread's stack,
	 * global rodata, and domain partitions.
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
 * __naked isn't defined by any of Zephyr's own toolchain headers (no other
 * arch uses it) -- it only happens to exist because picolibc's
 * sys/cdefs.h defines it, so it silently disappears (parsed as a stray
 * identifier, not a missing macro) on any other libc, e.g.
 * CONFIG_MINIMAL_LIBC. Define it locally so this doesn't depend on which
 * libc a given build happens to select.
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
	 * User function returned -- call k_thread_abort(self) via
	 * explicit trap0 syscall.  We cannot use the C wrapper because
	 * the compiler may optimize away the user-mode check.
	 *
	 * Syscall convention: r0 = arg (thread), r6 = syscall number.
	 * k_current_get() is just a memory read (no privilege needed).
	 */
	__asm__ volatile(
		/* r0 = _kernel.cpus[0].current (k_current_get) */
		"r0 = ##_kernel\n\t"
		"r0 = add(r0, #%[cpus_off])\n\t"
		"r0 = memw(r0+#%[cur_off])\n\t"
		/* syscall: k_thread_abort(r0) */
		"r6 = #%[sc_id]\n\t"
		"trap0(#0x1)\n\t"
		/* should not return -- loop as backstop */
		"1: jump 1b\n\t"
		:
		: [cpus_off] "i"(___kernel_t_cpus_OFFSET),
		  [cur_off] "i"(___cpu_t_current_OFFSET),
		  [sc_id] "i"(K_SYSCALL_K_THREAD_ABORT)
		:
	);
}

void arch_user_mode_enter(k_thread_entry_t user_entry, void *p1, void *p2, void *p3)
{
	/*
	 * Not k_current_get(): that reads the CONFIG_CURRENT_THREAD_USE_TLS
	 * cache (z_tls_current, at a fixed ugp-relative offset), which
	 * k_thread_user_mode_enter() -- the sole caller -- has just made
	 * stale. It called arch_tls_stack_setup() to lay out this thread's
	 * user-mode TLS block right before this call, and that recopies the
	 * whole .tdata/.tbss template into place, zeroing z_tls_current
	 * along with the rest of .tbss. ugp itself will not be reloaded
	 * from the new block until the vmrte below, so z_tls_current stays
	 * stale (read as NULL) for any call made between the two.
	 */
	struct k_thread *thread = k_sched_current_thread_query();

	/*
	 * stack_info.delta reserves the TLS block (and any other
	 * per-thread headroom) at the top of the stack buffer; the
	 * generic (start + size - delta) is the initial stack pointer,
	 * documented in struct _stack_info. Without subtracting delta,
	 * user_sp would start inside the TLS block that
	 * k_thread_user_mode_enter() just populated via
	 * arch_tls_stack_setup(), and the user thread's own first pushes
	 * would immediately overwrite it.
	 *
	 * Nothing else needs to be reserved here: unlike an earlier version
	 * of this port, the kernel-side event-handling stack (kernel_sp,
	 * below) is no longer carved out of this same buffer -- the whole
	 * declared stack, minus only the TLS headroom, belongs to user code.
	 */
	uintptr_t user_sp = thread->stack_info.start + thread->stack_info.size -
			    thread->stack_info.delta;

	user_sp = ROUND_DOWN(user_sp, ARCH_STACK_PTR_ALIGN);

	__ASSERT(user_sp >= thread->stack_info.start,
		 "declared stack (%zu bytes) too small to hold TLS/headroom (%zu bytes)",
		 thread->stack_info.size, thread->stack_info.delta);

	/*
	 * Save a kernel SP as GOSP.  When H2 delivers an event from user
	 * mode (trap0 syscall or interrupt), it swaps r29 with GOSP --
	 * landing the kernel event handler on this kernel stack rather than
	 * the user stack.
	 *
	 * This must be a genuinely separate, adequately-sized stack, not a
	 * point picked out of the current C call chain: __builtin_frame_address(0)
	 * once returned an address only a few dozen bytes below user_sp
	 * (however deep the arch_tls_stack_setup()/arch_user_mode_enter()
	 * call chain happened to be), which the user thread's own stack
	 * usage grows past almost immediately -- EVENT_ENTRY's allocframe
	 * and register-context save then land on top of the user thread's
	 * live frames, corrupting saved return addresses, instead of below
	 * them as intended.
	 *
	 * It also must be *per-thread*, not one address shared system-wide:
	 * a shared address (the per-CPU IRQ stack top, tried as the second
	 * attempt) is only safe while a thread's entire event round trip -- entry,
	 * handler, vmrte back to user mode -- completes before any other
	 * thread's own round trip starts at that same address. That does
	 * not hold once more than one K_USER thread exists: trap0 handling
	 * deliberately re-enables interrupts so blocking syscalls can
	 * z_swap() (see z_hexagon_event_handler()), so the scheduler can
	 * switch away from a thread mid-syscall, before its EVENT_EXIT has
	 * unwound. A second thread's own event entry then lands on the
	 * shared address while the first thread's saved context is still
	 * live there, corrupting it. Reproduced with
	 * tests/kernel/mem_protect/syscalls's test_syscall_switch_stress
	 * (multiple K_USER threads making syscalls concurrently).
	 *
	 * A third attempt carved a fixed-size reserve out of the top of
	 * *this* thread's own declared stack, below the TLS block: genuinely
	 * per-thread, but competing with the thread's own declared stack
	 * size for space. That silently broke any K_USER thread whose
	 * declared stack was close to or smaller than the reserve --
	 * CONFIG_DYNAMIC_THREAD_STACK_SIZE's 1024-byte default, for one --
	 * and even a size comfortably larger than the reserve wasn't
	 * necessarily safe for every syscall: some syscalls' kernel-side C
	 * call depth exceeded whatever fixed size was picked, silently
	 * overrunning it and corrupting memory below (found live with
	 * LOG_PRINTK's syscall path -- deeper than a plain string-arg one).
	 *
	 * Both problems trace back to the same cause: sizing this stack
	 * against the thread's own declared (and highly variable, often
	 * deliberately small) stack size at all. thread->arch.priv_stack is
	 * a fixed CONFIG_PRIVILEGED_STACK_SIZE-byte array living in the
	 * thread control block itself -- entirely decoupled from the
	 * thread's declared stack, exactly like the separate privileged
	 * stack RISC-V/ARM/ARC/x86/xtensa give every K_USER thread (see
	 * CONFIG_PRIVILEGED_STACK_SIZE's help text: "used in addition to the
	 * user mode thread stack"). A thread's declared stack size no longer
	 * has any bearing on how much kernel-side headroom it gets.
	 */
	uintptr_t kernel_sp = ROUND_DOWN((uintptr_t)thread->arch.priv_stack +
					 sizeof(thread->arch.priv_stack),
					 ARCH_STACK_PTR_ALIGN);

	/*
	 * Zero only the unused portion of the stack, below where this
	 * function itself is running: k_thread_user_mode_enter() is called
	 * directly from a thread's own body, on that thread's own stack, so
	 * the C call chain up to and including this frame is live. Zeroing
	 * the full [stack_info.start, user_sp) range as the other Zephyr
	 * architectures' equivalent K_INIT_STACKS pass do (they run on a
	 * separate privileged stack, so their target range is never the
	 * stack they are executing on) would overwrite that live chain with
	 * the memset() fill value.
	 *
	 * kernel_sp (r30, the frame pointer after allocframe) is NOT a safe
	 * lower bound for that: allocframe places the new FP:LR save slot
	 * at the top of the frame and moves SP (r29) below it, so this
	 * frame's own locals live in [r29, r30), below kernel_sp.
	 *
	 * The current stack pointer is not quite enough either: memset()
	 * itself opens an 8-byte leaf frame (its own FP:LR save slot) at
	 * [r29-8, r29) on entry, using this function's r29 as its caller
	 * SP, before it writes a single byte. Asking it to zero all the way
	 * up to r29 makes its own fill pass overwrite that slot; by the
	 * time the loop finishes and memset() returns, the return address
	 * it reads back is the zero it just wrote. Stop 8 bytes short of
	 * r29 to leave that slot alone.
	 */
	uintptr_t stack_ptr;

	__asm__ volatile("%0 = r29" : "=r"(stack_ptr));

	/*
	 * stack_ptr - 8 - stack_info.start underflows (size_t, wraps to a
	 * huge value) unless the live SP is actually above stack_info.start
	 * + 8. It always has been in practice -- k_thread_user_mode_enter()
	 * is called early in the user thread's own body -- but nothing
	 * guarantees it for an unusually small declared stack combined with
	 * a deep call chain before this point; skip the clear rather than
	 * memset() far past the stack buffer in that case.
	 */
	if (stack_ptr > thread->stack_info.start + 8) {
		memset((void *)thread->stack_info.start, 0,
		       stack_ptr - 8 - thread->stack_info.start);
	}

	/*
	 * k_thread_user_mode_enter() called arch_tls_stack_setup() right
	 * before this function, which re-copies the whole .tdata/.tbss
	 * template into this thread's TLS block -- zeroing z_tls_current
	 * along with the rest of .tbss. ugp is not reloaded from that block
	 * until the vmrte below, and vmrte itself does not touch ugp either
	 * (the HVM event record it restores is GELR/GSR/GOSP/GBADVA only),
	 * so ugp keeps pointing at this same block afterwards. Restore
	 * z_tls_current now so that k_current_get() -- called by every
	 * syscall this user thread makes, starting with the very first one
	 * -- reads the right thread pointer instead of the zeroed template
	 * value.
	 */
#ifdef CONFIG_CURRENT_THREAD_USE_TLS
	extern Z_THREAD_LOCAL k_tid_t z_tls_current;

	z_tls_current = thread;
#endif

	/*
	 * Record the drop to user mode on the thread itself so that
	 * z_hexagon_user_mode_sync() -- called on every event exit, even
	 * one that has nothing to do with this thread, such as an
	 * interrupt serviced while it is _current -- keeps re-deriving
	 * _hexagon_user_mode_active as 1 for as long as this thread runs.
	 * Without this, the first unrelated event resets the flag to 0
	 * and every subsequent syscall this thread makes (starting with
	 * whichever one it calls first) skips the trap0 trampoline and
	 * runs privileged kernel code, such as UART MMIO access, directly
	 * from real hardware user privilege.
	 */
	thread->arch.priv_level = 1;

	/*
	 * Set the global flag now so that arch_is_user_context() returns
	 * true immediately after vmrte, before the first trap0 fires.
	 * z_hexagon_user_mode_sync() will keep it in sync on every
	 * subsequent kernel re-entry.
	 */
	_hexagon_user_mode_active = 1;

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

int arch_mem_domain_max_partitions_get(void)
{
	return CONFIG_MAX_DOMAIN_PARTITIONS;
}

int arch_mem_domain_init(struct k_mem_domain *domain)
{
	ARG_UNUSED(domain);
	return 0;
}

int arch_mem_domain_partition_add(struct k_mem_domain *domain, uint32_t partition_id)
{
	ARG_UNUSED(domain);
	ARG_UNUSED(partition_id);
	return 0;
}

int arch_mem_domain_partition_remove(struct k_mem_domain *domain, uint32_t partition_id)
{
	ARG_UNUSED(domain);
	ARG_UNUSED(partition_id);
	return 0;
}

void arch_mem_domain_thread_add(struct k_thread *thread)
{
	ARG_UNUSED(thread);
}

void arch_mem_domain_thread_remove(struct k_thread *thread)
{
	ARG_UNUSED(thread);
}

#endif /* CONFIG_USERSPACE */
