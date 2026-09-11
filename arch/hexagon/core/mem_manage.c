/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Hexagon MMU management via the H2 hypervisor page table interface.
 *
 * hexagon_mmu_init() builds a 2-level page table (a 1024-entry L1 "PGD"
 * plus a pool of 64KB-granularity L2 tables, see HEX_PAGE_SIZE below for
 * why 64KB) and activates it via the vmnewmap hypercall, replacing the
 * single-level, 4MB-superpage table that an earlier version of this port
 * used (a flat identity map with
 * one uniform R+W+X+U permission set covering all of code, rodata, data
 * and every thread's stack -- there was no way to tell the MMU that
 * .rodata/.text should not be writable, or that a stack should not be
 * executable).
 *
 * The code and rodata region (__rom_region_start/end) and the data/bss/
 * stacks region (from there through _image_ram_end) are mapped through
 * the L2 tables with different permissions: R+X (no W) for the former;
 * for the latter, R+W (no X) everywhere, further split (under
 * CONFIG_USERSPACE) into U and non-U sub-ranges along the
 * _app_smem/z_user_stacks boundaries already in
 * soc/qemu/hexagon/linker.ld -- see the comment in hexagon_mmu_init()
 * below. The device MMIO window and H2's own kernel region don't need
 * per-page granularity -- they keep the same direct, single L1-entry
 * 4MB mappings the previous flat map used, just built in C instead of
 * assembly.
 *
 * This *is* the only page table ever installed with H2 (built once, at
 * boot, before z_prep_c() runs) -- there is deliberately no second,
 * disjoint table: an earlier version of this file built and activated
 * one from arch_mem_map()/arch_mem_unmap(), which left the CPU walking
 * the boot table while vmclrmap() invalidated TLB state for a virtual
 * range whose only "mapping" lived in that unused second table,
 * corrupting unrelated code-page translations. CONFIG_KERNEL_DIRECT_MAP
 * still makes every arch_mem_map() request resolve virt == phys, and
 * the range asked for is always already covered by this same table, so
 * arch_mem_map()/arch_mem_unmap() remain no-ops.
 *
 * Known limitation, now narrower than it was: hexagon_pgd/hexagon_l2
 * below are ordinary static arrays with no K_APP_BMEM/DMEM tag, so
 * CONFIG_USERSPACE's kernel/user RAM split (again, see
 * hexagon_mmu_init()) now maps them non-U like any other kernel .bss --
 * user-mode code can no longer overwrite its own page tables directly.
 * arch_mem_domain_partition_add/remove() and
 * arch_mem_domain_thread_add/remove() (userspace.c) still remain no-ops,
 * but app-shared-memory partitions are genuinely gated by domain
 * membership: hexagon_mmu_sync_domain_access() below re-derives the
 * granted set from the resuming thread's own domain on every entry to
 * and resumption of user mode. A thread's own stack has no equivalent
 * per-domain gating and stays uniformly U-accessible once granted; see
 * hexagon_mmu_grant_user_stack()'s own comment for why that is an
 * accepted, narrower trade-off.
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/linker/linker-defs.h>
#include <zephyr/arch/hexagon/syscall.h>
#include <kernel_internal.h>
#include <hexagon_vm.h>

#ifdef CONFIG_USERSPACE
/* .hex_user_readable bounds (soc/qemu/hexagon/linker.ld) -- Hexagon-
 * specific, so not part of the generic zephyr/linker/linker-defs.h set.
 */
extern char z_hex_user_readable_start[];
extern char z_hex_user_readable_end[];
#endif

/* L1 (PGD) geometry: 1024 entries, each spanning 4MB of address space. */
#define HEX_PGD_ENTRIES  1024
#define HEX_PGDIR_SHIFT  22
#define HEX_PGDIR_SIZE   (1U << HEX_PGDIR_SHIFT)

/*
 * L2 page size for the ROM/RAM split. Must match the alignment the linker
 * script (soc/qemu/hexagon/linker.ld) actually uses for the boundary
 * between them -- HEXAGON_ROM_RAM_ALIGN there, kept in sync with the
 * 0x10000 here by the runtime check in hexagon_mmu_init() below, since a
 * BUILD_ASSERT can't see the linker script's value directly.
 *
 * This is deliberately its own constant, not CONFIG_MMU_PAGE_SIZE (4KB):
 * an earlier version of this file used that directly, which is correct
 * but not efficient. A single H2 second-level page table uses one page
 * size for every entry across the whole 4MB PGD entry it covers (HVM
 * spec S9.4), and this port's images are typically well under 4MB, so
 * ROM and RAM share one such table; at 4KB granularity that means a TLB
 * fill for every 4KB of code or data ever touched. Under heavy host CPU
 * contention (e.g. many qemu-system-hexagon instances running at once in
 * a test sweep) that was enough extra trap-and-refill overhead to make
 * some larger images time out that ran fine standalone -- reliably
 * reproduced at 4KB granularity, gone at 64KB. See HEXAGON_ROM_RAM_ALIGN
 * in the linker script for the full explanation.
 */
#define HEX_PAGE_SIZE       0x10000
#define HEX_PAGE_SHIFT      16
#define HEX_L2_ENTRIES      (HEX_PGDIR_SIZE / HEX_PAGE_SIZE)
#define HEX_L2_TABLE_BYTES  (HEX_L2_ENTRIES * sizeof(uint32_t))

BUILD_ASSERT(HEX_PAGE_SIZE == (1 << HEX_PAGE_SHIFT), "HEX_PAGE_SHIFT out of sync");
#define HEX_PDE_S __HVM_PDE_S_64KB /* keep in sync with HEX_PAGE_SIZE above */

/* PDE (Figure 9-3 in the HVM spec): L2 table address | S field only --
 * no permission bits at this level. L2 tables must be aligned to their
 * own size (256 bytes for 64 4-byte entries, at the 64KB HEX_PAGE_SIZE
 * above).
 */
#define HEX_PDE_ADDR_MASK   (~(HEX_L2_TABLE_BYTES - 1))

/*
 * Number of L2 tables to pre-allocate. Each covers 4MB of ROM+RAM. This
 * port's only board has 16MB of RAM total, so 8 (32MB) is already more
 * than could ever be needed; hexagon_mmu_init() asserts rather than
 * silently overrunning the pool if that ever changes.
 */
#define HEX_MAX_L2_TABLES 8

static uint32_t hexagon_pgd[HEX_PGD_ENTRIES] __aligned(4096);
static uint32_t hexagon_l2[HEX_MAX_L2_TABLES][HEX_L2_ENTRIES] __aligned(HEX_L2_TABLE_BYTES);
static uint32_t hexagon_l2_next;

static uint32_t *hexagon_l2_alloc(void)
{
	__ASSERT(hexagon_l2_next < HEX_MAX_L2_TABLES,
		 "hexagon L2 page table pool exhausted (%u/%u)",
		 hexagon_l2_next, HEX_MAX_L2_TABLES);

	return hexagon_l2[hexagon_l2_next++];
}

/* Get (allocating if necessary) the L2 table backing the 4MB PGD entry
 * that virt falls in.
 */
static uint32_t *hexagon_l2_table_for(uintptr_t virt)
{
	uint32_t pgd_idx = virt >> HEX_PGDIR_SHIFT;
	uint32_t pde = hexagon_pgd[pgd_idx];
	uint32_t *l2;

	if (pde == 0) {
		l2 = hexagon_l2_alloc();
		memset(l2, 0, HEX_L2_TABLE_BYTES);
		hexagon_pgd[pgd_idx] = (((uint32_t)(uintptr_t)l2) & HEX_PDE_ADDR_MASK) |
				       HEX_PDE_S;
	} else {
		l2 = (uint32_t *)(uintptr_t)(pde & HEX_PDE_ADDR_MASK);
	}

	return l2;
}

/* Map [virt_start, virt_end) (both HEX_PAGE_SIZE-aligned) identity
 * (phys == virt, per CONFIG_KERNEL_DIRECT_MAP) through the L2 tables
 * with the given permission and cache-attribute bits.
 */
static void hexagon_l2_map_range(uintptr_t virt_start, uintptr_t virt_end,
				 uint32_t pte_perm_bits, uint32_t cache_attr)
{
	uint32_t pte_flags = pte_perm_bits | (cache_attr << 6);
	uintptr_t va;

	__ASSERT((virt_start & (HEX_PAGE_SIZE - 1)) == 0,
		 "unaligned range start 0x%lx", (unsigned long)virt_start);
	__ASSERT((virt_end & (HEX_PAGE_SIZE - 1)) == 0,
		 "unaligned range end 0x%lx", (unsigned long)virt_end);

	for (va = virt_start; va < virt_end; va += HEX_PAGE_SIZE) {
		uint32_t *l2 = hexagon_l2_table_for(va);
		uint32_t idx = (va >> HEX_PAGE_SHIFT) & (HEX_L2_ENTRIES - 1);

		l2[idx] = (uint32_t)va | pte_flags;
	}
}

/* Set a single direct L1 PTE (Figure 9-4: S=4MB, permissions and cache
 * attributes embedded in the entry itself, no L2 table). phys must be
 * 4MB-aligned. Used for the device MMIO window and H2's own kernel
 * region, which don't need per-page permission splits.
 */
static void hexagon_l1_map_4mb(uintptr_t phys, uint32_t pte_perm_bits, uint32_t cache_attr)
{
	__ASSERT((phys & (HEX_PGDIR_SIZE - 1)) == 0,
		 "unaligned 4MB mapping 0x%lx", (unsigned long)phys);

	hexagon_pgd[phys >> HEX_PGDIR_SHIFT] =
		(uint32_t)phys | pte_perm_bits | (cache_attr << 6) | __HVM_PDE_S_4MB;
}

void hexagon_mmu_init(void)
{
	uintptr_t rom_start = (uintptr_t)__rom_region_start;
	uintptr_t rom_end = ROUND_UP((uintptr_t)__rom_region_end, HEX_PAGE_SIZE);
	uintptr_t ram_end = ROUND_UP((uintptr_t)_image_ram_end, HEX_PAGE_SIZE);
	int32_t ret;

	/*
	 * rom_start is __start, the very first thing linked into RAM --
	 * its address is the board's SRAM origin, fixed by devicetree/
	 * hardware, not something the linker script itself aligns. Every
	 * board this port targets happens to have a page-aligned (indeed
	 * far more than page-aligned) SRAM origin; assert rather than
	 * silently mis-map if that's ever not true.
	 */
	__ASSERT((rom_start & (HEX_PAGE_SIZE - 1)) == 0,
		 "SRAM origin 0x%lx is not page-aligned", (unsigned long)rom_start);

	/*
	 * rom_end must land exactly on _image_ram_start: this file's
	 * HEX_PAGE_SIZE and the linker script's HEXAGON_ROM_RAM_ALIGN have
	 * to agree for the same reason neither can be CONFIG_MMU_PAGE_SIZE
	 * alone (see the comments on both). A silent mismatch here either
	 * folds real writable data into the read-only ROM mapping or opens
	 * an unintended gap -- exactly the class of bug this assert exists
	 * to catch instead of a hard-to-diagnose fault deep in unrelated
	 * code the first time something touches the misplaced boundary.
	 */
	__ASSERT(rom_end == (uintptr_t)_image_ram_start,
		 "HEX_PAGE_SIZE (0x%x) and linker.ld's HEXAGON_ROM_RAM_ALIGN disagree: "
		 "rom_end=0x%lx != _image_ram_start=0x%lx",
		 HEX_PAGE_SIZE, (unsigned long)rom_end, (unsigned long)_image_ram_start);

	memset(hexagon_pgd, 0, sizeof(hexagon_pgd));
	hexagon_l2_next = 0;

	/* .text + .rodata: readable and executable, never writable. */
	hexagon_l2_map_range(rom_start, rom_end,
			     __HVM_PTE_R | __HVM_PTE_X | __HVM_PTE_U, __HEXAGON_C_WB_L2);

	/*
	 * .data/.bss/.noinit/stacks/heap: readable and writable, never
	 * executable. rom_end is exactly where this region starts: the
	 * linker script MMU-page-aligns the RAM section's first byte
	 * (soc/qemu/hexagon/linker.ld), with nothing but that alignment
	 * gap -- itself harmlessly covered by the ROM mapping above --
	 * between __rom_region_end and it.
	 */
#ifdef CONFIG_USERSPACE
	/*
	 * Split the R+W range into kernel-only and user-accessible pieces
	 * instead of one blanket R+W+U mapping. Without this split, any
	 * ordinary kernel .bss/.data object -- struct k_thread included --
	 * is directly readable and writable from user mode with no fault at
	 * all, regardless of memory-domain membership: this was the "known
	 * limitation" this file's header comment used to describe, and the
	 * root cause behind tests/kernel/mem_protect/userspace's
	 * read/write_kernram, read/write_kernel_data and
	 * read/write_priv_stack all failing to fault.
	 *
	 * _app_smem_start/end, z_hex_user_readable_start/end and
	 * z_user_stacks_start/end are each aligned to HEXAGON_ROM_RAM_ALIGN
	 * at both ends (APP_SHARED_ALIGN for the first, an explicit
	 * alignment added to soc/qemu/hexagon/linker.ld's two other
	 * sections), so every sub-range below is a whole number of L2
	 * pages, exactly like rom_end/ram_end already are --
	 * hexagon_l2_map_range() would otherwise assert. That page
	 * granularity is also why .hex_user_readable exists as a section of
	 * its own rather than a special case bolted on afterwards: this
	 * image's entire ordinary .bss/.noinit combined is itself well
	 * under one 64KB page, so carving U access back out for one flag
	 * *after* mapping the kernel-RAM range below would reopen that
	 * whole page, not just the flag.
	 *
	 * Every user thread's stack is accessible to any other regardless of
	 * memory domain, via the boot-time blanket grant below -- real
	 * per-thread stack isolation would need the same kind of dynamic,
	 * per-thread page-table update hexagon_mmu_grant_user_stack() already
	 * does for a stack outside this range entirely (dynamically
	 * allocated ones), generalized and made revocable. Not attempted
	 * here. Application shared memory (_app_smem), by contrast, *is*
	 * given real per-domain isolation below: boot with none of it
	 * U-accessible, and let hexagon_mmu_sync_domain_access() (called
	 * from arch_user_mode_enter() on every entry to user mode, same as
	 * the stack grant) grant exactly the partitions the thread about to
	 * run is actually a member of.
	 */
	uintptr_t smem_start = (uintptr_t)_app_smem_start;
	uintptr_t smem_end = (uintptr_t)_app_smem_end;
	uintptr_t ur_start = (uintptr_t)z_hex_user_readable_start;
	uintptr_t ur_end = (uintptr_t)z_hex_user_readable_end;
	uintptr_t ustacks_start = (uintptr_t)z_user_stacks_start;
	uintptr_t ustacks_end = (uintptr_t)z_user_stacks_end;

	__ASSERT(smem_start == rom_end,
		 "app shared memory 0x%lx does not start at rom_end 0x%lx",
		 (unsigned long)smem_start, (unsigned long)rom_end);

	/*
	 * Application shared memory (K_APP_BMEM/K_APP_DMEM): R+W, no U at
	 * boot. hexagon_mmu_sync_domain_access() grants U back per-partition
	 * before any user code could run (the very first arch_user_mode_enter()
	 * call, which necessarily precedes it).
	 */
	hexagon_l2_map_range(smem_start, smem_end,
			     __HVM_PTE_R | __HVM_PTE_W, __HEXAGON_C_WB_L2);

	/* Ordinary kernel .data/.tdata/.tbss/.bss/.noinit: R+W, no U. */
	hexagon_l2_map_range(smem_end, ur_start,
			     __HVM_PTE_R | __HVM_PTE_W, __HEXAGON_C_WB_L2);

	/*
	 * .hex_user_readable (_hexagon_user_mode_active): R+W+U. Never
	 * automatically zeroed at boot like ordinary .bss, so zero it
	 * explicitly here -- nothing can have read it before this point.
	 */
	hexagon_l2_map_range(ur_start, ur_end,
			     __HVM_PTE_R | __HVM_PTE_W | __HVM_PTE_U, __HEXAGON_C_WB_L2);
	_hexagon_user_mode_active = 0;

	/* User-mode thread stacks (K_THREAD_STACK_DEFINE): R+W+U. */
	hexagon_l2_map_range(ustacks_start, ustacks_end,
			     __HVM_PTE_R | __HVM_PTE_W | __HVM_PTE_U, __HEXAGON_C_WB_L2);

	/* Interrupt stack + tail padding: R+W, no U. */
	hexagon_l2_map_range(ustacks_end, ram_end,
			     __HVM_PTE_R | __HVM_PTE_W, __HEXAGON_C_WB_L2);
#else
	hexagon_l2_map_range(rom_end, ram_end,
			     __HVM_PTE_R | __HVM_PTE_W | __HVM_PTE_U, __HEXAGON_C_WB_L2);
#endif

	/*
	 * PL011 UART at 0x10000000: supervisor-only (no U), matching the
	 * previous flat map. SHARED bypasses H2's guestmap translation so
	 * the guest physical address is used directly as the host address.
	 */
	hexagon_l1_map_4mb(0x10000000, __HVM_PTE_R | __HVM_PTE_W | __HVM_PTE_SHARED,
			   __HEXAGON_C_DEV);

	/* H2's own kernel, two 4MB regions: supervisor-only RWX, matching
	 * the previous flat map.
	 */
	hexagon_l1_map_4mb(0x9b800000, __HVM_PTE_R | __HVM_PTE_W | __HVM_PTE_X,
			   __HEXAGON_C_WB_L2);
	hexagon_l1_map_4mb(0x9bc00000, __HVM_PTE_R | __HVM_PTE_W | __HVM_PTE_X,
			   __HEXAGON_C_WB_L2);

	/* Flush so H2's table walker sees the freshly written entries. */
	hexagon_vm_cache(hvmc_dccleaninva, 0, 0);

	ret = hexagon_vm_newmap(hexagon_pgd, VM_TRANS_TYPE_TABLE, VM_TLB_INVALIDATE_TRUE);
	if (ret != 0) {
		hexagon_vm_stop(VM_STOP_HALT);
	}
}

#ifdef CONFIG_USERSPACE
/*
 * Grant R+W+U access to a K_USER thread's own stack, at whatever page(s)
 * it actually lives in. Called from arch_user_mode_enter() (userspace.c)
 * every time a thread -- freshly created or already running -- is about
 * to execute in user mode.
 *
 * hexagon_mmu_init()'s kernel/user RAM split above only covers memory
 * whose extent is known at link time (_app_smem, .user_stacks,
 * .hex_user_readable): a stack allocated at runtime via
 * k_thread_stack_alloc() (CONFIG_DYNAMIC_THREAD_STACK_SIZE) can instead
 * come from _system_heap's backing storage, ordinary kernel .noinit that
 * the split maps without U. Rather than making that whole heap -- shared
 * by every kernel subsystem, not just user stacks -- blanket-U (a
 * materially bigger exposure than any of the narrow, purpose-built
 * regions above), grant U access to just the pages this one thread's
 * stack occupies, every time it (re-)enters user mode.
 *
 * hexagon_l2_map_range() writes directly into the live hexagon_l2 pool
 * already installed via hexagon_vm_newmap() above -- no second table, no
 * re-install -- so H2's next walk of these PTEs sees the new bits as
 * soon as the cache is flushed and any stale translation is cleared.
 * hexagon_vm_clrmap() (also used by tests/kernel/mem_protect/userspace's
 * test_userspace_disable_mmu_mpu from user mode, deliberately rejected
 * there) is exactly that: it invalidates cached translations for a
 * range without touching the table, cheap enough to call on every
 * user-mode entry -- unlike hexagon_vm_newmap(), it is not a full
 * table swap, so this does not reintroduce the TLB-pressure regression
 * HEX_PAGE_SIZE was widened to 64KB to fix.
 *
 * Collateral, same page-granularity trade-off as elsewhere in this file:
 * whatever else shares this stack's 64KB page(s) becomes U-accessible
 * too. And this is grant-only, with no revoke: once a page has hosted a
 * user stack it stays U-accessible even after that thread exits and the
 * page is freed and reused for something else -- the same
 * already-permanent-U model .user_stacks itself has, not a new weaker
 * guarantee. Real per-thread revocation of a *stack* specifically is not
 * attempted -- unlike app-shared-memory partitions (see
 * hexagon_mmu_sync_domain_access() below), a thread's own stack is never
 * meant to become inaccessible to it while it still runs, so there is no
 * test or real use case pushing on this the way there was for domains.
 */
void hexagon_mmu_grant_user_stack(uintptr_t start, size_t size)
{
	uintptr_t page_start = ROUND_DOWN(start, HEX_PAGE_SIZE);
	uintptr_t page_end = ROUND_UP(start + size, HEX_PAGE_SIZE);

	hexagon_l2_map_range(page_start, page_end,
			     __HVM_PTE_R | __HVM_PTE_W | __HVM_PTE_U, __HEXAGON_C_WB_L2);

	/*
	 * Flush the whole L2 pool rather than tracking exactly which
	 * table(s) page_start..page_end touched: it is small (at most
	 * HEX_MAX_L2_TABLES * HEX_L2_TABLE_BYTES, a couple KB) and this is
	 * not a hot path.
	 */
	hexagon_vm_cache(hvmc_dccleaninva, (uint32_t)(uintptr_t)hexagon_l2,
			 (uint32_t)sizeof(hexagon_l2));
	hexagon_vm_clrmap((void *)page_start, page_end - page_start);
}

/*
 * Re-sync application-shared-memory (K_APP_BMEM/K_APP_DMEM) U access to
 * exactly the partitions the given thread's memory domain actually
 * contains: real per-domain isolation for direct (non-syscall) user-mode
 * memory access, matching what arch_buffer_validate() above already does
 * in software for syscall-mediated access. Called from
 * arch_user_mode_enter() on every entry to user mode, same as
 * hexagon_mmu_grant_user_stack() -- and from z_hexagon_user_mode_sync()
 * (user_mode_state.c) whenever an ordinary preemptive switch resumes a
 * different, already-user-mode thread, since that path never goes back
 * through arch_user_mode_enter(). Without the second call site, the
 * shared page table kept whichever thread's grants were synced last, so
 * a switch between two K_USER threads in different domains left the
 * previous thread's grants live for the next one -- there being only
 * ever one thread actually *executing* in user mode at a time (see the
 * CONFIG_SMP BUILD_ASSERT in user_mode_state.c) says nothing about how
 * many take turns doing so.
 *
 * Deny-by-default full resync on every call, not incremental add/remove
 * tracking: arch_mem_domain_partition_add/remove() and
 * arch_mem_domain_thread_add/remove() (userspace.c) stay no-ops, and
 * there is no per-domain or per-thread state to keep consistent between
 * calls. Held under z_mem_domain_lock, the same lock
 * k_mem_domain_add_partition()/remove_partition() mutate partitions[]
 * under, since trap0 handling re-enables guest interrupts and a timer
 * tick can preempt this scan mid-loop.
 *
 * Each K_APP_BMEM/K_APP_DMEM partition is already its own whole number
 * of L2 pages (SMEM_PARTITION_ALIGN in soc/qemu/hexagon/linker.ld pads
 * every partition to HEXAGON_ROM_RAM_ALIGN at both ends, same as
 * _app_smem itself), so granting one partition never touches any page
 * belonging to another -- unlike hexagon_mmu_grant_user_stack()'s
 * necessarily coarser, whatever-else-shares-the-page trade-off.
 */
void hexagon_mmu_sync_domain_access(struct k_thread *thread)
{
	k_spinlock_key_t key = k_spin_lock(&z_mem_domain_lock);
	struct k_mem_domain *domain = thread->mem_domain_info.mem_domain;
	int remaining = domain != NULL ? domain->num_partitions : 0;

	/* Deny by default. */
	hexagon_l2_map_range((uintptr_t)_app_smem_start, (uintptr_t)_app_smem_end,
			     __HVM_PTE_R | __HVM_PTE_W, __HEXAGON_C_WB_L2);

	/* Grant back exactly this thread's domain's partitions. */
	for (int i = 0; remaining > 0 && i < CONFIG_MAX_DOMAIN_PARTITIONS; i++) {
		const struct k_mem_partition *part = &domain->partitions[i];
		uint32_t perm;

		if (part->size == 0) {
			continue; /* Unused hole left behind by a removed partition. */
		}
		remaining--;

		perm = __HVM_PTE_R | __HVM_PTE_U;
		if (K_MEM_PARTITION_IS_WRITABLE(part->attr)) {
			perm |= __HVM_PTE_W;
		}

		hexagon_l2_map_range(part->start, part->start + part->size, perm,
				     __HEXAGON_C_WB_L2);
	}

	hexagon_vm_cache(hvmc_dccleaninva, (uint32_t)(uintptr_t)hexagon_l2,
			 (uint32_t)sizeof(hexagon_l2));
	hexagon_vm_clrmap(_app_smem_start,
			 (uintptr_t)_app_smem_end - (uintptr_t)_app_smem_start);

	k_spin_unlock(&z_mem_domain_lock, key);
}
#endif /* CONFIG_USERSPACE */

#ifdef CONFIG_MMU

/*
 * CONFIG_KERNEL_DIRECT_MAP makes every arch_mem_map() request (device
 * MMIO via DEVICE_MMIO_MAP(), or an explicit K_MEM_DIRECT_MAP caller)
 * resolve virt == phys, so the range asked for is always already
 * covered by the table hexagon_mmu_init() installed. Nothing left to do
 * here; see the file header for why a second, dynamically-modified
 * table is deliberately not built.
 */
int arch_mem_map(void *virt, uintptr_t phys, size_t size, uint32_t flags)
{
	ARG_UNUSED(flags);

	__ASSERT((uintptr_t)virt == phys,
		 "hexagon requires CONFIG_KERNEL_DIRECT_MAP (virt %p != phys 0x%lx)",
		 virt, phys);
	ARG_UNUSED(size);

	return (uintptr_t)virt == phys ? 0 : -EINVAL;
}

int arch_mem_unmap(void *addr, size_t size)
{
	ARG_UNUSED(addr);
	ARG_UNUSED(size);

	return 0;
}

int arch_page_phys_get(void *virt, uintptr_t *phys)
{
	if (phys != NULL) {
		*phys = (uintptr_t)virt;
	}

	return 0;
}

#endif /* CONFIG_MMU */

/* Cache operations using HVM vmcache hypercall */

void arch_dcache_flush_all(void)
{
	hexagon_vm_cache(hvmc_dccleaninva, 0, 0);
}

void arch_dcache_invd_all(void)
{
	hexagon_vm_cache(hvmc_dckill, 0, 0);
}

void arch_dcache_flush_and_invd_all(void)
{
	hexagon_vm_cache(hvmc_dccleaninva, 0, 0);
}
