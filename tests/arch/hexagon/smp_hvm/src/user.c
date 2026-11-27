/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/ztest.h>

#define STACK_SIZE 2048
#define RUN_COUNT  100000U

static uint32_t __aligned(CONFIG_MMU_PAGE_SIZE)
	shared_buf[CONFIG_MMU_PAGE_SIZE / sizeof(uint32_t)];
K_MEM_PARTITION_DEFINE(shared_part, shared_buf, sizeof(shared_buf), K_MEM_PARTITION_P_RW_U_RW);

static K_THREAD_STACK_ARRAY_DEFINE(user_stacks, 2, STACK_SIZE);
static struct k_thread user_threads[2];

/*
 * Each thread runs on its own CPU at the same time as the other, in the
 * same domain: both stacks and the partition must be mapped at once.
 */
static void user_fn(void *p1, void *p2, void *p3)
{
	uint32_t idx = (uint32_t)(uintptr_t)p1;
	volatile uint32_t *mine = &shared_buf[idx];
	volatile uint32_t *other = &shared_buf[idx ^ 1U];
	volatile uint32_t local = 0U;

	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	*mine = 1U;
	while (*other == 0U) {
	}
	while (local < RUN_COUNT) {
		local++;
		*mine = local;
	}
}

ZTEST(smp_hvm, test_user_threads_each_cpu)
{
	static struct k_mem_domain domain;
	struct k_mem_partition *parts[] = {&shared_part};

	(void)memset(shared_buf, 0, sizeof(shared_buf));
	zassert_equal(k_mem_domain_init(&domain, ARRAY_SIZE(parts), parts), 0);

	for (uint32_t i = 0U; i < 2U; i++) {
		k_tid_t tid = k_thread_create(&user_threads[i], user_stacks[i], STACK_SIZE,
					      user_fn, (void *)(uintptr_t)i, NULL, NULL,
					      K_PRIO_PREEMPT(1), K_USER, K_FOREVER);

		zassert_equal(k_mem_domain_add_thread(&domain, tid), 0);
		zassert_equal(k_thread_cpu_pin(tid, (int)i), 0);
	}
	for (uint32_t i = 0U; i < 2U; i++) {
		k_thread_start(&user_threads[i]);
	}
	for (uint32_t i = 0U; i < 2U; i++) {
		zassert_equal(k_thread_join(&user_threads[i], K_SECONDS(10)), 0,
			      "user thread %u did not finish", i);
	}

	for (uint32_t i = 0U; i < 2U; i++) {
		zassert_equal(shared_buf[i], RUN_COUNT, "thread %u count %u", i, shared_buf[i]);
	}
}
