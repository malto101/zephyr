/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_ARCH_HEXAGON_INCLUDE_HEXAGON_HVX_H_
#define ZEPHYR_ARCH_HEXAGON_INCLUDE_HEXAGON_HVX_H_

#include <stdbool.h>
#include <zephyr/kernel.h>

/* 128-byte vectors: V0-V31, then Q0-Q3 packed into one vector */
#define HEXAGON_HVX_VLEN      128U
#define HEXAGON_HVX_SAVE_SIZE (33U * HEXAGON_HVX_VLEN)

/* Cause 0x16 handler, returns true if the faulting packet can run again */
bool z_hexagon_hvx_trap(void);

/* Free @p thread's context and saved state, interrupts must be locked */
void z_hexagon_hvx_thread_release(struct k_thread *thread);

/* Called from z_hexagon_arch_switch() after _current is the new thread */
void z_hexagon_hvx_switch(void);

/* In hvx_context.S, buffers are HEXAGON_HVX_SAVE_SIZE bytes and vector aligned */
void z_hexagon_hvx_save(void *buf);
void z_hexagon_hvx_restore(const void *buf);
void z_hexagon_hvx_zero(void);

#endif /* ZEPHYR_ARCH_HEXAGON_INCLUDE_HEXAGON_HVX_H_ */
