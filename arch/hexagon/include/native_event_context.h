/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Native Hexagon event context layout.
 *
 * This layout is deliberately separate from event_context.h. The HVM event
 * ABI saves GELR/GSR, while native event return state is held in ELR/SSR.
 * The frame is fixed-size so the assembly entry path and future native fault
 * consumers share one contract.
 */

#ifndef ZEPHYR_ARCH_HEXAGON_INCLUDE_NATIVE_EVENT_CONTEXT_H_
#define ZEPHYR_ARCH_HEXAGON_INCLUDE_NATIVE_EVENT_CONTEXT_H_

#define NATIVE_EVENT_CTX_R0_R1   0x00
#define NATIVE_EVENT_CTX_R2_R3   0x08
#define NATIVE_EVENT_CTX_R4_R5   0x10
#define NATIVE_EVENT_CTX_R6_R7   0x18
#define NATIVE_EVENT_CTX_R8_R9   0x20
#define NATIVE_EVENT_CTX_R10_R11 0x28
#define NATIVE_EVENT_CTX_R12_R13 0x30
#define NATIVE_EVENT_CTX_R14_R15 0x38
#define NATIVE_EVENT_CTX_PRED    0x40
#define NATIVE_EVENT_CTX_LR      0x44
#define NATIVE_EVENT_CTX_R28     0x48
#define NATIVE_EVENT_CTX_SSR     0x4c
#define NATIVE_EVENT_CTX_ELR     0x50
#define NATIVE_EVENT_CTX_SA0     0x54
#define NATIVE_EVENT_CTX_LC0     0x58
#define NATIVE_EVENT_CTX_SA1     0x5c
#define NATIVE_EVENT_CTX_LC1     0x60
#define NATIVE_EVENT_CTX_M0      0x64
#define NATIVE_EVENT_CTX_M1      0x68
#define NATIVE_EVENT_CTX_USR     0x6c
#define NATIVE_EVENT_CTX_G2      0x70
#define NATIVE_EVENT_CTX_GP      0x74
#define NATIVE_EVENT_CTX_R16_R17 0x78
#define NATIVE_EVENT_CTX_R18_R19 0x80
#define NATIVE_EVENT_CTX_R20_R21 0x88
#define NATIVE_EVENT_CTX_R22_R23 0x90
#define NATIVE_EVENT_CTX_R24_R25 0x98
#define NATIVE_EVENT_CTX_R26_R27 0xa0
#define NATIVE_EVENT_CTX_UGP     0xa8
#define NATIVE_EVENT_CTX_SIZE    0xb0

#ifndef _ASMLANGUAGE

#include <zephyr/types.h>

struct native_event_context {
	uint32_t r0_r1[2];
	uint32_t r2_r3[2];
	uint32_t r4_r5[2];
	uint32_t r6_r7[2];
	uint32_t r8_r9[2];
	uint32_t r10_r11[2];
	uint32_t r12_r13[2];
	uint32_t r14_r15[2];
	uint32_t pred_regs;
	uint32_t link_reg;
	uint32_t r28;
	uint32_t ssr;
	uint32_t elr;
	uint32_t sa0;
	uint32_t lc0;
	uint32_t sa1;
	uint32_t lc1;
	uint32_t m0;
	uint32_t m1;
	uint32_t usr;
	uint32_t g2;
	uint32_t gp;
	uint32_t r16_r17[2];
	uint32_t r18_r19[2];
	uint32_t r20_r21[2];
	uint32_t r22_r23[2];
	uint32_t r24_r25[2];
	uint32_t r26_r27[2];
	uint32_t ugp;
	uint32_t reserved;
};

BUILD_ASSERT(sizeof(struct native_event_context) == NATIVE_EVENT_CTX_SIZE,
	"native event context size does not match assembly ABI");

#endif /* _ASMLANGUAGE */

#endif /* ZEPHYR_ARCH_HEXAGON_INCLUDE_NATIVE_EVENT_CONTEXT_H_ */
