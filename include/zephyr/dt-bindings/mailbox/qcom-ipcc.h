/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_MAILBOX_QCOM_IPCC_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_MAILBOX_QCOM_IPCC_H_

#define IPCC_CHANNEL(client, signal) (((client) << 16) | (signal))

/* Signals of the MPROC protocol */
#define IPCC_MPROC_SIGNAL_GLINK_QMP 0
#define IPCC_MPROC_SIGNAL_TZ        1
#define IPCC_MPROC_SIGNAL_SMP2P     2
#define IPCC_MPROC_SIGNAL_PING      3

/* Clients */
#define IPCC_CLIENT_AOP  0
#define IPCC_CLIENT_TZ   1
#define IPCC_CLIENT_MPSS 2
#define IPCC_CLIENT_LPASS 3
#define IPCC_CLIENT_SLPI 4
#define IPCC_CLIENT_SDC  5
#define IPCC_CLIENT_CDSP 6
#define IPCC_CLIENT_NPU  7
#define IPCC_CLIENT_APSS 8

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_MAILBOX_QCOM_IPCC_H_ */
