/*
 * Copyright (c) 2020, STMICROELECTRONICS
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/drivers/ipm.h>

#include <openamp/open_amp.h>
#include <metal/sys.h>
#include <metal/io.h>
#include <resource_table.h>

#ifdef CONFIG_SHELL_BACKEND_RPMSG
#include <zephyr/shell/shell_rpmsg.h>
#endif

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(openamp_rsc_table, LOG_LEVEL_INF);

#define SHM_DEVICE_NAME	"shm"

#if !DT_HAS_CHOSEN(zephyr_ipc_shm)
#error "Sample requires definition of shared memory for rpmsg"
#endif

/* Constants derived from device tree */
#define SHM_NODE		DT_CHOSEN(zephyr_ipc_shm)
#define SHM_START_ADDR	DT_REG_ADDR(SHM_NODE)
#define SHM_SIZE		DT_REG_SIZE(SHM_NODE)

#define APP_TASK_STACK_SIZE (1024)

/* Add 1024 extra bytes for the TTY task stack for the "tx_buff" buffer. */
#define APP_TTY_TASK_STACK_SIZE (1536)

K_THREAD_STACK_DEFINE(thread_mng_stack, APP_TASK_STACK_SIZE);
K_THREAD_STACK_DEFINE(thread_rp__client_stack, APP_TASK_STACK_SIZE);
K_THREAD_STACK_DEFINE(thread_tty_stack, APP_TTY_TASK_STACK_SIZE);

static struct k_thread thread_mng_data;
static struct k_thread thread_rp__client_data;
static struct k_thread thread_tty_data;

static const struct device *const ipm_handle =
	DEVICE_DT_GET(DT_CHOSEN(zephyr_ipc));

static metal_phys_addr_t shm_physmap = SHM_START_ADDR;
static metal_phys_addr_t rsc_tab_physmap;

static struct metal_io_region shm_io_data; /* shared memory */
static struct metal_io_region rsc_io_data; /* rsc_table memory */

struct rpmsg_rcv_msg {
	void *data;
	size_t len;
};

static struct metal_io_region *shm_io = &shm_io_data;

static struct metal_io_region *rsc_io = &rsc_io_data;
static struct rpmsg_virtio_device rvdev;

static struct fw_resource_table *rsc_table;
static struct rpmsg_device *rpdev;

static char rx_sc_msg[20];  /* should receive "Hello world!" */
static struct rpmsg_endpoint sc_ept;
static struct rpmsg_rcv_msg sc_msg = {.data = rx_sc_msg};

static struct rpmsg_endpoint tty_ept;
static struct rpmsg_rcv_msg tty_msg;

static K_SEM_DEFINE(data_sem, 0, 1);
static K_SEM_DEFINE(data_sc_sem, 0, 1);
static K_SEM_DEFINE(data_tty_sem, 0, 1);

static void platform_ipm_callback(const struct device *dev, void *context,
				  uint32_t id, volatile void *data)
{
	LOG_DBG("msg received from mb %d: data: 0x%08x, size: %d", id, *((uint32_t *)data), 4);
	k_sem_give(&data_sem);
}

static int rpmsg_recv_cs_callback(struct rpmsg_endpoint *ept, void *data,
				  size_t len, uint32_t src, void *priv)
{
	memcpy(sc_msg.data, data, len);
	sc_msg.len = len;
	k_sem_give(&data_sc_sem);

	return RPMSG_SUCCESS;
}

static int rpmsg_recv_tty_callback(struct rpmsg_endpoint *ept, void *data,
				   size_t len, uint32_t src, void *priv)
{
	struct rpmsg_rcv_msg *msg = priv;

	rpmsg_hold_rx_buffer(ept, data);
	msg->data = data;
	msg->len = len;
	k_sem_give(&data_tty_sem);

	return RPMSG_SUCCESS;
}

static void receive_message(unsigned char **msg, unsigned int *len)
{
	int status = k_sem_take(&data_sem, K_FOREVER);

	if (status == 0) {
		rproc_virtio_notified(rvdev.vdev, VRING1_ID);
	}
}

static void new_service_cb(struct rpmsg_device *rdev, const char *name,
			   uint32_t src)
{
	LOG_ERR("%s: unexpected ns service receive for name %s",
		__func__, name);
}

int mailbox_notify(struct remoteproc *rproc, uint32_t id)
{
	ARG_UNUSED(rproc);

	LOG_DBG("msg to send to host with id: %d", id);
	ipm_send(ipm_handle, 0, id, &id, 4);

	return 0;
}

int platform_init(void)
{
	int rsc_size;
	struct metal_init_params metal_params = METAL_INIT_DEFAULTS;
	int status;

	status = metal_init(&metal_params);
	if (status) {
		LOG_ERR("metal_init: failed: %d", status);
		return -1;
	}

	/* declare shared memory region */
	metal_io_init(shm_io, (void *)SHM_START_ADDR, &shm_physmap,
		      SHM_SIZE, -1, 0, NULL);

	/* declare resource table region */
	rsc_table_get(&rsc_table, &rsc_size);
	rsc_tab_physmap = (uintptr_t)rsc_table;

	printk(" *** Got rsc_table: %p\n", rsc_table);

	metal_io_init(rsc_io, rsc_table,
		      &rsc_tab_physmap, rsc_size, -1, 0, NULL);

	/* setup IPM */
	if (!device_is_ready(ipm_handle)) {
		LOG_ERR("IPM device is not ready");
		return -1;
	}

	ipm_register_callback(ipm_handle, platform_ipm_callback, NULL);

	status = ipm_set_enabled(ipm_handle, 1);
	if (status) {
		LOG_ERR("ipm_set_enabled failed");
		return -1;
	}

	return 0;
}

static struct remoteproc *
rproc_init(struct remoteproc *rproc,
			const struct remoteproc_ops *ops, void *arg)
{
	struct remoteproc_priv *prproc = arg;

	rproc->priv = prproc;
	rproc->ops = ops;
	return rproc;
}

const struct remoteproc_ops rproc_ops = {
	.init = rproc_init,
	.notify = mailbox_notify,
};

static struct remoteproc rproc_inst;

static void cleanup_system(void)
{
	ipm_set_enabled(ipm_handle, 0);
	rpmsg_deinit_vdev(&rvdev);
	metal_finish();
}

struct  rpmsg_device *
platform_create_rpmsg_vdev(unsigned int vdev_index,
			   unsigned int role,
			   void (*rst_cb)(struct virtio_device *vdev),
			   rpmsg_ns_bind_cb ns_cb)
{
	struct remoteproc_mem *mem;
	struct virtio_device *vdev;
	int rsc_size;
	int ret;

	/* Initialize remoteproc instance */
	if (!remoteproc_init(&rproc_inst, &rproc_ops, NULL))
		return NULL;

	/* Add resource table memory to remoteproc */
	rsc_table_get(&rsc_table, &rsc_size);
	rsc_tab_physmap = (uintptr_t)rsc_table;
	rproc_inst.rsc_io = rsc_io;
	mem = metal_allocate_memory(sizeof(*mem));
	if (!mem)
		goto failed2;
	remoteproc_init_mem(mem, NULL, rsc_tab_physmap, rsc_tab_physmap, rsc_size, rsc_io);
	remoteproc_add_mem(&rproc_inst, mem);

	/* Add shared memory region to remoteproc */
	mem = metal_allocate_memory(sizeof(*mem));
	if (!mem)
		goto failed2;
	remoteproc_init_mem(mem, NULL, shm_physmap, shm_physmap, SHM_SIZE, shm_io);
	remoteproc_add_mem(&rproc_inst, mem);

//	printk("rsc_table->vdev->notifyid: %d\n", rsc_table->vdev->notifyid);
//	printk("rsc_table->vdev->da: 0x%08x\n", rsc_table->vdev->da);
//	printk("rsc_table->vdev->num: %d\n", rsc_table->vdev->num);
//	printk("rsc_table->vdev->align: %d\n", rsc_table->vdev->align);

	/* Pass resource table to remoteproc */
	ret = remoteproc_set_rsc_table(&rproc_inst, (struct resource_table *)rsc_table, rsc_size);
	if (ret) {
		LOG_ERR("failed remoteproc_set_rsc_table\n");
		goto failed2;
	}

	/* Create virtio device from remoteproc */
	vdev = remoteproc_create_virtio(&rproc_inst, vdev_index, role, rst_cb);
	if (!vdev) {
		LOG_ERR("failed remoteproc_create_virtio\n");
		goto failed2;
	}

	/* Create rpmsg device from virtio device */
	ret = rpmsg_init_vdev(&rvdev, vdev, ns_cb, shm_io, NULL);
	if (ret) {
		LOG_ERR("failed rpmsg_init_vdev");
		goto failed;
	}

	return rpmsg_virtio_get_rpmsg_device(&rvdev);

failed:
	remoteproc_remove_virtio(&rproc_inst, vdev);
failed2:
	remoteproc_remove(&rproc_inst);

	return NULL;
}

void app_rpmsg_client_sample(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	unsigned int msg_cnt = 0;
	int ret = 0;

	k_sem_take(&data_sc_sem,  K_FOREVER);

	LOG_INF("OpenAMP[remote] Linux sample client responder started");

	ret = rpmsg_create_ept(&sc_ept, rpdev, "rpmsg-client-sample",
			       RPMSG_ADDR_ANY, RPMSG_ADDR_ANY,
			       rpmsg_recv_cs_callback, NULL);
	if (ret) {
		LOG_ERR("[Linux sample client] Could not create endpoint: %d", ret);
		goto task_end;
	}

	printk("OpenAMP[remote] rpmsg_create_ept ret: %d\n", ret);

	while (msg_cnt < 100) {
		k_sem_take(&data_sc_sem,  K_FOREVER);
		msg_cnt++;
		LOG_INF("[Linux sample client] incoming msg %d: %.*s", msg_cnt, sc_msg.len,
			(char *)sc_msg.data);
		rpmsg_send(&sc_ept, sc_msg.data, sc_msg.len);
	}
	rpmsg_destroy_ept(&sc_ept);

	virtqueue_dump(rvdev.svq);
	virtqueue_dump(rvdev.rvq);

task_end:
	LOG_INF("OpenAMP Linux sample client responder ended");
}

void app_rpmsg_tty(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	unsigned char tx_buff[512];
	int ret = 0;

	k_sem_take(&data_tty_sem,  K_FOREVER);

	LOG_INF("OpenAMP[remote] Linux TTY responder started");

//	volatile int *x = 0x80000004;
//	*x = 0x1234;

	tty_ept.priv = &tty_msg;
	ret = rpmsg_create_ept(&tty_ept, rpdev, "rpmsg-tty",
			       RPMSG_ADDR_ANY, RPMSG_ADDR_ANY,
			       rpmsg_recv_tty_callback, NULL);
	if (ret) {
		LOG_ERR("[Linux TTY] Could not create endpoint: %d", ret);
		goto task_end;
	}

	while (tty_ept.addr !=  RPMSG_ADDR_ANY) {
		k_sem_take(&data_tty_sem,  K_FOREVER);
		if (tty_msg.len) {
			LOG_INF("[Linux TTY] incoming msg: %.*s",
				tty_msg.len, (char *)tty_msg.data);
			snprintf(tx_buff, 13, "TTY 0x%04x: ", tty_ept.addr);
			memcpy(&tx_buff[12], tty_msg.data, tty_msg.len);
			rpmsg_send(&tty_ept, tx_buff, tty_msg.len + 12);
			rpmsg_release_rx_buffer(&tty_ept, tty_msg.data);
		}
		tty_msg.len = 0;
		tty_msg.data = NULL;
	}
	rpmsg_destroy_ept(&tty_ept);

task_end:
	LOG_INF("OpenAMP Linux TTY responder ended");
}

void rpmsg_mng_task(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	unsigned char *msg;
	unsigned int len;
	int ret = 0;

	LOG_INF("OpenAMP[remote] Linux responder demo started");

	/* Initialize platform */
	ret = platform_init();
	if (ret) {
		LOG_ERR("Failed to initialize platform");
		ret = -1;
		goto task_end;
	}

	rpdev = platform_create_rpmsg_vdev(0, VIRTIO_DEV_DEVICE, NULL,
					   new_service_cb);
	if (!rpdev) {
		LOG_ERR("Failed to create rpmsg virtio device");
		ret = -1;
		goto task_end;
	}

#ifdef CONFIG_SHELL_BACKEND_RPMSG
	(void)shell_backend_rpmsg_init_transport(rpdev);
#endif

	/* start the rpmsg clients */
	k_sem_give(&data_sc_sem);
#ifndef CONFIG_SHELL_BACKEND_RPMSG
	k_sem_give(&data_tty_sem);
#endif

	while (1) {
		receive_message(&msg, &len);
	}

task_end:
	cleanup_system();

	LOG_INF("OpenAMP demo ended");
}

int main(void)
{
	LOG_INF("Starting application threads!");
	k_thread_create(&thread_mng_data, thread_mng_stack, APP_TASK_STACK_SIZE,
			rpmsg_mng_task,
			NULL, NULL, NULL, K_PRIO_COOP(8), 0, K_NO_WAIT);
	k_thread_create(&thread_rp__client_data, thread_rp__client_stack, APP_TASK_STACK_SIZE,
			app_rpmsg_client_sample,
			NULL, NULL, NULL, K_PRIO_COOP(7), 0, K_NO_WAIT);
	k_thread_create(&thread_tty_data, thread_tty_stack, APP_TTY_TASK_STACK_SIZE,
			app_rpmsg_tty,
			NULL, NULL, NULL, K_PRIO_COOP(7), 0, K_NO_WAIT);
	return 0;
}
