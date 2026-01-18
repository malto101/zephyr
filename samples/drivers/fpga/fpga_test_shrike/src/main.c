/*
 * Copyright (c) 2025 Dhruv Menon <dhruvmenon1104@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/fpga.h>

/* Include the bitstream header file */
#include "blink.h"

/* Get FPGA device from device tree */
#define FPGA_NODE DT_NODELABEL(fpga0)
const struct device *const fpga = DEVICE_DT_GET(FPGA_NODE);

/* Note: blink_bitstream is uint8_t[] (bytes) since SPI uses 8-bit words */

int main(void)
{
	int ret;

	printk("FPGA Test Application for Shrike-lite\n");
	printk("=====================================\n\n");

	/* Check if FPGA device is ready */
	if (!device_is_ready(fpga)) {
		printk("ERROR: FPGA device is not ready!\n");
		return -1;
	}

	printk("FPGA device: %s\n", fpga->name);

	/* Get FPGA status */
	enum FPGA_status status = fpga_get_status(fpga);
	printk("FPGA status: %s\n", 
	       status == FPGA_STATUS_ACTIVE ? "ACTIVE" : "INACTIVE");

	/* Power on FPGA */
	printk("\nPowering on FPGA...\n");
	ret = fpga_on(fpga);
	if (ret < 0) {
		printk("ERROR: Failed to power on FPGA: %d\n", ret);
		return ret;
	}
	printk("FPGA powered on\n");

	/* Wait a bit */
	k_msleep(100);

	/* Load bitstream */
	printk("\nLoading bitstream (blink.bin)...\n");
	printk("Bitstream address: %p\n", blink_bitstream);
	printk("Bitstream size: %u bytes\n", blink_bitstream_len);

	/* fpga_load API uses uint32_t* pointer type, but data is sent as 8-bit words (MSB first)
	 * The SPI driver is configured with SPI_WORD_SET(8) | SPI_TRANSFER_MSB
	 */
	ret = fpga_load(fpga, (uint32_t *)blink_bitstream, blink_bitstream_len);
	if (ret < 0) {
		printk("ERROR: Failed to load bitstream: %d\n", ret);
		return ret;
	}

	printk("Bitstream loaded successfully!\n");

	/* Get FPGA info */
	const char *info = fpga_get_info(fpga);
	printk("FPGA info: %s\n", info);

	/* Check status again */
	status = fpga_get_status(fpga);
	printk("FPGA status after load: %s\n",
	       status == FPGA_STATUS_ACTIVE ? "ACTIVE" : "INACTIVE");

	printk("\nFPGA configuration complete!\n");
	printk("If your bitstream implements LED blinking, you should see it now.\n");

	return 0;
}
