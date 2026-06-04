/*
 * SPDX-FileCopyrightText: Copyright (c) 2026 Cherrence Sarip <cherrence.sarip@analog.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>

#include "tmc6460.h"

LOG_MODULE_REGISTER(tmc6460_sample, LOG_LEVEL_INF);

int main(void)
{
	const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(tmc6460));
	uint32_t val;
	int ret;

	if (!device_is_ready(dev)) {
		LOG_ERR("TMC6460 device not ready");
		return -ENODEV;
	}

	LOG_INF("TMC6460 device ready: %s", dev->name);

	/* Read CHIP.ID to verify SPI communication */
	ret = tmc6460_read(dev, TMC6460_REG_CHIP_ID, &val);
	if (ret != 0) {
		LOG_ERR("Failed to read CHIP.ID: %d", ret);
		return ret;
	}
	LOG_INF("CHIP.ID = 0x%08x", val);

	/* Read IO CONFIG */
	ret = tmc6460_read(dev, TMC6460_REG_CHIP_IO_CONFIG, &val);
	if (ret != 0) {
		LOG_ERR("Failed to read IO_CONFIG: %d", ret);
		return ret;
	}
	LOG_INF("IO_CONFIG = 0x%08x", val);

	/* Write IO CONFIG example */
	ret = tmc6460_write(dev, TMC6460_REG_CHIP_IO_CONFIG, 0x00000004U);
	if (ret != 0) {
		LOG_ERR("Failed to write IO_CONFIG: %d", ret);
		return ret;
	}
	LOG_INF("Wrote IO_CONFIG = 0x00000004");

	/* Read back IO CONFIG to confirm */
	ret = tmc6460_read(dev, TMC6460_REG_CHIP_IO_CONFIG, &val);
	if (ret != 0) {
		LOG_ERR("Failed to read back IO_CONFIG: %d", ret);
		return ret;
	}
	LOG_INF("IO_CONFIG readback = 0x%08x", val);

	LOG_INF("TMC6460 sample complete");
	return 0;
}
