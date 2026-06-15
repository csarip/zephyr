/*
 * SPDX-FileCopyrightText: Copyright (c) 2026 Cherrence Sarip <cherrence.sarip@analog.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

#include "tmc6460.h"

LOG_MODULE_REGISTER(tmc6460_sample, LOG_LEVEL_INF);

#define DRV_EN_NODE DT_ALIAS(tmc6460_drv_en)
static const struct gpio_dt_spec drv_en = GPIO_DT_SPEC_GET(DRV_EN_NODE, gpios);

#define SLEEPN_NODE DT_ALIAS(tmc6460_sleepn)
static const struct gpio_dt_spec sleepn = GPIO_DT_SPEC_GET(SLEEPN_NODE, gpios);

int main(void)
{
	const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(tmc6460));
	uint32_t val;
	int ret;

	/* Deassert SLEEPN (drive HIGH to wake TMC6460) */
	if (!gpio_is_ready_dt(&sleepn)) {
		LOG_ERR("SLEEPN GPIO not ready");
		return -ENODEV;
	}

	ret = gpio_pin_configure_dt(&sleepn, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		LOG_ERR("Failed to configure SLEEPN: %d", ret);
		return ret;
	}
	LOG_INF("SLEEPN deasserted (PF14 HIGH)");

	/* Assert DRV_EN high to enable the TMC6460 */
	if (!gpio_is_ready_dt(&drv_en)) {
		LOG_ERR("DRV_EN GPIO not ready");
		return -ENODEV;
	}

	ret = gpio_pin_configure_dt(&drv_en, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		LOG_ERR("Failed to configure DRV_EN: %d", ret);
		return ret;
	}
	LOG_INF("DRV_EN asserted (PB11 HIGH)");

	/* Longer delay for TMC6460 PLL to lock after reset */
	k_msleep(100);

	if (!device_is_ready(dev)) {
		LOG_ERR("TMC6460 device not ready");
		return -ENODEV;
	}

	LOG_INF("TMC6460 device ready: %s", dev->name);

	/* Read CHIP.ID to verify SPI communication */
	ret = tmc6460_read(dev, TMC6460_CHIP_ID, &val);
	if (ret != 0) {
		LOG_ERR("Failed to read CHIP.ID: %d", ret);
		return ret;
	}
	LOG_INF("CHIP.ID = 0x%08x", val);

	/* Read IO CONFIG (read-only, shows pin states) */
	ret = tmc6460_read(dev, TMC6460_CHIP_IO_CONFIG, &val);
	if (ret != 0) {
		LOG_ERR("Failed to read IO_CONFIG: %d", ret);
		return ret;
	}
	LOG_INF("IO_CONFIG = 0x%08x", val);

	/* Read more chip info */
	ret = tmc6460_read(dev, TMC6460_CHIP_VARIANT, &val);
	if (ret == 0) {
		LOG_INF("CHIP_VARIANT = 0x%08x", val);
	}

	ret = tmc6460_read(dev, TMC6460_CHIP_REVISION, &val);
	if (ret == 0) {
		LOG_INF("CHIP_REVISION = 0x%08x", val);
	}

	/* Check PLL status */
	ret = tmc6460_read(dev, TMC6460_CLK_CTRL_STATUS, &val);
	if (ret == 0) {
		LOG_INF("CLK_CTRL_STATUS = 0x%08x", val);
		LOG_INF("  CLK_1M0_OK=%u  PLL_LOCK=%u  PLL_READY=%u",
			(val >> 0) & 1U, (val >> 5) & 1U, (val >> 6) & 1U);
	}

	/* Read CLK_CTRL_CONFIG */
	ret = tmc6460_read(dev, TMC6460_CLK_CTRL_CONFIG, &val);
	if (ret == 0) {
		LOG_INF("CLK_CTRL_CONFIG = 0x%08x", val);
	}

	/*
	 * Step 1: Write CLK_CTRL_CONFIG WITHOUT commit to verify write works.
	 * Use clock divider = 15 for 15MHz/15 = 1MHz reference.
	 * PLL_EN=1, ADC_CLK_EN=1, PWM_CLK_EN=1, CLK_FSM_EN=1.
	 * CLOCK_DIVIDER[12:8] = 15 = 0x0F.
	 * Value = (0x0F << 8) | PLL_EN | ADC_CLK_EN | PWM_CLK_EN | CLK_FSM_EN
	 *       = 0x0F00 | 0x04 | 0x08 | 0x10 | 0x20 = 0x0F3C
	 */
	ret = tmc6460_write(dev, TMC6460_CLK_CTRL_CONFIG, 0x00000F3CU);
	if (ret != 0) {
		LOG_ERR("Failed to write CLK config: %d", ret);
		return ret;
	}
	LOG_INF("Wrote CLK_CTRL_CONFIG = 0x0F3C (no COMMIT yet)");

	/* Read back to verify write persists */
	ret = tmc6460_read(dev, TMC6460_CLK_CTRL_CONFIG, &val);
	if (ret == 0) {
		LOG_INF("CLK_CTRL_CONFIG readback = 0x%08x", val);
	}

	/* Step 2: Now write with COMMIT bit to apply */
	ret = tmc6460_write(dev, TMC6460_CLK_CTRL_CONFIG, 0x00000F3DU);
	if (ret != 0) {
		LOG_ERR("Failed to commit CLK config: %d", ret);
		return ret;
	}
	LOG_INF("CLK_CTRL_CONFIG committed (0x0F3D)");

	/* Wait for PLL to lock (needs time) */
	k_msleep(50);

	/* Check PLL status again */
	ret = tmc6460_read(dev, TMC6460_CLK_CTRL_STATUS, &val);
	if (ret == 0) {
		LOG_INF("CLK_CTRL_STATUS after commit = 0x%08x", val);
		LOG_INF("  CLK_1M0_OK=%u  PLL_LOCK=%u  PLL_READY=%u",
			(val >> 0) & 1U, (val >> 5) & 1U, (val >> 6) & 1U);
	}

	/* Read back config again (COMMIT is a strobe, should self-clear) */
	ret = tmc6460_read(dev, TMC6460_CLK_CTRL_CONFIG, &val);
	if (ret == 0) {
		LOG_INF("CLK_CTRL_CONFIG after commit = 0x%08x", val);
	}

	/* Also try writing to a CHIP-domain writable register */
	ret = tmc6460_write(dev, TMC6460_CHIP_SPI_STATUS_MASK, 0x0000000FU);
	if (ret != 0) {
		LOG_ERR("Failed to write SPI_STATUS_MASK: %d", ret);
	}
	ret = tmc6460_read(dev, TMC6460_CHIP_SPI_STATUS_MASK, &val);
	if (ret == 0) {
		LOG_INF("CHIP_SPI_STATUS_MASK readback = 0x%08x", val);
	}

	/* Test write: set RAMPER_V_MAX to a known value */
	ret = tmc6460_write(dev, TMC6460_RAMPER_V_MAX, 0x00001000U);
	if (ret != 0) {
		LOG_ERR("Failed to write RAMPER_V_MAX: %d", ret);
		return ret;
	}
	LOG_INF("Wrote RAMPER_V_MAX = 0x00001000");

	/* Read back RAMPER_V_MAX to confirm write */
	ret = tmc6460_read(dev, TMC6460_RAMPER_V_MAX, &val);
	if (ret == 0) {
		LOG_INF("RAMPER_V_MAX readback = 0x%08x", val);
	}

	LOG_INF("TMC6460 sample complete");
	return 0;
}
