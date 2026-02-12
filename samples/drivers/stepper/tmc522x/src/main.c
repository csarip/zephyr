/*
 * SPDX-FileCopyrightText: Copyright (c) 2026 Cherrence Sarip <Cherrence.sarip@analog.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/stepper.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "../../../../drivers/stepper/adi_tmc/tmc522x/tmc522x.h"

LOG_MODULE_REGISTER(stepper_tmc522x, LOG_LEVEL_INF);

#define STEPPER_NODE DT_ALIAS(stepper0)

int main(void)
{
	const struct device *stepper = DEVICE_DT_GET(STEPPER_NODE);
	int ret;

	LOG_INF("TMC522x stepper sample");

	if (!device_is_ready(stepper)) {
		LOG_ERR("Device not ready");
		return -ENODEV;
	}

	ret = stepper_enable(stepper);
	if (ret < 0) {
		LOG_ERR("Failed to enable: %d", ret);
		return ret;
	}

	/* Test 1: SpreadCycle Position Mode */
	LOG_INF("Test 1: SpreadCycle + Position");
	tmc522x_stepper_set_chopper_mode(stepper, false);
	k_msleep(100);

	ret = stepper_move_by(stepper, 5000);
	if (ret < 0) {
		LOG_ERR("Position move failed: %d", ret);
		return ret;
	}
	k_msleep(3000);

	/* Test 2: SpreadCycle Velocity Mode */
	LOG_INF("Test 2: SpreadCycle + Velocity");
	ret = stepper_run(stepper, STEPPER_DIRECTION_POSITIVE);
	if (ret < 0) {
		LOG_ERR("Velocity mode failed: %d", ret);
		return ret;
	}
	k_msleep(3000);
	stepper_stop(stepper);
	k_msleep(1000);

	/* Test 3: StealthChop Position Mode */
	LOG_INF("Test 3: StealthChop + Position");
	tmc522x_stepper_set_chopper_mode(stepper, true);
	k_msleep(100);

	ret = stepper_move_by(stepper, 5000);
	if (ret < 0) {
		LOG_ERR("Position move failed: %d", ret);
		return ret;
	}
	k_msleep(3000);

	/* Test 4: StealthChop Velocity Mode */
	LOG_INF("Test 4: StealthChop + Velocity");
	ret = stepper_run(stepper, STEPPER_DIRECTION_POSITIVE);
	if (ret < 0) {
		LOG_ERR("Velocity mode failed: %d", ret);
		return ret;
	}
	k_msleep(3000);
	stepper_stop(stepper);
	k_msleep(1000);

	/* Test 5: Reverse Velocity */
	LOG_INF("Test 5: Reverse Velocity");
	tmc522x_stepper_set_chopper_mode(stepper, false);
	k_msleep(100);

	ret = tmc522x_set_velocity(stepper, -5000);
	if (ret < 0) {
		LOG_ERR("Reverse velocity failed: %d", ret);
		return ret;
	}
	k_msleep(3000);
	stepper_stop(stepper);

	LOG_INF("All tests complete");

	return 0;
}

