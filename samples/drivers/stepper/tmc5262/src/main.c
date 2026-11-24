/*
 * SPDX-FileCopyrightText: Copyright (c) 2026 Analog Devices, Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/stepper/stepper.h>
#include <zephyr/drivers/stepper/stepper_ctrl.h>
#include <zephyr/drivers/stepper/stepper_tmc5262.h>
#include <zephyr/kernel.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(stepper_tmc5262, CONFIG_STEPPER_LOG_LEVEL);

/* The TMC2262 has no ramp generator, its steps are GPIO pulses and need a slower rate */
#define STEP_INTERVAL_NS                                                                           \
	(DT_NODE_HAS_COMPAT(DT_ALIAS(stepper_chip), adi_tmc2262) ? CONFIG_SAMPLE_STEP_INTERVAL_NS  \
								 : CONFIG_SAMPLE_RAMP_INTERVAL_NS)

/* The overlay of each device defines the three aliases (see tmc5262.overlay etc.) */
static const struct device *const chip = DEVICE_DT_GET(DT_ALIAS(stepper_chip));
static const struct device *const stepper_driver = DEVICE_DT_GET(DT_ALIAS(stepper_driver));
static const struct device *const stepper_ctrl = DEVICE_DT_GET(DT_ALIAS(stepper_ctrl));

K_SEM_DEFINE(steps_completed_sem, 0, 1);

static void ctrl_event_cb(const struct device *dev, const enum stepper_ctrl_event event,
			  void *user_data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(user_data);

	if (event == STEPPER_CTRL_EVENT_STEPS_COMPLETED) {
		k_sem_give(&steps_completed_sem);
	}
}

static void driver_event_cb(const struct device *dev, const enum stepper_event event,
			    void *user_data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(user_data);

	if (event == STEPPER_EVENT_FAULT_DETECTED) {
		LOG_ERR("Fault detected, power stage is disabled");
	} else if (event == STEPPER_EVENT_STALL_DETECTED) {
		LOG_WRN("Stall detected");
	}
}

/* The supply voltage and the die temperature through the sensor API, if the overlay has a sensor */
#if DT_NODE_HAS_STATUS_OKAY(DT_ALIAS(stepper_sensor))
static const struct device *const sensor = DEVICE_DT_GET(DT_ALIAS(stepper_sensor));

static void report_sensor(void)
{
	struct sensor_value voltage;
	struct sensor_value temp;

	if (!device_is_ready(sensor) || sensor_sample_fetch(sensor) != 0 ||
	    sensor_channel_get(sensor, SENSOR_CHAN_VOLTAGE, &voltage) != 0 ||
	    sensor_channel_get(sensor, SENSOR_CHAN_DIE_TEMP, &temp) != 0) {
		LOG_ERR("Failed to read the sensor");
		return;
	}

	LOG_INF("Sensor: supply %d.%03d V, die temperature %d.%03d degC", voltage.val1,
		voltage.val2 / 1000, temp.val1, temp.val2 / 1000);
}
#else
static void report_sensor(void)
{
	int32_t supply_mv;
	int32_t temp_mdegc;

	if (tmc5262_get_supply_mv(chip, &supply_mv) == 0 &&
	    tmc5262_get_temperature_mdegc(chip, &temp_mdegc) == 0) {
		LOG_INF("Supply %d mV, chip temperature %d.%03d degC", supply_mv, temp_mdegc / 1000,
			(temp_mdegc < 0 ? -(temp_mdegc % 1000) : (temp_mdegc % 1000)));
	}
}
#endif

static void report_status(void)
{
	struct tmc5262_drv_status status;
	int ret;

	ret = tmc5262_get_drv_status(chip, &status);
	if (ret == 0) {
		LOG_INF("DRV_STATUS: standstill=%d otpw=%d ot=%d stall=%d cs_actual=%u "
			"sg_result=%u",
			status.standstill, status.otpw, status.ot, status.stall, status.cs_actual,
			status.sg_result);
		LOG_INF("faults: short_gnd=%d/%d short_vs=%d/%d open_load=%d/%d (informative)",
			status.short_gnd_a, status.short_gnd_b, status.short_vs_a,
			status.short_vs_b, status.open_load_a, status.open_load_b);
	} else {
		LOG_ERR("Failed to read DRV_STATUS (%d)", ret);
	}

	report_sensor();
}

int main(void)
{
	int32_t position;
	bool moving;
	int ret;

	if (!device_is_ready(chip) || !device_is_ready(stepper_driver) ||
	    !device_is_ready(stepper_ctrl)) {
		LOG_ERR("Devices are not ready");
		return -ENODEV;
	}

	LOG_INF("%s sample, CONFIG_SAMPLE_MOVE_MICROSTEPS=%d", tmc5262_get_variant_name(chip),
		CONFIG_SAMPLE_MOVE_MICROSTEPS);

	stepper_set_event_cb(stepper_driver, driver_event_cb, NULL);
	stepper_ctrl_set_event_cb(stepper_ctrl, ctrl_event_cb, NULL);

	ret = stepper_ctrl_set_microstep_interval(stepper_ctrl, STEP_INTERVAL_NS);
	if (ret < 0) {
		LOG_ERR("Failed to set the microstep interval (%d)", ret);
		return ret;
	}

	ret = stepper_enable(stepper_driver);
	if (ret < 0) {
		LOG_ERR("Failed to enable the stepper (%d)", ret);
		return ret;
	}

	ret = stepper_ctrl_set_reference_position(stepper_ctrl, 0);
	if (ret < 0) {
		LOG_WRN("Could not set the reference position (%d), motor not at standstill?", ret);
	}

	/* Positioning on the ramp generator (TMC5262, TMC3241), STEP/DIR pulses (TMC2262) */
	ret = stepper_ctrl_move_by(stepper_ctrl, CONFIG_SAMPLE_MOVE_MICROSTEPS);
	if (ret < 0) {
		LOG_ERR("Failed to start the move (%d)", ret);
		goto out;
	}

	if (k_sem_take(&steps_completed_sem, K_MSEC(CONFIG_SAMPLE_MOVE_TIMEOUT_MS)) != 0) {
		LOG_WRN("No steps completed event, stopping");
	}

	(void)stepper_ctrl_get_actual_position(stepper_ctrl, &position);
	LOG_INF("Actual position: %d", position);

	report_status();

out:
	/* Stop safely: decelerate, wait for standstill, then disable the power stage */
	(void)stepper_ctrl_stop(stepper_ctrl);
	for (int i = 0; i < 100; i++) {
		if (stepper_ctrl_is_moving(stepper_ctrl, &moving) != 0 || !moving) {
			break;
		}
		k_msleep(20);
	}

	(void)stepper_disable(stepper_driver);
	LOG_INF("Stopped and disabled");

	return ret;
}
