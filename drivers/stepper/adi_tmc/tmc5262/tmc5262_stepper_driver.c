/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-FileCopyrightText: Copyright (c) 2026 Analog Devices Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT adi_tmc5262_stepper_driver

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/stepper/stepper.h>

#include <adi_tmc5xxx_common.h>

#include "tmc5262.h"
#include "tmc5262_reg.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(tmc5262, CONFIG_STEPPER_LOG_LEVEL);

struct tmc5262_stepper_driver_config {
	const uint16_t default_micro_step_res;
	const int8_t sg_threshold;
	const struct gpio_dt_spec en_gpio;
	/* parent controller required for bus communication */
	const struct device *controller;
};

struct tmc5262_stepper_driver_data {
	stepper_event_cb_t drv_event_cb;
	void *drv_event_cb_user_data;
};

void tmc5262_stepper_driver_trigger_cb(const struct device *dev, const enum stepper_event event)
{
	struct tmc5262_stepper_driver_data *data = dev->data;

	if (!data->drv_event_cb) {
		LOG_WRN_ONCE("No stepper driver callback registered");
		return;
	}
	data->drv_event_cb(dev, event, data->drv_event_cb_user_data);
}

static int tmc5262_stepper_driver_set_event_cb(const struct device *dev,
					       stepper_event_cb_t callback, void *user_data)
{
	struct tmc5262_stepper_driver_data *data = dev->data;

	data->drv_event_cb = callback;
	data->drv_event_cb_user_data = user_data;

	return 0;
}

static int tmc5262_stepper_driver_enable(const struct device *dev)
{
	const struct tmc5262_stepper_driver_config *config = dev->config;
	const struct device *controller = config->controller;
	uint32_t reg_value;
	int err;

	LOG_DBG("Enabling Stepper Driver %s", dev->name);

	if (config->en_gpio.port != NULL) {
		err = gpio_pin_set_dt(&config->en_gpio, 1);
		if (err < 0) {
			LOG_ERR("Failed to assert enable GPIO (%d)", err);
			return err;
		}
	}

	err = tmc5262_read(controller, TMC5262_CHOPCONF, &reg_value);
	if (err != 0) {
		return -EIO;
	}

	reg_value |= TMC5XXX_CHOPCONF_DRV_ENABLE_MASK;

	return tmc5262_write(controller, TMC5262_CHOPCONF, reg_value);
}

static int tmc5262_stepper_driver_disable(const struct device *dev)
{
	const struct tmc5262_stepper_driver_config *config = dev->config;
	const struct device *controller = config->controller;
	uint32_t reg_value;
	int err;

	LOG_DBG("Disabling Stepper Driver %s", dev->name);

	if (config->en_gpio.port != NULL) {
		err = gpio_pin_set_dt(&config->en_gpio, 0);
		if (err < 0) {
			LOG_ERR("Failed to deassert enable GPIO (%d)", err);
			return err;
		}
	}

	err = tmc5262_read(controller, TMC5262_CHOPCONF, &reg_value);
	if (err != 0) {
		return -EIO;
	}

	reg_value &= ~TMC5XXX_CHOPCONF_DRV_ENABLE_MASK;

	return tmc5262_write(controller, TMC5262_CHOPCONF, reg_value);
}

static int tmc5262_stepper_driver_set_micro_step_res(const struct device *dev,
						     enum stepper_micro_step_resolution res)
{
	const struct tmc5262_stepper_driver_config *config = dev->config;
	const struct device *controller = config->controller;
	uint32_t reg_value;
	int err;

	err = tmc5262_read(controller, TMC5262_CHOPCONF, &reg_value);
	if (err != 0) {
		return -EIO;
	}

	reg_value &= ~TMC5XXX_CHOPCONF_MRES_MASK;
	reg_value |= ((MICRO_STEP_RES_INDEX(STEPPER_MICRO_STEP_256) - LOG2(res))
		      << TMC5XXX_CHOPCONF_MRES_SHIFT);

	err = tmc5262_write(controller, TMC5262_CHOPCONF, reg_value);
	if (err != 0) {
		return -EIO;
	}

	LOG_DBG("Stepper driver %s set micro step resolution to 0x%x", dev->name, reg_value);
	return 0;
}

static int tmc5262_stepper_driver_get_micro_step_res(const struct device *dev,
						     enum stepper_micro_step_resolution *res)
{
	const struct tmc5262_stepper_driver_config *config = dev->config;
	const struct device *controller = config->controller;
	uint32_t reg_value;
	int err;

	err = tmc5262_read(controller, TMC5262_CHOPCONF, &reg_value);
	if (err != 0) {
		return -EIO;
	}
	reg_value &= TMC5XXX_CHOPCONF_MRES_MASK;
	reg_value >>= TMC5XXX_CHOPCONF_MRES_SHIFT;
	*res = (1 << (MICRO_STEP_RES_INDEX(STEPPER_MICRO_STEP_256) - reg_value));
	LOG_DBG("Stepper driver %s get micro step resolution: %d", dev->name, *res);
	return 0;
}

static int tmc5262_stepper_driver_init(const struct device *dev)
{
	const struct tmc5262_stepper_driver_config *config = dev->config;
	const struct device *controller = config->controller;
	int err;

	if (config->en_gpio.port != NULL) {
		if (!gpio_is_ready_dt(&config->en_gpio)) {
			LOG_ERR("Enable GPIO not ready");
			return -ENODEV;
		}

		err = gpio_pin_configure_dt(&config->en_gpio, GPIO_OUTPUT_INACTIVE);
		if (err < 0) {
			LOG_ERR("Failed to configure enable GPIO (%d)", err);
			return err;
		}
	}

	if (!IN_RANGE(config->sg_threshold, TMC5XXX_SG_MIN_VALUE, TMC5XXX_SG_MAX_VALUE)) {
		LOG_ERR("Stallguard threshold out of range");
		return -EINVAL;
	}

	err = tmc5262_update(controller, TMC5262_SGP_CONF, TMC5262_SGP_THRS_MASK,
			     (uint32_t)config->sg_threshold & TMC5262_SGP_THRS_MASK);
	if (err != 0) {
		return -EIO;
	}

	err = tmc5262_stepper_driver_set_micro_step_res(dev, config->default_micro_step_res);
	if (err != 0) {
		return -EIO;
	}

	LOG_DBG("Stepper driver %s initialized with stallguard %d", dev->name,
		config->sg_threshold);
	return 0;
}

static DEVICE_API(stepper, tmc5262_stepper_driver_api) = {
	.enable = tmc5262_stepper_driver_enable,
	.disable = tmc5262_stepper_driver_disable,
	.set_micro_step_res = tmc5262_stepper_driver_set_micro_step_res,
	.get_micro_step_res = tmc5262_stepper_driver_get_micro_step_res,
	.set_event_cb = tmc5262_stepper_driver_set_event_cb,
};

#define TMC5262_STEPPER_DRIVER_DEFINE(inst)                                                        \
	static const struct tmc5262_stepper_driver_config tmc5262_stepper_driver_config_##inst = { \
		.controller = DEVICE_DT_GET(DT_PARENT(DT_DRV_INST(inst))),                         \
		.default_micro_step_res = DT_INST_PROP(inst, micro_step_res),                      \
		.sg_threshold = DT_INST_PROP(inst, stallguard2_threshold),                         \
		.en_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, en_gpios, {0}),                          \
	};                                                                                         \
	static struct tmc5262_stepper_driver_data tmc5262_stepper_driver_data_##inst;              \
	DEVICE_DT_INST_DEFINE(inst, tmc5262_stepper_driver_init, NULL,                             \
			      &tmc5262_stepper_driver_data_##inst,                                 \
			      &tmc5262_stepper_driver_config_##inst, POST_KERNEL,                  \
			      CONFIG_STEPPER_INIT_PRIORITY, &tmc5262_stepper_driver_api);

DT_INST_FOREACH_STATUS_OKAY(TMC5262_STEPPER_DRIVER_DEFINE)
