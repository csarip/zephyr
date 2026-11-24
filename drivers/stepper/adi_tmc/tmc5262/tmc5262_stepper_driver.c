/*
 * SPDX-FileCopyrightText: Copyright (c) 2026 Analog Devices, Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Zephyr stepper (driver) API for the TMC5262, TMC2262 and TMC3241: power stage enable,
 * microstep resolution and fault / stall events. The node also serves as the
 * "stepper-driver" of a zephyr,gpio-step-dir-stepper-ctrl node, which provides STEP/DIR
 * pulse generation and position tracking (the TMC2262 has no internal ramp generator).
 */

#define DT_DRV_COMPAT adi_tmc5262_stepper_driver

#include <zephyr/drivers/stepper/stepper.h>
#include <zephyr/kernel.h>

#include <step_dir_stepper_common.h>

#include "tmc5262.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(tmc5262, CONFIG_STEPPER_ADI_TMC5262_LOG_LEVEL);

struct tmc5262_stepper_drv_config {
	/* Must be first: read by zephyr,gpio-step-dir-stepper-ctrl */
	struct step_dir_stepper_common_config common;
	const struct device *controller;
	uint16_t default_micro_step_res;
};

struct tmc5262_stepper_drv_data {
	stepper_event_cb_t cb;
	void *cb_user_data;
};

STEP_DIR_STEPPER_STRUCT_CHECK(struct tmc5262_stepper_drv_config);

void tmc5262_stepper_driver_notify(const struct device *drv, enum stepper_event event)
{
	const struct tmc5262_stepper_drv_config *config = drv->config;
	struct tmc5262_stepper_drv_data *data = drv->data;
	stepper_event_cb_t cb;
	void *user_data;

	/* Called from the event work item; the callback is read under the chip lock */
	tmc5262_lock(config->controller);
	cb = data->cb;
	user_data = data->cb_user_data;
	tmc5262_unlock(config->controller);

	if (cb != NULL) {
		cb(drv, event, user_data);
	}
}

static int tmc5262_stepper_drv_enable(const struct device *dev)
{
	const struct tmc5262_stepper_drv_config *config = dev->config;
	const struct device *chip = config->controller;
	bool fault;
	bool recovered = false;
	int err;

	tmc5262_lock(chip);

	err = tmc5262_check_reset(chip);
	if (err < 0) {
		goto out;
	}

	err = tmc5262_fault_present(chip, &fault);
	if (err < 0) {
		goto out;
	}

	/*
	 * A latched fault (drv_err, short, overtemperature) only clears when the driver is
	 * cycled through DRV_ENN / TOFF = 0. Do that before enabling (FR-DIAG-003).
	 */
	if (fault) {
		LOG_WRN("%s: fault latched, cycling the power stage", dev->name);

		err = tmc5262_clear_fault(chip);
		if (err < 0) {
			LOG_ERR("%s: driver fault is still present, cannot enable", dev->name);
			goto out;
		}
		recovered = true;
	}

	LOG_DBG("%s: enabling power stage", dev->name);

	err = tmc5262_power_stage_enable(chip);
	if (err < 0 || !recovered) {
		goto out;
	}

	/* The fault flags set again within a few chopper cycles if the cause is still there */
	k_msleep(5);

	err = tmc5262_fault_present(chip, &fault);
	if (err == 0 && fault) {
		LOG_ERR("%s: fault persists after the power stage cycle", dev->name);
		(void)tmc5262_power_stage_disable(chip);
		err = -EIO;
	}

out:
	tmc5262_unlock(chip);

	return err;
}

static int tmc5262_stepper_drv_disable(const struct device *dev)
{
	const struct tmc5262_stepper_drv_config *config = dev->config;
	int err;

	err = tmc5262_stop_motion(config->controller);
	if (err < 0) {
		LOG_WRN("%s: motion did not stop before disable (%d)", dev->name, err);
	}

	LOG_DBG("%s: disabling power stage", dev->name);

	return tmc5262_power_stage_disable(config->controller);
}

static int tmc5262_stepper_drv_set_micro_step_res(const struct device *dev,
						  enum stepper_micro_step_resolution res)
{
	const struct tmc5262_stepper_drv_config *config = dev->config;
	uint32_t mres;

	if (!VALID_MICRO_STEP_RES(res)) {
		return -EINVAL;
	}

	/* MRES 0 = 256 ... 8 = full step on all variants. On the TMC3241 the value is in
	 * electrical microsteps per quarter sine wave; one full step is 2/3 of that value.
	 */
	mres = 8U - LOG2((uint32_t)res);

	LOG_DBG("%s: microstep resolution 1/%d (MRES %u)", dev->name, res, mres);

	return tmc5262_cfg_update(config->controller, TMC5262_REG_CHOPCONF, TMC5262_CHOPCONF_MRES,
				  FIELD_PREP(TMC5262_CHOPCONF_MRES, mres));
}

static int tmc5262_stepper_drv_get_micro_step_res(const struct device *dev,
						  enum stepper_micro_step_resolution *res)
{
	const struct tmc5262_stepper_drv_config *config = dev->config;
	uint32_t chopconf;
	uint32_t mres;
	int err;

	err = tmc5262_read(config->controller, TMC5262_REG_CHOPCONF, &chopconf);
	if (err < 0) {
		return err;
	}

	mres = FIELD_GET(TMC5262_CHOPCONF_MRES, chopconf);
	/* MRES values above 8 are all full step */
	*res = (enum stepper_micro_step_resolution)(1U << (8U - MIN(mres, 8U)));

	return 0;
}

static int tmc5262_stepper_drv_set_event_cb(const struct device *dev, stepper_event_cb_t callback,
					    void *user_data)
{
	const struct tmc5262_stepper_drv_config *config = dev->config;
	struct tmc5262_stepper_drv_data *data = dev->data;
	bool had_cb;

	tmc5262_lock(config->controller);

	had_cb = data->cb != NULL;
	data->cb = callback;
	data->cb_user_data = user_data;

	if (callback != NULL && !had_cb) {
		tmc5262_event_users_add(config->controller);
	} else if (callback == NULL && had_cb) {
		tmc5262_event_users_remove(config->controller);
	}

	tmc5262_unlock(config->controller);

	return 0;
}

static int tmc5262_stepper_drv_init(const struct device *dev)
{
	const struct tmc5262_stepper_drv_config *config = dev->config;

	if (!device_is_ready(config->controller)) {
		LOG_ERR_DEVICE_NOT_READY(config->controller);
		return -ENODEV;
	}

	return tmc5262_stepper_drv_set_micro_step_res(dev, config->default_micro_step_res);
}

static DEVICE_API(stepper, tmc5262_stepper_driver_api) = {
	.enable = tmc5262_stepper_drv_enable,
	.disable = tmc5262_stepper_drv_disable,
	.set_micro_step_res = tmc5262_stepper_drv_set_micro_step_res,
	.get_micro_step_res = tmc5262_stepper_drv_get_micro_step_res,
	.set_event_cb = tmc5262_stepper_drv_set_event_cb,
};

/* CHOPCONF.dedge of the chip (see the parent binding) is the dual edge setting */
#define TMC5262_STEPPER_DRV_DEFINE(inst)                                                           \
	static const struct tmc5262_stepper_drv_config tmc5262_stepper_drv_config_##inst = {       \
		.common.step_width_ns = DT_INST_PROP(inst, step_width_ns),                         \
		.common.dual_edge = DT_PROP(DT_INST_PARENT(inst), dedge),                          \
		.controller = DEVICE_DT_GET(DT_INST_PARENT(inst)),                                 \
		.default_micro_step_res = DT_INST_PROP(inst, micro_step_res),                      \
	};                                                                                         \
	static struct tmc5262_stepper_drv_data tmc5262_stepper_drv_data_##inst;                    \
	DEVICE_DT_INST_DEFINE(inst, tmc5262_stepper_drv_init, NULL,                                \
			      &tmc5262_stepper_drv_data_##inst,                                    \
			      &tmc5262_stepper_drv_config_##inst, POST_KERNEL,                     \
			      CONFIG_STEPPER_INIT_PRIORITY, &tmc5262_stepper_driver_api);

DT_INST_FOREACH_STATUS_OKAY(TMC5262_STEPPER_DRV_DEFINE)
