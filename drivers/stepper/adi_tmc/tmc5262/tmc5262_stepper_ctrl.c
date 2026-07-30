/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-FileCopyrightText: Copyright (c) 2026 Analog Devices Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT adi_tmc5262_stepper_ctrl

#include <stdlib.h>

#include <zephyr/drivers/stepper/stepper_ctrl.h>

#include <adi_tmc5xxx_common.h>

#include "tmc5262.h"
#include "tmc5262_reg.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(tmc5262, CONFIG_STEPPER_LOG_LEVEL);

struct tmc5262_stepper_ctrl_config {
	const bool is_sg_enabled;
	const uint32_t sg_threshold_velocity;
	const uint32_t sg_velocity_check_interval_ms;
#ifdef CONFIG_STEPPER_ADI_TMC5262_RAMP_GEN
	const struct tmc5262_ramp_generator_data default_ramp_config;
#endif
	/* parent controller required for bus communication */
	const struct device *controller;
};

struct tmc5262_stepper_ctrl_data {
	struct k_work_delayable stallguard_dwork;
	const struct device *dev;
	stepper_ctrl_event_callback_t callback;
	void *event_cb_user_data;
};

void tmc5262_stepper_ctrl_trigger_cb(const struct device *dev, const enum stepper_ctrl_event event)
{
	struct tmc5262_stepper_ctrl_data *data = dev->data;

	if (!data->callback) {
		LOG_WRN_ONCE("No motion controller callback registered");
		return;
	}
	data->callback(dev, event, data->event_cb_user_data);
}

static int read_vactual(const struct device *dev, int32_t *actual_velocity)
{
	const struct tmc5262_stepper_ctrl_config *config = dev->config;
	const struct device *controller = config->controller;
	uint32_t raw_value;
	int err;

	err = tmc5262_read(controller, TMC5262_VACTUAL, &raw_value);
	if (err) {
		LOG_ERR("Failed to read VACTUAL register");
		return err;
	}

	*actual_velocity = sign_extend(raw_value, TMC_RAMP_VACTUAL_SHIFT);
	return 0;
}

int tmc5262_stepper_ctrl_stallguard_enable(const struct device *dev, const bool enable)
{
	const struct tmc5262_stepper_ctrl_config *config = dev->config;
	const struct device *controller = config->controller;
	uint32_t reg_value;
	int err;

	err = tmc5262_read(controller, TMC5262_SWMODE, &reg_value);
	if (err) {
		LOG_ERR("Failed to read SWMODE register");
		return -EIO;
	}

	if (enable) {
		reg_value |= TMC5XXX_SW_MODE_SG_STOP_ENABLE;

		int32_t actual_velocity;

		err = read_vactual(dev, &actual_velocity);
		if (err) {
			return -EIO;
		}
		if (abs(actual_velocity) < config->sg_threshold_velocity) {
			return -EAGAIN;
		}
	} else {
		reg_value &= ~TMC5XXX_SW_MODE_SG_STOP_ENABLE;
	}
	err = tmc5262_write(controller, TMC5262_SWMODE, reg_value);
	if (err) {
		LOG_ERR("Failed to write SWMODE register");
		return -EIO;
	}

	LOG_DBG("Stallguard %s", enable ? "enabled" : "disabled");
	return 0;
}

static void stallguard_work_handler(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct tmc5262_stepper_ctrl_data const *data =
		CONTAINER_OF(dwork, struct tmc5262_stepper_ctrl_data, stallguard_dwork);
	const struct device *dev = data->dev;
	const struct tmc5262_stepper_ctrl_config *config = dev->config;
	int err;

	err = tmc5262_stepper_ctrl_stallguard_enable(dev, true);
	if (err == -EAGAIN) {
		k_work_reschedule(dwork, K_MSEC(config->sg_velocity_check_interval_ms));
	}
	if (err == -EIO) {
		LOG_ERR("Failed to enable stallguard because of I/O error");
	}
}

static int tmc5262_stepper_ctrl_set_event_cb(const struct device *dev,
					     stepper_ctrl_event_callback_t callback,
					     void *user_data)
{
	struct tmc5262_stepper_ctrl_data *data = dev->data;

	data->callback = callback;
	data->event_cb_user_data = user_data;

	return 0;
}

static int tmc5262_stepper_ctrl_is_moving(const struct device *dev, bool *is_moving)
{
	const struct tmc5262_stepper_ctrl_config *config = dev->config;
	const struct device *controller = config->controller;
	uint32_t reg_value;
	int err;

	err = tmc5262_read(controller, TMC5262_DRVSTATUS, &reg_value);
	if (err != 0) {
		LOG_ERR("%s: Failed to read DRVSTATUS register", dev->name);
		return -EIO;
	}

	*is_moving = (FIELD_GET(TMC5XXX_DRV_STATUS_STST_BIT, reg_value) != 1U);
	LOG_DBG("Motion controller %s is moving: %d", dev->name, *is_moving);
	return 0;
}

static int tmc5262_stepper_ctrl_set_reference_position(const struct device *dev,
						       const int32_t position)
{
	const struct tmc5262_stepper_ctrl_config *config = dev->config;
	const struct device *controller = config->controller;
	int err;

	err = tmc5262_write(controller, TMC5262_RAMPMODE, TMC5XXX_RAMPMODE_HOLD_MODE);
	if (err != 0) {
		return -EIO;
	}

	err = tmc5262_write(controller, TMC5262_XACTUAL, position);
	if (err != 0) {
		return -EIO;
	}
	LOG_DBG("Motion controller %s set actual position to %d", dev->name, position);
	return 0;
}

static int tmc5262_stepper_ctrl_get_actual_position(const struct device *dev, int32_t *position)
{
	const struct tmc5262_stepper_ctrl_config *config = dev->config;
	int err;

	err = tmc5262_read_actual_position(config->controller, position);
	if (err != 0) {
		return -EIO;
	}
	LOG_DBG("%s actual position: %d", dev->name, *position);
	return 0;
}

static int tmc5262_stepper_ctrl_move_to(const struct device *dev, const int32_t micro_steps)
{
	const struct tmc5262_stepper_ctrl_config *config = dev->config;
	struct tmc5262_stepper_ctrl_data *data = dev->data;
	const struct device *controller = config->controller;
	int err;

	LOG_DBG("%s set target position to %d", dev->name, micro_steps);

	if (config->is_sg_enabled) {
		tmc5262_stepper_ctrl_stallguard_enable(dev, false);
	}

	err = tmc5262_write(controller, TMC5262_RAMPMODE, TMC5XXX_RAMPMODE_POSITIONING_MODE);
	if (err != 0) {
		return -EIO;
	}
	err = tmc5262_write(controller, TMC5262_XTARGET, micro_steps);
	if (err != 0) {
		return -EIO;
	}

	if (config->is_sg_enabled) {
		k_work_reschedule(&data->stallguard_dwork,
				  K_MSEC(config->sg_velocity_check_interval_ms));
	}
	if (data->callback) {
		/* For SPI with DIAG0 pin, we use an interrupt-driven approach */
		if (tmc5262_is_interrupt_driven(controller)) {
			return 0;
		}
		/* For SPI without DIAG0, reschedule RAMPSTAT polling */
		tmc5262_reschedule_rampstat_callback(controller);
	}
	return 0;
}

static int tmc5262_stepper_ctrl_move_by(const struct device *dev, const int32_t micro_steps)
{
	int32_t position;
	int err;

	err = tmc5262_stepper_ctrl_get_actual_position(dev, &position);
	if (err != 0) {
		return -EIO;
	}
	int32_t target_position = position + micro_steps;

	LOG_DBG("%s moved to %d by steps: %d", dev->name, target_position, micro_steps);

	return tmc5262_stepper_ctrl_move_to(dev, target_position);
}

static int tmc5262_stepper_ctrl_run(const struct device *dev,
				    const enum stepper_ctrl_direction direction)
{
	const struct tmc5262_stepper_ctrl_config *config = dev->config;
	struct tmc5262_stepper_ctrl_data *data = dev->data;
	const struct device *controller = config->controller;
	int err;

	LOG_DBG("Motion controller %s run", dev->name);

	if (config->is_sg_enabled) {
		err = tmc5262_stepper_ctrl_stallguard_enable(dev, false);
		if (err != 0) {
			return -EIO;
		}
	}

	switch (direction) {
	case STEPPER_CTRL_DIRECTION_POSITIVE:
		err = tmc5262_write(controller, TMC5262_RAMPMODE,
				    TMC5XXX_RAMPMODE_POSITIVE_VELOCITY_MODE);
		if (err != 0) {
			return -EIO;
		}
		break;

	case STEPPER_CTRL_DIRECTION_NEGATIVE:
		err = tmc5262_write(controller, TMC5262_RAMPMODE,
				    TMC5XXX_RAMPMODE_NEGATIVE_VELOCITY_MODE);
		if (err != 0) {
			return -EIO;
		}
		break;
	}

	if (config->is_sg_enabled) {
		k_work_reschedule(&data->stallguard_dwork,
				  K_MSEC(config->sg_velocity_check_interval_ms));
	}
	if (data->callback) {
		/* For SPI with DIAG0 pin, we use an interrupt-driven approach */
		if (tmc5262_is_interrupt_driven(controller)) {
			return 0;
		}
		/* For SPI without DIAG0, reschedule RAMPSTAT polling */
		tmc5262_reschedule_rampstat_callback(controller);
	}
	return 0;
}

static int tmc5262_stepper_ctrl_stop(const struct device *dev)
{
	const struct tmc5262_stepper_ctrl_config *config = dev->config;
	const struct device *controller = config->controller;
	int err;

	err = tmc5262_write(controller, TMC5262_RAMPMODE, TMC5XXX_RAMPMODE_POSITIVE_VELOCITY_MODE);
	if (err != 0) {
		return -EIO;
	}

	err = tmc5262_write(controller, TMC5262_VMAX, 0);
	if (err != 0) {
		return -EIO;
	}

	return 0;
}

static int tmc5262_stepper_ctrl_configure_ramp(const struct device *dev,
					       const struct stepper_ctrl_ramp *ramp)
{
	const struct tmc5262_stepper_ctrl_config *config = dev->config;
	const struct device *controller = config->controller;
	const uint32_t clock_frequency = tmc5262_get_clock_frequency(controller);
	uint32_t velocity_fclk =
		tmc5xxx_calculate_velocity_from_hz_to_fclk(ramp->speed_max, clock_frequency);
	uint32_t accel_fclk = tmc5xxx_calculate_acceleration_from_hz_to_fclk(ramp->acceleration_max,
									     clock_frequency);
	uint32_t decel_fclk = tmc5xxx_calculate_acceleration_from_hz_to_fclk(ramp->deceleration_max,
									     clock_frequency);
	int err;

	LOG_DBG("Motion controller %s configure ramp", dev->name);

	err = tmc5262_write(controller, TMC5262_A1, accel_fclk);
	if (err != 0) {
		return -EIO;
	}
	err = tmc5262_write(controller, TMC5262_AMAX, accel_fclk);
	if (err != 0) {
		return -EIO;
	}
	err = tmc5262_write(controller, TMC5262_VMAX, velocity_fclk);
	if (err != 0) {
		return -EIO;
	}
	err = tmc5262_write(controller, TMC5262_DMAX, decel_fclk);
	if (err != 0) {
		return -EIO;
	}
	err = tmc5262_write(controller, TMC5262_D1, decel_fclk);
	if (err != 0) {
		return -EIO;
	}

	return 0;
}

#ifdef CONFIG_STEPPER_ADI_TMC5262_RAMP_GEN

static int tmc5262_stepper_ctrl_set_ramp(const struct device *dev,
					 const struct tmc5262_ramp_generator_data *ramp_data)
{
	const struct tmc5262_stepper_ctrl_config *config = dev->config;
	const struct device *controller = config->controller;
	int err;

	LOG_DBG("Motion controller %s set ramp", dev->name);

	err = tmc5262_write(controller, TMC5262_VSTART, ramp_data->vstart);
	if (err != 0) {
		return -EIO;
	}
	err = tmc5262_write(controller, TMC5262_A1, ramp_data->a1);
	if (err != 0) {
		return -EIO;
	}
	err = tmc5262_write(controller, TMC5262_A2, ramp_data->a2);
	if (err != 0) {
		return -EIO;
	}
	err = tmc5262_write(controller, TMC5262_AMAX, ramp_data->amax);
	if (err != 0) {
		return -EIO;
	}
	err = tmc5262_write(controller, TMC5262_D1, ramp_data->d1);
	if (err != 0) {
		return -EIO;
	}
	err = tmc5262_write(controller, TMC5262_D2, ramp_data->d2);
	if (err != 0) {
		return -EIO;
	}
	err = tmc5262_write(controller, TMC5262_DMAX, ramp_data->dmax);
	if (err != 0) {
		return -EIO;
	}
	err = tmc5262_write(controller, TMC5262_V1, ramp_data->v1);
	if (err != 0) {
		return -EIO;
	}
	err = tmc5262_write(controller, TMC5262_V2, ramp_data->v2);
	if (err != 0) {
		return -EIO;
	}
	err = tmc5262_write(controller, TMC5262_VMAX, ramp_data->vmax);
	if (err != 0) {
		return -EIO;
	}
	err = tmc5262_write(controller, TMC5262_VSTOP, ramp_data->vstop);
	if (err != 0) {
		return -EIO;
	}
	err = tmc5262_write(controller, TMC5262_TZEROWAIT, ramp_data->tzerowait);
	if (err != 0) {
		return -EIO;
	}
	err = tmc5262_write(controller, TMC5262_THIGH, ramp_data->thigh);
	if (err != 0) {
		return -EIO;
	}
	err = tmc5262_write(controller, TMC5262_TCOOLTHRS, ramp_data->tcoolthrs);
	if (err != 0) {
		return -EIO;
	}
	err = tmc5262_write(controller, TMC5262_TPWMTHRS, ramp_data->tpwmthrs);
	if (err != 0) {
		return -EIO;
	}
	err = tmc5262_write(controller, TMC5262_TPOWER_DOWN, ramp_data->tpowerdown);
	if (err != 0) {
		return -EIO;
	}
	err = tmc5262_write(controller, TMC5262_IHOLD_IRUN, ramp_data->iholdrun);
	if (err != 0) {
		return -EIO;
	}
	return 0;
}

#endif /* CONFIG_STEPPER_ADI_TMC5262_RAMP_GEN */

static int tmc5262_stepper_ctrl_init(const struct device *dev)
{
	const struct tmc5262_stepper_ctrl_config *config = dev->config;
	struct tmc5262_stepper_ctrl_data *data = dev->data;
	const struct device *controller = config->controller;
	int err;

	data->dev = dev;

	k_work_init_delayable(&data->stallguard_dwork, stallguard_work_handler);

	err = tmc5262_update(controller, TMC5262_SWMODE, TMC5262_SG_STOP_MASK, 1);
	if (err != 0) {
		return -EIO;
	}

	if (config->is_sg_enabled) {
		LOG_DBG("stallguard delay %d ms", config->sg_velocity_check_interval_ms);
		k_work_reschedule(&data->stallguard_dwork, K_NO_WAIT);
	}

#ifdef CONFIG_STEPPER_ADI_TMC5262_RAMP_GEN
	err = tmc5262_stepper_ctrl_set_ramp(dev, &config->default_ramp_config);
	if (err != 0) {
		return -EIO;
	}
#endif
	return 0;
}

static DEVICE_API(stepper_ctrl, tmc5262_stepper_ctrl_api) = {
	.is_moving = tmc5262_stepper_ctrl_is_moving,
	.move_by = tmc5262_stepper_ctrl_move_by,
	.set_reference_position = tmc5262_stepper_ctrl_set_reference_position,
	.get_actual_position = tmc5262_stepper_ctrl_get_actual_position,
	.move_to = tmc5262_stepper_ctrl_move_to,
	.run = tmc5262_stepper_ctrl_run,
	.stop = tmc5262_stepper_ctrl_stop,
	.set_event_cb = tmc5262_stepper_ctrl_set_event_cb,
	.configure_ramp = tmc5262_stepper_ctrl_configure_ramp,
};

/* clang-format off */
#define TMC5262_STEPPER_CTRL_DEFINE(inst)                                                          \
	IF_ENABLED(CONFIG_STEPPER_ADI_TMC5262_RAMP_GEN, (CHECK_RAMP_DT_DATA(inst)));               \
	static const struct tmc5262_stepper_ctrl_config tmc5262_stepper_ctrl_cfg_##inst = {        \
		.controller = DEVICE_DT_GET(DT_PARENT(DT_DRV_INST(inst))),                         \
		.is_sg_enabled = DT_INST_PROP(inst, activate_stallguard2),                         \
		.sg_threshold_velocity = DT_INST_PROP(inst, stallguard_threshold_velocity),        \
		.sg_velocity_check_interval_ms =                                                   \
			DT_INST_PROP(inst, stallguard_velocity_check_interval_ms),                 \
		IF_ENABLED(CONFIG_STEPPER_ADI_TMC5262_RAMP_GEN,                                    \
			   (.default_ramp_config = TMC_RAMP_DT_SPEC_GET_TMC5262(inst)))};          \
	static struct tmc5262_stepper_ctrl_data tmc5262_stepper_ctrl_data_##inst;                  \
	DEVICE_DT_INST_DEFINE(inst, tmc5262_stepper_ctrl_init, NULL,                               \
			      &tmc5262_stepper_ctrl_data_##inst, &tmc5262_stepper_ctrl_cfg_##inst, \
			      POST_KERNEL, CONFIG_STEPPER_INIT_PRIORITY,                           \
			      &tmc5262_stepper_ctrl_api);
/* clang-format on */

DT_INST_FOREACH_STATUS_OKAY(TMC5262_STEPPER_CTRL_DEFINE)
