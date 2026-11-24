/*
 * SPDX-FileCopyrightText: Copyright (c) 2026 Analog Devices, Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Zephyr stepper_ctrl API on the internal ramp generator of the TMC5262 and TMC3241.
 * The TMC2262 has no ramp generator: use zephyr,gpio-step-dir-stepper-ctrl with
 * stepper-driver pointing to the adi,tmc5262-stepper-driver node instead.
 *
 * VMAX safety rule: a non-zero VMAX is only written together with an intentional motion
 * (positioning move, velocity run) or while the motor is already running. A stop never
 * leaves the generator in velocity mode: it switches to positioning mode with
 * XTARGET = XACTUAL so that the motor decelerates with DMAX / D1 / D2.
 *
 * All sequences hold the chip lock (recursive) for their whole duration.
 */

#define DT_DRV_COMPAT adi_tmc5262_stepper_ctrl

#include <zephyr/drivers/stepper/stepper_ctrl.h>
#include <zephyr/kernel.h>

#include "tmc5262.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(tmc5262, CONFIG_STEPPER_ADI_TMC5262_LOG_LEVEL);

/* Ramp parameters in register units (FR-MOTION-007) */
struct tmc5262_ramp {
	uint32_t vstart;
	uint32_t a1;
	uint32_t v1;
	uint32_t amax;
	uint32_t vmax;
	uint32_t dmax;
	uint32_t d1;
	uint32_t vstop;
	uint32_t tzerowait;
	uint32_t v2;
	uint32_t a2;
	uint32_t d2;
};

struct tmc5262_stepper_ctrl_config {
	const struct device *controller;
	struct tmc5262_ramp default_ramp;
};

struct tmc5262_stepper_ctrl_data {
	stepper_ctrl_event_callback_t cb;
	void *cb_user_data;
	/* Cached ramp, VMAX is written to the device only by a motion command */
	struct tmc5262_ramp ramp;
	bool positioning;
	bool stopping;
};

/* The chip lock is held by the callers of these two */
bool tmc5262_stepper_ctrl_wants_events(const struct device *ctrl)
{
	struct tmc5262_stepper_ctrl_data *data = ctrl->data;

	return data->cb != NULL;
}

bool tmc5262_stepper_ctrl_consume_stop(const struct device *ctrl)
{
	struct tmc5262_stepper_ctrl_data *data = ctrl->data;
	bool stopping = data->stopping;

	data->stopping = false;

	return stopping;
}

void tmc5262_stepper_ctrl_notify(const struct device *ctrl, enum stepper_ctrl_event event)
{
	const struct tmc5262_stepper_ctrl_config *config = ctrl->config;
	struct tmc5262_stepper_ctrl_data *data = ctrl->data;
	stepper_ctrl_event_callback_t cb;
	void *user_data;

	/* Called from the event work item; the callback is read under the chip lock */
	tmc5262_lock(config->controller);
	cb = data->cb;
	user_data = data->cb_user_data;
	tmc5262_unlock(config->controller);

	if (cb != NULL) {
		cb(ctrl, event, user_data);
	}
}

static int tmc5262_ramp_validate(const struct tmc5262_ramp *r)
{
	if ((r->vstart & ~TMC5262_VSTART_MASK) != 0U || (r->a1 & ~TMC5262_A1_MASK) != 0U ||
	    (r->v1 & ~TMC5262_V1_MASK) != 0U || (r->amax & ~TMC5262_AMAX_MASK) != 0U ||
	    (r->vmax & ~TMC5262_VMAX_MASK) != 0U || (r->dmax & ~TMC5262_DMAX_MASK) != 0U ||
	    (r->d1 & ~TMC5262_D1_MASK) != 0U || (r->vstop & ~TMC5262_VSTOP_MASK) != 0U ||
	    (r->tzerowait & ~TMC5262_TZEROWAIT_MASK) != 0U || (r->v2 & ~TMC5262_V2_MASK) != 0U ||
	    (r->a2 & ~TMC5262_A2_MASK) != 0U || (r->d2 & ~TMC5262_D2_MASK) != 0U) {
		return -EINVAL;
	}

	/* A motion needs a velocity and both ramps, D1 must not be 0 in positioning mode */
	if (r->vmax == 0U || r->amax == 0U || r->dmax == 0U || r->d1 == 0U) {
		return -EINVAL;
	}

	/* Datasheet: always set VSTOP identical to or above VSTART */
	if (r->vstop < r->vstart) {
		return -EINVAL;
	}

	return 0;
}

/* Write all ramp registers. VMAX only if @p with_vmax. */
static int tmc5262_ramp_write(const struct device *controller, const struct tmc5262_ramp *r,
			      bool with_vmax, bool with_tzerowait)
{
	const struct {
		uint8_t reg;
		uint32_t val;
	} list[] = {
		{TMC5262_REG_VSTART, r->vstart},
		{TMC5262_REG_A1, r->a1},
		{TMC5262_REG_V1, r->v1},
		{TMC5262_REG_A2, r->a2},
		{TMC5262_REG_V2, r->v2},
		{TMC5262_REG_AMAX, r->amax},
		{TMC5262_REG_DMAX, r->dmax},
		{TMC5262_REG_D2, r->d2},
		{TMC5262_REG_D1, r->d1},
		{TMC5262_REG_VSTOP, r->vstop},
		{TMC5262_REG_TZEROWAIT, r->tzerowait},
	};
	int err;

	for (size_t i = 0; i < ARRAY_SIZE(list); i++) {
		/* A pending stop holds TZEROWAIT at its maximum, it is restored at standstill */
		if (list[i].reg == TMC5262_REG_TZEROWAIT && !with_tzerowait) {
			continue;
		}

		err = tmc5262_write(controller, list[i].reg, list[i].val);
		if (err < 0) {
			return err;
		}
	}

	if (with_vmax) {
		return tmc5262_write(controller, TMC5262_REG_VMAX, r->vmax);
	}

	return 0;
}

/* Common pre-condition of every motion command, the chip lock is held */
static int tmc5262_ctrl_prepare(const struct device *dev)
{
	const struct tmc5262_stepper_ctrl_config *config = dev->config;
	int err;

	err = tmc5262_require(config->controller, TMC5262_CAP_RAMP_GEN);
	if (err < 0) {
		return err;
	}

	/* FR-MOTION-003: no ramp generator motion in STEP/DIR operation */
	if (!tmc5262_ramp_is_step_source(config->controller)) {
		return -ENOTSUP;
	}

	/* FR-POWER-004 */
	return tmc5262_check_reset(config->controller);
}

/* After a reset or wake-up the registers are at their defaults, the chip lock is held */
int tmc5262_stepper_ctrl_reapply(const struct device *ctrl, bool lost)
{
	const struct tmc5262_stepper_ctrl_config *config = ctrl->config;
	struct tmc5262_stepper_ctrl_data *data = ctrl->data;
	const struct device *chip = config->controller;
	int err = 0;

	if (!lost) {
		/* The register bank is intact (motion may be running): only rewrite the ramp */
		return tmc5262_ramp_write(chip, &data->ramp, false, true);
	}

	/* Reset values: make sure nothing moves: VMAX 0, positioning mode, XTARGET = XACTUAL = 0 */
	err = tmc5262_write(chip, TMC5262_REG_VMAX, 0);
	if (err == 0) {
		err = tmc5262_write(chip, TMC5262_REG_RAMPMODE, TMC5262_RAMPMODE_POSITION);
	}
	if (err == 0) {
		err = tmc5262_write(chip, TMC5262_REG_XACTUAL, 0);
	}
	if (err == 0) {
		err = tmc5262_write(chip, TMC5262_REG_XTARGET, 0);
	}
	if (err == 0) {
		err = tmc5262_ramp_write(chip, &data->ramp, true, true);
	}

	data->positioning = true;
	data->stopping = false;

	return err;
}

/* Position mode move, the chip lock is held. Order: mode, VMAX, XTARGET. */
static int tmc5262_ctrl_goto(const struct device *dev, int32_t target)
{
	const struct tmc5262_stepper_ctrl_config *config = dev->config;
	struct tmc5262_stepper_ctrl_data *data = dev->data;
	const struct device *chip = config->controller;
	uint32_t rampmode;
	int err;

	/* A new move replaces a stop that is still completing */
	err = tmc5262_ramp_stop_cancel(chip);
	if (err < 0) {
		return err;
	}

	/* The actual mode decides, not the cache: tmc5262_abort() leaves hold mode behind */
	err = tmc5262_read(chip, TMC5262_REG_RAMPMODE, &rampmode);
	if (err < 0) {
		return err;
	}

	if ((rampmode & GENMASK(1, 0)) == TMC5262_RAMPMODE_POSITION) {
		/* Positioning mode with a valid target: VMAX first (no motion while idle) */
		err = tmc5262_write(chip, TMC5262_REG_VMAX, data->ramp.vmax);
		if (err == 0) {
			err = tmc5262_write(chip, TMC5262_REG_RAMPMODE, TMC5262_RAMPMODE_POSITION);
		}
		if (err == 0) {
			data->positioning = true;
			data->stopping = false;
			/* Writing XTARGET starts the motion, which then runs without the CPU */
			err = tmc5262_write(chip, TMC5262_REG_XTARGET, (uint32_t)target);
		}

		return err;
	}

	/*
	 * Coming from velocity or hold mode: XTARGET is unused there (and may be stale after an
	 * abort), so write the new target before the mode switch, VMAX last.
	 */
	err = tmc5262_write(chip, TMC5262_REG_XTARGET, (uint32_t)target);
	if (err == 0) {
		err = tmc5262_write(chip, TMC5262_REG_RAMPMODE, TMC5262_RAMPMODE_POSITION);
	}
	if (err == 0) {
		data->positioning = true;
		data->stopping = false;
		err = tmc5262_write(chip, TMC5262_REG_VMAX, data->ramp.vmax);
	}

	return err;
}

static int tmc5262_ctrl_move_to(const struct device *dev, const int32_t micro_steps)
{
	const struct tmc5262_stepper_ctrl_config *config = dev->config;
	int err;

	tmc5262_lock(config->controller);

	err = tmc5262_ctrl_prepare(dev);
	if (err == 0) {
		LOG_DBG("%s: move to %d", dev->name, micro_steps);
		err = tmc5262_ctrl_goto(dev, micro_steps);
	}

	tmc5262_unlock(config->controller);

	return err;
}

static int tmc5262_ctrl_move_by(const struct device *dev, const int32_t micro_steps)
{
	const struct tmc5262_stepper_ctrl_config *config = dev->config;
	uint32_t xactual;
	int32_t target;
	int err;

	tmc5262_lock(config->controller);

	err = tmc5262_ctrl_prepare(dev);
	if (err == 0) {
		err = tmc5262_read(config->controller, TMC5262_REG_XACTUAL, &xactual);
	}
	if (err == 0 && __builtin_add_overflow((int32_t)xactual, micro_steps, &target)) {
		err = -EINVAL;
	}
	if (err == 0) {
		err = tmc5262_ctrl_goto(dev, target);
	}

	tmc5262_unlock(config->controller);

	return err;
}

static int tmc5262_ctrl_run(const struct device *dev, const enum stepper_ctrl_direction direction)
{
	const struct tmc5262_stepper_ctrl_config *config = dev->config;
	struct tmc5262_stepper_ctrl_data *data = dev->data;
	int err;

	tmc5262_lock(config->controller);

	err = tmc5262_ctrl_prepare(dev);
	if (err == 0) {
		/* A velocity run replaces a stop that is still completing */
		err = tmc5262_ramp_stop_cancel(config->controller);
	}
	if (err == 0) {
		/* An intentional velocity run: VMAX, then the velocity mode starts it */
		err = tmc5262_write(config->controller, TMC5262_REG_VMAX, data->ramp.vmax);
	}
	if (err == 0) {
		data->positioning = false;
		data->stopping = false;
		err = tmc5262_write(config->controller, TMC5262_REG_RAMPMODE,
				    direction == STEPPER_CTRL_DIRECTION_POSITIVE
					    ? TMC5262_RAMPMODE_VELOCITY_POS
					    : TMC5262_RAMPMODE_VELOCITY_NEG);
	}

	tmc5262_unlock(config->controller);

	return err;
}

static int tmc5262_ctrl_stop(const struct device *dev)
{
	const struct tmc5262_stepper_ctrl_config *config = dev->config;
	struct tmc5262_stepper_ctrl_data *data = dev->data;
	int err;

	tmc5262_lock(config->controller);

	err = tmc5262_ctrl_prepare(dev);
	if (err == 0) {
		/* Positioning mode, XTARGET = XACTUAL: decelerate with DMAX / D1 / D2 */
		err = tmc5262_ramp_stop(config->controller, false);
	}
	if (err == 0) {
		data->positioning = true;
		data->stopping = true;
	}

	tmc5262_unlock(config->controller);

	return err;
}

static int tmc5262_ctrl_is_moving(const struct device *dev, bool *is_moving)
{
	const struct tmc5262_stepper_ctrl_config *config = dev->config;
	struct tmc5262_stepper_ctrl_data *data = dev->data;
	uint32_t ramp_stat;
	int err;

	err = tmc5262_require(config->controller, TMC5262_CAP_RAMP_GEN);
	if (err < 0) {
		return err;
	}

	tmc5262_lock(config->controller);

	err = tmc5262_read(config->controller, TMC5262_REG_RAMP_STAT, &ramp_stat);
	if (err == 0) {
		*is_moving = (ramp_stat & TMC5262_RAMP_STAT_VZERO) == 0U ||
			     (data->positioning &&
			      (ramp_stat & TMC5262_RAMP_STAT_POSITION_REACHED) == 0U);
	}

	tmc5262_unlock(config->controller);

	return err;
}

static int tmc5262_ctrl_set_reference_position(const struct device *dev, const int32_t value)
{
	const struct tmc5262_stepper_ctrl_config *config = dev->config;
	struct tmc5262_stepper_ctrl_data *data = dev->data;
	const struct device *chip = config->controller;
	int err;

	err = tmc5262_require(chip, TMC5262_CAP_RAMP_GEN);
	if (err < 0) {
		return err;
	}

	tmc5262_lock(chip);

	/* FR-MOTION-012: only at standstill. A stop that is still completing is cancelled. */
	err = tmc5262_check_standstill(chip);
	if (err == 0) {
		err = tmc5262_ramp_stop_cancel(chip);
	}

	/* Hold while XACTUAL and XTARGET are rewritten so that the motor does not move */
	if (err == 0) {
		err = tmc5262_write(chip, TMC5262_REG_RAMPMODE, TMC5262_RAMPMODE_HOLD);
	}
	if (err == 0) {
		err = tmc5262_write(chip, TMC5262_REG_XACTUAL, (uint32_t)value);
	}
	if (err == 0) {
		err = tmc5262_write(chip, TMC5262_REG_XTARGET, (uint32_t)value);
	}
	if (err == 0) {
		data->positioning = true;
		err = tmc5262_write(chip, TMC5262_REG_RAMPMODE, TMC5262_RAMPMODE_POSITION);
	}

	tmc5262_unlock(chip);

	return err;
}

static int tmc5262_ctrl_get_actual_position(const struct device *dev, int32_t *value)
{
	const struct tmc5262_stepper_ctrl_config *config = dev->config;
	uint32_t xactual;
	int err;

	err = tmc5262_require(config->controller, TMC5262_CAP_RAMP_GEN);
	if (err < 0) {
		return err;
	}

	err = tmc5262_read(config->controller, TMC5262_REG_XACTUAL, &xactual);
	if (err < 0) {
		return err;
	}

	*value = (int32_t)xactual;

	return 0;
}

static int tmc5262_ctrl_set_event_cb(const struct device *dev, stepper_ctrl_event_callback_t cb,
				     void *user_data)
{
	const struct tmc5262_stepper_ctrl_config *config = dev->config;
	struct tmc5262_stepper_ctrl_data *data = dev->data;
	bool had_cb;

	tmc5262_lock(config->controller);

	had_cb = data->cb != NULL;
	data->cb = cb;
	data->cb_user_data = user_data;

	if (cb != NULL && !had_cb) {
		tmc5262_event_users_add(config->controller);
	} else if (cb == NULL && had_cb) {
		tmc5262_event_users_remove(config->controller);
	}

	tmc5262_unlock(config->controller);

	return 0;
}

/*
 * Apply a changed ramp: the cache is always updated. The acceleration registers are
 * written, VMAX only if the motor is running at the moment (a change of the target
 * velocity is then wanted); an idle motor takes the new VMAX with the next motion command.
 */
static int tmc5262_ctrl_apply_ramp(const struct device *dev, const struct tmc5262_ramp *r)
{
	const struct tmc5262_stepper_ctrl_config *config = dev->config;
	struct tmc5262_stepper_ctrl_data *data = dev->data;
	uint32_t ramp_stat;
	bool running;
	int err;

	err = tmc5262_read(config->controller, TMC5262_REG_RAMP_STAT, &ramp_stat);
	if (err < 0) {
		return err;
	}

	running = (ramp_stat & TMC5262_RAMP_STAT_VZERO) == 0U;

	/*
	 * While a stop is completing TZEROWAIT is not written (it would drop from its maximum
	 * and let the motor drive back to the stale target). The cache is updated.
	 */
	err = tmc5262_ramp_write(config->controller, r, running,
				 !tmc5262_stop_is_pending(config->controller));
	if (err < 0) {
		return err;
	}

	data->ramp = *r;

	return 0;
}

static int tmc5262_ctrl_set_microstep_interval(const struct device *dev,
					       const uint64_t microstep_interval_ns)
{
	const struct tmc5262_stepper_ctrl_config *config = dev->config;
	struct tmc5262_stepper_ctrl_data *data = dev->data;
	struct tmc5262_ramp r;
	uint32_t vmax;
	int err;

	if (microstep_interval_ns == 0U) {
		return -EINVAL;
	}

	err = tmc5262_require(config->controller, TMC5262_CAP_RAMP_GEN);
	if (err < 0) {
		return err;
	}

	vmax = tmc5262_velocity_to_reg(
		(uint32_t)MIN(NSEC_PER_SEC / microstep_interval_ns, (uint64_t)UINT32_MAX));
	if (vmax == 0U || (vmax & ~TMC5262_VMAX_MASK) != 0U) {
		return -EINVAL;
	}

	tmc5262_lock(config->controller);

	r = data->ramp;
	r.vmax = vmax;
	err = tmc5262_ctrl_apply_ramp(dev, &r);

	tmc5262_unlock(config->controller);

	return err;
}

static int tmc5262_ctrl_configure_ramp(const struct device *dev,
				       const struct stepper_ctrl_ramp *ramp)
{
	const struct tmc5262_stepper_ctrl_config *config = dev->config;
	struct tmc5262_stepper_ctrl_data *data = dev->data;
	struct tmc5262_ramp r;
	int err;

	err = tmc5262_require(config->controller, TMC5262_CAP_RAMP_GEN);
	if (err < 0) {
		return err;
	}

	tmc5262_lock(config->controller);

	r = data->ramp;
	r.vmax = tmc5262_velocity_to_reg(ramp->speed_max);
	err = tmc5262_accel_to_reg(ramp->acceleration_max, &r.amax);
	if (err == 0) {
		err = tmc5262_accel_to_reg(ramp->deceleration_max, &r.dmax);
	}
	if (err < 0) {
		/* Above the 18 bit register range: rejected, not clamped */
		tmc5262_unlock(config->controller);
		return err;
	}
	/* Single phase acceleration: A1/A2 = AMAX, D1/D2 = DMAX, V1 = V2 = 0 disables the phases */
	r.a1 = r.amax;
	r.a2 = r.amax;
	r.d1 = r.dmax;
	r.d2 = r.dmax;
	r.v1 = 0;
	r.v2 = 0;

	err = tmc5262_ramp_validate(&r);
	if (err == 0) {
		err = tmc5262_ctrl_apply_ramp(dev, &r);
	}

	tmc5262_unlock(config->controller);

	return err;
}

static int tmc5262_ctrl_init(const struct device *dev)
{
	const struct tmc5262_stepper_ctrl_config *config = dev->config;
	struct tmc5262_stepper_ctrl_data *data = dev->data;
	int err;

	if (!device_is_ready(config->controller)) {
		LOG_ERR_DEVICE_NOT_READY(config->controller);
		return -ENODEV;
	}

	err = tmc5262_require(config->controller, TMC5262_CAP_RAMP_GEN);
	if (err < 0) {
		LOG_ERR("%s: %s has no ramp generator", dev->name,
			tmc5262_get_variant_name(config->controller));
		return err;
	}

	if (!tmc5262_ramp_is_step_source(config->controller)) {
		LOG_ERR("%s: step-source of the chip is STEP/DIR, ramp generator unavailable",
			dev->name);
		return -ENOTSUP;
	}

	data->ramp = config->default_ramp;
	data->positioning = true;
	data->stopping = false;

	/* Unset second phase values default to the final phase so that D2/A2 are never 0 */
	if (data->ramp.a2 == 0U) {
		data->ramp.a2 = data->ramp.amax;
	}
	if (data->ramp.d2 == 0U) {
		data->ramp.d2 = data->ramp.dmax;
	}

	err = tmc5262_ramp_validate(&data->ramp);
	if (err < 0) {
		LOG_ERR("%s: invalid ramp (range, vmax/amax/dmax/d1 = 0 or vstop < vstart)",
			dev->name);
		return err;
	}

	tmc5262_lock(config->controller);
	err = tmc5262_stepper_ctrl_reapply(dev, true);
	tmc5262_unlock(config->controller);

	return err;
}

static DEVICE_API(stepper_ctrl, tmc5262_stepper_ctrl_api) = {
	.move_by = tmc5262_ctrl_move_by,
	.move_to = tmc5262_ctrl_move_to,
	.run = tmc5262_ctrl_run,
	.stop = tmc5262_ctrl_stop,
	.is_moving = tmc5262_ctrl_is_moving,
	.set_reference_position = tmc5262_ctrl_set_reference_position,
	.get_actual_position = tmc5262_ctrl_get_actual_position,
	.set_event_cb = tmc5262_ctrl_set_event_cb,
	.set_microstep_interval = tmc5262_ctrl_set_microstep_interval,
	.configure_ramp = tmc5262_ctrl_configure_ramp,
};

#define TMC5262_STEPPER_CTRL_DEFINE(inst)                                                          \
	static const struct tmc5262_stepper_ctrl_config tmc5262_stepper_ctrl_config_##inst = {     \
		.controller = DEVICE_DT_GET(DT_INST_PARENT(inst)),                                 \
		.default_ramp =                                                                    \
			{                                                                          \
				.vstart = DT_INST_PROP(inst, vstart),                              \
				.a1 = DT_INST_PROP(inst, a1),                                      \
				.v1 = DT_INST_PROP(inst, v1),                                      \
				.amax = DT_INST_PROP(inst, amax),                                  \
				.vmax = DT_INST_PROP(inst, vmax),                                  \
				.dmax = DT_INST_PROP(inst, dmax),                                  \
				.d1 = DT_INST_PROP(inst, d1),                                      \
				.vstop = DT_INST_PROP(inst, vstop),                                \
				.tzerowait = DT_INST_PROP(inst, tzerowait),                        \
				.v2 = DT_INST_PROP(inst, v2),                                      \
				.a2 = DT_INST_PROP(inst, a2),                                      \
				.d2 = DT_INST_PROP(inst, d2),                                      \
			},                                                                         \
	};                                                                                         \
	static struct tmc5262_stepper_ctrl_data tmc5262_stepper_ctrl_data_##inst;                  \
	DEVICE_DT_INST_DEFINE(inst, tmc5262_ctrl_init, NULL, &tmc5262_stepper_ctrl_data_##inst,    \
			      &tmc5262_stepper_ctrl_config_##inst, POST_KERNEL,                    \
			      CONFIG_STEPPER_INIT_PRIORITY, &tmc5262_stepper_ctrl_api);

DT_INST_FOREACH_STATUS_OKAY(TMC5262_STEPPER_CTRL_DEFINE)
