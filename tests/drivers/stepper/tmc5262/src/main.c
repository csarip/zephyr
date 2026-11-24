/*
 * SPDX-FileCopyrightText: Copyright (c) 2026 Analog Devices, Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * ztest suite of the unified TMC5262 / TMC2262 / TMC3241 driver on the test SPI emulator.
 * Test case comments name the related test case (TC-*) of the software test specification
 * SRS/STS-TMC5262-TMC2262-TMC3241-ZEPHYR.
 */

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/gpio/gpio_emul.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/stepper/stepper.h>
#include <zephyr/drivers/stepper/stepper_ctrl.h>
#include <zephyr/drivers/stepper/stepper_tmc5262.h>
#include <zephyr/kernel.h>
#include <zephyr/pm/device.h>
#include <zephyr/ztest.h>

#include "tmc5262_emul.h"
#include "tmc5262_reg.h"

struct chip {
	const struct device *dev;
	const struct emul *emul;
	const struct device *drv;
	const struct device *ctrl; /* NULL without ramp generator child */
	struct gpio_dt_spec drv_enn;
	struct gpio_dt_spec sleepn;
	enum tmc5262_variant_id variant;
	/* Devicetree values that tests may change at run time */
	uint8_t irun;
	uint8_t ihold;
	uint8_t range;
	enum stepper_micro_step_resolution mres;
	bool stealthchop;
	uint32_t tpwmthrs;
};

#define CHIP(label, drv_label, ctrl_expr, var, _irun, _ihold, _range, _mres, _stealth, _tpwm)      \
	{                                                                                          \
		.dev = DEVICE_DT_GET(DT_NODELABEL(label)),                                         \
		.emul = EMUL_DT_GET(DT_NODELABEL(label)),                                          \
		.drv = DEVICE_DT_GET(DT_NODELABEL(drv_label)),                                     \
		.ctrl = ctrl_expr,                                                                 \
		.drv_enn = GPIO_DT_SPEC_GET_OR(DT_NODELABEL(label), drv_enn_gpios, {0}),           \
		.sleepn = GPIO_DT_SPEC_GET_OR(DT_NODELABEL(label), sleepn_gpios, {0}),             \
		.variant = var,                                                                    \
		.irun = _irun,                                                                     \
		.ihold = _ihold,                                                                   \
		.range = _range,                                                                   \
		.mres = _mres,                                                                     \
		.stealthchop = _stealth,                                                           \
		.tpwmthrs = _tpwm,                                                                 \
	}

static const struct chip chips[] = {
	CHIP(c5262, c5262_drv, DEVICE_DT_GET(DT_NODELABEL(c5262_ctrl)), TMC5262_VARIANT_TMC5262,
	     127, 32, 3, STEPPER_MICRO_STEP_16, true, 500),
	CHIP(c2262, c2262_drv, NULL, TMC5262_VARIANT_TMC2262, 240, 64, 3, STEPPER_MICRO_STEP_16,
	     false, 0),
	CHIP(c3241, c3241_drv, DEVICE_DT_GET(DT_NODELABEL(c3241_ctrl)), TMC5262_VARIANT_TMC3241,
	     240, 64, 3, STEPPER_MICRO_STEP_256, false, 0),
	CHIP(c3241b, c3241b_drv, NULL, TMC5262_VARIANT_TMC3241, 240, 64, 0, STEPPER_MICRO_STEP_256,
	     false, 0),
};

#define C5262  (&chips[0])
#define C2262  (&chips[1])
#define C3241  (&chips[2])
#define C3241B (&chips[3])

/* STEP/DIR motion controller of the TMC2262 (no ramp generator on that chip) */
#define C2262_CTRL DEVICE_DT_GET(DT_NODELABEL(c2262_ctrl))
#define C2262_STEP GPIO_DT_SPEC_GET(DT_NODELABEL(c2262_ctrl), step_gpios)
#define C2262_DIR  GPIO_DT_SPEC_GET(DT_NODELABEL(c2262_ctrl), dir_gpios)

#define WAIT_MS 400

/* Wait until DRV_STATUS.stst reports standstill, which the emulator delays like the chip does */
static bool wait_standstill(const struct chip *c)
{
	struct tmc5262_drv_status st;

	for (int i = 0; i < 100; i++) {
		if (tmc5262_get_drv_status(c->dev, &st) == 0 && st.standstill) {
			return true;
		}
	}

	return false;
}

/* Event counters of the chip under test */
static atomic_t n_stall, n_fault, n_steps, n_stopped, n_left, n_right;

static void drv_cb(const struct device *dev, const enum stepper_event event, void *user_data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(user_data);

	if (event == STEPPER_EVENT_STALL_DETECTED) {
		atomic_inc(&n_stall);
	} else if (event == STEPPER_EVENT_FAULT_DETECTED) {
		atomic_inc(&n_fault);
	}
}

static void ctrl_cb(const struct device *dev, const enum stepper_ctrl_event event, void *user_data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(user_data);

	switch (event) {
	case STEPPER_CTRL_EVENT_STEPS_COMPLETED:
		atomic_inc(&n_steps);
		break;
	case STEPPER_CTRL_EVENT_STOPPED:
		atomic_inc(&n_stopped);
		break;
	case STEPPER_CTRL_EVENT_LEFT_END_STOP_DETECTED:
		atomic_inc(&n_left);
		break;
	case STEPPER_CTRL_EVENT_RIGHT_END_STOP_DETECTED:
		atomic_inc(&n_right);
		break;
	}
}

static void clear_counters(void)
{
	atomic_clear(&n_stall);
	atomic_clear(&n_fault);
	atomic_clear(&n_steps);
	atomic_clear(&n_stopped);
	atomic_clear(&n_left);
	atomic_clear(&n_right);
}

/* Wait until a counter reaches at least @p count, at most WAIT_MS */
static bool wait_count(atomic_t *counter, int count)
{
	for (int i = 0; i < WAIT_MS; i += 5) {
		if (atomic_get(counter) >= count) {
			return true;
		}
		k_msleep(5);
	}

	return false;
}

/* Undo run-time configuration changes of a test: the driver keeps them in its register cache */
static void restore_config(const struct chip *c)
{
	(void)tmc5262_set_irun(c->dev, c->irun);
	(void)tmc5262_set_ihold(c->dev, c->ihold);
	(void)tmc5262_set_irundelay(c->dev, 4);
	(void)tmc5262_set_iholddelay(c->dev, 7);
	(void)tmc5262_set_tpowerdown(c->dev, 10);
	(void)tmc5262_set_current_range(c->dev, c->range, 100);
	(void)stepper_set_micro_step_res(c->drv, c->mres);
	(void)tmc5262_set_direction_inverted(c->dev, false);
	/* StallGuard is off by default: TCOOLTHRS = 0, THIGH = 0, no THIGH_SG_OFF */
	(void)tmc5262_set_tcoolthrs(c->dev, 0);
	(void)tmc5262_set_thigh(c->dev, 0);
	if (c->variant == TMC5262_VARIANT_TMC5262) {
		uint32_t coolconf;

		if (tmc5262_reg_read(c->dev, TMC5262_REG_COOLCONF, &coolconf) == 0 &&
		    (coolconf & TMC5262_COOLCONF_THIGH_SG_OFF) != 0U) {
			(void)tmc5262_reg_write(c->dev, TMC5262_REG_COOLCONF,
						coolconf & ~TMC5262_COOLCONF_THIGH_SG_OFF);
		}
	}
	if (c->variant != TMC5262_VARIANT_TMC3241) {
		(void)tmc5262_set_stealthchop(c->dev, c->stealthchop);
		(void)tmc5262_set_tpwmthrs(c->dev, c->tpwmthrs);
	}
	if (c->ctrl != NULL) {
		/* The devicetree speed and acceleration (VMAX 53687, AMAX = DMAX = 440) */
		struct stepper_ctrl_ramp ramp = {
			.acceleration_max = 51223,
			.speed_max = 51200,
			.deceleration_max = 51223,
		};

		(void)tmc5262_set_step_source(c->dev, true);
		(void)stepper_ctrl_configure_ramp(c->ctrl, &ramp);
	}
}

/* Bring the chip and the emulator back to a known state: powered off, reset values, no events */
static void restore(const struct chip *c)
{
	bool moving;

	(void)stepper_set_event_cb(c->drv, NULL, NULL);
	if (c->ctrl != NULL) {
		(void)stepper_ctrl_set_event_cb(c->ctrl, NULL, NULL);
	}

	tmc5262_emul_set_silent(c->emul, false);
	tmc5262_emul_set_floating(c->emul, false);
	tmc5262_emul_io_error(c->emul, 0, 0);
	tmc5262_emul_rcoil_model(c->emul, -1, 0);
	tmc5262_emul_set_no_resistor(c->emul, false);
	tmc5262_emul_pll_fail_next(c->emul, 0);

	/* Wake up if a test left the chip asleep */
	(void)tmc5262_wake(c->dev);

	/* Finish a running motion and any stop sequence of the previous test */
	tmc5262_emul_finish_move(c->emul);
	(void)stepper_disable(c->drv);
	if (c->ctrl != NULL) {
		(void)stepper_ctrl_is_moving(c->ctrl, &moving);
	}

	tmc5262_emul_reset(c->emul);
	zassert_ok(tmc5262_reinit(c->dev));
	(void)stepper_disable(c->drv);
	restore_config(c);
	tmc5262_emul_log_clear(c->emul);
}

/* The STEP/DIR controller of the TMC2262 is independent of the chip: stop it and rewind */
static void restore_step_dir(void)
{
	(void)stepper_ctrl_set_event_cb(C2262_CTRL, NULL, NULL);
	(void)stepper_ctrl_stop(C2262_CTRL);
	(void)stepper_ctrl_set_reference_position(C2262_CTRL, 0);
	(void)stepper_ctrl_set_microstep_interval(C2262_CTRL, 1000000);
}

static void before(void *f)
{
	ARG_UNUSED(f);

	restore_step_dir();

	for (size_t i = 0; i < ARRAY_SIZE(chips); i++) {
		restore(&chips[i]);
	}

	clear_counters();
}

ZTEST_SUITE(tmc5262, NULL, NULL, before, NULL, NULL);

/* Value of the first logged write to @p reg at or after @p start, fails the test if none */
static uint32_t logged(const struct chip *c, uint8_t reg, int *idx)
{
	int i = tmc5262_emul_log_find(c->emul, reg, false, 0, idx != NULL ? *idx : 0);

	zassert_true(i >= 0, "register 0x%02x was not written", reg);
	if (idx != NULL) {
		*idx = i + 1;
	}

	return tmc5262_emul_log_get(c->emul, i)->val;
}

/* TC-INIT-001, TC-VAR-001, TC-VAR-019: one ready device per node, variant from the compatible */
ZTEST(tmc5262, test_instances_and_variants)
{
	static const char *const names[] = {"TMC5262", "TMC2262", "TMC3241", "TMC3241"};

	for (size_t i = 0; i < ARRAY_SIZE(chips); i++) {
		zassert_true(device_is_ready(chips[i].dev), "chip %zu", i);
		zassert_true(device_is_ready(chips[i].drv), "driver %zu", i);
		zassert_equal(tmc5262_get_variant(chips[i].dev), chips[i].variant);
		zassert_str_equal(tmc5262_get_variant_name(chips[i].dev), names[i]);
	}

	zassert_true(device_is_ready(C5262->ctrl));
	zassert_true(device_is_ready(C3241->ctrl));
}

/* TC-INIT-009: PLL start-up sequence; TC-INIT-013: power-up flags; TC-INIT-005 reset values */
ZTEST(tmc5262, test_init_sequence)
{
	for (size_t i = 0; i < ARRAY_SIZE(chips); i++) {
		const struct chip *c = &chips[i];
		int idx = 0;
		uint32_t pll;

		tmc5262_emul_reset(c->emul);
		tmc5262_emul_log_clear(c->emul);
		zassert_ok(tmc5262_reinit(c->dev));

		/* The PLL FSM is stopped first, then commit with sys_sel, fsm_ena, divider 15 and
		 * the timeout clear ...
		 */
		pll = logged(c, TMC5262_REG_PLL, &idx);
		zassert_equal(pll, 0x1E4U, "FSM off word 0x%08x", pll);
		pll = logged(c, TMC5262_REG_PLL, &idx);
		zassert_equal(pll, 0x11EDU, "commit word 0x%08x", pll);
		/* ... then the same without commit and the flags written to clear */
		pll = logged(c, TMC5262_REG_PLL, &idx);
		zassert_equal(pll, 0x71ECU, "clear word 0x%08x", pll);

		/* The power-up flags are cleared */
		zassert_equal(tmc5262_emul_reg_get(c->emul, TMC5262_REG_GSTAT) &
				      TMC5262_GSTAT_POWERUP_CLEAR,
			      0U);
		zassert_equal(tmc5262_emul_reg_get(c->emul, TMC5262_REG_PLL) &
				      (TMC5262_PLL_FLAGS | TMC5262_PLL_COMMIT),
			      0U);
		zassert_equal(tmc5262_emul_invalid_access(c->emul), 0U);
	}
}

/* TC-INIT-010: PLL retry (3 attempts) and failure */
ZTEST(tmc5262, test_pll_retry)
{
	const struct emul *e = C5262->emul;
	int before_commits;

	tmc5262_emul_reset(e);
	before_commits = tmc5262_emul_pll_commits(e);
	tmc5262_emul_pll_fail_next(e, 1);
	zassert_ok(tmc5262_reinit(C5262->dev));
	zassert_equal(tmc5262_emul_pll_commits(e) - before_commits, 2);

	tmc5262_emul_reset(e);
	before_commits = tmc5262_emul_pll_commits(e);
	tmc5262_emul_pll_fail_next(e, 3);
	zassert_equal(tmc5262_reinit(C5262->dev), -EIO);
	zassert_equal(tmc5262_emul_pll_commits(e) - before_commits, 3);

	/* The PLL restart API recovers */
	tmc5262_emul_pll_fail_next(e, 0);
	zassert_ok(tmc5262_restart_pll(C5262->dev));
}

/* TC-INIT-007, TC-CONFIG-011: power stage off after init, DRV_ENN polarity and readback */
ZTEST(tmc5262, test_power_stage_off_and_drv_enn)
{
	for (size_t i = 0; i < 3; i++) {
		const struct chip *c = &chips[i];
		uint32_t ioin;

		/* DRV_ENN is released (physical high) and IOIN.DRV_ENN reads 1 = disabled */
		zassert_equal(gpio_emul_output_get(c->drv_enn.port, c->drv_enn.pin), 1);
		zassert_ok(tmc5262_reg_read(c->dev, TMC5262_REG_IOIN, &ioin));
		zassert_true((ioin & TMC5262_IOIN_DRV_ENN) != 0U);

		zassert_ok(stepper_enable(c->drv));
		zassert_equal(gpio_emul_output_get(c->drv_enn.port, c->drv_enn.pin), 0);
		zassert_ok(tmc5262_reg_read(c->dev, TMC5262_REG_IOIN, &ioin));
		zassert_true((ioin & TMC5262_IOIN_DRV_ENN) == 0U);

		zassert_ok(stepper_disable(c->drv));
		zassert_equal(gpio_emul_output_get(c->drv_enn.port, c->drv_enn.pin), 1);
	}

	/* Without DRV_ENN pin the driver is disabled by TOFF = 0 */
	zassert_equal(tmc5262_emul_reg_get(C3241B->emul, TMC5262_REG_CHOPCONF) &
			      TMC5262_CHOPCONF_TOFF,
		      0U);
	zassert_ok(stepper_enable(C3241B->drv));
	zassert_not_equal(tmc5262_emul_reg_get(C3241B->emul, TMC5262_REG_CHOPCONF) &
				  TMC5262_CHOPCONF_TOFF,
			  0U);
	zassert_ok(stepper_disable(C3241B->drv));
	zassert_equal(tmc5262_emul_reg_get(C3241B->emul, TMC5262_REG_CHOPCONF) &
			      TMC5262_CHOPCONF_TOFF,
		      0U);
}

/* TC-INIT-014, TC-CONFIG-001, -002, -004, -009: register words composed from devicetree */
ZTEST(tmc5262, test_devicetree_registers)
{
	const struct emul *e;

	e = C5262->emul;
	zassert_equal(tmc5262_emul_reg_get(e, TMC5262_REG_DRV_CONF), 0x3FU);
	zassert_equal(tmc5262_emul_reg_get(e, TMC5262_REG_IHOLD_IRUN), 0x04077F20U);
	zassert_equal(tmc5262_emul_reg_get(e, TMC5262_REG_CHOPCONF) & ~TMC5262_CHOPCONF_MRES,
		      0x10410255U);
	zassert_equal(tmc5262_emul_reg_get(e, TMC5262_REG_GCONF),
		      0x14U | TMC5262_GCONF_EN_STEALTHCHOP);
	zassert_equal(tmc5262_emul_reg_get(e, TMC5262_REG_TPWMTHRS), 500U);
	zassert_equal(tmc5262_emul_reg_get(e, TMC5262_REG_COOLCONF) & TMC5262_COOLCONF_SGT,
		      0x7BU << 16);

	e = C3241->emul;
	zassert_equal(tmc5262_emul_reg_get(e, TMC5262_REG_GCONF), 0x40000014U);
	/* FR-MOTION-014, FR-FEATURE-011: switches, virtual stops, position compare from devicetree
	 */
	zassert_equal(tmc5262_emul_reg_get(e, TMC5262_REG_SW_MODE),
		      TMC5262_SW_MODE_STOP_L_ENABLE | TMC5262_SW_MODE_EN_SOFTSTOP);
	zassert_equal(tmc5262_emul_reg_get(e, TMC5262_REG_VIRTUAL_STOP_L), (uint32_t)-100);
	zassert_equal(tmc5262_emul_reg_get(e, TMC5262_REG_VIRTUAL_STOP_R), 100000U);
	zassert_equal(tmc5262_emul_reg_get(e, TMC5262_REG_X_COMPARE), 5000U);
	zassert_equal(tmc5262_emul_reg_get(e, TMC5262_REG_X_COMPARE_REPEAT), 10U);
	/* DO0 maps the position compare event */
	zassert_equal(tmc5262_emul_reg_get(e, TMC5262_REG_DO_CONF) & TMC5262_DO0_SRC_xcomp,
		      TMC5262_DO0_SRC_xcomp);
	zassert_equal(tmc5262_emul_reg_get(e, TMC5262_REG_CHOPCONF), 0x100104A2U);

	e = C2262->emul;
	zassert_equal(tmc5262_emul_reg_get(e, TMC5262_REG_DRV_CONF), 0x3FU);
	zassert_equal(tmc5262_emul_reg_get(e, TMC5262_REG_GCONF), 0x14U);
	/* DO1 error and stall, both outputs low active (reset polarity) */
	zassert_equal(tmc5262_emul_reg_get(e, TMC5262_REG_DO_CONF),
		      TMC5262_DO_CONF_DO0_INVPP | TMC5262_DO_CONF_DO1_INVPP |
			      TMC5262_DO1_SRC_error | TMC5262_DO1_SRC_stall);
}

/* M4: registers that are not set from devicetree get their datasheet reset values, not 0 */
ZTEST(tmc5262, test_reset_value_seeds)
{
	const struct emul *e = C5262->emul;

	tmc5262_emul_reset(e);
	tmc5262_emul_log_clear(e);
	zassert_ok(tmc5262_reinit(C5262->dev));
	zassert_equal(logged(C5262, TMC5262_REG_ENC_CONST, NULL), 0x00010000U);
	zassert_equal(logged(C5262, TMC5262_REG_T_RCOIL_MEAS, NULL), 0x1000U);
	zassert_equal(logged(C5262, TMC5262_REG_UDC_CONF, NULL), 0xCEU);
	zassert_equal(logged(C5262, TMC5262_REG_ENCMODE, NULL), 16U << 16);
	zassert_equal(logged(C5262, TMC5262_REG_SW_MODE, NULL),
		      TMC5262_SW_MODE_HARD_STOP_CLR_CUR_INT);
	zassert_equal(logged(C5262, TMC5262_REG_OTW_OV_VTH, NULL), 0x01FF01FFU);

	e = C3241->emul;
	tmc5262_emul_reset(e);
	tmc5262_emul_log_clear(e);
	zassert_ok(tmc5262_reinit(C3241->dev));
	zassert_equal(logged(C3241, TMC5262_REG_ENC_CONST, NULL), 0x00010000U);
	zassert_equal(tmc5262_emul_log_find(e, TMC5262_REG_UDC_CONF, false, 0, 0), -1);
	zassert_equal(tmc5262_emul_invalid_access(e), 0U);
}

/* TC-VAR-002 .. TC-VAR-014: capability gating, -ENOTSUP without any bus access */
ZTEST(tmc5262, test_capability_gating)
{
	struct tmc5262_rcoil rc;
	struct tmc5262_encoder enc;
	uint32_t u;
	int32_t s;
	int16_t a;
	int16_t b;

	/* TMC2262: no ramp generator, switches, position compare, direct mode */
	const struct emul *e = C2262->emul;
	const struct device *d = C2262->dev;

	tmc5262_emul_log_clear(e);
	zassert_equal(tmc5262_set_switch_config(d, 0, 0, 0), -ENOTSUP);
	zassert_equal(tmc5262_set_stallguard_stop(d, true), -ENOTSUP);
	zassert_equal(tmc5262_get_xlatch(d, &s), -ENOTSUP);
	zassert_equal(tmc5262_set_position_compare(d, 0, 0), -ENOTSUP);
	zassert_equal(tmc5262_set_direct_current(d, true, 0, 0), -ENOTSUP);
	zassert_equal(tmc5262_abort(d), -ENOTSUP);
	zassert_equal(tmc5262_get_actual_velocity(d, &s), -ENOTSUP);
	zassert_equal(tmc5262_set_step_source(d, true), -ENOTSUP);
	zassert_equal(tmc5262_set_step_source(d, false), -ENOTSUP);
	zassert_equal(tmc5262_emul_log_count(e), 0);

	/* TMC3241: no StealthChop+, StallGuard+, CoolStep+, uDcStep, coil measurement, ADC_I */
	e = C3241->emul;
	d = C3241->dev;
	tmc5262_emul_log_clear(e);
	zassert_equal(tmc5262_set_stealthchop(d, true), -ENOTSUP);
	zassert_equal(tmc5262_set_tpwmthrs(d, 1), -ENOTSUP);
	zassert_equal(tmc5262_set_pwmconf(d, 0), -ENOTSUP);
	zassert_equal(tmc5262_set_udcstep(d, false, 1, 2, 0), -ENOTSUP);
	zassert_equal(tmc5262_set_stallguard_plus(d, 0, 0), -ENOTSUP);
	zassert_equal(tmc5262_set_coolstep_plus(d, 0, 0, 0, 0), -ENOTSUP);
	zassert_equal(tmc5262_get_coolstep_plus_load_reserve(d, &u), -ENOTSUP);
	zassert_equal(tmc5262_measure_rcoil(d, 0, &rc), -ENOTSUP);
	zassert_equal(tmc5262_set_rcoil_user(d, 1, 1), -ENOTSUP);
	zassert_equal(tmc5262_get_coil_current_raw(d, &a, &b), -ENOTSUP);
	zassert_equal(tmc5262_get_encoder_deviation(d, &u), -ENOTSUP);
	zassert_equal(tmc5262_set_tricoder(d, true, 0), -ENOTSUP);
	zassert_equal(tmc5262_get_stealthchop_measurements(d, &u, &u), -ENOTSUP);
	zassert_equal(tmc5262_emul_log_count(e), 0);

	/* The features exist where the SRS says: no -ENOTSUP, no invalid register access */
	e = C5262->emul;
	d = C5262->dev;
	zassert_ok(tmc5262_set_stealthchop(d, false));
	zassert_ok(tmc5262_set_tpwmthrs(d, 1));
	zassert_ok(tmc5262_set_stallguard_plus(d, 0, 100));
	/* back to the devicetree (reset) values: the driver caches them */
	zassert_ok(tmc5262_set_stallguard_plus(d, TMC5262_SGP_CONF_CLEAR_CUR_PI, 0x1000));
	zassert_ok(tmc5262_get_coil_current_raw(d, &a, &b));
	zassert_ok(tmc5262_get_encoder_deviation(d, &u));
	zassert_ok(tmc5262_encoder_read(d, &enc));
	zassert_ok(tmc5262_get_actual_velocity(d, &s));
	zassert_ok(tmc5262_set_position_compare(d, 5, 0));
	zassert_ok(tmc5262_set_position_compare(d, -1, 0));
	zassert_equal(tmc5262_emul_invalid_access(e), 0U);
	zassert_equal(tmc5262_emul_invalid_access(C2262->emul), 0U);
	zassert_equal(tmc5262_emul_invalid_access(C3241->emul), 0U);

	/* Reading a register that the variant does not have is rejected (FR-FEATURE-012) */
	zassert_equal(tmc5262_reg_read(C2262->dev, TMC5262_REG_RAMPMODE, &u), -EINVAL);
	zassert_equal(tmc5262_reg_write(C3241->dev, TMC5262_REG_PWMCONF, 0), -EINVAL);
	zassert_equal(tmc5262_reg_read(C5262->dev, 0x7F, &u), -EINVAL);
	zassert_ok(tmc5262_reg_read(C5262->dev, TMC5262_REG_RAMPMODE, &u));
}

/* TC-ERR-002: parameters are validated before any register is written */
ZTEST(tmc5262, test_parameter_validation)
{
	const struct device *d = C5262->dev;
	const struct emul *e = C5262->emul;
	uint32_t u;

	tmc5262_emul_log_clear(e);
	zassert_equal(tmc5262_set_irun(d, 256), -EINVAL);
	zassert_equal(tmc5262_set_ihold(d, 256), -EINVAL);
	zassert_equal(tmc5262_set_iholddelay(d, 256), -EINVAL);
	zassert_equal(tmc5262_set_irundelay(d, 16), -EINVAL);
	zassert_equal(tmc5262_set_tpowerdown(d, 256), -EINVAL);
	zassert_equal(tmc5262_set_current_range(d, 4, 100), -EINVAL);
	zassert_equal(tmc5262_set_current_range(d, 1, 50), -EINVAL); /* scale needs lowest range */
	zassert_equal(tmc5262_set_current_range(d, 0, 60), -EINVAL);
	zassert_equal(tmc5262_set_stallguard2(d, 64, false), -EINVAL);
	zassert_equal(tmc5262_set_stallguard2(d, -65, false), -EINVAL);
	zassert_equal(tmc5262_set_tcoolthrs(d, 0x100000), -EINVAL);
	zassert_equal(tmc5262_set_thigh(d, 0x100000), -EINVAL);
	zassert_equal(tmc5262_set_otw_ov_thresholds(d, 512, 0), -EINVAL);
	zassert_equal(tmc5262_set_coolstep(d, 16, 0, 0, 0, false), -EINVAL);
	zassert_equal(tmc5262_set_coolstep(d, 0, 0, 0, 8, false), -EINVAL);
	zassert_equal(tmc5262_set_udcstep(d, false, 5, 5, 0), -EINVAL);
	zassert_equal(tmc5262_set_direct_current(d, true, 300, 0), -EINVAL);
	zassert_equal(tmc5262_clear_gstat(d, 0x100), -EINVAL);
	zassert_equal(tmc5262_get_gstat(d, NULL), -EINVAL);
	zassert_equal(tmc5262_encoder_configure(C3241->dev, TMC5262_ENCMODE_BEMF_BLANK_TIME, 0),
		      -EINVAL);
	zassert_equal(tmc5262_emul_log_count(e), 0, "no write on a rejected call");
	zassert_equal(tmc5262_emul_log_count(C3241->emul), 0);

	/* Valid values are accepted */
	zassert_ok(tmc5262_set_irun(d, 255));
	zassert_ok(tmc5262_set_irundelay(d, 15));
	zassert_equal(tmc5262_emul_reg_get(e, TMC5262_REG_IHOLD_IRUN) & TMC5262_IHOLD_IRUN_IRUN,
		      255U << 8);
	zassert_ok(tmc5262_get_gstat(d, &u));
}

/* TC-CONFIG-001: range, scale and the full scale current table; -EBUSY while moving */
ZTEST(tmc5262, test_current_range)
{
	uint32_t ma;
	bool peak;

	zassert_ok(tmc5262_get_current_range(C5262->dev, &ma, &peak));
	zassert_equal(ma, 4240U);
	zassert_false(peak);
	zassert_ok(tmc5262_get_current_range(C2262->dev, &ma, &peak));
	zassert_equal(ma, 6000U);
	zassert_true(peak);

	zassert_ok(tmc5262_set_current_range(C5262->dev, 0, 50));
	zassert_ok(tmc5262_get_current_range(C5262->dev, &ma, &peak));
	zassert_equal(ma, 530U);
	zassert_ok(tmc5262_set_current_range(C5262->dev, 2, 100));
	zassert_equal(
		tmc5262_emul_reg_get(C5262->emul, TMC5262_REG_DRV_CONF) &
			(TMC5262_DRV_CONF_CURRENT_RANGE | TMC5262_DRV_CONF_CURRENT_RANGE_SCALE),
		0x0EU);

	tmc5262_emul_start_motion(C5262->emul, 1000);
	zassert_equal(tmc5262_set_current_range(C5262->dev, 0, 100), -EBUSY);
	tmc5262_emul_finish_move(C5262->emul);
	zassert_ok(tmc5262_set_current_range(C5262->dev, 0, 100));
}

/* TC-CONFIG-003: microstep resolution mapping and read-back */
ZTEST(tmc5262, test_microstep_resolution)
{
	static const struct {
		enum stepper_micro_step_resolution res;
		uint32_t mres;
	} map[] = {
		{STEPPER_MICRO_STEP_256, 0}, {STEPPER_MICRO_STEP_128, 1},
		{STEPPER_MICRO_STEP_64, 2},  {STEPPER_MICRO_STEP_32, 3},
		{STEPPER_MICRO_STEP_16, 4},  {STEPPER_MICRO_STEP_8, 5},
		{STEPPER_MICRO_STEP_4, 6},   {STEPPER_MICRO_STEP_2, 7},
		{STEPPER_MICRO_STEP_1, 8},
	};

	for (size_t i = 0; i < ARRAY_SIZE(chips); i++) {
		for (size_t j = 0; j < ARRAY_SIZE(map); j++) {
			enum stepper_micro_step_resolution res;

			zassert_ok(stepper_set_micro_step_res(chips[i].drv, map[j].res));
			zassert_equal(FIELD_GET(TMC5262_CHOPCONF_MRES,
						tmc5262_emul_reg_get(chips[i].emul,
								     TMC5262_REG_CHOPCONF)),
				      map[j].mres);
			zassert_ok(stepper_get_micro_step_res(chips[i].drv, &res));
			zassert_equal(res, map[j].res);
		}
		zassert_equal(stepper_set_micro_step_res(chips[i].drv, 3), -EINVAL);
	}
}

/* TC-CONFIG-006, -007, -008 .. -010: direction, step source, StealthChop+ switching */
ZTEST(tmc5262, test_gconf_runtime_changes)
{
	/* Direction inversion on STEP/DIR operation; not while the ramp generator steps (TMC5262)
	 */
	zassert_equal(tmc5262_set_direction_inverted(C5262->dev, true), -ENOTSUP);
	zassert_ok(tmc5262_set_direction_inverted(C2262->dev, true));
	zassert_true((tmc5262_emul_reg_get(C2262->emul, TMC5262_REG_GCONF) & TMC5262_GCONF_SHAFT) !=
		     0U);

	/* Step source selection and its polarity on the TMC3241 */
	zassert_ok(tmc5262_set_step_source(C3241->dev, false));
	zassert_equal(tmc5262_emul_reg_get(C3241->emul, TMC5262_REG_GCONF) &
			      TMC3241_GCONF_THREEPHASE_MCC,
		      0U);
	zassert_ok(tmc5262_set_step_source(C3241->dev, true));
	zassert_not_equal(tmc5262_emul_reg_get(C3241->emul, TMC5262_REG_GCONF) &
				  TMC3241_GCONF_THREEPHASE_MCC,
			  0U);

	/* StealthChop+ only at standstill */
	tmc5262_emul_start_motion(C2262->emul, 1000);
	zassert_equal(tmc5262_set_stealthchop(C2262->dev, true), -EBUSY);
	tmc5262_emul_finish_move(C2262->emul);
	/* STEP/DIR: standstill is DRV_STATUS.stst, which lags the last step */
	zassert_equal(tmc5262_set_stealthchop(C2262->dev, true), -EBUSY);
	zassert_true(wait_standstill(C2262));
	zassert_ok(tmc5262_set_stealthchop(C2262->dev, true));
	zassert_true((tmc5262_emul_reg_get(C2262->emul, TMC5262_REG_GCONF) &
		      TMC5262_GCONF_EN_STEALTHCHOP) != 0U);
}

/* TC-DIAG-001, -002, -004: GSTAT, decoded DRV_STATUS, ADC conversions */
ZTEST(tmc5262, test_diagnostics)
{
	struct tmc5262_drv_status st;
	uint32_t gstat;
	int32_t mv;
	int32_t mdegc;
	int16_t ia;
	int16_t ib;

	/* Per-coil faults on the TMC5262 */
	tmc5262_emul_set_fault(C5262->emul, TMC5262_DRV_STATUS_S2GB | TMC5262_DRV_STATUS_OLA, true);
	zassert_ok(tmc5262_get_drv_status(C5262->dev, &st));
	zassert_true(st.short_gnd_b);
	zassert_false(st.short_gnd_a);
	zassert_true(st.open_load_a);
	zassert_true((st.valid & TMC5262_STATUS_PER_COIL) != 0U);
	zassert_true(st.standstill);
	zassert_ok(tmc5262_get_gstat(C5262->dev, &gstat));
	zassert_true((gstat & TMC5262_GSTAT_DRV_ERR) != 0U);

	/* Combined faults on the TMC3241 */
	tmc5262_emul_set_fault(C3241->emul, TMC3241_DRV_STATUS_S2G | TMC3241_DRV_STATUS_OT, true);
	zassert_ok(tmc5262_get_drv_status(C3241->dev, &st));
	zassert_true(st.short_gnd_a);
	zassert_true(st.ot);
	zassert_equal(st.valid & TMC5262_STATUS_PER_COIL, 0U);

	/* Write 1 to clear */
	zassert_ok(tmc5262_clear_gstat(C3241->dev, TMC5262_GSTAT_RESET));
	zassert_ok(tmc5262_get_gstat(C3241->dev, &gstat));
	zassert_equal(gstat & TMC5262_GSTAT_RESET, 0U);

	/* VS = 0.1409 V x ADC_VSUPPLY, T = 1.042 x ADC_TEMP - 264.6 degC */
	tmc5262_emul_reg_set(C5262->emul, TMC5262_REG_ADC_VSUPPLY_TEMP, (300U << 16) | 170U);
	zassert_ok(tmc5262_get_supply_mv(C5262->dev, &mv));
	zassert_equal(mv, 23953);
	zassert_ok(tmc5262_get_temperature_mdegc(C5262->dev, &mdegc));
	zassert_equal(mdegc, 48000);

	tmc5262_emul_reg_set(C5262->emul, TMC5262_REG_ADC_I, (0xFFEU << 16) | 0x7FU);
	zassert_ok(tmc5262_get_coil_current_raw(C5262->dev, &ia, &ib));
	zassert_equal(ia, 127);
	zassert_equal(ib, -2);
}

/* TC-BUS-005: SPI_STATUS is captured; bits 7..4 are only valid with a ramp generator */
ZTEST(tmc5262, test_spi_status)
{
	uint8_t status;
	uint32_t u;

	/* After the reset: reset_flag (bit 0) is set by the emulator, standstill (bit 3) */
	tmc5262_emul_reg_set(C5262->emul, TMC5262_REG_GSTAT, TMC5262_GSTAT_RESET);
	zassert_ok(tmc5262_reg_read(C5262->dev, TMC5262_REG_GSTAT, &u));
	zassert_ok(tmc5262_reg_read(C5262->dev, TMC5262_REG_GSTAT, &u));
	zassert_ok(tmc5262_get_spi_status(C5262->dev, &status));
	zassert_true((status & TMC5262_SPI_STATUS_RESET_FLAG) != 0U);
	zassert_true((status & TMC5262_SPI_STATUS_STANDSTILL) != 0U);
	/* position_reached (bit 5) is visible on the TMC5262 */
	zassert_true((status & TMC5262_SPI_STATUS_POSITION_REACHED) != 0U);

	/* TMC2262: the ramp bits are cleared in the driver */
	zassert_ok(tmc5262_reg_read(C2262->dev, TMC5262_REG_GSTAT, &u));
	zassert_ok(tmc5262_get_spi_status(C2262->dev, &status));
	zassert_equal(status & TMC5262_SPI_STATUS_RAMP_BITS, 0U);
}

/* TC-MOTION-005, -009, -012, -013: positioning, reference position, steps completed event */
ZTEST(tmc5262, test_positioning_and_events)
{
	int32_t pos;
	bool moving;

	zassert_ok(stepper_ctrl_set_event_cb(C5262->ctrl, ctrl_cb, NULL));
	zassert_ok(stepper_enable(C5262->drv));

	zassert_ok(stepper_ctrl_set_reference_position(C5262->ctrl, 1000));
	zassert_ok(stepper_ctrl_get_actual_position(C5262->ctrl, &pos));
	zassert_equal(pos, 1000);

	tmc5262_emul_log_clear(C5262->emul);
	zassert_ok(stepper_ctrl_move_by(C5262->ctrl, 5000));
	zassert_equal(tmc5262_emul_reg_get(C5262->emul, TMC5262_REG_XTARGET), 6000U);
	zassert_equal(tmc5262_emul_reg_get(C5262->emul, TMC5262_REG_VMAX), 53687U);
	zassert_ok(stepper_ctrl_is_moving(C5262->ctrl, &moving));
	zassert_true(moving);
	/* A reference position cannot be set while moving */
	zassert_equal(stepper_ctrl_set_reference_position(C5262->ctrl, 0), -EBUSY);

	tmc5262_emul_finish_move(C5262->emul);
	zassert_true(wait_count(&n_steps, 1), "no steps completed event");
	k_msleep(50);
	zassert_equal(atomic_get(&n_steps), 1, "steps completed reported once");
	zassert_equal(atomic_get(&n_stopped), 0);
	zassert_ok(stepper_ctrl_is_moving(C5262->ctrl, &moving));
	zassert_false(moving);

	/* The TMC3241 ramp generator accepts a move as well */
	zassert_equal(stepper_ctrl_move_to(C3241->ctrl, 10), 0);
	zassert_equal(tmc5262_emul_reg_get(C3241->emul, TMC5262_REG_XTARGET), 10U);
}

/* TC-MOTION-011, N1: stop sequence, TC-MOTION-013: STOPPED; C1: no VMAX write, no reverse move */
ZTEST(tmc5262, test_stop_sequence)
{
	const struct emul *e = C5262->emul;
	const struct device *ctrl = C5262->ctrl;
	bool moving;
	int n0;
	int idx = 0;
	uint32_t xact_at_stop;

	zassert_ok(stepper_ctrl_set_event_cb(ctrl, ctrl_cb, NULL));
	zassert_ok(stepper_enable(C5262->drv));
	zassert_ok(stepper_ctrl_move_to(ctrl, 100000));
	zassert_ok(stepper_ctrl_is_moving(ctrl, &moving));
	zassert_true(moving);

	xact_at_stop = tmc5262_emul_reg_get(e, TMC5262_REG_XACTUAL);
	tmc5262_emul_log_clear(e);
	zassert_ok(stepper_ctrl_stop(ctrl));

	/* TZEROWAIT to its maximum, XTARGET = XACTUAL, positioning mode, VMAX not written */
	zassert_equal(logged(C5262, TMC5262_REG_TZEROWAIT, &idx), TMC5262_TZEROWAIT_MASK);
	zassert_equal(logged(C5262, TMC5262_REG_XTARGET, &idx), xact_at_stop);
	zassert_equal(logged(C5262, TMC5262_REG_RAMPMODE, &idx), TMC5262_RAMPMODE_POSITION);
	n0 = tmc5262_emul_log_find(e, TMC5262_REG_VMAX, false, 0, 0);
	zassert_equal(n0, -1, "VMAX must not be written by a stop");

	/* Still moving until the final standstill, then STOPPED once */
	zassert_true(wait_count(&n_stopped, 1), "no STOPPED event");
	k_msleep(50);
	zassert_equal(atomic_get(&n_stopped), 1);
	zassert_equal(atomic_get(&n_steps), 0, "no spurious steps completed after a stop");
	zassert_ok(stepper_ctrl_is_moving(ctrl, &moving));
	zassert_false(moving);

	/* Final XTARGET = XACTUAL (after the overshoot) and TZEROWAIT restored, in this order */
	zassert_equal(tmc5262_emul_reg_get(e, TMC5262_REG_XTARGET),
		      tmc5262_emul_reg_get(e, TMC5262_REG_XACTUAL));
	zassert_not_equal(tmc5262_emul_reg_get(e, TMC5262_REG_XACTUAL), xact_at_stop);
	zassert_equal(tmc5262_emul_reg_get(e, TMC5262_REG_TZEROWAIT), 0U);
	zassert_equal(tmc5262_emul_reverse_risk(e), 0U);
	zassert_equal(tmc5262_emul_vmax_in_hold(e), 0U);
	/* The event of the final XTARGET rewrite is cleared */
	zassert_equal(tmc5262_emul_reg_get(e, TMC5262_REG_RAMP_STAT) &
			      TMC5262_RAMP_STAT_EVENT_POS_REACHED,
		      0U);
}

/* N-c: the waiting stop of stepper_disable() completes it and still reports STOPPED */
ZTEST(tmc5262, test_disable_waits_for_stop)
{
	const struct emul *e = C3241->emul;
	bool moving;

	zassert_ok(stepper_ctrl_set_event_cb(C3241->ctrl, ctrl_cb, NULL));
	zassert_ok(stepper_enable(C3241->drv));
	zassert_ok(stepper_ctrl_move_to(C3241->ctrl, 50000));
	zassert_ok(stepper_ctrl_stop(C3241->ctrl));
	zassert_ok(stepper_disable(C3241->drv));

	zassert_equal(gpio_emul_output_get(C3241->drv_enn.port, C3241->drv_enn.pin), 1);
	zassert_equal(tmc5262_emul_reg_get(e, TMC5262_REG_XTARGET),
		      tmc5262_emul_reg_get(e, TMC5262_REG_XACTUAL));
	zassert_equal(tmc5262_emul_reg_get(e, TMC5262_REG_TZEROWAIT), 0U);
	zassert_equal(tmc5262_emul_reverse_risk(e), 0U);
	zassert_ok(stepper_ctrl_is_moving(C3241->ctrl, &moving));
	zassert_false(moving);
	zassert_true(wait_count(&n_stopped, 1), "STOPPED is reported after a waiting stop");
	k_msleep(50);
	zassert_equal(atomic_get(&n_stopped), 1);
	zassert_equal(atomic_get(&n_steps), 0);
}

/* C1: VMAX is never non-zero in hold mode; a move after an abort sets the mode first */
ZTEST(tmc5262, test_vmax_rules)
{
	const struct emul *e = C5262->emul;
	struct stepper_ctrl_ramp ramp = {
		.acceleration_max = 50000,
		.speed_max = 40000,
		.deceleration_max = 50000,
	};
	int i_x;
	int i_m;
	int i_v;

	zassert_ok(stepper_enable(C5262->drv));
	zassert_ok(stepper_ctrl_move_to(C5262->ctrl, 20000));
	zassert_ok(tmc5262_abort(C5262->dev));
	zassert_equal(tmc5262_emul_reg_get(e, TMC5262_REG_VMAX), 0U);
	zassert_equal(tmc5262_emul_reg_get(e, TMC5262_REG_RAMPMODE), TMC5262_RAMPMODE_HOLD);

	/* Re-configuring the ramp in hold mode must not start anything ... */
	tmc5262_emul_finish_move(e);
	zassert_ok(stepper_ctrl_configure_ramp(C5262->ctrl, &ramp));
	zassert_equal(tmc5262_emul_vmax_in_hold(e), 0U);

	/* ... and a new move writes the target, then the mode, then VMAX */
	tmc5262_emul_log_clear(e);
	zassert_ok(stepper_ctrl_move_to(C5262->ctrl, 30000));
	i_x = tmc5262_emul_log_find(e, TMC5262_REG_XTARGET, true, 30000, 0);
	i_m = tmc5262_emul_log_find(e, TMC5262_REG_RAMPMODE, true, TMC5262_RAMPMODE_POSITION, 0);
	i_v = tmc5262_emul_log_find(e, TMC5262_REG_VMAX, false, 0, 0);
	zassert_true(i_x >= 0 && i_m >= 0 && i_v >= 0);
	zassert_true(i_x < i_m && i_m < i_v, "order %d %d %d", i_x, i_m, i_v);
	zassert_equal(tmc5262_emul_vmax_in_hold(e), 0U);

	/* Changing the speed while a stop is completing must not lower TZEROWAIT */
	zassert_ok(stepper_ctrl_stop(C5262->ctrl));
	zassert_ok(stepper_ctrl_set_microstep_interval(C5262->ctrl, 50000));
	zassert_ok(stepper_ctrl_configure_ramp(C5262->ctrl, &ramp));
	zassert_equal(tmc5262_emul_reverse_risk(e), 0U);
	zassert_ok(stepper_disable(C5262->drv));
	zassert_equal(tmc5262_emul_reverse_risk(e), 0U);
}

/* TC-MOTION-007, -008: unit conversion with the fixed 16 MHz clock, range checks */
ZTEST(tmc5262, test_ramp_conversion)
{
	const struct emul *e = C5262->emul;
	struct stepper_ctrl_ramp ramp = {
		.acceleration_max = 100000,
		.speed_max = 51200,
		.deceleration_max = 100000,
	};

	zassert_ok(stepper_ctrl_configure_ramp(C5262->ctrl, &ramp));
	/* v = 51200 / 0.953674 = 53687 */
	tmc5262_emul_log_clear(e);
	zassert_ok(stepper_enable(C5262->drv));
	zassert_ok(stepper_ctrl_move_to(C5262->ctrl, 1));
	zassert_equal(tmc5262_emul_reg_get(e, TMC5262_REG_VMAX), 53687U);
	/* a = 100000 / 116.415 = 859 */
	zassert_equal(tmc5262_emul_reg_get(e, TMC5262_REG_AMAX), 859U);
	zassert_equal(tmc5262_emul_reg_get(e, TMC5262_REG_DMAX), 859U);

	/* An interval of 100 us is 10 kHz */
	zassert_ok(stepper_ctrl_set_microstep_interval(C5262->ctrl, 100000));
	zassert_equal(stepper_ctrl_set_microstep_interval(C5262->ctrl, 0), -EINVAL);
	/* A speed above the 23 bit VMAX range is rejected */
	ramp.speed_max = 9000000;
	zassert_equal(stepper_ctrl_configure_ramp(C5262->ctrl, &ramp), -EINVAL);
	ramp.speed_max = 0;
	zassert_equal(stepper_ctrl_configure_ramp(C5262->ctrl, &ramp), -EINVAL);
}

/* TC-DIAG-003, M6: fault latched on enable cycles the power stage; a persistent fault fails */
ZTEST(tmc5262, test_fault_cycle_on_enable)
{
	const struct emul *e = C5262->emul;
	uint32_t u;
	int i0;
	int i1;

	tmc5262_emul_set_fault(e, TMC5262_DRV_STATUS_S2GA, false);
	tmc5262_emul_log_clear(e);
	zassert_ok(stepper_enable(C5262->drv));

	/* TOFF = 0 for a moment, GSTAT.drv_err cleared, TOFF restored */
	i0 = tmc5262_emul_log_find(e, TMC5262_REG_GSTAT, true, TMC5262_GSTAT_DRV_ERR, 0);
	zassert_true(i0 >= 0);
	i1 = tmc5262_emul_log_find(e, TMC5262_REG_CHOPCONF, false, 0, 0);
	zassert_true(i1 >= 0 && i1 < i0);
	zassert_equal(tmc5262_emul_log_get(e, i1)->val & TMC5262_CHOPCONF_TOFF, 0U);
	zassert_equal(tmc5262_emul_reg_get(e, TMC5262_REG_GSTAT) & TMC5262_GSTAT_DRV_ERR, 0U);
	zassert_equal(tmc5262_emul_reg_get(e, TMC5262_REG_DRV_STATUS) & TMC5262_DRV_STATUS_S2GA,
		      0U);
	zassert_equal(gpio_emul_output_get(C5262->drv_enn.port, C5262->drv_enn.pin), 0);
	zassert_not_equal(tmc5262_emul_reg_get(e, TMC5262_REG_CHOPCONF) & TMC5262_CHOPCONF_TOFF,
			  0U);
	zassert_ok(stepper_disable(C5262->drv));

	/* A fault that does not go away: enable fails and the power stage stays off */
	tmc5262_emul_set_fault(e, TMC5262_DRV_STATUS_S2GA, true);
	zassert_equal(stepper_enable(C5262->drv), -EIO);
	zassert_equal(gpio_emul_output_get(C5262->drv_enn.port, C5262->drv_enn.pin), 1);
	zassert_ok(tmc5262_reg_read(C5262->dev, TMC5262_REG_GSTAT, &u));
	zassert_true((u & TMC5262_GSTAT_DRV_ERR) != 0U);
}

/* TC-DIAG-003: a fault while running disables the power stage and is reported once */
ZTEST(tmc5262, test_fault_event_disables_stage)
{
	zassert_ok(stepper_set_event_cb(C5262->drv, drv_cb, NULL));
	zassert_ok(stepper_enable(C5262->drv));
	zassert_equal(gpio_emul_output_get(C5262->drv_enn.port, C5262->drv_enn.pin), 0);

	tmc5262_emul_set_fault(C5262->emul, TMC5262_DRV_STATUS_S2GA, true);
	zassert_true(wait_count(&n_fault, 1), "no fault event");
	k_msleep(50);
	zassert_equal(atomic_get(&n_fault), 1);
	zassert_equal(gpio_emul_output_get(C5262->drv_enn.port, C5262->drv_enn.pin), 1);
}

/* TC-MOTION-013, m15: a stall is reported once; stop events stay latched until cleared */
ZTEST(tmc5262, test_stall_and_stop_events)
{
	const struct emul *e = C5262->emul;

	zassert_ok(stepper_set_event_cb(C5262->drv, drv_cb, NULL));
	zassert_ok(stepper_ctrl_set_event_cb(C5262->ctrl, ctrl_cb, NULL));
	zassert_ok(stepper_enable(C5262->drv));

	/* StallGuard is active with TCOOLTHRS >= TSTEP: TSTEP 100 <= 1000 */
	zassert_ok(tmc5262_set_tcoolthrs(C5262->dev, 1000));
	tmc5262_emul_start_motion(e, 1000);
	k_msleep(50);
	zassert_equal(atomic_get(&n_stall), 0, "no stall without a stall");

	tmc5262_emul_set_stall(e, true);
	zassert_true(wait_count(&n_stall, 1), "no stall event");
	k_msleep(80);
	zassert_equal(atomic_get(&n_stall), 1, "a stall is reported once");

	/* The stop-on-stall event stays latched in RAMP_STAT until the application clears it */
	zassert_not_equal(tmc5262_emul_reg_get(e, TMC5262_REG_RAMP_STAT) &
				  TMC5262_RAMP_STAT_EVENT_STOP_SG,
			  0U);

	/* Left and right stop events, once each */
	tmc5262_emul_ramp_stat_set(e,
				   TMC5262_RAMP_STAT_EVENT_STOP_L | TMC5262_RAMP_STAT_EVENT_STOP_R);
	zassert_true(wait_count(&n_left, 1));
	zassert_true(wait_count(&n_right, 1));
	k_msleep(50);
	zassert_equal(atomic_get(&n_left), 1);
	zassert_equal(atomic_get(&n_right), 1);
}

/*
 * DRV_STATUS.stallguard reads 1 whenever StallGuard is not active: with TCOOLTHRS = 0 or above
 * the velocity window (TSTEP > TCOOLTHRS). That is no stall. One chip of every variant.
 */
static void check_stall_window(const struct chip *c)
{
	const struct emul *e = c->emul;

	zassert_ok(stepper_set_event_cb(c->drv, drv_cb, NULL));
	zassert_ok(stepper_enable(c->drv));
	tmc5262_emul_reg_set(e, TMC5262_REG_TSTEP, 0);

	/* TCOOLTHRS = 0 (off): the motor runs, the raw flag is set, no stall */
	tmc5262_emul_set_tstep(e, 100);
	tmc5262_emul_start_motion(e, 1000);
	k_msleep(60);
	zassert_equal(atomic_get(&n_stall), 0, "StallGuard off, TCOOLTHRS = 0");
	tmc5262_emul_set_stall(e, true);
	k_msleep(60);
	zassert_equal(atomic_get(&n_stall), 0, "StallGuard off, flag forced");
	tmc5262_emul_set_stall(e, false);

	/* Slow: TSTEP 5000 > TCOOLTHRS 1000, outside of the window even with the flag forced */
	zassert_ok(tmc5262_set_tcoolthrs(c->dev, 1000));
	tmc5262_emul_set_tstep(e, 5000);
	tmc5262_emul_set_stall(e, true);
	k_msleep(60);
	zassert_equal(atomic_get(&n_stall), 0, "slow, TSTEP > TCOOLTHRS");

	/* Fast enough: TSTEP 500 <= 1000, the flag is a stall now, reported once */
	tmc5262_emul_set_tstep(e, 500);
	zassert_true(wait_count(&n_stall, 1), "no stall inside the window");
	k_msleep(60);
	zassert_equal(atomic_get(&n_stall), 1, "reported once");

	/* The stall ends, the motor slows down below the window: no new event */
	tmc5262_emul_set_stall(e, false);
	k_msleep(30);
	tmc5262_emul_set_tstep(e, 5000);
	k_msleep(60);
	zassert_equal(atomic_get(&n_stall), 1);

	/* A second stall in the window is a new one */
	tmc5262_emul_set_tstep(e, 500);
	k_msleep(30);
	tmc5262_emul_set_stall(e, true);
	zassert_true(wait_count(&n_stall, 2), "second stall");
	k_msleep(60);
	zassert_equal(atomic_get(&n_stall), 2);
}

/* Stall events need StallGuard to be active (TCOOLTHRS), TMC5262 */
ZTEST(tmc5262, test_stall_window_tmc5262)
{
	check_stall_window(C5262);
}

/* TMC2262: STEP/DIR, TSTEP and StallGuard exist */
ZTEST(tmc5262, test_stall_window_tmc2262)
{
	check_stall_window(C2262);
}

/* TMC3241: StallGuard2 only */
ZTEST(tmc5262, test_stall_window_tmc3241)
{
	check_stall_window(C3241);
}

/* With COOLCONF.thigh_sg_off StallGuard is also off above THIGH (TSTEP < THIGH) */
ZTEST(tmc5262, test_stall_window_thigh)
{
	const struct emul *e = C5262->emul;
	uint32_t coolconf;

	zassert_ok(stepper_set_event_cb(C5262->drv, drv_cb, NULL));
	zassert_ok(stepper_enable(C5262->drv));
	zassert_ok(tmc5262_set_tcoolthrs(C5262->dev, 1000));
	zassert_ok(tmc5262_set_thigh(C5262->dev, 300));
	tmc5262_emul_start_motion(e, 1000);

	/* TSTEP 200 < THIGH 300, but THIGH_SG_OFF is clear: still inside the window */
	tmc5262_emul_set_tstep(e, 200);
	tmc5262_emul_set_stall(e, true);
	zassert_true(wait_count(&n_stall, 1), "THIGH without THIGH_SG_OFF");
	tmc5262_emul_set_stall(e, false);
	k_msleep(30);

	zassert_ok(tmc5262_reg_read(C5262->dev, TMC5262_REG_COOLCONF, &coolconf));
	zassert_ok(tmc5262_reg_write(C5262->dev, TMC5262_REG_COOLCONF,
				     coolconf | TMC5262_COOLCONF_THIGH_SG_OFF));
	atomic_clear(&n_stall);
	tmc5262_emul_set_stall(e, true);
	k_msleep(60);
	zassert_equal(atomic_get(&n_stall), 0, "above THIGH with THIGH_SG_OFF");

	tmc5262_emul_set_stall(e, false);
	k_msleep(30);
	tmc5262_emul_set_tstep(e, 400);
	tmc5262_emul_set_stall(e, true);
	zassert_true(wait_count(&n_stall, 1), "TCOOLTHRS >= TSTEP >= THIGH");
}

/*
 * RAMP_STAT.event_pos_reached is latched after every XTARGET = XACTUAL (also at init), it is
 * no completed move: nothing is reported until a move is commanded.
 */
ZTEST(tmc5262, test_no_spurious_steps_completed)
{
	const struct emul *e = C5262->emul;

	/* The latch is set after the initialization, as on the chip */
	tmc5262_emul_reset(e);
	zassert_ok(tmc5262_reinit(C5262->dev));
	zassert_not_equal(tmc5262_emul_reg_get(e, TMC5262_REG_RAMP_STAT) &
				  TMC5262_RAMP_STAT_EVENT_POS_REACHED,
			  0U);

	zassert_ok(stepper_ctrl_set_event_cb(C5262->ctrl, ctrl_cb, NULL));
	zassert_ok(stepper_enable(C5262->drv));
	/* Several poll intervals */
	k_msleep(100);
	zassert_equal(atomic_get(&n_steps), 0, "steps completed without a move");

	/* A commanded move is reported exactly once */
	zassert_ok(stepper_ctrl_move_by(C5262->ctrl, 500));
	tmc5262_emul_finish_move(e);
	zassert_true(wait_count(&n_steps, 1), "no steps completed event");
	k_msleep(60);
	zassert_equal(atomic_get(&n_steps), 1);

	/* Nothing more afterwards */
	k_msleep(60);
	zassert_equal(atomic_get(&n_steps), 1);
}

/* Moves while nobody listens are not reported when a callback is registered later */
ZTEST(tmc5262, test_no_stale_steps_completed_on_registration)
{
	const struct emul *e = C5262->emul;

	zassert_ok(stepper_enable(C5262->drv));
	zassert_ok(stepper_ctrl_move_by(C5262->ctrl, 500));
	tmc5262_emul_finish_move(e);
	zassert_not_equal(tmc5262_emul_reg_get(e, TMC5262_REG_RAMP_STAT) &
				  TMC5262_RAMP_STAT_EVENT_POS_REACHED,
			  0U);

	zassert_ok(stepper_ctrl_set_event_cb(C5262->ctrl, ctrl_cb, NULL));
	k_msleep(100);
	zassert_equal(atomic_get(&n_steps), 0, "stale event after the registration");

	zassert_ok(stepper_ctrl_move_by(C5262->ctrl, 500));
	tmc5262_emul_finish_move(e);
	zassert_true(wait_count(&n_steps, 1), "no steps completed event");
	k_msleep(60);
	zassert_equal(atomic_get(&n_steps), 1);

	/* A move that is still running when the callback is registered is reported */
	zassert_ok(stepper_ctrl_set_event_cb(C5262->ctrl, NULL, NULL));
	zassert_ok(stepper_ctrl_move_by(C5262->ctrl, 500));
	zassert_ok(stepper_ctrl_set_event_cb(C5262->ctrl, ctrl_cb, NULL));
	k_msleep(60);
	zassert_equal(atomic_get(&n_steps), 1);
	tmc5262_emul_finish_move(e);
	zassert_true(wait_count(&n_steps, 2), "running move not reported");
}

/* TC-DIAG-006: the DO interrupt wakes the work item (the poll is only a slow fallback) */
ZTEST(tmc5262, test_do_interrupt)
{
	const struct gpio_dt_spec do1 = GPIO_DT_SPEC_GET(DT_NODELABEL(c2262), do1_gpios);

	zassert_ok(stepper_set_event_cb(C2262->drv, drv_cb, NULL));
	zassert_ok(stepper_enable(C2262->drv));
	k_msleep(20);
	clear_counters();

	tmc5262_emul_set_fault(C2262->emul, TMC5262_DRV_STATUS_S2GB, true);
	zassert_ok(gpio_emul_input_set(do1.port, do1.pin, 1));
	/* The fallback poll runs at ten times the interval: 100 ms, so 60 ms proves the IRQ */
	for (int i = 0; i < 12 && atomic_get(&n_fault) == 0; i++) {
		k_msleep(5);
	}
	zassert_equal(atomic_get(&n_fault), 1, "fault via the DO interrupt");
}

/* TC-POWER-004: an unexpected reset is detected before a motion command and reported */
ZTEST(tmc5262, test_unexpected_reset)
{
	const struct emul *e = C5262->emul;

	zassert_ok(stepper_set_event_cb(C5262->drv, drv_cb, NULL));
	zassert_ok(stepper_enable(C5262->drv));
	tmc5262_emul_reset(e);
	tmc5262_emul_log_clear(e);

	zassert_ok(stepper_ctrl_move_to(C5262->ctrl, 1234));
	/* The configuration was re-applied before the move and the stage is on again */
	zassert_equal(tmc5262_emul_reg_get(e, TMC5262_REG_IHOLD_IRUN), 0x04077F20U);
	zassert_equal(tmc5262_emul_reg_get(e, TMC5262_REG_XTARGET), 1234U);
	zassert_equal(gpio_emul_output_get(C5262->drv_enn.port, C5262->drv_enn.pin), 0);
	zassert_true(wait_count(&n_fault, 1), "reset reported to the application");
}

/* TC-POWER-001, -002, -003: sleep, wake and power management */
ZTEST(tmc5262, test_sleep_wake_and_pm)
{
	const struct emul *e = C5262->emul;
	uint32_t u;
	int commits;

	zassert_ok(stepper_enable(C5262->drv));
	zassert_ok(tmc5262_sleep(C5262->dev));
	zassert_equal(gpio_emul_output_get(C5262->sleepn.port, C5262->sleepn.pin), 0);
	zassert_equal(gpio_emul_output_get(C5262->drv_enn.port, C5262->drv_enn.pin), 1);
	/* No register access while asleep, and not -EBUSY */
	zassert_equal(tmc5262_reg_read(C5262->dev, TMC5262_REG_GSTAT, &u), -ENODEV);
	tmc5262_emul_sync_pins(e);

	commits = tmc5262_emul_pll_commits(e);
	zassert_ok(tmc5262_wake(C5262->dev));
	zassert_equal(tmc5262_emul_pll_commits(e) - commits, 1, "PLL restarted");
	zassert_equal(tmc5262_emul_reg_get(e, TMC5262_REG_IHOLD_IRUN), 0x04077F20U);
	zassert_equal(tmc5262_emul_reg_get(e, TMC5262_REG_GSTAT) & TMC5262_GSTAT_POWERUP_CLEAR, 0U);
	zassert_ok(tmc5262_reg_read(C5262->dev, TMC5262_REG_GSTAT, &u));
	/* The power stage is off after the wake-up until it is enabled again */
	zassert_equal(gpio_emul_output_get(C5262->drv_enn.port, C5262->drv_enn.pin), 1);

	/* PM device actions use the same path */
	zassert_ok(pm_device_action_run(C5262->dev, PM_DEVICE_ACTION_SUSPEND));
	tmc5262_emul_sync_pins(e);
	zassert_ok(pm_device_action_run(C5262->dev, PM_DEVICE_ACTION_RESUME));

	/* m6: no SLEEPN pin: suspend only turns the power stage off, tmc5262_sleep is -ENOTSUP */
	zassert_equal(tmc5262_sleep(C3241B->dev), -ENOTSUP);
	zassert_ok(stepper_enable(C3241B->drv));
	zassert_ok(pm_device_action_run(C3241B->dev, PM_DEVICE_ACTION_SUSPEND));
	zassert_equal(tmc5262_emul_reg_get(C3241B->emul, TMC5262_REG_CHOPCONF) &
			      TMC5262_CHOPCONF_TOFF,
		      0U);
	zassert_ok(pm_device_action_run(C3241B->dev, PM_DEVICE_ACTION_RESUME));
}

/* TC-FEATURE-009, -012: steps lost, direct register access with cache coherence */
ZTEST(tmc5262, test_misc_readbacks)
{
	uint32_t u;
	uint16_t mscnt;
	int16_t ca;
	int16_t cb;

	tmc5262_emul_reg_set(C5262->emul, TMC5262_REG_STEPS_LOST, 77);
	zassert_ok(tmc5262_get_steps_lost(C5262->dev, &u));
	zassert_equal(u, 77U);

	tmc5262_emul_reg_set(C5262->emul, TMC5262_REG_MSCNT, 0x123);
	tmc5262_emul_reg_set(C5262->emul, TMC5262_REG_MSCURACT, (0x1FFU << 16) | 0x7FU);
	zassert_ok(tmc5262_get_sequencer(C5262->dev, &mscnt, &ca, &cb));
	zassert_equal(mscnt, 0x123);
	zassert_equal(ca, -1);
	zassert_equal(cb, 127);

	tmc5262_emul_reg_set(C5262->emul, TMC5262_REG_TSTEP, 1000);
	zassert_ok(tmc5262_get_step_frequency(C5262->dev, &u));
	zassert_equal(u, 16000U);

	/* A write through the cache is restored by a re-initialization */
	zassert_ok(tmc5262_reg_write(C5262->dev, TMC5262_REG_TPOWERDOWN, 33));
	tmc5262_emul_reset(C5262->emul);
	zassert_ok(tmc5262_reinit(C5262->dev));
	zassert_equal(tmc5262_emul_reg_get(C5262->emul, TMC5262_REG_TPOWERDOWN), 33U);
}

/* The silent device and the missing resistor are not accepted by the presence check; the
 * check runs in the init function, here only the emulator hooks are exercised
 */
ZTEST(tmc5262, test_emulator_hooks)
{
	uint32_t u;

	tmc5262_emul_set_silent(C5262->emul, true);
	zassert_ok(tmc5262_reg_read(C5262->dev, TMC5262_REG_IOIN, &u));
	zassert_ok(tmc5262_reg_read(C5262->dev, TMC5262_REG_IOIN, &u));
	zassert_equal(u, 0U);
	tmc5262_emul_set_silent(C5262->emul, false);
	zassert_ok(tmc5262_reg_read(C5262->dev, TMC5262_REG_IOIN, &u));
	zassert_ok(tmc5262_reg_read(C5262->dev, TMC5262_REG_IOIN, &u));
	zassert_true((u & TMC5262_IOIN_EXT_RES_DET) != 0U);
}

static void check_sensor(const struct chip *c, const struct device *sensor)
{
	struct sensor_value val;
	uint32_t reads;

	zassert_true(device_is_ready(sensor));
	tmc5262_emul_reg_set(c->emul, TMC5262_REG_ADC_VSUPPLY_TEMP, (300U << 16) | 170U);
	reads = tmc5262_emul_read_requests(c->emul, TMC5262_REG_ADC_VSUPPLY_TEMP);
	zassert_ok(sensor_sample_fetch(sensor));
	/* Voltage and temperature come from one read of the register: one consistent sample */
	zassert_equal(tmc5262_emul_read_requests(c->emul, TMC5262_REG_ADC_VSUPPLY_TEMP) - reads,
		      1U);
	/* channel_get returns the fetched sample, not a new measurement */
	tmc5262_emul_reg_set(c->emul, TMC5262_REG_ADC_VSUPPLY_TEMP, (100U << 16) | 100U);
	zassert_ok(sensor_channel_get(sensor, SENSOR_CHAN_VOLTAGE, &val));
	zassert_equal(val.val1, 23);
	zassert_equal(val.val2, 953000);
	zassert_ok(sensor_channel_get(sensor, SENSOR_CHAN_DIE_TEMP, &val));
	zassert_equal(val.val1, 48);
	zassert_equal(val.val2, 0);
	zassert_equal(sensor_channel_get(sensor, SENSOR_CHAN_ACCEL_X, &val), -ENOTSUP);
	zassert_equal(sensor_sample_fetch_chan(sensor, SENSOR_CHAN_ACCEL_X), -ENOTSUP);
}

/* FR-DIAG-005 / TC-DIAG-005: supply voltage and temperature through the sensor API, on the
 * TMC5262 and on the TMC2262 (same shared code, no ramp generator)
 */
ZTEST(tmc5262, test_sensor_api)
{
	check_sensor(C5262, DEVICE_DT_GET(DT_NODELABEL(c5262_sensor)));
	check_sensor(C2262, DEVICE_DT_GET(DT_NODELABEL(c2262_sensor)));
}

/* FR-FEATURE-005, -002, -004, -006, FR-DIAG-007: non-default and default devicetree groups */
ZTEST(tmc5262, test_devicetree_feature_groups)
{
	const struct emul *e = C5262->emul;

	/* TMC5262 without any of the properties: the datasheet reset values */
	zassert_equal(tmc5262_emul_reg_get(e, TMC5262_REG_CURRENT_PI_REG), 0x000A0040U);
	zassert_equal(tmc5262_emul_reg_get(e, TMC5262_REG_ANGLE_PI_REG), 0x00140032U);
	zassert_equal(tmc5262_emul_reg_get(e, TMC5262_REG_CUR_ANGLE_LIMIT), 0x0FFF0100U);
	zassert_equal(tmc5262_emul_reg_get(e, TMC5262_REG_ANGLE_LOWER_LIMIT), 0x100U);
	zassert_equal(tmc5262_emul_reg_get(e, TMC5262_REG_SGP_CONF), TMC5262_SGP_CONF_CLEAR_CUR_PI);
	zassert_equal(tmc5262_emul_reg_get(e, TMC5262_REG_TSGP_LOW_VEL_THRS), 0x1000U);
	zassert_equal(tmc5262_emul_reg_get(e, TMC5262_REG_COOLSTEPPLUS_CONF), 0x12U);
	zassert_equal(tmc5262_emul_reg_get(e, TMC5262_REG_COOLSTEPPLUS_PI_REG), 0x00100080U);
	zassert_equal(tmc5262_emul_reg_get(e, TMC5262_REG_COOLSTEPPLUS_PI_DOWN), 0x00400080U);
	zassert_equal(tmc5262_emul_reg_get(e, TMC5262_REG_COOLSTEPPLUS_RESERVE_CONF), 0x64DC3296U);
	zassert_equal(tmc5262_emul_reg_get(e, TMC5262_REG_UDC_CONF), 0xCEU);
	zassert_equal(tmc5262_emul_reg_get(e, TMC5262_REG_T_RCOIL_MEAS), 0x1000U);
	zassert_equal(tmc5262_emul_reg_get(e, TMC5262_REG_SW_MODE),
		      TMC5262_SW_MODE_HARD_STOP_CLR_CUR_INT);
	zassert_equal(tmc5262_emul_reg_get(e, TMC5262_REG_X_COMPARE), 0xFFFFFFFFU);

	/* TMC2262 with non-default values */
	e = C2262->emul;
	zassert_equal(tmc5262_emul_reg_get(e, TMC5262_REG_CURRENT_PI_REG), 0x000A0046U);
	zassert_equal(tmc5262_emul_reg_get(e, TMC5262_REG_SGP_CONF),
		      TMC5262_SGP_CONF_CLEAR_CUR_PI | TMC5262_SGP_CONF_FILT_EN | 20U);
	zassert_equal(tmc5262_emul_reg_get(e, TMC5262_REG_COOLSTEPPLUS_CONF), 0x13U);
	zassert_equal(tmc5262_emul_reg_get(e, TMC5262_REG_T_RCOIL_MEAS), 2000U);
	zassert_equal(tmc5262_emul_reg_get(e, TMC5262_REG_COIL_INDUCT),
		      TMC5262_COIL_INDUCT_RCOIL_MANUAL);
	zassert_equal(tmc5262_emul_reg_get(e, TMC5262_REG_R_COIL_USER), (100U << 16) | 200U);
	/* The TMC2262 has no ramp generator registers: none were touched */
	zassert_equal(tmc5262_emul_invalid_access(e), 0U);
}

/* FR-MOTION-003, B5: switch and stop-on-stall configuration need the ramp generator as source */
ZTEST(tmc5262, test_switch_config_needs_ramp_source)
{
	const struct device *d = C3241->dev;

	zassert_ok(tmc5262_set_switch_config(d, TMC5262_SW_MODE_STOP_R_ENABLE, -5, 5));
	zassert_ok(tmc5262_set_stallguard_stop(d, true));
	zassert_equal(tmc5262_emul_reg_get(C3241->emul, TMC5262_REG_SW_MODE),
		      TMC5262_SW_MODE_STOP_R_ENABLE | TMC5262_SW_MODE_SG_STOP);

	zassert_ok(tmc5262_set_step_source(d, false));
	tmc5262_emul_log_clear(C3241->emul);
	zassert_equal(tmc5262_set_switch_config(d, 0, 0, 0), -ENOTSUP);
	zassert_equal(tmc5262_set_stallguard_stop(d, false), -ENOTSUP);
	zassert_equal(stepper_ctrl_move_to(C3241->ctrl, 10), -ENOTSUP);
	zassert_equal(tmc5262_emul_log_count(C3241->emul), 0);
	zassert_ok(tmc5262_set_step_source(d, true));

	/* A write through the API is cached: a reset restores it */
	zassert_ok(tmc5262_set_position_compare(d, 777, 3));
	tmc5262_emul_reset(C3241->emul);
	zassert_ok(tmc5262_reinit(d));
	zassert_equal(tmc5262_emul_reg_get(C3241->emul, TMC5262_REG_X_COMPARE), 777U);
	zassert_equal(tmc5262_emul_reg_get(C3241->emul, TMC5262_REG_X_COMPARE_REPEAT), 3U);
	zassert_ok(tmc5262_set_position_compare(d, 5000, 10));
	zassert_ok(tmc5262_set_switch_config(
		d, TMC5262_SW_MODE_STOP_L_ENABLE | TMC5262_SW_MODE_EN_SOFTSTOP, -100, 100000));
}

/* FR-BUS-004, B6: decoded SPI_STATUS */
ZTEST(tmc5262, test_spi_status_decoded)
{
	struct tmc5262_spi_status st;
	uint32_t u;

	zassert_equal(tmc5262_get_spi_status_decoded(C5262->dev, NULL), -EINVAL);

	tmc5262_emul_reg_set(C5262->emul, TMC5262_REG_GSTAT,
			     TMC5262_GSTAT_RESET | TMC5262_GSTAT_DRV_ERR);
	tmc5262_emul_ramp_stat_set(C5262->emul, TMC5262_RAMP_STAT_STATUS_STOP_L);
	zassert_ok(tmc5262_reg_read(C5262->dev, TMC5262_REG_GSTAT, &u));
	zassert_ok(tmc5262_reg_read(C5262->dev, TMC5262_REG_GSTAT, &u));
	zassert_ok(tmc5262_get_spi_status_decoded(C5262->dev, &st));
	/* StallGuard is off (TCOOLTHRS = 0): the flag reads 1 */
	zassert_true(st.reset_flag && st.driver_error && st.standstill && st.sg);
	zassert_true(st.ramp_valid);
	zassert_true(st.stop_l && !st.stop_r && st.position_reached);

	/* On the TMC2262 bits 7..4 are not used */
	zassert_ok(tmc5262_reg_read(C2262->dev, TMC5262_REG_GSTAT, &u));
	zassert_ok(tmc5262_get_spi_status_decoded(C2262->dev, &st));
	zassert_false(st.ramp_valid);
	zassert_false(st.velocity_reached || st.position_reached || st.stop_l || st.stop_r);
}

static atomic_t n_enc_dev, n_enc_n;

static void enc_cb(const struct device *dev, enum tmc5262_encoder_event ev, void *ud)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(ud);

	atomic_inc(ev == TMC5262_ENCODER_EVENT_DEVIATION ? &n_enc_dev : &n_enc_n);
}

/* FR-FEATURE-010, -007, B7: encoder / TriCoder events and the TriCoder readback */
ZTEST(tmc5262, test_tricoder_and_encoder_events)
{
	struct tmc5262_tricoder tc;
	const struct emul *e = C5262->emul;

	atomic_clear(&n_enc_dev);
	atomic_clear(&n_enc_n);
	zassert_ok(tmc5262_set_encoder_event_cb(C5262->dev, enc_cb, NULL));

	/* A persisting deviation warning (TriCoder step loss) is reported once */
	tmc5262_emul_set_enc_status(e, TMC5262_ENC_STATUS_DEVIATION_WARN, true);
	zassert_true(wait_count(&n_enc_dev, 1));
	k_msleep(60);
	zassert_equal(atomic_get(&n_enc_dev), 1);
	zassert_equal(atomic_get(&n_enc_n), 0);

	/* The N event is write-1-to-clear: reported once, then cleared by the driver */
	tmc5262_emul_set_enc_status(e, TMC5262_ENC_STATUS_N_EVENT, false);
	zassert_true(wait_count(&n_enc_n, 1));
	k_msleep(40);
	zassert_equal(atomic_get(&n_enc_n), 1);
	zassert_equal(tmc5262_emul_reg_get(e, TMC5262_REG_ENC_STATUS) & TMC5262_ENC_STATUS_N_EVENT,
		      0U);
	zassert_ok(tmc5262_set_encoder_event_cb(C5262->dev, NULL, NULL));

	/* SGP_BEMF readback */
	tmc5262_emul_reg_set(e, TMC5262_REG_SGP_BEMF, (0x123U << 16) | 0x055U);
	zassert_ok(tmc5262_get_tricoder(C5262->dev, &tc));
	zassert_equal(tc.ubemf_abs, 0x123);
	zassert_equal(tc.sgp_raw, 0x055);
	zassert_true(tc.step_loss);
	zassert_equal(tmc5262_get_tricoder(C3241->dev, &tc), -ENOTSUP);

	/* The TriCoder selection bit comes from the variant descriptor */
	zassert_ok(tmc5262_set_tricoder(C5262->dev, true,
					TMC5262_ENCMODE_BEMF_BLANK_TIME & (20U << 16)));
	zassert_equal(tmc5262_emul_reg_get(e, TMC5262_REG_ENCMODE) & TMC5262_ENCMODE_NBEMF_ABN_SEL,
		      0U);
	zassert_ok(tmc5262_set_tricoder(C5262->dev, false, 0));
	zassert_not_equal(
		tmc5262_emul_reg_get(e, TMC5262_REG_ENCMODE) & TMC5262_ENCMODE_NBEMF_ABN_SEL, 0U);
	zassert_equal(tmc5262_set_tricoder(C5262->dev, true, TMC5262_ENCMODE_POL_A), -EINVAL);
	zassert_ok(tmc5262_set_encoder_event_cb(C5262->dev, NULL, NULL));
}

/* FR-ERR-002: multi-register APIs validate every parameter before the first write */
ZTEST(tmc5262, test_no_partial_writes)
{
	const struct emul *e = C5262->emul;
	const struct device *d = C5262->dev;

	tmc5262_emul_log_clear(e);
	zassert_equal(tmc5262_set_stealthchop_regulators(d, 0x000A0040U, 0x00140032U, 0x0FFF0100U,
							 0x1000U),
		      -EINVAL);
	zassert_equal(tmc5262_set_stealthchop_regulators(d, 0x80000000U, 0, 0, 0), -EINVAL);
	zassert_equal(tmc5262_set_coolstep_plus(d, 0x12, 0x00100080U, 0xF0000000U, 0), -EINVAL);
	zassert_equal(tmc5262_set_coolstep_plus(d, 0x20, 0, 0, 0), -EINVAL);
	zassert_equal(tmc5262_set_stallguard_plus(d, 0, 0x100000U), -EINVAL);
	zassert_equal(tmc5262_set_rcoil_user(d, 0x1000, 0), -EINVAL);
	zassert_equal(tmc5262_emul_log_count(e), 0,
		      "a rejected call leaves the hardware unchanged");

	/* Accepted values are cached and survive a reset */
	zassert_ok(tmc5262_set_stealthchop_regulators(d, 0x000B0041U, 0x00150033U, 0x0FFF0101U,
						      0x101U));
	tmc5262_emul_reset(e);
	zassert_ok(tmc5262_reinit(d));
	zassert_equal(tmc5262_emul_reg_get(e, TMC5262_REG_CURRENT_PI_REG), 0x000B0041U);
	zassert_equal(tmc5262_emul_reg_get(e, TMC5262_REG_ANGLE_LOWER_LIMIT), 0x101U);
	zassert_ok(tmc5262_set_stealthchop_regulators(d, 0x000A0040U, 0x00140032U, 0x0FFF0100U,
						      0x100U));
}

/* m11: accelerations above the 18 bit register range are rejected, not clamped */
ZTEST(tmc5262, test_acceleration_range)
{
	struct stepper_ctrl_ramp ramp = {
		.acceleration_max = 30000000,
		.speed_max = 40000,
		.deceleration_max = 30000000,
	};
	const struct emul *e = C5262->emul;

	tmc5262_emul_log_clear(e);
	/* 4 x 10^7 would be 343597 register units (limit 262143) */
	ramp.acceleration_max = 40000000;
	zassert_equal(stepper_ctrl_configure_ramp(C5262->ctrl, &ramp), -EINVAL);
	ramp.acceleration_max = 30000000;
	ramp.deceleration_max = 40000000;
	zassert_equal(stepper_ctrl_configure_ramp(C5262->ctrl, &ramp), -EINVAL);
	zassert_equal(tmc5262_emul_log_count(e), 0);

	ramp.deceleration_max = 30000000;
	zassert_ok(stepper_ctrl_configure_ramp(C5262->ctrl, &ramp));
	/* 30e6 * 2^41 / (16e6)^2 = 257698 */
	zassert_equal(tmc5262_emul_reg_get(e, TMC5262_REG_AMAX), 257698U);
}

/* TC-INIT-007: a PLL commit that never clears times out after 10 ms, three attempts */
ZTEST(tmc5262, test_pll_commit_timeout)
{
	const struct emul *e = C5262->emul;
	int64_t t0;
	int commits;

	tmc5262_emul_reset(e);
	commits = tmc5262_emul_pll_commits(e);
	tmc5262_emul_pll_commit_stuck(e, true);

	t0 = k_uptime_get();
	zassert_equal(tmc5262_reinit(C5262->dev), -ETIMEDOUT);
	zassert_true(k_uptime_get() - t0 >= 30, "three polls of 10 ms");
	zassert_true(k_uptime_get() - t0 < 200, "the polling is bounded");
	zassert_equal(tmc5262_emul_pll_commits(e) - commits, 3);

	/* The power stage is not enabled by a failed start-up and a retry recovers */
	zassert_equal(gpio_emul_output_get(C5262->drv_enn.port, C5262->drv_enn.pin), 1);
	tmc5262_emul_pll_commit_stuck(e, false);
	zassert_ok(tmc5262_restart_pll(C5262->dev));
}

/* Ramp generator variants take the standstill from RAMP_STAT.vzero, not from DRV_STATUS.stst */
ZTEST(tmc5262, test_standstill_source)
{
	struct tmc5262_drv_status st;

	tmc5262_emul_start_motion(C5262->emul, 1000);
	tmc5262_emul_finish_move(C5262->emul);

	/* stst is not set yet (it needs time after the last step) ... */
	zassert_ok(tmc5262_get_drv_status(C5262->dev, &st));
	zassert_false(st.standstill);
	/* ... but the ramp generator reports vzero, which is what the driver uses */
	zassert_ok(tmc5262_set_current_range(C5262->dev, 3, 100));
	zassert_ok(stepper_ctrl_set_reference_position(C5262->ctrl, 5));
	zassert_true(wait_standstill(C5262));
}

/* The datagram after a write returns the written data: reads discard the first response */
ZTEST(tmc5262, test_read_after_write_pipelining)
{
	uint32_t u;

	tmc5262_emul_reg_set(C5262->emul, TMC5262_REG_GSTAT, TMC5262_GSTAT_UV_CP);
	zassert_ok(tmc5262_reg_write(C5262->dev, TMC5262_REG_TPOWERDOWN, 0x55));
	zassert_ok(tmc5262_reg_read(C5262->dev, TMC5262_REG_GSTAT, &u));
	zassert_equal(u, (uint32_t)TMC5262_GSTAT_UV_CP, "the echo of the write is not the data");
	zassert_ok(tmc5262_reg_write(C5262->dev, TMC5262_REG_TPOWERDOWN, 10));
	zassert_ok(tmc5262_reg_read(C5262->dev, TMC5262_REG_TPOWERDOWN, &u));
	zassert_equal(u, 10U);
}

/* ---- Tests added for the traceability audit findings ---- */

/* TC-MOTION-010: velocity run in both directions, VMAX before the mode, stop after the run */
ZTEST(tmc5262, test_ctrl_run_ramp_variants)
{
	const struct chip *const ramp_chips[] = {C5262, C3241};

	for (size_t i = 0; i < ARRAY_SIZE(ramp_chips); i++) {
		const struct chip *c = ramp_chips[i];
		const struct emul *e = c->emul;
		bool moving;

		clear_counters();
		zassert_ok(stepper_ctrl_set_event_cb(c->ctrl, ctrl_cb, NULL));
		zassert_ok(stepper_enable(c->drv));

		for (int dir = 0; dir < 2; dir++) {
			const uint32_t mode = dir == 0 ? TMC5262_RAMPMODE_VELOCITY_POS
						       : TMC5262_RAMPMODE_VELOCITY_NEG;
			int i_v;
			int i_m;

			tmc5262_emul_log_clear(e);
			zassert_ok(stepper_ctrl_run(c->ctrl,
						    dir == 0 ? STEPPER_CTRL_DIRECTION_POSITIVE
							     : STEPPER_CTRL_DIRECTION_NEGATIVE));

			/* The devicetree VMAX is written first, then the mode starts the motion */
			i_v = tmc5262_emul_log_find(e, TMC5262_REG_VMAX, true, 53687U, 0);
			i_m = tmc5262_emul_log_find(e, TMC5262_REG_RAMPMODE, true, mode, 0);
			zassert_true(i_v >= 0 && i_m > i_v, "chip %zu dir %d: %d %d", i, dir, i_v,
				     i_m);
			zassert_equal(tmc5262_emul_reg_get(e, TMC5262_REG_RAMPMODE), mode);
			zassert_equal(tmc5262_emul_reg_get(e, TMC5262_REG_VMAX), 53687U);
			zassert_ok(stepper_ctrl_is_moving(c->ctrl, &moving));
			zassert_true(moving);

			/* The motor stops after the run: positioning mode, STOPPED reported once */
			zassert_ok(stepper_ctrl_stop(c->ctrl));
			zassert_true(wait_count(&n_stopped, dir + 1), "chip %zu dir %d", i, dir);
			zassert_ok(stepper_ctrl_is_moving(c->ctrl, &moving));
			zassert_false(moving);
			zassert_equal(tmc5262_emul_reg_get(e, TMC5262_REG_RAMPMODE),
				      TMC5262_RAMPMODE_POSITION);
			zassert_equal(tmc5262_emul_reverse_risk(e), 0U);
			zassert_equal(tmc5262_emul_vmax_in_hold(e), 0U);
		}

		k_msleep(50);
		zassert_equal(atomic_get(&n_stopped), 2);
		zassert_equal(tmc5262_emul_invalid_access(e), 0U);
		zassert_ok(stepper_disable(c->drv));
	}
}

static int run_init(const struct chip *c)
{
	return c->dev->ops.init(c->dev);
}

/* TC-INIT-006, TC-INIT-008, TC-INIT-003: init fails when the chip is missing or the SPI fails */
ZTEST(tmc5262, test_init_failure_paths)
{
	/* With SLEEPN: the reset handshake waits 10 ms for a plausible reply */
	const struct chip *c = C5262;
	const struct emul *e = c->emul;

	tmc5262_emul_set_silent(e, true);
	zassert_equal(run_init(c), -ETIMEDOUT, "silent device");
	tmc5262_emul_set_silent(e, false);

	tmc5262_emul_set_floating(e, true);
	zassert_equal(run_init(c), -ETIMEDOUT, "floating bus (all ones)");
	tmc5262_emul_set_floating(e, false);

	tmc5262_emul_io_error(e, -EIO, 0);
	zassert_equal(run_init(c), -ETIMEDOUT, "SPI transfers fail");
	tmc5262_emul_io_error(e, 0, 0);

	/* The reference resistor is missing: presence check fails */
	tmc5262_emul_set_no_resistor(e, true);
	zassert_equal(run_init(c), -EIO, "no reference resistor");
	tmc5262_emul_set_no_resistor(e, false);

	/* The SPI fails during the PLL start-up and the configuration */
	for (int after = 6; after < 14; after += 4) {
		tmc5262_emul_io_error(e, -EIO, after);
		zassert_true(run_init(c) < 0, "SPI error after %d transfers", after);
		tmc5262_emul_io_error(e, 0, 0);
	}

	/* A failed init never leaves the power stage enabled */
	zassert_equal(gpio_emul_output_get(c->drv_enn.port, c->drv_enn.pin), 1);

	/* Without SLEEPN the presence check itself reports the failure */
	c = C3241B;
	e = c->emul;
	tmc5262_emul_set_silent(e, true);
	zassert_equal(run_init(c), -EIO, "silent device without SLEEPN");
	tmc5262_emul_set_silent(e, false);
	tmc5262_emul_set_floating(e, true);
	zassert_equal(run_init(c), -EIO, "floating bus without SLEEPN");
	tmc5262_emul_set_floating(e, false);
	tmc5262_emul_io_error(e, -EIO, 0);
	zassert_equal(run_init(c), -EIO, "SPI error without SLEEPN");
	tmc5262_emul_io_error(e, 0, 0);
	tmc5262_emul_io_error(e, -EIO, 5);
	zassert_true(run_init(c) < 0, "SPI error in the PLL start-up");
	tmc5262_emul_io_error(e, 0, 0);

	/* A running device is not accepted either: reinit reports the SPI error */
	zassert_ok(run_init(C5262));
	tmc5262_emul_io_error(C5262->emul, -EIO, 0);
	zassert_true(tmc5262_reinit(C5262->dev) < 0);
	tmc5262_emul_io_error(C5262->emul, 0, 0);

	/* reinit checks the presence: a chip that stopped answering is reported */
	tmc5262_emul_set_silent(C5262->emul, true);
	zassert_equal(tmc5262_reinit(C5262->dev), -EIO, "silent device on reinit");
	tmc5262_emul_set_silent(C5262->emul, false);
	tmc5262_emul_set_floating(C5262->emul, true);
	zassert_equal(tmc5262_reinit(C5262->dev), -EIO, "floating bus on reinit");
	tmc5262_emul_set_floating(C5262->emul, false);
	tmc5262_emul_set_no_resistor(C5262->emul, true);
	zassert_equal(tmc5262_reinit(C5262->dev), -EIO, "no resistor on reinit");
	tmc5262_emul_set_no_resistor(C5262->emul, false);

	/* Everything recovers once the chip answers again */
	tmc5262_emul_reset(C5262->emul);
	zassert_ok(run_init(C5262));
	zassert_ok(run_init(C3241B));
	zassert_ok(tmc5262_reinit(C5262->dev));
	zassert_equal(tmc5262_emul_reg_get(C5262->emul, TMC5262_REG_GSTAT) &
			      TMC5262_GSTAT_POWERUP_CLEAR,
		      0U);
}

/* TC-INIT-005, TC-POWER-002: the first SPI access follows SLEEPN high by at least tWAKE */
ZTEST(tmc5262, test_sleepn_wake_timing)
{
	for (size_t i = 0; i < 3; i++) {
		const struct chip *c = &chips[i];
		int64_t t0;

		/* Hardware reset in the init function: SLEEPN low, high, wait 2 ms, then SPI */
		tmc5262_emul_mark_next_transfer(c->emul);
		t0 = k_uptime_get();
		zassert_ok(run_init(c));
		zassert_true(tmc5262_emul_marked_transfer_ms(c->emul) - t0 >= 2, "init chip %zu",
			     i);
		zassert_equal(gpio_emul_output_get(c->sleepn.port, c->sleepn.pin), 1);
		zassert_equal(gpio_emul_output_get(c->drv_enn.port, c->drv_enn.pin), 1);

		/* Wake-up from sleep mode */
		zassert_ok(tmc5262_sleep(c->dev));
		zassert_equal(gpio_emul_output_get(c->sleepn.port, c->sleepn.pin), 0);
		tmc5262_emul_sync_pins(c->emul);
		tmc5262_emul_mark_next_transfer(c->emul);
		t0 = k_uptime_get();
		zassert_ok(tmc5262_wake(c->dev));
		zassert_true(tmc5262_emul_marked_transfer_ms(c->emul) - t0 >= 2, "wake chip %zu",
			     i);
		zassert_equal(gpio_emul_output_get(c->sleepn.port, c->sleepn.pin), 1);
	}
}

/* TC-FEATURE-003: a valid CoolStep configuration reaches COOLCONF, other fields stay */
ZTEST(tmc5262, test_coolstep_config)
{
	const struct chip *const cs_chips[] = {C5262, C2262, C3241};

	for (size_t i = 0; i < ARRAY_SIZE(cs_chips); i++) {
		const struct chip *c = cs_chips[i];
		const struct emul *e = c->emul;
		uint32_t cc0 = tmc5262_emul_reg_get(e, TMC5262_REG_COOLCONF);
		uint32_t cc;

		zassert_ok(tmc5262_set_coolstep(c->dev, 5, 2, 3, 1, true));
		cc = tmc5262_emul_reg_get(e, TMC5262_REG_COOLCONF);
		zassert_equal(FIELD_GET(TMC5262_COOLCONF_SEMIN, cc), 5U);
		zassert_equal(FIELD_GET(TMC5262_COOLCONF_SEUP, cc), 2U);
		zassert_equal(FIELD_GET(TMC5262_COOLCONF_SEMAX, cc), 3U);
		zassert_equal(FIELD_GET(TMC5262_COOLCONF_SEDN, cc), 1U);
		zassert_true((cc & TMC5262_COOLCONF_SEIMIN) != 0U);
		/* StallGuard2 threshold and filter are not touched */
		zassert_equal(cc & (TMC5262_COOLCONF_SGT | TMC5262_COOLCONF_SFILT),
			      cc0 & (TMC5262_COOLCONF_SGT | TMC5262_COOLCONF_SFILT));

		/* Largest values are accepted, one more is not */
		zassert_ok(tmc5262_set_coolstep(c->dev, 15, 3, 15, 7, false));
		zassert_equal(tmc5262_set_coolstep(c->dev, 16, 0, 0, 0, false), -EINVAL);
		zassert_equal(tmc5262_set_coolstep(c->dev, 1, 4, 0, 0, false), -EINVAL);
		zassert_equal(tmc5262_set_coolstep(c->dev, 1, 0, 16, 0, false), -EINVAL);
		zassert_equal(tmc5262_set_coolstep(c->dev, 1, 0, 0, 8, false), -EINVAL);

		/* semin = 0 disables CoolStep: back to the devicetree configuration */
		zassert_ok(tmc5262_set_coolstep(c->dev, 0, 0, 0, 0, false));
		zassert_equal(FIELD_GET(TMC5262_COOLCONF_SEMIN,
					tmc5262_emul_reg_get(e, TMC5262_REG_COOLCONF)),
			      0U);
		zassert_equal(tmc5262_emul_reg_get(e, TMC5262_REG_COOLCONF), cc0);
		zassert_equal(tmc5262_emul_invalid_access(e), 0U);
	}
}

/* TC-FEATURE-006, TC-VAR-015: uDcStep register layout per variant */
ZTEST(tmc5262, test_udcstep_layout)
{
	const struct emul *e = C2262->emul;
	const struct device *d = C2262->dev;
	int idx = 0;

	/* TMC2262: UDC_CONF has only DECEL_THRS (reset value 0xE) and udc_enable, bits 7:4 are
	 * reserved. The devicetree configuration must not write ACCEL_THRS there.
	 */
	tmc5262_emul_reset(e);
	tmc5262_emul_log_clear(e);
	zassert_ok(tmc5262_reinit(d));
	zassert_equal(logged(C2262, TMC5262_REG_UDC_CONF, &idx), 0x0EU);
	zassert_equal(tmc5262_emul_reg_get(e, TMC5262_REG_UDC_CONF), 0x0EU);
	zassert_equal(tmc5262_emul_reserved_writes(e), 0U);

	/* No ACCEL_THRS: it must be 0, DECEL_THRS is free (no decel > accel rule) */
	zassert_equal(tmc5262_set_udcstep(d, false, 4, 8, 0), -EINVAL);
	zassert_equal(tmc5262_set_udcstep(d, false, 0, 16, 0), -EINVAL);
	zassert_ok(tmc5262_set_udcstep(d, false, 0, 0, 100));
	zassert_equal(tmc5262_emul_reg_get(e, TMC5262_REG_UDC_CONF), 0U);
	zassert_equal(tmc5262_emul_reg_get(e, TMC5262_REG_TUDCSTEP), 100U);
	zassert_ok(tmc5262_set_udcstep(d, false, 0, 3, 100));
	zassert_equal(tmc5262_emul_reg_get(e, TMC5262_REG_UDC_CONF), 3U);

	/* Enabling needs StealthChop+ */
	zassert_equal(tmc5262_set_udcstep(d, true, 0, 5, 100), -EPERM);
	zassert_true(wait_standstill(C2262));
	zassert_ok(tmc5262_set_stealthchop(d, true));
	zassert_ok(tmc5262_set_udcstep(d, true, 0, 5, 100));
	zassert_equal(tmc5262_emul_reg_get(e, TMC5262_REG_UDC_CONF), TMC5262_UDC_CONF_ENABLE | 5U);
	zassert_equal(tmc5262_emul_reserved_writes(e), 0U);

	/* Back to the devicetree values (DECEL_THRS 14) */
	zassert_ok(tmc5262_set_udcstep(d, false, 0, 14, 0));
	zassert_equal(tmc5262_emul_reg_get(e, TMC5262_REG_UDC_CONF), 0x0EU);

	/* TMC5262: ACCEL_THRS in bits 7:4, decel must be greater than accel */
	e = C5262->emul;
	d = C5262->dev;
	zassert_equal(tmc5262_set_udcstep(d, false, 8, 8, 0), -EINVAL);
	zassert_equal(tmc5262_set_udcstep(d, false, 9, 8, 0), -EINVAL);
	zassert_ok(tmc5262_set_udcstep(d, false, 4, 8, 0));
	zassert_equal(tmc5262_emul_reg_get(e, TMC5262_REG_UDC_CONF), 0x48U);
	zassert_ok(tmc5262_set_udcstep(d, false, 12, 14, 0));
	zassert_equal(tmc5262_emul_reg_get(e, TMC5262_REG_UDC_CONF), 0xCEU);
}

/* TC-FEATURE-011: X_COMPARE_REPEAT is 24 bits wide */
ZTEST(tmc5262, test_position_compare_range)
{
	const struct {
		const struct chip *c;
		int32_t x_compare;
		uint32_t repeat;
	} dt[] = {{C5262, -1, 0}, {C3241, 5000, 10}};

	for (size_t i = 0; i < ARRAY_SIZE(dt); i++) {
		const struct chip *c = dt[i].c;
		const struct emul *e = c->emul;

		zassert_ok(tmc5262_set_position_compare(c->dev, 100, 0xFFFFFF));
		zassert_equal(tmc5262_emul_reg_get(e, TMC5262_REG_X_COMPARE_REPEAT), 0xFFFFFFU);
		zassert_equal(tmc5262_emul_reg_get(e, TMC5262_REG_X_COMPARE), 100U);

		/* A repeat distance that does not fit in 24 bits writes nothing */
		tmc5262_emul_log_clear(e);
		zassert_equal(tmc5262_set_position_compare(c->dev, 200, 0x1000000U), -EINVAL);
		zassert_equal(tmc5262_set_position_compare(c->dev, 200, 0xFFFFFFFFU), -EINVAL);
		zassert_equal(tmc5262_emul_log_count(e), 0);
		zassert_equal(tmc5262_emul_reg_get(e, TMC5262_REG_X_COMPARE_REPEAT), 0xFFFFFFU);

		zassert_ok(tmc5262_set_position_compare(c->dev, dt[i].x_compare, dt[i].repeat));
	}
}

/* TC-MOTION-007, -008: VMAX is limited to 2^23 - 512 */
ZTEST(tmc5262, test_vmax_limit)
{
	const struct chip *const ramp_chips[] = {C5262, C3241};
	struct stepper_ctrl_ramp ramp = {
		.acceleration_max = 100000,
		.speed_max = 7999512, /* VMAX register 8388096 = 2^23 - 512 */
		.deceleration_max = 100000,
	};

	for (size_t i = 0; i < ARRAY_SIZE(ramp_chips); i++) {
		const struct chip *c = ramp_chips[i];
		const struct emul *e = c->emul;

		zassert_ok(stepper_enable(c->drv));
		zassert_ok(stepper_ctrl_configure_ramp(c->ctrl, &ramp));
		zassert_ok(stepper_ctrl_move_to(c->ctrl, 1));
		zassert_equal(tmc5262_emul_reg_get(e, TMC5262_REG_VMAX), (1U << 23) - 512U);
		tmc5262_emul_finish_move(e);

		/* One step above the limit, a value below 2^23 that the field would accept */
		tmc5262_emul_log_clear(e);
		ramp.speed_max = 7999513;
		zassert_equal(stepper_ctrl_configure_ramp(c->ctrl, &ramp), -EINVAL);
		ramp.speed_max = 7999600;
		zassert_equal(stepper_ctrl_configure_ramp(c->ctrl, &ramp), -EINVAL);
		ramp.speed_max = 8000000;
		zassert_equal(stepper_ctrl_configure_ramp(c->ctrl, &ramp), -EINVAL);
		/* 125 ns per microstep is 8 MHz, 126 ns is below the limit */
		zassert_equal(stepper_ctrl_set_microstep_interval(c->ctrl, 125), -EINVAL);
		zassert_equal(tmc5262_emul_log_count(e), 0, "a rejected ramp writes nothing");
		zassert_ok(stepper_ctrl_set_microstep_interval(c->ctrl, 126));
		ramp.speed_max = 7999512;
		zassert_ok(stepper_disable(c->drv));
	}
}

/* TC-DIAG-007: the coil resistance measurement waits for the chip */
ZTEST(tmc5262, test_measure_rcoil_wait)
{
	const struct {
		const struct chip *c;
		uint32_t t_meas;
	} dt[] = {{C5262, 4096}, {C2262, 2000}};

	for (size_t i = 0; i < ARRAY_SIZE(dt); i++) {
		const struct chip *c = dt[i].c;
		const struct emul *e = c->emul;
		struct tmc5262_rcoil rc;
		int64_t t0;

		zassert_true(wait_standstill(c));

		/* The result appears 60 ms after the start: the read waits at least 100 ms */
		tmc5262_emul_rcoil_model(e, 60, (216U << 16) | 220U);
		zassert_ok(tmc5262_measure_rcoil(c->dev, dt[i].t_meas, &rc));
		zassert_equal(rc.a, 216);
		zassert_equal(rc.b, 220);
		zassert_true(tmc5262_emul_rcoil_first_read_ms(e) >= 100,
			     "R_COIL read after %lld ms", tmc5262_emul_rcoil_first_read_ms(e));
		zassert_equal(tmc5262_emul_reg_get(e, TMC5262_REG_T_RCOIL_MEAS), dt[i].t_meas);

		/* Slower chip: still 0 after 100 ms, polled again */
		tmc5262_emul_rcoil_model(e, 130, (100U << 16) | 110U);
		zassert_ok(tmc5262_measure_rcoil(c->dev, dt[i].t_meas, &rc));
		zassert_equal(rc.a, 100);
		zassert_equal(rc.b, 110);
		/* the first read at 100 ms returned 0, the driver polled again */
		zassert_true(tmc5262_emul_rcoil_first_read_ms(e) < 130);

		/* A chip that never measures: bounded polling, then -ETIMEDOUT */
		tmc5262_emul_rcoil_model(e, 100000, 0x00010001U);
		t0 = k_uptime_get();
		zassert_equal(tmc5262_measure_rcoil(c->dev, dt[i].t_meas, &rc), -ETIMEDOUT);
		zassert_true(k_uptime_get() - t0 >= 300, "waited %lld ms", k_uptime_get() - t0);
		zassert_true(k_uptime_get() - t0 < 400, "bounded: %lld ms", k_uptime_get() - t0);

		/* Only valid at standstill, a rejected call does not wait */
		tmc5262_emul_rcoil_model(e, -1, 0);
		tmc5262_emul_start_motion(e, 1000);
		t0 = k_uptime_get();
		zassert_equal(tmc5262_measure_rcoil(c->dev, dt[i].t_meas, &rc), -EBUSY);
		zassert_true(k_uptime_get() - t0 < 50);
		tmc5262_emul_finish_move(e);
		zassert_equal(tmc5262_measure_rcoil(c->dev, 0x100000U, &rc), -EINVAL);
	}
}

/* TC-MOTION-001, -004, -005: STEP/DIR motion of the TMC2262 through the gpio step-dir
 * controller, which reads the step width and active edge from the driver child
 */
ZTEST(tmc5262, test_tmc2262_step_dir_motion)
{
	const struct emul *e = C2262->emul;
	const struct gpio_dt_spec dir = C2262_DIR;
	int32_t pos;
	bool moving;

	zassert_true(device_is_ready(C2262_CTRL));
	zassert_ok(stepper_ctrl_set_event_cb(C2262_CTRL, ctrl_cb, NULL));
	zassert_ok(stepper_enable(C2262->drv));
	tmc5262_emul_log_clear(e);

	/* No ramp generator: the ramp generator API of the chip is rejected without bus access */
	zassert_equal(tmc5262_set_step_source(C2262->dev, true), -ENOTSUP);
	zassert_equal(tmc5262_get_actual_velocity(C2262->dev, &pos), -ENOTSUP);

	zassert_ok(stepper_ctrl_set_microstep_interval(C2262_CTRL, 1000000));
	zassert_ok(stepper_ctrl_move_by(C2262_CTRL, 10));
	zassert_true(wait_count(&n_steps, 1), "steps completed");
	zassert_ok(stepper_ctrl_get_actual_position(C2262_CTRL, &pos));
	zassert_equal(pos, 10);
	zassert_equal(gpio_emul_output_get(dir.port, dir.pin), 1);

	zassert_ok(stepper_ctrl_move_by(C2262_CTRL, -4));
	zassert_true(wait_count(&n_steps, 2), "steps completed");
	zassert_ok(stepper_ctrl_get_actual_position(C2262_CTRL, &pos));
	zassert_equal(pos, 6);
	zassert_equal(gpio_emul_output_get(dir.port, dir.pin), 0);
	zassert_ok(stepper_ctrl_is_moving(C2262_CTRL, &moving));
	zassert_false(moving);

	/* The STEP/DIR motion is independent of the SPI interface */
	zassert_equal(tmc5262_emul_log_count(e), 0);
	zassert_equal(tmc5262_emul_invalid_access(e), 0U);

	zassert_ok(stepper_ctrl_set_reference_position(C2262_CTRL, 100));
	zassert_ok(stepper_ctrl_get_actual_position(C2262_CTRL, &pos));
	zassert_equal(pos, 100);
	zassert_ok(stepper_disable(C2262->drv));
}

/* TC-MOTION-006 (STEP/DIR note): the step-dir controller has no link back to the driver, so
 * stepper_disable() cannot stop it. The application stops the controller first, as documented
 * in stepper_tmc5262.h.
 */
ZTEST(tmc5262, test_tmc2262_disable_needs_ctrl_stop)
{
	int32_t pos0;
	int32_t pos1;
	int32_t pos2;
	bool moving;

	zassert_ok(stepper_enable(C2262->drv));
	zassert_ok(stepper_ctrl_set_microstep_interval(C2262_CTRL, 1000000));
	zassert_ok(stepper_ctrl_run(C2262_CTRL, STEPPER_CTRL_DIRECTION_POSITIVE));
	k_msleep(100);

	/* The driver is disabled (DRV_ENN high) ... */
	zassert_ok(stepper_disable(C2262->drv));
	zassert_equal(gpio_emul_output_get(C2262->drv_enn.port, C2262->drv_enn.pin), 1);

	/* ... but the controller keeps stepping until the application stops it */
	zassert_ok(stepper_ctrl_get_actual_position(C2262_CTRL, &pos0));
	zassert_true(pos0 > 0);
	k_msleep(100);
	zassert_ok(stepper_ctrl_get_actual_position(C2262_CTRL, &pos1));
	zassert_true(pos1 > pos0, "the controller still steps: %d %d", pos0, pos1);

	zassert_ok(stepper_ctrl_stop(C2262_CTRL));
	zassert_ok(stepper_ctrl_get_actual_position(C2262_CTRL, &pos1));
	k_msleep(100);
	zassert_ok(stepper_ctrl_get_actual_position(C2262_CTRL, &pos2));
	zassert_equal(pos1, pos2, "stopped");
	zassert_ok(stepper_ctrl_is_moving(C2262_CTRL, &moving));
	zassert_false(moving);
}

/* TC-POWER-004 on the TMC2262: motion commands go to the step-dir controller and never reach
 * the driver, so the reset is detected by stepper_enable() (and by events), not by a move
 */
ZTEST(tmc5262, test_tmc2262_reset_detected_on_enable)
{
	const struct emul *e = C2262->emul;

	zassert_ok(stepper_enable(C2262->drv));
	zassert_ok(stepper_ctrl_set_event_cb(C2262_CTRL, ctrl_cb, NULL));
	zassert_ok(stepper_ctrl_set_microstep_interval(C2262_CTRL, 1000000));
	tmc5262_emul_reset(e);
	tmc5262_emul_log_clear(e);

	zassert_ok(stepper_ctrl_move_by(C2262_CTRL, 5));
	zassert_true(wait_count(&n_steps, 1));
	zassert_equal(tmc5262_emul_log_count(e), 0, "the move did not touch the chip");

	zassert_ok(stepper_enable(C2262->drv));
	zassert_true(tmc5262_emul_log_find(e, TMC5262_REG_PLL, false, 0, 0) >= 0,
		     "the reset was detected and the configuration applied");
	zassert_equal(gpio_emul_output_get(C2262->drv_enn.port, C2262->drv_enn.pin), 0);
	zassert_ok(stepper_disable(C2262->drv));
}

/* TC-DIAG-004, -006, TC-FEATURE-007, -008, -009, TC-POWER-001..003 on the TMC2262: the shared
 * code paths that the other tests run on the TMC5262 only
 */
ZTEST(tmc5262, test_tmc2262_shared_features)
{
	const struct emul *e = C2262->emul;
	const struct device *d = C2262->dev;
	struct tmc5262_tricoder tc;
	int32_t mv;
	int32_t mdegc;
	int16_t ia;
	int16_t ib;
	int16_t ca;
	int16_t cb;
	uint16_t mscnt;
	uint32_t u;

	/* ADC: supply voltage, temperature and coil current */
	tmc5262_emul_reg_set(e, TMC5262_REG_ADC_VSUPPLY_TEMP, (300U << 16) | 170U);
	zassert_ok(tmc5262_get_supply_mv(d, &mv));
	zassert_equal(mv, 23953);
	zassert_ok(tmc5262_get_temperature_mdegc(d, &mdegc));
	zassert_equal(mdegc, 48000);
	tmc5262_emul_reg_set(e, TMC5262_REG_ADC_I, (0xFFEU << 16) | 0x7FU);
	zassert_ok(tmc5262_get_coil_current_raw(d, &ia, &ib));
	zassert_equal(ia, 127);
	zassert_equal(ib, -2);

	/* TSTEP, steps lost, sequencer */
	tmc5262_emul_reg_set(e, TMC5262_REG_TSTEP, 1000);
	zassert_ok(tmc5262_get_step_frequency(d, &u));
	zassert_equal(u, 16000U);
	tmc5262_emul_reg_set(e, TMC5262_REG_STEPS_LOST, 77);
	zassert_ok(tmc5262_get_steps_lost(d, &u));
	zassert_equal(u, 77U);
	tmc5262_emul_reg_set(e, TMC5262_REG_MSCNT, 0x123);
	tmc5262_emul_reg_set(e, TMC5262_REG_MSCURACT, (0x1FFU << 16) | 0x7FU);
	zassert_ok(tmc5262_get_sequencer(d, &mscnt, &ca, &cb));
	zassert_equal(mscnt, 0x123);
	zassert_equal(ca, -1);
	zassert_equal(cb, 127);

	/* TriCoder readback and selection */
	tmc5262_emul_reg_set(e, TMC5262_REG_SGP_BEMF, (0x123U << 16) | 0x055U);
	zassert_ok(tmc5262_get_tricoder(d, &tc));
	zassert_equal(tc.ubemf_abs, 0x123);
	zassert_equal(tc.sgp_raw, 0x055);
	zassert_ok(tmc5262_set_tricoder(d, true, FIELD_PREP(TMC5262_ENCMODE_BEMF_BLANK_TIME, 20)));
	zassert_equal(tmc5262_emul_reg_get(e, TMC5262_REG_ENCMODE) & TMC5262_ENCMODE_NBEMF_ABN_SEL,
		      0U);
	zassert_equal(FIELD_GET(TMC5262_ENCMODE_BEMF_BLANK_TIME,
				tmc5262_emul_reg_get(e, TMC5262_REG_ENCMODE)),
		      20U);
	zassert_ok(tmc5262_set_tricoder(d, false, 0));
	zassert_not_equal(
		tmc5262_emul_reg_get(e, TMC5262_REG_ENCMODE) & TMC5262_ENCMODE_NBEMF_ABN_SEL, 0U);
	/* Back to the reset configuration */
	zassert_ok(tmc5262_set_tricoder(d, true, FIELD_PREP(TMC5262_ENCMODE_BEMF_BLANK_TIME, 16)));

	/* Diagnostic events: a stall, then a fault that disables the power stage */
	zassert_ok(stepper_set_event_cb(C2262->drv, drv_cb, NULL));
	zassert_ok(stepper_enable(C2262->drv));
	/* StallGuard is active for TCOOLTHRS >= TSTEP */
	zassert_ok(tmc5262_set_tcoolthrs(d, 1000));
	tmc5262_emul_reg_set(e, TMC5262_REG_TSTEP, 0);
	tmc5262_emul_start_motion(e, 1000);
	tmc5262_emul_set_stall(e, true);
	zassert_true(wait_count(&n_stall, 1), "no stall event");
	tmc5262_emul_set_stall(e, false);
	tmc5262_emul_set_fault(e, TMC5262_DRV_STATUS_S2GA, true);
	zassert_true(wait_count(&n_fault, 1), "no fault event");
	zassert_equal(gpio_emul_output_get(C2262->drv_enn.port, C2262->drv_enn.pin), 1);

	/* Sleep, wake and device power management */
	zassert_ok(stepper_set_event_cb(C2262->drv, NULL, NULL));
	tmc5262_emul_reset(e);
	zassert_ok(tmc5262_reinit(d));
	zassert_ok(tmc5262_sleep(d));
	zassert_equal(gpio_emul_output_get(C2262->sleepn.port, C2262->sleepn.pin), 0);
	zassert_equal(tmc5262_reg_read(d, TMC5262_REG_GSTAT, &u), -ENODEV);
	tmc5262_emul_sync_pins(e);
	zassert_ok(tmc5262_wake(d));
	zassert_ok(tmc5262_reg_read(d, TMC5262_REG_GSTAT, &u));
	zassert_ok(pm_device_action_run(d, PM_DEVICE_ACTION_SUSPEND));
	tmc5262_emul_sync_pins(e);
	zassert_ok(pm_device_action_run(d, PM_DEVICE_ACTION_RESUME));
	zassert_equal(tmc5262_emul_invalid_access(e), 0U);
}
