/*
 * SPDX-FileCopyrightText: Copyright (c) 2026 Analog Devices, Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Private definitions of the unified TMC5262 / TMC2262 / TMC3241 driver
 */

#ifndef ZEPHYR_DRIVERS_STEPPER_ADI_TMC_TMC5262_TMC5262_H_
#define ZEPHYR_DRIVERS_STEPPER_ADI_TMC_TMC5262_TMC5262_H_

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/stepper/stepper.h>
#include <zephyr/drivers/stepper/stepper_ctrl.h>
#include <zephyr/drivers/stepper/stepper_tmc5262.h>
#include <zephyr/kernel.h>

#include <adi_tmc_bus.h>

#include "tmc5262_reg.h"

#ifdef __cplusplus
extern "C" {
#endif

/** The PLL always provides a fixed 16 MHz system clock to the logic (datasheet "Internal PLL") */
#define TMC5262_FCLK_HZ 16000000U

/** Time after which a SPI access is retried / PLL commit is declared stuck */
#define TMC5262_PLL_COMMIT_TIMEOUT_MS 10
#define TMC5262_PLL_MAX_ATTEMPTS      3
#define TMC5262_RESET_PULSE_US        50
#define TMC5262_PRESENCE_TIMEOUT_MS   10
#define TMC5262_STOP_TIMEOUT_MIN_MS   200
#define TMC5262_STOP_TIMEOUT_MAX_MS   10000
/** Slow polling factor used for events when DO interrupts are available */
#define TMC5262_POLL_SLOW_FACTOR      10
/** Polling interval while a stop is completing */
#define TMC5262_STOP_POLL_MS          5
/** Time for the ramp generator to latch an event after a register write (fCLK / 512) */
#define TMC5262_POS_EVENT_SETTLE_US   100

/** Error returned while the chip is in sleep mode (-EBUSY is reserved for motion conflicts) */
#define TMC5262_ERR_ASLEEP (-ENODEV)

/** Bits of tmc5262_data.pending, handled by the event work item */
#define TMC5262_PEND_FAULT   0
#define TMC5262_PEND_STOPPED 1

/** @name Variant capability flags (FR-VAR-001, FR-VAR-002) */
/** @{ */
#define TMC5262_CAP_RAMP_GEN        BIT(0)
#define TMC5262_CAP_REF_SWITCH      BIT(1)
#define TMC5262_CAP_POS_COMPARE     BIT(2)
#define TMC5262_CAP_STEALTHCHOP     BIT(3)
#define TMC5262_CAP_STALLGUARD_PLUS BIT(4)
#define TMC5262_CAP_COOLSTEP_PLUS   BIT(5)
#define TMC5262_CAP_UDCSTEP         BIT(6)
#define TMC5262_CAP_COIL_MEAS       BIT(7)
#define TMC5262_CAP_ADC_I           BIT(8)
#define TMC5262_CAP_ENC_DEVIATION   BIT(9)
#define TMC5262_CAP_DIRECT_MODE     BIT(10)
#define TMC5262_CAP_TRICODER        BIT(11)
#define TMC5262_CAP_PER_COIL_FAULT  BIT(12)
#define TMC5262_CAP_XLATCH          BIT(13)
#define TMC5262_CAP_VACTUAL         BIT(14)
#define TMC5262_CAP_STEP_SOURCE_SEL BIT(15)
/** @} */

/** Inclusive register address range */
struct tmc5262_reg_range {
	uint8_t first;
	uint8_t last;
};

/** DRV_STATUS flag masks of a variant, 0 if the flag does not exist */
struct tmc5262_status_masks {
	uint32_t stst;
	uint32_t ol_a;
	uint32_t ol_b;
	uint32_t s2g_a;
	uint32_t s2g_b;
	uint32_t s2vs_a;
	uint32_t s2vs_b;
	uint32_t otpw;
	uint32_t ot;
	uint32_t stallguard;
	uint32_t stealth;
	uint32_t ov;
	uint32_t seq_stopped;
};

/**
 * Constant per-variant data: register availability, bit-field layouts, current
 * range table, timing and capabilities (FR-VAR-001).
 */
struct tmc5262_variant {
	const char *name;
	enum tmc5262_variant_id id;
	uint32_t caps;
	/** Wake-up time after SLEEPN is released */
	uint16_t twake_ms;
	/** Number of motor phases (2 or 3) */
	uint8_t n_phases;
	/** GCONF bit that selects the step source, 0 if fixed to STEP/DIR */
	uint32_t step_src_mask;
	/** true if the step_src_mask bit set means "ramp generator" */
	bool step_src_ramp_when_set;
	/** GCONF en_stealthchop bit, 0 if StealthChop+ does not exist */
	uint32_t gconf_stealth_mask;
	/** ENCMODE bits that exist on the variant */
	uint32_t encmode_mask;
	/** ENCMODE back-EMF (TriCoder) fields, 0 if the variant has no TriCoder */
	uint32_t bemf_mask;
	/** ENCMODE bit that selects the incremental encoder (set) instead of the TriCoder */
	uint32_t bemf_encoder_sel;
	/** CHOPCONF bits that exist on the variant */
	uint32_t chopconf_mask;
	/** DRV_STATUS layout */
	struct tmc5262_status_masks status;
	/** DRV_CONF.CURRENT_RANGE decode, in mA (RMS, or peak if current_range_is_peak) */
	uint16_t current_range_ma[4];
	bool current_range_is_peak;
	/** Valid register address ranges */
	const struct tmc5262_reg_range *regs;
	uint8_t n_regs;
};

#if defined(CONFIG_STEPPER_ADI_TMC5262_TMC5262) || defined(__DOXYGEN__)
extern const struct tmc5262_variant tmc5262_variant_tmc5262;
#endif
#if defined(CONFIG_STEPPER_ADI_TMC5262_TMC2262) || defined(__DOXYGEN__)
extern const struct tmc5262_variant tmc5262_variant_tmc2262;
#endif
#if defined(CONFIG_STEPPER_ADI_TMC5262_TMC3241) || defined(__DOXYGEN__)
extern const struct tmc5262_variant tmc5262_variant_tmc3241;
#endif

/** Check that a register exists on the variant. */
bool tmc5262_variant_has_reg(const struct tmc5262_variant *variant, uint8_t reg);

/** Registers whose value is cached so that it can be re-applied after a reset */
struct tmc5262_shadow {
	uint32_t gconf;
	uint32_t do_conf;
	uint32_t drv_conf;
	uint32_t ihold_irun;
	uint32_t tpowerdown;
	uint32_t chopconf;
	uint32_t coolconf;
	uint32_t pwmconf;
	uint32_t tpwmthrs;
	uint32_t tcoolthrs;
	uint32_t thigh;
	uint32_t t_rcoil_meas;
	uint32_t coil_induct;
	uint32_t sw_mode;
	uint32_t encmode;
	uint32_t enc_const;
	uint32_t udc_conf;
	uint32_t tudcstep;
	uint32_t otw_ov_vth;
	uint32_t virtual_stop_l;
	uint32_t virtual_stop_r;
	uint32_t x_compare;
	uint32_t x_compare_repeat;
	uint32_t r_coil_user;
	uint32_t current_pi;
	uint32_t angle_pi;
	uint32_t cur_angle_limit;
	uint32_t angle_lower_limit;
	uint32_t sgp_conf;
	uint32_t tsgp_low_vel_thrs;
	uint32_t coolplus_conf;
	uint32_t coolplus_pi_reg;
	uint32_t coolplus_pi_down;
	uint32_t coolplus_reserve;
};

/** Constant per-instance configuration, composed from devicetree at build time */
struct tmc5262_config {
	union tmc_bus bus;
	const struct tmc_bus_io *bus_io;
	uint8_t comm_type;
	const struct tmc5262_variant *variant;
	struct gpio_dt_spec sleepn_gpio;
	struct gpio_dt_spec drv_enn_gpio;
	struct gpio_dt_spec do0_gpio;
	struct gpio_dt_spec do1_gpio;
	bool ext_clock;
	uint8_t clock_divider;
	/** Initial step source: true = ramp generator (only used with TMC5262_CAP_STEP_SOURCE_SEL)
	 */
	bool ramp_step_source;
	/** Initial register image */
	struct tmc5262_shadow init;
	const struct device *stepper_driver;
	const struct device *stepper_ctrl;
};

/** Mutable per-instance state */
struct tmc5262_data {
	struct k_mutex lock;
	const struct device *dev;
	uint8_t spi_status;
	struct tmc5262_shadow shadow;
	struct gpio_callback do0_cb;
	struct gpio_callback do1_cb;
	struct k_work event_work;
	struct k_work_delayable poll_work;
	atomic_t cb_users;
	bool step_ramp_source;
	bool power_stage_on;
	bool asleep;
	bool stall_active;
	/** Encoder / TriCoder event callback (FR-FEATURE-010) and the ENC_STATUS bits reported */
	tmc5262_encoder_event_cb_t enc_cb;
	void *enc_cb_user_data;
	uint32_t enc_reported;
	/** A stop is in progress: TZEROWAIT is raised and has to be restored at standstill */
	bool stop_pending;
	uint32_t saved_tzerowait;
	/** RAMP_STAT latched events already reported to the application */
	uint32_t ev_reported;
	/** Events to dispatch from the work item (TMC5262_PEND_*) */
	atomic_t pending;
};

/** @name Core accessors used by the child devices */
/** @{ */
static inline const struct tmc5262_variant *tmc5262_get_variant_data(const struct device *dev)
{
	const struct tmc5262_config *config = dev->config;

	return config->variant;
}

/** Return 0 if the variant has all capabilities in @p caps, else -ENOTSUP (no bus access). */
int tmc5262_require(const struct device *dev, uint32_t caps);

int tmc5262_write(const struct device *dev, uint8_t reg, uint32_t val);
int tmc5262_read(const struct device *dev, uint8_t reg, uint32_t *val);

/**
 * Device level recursive lock: hold it across multi-register sequences and while changing
 * the driver state (asleep, power_stage_on, step_ramp_source, child caches).
 */
void tmc5262_lock(const struct device *dev);
void tmc5262_unlock(const struct device *dev);

/** Read ADC_VSUPPLY_TEMP once: supply voltage in mV and temperature in mdegC (either may be NULL).
 */
int tmc5262_read_adc(const struct device *dev, int32_t *mv, int32_t *mdegc);

/** Read-modify-write of a cached register. Updates the shadow copy and the device. */
int tmc5262_cfg_update(const struct device *dev, uint8_t reg, uint32_t mask, uint32_t val);

/** Write a whole cached register (update the shadow copy and the device). */
int tmc5262_cfg_set(const struct device *dev, uint8_t reg, uint32_t val);

/**
 * Return -EBUSY if the motor is moving, 0 at standstill, -EIO on bus failure. Uses
 * RAMP_STAT.vzero when the ramp generator is the step source, else DRV_STATUS.stst.
 */
int tmc5262_check_standstill(const struct device *dev);

/** true if the internal ramp generator is the active step source */
bool tmc5262_ramp_is_step_source(const struct device *dev);

/** Set *present if a driver fault is latched (GSTAT.drv_err or s2g, s2vs, ot in DRV_STATUS). */
int tmc5262_fault_present(const struct device *dev, bool *present);

/** Enable / disable the power stage (DRV_ENN or TOFF) */
int tmc5262_power_stage_enable(const struct device *dev);
int tmc5262_power_stage_disable(const struct device *dev);

/**
 * Check for an unexpected reset (GSTAT.reset / register_reset). Re-applies the cached
 * configuration and reports the event if one occurred. Call before motion commands.
 */
int tmc5262_check_reset(const struct device *dev);

/**
 * Stop a ramp generator motion: switch to positioning mode and set XTARGET = XACTUAL so
 * that DMAX/D1/D2 decelerate the motor. VMAX is never written. With @p wait the call waits
 * (bounded, derived from the velocity and deceleration) for RAMP_STAT.vzero.
 */
int tmc5262_ramp_stop(const struct device *dev, bool wait);

/** tmc5262_ramp_stop() with wait = true */
int tmc5262_stop_motion(const struct device *dev);

/** true while a stop is completing (TZEROWAIT raised), the chip lock must be held */
static inline bool tmc5262_stop_is_pending(const struct device *dev)
{
	const struct tmc5262_data *data = dev->data;

	return data->stop_pending;
}

/** Abandon a pending stop (a new motion starts): restore TZEROWAIT. */
int tmc5262_ramp_stop_cancel(const struct device *dev);

/** Event arming: callers register/unregister a callback consumer for polling */
void tmc5262_event_users_add(const struct device *dev);
void tmc5262_event_users_remove(const struct device *dev);

/** Convert a velocity in microsteps/s to VMAX / V1 register units and back. */
uint32_t tmc5262_velocity_to_reg(uint32_t microsteps_per_s);
int32_t tmc5262_velocity_from_reg(int32_t reg);
/** Convert an acceleration in microsteps/s^2 to the A1/AMAX/DMAX register units. */
int tmc5262_accel_to_reg(uint32_t microsteps_per_s2, uint32_t *reg);

/** Child device hooks (implemented in the stepper driver and controller files) */
#ifdef CONFIG_STEPPER_ADI_TMC5262_STEPPER_DRIVER
void tmc5262_stepper_driver_notify(const struct device *drv, enum stepper_event event);
#else
static inline void tmc5262_stepper_driver_notify(const struct device *drv, enum stepper_event event)
{
	ARG_UNUSED(drv);
	ARG_UNUSED(event);
}
#endif

#ifdef CONFIG_STEPPER_ADI_TMC5262_STEPPER_CTRL
void tmc5262_stepper_ctrl_notify(const struct device *ctrl, enum stepper_ctrl_event event);
/** @p lost: the register bank was reset, XACTUAL is 0; else the position is kept */
int tmc5262_stepper_ctrl_reapply(const struct device *ctrl, bool lost);
bool tmc5262_stepper_ctrl_wants_events(const struct device *ctrl);
/** true once if a stop was requested and has not been reported yet */
bool tmc5262_stepper_ctrl_consume_stop(const struct device *ctrl);
#else
static inline void tmc5262_stepper_ctrl_notify(const struct device *ctrl,
					       enum stepper_ctrl_event event)
{
	ARG_UNUSED(ctrl);
	ARG_UNUSED(event);
}

static inline int tmc5262_stepper_ctrl_reapply(const struct device *ctrl, bool lost)
{
	ARG_UNUSED(ctrl);
	ARG_UNUSED(lost);
	return 0;
}

static inline bool tmc5262_stepper_ctrl_wants_events(const struct device *ctrl)
{
	ARG_UNUSED(ctrl);
	return false;
}

static inline bool tmc5262_stepper_ctrl_consume_stop(const struct device *ctrl)
{
	ARG_UNUSED(ctrl);
	return false;
}
#endif
/** @} */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_DRIVERS_STEPPER_ADI_TMC_TMC5262_TMC5262_H_ */
