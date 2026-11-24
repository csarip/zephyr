/*
 * SPDX-FileCopyrightText: Copyright (c) 2026 Analog Devices, Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Extended API of the unified TMC5262 / TMC2262 / TMC3241 stepper driver
 *
 * The standard Zephyr stepper API (@ref stepper.h and @ref stepper_ctrl.h) covers
 * enable, disable, microstepping, motion and events. This header adds the
 * chip-level configuration and diagnostics functions that have no equivalent in
 * the generic API. All functions take the chip (parent) device, i.e. the node
 * with compatible "adi,tmc5262", "adi,tmc2262" or "adi,tmc3241".
 *
 * Every function that depends on an optional feature checks the variant
 * capability first and returns -ENOTSUP without any bus access when the feature
 * does not exist on the variant. All functions must be called from thread context.
 *
 * Common return values:
 * - -ENOTSUP : feature not available on the variant or in the selected mode
 * - -EINVAL  : parameter out of range
 * - -EBUSY   : operation conflicts with a motion in progress
 * - -EPERM   : feature combination forbidden by the datasheet
 * - -EIO     : SPI transfer failure or device fault
 * - -ENODEV  : the chip is in sleep mode (-EBUSY is only used for conflicts with a motion)
 * - -ETIMEDOUT : bounded polling loop expired
 *
 * Event callbacks (stepper_set_event_cb(), stepper_ctrl_set_event_cb()) all run in the system
 * work queue thread, never in an interrupt. Fault events also report an unexpected chip reset
 * after which the cached configuration was re-applied.
 *
 * The callbacks run in the system work queue, which is blocked while one of them runs. A
 * callback that calls stepper_disable() or tmc5262_sleep() therefore blocks the work queue for
 * the duration of the stop (derived from the velocity and deceleration, at most 10 s): request
 * the stop with stepper_ctrl_stop() instead and disable the driver from the application
 * thread after the STEPPER_CTRL_EVENT_STOPPED event.
 *
 * Stopping the ramp generator (stepper_ctrl_stop(), stepper_disable(), sleep): TZEROWAIT is
 * raised to its maximum, XTARGET is set to XACTUAL and the generator decelerates with
 * DMAX/D1/D2. At standstill XTARGET is set to XACTUAL again and TZEROWAIT is restored, so the
 * motor neither overshoots back nor moves again. STEPPER_CTRL_EVENT_STOPPED is reported at
 * that point. A new move or run cancels a stop that is still completing.
 * stepper_ctrl_set_microstep_interval() and stepper_ctrl_configure_ramp() called during that
 * time update the ramp (and the cached values) but leave TZEROWAIT at its maximum until
 * standstill, so the motor cannot reverse.
 *
 * Recovery actions (FR-ERR-004): retry on -ETIMEDOUT of the SPI bus; for clock
 * faults call tmc5262_restart_pll(); after a reset (GSTAT.reset) the driver
 * re-applies its cached configuration on the next enable or by calling
 * tmc5262_reinit(); thermal and short-circuit faults need application
 * intervention (remove the cause, tmc5262_clear_fault(), stepper_enable()).
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_STEPPER_STEPPER_TMC5262_H_
#define ZEPHYR_INCLUDE_DRIVERS_STEPPER_STEPPER_TMC5262_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/sys/util.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup stepper_tmc5262 TMC5262 / TMC2262 / TMC3241 extended API
 * @ingroup stepper_interface_ext
 * @{
 */

/** Device variant, defined by the devicetree compatible only. */
enum tmc5262_variant_id {
	/** adi,tmc5262: two-phase controller and driver */
	TMC5262_VARIANT_TMC5262 = 0,
	/** adi,tmc2262: two-phase driver (STEP/DIR only) */
	TMC5262_VARIANT_TMC2262,
	/** adi,tmc3241: three-phase controller and driver */
	TMC5262_VARIANT_TMC3241,
};

/** Bits of @ref tmc5262_drv_status.valid. A set bit means the field is meaningful. */
enum tmc5262_status_valid {
	/** Per-coil flags (ol, s2g, s2vs a/b) are valid, otherwise only the "a" member is used
	 *  and it holds the combined flag.
	 */
	TMC5262_STATUS_PER_COIL = BIT(0),
	/** @ref tmc5262_drv_status.stealth is valid (StealthChop+ variants) */
	TMC5262_STATUS_STEALTH = BIT(1),
	/** @ref tmc5262_drv_status.seq_stopped is valid */
	TMC5262_STATUS_SEQ_STOPPED = BIT(2),
	/** @ref tmc5262_drv_status.ov is valid */
	TMC5262_STATUS_OV = BIT(3),
};

/** Variant independent decoded DRV_STATUS (FR-DIAG-002). */
struct tmc5262_drv_status {
	/** @ref tmc5262_status_valid */
	uint32_t valid;
	bool standstill;
	/** Open load phase A (or combined). Informative only, may be falsely reported. */
	bool open_load_a;
	/** Open load phase B (per-coil variants only). Informative only. */
	bool open_load_b;
	/** Short to ground phase A (or combined) */
	bool short_gnd_a;
	/** Short to ground phase B (per-coil variants only) */
	bool short_gnd_b;
	/** Short to supply phase A (or combined) */
	bool short_vs_a;
	/** Short to supply phase B (per-coil variants only) */
	bool short_vs_b;
	/** Overtemperature pre-warning */
	bool otpw;
	/** Overtemperature shutdown */
	bool ot;
	/** StallGuard2/StallGuard+ stall flag */
	bool stall;
	/** StealthChop+ active (valid if TMC5262_STATUS_STEALTH) */
	bool stealth;
	/** Overvoltage */
	bool ov;
	/** Sequencer stopped by ENCA emergency stop */
	bool seq_stopped;
	/** Scaled coil current (CoolStep) */
	uint8_t cs_actual;
	/** StallGuard result (10 bit) */
	uint16_t sg_result;
};

/** Encoder readback (FR-FEATURE-010). */
struct tmc5262_encoder {
	int32_t x_enc;
	int32_t enc_latch;
	/** ENC_STATUS.n_event */
	bool n_event;
	/** ENC_STATUS.deviation_warn (TMC5262, TMC2262 only) */
	bool deviation_warn;
};

/** Decoded SPI_STATUS (FR-BUS-004). */
struct tmc5262_spi_status {
	bool standstill;
	/** StallGuard flag */
	bool sg;
	bool driver_error;
	/** GSTAT.reset */
	bool reset_flag;
	/** The four ramp generator flags below are only valid on variants with a ramp generator */
	bool ramp_valid;
	bool velocity_reached;
	bool position_reached;
	bool stop_l;
	bool stop_r;
};

/** Encoder / TriCoder events reported to a tmc5262_encoder_event_cb_t. */
enum tmc5262_encoder_event {
	/**
	 * ENC_STATUS.deviation_warn: X_ENC passed ENC_DEVIATION. With the TriCoder this is the
	 * standstill step-loss indication.
	 */
	TMC5262_ENCODER_EVENT_DEVIATION,
	/** ENC_STATUS.n_event: the encoder N channel event occurred */
	TMC5262_ENCODER_EVENT_N,
};

/**
 * @brief Callback of encoder / TriCoder events. Runs in the system work queue.
 */
typedef void (*tmc5262_encoder_event_cb_t)(const struct device *dev,
					   enum tmc5262_encoder_event event, void *user_data);

/** TriCoder (back-EMF standstill step-loss detection) readback. */
struct tmc5262_tricoder {
	/** SGP_BEMF.UBEMF_ABS, absolute back-EMF voltage measurement */
	uint16_t ubemf_abs;
	/** SGP_BEMF.SGP_RAW, raw StallGuard+ value */
	uint16_t sgp_raw;
	/** ENC_STATUS.deviation_warn: the motor was moved by an external force (step loss) */
	bool step_loss;
	/** ENC_STATUS.n_event */
	bool n_event;
};

/** Coil resistance measurement result (FR-DIAG-007), raw register units. */
struct tmc5262_rcoil {
	uint16_t a;
	uint16_t b;
};

/** @name Variant information (FR-VAR-006) @{ */

/**
 * @brief Get the variant of a chip device (derived from the devicetree compatible).
 * @param dev Chip device
 * @return the variant
 */
enum tmc5262_variant_id tmc5262_get_variant(const struct device *dev);

/**
 * @brief Get the printable variant name ("TMC5262", "TMC2262" or "TMC3241").
 * @param dev Chip device
 */
const char *tmc5262_get_variant_name(const struct device *dev);

/** @} */

/** @name Direct register access (FR-FEATURE-012) @{ */

/**
 * @brief Read any register of the variant.
 * @param dev Chip device
 * @param reg Register address
 * @param val Returned register value
 * @retval -EINVAL if the register does not exist on the variant
 */
int tmc5262_reg_read(const struct device *dev, uint8_t reg, uint32_t *val);

/**
 * @brief Write any register of the variant.
 *
 * Registers that the driver caches (GCONF, DO_CONF, DRV_CONF, IHOLD_IRUN, TPOWERDOWN,
 * CHOPCONF, COOLCONF, PWMCONF, TPWMTHRS, TCOOLTHRS, THIGH, T_RCOIL_MEAS, COIL_INDUCT, SW_MODE,
 * ENCMODE, ENC_CONST, UDC_CONF, TUDCSTEP, OTW_OV_VTH and the virtual stops) are written through
 * the cache, so the value is restored after a reset or wake-up. All other registers are written
 * directly and are lost on reset.
 *
 * RAMP_STAT: the position reached event is cleared by the driver when it is reported. The stop
 * (event_stop_l, event_stop_r) and stall (event_stop_sg) events stay latched in RAMP_STAT until
 * the application clears them by writing 1 to the bit, for example with this function.
 * They are reported once per occurrence. event_stop_sg blocks a restart until it is cleared.
 *
 * @param dev Chip device
 * @param reg Register address
 * @param val Value
 * @retval -EINVAL if the register does not exist on the variant
 */
int tmc5262_reg_write(const struct device *dev, uint8_t reg, uint32_t val);

/** @} */

/** @name Status and diagnostics (FR-DIAG-*) @{ */

/** Read GSTAT. */
int tmc5262_get_gstat(const struct device *dev, uint32_t *gstat);

/** Clear selected GSTAT flags (writes 1 to the given flags, see TMC5262_GSTAT_*). */
int tmc5262_clear_gstat(const struct device *dev, uint32_t flags);

/** Read and decode DRV_STATUS using the variant layout. */
int tmc5262_get_drv_status(const struct device *dev, struct tmc5262_drv_status *status);

/**
 * @brief Most recent SPI_STATUS byte captured from any transfer (FR-BUS-004).
 *
 * Bits 3..0: standstill, sg, driver_error, reset_flag. Bits 7..4 (velocity_reached,
 * position_reached, status_stop_l, status_stop_r) are only meaningful on variants
 * with a ramp generator and are cleared on the TMC2262.
 */
int tmc5262_get_spi_status(const struct device *dev, uint8_t *spi_status);

/**
 * @brief Read the most recent SPI_STATUS byte decoded into a structure (FR-BUS-004).
 */
int tmc5262_get_spi_status_decoded(const struct device *dev, struct tmc5262_spi_status *st);

/**
 * @brief Clear a latched fault so that the driver can be enabled again.
 *
 * Cycles the power stage (DRV_ENN released and TOFF = 0 for a moment, as the short and
 * overtemperature flags only clear that way) and clears GSTAT.drv_err (it can only clear
 * once the cause is gone). The power stage remains disabled until stepper_enable() is
 * called; stepper_enable() calls this itself when a fault is latched.
 *
 * @retval -EIO if the fault is still present
 */
int tmc5262_clear_fault(const struct device *dev);

/** Supply voltage in millivolts, from ADC_VSUPPLY (VS = 0.1409 V x ADC_VSUPPLY). */
int tmc5262_get_supply_mv(const struct device *dev, int32_t *mv);

/** Chip temperature in millidegrees Celsius (T = 1.042 x ADC_TEMP - 264.6 degC). */
int tmc5262_get_temperature_mdegc(const struct device *dev, int32_t *mdegc);

/**
 * @brief Set the overtemperature pre-warning and overvoltage ADC thresholds (OTW_OV_VTH).
 * @param otw_adc 9-bit temperature ADC threshold
 * @param ov_adc 9-bit supply ADC threshold
 */
int tmc5262_set_otw_ov_thresholds(const struct device *dev, uint32_t otw_adc, uint32_t ov_adc);

#if defined(CONFIG_STEPPER_ADI_TMC5262_PLUS) || defined(__DOXYGEN__)
/**
 * @brief Read the momentary coil currents (raw 12 bit signed ADC codes).
 * @note TMC5262 and TMC2262 only.
 */
int tmc5262_get_coil_current_raw(const struct device *dev, int16_t *coil_a, int16_t *coil_b);
#endif

/**
 * @brief Measured step frequency, as TSTEP converted with the system clock.
 *
 * Returns fCLK / TSTEP in Hz. TSTEP is the time between two 1/256 microsteps (TMC5262,
 * TMC2262) or between two 1/171 microsteps of the electrical revolution (TMC3241,
 * about 171 per full step), so the result is the step frequency expressed in that
 * reference resolution, not in the configured microstep resolution. Returns 0 on overflow
 * (TSTEP = 2^20 - 1) or standstill.
 */
int tmc5262_get_step_frequency(const struct device *dev, uint32_t *hz);

#if defined(CONFIG_STEPPER_ADI_TMC5262_RAMP_GEN) || defined(__DOXYGEN__)
/**
 * @brief Actual ramp generator velocity in microsteps per second (signed).
 * @note Variants with ramp generator only.
 */
int tmc5262_get_actual_velocity(const struct device *dev, int32_t *microsteps_per_s);
#endif

/** Read STEPS_LOST (FR-FEATURE-009). */
int tmc5262_get_steps_lost(const struct device *dev, uint32_t *steps_lost);

/** Read the sequencer position MSCNT and the coil currents MSCURACT (FR-MOTION-004). */
int tmc5262_get_sequencer(const struct device *dev, uint16_t *mscnt, int16_t *cur_a,
			  int16_t *cur_b);

/** @} */

/** @name Current and chopper configuration (FR-CONFIG-*) @{ */

/**
 * @brief Select the current range index (0..3) and scale (25, 50, 75, 100 percent).
 *
 * The range maps to the variant table, full scale 1.5, 3, 4.5 and 6 A peak
 * (1.06, 2.12, 3.18 and 4.24 A RMS) on all variants; see tmc5262_get_current_range().
 *
 * @retval -EINVAL if scale is not 100 percent while the range is not the lowest
 * @retval -EBUSY if the motor is moving
 */
int tmc5262_set_current_range(const struct device *dev, uint32_t range, uint32_t scale_percent);

/**
 * @brief Get the full scale current of the configured range and scale in mA.
 *
 * TMC5262 and TMC3241 return mA RMS (1060, 2120, 3180, 4240 at 100 percent), the TMC2262
 * returns mA peak (1500, 3000, 4500, 6000).
 *
 * @param[out] full_scale_ma full scale current in mA
 * @param[out] is_peak true if the value is a peak value
 */
int tmc5262_get_current_range(const struct device *dev, uint32_t *full_scale_ma, bool *is_peak);

/** Set IRUN (0..255). */
int tmc5262_set_irun(const struct device *dev, uint32_t irun);
/** Set IHOLD (0..255). */
int tmc5262_set_ihold(const struct device *dev, uint32_t ihold);
/** Set IHOLDDELAY (0..255). */
int tmc5262_set_iholddelay(const struct device *dev, uint32_t delay);
/** Set IRUNDELAY (0..15). */
int tmc5262_set_irundelay(const struct device *dev, uint32_t delay);
/** Set TPOWERDOWN (0..255). */
int tmc5262_set_tpowerdown(const struct device *dev, uint32_t tpowerdown);

/**
 * @brief Invert the motor direction (GCONF.shaft).
 * @retval -ENOTSUP if the ramp generator is the step source
 */
int tmc5262_set_direction_inverted(const struct device *dev, bool inverted);

/**
 * @brief Select the step source (FR-CONFIG-007).
 * @param ramp_generator true for the internal ramp generator, false for STEP/DIR
 * @note TMC5262 and TMC3241. On the TMC2262 the step source is fixed to STEP/DIR.
 * @retval -ENOTSUP TMC2262, or ramp_generator requested on a variant without one
 * @retval -EBUSY if the motor is moving
 */
int tmc5262_set_step_source(const struct device *dev, bool ramp_generator);

#if defined(CONFIG_STEPPER_ADI_TMC5262_PLUS) || defined(__DOXYGEN__)
/**
 * @brief Select StealthChop+ (true) or SpreadCycle (false) (GCONF.en_stealthchop).
 * @note TMC5262 and TMC2262. Only changes at standstill.
 * @retval -EBUSY if the motor is moving
 */
int tmc5262_set_stealthchop(const struct device *dev, bool enable);
#endif

#if defined(CONFIG_STEPPER_ADI_TMC5262_PLUS) || defined(__DOXYGEN__)
/**
 * @brief Set the StealthChop+ upper velocity threshold TPWMTHRS (20 bit, in TSTEP units).
 * @note TMC5262 and TMC2262.
 */
int tmc5262_set_tpwmthrs(const struct device *dev, uint32_t tpwmthrs);
#endif

/** Set the CoolStep / StallGuard lower velocity threshold TCOOLTHRS (20 bit). */
int tmc5262_set_tcoolthrs(const struct device *dev, uint32_t tcoolthrs);

/** Set the high velocity threshold THIGH (20 bit). */
int tmc5262_set_thigh(const struct device *dev, uint32_t thigh);

/**
 * @brief Configure StallGuard2 (COOLCONF.sgt, sfilt).
 * @param sgt threshold, -64..63
 * @param sfilt enable the StallGuard2 filter
 */
int tmc5262_set_stallguard2(const struct device *dev, int32_t sgt, bool sfilt);

#if defined(CONFIG_STEPPER_ADI_TMC5262_RAMP_GEN) || defined(__DOXYGEN__)
/**
 * @brief Enable stall-triggered stop (SW_MODE.sg_stop).
 * @note Variants with ramp generator.
 */
int tmc5262_set_stallguard_stop(const struct device *dev, bool enable);
#endif

/**
 * @brief Configure CoolStep (COOLCONF fields semin, seup, semax, sedn, seimin).
 *
 * semin = 0 disables CoolStep.
 */
int tmc5262_set_coolstep(const struct device *dev, uint32_t semin, uint32_t seup, uint32_t semax,
			 uint32_t sedn, bool seimin);

#if defined(CONFIG_STEPPER_ADI_TMC5262_PLUS) || defined(__DOXYGEN__)
/**
 * @brief Configure uDcStep (FR-FEATURE-006).
 * @retval -EINVAL if decel_thrs is not greater than accel_thrs
 * @retval -EPERM if enabling while StealthChop+ is not active
 * @note TMC5262 and TMC2262.
 */
int tmc5262_set_udcstep(const struct device *dev, bool enable, uint32_t accel_thrs,
			uint32_t decel_thrs, uint32_t tudcstep);
#endif

/** @} */

/** @name Ramp generator extras (FR-MOTION-014, FR-MOTION-015, FR-FEATURE-011) @{ */

#if defined(CONFIG_STEPPER_ADI_TMC5262_RAMP_GEN) || defined(__DOXYGEN__)
/**
 * @brief Configure reference switches and virtual stops.
 * @param sw_mode full SW_MODE value, see TMC5262_SW_MODE_* in the driver header documentation
 * @param virtual_stop_l left virtual stop position
 * @param virtual_stop_r right virtual stop position
 * @note Variants with reference switches (TMC5262, TMC3241).
 */
int tmc5262_set_switch_config(const struct device *dev, uint32_t sw_mode, int32_t virtual_stop_l,
			      int32_t virtual_stop_r);
#endif

#if defined(CONFIG_STEPPER_ADI_TMC5262_RAMP_GEN) || defined(__DOXYGEN__)
/** Read the position latched on a reference switch event (XLATCH). TMC5262, TMC3241. */
int tmc5262_get_xlatch(const struct device *dev, int32_t *xlatch);
#endif

#if defined(CONFIG_STEPPER_ADI_TMC5262_RAMP_GEN) || defined(__DOXYGEN__)
/** Configure the position compare registers. TMC5262, TMC3241. */
int tmc5262_set_position_compare(const struct device *dev, int32_t x_compare, uint32_t repeat);
#endif

#if defined(CONFIG_STEPPER_ADI_TMC5262_RAMP_GEN) || defined(__DOXYGEN__)
/**
 * @brief Enter or leave direct current mode.
 *
 * In direct current mode the signed coil currents are written through XTARGET
 * (coils A and B on the TMC5262, coils U and V on the TMC3241). Velocity based
 * regulation is not available in this mode.
 *
 * Direct mode is entered and left at standstill only (-EBUSY otherwise). The ramp generator is
 * put in hold mode first; on leaving, XTARGET is set to XACTUAL so that no motion starts.
 *
 * @param enable true to enable direct mode
 * @param coil_1 signed 9-bit current of coil A (U)
 * @param coil_2 signed 9-bit current of coil B (V)
 * @note TMC5262, TMC3241
 */
int tmc5262_set_direct_current(const struct device *dev, bool enable, int32_t coil_1,
			       int32_t coil_2);
#endif

#if defined(CONFIG_STEPPER_ADI_TMC5262_RAMP_GEN) || defined(__DOXYGEN__)
/** Abort motion: VMAX = 0 and RAMPMODE hold (FR-MOTION-011). Variants with ramp generator. */
int tmc5262_abort(const struct device *dev);
#endif

/** @} */

/** @name Encoder (FR-FEATURE-010) @{ */

/**
 * @brief Configure the incremental encoder.
 * @param encmode ENCMODE value (variant layout, unsupported bits are rejected with -EINVAL)
 * @param enc_const ENC_CONST value
 * @retval -EPERM if ENCN is configured as overvoltage output and the N channel is used
 */
int tmc5262_encoder_configure(const struct device *dev, uint32_t encmode, uint32_t enc_const);

/** Read X_ENC, ENC_LATCH and ENC_STATUS. */
int tmc5262_encoder_read(const struct device *dev, struct tmc5262_encoder *enc);

/**
 * @brief Register a callback for encoder events (deviation warning, N event).
 *
 * The events are detected from ENC_STATUS by the event work item (DO interrupt or polling) and
 * reported once per occurrence. The datasheet gives no dedicated TriCoder event: the deviation
 * warning is the step-loss indication. Pass NULL to unregister.
 */
int tmc5262_set_encoder_event_cb(const struct device *dev, tmc5262_encoder_event_cb_t cb,
				 void *user_data);

#if defined(CONFIG_STEPPER_ADI_TMC5262_PLUS) || defined(__DOXYGEN__)
/** Read ENC_DEVIATION. TMC5262 and TMC2262. */
int tmc5262_get_encoder_deviation(const struct device *dev, uint32_t *deviation);
#endif

/** @} */

/** @name StealthChop+, coil measurement and TriCoder (TMC5262, TMC2262) @{ */

#if defined(CONFIG_STEPPER_ADI_TMC5262_PLUS) || defined(__DOXYGEN__)
/** Configure PWMCONF. */
int tmc5262_set_pwmconf(const struct device *dev, uint32_t pwmconf);
#endif

#if defined(CONFIG_STEPPER_ADI_TMC5262_PLUS) || defined(__DOXYGEN__)
/** Write the StealthChop+ regulator registers (CURRENT_PI_REG, ANGLE_PI_REG, CUR_ANGLE_LIMIT,
 *  ANGLE_LOWER_LIMIT).
 */
int tmc5262_set_stealthchop_regulators(const struct device *dev, uint32_t current_pi,
				       uint32_t angle_pi, uint32_t cur_angle_limit,
				       uint32_t angle_lower_limit);
#endif

#if defined(CONFIG_STEPPER_ADI_TMC5262_PLUS) || defined(__DOXYGEN__)
/** Read CUR_ANGLE_MEAS and PI_RESULTS for tuning. */
int tmc5262_get_stealthchop_measurements(const struct device *dev, uint32_t *cur_angle_meas,
					 uint32_t *pi_results);
#endif

#if defined(CONFIG_STEPPER_ADI_TMC5262_PLUS) || defined(__DOXYGEN__)
/** Configure StallGuard+ (SGP_CONF and TSGP_LOW_VEL_THRS). */
int tmc5262_set_stallguard_plus(const struct device *dev, uint32_t sgp_conf, uint32_t low_vel_thrs);
#endif

#if defined(CONFIG_STEPPER_ADI_TMC5262_PLUS) || defined(__DOXYGEN__)
/** Configure CoolStep+ registers (0x4E to 0x51). */
int tmc5262_set_coolstep_plus(const struct device *dev, uint32_t conf, uint32_t pi_reg,
			      uint32_t pi_down, uint32_t reserve_conf);
#endif

#if defined(CONFIG_STEPPER_ADI_TMC5262_PLUS) || defined(__DOXYGEN__)
/** Read COOLSTEPPLUS_LOAD_RESERVE (0x52). */
int tmc5262_get_coolstep_plus_load_reserve(const struct device *dev, uint32_t *load_reserve);
#endif

#if defined(CONFIG_STEPPER_ADI_TMC5262_PLUS) || defined(__DOXYGEN__)
/**
 * @brief Run a coil resistance measurement setup and read the result.
 *
 * Configures T_RCOIL_MEAS, performs the read of R_COIL and returns it. Measurements are
 * only valid at standstill.
 *
 * @param t_rcoil_meas T_RCOIL_MEAS value (20 bit)
 * @retval -EBUSY if the motor is moving
 */
int tmc5262_measure_rcoil(const struct device *dev, uint32_t t_rcoil_meas,
			  struct tmc5262_rcoil *rcoil);
#endif

#if defined(CONFIG_STEPPER_ADI_TMC5262_PLUS) || defined(__DOXYGEN__)
/** Program user coil resistances (R_COIL_USER) and select them (manual mode). */
int tmc5262_set_rcoil_user(const struct device *dev, uint32_t rcoil_a, uint32_t rcoil_b);
#endif

#if defined(CONFIG_STEPPER_ADI_TMC5262_PLUS) || defined(__DOXYGEN__)
/**
 * @brief Read the TriCoder back-EMF measurement (SGP_BEMF) and the step-loss status.
 * @note TMC5262 and TMC2262.
 */
int tmc5262_get_tricoder(const struct device *dev, struct tmc5262_tricoder *tc);
#endif

#if defined(CONFIG_STEPPER_ADI_TMC5262_PLUS) || defined(__DOXYGEN__)
/** Enable the TriCoder back-EMF fields of ENCMODE. TMC5262, TMC2262. */
int tmc5262_set_tricoder(const struct device *dev, bool enable, uint32_t bemf_encmode_bits);
#endif

/** @} */

/** @name Power management and recovery (FR-POWER-*, FR-ERR-004) @{ */

/**
 * @brief Enter sleep mode (SLEEPN low).
 *
 * Stops motion, disables the power stage and then drives SLEEPN low.
 *
 * @retval -ENOTSUP if no SLEEPN GPIO is provided
 */
int tmc5262_sleep(const struct device *dev);

/**
 * @brief Leave sleep mode: release SLEEPN, wait tWAKE, restart the PLL and re-apply the
 *        cached configuration. The power stage stays disabled.
 */
int tmc5262_wake(const struct device *dev);

/** Restart the PLL (clears clk_fsm_ena, then the standard start-up sequence). */
int tmc5262_restart_pll(const struct device *dev);

/**
 * @brief Re-apply the cached configuration (after GSTAT.reset). The power stage stays
 *        disabled.
 */
int tmc5262_reinit(const struct device *dev);

/** @} */

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_STEPPER_STEPPER_TMC5262_H_ */
