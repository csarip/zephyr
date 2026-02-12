/*
 * SPDX-FileCopyrightText: Copyright (c) 2026 Cherrence Sarip <Cherrence.sarip@analog.com>
 * SPDX-License-Identifier: Apache-2.0
 *
 * Driver for TMC5221 (SPI) and TMC5222 (I2C) stepper motor controllers.
 * Both devices share the same register map and differ only in the bus interface.
 */

#define DT_DRV_COMPAT adi_tmc522x

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <adi_tmc_reg.h>

#include "tmc522x.h"

LOG_MODULE_REGISTER(tmc522x, CONFIG_STEPPER_LOG_LEVEL);

/* Helper macros for bit field manipulation */
#define TMC522X_FIELD_PREP(mask, val) (((val) << (__builtin_ctz(mask))) & (mask))
#define TMC522X_FIELD_GET(mask, reg)  (((reg) & (mask)) >> (__builtin_ctz(mask)))

/* Low-level register write function following TMC driver pattern */
int tmc522x_write(const struct device *dev, const uint8_t reg_addr, const uint32_t reg_val)
{
	const struct tmc522x_config *config = dev->config;
	struct tmc522x_data *data = dev->data;
	int err;

	k_sem_take(&data->sem, K_FOREVER);

	err = config->bus_io->write(&config->bus, reg_addr, reg_val);

	k_sem_give(&data->sem);

	if (err < 0) {
		LOG_ERR("Failed to write register 0x%x with value 0x%x", reg_addr, reg_val);
		return err;
	}

	LOG_DBG("WRITE reg 0x%02X <= 0x%08X", reg_addr, reg_val);
	return 0;
}

/* Low-level register read function following TMC driver pattern */
int tmc522x_read(const struct device *dev, const uint8_t reg_addr, uint32_t *reg_val)
{
	const struct tmc522x_config *config = dev->config;
	struct tmc522x_data *data = dev->data;
	int err;

	k_sem_take(&data->sem, K_FOREVER);

	err = config->bus_io->read(&config->bus, reg_addr, reg_val);

	k_sem_give(&data->sem);

	if (err < 0) {
		LOG_ERR("Failed to read register 0x%x", reg_addr);
		return err;
	}

	LOG_DBG("READ reg 0x%02X => 0x%08X", reg_addr, *reg_val);
	return 0;
}

/* Forward declarations */
int tmc522x_clear_stall(const struct device *dev);
static int tmc522x_clear_pos_reached(const struct device *dev);

/*
 * Motion control functions - used by stepper API wrapper functions
 * Following the pattern used by TMC51xx and TMC50xx drivers:
 * - Motion parameters (VMAX, AMAX, etc.) are written once during initialization
 * - Only RAMPMODE and XTARGET are written during position moves
 */
static int tmc522x_set_position(const struct device *dev, int32_t position)
{
	struct tmc522x_data *data = dev->data;
	int ret;

	ret = tmc522x_clear_stall(dev);
	if (ret) {
		return ret;
	}

	/* Restore VMAX to configured value (in case it was set to 0 by stop)
	 * Position mode needs VMAX to accelerate to the target position.
	 */
	ret = tmc522x_write(dev, TMC522X_REG_VMAX, data->vmax);
	if (ret) {
		LOG_ERR("Failed to restore VMAX: %d", ret);
		return ret;
	}

	/* Set to POSITION mode */
	ret = tmc522x_write(dev, TMC522X_REG_RAMPMODE, TMC522X_RAMPMODE_POSITION);
	if (ret) {
		return ret;
	}

	/* Write the TARGET position - this starts the movement */
	ret = tmc522x_write(dev, TMC522X_REG_XTARGET, position);
	if (ret) {
		return ret;
	}

	/* Cache the target position */
	data->xtarget = position;

	return 0;
}

/* Event clearing functions */
int tmc522x_clear_stall(const struct device *dev)
{
	/* Write-1-to-clear the stall event bit
	 * For W1C (write-1-to-clear) bits, write only the bit to clear
	 */
	return tmc522x_write(dev, TMC522X_REG_RAMP_STAT, TMC522X_EVENT_STOP_SG_MASK);
}

__maybe_unused
static int tmc522x_clear_pos_reached(const struct device *dev)
{
	/* Write-1-to-clear the position reached event bit
	 * For W1C (write-1-to-clear) bits, write only the bit to clear
	 */
	return tmc522x_write(dev, TMC522X_REG_RAMP_STAT, TMC522X_EVENT_POS_REACH_MASK);
}

/* Forward declarations - implementations are in API section below */
int tmc522x_set_velocity(const struct device *dev, int32_t velocity);
int tmc522x_set_stealthchop_threshold(const struct device *dev, uint32_t velocity_pps);
int tmc522x_set_coolstep_threshold(const struct device *dev, uint32_t velocity_pps);
int tmc522x_set_fullstep_threshold(const struct device *dev, uint32_t velocity_pps);

/* Enable/disable motor via GPIO */
__maybe_unused
static int tmc522x_set_enable(const struct device *dev, bool enable)
{
	const struct tmc522x_config *cfg = dev->config;
	struct tmc522x_data *data = dev->data;
	int ret = 0;

	if (enable) {
		/* Enable driver outputs via DRVEN GPIO */
		if (cfg->has_drven) {
			ret = gpio_pin_set_dt(&cfg->drven_gpio, 1);
			if (ret) {
				LOG_ERR("Failed to enable driver outputs");
				return ret;
			}
			k_msleep(10);
		}

		/* Set position mode and target */
		ret = tmc522x_set_position(dev, data->xtarget);
		if (ret) {
			return ret;
		}

		data->enabled = true;
		LOG_INF("Motor enabled");
	} else {
		/* Disable driver outputs via DRVEN GPIO */
		if (cfg->has_drven) {
			ret = gpio_pin_set_dt(&cfg->drven_gpio, 0);
			if (ret) {
				LOG_ERR("Failed to disable driver outputs");
				return ret;
			}
		}

		data->enabled = false;
		LOG_INF("Motor disabled");
	}

	return 0;
}

/* Read and parse DRV_STATUS register for diagnostics */
__maybe_unused
static int tmc522x_read_drv_status(const struct device *dev, uint32_t *status)
{
	int ret;

	ret = tmc522x_read(dev, TMC522X_REG_DRV_STATUS, status);
	if (ret) {
		LOG_ERR("Failed to read DRV_STATUS register");
		return ret;
	}

	/* Log diagnostic information */
	LOG_DBG("DRV_STATUS: 0x%08X", *status);
	LOG_DBG("  Standstill: %d", !!(*status & TMC522X_DRV_STATUS_STST));
	LOG_DBG("  Open Load B: %d", !!(*status & TMC522X_DRV_STATUS_OLB));
	LOG_DBG("  Open Load A: %d", !!(*status & TMC522X_DRV_STATUS_OLA));
	LOG_DBG("  Short to Ground B: %d", !!(*status & TMC522X_DRV_STATUS_S2GB));
	LOG_DBG("  Short to Ground A: %d", !!(*status & TMC522X_DRV_STATUS_S2GA));
	LOG_DBG("  Overtemp Warning: %d", !!(*status & TMC522X_DRV_STATUS_OTPW));
	LOG_DBG("  Overtemp: %d", !!(*status & TMC522X_DRV_STATUS_OT));
	LOG_DBG("  StallGuard: %d", !!(*status & TMC522X_DRV_STATUS_STALLGUARD));
	LOG_DBG("  Fullstep Active: %d", !!(*status & TMC522X_DRV_STATUS_FSACTIVE));
	LOG_DBG("  StealthChop Active: %d", !!(*status & TMC522X_DRV_STATUS_STEALTHCHOP));
	LOG_DBG("  Short to Supply B: %d", !!(*status & TMC522X_DRV_STATUS_S2VSB));
	LOG_DBG("  Short to Supply A: %d", !!(*status & TMC522X_DRV_STATUS_S2VSA));

	return 0;
}

/* Check for error conditions in DRV_STATUS */
__maybe_unused
static int tmc522x_check_errors(const struct device *dev)
{
	uint32_t status;
	int ret;

	ret = tmc522x_read(dev, TMC522X_REG_DRV_STATUS, &status);
	if (ret) {
		return ret;
	}

	/* Check for critical errors */
	if (status & TMC522X_DRV_STATUS_OT) {
		LOG_ERR("CRITICAL: Overtemperature shutdown!");
		return -EIO;
	}

	if (status & TMC522X_DRV_STATUS_OTPW) {
		LOG_WRN("WARNING: Overtemperature pre-warning");
	}

	if (status & TMC522X_DRV_STATUS_S2GA) {
		LOG_ERR("ERROR: Short to ground on phase A");
		return -EIO;
	}

	if (status & TMC522X_DRV_STATUS_S2GB) {
		LOG_ERR("ERROR: Short to ground on phase B");
		return -EIO;
	}

	if (status & TMC522X_DRV_STATUS_S2VSA) {
		LOG_ERR("ERROR: Short to supply on phase A");
		return -EIO;
	}

	if (status & TMC522X_DRV_STATUS_S2VSB) {
		LOG_ERR("ERROR: Short to supply on phase B");
		return -EIO;
	}

	if (status & TMC522X_DRV_STATUS_OLA) {
		LOG_WRN("WARNING: Open load on phase A");
	}

	if (status & TMC522X_DRV_STATUS_OLB) {
		LOG_WRN("WARNING: Open load on phase B");
	}

	return 0;
}

/* Read and display general status (GSTAT register) */
__maybe_unused
static int tmc522x_read_general_status(const struct device *dev, uint32_t *status)
{
	int ret;

	ret = tmc522x_read(dev, TMC522X_REG_GSTAT, status);
	if (ret) {
		LOG_ERR("Failed to read GSTAT register");
		return ret;
	}

	LOG_DBG("GSTAT: 0x%08X", *status);
	return 0;
}

/*
 * Write motion parameters to TMC522X registers
 * This should be called once during initialization to configure the ramp generator.
 * Following the pattern used by TMC51xx and TMC50xx drivers.
 */
static int tmc522x_write_motion_params(const struct device *dev)
{
	struct tmc522x_data *data = dev->data;
	int ret;

	LOG_DBG("Writing motion parameters to TMC522X");

	ret = tmc522x_write(dev, TMC522X_REG_VSTART, data->vstart);
	if (ret) {
		LOG_ERR("Failed to write VSTART");
		return ret;
	}

	ret = tmc522x_write(dev, TMC522X_REG_A1, data->a1);
	if (ret) {
		LOG_ERR("Failed to write A1");
		return ret;
	}

	ret = tmc522x_write(dev, TMC522X_REG_V1, data->v1);
	if (ret) {
		LOG_ERR("Failed to write V1");
		return ret;
	}

	ret = tmc522x_write(dev, TMC522X_REG_A2, data->a2);
	if (ret) {
		LOG_ERR("Failed to write A2");
		return ret;
	}

	ret = tmc522x_write(dev, TMC522X_REG_V2, data->v2);
	if (ret) {
		LOG_ERR("Failed to write V2");
		return ret;
	}

	ret = tmc522x_write(dev, TMC522X_REG_AMAX, data->amax);
	if (ret) {
		LOG_ERR("Failed to write AMAX");
		return ret;
	}

	ret = tmc522x_write(dev, TMC522X_REG_VMAX, data->vmax);
	if (ret) {
		LOG_ERR("Failed to write VMAX");
		return ret;
	}

	ret = tmc522x_write(dev, TMC522X_REG_DMAX, data->dmax);
	if (ret) {
		LOG_ERR("Failed to write DMAX");
		return ret;
	}

	ret = tmc522x_write(dev, TMC522X_REG_D2, data->d2);
	if (ret) {
		LOG_ERR("Failed to write D2");
		return ret;
	}

	ret = tmc522x_write(dev, TMC522X_REG_D1, data->d1);
	if (ret) {
		LOG_ERR("Failed to write D1");
		return ret;
	}

	ret = tmc522x_write(dev, TMC522X_REG_VSTOP, data->vstop);
	if (ret) {
		LOG_ERR("Failed to write VSTOP");
		return ret;
	}

	/* Write TCOOLTHRS - StallGuard/CoolStep velocity threshold
	 * NOTE: TCOOLTHRS behavior is complex and chip-specific.
	 * For TMC5221 with StealthChop:
	 *   TCOOLTHRS = 0: StallGuard works, good sensitivity
	 *   TCOOLTHRS = high: May change sensitivity/behavior
	 *
	 * Setting to 0 for maximum compatibility and known-good behavior.
	 */
	ret = tmc522x_write(dev, TMC522X_REG_TCOOLTHRS, 0);
	if (ret) {
		LOG_ERR("Failed to write TCOOLTHRS");
		return ret;
	}
	data->tcoolthrs = 0;
	LOG_INF("TCOOLTHRS set to 0x%X for StallGuard detection", data->tcoolthrs);

	LOG_INF("Motion parameters initialized (VMAX=%u, AMAX=%u, VSTART=%u, VSTOP=%u)",
		data->vmax, data->amax, data->vstart, data->vstop);
	return 0;
}

/*
 * Configure TMC522X hardware registers from devicetree settings
 * This should be called after basic initialization to set up chopper,
 * current control, PWM, and other hardware parameters.
 */
__maybe_unused
static int tmc522x_configure_hardware(const struct device *dev)
{
	struct tmc522x_data *data = dev->data;
	const struct tmc522x_config *cfg = dev->config;
	uint32_t val;
	int ret;

	LOG_DBG("Configuring TMC522X hardware registers");

	/* Configure DRVCONF - FS Sense & FS Gain */
	val = TMC522X_FIELD_PREP(TMC522X_DRVCONF_FS_SENSE_MASK, data->fs_sense) |
	      TMC522X_FIELD_PREP(TMC522X_DRVCONF_FS_GAIN_MASK, data->fs_gain);
	ret = tmc522x_write(dev, TMC522X_REG_DRVCONF, val);
	if (ret) {
		LOG_ERR("Failed to write DRVCONF");
		return ret;
	}

	/* Configure GLOBALSCALER - separate scalers for coils A and B */
	val = TMC522X_FIELD_PREP(TMC522X_GLOBALSCALER_A_MASK, data->global_scaler_a) |
	      TMC522X_FIELD_PREP(TMC522X_GLOBALSCALER_B_MASK, data->global_scaler_b);
	ret = tmc522x_write(dev, TMC522X_REG_GLOBALSCALER, val);
	if (ret) {
		LOG_ERR("Failed to write GLOBALSCALER");
		return ret;
	}

	/* Configure IHOLD_IRUN - current control */
	val = (data->irundelay << 24) |
	      (data->iholddelay << 16) |
	      (data->current_run << 8) |
	      (data->current_hold);

	LOG_INF("Writing IHOLD_IRUN: 0x%08X (ihold=%d, irun=%d, iholddelay=%d, irundelay=%d)",
	        val, data->current_hold, data->current_run, data->iholddelay, data->irundelay);

	ret = tmc522x_write(dev, TMC522X_REG_IHOLD_IRUN, val);
	if (ret) {
		LOG_ERR("Failed to write IHOLD_IRUN");
		return ret;
	}

	/* Read back to verify */
	uint32_t ihold_irun_readback;
	ret = tmc522x_read(dev, TMC522X_REG_IHOLD_IRUN, &ihold_irun_readback);
	if (ret == 0) {
		LOG_INF("IHOLD_IRUN readback: 0x%08X", ihold_irun_readback);
	}

	/* Configure TPOWERDOWN - delay before standby current reduction */
	ret = tmc522x_write(dev, TMC522X_REG_TPOWERDOWN, data->tpowerdown);
	if (ret) {
		LOG_ERR("Failed to write TPOWERDOWN");
		return ret;
	}

	/* Configure CHOPCONF - chopper configuration */
	ret = tmc522x_read(dev, TMC522X_REG_CHOPCONF, &val);
	if (ret) {
		LOG_ERR("Failed to read CHOPCONF");
		return ret;
	}

	/* Set TOFF (off time) */
	val &= ~TMC522X_CHOPCONF_TOFF_MASK;
	val |= TMC522X_FIELD_PREP(TMC522X_CHOPCONF_TOFF_MASK, data->toff);

	/* Set TBL (blanking time) */
	val &= ~TMC522X_CHOPCONF_TBL_MASK;
	val |= TMC522X_FIELD_PREP(TMC522X_CHOPCONF_TBL_MASK, data->tbl);

	/* Set MRES (microstep resolution) */
	val &= ~TMC522X_CHOPCONF_MRES_MASK;
	val |= TMC522X_FIELD_PREP(TMC522X_CHOPCONF_MRES_MASK, data->microstep_res);

	/* Set HSTRT and HEND (hysteresis) */
	val &= ~TMC522X_CHOPCONF_HSTRT_TFD210_MASK;
	val |= TMC522X_FIELD_PREP(TMC522X_CHOPCONF_HSTRT_TFD210_MASK, data->hstrt_tfd210);

	val &= ~TMC522X_CHOPCONF_HEND_OFFSET_MASK;
	val |= TMC522X_FIELD_PREP(TMC522X_CHOPCONF_HEND_OFFSET_MASK, data->hend_offset);

	/* Enable software driver control (DRV_EN_SW) - CRITICAL for motor enable! */
	/* This bit must be set to allow DRVEN GPIO to control the driver outputs */
	/* Without this, the driver stage will not enable regardless of GPIO state */
	val |= TMC522X_CHOPCONF_DRV_EN_SW_MASK;

	/* Set VHIGHCHM and VHIGHFS if configured */
	if (data->vhighchm) {
		val |= TMC522X_CHOPCONF_VHIGHCHM_MASK;
	} else {
		val &= ~TMC522X_CHOPCONF_VHIGHCHM_MASK;
	}

	if (data->vhighfs) {
		val |= TMC522X_CHOPCONF_VHIGHFS_MASK;
	} else {
		val &= ~TMC522X_CHOPCONF_VHIGHFS_MASK;
	}

	ret = tmc522x_write(dev, TMC522X_REG_CHOPCONF, val);
	if (ret) {
		LOG_ERR("Failed to write CHOPCONF");
		return ret;
	}

	/* Configure PWMCONF - PWM configuration */
	ret = tmc522x_read(dev, TMC522X_REG_PWMCONF, &val);
	if (ret) {
		LOG_ERR("Failed to read PWMCONF");
		return ret;
	}

	/* Set PWM freewheel mode */
	val &= ~TMC522X_PWMCONF_FREEWHEEL_MASK;
	val |= TMC522X_FIELD_PREP(TMC522X_PWMCONF_FREEWHEEL_MASK, data->pwm_freewheel);

	/* Set PWM frequency */
	val &= ~TMC522X_PWMCONF_FREQ_MASK;
	val |= TMC522X_FIELD_PREP(TMC522X_PWMCONF_FREQ_MASK, data->pwm_freq);

	/* Set PWM_DIS_REG_STST (disable regulation in standstill) */
	if (data->pwm_dis_reg_stat) {
		val |= TMC522X_PWMCONF_PWM_DIS_REG_STST_MASK;
	} else {
		val &= ~TMC522X_PWMCONF_PWM_DIS_REG_STST_MASK;
	}

	ret = tmc522x_write(dev, TMC522X_REG_PWMCONF, val);
	if (ret) {
		LOG_ERR("Failed to write PWMCONF");
		return ret;
	}

	/* Configure COOLCONF - StallGuard2 threshold + CoolStep parameters */
	ret = tmc522x_read(dev, TMC522X_REG_COOLCONF, &val);
	if (ret) {
		LOG_ERR("Failed to read COOLCONF");
		return ret;
	}

	/* Clear all CoolStep and StallGuard fields */
	val &= ~(TMC522X_COOLCONF_SEMIN_MASK |
		 TMC522X_COOLCONF_SEMAX_MASK |
		 TMC522X_COOLCONF_SEUP_MASK |
		 TMC522X_COOLCONF_SEDN_MASK |
		 TMC522X_COOLCONF_SEIMIN_MASK |
		 TMC522X_COOLCONF_SGT_MASK);

	/* Set CoolStep parameters */
	val |= TMC522X_FIELD_PREP(TMC522X_COOLCONF_SEMIN_MASK, data->semin);
	val |= TMC522X_FIELD_PREP(TMC522X_COOLCONF_SEMAX_MASK, data->semax);
	val |= TMC522X_FIELD_PREP(TMC522X_COOLCONF_SEUP_MASK, data->seup);
	val |= TMC522X_FIELD_PREP(TMC522X_COOLCONF_SEDN_MASK, data->sedn);
	if (data->seimin) {
		val |= TMC522X_COOLCONF_SEIMIN_MASK;
	}

	/* Set StallGuard2 threshold */
	val |= TMC522X_FIELD_PREP(TMC522X_COOLCONF_SGT_MASK, data->sgt);

	ret = tmc522x_write(dev, TMC522X_REG_COOLCONF, val);
	if (ret) {
		LOG_ERR("Failed to write COOLCONF");
		return ret;
	}

	/* Log CoolStep configuration if enabled */
	if (data->semin > 0) {
		LOG_INF("CoolStep enabled: semin=%u, semax=%u, seup=%u, sedn=%u, seimin=%u",
			data->semin, data->semax, data->seup, data->sedn, data->seimin);
	} else {
		LOG_INF("CoolStep disabled (semin=0)");
	}

	/* Configure SG4_THRS - StallGuard4 threshold */
	ret = tmc522x_read(dev, TMC522X_REG_SG4_THRS, &val);
	if (ret) {
		LOG_ERR("Failed to read SG4_THRS");
		return ret;
	}

	val &= ~TMC522X_SG4_THRS_MASK;
	val |= TMC522X_FIELD_PREP(TMC522X_SG4_THRS_MASK, data->sgt4);

	ret = tmc522x_write(dev, TMC522X_REG_SG4_THRS, val);
	if (ret) {
		LOG_ERR("Failed to write SG4_THRS");
		return ret;
	}

	/* Configure DIAG_CONF - diagnostic pin configuration */
	/* Enable DIAG0 and DIAG1 as active low push-pull */
	val = TMC522X_DIAG1_INVPP_MASK | TMC522X_DIAG1_NOD_PP_MASK |
	      TMC522X_DIAG0_INVPP_MASK | TMC522X_DIAG0_NOD_PP_MASK;

	/* Configure DIAG0 as STALL event and DIAG1 as POS_REACHED event */
	val |= TMC522X_DIAG0_STALL_MASK | TMC522X_DIAG1_POS_REACH_MASK;

	ret = tmc522x_write(dev, TMC522X_REG_DIAG_CONF, val);
	if (ret) {
		LOG_ERR("Failed to write DIAG_CONF");
		return ret;
	}

	/* Read back to verify */
	ret = tmc522x_read(dev, TMC522X_REG_DIAG_CONF, &val);
	if (ret) {
		LOG_ERR("Failed to read back DIAG_CONF");
	} else {
		LOG_INF("DIAG_CONF = 0x%08X (DIAG0_STALL=%d, DIAG1_POS=%d)",
			val,
			!!(val & TMC522X_DIAG0_STALL_MASK),
			!!(val & TMC522X_DIAG1_POS_REACH_MASK));
	}

	/* Configure chopper mode (StealthChop or SpreadCycle) */
	LOG_INF("enable_stealthchop from DT = %d (0=SpreadCycle, 1=StealthChop)",
		cfg->enable_stealthchop);

	ret = tmc522x_stepper_set_chopper_mode(dev, cfg->enable_stealthchop);
	if (ret) {
		LOG_ERR("Failed to set chopper mode: %d", ret);
		return ret;
	}

	/* Small delay to ensure UART buffer doesn't overflow */
	k_msleep(10);

	LOG_INF("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
	LOG_INF("  CHOPPER MODE: %s", cfg->enable_stealthchop ? "StealthChop" : "SpreadCycle");
	LOG_INF("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");

	/* Read back GCONF to verify chopper mode */
	ret = tmc522x_read(dev, TMC522X_REG_GCONF, &val);
	if (ret == 0) {
		LOG_INF("GCONF = 0x%08X (EN_PWM_MODE=%d)", val, !!(val & TMC522X_GCONF_EN_PWM_MODE));
	}

	/* Read DRV_STATUS to check if StealthChop is actually active during motion */
	ret = tmc522x_read(dev, TMC522X_REG_DRV_STATUS, &val);
	if (ret == 0) {
		LOG_INF("DRV_STATUS = 0x%08X (StealthChop_active=%d, StallGuard_flag=%d)",
			val,
			!!(val & TMC522X_DRV_STATUS_STEALTHCHOP),
			!!(val & TMC522X_DRV_STATUS_STALLGUARD));
	}

	LOG_INF("TMC522X hardware configuration complete");
	return 0;
}

/*
 * ============================================================================
 * Zephyr Stepper API Implementation
 * ============================================================================
 */

static int tmc522x_stepper_enable(const struct device *dev)
{
	const struct tmc522x_config *cfg = dev->config;
	struct tmc522x_data *data = dev->data;
	int ret;

	LOG_INF("Enabling TMC522X stepper motor");

	/* Step 3: Assert DRVEN GPIO HIGH - Hardware Safety Release
	 * This is the primary safety mechanism. The DRV_EN pin has an internal
	 * pull-down, so if this GPIO is not driven HIGH, the motor is guaranteed
	 * to be disabled at the hardware level, regardless of software state.
	 */
	if (cfg->has_drven) {
		LOG_INF("Setting DRVEN GPIO (port=%s pin=%d) to HIGH",
		        cfg->drven_gpio.port->name, cfg->drven_gpio.pin);
		ret = gpio_pin_set_dt(&cfg->drven_gpio, 1);
		if (ret) {
			LOG_ERR("Failed to enable DRVEN GPIO: %d", ret);
			return ret;
		}
		LOG_INF("DRVEN GPIO set command completed");

		/* Try to read back GPIO state (if supported) */
		int gpio_state = gpio_pin_get_dt(&cfg->drven_gpio);
		if (gpio_state >= 0) {
			LOG_INF("DRVEN GPIO readback: %d (should be 1)", gpio_state);
			if (gpio_state == 0) {
				LOG_ERR("ERROR: GPIO readback shows LOW! Pin may have external pull-down or conflict!");
			}
		} else {
			LOG_WRN("Cannot read GPIO state (ret=%d)", gpio_state);
		}

		k_msleep(1);  /* Brief delay for driver stage to stabilize */
	} else {
		LOG_WRN("DRVEN GPIO not configured - motor may not work!");
	}

	/* Step 5: Clear drv_enn bit in GCONF register - Software Gate Release
	 * This is the software enable gate. Even with DRV_EN HIGH, the driver
	 * will not activate until this bit is cleared.
	 */
	ret = tmc522x_driver_enable(dev);
	if (ret) {
		LOG_ERR("Failed to enable driver via GCONF register: %d", ret);
		return ret;
	}

	/* Apply chopper mode from devicetree configuration
	 * This must be done AFTER driver_enable() to ensure EN_PWM_MODE bit is preserved
	 */
	ret = tmc522x_stepper_set_chopper_mode(dev, cfg->enable_stealthchop);
	if (ret) {
		LOG_ERR("Failed to set chopper mode during enable: %d", ret);
		return ret;
	}

	data->enabled = true;
	LOG_INF("TMC522X enabled - motor ready");
	return 0;
}

static int tmc522x_stepper_disable(const struct device *dev)
{
	const struct tmc522x_config *cfg = dev->config;
	struct tmc522x_data *data = dev->data;
	int ret;

	LOG_DBG("Disabling TMC522X stepper motor");

	/* Step 1: Set drv_enn bit in GCONF register - Software Gate Engage
	 * This prevents the driver from being enabled via software until
	 * stepper_enable() is called again.
	 */
	ret = tmc522x_driver_disable(dev);
	if (ret) {
		LOG_ERR("Failed to disable driver via GCONF register: %d", ret);
		return ret;
	}

	/* Step 2: Clear DRVEN GPIO - Hardware Safety Engage
	 * This is the final safety step. With DRV_EN LOW (pulled down by
	 * internal resistor), the driver stage is guaranteed disabled at
	 * the hardware level, providing a fail-safe mechanism.
	 */
	if (cfg->has_drven) {
		ret = gpio_pin_set_dt(&cfg->drven_gpio, 0);
		if (ret) {
			LOG_ERR("Failed to disable DRVEN GPIO: %d", ret);
			return ret;
		}
	}

	data->enabled = false;
	LOG_INF("TMC522X disabled");
	return 0;
}

static int tmc522x_stepper_move_to(const struct device *dev, int32_t micro_steps)
{
	LOG_DBG("Moving to absolute position: %d micro-steps", micro_steps);
	return tmc522x_set_position(dev, micro_steps);
}

static int tmc522x_stepper_move_by(const struct device *dev, int32_t micro_steps)
{
	int32_t current_pos;
	uint32_t xactual;
	int ret;

	/* Read current position from XACTUAL register */
	ret = tmc522x_read(dev, TMC522X_REG_XACTUAL, &xactual);
	if (ret) {
		LOG_ERR("Failed to read current position: %d", ret);
		return ret;
	}

	current_pos = (int32_t)xactual;
	LOG_DBG("Moving by %d micro-steps from position %d", micro_steps, current_pos);

	/* Move to new position */
	return tmc522x_set_position(dev, current_pos + micro_steps);
}

static int tmc522x_stepper_run(const struct device *dev, enum stepper_direction direction)
{
	struct tmc522x_data *data = dev->data;
	int32_t velocity;

	/* Use VMAX for continuous velocity mode */
	velocity = (direction == STEPPER_DIRECTION_POSITIVE) ?
		   (int32_t)data->vmax : -(int32_t)data->vmax;

	LOG_DBG("Running continuously at velocity: %d", velocity);
	return tmc522x_set_velocity(dev, velocity);
}

static int tmc522x_stepper_stop(const struct device *dev)
{
	LOG_DBG("Stopping motor");
	/* Set velocity to 0 to decelerate and stop */
	return tmc522x_set_velocity(dev, 0);
}

static int tmc522x_stepper_set_micro_step_res(const struct device *dev,
					      enum stepper_micro_step_resolution resolution)
{
	struct tmc522x_data *data = dev->data;
	uint32_t chopconf;
	uint8_t mres;
	int ret;

	/* Validate resolution */
	if (!VALID_MICRO_STEP_RES(resolution)) {
		LOG_ERR("Invalid micro-step resolution: %d", resolution);
		return -EINVAL;
	}

	/* Convert enum to MRES register value
	 * MRES encoding: 0=256, 1=128, 2=64, 3=32, 4=16, 5=8, 6=4, 7=2, 8=fullstep
	 * resolution enum: 256, 128, 64, 32, 16, 8, 4, 2, 1
	 */
	mres = 8 - MICRO_STEP_RES_INDEX(resolution);

	/* Read CHOPCONF register */
	ret = tmc522x_read(dev, TMC522X_REG_CHOPCONF, &chopconf);
	if (ret) {
		LOG_ERR("Failed to read CHOPCONF register: %d", ret);
		return ret;
	}

	/* Update MRES field */
	chopconf &= ~TMC522X_CHOPCONF_MRES_MASK;
	chopconf |= TMC522X_FIELD_PREP(TMC522X_CHOPCONF_MRES_MASK, mres);

	/* Write back CHOPCONF register */
	ret = tmc522x_write(dev, TMC522X_REG_CHOPCONF, chopconf);
	if (ret) {
		LOG_ERR("Failed to write CHOPCONF register: %d", ret);
		return ret;
	}

	/* Update cached value */
	data->microstep_res = mres;
	LOG_INF("Micro-step resolution set to 1/%d", resolution);
	return 0;
}

static int tmc522x_stepper_get_micro_step_res(const struct device *dev,
					      enum stepper_micro_step_resolution *resolution)
{
	struct tmc522x_data *data = dev->data;

	if (resolution == NULL) {
		return -EINVAL;
	}

	/* Convert MRES value back to enum
	 * MRES: 0=256, 1=128, 2=64, ..., 8=fullstep
	 * enum: 256,  128,  64,  ..., 1
	 */
	*resolution = 1 << (8 - data->microstep_res);
	LOG_DBG("Current micro-step resolution: 1/%d", *resolution);
	return 0;
}

static int tmc522x_stepper_set_reference_position(const struct device *dev, int32_t value)
{
	LOG_DBG("Setting reference position to: %d", value);
	return tmc522x_write(dev, TMC522X_REG_XACTUAL, (uint32_t)value);
}

static int tmc522x_stepper_get_actual_position(const struct device *dev, int32_t *position)
{
	uint32_t xactual;
	int ret;

	if (position == NULL) {
		return -EINVAL;
	}

	ret = tmc522x_read(dev, TMC522X_REG_XACTUAL, &xactual);
	if (ret) {
		LOG_ERR("Failed to read XACTUAL register: %d", ret);
		return ret;
	}

	*position = (int32_t)xactual;
	LOG_DBG("Actual position: %d", *position);
	return 0;
}

static int tmc522x_stepper_is_moving(const struct device *dev, bool *is_moving)
{
	uint32_t vactual;
	int ret;

	if (is_moving == NULL) {
		return -EINVAL;
	}

	/* Read actual velocity from VACTUAL register */
	ret = tmc522x_read(dev, TMC522X_REG_VACTUAL, &vactual);
	if (ret) {
		LOG_ERR("Failed to read VACTUAL register: %d", ret);
		return ret;
	}

	/* Motor is moving if velocity is non-zero */
	*is_moving = (vactual != 0);
	LOG_DBG("Motor %s", *is_moving ? "is moving" : "is stopped");
	return 0;
}

static int tmc522x_stepper_set_event_callback(const struct device *dev,
					      stepper_event_callback_t callback,
					      void *user_data)
{
	struct tmc522x_data *data = dev->data;

	data->event_callback = callback;
	data->event_callback_user_data = user_data;
	LOG_DBG("Event callback %s", callback ? "registered" : "cleared");
	return 0;
}

static DEVICE_API(stepper, tmc522x_stepper_api) = {
	.enable = tmc522x_stepper_enable,
	.disable = tmc522x_stepper_disable,
	.move_by = tmc522x_stepper_move_by,
	.move_to = tmc522x_stepper_move_to,
	.run = tmc522x_stepper_run,
	.stop = tmc522x_stepper_stop,
	.set_micro_step_res = tmc522x_stepper_set_micro_step_res,
	.get_micro_step_res = tmc522x_stepper_get_micro_step_res,
	.set_reference_position = tmc522x_stepper_set_reference_position,
	.get_actual_position = tmc522x_stepper_get_actual_position,
	.is_moving = tmc522x_stepper_is_moving,
	.set_event_callback = tmc522x_stepper_set_event_callback,
};

/*
 * ============================================================================
 * Public Helper API Functions (Non-Standard)
 * ============================================================================
 */

/* TMC522X register definitions */
#define TMC522X_WRITE_BIT    0x80U
#define TMC522X_ADDRESS_MASK 0x7FU

/* Public API functions */
int tmc522x_reg_write(const struct device *dev, uint8_t reg_addr, uint32_t value)
{
	return tmc522x_write(dev, reg_addr, value);
}

int tmc522x_reg_read(const struct device *dev, uint8_t reg_addr, uint32_t *value)
{
	return tmc522x_read(dev, reg_addr, value);
}

int tmc522x_burst_read(const struct device *dev, uint8_t start_addr, uint8_t *buf, size_t num_regs)
{
	int err;

	/* SPI requires individual reads for each register */
	for (size_t i = 0; i < num_regs; i++) {
		uint32_t value;

		err = tmc522x_read(dev, start_addr + i, &value);
		if (err) {
			LOG_ERR("Burst read failed at reg 0x%02x: %d", start_addr + i, err);
			return err;
		}

		/* Store as big-endian in buffer */
		buf[i * 4 + 0] = (uint8_t)(value >> 24);
		buf[i * 4 + 1] = (uint8_t)(value >> 16);
		buf[i * 4 + 2] = (uint8_t)(value >> 8);
		buf[i * 4 + 3] = (uint8_t)(value);
	}

	return 0;
}

int tmc522x_stepper_set_chopper_mode(const struct device *dev, bool stealthchop2)
{
	uint32_t gconf;
	int ret = tmc522x_read(dev, TMC522X_REG_GCONF, &gconf);

	if (ret) {
		LOG_ERR("Failed to read GCONF: %d", ret);
		return ret;
	}

	if (stealthchop2) {
		gconf |= TMC522X_GCONF_EN_PWM_MODE;  /* Enable StealthChop2 */
	} else {
		gconf &= ~TMC522X_GCONF_EN_PWM_MODE; /* Enable SpreadCycle */
	}

	ret = tmc522x_write(dev, TMC522X_REG_GCONF, gconf);
	if (ret) {
		LOG_ERR("Failed to write GCONF: %d", ret);
	}
	return ret;
}

/* Enable driver stage via GCONF register (clear drv_enn software gate)
 * IMPORTANT: Driver will NOT activate unless DRV_EN pin is HIGH (hardware safety),
 * DRV_EN_SW is set, drv_enn is clear, and TOFF is non-zero.
 */
int tmc522x_driver_enable(const struct device *dev)
{
	uint32_t gconf;
	int ret = tmc522x_read(dev, TMC522X_REG_GCONF, &gconf);

	if (ret) {
		LOG_ERR("Failed to read GCONF: %d", ret);
		return ret;
	}

	LOG_INF("GCONF before enable: 0x%08X (drv_enn bit 9 = %d)",
	        gconf, (gconf >> 9) & 1);

	/* SET drv_enn (bit 9) to enable the driver
	 * NOTE: Contrary to typical naming, bit 9 must be SET (1) to enable!
	 * The hardware DRV_EN pin must also be HIGH for motor to activate.
	 */
	gconf |= TMC522X_GCONF_DRV_ENN;

	LOG_INF("GCONF after enable:  0x%08X (drv_enn bit 9 = %d)",
	        gconf, (gconf >> 9) & 1);

	ret = tmc522x_write(dev, TMC522X_REG_GCONF, gconf);
	if (ret) {
		LOG_ERR("Failed to write GCONF: %d", ret);
		return ret;
	}

	/* Read back to verify */
	ret = tmc522x_read(dev, TMC522X_REG_GCONF, &gconf);
	if (ret == 0) {
		LOG_INF("GCONF readback:     0x%08X (drv_enn bit 9 = %d)",
		        gconf, (gconf >> 9) & 1);
		if ((gconf >> 9) & 1) {
			LOG_INF("SUCCESS: drv_enn bit SET to 1. Driver enabled.");
		} else {
			LOG_ERR("ERROR: drv_enn bit is 0! Driver is DISABLED!");
		}
	}

	return ret;
}

/* Disable driver stage via GCONF register */
int tmc522x_driver_disable(const struct device *dev)
{
	uint32_t gconf;
	int ret = tmc522x_read(dev, TMC522X_REG_GCONF, &gconf);

	if (ret) {
		LOG_ERR("Failed to read GCONF: %d", ret);
		return ret;
	}

	/* CLEAR drv_enn (bit 9) to disable the driver
	 * NOTE: Despite the name, bit 9 = 0 disables, bit 9 = 1 enables!
	 */
	gconf &= ~TMC522X_GCONF_DRV_ENN;

	ret = tmc522x_write(dev, TMC522X_REG_GCONF, gconf);
	if (ret) {
		LOG_ERR("Failed to write GCONF: %d", ret);
	}
	return ret;
}

int tmc522x_drven_enable(const struct device *dev)
{
	const struct tmc522x_config *cfg = dev->config;
	int ret;

	if (!cfg->has_drven) {
		return 0;
	}
	ret = gpio_pin_set_dt(&cfg->drven_gpio, 1);
	if (ret) {
		LOG_ERR("Failed to set DRVEN GPIO: %d", ret);
	}
	return ret;
}

int tmc522x_drven_disable(const struct device *dev)
{
	const struct tmc522x_config *cfg = dev->config;
	int ret;

	if (!cfg->has_drven) {
		return 0;
	}
	ret = gpio_pin_set_dt(&cfg->drven_gpio, 0);
	if (ret) {
		LOG_ERR("Failed to clear DRVEN GPIO: %d", ret);
	}
	return ret;
}

/* Velocity Mode and Runtime Monitoring */

int tmc522x_set_velocity(const struct device *dev, int32_t velocity)
{
	struct tmc522x_data *data = dev->data;
	uint32_t vmax_abs;
	enum tmc522x_rampmode rampmode;
	int ret;

	if (velocity == 0) {
		/* Stop motor - set VMAX to 0 to decelerate using ramps
		 * NOTE: Don't use HOLD mode! HOLD freezes the ramp generator
		 * but doesn't decelerate the motor - it keeps spinning!
		 *
		 * Setting VMAX=0 tells the motor controller to decelerate
		 * to zero velocity using the configured DMAX deceleration.
		 */
		ret = tmc522x_write(dev, TMC522X_REG_VMAX, 0);
		if (ret) {
			LOG_ERR("Failed to set VMAX=0: %d", ret);
			return ret;
		}
		LOG_DBG("Set VMAX=0 to decelerate motor to stop");
		return 0;
	}

	/* Determine direction and set ramp mode */
	if (velocity < 0) {
		vmax_abs = (uint32_t)(-velocity);
		rampmode = TMC522X_RAMPMODE_NEGATIVE_VELOCITY;
	} else {
		vmax_abs = (uint32_t)velocity;
		rampmode = TMC522X_RAMPMODE_POSITIVE_VELOCITY;
	}

	/* CRITICAL FIX: Ensure VSTART is non-zero for velocity mode
	 * TMC5221 datasheet Section 6.1: VSTART = 0 prevents motor from starting.
	 * VSTART defines the initial velocity before acceleration begins.
	 * Minimum recommended value: 10
	 */
	if (data->vstart == 0) {
		uint32_t vstart_default = 10;  /* Minimum starting velocity */
		LOG_WRN("VSTART is 0! Setting to %u for velocity mode to work", vstart_default);
		ret = tmc522x_write(dev, TMC522X_REG_VSTART, vstart_default);
		if (ret) {
			LOG_ERR("Failed to write VSTART: %d", ret);
			return ret;
		}
		data->vstart = vstart_default;
	}

	/* Ensure VSTOP is non-zero (minimum velocity threshold) */
	if (data->vstop == 0) {
		uint32_t vstop_default = 10;
		LOG_WRN("VSTOP is 0! Setting to %u", vstop_default);
		ret = tmc522x_write(dev, TMC522X_REG_VSTOP, vstop_default);
		if (ret) {
			LOG_ERR("Failed to write VSTOP: %d", ret);
			return ret;
		}
		data->vstop = vstop_default;
	}

	/* Write VMAX register with absolute velocity */
	ret = tmc522x_write(dev, TMC522X_REG_VMAX, vmax_abs);
	if (ret) {
		LOG_ERR("Failed to write VMAX: %d", ret);
		return ret;
	}

	/* Write AMAX if it's zero (ensure motor can accelerate) */
	if (data->amax == 0) {
		LOG_WRN("AMAX is 0, motor may not accelerate. Setting to default.");
		ret = tmc522x_write(dev, TMC522X_REG_AMAX, 1000);
		if (ret) {
			LOG_ERR("Failed to write AMAX: %d", ret);
			return ret;
		}
		data->amax = 1000;
	}

	/* Set ramp mode to velocity mode */
	ret = tmc522x_write(dev, TMC522X_REG_RAMPMODE, rampmode);
	if (ret) {
		LOG_ERR("Failed to set velocity mode: %d", ret);
		return ret;
	}

	data->rampmode = rampmode;
	data->vmax = vmax_abs;

	LOG_DBG("Velocity mode set: velocity=%d, mode=%d", velocity, rampmode);
	return 0;
}

int tmc522x_get_actual_velocity(const struct device *dev, int32_t *velocity)
{
	uint32_t vactual_raw;
	int ret;

	if (!velocity) {
		return -EINVAL;
	}

	ret = tmc522x_reg_read(dev, TMC522X_REG_VACTUAL, &vactual_raw);
	if (ret) {
		LOG_ERR("Failed to read VACTUAL: %d", ret);
		return ret;
	}

	/* VACTUAL is a 24-bit signed value
	 * Sign-extend by shifting left 8 bits, then arithmetic shift right 8 bits
	 * CRITICAL: Cast to int32_t BEFORE right shift to ensure arithmetic shift
	 */
	*velocity = ((int32_t)(vactual_raw << 8)) >> 8;

	return 0;
}

int tmc522x_get_actual_acceleration(const struct device *dev, int32_t *accel)
{
	uint32_t aactual_raw;
	int ret;

	if (!accel) {
		return -EINVAL;
	}

	ret = tmc522x_reg_read(dev, TMC522X_REG_AACTUAL, &aactual_raw);
	if (ret) {
		LOG_ERR("Failed to read AACTUAL: %d", ret);
		return ret;
	}

	/* AACTUAL is a 24-bit signed value
	 * Sign-extend by shifting left 8 bits, then arithmetic shift right 8 bits
	 * CRITICAL: Cast to int32_t BEFORE right shift to ensure arithmetic shift
	 */
	*accel = ((int32_t)(aactual_raw << 8)) >> 8;

	return 0;
}

/* Velocity Threshold Configuration */

static uint32_t tmc522x_velocity_to_threshold(const struct device *dev, uint32_t velocity_pps)
{
	struct tmc522x_data *data = dev->data;
	uint64_t numerator, denominator;
	uint32_t microsteps_per_fullstep;
	uint32_t threshold;

	if (velocity_pps == 0) {
		return 0;
	}

	microsteps_per_fullstep = 256 >> data->microstep_res;

	/* Calculate: (clock * step_angle_millideg) / (velocity * 1000 * microsteps) */
	numerator = (uint64_t)data->clock * data->step_angle_millidegrees;
	denominator = (uint64_t)velocity_pps * 1000ULL * microsteps_per_fullstep;

	if (denominator == 0) {
		return UINT32_MAX;
	}

	threshold = (uint32_t)(numerator / denominator);

	LOG_DBG("Velocity %u pps -> threshold %u (clock=%u, angle=%u, mres=%u)",
		velocity_pps, threshold, data->clock, data->step_angle_millidegrees,
		data->microstep_res);

	return threshold;
}

int tmc522x_set_stealthchop_threshold(const struct device *dev, uint32_t velocity_pps)
{
	struct tmc522x_data *data = dev->data;
	uint32_t tpwmthrs;
	uint32_t gconf;
	int ret;

	k_sem_take(&data->sem, K_FOREVER);

	if (velocity_pps == 0) {
		/* Disable StealthChop2 */
		ret = tmc522x_read(dev, TMC522X_REG_GCONF, &gconf);
		if (ret) {
			k_sem_give(&data->sem);
			return ret;
		}

		gconf &= ~TMC522X_GCONF_EN_PWM_MODE;
		ret = tmc522x_write(dev, TMC522X_REG_GCONF, gconf);
		if (ret) {
			k_sem_give(&data->sem);
			return ret;
		}

		ret = tmc522x_write(dev, TMC522X_REG_TPWMTHRS, 0);
		if (ret) {
			k_sem_give(&data->sem);
			return ret;
		}

		data->tpwmthrs = 0;
		LOG_INF("StealthChop2 disabled");
	} else {
		/* Convert velocity to threshold value */
		tpwmthrs = tmc522x_velocity_to_threshold(dev, velocity_pps);

		/* Enable StealthChop2 in GCONF */
		ret = tmc522x_read(dev, TMC522X_REG_GCONF, &gconf);
		if (ret) {
			k_sem_give(&data->sem);
			return ret;
		}

		gconf |= TMC522X_GCONF_EN_PWM_MODE;
		ret = tmc522x_write(dev, TMC522X_REG_GCONF, gconf);
		if (ret) {
			k_sem_give(&data->sem);
			return ret;
		}

		/* Write threshold */
		ret = tmc522x_write(dev, TMC522X_REG_TPWMTHRS, tpwmthrs);
		if (ret) {
			k_sem_give(&data->sem);
			return ret;
		}

		data->tpwmthrs = tpwmthrs;
		LOG_INF("StealthChop2 enabled below %u pps (TPWMTHRS=%u)", velocity_pps, tpwmthrs);
	}

	k_sem_give(&data->sem);
	return 0;
}

int tmc522x_set_coolstep_threshold(const struct device *dev, uint32_t velocity_pps)
{
	struct tmc522x_data *data = dev->data;
	uint32_t tcoolthrs;
	int ret;

	k_sem_take(&data->sem, K_FOREVER);

	/* Convert velocity to threshold value */
	tcoolthrs = tmc522x_velocity_to_threshold(dev, velocity_pps);

	/* Write threshold */
	ret = tmc522x_write(dev, TMC522X_REG_TCOOLTHRS, tcoolthrs);
	if (ret) {
		k_sem_give(&data->sem);
		return ret;
	}

	data->tcoolthrs = tcoolthrs;

	if (velocity_pps == 0) {
		LOG_INF("CoolStep/StallGuard disabled");
	} else {
		LOG_INF("CoolStep/StallGuard active above %u pps (TCOOLTHRS=%u)",
			velocity_pps, tcoolthrs);
	}

	k_sem_give(&data->sem);
	return 0;
}

int tmc522x_set_fullstep_threshold(const struct device *dev, uint32_t velocity_pps)
{
	struct tmc522x_data *data = dev->data;
	uint32_t thigh;
	uint32_t chopconf;
	int ret;

	k_sem_take(&data->sem, K_FOREVER);

	if (velocity_pps == 0) {
		/* Disable fullstep switching */
		ret = tmc522x_read(dev, TMC522X_REG_CHOPCONF, &chopconf);
		if (ret) {
			k_sem_give(&data->sem);
			return ret;
		}

		chopconf &= ~TMC522X_CHOPCONF_VHIGHFS_MASK;
		ret = tmc522x_write(dev, TMC522X_REG_CHOPCONF, chopconf);
		if (ret) {
			k_sem_give(&data->sem);
			return ret;
		}

		ret = tmc522x_write(dev, TMC522X_REG_THIGH, 0);
		if (ret) {
			k_sem_give(&data->sem);
			return ret;
		}

		data->thigh = 0;
		LOG_INF("Fullstep switching disabled");
	} else {
		/* Convert velocity to threshold value */
		thigh = tmc522x_velocity_to_threshold(dev, velocity_pps);

		/* Enable VHIGHFS in CHOPCONF */
		ret = tmc522x_read(dev, TMC522X_REG_CHOPCONF, &chopconf);
		if (ret) {
			k_sem_give(&data->sem);
			return ret;
		}

		chopconf |= TMC522X_CHOPCONF_VHIGHFS_MASK;
		ret = tmc522x_write(dev, TMC522X_REG_CHOPCONF, chopconf);
		if (ret) {
			k_sem_give(&data->sem);
			return ret;
		}

		/* Write threshold */
		ret = tmc522x_write(dev, TMC522X_REG_THIGH, thigh);
		if (ret) {
			k_sem_give(&data->sem);
			return ret;
		}

		data->thigh = thigh;
		LOG_INF("Fullstep switching above %u pps (THIGH=%u)", velocity_pps, thigh);
	}

	k_sem_give(&data->sem);
	return 0;
}

int tmc522x_enable_stallguard_event(const struct device *dev, bool enable)
{
	struct tmc522x_data *data = dev->data;
	const struct tmc522x_config *cfg = dev->config;
	uint32_t ramp_stat = 0;
	int ret;

	/* Enable/disable the flag */
	data->stallguard_event_enabled = enable;

	if (enable && cfg->has_stallguard_irq) {
		/* Read current RAMP_STAT to check for pending stall events */
		ret = tmc522x_read(dev, TMC522X_REG_RAMP_STAT, &ramp_stat);
		if (ret == 0) {
			LOG_INF("RAMP_STAT at enable: 0x%08X (bit6_stall=%d)",
				ramp_stat, !!(ramp_stat & TMC522X_EVENT_STOP_SG_MASK));

			/* If stall bit is already set, clear it */
			if (ramp_stat & TMC522X_EVENT_STOP_SG_MASK) {
				LOG_WRN("Stall event already pending - clearing");
				tmc522x_write(dev, TMC522X_REG_RAMP_STAT, TMC522X_EVENT_STOP_SG_MASK);
			}
		}

		/* Log GPIO state */
		LOG_INF("DIAG0 GPIO: port=%s pin=%d",
			cfg->stallguard_gpio.port->name, cfg->stallguard_gpio.pin);
	}

	LOG_INF("StallGuard event notifications %s", enable ? "enabled" : "disabled");
	return 0;
}

/* Interrupt Support */

static void tmc522x_stallguard_work_handler(struct k_work *work)
{
	struct tmc522x_data *data = CONTAINER_OF(work, struct tmc522x_data, stallguard_work);
	const struct device *dev = data->dev;
	uint32_t ramp_stat;
	int ret;

	/* Read RAMP_STAT to confirm stall event */
	ret = tmc522x_reg_read(dev, TMC522X_REG_RAMP_STAT, &ramp_stat);
	if (ret) {
		LOG_ERR("Failed to read RAMP_STAT in stallguard handler: %d", ret);
		return;
	}

	if (ramp_stat & TMC522X_EVENT_STOP_SG_MASK) {
		LOG_INF("StallGuard event detected! RAMP_STAT: 0x%08x", ramp_stat);

		/* Clear the event by writing 1 to the bit */
		ret = tmc522x_reg_write(dev, TMC522X_REG_RAMP_STAT,
					TMC522X_EVENT_STOP_SG_MASK);
		if (ret) {
			LOG_ERR("Failed to clear stallguard event: %d", ret);
		}

		/* Notify application via callback if registered */
		if (data->event_callback && data->stallguard_event_enabled) {
			enum stepper_event event = STEPPER_EVENT_STALL_DETECTED;
			data->event_callback(dev, event, data->event_callback_user_data);
		}
	}
}

static void tmc522x_pos_reached_work_handler(struct k_work *work)
{
	struct tmc522x_data *data = CONTAINER_OF(work, struct tmc522x_data,
						  pos_reached_work);
	const struct device *dev = data->dev;
	uint32_t ramp_stat;
	int ret;

	/* Read RAMP_STAT to confirm position reached event */
	ret = tmc522x_reg_read(dev, TMC522X_REG_RAMP_STAT, &ramp_stat);
	if (ret) {
		LOG_ERR("Failed to read RAMP_STAT in pos_reached handler: %d", ret);
		return;
	}

	if (ramp_stat & TMC522X_EVENT_POS_REACH_MASK) {
		LOG_INF("Position reached event detected! RAMP_STAT: 0x%08x", ramp_stat);

		/* Clear the event by writing 1 to the bit */
		ret = tmc522x_reg_write(dev, TMC522X_REG_RAMP_STAT,
					TMC522X_EVENT_POS_REACH_MASK);
		if (ret) {
			LOG_ERR("Failed to clear position reached event: %d", ret);
		}

		/* Notify application via callback if registered */
		if (data->event_callback && data->pos_reached_event_enabled) {
			enum stepper_event event = STEPPER_EVENT_STEPS_COMPLETED;
			data->event_callback(dev, event, data->event_callback_user_data);
		}
	}
}

static void tmc522x_stallguard_gpio_callback(const struct device *port,
					      struct gpio_callback *cb,
					      gpio_port_pins_t pins)
{
	struct tmc522x_data *data = CONTAINER_OF(cb, struct tmc522x_data, stallguard_cb);

	/* Submit work to system workqueue to handle in thread context */
	k_work_submit(&data->stallguard_work);
}

static void tmc522x_pos_reached_gpio_callback(const struct device *port,
						struct gpio_callback *cb,
						gpio_port_pins_t pins)
{
	struct tmc522x_data *data = CONTAINER_OF(cb, struct tmc522x_data, pos_reached_cb);

	/* Submit work to system workqueue to handle in thread context */
	k_work_submit(&data->pos_reached_work);
}

static int tmc522x_init(const struct device *dev)
{
	const struct tmc522x_config *cfg = dev->config;
	struct tmc522x_data *data = dev->data;

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(spi)
	if (cfg->bus_io == &tmc522x_bus_io_spi) {
		LOG_INF("Initializing TMC522X on SPI bus");
	}
#endif
#if DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c)
	if (cfg->bus_io == &tmc522x_bus_io_i2c) {
		LOG_INF("Initializing TMC522X on I2C bus");
	}
#endif

	/* Initialize semaphore for bus protection */
	k_sem_init(&data->sem, 1, 1);

	/* Store device pointer for interrupt context */
	data->dev = dev;

	/* Read configuration from devicetree */
	data->clock = cfg->clock_frequency;
	data->enabled = false;

	/* Initialize motion parameters from devicetree defaults */
	data->vstart = cfg->default_vstart;
	data->a1 = cfg->default_a1;
	data->v1 = cfg->default_v1;
	data->amax = cfg->default_amax;
	data->vmax = cfg->default_vmax;
	data->dmax = cfg->default_dmax;
	data->d1 = cfg->default_d1;
	data->vstop = cfg->default_vstop;

	/* Initialize secondary ramp phase (TMC522X two-phase ramp) */
	data->a2 = cfg->default_a2;
	data->v2 = cfg->default_v2;
	data->d2 = cfg->default_d2;

	/* Initialize current settings from devicetree defaults */
	data->current_run = cfg->default_irun;
	data->current_hold = cfg->default_ihold;
	data->iholddelay = cfg->default_iholddelay;
	data->irundelay = cfg->default_irundelay;
	data->tpowerdown = cfg->default_tpowerdown;

	/* Initialize microstep resolution from devicetree */
	data->microstep_res = cfg->default_micro_step_res;

	/* Initialize ramp mode */
	data->rampmode = TMC522X_RAMPMODE_POSITION;

	/* Initialize event flags */
	data->stallguard_event_enabled = false;
	data->pos_reached_event_enabled = false;

	/* Initialize StallGuard configuration */
	data->sg_stop = false;

	/* Initialize cached threshold values */
	data->tpwmthrs = 0;
	data->tcoolthrs = 0;
	data->thigh = 0;

	/* Initialize step angle from devicetree */
	data->step_angle_millidegrees = cfg->default_step_angle_millidegrees;

	/* Check if bus is ready (SPI or I2C) */
	int ret = cfg->bus_io->check(&cfg->bus);
	if (ret) {
		LOG_ERR("Bus not ready: %d", ret);
		return ret;
	}

	/* Configure SLEEPN GPIO (Step 1 of datasheet power-up sequence) */
	if (cfg->has_sleepn) {
		if (!gpio_is_ready_dt(&cfg->sleepn_gpio)) {
			LOG_ERR("SLEEPN GPIO not ready");
			return -ENODEV;
		}
		ret = gpio_pin_configure_dt(&cfg->sleepn_gpio, GPIO_OUTPUT_ACTIVE);
		if (ret) {
			LOG_ERR("Failed to configure SLEEPN GPIO: %d", ret);
			return ret;
		}
		LOG_DBG("SLEEPN GPIO set HIGH (wake mode)");
	}

	/* Configure DRVEN GPIO (used in stepper_enable() - Step 3 of power-up sequence) */
	if (cfg->has_drven) {
		LOG_INF("Configuring DRVEN GPIO: port=%s pin=%d",
		        cfg->drven_gpio.port->name, cfg->drven_gpio.pin);
		if (!gpio_is_ready_dt(&cfg->drven_gpio)) {
			LOG_ERR("DRVEN GPIO port not ready");
			return -ENODEV;
		}
		ret = gpio_pin_configure_dt(&cfg->drven_gpio, GPIO_OUTPUT_INACTIVE);
		if (ret) {
			LOG_ERR("Failed to configure DRVEN GPIO: %d", ret);
			return ret;
		}
		LOG_INF("DRVEN GPIO configured successfully (initially LOW)");
	} else {
		LOG_WRN("DRVEN GPIO not specified in device tree!");
	}

	/* Wait 10ms after GPIO setup for chip to be ready (Step 2 of datasheet power-up sequence) */
	k_msleep(10);
	LOG_DBG("Waited 10ms for chip initialization");

	/* Detect and configure clock source */
	uint32_t ioin_val;
	ret = tmc522x_read(dev, TMC522X_REG_IOIN, &ioin_val);

	if (ret == 0) {
		uint32_t ext_clk = TMC522X_FIELD_GET(TMC522X_IOIN_EXT_CLK_MASK, ioin_val);

		switch (ext_clk) {
		case 0:
			/* Internal clock */
			data->clock = TMC522X_INTERNAL_CLOCK_HZ;
			LOG_INF("Using internal clock: %u Hz", data->clock);
			break;
		case 1:
			/* External clock - use value from devicetree */
			if (data->clock == 0) {
				LOG_WRN("External clock detected but clock-frequency not set in DT, using default");
				data->clock = TMC522X_INTERNAL_CLOCK_HZ;
			}
			LOG_INF("Using external clock: %u Hz", data->clock);
			break;
		case 3:
			/* Quiescent mode */
			data->clock = TMC522X_QUIESCENT_CLOCK_HZ;
			LOG_INF("Using quiescent clock: %u Hz", data->clock);
			break;
		default:
			LOG_WRN("Unknown EXT_CLK value: %u, using internal clock", ext_clk);
			data->clock = TMC522X_INTERNAL_CLOCK_HZ;
			break;
		}
	} else {
		LOG_WRN("Failed to read IOIN register, using clock from DT: %u Hz", data->clock);
	}

	/* Configure hardware registers per datasheet power-up sequence */
	/* Initialize hardware configuration from devicetree */
	data->fs_sense = cfg->default_fs_sense;
	data->fs_gain = cfg->default_fs_gain;
	data->global_scaler_a = cfg->default_global_scaler;
	data->global_scaler_b = cfg->default_global_scaler;
	data->toff = cfg->default_toff;
	data->tbl = cfg->default_tbl;
	data->hstrt_tfd210 = cfg->default_hstrt;
	data->hend_offset = cfg->default_hend;
	data->vhighchm = 0;           /* High velocity chopper mode disabled */
	data->vhighfs = 0;            /* High velocity fullstep disabled */
	data->pwm_freewheel = cfg->default_pwm_freewheel;
	data->pwm_freq = cfg->default_pwm_freq;
	data->pwm_dis_reg_stat = 0;   /* PWM regulation in standstill enabled */
	data->sgt = cfg->default_sgt;
	data->sgt4 = cfg->default_sgt4;

	/* Initialize CoolStep parameters from devicetree */
	data->semin = cfg->default_semin;
	data->semax = cfg->default_semax;
	data->seup = cfg->default_seup;
	data->sedn = cfg->default_sedn;
	data->seimin = cfg->default_seimin;

	/* Write hardware configuration registers (GCONF, DRV_CONF, IHOLD_IRUN, TPOWERDOWN, CHOPCONF) */
	ret = tmc522x_configure_hardware(dev);
	if (ret) {
		LOG_ERR("Hardware configuration failed: %d", ret);
		return ret;
	}

	/* Configure DIAG pins for interrupt support (optional) */
	uint32_t diag_conf = 0;

	/* Configure DIAG0 and DIAG1 as active-low push-pull outputs */
	diag_conf |= TMC522X_DIAG1_INVPP_MASK | TMC522X_DIAG1_NOD_PP_MASK;
	diag_conf |= TMC522X_DIAG0_INVPP_MASK | TMC522X_DIAG0_NOD_PP_MASK;

	/* Configure DIAG0 for StallGuard events */
	diag_conf |= TMC522X_DIAG0_STALL_MASK;

	/* Configure DIAG1 for position reached events */
	diag_conf |= TMC522X_DIAG1_POS_REACH_MASK;

	ret = tmc522x_reg_write(dev, TMC522X_REG_DIAG_CONF, diag_conf);
	if (ret) {
		LOG_ERR("Failed to configure DIAG pins: %d", ret);
		return ret;
	}
	LOG_DBG("DIAG pins configured (DIAG0=StallGuard, DIAG1=Position)");

	/* Initialize work items for interrupt handling */
	k_work_init(&data->stallguard_work, tmc522x_stallguard_work_handler);
	k_work_init(&data->pos_reached_work, tmc522x_pos_reached_work_handler);

	/* Setup StallGuard interrupt (DIAG0) if GPIO is configured */
	if (cfg->has_stallguard_irq) {
		if (!gpio_is_ready_dt(&cfg->stallguard_gpio)) {
			LOG_ERR("StallGuard GPIO not ready");
			return -ENODEV;
		}

		ret = gpio_pin_configure_dt(&cfg->stallguard_gpio, GPIO_INPUT);
		if (ret) {
			LOG_ERR("Failed to configure StallGuard GPIO: %d", ret);
			return ret;
		}

		ret = gpio_pin_interrupt_configure_dt(&cfg->stallguard_gpio,
						       GPIO_INT_EDGE_FALLING);
		if (ret) {
			LOG_ERR("Failed to configure StallGuard interrupt: %d", ret);
			return ret;
		}

		gpio_init_callback(&data->stallguard_cb,
				   tmc522x_stallguard_gpio_callback,
				   BIT(cfg->stallguard_gpio.pin));

		ret = gpio_add_callback(cfg->stallguard_gpio.port, &data->stallguard_cb);
		if (ret) {
			LOG_ERR("Failed to add StallGuard GPIO callback: %d", ret);
			return ret;
		}

		LOG_INF("StallGuard interrupt configured on DIAG0 (disabled by default)");
	}

	/* Setup Position Reached interrupt (DIAG1) if GPIO is configured */
	if (cfg->has_pos_reached_irq) {
		if (!gpio_is_ready_dt(&cfg->pos_reached_gpio)) {
			LOG_ERR("Position Reached GPIO not ready");
			return -ENODEV;
		}

		ret = gpio_pin_configure_dt(&cfg->pos_reached_gpio, GPIO_INPUT);
		if (ret) {
			LOG_ERR("Failed to configure Position Reached GPIO: %d", ret);
			return ret;
		}

		ret = gpio_pin_interrupt_configure_dt(&cfg->pos_reached_gpio,
						       GPIO_INT_EDGE_FALLING);
		if (ret) {
			LOG_ERR("Failed to configure Position Reached interrupt: %d", ret);
			return ret;
		}

		gpio_init_callback(&data->pos_reached_cb,
				   tmc522x_pos_reached_gpio_callback,
				   BIT(cfg->pos_reached_gpio.pin));

		ret = gpio_add_callback(cfg->pos_reached_gpio.port, &data->pos_reached_cb);
		if (ret) {
			LOG_ERR("Failed to add Position Reached GPIO callback: %d", ret);
			return ret;
		}

		LOG_INF("Position Reached interrupt configured on DIAG1 (disabled by default)");
	}

	LOG_INF("Hardware configuration complete");

	/* Write motion parameters (VMAX, AMAX, VSTART, etc.) once during initialization */
	ret = tmc522x_write_motion_params(dev);
	if (ret) {
		LOG_ERR("Failed to write motion parameters: %d", ret);
		return ret;
	}

	/* Note: Motor is disabled by default per datasheet power-up sequence */
	/* Use stepper_enable() to enable driver (sets GCONF[9]=0 and DRV_EN=1) */
	/* Note: Chopper mode can be configured via tmc522x_stepper_set_chopper_mode() */

	LOG_INF("TMC522X init OK - driver disabled, ready for use");
	return 0;
}

/*
 * Optional shutdown function for graceful driver cleanup
 * This is called when the device is being removed or system is shutting down
 * Currently unused (NULL in DEVICE_DT_INST_DEFINE) but available if needed
 */
__maybe_unused
static int tmc522x_shutdown(const struct device *dev)
{
	const struct tmc522x_config *cfg = dev->config;
	struct tmc522x_data *data = dev->data;
	int ret;

	LOG_INF("Shutting down TMC522X");

	/* Disable motor if enabled */
	if (data->enabled) {
		data->enabled = false;
		LOG_DBG("Motor disabled during shutdown");
	}

	/* Disable driver outputs via DRVEN GPIO */
	if (cfg->has_drven) {
		ret = gpio_pin_set_dt(&cfg->drven_gpio, 0);
		if (ret) {
			LOG_WRN("Failed to disable DRVEN GPIO during shutdown: %d", ret);
		}
	}

	/* Enter sleep mode via SLEEPN pin for lowest power consumption */
	if (cfg->has_sleepn) {
		ret = gpio_pin_set_dt(&cfg->sleepn_gpio, 0);
		if (ret) {
			LOG_WRN("Failed to set SLEEPN LOW during shutdown: %d", ret);
		} else {
			LOG_DBG("SLEEPN set LOW (sleep mode for lowest power consumption)");
		}
	}

	LOG_INF("TMC522X shutdown complete");
	return 0;
}

/* Helper macros for bus initialization */
#if DT_ANY_INST_ON_BUS_STATUS_OKAY(spi)
#define TMC522X_BUS_INIT_SPI(inst)                                                                 \
	.bus = {.spi = SPI_DT_SPEC_INST_GET(inst,                                                  \
		(SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB | SPI_MODE_CPOL | SPI_MODE_CPHA |          \
		 SPI_WORD_SET(8)))},                                                               \
	.bus_io = &tmc522x_bus_io_spi,
#else
#define TMC522X_BUS_INIT_SPI(inst)
#endif

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c)
#define TMC522X_BUS_INIT_I2C(inst)                                                                 \
	.bus = {.i2c = I2C_DT_SPEC_INST_GET(inst)},                                                \
	.bus_io = &tmc522x_bus_io_i2c,
#else
#define TMC522X_BUS_INIT_I2C(inst)
#endif

#define TMC522X_DEFINE(inst)                                                                       \
	static struct tmc522x_data tmc522x_data_##inst;                                            \
	static const struct tmc522x_config tmc522x_config_##inst = {                               \
		COND_CODE_1(DT_INST_ON_BUS(inst, spi), (TMC522X_BUS_INIT_SPI(inst)),              \
			    (TMC522X_BUS_INIT_I2C(inst)))                                          \
		.drven_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, drven_gpios, {0}),                   \
		.sleepn_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, sleepn_gpios, {0}),                 \
		.stallguard_gpio = GPIO_DT_SPEC_INST_GET_BY_IDX_OR(inst, interrupt_gpios, 0, {0}), \
		.pos_reached_gpio = GPIO_DT_SPEC_INST_GET_BY_IDX_OR(inst, interrupt_gpios, 1, {0}), \
		.has_drven = DT_INST_NODE_HAS_PROP(inst, drven_gpios),                             \
		.has_sleepn = DT_INST_NODE_HAS_PROP(inst, sleepn_gpios),                           \
		.has_stallguard_irq = DT_INST_PROP_LEN_OR(inst, interrupt_gpios, 0) > 0,          \
		.has_pos_reached_irq = DT_INST_PROP_LEN_OR(inst, interrupt_gpios, 0) > 1,         \
		.enable_stealthchop = (DT_INST_ENUM_IDX(inst, chopper_mode) == 1),                \
		.clock_frequency = DT_INST_PROP(inst, clock_frequency),                            \
		.default_micro_step_res = DT_INST_PROP(inst, micro_step_res),                      \
		/* Ramp parameters */                                                              \
		.default_vstart = DT_INST_PROP(inst, vstart),                                      \
		.default_a1 = DT_INST_PROP(inst, a1),                                              \
		.default_v1 = DT_INST_PROP(inst, v1),                                              \
		.default_a2 = DT_INST_PROP(inst, a2),                                              \
		.default_v2 = DT_INST_PROP(inst, v2),                                              \
		.default_amax = DT_INST_PROP(inst, amax),                                          \
		.default_vmax = DT_INST_PROP(inst, vmax),                                          \
		.default_dmax = DT_INST_PROP(inst, dmax),                                          \
		.default_d2 = DT_INST_PROP(inst, d2),                                              \
		.default_d1 = DT_INST_PROP(inst, d1),                                              \
		.default_vstop = DT_INST_PROP(inst, vstop),                                        \
		/* Current control */                                                              \
		.default_ihold = DT_INST_PROP(inst, ihold),                                        \
		.default_irun = DT_INST_PROP(inst, irun),                                          \
		.default_iholddelay = DT_INST_PROP(inst, iholddelay),                              \
		.default_irundelay = DT_INST_PROP(inst, irundelay),                                \
		.default_tpowerdown = DT_INST_PROP(inst, tpowerdown),                              \
		/* Driver configuration */                                                         \
		.default_fs_gain = DT_INST_PROP(inst, fs_gain),                                    \
		.default_fs_sense = DT_INST_PROP(inst, fs_sense),                                  \
		.default_global_scaler = DT_INST_PROP(inst, global_scaler),                        \
		/* Chopper configuration */                                                        \
		.default_toff = DT_INST_PROP(inst, toff),                                          \
		.default_tbl = DT_INST_PROP(inst, tbl),                                            \
		.default_hstrt = DT_INST_PROP(inst, hstrt),                                        \
		.default_hend = DT_INST_PROP(inst, hend),                                          \
		/* PWM configuration */                                                            \
		.default_pwm_freewheel = DT_INST_PROP(inst, pwm_freewheel),                        \
		.default_pwm_freq = DT_INST_PROP(inst, pwm_freq),                                  \
		/* StallGuard configuration */                                                     \
		.default_sgt = DT_INST_PROP(inst, sgt),                                            \
		.default_sgt4 = DT_INST_PROP(inst, sgt4),                                          \
		/* CoolStep configuration */                                                       \
		.default_semin = DT_INST_PROP(inst, semin),                                        \
		.default_semax = DT_INST_PROP(inst, semax),                                        \
		.default_seup = DT_INST_PROP(inst, seup),                                          \
		.default_sedn = DT_INST_PROP(inst, sedn),                                          \
		.default_seimin = DT_INST_PROP(inst, seimin),                                      \
		/* Motor physical parameters */                                                    \
		.default_step_angle_millidegrees = DT_INST_PROP(inst, step_angle_millidegrees),    \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(inst, tmc522x_init, NULL, &tmc522x_data_##inst,                      \
			      &tmc522x_config_##inst, POST_KERNEL,                                 \
			      CONFIG_STEPPER_INIT_PRIORITY, &tmc522x_stepper_api);

DT_INST_FOREACH_STATUS_OKAY(TMC522X_DEFINE)

