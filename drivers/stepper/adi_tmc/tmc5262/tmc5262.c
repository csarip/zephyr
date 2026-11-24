/*
 * SPDX-FileCopyrightText: Copyright (c) 2026 Analog Devices, Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Unified chip (parent) driver for the ADI TMC5262, TMC2262 and TMC3241 stepper
 * controller / driver ICs. This file holds the shared core: SPI access through the
 * adi_tmc bus I/O interface, reset, PLL start-up, presence check, configuration
 * cache, status/diagnostics, events and power management. The Zephyr stepper API
 * is implemented by the child devices (tmc5262_stepper_driver.c and
 * tmc5262_stepper_ctrl.c).
 */

#include <stdlib.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/pm/device.h>
#include <zephyr/sys/util.h>

#include <adi_tmc_bus.h>
#include <adi_tmc_spi.h>
#include <adi_tmc5xxx_common.h>

#include "tmc5262.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(tmc5262, CONFIG_STEPPER_ADI_TMC5262_LOG_LEVEL);

/* -------------------------------------------------------------------------- */
/* Bus I/O (FR-BUS-001..004, FR-BUS-007)                                       */
/* -------------------------------------------------------------------------- */

static int tmc5262_bus_check_spi(const union tmc_bus *bus, uint8_t comm_type)
{
	if (comm_type != TMC_COMM_SPI) {
		return -ENOTSUP;
	}

	if (!spi_is_ready_dt(&bus->spi)) {
		LOG_ERR_DEVICE_NOT_READY(bus->spi.bus);
		return -ENODEV;
	}

	return 0;
}

static void tmc5262_capture_spi_status(const struct device *dev, uint8_t status)
{
	const struct tmc5262_config *config = dev->config;
	struct tmc5262_data *data = dev->data;

	/* Bits 7..4 are only valid on variants that have a ramp generator */
	if ((config->variant->caps & TMC5262_CAP_RAMP_GEN) == 0U) {
		status &= (uint8_t)~TMC5262_SPI_STATUS_RAMP_BITS;
	}

	data->spi_status = status;
}

static int tmc5262_reg_read_spi(const struct device *dev, uint8_t reg, uint32_t *val)
{
	const struct tmc5262_config *config = dev->config;
	uint8_t status = 0U;
	int err;

	err = tmc_spi_read_register_status(&config->bus.spi, TMC5XXX_ADDRESS_MASK, reg, val,
					   &status);
	if (err < 0) {
		return -EIO;
	}

	tmc5262_capture_spi_status(dev, status);

	return 0;
}

static int tmc5262_reg_write_spi(const struct device *dev, uint8_t reg, uint32_t val)
{
	const struct tmc5262_config *config = dev->config;
	uint8_t status = 0U;
	int err;

	err = tmc_spi_write_register_status(&config->bus.spi, TMC5XXX_WRITE_BIT, reg, val, &status);
	if (err < 0) {
		return -EIO;
	}

	tmc5262_capture_spi_status(dev, status);

	return 0;
}

static const struct tmc_bus_io tmc5262_spi_bus_io = {
	.check = tmc5262_bus_check_spi,
	.read = tmc5262_reg_read_spi,
	.write = tmc5262_reg_write_spi,
};

/* -------------------------------------------------------------------------- */
/* Serialized register access and configuration cache (FR-BUS-005, FR-BUS-006) */
/* -------------------------------------------------------------------------- */

void tmc5262_lock(const struct device *dev)
{
	struct tmc5262_data *data = dev->data;

	k_mutex_lock(&data->lock, K_FOREVER);
}

void tmc5262_unlock(const struct device *dev)
{
	struct tmc5262_data *data = dev->data;

	k_mutex_unlock(&data->lock);
}

int tmc5262_require(const struct device *dev, uint32_t caps)
{
	const struct tmc5262_config *config = dev->config;

	return ((config->variant->caps & caps) == caps) ? 0 : -ENOTSUP;
}

static int tmc5262_write_unchecked(const struct device *dev, uint8_t reg, uint32_t val)
{
	const struct tmc5262_config *config = dev->config;
	struct tmc5262_data *data = dev->data;
	int err;

	k_mutex_lock(&data->lock, K_FOREVER);
	err = config->bus_io->write(dev, reg, val);
	k_mutex_unlock(&data->lock);

	if (err < 0) {
		LOG_ERR("%s: write of register 0x%02x failed (%d)", config->variant->name, reg,
			err);
	}

	return err;
}

static int tmc5262_read_unchecked(const struct device *dev, uint8_t reg, uint32_t *val)
{
	const struct tmc5262_config *config = dev->config;
	struct tmc5262_data *data = dev->data;
	int err;

	k_mutex_lock(&data->lock, K_FOREVER);
	err = config->bus_io->read(dev, reg, val);
	k_mutex_unlock(&data->lock);

	if (err < 0) {
		LOG_ERR("%s: read of register 0x%02x failed (%d)", config->variant->name, reg, err);
	}

	return err;
}

int tmc5262_write(const struct device *dev, uint8_t reg, uint32_t val)
{
	const struct tmc5262_config *config = dev->config;
	struct tmc5262_data *data = dev->data;

	if (!tmc5262_variant_has_reg(config->variant, reg)) {
		return -EINVAL;
	}

	if (data->asleep) {
		return TMC5262_ERR_ASLEEP;
	}

	return tmc5262_write_unchecked(dev, reg, val);
}

int tmc5262_read(const struct device *dev, uint8_t reg, uint32_t *val)
{
	const struct tmc5262_config *config = dev->config;
	struct tmc5262_data *data = dev->data;

	if (!tmc5262_variant_has_reg(config->variant, reg)) {
		return -EINVAL;
	}

	if (data->asleep) {
		return TMC5262_ERR_ASLEEP;
	}

	return tmc5262_read_unchecked(dev, reg, val);
}

static uint32_t *tmc5262_shadow_slot(struct tmc5262_shadow *s, uint8_t reg)
{
	switch (reg) {
	case TMC5262_REG_GCONF:
		return &s->gconf;
	case TMC5262_REG_DO_CONF:
		return &s->do_conf;
	case TMC5262_REG_DRV_CONF:
		return &s->drv_conf;
	case TMC5262_REG_IHOLD_IRUN:
		return &s->ihold_irun;
	case TMC5262_REG_TPOWERDOWN:
		return &s->tpowerdown;
	case TMC5262_REG_CHOPCONF:
		return &s->chopconf;
	case TMC5262_REG_COOLCONF:
		return &s->coolconf;
	case TMC5262_REG_PWMCONF:
		return &s->pwmconf;
	case TMC5262_REG_TPWMTHRS:
		return &s->tpwmthrs;
	case TMC5262_REG_TCOOLTHRS:
		return &s->tcoolthrs;
	case TMC5262_REG_THIGH:
		return &s->thigh;
	case TMC5262_REG_T_RCOIL_MEAS:
		return &s->t_rcoil_meas;
	case TMC5262_REG_COIL_INDUCT:
		return &s->coil_induct;
	case TMC5262_REG_SW_MODE:
		return &s->sw_mode;
	case TMC5262_REG_ENCMODE:
		return &s->encmode;
	case TMC5262_REG_ENC_CONST:
		return &s->enc_const;
	case TMC5262_REG_UDC_CONF:
		return &s->udc_conf;
	case TMC5262_REG_TUDCSTEP:
		return &s->tudcstep;
	case TMC5262_REG_OTW_OV_VTH:
		return &s->otw_ov_vth;
	case TMC5262_REG_VIRTUAL_STOP_L:
		return &s->virtual_stop_l;
	case TMC5262_REG_VIRTUAL_STOP_R:
		return &s->virtual_stop_r;
	case TMC5262_REG_X_COMPARE:
		return &s->x_compare;
	case TMC5262_REG_X_COMPARE_REPEAT:
		return &s->x_compare_repeat;
	case TMC5262_REG_R_COIL_USER:
		return &s->r_coil_user;
	case TMC5262_REG_CURRENT_PI_REG:
		return &s->current_pi;
	case TMC5262_REG_ANGLE_PI_REG:
		return &s->angle_pi;
	case TMC5262_REG_CUR_ANGLE_LIMIT:
		return &s->cur_angle_limit;
	case TMC5262_REG_ANGLE_LOWER_LIMIT:
		return &s->angle_lower_limit;
	case TMC5262_REG_SGP_CONF:
		return &s->sgp_conf;
	case TMC5262_REG_TSGP_LOW_VEL_THRS:
		return &s->tsgp_low_vel_thrs;
	case TMC5262_REG_COOLSTEPPLUS_CONF:
		return &s->coolplus_conf;
	case TMC5262_REG_COOLSTEPPLUS_PI_REG:
		return &s->coolplus_pi_reg;
	case TMC5262_REG_COOLSTEPPLUS_PI_DOWN:
		return &s->coolplus_pi_down;
	case TMC5262_REG_COOLSTEPPLUS_RESERVE_CONF:
		return &s->coolplus_reserve;
	default:
		return NULL;
	}
}

/* CHOPCONF as written to the device: without a DRV_ENN pin TOFF = 0 keeps the driver off */
static uint32_t tmc5262_chopconf_for_device(const struct device *dev)
{
	const struct tmc5262_config *config = dev->config;
	struct tmc5262_data *data = dev->data;
	uint32_t chopconf = data->shadow.chopconf;

	if (config->drv_enn_gpio.port == NULL && !data->power_stage_on) {
		chopconf &= ~TMC5262_CHOPCONF_TOFF;
	}

	return chopconf;
}

int tmc5262_cfg_update(const struct device *dev, uint8_t reg, uint32_t mask, uint32_t val)
{
	const struct tmc5262_config *config = dev->config;
	struct tmc5262_data *data = dev->data;
	uint32_t *slot = tmc5262_shadow_slot(&data->shadow, reg);
	uint32_t newval;
	int err;

	if (slot == NULL || !tmc5262_variant_has_reg(config->variant, reg)) {
		return -EINVAL;
	}

	/* Reject CHOPCONF bits that do not exist on the variant */
	if (reg == TMC5262_REG_CHOPCONF && (val & mask & ~config->variant->chopconf_mask) != 0U) {
		return -EINVAL;
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	if (data->asleep) {
		k_mutex_unlock(&data->lock);
		return TMC5262_ERR_ASLEEP;
	}

	newval = (*slot & ~mask) | (val & mask);

	if (reg == TMC5262_REG_CHOPCONF) {
		uint32_t old = *slot;

		*slot = newval;
		err = tmc5262_write_unchecked(dev, reg, tmc5262_chopconf_for_device(dev));
		if (err < 0) {
			*slot = old;
		}
	} else {
		err = tmc5262_write_unchecked(dev, reg, newval);
		if (err == 0) {
			*slot = newval;
		}
	}

	k_mutex_unlock(&data->lock);

	return err;
}

int tmc5262_cfg_set(const struct device *dev, uint8_t reg, uint32_t val)
{
	return tmc5262_cfg_update(dev, reg, UINT32_MAX, val);
}

struct tmc5262_reg_val {
	uint8_t reg;
	uint32_t val;
};

static int tmc5262_write_list(const struct device *dev, const struct tmc5262_reg_val *list,
			      size_t count)
{
	int err;

	for (size_t i = 0; i < count; i++) {
		err = tmc5262_write_unchecked(dev, list[i].reg, list[i].val);
		if (err < 0) {
			return err;
		}
	}

	return 0;
}

/* Write the cached register image to the device in a safe order */
static int tmc5262_apply_config(const struct device *dev)
{
	const struct tmc5262_config *config = dev->config;
	const struct tmc5262_variant *variant = config->variant;
	struct tmc5262_data *data = dev->data;
	const struct tmc5262_shadow *s = &data->shadow;
	const struct tmc5262_reg_val common[] = {
		{TMC5262_REG_DRV_CONF, s->drv_conf},
		{TMC5262_REG_CHOPCONF, tmc5262_chopconf_for_device(dev)},
		{TMC5262_REG_IHOLD_IRUN, s->ihold_irun},
		{TMC5262_REG_TPOWERDOWN, s->tpowerdown},
		{TMC5262_REG_GCONF, s->gconf},
		{TMC5262_REG_DO_CONF, s->do_conf},
		{TMC5262_REG_TCOOLTHRS, s->tcoolthrs},
		{TMC5262_REG_THIGH, s->thigh},
		{TMC5262_REG_COOLCONF, s->coolconf},
		{TMC5262_REG_ENC_CONST, s->enc_const},
		{TMC5262_REG_ENCMODE, s->encmode},
		{TMC5262_REG_OTW_OV_VTH, s->otw_ov_vth},
	};
	const struct tmc5262_reg_val stealth[] = {
		{TMC5262_REG_PWMCONF, s->pwmconf},
		{TMC5262_REG_TPWMTHRS, s->tpwmthrs},
		{TMC5262_REG_CURRENT_PI_REG, s->current_pi},
		{TMC5262_REG_ANGLE_PI_REG, s->angle_pi},
		{TMC5262_REG_CUR_ANGLE_LIMIT, s->cur_angle_limit},
		{TMC5262_REG_ANGLE_LOWER_LIMIT, s->angle_lower_limit},
	};
	const struct tmc5262_reg_val sgplus[] = {
		{TMC5262_REG_TSGP_LOW_VEL_THRS, s->tsgp_low_vel_thrs},
		{TMC5262_REG_SGP_CONF, s->sgp_conf},
	};
	const struct tmc5262_reg_val coolplus[] = {
		{TMC5262_REG_COOLSTEPPLUS_PI_REG, s->coolplus_pi_reg},
		{TMC5262_REG_COOLSTEPPLUS_PI_DOWN, s->coolplus_pi_down},
		{TMC5262_REG_COOLSTEPPLUS_RESERVE_CONF, s->coolplus_reserve},
		{TMC5262_REG_COOLSTEPPLUS_CONF, s->coolplus_conf},
	};
	const struct tmc5262_reg_val poscmp[] = {
		{TMC5262_REG_X_COMPARE_REPEAT, s->x_compare_repeat},
		{TMC5262_REG_X_COMPARE, s->x_compare},
	};
	const struct tmc5262_reg_val coil[] = {
		{TMC5262_REG_T_RCOIL_MEAS, s->t_rcoil_meas},
		{TMC5262_REG_COIL_INDUCT, s->coil_induct},
		{TMC5262_REG_R_COIL_USER, s->r_coil_user},
	};
	const struct tmc5262_reg_val ref[] = {
		{TMC5262_REG_VIRTUAL_STOP_L, s->virtual_stop_l},
		{TMC5262_REG_VIRTUAL_STOP_R, s->virtual_stop_r},
		{TMC5262_REG_SW_MODE, s->sw_mode},
	};
	const struct tmc5262_reg_val udc[] = {
		{TMC5262_REG_TUDCSTEP, s->tudcstep},
		{TMC5262_REG_UDC_CONF, s->udc_conf},
	};
	int err;

	err = tmc5262_write_list(dev, common, ARRAY_SIZE(common));
	if (err < 0) {
		return err;
	}

	if ((variant->caps & TMC5262_CAP_STEALTHCHOP) != 0U) {
		err = tmc5262_write_list(dev, stealth, ARRAY_SIZE(stealth));
		if (err < 0) {
			return err;
		}
	}

	if ((variant->caps & TMC5262_CAP_STALLGUARD_PLUS) != 0U) {
		err = tmc5262_write_list(dev, sgplus, ARRAY_SIZE(sgplus));
		if (err < 0) {
			return err;
		}
	}

	if ((variant->caps & TMC5262_CAP_COOLSTEP_PLUS) != 0U) {
		err = tmc5262_write_list(dev, coolplus, ARRAY_SIZE(coolplus));
		if (err < 0) {
			return err;
		}
	}

	if ((variant->caps & TMC5262_CAP_POS_COMPARE) != 0U) {
		err = tmc5262_write_list(dev, poscmp, ARRAY_SIZE(poscmp));
		if (err < 0) {
			return err;
		}
	}

	if ((variant->caps & TMC5262_CAP_COIL_MEAS) != 0U) {
		err = tmc5262_write_list(dev, coil, ARRAY_SIZE(coil));
		if (err < 0) {
			return err;
		}
	}

	if ((variant->caps & TMC5262_CAP_REF_SWITCH) != 0U) {
		err = tmc5262_write_list(dev, ref, ARRAY_SIZE(ref));
		if (err < 0) {
			return err;
		}
	}

	if ((variant->caps & TMC5262_CAP_UDCSTEP) != 0U) {
		err = tmc5262_write_list(dev, udc, ARRAY_SIZE(udc));
		if (err < 0) {
			return err;
		}
	}

	return 0;
}

/* -------------------------------------------------------------------------- */
/* Unit conversion (FR-MOTION-008)                                             */
/* -------------------------------------------------------------------------- */

uint32_t tmc5262_velocity_to_reg(uint32_t microsteps_per_s)
{
	return tmc5xxx_calculate_velocity_from_hz_to_fclk(microsteps_per_s, TMC5262_FCLK_HZ);
}

int32_t tmc5262_velocity_from_reg(int32_t reg)
{
	return (int32_t)(((int64_t)reg * TMC5262_FCLK_HZ) >> TMC5XXX_CLOCK_FREQ_SHIFT);
}

int tmc5262_accel_to_reg(uint32_t microsteps_per_s2, uint32_t *reg)
{
	/*
	 * a[reg] = a[Hz/s] * 2^41 / fCLK^2 (datasheet), with fCLK = 16 MHz = 2^10 * 5^6 this is
	 * a * 2^21 / 5^12. The product fits 64 bit for every 32 bit input.
	 */
	uint64_t val;

	BUILD_ASSERT(TMC5262_FCLK_HZ == 16000000U, "the accel conversion assumes fCLK = 16 MHz");

	val = DIV_ROUND_CLOSEST((uint64_t)microsteps_per_s2 << 21, 244140625ULL);
	if (val > TMC5262_AMAX_MASK) {
		return -EINVAL;
	}

	*reg = (uint32_t)val;

	return 0;
}

/* -------------------------------------------------------------------------- */
/* PLL start-up (FR-INIT-007, FR-INIT-008)                                     */
/* -------------------------------------------------------------------------- */

static uint32_t tmc5262_pll_word(const struct tmc5262_config *config, bool fsm_ena, bool commit)
{
	uint32_t pll = TMC5262_PLL_CLK_SYS_SEL;

	pll |= FIELD_PREP(TMC5262_PLL_CLOCK_DIVIDER, config->clock_divider);

	if (config->ext_clock) {
		pll |= TMC5262_PLL_EXT_NOT_INT;
	}

	if (fsm_ena) {
		pll |= TMC5262_PLL_CLK_FSM_ENA;
	}

	if (commit) {
		pll |= TMC5262_PLL_COMMIT;
	}

	return pll;
}

static int tmc5262_pll_try(const struct device *dev)
{
	const struct tmc5262_config *config = dev->config;
	int64_t deadline;
	uint32_t pll;
	int err;

	/* Select clock, enable the PLL FSM and commit; write 1 to clear a stale tmo flag */
	err = tmc5262_write_unchecked(dev, TMC5262_REG_PLL,
				      tmc5262_pll_word(config, true, true) |
					      TMC5262_PLL_CLK_1M0_TMO);
	if (err < 0) {
		return err;
	}

	/* Poll commit until it clears, bounded by 10 ms */
	deadline = k_uptime_get() + TMC5262_PLL_COMMIT_TIMEOUT_MS;
	do {
		k_msleep(1);
		err = tmc5262_read_unchecked(dev, TMC5262_REG_PLL, &pll);
		if (err < 0) {
			return err;
		}
	} while ((pll & TMC5262_PLL_COMMIT) != 0U && k_uptime_get() < deadline);

	if ((pll & TMC5262_PLL_COMMIT) != 0U) {
		return -ETIMEDOUT;
	}

	/* Clear clk_is_stuck, clk_loss and clk_1m0_tmo (commit stays 0) */
	err = tmc5262_write_unchecked(dev, TMC5262_REG_PLL,
				      tmc5262_pll_word(config, true, false) | TMC5262_PLL_FLAGS);
	if (err < 0) {
		return err;
	}

	err = tmc5262_read_unchecked(dev, TMC5262_REG_PLL, &pll);
	if (err < 0) {
		return err;
	}

	if ((pll & TMC5262_PLL_FLAGS) != 0U) {
		LOG_ERR("%s: PLL fault flags 0x%04x", config->variant->name,
			(uint32_t)(pll & TMC5262_PLL_FLAGS));
		return -EIO;
	}

	return 0;
}

static int tmc5262_pll_start(const struct device *dev)
{
	const struct tmc5262_config *config = dev->config;
	int err = -EIO;

	for (int attempt = 0; attempt < TMC5262_PLL_MAX_ATTEMPTS; attempt++) {
		err = tmc5262_pll_try(dev);
		if (err == 0) {
			return 0;
		}

		LOG_WRN("%s: PLL start attempt %d failed (%d)", config->variant->name, attempt + 1,
			err);

		/* Reset the PLL FSM before the next attempt */
		(void)tmc5262_write_unchecked(dev, TMC5262_REG_PLL,
					      tmc5262_pll_word(config, false, false));
	}

	return err;
}

static int tmc5262_clear_powerup_flags(const struct device *dev);

int tmc5262_restart_pll(const struct device *dev)
{
	const struct tmc5262_config *config = dev->config;
	struct tmc5262_data *data = dev->data;
	bool was_on;
	int err;

	tmc5262_lock(dev);

	if (data->asleep) {
		tmc5262_unlock(dev);
		return TMC5262_ERR_ASLEEP;
	}

	/* While the PLL is down the driver is off: release the power stage, restore it after */
	was_on = data->power_stage_on;
	data->power_stage_on = false;
	if (config->drv_enn_gpio.port != NULL) {
		(void)gpio_pin_set_dt(&config->drv_enn_gpio, 0);
	}

	(void)tmc5262_write_unchecked(dev, TMC5262_REG_PLL, tmc5262_pll_word(config, false, false));

	err = tmc5262_pll_start(dev);
	if (err == 0) {
		/* The restart sets the reset flags: clear them so no spurious fault is reported */
		err = tmc5262_clear_powerup_flags(dev);
	}
	if (err == 0 && was_on) {
		bool present;

		err = tmc5262_fault_present(dev, &present);
		if (err == 0 && present) {
			/* No re-enable over a latched fault, use stepper_enable() */
			LOG_WRN("%s: fault latched, power stage stays off", config->variant->name);
			err = -EIO;
		} else if (err == 0) {
			err = tmc5262_power_stage_enable(dev);
		}
	}

	tmc5262_unlock(dev);

	return err;
}

/* -------------------------------------------------------------------------- */
/* Reset, presence check and (re)initialization (FR-INIT-*, FR-POWER-002)      */
/* -------------------------------------------------------------------------- */

static bool tmc5262_reply_valid(int err, uint32_t val)
{
	return err == 0 && val != 0U && val != UINT32_MAX;
}

static int tmc5262_hw_reset(const struct device *dev)
{
	const struct tmc5262_config *config = dev->config;
	int64_t deadline;
	uint32_t ioin = 0U;
	uint32_t gstat = 0U;
	int err;

	if (config->sleepn_gpio.port == NULL) {
		return 0;
	}

	/* SLEEPN low for > 30 us resets the chip, use at least 50 us */
	err = gpio_pin_set_dt(&config->sleepn_gpio, 0);
	if (err < 0) {
		return err;
	}

	k_busy_wait(TMC5262_RESET_PULSE_US);

	err = gpio_pin_set_dt(&config->sleepn_gpio, 1);
	if (err < 0) {
		return err;
	}

	k_msleep(config->variant->twake_ms);

	/*
	 * The device must answer within 10 ms after tWAKE. A transfer to a silent device
	 * completes without error, so the reply has to be plausible: IOIN or GSTAT neither
	 * all zero nor all ones (GSTAT.reset is set after a reset).
	 */
	deadline = k_uptime_get() + TMC5262_PRESENCE_TIMEOUT_MS;
	do {
		err = tmc5262_read_unchecked(dev, TMC5262_REG_IOIN, &ioin);
		if (tmc5262_reply_valid(err, ioin)) {
			return 0;
		}

		err = tmc5262_read_unchecked(dev, TMC5262_REG_GSTAT, &gstat);
		if (tmc5262_reply_valid(err, gstat)) {
			return 0;
		}

		k_msleep(1);
	} while (k_uptime_get() < deadline);

	return -ETIMEDOUT;
}

/* None of the devices has a chip ID: verify with IOIN and GSTAT (FR-INIT-006) */
static int tmc5262_verify_presence(const struct device *dev, bool expect_reset_flag)
{
	const struct tmc5262_config *config = dev->config;
	uint32_t ioin;
	uint32_t gstat;
	int err;

	err = tmc5262_read_unchecked(dev, TMC5262_REG_IOIN, &ioin);
	if (err < 0) {
		return err;
	}

	err = tmc5262_read_unchecked(dev, TMC5262_REG_GSTAT, &gstat);
	if (err < 0) {
		return err;
	}

	if ((ioin == 0U || ioin == UINT32_MAX) && (gstat == 0U || gstat == UINT32_MAX)) {
		LOG_ERR("%s: no response on SPI (IOIN=0x%08x GSTAT=0x%08x)", config->variant->name,
			ioin, gstat);
		return -EIO;
	}

	if ((ioin & TMC5262_IOIN_EXT_RES_DET) == 0U) {
		LOG_ERR("%s: reference resistor not detected (IOIN=0x%08x)", config->variant->name,
			ioin);
		return -EIO;
	}

	/* A reset sets GSTAT.reset and GSTAT.register_reset */
	if (expect_reset_flag &&
	    (gstat & (TMC5262_GSTAT_RESET | TMC5262_GSTAT_REGISTER_RESET)) == 0U) {
		LOG_WRN("%s: no reset flag in GSTAT after reset (GSTAT=0x%02x)",
			config->variant->name, (uint32_t)gstat);
	}

	LOG_INF("%s (%u-phase): SPI up, silicon revision %u", config->variant->name,
		config->variant->n_phases, (uint32_t)FIELD_GET(TMC5262_IOIN_SILICON_RV, ioin));

	return 0;
}

static int tmc5262_clear_powerup_flags(const struct device *dev)
{
	const struct tmc5262_config *config = dev->config;
	uint32_t gstat;
	int err;

	err = tmc5262_write_unchecked(dev, TMC5262_REG_GSTAT, TMC5262_GSTAT_POWERUP_CLEAR);
	if (err < 0) {
		return err;
	}

	err = tmc5262_read_unchecked(dev, TMC5262_REG_GSTAT, &gstat);
	if (err < 0) {
		return err;
	}

	if ((gstat & TMC5262_GSTAT_POWERUP_CLEAR) != 0U) {
		LOG_WRN("%s: GSTAT flags still set after clear: 0x%02x", config->variant->name,
			(uint32_t)(gstat & TMC5262_GSTAT_POWERUP_CLEAR));
	}

	return 0;
}

/* PLL start, clear power-up flags, write the cached configuration */
static int tmc5262_start_and_configure(const struct device *dev, bool check_presence)
{
	int err;

	if (check_presence) {
		err = tmc5262_verify_presence(dev, true);
		if (err < 0) {
			return err;
		}
	}

	err = tmc5262_pll_start(dev);
	if (err < 0) {
		return err;
	}

	err = tmc5262_clear_powerup_flags(dev);
	if (err < 0) {
		return err;
	}

	return tmc5262_apply_config(dev);
}

/* -------------------------------------------------------------------------- */
/* Power stage, motion state (FR-MOTION-006, FR-INIT-005)                      */
/* -------------------------------------------------------------------------- */

bool tmc5262_ramp_is_step_source(const struct device *dev)
{
	const struct tmc5262_config *config = dev->config;
	struct tmc5262_data *data = dev->data;

	return (config->variant->caps & TMC5262_CAP_RAMP_GEN) != 0U && data->step_ramp_source;
}

int tmc5262_check_standstill(const struct device *dev)
{
	const struct tmc5262_config *config = dev->config;
	uint32_t val;
	int err;

	/* With the ramp generator vzero is the authoritative standstill indication */
	if (tmc5262_ramp_is_step_source(dev)) {
		err = tmc5262_read(dev, TMC5262_REG_RAMP_STAT, &val);
		if (err < 0) {
			return err;
		}

		return (val & TMC5262_RAMP_STAT_VZERO) != 0U ? 0 : -EBUSY;
	}

	err = tmc5262_read(dev, TMC5262_REG_DRV_STATUS, &val);
	if (err < 0) {
		return err;
	}

	return (val & config->variant->status.stst) != 0U ? 0 : -EBUSY;
}

int tmc5262_power_stage_enable(const struct device *dev)
{
	const struct tmc5262_config *config = dev->config;
	struct tmc5262_data *data = dev->data;
	uint32_t ioin;
	int err;

	tmc5262_lock(dev);

	if (config->drv_enn_gpio.port != NULL) {
		err = gpio_pin_set_dt(&config->drv_enn_gpio, 1);
		if (err < 0) {
			goto out;
		}

		/* IOIN.DRV_ENN: 0 = enabled, 1 = disabled. Verify the pin polarity. */
		err = tmc5262_read(dev, TMC5262_REG_IOIN, &ioin);
		if (err == 0 && (ioin & TMC5262_IOIN_DRV_ENN) != 0U) {
			LOG_ERR("%s: DRV_ENN still reads disabled, check drv-enn-gpios polarity",
				config->variant->name);
			(void)gpio_pin_set_dt(&config->drv_enn_gpio, 0);
			err = -EIO;
		}

		if (err == 0) {
			data->power_stage_on = true;
		}
		goto out;
	}

	/* No DRV_ENN pin: TOFF != 0 enables the driver */
	data->power_stage_on = true;
	err = tmc5262_write(dev, TMC5262_REG_CHOPCONF, tmc5262_chopconf_for_device(dev));
	if (err < 0) {
		data->power_stage_on = false;
	}

out:
	tmc5262_unlock(dev);

	return err;
}

int tmc5262_power_stage_disable(const struct device *dev)
{
	const struct tmc5262_config *config = dev->config;
	struct tmc5262_data *data = dev->data;
	int err;

	tmc5262_lock(dev);

	data->power_stage_on = false;

	if (config->drv_enn_gpio.port != NULL) {
		err = gpio_pin_set_dt(&config->drv_enn_gpio, 0);
	} else {
		err = tmc5262_write(dev, TMC5262_REG_CHOPCONF, tmc5262_chopconf_for_device(dev));
	}

	tmc5262_unlock(dev);

	return err;
}

/*
 * Stop of the ramp generator: datasheet "Early Ramp Termination" option c, completed by
 * a final target rewrite inside the zero-wait window.
 *
 * 1. Save TZEROWAIT and set it to its maximum (about 2 s). After the deceleration the
 *    generator then waits at zero velocity instead of driving back to a target that lies
 *    behind the motor.
 * 2. Read XACTUAL and write it to XTARGET (in velocity mode XTARGET is unused, so this is
 *    safe), then switch to positioning mode. The generator decelerates with DMAX / D1 /
 *    D2 and VSTOP. VMAX is not touched.
 * 3. When RAMP_STAT.vzero is set (inside the TZEROWAIT window) read the final XACTUAL, write
 *    it to XTARGET again (no reverse move is possible any more) and restore TZEROWAIT.
 *
 * With wait = false step 3 is done from the event/poll work item.
 */
static int tmc5262_stop_finish_locked(const struct device *dev, bool *done)
{
	struct tmc5262_data *data = dev->data;
	uint32_t ramp_stat;
	uint32_t xactual;
	int err;

	*done = !data->stop_pending;
	if (*done) {
		return 0;
	}

	err = tmc5262_read(dev, TMC5262_REG_RAMP_STAT, &ramp_stat);
	if (err < 0 || (ramp_stat & TMC5262_RAMP_STAT_VZERO) == 0U) {
		return err;
	}

	err = tmc5262_read(dev, TMC5262_REG_XACTUAL, &xactual);
	if (err == 0) {
		err = tmc5262_write(dev, TMC5262_REG_XTARGET, xactual);
	}
	if (err == 0) {
		err = tmc5262_write(dev, TMC5262_REG_TZEROWAIT, data->saved_tzerowait);
	}
	if (err == 0) {
		data->stop_pending = false;
		*done = true;

		/*
		 * Rewriting XTARGET = XACTUAL makes the generator latch event_pos_reached (one ramp
		 * update, fCLK / 512 = 32 us, later). It does not belong to a commanded move: wait
		 * for it and clear it (write 1 to clear).
		 */
		k_busy_wait(TMC5262_POS_EVENT_SETTLE_US);
		(void)tmc5262_write(dev, TMC5262_REG_RAMP_STAT,
				    TMC5262_RAMP_STAT_EVENT_POS_REACHED);
	}

	return err;
}

int tmc5262_ramp_stop_cancel(const struct device *dev)
{
	struct tmc5262_data *data = dev->data;
	int err = 0;

	tmc5262_lock(dev);

	if (data->stop_pending) {
		err = tmc5262_write(dev, TMC5262_REG_TZEROWAIT, data->saved_tzerowait);
		if (err == 0) {
			data->stop_pending = false;
		}
	}

	tmc5262_unlock(dev);

	return err;
}

/* Time needed to decelerate from the actual velocity, from VACTUAL and the slowest D phase */
static uint32_t tmc5262_stop_timeout_ms(const struct device *dev)
{
	static const uint8_t decel_regs[] = {TMC5262_REG_DMAX, TMC5262_REG_D1, TMC5262_REG_D2};
	uint32_t raw;
	uint32_t amin = 0U;
	uint64_t v;
	uint64_t ms;
	int32_t sv;

	if (tmc5262_read(dev, TMC5262_REG_VACTUAL, &raw) < 0) {
		return TMC5262_STOP_TIMEOUT_MAX_MS;
	}

	sv = sign_extend(raw, TMC5262_VACTUAL_SIGN_BIT);
	v = (sv < 0) ? (uint64_t)(-(int64_t)sv) : (uint64_t)sv;

	for (size_t i = 0; i < ARRAY_SIZE(decel_regs); i++) {
		if (tmc5262_read(dev, decel_regs[i], &raw) < 0) {
			return TMC5262_STOP_TIMEOUT_MAX_MS;
		}

		raw &= TMC5262_AMAX_MASK;
		if (raw != 0U && (amin == 0U || raw < amin)) {
			amin = raw;
		}
	}

	if (amin == 0U) {
		return TMC5262_STOP_TIMEOUT_MAX_MS;
	}

	/* t = v / a = |VACTUAL| * 2^17 / (a * fCLK) seconds, x2 margin */
	ms = (2000ULL * v * (1ULL << TMC5XXX_ACCEL_CALC_SHIFT)) /
	     ((uint64_t)amin * TMC5262_FCLK_HZ);
	ms += TMC5262_STOP_TIMEOUT_MIN_MS;

	return (uint32_t)MIN(ms, (uint64_t)TMC5262_STOP_TIMEOUT_MAX_MS);
}

int tmc5262_ramp_stop(const struct device *dev, bool wait)
{
	const struct tmc5262_config *config = dev->config;
	struct tmc5262_data *data = dev->data;
	int64_t deadline = 0;
	uint32_t tzerowait;
	uint32_t xactual;
	bool done = false;
	int err = 0;

	tmc5262_lock(dev);

	if (!tmc5262_ramp_is_step_source(dev)) {
		tmc5262_unlock(dev);
		return 0;
	}

	if (!data->stop_pending) {
		err = tmc5262_read(dev, TMC5262_REG_TZEROWAIT, &tzerowait);
		if (err == 0) {
			data->saved_tzerowait = tzerowait & TMC5262_TZEROWAIT_MASK;
			err = tmc5262_write(dev, TMC5262_REG_TZEROWAIT, TMC5262_TZEROWAIT_MASK);
		}
		if (err == 0) {
			data->stop_pending = true;
		}
	}

	if (err == 0) {
		err = tmc5262_read(dev, TMC5262_REG_XACTUAL, &xactual);
	}
	if (err == 0) {
		err = tmc5262_write(dev, TMC5262_REG_XTARGET, xactual);
	}
	if (err == 0) {
		err = tmc5262_write(dev, TMC5262_REG_RAMPMODE, TMC5262_RAMPMODE_POSITION);
	}
	if (err == 0 && wait) {
		deadline = k_uptime_get() + tmc5262_stop_timeout_ms(dev);
	}

	if (err < 0) {
		(void)tmc5262_ramp_stop_cancel(dev);
		tmc5262_unlock(dev);
		return err;
	}

	tmc5262_unlock(dev);

	if (!wait) {
		/* The work item completes the stop at standstill */
		k_work_reschedule(&data->poll_work, K_MSEC(TMC5262_STOP_POLL_MS));
		return 0;
	}

	do {
		tmc5262_lock(dev);
		err = tmc5262_stop_finish_locked(dev, &done);
		if (err == 0 && done && config->stepper_ctrl != NULL &&
		    tmc5262_stepper_ctrl_consume_stop(config->stepper_ctrl)) {
			/* A commanded stop ends here: report it from the work item */
			atomic_set_bit(&data->pending, TMC5262_PEND_STOPPED);
			k_work_submit(&data->event_work);
		}
		tmc5262_unlock(dev);

		if (err < 0 || done) {
			return err;
		}

		k_msleep(TMC5262_STOP_POLL_MS);
	} while (k_uptime_get() < deadline);

	/* Let the work item finish the stop once the motor is at standstill */
	k_work_reschedule(&data->poll_work, K_MSEC(TMC5262_STOP_POLL_MS));

	return -ETIMEDOUT;
}

int tmc5262_stop_motion(const struct device *dev)
{
	return tmc5262_ramp_stop(dev, true);
}

/* -------------------------------------------------------------------------- */
/* Events (FR-DIAG-003, FR-DIAG-006, FR-MOTION-013, FR-POWER-004)              */
/* -------------------------------------------------------------------------- */

static bool tmc5262_has_do_irq(const struct tmc5262_config *config)
{
	return config->do0_gpio.port != NULL || config->do1_gpio.port != NULL;
}

static uint32_t tmc5262_poll_interval_ms(const struct tmc5262_config *config)
{
	/* With DO interrupts the polling is only a slow fallback (for example for STOPPED) */
	uint32_t interval = CONFIG_STEPPER_ADI_TMC5262_EVENT_POLL_INTERVAL_MS;

	return tmc5262_has_do_irq(config) ? interval * TMC5262_POLL_SLOW_FACTOR : interval;
}

void tmc5262_event_users_add(const struct device *dev)
{
	const struct tmc5262_config *config = dev->config;
	struct tmc5262_data *data = dev->data;

	if (atomic_inc(&data->cb_users) == 0) {
		k_work_reschedule(&data->poll_work, K_MSEC(tmc5262_poll_interval_ms(config)));
	}
}

void tmc5262_event_users_remove(const struct device *dev)
{
	struct tmc5262_data *data = dev->data;

	if (atomic_get(&data->cb_users) > 0) {
		atomic_dec(&data->cb_users);
	}
}

static void tmc5262_notify_driver(const struct device *dev, enum stepper_event event)
{
	const struct tmc5262_config *config = dev->config;

	if (config->stepper_driver != NULL && device_is_ready(config->stepper_driver)) {
		tmc5262_stepper_driver_notify(config->stepper_driver, event);
	}
}

static void tmc5262_notify_ctrl(const struct device *dev, enum stepper_ctrl_event event)
{
	const struct tmc5262_config *config = dev->config;

	if (config->stepper_ctrl != NULL && device_is_ready(config->stepper_ctrl)) {
		tmc5262_stepper_ctrl_notify(config->stepper_ctrl, event);
	}
}

static int tmc5262_reapply_all(const struct device *dev, bool lost)
{
	const struct tmc5262_config *config = dev->config;
	struct tmc5262_data *data = dev->data;
	bool was_on;
	int err;

	tmc5262_lock(dev);

	was_on = data->power_stage_on;
	data->power_stage_on = false;
	data->ev_reported = 0U;
	data->enc_reported = 0U;
	data->stall_active = false;

	if (lost) {
		/* The register bank is back at its defaults: no stop sequence is in progress */
		data->stop_pending = false;
	}

	if (config->drv_enn_gpio.port != NULL) {
		(void)gpio_pin_set_dt(&config->drv_enn_gpio, 0);
	}

	(void)tmc5262_write_unchecked(dev, TMC5262_REG_PLL, tmc5262_pll_word(config, false, false));

	err = tmc5262_start_and_configure(dev, false);

	if (err == 0 && config->stepper_ctrl != NULL && device_is_ready(config->stepper_ctrl)) {
		err = tmc5262_stepper_ctrl_reapply(config->stepper_ctrl, lost);
	}

	if (err == 0 && was_on) {
		err = tmc5262_power_stage_enable(dev);
	}

	tmc5262_unlock(dev);

	return err;
}

/**
 * Check for an unexpected reset (FR-POWER-004). If GSTAT.reset or GSTAT.register_reset is
 * set the cached configuration is re-applied and the application is told. Called before
 * motion commands and on enable.
 */
/*
 * Check for an unexpected reset (FR-POWER-004): GSTAT.reset or GSTAT.register_reset set.
 * The cached configuration is re-applied. The caller holds the device lock.
 */
static int tmc5262_check_reset_locked(const struct device *dev, bool *seen)
{
	uint32_t gstat;
	int err;

	*seen = false;

	err = tmc5262_read(dev, TMC5262_REG_GSTAT, &gstat);
	if (err < 0) {
		return err;
	}

	if ((gstat & (TMC5262_GSTAT_RESET | TMC5262_GSTAT_REGISTER_RESET)) == 0U) {
		return 0;
	}

	LOG_WRN("unexpected reset detected (GSTAT=0x%02x), re-applying configuration",
		(uint32_t)gstat);

	*seen = true;

	/* All registers returned to their defaults, XACTUAL is 0 */
	return tmc5262_reapply_all(dev, true);
}

/*
 * Called before motion commands and on enable. The application is told through the
 * event work item so that all callbacks run in the same (system work queue) context.
 */
int tmc5262_check_reset(const struct device *dev)
{
	struct tmc5262_data *data = dev->data;
	bool seen;
	int err;

	tmc5262_lock(dev);
	err = tmc5262_check_reset_locked(dev, &seen);
	tmc5262_unlock(dev);

	if (seen) {
		atomic_set_bit(&data->pending, TMC5262_PEND_FAULT);
		k_work_submit(&data->event_work);
	}

	return err;
}

/*
 * Event processing, runs from the system work queue only. The state is evaluated with
 * the device lock held, the callbacks are dispatched after releasing it.
 */
/*
 * Event processing, runs from the system work queue only. The state is evaluated with
 * the device lock held, the callbacks are dispatched after releasing it.
 */
static void tmc5262_process_events(const struct device *dev)
{
	const struct tmc5262_config *config = dev->config;
	struct tmc5262_data *data = dev->data;
	bool n_fault = false;
	bool n_stall = false;
	bool n_left = false;
	bool n_right = false;
	bool n_steps = false;
	bool n_stopped = false;
	bool drv_stall = false;
	bool ev_stall = false;
	bool was_pending;
	bool done;
	tmc5262_encoder_event_cb_t enc_cb = NULL;
	void *enc_ud = NULL;
	bool n_enc_dev = false;
	bool n_enc_n = false;
	uint32_t gstat;
	uint32_t drv_status;
	uint32_t ramp_stat;
	uint32_t enc_stat;
	bool reset_seen;
	int err;

	tmc5262_lock(dev);

	if (data->asleep) {
		tmc5262_unlock(dev);
		return;
	}

	if (atomic_test_and_clear_bit(&data->pending, TMC5262_PEND_FAULT)) {
		n_fault = true;
	}

	/* A stop completed by a waiting caller (stepper_disable) */
	if (atomic_test_and_clear_bit(&data->pending, TMC5262_PEND_STOPPED)) {
		n_stopped = true;
	}

	err = tmc5262_read(dev, TMC5262_REG_GSTAT, &gstat);
	if (err == 0) {
		if ((gstat & (TMC5262_GSTAT_RESET | TMC5262_GSTAT_REGISTER_RESET)) != 0U) {
			(void)tmc5262_check_reset_locked(dev, &reset_seen);
			n_fault |= reset_seen;
		} else if ((gstat & TMC5262_GSTAT_DRV_ERR) != 0U && data->power_stage_on) {
			/* Keep the power stage off until the fault is cleared and re-enabled */
			(void)tmc5262_power_stage_disable(dev);
			n_fault = true;
		}
	}

	/* Complete a stop that was started without waiting; STOPPED only at the final standstill */
	was_pending = data->stop_pending;
	if (was_pending) {
		(void)tmc5262_stop_finish_locked(dev, &done);
		if (done && config->stepper_ctrl != NULL &&
		    tmc5262_stepper_ctrl_consume_stop(config->stepper_ctrl)) {
			n_stopped = true;
		}
	}

	err = tmc5262_read(dev, TMC5262_REG_DRV_STATUS, &drv_status);
	if (err == 0) {
		drv_stall = (drv_status & config->variant->status.stallguard) != 0U &&
			    data->power_stage_on;
	}

	if ((config->variant->caps & TMC5262_CAP_RAMP_GEN) != 0U &&
	    tmc5262_ramp_is_step_source(dev) && config->stepper_ctrl != NULL &&
	    tmc5262_stepper_ctrl_wants_events(config->stepper_ctrl) &&
	    tmc5262_read(dev, TMC5262_REG_RAMP_STAT, &ramp_stat) == 0) {
		uint32_t latched = ramp_stat & (TMC5262_RAMP_STAT_EVENT_STOP_L |
						TMC5262_RAMP_STAT_EVENT_STOP_R |
						TMC5262_RAMP_STAT_EVENT_STOP_SG);
		uint32_t fresh = latched & ~data->ev_reported;

		/*
		 * The stop and stall events stay latched in RAMP_STAT until the application
		 * clears them, they are reported once per occurrence. Only the one-shot position
		 * reached event is cleared here (write 1 to clear).
		 */
		data->ev_reported = latched;
		n_left = (fresh & TMC5262_RAMP_STAT_EVENT_STOP_L) != 0U;
		n_right = (fresh & TMC5262_RAMP_STAT_EVENT_STOP_R) != 0U;
		ev_stall = (latched & TMC5262_RAMP_STAT_EVENT_STOP_SG) != 0U;

		/* The position reached event of a stop is not a completed move */
		if ((ramp_stat & TMC5262_RAMP_STAT_EVENT_POS_REACHED) != 0U) {
			(void)tmc5262_write(dev, TMC5262_REG_RAMP_STAT,
					    TMC5262_RAMP_STAT_EVENT_POS_REACHED);
			n_steps = !was_pending;
		}
	}

	/* Encoder deviation (TriCoder step loss) and N events: reported on the rising edge */
	if (data->enc_cb != NULL && tmc5262_read(dev, TMC5262_REG_ENC_STATUS, &enc_stat) == 0) {
		uint32_t bits =
			enc_stat & (TMC5262_ENC_STATUS_DEVIATION_WARN | TMC5262_ENC_STATUS_N_EVENT);
		uint32_t fresh = bits & ~data->enc_reported;

		n_enc_dev = (fresh & TMC5262_ENC_STATUS_DEVIATION_WARN) != 0U;
		n_enc_n = (fresh & TMC5262_ENC_STATUS_N_EVENT) != 0U;
		data->enc_reported = bits;
		enc_cb = data->enc_cb;
		enc_ud = data->enc_cb_user_data;
		if (fresh != 0U) {
			/* Write 1 to clear (a persisting deviation warning stays set) */
			(void)tmc5262_write(dev, TMC5262_REG_ENC_STATUS, fresh);
		}
	}

	/* One stall (StallGuard flag or stop-on-stall event) is reported once */
	if ((drv_stall || ev_stall) && !data->stall_active) {
		n_stall = true;
	}
	data->stall_active = drv_stall || ev_stall;

	tmc5262_unlock(dev);

	if (n_fault) {
		tmc5262_notify_driver(dev, STEPPER_EVENT_FAULT_DETECTED);
	}
	if (n_stall) {
		tmc5262_notify_driver(dev, STEPPER_EVENT_STALL_DETECTED);
	}
	if (n_left) {
		tmc5262_notify_ctrl(dev, STEPPER_CTRL_EVENT_LEFT_END_STOP_DETECTED);
	}
	if (n_right) {
		tmc5262_notify_ctrl(dev, STEPPER_CTRL_EVENT_RIGHT_END_STOP_DETECTED);
	}
	if (n_steps) {
		tmc5262_notify_ctrl(dev, STEPPER_CTRL_EVENT_STEPS_COMPLETED);
	}
	if (n_stopped) {
		tmc5262_notify_ctrl(dev, STEPPER_CTRL_EVENT_STOPPED);
	}
	if (n_enc_dev && enc_cb != NULL) {
		enc_cb(dev, TMC5262_ENCODER_EVENT_DEVIATION, enc_ud);
	}
	if (n_enc_n && enc_cb != NULL) {
		enc_cb(dev, TMC5262_ENCODER_EVENT_N, enc_ud);
	}
}

static void tmc5262_event_work_handler(struct k_work *work)
{
	struct tmc5262_data *data = CONTAINER_OF(work, struct tmc5262_data, event_work);

	tmc5262_process_events(data->dev);
}

static void tmc5262_poll_work_handler(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct tmc5262_data *data = CONTAINER_OF(dwork, struct tmc5262_data, poll_work);
	const struct tmc5262_config *config = data->dev->config;

	tmc5262_process_events(data->dev);

	if (data->asleep) {
		return;
	}

	if (data->stop_pending) {
		/* Fast polling until the stop is complete */
		k_work_reschedule(&data->poll_work, K_MSEC(TMC5262_STOP_POLL_MS));
	} else if (atomic_get(&data->cb_users) > 0) {
		k_work_reschedule(&data->poll_work, K_MSEC(tmc5262_poll_interval_ms(config)));
	}
}

/* No SPI access and no logging here: all work is deferred (FR-DIAG-006, FR-ERR-003) */
static void tmc5262_do0_isr(const struct device *port, struct gpio_callback *cb, uint32_t pins)
{
	struct tmc5262_data *data = CONTAINER_OF(cb, struct tmc5262_data, do0_cb);

	ARG_UNUSED(port);
	ARG_UNUSED(pins);

	k_work_submit(&data->event_work);
}

static void tmc5262_do1_isr(const struct device *port, struct gpio_callback *cb, uint32_t pins)
{
	struct tmc5262_data *data = CONTAINER_OF(cb, struct tmc5262_data, do1_cb);

	ARG_UNUSED(port);
	ARG_UNUSED(pins);

	k_work_submit(&data->event_work);
}

static int tmc5262_setup_do_irq(const struct gpio_dt_spec *spec, struct gpio_callback *cb,
				gpio_callback_handler_t handler)
{
	int err;

	if (spec->port == NULL) {
		return 0;
	}

	err = gpio_pin_configure_dt(spec, GPIO_INPUT);
	if (err < 0) {
		return err;
	}

	gpio_init_callback(cb, handler, BIT(spec->pin));

	err = gpio_add_callback(spec->port, cb);
	if (err < 0) {
		return err;
	}

	err = gpio_pin_interrupt_configure_dt(spec, GPIO_INT_EDGE_TO_ACTIVE);
	if (err < 0) {
		(void)gpio_remove_callback(spec->port, cb);
	}

	return err;
}

static void tmc5262_teardown_do_irq(const struct gpio_dt_spec *spec, struct gpio_callback *cb)
{
	if (spec->port == NULL) {
		return;
	}

	(void)gpio_pin_interrupt_configure_dt(spec, GPIO_INT_DISABLE);
	(void)gpio_remove_callback(spec->port, cb);
}

/* -------------------------------------------------------------------------- */
/* Power management (FR-POWER-001..003)                                        */
/* -------------------------------------------------------------------------- */

int tmc5262_sleep(const struct device *dev)
{
	const struct tmc5262_config *config = dev->config;
	struct tmc5262_data *data = dev->data;
	int err;

	if (config->sleepn_gpio.port == NULL) {
		return -ENOTSUP;
	}

	/* Stop first, without holding the lock during the (up to 10 s) deceleration */
	err = tmc5262_stop_motion(dev);
	if (err < 0) {
		LOG_WRN("%s: motion did not stop cleanly before sleep (%d)", config->variant->name,
			err);
	}

	tmc5262_lock(dev);

	if (data->asleep) {
		tmc5262_unlock(dev);
		return 0;
	}

	err = tmc5262_power_stage_disable(dev);
	if (err < 0) {
		tmc5262_unlock(dev);
		return err;
	}

	(void)k_work_cancel_delayable(&data->poll_work);

	err = gpio_pin_set_dt(&config->sleepn_gpio, 0);
	if (err == 0) {
		data->asleep = true;
		/* The register bank is lost in sleep mode */
		data->stop_pending = false;
	}

	tmc5262_unlock(dev);

	return err;
}

int tmc5262_wake(const struct device *dev)
{
	const struct tmc5262_config *config = dev->config;
	struct tmc5262_data *data = dev->data;
	int err;

	if (config->sleepn_gpio.port == NULL) {
		return -ENOTSUP;
	}

	tmc5262_lock(dev);

	if (!data->asleep) {
		tmc5262_unlock(dev);
		return 0;
	}

	err = gpio_pin_set_dt(&config->sleepn_gpio, 1);
	if (err < 0) {
		goto out;
	}

	k_msleep(config->variant->twake_ms);

	/* All registers returned to their defaults: PLL start-up and re-configuration */
	data->power_stage_on = false;
	data->ev_reported = 0U;
	data->stall_active = false;
	data->stop_pending = false;

	err = tmc5262_start_and_configure(dev, false);
	if (err < 0) {
		/* Stay asleep: SLEEPN low again */
		(void)gpio_pin_set_dt(&config->sleepn_gpio, 0);
		goto out;
	}

	/* The chip is up: from now on the registers may be accessed */
	data->asleep = false;

	if (config->stepper_ctrl != NULL && device_is_ready(config->stepper_ctrl)) {
		err = tmc5262_stepper_ctrl_reapply(config->stepper_ctrl, true);
		if (err < 0) {
			goto out;
		}
	}

	if (atomic_get(&data->cb_users) > 0) {
		k_work_reschedule(&data->poll_work, K_MSEC(tmc5262_poll_interval_ms(config)));
	}

out:
	tmc5262_unlock(dev);

	return err;
}

int tmc5262_reinit(const struct device *dev)
{
	struct tmc5262_data *data = dev->data;
	uint32_t gstat;
	bool lost;
	int err;

	tmc5262_lock(dev);

	if (data->asleep) {
		err = TMC5262_ERR_ASLEEP;
	} else if (data->stop_pending) {
		err = -EBUSY;
	} else {
		err = tmc5262_read(dev, TMC5262_REG_GSTAT, &gstat);
		if (err == 0) {
			/* Only a real reset puts XACTUAL back to 0, else the position is kept */
			lost = (gstat & (TMC5262_GSTAT_RESET | TMC5262_GSTAT_REGISTER_RESET)) != 0U;
			err = tmc5262_reapply_all(dev, lost);
		}
	}

	tmc5262_unlock(dev);

	return err;
}

#ifdef CONFIG_PM_DEVICE
static int tmc5262_pm_action(const struct device *dev, enum pm_device_action action)
{
	const struct tmc5262_config *config = dev->config;

	switch (action) {
	case PM_DEVICE_ACTION_SUSPEND:
		if (config->sleepn_gpio.port == NULL) {
			/* No SLEEPN pin: stop and release the power stage */
			(void)tmc5262_stop_motion(dev);
			return tmc5262_power_stage_disable(dev);
		}
		return tmc5262_sleep(dev);
	case PM_DEVICE_ACTION_RESUME:
		/* Without SLEEPN the chip never slept; the power stage stays off until enabled */
		return config->sleepn_gpio.port == NULL ? 0 : tmc5262_wake(dev);
	default:
		return -ENOTSUP;
	}
}
#endif

/* -------------------------------------------------------------------------- */
/* Public API: variant and register access                                     */
/* -------------------------------------------------------------------------- */

enum tmc5262_variant_id tmc5262_get_variant(const struct device *dev)
{
	const struct tmc5262_config *config = dev->config;

	return config->variant->id;
}

const char *tmc5262_get_variant_name(const struct device *dev)
{
	const struct tmc5262_config *config = dev->config;

	return config->variant->name;
}

int tmc5262_reg_read(const struct device *dev, uint8_t reg, uint32_t *val)
{
	if (val == NULL) {
		return -EINVAL;
	}

	return tmc5262_read(dev, reg, val);
}

int tmc5262_reg_write(const struct device *dev, uint8_t reg, uint32_t val)
{
	/* Registers with a cached image keep the cache coherent */
	struct tmc5262_data *data = dev->data;
	uint32_t *slot = tmc5262_shadow_slot(&data->shadow, reg);

	if (slot != NULL && tmc5262_variant_has_reg(tmc5262_get_variant_data(dev), reg)) {
		return tmc5262_cfg_set(dev, reg, val);
	}

	return tmc5262_write(dev, reg, val);
}

/* -------------------------------------------------------------------------- */
/* Public API: status and diagnostics                                          */
/* -------------------------------------------------------------------------- */

int tmc5262_get_gstat(const struct device *dev, uint32_t *gstat)
{
	if (gstat == NULL) {
		return -EINVAL;
	}

	return tmc5262_read(dev, TMC5262_REG_GSTAT, gstat);
}

int tmc5262_clear_gstat(const struct device *dev, uint32_t flags)
{
	if ((flags & ~TMC5262_GSTAT_ALL) != 0U) {
		return -EINVAL;
	}

	return tmc5262_write(dev, TMC5262_REG_GSTAT, flags);
}

int tmc5262_get_drv_status(const struct device *dev, struct tmc5262_drv_status *status)
{
	const struct tmc5262_config *config = dev->config;
	const struct tmc5262_status_masks *m = &config->variant->status;
	uint32_t raw;
	int err;

	if (status == NULL) {
		return -EINVAL;
	}

	err = tmc5262_read(dev, TMC5262_REG_DRV_STATUS, &raw);
	if (err < 0) {
		return err;
	}

	memset(status, 0, sizeof(*status));

	status->valid = TMC5262_STATUS_SEQ_STOPPED | TMC5262_STATUS_OV;
	if ((config->variant->caps & TMC5262_CAP_PER_COIL_FAULT) != 0U) {
		status->valid |= TMC5262_STATUS_PER_COIL;
	}
	if (m->stealth != 0U) {
		status->valid |= TMC5262_STATUS_STEALTH;
	}

	status->standstill = (raw & m->stst) != 0U;
	status->open_load_a = (raw & m->ol_a) != 0U;
	status->open_load_b = (raw & m->ol_b) != 0U;
	status->short_gnd_a = (raw & m->s2g_a) != 0U;
	status->short_gnd_b = (raw & m->s2g_b) != 0U;
	status->short_vs_a = (raw & m->s2vs_a) != 0U;
	status->short_vs_b = (raw & m->s2vs_b) != 0U;
	status->otpw = (raw & m->otpw) != 0U;
	status->ot = (raw & m->ot) != 0U;
	status->stall = (raw & m->stallguard) != 0U;
	status->stealth = (raw & m->stealth) != 0U;
	status->ov = (raw & m->ov) != 0U;
	status->seq_stopped = (raw & m->seq_stopped) != 0U;
	status->cs_actual = (uint8_t)FIELD_GET(TMC5262_DRV_STATUS_CS_ACTUAL, raw);
	status->sg_result = (uint16_t)FIELD_GET(TMC5262_DRV_STATUS_SG_RESULT, raw);

	return 0;
}

int tmc5262_get_spi_status(const struct device *dev, uint8_t *spi_status)
{
	struct tmc5262_data *data = dev->data;

	if (spi_status == NULL) {
		return -EINVAL;
	}

	*spi_status = data->spi_status;

	return 0;
}

int tmc5262_clear_fault(const struct device *dev)
{
	const struct tmc5262_config *config = dev->config;
	struct tmc5262_data *data = dev->data;
	uint32_t gstat;
	int err;

	tmc5262_lock(dev);

	/*
	 * The short and overtemperature flags only clear when the driver is disabled by DRV_ENN
	 * or by TOFF = 0. Do both, then clear GSTAT.drv_err and restore TOFF.
	 */
	err = tmc5262_power_stage_disable(dev);
	if (err < 0) {
		goto out;
	}

	err = tmc5262_write(dev, TMC5262_REG_CHOPCONF,
			    data->shadow.chopconf & ~TMC5262_CHOPCONF_TOFF);
	if (err < 0) {
		goto out;
	}

	k_msleep(1);

	err = tmc5262_write(dev, TMC5262_REG_GSTAT, TMC5262_GSTAT_DRV_ERR);
	if (err == 0) {
		err = tmc5262_write(dev, TMC5262_REG_CHOPCONF, tmc5262_chopconf_for_device(dev));
	}
	if (err == 0) {
		err = tmc5262_read(dev, TMC5262_REG_GSTAT, &gstat);
	}
	if (err == 0 && (gstat & TMC5262_GSTAT_DRV_ERR) != 0U) {
		LOG_ERR("%s: driver error persists", config->variant->name);
		err = -EIO;
	}

out:
	tmc5262_unlock(dev);

	return err;
}

int tmc5262_fault_present(const struct device *dev, bool *present)
{
	const struct tmc5262_config *config = dev->config;
	const struct tmc5262_status_masks *m = &config->variant->status;
	uint32_t gstat;
	uint32_t drv_status;
	int err;

	err = tmc5262_read(dev, TMC5262_REG_GSTAT, &gstat);
	if (err < 0) {
		return err;
	}

	err = tmc5262_read(dev, TMC5262_REG_DRV_STATUS, &drv_status);
	if (err < 0) {
		return err;
	}

	*present = (gstat & TMC5262_GSTAT_DRV_ERR) != 0U ||
		   (drv_status & (m->s2g_a | m->s2g_b | m->s2vs_a | m->s2vs_b | m->ot)) != 0U;

	return 0;
}

int tmc5262_read_adc(const struct device *dev, int32_t *mv, int32_t *mdegc)
{
	uint32_t raw;
	int err;

	/* One read of ADC_VSUPPLY_TEMP: the voltage and the temperature are one sample */
	err = tmc5262_read(dev, TMC5262_REG_ADC_VSUPPLY_TEMP, &raw);
	if (err < 0) {
		return err;
	}

	/* VS = 0.1409 V x ADC_VSUPPLY, T [degC] = 1.042 x ADC_TEMP - 264.6 */
	if (mv != NULL) {
		*mv = (int32_t)((FIELD_GET(TMC5262_ADC_VSUPPLY, raw) * 1409U) / 10U);
	}
	if (mdegc != NULL) {
		*mdegc = (int32_t)(FIELD_GET(TMC5262_ADC_TEMP, raw) * 1042U) - 264600;
	}

	return 0;
}

int tmc5262_get_supply_mv(const struct device *dev, int32_t *mv)
{
	if (mv == NULL) {
		return -EINVAL;
	}

	return tmc5262_read_adc(dev, mv, NULL);
}

int tmc5262_get_temperature_mdegc(const struct device *dev, int32_t *mdegc)
{
	if (mdegc == NULL) {
		return -EINVAL;
	}

	return tmc5262_read_adc(dev, NULL, mdegc);
}

int tmc5262_set_otw_ov_thresholds(const struct device *dev, uint32_t otw_adc, uint32_t ov_adc)
{
	if (otw_adc > TMC5262_ADC_9BIT_MAX || ov_adc > TMC5262_ADC_9BIT_MAX) {
		return -EINVAL;
	}

	return tmc5262_cfg_set(dev, TMC5262_REG_OTW_OV_VTH,
			       FIELD_PREP(TMC5262_OTW_VTH, otw_adc) |
				       FIELD_PREP(TMC5262_OV_VTH, ov_adc));
}

#ifdef CONFIG_STEPPER_ADI_TMC5262_PLUS
int tmc5262_get_coil_current_raw(const struct device *dev, int16_t *coil_a, int16_t *coil_b)
{
	uint32_t raw;
	int err;

	if (coil_a == NULL || coil_b == NULL) {
		return -EINVAL;
	}

	err = tmc5262_require(dev, TMC5262_CAP_ADC_I);
	if (err < 0) {
		return err;
	}

	err = tmc5262_read(dev, TMC5262_REG_ADC_I, &raw);
	if (err < 0) {
		return err;
	}

	*coil_a = (int16_t)sign_extend(FIELD_GET(TMC5262_ADC_I_A, raw), 11);
	*coil_b = (int16_t)sign_extend(FIELD_GET(TMC5262_ADC_I_B, raw), 11);

	return 0;
}
#endif /* CONFIG_STEPPER_ADI_TMC5262_PLUS */

int tmc5262_get_step_frequency(const struct device *dev, uint32_t *hz)
{
	uint32_t tstep;
	int err;

	if (hz == NULL) {
		return -EINVAL;
	}

	err = tmc5262_read(dev, TMC5262_REG_TSTEP, &tstep);
	if (err < 0) {
		return err;
	}

	tstep &= TMC5262_VELOCITY_THRS_MASK;
	*hz = (tstep == 0U || tstep == TMC5262_VELOCITY_THRS_MASK) ? 0U : TMC5262_FCLK_HZ / tstep;

	return 0;
}

#ifdef CONFIG_STEPPER_ADI_TMC5262_RAMP_GEN
int tmc5262_get_actual_velocity(const struct device *dev, int32_t *microsteps_per_s)
{
	uint32_t raw;
	int err;

	if (microsteps_per_s == NULL) {
		return -EINVAL;
	}

	err = tmc5262_require(dev, TMC5262_CAP_VACTUAL);
	if (err < 0) {
		return err;
	}

	err = tmc5262_read(dev, TMC5262_REG_VACTUAL, &raw);
	if (err < 0) {
		return err;
	}

	*microsteps_per_s = tmc5262_velocity_from_reg(sign_extend(raw, TMC5262_VACTUAL_SIGN_BIT));

	return 0;
}
#endif /* CONFIG_STEPPER_ADI_TMC5262_RAMP_GEN */

int tmc5262_get_steps_lost(const struct device *dev, uint32_t *steps_lost)
{
	if (steps_lost == NULL) {
		return -EINVAL;
	}

	return tmc5262_read(dev, TMC5262_REG_STEPS_LOST, steps_lost);
}

int tmc5262_get_sequencer(const struct device *dev, uint16_t *mscnt, int16_t *cur_a, int16_t *cur_b)
{
	uint32_t raw;
	int err;

	if (mscnt == NULL || cur_a == NULL || cur_b == NULL) {
		return -EINVAL;
	}

	err = tmc5262_read(dev, TMC5262_REG_MSCNT, &raw);
	if (err < 0) {
		return err;
	}
	*mscnt = (uint16_t)(raw & TMC5262_MSCNT_MASK);

	err = tmc5262_read(dev, TMC5262_REG_MSCURACT, &raw);
	if (err < 0) {
		return err;
	}
	*cur_a = (int16_t)sign_extend(FIELD_GET(TMC5262_MSCURACT_CUR_A, raw), 8);
	*cur_b = (int16_t)sign_extend(FIELD_GET(TMC5262_MSCURACT_CUR_B, raw), 8);

	return 0;
}

/* -------------------------------------------------------------------------- */
/* Public API: current and chopper configuration                               */
/* -------------------------------------------------------------------------- */

int tmc5262_get_current_range(const struct device *dev, uint32_t *full_scale_ma, bool *is_peak)
{
	const struct tmc5262_variant *variant = tmc5262_get_variant_data(dev);
	struct tmc5262_data *data = dev->data;
	uint32_t range;
	uint32_t scale_idx;

	if (full_scale_ma == NULL || is_peak == NULL) {
		return -EINVAL;
	}

	tmc5262_lock(dev);
	range = FIELD_GET(TMC5262_DRV_CONF_CURRENT_RANGE, data->shadow.drv_conf);
	scale_idx = FIELD_GET(TMC5262_DRV_CONF_CURRENT_RANGE_SCALE, data->shadow.drv_conf);
	tmc5262_unlock(dev);

	*full_scale_ma = (variant->current_range_ma[range] * (scale_idx + 1U)) / 4U;
	*is_peak = variant->current_range_is_peak;

	return 0;
}

static int tmc5262_set_current_range_locked(const struct device *dev, uint32_t range,
					    uint32_t scale_percent)
{
	uint32_t scale_idx;
	int err;

	if (range > 3U || scale_percent < 25U || scale_percent > 100U ||
	    (scale_percent % 25U) != 0U) {
		return -EINVAL;
	}

	scale_idx = (scale_percent / 25U) - 1U;

	/* Scaling below 100 percent is only allowed with the lowest range */
	if (scale_idx != TMC5262_CURRENT_SCALE_100 && range != 0U) {
		return -EINVAL;
	}

	err = tmc5262_check_standstill(dev);
	if (err < 0) {
		return err;
	}

	return tmc5262_cfg_update(
		dev, TMC5262_REG_DRV_CONF,
		TMC5262_DRV_CONF_CURRENT_RANGE | TMC5262_DRV_CONF_CURRENT_RANGE_SCALE,
		FIELD_PREP(TMC5262_DRV_CONF_CURRENT_RANGE, range) |
			FIELD_PREP(TMC5262_DRV_CONF_CURRENT_RANGE_SCALE, scale_idx));
}

int tmc5262_set_current_range(const struct device *dev, uint32_t range, uint32_t scale_percent)
{
	int err;

	tmc5262_lock(dev);
	err = tmc5262_set_current_range_locked(dev, range, scale_percent);
	tmc5262_unlock(dev);

	return err;
}

int tmc5262_set_irun(const struct device *dev, uint32_t irun)
{
	if (irun > 255U) {
		return -EINVAL;
	}

	return tmc5262_cfg_update(dev, TMC5262_REG_IHOLD_IRUN, TMC5262_IHOLD_IRUN_IRUN,
				  FIELD_PREP(TMC5262_IHOLD_IRUN_IRUN, irun));
}

int tmc5262_set_ihold(const struct device *dev, uint32_t ihold)
{
	if (ihold > 255U) {
		return -EINVAL;
	}

	return tmc5262_cfg_update(dev, TMC5262_REG_IHOLD_IRUN, TMC5262_IHOLD_IRUN_IHOLD,
				  FIELD_PREP(TMC5262_IHOLD_IRUN_IHOLD, ihold));
}

int tmc5262_set_iholddelay(const struct device *dev, uint32_t delay)
{
	if (delay > 255U) {
		return -EINVAL;
	}

	return tmc5262_cfg_update(dev, TMC5262_REG_IHOLD_IRUN, TMC5262_IHOLD_IRUN_IHOLDDELAY,
				  FIELD_PREP(TMC5262_IHOLD_IRUN_IHOLDDELAY, delay));
}

int tmc5262_set_irundelay(const struct device *dev, uint32_t delay)
{
	if (delay > 15U) {
		return -EINVAL;
	}

	return tmc5262_cfg_update(dev, TMC5262_REG_IHOLD_IRUN, TMC5262_IHOLD_IRUN_IRUNDELAY,
				  FIELD_PREP(TMC5262_IHOLD_IRUN_IRUNDELAY, delay));
}

int tmc5262_set_tpowerdown(const struct device *dev, uint32_t tpowerdown)
{
	if (tpowerdown > 255U) {
		return -EINVAL;
	}

	return tmc5262_cfg_set(dev, TMC5262_REG_TPOWERDOWN, tpowerdown);
}

int tmc5262_set_direction_inverted(const struct device *dev, bool inverted)
{
	int err;

	tmc5262_lock(dev);

	if (tmc5262_ramp_is_step_source(dev)) {
		err = -ENOTSUP;
	} else {
		err = tmc5262_cfg_update(dev, TMC5262_REG_GCONF, TMC5262_GCONF_SHAFT,
					 inverted ? TMC5262_GCONF_SHAFT : 0U);
	}

	tmc5262_unlock(dev);

	return err;
}

int tmc5262_set_step_source(const struct device *dev, bool ramp_generator)
{
	const struct tmc5262_variant *variant = tmc5262_get_variant_data(dev);
	struct tmc5262_data *data = dev->data;
	bool set_bit;
	int err;

	if (variant->step_src_mask == 0U) {
		return -ENOTSUP;
	}

	if (ramp_generator && (variant->caps & TMC5262_CAP_RAMP_GEN) == 0U) {
		return -ENOTSUP;
	}

	tmc5262_lock(dev);

	err = tmc5262_check_standstill(dev);
	if (err == 0) {
		set_bit = ramp_generator ? variant->step_src_ramp_when_set
					 : !variant->step_src_ramp_when_set;

		err = tmc5262_cfg_update(dev, TMC5262_REG_GCONF, variant->step_src_mask,
					 set_bit ? variant->step_src_mask : 0U);
		if (err == 0) {
			data->step_ramp_source = ramp_generator;
		}
	}

	tmc5262_unlock(dev);

	return err;
}

#ifdef CONFIG_STEPPER_ADI_TMC5262_PLUS
static int tmc5262_set_stealthchop_locked(const struct device *dev, bool enable)
{
	const struct tmc5262_variant *variant = tmc5262_get_variant_data(dev);
	int err;

	err = tmc5262_require(dev, TMC5262_CAP_STEALTHCHOP);
	if (err < 0) {
		return err;
	}

	err = tmc5262_check_standstill(dev);
	if (err < 0) {
		return err;
	}

	return tmc5262_cfg_update(dev, TMC5262_REG_GCONF, variant->gconf_stealth_mask,
				  enable ? variant->gconf_stealth_mask : 0U);
}
#endif /* CONFIG_STEPPER_ADI_TMC5262_PLUS */

#ifdef CONFIG_STEPPER_ADI_TMC5262_PLUS
int tmc5262_set_stealthchop(const struct device *dev, bool enable)
{
	int err;

	tmc5262_lock(dev);
	err = tmc5262_set_stealthchop_locked(dev, enable);
	tmc5262_unlock(dev);

	return err;
}
#endif /* CONFIG_STEPPER_ADI_TMC5262_PLUS */

static int tmc5262_set_velocity_threshold(const struct device *dev, uint8_t reg, uint32_t val)
{
	if (val > TMC5262_VELOCITY_THRS_MASK) {
		return -EINVAL;
	}

	return tmc5262_cfg_set(dev, reg, val);
}

#ifdef CONFIG_STEPPER_ADI_TMC5262_PLUS
int tmc5262_set_tpwmthrs(const struct device *dev, uint32_t tpwmthrs)
{
	int err = tmc5262_require(dev, TMC5262_CAP_STEALTHCHOP);

	if (err < 0) {
		return err;
	}

	return tmc5262_set_velocity_threshold(dev, TMC5262_REG_TPWMTHRS, tpwmthrs);
}
#endif /* CONFIG_STEPPER_ADI_TMC5262_PLUS */

int tmc5262_set_tcoolthrs(const struct device *dev, uint32_t tcoolthrs)
{
	return tmc5262_set_velocity_threshold(dev, TMC5262_REG_TCOOLTHRS, tcoolthrs);
}

int tmc5262_set_thigh(const struct device *dev, uint32_t thigh)
{
	return tmc5262_set_velocity_threshold(dev, TMC5262_REG_THIGH, thigh);
}

int tmc5262_set_stallguard2(const struct device *dev, int32_t sgt, bool sfilt)
{
	if (sgt < TMC5262_SGT_MIN || sgt > TMC5262_SGT_MAX) {
		return -EINVAL;
	}

	return tmc5262_cfg_update(dev, TMC5262_REG_COOLCONF,
				  TMC5262_COOLCONF_SGT | TMC5262_COOLCONF_SFILT,
				  FIELD_PREP(TMC5262_COOLCONF_SGT, (uint32_t)sgt & 0x7FU) |
					  (sfilt ? TMC5262_COOLCONF_SFILT : 0U));
}

#ifdef CONFIG_STEPPER_ADI_TMC5262_RAMP_GEN
int tmc5262_set_stallguard_stop(const struct device *dev, bool enable)
{
	int err = tmc5262_require(dev, TMC5262_CAP_RAMP_GEN | TMC5262_CAP_REF_SWITCH);

	if (err < 0) {
		return err;
	}

	/* FR-MOTION-003: stop-on-stall is a ramp generator function */
	if (!tmc5262_ramp_is_step_source(dev)) {
		return -ENOTSUP;
	}

	return tmc5262_cfg_update(dev, TMC5262_REG_SW_MODE, TMC5262_SW_MODE_SG_STOP,
				  enable ? TMC5262_SW_MODE_SG_STOP : 0U);
}
#endif /* CONFIG_STEPPER_ADI_TMC5262_RAMP_GEN */

int tmc5262_set_coolstep(const struct device *dev, uint32_t semin, uint32_t seup, uint32_t semax,
			 uint32_t sedn, bool seimin)
{
	if (semin > 15U || seup > 3U || semax > 15U || sedn > 7U) {
		return -EINVAL;
	}

	return tmc5262_cfg_update(dev, TMC5262_REG_COOLCONF,
				  TMC5262_COOLCONF_SEMIN | TMC5262_COOLCONF_SEUP |
					  TMC5262_COOLCONF_SEMAX | TMC5262_COOLCONF_SEDN |
					  TMC5262_COOLCONF_SEIMIN,
				  FIELD_PREP(TMC5262_COOLCONF_SEMIN, semin) |
					  FIELD_PREP(TMC5262_COOLCONF_SEUP, seup) |
					  FIELD_PREP(TMC5262_COOLCONF_SEMAX, semax) |
					  FIELD_PREP(TMC5262_COOLCONF_SEDN, sedn) |
					  (seimin ? TMC5262_COOLCONF_SEIMIN : 0U));
}

#ifdef CONFIG_STEPPER_ADI_TMC5262_PLUS
static int tmc5262_set_udcstep_locked(const struct device *dev, bool enable, uint32_t accel_thrs,
				      uint32_t decel_thrs, uint32_t tudcstep)
{
	const struct tmc5262_variant *variant = tmc5262_get_variant_data(dev);
	struct tmc5262_data *data = dev->data;
	int err;

	err = tmc5262_require(dev, TMC5262_CAP_UDCSTEP);
	if (err < 0) {
		return err;
	}

	if (accel_thrs > 15U || decel_thrs > 15U || decel_thrs <= accel_thrs ||
	    tudcstep > TMC5262_VELOCITY_THRS_MASK) {
		return -EINVAL;
	}

	if (enable) {
		if ((data->shadow.gconf & variant->gconf_stealth_mask) == 0U) {
			return -EPERM;
		}

		if ((variant->caps & TMC5262_CAP_RAMP_GEN) != 0U &&
		    (data->shadow.sw_mode & TMC5262_SW_MODE_EN_SOFTSTOP) == 0U) {
			return -EPERM;
		}
	}

	err = tmc5262_cfg_set(dev, TMC5262_REG_TUDCSTEP, tudcstep);
	if (err < 0) {
		return err;
	}

	return tmc5262_cfg_set(dev, TMC5262_REG_UDC_CONF,
			       FIELD_PREP(TMC5262_UDC_CONF_ACCEL_THRS, accel_thrs) |
				       FIELD_PREP(TMC5262_UDC_CONF_DECEL_THRS, decel_thrs) |
				       (enable ? TMC5262_UDC_CONF_ENABLE : 0U));
}
#endif /* CONFIG_STEPPER_ADI_TMC5262_PLUS */

#ifdef CONFIG_STEPPER_ADI_TMC5262_PLUS
int tmc5262_set_udcstep(const struct device *dev, bool enable, uint32_t accel_thrs,
			uint32_t decel_thrs, uint32_t tudcstep)
{
	int err;

	tmc5262_lock(dev);
	err = tmc5262_set_udcstep_locked(dev, enable, accel_thrs, decel_thrs, tudcstep);
	tmc5262_unlock(dev);

	return err;
}
#endif /* CONFIG_STEPPER_ADI_TMC5262_PLUS */

/* -------------------------------------------------------------------------- */
/* Public API: ramp generator extras                                           */
/* -------------------------------------------------------------------------- */

#ifdef CONFIG_STEPPER_ADI_TMC5262_RAMP_GEN
static int tmc5262_set_switch_config_locked(const struct device *dev, uint32_t sw_mode,
					    int32_t virtual_stop_l, int32_t virtual_stop_r)
{
	int err = tmc5262_require(dev, TMC5262_CAP_RAMP_GEN | TMC5262_CAP_REF_SWITCH);

	if (err < 0) {
		return err;
	}

	/* FR-MOTION-003: the reference switch inputs are STEP/DIR inputs in STEP/DIR operation */
	if (!tmc5262_ramp_is_step_source(dev)) {
		return -ENOTSUP;
	}

	if ((sw_mode & ~GENMASK(15, 0)) != 0U) {
		return -EINVAL;
	}

	err = tmc5262_cfg_set(dev, TMC5262_REG_VIRTUAL_STOP_L, (uint32_t)virtual_stop_l);
	if (err < 0) {
		return err;
	}

	err = tmc5262_cfg_set(dev, TMC5262_REG_VIRTUAL_STOP_R, (uint32_t)virtual_stop_r);
	if (err < 0) {
		return err;
	}

	return tmc5262_cfg_set(dev, TMC5262_REG_SW_MODE, sw_mode);
}
#endif /* CONFIG_STEPPER_ADI_TMC5262_RAMP_GEN */

#ifdef CONFIG_STEPPER_ADI_TMC5262_RAMP_GEN
int tmc5262_set_switch_config(const struct device *dev, uint32_t sw_mode, int32_t virtual_stop_l,
			      int32_t virtual_stop_r)
{
	int err;

	tmc5262_lock(dev);
	err = tmc5262_set_switch_config_locked(dev, sw_mode, virtual_stop_l, virtual_stop_r);
	tmc5262_unlock(dev);

	return err;
}
#endif /* CONFIG_STEPPER_ADI_TMC5262_RAMP_GEN */

#ifdef CONFIG_STEPPER_ADI_TMC5262_RAMP_GEN
int tmc5262_get_xlatch(const struct device *dev, int32_t *xlatch)
{
	uint32_t raw;
	int err;

	if (xlatch == NULL) {
		return -EINVAL;
	}

	err = tmc5262_require(dev, TMC5262_CAP_XLATCH);
	if (err < 0) {
		return err;
	}

	err = tmc5262_read(dev, TMC5262_REG_XLATCH, &raw);
	if (err < 0) {
		return err;
	}

	*xlatch = (int32_t)raw;

	return 0;
}
#endif /* CONFIG_STEPPER_ADI_TMC5262_RAMP_GEN */

#ifdef CONFIG_STEPPER_ADI_TMC5262_RAMP_GEN
static int tmc5262_set_position_compare_locked(const struct device *dev, int32_t x_compare,
					       uint32_t repeat)
{
	int err = tmc5262_require(dev, TMC5262_CAP_POS_COMPARE);

	if (err < 0) {
		return err;
	}

	err = tmc5262_cfg_set(dev, TMC5262_REG_X_COMPARE_REPEAT, repeat);
	if (err < 0) {
		return err;
	}

	return tmc5262_cfg_set(dev, TMC5262_REG_X_COMPARE, (uint32_t)x_compare);
}
#endif /* CONFIG_STEPPER_ADI_TMC5262_RAMP_GEN */

#ifdef CONFIG_STEPPER_ADI_TMC5262_RAMP_GEN
int tmc5262_set_position_compare(const struct device *dev, int32_t x_compare, uint32_t repeat)
{
	int err;

	tmc5262_lock(dev);
	err = tmc5262_set_position_compare_locked(dev, x_compare, repeat);
	tmc5262_unlock(dev);

	return err;
}
#endif /* CONFIG_STEPPER_ADI_TMC5262_RAMP_GEN */

#ifdef CONFIG_STEPPER_ADI_TMC5262_RAMP_GEN
int tmc5262_set_direct_current(const struct device *dev, bool enable, int32_t coil_1,
			       int32_t coil_2)
{
	uint32_t xactual;
	int err;

	err = tmc5262_require(dev, TMC5262_CAP_DIRECT_MODE);
	if (err < 0) {
		return err;
	}

	if (enable && (coil_1 < -256 || coil_1 > 255 || coil_2 < -256 || coil_2 > 255)) {
		return -EINVAL;
	}

	tmc5262_lock(dev);

	/* Direct mode is only entered and left at standstill, in hold mode */
	err = tmc5262_check_standstill(dev);
	if (err < 0) {
		goto out;
	}

	err = tmc5262_write(dev, TMC5262_REG_RAMPMODE, TMC5262_RAMPMODE_HOLD);
	if (err < 0) {
		goto out;
	}

	if (enable) {
		/* Signed 9 bit currents: coil 1 in bits 8..0, coil 2 in bits 24..16 */
		err = tmc5262_write(dev, TMC5262_REG_XTARGET,
				    ((uint32_t)coil_1 & 0x1FFU) |
					    (((uint32_t)coil_2 & 0x1FFU) << 16));
		if (err == 0) {
			err = tmc5262_cfg_update(dev, TMC5262_REG_GCONF, TMC5262_GCONF_DIRECT_MODE,
						 TMC5262_GCONF_DIRECT_MODE);
		}
		goto out;
	}

	/* Leave direct mode, then restore XTARGET = XACTUAL so that no motion starts */
	err = tmc5262_cfg_update(dev, TMC5262_REG_GCONF, TMC5262_GCONF_DIRECT_MODE, 0U);
	if (err == 0) {
		err = tmc5262_read(dev, TMC5262_REG_XACTUAL, &xactual);
	}
	if (err == 0) {
		err = tmc5262_write(dev, TMC5262_REG_XTARGET, xactual);
	}
	if (err == 0) {
		err = tmc5262_write(dev, TMC5262_REG_RAMPMODE, TMC5262_RAMPMODE_POSITION);
	}

out:
	tmc5262_unlock(dev);

	return err;
}
#endif /* CONFIG_STEPPER_ADI_TMC5262_RAMP_GEN */

#ifdef CONFIG_STEPPER_ADI_TMC5262_RAMP_GEN
static int tmc5262_abort_locked(const struct device *dev)
{
	int err = tmc5262_require(dev, TMC5262_CAP_RAMP_GEN);

	if (err < 0) {
		return err;
	}

	/* An abort replaces a stop that is still completing (restores TZEROWAIT) */
	err = tmc5262_ramp_stop_cancel(dev);
	if (err < 0) {
		return err;
	}

	err = tmc5262_write(dev, TMC5262_REG_VMAX, 0);
	if (err < 0) {
		return err;
	}

	return tmc5262_write(dev, TMC5262_REG_RAMPMODE, TMC5262_RAMPMODE_HOLD);
}
#endif /* CONFIG_STEPPER_ADI_TMC5262_RAMP_GEN */

#ifdef CONFIG_STEPPER_ADI_TMC5262_RAMP_GEN
int tmc5262_abort(const struct device *dev)
{
	int err;

	tmc5262_lock(dev);
	err = tmc5262_abort_locked(dev);
	tmc5262_unlock(dev);

	return err;
}
#endif /* CONFIG_STEPPER_ADI_TMC5262_RAMP_GEN */

/* -------------------------------------------------------------------------- */
/* Public API: encoder                                                         */
/* -------------------------------------------------------------------------- */

static int tmc5262_encoder_configure_locked(const struct device *dev, uint32_t encmode,
					    uint32_t enc_const)
{
	const struct tmc5262_variant *variant = tmc5262_get_variant_data(dev);
	struct tmc5262_data *data = dev->data;
	const uint32_t n_channel_bits = TMC5262_ENCMODE_POL_N | TMC5262_ENCMODE_CLR_ONCE |
					TMC5262_ENCMODE_CLR_CONT | TMC5262_ENCMODE_LATCH_X_ACT |
					TMC5262_ENCMODE_CLR_ENC_X;
	int err;

	if ((encmode & ~variant->encmode_mask) != 0U) {
		return -EINVAL;
	}

	/* ENCN cannot be both the overvoltage output and the encoder N channel */
	if ((data->shadow.gconf & TMC5262_GCONF_OV_NN) != 0U && (encmode & n_channel_bits) != 0U) {
		return -EPERM;
	}

	err = tmc5262_cfg_set(dev, TMC5262_REG_ENC_CONST, enc_const);
	if (err < 0) {
		return err;
	}

	return tmc5262_cfg_set(dev, TMC5262_REG_ENCMODE, encmode);
}

int tmc5262_encoder_configure(const struct device *dev, uint32_t encmode, uint32_t enc_const)
{
	int err;

	tmc5262_lock(dev);
	err = tmc5262_encoder_configure_locked(dev, encmode, enc_const);
	tmc5262_unlock(dev);

	return err;
}

int tmc5262_encoder_read(const struct device *dev, struct tmc5262_encoder *enc)
{
	uint32_t raw;
	int err;

	if (enc == NULL) {
		return -EINVAL;
	}

	err = tmc5262_read(dev, TMC5262_REG_X_ENC, &raw);
	if (err < 0) {
		return err;
	}
	enc->x_enc = (int32_t)raw;

	err = tmc5262_read(dev, TMC5262_REG_ENC_LATCH, &raw);
	if (err < 0) {
		return err;
	}
	enc->enc_latch = (int32_t)raw;

	err = tmc5262_read(dev, TMC5262_REG_ENC_STATUS, &raw);
	if (err < 0) {
		return err;
	}
	enc->n_event = (raw & TMC5262_ENC_STATUS_N_EVENT) != 0U;
	enc->deviation_warn = (raw & TMC5262_ENC_STATUS_DEVIATION_WARN) != 0U;

	return 0;
}

#ifdef CONFIG_STEPPER_ADI_TMC5262_PLUS
int tmc5262_get_encoder_deviation(const struct device *dev, uint32_t *deviation)
{
	int err;

	if (deviation == NULL) {
		return -EINVAL;
	}

	err = tmc5262_require(dev, TMC5262_CAP_ENC_DEVIATION);
	if (err < 0) {
		return err;
	}

	return tmc5262_read(dev, TMC5262_REG_ENC_DEVIATION, deviation);
}
#endif /* CONFIG_STEPPER_ADI_TMC5262_PLUS */

/* -------------------------------------------------------------------------- */
/* Public API: StealthChop+, StallGuard+, CoolStep+, coil measurement, TriCoder */
/* -------------------------------------------------------------------------- */

#ifdef CONFIG_STEPPER_ADI_TMC5262_PLUS
int tmc5262_set_pwmconf(const struct device *dev, uint32_t pwmconf)
{
	int err = tmc5262_require(dev, TMC5262_CAP_STEALTHCHOP);

	if (err < 0) {
		return err;
	}

	return tmc5262_cfg_set(dev, TMC5262_REG_PWMCONF, pwmconf);
}
#endif /* CONFIG_STEPPER_ADI_TMC5262_PLUS */

#ifdef CONFIG_STEPPER_ADI_TMC5262_PLUS
static int tmc5262_set_stealthchop_regulators_locked(const struct device *dev, uint32_t current_pi,
						     uint32_t angle_pi, uint32_t cur_angle_limit,
						     uint32_t angle_lower_limit)
{
	int err = tmc5262_require(dev, TMC5262_CAP_STEALTHCHOP);

	if (err < 0) {
		return err;
	}

	/* Validate everything before the first write: no partial update */
	if ((current_pi & ~TMC5262_PI_REG_MASK) != 0U || (angle_pi & ~TMC5262_PI_REG_MASK) != 0U ||
	    (cur_angle_limit & ~TMC5262_CUR_ANGLE_LIMIT_MASK) != 0U ||
	    (angle_lower_limit & ~TMC5262_ANGLE_LOWER_I_LIMIT) != 0U) {
		return -EINVAL;
	}

	err = tmc5262_cfg_set(dev, TMC5262_REG_CURRENT_PI_REG, current_pi);
	if (err == 0) {
		err = tmc5262_cfg_set(dev, TMC5262_REG_ANGLE_PI_REG, angle_pi);
	}
	if (err == 0) {
		err = tmc5262_cfg_set(dev, TMC5262_REG_CUR_ANGLE_LIMIT, cur_angle_limit);
	}
	if (err == 0) {
		err = tmc5262_cfg_set(dev, TMC5262_REG_ANGLE_LOWER_LIMIT, angle_lower_limit);
	}

	return err;
}
#endif /* CONFIG_STEPPER_ADI_TMC5262_PLUS */

#ifdef CONFIG_STEPPER_ADI_TMC5262_PLUS
int tmc5262_set_stealthchop_regulators(const struct device *dev, uint32_t current_pi,
				       uint32_t angle_pi, uint32_t cur_angle_limit,
				       uint32_t angle_lower_limit)
{
	int err;

	tmc5262_lock(dev);
	err = tmc5262_set_stealthchop_regulators_locked(dev, current_pi, angle_pi, cur_angle_limit,
							angle_lower_limit);
	tmc5262_unlock(dev);

	return err;
}
#endif /* CONFIG_STEPPER_ADI_TMC5262_PLUS */

#ifdef CONFIG_STEPPER_ADI_TMC5262_PLUS
int tmc5262_get_stealthchop_measurements(const struct device *dev, uint32_t *cur_angle_meas,
					 uint32_t *pi_results)
{
	int err;

	if (cur_angle_meas == NULL || pi_results == NULL) {
		return -EINVAL;
	}

	err = tmc5262_require(dev, TMC5262_CAP_STEALTHCHOP);
	if (err < 0) {
		return err;
	}

	err = tmc5262_read(dev, TMC5262_REG_CUR_ANGLE_MEAS, cur_angle_meas);
	if (err < 0) {
		return err;
	}

	return tmc5262_read(dev, TMC5262_REG_PI_RESULTS, pi_results);
}
#endif /* CONFIG_STEPPER_ADI_TMC5262_PLUS */

#ifdef CONFIG_STEPPER_ADI_TMC5262_PLUS
static int tmc5262_set_stallguard_plus_locked(const struct device *dev, uint32_t sgp_conf,
					      uint32_t low_vel_thrs)
{
	int err = tmc5262_require(dev, TMC5262_CAP_STALLGUARD_PLUS);

	if (err < 0) {
		return err;
	}

	if (low_vel_thrs > TMC5262_VELOCITY_THRS_MASK) {
		return -EINVAL;
	}

	err = tmc5262_cfg_set(dev, TMC5262_REG_TSGP_LOW_VEL_THRS, low_vel_thrs);
	if (err < 0) {
		return err;
	}

	return tmc5262_cfg_set(dev, TMC5262_REG_SGP_CONF, sgp_conf);
}
#endif /* CONFIG_STEPPER_ADI_TMC5262_PLUS */

#ifdef CONFIG_STEPPER_ADI_TMC5262_PLUS
int tmc5262_set_stallguard_plus(const struct device *dev, uint32_t sgp_conf, uint32_t low_vel_thrs)
{
	int err;

	tmc5262_lock(dev);
	err = tmc5262_set_stallguard_plus_locked(dev, sgp_conf, low_vel_thrs);
	tmc5262_unlock(dev);

	return err;
}
#endif /* CONFIG_STEPPER_ADI_TMC5262_PLUS */

#ifdef CONFIG_STEPPER_ADI_TMC5262_PLUS
static int tmc5262_set_coolstep_plus_locked(const struct device *dev, uint32_t conf,
					    uint32_t pi_reg, uint32_t pi_down,
					    uint32_t reserve_conf)
{
	int err = tmc5262_require(dev, TMC5262_CAP_COOLSTEP_PLUS);

	if (err < 0) {
		return err;
	}

	/* Validate everything before the first write: no partial update */
	if ((conf & ~TMC5262_COOLPLUS_CONF_MASK) != 0U || (pi_reg & ~TMC5262_PI_REG_MASK) != 0U ||
	    (pi_down & ~TMC5262_COOLPLUS_PI_DOWN_MASK) != 0U) {
		return -EINVAL;
	}

	err = tmc5262_cfg_set(dev, TMC5262_REG_COOLSTEPPLUS_PI_REG, pi_reg);
	if (err == 0) {
		err = tmc5262_cfg_set(dev, TMC5262_REG_COOLSTEPPLUS_PI_DOWN, pi_down);
	}
	if (err == 0) {
		err = tmc5262_cfg_set(dev, TMC5262_REG_COOLSTEPPLUS_RESERVE_CONF, reserve_conf);
	}
	if (err == 0) {
		err = tmc5262_cfg_set(dev, TMC5262_REG_COOLSTEPPLUS_CONF, conf);
	}

	return err;
}
#endif /* CONFIG_STEPPER_ADI_TMC5262_PLUS */

#ifdef CONFIG_STEPPER_ADI_TMC5262_PLUS
int tmc5262_set_coolstep_plus(const struct device *dev, uint32_t conf, uint32_t pi_reg,
			      uint32_t pi_down, uint32_t reserve_conf)
{
	int err;

	tmc5262_lock(dev);
	err = tmc5262_set_coolstep_plus_locked(dev, conf, pi_reg, pi_down, reserve_conf);
	tmc5262_unlock(dev);

	return err;
}
#endif /* CONFIG_STEPPER_ADI_TMC5262_PLUS */

#ifdef CONFIG_STEPPER_ADI_TMC5262_PLUS
int tmc5262_get_coolstep_plus_load_reserve(const struct device *dev, uint32_t *load_reserve)
{
	int err;

	if (load_reserve == NULL) {
		return -EINVAL;
	}

	err = tmc5262_require(dev, TMC5262_CAP_COOLSTEP_PLUS);
	if (err < 0) {
		return err;
	}

	return tmc5262_read(dev, TMC5262_REG_COOLSTEPPLUS_LOAD_RESERVE, load_reserve);
}
#endif /* CONFIG_STEPPER_ADI_TMC5262_PLUS */

#ifdef CONFIG_STEPPER_ADI_TMC5262_PLUS
static int tmc5262_measure_rcoil_locked(const struct device *dev, uint32_t t_rcoil_meas,
					struct tmc5262_rcoil *rcoil)
{
	uint32_t raw;
	int err;

	if (rcoil == NULL || t_rcoil_meas > TMC5262_VELOCITY_THRS_MASK) {
		return -EINVAL;
	}

	err = tmc5262_require(dev, TMC5262_CAP_COIL_MEAS);
	if (err < 0) {
		return err;
	}

	err = tmc5262_check_standstill(dev);
	if (err < 0) {
		return err;
	}

	err = tmc5262_cfg_set(dev, TMC5262_REG_T_RCOIL_MEAS, t_rcoil_meas);
	if (err < 0) {
		return err;
	}

	err = tmc5262_read(dev, TMC5262_REG_R_COIL, &raw);
	if (err < 0) {
		return err;
	}

	rcoil->a = (uint16_t)FIELD_GET(TMC5262_R_COIL_A, raw);
	rcoil->b = (uint16_t)FIELD_GET(TMC5262_R_COIL_B, raw);

	return 0;
}
#endif /* CONFIG_STEPPER_ADI_TMC5262_PLUS */

#ifdef CONFIG_STEPPER_ADI_TMC5262_PLUS
int tmc5262_measure_rcoil(const struct device *dev, uint32_t t_rcoil_meas,
			  struct tmc5262_rcoil *rcoil)
{
	int err;

	tmc5262_lock(dev);
	err = tmc5262_measure_rcoil_locked(dev, t_rcoil_meas, rcoil);
	tmc5262_unlock(dev);

	return err;
}
#endif /* CONFIG_STEPPER_ADI_TMC5262_PLUS */

#ifdef CONFIG_STEPPER_ADI_TMC5262_PLUS
static int tmc5262_set_rcoil_user_locked(const struct device *dev, uint32_t rcoil_a,
					 uint32_t rcoil_b)
{
	int err = tmc5262_require(dev, TMC5262_CAP_COIL_MEAS);

	if (err < 0) {
		return err;
	}

	if (rcoil_a > 0xFFFU || rcoil_b > 0xFFFU) {
		return -EINVAL;
	}

	err = tmc5262_check_standstill(dev);
	if (err < 0) {
		return err;
	}

	err = tmc5262_cfg_set(dev, TMC5262_REG_R_COIL_USER,
			      FIELD_PREP(TMC5262_R_COIL_A, rcoil_a) |
				      FIELD_PREP(TMC5262_R_COIL_B, rcoil_b));
	if (err < 0) {
		return err;
	}

	return tmc5262_cfg_update(dev, TMC5262_REG_COIL_INDUCT, TMC5262_COIL_INDUCT_RCOIL_MANUAL,
				  TMC5262_COIL_INDUCT_RCOIL_MANUAL);
}
#endif /* CONFIG_STEPPER_ADI_TMC5262_PLUS */

#ifdef CONFIG_STEPPER_ADI_TMC5262_PLUS
int tmc5262_set_rcoil_user(const struct device *dev, uint32_t rcoil_a, uint32_t rcoil_b)
{
	int err;

	tmc5262_lock(dev);
	err = tmc5262_set_rcoil_user_locked(dev, rcoil_a, rcoil_b);
	tmc5262_unlock(dev);

	return err;
}
#endif /* CONFIG_STEPPER_ADI_TMC5262_PLUS */

#ifdef CONFIG_STEPPER_ADI_TMC5262_PLUS
int tmc5262_set_tricoder(const struct device *dev, bool enable, uint32_t bemf_encmode_bits)
{
	const struct tmc5262_variant *variant = tmc5262_get_variant_data(dev);
	int err = tmc5262_require(dev, TMC5262_CAP_TRICODER);

	if (err < 0) {
		return err;
	}

	if ((bemf_encmode_bits & ~variant->bemf_mask) != 0U) {
		return -EINVAL;
	}

	/* The selection bit: 0 = back-EMF full step TriCoder, 1 = incremental encoder interface */
	bemf_encmode_bits &= ~variant->bemf_encoder_sel;
	if (!enable) {
		bemf_encmode_bits |= variant->bemf_encoder_sel;
	}

	return tmc5262_cfg_update(dev, TMC5262_REG_ENCMODE, variant->bemf_mask, bemf_encmode_bits);
}
#endif /* CONFIG_STEPPER_ADI_TMC5262_PLUS */

#ifdef CONFIG_STEPPER_ADI_TMC5262_PLUS
int tmc5262_get_tricoder(const struct device *dev, struct tmc5262_tricoder *tc)
{
	uint32_t raw;
	int err;

	if (tc == NULL) {
		return -EINVAL;
	}

	err = tmc5262_require(dev, TMC5262_CAP_TRICODER);
	if (err < 0) {
		return err;
	}

	tmc5262_lock(dev);

	err = tmc5262_read(dev, TMC5262_REG_SGP_BEMF, &raw);
	if (err == 0) {
		tc->ubemf_abs = (uint16_t)FIELD_GET(TMC5262_SGP_BEMF_UBEMF_ABS, raw);
		tc->sgp_raw = (uint16_t)FIELD_GET(TMC5262_SGP_BEMF_SGP_RAW, raw);
		err = tmc5262_read(dev, TMC5262_REG_ENC_STATUS, &raw);
	}
	if (err == 0) {
		/* ENC_STATUS.deviation_warn: X_ENC passed ENC_DEVIATION, only valid with TriCoder
		 */
		tc->step_loss = (raw & TMC5262_ENC_STATUS_DEVIATION_WARN) != 0U;
		tc->n_event = (raw & TMC5262_ENC_STATUS_N_EVENT) != 0U;
	}

	tmc5262_unlock(dev);

	return err;
}
#endif /* CONFIG_STEPPER_ADI_TMC5262_PLUS */

int tmc5262_get_spi_status_decoded(const struct device *dev, struct tmc5262_spi_status *st)
{
	const struct tmc5262_config *config = dev->config;
	struct tmc5262_data *data = dev->data;
	uint8_t raw;

	if (st == NULL) {
		return -EINVAL;
	}

	tmc5262_lock(dev);
	raw = data->spi_status;
	tmc5262_unlock(dev);

	st->standstill = (raw & TMC5262_SPI_STATUS_STANDSTILL) != 0U;
	st->sg = (raw & TMC5262_SPI_STATUS_SG) != 0U;
	st->driver_error = (raw & TMC5262_SPI_STATUS_DRIVER_ERROR) != 0U;
	st->reset_flag = (raw & TMC5262_SPI_STATUS_RESET_FLAG) != 0U;
	/* Bits 7..4 are only meaningful (and only captured) on variants with a ramp generator */
	st->ramp_valid = (config->variant->caps & TMC5262_CAP_RAMP_GEN) != 0U;
	st->velocity_reached = (raw & TMC5262_SPI_STATUS_VELOCITY_REACHED) != 0U;
	st->position_reached = (raw & TMC5262_SPI_STATUS_POSITION_REACHED) != 0U;
	st->stop_l = (raw & TMC5262_SPI_STATUS_STOP_L) != 0U;
	st->stop_r = (raw & TMC5262_SPI_STATUS_STOP_R) != 0U;

	return 0;
}

int tmc5262_set_encoder_event_cb(const struct device *dev, tmc5262_encoder_event_cb_t cb,
				 void *user_data)
{
	struct tmc5262_data *data = dev->data;
	bool had_cb;

	tmc5262_lock(dev);

	had_cb = data->enc_cb != NULL;
	data->enc_cb = cb;
	data->enc_cb_user_data = user_data;
	data->enc_reported = 0U;

	if (cb != NULL && !had_cb) {
		tmc5262_event_users_add(dev);
	} else if (cb == NULL && had_cb) {
		tmc5262_event_users_remove(dev);
	}

	tmc5262_unlock(dev);

	return 0;
}

/* -------------------------------------------------------------------------- */
/* Initialization                                                              */
/* -------------------------------------------------------------------------- */

static int tmc5262_init(const struct device *dev)
{
	const struct tmc5262_config *config = dev->config;
	struct tmc5262_data *data = dev->data;
	uint32_t ioin;
	int err;

	k_mutex_init(&data->lock);
	k_work_init(&data->event_work, tmc5262_event_work_handler);
	k_work_init_delayable(&data->poll_work, tmc5262_poll_work_handler);
	atomic_clear(&data->cb_users);
	atomic_clear(&data->pending);

	data->dev = dev;
	data->shadow = config->init;
	data->step_ramp_source = config->ramp_step_source;
	data->power_stage_on = false;
	data->asleep = false;
	data->stall_active = false;
	data->ev_reported = 0U;

	err = config->bus_io->check(&config->bus, config->comm_type);
	if (err < 0) {
		return -ENODEV;
	}

	/* FR-INIT-003: every GPIO listed in devicetree must be ready */
	const struct gpio_dt_spec *gpios[] = {&config->sleepn_gpio, &config->drv_enn_gpio,
					      &config->do0_gpio, &config->do1_gpio};

	for (size_t i = 0; i < ARRAY_SIZE(gpios); i++) {
		if (gpios[i]->port != NULL && !gpio_is_ready_dt(gpios[i])) {
			LOG_ERR_DEVICE_NOT_READY(gpios[i]->port);
			return -ENODEV;
		}
	}

	/* FR-INIT-005: the power stage stays off until explicitly enabled */
	if (config->drv_enn_gpio.port != NULL) {
		err = gpio_pin_configure_dt(&config->drv_enn_gpio, GPIO_OUTPUT_INACTIVE);
		if (err < 0) {
			return err;
		}
	}

	if (config->sleepn_gpio.port != NULL) {
		err = gpio_pin_configure_dt(&config->sleepn_gpio, GPIO_OUTPUT_ACTIVE);
		if (err < 0) {
			return err;
		}

		err = tmc5262_hw_reset(dev);
		if (err < 0) {
			LOG_ERR("%s: reset failed (%d)", config->variant->name, err);
			return err;
		}
	}

	err = tmc5262_start_and_configure(dev, true);
	if (err < 0) {
		LOG_ERR("%s: initialization failed (%d)", config->variant->name, err);
		return err;
	}

	/* IOIN.DRV_ENN must read 1 (disabled): a wrong pin polarity would enable the bridges */
	if (config->drv_enn_gpio.port != NULL) {
		err = tmc5262_read_unchecked(dev, TMC5262_REG_IOIN, &ioin);
		if (err < 0) {
			return err;
		}

		if ((ioin & TMC5262_IOIN_DRV_ENN) == 0U) {
			LOG_ERR("%s: DRV_ENN reads enabled while disabled, check the polarity of "
				"drv-enn-gpios (GPIO_ACTIVE_LOW)",
				config->variant->name);
			return -EIO;
		}
	}

	err = tmc5262_setup_do_irq(&config->do0_gpio, &data->do0_cb, tmc5262_do0_isr);
	if (err < 0) {
		LOG_ERR("%s: DO0 interrupt setup failed (%d)", config->variant->name, err);
		return err;
	}

	err = tmc5262_setup_do_irq(&config->do1_gpio, &data->do1_cb, tmc5262_do1_isr);
	if (err < 0) {
		LOG_ERR("%s: DO1 interrupt setup failed (%d)", config->variant->name, err);
		/* Do not leave the DO0 callback registered */
		tmc5262_teardown_do_irq(&config->do0_gpio, &data->do0_cb);
		return err;
	}

	LOG_INF("%s initialized (%s clock)", config->variant->name,
		config->ext_clock ? "external" : "internal");

	return 0;
}

/* -------------------------------------------------------------------------- */
/* Devicetree instantiation (FR-INIT-001, FR-VAR-005)                          */
/* -------------------------------------------------------------------------- */

#define TMC5262_DO_COMPOSE(inst, prop, pfx)                                                        \
	COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, prop),                                             \
		    (DT_INST_FOREACH_PROP_ELEM_SEP(inst, prop, pfx, (|))), (0))

#define TMC5262_DO0_ELEM(node_id, prop, idx)                                                       \
	UTIL_CAT(TMC5262_DO0_SRC_, DT_STRING_TOKEN_BY_IDX(node_id, prop, idx))
#define TMC5262_DO1_ELEM(node_id, prop, idx)                                                       \
	UTIL_CAT(TMC5262_DO1_SRC_, DT_STRING_TOKEN_BY_IDX(node_id, prop, idx))

#define TMC5262_DO_CONF_VAL(inst)                                                                  \
	(TMC5262_DO_COMPOSE(inst, do0_sources, TMC5262_DO0_ELEM) |                                 \
	 TMC5262_DO_COMPOSE(inst, do1_sources, TMC5262_DO1_ELEM) |                                 \
	 (DT_INST_PROP(inst, do0_push_pull) ? TMC5262_DO_CONF_DO0_NOD_PP : 0U) |                   \
	 (DT_INST_PROP(inst, do0_active_high) ? 0U : TMC5262_DO_CONF_DO0_INVPP) |                  \
	 (DT_INST_PROP(inst, do1_push_pull) ? TMC5262_DO_CONF_DO1_NOD_PP : 0U) |                   \
	 (DT_INST_PROP(inst, do1_active_high) ? 0U : TMC5262_DO_CONF_DO1_INVPP))

#define TMC5262_GCONF_COMMON_VAL(inst)                                                             \
	((DT_INST_PROP(inst, fast_standstill) ? TMC5262_GCONF_FAST_STANDSTILL : 0U) |              \
	 (DT_INST_PROP(inst, multistep_filt) ? TMC5262_GCONF_MULTISTEP_FILT : 0U) |                \
	 (DT_INST_PROP(inst, invert_direction) ? TMC5262_GCONF_SHAFT : 0U) |                       \
	 (DT_INST_PROP(inst, small_hysteresis) ? TMC5262_GCONF_SMALL_HYSTERESIS : 0U) |            \
	 (DT_INST_PROP(inst, stop_enable) ? TMC5262_GCONF_STOP_ENABLE : 0U) |                      \
	 (DT_INST_PROP(inst, ov_output) ? TMC5262_GCONF_OV_NN : 0U))

#define TMC5262_DRV_CONF_VAL(inst)                                                                 \
	(FIELD_PREP(TMC5262_DRV_CONF_CURRENT_RANGE, DT_INST_ENUM_IDX(inst, current_range_ma)) |    \
	 FIELD_PREP(TMC5262_DRV_CONF_CURRENT_RANGE_SCALE,                                          \
		    (DT_INST_PROP(inst, current_range_scale_percent) / 25) - 1) |                  \
	 FIELD_PREP(TMC5262_DRV_CONF_SLOPE_CONTROL, DT_INST_PROP(inst, slope_control)))

#define TMC5262_IHOLD_IRUN_VAL(inst)                                                               \
	(FIELD_PREP(TMC5262_IHOLD_IRUN_IHOLD, DT_INST_PROP(inst, ihold)) |                         \
	 FIELD_PREP(TMC5262_IHOLD_IRUN_IRUN, DT_INST_PROP(inst, irun)) |                           \
	 FIELD_PREP(TMC5262_IHOLD_IRUN_IHOLDDELAY, DT_INST_PROP(inst, iholddelay)) |               \
	 FIELD_PREP(TMC5262_IHOLD_IRUN_IRUNDELAY, DT_INST_PROP(inst, irundelay)))

#define TMC5262_COOLCONF_VAL(inst)                                                                 \
	(FIELD_PREP(TMC5262_COOLCONF_SGT,                                                          \
		    ((uint32_t)DT_INST_PROP(inst, stallguard2_threshold)) & 0x7FU) |               \
	 (DT_INST_PROP(inst, stallguard2_filter) ? TMC5262_COOLCONF_SFILT : 0U))

#define TMC5262_DRV_ENN_FLAGS_OK(inst)                                                             \
	COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, drv_enn_gpios),                                    \
		    ((DT_INST_GPIO_FLAGS(inst, drv_enn_gpios) & GPIO_ACTIVE_LOW) != 0), (1))

#define TMC5262_SLEEPN_FLAGS_OK(inst)                                                              \
	COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, sleepn_gpios),                                     \
		    ((DT_INST_GPIO_FLAGS(inst, sleepn_gpios) & GPIO_ACTIVE_LOW) == 0), (1))

/* Common checks that apply to every variant (FR-INIT-008, FR-BUS-001, FR-CONFIG-001/010) */
#define TMC5262_COMMON_CHECKS(inst)                                                                \
	BUILD_ASSERT(DT_INST_PROP(inst, spi_max_frequency) <= 8000000,                             \
		     "TMC5262: spi-max-frequency must not exceed 8 MHz");                          \
	BUILD_ASSERT(DT_INST_ENUM_IDX(inst, clock_source) == 0 ||                                  \
			     (DT_INST_PROP(inst, clock_frequency) % 1000000 == 0 &&                \
			      DT_INST_PROP(inst, clock_frequency) >= 1000000 &&                    \
			      DT_INST_PROP(inst, clock_frequency) <= 32000000),                    \
		     "TMC5262: external clock must be 1..32 MHz in 1 MHz steps");                  \
	BUILD_ASSERT(DT_INST_PROP(inst, current_range_scale_percent) == 100 ||                     \
			     DT_INST_ENUM_IDX(inst, current_range_ma) == 0,                        \
		     "TMC5262: current-range-scale-percent below 100 requires the lowest range");  \
	BUILD_ASSERT(!(DT_INST_PROP(inst, ov_output) && DT_INST_PROP(inst, encoder_n_channel)),    \
		     "TMC5262: ENCN cannot be overvoltage output and encoder N channel");          \
	BUILD_ASSERT(DT_INST_PROP(inst, toff) > 0,                                                 \
		     "TMC5262: toff = 0 disables the driver, use a value from 1 to 15");           \
	BUILD_ASSERT(TMC5262_DRV_ENN_FLAGS_OK(inst),                                               \
		     "TMC5262: drv-enn-gpios must be GPIO_ACTIVE_LOW (DRV_ENN is active low)");    \
	BUILD_ASSERT(TMC5262_SLEEPN_FLAGS_OK(inst),                                                \
		     "TMC5262: sleepn-gpios must be GPIO_ACTIVE_HIGH");                            \
	BUILD_ASSERT(!DT_INST_NODE_HAS_PROP(inst, do0_gpios) ||                                    \
			     DT_INST_NODE_HAS_PROP(inst, do0_sources),                             \
		     "TMC5262: do0-gpios needs do0-sources, else no event reaches the pin");       \
	BUILD_ASSERT(!DT_INST_NODE_HAS_PROP(inst, do1_gpios) ||                                    \
			     DT_INST_NODE_HAS_PROP(inst, do1_sources),                             \
		     "TMC5262: do1-gpios needs do1-sources, else no event reaches the pin")

#define TMC5262_CLOCK_DIVIDER(inst)                                                                \
	(DT_INST_ENUM_IDX(inst, clock_source) == 0                                                 \
		 ? 15                                                                              \
		 : (DT_INST_PROP(inst, clock_frequency) / 1000000 - 1))

#define TMC5262_CHILD_BY_COMPAT_HELPER(node_id, compat)                                            \
	COND_CODE_1(DT_NODE_HAS_COMPAT(node_id, compat), (node_id), ())

#define TMC5262_CHILD_BY_COMPAT(inst, compat)                                                      \
	DT_FOREACH_CHILD_STATUS_OKAY_VARGS(DT_DRV_INST(inst), TMC5262_CHILD_BY_COMPAT_HELPER,      \
					   compat)

/* The child device pointer is NULL when the child driver is not built */
#define TMC5262_CHILD_DEVICE(inst, kconfig, compat)                                                \
	COND_CODE_1(CONFIG_STEPPER_ADI_TMC5262_##kconfig,                                          \
		    (DEVICE_DT_GET_OR_NULL(TMC5262_CHILD_BY_COMPAT(inst, compat))), (NULL))

#define TMC5262_PLUS_INIT(inst)                                                                    \
	.current_pi = FIELD_PREP(TMC5262_PI_I, DT_INST_PROP(inst, cur_i)) |                        \
		      FIELD_PREP(TMC5262_PI_P, DT_INST_PROP(inst, cur_p)),                         \
	.angle_pi = FIELD_PREP(TMC5262_PI_I, DT_INST_PROP(inst, angle_i)) |                        \
		    FIELD_PREP(TMC5262_PI_P, DT_INST_PROP(inst, angle_p)),                         \
	.cur_angle_limit =                                                                         \
		FIELD_PREP(TMC5262_CUR_ANGLE_LIMIT_CUR, DT_INST_PROP(inst, cur_pi_limit)) |        \
		FIELD_PREP(TMC5262_CUR_ANGLE_LIMIT_ANGLE, DT_INST_PROP(inst, angle_pi_limit)),     \
	.angle_lower_limit =                                                                       \
		FIELD_PREP(TMC5262_ANGLE_LOWER_I_LIMIT, DT_INST_PROP(inst, angle_lower_i_limit)),  \
	.sgp_conf =                                                                                \
		FIELD_PREP(TMC5262_SGP_CONF_THRS, DT_INST_PROP(inst, sgp_thrs)) |                  \
		FIELD_PREP(TMC5262_SGP_CONF_LOW_VEL_SLOPE,                                         \
			   DT_INST_PROP(inst, sgp_low_vel_slope)) |                                \
		(DT_INST_PROP(inst, sgp_filt_en) ? TMC5262_SGP_CONF_FILT_EN : 0U) |                \
		(DT_INST_PROP(inst, sgp_low_vel_freeze) ? TMC5262_SGP_CONF_LOW_VEL_FREEZE : 0U) |  \
		(DT_INST_PROP(inst, sgp_clear_cur_pi) ? TMC5262_SGP_CONF_CLEAR_CUR_PI : 0U),       \
	.tsgp_low_vel_thrs = DT_INST_PROP(inst, tsgp_low_vel_thrs),                                \
	.coolplus_conf =                                                                           \
		FIELD_PREP(TMC5262_COOLPLUS_CONF_CUR_DIV, DT_INST_PROP(inst, cool_cur_div)) |      \
		(DT_INST_PROP(inst, load_filt_en) ? TMC5262_COOLPLUS_CONF_LOAD_FILT_EN : 0U),      \
	.coolplus_pi_reg = FIELD_PREP(TMC5262_PI_I, DT_INST_PROP(inst, cool_i)) |                  \
			   FIELD_PREP(TMC5262_PI_P, DT_INST_PROP(inst, cool_p)),                   \
	.coolplus_pi_down = FIELD_PREP(TMC5262_COOLPLUS_PI_DOWN_OFF_SPEED,                         \
				       DT_INST_PROP(inst, cool_pi_off_speed)) |                    \
			    FIELD_PREP(TMC5262_COOLPLUS_PI_DOWN_LIMIT,                             \
				       DT_INST_PROP(inst, cool_pi_down_limit)),                    \
	.coolplus_reserve = FIELD_PREP(TMC5262_COOLPLUS_RES_HI_GEN,                                \
				       DT_INST_PROP(inst, cool_hi_generatoric_reserve)) |          \
			    FIELD_PREP(TMC5262_COOLPLUS_RES_LOW_GEN,                               \
				       DT_INST_PROP(inst, cool_low_generatoric_reserve)) |         \
			    FIELD_PREP(TMC5262_COOLPLUS_RES_HI_LOAD,                               \
				       DT_INST_PROP(inst, cool_hi_load_reserve)) |                 \
			    FIELD_PREP(TMC5262_COOLPLUS_RES_LOW_LOAD,                              \
				       DT_INST_PROP(inst, cool_low_load_reserve)),                 \
	.udc_conf = FIELD_PREP(TMC5262_UDC_CONF_ACCEL_THRS, DT_INST_PROP(inst, udc_accel_thrs)) |  \
		    FIELD_PREP(TMC5262_UDC_CONF_DECEL_THRS, DT_INST_PROP(inst, udc_decel_thrs)) |  \
		    (DT_INST_PROP(inst, udc_enable) ? TMC5262_UDC_CONF_ENABLE : 0U),               \
	.tudcstep = DT_INST_PROP(inst, tudcstep),                                                  \
	.t_rcoil_meas = DT_INST_PROP(inst, t_rcoil_meas),                                          \
	.coil_induct =                                                                             \
		FIELD_PREP(TMC5262_COIL_INDUCT_VALUE, DT_INST_PROP(inst, coil_inductance)) |       \
		(DT_INST_PROP(inst, rcoil_manual) ? TMC5262_COIL_INDUCT_RCOIL_MANUAL : 0U),        \
	.r_coil_user = FIELD_PREP(TMC5262_R_COIL_A, DT_INST_PROP(inst, rcoil_user_a)) |            \
		       FIELD_PREP(TMC5262_R_COIL_B, DT_INST_PROP(inst, rcoil_user_b))

/* Build-time checks of the StealthChop+ / uDcStep properties (FR-FEATURE-006) */
#define TMC5262_PLUS_CHECKS(inst)                                                                  \
	BUILD_ASSERT(DT_INST_PROP(inst, udc_decel_thrs) > DT_INST_PROP(inst, udc_accel_thrs),      \
		     "TMC5262: udc-decel-thrs must be greater than udc-accel-thrs");               \
	BUILD_ASSERT(!DT_INST_PROP(inst, udc_enable) || DT_INST_PROP(inst, en_stealthchop),        \
		     "TMC5262: udc-enable needs en-stealthchop (uDcStep requires StealthChop+)")

#define TMC5262_SW_MODE_VAL(inst, hard_stop)                                                       \
	((DT_INST_PROP(inst, stop_l_enable) ? TMC5262_SW_MODE_STOP_L_ENABLE : 0U) |                \
	 (DT_INST_PROP(inst, stop_r_enable) ? TMC5262_SW_MODE_STOP_R_ENABLE : 0U) |                \
	 (DT_INST_PROP(inst, pol_stop_l) ? TMC5262_SW_MODE_POL_STOP_L : 0U) |                      \
	 (DT_INST_PROP(inst, pol_stop_r) ? TMC5262_SW_MODE_POL_STOP_R : 0U) |                      \
	 (DT_INST_PROP(inst, swap_lr) ? TMC5262_SW_MODE_SWAP_LR : 0U) |                            \
	 (DT_INST_PROP(inst, latch_l_active) ? TMC5262_SW_MODE_LATCH_L_ACTIVE : 0U) |              \
	 (DT_INST_PROP(inst, latch_l_inactive) ? TMC5262_SW_MODE_LATCH_L_INACTIVE : 0U) |          \
	 (DT_INST_PROP(inst, latch_r_active) ? TMC5262_SW_MODE_LATCH_R_ACTIVE : 0U) |              \
	 (DT_INST_PROP(inst, latch_r_inactive) ? TMC5262_SW_MODE_LATCH_R_INACTIVE : 0U) |          \
	 (DT_INST_PROP(inst, en_latch_encoder) ? TMC5262_SW_MODE_EN_LATCH_ENCODER : 0U) |          \
	 (DT_INST_PROP(inst, sg_stop) ? TMC5262_SW_MODE_SG_STOP : 0U) |                            \
	 (DT_INST_PROP(inst, en_softstop) ? TMC5262_SW_MODE_EN_SOFTSTOP : 0U) |                    \
	 (DT_INST_PROP(inst, en_virtual_stop_l) ? TMC5262_SW_MODE_EN_VIRTUAL_STOP_L : 0U) |        \
	 (DT_INST_PROP(inst, en_virtual_stop_r) ? TMC5262_SW_MODE_EN_VIRTUAL_STOP_R : 0U) |        \
	 (DT_INST_PROP(inst, virtual_stop_enc) ? TMC5262_SW_MODE_VIRTUAL_STOP_ENC : 0U) |          \
	 (hard_stop))

#define TMC5262_RAMPEXT_INIT(inst, hard_stop)                                                      \
	.sw_mode = TMC5262_SW_MODE_VAL(inst, hard_stop),                                           \
	.virtual_stop_l = DT_INST_PROP(inst, virtual_stop_l),                                      \
	.virtual_stop_r = DT_INST_PROP(inst, virtual_stop_r),                                      \
	.x_compare = DT_INST_PROP(inst, x_compare),                                                \
	.x_compare_repeat = DT_INST_PROP(inst, x_compare_repeat)

/* The switch and position compare properties are ramp generator functions (FR-MOTION-003) */
#define TMC5262_RAMPEXT_CHECKS(inst)                                                               \
	BUILD_ASSERT(                                                                              \
		DT_INST_ENUM_IDX(inst, step_source) == 0 ||                                        \
			!(DT_INST_PROP(inst, stop_l_enable) ||                                     \
			  DT_INST_PROP(inst, stop_r_enable) || DT_INST_PROP(inst, pol_stop_l) ||   \
			  DT_INST_PROP(inst, pol_stop_r) || DT_INST_PROP(inst, swap_lr) ||         \
			  DT_INST_PROP(inst, latch_l_active) ||                                    \
			  DT_INST_PROP(inst, latch_l_inactive) ||                                  \
			  DT_INST_PROP(inst, latch_r_active) ||                                    \
			  DT_INST_PROP(inst, latch_r_inactive) ||                                  \
			  DT_INST_PROP(inst, en_latch_encoder) || DT_INST_PROP(inst, sg_stop) ||   \
			  DT_INST_PROP(inst, en_softstop) ||                                       \
			  DT_INST_PROP(inst, en_virtual_stop_l) ||                                 \
			  DT_INST_PROP(inst, en_virtual_stop_r) ||                                 \
			  DT_INST_PROP(inst, virtual_stop_enc) ||                                  \
			  DT_INST_PROP(inst, virtual_stop_l) != 0 ||                               \
			  DT_INST_PROP(inst, virtual_stop_r) != 0),                                \
		"TMC5262: switch and virtual stop properties need step-source = ramp-generator")

/* Datasheet reset values of the registers that devicetree does not configure (per variant) */
#define TMC5262_RESET_SEEDS(inst)                                                                  \
	.enc_const = 0x00010000U, .encmode = FIELD_PREP(TMC5262_ENCMODE_BEMF_BLANK_TIME, 16),      \
	TMC5262_PLUS_INIT(inst),                                                                   \
	TMC5262_RAMPEXT_INIT(inst, DT_INST_PROP(inst, hard_stop_clr_cur_int)                       \
					   ? TMC5262_SW_MODE_HARD_STOP_CLR_CUR_INT                 \
					   : 0U)
#define TMC2262_RESET_SEEDS(inst)                                                                  \
	.enc_const = 0x00010000U, .encmode = FIELD_PREP(TMC5262_ENCMODE_BEMF_BLANK_TIME, 16),      \
	TMC5262_PLUS_INIT(inst)
#define TMC3241_RESET_SEEDS(inst) .enc_const = 0x00010000U, TMC5262_RAMPEXT_INIT(inst, 0U)

/*
 * Shared instance definition. _variant is the descriptor symbol, _gconf / _chopconf /
 * _pwmconf are the per-variant composed register words and _ramp_src the initial step
 * source.
 */
#define TMC5262_INST_DEFINE(tag, inst, _variant, _gconf, _chopconf, _pwmconf, _tpwmthrs, _ramp_src,     \
		       _seeds)                                                                     \
	TMC5262_COMMON_CHECKS(inst);                                                               \
	PM_DEVICE_DT_INST_DEFINE(inst, tmc5262_pm_action);                                         \
	static struct tmc5262_data tmc5262_data_##tag##_##inst;                                    \
	static const struct tmc5262_config tmc5262_config_##tag##_##inst = {                       \
		.bus.spi = SPI_DT_SPEC_INST_GET(inst, (SPI_OP_MODE_CONTROLLER | SPI_TRANSFER_MSB | \
						       SPI_MODE_CPOL | SPI_MODE_CPHA |             \
						       SPI_WORD_SET(8))),                          \
		.bus_io = &tmc5262_spi_bus_io,                                                     \
		.comm_type = TMC_COMM_SPI,                                                         \
		.variant = &(_variant),                                                            \
		.sleepn_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, sleepn_gpios, {0}),                  \
		.drv_enn_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, drv_enn_gpios, {0}),                \
		.do0_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, do0_gpios, {0}),                        \
		.do1_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, do1_gpios, {0}),                        \
		.ext_clock = DT_INST_ENUM_IDX(inst, clock_source) == 1,                            \
		.clock_divider = TMC5262_CLOCK_DIVIDER(inst),                                      \
		.ramp_step_source = (_ramp_src),                                                   \
		.init =                                                                            \
			{                                                                          \
				.gconf = (_gconf),                                                 \
				.do_conf = TMC5262_DO_CONF_VAL(inst),                              \
				.drv_conf = TMC5262_DRV_CONF_VAL(inst),                            \
				.ihold_irun = TMC5262_IHOLD_IRUN_VAL(inst),                        \
				.tpowerdown = DT_INST_PROP(inst, tpowerdown),                      \
				.chopconf = (_chopconf),                                           \
				.coolconf = TMC5262_COOLCONF_VAL(inst),                            \
				.pwmconf = (_pwmconf),                                             \
				.tpwmthrs = (_tpwmthrs),                                           \
				.tcoolthrs = DT_INST_PROP(inst, tcoolthrs),                        \
				.thigh = DT_INST_PROP(inst, thigh),                                \
				.otw_ov_vth = FIELD_PREP(TMC5262_OTW_VTH, TMC5262_ADC_9BIT_MAX) |  \
					      FIELD_PREP(TMC5262_OV_VTH, TMC5262_ADC_9BIT_MAX),    \
				_seeds,                                                            \
			},                                                                         \
		.stepper_driver =                                                                  \
			TMC5262_CHILD_DEVICE(inst, STEPPER_DRIVER, adi_tmc5262_stepper_driver),    \
		.stepper_ctrl =                                                                    \
			TMC5262_CHILD_DEVICE(inst, STEPPER_CTRL, adi_tmc5262_stepper_ctrl),        \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(inst, tmc5262_init, PM_DEVICE_DT_INST_GET(inst),                     \
			      &tmc5262_data_##tag##_##inst, &tmc5262_config_##tag##_##inst,        \
			      POST_KERNEL, CONFIG_STEPPER_ADI_TMC5262_INIT_PRIORITY, NULL);

/* ---- TMC5262 ---- */
#define DT_DRV_COMPAT adi_tmc5262

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

#define TMC5262_CHOPCONF_VAL(inst)                                                                 \
	(FIELD_PREP(TMC5262_CHOPCONF_TOFF, DT_INST_PROP(inst, toff)) |                             \
	 FIELD_PREP(TMC5262_CHOPCONF_TBL, DT_INST_PROP(inst, tbl)) |                               \
	 (DT_INST_PROP(inst, chm) ? TMC5262_CHOPCONF_CHM : 0U) |                                   \
	 FIELD_PREP(TMC5262_CHOPCONF_HSTRT_TFD210, DT_INST_PROP(inst, hstrt_tfd210)) |             \
	 FIELD_PREP(TMC5262_CHOPCONF_HEND_OFFSET, DT_INST_PROP(inst, hend_offset)) |               \
	 FIELD_PREP(TMC5262_CHOPCONF_TPFD, DT_INST_PROP(inst, tpfd)) |                             \
	 (DT_INST_PROP(inst, fd3) ? TMC5262_CHOPCONF_FD3 : 0U) |                                   \
	 (DT_INST_PROP(inst, disfdcc) ? TMC5262_CHOPCONF_DISFDCC : 0U) |                           \
	 (DT_INST_PROP(inst, dedge) ? TMC5262_CHOPCONF_DEDGE : 0U) |                               \
	 (DT_INST_PROP(inst, intpol) ? TMC5262_CHOPCONF_INTPOL : 0U))

#define TMC5262_GCONF_VAL(inst)                                                                    \
	(TMC5262_GCONF_COMMON_VAL(inst) |                                                          \
	 (DT_INST_PROP(inst, en_stealthchop) ? TMC5262_GCONF_EN_STEALTHCHOP : 0U) |                \
	 (DT_INST_ENUM_IDX(inst, step_source) == 1 ? TMC5262_GCONF_STEP_DIR : 0U))

#define TMC5262_PWMCONF_VAL(inst)                                                                  \
	(FIELD_PREP(TMC5262_PWMCONF_PWM_FREQ, DT_INST_PROP(inst, pwm_freq)) |                      \
	 FIELD_PREP(TMC5262_PWMCONF_FREEWHEEL, DT_INST_PROP(inst, freewheel)) |                    \
	 FIELD_PREP(TMC5262_PWMCONF_OL_THRSH, DT_INST_PROP(inst, ol_thrsh)) |                      \
	 FIELD_PREP(TMC5262_PWMCONF_SD_ON_MEAS_LO, DT_INST_PROP(inst, sd_on_meas_lo)) |            \
	 FIELD_PREP(TMC5262_PWMCONF_SD_ON_MEAS_HI, DT_INST_PROP(inst, sd_on_meas_hi)))

#define TMC5262_DEFINE(inst)                                                                       \
	TMC5262_PLUS_CHECKS(inst);                                                                 \
	TMC5262_RAMPEXT_CHECKS(inst);                                                              \
	BUILD_ASSERT(!DT_INST_PROP(inst, udc_enable) || DT_INST_PROP(inst, en_softstop),           \
		     "TMC5262: udc-enable needs en-softstop");                                     \
	BUILD_ASSERT(DT_INST_PROP(inst, sd_on_meas_hi) > DT_INST_PROP(inst, sd_on_meas_lo),        \
		     "TMC5262: sd-on-meas-hi must be greater than sd-on-meas-lo");                 \
	TMC5262_INST_DEFINE(tmc5262, inst, tmc5262_variant_tmc5262, TMC5262_GCONF_VAL(inst),            \
		       TMC5262_CHOPCONF_VAL(inst), TMC5262_PWMCONF_VAL(inst),                      \
		       DT_INST_PROP(inst, tpwmthrs), DT_INST_ENUM_IDX(inst, step_source) == 0,     \
		       TMC5262_RESET_SEEDS(inst))

DT_INST_FOREACH_STATUS_OKAY(TMC5262_DEFINE)

#endif /* TMC5262 */

#undef DT_DRV_COMPAT

/* ---- TMC2262 ---- */
#define DT_DRV_COMPAT adi_tmc2262

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

#define TMC2262_CHOPCONF_VAL(inst)                                                                 \
	(FIELD_PREP(TMC5262_CHOPCONF_TOFF, DT_INST_PROP(inst, toff)) |                             \
	 FIELD_PREP(TMC5262_CHOPCONF_TBL, DT_INST_PROP(inst, tbl)) |                               \
	 (DT_INST_PROP(inst, chm) ? TMC5262_CHOPCONF_CHM : 0U) |                                   \
	 FIELD_PREP(TMC5262_CHOPCONF_HSTRT_TFD210, DT_INST_PROP(inst, hstrt_tfd210)) |             \
	 FIELD_PREP(TMC5262_CHOPCONF_HEND_OFFSET, DT_INST_PROP(inst, hend_offset)) |               \
	 FIELD_PREP(TMC5262_CHOPCONF_TPFD, DT_INST_PROP(inst, tpfd)) |                             \
	 (DT_INST_PROP(inst, fd3) ? TMC5262_CHOPCONF_FD3 : 0U) |                                   \
	 (DT_INST_PROP(inst, dedge) ? TMC5262_CHOPCONF_DEDGE : 0U) |                               \
	 (DT_INST_PROP(inst, intpol) ? TMC5262_CHOPCONF_INTPOL : 0U))

#define TMC2262_GCONF_VAL(inst)                                                                    \
	(TMC5262_GCONF_COMMON_VAL(inst) |                                                          \
	 (DT_INST_PROP(inst, en_stealthchop) ? TMC5262_GCONF_EN_STEALTHCHOP : 0U))

#define TMC2262_PWMCONF_VAL(inst)                                                                  \
	(FIELD_PREP(TMC5262_PWMCONF_PWM_FREQ, DT_INST_PROP(inst, pwm_freq)) |                      \
	 FIELD_PREP(TMC5262_PWMCONF_FREEWHEEL, DT_INST_PROP(inst, freewheel)) |                    \
	 FIELD_PREP(TMC5262_PWMCONF_OL_THRSH, DT_INST_PROP(inst, ol_thrsh)) |                      \
	 FIELD_PREP(TMC5262_PWMCONF_SD_ON_MEAS_LO, DT_INST_PROP(inst, sd_on_meas_lo)) |            \
	 FIELD_PREP(TMC5262_PWMCONF_SD_ON_MEAS_HI, DT_INST_PROP(inst, sd_on_meas_hi)))

#define TMC2262_DEFINE(inst)                                                                       \
	TMC5262_PLUS_CHECKS(inst);                                                                 \
	BUILD_ASSERT(DT_INST_PROP(inst, sd_on_meas_hi) > DT_INST_PROP(inst, sd_on_meas_lo),        \
		     "TMC2262: sd-on-meas-hi must be greater than sd-on-meas-lo");                 \
	TMC5262_INST_DEFINE(tmc2262, inst, tmc5262_variant_tmc2262, TMC2262_GCONF_VAL(inst),            \
		       TMC2262_CHOPCONF_VAL(inst), TMC2262_PWMCONF_VAL(inst),                      \
		       DT_INST_PROP(inst, tpwmthrs), false, TMC2262_RESET_SEEDS(inst))

DT_INST_FOREACH_STATUS_OKAY(TMC2262_DEFINE)

#endif /* TMC2262 */

#undef DT_DRV_COMPAT

/* ---- TMC3241 ---- */
#define DT_DRV_COMPAT adi_tmc3241

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

#define TMC3241_CHOPCONF_VAL(inst)                                                                 \
	(FIELD_PREP(TMC5262_CHOPCONF_TOFF, DT_INST_PROP(inst, toff)) |                             \
	 FIELD_PREP(TMC5262_CHOPCONF_TBL, DT_INST_PROP(inst, tbl)) |                               \
	 FIELD_PREP(TMC3241_CHOPCONF_HYST, DT_INST_PROP(inst, hyst)) |                             \
	 (DT_INST_PROP(inst, nosd) ? TMC3241_CHOPCONF_NOSD : 0U) |                                 \
	 (DT_INST_PROP(inst, csync) ? TMC3241_CHOPCONF_CSYNC : 0U) |                               \
	 (DT_INST_PROP(inst, sg_speed) ? TMC3241_CHOPCONF_SG_SPEED : 0U) |                         \
	 (DT_INST_PROP(inst, dedge) ? TMC5262_CHOPCONF_DEDGE : 0U) |                               \
	 (DT_INST_PROP(inst, intpol) ? TMC5262_CHOPCONF_INTPOL : 0U))

#define TMC3241_GCONF_VAL(inst)                                                                    \
	(TMC5262_GCONF_COMMON_VAL(inst) |                                                          \
	 (DT_INST_ENUM_IDX(inst, step_source) == 0 ? TMC3241_GCONF_THREEPHASE_MCC : 0U))

#define TMC3241_DEFINE(inst)                                                                       \
	TMC5262_RAMPEXT_CHECKS(inst);                                                              \
	TMC5262_INST_DEFINE(tmc3241, inst, tmc5262_variant_tmc3241, TMC3241_GCONF_VAL(inst),            \
		       TMC3241_CHOPCONF_VAL(inst), 0U, 0U,                                         \
		       DT_INST_ENUM_IDX(inst, step_source) == 0, TMC3241_RESET_SEEDS(inst))

DT_INST_FOREACH_STATUS_OKAY(TMC3241_DEFINE)

#endif /* TMC3241 */

#undef DT_DRV_COMPAT
