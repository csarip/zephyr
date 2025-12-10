/*
 * SPDX-FileCopyrightText: Copyright (c) 2025 Cherrence Sarip <Cherrence.sarip@analog.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT adi_tmc5222

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

#include <adi_tmc_spi.h>
#include <adi_tmc_reg.h>

#include "tmc5222.h"

LOG_MODULE_REGISTER(tmc5222, CONFIG_LOG_DEFAULT_LEVEL);

/* TMC5222 register definitions */
#define TMC5222_WRITE_BIT    0x80U
#define TMC5222_ADDRESS_MASK 0x7FU

int tmc5222_reg_write(const struct device *dev, uint8_t reg_addr, uint32_t value)
{
	const struct tmc5222_config *cfg = dev->config;
	int err;

	err = tmc_spi_write_register(&cfg->spi, TMC5222_WRITE_BIT, reg_addr, value);
	if (err < 0) {
		LOG_ERR("Failed to write register 0x%02x with value 0x%08x: %d",
			reg_addr, value, err);
		return err;
	}

	LOG_DBG("WRITE reg 0x%02X <= 0x%08X", reg_addr, value);
	return 0;
}

int tmc5222_reg_read(const struct device *dev, uint8_t reg_addr, uint32_t *value)
{
	const struct tmc5222_config *cfg = dev->config;
	int err;

	err = tmc_spi_read_register(&cfg->spi, TMC5222_ADDRESS_MASK, reg_addr, value);
	if (err < 0) {
		LOG_ERR("Failed to read register 0x%02x: %d", reg_addr, err);
		return err;
	}

	LOG_DBG("READ reg 0x%02X => 0x%08X", reg_addr, *value);
	return 0;
}

int tmc5222_burst_read(const struct device *dev, uint8_t start_addr, uint8_t *buf, size_t num_regs)
{
	int err;

	/* SPI requires individual reads for each register */
	for (size_t i = 0; i < num_regs; i++) {
		uint32_t value;

		err = tmc5222_reg_read(dev, start_addr + i, &value);
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

int tmc5222_set_chopper_mode(const struct device *dev, bool stealthchop2)
{
	uint32_t gconf;
	int ret = tmc5222_reg_read(dev, TMC5222_REG_GCONF, &gconf);

	if (ret) {
		LOG_ERR("Failed to read GCONF: %d", ret);
		return ret;
	}

	if (stealthchop2) {
		gconf |= TMC5222_GCONF_EN_PWM_MODE;  /* Enable StealthChop2 */
	} else {
		gconf &= ~TMC5222_GCONF_EN_PWM_MODE; /* Enable SpreadCycle */
	}

	ret = tmc5222_reg_write(dev, TMC5222_REG_GCONF, gconf);
	if (ret) {
		LOG_ERR("Failed to write GCONF: %d", ret);
	}
	return ret;
}

int tmc5222_driver_enable(const struct device *dev)
{
	uint32_t gconf;
	int ret = tmc5222_reg_read(dev, TMC5222_REG_GCONF, &gconf);

	if (ret) {
		LOG_ERR("Failed to read GCONF: %d", ret);
		return ret;
	}

	/* Clear drv_enn (bit 9) to enable driver stage */
	/* EN = DRV_EN && !drv_enn && (TOFF != 0) */
	gconf &= ~TMC5222_GCONF_DRV_ENN;

	ret = tmc5222_reg_write(dev, TMC5222_REG_GCONF, gconf);
	if (ret) {
		LOG_ERR("Failed to write GCONF: %d", ret);
	}
	return ret;
}

int tmc5222_driver_disable(const struct device *dev)
{
	uint32_t gconf;
	int ret = tmc5222_reg_read(dev, TMC5222_REG_GCONF, &gconf);

	if (ret) {
		LOG_ERR("Failed to read GCONF: %d", ret);
		return ret;
	}

	/* Set drv_enn (bit 9) to disable driver stage (freewheeling) */
	/* This preserves TOFF value while disabling outputs */
	gconf |= TMC5222_GCONF_DRV_ENN;

	ret = tmc5222_reg_write(dev, TMC5222_REG_GCONF, gconf);
	if (ret) {
		LOG_ERR("Failed to write GCONF: %d", ret);
	}
	return ret;
}

int tmc5222_drven_enable(const struct device *dev)
{
	const struct tmc5222_config *cfg = dev->config;
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

int tmc5222_drven_disable(const struct device *dev)
{
	const struct tmc5222_config *cfg = dev->config;
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

int tmc5222_enable(const struct device *dev)
{
	const struct tmc5222_config *cfg = dev->config;
	int ret;

	if (!cfg->has_enable) {
		return 0;
	}
	ret = gpio_pin_set_dt(&cfg->enable_gpio, 1);
	if (ret) {
		LOG_ERR("Failed to set enable GPIO: %d", ret);
	}
	return ret;
}

int tmc5222_disable(const struct device *dev)
{
	const struct tmc5222_config *cfg = dev->config;
	int ret;

	if (!cfg->has_enable) {
		return 0;
	}
	ret = gpio_pin_set_dt(&cfg->enable_gpio, 0);
	if (ret) {
		LOG_ERR("Failed to clear enable GPIO: %d", ret);
	}
	return ret;
}

int tmc5222_hw_reset(const struct device *dev)
{
	const struct tmc5222_config *cfg = dev->config;

	if (!cfg->has_reset) {
		return 0;
	}

	/* Toggle reset: LOW -> HIGH */
	int ret = gpio_pin_set_dt(&cfg->reset_gpio, 0);

	if (ret) {
		LOG_ERR("Failed to set reset GPIO low: %d", ret);
		return ret;
	}
	k_msleep(1);
	ret = gpio_pin_set_dt(&cfg->reset_gpio, 1);
	if (ret) {
		LOG_ERR("Failed to set reset GPIO high: %d", ret);
	}
	return ret;
}

static int tmc5222_init(const struct device *dev)
{
	const struct tmc5222_config *cfg = dev->config;

	if (!spi_is_ready_dt(&cfg->spi)) {
		LOG_ERR("SPI bus not ready");
		return -ENODEV;
	}

	/* Configure optional GPIOs */
	if (cfg->has_enable) {
		if (!gpio_is_ready_dt(&cfg->enable_gpio)) {
			LOG_ERR("Enable GPIO not ready");
			return -ENODEV;
		}
		int ret = gpio_pin_configure_dt(&cfg->enable_gpio, GPIO_OUTPUT_INACTIVE);

		if (ret) {
			LOG_ERR("Failed to configure enable GPIO");
			return ret;
		}
	}

	if (cfg->has_reset) {
		if (!gpio_is_ready_dt(&cfg->reset_gpio)) {
			LOG_ERR("Reset GPIO not ready");
			return -ENODEV;
		}
		int ret = gpio_pin_configure_dt(&cfg->reset_gpio, GPIO_OUTPUT_ACTIVE);

		if (ret) {
			LOG_ERR("Failed to configure reset GPIO");
			return ret;
		}
	}

	if (cfg->has_drven) {
		if (!gpio_is_ready_dt(&cfg->drven_gpio)) {
			LOG_ERR("DRVEN GPIO not ready");
			return -ENODEV;
		}
		int ret = gpio_pin_configure_dt(&cfg->drven_gpio, GPIO_OUTPUT_ACTIVE);

		if (ret) {
			LOG_ERR("Failed to configure DRVEN GPIO");
			return ret;
		}
	}

	/* Optional: enable device and perform hardware reset */
	int ret = tmc5222_enable(dev);

	if (ret) {
		LOG_WRN("Failed to enable device during init: %d", ret);
	}

	ret = tmc5222_hw_reset(dev);
	if (ret) {
		LOG_WRN("Failed to reset device during init: %d", ret);
	}

	/* Set default chopper mode from device tree */
	ret = tmc5222_set_chopper_mode(dev, cfg->enable_stealthchop);
	if (ret) {
		LOG_WRN("Failed to set default chopper mode: %d", ret);
	} else {
		LOG_INF("Chopper mode: %s",
			cfg->enable_stealthchop ? "StealthChop2" : "SpreadCycle");
	}

	LOG_INF("TMC5222 init OK on SPI bus");
	return 0;
}

#define TMC5222_DEFINE(inst)                                                                       \
	static struct tmc5222_data tmc5222_data_##inst;                                            \
	static const struct tmc5222_config tmc5222_config_##inst = {                               \
		.spi = SPI_DT_SPEC_INST_GET(                                                       \
			inst,                                                                      \
			(SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB | SPI_MODE_CPOL | SPI_MODE_CPHA |  \
			 SPI_WORD_SET(8)),                                                         \
			0U),                                                                       \
		.enable_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, enable_gpios, {0}),                 \
		.reset_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, reset_gpios, {0}),                   \
		.drven_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, drven_gpios, {0}),                   \
		.has_enable = DT_INST_NODE_HAS_PROP(inst, enable_gpios),                           \
		.has_reset = DT_INST_NODE_HAS_PROP(inst, reset_gpios),                             \
		.has_drven = DT_INST_NODE_HAS_PROP(inst, drven_gpios),                             \
		.enable_stealthchop =                                                              \
			DT_INST_STRING_TOKEN(inst, chopper_mode) == stealthchop2,                 \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(inst, tmc5222_init, NULL, &tmc5222_data_##inst,                      \
			      &tmc5222_config_##inst, POST_KERNEL,                                 \
			      CONFIG_KERNEL_INIT_PRIORITY_DEVICE, NULL);

DT_INST_FOREACH_STATUS_OKAY(TMC5222_DEFINE)

