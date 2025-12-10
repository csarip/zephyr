/* SPDX-License-Identifier: Apache-2.0 */
/*
 * SPDX-FileCopyrightText: Copyright (c) 2025 Cherrence Sarip <Cherrence.sarip@analog.com>
 */

#ifndef ZEPHYR_DRIVERS_STEPPER_TMC5222_H_
#define ZEPHYR_DRIVERS_STEPPER_TMC5222_H_

#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name TMC5222 Register addresses
 * @{
 */
#define TMC5222_REG_GCONF 0x00
/** @} */

/**
 * @name TMC5222 GCONF register bits
 * @{
 */
#define TMC5222_GCONF_EN_PWM_MODE BIT(0) /* 0: SpreadCycle, 1: StealthChop2 */
#define TMC5222_GCONF_DRV_ENN     BIT(9) /* 1: Driver disable, 0: Driver enable */
/** @} */

/**
 * @brief TMC5222 device configuration (compile-time constant)
 */
struct tmc5222_config {
	/** SPI bus specification */
	struct spi_dt_spec spi;
	/** Optional reset GPIO specification */
	struct gpio_dt_spec reset_gpio;
	/** Optional enable GPIO specification (general device enable) */
	struct gpio_dt_spec enable_gpio;
	/** Optional driver enable GPIO specification (DRVEN pin) */
	struct gpio_dt_spec drven_gpio;
	/** Flag indicating if reset GPIO is present */
	bool has_reset;
	/** Flag indicating if enable GPIO is present */
	bool has_enable;
	/** Flag indicating if drven GPIO is present */
	bool has_drven;
	/** Enable StealthChop2 mode (true=StealthChop2, false=SpreadCycle) */
	bool enable_stealthchop;
};

/**
 * @brief TMC5222 runtime data
 */
struct tmc5222_data {
	/** Cached microstep resolution (placeholder for future use) */
	uint32_t cached_microstep_res;
};

/**
 * @brief Write a 32-bit register to the TMC5222 via SPI
 *
 * @param dev      Pointer to the TMC5222 device
 * @param reg_addr Register address (0x00-0x7F)
 * @param value    32-bit value to write
 * @return         0 on success, negative errno on failure
 */
int tmc5222_reg_write(const struct device *dev, uint8_t reg_addr, uint32_t value);

/**
 * @brief Read a 32-bit register from the TMC5222 via SPI
 *
 * @param dev      Pointer to the TMC5222 device
 * @param reg_addr Register address (0x00-0x7F)
 * @param value    Pointer to store the read value
 * @return         0 on success, negative errno on failure
 */
int tmc5222_reg_read(const struct device *dev, uint8_t reg_addr, uint32_t *value);

/**
 * @brief Burst-read N consecutive 32-bit registers from TMC5222
 *
 * For SPI, this performs individual reads for each register.
 *
 * @param dev        Pointer to the TMC5222 device
 * @param start_addr Starting register address
 * @param buf        Buffer to receive N*4 bytes
 * @param num_regs   Number of 32-bit registers to read
 * @return           0 on success, negative errno on failure
 */
int tmc5222_burst_read(const struct device *dev, uint8_t start_addr, uint8_t *buf, size_t num_regs);

/**
 * @brief Set chopper mode (SpreadCycle or StealthChop2)
 *
 * Configures the EN_PWM_MODE bit in GCONF register to select between
 * SpreadCycle (traditional chopper) or StealthChop2 (quiet) mode.
 *
 * @param dev          Pointer to the TMC5222 device
 * @param stealthchop2 true for StealthChop2, false for SpreadCycle
 * @return             0 on success, negative errno on failure
 */
int tmc5222_set_chopper_mode(const struct device *dev, bool stealthchop2);

/**
 * @brief Enable the driver stage via register (clears drv_enn bit)
 *
 * Enables the power MOSFETs by clearing the drv_enn bit (bit 9) in GCONF register.
 * This allows the motor to be energized and produce torque.
 *
 * Note: Actual driver enable = DRV_EN && drv_enn && (TOFF != 0)
 * This function sets drv_enn=0 (enabled) while preserving TOFF value.
 *
 * @param dev Pointer to the TMC5222 device
 * @return    0 on success, negative errno on failure
 */
int tmc5222_driver_enable(const struct device *dev);

/**
 * @brief Disable the driver stage via register (sets drv_enn bit)
 *
 * Disables the power MOSFETs by setting the drv_enn bit (bit 9) in GCONF register.
 * This puts the motor into freewheeling mode while preserving all register content
 * including TOFF configuration.
 *
 * Note: Actual driver enable = DRV_EN && drv_enn && (TOFF != 0)
 * This function sets drv_enn=1 (disabled) to switch off the driver stage.
 *
 * @param dev Pointer to the TMC5222 device
 * @return    0 on success, negative errno on failure
 */
int tmc5222_driver_disable(const struct device *dev);

/**
 * @brief Enable driver via DRVEN pin (hardware emergency stop release)
 *
 * Drives the DRVEN pin high to enable the driver output stage. This is
 * an asynchronous hardware control that bypasses digital blocks.
 *
 * @param dev Pointer to the TMC5222 device
 * @return    0 on success, negative errno on failure (or 0 if no GPIO configured)
 */
int tmc5222_drven_enable(const struct device *dev);

/**
 * @brief Disable driver via DRVEN pin (hardware emergency stop)
 *
 * Drives the DRVEN pin low to immediately tri-state the driver output stage.
 * This provides a hardware-based safety channel independent of MCU software.
 * Torque is immediately removed from motor. Register content is preserved.
 *
 * @param dev Pointer to the TMC5222 device
 * @return    0 on success, negative errno on failure (or 0 if no GPIO configured)
 */
int tmc5222_drven_disable(const struct device *dev);

/**
 * @brief Enable the TMC5222 via optional enable GPIO
 *
 * @param dev Pointer to the TMC5222 device
 * @return    0 on success, negative errno on failure (or 0 if no GPIO configured)
 */
int tmc5222_enable(const struct device *dev);

/**
 * @brief Disable the TMC5222 via optional enable GPIO
 *
 * @param dev Pointer to the TMC5222 device
 * @return    0 on success, negative errno on failure (or 0 if no GPIO configured)
 */
int tmc5222_disable(const struct device *dev);

/**
 * @brief Hardware reset the TMC5222 via optional reset GPIO
 *
 * Toggles the reset pin LOW then HIGH with 1ms delay.
 *
 * @param dev Pointer to the TMC5222 device
 * @return    0 on success, negative errno on failure (or 0 if no GPIO configured)
 */
int tmc5222_hw_reset(const struct device *dev);

#endif /* ZEPHYR_DRIVERS_STEPPER_TMC5222_H_ */
