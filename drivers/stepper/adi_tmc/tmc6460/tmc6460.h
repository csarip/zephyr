/*
 * SPDX-FileCopyrightText: Copyright (c) 2026 Cherrence Sarip <cherrence.sarip@analog.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_STEPPER_ADI_TMC_TMC6460_H_
#define ZEPHYR_DRIVERS_STEPPER_ADI_TMC_TMC6460_H_

#include <zephyr/types.h>
#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

/* TMC6460 register addresses (16-bit) */
#define TMC6460_REG_CHIP_ID            0x000U
#define TMC6460_REG_CHIP_IO_CONFIG     0x008U

/**
 * @brief Write a value to a TMC6460 register.
 *
 * @param dev Pointer to the TMC6460 device.
 * @param reg_addr 16-bit register address.
 * @param reg_val 32-bit value to write.
 *
 * @retval 0 on success.
 * @retval -errno on SPI failure.
 */
int tmc6460_write(const struct device *dev, uint16_t reg_addr, uint32_t reg_val);

/**
 * @brief Read a value from a TMC6460 register.
 *
 * @param dev Pointer to the TMC6460 device.
 * @param reg_addr 16-bit register address.
 * @param reg_val Pointer to store the 32-bit read value.
 *
 * @retval 0 on success.
 * @retval -errno on SPI failure.
 */
int tmc6460_read(const struct device *dev, uint16_t reg_addr, uint32_t *reg_val);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_DRIVERS_STEPPER_ADI_TMC_TMC6460_H_ */
