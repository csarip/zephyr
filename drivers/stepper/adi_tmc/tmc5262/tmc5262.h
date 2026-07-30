/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-FileCopyrightText: Copyright (c) 2026 Analog Devices Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_STEPPER_ADI_TMC5262_H
#define ZEPHYR_DRIVERS_STEPPER_ADI_TMC5262_H

#include <zephyr/drivers/stepper/stepper.h>
#include <zephyr/drivers/stepper/stepper_ctrl.h>
#include <zephyr/drivers/stepper/stepper_trinamic.h>

/**
 * @brief Trigger the registered callback for stepper driver events.
 *
 * @param dev Pointer to the stepper driver device.
 * @param event The stepper driver event that occurred.
 */
void tmc5262_stepper_driver_trigger_cb(const struct device *dev, const enum stepper_event event);

/**
 * @brief Trigger the registered callback for motion controller events.
 *
 * @param dev Pointer to the motion controller device.
 * @param event The motion controller event that occurred.
 */
void tmc5262_stepper_ctrl_trigger_cb(const struct device *dev, const enum stepper_ctrl_event event);

/**
 * @brief Enable or disable the StallGuard feature.
 *
 * @param dev Pointer to the motion controller device.
 * @param enable true to enable, false to disable.
 * @retval -EIO on failure, -EAGAIN if velocity is too low, 0 on success.
 */
int tmc5262_stepper_ctrl_stallguard_enable(const struct device *dev, const bool enable);

/**
 * @brief Read the actual position from the TMC5262 device.
 *
 * @param dev Pointer to the TMC5262 controller device.
 * @param position Pointer to store the actual position in microsteps.
 * @retval -EIO on failure, 0 on success.
 */
int tmc5262_read_actual_position(const struct device *dev, int32_t *position);

/**
 * @brief Reschedule the ramp status callback work item.
 *
 * @param dev Pointer to the TMC5262 controller device.
 */
void tmc5262_reschedule_rampstat_callback(const struct device *dev);

/**
 * @brief Write a 32-bit value to a TMC5262 register.
 *
 * @param dev Pointer to the TMC5262 controller device.
 * @param reg_addr Register address to write to.
 * @param reg_val Value to write to the register.
 * @retval -EIO on failure, 0 on success.
 */
int tmc5262_write(const struct device *dev, const uint8_t reg_addr, const uint32_t reg_val);

/**
 * @brief Read a 32-bit value from a TMC5262 register.
 *
 * @param dev Pointer to the TMC5262 controller device.
 * @param reg_addr Register address to read from.
 * @param reg_val Pointer to store the read value.
 * @retval -EIO on failure, 0 on success.
 */
int tmc5262_read(const struct device *dev, const uint8_t reg_addr, uint32_t *reg_val);

/**
 * @brief Read-modify-write a masked field of a TMC5262 register.
 *
 * @param dev Pointer to the TMC5262 controller device.
 * @param reg_addr Register address to update.
 * @param mask Bitmask of the field to update.
 * @param value Field value (unshifted) to write.
 * @retval -EIO on failure, 0 on success.
 */
int tmc5262_update(const struct device *dev, const uint8_t reg_addr, uint32_t mask, uint32_t value);

/**
 * @brief Check if the TMC5262 controller uses an interrupt-driven approach.
 *
 * @param dev Pointer to the TMC5262 controller device.
 * @return true if interrupt driven (DIAG0 pin present), false otherwise.
 */
bool tmc5262_is_interrupt_driven(const struct device *dev);

/**
 * @brief Get the clock frequency in Hz of the TMC5262 device.
 *
 * @param dev Pointer to the TMC5262 controller device.
 * @return Clock frequency in Hz.
 */
int tmc5262_get_clock_frequency(const struct device *dev);

#endif /* ZEPHYR_DRIVERS_STEPPER_ADI_TMC5262_H */
