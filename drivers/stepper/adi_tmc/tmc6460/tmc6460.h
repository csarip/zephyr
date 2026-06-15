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

/* CHIP registers */
#define TMC6460_CHIP_ID                        0x0000U
#define TMC6460_CHIP_VARIANT                   0x0001U
#define TMC6460_CHIP_REVISION                  0x0002U
#define TMC6460_CHIP_INPUTS_RAW                0x0004U
#define TMC6460_CHIP_OUTPUTS_RAW               0x0005U
#define TMC6460_CHIP_IO_MATRIX                 0x0006U
#define TMC6460_CHIP_IO_PU_PD                  0x0007U
#define TMC6460_CHIP_IO_CONFIG                 0x0008U
#define TMC6460_CHIP_STATUS_FLAGS              0x0009U
#define TMC6460_CHIP_EVENTS                    0x000AU
#define TMC6460_CHIP_FAULTN_INT_MASK           0x000BU
#define TMC6460_CHIP_SPI_STATUS_MASK           0x000CU

/* CLK_CTRL registers */
#define TMC6460_CLK_CTRL_CONFIG                0x0040U
#define TMC6460_CLK_CTRL_STATUS                0x0041U

/* ADC registers */
#define TMC6460_ADC_CONFIG                     0x0080U
#define TMC6460_ADC_ADC_VERSION                0x0081U
#define TMC6460_ADC_I2_I1_RAW                  0x0082U
#define TMC6460_ADC_VM_I3_RAW                  0x0083U
#define TMC6460_ADC_TEMP_RAW                   0x0084U
#define TMC6460_ADC_AIN_V_AIN_U_RAW           0x0085U
#define TMC6460_ADC_AIN_AIN_W_RAW             0x0086U
#define TMC6460_ADC_STATUS                     0x008AU

/* MCC_CONFIG registers */
#define TMC6460_MCC_CONFIG_GDRV               0x0101U
#define TMC6460_MCC_CONFIG_PWM                0x0102U
#define TMC6460_MCC_CONFIG_PWM_PERIOD         0x0103U
#define TMC6460_MCC_CONFIG_BRAKE_CHOPPER      0x0104U
#define TMC6460_MCC_CONFIG_MCC_STATUS         0x0105U

/* FOC registers */
#define TMC6460_FOC_PID_CONFIG                0x0140U
#define TMC6460_FOC_PID_U_S_MAX               0x0141U
#define TMC6460_FOC_PWM_V_U                   0x0158U

/* RAMPER registers */
#define TMC6460_RAMPER_TIME_CONFIG             0x01C0U
#define TMC6460_RAMPER_SWITCH_MODE             0x01C1U
#define TMC6460_RAMPER_PHI_E                   0x01C3U
#define TMC6460_RAMPER_A1                      0x01C4U
#define TMC6460_RAMPER_A2                      0x01C5U
#define TMC6460_RAMPER_A_MAX                   0x01C6U
#define TMC6460_RAMPER_D1                      0x01C7U
#define TMC6460_RAMPER_D2                      0x01C8U
#define TMC6460_RAMPER_D_MAX                   0x01C9U
#define TMC6460_RAMPER_V_START                 0x01CAU
#define TMC6460_RAMPER_V1                      0x01CBU
#define TMC6460_RAMPER_V2                      0x01CCU
#define TMC6460_RAMPER_V_STOP                  0x01CDU
#define TMC6460_RAMPER_V_MAX                   0x01CEU
#define TMC6460_RAMPER_ACCELERATION            0x01CFU
#define TMC6460_RAMPER_V_ACTUAL                0x01D0U
#define TMC6460_RAMPER_POSITION                0x01D1U
#define TMC6460_RAMPER_POSITION_LATCH          0x01D2U
#define TMC6460_RAMPER_POSITION_ACTUAL_LATCH   0x01D3U
#define TMC6460_RAMPER_STATUS                  0x01D4U
#define TMC6460_RAMPER_EVENTS                  0x01D5U

/**
 * @brief Write a value to a TMC6460 register.
 */
int tmc6460_write(const struct device *dev, uint16_t reg_addr, uint32_t reg_val);

/**
 * @brief Read a value from a TMC6460 register.
 */
int tmc6460_read(const struct device *dev, uint16_t reg_addr, uint32_t *reg_val);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_DRIVERS_STEPPER_ADI_TMC_TMC6460_H_ */
