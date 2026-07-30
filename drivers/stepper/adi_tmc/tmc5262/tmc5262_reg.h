/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-FileCopyrightText: Copyright (c) 2026 Analog Devices Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file drivers/stepper/adi_tmc/tmc5262/tmc5262_reg.h
 *
 * @brief TMC5262 register addresses and field definitions.
 *
 * @details Register addresses and field masks/shifts are taken from the ADI
 * TMC5262 datasheet and the public ADI TMC-API hardware abstraction header.
 * Only the subset required by the Zephyr driver is defined here.
 */

#ifndef ZEPHYR_DRIVERS_STEPPER_ADI_TMC_TMC5262_TMC5262_REG_H_
#define ZEPHYR_DRIVERS_STEPPER_ADI_TMC_TMC5262_TMC5262_REG_H_

#include <zephyr/sys/util.h>

#include <adi_tmc_reg.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name TMC5262 register addresses
 * @{
 */
#define TMC5262_DIAG_CONF                 0x02U
#define TMC5262_IOIN                      0x04U
#define TMC5262_PLL                       0x0BU
#define TMC5262_IHOLD_IRUN                0x10U
#define TMC5262_TPOWER_DOWN               0x11U
#define TMC5262_TSTEP                     0x12U
#define TMC5262_TPWMTHRS                  0x13U
#define TMC5262_TCOOLTHRS                 0x14U
#define TMC5262_THIGH                     0x15U
#define TMC5262_T_RCOIL_MEAS              0x17U
#define TMC5262_RAMPMODE                  0x20U
#define TMC5262_XACTUAL                   0x21U
#define TMC5262_VACTUAL                   0x22U
#define TMC5262_VSTART                    0x23U
#define TMC5262_A1                        0x24U
#define TMC5262_V1                        0x25U
#define TMC5262_AMAX                      0x26U
#define TMC5262_VMAX                      0x27U
#define TMC5262_DMAX                      0x28U
#define TMC5262_D1                        0x2AU
#define TMC5262_VSTOP                     0x2BU
#define TMC5262_TZEROWAIT                 0x2CU
#define TMC5262_XTARGET                   0x2DU
#define TMC5262_V2                        0x2EU
#define TMC5262_A2                        0x2FU
#define TMC5262_D2                        0x30U
#define TMC5262_SWMODE                    0x34U
#define TMC5262_RAMPSTAT                  0x35U
#define TMC5262_COIL_INDUCT               0x46U
#define TMC5262_SGP_CONF                  0x49U
#define TMC5262_COOLSTEPPLUS_CONF         0x4EU
#define TMC5262_COOLSTEPPLUS_LOAD_RESERVE 0x52U
#define TMC5262_TSTEP_VELOCITY            0x53U
#define TMC5262_CHOPCONF                  0x6CU
#define TMC5262_DRVSTATUS                 0x6FU
/** @} */

/**
 * @name TMC5262 GCONF (0x00) field shifts
 * @{
 */
#define TMC5262_GCONF_FAST_STANDSTILL_SHIFT  0
#define TMC5262_GCONF_EN_STEALTHCHOP_SHIFT   1
#define TMC5262_GCONF_MULTISTEP_FILT_SHIFT   2
#define TMC5262_GCONF_SHAFT_SHIFT            3
#define TMC5262_GCONF_SMALL_HYSTERESIS_SHIFT 4
#define TMC5262_GCONF_STOP_ENABLE_SHIFT      5
#define TMC5262_GCONF_DIRECT_MODE_SHIFT      6
#define TMC5262_GCONF_LENGTH_STEPPULSE_SHIFT 8
#define TMC5262_GCONF_OV_NN_SHIFT            12
#define TMC5262_GCONF_STEP_DIR_SHIFT         31
/** @} */

/**
 * @name TMC5262 PLL (0x0B) field masks
 * @{
 */
/* clk_ext: select external clock source (bit 0). */
#define TMC5262_CLOCK_SOURCE_MASK  BIT(0)
/* clk_divider: PLL input divider (bits 9:5). */
#define TMC5262_CLOCK_DIVIDER_MASK GENMASK(9, 5)
/* Clock status flags: clk_1mo_tmo, clk_loss, clk_is_stuck (bits 14:12). */
#define TMC5262_CLOCK_FLAGS_MASK   GENMASK(14, 12)
/** @} */

/**
 * @name TMC5262 DIAG_CONF (0x02) rampstat-to-DIAG0 mapping masks
 * @{
 */
#define TMC5262_MAP_RAMPSTAT_STOP_LR_DIAG0_MASK     BIT(9)
#define TMC5262_MAP_RAMPSTAT_SG_STOP_DIAG0_MASK     BIT(10)
#define TMC5262_MAP_RAMPSTAT_POS_REACHED_DIAG0_MASK BIT(11)
/** @} */

/**
 * @name TMC5262 miscellaneous field masks
 * @{
 */
#define TMC5262_TSTEP_MASK          GENMASK(19, 0)
#define TMC5262_TSTEP_VELOCITY_MASK GENMASK(22, 0)
#define TMC5262_SGP_THRS_MASK       GENMASK(8, 0)
#define TMC5262_SGP_RESULT_MASK     GENMASK(9, 0)
#define TMC5262_COIL_INDUCT_MASK    GENMASK(14, 0)
#define TMC5262_T_RCOIL_MEAS_MASK   GENMASK(19, 0)
#define TMC5262_COOL_CUR_DIV_MASK   GENMASK(3, 0)
#define TMC5262_SG_STOP_MASK        BIT(10)
/** @} */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_DRIVERS_STEPPER_ADI_TMC_TMC5262_TMC5262_REG_H_ */
