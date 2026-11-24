/*
 * SPDX-FileCopyrightText: Copyright (c) 2026 Analog Devices, Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Register map and bit-field definitions for TMC5262, TMC2262 and TMC3241
 *
 * Register addresses are shared by all three devices. Bit positions that differ
 * between devices are grouped by device (TMC5262_*, TMC2262_*, TMC3241_*) and are
 * only consumed by the variant descriptors (tmc5262_variant.c) and the devicetree
 * register-word composition macros. Core code accesses variant-dependent fields
 * through the descriptor masks only.
 */

#ifndef ZEPHYR_DRIVERS_STEPPER_ADI_TMC_TMC5262_TMC5262_REG_H_
#define ZEPHYR_DRIVERS_STEPPER_ADI_TMC_TMC5262_TMC5262_REG_H_

#include <zephyr/sys/util.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @name Register addresses (shared by all variants, availability per variant) */
/** @{ */
#define TMC5262_REG_GCONF                     0x00
#define TMC5262_REG_GSTAT                     0x01
#define TMC5262_REG_DO_CONF                   0x02
#define TMC5262_REG_IOIN                      0x04
#define TMC5262_REG_X_COMPARE                 0x05
#define TMC5262_REG_X_COMPARE_REPEAT          0x06
#define TMC5262_REG_DRV_CONF                  0x0A
#define TMC5262_REG_PLL                       0x0B
#define TMC5262_REG_IHOLD_IRUN                0x10
#define TMC5262_REG_TPOWERDOWN                0x11
#define TMC5262_REG_TSTEP                     0x12
#define TMC5262_REG_TPWMTHRS                  0x13
#define TMC5262_REG_TCOOLTHRS                 0x14
#define TMC5262_REG_THIGH                     0x15
#define TMC5262_REG_TSGP_LOW_VEL_THRS         0x16
#define TMC5262_REG_T_RCOIL_MEAS              0x17
#define TMC5262_REG_TUDCSTEP                  0x18
#define TMC5262_REG_UDC_CONF                  0x19
#define TMC5262_REG_STEPS_LOST                0x1A
#define TMC5262_REG_RAMPMODE                  0x20
#define TMC5262_REG_XACTUAL                   0x21
#define TMC5262_REG_VACTUAL                   0x22
#define TMC5262_REG_VSTART                    0x23
#define TMC5262_REG_A1                        0x24
#define TMC5262_REG_V1                        0x25
#define TMC5262_REG_AMAX                      0x26
#define TMC5262_REG_VMAX                      0x27
#define TMC5262_REG_DMAX                      0x28
#define TMC5262_REG_TVMAX                     0x29
#define TMC5262_REG_D1                        0x2A
#define TMC5262_REG_VSTOP                     0x2B
#define TMC5262_REG_TZEROWAIT                 0x2C
#define TMC5262_REG_XTARGET                   0x2D
#define TMC5262_REG_V2                        0x2E
#define TMC5262_REG_A2                        0x2F
#define TMC5262_REG_D2                        0x30
#define TMC5262_REG_AACTUAL                   0x31
#define TMC5262_REG_SW_MODE                   0x34
#define TMC5262_REG_RAMP_STAT                 0x35
#define TMC5262_REG_XLATCH                    0x36
#define TMC5262_REG_ENCMODE                   0x38
#define TMC5262_REG_X_ENC                     0x39
#define TMC5262_REG_ENC_CONST                 0x3A
#define TMC5262_REG_ENC_STATUS                0x3B
#define TMC5262_REG_ENC_LATCH                 0x3C
#define TMC5262_REG_ENC_DEVIATION             0x3D
#define TMC5262_REG_VIRTUAL_STOP_L            0x3E
#define TMC5262_REG_VIRTUAL_STOP_R            0x3F
#define TMC5262_REG_CURRENT_PI_REG            0x40
#define TMC5262_REG_ANGLE_PI_REG              0x41
#define TMC5262_REG_CUR_ANGLE_LIMIT           0x42
#define TMC5262_REG_ANGLE_LOWER_LIMIT         0x43
#define TMC5262_REG_CUR_ANGLE_MEAS            0x44
#define TMC5262_REG_PI_RESULTS                0x45
#define TMC5262_REG_COIL_INDUCT               0x46
#define TMC5262_REG_R_COIL                    0x47
#define TMC5262_REG_R_COIL_USER               0x48
#define TMC5262_REG_SGP_CONF                  0x49
#define TMC5262_REG_SGP_IND_2_3               0x4A
#define TMC5262_REG_SGP_IND_0_1               0x4B
#define TMC5262_REG_INDUCTANCE_VOLTAGE        0x4C
#define TMC5262_REG_SGP_BEMF                  0x4D
#define TMC5262_REG_COOLSTEPPLUS_CONF         0x4E
#define TMC5262_REG_COOLSTEPPLUS_PI_REG       0x4F
#define TMC5262_REG_COOLSTEPPLUS_PI_DOWN      0x50
#define TMC5262_REG_COOLSTEPPLUS_RESERVE_CONF 0x51
#define TMC5262_REG_COOLSTEPPLUS_LOAD_RESERVE 0x52
#define TMC5262_REG_ADC_VSUPPLY_TEMP          0x58
#define TMC5262_REG_ADC_I                     0x59
#define TMC5262_REG_OTW_OV_VTH                0x5A
#define TMC5262_REG_MSLUT_0                   0x60
#define TMC5262_REG_MSLUT_7                   0x67
#define TMC5262_REG_MSLUTSEL                  0x68
#define TMC5262_REG_MSLUTSTART                0x69
#define TMC5262_REG_MSCNT                     0x6A
#define TMC5262_REG_MSCURACT                  0x6B
#define TMC5262_REG_CHOPCONF                  0x6C
#define TMC5262_REG_COOLCONF                  0x6D
#define TMC5262_REG_DRV_STATUS                0x6F
#define TMC5262_REG_PWMCONF                   0x70
/** @} */

/** @name SPI datagram and SPI_STATUS byte */
/** @{ */
#define TMC5262_SPI_STATUS_STANDSTILL       BIT(3)
#define TMC5262_SPI_STATUS_SG               BIT(2)
#define TMC5262_SPI_STATUS_DRIVER_ERROR     BIT(1)
#define TMC5262_SPI_STATUS_RESET_FLAG       BIT(0)
#define TMC5262_SPI_STATUS_VELOCITY_REACHED BIT(4)
#define TMC5262_SPI_STATUS_POSITION_REACHED BIT(5)
#define TMC5262_SPI_STATUS_STOP_L           BIT(6)
#define TMC5262_SPI_STATUS_STOP_R           BIT(7)
/** Bits 7..4 are only valid on variants with a ramp generator */
#define TMC5262_SPI_STATUS_RAMP_BITS        GENMASK(7, 4)
/** @} */

/** @name GCONF fields shared by all variants */
/** @{ */
#define TMC5262_GCONF_FAST_STANDSTILL  BIT(0)
#define TMC5262_GCONF_MULTISTEP_FILT   BIT(2)
#define TMC5262_GCONF_SHAFT            BIT(3)
#define TMC5262_GCONF_SMALL_HYSTERESIS BIT(4)
#define TMC5262_GCONF_STOP_ENABLE      BIT(5)
#define TMC5262_GCONF_DIRECT_MODE      BIT(6)
#define TMC5262_GCONF_LENGTH_STEPPULSE GENMASK(11, 8)
#define TMC5262_GCONF_OV_NN            BIT(12)
/** @} */

/** @name GCONF fields that differ per variant */
/** @{ */
/** TMC5262: 1 = STEP/DIR is the step source, 0 = ramp generator */
#define TMC5262_GCONF_STEP_DIR       BIT(31)
/** TMC5262 and TMC2262: StealthChop+ enable */
#define TMC5262_GCONF_EN_STEALTHCHOP BIT(1)
/** TMC3241: 1 = ramp generator is the step source, 0 = STEP/DIR */
#define TMC3241_GCONF_THREEPHASE_MCC BIT(30)
/** @} */

/** @name GSTAT flags (write 1 to clear) */
/** @{ */
#define TMC5262_GSTAT_RESET          BIT(0)
#define TMC5262_GSTAT_DRV_ERR        BIT(1)
#define TMC5262_GSTAT_UV_CP          BIT(2)
#define TMC5262_GSTAT_REGISTER_RESET BIT(3)
#define TMC5262_GSTAT_VM_UVLO        BIT(4)
#define TMC5262_GSTAT_VCCIO_UV       BIT(5)
#define TMC5262_GSTAT_ALL            GENMASK(5, 0)
/** Flags that are cleared after power-up (FR-INIT-009) */
#define TMC5262_GSTAT_POWERUP_CLEAR                                                                \
	(TMC5262_GSTAT_RESET | TMC5262_GSTAT_REGISTER_RESET | TMC5262_GSTAT_VM_UVLO |              \
	 TMC5262_GSTAT_UV_CP | TMC5262_GSTAT_VCCIO_UV)
/** @} */

/** @name DO_CONF fields (output type and polarity are common) */
/** @{ */
#define TMC5262_DO_CONF_DO0_NOD_PP     BIT(28)
#define TMC5262_DO_CONF_DO0_INVPP      BIT(29)
#define TMC5262_DO_CONF_DO1_NOD_PP     BIT(30)
#define TMC5262_DO_CONF_DO1_INVPP      BIT(31)
/** DO1 source bits are the DO0 source bits shifted by 13 */
#define TMC5262_DO_CONF_DO1_SHIFT      13
/**
 * DO0 source bit positions. Tokens are the (underscored) DT string values; a
 * token missing for a variant is rejected by the binding enum at build time.
 */
#define TMC5262_DO0_SRC_error          BIT(0)
#define TMC5262_DO0_SRC_otpw           BIT(1)
#define TMC5262_DO0_SRC_stall          BIT(2)
#define TMC5262_DO0_SRC_index          BIT(3)
#define TMC5262_DO0_SRC_step           BIT(4)
#define TMC5262_DO0_SRC_dir            BIT(5)
#define TMC5262_DO0_SRC_xcomp          BIT(6)
#define TMC5262_DO0_SRC_ov             BIT(7)
#define TMC5262_DO0_SRC_udcstep        BIT(8)
#define TMC5262_DO0_SRC_ev_stop_ref    BIT(9)
#define TMC5262_DO0_SRC_ev_stop_sg     BIT(10)
#define TMC5262_DO0_SRC_ev_pos_reached BIT(11)
#define TMC5262_DO0_SRC_ev_n_deviation BIT(12)
#define TMC5262_DO1_SRC_error          (TMC5262_DO0_SRC_error << 13)
#define TMC5262_DO1_SRC_otpw           (TMC5262_DO0_SRC_otpw << 13)
#define TMC5262_DO1_SRC_stall          (TMC5262_DO0_SRC_stall << 13)
#define TMC5262_DO1_SRC_index          (TMC5262_DO0_SRC_index << 13)
#define TMC5262_DO1_SRC_step           (TMC5262_DO0_SRC_step << 13)
#define TMC5262_DO1_SRC_dir            (TMC5262_DO0_SRC_dir << 13)
#define TMC5262_DO1_SRC_xcomp          (TMC5262_DO0_SRC_xcomp << 13)
#define TMC5262_DO1_SRC_ov             (TMC5262_DO0_SRC_ov << 13)
#define TMC5262_DO1_SRC_udcstep        (TMC5262_DO0_SRC_udcstep << 13)
#define TMC5262_DO1_SRC_ev_stop_ref    (TMC5262_DO0_SRC_ev_stop_ref << 13)
#define TMC5262_DO1_SRC_ev_stop_sg     (TMC5262_DO0_SRC_ev_stop_sg << 13)
#define TMC5262_DO1_SRC_ev_pos_reached (TMC5262_DO0_SRC_ev_pos_reached << 13)
#define TMC5262_DO1_SRC_ev_n_deviation (TMC5262_DO0_SRC_ev_n_deviation << 13)
/** @} */

/** @name IOIN fields */
/** @{ */
#define TMC5262_IOIN_SILICON_RV  GENMASK(18, 16)
#define TMC5262_IOIN_EXT_RES_DET BIT(13)
/** IOIN.DRV_ENN: 0 = power stage enabled, 1 = disabled */
#define TMC5262_IOIN_DRV_ENN     BIT(4)
#define TMC5262_IOIN_EXT_CLK     BIT(14)
/** @} */

/** @name DRV_CONF fields */
/** @{ */
#define TMC5262_DRV_CONF_CURRENT_RANGE       GENMASK(1, 0)
#define TMC5262_DRV_CONF_CURRENT_RANGE_SCALE GENMASK(3, 2)
#define TMC5262_DRV_CONF_SLOPE_CONTROL       GENMASK(5, 4)
#define TMC5262_CURRENT_SCALE_100            3
/** @} */

/** @name PLL fields */
/** @{ */
#define TMC5262_PLL_COMMIT        BIT(0)
#define TMC5262_PLL_EXT_NOT_INT   BIT(1)
#define TMC5262_PLL_CLK_SYS_SEL   BIT(2)
#define TMC5262_PLL_CLK_FSM_ENA   BIT(3)
#define TMC5262_PLL_CLOCK_DIVIDER GENMASK(9, 5)
#define TMC5262_PLL_CLK_1M0_TMO   BIT(12)
#define TMC5262_PLL_CLK_LOSS      BIT(13)
#define TMC5262_PLL_CLK_IS_STUCK  BIT(14)
#define TMC5262_PLL_FLAGS                                                                          \
	(TMC5262_PLL_CLK_1M0_TMO | TMC5262_PLL_CLK_LOSS | TMC5262_PLL_CLK_IS_STUCK)
/** @} */

/** @name IHOLD_IRUN fields (8-bit currents on all variants) */
/** @{ */
#define TMC5262_IHOLD_IRUN_IHOLD      GENMASK(7, 0)
#define TMC5262_IHOLD_IRUN_IRUN       GENMASK(15, 8)
#define TMC5262_IHOLD_IRUN_IHOLDDELAY GENMASK(23, 16)
#define TMC5262_IHOLD_IRUN_IRUNDELAY  GENMASK(27, 24)
/** @} */

/** @name Register field widths */
/** @{ */
#define TMC5262_TPOWERDOWN_MASK       GENMASK(7, 0)
#define TMC5262_VELOCITY_THRS_MASK    GENMASK(19, 0)
#define TMC5262_VSTART_MASK           GENMASK(17, 0)
#define TMC5262_A1_MASK               GENMASK(17, 0)
#define TMC5262_V1_MASK               GENMASK(19, 0)
#define TMC5262_AMAX_MASK             GENMASK(17, 0)
#define TMC5262_VMAX_MASK             GENMASK(22, 0)
/** VMAX limit of the datasheet: 0 .. 2^23 - 512 */
#define TMC5262_VMAX_MAX              (BIT(23) - 512U)
#define TMC5262_X_COMPARE_REPEAT_MASK GENMASK(23, 0)
#define TMC5262_DMAX_MASK             GENMASK(17, 0)
#define TMC5262_D1_MASK               GENMASK(17, 0)
#define TMC5262_VSTOP_MASK            GENMASK(17, 0)
#define TMC5262_TZEROWAIT_MASK        GENMASK(15, 0)
#define TMC5262_V2_MASK               GENMASK(19, 0)
#define TMC5262_A2_MASK               GENMASK(17, 0)
#define TMC5262_D2_MASK               GENMASK(17, 0)
#define TMC5262_VACTUAL_SIGN_BIT      23
#define TMC5262_AACTUAL_SIGN_BIT      23
/** @} */

/** @name RAMPMODE values */
/** @{ */
#define TMC5262_RAMPMODE_POSITION     0
#define TMC5262_RAMPMODE_VELOCITY_POS 1
#define TMC5262_RAMPMODE_VELOCITY_NEG 2
#define TMC5262_RAMPMODE_HOLD         3
/** @} */

/** @name SW_MODE fields (identical on TMC5262 and TMC3241) */
/** @{ */
#define TMC5262_SW_MODE_STOP_L_ENABLE         BIT(0)
#define TMC5262_SW_MODE_STOP_R_ENABLE         BIT(1)
#define TMC5262_SW_MODE_POL_STOP_L            BIT(2)
#define TMC5262_SW_MODE_POL_STOP_R            BIT(3)
#define TMC5262_SW_MODE_SWAP_LR               BIT(4)
#define TMC5262_SW_MODE_LATCH_L_ACTIVE        BIT(5)
#define TMC5262_SW_MODE_LATCH_L_INACTIVE      BIT(6)
#define TMC5262_SW_MODE_LATCH_R_ACTIVE        BIT(7)
#define TMC5262_SW_MODE_LATCH_R_INACTIVE      BIT(8)
#define TMC5262_SW_MODE_EN_LATCH_ENCODER      BIT(9)
#define TMC5262_SW_MODE_SG_STOP               BIT(10)
#define TMC5262_SW_MODE_EN_SOFTSTOP           BIT(11)
#define TMC5262_SW_MODE_EN_VIRTUAL_STOP_L     BIT(12)
#define TMC5262_SW_MODE_EN_VIRTUAL_STOP_R     BIT(13)
#define TMC5262_SW_MODE_VIRTUAL_STOP_ENC      BIT(14)
#define TMC5262_SW_MODE_HARD_STOP_CLR_CUR_INT BIT(15)
/** @} */

/** @name RAMP_STAT fields (event bits are write-1-to-clear) */
/** @{ */
#define TMC5262_RAMP_STAT_STATUS_STOP_L         BIT(0)
#define TMC5262_RAMP_STAT_STATUS_STOP_R         BIT(1)
#define TMC5262_RAMP_STAT_STATUS_LATCH_L        BIT(2)
#define TMC5262_RAMP_STAT_STATUS_LATCH_R        BIT(3)
#define TMC5262_RAMP_STAT_EVENT_STOP_L          BIT(4)
#define TMC5262_RAMP_STAT_EVENT_STOP_R          BIT(5)
#define TMC5262_RAMP_STAT_EVENT_STOP_SG         BIT(6)
#define TMC5262_RAMP_STAT_EVENT_POS_REACHED     BIT(7)
#define TMC5262_RAMP_STAT_VELOCITY_REACHED      BIT(8)
#define TMC5262_RAMP_STAT_POSITION_REACHED      BIT(9)
#define TMC5262_RAMP_STAT_VZERO                 BIT(10)
#define TMC5262_RAMP_STAT_T_ZEROWAIT_ACTIVE     BIT(11)
#define TMC5262_RAMP_STAT_SECOND_MOVE           BIT(12)
#define TMC5262_RAMP_STAT_STATUS_SG             BIT(13)
#define TMC5262_RAMP_STAT_STATUS_VIRTUAL_STOP_L BIT(14)
#define TMC5262_RAMP_STAT_STATUS_VIRTUAL_STOP_R BIT(15)
#define TMC5262_RAMP_STAT_EVENTS                                                                   \
	(TMC5262_RAMP_STAT_EVENT_STOP_L | TMC5262_RAMP_STAT_EVENT_STOP_R |                         \
	 TMC5262_RAMP_STAT_EVENT_STOP_SG | TMC5262_RAMP_STAT_EVENT_POS_REACHED)
/** @} */

/** @name ENCMODE fields */
/** @{ */
#define TMC5262_ENCMODE_POL_A           BIT(0)
#define TMC5262_ENCMODE_POL_B           BIT(1)
#define TMC5262_ENCMODE_POL_N           BIT(2)
#define TMC5262_ENCMODE_IGNORE_AB       BIT(3)
#define TMC5262_ENCMODE_CLR_CONT        BIT(4)
#define TMC5262_ENCMODE_CLR_ONCE        BIT(5)
#define TMC5262_ENCMODE_POS_NEG_EDGE    GENMASK(7, 6)
#define TMC5262_ENCMODE_CLR_ENC_X       BIT(8)
#define TMC5262_ENCMODE_LATCH_X_ACT     BIT(9)
#define TMC5262_ENCMODE_ENC_SEL_DECIMAL BIT(10)
/** Back-EMF (TriCoder) fields, TMC5262 and TMC2262 only */
#define TMC5262_ENCMODE_NBEMF_ABN_SEL   BIT(11)
#define TMC5262_ENCMODE_BEMF_HYST       GENMASK(14, 12)
#define TMC5262_ENCMODE_BEMF_BLANK_TIME GENMASK(23, 16)
#define TMC5262_ENCMODE_BEMF_FILTER_SEL GENMASK(29, 28)
/** Fields available on every variant */
#define TMC5262_ENCMODE_COMMON_MASK     GENMASK(10, 0)
#define TMC5262_ENCMODE_BEMF_MASK                                                                  \
	(TMC5262_ENCMODE_NBEMF_ABN_SEL | TMC5262_ENCMODE_BEMF_HYST |                               \
	 TMC5262_ENCMODE_BEMF_BLANK_TIME | TMC5262_ENCMODE_BEMF_FILTER_SEL)
#define TMC5262_ENC_STATUS_N_EVENT        BIT(0)
#define TMC5262_ENC_STATUS_DEVIATION_WARN BIT(1)
/** @} */

/** @name CHOPCONF fields shared by all variants */
/** @{ */
#define TMC5262_CHOPCONF_TOFF   GENMASK(3, 0)
#define TMC5262_CHOPCONF_TBL    GENMASK(16, 15)
#define TMC5262_CHOPCONF_CHM    BIT(14)
#define TMC5262_CHOPCONF_MRES   GENMASK(27, 24)
#define TMC5262_CHOPCONF_INTPOL BIT(28)
#define TMC5262_CHOPCONF_DEDGE  BIT(29)
/** @} */

/** @name CHOPCONF fields of TMC5262 / TMC2262 */
/** @{ */
#define TMC5262_CHOPCONF_HSTRT_TFD210 GENMASK(6, 4)
#define TMC5262_CHOPCONF_HEND_OFFSET  GENMASK(10, 7)
#define TMC5262_CHOPCONF_FD3          BIT(11)
/** TMC5262 only */
#define TMC5262_CHOPCONF_DISFDCC      BIT(12)
#define TMC5262_CHOPCONF_TPFD         GENMASK(23, 20)
/** @} */

/** @name CHOPCONF fields of TMC3241 */
/** @{ */
#define TMC3241_CHOPCONF_HYST     GENMASK(9, 4)
#define TMC3241_CHOPCONF_NOSD     BIT(10)
#define TMC3241_CHOPCONF_SG_SPEED BIT(11)
#define TMC3241_CHOPCONF_CSYNC    BIT(12)
/** @} */

/** @name COOLCONF fields (StallGuard2 and CoolStep) */
/** @{ */
#define TMC5262_COOLCONF_SEMIN        GENMASK(3, 0)
#define TMC5262_COOLCONF_SEUP         GENMASK(6, 5)
#define TMC5262_COOLCONF_SEMAX        GENMASK(11, 8)
#define TMC5262_COOLCONF_SEDN         GENMASK(14, 12)
#define TMC5262_COOLCONF_SEIMIN       BIT(15)
#define TMC5262_COOLCONF_SGT          GENMASK(22, 16)
#define TMC5262_COOLCONF_THIGH_SG_OFF BIT(23)
#define TMC5262_COOLCONF_SFILT        BIT(24)
/** @} */

/** @name DRV_STATUS fields independent of the variant */
/** @{ */
#define TMC5262_DRV_STATUS_SG_RESULT GENMASK(9, 0)
#define TMC5262_DRV_STATUS_CS_ACTUAL GENMASK(23, 16)
/** @} */

/** @name DRV_STATUS fields of TMC5262 / TMC2262 (per-coil flags) */
/** @{ */
#define TMC5262_DRV_STATUS_SEQ_STOPPED BIT(10)
#define TMC5262_DRV_STATUS_OV          BIT(11)
#define TMC5262_DRV_STATUS_S2VSA       BIT(12)
#define TMC5262_DRV_STATUS_S2VSB       BIT(13)
#define TMC5262_DRV_STATUS_STEALTH     BIT(14)
#define TMC5262_DRV_STATUS_STALLGUARD  BIT(24)
#define TMC5262_DRV_STATUS_OT          BIT(25)
#define TMC5262_DRV_STATUS_OTPW        BIT(26)
#define TMC5262_DRV_STATUS_S2GA        BIT(27)
#define TMC5262_DRV_STATUS_S2GB        BIT(28)
#define TMC5262_DRV_STATUS_OLA         BIT(29)
#define TMC5262_DRV_STATUS_OLB         BIT(30)
#define TMC5262_DRV_STATUS_STST        BIT(31)
/** @} */

/** @name DRV_STATUS fields of TMC3241 (combined flags) */
/** @{ */
#define TMC3241_DRV_STATUS_SEQ_STOPPED BIT(10)
#define TMC3241_DRV_STATUS_OV          BIT(11)
#define TMC3241_DRV_STATUS_S2VS        BIT(12)
#define TMC3241_DRV_STATUS_STALLGUARD  BIT(24)
#define TMC3241_DRV_STATUS_OT          BIT(25)
#define TMC3241_DRV_STATUS_OTPW        BIT(26)
#define TMC3241_DRV_STATUS_S2G         BIT(27)
#define TMC3241_DRV_STATUS_OL          BIT(29)
#define TMC3241_DRV_STATUS_STST        BIT(31)
/** @} */

/** @name PWMCONF fields (StealthChop+, TMC5262 and TMC2262) */
/** @{ */
#define TMC5262_PWMCONF_PWM_FREQ      GENMASK(3, 0)
#define TMC5262_PWMCONF_FREEWHEEL     GENMASK(5, 4)
#define TMC5262_PWMCONF_OL_THRSH      GENMASK(7, 6)
#define TMC5262_PWMCONF_SD_ON_MEAS_LO GENMASK(15, 12)
#define TMC5262_PWMCONF_SD_ON_MEAS_HI GENMASK(19, 16)
/** @} */

/** @name UDC_CONF fields (uDcStep, TMC5262 and TMC2262; ACCEL_THRS exists on the TMC5262 only) */
/** @{ */
#define TMC5262_UDC_CONF_DECEL_THRS GENMASK(3, 0)
#define TMC5262_UDC_CONF_ACCEL_THRS GENMASK(7, 4)
#define TMC5262_UDC_CONF_ENABLE     BIT(8)
/** @} */

/** @name ADC registers */
/** @{ */
#define TMC5262_ADC_VSUPPLY  GENMASK(8, 0)
#define TMC5262_ADC_TEMP     GENMASK(24, 16)
#define TMC5262_ADC_I_A      GENMASK(11, 0)
#define TMC5262_ADC_I_B      GENMASK(27, 16)
#define TMC5262_OTW_VTH      GENMASK(24, 16)
#define TMC5262_OV_VTH       GENMASK(8, 0)
#define TMC5262_ADC_9BIT_MAX 0x1FF
/** @} */

/** @name COIL_INDUCT fields */
/** @{ */
#define TMC5262_COIL_INDUCT_VALUE                  GENMASK(14, 0)
#define TMC5262_COIL_INDUCT_RCOIL_MANUAL           BIT(16)
#define TMC5262_COIL_INDUCT_RCOIL_THERMAL_COUPLING BIT(17)
/** @} */

/** @name R_COIL and R_COIL_USER fields */
/** @{ */
#define TMC5262_R_COIL_A GENMASK(27, 16)
#define TMC5262_R_COIL_B GENMASK(11, 0)
/** @} */

/** @name StealthChop+ regulators, StallGuard+, CoolStep+ fields (TMC5262, TMC2262) */
/** @{ */
#define TMC5262_PI_I                       GENMASK(25, 16)
#define TMC5262_PI_P                       GENMASK(11, 0)
#define TMC5262_PI_REG_MASK                (TMC5262_PI_I | TMC5262_PI_P)
#define TMC5262_CUR_ANGLE_LIMIT_CUR        GENMASK(27, 16)
#define TMC5262_CUR_ANGLE_LIMIT_ANGLE      GENMASK(9, 0)
#define TMC5262_CUR_ANGLE_LIMIT_MASK       (GENMASK(27, 16) | GENMASK(9, 0))
#define TMC5262_ANGLE_LOWER_I_LIMIT        GENMASK(9, 0)
#define TMC5262_SGP_CONF_THRS              GENMASK(8, 0)
#define TMC5262_SGP_CONF_FILT_EN           BIT(12)
#define TMC5262_SGP_CONF_LOW_VEL_FREEZE    BIT(13)
#define TMC5262_SGP_CONF_CLEAR_CUR_PI      BIT(14)
#define TMC5262_SGP_CONF_LOW_VEL_SLOPE     GENMASK(23, 16)
#define TMC5262_COOLPLUS_CONF_CUR_DIV      GENMASK(3, 0)
#define TMC5262_COOLPLUS_CONF_LOAD_FILT_EN BIT(4)
#define TMC5262_COOLPLUS_CONF_MASK         GENMASK(4, 0)
#define TMC5262_COOLPLUS_PI_DOWN_OFF_SPEED GENMASK(27, 16)
#define TMC5262_COOLPLUS_PI_DOWN_LIMIT     GENMASK(11, 0)
#define TMC5262_COOLPLUS_PI_DOWN_MASK                                                              \
	(TMC5262_COOLPLUS_PI_DOWN_OFF_SPEED | TMC5262_COOLPLUS_PI_DOWN_LIMIT)
#define TMC5262_COOLPLUS_RES_HI_GEN   GENMASK(31, 24)
#define TMC5262_COOLPLUS_RES_LOW_GEN  GENMASK(23, 16)
#define TMC5262_COOLPLUS_RES_HI_LOAD  GENMASK(15, 8)
#define TMC5262_COOLPLUS_RES_LOW_LOAD GENMASK(7, 0)
#define TMC5262_SGP_BEMF_UBEMF_ABS    GENMASK(27, 16)
#define TMC5262_SGP_BEMF_SGP_RAW      GENMASK(9, 0)
/** @} */

/** @name Misc */
/** @{ */
#define TMC5262_MSCURACT_CUR_A GENMASK(24, 16)
#define TMC5262_MSCURACT_CUR_B GENMASK(8, 0)
#define TMC5262_MSCNT_MASK     GENMASK(9, 0)
#define TMC5262_SGT_MIN        (-64)
#define TMC5262_SGT_MAX        63
/** @} */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_DRIVERS_STEPPER_ADI_TMC_TMC5262_TMC5262_REG_H_ */
