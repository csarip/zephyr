/*
 * SPDX-FileCopyrightText: Copyright (c) 2026 Analog Devices, Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Per-variant descriptors. This is the only place (besides the devicetree register
 * composition macros in tmc5262.c) that holds bit positions specific to one device.
 */

#include "tmc5262.h"

/* Registers common to all three variants (SRS section "Variant Differences") */
#define TMC5262_COMMON_REGS                                                                        \
	{TMC5262_REG_GCONF, TMC5262_REG_GSTAT}, {TMC5262_REG_DO_CONF, TMC5262_REG_DO_CONF},        \
		{TMC5262_REG_IOIN, TMC5262_REG_IOIN}, {TMC5262_REG_DRV_CONF, TMC5262_REG_PLL},     \
		{TMC5262_REG_IHOLD_IRUN, TMC5262_REG_TSTEP},                                       \
		{TMC5262_REG_TCOOLTHRS, TMC5262_REG_THIGH},                                        \
		{TMC5262_REG_STEPS_LOST, TMC5262_REG_STEPS_LOST},                                  \
		{TMC5262_REG_ENCMODE, TMC5262_REG_ENC_LATCH},                                      \
		{TMC5262_REG_ADC_VSUPPLY_TEMP, TMC5262_REG_ADC_VSUPPLY_TEMP},                      \
		{TMC5262_REG_OTW_OV_VTH, TMC5262_REG_OTW_OV_VTH},                                  \
		{TMC5262_REG_MSLUT_0, TMC5262_REG_MSCURACT},                                       \
		{TMC5262_REG_CHOPCONF, TMC5262_REG_COOLCONF},                                      \
	{                                                                                          \
		TMC5262_REG_DRV_STATUS, TMC5262_REG_DRV_STATUS                                     \
	}

/* Ramp generator, switches, position compare and ramp status (TMC5262 and TMC3241) */
#define TMC5262_RAMP_REGS                                                                          \
	{TMC5262_REG_X_COMPARE, TMC5262_REG_X_COMPARE_REPEAT},                                     \
		{TMC5262_REG_RAMPMODE, TMC5262_REG_AACTUAL},                                       \
		{TMC5262_REG_SW_MODE, TMC5262_REG_XLATCH},                                         \
	{                                                                                          \
		TMC5262_REG_VIRTUAL_STOP_L, TMC5262_REG_VIRTUAL_STOP_R                             \
	}

/* StealthChop+, StallGuard+, CoolStep+, uDcStep, coil measurement (TMC5262 and TMC2262) */
#define TMC5262_PLUS_REGS                                                                          \
	{TMC5262_REG_TPWMTHRS, TMC5262_REG_TPWMTHRS},                                              \
		{TMC5262_REG_TSGP_LOW_VEL_THRS, TMC5262_REG_UDC_CONF},                             \
		{TMC5262_REG_ENC_DEVIATION, TMC5262_REG_ENC_DEVIATION},                            \
		{TMC5262_REG_CURRENT_PI_REG, TMC5262_REG_COOLSTEPPLUS_LOAD_RESERVE},               \
		{TMC5262_REG_ADC_I, TMC5262_REG_ADC_I},                                            \
	{                                                                                          \
		TMC5262_REG_PWMCONF, TMC5262_REG_PWMCONF                                           \
	}

/* clang-format would put the brace of ".status = {" on its own line, checkpatch forbids that */
/* clang-format off */
#ifdef CONFIG_STEPPER_ADI_TMC5262_TMC5262
static const struct tmc5262_reg_range tmc5262_regs[] = {
	TMC5262_COMMON_REGS,
	TMC5262_RAMP_REGS,
	TMC5262_PLUS_REGS,
};

const struct tmc5262_variant tmc5262_variant_tmc5262 = {
	.name = "TMC5262",
	.id = TMC5262_VARIANT_TMC5262,
	.caps = TMC5262_CAP_RAMP_GEN | TMC5262_CAP_REF_SWITCH | TMC5262_CAP_POS_COMPARE |
		TMC5262_CAP_STEALTHCHOP | TMC5262_CAP_STALLGUARD_PLUS | TMC5262_CAP_COOLSTEP_PLUS |
		TMC5262_CAP_UDCSTEP | TMC5262_CAP_COIL_MEAS | TMC5262_CAP_ADC_I |
		TMC5262_CAP_ENC_DEVIATION | TMC5262_CAP_DIRECT_MODE | TMC5262_CAP_TRICODER |
		TMC5262_CAP_PER_COIL_FAULT | TMC5262_CAP_XLATCH | TMC5262_CAP_VACTUAL |
		TMC5262_CAP_STEP_SOURCE_SEL,
	.twake_ms = 2,
	.n_phases = 2,
	.step_src_mask = TMC5262_GCONF_STEP_DIR,
	.step_src_ramp_when_set = false,
	.gconf_stealth_mask = TMC5262_GCONF_EN_STEALTHCHOP,
	.encmode_mask = TMC5262_ENCMODE_COMMON_MASK | TMC5262_ENCMODE_BEMF_MASK,
	.bemf_mask = TMC5262_ENCMODE_BEMF_MASK,
	.bemf_encoder_sel = TMC5262_ENCMODE_NBEMF_ABN_SEL,
	.udc_accel_mask = TMC5262_UDC_CONF_ACCEL_THRS,
	.chopconf_mask = TMC5262_CHOPCONF_TOFF | TMC5262_CHOPCONF_TBL | TMC5262_CHOPCONF_CHM |
			 TMC5262_CHOPCONF_MRES | TMC5262_CHOPCONF_INTPOL | TMC5262_CHOPCONF_DEDGE |
			 TMC5262_CHOPCONF_HSTRT_TFD210 | TMC5262_CHOPCONF_HEND_OFFSET |
			 TMC5262_CHOPCONF_FD3 | TMC5262_CHOPCONF_DISFDCC | TMC5262_CHOPCONF_TPFD,
	.status = {
			.stst = TMC5262_DRV_STATUS_STST,
			.ol_a = TMC5262_DRV_STATUS_OLA,
			.ol_b = TMC5262_DRV_STATUS_OLB,
			.s2g_a = TMC5262_DRV_STATUS_S2GA,
			.s2g_b = TMC5262_DRV_STATUS_S2GB,
			.s2vs_a = TMC5262_DRV_STATUS_S2VSA,
			.s2vs_b = TMC5262_DRV_STATUS_S2VSB,
			.otpw = TMC5262_DRV_STATUS_OTPW,
			.ot = TMC5262_DRV_STATUS_OT,
			.stallguard = TMC5262_DRV_STATUS_STALLGUARD,
			.stealth = TMC5262_DRV_STATUS_STEALTH,
			.ov = TMC5262_DRV_STATUS_OV,
			.seq_stopped = TMC5262_DRV_STATUS_SEQ_STOPPED,
		},
	/* 1.5, 3, 4.5 and 6 A peak = 1.06, 2.12, 3.18 and 4.24 A RMS */
	.current_range_ma = {1060, 2120, 3180, 4240},
	.current_range_is_peak = false,
	.regs = tmc5262_regs,
	.n_regs = ARRAY_SIZE(tmc5262_regs),
};
#endif /* CONFIG_STEPPER_ADI_TMC5262_TMC5262 */

#ifdef CONFIG_STEPPER_ADI_TMC5262_TMC2262
static const struct tmc5262_reg_range tmc2262_regs[] = {
	TMC5262_COMMON_REGS,
	TMC5262_PLUS_REGS,
};

const struct tmc5262_variant tmc5262_variant_tmc2262 = {
	.name = "TMC2262",
	.id = TMC5262_VARIANT_TMC2262,
	/* No ramp generator, no reference switches, no position compare, no direct mode.
	 * The TMC2262 datasheet lists TriCoder (back-EMF standstill detection) as available.
	 */
	.caps = TMC5262_CAP_STEALTHCHOP | TMC5262_CAP_STALLGUARD_PLUS | TMC5262_CAP_COOLSTEP_PLUS |
		TMC5262_CAP_UDCSTEP | TMC5262_CAP_COIL_MEAS | TMC5262_CAP_ADC_I |
		TMC5262_CAP_ENC_DEVIATION | TMC5262_CAP_TRICODER | TMC5262_CAP_PER_COIL_FAULT,
	.twake_ms = 2,
	.n_phases = 2,
	.step_src_mask = 0,
	.step_src_ramp_when_set = false,
	.gconf_stealth_mask = TMC5262_GCONF_EN_STEALTHCHOP,
	.encmode_mask = TMC5262_ENCMODE_COMMON_MASK | TMC5262_ENCMODE_BEMF_MASK,
	.bemf_mask = TMC5262_ENCMODE_BEMF_MASK,
	.bemf_encoder_sel = TMC5262_ENCMODE_NBEMF_ABN_SEL,
	.chopconf_mask = TMC5262_CHOPCONF_TOFF | TMC5262_CHOPCONF_TBL | TMC5262_CHOPCONF_CHM |
			 TMC5262_CHOPCONF_MRES | TMC5262_CHOPCONF_INTPOL | TMC5262_CHOPCONF_DEDGE |
			 TMC5262_CHOPCONF_HSTRT_TFD210 | TMC5262_CHOPCONF_HEND_OFFSET |
			 TMC5262_CHOPCONF_FD3 | TMC5262_CHOPCONF_TPFD,
	.status = {
			.stst = TMC5262_DRV_STATUS_STST,
			.ol_a = TMC5262_DRV_STATUS_OLA,
			.ol_b = TMC5262_DRV_STATUS_OLB,
			.s2g_a = TMC5262_DRV_STATUS_S2GA,
			.s2g_b = TMC5262_DRV_STATUS_S2GB,
			.s2vs_a = TMC5262_DRV_STATUS_S2VSA,
			.s2vs_b = TMC5262_DRV_STATUS_S2VSB,
			.otpw = TMC5262_DRV_STATUS_OTPW,
			.ot = TMC5262_DRV_STATUS_OT,
			.stallguard = TMC5262_DRV_STATUS_STALLGUARD,
			.stealth = TMC5262_DRV_STATUS_STEALTH,
			.ov = TMC5262_DRV_STATUS_OV,
			.seq_stopped = TMC5262_DRV_STATUS_SEQ_STOPPED,
		},
	.current_range_ma = {1500, 3000, 4500, 6000},
	.current_range_is_peak = true,
	.regs = tmc2262_regs,
	.n_regs = ARRAY_SIZE(tmc2262_regs),
};
#endif /* CONFIG_STEPPER_ADI_TMC5262_TMC2262 */

#ifdef CONFIG_STEPPER_ADI_TMC5262_TMC3241
static const struct tmc5262_reg_range tmc3241_regs[] = {
	TMC5262_COMMON_REGS,
	TMC5262_RAMP_REGS,
};

const struct tmc5262_variant tmc5262_variant_tmc3241 = {
	.name = "TMC3241",
	.id = TMC5262_VARIANT_TMC3241,
	.caps = TMC5262_CAP_RAMP_GEN | TMC5262_CAP_REF_SWITCH | TMC5262_CAP_POS_COMPARE |
		TMC5262_CAP_DIRECT_MODE | TMC5262_CAP_XLATCH | TMC5262_CAP_VACTUAL |
		TMC5262_CAP_STEP_SOURCE_SEL,
	.twake_ms = 2,
	.n_phases = 3,
	.step_src_mask = TMC3241_GCONF_THREEPHASE_MCC,
	.step_src_ramp_when_set = true,
	.gconf_stealth_mask = 0,
	.encmode_mask = TMC5262_ENCMODE_COMMON_MASK,
	.chopconf_mask = TMC5262_CHOPCONF_TOFF | TMC5262_CHOPCONF_TBL | TMC5262_CHOPCONF_CHM |
			 TMC5262_CHOPCONF_MRES | TMC5262_CHOPCONF_INTPOL | TMC5262_CHOPCONF_DEDGE |
			 TMC3241_CHOPCONF_HYST | TMC3241_CHOPCONF_NOSD | TMC3241_CHOPCONF_SG_SPEED |
			 TMC3241_CHOPCONF_CSYNC,
	.status = {
			.stst = TMC3241_DRV_STATUS_STST,
			.ol_a = TMC3241_DRV_STATUS_OL,
			.s2g_a = TMC3241_DRV_STATUS_S2G,
			.s2vs_a = TMC3241_DRV_STATUS_S2VS,
			.otpw = TMC3241_DRV_STATUS_OTPW,
			.ot = TMC3241_DRV_STATUS_OT,
			.stallguard = TMC3241_DRV_STATUS_STALLGUARD,
			.ov = TMC3241_DRV_STATUS_OV,
			.seq_stopped = TMC3241_DRV_STATUS_SEQ_STOPPED,
		},
	/* 1.5, 3, 4.5 and 6 A peak = 1.06, 2.12, 3.18 and 4.24 A RMS */
	.current_range_ma = {1060, 2120, 3180, 4240},
	.current_range_is_peak = false,
	.regs = tmc3241_regs,
	.n_regs = ARRAY_SIZE(tmc3241_regs),
};
#endif /* CONFIG_STEPPER_ADI_TMC5262_TMC3241 */
/* clang-format on */

bool tmc5262_variant_has_reg(const struct tmc5262_variant *variant, uint8_t reg)
{
	for (uint8_t i = 0; i < variant->n_regs; i++) {
		if (reg >= variant->regs[i].first && reg <= variant->regs[i].last) {
			return true;
		}
	}

	return false;
}
