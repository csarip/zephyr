/**
 * @file drivers/stepper/stepper_trinamic.h
 *
 * @brief Public API for Trinamic Stepper Controller Specific Functions
 *
 */

/*
 * SPDX-FileCopyrightText: Copyright (c) 2024 Carl Zeiss Meditec AG
 * SPDX-FileCopyrightText: Copyright (c) 2025 Prevas A/S
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_STEPPER_STEPPER_TRINAMIC_H_
#define ZEPHYR_INCLUDE_DRIVERS_STEPPER_STEPPER_TRINAMIC_H_

/**
 * @brief Trinamic Stepper Controller
 * @defgroup trinamic_stepper_ctrl Trinamic Stepper Controller
 * @ingroup stepper_ctrl
 * @since 4.0
 * @version 0.9.0
 * @{
 */

#include <stdint.h>
#include <zephyr/drivers/stepper/stepper.h>
#include <zephyr/sys/util.h>

#ifdef __cplusplus
extern "C" {
#endif

#define TMC_RAMP_VACTUAL_SHIFT  23
#define TMC_RAMP_XACTUAL_SHIFT  31

/**
 * @brief Trinamic Stepper StallGuard Settings
 */
struct tmc_stallguard_settings {
	/** Enable StallGuard2 feature*/
	bool is_sg_enabled;
	/**
	 * Stallguard should not be enabled during motor spin-up.
	 * This delay is used to check if the actual stepper velocity is greater than
	 * stallguard-threshold-velocity before enabling stallguard.
	 */
	uint16_t sg_velocity_check_interval_ms;
	/** StallGuard2 threshold velocity */
	uint32_t sg_threshold_velocity;
};

/**
 * @brief Trinamic Stepper Ramp Generator data
 */
struct tmc_ramp_generator_data {
	uint32_t vstart;
	uint32_t v1;
	uint32_t vmax;
	uint16_t a1;
	uint16_t amax;
	uint16_t d1;
	uint16_t dmax;
	uint32_t vstop;
	uint16_t tzerowait;
	uint32_t iholdrun;
	union {
		/* TMC50XX specific */
		struct {
			uint32_t vcoolthrs;
			uint32_t vhigh;
		};
		/* TMC51XX specific */
		struct {
			uint32_t tpowerdown;
			uint32_t tpwmthrs;
			uint32_t tcoolthrs;
			uint32_t thigh;
		};
	};
};

/**
 * @brief Trinamic Stepper Ramp Generator data for 8-point ramps
 *
 * @details Extends the classic 6-point ramp with the second acceleration,
 * deceleration and velocity thresholds (a2/d2/v2) supported by the TMC5262.
 */
struct tmc5262_ramp_generator_data {
	uint32_t vstart;
	uint32_t v1;
	uint32_t v2;
	uint32_t vmax;
	uint16_t a1;
	uint16_t a2;
	uint16_t amax;
	uint16_t d1;
	uint16_t d2;
	uint16_t dmax;
	uint32_t vstop;
	uint16_t tzerowait;
	uint32_t iholdrun;
	uint32_t tpowerdown;
	uint32_t tpwmthrs;
	uint32_t tcoolthrs;
	uint32_t thigh;
};

/**
 * @brief Get Trinamic Stepper Ramp Generator data from DT
 *
 * @param node DT node identifier
 *
 * @return struct tmc_ramp_generator_data
 */
#define TMC_RAMP_DT_SPEC_GET_COMMON(node)					\
		.vstart = DT_PROP(node, vstart),				\
		.v1 = DT_PROP(node, v1),					\
		.vmax = DT_PROP(node, vmax),					\
		.a1 = DT_PROP(node, a1),					\
		.amax = DT_PROP(node, amax),					\
		.d1 = DT_PROP(node, d1),					\
		.dmax = DT_PROP(node, dmax),					\
		.vstop = DT_PROP(node, vstop),					\
		.tzerowait = DT_PROP(node, tzerowait),				\
		.iholdrun = (TMC5XXX_IRUN(DT_PROP(node, irun)) |		\
			     TMC5XXX_IHOLD(DT_PROP(node, ihold)) |		\
			     TMC5XXX_IHOLDDELAY(DT_PROP(node, iholddelay))),

#define TMC_RAMP_DT_SPEC_GET_TMC50XX(node)					\
	{									\
		TMC_RAMP_DT_SPEC_GET_COMMON(node)				\
		.vhigh = DT_PROP(node, vhigh),					\
		.vcoolthrs = DT_PROP(node, vcoolthrs),				\
	}

#define TMC_RAMP_DT_SPEC_GET_TMC51XX(node)					\
	{									\
		TMC_RAMP_DT_SPEC_GET_COMMON(DT_DRV_INST(node))			\
		.tpowerdown = DT_INST_PROP(node, tpowerdown),			\
		.tpwmthrs = DT_INST_PROP(node, tpwmthrs),			\
		.tcoolthrs = DT_INST_PROP(node, tcoolthrs),			\
		.thigh = DT_INST_PROP(node, thigh),				\
	}

#define TMC_RAMP_DT_SPEC_GET_TMC5262(node)					\
	{									\
		.vstart = DT_INST_PROP(node, vstart),				\
		.v1 = DT_INST_PROP(node, v1),					\
		.v2 = DT_INST_PROP(node, v2),					\
		.vmax = DT_INST_PROP(node, vmax),				\
		.a1 = DT_INST_PROP(node, a1),					\
		.a2 = DT_INST_PROP(node, a2),					\
		.amax = DT_INST_PROP(node, amax),				\
		.d1 = DT_INST_PROP(node, d1),					\
		.d2 = DT_INST_PROP(node, d2),					\
		.dmax = DT_INST_PROP(node, dmax),				\
		.vstop = DT_INST_PROP(node, vstop),				\
		.tzerowait = DT_INST_PROP(node, tzerowait),			\
		.iholdrun = (TMC5XXX_IRUN(DT_INST_PROP(node, irun)) |		\
			     TMC5XXX_IHOLD(DT_INST_PROP(node, ihold)) |		\
			     TMC5XXX_IHOLDDELAY(DT_INST_PROP(node, iholddelay))),\
		.tpowerdown = DT_INST_PROP(node, tpowerdown),			\
		.tpwmthrs = DT_INST_PROP(node, tpwmthrs),			\
		.tcoolthrs = DT_INST_PROP(node, tcoolthrs),			\
		.thigh = DT_INST_PROP(node, thigh),				\
	}

/**
 * @brief Compile-time validation of the extended ramp devicetree data
 *
 * @param inst DT instance number
 */
#define CHECK_RAMP_DT_DATA(inst)						\
	BUILD_ASSERT(DT_INST_PROP(inst, vmax) <= TMC5XXX_RAMPGEN_VMAX_MAX_VALUE,	\
		     "vmax exceeds the maximum allowed value");

/**
 * @brief Convert a velocity/acceleration in microsteps per second to the
 * TMC5262 internal representation.
 *
 * @param ustep_hz Velocity or acceleration in microsteps per second
 * @param clock_frequency Clock frequency in Hz
 *
 * @return Value in the TMC5262 internal (full clock cycle) representation
 */
static inline uint32_t tmc5262_convert_ustep_to_internal(uint64_t ustep_hz,
							 uint32_t clock_frequency)
{
	/* Internal value = ustep_hz * 2^24 / fCLK */
	const uint8_t clock_freq_shift = 24U;

	__ASSERT_NO_MSG(clock_frequency);
	return (uint32_t)((ustep_hz << clock_freq_shift) / clock_frequency);
}

/**
 * @brief Configure Trinamic Stepper Ramp Generator
 *
 * @param dev Pointer to the stepper motor controller instance
 * @param ramp_data Pointer to a struct containing the required ramp parameters
 *
 * @retval -EIO General input / output error
 * @retval -ENOSYS If not implemented by device driver
 * @retval 0 Success
 */
int tmc50xx_stepper_ctrl_set_ramp(const struct device *dev,
			     const struct tmc_ramp_generator_data *ramp_data);

/**
 * @brief Set the maximum velocity of the stepper motor
 *
 * @param dev Pointer to the stepper driver instance
 * @param velocity Maximum velocity in microsteps per second.
 *
 * @retval -EIO General input / output error
 * @retval 0 Success
 */
int tmc50xx_stepper_ctrl_set_max_velocity(const struct device *dev, uint32_t velocity);

/**
 * @brief Configure TMC50XX Stepper StallGuard settings
 *
 * @param dev Pointer to the stepper motor controller instance
 * @param sg_settings Pointer to a struct containing the required StallGuard parameters
 *
 */
void tmc50xx_stepper_ctrl_configure_stallguard(const struct device *dev,
					       const struct tmc_stallguard_settings *sg_settings);

/**
 * @brief Configure TMC51XX Stepper StallGuard settings
 *
 * @param dev Pointer to the stepper motor controller instance
 * @param sg_settings Pointer to a struct containing the required StallGuard parameters
 */
void tmc51xx_stepper_ctrl_configure_stallguard(const struct device *dev,
					       const struct tmc_stallguard_settings *sg_settings);

/**
 * @brief Set the maximum velocity of the stepper motor
 *
 * @param dev Pointer to the stepper driver instance
 * @param velocity Maximum velocity in microsteps per second.
 *
 * @retval -EIO General input / output error
 * @retval 0 Success
 */
int tmc51xx_stepper_ctrl_set_max_velocity(const struct device *dev, uint32_t velocity);
/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_STEPPER_STEPPER_TRINAMIC_H_ */
