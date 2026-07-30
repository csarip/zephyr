/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-FileCopyrightText: Copyright (c) 2026 Analog Devices Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT adi_tmc5262

#include <stdlib.h>

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/stepper/stepper.h>
#include <zephyr/drivers/stepper/stepper_ctrl.h>
#include <zephyr/drivers/stepper/stepper_trinamic.h>

#include <adi_tmc_bus.h>
#include <adi_tmc_spi.h>
#include <adi_tmc5xxx_common.h>

#include "tmc5262.h"
#include "tmc5262_reg.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(tmc5262, CONFIG_STEPPER_LOG_LEVEL);

/* Check for supported bus types */
#define TMC5262_BUS_SPI DT_ANY_INST_ON_BUS_STATUS_OKAY(spi)

/* Maximum number of PLL lock attempts before giving up during initialization. */
#define TMC5262_PLL_LOCK_RETRIES 100U

struct tmc5262_config {
	union tmc_bus bus;
	const struct tmc_bus_io *bus_io;
	uint8_t comm_type;
	const uint32_t gconf;
	const uint32_t clock_frequency;
	const uint32_t motor_inductance;
	const uint32_t t_rcoil_meas;
	const uint8_t cool_cur_div;
#if TMC5262_BUS_SPI
	struct gpio_dt_spec sleepn_gpio;
	struct gpio_dt_spec drvenn_gpio;
	struct gpio_dt_spec diag0_gpio;
#endif
	const struct device *motion_controller;
	const struct device *stepper_driver;
};

struct tmc5262_data {
	struct k_sem sem;
	struct k_work_delayable rampstat_callback_dwork;
	struct gpio_callback diag0_cb;
	const struct device *dev;
};

#if TMC5262_BUS_SPI

static int tmc5262_bus_check_spi(const union tmc_bus *bus, uint8_t comm_type)
{
	if (comm_type != TMC_COMM_SPI) {
		return -ENOTSUP;
	}

	if (!spi_is_ready_dt(&bus->spi)) {
		return -ENODEV;
	}

	return 0;
}

static int tmc5262_reg_write_spi(const struct device *dev, const uint8_t reg_addr,
				 const uint32_t reg_val)
{
	const struct tmc5262_config *config = dev->config;
	int err;

	err = tmc_spi_write_register(&config->bus.spi, TMC5XXX_WRITE_BIT, reg_addr, reg_val);
	if (err < 0) {
		LOG_ERR("Failed to write register 0x%x with value 0x%x", reg_addr, reg_val);
	}

	return err;
}

static int tmc5262_reg_read_spi(const struct device *dev, const uint8_t reg_addr, uint32_t *reg_val)
{
	const struct tmc5262_config *config = dev->config;
	int err;

	err = tmc_spi_read_register(&config->bus.spi, TMC5XXX_ADDRESS_MASK, reg_addr, reg_val);
	if (err < 0) {
		LOG_ERR("Failed to read register 0x%x", reg_addr);
	}

	return err;
}

static const struct tmc_bus_io tmc5262_spi_bus_io = {
	.check = tmc5262_bus_check_spi,
	.read = tmc5262_reg_read_spi,
	.write = tmc5262_reg_write_spi,
};
#endif /* TMC5262_BUS_SPI */

static inline int tmc5262_bus_check(const struct device *dev)
{
	const struct tmc5262_config *config = dev->config;

	return config->bus_io->check(&config->bus, config->comm_type);
}

int tmc5262_get_clock_frequency(const struct device *dev)
{
	const struct tmc5262_config *config = dev->config;

	return config->clock_frequency;
}

int tmc5262_write(const struct device *dev, const uint8_t reg_addr, const uint32_t reg_val)
{
	const struct tmc5262_config *config = dev->config;
	struct tmc5262_data *data = dev->data;
	int err;

	k_sem_take(&data->sem, K_FOREVER);

	err = config->bus_io->write(dev, reg_addr, reg_val);

	k_sem_give(&data->sem);

	if (err < 0) {
		LOG_ERR("Failed to write register 0x%x with value 0x%x", reg_addr, reg_val);
		return err;
	}
	return 0;
}

int tmc5262_read(const struct device *dev, const uint8_t reg_addr, uint32_t *reg_val)
{
	const struct tmc5262_config *config = dev->config;
	struct tmc5262_data *data = dev->data;
	int err;

	k_sem_take(&data->sem, K_FOREVER);

	err = config->bus_io->read(dev, reg_addr, reg_val);

	k_sem_give(&data->sem);

	if (err < 0) {
		LOG_ERR("Failed to read register 0x%x", reg_addr);
		return err;
	}
	return 0;
}

int tmc5262_update(const struct device *dev, const uint8_t reg_addr, uint32_t mask, uint32_t value)
{
	uint32_t reg_val;
	int err;

	err = tmc5262_read(dev, reg_addr, &reg_val);
	if (err < 0) {
		return -EIO;
	}

	reg_val &= ~mask;
	reg_val |= FIELD_PREP(mask, value);

	err = tmc5262_write(dev, reg_addr, reg_val);
	if (err < 0) {
		return -EIO;
	}

	return 0;
}

int tmc5262_read_actual_position(const struct device *dev, int32_t *position)
{
	uint32_t raw_value;
	int err;

	err = tmc5262_read(dev, TMC5262_XACTUAL, &raw_value);
	if (err != 0) {
		return -EIO;
	}

	*position = sign_extend(raw_value, TMC_RAMP_XACTUAL_SHIFT);
	return 0;
}

bool tmc5262_is_interrupt_driven(const struct device *dev)
{
	__maybe_unused const struct tmc5262_config *config = dev->config;

	IF_ENABLED(TMC5262_BUS_SPI, ({
if (config->comm_type == TMC_COMM_SPI && config->diag0_gpio.port) {
	return true;
}
}))
	return false;
}

void tmc5262_reschedule_rampstat_callback(const struct device *dev)
{
	struct tmc5262_data *data = dev->data;

	k_work_reschedule(&data->rampstat_callback_dwork,
			  K_MSEC(CONFIG_STEPPER_ADI_TMC5262_RAMPSTAT_POLL_INTERVAL_IN_MSEC));
}

#ifdef CONFIG_STEPPER_ADI_TMC5262_RAMPSTAT_POLL_STALLGUARD_LOG

static void log_stallguard(const struct device *dev, const uint32_t drv_status)
{
	int32_t position;
	int err;

	err = tmc5262_read_actual_position(dev, &position);
	if (err != 0) {
		LOG_ERR("%s: Failed to read XACTUAL register", dev->name);
		return;
	}

	const uint8_t sg_result = FIELD_GET(TMC5XXX_DRV_STATUS_SG_RESULT_MASK, drv_status);
	const bool sg_status = FIELD_GET(TMC5XXX_DRV_STATUS_SG_STATUS_MASK, drv_status);

	LOG_DBG("%s position: %d | sg result: %3d status: %d", dev->name, position, sg_result,
		sg_status);
}

#endif /* CONFIG_STEPPER_ADI_TMC5262_RAMPSTAT_POLL_STALLGUARD_LOG */

static int rampstat_read_clear(const struct device *dev, uint32_t *rampstat_value)
{
	int err;

	err = tmc5262_read(dev, TMC5262_RAMPSTAT, rampstat_value);
	if (err == 0) {
		err = tmc5262_write(dev, TMC5262_RAMPSTAT, *rampstat_value);
	}
	return err;
}

static void rampstat_work_handler(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct tmc5262_data *stepper_data =
		CONTAINER_OF(dwork, struct tmc5262_data, rampstat_callback_dwork);
	const struct device *dev = stepper_data->dev;
	__maybe_unused const struct tmc5262_config *config = dev->config;
	__maybe_unused const struct device *motion_controller = config->motion_controller;
	__maybe_unused const struct device *stepper_driver = config->stepper_driver;
	uint32_t drv_status;
	int err;

	__ASSERT_NO_MSG(dev);

	err = tmc5262_read(dev, TMC5262_DRVSTATUS, &drv_status);
	if (err != 0) {
		LOG_ERR("%s: Failed to read DRVSTATUS register", dev->name);
		return;
	}
#ifdef CONFIG_STEPPER_ADI_TMC5262_RAMPSTAT_POLL_STALLGUARD_LOG
	log_stallguard(dev, drv_status);
#endif
	if (FIELD_GET(TMC5XXX_DRV_STATUS_SG_STATUS_MASK, drv_status) == 1U) {
		LOG_INF("%s: Stall detected", dev->name);
		err = tmc5262_write(dev, TMC5262_RAMPMODE, TMC5XXX_RAMPMODE_HOLD_MODE);
		if (err != 0) {
			LOG_ERR("%s: Failed to stop motor", dev->name);
			return;
		}
	}

	uint32_t rampstat_value;

	err = rampstat_read_clear(dev, &rampstat_value);
	if (err != 0) {
		LOG_ERR("%s: Failed to read RAMPSTAT register", dev->name);
		return;
	}

	const uint8_t ramp_stat_values = FIELD_GET(TMC5XXX_RAMPSTAT_INT_MASK, rampstat_value);

	if (ramp_stat_values > 0) {
		switch (ramp_stat_values) {
#ifdef CONFIG_STEPPER_ADI_TMC5262_STEPPER_CTRL
		case TMC5XXX_STOP_LEFT_EVENT:
			LOG_DBG("RAMPSTAT %s:Left end-stop detected", dev->name);
			tmc5262_stepper_ctrl_trigger_cb(motion_controller,
							STEPPER_CTRL_EVENT_LEFT_END_STOP_DETECTED);
			break;

		case TMC5XXX_STOP_RIGHT_EVENT:
			LOG_DBG("RAMPSTAT %s:Right end-stop detected", dev->name);
			tmc5262_stepper_ctrl_trigger_cb(motion_controller,
							STEPPER_CTRL_EVENT_RIGHT_END_STOP_DETECTED);
			break;

		case TMC5XXX_POS_REACHED_EVENT:
		case TMC5XXX_POS_REACHED:
		case TMC5XXX_POS_REACHED_AND_EVENT:
			LOG_DBG("RAMPSTAT %s:Position reached", dev->name);
			tmc5262_stepper_ctrl_trigger_cb(motion_controller,
							STEPPER_CTRL_EVENT_STEPS_COMPLETED);
			break;
#endif /* CONFIG_STEPPER_ADI_TMC5262_STEPPER_CTRL */
#ifdef CONFIG_STEPPER_ADI_TMC5262_STEPPER_DRIVER
		case TMC5XXX_STOP_SG_EVENT:
			LOG_DBG("RAMPSTAT %s:Stall detected", dev->name);
			tmc5262_stepper_ctrl_stallguard_enable(motion_controller, false);
			tmc5262_stepper_driver_trigger_cb(stepper_driver,
							  STEPPER_EVENT_STALL_DETECTED);
			break;
#endif /* CONFIG_STEPPER_ADI_TMC5262_STEPPER_DRIVER */
		default:
			LOG_ERR("Illegal ramp stat bit field 0x%x", ramp_stat_values);
			break;
		}
	} else {
		/* For SPI with DIAG0 pin, we use an interrupt-driven approach */
		IF_ENABLED(TMC5262_BUS_SPI, ({
			if (config->comm_type == TMC_COMM_SPI && config->diag0_gpio.port) {
				/* Interrupt-driven approach - no polling needed */
				return;
			}
			}))

		/* For SPI without DIAG0, reschedule RAMPSTAT polling */
		k_work_reschedule(
			&stepper_data->rampstat_callback_dwork,
			K_MSEC(CONFIG_STEPPER_ADI_TMC5262_RAMPSTAT_POLL_INTERVAL_IN_MSEC));
	}
}

static void __maybe_unused tmc5262_diag0_gpio_callback_handler(const struct device *port,
							       struct gpio_callback *cb,
							       gpio_port_pins_t pins)
{
	ARG_UNUSED(port);
	ARG_UNUSED(pins);

	struct tmc5262_data *stepper_data = CONTAINER_OF(cb, struct tmc5262_data, diag0_cb);

	k_work_reschedule(&stepper_data->rampstat_callback_dwork, K_NO_WAIT);
}

static int tmc5262_pll_config(const struct device *dev)
{
	const struct tmc5262_config *config = dev->config;
	uint32_t divider;
	uint32_t reg_val;
	int err;

	divider = config->clock_frequency / 1000000U - 1U;

	/* Enable PLL fsm, enable ADC clock, enable PWM clock */
	reg_val = 0x041CU;

	/* Use the internal clock source */
	reg_val &= ~TMC5262_CLOCK_SOURCE_MASK;

	reg_val &= ~TMC5262_CLOCK_DIVIDER_MASK;
	reg_val |= FIELD_PREP(TMC5262_CLOCK_DIVIDER_MASK, divider);

	reg_val &= ~TMC5262_CLOCK_FLAGS_MASK;
	reg_val |= FIELD_PREP(TMC5262_CLOCK_FLAGS_MASK, 0xFU);

	LOG_DBG("PLL register initial value for '%s': 0x%08X", dev->name, reg_val);

	for (uint32_t attempt = 0; attempt < TMC5262_PLL_LOCK_RETRIES; attempt++) {
		err = tmc5262_write(dev, TMC5262_PLL, reg_val | 0x1U);
		if (err < 0) {
			return -EIO;
		}

		err = tmc5262_write(dev, TMC5262_PLL, reg_val);
		if (err < 0) {
			return -EIO;
		}

		err = tmc5262_read(dev, TMC5262_PLL, &reg_val);
		if (err < 0) {
			return -EIO;
		}

		if (!(reg_val & TMC5262_CLOCK_FLAGS_MASK)) {
			return 0;
		}
	}

	LOG_ERR("PLL failed to lock for '%s'", dev->name);
	return -ETIMEDOUT;
}

static int tmc5262_coil_config(const struct device *dev)
{
	const struct tmc5262_config *config = dev->config;
	int err;

	err = tmc5262_update(dev, TMC5262_COIL_INDUCT, TMC5262_COIL_INDUCT_MASK,
			     config->motor_inductance);
	if (err < 0) {
		LOG_ERR("Failed to set COIL_INDUCT register");
		return -EIO;
	}

	err = tmc5262_update(dev, TMC5262_T_RCOIL_MEAS, TMC5262_T_RCOIL_MEAS_MASK,
			     config->t_rcoil_meas);
	if (err < 0) {
		LOG_ERR("Failed to set T_RCOIL_MEAS register");
		return -EIO;
	}

	err = tmc5262_update(dev, TMC5262_COOLSTEPPLUS_CONF, TMC5262_COOL_CUR_DIV_MASK,
			     config->cool_cur_div);
	if (err < 0) {
		LOG_ERR("Failed to set COOLSTEPPLUS_CONF register");
		return -EIO;
	}

	return 0;
}

static int tmc5262_init(const struct device *dev)
{
	const struct tmc5262_config *config = dev->config;
	struct tmc5262_data *data = dev->data;
	uint32_t reg_val;
	int err;

	LOG_DBG("Initializing TMC5262 controller %s", dev->name);

	k_sem_init(&data->sem, 1, 1);

#if TMC5262_BUS_SPI
	if (!gpio_is_ready_dt(&config->sleepn_gpio)) {
		LOG_ERR("SLEEPN GPIO not ready");
		return -ENODEV;
	}

	err = gpio_pin_configure_dt(&config->sleepn_gpio, GPIO_OUTPUT_ACTIVE);
	if (err < 0) {
		LOG_ERR("Could not configure SLEEPN GPIO (%d)", err);
		return err;
	}

	if (!gpio_is_ready_dt(&config->drvenn_gpio)) {
		LOG_ERR("DRVENN GPIO not ready");
		return -ENODEV;
	}

	err = gpio_pin_configure_dt(&config->drvenn_gpio, GPIO_OUTPUT_INACTIVE);
	if (err < 0) {
		LOG_ERR("Could not configure DRVENN GPIO (%d)", err);
		return err;
	}

	k_sleep(K_MSEC(1));

	err = gpio_pin_set_dt(&config->sleepn_gpio, 0);
	if (err < 0) {
		LOG_ERR("Could not wake device via SLEEPN GPIO (%d)", err);
		return err;
	}

	k_sleep(K_MSEC(100));
#endif /* TMC5262_BUS_SPI */

	err = tmc5262_bus_check(dev);
	if (err < 0) {
		LOG_ERR("Bus not ready for '%s'", dev->name);
		return err;
	}

	err = tmc5262_pll_config(dev);
	if (err < 0) {
		LOG_ERR("Failed to configure PLL for '%s'", dev->name);
		return err;
	}

	/* Read and write GSTAT register to clear any SPI Datagram errors. */
	err = tmc5262_read(dev, TMC5XXX_GSTAT, &reg_val);
	if (err != 0) {
		return -EIO;
	}

	err = tmc5262_write(dev, TMC5XXX_GSTAT, reg_val);
	if (err != 0) {
		return -EIO;
	}

	LOG_DBG("GCONF: %d", config->gconf);
	err = tmc5262_write(dev, TMC5XXX_GCONF, config->gconf);
	if (err != 0) {
		return -EIO;
	}

	err = tmc5262_coil_config(dev);
	if (err != 0) {
		return -EIO;
	}

	k_work_init_delayable(&data->rampstat_callback_dwork, rampstat_work_handler);

	/* Configure DIAG0 GPIO interrupt pin */
	IF_ENABLED(TMC5262_BUS_SPI, ({
	if ((config->comm_type == TMC_COMM_SPI) && config->diag0_gpio.port) {
		LOG_DBG("Configuring DIAG0 GPIO interrupt pin");
		if (!gpio_is_ready_dt(&config->diag0_gpio)) {
			LOG_ERR("DIAG0 interrupt GPIO not ready");
			return -ENODEV;
		}

		err = gpio_pin_configure_dt(&config->diag0_gpio, GPIO_INPUT);
		if (err < 0) {
			LOG_ERR("Could not configure DIAG0 GPIO (%d)", err);
			return err;
		}

		/* DIAG0 and DIAG1 active low, push-pull output */
		reg_val = 0xF0000000U;
		reg_val |= TMC5262_MAP_RAMPSTAT_POS_REACHED_DIAG0_MASK;
		reg_val |= TMC5262_MAP_RAMPSTAT_SG_STOP_DIAG0_MASK;
		reg_val |= TMC5262_MAP_RAMPSTAT_STOP_LR_DIAG0_MASK;

		err = tmc5262_write(dev, TMC5262_DIAG_CONF, reg_val);
		if (err < 0) {
			LOG_ERR("Failed to configure DIAG_CONF register");
			return -EIO;
		}

		err = gpio_pin_interrupt_configure_dt(&config->diag0_gpio, GPIO_INT_EDGE_RISING);
		if (err < 0) {
			LOG_ERR("Failed to configure DIAG0 interrupt (err %d)", err);
			return -EIO;
		}

		gpio_init_callback(&data->diag0_cb, tmc5262_diag0_gpio_callback_handler,
				   BIT(config->diag0_gpio.pin));

		err = gpio_add_callback(config->diag0_gpio.port, &data->diag0_cb);
		if (err < 0) {
			LOG_ERR("Could not add DIAG0 pin GPIO callback (%d)", err);
			return -EIO;
		}
	}
}))

	(void)rampstat_read_clear(dev, &reg_val);

#if TMC5262_BUS_SPI
	err = gpio_pin_set_dt(&config->drvenn_gpio, 1);
	if (err < 0) {
		LOG_ERR("Failed to enable driver via DRVENN GPIO for '%s'", dev->name);
		return -EIO;
	}
#endif /* TMC5262_BUS_SPI */

	return 0;
}

#define DT_CHILD_BY_COMPATIBLE(parent_node_id, compat)                                             \
	DT_FOREACH_CHILD_STATUS_OKAY_VARGS(parent_node_id, _DT_CHILD_BY_COMPAT_HELPER, compat)

#define _DT_CHILD_BY_COMPAT_HELPER(node_id, compat)                                                \
	COND_CODE_1(DT_NODE_HAS_COMPAT(node_id, compat), (node_id), ())

/* Initializes a struct tmc5262_config for an instance on a SPI bus. */
#define TMC5262_CONFIG_SPI(inst)                                                                   \
	.comm_type = TMC_COMM_SPI,                                                                 \
	.bus.spi = SPI_DT_SPEC_INST_GET(inst, (SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB |             \
					       SPI_MODE_CPOL | SPI_MODE_CPHA | SPI_WORD_SET(8))),  \
	.bus_io = &tmc5262_spi_bus_io,                                                             \
	.sleepn_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, sleepn_gpios, {0}),                          \
	.drvenn_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, drvenn_gpios, {0}),                          \
	.diag0_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, diag0_gpios, {0})

/* Device initialization macros */
#define TMC5262_DEFINE(inst)                                                                       \
	BUILD_ASSERT((DT_INST_PROP(inst, clock_frequency) > 0),                                    \
		     "clock frequency must be non-zero positive value");                           \
	static struct tmc5262_data tmc5262_data_##inst = {                                         \
		.dev = DEVICE_DT_GET(DT_DRV_INST(inst))};                                          \
	static const struct tmc5262_config tmc5262_config_##inst = {                               \
		TMC5262_CONFIG_SPI(inst),                                                          \
		.gconf = ((DT_INST_PROP(inst, step_dir) << TMC5262_GCONF_STEP_DIR_SHIFT) |         \
			  (DT_INST_PROP(inst, ov_nn) << TMC5262_GCONF_OV_NN_SHIFT) |               \
			  (DT_INST_PROP(inst, length_steppulse)                                    \
			   << TMC5262_GCONF_LENGTH_STEPPULSE_SHIFT) |                              \
			  (DT_INST_PROP(inst, direct_mode) << TMC5262_GCONF_DIRECT_MODE_SHIFT) |   \
			  (DT_INST_PROP(inst, stop_enable) << TMC5262_GCONF_STOP_ENABLE_SHIFT) |   \
			  (DT_INST_PROP(inst, small_hysteresis)                                    \
			   << TMC5262_GCONF_SMALL_HYSTERESIS_SHIFT) |                              \
			  (DT_INST_PROP(inst, shaft) << TMC5262_GCONF_SHAFT_SHIFT) |               \
			  (DT_INST_PROP(inst, multistep_filt)                                      \
			   << TMC5262_GCONF_MULTISTEP_FILT_SHIFT) |                                \
			  (DT_INST_PROP(inst, en_stealthchop)                                      \
			   << TMC5262_GCONF_EN_STEALTHCHOP_SHIFT) |                                \
			  (DT_INST_PROP(inst, fast_standstill)                                     \
			   << TMC5262_GCONF_FAST_STANDSTILL_SHIFT)),                               \
		.clock_frequency = DT_INST_PROP(inst, clock_frequency),                            \
		.motor_inductance = DT_INST_PROP(inst, motor_inductance),                          \
		.cool_cur_div = DT_INST_PROP(inst, cool_cur_div),                                  \
		.t_rcoil_meas = DT_INST_PROP(inst, t_rcoil_meas),                                  \
		.motion_controller = DEVICE_DT_GET_OR_NULL(                                        \
			DT_CHILD_BY_COMPATIBLE(DT_DRV_INST(inst), adi_tmc5262_stepper_ctrl)),      \
		.stepper_driver = DEVICE_DT_GET_OR_NULL(                                           \
			DT_CHILD_BY_COMPATIBLE(DT_DRV_INST(inst), adi_tmc5262_stepper_driver))};   \
	DEVICE_DT_INST_DEFINE(inst, tmc5262_init, NULL, &tmc5262_data_##inst,                      \
			      &tmc5262_config_##inst, POST_KERNEL, CONFIG_STEPPER_INIT_PRIORITY,   \
			      NULL);

DT_INST_FOREACH_STATUS_OKAY(TMC5262_DEFINE)
