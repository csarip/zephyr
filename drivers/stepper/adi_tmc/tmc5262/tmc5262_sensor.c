/*
 * SPDX-FileCopyrightText: Copyright (c) 2026 Analog Devices, Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Zephyr sensor API on the on-chip ADC of the TMC5262, TMC2262 and TMC3241 (FR-DIAG-005):
 * supply voltage (SENSOR_CHAN_VOLTAGE) and chip temperature (SENSOR_CHAN_DIE_TEMP).
 */

#define DT_DRV_COMPAT adi_tmc5262_sensor

#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>

#include "tmc5262.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(tmc5262, CONFIG_STEPPER_ADI_TMC5262_LOG_LEVEL);

struct tmc5262_sensor_config {
	const struct device *controller;
};

struct tmc5262_sensor_data {
	int32_t supply_mv;
	int32_t temp_mdegc;
};

static int tmc5262_sensor_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	const struct tmc5262_sensor_config *config = dev->config;
	struct tmc5262_sensor_data *data = dev->data;
	int32_t mv;
	int32_t mdegc;
	int err;

	if (chan != SENSOR_CHAN_ALL && chan != SENSOR_CHAN_VOLTAGE &&
	    chan != SENSOR_CHAN_DIE_TEMP) {
		return -ENOTSUP;
	}

	/* One read of ADC_VSUPPLY_TEMP gives both values: one consistent sample */
	err = tmc5262_read_adc(config->controller, &mv, &mdegc);

	tmc5262_lock(config->controller);
	if (err == 0) {
		data->supply_mv = mv;
		data->temp_mdegc = mdegc;
	}
	tmc5262_unlock(config->controller);

	return err;
}

static int tmc5262_sensor_channel_get(const struct device *dev, enum sensor_channel chan,
				      struct sensor_value *val)
{
	const struct tmc5262_sensor_config *config = dev->config;
	struct tmc5262_sensor_data *data = dev->data;
	int32_t mv;
	int32_t mdegc;

	/* Copy the sample under the chip lock: a concurrent fetch must not tear it */
	tmc5262_lock(config->controller);
	mv = data->supply_mv;
	mdegc = data->temp_mdegc;
	tmc5262_unlock(config->controller);

	switch (chan) {
	case SENSOR_CHAN_VOLTAGE:
		return sensor_value_from_milli(val, mv);
	case SENSOR_CHAN_DIE_TEMP:
		return sensor_value_from_milli(val, mdegc);
	default:
		return -ENOTSUP;
	}
}

static int tmc5262_sensor_init(const struct device *dev)
{
	const struct tmc5262_sensor_config *config = dev->config;

	if (!device_is_ready(config->controller)) {
		LOG_ERR_DEVICE_NOT_READY(config->controller);
		return -ENODEV;
	}

	return 0;
}

static DEVICE_API(sensor, tmc5262_sensor_api) = {
	.sample_fetch = tmc5262_sensor_sample_fetch,
	.channel_get = tmc5262_sensor_channel_get,
};

#define TMC5262_SENSOR_DEFINE(inst)                                                                \
	static const struct tmc5262_sensor_config tmc5262_sensor_config_##inst = {                 \
		.controller = DEVICE_DT_GET(DT_INST_PARENT(inst)),                                 \
	};                                                                                         \
	static struct tmc5262_sensor_data tmc5262_sensor_data_##inst;                              \
	SENSOR_DEVICE_DT_INST_DEFINE(inst, tmc5262_sensor_init, NULL, &tmc5262_sensor_data_##inst, \
				     &tmc5262_sensor_config_##inst, POST_KERNEL,                   \
				     CONFIG_SENSOR_INIT_PRIORITY, &tmc5262_sensor_api);

DT_INST_FOREACH_STATUS_OKAY(TMC5262_SENSOR_DEFINE)
