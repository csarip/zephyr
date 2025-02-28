/*
 * Copyright (c) 2025 Analog Devices Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT adi_max2221x

#include <zephyr/drivers/mfd/max2221x.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(MFD_MAX2221X, CONFIG_MFD_LOG_LEVEL);

struct max2221x_config {
    struct spi_dt_spec spi;
};

struct max2221x_data {
    // Add any necessary data fields here
};

int max2221x_reg_read(const struct device *dev, uint8_t addr, uint16_t *value)
{
    int ret;
    const struct max2221x_config *config = dev->config;
    uint8_t rxbuffer[3] = {0};
    size_t rx_len = 3;

    *value = 0;
    addr = FIELD_PREP(MAX2221X_SPI_TRANS_ADDR, addr) | FIELD_PREP(MAX2221X_SPI_TRANS_DIR, 0);

    const struct spi_buf txb[] = {
        {
            .buf = &addr,
            .len = 1,
        },
    };

    const struct spi_buf rxb[] = {
        {
            .buf = rxbuffer,
            .len = rx_len,
        },
    };

    struct spi_buf_set tx = {
        .buffers = txb,
        .count = ARRAY_SIZE(txb),
    };

    struct spi_buf_set rx = {
        .buffers = rxb,
        .count = ARRAY_SIZE(rxb),
    };

    ret = spi_transceive_dt(&config->spi, &tx, &rx);

    if (ret) {
        return ret;
    }

    *value = rxbuffer[1] | (rxbuffer[2] << 8);
    *value = sys_be16_to_cpu(*value);

    return 0;
}

int max2221x_reg_write(const struct device *dev, uint8_t addr, uint16_t value)
{
    const struct max2221x_config *config = dev->config;

    addr = FIELD_PREP(MAX2221X_SPI_TRANS_ADDR, addr) | FIELD_PREP(MAX2221X_SPI_TRANS_DIR, 1);
    value = sys_cpu_to_be16(value);

    const struct spi_buf buf[] = {
        {
            .buf = &addr,
            .len = 1,
        },
        {
            .buf = &value,
            .len = 2,
        },
    };

    struct spi_buf_set tx = {
        .buffers = buf,
        .count = ARRAY_SIZE(buf),
    };

    return spi_write_dt(&config->spi, &tx);
}

int max2221x_reg_update(const struct device *dev, uint8_t addr, uint16_t mask, uint16_t val)
{
    uint16_t tmp;
    int ret;

    ret = max2221x_reg_read(dev, addr, &tmp);
    if (ret)
        return ret;

    tmp = (tmp & ~mask) | FIELD_PREP(mask, val);

    return max2221x_reg_write(dev, addr, tmp);
}

static int max2221x_init(const struct device *dev)
{
    const struct max2221x_config *config = dev->config;

    if (!spi_is_ready_dt(&config->spi)) {
        LOG_ERR("SPI device %s not ready", config->spi.bus->name);
        return -ENODEV;
    }

    return 0;
}

#define MAX2221X_DEFINE(inst)									\
    static struct max2221x_data max2221x_data_##inst;						\
    static const struct max2221x_config max2221x_config_##inst = {				\
        .spi = SPI_DT_SPEC_INST_GET(inst, SPI_WORD_SET(8) | SPI_TRANSFER_MSB, 0),	\
    };											\
                                                \
    DEVICE_DT_INST_DEFINE(inst, max2221x_init, NULL, &max2221x_data_##inst, &max2221x_config_##inst,	   \
                  POST_KERNEL, CONFIG_MFD_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(MAX2221X_DEFINE)