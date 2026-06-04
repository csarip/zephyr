/*
 * SPDX-FileCopyrightText: Copyright (c) 2026 Cherrence Sarip <cherrence.sarip@analog.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT adi_tmc6460

#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>

#include "tmc6460.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(tmc6460, CONFIG_STEPPER_LOG_LEVEL);

/*
 * TMC6460 SPI protocol:
 * - SPI mode 1 (CPOL=0, CPHA=1)
 * - 48-bit (6-byte) datagrams
 * - Byte 0: write bit (bit 7) | address[9:8] (bits 1:0)
 * - Byte 1: address[7:0]
 * - Bytes 2-5: 32-bit data (MSB first)
 * - Read requires two transfers (first sends address, second returns data)
 */

#define TMC6460_SPI_WRITE_BIT        0x80U
#define TMC6460_DATAGRAM_SIZE        6U

struct tmc6460_config {
	struct spi_dt_spec spi;
};

struct tmc6460_data {
	struct k_sem sem;
};

/* Low-level 6-byte SPI transceive */

static int tmc6460_transceive(const struct device *dev,
			      uint8_t *tx, uint8_t *rx, size_t len)
{
	const struct tmc6460_config *config = dev->config;

	struct spi_buf tx_buf = {
		.buf = tx,
		.len = len,
	};
	struct spi_buf rx_buf = {
		.buf = rx,
		.len = len,
	};
	struct spi_buf_set tx_set = {
		.buffers = &tx_buf,
		.count = 1,
	};
	struct spi_buf_set rx_set = {
		.buffers = &rx_buf,
		.count = 1,
	};

	return spi_transceive_dt(&config->spi, &tx_set, &rx_set);
}

/* Register write */

int tmc6460_write(const struct device *dev, uint16_t reg_addr, uint32_t reg_val)
{
	struct tmc6460_data *data = dev->data;
	uint8_t tx[TMC6460_DATAGRAM_SIZE] = {0};
	uint8_t rx[TMC6460_DATAGRAM_SIZE] = {0};
	int err;

	/* Byte 0: write bit | address high bits */
	tx[0] = TMC6460_SPI_WRITE_BIT | ((reg_addr >> 8) & 0x03U);
	/* Byte 1: address low byte */
	tx[1] = reg_addr & 0xFFU;
	/* Bytes 2-5: data MSB first */
	tx[2] = (uint8_t)(reg_val >> 24);
	tx[3] = (uint8_t)(reg_val >> 16);
	tx[4] = (uint8_t)(reg_val >> 8);
	tx[5] = (uint8_t)(reg_val);

	k_sem_take(&data->sem, K_FOREVER);

	err = tmc6460_transceive(dev, tx, rx, TMC6460_DATAGRAM_SIZE);

	k_sem_give(&data->sem);

	if (err < 0) {
		LOG_ERR("Failed to write register 0x%04x: %d", reg_addr, err);
	}

	return err;
}

/* Register read */

int tmc6460_read(const struct device *dev, uint16_t reg_addr, uint32_t *reg_val)
{
	struct tmc6460_data *data = dev->data;
	uint8_t tx[TMC6460_DATAGRAM_SIZE] = {0};
	uint8_t rx[TMC6460_DATAGRAM_SIZE] = {0};
	int err;

	/* Byte 0: read (write bit clear) | address high bits */
	tx[0] = (reg_addr >> 8) & 0x03U;
	/* Byte 1: address low byte */
	tx[1] = reg_addr & 0xFFU;

	k_sem_take(&data->sem, K_FOREVER);

	/* First transfer: send read request */
	err = tmc6460_transceive(dev, tx, rx, TMC6460_DATAGRAM_SIZE);
	if (err < 0) {
		k_sem_give(&data->sem);
		LOG_ERR("Failed to read register 0x%04x (request): %d", reg_addr, err);
		return err;
	}

	/* Second transfer: clock out response */
	memset(tx, 0, TMC6460_DATAGRAM_SIZE);
	tx[0] = (reg_addr >> 8) & 0x03U;
	tx[1] = reg_addr & 0xFFU;

	err = tmc6460_transceive(dev, tx, rx, TMC6460_DATAGRAM_SIZE);

	k_sem_give(&data->sem);

	if (err < 0) {
		LOG_ERR("Failed to read register 0x%04x (response): %d", reg_addr, err);
		return err;
	}

	/* Data is in bytes 2-5 of the response */
	*reg_val = ((uint32_t)rx[2] << 24) |
		   ((uint32_t)rx[3] << 16) |
		   ((uint32_t)rx[4] << 8) |
		   ((uint32_t)rx[5]);

	return 0;
}

/* Init */

static int tmc6460_init(const struct device *dev)
{
	const struct tmc6460_config *config = dev->config;
	struct tmc6460_data *data = dev->data;

	LOG_DBG("Initializing TMC6460 %s", dev->name);

	k_sem_init(&data->sem, 1, 1);

	if (!spi_is_ready_dt(&config->spi)) {
		LOG_ERR("SPI bus not ready");
		return -ENODEV;
	}

	LOG_DBG("TMC6460 %s ready", dev->name);
	return 0;
}

/* Device instantiation */

#define TMC6460_DEFINE(inst)                                                       \
	static struct tmc6460_data tmc6460_data_##inst;                            \
                                                                                   \
	static const struct tmc6460_config tmc6460_config_##inst = {                \
		.spi = SPI_DT_SPEC_INST_GET(inst,                                  \
					    (SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB |   \
					     SPI_MODE_CPHA | SPI_WORD_SET(8)),         \
					    0),                                        \
	};                                                                         \
                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, tmc6460_init, NULL,                            \
			      &tmc6460_data_##inst, &tmc6460_config_##inst,        \
			      POST_KERNEL, CONFIG_STEPPER_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(TMC6460_DEFINE)
