/*
 * Copyright (c) 2024 Analog Devices Inc.
 * Copyright (c) 2024 Baylibre SAS
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_MFD_MAX2221X_H_
#define ZEPHYR_INCLUDE_DRIVERS_MFD_MAX2221X_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <zephyr/device.h>
#include <zephyr/sys/util.h>

#define MAX2221X_SPI_TRANS_ADDR    GENMASK(6, 0)
#define MAX2221X_SPI_TRANS_DIR     BIT(7)

/**
 * @defgroup mdf_interface_max2221x MFD MAX2221X interface
 * @ingroup mfd_interfaces
 * @{
 */

/**
 * @brief Read register from max2221x
 *
 * @param dev max2221x mfd device
 * @param addr register address to read from
 * @param value pointer to buffer for received data
 * @retval 0 If successful
 * @retval -errno In case of any error (see spi_transceive_dt())
 */
int max2221x_reg_read(const struct device *dev, uint8_t addr, uint16_t *value);

/**
 * @brief Write register to max2221x
 *
 * @param dev max2221x mfd device
 * @param addr register address to write to
 * @param value content to write
 * @retval 0 If successful
 * @retval -errno In case of any error (see spi_write_dt())
 */
int max2221x_reg_write(const struct device *dev, uint8_t addr, uint16_t value);

/**
 * @brief Update register in max2221x
 *
 * @param dev max2221x mfd device
 * @param addr register address to update
 * @param mask mask to apply to the register
 * @param val value to write to the register
 * @retval 0 If successful
 * @retval -errno In case of any error (see spi_transceive_dt())
 */
int max2221x_reg_update(const struct device *dev, uint8_t addr, uint16_t mask,
			uint16_t val);
                        
/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_MFD_MAX2221X_H_ */
