/**
 * @file drivers/stepper/adi_tmc/bus/include/adi_tmc_spi.h
 *
 * @brief Private API for Trinamic SPI bus
 *
 */

/*
 * SPDX-FileCopyrightText: Copyright (c) 2024 Carl Zeiss Meditec AG
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_STEPPER_ADI_TMC_BUS_SPI_H_
#define ZEPHYR_DRIVERS_STEPPER_ADI_TMC_BUS_SPI_H_

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief TMC SPI INTERFACE
 * @ingroup io_priv_interfaces
 * @{
 *
 */

#include <zephyr/drivers/spi.h>

/**
 * @brief Read a register from the TMC module using the SPI Bus.
 *
 * @param bus SPI DT information of the bus.
 * @param read_address_mask Address Mask for read operation.
 * @param register_address Register.
 * @param data Pointer to read value.
 *
 * @return a value from spi_transceive().
 */
int tmc_spi_read_register(const struct spi_dt_spec *bus, const uint8_t read_address_mask,
			  const uint8_t register_address, uint32_t *data);

/**
 * @brief Write into a register in the TMC module using the SPI Bus.
 *
 * @param bus SPI DT information of the bus.
 * @param write_bit Write bit for write operation.
 * @param register_address Register.
 * @param data Value to be written in the register.
 *
 * @return a value from spi_transceive().
 */
int tmc_spi_write_register(const struct spi_dt_spec *bus, const uint8_t write_bit,
			   const uint8_t register_address, const uint32_t data);

/**
 * @brief Read a register and capture the SPI status byte.
 *
 * Same as tmc_spi_read_register() but additionally returns byte 0 of the
 * second (data) datagram, which carries the SPI_STATUS of devices that provide
 * one (TMC5xxx / TMC22xx3 families).
 *
 * @param bus SPI DT information of the bus.
 * @param read_address_mask Address Mask for read operation.
 * @param register_address Register.
 * @param data Pointer to read value.
 * @param status_byte Pointer to store the SPI status byte, may be NULL.
 *
 * @return a value from spi_transceive().
 */
int tmc_spi_read_register_status(const struct spi_dt_spec *bus, const uint8_t read_address_mask,
				 const uint8_t register_address, uint32_t *data,
				 uint8_t *status_byte);

/**
 * @brief Write a register and capture the SPI status byte.
 *
 * Same as tmc_spi_write_register() but additionally returns byte 0 of the
 * datagram response (SPI_STATUS).
 *
 * @param bus SPI DT information of the bus.
 * @param write_bit Write bit for write operation.
 * @param register_address Register.
 * @param data Value to be written in the register.
 * @param status_byte Pointer to store the SPI status byte, may be NULL.
 *
 * @return a value from spi_transceive().
 */
int tmc_spi_write_register_status(const struct spi_dt_spec *bus, const uint8_t write_bit,
				  const uint8_t register_address, const uint32_t data,
				  uint8_t *status_byte);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_DRIVERS_STEPPER_ADI_TMC_BUS_SPI_H_ */
