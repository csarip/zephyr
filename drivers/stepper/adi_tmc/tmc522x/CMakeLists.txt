# SPDX-FileCopyrightText: Copyright (c) 2026 Cherrence Sarip <Cherrence.sarip@analog.com>
# SPDX-License-Identifier: Apache-2.0

zephyr_library_sources_ifdef(CONFIG_STEPPER_ADI_TMC522X tmc522x.c)
zephyr_library_sources_ifdef(CONFIG_TMC522X_BUS_SPI tmc522x_spi.c)
zephyr_library_sources_ifdef(CONFIG_TMC522X_BUS_I2C tmc522x_i2c.c)
