# SPDX-FileCopyrightText: Copyright (c) 2026 Analog Devices, Inc.
# SPDX-License-Identifier: Apache-2.0

zephyr_library_sources(tmc5262.c tmc5262_variant.c)
zephyr_library_sources_ifdef(CONFIG_STEPPER_ADI_TMC5262_STEPPER_DRIVER tmc5262_stepper_driver.c)
zephyr_library_sources_ifdef(CONFIG_STEPPER_ADI_TMC5262_STEPPER_CTRL tmc5262_stepper_ctrl.c)
zephyr_library_sources_ifdef(CONFIG_STEPPER_ADI_TMC5262_SENSOR tmc5262_sensor.c)
