# SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
# SPDX-FileCopyrightText: Copyright (c) 2026 Analog Devices Inc.
# SPDX-License-Identifier: Apache-2.0

zephyr_library_sources_ifdef(CONFIG_STEPPER_ADI_TMC5262 tmc5262.c)
zephyr_library_sources_ifdef(CONFIG_STEPPER_ADI_TMC5262_STEPPER_CTRL tmc5262_stepper_ctrl.c)
zephyr_library_sources_ifdef(CONFIG_STEPPER_ADI_TMC5262_STEPPER_DRIVER tmc5262_stepper_driver.c)
