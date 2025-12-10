# SPDX-FileCopyrightText: Copyright (c) 2025 Cherrence Sarip <Cherrence.sarip@analog.com>
# SPDX-License-Identifier: Apache-2.0

zephyr_library_sources_ifdef(CONFIG_STEPPER_ADI_TMC5222 tmc5222.c)
