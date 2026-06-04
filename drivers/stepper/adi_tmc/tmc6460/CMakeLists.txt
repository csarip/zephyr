# SPDX-FileCopyrightText: Copyright (c) 2026 Cherrence Sarip <cherrence.sarip@analog.com>
# SPDX-License-Identifier: Apache-2.0

zephyr_library_sources_ifdef(CONFIG_STEPPER_ADI_TMC6460 tmc6460.c)
