# SPDX-FileCopyrightText: Copyright (c) 2026 Analog Devices, Inc.
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.28.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(tmc5262)

target_sources(app PRIVATE src/main.c src/tmc5262_emul.c)
# Register map of the driver, used by the emulator and the tests
target_include_directories(app PRIVATE
	${ZEPHYR_BASE}/drivers/stepper/adi_tmc/tmc5262
	${ZEPHYR_BASE}/drivers/stepper/adi_tmc/bus/include)
