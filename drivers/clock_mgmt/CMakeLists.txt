# Copyright 2024 NXP
# SPDX-License-Identifier: Apache-2.0

zephyr_include_directories(.)
zephyr_library()

zephyr_library_sources(clock_mgmt_common.c)

# Include headers for all clock management drivers, registered with add_clock_mgmt_header
zephyr_library_compile_options("-include$<JOIN:$<TARGET_PROPERTY:clock_mgmt_header_target,COMPILE_DEFINITIONS>,;-include>")
