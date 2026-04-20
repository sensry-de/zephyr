# SPDX-FileCopyrightText: Copyright 2026 sensry.io
# SPDX-License-Identifier: Apache-2.0

zephyr_include_directories(${SOC_FAMILY}/${SOC_SERIES}/common)

add_subdirectory_ifdef(CONFIG_SOC_SERIES_EV1X edgevision/ev1x)
