# SPDX-FileCopyrightText: Copyright 2026 sensry.io
# SPDX-License-Identifier: Apache-2.0

zephyr_include_directories(common)

zephyr_sources(
        common/soc.c
)

#zephyr_linker_sources(ROM_START SORT_KEY 0x0vectors common/vector_table.ld)

set(SOC_LINKER_SCRIPT ${ZEPHYR_BASE}/include/zephyr/arch/arm/cortex_m/scripts/linker.ld CACHE INTERNAL "")
