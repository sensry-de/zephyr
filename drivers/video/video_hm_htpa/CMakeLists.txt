# Copyright 2026 sensry.io
# SPDX-License-Identifier: Apache-2.0

if(CONFIG_VIDEO_HEIMANN_HTPA)
  zephyr_library_include_directories(
    common
    ./
  )
  zephyr_library_sources(
    common/htpa-calib.c
    common/htpa-proc.c
    htpa-common.c
  )
endif()
