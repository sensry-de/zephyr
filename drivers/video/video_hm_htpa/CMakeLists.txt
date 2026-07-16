# Copyright 2026 sensry.io
# SPDX-License-Identifier: Apache-2.0

if(CONFIG_VIDEO_HEIMANN_HTPA)
  zephyr_library_sources(
    common/htpa-calib.c
    common/htpa-common.c
    common/htpa-proc.c
    common/htpa-sens.c
  )
  zephyr_library_sources_ifdef(CONFIG_DT_HAS_HEIMANN_HTPA_160X120_ENABLED  htpa-160x120.c)
  zephyr_library_sources_ifdef(CONFIG_DT_HAS_HEIMANN_HTPA_120X84_ENABLED  htpa-120x84.c)

  ##zephyr_library_sources(eth_nxp_imx_netc_psi.c)
  ##zephyr_library_sources_ifdef(CONFIG_DT_HAS_NXP_IMX_NETC_BLK_CTRL_ENABLED eth_nxp_imx_netc_blk.c)
endif()
