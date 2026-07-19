/*
 * Copyright (c) 2026 sensry.io
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(htpa_120x84, CONFIG_VIDEO_LOG_LEVEL);

#define DT_DRV_COMPAT heimann_htpa_120x84

#include "common/htpa-common.h"

#define HTPA_DEFAULT_ACQUISITION_TIME_USEC 1000

#if defined(CONFIG_VIDEO_HM_HTPA_AUTOSCALE_HISTOGRAM)
BUILD_ASSERT((120 * 84) <= UINT16_MAX, "Histogram counter type is too small");
#endif

static const struct hm_htpa_sensor_config hm_htpa_sensor_cfg_120x84 = {
	.width = 120,
	.height = 84,
	.block_count = 6,
	.pixels_per_block = 840,
	.block_length = 1682,
	.data_offset = 6,
	.e_id = {0x0074, 0x0075, 0x0076, 0x0077},
	.e_mbit_calib = 0x001a,
	.e_bias_calib = 0x001b,
	.e_clk_calib = 0x001c,
	.e_bpa_calib = 0x001d,
};

static const struct video_format_cap hm_htpa_caps_120x84[] = {{
							       .pixelformat = VIDEO_PIX_FMT_Y16,
							       .width_min = 120,
							       .width_max = 120,
							       .height_min = 84,
							       .height_max = 84,
							       .width_step = 0,
							       .height_step = 0,
						       },
						       {0}};

#define HM_HTPA_INIT(inst)                                                                         \
	static int16_t __aligned(4) hm_htpa_pixels_##inst[HTPA_FRAME_QUEUE_SIZE][120 * 84];        \
	static uint8_t __aligned(4) hm_htpa_raw_top_##inst[6][1682];                               \
	static uint8_t                                                                             \
		__aligned(4) hm_htpa_raw_bottom_##inst[6][1682];                                  \
	static uint8_t __aligned(4) hm_htpa_el_top_##inst[1682];                                   \
	static uint8_t __aligned(4) hm_htpa_el_bottom_##inst[1682];                                \
	static const struct hm_htpa_config hm_htpa_cfg_##inst = {                                  \
		.hm_htpa_caps = hm_htpa_caps_120x84,                                               \
		.calibration_flash = DEVICE_DT_GET(DT_INST_PHANDLE(inst, calibration_flash)),      \
		.sensor = &hm_htpa_sensor_cfg_120x84,                                              \
	};                                                                                         \
	static struct hm_htpa_data hm_htpa_data_##inst = {                                         \
		.grab =                                                                            \
			{                                                                          \
				.acquisition_time = HTPA_DEFAULT_ACQUISITION_TIME_USEC,            \
				.frames = {{.pixels = hm_htpa_pixels_##inst[0]},                   \
					   {.pixels = hm_htpa_pixels_##inst[1]}},                  \
				.raw_top = &hm_htpa_raw_top_##inst[0][0],                          \
				.raw_bottom = &hm_htpa_raw_bottom_##inst[0][0],                    \
				.el_top_offsets = hm_htpa_el_top_##inst,                           \
				.el_bottom_offsets = hm_htpa_el_bottom_##inst,                     \
			},                                                                         \
		.spi = SPI_DT_SPEC_INST_GET(inst, SPI_OP_MODE_MASTER | SPI_WORD_SET(8) |           \
							  SPI_LINES_SINGLE),                       \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(inst, hm_htpa_init, NULL, &hm_htpa_data_##inst, &hm_htpa_cfg_##inst, \
			      POST_KERNEL, CONFIG_VIDEO_INIT_PRIORITY, &hm_htpa_api);

DT_INST_FOREACH_STATUS_OKAY(HM_HTPA_INIT)
