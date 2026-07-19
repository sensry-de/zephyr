/*
 * Copyright (c) 2026 sensry.io
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(htpa_160x120, CONFIG_VIDEO_LOG_LEVEL);

#define DT_DRV_COMPAT heimann_htpa_160x120

#include "common/htpa-common.h"

#define HTPA_DEFAULT_ACQUISITION_TIME_USEC 1000

enum {
	HTPA_WIDTH_160X120 = 160,
	HTPA_HEIGHT_160X120 = 120,
	HTPA_BLOCK_COUNT_160X120 = 12,
	HTPA_PIXELS_PER_BLOCK_160X120 = 800,
	HTPA_BLOCK_LENGTH_160X120 = 1606,
	HTPA_DATA_OFFSET_160X120 = 6,
};

#if defined(CONFIG_VIDEO_HM_HTPA_AUTOSCALE_HISTOGRAM)
BUILD_ASSERT((HTPA_WIDTH_160X120 * HTPA_HEIGHT_160X120) <= UINT16_MAX,
	     "Histogram counter type is too small");
#endif

static const struct hm_htpa_sensor_config hm_htpa_sensor_cfg_160x120 = {
	.width = HTPA_WIDTH_160X120,
	.height = HTPA_HEIGHT_160X120,
	.block_count = HTPA_BLOCK_COUNT_160X120,
	.pixels_per_block = HTPA_PIXELS_PER_BLOCK_160X120,
	.block_length = HTPA_BLOCK_LENGTH_160X120,
	.data_offset = HTPA_DATA_OFFSET_160X120,
	.e_id = {0x0074, 0x0075, 0x0076, 0x0077},
	.e_mbit_calib = 0x001a,
	.e_bias_calib = 0x001b,
	.e_clk_calib = 0x001c,
	.e_bpa_calib = 0x001d,
};

static const struct video_format_cap hm_htpa_caps_160x120[] = {{
							       .pixelformat = VIDEO_PIX_FMT_Y16,
							       .width_min = HTPA_WIDTH_160X120,
							       .width_max = HTPA_WIDTH_160X120,
							       .height_min = HTPA_HEIGHT_160X120,
							       .height_max = HTPA_HEIGHT_160X120,
							       .width_step = 0,
							       .height_step = 0,
						       },
						       {0}};

#define HM_HTPA_INIT(inst)                                                                         \
	static int16_t __aligned(4) hm_htpa_pixels_##inst[HTPA_FRAME_QUEUE_SIZE]                   \
				 [HTPA_WIDTH_160X120 * HTPA_HEIGHT_160X120];                     \
	static uint8_t __aligned(4)                                                               \
		hm_htpa_raw_top_##inst[HTPA_BLOCK_COUNT_160X120][HTPA_BLOCK_LENGTH_160X120];       \
	static uint8_t                                                                             \
		__aligned(4) hm_htpa_raw_bottom_##inst[HTPA_BLOCK_COUNT_160X120]                   \
						  [HTPA_BLOCK_LENGTH_160X120];                    \
	static uint8_t                                                                             \
		__aligned(4) hm_htpa_el_top_##inst[HTPA_BLOCK_LENGTH_160X120]; \
	static uint8_t                                                                             \
		__aligned(4) hm_htpa_el_bottom_##inst[HTPA_BLOCK_LENGTH_160X120]; \
	static const struct hm_htpa_config hm_htpa_cfg_##inst = {                                  \
		.hm_htpa_caps = hm_htpa_caps_160x120,                                              \
		.calibration_flash = DEVICE_DT_GET(DT_INST_PHANDLE(inst, calibration_flash)),      \
		.sensor = &hm_htpa_sensor_cfg_160x120,                                             \
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
