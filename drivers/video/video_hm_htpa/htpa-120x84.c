//
// Created by tswaehn on 7/13/26.
//

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(htpa_120x84, CONFIG_VIDEO_LOG_LEVEL);

#define DT_DRV_COMPAT heimann_htpa_120x84

#include "htpa-common.h"

static const struct video_format_cap hm_htpa_caps[] = {
	{
		.pixelformat = VIDEO_PIX_FMT_Y16,
		.width_min = 120,
		.width_max = 120,
		.height_min = 84,
		.height_max = 84,
		.width_step = 0,
		.height_step = 0,
	},
	{0}
};


#define HM_HTPA_INIT(inst)									\
	static const struct hm_htpa_config hm_htpa_cfg_## inst = {				\
		.hm_htpa_caps = hm_htpa_caps,							\
		.calibration_flash = DEVICE_DT_GET(						\
			DT_INST_PHANDLE(inst, calibration_flash)),				\
	};											\
	static struct hm_htpa_data hm_htpa_data_## inst = {					\
		.grab = {									\
			.acquisition_time = HTPA_DEFAULT_ACQUISITION_TIME_USEC,			\
		},										\
		.spi = SPI_DT_SPEC_INST_GET( inst,						\
			SPI_OP_MODE_MASTER | SPI_WORD_SET(8) | SPI_LINES_SINGLE),		\
	};											\
	DEVICE_DT_INST_DEFINE(inst, hm_htpa_init, NULL,						\
		&hm_htpa_data_## inst, &hm_htpa_cfg_## inst,					\
		POST_KERNEL, CONFIG_VIDEO_INIT_PRIORITY, &hm_htpa_api);

	//VIDEO_DEVICE_DEFINE(hm_htpa_##inst, DEVICE_DT_INST_GET(inst), NULL);

DT_INST_FOREACH_STATUS_OKAY(HM_HTPA_INIT)
