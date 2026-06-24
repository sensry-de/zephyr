#include <errno.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/video.h>
//#include <zephyr/drivers/video/video_device.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(htpa, CONFIG_VIDEO_LOG_LEVEL);

#define DT_DRV_COMPAT heimann_htpa

struct hm_htpa_config
{
	struct spi_dt_spec bus;
};

struct hm_htpa_data
{
	struct video_format fmt;
	bool streaming;
};

static const struct video_format_cap hm_htpa_caps[] = {
	{
		.pixelformat = VIDEO_PIX_FMT_RGB565,
		.width_min = 320,
		.width_max = 320,
		.height_min = 240,
		.height_max = 240,
		.width_step = 0,
		.height_step = 0,
	},
	{0}
};

static int hm_htpa_get_caps(const struct device *dev, struct video_caps *caps)
{
	ARG_UNUSED(dev);

	caps->type = VIDEO_BUF_TYPE_OUTPUT;
	caps->format_caps = hm_htpa_caps;
	caps->min_vbuf_count = 0;
	caps->buf_align = 1;

	return 0;
}

static int hm_htpa_set_fmt(const struct device *dev, struct video_format *fmt)
{
	struct hm_htpa_data *data = dev->data;

	if (fmt->type != VIDEO_BUF_TYPE_OUTPUT) {
		return -ENOTSUP;
	}

	if (fmt->pixelformat != VIDEO_PIX_FMT_RGB565) {
		return -ENOTSUP;
	}

	if (fmt->width != 320 || fmt->height != 240) {
		return -ENOTSUP;
	}

	fmt->pitch = fmt->width * 2U;
	fmt->size = fmt->pitch * fmt->height;

	memcpy(&data->fmt, fmt, sizeof(data->fmt));

	return 0;
}

static int hm_htpa_get_fmt(const struct device *dev, struct video_format *fmt)
{
	struct hm_htpa_data *data = dev->data;

	memcpy(fmt, &data->fmt, sizeof(*fmt));

	return 0;
}

static int hm_htpa_set_stream(const struct device *dev, bool enable,
                              enum video_buf_type type)
{
	struct hm_htpa_data *data = dev->data;

	if (type != VIDEO_BUF_TYPE_OUTPUT) {
		return -ENOTSUP;
	}

	data->streaming = enable;
	LOG_INF("SPI IR camera stream %s", enable ? "enabled" : "disabled");

	return 0;
}

static DEVICE_API(video, hm_htpa_api) = {
	.set_format = hm_htpa_set_fmt,
	.get_format = hm_htpa_get_fmt,
	.set_stream = hm_htpa_set_stream,
	.get_caps = hm_htpa_get_caps,
};

static int hm_htpa_init(const struct device *dev)
{
	const struct hm_htpa_config *cfg = dev->config;
	struct hm_htpa_data *data = dev->data;
	struct video_format default_fmt = {
		.type = VIDEO_BUF_TYPE_OUTPUT,
		.pixelformat = VIDEO_PIX_FMT_RGB565,
		.width = 320,
		.height = 240,
		.pitch = 320 * 2U,
		.size = 320 * 240 * 2U,
	};

	if (!spi_is_ready_dt(&cfg->bus)) {
		LOG_ERR("SPI bus not ready");
		return -ENODEV;
	}

	memcpy(&data->fmt, &default_fmt, sizeof(data->fmt));
	data->streaming = false;

	LOG_INF("SPI IR camera initialized on %s", cfg->bus.bus->name);

	return 0;
}

#define HM_HTPA_INIT(inst)                                                             \
	static const struct hm_htpa_config hm_htpa_cfg_##inst = {             \
		.bus = SPI_DT_SPEC_INST_GET(                                                   \
			inst,                                                                  \
			SPI_OP_MODE_MASTER | SPI_WORD_SET(8) | SPI_LINES_SINGLE),              \
	};                                                                                     \
	static struct hm_htpa_data hm_htpa_data_##inst;                       \
	DEVICE_DT_INST_DEFINE(inst, hm_htpa_init, NULL,                                 \
			      &hm_htpa_data_##inst, &hm_htpa_cfg_##inst,      \
			      POST_KERNEL, CONFIG_VIDEO_INIT_PRIORITY, &hm_htpa_api);   //\
	//VIDEO_DEVICE_DEFINE(hm_htpa_##inst, DEVICE_DT_INST_GET(inst), NULL);

DT_INST_FOREACH_STATUS_OKAY(HM_HTPA_INIT)
