/*
 * Copyright (c) 2026 sensry.io
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include "htpa-sens.h"

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(htpa, CONFIG_VIDEO_LOG_LEVEL);

#include "htpa-common.h"

static void hm_htpa_worker(void *p1, void *p2, void *p3)
{
	const struct device *dev = p1;
	struct hm_htpa_data *data = dev->data;
	struct video_buffer *vbuf;

	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (true) {
		k_sem_take(&data->worker_sem, K_FOREVER);

		while (true) {
			int ret;

			k_mutex_lock(&data->lock, K_FOREVER);
			if (data->canceling || (!data->streaming && !data->flushing)) {
				k_mutex_unlock(&data->lock);
				break;
			}

			vbuf = k_fifo_get(&data->framebuffer_take_queue, K_NO_WAIT);
			if (vbuf == NULL) {
				k_condvar_broadcast(&data->state_changed);
				k_mutex_unlock(&data->lock);
				break;
			}

			data->in_flight = vbuf;
			atomic_clear(&data->in_flight_canceled);
			k_mutex_unlock(&data->lock);

			ret = htpa_consume_frame(dev, vbuf);

			k_mutex_lock(&data->lock, K_FOREVER);
			if (ret != 0 || atomic_get(&data->in_flight_canceled)) {
				vbuf->bytesused = 0;
				vbuf->timestamp = k_uptime_get_32();
				if (ret != 0 && ret != -ECANCELED) {
					LOG_ERR("Frame acquisition failed: %d", ret);
				}
			}

			k_fifo_put(&data->framebuffer_release_queue, vbuf);
			data->in_flight = NULL;
			k_condvar_broadcast(&data->state_changed);
			k_mutex_unlock(&data->lock);
		}
	}
}

static int hm_htpa_get_caps(const struct device *dev, struct video_caps *caps)
{
	const struct hm_htpa_config *cfg = dev->config;

	caps->type = VIDEO_BUF_TYPE_OUTPUT;
	caps->format_caps = cfg->hm_htpa_caps;
	caps->min_vbuf_count = 1;
	caps->buf_align = sizeof(uint16_t);

	return 0;
}

static int hm_htpa_set_fmt(const struct device *dev, struct video_format *fmt)
{
	struct hm_htpa_data *data = dev->data;

	if (fmt->type != data->fmt.type) {
		return -ENOTSUP;
	}

	if (fmt->pixelformat != data->fmt.pixelformat) {
		return -ENOTSUP;
	}

	if (fmt->width != data->fmt.width || fmt->height != data->fmt.height) {
		return -ENOTSUP;
	}

	fmt->pitch = data->fmt.pitch;
	fmt->size = data->fmt.size;

	memcpy(&data->fmt, fmt, sizeof(data->fmt));

	return 0;
}

static int hm_htpa_get_fmt(const struct device *dev, struct video_format *fmt)
{
	struct hm_htpa_data *data = dev->data;

	memcpy(fmt, &data->fmt, sizeof(*fmt));

	return 0;
}

static int hm_htpa_set_stream(const struct device *dev, bool enable, enum video_buf_type type)
{
	struct hm_htpa_data *data = dev->data;

	if (type != VIDEO_BUF_TYPE_OUTPUT) {
		return -ENOTSUP;
	}

	k_mutex_lock(&data->lock, K_FOREVER);
	data->streaming = enable;
	LOG_INF("SPI IR camera stream %s", enable ? "enabled" : "disabled");

	if (enable) {
		k_sem_give(&data->worker_sem);
	}
	k_mutex_unlock(&data->lock);

	return 0;
}

static int hm_htpa_enqueue(const struct device *dev, struct video_buffer *vbuf)
{
	struct hm_htpa_data *data = dev->data;
	struct video_format *fmt = &data->fmt;

	if (vbuf->type != VIDEO_BUF_TYPE_OUTPUT) {
		return -EINVAL;
	}

	if (vbuf->size < fmt->size) {
		return -EINVAL;
	}
	if (!IS_ALIGNED((uintptr_t)vbuf->buffer, sizeof(uint16_t))) {
		return -EINVAL;
	}

	k_mutex_lock(&data->lock, K_FOREVER);
	if (data->flushing) {
		k_mutex_unlock(&data->lock);
		return -EBUSY;
	}
	vbuf->bytesused = 0;
	vbuf->timestamp = 0;
	k_fifo_put(&data->framebuffer_take_queue, vbuf);

	if (data->streaming) {
		k_sem_give(&data->worker_sem);
	}
	k_mutex_unlock(&data->lock);

	return 0;
}

static int hm_htpa_dequeue(const struct device *dev, struct video_buffer **vbuf,
			   k_timeout_t timeout)
{
	struct hm_htpa_data *data = dev->data;

	*vbuf = k_fifo_get(&data->framebuffer_release_queue, timeout);
	if (*vbuf == NULL) {
		return -EAGAIN;
	}

	return 0;
}

static int hm_htpa_flush(const struct device *dev, bool cancel)
{
	struct hm_htpa_data *data = dev->data;
	struct video_buffer *vbuf;

	k_mutex_lock(&data->lock, K_FOREVER);
	while (data->flushing) {
		k_condvar_wait(&data->state_changed, &data->lock, K_FOREVER);
	}

	data->flushing = true;
	data->canceling = cancel;
	if (cancel) {
		if (data->in_flight != NULL) {
			atomic_set(&data->in_flight_canceled, 1);
		}

		while ((vbuf = k_fifo_get(&data->framebuffer_take_queue, K_NO_WAIT)) != NULL) {
			vbuf->bytesused = 0;
			vbuf->timestamp = k_uptime_get_32();
			k_fifo_put(&data->framebuffer_release_queue, vbuf);
		}
	}

	k_sem_give(&data->worker_sem);
	while (data->in_flight != NULL ||
	       (!cancel && !k_fifo_is_empty(&data->framebuffer_take_queue))) {
		k_condvar_wait(&data->state_changed, &data->lock, K_FOREVER);
	}

	data->canceling = false;
	data->flushing = false;
	k_condvar_broadcast(&data->state_changed);
	k_mutex_unlock(&data->lock);

	return 0;
}

static DEVICE_API(video, hm_htpa_api) = {
	.set_format = hm_htpa_set_fmt,
	.get_format = hm_htpa_get_fmt,
	.set_stream = hm_htpa_set_stream,
	.get_caps = hm_htpa_get_caps,
	.enqueue = hm_htpa_enqueue,
	.dequeue = hm_htpa_dequeue,
	.flush = hm_htpa_flush,
};

static int hm_htpa_init(const struct device *dev)
{
	const struct hm_htpa_config *cfg = dev->config;
	struct hm_htpa_data *data = dev->data;
	struct video_format default_fmt = {
		.type = VIDEO_BUF_TYPE_OUTPUT,
		.pixelformat = cfg->hm_htpa_caps[0].pixelformat,
		.width = cfg->hm_htpa_caps[0].width_max,
		.height = cfg->hm_htpa_caps[0].height_max,
		.pitch = cfg->hm_htpa_caps[0].width_max * 2,
		.size = cfg->hm_htpa_caps[0].width_max * cfg->hm_htpa_caps[0].height_max * 2,
	};

	data->communication_error = 0;
	data->communication_error_count = 0;

	if (!spi_is_ready_dt(&data->spi)) {
		LOG_ERR("SPI bus for sensor and flash not ready");
		return -ENODEV;
	}

	if (0 != htpa_read_calibration(dev, &data->calib)) {
		LOG_ERR("Error reading calibration data");
		return -EINVAL;
	}

	LOG_INF("Calibration data id: %d", data->calib.id);

	/*
	 * The CS pin (PA24) is shared:
	 *   LOW  = camera deselected, flash selected
	 *   HIGH = camera selected, flash deselected
	 *
	 * The flash is no longer needed, so leave the line configured
	 * according to the camera CS definition.
	 */
	data->spi.config.cs.gpio.dt_flags &= ~GPIO_ACTIVE_LOW;

	int ret = gpio_pin_configure_dt(&data->spi.config.cs.gpio, GPIO_OUTPUT_ACTIVE);

	if (ret < 0) {
		return ret;
	}

	if (0 != htpa_grab_start_acquisition(dev)) {
		LOG_ERR("Failed to initialize sensor %s", dev->name);
		return -EINVAL;
	}

	LOG_INF("HTPA camera device is ready");

	memcpy(&data->fmt, &default_fmt, sizeof(data->fmt));
	k_fifo_init(&data->framebuffer_take_queue);
	k_fifo_init(&data->framebuffer_release_queue);
	k_sem_init(&data->worker_sem, 0, 1);
	k_mutex_init(&data->lock);
	k_condvar_init(&data->state_changed);
	data->in_flight = NULL;
	data->streaming = false;
	data->flushing = false;
	data->canceling = false;
	atomic_clear(&data->in_flight_canceled);
	k_thread_create(&data->worker_thread, data->worker_stack,
			K_THREAD_STACK_SIZEOF(data->worker_stack), hm_htpa_worker, (void *)dev,
			NULL, NULL, CONFIG_VIDEO_HM_HTPA_WORKER_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&data->worker_thread, "htpa_worker");

	LOG_INF("SPI IR camera initialized on %s", data->spi.bus->name);

	return 0;
}

#define HTPA_DEFAULT_ACQUISITION_TIME_USEC 1000

#define HM_HTPA_DEFINE(inst, model, width, height, block_count, block_length, caps, sensor_cfg)    \
	static int16_t __aligned(4) hm_htpa_pixels_##model##_##inst[HTPA_FRAME_QUEUE_SIZE]         \
								   [(width) * (height)];           \
	static uint8_t __aligned(4) hm_htpa_raw_top_##model##_##inst[block_count][block_length];   \
	static uint8_t                                                                             \
		__aligned(4) hm_htpa_raw_bottom_##model##_##inst[block_count][block_length];       \
	static uint8_t __aligned(4) hm_htpa_el_top_##model##_##inst[block_length];                 \
	static uint8_t __aligned(4) hm_htpa_el_bottom_##model##_##inst[block_length];              \
	static const struct hm_htpa_config hm_htpa_cfg_##model##_##inst = {                        \
		.hm_htpa_caps = caps,                                                              \
		.calibration_flash = DEVICE_DT_GET(DT_INST_PHANDLE(inst, calibration_flash)),      \
		.sensor = sensor_cfg,                                                              \
	};                                                                                         \
	static struct hm_htpa_data hm_htpa_data_##model##_##inst = {                               \
		.grab =                                                                            \
			{                                                                          \
				.acquisition_time = HTPA_DEFAULT_ACQUISITION_TIME_USEC,            \
				.frames = {{.pixels = hm_htpa_pixels_##model##_##inst[0]},         \
					   {.pixels = hm_htpa_pixels_##model##_##inst[1]}},        \
				.raw_top = &hm_htpa_raw_top_##model##_##inst[0][0],                \
				.raw_bottom = &hm_htpa_raw_bottom_##model##_##inst[0][0],          \
				.el_top_offsets = hm_htpa_el_top_##model##_##inst,                 \
				.el_bottom_offsets = hm_htpa_el_bottom_##model##_##inst,           \
			},                                                                         \
		.spi = SPI_DT_SPEC_INST_GET(inst, SPI_OP_MODE_MASTER | SPI_WORD_SET(8) |           \
							  SPI_LINES_SINGLE),                       \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(inst, hm_htpa_init, NULL, &hm_htpa_data_##model##_##inst,            \
			      &hm_htpa_cfg_##model##_##inst, POST_KERNEL,                          \
			      CONFIG_VIDEO_INIT_PRIORITY, &hm_htpa_api);

#if defined(CONFIG_DT_HAS_HEIMANN_HTPA_120X84_ENABLED)

#define DT_DRV_COMPAT heimann_htpa_120x84

enum {
	HTPA_WIDTH_120X84 = 120,
	HTPA_HEIGHT_120X84 = 84,
	HTPA_BLOCK_COUNT_120X84 = 6,
	HTPA_PIXELS_PER_BLOCK_120X84 = 840,
	HTPA_BLOCK_LENGTH_120X84 = 1682,
	HTPA_DATA_OFFSET_120X84 = 6,
};

#if defined(CONFIG_VIDEO_HM_HTPA_AUTOSCALE_HISTOGRAM)
BUILD_ASSERT((HTPA_WIDTH_120X84 * HTPA_HEIGHT_120X84) <= UINT16_MAX,
	     "Histogram counter type is too small");
#endif

static const struct hm_htpa_sensor_config hm_htpa_sensor_cfg_120x84 = {
	.width = HTPA_WIDTH_120X84,
	.height = HTPA_HEIGHT_120X84,
	.block_count = HTPA_BLOCK_COUNT_120X84,
	.pixels_per_block = HTPA_PIXELS_PER_BLOCK_120X84,
	.block_length = HTPA_BLOCK_LENGTH_120X84,
	.data_offset = HTPA_DATA_OFFSET_120X84,
	.e_id = {0x0074, 0x0075, 0x0076, 0x0077},
	.e_mbit_calib = 0x001a,
	.e_bias_calib = 0x001b,
	.e_clk_calib = 0x001c,
	.e_bpa_calib = 0x001d,
};

static const struct video_format_cap hm_htpa_caps_120x84[] = {
	{
		.pixelformat = VIDEO_PIX_FMT_Y16,
		.width_min = HTPA_WIDTH_120X84,
		.width_max = HTPA_WIDTH_120X84,
		.height_min = HTPA_HEIGHT_120X84,
		.height_max = HTPA_HEIGHT_120X84,
		.width_step = 0,
		.height_step = 0,
	},
	{0},
};

#define HM_HTPA_INIT_120X84(inst)                                                                  \
	HM_HTPA_DEFINE(inst, htpa_120x84, HTPA_WIDTH_120X84, HTPA_HEIGHT_120X84,                   \
		       HTPA_BLOCK_COUNT_120X84, HTPA_BLOCK_LENGTH_120X84, hm_htpa_caps_120x84,     \
		       &hm_htpa_sensor_cfg_120x84)

DT_INST_FOREACH_STATUS_OKAY(HM_HTPA_INIT_120X84)

#undef DT_DRV_COMPAT

#endif /* CONFIG_DT_HAS_HEIMANN_HTPA_120X84_ENABLED */

#if defined(CONFIG_DT_HAS_HEIMANN_HTPA_160X120_ENABLED)

#define DT_DRV_COMPAT heimann_htpa_160x120

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

static const struct video_format_cap hm_htpa_caps_160x120[] = {
	{
		.pixelformat = VIDEO_PIX_FMT_Y16,
		.width_min = HTPA_WIDTH_160X120,
		.width_max = HTPA_WIDTH_160X120,
		.height_min = HTPA_HEIGHT_160X120,
		.height_max = HTPA_HEIGHT_160X120,
		.width_step = 0,
		.height_step = 0,
	},
	{0},
};

#define HM_HTPA_INIT_160X120(inst)                                                                 \
	HM_HTPA_DEFINE(inst, htpa_160x120, HTPA_WIDTH_160X120, HTPA_HEIGHT_160X120,                \
		       HTPA_BLOCK_COUNT_160X120, HTPA_BLOCK_LENGTH_160X120, hm_htpa_caps_160x120,  \
		       &hm_htpa_sensor_cfg_160x120)

DT_INST_FOREACH_STATUS_OKAY(HM_HTPA_INIT_160X120)

#undef DT_DRV_COMPAT

#endif /* CONFIG_DT_HAS_HEIMANN_HTPA_160X120_ENABLED */
