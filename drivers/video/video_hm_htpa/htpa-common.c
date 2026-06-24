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

static void hm_htpa_fill_mock_frame(struct hm_htpa_data *data, struct video_buffer *vbuf)
{
	uint8_t *row;
	uint16_t sample;
	static uint32_t frame = 0;

	for (uint32_t y = 0; y < data->fmt.height; y++) {
		row = vbuf->buffer + (y * data->fmt.pitch);
		for (uint32_t x = 0; x < data->fmt.width; x++) {
			sample = (uint16_t)((frame * 23U) + (y * 3U) + x);
			sys_put_le16(sample, row + (x * sizeof(uint16_t)));
		}
	}

	vbuf->bytesused = data->fmt.size;
	vbuf->timestamp = k_uptime_get_32();
	vbuf->line_offset = 0;
}

static void hm_htpa_worker(void *p1, void *p2, void *p3)
{
	const struct device *dev = p1;
	struct hm_htpa_data *data = dev->data;
	struct video_buffer *vbuf;

	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (true) {
		k_sem_take(&data->worker_sem, K_FOREVER);

		if (!data->streaming) {
			continue;
		}

		while (data->streaming) {
			vbuf = k_fifo_get(&data->framebuffer_take_queue, K_NO_WAIT);
			if (vbuf == NULL) {
				break;
			}

			htpa_copy_frame(dev, vbuf);

			k_fifo_put(&data->framebuffer_release_queue, vbuf);
		}
	}
}

static int hm_htpa_get_caps(const struct device *dev, struct video_caps *caps)
{
	const struct hm_htpa_config *cfg = dev->config;

	caps->type = VIDEO_BUF_TYPE_OUTPUT;
	caps->format_caps = cfg->hm_htpa_caps;
	caps->min_vbuf_count = 1;
	caps->buf_align = 1;

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

static int hm_htpa_set_stream(const struct device *dev, bool enable,
                              enum video_buf_type type)
{
	struct hm_htpa_data *data = dev->data;

	if (type != VIDEO_BUF_TYPE_OUTPUT) {
		return -ENOTSUP;
	}

	data->streaming = enable;
	LOG_INF("SPI IR camera stream %s", enable ? "enabled" : "disabled");

	if (enable) {
		k_sem_give(&data->worker_sem);
	}

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

	vbuf->bytesused = 0;
	vbuf->timestamp = 0;
	k_fifo_put(&data->framebuffer_take_queue, vbuf);

	if (data->streaming) {
		k_sem_give(&data->worker_sem);
	}

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

	if (cancel) {
		while ((vbuf = k_fifo_get(&data->framebuffer_take_queue, K_NO_WAIT)) != NULL) {
			vbuf->bytesused = 0;
			vbuf->timestamp = k_uptime_get_32();
			k_fifo_put(&data->framebuffer_release_queue, vbuf);
		}
	}

	return 0;
}

DEVICE_API(video, hm_htpa_api) = {
	.set_format = hm_htpa_set_fmt,
	.get_format = hm_htpa_get_fmt,
	.set_stream = hm_htpa_set_stream,
	.get_caps = hm_htpa_get_caps,
	.enqueue = hm_htpa_enqueue,
	.dequeue = hm_htpa_dequeue,
	.flush = hm_htpa_flush,
};

int hm_htpa_init(const struct device *dev)
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
	 * The cs-pin (PA24) is shared:
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

	if (0 != htpa_init_sensor(dev)) {
		LOG_ERR("Failed to initialize sensor %s", dev->name);
		return -EINVAL;
	}

	LOG_INF("HTPA camera device is ready");

	memcpy(&data->fmt, &default_fmt, sizeof(data->fmt));
	k_fifo_init(&data->framebuffer_take_queue);
	k_fifo_init(&data->framebuffer_release_queue);
	k_sem_init(&data->worker_sem, 0, 1);
	k_thread_create(&data->worker_thread, data->worker_stack,
			K_THREAD_STACK_SIZEOF(data->worker_stack), hm_htpa_worker, dev,
			NULL, NULL, CONFIG_VIDEO_HM_HTPA_WORKER_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&data->worker_thread, "htpa_worker");

	data->streaming = false;

	LOG_INF("SPI IR camera initialized on %s", data->spi.bus->name);

	return 0;
}
