/*
 * Copyright (c) 2026 sensry.io
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>

#include <zephyr/sys/util.h>

#include "htpa-common.h"
#include "htpa-proc.h"
#include "htpa-sens.h"

#define HTPA_AUTOSCALE_CLIP_PERCENT 1U

#if defined(CONFIG_VIDEO_HM_HTPA_AUTOSCALE_LEGACY)
static void htpa_legacy_min_max(const struct hm_htpa_sensor_config *sensor,
				const struct hm_htpa_frame *frame, int32_t *minimum,
				int32_t *maximum)
{
	uint32_t minimum_count = 0U;
	uint32_t maximum_count = 0U;

	*minimum = INT16_MAX;
	*maximum = INT16_MIN;

	for (uint32_t y = 0; y < sensor->height; y++) {
		for (uint32_t x = 0; x < sensor->width; x++) {
			const int16_t pixel = frame->pixels[x * sensor->height + y];

			if (pixel < *minimum) {
				minimum_count++;
				if (minimum_count > 2U) {
					*minimum = pixel;
				}
			} else {
				minimum_count = 0U;
			}

			if (pixel > *maximum) {
				maximum_count++;
				if (maximum_count > 2U) {
					*maximum = pixel;
				}
			} else {
				maximum_count = 0U;
			}
		}
	}
}
#endif

#if defined(CONFIG_VIDEO_HM_HTPA_AUTOSCALE_HISTOGRAM)
static uint32_t htpa_histogram_bin(int16_t sample, int16_t minimum, uint32_t range)
{
	uint32_t offset = (uint32_t)((int32_t)sample - minimum);

	return (offset * (HTPA_HISTOGRAM_BIN_COUNT - 1U)) / range;
}

static void htpa_histogram_min_max(const struct hm_htpa_sensor_config *sensor,
				   struct hm_htpa_data *data, const struct hm_htpa_frame *frame,
				   int32_t *lower, int32_t *upper)
{
	const uint32_t pixel_count = sensor->width * sensor->height;
	const uint32_t clip_count = pixel_count * HTPA_AUTOSCALE_CLIP_PERCENT / 100U;
	int16_t minimum = INT16_MAX;
	int16_t maximum = INT16_MIN;
	int16_t retained_minimum = INT16_MAX;
	int16_t retained_maximum = INT16_MIN;
	uint32_t cumulative;
	uint32_t lower_bin;
	uint32_t upper_bin;
	uint32_t range;

	for (uint32_t y = 0; y < sensor->height; y++) {
		for (uint32_t x = 0; x < sensor->width; x++) {
			minimum = MIN(minimum, frame->pixels[x * sensor->height + y]);
			maximum = MAX(maximum, frame->pixels[x * sensor->height + y]);
		}
	}

	*lower = minimum;
	*upper = maximum;
	if (minimum == maximum) {
		return;
	}

	range = (uint32_t)((int32_t)maximum - minimum);
	memset(data->proc.histogram, 0, sizeof(data->proc.histogram));

	for (uint32_t y = 0; y < sensor->height; y++) {
		for (uint32_t x = 0; x < sensor->width; x++) {
			uint32_t bin = htpa_histogram_bin(frame->pixels[x * sensor->height + y],
							  minimum, range);

			data->proc.histogram[bin]++;
		}
	}

	cumulative = 0U;
	for (lower_bin = 0U; lower_bin < HTPA_HISTOGRAM_BIN_COUNT; lower_bin++) {
		cumulative += data->proc.histogram[lower_bin];
		if (cumulative > clip_count) {
			break;
		}
	}

	cumulative = 0U;
	for (upper_bin = HTPA_HISTOGRAM_BIN_COUNT - 1U; upper_bin > 0U; upper_bin--) {
		cumulative += data->proc.histogram[upper_bin];
		if (cumulative > clip_count) {
			break;
		}
	}

	for (uint32_t y = 0; y < sensor->height; y++) {
		for (uint32_t x = 0; x < sensor->width; x++) {
			int16_t sample = frame->pixels[x * sensor->height + y];
			uint32_t bin = htpa_histogram_bin(sample, minimum, range);

			if ((bin >= lower_bin) && (bin <= upper_bin)) {
				retained_minimum = MIN(retained_minimum, sample);
				retained_maximum = MAX(retained_maximum, sample);
			}
		}
	}

	if (retained_minimum < retained_maximum) {
		*lower = retained_minimum;
		*upper = retained_maximum;
	}
}
#endif

static uint16_t htpa_clamp_pixel(float pixel)
{
	if (pixel > UINT16_MAX) {
		return UINT16_MAX;
	}

	if (pixel < 0.0f) {
		return 0U;
	}

	return (uint16_t)pixel;
}

static void htpa_scale_frame(const struct hm_htpa_sensor_config *sensor, struct hm_htpa_data *data,
			     const struct hm_htpa_frame *frame, struct video_buffer *vbuf)
{
	uint16_t *output = (uint16_t *)vbuf->buffer;
	int32_t minimum;
	int32_t maximum;
	float offset;
	float scale;

#if defined(CONFIG_VIDEO_HM_HTPA_AUTOSCALE_HISTOGRAM)
	htpa_histogram_min_max(sensor, data, frame, &minimum, &maximum);
#else
	ARG_UNUSED(data);
	htpa_legacy_min_max(sensor, frame, &minimum, &maximum);
#endif

	offset = minimum;
	scale = maximum > minimum ? (float)UINT16_MAX / (float)(maximum - minimum) : 0.0f;

	for (uint32_t y = 0; y < sensor->height; y++) {
		for (uint32_t x = 0; x < sensor->width; x++) {
			float pixel = frame->pixels[x * sensor->height + y];

			pixel = (pixel - offset) * scale;
			output[y * sensor->width + x] = htpa_clamp_pixel(pixel);
		}
	}

	vbuf->bytesused = sensor->width * sensor->height * sizeof(*output);
}

int htpa_consume_frame(const struct device *dev, struct video_buffer *vbuf)
{
	const struct hm_htpa_config *cfg = dev->config;
	struct hm_htpa_data *data = dev->data;
	struct hm_htpa_frame *frame;
	int64_t deadline;
	int ret;

	deadline = k_uptime_get() + HTPA_FRAME_TIMEOUT_MS(cfg->sensor->block_count);
	do {
		if (atomic_get(&data->in_flight_canceled)) {
			return -ECANCELED;
		}

		frame = k_fifo_get(&data->grab.frame_ready_queue, K_MSEC(10));
	} while (frame == NULL && k_uptime_get() < deadline);

	if (frame == NULL) {
		return -ETIMEDOUT;
	}
	if (atomic_get(&data->in_flight_canceled)) {
		k_fifo_put(&data->grab.frame_free_queue, frame);
		return -ECANCELED;
	}

	ret = frame->result;
	if (ret == 0) {
		htpa_scale_frame(cfg->sensor, data, frame, vbuf);
		vbuf->timestamp = k_uptime_get_32();
	}

	k_fifo_put(&data->grab.frame_free_queue, frame);

	return ret;
}
