/*
 * Copyright (c) 2026 sensry.io
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>

#include <zephyr/sys/util.h>

#include "video_htpa_sensor.h"
#include "video_htpa_proc.h"

#define HTPA_AUTOSCALE_CLIP_PERCENT 1U

#if defined(CONFIG_VIDEO_HM_HTPA_AUTOSCALE_LEGACY)
static void htpa_legacy_min_max(const struct htpa_sensor_config *sensor,
				const struct video_buffer *frame, int32_t *minimum,
				int32_t *maximum)
{
	const int16_t *pixels = (const int16_t *)frame->buffer;
	uint32_t minimum_count = 0U;
	uint32_t maximum_count = 0U;

	*minimum = INT16_MAX;
	*maximum = INT16_MIN;

	for (uint32_t y = 0; y < sensor->height; y++) {
		for (uint32_t x = 0; x < sensor->width; x++) {
			const int16_t pixel = pixels[x * sensor->height + y];

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

static void htpa_histogram_min_max(const struct htpa_sensor_config *sensor, struct htpa_data *data,
				   const struct video_buffer *frame, int32_t *lower, int32_t *upper)
{
	const uint32_t pixel_count = sensor->width * sensor->height;
	const uint32_t clip_count = pixel_count * HTPA_AUTOSCALE_CLIP_PERCENT / 100U;
	const int16_t *pixels = (const int16_t *)frame->buffer;
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
			minimum = MIN(minimum, pixels[x * sensor->height + y]);
			maximum = MAX(maximum, pixels[x * sensor->height + y]);
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
			uint32_t bin =
				htpa_histogram_bin(pixels[x * sensor->height + y], minimum, range);

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
			int16_t sample = pixels[x * sensor->height + y];
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

void htpa_process_frame(const struct htpa_sensor_config *sensor, struct htpa_data *data,
			const struct video_buffer *frame, struct video_buffer *vbuf)
{
	const int16_t *pixels = (const int16_t *)frame->buffer;
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
			float pixel = pixels[x * sensor->height + y];

			pixel = (pixel - offset) * scale;
			output[y * sensor->width + x] = htpa_clamp_pixel(pixel);
		}
	}

	vbuf->bytesused = sensor->width * sensor->height * sizeof(*output);
}
