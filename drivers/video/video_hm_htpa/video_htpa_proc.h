/*
 * Copyright (c) 2026 sensry.io
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_VIDEO_VIDEO_HM_HTPA_VIDEO_HTPA_PROC_H_
#define ZEPHYR_DRIVERS_VIDEO_VIDEO_HM_HTPA_VIDEO_HTPA_PROC_H_

#define HTPA_HISTOGRAM_BIN_COUNT 256U

struct htpa_data;
struct htpa_sensor_config;
struct video_buffer;

/**
 * @brief Process a raw sensor frame into an output video buffer.
 *
 * @param sensor HTPA sensor configuration.
 * @param data HTPA runtime data.
 * @param frame Raw sensor frame.
 * @param vbuf Output video buffer.
 */
void htpa_process_frame(const struct htpa_sensor_config *sensor, struct htpa_data *data,
			const struct video_buffer *frame, struct video_buffer *vbuf);

#endif /* ZEPHYR_DRIVERS_VIDEO_VIDEO_HM_HTPA_VIDEO_HTPA_PROC_H_ */
