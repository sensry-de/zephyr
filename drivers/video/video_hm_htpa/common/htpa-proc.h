/*
 * Copyright (c) 2026 sensry.io
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_VIDEO_VIDEO_HM_HTPA_COMMON_HTPA_PROC_H_
#define ZEPHYR_DRIVERS_VIDEO_VIDEO_HM_HTPA_COMMON_HTPA_PROC_H_

#define HTPA_HISTOGRAM_BIN_COUNT 256U

struct device;
struct video_buffer;

/**
 * @brief Consume a frame from the internal multi-buffer frame queue.
 *
 * Wait for a ready frame, process it into the output buffer, and return the
 * consumed frame to the internal free-frame queue.
 *
 * @param dev HTPA device instance.
 * @param vbuf Output video buffer.
 *
 * @retval 0 Frame consumed successfully.
 */
int htpa_consume_frame(const struct device *dev, struct video_buffer *vbuf);

#endif /* ZEPHYR_DRIVERS_VIDEO_VIDEO_HM_HTPA_COMMON_HTPA_PROC_H_ */
