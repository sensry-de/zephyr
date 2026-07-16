/*
 * Copyright (c) 2026 sensry.io
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_VIDEO_VIDEO_HM_HTPA_COMMON_HTPA_SENS_H_
#define ZEPHYR_DRIVERS_VIDEO_VIDEO_HM_HTPA_COMMON_HTPA_SENS_H_

#include <zephyr/device.h>

#define HTPA_CONVERSION_TIMEOUT_MS 100U
#define HTPA_FRAME_TIMEOUT_MS(block_count)                                                         \
	(((block_count) + 1U) * HTPA_CONVERSION_TIMEOUT_MS * 2U + MSEC_PER_SEC)

int htpa_start_sensor_acquisition(const struct device *dev);

#endif /* ZEPHYR_DRIVERS_VIDEO_VIDEO_HM_HTPA_COMMON_HTPA_SENS_H_ */
