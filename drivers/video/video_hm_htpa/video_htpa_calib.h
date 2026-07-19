/*
 * Copyright (c) 2026 sensry.io
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_VIDEO_VIDEO_HM_HTPA_VIDEO_HTPA_CALIB_H_
#define ZEPHYR_DRIVERS_VIDEO_VIDEO_HM_HTPA_VIDEO_HTPA_CALIB_H_

#include <stdint.h>

struct htpa_calib{
	/* EEPROM calibration data */
	uint8_t mbit_calib;
	uint8_t bias_calib;
	uint8_t clk_calib;
	uint8_t bpa_calib;
	uint8_t pu_calib;
	uint8_t mbit_user;
	uint8_t bias_user;
	uint8_t clk_user;
	uint8_t bpa_user;
	uint8_t pu_user;
	uint32_t id;
};

int htpa_read_calibration(const struct device *dev, struct htpa_calib *calib);

#endif /* ZEPHYR_DRIVERS_VIDEO_VIDEO_HM_HTPA_VIDEO_HTPA_CALIB_H_ */
