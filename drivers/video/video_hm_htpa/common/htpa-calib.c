/*
 * Copyright (c) 2026 sensry.io
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(htpa_calib, CONFIG_VIDEO_LOG_LEVEL);

#include <zephyr/device.h>

#include "../htpa-common.h"
#include "htpa-calib.h"

#include "zephyr/drivers/flash.h"

#define read_EEPROM(reg) read_flash_bytewise(dev, reg)

static uint8_t read_flash_bytewise(const struct device *dev, unsigned int address)
{
	const struct hm_htpa_config *cfg = dev->config;
	struct hm_htpa_data *data = dev->data;
	const struct device *flash_dev = cfg->calibration_flash;
	uint8_t rx_byte;
	int ret;

	ret = flash_read(flash_dev, address, &rx_byte, 1);
	if (ret != 0) {
		data->communication_error = ret;
		data->communication_error_count++;
		return 0x0;
	}

	return rx_byte;
}

/*
 * Read a small subset of the sensor calibration data, which is typically much larger.
 * This will be extended as more processing is added.
 */
int htpa_read_calibration(const struct device *dev, heimann_calibration_t *calib)
{
	const struct hm_htpa_config *cfg = dev->config;
	const struct hm_htpa_sensor_config *sensor = cfg->sensor;
	struct hm_htpa_data *data = dev->data;

	if (!device_is_ready(cfg->calibration_flash)) {
		LOG_ERR("calibration flash is not ready");
		return -ENODEV;
	}

	calib->id = (uint32_t)read_EEPROM(sensor->e_id[3]) << 24 |
		    (uint32_t)read_EEPROM(sensor->e_id[2]) << 16 |
		    (uint32_t)read_EEPROM(sensor->e_id[1]) << 8 |
		    (uint32_t)read_EEPROM(sensor->e_id[0]);
	calib->mbit_calib = read_EEPROM(sensor->e_mbit_calib);
	calib->bias_calib = read_EEPROM(sensor->e_bias_calib);
	calib->clk_calib = read_EEPROM(sensor->e_clk_calib);
	calib->bpa_calib = read_EEPROM(sensor->e_bpa_calib);

	if ((data->calib.id == 0) || (data->calib.id == 0xffffffffU)) {
		LOG_ERR("Invalid calibration data id");
		return -EINVAL;
	}

	return 0;
}
