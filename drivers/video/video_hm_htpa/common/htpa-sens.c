/*
 * Copyright (c) 2026 sensry.io
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(htpa_sens, CONFIG_VIDEO_LOG_LEVEL);

#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>
#include <zephyr/toolchain.h>
#include <zephyr/drivers/spi.h>

#include "../htpa-common.h"
#include "htpa-sens.h"

/*
 * Configuration register (write only):
 *
 * |  7  |  6  |  5  |  4  |   3   |    2     |   1   |    0   |
 * | RESERVED  |   Block   | Start | VDD_MEAS | BLIND | WAKEUP |
 */
#define CONFIGURATION_REGISTER 0x01

/*
 * Status register (read only):
 *
 * |  7  |  6  |  5  |  4  |   3   |    2     |   1   |   0   |
 * | RESERVED  |   Block   | RESERVED | VDD_MEAS | BLIND | EOC |
 */
#define STATUS_REGISTER 0x02

/*
 * Trim register 1 (write only):
 *
 * |  7  |  6  |  5  |  4  |  3  |  2  |  1  |  0  |
 * | RESERVED  |  REF_CAL  |         MBIT          |
 */
#define TRIM_REGISTER1 0x03

/*
 * Trim register 2 (write only):
 *
 * |  7  |  6  |  5  |  4  |  3  |  2  |  1  |  0  |
 * |    RESERVED     |       BIAS TRIM TOP         |
 */
#define TRIM_REGISTER2 0x04

/*
 * Trim register 3 (write only):
 *
 * |  7  |  6  |  5  |  4  |  3  |  2  |  1  |  0  |
 * |    RESERVED     |       BIAS TRIM BOT         |
 */
#define TRIM_REGISTER3 0x05

/*
 * Trim register 4 (write only):
 *
 * |  7  |  6  |  5  |  4  |  3  |  2  |  1  |  0  |
 * |    RESERVED     |             CLK TRIM        |
 */
#define TRIM_REGISTER4 0x06

/*
 * Trim register 5 (write only):
 *
 * |  7  |  6  |  5  |  4  |  3  |  2  |  1  |  0  |
 * |    RESERVED     |        BPA TRIM TOP         |
 */
#define TRIM_REGISTER5 0x07

/*
 * Trim register 6 (write only):
 *
 * |  7  |  6  |  5  |  4  |  3  |  2  |  1  |  0  |
 * |    RESERVED     |       BPA TRIM BOT          |
 */
#define TRIM_REGISTER6 0x08

/*
 * Trim register 7 (write only):
 *
 * |  7  |  6  |  5  |  4  |  3  |  2  |  1  |  0  |
 * |       PU SDA TRIM     |       PU SDA TRIM     |
 */
#define TRIM_REGISTER7 0x09

/* Select the top-half sensor data for reading or writing. */
#define TOP_HALF_SENSOR_DATA_REGISTER 0x0A

/* Select the bottom-half sensor data for reading or writing. */
#define BOTTOM_HALF_SENSOR_DATA_REGISTER 0x0B

static int htpa_grab_init_mem(const struct device *dev)
{
	struct hm_htpa_data *data = dev->data;

	k_fifo_init(&data->grab.frame_free_queue);
	k_fifo_init(&data->grab.frame_ready_queue);
	for (size_t i = 0; i < ARRAY_SIZE(data->grab.frames); i++) {
		k_fifo_put(&data->grab.frame_free_queue, &data->grab.frames[i]);
	}
	return 0;
}

static int htpa_sens_has_errors_active(const struct device *dev)
{
	struct hm_htpa_data *data = dev->data;

	if ((data->communication_error_count > 0) || (data->communication_error != 0)) {
		LOG_ERR("Communication error(s): %d during sensor readout",
			data->communication_error);
		data->communication_error_count = 0;
		data->communication_error = 0;
		return -EIO;
	}

	return 0;
}

static int htpa_sens_write_reg(const struct device *dev, uint8_t reg, uint8_t value)
{
	struct hm_htpa_data *data = dev->data;

	/* Transmit the register address and value. */
	struct spi_buf tx_buf[2];

	tx_buf[0].buf = &reg;
	tx_buf[0].len = 1;

	tx_buf[1].buf = &value;
	tx_buf[1].len = 1;

	const struct spi_buf_set tx = {
		.buffers = (struct spi_buf *)&tx_buf,
		.count = ARRAY_SIZE(tx_buf),
	};

	int ret = spi_transceive_dt(&data->spi, &tx, NULL);

	if (ret) {
		data->communication_error = ret;
		LOG_ERR("%s, write sensor reg: %u, failed: %d", data->spi.bus->name, reg, ret);
		return -EIO;
	}

	return 0;
}

static int htpa_grab_select_sensor_block(const struct device *dev, uint32_t block,
				    uint16_t block_count)
{
	uint8_t configuration = 0x0bU;

	if (block < block_count) {
		configuration = (uint8_t)(0x09U | (block << 4));
	}

	return htpa_sens_write_reg(dev, CONFIGURATION_REGISTER, configuration);
}

static int htpa_sens_read(const struct device *dev, uint8_t reg, uint8_t *rx_buffer,
			    size_t rx_len)
{
	struct hm_htpa_data *data = dev->data;

	/* The register address selects the sensor data to read. */
	uint8_t tx_buffer[1] = {reg};
	uint32_t tx_len = 1;

	/* Transmit the register address. */
	struct spi_buf tx_buf[1];

	tx_buf[0].buf = tx_buffer;
	tx_buf[0].len = tx_len;

	const struct spi_buf_set tx = {.buffers = (struct spi_buf *)&tx_buf, .count = 1};

	/* Receive the sensor data after the address phase. */
	struct spi_buf rx_buf[2];

	rx_buf[0].buf = NULL;
	rx_buf[0].len = 1;

	rx_buf[1].buf = rx_buffer;
	rx_buf[1].len = rx_len;

	const struct spi_buf_set rx = {
		.buffers = (struct spi_buf *)&rx_buf,
		.count = ARRAY_SIZE(rx_buf),
	};

	int ret = spi_transceive_dt(&data->spi, &tx, &rx);

	if (ret) {
		data->communication_error = ret;
		LOG_ERR("%s, read sensor reg: %u, failed: %d", data->spi.bus->name, reg, ret);
		return -EIO;
	}

	return 0;
}

static int htpa_sens_read_reg(const struct device *dev, uint8_t reg, uint8_t *value)
{
	if (0 != htpa_sens_read(dev, reg, value, 1)) {
		return -EIO;
	}
	return 0;
}

static int htpa_grab_image(const struct device *dev)
{
	const struct hm_htpa_config *cfg = dev->config;
	const struct hm_htpa_sensor_config *sensor = cfg->sensor;
	struct hm_htpa_data *data = dev->data;

	uint8_t status = 0x0;
	uint8_t *top_block = 0;
	uint8_t *bottom_block = 0;
	int ret;

	/* Read one additional block containing the electrical offsets. */
	for (uint32_t b = 0; b < (sensor->block_count + 1); b++) {
		ret = htpa_grab_select_sensor_block(dev, b, sensor->block_count);
		if (ret != 0) {
			return ret;
		}

		/* Adjust the acquisition time in 100 us steps to reduce polling. */
		uint32_t waiting_loops = 0;

		k_sleep(K_USEC(data->grab.acquisition_time));
		status = 0U;

		int64_t deadline = k_uptime_get() + HTPA_CONVERSION_TIMEOUT_MS;

		while ((status & 0x1U) == 0U) {
			/* Wait for the conversion to complete. */
			ret = htpa_sens_read_reg(dev, STATUS_REGISTER, &status);
			if (ret != 0) {
				return ret;
			}
			if ((status & 0x1U) != 0U) {
				continue;
			}
			if (k_uptime_get() >= deadline) {
				LOG_ERR("Sensor conversion timed out on block %u", b);
				return -ETIMEDOUT;
			}
			waiting_loops++;
			k_sleep(K_MSEC(1));
		}

		if (waiting_loops > 0) {
			data->grab.acquisition_time =
				MIN(data->grab.acquisition_time + 100U,
				    HTPA_CONVERSION_TIMEOUT_MS * USEC_PER_MSEC);
			LOG_DBG("Waiting for conversion ready: %d loops, new acquisition time: "
				"%d us",
				waiting_loops, data->grab.acquisition_time);
		}

		if (b == sensor->block_count) {
			/* Store the electrical offsets separately from the image data. */
			top_block = data->grab.el_top_offsets;
			bottom_block = data->grab.el_bottom_offsets;
		} else {
			/* Store the exposed-array pixel data. */
			top_block = &data->grab.raw_top[b * sensor->block_length];
			bottom_block = &data->grab.raw_bottom[b * sensor->block_length];
		}

		ret = htpa_sens_read(dev, TOP_HALF_SENSOR_DATA_REGISTER, top_block,
				       sensor->block_length);
		if (ret != 0) {
			return ret;
		}

		ret = htpa_sens_read(dev, BOTTOM_HALF_SENSOR_DATA_REGISTER, bottom_block,
				       sensor->block_length);
		if (ret != 0) {
			return ret;
		}
	}

	return 0;
}

static int htpa_sort_pixels(const struct device *dev, struct hm_htpa_frame *frame)
{
	const struct hm_htpa_config *cfg = dev->config;
	const struct hm_htpa_sensor_config *sensor = cfg->sensor;
	struct hm_htpa_data *data = dev->data;

	uint32_t x = 0;
	uint32_t y = 0;
	const uint8_t *block;
	const int16_t *el_offsets;

	/* Convert electrical offsets once; each offset is reused for every block. */
	for (uint32_t i = 0; i < sensor->pixels_per_block; i++) {
		const size_t offset = sensor->data_offset + i * sizeof(int16_t);
		int16_t el_offset;

		el_offset = (int16_t)sys_get_be16(&data->grab.el_top_offsets[offset]);
		UNALIGNED_PUT(el_offset, (int16_t *)&data->grab.el_top_offsets[offset]);

		el_offset = (int16_t)sys_get_be16(&data->grab.el_bottom_offsets[offset]);
		UNALIGNED_PUT(el_offset, (int16_t *)&data->grab.el_bottom_offsets[offset]);
	}

	/* Convert the top-half data blocks to pixels in forward order. */
	for (uint32_t b = 0; b < sensor->block_count; b++) {
		block = &data->grab.raw_top[b * sensor->block_length + sensor->data_offset];
		el_offsets = (const int16_t *)&data->grab.el_top_offsets[sensor->data_offset];

		for (uint32_t i = 0; i < sensor->pixels_per_block; i++) {
			const size_t offset = i * sizeof(int16_t);
			const int16_t px = (int16_t)sys_get_be16(&block[offset]);
			const int16_t el_offset = UNALIGNED_GET(&el_offsets[i]);

			frame->pixels[x * sensor->height + y] = (int16_t)(px - el_offset);

			x++;
			if (x >= sensor->width) {
				x = 0;
				y++;
			}
		}
	}

	/* Convert the bottom-half data blocks to pixels in reverse order. */
	y = sensor->height - 1;
	for (uint32_t b = 0; b < sensor->block_count; b++) {
		block = &data->grab.raw_bottom[b * sensor->block_length + sensor->data_offset];
		el_offsets = (const int16_t *)&data->grab.el_bottom_offsets[sensor->data_offset];

		for (uint32_t i = 0; i < sensor->pixels_per_block; i++) {
			const size_t offset = i * sizeof(int16_t);
			const int16_t px = (int16_t)sys_get_be16(&block[offset]);
			const int16_t el_offset = UNALIGNED_GET(&el_offsets[i]);

			frame->pixels[x * sensor->height + y] = (int16_t)(px - el_offset);

			x++;
			if (x >= sensor->width) {
				x = 0;
				y--;
			}
		}
	}

	return 0;
}

static void htpa_grab_thread(void *p1, void *p2, void *p3)
{
	const struct device *dev = p1;
	struct hm_htpa_data *data = dev->data;
	struct hm_htpa_frame *frame;
	int ret;

	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (true) {
		frame = k_fifo_get(&data->grab.frame_free_queue, K_FOREVER);
		ret = htpa_grab_image(dev);

		if (htpa_sens_has_errors_active(dev) != 0 && ret == 0) {
			ret = -EIO;
		}

		if (ret == 0) {
			ret = htpa_sort_pixels(dev, frame);
		}

		frame->result = ret;
		k_fifo_put(&data->grab.frame_ready_queue, frame);
	}
}

static int htpa_sens_weakup(const struct device *dev)
{
	struct hm_htpa_data *data = dev->data;

	if (0 != htpa_sens_write_reg(dev, CONFIGURATION_REGISTER, 0x01)) {
		LOG_ERR("Failed to wakeup sensor %s", dev->name);
		return -EINVAL;
	}

	uint8_t status;
	uint32_t retry_counter = 100;

	do {
		int ret = htpa_sens_read_reg(dev, STATUS_REGISTER, &status);

		if (ret != 0) {
			return ret;
		}
		k_sleep(K_MSEC(1));
		retry_counter--;
		if (0 == retry_counter) {
			LOG_ERR("Failed to wakeup sensor %s", dev->name);
			return -EINVAL;
		}
	} while (!(status & 0x01));

	if (0 != htpa_sens_write_reg(dev, TRIM_REGISTER1, data->calib.mbit_calib)) {
		LOG_ERR("Failed to write TRIM_REGISTER1 %s", dev->name);
		return -EINVAL;
	}

	if (0 != htpa_sens_write_reg(dev, TRIM_REGISTER2, data->calib.bias_calib)) {
		LOG_ERR("Failed to write TRIM_REGISTER2 %s", dev->name);
		return -EINVAL;
	}

	if (0 != htpa_sens_write_reg(dev, TRIM_REGISTER3, data->calib.bpa_calib)) {
		LOG_ERR("Failed to write TRIM_REGISTER3 %s", dev->name);
		return -EINVAL;
	}

	if (0 != htpa_sens_write_reg(dev, TRIM_REGISTER4, data->calib.clk_calib)) {
		LOG_ERR("Failed to write TRIM_REGISTER4 %s", dev->name);
		return -EINVAL;
	}

	return 0;
}

int htpa_grab_start_acquisition(const struct device *dev)
{
	struct hm_htpa_data *data = dev->data;

	if (0 != htpa_grab_init_mem(dev)) {
		LOG_ERR("Failed to initialize memory %s", dev->name);
		return -EINVAL;
	}

	if (0 != htpa_sens_weakup(dev)) {
		LOG_ERR("Failed to wakeup sensor %s", dev->name);
		return -EINVAL;
	}

	if (htpa_sens_has_errors_active(dev) != 0) {
		return -EIO;
	}

	k_thread_create(&data->grab.thread, data->grab.stack,
			K_THREAD_STACK_SIZEOF(data->grab.stack), htpa_grab_thread, (void *)dev,
			NULL, NULL, CONFIG_VIDEO_HM_HTPA_GRAB_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&data->grab.thread, "htpa_grab");

	return 0;
}
