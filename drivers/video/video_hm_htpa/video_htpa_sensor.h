/*
 * Copyright (c) 2026 sensry.io
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_VIDEO_VIDEO_HM_HTPA_VIDEO_HTPA_SENSOR_H_
#define ZEPHYR_DRIVERS_VIDEO_VIDEO_HM_HTPA_VIDEO_HTPA_SENSOR_H_

#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/video.h>
#include <zephyr/kernel.h>

#include "video_htpa_proc.h"

struct htpa_calib {
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

struct htpa_sensor_config {
	uint16_t width;
	uint16_t height;
	uint16_t block_count;
	uint16_t pixels_per_block;
	uint16_t block_length;
	uint16_t data_offset;
	uint32_t e_id[4];
	uint32_t e_mbit_calib;
	uint32_t e_bias_calib;
	uint32_t e_clk_calib;
	uint32_t e_bpa_calib;
};

struct htpa_config {
	const struct video_format_cap *hm_htpa_caps;
	const struct device *calibration_flash;
	const struct htpa_sensor_config *sensor;
};

#define HTPA_FRAME_QUEUE_SIZE      2
#define HTPA_CONVERSION_TIMEOUT_MS 100U
#define HTPA_FRAME_TIMEOUT_MS(block_count)                                                         \
	(((block_count) + 1U) * HTPA_CONVERSION_TIMEOUT_MS * 2U + MSEC_PER_SEC)

struct htpa_grabbed_frame {
	void *fifo_reserved;
	int16_t *pixels;
	int result;
};

struct htpa_data {
	struct spi_dt_spec spi;

	struct video_format fmt;
	struct k_fifo framebuffer_take_queue;
	struct k_fifo framebuffer_release_queue;
	struct k_thread worker_thread;

	K_KERNEL_STACK_MEMBER(worker_stack, CONFIG_VIDEO_HM_HTPA_WORKER_STACK_SIZE);
	struct k_sem worker_sem;
	struct k_mutex lock;
	struct k_condvar state_changed;
	struct video_buffer *in_flight;
	bool streaming;
	bool flushing;
	bool canceling;
	atomic_t in_flight_canceled;

	int communication_error;
	uint32_t communication_error_count;

	struct htpa_calib calib;

	struct {
		uint32_t acquisition_time;
		struct k_thread thread;

		K_KERNEL_STACK_MEMBER(stack, CONFIG_VIDEO_HM_HTPA_GRAB_STACK_SIZE);
		struct htpa_grabbed_frame frames[HTPA_FRAME_QUEUE_SIZE];
		struct k_fifo frame_free_queue;
		struct k_fifo frame_ready_queue;

		/* raw sensor data */
		uint8_t *raw_top;
		uint8_t *raw_bottom;
		uint8_t *el_top_offsets;
		uint8_t *el_bottom_offsets;

	} grab;

#if defined(CONFIG_VIDEO_HM_HTPA_AUTOSCALE_HISTOGRAM)
	struct {
		uint16_t histogram[HTPA_HISTOGRAM_BIN_COUNT];
	} proc;
#endif
};

#endif /* ZEPHYR_DRIVERS_VIDEO_VIDEO_HM_HTPA_VIDEO_HTPA_SENSOR_H_ */
