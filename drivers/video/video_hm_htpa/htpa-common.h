/*
 * Copyright (c) 2026 sensry.io
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_VIDEO_VIDEO_HTPA_SENSOR_HTPA_COMMON_H_
#define ZEPHYR_DRIVERS_VIDEO_VIDEO_HTPA_SENSOR_HTPA_COMMON_H_

#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/video.h>
#include <zephyr/nvmem.h>
#include <zephyr/kernel.h>

#include "htpa-calib.h"
#include "htpa-proc.h"

struct hm_htpa_sensor_config {
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

struct hm_htpa_config {
	const struct video_format_cap *hm_htpa_caps;
	const struct device *calibration_flash;
	const struct hm_htpa_sensor_config *sensor;
};

#define HTPA_FRAME_QUEUE_SIZE 2

struct hm_htpa_frame {
	void *fifo_reserved;
	int16_t *pixels;
	int result;
};

struct hm_htpa_data {
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

	heimann_calibration_t calib;

	struct {
		uint32_t acquisition_time;
		struct k_thread thread;

		K_KERNEL_STACK_MEMBER(stack, CONFIG_VIDEO_HM_HTPA_GRAB_STACK_SIZE);
		struct hm_htpa_frame frames[HTPA_FRAME_QUEUE_SIZE];
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

int hm_htpa_init(const struct device *dev);
extern const struct video_driver_api hm_htpa_api;

#endif /* ZEPHYR_DRIVERS_VIDEO_VIDEO_HTPA_SENSOR_HTPA_COMMON_H_ */
