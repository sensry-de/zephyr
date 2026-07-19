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
	uint32_t e_offs_pixcmin[4];     /* Minimum PixC (float) */
	uint32_t e_offs_pixcmax[4];     /* Maximum PixC (float) */
	uint32_t e_offs_gradscale;      /* PTAT compensation scale */
	uint32_t e_offs_tablenumber[2]; /* Table number (uint16) */
	uint32_t e_offs_epsilon;        /* Emissivity percent */
	uint32_t e_offs_id[4];          /* Sensor ID (uint32) */
	uint32_t e_offs_mbit_calib;     /* Trim register 1 calibration */
	uint32_t e_offs_bias_calib;     /* Trim registers 2/3 calibration */
	uint32_t e_offs_clk_calib;      /* Trim register 4 calibration */
	uint32_t e_offs_bpa_calib;      /* Trim registers 5/6 calibration */
	uint32_t e_offs_pu_calib;       /* Trim register 7 calibration */
	uint32_t e_offs_arraytype;      /* Sensor array type */
	uint32_t e_offs_vddth1[2];      /* HS VDD threshold 1 */
	uint32_t e_offs_vddth2[2];      /* HS VDD threshold 2 */
	uint32_t e_offs_ptatgr[4];      /* PTAT gradient (float) */
	uint32_t e_offs_ptatoff[4];     /* PTAT offset (float) */
	uint32_t e_offs_ptatth1[2];     /* HS PTAT threshold 1 */
	uint32_t e_offs_ptatth2[2];     /* HS PTAT threshold 2 */
	uint32_t e_offs_vddscgrad;      /* HS VDD scale gradient */
	uint32_t e_offs_vddscoff;       /* HS VDD scale offset */
	uint32_t e_offs_globaloff;      /* Global offset (int8) */
	uint32_t e_offs_globalgain[2];  /* Global gain (uint16) */
	uint32_t e_offs_mbit_user;      /* Trim register 1 user value */
	uint32_t e_offs_bias_user;      /* Trim registers 2/3 user value */
	uint32_t e_offs_clk_user;       /* Trim register 4 user value */
	uint32_t e_offs_bpa_user;       /* Trim registers 5/6 user value */
	uint32_t e_offs_pu_user;        /* Trim register 7 user value */
	uint32_t e_offs_nrofdefpix;     /* Defective pixel count */
	uint32_t e_offs_deadpixadr;     /* Dead pixel address table */
	uint32_t e_offs_deadpixmask;    /* Dead pixel mask table */
	uint32_t e_offs_vddcompgrad;    /* VDD compensation gradient */
	uint32_t e_offs_vddcompoff;     /* VDD compensation offset */
	uint32_t e_offs_thgrad;         /* Thermal gradient table */
	uint32_t e_offs_thoffset;       /* Thermal offset table */
	uint32_t e_offs_pij;            /* Pixel constants (PixC) */
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
		struct video_buffer frames[HTPA_FRAME_QUEUE_SIZE];
		int frame_results[HTPA_FRAME_QUEUE_SIZE];
		int16_t *frame_pixels;
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
