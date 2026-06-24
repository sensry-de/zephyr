//
// Created by tswaehn on 7/13/26.
//

#ifndef VIDEO_CAPTURE_HTPA_COMMON_H
#define VIDEO_CAPTURE_HTPA_COMMON_H


#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/video.h>
#include <zephyr/nvmem.h>
#include <zephyr/kernel.h>


#if defined(CONFIG_DT_HAS_HEIMANN_HTPA_160X120_ENABLED)
#include "HTPA160x120/hm_const.h"
#elif defined(CONFIG_DT_HAS_HEIMANN_HTPA_128X128_ENABLED)
#include "HTPA128x128/hm_const.h"
#else
error "missing sensor definitions"
#endif

#include "htpa-calib.h"

struct hm_htpa_config
{
	const struct video_format_cap * hm_htpa_caps;
	const struct device *calibration_flash;
};

#define HTPA_FRAME_QUEUE_SIZE 2

struct hm_htpa_frame {
	void *fifo_reserved;
	int16_t __aligned(4) pixels[PIXEL_PER_ROW][PIXEL_PER_COLUMN];
};

struct hm_htpa_data
{
	struct spi_dt_spec spi;

	struct video_format fmt;
	struct k_fifo framebuffer_take_queue;
	struct k_fifo framebuffer_release_queue;
	struct k_thread worker_thread;
	K_KERNEL_STACK_MEMBER(worker_stack, CONFIG_VIDEO_HM_HTPA_WORKER_STACK_SIZE);
	struct k_sem worker_sem;
	bool streaming;
	struct k_fifo frame_free_queue;
	struct k_fifo frame_ready_queue;
	struct k_thread grab_thread;
	K_KERNEL_STACK_MEMBER(grab_stack, CONFIG_VIDEO_HM_HTPA_GRAB_STACK_SIZE);
	struct hm_htpa_frame frames[HTPA_FRAME_QUEUE_SIZE];

	int communication_error;
	uint32_t communication_error_count;

	heimann_calibration_t calib;

	struct {
		uint32_t acquisition_time;

		/* raw sensor data */
		uint8_t __aligned(4) raw_top[NUMBER_OF_BLOCKS][BLOCK_LENGTH] ;
		uint8_t __aligned(4) raw_bottom[NUMBER_OF_BLOCKS][BLOCK_LENGTH];
		uint8_t __aligned(4) el_top_offsets[BLOCK_LENGTH];
		uint8_t __aligned(4) el_bottom_offsets[BLOCK_LENGTH];

	} grab;
};

int hm_htpa_init(const struct device *dev);
extern DEVICE_API(video, hm_htpa_api);

#endif //VIDEO_CAPTURE_HTPA_COMMON_H
