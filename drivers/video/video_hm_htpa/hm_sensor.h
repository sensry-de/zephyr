//
// Created by tswaehn on 1/13/25.
//

#ifndef ZEPHYR_MINIMAL_APP_SRC_HEIMANN_SENSOR_H
#define ZEPHYR_MINIMAL_APP_SRC_HEIMANN_SENSOR_H

#if defined(CONFIG_HEIMANN_SENSOR_HTPA120X84)
	#include "HTPA120x84/hm_const.h"
#elif defined(CONFIG_HEIMANN_SENSOR_HTPA160X120)
	#include "HTPA160x120/hm_const.h"
	#include "HTPA160x120/hm_img.h"
#else
	#error "No Heimann sensor type selected"
#endif


//
#define FRAME_WIDTH  (PIXEL_PER_ROW)
#define FRAME_HEIGHT (PIXEL_PER_COLUMN)

#define FRAME_BLOCK_COUNT (NUMBER_OF_BLOCKS)
#define FRAME_BLOCK_LENGTH (BLOCK_LENGTH)
#define FRAME_DATA_OFFS	(DATA_POS)

#define DATA_SIZE (FRAME_WIDTH * FRAME_HEIGHT)
#define FRAME_SIZE (DATA_SIZE * 3)
#define FRAME_YUYV_SIZE (DATA_SIZE * 2)

#define RAW_FRAME_BLOCKS     (2 * NUMBER_OF_BLOCKS + 2)
#define RAW_FRAME_SIZE       (RAW_FRAME_BLOCKS * FRAME_BLOCK_LENGTH)

int32_t hm_init_sensor();
bool is_sensor_initialized();
void hm_grab_image(uint8_t frame[RAW_FRAME_BLOCKS][FRAME_BLOCK_LENGTH]);


#endif // ZEPHYR_MINIMAL_APP_SRC_HEIMANN_SENSOR_H
