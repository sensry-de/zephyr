//
// Created by tswaehn on 7/15/26.
//

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(htpa_sens, CONFIG_VIDEO_LOG_LEVEL);

#include <zephyr/sys/byteorder.h>
#include <zephyr/drivers/spi.h>

#include "htpa-common.h"
#include "htpa-sens.h"


// SENSOR REGISTER

#define CONFIGURATION_REGISTER 0x01 // configuration register (WRITE only)
// |  7  |  6  |  5  |  4  |   3   |    2     |   1   |    0   |
// |    RFU    |   Block   | Start | VDD_MEAS | BLIND | WAKEUP |
#define STATUS_REGISTER 0x02 // adress of status register (READ only)
// |  7  |  6  |  5  |  4  |   3   |    2     |   1   |    0   |
// |    RFU    |   Block   |  RFU  | VDD_MEAS | BLIND |   EOC  |
#define TRIM_REGISTER1 0x03 // adress for trim register 1 (WRITE only)
// |  7  |  6  |  5  |  4  |  3  |  2  |  1  |  0  |
// |    RFU    |  REF_CAL  |         MBIT          |
#define TRIM_REGISTER2 0x04 // adress for trim register 2 (WRITE only)
// |  7  |  6  |  5  |  4  |  3  |  2  |  1  |  0  |
// |       RFU       |       BIAS TRIM TOP         |
#define TRIM_REGISTER3 0x05 // adress for trim register 3 (WRITE only)
// |  7  |  6  |  5  |  4  |  3  |  2  |  1  |  0  |
// |       RFU       |       BIAS TRIM BOT         |
#define TRIM_REGISTER4 0x06 // adress for trim register 4 (WRITE only)
// |  7  |  6  |  5  |  4  |  3  |  2  |  1  |  0  |
// |       RFU |             CLK TRIM              |
#define TRIM_REGISTER5 0x07 // adress for trim register 5 (WRITE only)
// |  7  |  6  |  5  |  4  |  3  |  2  |  1  |  0  |
// |       RFU       |        BPA TRIM TOP         |
#define TRIM_REGISTER6 0x08 // adress for trim register 6 (WRITE only)
// |  7  |  6  |  5  |  4  |  3  |  2  |  1  |  0  |
// |       RFU       |       BPA TRIM BOT          |
#define TRIM_REGISTER7 0x09 // adress for trim register 7 (WRITE only)
// |  7  |  6  |  5  |  4  |  3  |  2  |  1  |  0  |
// |       PU SDA TRIM     |       PU SDA TRIM     |

#define TOP_HALF_SENSOR_DATA_REGISTER		0x0A
// set the read / write mode pointer to the top half sensor data register

#define BOTTOM_HALF_SENSOR_DATA_REGISTER	0x0B
// set the read / write mode pointer to the bottom half sensor data register


static int htpa_init_mem(const struct device *dev)
{
	struct hm_htpa_data *data = dev->data;

	k_fifo_init(&data->frame_free_queue);
	k_fifo_init(&data->frame_ready_queue);
	for (size_t i = 0; i < ARRAY_SIZE(data->frames); i++) {
		k_fifo_put(&data->frame_free_queue, &data->frames[i]);
	}
	return 0;
}

static int hm_htap_has_errors_active(const struct device *dev)
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

int htpa_write_sensor_reg(const struct device *dev, uint8_t reg, uint8_t value)
{
	const struct hm_htpa_config *cfg = dev->config;
	struct hm_htpa_data *data = dev->data;

	/* tx */
	struct spi_buf tx_buf[2];
	tx_buf[0].buf = &reg;
	tx_buf[0].len = 1;

	tx_buf[1].buf = &value;
	tx_buf[1].len = 1;

	const struct spi_buf_set tx = {
		.buffers = (struct spi_buf *) &tx_buf,
		.count = ARRAY_SIZE(tx_buf),
	};

	//chip_select_high();

	int ret = spi_transceive_dt(&data->spi, &tx, NULL);
	if (ret) {
		data->communication_error = ret;
		LOG_ERR("%s, write sensor reg: %u, failed: %d", data->spi.bus->name, reg, ret);
		return -EIO;
	}

	//chip_select_low();

	return 0;
}

int htpa_read_sensor(const struct device *dev, uint8_t reg, uint8_t *rx_buffer, size_t rx_len)
{
	const struct hm_htpa_config *cfg = dev->config;
	struct hm_htpa_data *data = dev->data;

	/* reg defines what to read */
	uint8_t tx_buffer[1] = {reg};
	uint32_t tx_len = 1;

	/* tx */
	struct spi_buf tx_buf[1];
	tx_buf[0].buf = tx_buffer;
	tx_buf[0].len = tx_len;

	const struct spi_buf_set tx = {.buffers = (struct spi_buf *)&tx_buf, .count = 1};

	/* rx */
	struct spi_buf rx_buf[2];
	rx_buf[0].buf = NULL;
	rx_buf[0].len = 1;

	rx_buf[1].buf = rx_buffer;
	rx_buf[1].len = rx_len;

	const struct spi_buf_set rx = {
		.buffers = (struct spi_buf *) &rx_buf,
		.count = ARRAY_SIZE(rx_buf),
	};

	//chip_select_high();

	int ret = spi_transceive_dt(&data->spi, &tx, &rx);
	if (ret) {
		data->communication_error = ret;
		LOG_ERR("%s, read sensor reg: %u, failed: %d", data->spi.bus->name, reg, ret);
		return -EIO;
	}

	//chip_select_low();

	return 0;
}

int htpa_read_sensor_reg(const struct device *dev, uint8_t reg, uint8_t *value)
{
	if (0 != htpa_read_sensor(dev, reg, value, 1)) {
		return -EIO;
	}
	return 0;
}

static int htpa_grab_image(const struct device *dev)
{
	struct hm_htpa_data *data = dev->data;

	uint8_t status = 0x0;
	uint8_t * top_block = 0;
	uint8_t * bottom_block = 0;

	/* we read +1 block for electrical offsets */
	for (uint32_t b = 0; b < (NUMBER_OF_BLOCKS + 1); b++) {

		if (b < NUMBER_OF_BLOCKS) {
			/* read image data block */
			htpa_write_sensor_reg(dev, CONFIGURATION_REGISTER, 0x9 | (b << 4));
		} else {
			/* read electronic offsets block */
			htpa_write_sensor_reg(dev, CONFIGURATION_REGISTER, 0x0b);
		}

		/* adjust acquisition time in 100 us steps to avoid waiting loops */
		uint32_t waiting_loops = 0;
		k_sleep(K_USEC(data->grab.acquisition_time));

		while (true) {
			/* wait for conversion ready */
			htpa_read_sensor_reg(dev, STATUS_REGISTER, &status);
			if ((status & 0x1) == 1) {
				/* acquisition ready */
				break;
			}
			waiting_loops++;
			k_sleep(K_MSEC(1));
		}

		if (waiting_loops > 0) {
			data->grab.acquisition_time += 100;
			LOG_WRN("Waiting for conversion ready: %d loops, new acquisition time: %d us", waiting_loops, data->grab.acquisition_time);
		}

		if (b == NUMBER_OF_BLOCKS) {
			/* electrical offsets */
			top_block = data->grab.el_top_offsets;
			bottom_block = data->grab.el_bottom_offsets;
		} else {
			/* read the pixel result of the exposed arrays */
			top_block = data->grab.raw_top[b];
			bottom_block= data->grab.raw_bottom[b];
		}

		htpa_read_sensor(dev, TOP_HALF_SENSOR_DATA_REGISTER, top_block, BLOCK_LENGTH);
		htpa_read_sensor(dev, BOTTOM_HALF_SENSOR_DATA_REGISTER, bottom_block, BLOCK_LENGTH);
	}

	if (0 != data->communication_error_count) {
		return -EIO;
	}

	return 0;
}

static int htpa_sort_pixels(const struct device *dev, struct hm_htpa_frame *frame)
{
	struct hm_htpa_data *data = dev->data;

	uint32_t x = 0;
	uint32_t y = 0;
	uint8_t * block;
	int16_t * el_offs;

	/* convert the electrical offsets from int16 msb to lsb order */
	for (uint32_t p = 0; p < PIXEL_PER_BLOCK; p++) {
		block = &data->grab.el_top_offsets[p*2 + DATA_POS];
		el_offs = (int16_t*) block;
		*el_offs = (int16_t) sys_get_be16(block);

		block = &data->grab.el_bottom_offsets[p*2 + DATA_POS];
		el_offs = (int16_t*) block;
		*el_offs = (int16_t) sys_get_be16(block);
	}

	/* convert each of the raw data blocks into a top half data pixel format (forward) */
	for (uint32_t b = 0; b < NUMBER_OF_BLOCKS; b++) {
		block = &data->grab.raw_top[b][DATA_POS];
		el_offs = (int16_t*) &data->grab.el_top_offsets[DATA_POS];

		for (uint32_t i = 0; i < PIXEL_PER_BLOCK; i++){
			const int16_t px = (int16_t) sys_get_be16(&block[i*2]);
			frame->pixels[x][y] = (int16_t)(px - el_offs[i]);

			x++;
			if (x >= PIXEL_PER_ROW) {
				x=0;
				y++;
			}
		}
	}

	/* convert each of the raw data blocks into a bottom half data pixel format (reverse) */
	y = PIXEL_PER_COLUMN - 1;
	for (uint32_t b = 0; b < NUMBER_OF_BLOCKS; b++) {
		block = &data->grab.raw_bottom[b][DATA_POS];
		el_offs = (int16_t*) &data->grab.el_bottom_offsets[DATA_POS];

		for (uint32_t i = 0; i < PIXEL_PER_BLOCK; i++){
			const int16_t px = (int16_t) sys_get_be16(&block[i*2]);
			frame->pixels[x][y] = (int16_t)(px - el_offs[i]);

			x++;
			if (x >= PIXEL_PER_ROW) {
				x=0;
				y--;
			}
		}
	}

	return 0;
}

static void min_max(const struct hm_htpa_frame *frame, int32_t *m_min, int32_t *m_max)
{
	uint32_t s0 = 0;
	uint32_t s1 = 0;

	*m_min = INT16_MAX;
	*m_max = INT16_MIN;

	/* */
	for (uint32_t y=0; y < PIXEL_PER_COLUMN; y++) {
		for (uint32_t x=0; x < PIXEL_PER_ROW; x++) {
			const int16_t *p = &frame->pixels[x][y];
			if (*p < *m_min) {
				s0++;
				if (s0>2) {
					*m_min = *p;
				}
			} else {
				s0= 0;
			}

			if (*p > *m_max) {
				s1++;
				if (s1>2) {
					*m_max = *p;
				}
			} else {
				s1=0;
			}
		}
	}
}

static inline uint16_t px_cap_uint16(const float * px)
{
	if (*px > UINT16_MAX) {
		return UINT16_MAX;
	}
	if (*px < 0) {
		return 0;
	}
	return (uint16_t)*px;
}

static void auto_scale_data(const struct hm_htpa_frame *frame, struct video_buffer *vbuf)
{
	uint16_t * v = (uint16_t*)vbuf->buffer;

	int32_t m_min;
	int32_t m_max;


	min_max(frame, &m_min, &m_max);

	float off = m_min;
	float scale = (float) (UINT16_MAX) / (float) (m_max - m_min);

	for (uint32_t y=0; y < PIXEL_PER_COLUMN; y++) {
		for (uint32_t x=0; x < PIXEL_PER_ROW; x++) {
			float px = frame->pixels[x][y];

			px -= off;
			px *= scale;

			v[y*PIXEL_PER_ROW + x] = px_cap_uint16(&px);
		}
	}

	vbuf->bytesused = vbuf->size;
}

int htpa_copy_frame(const struct device *dev, struct video_buffer *vbuf)
{
	struct hm_htpa_data *data = dev->data;
	struct hm_htpa_frame *frame;

	frame = k_fifo_get(&data->frame_ready_queue, K_FOREVER);

	auto_scale_data(frame, vbuf);

	k_fifo_put(&data->frame_free_queue, frame);

	vbuf->timestamp = k_cycle_get_32();
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
		frame = k_fifo_get(&data->frame_free_queue, K_FOREVER);
		ret = htpa_grab_image(dev);

		if ((hm_htap_has_errors_active(dev) != 0) || (ret != 0)) {
			k_fifo_put(&data->frame_free_queue, frame);
			continue;
		}

		htpa_sort_pixels(dev, frame);

		k_fifo_put(&data->frame_ready_queue, frame);
	}
}

static int htpa_weakup_sensor(const struct device *dev)
{
	struct hm_htpa_data *data = dev->data;

	if (0 != htpa_write_sensor_reg(dev, CONFIGURATION_REGISTER, 0x01)) {
		LOG_ERR("Failed to wakeup sensor %s", dev->name);
		return -EINVAL;
	}

	uint8_t status;
	uint32_t retry_counter = 100;
	do {
		htpa_read_sensor_reg(dev, STATUS_REGISTER, &status);
		k_sleep(K_MSEC(1));
		retry_counter--;
		if (0 == retry_counter) {
			LOG_ERR("Failed to wakeup sensor %s", dev->name);
			return -EINVAL;
		}
	} while (!(status & 0x01));

	if (0 != htpa_write_sensor_reg(dev, TRIM_REGISTER1, data->calib.mbit_calib)) {
		LOG_ERR("Failed to write TRIM_REGISTER1 %s", dev->name);
		return -EINVAL;
	}

	if (0 != htpa_write_sensor_reg(dev, TRIM_REGISTER2, data->calib.bias_calib)) {
		LOG_ERR("Failed to write TRIM_REGISTER2 %s", dev->name);
		return -EINVAL;
	}

	if (0 != htpa_write_sensor_reg(dev, TRIM_REGISTER3, data->calib.bpa_calib)) {
		LOG_ERR("Failed to write TRIM_REGISTER3 %s", dev->name);
		return -EINVAL;
	}

	if (0 != htpa_write_sensor_reg(dev, TRIM_REGISTER4, data->calib.clk_calib)) {
		LOG_ERR("Failed to write TRIM_REGISTER4 %s", dev->name);
		return -EINVAL;
	}

	return 0;
}

int htpa_init_sensor(const struct device *dev)
{
	struct hm_htpa_data *data = dev->data;

	if (0 != htpa_init_mem(dev)) {
		LOG_ERR("Failed to initialize memory %s", dev->name);
		return -EINVAL;
	}

	if (0 != htpa_weakup_sensor(dev)) {
		LOG_ERR("Failed to wakeup sensor %s", dev->name);
		return -EINVAL;
	}

	if (hm_htap_has_errors_active(dev) != 0) {
		return -EIO;
	}

	k_thread_create(&data->grab_thread, data->grab_stack,
			K_THREAD_STACK_SIZEOF(data->grab_stack), htpa_grab_thread, dev,
			NULL, NULL, CONFIG_VIDEO_HM_HTPA_GRAB_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&data->grab_thread, "htpa_grab");

	return 0;
}
