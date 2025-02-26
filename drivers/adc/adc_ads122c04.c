/*
 * Copyright (c) 2026 sensry.io
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

// LOG_MODULE_REGISTER(ads122c04, CONFIG_ADC_LOG_LEVEL);
LOG_MODULE_REGISTER(ads122c04, LOG_LEVEL_INF);

#define ADC_CONTEXT_USES_KERNEL_TIMER
#include "adc_context.h"

#define ADS122C04_REF_INTERNAL	  0x815
#define ADS122C04_RESOLUTION_BITS 24

#define ADS122C04_REG_ADDR_BIT_OFFS   2
#define ADS122C04_REG_DATA_BYTE_COUNT 1
#define ADS122C04_REG_COUNT           4
#define ADS122C04_SAMPLE_BYTE_COUNT   3
#define ADS122C04_CHANNEL_COUNT       4

/* config0 */
#define ADS122C04_CFG0_MUX_OFFS         4
#define ADS122C04_CFG0_GAIN_OFFS        1
#define ADS122C04_CFG0_PGA_DISABLE_OFFS 0

#define ADS122C04_MUX_SINGLE(x) ((0x8 | (x)) << ADS122C04_CFG0_MUX_OFFS)
#define ADS122C04_GAIN(x)       (LOG2(x) << ADS122C04_CFG0_GAIN_OFFS)

/* config1 */
#define ADS122C04_CFG1_CONTINUOUS_OFFS 3

/* config2 */
#define ADS122C04_CFG2_DRDY_OFFS 7

enum ads122c04_cmd
{
	ADS122C04_CMD_RESET = 0x06,
	ADS122C04_CMD_START = 0x08,
	ADS122C04_CMD_POWER_DOWN = 0x02,
	ADS122C04_CMD_RDATA = 0x10,
	ADS122C04_CMD_RREG = 0x20,
	ADS122C04_CMD_WREG = 0x40,
};

enum ads122c04_reg
{
	ADS122C04_REG_CFG0 = 0x0,
	ADS122C04_REG_CFG1 = 0x1,
	ADS122C04_REG_CFG2 = 0x2,
	ADS122C04_REG_CFG3 = 0x3,
};

enum ads122c04_channels
{
	AIN0 = 0,
	AIN1 = 1,
	AIN2 = 2,
	AIN3 = 3,
	CHIP_TEMPERATURE = 4,
	VREF_P = 5,
	VREF_N = 6,
	GND = 7
};

struct ads122c04_profile
{
	/* reg 0 */
	uint8_t pga_bypass : 1;
	uint8_t gain : 3;
	uint8_t mux : 4;

	/* reg 1 */
	uint8_t temperature_sensor_mode : 1;
	uint8_t vref : 2;
	uint8_t conversion_mode : 1;
	uint8_t speed_mode : 1;
	uint8_t data_rate : 3;

	/* reg 2 */
	uint8_t idac : 3;
	uint8_t crc_enabled : 2;
	uint8_t data_count_enable : 1;

	/* reg 3 */
	uint8_t i2mux : 3;
	uint8_t i1mux : 3;

	/* expected conversion time, based on acquisition time */
	k_timeout_t ready_time;

	/* cache for config */
	uint8_t raw_regs[ADS122C04_REG_COUNT];
};

/* reg 0 shift offsets (for raw access) */
#define ADS122C04_REG0_PGA_BYPASS_SHIFT 0
#define ADS122C04_REG0_GAIN_SHIFT       1
#define ADS122C04_REG0_MUX_SHIFT        4

/* reg 1 shift offsets (for raw access) */
#define ADS122C04_REG1_TS_SHIFT        0
#define ADS122C04_REG1_VREF_SHIFT      1
#define ADS122C04_REG1_CM_SHIFT        3
#define ADS122C04_REG1_OP_MODE_SHIFT   4
#define ADS122C04_REG1_DATA_RATE_SHIFT 5

/* reg 2 shift offsets (for raw access) */
#define ADS122C04_REG2_IDAC_SHIFT 0
#define ADS122C04_REG2_BCS_SHIFT  3
#define ADS122C04_REG2_CRC_SHIFT  4
#define ADS122C04_REG2_DCNT_SHIFT 6
#define ADS122C04_REG2_DRDY_SHIFT 7

/* reg 3 shift offsets (for raw access) */
#define ADS122C04_REG3_I2MUX_SHIFT 2
#define ADS122C04_REG3_I1MUX_SHIFT 5

#define ADS122C04_FIELD_MASK(width) ((1U << (width)) - 1U)

/* current source */
#define ADS122C04_CURRENT_SOURCE_CURRENT_MSK   GENMASK(6, 4)
#define ADS122C04_CURRENT_SOURCE_MUX_MSK       GENMASK(3, 0)
#define ADS122C04_CURRENT_SOURCE_CURRENT_SHIFT 4

/* number of pre-definable adc profiles, cache in RAM */
#define ADS122C04_MAX_PROFILE_COUNT 8

struct ads122c04_config
{
	const struct i2c_dt_spec bus;
};

struct ads122c04_data
{
	const struct device *dev;
	struct adc_context ctx;

	/* trigger acquisition */
	struct k_sem acquisition_sem;

	/* result buffer and advancing ptr for write-back of conversion results */
	uint32_t *buffer;
	uint32_t *buffer_ptr;

	/* holds the sequence mask and will work down to 0 */
	uint32_t channel_mask;

	/* cached, prepared profiles for a quick lookup at read() */
	struct ads122c04_profile profiles[ADS122C04_MAX_PROFILE_COUNT];
	struct ads122c04_profile *current_profile;
	uint8_t profile_valid_mask;

	/* acquisition thread */
	struct k_thread thread;
	K_KERNEL_STACK_MEMBER(stack, CONFIG_ADC_ADS122C04_ACQUISITION_THREAD_STACK_SIZE);
};

/* ------ */
/* ad559x */
/* ------ */

static void ads122c04_acquisition_thread(const struct device *dev);
static int ads122c04_read_reg(const struct device *dev, enum ads122c04_reg reg_addr, uint8_t *buf);
static int ads122c04_write_reg(const struct device *dev, enum ads122c04_reg reg_addr,
                               uint8_t reg_val);
static int ads122c04_apply_channel_config(const struct device *dev, uint32_t channel_id);
static int ads122c04_start_conversion(const struct device *dev);
static int ads122c04_read_sample(const struct device *dev, uint32_t *buff);
static int ads122c04_wait_data_ready(const struct device *dev);

static int ads122c04_init(const struct device *dev)
{
	const struct ads122c04_config *config = dev->config;
	struct ads122c04_data *data = dev->data;
	int ret;
	uint8_t reg;
	k_tid_t tid;

	data->dev = dev;

	data->profile_valid_mask = 0;
	data->current_profile = NULL;

	adc_context_init(&data->ctx);

	k_sem_init(&data->acquisition_sem, 0, 1);

	if (!device_is_ready(config->bus.bus)) {
		return -ENODEV;
	}

	tid = k_thread_create(&data->thread, data->stack, K_KERNEL_STACK_SIZEOF(data->stack),
	                      (k_thread_entry_t)ads122c04_acquisition_thread, dev, NULL, NULL,
	                      CONFIG_ADC_ADS122C04_ACQUISITION_THREAD_PRIO, 0, K_NO_WAIT);

	if (IS_ENABLED(CONFIG_THREAD_NAME)) {
		ret = k_thread_name_set(tid, "ads122c04");
		if (ret < 0) {
			return ret;
		}
	}

	adc_context_unlock_unconditionally(&data->ctx);

	printk("adc init");
	return 0;
}

static void ads122c04_acquisition_thread(const struct device *dev)
{
	struct ads122c04_data *data = dev->data;
	uint32_t result;
	uint8_t channel;
	int ret;

	while (true) {
		k_sem_take(&data->acquisition_sem, K_FOREVER);

		LOG_DBG("ads122c04_acquisition_thread -- sampling started");

		uint32_t number = POPCOUNT(data->channel_mask);

		while (data->channel_mask != 0) {
			channel = find_lsb_set(data->channel_mask) - 1;

			//LOG_DBG("configuring channel %d", channel);

			/* make sure to have the correct config applied for the current channel */
			ret = ads122c04_apply_channel_config(dev, channel);
			if (ret < 0) {
				LOG_ERR("failed to apply channel %d config (ret %d)", channel, ret);
				break;
			}
			//LOG_DBG("channel %d started to sample", channel);

			ret = ads122c04_start_conversion(dev);
			if (ret < 0) {
				/* if we fail to complete the I2C operations to start
				 * sampling, return an immediate error (likely -EIO) rather
				 * than handing it off to the acquisition thread.
				 */
				LOG_ERR("failed to start conversion (ret %d)", ret);
				adc_context_complete(&data->ctx, ret);
				break;
			}

			//LOG_DBG("waiting for channel %d", channel);

			ret = ads122c04_wait_data_ready(dev);
			if (ret < 0) {
				LOG_ERR("failed to wait for channel %d (ret %d)", channel, ret);
				break;
			}

			//LOG_DBG("channel %d ready", channel);

			ret = ads122c04_read_sample(dev, &result);
			if (ret < 0) {
				LOG_ERR("failed to read channel %d (ret %d)", channel, ret);
				adc_context_complete(&data->ctx, ret);
				break;
			}
			//LOG_DBG("%p, channel %d result %d", data->buffer_ptr, channel, result);

			*data->buffer_ptr = result;
			data->buffer_ptr++;

			WRITE_BIT(data->channel_mask, channel, 0);
		}

		/*
		for (uint32_t i = 0; i < number; i++) {
			LOG_DBG("channel result [%d]= %i", i, data->buffer[i]);
		}
		*/
		LOG_DBG("ads122c04_acquisition_thread -- sampling done");
		adc_context_on_sampling_done(&data->ctx, data->dev);
	}
}

static int ads122c04_read_reg(const struct device *dev, enum ads122c04_reg reg_addr, uint8_t *buf)
{
	const struct ads122c04_config *config = dev->config;
	int ret;

	/* generate combined command and reg addr byte */
	reg_addr = ADS122C04_CMD_RREG | (reg_addr << ADS122C04_REG_ADDR_BIT_OFFS);

	//ret = i2c_burst_read_dt(&config->bus, reg_addr, buf, ADS122C04_REG_DATA_BYTE_COUNT);
	ret = i2c_reg_read_byte_dt(&config->bus, reg_addr, buf);
	if (ret < 0) {
		LOG_ERR("ADS1X1X[0x%X]: error reading register 0x%X (%d)", config->bus.addr,
		        reg_addr, ret);
		return ret;
	}

	return 0;
}

static int ads122c04_write_reg(const struct device *dev, enum ads122c04_reg reg_addr,
                               uint8_t reg_val)
{
	const struct ads122c04_config *config = dev->config;
	uint8_t buf[3];
	int ret;
	uint8_t read_reg_val;

	enum ads122c04_reg raw_reg = reg_addr;
	/* generate combined command and reg addr byte */
	reg_addr = ADS122C04_CMD_WREG | (reg_addr << ADS122C04_REG_ADDR_BIT_OFFS);

	buf[0] = reg_addr;
	buf[1] = reg_val;

	ret = -1;
	uint8_t retry = 3;

	while (retry--) {
		ret = i2c_reg_write_byte_dt(&config->bus, reg_addr, reg_val);
		// ret = i2c_write_dt(&config->bus, buf, sizeof(buf));

		if (ret == 0) {
			break;
		}
		LOG_ERR("i2c write error on cmd %#x", reg_addr);

		k_sleep(K_USEC(50000));
	}

	// if (ret != 0) {
	if (1) {

		retry = 3;

		while (0 != ads122c04_read_reg(dev, raw_reg, &read_reg_val)) {
			k_sleep(K_USEC(50000));

			retry--;

			if (retry == 0) {
				LOG_ERR("i2c read error on cmd %#x", reg_addr);
				return -5;
			}
		}

		if (raw_reg == ADS122C04_REG_CFG2) {
			read_reg_val &= ~BIT(7); // clear DRDY bit
		}

		if (reg_val != read_reg_val) {
			LOG_ERR("ADS1X1X[0x%X]: error register read value 0x%X not as written "
			        "0x%X, reg 0x%X, ret (%d)",
			        config->bus.addr, read_reg_val, reg_val, reg_addr, ret);
			return ret;
		}
	}

	return 0;
}

static int ads122c04_apply_channel_config(const struct device *dev, uint32_t channel_id)
{
	struct ads122c04_data *data = dev->data;
	int ret;

	struct ads122c04_profile *new_profile = &data->profiles[channel_id];

	if (new_profile == data->current_profile) {
		/* the current profile is already applied */
		return 0;
	}

	data->current_profile = new_profile;

	/* transfer to adc */
	for (uint32_t i = 0; i < ADS122C04_REG_COUNT; i++) {
		enum ads122c04_reg reg = i;

		ret = ads122c04_write_reg(dev, reg, data->current_profile->raw_regs[i]);
		if (ret) {
			LOG_ERR("failed to configure reg %d", reg);
			return -EINVAL;
		}
	}

	return 0;
}

static int ads122c04_cfg_gain(enum adc_gain gain, struct ads122c04_profile *profile)
{
	uint8_t gain_value;

	switch (gain) {
	case ADC_GAIN_1:
		gain_value = 1;
		break;
	case ADC_GAIN_2:
		gain_value = 2;
		break;
	case ADC_GAIN_4:
		gain_value = 4;
		break;
	case ADC_GAIN_8:
		gain_value = 8;
		break;
	case ADC_GAIN_16:
		gain_value = 16;
		break;
	case ADC_GAIN_32:
		gain_value = 32;
		break;
	case ADC_GAIN_64:
		gain_value = 64;
		break;
	case ADC_GAIN_128:
		gain_value = 128;
		break;
	default:
		return -EINVAL;
	}

	profile->gain = LOG2(gain_value);
	return 0;
}

static int ads122c04_mux_from_diff_inputs(uint8_t ain_p, uint8_t ain_n, uint8_t *mux)
{
	static const uint8_t mux_map[4][4] = {
		/* N=0  N=1  N=2  N=3 */
		{0xFF, 0x00, 0x01, 0x02}, /* P=0 */
		{0x03, 0xFF, 0x04, 0x05}, /* P=1 */
		{0xFF, 0xFF, 0xFF, 0x06}, /* P=2 */
		{0xFF, 0xFF, 0x07, 0xFF}, /* P=3 */
	};

	if (ain_p >= 4 || ain_n >= 4) {
		return -EINVAL;
	}

	if (mux_map[ain_p][ain_n] == 0xFF) {
		return -EINVAL; /* invalid pair */
	}

	*mux = mux_map[ain_p][ain_n];
	return 0;
}

static int ads122c04_cfg_mux(const struct adc_channel_cfg *channel_cfg,
                             struct ads122c04_profile *profile)
{
	int ret;
	uint8_t ts_mode = 0;
	uint8_t mux_value = 0;

	if (channel_cfg->differential) {
		/* differential */
		ret = ads122c04_mux_from_diff_inputs(channel_cfg->input_positive,
		                                     channel_cfg->input_negative, &mux_value);
		if (ret < 0) {
			LOG_ERR("failed to setup mux value for ain_p: %d, ain_n: %d",
			        channel_cfg->input_positive, channel_cfg->input_negative);
			return ret;
		}

	} else {
		/* single ended */
		if (channel_cfg->input_positive <= AIN3) {
			/* directly map channels 0..3 */
			mux_value = channel_cfg->input_positive | BIT(3);
		} else if (channel_cfg->input_positive == CHIP_TEMPERATURE) {
			/* treat virtual channel 4 as chip temperature reading */
			ts_mode = 1;
			mux_value = 0;
		} else {
			LOG_ERR("unsupported input %d", channel_cfg->input_positive);
			return -EINVAL;
		}
	}

	profile->mux = mux_value;
	profile->temperature_sensor_mode = ts_mode;

	return 0;
}

static int ads122c04_cfg_acquisition_time(const struct adc_channel_cfg *channel_cfg,
                                          struct ads122c04_profile *profile)
{
	uint16_t sample_freq = 0;
	uint8_t odr = 0;
	k_timeout_t ready_time;

	if (channel_cfg->acquisition_time == ADC_ACQ_TIME_DEFAULT) {
		sample_freq = 20;
	} else {
		if (ADC_ACQ_TIME_UNIT(channel_cfg->acquisition_time) != ADC_ACQ_TIME_TICKS) {
			LOG_ERR("unsupported acquisition time unit");
			return -EINVAL;
		}

		sample_freq = ADC_ACQ_TIME_VALUE(channel_cfg->acquisition_time);
	}

	switch (sample_freq) {
	case 20:
		odr = 0;
		ready_time = K_USEC(1000*1000 / 20 + 10);
		break;
	case 45:
		odr = 1;
		ready_time = K_USEC(1000*1000 / 40 + 10);
		break;
	case 90:
		odr = 2;
		ready_time = K_USEC(1000*1000 / 90 + 10);
		break;
	case 175:
		odr = 3;
		ready_time = K_USEC(1000*1000 / 175 + 10);
		break;
	case 330:
		odr = 4;
		ready_time = K_USEC(1000*1000 / 330 + 10);
		break;
	case 600:
		odr = 5;
		ready_time = K_USEC(1000*1000 / 600 + 10);
		break;
	case 1000:
		odr = 6;
		ready_time = K_USEC(1000*1000 / 1000 + 10);
		break;
	default:
		LOG_ERR("unsupported acquisition time value");
		return -EINVAL;
	}

	profile->data_rate = odr;
	profile->ready_time = ready_time;

	return 0;
}

static int ads122c04_cfg_idac(const struct adc_channel_cfg *channel_cfg,
                              struct ads122c04_profile *profile)
{
	if (channel_cfg->current_source_pin_set) {

		if (channel_cfg->current_source_pin[0]) {
			if (channel_cfg->current_source_pin[0] &
			    ADS122C04_CURRENT_SOURCE_CURRENT_MSK) {
				profile->idac =
					(channel_cfg->current_source_pin[0] &
					 ADS122C04_CURRENT_SOURCE_CURRENT_MSK) >>
					ADS122C04_CURRENT_SOURCE_CURRENT_SHIFT;
			}

			profile->i1mux = channel_cfg->current_source_pin[0] &
			                 ADS122C04_CURRENT_SOURCE_MUX_MSK;
		}

		if (channel_cfg->current_source_pin[1]) {
			if (channel_cfg->current_source_pin[1] &
			    ADS122C04_CURRENT_SOURCE_CURRENT_MSK) {
				profile->idac =
					(channel_cfg->current_source_pin[1] &
					 ADS122C04_CURRENT_SOURCE_CURRENT_MSK) >>
					ADS122C04_CURRENT_SOURCE_CURRENT_SHIFT;
			}

			profile->i2mux = channel_cfg->current_source_pin[1] &
			                 ADS122C04_CURRENT_SOURCE_MUX_MSK;
		}
	}
	return 0;
}

static int ads122c04_profile_from_channel_cfg(const struct adc_channel_cfg *channel_cfg,
                                              struct ads122c04_profile *profile)
{
	int ret;

	memset(profile, 0, sizeof(*profile));

	/* gain */
	ret = ads122c04_cfg_gain(channel_cfg->gain, profile);
	if (ret < 0) {
		LOG_ERR("unsupported gain value");
		return ret;
	}

	/* pin mux */
	ret = ads122c04_cfg_mux(channel_cfg, profile);
	if (ret < 0) {
		LOG_ERR("failed to configure pin mux");
		return ret;
	}

	/* acquisition time */
	ret = ads122c04_cfg_acquisition_time(channel_cfg, profile);
	if (ret < 0) {
		LOG_ERR("failed to configure acquisition time %u",
		        channel_cfg->acquisition_time);
		return ret;
	}

	/* idac */
	ret = ads122c04_cfg_idac(channel_cfg, profile);
	if (ret < 0) {
		LOG_ERR("failed to configure IDAC");
		return ret;
	}

	if (channel_cfg->reference == ADC_REF_EXTERNAL0) {
	}

	profile->vref = 0;
	profile->conversion_mode = 0;
	profile->speed_mode = 0;

	profile->crc_enabled = 0;
	profile->data_count_enable = 0;

	return 0;
}

static inline void ads122c04_regs_to_raw(const struct ads122c04_profile *profile)
{
	uint8_t *raw = profile->raw_regs;

	raw[ADS122C04_REG_CFG0] =
		((profile->pga_bypass & ADS122C04_FIELD_MASK(1))
		 << ADS122C04_REG0_PGA_BYPASS_SHIFT) |
		((profile->gain & ADS122C04_FIELD_MASK(3)) << ADS122C04_REG0_GAIN_SHIFT) |
		((profile->mux & ADS122C04_FIELD_MASK(4)) << ADS122C04_REG0_MUX_SHIFT);

	raw[ADS122C04_REG_CFG1] =
		((profile->temperature_sensor_mode & ADS122C04_FIELD_MASK(1)) <<
		 ADS122C04_REG1_TS_SHIFT) |
		((profile->vref & ADS122C04_FIELD_MASK(2)) << ADS122C04_REG1_VREF_SHIFT) |
		((profile->conversion_mode & ADS122C04_FIELD_MASK(1)) << ADS122C04_REG1_CM_SHIFT) |
		((profile->speed_mode & ADS122C04_FIELD_MASK(1)) << ADS122C04_REG1_OP_MODE_SHIFT) |
		((profile->data_rate & ADS122C04_FIELD_MASK(3)) << ADS122C04_REG1_DATA_RATE_SHIFT);

	raw[ADS122C04_REG_CFG2] =
		((profile->idac & ADS122C04_FIELD_MASK(3)) << ADS122C04_REG2_IDAC_SHIFT) |
		((profile->crc_enabled & ADS122C04_FIELD_MASK(2)) << ADS122C04_REG2_CRC_SHIFT) |
		((profile->data_count_enable & ADS122C04_FIELD_MASK(1)) <<
		 ADS122C04_REG2_DCNT_SHIFT);

	raw[ADS122C04_REG_CFG3] =
		((profile->i2mux & ADS122C04_FIELD_MASK(3)) << ADS122C04_REG3_I2MUX_SHIFT) |
		((profile->i1mux & ADS122C04_FIELD_MASK(3)) << ADS122C04_REG3_I1MUX_SHIFT);
}

static int ads122c04_wait_data_ready(const struct device *dev)
{
	struct ads122c04_data *data = dev->data;
	int ret;
	uint8_t status = 0;

	/* wait for the expected conversion time to pass */
	k_sleep(data->current_profile->ready_time);

	while (true) {
		/* check the status for DRDY bit */
		ret = ads122c04_read_reg(dev, ADS122C04_REG_CFG2, &status);
		if (ret < 0) {
			LOG_ERR("error reading status %d", ret);
			return ret;
		}

		if ((status & BIT(ADS122C04_CFG2_DRDY_OFFS))) {
			break;
		}

		k_sleep(K_USEC(1000));
	}

	return 0;
}

static int ads122c04_read_sample(const struct device *dev, uint32_t *buff)
{
	const struct ads122c04_config *config = dev->config;
	int ret;
	uint8_t sample[ADS122C04_SAMPLE_BYTE_COUNT] = {0};

	ret = i2c_burst_read_dt(&config->bus, ADS122C04_CMD_RDATA, sample,
	                        ADS122C04_SAMPLE_BYTE_COUNT);
	if (ret != 0) {
		LOG_ERR("ADS1X1X[0x%X]: error reading sample %d", config->bus.addr, ret);
		return ret;
	}

	uint32_t raw_value = sys_get_be24(sample);
	if (raw_value & 0x00800000) {
		raw_value |= 0xff000000;
	}

	*buff = raw_value;

	return 0;
}

static int ads122c04_validate_sequence(const struct device *dev,
                                       const struct adc_sequence *sequence)
{
	struct ads122c04_data *data = dev->data;
	uint32_t extra_samplings = 0;
	size_t num_channels;
	size_t required;

	if (sequence->oversampling) {
		LOG_ERR("Oversampling not supported");
		return -EINVAL;
	}

	if (sequence->resolution != ADS122C04_RESOLUTION_BITS) {
		LOG_ERR("Resolution not supported");
		return -EINVAL;
	}

	if (sequence->calibrate) {
		LOG_ERR("Calibration not supported");
		return -EINVAL;
	}

	/* make sure all channels are configured */
	for (uint32_t i = 0; i < ADS122C04_MAX_PROFILE_COUNT; i++) {
		if (sequence->channels & BIT(i)) {
			if ((!data->profile_valid_mask) & BIT(i)) {
				LOG_ERR("Channel %d not configured", i);
				return -EINVAL;
			}
		}
	}

	if (sequence->options) {
		extra_samplings = sequence->options->extra_samplings;
	}

	num_channels = POPCOUNT(sequence->channels);
	required = num_channels * ADS122C04_SAMPLE_BYTE_COUNT * (1 + extra_samplings);

	if (sequence->buffer_size < required) {
		LOG_ERR("Insufficient buffer size for extra samplings");
		return -ENOMEM;
	}

	return 0;
}

static int ads122c04_start_conversion(const struct device *dev)
{
	const struct ads122c04_config *config = dev->config;

	int ret;

	const uint8_t cmd = ADS122C04_CMD_START;

	ret = i2c_write_dt(&config->bus, &cmd, 1);
	if (ret != 0) {
		LOG_ERR("ADS1X1X[0x%X]: error start conversion %d", config->bus.addr, ret);
		return ret;
	}

	return 0;
}

static void adc_context_start_sampling(struct adc_context *ctx)
{
	struct ads122c04_data *data = CONTAINER_OF(ctx, struct ads122c04_data, ctx);

	data->channel_mask = ctx->sequence.channels;

	k_sem_give(&data->acquisition_sem);
}

static void adc_context_update_buffer_pointer(struct adc_context *ctx, bool repeat_sampling)
{
	struct ads122c04_data *data = CONTAINER_OF(ctx, struct ads122c04_data, ctx);
	uint32_t number;
	number = POPCOUNT(ctx->sequence.channels);

	if (!repeat_sampling) {
		/* advance buffer pointer to accommodate the next channel sweep */
		data->buffer = &data->buffer[number];
	}

	data->buffer_ptr = data->buffer;
}

/*
 * translate channel_cfg into an adc configuration profile and cache
 * this into the appropriate profile slot for quick lookup at the read()
 *
 * note: here we only create profiles; profiles will be applied only on read()
 */
static int ads122c04_channel_setup(const struct device *dev,
                                   const struct adc_channel_cfg *channel_cfg)
{
	struct ads122c04_data *data = dev->data;
	int ret;
	uint8_t cfg;

	if (channel_cfg->channel_id >= ADS122C04_MAX_PROFILE_COUNT) {
		return -EINVAL;
	}

	/* as we are modifying the profiles, so we need to block reading */
	adc_context_lock(&data->ctx, false, NULL);

	struct ads122c04_profile *profile = &data->profiles[channel_cfg->channel_id];

	ret = ads122c04_profile_from_channel_cfg(channel_cfg, profile);
	if (ret < 0) {
		adc_context_release(&data->ctx, ret);
		return -EINVAL;
	}

	/* convert profile to raw regs and store into the lookup slot */
	ads122c04_regs_to_raw(profile);
	data->profile_valid_mask |= BIT(channel_cfg->channel_id);

	if (data->current_profile == profile) {
		/* we modified the current active profile; enforce reload */
		data->current_profile = NULL;
	}

	adc_context_release(&data->ctx, ret);

	LOG_DBG("Channel %d configured", channel_cfg->channel_id);
	return 0;
}

static int ads122c04_read(const struct device *dev, const struct adc_sequence *sequence)
{
	struct ads122c04_data *data = dev->data;
	int ret;

	adc_context_lock(&data->ctx, false, NULL);

	ret = ads122c04_validate_sequence(dev, sequence);
	if (ret < 0) {
		adc_context_release(&data->ctx, ret);
		return ret;
	}

	data->buffer = sequence->buffer;
	data->buffer_ptr = data->buffer;

	LOG_DBG("start reading channels %x", sequence->channels);

	adc_context_start_read(&data->ctx, sequence);

	ret = adc_context_wait_for_completion(&data->ctx);

	adc_context_release(&data->ctx, ret);
	return ret;
}

#define DT_DRV_COMPAT ti_ads122c04

static DEVICE_API(adc, api) = {
	.channel_setup = ads122c04_channel_setup,
	.read = ads122c04_read,
	.ref_internal = ADS122C04_REF_INTERNAL,
};

#define ADC_ADS122C04_INST_DEFINE(n)                                                               \
	static const struct ads122c04_config ads122c04_config_##n = {                              \
		.bus = I2C_DT_SPEC_INST_GET(n)};                                                   \
	static struct ads122c04_data ads122c04_data_##n = {                                        \
		ADC_CONTEXT_INIT_LOCK(ads122c04_data_##n, ctx),                                    \
		ADC_CONTEXT_INIT_TIMER(ads122c04_data_##n, ctx),                                   \
		ADC_CONTEXT_INIT_SYNC(ads122c04_data_##n, ctx),                                    \
                                                                                                   \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(n, ads122c04_init, NULL, &ads122c04_data_##n, &ads122c04_config_##n, \
			      POST_KERNEL, CONFIG_ADC_INIT_PRIORITY, &api);

DT_INST_FOREACH_STATUS_OKAY(ADC_ADS122C04_INST_DEFINE);
