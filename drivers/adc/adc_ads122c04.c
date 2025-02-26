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
LOG_MODULE_REGISTER(ads122c04, LOG_LEVEL_DBG);

#define ADC_CONTEXT_USES_KERNEL_TIMER
#include "adc_context.h"

#define ADS122C04_REF_INTERNAL 0x815

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

enum ads122c04_cmd {
	ADS122C04_CMD_RESET = 0x06,
	ADS122C04_CMD_START = 0x08,
	ADS122C04_CMD_POWER_DOWN = 0x02,
	ADS122C04_CMD_RDATA = 0x10,
	ADS122C04_CMD_RREG = 0x20,
	ADS122C04_CMD_WREG = 0x40,
};

enum ads122c04_reg {
	ADS122C04_REG_CFG0 = 0x0,
	ADS122C04_REG_CFG1 = 0x1,
	ADS122C04_REG_CFG2 = 0x2,
	ADS122C04_REG_CFG3 = 0x3,
};

struct ads122c04_config {
	const struct i2c_dt_spec bus;
};

struct ads122c04_data {
	const struct device *dev;
	struct adc_context ctx;
	k_timeout_t ready_time;
	struct k_sem acq_sem;
	int32_t *buffer;
	int32_t *buffer_ptr;
	bool differential;
};

static int ads122c04_read_reg(const struct device *dev, enum ads122c04_reg reg_addr, uint8_t *buf);
static int ads122c04_write_reg(const struct device *dev, enum ads122c04_reg reg_addr,
			       uint8_t reg_val);
static int ads122c04_start_conversion(const struct device *dev);

static int ads122c04_init(const struct device *dev)
{
	const struct ads122c04_config *config = dev->config;
	struct ads122c04_data *data = dev->data;
	int res;
	uint8_t reg;

	data->dev = dev;

	adc_context_init(&data->ctx);

	k_sem_init(&data->acq_sem, 0, 1);

	if (!device_is_ready(config->bus.bus)) {
		return -ENODEV;
	}

	adc_context_unlock_unconditionally(&data->ctx);

	printk("adc init");
	return 0;
}

static void adc_context_start_sampling(struct adc_context *ctx)
{
	struct ads122c04_data *data = CONTAINER_OF(ctx, struct ads122c04_data, ctx);
	int ret;

	data->buffer_ptr = data->buffer;

	ret = ads122c04_start_conversion(data->dev);
	if (ret != 0) {
		/* if we fail to complete the I2C operations to start
		 * sampling, return an immediate error (likely -EIO) rather
		 * than handing it off to the acquisition thread.
		 */
		adc_context_complete(ctx, ret);
		return;
	}

	k_sem_give(&data->acq_sem);
}

static void adc_context_update_buffer_pointer(struct adc_context *ctx, bool repeat_sampling)
{
	struct ads122c04_data *data = CONTAINER_OF(ctx, struct ads122c04_data, ctx);

	if (repeat_sampling) {
		data->buffer = data->buffer_ptr;
	}
}

static int ads122c04_allowed_gain(enum adc_gain gain)
{
	switch (gain) {
	case ADC_GAIN_1:
		return 1;
	case ADC_GAIN_2:
		return 2;
	case ADC_GAIN_4:
		return 4;
	case ADC_GAIN_8:
		return 8;
	case ADC_GAIN_16:
		return 16;
	case ADC_GAIN_32:
		return 32;
	case ADC_GAIN_64:
		return 64;
	case ADC_GAIN_128:
		return 128;
	default:
		return 1;
	}
}

static int ads122c04_channel_setup(const struct device *dev,
				   const struct adc_channel_cfg *channel_cfg)
{
	struct ads122c04_data *data = dev->data;
	int ret;
	uint8_t cfg;

	if (channel_cfg->channel_id >= ADS122C04_CHANNEL_COUNT) {
		// return -EINVAL;
	}

	if (channel_cfg->differential) {

	} else {
		/* single ended */

		/* config 0 */
		// cfg = ADS122C04_MUX_SINGLE(channel_cfg->channel_id);
		cfg = 0;
		// cfg |= (0x0 << 4); // AINP = AIN0, AINN = AIN1
		cfg |= (0x6 << 4); // AINP = AIN1, AINN = AIN0
		// cfg |= (0x8 << 4); // AINP = AIN1, AINN = AIN0

		// cfg |= (0x6 << 4); // AINP = AIN2, AINN = AIN3
		// cfg |= (7 << 4); // AINP = AIN3, AINN = AIN2
		//  cfg |= (0xa << 4); // AINP = AIN2, AINN = GND

		cfg |= ADS122C04_GAIN(ads122c04_allowed_gain(channel_cfg->gain));
		// cfg &= ~BIT(ADS122C04_CFG0_PGA_DISABLE_OFFS);
		// cfg |= BIT(ADS122C04_CFG0_PGA_DISABLE_OFFS);

		ret = ads122c04_write_reg(dev, ADS122C04_REG_CFG0, cfg);
		if (ret) {
			LOG_ERR("failed to configure reg %d", ADS122C04_REG_CFG0);
			return -EINVAL;
		}

		/* config 1 */
		cfg = 0;
		// cfg |= (0x2 << 5); // 90 SPS
		// cfg |= (0x5 << 5); // 600 SPS
		//  cfg |= (0x6 << 5); // 1000 SPS

		// cfg |= (0x1 << 1); // reference REFP - REFN

		// cfg &= ~BIT(ADS122C04_CFG1_CONTINUOUS_OFFS);

		ret = ads122c04_write_reg(dev, ADS122C04_REG_CFG1, cfg);
		if (ret) {
			LOG_ERR("failed to configure reg %d", ADS122C04_REG_CFG1);
			return -EINVAL;
		}

		/* config 2 */
		cfg = 0;
		// cfg |= 3;	// measurement current of 100uA
		// cfg |= 6;	// measurement current of 1mA
		// cfg |= 7;	// measurement current of 1.5mA

		ret = ads122c04_write_reg(dev, ADS122C04_REG_CFG2, cfg);
		if (ret) {
			LOG_ERR("failed to configure reg %d", ADS122C04_REG_CFG2);
			return -EINVAL;
		}

		/* config 3 */
		cfg = 0;
		// cfg |= (4 << 5);	// inject current into channel AIN3

		ret = ads122c04_write_reg(dev, ADS122C04_REG_CFG3, cfg);
		if (ret) {
			LOG_ERR("failed to configure reg %d", ADS122C04_REG_CFG3);
			return -EINVAL;
		}
	}

	return 0;
}

static int ads122c04_wait_data_ready(const struct device *dev)
{
	int rc;
	struct ads122c04_data *data = dev->data;

	k_sleep(data->ready_time);
	uint8_t status = 0;

	rc = ads122c04_read_reg(dev, ADS122C04_REG_CFG2, &status);
	if (rc != 0) {
		return rc;
	}

	while ((status & BIT(ADS122C04_CFG2_DRDY_OFFS)) == 0) {

		k_sleep(K_USEC(100));
		rc = ads122c04_read_reg(dev, ADS122C04_REG_CFG2, &status);
		if (rc != 0) {
			return rc;
		}
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

static int ads122c04_start_conversion(const struct device *dev)
{
	const struct ads122c04_config *config = dev->config;
	int ret;

	uint8_t cmd = ADS122C04_CMD_START;

	ret = i2c_write_dt(&config->bus, &cmd, 1);
	if (ret != 0) {
		LOG_ERR("ADS1X1X[0x%X]: error start conversion %d", config->bus.addr, ret);
		return ret;
	}

	return 0;
}

static int ads122c04_validate_buffer_size(const struct adc_sequence *sequence)
{
	size_t needed = sizeof(int32_t);

	if (sequence->options) {
		needed *= (1 + sequence->options->extra_samplings);
	}

	if (sequence->buffer_size < needed) {
		LOG_ERR("Insufficient buffer %i < %i", sequence->buffer_size, needed);
		return -ENOMEM;
	}

	return 0;
}

static int ads122c04_validate_sequence(const struct device *dev,
				       const struct adc_sequence *sequence)
{
	if (sequence->channels != BIT(0)) {
		LOG_ERR("Invalid Channel 0x%x", sequence->channels);
		return -EINVAL;
	}

	if (sequence->oversampling) {
		LOG_ERR("Oversampling not supported");
		return -EINVAL;
	}

	return ads122c04_validate_buffer_size(sequence);
}

static int ads122c04_adc_start_read(const struct device *dev, const struct adc_sequence *sequence,
				    bool wait)
{
	int rc = 0;
	struct ads122c04_data *data = dev->data;

	rc = ads122c04_validate_sequence(dev, sequence);
	if (rc != 0) {
		return rc;
	}

	data->buffer = sequence->buffer;

	adc_context_start_read(&data->ctx, sequence);

	if (wait) {
		rc = adc_context_wait_for_completion(&data->ctx);
	}
	return rc;
}

static int ads122c04_adc_perform_read(const struct device *dev)
{
	int rc;
	struct ads122c04_data *data = dev->data;

	k_sem_take(&data->acq_sem, K_FOREVER);

	rc = ads122c04_wait_data_ready(dev);
	if (rc != 0) {
		adc_context_complete(&data->ctx, rc);
		return rc;
	}

	rc = ads122c04_read_sample(dev, data->buffer);
	if (rc != 0) {
		adc_context_complete(&data->ctx, rc);
		return rc;
	}
	data->buffer++;

	adc_context_on_sampling_done(&data->ctx, dev);

	return rc;
}

static int ads122c04_read(const struct device *dev, const struct adc_sequence *sequence)
{
	int rc;
	struct ads122c04_data *data = dev->data;

	adc_context_lock(&data->ctx, false, NULL);
	rc = ads122c04_adc_start_read(dev, sequence, false);

	while (rc == 0 && k_sem_take(&data->ctx.sync, K_NO_WAIT) != 0) {
		rc = ads122c04_adc_perform_read(dev);
	}

	adc_context_release(&data->ctx, rc);
	return rc;
}

static int ads122c04_read_reg(const struct device *dev, enum ads122c04_reg reg_addr, uint8_t *buf)
{
	const struct ads122c04_config *config = dev->config;
	int ret;

	/* generate combined command and reg addr byte */
	reg_addr = ADS122C04_CMD_RREG | (reg_addr << ADS122C04_REG_ADDR_BIT_OFFS);

	ret = i2c_burst_read_dt(&config->bus, reg_addr, buf, ADS122C04_REG_DATA_BYTE_COUNT);
	if (ret != 0) {
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

	/* generate combined command and reg addr byte */
	reg_addr = ADS122C04_CMD_WREG | (reg_addr << ADS122C04_REG_ADDR_BIT_OFFS);

	buf[0] = reg_addr;
	buf[1] = reg_val;

	ret = i2c_write_dt(&config->bus, buf, sizeof(buf));

	if (ret != 0) {
		LOG_ERR("ADS1X1X[0x%X]: error writing register 0x%X (%d)", config->bus.addr,
			reg_addr, ret);
		return ret;
	}

	return 0;
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
