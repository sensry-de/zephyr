/* Bosch BMA4xx 3-axis accelerometer driver
 *
 * Copyright (c) 2023 Google LLC
 * Copyright (c) 2024 Croxel Inc.
 * Copyright (c) 2024 Cienet
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT bosch_bma4xx

#include <zephyr/init.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/device.h>
#include <zephyr/rtio/work.h>

#include "bma4xx.h"
#include "bma4xx_interrupt.h"
#include "bma4xx_decoder.h"
#include "bma4xx_rtio.h"

LOG_MODULE_REGISTER(bma4xx, CONFIG_SENSOR_LOG_LEVEL);

/**
 * @brief Helper for converting m/s^2 offset values into register values
 */
static int bma4xx_offset_to_reg_val(const struct sensor_value *val, uint8_t *reg_val)
{
	int32_t ug = sensor_ms2_to_ug(val);

	if (ug < BMA4XX_OFFSET_MICROG_MIN || ug > BMA4XX_OFFSET_MICROG_MAX) {
		return -ERANGE;
	}

	*reg_val = ug / BMA4XX_OFFSET_MICROG_PER_BIT;
	return 0;
}

/**
 * @brief Set the X, Y, or Z axis offsets.
 */
static int bma4xx_attr_set_offset(const struct device *dev, enum sensor_channel chan,
				  const struct sensor_value *val)
{
	struct bma4xx_data *bma4xx = dev->data;
	uint8_t reg_addr;
	uint8_t reg_val[3];
	int rc;

	switch (chan) {
	case SENSOR_CHAN_ACCEL_X:
	case SENSOR_CHAN_ACCEL_Y:
	case SENSOR_CHAN_ACCEL_Z:
		reg_addr = BMA4XX_REG_OFFSET_0 + (chan - SENSOR_CHAN_ACCEL_X);
		rc = bma4xx_offset_to_reg_val(val, &reg_val[0]);
		if (rc) {
			return rc;
		}
		return bma4xx->hw_ops->write_reg(dev, reg_addr, reg_val[0]);
	case SENSOR_CHAN_ACCEL_XYZ:
		/* Expect val to point to an array of three sensor_values */
		reg_addr = BMA4XX_REG_OFFSET_0;
		for (int i = 0; i < 3; i++) {
			rc = bma4xx_offset_to_reg_val(&val[i], &reg_val[i]);
			if (rc) {
				return rc;
			}
		}
		return bma4xx->hw_ops->write_data(dev, reg_addr, (uint8_t *)reg_val,
						  sizeof(reg_val));
	default:
		return -ENOTSUP;
	}
}

static const uint32_t odr_to_reg_map[] = {
	0,          /* Invalid */
	781250,     /* 0.78125 Hz (25/32) => 0x1 */
	1562500,    /* 1.5625 Hz (25/16) => 0x2 */
	3125000,    /* 3.125 Hz (25/8) => 0x3 */
	6250000,    /* 6.25 Hz (25/4) => 0x4 */
	12500000,   /* 12.5 Hz (25/2) => 0x5 */
	25000000,   /* 25 Hz => 0x6 */
	50000000,   /* 50 Hz => 0x7*/
	100000000,  /* 100 Hz => 0x8*/
	200000000,  /* 200 Hz => 0x9*/
	400000000,  /* 400 Hz => 0xa*/
	800000000,  /* 800 Hz => 0xb*/
	1600000000, /* 1600 Hz => 0xc*/
};

/**
 * @brief Convert an ODR rate in Hz to a register value
 */
static int bma4xx_odr_to_reg(uint32_t microhertz, uint8_t *reg_val)
{
	if (microhertz == 0) {
		/* Illegal ODR value */
		return -ERANGE;
	}

	for (uint8_t i = 0; i < ARRAY_SIZE(odr_to_reg_map); i++) {
		if (microhertz <= odr_to_reg_map[i]) {
			*reg_val = i;
			return 0;
		}
	}

	/* Requested ODR too high */
	return -ERANGE;
}

/**
 * Set the sensor's acceleration offset (per axis).
 */
static int bma4xx_attr_set_odr(const struct sensor_value *val,
			       struct bma4xx_runtime_config *new_config)
{
	int status;
	uint8_t reg_val;

	/* Convert ODR Hz value to microhertz and round up to closest register setting */
	status = bma4xx_odr_to_reg(val->val1 * 1000000 + val->val2, &reg_val);
	if (status < 0) {
		return status;
	}

	new_config->accel_odr = reg_val;

	return 0;
}

static const uint32_t fs_to_reg_map[] = {
	2000000,  /* +/-2G => 0x0 */
	4000000,  /* +/-4G => 0x1 */
	8000000,  /* +/-8G => 0x2 */
	16000000, /* +/-16G => 0x3 */
};

static int bma4xx_fs_to_reg(int32_t range_ug, uint8_t *reg_val)
{
	if (range_ug == 0) {
		/* Illegal value */
		return -ERANGE;
	}

	range_ug = abs(range_ug);

	for (uint8_t i = 0; i < 4; i++) {
		if (range_ug <= fs_to_reg_map[i]) {
			*reg_val = i;
			return 0;
		}
	}

	/* Requested range too high */
	return -ERANGE;
}

/**
 * Set the sensor's full-scale range
 */
static int bma4xx_attr_set_range(const struct sensor_value *val,
				 struct bma4xx_runtime_config *new_config)
{
	int status;
	uint8_t reg_val;

	/* Convert m/s^2 to micro-G's and find closest register setting */
	status = bma4xx_fs_to_reg(sensor_ms2_to_ug(val), &reg_val);
	if (status < 0) {
		return status;
	}

	new_config->accel_fs_range = reg_val;

	return 0;
}

/**
 * Set the sensor's bandwidth parameter (one of BMA4XX_BWP_*)
 */
static int bma4xx_attr_set_bwp(const struct sensor_value *val,
			       struct bma4xx_runtime_config *new_config)
{
	/* Ensure a valid BWP for performance mode, the only mode currently supported */
	if (val->val2 != 0 || val->val1 < BMA4XX_BWP_OSR4_AVG1 ||
	    val->val1 > BMA4XX_BWP_NORM_AVG4) {
		return -EINVAL;
	}

	new_config->accel_bwp = (uint8_t)val->val1;

	return 0;
}

/**
 * @brief Implement the sensor API attribute set method.
 */
static int bma4xx_attr_set(const struct device *dev, enum sensor_channel chan,
			   enum sensor_attribute attr, const struct sensor_value *val)
{
	const struct bma4xx_data *data = dev->data;
	struct bma4xx_runtime_config new_config = data->cfg;
	int res = 0;

	__ASSERT_NO_MSG(val != NULL);

	switch (chan) {
	case SENSOR_CHAN_ACCEL_X:
	case SENSOR_CHAN_ACCEL_Y:
	case SENSOR_CHAN_ACCEL_Z:
	case SENSOR_CHAN_ACCEL_XYZ:
		if (attr == SENSOR_ATTR_SAMPLING_FREQUENCY) {
			res = bma4xx_attr_set_odr(val, &new_config);
		} else if (attr == SENSOR_ATTR_FULL_SCALE) {
			res = bma4xx_attr_set_range(val, &new_config);
		} else if (attr == SENSOR_ATTR_OFFSET) {
			res = bma4xx_attr_set_offset(dev, chan, val);
		} else if (attr == SENSOR_ATTR_CONFIGURATION) {
			/* Use for setting the bandwidth parameter (BWP) */
			res = bma4xx_attr_set_bwp(val, &new_config);
		} else {
			LOG_ERR("Unsupported attribute");
			res = -ENOTSUP;
		}
		break;
	case SENSOR_CHAN_ALL:
		if (attr == SENSOR_ATTR_BATCH_DURATION) {
			if (val->val1 < 0) {
				return -EINVAL;
			}
			new_config.batch_ticks = val->val1;
		} else {
			LOG_ERR("Unsupported attribute");
			res = -EINVAL;
		}
		break;
	default:
		LOG_ERR("Unsupported channel");
		res = -EINVAL;
		break;
	}

	if (res) {
		LOG_ERR("Failed to set attribute");
		return res;
	}

	return bma4xx_safely_configure(dev, &new_config);
}

/**
 * Internal device initialization function for both bus types.
 */
static int bma4xx_chip_init(const struct device *dev)
{
	struct bma4xx_data *bma4xx = dev->data;
	const struct bma4xx_config *cfg = dev->config;
	int status;

	/* Sensor bus-specific initialization */
	status = cfg->bus_init(dev);
	if (status) {
		LOG_ERR("Failed to initialize bus: %d", status);
		return status;
	}

	/* Read Chip ID */
	status = bma4xx->hw_ops->read_reg(dev, BMA4XX_REG_CHIP_ID, &bma4xx->chip_id);
	if (status) {
		LOG_ERR("could not read chip_id: %d", status);
		return status;
	}
	LOG_DBG("chip_id is 0x%02x", bma4xx->chip_id);

	switch (bma4xx->chip_id) {
	case BMA4XX_CHIP_ID_BMA423:
		LOG_WRN("Driver tested for BMA422/BMA400. Check for unintended operation.");
	case BMA4XX_CHIP_ID_BMA400:
	case BMA4XX_CHIP_ID_BMA422:
	case BMA4XX_CHIP_ID_BMA456:
		break;
	default:
		LOG_ERR("Chip id (0x%02X) not supported", bma4xx->chip_id);
		return -ENODEV;
	}

	if (bma4xx->chip_id != BMA4XX_CHIP_ID_BMA422) {
		LOG_WRN("Driver tested for BMA422. Check for unintended operation.");
	}

	/* Issue soft reset command */
	status = bma4xx->hw_ops->write_reg(dev, BMA4XX_REG_CMD, BMA4XX_CMD_SOFT_RESET);
	if (status) {
		LOG_ERR("Could not soft-reset chip: %d", status);
		return status;
	}

	if (IS_ENABLED(CONFIG_BMA4XX_STREAM)) {
		status = bma4xx_init_interrupt(dev);
		if (status != 0) {
			LOG_ERR("Failed to initialize bma4xx interrupt");
			return status;
		}
	}

	/* Default is: range = +/-4G, ODR = 100 Hz, BWP = "NORM_AVG4" */
	bma4xx->cfg.accel_fs_range = BMA4XX_RANGE_4G;
	bma4xx->cfg.accel_bwp = BMA4XX_BWP_NORM_AVG4;
	bma4xx->cfg.accel_odr = BMA4XX_ODR_100;

	status = bma4xx_configure(dev, &bma4xx->cfg);
	if (status) {
		LOG_ERR("Failed to initialize bma4xx trigger");
		return status;
	}

	return 0;
}

static int bma4xx_attr_get(const struct device *dev, enum sensor_channel chan,
			   enum sensor_attribute attr, struct sensor_value *val)
{
	struct bma4xx_data *bma4xx = dev->data;

	switch (attr) {
	case SENSOR_ATTR_CHIP_ID:
		val->val1 = bma4xx->chip_id;
		return 0;
	default:
		LOG_ERR("Attribute not supported");
		return -EINVAL;
	}
}

int bma4xx_sensor_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	struct bma4xx_data *bma4xx = dev->data;

	int16_t values[3];
	int8_t temperature;
	bma4xx_sample_fetch(dev, &values[0], &values[1], &values[2]);
	bma4xx_temp_fetch(dev, &temperature);

	switch (chan) {
	case SENSOR_CHAN_ACCEL_X:
		bma4xx->fetched_values[0] = values[0];
		return 0;
	case SENSOR_CHAN_ACCEL_Y:
		bma4xx->fetched_values[1] = values[1];
		return 0;
	case SENSOR_CHAN_ACCEL_Z:
		bma4xx->fetched_values[2] = values[2];
		return 0;
	case SENSOR_CHAN_ACCEL_XYZ:
		memcpy(bma4xx->fetched_values, values, sizeof(values));
		return 0;
	case SENSOR_CHAN_DIE_TEMP:
		bma4xx->fetched_temperature = temperature;
		return 0;
	case SENSOR_CHAN_ALL:
		memcpy(bma4xx->fetched_values, values, sizeof(values));
		bma4xx->fetched_temperature = temperature;
		return 0;
	}

	return -EINVAL;
}

static void bma4xx_convert_raw_accel_to_q31(int16_t raw_val, q31_t *out)
{
	/* The full calculation is (assuming floating math):
	 *   value_ms2 = raw_value * range * 9.8065 / BIT(11)
	 * We can treat 'range * 9.8065' as a scale, the scale is calculated by first getting 1g
	 * represented as a q31 value with the same shift as our result:
	 *   1g = (9.8065 * BIT(31)) >> shift
	 * Next, we need to multiply it by our range in g, which for this driver is one of
	 * [2, 4, 8, 16] and maps to a left shift of [1, 2, 3, 4]:
	 *   1g <<= log2(range)
	 * Note we used a right shift by 'shift' and left shift by log2(range). 'shift' is
	 * [5, 6, 7, 8] for range values [2, 4, 8, 16] since it's the final shift in m/s2. It is
	 * calculated via:
	 *   shift = ceil(log2(range * 9.8065))
	 * This means that we can shorten the above 1g alterations to:
	 *   1g = (1g >> ceil(log2(range * 9.8065))) << log2(range)
	 * For the range values [2, 4, 8, 16], the following is true:
	 *   (x >> ceil(log2(range * 9.8065))) << log2(range)
	 *   = x >> 4
	 * Since the range cancels out in the right and left shift, we've now reduced the following:
	 *   range * 9.8065 = 9.8065 * BIT(31 - 4)
	 * All that's left is to divide by the bma4xx's maximum range BIT(11).
	 */
	const int64_t scale = (int64_t)(9.8065 * BIT64(31 - 4));

	*out = CLAMP(((int64_t)raw_val * scale) >> 11, INT32_MIN, INT32_MAX);
}

#ifdef CONFIG_BMA4XX_TEMPERATURE
/**
 * @brief Convert the 8-bit temp register value into a Q31 celsius value
 */
static void bma4xx_convert_raw_temp_to_q31(int8_t raw_val, q31_t *out)
{
	/* Value of 0 equals 23 degrees C. Each bit count equals 1 degree C */

	int64_t intermediate =
		((int64_t)raw_val + 23) * ((int64_t)INT32_MAX + 1) / (1 << BMA4XX_TEMP_SHIFT);

	*out = CLAMP(intermediate, INT32_MIN, INT32_MAX);
}
#endif /* CONFIG_BMA4XX_TEMPERATURE */

static int bma4xx_one_shot_decode(const uint8_t *buffer, struct sensor_chan_spec ch,
				  uint32_t *fit, uint16_t max_count, void *data_out)
{
	const struct bma4xx_encoded_data *edata = (const struct bma4xx_encoded_data *)buffer;
	const struct bma4xx_decoder_header *header = &edata->header;
	int rc;

	if (*fit != 0) {
		return 0;
	}
	if (max_count == 0 || ch.chan_idx != 0) {
		return -EINVAL;
	}

	switch (ch.chan_type) {
	case SENSOR_CHAN_ACCEL_X:
	case SENSOR_CHAN_ACCEL_Y:
	case SENSOR_CHAN_ACCEL_Z:
	case SENSOR_CHAN_ACCEL_XYZ: {
		if (!edata->has_accel) {
			return -ENODATA;
		}

		struct sensor_three_axis_data *out = (struct sensor_three_axis_data *)data_out;

		out->header.base_timestamp_ns = edata->header.timestamp;
		out->header.reading_count = 1;
		rc = bma4xx_get_shift((struct sensor_chan_spec){.chan_type = SENSOR_CHAN_ACCEL_XYZ,
								.chan_idx = 0},
				      header->accel_fs, &out->shift);
		if (rc != 0) {
			return -EINVAL;
		}

		bma4xx_convert_raw_accel_to_q31(edata->accel_xyz[0], &out->readings[0].x);
		bma4xx_convert_raw_accel_to_q31(edata->accel_xyz[1], &out->readings[0].y);
		bma4xx_convert_raw_accel_to_q31(edata->accel_xyz[2], &out->readings[0].z);

		*fit = 1;
		return 1;
	}
#ifdef CONFIG_BMA4XX_TEMPERATURE
	case SENSOR_CHAN_DIE_TEMP: {
		if (!edata->has_temp) {
			return -ENODATA;
		}

		struct sensor_q31_data *out = (struct sensor_q31_data *)data_out;

		out->header.base_timestamp_ns = edata->header.timestamp;
		out->header.reading_count = 1;
		rc = bma4xx_get_shift((struct sensor_chan_spec){.chan_type = SENSOR_CHAN_DIE_TEMP,
								.chan_idx = 0}, 0, &out->shift);
		if (rc != 0) {
			return -EINVAL;
		}

		bma4xx_convert_raw_temp_to_q31(edata->temp, &out->readings[0].temperature);

		*fit = 1;
		return 1;
	}
#endif /* CONFIG_BMA4XX_TEMPERATURE */
	default:
		return -EINVAL;
	}
}

static int bma4xx_decoder_decode(const uint8_t *buffer, struct sensor_chan_spec ch,
				 uint32_t *fit, uint16_t max_count,
				 void *data_out)
{
	const struct bma4xx_decoder_header *header = (const struct bma4xx_decoder_header *)buffer;

	if (header->is_fifo) {
		/* FIFO (streaming) mode operation is not yet supported */
		return -ENOTSUP;
	}

	return bma4xx_one_shot_decode(buffer, ch, fit, max_count, data_out);
}

SENSOR_DECODER_API_DT_DEFINE() = {
	.get_frame_count = bma4xx_decoder_get_frame_count,
	.get_size_info = bma4xx_decoder_get_size_info,
	.decode = bma4xx_decoder_decode,
};

static int bma4xx_get_decoder(const struct device *dev, const struct sensor_decoder_api **decoder)
{
	ARG_UNUSED(dev);
	*decoder = &SENSOR_DECODER_NAME();

	return 0;
}

int bma4xx_sensor_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	struct bma4xx_data *bma4xx = dev->data;

	int16_t values[3];
	int8_t temperature;
	bma4xx_sample_fetch(dev, &values[0], &values[1], &values[2]);
	bma4xx_temp_fetch(dev, &temperature);

	switch (chan) {
	case SENSOR_CHAN_ACCEL_X:
		bma4xx->fetched_values[0] = values[0];
		return 0;
	case SENSOR_CHAN_ACCEL_Y:
		bma4xx->fetched_values[1] = values[1];
		return 0;
	case SENSOR_CHAN_ACCEL_Z:
		bma4xx->fetched_values[2] = values[2];
		return 0;
	case SENSOR_CHAN_ACCEL_XYZ:
		memcpy(bma4xx->fetched_values, values, sizeof(values));
		return 0;
	case SENSOR_CHAN_DIE_TEMP:
		bma4xx->fetched_temperature = temperature;
		return 0;
	case SENSOR_CHAN_ALL:
		memcpy(bma4xx->fetched_values, values, sizeof(values));
		bma4xx->fetched_temperature = temperature;
		return 0;
	}

	return -EINVAL;
}

void q31_to_sensor_value(q31_t q31_value, struct sensor_value *sensor_val) {
	// Extract the integer part by shifting right by 31 bits
	sensor_val->val1 = (q31_value >> 31);

	// Extract fractional part, scaled to micro-units (val2 is usually in microseconds or microvolts)
	sensor_val->val2 = ((q31_value & 0x7FFFFFFF) * 1000000) >> 31;

	// Handle negative values for fractional part
	if (q31_value < 0 && sensor_val->val2 != 0) {
		sensor_val->val1 -= 1;
		sensor_val->val2 = 1000000 - sensor_val->val2;
	}
}

int bma4xx_sensor_channel_get(const struct device *dev, enum sensor_channel chan,
			 struct sensor_value *val)
{
	struct bma4xx_data *bma4xx = dev->data;

	q31_t q31;

	switch (chan) {
	case SENSOR_CHAN_ACCEL_X:
		bma4xx_convert_raw_accel_to_q31(bma4xx->fetched_values[0], &q31);
		q31_to_sensor_value(q31, val);
		return 0;
	case SENSOR_CHAN_ACCEL_Y:
		bma4xx_convert_raw_accel_to_q31(bma4xx->fetched_values[1], &q31);
		q31_to_sensor_value(q31, val);
		return 0;
	case SENSOR_CHAN_ACCEL_Z:
		bma4xx_convert_raw_accel_to_q31(bma4xx->fetched_values[2], &q31);
		q31_to_sensor_value(q31, val);
		return 0;
	case SENSOR_CHAN_DIE_TEMP:
		val->val1 = bma4xx->fetched_temperature;
		return 0;
	}

	return -EINVAL;
}

/*
 * Sensor driver API
 */

static DEVICE_API(sensor, bma4xx_driver_api) = {
	.attr_get = bma4xx_attr_get,
	.attr_set = bma4xx_attr_set,
	.get_decoder = bma4xx_get_decoder,
	.submit = bma4xx_submit,
	.sample_fetch = bma4xx_sensor_sample_fetch,
	.channel_get = bma4xx_sensor_channel_get,
};

/*
 * Device instantiation macros
 */

#define BMA4XX_RTIO_I2C_DEFINE(inst)                                                               \
	I2C_DT_IODEV_DEFINE(bma4xx_iodev_##inst, DT_DRV_INST(inst));                               \
	RTIO_DEFINE(bma4xx_rtio_##inst, 8, 8);

#define BMA4XX_RTIO_SPI_DEFINE(inst)                                                               \
	SPI_DT_IODEV_DEFINE(bma4xx_iodev_##inst, DT_DRV_INST(inst), 0);                            \
	RTIO_DEFINE(bma4xx_rtio_##inst, 8, 8);

#define BMA4XX_DEFINE_RTIO(inst)                                                                   \
	COND_CODE_1(DT_INST_ON_BUS(inst, spi), (BMA4XX_RTIO_SPI_DEFINE(inst)), \
	(BMA4XX_RTIO_I2C_DEFINE(inst)))

/* Initializes a struct bma4xx_config for an instance on a SPI bus.
 * SPI operation is not currently supported.
 */
#define BMA4XX_CONFIG_SPI(inst)                                                                    \
	.bus_cfg.spi = SPI_DT_SPEC_INST_GET(inst, 0), .bus_init = &bma4xx_spi_init,                \
	.bus_type = BMA4XX_BUS_SPI,

/* Initializes a struct bma4xx_config for an instance on an I2C bus. */
#define BMA4XX_CONFIG_I2C(inst)                                                                    \
	.bus_cfg.i2c = I2C_DT_SPEC_INST_GET(inst), .bus_init = &bma4xx_i2c_init,                   \
	.bus_type = BMA4XX_BUS_I2C,

#define BMA4XX_DEFINE_BUS(inst)                                                                    \
	COND_CODE_1(DT_INST_ON_BUS(inst, spi), (BMA4XX_CONFIG_SPI(inst)), (BMA4XX_CONFIG_I2C(inst)))

#define BMA4XX_DT_CONFIG_INIT(inst)                                                                \
	{                                                                                          \
		.fifo_en = IS_ENABLED(CONFIG_BMA4XX_STREAM),                                       \
		.batch_ticks = 0,                                                                  \
		.interrupt1_fifo_wm = false,                                                       \
		.interrupt1_fifo_full = false,                                                     \
	}

#define BMA4XX_DEFINE_DATA(inst)                                                                   \
	BMA4XX_DEFINE_RTIO(inst);                                                                  \
	static struct bma4xx_data bma4xx_driver_##inst = {                                         \
		.cfg = BMA4XX_DT_CONFIG_INIT(inst),                                                \
		.r = &bma4xx_rtio_##inst,                                                          \
		.iodev = &bma4xx_iodev_##inst,                                                     \
	};

/*
 * Main instantiation macro, which selects the correct bus-specific
 * instantiation macros for the instance.
 */
#define BMA4XX_INIT(inst)                                                                          \
	BMA4XX_DEFINE_DATA(inst);                                                                  \
                                                                                                   \
	static const struct bma4xx_config bma4xx_config_##inst = {                                 \
		BMA4XX_DEFINE_BUS(inst)                                                            \
			IF_ENABLED(CONFIG_BMA4XX_STREAM, ( \
		.gpio_interrupt = GPIO_DT_SPEC_INST_GET_OR(inst, int1_gpios, {0}),))};         \
                                                                                                   \
	SENSOR_DEVICE_DT_INST_DEFINE(inst, bma4xx_chip_init, NULL, &bma4xx_driver_##inst,          \
				     &bma4xx_config_##inst, POST_KERNEL,                           \
				     CONFIG_SENSOR_INIT_PRIORITY, &bma4xx_driver_api);

DT_INST_FOREACH_STATUS_OKAY(BMA4XX_INIT);
