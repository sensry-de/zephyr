/*
 * Copyright (c) 2023 Google LLC
 * Copyright (c) 2024 Cienet
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_BMA4XX_BMA4XX_H_
#define ZEPHYR_DRIVERS_SENSOR_BMA4XX_BMA4XX_H_

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/gpio.h>
#include "bma4xx_defs.h"

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(spi)
#include <zephyr/drivers/spi.h>
#endif /* DT_ANY_INST_ON_BUS_STATUS_OKAY(spi) */

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c)
#include <zephyr/drivers/i2c.h>
#endif /* DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c) */

/*
 * Register definitions
 */

#define BMA4XX_REG_CHIP_ID         (0x00)
#define BMA4XX_REG_ERROR           (0x02)
#define BMA4XX_REG_STATUS          (0x03)
#define BMA4XX_REG_DATA_0          (0x0A)
#define BMA4XX_REG_DATA_1          (0x0B)
#define BMA4XX_REG_DATA_2          (0x0C)
#define BMA4XX_REG_DATA_3          (0x0D)
#define BMA4XX_REG_DATA_4          (0x0E)
#define BMA4XX_REG_DATA_5          (0x0F)
#define BMA4XX_REG_DATA_6          (0x10)
#define BMA4XX_REG_DATA_7          (0x11)
#define BMA4XX_REG_DATA_8          (0x12)
#define BMA4XX_REG_DATA_9          (0x13)
#define BMA4XX_REG_DATA_10         (0x14)
#define BMA4XX_REG_DATA_11         (0x15)
#define BMA4XX_REG_DATA_12         (0x16)
#define BMA4XX_REG_DATA_13         (0x17)
#define BMA4XX_REG_SENSORTIME_0    (0x18)
#define BMA4XX_REG_INT_STAT_0      (0x1C)
#define BMA4XX_REG_INT_STAT_1      (0x1D)
#define BMA4XX_REG_STEP_CNT_OUT_0  (0x1E)
#define BMA4XX_REG_HIGH_G_OUT      (0x1F)
#define BMA4XX_REG_TEMPERATURE     (0x22)
#define BMA4XX_REG_FIFO_LENGTH_0   (0x24)
#define BMA4XX_REG_FIFO_LENGTH_1   (0x25)
#define BMA4XX_REG_FIFO_DATA       (0x26)
#define BMA4XX_REG_ACTIVITY_OUT    (0x27)
#define BMA4XX_REG_ORIENTATION_OUT (0x28)
#define BMA4XX_REG_ACCEL_CONFIG    (0x40)
#define BMA4XX_REG_ACCEL_RANGE     (0x41)
#define BMA4XX_REG_AUX_CONFIG      (0x44)
#define BMA4XX_REG_FIFO_DOWN       (0x45)
#define BMA4XX_REG_FIFO_WTM_0      (0x46)
#define BMA4XX_REG_FIFO_CONFIG_0   (0x48)
#define BMA4XX_REG_FIFO_CONFIG_1   (0x49)
#define BMA4XX_REG_AUX_DEV_ID      (0x4B)
#define BMA4XX_REG_AUX_IF_CONF     (0x4C)
#define BMA4XX_REG_AUX_RD          (0x4D)
#define BMA4XX_REG_AUX_WR          (0x4E)
#define BMA4XX_REG_AUX_WR_DATA     (0x4F)
#define BMA4XX_REG_INT1_IO_CTRL    (0x53)
#define BMA4XX_REG_INT2_IO_CTRL    (0x54)
#define BMA4XX_REG_INT_LATCH       (0x55)
#define BMA4XX_REG_INT_MAP_1       (0x56)
#define BMA4XX_REG_INT_MAP_2       (0x57)
#define BMA4XX_REG_INT_MAP_DATA    (0x58)
#define BMA4XX_REG_INIT_CTRL       (0x59)
#define BMA4XX_REG_RESERVED_REG_5B (0x5B)
#define BMA4XX_REG_RESERVED_REG_5C (0x5C)
#define BMA4XX_REG_FEATURE_CONFIG  (0x5E)
#define BMA4XX_REG_IF_CONFIG       (0x6B)
#define BMA4XX_REG_ACC_SELF_TEST   (0x6D)
#define BMA4XX_REG_NV_CONFIG       (0x70)
#define BMA4XX_REG_OFFSET_0        (0x71)
#define BMA4XX_REG_OFFSET_1        (0x72)
#define BMA4XX_REG_OFFSET_2        (0x73)
#define BMA4XX_REG_POWER_CONF      (0x7C)
#define BMA4XX_REG_POWER_CTRL      (0x7D)
#define BMA4XX_REG_CMD             (0x7E)

/*
 * Bit positions and masks
 */

#define BMA4XX_BIT_ADV_PWR_SAVE BIT(0)

#define BMA4XX_MASK_ACC_CONF_ODR  GENMASK(3, 0)
#define BMA4XX_MASK_ACC_CONF_BWP  GENMASK(6, 4)
#define BMA4XX_SHIFT_ACC_CONF_BWP (4)

#define BMA4XX_MASK_ACC_RANGE GENMASK(1, 0)

#define BMA4XX_BIT_ACC_PERF_MODE BIT(7)

#define BMA4XX_BIT_ACC_EN BIT(2)

/* INT_STATUS_1 accelerometer data ready to interrupt */
#define BMA4XX_ACC_DRDY_INT BIT(7)

/* CMD: Clears all data in FIFO, does not change FIFO_CONFIG and FIFO_DOWNS register */
#define BMA4XX_CMD_FIFO_FLUSH (0xB0)

/* FIFO_CONFIG_1 enable: Store Accelerometer data in FIFO (all 3 axes) */
#define BMA4XX_FIFO_ACC_EN BIT(6)

/* Bandwidth parameters */
#define BMA4XX_BWP_OSR4_AVG1  (0x0)
#define BMA4XX_BWP_OSR2_AVG2  (0x1)
#define BMA4XX_BWP_NORM_AVG4  (0x2)
#define BMA4XX_BWP_CIC_AVG8   (0x3)
#define BMA4XX_BWP_RES_AVG16  (0x4)
#define BMA4XX_BWP_RES_AVG32  (0x5)
#define BMA4XX_BWP_RES_AVG64  (0x6)
#define BMA4XX_BWP_RES_AVG128 (0x7)

/* Full-scale ranges */
#define BMA4XX_RANGE_2G  (0x0)
#define BMA4XX_RANGE_4G  (0x1)
#define BMA4XX_RANGE_8G  (0x2)
#define BMA4XX_RANGE_16G (0x3)

/* Output data rates (ODR) */
#define BMA4XX_ODR_RESERVED (0x00)
#define BMA4XX_ODR_0_78125  (0x01)
#define BMA4XX_ODR_1_5625   (0x02)
#define BMA4XX_ODR_3_125    (0x03)
#define BMA4XX_ODR_6_25     (0x04)
#define BMA4XX_ODR_12_5     (0x05)
#define BMA4XX_ODR_25       (0x06)
#define BMA4XX_ODR_50       (0x07)
#define BMA4XX_ODR_100      (0x08)
#define BMA4XX_ODR_200      (0x09)
#define BMA4XX_ODR_400      (0x0a)
#define BMA4XX_ODR_800      (0x0b)
#define BMA4XX_ODR_1600     (0x0c)
#define BMA4XX_ODR_3200     (0x0d)
#define BMA4XX_ODR_6400     (0x0e)
#define BMA4XX_ODR_12800    (0x0f)

/*
 * BMA4xx commands
 */

#define BMA4XX_CMD_SOFT_RESET (0xB6)

#define BMA4XX_CHIP_ID_BMA422 (0x12)
#define BMA4XX_CHIP_ID_BMA423 (0x13)

/*
 * Other constants
 */

/* Each bit count is 3.9mG or 3900uG */
#define BMA4XX_OFFSET_MICROG_PER_BIT (3900)
#define BMA4XX_OFFSET_MICROG_MIN     (INT8_MIN * BMA4XX_OFFSET_MICROG_PER_BIT)
#define BMA4XX_OFFSET_MICROG_MAX     (INT8_MAX * BMA4XX_OFFSET_MICROG_PER_BIT)

/* Range is -104C to 150C. Use shift of 8 (+/-256) */
#define BMA4XX_TEMP_SHIFT (8)

/* The total number of used registers specified in bma422 datasheet is 7E */
#define BMA4XX_NUM_REGS 0x7E

/*
 * Types
 */

#define BMA4XX_BUS_I2C 0
#define BMA4XX_BUS_SPI 1

union bma4xx_bus_cfg {
#if DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c)
	struct i2c_dt_spec i2c;
#endif /* DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c) */

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(spi)
	struct spi_dt_spec spi;
#endif /* DT_ANY_INST_ON_BUS_STATUS_OKAY(spi) */
};

struct bma4xx_config {
	int (*bus_init)(const struct device *dev);
	const union bma4xx_bus_cfg bus_cfg;
	uint8_t bus_type;

	const struct gpio_dt_spec gpio_interrupt;
};

/** Used to implement bus-specific R/W operations. See bma4xx_i2c.c and
 *  bma4xx_spi.c.
 */
struct bma4xx_hw_operations {
	int (*read_data)(const struct device *dev, uint8_t reg_addr, uint8_t *value, uint8_t len);
	int (*write_data)(const struct device *dev, uint8_t reg_addr, uint8_t *value, uint8_t len);
	int (*read_reg)(const struct device *dev, uint8_t reg_addr, uint8_t *value);
	int (*write_reg)(const struct device *dev, uint8_t reg_addr, uint8_t value);
	int (*update_reg)(const struct device *dev, uint8_t reg_addr, uint8_t mask, uint8_t value);
};

struct bma4xx_runtime_config {
	bool fifo_en;
	int32_t batch_ticks;

	bool interrupt1_fifo_wm;
	bool interrupt1_fifo_full;

	uint8_t accel_pwr_mode;
	uint8_t aux_pwr_mode;

	/** Current values that are requested via the sensor fetch API */
	int16_t fetched_values[3];
	int8_t fetched_temperature;
	/** Current full-scale range setting as a register value */
	uint8_t accel_fs_range;
	/** Current bandwidth parameter (BWP) as a register value */
	uint8_t accel_bwp;
	/** Current output data rate as a register value */
	uint8_t accel_odr;
};

struct bma4xx_data {
	struct bma4xx_runtime_config cfg;
	/** Pointer to bus-specific I/O API */
	const struct bma4xx_hw_operations *hw_ops;
	/** Chip ID value stored in BMA4XX_REG_CHIP_ID */
	uint8_t chip_id;
	struct rtio *r;
	struct rtio_iodev *iodev;
#ifdef CONFIG_BMA4XX_STREAM
	struct rtio_iodev_sqe *streaming_sqe;
	uint8_t int_status;
	uint16_t fifo_count;
	uint64_t timestamp;
	const struct device *dev;
	struct gpio_callback gpio_cb;
#endif /* CONFIG_BMA4XX_STREAM */
};

static inline int bma4xx_accel_reg_to_hz(uint8_t odr, struct sensor_value *out)
{
	static const struct sensor_value odr_values[] = {
		{0, 0},    {0, 781250}, {1, 562500}, {3, 125000}, {6, 250000}, {12, 500000},
		{25, 0},   {50, 0},     {100, 0},    {200, 0},    {400, 0},    {800, 0},
		{1600, 0}, {3200, 0},   {6400, 0},   {12800, 0},
	};

	if (odr >= ARRAY_SIZE(odr_values)) {
		return -EINVAL;
	}

	*out = odr_values[odr];

	return 0;
}

/**
 * @brief Initialize the SPI bus
 *
 * @param dev bma4xx device pointer
 *
 * @retval 0 success
 * @retval -errdev Error
 */
int bma4xx_spi_init(const struct device *dev);

/**
 * @brief Initialize the I2C bus
 *
 * @param dev bma4xx device pointer
 *
 * @retval 0 success
 * @retval -errdev Error
 */
int bma4xx_i2c_init(const struct device *dev);

/**
 * @brief (Re)Configure the sensor with the given configuration
 *
 * @param dev bma4xx device pointer
 * @param cfg bma4xx_runtime_config pointer
 *
 * @retval 0 success
 * @retval -errno Error
 */
int bma4xx_configure(const struct device *dev, struct bma4xx_runtime_config *cfg);

/**
 * @brief Safely (re)Configure the sensor with the given configuration
 *
 * Will rollback to prior configuration if new configuration is invalid
 *
 * @param dev bma4xx device pointer
 * @param cfg bma4xx_runtime_config pointer
 *
 * @retval 0 success
 * @retval -errno Error
 */
int bma4xx_safely_configure(const struct device *dev, struct bma4xx_runtime_config *cfg);

#endif /* ZEPHYR_DRIVERS_SENSOR_BMA4XX_BMA4XX_H_ */
