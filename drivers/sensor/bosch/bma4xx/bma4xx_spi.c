/* Bosch BMA4xx 3-axis accelerometer driver
 *
 * Copyright (c) 2023 Google LLC
 * Copyright (c) 2024 sensry.io
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT bosch_bma4xx

#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>

#include "bma4xx.h"

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(spi)

LOG_MODULE_DECLARE(bma4xx, CONFIG_SENSOR_LOG_LEVEL);

#define BMA4XX_SPI_READ_BIT  0x80
#define BMA4XX_SPI_WRITE_MSK 0x7f

static int bma4xx_spi_read_data(const struct device *dev, uint8_t reg_addr, uint8_t *value,
				uint8_t len)
{
	const struct bma4xx_config *config = dev->config;
	int err;
	uint8_t addr;
	const struct spi_buf tx_buf = {.buf = &addr, .len = 1};
	const struct spi_buf_set tx = {.buffers = &tx_buf, .count = 1};
	struct spi_buf rx_buf[2] = {
		[0] = {.buf = NULL, .len = 1},
		[1] = {.buf = value, .len = len},
	};
	const struct spi_buf_set rx = {.buffers = rx_buf, .count = ARRAY_SIZE(rx_buf)};
	addr = reg_addr | BMA4XX_SPI_READ_BIT;

	return spi_transceive_dt(&config->bus_cfg.spi, &tx, &rx);
}

static int bma4xx_spi_write_data(const struct device *dev, uint8_t reg_addr, uint8_t *value,
				 uint8_t len)
{
	const struct bma4xx_config *config = dev->config;
	uint8_t cmd[] = {reg_addr & BMA4XX_SPI_WRITE_MSK};
	const struct spi_buf tx_buf[2] = {
		[0] = {.buf = cmd, .len = sizeof(cmd)}, [1] = {.buf = value, .len = len}};
	const struct spi_buf_set tx = {.buffers = &tx_buf[0], .count = ARRAY_SIZE(tx_buf)};

	return spi_write_dt(&config->bus_cfg.spi, &tx);
}

static int bma4xx_spi_read_reg(const struct device *dev, uint8_t reg_addr, uint8_t *value)
{
	return bma4xx_spi_read_data(dev, reg_addr, value, sizeof(*value));
}

static int bma4xx_spi_write_reg(const struct device *dev, uint8_t reg_addr, uint8_t value)
{
	return bma4xx_spi_write_data(dev, reg_addr, &value, sizeof(value));
}

static int bma4xx_spi_update_reg(const struct device *dev, uint8_t reg_addr, uint8_t mask,
				 uint8_t value)
{
	/* read the current value */
	int err;
	uint8_t old_value;

	err = bma4xx_spi_read_reg(dev, reg_addr, &old_value);
	if (err) {
		return -1;
	}

	/* apply mask */
	uint8_t new_value = (old_value & mask) | (value & mask);

	/* write the updated value */
	err = bma4xx_spi_write_reg(dev, reg_addr, new_value);

	return err;
}

static const struct bma4xx_hw_operations spi_ops = {
	.read_data = bma4xx_spi_read_data,
	.write_data = bma4xx_spi_write_data,
	.read_reg = bma4xx_spi_read_reg,
	.write_reg = bma4xx_spi_write_reg,
	.update_reg = bma4xx_spi_update_reg,
};

int bma4xx_spi_init(const struct device *dev)
{
	struct bma4xx_data *data = dev->data;
	const struct bma4xx_config *cfg = dev->config;

	if (!device_is_ready(cfg->bus_cfg.spi.bus)) {
		LOG_ERR("SPI bus device is not ready");
		return -ENODEV;
	}

	data->hw_ops = &spi_ops;

	return 0;
}
#endif /* DT_ANY_INST_ON_BUS_STATUS_OKAY(spi) */
