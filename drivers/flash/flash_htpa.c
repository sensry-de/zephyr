/*
 * Copyright (c) 2026 sensryio
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT heimann_htpa_flash

#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/init.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(flash_htpa, CONFIG_FLASH_LOG_LEVEL);

#define HTPA_FLASH_READ_OPCODE 0x03
#define HTPA_FLASH_STATUS_OPCODE 0x05
#define HTPA_FLASH_ENABLE_WRITE 0x06
#define HTPA_FLASH_DISABLE_WRITE 0x06

struct flash_htpa_config {
	struct spi_dt_spec spi;
	size_t size;
#if defined(CONFIG_FLASH_PAGE_LAYOUT)
	struct flash_pages_layout layout;
#endif
};

struct flash_htpa_data {
	int unused;
};

static const struct flash_parameters flash_htpa_parameters = {
	.write_block_size = 1,
	.erase_value = 0xff,
};

static int flash_htpa_set_write_enable(const struct device *dev, bool enable)
{
	const struct flash_htpa_config *config = dev->config;

	uint8_t tx_buffer[1] = {enable?HTPA_FLASH_ENABLE_WRITE:HTPA_FLASH_ENABLE_WRITE};
	uint32_t tx_len = 1;

	/* tx */
	struct spi_buf tx_buf[1];
	tx_buf[0].buf = tx_buffer;
	tx_buf[0].len = tx_len;

	const struct spi_buf_set tx = {.buffers = (struct spi_buf *) &tx_buf, .count = 1};

	/* rx */
	struct spi_buf rx_buf[1];
	rx_buf[0].buf = NULL;
	rx_buf[0].len = 0;

	const struct spi_buf_set rx = {
		.buffers = (struct spi_buf *) &rx_buf,
		.count = ARRAY_SIZE(rx_buf),
	};

	//chip_select_low();

	int ret = spi_transceive_dt(&config->spi, &tx, &rx);
	if (ret) {
		LOG_ERR("SPI read failed: %d", ret);
		return -EIO;
	}

	//chip_select_high();
	return 0;
}

static int flash_read_status(const struct device *dev, uint8_t *status)
{
	const struct flash_htpa_config *config = dev->config;

	uint8_t rx_buffer[1];
	uint32_t rx_len = 1;

	uint8_t tx_buffer[1] = {HTPA_FLASH_STATUS_OPCODE};
	uint32_t tx_len = 1;

	/* tx */
	struct spi_buf tx_buf[1];
	tx_buf[0].buf = tx_buffer;
	tx_buf[0].len = tx_len;

	const struct spi_buf_set tx = {.buffers = (struct spi_buf *) &tx_buf, .count = 1};

	/* rx */
	struct spi_buf rx_buf[2];
	rx_buf[0].buf = NULL;
	rx_buf[0].len = 0;

	rx_buf[1].buf = rx_buffer;
	rx_buf[1].len = rx_len;

	const struct spi_buf_set rx = {
		.buffers = (struct spi_buf *) &rx_buf,
		.count = ARRAY_SIZE(rx_buf),
	};

	//chip_select_low();

	int ret = spi_transceive_dt(&config->spi, &tx, &rx);
	if (ret) {
		LOG_ERR("SPI read failed: %d", ret);
		return -EIO;
	}

	//chip_select_high();
	*status = rx_buffer[0];

	return 0;
}

static bool flash_htpa_valid_range(const struct flash_htpa_config *config, off_t offset,
				   size_t len)
{
	size_t uoffset;

	if (offset < 0) {
		return false;
	}

	uoffset = (size_t)offset;
	if ((uoffset > config->size) || (len > (config->size - uoffset))) {
		return false;
	}

	return true;
}

static int flash_htpa_read(const struct device *dev, off_t offset, void *data, size_t len)
{
#define ADD1(X)   (unsigned int)((X & 0xFF0000)>>16)
#define ADD2(X)   (unsigned int)((X & 0x00FF00)>>8)
#define ADD3(X)   (unsigned int)(X & 0x0000FF)

	const struct flash_htpa_config *config = dev->config;
	uint8_t cmd[4] = {
		0x03,
		ADD1(offset),
		ADD2(offset),
		ADD3(offset)
	};

	const struct spi_buf tx_bufs[] = {
		{
			.buf = cmd,
			.len = sizeof(cmd),
		},
	};
	const struct spi_buf_set tx = {
		.buffers = tx_bufs,
		.count = ARRAY_SIZE(tx_bufs),
	};
	const struct spi_buf rx_bufs[] = {
		{
			.buf = NULL,
			.len = sizeof(cmd),
		},
		{
			.buf = data,
			.len = len,
		},
	};
	const struct spi_buf_set rx = {
		.buffers = rx_bufs,
		.count = ARRAY_SIZE(rx_bufs),
	};
	int ret;

	if (len == 0) {
		return 0;
	}

	if (!flash_htpa_valid_range(config, offset, len)) {
		LOG_ERR("Read outside flash range");
		return -EINVAL;
	}


	ret = spi_transceive_dt(&config->spi, &tx, &rx);
	if (ret < 0) {
		LOG_ERR("Flash read failed: %d", ret);
		return ret;
	}

	return 0;
}

static int flash_htpa_write(const struct device *dev, off_t offset, const void *data, size_t len)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(offset);
	ARG_UNUSED(data);
	ARG_UNUSED(len);

	return -EOPNOTSUPP;
}

static int flash_htpa_erase(const struct device *dev, off_t offset, size_t size)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(offset);
	ARG_UNUSED(size);

	return -EOPNOTSUPP;
}

static const struct flash_parameters *flash_htpa_get_parameters(const struct device *dev)
{
	ARG_UNUSED(dev);

	return &flash_htpa_parameters;
}

static int flash_htpa_get_size(const struct device *dev, uint64_t *size)
{
	const struct flash_htpa_config *config = dev->config;

	*size = config->size;
	return 0;
}

#if defined(CONFIG_FLASH_PAGE_LAYOUT)
static void flash_htpa_page_layout(const struct device *dev,
				   const struct flash_pages_layout **layout,
				   size_t *layout_size)
{
	const struct flash_htpa_config *config = dev->config;

	*layout = &config->layout;
	*layout_size = 1;
}
#endif /* CONFIG_FLASH_PAGE_LAYOUT */

static int flash_htpa_init(const struct device *dev)
{
	const struct flash_htpa_config *config = dev->config;

	if (!spi_is_ready_dt(&config->spi)) {
		LOG_ERR("SPI bus not ready");
		return -ENODEV;
	}

	uint8_t status = 0;
	uint32_t counter = 10;
	do {
		if (0 != flash_read_status(dev, &status)) {
			LOG_ERR("Failed to read status");
			return -EIO;
		}
		k_sleep(K_MSEC(10));
		counter--;
		if (counter == 0) {
			break;
		}
	} while (status & 0x81);

	/* read flash id */
	uint32_t id = 0;
	do {
		if (0 != flash_htpa_read(dev, 0x74, &id, 4)) {
			LOG_ERR("Failed to read flash id");
			return -EIO;
		}
		k_sleep(K_MSEC(10));
	} while ((id == 0) || (id == 0xFFFFFFFF));
	LOG_INF("Flash id: 0x%x", id);

	return 0;
}

static DEVICE_API(flash, flash_htpa_api) = {
	.read = flash_htpa_read,
	.write = flash_htpa_write,
	.erase = flash_htpa_erase,
	.get_parameters = flash_htpa_get_parameters,
	.get_size = flash_htpa_get_size,
#if defined(CONFIG_FLASH_PAGE_LAYOUT)
	.page_layout = flash_htpa_page_layout,
#endif
};

#if defined(CONFIG_FLASH_PAGE_LAYOUT)
#define FLASH_HTPA_LAYOUT_INIT(inst)                                                               \
	.layout = {                                                                                \
		.pages_count = 1,                                                                  \
		.pages_size = DT_INST_PROP(inst, size),                                            \
	},
#else
#define FLASH_HTPA_LAYOUT_INIT(inst)
#endif

#define FLASH_HTPA_INIT(inst)                                                                      \
	static struct flash_htpa_data flash_htpa_data_##inst;                                     \
	static const struct flash_htpa_config flash_htpa_config_##inst = {                         \
		.spi = SPI_DT_SPEC_INST_GET(inst, SPI_OP_MODE_MASTER | SPI_WORD_SET(8), 0),         \
		.size = DT_INST_PROP(inst, size),                                                  \
		FLASH_HTPA_LAYOUT_INIT(inst)                                                       \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(inst, flash_htpa_init, NULL, &flash_htpa_data_##inst,                \
			      &flash_htpa_config_##inst, POST_KERNEL, CONFIG_FLASH_INIT_PRIORITY,  \
			      &flash_htpa_api);

DT_INST_FOREACH_STATUS_OKAY(FLASH_HTPA_INIT)
