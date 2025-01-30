/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2024 sensry.io
 */

#define DT_DRV_COMPAT sensry_sy1xx_spi

#define LOG_LEVEL CONFIG_SPI_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(spi_sy1xx);

#include "spi_context.h"
#include <errno.h>
#include <zephyr/spinlock.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/sys/util.h>
#include <udma.h>
#include <zephyr/drivers/pinctrl.h>

struct sy1xx_spi_config {
	uint32_t base;
	uint32_t inst;
	int32_t cs_pin;

	uint8_t cpol;
	uint8_t cpha;
	uint8_t div;
	uint8_t quad_spi;

	const struct pinctrl_dev_config *pcfg;
};

#define SY1XX_SPI_MAX_BIT_COUNT   (12800)
#define SY1XX_SPI_MAX_BUFFER_SIZE (SY1XX_SPI_MAX_BIT_COUNT / 8)

struct sy1xx_spi_data {
	struct spi_context ctx;
	struct k_spinlock lock;

	uint8_t write[SY1XX_SPI_MAX_BUFFER_SIZE / 2];
	uint8_t read[SY1XX_SPI_MAX_BUFFER_SIZE];
};

#define SPI_CMD_OFFSET (4)

/* Commands for SPI UDMA */
#define SPI_CMD_CFG       (0 << SPI_CMD_OFFSET)
#define SPI_CMD_SOT       (1 << SPI_CMD_OFFSET)
#define SPI_CMD_SEND_CMD  (2 << SPI_CMD_OFFSET)
#define SPI_CMD_SEND_ADDR (3 << SPI_CMD_OFFSET)
#define SPI_CMD_DUMMY     (4 << SPI_CMD_OFFSET)
#define SPI_CMD_WAIT      (5 << SPI_CMD_OFFSET)
#define SPI_CMD_TX_DATA   (6 << SPI_CMD_OFFSET)
#define SPI_CMD_RX_DATA   (7 << SPI_CMD_OFFSET)
#define SPI_CMD_RPT       (8 << SPI_CMD_OFFSET)
#define SPI_CMD_EOT       (9 << SPI_CMD_OFFSET)
#define SPI_CMD_RPT_END   (10 << SPI_CMD_OFFSET)
#define SPI_CMD_RX_CHECK  (11 << SPI_CMD_OFFSET)
#define SPI_CMD_FULL_DPLX (12 << SPI_CMD_OFFSET)

/* CMD CFG */
#define SPI_CMD_CFG_0()           (SPI_CMD_CFG)
#define SPI_CMD_CFG_1()           (0)
#define SPI_CMD_CFG_2(cpol, cpha) (((cpol & 0x1) << 1) | ((cpha & 0x1) << 0))
#define SPI_CMD_CFG_3(div)        (div & 0xff)

/* CMD SOT */
#define SPI_CMD_SOT_0()   (SPI_CMD_SOT)
#define SPI_CMD_SOT_1()   (0)
#define SPI_CMD_SOT_2()   (0)
#define SPI_CMD_SOT_3(cs) (cs & 0x1)

/* CMD SEND_ADDR */
#define SPI_CMD_SEND_ADDR0(qspi)     (SPI_CMD_SEND_ADDR | ((qspi & 0x1) << 3))
#define SPI_CMD_SEND_ADDR1(num_bits) ((num_bits - 1) & 0x1f)
#define SPI_CMD_SEND_ADDR2()         (0)
#define SPI_CMD_SEND_ADDR3()         (0)

/* CMD SEND_DATA */
#define SPI_CMD_SEND_DATA0(qspi)     (SPI_CMD_TX_DATA | ((qspi & 0x1) << 3))
#define SPI_CMD_SEND_DATA1()         (0)
#define SPI_CMD_SEND_DATA2(num_bits) (UINT16_BYTE0((num_bits - 1)) & 0xffff)
#define SPI_CMD_SEND_DATA3(num_bits) (UINT16_BYTE1((num_bits - 1)) & 0xffff)

/* CMD READ_DATA */
#define SPI_CMD_READ_DATA0(qspi, align)                                                            \
	(SPI_CMD_RX_DATA | ((qspi & 0x1) << 3) | ((align & 0x3) << 1))
#define SPI_CMD_READ_DATA1()         (0)
#define SPI_CMD_READ_DATA2(num_bits) (UINT16_BYTE0((num_bits - 1)) & 0xffff)
#define SPI_CMD_READ_DATA3(num_bits) (UINT16_BYTE1((num_bits - 1)) & 0xffff)

/* CMD READ_DATA */
#define SPI_CMD_FULL_DPLX_DATA0(qspi, align)                                                       \
	(SPI_CMD_FULL_DPLX | ((qspi & 0x1) << 3) | ((align & 0x3) << 1))
#define SPI_CMD_FULL_DPLX_DATA1()         (0)
#define SPI_CMD_FULL_DPLX_DATA2(num_bits) (UINT16_BYTE0((num_bits - 1)) & 0xffff)
#define SPI_CMD_FULL_DPLX_DATA3(num_bits) (UINT16_BYTE1((num_bits - 1)) & 0xffff)

/* CMD EOT */
#define SPI_CMD_EOT0()    (SPI_CMD_EOT)
#define SPI_CMD_EOT1()    (0)
#define SPI_CMD_EOT2()    (0)
#define SPI_CMD_EOT3(evt) (evt & 0x1)

#define UINT32_BYTE0(uint32) ((uint32 >> 24) & 0xff)
#define UINT32_BYTE1(uint32) ((uint32 >> 16) & 0xff)
#define UINT32_BYTE2(uint32) ((uint32 >> 8) & 0xff)
#define UINT32_BYTE3(uint32) ((uint32 >> 0) & 0xff)

#define UINT16_BYTE0(uint16) ((uint16 >> 8) & 0xff)
#define UINT16_BYTE1(uint16) ((uint16 >> 0) & 0xff)

#define ROUND_UP_DIV_4(x) ((x + 3) / 4)

static void sy1xx_spi_udma_reset(const struct device *dev)
{
	struct sy1xx_spi_config *config = (struct sy1xx_spi_config *)dev->config;

	SY1XX_UDMA_CANCEL_RX(config->base);
	SY1XX_UDMA_CANCEL_TX(config->base);

	SY1XX_UDMA_WAIT_FOR_FINISHED_TX(config->base);
	SY1XX_UDMA_WAIT_FOR_FINISHED_RX(config->base);
}

static int sy1xx_spi_init(const struct device *dev)
{

	struct sy1xx_spi_config *config = (struct sy1xx_spi_config *)dev->config;
	struct sy1xx_spi_data *data = (struct sy1xx_spi_data *)dev->data;
	int32_t ret;

	for (uint32_t i = 0; i < SY1XX_SPI_MAX_BUFFER_SIZE; i++) {
		data->write[i] = 0x22;
		data->read[i] = 0x44;
	}

	/* UDMA clock enable */
	sy1xx_udma_enable_clock(SY1XX_UDMA_MODULE_SPI, config->inst);

	/* PAD config */
	ret = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		LOG_ERR("SPI failed to set pin config for %u", config->inst);
		return ret;
	}

	uint32_t spi_freq = 8000000;
	uint32_t div = sy1xx_soc_get_peripheral_clock() / spi_freq;

	config->cpol = 0;
	config->cpha = 0;
	config->div = div;
	config->quad_spi = 0;

	sy1xx_spi_udma_reset(dev);

	return 0;
}

void print_hex(uint8_t *buf, uint32_t len)
{
	for (uint32_t i = 0; i < len; i++) {
		if ((i % 4) == 0) {
			printf("\t");
		}
		printf("%02x ", buf[i]);
	}
	printf("\n");
}

static int32_t sy1xx_spi_full_duplex_transfer(const struct device *dev, uint8_t *tx_buf,
					      uint8_t *rx_buf, uint32_t xfer_len)
{
	struct sy1xx_spi_config *config = (struct sy1xx_spi_config *)dev->config;
	struct sy1xx_spi_data *data = (struct sy1xx_spi_data *)dev->data;

	printf("SPI transfer %d bytes\n", xfer_len);

	if (xfer_len == 0) {
		return -EINVAL;
	}

	uint8_t *cmd_buf = &data->write[0];
	uint32_t count = 0;

	/* expected data config (bitlen in bits) and spi transfer type */
	cmd_buf[count++] = SPI_CMD_FULL_DPLX_DATA3(xfer_len * 8);
	cmd_buf[count++] = SPI_CMD_FULL_DPLX_DATA2(xfer_len * 8);
	cmd_buf[count++] = SPI_CMD_FULL_DPLX_DATA1();
	cmd_buf[count++] = SPI_CMD_FULL_DPLX_DATA0(config->quad_spi, 0);

	/* data; we need to fill multiple of 32bit size */
	uint32_t padded_len = ROUND_UP_DIV_4(xfer_len) * 4;

	for (uint32_t i = 0; i < padded_len; i++) {
		cmd_buf[count + i] = (i < xfer_len) ? tx_buf[i] : 0xff;
	}

	count += padded_len;

	SY1XX_UDMA_START_RX(config->base, (uint32_t)data->read, xfer_len,
			    SY1XX_UDMA_RX_DATA_ADDR_INC_SIZE_32);

	SY1XX_UDMA_START_TX(config->base, (uint32_t)cmd_buf, count, 0);

	SY1XX_UDMA_WAIT_FOR_FINISHED_TX(config->base);
	SY1XX_UDMA_WAIT_FOR_FINISHED_RX(config->base);

	int32_t transferred_count = (int32_t)xfer_len - SY1XX_UDMA_GET_REMAINING_RX(config->base);

	if ((transferred_count > 0) && (transferred_count <= xfer_len)) {
		memcpy(rx_buf, &data->read[0], transferred_count);
		printf("TX>");
		print_hex(cmd_buf, count);
		printf("RX>");
		print_hex(rx_buf, transferred_count);
		return transferred_count;
	}

	return 0;
}

static int32_t sy1xx_spi_set_cs(const struct device *dev)
{
	struct sy1xx_spi_config *config = (struct sy1xx_spi_config *)dev->config;
	struct sy1xx_spi_data *data = (struct sy1xx_spi_data *)dev->data;

	LOG_DBG("SPI set CS%d", config->cs_pin);

	uint8_t *cmd_buf = &data->write[0];
	uint32_t count = 0;

	/* stop any prior transmission */
	SY1XX_UDMA_WAIT_FOR_FINISHED_TX(config->base);
	SY1XX_UDMA_WAIT_FOR_FINISHED_RX(config->base);

	/* prepare spi cfg */
	cmd_buf[count++] = SPI_CMD_CFG_3(config->div);
	cmd_buf[count++] = SPI_CMD_CFG_2(config->cpol, config->cpha);
	cmd_buf[count++] = SPI_CMD_CFG_1();
	cmd_buf[count++] = SPI_CMD_CFG_0();

	/* check if spi controller chip select is configured */
	if (config->cs_pin >= 0) {
		/* start with selecting hardware chip-select */
		cmd_buf[count++] = SPI_CMD_SOT_3(config->cs_pin);
		cmd_buf[count++] = SPI_CMD_SOT_2();
		cmd_buf[count++] = SPI_CMD_SOT_1();
		cmd_buf[count++] = SPI_CMD_SOT_0();
	}

	/* enable gpio cs (if configured) */
	spi_context_cs_control(&data->ctx, true);

	/* transfer configuration via udma to spi controller */
	SY1XX_UDMA_START_TX(config->base, (uint32_t)cmd_buf, count, 0);
	SY1XX_UDMA_WAIT_FOR_FINISHED_TX(config->base);

	int32_t xfer_count = (int32_t)count - SY1XX_UDMA_GET_REMAINING_TX(config->base);
	if (xfer_count != count) {
		return -EINVAL;
	}
	return 0;
}

static int32_t sy1xx_spi_reset_cs(const struct device *dev)
{
	struct sy1xx_spi_config *config = (struct sy1xx_spi_config *)dev->config;
	struct sy1xx_spi_data *data = (struct sy1xx_spi_data *)dev->data;

	LOG_DBG("SPI reset CS");

	/* reset gpio chip select */
	spi_context_cs_control(&data->ctx, false);

	uint8_t *cmd_buf = &data->write[0];
	uint32_t count = 0;

	/* prepare end of transmission (includes resetting any enabled chip selects) */
	cmd_buf[count++] = SPI_CMD_EOT3(0);
	cmd_buf[count++] = SPI_CMD_EOT2();
	cmd_buf[count++] = SPI_CMD_EOT1();
	cmd_buf[count++] = SPI_CMD_EOT0();

	SY1XX_UDMA_START_TX(config->base, (uint32_t)cmd_buf, count, 0);
	SY1XX_UDMA_WAIT_FOR_FINISHED_TX(config->base);

	int32_t xfer_count = (int32_t)count - SY1XX_UDMA_GET_REMAINING_TX(config->base);
	if (xfer_count != count) {
		return -EINVAL;
	}
	return 0;
}

/**
 * @return:
 *  - < 0: Error
 *  - = 0: No bytes received
 *  - > 0: Number of bytes received
 */
static int32_t sy1xx_spi_half_duplex_read(const struct device *dev, uint8_t *rx_buf,
					  uint32_t rx_len)
{

	struct sy1xx_spi_config *config = (struct sy1xx_spi_config *)dev->config;
	struct sy1xx_spi_data *data = (struct sy1xx_spi_data *)dev->data;

	LOG_DBG("SPI read %d bytes", rx_len);

	uint8_t *cmd_buf = &data->write[0];
	uint32_t count = 0;

	/* expected data config */
	cmd_buf[count++] = SPI_CMD_READ_DATA3(rx_len * 8);
	cmd_buf[count++] = SPI_CMD_READ_DATA2(rx_len * 8);
	cmd_buf[count++] = SPI_CMD_READ_DATA1();
	cmd_buf[count++] = SPI_CMD_READ_DATA0(0, 0);

	SY1XX_UDMA_START_RX(config->base, (uint32_t)data->read, rx_len,
			    SY1XX_UDMA_RX_DATA_ADDR_INC_SIZE_32);
	SY1XX_UDMA_START_TX(config->base, (uint32_t)cmd_buf, count, 0);

	SY1XX_UDMA_WAIT_FOR_FINISHED_TX(config->base);
	SY1XX_UDMA_WAIT_FOR_FINISHED_RX(config->base);

	int32_t xfer_count = (int32_t)rx_len - SY1XX_UDMA_GET_REMAINING_RX(config->base);

	if ((xfer_count > 0) && (xfer_count <= rx_len)) {
		memcpy(rx_buf, &data->read[0], xfer_count);
#if DEBUG_OUT
		printf("RX>");
		print_hex(rx_buf, xfer_count);
#endif
		return xfer_count;
	}

	return -EINVAL;
}

/**
 * @return:
 *  - < 0: Error
 *  - = 0: No bytes written
 *  - > 0: Number of bytes written
 */
static int32_t sy1xx_spi_half_duplex_write(const struct device *dev, uint8_t *tx_buf,
					   uint32_t tx_len)
{

	struct sy1xx_spi_config *config = (struct sy1xx_spi_config *)dev->config;
	struct sy1xx_spi_data *data = (struct sy1xx_spi_data *)dev->data;

	LOG_DBG("SPI write %d bytes", tx_len);

	uint8_t *cmd_buf = &data->write[0];
	uint32_t count = 0;

	/* expected data config and spi transfer type */
	cmd_buf[count++] = SPI_CMD_SEND_DATA3(tx_len * 8);
	cmd_buf[count++] = SPI_CMD_SEND_DATA2(tx_len * 8);
	cmd_buf[count++] = SPI_CMD_SEND_DATA1();
	cmd_buf[count++] = SPI_CMD_SEND_DATA0(config->quad_spi);

	/* data; we need to fill multiple of 32bit size */
	uint32_t padded_len = ROUND_UP_DIV_4(tx_len) * 4;

	for (uint32_t i = 0; i < padded_len; i++) {
		cmd_buf[count + i] = (i < tx_len) ? tx_buf[i] : 0xff;
	}

	count += padded_len;

	SY1XX_UDMA_START_TX(config->base, (uint32_t)cmd_buf, count, 0);

	SY1XX_UDMA_WAIT_FOR_FINISHED_TX(config->base);

	int32_t xfer_count = (int32_t)tx_len - SY1XX_UDMA_GET_REMAINING_TX(config->base);

	if ((xfer_count > 0) && (xfer_count <= tx_len)) {
#if DEBUG_OUT
		printf("TX>");
		print_hex(cmd_buf, count);
#endif
		return xfer_count;
	}

	return -EINVAL;
}

/**
 * the problem with transfer function is, that it comes in
 * multiple chunks. where we actually want to use DMA to transfer all address + data
 * at once.
 *
 * transfers could be,
 * - read (data)
 * - write (data)
 * - write (addr), read (data)
 * - write (addr), write (data)
 *
 * DMA can distinguish between address and data writing, so we
 * have to de-puzzle the addr and data from tx_bufs and rx_bufs.
 *
 * @return
 *  - < 0: Error
 *  -   0: OK
 */
static int sy1xx_spi_transceive_sync(const struct device *dev, const struct spi_config *config,
				     const struct spi_buf_set *tx_bufs,
				     const struct spi_buf_set *rx_bufs)
{
	struct sy1xx_spi_data *data = (struct sy1xx_spi_data *)dev->data;

	int ret = 0;
	size_t tx_count = 0;
	size_t rx_count = 0;
	const struct spi_buf *tx = NULL;
	const struct spi_buf *rx = NULL;

	if (tx_bufs) {
		tx = tx_bufs->buffers;
		tx_count = tx_bufs->count;
	}

	if (rx_bufs) {
		rx = rx_bufs->buffers;
		rx_count = rx_bufs->count;
	}

	spi_context_lock(&data->ctx, false, NULL, NULL, config);

	ret = sy1xx_spi_set_cs(dev);
	if (ret) {
		LOG_ERR("SPI set cs failed");
		goto done;
	}

	/* handle symmetrical tx and rx transfers */
	while (tx_count != 0 && rx_count != 0) {
		if (tx->buf == NULL) {
			ret = sy1xx_spi_half_duplex_read(dev, rx->buf, rx->len);
		} else if (rx->buf == NULL) {
			ret = sy1xx_spi_half_duplex_write(dev, tx->buf, tx->len);
		} else if (rx->len == tx->len) {
			ret = sy1xx_spi_full_duplex_transfer(dev, tx->buf, rx->buf, rx->len);
		} else {
			__ASSERT_NO_MSG("Invalid SPI transfer configuration");
		}

		if (ret < 0) {
			LOG_ERR("Failed to transfer data");
			goto done;
		}

		tx++;
		tx_count--;
		rx++;
		rx_count--;
	}

	/* handle the left-overs for tx only */
	for (; tx_count != 0; tx_count--) {
		ret = sy1xx_spi_half_duplex_write(dev, tx->buf, tx->len);
		if (ret < 0) {
			LOG_ERR("Failed to transfer data");
			goto done;
		}
		tx++;
	}

	/* handle the left-overs for rx only */
	for (; rx_count != 0; rx_count--) {
		ret = sy1xx_spi_half_duplex_read(dev, rx->buf, rx->len);
		if (ret < 0) {
			LOG_ERR("Failed to transfer data");
			goto done;
		}
		rx++;
	}

done:
	sy1xx_spi_reset_cs(dev);

	spi_context_release(&data->ctx, ret);

	return ret;
}

static int sy1xx_spi_release(const struct device *dev, const struct spi_config *config)
{
	/* the uDMA is releasing after transfer automatically */
	return 0;
}

static const struct spi_driver_api sy1xx_spi_driver_api = {
	.transceive = sy1xx_spi_transceive_sync,
	.release = sy1xx_spi_release,
};

#define SPI_SY1XX_DEVICE_INIT(n)                                                                   \
                                                                                                   \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
                                                                                                   \
	static struct sy1xx_spi_config sy1xx_spi_dev_config_##n = {                                \
		.base = (uint32_t)DT_INST_REG_ADDR(n),                                             \
		.inst = (uint32_t)DT_INST_PROP(n, instance),                                       \
		.cs_pin = DT_PROP_OR(DT_NODELABEL(n), cs_pin, -1),                                 \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                         \
	};                                                                                         \
                                                                                                   \
	static struct sy1xx_spi_data __attribute__((section(".udma_access")))                      \
	__aligned(4) sy1xx_spi_dev_data_##n = {                                                    \
		SPI_CONTEXT_INIT_LOCK(sy1xx_spi_dev_data_##n, ctx),                                \
		SPI_CONTEXT_INIT_SYNC(sy1xx_spi_dev_data_##n, ctx),                                \
		SPI_CONTEXT_CS_GPIOS_INITIALIZE(DT_DRV_INST(n), ctx)};                             \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, &sy1xx_spi_init, NULL, &sy1xx_spi_dev_data_##n,                   \
			      &sy1xx_spi_dev_config_##n, POST_KERNEL, CONFIG_SPI_INIT_PRIORITY,    \
			      &sy1xx_spi_driver_api);

DT_INST_FOREACH_STATUS_OKAY(SPI_SY1XX_DEVICE_INIT)
