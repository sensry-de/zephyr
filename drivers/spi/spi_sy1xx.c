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
#include <pad_ctrl.h>

struct sy1xx_spi_config {
	uint32_t base;
	uint32_t inst;

	uint8_t cpol;
	uint8_t cpha;
	uint8_t div;
	uint8_t byteAlign;
	uint8_t quad_spi;
};

#define DEVICE_MAX_BUFFER_SIZE (512)

struct sy1xx_spi_data {
	struct spi_context ctx;
	struct k_spinlock lock;

	uint8_t write[DEVICE_MAX_BUFFER_SIZE];
	uint8_t read[DEVICE_MAX_BUFFER_SIZE];
};

typedef struct {
	uint8_t cs;

	uint32_t addr_width;
	uint32_t addr;

	uint16_t data_len;
	uint8_t *data;

} spiTransfer_t;

#define SPI_CMD_OFFSET 4

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

// CMD CFG
#define SPI_CMD_CFG_0()           (SPI_CMD_CFG)
#define SPI_CMD_CFG_1()           (0)
#define SPI_CMD_CFG_2(cpol, cpha) (((cpol & 0x1) << 1) | ((cpha & 0x1) << 0))
#define SPI_CMD_CFG_3(div)        (div & 0xff)

// CMD SOT
#define SPI_CMD_SOT_0()   (SPI_CMD_SOT)
#define SPI_CMD_SOT_1()   (0)
#define SPI_CMD_SOT_2()   (0)
#define SPI_CMD_SOT_3(cs) (cs & 0x1)

// CMD SEND_ADDR
#define SPI_CMD_SEND_ADDR0(qspi)     (SPI_CMD_SEND_ADDR | ((qspi & 0x1) << 3))
#define SPI_CMD_SEND_ADDR1(num_bits) ((num_bits - 1) & 0x1f)
#define SPI_CMD_SEND_ADDR2()         (0)
#define SPI_CMD_SEND_ADDR3()         (0)

// CMD SEND_DATA
#define SPI_CMD_SEND_DATA0(qspi)     (SPI_CMD_TX_DATA | ((qspi & 0x1) << 3))
#define SPI_CMD_SEND_DATA1()         (0)
#define SPI_CMD_SEND_DATA2(num_bits) (UINT16_BYTE0((num_bits - 1)) & 0xffff)
#define SPI_CMD_SEND_DATA3(num_bits) (UINT16_BYTE1((num_bits - 1)) & 0xffff)

// CMD READ_DATA
#define SPI_CMD_READ_DATA0(qspi, align)                                                            \
	(SPI_CMD_RX_DATA | ((qspi & 0x1) << 3) | ((align & 0x3) << 1))
#define SPI_CMD_READ_DATA1()         (0)
#define SPI_CMD_READ_DATA2(num_bits) (UINT16_BYTE0((num_bits - 1)) & 0xffff)
#define SPI_CMD_READ_DATA3(num_bits) (UINT16_BYTE1((num_bits - 1)) & 0xffff)

// CMD EOT
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

	UDMA_CANCEL_RX(config->base);
	UDMA_CANCEL_TX(config->base);

	UDMA_WAIT_FOR_FINISHED_TX(config->base);
	UDMA_WAIT_FOR_FINISHED_RX(config->base);
}

static int sy1xx_spi_init(const struct device *dev)
{

	struct sy1xx_spi_config *config = (struct sy1xx_spi_config *)dev->config;
	struct sy1xx_spi_data *data = (struct sy1xx_spi_data *)dev->data;

	for (uint32_t i = 0; i < DEVICE_MAX_BUFFER_SIZE; i++) {
		data->write[i] = 0x22;
		data->read[i] = 0x44;
	}

	// UDMA clock enable
	drivers_udma_enable_clock(DRIVERS_UDMA_SPI, config->inst);

	// PAD config
	uint32_t pad_config_cs1 =
		PAD_CONFIG(0, PAD_SMT_DISABLE, PAD_SLEW_LOW, PAD_PULLUP_EN, PAD_PULLDOWN_DIS,
			   PAD_DRIVE_2PF, PAD_PMOD_NORMAL, PAD_DIR_OUTPUT);

	uint32_t pad_config_cs0 =
		PAD_CONFIG(8, PAD_SMT_DISABLE, PAD_SLEW_LOW, PAD_PULLUP_EN, PAD_PULLDOWN_DIS,
			   PAD_DRIVE_2PF, PAD_PMOD_NORMAL, PAD_DIR_OUTPUT);

	uint32_t pad_config_miso =
		PAD_CONFIG(16, PAD_SMT_DISABLE, PAD_SLEW_LOW, PAD_PULLUP_DIS, PAD_PULLDOWN_DIS,
			   PAD_DRIVE_2PF, PAD_PMOD_NORMAL, PAD_DIR_INPUT);

	uint32_t pad_config_mosi =
		PAD_CONFIG(24, PAD_SMT_DISABLE, PAD_SLEW_LOW, PAD_PULLUP_DIS, PAD_PULLDOWN_DIS,
			   PAD_DRIVE_2PF, PAD_PMOD_NORMAL, PAD_DIR_OUTPUT);

	uint32_t pad_config_clk =
		PAD_CONFIG(0, PAD_SMT_DISABLE, PAD_SLEW_LOW, PAD_PULLUP_DIS, PAD_PULLDOWN_DIS,
			   PAD_DRIVE_2PF, PAD_PMOD_NORMAL, PAD_DIR_OUTPUT);

	// 2 registers for pad config
	sys_write32((pad_config_cs1 | pad_config_cs0 | pad_config_miso | pad_config_mosi),
		    PAD_CONFIG_ADDR_SPI + (config->inst * 8 + 0));
	sys_write32((pad_config_clk), PAD_CONFIG_ADDR_SPI + (config->inst * 8 + 4));

	config->cpol = 0;
	config->cpha = 0;
	config->div = 0xf0;
	config->quad_spi = 0;

	config->byteAlign = 0;

	sy1xx_spi_udma_reset(dev);

	printk("spi%d init done\n", config->inst);

	return 0;
}

static int32_t sy1xx_spi_write(const struct device *dev, spiTransfer_t *request)
{

	struct sy1xx_spi_config *config = (struct sy1xx_spi_config *)dev->config;
	struct sy1xx_spi_data *data = (struct sy1xx_spi_data *)dev->data;

	if (request->data_len == 0) {
		return -2;
	}

	if (request->data_len > DEVICE_MAX_BUFFER_SIZE) {
		return -3;
	}

	// stop any prior transmission
	UDMA_WAIT_FOR_FINISHED_TX(config->base);
	UDMA_WAIT_FOR_FINISHED_RX(config->base);

	uint8_t *buf = &data->write[0];

	uint32_t count = 0;

	// start writing
	if (request->data_len > 0) {
		count = 0;

		// cfg
		buf[count++] = SPI_CMD_CFG_3(config->div);
		buf[count++] = SPI_CMD_CFG_2(config->cpol, config->cpha);
		buf[count++] = SPI_CMD_CFG_1();
		buf[count++] = SPI_CMD_CFG_0();

		// start
		buf[count++] = SPI_CMD_SOT_3(request->cs);
		buf[count++] = SPI_CMD_SOT_2();
		buf[count++] = SPI_CMD_SOT_1();
		buf[count++] = SPI_CMD_SOT_0();

		if (request->addr_width > 0) {
			// addr
			buf[count++] = SPI_CMD_SEND_ADDR3();
			buf[count++] = SPI_CMD_SEND_ADDR2();
			buf[count++] = SPI_CMD_SEND_ADDR1(request->addr_width);
			buf[count++] = SPI_CMD_SEND_ADDR0(config->quad_spi);

			// shift according to address width
			uint32_t addr;
			if (request->addr_width <= 8) {
				addr = request->addr << 24;
			} else if (request->addr_width <= 16) {
				addr = request->addr << 16;
			} else if (request->addr_width <= 24) {
				addr = request->addr << 8;
			} else {
				addr = request->addr;
			}

			// addr uint32
			buf[count++] = UINT32_BYTE3(addr);
			buf[count++] = UINT32_BYTE2(addr);
			buf[count++] = UINT32_BYTE1(addr);
			buf[count++] = UINT32_BYTE0(addr);
		}

		// data cfg
		buf[count++] = SPI_CMD_SEND_DATA3(request->data_len * 8);
		buf[count++] = SPI_CMD_SEND_DATA2(request->data_len * 8);
		buf[count++] = SPI_CMD_SEND_DATA1();
		buf[count++] = SPI_CMD_SEND_DATA0(config->quad_spi);

		// data; we need to fill multiple of word size
		uint32_t wordCount = ROUND_UP_DIV_4(request->data_len);
		for (uint32_t i = 0; i < request->data_len; i++) {
			buf[count + i] = request->data[i];
		}
		count += (wordCount * 4);

		// end of transmission
		buf[count++] = SPI_CMD_EOT3(0);
		buf[count++] = SPI_CMD_EOT2();
		buf[count++] = SPI_CMD_EOT1();
		buf[count++] = SPI_CMD_EOT0();

		UDMA_START_TX(config->base, (uint32_t)buf, count, 0);

		UDMA_WAIT_FOR_FINISHED_TX(config->base);

		int32_t xfer_count = request->data_len - UDMA_GET_REMAINING_RX(config->base);

		/*
		printf("<writing> count: %d, transfered %d; remain: %d\n", count,
		request->already_transferred, request->still_to_transfer);
		*/

		//sy1xx_spi_udma_reset(dev);

		if ((xfer_count > 0) && (xfer_count <= request->data_len)) {
			// request->data_len = read_count;
			//  memcpy(request->data, deviceBuffer[inst].read, read_count);
			return xfer_count;
		}
	}

	return 0;
}

/*
 * \return:
 * - < 0 ... error
 * - = 0 ... no bytes received
 * - > 0 ... number of bytes received
 */
int32_t sy1xx_spi_read(const struct device *dev, spiTransfer_t *request)
{

	struct sy1xx_spi_config *config = (struct sy1xx_spi_config *)dev->config;
	struct sy1xx_spi_data *data = (struct sy1xx_spi_data *)dev->data;

	if (request->data_len == 0) {
		return -2;
	}

	if (request->data_len > DEVICE_MAX_BUFFER_SIZE) {
		return -3;
	}

	// stop any prior transmission
	UDMA_WAIT_FOR_FINISHED_TX(config->base);
	UDMA_WAIT_FOR_FINISHED_RX(config->base);

	uint8_t *buf = &data->write[0];

	uint32_t count = 0;

	// start writing
	if (request->data_len > 0) {
		count = 0;

		// cfg
		buf[count++] = SPI_CMD_CFG_3(config->div);
		buf[count++] = SPI_CMD_CFG_2(config->cpol, config->cpha);
		buf[count++] = SPI_CMD_CFG_1();
		buf[count++] = SPI_CMD_CFG_0();

		// start
		buf[count++] = SPI_CMD_SOT_3(request->cs);
		buf[count++] = SPI_CMD_SOT_2();
		buf[count++] = SPI_CMD_SOT_1();
		buf[count++] = SPI_CMD_SOT_0();

		if (request->addr_width > 0) {
			// addr
			buf[count++] = SPI_CMD_SEND_ADDR3();
			buf[count++] = SPI_CMD_SEND_ADDR2();
			buf[count++] = SPI_CMD_SEND_ADDR1(request->addr_width);
			buf[count++] = SPI_CMD_SEND_ADDR0(config->quad_spi);

			// shift according to address width
			uint32_t addr;
			if (request->addr_width <= 8) {
				addr = request->addr << 24;
			} else if (request->addr_width <= 16) {
				addr = request->addr << 16;
			} else if (request->addr_width <= 24) {
				addr = request->addr << 8;
			} else {
				addr = request->addr;
			}

			// addr uint32
			buf[count++] = UINT32_BYTE3(addr);
			buf[count++] = UINT32_BYTE2(addr);
			buf[count++] = UINT32_BYTE1(addr);
			buf[count++] = UINT32_BYTE0(addr);
		}

		// data cfg
		buf[count++] = SPI_CMD_READ_DATA3(request->data_len * 8);
		buf[count++] = SPI_CMD_READ_DATA2(request->data_len * 8);
		buf[count++] = SPI_CMD_READ_DATA1();
		buf[count++] = SPI_CMD_READ_DATA0(0, 0);

		// end of transmission
		buf[count++] = SPI_CMD_EOT3(0);
		buf[count++] = SPI_CMD_EOT2();
		buf[count++] = SPI_CMD_EOT1();
		buf[count++] = SPI_CMD_EOT0();

		UDMA_START_RX(config->base, (uint32_t)data->read, request->data_len,
			      UDMA_RX_DATA_ADDR_INC_SIZE_32);
		UDMA_START_TX(config->base, (uint32_t)buf, count, 0);

		UDMA_WAIT_FOR_FINISHED_TX(config->base);
		UDMA_WAIT_FOR_FINISHED_RX(config->base);

		int32_t xfer_count = request->data_len - UDMA_GET_REMAINING_RX(config->base);

		/*
		printf("<reading> count: %d, transfered %d; remain: %d; received: %d \n", count,
		request->already_transferred, request->still_to_transfer, read_count);
		*/

		// sy1xx_spi_udma_reset(dev);

		if ((xfer_count > 0) && (xfer_count <= request->data_len)) {
			// request->data_len = read_count;
			memcpy(request->data, &data->read[0], xfer_count);

			LOG_HEXDUMP_DBG(request->data, xfer_count, "SPI:");

			return xfer_count;
		}
	}

	return 0;
}

/*
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
 */
static int sy1xx_spi_transceive_sync(const struct device *dev, const struct spi_config *spi_config,
				     const struct spi_buf_set *tx_bufs,
				     const struct spi_buf_set *rx_bufs)
{
	int ret = -EINVAL;

	uint32_t tx_buf_count = 0;
	uint32_t rx_buf_count = 0;

	if (tx_bufs != NULL) {
		tx_buf_count = tx_bufs->count;
	}

	if (rx_bufs != NULL) {
		rx_buf_count = rx_bufs->count;
	}

	spiTransfer_t request = {
		.cs = 0,
		.addr_width = 0,
		.addr = 0,

		.data = 0,
		.data_len = 0,
	};

	if ((tx_buf_count == 0) && (rx_buf_count == 1)) {
		/* pure read */

		request.data_len = rx_bufs->buffers[0].len;
		request.data = rx_bufs->buffers[0].buf;

		ret = sy1xx_spi_read(dev, &request);

	} else if ((tx_buf_count == 1) && (rx_buf_count == 0)) {
		/* pure write */

		request.data_len = tx_bufs->buffers[0].len;
		request.data = tx_bufs->buffers[0].buf;

		ret = sy1xx_spi_write(dev, &request);

	} else if ((tx_buf_count == 1) && (rx_buf_count == 2)) {
		/* read from address */

		/* decode address */
		request.addr_width = 8;
		request.addr = ((uint8_t *)tx_bufs->buffers[0].buf)[0];

		/* decode data */
		request.data = (uint8_t *)rx_bufs->buffers[1].buf;
		request.data_len = rx_bufs->buffers[1].len;

		ret = sy1xx_spi_read(dev, &request);

	} else if ((tx_buf_count == 2) && (rx_buf_count == 0)) {
		/* write to address */

		/* decode address */
		request.addr_width = 8;
		request.addr = ((uint8_t *)tx_bufs->buffers[0].buf)[0];

		/* decode data */
		request.data = (uint8_t *)tx_bufs->buffers[1].buf;
		request.data_len = tx_bufs->buffers[1].len;

		ret = sy1xx_spi_write(dev, &request);
	}

	if (ret == request.data_len) {
		/* success */
		return 0;
	} else {

		if (ret < 0) {
			printk("err: invalid spi transfer: %d\n", ret);
			return ret;
		} else {
			printk("wrn: invalid number of data transferred\n");
			return -1;
		}
	}
}

static int sy1xx_spi_release(const struct device *dev, const struct spi_config *config)
{

	return 0;
}

static const struct spi_driver_api sy1xx_spi_driver_api = {
	.transceive = sy1xx_spi_transceive_sync,
	.release = sy1xx_spi_release,
};

#define SPI_SY1XX_DEVICE_INIT(n)                                                                   \
                                                                                                   \
	static struct sy1xx_spi_config sy1xx_spi_dev_config_##n = {                                \
		.base = (uint32_t)DT_INST_REG_ADDR(n),                                             \
		.inst = (uint32_t)DT_INST_PROP(n, instance),                                       \
	};                                                                                         \
                                                                                                   \
	static struct sy1xx_spi_data __attribute__((section(".udma_access")))                      \
	__aligned(4) sy1xx_spi_dev_data_##n = {                                                    \
		SPI_CONTEXT_INIT_LOCK(sy1xx_spi_dev_data_##n, ctx),                                \
		SPI_CONTEXT_INIT_SYNC(sy1xx_spi_dev_data_##n, ctx),                                \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, &sy1xx_spi_init, NULL, &sy1xx_spi_dev_data_##n,                   \
			      &sy1xx_spi_dev_config_##n, POST_KERNEL, CONFIG_SPI_INIT_PRIORITY,    \
			      &sy1xx_spi_driver_api);

DT_INST_FOREACH_STATUS_OKAY(SPI_SY1XX_DEVICE_INIT)
