//
// Created by tswaehn on 1/9/25.
//

#include "hm_spi.h"

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(heimann_spi, LOG_LEVEL_INF);

#ifdef CONFIG_SOC_FAMILY_ATMEL_SAM
#define HEIMANN_GPIO_CHIP_SELECT_PIN 24

static const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(spi0));
static const struct device *gpio_dev = DEVICE_DT_GET(DT_NODELABEL(pioa));

#define SPI_SLAVE_ADDR 	0x0
#define SPI_SPEED	20000000

static struct spi_config hm_spi_cfg = {
	.frequency = SPI_SPEED, // 1 MHz
	.operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB,
	.slave = SPI_SLAVE_ADDR, // Adjust as needed
	.cs = NULL, // Use chip select if required
};

#endif

#if CONFIG_BOARD_GANYMED_SK
#define HEIMANN_GPIO_CHIP_SELECT_RES_PIN 14
#define HEIMANN_GPIO_CHIP_SELECT_PIN 27

static const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(spi3));
static const struct device *gpio_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));

#define SPI_SLAVE_ADDR 	0x80
#define SPI_SPEED	10000000

static struct spi_config hm_spi_cfg = {
	.frequency = SPI_SPEED, // 1 MHz
	.operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB,
	.slave = SPI_SLAVE_ADDR, // Adjust as needed
	.cs = NULL, // Use chip select if required
};

#endif

#if CONFIG_BOARD_GANYMED_BOB
#define HEIMANN_GPIO_CHIP_SELECT_RES_PIN 14
#define HEIMANN_GPIO_CHIP_SELECT_PIN 15

const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(spi5));
static const struct device *gpio_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));

#define SPI_SLAVE_ADDR 	0x80
#define SPI_SPEED	10000000

static struct spi_config hm_spi_cfg = {
	.frequency = SPI_SPEED, // 1 MHz
	.operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB,
	.slave = SPI_SLAVE_ADDR, // Adjust as needed
	.cs = NULL, // Use chip select if required
};
#endif



int32_t heimann_spi_init(){
	if (!device_is_ready(dev)) {
		printk("device not ready\n");
		return -1;
	}
	int ret;

	ret = gpio_pin_configure(gpio_dev, HEIMANN_GPIO_CHIP_SELECT_PIN, GPIO_OUTPUT);
	if (ret < 0) {
		printk("Failed to configure GPIO pin\n");
		return -1;
	}

	return 0;
}

static void chip_select_high()
{
	gpio_pin_set(gpio_dev, HEIMANN_GPIO_CHIP_SELECT_PIN, 1); // Set the pin high
}

static void chip_select_low()
{
	gpio_pin_set(gpio_dev, HEIMANN_GPIO_CHIP_SELECT_PIN, 0); // Set the pin high
}

void heimann_sensor_read(uint8_t reg, uint8_t *rx_buffer, size_t rx_len)
{

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

	chip_select_high();

	int ret = spi_transceive(dev, &hm_spi_cfg, &tx, &rx);
	if (ret) {
		LOG_ERR("SPI read failed: %d", ret);
	}

	chip_select_low();
}


void heimann_sensor_write(uint8_t reg, uint8_t *tx_buffer, size_t tx_len)
{

	/* tx */
	struct spi_buf tx_buf[2];
	tx_buf[0].buf = &reg;
	tx_buf[0].len = 1;

	tx_buf[1].buf = tx_buffer;
	tx_buf[1].len = tx_len;

	const struct spi_buf_set tx = {
		.buffers = (struct spi_buf *) &tx_buf,
		.count = ARRAY_SIZE(tx_buf),
	};

	chip_select_high();

	int ret = spi_write(dev, &hm_spi_cfg, &tx);
	if (ret) {
		LOG_ERR("SPI write failed: %d", ret);
	}

	chip_select_low();
}


void writeSensorReg(uint8_t reg, uint8_t value)
{
	heimann_sensor_write(reg, &value, 1);
}

void readSensorReg(uint8_t reg, uint8_t *value)
{

	heimann_sensor_read(reg, value, 1);
}


void write_sensor_byte(unsigned char addr, unsigned char input){
	writeSensorReg(addr, input);
}

#if defined(CONFIG_HEIMANN_SENSOR_HTPA120X84)
void heimann_eeprom_set_cmd(uint8_t cmd)
{
	uint8_t tx_buffer[1] = {cmd};
	uint32_t tx_len = 1;

	/* tx */
	struct spi_buf tx_buf[1];
	tx_buf[0].buf = tx_buffer;
	tx_buf[0].len = tx_len;

	const struct spi_buf_set tx = {.buffers = (struct spi_buf *) &tx_buf, .count = 1};


	chip_select_low();

	int ret = spi_transceive(dev, &hm_spi_cfg, &tx, NULL);
	if (ret) {
		LOG_ERR("SPI read failed: %d", ret);
	}

	chip_select_high();

}

void heimann_eeprom_set_addr(uint8_t cmd, uint16_t addr)
{


	uint8_t tx_buffer[3] = {cmd,  (addr >> 8) & 0xff, (addr & 0xff)};
	uint32_t tx_len = 3;

	/* tx */
	struct spi_buf tx_buf[1];
	tx_buf[0].buf = tx_buffer;
	tx_buf[0].len = tx_len;

	const struct spi_buf_set tx = {.buffers = (struct spi_buf *) &tx_buf, .count = 1};

	chip_select_low();

	int ret = spi_transceive(dev, &hm_spi_cfg, &tx, NULL);
	if (ret) {
		LOG_ERR("SPI read failed: %d", ret);
	}

	chip_select_high();

}

void heimann_eeprom_read(uint8_t cmd, uint8_t *rx_buffer, uint32_t rx_len)
{


	uint8_t tx_buffer[1] = {cmd};
	uint32_t tx_len = 1;

	/* tx */
	struct spi_buf tx_buf[1];
	tx_buf[0].buf = tx_buffer;
	tx_buf[0].len = tx_len;

	const struct spi_buf_set tx = {.buffers = (struct spi_buf *) &tx_buf, .count = 1};


	/* rx */
	struct spi_buf rx_buf[2];
	rx_buf[0].buf = NULL;
	rx_buf[0].len = 1;

	if (rx_len) {
		rx_buf[1].buf = rx_buffer;
		rx_buf[1].len = rx_len;
	} else {

	}

	const struct spi_buf_set rx = {
		.buffers = (struct spi_buf *) &rx_buf,
		.count = ARRAY_SIZE(rx_buf),
	};

	chip_select_low();

	int ret = spi_transceive(dev, &hm_spi_cfg, &tx, &rx);
	if (ret) {
		LOG_ERR("SPI read failed: %d", ret);
	}

	chip_select_high();
}

/********************************************************************
 Function:        void read_EEPROM_byte(unsigned int eeaddress )
 Description:     read eeprom register as 8
 Dependencies:    register address (address)
*******************************************************************/
byte read_EEPROM_byte(unsigned int address ) {
	byte rdata = 0xFF;

	uint8_t data[4];

	/* set address */
	heimann_eeprom_set_addr(0x90, address / 4);

	/* set read mode */
	heimann_eeprom_set_addr(0xb0, 0x0000);

	/* actually read */
	heimann_eeprom_read(0xb1, (uint8_t*) &data, 4);

	rdata = data[3 - (address % 4)];

	LOG_DBG("eeprom %#.8x = %d", address, rdata);

	return rdata;
}

void wakeup_flash(){
	/*
	chip_select_low();
	k_sleep(K_MSEC(1));
	chip_select_high();
	k_sleep(K_MSEC(1));

	heimann_eeprom_set_cmd(0x10);
	k_sleep(K_MSEC(1));
	heimann_eeprom_set_cmd(0x20);
	k_sleep(K_MSEC(1));
	*/
}
#endif

#if defined(CONFIG_HEIMANN_SENSOR_HTPA160X120)
void flash_send_byte( uint8_t data_byte)
{
	uint8_t tx_buffer[1] = {data_byte};
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

	chip_select_low();

	int ret = spi_transceive(dev, &hm_spi_cfg, &tx, &rx);
	if (ret) {
		LOG_ERR("SPI read failed: %d", ret);
	}

	chip_select_high();
}

uint8_t flash_read_status()
{
	uint8_t rx_buffer[1];
	uint32_t rx_len = 1;

	uint8_t tx_buffer[1] = {0x05};
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

	chip_select_low();

	int ret = spi_transceive(dev, &hm_spi_cfg, &tx, &rx);
	if (ret) {
		LOG_ERR("SPI read failed: %d", ret);
	}

	chip_select_high();

	return rx_buffer[0];
}

void flash_write_enable()
{
	flash_send_byte(0x06);
}

void flash_write_disable()
{
	flash_send_byte(0x06);
}

#define ADD1(X)   (unsigned int)((X & 0xFF0000)>>16)
#define ADD2(X)   (unsigned int)((X & 0x00FF00)>>8)
#define ADD3(X)   (unsigned int)(X & 0x0000FF)

byte read_EEPROM_bytes(unsigned int address, uint8_t * rx_buffer, uint32_t rx_len)
{

	uint8_t tx_buffer[4] = {
		0x03,
		ADD1(address),
		ADD2(address),
		ADD3(address)
	};

	uint32_t tx_len = 4;

	/* tx */
	struct spi_buf tx_buf[1];
	tx_buf[0].buf = tx_buffer;
	tx_buf[0].len = tx_len;

	const struct spi_buf_set tx = {.buffers = (struct spi_buf *) &tx_buf, .count = 1};


	/* rx */
	struct spi_buf rx_buf[2];
	rx_buf[0].buf = NULL;
	rx_buf[0].len = 4;

	if (rx_len) {
		rx_buf[1].buf = rx_buffer;
		rx_buf[1].len = rx_len;
	} else {

	}

	const struct spi_buf_set rx = {
		.buffers = (struct spi_buf *) &rx_buf,
		.count = ARRAY_SIZE(rx_buf),
	};

	chip_select_low();

	int ret = spi_transceive(dev, &hm_spi_cfg, &tx, &rx);
	if (ret) {
		LOG_ERR("SPI read failed: %d", ret);
		return -1;
	}

	chip_select_high();

	return 0;
}

void wakeup_flash()
{
	flash_write_enable();
	uint8_t ret = 0xff;

	do {
		ret = flash_read_status();
	} while ((ret & 0x1) || (ret &0x80));

	flash_write_disable();
}

byte read_EEPROM_byte(unsigned int address)
{
	uint8_t rx_byte;

	if (0 == read_EEPROM_bytes(address, &rx_byte, 1)) {
		return rx_byte;
	}

	return 0xff;
}

#endif