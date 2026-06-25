//
// Created by tswaehn on 1/9/25.
//

#ifndef ZEPHYR_MINIMAL_APP_SRC_HEIMANN_SPI_H
#define ZEPHYR_MINIMAL_APP_SRC_HEIMANN_SPI_H

#include <stdint.h>

typedef uint8_t byte;

int32_t heimann_spi_init();

// sensor
void readSensorReg(uint8_t reg, uint8_t *value);
void writeSensorReg(uint8_t reg, uint8_t value);
void heimann_sensor_read(uint8_t reg, uint8_t *rx_buffer, uint32_t rx_len);
// deprecated
void write_sensor_byte(unsigned char addr, unsigned char input);

// flash
void heimann_eeprom_set_cmd(uint8_t cmd);
void heimann_eeprom_set_addr(uint8_t cmd, uint16_t addr);
void heimann_eeprom_read(uint8_t cmd, uint8_t *rx_buffer, uint32_t rx_len);

byte read_EEPROM_byte(unsigned int address);
void wakeup_flash();

#endif // ZEPHYR_MINIMAL_APP_SRC_HEIMANN_SPI_H
