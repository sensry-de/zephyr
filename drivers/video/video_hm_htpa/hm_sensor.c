//
// Created by tswaehn on 1/13/25.
//

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(hm_sensor, LOG_LEVEL_DBG);



#include "hm_sensor.h"
#include "common/hm_spi.h"
#include "common/hm_flash.h"

static bool initialized = false;

// prototypes
static void hm_weakup_sensor(heimann_flash_t * flash);
void write_calibration_settings_to_sensor(heimann_flash_t * flash);

int32_t hm_init_sensor()
{
	init_buffers();

	if (0 != heimann_spi_init()) {
		LOG_ERR("failed to init spi");
		return -1;
	}


	wakeup_flash();

	LOG_INF("searching device... ");
	flash.id = 0;

	while (flash.id == 0 || flash.id == 0xFFFFFFFF) {
		flash.id = read_EEPROM_byte(E_ID1) << 24 | read_EEPROM_byte(E_ID2) << 16 | read_EEPROM_byte(E_ID3) << 8 | read_EEPROM_byte(E_ID4);
		if (flash.id > 0x00 && flash.id < 0xFFFFFFFF) {
			LOG_INF("HTPAd detected - %d [%#.8x}", flash.id, flash.id);
		}
		else {
			LOG_ERR("no HTPAd detected; next try in a seconds");
			return -1;
		}
	}

	// read the whole eeprom only once
	read_eeprom(&flash);
	LOG_INF("read eeprom done");

	LOG_INF("init calc...");
	init_calc();
	LOG_INF("init calc done");

	LOG_INF("wakeup sensor...");
	hm_weakup_sensor(&flash);
	LOG_INF("wakeup sensor done");


	LOG_INF("HTPAd is ready");

	initialized = true;
	return 0;
}

bool is_sensor_initialized()
{
	return initialized;
}



/********************************************************************
 Function:        void write_calibration_settings_to_sensor()
 Description:     write calibration data (from eeprom) to trim registers (sensor)
*******************************************************************/
void write_calibration_settings_to_sensor(heimann_flash_t * flash) {

	write_sensor_byte(TRIM_REGISTER1, flash->mbit_calib);
	k_sleep(K_MSEC(5));
	write_sensor_byte(TRIM_REGISTER2, flash->bias_calib);
	k_sleep(K_MSEC(5));
	write_sensor_byte(TRIM_REGISTER3, flash->bpa_calib);
	k_sleep(K_MSEC(5));
	write_sensor_byte(TRIM_REGISTER4, flash->clk_calib);

}

void hm_weakup_sensor(heimann_flash_t * flash)
{

	//*******************************************************************
	// wake up and start the sensor
	//*******************************************************************
	// to wake up sensor set configuration register to 0x01
	// |    RFU    |   Block   | Start | VDD_MEAS | BLIND | WAKEUP |
	// |  0  |  0  |  0  |  0  |   0   |    0     |   0   |    1   |
	write_sensor_byte(CONFIGURATION_REGISTER, 0x01);

	// write the calibration settings into the trim registers
	write_calibration_settings_to_sensor(flash);

}


void hm_grab_image(uint8_t frame[RAW_FRAME_BLOCKS][FRAME_BLOCK_LENGTH])
{
	uint8_t status = 0x0;
	uint8_t * blockMem = 0;

	static uint32_t acquisition_time_usec = 1000;

	/* we read +1 block for electrical offsets */
	for (uint32_t b = 0; b <= NUMBER_OF_BLOCKS; b++) {

		if (b < NUMBER_OF_BLOCKS) {
			/* read image block ex. 0..5 */
			writeSensorReg(0x01, 0x9 | (b << 4));
		} else {
			/* read electronic offsets ex. 6 */
			writeSensorReg(0x01, 0x0b);
		}

		//		readSensorReg(0x2, &status);
		uint32_t waiting_loops = 0;
		k_sleep(K_USEC(acquisition_time_usec));

		while (true) {
			/* wait for conversion ready */
			readSensorReg(0x2, &status);
			if ((status & 0x1) == 1) {
				// k_sleep(K_MSEC(5));
				break;
			}
			waiting_loops++;
			k_sleep(K_MSEC(1));
		}

		if (waiting_loops > 0) {
			acquisition_time_usec += 100;
			LOG_WRN("Waiting for conversion ready: %d loops, new acquisition time: %d us", waiting_loops, acquisition_time_usec);
		}

		/* top array */
		blockMem = frame[b];
		heimann_sensor_read(0xa, blockMem, FRAME_BLOCK_LENGTH);

		/* bottom array */
		if (b == NUMBER_OF_BLOCKS) {
			/* electrical offsets */
			blockMem = frame[NUMBER_OF_BLOCKS+1];
		} else {
			blockMem = frame[(NUMBER_OF_BLOCKS+1) + (NUMBER_OF_BLOCKS-b)];
		}
		heimann_sensor_read(0xb, blockMem, FRAME_BLOCK_LENGTH);
	}


}