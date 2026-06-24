//
// Created by tswaehn on 7/14/26.
//


#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(htpa_calib, CONFIG_VIDEO_LOG_LEVEL);

#include <zephyr/device.h>

#include "htpa-common.h"
#include "htpa-calib.h"

#include "zephyr/drivers/flash.h"

#define read_EEPROM(reg)	read_flash_bytewise(dev, reg)

static uint8_t read_flash_bytewise(const struct device *dev, unsigned int address)
{
	const struct hm_htpa_config *cfg = dev->config;
	struct hm_htpa_data *data = dev->data;
	const struct device *flash_dev = cfg->calibration_flash;
	uint8_t rx_byte;
	int ret;

	ret = flash_read(flash_dev, address, &rx_byte, 1);
	if (0 != ret) {
		data->communication_error = ret;
		data->communication_error_count++;
		return 0x0;
	}
		
	return rx_byte;
}


/********************************************************************
 Function:        void read_eeprom()
 Description:     read all values from eeprom
*******************************************************************/
int htpa_read_calibration(const struct device *dev, heimann_calibration_t *calib)
{
	const struct hm_htpa_config *cfg = dev->config;
	struct hm_htpa_data *data = dev->data;
	int m = 0;
	int n = 0;
	uint8_t b[4];
	bool has_error = false;

	if (!device_is_ready(cfg->calibration_flash)) {
		LOG_ERR("calibration flash is not ready");
		return -ENODEV;
	}

	calib->id = read_EEPROM(E_ID4) << 24 | read_EEPROM(E_ID3) << 16 |
		    read_EEPROM(E_ID2) << 8 | read_EEPROM(E_ID1);
	calib->mbit_calib = read_EEPROM(E_MBIT_CALIB);
	calib->bias_calib = read_EEPROM(E_BIAS_CALIB);
	calib->clk_calib = read_EEPROM(E_CLK_CALIB);
	calib->bpa_calib = read_EEPROM(E_BPA_CALIB);
	calib->pu_calib = read_EEPROM(E_PU_CALIB);
	calib->mbit_user = read_EEPROM(E_MBIT_USER);
	calib->bias_user = read_EEPROM(E_BIAS_USER);
	calib->clk_user = read_EEPROM(E_CLK_USER);
	calib->bpa_user = read_EEPROM(E_BPA_USER);
	calib->pu_user = read_EEPROM(E_PU_USER);
	calib->vddth1 = read_EEPROM(E_VDDTH1_2) << 8 | read_EEPROM(E_VDDTH1_1);
	calib->vddth2 = read_EEPROM(E_VDDTH2_2) << 8 | read_EEPROM(E_VDDTH2_1);
	calib->vddscgrad = read_EEPROM(E_VDDSCGRAD);
	calib->vddscoff = read_EEPROM(E_VDDSCOFF);
	calib->ptatth1 = read_EEPROM(E_PTATTH1_2) << 8 | read_EEPROM(E_PTATTH1_1);
	calib->ptatth2 = read_EEPROM(E_PTATTH2_2) << 8 | read_EEPROM(E_PTATTH2_1);
	calib->nrofdefpix = read_EEPROM(E_NROFDEFPIX);
	calib->gradscale = read_EEPROM(E_GRADSCALE);
	calib->tablenumber =
		read_EEPROM(E_TABLENUMBER2) << 8 | read_EEPROM(E_TABLENUMBER1);
	calib->arraytype = read_EEPROM(E_ARRAYTYPE);
	b[0] = read_EEPROM(E_PTATGR_1);
	b[1] = read_EEPROM(E_PTATGR_2);
	b[2] = read_EEPROM(E_PTATGR_3);
	b[3] = read_EEPROM(E_PTATGR_4);
	calib->ptatgr_float = *(float *)b;
	b[0] = read_EEPROM(E_PTATOFF_1);
	b[1] = read_EEPROM(E_PTATOFF_2);
	b[2] = read_EEPROM(E_PTATOFF_3);
	b[3] = read_EEPROM(E_PTATOFF_4);
	calib->ptatoff_float = *(float *)b;
	b[0] = read_EEPROM(E_PIXCMIN_1);
	b[1] = read_EEPROM(E_PIXCMIN_2);
	b[2] = read_EEPROM(E_PIXCMIN_3);
	b[3] = read_EEPROM(E_PIXCMIN_4);
	calib->pixcmin = *(float *)b;
	b[0] = read_EEPROM(E_PIXCMAX_1);
	b[1] = read_EEPROM(E_PIXCMAX_2);
	b[2] = read_EEPROM(E_PIXCMAX_3);
	b[3] = read_EEPROM(E_PIXCMAX_4);
	calib->pixcmax = *(float *)b;
	calib->epsilon = read_EEPROM(E_EPSILON);
	calib->globaloff = read_EEPROM(E_GLOBALOFF);
	calib->globalgain =
		read_EEPROM(E_GLOBALGAIN_2) << 8 | read_EEPROM(E_GLOBALGAIN_1);

	// for (int m = 0; m < PIXEL_PER_COLUMN; m++) {
	//   for (int n = 0; n < PIXEL_PER_ROW; n++) {

	// --- DeadPixAdr ---
	for (int i = 0; i < calib->nrofdefpix; i++) {
		calib->deadpixadr[i] = read_EEPROM(E_DEADPIXADR + 2 * i + 1) << 8 |
				       read_EEPROM(E_DEADPIXADR + 2 * i);
		if (calib->deadpixadr[i] >
		    (unsigned short)(NUMBER_OF_PIXEL / 2)) { // adaptedAdr:
			calib->deadpixadr[i] =
				(unsigned short)(NUMBER_OF_PIXEL) +
				(unsigned short)(NUMBER_OF_PIXEL / 2) -
				calib->deadpixadr[i] +
				2 * (unsigned short)(calib->deadpixadr[i] % PIXEL_PER_ROW) -
				PIXEL_PER_ROW;
		}
	}

	// --- DeadPixMask ---
	for (int i = 0; i < calib->nrofdefpix; i++) {
		calib->deadpixmask[i] = read_EEPROM(E_DEADPIXMASK + i);
	}

	// --- Thgrad_ij, ThOffset_ij and P_ij ---
	m = 0;
	n = 0;
/*
	extern uint16_t pixc2_0[NUMBER_OF_PIXEL];
	uint16_t * pixc2 = pixc2_0; // set pointer to start address of the allocated heap //
				       // reset pointer to initial address

#if defined(CONFIG_HEIMANN_SENSOR_HTPA120X84)
	uint16_t * pixc2bot = pixc2;
#endif

#if defined(CONFIG_HEIMANN_SENSOR_HTPA160X120)
	extern uint16_t pixc2bot_0[NUMBER_OF_PIXEL];
	uint16_t * pixc2bot = pixc2bot_0;
#endif



	// top half
	for (int i = 0; i < (unsigned short)(NUMBER_OF_PIXEL / 2); i++) {
		calib->thgrad[m][n] = (signed char)(read_EEPROM(E_THGRAD + i));
		calib->thoffset[m][n] = read_EEPROM(E_THOFFSET + 2 * i + 1) << 8 |
					read_EEPROM(E_THOFFSET + 2 * i);
		*(pixc2 + m * PIXEL_PER_ROW + n) =
			read_EEPROM(E_PIJ + 2 * i + 1) << 8 | read_EEPROM(E_PIJ + 2 * i);
		n++;
		if (n == PIXEL_PER_ROW) {
			n = 0;
			m++; // !!!! forwards !!!!
		}
	}
	// bottom half
	m = (unsigned char)(PIXEL_PER_COLUMN/2 - 1);
	n = 0;
	for (int i = (unsigned short)(NUMBER_OF_PIXEL / 2); i < (unsigned short)(NUMBER_OF_PIXEL); i++) {
		calib->thgrad[m][n] = read_EEPROM(E_THGRAD + i);
		calib->thoffset[m][n] = read_EEPROM(E_THOFFSET + 2 * i + 1) << 8 |
					read_EEPROM(E_THOFFSET + 2 * i);
		*(pixc2bot + m * PIXEL_PER_ROW + n) =
			read_EEPROM(E_PIJ + 2 * i + 1) << 8 | read_EEPROM(E_PIJ + 2 * i);
		n++;

		if (n == PIXEL_PER_ROW) {
			n = 0;
			m--; // !!!! backwards !!!!
		}
	}

	//---VddCompGrad and VddCompOff---
	// top half
	m = 0;
	n = 0;
	// top half
	for (int i = 0; i < (unsigned short)(PIXEL_PER_BLOCK); i++) {
		calib->vddcompgrad[m][n] = read_EEPROM(E_VDDCOMPGRAD + 2 * i + 1) << 8 |
					   read_EEPROM(E_VDDCOMPGRAD + 2 * i);
		calib->vddcompoff[m][n] = read_EEPROM(E_VDDCOMPOFF + 2 * i + 1) << 8 |
					  read_EEPROM(E_VDDCOMPOFF + 2 * i);
		n++;
		if (n == PIXEL_PER_ROW) {
			n = 0;
			m++; // !!!! forwards !!!!
		}
	}
	// bottom half
	m = (unsigned char)(ROW_PER_BLOCK * 2 - 1);
	n = 0;
	for (int i = (unsigned short)(PIXEL_PER_BLOCK);
	     i < (unsigned short)(PIXEL_PER_BLOCK * 2); i++) {
		calib->vddcompgrad[m][n] = read_EEPROM(E_VDDCOMPGRAD + 2 * i + 1) << 8 |
					   read_EEPROM(E_VDDCOMPGRAD + 2 * i);
		calib->vddcompoff[m][n] = read_EEPROM(E_VDDCOMPOFF + 2 * i + 1) << 8 |
					  read_EEPROM(E_VDDCOMPOFF + 2 * i);
		n++;
		if (n == PIXEL_PER_ROW) {
			n = 0;
			m--; // !!!! backwards !!!!
		}
	}

*/

	if (has_error) {
		LOG_ERR("Error reading EEPROM");
		return -EINVAL;
	}

	if ((data->calib.id == 0) || (data->calib.id == 0xffffffff)) {
		LOG_ERR("Invalid calibration data id");
		return -EINVAL;
	}

	return 0;
}
