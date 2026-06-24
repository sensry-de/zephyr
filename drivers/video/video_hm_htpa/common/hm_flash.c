//
// Created by tswaehn on 1/13/25.
//

#include <ext_sram.h>


#if defined(CONFIG_HEIMANN_SENSOR_HTPA120X84)
	#include "../HTPA120x84/hm_const.h"
#elif defined(CONFIG_HEIMANN_SENSOR_HTPA160X120)
	#include "../HTPA160x120/hm_const.h"
#else
	#error "No Heimann sensor type selected"
#endif


#include "hm_flash.h"
#include "hm_spi.h"

heimann_flash_t flash EXT_SRAM_SECTION;

/********************************************************************
 Function:        void read_eeprom()
 Description:     read all values from eeprom
*******************************************************************/
void read_eeprom(heimann_flash_t *flash)
{
	int m = 0;
	int n = 0;
	byte b[4];
	flash->id = read_EEPROM_byte(E_ID4) << 24 | read_EEPROM_byte(E_ID3) << 16 |
		    read_EEPROM_byte(E_ID2) << 8 | read_EEPROM_byte(E_ID1);
	flash->mbit_calib = read_EEPROM_byte(E_MBIT_CALIB);
	flash->bias_calib = read_EEPROM_byte(E_BIAS_CALIB);
	flash->clk_calib = read_EEPROM_byte(E_CLK_CALIB);
	flash->bpa_calib = read_EEPROM_byte(E_BPA_CALIB);
	flash->pu_calib = read_EEPROM_byte(E_PU_CALIB);
	flash->mbit_user = read_EEPROM_byte(E_MBIT_USER);
	flash->bias_user = read_EEPROM_byte(E_BIAS_USER);
	flash->clk_user = read_EEPROM_byte(E_CLK_USER);
	flash->bpa_user = read_EEPROM_byte(E_BPA_USER);
	flash->pu_user = read_EEPROM_byte(E_PU_USER);
	flash->vddth1 = read_EEPROM_byte(E_VDDTH1_2) << 8 | read_EEPROM_byte(E_VDDTH1_1);
	flash->vddth2 = read_EEPROM_byte(E_VDDTH2_2) << 8 | read_EEPROM_byte(E_VDDTH2_1);
	flash->vddscgrad = read_EEPROM_byte(E_VDDSCGRAD);
	flash->vddscoff = read_EEPROM_byte(E_VDDSCOFF);
	flash->ptatth1 = read_EEPROM_byte(E_PTATTH1_2) << 8 | read_EEPROM_byte(E_PTATTH1_1);
	flash->ptatth2 = read_EEPROM_byte(E_PTATTH2_2) << 8 | read_EEPROM_byte(E_PTATTH2_1);
	flash->nrofdefpix = read_EEPROM_byte(E_NROFDEFPIX);
	flash->gradscale = read_EEPROM_byte(E_GRADSCALE);
	flash->tablenumber =
		read_EEPROM_byte(E_TABLENUMBER2) << 8 | read_EEPROM_byte(E_TABLENUMBER1);
	flash->arraytype = read_EEPROM_byte(E_ARRAYTYPE);
	b[0] = read_EEPROM_byte(E_PTATGR_1);
	b[1] = read_EEPROM_byte(E_PTATGR_2);
	b[2] = read_EEPROM_byte(E_PTATGR_3);
	b[3] = read_EEPROM_byte(E_PTATGR_4);
	flash->ptatgr_float = *(float *)b;
	b[0] = read_EEPROM_byte(E_PTATOFF_1);
	b[1] = read_EEPROM_byte(E_PTATOFF_2);
	b[2] = read_EEPROM_byte(E_PTATOFF_3);
	b[3] = read_EEPROM_byte(E_PTATOFF_4);
	flash->ptatoff_float = *(float *)b;
	b[0] = read_EEPROM_byte(E_PIXCMIN_1);
	b[1] = read_EEPROM_byte(E_PIXCMIN_2);
	b[2] = read_EEPROM_byte(E_PIXCMIN_3);
	b[3] = read_EEPROM_byte(E_PIXCMIN_4);
	flash->pixcmin = *(float *)b;
	b[0] = read_EEPROM_byte(E_PIXCMAX_1);
	b[1] = read_EEPROM_byte(E_PIXCMAX_2);
	b[2] = read_EEPROM_byte(E_PIXCMAX_3);
	b[3] = read_EEPROM_byte(E_PIXCMAX_4);
	flash->pixcmax = *(float *)b;
	flash->epsilon = read_EEPROM_byte(E_EPSILON);
	flash->globaloff = read_EEPROM_byte(E_GLOBALOFF);
	flash->globalgain =
		read_EEPROM_byte(E_GLOBALGAIN_2) << 8 | read_EEPROM_byte(E_GLOBALGAIN_1);

	// for (int m = 0; m < PIXEL_PER_COLUMN; m++) {
	//   for (int n = 0; n < PIXEL_PER_ROW; n++) {

	// --- DeadPixAdr ---
	for (int i = 0; i < flash->nrofdefpix; i++) {
		flash->deadpixadr[i] = read_EEPROM_byte(E_DEADPIXADR + 2 * i + 1) << 8 |
				       read_EEPROM_byte(E_DEADPIXADR + 2 * i);
		if (flash->deadpixadr[i] >
		    (unsigned short)(NUMBER_OF_PIXEL / 2)) { // adaptedAdr:
			flash->deadpixadr[i] =
				(unsigned short)(NUMBER_OF_PIXEL) +
				(unsigned short)(NUMBER_OF_PIXEL / 2) -
				flash->deadpixadr[i] +
				2 * (unsigned short)(flash->deadpixadr[i] % PIXEL_PER_ROW) -
				PIXEL_PER_ROW;
		}
	}

	// --- DeadPixMask ---
	for (int i = 0; i < flash->nrofdefpix; i++) {
		flash->deadpixmask[i] = read_EEPROM_byte(E_DEADPIXMASK + i);
	}

	// --- Thgrad_ij, ThOffset_ij and P_ij ---
	m = 0;
	n = 0;

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
		flash->thgrad[m][n] = (signed char)(read_EEPROM_byte(E_THGRAD + i));
		flash->thoffset[m][n] = read_EEPROM_byte(E_THOFFSET + 2 * i + 1) << 8 |
					read_EEPROM_byte(E_THOFFSET + 2 * i);
		*(pixc2 + m * PIXEL_PER_ROW + n) =
			read_EEPROM_byte(E_PIJ + 2 * i + 1) << 8 | read_EEPROM_byte(E_PIJ + 2 * i);
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
		flash->thgrad[m][n] = read_EEPROM_byte(E_THGRAD + i);
		flash->thoffset[m][n] = read_EEPROM_byte(E_THOFFSET + 2 * i + 1) << 8 |
					read_EEPROM_byte(E_THOFFSET + 2 * i);
		*(pixc2bot + m * PIXEL_PER_ROW + n) =
			read_EEPROM_byte(E_PIJ + 2 * i + 1) << 8 | read_EEPROM_byte(E_PIJ + 2 * i);
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
		flash->vddcompgrad[m][n] = read_EEPROM_byte(E_VDDCOMPGRAD + 2 * i + 1) << 8 |
					   read_EEPROM_byte(E_VDDCOMPGRAD + 2 * i);
		flash->vddcompoff[m][n] = read_EEPROM_byte(E_VDDCOMPOFF + 2 * i + 1) << 8 |
					  read_EEPROM_byte(E_VDDCOMPOFF + 2 * i);
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
		flash->vddcompgrad[m][n] = read_EEPROM_byte(E_VDDCOMPGRAD + 2 * i + 1) << 8 |
					   read_EEPROM_byte(E_VDDCOMPGRAD + 2 * i);
		flash->vddcompoff[m][n] = read_EEPROM_byte(E_VDDCOMPOFF + 2 * i + 1) << 8 |
					  read_EEPROM_byte(E_VDDCOMPOFF + 2 * i);
		n++;
		if (n == PIXEL_PER_ROW) {
			n = 0;
			m--; // !!!! backwards !!!!
		}
	}
}
