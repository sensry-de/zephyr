//
// Created by tswaehn on 6/24/26.
//

#ifndef HEIMANN_APP_HM_FLASH_H
#define HEIMANN_APP_HM_FLASH_H

#include <stdint.h>

typedef struct {
	//-----------------------------------------
	// EEPROM DATA
	unsigned char mbit_calib, bias_calib, clk_calib, bpa_calib, pu_calib, mbit_user, bias_user, clk_user, bpa_user, pu_user;
	unsigned char nrofdefpix, gradscale, vddscgrad, vddscoff, epsilon, lastepsilon, arraytype;
	unsigned char deadpixmask[ALLOWED_DEADPIX];
	signed char globaloff;
	int8_t thgrad[PIXEL_PER_COLUMN][PIXEL_PER_ROW];
	unsigned short tablenumber, vddth1, vddth2, ptatth1, ptatth2, ptatgr, globalgain;
	unsigned short deadpixadr[ALLOWED_DEADPIX * 2];
	int16_t thoffset[PIXEL_PER_COLUMN][PIXEL_PER_ROW];
	signed short vddcompgrad[ROW_PER_BLOCK * 2][PIXEL_PER_ROW];
	signed short vddcompoff[ROW_PER_BLOCK * 2][PIXEL_PER_ROW];
	unsigned long id, ptatoff;
	float ptatgr_float, ptatoff_float, pixcmin, pixcmax, bw;

} heimann_flash_t;

void read_eeprom(heimann_flash_t *flash);

extern heimann_flash_t flash;

#endif //HEIMANN_APP_HM_FLASH_H
