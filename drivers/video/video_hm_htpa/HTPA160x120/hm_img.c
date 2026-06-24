#include <stdint.h>

#include "hm_img.h"

#include <ext_sram.h>

#include "hm_const.h"
#include "../common/hm_flash.h"


extern const unsigned int TempTable [NROFADELEMENTS][NROFTAELEMENTS];
extern const unsigned int XTATemps[NROFTAELEMENTS];
extern const unsigned int YADValues[NROFADELEMENTS];

int32_t data_pixel[PIXEL_PER_COLUMN][PIXEL_PER_ROW] EXT_SRAM_SECTION;
uint8_t RAMoutput[2 * NUMBER_OF_BLOCKS + 2][BLOCK_LENGTH] EXT_SRAM_SECTION;



// STRUCT WITH ALL SENSOR CHARACTERISTICS
typedef struct {
	unsigned short NumberOfPixel;
	unsigned char NumberOfBlocks;
	unsigned char RowPerBlock;
	unsigned short PixelPerBlock;
	unsigned short PixelPerColumn;
	unsigned short PixelPerRow;
	unsigned char AllowedDeadPix;
	unsigned short TableNumber;
	unsigned short TableOffset;
	unsigned char PTATPos;
	unsigned char VDDPos;
	unsigned char PTATVDDSwitch;
	unsigned char CyclopsActive;
	unsigned char CyclopsPos;
	unsigned char DataPos;
} SensorConst_t;


const SensorConst_t DevConst = {
	.NumberOfPixel=NUMBER_OF_PIXEL,
	.NumberOfBlocks=NUMBER_OF_BLOCKS,
	.RowPerBlock=ROW_PER_BLOCK,
	.PixelPerBlock=PIXEL_PER_BLOCK,
	.PixelPerColumn=PIXEL_PER_COLUMN,
	.PixelPerRow=PIXEL_PER_ROW,
	.AllowedDeadPix=ALLOWED_DEADPIX,
	.TableNumber=TABLENUMBER,
	.TableOffset=TABLEOFFSET,
	.PTATPos=PTAT_POS,
	.VDDPos=VDD_POS,
	.PTATVDDSwitch=PTAT_VDD_SWITCH,
	.CyclopsActive=ATC_ACTIVE,
	.CyclopsPos=ATC_POS,
	.DataPos=DATA_POS,
};




/*   */


typedef struct {
	uint16_t Ta, ptat_av_uint16, vdd_av_uint16, ATC0, ATC1;
	uint32_t gradscale_div;
	int32_t vddcompgrad_n;
	int vddcompoff_n;
	uint32_t vddscgrad_div;
	uint32_t vddscoff_div;
} processing_t;


static processing_t proc EXT_SRAM_SECTION;


static int32_t eloffset[ROW_PER_BLOCK * 2][PIXEL_PER_ROW] EXT_SRAM_SECTION;

static uint16_t ptat_buffer[PTAT_BUFFER_SIZE] EXT_SRAM_SECTION;
static uint16_t vdd_buffer[VDD_BUFFER_SIZE] EXT_SRAM_SECTION;

uint16_t pixc2_0[NUMBER_OF_PIXEL/2] EXT_SRAM_SECTION;
uint16_t pixc2bot_0[NUMBER_OF_PIXEL/2] EXT_SRAM_SECTION;
static int32_t pixcij_0[NUMBER_OF_PIXEL] EXT_SRAM_SECTION;

static int16_t thgrad2_0[NUMBER_OF_PIXEL] EXT_SRAM_SECTION;
static int16_t thoff2_0[NUMBER_OF_PIXEL] EXT_SRAM_SECTION;

static int16_t vddgrad2_0[ROW_PER_BLOCK * 2 * PIXEL_PER_ROW] EXT_SRAM_SECTION;
static int16_t vddoff2_0[ROW_PER_BLOCK * 2 * PIXEL_PER_ROW] EXT_SRAM_SECTION;


/*  */


void init_buffers()
{
	proc = (processing_t){0};

	for (uint32_t i = 0; i < (ROW_PER_BLOCK * 2); i++) {
		for (uint32_t k=0; k< PIXEL_PER_ROW; k++) {
			eloffset[i][k] = 0;
		}
	}

	for (uint32_t i = 0; i < PTAT_BUFFER_SIZE; i++) {
		ptat_buffer[i] = 0;
	}
	for (uint32_t i = 0; i < VDD_BUFFER_SIZE; i++) {
		vdd_buffer[i] = 0;
	}
	for (uint32_t i = 0; i < NUMBER_OF_PIXEL/2; i++) {
		pixc2_0[i] = 0;
		pixc2bot_0[i] = 0;
	}
	for (uint32_t i = 0; i < NUMBER_OF_PIXEL; i++) {
		pixcij_0[i] = 0;
	}

	for (uint32_t i = 0; i < NUMBER_OF_PIXEL; i++) {
		thgrad2_0[i] = 0;
		thoff2_0[i] = 0;
	}

	for (uint32_t i = 0; i < ROW_PER_BLOCK * 2 * PIXEL_PER_ROW; i++) {
		vddgrad2_0[i] = 0;
		vddoff2_0[i] = 0;
	}
}


/********************************************************************
 Function:      calcPixC
 Description:   calculates the pixel constants with the unscaled
		values from EEPROM
*******************************************************************/
void calcPixC()
{

	/* uses the formula from datasheet:

			   PixC_uns[m][n]*(PixCmax-PixCmin)               epsilon   GlobalGain
	    PixC[m][n] = ( -------------------------------- + PixCmin ) * ------- * ----------
					65535                               100        1000
	*/

	for (uint32_t i = 0; i < NUMBER_OF_PIXEL / 2; i++) {
		unsigned long long pixcij = (unsigned long long)flash.pixcmax;

		pixcij -= (unsigned long long)flash.pixcmin;
		pixcij /= (unsigned long long)65535.0;
		pixcij *= (unsigned long long)pixc2_0[i];
		pixcij += (unsigned long long)flash.pixcmin;
		pixcij /= (unsigned long long)100.0;
		pixcij *= (unsigned long long)flash.epsilon;
		pixcij /= (unsigned long long)10000.0;
		pixcij *= (unsigned long long)flash.globalgain;
		pixcij_0[i] = (int32_t)pixcij;

		pixcij = (unsigned long long)flash.pixcmax;
		pixcij -= (unsigned long long)flash.pixcmin;
		pixcij /= (unsigned long long)65535.0;
		pixcij *= (unsigned long long)pixc2bot_0[i];
		pixcij += (unsigned long long)flash.pixcmin;
		pixcij /= (unsigned long long)100.0;
		pixcij *= (unsigned long long)flash.epsilon;
		pixcij /= (unsigned long long)10000.0;
		pixcij *= (unsigned long long)flash.globalgain;
		pixcij_0[NUMBER_OF_PIXEL / 2 + i] = (int32_t)pixcij;
	}

	flash.lastepsilon = flash.epsilon;
}

void init_calc()
{
	//*******************************************************************
	// do bigger calculation here before you jump into the loop() function
	//*******************************************************************
	proc.gradscale_div = 1U << flash.gradscale;
	proc.vddscgrad_div = 1U << flash.vddscgrad;
	proc.vddscoff_div = 1U << flash.vddscoff;

	calcPixC();

	//*******************************************************************
	// timer initialization
	//*******************************************************************
	//timert = calc_timert(clk_calib, mbit_calib);
	//TimerLib.setInterval_us(ISR, timert );

}

/********************************************************************
   Function:        void sort_data()
   Description:     sort the raw data blocks in 2d array and calculate ambient temperature, ptat and vdd
 *******************************************************************/
void sort_data()
{

	static uint32_t counter = 0;
	static uint32_t use_ptat_buffer = 0;
	static uint32_t use_vdd_buffer = 0;
	static uint8_t ptat_i = 0;
	static uint8_t vdd_i = 0;

	unsigned long sum = 0, sum2 = 0;
	unsigned short pos = 0;

	for (int m = 0; m < DevConst.RowPerBlock; m++) {
		for (int n = 0; n < DevConst.PixelPerRow; n++) {

			/*
			   for example: a normal line of RAMoutput for HTPAd80x64 looks like:
			   RAMoutput[0][] = [ PTAT(MSB), PTAT(LSB), DATA0[MSB], DATA0[LSB], DATA1[MSB], DATA1[LSB], ... , DATA640[MSB], DATA640LSB];
			                                                |
			                                                |-- DATA_Pos = 2 (first data byte)
			*/
			pos = (unsigned short)(
				2 * n + DevConst.DataPos + m * 2 * DevConst.PixelPerRow);

			/******************************************************************************************************************
			  new PIXEL values
			******************************************************************************************************************/
			for (int i = 0; i < DevConst.NumberOfBlocks; i++) {

				volatile uint16_t v_t = RAMoutput[i][pos] << 8 | RAMoutput[i][pos + 1];

				// top half
				data_pixel[m + i * DevConst.RowPerBlock][n] = (int32_t)(v_t);

				uint16_t v_b = RAMoutput[2 * DevConst.NumberOfBlocks + 2 - i - 1][
							pos] << 8 | RAMoutput[
							2 * DevConst.NumberOfBlocks + 2 - i - 1][
							pos + 1];
				// bottom half
				data_pixel[DevConst.PixelPerColumn - 1 - m - i * DevConst.
				           RowPerBlock][n] = (int32_t)(v_b);
			}

			/******************************************************************************************************************
			  new electrical offset values (store them in electrical offset buffer and calculate the average for pixel compensation
			******************************************************************************************************************/
			if (counter == 0) {
				if (eloffset[m][n] == 0) {
					// top half
					volatile uint16_t v_t =RAMoutput[DevConst.NumberOfBlocks][pos] << 8 |
						RAMoutput[DevConst.NumberOfBlocks][pos + 1];

					eloffset[m][n] = (int32_t)(v_t);

					// bottom half

					uint16_t v_b = RAMoutput[DevConst.NumberOfBlocks + 1][pos]
						       << 8 | RAMoutput[DevConst.NumberOfBlocks + 1]
						       [pos + 1];

					eloffset[2 * DevConst.RowPerBlock - 1 - m][n] = (int32_t)(v_b);


				} else {
					// use a moving average filter
					// top half
					sum = (unsigned long)eloffset[m][n] * (unsigned long)(
						      ELOFFSETS_BUFFER_SIZE - 1);
					sum += (unsigned long)(
						RAMoutput[DevConst.NumberOfBlocks][pos] << 8 |
						RAMoutput[DevConst.NumberOfBlocks][pos + 1]);
					eloffset[m][n] = (unsigned short)(
						(sum + ELOFFSETS_BUFFER_SIZE / 2) / ELOFFSETS_BUFFER_SIZE);
					// bottom half
					sum = (unsigned long)eloffset[
						      2 * DevConst.RowPerBlock - 1 - m][n] * (
						      unsigned long)(ELOFFSETS_BUFFER_SIZE - 1);
					sum += (unsigned long)(
						RAMoutput[DevConst.NumberOfBlocks + 1][pos] << 8 |
						RAMoutput[DevConst.NumberOfBlocks + 1][pos + 1]);
					eloffset[2 * DevConst.RowPerBlock - 1 - m][n] = (unsigned
						short)((sum + ELOFFSETS_BUFFER_SIZE / 2) / ELOFFSETS_BUFFER_SIZE);
				}
			}

		}

	}

	if (counter == 0) {
		/******************************************************************************************************************
		  new PTAT values (store them in PTAT buffer and calculate the average for pixel compensation
		******************************************************************************************************************/
		sum = 0;
		// calculate ptat average (datasheet, chapter: 11.1 Ambient Temperature )
		for (int i = 0; i < DevConst.NumberOfBlocks; i++) {
			// block top half
			sum += (unsigned short)(RAMoutput[i][DevConst.PTATPos] << 8 | RAMoutput[i][
							DevConst.PTATPos + 1]);
			// block bottom half
			sum += (unsigned short)(RAMoutput[2 * DevConst.NumberOfBlocks - i + 1][DevConst.
							PTATPos] << 8 | RAMoutput[
							2 * DevConst.NumberOfBlocks - i + 1][
							DevConst.PTATPos + 1]);
		}
		proc.ptat_av_uint16 = (unsigned short)((float)sum / (float)(2.0 * DevConst.NumberOfBlocks));
		proc.Ta = (unsigned short)((unsigned short)proc.ptat_av_uint16 * (float)flash.ptatgr_float + (float)
				      flash.ptatoff_float);

		ptat_buffer[ptat_i] = proc.ptat_av_uint16;
		ptat_i++;
		if (ptat_i == PTAT_BUFFER_SIZE) {
			if (use_ptat_buffer == 0) {
				//Serial.print(" | PTAT buffer complete");
				use_ptat_buffer = 1;
			}
			ptat_i = 0;
		}

		if (use_ptat_buffer) {
			// now overwrite the old ptat average
			sum = 0;
			for (int i = 0; i < PTAT_BUFFER_SIZE; i++) {
				sum += ptat_buffer[i];
			}
			proc.ptat_av_uint16 = (uint16_t)((float)sum / PTAT_BUFFER_SIZE);
		}

		/******************************************************************************************************************
		  new VDD values (store them in VDD buffer and calculate the average for pixel compensation
		******************************************************************************************************************/
		sum = 0;
		// calculate vdd average (datasheet, chapter: 11.4 Vdd Compensation )
		for (int i = 0; i < DevConst.NumberOfBlocks; i++) {
			// block top half
			sum += (unsigned short)(RAMoutput[i][DevConst.VDDPos] << 8 | RAMoutput[i][
							DevConst.VDDPos + 1]);
			// block bottom half
			sum += (unsigned short)(RAMoutput[2 * DevConst.NumberOfBlocks - i + 1][DevConst.
							VDDPos] << 8 | RAMoutput[
							2 * DevConst.NumberOfBlocks - i + 1][
							DevConst.VDDPos + 1]);
		}
		proc.vdd_av_uint16 = (unsigned short)((float)sum / (float)(2.0 * DevConst.NumberOfBlocks));

		// write into vdd buffer
		vdd_buffer[vdd_i] = proc.vdd_av_uint16;
		vdd_i++;
		if (vdd_i == VDD_BUFFER_SIZE) {
			if (use_vdd_buffer == 0) {
				//Serial.print(" | VDD buffer complete");
				use_vdd_buffer = 1;
			}
			vdd_i = 0;
		}
		if (use_vdd_buffer) {
			sum = 0;
			for (int i = 0; i < VDD_BUFFER_SIZE; i++) {
				sum += vdd_buffer[i];
			}
			// now overwrite the old vdd average
			proc.vdd_av_uint16 = (uint16_t)((float)sum / VDD_BUFFER_SIZE);
		}

		/******************************************************************************************************************
		  new ATC values (store them in VDD buffer and calculate the average for pixel compensation
		******************************************************************************************************************/
		sum = 0;
		sum2 = 0;
		for (int i = 0; i < DevConst.NumberOfBlocks; i++) {
			// block top half
			sum += (unsigned short)(RAMoutput[i][DevConst.VDDPos] << 8 | RAMoutput[i][
							DevConst.VDDPos + 1]);
			// block bottom half
			sum2 += (unsigned short)(
				RAMoutput[2 * DevConst.NumberOfBlocks - i][DevConst.VDDPos] << 8 | RAMoutput
				[2 * DevConst.NumberOfBlocks - i][DevConst.VDDPos + 1]);
		}
		proc.ATC0 = (unsigned short)((float)sum / (float)(DevConst.NumberOfBlocks));
		proc.ATC1 = (unsigned short)((float)sum2 / (float)(DevConst.NumberOfBlocks));
	}

	counter ++;
	if (counter > 4) {
		counter = 0;
	}

}


/********************************************************************
   Function:        void pixel_masking()
   Description:     repair dead pixel by using the average of the neighbors
 *******************************************************************/
void pixel_masking() {


  uint8_t number_neighbours[ALLOWED_DEADPIX];
  uint32_t temp_defpix[ALLOWED_DEADPIX];

  for (int i = 0; i < flash.nrofdefpix; i++) {
    number_neighbours[i] = 0;
    temp_defpix[i] = 0;

    // top half
    if (flash.deadpixadr[i] < (unsigned short)(NUMBER_OF_PIXEL / 2)) {

      if ( (flash.deadpixmask[i] & 1 )  == 1) {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + data_pixel[(flash.deadpixadr[i] / PIXEL_PER_ROW) - 1][(flash.deadpixadr[i] % PIXEL_PER_ROW)];
      }

      if ( (flash.deadpixmask[i] & 2 )  == 2 ) {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + data_pixel[(flash.deadpixadr[i] / PIXEL_PER_ROW) - 1][(flash.deadpixadr[i] % PIXEL_PER_ROW) + 1];
      }

      if ( (flash.deadpixmask[i] & 4 )  == 4 ) {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + data_pixel[(flash.deadpixadr[i] / PIXEL_PER_ROW)][(flash.deadpixadr[i] % PIXEL_PER_ROW) + 1];
      }

      if ( (flash.deadpixmask[i] & 8 )  == 8 ) {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + data_pixel[(flash.deadpixadr[i] / PIXEL_PER_ROW) + 1][(flash.deadpixadr[i] % PIXEL_PER_ROW) + 1];
      }

      if ( (flash.deadpixmask[i] & 16 )  == 16 ) {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + data_pixel[(flash.deadpixadr[i] / PIXEL_PER_ROW) + 1][(flash.deadpixadr[i] % PIXEL_PER_ROW)];
      }

      if ( (flash.deadpixmask[i] & 32 )  == 32 ) {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + data_pixel[(flash.deadpixadr[i] / PIXEL_PER_ROW) + 1][(flash.deadpixadr[i] % PIXEL_PER_ROW) - 1];
      }

      if ( (flash.deadpixmask[i] & 64 )  == 64 ) {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + data_pixel[(flash.deadpixadr[i] / PIXEL_PER_ROW)][(flash.deadpixadr[i] % PIXEL_PER_ROW) - 1];
      }

      if ( (flash.deadpixmask[i] & 128 )  == 128 ) {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + data_pixel[(flash.deadpixadr[i] / PIXEL_PER_ROW) - 1][(flash.deadpixadr[i] % PIXEL_PER_ROW) - 1];
      }

    }

    // bottom half
    else {

      if ( (flash.deadpixmask[i] & 1 )  == 1 ) {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + data_pixel[(flash.deadpixadr[i] / PIXEL_PER_ROW) + 1][(flash.deadpixadr[i] % PIXEL_PER_ROW)];
      }

      if ( (flash.deadpixmask[i] & 2 )  == 2 ) {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + data_pixel[(flash.deadpixadr[i] / PIXEL_PER_ROW) + 1][(flash.deadpixadr[i] % PIXEL_PER_ROW) + 1];
      }

      if ( (flash.deadpixmask[i] & 4 )  == 4 ) {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + data_pixel[(flash.deadpixadr[i] / PIXEL_PER_ROW)][(flash.deadpixadr[i] % PIXEL_PER_ROW) + 1];
      }

      if ( (flash.deadpixmask[i] & 8 )  == 8 ) {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + data_pixel[(flash.deadpixadr[i] / PIXEL_PER_ROW) - 1][(flash.deadpixadr[i] % PIXEL_PER_ROW) + 1];
      }

      if ( (flash.deadpixmask[i] & 16 )  == 16 ) {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + data_pixel[(flash.deadpixadr[i] / PIXEL_PER_ROW) - 1][(flash.deadpixadr[i] % PIXEL_PER_ROW)];
      }

      if ( (flash.deadpixmask[i] & 32 )  == 32 ) {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + data_pixel[(flash.deadpixadr[i] / PIXEL_PER_ROW) - 1][(flash.deadpixadr[i] % PIXEL_PER_ROW) - 1];
      }

      if ( (flash.deadpixmask[i] & 64 )  == 64 ) {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + data_pixel[(flash.deadpixadr[i] / PIXEL_PER_ROW)][(flash.deadpixadr[i] % PIXEL_PER_ROW) - 1];
      }

      if ( (flash.deadpixmask[i] & 128 )  == 128 ) {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + data_pixel[(flash.deadpixadr[i] / PIXEL_PER_ROW) + 1][(flash.deadpixadr[i] % PIXEL_PER_ROW) - 1];
      }
    }

    temp_defpix[i] = temp_defpix[i] / number_neighbours[i];
    data_pixel[flash.deadpixadr[i] / PIXEL_PER_ROW][flash.deadpixadr[i] % PIXEL_PER_ROW] = temp_defpix[i];

  }

}


/********************************************************************
   Function:        calculate_pixel_temp()
   Description:     compensate thermal, electrical offset and vdd and multiply sensitivity coeff
                    look for the correct temp in lookup table
 *******************************************************************/
void calculate_pixel_temp() {

  uint16_t table_col = 0;
  int32_t vx, vy, dta;
  int32_t *pixcij = pixcij_0;
  const int32_t ptat = proc.ptat_av_uint16;
  const int32_t vdd_ptat_slope = ((int32_t)flash.vddth2 - (int32_t)flash.vddth1) /
				  ((int32_t)flash.ptatth2 - (int32_t)flash.ptatth1);
  const int32_t vdd_delta = (int32_t)proc.vdd_av_uint16 - (int32_t)flash.vddth1 -
			    vdd_ptat_slope * (ptat - (int32_t)flash.ptatth1);

  /******************************************************************************************************************
    step 0: find column of lookup table
  ******************************************************************************************************************/
  for (int i = 0; i < NROFTAELEMENTS; i++) {
    if (proc.Ta > XTATemps[i]) {
      table_col = i;
    }
  }
  dta = proc.Ta - XTATemps[table_col];

  for (uint32_t m = 0; m < DevConst.PixelPerColumn; m++) {
    const uint32_t coeff_row = (m < DevConst.PixelPerColumn / 2) ?
			       (m % DevConst.RowPerBlock) :
			       (m % DevConst.RowPerBlock + DevConst.RowPerBlock);
    const int32_t *eloffset_row = eloffset[coeff_row];
    const int16_t *vddgrad_row = &vddgrad2_0[coeff_row * DevConst.PixelPerRow];
    const int16_t *vddoff_row = &vddoff2_0[coeff_row * DevConst.PixelPerRow];
    const int16_t *thgrad_row = &thgrad2_0[m * DevConst.PixelPerRow];
    const int16_t *thoff_row = &thoff2_0[m * DevConst.PixelPerRow];
    int32_t *pixel_row = data_pixel[m];

    for (uint32_t n = 0; n < DevConst.PixelPerRow; n++) {
      /******************************************************************************************************************
             step 1: use a variable with bigger data format for the compensation steps
      ******************************************************************************************************************/
      int32_t pixel = pixel_row[n];

      /******************************************************************************************************************
         step 2: compensate thermal drifts (see datasheet, chapter: Thermal Offset)
       ******************************************************************************************************************/
      pixel -= (int32_t)(((int32_t)thgrad_row[n] * ptat) / (int32_t)proc.gradscale_div);
      pixel -= (int32_t)thoff_row[n];


      /******************************************************************************************************************
         step 3: compensate electrical offset (see datasheet, chapter: Electrical Offset)
       ******************************************************************************************************************/
      pixel -= eloffset_row[n];

      /******************************************************************************************************************
         step 4: compensate vdd (see datasheet, chapter: Vdd Compensation)
       ******************************************************************************************************************/
      int64_t vdd_calc_steps = (int64_t)vddgrad_row[n] * ptat;
      vdd_calc_steps = vdd_calc_steps / proc.vddscgrad_div;
      vdd_calc_steps = vdd_calc_steps + vddoff_row[n];
      vdd_calc_steps = vdd_calc_steps * vdd_delta;
      vdd_calc_steps = vdd_calc_steps / proc.vddscoff_div;
      pixel -= (int32_t)vdd_calc_steps;

      /******************************************************************************************************************
         step 5: multiply sensitivity coeff for each pixel (see datasheet, chapter: Object Temperature)
       ******************************************************************************************************************/
      const int32_t pixcij_n = *pixcij++;

      if (pixcij_n != 0) {
        pixel = (int32_t)(((int64_t)pixel * (int64_t)PCSCALEVAL) / pixcij_n);
      }

      /******************************************************************************************************************
         step 6: find correct temp for this sensor in lookup table and do a bilinear interpolation (see datasheet, chapter:  Look-up table)
       ******************************************************************************************************************/
      const int32_t table_input = pixel + TABLEOFFSET;
      const uint16_t table_row = (uint16_t)table_input >> ADEXPBITS;

      // bilinear interpolation
      vx = ((((int32_t)TempTable[table_row][table_col + 1] - (int32_t)TempTable[table_row][table_col]) * (int32_t)dta) / (int32_t)TAEQUIDISTANCE) + (int32_t)TempTable[table_row][table_col];
      vy = ((((int32_t)TempTable[table_row + 1][table_col + 1] - (int32_t)TempTable[table_row + 1][table_col]) * (int32_t)dta) / (int32_t)TAEQUIDISTANCE) + (int32_t)TempTable[table_row + 1][table_col];
      pixel = (uint32_t)((vy - vx) * (table_input - (int32_t)YADValues[table_row]) / (int32_t)ADEQUIDISTANCE + (int32_t)vx);

      /******************************************************************************************************************
         step 7: add GlobalOffset (stored as signed char)
       ******************************************************************************************************************/
      pixel += flash.globaloff;
      /******************************************************************************************************************
        step 8: overwrite the uncompensate pixel with the new calculated compensated value
      ******************************************************************************************************************/
      pixel_row[n] = pixel;
    }
  }

  /******************************************************************************************************************
    step 8: overwrite the uncompensate pixel with the new calculated compensated value
  ******************************************************************************************************************/
  pixel_masking();

}

