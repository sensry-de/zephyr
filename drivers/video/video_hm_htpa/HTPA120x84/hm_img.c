
#include <stdint.h>

#include "heimann/modes/sensors/hm_img.h"

#include <math.h>
#include <ext_sram.h>


#include "heimann/modes/sensors/hm_flash.h"
#include "zephyr/linker/section_tags.h"

typedef struct {

	uint16_t pixel_heap_buffer[NUMBER_OF_PIXEL];

	// use a heap allocated memory to store the pixc instead of a nxm array
	uint16_t *pixc2_0; // start address of the allocated heap memory
	uint16_t *pixc2;   // increasing address pointer

	uint16_t timert;

	uint16_t ptat_buffer[PTAT_BUFFER_SIZE];
	uint16_t vdd_buffer[VDD_BUFFER_SIZE];

	unsigned short Ta, ptat_av_uint16, vdd_av_uint16, ATC0, ATC1;
	//float ptatgr_float, ptatoff_float, pixcmin, pixcmax, bw;

	unsigned short eloffset[ROW_PER_BLOCK * 2][PIXEL_PER_ROW];
	uint32_t gradscale_div;
	uint32_t vddscgrad_div;
	uint32_t vddscoff_div;
	int vddcompgrad_n;
	int vddcompoff_n;

} processing_t;


uint16_t data_pixel[PIXEL_PER_COLUMN][PIXEL_PER_ROW] EXT_SRAM_SECTION;
uint8_t RAMoutput[2 * NUMBER_OF_BLOCKS + 2][FRAME_BLOCK_LENGTH] EXT_SRAM_SECTION;

extern heimann_flash_t flash;

static processing_t processing;

/*   */
__noinit __aligned(4) uint16_t pixc2_0[NUMBER_OF_PIXEL];


/*   */


void init_buffers()
{
	for (uint32_t i = 0; i < PTAT_BUFFER_SIZE; i++) {
		processing.ptat_buffer[i] = 0;
	}
	for (uint32_t i = 0; i < VDD_BUFFER_SIZE; i++) {
		processing.vdd_buffer[i] = 0;
	}
	for (uint32_t i = 0; i < (ROW_PER_BLOCK * 2); i++) {
		for (uint32_t k=0; k< PIXEL_PER_ROW; k++) {
			processing.eloffset[i][k] = 0;
		}
	}

	processing.Ta = 0;
	processing.ptat_av_uint16 = 0;
	processing.vdd_av_uint16 = 0;
	processing.ATC0 = 0;
	processing.ATC1 = 0;

	processing.gradscale_div = 0;
	processing.vddscgrad_div = 0;
	processing.vddscoff_div = 0;
	processing.vddcompgrad_n = 0;
	processing.vddcompoff_n = 0;



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

	double pixcij;
	uint16_t * pixc2 = pixc2_0; // set pointer to start address of the allocated heap

	for (int m = 0; m < DevConst.PixelPerColumn; m++) {
		for (int n = 0; n < DevConst.PixelPerRow; n++) {

			pixcij = (double)flash.pixcmax;
			pixcij -= (double)flash.pixcmin;
			pixcij /= (double)65535.0;
			pixcij *= (double)*pixc2;
			pixcij += (double)flash.pixcmin;
			pixcij /= (double)100.0;
			pixcij *= (double)flash.epsilon;
			pixcij /= (double)10000.0;
			pixcij *= (double)flash.globalgain;
			pixcij += 0.5;

			*pixc2 = (unsigned long)pixcij;
			pixc2++;
		}
	}

	// lastepsilon = flash.epsilon;
}


void init_calc()
{
	//*******************************************************************
	// do bigger calculation here before you jump into the loop() function
	//*******************************************************************
	processing.gradscale_div = pow(2, flash.gradscale);
	processing.vddscgrad_div = pow(2, flash.vddscgrad);
	processing.vddscoff_div = pow(2, flash.vddscoff);


	calcPixC(); // calculate the pixel constants

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
void sort_data() {
	static uint32_t pic_counter = 0;
	static uint32_t use_eloffsets_buffer = 0;
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
			pos = (unsigned short)(2 * n + DevConst.DataPos + m * 2 * DevConst.PixelPerRow);



			/******************************************************************************************************************
			  new PIXEL values
			******************************************************************************************************************/
			for (int i = 0; i < DevConst.NumberOfBlocks; i++) {
				// top half
				data_pixel[m + i * DevConst.RowPerBlock][n] =
					(unsigned short)(RAMoutput[i][pos] << 8 | RAMoutput[i][pos + 1]);
				// bottom half
				data_pixel[DevConst.PixelPerColumn - 1 - m - i * DevConst.RowPerBlock][n] =
					(unsigned short)(RAMoutput[2 * DevConst.NumberOfBlocks + 2 - i - 1][pos] << 8 | RAMoutput[2 * DevConst.NumberOfBlocks + 2 - i - 1][pos + 1]);
			}


			/******************************************************************************************************************
			  new electrical offset values (store them in electrical offset buffer and calculate the average for pixel compensation
			******************************************************************************************************************/
			if (pic_counter % ELOFFSETS_BUFFER_SIZE == 1) {
				if ((!processing.eloffset[m][n]) || (pic_counter < ELOFFSETS_FILTER_START_DELAY)) {
					// top half
					processing.eloffset[m][n] = (unsigned short)(RAMoutput[DevConst.NumberOfBlocks][pos] << 8 | RAMoutput[DevConst.NumberOfBlocks][pos + 1]);
					// bottom half
					processing.eloffset[2 * DevConst.RowPerBlock - 1 - m][n] = (unsigned short)(RAMoutput[DevConst.NumberOfBlocks + 1][pos] << 8 | RAMoutput[DevConst.NumberOfBlocks + 1][pos + 1]);
					use_eloffsets_buffer = 1;

				}
				else {
					// use a moving average filter
					// top half
					sum = (unsigned long)processing.eloffset[m][n] * (unsigned long)(ELOFFSETS_BUFFER_SIZE - 1);
					sum += (unsigned long)(RAMoutput[DevConst.NumberOfBlocks][pos] << 8 | RAMoutput[DevConst.NumberOfBlocks][pos + 1]);
					processing.eloffset[m][n] = (unsigned short)((float)sum / ELOFFSETS_BUFFER_SIZE + 0.5);
					// bottom half
					sum = (unsigned long)processing.eloffset[2 * DevConst.RowPerBlock - 1 - m][n] * (unsigned long)(ELOFFSETS_BUFFER_SIZE - 1);
					sum += (unsigned long)(RAMoutput[DevConst.NumberOfBlocks + 1][pos] << 8 | RAMoutput[DevConst.NumberOfBlocks + 1][pos + 1]);
					processing.eloffset[2 * DevConst.RowPerBlock - 1 - m][n] = (unsigned short)((float)sum / ELOFFSETS_BUFFER_SIZE + 0.5);
				}
			}


		}

	}



	/******************************************************************************************************************
	  new PTAT values (store them in PTAT buffer and calculate the average for pixel compensation
	******************************************************************************************************************/
	sum = 0;
	// calculate ptat average (datasheet, chapter: 11.1 Ambient Temperature )
	for (int i = 0; i < DevConst.NumberOfBlocks; i++) {
		// block top half
		sum += (unsigned short)(RAMoutput[i][DevConst.PTATPos] << 8 | RAMoutput[i][DevConst.PTATPos + 1]);
		// block bottom half
		sum += (unsigned short)(RAMoutput[2 * DevConst.NumberOfBlocks - i + 1][DevConst.PTATPos] << 8 | RAMoutput[2 * DevConst.NumberOfBlocks - i + 1][DevConst.PTATPos + 1]);
	}
	processing.ptat_av_uint16 = (unsigned short)((float)sum / (float)(2.0 * DevConst.NumberOfBlocks));
	processing.Ta = (unsigned short)((unsigned short)processing.ptat_av_uint16 * (float)flash.ptatgr_float + (float)flash.ptatoff_float);

#ifdef CONFIG_HEIMANN_FRAME_INFO
	LOG_DBG("Ta: %u", processing.Ta);
#endif

	processing.ptat_buffer[ptat_i] = processing.ptat_av_uint16;
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
			sum += processing.ptat_buffer[i];
		}
		processing.ptat_av_uint16 = (uint16_t)((float)sum / PTAT_BUFFER_SIZE);
	}





	/******************************************************************************************************************
	  new VDD values (store them in VDD buffer and calculate the average for pixel compensation
	******************************************************************************************************************/
	sum = 0;
	// calculate vdd average (datasheet, chapter: 11.4 Vdd Compensation )
	for (int i = 0; i < DevConst.NumberOfBlocks; i++) {
		// block top half
		sum += (unsigned short)(RAMoutput[i][DevConst.VDDPos] << 8 | RAMoutput[i][DevConst.VDDPos + 1]);
		// block bottom half
		sum += (unsigned short)(RAMoutput[2 * DevConst.NumberOfBlocks - i + 1][DevConst.VDDPos] << 8 | RAMoutput[2 * DevConst.NumberOfBlocks - i + 1][DevConst.VDDPos + 1]);
	}
	processing.vdd_av_uint16 = (unsigned short)((float)sum / (float)(2.0 * DevConst.NumberOfBlocks));

	float voltage = ((float) processing.vdd_av_uint16) / flash.vddscgrad + flash.vddscoff;
#ifdef CONFIG_HEIMANN_FRAME_INFO
	LOG_DBG("Vdd: %u, voltage: %.2f", processing.vdd_av_uint16, voltage);
#endif

	// write into vdd buffer
	processing.vdd_buffer[vdd_i] = processing.vdd_av_uint16;
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
			sum += processing.vdd_buffer[i];
		}
		// now overwrite the old vdd average
		processing.vdd_av_uint16 = (uint16_t)((float)sum / VDD_BUFFER_SIZE);
	}


	/******************************************************************************************************************
	  new ATC values (store them in VDD buffer and calculate the average for pixel compensation
	******************************************************************************************************************/
	sum = 0;
	sum2 = 0;
	for (int i = 0; i < DevConst.NumberOfBlocks; i++) {
		// block top half
		sum += (unsigned short)(RAMoutput[i][DevConst.VDDPos] << 8 | RAMoutput[i][DevConst.VDDPos + 1]);
		// block bottom half
		sum2 += (unsigned short)(RAMoutput[2 * DevConst.NumberOfBlocks - i][DevConst.VDDPos] << 8 | RAMoutput[2 * DevConst.NumberOfBlocks - i][DevConst.VDDPos + 1]);
	}
	processing.ATC0 = (unsigned short)((float)sum / (float)(DevConst.NumberOfBlocks));
	processing.ATC1 = (unsigned short)((float)sum2 / (float)(DevConst.NumberOfBlocks));


	pic_counter++;
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

	int64_t vij_pixc_and_pcscaleval;
	int64_t pixcij;
	int64_t vdd_calc_steps;
	uint16_t table_row, table_col;
	int32_t vx, vy, ydist, dta;
	signed long pixel;
	signed long pixelxx;


	/******************************************************************************************************************
	  step 0: find column of lookup table
	******************************************************************************************************************/
	for (int i = 0; i < NROFTAELEMENTS; i++) {
		if (processing.Ta > XTATemps[i]) {
			table_col = i;
		}
	}
	dta = processing.Ta - XTATemps[table_col];
	ydist = (int32_t)ADEQUIDISTANCE;


	for (int m = 0; m < DevConst.PixelPerColumn; m++) {
		for (int n = 0; n < DevConst.PixelPerRow; n++) {

			/******************************************************************************************************************
			   step 1: use a variable with bigger data format for the compensation steps
			 ******************************************************************************************************************/
			pixel = (signed long) data_pixel[m][n];
			pixelxx = pixel;

			/******************************************************************************************************************
			   step 2: compensate thermal drifts (see datasheet, chapter: Thermal Offset)
			 ******************************************************************************************************************/
			pixelxx -= (int32_t)(((int32_t)flash.thgrad[m][n] * (int32_t)processing.ptat_av_uint16) / (int32_t)processing.gradscale_div);
			pixelxx -= (int32_t)flash.thoffset[m][n];

			/******************************************************************************************************************
			   step 3: compensate electrical offset (see datasheet, chapter: Electrical Offset)
			 ******************************************************************************************************************/
			if (m < DevConst.PixelPerColumn / 2) { // top half
				pixelxx -= processing.eloffset[m % DevConst.RowPerBlock][n];
			}
			else { // bottom half
				pixelxx -= processing.eloffset[m % DevConst.RowPerBlock + DevConst.RowPerBlock][n];
			}

			/******************************************************************************************************************
			   step 4: compensate vdd (see datasheet, chapter: Vdd Compensation)
			 ******************************************************************************************************************/
			// first select VddCompGrad and VddCompOff for pixel m,n:
			if (m < DevConst.PixelPerColumn / 2) {      // top half
				processing.vddcompgrad_n = flash.vddcompgrad[m % DevConst.RowPerBlock][n];
				processing.vddcompoff_n = flash.vddcompoff[m % DevConst.RowPerBlock][n];
			}
			else {       // bottom half
				processing.vddcompgrad_n = flash.vddcompgrad[m % DevConst.RowPerBlock + DevConst.RowPerBlock][n];
				processing.vddcompoff_n = flash.vddcompoff[m % DevConst.RowPerBlock + DevConst.RowPerBlock][n];
			}
			// now do the vdd calculation
			vdd_calc_steps = processing.vddcompgrad_n * processing.ptat_av_uint16;
			vdd_calc_steps = vdd_calc_steps / processing.vddscgrad_div;
			vdd_calc_steps = vdd_calc_steps + processing.vddcompoff_n;
			vdd_calc_steps = vdd_calc_steps * ( processing.vdd_av_uint16 - flash.vddth1 - ((flash.vddth2 - flash.vddth1) / (flash.ptatth2 - flash.ptatth1)) * (processing.ptat_av_uint16  - flash.ptatth1));
			vdd_calc_steps = vdd_calc_steps / processing.vddscoff_div;
			pixelxx -= vdd_calc_steps;

			/******************************************************************************************************************
			   step 5: multiply sensitivity coeff for each pixel (see datasheet, chapter: Object Temperature)
			 ******************************************************************************************************************/
			vij_pixc_and_pcscaleval = pixel * (int64_t)PCSCALEVAL;
			pixel =  (int32_t)(vij_pixc_and_pcscaleval / *processing.pixc2);
			processing.pixc2++;
			/******************************************************************************************************************
			   step 6: find correct temp for this sensor in lookup table and do a bilinear interpolation (see datasheet, chapter:  Look-up table)
			 ******************************************************************************************************************/
			table_row = pixelxx + TABLEOFFSET;
			table_row = table_row >> ADEXPBITS;
			// bilinear interpolation
			vx = ((((int32_t)TempTable[table_row][table_col + 1] - (int32_t)TempTable[table_row][table_col]) * (int32_t)dta) / (int32_t)TAEQUIDISTANCE) + (int32_t)TempTable[table_row][table_col];
			vy = ((((int32_t)TempTable[table_row + 1][table_col + 1] - (int32_t)TempTable[table_row + 1][table_col]) * (int32_t)dta) / (int32_t)TAEQUIDISTANCE) + (int32_t)TempTable[table_row + 1][table_col];
			pixelxx = (uint32_t)((vy - vx) * ((int32_t)(pixelxx + TABLEOFFSET) - (int32_t)YADValues[table_row]) / ydist + (int32_t)vx);

			/******************************************************************************************************************
			   step 7: add GlobalOffset (stored as signed char)
			 ******************************************************************************************************************/
			pixelxx += flash.globaloff;

			/******************************************************************************************************************
			  step 8: overwrite the uncompensate pixel with the new calculated compensated value
			******************************************************************************************************************/
			data_pixel[m][n] = (unsigned short)pixelxx;

		}
	}

	/******************************************************************************************************************
	  step 8: overwrite the uncompensate pixel with the new calculated compensated value
	******************************************************************************************************************/
	pixel_masking();

}
