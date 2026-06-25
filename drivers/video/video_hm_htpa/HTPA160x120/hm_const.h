//
// Created by tswaehn on 6/2/26.
//

#ifndef HEIMANN_APP_HTPA160x120_HM_CONST_H
#define HEIMANN_APP_HTPA160x120_HM_CONST_H

// STREAMING SETTING
#define PTAT_BUFFER_SIZE 10
#define VDD_BUFFER_SIZE 10
#define ELOFFSETS_BUFFER_SIZE 10
#define ELOFFSETS_FILTER_START_DELAY  10
#define START_WITH_BLOCK 12 // start with electrical offsets
#define READ_ELOFFSET_EVERYX 10

// PARAMETER LOOKUPTABLE
#define TABLENUMBER 171      // table number of this sensor type
#define PCSCALEVAL 100000000 // scale value for PixC (see formula in datasheet)
#define NROFTAELEMENTS 13    // number of columns (ambient temperature steps)
#define NROFADELEMENTS 1597  // number of rows (digit steps)
#define TAEQUIDISTANCE 100   // distance between two columns
#define ADEQUIDISTANCE 64    // distance between two rows
#define ADEXPBITS 6
#define TABLEOFFSET 2176     // table offset in digits


// SENSOR INFO
#define NUMBER_OF_PIXEL 19200 // number of all pixels
#define NUMBER_OF_BLOCKS 12 // number of blocks for each half
#define PIXEL_PER_BLOCK 800 // number of pixels of each block
#define PIXEL_PER_COLUMN 120 // number of pixels of each column
#define PIXEL_PER_ROW 160 // number of pixels of each row
#define ROW_PER_BLOCK 5 // number of rows of each block
#define ALLOWED_DEADPIX 96 // max. 0.5% of the pixel number
#define ATC_ACTIVE 1 // 1 - sensor has... / 0 - sensor hasn't a cyclops
#define ATC_POS 0 // postion of the cyclops in block reading order
#define PTAT_POS 2 // position of PTAT in block reading order
#define VDD_POS 4 // postion of VDD in block reading order
#define PTAT_VDD_SWITCH 0 // 1 - PTAT and VDD alternate after each pic / 0 - not
#define BLOCK_LENGTH 1606 // number of char in each block
#define DATA_POS 6  // position of first data byte in each block


// SENSOR REGISTER
#define TOP_HALF 0x0A // data of top half
#define BOTTOM_HALF 0x0B // data of bot half
#define CONFIGURATION_REGISTER 0x01 // configuration register (WRITE only)
// |  7  |  6  |  5  |  4  |   3   |    2     |   1   |    0   |
// |    RFU    |   Block   | Start | VDD_MEAS | BLIND | WAKEUP |
#define STATUS_REGISTER 0x02 // adress of status register (READ only)
// |  7  |  6  |  5  |  4  |   3   |    2     |   1   |    0   |
// |    RFU    |   Block   |  RFU  | VDD_MEAS | BLIND |   EOC  |
#define TRIM_REGISTER1 0x03 // adress for trim register 1 (WRITE only)
// |  7  |  6  |  5  |  4  |  3  |  2  |  1  |  0  |
// |    RFU    |  REF_CAL  |         MBIT          |
#define TRIM_REGISTER2 0x04 // adress for trim register 2 (WRITE only)
// |  7  |  6  |  5  |  4  |  3  |  2  |  1  |  0  |
// |       RFU       |       BIAS TRIM TOP         |
#define TRIM_REGISTER3 0x05 // adress for trim register 3 (WRITE only)
// |  7  |  6  |  5  |  4  |  3  |  2  |  1  |  0  |
// |       RFU       |       BIAS TRIM BOT         |
#define TRIM_REGISTER4 0x06 // adress for trim register 4 (WRITE only)
// |  7  |  6  |  5  |  4  |  3  |  2  |  1  |  0  |
// |       RFU |             CLK TRIM              |
#define TRIM_REGISTER5 0x07 // adress for trim register 5 (WRITE only)
// |  7  |  6  |  5  |  4  |  3  |  2  |  1  |  0  |
// |       RFU       |        BPA TRIM TOP         |
#define TRIM_REGISTER6 0x08 // adress for trim register 6 (WRITE only)
// |  7  |  6  |  5  |  4  |  3  |  2  |  1  |  0  |
// |       RFU       |       BPA TRIM BOT          |
#define TRIM_REGISTER7 0x09 // adress for trim register 7 (WRITE only)
// |  7  |  6  |  5  |  4  |  3  |  2  |  1  |  0  |
// |       PU SDA TRIM     |       PU SDA TRIM     |


// FLASH REGISTER
#define E_PIXCMIN_1 0x0000 // Min of PixC, stored as float in four 8-bit fields
#define E_PIXCMIN_2 0x0001
#define E_PIXCMIN_3 0x0002
#define E_PIXCMIN_4 0x0003
#define E_PIXCMAX_1 0x0004// Max of PixC, stored as float in four 8-bit fields
#define E_PIXCMAX_2 0x0005
#define E_PIXCMAX_3 0x0006
#define E_PIXCMAX_4 0x0007
#define E_GRADSCALE 0x0008 // stored as unsigned char, important for PTAT comp.
#define E_TABLENUMBER1 0x000B // table number of this sensor, stored as unsigned short
#define E_TABLENUMBER2 0x000C
#define E_EPSILON 0x000D // emissivity in percentage, stored as unsigned char
#define E_MBIT_CALIB 0x001A // calibration settings of trim register 1
#define E_BIAS_CALIB 0x001B // calibration settings of trim register 2 & 3
#define E_CLK_CALIB 0x001C // calibration settings of trim register 4
#define E_BPA_CALIB 0x001D // calibration settings of trim register 5 & 6
#define E_PU_CALIB 0x001E // calibration settings of trim register 7
#define E_ARRAYTYPE 0x0022 // array type of this sensor
#define E_VDDTH1_1 0x0026 // HS calibration value
#define E_VDDTH1_2 0x0027 // HS calibration value
#define E_VDDTH2_1 0x0028 // HS calibration value
#define E_VDDTH2_2 0x0029 // HS calibration value
#define E_PTATGR_1 0x0034 // PTAT gradient stored as float in four 8-bit fields
#define E_PTATGR_2 0x0035
#define E_PTATGR_3 0x0036
#define E_PTATGR_4 0x0037
#define E_PTATOFF_1 0x0038 // PTAT offset stored as float in four 8-bit fields
#define E_PTATOFF_2 0x0039
#define E_PTATOFF_3 0x003A
#define E_PTATOFF_4 0x003B
#define E_PTATTH1_1 0x003C // HS calibration value
#define E_PTATTH1_2 0x003D // HS calibration value
#define E_PTATTH2_1 0x003E // HS calibration value
#define E_PTATTH2_2 0x003F // HS calibration value
#define E_VDDSCGRAD 0x004E // HS calibration value
#define E_VDDSCOFF 0x004F // HS calibration value
#define E_GLOBALOFF 0x0054 // global offset, stored as signed char
#define E_GLOBALGAIN_1 0x0055 // global gain, stored as unsigned short
#define E_GLOBALGAIN_2 0x0056
#define E_MBIT_USER 0x0060 // user settings of trim register 1
#define E_BIAS_USER 0x0061 // user settings of trim register 2 & 3
#define E_CLK_USER 0x0062 // user settings of trim register 4
#define E_BPA_USER 0x0063 // user settings of trim register 5 & 6
#define E_PU_USER 0x0064 // user settings of trim register 7
#define E_ID1 0x0074 // sensor idenification number, stored as unsigned long (1 of 4)
#define E_ID2 0x0075
#define E_ID3 0x0076
#define E_ID4 0x0077
#define E_NROFDEFPIX 0x007F // number of defekt pixel, stored as unsigned char
#define E_DEADPIXADR 0x0080 // first register of dead adress
#define E_DEADPIXMASK 0x0140 // first register of dead pixel mask
#define E_VDDCOMPGRAD 0x2100 // first register of VDD comp gradient
#define E_VDDCOMPOFF 0x2D80 // first register ofVDD comp offset
#define E_THGRAD 0x3A00 // first register of thermal gradient
#define E_THOFFSET 0xD000 // first register of thermal offset
#define E_PIJ 0x16600 // first register of pixel constants (PixC)
#define FLASH_SIZE 0x45400 // total number of EEPROM bytes


#endif //HEIMANN_APP_HTPA160x120_HM_CONST_H
