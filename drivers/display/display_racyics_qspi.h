/*
 * Copyright (c) 2026 Racyics GmbH.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _RI_FW_QSPI_DISPLAY_H
#define _RI_FW_QSPI_DISPLAY_H

typedef struct {
    uint8_t    cpha;
    uint8_t    cpol;
    uint8_t    cdiv;
    uint8_t    css;
    uint8_t    csh;
    uint8_t    csi;
    bool       is_open;   //!< flag to indicate that this interface is open.
} ri_qspi_dev_t;

typedef struct {
    uint32_t px_bytes_num;
    uint8_t wr_len;
    uint8_t rd_len;
    uint8_t wr_data_byte_0;
    uint8_t wr_data_byte_1;
    uint8_t wr_data_byte_2;
    uint8_t wr_data_byte_3;
    uint8_t wr_data_byte_4;
    uint8_t wr_data_byte_5;
    uint8_t wr_data_byte_6;
    uint8_t wr_data_byte_7;
    uint8_t wr_data_byte_8;
    uint8_t wr_data_byte_9;
    uint8_t wr_data_byte_10;
    uint8_t wr_data_byte_11;
    uint8_t wr_data_byte_12;
    uint8_t wr_data_byte_13;
    uint8_t wr_data_byte_14;
    uint8_t wr_data_byte_15;
    bool    valid;
} ri_qspi_job_t;

typedef struct {
    uint8_t read_en;
    uint8_t address;
    uint8_t len;
    uint8_t wr_data_byte_0;
    uint8_t wr_data_byte_1;
    uint8_t wr_data_byte_2;
    uint8_t wr_data_byte_3;
} ri_qspi_display_cmd_t;

typedef enum {
    PIXEL_SIZE08 = 1,   // one pixel has 1 byte
    PIXEL_SIZE16 = 2,   // one pixel has 2 byte
    PIXEL_SIZE24 = 3,   // one pixel has 3 byte
    PIXEL_SIZE32 = 4    // one pixel has 4 byte
} ri_pixel_size_e;

#endif /*_RI_FW_QSPI_DISPLAY_H*/