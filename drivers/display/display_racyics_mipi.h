/*
 * Copyright (c) 2026 Racyics GmbH.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _RI_FW_MIPI_DISPLAY_H
#define _RI_FW_MIPI_DISPLAY_H


#define DCU_AXIBUSWORDS_BYTES 16
typedef enum {
    RI_MIPI_DISP_TYPE_UNDEFINED,
    RI_MIPI_DISP_TYPE_GENERIC,
    RI_MIPI_DISP_TYPE_TFT_F_AND_S
} ri_mipi_disp_type_t;

typedef struct {
    uint16_t active_lines;
    uint16_t front_porch;
    uint16_t sync_pulse;
    uint16_t back_porch;
} ri_mipi_disp_side_parameters_t;


#define RI_MIPI_DISP_ACTIVE_WIDTH_TFT_F_AND_S   800
#define RI_MIPI_DISP_ACTIVE_HEIGTH_TFT_F_AND_S  1280

#endif /*_RI_FW_MIPI_DISPLAY_H*/