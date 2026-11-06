/*
 * Copyright (c) 2026 Racyics GmbH.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_DISPLAY_DISPLAY_RACYICS_MIPI_H_
#define ZEPHYR_DRIVERS_DISPLAY_DISPLAY_RACYICS_MIPI_H_

/* Width in bytes of the AXI bus of the display controller unit */
#define DCU_AXIBUSWORDS_BYTES	16U

/* Blanking timings of the F&S T070QYH-13CP (NT35521) panel */
#define RI_MIPI_DISP_H_FRONT_PORCH	56U
#define RI_MIPI_DISP_H_SYNC_PULSE	128U
#define RI_MIPI_DISP_H_BACK_PORCH	72U
#define RI_MIPI_DISP_V_FRONT_PORCH	3U
#define RI_MIPI_DISP_V_SYNC_PULSE	10U
#define RI_MIPI_DISP_V_BACK_PORCH	38U

#endif /* ZEPHYR_DRIVERS_DISPLAY_DISPLAY_RACYICS_MIPI_H_ */
