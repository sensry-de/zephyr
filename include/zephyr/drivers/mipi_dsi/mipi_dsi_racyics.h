/*
 * Copyright (c) 2026 Racyics GmbH.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Racyics EdgeVision MIPI DSI2 host specific API
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_MIPI_DSI_MIPI_DSI_RACYICS_H_
#define ZEPHYR_INCLUDE_DRIVERS_MIPI_DSI_MIPI_DSI_RACYICS_H_

#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Switch the DSI host to command mode.
 *
 * The host must be attached. Packets sent with mipi_dsi_transfer() are only
 * accepted in command mode.
 *
 * @param dev DSI host device.
 *
 * @retval 0 On success.
 * @retval -ETIMEDOUT The automatic calculation of the host did not finish.
 */
int mipi_dsi_racyics_set_command_mode(const struct device *dev);

/**
 * @brief Enable the high speed link and switch the DSI host to video mode.
 *
 * Call this after the panel has been initialized in command mode.
 *
 * @param dev DSI host device.
 *
 * @retval 0 On success.
 */
int mipi_dsi_racyics_start_video(const struct device *dev);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_MIPI_DSI_MIPI_DSI_RACYICS_H_ */
