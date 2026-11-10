/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_PTP_CLOCK_SY1XX_H_
#define ZEPHYR_DRIVERS_PTP_CLOCK_SY1XX_H_

#include <zephyr/device.h>
#include <zephyr/net/ptp_time.h>

int sy1xx_ptp_clock_apply_offset(const struct device *dev, struct net_ptp_time *timestamp);

#endif /* ZEPHYR_DRIVERS_PTP_CLOCK_SY1XX_H_ */
