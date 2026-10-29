/*
 * Copyright (c) 2026 sensry.io
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT sensry_sy1xx_ptp_clock

#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/drivers/ptp_clock.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/sys_io.h>

LOG_MODULE_REGISTER(sy1xx_ptp_clock);

struct sy1xx_ptp_clock_config {
	uintptr_t base_addr;
};

struct sy1xx_ptp_clock_data {
	struct k_mutex mutex;
};

static int sy1xx_ptp_clock_set(const struct device *dev, struct net_ptp_time *tm)
{
	const struct sy1xx_ptp_clock_config *cfg = dev->config;

	/* TODO: Write seconds and nanoseconds to the PTP clock registers. */
	(void)cfg;
	(void)tm;

	return -ENOSYS;
}

static int sy1xx_ptp_clock_get(const struct device *dev, struct net_ptp_time *tm)
{
	const struct sy1xx_ptp_clock_config *cfg = dev->config;

	/* TODO: Read seconds and nanoseconds from the PTP clock registers. */
	(void)cfg;
	(void)tm;

	return -ENOSYS;
}

static int sy1xx_ptp_clock_adjust(const struct device *dev, int increment)
{
	const struct sy1xx_ptp_clock_config *cfg = dev->config;

	/* TODO: Apply a signed nanosecond adjustment using the hardware registers. */
	(void)cfg;
	(void)increment;

	return -ENOSYS;
}

static int sy1xx_ptp_clock_rate_adjust(const struct device *dev, double ratio)
{
	const struct sy1xx_ptp_clock_config *cfg = dev->config;

	/* TODO: Convert the rate ratio to the PTP peripheral's frequency controls. */
	(void)cfg;
	(void)ratio;

	return -ENOSYS;
}

static DEVICE_API(ptp_clock, sy1xx_ptp_clock_api) = {
	.set = sy1xx_ptp_clock_set,
	.get = sy1xx_ptp_clock_get,
	.adjust = sy1xx_ptp_clock_adjust,
	.rate_adjust = sy1xx_ptp_clock_rate_adjust,
};

static int sy1xx_ptp_clock_init(const struct device *dev)
{
	struct sy1xx_ptp_clock_data *data = dev->data;

	k_mutex_init(&data->mutex);

	return 0;
}

#define SY1XX_PTP_CLOCK_INIT(inst)                                                              \
	static const struct sy1xx_ptp_clock_config sy1xx_ptp_clock_##inst##_config = {            \
		.base_addr = DT_INST_REG_ADDR(inst),                                                 \
	};                                                                                         \
	static struct sy1xx_ptp_clock_data sy1xx_ptp_clock_##inst##_data;                          \
	DEVICE_DT_INST_DEFINE(inst, sy1xx_ptp_clock_init, NULL,                                    \
			      &sy1xx_ptp_clock_##inst##_data,                                       \
			      &sy1xx_ptp_clock_##inst##_config, POST_KERNEL,                         \
			      CONFIG_PTP_CLOCK_INIT_PRIORITY, &sy1xx_ptp_clock_api);

DT_INST_FOREACH_STATUS_OKAY(SY1XX_PTP_CLOCK_INIT)
