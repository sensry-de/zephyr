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
#include <zephyr/sys/util.h>

/* RTC register offsets, relative to the TSN_CTRL RTC base at offset 0x0800. */
#define SY1XX_RTC_SR		0x004U
#define SY1XX_RTC_CURTIME_NS	0x010U
#define SY1XX_RTC_CURTIME_SEC	0x014U
#define SY1XX_RTC_TIMER_INCR	0x01cU
#define SY1XX_RTC_OFS_NS	0x020U
#define SY1XX_RTC_OFS_SEC_LO	0x024U
#define SY1XX_RTC_OFS_SEC_HI	0x028U

/* RTC fields: CTVAL indicates a completed snapshot; time fields are 30/32 bits. */
#define SY1XX_RTC_SR_CTVAL	BIT(31)
#define SY1XX_RTC_NS_MASK	GENMASK(29, 0)
#define SY1XX_RTC_SEC_HI_MASK	GENMASK(15, 0)
#define SY1XX_RTC_TIMER_INCR_MASK	UINT32_MAX
#define SY1XX_RTC_NOMINAL_INCREMENT_NS	2U
#define SY1XX_RTC_SNAPSHOT_TIMEOUT_US	1000U
#define SY1XX_PTP_SEC_MAX	GENMASK64(47, 0)

LOG_MODULE_REGISTER(sy1xx_ptp_clock);

struct sy1xx_ptp_clock_config {
	uintptr_t base_addr;
};

struct sy1xx_ptp_clock_data {
	struct k_mutex mutex;
	uint32_t nominal_timer_increment;
};

static int sy1xx_ptp_clock_read_local_time(const struct sy1xx_ptp_clock_config *cfg,
						  struct net_ptp_time *tm)
{
	uint32_t status;
	uint32_t timeout = SY1XX_RTC_SNAPSHOT_TIMEOUT_US;

	/* CURTIME_NS is write-to-update: writing it requests an atomic snapshot. */
	sys_write32(0U, cfg->base_addr + SY1XX_RTC_CURTIME_NS);
	do {
		status = sys_read32(cfg->base_addr + SY1XX_RTC_SR);
		if ((status & SY1XX_RTC_SR_CTVAL) != 0U) {
			break;
		}
		k_busy_wait(1U);
	} while (timeout-- > 0U);

	if ((status & SY1XX_RTC_SR_CTVAL) == 0U) {
		return -ETIMEDOUT;
	}

	tm->nanosecond = sys_read32(cfg->base_addr + SY1XX_RTC_CURTIME_NS) &
			 SY1XX_RTC_NS_MASK;
	tm->second = sys_read32(cfg->base_addr + SY1XX_RTC_CURTIME_SEC);

	return 0;
}

static int sy1xx_ptp_clock_read_offset(const struct sy1xx_ptp_clock_config *cfg,
					      struct net_ptp_time *tm)
{
	uint32_t seconds_lo;
	uint32_t seconds_hi;

	tm->nanosecond = sys_read32(cfg->base_addr + SY1XX_RTC_OFS_NS) &
			 SY1XX_RTC_NS_MASK;
	seconds_lo = sys_read32(cfg->base_addr + SY1XX_RTC_OFS_SEC_LO);
	seconds_hi = sys_read32(cfg->base_addr + SY1XX_RTC_OFS_SEC_HI) &
		     SY1XX_RTC_SEC_HI_MASK;
	tm->second = ((uint64_t)seconds_hi << 32U) | seconds_lo;

	return 0;
}

static int sy1xx_ptp_clock_write_offset(const struct sy1xx_ptp_clock_config *cfg,
					       const struct net_ptp_time *tm)
{
	if ((tm->second > SY1XX_PTP_SEC_MAX) || (tm->nanosecond >= NSEC_PER_SEC)) {
		return -EINVAL;
	}

	sys_write32(tm->nanosecond, cfg->base_addr + SY1XX_RTC_OFS_NS);
	sys_write32((uint32_t)tm->second, cfg->base_addr + SY1XX_RTC_OFS_SEC_LO);
	sys_write32((uint32_t)(tm->second >> 32U), cfg->base_addr + SY1XX_RTC_OFS_SEC_HI);

	return 0;
}

static int sy1xx_ptp_clock_read_time(const struct sy1xx_ptp_clock_config *cfg,
					    struct net_ptp_time *tm)
{
	struct net_ptp_time local_time;
	struct net_ptp_time offset;
	int ret;

	ret = sy1xx_ptp_clock_read_local_time(cfg, &local_time);
	if (ret != 0) {
		return ret;
	}
	(void)sy1xx_ptp_clock_read_offset(cfg, &offset);

	tm->second = local_time.second + offset.second;
	tm->nanosecond = local_time.nanosecond + offset.nanosecond;
	if (tm->nanosecond >= NSEC_PER_SEC) {
		tm->nanosecond -= NSEC_PER_SEC;
		tm->second++;
	}

	return (tm->second > SY1XX_PTP_SEC_MAX) ? -ERANGE : 0;
}

static int sy1xx_ptp_clock_set_time(const struct sy1xx_ptp_clock_config *cfg,
					   const struct net_ptp_time *tm)
{
	struct net_ptp_time local_time;
	struct net_ptp_time offset;
	int ret;

	if ((tm->second > SY1XX_PTP_SEC_MAX) || (tm->nanosecond >= NSEC_PER_SEC)) {
		return -EINVAL;
	}

	ret = sy1xx_ptp_clock_read_local_time(cfg, &local_time);
	if (ret != 0) {
		return ret;
	}
	if (tm->second < local_time.second ||
	    (tm->second == local_time.second && tm->nanosecond < local_time.nanosecond)) {
		return -ERANGE;
	}

	offset.second = tm->second - local_time.second;
	if (tm->nanosecond >= local_time.nanosecond) {
		offset.nanosecond = tm->nanosecond - local_time.nanosecond;
	} else {
		offset.second--;
		offset.nanosecond = NSEC_PER_SEC + tm->nanosecond - local_time.nanosecond;
	}

	return sy1xx_ptp_clock_write_offset(cfg, &offset);
}

static int sy1xx_ptp_clock_set(const struct device *dev, struct net_ptp_time *tm)
{
	const struct sy1xx_ptp_clock_config *cfg = dev->config;
	struct sy1xx_ptp_clock_data *data = dev->data;
	int ret;

	k_mutex_lock(&data->mutex, K_FOREVER);
	ret = sy1xx_ptp_clock_set_time(cfg, tm);
	k_mutex_unlock(&data->mutex);

	return ret;
}

static int sy1xx_ptp_clock_get(const struct device *dev, struct net_ptp_time *tm)
{
	const struct sy1xx_ptp_clock_config *cfg = dev->config;
	struct sy1xx_ptp_clock_data *data = dev->data;
	int ret;

	k_mutex_lock(&data->mutex, K_FOREVER);
	ret = sy1xx_ptp_clock_read_time(cfg, tm);
	k_mutex_unlock(&data->mutex);

	return ret;
}

static int sy1xx_ptp_clock_adjust(const struct device *dev, int increment)
{
	const struct sy1xx_ptp_clock_config *cfg = dev->config;
	struct sy1xx_ptp_clock_data *data = dev->data;
	struct net_ptp_time offset;
	uint32_t magnitude;
	int ret;

	k_mutex_lock(&data->mutex, K_FOREVER);
	ret = sy1xx_ptp_clock_read_offset(cfg, &offset);
	if (ret == 0) {
		if (increment >= 0) {
			magnitude = (uint32_t)increment;
			offset.second += magnitude / NSEC_PER_SEC;
			magnitude %= NSEC_PER_SEC;
			offset.nanosecond += magnitude;
			if (offset.nanosecond >= NSEC_PER_SEC) {
				offset.nanosecond -= NSEC_PER_SEC;
				offset.second++;
			}
		} else {
			magnitude = (uint32_t)(-(int64_t)increment);
			if (offset.second < magnitude / NSEC_PER_SEC) {
				ret = -ERANGE;
			} else {
				offset.second -= magnitude / NSEC_PER_SEC;
				magnitude %= NSEC_PER_SEC;
				if (offset.nanosecond >= magnitude) {
					offset.nanosecond -= magnitude;
				} else if (offset.second > 0U) {
					offset.second--;
					offset.nanosecond = NSEC_PER_SEC + offset.nanosecond - magnitude;
				} else {
					ret = -ERANGE;
				}
			}
		}
		if (ret == 0) {
			ret = sy1xx_ptp_clock_write_offset(cfg, &offset);
		}
	}
	k_mutex_unlock(&data->mutex);

	return ret;
}

static int sy1xx_ptp_clock_rate_adjust(const struct device *dev, double ratio)
{
	const struct sy1xx_ptp_clock_config *cfg = dev->config;
	struct sy1xx_ptp_clock_data *data = dev->data;
	uint32_t increment;
	double adjusted;
	int ret = 0;

	if ((ratio <= 0.0) || (ratio != ratio)) {
		return -EINVAL;
	}

	k_mutex_lock(&data->mutex, K_FOREVER);
	increment = data->nominal_timer_increment;
	adjusted = (double)increment * ratio;
	if ((adjusted < 1.0) || (adjusted > (double)SY1XX_RTC_TIMER_INCR_MASK)) {
		ret = -ERANGE;
	} else {
		/* NS occupies bits 31:24; FNS is the 24-bit fractional part. */
		sys_write32((uint32_t)adjusted, cfg->base_addr + SY1XX_RTC_TIMER_INCR);
	}
	k_mutex_unlock(&data->mutex);

	return ret;
}

static DEVICE_API(ptp_clock, sy1xx_ptp_clock_api) = {
	.set = sy1xx_ptp_clock_set,
	.get = sy1xx_ptp_clock_get,
	.adjust = sy1xx_ptp_clock_adjust,
	.rate_adjust = sy1xx_ptp_clock_rate_adjust,
};

static int sy1xx_ptp_clock_init(const struct device *dev)
{
	const struct sy1xx_ptp_clock_config *cfg = dev->config;
	struct sy1xx_ptp_clock_data *data = dev->data;

	k_mutex_init(&data->mutex);
	/* The TSN RTC timer runs at 500 MHz: 2 ns per timer tick. */
	data->nominal_timer_increment = SY1XX_RTC_NOMINAL_INCREMENT_NS << 24U;
	sys_write32(data->nominal_timer_increment, cfg->base_addr + SY1XX_RTC_TIMER_INCR);

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
