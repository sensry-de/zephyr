/*
 * Copyright (c) 2026 Racyics GmbH.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT racyics_edgevision_i2c

#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <soc.h>

LOG_MODULE_REGISTER(i2c_racyics_edgevision, CONFIG_I2C_LOG_LEVEL);

#define RI_I2C_REF_CLOCK_HZ      40000000UL
#define RI_I2C_SPEED_STANDARD_HZ 100000U
#define RI_I2C_SPEED_FAST_HZ     400000U
#define RI_I2C_SPEED_FAST_PLUS_HZ 1000000U

#define I2C_ADDR_TIMEOUT_MS 10000
#define I2C_BUS_TIMEOUT_MS  1000

struct i2c_edgevision_config {
	uint32_t clock_frequency;
	void (*irq_config_func)(const struct device *dev);
};

struct i2c_edgevision_data {
	struct k_sem bus_sem;
	struct k_sem irq_sem;
	uint32_t     irq_status;
	uint32_t     irq_wait_mask;
	int64_t      last_fault_log_ms;
	bool sccb_mode;
};

static uint32_t wait_for_irq(struct i2c_edgevision_data *data, uint32_t timeout)
{
	/* Wait for transmission  to happen, semaphore give in ISR */
	if (k_sem_take(&data->irq_sem, K_MSEC(timeout)) != 0) {
		return 0;
	}
	/* Read+clear under lock: ISR may OR a fused event in after the give */
	unsigned int key = irq_lock();
	uint32_t status = data->irq_status;

	data->irq_status = 0;
	irq_unlock(key);
	return status;
}

static void i2c_send_stop(struct i2c_edgevision_data *data)
{
	/* Clear all stale flags so wait_for_irq sees only this STOP */
	WRITEREG(I2C_MST_IRQ_STATUS_REG,
		 RI_I2C_REG__IRQ_STATUS__ACK__M    |
		 RI_I2C_REG__IRQ_STATUS__NACK__M   |
		 RI_I2C_REG__IRQ_STATUS__STOP__M   |
		 RI_I2C_REG__IRQ_STATUS__ARBLOST__M);
	data->irq_wait_mask = RI_I2C_REG__IRQ_STATUS__STOP__M;
	WRITEREG(I2C_MST_CMD_REG, RI_I2C_REG__CMD__STOP__M);
	if (wait_for_irq(data, I2C_BUS_TIMEOUT_MS) == 0) {
		LOG_ERR("I2C: STOP did not complete");
	}
}

static void i2c_hw_configure(uint32_t bus_hz)
{
	WRITEREG(I2C_MST_CTRL_REG, 0U);
	/* Set bus CLK prescaler, TODO verify correctness for current FPGA */
	WRITEREG(I2C_MST_CLK_PRESCALER_REG, (RI_I2C_REF_CLOCK_HZ / (5U * bus_hz)) - 1U);

	WRITEREG(I2C_MST_CTRL_REG, RI_I2C_REG__CTRL__CORE_EN__M);

	/* Enable all interrupts in HW*/
	WRITEREG(I2C_MST_IRQ_STATUS_ENABLE_REG,
		 RI_I2C_REG__IRQ_STATUS_ENABLE__ACK__M     |
		 RI_I2C_REG__IRQ_STATUS_ENABLE__NACK__M    |
		 RI_I2C_REG__IRQ_STATUS_ENABLE__STOP__M    |
		 RI_I2C_REG__IRQ_STATUS_ENABLE__ARBLOST__M);
}

static int i2c_send_addr(struct i2c_edgevision_data *data,
			 uint16_t addr, bool read)
{
	WRITEREG(I2C_MST_TX_REG, ((addr & 0x7F) << 1) | (read ? 1U : 0U));
	WRITEREG(I2C_MST_IRQ_STATUS_REG,
		 RI_I2C_REG__IRQ_STATUS__ACK__M   |
		 RI_I2C_REG__IRQ_STATUS__NACK__M  |
		 RI_I2C_REG__IRQ_STATUS__STOP__M  |
		 RI_I2C_REG__IRQ_STATUS__ARBLOST__M);
	data->irq_wait_mask = RI_I2C_REG__IRQ_STATUS__ACK__M |
			      RI_I2C_REG__IRQ_STATUS__NACK__M;
	WRITEREG(I2C_MST_CMD_REG,
		 RI_I2C_REG__CMD__START__M |
		 RI_I2C_REG__CMD__WRITE__M);

	uint32_t ev = wait_for_irq(data, I2C_ADDR_TIMEOUT_MS);

	if (ev == 0) {
		i2c_send_stop(data);
		return -ETIMEDOUT;
	}
	if (ev & RI_I2C_REG__IRQ_STATUS__NACK__M) {
		i2c_send_stop(data);
		if (!data->sccb_mode) {
			return -ENXIO;
		}
	}
	return 0;
}

static int i2c_write_buf(struct i2c_edgevision_data *data,
			 const uint8_t *buf, uint32_t len, bool send_stop)
{
	for (uint32_t i = 0; i < len; i++) {
		bool last           = (i == len - 1);
		bool stop_this_byte = last && send_stop;

		WRITEREG(I2C_MST_TX_REG, buf[i]);
		WRITEREG(I2C_MST_IRQ_STATUS_REG,
			 RI_I2C_REG__IRQ_STATUS__ACK__M  |
			 RI_I2C_REG__IRQ_STATUS__NACK__M |
			 RI_I2C_REG__IRQ_STATUS__STOP__M);

		data->irq_wait_mask = RI_I2C_REG__IRQ_STATUS__ACK__M  |
				RI_I2C_REG__IRQ_STATUS__NACK__M |
				RI_I2C_REG__IRQ_STATUS__STOP__M;

		WRITEREG(I2C_MST_CMD_REG,
			 RI_I2C_REG__CMD__WRITE__M |
			 (stop_this_byte ? RI_I2C_REG__CMD__STOP__M : 0U));

		uint32_t ev = wait_for_irq(data, I2C_BUS_TIMEOUT_MS);

		if (ev == 0) {
			LOG_ERR("I2C: %s timeout on write byte %u",
				stop_this_byte ? "STOP" : "ACK", i);
			return -ETIMEDOUT;
		}

		if (ev & RI_I2C_REG__IRQ_STATUS__NACK__M) {
			if (!data->sccb_mode) {
				if (!last) {
					i2c_send_stop(data);
				} else if (stop_this_byte) {
					if (ev & RI_I2C_REG__IRQ_STATUS__STOP__M) {
						/*
						 *NACK+STOP fused: drain stale second
						 * give if two ISRs fired
						 */
						k_sem_take(&data->irq_sem, K_NO_WAIT);
					} else {
						data->irq_wait_mask = RI_I2C_REG__IRQ_STATUS__STOP__M;
						if (wait_for_irq(data, I2C_BUS_TIMEOUT_MS) == 0) {
							LOG_ERR("I2C: bus stuck after NACK on last byte");
						}
					}
				}
				return -EIO;
			}
			/* SCCB: NACK here is ignored */
			if (stop_this_byte &&
			    !(ev & RI_I2C_REG__IRQ_STATUS__STOP__M)) {
				data->irq_wait_mask = RI_I2C_REG__IRQ_STATUS__STOP__M;
				if (wait_for_irq(data, I2C_BUS_TIMEOUT_MS) == 0) {
					LOG_ERR("I2C: STOP timeout after SCCB write byte %u", i);
					return -ETIMEDOUT;
				}
			}
			continue;
		}

		if (stop_this_byte) {
			if (ev & RI_I2C_REG__IRQ_STATUS__STOP__M) {
				/* ACK+STOP fused drain any stale second give with */
				k_sem_take(&data->irq_sem, K_NO_WAIT);
			} else {
				/* Wait for the STOP ISR */
				data->irq_wait_mask = RI_I2C_REG__IRQ_STATUS__STOP__M;
				if (wait_for_irq(data, I2C_BUS_TIMEOUT_MS) == 0) {
					LOG_ERR("I2C: STOP timeout after last write byte");
					return -ETIMEDOUT;
				}
			}
		}
	}
	return 0;
}

static int i2c_read_buf(struct i2c_edgevision_data *data,
			uint8_t *buf, uint32_t len, bool send_stop)
{
	for (uint32_t i = 0; i < len; i++) {
		bool last          = (i == len - 1);
		bool stop_this_byte = last && send_stop;

		WRITEREG(I2C_MST_IRQ_STATUS_REG,
			 RI_I2C_REG__IRQ_STATUS__ACK__M   |
			 RI_I2C_REG__IRQ_STATUS__NACK__M  |
			 RI_I2C_REG__IRQ_STATUS__STOP__M  |
			 RI_I2C_REG__IRQ_STATUS__ARBLOST__M);

		if (stop_this_byte) {
			data->irq_wait_mask = RI_I2C_REG__IRQ_STATUS__NACK__M |
					      RI_I2C_REG__IRQ_STATUS__STOP__M;
		} else if (last) {
			data->irq_wait_mask = RI_I2C_REG__IRQ_STATUS__ACK__M |
					      RI_I2C_REG__IRQ_STATUS__NACK__M;
		} else {
			/* SCCB also accept NACK on middle bytes */
			data->irq_wait_mask = RI_I2C_REG__IRQ_STATUS__ACK__M |
					   (data->sccb_mode ? RI_I2C_REG__IRQ_STATUS__NACK__M : 0U);
		}
		WRITEREG(I2C_MST_CMD_REG,
			 RI_I2C_REG__CMD__READ__M |
			 ((last || data->sccb_mode) ? RI_I2C_REG__CMD__NACK__M : 0U) |
			 (stop_this_byte ? RI_I2C_REG__CMD__STOP__M : 0U));

		uint32_t ev = wait_for_irq(data, I2C_BUS_TIMEOUT_MS);

		if (ev == 0) {
			LOG_ERR("I2C: %s timeout on read byte %u",
				last ? "STOP" : "ACK", i);
			return -ETIMEDOUT;
		}
		if (!last && (ev & RI_I2C_REG__IRQ_STATUS__NACK__M) && !data->sccb_mode) {
			i2c_send_stop(data);
			return -EIO;
		}
		if (stop_this_byte) {
			if (ev & RI_I2C_REG__IRQ_STATUS__STOP__M) {
				/* NACK+STOP fused: drain stale second give if two ISRs fired */
				k_sem_take(&data->irq_sem, K_NO_WAIT);
			} else {
				/* NACK arrived first, wait for STOP separately */
				data->irq_wait_mask = RI_I2C_REG__IRQ_STATUS__STOP__M;
				if (wait_for_irq(data, I2C_BUS_TIMEOUT_MS) == 0) {
					LOG_ERR("I2C: STOP timeout after last read byte");
					return -ETIMEDOUT;
				}
			}
		}
		buf[i] = (uint8_t)READREG(I2C_MST_RX_REG);
	}
	return 0;
}

static int i2c_edgevision_configure(const struct device *dev, uint32_t dev_config)
{
	struct i2c_edgevision_data *data = dev->data;
	uint32_t bus_hz;

	switch (I2C_SPEED_GET(dev_config)) {
	case I2C_SPEED_STANDARD:
		/* 100kHz */
		bus_hz = RI_I2C_SPEED_STANDARD_HZ;
		break;
	case I2C_SPEED_FAST:
		/* 400kHz */
		bus_hz = RI_I2C_SPEED_FAST_HZ;
		break;
	case I2C_SPEED_FAST_PLUS:
		/* 1MHz */
		bus_hz = RI_I2C_SPEED_FAST_PLUS_HZ;
		break;
	default:
		LOG_ERR("unsupported speed");
		return -ENOTSUP;
	}

	k_sem_take(&data->bus_sem, K_FOREVER);
	i2c_hw_configure(bus_hz);
	k_sem_give(&data->bus_sem);
	return 0;
}

static int i2c_edgevision_get_config(const struct device *dev, uint32_t *dev_config)
{
	const struct i2c_edgevision_config *cfg = dev->config;
	uint32_t speed;

	switch (cfg->clock_frequency) {
	case RI_I2C_SPEED_STANDARD_HZ:
		speed = I2C_SPEED_STANDARD;
		break;
	case RI_I2C_SPEED_FAST_HZ:
		speed = I2C_SPEED_FAST;
		break;
	case RI_I2C_SPEED_FAST_PLUS_HZ:
		speed = I2C_SPEED_FAST_PLUS;
		break;
	default:
		return -ENOTSUP;
	}

	*dev_config = I2C_MODE_CONTROLLER | I2C_SPEED_SET(speed);
	return 0;
}


static int i2c_edgevision_transfer(const struct device *dev,
				   struct i2c_msg *msgs,
				   uint8_t num_msgs,
				   uint16_t addr)
{
	struct i2c_edgevision_data *data = dev->data;
	int ret = 0;

	/* Acquire semaphore, block bus for other threads */
	k_sem_take(&data->bus_sem, K_FOREVER);

	for (uint8_t i = 0; i < num_msgs; i++) {
		bool read      = msgs[i].flags & I2C_MSG_READ;
		bool send_stop = msgs[i].flags & I2C_MSG_STOP;

		ret = i2c_send_addr(data, addr, read);
		if (ret) {
			goto unlock;
		}

		if (msgs[i].len == 0) {
			i2c_send_stop(data);
		} else if (read) {
			ret = i2c_read_buf(data, msgs[i].buf, msgs[i].len, send_stop);
		} else {
			ret = i2c_write_buf(data, msgs[i].buf, msgs[i].len, send_stop);
		}
	}

unlock:
	k_sem_give(&data->bus_sem);
	return ret;
}

static const struct i2c_driver_api i2c_edgevision_driver_api = {
	.configure  = i2c_edgevision_configure,
	.transfer   = i2c_edgevision_transfer,
	.get_config = i2c_edgevision_get_config,
};

static void i2c_edgevision_isr(const struct device *dev)
{
	struct i2c_edgevision_data *data = dev->data;

	uint32_t status = READREG(I2C_MST_IRQ_STATUS_REG);

	WRITEREG(I2C_MST_IRQ_STATUS_REG, status);

	/* Sources the current phase never waits on rate-limited to 1/s.
	 */
	uint32_t unexpected = status & ~data->irq_wait_mask;

	if (unexpected != 0U) {
		if (k_uptime_get() - data->last_fault_log_ms > 0) {
			data->last_fault_log_ms = k_uptime_get();
			LOG_ERR("I2C: unexpected IRQ status 0x%02x (bus fault?)",
				(unsigned int)unexpected);
		}
	}

	status &= data->irq_wait_mask;
	if (status == 0) {
		return;
	}
	/* Accumulate: E.g. a STOP may arrive before the waiter consumes ACK */
	data->irq_status |= status;
	k_sem_give(&data->irq_sem);
}

static int i2c_edgevision_init(const struct device *dev)
{
	const struct i2c_edgevision_config *cfg = dev->config;
	struct i2c_edgevision_data *data = dev->data;
	int ret;

    LOG_INF("Initializing I2C ");

	ret = k_sem_init(&data->bus_sem, 1, 1);
	if (ret) {
		return ret;
	}
	ret = k_sem_init(&data->irq_sem, 0, 1);
	if (ret) {
		return ret;
	}

	i2c_hw_configure(cfg->clock_frequency);
	cfg->irq_config_func(dev);

    LOG_INF("Initialized @%dHz...", cfg->clock_frequency);
	return 0;
}

#define I2C_EDGEVISION_INIT(n)							\
										\
static void i2c_edgevision_irq_config_##n(const struct device *dev)	\
{										\
	IRQ_CONNECT(DT_INST_IRQN(n),						\
		    DT_INST_IRQ(n, priority),					\
		    i2c_edgevision_isr,						\
		    DEVICE_DT_INST_GET(n),					\
		    0);								\
	irq_enable(DT_INST_IRQN(n));						\
}										\
										\
static const struct i2c_edgevision_config i2c_edgevision_config_##n = {	\
	.clock_frequency = DT_INST_PROP_OR(n, clock_frequency, 100000),		\
	.irq_config_func = i2c_edgevision_irq_config_##n,			\
};										\
										\
static struct i2c_edgevision_data i2c_edgevision_data_##n = {		\
	.sccb_mode = DT_INST_PROP(n, sccb_mode),			\
};									\
										\
DEVICE_DT_INST_DEFINE(n,						\
		      i2c_edgevision_init,					\
		      NULL,							\
		      &i2c_edgevision_data_##n,					\
		      &i2c_edgevision_config_##n,				\
		      POST_KERNEL,						\
		      CONFIG_I2C_INIT_PRIORITY,					\
		      &i2c_edgevision_driver_api);

DT_INST_FOREACH_STATUS_OKAY(I2C_EDGEVISION_INIT)
