/*
 * Copyright (c) 2026 Racyics GmbH.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT racyics_edgevision_i2s

#include <zephyr/kernel.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/logging/log.h>
#include <zephyr/cache.h>
#include <soc.h>

LOG_MODULE_REGISTER(i2s_racyics_edgevision, CONFIG_I2S_LOG_LEVEL);

/* Register accessors — base addresses resolved at init from DT. */
#define I2S_TARGET_CFG_REG(cfg)        ((cfg)->base + RI_I2S_SLAVE_REG__CFG__OFFSET)
#define I2S_TARGET_CTRL_REG(cfg)       ((cfg)->base + RI_I2S_SLAVE_REG__CTRL__OFFSET)
#define I2S_TARGET_STATUS_REG(cfg)     ((cfg)->base + RI_I2S_SLAVE_REG__STATUS__OFFSET)
#define I2S_TARGET_IRQ_STATUS_REG(cfg) ((cfg)->base + RI_I2S_SLAVE_REG__IRQ_STATUS__OFFSET)

#define I2S_CONTROLLER_CFG_REG(cfg)        ((cfg)->base + RI_I2S_MASTER_REG__CFG__OFFSET)
#define I2S_CONTROLLER_CTRL_REG(cfg)       ((cfg)->base + RI_I2S_MASTER_REG__CTRL__OFFSET)
#define I2S_CONTROLLER_STATUS_REG(cfg)     ((cfg)->base + RI_I2S_MASTER_REG__STATUS__OFFSET)
#define I2S_CONTROLLER_IRQ_STATUS_REG(cfg) ((cfg)->base + RI_I2S_MASTER_REG__IRQ_STATUS__OFFSET)

#define WDMA_CTRL_REG(cfg)         ((cfg)->dma_base + RI_WDMA_BASE_REG__CTRL__OFFSET)
#define WDMA_STARTADDR_REG(cfg)    ((cfg)->dma_base + RI_WDMA_BASE_REG__STARTADDR__OFFSET)
#define WDMA_STARTADDR_DB_REG(cfg) ((cfg)->dma_base + RI_WDMA_BASE_REG__STARTADDR_DB__OFFSET)
#define WDMA_MAXSIZE_REG(cfg)      ((cfg)->dma_base + RI_WDMA_BASE_REG__MAXSIZE__OFFSET)
#define WDMA_SIZE1_REG(cfg)        ((cfg)->dma_base + RI_WDMA_BASE_REG__SIZE1__OFFSET)
#define WDMA_CURR_ADDR_REG(cfg)    ((cfg)->dma_base + RI_WDMA_BASE_REG__CURR_ADDR__OFFSET)
#define WDMA_IRQ_STATUS_REG(cfg)   ((cfg)->dma_base + RI_WDMA_BASE_REG__IRQ_STATUS__OFFSET)
#define WDMA_IRQ_STATUS_ENABLE_REG(cfg)                                                            \
	((cfg)->dma_base + RI_WDMA_BASE_REG__IRQ_STATUS_ENABLE__OFFSET)

#define WDMA_BASE_OFFSET 0x40U

#define RDMA_CTRL_REG(cfg)         ((cfg)->dma_base + RI_RDMA_BASE_REG__CTRL__OFFSET)
#define RDMA_STARTADDR_REG(cfg)    ((cfg)->dma_base + RI_RDMA_BASE_REG__STARTADDR__OFFSET)
#define RDMA_STARTADDR_DB_REG(cfg) ((cfg)->dma_base + RI_RDMA_BASE_REG__STARTADDR_DB__OFFSET)
#define RDMA_MAXSIZE_REG(cfg)      ((cfg)->dma_base + RI_RDMA_BASE_REG__MAXSIZE__OFFSET)
#define RDMA_SIZE1_REG(cfg)        ((cfg)->dma_base + RI_RDMA_BASE_REG__SIZE1__OFFSET)
#define RDMA_CURR_ADDR_REG(cfg)    ((cfg)->dma_base + RI_RDMA_BASE_REG__CURR_ADDR__OFFSET)
#define RDMA_IRQ_STATUS_REG(cfg)   ((cfg)->dma_base + RI_RDMA_BASE_REG__IRQ_STATUS__OFFSET)
#define RDMA_IRQ_STATUS_ENABLE_REG(cfg)                                                            \
	((cfg)->dma_base + RI_RDMA_BASE_REG__IRQ_STATUS_ENABLE__OFFSET)

#define RDMA_BASE_OFFSET 0x40U

/* Minimum slab blocks for RX: 2 permanent DMA buffers + 1 staging block */
#define RX_SLAB_BLOCKS_MIN 3U

/* Depth of the RX message queue */
#define RX_QUEUE_DEPTH 2U

struct rx_queue_item {
	void *mem_block;
	size_t size;
};

enum i2s_edgevision_role {
	I2S_TARGET_e,
	I2S_CONTROLLER_e,
};

/* Function pointers for separate controller/target behaviour*/
struct i2s_edgevision_ops {
	int (*configure)(const struct device *dev, enum i2s_dir dir);
	int (*start)(const struct device *dev);
	void (*disable)(const struct device *dev);
	void (*handle_irq)(const struct device *dev);
};

struct i2s_edgevision_config {
	mem_addr_t base;
	mem_addr_t dma_base;
	enum i2s_edgevision_role role;
	uint32_t irq_num;
	void (*irq_config_func)(const struct device *dev);
	const struct i2s_edgevision_ops *ops;
};

struct i2s_edgevision_data {
	size_t buffer_size;
	void *dma_buf[2];
	uint8_t active_buf;

	struct k_mem_slab *mem_slab;
	struct i2s_config cfg;
	enum i2s_state state;
	struct k_spinlock lock;

	struct k_msgq rx_queue;
	struct rx_queue_item rx_queue_buf[RX_QUEUE_DEPTH];
};

#define I2S_EDGEVISION_ROLE(n) (DT_INST_ENUM_IDX(n, role) == 0 ? I2S_CONTROLLER_e : I2S_TARGET_e)
#define I2S_EDGEVISION_OPS(n)                                                                      \
	(DT_INST_ENUM_IDX(n, role) == 0 ? &i2s_controller_ops : &i2s_target_ops)

/*
 * --------------------------------------------------------------------------
 * Target (RX) implementation
 * --------------------------------------------------------------------------
 */

static int ri_i2s_target_configure(const struct device *dev, enum i2s_dir dir)
{
	const struct i2s_edgevision_config *cfg = dev->config;
	struct i2s_edgevision_data *data = dev->data;
	int ret;

	if (dir != I2S_DIR_RX) {
		LOG_ERR("I2S target only supports RX");
		return -ENOTSUP;
	}

	if (data->dma_buf[0] != NULL) {
		k_mem_slab_free(data->mem_slab, data->dma_buf[0]);
		data->dma_buf[0] = NULL;
	}
	if (data->dma_buf[1] != NULL) {
		k_mem_slab_free(data->mem_slab, data->dma_buf[1]);
		data->dma_buf[1] = NULL;
	}

	if (k_mem_slab_num_free_get(data->mem_slab) < RX_SLAB_BLOCKS_MIN) {
		LOG_ERR("mem_slab needs at least %u free blocks, has %u", RX_SLAB_BLOCKS_MIN,
			k_mem_slab_num_free_get(data->mem_slab));
		return -ENOMEM;
	}

	ret = k_mem_slab_alloc(data->mem_slab, &data->dma_buf[0], K_NO_WAIT);
	if (ret == 0) {
		sys_cache_data_flush_and_invd_range(data->dma_buf[0], data->buffer_size);
	}
	if (ret != 0) {
		LOG_ERR("failed to allocate DMA buffer 0");
		return -ENOMEM;
	}

	ret = k_mem_slab_alloc(data->mem_slab, &data->dma_buf[1], K_NO_WAIT);
	if (ret == 0) {
		sys_cache_data_flush_and_invd_range(data->dma_buf[1], data->buffer_size);
	}
	if (ret != 0) {
		k_mem_slab_free(data->mem_slab, data->dma_buf[0]);
		data->dma_buf[0] = NULL;
		LOG_ERR("failed to allocate DMA buffer 1");
		return -ENOMEM;
	}

	data->active_buf = 0U;

	WRITEREG(I2S_TARGET_CFG_REG(cfg), RI_I2S_SLAVE_REG__CFG__WS_POLARITY__M);

	WRITEREG(WDMA_MAXSIZE_REG(cfg), data->buffer_size);
	WRITEREG(WDMA_STARTADDR_REG(cfg), (uintptr_t)data->dma_buf[0]);
	WRITEREG(WDMA_STARTADDR_DB_REG(cfg), (uintptr_t)data->dma_buf[1]);
	WRITEREG(WDMA_IRQ_STATUS_ENABLE_REG(cfg), RI_WDMA_BASE_REG__IRQ_STATUS__MAXSIZE__M);

	data->state = I2S_STATE_READY;
	LOG_INF("I2S target configured: DMA buf[0]=0x%08X buf[1]=0x%08X",
		(uint32_t)(uintptr_t)data->dma_buf[0], (uint32_t)(uintptr_t)data->dma_buf[1]);
	return 0;
}

static int ri_i2s_target_start(const struct device *dev)
{
	const struct i2s_edgevision_config *cfg = dev->config;
	struct i2s_edgevision_data *data = dev->data;

	k_spinlock_key_t key = k_spin_lock(&data->lock);

	data->state = I2S_STATE_RUNNING;
	data->active_buf = 0U;

	/* Sync driver and HW buffers */
	WRITEREG(WDMA_STARTADDR_REG(cfg), (uintptr_t)data->dma_buf[0]);
	WRITEREG(WDMA_STARTADDR_DB_REG(cfg), (uintptr_t)data->dma_buf[1]);

	WRITEREG(I2S_TARGET_CTRL_REG(cfg),
		 RI_I2S_SLAVE_REG__CTRL__ENABLE__M | RI_I2S_SLAVE_REG__CTRL__RESET_DISABLE__M);
	WRITEREG(WDMA_CTRL_REG(cfg),
		 RI_WDMA_BASE_REG__CTRL__ENABLE__M | RI_WDMA_BASE_REG__CTRL__DOUBLE_BUF__M);

	k_spin_unlock(&data->lock, key);
	LOG_INF("I2S RX started");
	return 0;
}

static void ri_i2s_target_disable(const struct device *dev)
{
	const struct i2s_edgevision_config *cfg = dev->config;
	struct i2s_edgevision_data *data = dev->data;

	k_spinlock_key_t key = k_spin_lock(&data->lock);

	ANDREG(I2S_TARGET_CTRL_REG(cfg), ~RI_I2S_SLAVE_REG__CTRL__ENABLE__M);
	ANDREG(WDMA_CTRL_REG(cfg), ~RI_WDMA_BASE_REG__CTRL__ENABLE__M);

	k_spin_unlock(&data->lock, key);

	/* Drain blocks still in the queue. */
	struct rx_queue_item item;

	while (k_msgq_get(&data->rx_queue, &item, K_NO_WAIT) == 0) {
		k_mem_slab_free(data->mem_slab, item.mem_block);
	}

	LOG_DBG("mem_slab has %u free blocks after %s", k_mem_slab_num_free_get(data->mem_slab),
		__func__);

	LOG_INF("I2S RX stopped");
}

static void ri_i2s_target_isr(const struct device *dev)
{
	const struct i2s_edgevision_config *cfg = dev->config;
	struct i2s_edgevision_data *data = dev->data;
	uint32_t irq_status;

	irq_status = READREG(WDMA_IRQ_STATUS_REG(cfg));

	if (!(irq_status & RI_WDMA_BASE_REG__IRQ_STATUS__MAXSIZE__M)) {
		return;
	}

	/* Clear the interrupt before doing any work. */
	WRITEREG(WDMA_IRQ_STATUS_REG(cfg), RI_WDMA_BASE_REG__IRQ_STATUS__MAXSIZE__M);

	K_SPINLOCK(&data->lock) {

		/* Stop transmission after filling final block */
		if (data->state == I2S_STATE_STOPPING) {
			/* Stop target HW */
			ANDREG(I2S_TARGET_CTRL_REG(cfg), ~RI_I2S_SLAVE_REG__CTRL__ENABLE__M);
			ANDREG(WDMA_CTRL_REG(cfg), ~RI_WDMA_BASE_REG__CTRL__ENABLE__M);

			void *new_buf;

			if (k_mem_slab_alloc(data->mem_slab, &new_buf, K_NO_WAIT)) {
				LOG_ERR("RX slab exhausted on stop — final frame dropped");
			} else {
				sys_cache_data_flush_and_invd_range(new_buf, data->buffer_size);
				void *final_completed = data->dma_buf[data->active_buf];
				struct rx_queue_item final_item = {
					.mem_block = final_completed,
					.size = data->buffer_size,
				};
				sys_cache_data_flush_and_invd_range(final_completed,
								    data->buffer_size);
				if (k_msgq_put(&data->rx_queue, &final_item, K_NO_WAIT)) {
					LOG_ERR("RX queue full on stop — final frame dropped");
					k_mem_slab_free(data->mem_slab, final_completed);
				}

				data->dma_buf[data->active_buf] = new_buf;
			}

			data->state = I2S_STATE_READY;
			data->active_buf ^= 1U;
			K_SPINLOCK_BREAK;
		}

		/* Check for room in the completed buffer queue */
		if (k_msgq_num_free_get(&data->rx_queue) == 0) {
			LOG_ERR("RX queue full — frame dropped (Overrun)");
			data->state = I2S_STATE_ERROR;
			ANDREG(I2S_TARGET_CTRL_REG(cfg), ~RI_I2S_SLAVE_REG__CTRL__ENABLE__M);
			ANDREG(WDMA_CTRL_REG(cfg), ~RI_WDMA_BASE_REG__CTRL__ENABLE__M);
			data->active_buf ^= 1U;
			K_SPINLOCK_BREAK;
		}

		/* Allocate replacement buffer*/
		void *new_buf;

		if (k_mem_slab_alloc(data->mem_slab, &new_buf, K_NO_WAIT) == 0) {
			sys_cache_data_flush_and_invd_range(new_buf, data->buffer_size);
		} else {
			LOG_ERR("RX slab exhausted — frame dropped (Overrun)");
			data->state = I2S_STATE_ERROR;
			ANDREG(I2S_TARGET_CTRL_REG(cfg), ~RI_I2S_SLAVE_REG__CTRL__ENABLE__M);
			ANDREG(WDMA_CTRL_REG(cfg), ~RI_WDMA_BASE_REG__CTRL__ENABLE__M);
			data->active_buf ^= 1U;
			K_SPINLOCK_BREAK;
		}

		void *completed_buf = data->dma_buf[data->active_buf];

		data->dma_buf[data->active_buf] = new_buf;

		/* Update HW buffer address register*/
		if (data->active_buf == 0U) {
			WRITEREG(WDMA_STARTADDR_REG(cfg), (uintptr_t)new_buf);
		} else {
			WRITEREG(WDMA_STARTADDR_DB_REG(cfg), (uintptr_t)new_buf);
		}
		data->active_buf ^= 1U;

		/* Put filled buffer in message queue */
		struct rx_queue_item item = {
			.mem_block = completed_buf,
			.size = data->buffer_size,
		};

		sys_cache_data_invd_range(completed_buf, data->buffer_size);
		if (k_msgq_put(&data->rx_queue, &item, K_NO_WAIT) != 0) {
			LOG_ERR("RX queue full — race condition frame dropped");
			k_mem_slab_free(data->mem_slab, completed_buf);
			data->state = I2S_STATE_ERROR;
			ANDREG(I2S_TARGET_CTRL_REG(cfg), ~RI_I2S_SLAVE_REG__CTRL__ENABLE__M);
			ANDREG(WDMA_CTRL_REG(cfg), ~RI_WDMA_BASE_REG__CTRL__ENABLE__M);
		}
	}
}

/*
 * --------------------------------------------------------------------------
 * Controller (TX) stubs — TODO
 * --------------------------------------------------------------------------
 */

static int ri_i2s_controller_configure(const struct device *dev, enum i2s_dir dir)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(dir);
	LOG_ERR("I2S controller TX not yet implemented");
	return -ENOTSUP;
}

static int ri_i2s_controller_start(const struct device *dev)
{
	const struct i2s_edgevision_config *cfg = dev->config;
	struct i2s_edgevision_data *data = dev->data;

	WRITEREG(RDMA_CTRL_REG(cfg),
		 RI_RDMA_BASE_REG__CTRL__ENABLE__M | RI_RDMA_BASE_REG__CTRL__DOUBLE_BUF__M);

	data->state = I2S_STATE_RUNNING;
	LOG_INF("I2S TX started");
	return 0;
}

static void ri_i2s_controller_disable(const struct device *dev)
{
	const struct i2s_edgevision_config *cfg = dev->config;

	ANDREG(I2S_TARGET_CTRL_REG(cfg), ~RI_I2S_SLAVE_REG__CTRL__ENABLE__M);
	ANDREG(WDMA_CTRL_REG(cfg), ~RI_WDMA_BASE_REG__CTRL__ENABLE__M);
	LOG_INF("I2S TX stopped");
}

static void ri_i2s_controller_isr(const struct device *dev)
{
	ARG_UNUSED(dev);
	LOG_ERR("I2S controller ISR not yet implemented");
}

/*
 * --------------------------------------------------------------------------
 * Op tables
 * --------------------------------------------------------------------------
 */

static const struct i2s_edgevision_ops i2s_target_ops = {
	.configure = ri_i2s_target_configure,
	.start = ri_i2s_target_start,
	.disable = ri_i2s_target_disable,
	.handle_irq = ri_i2s_target_isr,
};

static const struct i2s_edgevision_ops i2s_controller_ops = {
	.configure = ri_i2s_controller_configure,
	.start = ri_i2s_controller_start,
	.disable = ri_i2s_controller_disable,
	.handle_irq = ri_i2s_controller_isr,
};

/*
 * --------------------------------------------------------------------------
 * Zephyr i2s_driver_api implementation
 * --------------------------------------------------------------------------
 */

static int i2s_edgevision_configure(const struct device *dev, enum i2s_dir dir,
				    const struct i2s_config *i2s_cfg)
{
	const struct i2s_edgevision_config *cfg = dev->config;
	struct i2s_edgevision_data *data = dev->data;
	int ret;

	if (dir == I2S_DIR_BOTH) {
		LOG_ERR("I2S_DIR_BOTH not supported");
		return -ENOTSUP;
	}

	if (!i2s_cfg->mem_slab) {
		LOG_ERR("memory slab required");
		return -EINVAL;
	}

	data->mem_slab = i2s_cfg->mem_slab;
	data->buffer_size = i2s_cfg->block_size;
	memcpy(&data->cfg, i2s_cfg, sizeof(struct i2s_config));

	ret = cfg->ops->configure(dev, dir);
	if (ret) {
		LOG_ERR("I2S configuration failed: %d", ret);
	}

	return ret;
}

static const struct i2s_config *i2s_edgevision_config_get(const struct device *dev,
							  enum i2s_dir dir)
{
	const struct i2s_edgevision_config *cfg = dev->config;
	struct i2s_edgevision_data *data = dev->data;

	if (data->state == I2S_STATE_NOT_READY) {
		LOG_ERR("config_get: invalid state %d", data->state);
		return NULL;
	}

	if ((cfg->role == I2S_TARGET_e && dir != I2S_DIR_RX) ||
	    (cfg->role == I2S_CONTROLLER_e && dir != I2S_DIR_TX)) {
		LOG_ERR("Unsupported direction for this I2S role");
		return NULL;
	}

	return &data->cfg;
}

static int i2s_edgevision_trigger(const struct device *dev, enum i2s_dir dir,
				  enum i2s_trigger_cmd cmd)
{
	const struct i2s_edgevision_config *cfg = dev->config;
	struct i2s_edgevision_data *data = dev->data;

	if (dir != I2S_DIR_RX) {
		return -ENOTSUP;
	}

	switch (cmd) {
	case I2S_TRIGGER_START:
		if (data->state != I2S_STATE_READY) {
			LOG_ERR("START: invalid state %d", data->state);
			return -EIO;
		}
		return cfg->ops->start(dev);

	case I2S_TRIGGER_STOP: {
		k_spinlock_key_t key = k_spin_lock(&data->lock);

		if (data->state != I2S_STATE_RUNNING) {
			k_spin_unlock(&data->lock, key);
			LOG_ERR("STOP: invalid state %d", data->state);
			return -EIO;
		}
		/* Let the ISR halt hardware at the next block boundary */
		data->state = I2S_STATE_STOPPING;
		k_spin_unlock(&data->lock, key);
		return 0;
	}

	case I2S_TRIGGER_DRAIN: {
		k_spinlock_key_t key = k_spin_lock(&data->lock);

		if (data->state != I2S_STATE_RUNNING) {
			k_spin_unlock(&data->lock, key);
			LOG_ERR("DRAIN: invalid state %d", data->state);
			return -EIO;
		}
		data->state = I2S_STATE_STOPPING;
		k_spin_unlock(&data->lock, key);
		return 0;
	}

	case I2S_TRIGGER_DROP:
		if (data->state == I2S_STATE_NOT_READY) {
			return -EIO;
		}
		/* Stop immediately */
		cfg->ops->disable(dev);
		data->state = I2S_STATE_READY;
		return 0;

	case I2S_TRIGGER_PREPARE:
		if (data->state != I2S_STATE_ERROR) {
			LOG_ERR("PREPARE: invalid state %d", data->state);
			return -EIO;
		}
		cfg->ops->disable(dev);
		data->state = I2S_STATE_READY;
		LOG_DBG("State I2S_TRIGGER_PREPARE: %d", data->state);
		return 0;

	default:
		LOG_ERR("unsupported trigger command %d", cmd);
		return -EINVAL;
	}
}

/* Pass block form msg queue to application */
static int i2s_edgevision_read(const struct device *dev, void **mem_block, size_t *size)
{
	struct i2s_edgevision_data *data = dev->data;
	struct rx_queue_item item;
	int ret;

	if (data->state == I2S_STATE_NOT_READY) {
		LOG_ERR("I2S RX not configured");
		return -EIO;
	}

	ret = k_msgq_get(&data->rx_queue, &item,
			 (data->state == I2S_STATE_ERROR) ? K_NO_WAIT : K_MSEC(data->cfg.timeout));

	/* Only fail after message queue is exhausted */
	if (ret) {
		return (data->state == I2S_STATE_ERROR) ? -EIO : -EAGAIN;
	}

	*mem_block = item.mem_block;
	*size = item.size;
	return 0;
}

static int i2s_edgevision_write(const struct device *dev, void *mem_block, size_t size)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(mem_block);
	ARG_UNUSED(size);

	return -ENOTSUP;
}

static void i2s_edgevision_isr(const struct device *dev)
{
	const struct i2s_edgevision_config *cfg = dev->config;

	cfg->ops->handle_irq(dev);
}

static DEVICE_API(i2s, i2s_edgevision_driver_api) = {
	.configure = i2s_edgevision_configure,
	.config_get = i2s_edgevision_config_get,
	.read = i2s_edgevision_read,
	.write = i2s_edgevision_write,
	.trigger = i2s_edgevision_trigger,
};

static int i2s_edgevision_init(const struct device *dev)
{
	const struct i2s_edgevision_config *cfg = dev->config;
	struct i2s_edgevision_data *data = dev->data;

	data->state = I2S_STATE_NOT_READY;
	data->active_buf = 0U;
	data->dma_buf[0] = NULL;
	data->dma_buf[1] = NULL;

	k_msgq_init(&data->rx_queue, (char *)data->rx_queue_buf, sizeof(struct rx_queue_item),
		    RX_QUEUE_DEPTH);

	if (cfg->role == I2S_CONTROLLER_e) {
		WRITEREG(I2S_CONTROLLER_CTRL_REG(cfg), RI_I2S_MASTER_REG__CTRL__RSTVAL);
		WRITEREG(RDMA_CTRL_REG(cfg), RI_RDMA_BASE_REG__CTRL__RSTVAL);
	} else {
		WRITEREG(I2S_TARGET_CTRL_REG(cfg), RI_I2S_SLAVE_REG__CTRL__RSTVAL);
		WRITEREG(WDMA_CTRL_REG(cfg), RI_WDMA_BASE_REG__CTRL__RSTVAL);
	}

	cfg->irq_config_func(dev);

	LOG_INF("I2S initialized at base 0x%08X", (uint32_t)cfg->base);

	return 0;
}

#define I2S_EDGEVISION_INIT(n)                                                                     \
                                                                                                   \
	static void i2s_edgevision_irq_config_##n(const struct device *dev)                        \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), i2s_edgevision_isr,         \
			    DEVICE_DT_INST_GET(n), 0);                                             \
		irq_enable(DT_INST_IRQN(n));                                                       \
	}                                                                                          \
                                                                                                   \
	static const struct i2s_edgevision_config i2s_edgevision_config_##n = {                    \
		.base = DT_INST_REG_ADDR(n),                                                       \
		.dma_base = DT_INST_REG_ADDR(n) + WDMA_BASE_OFFSET,                                \
		.irq_num = DT_INST_IRQN(n),                                                        \
		.irq_config_func = i2s_edgevision_irq_config_##n,                                  \
		.role = I2S_EDGEVISION_ROLE(n),                                                    \
		.ops = I2S_EDGEVISION_OPS(n),                                                      \
	};                                                                                         \
                                                                                                   \
	static struct i2s_edgevision_data i2s_edgevision_data_##n;                                 \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, i2s_edgevision_init, NULL, &i2s_edgevision_data_##n,              \
			      &i2s_edgevision_config_##n, POST_KERNEL, CONFIG_I2S_INIT_PRIORITY,   \
			      &i2s_edgevision_driver_api);

DT_INST_FOREACH_STATUS_OKAY(I2S_EDGEVISION_INIT)
