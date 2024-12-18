/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2024 sensry.io
 */

#define DT_DRV_COMPAT sensry_sy1xx_mdio

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(sy1xx_mdio, CONFIG_MDIO_LOG_LEVEL);

#include <zephyr/drivers/mdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/mdio.h>
#include <udma.h>

struct sy1xx_mdio_dev_config {
	const struct pinctrl_dev_config *pcfg;
	uint32_t base_addr;
	uint32_t mdc_freq;
	uint32_t timeout_msec;
};

struct sy1xx_mdio_dev_data {
	struct k_sem sem;
};

/* mdio register offsets */
#define SY1XX_MDIO_CFG_REG        0x0020
#define SY1XX_MDIO_CTRL_REG       0x0024
#define SY1XX_MDIO_READ_DATA_REG  0x0028
#define SY1XX_MDIO_WRITE_DATA_REG 0x002c
#define SY1XX_MDIO_IRQ_REG        0x0030

/* mdio config register bit offsets */
#define SY1XX_MDIO_CFG_DIV_OFFS (0)
#define SY1XX_MDIO_CFG_EN_OFFS  (8)

/* mdio ctrl register bit offsets */
#define SY1XX_MDIO_CTRL_READY_OFFS    (0)
#define SY1XX_MDIO_CTRL_INIT_OFFS     (8)
#define SY1XX_MDIO_CTRL_REG_ADDR_OFFS (16)
#define SY1XX_MDIO_CTRL_PHY_ADDR_OFFS (24)
#define SY1XX_MDIO_CTRL_OP_OFFS       (30)

/* mdio ctrl operations */
#define SY1XX_MDIO_CTRL_OP_WRITE (0x1)
#define SY1XX_MDIO_CTRL_OP_READ  (0x2)

static int sy1xx_mdio_initialize(const struct device *dev)
{

	LOG_INF("initializing");

	struct sy1xx_mdio_dev_config *cfg = (struct sy1xx_mdio_dev_config *)dev->config;

	/* zero mdio controller regs */
	sys_write32(0x0, cfg->base_addr + SY1XX_MDIO_CFG_REG);
	sys_write32(0x0, cfg->base_addr + SY1XX_MDIO_CTRL_REG);
	sys_write32(0x0, cfg->base_addr + SY1XX_MDIO_READ_DATA_REG);
	sys_write32(0x0, cfg->base_addr + SY1XX_MDIO_WRITE_DATA_REG);
	sys_write32(0x0, cfg->base_addr + SY1XX_MDIO_IRQ_REG);

	/* prepare mdio clock and enable mdio controller */
	uint32_t div = (((sy1xx_soc_get_peripheral_clock() / cfg->mdc_freq) / 2) - 1) & 0xff;

	uint32_t reg = (div << SY1XX_MDIO_CFG_DIV_OFFS) | (1 << SY1XX_MDIO_CFG_EN_OFFS);

	sys_write32(reg, cfg->base_addr + SY1XX_UDMA_CFG_REG);

	/* PAD config */
	if (0 != pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT)) {
		LOG_ERR("failed to configure pins");
	}

	return 0;
}

static uint32_t sy1xx_mdio_is_ready(const struct device *dev)
{
	struct sy1xx_mdio_dev_config *cfg = (struct sy1xx_mdio_dev_config *)dev->config;
	uint32_t status = sys_read32(cfg->base_addr + SY1XX_MDIO_CTRL_REG);

	return (status & (1 << SY1XX_MDIO_CTRL_READY_OFFS));
}

static int32_t sy1xx_mdio_wait_for_ready(const struct device *dev)
{
	struct sy1xx_mdio_dev_config *cfg = (struct sy1xx_mdio_dev_config *)dev->config;
	uint32_t timeout = cfg->timeout_msec;

	while (0 == sy1xx_mdio_is_ready(dev)) {
		k_sleep(K_MSEC(1));
		timeout--;
		if (timeout == 0) {
			return -1;
		}
	}
	return 0;
}

static int sy1xx_mdio_read(const struct device *dev, uint8_t prtad, uint8_t regad, uint16_t *data)
{
	struct sy1xx_mdio_dev_config *cfg = (struct sy1xx_mdio_dev_config *)dev->config;

	prtad &= 0x1f;
	regad &= 0x1f;

	uint32_t v = (SY1XX_MDIO_CTRL_OP_READ << SY1XX_MDIO_CTRL_OP_OFFS) |
		     (prtad << SY1XX_MDIO_CTRL_PHY_ADDR_OFFS) |
		     (regad << SY1XX_MDIO_CTRL_REG_ADDR_OFFS) | (1 << SY1XX_MDIO_CTRL_INIT_OFFS);

	/* start the reading procedure */
	sys_write32(cfg->base_addr + SY1XX_MDIO_CTRL_REG, v);

	/* wait for the reading operation to finish */
	if (0 != sy1xx_mdio_wait_for_ready(dev)) {
		LOG_WRN("timeout while reading from phy: %d, reg: %d", prtad, regad);
		return -ETIMEDOUT;
	}

	/* get the data from the read result register */
	*data = sys_read32(cfg->base_addr + SY1XX_MDIO_READ_DATA_REG);

	LOG_INF("read phy: %d, reg: %d, value: %d", prtad, regad, *data);

	return 0;
}

static int sy1xx_mdio_write(const struct device *dev, uint8_t prtad, uint8_t regad, uint16_t data)
{
	struct sy1xx_mdio_dev_config *cfg = (struct sy1xx_mdio_dev_config *)dev->config;

	prtad &= 0x1f;
	regad &= 0x1f;

	/* put the data to the write register */
	sys_write32(cfg->base_addr + SY1XX_MDIO_WRITE_DATA_REG, data);

	uint32_t v = (SY1XX_MDIO_CTRL_OP_WRITE << SY1XX_MDIO_CTRL_OP_OFFS) |
		     (prtad << SY1XX_MDIO_CTRL_PHY_ADDR_OFFS) |
		     (regad << SY1XX_MDIO_CTRL_REG_ADDR_OFFS) | (1 << SY1XX_MDIO_CTRL_INIT_OFFS);

	/* start the writing procedure */
	sys_write32(cfg->base_addr + SY1XX_MDIO_CTRL_REG, v);

	/* wait for the reading operation to finish */
	if (0 != sy1xx_mdio_wait_for_ready(dev)) {
		LOG_WRN("timeout while writing to phy: %d, reg: %d, val: %d", prtad, regad, data);
		return -ETIMEDOUT;
	}

	LOG_INF("write phy: %d, reg: %d, value: %d", prtad, regad, data);

	return 0;
}

static DEVICE_API(mdio, sy1xx_mdio_driver_api) = {
	.read = sy1xx_mdio_read,
	.write = sy1xx_mdio_write,
};

#define SY1XX_MDIO_INIT(n)                                                                         \
                                                                                                   \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
                                                                                                   \
	static const struct sy1xx_mdio_dev_config sy1xx_mdio_dev_config_##n = {                    \
		.base_addr = (uint32_t)DT_INST_REG_ADDR(n),                                        \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                         \
		.mdc_freq = DT_INST_PROP(n, clock_frequency),                                      \
		.timeout_msec = DT_INST_PROP(n, timeout_msec),                                     \
	};                                                                                         \
                                                                                                   \
	static struct sy1xx_mdio_dev_data sy1xx_mdio_dev_data##n;                                  \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, &sy1xx_mdio_initialize, NULL, &sy1xx_mdio_dev_data##n,            \
			      &sy1xx_mdio_dev_config_##n, POST_KERNEL, CONFIG_MDIO_INIT_PRIORITY,  \
			      &sy1xx_mdio_driver_api);

DT_INST_FOREACH_STATUS_OKAY(SY1XX_MDIO_INIT)
