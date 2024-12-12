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

/* mdio register bit offsets */
#define SY1XX_MDIO_CFG_DIV_OFFS (0)
#define SY1XX_MDIO_CFG_EN_OFFS  (8)

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
	if (0 != pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT)){
		LOG_ERR("failed to configure pins");
	}

	return 0;
}

static int sy1xx_mdio_read(const struct device *dev, uint8_t prtad, uint8_t regad, uint16_t *data)
{
	LOG_INF("read %d", regad);
	return 0;
}

static int sy1xx_mdio_write(const struct device *dev, uint8_t prtad, uint8_t regad, uint16_t data)
{
	LOG_INF("write %d", regad);
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
		.mdc_freq = DT_INST_PROP(n , clock_frequency),                                   \
	};                                                                                         \
                                                                                                   \
	static struct sy1xx_mdio_dev_data sy1xx_mdio_dev_data##n;                                  \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, &sy1xx_mdio_initialize, NULL, &sy1xx_mdio_dev_data##n,            \
			      &sy1xx_mdio_dev_config_##n, POST_KERNEL, CONFIG_MDIO_INIT_PRIORITY,  \
			      &sy1xx_mdio_driver_api);

DT_INST_FOREACH_STATUS_OKAY(SY1XX_MDIO_INIT)
