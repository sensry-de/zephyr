/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2024 sensry.io
 */

#define DT_DRV_COMPAT microchip_vsc8541

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(microchip_vsc8541, CONFIG_PHY_LOG_LEVEL);

#include <zephyr/kernel.h>
#include <zephyr/net/phy.h>
#include <zephyr/net/mii.h>
#include <zephyr/drivers/mdio.h>
#include <string.h>
#include <zephyr/sys/util_macro.h>
#include <zephyr/drivers/gpio.h>

enum vsc8541_interface {
	VSC8541_MII,
	VSC8541_RMII,
	VSC8541_RMII_25MHZ,
};

struct mc_vsc8541_config {
	uint8_t addr;
	const struct device *mdio_dev;
	enum vsc8541_interface phy_iface;
#if DT_ANY_INST_HAS_PROP_STATUS_OKAY(reset_gpios)
	const struct gpio_dt_spec reset_gpio;
#endif
#if DT_ANY_INST_HAS_PROP_STATUS_OKAY(int_gpios)
	const struct gpio_dt_spec interrupt_gpio;
#endif
};

/* Thread stack size */
#define STACK_SIZE 512

/* Thread priority */
#define THREAD_PRIORITY 7

static void phy_mc_vsc8541_link_monitor(void *arg1, void *arg2, void *arg3);

struct mc_vsc8541_data {
	const struct device *dev;
	struct phy_link_state state;
	phy_callback_t cb;
	void *cb_data;
	struct k_mutex mutex;

	struct k_thread link_monitor_thread;
	uint8_t link_monitor_thread_stack[STACK_SIZE];
};

static int phy_mc_vsc8541_init(const struct device *dev)
{
	struct mc_vsc8541_data *data = dev->data;

	LOG_INF("initialize");

	data->cb = NULL;
	data->cb_data = NULL;
	data->state.is_up = false;
	data->state.speed = LINK_HALF_10BASE_T;

	/* setup thread to watch link state */
	/* \TODO: should be interrupt driven by phy gpio */
	k_thread_create(&data->link_monitor_thread,
			(k_thread_stack_t *)data->link_monitor_thread_stack, STACK_SIZE,
			phy_mc_vsc8541_link_monitor, (void *)dev, NULL, NULL, THREAD_PRIORITY, 0,
			K_NO_WAIT);

	k_thread_name_set(&data->link_monitor_thread, "phy-link-mon");

	return 0;
}

static int phy_mc_vsc8541_get_link(const struct device *dev, struct phy_link_state *state)
{
	LOG_INF("get link status: %d", state->is_up);
	return 0;
}

static int phy_mc_vsc8541_cfg_link(const struct device *dev, enum phy_link_speed speeds)
{
}

static int phy_mc_vsc8541_link_cb_set(const struct device *dev, phy_callback_t cb, void *user_data)
{
	struct mc_vsc8541_data *data = dev->data;

	data->cb = cb;
	data->cb_data = user_data;

	phy_mc_vsc8541_get_link(dev, &data->state);

	data->cb(dev, &data->state, data->cb_data);

	return 0;
}

void phy_mc_vsc8541_link_monitor(void *arg1, void *arg2, void *arg3)
{
	const struct device *dev = arg1;
	struct mc_vsc8541_data *data = dev->data;

	while (1) {
		k_sleep(K_MSEC(5000));

		data->state.is_up = true;
		if (data->cb) {
			data->cb(dev, &data->state, data->cb_data);
		}

		k_sleep(K_MSEC(60000));

		data->state.is_up = false;
		if (data->cb) {
			data->cb(dev, &data->state, data->cb_data);
		}
	}
}

static int phy_mc_vsc8541_read(const struct device *dev, uint16_t reg_addr, uint32_t *data)
{
	LOG_INF("read");
	return 0;
}

static int phy_mc_vsc8541_write(const struct device *dev, uint16_t reg_addr, uint32_t data)
{
	LOG_INF("write");
	return 0;
}

static DEVICE_API(ethphy, mc_vsc8541_phy_api) = {
	.get_link = phy_mc_vsc8541_get_link,
	.cfg_link = phy_mc_vsc8541_cfg_link,
	.link_cb_set = phy_mc_vsc8541_link_cb_set,
	.read = phy_mc_vsc8541_read,
	.write = phy_mc_vsc8541_write,
};

#if DT_ANY_INST_HAS_PROP_STATUS_OKAY(reset_gpios)
#define RESET_GPIO(n) .reset_gpio = GPIO_DT_SPEC_INST_GET_OR(n, reset_gpios, {0}),
#else
#define RESET_GPIO(n)
#endif /* reset gpio */

#if DT_ANY_INST_HAS_PROP_STATUS_OKAY(int_gpios)
#define INTERRUPT_GPIO(n) .interrupt_gpio = GPIO_DT_SPEC_INST_GET_OR(n, int_gpios, {0}),
#else
#define INTERRUPT_GPIO(n)
#endif /* interrupt gpio */

#define MICROCHIP_VSC8541_INIT(n)                                                                  \
	static const struct mc_vsc8541_config mc_vsc8541_##n##_config = {                          \
		.addr = DT_INST_REG_ADDR(n),                                                       \
		.mdio_dev = DEVICE_DT_GET(DT_INST_PARENT(n)),                                      \
		.phy_iface = DT_INST_ENUM_IDX(n, microchip_interface_type),                        \
		RESET_GPIO(n) INTERRUPT_GPIO(n)};                                                  \
                                                                                                   \
	static struct mc_vsc8541_data mc_vsc8541_##n##_data;                                       \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, &phy_mc_vsc8541_init, NULL, &mc_vsc8541_##n##_data,               \
			      &mc_vsc8541_##n##_config, POST_KERNEL, CONFIG_PHY_INIT_PRIORITY,     \
			      &mc_vsc8541_phy_api);

DT_INST_FOREACH_STATUS_OKAY(MICROCHIP_VSC8541_INIT)
