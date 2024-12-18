//
// Created by tswaehn on 12/16/24.
//

#define DT_DRV_COMPAT sensry_sy1xx_gpio

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(sy1xx_gpio, CONFIG_GPIO_LOG_LEVEL);

#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <soc.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/gpio/gpio_utils.h>
#include <pinctrl_soc.h>
#include <zephyr/drivers/pinctrl.h>

/* Driver-specific configuration structure */
struct sy1xx_gpio_config {
	/* Base address or registers for GPIO */
	uint32_t pinctrl_base_addr;
	uint32_t port_set_addr;
	uint32_t port_clr_addr;
	uint32_t port_get_addr;
	uint32_t pin_mask;
	/* Other driver-specific configurations */
};

/* Driver-specific runtime structure */
struct sy1xx_gpio_data {
	/* Runtime state, like pin configurations */
};

/* Function prototypes for the GPIO API */
static int sy1xx_gpio_driver_configure(const struct device *dev, gpio_pin_t pin,
				       gpio_flags_t flags);
static int sy1xx_gpio_driver_port_get_raw(const struct device *dev, gpio_port_value_t *value);
static int sy1xx_gpio_driver_port_set_masked_raw(const struct device *dev, gpio_port_pins_t mask,
						 gpio_port_value_t value);
static int sy1xx_gpio_driver_port_set_bits_raw(const struct device *dev, gpio_port_pins_t pins);
static int sy1xx_gpio_driver_port_clear_bits_raw(const struct device *dev, gpio_port_pins_t pins);
static int sy1xx_gpio_driver_port_toggle_bits(const struct device *dev, gpio_port_pins_t pins);

static int sy1xx_gpio_driver_init(const struct device *dev)
{
	return 0;
}

int sy1xx_gpio_driver_configure(const struct device *dev, gpio_pin_t pin, gpio_flags_t flags)
{
	const struct sy1xx_gpio_config *const cfg = dev->config;

	LOG_DBG("configure gpio %d to %x", pin, flags);

	if (((1 << pin) & cfg->pin_mask) == 0) {
		return -EINVAL;
	}

	pinctrl_soc_pin_t pcfg = {
		.addr = cfg->pinctrl_base_addr + ROUND_DOWN(pin, 4),
		.iro = (pin % 4) * 8,
		.cfg = 0,
	};

	/* translate gpio flags into pinctrl config */
	if (flags & GPIO_INPUT) {

		if (flags & GPIO_PULL_UP) {
			pcfg.cfg |= (1 << SY1XX_PAD_PULL_UP_OFFS);
		}
		if (flags & GPIO_PULL_DOWN) {
			pcfg.cfg |= (1 << SY1XX_PAD_PULL_DOWN_OFFS);
		}

	} else if (flags & GPIO_OUTPUT) {
		pcfg.cfg |= (1 << SY1XX_PAD_DIR_OFFS);

		//pcfg.cfg |= (1 << SY1XX_PAD_TRISTATE_OFFS);
		if (flags & GPIO_OUTPUT_INIT_LOW) {
		}
		if (flags & GPIO_OUTPUT_INIT_HIGH) {
		}

	} else if (flags == GPIO_DISCONNECTED) {
		pcfg.cfg |= (1 << SY1XX_PAD_TRISTATE_OFFS);

	} else {
		return -ENOTSUP;
	}

	/* PAD config */
	int32_t ret = pinctrl_configure_pins(&pcfg, 1, PINCTRL_STATE_DEFAULT);
	if (ret != 0) {
		return -EINVAL;
	}


	return 0;
}

int sy1xx_gpio_driver_port_get_raw(const struct device *dev, gpio_port_value_t *value)
{
	const struct sy1xx_gpio_config *const cfg = dev->config;

	*value = sys_read32(cfg->port_get_addr);
	return 0;
}

int sy1xx_gpio_driver_port_set_masked_raw(const struct device *dev, gpio_port_pins_t mask,
					  gpio_port_value_t value)
{
	const struct sy1xx_gpio_config *const cfg = dev->config;

	uint32_t set_mask = (mask & value) & (cfg->pin_mask);
	uint32_t clr_mask = (mask & (~value)) & (cfg->pin_mask);

	sy1xx_gpio_driver_port_set_bits_raw(dev, set_mask);
	sy1xx_gpio_driver_port_clear_bits_raw(dev, clr_mask);
	return 0;
}

int sy1xx_gpio_driver_port_set_bits_raw(const struct device *dev, gpio_port_pins_t pins)
{
	const struct sy1xx_gpio_config *const cfg = dev->config;

	/* affects only pins that are set to logical '1' */
	sys_write32((uint32_t)pins, cfg->port_set_addr);
	return 0;
}

int sy1xx_gpio_driver_port_clear_bits_raw(const struct device *dev, gpio_port_pins_t pins)
{
	const struct sy1xx_gpio_config *const cfg = dev->config;

	/* affects only pins that are set to logical '1' */
	sys_write32((uint32_t)pins, cfg->port_clr_addr);
	return 0;
}

int sy1xx_gpio_driver_port_toggle_bits(const struct device *dev, gpio_port_pins_t pins)
{
	const struct sy1xx_gpio_config *const cfg = dev->config;

	uint32_t current = sys_read32(cfg->port_get_addr);

	sy1xx_gpio_driver_port_set_masked_raw(dev, pins, ~current);
	return 0;
}

/* Define the GPIO API structure */
static const struct gpio_driver_api sy1xx_gpio_driver_api = {
	.pin_configure = sy1xx_gpio_driver_configure,
	.port_get_raw = sy1xx_gpio_driver_port_get_raw,
	.port_set_masked_raw = sy1xx_gpio_driver_port_set_masked_raw,
	.port_set_bits_raw = sy1xx_gpio_driver_port_set_bits_raw,
	.port_clear_bits_raw = sy1xx_gpio_driver_port_clear_bits_raw,
	.port_toggle_bits = sy1xx_gpio_driver_port_toggle_bits,
};

#define SY1XX_GPIO_INIT(n)                                                                         \
                                                                                                   \
	static const struct sy1xx_gpio_config sy1xx_gpio_##n##_config = {                          \
		.pinctrl_base_addr = (uint32_t)DT_INST_REG_ADDR_BY_IDX(n, 0),                      \
		.port_set_addr = (uint32_t)DT_INST_REG_ADDR_BY_IDX(n, 1),                          \
		.port_clr_addr = (uint32_t)DT_INST_REG_ADDR_BY_IDX(n, 2),                          \
		.port_get_addr = (uint32_t)DT_INST_REG_ADDR_BY_IDX(n, 3),                          \
		.pin_mask = (uint32_t)GPIO_PORT_PIN_MASK_FROM_DT_INST(n)};                         \
                                                                                                   \
	static struct sy1xx_gpio_data sy1xx_gpio_##n##_data;                                       \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, sy1xx_gpio_driver_init, NULL, &sy1xx_gpio_##n##_data,             \
			      &sy1xx_gpio_##n##_config, PRE_KERNEL_1, CONFIG_GPIO_INIT_PRIORITY,   \
			      &sy1xx_gpio_driver_api);

DT_INST_FOREACH_STATUS_OKAY(SY1XX_GPIO_INIT)
