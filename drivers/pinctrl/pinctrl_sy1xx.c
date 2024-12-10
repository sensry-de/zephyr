/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2024 sensry.io
 */

#define DT_DRV_COMPAT sensry_sy1xx_pinctrl

#include <zephyr/drivers/pinctrl.h>

#include <soc.h>
#include <zephyr/arch/common/sys_io.h>

#define SY1XX_PAD_CONFIG_ADDR (SY1XX_ARCHI_SOC_PERIPHERALS_ADDR + SY1XX_ARCHI_APB_SOC_CTRL_OFFSET)

/**
 * @brief Configure a pin.
 *
 * @param pin The pin to configure.
 */
static int32_t pinctrl_configure_pin(const pinctrl_soc_pin_t *pin)
{
	if ((pin->addr & 0xFFFFF000) != SY1XX_PAD_CONFIG_ADDR) {
		/* invalid address range */
		return -EINVAL;
	}

	switch (pin->iro) {
	case 0:
	case 8:
	case 16:
	case 24:
		/* fall through */
		break;
	default:
		/* invalid inter address offset */
		return -EINVAL;
	}

	volatile uint32_t reg = sys_read32(pin->addr);
	reg &= ~(0xFFUL << pin->iro);
	reg |= (pin->cfg << pin->iro);
	sys_write32(reg, pin->addr);

	return 0;
}

/**
 * @brief Configure a set of pins.
 *
 * This function will configure the necessary hardware blocks to make the
 * configuration immediately effective.
 *
 * @warning This function must never be used to configure pins used by an
 * instantiated device driver.
 *
 * @param pins List of pins to be configured.
 * @param pin_cnt Number of pins.
 * @param reg Device register (optional, use #PINCTRL_REG_NONE if not used).
 *
 * @retval 0 If succeeded
 * @retval -errno Negative errno for other failures.
 */
int pinctrl_configure_pins(const pinctrl_soc_pin_t *pins, uint8_t pin_cnt, uintptr_t reg)
{
	ARG_UNUSED(reg);

	for (uint8_t i = 0U; i < pin_cnt; i++) {
		int32_t ret = pinctrl_configure_pin(pins++);

		if (ret != 0) {
			return ret;
		}
	}

	return 0;
}
