/*
* SPDX-License-Identifier: Apache-2.0
* Copyright (c) 2024 sensry.io
*/

#define DT_DRV_COMPAT sensry_sy1xx_pinctrl

#include <zephyr/drivers/pinctrl.h>

int pinctrl_configure_pins(const pinctrl_soc_pin_t *pins, uint8_t pin_cnt,
			   uintptr_t reg)
{
	return 0;
}
