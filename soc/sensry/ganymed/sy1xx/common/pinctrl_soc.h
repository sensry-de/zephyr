/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2024 sensry.io
 */

#ifndef GANYMED_SY1XX_PINCTRL_SOC_H
#define GANYMED_SY1XX_PINCTRL_SOC_H

#include <stdint.h>

#include <zephyr/devicetree.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
	uint32_t addr;
	uint8_t iro;
} pinctrl_soc_pin_t;

#define Z_PINCTRL_STATE_PIN_INIT(node, pr, idx)                                                    \
	{                                                                                          \
                                                                                                   \
		.addr = DT_PROP_BY_PHANDLE_IDX(node, pr, idx, addr),                               \
		.iro = DT_PROP_BY_PHANDLE_IDX(node, pr, idx, iro),                                 \
                                                                                                   \
	},

#define Z_PINCTRL_STATE_PINS_INIT(node_id, prop)                                                   \
	{DT_FOREACH_PROP_ELEM(node_id, prop, Z_PINCTRL_STATE_PIN_INIT)}

#ifdef __cplusplus
}
#endif

#endif // GANYMED_SY1XX_PINCTRL_SOC_H
