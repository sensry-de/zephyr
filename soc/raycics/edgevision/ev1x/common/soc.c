//
// Created by tswaehn on 4/20/26.
//

#include "soc.h"

#include <zephyr/drivers/pinctrl.h>
#include <zephyr/arch/arm/nmi.h>
#include <zephyr/cache.h>
#include <zephyr/sys/util.h>



void soc_reset_hook(void)
{

}

void soc_early_init_hook(void)
{


}

void soc_late_init_hook(void)
{

}

int pinctrl_configure_pins(const pinctrl_soc_pin_t *pins, uint8_t pin_cnt, uintptr_t reg)
{
	ARG_UNUSED(pins);
	ARG_UNUSED(pin_cnt);
	ARG_UNUSED(reg);

	return 0;
}
