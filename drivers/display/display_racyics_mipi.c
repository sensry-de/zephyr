/*
 * Copyright (c) 2026 Racyics GmbH.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

 /* Includes */

#include <errno.h>
#include <soc.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/display.h>
#include <zephyr/drivers/mipi_dsi.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/pm/device.h>

#include "display_racyics_mipi.h"

LOG_MODULE_REGISTER(display_racyics_mipi, CONFIG_DISPLAY_LOG_LEVEL);

/* Defines */

#define DT_DRV_COMPAT racyics_mipi_display

#define RI_MIPI_DISPLAY_PIXEL_SIZE(inst) ( DISPLAY_BITS_PER_PIXEL(DT_INST_PROP(inst, pixel_format)) / BITS_PER_BYTE )

#define I2C_NODE DT_NODELABEL(i2c_controller)

struct ri_mipi_cfg {
    uint32_t               id;
    uint32_t               x_res;
    uint32_t               y_res;
    const struct device    *i2c_bus;
};

struct ri_mipi_data {
    uint8_t                pixel_size;
    uint8_t                pixel_format;
};

/* Helper Declarations */

/* Helper Functions */

/* API Functions */

static int ri_mipi_display_init(const struct device *dev)
{
    int err = 0;

    const struct ri_mipi_cfg *cfg = dev->config;

    LOG_INF("Initializing MIPI Display ");

    // Init of I2C (10KHz)
    if (!device_is_ready(cfg->i2c_bus)) {
        LOG_ERR("I2C device not ready!");
        return -EINVAL;
    }

    { // Config specific display
        ri_mipi_disp_parameters_t display_cfg;
       display_cfg.type=RI_MIPI_DISP_TYPE_TFT_F_AND_S;
       display_cfg.vertical.active_lines      = RI_MIPI_DISP_ACTIVE_HEIGTH_TFT_F_AND_S;
       display_cfg.vertical.front_porch       = 3;
       display_cfg.vertical.sync_pulse        = 10;
       display_cfg.vertical.back_porch        = 38;
       display_cfg.horizontal.active_lines    = RI_MIPI_DISP_ACTIVE_WIDTH_TFT_F_AND_S;
       display_cfg.horizontal.front_porch     = 56;
       display_cfg.horizontal.sync_pulse      = 128;
       display_cfg.horizontal.back_porch      = 72;
       display_cfg.nr_of_lanes                = 4;
    }

    LOG_INF("DONE");

    return err;
}

static int ri_mipi_display_image_write( const struct device *dev,
                                        const uint16_t x, const uint16_t y,
                                        const struct display_buffer_descriptor *desc,
                                        const void *buf)
{
    int err = 0;
    return err;
}

static void ri_mipi_display_get_capabilities(const struct device *dev,
			                                struct display_capabilities *capabilities)
{
    const struct ri_mipi_cfg *cfg = dev->config;
    struct ri_mipi_data *data = dev->data;

    memset(capabilities, 0, sizeof(struct display_capabilities));

    capabilities->x_resolution = cfg->x_res;
	capabilities->y_resolution = cfg->y_res;
    capabilities->current_orientation = DISPLAY_ORIENTATION_NORMAL;
	capabilities->current_pixel_format = data->pixel_format;
    capabilities->supported_pixel_formats = PIXEL_FORMAT_RGB_888;
}

#ifdef CONFIG_PM_DEVICE
static int ri_mipi_display_pm_action(const struct device *dev, enum pm_device_action action)
{
    int ret = 0; 

    switch (action) {
	case PM_DEVICE_ACTION_RESUME:
		/* TODO*/
		break;
	case PM_DEVICE_ACTION_SUSPEND:
		/* TODO*/
		break;
	default:
		ret = -ENOTSUP;
		break;
	}

	return ret;
}
#endif /* CONFIG_PM_DEVICE */

/* Device Instatiation */
static DEVICE_API(display, ri_mipi_display_api) = {
	.write =  ri_mipi_display_image_write,
    .get_capabilities = ri_mipi_display_get_capabilities
};

#define RI_MIPI_DISP_DEFINE(inst)                                       \
                                                                        \
                                                                        \
    static const struct ri_mipi_cfg ri_mipi_cfg_##inst = {              \
        .id = inst,                                                     \
        .x_res = DT_INST_PROP(inst, width),                             \
        .y_res = DT_INST_PROP(inst, height),                            \
        .i2c_bus = DEVICE_DT_GET(DT_INST_PHANDLE(inst, i2c_bus))        \
    };                                                                  \
                                                                        \
    static struct ri_mipi_data ri_mipi_data_##inst = {                  \
        .pixel_size = RI_MIPI_DISPLAY_PIXEL_SIZE(inst),                 \
        .pixel_format = DT_INST_PROP(inst, pixel_format),               \
    };                                                                  \
                                                                        \
    PM_DEVICE_DT_INST_DEFINE(inst, ri_mipi_display_pm_action);          \
                                                                        \
    DEVICE_DT_INST_DEFINE(  inst, ri_mipi_display_init,                 \
                            PM_DEVICE_DT_INST_GET(inst),                \
                            &ri_mipi_data_##inst, &ri_mipi_cfg_##inst,  \
                            POST_KERNEL, CONFIG_DISPLAY_INIT_PRIORITY,  \
                            &ri_mipi_display_api );                     \

DT_INST_FOREACH_STATUS_OKAY(RI_MIPI_DISP_DEFINE)