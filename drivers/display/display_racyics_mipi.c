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

#include "display_racyics_mipi.h"

/* Defines */

#define DT_DRV_COMPAT racyics_mipi_display

#define RI_MIPI_DISPLAY_PIXEL_SIZE(inst) ( DISPLAY_BITS_PER_PIXEL(DT_INST_PROP(inst, pixel_format)) / BITS_PER_BYTE )

struct ri_mipi_cfg {
    uint32_t               id;
    uint32_t               x_res;
    uint32_t               y_res;
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
        .y_res = DT_INST_PROP(inst, height)                             \
                                                                        \
    };                                                                  \
                                                                        \
    static struct ri_mipi_data ri_mipi_data_##inst = {                  \
        .pixel_size = RI_MIPI_DISPLAY_PIXEL_SIZE(inst),                 \
        .pixel_format = DT_INST_PROP(inst, pixel_format),               \
    };                                                                  \
                                                                        \
    DEVICE_DT_INST_DEFINE(  inst, ri_mipi_display_init,                 \
                            PM_DEVICE_DT_INST_GET(inst),                \
                            &ri_mipi_data_##inst, &ri_mipi_cfg_##inst,  \
                            POST_KERNEL, CONFIG_DISPLAY_INIT_PRIORITY,  \
                            &ri_mipi_display_api );                     \

DT_INST_FOREACH_STATUS_OKAY(RI_MIPI_DISP_DEFINE)