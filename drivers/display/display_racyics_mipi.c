/*
 * Copyright (c) 2026 Racyics GmbH.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT racyics_mipi_display

#include <errno.h>
#include <string.h>
#include <soc.h>
#include <zephyr/cache.h>
#include <zephyr/kernel.h>
#include <zephyr/linker/devicetree_regions.h>
#include <zephyr/logging/log.h>
#include <zephyr/display/mipi_display.h>
#include <zephyr/drivers/display.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/mipi_dsi.h>
#include <zephyr/drivers/mipi_dsi/mipi_dsi_racyics.h>
#include <zephyr/pm/device.h>

#include "display_racyics_mipi.h"
#include "display_racyics_mipi_nt35521.h"

LOG_MODULE_REGISTER(display_racyics_mipi, CONFIG_DISPLAY_LOG_LEVEL);

#define RI_MIPI_BYTES_PER_PIXEL	3U
#define RI_MIPI_FB_WIDTH	CONFIG_DISPLAY_RACYICS_MIPI_FB_WIDTH
#define RI_MIPI_FB_HEIGHT	CONFIG_DISPLAY_RACYICS_MIPI_FB_HEIGHT

#define RI_MIPI_FB_SECTION                                                                         \
	Z_GENERIC_SECTION(LINKER_DT_NODE_REGION_NAME(DT_NODELABEL(data_sram1)))

/* DCU layer pixel format register value for RGB888 */
#define DCU_LAYER_PIXFMT_RGB888		6U

/* DCU layer reload control values */
#define DCU_LAYER_RELOAD_IMMEDIATE	1U
#define DCU_LAYER_RELOAD_NEXT_VSYNC	2U

#define DCU_LAYER_DEFAULT_COLOR		0xFF00FF00U

/* Control registers of the PCA9634 LED driver on the F&S adapter board */
#define LEDDRV_ADDR			0x61U
#define LEDDRV_REG_MODE1		0x00U
#define LEDDRV_REG_MODE2		0x01U
#define LEDDRV_REG_LEDOUT0		0x0CU
#define LEDDRV_OUT_BLON			0U
#define LEDDRV_OUT_BLPWM		1U
#define LEDDRV_OUT_RESX			3U
#define LEDDRV_OUT_COUNT		8U

#define FPGA_TOP_OUT_DSI_I2C_OE_N	BIT(25)

#define PANEL_RESET_DELAY_MS		200
#define PANEL_SOFT_RESET_DELAY_MS	120
#define PANEL_SLEEP_OUT_DELAY_MS	800
#define PANEL_DISPLAY_ON_DELAY_MS	40

#define PANEL_COLOR_FORMAT_RGB888	0x77U

struct ri_mipi_cfg {
	const struct device *mipi_dsi;
	const struct device *i2c_bus;
	uint8_t channel;
	uint8_t num_of_lanes;
	uint16_t x_res;
	uint16_t y_res;
	uint8_t *fb;
};

struct ri_mipi_data {
	uint8_t ledout[LEDDRV_OUT_COUNT / 4U];
	bool blanking;
};

/* LED driver helpers */

static int leddrv_write(const struct device *dev, uint8_t reg, uint8_t val)
{
	const struct ri_mipi_cfg *cfg = dev->config;
	int err;

	err = i2c_reg_write_byte(cfg->i2c_bus, LEDDRV_ADDR, reg, val);
	if (err != 0) {
		LOG_ERR("I2C write 0x%02x -> 0x%02x failed (%d)", reg, val, err);
	}

	return err;
}

static int leddrv_set_out(const struct device *dev, uint8_t idx, bool val)
{
	const struct ri_mipi_cfg *cfg = dev->config;
	struct ri_mipi_data *data = dev->data;
	const uint8_t reg_idx = idx / 4U;
	const uint8_t shift = (idx % 4U) * 2U;
	uint8_t expected;
	uint8_t readback;
	int err;

	if (idx >= LEDDRV_OUT_COUNT) {
		return -EINVAL;
	}

	/* 1 drives the output fully on, 0 fully off */
	expected = data->ledout[reg_idx] & (uint8_t)~(0x3U << shift);
	expected |= (uint8_t)((val ? 1U : 0U) << shift);

	err = leddrv_write(dev, LEDDRV_REG_LEDOUT0 + reg_idx, expected);
	if (err != 0) {
		return err;
	}

	err = i2c_reg_read_byte(cfg->i2c_bus, LEDDRV_ADDR, LEDDRV_REG_LEDOUT0 + reg_idx,
				&readback);
	if (err != 0) {
		LOG_ERR("I2C read back of LEDOUT%u failed (%d)", reg_idx, err);
		return err;
	}

	data->ledout[reg_idx] = readback;
	if (readback != expected) {
		LOG_ERR("LEDOUT%u mismatch: expected 0x%02x, got 0x%02x", reg_idx, expected,
			readback);
		return -EIO;
	}

	return 0;
}

static int leddrv_init(const struct device *dev)
{
	int err;

	/* MODE1: low power mode, all outputs off */
	err = leddrv_write(dev, LEDDRV_REG_MODE1, BIT(4));
	if (err != 0) {
		return err;
	}

	/* MODE2: inverted output logic, so written values match the level on the wire */
	err = leddrv_write(dev, LEDDRV_REG_MODE2, 0x15U);
	if (err != 0) {
		return err;
	}

	for (uint8_t i = 0U; i < LEDDRV_OUT_COUNT; i++) {
		err = leddrv_set_out(dev, i, true);
		if (err != 0) {
			return err;
		}
	}

	/* MODE1: normal operation */
	return leddrv_write(dev, LEDDRV_REG_MODE1, 0U);
}

static int panel_hw_reset(const struct device *dev)
{
	int err;

	err = leddrv_set_out(dev, LEDDRV_OUT_RESX, false);
	if (err != 0) {
		return err;
	}
	k_msleep(PANEL_RESET_DELAY_MS);

	err = leddrv_set_out(dev, LEDDRV_OUT_RESX, true);
	if (err != 0) {
		return err;
	}
	k_msleep(PANEL_RESET_DELAY_MS);

	return 0;
}

/* Panel helpers */

static int panel_dcs_write(const struct device *dev, uint8_t cmd, const void *buf, size_t len)
{
	const struct ri_mipi_cfg *cfg = dev->config;
	ssize_t ret;

	ret = mipi_dsi_dcs_write(cfg->mipi_dsi, cfg->channel, cmd, buf, len);

	return (ret < 0) ? (int)ret : 0;
}

static int panel_init(const struct device *dev)
{
	const struct ri_mipi_cfg *cfg = dev->config;
	const uint8_t color_format = PANEL_COLOR_FORMAT_RGB888;
	const uint8_t brightness[2] = {0xFFU, 0xFFU};
	ssize_t ret;
	int err;

	err = panel_dcs_write(dev, MIPI_DCS_SOFT_RESET, NULL, 0U);
	if (err != 0) {
		LOG_ERR("Soft reset failed (%d)", err);
		return err;
	}
	k_msleep(PANEL_SOFT_RESET_DELAY_MS);

	for (size_t i = 0U; i < ARRAY_SIZE(nt35521_t070qyh_13cp_init); i++) {
		const struct nt35521_cmd *cmd = &nt35521_t070qyh_13cp_init[i];

		ret = mipi_dsi_generic_write(cfg->mipi_dsi, cfg->channel, cmd->data, cmd->len);
		if (ret < 0) {
			LOG_ERR("Init command %zu failed (%d)", i, (int)ret);
			return (int)ret;
		}
	}

	err = panel_dcs_write(dev, MIPI_DCS_SET_PIXEL_FORMAT, &color_format, 1U);
	if (err != 0) {
		LOG_ERR("Setting the pixel format failed (%d)", err);
		return err;
	}

	err = panel_dcs_write(dev, MIPI_DCS_SET_DISPLAY_BRIGHTNESS, brightness,
			      sizeof(brightness));
	if (err != 0) {
		LOG_ERR("Setting the brightness failed (%d)", err);
		return err;
	}

	err = panel_dcs_write(dev, MIPI_DCS_EXIT_SLEEP_MODE, NULL, 0U);
	if (err != 0) {
		LOG_ERR("Exit sleep mode failed (%d)", err);
		return err;
	}
	k_msleep(PANEL_SLEEP_OUT_DELAY_MS);

	err = panel_dcs_write(dev, MIPI_DCS_SET_DISPLAY_ON, NULL, 0U);
	if (err != 0) {
		LOG_ERR("Display on failed (%d)", err);
		return err;
	}
	k_msleep(PANEL_DISPLAY_ON_DELAY_MS);

	return 0;
}

/* Display controller unit helpers */

static void dcu_config_canvas(void)
{
	uint32_t x = RI_MIPI_DISP_H_SYNC_PULSE;
	uint32_t y = RI_MIPI_DISP_V_SYNC_PULSE;

	WRITEREG_LOG("DCU_SYNC_SIZE", DCU_GLOBAL_SYNC_SIZE_REG, ((x - 1U) << 16) | (y - 1U));

	x += RI_MIPI_DISP_H_BACK_PORCH;
	y += RI_MIPI_DISP_V_BACK_PORCH;
	WRITEREG_LOG("DCU_BACK_PORCH", DCU_GLOBAL_BACK_PORCH__ACCUMULATED__REG,
		     ((x - 1U) << 16) | (y - 1U));

	x += DT_PROP(DT_NODELABEL(panel), width);
	y += DT_PROP(DT_NODELABEL(panel), height);
	WRITEREG_LOG("DCU_ACTIVE_WIDTH", DCU_GLOBAL_ACTIVE_WIDTH_REG,
		     ((x - 1U) << 16) | (y - 1U));

	x += RI_MIPI_DISP_H_FRONT_PORCH;
	y += RI_MIPI_DISP_V_FRONT_PORCH;
	WRITEREG_LOG("DCU_TOTAL_WIDTH", DCU_GLOBAL_TOTAL_WIDTH_REG, ((x - 1U) << 16) | (y - 1U));
}

static void dcu_start(void)
{
	WRITEREG_LOG("DCU_IRQ_CLEAR", DCU_GLOBAL_IRQ_CLEAR_REG, 0xFFFFFFFFU);
	WRITEREG_LOG("DCU_SECONDARY_IRQ_CLEAR", DCU_GLOBAL_SECONDARY_IRQ_CLEAR_REG, 0xFFFFFFFFU);
	WRITEREG_LOG("DCU_IRQ_ENABLE", DCU_GLOBAL_IRQ_ENABLE_REG, 0x1U);
	WRITEREG_LOG("DCU_SHADOW_RELOAD", DCU_GLOBAL_SHADOW_RELOAD_CONTROL_REG, 1U);

	MODIFYREG(DCU_GLOBAL_GLOBAL_CONTROL_REG,
		  GLOBAL__GLOBAL_CONTROL__BLANK_POLARITY__FIELD |
		  GLOBAL__GLOBAL_CONTROL__VSYNC_POLARITY__FIELD |
		  GLOBAL__GLOBAL_CONTROL__HSYNC_POLARITY__FIELD |
		  GLOBAL__GLOBAL_CONTROL__CRC_EN__FIELD |
		  GLOBAL__GLOBAL_CONTROL__GLOBAL_ENABLE__FIELD |
		  GLOBAL__GLOBAL_CONTROL__STREAM_EN__FIELD |
		  GLOBAL__GLOBAL_CONTROL__SINGLE_FRAME_MODE_EN__FIELD,
		  GLOBAL__GLOBAL_CONTROL__VSYNC_POLARITY__FIELD |
		  GLOBAL__GLOBAL_CONTROL__HSYNC_POLARITY__FIELD |
		  GLOBAL__GLOBAL_CONTROL__CRC_EN__FIELD |
		  GLOBAL__GLOBAL_CONTROL__GLOBAL_ENABLE__FIELD);
}

static void dcu_layer_set_on(bool on, uint32_t reload)
{
	MODIFYREG(DCU_LAYER_LAYER_CONTROL_REG, LAYER__LAYER_CONTROL__LAYER_ON__FIELD,
		  on ? LAYER__LAYER_CONTROL__LAYER_ON__FIELD : 0U);
	WRITEREG(DCU_LAYER_LAYER_RELOAD_CONTROL_REG, reload);
}

static void dcu_layer_config(const struct ri_mipi_cfg *cfg)
{
	const uint32_t pitch = RI_MIPI_FB_WIDTH * RI_MIPI_BYTES_PER_PIXEL;
	const uint32_t x0 = RI_MIPI_DISP_H_SYNC_PULSE + RI_MIPI_DISP_H_BACK_PORCH +
			    ((cfg->x_res - RI_MIPI_FB_WIDTH) / 2U);
	const uint32_t y0 = RI_MIPI_DISP_V_SYNC_PULSE + RI_MIPI_DISP_V_BACK_PORCH +
			    ((cfg->y_res - RI_MIPI_FB_HEIGHT) / 2U);

	WRITEREG_LOG("LAYER_RELOAD", DCU_LAYER_LAYER_RELOAD_CONTROL_REG,
		     DCU_LAYER_RELOAD_NEXT_VSYNC);
	WRITEREG_LOG("LAYER_H_POS", DCU_LAYER_WINDOW_HORIZONTAL_POSITION_REG,
		     ((x0 + RI_MIPI_FB_WIDTH - 1U) << 16) | x0);
	WRITEREG_LOG("LAYER_V_POS", DCU_LAYER_WINDOW_VERTICAL_POSITION_REG,
		     ((y0 + RI_MIPI_FB_HEIGHT - 1U) << 16) | y0);
	WRITEREG_LOG("LAYER_FB_ADDR", DCU_LAYER_COLOR_FB_ADDRESS_REG, (uint32_t)cfg->fb);
	WRITEREG_LOG("LAYER_FB_LENGTH", DCU_LAYER_COLOR_FB_LENGTH_REG,
		     (pitch << 16) | (pitch + DCU_AXIBUSWORDS_BYTES - 1U));
	WRITEREG_LOG("LAYER_FB_LINES", DCU_LAYER_COLOR_FB_LINES_REG, RI_MIPI_FB_HEIGHT);
	WRITEREG_LOG("LAYER_PIXEL_FORMAT", DCU_LAYER_PIXEL_FORMAT_REG, DCU_LAYER_PIXFMT_RGB888);
	WRITEREG_LOG("LAYER_DEFAULT_COLOR", DCU_LAYER_DEFAULT_COLOR_REG,
		     DCU_LAYER_DEFAULT_COLOR);
	WRITEREG_LOG("LAYER_FB_BUS_CONTROL", DCU_LAYER_FB_BUS_CONTROL_REG, 0U);

	MODIFYREG(DCU_LAYER_LAYER_CONTROL_REG,
		  LAYER__LAYER_CONTROL__LAYER_ON__FIELD | LAYER__LAYER_CONTROL__SCALER_ENABLE__FIELD,
		  LAYER__LAYER_CONTROL__LAYER_ON__FIELD);
	WRITEREG_LOG("LAYER_RELOAD", DCU_LAYER_LAYER_RELOAD_CONTROL_REG,
		     DCU_LAYER_RELOAD_IMMEDIATE);
}

/* Make CPU writes visible to the DCU, a missing data cache needs no flush */
static int fb_flush(void *addr, size_t size)
{
	int err = sys_cache_data_flush_range(addr, size);

	return (err == -ENOTSUP) ? 0 : err;
}

/* API Functions */

static int ri_mipi_display_write(const struct device *dev, const uint16_t x, const uint16_t y,
				 const struct display_buffer_descriptor *desc, const void *buf)
{
	const struct ri_mipi_cfg *cfg = dev->config;
	const uint8_t *src = buf;
	const size_t fb_pitch = RI_MIPI_FB_WIDTH * RI_MIPI_BYTES_PER_PIXEL;
	const size_t src_pitch = (size_t)desc->pitch * RI_MIPI_BYTES_PER_PIXEL;
	const size_t row_bytes = (size_t)desc->width * RI_MIPI_BYTES_PER_PIXEL;

	if ((buf == NULL) || (desc->width == 0U) || (desc->height == 0U)) {
		return -EINVAL;
	}

	if (desc->pitch < desc->width) {
		LOG_ERR("Pitch %u is smaller than the width %u", desc->pitch, desc->width);
		return -EINVAL;
	}

	if (desc->buf_size < (src_pitch * (desc->height - 1U) + row_bytes)) {
		LOG_ERR("Buffer of %u bytes is too small", desc->buf_size);
		return -EINVAL;
	}

	if (((uint32_t)x + desc->width > RI_MIPI_FB_WIDTH) ||
	    ((uint32_t)y + desc->height > RI_MIPI_FB_HEIGHT)) {
		LOG_ERR("Area %ux%u at (%u,%u) exceeds the %ux%u framebuffer", desc->width,
			desc->height, x, y, RI_MIPI_FB_WIDTH, RI_MIPI_FB_HEIGHT);
		return -EINVAL;
	}

	for (uint16_t row = 0U; row < desc->height; row++) {
		uint8_t *dst = cfg->fb + ((size_t)(y + row) * fb_pitch) +
			       ((size_t)x * RI_MIPI_BYTES_PER_PIXEL);

		memcpy(dst, src + ((size_t)row * src_pitch), row_bytes);
	}

	/* The DCU reads the framebuffer through its own bus master */
	return fb_flush(cfg->fb + ((size_t)y * fb_pitch), (size_t)desc->height * fb_pitch);
}

static int ri_mipi_display_blanking_on(const struct device *dev)
{
	struct ri_mipi_data *data = dev->data;

	dcu_layer_set_on(false, DCU_LAYER_RELOAD_NEXT_VSYNC);
	data->blanking = true;

	return 0;
}

static int ri_mipi_display_blanking_off(const struct device *dev)
{
	struct ri_mipi_data *data = dev->data;

	dcu_layer_set_on(true, DCU_LAYER_RELOAD_NEXT_VSYNC);
	data->blanking = false;

	return 0;
}

static void ri_mipi_display_get_capabilities(const struct device *dev,
					     struct display_capabilities *capabilities)
{
	memset(capabilities, 0, sizeof(*capabilities));

	capabilities->x_resolution = RI_MIPI_FB_WIDTH;
	capabilities->y_resolution = RI_MIPI_FB_HEIGHT;
	capabilities->supported_pixel_formats = PIXEL_FORMAT_RGB_888;
	capabilities->current_pixel_format = PIXEL_FORMAT_RGB_888;
	capabilities->current_orientation = DISPLAY_ORIENTATION_NORMAL;
}

#ifdef CONFIG_PM_DEVICE
static int ri_mipi_display_pm_action(const struct device *dev, enum pm_device_action action)
{
	switch (action) {
	case PM_DEVICE_ACTION_RESUME:
		return ri_mipi_display_blanking_off(dev);
	case PM_DEVICE_ACTION_SUSPEND:
		return ri_mipi_display_blanking_on(dev);
	case PM_DEVICE_ACTION_TURN_ON:
	case PM_DEVICE_ACTION_TURN_OFF:
		return 0;
	default:
		return -ENOTSUP;
	}
}
#endif /* CONFIG_PM_DEVICE */

static int ri_mipi_display_init(const struct device *dev)
{
	const struct ri_mipi_cfg *cfg = dev->config;
	struct mipi_dsi_device mdev = {
		.data_lanes = cfg->num_of_lanes,
		.pixfmt = MIPI_DSI_PIXFMT_RGB888,
		.mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_BURST,
		.timings = {
			.hactive = cfg->x_res,
			.hfp = RI_MIPI_DISP_H_FRONT_PORCH,
			.hbp = RI_MIPI_DISP_H_BACK_PORCH,
			.hsync = RI_MIPI_DISP_H_SYNC_PULSE,
			.vactive = cfg->y_res,
			.vfp = RI_MIPI_DISP_V_FRONT_PORCH,
			.vbp = RI_MIPI_DISP_V_BACK_PORCH,
			.vsync = RI_MIPI_DISP_V_SYNC_PULSE,
		},
	};
	int err;

	if (!device_is_ready(cfg->i2c_bus)) {
		LOG_ERR("I2C bus is not ready");
		return -ENODEV;
	}

	if (!device_is_ready(cfg->mipi_dsi)) {
		LOG_ERR("DSI host is not ready");
		return -ENODEV;
	}

	if (READREG(DCU_GLOBAL_HW_VERSION_REG) == 0U) {
		LOG_ERR("DCU hardware version read failed");
		return -ENODEV;
	}

	memset(cfg->fb, 0, (size_t)RI_MIPI_FB_WIDTH * RI_MIPI_FB_HEIGHT * RI_MIPI_BYTES_PER_PIXEL);
	err = fb_flush(cfg->fb, (size_t)RI_MIPI_FB_WIDTH * RI_MIPI_FB_HEIGHT *
				RI_MIPI_BYTES_PER_PIXEL);
	if (err != 0) {
		return err;
	}

	err = mipi_dsi_attach(cfg->mipi_dsi, cfg->channel, &mdev);
	if (err != 0) {
		LOG_ERR("DSI attach failed (%d)", err);
		return err;
	}

	dcu_config_canvas();

	if (IS_ENABLED(CONFIG_BOARD_RIEV_FPGA_DEV_BOARD) ||
	    IS_ENABLED(CONFIG_BOARD_RIEV_DEV_BOARD)) {
		err = leddrv_init(dev);
		if (err != 0) {
			LOG_ERR("LED driver init failed (%d)", err);
			return err;
		}

		/* Release the output enable of the LED driver on the adapter board */
		MODIFYREG(RBB_FPGA_TOP_OUT_0_REG, FPGA_TOP_OUT_DSI_I2C_OE_N, 0U);

		err = panel_hw_reset(dev);
		if (err != 0) {
			LOG_ERR("Panel reset failed (%d)", err);
			return err;
		}

		err = leddrv_set_out(dev, LEDDRV_OUT_BLON, true);
		if (err != 0) {
			return err;
		}

		err = leddrv_set_out(dev, LEDDRV_OUT_BLPWM, false);
		if (err != 0) {
			return err;
		}
	}

	dcu_start();

	err = mipi_dsi_racyics_set_command_mode(cfg->mipi_dsi);
	if (err != 0) {
		return err;
	}

	err = panel_init(dev);
	if (err != 0) {
		return err;
	}

	err = mipi_dsi_racyics_start_video(cfg->mipi_dsi);
	if (err != 0) {
		return err;
	}

	dcu_layer_config(cfg);

	LOG_INF("Display initialized, %ux%u framebuffer", RI_MIPI_FB_WIDTH, RI_MIPI_FB_HEIGHT);

	return 0;
}

/* Device Instantiation */

static DEVICE_API(display, ri_mipi_display_api) = {
	.write = ri_mipi_display_write,
	.blanking_on = ri_mipi_display_blanking_on,
	.blanking_off = ri_mipi_display_blanking_off,
	.get_capabilities = ri_mipi_display_get_capabilities,
};

#define RI_MIPI_DISP_DEFINE(inst)                                                                  \
	BUILD_ASSERT((CONFIG_DISPLAY_RACYICS_MIPI_FB_WIDTH % 16) == 0,                             \
		     "Framebuffer width must be a multiple of 16");                                \
	BUILD_ASSERT(CONFIG_DISPLAY_RACYICS_MIPI_FB_WIDTH <= DT_INST_PROP(inst, width),            \
		     "Framebuffer wider than the panel");                                          \
	BUILD_ASSERT(CONFIG_DISPLAY_RACYICS_MIPI_FB_HEIGHT <= DT_INST_PROP(inst, height),          \
		     "Framebuffer higher than the panel");                                         \
	BUILD_ASSERT(CONFIG_DISPLAY_RACYICS_MIPI_FB_WIDTH * CONFIG_DISPLAY_RACYICS_MIPI_FB_HEIGHT  \
		     * RI_MIPI_BYTES_PER_PIXEL <= DT_REG_SIZE(DT_NODELABEL(data_sram1)),           \
		     "Framebuffer does not fit into DATA_SRAM1");                                  \
                                                                                                   \
	static uint8_t ri_mipi_fb_##inst[CONFIG_DISPLAY_RACYICS_MIPI_FB_WIDTH *                    \
					 CONFIG_DISPLAY_RACYICS_MIPI_FB_HEIGHT *                   \
					 RI_MIPI_BYTES_PER_PIXEL]                                  \
		__aligned(CONFIG_DCACHE_LINE_SIZE) RI_MIPI_FB_SECTION;                             \
                                                                                                   \
	static const struct ri_mipi_cfg ri_mipi_cfg_##inst = {                                     \
		.mipi_dsi = DEVICE_DT_GET(DT_INST_BUS(inst)),                                      \
		.i2c_bus = DEVICE_DT_GET(DT_INST_PHANDLE(inst, i2c_bus)),                          \
		.channel = DT_INST_REG_ADDR(inst),                                                 \
		.num_of_lanes = DT_INST_PROP_BY_IDX(inst, data_lanes, 0),                          \
		.x_res = DT_INST_PROP(inst, width),                                                \
		.y_res = DT_INST_PROP(inst, height),                                               \
		.fb = ri_mipi_fb_##inst,                                                           \
	};                                                                                         \
                                                                                                   \
	static struct ri_mipi_data ri_mipi_data_##inst;                                            \
                                                                                                   \
	PM_DEVICE_DT_INST_DEFINE(inst, ri_mipi_display_pm_action);                                 \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, ri_mipi_display_init, PM_DEVICE_DT_INST_GET(inst),             \
			      &ri_mipi_data_##inst, &ri_mipi_cfg_##inst, POST_KERNEL,              \
			      CONFIG_DISPLAY_INIT_PRIORITY, &ri_mipi_display_api);

DT_INST_FOREACH_STATUS_OKAY(RI_MIPI_DISP_DEFINE)
