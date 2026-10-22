/*
 * Copyright (c) 2026 Racyics GmbH.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

 /* Includes */

#include <errno.h>
#include <soc.h>
#include <math.h>
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

// TODELETE
#define CFG_FPGA 
#define CFG_SIL
//#define DSI_MANUAL_MODE
// TODELETE

struct ri_mipi_cfg {
    uint32_t               id;
    uint32_t               x_res;
    uint32_t               y_res;
    const struct device    *i2c_bus;
    ri_mipi_disp_type_t            type;
    ri_mipi_disp_side_parameters_t vertical;
    ri_mipi_disp_side_parameters_t horizontal;
    uint8_t                        nr_of_lanes;
};

struct ri_mipi_data {
    uint8_t                pixel_size;
    uint8_t                pixel_format;
};

/* Helper Declarations */

static int mipi_disp_config(const struct device *dev);
static uint16_t blanking_total(const ri_mipi_disp_side_parameters_t* p_display_cfg);
static uint32_t doubleToHex(double value);

/* Helper Functions */

uint32_t doubleToHex(double value) {

    uint16_t high_bytes = floor(value);

    uint16_t low_bytes = (1 << 16) * (value - floor(value));
    LOG_DBG(" \t\t Value to convert: %.3f", value);
    LOG_DBG("\t\t\t ->hex value: 0x%08X\n", (high_bytes << 16) | (low_bytes & 0xFFFF));
    return (high_bytes << 16) | (low_bytes & 0xFFFF);
}


uint16_t blanking_total(const ri_mipi_disp_side_parameters_t* p_display_cfg)
{
  LOG_DBG("\tFront Porch: %d", p_display_cfg->front_porch);
  LOG_DBG("\tSync Pulse: %d", p_display_cfg->sync_pulse);
  LOG_DBG("\tBack Porch: %d",p_display_cfg->back_porch);
  return (p_display_cfg->front_porch + p_display_cfg->sync_pulse + p_display_cfg->back_porch);
}


int mipi_disp_config(const struct device *dev)
{
    const struct ri_mipi_cfg *cfg = dev->config;
    
    uint32_t reg_value;

    reg_value = READREG(DCU_GLOBAL_HW_VERSION_REG);
    if (reg_value == 0x0) {
      LOG_INF("HW VERSION READ Failed 0x%08X\n", reg_value);
      return -EINVAL;
    }

    /* Check Config Sanity */
    if (    (cfg->type == RI_MIPI_DISP_TYPE_GENERIC || cfg->type == RI_MIPI_DISP_TYPE_TFT_F_AND_S)
            && (cfg->vertical.active_lines != 0)
            && (cfg->horizontal.active_lines != 0)
            && (cfg->nr_of_lanes != 0) ) {
        LOG_DBG("Basics configs are safe");
    }
    else {
        LOG_ERR("Invalid Configs");
        return -EINVAL;
    }

    LOG_DBG("-------------- Window Parameters ------------\n");
    LOG_DBG("Horizontal active width:      %d\n", cfg->horizontal.active_lines);
    LOG_DBG("Horizontal sync pulse width:  %d\n", cfg->horizontal.sync_pulse);
    LOG_DBG("Horizontal back_porch width:  %d\n", cfg->horizontal.back_porch);
    LOG_DBG("Horizontal front_porch width: %d\n", cfg->horizontal.front_porch);
    LOG_DBG("DCU_AXIBUSWORDS_BYTES:        %d\n", DCU_AXIBUSWORDS_BYTES);
    LOG_DBG("-------------Vertical Parameters ------------\n");
    LOG_DBG("Vertical active height:      %d\n", cfg->vertical.active_lines);
    LOG_DBG("Vertical sync pulse height:  %d\n", cfg->vertical.sync_pulse);
    LOG_DBG("Vertical back_porch height:  %d\n", cfg->vertical.back_porch);
    LOG_DBG("Vertical front_porch height: %d\n", cfg->vertical.front_porch);

    
    LOG_DBG("Horizontal Parameters:");
    uint16_t horizontal_blanking_total = blanking_total(&cfg->horizontal);
    uint16_t total_pixel_per_line = horizontal_blanking_total + cfg->horizontal.active_lines;
    LOG_DBG("Total Pixel Per Line: %d\n", total_pixel_per_line);
    LOG_DBG("Vertical Parameters:");
    uint16_t vertical_blanking_total = blanking_total(&cfg->vertical);
    uint16_t total_lines_per_frame = vertical_blanking_total + cfg->vertical.active_lines;
    LOG_DBG("Total Pixel Per Frame: %d\n", total_lines_per_frame);

    float min_refresh_rate_hz = 51.3;
    float min_pixel_clk_mhz = total_pixel_per_line * total_lines_per_frame * min_refresh_rate_hz;
    min_pixel_clk_mhz /= 1000000;

    
    #ifndef CFG_FPGA
    float sel_pixel_clk_mhz = round((min_pixel_clk_mhz * 1.00164) * 10000) / 10000 ;
    #else
    float sel_pixel_clk_mhz =  72.2222; // Xilinx DPHY IP config
    #endif

    if (sel_pixel_clk_mhz<min_pixel_clk_mhz) {
      LOG_ERR("Selected pixel clock is too small\n");
      return -EINVAL;
    }

    float dsi_byte_clk = sel_pixel_clk_mhz * 3 / cfg->nr_of_lanes;
    float dphy_lane_speed = dsi_byte_clk * 8;
    float dphy_lane_speed_burst = dphy_lane_speed * 1.1; // 10 % Margin for burst mode
    float selected_bitclk = floor(dphy_lane_speed_burst);
    float selected_byteclk = selected_bitclk / 8;
    float sel_refresh_rate_hz = (sel_pixel_clk_mhz*1000.0*1000.0/((float)(total_pixel_per_line * total_lines_per_frame)));
    LOG_DBG("------------------ Display Clock config -------------------------\n");
    LOG_DBG("Minimum calculated min_pixel_clk_mhz: %.4f", min_pixel_clk_mhz);
    LOG_DBG("Selected sel_pixel_clk_mhz: %.4f", sel_pixel_clk_mhz);
    LOG_DBG("Framerate: %.4f", sel_refresh_rate_hz);
    LOG_DBG("Minimum DSI HSByteClk: %.3f MHz || Minimum DPHY Bitrate: %.1f MBit/s || Minimum DPHY Bitrate for Burst mode: %.1f MBit/s", dsi_byte_clk, dphy_lane_speed, dphy_lane_speed_burst);
    LOG_DBG("Selected DPHY Bitrate: %.2f Mbit/s || Selected DPHY HSByteClk: %.1f MBit/s", selected_bitclk, selected_byteclk);
    
    
    // ----------------------------------------
    // MIPI DSI host controller configuration
    // ----------------------------------------
    LOG_DBG("------------------ MIPI DSI Host Config -----------------\n");
    reg_value = READREG(DSI_DISP_CORE_ID);
    if (reg_value != 0x30313130) {
      LOG_ERR("CORE ID READ Failed 0x%08Xn", reg_value);
      return -EINVAL;
    }
    reg_value = READREG(DSI_DISP_VERSION);
    if (reg_value != 0X1A000110) {
      LOG_ERR("DISP VERSION READ Failed 0x%08Xn", reg_value);
      return -EINVAL;
    }
    
    LOG_DBG("Core ID and Version succesfully read");

    WRITEREG_LOG("DSI_DISP_PHY_MODE_CFG         ", DSI_DISP_PHY_MODE_CFG,      0x00000030);

    #ifndef CFG_FPGA
    WRITEREG_LOG("DSI_DISP_PHY_CLK_CFG          ", DSI_DISP_PHY_CLK_CFG, 0x00000701);
    #else 
    WRITEREG_LOG("DSI_DISP_PHY_CLK_CFG          ", DSI_DISP_PHY_CLK_CFG, 0x00000301);
    #endif

    WRITEREG_LOG("DSI_DISP_DSI_GENERAL_CFG      ", DSI_DISP_DSI_GENERAL_CFG,   0x00000001);

    WRITEREG_LOG("DSI_DISP_DSI_VID_TX_CFG       ", DSI_DISP_DSI_VID_TX_CFG,    0x00100002); // LPDT_DISPLAY_CMD_EN (commands only in LP mode) , VID_MODE_TYPE=Burst mode
    #ifndef CFG_SIL
    WRITEREG_LOG("DSI_DISP_DSI_DS_TX_CFG        ", DSI_DISP_DSI_DS_TX_CFG,     0x00000001);                  //DS_HS_LINK_EN = 1 see Email from Synopsis
    #endif
    WRITEREG_LOG("DSI_DISP_IPI_COLOR_MAN_CFG    ", DSI_DISP_IPI_COLOR_MAN_CFG, 0x00000050); // 8 bits
    
    #ifdef DSI_MANUAL_MODE
      WRITEREG_LOG("DSI_DISP_MANUAL_MODE_CFG      ", DSI_DISP_MANUAL_MODE_CFG, 0x00000001);
      // IPI Config Select Automatic Mode as reset
      // IRQ Mask Config ?
      // double_value = 0.823846407;
      double ipi_ratio = selected_byteclk / sel_pixel_clk_mhz;
      WRITEREG_LOG("DSI_DISP_PHY_IPI_RATIO_MAN_CFG", DSI_DISP_PHY_IPI_RATIO_MAN_CFG, doubleToHex(ipi_ratio));
      WRITEREG_LOG("DSI_DISP_PHY_SYS_RATIO_MAN_CFG", DSI_DISP_PHY_SYS_RATIO_MAN_CFG, doubleToHex(ipi_ratio));

      // double_value = 105.4523401;
      double hsa_man = (cfg->horizontal.sync_pulse / sel_pixel_clk_mhz) * selected_byteclk;
      WRITEREG_LOG("DSI_DISP_IPI_VID_HSA_MAN_CFG", DSI_DISP_IPI_VID_HSA_MAN_CFG,   doubleToHex(hsa_man));
      // double_value = 59.3169413;
      double hbp_man = (cfg->horizontal.back_porch / sel_pixel_clk_mhz) * selected_byteclk;
      WRITEREG_LOG("DSI_DISP_IPI_VID_HBP_MAN_CFG", DSI_DISP_IPI_VID_HBP_MAN_CFG,   doubleToHex(hbp_man));

      // double_value = 659.077125870;
      double hact_man = (cfg->horizontal.active_lines / sel_pixel_clk_mhz) * selected_byteclk;
      WRITEREG_LOG("DSI_DISP_IPI_VID_HACT_MAN_CFG", DSI_DISP_IPI_VID_HACT_MAN_CFG,  doubleToHex(hact_man));

      // double_value = 869.98180615;
      double hline_man = (total_pixel_per_line / sel_pixel_clk_mhz) * selected_byteclk;
      WRITEREG_LOG("DSI_DISP_IPI_VID_HLINE_MAN_CFG", DSI_DISP_IPI_VID_HLINE_MAN_CFG, doubleToHex(hline_man));
      WRITEREG_LOG("DSI_DISP_IPI_VID_VSA_MAN_CFG  ", DSI_DISP_IPI_VID_VSA_MAN_CFG,   0x0000000A);
      WRITEREG_LOG("DSI_DISP_IPI_VID_VBP_MAN_CFG  ", DSI_DISP_IPI_VID_VBP_MAN_CFG,   0x00000026);
      WRITEREG_LOG("DSI_DISP_IPI_VID_VACT_MAN_CFG ", DSI_DISP_IPI_VID_VACT_MAN_CFG,  0x00000500);
      WRITEREG_LOG("DSI_DISP_IPI_VID_VFP_MAN_CFG  ", DSI_DISP_IPI_VID_VFP_MAN_CFG,   0x00000003);
      WRITEREG_LOG("DSI_DISP_PWR_UP               ", DSI_DISP_PWR_UP,                0x00000001);
      #ifdef CFG_FPGA
        WRITEREG_LOG("DSI_DISP_MODE_CTRL            ", DSI_DISP_MODE_CTRL,             0x00000003); // Video mode
      #endif
    #else
    WRITEREG_LOG("DSI_DISP_MANUAL_MODE_CFG      ", DSI_DISP_MANUAL_MODE_CFG,       0x00000000);
    WRITEREG_LOG("DSI_DISP_PWR_UP               ", DSI_DISP_PWR_UP,                0x00000001);
    WRITEREG_LOG("DSI_DISP_MODE_CTRL            ", DSI_DISP_MODE_CTRL,             0x00000001); // AutoCalculation mode
    #endif
    
    // ------------------------
    // TES CDC configuration
    // ------------------------
    uint32_t x, y;

    // configure global canvas resolution
    LOG_DBG("-------------- TES CDC Canvas ------------\n");
    LOG_DBG("TES CDC: Horizontal canvas settings\n");
    LOG_DBG("Horizontal active width:      %d", cfg->horizontal.active_lines);
    LOG_DBG("Horizontal sync pulse width:  %d", cfg->horizontal.sync_pulse);
    LOG_DBG("Horizontal back_porch width:  %d", cfg->horizontal.back_porch);
    LOG_DBG("Horizontal front_porch width: %d\n", cfg->horizontal.front_porch);
    LOG_DBG("TES CDC: Vertical canvas settings\n");
    LOG_DBG("Vertical active height:      %d", cfg->vertical.active_lines);
    LOG_DBG("Vertical sync pulse height:  %d", cfg->vertical.sync_pulse);
    LOG_DBG("Vertical back_porch height:  %d", cfg->vertical.back_porch);
    LOG_DBG("Vertical front_porch height: %d", cfg->vertical.front_porch);
    LOG_DBG("-----------------------------------------\n");
    x = cfg->horizontal.sync_pulse;
    y = cfg->vertical.sync_pulse;
    WRITEREG_LOG("_SYNC_SIZE                ", DCU_GLOBAL_SYNC_SIZE_REG,                                 ((x - 1) << 16) | (y - 1));
    x += cfg->horizontal.back_porch;
    y += cfg->vertical.back_porch;
    WRITEREG_LOG("_BACK_PORCH__ACCUMULATED_ ", DCU_GLOBAL_BACK_PORCH__ACCUMULATED__REG,   ((x - 1) << 16) | (y - 1));
    x += cfg->horizontal.active_lines;
    y += cfg->vertical.active_lines;
    WRITEREG_LOG("_ACTIVE_WIDTH             ", DCU_GLOBAL_ACTIVE_WIDTH_REG,                           ((x - 1) << 16) | (y - 1));  // screen size: 800 x 1280
    x += cfg->horizontal.front_porch;
    y += cfg->vertical.front_porch;
    WRITEREG_LOG("_TOTAL_WIDTH              ", DCU_GLOBAL_TOTAL_WIDTH_REG,                             ((x - 1) << 16) | (y - 1));  // full ri_mipi_disp_cfg_g
    
    
    return 0;
}

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

    err = mipi_disp_config(dev);
    if ( 0 != err ) return err;

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
        .i2c_bus = DEVICE_DT_GET(DT_INST_PHANDLE(inst, i2c_bus)),       \
        .type = RI_MIPI_DISP_TYPE_TFT_F_AND_S,                          \
        .vertical = {                                                   \
            .active_lines      = RI_MIPI_DISP_ACTIVE_HEIGTH_TFT_F_AND_S,  \
            .front_porch       = 3,                                       \
            .sync_pulse        = 10,                                      \
            .back_porch        = 38,                                      \
        },                                                                \
        .horizontal = {                                                   \
            .active_lines    = RI_MIPI_DISP_ACTIVE_WIDTH_TFT_F_AND_S,     \
            .front_porch     = 56,                                        \
            .sync_pulse      = 128,                                       \
            .back_porch      = 72,                                        \
        },                                                                \
        .nr_of_lanes = 4,                                                 \
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