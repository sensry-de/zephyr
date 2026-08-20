/*
 * Copyright (c) 2026 Racyics GmbH.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Includes*/

#include <errno.h>
#include <soc.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/display.h>
#include <zephyr/sys/util.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/linker/devicetree_regions.h>

#include "display_racyics_qspi.h"

LOG_MODULE_REGISTER(display_racyics_qspi, CONFIG_DISPLAY_LOG_LEVEL);

/* Defines*/ 
#define DT_DRV_COMPAT racyics_qspi_display
#define BYTES_PER_PIXEL 4
#define QSPI_DISP_MAX_BRIGHTNESS 0x7F

#if CONFIG_DISPLAY_RACYICS_QSPI_BUFFER
#define DISPLAY_BUFFER_LINKER_SECTION \
	Z_GENERIC_SECTION(LINKER_DT_NODE_REGION_NAME(DT_NODELABEL(data_sram1)))
#else
#define DISPLAY_BUFFER_LINKER_SECTION
#endif

#define RI_QSPI_DISPLAY_PIXEL_SIZE(inst) ( DISPLAY_BITS_PER_PIXEL(DT_INST_PROP(inst, pixel_format)) / BITS_PER_BYTE )

#define ri_wait_ms k_msleep
#define ri_wait_us k_usleep


/* Internal Variables*/

struct ri_qspi_cfg {
    uint32_t               id;              //!< Display instance ID
    mm_reg_t               base_addr;       //!< Instance base address
    ri_qspi_dev_t          qspi_dev;        //!< the QSPI interface device setting
    uint32_t               x_res;           //!< horizontal resolution of the display
    uint32_t               y_res;           //!< vertical   resolution of the display
};

struct ri_qspi_data {
    uint8_t                pixel_size;
    uint8_t                pixel_format;
    uint8_t*               frame_buf_p;             //!< Frame buffer
    uint32_t               frame_buf_size8;         //!< size of the frame buffer
    ri_qspi_display_cmd_t  display_command;
    ri_qspi_job_t          spi_job;
};

/* Helper declarations */
static void display_reset(void);
static void poll_qspi_display_ctrl_status_idle(void);
static void config_RDMA(const struct device *dev);
static void config_spi_m(const struct device *dev);
static int config_pixel_format(const struct device *dev);
static void display_init(const struct device *dev);
static void exec_spi_job(const struct device *dev);
static void exec_display_command(const struct device *dev);
static void switch_qspi_mode(const struct device *dev);
static void switch_spi_mode(const struct device *dev);
static void display_image_write(const struct device *dev);
static int  compile_qspi_display_frame(  const struct device *dev,
                                        const uint16_t x, const uint16_t y,
                                        const struct display_buffer_descriptor *desc,
                                        const void *buf);

static int  copy_image_line_3bpp_optimized(uint8_t* dst_row, const uint8_t* line_buffer, uint32_t copy_width, uint32_t img_px_size);
static int  copy_image_line_3bpp_to_4bpp_optimized(uint8_t* dst_row, const uint8_t* line_buffer, uint32_t copy_width);

/* Helper functions */

int compile_qspi_display_frame( const struct device *dev,
                                const uint16_t x, const uint16_t y,
                                const struct display_buffer_descriptor *desc,
                                const void *buf)
{

    const struct ri_qspi_cfg *cfg = dev->config;
    struct ri_qspi_data *data = dev->data;

    uint32_t image_width  = desc->width;
    uint32_t image_height = desc->height;
    uint32_t x_pos        = (uint32_t) x;
    uint32_t y_pos        = (uint32_t) y;
    uint32_t img_px_size  = data->pixel_size;
    uint32_t fb_px_size   = data->pixel_size;
    uint32_t framebuffer_width   = cfg->x_res;
    uint32_t framebuffer_height  = cfg->y_res;

    // Clipping
    uint32_t copy_width  = (x_pos + image_width  > framebuffer_width)  ? (framebuffer_width  - x_pos) : image_width;
    uint32_t copy_height = (y_pos + image_height > framebuffer_height) ? (framebuffer_height - y_pos) : image_height;

    const uint8_t* image_buffer = buf;
    uint8_t* frame_buffer = data->frame_buf_p;

    // copy image line by line
    //  - read line and copy to local line buffer
    //  - copy line buffer optimized, 32-bit aligned into frame buffer
    for (uint32_t y_i = 0; y_i < copy_height; ++y_i) {
        const uint8_t* src_row = image_buffer + y_i * image_width * img_px_size;
        uint8_t line_buffer[copy_width * img_px_size];

        // read image line
        memcpy(line_buffer, src_row, copy_width * img_px_size);

        // write line to frame buffer
        // different algorithmns, depending of pixel size of frame buffer and image
        uint8_t* dst_row = frame_buffer + ((y_i + y_pos) * framebuffer_width + x_pos) * fb_px_size;

        if (fb_px_size == 3) {
            copy_image_line_3bpp_optimized(dst_row, line_buffer, copy_width, img_px_size);
        }
        else if (fb_px_size == 4) {
           if (img_px_size == 4) {
               // direct compatible → memcpy
               memcpy(dst_row, line_buffer, copy_width * 4);
           } else if (img_px_size == 3) {
              copy_image_line_3bpp_to_4bpp_optimized(dst_row, line_buffer, copy_width);
          }
        }
    }
    return 0;
}

int copy_image_line_3bpp_optimized(
    uint8_t*        dst_row,
    const uint8_t*  line_buffer,
    uint32_t        copy_width,
    uint32_t        img_px_size)
{
    uintptr_t dst_addr = (uintptr_t)dst_row;
    uint32_t i_dst = 0;
    uint32_t i_src = 0;
    const uint32_t total_rgb_bytes = copy_width * 3;

    // Byteweise bis zur 32-Bit-Grenze
    while (((dst_addr + i_dst) % 4 != 0) && (i_dst < total_rgb_bytes)) {
        const uint8_t* s = line_buffer + i_src;
        dst_row[i_dst++] = s[0];
        dst_row[i_dst++] = s[1];
        dst_row[i_dst++] = s[2];
        i_src += img_px_size;
    }

    // optimized in blocks of 12 bytes (4 pixels)
    while ((i_dst + 12) <= total_rgb_bytes) {
        uint8_t* d = dst_row + i_dst;

        const uint8_t* s0 = line_buffer + i_src + 0 * img_px_size;
        const uint8_t* s1 = line_buffer + i_src + 1 * img_px_size;
        const uint8_t* s2 = line_buffer + i_src + 2 * img_px_size;
        const uint8_t* s3 = line_buffer + i_src + 3 * img_px_size;

        ((uint32_t*)d)[0] =
            ((uint32_t)s0[0])       |
            ((uint32_t)s0[1] << 8)  |
            ((uint32_t)s0[2] << 16) |
            ((uint32_t)s1[0] << 24);

        ((uint32_t*)d)[1] =
            ((uint32_t)s1[1])       |
            ((uint32_t)s1[2] << 8)  |
            ((uint32_t)s2[0] << 16) |
            ((uint32_t)s2[1] << 24);

        ((uint32_t*)d)[2] =
            ((uint32_t)s2[2])       |
            ((uint32_t)s3[0] << 8)  |
            ((uint32_t)s3[1] << 16) |
            ((uint32_t)s3[2] << 24);

        i_dst += 12;
        i_src += 4 * img_px_size;
    }

    // Rest byteweise
    while (i_dst < total_rgb_bytes) {
        const uint8_t* s = line_buffer + i_src;
        dst_row[i_dst++] = s[0];
        dst_row[i_dst++] = s[1];
        dst_row[i_dst++] = s[2];
        i_src += img_px_size;
    }

    return 0;
}

int copy_image_line_3bpp_to_4bpp_optimized(
    uint8_t* dst_row,
    const uint8_t* line_buffer,
    uint32_t copy_width)
{
    for (uint32_t x = 0; x < copy_width; ++x) {
        const uint8_t* src_px = line_buffer + x * 3;
        uint32_t* dst_px = (uint32_t*)(dst_row + x * 4);

        // write 32 bit: RGB + (Alpha)
        *dst_px = ((uint32_t)src_px[0])       |  // R
                  ((uint32_t)src_px[1] << 8)  |  // G
                  ((uint32_t)src_px[2] << 16);   // B
                  // byte 3 is unchanged / unused
    }
    return 0;
}

void config_spi_m(const struct device *dev)
{
    const struct ri_qspi_cfg *cfg = dev->config;

    LOG_DBG("Writing SPI master configuration");

    // poll_spi_master_status_idle();
    MODIFYREG(RI_QSPI_DISPLAY_QSPI_MASTER_CFG_ITF_REG      ,  RI_QSPI_MASTER_REGFILE__CFG_ITF__SPI_MODE__FIELD       ,              RI_QSPI_MASTER_REGFILE__CFG_ITF__SPI_MODE_SNGL_SPI__E); // begin with single SPI
    MODIFYREG(RI_QSPI_DISPLAY_QSPI_MASTER_CFG_ITF_REG      ,  RI_QSPI_MASTER_REGFILE__CFG_ITF__CPHA__FIELD           , (cfg->qspi_dev.cpha << RI_QSPI_MASTER_REGFILE__CFG_ITF__CPHA__SHIFT));
    MODIFYREG(RI_QSPI_DISPLAY_QSPI_MASTER_CFG_ITF_REG      ,  RI_QSPI_MASTER_REGFILE__CFG_ITF__CPOL__FIELD           , (cfg->qspi_dev.cpol << RI_QSPI_MASTER_REGFILE__CFG_ITF__CPOL__SHIFT));
    MODIFYREG(RI_QSPI_DISPLAY_QSPI_MASTER_CFG_ITF_REG      ,  RI_QSPI_MASTER_REGFILE__CFG_ITF__MISO_SEL__FIELD       ,              RI_QSPI_MASTER_REGFILE__CFG_ITF__MISO_SEL_MISO_0__E);// the display drives on bit 0, only in the resprective read response phase, the normal SPI is e wire

    MODIFYREG(RI_QSPI_DISPLAY_QSPI_MASTER_CFG_SCLK_REG     ,  RI_QSPI_MASTER_REGFILE__CFG_SCLK__CDIV__FIELD          , (cfg->qspi_dev.cdiv << RI_QSPI_MASTER_REGFILE__CFG_SCLK__CDIV__SHIFT));
    MODIFYREG(RI_QSPI_DISPLAY_QSPI_MASTER_CFG_SCLK_REG     ,  RI_QSPI_MASTER_REGFILE__CFG_SCLK__SMPL_POINT__FIELD    , (cfg->qspi_dev.cdiv << RI_QSPI_MASTER_REGFILE__CFG_SCLK__SMPL_POINT__SHIFT)); // sample at rising edge

    MODIFYREG(RI_QSPI_DISPLAY_QSPI_MASTER_CFG_SS_TIMING_REG,  RI_QSPI_MASTER_REGFILE__CFG_SS_TIMING__SETUP__FIELD    , (cfg->qspi_dev.css << RI_QSPI_MASTER_REGFILE__CFG_SS_TIMING__SETUP__SHIFT));
    MODIFYREG(RI_QSPI_DISPLAY_QSPI_MASTER_CFG_SS_TIMING_REG,  RI_QSPI_MASTER_REGFILE__CFG_SS_TIMING__HOLD__FIELD     , (cfg->qspi_dev.csh << RI_QSPI_MASTER_REGFILE__CFG_SS_TIMING__HOLD__SHIFT));
    MODIFYREG(RI_QSPI_DISPLAY_QSPI_MASTER_CFG_SS_TIMING_REG,  RI_QSPI_MASTER_REGFILE__CFG_SS_TIMING__INACTIVE__FIELD , (cfg->qspi_dev.csi << RI_QSPI_MASTER_REGFILE__CFG_SS_TIMING__INACTIVE__SHIFT));
}

int config_pixel_format (const struct device *dev)
{
    const struct ri_qspi_cfg *cfg = dev->config;
    struct ri_qspi_data *data = dev->data;

    // sanity check
    uint32_t min_buf_size = (cfg->x_res * cfg->y_res * data->pixel_size);
    if (data->frame_buf_size8 < min_buf_size) {
        LOG_ERR("Frame buffer size (%d) is under the minimum (%d). Please reserve more space", 
                data->frame_buf_size8, min_buf_size);
        return 1;
    }

    LOG_DBG("configure pixel format");
    // source pixel format & display pixel format (sink)
    switch (data->pixel_size) {
      case PIXEL_SIZE08:
        MODIFYREG(RI_QSPI_DISPLAY_DISP_CTRL_CFG_REG, RI_QSPI_DISP_REGFILE__CFG__PX_DWIDTH_SRC__FIELD, (RI_QSPI_DISP_REGFILE__CFG__PX_DWIDTH_SRC_PX_8_BIT__E));
        MODIFYREG(RI_QSPI_DISPLAY_DISP_CTRL_CFG_REG, RI_QSPI_DISP_REGFILE__CFG__PX_DWIDTH_SNK__FIELD, (RI_QSPI_DISP_REGFILE__CFG__PX_DWIDTH_SNK_PX_8_BIT__E));
        break;
      case PIXEL_SIZE16:
        MODIFYREG(RI_QSPI_DISPLAY_DISP_CTRL_CFG_REG, RI_QSPI_DISP_REGFILE__CFG__PX_DWIDTH_SRC__FIELD, (RI_QSPI_DISP_REGFILE__CFG__PX_DWIDTH_SRC_PX_16_BIT__E));
        MODIFYREG(RI_QSPI_DISPLAY_DISP_CTRL_CFG_REG, RI_QSPI_DISP_REGFILE__CFG__PX_DWIDTH_SNK__FIELD, (RI_QSPI_DISP_REGFILE__CFG__PX_DWIDTH_SNK_PX_16_BIT__E));
        break;
      case PIXEL_SIZE24:
        MODIFYREG(RI_QSPI_DISPLAY_DISP_CTRL_CFG_REG, RI_QSPI_DISP_REGFILE__CFG__PX_DWIDTH_SRC__FIELD, (RI_QSPI_DISP_REGFILE__CFG__PX_DWIDTH_SRC_PX_24_BIT__E));
        MODIFYREG(RI_QSPI_DISPLAY_DISP_CTRL_CFG_REG, RI_QSPI_DISP_REGFILE__CFG__PX_DWIDTH_SNK__FIELD, (RI_QSPI_DISP_REGFILE__CFG__PX_DWIDTH_SNK_PX_24_BIT__E));
        break;
      case PIXEL_SIZE32:
        MODIFYREG(RI_QSPI_DISPLAY_DISP_CTRL_CFG_REG, RI_QSPI_DISP_REGFILE__CFG__PX_DWIDTH_SRC__FIELD, (RI_QSPI_DISP_REGFILE__CFG__PX_DWIDTH_SRC_PX_32_BIT__E));
        MODIFYREG(RI_QSPI_DISPLAY_DISP_CTRL_CFG_REG, RI_QSPI_DISP_REGFILE__CFG__PX_DWIDTH_SNK__FIELD, (RI_QSPI_DISP_REGFILE__CFG__PX_DWIDTH_SNK_PX_32_BIT__E));
        break;
      default:
        LOG_ERR("Unsupported pixel size. Supported pixel sizes are {8,16,24,32} bit.");
        break;
    }
    
    MODIFYREG(RI_QSPI_DISPLAY_DISP_CTRL_CFG_REG,  RI_QSPI_DISP_REGFILE__CFG__PX_CONVERT_SELECT__FIELD, (0x0 << RI_QSPI_DISP_REGFILE__CFG__PX_CONVERT_SELECT__SHIFT));

    return 0;
}

void switch_qspi_mode(const struct device *dev)
{
    struct ri_qspi_data *data = dev->data;
    ri_qspi_job_t* job_p = &data->spi_job;
    
    job_p->px_bytes_num    = 0;
    job_p->wr_len          = 1;
    job_p->rd_len          = 0;
    job_p->wr_data_byte_0  = 0x38; // enable QSPI (Display)
    job_p->valid           = false;

    exec_spi_job(dev);

    ri_wait_us(10); // FIXME (without the PRINT_D's around this line)

    LOG_DBG("\nconfigure SPI master to QSPI mode");
    MODIFYREG(RI_QSPI_DISPLAY_QSPI_MASTER_CFG_ITF_REG, RI_QSPI_MASTER_REGFILE__CFG_ITF__SPI_MODE__FIELD, RI_QSPI_MASTER_REGFILE__CFG_ITF__SPI_MODE_QUAD_SPI__E);    // switch to quad SPI mode
}

void switch_spi_mode(const struct device *dev)
{
    struct ri_qspi_data *data = dev->data;
    ri_qspi_job_t* job_p = &data->spi_job;
    
    job_p->px_bytes_num    = 0;
    job_p->wr_len          = 1;
    job_p->rd_len          = 0;
    job_p->wr_data_byte_0  = 0xFF; // enable SPI (Display)
    job_p->valid           = false;

    exec_spi_job(dev);

    LOG_DBG("\nconfigure SPI master to SPI mode");
    MODIFYREG(RI_QSPI_DISPLAY_QSPI_MASTER_CFG_ITF_REG, RI_QSPI_MASTER_REGFILE__CFG_ITF__SPI_MODE__FIELD, RI_QSPI_MASTER_REGFILE__CFG_ITF__SPI_MODE_SNGL_SPI__E);    // switch to normal SPI mode
}

void poll_qspi_display_ctrl_status_idle () 
{
    uint32_t reg_val;
    while(1) {
        reg_val = READREG(RI_QSPI_DISPLAY_DISP_CTRL_STATUS_REG);
        LOG_DBG("\nRead RI_QSPI_DISPLAY_DISP_CTRL_STATUS_REG = %X", reg_val);
        if (reg_val == RI_QSPI_DISP_REGFILE__STATUS__STATE_IDLE__E) {
            LOG_DBG("\nQSPI display control is IDLE -> continue");
            break;
        } else {
            LOG_DBG("\nQSPI display control is BUSY");
            ;
        }
    }
}

void config_RDMA (const struct device *dev)
{
    const struct ri_qspi_cfg *cfg = dev->config;
    struct ri_qspi_data *data = dev->data;

    uint32_t  frame_size_memory    = cfg->x_res * cfg->y_res * data->pixel_size;
    uint32_t  buf_addr = (uint32_t)data->frame_buf_p;

    LOG_DBG("\nconfigure RDMA");
    LOG_DBG("DMA address  :%X", buf_addr);
    LOG_DBG("DMA num bytes:%d", frame_size_memory);
    WRITEREG(RI_QSPI_DISPLAY_RDMA_RI_RDMA_REG_STARTADDR_REG, buf_addr);
    WRITEREG(RI_QSPI_DISPLAY_RDMA_RI_RDMA_REG_MAXSIZE_REG, frame_size_memory);
    WRITEREG(RI_QSPI_DISPLAY_RDMA_RI_RDMA_REG_IRQ_STATUS_ENABLE_REG, 0x0);
    WRITEREG(RI_QSPI_DISPLAY_RDMA_RI_RDMA_REG_CTRL_REG, 0x1);
    }

void display_reset()
{
    MODIFYREG(RBB_FPGA_TOP_OUT_0_REG, (0x1 << 31), (0x1 << 31));
    LOG_DBG(" -    LED_YELLOW2_LED (fpga board) SHOULD BE ON");
    MODIFYREG(RBB_FPGA_TOP_OUT_1_REG, (0x1 << (RBB_REG__FPGA_TOP_OUT_1__OTHER_OUTS_0__SHIFT + 0)), (0x1 << (RBB_REG__FPGA_TOP_OUT_1__OTHER_OUTS_0__SHIFT + 0)));
    LOG_DBG(" -    MIPI V2 PCB relevant,QSPI_33_EN -> 1 (voltag level translator activated)");
    ri_wait_ms(100);
    // QSPI display reset:
    LOG_DBG("reset QSPI display");
    MODIFYREG(RBB_FPGA_TOP_OUT_0_REG, (0x1 << 18), (0x0 << 18));
    
    ri_wait_ms(200);    // reset pulse 100ms
    
    MODIFYREG(RBB_FPGA_TOP_OUT_0_REG, (0x1 << 19), (0x1 << 19));
    LOG_DBG(" -    QSPI DISP VCI SHOULD BE ON");

    ri_wait_ms(200);    // reset pulse 100ms

    LOG_DBG("deassertsource_interna reset QSPI display");
    MODIFYREG(RBB_FPGA_TOP_OUT_0_REG, (0x1 << 18), (0x1 << 18));
    
    ri_wait_ms(200);     // data sheet:PowerOnSequence: T3(RESET high to first command) min 10ms
    
    MODIFYREG(RBB_FPGA_TOP_OUT_0_REG, (0x1 << 24), (0x1 << 24)); 
    LOG_DBG(" -    MIPI CLK200 switched ON, LED SHOULD BE OFF");

}

void exec_display_command(const struct device *dev)
{
    struct ri_qspi_data *data = dev->data;
    ri_qspi_display_cmd_t*  cmd_p = &data->display_command;
    ri_qspi_job_t*          job_p = &data->spi_job;

    job_p->px_bytes_num    = 0;
    job_p->wr_len          = cmd_p->read_en ? 4 : cmd_p->len + 4;
    job_p->rd_len          = cmd_p->read_en ? cmd_p->len : 0;
    job_p->wr_data_byte_0  = cmd_p->read_en ? 0x03 : 0x02;
    job_p->wr_data_byte_1  = 0x00;
    job_p->wr_data_byte_2  = cmd_p->address;
    job_p->wr_data_byte_3  = 0x00;
    job_p->wr_data_byte_4  = cmd_p->wr_data_byte_0;
    job_p->wr_data_byte_5  = cmd_p->wr_data_byte_1;
    job_p->wr_data_byte_6  = cmd_p->wr_data_byte_2;
    job_p->wr_data_byte_7  = cmd_p->wr_data_byte_3;
    job_p->valid           = false;

    exec_spi_job(dev);
}

void display_init(const struct device *dev)
{
    struct ri_qspi_data *data = dev->data;

    ri_qspi_display_cmd_t* cmd_p = &data->display_command;
    
    switch_spi_mode(dev);
    switch_qspi_mode(dev);

    cmd_p->read_en         = 0; // write
    cmd_p->len             = 1;

    cmd_p->address         = 0xFE; // UCS CMD1 (default page after power-on )
    cmd_p->wr_data_byte_0  = 0x00;
    exec_display_command(dev);

    cmd_p->address         = 0xC4; // set SPI mode
    cmd_p->wr_data_byte_0  = 0x80; // SPI write SRAM enable
    exec_display_command(dev);

    cmd_p->address         = 0x3A; // COLMOD: Interface Pixel Format
    cmd_p->wr_data_byte_0  = (BYTES_PER_PIXEL == 2) ? 0x55 : 0x77; // 16bit/pixel : 24bit/pixel
    exec_display_command(dev);

    cmd_p->address         = 0x35; // TEON: Tearing Effect Line ON
    // cmd_p->address         = 0x34; // TEOFF: Tearing Effect Line OFF
    cmd_p->wr_data_byte_0  = 0x00; //
    exec_display_command(dev);

    cmd_p->address         = 0x53; // WRCTRLD: Write CTRL Display
    cmd_p->wr_data_byte_0  = 0x20; // [5] Brighness Control Block ON; [3] Display dimming OFF
    exec_display_command(dev);

    cmd_p->address         = 0x51; // WRDISBV: Write Display Brightness
    cmd_p->wr_data_byte_0  = 0x00; // 0% initial (then fade in)
    exec_display_command(dev);

    cmd_p->address         = 0x63; // WRHBMDISBV: Write HBM Display Brightness
    cmd_p->wr_data_byte_0  = 0x7F; // 50% // MARKER: Test reg for readbacks
    exec_display_command(dev);

    cmd_p->len             = 4;

    cmd_p->address         = 0x2A; // CASET: Set Column Start Address
    cmd_p->wr_data_byte_0  = 0x00; // start column [9:8]
    cmd_p->wr_data_byte_1  = 0x06; // start column [7:0] - 6
    cmd_p->wr_data_byte_2  = 0x01; //   end column [9:8]
    cmd_p->wr_data_byte_3  = 0xD7; //   end column [7:0] - 471
    exec_display_command(dev);

    cmd_p->address         = 0x2B; // RASET: Set Row Start Address
    cmd_p->wr_data_byte_0  = 0x00; // start row [9:8]
    cmd_p->wr_data_byte_1  = 0x00; // start row [7:0] - 0
    cmd_p->wr_data_byte_2  = 0x01; //   end row [9:8]
    cmd_p->wr_data_byte_3  = 0xD1; //   end row [7:0] - 465
    exec_display_command(dev);

    cmd_p->len             = 0;

    cmd_p->address         = 0x11; // SLPOUT: Sleep Out
    exec_display_command(dev);

    ri_wait_ms(200); // min 120ms sleep out time, until next command is accepted

    cmd_p->address         = 0x29; // DISPON: Display On
    exec_display_command(dev);

#if 1
    // !!! ATTENTION !!! Dont ues this with the flatcable variant! Readback might cause damage, driver of display drives against level shifter

    switch_spi_mode(dev);

    // read back CASET and RASET (for debugging)
    cmd_p->read_en         = 1; // read
    cmd_p->len             = 4;

    cmd_p->address         = 0x2A; // CASET: Set Column Start Address
    exec_display_command(dev);
    uint32_t reg_val;
    reg_val = READREG(RI_QSPI_DISPLAY_DISP_CTRL_SPI_READ_DATA_REG);
    LOG_DBG("read CASET");
    LOG_DBG("start column = %6d"  , ( ( reg_val >> 16) & 0x0000FFFF));
    LOG_DBG(" end column = %6d",           ( reg_val        & 0x0000FFFF));

    cmd_p->address         = 0x2B; // RASET: Set Row Start Address
    exec_display_command(dev);
    reg_val = READREG(RI_QSPI_DISPLAY_DISP_CTRL_SPI_READ_DATA_REG);
    LOG_DBG("read RASET");
    LOG_DBG("start    row = %6d"  , ( ( reg_val >> 16) & 0x0000FFFF));
    LOG_DBG(        " end    row = %6d",           ( reg_val        & 0x0000FFFF));

    cmd_p->len             = 1;

    cmd_p->address         = 0xFE; // UCS CMD1 (default page after power-on )
    exec_display_command(dev);
    reg_val = 0xFF & READREG(RI_QSPI_DISPLAY_DISP_CTRL_SPI_READ_DATA_REG);
    LOG_DBG("read from 0x%6x = 0x%02x",cmd_p->address,reg_val);
    cmd_p->address         = 0xC4; // set SPI mode
    exec_display_command(dev);
    reg_val = 0xFF & READREG(RI_QSPI_DISPLAY_DISP_CTRL_SPI_READ_DATA_REG);
    LOG_DBG("read from 0x%6x = 0x%02x",cmd_p->address,reg_val);
    cmd_p->address         = 0x3A; // COLMOD: Interface Pixel Format
    exec_display_command(dev);
    reg_val = 0xFF & READREG(RI_QSPI_DISPLAY_DISP_CTRL_SPI_READ_DATA_REG);
    LOG_DBG("read from 0x%6x = 0x%02x",cmd_p->address,reg_val);
    cmd_p->address         = 0x35; // TEON: Tearing Effect Line ON
    exec_display_command(dev);
    reg_val = 0xFF & READREG(RI_QSPI_DISPLAY_DISP_CTRL_SPI_READ_DATA_REG);
    LOG_DBG("read from 0x%6x = 0x%02x",cmd_p->address,reg_val);
    cmd_p->address         = 0x53; // WRCTRLD: Write CTRL Display
    exec_display_command(dev);
    reg_val = 0xFF & READREG(RI_QSPI_DISPLAY_DISP_CTRL_SPI_READ_DATA_REG);
    LOG_DBG("read from 0x%6x = 0x%02x",cmd_p->address,reg_val);
    cmd_p->address         = 0x51; // WRDISBV: Write Display Brightness
    exec_display_command(dev);
    reg_val = 0xFF & READREG(RI_QSPI_DISPLAY_DISP_CTRL_SPI_READ_DATA_REG);
    LOG_DBG("read from 0x%6x = 0x%02x",cmd_p->address,reg_val);

    cmd_p->address         = 0x63; // WRHBMDISBV: Write HBM Display Brightness
    exec_display_command(dev);
    reg_val = 0xFF & READREG(RI_QSPI_DISPLAY_DISP_CTRL_SPI_READ_DATA_REG);
    LOG_DBG("read from 0x%6x = 0x%02x",cmd_p->address,reg_val);

    cmd_p->address         = 0x0A;
    exec_display_command(dev);
    reg_val = 0xFF & READREG(RI_QSPI_DISPLAY_DISP_CTRL_SPI_READ_DATA_REG);
    LOG_DBG("read from 0x%6x = 0x%02x",cmd_p->address,reg_val);

    cmd_p->address         = 0xc3;
    exec_display_command(dev);
    reg_val = 0xFF & READREG(RI_QSPI_DISPLAY_DISP_CTRL_SPI_READ_DATA_REG);
    LOG_DBG("read from 0x%6x = 0x%02x",cmd_p->address,reg_val);

    cmd_p->address         = 0xc5;
    exec_display_command(dev);
    reg_val = 0xFF & READREG(RI_QSPI_DISPLAY_DISP_CTRL_SPI_READ_DATA_REG);
    LOG_DBG("read from 0x%6x = 0x%02x",cmd_p->address,reg_val);


    MODIFYREG(RBB_FPGA_TOP_OUT_0_REG, (0x1 << 6), (0x0 << 6));  // Trigger 0 SPARE1 (Rubin board X7-1)

    cmd_p->read_en         = 0;
    cmd_p->len             = 4;

    switch_qspi_mode(dev);

#endif
}

void display_image_write(const struct device *dev)
{
    const struct ri_qspi_cfg *cfg = dev->config;
    struct ri_qspi_data *data = dev->data;
    ri_qspi_job_t* job_p = &data->spi_job;
    
    LOG_DBG("------------------------");
    LOG_DBG("write image to display");

    uint32_t  frame_size_display   = cfg->x_res * cfg->y_res * data->pixel_size;


    job_p->px_bytes_num    = frame_size_display;
    job_p->wr_len          = 4;
    job_p->rd_len          = 0;
    job_p->wr_data_byte_0  = 0x02;
    job_p->wr_data_byte_1  = 0x00;
    job_p->wr_data_byte_2  = 0x2C;
    job_p->wr_data_byte_3  = 0x00;
    job_p->valid           = true;

    config_RDMA(dev);
    exec_spi_job(dev);
    poll_qspi_display_ctrl_status_idle();
    WRITEREG(RI_QSPI_DISPLAY_RDMA_RI_RDMA_REG_CTRL_REG, 0x0);

    WRITEREG(RI_QSPI_DISPLAY_DISP_CTRL_QSPI_DISP_CTRL_REG, 0x1); // clear buffers

}

void exec_spi_job (const struct device *dev)
{
    struct ri_qspi_data *data = dev->data;
    ri_qspi_job_t* job_p = &data->spi_job;

    WRITEREG(RI_QSPI_DISPLAY_DISP_CTRL_SPI_CMD_CTRL_0_REG, (job_p->wr_len << RI_QSPI_DISP_REGFILE__SPI_CMD_CTRL_0__WRITE_LENGTH__SHIFT) |
                                                           (job_p->rd_len << RI_QSPI_DISP_REGFILE__SPI_CMD_CTRL_0__READ_LENGTH__SHIFT) );

    WRITEREG(RI_QSPI_DISPLAY_DISP_CTRL_PIXEL_DATA_TRANSFER_REG, job_p->px_bytes_num);

    WRITEREG(RI_QSPI_DISPLAY_DISP_CTRL_SPI_CMD_WRITE_DATA_0_REG,  (job_p->wr_data_byte_0  << RI_QSPI_DISP_REGFILE__SPI_CMD_WRITE_DATA_0__BYTE_0__SHIFT) |
                                                                  (job_p->wr_data_byte_1  << RI_QSPI_DISP_REGFILE__SPI_CMD_WRITE_DATA_0__BYTE_1__SHIFT) |
                                                                  (job_p->wr_data_byte_2  << RI_QSPI_DISP_REGFILE__SPI_CMD_WRITE_DATA_0__BYTE_2__SHIFT) |
                                                                  (job_p->wr_data_byte_3  << RI_QSPI_DISP_REGFILE__SPI_CMD_WRITE_DATA_0__BYTE_3__SHIFT));

    if (job_p->wr_len > 4) {
        WRITEREG(RI_QSPI_DISPLAY_DISP_CTRL_SPI_CMD_WRITE_DATA_1_REG,  (job_p->wr_data_byte_4  << RI_QSPI_DISP_REGFILE__SPI_CMD_WRITE_DATA_1__BYTE_4__SHIFT) |
                                                                      (job_p->wr_data_byte_5  << RI_QSPI_DISP_REGFILE__SPI_CMD_WRITE_DATA_1__BYTE_5__SHIFT) |
                                                                      (job_p->wr_data_byte_6  << RI_QSPI_DISP_REGFILE__SPI_CMD_WRITE_DATA_1__BYTE_6__SHIFT) |
                                                                      (job_p->wr_data_byte_7  << RI_QSPI_DISP_REGFILE__SPI_CMD_WRITE_DATA_1__BYTE_7__SHIFT));
    }
    if (job_p->wr_len > 8) {
        WRITEREG(RI_QSPI_DISPLAY_DISP_CTRL_SPI_CMD_WRITE_DATA_2_REG,  (job_p->wr_data_byte_8  << RI_QSPI_DISP_REGFILE__SPI_CMD_WRITE_DATA_2__BYTE_8__SHIFT) |
                                                                      (job_p->wr_data_byte_9  << RI_QSPI_DISP_REGFILE__SPI_CMD_WRITE_DATA_2__BYTE_9__SHIFT) |
                                                                      (job_p->wr_data_byte_10 << RI_QSPI_DISP_REGFILE__SPI_CMD_WRITE_DATA_2__BYTE_10__SHIFT) |
                                                                      (job_p->wr_data_byte_11 << RI_QSPI_DISP_REGFILE__SPI_CMD_WRITE_DATA_2__BYTE_11__SHIFT));
    }
    if (job_p->wr_len > 12) {
        WRITEREG(RI_QSPI_DISPLAY_DISP_CTRL_SPI_CMD_WRITE_DATA_3_REG,  (job_p->wr_data_byte_12 << RI_QSPI_DISP_REGFILE__SPI_CMD_WRITE_DATA_3__BYTE_12__SHIFT) |
                                                                      (job_p->wr_data_byte_13 << RI_QSPI_DISP_REGFILE__SPI_CMD_WRITE_DATA_3__BYTE_13__SHIFT) |
                                                                      (job_p->wr_data_byte_14 << RI_QSPI_DISP_REGFILE__SPI_CMD_WRITE_DATA_3__BYTE_14__SHIFT) |
                                                                      (job_p->wr_data_byte_15 << RI_QSPI_DISP_REGFILE__SPI_CMD_WRITE_DATA_3__BYTE_15__SHIFT));
    }

    WRITEREG(RI_QSPI_DISPLAY_DISP_CTRL_SPI_CMD_CTRL_1_REG,  1); // start SPI transfer
    LOG_DBG("exec SPI job");

    if (job_p->valid == false) { // Wait & Poll until valid
        poll_qspi_display_ctrl_status_idle();
    }

}

/* API Functions */

static int ri_qspi_display_init(const struct device *dev) {
    LOG_INF("Initializing Racyics display...");

    const struct ri_qspi_cfg *cfg = dev->config;

    int err = 0;

    config_spi_m(dev);
    switch_spi_mode(dev);
    config_pixel_format(dev);
    display_reset();
    display_init(dev);

    LOG_INF("QSPI display %d initialized: %dx%d", cfg->id, cfg->x_res, cfg->y_res);

    return err;
}

static int ri_qspi_display_image_write( const struct device *dev,
                                        const uint16_t x, const uint16_t y,
                                        const struct display_buffer_descriptor *desc,
                                        const void *buf)
{
    // display_setup_partial_mode(image_p);
    struct ri_qspi_data *data = dev->data;
    int err = 0;

    if (!data->frame_buf_p || !buf) {
        LOG_ERR("ri_qspi_display_image_write: invalid buffer");
        return 1;
    }
    if (data->frame_buf_p == buf) {
      // if image is already in frame buffer, set pixel format accordingly
      LOG_DBG("Image already in buffer");
      //data->pixel_size = image_p->pixel_size;
      config_pixel_format(dev);
    } else {
        LOG_DBG("Compiling frame into buffer");
        compile_qspi_display_frame(dev, x, y, desc, buf);
    }

    display_image_write(dev);

	return err;
}

static void ri_qspi_display_get_capabilities(const struct device *dev,
			                                struct display_capabilities *capabilities)
{
    const struct ri_qspi_cfg *cfg = dev->config;
    struct ri_qspi_data *data = dev->data;
    
    memset(capabilities, 0, sizeof(struct display_capabilities));

    capabilities->x_resolution = cfg->x_res;
	capabilities->y_resolution = cfg->y_res;
    capabilities->current_orientation = DISPLAY_ORIENTATION_NORMAL;
	capabilities->current_pixel_format = data->pixel_format;
    capabilities->supported_pixel_formats = PIXEL_FORMAT_RGB_888;

}

static void *ri_qspi_display_get_framebuffer(const struct device *dev)
{
    struct ri_qspi_data *data = dev->data; 

    return data->frame_buf_p;
}

static int ri_qspi_display_set_brightness( const struct device *dev,
					                        const uint8_t brightness)
{
    struct ri_qspi_data *data = dev->data;
    ri_qspi_display_cmd_t*  cmd_p = &data->display_command;

    cmd_p->len     = 1;
    cmd_p->address = 0x51; // WRDISBV: Write Display Brightness
    uint8_t clipped_brightness = MIN(QSPI_DISP_MAX_BRIGHTNESS, brightness); // clip to maximum

    LOG_DBG("brightness %d", clipped_brightness);
    cmd_p->wr_data_byte_0  = clipped_brightness;
    exec_display_command(dev);

    LOG_INF("Brightness set to %u", brightness);

    return 0;
}

/* Device instantiation */

static DEVICE_API(display, ri_qspi_display_api) = {
	.write =  ri_qspi_display_image_write,
    .set_brightness = ri_qspi_display_set_brightness,
    .get_capabilities = ri_qspi_display_get_capabilities,
	.get_framebuffer = ri_qspi_display_get_framebuffer
};

#define RI_QSPI_DISP_DEFINE(inst)                                       \
                                                                        \
    __aligned(4) static uint8_t buffer_##inst[                          \
        BYTES_PER_PIXEL * DT_INST_PROP(inst, width) *                   \
        DT_INST_PROP(inst, height)] DISPLAY_BUFFER_LINKER_SECTION;      \
                                                                        \
    static const struct ri_qspi_cfg ri_qspi_cfg_##inst = {              \
        .id = inst,                                                     \
        .base_addr = (mm_reg_t)DT_INST_REG_ADDR(inst),                  \
        .qspi_dev = {                                                   \
            .cpha = 1,                                                  \
            .cpol = 1,                                                  \
            .cdiv = 80,                                                 \
            .css  = 1,                                                  \
            .csh  = 1,                                                  \
            .csi  = 1                                                   \
        },                                                              \
        .x_res = DT_INST_PROP(inst, width),                             \
        .y_res = DT_INST_PROP(inst, height)                             \
                                                                        \
    };                                                                  \
                                                                        \
    static struct ri_qspi_data ri_qspi_data_##inst = {                  \
        .pixel_size = RI_QSPI_DISPLAY_PIXEL_SIZE(inst),                 \
        .pixel_format = DT_INST_PROP(inst, pixel_format),               \
        .frame_buf_p = &buffer_##inst[0],                               \
        .frame_buf_size8 = sizeof(buffer_##inst),                       \
        .display_command = (ri_qspi_display_cmd_t){0},                  \
    };                                                                  \
                                                                        \
    DEVICE_DT_INST_DEFINE(  inst, ri_qspi_display_init,                 \
                            PM_DEVICE_DT_INST_GET(inst),                \
                            &ri_qspi_data_##inst, &ri_qspi_cfg_##inst,  \
                            POST_KERNEL, CONFIG_DISPLAY_INIT_PRIORITY,  \
                            &ri_qspi_display_api );                     \

DT_INST_FOREACH_STATUS_OKAY(RI_QSPI_DISP_DEFINE)
