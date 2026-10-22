/*
 * Copyright (c) 2026 Racyics GmbH.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/devicetree.h>
#include <soc.h>

LOG_MODULE_REGISTER(riev_fpga_dev_board, CONFIG_LOG_DEFAULT_LEVEL);

/* FPGA_TOP_OUT_1 bits */
#define FPGA_OUT1_CSI_33_EN_BIT \
	(1U << (RBB_REG__FPGA_TOP_OUT_1__OTHER_OUTS_0__SHIFT + 1U))

/* FPGA_TOP_OUT_0 bits */
#define FPGA_OUT0_CAM2_PWUP_BIT     (1U << 22)
#define FPGA_OUT0_CAM1_CLK_EN_BIT   (1U << 23)
#define FPGA_OUT0_ENABLE_CODEC_BIT  ((1U << 14) | (1U << 13))
#define FPGA_OUT0_BOOT_N_GYRO_BIT   ((1U << 12) | (1U << 11))
#define FPGA_OUT0_RSTN_N_GYRO_BIT   (1U << 10)

typedef enum {
	FPGA_I2C_MUX_SEL_NONE = 0,
	FPGA_I2C_MUX_SEL_1_DSI = 1,
	FPGA_I2C_MUX_SEL_2_CSI = 2,
	FPGA_I2C_MUX_SEL_3_OTH = 3,
	FPGA_I2C_MUX_SEL_COUNT = 4
} fpga_i2c_mux_sel_t;

static void fpga_i2c_mux_set(fpga_i2c_mux_sel_t sel)
{
	if (sel >= FPGA_I2C_MUX_SEL_COUNT) {
		LOG_ERR("I2C mux sel out of range: %d", (int)sel);
		return;
	}

	MODIFYREG(RBB_FPGA_TOP_OUT_0_REG,
		  RBB_REG__FPGA_TOP_OUT_0__I2C_MUX_SEL__FIELD,
		  (uint32_t)sel << RBB_REG__FPGA_TOP_OUT_0__I2C_MUX_SEL__SHIFT);

	LOG_INF("FPGA_TOP_OUT_0 after mux set: 0x%08x (sel=%d)",
		(unsigned int)READREG(RBB_FPGA_TOP_OUT_0_REG), (int)sel);
}

static int fpga_dev_board_init(void)
{
	/* 1. Camera / MIPI CSI-2 init if enabled in DT */
#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(csi0))
	LOG_INF("Init FPGA MIPI CSI camera...");
	MODIFYREG(RBB_FPGA_TOP_OUT_1_REG, FPGA_OUT1_CSI_33_EN_BIT, FPGA_OUT1_CSI_33_EN_BIT);
	k_msleep(100);

	MODIFYREG(RBB_FPGA_TOP_OUT_0_REG, FPGA_OUT0_CAM2_PWUP_BIT, 0U);
	k_msleep(100);

	MODIFYREG(RBB_FPGA_TOP_OUT_0_REG, FPGA_OUT0_CAM1_CLK_EN_BIT, FPGA_OUT0_CAM1_CLK_EN_BIT);

	MODIFYREG(RBB_FPGA_TOP_OUT_0_REG, FPGA_OUT0_CAM2_PWUP_BIT, FPGA_OUT0_CAM2_PWUP_BIT);
	k_msleep(100);

	fpga_i2c_mux_set(FPGA_I2C_MUX_SEL_2_CSI);
#endif

	/* 2. Audio Codec (CMX655D) & Gyroscope init if I2S is enabled in DT */
#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(i2s0))
	LOG_INF("Init FPGA Audio codec & gyro...");
	MODIFYREG(RBB_FPGA_TOP_OUT_0_REG,
		  FPGA_OUT0_BOOT_N_GYRO_BIT | FPGA_OUT0_RSTN_N_GYRO_BIT, 0U);

	fpga_i2c_mux_set(FPGA_I2C_MUX_SEL_3_OTH);

	MODIFYREG(RBB_FPGA_TOP_OUT_0_REG, FPGA_OUT0_BOOT_N_GYRO_BIT, FPGA_OUT0_BOOT_N_GYRO_BIT);
	MODIFYREG(RBB_FPGA_TOP_OUT_0_REG, FPGA_OUT0_RSTN_N_GYRO_BIT, FPGA_OUT0_RSTN_N_GYRO_BIT);

	MODIFYREG(RBB_FPGA_TOP_OUT_0_REG, FPGA_OUT0_ENABLE_CODEC_BIT, FPGA_OUT0_ENABLE_CODEC_BIT);
#endif

	return 0;
}

SYS_INIT(fpga_dev_board_init, POST_KERNEL, 45);

