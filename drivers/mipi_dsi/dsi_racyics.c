/*
 * Copyright (c) 2026 Racyics GmbH.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT racyics_mipi_dsi

#include <errno.h>
#include <string.h>
#include <soc.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/mipi_dsi.h>
#include <zephyr/drivers/mipi_dsi/mipi_dsi_racyics.h>
#include <zephyr/display/mipi_display.h>

LOG_MODULE_REGISTER(dsi_racyics, CONFIG_MIPI_DSI_LOG_LEVEL);

#define DSI_CORE_ID_EXPECTED		0x30313130U
#define DSI_CORE_VERSION_EXPECTED	0x1A000110U

#define DSI_MODE_CTRL_AUTO_CALC		1U
#define DSI_MODE_CTRL_COMMAND		2U
#define DSI_MODE_CTRL_VIDEO		3U

#define DSI_CORE_STATUS_CRI_BUSY_SHIFT	16U

#define DSI_CRI_HDR_VC_SHIFT		6U
#define DSI_CRI_HDR_TYPE_MASK		0x3FU
#define DSI_CRI_HDR_BYTE_SHIFT(n)	(8U * (n))
#define DSI_CRI_HDR_LP_BIT		BIT(24)
#define DSI_CRI_HDR_LONG_BIT		BIT(29)

#define DSI_CRI_BUSY_TIMEOUT_US		100000U
#define DSI_AUTO_CALC_TIMEOUT_US	1000000U

struct mipi_dsi_racyics_data {
	struct k_mutex lock;
	bool attached;
};

/* Helper Functions */

static bool dsi_msg_is_dcs(uint8_t type)
{
	return (type == MIPI_DSI_DCS_SHORT_WRITE) || (type == MIPI_DSI_DCS_SHORT_WRITE_PARAM) ||
	       (type == MIPI_DSI_DCS_LONG_WRITE);
}

static bool dsi_msg_is_long(uint8_t type)
{
	return (type == MIPI_DSI_GENERIC_LONG_WRITE) || (type == MIPI_DSI_DCS_LONG_WRITE);
}

static bool dsi_msg_is_short(uint8_t type)
{
	return (type == MIPI_DSI_GENERIC_SHORT_WRITE_0_PARAM) ||
	       (type == MIPI_DSI_GENERIC_SHORT_WRITE_1_PARAM) ||
	       (type == MIPI_DSI_GENERIC_SHORT_WRITE_2_PARAM) ||
	       (type == MIPI_DSI_DCS_SHORT_WRITE) || (type == MIPI_DSI_DCS_SHORT_WRITE_PARAM);
}

static int dsi_wait_cri_idle(void)
{
	bool idle;

	idle = WAIT_FOR(((READREG(DSI_DISP_CORE_STATUS) >> DSI_CORE_STATUS_CRI_BUSY_SHIFT) &
			 0x1U) == 0U,
			DSI_CRI_BUSY_TIMEOUT_US, k_busy_wait(10));
	if (!idle) {
		LOG_ERR("CRI busy timeout");
		return -ETIMEDOUT;
	}

	return 0;
}

/*
 * Push the payload into the CRI payload FIFO as little endian 32 bit words.
 * For DCS long writes the DCS command byte leads the payload.
 */
static int dsi_write_payload(const struct mipi_dsi_msg *msg, bool dcs_cmd_first)
{
	const uint8_t *buf = msg->tx_buf;
	uint32_t word = 0U;
	size_t total = msg->tx_len + (dcs_cmd_first ? 1U : 0U);
	size_t i;

	for (i = 0U; i < total; i++) {
		uint8_t byte;

		if (dcs_cmd_first && (i == 0U)) {
			byte = msg->cmd;
		} else {
			byte = buf[i - (dcs_cmd_first ? 1U : 0U)];
		}

		word |= (uint32_t)byte << (8U * (i % 4U));

		if ((i % 4U) == 3U) {
			WRITEREG(DSI_DISP_CRI_TX_PLD, word);
			word = 0U;
		}
	}

	if ((total % 4U) != 0U) {
		WRITEREG(DSI_DISP_CRI_TX_PLD, word);
	}

	return 0;
}

/* API Functions */

static int mipi_dsi_racyics_attach(const struct device *dev, uint8_t channel,
				   const struct mipi_dsi_device *mdev)
{
	struct mipi_dsi_racyics_data *data = dev->data;
	uint32_t reg;

	if (channel > 3U) {
		return -EINVAL;
	}

	if ((mdev->data_lanes == 0U) || (mdev->data_lanes > 4U)) {
		LOG_ERR("Unsupported number of data lanes: %u", mdev->data_lanes);
		return -EINVAL;
	}

	if (mdev->pixfmt != MIPI_DSI_PIXFMT_RGB888) {
		LOG_ERR("Unsupported pixel format: %u", mdev->pixfmt);
		return -ENOTSUP;
	}

	reg = READREG(DSI_DISP_CORE_ID);
	if (reg != DSI_CORE_ID_EXPECTED) {
		LOG_ERR("Unexpected core ID 0x%08x", reg);
		return -ENODEV;
	}

	reg = READREG(DSI_DISP_VERSION);
	if (reg != DSI_CORE_VERSION_EXPECTED) {
		LOG_ERR("Unexpected core version 0x%08x", reg);
		return -ENODEV;
	}

	WRITEREG_LOG("DSI_DISP_PHY_MODE_CFG", DSI_DISP_PHY_MODE_CFG, 0x00000030U);

	if (IS_ENABLED(CONFIG_BOARD_RIEV_FPGA_DEV_BOARD)) {
		WRITEREG_LOG("DSI_DISP_PHY_CLK_CFG", DSI_DISP_PHY_CLK_CFG, 0x00000301U);
	} else {
		WRITEREG_LOG("DSI_DISP_PHY_CLK_CFG", DSI_DISP_PHY_CLK_CFG, 0x00000701U);
	}

	WRITEREG_LOG("DSI_DISP_DSI_GENERAL_CFG", DSI_DISP_DSI_GENERAL_CFG, 0x00000001U);

	/* Commands only in LP mode, video mode type burst */
	WRITEREG_LOG("DSI_DISP_DSI_VID_TX_CFG", DSI_DISP_DSI_VID_TX_CFG, 0x00100002U);

	if (!IS_ENABLED(CONFIG_BOARD_RIEV_FPGA_DEV_BOARD) &&
	    !IS_ENABLED(CONFIG_BOARD_RIEV_DEV_BOARD)) {
		WRITEREG_LOG("DSI_DISP_DSI_DS_TX_CFG", DSI_DISP_DSI_DS_TX_CFG, 0x00000001U);
	}

	WRITEREG_LOG("DSI_DISP_IPI_COLOR_MAN_CFG", DSI_DISP_IPI_COLOR_MAN_CFG, 0x00000050U);
	WRITEREG_LOG("DSI_DISP_MANUAL_MODE_CFG", DSI_DISP_MANUAL_MODE_CFG, 0x00000000U);
	WRITEREG_LOG("DSI_DISP_PWR_UP", DSI_DISP_PWR_UP, 0x00000001U);
	WRITEREG_LOG("DSI_DISP_MODE_CTRL", DSI_DISP_MODE_CTRL, DSI_MODE_CTRL_AUTO_CALC);

	data->attached = true;

	return 0;
}

static ssize_t mipi_dsi_racyics_transfer(const struct device *dev, uint8_t channel,
					 struct mipi_dsi_msg *msg)
{
	struct mipi_dsi_racyics_data *data = dev->data;
	bool dcs = dsi_msg_is_dcs(msg->type);
	bool is_long = dsi_msg_is_long(msg->type);
	uint32_t hdr;
	ssize_t ret;

	if (channel > 3U) {
		return -EINVAL;
	}

	if (!data->attached) {
		return -ENODEV;
	}

	if (!is_long && !dsi_msg_is_short(msg->type)) {
		LOG_ERR("Unsupported message type 0x%02x", msg->type);
		return -ENOTSUP;
	}

	if ((msg->rx_buf != NULL) && (msg->rx_len > 0U)) {
		LOG_ERR("Reads are not supported");
		return -ENOTSUP;
	}

	if ((msg->tx_len > 0U) && (msg->tx_buf == NULL)) {
		return -EINVAL;
	}

	hdr = ((uint32_t)(channel & 0x3U) << DSI_CRI_HDR_VC_SHIFT) |
	      (msg->type & DSI_CRI_HDR_TYPE_MASK);
	/* The host sends every command packet in LP mode */
	hdr |= DSI_CRI_HDR_LP_BIT;

	if (is_long) {
		size_t wc = msg->tx_len + (dcs ? 1U : 0U);

		if (wc > UINT16_MAX) {
			return -EINVAL;
		}

		hdr |= DSI_CRI_HDR_LONG_BIT;
		hdr |= (uint32_t)(wc & 0xFFU) << DSI_CRI_HDR_BYTE_SHIFT(1);
		hdr |= (uint32_t)((wc >> 8) & 0xFFU) << DSI_CRI_HDR_BYTE_SHIFT(2);
	} else {
		const uint8_t *tx = msg->tx_buf;
		uint8_t p0 = 0U;
		uint8_t p1 = 0U;

		if (dcs) {
			p0 = msg->cmd;
			p1 = (msg->tx_len > 0U) ? tx[0] : 0U;
		} else {
			p0 = (msg->tx_len > 0U) ? tx[0] : 0U;
			p1 = (msg->tx_len > 1U) ? tx[1] : 0U;
		}

		hdr |= (uint32_t)p0 << DSI_CRI_HDR_BYTE_SHIFT(1);
		hdr |= (uint32_t)p1 << DSI_CRI_HDR_BYTE_SHIFT(2);
	}

	(void)k_mutex_lock(&data->lock, K_FOREVER);

	ret = dsi_wait_cri_idle();
	if (ret == 0) {
		if (is_long) {
			(void)dsi_write_payload(msg, dcs);
		}
		WRITEREG(DSI_DISP_CRI_TX_HDR, hdr);
		ret = (ssize_t)msg->tx_len;
	}

	k_mutex_unlock(&data->lock);

	return ret;
}

int mipi_dsi_racyics_set_command_mode(const struct device *dev)
{
	struct mipi_dsi_racyics_data *data = dev->data;
	bool done;

	if (!data->attached) {
		return -ENODEV;
	}

	/* MODE_STATUS reads 0 once the automatic calculation has finished */
	done = WAIT_FOR(READREG(DSI_DISP_MODE_STATUS) == 0U, DSI_AUTO_CALC_TIMEOUT_US,
			k_busy_wait(100));
	if (!done) {
		LOG_ERR("Automatic calculation did not finish");
		return -ETIMEDOUT;
	}

	WRITEREG_LOG("DSI_DISP_MODE_CTRL", DSI_DISP_MODE_CTRL, DSI_MODE_CTRL_COMMAND);

	return 0;
}

int mipi_dsi_racyics_start_video(const struct device *dev)
{
	struct mipi_dsi_racyics_data *data = dev->data;

	if (!data->attached) {
		return -ENODEV;
	}

	/* DS_HS_LINK_EN */
	WRITEREG_LOG("DSI_DISP_DSI_DS_TX_CFG", DSI_DISP_DSI_DS_TX_CFG, 0x00000001U);
	WRITEREG_LOG("DSI_DISP_MODE_CTRL", DSI_DISP_MODE_CTRL, DSI_MODE_CTRL_VIDEO);

	return 0;
}

/* Device Instantiation */

static DEVICE_API(mipi_dsi, mipi_dsi_racyics_api) = {
	.attach = mipi_dsi_racyics_attach,
	.transfer = mipi_dsi_racyics_transfer,
};

static int mipi_dsi_racyics_init(const struct device *dev)
{
	struct mipi_dsi_racyics_data *data = dev->data;

	k_mutex_init(&data->lock);
	data->attached = false;

	return 0;
}

#define RACYICS_MIPI_DSI_DEVICE(inst)                                                              \
	static struct mipi_dsi_racyics_data mipi_dsi_racyics_data_##inst;                          \
	DEVICE_DT_INST_DEFINE(inst, mipi_dsi_racyics_init, NULL, &mipi_dsi_racyics_data_##inst,    \
			      NULL, POST_KERNEL, CONFIG_MIPI_DSI_INIT_PRIORITY,                    \
			      &mipi_dsi_racyics_api);

DT_INST_FOREACH_STATUS_OKAY(RACYICS_MIPI_DSI_DEVICE)
