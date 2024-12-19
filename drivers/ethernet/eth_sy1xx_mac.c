//
// Created by tswaehn on 7/30/24.
//

#define DT_DRV_COMPAT sensry_sy1xx_mac

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(sy1xx_mac, CONFIG_ETHERNET_LOG_LEVEL);

#include <sys/types.h>
#include <zephyr/kernel.h>
#include <zephyr/cache.h>
#include <zephyr/net/ethernet.h>
#include <zephyr/sys/barrier.h>
#include <ethernet/eth_stats.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/net/phy.h>
#include <udma.h>

/* MAC register offsets */
#define SY1XX_MAC_VERSION_REG      0x0000
#define SY1XX_MAC_ADDRESS_LOW_REG  0x0004
#define SY1XX_MAC_ADDRESS_HIGH_REG 0x0008
#define SY1XX_MAC_CTRL_REG         0x000c

/* MAC control register bit offsets */
#define SY1XX_MAC_CTRL_GMII_OFFS    (3)
#define SY1XX_MAC_CTRL_CLK_DIV_OFFS (8)
#define SY1XX_MAC_CTRL_CLK_SEL_OFFS (10)

#define MAX_MAC_PACKET_LEN 1600

#define STACK_SIZE      4096
#define THREAD_PRIORITY K_PRIO_PREEMPT(0)

static void sy1xx_mac_rx_thread_entry(void *p1, void *p2, void *p3);

struct sy1xx_mac_dev_config {
	struct net_if *iface;
	const struct device *dev;
	const struct device *phy_dev;
	const struct pinctrl_dev_config *pcfg;
	uint32_t base_addr;
	uint32_t udma_base;
};

struct dma_buffers {
	uint8_t tx[MAX_MAC_PACKET_LEN];
	uint8_t rx[MAX_MAC_PACKET_LEN];
};

struct sy1xx_mac_dev_data {
	struct k_sem sem;

	uint8_t mac[6];

	bool link_up;

	bool promiscuous_mode;

	uint8_t tx[MAX_MAC_PACKET_LEN];
	uint16_t tx_len;

	uint8_t rx[MAX_MAC_PACKET_LEN];
	uint16_t rx_len;

	struct dma_buffers *dma_buffers;

	struct k_thread rx_data_thread;
	uint8_t rx_data_thread_stack[STACK_SIZE];
};

static int sy1xx_mac_initialize(const struct device *dev)
{
	struct sy1xx_mac_dev_config *cfg = (struct sy1xx_mac_dev_config *)dev->config;
	struct sy1xx_mac_dev_data *data = (struct sy1xx_mac_dev_data *)dev->data;

	/* PAD config */
	if (0 != pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT)) {
		LOG_ERR("failed to configure pins");
	}

	data->link_up = false;
	data->promiscuous_mode = false;
	cfg->udma_base = 0x1A102a00;

	for (uint32_t i = 0; i < 6; i++) {
		data->mac[i] = 0x02 * (i + 1);
	}

	data->mac[0] = 0xb2;
	data->mac[1] = 0x25;
	data->mac[2] = 0xaa;
	data->mac[3] = 0x4e;
	data->mac[4] = 0xd5;
	data->mac[5] = 0x5a;

	/* create the receiver thread in stopped mode; start with device start request */
	k_thread_create(&data->rx_data_thread, (k_thread_stack_t *)data->rx_data_thread_stack,
			STACK_SIZE, sy1xx_mac_rx_thread_entry, (void *)dev, NULL, NULL,
			THREAD_PRIORITY, 0, K_FOREVER);

	k_thread_name_set(&data->rx_data_thread, "mac-rx-thread");

	LOG_INF("initialize done");
	return 0;
}

static int sy1xx_mac_start(const struct device *dev)
{
	struct sy1xx_mac_dev_config *cfg = (struct sy1xx_mac_dev_config *)dev->config;
	struct sy1xx_mac_dev_data *data = (struct sy1xx_mac_dev_data *)dev->data;

	extern void sy1xx_udma_disable_clock(sy1xx_udma_module_t module, uint32_t instance);

	/* UDMA clock enable */
	sy1xx_udma_enable_clock(SY1XX_UDMA_MODULE_MAC, 0);
	sy1xx_udma_disable_clock(SY1XX_UDMA_MODULE_MAC, 0);
	sy1xx_udma_enable_clock(SY1XX_UDMA_MODULE_MAC, 0);

	/* reset mac */
	sys_write32(0x0001, cfg->base_addr + SY1XX_MAC_CTRL_REG);
	sys_write32(0x0000, cfg->base_addr + SY1XX_MAC_CTRL_REG);

	/* update mac in controller */
	uint32_t v0 = sys_read32(cfg->base_addr + SY1XX_MAC_ADDRESS_LOW_REG);
	uint32_t v1 = sys_read32(cfg->base_addr + SY1XX_MAC_ADDRESS_HIGH_REG);

	/* mask out relevant fields, leave lower bits */
	v0 &= 0x00000000;
	v1 &= 0x0000ffff;

	v0 = (data->mac[0] << 0) | (data->mac[1] << 8) | (data->mac[2] << 16) |
	     (data->mac[3] << 24);
	v1 = (data->mac[4] << 0) | (data->mac[5] << 8);

	sys_write32(v0, cfg->base_addr + SY1XX_MAC_ADDRESS_LOW_REG);
	sys_write32(v1, cfg->base_addr + SY1XX_MAC_ADDRESS_HIGH_REG);

	/* set promiscuous mode */
	uint32_t prom = sys_read32(cfg->base_addr + SY1XX_MAC_ADDRESS_HIGH_REG);
	if (data->promiscuous_mode) {
		prom &= ~(1 << 16);
	} else {
		prom |= (1 << 16);
	}
	sys_write32(prom, cfg->base_addr + SY1XX_MAC_ADDRESS_HIGH_REG);

	k_thread_resume(&data->rx_data_thread);
	LOG_INF("started device");

	return 0;
}

static int sy1xx_mac_stop(const struct device *dev)
{
	struct sy1xx_mac_dev_data *data = (struct sy1xx_mac_dev_data *)dev->data;

	k_thread_suspend(&data->rx_data_thread);
	LOG_INF("stopped device");

	return 0;
}

static void phy_link_state_changed(const struct device *pdev, struct phy_link_state *state,
				   void *user_data)
{
	const struct device *dev = (const struct device *)user_data;
	struct sy1xx_mac_dev_data *const data = dev->data;
	const struct sy1xx_mac_dev_config *const cfg = dev->config;
	bool is_up;
	enum phy_link_speed speed;

	is_up = state->is_up;
	speed = state->speed;

	if (is_up && !data->link_up) {
		LOG_INF("Link up");

		/* Announce link up status */
		data->link_up = true;
		net_eth_carrier_on(cfg->iface);

		/* configure mac, based on provided link information 1Gbs/100MBit/... */
		uint32_t v = sys_read32(cfg->base_addr + SY1XX_MAC_CTRL_REG);
		enum {
			GMII = 3,
			CLK_DIV = 8,
			CLK_SEL = 10
		};
		v &= ~((1 << SY1XX_MAC_CTRL_GMII_OFFS) | (1 << SY1XX_MAC_CTRL_CLK_SEL_OFFS) |
		       (3 << SY1XX_MAC_CTRL_CLK_DIV_OFFS));

		switch (speed) {
		case LINK_FULL_10BASE_T:
			LOG_INF("link speed FULL_10BASE_T");
			v |= (0 << GMII) | (1 << CLK_SEL) |
			     (2 << CLK_DIV); // 2.5MHz, MAC is clock source
			break;
		case LINK_FULL_100BASE_T:
			LOG_INF("link speed FULL_100BASE_T");
			v |= (0 << GMII) | (1 << CLK_SEL) |
			     (0 << CLK_DIV); // 25MHz, MAC is clock source
			break;
		case LINK_FULL_1000BASE_T:
			LOG_INF("link speed FULL_1000BASE_T");
			v |= (1 << GMII) | (0 << CLK_SEL) |
			     (0 << CLK_DIV); // 125MHz, Phy is clock source
			break;
		default:
			LOG_WRN("invalid speed link speed");
			return;
		}

		sys_write32(v, cfg->base_addr + SY1XX_MAC_CTRL_REG);

		/* enable mac controller */
		uint32_t en = sys_read32(cfg->base_addr + SY1XX_MAC_CTRL_REG);
		en |= (3 << 1);
		sys_write32(en, cfg->base_addr + SY1XX_MAC_CTRL_REG);

	} else if (!is_up && data->link_up) {
		LOG_INF("Link down");

		/* disable mac controller */
		uint32_t en = sys_read32(cfg->base_addr + SY1XX_MAC_CTRL_REG);
		en &= ~(3 << 1);
		sys_write32(en, cfg->base_addr + SY1XX_MAC_CTRL_REG);

		/* Announce link down status */
		data->link_up = false;
		net_eth_carrier_off(cfg->iface);
	}
}

static void sy1xx_mac_iface_init(struct net_if *iface)
{
	LOG_INF("interface init %s (%.8x)", net_if_get_device(iface)->name, iface);

	const struct device *dev = net_if_get_device(iface);
	struct sy1xx_mac_dev_config *cfg = (struct sy1xx_mac_dev_config *)dev->config;
	struct sy1xx_mac_dev_data *const data = dev->data;

	cfg->iface = iface;

	ethernet_init(iface);

	net_if_set_link_addr(iface, data->mac, sizeof(data->mac), NET_LINK_ETHERNET);

	if (device_is_ready(cfg->phy_dev)) {
		phy_link_callback_set(cfg->phy_dev, &phy_link_state_changed, (void *)dev);

	} else {
		LOG_ERR("PHY device not ready");
	}

	/* Do not start the interface until PHY link is up */
	if (!(data->link_up)) {
		LOG_WRN("found PHY link down");
		net_if_carrier_off(iface);
	}

	LOG_INF("interface init done");
}

static enum ethernet_hw_caps sy1xx_mac_get_caps(const struct device *dev)
{
	enum ethernet_hw_caps supported = 0;

	supported |= ETHERNET_PROMISC_MODE;
	supported |= ETHERNET_LINK_1000BASE_T;

	return supported;
}

static int sy1xx_mac_get_config(const struct device *dev, enum ethernet_config_type type,
				struct ethernet_config *config)
{
	LOG_INF("ganymed mac get config");
	if (type == ETHERNET_CONFIG_TYPE_LINK) {
		config->l.link_1000bt = true;
	}
	if (type == ETHERNET_CONFIG_TYPE_DUPLEX) {
		config->full_duplex = true;
	}

	return 0;
}

static int sy1xx_mac_set_config(const struct device *dev, enum ethernet_config_type type,
				const struct ethernet_config *config)
{
	LOG_INF("set config");
	switch (type) {
	case ETHERNET_CONFIG_TYPE_MAC_ADDRESS:
		LOG_DBG("setting mac address");
		/*
		memcpy(p->mac_addr, config->mac_address.addr, sizeof(p->mac_addr));
		dwmac_set_mac_addr(p, p->mac_addr, 0);
		net_if_set_link_addr(p->iface, p->mac_addr,
							 sizeof(p->mac_addr), NET_LINK_ETHERNET);
		*/
		break;
	}

	return 0;
}

static const struct device *sy1xx_mac_get_phy(const struct device *dev)
{
	const struct sy1xx_mac_dev_config *const cfg = dev->config;

	return cfg->phy_dev;
}

/*
 * rx ready status of eth is different to any other rx udma,
 * so we implement here
 */
int32_t sy1xx_mac_udma_is_finished_rx(uint32_t base)
{
	uint32_t isBusy = SY1XX_UDMA_READ_REG(base, SY1XX_UDMA_CFG_REG + 0x00) & ((1 << 17));
	return isBusy ? 0 : 1;
}

/**
 * @return
 *  - < 0: Error
 *  -   0: OK
 *  - > 0: Busy
 */
static int sy1xx_mac_low_level_send(const struct device *dev, uint8_t *tx, uint16_t len)
{
	struct sy1xx_mac_dev_config *cfg = (struct sy1xx_mac_dev_config *)dev->config;
	struct sy1xx_mac_dev_data *const data = dev->data;

	LOG_INF("send %d", len);

	if (len == 0) {
		return -EINVAL;
	}

	if (len > MAX_MAC_PACKET_LEN) {
		return -EINVAL;
	}

	int32_t is_finished = SY1XX_UDMA_IS_FINISHED_TX(cfg->udma_base);
	if (!is_finished) {
		return EBUSY;
	}

	// udma is ready, double check if last transmission was successful
	uint32_t remain = SY1XX_UDMA_GET_REMAINING_TX(cfg->udma_base);
	if (remain != 0) {
		SY1XX_UDMA_CANCEL_TX(cfg->udma_base);
		printf("tx - last transmission failed\n");
		return -EINVAL;
	}

	// stop any prior transmission
	// UDMA_WAIT_FOR_FINISHED_TX(ETH_BASE(inst));

	/* copy data to dma buffer */
	for (uint32_t i = 0; i < len; i++) {
		data->dma_buffers->tx[i] = tx[i];
	}

	// start dma transfer
	SY1XX_UDMA_START_TX(cfg->udma_base, (uint32_t)data->dma_buffers->tx, len, 0);

	return 0;
}

static int sy1xx_mac_low_level_receive(const struct device *dev, uint8_t *rx, uint16_t *len)
{
	struct sy1xx_mac_dev_config *cfg = (struct sy1xx_mac_dev_config *)dev->config;
	struct sy1xx_mac_dev_data *const dev_data = dev->data;

	*len = 0;

	/* rx udma still busy */
	if (0 == sy1xx_mac_udma_is_finished_rx(cfg->udma_base)) {
		return EBUSY;
	}

	/* rx udma is ready */
	uint32_t bytes_transferred =
		SY1XX_UDMA_READ_REG(cfg->udma_base, SY1XX_UDMA_CFG_REG) & 0x0000ffff;
	if (bytes_transferred > 0) {
		if (bytes_transferred > 9) {
			// remove header
			bytes_transferred -= 9;
		}

		if (bytes_transferred >= 12) {
			// copy data
			for (uint32_t i = 0; i < bytes_transferred; i++) {
				rx[i] = dev_data->dma_buffers->rx[i];
			}

			*len = bytes_transferred;
		}
	}

	// stop any prior transmission
	// drivers_eth_wait_for_finished_rx(ETH_BASE(inst));

	// start new transmission
	SY1XX_UDMA_START_RX(cfg->udma_base, (uint32_t)dev_data->dma_buffers->rx, MAX_MAC_PACKET_LEN,
			    0);

	/*
	drivers_eth_wait_for_finished_rx(ETH_BASE(inst));

	volatile uint32_t cfg = UDMA_READ_REG(ETH_BASE(inst), UDMA_CFG_REG);

	UDMA_WRITE_REG(ETH_BASE(inst), UDMA_CFG_REG, ((1 << 18) | (1<<17)));

	if ((cfg & 0xffff0000) > 0){
		// timeout
		return -4;
	}
	*/

	return 0;
}

static int sy1xx_mac_send(const struct device *dev, struct net_pkt *pkt)
{
	struct sy1xx_mac_dev_config *cfg = (struct sy1xx_mac_dev_config *)dev->config;
	struct sy1xx_mac_dev_data *const data = dev->data;

	LOG_INF("mac send %d", pkt->buffer->len);

	volatile struct net_buf *frag = pkt->buffer;

	/* push all fragments of the packet into one linear buffer */
	data->tx_len = 0;
	do {

		// copy fragments to buffer
		for (uint32_t i = 0; i < frag->len; i++) {
			if (data->tx_len < MAX_MAC_PACKET_LEN) {
				data->tx[data->tx_len++] = frag->data[i];
			} else {
				LOG_ERR("tx buffer overflow");
				return -1;
			}
		}

		frag = frag->frags;
	} while (frag);

	int ret = sy1xx_mac_low_level_send(dev, data->tx, data->tx_len);
	if (ret == 0) {
		return 0;
	}
	if (ret > 0) {
		LOG_ERR("tx busy");
		return -2;
	}

	LOG_ERR("tx error");
	return ret;
}

int32_t sy1xx_mac_receive_data(const struct device *dev, uint8_t *rx, uint16_t len)
{
	struct sy1xx_mac_dev_config *cfg = (struct sy1xx_mac_dev_config *)dev->config;

	struct net_pkt *rx_pkt; // = net_pkt_rx_alloc_on_iface(ganymed_instance.iface, K_NO_WAIT);

	rx_pkt = net_pkt_alloc_with_buffer(cfg->iface, len, AF_UNSPEC, 0, K_NO_WAIT);
	if (rx_pkt == NULL) {
		LOG_ERR("rx packet allocation failed");
		return -1;
	}

	// Add data to the net_pkt
	if (net_pkt_write(rx_pkt, rx, len)) {
		LOG_ERR("Failed to write data to net_pkt");
		net_pkt_unref(rx_pkt);
		return -2;
	}

	volatile int32_t ret = net_recv_data(cfg->iface, rx_pkt);
	if (0 != ret) {
		LOG_ERR("rx packet registration failed");
		// net_pkt_unref(rx_pkt);
		return -EINVAL;
	}

	return 0;
}

void sy1xx_mac_rx_thread_entry(void *p1, void *p2, void *p3)
{
	LOG_INF("rx thread started");

	const struct device *dev = p1;
	struct sy1xx_mac_dev_config *cfg = (struct sy1xx_mac_dev_config *)dev->config;
	struct sy1xx_mac_dev_data *const data = dev->data;

	while (1) {
		int ret = sy1xx_mac_low_level_receive(dev, data->rx, &data->rx_len);

		if (ret == 0) {
			/* register a new received frame */
			if (data->rx_len > 0){
				sy1xx_mac_receive_data(dev, data->rx, data->rx_len);
			}
			k_yield();
		} else if (ret < 0) {
			/* an error happend */

			LOG_ERR("rx error");
			k_sleep(K_MSEC(1000));
		} else {
			/* busy */
			k_sleep(K_MSEC(1));
		}
	}
}

const struct ethernet_api sy1xx_mac_driver_api = {
	.start = sy1xx_mac_start,
	.stop = sy1xx_mac_stop,
	.iface_api.init = sy1xx_mac_iface_init,
	.get_capabilities = sy1xx_mac_get_caps,
	.get_config = sy1xx_mac_get_config,
	.set_config = sy1xx_mac_set_config,
	.send = sy1xx_mac_send,
	.get_phy = sy1xx_mac_get_phy,
};

#define SY1XX_MAC_INIT(n)                                                                          \
                                                                                                   \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
                                                                                                   \
	static const struct sy1xx_mac_dev_config sy1xx_mac_dev_config_##n = {                      \
		.base_addr = (uint32_t)DT_INST_REG_ADDR(n),                                        \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                         \
		.phy_dev = DEVICE_DT_GET(DT_INST_PHANDLE(0, phy_handle))};                         \
                                                                                                   \
	static struct dma_buffers __attribute__((section(".udma_access")))                         \
	__aligned(4) dma_buffers_##n;                                                              \
                                                                                                   \
	static struct sy1xx_mac_dev_data sy1xx_mac_dev_data##n = {                                 \
		.dma_buffers = &dma_buffers_##n,                                                   \
	};                                                                                         \
                                                                                                   \
	ETH_NET_DEVICE_DT_INST_DEFINE(n, &sy1xx_mac_initialize, NULL, &sy1xx_mac_dev_data##n,      \
				      &sy1xx_mac_dev_config_##n, CONFIG_ETH_INIT_PRIORITY,         \
				      &sy1xx_mac_driver_api, NET_ETH_MTU);

DT_INST_FOREACH_STATUS_OKAY(SY1XX_MAC_INIT)
