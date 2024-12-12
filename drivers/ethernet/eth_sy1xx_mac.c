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
};

struct sy1xx_mac_dev_data {
	struct k_sem sem;

	uint8_t mac[6];

	bool link_up;

	uint8_t tx_buffer[MAX_MAC_PACKET_LEN];
	uint16_t tx_buffer_len;

	uint8_t rx_buffer[MAX_MAC_PACKET_LEN];
	uint16_t rx_buffer_len;

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

	for (uint32_t i = 0; i < 6; i++) {
		data->mac[i] = 0x02 * (i + 1);
	}

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
	struct sy1xx_mac_dev_data *data = (struct sy1xx_mac_dev_data *)dev->data;

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

	is_up = state->is_up;

	if (is_up && !data->link_up) {
		LOG_INF("Link up");

		/* Announce link up status */
		data->link_up = true;
		net_eth_carrier_on(cfg->iface);

		/* configure mac, based on provided link information 1Gbs/100MBit/... */
		/* ... */

	} else if (!is_up && data->link_up) {
		LOG_INF("Link down");

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
	LOG_DBG("ganymed mac get_caps");

	return ETHERNET_LINK_1000BASE_T;
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
	LOG_INF("ganymed mac set config");
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

static int sy1xx_mac_send(const struct device *dev, struct net_pkt *pkt)
{
	LOG_INF("send %d", pkt->buffer->len);

	return 0;
}

int32_t sy1xx_mac_receive_data(uint8_t *data, uint16_t len)
{
	/*
		struct net_pkt *rx_pkt; // = net_pkt_rx_alloc_on_iface(ganymed_instance.iface,
	   K_NO_WAIT); rx_pkt = net_pkt_alloc_with_buffer(ganymed_instance.iface, len, AF_UNSPEC, 0,
	   K_NO_WAIT); if (rx_pkt == NULL) { LOG_ERR("rx packet allocation failed"); return -1;
		}

		// Add data to the net_pkt
		if (net_pkt_write(rx_pkt, data, len)) {
			LOG_ERR("Failed to write data to net_pkt");
			net_pkt_unref(rx_pkt);
			return -2;
		}

		volatile int32_t ret = net_recv_data(ganymed_instance.iface, rx_pkt);
		if (0 != ret) {
			LOG_ERR("rx packet registration failed");
			// net_pkt_unref(rx_pkt);
			return -2;
		}
	*/
	return 0;
}

void sy1xx_mac_rx_thread_entry(void *p1, void *p2, void *p3)
{

	LOG_INF("rx thread started");

	while (1) {

		k_sleep(K_MSEC(1000));
		// ret = sbi_eth_read(DRIVERS_ETH0, ganymed_instance.rx_buffer, MAX_MAC_PACKET_LEN,
		// &ganymed_instance.rx_buffer_len);
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
	static struct sy1xx_mac_dev_data sy1xx_mac_dev_data##n;                                    \
                                                                                                   \
	ETH_NET_DEVICE_DT_INST_DEFINE(n, &sy1xx_mac_initialize, NULL, &sy1xx_mac_dev_data##n,      \
				      &sy1xx_mac_dev_config_##n, CONFIG_ETH_INIT_PRIORITY,         \
				      &sy1xx_mac_driver_api, NET_ETH_MTU);

DT_INST_FOREACH_STATUS_OKAY(SY1XX_MAC_INIT)
