// SPDX-License-Identifier: GPL-2.0
/* Copyright (C) 2019 Hewlett-Packard Development Company, L.P.
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/dma-mapping.h>
#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/interrupt.h>
#include <linux/skbuff.h>
#include <linux/crc32.h>
#include <linux/mii.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_mdio.h>
#include <linux/of_net.h>
#include <linux/ethtool.h>
#include <linux/platform_device.h>
#include <linux/irq.h>
#include <net/ip.h>
#include <net/ncsi.h>
#include <linux/phy.h>
#include <linux/errno.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include "gxp-umac.h"

//page 0
#define PHY_88E1514_COPPER_CONTROL_REG		0
#define PHY_88E1514_PAGE_ADDRESS		22

//page 18
#define PHY_88E1514_GENERAL_CONTROL_REG1	20

#define DRV_MODULE_NAME		"gxp-umac"
#define DRV_MODULE_VERSION	"0.1"

struct umac_priv {
	void __iomem *base;
	int irq;
	struct platform_device *pdev;

	struct umac_tx_descs	*tx_descs;
	struct umac_rx_descs	*rx_descs;
	dma_addr_t tx_descs_dma_addr;
	dma_addr_t rx_descs_dma_addr;

	unsigned int tx_cur;
	unsigned int tx_done;
	unsigned int rx_cur;

	struct napi_struct napi;
	struct net_device *ndev;

	struct phy_device *phy_dev;
	struct phy_device *int_phy_dev;

	struct ncsi_dev *ncsidev;
	bool use_ncsi;
};

static void umac_get_drvinfo(struct net_device *ndev,
			     struct ethtool_drvinfo *info)
{
	strlcpy(info->driver, DRV_MODULE_NAME, sizeof(info->driver));
	strlcpy(info->version, DRV_MODULE_VERSION, sizeof(info->version));
}

static int umac_get_link_ksettings(struct net_device *ndev,
				   struct ethtool_link_ksettings *cmd)
{
	phy_ethtool_ksettings_get(ndev->phydev, cmd);
	return 0;
}

static int umac_set_link_ksettings(struct net_device *ndev,
				   const struct ethtool_link_ksettings *cmd)
{
	return phy_ethtool_ksettings_set(ndev->phydev, cmd);
}

static int umac_nway_reset(struct net_device *ndev)
{
	return genphy_restart_aneg(ndev->phydev);
}

static u32 umac_get_link(struct net_device *ndev)
{
	int err;

	err = genphy_update_link(ndev->phydev);
	if (err)
		return ethtool_op_get_link(ndev);

	return ndev->phydev->link;
}

static struct net_device_stats *umac_get_stats(struct net_device *ndev)
{
	return &ndev->stats;
}

static int umac_ioctl(struct net_device *ndev, struct ifreq *ifr, int cmd)
{
	if (!netif_running(ndev))
		return -EINVAL;

	if (!ndev->phydev)
		return -ENODEV;

	return phy_mii_ioctl(ndev->phydev, ifr, cmd);
}

static void umac_set_mac_address(struct net_device *ndev, void *p_addr)
{
	struct umac_priv *umac = netdev_priv(ndev);
	char *addr = (char *)p_addr;
	unsigned int value;

	//update address to register
	value = addr[0] << 8 | addr[1];
	writel(value, umac->base + UMAC_MAC_ADDR_HI);
	value = addr[2] << 8 | addr[3];
	writel(value, umac->base + UMAC_MAC_ADDR_MID);
	value = addr[4] << 8 | addr[5];
	writel(value, umac->base + UMAC_MAC_ADDR_LO);
}

static int umac_eth_mac_addr(struct net_device *ndev, void *p)
{
	int ret;
	struct sockaddr *addr = p;

	ret = eth_prepare_mac_addr_change(ndev, p);
	if (ret < 0)
		return ret;

	eth_commit_mac_addr_change(ndev, p);
	umac_set_mac_address(ndev, addr->sa_data);

	return 0;
}

static void umac_channel_enable(struct umac_priv *umac)
{
	unsigned int value;

	value = readl(umac->base + UMAC_CONFIG_STATUS);
	value |= UMAC_CFG_TXEN | UMAC_CFG_RXEN;
	writel(value, umac->base + UMAC_CONFIG_STATUS);

	// start processing by writing the ring prompt register
	writel(0, umac->base + UMAC_RING_PROMPT);
}

static void umac_channel_disable(struct umac_priv *umac)
{
	writel(0, umac->base + UMAC_CONFIG_STATUS);
}

static int umac_init_ring_discriptor(struct net_device *ndev)
{
	struct umac_priv *umac = netdev_priv(ndev);
	struct platform_device *pdev = umac->pdev;

	struct umac_tx_desc_entry *ptxdesc;
	struct umac_rx_desc_entry *prxdesc;

	unsigned int i;

	if (dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32))) {
		netdev_warn(ndev, "No suitable DMA available\n");
		return -ENOMEM;
	}

	umac->tx_descs = dma_alloc_coherent(&pdev->dev,
					    sizeof(struct umac_tx_descs),
					&umac->tx_descs_dma_addr, GFP_KERNEL);
	if (!umac->tx_descs)
		return -ENOMEM;

	umac->rx_descs = dma_alloc_coherent(&pdev->dev,
					    sizeof(struct umac_rx_descs),
					&umac->rx_descs_dma_addr, GFP_KERNEL);
	if (!umac->rx_descs) {
		dma_free_coherent(&pdev->dev, sizeof(struct umac_tx_descs),
				  umac->tx_descs,
					umac->tx_descs_dma_addr);
		return -ENOMEM;
	}

	for (i = 0; i < UMAC_MAX_TX_DESC_ENTRIES; i++) {
		ptxdesc = &umac->tx_descs->entrylist[i];
		ptxdesc->dmaaddress = cpu_to_le32(umac->tx_descs_dma_addr +
						offsetof(struct umac_tx_descs,
							 framelist[i][0]));
	}

	for (i = 0; i < UMAC_MAX_RX_DESC_ENTRIES; i++) {
		prxdesc = &umac->rx_descs->entrylist[i];
		prxdesc->dmaaddress = cpu_to_le32(umac->rx_descs_dma_addr +
						offsetof(struct umac_rx_descs,
							 framelist[i][0]));
		prxdesc->status = UMAC_RING_ENTRY_HW_OWN;
		prxdesc->count = UMAC_MAX_RX_FRAME_SIZE;
	}

	umac->tx_cur = 0;
	umac->tx_done = 0;
	umac->rx_cur = 0;

	return 0;
}

static int umac_int_phy_init(struct umac_priv *umac)
{
	struct phy_device *phy_dev = umac->int_phy_dev;
	unsigned int value;

	value = phy_read(phy_dev, 0);
	if (value & 0x4000)
		pr_info("Internal PHY loopback is enabled - clearing\n");

	value &= ~0x4000; //disable loopback
	phy_write(phy_dev, 0, value);

	value = phy_read(phy_dev, 0);
	value |= 0x1000; //set aneg enable
	value |= 0x8000; //SW reset
	phy_write(phy_dev, 0, value);

	do {
		value = phy_read(phy_dev, 0);
	} while (value & 0x8000);

	return 0;
}

static int umac_phy_fixup(struct phy_device *phy_dev)
{
	unsigned int value;

	//set phy mode to SGMII to copper
	//set page to 18 by writing 18 to register 22
	phy_write(phy_dev, PHY_88E1514_PAGE_ADDRESS, 18);
	value = phy_read(phy_dev, PHY_88E1514_GENERAL_CONTROL_REG1);
	value &= ~0x07;
	value |= 0x01;
	phy_write(phy_dev, PHY_88E1514_GENERAL_CONTROL_REG1, value);

	//perform mode reset by setting bit 15 in general_control_reg1
	phy_write(phy_dev, PHY_88E1514_GENERAL_CONTROL_REG1, value | 0x8000);

	do {
		value = phy_read(phy_dev, PHY_88E1514_GENERAL_CONTROL_REG1);
	} while (value & 0x8000);

	//after setting the mode, must perform a SW reset
	phy_write(phy_dev, PHY_88E1514_PAGE_ADDRESS, 0); //set page to 0

	value = phy_read(phy_dev, PHY_88E1514_COPPER_CONTROL_REG);
	value |= 0x8000;
	phy_write(phy_dev, PHY_88E1514_COPPER_CONTROL_REG, value);

	do {
		value = phy_read(phy_dev, PHY_88E1514_COPPER_CONTROL_REG);
	} while (value & 0x8000);

	return 0;
}

static int umac_init_hw(struct net_device *ndev)
{
	struct umac_priv *umac = netdev_priv(ndev);
	unsigned int value;

	// initialize tx and rx rings to first entry
	writel(0, umac->base + UMAC_RING_PTR);

	// clear the missed bit
	writel(0, umac->base + UMAC_CLEAR_STATUS);

	// disable checksum generation
	writel(0, umac->base + UMAC_CKSUM_CONFIG);

	// write the ring size register
	value = ((UMAC_RING_SIZE_256 << UMAC_TX_RING_SIZE_SHIFT) &
			UMAC_TX_RING_SIZE_MASK) |
		((UMAC_RING_SIZE_256 << UMAC_RX_RING_SIZE_SHIFT) &
			UMAC_RX_RING_SIZE_MASK);
	writel(value, umac->base + UMAC_RING_SIZE);

	// write rx ring base address
	writel(cpu_to_le32(umac->rx_descs_dma_addr),
	       umac->base + UMAC_RX_RING_ADDR);

	// write tx ring base address
	writel(cpu_to_le32(umac->tx_descs_dma_addr),
	       umac->base + UMAC_TX_RING_ADDR);

	// write burst size
	writel(0x22, umac->base + UMAC_DMA_CONFIG);

	umac_channel_disable(umac);

	// disable clocks and gigabit mode (leave channels disabled)
	value = readl(umac->base + UMAC_CONFIG_STATUS);
	value &= 0xfffff9ff;
	writel(value, umac->base + UMAC_CONFIG_STATUS);
	udelay(2);

	if (umac->use_ncsi) {
		// set correct tx clock
		value &= UMAC_CFG_TX_CLK_EN;
		value &= ~UMAC_CFG_GTX_CLK_EN;
		value &= ~UMAC_CFG_GIGABIT_MODE; // RMII mode
		value |= UMAC_CFG_FULL_DUPLEX; // full duplex
	} else {
		if (ndev->phydev->duplex)
			value |= UMAC_CFG_FULL_DUPLEX;
		else
			value &= ~UMAC_CFG_FULL_DUPLEX;

		if (ndev->phydev->speed == SPEED_1000) {
			value &= ~UMAC_CFG_TX_CLK_EN;
			value |= UMAC_CFG_GTX_CLK_EN;
			value |= UMAC_CFG_GIGABIT_MODE;
		} else {
			value |= UMAC_CFG_TX_CLK_EN;
			value &= ~UMAC_CFG_GTX_CLK_EN;
			value &= ~UMAC_CFG_GIGABIT_MODE;
		}
	}
	writel(value, umac->base + UMAC_CONFIG_STATUS);
	udelay(2);

	umac_channel_enable(umac);

	return 0;
}

static int umac_start_xmit(struct sk_buff *skb, struct net_device *ndev)
{
	struct umac_priv *umac = netdev_priv(ndev);
	struct umac_tx_desc_entry *ptxdesc;
	u8 *pframe;
	unsigned int length;

	ptxdesc = &umac->tx_descs->entrylist[umac->tx_cur];
	pframe = umac->tx_descs->framelist[umac->tx_cur];

	length = skb->len;
	if (length > 1514) {
		netdev_err(ndev, "send data %d bytes > 1514, clamp it to 1514\n",
			   skb->len);
		length = 1514;
	}

	memset(pframe, 0, UMAC_MAX_FRAME_SIZE);
	memcpy(pframe, skb->data, length);

	if (length < ETH_ZLEN)
		length = ETH_ZLEN; // minimum tx byte

	//dev_info(&pdev->dev, "ximt tx_cur=%d pkt len=%d\n",
	//		umac->tx_cur, length);
	ptxdesc->count = length;
	ptxdesc->status = UMAC_RING_ENTRY_HW_OWN;
	ptxdesc->cksumoffset = 0; //disable checksum generation

	umac->tx_cur++;
	if (umac->tx_cur >= UMAC_MAX_TX_DESC_ENTRIES)
		umac->tx_cur = 0;

	// if current tx ring buffer is full, stop the queue
	ptxdesc = &umac->tx_descs->entrylist[umac->tx_cur];
	if (ptxdesc->status & UMAC_RING_ENTRY_HW_OWN)
		netif_stop_queue(ndev);

	// start processing by writing the ring prompt register
	writel(0, umac->base + UMAC_RING_PROMPT);
	dev_kfree_skb(skb);

	return NETDEV_TX_OK;
}

static int umac_rx(struct net_device *ndev, int budget)
{
	struct umac_priv *umac = netdev_priv(ndev);

	struct umac_rx_desc_entry *prxdesc;
	struct sk_buff *skb;

	unsigned int rxlength;
	int rxpktcount = 0;
	u8 *pframe;
	u8 *skb_buf;

	prxdesc = &umac->rx_descs->entrylist[umac->rx_cur];
	pframe = umac->rx_descs->framelist[umac->rx_cur];

	while (!(prxdesc->status & UMAC_RING_ENTRY_HW_OWN)) {
			// do status flag check and statistic the rx packet
		rxlength = prxdesc->count;
		skb = netdev_alloc_skb(ndev, rxlength);
		if (!skb) {
			// run out of memory
			ndev->stats.rx_dropped++;
			return rxpktcount;
		}

		//make 16 bytes aligned for 14 bytes ethernet header
		//skb_reserve(skb, NET_IP_ALIGN);
		skb_buf = skb_put(skb, rxlength);
		//skb_copy_to_linear_data(skb, pframe, rxlength);
		memcpy(skb_buf, pframe, rxlength);

		skb->protocol = eth_type_trans(skb, ndev);
		netif_receive_skb(skb);
		rxpktcount++;

		prxdesc->status = UMAC_RING_ENTRY_HW_OWN;
		prxdesc->count = UMAC_MAX_FRAME_SIZE;

		ndev->stats.rx_packets++;
		ndev->stats.rx_bytes += rxlength;

		//move to next buffer
		umac->rx_cur++;
		if (umac->rx_cur >= UMAC_MAX_RX_DESC_ENTRIES)
			umac->rx_cur = 0;

		if (rxpktcount >= budget)
			break;

		prxdesc = &umac->rx_descs->entrylist[umac->rx_cur];
		pframe = umac->rx_descs->framelist[umac->rx_cur];
	}
	// start processing by writing the ring prompt register
	writel(0, umac->base + UMAC_RING_PROMPT);

	return rxpktcount;
}

static void umac_tx_done(struct net_device *ndev)
{
	struct umac_priv *umac = netdev_priv(ndev);

	unsigned int txptr;
	unsigned int value;
	struct umac_tx_desc_entry *ptxdesc;

	value = readl(umac->base + UMAC_RING_PTR);
	txptr = (value & UMAC_TX_RING_PTR_MASK) >> UMAC_TX_RING_PTR_SHIFT;

	//for each entry between tx_done and tx ring ptr
	//statistic the tx packet
	ptxdesc = &umac->tx_descs->entrylist[umac->tx_done];

	while (!(ptxdesc->status & UMAC_RING_ENTRY_HW_OWN)) {
		if (umac->tx_done == txptr)
			break;
		//dev_info(&pdev->dev,
		//"umac_tx_done(): tx_done=%d pkt len=%d\n",
		//umac->tx_done, ptxdesc->count);

		ndev->stats.tx_packets++;
		ndev->stats.tx_bytes += ptxdesc->count;

		umac->tx_done++;
		if (umac->tx_done >= UMAC_MAX_TX_DESC_ENTRIES)
			umac->tx_done = 0;
		ptxdesc = &umac->tx_descs->entrylist[umac->tx_done];
	}

	//clear tx interrupt
	value = readl(umac->base + UMAC_INTERRUPT);
	value &= ~UMAC_TX_INT;
	writel(value, umac->base + UMAC_INTERRUPT);

	if (netif_queue_stopped(ndev))
		netif_wake_queue(ndev);
}

static void umac_irq_enable(struct umac_priv *umac)
{
	unsigned int value;

	//enable interrupt
	value = readl(umac->base + UMAC_INTERRUPT);
	value |= (UMAC_RX_INTEN | UMAC_TX_INTEN);
	writel(value, umac->base + UMAC_INTERRUPT);
}

static void umac_irq_disable(struct umac_priv *umac)
{
	unsigned int value;

	//clear and disable interrupt
	value = readl(umac->base + UMAC_INTERRUPT);
	value |= (UMAC_RX_INT | UMAC_TX_INT);
	value &= ~(UMAC_RX_INTEN | UMAC_TX_INTEN);
	writel(value, umac->base + UMAC_INTERRUPT);
}

static irqreturn_t umac_interrupt(int irq, void *p_ndev)
{
	struct net_device *ndev = (struct net_device *)p_ndev;
	struct umac_priv *umac = netdev_priv(ndev);

	if (umac->use_ncsi || netif_running(ndev)) {
		umac_irq_disable(umac);
		napi_schedule(&umac->napi);
	}

	return IRQ_HANDLED;
}

static int umac_poll(struct napi_struct *napi, int budget)
{
	struct umac_priv *umac = container_of(napi, struct umac_priv, napi);
	struct net_device *ndev = umac->ndev;
	unsigned int value;
	int rx_done = 0;

	umac_tx_done(ndev);

	rx_done = umac_rx(ndev, budget);

	if (rx_done < budget) {
		napi_complete_done(napi, rx_done);
		//clear rx interrupt
		value = readl(umac->base + UMAC_INTERRUPT);
		value &= ~UMAC_RX_INT;
		writel(value, umac->base + UMAC_INTERRUPT);

		//enable interrupt
		umac_irq_enable(umac);
	}

	return rx_done;
}

static int umac_open(struct net_device *ndev)
{
	struct umac_priv *umac = netdev_priv(ndev);
	int err;

	if (request_irq(ndev->irq, umac_interrupt, 0x0, ndev->name, ndev)) {
		netdev_err(ndev, "failed to register irq\n");
		return -EAGAIN;
	}

	umac_init_ring_discriptor(ndev);
	umac_init_hw(ndev);

	if (umac->use_ncsi)
		netif_carrier_on(ndev);
	else
		phy_start(ndev->phydev);

	napi_enable(&umac->napi);
	netif_start_queue(ndev);
	umac_irq_enable(umac);

	if (umac->use_ncsi) {
		err = ncsi_start_dev(umac->ncsidev);
		if (err) {
			netdev_err(ndev, "failed to start ncsi\n");
			free_irq(ndev->irq, ndev);
			return err;
		}
	}

	netdev_info(ndev, "%s is OPENED\n", ndev->name);
	return 0;
}

static int umac_stop(struct net_device *ndev)
{
	struct umac_priv *umac = netdev_priv(ndev);
	struct platform_device *pdev = umac->pdev;

	dma_free_coherent(&pdev->dev, sizeof(struct umac_tx_descs),
			  umac->tx_descs, umac->tx_descs_dma_addr);
	dma_free_coherent(&pdev->dev, sizeof(struct umac_rx_descs),
			  umac->rx_descs, umac->rx_descs_dma_addr);
	netif_stop_queue(ndev);

	if (umac->use_ncsi)
		ncsi_stop_dev(umac->ncsidev);
	else
		phy_stop(ndev->phydev);
	umac_irq_disable(umac);
	umac_channel_disable(umac);
	napi_disable(&umac->napi);

	free_irq(ndev->irq, ndev);

	return 0;
}

static const struct ethtool_ops umac_ethtool_ops = {
	.get_ts_info	= ethtool_op_get_ts_info,
	.get_link_ksettings	= umac_get_link_ksettings,
	.set_link_ksettings	= umac_set_link_ksettings,
	.get_drvinfo	= umac_get_drvinfo,
	.nway_reset	= umac_nway_reset,
	.get_link	= umac_get_link,
};

static const struct net_device_ops umac_netdev_ops = {
	.ndo_open		= umac_open,
	.ndo_stop		= umac_stop,
	.ndo_start_xmit		= umac_start_xmit,
	.ndo_get_stats		= umac_get_stats,
	.ndo_do_ioctl		= umac_ioctl,
	//.ndo_change_mtu		= eth_change_mtu,
	.ndo_validate_addr	= eth_validate_addr,
	.ndo_set_mac_address	= umac_eth_mac_addr,
};

static int umac_init_mac_address(struct net_device *ndev)
{
	struct umac_priv *umac = netdev_priv(ndev);
	struct platform_device *pdev = umac->pdev;

	const void *of_mac_addr;
	char addr[ETH_ALEN];

	of_mac_addr = of_get_mac_address(pdev->dev.of_node);
	if (of_mac_addr)
		memcpy(addr, of_mac_addr, ETH_ALEN);

	if (is_valid_ether_addr(addr)) {
		ether_addr_copy(ndev->dev_addr, addr);
		netdev_info(ndev,
			    "Read MAC address %pM from DTB\n", ndev->dev_addr);
	} else {
		eth_hw_addr_random(ndev);
		netdev_info(ndev, "Generated random MAC address %pM\n",
			    ndev->dev_addr);
	}

	memcpy(ndev->dev_addr, &addr, ETH_ALEN);
	umac_set_mac_address(ndev, addr);

	return 0;
}

static void umac_ncsi_handler(struct ncsi_dev *ncsidev)
{
	if (unlikely(ncsidev->state != ncsi_dev_state_functional))
		return;

	netdev_info(ncsidev->dev, "NCSI interface %s\n",
		    ncsidev->link_up ? "up" : "down");
}

static void umac_adjust_link(struct net_device *ndev)
{
	struct umac_priv *umac = netdev_priv(ndev);
	int value;

	if (ndev->phydev->link) {
		// disable both clock
		value = readl(umac->base + UMAC_CONFIG_STATUS);
		value &= 0xfffff9ff;
		writel(value, umac->base + UMAC_CONFIG_STATUS);
		udelay(2);

		if (ndev->phydev->duplex)
			value |= UMAC_CFG_FULL_DUPLEX;
		else
			value &= ~UMAC_CFG_FULL_DUPLEX;

		switch (ndev->phydev->speed) {
		case SPEED_1000:
			value &= ~UMAC_CFG_TX_CLK_EN;
			value |= UMAC_CFG_GTX_CLK_EN;
			value |= UMAC_CFG_GIGABIT_MODE;
			break;
		case SPEED_100:
			value |= UMAC_CFG_TX_CLK_EN;
			value &= ~UMAC_CFG_GTX_CLK_EN;
			value &= ~UMAC_CFG_GIGABIT_MODE;
			break;
		}
		//update duplex and gigabit_mode to umac
		writel(value, umac->base + UMAC_CONFIG_STATUS);
		udelay(2);

		netif_carrier_on(ndev);
	} else {
		// disable both clock
		value = readl(umac->base + UMAC_CONFIG_STATUS);
		value &= 0xfffff9ff;
		writel(value, umac->base + UMAC_CONFIG_STATUS);
		udelay(2);

		value &= ~UMAC_CFG_FULL_DUPLEX;
		value &= ~UMAC_CFG_GTX_CLK_EN;
		value &= ~UMAC_CFG_GIGABIT_MODE;
		value |= UMAC_CFG_TX_CLK_EN;
		writel(value, umac->base + UMAC_CONFIG_STATUS);
		udelay(2);

		netif_carrier_off(ndev);
	}
}

static int umac_setup_phy(struct net_device *ndev)
{
	struct umac_priv *umac = netdev_priv(ndev);
	struct platform_device *pdev = umac->pdev;
	struct device_node *phy_handle;
	phy_interface_t interface;
	int ret;
	int err;

	phy_handle = of_parse_phandle(pdev->dev.of_node, "int-phy-handle", 0);
	if (phy_handle) {
		umac->int_phy_dev = of_phy_find_device(phy_handle);
		if (!umac->int_phy_dev)
			return -ENODEV;

		umac_int_phy_init(umac);
	}

	phy_handle = of_parse_phandle(pdev->dev.of_node, "phy-handle", 0);
	if (phy_handle) {
		// register the phy board fixup
		ret = phy_register_fixup_for_uid(0x01410dd1, 0xffffffff,
						 umac_phy_fixup);
		if (ret)
			pr_info("cannot register phy board fixup\n");

		err = of_get_phy_mode(phy_handle, &interface);
		if (err)
			interface = PHY_INTERFACE_MODE_NA;

		umac->phy_dev = of_phy_connect(ndev, phy_handle,
			&umac_adjust_link, 0, interface);

		if (!umac->phy_dev)
			return -ENODEV;
	}

	/*
	 * If the specified phy-handle has a fixed-link declaration, use the
	 * fixed-link properties to set the configuration for the PHY
	 */
	if (of_phy_is_fixed_link(phy_handle)) {
		struct device_node *fixed_link_node = of_get_child_by_name(phy_handle, "fixed-link");

		netdev_info(ndev, "Setting fixed link configuration for PHY\n");
		if (of_property_read_u32(fixed_link_node, "speed",
				&umac->phy_dev->speed)) {
			netdev_err(ndev, "Invalid fixed-link specified.\n");
			return -EINVAL;
		}
		umac->phy_dev->duplex = of_property_read_bool(fixed_link_node,
			"full-duplex");
		umac->phy_dev->pause = of_property_read_bool(fixed_link_node, "pause");
		umac->phy_dev->asym_pause = of_property_read_bool(fixed_link_node,
			"asym-pause");
		umac->phy_dev->autoneg = AUTONEG_DISABLE;
		__clear_bit(ETHTOOL_LINK_MODE_Autoneg_BIT, umac->phy_dev->advertising);
	}

	return 0;
}

static int umac_probe(struct platform_device *pdev)
{
	struct umac_priv *umac;
	struct net_device *ndev;
	struct resource *res;
	int ret = 0;

	ndev = alloc_etherdev(sizeof(struct umac_priv));
	if (!ndev)
		return -ENOMEM;

	SET_NETDEV_DEV(ndev, &pdev->dev);

	umac = netdev_priv(ndev);
	umac->pdev = pdev;
	umac->ndev = ndev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		netdev_err(ndev, "failed to get I/O memory\n");
		free_netdev(ndev);
		return -ENXIO;
	}

	umac->base = devm_ioremap_resource(&pdev->dev, res);
	if (!umac->base) {
		netdev_err(ndev, "failed to remap I/O memory\n");
		free_netdev(ndev);
		return -EBUSY;
	}

	ndev->irq = platform_get_irq(pdev, 0);
	if (ndev->irq < 0) {
		netdev_err(ndev, "failed to get irq\n");
		free_netdev(ndev);
		return -ENXIO;
	}

	platform_set_drvdata(pdev, ndev);

	ndev->netdev_ops = &umac_netdev_ops;
	ndev->ethtool_ops = &umac_ethtool_ops;

	umac_init_mac_address(ndev);
	umac_channel_disable(umac);
	ret = umac_setup_phy(ndev);
	if (ret != 0) {
		netdev_err(ndev, "failed to setup phy ret=%d\n", ret);
		return -ENODEV;
	}

	umac->use_ncsi = false;
	if (of_get_property(pdev->dev.of_node, "use-ncsi", NULL)) {
		if (!IS_ENABLED(CONFIG_NET_NCSI)) {
			netdev_err(ndev, "NCSI stack not enabled\n");
			free_netdev(ndev);
			return 0;
		}

		dev_info(&pdev->dev, "Using NCSI interface\n");
		umac->use_ncsi = true;
		umac->ncsidev = ncsi_register_dev(ndev, umac_ncsi_handler);
		if (!umac->ncsidev) {
			free_netdev(ndev);
			return -ENODEV;
		}
	}

	netif_napi_add(ndev, &umac->napi, umac_poll, 64);
	ret = register_netdev(ndev);
	if (ret != 0) {
		netdev_err(ndev, "failed to register UMAC ret=%d\n", ret);
		netif_napi_del(&umac->napi);
		free_netdev(ndev);
		return -ENODEV;
	}

	return ret;
}

static int umac_remove(struct platform_device *pdev)
{
	struct net_device *ndev = platform_get_drvdata(pdev);
	struct umac_priv *umac = netdev_priv(ndev);

	unregister_netdev(ndev);
	iounmap(umac->base);
	free_netdev(ndev);
	return 0;
}

static const struct of_device_id umac_of_matches[] = {
	{ .compatible = "hpe, gxp-umac", },
	{},
};
MODULE_DEVICE_TABLE(of, umac_of_matches);

static struct platform_driver umac_driver = {
	.driver	= {
		.name    = "gxp-umac",
		.of_match_table = of_match_ptr(umac_of_matches),
	},
	.probe   = umac_probe,
	.remove  = umac_remove,
};

module_platform_driver(umac_driver);

MODULE_AUTHOR("Gilbert Chen <gilbert.chen@hpe.com");
MODULE_DESCRIPTION("HPE GXP UMAC driver");
