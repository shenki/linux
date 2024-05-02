// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2020, Intel Corporation.

#include <linux/bitfield.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/mfd/syscon.h>
#include <linux/miscdevice.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/pci.h>
#include <linux/poll.h>
#include <linux/ptr_ring.h>
#include <linux/regmap.h>
#include <linux/reset.h>
#include <linux/slab.h>
#include <linux/swab.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>
#include <uapi/linux/aspeed-mctp.h>

/* AST2600 MCTP Controller registers */
#define ASPEED_MCTP_CTRL	0x000
#define  TX_CMD_TRIGGER		BIT(0)
#define  RX_CMD_READY		BIT(4)
#define  MATCHING_EID		BIT(9)

#define ASPEED_MCTP_TX_CMD	0x004
#define ASPEED_MCTP_RX_CMD	0x008

#define ASPEED_MCTP_INT_STS	0x00c
#define ASPEED_MCTP_INT_EN	0x010
#define  TX_CMD_SENT_INT	BIT(0)
#define  TX_CMD_LAST_INT	BIT(1)
#define  TX_CMD_WRONG_INT	BIT(2)
#define  RX_CMD_RECEIVE_INT	BIT(8)
#define  RX_CMD_NO_MORE_INT	BIT(9)

#define ASPEED_MCTP_EID		0x014
#define ASPEED_MCTP_OBFF_CTRL	0x018

#define ASPEED_MCTP_ENGINE_CTRL		0x01c
#define  TX_MAX_PAYLOAD_SIZE_SHIFT	0
#define  TX_MAX_PAYLOAD_SIZE_MASK	GENMASK(1, TX_MAX_PAYLOAD_SIZE_SHIFT)
#define  TX_MAX_PAYLOAD_SIZE(x) \
	(((x) << TX_MAX_PAYLOAD_SIZE_SHIFT) & TX_MAX_PAYLOAD_SIZE_MASK)
#define  RX_MAX_PAYLOAD_SIZE_SHIFT	4
#define  RX_MAX_PAYLOAD_SIZE_MASK	GENMASK(5, RX_MAX_PAYLOAD_SIZE_SHIFT)
#define  RX_MAX_PAYLOAD_SIZE(x) \
	(((x) << RX_MAX_PAYLOAD_SIZE_SHIFT) & RX_MAX_PAYLOAD_SIZE_MASK)
#define  FIFO_LAYOUT_SHIFT		8
#define  FIFO_LAYOUT_MASK		GENMASK(9, FIFO_LAYOUT_SHIFT)
#define  FIFO_LAYOUT(x) \
	(((x) << FIFO_LAYOUT_SHIFT) & FIFO_LAYOUT_MASK)

#define ASPEED_MCTP_RX_BUF_ADDR		0x020
#define ASPEED_MCTP_RX_BUF_SIZE		0x024
#define ASPEED_MCTP_RX_BUF_RD_PTR	0x028
#define  UPDATE_RX_RD_PTR		BIT(31)
#define  RX_BUFFER_RD_PTR		GENMASK(11, 0)
#define ASPEED_MCTP_RX_BUF_WR_PTR	0x02c
#define  RX_BUFFER_WR_PTR		GENMASK(11, 0)

#define ASPEED_MCTP_TX_BUF_ADDR		0x030
#define ASPEED_MCTP_TX_BUF_SIZE		0x034
#define ASPEED_MCTP_TX_BUF_RD_PTR	0x038
#define  UPDATE_TX_RD_PTR		BIT(31)
#define  TX_BUFFER_RD_PTR		GENMASK(11, 0)
#define ASPEED_MCTP_TX_BUF_WR_PTR	0x03c
#define  TX_BUFFER_WR_PTR		GENMASK(11, 0)

#define ADDR_LEN	(BIT(26) - 1)
#define DATA_ADDR(x)	(((x) >> 4) & ADDR_LEN)

/* TX command */
#define TX_LAST_CMD		BIT(31)
#define TX_DATA_ADDR_SHIFT	4
#define TX_DATA_ADDR_MASK	GENMASK(30, TX_DATA_ADDR_SHIFT)
#define TX_DATA_ADDR(x) \
	((DATA_ADDR(x) << TX_DATA_ADDR_SHIFT) & TX_DATA_ADDR_MASK)
#define TX_RESERVED_1_MASK	GENMASK(1, 0) /* must be 1 */
#define TX_RESERVED_1		1
#define TX_STOP_AFTER_CMD	BIT(16)
#define TX_INTERRUPT_AFTER_CMD	BIT(15)
#define TX_PACKET_SIZE_SHIFT	2
#define TX_PACKET_SIZE_MASK	GENMASK(12, TX_PACKET_SIZE_SHIFT)
#define TX_PACKET_SIZE(x) \
	(((x) << TX_PACKET_SIZE_SHIFT) & TX_PACKET_SIZE_MASK)
#define TX_RESERVED_0_MASK	GENMASK(1, 0) /* MBZ */
#define TX_RESERVED_0		0

/* RX command */
#define RX_INTERRUPT_AFTER_CMD	BIT(2)
#define RX_DATA_ADDR_SHIFT	4
#define RX_DATA_ADDR_MASK	GENMASK(30, RX_DATA_ADDR_SHIFT)
#define RX_DATA_ADDR(x) \
	((DATA_ADDR(x) << RX_DATA_ADDR_SHIFT) & RX_DATA_ADDR_MASK)

/* Buffer sizes */
#define TX_PACKET_COUNT 64
#define RX_PACKET_COUNT 64
#define TX_MAX_PACKET_COUNT SZ_4K
#define RX_MAX_PACKET_COUNT SZ_4K

#define TX_CMD_COUNT TX_PACKET_COUNT
/* XXX: We need twice the amount programmed in RX_BUF_SIZE */
#define RX_CMD_COUNT (2 * RX_PACKET_COUNT)

/* PCIe Host Controller registers */
#define ASPEED_PCIE_MISC_STS_1 0x0c4

/* PCI address definitions */
#define PCI_DEV_NUM_MASK	GENMASK(4, 0)
#define PCI_BUS_NUM_SHIFT	5
#define PCI_BUS_NUM_MASK	GENMASK(12, PCI_BUS_NUM_SHIFT)
#define GET_PCI_DEV_NUM(x)	((x) & PCI_DEV_NUM_MASK)
#define GET_PCI_BUS_NUM(x)	(((x) & PCI_BUS_NUM_MASK) >> PCI_BUS_NUM_SHIFT)

/* FIXME: ast2600 supports variable max transmission unit */
#define ASPEED_MCTP_MTU 64

/* PCIe header definitions */
#define PCIE_VDM_HDR_REQUESTER_BDF_DW 1
#define PCIE_VDM_HDR_REQUESTER_BDF_MASK GENMASK(31, 16)

#define PCIE_VDM_HDR_SIZE_DW (ASPEED_MCTP_PCIE_VDM_HDR_SIZE / 4)
#define PCIE_VDM_DATA_SIZE_DW (ASPEED_MCTP_MTU / 4)

#define PCIE_MCTP_MIN_PACKET_SIZE (ASPEED_MCTP_PCIE_VDM_HDR_SIZE + 4)

struct mctp_pcie_packet_data {
	u32 hdr[PCIE_VDM_HDR_SIZE_DW];
	u32 payload[PCIE_VDM_DATA_SIZE_DW];
};

struct mctp_pcie_packet {
	struct mctp_pcie_packet_data data;
	u32 size;
};

struct aspeed_mctp_tx_cmd {
	u32 tx_lo;
	u32 tx_hi;
};

struct mctp_buffer {
	void *vaddr;
	dma_addr_t dma_handle;
};

struct mctp_channel {
	struct mctp_buffer data;
	struct mctp_buffer cmd;
	struct tasklet_struct tasklet;
	u32 rd_ptr;
	u32 wr_ptr;
};

struct aspeed_mctp {
	struct device *dev;
	struct regmap *map;
	struct reset_control *reset;
	struct mctp_channel tx;
	struct mctp_channel rx;
	struct list_head clients;
	struct mctp_client *default_client;
	spinlock_t clients_lock; /* to protect clients list operations */
	struct {
		struct regmap *map;
		struct delayed_work rst_dwork;
		bool need_uevent;
		u16 bdf;
	} pcie;
	bool wa_runaway_rx;
};

struct mctp_client {
	struct kref ref;
	struct aspeed_mctp *priv;
	struct ptr_ring tx_queue;
	struct ptr_ring rx_queue;
	struct list_head link;
	wait_queue_head_t wait_queue;
};

#define TX_CMD_BUF_SIZE \
	PAGE_ALIGN(TX_CMD_COUNT * sizeof(struct aspeed_mctp_tx_cmd))
#define TX_DATA_BUF_SIZE \
	 PAGE_ALIGN(TX_PACKET_COUNT * sizeof(struct mctp_pcie_packet_data))
#define RX_CMD_BUF_SIZE PAGE_ALIGN(RX_CMD_COUNT * sizeof(u32))
#define RX_DATA_BUF_SIZE \
	PAGE_ALIGN(RX_PACKET_COUNT * sizeof(struct mctp_pcie_packet_data))

struct kmem_cache *packet_cache;

static void *packet_alloc(gfp_t flags)
{
	return kmem_cache_alloc(packet_cache, flags);
}

static void packet_free(void *packet)
{
	kmem_cache_free(packet_cache, packet);
}

static void aspeed_mctp_rx_trigger(struct mctp_channel *rx)
{
	struct aspeed_mctp *priv = container_of(rx, typeof(*priv), rx);

	/*
	 * Even though rx_buf_addr doesn't change, if we don't do the write
	 * here, the HW doesn't trigger RX. We're also clearing the
	 * RX_CMD_READY bit, otherwise we're observing a rare case where
	 * trigger isn't registered by the HW, and we're ending up with stuck
	 * HW (not reacting to wr_ptr writes).
	 * Also, note that we're writing 0 as wr_ptr. If we're writing other
	 * value, the HW behaves in a bizarre way that's hard to explain...
	 */
	regmap_update_bits(priv->map, ASPEED_MCTP_CTRL, RX_CMD_READY, 0);
	regmap_write(priv->map, ASPEED_MCTP_RX_BUF_ADDR, rx->cmd.dma_handle);
	regmap_write(priv->map, ASPEED_MCTP_RX_BUF_WR_PTR, 0);
	regmap_update_bits(priv->map, ASPEED_MCTP_CTRL, RX_CMD_READY,
			   RX_CMD_READY);
}

static void aspeed_mctp_tx_trigger(struct mctp_channel *tx)
{
	struct aspeed_mctp *priv = container_of(tx, typeof(*priv), tx);
	struct aspeed_mctp_tx_cmd *last_cmd;

	last_cmd = (struct aspeed_mctp_tx_cmd *)tx->cmd.vaddr +
		   (tx->wr_ptr - 1) % TX_PACKET_COUNT;
	last_cmd->tx_hi |= TX_LAST_CMD;
	last_cmd->tx_lo |= TX_STOP_AFTER_CMD | TX_INTERRUPT_AFTER_CMD;

	regmap_write(priv->map, ASPEED_MCTP_TX_BUF_ADDR,
		     tx->cmd.dma_handle);
	regmap_write(priv->map, ASPEED_MCTP_TX_BUF_WR_PTR, tx->wr_ptr);
	regmap_update_bits(priv->map, ASPEED_MCTP_CTRL, TX_CMD_TRIGGER,
			   TX_CMD_TRIGGER);
}

static void aspeed_mctp_emit_tx_cmd(struct mctp_channel *tx,
				    struct mctp_pcie_packet *packet)
{
	struct aspeed_mctp_tx_cmd *tx_cmd =
		(struct aspeed_mctp_tx_cmd *)tx->cmd.vaddr + tx->wr_ptr;
	u32 packet_sz_dw = packet->size / sizeof(u32) -
		sizeof(packet->data.hdr) / sizeof(u32);
	u32 offset = tx->wr_ptr * sizeof(packet->data);

	memcpy((u8 *)tx->data.vaddr + offset, &packet->data,
	       sizeof(packet->data));

	tx_cmd->tx_lo = TX_PACKET_SIZE(packet_sz_dw);
	tx_cmd->tx_hi = TX_RESERVED_1;
	tx_cmd->tx_hi |= TX_DATA_ADDR(tx->data.dma_handle + offset);

	tx->wr_ptr++;
}

static struct mctp_client *aspeed_mctp_client_alloc(struct aspeed_mctp *priv)
{
	struct mctp_client *client;

	client = kzalloc(sizeof(*client), GFP_KERNEL);
	if (!client)
		goto out;

	kref_init(&client->ref);
	client->priv = priv;
	ptr_ring_init(&client->tx_queue, TX_PACKET_COUNT, GFP_KERNEL);
	ptr_ring_init(&client->rx_queue, RX_PACKET_COUNT, GFP_ATOMIC);

out:
	return client;
}

static void aspeed_mctp_client_free(struct kref *ref)
{
	struct mctp_client *client = container_of(ref, typeof(*client), ref);

	ptr_ring_cleanup(&client->rx_queue, &packet_free);
	ptr_ring_cleanup(&client->tx_queue, &packet_free);

	kfree(client);
}

static void aspeed_mctp_client_get(struct mctp_client *client)
{
	lockdep_assert_held(&client->priv->clients_lock);

	kref_get(&client->ref);
}

static void aspeed_mctp_client_put(struct mctp_client *client)
{
	kref_put(&client->ref, &aspeed_mctp_client_free);
}

static void aspeed_mctp_dispatch_packet(struct aspeed_mctp *priv,
					struct mctp_pcie_packet *packet)
{
	struct mctp_client *client;
	int ret;

	spin_lock(&priv->clients_lock);

	client = priv->default_client;

	if (client)
		aspeed_mctp_client_get(client);

	spin_unlock(&priv->clients_lock);

	if (client) {
		ret = ptr_ring_produce(&client->rx_queue, packet);
		if (ret) {
			dev_warn(priv->dev, "Failed to produce RX packet\n");
			packet_free(packet);
		} else {
			wake_up_all(&client->wait_queue);
		}
		aspeed_mctp_client_put(client);
	} else {
		dev_dbg(priv->dev, "Failed to dispatch RX packet\n");
		packet_free(packet);
	}
}

static void aspeed_mctp_tx_tasklet(unsigned long data)
{
	struct mctp_channel *tx = (struct mctp_channel *)data;
	struct aspeed_mctp *priv = container_of(tx, typeof(*priv), tx);
	u32 rd_ptr = READ_ONCE(tx->rd_ptr);
	struct mctp_client *client;
	bool trigger = false;

	/* we're called while there's still TX in progress */
	if (rd_ptr == 0 && tx->wr_ptr != 0)
		return;

	/* last tx ended up with buffer size, meaning we now restart from 0 */
	if (rd_ptr == TX_PACKET_COUNT) {
		WRITE_ONCE(tx->rd_ptr, 0);
		tx->wr_ptr = 0;
	}

	spin_lock(&priv->clients_lock);

	list_for_each_entry(client, &priv->clients, link) {
		while (tx->wr_ptr < TX_PACKET_COUNT) {
			struct mctp_pcie_packet *packet;

			packet = __ptr_ring_peek(&client->tx_queue);
			if (!packet)
				break;

			ptr_ring_consume(&client->tx_queue);
			aspeed_mctp_emit_tx_cmd(tx, packet);
			packet_free(packet);
			trigger = true;
		}
	}

	spin_unlock(&priv->clients_lock);

	if (trigger)
		aspeed_mctp_tx_trigger(tx);
}

static void aspeed_mctp_rx_tasklet(unsigned long data)
{
	struct mctp_channel *rx = (struct mctp_channel *)data;
	struct aspeed_mctp *priv = container_of(rx, typeof(*priv), rx);
	struct mctp_pcie_packet *rx_packet;
	struct mctp_pcie_packet_data *rx_buf;
	int i;
	u32 *hdr;

	/*
	 * XXX: Using rd_ptr obtained from HW is unreliable so we need to
	 * maintain the state of buffer on our own by peeking into the buffer
	 * and checking where the packet was written.
	 */
	rx_buf = (struct mctp_pcie_packet_data *)rx->data.vaddr;
	hdr = (u32 *)&rx_buf[rx->wr_ptr];

	while (*hdr != 0) {
		rx_packet = packet_alloc(GFP_ATOMIC);
		if (!rx_packet) {
			dev_err(priv->dev, "Failed to allocate RX packet\n");
			break;
		}
		/*
		 * XXX: HW outputs VDM header in little endian, swap to present
		 * the whole packet to userspace in network order
		 */
		for (i = 0; i < PCIE_VDM_HDR_SIZE_DW; i++)
			rx_packet->data.hdr[i] = swab32(hdr[i]);

		memcpy(&rx_packet->data.payload, hdr + PCIE_VDM_HDR_SIZE_DW,
		       sizeof(rx_packet->data) - sizeof(rx_packet->data.hdr));

		*hdr = 0;
		rx->wr_ptr = (rx->wr_ptr + 1) % RX_PACKET_COUNT;
		hdr = (u32 *)&rx_buf[rx->wr_ptr];

		aspeed_mctp_dispatch_packet(priv, rx_packet);
	}
	if (rx->wr_ptr == 0)
		aspeed_mctp_rx_trigger(rx);
}

static void aspeed_mctp_rx_chan_init(struct mctp_channel *rx)
{
	struct aspeed_mctp *priv = container_of(rx, typeof(*priv), rx);
	u32 *rx_cmd = (u32 *)rx->cmd.vaddr;
	u32 data_size = sizeof(struct mctp_pcie_packet_data);
	u32 packet_offset, i;

	/*
	 * XXX: Sporadically, after BMC reboot, MCTP hardware starts processing
	 * commands outside of RX command buffer range defined in registers.
	 * Generally, it executes commands from memory until it reaches twice
	 * the expected buffer size. After a single "overflow" it goes back to
	 * defined range.
	 * To handle this case (and avoid losing initial packets), we need to
	 * insert additional RX commands that point to our data buffer.
	 */
	for (i = 0; i < RX_CMD_COUNT; i++) {
		packet_offset = data_size * (i % RX_PACKET_COUNT);
		*rx_cmd = RX_DATA_ADDR(rx->data.dma_handle + packet_offset);
		*rx_cmd |= RX_INTERRUPT_AFTER_CMD;
		rx_cmd++;
	}
	regmap_write(priv->map, ASPEED_MCTP_RX_BUF_SIZE, RX_PACKET_COUNT);
}

static void aspeed_mctp_tx_chan_init(struct mctp_channel *tx)
{
	struct aspeed_mctp *priv = container_of(tx, typeof(*priv), tx);

	regmap_write(priv->map, ASPEED_MCTP_TX_BUF_SIZE, TX_PACKET_COUNT);
	regmap_write(priv->map, ASPEED_MCTP_TX_BUF_WR_PTR, 0);
}

static struct mctp_client *aspeed_mctp_create_client(struct aspeed_mctp *priv)
{
	struct mctp_client *client;

	client = aspeed_mctp_client_alloc(priv);
	if (!client)
		return NULL;

	init_waitqueue_head(&client->wait_queue);

	spin_lock_bh(&priv->clients_lock);
	list_add_tail(&client->link, &priv->clients);
	spin_unlock_bh(&priv->clients_lock);

	/*
	 * kick the tasklet to trigger rx
	 * bh_disable/enable is just to make sure that the tasklet gets
	 * scheduled immediately in process context without any unnecessary
	 * delay
	 */
	local_bh_disable();
	tasklet_hi_schedule(&priv->rx.tasklet);
	local_bh_enable();

	return client;
}

static int aspeed_mctp_open(struct inode *inode, struct file *file)
{
	struct miscdevice *misc = file->private_data;
	struct platform_device *pdev = to_platform_device(misc->parent);
	struct aspeed_mctp *priv = platform_get_drvdata(pdev);
	struct mctp_client *client;

	client = aspeed_mctp_create_client(priv);
	if (!client)
		return -ENOMEM;

	file->private_data = client;

	return 0;
}

static void aspeed_mctp_delete_client(struct mctp_client *client)
{
	struct aspeed_mctp *priv = client->priv;

	spin_lock_bh(&priv->clients_lock);

	list_del(&client->link);

	if (priv->default_client == client)
		priv->default_client = NULL;

	spin_unlock_bh(&priv->clients_lock);

	/* Disable the tasklet to appease lockdep */
	local_bh_disable();
	aspeed_mctp_client_put(client);
	local_bh_enable();
}

static int aspeed_mctp_release(struct inode *inode, struct file *file)
{
	struct mctp_client *client = file->private_data;

	aspeed_mctp_delete_client(client);

	return 0;
}

static ssize_t aspeed_mctp_read(struct file *file, char __user *buf,
				size_t count, loff_t *ppos)
{
	struct mctp_client *client = file->private_data;
	struct aspeed_mctp *priv = client->priv;
	struct mctp_pcie_packet *rx_packet;
	size_t packet_sz = sizeof(rx_packet->data);

	if (count < PCIE_MCTP_MIN_PACKET_SIZE)
		return -EINVAL;

	if (priv->pcie.bdf == 0)
		return -EIO;

	if (count > packet_sz)
		count = packet_sz;

	rx_packet = ptr_ring_consume_bh(&client->rx_queue);
	if (!rx_packet)
		return -EAGAIN;

	if (copy_to_user(buf, &rx_packet->data, count)) {
		dev_err(priv->dev, "copy to user failed\n");
		count = -EFAULT;
	}

	packet_free(rx_packet);

	return count;
}

static ssize_t aspeed_mctp_write(struct file *file, const char __user *buf,
				 size_t count, loff_t *ppos)
{
	struct mctp_client *client = file->private_data;
	struct aspeed_mctp *priv = client->priv;
	struct mctp_pcie_packet *tx_packet;
	u16 bdf = priv->pcie.bdf;
	int ret, i;

	if (count < PCIE_MCTP_MIN_PACKET_SIZE)
		return -EINVAL;

	if (count > sizeof(tx_packet->data))
		return -ENOSPC;

	if (bdf == 0)
		return -EIO;

	tx_packet = packet_alloc(GFP_KERNEL);
	if (!tx_packet) {
		ret = -ENOMEM;
		goto out;
	}

	if (copy_from_user(&tx_packet->data, buf, count)) {
		dev_err(priv->dev, "copy from user failed\n");
		ret = -EFAULT;
		goto out_packet;
	}
	tx_packet->size = count;

	/* Update PCIE header with requester BDF */
	be32p_replace_bits(&tx_packet->data.hdr[PCIE_VDM_HDR_REQUESTER_BDF_DW],
			   bdf, PCIE_VDM_HDR_REQUESTER_BDF_MASK);

	/*
	 * XXX: HW expects VDM header in little endian, swap to let
	 * userspace use network order for the whole packet
	 */
	for (i = 0; i < PCIE_VDM_HDR_SIZE_DW; i++)
		tx_packet->data.hdr[i] = swab32(tx_packet->data.hdr[i]);

	ret = ptr_ring_produce_bh(&client->tx_queue, tx_packet);
	if (ret)
		goto out_packet;

	tasklet_hi_schedule(&priv->tx.tasklet);

	return count;

out_packet:
	packet_free(tx_packet);
out:
	return ret;
}

static int aspeed_mctp_register_default_handler(struct mctp_client *client)
{
	struct aspeed_mctp *priv = client->priv;
	int ret = 0;

	spin_lock_bh(&priv->clients_lock);

	if (!priv->default_client)
		priv->default_client = client;
	else if (priv->default_client != client)
		ret = -EBUSY;

	spin_unlock_bh(&priv->clients_lock);

	return ret;
}

static int
aspeed_mctp_filter_eid(struct aspeed_mctp *priv, void __user *userbuf)
{
	struct aspeed_mctp_filter_eid eid;

	if (copy_from_user(&eid, userbuf, sizeof(eid))) {
		dev_err(priv->dev, "copy from user failed\n");
		return -EFAULT;
	}

	if (eid.enable) {
		regmap_write(priv->map, ASPEED_MCTP_EID, eid.eid);
		regmap_update_bits(priv->map, ASPEED_MCTP_CTRL,
				   MATCHING_EID, MATCHING_EID);
	} else {
		regmap_update_bits(priv->map, ASPEED_MCTP_CTRL,
				   MATCHING_EID, 0);
	}
	return 0;
}

static int aspeed_mctp_get_bdf(struct aspeed_mctp *priv, void __user *userbuf)
{
	struct aspeed_mctp_get_bdf bdf = { priv->pcie.bdf };

	if (copy_to_user(userbuf, &bdf, sizeof(bdf))) {
		dev_err(priv->dev, "copy to user failed\n");
		return -EFAULT;
	}
	return 0;
}

static int
aspeed_mctp_get_medium_id(struct aspeed_mctp *priv, void __user *userbuf)
{
	struct aspeed_mctp_get_medium_id id = { 0x09 }; /* PCIe revision 2.0 */

	if (copy_to_user(userbuf, &id, sizeof(id))) {
		dev_err(priv->dev, "copy to user failed\n");
		return -EFAULT;
	}
	return 0;
}

static int
aspeed_mctp_get_mtu(struct aspeed_mctp *priv, void __user *userbuf)
{
	struct aspeed_mctp_get_mtu id = { ASPEED_MCTP_MTU };

	if (copy_to_user(userbuf, &id, sizeof(id))) {
		dev_err(priv->dev, "copy to user failed\n");
		return -EFAULT;
	}
	return 0;
}

static long
aspeed_mctp_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct mctp_client *client = file->private_data;
	struct aspeed_mctp *priv = client->priv;
	void __user *userbuf = (void __user *)arg;
	int ret;

	switch (cmd) {
	case ASPEED_MCTP_IOCTL_FILTER_EID:
		ret = aspeed_mctp_filter_eid(priv, userbuf);
	break;

	case ASPEED_MCTP_IOCTL_GET_BDF:
		ret = aspeed_mctp_get_bdf(priv, userbuf);
	break;

	case ASPEED_MCTP_IOCTL_GET_MEDIUM_ID:
		ret = aspeed_mctp_get_medium_id(priv, userbuf);
	break;

	case ASPEED_MCTP_IOCTL_GET_MTU:
		ret = aspeed_mctp_get_mtu(priv, userbuf);
	break;

	case ASPEED_MCTP_IOCTL_REGISTER_DEFAULT_HANDLER:
		ret = aspeed_mctp_register_default_handler(client);
	break;

	default:
		dev_err(priv->dev, "Command not found\n");
		ret = -ENOTTY;
	}

	return ret;
}

static __poll_t aspeed_mctp_poll(struct file *file,
				 struct poll_table_struct *pt)
{
	struct mctp_client *client = file->private_data;
	__poll_t ret = 0;

	poll_wait(file, &client->wait_queue, pt);

	if (!ptr_ring_full_bh(&client->tx_queue))
		ret |= EPOLLOUT;

	if (__ptr_ring_peek(&client->rx_queue))
		ret |= EPOLLIN;

	return ret;
}

static const struct file_operations aspeed_mctp_fops = {
	.owner = THIS_MODULE,
	.open = aspeed_mctp_open,
	.release = aspeed_mctp_release,
	.read = aspeed_mctp_read,
	.write = aspeed_mctp_write,
	.unlocked_ioctl = aspeed_mctp_ioctl,
	.poll = aspeed_mctp_poll,
};

static const struct regmap_config aspeed_mctp_regmap_cfg = {
	.reg_bits	= 32,
	.reg_stride	= 4,
	.val_bits	= 32,
	.max_register	= ASPEED_MCTP_TX_BUF_WR_PTR,
};

struct device_type aspeed_mctp_type = {
	.name		= "aspeed-mctp",
};

static struct miscdevice aspeed_mctp_miscdev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "aspeed-mctp",
	.fops = &aspeed_mctp_fops,
};

static void aspeed_mctp_send_pcie_uevent(struct kobject *kobj, bool ready)
{
	char *pcie_not_ready_event[] = { ASPEED_MCTP_READY "=0", NULL };
	char *pcie_ready_event[] = { ASPEED_MCTP_READY "=1", NULL };

	kobject_uevent_env(kobj, KOBJ_CHANGE,
			   ready ? pcie_ready_event : pcie_not_ready_event);
}

static void aspeed_mctp_pcie_setup(struct aspeed_mctp *priv)
{
	u32 reg;

	regmap_read(priv->pcie.map, ASPEED_PCIE_MISC_STS_1, &reg);

	priv->pcie.bdf = PCI_DEVID(GET_PCI_BUS_NUM(reg), GET_PCI_DEV_NUM(reg));
	if (priv->pcie.bdf != 0)
		cancel_delayed_work(&priv->pcie.rst_dwork);
	else
		schedule_delayed_work(&priv->pcie.rst_dwork,
				      msecs_to_jiffies(1000));
}

static void aspeed_mctp_irq_enable(struct aspeed_mctp *priv)
{
	u32 enable = TX_CMD_LAST_INT | TX_CMD_WRONG_INT |
		     RX_CMD_RECEIVE_INT;

	regmap_write(priv->map, ASPEED_MCTP_INT_EN, enable);
}

static void aspeed_mctp_irq_disable(struct aspeed_mctp *priv)
{
	regmap_write(priv->map, ASPEED_MCTP_INT_EN, 0);
}

static void aspeed_mctp_reset_work(struct work_struct *work)
{
	struct aspeed_mctp *priv = container_of(work, typeof(*priv),
						pcie.rst_dwork.work);

	if (priv->pcie.need_uevent) {
		struct kobject *kobj = &aspeed_mctp_miscdev.this_device->kobj;

		aspeed_mctp_send_pcie_uevent(kobj, false);
		priv->pcie.need_uevent = false;
	}

	aspeed_mctp_pcie_setup(priv);

	if (priv->pcie.bdf) {
		aspeed_mctp_send_pcie_uevent(&priv->dev->kobj, true);
		aspeed_mctp_irq_enable(priv);
	}
}

static void aspeed_mctp_channels_init(struct aspeed_mctp *priv)
{
	aspeed_mctp_rx_chan_init(&priv->rx);
	aspeed_mctp_tx_chan_init(&priv->tx);
}

static irqreturn_t aspeed_mctp_irq_handler(int irq, void *arg)
{
	struct aspeed_mctp *priv = arg;
	u32 handled = 0;
	u32 status;

	regmap_read(priv->map, ASPEED_MCTP_INT_STS, &status);
	regmap_write(priv->map, ASPEED_MCTP_INT_STS, status);

	if (status & TX_CMD_LAST_INT) {
		u32 rd_ptr;

		regmap_write(priv->map, ASPEED_MCTP_TX_BUF_RD_PTR,
			     UPDATE_RX_RD_PTR);
		regmap_read(priv->map, ASPEED_MCTP_TX_BUF_RD_PTR, &rd_ptr);
		rd_ptr &= TX_BUFFER_RD_PTR;

		/*
		 * rd_ptr on TX side seems to be busted...
		 * Since we're always reading zeroes, let's trust that when
		 * we're getting LAST_CMD irq, everything we previously
		 * submitted was transmitted and start from 0
		 */
		WRITE_ONCE(priv->tx.rd_ptr, TX_PACKET_COUNT);

		tasklet_hi_schedule(&priv->tx.tasklet);

		handled |= TX_CMD_LAST_INT;
	}

	if (status & TX_CMD_WRONG_INT) {
		/* TODO: print the actual command */
		dev_warn(priv->dev, "TX wrong");

		handled |= TX_CMD_WRONG_INT;
	}

	if (status & RX_CMD_RECEIVE_INT) {
		/*
		 * XXX: After BMC reboot, it is possible that rd_ptr of RX
		 * buffer no longer matches the starting point of the RX
		 * buffer.
		 * Since we don't have any mechanism to reset it, we're
		 * determining the starting point ourselves, based on buffer
		 * contents when first packet is received.
		 */
		if (unlikely(priv->wa_runaway_rx)) {
			struct mctp_pcie_packet_data *rx_buf;
			u32 i;

			rx_buf = priv->rx.data.vaddr;

			for (i = 0; i < RX_PACKET_COUNT; i++) {
				u32 *hdr = (u32 *)&rx_buf[i];

				if (*hdr != 0)
					break;
			}
			WRITE_ONCE(priv->rx.wr_ptr, i % RX_PACKET_COUNT);
			priv->wa_runaway_rx = false;
		}

		tasklet_hi_schedule(&priv->rx.tasklet);

		handled |= RX_CMD_RECEIVE_INT;
	}

	if (!handled)
		return IRQ_NONE;

	return IRQ_HANDLED;
}

static irqreturn_t aspeed_mctp_pcie_rst_irq_handler(int irq, void *arg)
{
	struct aspeed_mctp *priv = arg;

	aspeed_mctp_channels_init(priv);

	priv->pcie.need_uevent = true;
	priv->pcie.bdf = 0;

	schedule_delayed_work(&priv->pcie.rst_dwork, 0);

	return IRQ_HANDLED;
}

static void aspeed_mctp_drv_init(struct aspeed_mctp *priv)
{
	INIT_LIST_HEAD(&priv->clients);

	spin_lock_init(&priv->clients_lock);

	INIT_DELAYED_WORK(&priv->pcie.rst_dwork, aspeed_mctp_reset_work);

	tasklet_init(&priv->tx.tasklet, aspeed_mctp_tx_tasklet,
		     (unsigned long)&priv->tx);
	tasklet_init(&priv->rx.tasklet, aspeed_mctp_rx_tasklet,
		     (unsigned long)&priv->rx);
}

static void aspeed_mctp_drv_fini(struct aspeed_mctp *priv)
{
	tasklet_disable(&priv->tx.tasklet);
	tasklet_kill(&priv->tx.tasklet);
	tasklet_disable(&priv->rx.tasklet);
	tasklet_kill(&priv->rx.tasklet);

	cancel_delayed_work_sync(&priv->pcie.rst_dwork);
}

static int aspeed_mctp_resources_init(struct aspeed_mctp *priv)
{
	struct platform_device *pdev = to_platform_device(priv->dev);
	void __iomem *regs;

	regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(regs)) {
		dev_err(priv->dev, "Failed to get regmap!\n");
		return PTR_ERR(regs);
	}

	priv->map = devm_regmap_init_mmio(priv->dev, regs,
					  &aspeed_mctp_regmap_cfg);
	if (IS_ERR(priv->map))
		return PTR_ERR(priv->map);

	priv->reset = devm_reset_control_get(priv->dev, 0);
	if (IS_ERR(priv->reset)) {
		dev_err(priv->dev, "Failed to get reset!\n");
		return PTR_ERR(priv->reset);
	}

	priv->pcie.map =
		syscon_regmap_lookup_by_phandle(priv->dev->of_node,
						"aspeed,pcieh");
	if (IS_ERR(priv->pcie.map)) {
		dev_err(priv->dev, "Failed to find PCIe Host regmap!\n");
		return PTR_ERR(priv->pcie.map);
	}

	platform_set_drvdata(pdev, priv);

	return 0;
}

static int aspeed_mctp_dma_init(struct aspeed_mctp *priv)
{
	struct mctp_channel *tx = &priv->tx;
	struct mctp_channel *rx = &priv->rx;
	int ret = -ENOMEM;

	BUILD_BUG_ON(TX_PACKET_COUNT >= TX_MAX_PACKET_COUNT);
	BUILD_BUG_ON(RX_CMD_COUNT >= RX_MAX_PACKET_COUNT);

	tx->cmd.vaddr = dma_alloc_coherent(priv->dev, TX_CMD_BUF_SIZE,
					   &tx->cmd.dma_handle, GFP_KERNEL);

	if (!tx->cmd.vaddr)
		return ret;

	tx->data.vaddr = dma_alloc_coherent(priv->dev, TX_DATA_BUF_SIZE,
					    &tx->data.dma_handle, GFP_KERNEL);

	if (!tx->data.vaddr)
		goto out_tx_data;

	rx->cmd.vaddr = dma_alloc_coherent(priv->dev, RX_CMD_BUF_SIZE,
					   &rx->cmd.dma_handle, GFP_KERNEL);

	if (!rx->cmd.vaddr)
		goto out_tx_cmd;

	rx->data.vaddr = dma_alloc_coherent(priv->dev, RX_DATA_BUF_SIZE,
					    &rx->data.dma_handle, GFP_KERNEL);

	if (!rx->data.vaddr)
		goto out_rx_data;

	return 0;
out_rx_data:
	dma_free_coherent(priv->dev, RX_CMD_BUF_SIZE, rx->cmd.vaddr,
			  rx->cmd.dma_handle);

out_tx_cmd:
	dma_free_coherent(priv->dev, TX_DATA_BUF_SIZE, tx->data.vaddr,
			  tx->data.dma_handle);

out_tx_data:
	dma_free_coherent(priv->dev, TX_CMD_BUF_SIZE, tx->cmd.vaddr,
			  tx->cmd.dma_handle);
	return ret;
}

static void aspeed_mctp_dma_fini(struct aspeed_mctp *priv)
{
	struct mctp_channel *tx = &priv->tx;
	struct mctp_channel *rx = &priv->rx;

	dma_free_coherent(priv->dev, TX_CMD_BUF_SIZE, tx->cmd.vaddr,
			  tx->cmd.dma_handle);

	dma_free_coherent(priv->dev, RX_CMD_BUF_SIZE, rx->cmd.vaddr,
			  rx->cmd.dma_handle);

	dma_free_coherent(priv->dev, TX_DATA_BUF_SIZE, tx->data.vaddr,
			  tx->data.dma_handle);

	dma_free_coherent(priv->dev, RX_DATA_BUF_SIZE, rx->data.vaddr,
			  rx->data.dma_handle);
}

static int aspeed_mctp_irq_init(struct aspeed_mctp *priv)
{
	struct platform_device *pdev = to_platform_device(priv->dev);
	int irq, ret;

	irq = platform_get_irq(pdev, 0);
	if (!irq)
		return -ENODEV;

	ret = devm_request_irq(priv->dev, irq, aspeed_mctp_irq_handler,
			       IRQF_SHARED, "aspeed-mctp", priv);
	if (ret)
		return ret;

	irq = platform_get_irq(pdev, 1);
	if (!irq)
		return -ENODEV;

	ret = devm_request_irq(priv->dev, irq,
			       aspeed_mctp_pcie_rst_irq_handler,
			       IRQF_SHARED, "aspeed-mctp", priv);
	if (ret)
		return ret;

	return 0;
}

static void aspeed_mctp_hw_reset(struct aspeed_mctp *priv)
{
	u32 reg;

	/*
	 * XXX: We need to skip the reset when we probe multiple times.
	 * Currently calling reset more than once seems to make the HW upset,
	 * however, we do need to reset once after the first boot before we're
	 * able to use the HW.
	 */
	regmap_read(priv->map, ASPEED_MCTP_TX_BUF_ADDR, &reg);

	if (reg) {
		priv->wa_runaway_rx = true;
		dev_info(priv->dev,
			 "Already initialized - skipping hardware reset\n");
		return;
	}

	if (reset_control_assert(priv->reset) != 0)
		dev_warn(priv->dev, "Failed to assert reset\n");

	if (reset_control_deassert(priv->reset) != 0)
		dev_warn(priv->dev, "Failed to deassert reset\n");
}

static int aspeed_mctp_probe(struct platform_device *pdev)
{
	struct aspeed_mctp *priv;
	int ret;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		ret = -ENOMEM;
		goto out;
	}
	priv->dev = &pdev->dev;

	aspeed_mctp_drv_init(priv);

	ret = aspeed_mctp_resources_init(priv);
	if (ret) {
		dev_err(priv->dev, "Failed to init resources\n");
		goto out_drv;
	}

	ret = aspeed_mctp_dma_init(priv);
	if (ret) {
		dev_err(priv->dev, "Failed to init DMA\n");
		goto out_drv;
	}

	aspeed_mctp_hw_reset(priv);

	aspeed_mctp_channels_init(priv);

	aspeed_mctp_miscdev.parent = priv->dev;
	ret = misc_register(&aspeed_mctp_miscdev);
	if (ret) {
		dev_err(priv->dev, "Failed to register miscdev\n");
		goto out_dma;
	}
	aspeed_mctp_miscdev.this_device->type = &aspeed_mctp_type;

	ret = aspeed_mctp_irq_init(priv);
	if (ret) {
		dev_err(priv->dev, "Failed to init IRQ!\n");
		goto out_dma;
	}

	aspeed_mctp_irq_enable(priv);

	aspeed_mctp_pcie_setup(priv);

	return 0;

out_dma:
	aspeed_mctp_dma_fini(priv);
out_drv:
	aspeed_mctp_drv_fini(priv);
out:
	dev_err(&pdev->dev, "Failed to probe Aspeed MCTP: %d\n", ret);
	return ret;
}

static int aspeed_mctp_remove(struct platform_device *pdev)
{
	struct aspeed_mctp *priv = platform_get_drvdata(pdev);

	misc_deregister(&aspeed_mctp_miscdev);

	aspeed_mctp_irq_disable(priv);

	aspeed_mctp_dma_fini(priv);

	aspeed_mctp_drv_fini(priv);

	return 0;
}

static const struct of_device_id aspeed_mctp_match_table[] = {
	{ .compatible = "aspeed,ast2600-mctp" },
	{ }
};

static struct platform_driver aspeed_mctp_driver = {
	.driver	= {
		.name		= "aspeed-mctp",
		.of_match_table	= of_match_ptr(aspeed_mctp_match_table),
	},
	.probe	= aspeed_mctp_probe,
	.remove	= aspeed_mctp_remove,
};

static int __init aspeed_mctp_init(void)
{
	packet_cache =
		kmem_cache_create_usercopy("mctp-packet",
					   sizeof(struct mctp_pcie_packet),
					   0, 0, 0,
					   sizeof(struct mctp_pcie_packet),
					   NULL);
	if (!packet_cache)
		return -ENOMEM;

	return platform_driver_register(&aspeed_mctp_driver);
}

static void __exit aspeed_mctp_exit(void)
{
	platform_driver_unregister(&aspeed_mctp_driver);
	kmem_cache_destroy(packet_cache);
}

module_init(aspeed_mctp_init)
module_exit(aspeed_mctp_exit)

MODULE_DEVICE_TABLE(of, aspeed_mctp_match_table);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Iwona Winiarska <iwona.winiarska@intel.com>");
MODULE_DESCRIPTION("Aspeed AST2600 MCTP driver");
