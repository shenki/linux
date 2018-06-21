/*
 * MCTP driver for the Aspeed SoC
 *
 * Copyright (C) ASPEED Technology Inc.
 * Ryan Chen <ryan_chen@aspeedtech.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 *
 */
#include <linux/sysfs.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/reset.h>
#include <linux/dma-mapping.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_device.h>
#include <linux/io.h>
#include <asm/uaccess.h>
/* register ************************************************************************************/
#define ASPEED_MCTP_CTRL 		0x00
#define ASPEED_MCTP_TX_CMD		0x04
#define ASPEED_MCTP_RX_CMD		0x08
#define ASPEED_MCTP_ISR 		0x0c
#define ASPEED_MCTP_IER 		0x10
#define ASPEED_MCTP_EID 		0x14
#define ASPEED_MCTP_OBFF 		0x18

/* ASPEED_MCTP_CTRL 		0x00 */
#define MCTP_RX_CMD_RDY			(1 << 4)
#define MCTP_TX_TRIGGER			(1)

/* ASPEED_MCTP_ISR 		0x0c */
#define MCTP_RX_NO_CMD			(1 << 9)
#define MCTP_RX_COMPLETE		(1 << 8)

#define MCTP_TX_LAST			(1 << 1)
#define MCTP_TX_COMPLETE		(1)

/*************************************************************************************/
//TX CMD desc0
#define BUS_NO(x)				((x & 0xff) << 24)
#define DEV_NO(x)				((x & 0x1f) << 19)
#define FUN_NO(x)				((x & 0x7) << 16)

//ast-g5
/* 0: route to RC, 1: route by ID, 2/3: broadcast from RC */
#define G5_ROUTING_TYPE_L(x)			((x & 0x1) << 14)
#define G5_ROUTING_TYPE_H(x)			(((x & 0x2) >> 1) << 12)
//ast old version
#define ROUTING_TYPE(x)			((x & 0x1) << 14)

#define TAG_OWN					(1 << 13)
#define PKG_SIZE(x)				((x & 0x7ff) << 2)
#define PADDING_LEN(x)			(x & 0x3)
//TX CMD desc1
#define LAST_CMD				(1 << 31)
//ast-g5
#define G5_TX_DATA_ADDR(x)			(((x >> 7) & 0x7fffff) << 8)
//ast old version
#define TX_DATA_ADDR(x)			(((x >> 6) & 0x7fffff) << 8)

#define DEST_EP_ID(x)			(x & 0xff)
/*************************************************************************************/
//RX CMD desc0
#define GET_PKG_LEN(x)			((x >> 24) & 0x7f)
#define GET_SRC_EP_ID(x)		((x >> 16) & 0xff)
#define GET_ROUTING_TYPE(x)	((x >> 14) & 0x7)
#define GET_SEQ_NO(x)			((x >> 11) & 0x3)
#define MCTP_SOM				(1 << 7)
#define MCTP_EOM				(1 << 6)
#define GET_PADDING_LEN(x)		((x >> 4) & 0x3)
#define CMD_UPDATE				(1)
//RX CMD desc1
#define LAST_CMD				(1 << 31)
#define RX_DATA_ADDR(x)			(((x >> 7) & 0x3fffff) << 7)

struct aspeed_mctp_cmd_desc {
	unsigned int desc0;
	unsigned int desc1;
};

/*************************************************************************************/
#define MAX_XFER_BUFF_SIZE 1024

struct aspeed_mctp_xfer {
	unsigned char	xfer_buff[MAX_XFER_BUFF_SIZE];
	unsigned int xfer_len;
	unsigned int ep_id;
	unsigned int bus_no;
	unsigned int dev_no;
	unsigned int fun_no;
	unsigned char rt;

};

#define MCTPIOC_BASE       'M'

#define ASPEED_MCTP_IOCRX			_IOWR(MCTPIOC_BASE, 0x10, struct aspeed_mctp_xfer)
#define ASPEED_MCTP_IOCTX			_IOW(MCTPIOC_BASE, 0x11, struct aspeed_mctp_xfer)

/*************************************************************************************/
//#define ASPEED_MCTP_DEBUG

#ifdef ASPEED_MCTP_DEBUG
#define MCTP_DBUG(fmt, args...) printk("%s() " fmt,__FUNCTION__, ## args)
#else
#define MCTP_DBUG(fmt, args...)
#endif

#define MCTP_MSG(fmt, args...) printk(fmt, ## args)

struct aspeed_mctp_info {
	void __iomem	*reg_base;
	int irq;				//MCTP IRQ number
	struct reset_control *reset;
	u8	aspeed_g5_mctp;
	u32 dram_base;
	wait_queue_head_t mctp_wq;
	u8 *mctp_dma;
	dma_addr_t mctp_dma_addr;

	struct aspeed_mctp_cmd_desc *tx_cmd_desc;
	dma_addr_t tx_cmd_desc_dma;
	u8 *tx_data;
	dma_addr_t tx_data_dma;

	struct aspeed_mctp_cmd_desc *rx_cmd_desc;
	dma_addr_t rx_cmd_desc_dma;
	u8 *rx_data;
	dma_addr_t rx_data_dma;
	u32 rx_index;
	u8 *rx_fifo;
	u8 rx_fifo_index;
	u32 rx_fifo_done;

	u32 flag;
	bool is_open;
	u32 state;
	//rx
	u32 rx_len;
	u8 ep_id;
	u8 rt;
	u8 seq_no;
};

/******************************************************************************/
static DEFINE_SPINLOCK(mctp_state_lock);

/******************************************************************************/

static inline u32
aspeed_mctp_read(struct aspeed_mctp_info *aspeed_mctp, u32 reg)
{
	u32 val;

	val = readl(aspeed_mctp->reg_base + reg);
	MCTP_DBUG("reg = 0x%08x, val = 0x%08x\n", reg, val);
	return val;
}

static inline void
aspeed_mctp_write(struct aspeed_mctp_info *aspeed_mctp, u32 val, u32 reg)
{
	MCTP_DBUG("reg = 0x%08x, val = 0x%08x\n", reg, val);

	writel(val, aspeed_mctp->reg_base + reg);
}

/*************************************************************************************/
static void aspeed_mctp_wait_tx_complete(struct aspeed_mctp_info *aspeed_mctp)
{
	wait_event_interruptible(aspeed_mctp->mctp_wq, (aspeed_mctp->flag == MCTP_TX_LAST));
	MCTP_DBUG("\n");
	aspeed_mctp->flag = 0;
}

static void aspeed_mctp_wait_rx_complete(struct aspeed_mctp_info *aspeed_mctp)
{
	wait_event_interruptible(aspeed_mctp->mctp_wq, (aspeed_mctp->flag == MCTP_EOM));
	MCTP_DBUG("\n");
	aspeed_mctp->flag = 0;
}

static void aspeed_mctp_tx_xfer(struct aspeed_mctp_info *aspeed_mctp, struct aspeed_mctp_xfer *mctp_xfer)
{

	u32 xfer_len = (mctp_xfer->xfer_len / 4);
	u32 padding_len;

	if ((mctp_xfer->xfer_len % 4)) {
		xfer_len++;
		padding_len = 4 - ((mctp_xfer->xfer_len) % 4);
	} else {
		padding_len = 0;
	}

	MCTP_DBUG("xfer_len = %d, padding len = %d , 4byte align %d\n", mctp_xfer->xfer_len, padding_len, xfer_len);

	if (aspeed_mctp->aspeed_g5_mctp) {
		//routing type [desc0 bit 12, desc0 bit 14]
		//bit 15 : interrupt enable
		//set default tag owner = 1;
		aspeed_mctp->tx_cmd_desc->desc0 = 0x0000a000 | G5_ROUTING_TYPE_H(mctp_xfer->rt) | G5_ROUTING_TYPE_L(mctp_xfer->rt) | PKG_SIZE(xfer_len) | BUS_NO(mctp_xfer->bus_no) | DEV_NO(mctp_xfer->dev_no) | FUN_NO(mctp_xfer->fun_no) | PADDING_LEN(padding_len);
	} else {
		//routing type bit 14
		//bit 15 : interrupt enable
		//set default tag owner = 1;
		aspeed_mctp->tx_cmd_desc->desc0 = 0x0000a000 | ROUTING_TYPE(mctp_xfer->rt) | PKG_SIZE(xfer_len) | BUS_NO(mctp_xfer->bus_no) | DEV_NO(mctp_xfer->dev_no) | FUN_NO(mctp_xfer->fun_no) | PADDING_LEN(padding_len);
	}

	//set dest ep id = 0;
	aspeed_mctp->tx_cmd_desc->desc1 |= LAST_CMD | DEST_EP_ID(0);

	MCTP_DBUG("memcpy  to %x from %x len = %d \n", aspeed_mctp->tx_data, mctp_xfer->xfer_buff, mctp_xfer->xfer_len);

	memset(aspeed_mctp->tx_data, 0,  1024);
	memcpy(aspeed_mctp->tx_data, mctp_xfer->xfer_buff,  mctp_xfer->xfer_len);

	//trigger tx
	aspeed_mctp_write(aspeed_mctp, aspeed_mctp_read(aspeed_mctp, ASPEED_MCTP_CTRL) | MCTP_TX_TRIGGER, ASPEED_MCTP_CTRL);

	//wait intr
	aspeed_mctp_wait_tx_complete(aspeed_mctp);

}

static void aspeed_mctp_rx_combine_data(struct aspeed_mctp_info *aspeed_mctp)
{
	int i;
	u32 rx_len = 0;
	u32 padding_len = 0;

	MCTP_DBUG(" \n");

	for (i = 0; i < 8; i++) {
		aspeed_mctp->rx_index %= 8;
		MCTP_DBUG("index %d, desc0 %x \n", aspeed_mctp->rx_index, aspeed_mctp->rx_cmd_desc[aspeed_mctp->rx_index].desc0);
		MCTP_DBUG("index %d, desc1 %x \n", aspeed_mctp->rx_index, aspeed_mctp->rx_cmd_desc[aspeed_mctp->rx_index].desc1);

		if (aspeed_mctp->rx_cmd_desc[aspeed_mctp->rx_index].desc0 & CMD_UPDATE) {
			if (aspeed_mctp->rx_fifo_done != 1) {
				MCTP_DBUG("rx fifo index %d \n", aspeed_mctp->rx_fifo_index);
				if (MCTP_SOM & aspeed_mctp->rx_cmd_desc[aspeed_mctp->rx_index].desc0) {
					MCTP_DBUG("MCTP_SOM \n");
					aspeed_mctp->rx_fifo_index = 0;
				}

				if (MCTP_EOM & aspeed_mctp->rx_cmd_desc[aspeed_mctp->rx_index].desc0) {
					MCTP_DBUG("MCTP_EOM aspeed_mctp->rx_fifo_done \n");
					aspeed_mctp->rx_fifo_done = 1;
					padding_len = GET_PADDING_LEN(aspeed_mctp->rx_cmd_desc[aspeed_mctp->rx_index].desc0);
					aspeed_mctp->flag = MCTP_EOM;
				}

				rx_len = GET_PKG_LEN(aspeed_mctp->rx_cmd_desc[aspeed_mctp->rx_index].desc0) * 4;
				rx_len -= padding_len;

				memcpy(aspeed_mctp->rx_fifo + (0x40 * aspeed_mctp->rx_fifo_index), aspeed_mctp->rx_data + (aspeed_mctp->rx_index * 0x80), rx_len);
				aspeed_mctp->rx_fifo_index++;

				aspeed_mctp->rx_len += rx_len;
				aspeed_mctp->ep_id = GET_SRC_EP_ID(aspeed_mctp->rx_cmd_desc[aspeed_mctp->rx_index].desc0);
				aspeed_mctp->rt = GET_ROUTING_TYPE(aspeed_mctp->rx_cmd_desc[aspeed_mctp->rx_index].desc0);
				aspeed_mctp->seq_no = GET_SEQ_NO(aspeed_mctp->rx_cmd_desc[aspeed_mctp->rx_index].desc0);
				MCTP_DBUG("rx len = %d , epid = %d, rt = %x, seq no = %d, total_len = %d \n", rx_len, aspeed_mctp->ep_id, aspeed_mctp->rt, aspeed_mctp->seq_no, aspeed_mctp->rx_len);

			} else {
				MCTP_DBUG("drop \n");
			}

			//RX CMD desc0
			aspeed_mctp->rx_cmd_desc[aspeed_mctp->rx_index].desc0 = 0;
			aspeed_mctp->rx_index++;

		} else {
			MCTP_DBUG("index %d break\n", aspeed_mctp->rx_index);
			break;
		}
	}
}

static irqreturn_t aspeed_mctp_isr(int this_irq, void *dev_id)
{
	struct aspeed_mctp_info *aspeed_mctp = dev_id;
	u32 status = aspeed_mctp_read(aspeed_mctp, ASPEED_MCTP_ISR);

	MCTP_DBUG("%x \n", status);

	if (status & MCTP_TX_LAST) {
		aspeed_mctp_write(aspeed_mctp, MCTP_TX_LAST | (status), ASPEED_MCTP_ISR);
		aspeed_mctp->flag = MCTP_TX_LAST;
	}

	if (status & MCTP_TX_COMPLETE) {
		aspeed_mctp_write(aspeed_mctp, MCTP_TX_COMPLETE | (status), ASPEED_MCTP_ISR);
//		aspeed_mctp->flag = MCTP_TX_COMPLETE;
	}

	if (status & MCTP_RX_COMPLETE) {
		aspeed_mctp_write(aspeed_mctp, MCTP_RX_COMPLETE | (status), ASPEED_MCTP_ISR);
//		aspeed_mctp->flag = MCTP_RX_COMPLETE;
//		aspeed_mctp->flag = 0;
		aspeed_mctp_rx_combine_data(aspeed_mctp);
	}

	if (status & MCTP_RX_NO_CMD) {
		aspeed_mctp_write(aspeed_mctp, MCTP_RX_NO_CMD | (status), ASPEED_MCTP_ISR);
		aspeed_mctp->flag = MCTP_RX_NO_CMD;
		printk("MCTP_RX_NO_CMD \n");
	}

	if (aspeed_mctp->flag) {
		wake_up_interruptible(&aspeed_mctp->mctp_wq);
		return IRQ_HANDLED;
	} else {
		printk("TODO Check MCTP's interrupt %x\n", status);
		return IRQ_NONE;
	}

}

static void aspeed_mctp_ctrl_init(struct aspeed_mctp_info *aspeed_mctp)
{
	int i = 0;

	MCTP_DBUG("dram base %x \n", aspeed_mctp->dram_base);
	aspeed_mctp_write(aspeed_mctp, aspeed_mctp->dram_base, ASPEED_MCTP_EID);
	//4K size memory
	//1st 1024 : cmd desc --> 0~512 : tx desc , 512 ~ 1024 : rx desc
	//2nd 1024 : tx data
	//3rd 1024 : rx data --> 8 - 0x00 , 0x80 , 0x100, 0x180, each 128 align
	//4th 1024 : rx data combine

	//tx
	aspeed_mctp->mctp_dma = dma_alloc_coherent(NULL,
						4096,
						&aspeed_mctp->mctp_dma_addr, GFP_KERNEL);

	if (((u32)aspeed_mctp->mctp_dma & 0xff) != 0x00)
		printk("ERROR dma addr !!!!\n");


	aspeed_mctp->tx_cmd_desc = (struct aspeed_mctp_cmd_desc *)aspeed_mctp->mctp_dma;
	aspeed_mctp->tx_cmd_desc_dma = aspeed_mctp->mctp_dma_addr;

	MCTP_DBUG("tx cmd desc %x , cmd desc dma %x \n", (u32)aspeed_mctp->tx_cmd_desc, (u32)aspeed_mctp->tx_cmd_desc_dma);

	aspeed_mctp->tx_data = (u8 *)(aspeed_mctp->mctp_dma + 1024);
	aspeed_mctp->tx_data_dma = aspeed_mctp->mctp_dma_addr + 1024;

	for (i = 0; i < 1024; i++) {
		aspeed_mctp->tx_data[i] = i;
	}

	if (aspeed_mctp->aspeed_g5_mctp)
		aspeed_mctp->tx_cmd_desc->desc1 |= G5_TX_DATA_ADDR(aspeed_mctp->tx_data_dma);
	else
		aspeed_mctp->tx_cmd_desc->desc1 |= TX_DATA_ADDR(aspeed_mctp->tx_data_dma);

	MCTP_DBUG("tx data %x , tx data dma %x \n", (u32)aspeed_mctp->tx_data, (u32)aspeed_mctp->tx_data_dma);

	aspeed_mctp_write(aspeed_mctp, aspeed_mctp->tx_cmd_desc_dma, ASPEED_MCTP_TX_CMD);

	//RX 8 buffer
	aspeed_mctp->rx_cmd_desc = (struct aspeed_mctp_cmd_desc *)(aspeed_mctp->mctp_dma + 512);
	aspeed_mctp->rx_cmd_desc_dma = aspeed_mctp->mctp_dma_addr + 512;

	MCTP_DBUG("rx cmd desc %x , cmd desc dma %x \n", (u32)aspeed_mctp->rx_cmd_desc, (u32)aspeed_mctp->rx_cmd_desc_dma);

	aspeed_mctp->rx_data = (u8 *)(aspeed_mctp->mctp_dma + 2048);
	aspeed_mctp->rx_data_dma = aspeed_mctp->mctp_dma_addr + 2048;

	MCTP_DBUG("rx data %x , rx data dma %x \n", (u32)aspeed_mctp->rx_data, (u32)aspeed_mctp->rx_data_dma);

	aspeed_mctp->rx_index = 0;
	aspeed_mctp->rx_fifo = (u8 *)(aspeed_mctp->mctp_dma + 3072);
	MCTP_DBUG("aspeed_mctp->rx_fifo %x \n", (u32)aspeed_mctp->rx_fifo);

	aspeed_mctp->rx_fifo_done = 0;
	aspeed_mctp->rx_fifo_index = 0;
	aspeed_mctp->rx_len = 0;
	memset(aspeed_mctp->rx_fifo, 0,  1024);

	for (i = 0; i < 8; i++) {
		aspeed_mctp->rx_cmd_desc[i].desc0 = 0;
		aspeed_mctp->rx_cmd_desc[i].desc1 = RX_DATA_ADDR((aspeed_mctp->rx_data_dma + (0x80 * i))) ;
		if (i == 7)
			aspeed_mctp->rx_cmd_desc[i].desc1 |= LAST_CMD;
		MCTP_DBUG("Rx [%d]: desc0: %x , desc1: %x \n", i, aspeed_mctp->rx_cmd_desc[i].desc0, aspeed_mctp->rx_cmd_desc[i].desc1);
	}

//	MCTP_DBUG("rx cmd desc %x , data dma %x \n", aspeed_mctp->rx_cmd_desc_dma, aspeed_mctp->rx_data_dma);

	aspeed_mctp_write(aspeed_mctp, aspeed_mctp->rx_cmd_desc_dma, ASPEED_MCTP_RX_CMD);

	aspeed_mctp_write(aspeed_mctp, aspeed_mctp_read(aspeed_mctp, ASPEED_MCTP_CTRL) | MCTP_RX_CMD_RDY, ASPEED_MCTP_CTRL);

	aspeed_mctp_write(aspeed_mctp, MCTP_RX_COMPLETE | MCTP_TX_LAST, ASPEED_MCTP_IER);
}

static long mctp_ioctl(struct file *file, unsigned int cmd,
		       unsigned long arg)
{
	struct miscdevice *c = file->private_data;
	struct aspeed_mctp_info *aspeed_mctp = dev_get_drvdata(c->this_device);
	void __user *argp = (void __user *)arg;
	struct aspeed_mctp_xfer xfer;

	switch (cmd) {
	case ASPEED_MCTP_IOCTX:
		MCTP_DBUG("ASPEED_MCTP_IOCTX \n");
		if (copy_from_user(&xfer, argp, sizeof(struct aspeed_mctp_xfer))) {
			MCTP_DBUG("copy_from_user  fail\n");
			return -EFAULT;
		} else
			aspeed_mctp_tx_xfer(aspeed_mctp, &xfer);
		break;
	case ASPEED_MCTP_IOCRX:
		MCTP_DBUG("ASPEED_MCTP_IOCRX \n");
		//wait intr
//			if(!aspeed_mctp->rx_fifo_done) {
//				MCTP_DBUG("wait for rx fifo done \n");
		aspeed_mctp_wait_rx_complete(aspeed_mctp);
//			}
		xfer.xfer_len = aspeed_mctp->rx_len;
		memcpy(xfer.xfer_buff, aspeed_mctp->rx_fifo,  aspeed_mctp->rx_len);
		if (copy_to_user(argp, &xfer, sizeof(struct aspeed_mctp_xfer)))
			return -EFAULT;
		else {
			aspeed_mctp->rx_fifo_done = 0;
			aspeed_mctp->rx_len = 0;
			memset(aspeed_mctp->rx_fifo, 0,  1024);
		}
		break;

	default:
		MCTP_DBUG("ERROR \n");
		return -ENOTTY;
	}

	return 0;
}

static int mctp_open(struct inode *inode, struct file *file)
{
	struct miscdevice *c = file->private_data;
	struct aspeed_mctp_info *aspeed_mctp = dev_get_drvdata(c->this_device);

	MCTP_DBUG("\n");
	spin_lock(&mctp_state_lock);

	if (aspeed_mctp->is_open) {
		spin_unlock(&mctp_state_lock);
		return -EBUSY;
	}

	aspeed_mctp->is_open = true;

	spin_unlock(&mctp_state_lock);

	return 0;
}

static int mctp_release(struct inode *inode, struct file *file)
{
	struct miscdevice *c = file->private_data;
	struct aspeed_mctp_info *aspeed_mctp = dev_get_drvdata(c->this_device);

	MCTP_DBUG("\n");
	spin_lock(&mctp_state_lock);

	aspeed_mctp->is_open = false;

	spin_unlock(&mctp_state_lock);

	return 0;
}

static const struct file_operations aspeed_mctp_fops = {
	.owner			= THIS_MODULE,
	.unlocked_ioctl	= mctp_ioctl,
	.open			= mctp_open,
	.release			= mctp_release,
};

static struct miscdevice aspeed_mctp_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "aspeed-mctp",
	.fops = &aspeed_mctp_fops,
};

static int aspeed_mctp_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct aspeed_mctp_info *aspeed_mctp;
	int ret = 0;

	MCTP_DBUG("\n");

	if (!(aspeed_mctp = devm_kzalloc(&pdev->dev, sizeof(struct aspeed_mctp_info), GFP_KERNEL))) {
		return -ENOMEM;
	}

	if (of_device_is_compatible(pdev->dev.of_node, "aspeed,ast2500-mctp"))
		aspeed_mctp->aspeed_g5_mctp = 1;
	else
		aspeed_mctp->aspeed_g5_mctp = 0;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (NULL == res) {
		dev_err(&pdev->dev, "cannot get IORESOURCE_MEM\n");
		ret = -ENOENT;
		goto out;
	}

	aspeed_mctp->reg_base = devm_ioremap_resource(&pdev->dev, res);
	if (!aspeed_mctp->reg_base) {
		ret = -EIO;
		goto out_region;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (NULL == res) {
		dev_err(&pdev->dev, "cannot get IORESOURCE_BUS\n");
		ret = -ENOENT;
		goto out_region;
	}

	aspeed_mctp->dram_base = (u32)res->start;

	aspeed_mctp->irq = platform_get_irq(pdev, 0);
	if (aspeed_mctp->irq < 0) {
		dev_err(&pdev->dev, "no irq specified\n");
		ret = -ENOENT;
		goto out_region;
	}

	aspeed_mctp->reset = devm_reset_control_get(&pdev->dev, NULL);
	if (IS_ERR(aspeed_mctp->reset)) {
		dev_err(&pdev->dev, "can't get mctp reset\n");
		return PTR_ERR(aspeed_mctp->reset);
	}

	//scu init
	reset_control_assert(aspeed_mctp->reset);
	reset_control_deassert(aspeed_mctp->reset);

	aspeed_mctp->flag = 0;
	init_waitqueue_head(&aspeed_mctp->mctp_wq);

	ret = misc_register(&aspeed_mctp_misc);
	if (ret) {
		printk(KERN_ERR "MCTP : failed to request interrupt\n");
		goto out_region;
	}

	platform_set_drvdata(pdev, aspeed_mctp);
	dev_set_drvdata(aspeed_mctp_misc.this_device, aspeed_mctp);

	aspeed_mctp_ctrl_init(aspeed_mctp);

	ret = devm_request_irq(&pdev->dev, aspeed_mctp->irq, aspeed_mctp_isr,
			       0, dev_name(&pdev->dev), aspeed_mctp);
	if (ret) {
		printk("MCTP Unable to get IRQ");
		goto out_irq;
	}

#if 0
	ret = sysfs_create_group(&aspeed_mctp_misc.this_device->kobj,
				 &mctp_attribute_group);
	if (ret) {
		printk(KERN_ERR "lattice: failed to create sysfs device attributes.\n");
		return -1;
	}
#endif
	printk(KERN_INFO "aspeed_mctp: driver successfully loaded.\n");

	return 0;


out_irq:
	free_irq(aspeed_mctp->irq, NULL);
out_region:
	release_mem_region(res->start, res->end - res->start + 1);
out:
	printk(KERN_WARNING "aspeed_mctp: driver init failed (ret=%d)!\n", ret);
	return ret;
}

static int aspeed_mctp_remove(struct platform_device *pdev)
{
	struct resource *res;
	struct aspeed_mctp_info *aspeed_mctp = platform_get_drvdata(pdev);

	MCTP_DBUG("\n");

	misc_deregister(&aspeed_mctp_misc);

	free_irq(aspeed_mctp->irq, aspeed_mctp);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	iounmap(aspeed_mctp->reg_base);

	platform_set_drvdata(pdev, NULL);

	release_mem_region(res->start, res->end - res->start + 1);

	return 0;
}

#ifdef CONFIG_PM
static int
aspeed_mctp_suspend(struct platform_device *pdev, pm_message_t state)
{
	return 0;
}

static int
aspeed_mctp_resume(struct platform_device *pdev)
{
	return 0;
}

#else
#define aspeed_mctp_suspend        NULL
#define aspeed_mctp_resume         NULL
#endif

static const struct of_device_id aspeed_mctp_of_matches[] = {
	{ .compatible = "aspeed,ast2400-mctp", },
	{ .compatible = "aspeed,ast2500-mctp", },
	{},
};
MODULE_DEVICE_TABLE(of, aspeed_mctp_of_matches);

static struct platform_driver aspeed_mctp_driver = {
	.probe 		= aspeed_mctp_probe,
	.remove 		= aspeed_mctp_remove,
#ifdef CONFIG_PM
	.suspend        = aspeed_mctp_suspend,
	.resume         = aspeed_mctp_resume,
#endif
	.driver         = {
		.name   = KBUILD_MODNAME,
		.of_match_table = aspeed_mctp_of_matches,
	},
};

module_platform_driver(aspeed_mctp_driver);

MODULE_AUTHOR("Ryan Chen <ryan_chen@aspeedtech.com>");
MODULE_DESCRIPTION("ASPEED MCTP Driver");
MODULE_LICENSE("GPL");
