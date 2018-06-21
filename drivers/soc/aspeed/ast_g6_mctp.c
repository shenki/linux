/*
 * ast-mctp.c - MCTP driver for the Aspeed SoC
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
#define ASPEED_MCTP_CTRL1 		0x1C
#define ASPEED_MCTP_RXBUFF_ADDR		0x20
#define ASPEED_MCTP_RXBUFF_SIZE		0x24
#define ASPEED_MCTP_WRITE_POINT		0x28
#define ASPEED_MCTP_READ_POINT		0x2C

/* AST_MCTP_CTRL - 0x00 : Engine Status and Engine Control */
#define MCTP_GET_CURRENT_CMD_CNT(x)	((x >> 24) & 0x3f)

#define MCTP_RX_PCIE_IDLE		(1 << 21)
#define MCTP_RX_DMA_IDLE		(1 << 20)

#define MCTP_TX_PCIE_IDLE		(1 << 17)
#define MCTP_TX_DMA_IDLE		(1 << 16)
#define MCTP_CPL2_ENABLE		(1 << 15)

#define MCTP_MATCH_EID			(1 << 9)
//\xregmid { 9   }{RW}{Matching MCTP EID}{
//                     1: Only accept packet whose MCTP Destination EID matches MCTP14[7:0]}
#define MCTP_RX_CMD_RDY			(1 << 4)
#define MCTP_TX_TRIGGER			(1)

/* AST_MCTP_ISR - 0x0c : Interrupt Status */
#define MCTP_MSG_OBFF_STS_CHG		(1 << 27)
#define MCTP_MSG_OBFF_ACTIVE		(1 << 26)
#define MCTP_MSG_OBFF_IDLE		(1 << 26)
#define MCTP_MSG_OBFF_STATE		(1 << 25)

#define MCTP_WAKE_OBFF_ACTIVE		(1 << 19)
#define MCTP_WAKE_POST_OBFF		(1 << 18)
#define MCTP_WAKE_OBFF_STATE		(1 << 17)
#define MCTP_WAKE_OBFF_IDLE		(1 << 16)

#define MCTP_RX_NO_CMD			(1 << 9)
#define MCTP_RX_COMPLETE		(1 << 8)

#define MCTP_TX_CMD_WRONG		(1 << 2)
#define MCTP_TX_LAST			(1 << 1)
#define MCTP_TX_COMPLETE		(1)

/* aspeed g6 mctp */
/* AST_MCTP_CTRL1 - 0x1C : Engine Control 1 */
#define MCTP_FIFO_FULL			(1 << 19)
#define MCTP_OBFF_DMA_ENABLE		(1 << 18)
#define MCTP_OBFF_MONITOR		(1 << 17)

#define MCTP_6KB_TX_2KB_RX_FIFO		(0 << 8)
#define MCTP_4KB_TX_4KB_RX_FIFO		(1 << 8)
#define MCTP_2KB_TX_6KB_RX_FIFO		(2 << 8)

#define MCTP_RX_PAYLOAD_64BYTE		(0 << 4)
#define MCTP_RX_PAYLOAD_128BYTE		(1 << 4)
#define MCTP_RX_PAYLOAD_256BYTE		(2 << 4)
#define MCTP_RX_PAYLOAD_512BYTE		(3 << 4)

#define MCTP_TX_PAYLOAD_64BYTE		(0)
#define MCTP_TX_PAYLOAD_128BYTE		(1)
#define MCTP_TX_PAYLOAD_256BYTE		(2)
#define MCTP_TX_PAYLOAD_512BYTE		(3)


/*************************************************************************************/
//TX CMD desc0
#define BUS_NO(x)			((x & 0xff) << 24)
#define DEV_NO(x)			((x & 0x1f) << 19)
#define FUN_NO(x)			((x & 0x7) << 16)
#define STOP_INT_ENABLE			(0x1 << 15)

//ast-g5
/* 0: route to RC, 1: route by ID, 2/3: broadcast from RC */
#define G5_ROUTING_TYPE_L(x)		((x & 0x1) << 14)
#define G5_ROUTING_TYPE_H(x)		(((x & 0x2) >> 1) << 12)
//ast old version
#define ROUTING_TYPE(x)			((x & 0x1) << 14)

#define TAG_OWN				(1 << 13)
#define PKG_SIZE(x)			((x & 0x7ff) << 2)
#define PADDING_LEN(x)			(x & 0x3)
//TX CMD desc1
#define LAST_CMD			(1 << 31)
//ast-g6
#define G6_TX_DATA_ADDR(x)		(((x >> 2) & 0x1fffffff) << 2)
#define G6_TX_DESC_VALID		(0x1)
//ast-g5
#define G5_TX_DATA_ADDR(x)		(((x >> 7) & 0x7fffff) << 8)
//ast old version
#define TX_DATA_ADDR(x)			(((x >> 6) & 0x7fffff) << 8)

#define DEST_EP_ID(x)			(x & 0xff)
/*************************************************************************************/
//RX CMD desc0
#define GET_PKG_LEN(x)			((x >> 24) & 0x7f)
#define GET_SRC_EP_ID(x)		((x >> 16) & 0xff)
#define GET_ROUTING_TYPE(x)		((x >> 14) & 0x7)
#define GET_SEQ_NO(x)			((x >> 11) & 0x3)
#define MCTP_SOM			(1 << 7)
#define MCTP_EOM			(1 << 6)
#define GET_PADDING_LEN(x)		((x >> 4) & 0x3)
#define CMD_UPDATE			(1)
//RX CMD desc1
#define LAST_CMD			(1 << 31)
#define RX_DATA_ADDR(x)			(((x >> 7) & 0x3fffff) << 7)

struct ast_mctp_cmd_desc {
	unsigned int desc0;
	unsigned int desc1;
};

/*************************************************************************************/
#define MAX_XFER_BUFF_SIZE 1024

struct ast_mctp_xfer {
	unsigned char	xfer_buff[MAX_XFER_BUFF_SIZE];
	unsigned int xfer_len;
	unsigned int ep_id;
	unsigned int bus_no;
	unsigned int dev_no;
	unsigned int fun_no;
	unsigned char rt;

};

#define MCTPIOC_BASE       'M'

#define AST_MCTP_IOCRX			_IOWR(MCTPIOC_BASE, 0x10, struct ast_mctp_xfer)
#define AST_MCTP_IOCTX			_IOW(MCTPIOC_BASE, 0x11, struct ast_mctp_xfer)

/*************************************************************************************/
//#define AST_MCTP_DEBUG

#ifdef AST_MCTP_DEBUG
#define MCTP_DBUG(fmt, args...) printk("%s() " fmt,__FUNCTION__, ## args)
#else
#define MCTP_DBUG(fmt, args...)
#endif

#define MCTP_MSG(fmt, args...) printk(fmt, ## args)

struct ast_mctp_info {
	void __iomem	*reg_base;
	int irq;				//MCTP IRQ number
	struct reset_control *reset;
	int	ast_mctp_version;
	u32 dram_base;
	wait_queue_head_t mctp_wq;
	u8 *mctp_dma;
	dma_addr_t mctp_dma_addr;

	struct ast_mctp_cmd_desc *tx_cmd_desc;
	dma_addr_t tx_cmd_desc_dma;
	u8 *tx_data;
	dma_addr_t tx_data_dma;

	struct ast_mctp_cmd_desc *rx_cmd_desc;
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
ast_mctp_read(struct ast_mctp_info *ast_mctp, u32 reg)
{
	u32 val;

	val = readl(ast_mctp->reg_base + reg);
	MCTP_DBUG("reg = 0x%08x, val = 0x%08x\n", reg, val);
	return val;
}

static inline void
ast_mctp_write(struct ast_mctp_info *ast_mctp, u32 val, u32 reg)
{
	MCTP_DBUG("reg = 0x%08x, val = 0x%08x\n", reg, val);

	writel(val, ast_mctp->reg_base + reg);
}

/*************************************************************************************/
static void ast_mctp_wait_tx_complete(struct ast_mctp_info *ast_mctp)
{
	wait_event_interruptible(ast_mctp->mctp_wq, (ast_mctp->flag == MCTP_TX_LAST));
	MCTP_DBUG("\n");
	ast_mctp->flag = 0;
}

static void ast_mctp_wait_rx_complete(struct ast_mctp_info *ast_mctp)
{
	wait_event_interruptible(ast_mctp->mctp_wq, (ast_mctp->flag == MCTP_EOM));
	MCTP_DBUG("\n");
	ast_mctp->flag = 0;
}

static void ast_mctp_tx_xfer(struct ast_mctp_info *ast_mctp, struct ast_mctp_xfer *mctp_xfer)
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

	switch(ast_mctp->ast_mctp_version) {
		case 0:
			//routing type bit 14
			//bit 15 : interrupt enable
			//set default tag owner = 1;
			ast_mctp->tx_cmd_desc->desc0 = 0x0000a000 | ROUTING_TYPE(mctp_xfer->rt) | PKG_SIZE(xfer_len) | BUS_NO(mctp_xfer->bus_no) | DEV_NO(mctp_xfer->dev_no) | FUN_NO(mctp_xfer->fun_no) | PADDING_LEN(padding_len);
 			break;
		case 5:
	 		//routing type [desc0 bit 12, desc0 bit 14]
			//bit 15 : interrupt enable
			//set default tag owner = 1;
			ast_mctp->tx_cmd_desc->desc0 = 0x0000a000 | G5_ROUTING_TYPE_H(mctp_xfer->rt) | G5_ROUTING_TYPE_L(mctp_xfer->rt) | PKG_SIZE(xfer_len) | BUS_NO(mctp_xfer->bus_no) | DEV_NO(mctp_xfer->dev_no) | FUN_NO(mctp_xfer->fun_no) | PADDING_LEN(padding_len);
			break;
		case 6:
 			//bit 15 : interrupt enable
			//set default tag owner = 1;
			ast_mctp->tx_cmd_desc->desc0 = 0x0000a000 | PKG_SIZE(xfer_len);
			break;
	}

	//set dest ep id = 0;
	ast_mctp->tx_cmd_desc->desc1 |= LAST_CMD | DEST_EP_ID(0);

	MCTP_DBUG("memcpy  to %x from %x len = %d \n", ast_mctp->tx_data, mctp_xfer->xfer_buff, mctp_xfer->xfer_len);

	memset(ast_mctp->tx_data, 0,  1024);
	memcpy(ast_mctp->tx_data, mctp_xfer->xfer_buff,  mctp_xfer->xfer_len);

	//trigger tx
	ast_mctp_write(ast_mctp, ast_mctp_read(ast_mctp, ASPEED_MCTP_CTRL) | MCTP_TX_TRIGGER, ASPEED_MCTP_CTRL);

	//wait intr
	ast_mctp_wait_tx_complete(ast_mctp);

}

static void ast_mctp_rx_combine_data(struct ast_mctp_info *ast_mctp)
{
	int i;
	u32 rx_len = 0;
	u32 padding_len = 0;

	MCTP_DBUG(" \n");

	for (i = 0; i < 8; i++) {
		ast_mctp->rx_index %= 8;
		MCTP_DBUG("index %d, desc0 %x \n", ast_mctp->rx_index, ast_mctp->rx_cmd_desc[ast_mctp->rx_index].desc0);
		MCTP_DBUG("index %d, desc1 %x \n", ast_mctp->rx_index, ast_mctp->rx_cmd_desc[ast_mctp->rx_index].desc1);

		if (ast_mctp->rx_cmd_desc[ast_mctp->rx_index].desc0 & CMD_UPDATE) {
			if (ast_mctp->rx_fifo_done != 1) {
				MCTP_DBUG("rx fifo index %d \n", ast_mctp->rx_fifo_index);
				if (MCTP_SOM & ast_mctp->rx_cmd_desc[ast_mctp->rx_index].desc0) {
					MCTP_DBUG("MCTP_SOM \n");
					ast_mctp->rx_fifo_index = 0;
				}

				if (MCTP_EOM & ast_mctp->rx_cmd_desc[ast_mctp->rx_index].desc0) {
					MCTP_DBUG("MCTP_EOM ast_mctp->rx_fifo_done \n");
					ast_mctp->rx_fifo_done = 1;
					padding_len = GET_PADDING_LEN(ast_mctp->rx_cmd_desc[ast_mctp->rx_index].desc0);
					ast_mctp->flag = MCTP_EOM;
				}

				rx_len = GET_PKG_LEN(ast_mctp->rx_cmd_desc[ast_mctp->rx_index].desc0) * 4;
				rx_len -= padding_len;

				memcpy(ast_mctp->rx_fifo + (0x40 * ast_mctp->rx_fifo_index), ast_mctp->rx_data + (ast_mctp->rx_index * 0x80), rx_len);
				ast_mctp->rx_fifo_index++;

				ast_mctp->rx_len += rx_len;
				ast_mctp->ep_id = GET_SRC_EP_ID(ast_mctp->rx_cmd_desc[ast_mctp->rx_index].desc0);
				ast_mctp->rt = GET_ROUTING_TYPE(ast_mctp->rx_cmd_desc[ast_mctp->rx_index].desc0);
				ast_mctp->seq_no = GET_SEQ_NO(ast_mctp->rx_cmd_desc[ast_mctp->rx_index].desc0);
				MCTP_DBUG("rx len = %d , epid = %d, rt = %x, seq no = %d, total_len = %d \n", rx_len, ast_mctp->ep_id, ast_mctp->rt, ast_mctp->seq_no, ast_mctp->rx_len);

			} else {
				MCTP_DBUG("drop \n");
			}

			//RX CMD desc0
			ast_mctp->rx_cmd_desc[ast_mctp->rx_index].desc0 = 0;
			ast_mctp->rx_index++;

		} else {
			MCTP_DBUG("index %d break\n", ast_mctp->rx_index);
			break;
		}
	}
}

static irqreturn_t ast_mctp_isr(int this_irq, void *dev_id)
{
	struct ast_mctp_info *ast_mctp = dev_id;
	u32 status = ast_mctp_read(ast_mctp, ASPEED_MCTP_ISR);

	MCTP_DBUG("%x \n", status);

	if (status & MCTP_TX_LAST) {
		ast_mctp_write(ast_mctp, MCTP_TX_LAST | (status), ASPEED_MCTP_ISR);
		ast_mctp->flag = MCTP_TX_LAST;
	}

	if (status & MCTP_TX_COMPLETE) {
		ast_mctp_write(ast_mctp, MCTP_TX_COMPLETE | (status), ASPEED_MCTP_ISR);
		ast_mctp->flag = MCTP_TX_COMPLETE;
	}

	if (status & MCTP_RX_COMPLETE) {
		ast_mctp_write(ast_mctp, MCTP_RX_COMPLETE | (status), ASPEED_MCTP_ISR);
//		ast_mctp->flag = MCTP_RX_COMPLETE;
//		ast_mctp->flag = 0;
		ast_mctp_rx_combine_data(ast_mctp);
	}

	if (status & MCTP_RX_NO_CMD) {
		ast_mctp_write(ast_mctp, MCTP_RX_NO_CMD | (status), ASPEED_MCTP_ISR);
		ast_mctp->flag = MCTP_RX_NO_CMD;
		printk("MCTP_RX_NO_CMD \n");
	}

	if (ast_mctp->flag) {
		wake_up_interruptible(&ast_mctp->mctp_wq);
		return IRQ_HANDLED;
	} else {
		printk("TODO Check MCTP's interrupt %x\n", status);
		return IRQ_NONE;
	}

}

static void ast_mctp_ctrl_init(struct ast_mctp_info *ast_mctp)
{
	int i = 0;

	MCTP_DBUG("dram base %x \n", ast_mctp->dram_base);
	ast_mctp_write(ast_mctp, ast_mctp->dram_base, ASPEED_MCTP_EID);
	//4K size memory
	//1st 1024 : cmd desc --> 0~512 : tx desc , 512 ~ 1024 : rx desc
	//2nd 1024 : tx data
	//3rd 1024 : rx data --> 8 - 0x00 , 0x80 , 0x100, 0x180, each 128 align
	//4th 1024 : rx data combine

	//tx
	ast_mctp->mctp_dma = dma_alloc_coherent(NULL,
						4096,
						&ast_mctp->mctp_dma_addr, GFP_KERNEL);

	if (((u32)ast_mctp->mctp_dma & 0xff) != 0x00)
		printk("ERROR dma addr !!!!\n");


	ast_mctp->tx_cmd_desc = (struct ast_mctp_cmd_desc *)ast_mctp->mctp_dma;
	ast_mctp->tx_cmd_desc_dma = ast_mctp->mctp_dma_addr;

	MCTP_DBUG("tx cmd desc %x , cmd desc dma %x \n", (u32)ast_mctp->tx_cmd_desc, (u32)ast_mctp->tx_cmd_desc_dma);

	ast_mctp->tx_data = (u8 *)(ast_mctp->mctp_dma + 1024);
	ast_mctp->tx_data_dma = ast_mctp->mctp_dma_addr + 1024;

	for (i = 0; i < 1024; i++) {
		ast_mctp->tx_data[i] = i;
	}

	switch(ast_mctp->ast_mctp_version) {
		case 0:
			ast_mctp->tx_cmd_desc->desc1 |= TX_DATA_ADDR(ast_mctp->tx_data_dma);
			break;
		case 5:
			ast_mctp->tx_cmd_desc->desc1 |= G5_TX_DATA_ADDR(ast_mctp->tx_data_dma);
			break;
		case 6:
			ast_mctp->tx_cmd_desc->desc1 |= G6_TX_DATA_ADDR(ast_mctp->tx_data_dma);
			break;
	}
		
	MCTP_DBUG("tx data %x , tx data dma %x \n", (u32)ast_mctp->tx_data, (u32)ast_mctp->tx_data_dma);

	ast_mctp_write(ast_mctp, ast_mctp->tx_cmd_desc_dma, ASPEED_MCTP_TX_CMD);

	//RX 8 buffer
	ast_mctp->rx_cmd_desc = (struct ast_mctp_cmd_desc *)(ast_mctp->mctp_dma + 512);
	ast_mctp->rx_cmd_desc_dma = ast_mctp->mctp_dma_addr + 512;

	MCTP_DBUG("rx cmd desc %x , cmd desc dma %x \n", (u32)ast_mctp->rx_cmd_desc, (u32)ast_mctp->rx_cmd_desc_dma);

	ast_mctp->rx_data = (u8 *)(ast_mctp->mctp_dma + 2048);
	ast_mctp->rx_data_dma = ast_mctp->mctp_dma_addr + 2048;

	MCTP_DBUG("rx data %x , rx data dma %x \n", (u32)ast_mctp->rx_data, (u32)ast_mctp->rx_data_dma);

	ast_mctp->rx_index = 0;
	ast_mctp->rx_fifo = (u8 *)(ast_mctp->mctp_dma + 3072);
	MCTP_DBUG("ast_mctp->rx_fifo %x \n", (u32)ast_mctp->rx_fifo);

	ast_mctp->rx_fifo_done = 0;
	ast_mctp->rx_fifo_index = 0;
	ast_mctp->rx_len = 0;
	memset(ast_mctp->rx_fifo, 0,  1024);

	for (i = 0; i < 8; i++) {
		ast_mctp->rx_cmd_desc[i].desc0 = 0;
		ast_mctp->rx_cmd_desc[i].desc1 = RX_DATA_ADDR((ast_mctp->rx_data_dma + (0x80 * i))) ;
		if (i == 7)
			ast_mctp->rx_cmd_desc[i].desc1 |= LAST_CMD;
		MCTP_DBUG("Rx [%d]: desc0: %x , desc1: %x \n", i, ast_mctp->rx_cmd_desc[i].desc0, ast_mctp->rx_cmd_desc[i].desc1);
	}

//	MCTP_DBUG("rx cmd desc %x , data dma %x \n", ast_mctp->rx_cmd_desc_dma, ast_mctp->rx_data_dma);

	ast_mctp_write(ast_mctp, ast_mctp->rx_cmd_desc_dma, ASPEED_MCTP_RX_CMD);

	ast_mctp_write(ast_mctp, ast_mctp_read(ast_mctp, ASPEED_MCTP_CTRL) | MCTP_RX_CMD_RDY, ASPEED_MCTP_CTRL);

	ast_mctp_write(ast_mctp, MCTP_RX_COMPLETE | MCTP_TX_LAST, ASPEED_MCTP_IER);
}

static long mctp_ioctl(struct file *file, unsigned int cmd,
		       unsigned long arg)
{
	struct miscdevice *c = file->private_data;
	struct ast_mctp_info *ast_mctp = dev_get_drvdata(c->this_device);
	void __user *argp = (void __user *)arg;
	struct ast_mctp_xfer xfer;

	switch (cmd) {
	case AST_MCTP_IOCTX:
		MCTP_DBUG("AST_MCTP_IOCTX \n");
		if (copy_from_user(&xfer, argp, sizeof(struct ast_mctp_xfer))) {
			MCTP_DBUG("copy_from_user  fail\n");
			return -EFAULT;
		} else
			ast_mctp_tx_xfer(ast_mctp, &xfer);
		break;
	case AST_MCTP_IOCRX:
		MCTP_DBUG("AST_MCTP_IOCRX \n");
		//wait intr
//			if(!ast_mctp->rx_fifo_done) {
//				MCTP_DBUG("wait for rx fifo done \n");
		ast_mctp_wait_rx_complete(ast_mctp);
//			}
		xfer.xfer_len = ast_mctp->rx_len;
		memcpy(xfer.xfer_buff, ast_mctp->rx_fifo,  ast_mctp->rx_len);
		if (copy_to_user(argp, &xfer, sizeof(struct ast_mctp_xfer)))
			return -EFAULT;
		else {
			ast_mctp->rx_fifo_done = 0;
			ast_mctp->rx_len = 0;
			memset(ast_mctp->rx_fifo, 0,  1024);
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
	struct ast_mctp_info *ast_mctp = dev_get_drvdata(c->this_device);

	MCTP_DBUG("\n");
	spin_lock(&mctp_state_lock);

	if (ast_mctp->is_open) {
		spin_unlock(&mctp_state_lock);
		return -EBUSY;
	}

	ast_mctp->is_open = true;

	spin_unlock(&mctp_state_lock);

	return 0;
}

static int mctp_release(struct inode *inode, struct file *file)
{
	struct miscdevice *c = file->private_data;
	struct ast_mctp_info *ast_mctp = dev_get_drvdata(c->this_device);

	MCTP_DBUG("\n");
	spin_lock(&mctp_state_lock);

	ast_mctp->is_open = false;

	spin_unlock(&mctp_state_lock);

	return 0;
}

static const struct file_operations ast_mctp_fops = {
	.owner			= THIS_MODULE,
	.unlocked_ioctl	= mctp_ioctl,
	.open			= mctp_open,
	.release			= mctp_release,
};

static struct miscdevice ast_mctp_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "ast-mctp",
	.fops = &ast_mctp_fops,
};

static const struct of_device_id ast_mctp_of_matches[] = {
	{ .compatible = "aspeed,ast-mctp", 		.data = (void *) 0, },
	{ .compatible = "aspeed,ast-g5-mctp", 	.data = (void *) 5, },
	{ .compatible = "aspeed,ast-g6-mctp", 	.data = (void *) 6, },
	{},
};
MODULE_DEVICE_TABLE(of, ast_mctp_of_matches);

static int ast_mctp_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct ast_mctp_info *ast_mctp;
	const struct of_device_id *mctp_dev_id;	
	int ret = 0;

	MCTP_DBUG("\n");

	if (!(ast_mctp = devm_kzalloc(&pdev->dev, sizeof(struct ast_mctp_info), GFP_KERNEL))) {
		return -ENOMEM;
	}

	mctp_dev_id = of_match_device(ast_mctp_of_matches, &pdev->dev);
	if (!mctp_dev_id)
		return -EINVAL;

	ast_mctp->ast_mctp_version = (unsigned long)mctp_dev_id->data;

	if (of_device_is_compatible(pdev->dev.of_node, "aspeed,ast-g5-mctp"))
		ast_mctp->ast_mctp_version = 1;
	else
		ast_mctp->ast_mctp_version = 0;



	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (NULL == res) {
		dev_err(&pdev->dev, "cannot get IORESOURCE_MEM\n");
		ret = -ENOENT;
		goto out;
	}

	ast_mctp->reg_base = devm_ioremap_resource(&pdev->dev, res);
	if (!ast_mctp->reg_base) {
		ret = -EIO;
		goto out_region;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (NULL == res) {
		dev_err(&pdev->dev, "cannot get IORESOURCE_BUS\n");
		ret = -ENOENT;
		goto out_region;
	}

	ast_mctp->dram_base = (u32)res->start;

	ast_mctp->irq = platform_get_irq(pdev, 0);
	if (ast_mctp->irq < 0) {
		dev_err(&pdev->dev, "no irq specified\n");
		ret = -ENOENT;
		goto out_region;
	}

	ast_mctp->reset = devm_reset_control_get_exclusive(&pdev->dev, "mctp");
	if (IS_ERR(ast_mctp->reset)) {
		dev_err(&pdev->dev, "can't get mctp reset\n");
		return PTR_ERR(ast_mctp->reset);
	}

	//scu init
	reset_control_assert(ast_mctp->reset);
	reset_control_deassert(ast_mctp->reset);

	ast_mctp->flag = 0;
	init_waitqueue_head(&ast_mctp->mctp_wq);

	ret = misc_register(&ast_mctp_misc);
	if (ret) {
		printk(KERN_ERR "MCTP : failed to request interrupt\n");
		goto out_region;
	}

	platform_set_drvdata(pdev, ast_mctp);
	dev_set_drvdata(ast_mctp_misc.this_device, ast_mctp);

	ast_mctp_ctrl_init(ast_mctp);

	ret = devm_request_irq(&pdev->dev, ast_mctp->irq, ast_mctp_isr,
			       0, dev_name(&pdev->dev), ast_mctp);
	if (ret) {
		printk("MCTP Unable to get IRQ");
		goto out_irq;
	}

#if 0
	ret = sysfs_create_group(&ast_mctp_misc.this_device->kobj,
				 &mctp_attribute_group);
	if (ret) {
		printk(KERN_ERR "lattice: failed to create sysfs device attributes.\n");
		return -1;
	}
#endif
	printk(KERN_INFO "ast_mctp: driver successfully loaded.\n");

	return 0;


out_irq:
	free_irq(ast_mctp->irq, NULL);
out_region:
	release_mem_region(res->start, res->end - res->start + 1);
out:
	printk(KERN_WARNING "ast_mctp: driver init failed (ret=%d)!\n", ret);
	return ret;
}

static int ast_mctp_remove(struct platform_device *pdev)
{
	struct resource *res;
	struct ast_mctp_info *ast_mctp = platform_get_drvdata(pdev);

	MCTP_DBUG("\n");

	misc_deregister(&ast_mctp_misc);

	free_irq(ast_mctp->irq, ast_mctp);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	iounmap(ast_mctp->reg_base);

	platform_set_drvdata(pdev, NULL);

	release_mem_region(res->start, res->end - res->start + 1);

	return 0;
}

#ifdef CONFIG_PM
static int
ast_mctp_suspend(struct platform_device *pdev, pm_message_t state)
{
	return 0;
}

static int
ast_mctp_resume(struct platform_device *pdev)
{
	return 0;
}

#else
#define ast_mctp_suspend        NULL
#define ast_mctp_resume         NULL
#endif

static struct platform_driver ast_mctp_driver = {
	.probe 		= ast_mctp_probe,
	.remove 		= ast_mctp_remove,
#ifdef CONFIG_PM
	.suspend        = ast_mctp_suspend,
	.resume         = ast_mctp_resume,
#endif
	.driver         = {
		.name   = KBUILD_MODNAME,
		.of_match_table = ast_mctp_of_matches,
	},
};

module_platform_driver(ast_mctp_driver);

MODULE_AUTHOR("Ryan Chen <ryan_chen@aspeedtech.com>");
MODULE_DESCRIPTION("AST MCTP Driver");
MODULE_LICENSE("GPL");
