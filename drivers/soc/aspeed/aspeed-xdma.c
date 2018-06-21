/*
 * xdma driver for the Aspeed SoC
 *
 * Copyright (C) ASPEED Technology Inc.
 * Ryan Chen <ryan_chen@aspeedtech.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <linux/sysfs.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/reset.h>
#include <asm/io.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <asm/uaccess.h>
/* register ************************************************************************************/
#define ASPEED_XDMA_HOST_CMDQ_LOW 		0x00
#define ASPEED_XDMA_HOST_CMDQ_ENDP 		0x04
#define ASPEED_XDMA_HOST_CMDQ_WRITEP 	0x08
#define ASPEED_XDMA_HOST_CMDQ_READP 	0x0C
#define ASPEED_XDMA_BMC_CMDQ_BASE 		0x10
#define ASPEED_XDMA_BMC_CMDQ_ENDP 		0x14
#define ASPEED_XDMA_BMC_CMDQ_WRITEP 	0x18
#define ASPEED_XDMA_BMC_CMDQ_READP 		0x1C
#define ASPEED_XDMA_CTRL_IER 				0x20
#define ASPEED_XDMA_CTRL_ISR 				0x24
#define ASPEED_XDMA_DS_TX_SIZE			0x28
#define ASPEED_XDMA_DS_PCIE				0x30
#define ASPEED_XDMA_US_PCIE				0x34
#define ASPEED_XDMA_DS_CMD1				0x38
#define ASPEED_XDMA_DS_CMD2				0x3C
#define ASPEED_XDMA_US_CMD0_LOW			0x40
#define ASPEED_XDMA_US_CMD0_HIGH		0x44
#define ASPEED_XDMA_US_CMD1_LOW			0x48
#define ASPEED_XDMA_US_CMD1_HIGH		0x4C
#define ASPEED_XDMA_US_CMD2_LOW			0x50
#define ASPEED_XDMA_US_CMD2_HIGH		0x54
#define ASPEED_XDMA_HOST_CMDQ_HIGH 		0x60

/* ASPEED_XDMA_CTRL_IER - 0x20 : Interrupt Enable and Engine Control */
#define XDMA_PCIE_64Bits_MODE_EN		(1 << 31)
#define XDMA_CK_DS_CMD_ID_EN			(1 << 29)
#define XDMA_DS_DATA_TO_EN			(1 << 28)
#define XDMA_DS_PKS_256				(1 << 17)
#define XDMA_DS_PKS_512				(2 << 17)
#define XDMA_DS_PKS_1K					(3 << 17)
#define XDMA_DS_PKS_2K					(4 << 17)
#define XDMA_DS_PKS_4K					(5 << 17)

#define XDMA_DS_DIRTY_FRAME			(1 << 6)
#define XDMA_DS_COMPLETE				(1 << 5)
#define XDMA_US_COMPLETE				(1 << 4)
/*************************************************************************************/
#define ASPEED_XDMA_CMD_DESC_NUM		2

//CMD0 Format	(0x00)
#define PCIE_DATA_ADDR(x)				(x << 3)
//CMD1 Format	(0x08)
#define UP_STREAM_XFER					(1 << 31)
//ast -g 5
//16byte align
#define G5_BYTE_ALIGN						16
#define G5_BMC_ADDR(x)						(x & 0x3ffffff0)
//old ast soc
//8byte align
#define BYTE_ALIGN						8
#define BMC_ADDR(x)						(x & 0x1ffffff8)

#define CMD1_XFER_ID							(1)

//CMD2 Format	(0x10)
#define INTER_CMD_FINISH						(1 << 31)
#define FRAM_LINE_NUM(x)						(x << 16)
#define INTER_DIRECTION_BMC					(1 << 15)
//g5
#define G5_FRAM_LINE_BYTE(x)						((x & 0x7ff) << 4)
//old soc
#define FRAM_LINE_BYTE(x)						((x & 0xfff) << 3)

#define CMD2_XFER_ID							(2)
/*************************************************************************************/
//ast g5
#define G5_UPDATE_WRITE_POINT				4
#define G5_DEFAULT_END_POINT					8
struct aspeed_g5_xdma_cmd_desc {
	u32 cmd0_low;
	u32 cmd0_high;
	u32 cmd1_low;
	u32 cmd1_high;
	u32 cmd2_low;
	u32 cmd2_high;
	u32 resv_low;
	u32 resv_high;
};

#define UPDATE_WRITE_POINT				3
#define DEFAULT_END_POINT					5
struct aspeed_xdma_cmd_desc {
	u32 cmd0_low;
	u32 cmd0_high;
	u32 cmd1_low;
	u32 cmd1_high;
	u32 cmd2_low;
	u32 cmd2_high;
};

/*************************************************************************************/
#define MAX_XFER_BUFF_SIZE 4096

struct aspeed_xdma_xfer {
	unsigned char stream_dir;
	unsigned char	xfer_buff[MAX_XFER_BUFF_SIZE];
	unsigned int xfer_len;
	unsigned int bmc_addr;
	unsigned int host_addr_low;
	unsigned int host_addr_high;
};

#define XDMAIOC_BASE       'D'

#define ASPEED_XDMA_IOCXFER		_IOWR(XDMAIOC_BASE, 0x0, struct aspeed_xdma_xfer)
/*************************************************************************************/
//#define ASPEED_XDMA_DEBUG

#ifdef ASPEED_XDMA_DEBUG
#define XDMA_DBUG(fmt, args...) printk(KERN_DEBUG "%s() " fmt,__FUNCTION__, ## args)
#else
#define XDMA_DBUG(fmt, args...)
#endif

#define XDMA_MSG(fmt, args...) printk(fmt, ## args)

struct aspeed_xdma_info {
	void __iomem	*reg_base;
	int irq;				//XDMA IRQ number
	struct reset_control *reset;
	u32 dram_base;
	u8 				aspeed_g5;
	wait_queue_head_t xdma_wq;

	u8 desc_index;
	struct aspeed_xdma_cmd_desc *xfer_cmd_desc;
	struct aspeed_g5_xdma_cmd_desc *g5_xfer_cmd_desc;
	dma_addr_t xfer_cmd_desc_dma;

	u8 *xfer_data;
	dma_addr_t xfer_data_dma;

	u32 flag;
	bool is_open;
	u32 state;
};

/******************************************************************************/
static DEFINE_SPINLOCK(xdma_state_lock);
/******************************************************************************/

static inline u32
aspeed_xdma_read(struct aspeed_xdma_info *aspeed_xdma, u32 reg)
{
	u32 val;

	val = readl(aspeed_xdma->reg_base + reg);
	XDMA_DBUG("reg = 0x%08x, val = 0x%08x\n", reg, val);
	return val;
}

static inline void
aspeed_xdma_write(struct aspeed_xdma_info *aspeed_xdma, u32 val, u32 reg)
{
	XDMA_DBUG("reg = 0x%08x, val = 0x%08x\n", reg, val);
	writel(val, aspeed_xdma->reg_base + reg);
}

/*************************************************************************************/
void aspeed_xdma_wait_us_complete(struct aspeed_xdma_info *aspeed_xdma)
{
	wait_event_interruptible(aspeed_xdma->xdma_wq, (aspeed_xdma->flag == XDMA_US_COMPLETE));
	XDMA_DBUG("\n");
	aspeed_xdma->flag = 0;
}

void aspeed_xdma_wait_ds_complete(struct aspeed_xdma_info *aspeed_xdma)
{
	wait_event_interruptible(aspeed_xdma->xdma_wq, (aspeed_xdma->flag == XDMA_DS_COMPLETE));
	XDMA_DBUG("\n");
	aspeed_xdma->flag = 0;
}

static void aspeed_xdma_xfer(struct aspeed_xdma_info *aspeed_xdma, struct aspeed_xdma_xfer *xdma_xfer)
{
	u32 xfer_len = 0;
	u32 bmc_addr = 0;

	XDMA_DBUG("\n");
	if (xdma_xfer->bmc_addr == 0)
		bmc_addr = aspeed_xdma->xfer_data_dma;
	else
		bmc_addr = xdma_xfer->bmc_addr;

	aspeed_xdma->desc_index %= 2;

	XDMA_DBUG("cmd index [%x] : bmc addr : %x , host addr : %x (L) %x (H), size : %d \n", aspeed_xdma->desc_index, bmc_addr, xdma_xfer->host_addr_low, xdma_xfer->host_addr_high, xdma_xfer->xfer_len);

	if (xdma_xfer->xfer_len % BYTE_ALIGN)
		xfer_len = (xdma_xfer->xfer_len / BYTE_ALIGN) + 1;
	else
		xfer_len = xdma_xfer->xfer_len / BYTE_ALIGN;

	aspeed_xdma->xfer_cmd_desc[aspeed_xdma->desc_index].cmd0_high = xdma_xfer->host_addr_high;
	aspeed_xdma->xfer_cmd_desc[aspeed_xdma->desc_index].cmd0_low = xdma_xfer->host_addr_low;

	if (xdma_xfer->stream_dir) {
		aspeed_xdma->xfer_cmd_desc[aspeed_xdma->desc_index].cmd1_low = UP_STREAM_XFER | BMC_ADDR(bmc_addr) | CMD1_XFER_ID;
		XDMA_DBUG("US cmd desc %x \n", aspeed_xdma->xfer_cmd_desc[aspeed_xdma->desc_index].cmd1_low);
		if (xdma_xfer->bmc_addr == 0)
			memcpy(aspeed_xdma->xfer_data, xdma_xfer->xfer_buff,  xdma_xfer->xfer_len);
	} else {
		aspeed_xdma->xfer_cmd_desc[aspeed_xdma->desc_index].cmd1_low = BMC_ADDR(bmc_addr) | CMD1_XFER_ID;
		XDMA_DBUG("DS cmd desc %x \n", aspeed_xdma->xfer_cmd_desc[aspeed_xdma->desc_index].cmd1_low);
		memset(aspeed_xdma->xfer_data, 0,  4096);
	}

	aspeed_xdma->xfer_cmd_desc[aspeed_xdma->desc_index].cmd2_low = INTER_CMD_FINISH | FRAM_LINE_NUM(1) | INTER_DIRECTION_BMC | FRAM_LINE_BYTE(xfer_len) | CMD2_XFER_ID;

	//trigger tx
	if (aspeed_xdma->desc_index == 0)
		aspeed_xdma_write(aspeed_xdma, UPDATE_WRITE_POINT, ASPEED_XDMA_BMC_CMDQ_WRITEP);
	else
		aspeed_xdma_write(aspeed_xdma, 0, ASPEED_XDMA_BMC_CMDQ_WRITEP);

	aspeed_xdma->desc_index++;

	if (xdma_xfer->stream_dir) {
		aspeed_xdma_wait_us_complete(aspeed_xdma);
	} else {
		aspeed_xdma_wait_ds_complete(aspeed_xdma);
		if (xdma_xfer->bmc_addr == 0)
			memcpy(xdma_xfer->xfer_buff, aspeed_xdma->xfer_data, xdma_xfer->xfer_len);
	}

}

static void aspeed_g5_xdma_xfer(struct aspeed_xdma_info *aspeed_xdma, struct aspeed_xdma_xfer *xdma_xfer)
{
	u32 xfer_len = 0;
	u32 bmc_addr = 0;

	XDMA_DBUG("\n");
	if (xdma_xfer->bmc_addr == 0)
		bmc_addr = aspeed_xdma->xfer_data_dma;
	else
		bmc_addr = xdma_xfer->bmc_addr;

	aspeed_xdma->desc_index %= 2;

	XDMA_DBUG("cmd index [%x] : bmc addr : %x , host addr : %x (L) %x (H), size : %d \n", aspeed_xdma->desc_index, bmc_addr, xdma_xfer->host_addr_low, xdma_xfer->host_addr_high, xdma_xfer->xfer_len);

	if (xdma_xfer->xfer_len % G5_BYTE_ALIGN)
		xfer_len = (xdma_xfer->xfer_len / G5_BYTE_ALIGN) + 1;
	else
		xfer_len = xdma_xfer->xfer_len / G5_BYTE_ALIGN;

	aspeed_xdma->xfer_cmd_desc[aspeed_xdma->desc_index].cmd0_high = xdma_xfer->host_addr_high;
	aspeed_xdma->xfer_cmd_desc[aspeed_xdma->desc_index].cmd0_low = xdma_xfer->host_addr_low;

	if (xdma_xfer->stream_dir) {
		aspeed_xdma->xfer_cmd_desc[aspeed_xdma->desc_index].cmd1_low = UP_STREAM_XFER | G5_BMC_ADDR(bmc_addr) | CMD1_XFER_ID;
		XDMA_DBUG("US cmd desc %x \n", aspeed_xdma->xfer_cmd_desc[aspeed_xdma->desc_index].cmd1_low);
		if (xdma_xfer->bmc_addr == 0)
			memcpy(aspeed_xdma->xfer_data, xdma_xfer->xfer_buff,  xdma_xfer->xfer_len);
	} else {
		aspeed_xdma->xfer_cmd_desc[aspeed_xdma->desc_index].cmd1_low = G5_BMC_ADDR(bmc_addr) | CMD1_XFER_ID;
		XDMA_DBUG("DS cmd desc %x \n", aspeed_xdma->xfer_cmd_desc[aspeed_xdma->desc_index].cmd1_low);
		memset(aspeed_xdma->xfer_data, 0,  4096);
	}

	aspeed_xdma->xfer_cmd_desc[aspeed_xdma->desc_index].cmd2_low = INTER_CMD_FINISH | FRAM_LINE_NUM(1) | INTER_DIRECTION_BMC | G5_FRAM_LINE_BYTE(xfer_len) | CMD2_XFER_ID;

	//trigger tx
	if (aspeed_xdma->desc_index == 0)
		aspeed_xdma_write(aspeed_xdma, G5_UPDATE_WRITE_POINT, ASPEED_XDMA_BMC_CMDQ_WRITEP);
	else
		aspeed_xdma_write(aspeed_xdma, 0, ASPEED_XDMA_BMC_CMDQ_WRITEP);

	aspeed_xdma->desc_index++;

	if (xdma_xfer->stream_dir) {
		aspeed_xdma_wait_us_complete(aspeed_xdma);
	} else {
		aspeed_xdma_wait_ds_complete(aspeed_xdma);
		if (xdma_xfer->bmc_addr == 0)
			memcpy(xdma_xfer->xfer_buff, aspeed_xdma->xfer_data, xdma_xfer->xfer_len);
	}

}

static irqreturn_t aspeed_xdma_isr(int this_irq, void *dev_id)
{
	struct aspeed_xdma_info *aspeed_xdma = dev_id;
	u32 status = aspeed_xdma_read(aspeed_xdma, ASPEED_XDMA_CTRL_ISR);


	XDMA_DBUG("%x \n", status);


	if (status & XDMA_DS_DIRTY_FRAME) {
		aspeed_xdma->flag = XDMA_DS_DIRTY_FRAME;
		XDMA_DBUG("XDMA_DS_DIRTY_FRAME \n");
	}

	if (status & XDMA_DS_COMPLETE) {
		aspeed_xdma->flag = XDMA_DS_COMPLETE;
		XDMA_DBUG("XDMA_DS_COMPLETE \n");
	}

	if (status & XDMA_US_COMPLETE) {
		aspeed_xdma->flag = XDMA_US_COMPLETE;
		XDMA_DBUG("XDMA_US_COMPLETE \n");
	}

	aspeed_xdma_write(aspeed_xdma, status, ASPEED_XDMA_CTRL_ISR);

	if (aspeed_xdma->flag) {
		wake_up_interruptible(&aspeed_xdma->xdma_wq);
		return IRQ_HANDLED;
	} else {
		printk("TODO Check MCTP's interrupt %x\n", status);
		return IRQ_NONE;
	}

}

static void aspeed_xdma_ctrl_init(struct aspeed_xdma_info *aspeed_xdma)
{
	//xfer buff
	aspeed_xdma->xfer_data = dma_alloc_coherent(NULL,
						  4096,
						  &aspeed_xdma->xfer_data_dma, GFP_KERNEL);

	XDMA_DBUG("xfer buff %x , dma %x \n", (u32)aspeed_xdma->xfer_data, (u32)aspeed_xdma->xfer_data_dma);

	//tx cmd
	aspeed_xdma->xfer_cmd_desc = dma_alloc_coherent(NULL,
							  sizeof(struct aspeed_xdma_cmd_desc) * ASPEED_XDMA_CMD_DESC_NUM,
							  &aspeed_xdma->xfer_cmd_desc_dma, GFP_KERNEL);

	if (((u32)aspeed_xdma->xfer_cmd_desc & 0xff) != 0x00)
		printk("ERROR dma addr !!!!\n");

	XDMA_DBUG("xfer cmd desc %x , cmd desc dma %x \n", (u32)aspeed_xdma->xfer_cmd_desc, (u32)aspeed_xdma->xfer_cmd_desc_dma);

	memset(aspeed_xdma->xfer_cmd_desc, 0,  sizeof(struct aspeed_xdma_cmd_desc) * 2);

	aspeed_xdma->xfer_cmd_desc[0].cmd1_high = 0x00080008;
	aspeed_xdma->xfer_cmd_desc[1].cmd1_high = 0x00080008;
	aspeed_xdma->desc_index = 0;

	aspeed_xdma_write(aspeed_xdma, aspeed_xdma->xfer_cmd_desc_dma, ASPEED_XDMA_BMC_CMDQ_BASE);
	aspeed_xdma_write(aspeed_xdma, DEFAULT_END_POINT, ASPEED_XDMA_BMC_CMDQ_ENDP);
	aspeed_xdma_write(aspeed_xdma, 0, ASPEED_XDMA_BMC_CMDQ_WRITEP);

	//register
	aspeed_xdma_write(aspeed_xdma, XDMA_CK_DS_CMD_ID_EN | XDMA_DS_DATA_TO_EN | XDMA_DS_PKS_256 |
				   XDMA_DS_DIRTY_FRAME | XDMA_DS_COMPLETE | XDMA_US_COMPLETE, ASPEED_XDMA_CTRL_IER);

}

static void aspeed_g5_xdma_ctrl_init(struct aspeed_xdma_info *aspeed_xdma)
{
	//xfer buff
	aspeed_xdma->xfer_data = dma_alloc_coherent(NULL,
						  4096,
						  &aspeed_xdma->xfer_data_dma, GFP_KERNEL);

	XDMA_DBUG("xfer buff %x , dma %x \n", (u32)aspeed_xdma->xfer_data, (u32)aspeed_xdma->xfer_data_dma);

	//tx cmd
	aspeed_xdma->xfer_cmd_desc = dma_alloc_coherent(NULL,
							  sizeof(struct aspeed_g5_xdma_cmd_desc) * ASPEED_XDMA_CMD_DESC_NUM,
							  &aspeed_xdma->xfer_cmd_desc_dma, GFP_KERNEL);

	if (((u32)aspeed_xdma->xfer_cmd_desc & 0xff) != 0x00)
		printk("ERROR dma addr !!!!\n");

	XDMA_DBUG("xfer cmd desc %x , cmd desc dma %x \n", (u32)aspeed_xdma->xfer_cmd_desc, (u32)aspeed_xdma->xfer_cmd_desc_dma);

	memset(aspeed_xdma->xfer_cmd_desc, 0,  sizeof(struct aspeed_g5_xdma_cmd_desc) * 2);

	aspeed_xdma->xfer_cmd_desc[0].cmd1_high = 0x00080008;
	aspeed_xdma->xfer_cmd_desc[1].cmd1_high = 0x00080008;
	aspeed_xdma->desc_index = 0;

	aspeed_xdma_write(aspeed_xdma, aspeed_xdma->xfer_cmd_desc_dma, ASPEED_XDMA_BMC_CMDQ_BASE);
	aspeed_xdma_write(aspeed_xdma, G5_DEFAULT_END_POINT, ASPEED_XDMA_BMC_CMDQ_ENDP);
	aspeed_xdma_write(aspeed_xdma, 0, ASPEED_XDMA_BMC_CMDQ_WRITEP);

	//register
	aspeed_xdma_write(aspeed_xdma, XDMA_CK_DS_CMD_ID_EN | XDMA_DS_DATA_TO_EN | XDMA_DS_PKS_256 |
				   XDMA_DS_DIRTY_FRAME | XDMA_DS_COMPLETE | XDMA_US_COMPLETE, ASPEED_XDMA_CTRL_IER);

}

static long xdma_ioctl(struct file *file, unsigned int cmd,
					   unsigned long arg)
{
	int ret = 0;
	struct miscdevice *c = file->private_data;
	struct aspeed_xdma_info *aspeed_xdma = dev_get_drvdata(c->this_device);
	void  *argp = (void *)arg;
	struct aspeed_xdma_xfer xfer;

	switch (cmd) {
	case ASPEED_XDMA_IOCXFER:
		XDMA_DBUG("ASPEED_XDMA_IOCXFER \n");
		if (copy_from_user(&xfer, argp, sizeof(struct aspeed_xdma_xfer))) {
			printk("copy_from_user  fail\n");
			ret = -EFAULT;
		} else {
			if (aspeed_xdma->aspeed_g5)
				aspeed_g5_xdma_xfer(aspeed_xdma, &xfer);
			else
				aspeed_xdma_xfer(aspeed_xdma, &xfer);
		}

		if (!xfer.stream_dir) {
			if (copy_to_user(argp, &xfer, sizeof(struct aspeed_xdma_xfer)))
				ret = -EFAULT;
		}
		break;
	default:
		XDMA_DBUG("ERROR \n");
		return -ENOTTY;
	}

	return ret;
}

static int xdma_open(struct inode *inode, struct file *file)
{
	struct miscdevice *c = file->private_data;
	struct aspeed_xdma_info *aspeed_xdma = dev_get_drvdata(c->this_device);

	XDMA_DBUG("\n");
	spin_lock(&xdma_state_lock);

	if (aspeed_xdma->is_open) {
		spin_unlock(&xdma_state_lock);
		return -EBUSY;
	}

	aspeed_xdma->is_open = true;

	spin_unlock(&xdma_state_lock);

	return 0;
}

static int xdma_release(struct inode *inode, struct file *file)
{
	struct miscdevice *c = file->private_data;
	struct aspeed_xdma_info *aspeed_xdma = dev_get_drvdata(c->this_device);

	XDMA_DBUG("\n");
	spin_lock(&xdma_state_lock);

	aspeed_xdma->is_open = false;

	spin_unlock(&xdma_state_lock);

	return 0;
}

static const struct file_operations aspeed_xdma_fops = {
	.owner			= THIS_MODULE,
	.unlocked_ioctl	= xdma_ioctl,
	.open			= xdma_open,
	.release			= xdma_release,
};

struct miscdevice aspeed_xdma_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "aspeed-xdma",
	.fops = &aspeed_xdma_fops,
};

static int aspeed_xdma_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct aspeed_xdma_info *aspeed_xdma;
	int ret = 0;

	XDMA_DBUG("\n");

	if (!(aspeed_xdma = devm_kzalloc(&pdev->dev, sizeof(struct aspeed_xdma_info), GFP_KERNEL))) {
		return -ENOMEM;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (NULL == res) {
		dev_err(&pdev->dev, "cannot get IORESOURCE_MEM\n");
		ret = -ENOENT;
		goto out;
	}

	aspeed_xdma->reg_base = devm_ioremap_resource(&pdev->dev, res);
	if (!aspeed_xdma->reg_base) {
		ret = -EIO;
		goto out_region;
	}

	aspeed_xdma->irq = platform_get_irq(pdev, 0);
	if (aspeed_xdma->irq < 0) {
		dev_err(&pdev->dev, "no irq specified\n");
		ret = -ENOENT;
		goto out_region;
	}

	aspeed_xdma->reset = devm_reset_control_get(&pdev->dev, NULL);
	if (IS_ERR(aspeed_xdma->reset)) {
		dev_err(&pdev->dev, "can't get xmda reset\n");
		return PTR_ERR(aspeed_xdma->reset);
	}

	//scu init
	reset_control_deassert(aspeed_xdma->reset);

	ret = devm_request_irq(&pdev->dev, aspeed_xdma->irq, aspeed_xdma_isr, 0, dev_name(&pdev->dev), aspeed_xdma);
	if (ret) {
		printk("MCTP Unable to get IRQ");
		goto out_region;
	}

	if (of_machine_is_compatible("aspeed,ast2500"))
		aspeed_xdma->aspeed_g5 = 1;
	else
		aspeed_xdma->aspeed_g5 = 0;

	aspeed_xdma->flag = 0;
	init_waitqueue_head(&aspeed_xdma->xdma_wq);

	ret = misc_register(&aspeed_xdma_misc);
	if (ret) {
		printk(KERN_ERR "MCTP : failed to request interrupt\n");
		goto out_irq;
	}

	platform_set_drvdata(pdev, aspeed_xdma);
	dev_set_drvdata(aspeed_xdma_misc.this_device, aspeed_xdma);

	if (aspeed_xdma->aspeed_g5)
		aspeed_g5_xdma_ctrl_init(aspeed_xdma);
	else
		aspeed_xdma_ctrl_init(aspeed_xdma);

	printk(KERN_INFO "aspeed_xdma: driver successfully loaded.\n");

	return 0;


out_irq:
	free_irq(aspeed_xdma->irq, NULL);
out_region:
	release_mem_region(res->start, res->end - res->start + 1);
out:
	printk(KERN_WARNING "applesmc: driver init failed (ret=%d)!\n", ret);
	return ret;
}

static int aspeed_xdma_remove(struct platform_device *pdev)
{
	struct resource *res;
	struct aspeed_xdma_info *aspeed_xdma = platform_get_drvdata(pdev);

	XDMA_DBUG("\n");

	misc_deregister(&aspeed_xdma_misc);

	free_irq(aspeed_xdma->irq, aspeed_xdma);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	iounmap(aspeed_xdma->reg_base);

	platform_set_drvdata(pdev, NULL);

	release_mem_region(res->start, res->end - res->start + 1);

	return 0;
}

#ifdef CONFIG_PM
static int
aspeed_xdma_suspend(struct platform_device *pdev, pm_message_t state)
{
	return 0;
}

static int
aspeed_xdma_resume(struct platform_device *pdev)
{
	return 0;
}

#else
#define aspeed_xdma_suspend        NULL
#define aspeed_xdma_resume         NULL
#endif

static const struct of_device_id aspeed_xdma_match[] = {
	{ .compatible = "aspeed,aspeed-xdma" },
	{ },
};

static struct platform_driver aspeed_xdma_driver = {
	.probe		= aspeed_xdma_probe,
	.remove 		= aspeed_xdma_remove,
#ifdef CONFIG_PM
	.suspend		= aspeed_xdma_suspend,
	.resume 		= aspeed_xdma_resume,
#endif
	.driver = {
		.name		= "aspeed-xdma",
		.of_match_table = aspeed_xdma_match,
	},
};

module_platform_driver(aspeed_xdma_driver);

MODULE_AUTHOR("Ryan Chen <ryan_chen@aspeedtech.com>");
MODULE_DESCRIPTION("ASPEED X-DMA Driver");
MODULE_LICENSE("GPL");
