/*
 * ast-xdma.c - xdma driver for the Aspeed SoC
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
#define AST_XDMA_HOST_CMDQ_LADDR	0x00
#define AST_XDMA_HOST_CMDQ_HADDR 	0x04
#define AST_XDMA_HOST_CMDQ_ENDP 	0x08
#define AST_XDMA_HOST_CMDQ_WRITEP 	0x0C
#define AST_XDMA_HOST_CMDQ_READP 	0x10
#define AST_XDMA_BMC_CMDQ_BASE 		0x14
#define AST_XDMA_BMC_CMDQ_ENDP 		0x18
#define AST_XDMA_BMC_CMDQ_WRITEP 	0x1C
#define AST_XDMA_BMC_CMDQ_READP 	0x20
#define AST_XDMA_VGA_CMDQ_LADDR		0x24
#define AST_XDMA_VGA_CMDQ_HADDR 	0x28
#define AST_XDMA_VGA_CMDQ_ENDP 		0x2C
#define AST_XDMA_VGA_CMDQ_WRITEP 	0x30
#define AST_XDMA_VGA_CMDQ_READP 	0x34
#define AST_XDMA_CTRL_IER 			0x38
#define AST_XDMA_CTRL_ISR 			0x3C
#define AST_XDMA_DS_CMD0_LOW		0x40
#define AST_XDMA_DS_CMD0_HIGH		0x44
#define AST_XDMA_DS_CMD1_LOW		0x48
#define AST_XDMA_DS_CMD1_HIGH		0x4C
#define AST_XDMA_DS_CMD2_LOW		0x50
#define AST_XDMA_DS_CMD2_HIGH		0x54
#define AST_XDMA_US_CMD0_LOW		0x58
#define AST_XDMA_US_CMD0_HIGH		0x5C
#define AST_XDMA_US_CMD1_LOW		0x60
#define AST_XDMA_US_CMD1_HIGH		0x64
#define AST_XDMA_US_CMD2_LOW		0x68
#define AST_XDMA_US_CMD2_HIGH		0x6C

/* AST_XDMA_CTRL_IER - 0x38 : Interrupt Enable and Engine Control */
#define XDMA_PCIE_64Bits_MODE_EN	(0x1 << 31)
#define XDMA_EDGE_INT				(0x1 << 24)
#define XDMA_DS_REQ_SIZE_MASK		(0x7 << 20)
#define XDMA_DS_REQ_SIZE_256		(0x1 << 20)
#define XDMA_DS_REQ_512				(0x2 << 20)
#define XDMA_DS_REQ_1K				(0x3 << 20)
#define XDMA_DS_REQ_2K				(0x4 << 20)
#define XDMA_DS_REQ_4K				(0x5 << 20)

#define XDMA_BMC_DS_DIRTY			(0x1 << 18)
#define XDMA_BMC_DS_DONE			(0x1 << 17)
#define XDMA_BMC_US_DONE			(0x1 << 16)
#define XDMA_VGA_DS_DONE			(0x1 << 15)
#define XDMA_VGA_US_DONE			(0x1 << 14)
#define XDMA_HOST_DS_DONE			(0x1 << 13)
#define XDMA_HOST_US_DONE			(0x1 << 12)

/* AST_XDMA_CTRL_ISR - 0x3C : Interrupt Flag and Engine Status */
#define XDMA_ALL_IDEL				(0x1 << 31)
#define XDMA_HOST_CMDQ_IDLE			(0x1 << 30)
#define XDMA_BMC_CMDQ_IDLE			(0x1 << 29)
#define XDMA_DS_IDLE				(0x1 << 28)
#define XDMA_US_IDLE				(0x1 << 27)
#define XDMA_VGA_CMDQ_IDLE			(0x1 << 26)
#define XDMA_HOST_CMDQ_DIRTY		(0x1 << 25)
#define XDMA_VGA_CMDQ_DIRTY			(0x1 << 24)

#define XDMA_BMC_DS_DIRTY_STS		(0x1 << 23)
#define XDMA_VGA_DS_DIRTY_STS		(0x1 << 22)
#define XDMA_HOST_DS_DIRTY_STS		(0x1 << 21)
/*
#define XDMA_BMC_DS_DIRTY			(0x1 << 18)
#define XDMA_BMC_DS_DONE			(0x1 << 17)
#define XDMA_BMC_US_DONE			(0x1 << 16)
#define XDMA_VGA_DS_DONE			(0x1 << 15)
#define XDMA_VGA_US_DONE			(0x1 << 14)
#define XDMA_HOST_DS_DONE			(0x1 << 13)
#define XDMA_HOST_US_DONE			(0x1 << 12)
*/
/*************************************************************************************/
#define AST_XDMA_CMD_DESC_NUM		2

//CMD0 Format	(0x00) : Base address of PCIe data [63:3]
//CMD1 Format	(0x08) : Pitch and Base Address 

//CMD2 Format Low	(0x10) : Fire Command
#define FRAM_LINE_NUM(x)			((x & 0xfff) << 16)
#define FRAM_LINE_BYTE(x)			(x & 0x7fff)

//CMD2 Format HIGH	(0x14) : Fire Command
#define CMD_BINT_EN					(0x1 << 5)
#define CMD_HINT_EN					(0x1 << 4)
#define CMD_US_DIR					(0x1)
/*************************************************************************************/
#define UPDATE_WRITE_POINT			4
#define DEFAULT_END_POINT			8

struct ast_xdma_cmd_desc {
	u32 cmd0_low;
	u32 cmd0_high;
	u32 cmd1_low;
	u32 cmd1_high;
	u32 cmd2_low;
	u32 cmd2_high;
};
/*************************************************************************************/
#define MAX_XFER_BUFF_SIZE 4096

struct ast_xdma_xfer {
	unsigned char stream_dir;
	unsigned char xfer_buff[MAX_XFER_BUFF_SIZE];
	unsigned int xfer_len;
	unsigned int bmc_addr;
	unsigned int host_addr_low;
	unsigned int host_addr_high;
};

#define XDMAIOC_BASE       'D'

#define AST_XDMA_IOCXFER		_IOWR(XDMAIOC_BASE, 0x0, struct ast_xdma_xfer)
/*************************************************************************************/
//#define AST_XDMA_DEBUG

#ifdef AST_XDMA_DEBUG
#define XDMA_DBUG(fmt, args...) printk(KERN_DEBUG "%s() " fmt,__FUNCTION__, ## args)
#else
#define XDMA_DBUG(fmt, args...)
#endif

#define XDMA_MSG(fmt, args...) printk(fmt, ## args)
/*************************************************************************************/
struct ast_xdma_info {
	void __iomem	*reg_base;
	int irq;				//XDMA IRQ number
	struct reset_control *reset;
	u32 dram_base;
	wait_queue_head_t xdma_wq;
	struct completion complete;
	u8 desc_index;
	struct ast_xdma_cmd_desc *xfer_cmd_desc;
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
ast_xdma_read(struct ast_xdma_info *ast_xdma, u32 reg)
{
#if 0	
	u32 val;

	val = readl(ast_xdma->reg_base + reg);
	XDMA_DBUG("reg = 0x%08x, val = 0x%08x\n", reg, val);
	return val;
#else
	return readl(ast_xdma->reg_base + reg);
#endif	
}

static inline void
ast_xdma_write(struct ast_xdma_info *ast_xdma, u32 val, u32 reg)
{
	XDMA_DBUG("reg = 0x%08x, val = 0x%08x\n", reg, val);
	writel(val, ast_xdma->reg_base + reg);
}

//ast g5
#define UPDATE_WRITE_POINT				4
#define DEFAULT_END_POINT				8

/*************************************************************************************/
static void ast_xdma_xfer(struct ast_xdma_info *ast_xdma, struct ast_xdma_xfer *xdma_xfer)
{
	u32 xfer_len = 0;
	u32 bmc_addr = 0;

	XDMA_DBUG("\n");
	if (xdma_xfer->bmc_addr == 0)
		bmc_addr = ast_xdma->xfer_data_dma;
	else
		bmc_addr = xdma_xfer->bmc_addr;

	ast_xdma->desc_index %= 2;

	XDMA_DBUG("cmd index [%x] : bmc addr : %x , host addr : %x (L) %x (H), size : %d \n", ast_xdma->desc_index, bmc_addr, xdma_xfer->host_addr_low, xdma_xfer->host_addr_high, xdma_xfer->xfer_len);

	xfer_len = xdma_xfer->xfer_len;

	ast_xdma->xfer_cmd_desc[ast_xdma->desc_index].cmd0_high = xdma_xfer->host_addr_high;
	ast_xdma->xfer_cmd_desc[ast_xdma->desc_index].cmd0_low = xdma_xfer->host_addr_low;

	if (xdma_xfer->stream_dir) {
		ast_xdma->xfer_cmd_desc[ast_xdma->desc_index].cmd2_high = CMD_US_DIR | CMD_HINT_EN | CMD_US_DIR;
		XDMA_DBUG("US cmd desc %x \n", ast_xdma->xfer_cmd_desc[ast_xdma->desc_index].cmd1_low);
		if (xdma_xfer->bmc_addr == 0)
			memcpy(ast_xdma->xfer_data, xdma_xfer->xfer_buff,  xdma_xfer->xfer_len);
	} else {
		ast_xdma->xfer_cmd_desc[ast_xdma->desc_index].cmd2_high = CMD_US_DIR | CMD_HINT_EN;
		XDMA_DBUG("DS cmd desc %x \n", ast_xdma->xfer_cmd_desc[ast_xdma->desc_index].cmd1_low);
		memset(ast_xdma->xfer_data, 0,  4096);
	}

	ast_xdma->xfer_cmd_desc[ast_xdma->desc_index].cmd2_low = FRAM_LINE_NUM(1) | FRAM_LINE_BYTE(xfer_len);

	//trigger tx
	if (ast_xdma->desc_index == 0)
		ast_xdma_write(ast_xdma, UPDATE_WRITE_POINT, AST_XDMA_BMC_CMDQ_WRITEP);
	else
		ast_xdma_write(ast_xdma, 0, AST_XDMA_BMC_CMDQ_WRITEP);

	ast_xdma->desc_index++;

	wait_for_completion(&ast_xdma->complete);

	if (!xdma_xfer->stream_dir) {
		memcpy(xdma_xfer->xfer_buff, ast_xdma->xfer_data, xdma_xfer->xfer_len);
	}
	ast_xdma->flag = 0;
}

static irqreturn_t ast_xdma_isr(int this_irq, void *dev_id)
{
	struct ast_xdma_info *ast_xdma = dev_id;
	u32 status = ast_xdma_read(ast_xdma, AST_XDMA_CTRL_ISR);

	XDMA_DBUG("%x \n", status);

	ast_xdma->flag = 0;

	if (status & XDMA_BMC_DS_DIRTY_STS) {
		ast_xdma->flag = 1;
		XDMA_DBUG("XDMA_BMC_DS_DIRTY_STS \n");
	}

	if (status & XDMA_DS_IDLE) {
		XDMA_DBUG("XDMA_DS_COMPLETE \n");
	}

	if (status & XDMA_US_IDLE) {
		XDMA_DBUG("XDMA_US_COMPLETE \n");
	}

	ast_xdma_write(ast_xdma, status, AST_XDMA_CTRL_ISR);

	complete(&ast_xdma->complete);

	return IRQ_HANDLED;
}

static void ast_xdma_ctrl_init(struct ast_xdma_info *ast_xdma)
{
	//xfer buff
	ast_xdma->xfer_data = dma_alloc_coherent(NULL,
						4096, &ast_xdma->xfer_data_dma, GFP_KERNEL);

	XDMA_DBUG("xfer buff %x , dma %x \n", (u32)ast_xdma->xfer_data, (u32)ast_xdma->xfer_data_dma);

	//tx cmd
	ast_xdma->xfer_cmd_desc = dma_alloc_coherent(NULL,
						sizeof(struct ast_xdma_cmd_desc) * AST_XDMA_CMD_DESC_NUM,
						&ast_xdma->xfer_cmd_desc_dma, GFP_KERNEL);

	XDMA_DBUG("xfer cmd desc %x , cmd desc dma %x \n", (u32)ast_xdma->xfer_cmd_desc, (u32)ast_xdma->xfer_cmd_desc_dma);

	memset(ast_xdma->xfer_cmd_desc, 0,  sizeof(struct ast_xdma_cmd_desc) * 2);

	ast_xdma->xfer_cmd_desc[0].cmd1_high = 0x00080008;
	ast_xdma->xfer_cmd_desc[1].cmd1_high = 0x00080008;
	ast_xdma->desc_index = 0;

	ast_xdma_write(ast_xdma, ast_xdma->xfer_cmd_desc_dma, AST_XDMA_BMC_CMDQ_BASE);
	ast_xdma_write(ast_xdma, DEFAULT_END_POINT, AST_XDMA_BMC_CMDQ_ENDP);
	ast_xdma_write(ast_xdma, 0, AST_XDMA_BMC_CMDQ_WRITEP);

	ast_xdma_write(ast_xdma, XDMA_BMC_CMDQ_IDLE | XDMA_DS_IDLE | XDMA_US_IDLE | XDMA_BMC_DS_DIRTY_STS, AST_XDMA_CTRL_IER);
}

static long xdma_ioctl(struct file *file, unsigned int cmd,
					   unsigned long arg)
{
	int ret = 0;
	struct miscdevice *c = file->private_data;
	struct ast_xdma_info *ast_xdma = dev_get_drvdata(c->this_device);
	void  *argp = (void *)arg;
	struct ast_xdma_xfer xfer;

	switch (cmd) {
	case AST_XDMA_IOCXFER:
		XDMA_DBUG("AST_XDMA_IOCXFER \n");
		if (copy_from_user(&xfer, argp, sizeof(struct ast_xdma_xfer))) {
			printk("copy_from_user  fail\n");
			ret = -EFAULT;
		} else {
			ast_xdma_xfer(ast_xdma, &xfer);
		}

		if (!xfer.stream_dir) {
			if (copy_to_user(argp, &xfer, sizeof(struct ast_xdma_xfer)))
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
	struct ast_xdma_info *ast_xdma = dev_get_drvdata(c->this_device);

	XDMA_DBUG("\n");
	spin_lock(&xdma_state_lock);

	if (ast_xdma->is_open) {
		spin_unlock(&xdma_state_lock);
		return -EBUSY;
	}

	ast_xdma->is_open = true;

	spin_unlock(&xdma_state_lock);

	return 0;
}

static int xdma_release(struct inode *inode, struct file *file)
{
	struct miscdevice *c = file->private_data;
	struct ast_xdma_info *ast_xdma = dev_get_drvdata(c->this_device);

	XDMA_DBUG("\n");
	spin_lock(&xdma_state_lock);

	ast_xdma->is_open = false;

	spin_unlock(&xdma_state_lock);

	return 0;
}

static const struct file_operations ast_xdma_fops = {
	.owner			= THIS_MODULE,
	.unlocked_ioctl	= xdma_ioctl,
	.open			= xdma_open,
	.release		= xdma_release,
};

static struct miscdevice ast_xdma_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "ast-xdma",
	.fops = &ast_xdma_fops,
};

static int ast_xdma_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct ast_xdma_info *ast_xdma;
	int ret = 0;

	XDMA_DBUG("\n");

	if (!(ast_xdma = devm_kzalloc(&pdev->dev, sizeof(struct ast_xdma_info), GFP_KERNEL))) {
		return -ENOMEM;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (NULL == res) {
		dev_err(&pdev->dev, "cannot get IORESOURCE_MEM\n");
		ret = -ENOENT;
		goto out;
	}

	ast_xdma->reg_base = devm_ioremap_resource(&pdev->dev, res);
	if (!ast_xdma->reg_base) {
		ret = -EIO;
		goto out_region;
	}

	ast_xdma->irq = platform_get_irq(pdev, 0);
	if (ast_xdma->irq < 0) {
		dev_err(&pdev->dev, "no irq specified\n");
		ret = -ENOENT;
		goto out_region;
	}

	ast_xdma->reset = devm_reset_control_get_exclusive(&pdev->dev, "xdma");
	if (IS_ERR(ast_xdma->reset)) {
		dev_err(&pdev->dev, "can't get xmda reset\n");
		return PTR_ERR(ast_xdma->reset);
	}

	//scu init
	reset_control_deassert(ast_xdma->reset);

	ret = devm_request_irq(&pdev->dev, ast_xdma->irq, ast_xdma_isr, 0, dev_name(&pdev->dev), ast_xdma);
	if (ret) {
		printk("MCTP Unable to get IRQ");
		goto out_region;
	}

	ast_xdma->flag = 0;

	init_completion(&ast_xdma->complete);
	ret = misc_register(&ast_xdma_misc);
	if (ret) {
		printk(KERN_ERR "XDMA : failed to request interrupt\n");
		goto out_irq;
	}

	platform_set_drvdata(pdev, ast_xdma);
	dev_set_drvdata(ast_xdma_misc.this_device, ast_xdma);

	ast_xdma_ctrl_init(ast_xdma);

	printk(KERN_INFO "ast_xdma: driver successfully loaded.\n");

	return 0;


out_irq:
	free_irq(ast_xdma->irq, NULL);
out_region:
	release_mem_region(res->start, res->end - res->start + 1);
out:
	printk(KERN_WARNING "applesmc: driver init failed (ret=%d)!\n", ret);
	return ret;
}

static int ast_xdma_remove(struct platform_device *pdev)
{
	struct resource *res;
	struct ast_xdma_info *ast_xdma = platform_get_drvdata(pdev);

	XDMA_DBUG("\n");

	misc_deregister(&ast_xdma_misc);

	free_irq(ast_xdma->irq, ast_xdma);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	iounmap(ast_xdma->reg_base);

	platform_set_drvdata(pdev, NULL);

	release_mem_region(res->start, res->end - res->start + 1);

	return 0;
}

#ifdef CONFIG_PM
static int
ast_xdma_suspend(struct platform_device *pdev, pm_message_t state)
{
	return 0;
}

static int
ast_xdma_resume(struct platform_device *pdev)
{
	return 0;
}

#else
#define ast_xdma_suspend        NULL
#define ast_xdma_resume         NULL
#endif

static const struct of_device_id ast_xdma_match[] = {
	{ .compatible = "aspeed,ast-xdma" },
	{ },
};

static struct platform_driver ast_xdma_driver = {
	.probe		= ast_xdma_probe,
	.remove 	= ast_xdma_remove,
#ifdef CONFIG_PM
	.suspend	= ast_xdma_suspend,
	.resume 	= ast_xdma_resume,
#endif
	.driver = {
		.name		= "ast-xdma",
		.of_match_table = ast_xdma_match,
	},
};

module_platform_driver(ast_xdma_driver);

MODULE_AUTHOR("Ryan Chen <ryan_chen@aspeedtech.com>");
MODULE_DESCRIPTION("ASPEED X-DMA Driver");
MODULE_LICENSE("GPL");