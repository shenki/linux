/*
 * ast-mailbox.c - Mailbox driver for the Aspeed SoC
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

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <asm/uaccess.h>

#include <linux/completion.h>
#include <linux/slab.h>

#include <linux/sched.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/miscdevice.h>
/***********************************************************************/
#define AST_MBX_DAT0				0x00
#define AST_MBX_DAT1				0x04
#define AST_MBX_DAT2				0x08
#define AST_MBX_DAT3				0x0C
#define AST_MBX_DAT4				0x10
#define AST_MBX_DAT5				0x14
#define AST_MBX_DAT6				0x18
#define AST_MBX_DAT7				0x1C
#define AST_MBX_DAT8				0x20
#define AST_MBX_DAT9				0x24
#define AST_MBX_DATA				0x28
#define AST_MBX_DATB				0x2C
#define AST_MBX_DATC				0x30
#define AST_MBX_DATD				0x34
#define AST_MBX_DATE				0x38
#define AST_MBX_DATF				0x3C
#define AST_MBX_STS0				0x40
#define AST_MBX_STS1				0x44
#define AST_MBX_BCR				0x48
#define AST_MBX_HCR				0x4C
#define AST_MBX_BIE0				0x50
#define AST_MBX_BIE1				0x54
#define AST_MBX_HIE0				0x58
#define AST_MBX_HIE1				0x5C

/* AST_MBX_BCR					0x48 	*/
#define	MBHIST			(1 << 7)
#define	MBHMK			(1 << 1)
#define	MBBINT			(1)
/***********************************************************************/
//#define CONFIG_AST_MBX_DEBUG

#ifdef CONFIG_AST_MBX_DEBUG
#define MBX_DBG(fmt, args...) printk("%s(): " fmt, __FUNCTION__, ## args)
#else
#define MBX_DBG(fmt, args...)
#endif

/***********************************************************************/
struct mailbox_info {
	u8	ch_enable;
	u8	ch_type;
};

//IOCTL ..
#define MBXIOC_BASE       'M'

#define AST_MBX_IOCRMAILBOX		_IOR(MBXIOC_BASE, 0, struct mailbox_info*)
#define AST_MBX_IOCWMAILBOX		_IOW(MBXIOC_BASE, 1, struct mailbox_info*)

struct ast_mailbox_data {
	struct device		*misc_dev;
	void __iomem		*reg_base;			/* virtual */
	int 				irq;
	bool is_open;
	spinlock_t lock;
	u32		sts0;
	u32		sts1;
	u32		bcr;
	struct completion		xfer_complete;
};

static inline void
ast_mbx_write(struct ast_mailbox_data *ast_mbx, u32 val, u32 reg)
{
//	MBX_DBG("write offset: %x, val: %x \n",reg,val);
	writel(val, ast_mbx->reg_base + reg);
}

static inline u32
ast_mbx_read(struct ast_mailbox_data *ast_mbx, u32 reg)
{
	u32 val = readl(ast_mbx->reg_base + reg);
//	MBX_DBG("read offset: %x, val: %x \n",reg,val);

	return val;
}

#if 0
static void ast_mailbox_host_mask_int(struct ast_mailbox_data *ast_mbx, u8 enable)
{
	if (enable)
		ast_mbx_write(ast_mbx, ast_mbx_read(ast_mbx, AST_MBX_BCR) | MBHMK, AST_MBX_BCR);
	else
		ast_mbx_write(ast_mbx, ast_mbx_read(ast_mbx, AST_MBX_BCR) & ~MBHMK, AST_MBX_BCR);
}
#endif

static void ast_mailbox_ctrl_init(struct ast_mailbox_data *ast_mbx)
{
	//per byte moniter ....
//	ast_mbx_write(ast_mbx, 0xff, AST_MBX_BIE0);
//	ast_mbx_write(ast_mbx, 0xff, AST_MBX_BIE1);
}

/***************************************************************************/

static irqreturn_t ast_mailbox_handler(int this_irq, void *dev_id)
{
	struct ast_mailbox_data *ast_mbx = dev_id;

//	ast_mbx->sts0 = ast_mbx_read(ast_mbx, AST_MBX_STS0);
//	ast_mbx->sts1 = ast_mbx_read(ast_mbx, AST_MBX_STS1);
	ast_mbx->bcr = ast_mbx_read(ast_mbx, AST_MBX_BCR);

	if ((ast_mbx->bcr & MBHIST) == 0)
		return IRQ_NONE;


	MBX_DBG("ast_mailbox_handler sts0 = %x, sts1 = %x, bcr = %x \n", ast_mbx->sts0, ast_mbx->sts1, ast_mbx->bcr);
//	ast_mbx_write(ast_mbx, ast_mbx->sts0, AST_MBX_STS0);
//	ast_mbx_write(ast_mbx, ast_mbx->sts1, AST_MBX_STS1);
	ast_mbx_write(ast_mbx, ast_mbx->bcr, AST_MBX_BCR);

	printk("Data: %x %x %x %x \n",
		   ast_mbx_read(ast_mbx, AST_MBX_DAT0),
		   ast_mbx_read(ast_mbx, AST_MBX_DAT4),
		   ast_mbx_read(ast_mbx, AST_MBX_DAT8),
		   ast_mbx_read(ast_mbx, AST_MBX_DATC));
//	complete(&ast_mbx->xfer_complete);

	return IRQ_HANDLED;

}

static long ast_mailbox_ioctl(struct file *fp,
							  unsigned int cmd, unsigned long arg)
{
	long ret = 0;
//	void __user *argp = (void __user *)arg;


	MBX_DBG("ast_mailbox_ioctl cmd %x \n", cmd);

	switch (cmd) {
	case AST_MBX_IOCRMAILBOX:
		break;

	case AST_MBX_IOCWMAILBOX:
		break;

	default:
		printk("ast_mailbox_ioctl command fail\n");
		ret = -ENOTTY;
		break;
	}

	return ret;
}

static int ast_mailbox_open(struct inode *inode, struct file *file)
{
	struct miscdevice *c = file->private_data;
	struct ast_mailbox_data *ast_mbx = dev_get_drvdata(c->this_device);

	MBX_DBG("\n");
	spin_lock(&ast_mbx->lock);

	if (ast_mbx->is_open) {
		spin_unlock(&ast_mbx->lock);
		return -1;
	}
	ast_mbx->is_open = true;

	spin_unlock(&ast_mbx->lock);

	return 0;
}

static int ast_mailbox_release(struct inode *inode, struct file *file)
{
	struct miscdevice *c = file->private_data;
	struct ast_mailbox_data *ast_mbx = dev_get_drvdata(c->this_device);

	MBX_DBG("\n");

	spin_lock(&ast_mbx->lock);
	ast_mbx->is_open = false;
	spin_unlock(&ast_mbx->lock);

	return 0;
}

static const struct file_operations ast_mailbox_fops = {
	.owner  =     	THIS_MODULE,
	.unlocked_ioctl =	ast_mailbox_ioctl,
	.open =         	ast_mailbox_open,
	.release =      	ast_mailbox_release,
};

struct miscdevice ast_mailbox_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "ast-mailbox",
	.fops = &ast_mailbox_fops,
};

static int ast_mailbox_probe(struct platform_device *pdev)
{
	static struct ast_mailbox_data *ast_mbx;
	struct resource *res;
	int ret = 0;

	MBX_DBG(" \n");

	ast_mbx = devm_kzalloc(&pdev->dev, sizeof(*ast_mbx), GFP_KERNEL);
	if (!ast_mbx)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (NULL == res) {
		dev_err(&pdev->dev, "cannot get IORESOURCE_MEM\n");
		ret = -ENOENT;
		goto out;
	}

	ast_mbx->reg_base = devm_ioremap_resource(&pdev->dev, res);
	if (!ast_mbx->reg_base) {
		ret = -EIO;
		goto out_region;
	}

	ast_mbx->irq = platform_get_irq(pdev, 0);
	if (ast_mbx->irq < 0) {
		dev_err(&pdev->dev, "no irq specified\n");
		ret = -ENOENT;
		goto out_region;
	}

	ret = devm_request_irq(&pdev->dev, ast_mbx->irq, ast_mailbox_handler,
						   0, dev_name(&pdev->dev), NULL);
	if (ret) {
		printk(KERN_INFO "MBX: Failed request irq %d\n", ast_mbx->irq);
		goto out_region;
	}

	ret = misc_register(&ast_mailbox_misc);
	if (ret) {
		printk(KERN_ERR "MBX : failed to request interrupt\n");
		goto out_irq;
	}

	spin_lock_init(&ast_mbx->lock);
	platform_set_drvdata(pdev, ast_mbx);
	dev_set_drvdata(ast_mailbox_misc.this_device, ast_mbx);

	ast_mailbox_ctrl_init(ast_mbx);

	printk(KERN_INFO "ast_mbx: driver successfully loaded.\n");

	return 0;

out_irq:
	free_irq(ast_mbx->irq, NULL);
out_region:
	release_mem_region(res->start, res->end - res->start + 1);
out:
	if (ast_mbx) {
		kfree(ast_mbx);
	}
	printk(KERN_WARNING "applesmc: driver init failed (ret=%d)!\n", ret);
	return ret;
}

static int ast_mailbox_remove(struct platform_device *pdev)
{
	struct resource *res;
	struct ast_mailbox_data *ast_mbx = platform_get_drvdata(pdev);

	MBX_DBG("ast_mailbox_remove\n");

	misc_deregister(&ast_mailbox_misc);

	free_irq(ast_mbx->irq, ast_mbx);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	iounmap(ast_mbx->reg_base);

	release_mem_region(res->start, res->end - res->start + 1);

	if (ast_mbx) {
		kfree(ast_mbx);
	}

	return 0;
}

#ifdef CONFIG_PM
static int
ast_mailbox_suspend(struct platform_device *pdev, pm_message_t state)
{
	printk("ast_mailbox_suspend : TODO \n");
	return 0;
}

static int
ast_mailbox_resume(struct platform_device *pdev)
{
	return 0;
}

#else
#define ast_mailbox_suspend        NULL
#define ast_mailbox_resume         NULL
#endif

static const struct of_device_id ast_mailbox_of_matches[] = {
	{ .compatible = "aspeed,ast-mailbox", },
	{},
};
MODULE_DEVICE_TABLE(of, ast_mailbox_of_matches);

static struct platform_driver ast_mailbox_driver = {
	.probe		= ast_mailbox_probe,
	.remove 		= ast_mailbox_remove,
#ifdef CONFIG_PM
	.suspend		= ast_mailbox_suspend,
	.resume 		= ast_mailbox_resume,
#endif
	.driver 		= {
		.name	= KBUILD_MODNAME,
		.of_match_table = ast_mailbox_of_matches,
	},
};

module_platform_driver(ast_mailbox_driver);


MODULE_AUTHOR("Ryan Chen <ryan_chen@aspeedtech.com>");
MODULE_DESCRIPTION("AST Mailbox driver");
MODULE_LICENSE("GPL");
