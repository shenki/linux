/*
 * ast-peci.c - PECI driver for the Aspeed SoC
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
#include <linux/reset.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/miscdevice.h>
/***********************************************************************/
/*AST PECI Register Definition */
#define AST_PECI_CTRL		0x00
#define AST_PECI_TIMING 		0x04
#define AST_PECI_CMD		0x08
#define AST_PECI_CMD_CTRL	0x0C
#define AST_PECI_EXP_FCS	0x10
#define AST_PECI_CAP_FCS	0x14
#define AST_PECI_INT_CTRL	0x18
#define AST_PECI_INT_STS	0x1C
#define AST_PECI_W_DATA0	0x20
#define AST_PECI_W_DATA1	0x24
#define AST_PECI_W_DATA2	0x28
#define AST_PECI_W_DATA3	0x2c
#define AST_PECI_R_DATA0	0x30
#define AST_PECI_R_DATA1	0x34
#define AST_PECI_R_DATA2	0x38
#define AST_PECI_R_DATA3	0x3c
#define AST_PECI_W_DATA4	0x40
#define AST_PECI_W_DATA5	0x44
#define AST_PECI_W_DATA6	0x48
#define AST_PECI_W_DATA7	0x4c
#define AST_PECI_R_DATA4	0x50
#define AST_PECI_R_DATA5	0x54
#define AST_PECI_R_DATA6	0x58
#define AST_PECI_R_DATA7	0x5c


/* AST_PECI_CTRL - 0x00 : Control Register */
#define PECI_CTRL_SAMPLING_MASK			(0xf << 16)
#define PECI_CTRL_SAMPLING(x)			(x << 16)
#define PECI_CTRL_READ_MODE_MASK		(0xf << 12)
#define PECI_CTRL_CONT_MODE				(1 << 16)
#define PECI_CTRL_DBG_MODE				(2 << 16)
#define PECI_CTRL_CLK_SOURCE			(0x1 << 11)	//0: 24Mhz, 1: MCLK
#define PECI_CTRL_CLK_DIV_MASK			(0x3 << 8)
#define PECI_CTRL_CLK_DIV(x)			(x << 8)
#define PECI_CTRL_INVERT_OUT			(0x1 << 7)
#define PECI_CTRL_INVERT_IN				(0x1 << 6)
#define PECI_CTRL_BUS_CONTENT_EN		(0x1 << 5)
#define PECI_CTRL_PECI_EN				(0x1 << 4)
#define PECI_CTRL_PECI_CLK_EN			(0x1)

/* AST_PECI_TIMING - 0x04 : Timing Negotiation */
#define PECI_TIMING_MESSAGE_GET(x)		((x & 0xff00) >> 8)
#define PECI_TIMING_MESSAGE(x)			(x << 8)
#define PECI_TIMING_ADDRESS_GET(x)		(x & 0xff)
#define PECI_TIMING_ADDRESS(x)			(x)

/* AST_PECI_CMD	- 0x08 : Command Register */
#define PECI_CMD_PIN_MON				(0x1 << 31)
#define PECI_CMD_STS					(0xf << 24)
#define PECI_CMD_FIRE					(0x1)

/* AST_PECI_LEN	- 0x0C : Read/Write Length Register */
#define PECI_AW_FCS_EN					(0x1 << 31)
#define PECI_READ_LEN_MASK				(0xff << 16)
#define PECI_READ_LEN(x)				(x << 16)
#define PECI_WRITE_LEN_MASK				(0xff << 8)
#define PECI_WRITE_LEN(x)				(x << 8)
#define PECI_TAGET_ADDR_MASK			(0xff)
#define PECI_TAGET_ADDR(x)				(x)


/* AST_PECI_EXP_FCS	- 0x10 : Expected FCS Data Register  */
#define PECI_PROGRAM_AW_FCS				(0xf << 24)
#define PECI_EXPECT_READ_FCS			(0xf << 16)
#define PECI_EXPECT_AW_FCS_AUTO			(0xf << 8)
#define PECI_EXPECT_WRITE_FCS			(0xf)

/* AST_PECI_CAP_FCS	- 0x14 : Captured FCS Data Register */
#define PECI_CAPTURE_READ_FCS(x)		((x & 0xff) >> 16)
#define PECI_CAPTURE_WRITE_FCS			(0xff)

/* AST_PECI_INT_CTRL/ STS  - 0x18/0x1c  : Interrupt Register */
#define PECI_INT_TIMING_RESULT_MASK		(0x3 << 30)
#define PECI_INT_TIMEOUT				(0x1 << 4)
#define PECI_INT_CONNECT				(0x1 << 3)
#define PECI_INT_W_FCS_BAD				(0x1 << 2)
#define PECI_INT_W_FCS_ABORT			(0x1 << 1)
#define PECI_INT_CMD_DONE				(0x1)

#define    AUTO_GEN_AWFCS                         1
//#define    ENABLE_BUS_CONTENTION                  0x20

#define    DISABLE_ENGINE                         0
#define    ENABLE_RX_ENGINE                       (1 << 0)
#define    ENABLE_TX_ENGINE                       (1 << 1)
#define    LEFT_CHANNEL_HIGH                      (1 << 16)
#define    DELAY_CLOCK_CYCLE                      (1 << 17)
/***********************************************************************/
//#define CONFIG_AST_PECI_DEBUG

#ifdef CONFIG_AST_PECI_DEBUG
#define PECI_DBG(fmt, args...) printk(KERN_DEBUG "%s() " fmt,__FUNCTION__, ## args)
#else
#define PECI_DBG(fmt, args...)
#endif

/***********************************************************************/
struct timing_negotiation {
	u8		msg_timing;
	u8		addr_timing;
};

struct xfer_msg {
	u8		client_addr;
	u8		tx_len;
	u8		rx_len;
	u8		tx_fcs;
	u8		rx_fcs;
	u8		fcs_en;
	u8		sw_fcs;
	u8		*tx_buf;
	u8		*rx_buf;
	u32		sts;
};

//IOCTL ..
#define PECIIOC_BASE       'P'

#define AST_PECI_IOCRTIMING		_IOR(PECIIOC_BASE, 0, struct timing_negotiation*)
#define AST_PECI_IOCWTIMING		_IOW(PECIIOC_BASE, 1, struct timing_negotiation*)
#define AST_PECI_IOCXFER		_IOWR(PECIIOC_BASE, 2, struct xfer_msg*)
/***********************************************************************/

static struct ast_peci_data {
	struct device			*misc_dev;
	void __iomem			*reg_base;			/* virtual */
	int 					irq;				//PECI IRQ number
	struct reset_control *reset;
	int 				open_count;
	struct completion		xfer_complete;
	u32					sts;
	struct mutex lock;
} ast_peci;

static inline void
ast_peci_write(u32 val, u32 reg)
{
	PECI_DBG("write offset: %x, val: %x \n", reg, val);
	writel(val, ast_peci.reg_base + reg);
}

static inline u32
ast_peci_read(u32 reg)
{
	u32 val = readl(ast_peci.reg_base + reg);
	PECI_DBG("read offset: %x, val: %x \n", reg, val);
	return val;
}

static long ast_peci_ioctl(struct file *fp,
						   unsigned int cmd, unsigned long arg)
{
	long ret = 0;
	void __user *argp = (void __user *)arg;
	struct xfer_msg msg;
	struct timing_negotiation tim_ng;
	u32	peci_head;
	int i = 0;
	int timeout = 5;
	u32 *tx_buf0 = (u32 *)(ast_peci.reg_base + AST_PECI_W_DATA0);
	u32 *tx_buf1 = (u32 *)(ast_peci.reg_base + AST_PECI_W_DATA4);
	u32 *rx_buf0 = (u32 *)(ast_peci.reg_base + AST_PECI_R_DATA0);
	u32 *rx_buf1 = (u32 *)(ast_peci.reg_base + AST_PECI_R_DATA4);
	u32 rx_data = 0;

	PECI_DBG("ast_peci_ioctl cmd %x \n", cmd);

	switch (cmd) {
	case AST_PECI_IOCRTIMING:
		tim_ng.msg_timing = PECI_TIMING_MESSAGE_GET(ast_peci_read(AST_PECI_TIMING));
		tim_ng.addr_timing = PECI_TIMING_ADDRESS_GET(ast_peci_read(AST_PECI_TIMING));
		if (copy_to_user(argp, &tim_ng, sizeof(struct timing_negotiation)))
			ret = -EFAULT;
		break;

	case AST_PECI_IOCWTIMING:
		if (copy_from_user(&tim_ng, argp, sizeof(struct timing_negotiation))) {
			ret = -EFAULT;
		} else {
			ast_peci_write(PECI_TIMING_MESSAGE(tim_ng.msg_timing) |
						   PECI_TIMING_ADDRESS(tim_ng.addr_timing), AST_PECI_TIMING);
		}
		break;

	case AST_PECI_IOCXFER:
		//Check ctrl idle sts & bus state
		while (ast_peci_read(AST_PECI_CMD) & (PECI_CMD_STS | PECI_CMD_PIN_MON)) {
			if (timeout-- < 0) {
				PECI_DBG("PECI: Timeout to wait idle!\n");
				goto out;
			}
			msleep(10);
		};

		if (copy_from_user(&msg, argp, sizeof(struct xfer_msg))) {
			ret = -EFAULT;
			goto out;
		}

#ifdef CONFIG_AST_PECI_DEBUG
		printk("fcs_en %d, client_addr %x, tx_len %d, rx_len %d", msg.fcs_en , msg.client_addr, msg.tx_len, msg.rx_len);
		printk("\ntx_buf : ");
		for (i = 0; i < msg.tx_len; i++)
			printk(" %x ", msg.tx_buf[i]);
		printk("\n");
#endif

		if (msg.fcs_en)
			peci_head = PECI_TAGET_ADDR(msg.client_addr) |
						PECI_WRITE_LEN(msg.tx_len) |
						PECI_READ_LEN(msg.rx_len) | PECI_AW_FCS_EN;
		else
			peci_head = PECI_TAGET_ADDR(msg.client_addr) |
						PECI_WRITE_LEN(msg.tx_len) |
						PECI_READ_LEN(msg.rx_len);


		ast_peci_write(peci_head, AST_PECI_CMD_CTRL);

		for (i = 0; i < msg.tx_len; i++) {
			if (i < 16) {
				if (i % 4 == 0)
					tx_buf0[i / 4] = 0;
				tx_buf0[i / 4] |= (msg.tx_buf[i] << ((i % 4) * 8)) ;
			} else {
				if (i % 4 == 0)
					tx_buf1[i / 4] = 0;
				tx_buf1[i / 4] |= (msg.tx_buf[i] << ((i % 4) * 8)) ;
			}
		}

#ifdef CONFIG_AST_PECI_DEBUG
		printk("\nWD \n ");
		ast_peci_read(AST_PECI_W_DATA0);
		ast_peci_read(AST_PECI_W_DATA1);
		ast_peci_read(AST_PECI_W_DATA2);
		ast_peci_read(AST_PECI_W_DATA3);
		ast_peci_read(AST_PECI_W_DATA4);
		ast_peci_read(AST_PECI_W_DATA5);
		ast_peci_read(AST_PECI_W_DATA6);
		ast_peci_read(AST_PECI_W_DATA7);
#endif
		init_completion(&ast_peci.xfer_complete);
		//Fire Command
		ast_peci_write(PECI_CMD_FIRE, AST_PECI_CMD);


		ret = wait_for_completion_interruptible_timeout(&ast_peci.xfer_complete, 30 * HZ);

		if (ret == 0)
			printk("peci controller timed out\n");

		for (i = 0; i < msg.rx_len; i++) {
			if (i < 16) {
				switch (i % 4) {
				case 0:
					rx_data = rx_buf0[i / 4];

					msg.rx_buf[i] = rx_data & 0xff;
					break;
				case 1:
					msg.rx_buf[i] = (rx_data & 0xff00) >> 8;
					break;
				case 2:
					msg.rx_buf[i] = (rx_data & 0xff0000) >> 16;
					break;
				case 3:
					msg.rx_buf[i] = (rx_data & 0xff000000) >> 24;
					break;

				}
			} else {
				switch (i % 4) {
				case 0:
					rx_data = rx_buf1[i / 4];
					msg.rx_buf[i] = rx_data & 0xff;
					break;
				case 1:
					msg.rx_buf[i] = (rx_data & 0xff00) >> 8;
					break;
				case 2:
					msg.rx_buf[i] = (rx_data & 0xff0000) >> 16;
					break;
				case 3:
					msg.rx_buf[i] = (rx_data & 0xff000000) >> 24;
					break;

				}
			}
		}
#ifdef CONFIG_AST_PECI_DEBUG
		printk("\nRD \n");
		ast_peci_read(AST_PECI_R_DATA0);
		ast_peci_read(AST_PECI_R_DATA1);
		ast_peci_read(AST_PECI_R_DATA2);
		ast_peci_read(AST_PECI_R_DATA3);
		ast_peci_read(AST_PECI_R_DATA4);
		ast_peci_read(AST_PECI_R_DATA5);
		ast_peci_read(AST_PECI_R_DATA6);
		ast_peci_read(AST_PECI_R_DATA7);

		printk("rx_buf : ");
		for (i = 0; i < msg.rx_len; i++)
			printk("%x ", msg.rx_buf[i]);
		printk("\n");
#endif
		msg.sts = ast_peci.sts;
		msg.rx_fcs = PECI_CAPTURE_READ_FCS(ast_peci_read(AST_PECI_CAP_FCS));
		if (copy_to_user(argp, &msg, sizeof(struct xfer_msg)))
			ret = -EFAULT;

		break;
	default:
		printk("ast_peci_ioctl command fail\n");
		ret = -ENOTTY;
		break;
	}

out:
	return ret;
}

static int ast_peci_open(struct inode *inode, struct file *file)
{
	PECI_DBG("ast_peci_open\n");

	/* Flush input queue on first open */
	if (ast_peci.open_count)
		return -1;

	ast_peci.open_count++;


	return 0;
}

static int ast_peci_release(struct inode *inode, struct file *file)
{
	PECI_DBG("ast_peci_release\n");
	ast_peci.open_count--;

	return 0;
}

static irqreturn_t ast_peci_handler(int this_irq, void *dev_id)
{
	ast_peci.sts = (0x1f & ast_peci_read(AST_PECI_INT_STS));

	switch (ast_peci.sts) {
	case PECI_INT_TIMEOUT:
		printk("PECI_INT_TIMEOUT \n");
		ast_peci_write(PECI_INT_TIMEOUT, AST_PECI_INT_STS);
		break;
	case PECI_INT_CONNECT:
		printk("PECI_INT_CONNECT \n");
		ast_peci_write(PECI_INT_CONNECT, AST_PECI_INT_STS);
		break;
	case PECI_INT_W_FCS_BAD:
		printk("PECI_INT_W_FCS_BAD \n");
		ast_peci_write(PECI_INT_W_FCS_BAD, AST_PECI_INT_STS);
		break;
	case PECI_INT_W_FCS_ABORT:
		printk("PECI_INT_W_FCS_ABORT \n");
		ast_peci_write(PECI_INT_W_FCS_ABORT, AST_PECI_INT_STS);
		break;
	case PECI_INT_CMD_DONE:
		printk("PECI_INT_CMD_DONE \n");
		ast_peci_write(PECI_INT_CMD_DONE, AST_PECI_INT_STS);
		ast_peci_write(0, AST_PECI_CMD);
		break;
	default:
		printk("no one handle .... \n");
		break;

	}

	complete(&ast_peci.xfer_complete);

	return IRQ_HANDLED;

}

static void ast_peci_ctrl_init(void)
{
	ast_peci_write(PECI_CTRL_CLK_DIV(3) |
				   PECI_CTRL_PECI_CLK_EN, AST_PECI_CTRL);
	udelay(1000);

	//PECI Timing Setting : should 4 times of peci clk period 64 = 16 * 4 ??
	ast_peci_write(PECI_TIMING_MESSAGE(64) | PECI_TIMING_ADDRESS(64), AST_PECI_TIMING);

	//PECI Programmable AWFCS
	//ast_peci_write(ast_peci, PECI_PROGRAM_AW_FCS, AST_PECI_EXP_FCS);

	//Issue Fix for interrupt accur
	//Clear Interrupt
	ast_peci_write(PECI_INT_TIMEOUT | PECI_INT_CONNECT |
				   PECI_INT_W_FCS_BAD | PECI_INT_W_FCS_ABORT |
				   PECI_INT_CMD_DONE, AST_PECI_INT_STS);


	//PECI Negotiation Selection , interrupt enable
	//Set nego mode :  1st bit of addr negotiation
	ast_peci_write(PECI_INT_TIMEOUT | PECI_INT_CONNECT |
				   PECI_INT_W_FCS_BAD | PECI_INT_W_FCS_ABORT |
				   PECI_INT_CMD_DONE, AST_PECI_INT_CTRL);

	//PECI Spec wide speed rangs [2kbps~2Mbps]
	//Sampling 8/16, READ mode : Point Sampling , CLK source : 24Mhz , DIV by 8 : 3 --> CLK is 3Mhz
	//PECI CTRL Enable
	ast_peci_write(PECI_CTRL_SAMPLING(8) | PECI_CTRL_CLK_DIV(3) |
				   PECI_CTRL_PECI_EN |
				   PECI_CTRL_PECI_CLK_EN, AST_PECI_CTRL);

}

static const struct file_operations ast_peci_fops = {
	.owner =        	THIS_MODULE,
	.llseek =       	no_llseek,
	.unlocked_ioctl =	ast_peci_ioctl,
	.open =         	ast_peci_open,
	.release =      	ast_peci_release,
};

struct miscdevice ast_peci_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "ast-peci",
	.fops = &ast_peci_fops,
};

static int ast_peci_probe(struct platform_device *pdev)
{
	struct resource *res;
	int ret = 0;

	PECI_DBG("ast_peci_probe\n");

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (NULL == res) {
		dev_err(&pdev->dev, "cannot get IORESOURCE_MEM\n");
		ret = -ENOENT;
		goto out;
	}

	ast_peci.reg_base = devm_ioremap_resource(&pdev->dev, res);
	if (!ast_peci.reg_base) {
		ret = -EIO;
		goto out_region;
	}

	ast_peci.irq = platform_get_irq(pdev, 0);
	if (ast_peci.irq < 0) {
		dev_err(&pdev->dev, "no irq specified\n");
		ret = -ENOENT;
		goto out_region;
	}

	ret = devm_request_irq(&pdev->dev, ast_peci.irq, ast_peci_handler,
						   0, dev_name(&pdev->dev), NULL);
	if (ret) {
		printk(KERN_INFO "PECI: Failed request irq %d\n", ast_peci.irq);
		goto out_region;
	}

	ast_peci.reset = devm_reset_control_get(&pdev->dev, NULL);
	if (IS_ERR(ast_peci.reset)) {
		dev_err(&pdev->dev, "can't get peci reset\n");
		return PTR_ERR(ast_peci.reset);
	}

	//scu init
	reset_control_assert(ast_peci.reset);
	udelay(3);
	reset_control_deassert(ast_peci.reset);

	ret = misc_register(&ast_peci_misc);
	if (ret) {
		printk(KERN_ERR "PECI : failed to request interrupt\n");
		goto out_irq;
	}

	ast_peci_ctrl_init();

	printk(KERN_INFO "ast_peci: driver successfully loaded.\n");

	return 0;

out_irq:
	free_irq(ast_peci.irq, NULL);
out_region:
	release_mem_region(res->start, res->end - res->start + 1);
out:
	printk(KERN_WARNING "applesmc: driver init failed (ret=%d)!\n", ret);
	return ret;
}

static int ast_peci_remove(struct platform_device *pdev)
{
	struct resource *res;

	PECI_DBG("ast_peci_remove\n");

	misc_deregister(&ast_peci_misc);

	free_irq(ast_peci.irq, &ast_peci);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	iounmap(ast_peci.reg_base);

	release_mem_region(res->start, res->end - res->start + 1);

	return 0;
}

#ifdef CONFIG_PM
static int
ast_peci_suspend(struct platform_device *pdev, pm_message_t state)
{
	printk("ast_peci_suspend : TODO \n");
	return 0;
}

static int
ast_peci_resume(struct platform_device *pdev)
{
	ast_peci_ctrl_init();
	return 0;
}
#else
#define ast_peci_suspend        NULL
#define ast_peci_resume         NULL
#endif

static const struct of_device_id of_ast_peci_match_table[] = {
	{ .compatible = "aspeed,ast-peci", },
	{},
};
MODULE_DEVICE_TABLE(of, of_ast_peci_match_table);

static struct platform_driver ast_peci_driver = {
	.probe		= ast_peci_probe,
	.remove 		= ast_peci_remove,
#ifdef CONFIG_PM
	.suspend        = ast_peci_suspend,
	.resume         = ast_peci_resume,
#endif
	.driver		= {
		.name	= KBUILD_MODNAME,
		.of_match_table = of_ast_peci_match_table,
	},
};

module_platform_driver(ast_peci_driver);

MODULE_AUTHOR("Ryan Chen <ryan_chen@aspeedtech.com>");
MODULE_DESCRIPTION("ASPEED PECI driver");
MODULE_LICENSE("GPL");
