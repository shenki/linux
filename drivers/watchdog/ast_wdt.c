/*
 * ast_wdt.c - WDT driver for the Aspeed SoC
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

#include <asm/io.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/watchdog.h>
#include <linux/notifier.h>
#include <linux/reboot.h>
#include <linux/init.h>
#include <linux/of.h>
#include <linux/of_device.h>
////////////////////////////////////////////////////////////
extern void ast_soc_wdt_reset(enum reboot_mode mode, const char *cmd)	
{
#if 0
	writel(0x10 , (void *)(wdt_reg_base+0x04));
	writel(0x4755, (void *)(wdt_reg_base+0x08));
	writel(0x3, (void *)(wdt_reg_base+0x0c));
#endif	
}

EXPORT_SYMBOL(ast_soc_wdt_reset);
////////////////////////////////////////////////////////////
#define WDT_CNT_STATUS		0x00
#define WDT_RELOAD			0x04
#define WDT_RESTART			0x08
#define WDT_CTRL			0x0C
#define WDT_TIMEOUT_STS	0x10
#define WDT_TIMEOUT_CLR	0x14
#define WDT_RESET_WIDTH	0x18
#define WDT_RESET_MASK		0x1C
/****************************************/
/*WDT_RESTART		0x08 : WDT Conuter Rester Register */
#define WDT_RESTART_MAGIC	0x4755
/* WDT_CTRL			0x0C : WDT Control Register */
#define WDT_CTRL_RESET_SOC			(0x00 << 5)
#define WDT_CTRL_RESET_FULL_CHIP	(0x01 << 5)
#define WDT_CTRL_RESET_ARM		(0x02 << 5)
/* The AST-G4 wdt can run at PCLK, or 1MHz. The AST-G5 only runs at 1MHz. */
#define WDT_CTRL_1MHZ_CLK			BIT(4)
#define WDT_CTRL_WDT_EXT			BIT(3)
#define WDT_CTRL_WDT_INTR			BIT(2)
#define WDT_CTRL_RESET_SYSTEM		BIT(1)
#define WDT_CTRL_ENABLE			BIT(0)
/*WDT_RESET_MASK  0x1C : WDT Reset Mask Register AST2500 only */

#define WDT_RESET_PWM_EN			BIT(17)
#define WDT_RESET_CRT_2D_EN			BIT(16)
#define WDT_RESET_MIC_EN			BIT(15)
#define WDT_RESET_SDHCI_EN			BIT(14)
#define WDT_RESET_LPC_EN			BIT(13)
#define WDT_RESET_HAC_EN			BIT(12)
#define WDT_RESET_VIDEO_EN			BIT(11)
#define WDT_RESET_EHCI_EN			BIT(10)
#define WDT_RESET_USB11_EN		BIT(9)
#define WDT_RESET_USB_EN			BIT(8)
#define WDT_RESET_CRT_EN			BIT(7)
#define WDT_RESET_MAC2_EN			BIT(6)
#define WDT_RESET_MAC1_EN			BIT(5)
#define WDT_RESET_I2C_EN			BIT(4)
#define WDT_RESET_AHB_EN			BIT(3)
#define WDT_RESET_SDRAM_EN			BIT(2)
#define WDT_RESET_COLDFIRE_EN			BIT(1)
#define WDT_RESET_ARM_EN			BIT(0)
/****************************************/
/* 32 bits at 1MHz, in milliseconds */
#define WDT_MAX_TIMEOUT_MS	4294967	/* 0xffffffff / 1,000,000*/
#define WDT_DEFAULT_TIMEOUT	30
#define WDT_RATE_1MHZ		1000000

struct ast_wdt {
	struct watchdog_device	wdd;
	void __iomem		*base;
};

static int timeout = WDT_DEFAULT_TIMEOUT;

module_param(timeout, int, 0);
MODULE_PARM_DESC(timeout, "Watchdog time in seconds. (default="
                                __MODULE_STRING(WDT_DEFAULT_TIME) ")");

static void ast_wdt_enable(struct ast_wdt *wdt, int count)
{
	writel(readl(wdt->base + WDT_CTRL) & ~WDT_CTRL_ENABLE, wdt->base + WDT_CTRL);
	writel(count, wdt->base + WDT_RELOAD);
	writel(WDT_RESTART_MAGIC, wdt->base + WDT_RESTART);
	writel(readl(wdt->base + WDT_CTRL) | WDT_CTRL_ENABLE, wdt->base + WDT_CTRL);
}

static int ast_wdt_start(struct watchdog_device *wdd)
{
	struct ast_wdt *wdt = watchdog_get_drvdata(wdd);
	ast_wdt_enable(wdt, wdd->timeout * WDT_RATE_1MHZ);
	return 0;
}

static int ast_wdt_stop(struct watchdog_device *wdd)
{
	struct ast_wdt *wdt = watchdog_get_drvdata(wdd);
	writel(readl(wdt->base + WDT_CTRL) & ~WDT_CTRL_ENABLE, wdt->base + WDT_CTRL);
	return 0;
}

static int ast_wdt_ping(struct watchdog_device *wdd)
{

	struct ast_wdt *wdt = watchdog_get_drvdata(wdd);
	printk("ast_wdt_ping \n");

	writel(WDT_RESTART_MAGIC, wdt->base + WDT_RESTART);

	return 0;
}

static int ast_wdt_set_timeout(struct watchdog_device *wdd,
				  unsigned int timeout)
{
	struct ast_wdt *wdt = watchdog_get_drvdata(wdd);
	u32 actual;
	printk("ast_wdt_set_timeout \n");

	wdd->timeout = timeout;

	actual = min(timeout, wdd->max_hw_heartbeat_ms * 1000);

	writel(actual * WDT_RATE_1MHZ, wdt->base + WDT_RELOAD);
	writel(WDT_RESTART_MAGIC, wdt->base + WDT_RESTART);

	return 0;
}

static int ast_wdt_restart(struct watchdog_device *wdd,
			      unsigned long action, void *data)
{
	struct ast_wdt *wdt = watchdog_get_drvdata(wdd);
	ast_wdt_enable(wdt, 128 * WDT_RATE_1MHZ / 1000);

	return 0;
}

static const struct watchdog_ops ast_wdt_ops = {
	.start		= ast_wdt_start,
	.stop		= ast_wdt_stop,
	.ping		= ast_wdt_ping,
	.set_timeout	= ast_wdt_set_timeout,
	.restart		= ast_wdt_restart,
	.owner		= THIS_MODULE,
};

static const struct watchdog_info ast_wdt_info = {
	.identity	= KBUILD_MODNAME,	
	.options	= WDIOF_KEEPALIVEPING | WDIOF_MAGICCLOSE | WDIOF_SETTIMEOUT,
};

static int ast_wdt_remove(struct platform_device *pdev)
{
	struct ast_wdt *wdt = platform_get_drvdata(pdev);

	iounmap(wdt->base);
	watchdog_unregister_device(&wdt->wdd);

	return 0;
}

static int ast_wdt_probe(struct platform_device *pdev)
{
	struct ast_wdt *wdt;
	struct resource *res;
	u32 reset_mask;
	int ret;

	wdt = devm_kzalloc(&pdev->dev, sizeof(*wdt), GFP_KERNEL);
	if (!wdt)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	wdt->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(wdt->base))
		return PTR_ERR(wdt->base);

	wdt->wdd.info = &ast_wdt_info;
	wdt->wdd.ops = &ast_wdt_ops;
	wdt->wdd.max_hw_heartbeat_ms = WDT_MAX_TIMEOUT_MS;
	wdt->wdd.parent = &pdev->dev;

	wdt->wdd.timeout = WDT_DEFAULT_TIMEOUT;
	watchdog_init_timeout(&wdt->wdd, 0, &pdev->dev);

	// initial disable 
	writel(WDT_CTRL_RESET_SOC | WDT_CTRL_1MHZ_CLK | WDT_CTRL_RESET_SYSTEM, 
		wdt->base + WDT_CTRL);

	//g4 reset mask is in scu, g5 supoort in register
	if(of_find_compatible_node(NULL, NULL, "aspeed,ast-g5-wdt")) {	
		if(of_property_read_u32(pdev->dev.of_node, "reset_mask", &reset_mask))
			writel(reset_mask, wdt->base + WDT_RESET_MASK);
//		printk("set reset mask %x \n", reset_mask);
	}
		
	platform_set_drvdata(pdev, wdt);
	watchdog_set_drvdata(&wdt->wdd, wdt);
	ret = devm_watchdog_register_device(&pdev->dev, &wdt->wdd);
	if (ret) {
		dev_err(&pdev->dev, "failed to register\n");
		return ret;
	}

	printk(KERN_INFO "ast_wdt[%s]: driver successfully loaded.\n", pdev->dev.of_node->name);

	return 0;

}

static const struct of_device_id ast_wdt_of_table[] = {
	{ .compatible = "aspeed,ast-wdt" },
	{ .compatible = "aspeed,ast-g5-wdt" },
	{ .compatible = "aspeed,ast-g4-wdt" },
	{ },
};
MODULE_DEVICE_TABLE(of, ast_wdt_of_table);


static struct platform_driver ast_watchdog_driver = {
	.probe = ast_wdt_probe,
	.remove = ast_wdt_remove,
	.driver = {
		.name = KBUILD_MODNAME,
		.of_match_table = of_match_ptr(ast_wdt_of_table),
	},
};
module_platform_driver(ast_watchdog_driver);

MODULE_AUTHOR("Ryan Chen <ryan_chen@aspeedtech.com>");
MODULE_DESCRIPTION("ASPEED Watchdog Driver");
MODULE_LICENSE("GPL");
