/*
 * SDHCI support for ASPEED AST SoCs
 *
 * Copyright (C) ASPEED Technology Inc.
 *
 * Licensed under GPLv2 or later.
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/mmc/host.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include "sdhci-pltfm.h"
#include <plat/ast-scu.h>

extern void ast_sd_set_8bit_mode(u8 mode);

static void sdhci_ast_set_clock(struct sdhci_host *host, unsigned int clock)
{
	int div;
	u16 clk;
	unsigned long timeout;

	if (clock == host->clock)
		return;

	sdhci_writew(host, 0, SDHCI_CLOCK_CONTROL);

	if (clock == 0)
		goto out;

	for (div = 1;div < 256;div *= 2) {
		if ((host->max_clk / div) <= clock)
			break;
	}
	div >>= 1;

	//Issue : For ast2300, ast2400 couldn't set div = 0 means /1 , so set source is ~50Mhz up 
	
	clk = div << SDHCI_DIVIDER_SHIFT;
	clk |= SDHCI_CLOCK_INT_EN;
	sdhci_writew(host, clk, SDHCI_CLOCK_CONTROL);

	/* Wait max 20 ms */
	timeout = 20;
	while (!((clk = sdhci_readw(host, SDHCI_CLOCK_CONTROL))
		& SDHCI_CLOCK_INT_STABLE)) {
		if (timeout == 0) {
			printk(KERN_ERR "%s: Internal clock never "
				"stabilised.\n", mmc_hostname(host->mmc));
//			sdhci_dumpregs(host);
			return;
		}
		timeout--;
		mdelay(1);
	}

	clk |= SDHCI_CLOCK_CARD_EN;
	sdhci_writew(host, clk, SDHCI_CLOCK_CONTROL);

out:
	host->clock = clock;
}

static void sdhci_ast_set_bus_width(struct sdhci_host *host, int width)
{
	u8 ctrl = sdhci_readb(host, SDHCI_HOST_CONTROL);

	if (width == MMC_BUS_WIDTH_8) {
		ast_sd_set_8bit_mode(1);
	} else {
		ast_sd_set_8bit_mode(0);
	}

	if (width == MMC_BUS_WIDTH_4)
		ctrl |= SDHCI_CTRL_4BITBUS;
	else
		ctrl &= ~SDHCI_CTRL_4BITBUS;
		
	sdhci_writeb(host, ctrl, SDHCI_HOST_CONTROL);
}

static unsigned int sdhci_ast_get_max_clk(struct sdhci_host *host)
{
	return ast_get_sd_clock_src();
}

static unsigned int sdhci_ast_get_timeout_clk(struct sdhci_host *host)
{
	return ast_get_sd_clock_src()/1000000;
}

/*
	AST2300/AST2400 : SDMA/PIO
	AST2500 : ADMA/SDMA/PIO
*/
static struct sdhci_ops sdhci_ast_ops = {
	.set_clock = sdhci_ast_set_clock,
	.get_max_clock	= sdhci_ast_get_max_clk,
	.set_bus_width = sdhci_ast_set_bus_width,
	.get_timeout_clock = sdhci_ast_get_timeout_clk,
	.reset = sdhci_reset,
	.set_uhs_signaling = sdhci_set_uhs_signaling,
};

static struct sdhci_pltfm_data sdhci_ast_pdata = {
	.ops = &sdhci_ast_ops,
	.quirks = SDHCI_QUIRK_BROKEN_TIMEOUT_VAL |
		SDHCI_QUIRK_MULTIBLOCK_READ_ACMD12|
//		SDHCI_QUIRK_BROKEN_DMA |
		SDHCI_QUIRK_CAP_CLOCK_BASE_BROKEN,
//		SDHCI_QUIRK_INVERTED_WRITE_PROTECT |
//		SDHCI_QUIRK_DELAY_AFTER_POWER,
};

static int sdhci_ast_probe(struct platform_device *pdev)
{
	return sdhci_pltfm_register(pdev, &sdhci_ast_pdata, 0);
}

static int sdhci_ast_remove(struct platform_device *pdev)
{
	return sdhci_pltfm_unregister(pdev);
}

static struct platform_driver sdhci_ast_driver = {
	.driver		= {
		.name	= "sdhci-ast",
		.pm	= SDHCI_PLTFM_PMOPS,
	},
	.probe		= sdhci_ast_probe,
	.remove		= sdhci_ast_remove,
};

module_platform_driver(sdhci_ast_driver);

MODULE_DESCRIPTION("SDHCI driver for AST SOC");
MODULE_AUTHOR("Ryan Chen <ryan_chen@aspeedtech.com>");
MODULE_LICENSE("GPL v2");

