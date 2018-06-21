/*
 * ASPEED Secure Digital Host Controller Interface.
 *
 * Copyright (C) ASPEED Technology Inc.
 * Ryan Chen <ryan_chen@aspeedtech.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at
 * your option) any later version.
 *
 */

#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/mmc/host.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/mmc/sdhci-aspeed-data.h>
#include <linux/reset.h>
#include "sdhci-pltfm.h"

static void sdhci_aspeed_set_clock(struct sdhci_host *host, unsigned int clock)
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

static void sdhci_aspeed_set_bus_width(struct sdhci_host *host, int width)
{
	struct sdhci_pltfm_host *pltfm_priv = sdhci_priv(host);
	struct aspeed_sdhci_irq *sdhci_irq = sdhci_pltfm_priv(pltfm_priv);

	u8 ctrl = sdhci_readb(host, SDHCI_HOST_CONTROL);

	if(sdhci_irq->regs) {
		if (width == MMC_BUS_WIDTH_8) {
			aspeed_sdhci_set_8bit_mode(sdhci_irq, 1);
		} else {
			aspeed_sdhci_set_8bit_mode(sdhci_irq, 0);
		}
	}
	if (width == MMC_BUS_WIDTH_4)
		ctrl |= SDHCI_CTRL_4BITBUS;
	else
		ctrl &= ~SDHCI_CTRL_4BITBUS;

	sdhci_writeb(host, ctrl, SDHCI_HOST_CONTROL);

}

static unsigned int sdhci_aspeed_get_max_clk(struct sdhci_host *host)
{
	struct sdhci_pltfm_host *pltfm_priv = sdhci_priv(host);

	return clk_get_rate(pltfm_priv->clk);
}

static unsigned int sdhci_aspeed_get_timeout_clk(struct sdhci_host *host)
{
	struct sdhci_pltfm_host *pltfm_priv = sdhci_priv(host);

	return (clk_get_rate(pltfm_priv->clk)/1000000);
}

/*
	AST2300/AST2400 : SDMA/PIO
	AST2500 : ADMA/SDMA/PIO
*/
static struct sdhci_ops  sdhci_aspeed_ops= {
#ifdef CONFIG_CAM
	.set_clock = sdhci_set_clock,
#else
	.set_clock = sdhci_aspeed_set_clock,
#endif	
	.get_max_clock = sdhci_aspeed_get_max_clk,
	.set_bus_width = sdhci_aspeed_set_bus_width,
	.get_timeout_clock = sdhci_aspeed_get_timeout_clk,
	.reset = sdhci_reset,
	.set_uhs_signaling = sdhci_set_uhs_signaling,
};

static struct sdhci_pltfm_data sdhci_aspeed_pdata = {
	.ops = &sdhci_aspeed_ops,
};

static int sdhci_aspeed_probe(struct platform_device *pdev)
{
	struct sdhci_host *host;
	struct device_node *pnode;
	struct device_node *np = pdev->dev.of_node;
	struct sdhci_pltfm_host *pltfm_host;
	struct aspeed_sdhci_irq *sdhci_irq;
	struct reset_control *reset;	

	int ret;

	host = sdhci_pltfm_init(pdev, &sdhci_aspeed_pdata, sizeof(struct aspeed_sdhci_irq));
	if (IS_ERR(host))
		return PTR_ERR(host);

	pltfm_host = sdhci_priv(host);
	sdhci_irq = sdhci_pltfm_priv(pltfm_host);

	host->quirks = SDHCI_QUIRK_CAP_CLOCK_BASE_BROKEN;
	host->quirks2 = SDHCI_QUIRK2_CLOCK_DIV_ZERO_BROKEN;
	sdhci_get_of_property(pdev);

	pltfm_host->clk = devm_clk_get(&pdev->dev, NULL);

	pnode = of_parse_phandle(np, "interrupt-parent", 0);
	if(pnode)
		memcpy(sdhci_irq, pnode->data, sizeof(struct aspeed_sdhci_irq));

	ret = mmc_of_parse(host->mmc);
	if (ret)
		goto err_sdhci_add;

	ret = sdhci_add_host(host);
	if (ret)
		goto err_sdhci_add;

	return 0;

err_sdhci_add:
	clk_disable_unprepare(pltfm_host->clk);
	sdhci_pltfm_free(pdev);
	return ret;
}


static const struct of_device_id sdhci_aspeed_of_match_table[] = {
        { .compatible = "aspeed,sdhci-ast", .data = &sdhci_aspeed_pdata },
        {}
};

MODULE_DEVICE_TABLE(of, sdhci_aspeed_of_match_table);

static struct platform_driver sdhci_aspeed_driver = {
	.driver		= {
		.name	= "sdhci-ast",
		.pm	= &sdhci_pltfm_pmops,
		.of_match_table = sdhci_aspeed_of_match_table,
	},
	.probe		= sdhci_aspeed_probe,
	.remove		= sdhci_pltfm_unregister,
};

module_platform_driver(sdhci_aspeed_driver);

MODULE_DESCRIPTION("Driver for the ASPEED SDHCI Controller");
MODULE_AUTHOR("Ryan Chen <ryan_chen@aspeedtech.com>");
MODULE_LICENSE("GPL v2");
