// SPDX-License-Identifier: GPL-2.0+
/*
 * ASPEED Secure Digital Host Controller Interface.
 * Copyright (C) ASPEED Technology Inc.
 * Ryan Chen <ryan_chen@aspeedtech.com>
 *
 * Copyright 2019 IBM Corp.
  *
 */

#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/mmc/host.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/reset.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>

#include "sdhci-pltfm.h"

#define SDIO_GEN_INFO	0x00
#define  ASPEED_SDHCI_S0MMC8                      BIT(24)

struct sdhci_aspeed {
	struct clk *extclk;
	struct regmap *regmap;
};

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

	for (div = 1; div < 256; div *= 2) {
		if ((host->max_clk / div) <= clock)
			break;
	}
	div >>= 1;

	clk = div << SDHCI_DIVIDER_SHIFT;
	clk |= SDHCI_CLOCK_INT_EN;
	sdhci_writew(host, clk, SDHCI_CLOCK_CONTROL);

	/* Wait max 20 ms */
	timeout = 20;
	while (!((clk = sdhci_readw(host, SDHCI_CLOCK_CONTROL))
		 & SDHCI_CLOCK_INT_STABLE)) {
		if (timeout == 0) {
			pr_err("%s: Internal clock never stabilised.\n",
			       mmc_hostname(host->mmc));
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
	struct sdhci_aspeed *sdhci_aspeed = sdhci_pltfm_priv(pltfm_priv);
	u8 ctrl;
	bool mode = width == MMC_BUS_WIDTH_8 ? ASPEED_SDHCI_S0MMC8 : 0;

	/* Set/clear 8 bit mode */
	regmap_update_bits(sdhci_aspeed->regmap, SDIO_GEN_INFO,
			ASPEED_SDHCI_S0MMC8, mode);

	/* Set/clear 1 or 4 bit mode */
	ctrl = sdhci_readb(host, SDHCI_HOST_CONTROL);

	if (width == MMC_BUS_WIDTH_4)
		ctrl |= SDHCI_CTRL_4BITBUS;
	else
		ctrl &= ~SDHCI_CTRL_4BITBUS;

	sdhci_writeb(host, ctrl, SDHCI_HOST_CONTROL);
}

static struct sdhci_ops sdhci_aspeed_ops = {
	.set_clock = sdhci_aspeed_set_clock,
	.get_max_clock = sdhci_pltfm_clk_get_max_clock,
	.set_bus_width = sdhci_aspeed_set_bus_width,
	.get_timeout_clock = sdhci_pltfm_clk_get_max_clock,
	.reset = sdhci_reset,
	.set_uhs_signaling = sdhci_set_uhs_signaling,
};

static struct sdhci_pltfm_data sdhci_aspeed_pdata = {
	.ops = &sdhci_aspeed_ops,
	.quirks = SDHCI_QUIRK_CAP_CLOCK_BASE_BROKEN,
	.quirks2 = SDHCI_QUIRK2_CLOCK_DIV_ZERO_BROKEN,
};

static int sdhci_aspeed_probe(struct platform_device *pdev)
{
	struct sdhci_host *host;
	struct sdhci_pltfm_host *pltfm_host;
	struct sdhci_aspeed *sdhci_aspeed;

	int ret;

	host = sdhci_pltfm_init(pdev, &sdhci_aspeed_pdata,
			sizeof(*sdhci_aspeed));
	if (IS_ERR(host))
		return PTR_ERR(host);

	pltfm_host = sdhci_priv(host);
	sdhci_aspeed = sdhci_pltfm_priv(pltfm_host);

	sdhci_get_of_property(pdev);

	sdhci_aspeed->regmap = syscon_regmap_lookup_by_compatible(
			"aspeed,aspeed-sdhci-irq");
	if (IS_ERR(sdhci_aspeed->regmap)) {
		dev_err(&pdev->dev, "Unable find sdhci syscon\n");
		ret = PTR_ERR(sdhci_aspeed->regmap);
		goto err_pltfm_free;
	}

	sdhci_aspeed->extclk = devm_clk_get(&pdev->dev, "sdclk_ext");
	if (IS_ERR(sdhci_aspeed->extclk))
		return PTR_ERR(sdhci_aspeed->extclk);

	pltfm_host->clk = devm_clk_get(&pdev->dev, "sdclk");
	if (IS_ERR(pltfm_host->clk))
		return PTR_ERR(pltfm_host->clk);

	ret = clk_prepare_enable(sdhci_aspeed->extclk);
	if (ret) {
		dev_err(&pdev->dev, "Unable to enable sdclk_ext\n");
		goto err_pltfm_free;
	}

	clk_prepare_enable(pltfm_host->clk);
	if (ret) {
		dev_err(&pdev->dev, "Unable to enable sdclk\n");
		goto err_clk;
	}

	ret = mmc_of_parse(host->mmc);
	if (ret)
		goto err_sdhci_add;

	ret = sdhci_add_host(host);
	if (ret)
		goto err_sdhci_add;

	return 0;

err_sdhci_add:
	clk_disable_unprepare(pltfm_host->clk);
err_clk:
	clk_disable_unprepare(sdhci_aspeed->extclk);
err_pltfm_free:
	sdhci_pltfm_free(pdev);
	return ret;
}

static int sdhci_aspeed_remove(struct platform_device *pdev)
{
	struct sdhci_host	*host = platform_get_drvdata(pdev);
	int dead = (readl(host->ioaddr + SDHCI_INT_STATUS) == 0xffffffff);

	sdhci_remove_host(host, dead);
	sdhci_pltfm_free(pdev);
	return 0;
}

static const struct of_device_id sdhci_aspeed_of_match[] = {
	{ .compatible = "aspeed,sdhci-ast2400", .data = &sdhci_aspeed_pdata },
	{ .compatible = "aspeed,sdhci-ast2500", .data = &sdhci_aspeed_pdata },
	{}
};

MODULE_DEVICE_TABLE(of, sdhci_aspeed_of_match);

static struct platform_driver sdhci_aspeed_driver = {
	.driver		= {
		.name	= "sdhci-aspeed",
		.pm	= &sdhci_pltfm_pmops,
		.of_match_table = sdhci_aspeed_of_match,
	},
	.probe		= sdhci_aspeed_probe,
	.remove		= sdhci_aspeed_remove,
};

module_platform_driver(sdhci_aspeed_driver);

MODULE_DESCRIPTION("Driver for the ASPEED SDHCI Controller");
MODULE_AUTHOR("Ryan Chen <ryan_chen@aspeedtech.com>");
MODULE_LICENSE("GPL v2");
