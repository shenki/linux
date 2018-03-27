/*
 * Copyright 2017 IBM Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */

#include <linux/clk-provider.h>
#include <linux/of_address.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>
#include <linux/platform_device.h>
#include <linux/of.h>

static int __init aspeed_clk_init_uart(struct device *dev,
					struct regmap *map)
{
	struct clk *clk;
	int ret, rate;
	u32 reg, offset;
	struct device_node *np = dev->of_node;
	const char *name = np->name;

	of_property_read_string(np, "clock-output-names", &name);

	ret = of_property_read_u32(np, "reg", &offset);
	if (ret) {
		dev_err(dev, "no reg property found\n");
		return ret;
	}

	/* SCU2C: Misc. Control Register */
	ret = regmap_read(map, offset, &reg);
	if (ret) {
		dev_err(dev, "regmap read failed\n");
		return ret;
	}

	/*
	 * The documentation does not indicate where this 24MHz clock is
	 * derived from.
	 */
	rate = 24000000;

	if (reg & BIT(12))
		rate /= 13;

	clk = clk_register_fixed_rate(dev, name, NULL, 0, rate);
	if (IS_ERR(clk)) {
		dev_err(dev, "failed to register clock\n");
		return PTR_ERR(clk);
	}

	return of_clk_add_provider(np, of_clk_src_simple_get, clk);
}

static int __init aspeed_clk_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = pdev->dev.of_node;
	struct regmap *map;
	int rc;

	map = syscon_node_to_regmap(np->parent);
	if (IS_ERR(map)) {
		pr_err("%s: No regmap found\n", np->full_name);
		return PTR_ERR(map);
	}

	rc = aspeed_clk_init_uart(dev, map);
	if (rc)
		return rc;

	return 0;
}

static const struct of_device_id aspeed_clk_of_match[] = {
	{ .compatible = "aspeed,g4-uart-clock" },
	{ .compatible = "aspeed,g5-uart-clock" },
	{}
};

static struct platform_driver aspeed_clk_driver = {
	.driver = {
		.name = "aspeed-clk",
		.of_match_table = aspeed_clk_of_match,
	},
};
builtin_platform_driver_probe(aspeed_clk_driver, aspeed_clk_probe);
