/*
 * Copyright 2016 IBM Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/io.h>
#include <linux/of_address.h>
#include <linux/clkdev.h>

static void __init aspeed_of_hpll_clk_init(struct device_node *node)
{
	struct clk *clk, *clkin_clk;
	void __iomem *base;
	int reg, rate, clkin;
	const char *name = node->name;
	const char *parent_name;
	const int rates[][4] = {
		{384, 360, 336, 408},
		{400, 375, 350, 425},
	};

	of_property_read_string(node, "clock-output-names", &name);
	parent_name = of_clk_get_parent_name(node, 0);

	base = of_iomap(node, 0);
	if (!base) {
		pr_err("%s: of_iomap failed\n", node->full_name);
		return;
	}
	reg = readl(base);
	iounmap(base);

	clkin_clk = of_clk_get(node, 0);
	if (IS_ERR(clkin_clk)) {
		pr_err("%s: of_clk_get failed\n", node->full_name);
		return;
	}

	clkin = clk_get_rate(clkin_clk);

	reg = (reg >> 8) & 0x2;

	if (clkin == 48000000 || clkin == 24000000)
		rate = rates[0][reg] * 1000000;
	else if (clkin == 25000000)
		rate = rates[1][reg] * 1000000;
	else {
		pr_err("%s: unknown clkin frequency %dHz\n",
				node->full_name, clkin);
		WARN_ON(1);
	}

	clk = clk_register_fixed_rate(NULL, name, parent_name, 0, rate);
	if (IS_ERR(clk)) {
		pr_err("%s: failed to register clock\n", node->full_name);
		return;
	}

	clk_register_clkdev(clk, NULL, name);
	of_clk_add_provider(node, of_clk_src_simple_get, clk);
}
CLK_OF_DECLARE(aspeed_hpll_clock, "aspeed,g4-hpll-clock",
	       aspeed_of_hpll_clk_init);

static void __init aspeed_of_apb_clk_init(struct device_node *node)
{
	struct clk *clk, *hpll_clk;
	void __iomem *base;
	int reg, rate;
	const char *name = node->name;
	const char *parent_name;

	of_property_read_string(node, "clock-output-names", &name);
	parent_name = of_clk_get_parent_name(node, 0);

	base = of_iomap(node, 0);
	if (!base) {
		pr_err("%s: of_iomap failed\n", node->full_name);
		return;
	}
	reg = readl(base);
	iounmap(base);

	hpll_clk = of_clk_get(node, 0);
	if (IS_ERR(hpll_clk)) {
		pr_err("%s: of_clk_get failed\n", node->full_name);
		return;
	}

	reg = (reg >> 23) & 0x3;
	rate = clk_get_rate(hpll_clk) / (2 + 2 * reg);

	clk = clk_register_fixed_rate(NULL, name, parent_name, 0, rate);
	if (IS_ERR(clk)) {
		pr_err("%s: failed to register clock\n", node->full_name);
		return;
	}

	clk_register_clkdev(clk, NULL, name);
	of_clk_add_provider(node, of_clk_src_simple_get, clk);
}
CLK_OF_DECLARE(aspeed_apb_clock, "aspeed,g4-apb-clock",
	       aspeed_of_apb_clk_init);
