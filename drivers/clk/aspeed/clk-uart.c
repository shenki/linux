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

static void __init aspeed_of_uart_clk_init(struct device_node *node)
{
	struct clk *clk;
	void __iomem *base;
	int reg, rate;
	const char *name = node->name;

	of_property_read_string(node, "clock-output-names", &name);

	base = of_iomap(node, 0);
	if (!base) {
		pr_err("%s: of_iomap failed\n", node->full_name);
		return;
	}
	reg = readl(base);
	iounmap(base);

	/*
	 * The documentation does not indicate where this 24MHz clock is
	 * derived from.
	 */
	rate = 24000000;

	if (reg & BIT(12))
		rate /= 13;

	clk = clk_register_fixed_rate(NULL, name, NULL, 0, rate);
	if (IS_ERR(clk)) {
		pr_err("%s: failed to register clock\n", node->full_name);
		return;
	}

	clk_register_clkdev(clk, NULL, name);
	of_clk_add_provider(node, of_clk_src_simple_get, clk);
}
CLK_OF_DECLARE(aspeed_uart_clock, "aspeed,uart-clock",
	       aspeed_of_uart_clk_init);
