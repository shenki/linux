/*
 * Copyright 2017 IBM Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */

#include <linux/io.h>
#include <linux/of_address.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>
#include <linux/slab.h>

#include "clk-aspeed.h"

void __init aspeed_clk_common_init(struct device_node *node,
				   const struct clk_ops *ops, unsigned long flags)
{
	struct clk *clk;
	int ret;
	const char *name = node->name;
	const char *parent_name;
	struct aspeed_clk *aspeed_clk;
	struct clk_init_data init;

	aspeed_clk = kzalloc(sizeof(*aspeed_clk), GFP_KERNEL);
	if (!aspeed_clk)
		return;

	of_property_read_string(node, "clock-output-names", &name);
	parent_name = of_clk_get_parent_name(node, 0);

	aspeed_clk->map = syscon_node_to_regmap(node->parent);
	if (IS_ERR(aspeed_clk->map)) {
		pr_err("%s: No regmap found\n", node->full_name);
		goto err;
	}
	of_property_read_u32(node, "div", &aspeed_clk->div);

	of_property_read_u32(node, "enable", &aspeed_clk->enable);

	init.name = name;
	init.ops = ops;
	init.flags = flags;
	init.parent_names = parent_name ? &parent_name : NULL;
	init.num_parents = parent_name ? 1 : 0;
	aspeed_clk->hw.init = &init;

	clk = clk_register(NULL, &aspeed_clk->hw);
	if (IS_ERR(clk)) {
		pr_err("%s: failed to register clock\n", node->full_name);
		goto err;
	}

	of_clk_add_provider(node, of_clk_src_simple_get, clk);

	return;
err:
	kfree(aspeed_clk);
}
