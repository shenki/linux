/*
 * Copyright 2017 IBM Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */

#include <linux/clk-provider.h>

struct aspeed_clk {
	struct clk_hw	hw;
	struct regmap	*map;
	u32		enable;
	u32		div;
};

#define to_aspeed_clk(_hw) container_of(_hw, struct aspeed_clk, hw)

void __init aspeed_clk_common_init(struct device_node *node,
				   const struct clk_ops *ops, unsigned long flags);

