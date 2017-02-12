/*
 * Copyright 2017 IBM Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */

#include <linux/clk-provider.h>
#include <linux/io.h>
#include <linux/of_address.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>

#include "clk-aspeed.h"

static unsigned long aspeed_clk_clkin_recalc_rate(struct clk_hw *hw,
						  unsigned long parent_rate)
{
	struct aspeed_clk *clkin = to_aspeed_clk(hw);
	unsigned long rate;
	int ret;
	u32 reg;

	/* SCU70: Hardware Strapping Register  */
	ret = regmap_read(clkin->map, clkin->reg, &reg);
	if (ret) {
		pr_err("%s: regmap read failed\n", clk_hw_get_name(hw));
		return ret;
	}

	if (reg & BIT(23))
		rate = 25 * 1000 * 1000;
	else
		rate = 24 * 1000 * 1000;

	return rate;
}

static unsigned long aspeed_clk_hpll_recalc_rate(struct clk_hw *hw,
						 unsigned long clkin_rate)
{
	struct aspeed_clk *hpll = to_aspeed_clk(hw);
	unsigned long rate;
	int ret;
	u32 reg;

	/* SCU24: H-PLL Parameter Register */
	ret = regmap_read(hpll->map, hpll->reg, &reg);
	if (ret) {
		pr_err("%s: regmap read failed\n", clk_hw_get_name(hw));
		return ret;
	}

	if (reg & BIT(21)) {
		rate = 0;
	} else if (reg & BIT(20)) {
		rate = clkin_rate;
	} else {
		int p = (reg >> 13) & 0x3f;
		int m = (reg >> 5) & 0xff;
		int n = reg & 0x1f;

		rate = clkin_rate * ((m + 1) / (n + 1)) / (p + 1);
	}

	return rate;
}

static unsigned long aspeed_clk_ahb_recalc_rate(struct clk_hw *hw,
						unsigned long hpll_rate)
{
	struct aspeed_clk *ahb = to_aspeed_clk(hw);
	unsigned long rate;
	int ret;
	u32 reg;

	/* Strap register SCU70 */
	ret = regmap_read(ahb->map, ahb->reg, &reg);
	if (ret) {
		pr_err("%s: regmap read failed\n", clk_hw_get_name(hw));
		return ret;
	}

	/* Bits 11:9 define the AXI/AHB clock frequency ratio */
	reg = (reg >> 9) & 0x7;

	/* A value of zero is undefined */
	WARN_ON(reg == 0);

	rate = hpll_rate / 2 / (reg + 1);

	return rate;
}
static unsigned long aspeed_clk_apb_recalc_rate(struct clk_hw *hw,
						unsigned long hpll_rate)
{
	struct aspeed_clk *apb = to_aspeed_clk(hw);
	unsigned long rate;
	int ret;
	u32 reg;

	/* Clock selection register SCU08 */
	ret = regmap_read(apb->map, apb->reg, &reg);
	if (ret) {
		pr_err("%s: regmap read failed\n", clk_hw_get_name(hw));
		return ret;
	}

	reg = (reg >> 23) & 0x7;

	rate = hpll_rate / (4 * (reg + 1));

	return rate;

}

static const struct clk_ops aspeed_clk_clkin_ops = {
	.recalc_rate = aspeed_clk_clkin_recalc_rate,
};
static const struct clk_ops aspeed_clk_hpll_ops = {
	.recalc_rate = aspeed_clk_hpll_recalc_rate,
};
static const struct clk_ops aspeed_clk_apb_ops = {
	.recalc_rate = aspeed_clk_apb_recalc_rate,
};
static const struct clk_ops aspeed_clk_ahb_ops = {
	.recalc_rate = aspeed_clk_ahb_recalc_rate,
};

static void __init aspeed_clk_clkin_init(struct device_node *node)
{
	aspeed_clk_common_init(node, &aspeed_clk_clkin_ops);
}
CLK_OF_DECLARE(aspeed_clkin_clk, "aspeed,g5-clkin-clock", aspeed_clk_clkin_init);

static void __init aspeed_clk_hpll_init(struct device_node *node)
{
	aspeed_clk_common_init(node, &aspeed_clk_hpll_ops);
}
CLK_OF_DECLARE(aspeed_hpll_clk, "aspeed,g5-hpll-clock", aspeed_clk_hpll_init);

static void __init aspeed_clk_apb_init(struct device_node *node)
{
	aspeed_clk_common_init(node, &aspeed_clk_apb_ops);
}
CLK_OF_DECLARE(aspeed_apb_clk, "aspeed,g5-apb-clock", aspeed_clk_apb_init);

static void __init aspeed_clk_ahb_init(struct device_node *node)
{
	aspeed_clk_common_init(node, &aspeed_clk_ahb_ops);
}
CLK_OF_DECLARE(aspeed_ahb_clk, "aspeed,g5-ahb-clock", aspeed_clk_ahb_init);
