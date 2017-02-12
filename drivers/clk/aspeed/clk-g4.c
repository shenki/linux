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
		rate = 25;
	else if (reg & BIT(18))
		rate = 48;
	else
		rate = 24;

	return rate * 1000000;
}

static unsigned long aspeed_clk_hpll_recalc_rate(struct clk_hw *hw,
						 unsigned long clkin_rate)
{
	struct aspeed_clk *hpll = to_aspeed_clk(hw);
	unsigned long rate;
	int ret;
	u32 reg;
	const int rates[][4] = {
		{384, 360, 336, 408},
		{400, 375, 350, 425},
	};

	/* SCU70: Hardware strapping register */
	ret = regmap_read(hpll->map, hpll->reg, &reg);
	if (ret) {
		pr_err("%s: regmap read failed\n", clk_hw_get_name(hw));
		return ret;
	}

	reg = (reg >> 8) & GENMASK(1, 0);

	if (clkin_rate == 48000000 || clkin_rate == 24000000)
		rate = rates[0][reg];
	else if (clkin_rate == 25000000)
		rate = rates[1][reg];
	else {
		WARN("%s: unknown clkin frequency %ldHz\n",
				clk_hw_get_name(hw), clkin_rate);
		return -EINVAL;
	}

	return rate * 1000000;
}

static unsigned long aspeed_clk_apb_recalc_rate(struct clk_hw *hw,
						unsigned long hpll_rate)
{
	struct aspeed_clk *apb = to_aspeed_clk(hw);
	int ret;
	u32 reg;

	/* SCU08: Clock Selection Register */
	ret = regmap_read(apb->map, apb->reg, &reg);
	if (ret) {
		pr_err("%s: regmap read failed\n", clk_hw_get_name(hw));
		return ret;
	}

	reg = (reg >> 23) & 0x3;

	return hpll_rate / (2 + 2 * reg);
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

	/* bits 11:10 define the CPU/AHB clock frequency ratio */
	reg = (reg >> 10) & 0x03;

	/* A value of zero is undefined */
	WARN_ON(reg == 0);

	rate = hpll_rate / (reg + 1);

	return rate;
}

static const struct clk_ops aspeed_clk_hpll_ops = {
	.recalc_rate = aspeed_clk_hpll_recalc_rate,
};
static const struct clk_ops aspeed_clk_apb_ops = {
	.recalc_rate = aspeed_clk_apb_recalc_rate,
};
static const struct clk_ops aspeed_clk_ahb_ops = {
	.recalc_rate = aspeed_clk_ahb_recalc_rate,
};
static const struct clk_ops aspeed_clk_clkin_ops = {
	.recalc_rate = aspeed_clk_clkin_recalc_rate,
};

static void __init aspeed_clk_clkin_init(struct device_node *node)
{
	aspeed_clk_common_init(node, &aspeed_clk_clkin_ops);
}
CLK_OF_DECLARE(aspeed_clkin_clk, "aspeed,g4-clkin-clock", aspeed_clk_clkin_init);

static void __init aspeed_clk_hpll_init(struct device_node *node)
{
	aspeed_clk_common_init(node, &aspeed_clk_hpll_ops);
}
CLK_OF_DECLARE(aspeed_hpll_clock, "aspeed,g4-hpll-clock",
	       aspeed_clk_hpll_init);

static void __init aspeed_clk_apb_init(struct device_node *node)
{
	aspeed_clk_common_init(node, &aspeed_clk_apb_ops);
}
CLK_OF_DECLARE(aspeed_apb_clock, "aspeed,g4-apb-clock",
	       aspeed_clk_apb_init);

static void __init aspeed_clk_ahb_init(struct device_node *node)
{
	aspeed_clk_common_init(node, &aspeed_clk_ahb_ops);
}
CLK_OF_DECLARE(aspeed_ahb_clk, "aspeed,g4-ahb-clock", aspeed_clk_ahb_init);
