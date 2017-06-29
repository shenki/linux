/*
 * Copyright 2017 IBM Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */

#define pr_fmt(fmt) "clk-aspeed: " fmt

#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/clk-provider.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>
#include <linux/spinlock.h>

#include <dt-bindings/clock/aspeed-clock.h>

#define ASPEED_RESET_CTRL	0x04
#define ASPEED_RESET_CTRL_MASK	(GENMASK(31, 26) | BIT(19) | BIT(17))
#define ASPEED_CLK_STOP_CTRL	0x0c

/* Globally visible clocks */
static DEFINE_SPINLOCK(aspeed_clk_lock);

struct aspeed_gate_data {
	u8 clock_idx;
	s8 reset_idx;
	const char *name;
	const char *parent_name;
	unsigned long flags;
};

struct aspeed_clk_gate {
	struct clk_hw hw;
	struct regmap 	*map;
	u8		clock_idx;
	u8		reset_idx;
	u8		flags;
	spinlock_t	*lock;
};

#define to_aspeed_clk_gate(_hw) container_of(_hw, struct aspeed_clk_gate, hw)

/* Keeps track of all clocks */
static struct clk_hw_onecell_data *aspeed_clk_data;

/* TODO: ask aspeed about the actual parent data */
static const struct aspeed_gate_data aspeed_gates[] __initconst = {
/* 	 clk rst  name			parent		flags	*/
/*	 0c  04 						*/
	{  0, -1, "eclk-gate",		"hpll",		0 }, /* Video Engine */
	{  1,  7, "gclk-gate",		"hpll",		0 }, /* 2D engine */
	{  2, -1, "mclk-gate",		"clkin",	CLK_IS_CRITICAL }, /* SDRAM */
	{  3,  6, "vclk-gate",		"hpll",		0 }, /* Video Capture */
	{  4, 10, "bclk-gate",		"hpll",		0 }, /* PCIe/PCI */
	{  5, -1, "dclk-gate",		"hpll",		0 }, /* DAC */
	{  6, -1, "refclk-gate",	"clkin",	CLK_IS_CRITICAL },
	{  8,  5, "lclk-gate",		"hpll",		0 }, /* LPC */
	{ 10, 13, "d1clk-gate",		"hpll",		0 }, /* GFX CRT */
	/* 11: reserved */
	/* 12: reserved */
	{ 13, 4, "yclk-gate",		"ahb", 		0 }, /* HAC */
	{ 15, -1, "uart1clk-gate",	"uart", 	0 }, /* UART1 */
	{ 16, -1, "uart2clk-gate",	"uart", 	0 }, /* UART2 */
	{ 17, -1, "uart5clk-gate",	"uart", 	0 }, /* UART5 */
	/* 18: reserved */
	{ 19, -1, "espiclk-gate",	"hclk",	0 }, /* eSPI */
	{ 20, 11, "mac1clk-gate",	"clkin", 	0 }, /* MAC1 */
	{ 21, 12, "mac2clk-gate",	"clkin", 	0 }, /* MAC2 */
	/* 22: reserved */
	/* 23: reserved */
	{ 24, -1, "rsaclk-gate",	"ahb", 		0 }, /* RSA */
	{ 25, -1, "uart3clk-gate",	"uart", 	0 }, /* UART3 */
	{ 26, -1, "uart4clk-gate",	"uart", 	0 }, /* UART4 */
	{ 27, 16, "sdclk-gate",		"hpll", 	0 }, /* SDIO/SD */
	{ 28, -1, "lhclk-gate",		"hpll",		0 }, /* LPCM /LPC+ */
	/* 29: reserved */
	/* 30: reserved */
	/* 31: reserved */
};

/* TODO: write custom handlers for usb enablement */
static const struct aspeed_gate_data aspeed_gates_usb[] __initconst = {
	{  7,  3, "usbclk-gate",	"hpll",		0 }, /* USB */
	{  9, 15, "usb11clk-gate",	"hpll",		0 }, /* USB1.1 */
	{ 14, 14, "usb2host-gate",	"hpll",		0 }, /* USB2 */
};

/* TODO: handle resets that don't have their own clocks */
static const struct aspeed_gate_data aspeed_resets[] __initconst = {
	{  -1, 25, "x-dma",		"hpll",		0 },
	{  -1, 24, "mctp",		"hpll",		0 },
	{  -1, 23, "adc",		"hpll",		0 },
	{  -1, 22, "jtag-master",	"hpll",		0 },
	{  -1, 18, "mic",		"hpll",		0 },
	{  -1, 	9, "pwm",		"hpll",		0 },
	{  -1, 	8, "pci-vga",		"hpll",		0 },
};

/* TODO: handle async resets that don't have their own clocks */
static const struct aspeed_gate_data aspeed_async_resets[] __initconst = {
	{  -1, 	2, "i2c",		"ahb",		0 },
	{  -1, 	1, "ahb",		"hpll",		0 },
};

static int aspeed_clk_enable(struct clk_hw *hw)
{
	struct aspeed_clk_gate *gate = to_aspeed_clk_gate(hw);
	unsigned long flags;
	u32 reg;

	spin_lock_irqsave(gate->lock, flags);

	if (gate->reset_idx >= 0) {
		/* Put IP in reset */
		regmap_read(gate->map, ASPEED_RESET_CTRL, &reg);
		reg |= BIT(gate->clock_idx);
		regmap_write(gate->map, ASPEED_RESET_CTRL, reg);

		/* Delay 100us */
		udelay(100);
	}

	/* Enable clock */
	regmap_read(gate->map, ASPEED_CLK_STOP_CTRL, &reg);
	reg &= ~BIT(gate->clock_idx);
	regmap_write(gate->map, ASPEED_CLK_STOP_CTRL, reg);

	if (gate->reset_idx >= 0) {
		/* Delay 10ms */
		/* TODO: can we sleep here? */
		msleep(10);

		/* Take IP out of reset */
		regmap_read(gate->map, ASPEED_RESET_CTRL, &reg);
		reg &= ~BIT(gate->clock_idx);
		regmap_write(gate->map, ASPEED_RESET_CTRL, reg);
	}

	spin_unlock_irqrestore(gate->lock, flags);

	return 0;
}

static void aspeed_clk_disable(struct clk_hw *hw)
{
	struct aspeed_clk_gate *gate = to_aspeed_clk_gate(hw);
	unsigned long flags;
	u32 reg;

	spin_lock_irqsave(gate->lock, flags);

	/* Disable clock */
	regmap_read(gate->map, ASPEED_CLK_STOP_CTRL, &reg);
	reg |= BIT(gate->clock_idx);
	regmap_write(gate->map, ASPEED_CLK_STOP_CTRL, reg);

	spin_unlock_irqrestore(gate->lock, flags);
}

static int aspeed_clk_is_enabled(struct clk_hw *hw)
{
	u32 reg;
	struct aspeed_clk_gate *gate = to_aspeed_clk_gate(hw);

	regmap_read(gate->map, ASPEED_CLK_STOP_CTRL, &reg);

	return (reg & BIT(gate->clock_idx)) ? 0 : 1;
}

static const struct clk_ops aspeed_clk_gate_ops = {
	.enable = aspeed_clk_enable,
	.disable = aspeed_clk_disable,
	.is_enabled = aspeed_clk_is_enabled,
};

static struct clk_hw *aspeed_clk_hw_register_gate(struct device *dev,
		const char *name, const char *parent_name, unsigned long flags,
		struct regmap *map, u8 clock_idx, u8 reset_idx,
		u8 clk_gate_flags, spinlock_t *lock)
{
	struct aspeed_clk_gate *gate;
	struct clk_hw *hw;
	struct clk_init_data init;
	int ret;

	/* allocate the gate */
	gate = kzalloc(sizeof(*gate), GFP_KERNEL);
	if (!gate)
		return ERR_PTR(-ENOMEM);

	init.name = name;
	init.ops = &aspeed_clk_gate_ops;
	init.flags = flags | CLK_IS_BASIC;
	init.parent_names = parent_name ? &parent_name : NULL;
	init.num_parents = parent_name ? 1 : 0;

	gate->map = map;
	gate->clock_idx = clock_idx;
	gate->reset_idx = reset_idx;
	gate->flags = clk_gate_flags;
	gate->lock = lock;
	gate->hw.init = &init;

	hw = &gate->hw;
	ret = clk_hw_register(dev, hw);
	if (ret) {
		kfree(gate);
		hw = ERR_PTR(ret);
	}

	return hw;
}


static int __init aspeed_clk_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct regmap *map;
	struct clk_hw *hw;
	u32 val;
	int i;

	map = syscon_node_to_regmap(np);
	if (IS_ERR(map)) {
		dev_err(dev, "no syscon regmap\n");
		return PTR_ERR(map);
	}

	/* UART clock div13 setting */
	regmap_read(map, 0x2c, &val);
	if (val & BIT(12))
		val = 24000000 / 13;
	else
		val = 24000000;
	hw = clk_hw_register_fixed_rate(NULL, "uart", NULL, 0, val);
	aspeed_clk_data->hws[ASPEED_CLK_UART] = hw;

	for (i = 0; i < ARRAY_SIZE(aspeed_gates); i++) {
		const struct aspeed_gate_data *gd;

		gd = &aspeed_gates[i];
		aspeed_clk_data->hws[ASPEED_CLK_GATES + i] =
			aspeed_clk_hw_register_gate(NULL, gd->name,
					     gd->parent_name,
					     gd->flags,
					     map,
					     gd->clock_idx,
					     gd->reset_idx,
					     CLK_GATE_SET_TO_DISABLE,
					     &aspeed_clk_lock);
	}

	/* TODO: Register USB gates, resets, async resets */

	return 0;
};

static const struct of_device_id aspeed_clk_dt_ids[] = {
	{ .compatible = "aspeed,g5-scu", },
	{ },
};

static struct platform_driver aspeed_clk_driver = {
	.probe  = aspeed_clk_probe,
	.driver = {
		.name = "aspeed-clk",
		.of_match_table = aspeed_clk_dt_ids,
		.suppress_bind_attrs = true,
	},
};
builtin_platform_driver(aspeed_clk_driver);

static void __init aspeed_cc_init(struct device_node *np)
{
	struct regmap *map;
	struct clk_hw *hw;
	unsigned long freq;
	unsigned int mult, div;
	u32 val;
	int ret;
	int i;

	aspeed_clk_data = kzalloc(sizeof(*aspeed_clk_data) +
			sizeof(*aspeed_clk_data->hws) * ASPEED_NUM_CLKS,
			GFP_KERNEL);
	if (!aspeed_clk_data)
		return;

	/*
	 * This way all clock fetched before the platform device probes,
	 * except those we assign here for early use, will be deferred.
	 */
	for (i = 0; i < ASPEED_NUM_CLKS; i++)
		aspeed_clk_data->hws[i] = ERR_PTR(-EPROBE_DEFER);

	map = syscon_node_to_regmap(np);
	if (IS_ERR(map)) {
		pr_err("no syscon regmap\n");
		return;
	}
	/*
	 * We check that the regmap works on this very first access,
	 * but as this is an MMIO-backed regmap, subsequent regmap
	 * access is not going to fail and we skip error checks from
	 * this point.
	 */
	ret = regmap_read(map, 0x70, &val);
	if (ret) {
		pr_err("failed to read strapping register\n");
		return;
	}

	/* CLKIN is the crystal oscillator, 24 or 25MHz selected by strapping */
	if (val & BIT(23))
		freq = 25000000;
	else
		freq = 24000000;
	hw = clk_hw_register_fixed_rate(NULL, "clkin", NULL, 0, freq);
	pr_debug("clkin @%lu MHz\n", freq / 1000000);

	/* High-speed PLL clock derived from the crystal. This the CPU clock */
	regmap_read(map, 0x24, &val);
	WARN_ON(val & BIT(21));
	if (val & BIT(20)) {
		mult = div = 1;
	} else {
		int p = (val >> 13) & 0x3f;
		int m = (val >> 5) & 0xff;
		int n = val & 0x1f;

		mult = (m + 1) / (n + 1);
		div = p + 1;
	}
	hw = clk_hw_register_fixed_factor(NULL, "hpll", "clkin", 0, mult, div);

	/* Strap bits 11:9 define the AXI/AHB clock frequency ratio (aka HCLK)*/
	regmap_read(map, 0x70, &val);
	val = (val >> 9) & 0x7;
	WARN_ON(val == 0);
	div = 2 * (val + 1);
	hw = clk_hw_register_fixed_factor(NULL, "ahb", "hpll", 0, 1, div); 
	aspeed_clk_data->hws[ASPEED_CLK_AHB] = hw;

	/* APB clock clock selection register SCU08 (aka PCLK) */
	regmap_read(map, 0x08, &val);
	val = (val >> 23) & 0x7;
	div = 4 * (val + 1);
	hw = clk_hw_register_fixed_factor(NULL, "apb", "hpll", 0, 1, div); 
	aspeed_clk_data->hws[ASPEED_CLK_APB] = hw;

	aspeed_clk_data->num = ASPEED_NUM_CLKS;
	ret = of_clk_add_hw_provider(np, of_clk_hw_onecell_get, aspeed_clk_data);
	if (ret)
		pr_err("failed to add DT provider: %d\n", ret);
};
CLK_OF_DECLARE_DRIVER(aspeed_cc, "aspeed,g5-scu", aspeed_cc_init);
