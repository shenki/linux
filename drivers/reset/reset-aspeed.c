/*
 * reset-aspeed.c - Reset Controller Driver for the Aspeed SoC
 *
 * Copyright (C) ASPEED Technology Inc.
 * Ryan Chen <ryan_chen@aspeedtech.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 *
 */

#include <linux/err.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/mfd/syscon.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/reset-controller.h>
#include <linux/regmap.h>
#include <linux/types.h>

struct aspeed_reset_data {
	struct regmap *reg;
	struct reset_controller_dev rcdev;
	u32 reset0;
	u32 reset1;	
};

#define to_aspeed_reset_data(p)		\
	container_of((p), struct aspeed_reset_data, rcdev)

static int aspeed_reset_assert(struct reset_controller_dev *rcdev,
			     unsigned long id)
{
	struct aspeed_reset_data *sysrst = to_aspeed_reset_data(rcdev);
	
	pr_debug("%s: %s reset id %ld\n", KBUILD_MODNAME, __func__, id);

	if(id >= 32) {
		return regmap_update_bits(sysrst->reg,
						sysrst->reset1,
						BIT(id-32),
						BIT(id-32));
	} else {
		return regmap_update_bits(sysrst->reg,
						sysrst->reset0,
						BIT(id),
						BIT(id));
	}
}

static int aspeed_reset_deassert(struct reset_controller_dev *rcdev,
			       unsigned long id)
{
	struct aspeed_reset_data *sysrst = to_aspeed_reset_data(rcdev);

	pr_debug("%s: %s reset id %ld\n", KBUILD_MODNAME, __func__, id);

	if(id >= 32) {
		return regmap_update_bits(sysrst->reg,
						sysrst->reset1,
						BIT(id-32),
						~BIT(id-32));
	} else {
		return regmap_update_bits(sysrst->reg,
						sysrst->reset0,
						BIT(id),
						~BIT(id));		
	}
}

static int aspeed_reset_status(struct reset_controller_dev *rcdev,
			     unsigned long id)
{
	struct aspeed_reset_data *sysrst = to_aspeed_reset_data(rcdev);
	int ret;
	u32 reg;

	pr_debug("%s: %s reset id %ld\n", KBUILD_MODNAME, __func__, id);

	if(id >= 32) {
		ret = regmap_read(sysrst->reg, sysrst->reset1, &reg);
		if (ret)
			return ret;
		return !!(reg & BIT(id - 32));
	} else {
		ret = regmap_read(sysrst->reg, sysrst->reset0, &reg);
		if (ret)
			return ret;
		return !!(reg & BIT(id));	
	}
}

static const struct reset_control_ops aspeed_reset_ops = {
	.assert		= aspeed_reset_assert,
	.deassert	= aspeed_reset_deassert,
	.status		= aspeed_reset_status,
};

static int aspeed_reset_probe(struct platform_device *pdev)
{
	struct resource *res0, *res1;
	struct aspeed_reset_data *sysrst;

	sysrst = devm_kzalloc(&pdev->dev, sizeof(*sysrst), GFP_KERNEL);
	if (!sysrst)
		return -ENOMEM;
	platform_set_drvdata(pdev, sysrst);

	sysrst->reg = syscon_node_to_regmap(pdev->dev.of_node->parent);

	if (IS_ERR(sysrst->reg)) {
		dev_err(&pdev->dev, "unable to get ast-reg regmap");
		return PTR_ERR(sysrst->reg);
	}

	res0 = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res0) {
		dev_err(&pdev->dev, "missing IO resource\n");
		return -ENODEV;
	}

	sysrst->reset0 = res0->start;

	res1 = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!res1) {
		dev_err(&pdev->dev, "no reset 2\n");
	} else {
		sysrst->reset1 = res1->start;
	}

	sysrst->rcdev.owner = THIS_MODULE;	
	sysrst->rcdev.nr_resets = (resource_size(res0) / 4 * BITS_PER_LONG);
	if(res1)
		sysrst->rcdev.nr_resets += (resource_size(res1) / 4 * BITS_PER_LONG);
	
	sysrst->rcdev.ops = &aspeed_reset_ops;
	sysrst->rcdev.of_node = pdev->dev.of_node;
//	printk("aspeed_reset_probe end res->start %x, sysrst->rcdev.nr_resets %d \n", res0->start, sysrst->rcdev.nr_resets);

	return devm_reset_controller_register(&pdev->dev, &sysrst->rcdev);
}

static const struct of_device_id aspeed_reset_dt_ids[] = {
	{ .compatible = "aspeed,ast-reset", },
	{ /* sentinel */ },
};

static struct platform_driver aspeed_reset_driver = {
	.probe	= aspeed_reset_probe,
	.driver = {
		.name		= KBUILD_MODNAME,
		.of_match_table	= aspeed_reset_dt_ids,
	},
};
//module_platform_driver(aspeed_reset_driver);
static int __init reset_aspeed_init(void)
{
	return platform_driver_register(&aspeed_reset_driver);
}
core_initcall(reset_aspeed_init);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Ryan Chen <ryan_chen@aspeedtech.com>");
MODULE_DESCRIPTION("ASPEED Reset Controller Driver");
