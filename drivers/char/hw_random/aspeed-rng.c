/*
 * Copyright 2016 IBM Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/io.h>
#include <linux/hw_random.h>
#include <linux/platform_device.h>

#define ASPEED_RNG_CR		0x0
#define  ASPEED_RNG_ENABLE	0x1
#define	 ASPEED_RNG_MODE_MASK	0xe
#define ASPEED_RNG_DATA		0x4

struct aspeed_rng {
	void __iomem *base;
	struct hwrng rng;
	unsigned long next_read;
};

static int aspeed_rng_read(struct hwrng *hwrng, void *data, size_t max,
			   bool wait)
{
	struct aspeed_rng *rng = container_of(hwrng, struct aspeed_rng, rng);

	if (max < sizeof(u32))
		return 0;

	if (time_after(rng->next_read, jiffies))
		return 0;

	*(u32 *)data = ioread32(rng->base + ASPEED_RNG_DATA);
	rng->next_read = jiffies + usecs_to_jiffies(1);

	return sizeof(u32);
}

static int aspeed_rng_probe(struct platform_device *pdev)
{
	struct aspeed_rng *rng;
	struct resource *res;
	int ret;

	rng = devm_kzalloc(&pdev->dev, sizeof(*rng), GFP_KERNEL);
	if (!rng)
		return -ENOMEM;

	/* TODO regmap instead of direct access? */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	rng->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(rng->base))
		return PTR_ERR(rng->base);

	rng->rng.name = pdev->name;
	rng->rng.read = aspeed_rng_read;


	iowrite32(ASPEED_RNG_ENABLE, rng->base + ASPEED_RNG_CR);
	rng->next_read = jiffies;

	ret = hwrng_register(&rng->rng);
	if (ret)
		return ret;

	platform_set_drvdata(pdev, rng);

	return 0;
}

static int aspeed_rng_remove(struct platform_device *pdev)
{
	struct aspeed_rng *rng = platform_get_drvdata(pdev);

	hwrng_unregister(&rng->rng);

	iowrite32(0, rng->base + ASPEED_RNG_CR);

	return 0;
}

static const struct of_device_id aspeed_rng_match_table[] = {
	{ .compatible = "aspeed,ast2400-rng" },
	{ .compatible = "aspeed,ast2500-rng" },
	{}
};
MODULE_DEVICE_TABLE(of, aspeed_rng_match_table);

static struct platform_driver aspeed_rng_driver = {
	.probe	= aspeed_rng_probe,
	.remove = aspeed_rng_remove,
	.driver	= {
		.name = KBUILD_MODNAME,
		.of_match_table = aspeed_rng_match_table,
	},
};

module_platform_driver(aspeed_rng_driver);
