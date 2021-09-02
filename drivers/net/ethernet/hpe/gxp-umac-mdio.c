// SPDX-License-Identifier: GPL-2.0
/* Copyright (C) 2019 Hewlett-Packard Development Company, L.P.
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/phy.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/iopoll.h>

#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_mdio.h>
#include "gxp-umac-mdio.h"

struct umac_mdio_priv {
	void __iomem *base;
};

static int umac_mdio_read(struct mii_bus *bus, int phy_id, int reg)
{
	struct umac_mdio_priv *umac_mdio = bus->priv;
	unsigned int value;
	unsigned int status;
	int ret;
	// int i;

	status = __raw_readl(umac_mdio->base + UMAC_MII);

	status &= ~(UMAC_MII_PHY_ADDR_MASK | UMAC_MII_REG_ADDR_MASK);
	status |= ((phy_id << UMAC_MII_PHY_ADDR_SHIFT) &
			UMAC_MII_PHY_ADDR_MASK);
	status |= (reg & UMAC_MII_REG_ADDR_MASK);
	status |= UMAC_MII_MRNW; // set bit for read mode
	__raw_writel(status, umac_mdio->base + UMAC_MII);

	status |= UMAC_MII_MOWNER; // set bit to activate mii transfer
	__raw_writel(status, umac_mdio->base + UMAC_MII);

	/* poll the MOWNER bit to be cleared.
	 * for (i = 10000; i > 0; i--) {
	 *     if ((__raw_readl(umac_mdio->base + UMAC_MII) &
	 *         UMAC_MII_MOWNER) == 0x00) {
	 *         break;
	 *     }
	 * }
	 * if (i <= 0) {
	 *     dev_warn(bus->parent, "mdio read time out\n");
	 *     return -ETIMEDOUT;
	 * }
	 */
	ret = readl_poll_timeout(umac_mdio->base + UMAC_MII, status,
				 !(status & UMAC_MII_MOWNER), 1000, 100000);
	if (ret) {
		dev_warn(bus->parent, "mdio read time out\n");
		return -ETIMEDOUT;
	}

	value = __raw_readl(umac_mdio->base + UMAC_MII_DATA);
	return value;
}

static int umac_mdio_write(struct mii_bus *bus, int phy_id, int reg, u16 value)
{
	struct umac_mdio_priv *umac_mdio = bus->priv;
	unsigned int status;
	int ret;
//	int i;

	__raw_writel(value, umac_mdio->base + UMAC_MII_DATA);

	status = __raw_readl(umac_mdio->base + UMAC_MII);

	status &= ~(UMAC_MII_PHY_ADDR_MASK | UMAC_MII_REG_ADDR_MASK);
	status |= ((phy_id << UMAC_MII_PHY_ADDR_SHIFT) &
			UMAC_MII_PHY_ADDR_MASK);
	status |= (reg & UMAC_MII_REG_ADDR_MASK);
	status &= ~UMAC_MII_MRNW; // clear bit for write mode
	__raw_writel(status, umac_mdio->base + UMAC_MII);

	status |= UMAC_MII_MOWNER; // set bit to activate mii transfer
	__raw_writel(status, umac_mdio->base + UMAC_MII);

	/* poll the MOWNER bit to be cleared.
	 * for (i = 10000; i > 0; i--) {
	 *     if ((__raw_readl(umac_mdio->base + UMAC_MII) &
	 *         UMAC_MII_MOWNER) == 0x00) {
	 *         break;
	 *     }
	 * }
	 * if (i <= 0) {
	 *     dev_warn(bus->parent, "mdio write time out\n");
	 *     return -ETIMEDOUT;
	 * }
	 */
	ret = readl_poll_timeout(umac_mdio->base + UMAC_MII, status,
				 !(status & UMAC_MII_MOWNER), 1000, 100000);
	if (ret) {
		dev_warn(bus->parent, "mdio read time out\n");
		return -ETIMEDOUT;
	}

	return 0;
}

static int umac_mdio_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct mii_bus *bus;
	struct umac_mdio_priv *umac_mdio;

	int ret;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "fail to get resource\n");
		return -ENODEV;
	}

	bus = devm_mdiobus_alloc_size(&pdev->dev,
				      sizeof(struct umac_mdio_priv));
	if (!bus) {
		dev_err(&pdev->dev, "failed to alloc mii bus\n");
		return -ENOMEM;
	}

	snprintf(bus->id, MII_BUS_ID_SIZE, "%s", dev_name(&pdev->dev));

	bus->name	= dev_name(&pdev->dev);
	bus->read	= umac_mdio_read,
	bus->write	= umac_mdio_write,
	bus->parent	= &pdev->dev;
	umac_mdio = bus->priv;
	umac_mdio->base = devm_ioremap_resource(&pdev->dev, res);
	if (!umac_mdio->base) {
		dev_err(&pdev->dev, "failed to do ioremap\n");
		return -ENODEV;
	}

	platform_set_drvdata(pdev, umac_mdio);

	ret = of_mdiobus_register(bus, pdev->dev.of_node);

	if (ret < 0) {
		dev_err(&pdev->dev, "Cannot register MDIO bus (%d)\n", ret);
		return ret;
	}

	return 0;
}

static int umac_mdio_remove(struct platform_device *pdev)
{
	struct mii_bus *bus = platform_get_drvdata(pdev);

	if (bus)
		mdiobus_unregister(bus);

	return 0;
}

static const struct of_device_id umac_mdio_of_matches[] = {
	{ .compatible = "hpe,gxp-umac-mdio", },
	{},
};
MODULE_DEVICE_TABLE(of, umac_mdio_of_matches);

static struct platform_driver umac_driver = {
	.driver	= {
		.name    = "gxp-umac-mdio",
		.of_match_table = of_match_ptr(umac_mdio_of_matches),
	},
	.probe   = umac_mdio_probe,
	.remove  = umac_mdio_remove,
};

module_platform_driver(umac_driver);

MODULE_AUTHOR("Gilbert Chen <gilbert.chen@hpe.com");
MODULE_DESCRIPTION("HPE GXP UMAC MDIO driver");
