/*
 * Copyright (c) Aspeed
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/acpi.h>
#include <linux/errno.h>
#include <linux/etherdevice.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/netdevice.h>
#include <linux/of_address.h>
#include <linux/of.h>
#include <linux/of_mdio.h>
#include <linux/of_platform.h>
#include <linux/phy.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/spinlock_types.h>

struct aspeed_mdio_regs {
	u32 phycr;
	u32 phydata;
};

struct aspeed_mdio_device {
	struct aspeed_mdio_regs __iomem *regs;
	struct clk	*clk;
	struct device	*dev;
	struct mii_bus	*bus;
};


#if 0
#define MDIO_CTL_DEV_ADDR(x)	(x & 0x1f)
#define MDIO_CTL_PORT_ADDR(x)	((x & 0x1f) << 5)

#define MDIO_TIMEOUT			1000000

/* mdio reg */
#define MDIO_COMMAND_REG		0x0
#define MDIO_ADDR_REG			0x4
#define MDIO_WDATA_REG			0x8
#define MDIO_RDATA_REG			0xc
#define MDIO_STA_REG			0x10

/* cfg phy bit map */
#define MDIO_CMD_DEVAD_M	0x1f
#define MDIO_CMD_DEVAD_S	0
#define MDIO_CMD_PRTAD_M	0x1f
#define MDIO_CMD_PRTAD_S	5
#define MDIO_CMD_OP_M		0x3
#define MDIO_CMD_OP_S		10
#define MDIO_CMD_ST_M		0x3
#define MDIO_CMD_ST_S		12
#define MDIO_CMD_START_B	14

#define MDIO_ADDR_DATA_M	0xffff
#define MDIO_ADDR_DATA_S	0

#define MDIO_WDATA_DATA_M	0xffff
#define MDIO_WDATA_DATA_S	0

#define MDIO_RDATA_DATA_M	0xffff
#define MDIO_RDATA_DATA_S	0

#define MDIO_STATE_STA_B	0

enum mdio_st_clause {
	MDIO_ST_CLAUSE_45 = 0,
	MDIO_ST_CLAUSE_22
};

enum mdio_c22_op_seq {
	MDIO_C22_WRITE = 1,
	MDIO_C22_READ = 2
};

enum mdio_c45_op_seq {
	MDIO_C45_WRITE_ADDR = 0,
	MDIO_C45_WRITE_DATA,
	MDIO_C45_READ_INCREMENT,
	MDIO_C45_READ
};

static void mdio_write_reg(void *base, u32 reg, u32 value)
{
	u8 __iomem *reg_addr = (u8 __iomem *)base;

	writel_relaxed(value, reg_addr + reg);
}

#define MDIO_WRITE_REG(a, reg, value) \
	mdio_write_reg((a)->vbase, (reg), (value))

static u32 mdio_read_reg(void *base, u32 reg)
{
	u8 __iomem *reg_addr = (u8 __iomem *)base;

	return readl_relaxed(reg_addr + reg);
}

#else
#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/of.h>
#include <linux/phy.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/crc32.h>
#include <linux/if_vlan.h>
#include <linux/of_net.h>
#include <net/ip.h>
#include <net/ncsi.h>

#include "ftgmac100.h"

//G6 MDC/MDIO 
#define ASPEED_G6_PHYCR_FIRE		BIT(31)
#define ASPEED_G6_PHYCR_ST_22		BIT(28)
#define ASPEED_G6_PHYCR_WRITE		BIT(26)
#define ASPEED_G6_PHYCR_READ		BIT(27)
#define ASPEED_G6_PHYCR_WDATA(x)	(x & 0xffff)
#define ASPEED_G6_PHYCR_PHYAD(x)	(((x) & 0x1f) << 21)
#define ASPEED_G6_PHYCR_REGAD(x)	(((x) & 0x1f) << 16)


//G4/G5 New MDC/MDIO 
#define ASPEED_G5_PHYCR_FIRE		BIT(15)
#define ASPEED_G5_PHYCR_BUSY		BIT(15)
#define ASPEED_G5_PHYCR_ST_22		BIT(12)
#define ASPEED_G5_PHYCR_WRITE		BIT(10)
#define ASPEED_G5_PHYCR_READ		BIT(11)
#define ASPEED_G5_PHYCR_WDATA(x)	((x & 0xffff) << 16)
#define ASPEED_G5_PHYCR_PHYAD(x)	(((x) & 0x1f) << 5)
#define ASPEED_G5_PHYCR_REGAD(x)	((x) & 0x1f)

#define ASPEED_PHYDATA_MIIWDATA(x)		((x) & 0xffff)

static int aspeed_g6_mdiobus_read(struct mii_bus *bus, int phy_addr, int regnum)
{
	struct aspeed_mdio_regs *regs = bus->priv;
	unsigned int phycr;
	int i;

	//Use New MDC and MDIO interface
	phycr = ASPEED_G6_PHYCR_FIRE | ASPEED_G6_PHYCR_ST_22 | ASPEED_G6_PHYCR_READ |
			ASPEED_G6_PHYCR_PHYAD(phy_addr) | // 20141114
			ASPEED_G6_PHYCR_REGAD(regnum); // 20141114
	
	iowrite32(phycr, &regs->phycr);
	
	for (i = 0; i < 10; i++) {
		phycr = ioread32(&regs->phycr);
	
		if ((phycr & ASPEED_G6_PHYCR_FIRE) == 0) {
			int data;
	
			data = ioread32(&regs->phydata);
			return ASPEED_PHYDATA_MIIWDATA(data);
		}
	
		mdelay(10);
	}

	dev_err(&bus->dev, "mdio g6 read timed out\n");
	return -EIO;

}

static int aspeed_g5_mdiobus_read(struct mii_bus *bus, int phy_addr, int regnum)
{
	struct aspeed_mdio_regs *regs = bus->priv;
	int i;

	//Use New MDC and MDIO interface
	u32 phycr = ASPEED_G5_PHYCR_FIRE | ASPEED_G5_PHYCR_ST_22 | ASPEED_G5_PHYCR_READ |
			ASPEED_G5_PHYCR_PHYAD(phy_addr) | // 20141114
			ASPEED_G5_PHYCR_REGAD(regnum); // 20141114
	
	iowrite32(phycr, &regs->phycr);
	
	for (i = 0; i < 10; i++) {
		phycr = ioread32(&regs->phycr);
	
		if ((phycr & ASPEED_G5_PHYCR_FIRE) == 0) {
			int data;
	
			data = ioread32(&regs->phydata);
			return ASPEED_PHYDATA_MIIWDATA(data);
		}
	
		mdelay(10);
	}

	dev_err(&bus->dev, "mdio g5 read timed out\n");
	return -EIO;

}

static int ftgmac100_mdiobus_read(struct mii_bus *bus, int phy_addr, int regnum)
{
	struct aspeed_mdio_regs *regs = bus->priv;
	unsigned int phycr;
	int i;

	phycr = ioread32(&regs->phycr);

	/* preserve MDC cycle threshold */
	phycr &= FTGMAC100_PHYCR_MDC_CYCTHR_MASK;

	phycr |= FTGMAC100_PHYCR_PHYAD(phy_addr) |
		 FTGMAC100_PHYCR_REGAD(regnum) |
		 FTGMAC100_PHYCR_MIIRD;

	iowrite32(phycr, &regs->phycr);

	for (i = 0; i < 10; i++) {
		phycr = ioread32(&regs->phycr);

		if ((phycr & FTGMAC100_PHYCR_MIIRD) == 0) {
			int data;

			data = ioread32(&regs->phydata);
			return FTGMAC100_PHYDATA_MIIRDATA(data);
		}

		udelay(100);
	}

	dev_err(&bus->dev, "mdio read timed out\n");
	return -EIO;
}

static int aspeed_g6_mdiobus_write(struct mii_bus *bus, int phy_addr,
				   int regnum, u16 value)
{
	struct aspeed_mdio_regs *regs = bus->priv;
	unsigned int phycr;
	int i;

	phycr = ASPEED_PHYDATA_MIIWDATA(value) |
			ASPEED_G6_PHYCR_FIRE | ASPEED_G6_PHYCR_ST_22 |
			ASPEED_G6_PHYCR_WRITE |
			ASPEED_G6_PHYCR_PHYAD(phy_addr) |
			ASPEED_G6_PHYCR_REGAD(regnum);
	
	iowrite32(phycr, &regs->phycr);
	
	for (i = 0; i < 10; i++) {
		phycr = ioread32(&regs->phycr);
	
		if ((phycr & ASPEED_G6_PHYCR_FIRE) == 0)
			return 0;
	
		mdelay(100);
	}

	dev_err(&bus->dev, "mdio g6 write timed out\n");
	return -EIO;

}

static int aspeed_g5_mdiobus_write(struct mii_bus *bus, int phy_addr,
				   int regnum, u16 value)
{
	struct aspeed_mdio_regs *regs = bus->priv;
	unsigned int phycr;
	int i;

	phycr = ASPEED_PHYDATA_MIIWDATA(value) |
			ASPEED_G5_PHYCR_FIRE | ASPEED_G5_PHYCR_ST_22 |
			ASPEED_G5_PHYCR_WRITE |
			ASPEED_G5_PHYCR_PHYAD(phy_addr) |
			ASPEED_G5_PHYCR_REGAD(regnum);
	
	iowrite32(phycr, &regs->phycr);
	
	for (i = 0; i < 10; i++) {
		phycr = ioread32(&regs->phycr);
	
		if ((phycr & ASPEED_G5_PHYCR_FIRE) == 0)
			return 0;
	
		mdelay(100);
	}

	dev_err(&bus->dev, "mdio g5 write timed out\n");
	return -EIO;

}
				   
static int ftgmac100_mdiobus_write(struct mii_bus *bus, int phy_addr,
				   int regnum, u16 value)
{
	struct aspeed_mdio_regs *regs = bus->priv;
	unsigned int phycr;
	int data;
	int i;

	phycr = ioread32(&regs->phycr);

	/* preserve MDC cycle threshold */
	phycr &= FTGMAC100_PHYCR_MDC_CYCTHR_MASK;

	phycr |= FTGMAC100_PHYCR_PHYAD(phy_addr) |
		 FTGMAC100_PHYCR_REGAD(regnum) |
		 FTGMAC100_PHYCR_MIIWR;

	data = FTGMAC100_PHYDATA_MIIWDATA(value);

	iowrite32(data, &regs->phydata);
	iowrite32(phycr, &regs->phycr);

	for (i = 0; i < 10; i++) {
		phycr = ioread32(&regs->phycr);

		if ((phycr & FTGMAC100_PHYCR_MIIWR) == 0)
			return 0;

		udelay(100);
	}

	dev_err(&bus->dev, "mdio write timed out\n");
	return -EIO;
}


extern int ftgmac100_setup_mdio(struct net_device *netdev)
{
	struct ftgmac100 *priv = netdev_priv(netdev);
	struct platform_device *pdev = to_platform_device(priv->dev);
	int i, err = 0;
	
	/* initialize mdio bus */
	priv->mii_bus = mdiobus_alloc();
	if (!priv->mii_bus)
		return -EIO;

	priv->mii_bus->name = "ftgmac100_mdio";
	snprintf(priv->mii_bus->id, MII_BUS_ID_SIZE, "%s-%d",
		 pdev->name, pdev->id);
	priv->mii_bus->parent = priv->dev;
	priv->mii_bus->priv = priv->mdio_base;//priv->netdev;
	if(priv->aspeed_version == 6) {
		priv->mii_bus->read = aspeed_g6_mdiobus_read;
		priv->mii_bus->write = aspeed_g6_mdiobus_write;
	} else if (priv->aspeed_version == 0) {
		priv->mii_bus->read = ftgmac100_mdiobus_read;
		priv->mii_bus->write = ftgmac100_mdiobus_write;		
	} else {
		priv->mii_bus->read = aspeed_g5_mdiobus_read;
		priv->mii_bus->write = aspeed_g5_mdiobus_write;		
	}
	for (i = 0; i < PHY_MAX_ADDR; i++)
		priv->mii_bus->irq[i] = PHY_POLL;

	err = mdiobus_register(priv->mii_bus);
//	err = of_mdiobus_register(priv->mii_bus, pdev->dev.of_node);
	if (err) {
		dev_err(priv->dev, "Cannot register MDIO bus!\n");
		goto err_register_mdiobus;
	}
	return 0;

err_mii_probe:
	mdiobus_unregister(priv->mii_bus);
err_register_mdiobus:
	mdiobus_free(priv->mii_bus);
	return err;
}

#if 0
/**
 * aspeed_mdio_probe - probe mdio device
 * @pdev: mdio platform device
 *
 * Return 0 on success, negative on failure
 */
static int aspeed_mdio_probe(struct platform_device *pdev)
{
	struct aspeed_mdio_device *mdio_dev;
	struct mii_bus *new_bus;
	struct resource *res;
	int ret = -ENODEV;
	int i;

	printk("aspeed_mdio_probe 0-----------------------------------------------\n");
	if (!pdev) {
		dev_err(NULL, "pdev is NULL!\r\n");
		return -ENODEV;
	}

	mdio_dev = devm_kzalloc(&pdev->dev, sizeof(*mdio_dev), GFP_KERNEL);
	if (!mdio_dev)
		return -ENOMEM;

	new_bus = devm_mdiobus_alloc(&pdev->dev);
	if (!new_bus) {
		dev_err(&pdev->dev, "mdiobus_alloc fail!\n");
		return -ENOMEM;
	}

	new_bus->name = "Aspeed MII Bus";
	new_bus->read = aspeed_g5_mdiobus_read;
	new_bus->write = aspeed_g5_mdiobus_write;
	snprintf(new_bus->id, MII_BUS_ID_SIZE, "%s-%s", "Mii",
		 dev_name(&pdev->dev));
	
//	new_bus->priv = mdio_dev;
	new_bus->parent = &pdev->dev;

	for (i = 0; i < PHY_MAX_ADDR; i++)
		new_bus->irq[i] = PHY_POLL;
	

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	printk("mdio reg start %x \n", res->start);
	mdio_dev->regs = devm_ioremap(&pdev->dev, res->start, resource_size(res));
	if (IS_ERR(mdio_dev->regs)) {
		ret = PTR_ERR(mdio_dev->regs);
		return ret;
	}

	printk("aspeed_mdio_probe map reg  %x \n", mdio_dev->regs);

new_bus->priv = mdio_dev->regs;	
	ret = of_mdiobus_register(new_bus, pdev->dev.of_node);
	if (ret) {
		dev_err(&pdev->dev, "Cannot register as MDIO bus!\n");
		platform_set_drvdata(pdev, NULL);
		return ret;
	}
	
	platform_set_drvdata(pdev, new_bus);

	return 0;
}

/**
 * aspeed_mdio_remove - remove mdio device
 * @pdev: mdio platform device
 *
 * Return 0 on success, negative on failure
 */
static int aspeed_mdio_remove(struct platform_device *pdev)
{
	struct mii_bus *bus;

	bus = platform_get_drvdata(pdev);

	mdiobus_unregister(bus);
	platform_set_drvdata(pdev, NULL);
	return 0;
}

static const struct of_device_id aspeed_mdio_match[] = {
	{.compatible = "aspeed,ast2400-mdio"},
	{.compatible = "aspeed,ast2500-mdio"},
	{.compatible = "aspeed,ast2600-mdio"},
	{}
};
MODULE_DEVICE_TABLE(of, aspeed_mdio_match);

static struct platform_driver aspeed_mdio_driver = {
	.probe = aspeed_mdio_probe,
	.remove = aspeed_mdio_remove,
	.driver = {
		   .name = KBUILD_MODNAME,
		   .of_match_table = aspeed_mdio_match,
		   },
};

module_platform_driver(aspeed_mdio_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("ASPEED Technology Inc.");
MODULE_DESCRIPTION("Aspeed MDIO driver");
#endif
#endif
