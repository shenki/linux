// SPDX-License-Identifier: GPL-2.0+
// Copyright (C) IBM Corporation 2018
// FSI master driver for AST2600

#include <linux/delay.h>
#include <linux/fsi.h>
#include <linux/io.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/slab.h>

#include "fsi-master.h"

/* Control Registers */
#define FSI_MMODE		0x0		/* R/W: mode */
#define FSI_MDLYR		0x4		/* R/W: delay */
#define FSI_MCRSP		0x8		/* R/W: clock rate */
#define FSI_MENP0		0x10		/* R/W: enable */
#define FSI_MLEVP0		0x18		/* R: plug detect */
#define FSI_MSENP0		0x18		/* S: Set enable */
#define FSI_MCENP0		0x20		/* C: Clear enable */
#define FSI_MAEB		0x70		/* R: Error address */
#define FSI_MVER		0x74		/* R: master version/type */
#define FSI_MRESP0		0xd0		/* W: Port reset */
#define FSI_MESRB0		0x1d0		/* R: Master error status */
#define FSI_MRESB0		0x1d0		/* W: Reset bridge */
#define FSI_MECTRL		0x2e0		/* W: Error control */

/* MMODE: Mode control */
#define FSI_MMODE_EIP		0x80000000	/* Enable interrupt polling */
#define FSI_MMODE_ECRC		0x40000000	/* Enable error recovery */
#define FSI_MMODE_EPC		0x10000000	/* Enable parity checking */
#define FSI_MMODE_P8_TO_LSB	0x00000010	/* Timeout value LSB */
						/*   MSB=1, LSB=0 is 0.8 ms */
						/*   MSB=0, LSB=1 is 0.9 ms */
#define FSI_MMODE_CRS0SHFT	18		/* Clk rate selection 0 shift */
#define FSI_MMODE_CRS0MASK	0x3ff		/* Clk rate selection 0 mask */
#define FSI_MMODE_CRS1SHFT	8		/* Clk rate selection 1 shift */
#define FSI_MMODE_CRS1MASK	0x3ff		/* Clk rate selection 1 mask */

/* MRESB: Reset brindge */
#define FSI_MRESB_RST_GEN	0x80000000	/* General reset */
#define FSI_MRESB_RST_ERR	0x40000000	/* Error Reset */

/* MRESB: Reset port */
#define FSI_MRESP_RST_ALL_MASTER 0x20000000	/* Reset all FSI masters */
#define FSI_MRESP_RST_ALL_LINK	0x10000000	/* Reset all FSI port contr. */
#define FSI_MRESP_RST_MCR	0x08000000	/* Reset FSI master reg. */
#define FSI_MRESP_RST_PYE	0x04000000	/* Reset FSI parity error */
#define FSI_MRESP_RST_ALL	0xfc000000	/* Reset any error */

/* MECTRL: Error control */
#define FSI_MECTRL_EOAE		0x8000		/* Enable machine check when */
						/* master 0 in error */
#define FSI_MECTRL_P8_AUTO_TERM	0x4000		/* Auto terminate */

#define FSI_ENGID_HUB_MASTER		0x1c
#define FSI_HUB_LINK_OFFSET		0x80000
#define FSI_HUB_LINK_SIZE		0x80000
#define FSI_HUB_MASTER_MAX_LINKS	8

#define FSI_LINK_ENABLE_SETUP_TIME	10	/* in mS */

/*
 * FSI hub master support
 *
 * A hub master increases the number of potential target devices that the
 * primary FSI master can access. For each link a primary master supports,
 * each of those links can in turn be chained to a hub master with multiple
 * links of its own.
 *
 * The hub is controlled by a set of control registers exposed as a regular fsi
 * device (the hub->upstream device), and provides access to the downstream FSI
 * bus as through an address range on the slave itself (->addr and ->size).
 *
 * [This differs from "cascaded" masters, which expose the entire downstream
 * bus entirely through the fsi device address range, and so have a smaller
 * accessible address space.]
 */
struct fsi_master_aspeed {
	struct fsi_master	master;
	struct device		*dev;
	void __iomem		*base;
};

#define to_fsi_master_aspeed(m) \
	container_of(m, struct fsi_master_aspeed, master)

/* Control register (size 0x400) */
static const u32 ctrl_base = 0x80000000;

static const u32 fsi_base = 0xa0000000;

#define OPB_FSI_VER	0x00
#define OPB_TRIGGER	0x04
#define OPB_CTRL_BASE	0x08
#define OPB_FSI_BASE	0x0c
#define OPB_CLK_SYNC	0x3c
#define OPB_IRQ_CLEAR	0x40
#define OPB_IRQ_MASK	0x44

#define OPB0_SELECT	0x10
#define OPB0_RW		0x14
#define OPB0_XFER_SIZE	0x18
#define OPB0_FSI_ADDR	0x1c
#define OPB0_FSI_DATA_W	0x20
#define OPB0_FSI_DATA_R	0x84

/* OPB_IRQ_MASK */
#define OPB1_XFER_ACK_EN BIT(1)
#define OPB0_XFER_ACK_EN BIT(0)

/* OPB_RW */
#define CMD_READ	BIT(0)
#define CMD_WRITE	0

/* OPBx_XFER_SIZE */
#define XFER_WORD	(BIT(1) | BIT(0))
#define XFER_NIBBLE	(BIT(0))
#define XFER_BYTE	(0)

#define CREATE_TRACE_POINTS
#include <trace/events/fsi_master_aspeed.h>

static void opb_write(void __iomem *base, uint32_t addr, uint32_t val,
		size_t size)
{
	trace_fsi_master_aspeed_opb_write(base, addr, val, size);

	/* TODO: implement other sizes, see 0x18 */
	BUG_ON(size != 4);

	writel(0x1, base + OPB0_SELECT);
	writel(CMD_WRITE, base + OPB0_RW);
	writel(XFER_WORD, base + OPB0_XFER_SIZE);
	writel(addr, base + OPB0_FSI_ADDR);
	writel(val, base + OPB0_FSI_DATA_W);
	writel(0x1, base + OPB_IRQ_CLEAR);
	writel(0x1, base + OPB_TRIGGER);
}

static u32 opb_read(void __iomem *base, uint32_t addr, size_t size)
{
	uint32_t result;

	/* TODO: implement other sizes, see 0x18 */
	BUG_ON(size != 4);

	writel(0x1, base + OPB0_SELECT);
	writel(CMD_READ, base + OPB0_RW);
	writel(XFER_WORD, base + OPB0_XFER_SIZE);
	writel(addr, base + OPB0_FSI_ADDR);
	writel(0x1, base + OPB_IRQ_CLEAR);
	writel(0x1, base + OPB_TRIGGER);
	result = readl(base + OPB0_FSI_DATA_R);

	trace_fsi_master_aspeed_opb_read(base, addr, size, result);

	return result;
}

static int aspeed_master_read(struct fsi_master *master, int link,
			uint8_t id, uint32_t addr, void *val, size_t size)
{
	struct fsi_master_aspeed *aspeed = to_fsi_master_aspeed(master);
	uint32_t data;

	if (id != 0)
		return -EINVAL;

	addr += link * FSI_HUB_LINK_SIZE;
	data = opb_read(aspeed->base, fsi_base + addr, size);
	memcpy(val, &data, size);

	return 0;
}

static int aspeed_master_write(struct fsi_master *master, int link,
			uint8_t id, uint32_t addr, const void *val, size_t size)
{
	struct fsi_master_aspeed *aspeed = to_fsi_master_aspeed(master);

	if (id != 0)
		return -EINVAL;

	addr += link * FSI_HUB_LINK_SIZE;
	opb_write(aspeed->base, fsi_base + addr, *(uint32_t *)val, size);

	return 0;
}

static int aspeed_master_break(struct fsi_master *master, int link)
{
	uint32_t addr;
	__be32 cmd;

	addr = 0x4;
	cmd = cpu_to_le32(0xc0de0000);

	return aspeed_master_write(master, link, 0, addr, &cmd, sizeof(cmd));
}

static int aspeed_master_link_enable(struct fsi_master *master, int link)
{
	struct fsi_master_aspeed *aspeed = to_fsi_master_aspeed(master);
	int idx, bit;
	__be32 reg, result;

	idx = link / 32;
	bit = link % 32;

	reg = cpu_to_le32(0x80000000 >> bit);

	opb_write(aspeed->base, ctrl_base + FSI_MSENP0 + (4 * idx), reg,
			sizeof(reg));

	mdelay(FSI_LINK_ENABLE_SETUP_TIME);

	result = opb_read(aspeed->base, ctrl_base + FSI_MENP0 + (4 * idx),
			sizeof(reg));

	if (result != reg) {
		dev_err(aspeed->dev, "%s failed: %08x\n", __func__, result);
		return -EIO;
	}

	return 0;
}

static void aspeed_master_release(struct device *dev)
{
	struct fsi_master_aspeed *aspeed = to_fsi_master_aspeed(dev_to_fsi_master(dev));

	kfree(aspeed);
}

/* mmode encoders */
static inline u32 fsi_mmode_crs0(u32 x)
{
	return (x & FSI_MMODE_CRS0MASK) << FSI_MMODE_CRS0SHFT;
}

static inline u32 fsi_mmode_crs1(u32 x)
{
	return (x & FSI_MMODE_CRS1MASK) << FSI_MMODE_CRS1SHFT;
}

static int aspeed_master_init(struct fsi_master_aspeed *aspeed)
{
	__be32 reg;

	reg = cpu_to_le32(FSI_MRESP_RST_ALL_MASTER | FSI_MRESP_RST_ALL_LINK
			| FSI_MRESP_RST_MCR | FSI_MRESP_RST_PYE);
	opb_write(aspeed->base, ctrl_base + FSI_MRESP0, reg, sizeof(reg));

	/* Initialize the MFSI (hub master) engine */
	reg = cpu_to_le32(FSI_MRESP_RST_ALL_MASTER | FSI_MRESP_RST_ALL_LINK
			| FSI_MRESP_RST_MCR | FSI_MRESP_RST_PYE);
	opb_write(aspeed->base, ctrl_base + FSI_MRESP0, reg, sizeof(reg));

	reg = cpu_to_le32(FSI_MECTRL_EOAE | FSI_MECTRL_P8_AUTO_TERM);
	opb_write(aspeed->base, ctrl_base + FSI_MECTRL, reg, sizeof(reg));

	reg = cpu_to_le32(FSI_MMODE_EIP | FSI_MMODE_ECRC | FSI_MMODE_EPC
			| fsi_mmode_crs0(1) | fsi_mmode_crs1(1)
			| FSI_MMODE_P8_TO_LSB);
	opb_write(aspeed->base, ctrl_base + FSI_MMODE, reg, sizeof(reg));

	reg = cpu_to_le32(0xffff0000);
	opb_write(aspeed->base, ctrl_base + FSI_MDLYR, reg, sizeof(reg));

	reg = cpu_to_le32(~0);
	opb_write(aspeed->base, ctrl_base + FSI_MSENP0, reg, sizeof(reg));

	/* Leave enabled long enough for master logic to set up */
	mdelay(FSI_LINK_ENABLE_SETUP_TIME);

	opb_write(aspeed->base, ctrl_base + FSI_MCENP0, reg, sizeof(reg));

	opb_read(aspeed->base, ctrl_base + FSI_MAEB, sizeof(reg));

	reg = cpu_to_le32(FSI_MRESP_RST_ALL_MASTER | FSI_MRESP_RST_ALL_LINK);
	opb_write(aspeed->base, ctrl_base + FSI_MRESP0, reg, sizeof(reg));

	opb_read(aspeed->base, ctrl_base + FSI_MLEVP0, sizeof(reg));

	/* Reset the master bridge */
	reg = cpu_to_le32(FSI_MRESB_RST_GEN);
	opb_write(aspeed->base, ctrl_base + FSI_MRESB0, reg, sizeof(reg));

	reg = cpu_to_le32(FSI_MRESB_RST_ERR);
	opb_write(aspeed->base, ctrl_base + FSI_MRESB0, reg, sizeof(reg));

	return 0;
}

static int fsi_master_aspeed_probe(struct platform_device *pdev)
{
	struct fsi_master_aspeed *aspeed;
	struct resource *res;
	struct regmap *scu;
	int rc, links, reg;
	__be32 __reg;

	aspeed = devm_kzalloc(&pdev->dev, sizeof(*aspeed), GFP_KERNEL);
	if (!aspeed)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	aspeed->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(aspeed->base))
		return PTR_ERR(aspeed->base);

	aspeed->dev = &pdev->dev;

	/* SCU enable */
	scu = syscon_regmap_lookup_by_compatible("aspeed,ast2600-scu");
	if (IS_ERR(scu))
		return PTR_ERR(scu);
	regmap_write(scu, 0x94, 0xffffffff); // SCU     - clk enable
	regmap_write(scu, 0x54, 0xffffffff); // SCU[27] - reset

	writel(0x1, aspeed->base + OPB_CLK_SYNC);
	writel(OPB1_XFER_ACK_EN | OPB0_XFER_ACK_EN,
			aspeed->base + OPB_IRQ_MASK);
	writel(0x10, aspeed->base + 0x64); // Retry counter number ???
	writel(0x0f, aspeed->base + 0xe4); // DMA Enable

	writel(ctrl_base, aspeed->base + OPB_CTRL_BASE);
	writel(fsi_base, aspeed->base + OPB_FSI_BASE);

	__reg = opb_read(aspeed->base, ctrl_base + FSI_MVER, sizeof(__reg));

	reg = le32_to_cpu(__reg);
	links = (reg >> 8) & 0xff;
	dev_info(&pdev->dev, "hub version %08x (%d links)\n", reg, links);

	aspeed->master.dev.parent = &pdev->dev;
	aspeed->master.dev.release = aspeed_master_release;
	aspeed->master.dev.of_node = of_node_get(dev_of_node(&pdev->dev));

	aspeed->master.n_links = links;
	aspeed->master.read = aspeed_master_read;
	aspeed->master.write = aspeed_master_write;
	aspeed->master.send_break = aspeed_master_break;
	aspeed->master.link_enable = aspeed_master_link_enable;

	dev_set_drvdata(&pdev->dev, aspeed);

	aspeed_master_init(aspeed);

	rc = fsi_master_register(&aspeed->master);
	if (rc)
		goto err_release;

	/* At this point, fsi_master_register performs the device_initialize(),
	 * and holds the sole reference on master.dev. This means the device
	 * will be freed (via ->release) during any subsequent call to
	 * fsi_master_unregister.  We add our own reference to it here, so we
	 * can perform cleanup (in _remove()) without it being freed before
	 * we're ready.
	 */
	get_device(&aspeed->master.dev);
	return 0;

err_release:
	// TODO(joel): Should we be claiming a range?
//	fsi_slave_release_range(fsi_dev->slave, FSI_HUB_LINK_OFFSET,
//			FSI_HUB_LINK_SIZE * links);
	return rc;
}

static int fsi_master_aspeed_remove(struct platform_device *pdev)
{
	struct fsi_master_aspeed *master = platform_get_drvdata(pdev);

	fsi_master_unregister(&master->master);

	return 0;
}

static const struct of_device_id fsi_master_aspeed_match[] = {
	{ .compatible = "aspeed,fsi-master-ast2600" },
	{ },
};

static struct platform_driver fsi_master_aspeed_driver = {
	.driver = {
		.name		= "fsi-master-aspeed",
		.of_match_table	= fsi_master_aspeed_match,
	},
	.probe	= fsi_master_aspeed_probe,
	.remove = fsi_master_aspeed_remove,
};

module_platform_driver(fsi_master_aspeed_driver);
MODULE_LICENSE("GPL");
