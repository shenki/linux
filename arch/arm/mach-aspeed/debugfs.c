/*
 * Copyright 2016 IBM Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */

#include <linux/init.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/debugfs.h>
#include <linux/slab.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#define AST_IO_VA	0xf0000000
#define AST_IO_PA	0x1e600000
#define AST_IO_SZ	0x00200000

#define AST_IO(__pa)	((void __iomem *)(((__pa) & 0x001fffff) | AST_IO_VA))

struct aspeed_controller {
	const char *name;
	unsigned int base;
	unsigned int nregs;
};

static int aspeed_show(struct seq_file *s, void *unused)
{
	struct aspeed_controller *ctrl = s->private;
	int i;

	for (i = 0; i < ctrl->nregs; i += 4)
		seq_printf(s, "0x%08X: 0x%08X\n", ctrl->base | i,
			   readl(AST_IO(ctrl->base | i)));

	return 0;
}

static ssize_t aspeed_write(struct file *filp, const char __user *ubuf,
			       size_t count, loff_t *ppos)
{
	struct aspeed_controller *ctrl = filp->private_data;
	unsigned int reg, val;
	char buf[50];
	int ret;

	ret = simple_write_to_buffer(buf, sizeof(buf), ppos, ubuf, count);
	if (!ret)
		return -EFAULT;

	ret = sscanf(buf, "%x=%x", &reg, &val);
	if (ret != 2)
		return -EINVAL;

	if (reg >= ctrl->nregs)
		return -EINVAL;

	writel(val, AST_IO(ctrl->base | reg));
	return count;
}

static int aspeed_open(struct inode *inode, struct file *file)
{
	return single_open(file, aspeed_show, inode->i_private);
}

static const struct file_operations aspeed_fops = {
	.open		= aspeed_open,
	.read		= seq_read,
	.write		= aspeed_write,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static struct dentry *aspeed_debugfs_root;

static int aspeed_device_show(struct device *dev, void *data)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct aspeed_controller *ctrl;
	struct resource *r;

	if (!pdev)
		return 0;

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!r)
		return 0;

	ctrl = kzalloc(sizeof(*ctrl), GFP_KERNEL);
	if (!ctrl)
		return -ENOMEM;

	ctrl->name = pdev->name;
	ctrl->base = r->start;
	ctrl->nregs = resource_size(r);

	debugfs_create_file(pdev->name, 0600, aspeed_debugfs_root,
			    ctrl, &aspeed_fops);
	return 0;
}

static int __init aspeed_debugfs_init(void)
{
	aspeed_debugfs_root = debugfs_create_dir("aspeed", NULL);

	bus_for_each_dev(&platform_bus_type, NULL, NULL, aspeed_device_show);

	return 0;
}

arch_initcall(aspeed_debugfs_init);
