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
	struct device *dev;
	const char *name;
	unsigned int base;
	unsigned int nregs;
};

static ssize_t aspeed_read(struct file *filp, char __user *ubuf,
			   size_t count, loff_t *ppos)
{
	struct aspeed_controller *ctrl = filp->private_data;
	char *kbuf;
	int ret, n = 0;
	size_t size;
	int i;

	size = ctrl->nregs * 23 + 1;
	kbuf = kzalloc(size, GFP_KERNEL);
	if (!kbuf)
		return -ENOMEM;

	for (i = 0; i < ctrl->nregs; i += 4) {
		n += scnprintf(kbuf + n, size - n, "0x%08X: 0x%08X\n",
			       ctrl->base | i, readl(AST_IO(ctrl->base | i)));
	}

	ret = simple_read_from_buffer(ubuf, count, ppos, kbuf, n);

	kfree(kbuf);

	return ret;
}

static ssize_t aspeed_write(struct file *filp, const char __user *ubuf,
			       size_t count, loff_t *ppos)
{
	struct aspeed_controller *ctrl = filp->private_data;
	unsigned int reg, val;
	char *kbuf = kmalloc(count + 1, GFP_KERNEL);
	int ret;

	if (!kbuf)
		return -ENOMEM;

	ret = simple_write_to_buffer(kbuf, count, ppos, ubuf, count);
	if (ret != count) {
		kfree(kbuf);
		return ret >= 0 ? -EIO : ret;
	}
	kbuf[count] = '\0';

	ret = sscanf(kbuf, "%x=%x", &reg, &val);
	kfree(kbuf);

	if (ret != 2)
		return -EINVAL;

	if ((ctrl->base & reg) != ctrl->base) {
		dev_err(ctrl->dev, "wrong base address: %x\n", reg);
		return -EINVAL;
	}

	if ((~ctrl->base & reg) >= ctrl->nregs) {
		dev_err(ctrl->dev, "wrong register: %x\n", reg);
		return -EINVAL;
	}

	writel(val, AST_IO(reg));
	return count;
}

static const struct file_operations aspeed_fops = {
	.open		= simple_open,
	.read		= aspeed_read,
	.write		= aspeed_write,
	.llseek		= generic_file_llseek,
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

	ctrl->dev = dev;
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
