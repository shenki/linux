/*
 * Copyright 2016 IBM Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */

#include <linux/mm.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/errno.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/spinlock.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/miscdevice.h>
#include <linux/mtd/mtd.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>
#include <linux/lpc-ctrl.h>

#define DEVICE_NAME	"lpc-ctrl"

#define HICR7 0x8
#define HICR8 0xc

struct lpc_ctrl {
	struct miscdevice	miscdev;
	struct regmap		*regmap;
	void __iomem		*ctrl;
	phys_addr_t		base;
	resource_size_t		size;
	uint32_t		pnor_size;
	uint32_t		pnor_base;
};

static atomic_t lpc_ctrl_open_count = ATOMIC_INIT(0);

static struct lpc_ctrl *file_lpc_ctrl(struct file *file)
{
	return container_of(file->private_data, struct lpc_ctrl, miscdev);
}

static int lpc_ctrl_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct lpc_ctrl *lpc_ctrl = file_lpc_ctrl(file);
	unsigned long vsize = vma->vm_end - vma->vm_start;

	if (vma->vm_pgoff + vsize > lpc_ctrl->base + lpc_ctrl->size)
		return -EINVAL;

	/* Other checks? */

	if (remap_pfn_range(vma, vma->vm_start,
		(lpc_ctrl->base >> PAGE_SHIFT) + vma->vm_pgoff,
		vsize, vma->vm_page_prot))
		return -EAGAIN;

	return 0;
}

static int lpc_ctrl_open(struct inode *inode, struct file *file)
{
	if (atomic_inc_return(&lpc_ctrl_open_count) == 1)
		return 0;

	atomic_dec(&lpc_ctrl_open_count);
	return -EBUSY;
}

static ssize_t lpc_ctrl_read(struct file *file, char __user *buf,
				size_t count, loff_t *ppos)
{
	if (!access_ok(VERIFY_WRITE, buf, count))
		return -EFAULT;

	WARN_ON(*ppos);

	return -EPERM;
}

static ssize_t lpc_ctrl_write(struct file *file, const char __user *buf,
				size_t count, loff_t *ppos)
{
	if (!access_ok(VERIFY_READ, buf, count))
		return -EFAULT;

	WARN_ON(*ppos);

	return -EPERM;
}

static long lpc_ctrl_ioctl(struct file *file, unsigned int cmd,
		unsigned long param)
{
	struct lpc_mapping map;
	struct lpc_ctrl *lpc_ctrl = file_lpc_ctrl(file);
	void __user *p = (void __user *)param;

	switch (cmd) {
	case LPC_CTRL_IOCTL_SIZE:
		return copy_to_user(p, &lpc_ctrl->size,
			sizeof(lpc_ctrl->size)) ? -EFAULT : 0;
	case LPC_CTRL_IOCTL_MAP:
		if (copy_from_user(&map, p, sizeof(map)))
			return -EFAULT;


		/*
		 * The top half of HICR7 is the MSB of the BMC address of the
		 * mapping.
		 * The bottom half of HICR7 is the MSB of the HOST LPC
		 * firmware space address of the mapping.
		 *
		 * The 1 bits in the top of half of HICR8 represent the bits
		 * (in the requested address) that should be ignored and
		 * replaced with those from the top half of HICR7.
		 * The 1 bits in the bottom half of HICR8 represent the bits
		 * (in the requested address) that should be kept and pass
		 * into the BMC address space.
		 */

		regmap_write(lpc_ctrl->regmap, HICR7,
				(lpc_ctrl->base | (map.hostaddr >> 16)));
		regmap_write(lpc_ctrl->regmap, HICR8,
			(~(map.size - 1)) | ((map.size >> 16) - 1));
		return 0;
	case LPC_CTRL_IOCTL_UNMAP:
		if (copy_from_user(&map, p, sizeof(map)))
			return -EFAULT;

		/*
		 * The top nibble in host lpc addresses references which
		 * firmware space, use space zero hence the & 0x0fff
		 */
		regmap_write(lpc_ctrl->regmap, HICR7,
			lpc_ctrl->pnor_base | (((-lpc_ctrl->pnor_size) >> 16) & 0x0fff));
		regmap_write(lpc_ctrl->regmap, HICR8,
			(~(lpc_ctrl->pnor_size - 1)) | ((lpc_ctrl->pnor_size >> 16) - 1));
		return 0;
	}

	return -EINVAL;
}

static int lpc_ctrl_release(struct inode *inode, struct file *file)
{
	return 0;
}

static const struct file_operations lpc_ctrl_fops = {
	.owner		= THIS_MODULE,
	.mmap		= lpc_ctrl_mmap,
	.open		= lpc_ctrl_open,
	.read		= lpc_ctrl_read,
	.write		= lpc_ctrl_write,
	.release	= lpc_ctrl_release,
	.unlocked_ioctl	= lpc_ctrl_ioctl,
};

static int lpc_ctrl_probe(struct platform_device *pdev)
{
	struct lpc_ctrl *lpc_ctrl;
	struct device *dev;
	struct device_node *node;
	struct resource *res;
	struct resource resm;
	struct mtd_info *mtd;
	int rc, i;

	if (!pdev || !pdev->dev.of_node)
		return -ENODEV;

	dev = &pdev->dev;

	lpc_ctrl = devm_kzalloc(dev, sizeof(*lpc_ctrl), GFP_KERNEL);
	if (!lpc_ctrl)
		return -ENOMEM;

	dev_set_drvdata(&pdev->dev, lpc_ctrl);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(dev, "Unable to find resources\n");
		rc = -ENXIO;
		goto out;
	}

	node = of_parse_phandle(dev->of_node, "memory-region", 0);
	if (!node) {
		/*
		 * Should probaby handle this by allocating 4-64k now and
		 * using that
		 */
		dev_err(dev, "Didn't find reserved memory\n");
		rc = -EINVAL;
		goto out;
	}

	rc = of_address_to_resource(node, 0, &resm);
	of_node_put(node);
	if (rc) {
		dev_err(dev, "Could address to resource\n");
		rc = -ENOMEM;
		goto out;
	}

	lpc_ctrl->size = resource_size(&resm);
	lpc_ctrl->base = resm.start;

	lpc_ctrl->miscdev.minor = MISC_DYNAMIC_MINOR;
	lpc_ctrl->miscdev.name = DEVICE_NAME;
	lpc_ctrl->miscdev.fops = &lpc_ctrl_fops;
	lpc_ctrl->miscdev.parent = dev;
	rc = misc_register(&lpc_ctrl->miscdev);
	if (rc) {
		dev_err(dev, "Unable to register device\n");
		goto out;
	}

	/* Find the size of the pnor */
	i = 0;
	mtd = get_mtd_device(NULL, i);
	while (!IS_ERR(mtd)) {
		struct device_node *node = mtd_get_of_node(mtd);

		if (mtd->name && strstr(mtd->name, "pnor") && node) {
			lpc_ctrl->pnor_size = mtd->size;
			rc = of_property_read_u32_index(node, "reg", 2,
					&lpc_ctrl->pnor_base);
			if (rc)
				mtd = ERR_PTR(rc);
			else
				dev_info(dev, "Host PNOR base: 0x%08x size: 0x%08x\n",
					lpc_ctrl->pnor_base, lpc_ctrl->pnor_size);
			break;
		}

		mtd = get_mtd_device(NULL, ++i);
	}
	if (IS_ERR(mtd)) {
		dev_err(dev, "Couldn't locate MTD PNOR partition\n");
		rc = -ENODEV;
		goto out;
	}

	lpc_ctrl->regmap = syscon_node_to_regmap(
			pdev->dev.parent->of_node);
	if (IS_ERR(lpc_ctrl->regmap)) {
		dev_err(dev, "Couldn't get regmap\n");
		rc = -ENODEV;
		goto out;
	}

	dev_info(dev, "Loaded at 0x%08x (0x%08x)\n",
			lpc_ctrl->base, lpc_ctrl->size);
	return 0;

out:
	return rc;
}

static int lpc_ctrl_remove(struct platform_device *pdev)
{
	struct lpc_ctrl *lpc_ctrl = dev_get_drvdata(&pdev->dev);

	misc_deregister(&lpc_ctrl->miscdev);
	lpc_ctrl = NULL;

	return 0;
}

static const struct of_device_id lpc_ctrl_match[] = {
	{ .compatible = "aspeed,ast2400-lpc-ctrl" },
	{ },
};

static struct platform_driver lpc_ctrl_driver = {
	.driver = {
		.name		= DEVICE_NAME,
		.of_match_table = lpc_ctrl_match,
	},
	.probe = lpc_ctrl_probe,
	.remove = lpc_ctrl_remove,
};

module_platform_driver(lpc_ctrl_driver);

MODULE_DEVICE_TABLE(of, lpc_ctrl_match);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Cyril Bur <cyrilbur@gmail.com>");
MODULE_DESCRIPTION("Linux device interface to control LPC bus");
