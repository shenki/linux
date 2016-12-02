/*
 * Mailbox (MBX) FSI Client device driver
 *
 * Copyright (C) IBM Corporation 2016
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERGCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/fsi.h>
#include <linux/module.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/miscdevice.h>
#include <linux/list.h>

#define FSI_ENGID_P9MBX		0x20
#define MBX_REG_MAX		0x1FC
#define MBX_IOCTL_READ_REG	0x01
#define MBX_IOCTL_WRITE_REG	0x02

struct mbx_device {
	struct list_head link;
	struct fsi_device *fsi_dev;
	struct miscdevice mdev;
	char	name[32];
};

#define to_mbx_dev(x)		container_of((x), struct mbx_device, mdev)

struct mbx_reg_data {
	uint32_t offset;	/* What address */
	uint32_t value;		/* value at address */
};

static struct list_head mbx_devices;
static atomic_t mbx_idx = ATOMIC_INIT(0);
static int mbx_probe(struct device *);

static ssize_t mbx_read(struct file *filep, char __user *buf, size_t len,
			loff_t *offset)
{
	int rc;
	u32 value;
	u32 pos = *offset;

	if (!mbx_offset_valid(pos))
		return -ERANGE;

	rc = fsi_device_read(mbx->fsi_dev, pos, &value, sizeof(value));
	if (rc)
		return -EIO;

	rc = copy_to_user(buf, &value, sizeof(value));
	if (rc)
		return -EFAULT;

	return 0;
}

static ssize_t mbx_write(struct file *filep, const char __user *buf,
			size_t len, loff_t *offset)
{
	int rc = 0;

	return rc ? rc : len;
}

static bool mbx_offset_valid(uint32_t offset)
{
	return (offset >= 0 && offset <= MBX_REG_MAX);
}

static long mbx_ioctl(struct file *filep, uint32_t cmd, unsigned long data)
{
	struct miscdevice *mdev =
				(struct miscdevice *)filep->private_data;
	struct mbx_device *mbx = to_mbx_dev(mdev);
	struct device *dev = &mbx->fsi_dev->dev;
	ssize_t rc;
	struct mbx_reg_data reg;

	switch (cmd) {
	case MBX_IOCTL_WRITE_REG:

		if (!data) {
			dev_info(dev, "Empty ioctl data\n");
			return -EINVAL;
		}
		rc = copy_from_user(&reg, (void *)data, sizeof(reg));
		if (rc) {
			dev_info(dev, "Failed to copy from user\n");
			return -EFAULT;
		}
		if (mbx_offset_valid(reg.offset))
			rc = fsi_device_write(mbx->fsi_dev, reg.offset,
					&reg.value, sizeof(reg.value));
		else
			return -EINVAL;
		break;

	case MBX_IOCTL_READ_REG:

		if (!data) {
			dev_info(dev, "Empty ioctl data\n");
			return -EINVAL;
		}
		rc = copy_from_user(&reg, (void *)data, sizeof(reg));
		if (rc) {
			dev_info(dev, "Failed to copy from user\n");
			return -EFAULT;
		}
		if (mbx_offset_valid(reg.offset)) {
			rc = fsi_device_read(mbx->fsi_dev, reg.offset,
					&reg.value, sizeof(reg.value));
			if (rc)
				return rc;
			rc = copy_to_user((void *)data, &reg, sizeof(reg));
			if (rc) {
				dev_info(dev, "copy to user failed\n");
				return -EFAULT;
			}
		} else
			return -EINVAL;
		break;
	default:
		dev_info(dev, "Ivalid ioctl command:%d\n", cmd);
		return -EINVAL;
	}
	return rc;
};

static loff_t mbx_llseek(struct file *file, loff_t offset, int whence)
{
	switch (whence) {
	case SEEK_CUR:
		break;
	case SEEK_SET:
		file->f_pos = offset;
		break;
	default:
		return -EINVAL;
	}

	return offset;
}

static const struct file_operations mbx_fops = {
	.owner	= THIS_MODULE,
	.llseek	= mbx_llseek,
	.read	= mbx_read,
	.write	= mbx_write,
	.compat_ioctl  = mbx_ioctl,
};

static int mbx_probe(struct device *dev)
{
	struct fsi_device *fsi_dev = to_fsi_dev(dev);
	struct mbx_device *mbx;

	mbx = devm_kzalloc(dev, sizeof(*mbx), GFP_KERNEL);
	if (!mbx)
		return -ENOMEM;

	snprintf(mbx->name, sizeof(mbx->name),
			"mbx%d", atomic_inc_return(&mbx_idx));
	mbx->fsi_dev = fsi_dev;
	mbx->mdev.minor = MISC_DYNAMIC_MINOR;
	mbx->mdev.fops = &mbx_fops;
	mbx->mdev.name = mbx->name;
	mbx->mdev.parent = dev;
	list_add(&mbx->link, &mbx_devices);

	return misc_register(&mbx->mdev);
}

static struct fsi_device_id mbx_ids[] = {
	{
		.engine_type = FSI_ENGID_P9MBX,
		.version = FSI_VERSION_ANY,
	},
	{ 0 }
};

static struct fsi_driver mbx_drv = {
	.id_table = mbx_ids,
	.drv = {
		.name = "mbx",
		.bus = &fsi_bus_type,
		.probe = mbx_probe,
	}
};

static int mbx_init(void)
{
	INIT_LIST_HEAD(&mbx_devices);
	return fsi_driver_register(&mbx_drv);
}

static void mbx_exit(void)
{
	struct list_head *pos;
	struct mbx_device *mbx;

	list_for_each(pos, &mbx_devices) {
		mbx = list_entry(pos, struct mbx_device, link);
		misc_deregister(&mbx->mdev);
		devm_kfree(&mbx->fsi_dev->dev, mbx);
	}
	fsi_driver_unregister(&mbx_drv);
}

module_init(mbx_init);
module_exit(mbx_exit);
MODULE_LICENSE("GPL");
