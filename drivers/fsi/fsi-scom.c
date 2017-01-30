/*
 * SCOM FSI Client device driver
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

#define FSI_ENGID_SCOM		0x5

#define SCOM_FSI2PIB_DELAY	50

/* SCOM engine register set */
#define SCOM_DATA0_REG		0x00
#define SCOM_DATA1_REG		0x04
#define SCOM_CMD_REG		0x08
#define SCOM_RESET_REG		0x1C

#define SCOM_RESET_CMD		0x80000000
#define SCOM_WRITE_CMD		0x80000000

struct scom_device {
	struct list_head link;
	struct fsi_device *fsi_dev;
	struct miscdevice mdev;
	char	name[32];
};

#define to_scom_dev(x)		container_of((x), struct scom_device, mdev)

static struct list_head scom_devices;
static atomic_t scom_idx = ATOMIC_INIT(0);

int fsi_scom_write(struct fsi_device *fsi, u64 value, u32 addr)
{
	int rc;
	u32 data = SCOM_RESET_CMD;

	rc = fsi_device_write(fsi, SCOM_RESET_REG, &data, sizeof(u32));
	if (rc)
		return rc;

	data = (value >> 32) & 0xffffffff;
	rc = fsi_device_write(fsi, SCOM_DATA0_REG, &data, sizeof(u32));
	if (rc)
		return rc;

	data = value & 0xffffffff;
	rc = fsi_device_write(fsi, SCOM_DATA1_REG, &data, sizeof(u32));
	if (rc)
		return rc;

	data = SCOM_WRITE_CMD | addr;
	return fsi_device_write(fsi, SCOM_CMD_REG, &data, sizeof(u32));
}
EXPORT_SYMBOL_GPL(fsi_scom_write);

int fsi_scom_read(struct fsi_device *fsi, u64 *value, u32 addr)
{
	u32 result, data;
	int rc;

	data = addr;
	rc = fsi_device_write(fsi, SCOM_CMD_REG, &data, sizeof(u32));
	if (rc)
		return rc;

	rc = fsi_device_read(fsi, SCOM_DATA0_REG, &result, sizeof(u32));
	if (rc)
		return rc;

	*value |= (u64) result << 32;
	rc = fsi_device_read(fsi, SCOM_DATA1_REG, &result, sizeof(u32));
	if (rc)
		return rc;

	*value |= result;

	return 0;
}
EXPORT_SYMBOL_GPL(fsi_scom_read);

static ssize_t scom_read(struct file *filep, char __user *buf, size_t len,
			loff_t *offset)
{
	int rc;
	struct miscdevice *mdev =
				(struct miscdevice *)filep->private_data;
	struct scom_device *scom = to_scom_dev(mdev);
	struct device *dev = &scom->fsi_dev->dev;
	uint64_t val;

	if (len != sizeof(uint64_t))
		return -EINVAL;

	rc = fsi_scom_read(scom->fsi_dev, &val, *offset);
	if (rc) {
		dev_dbg(dev, "%s failed: %d\n", __func__, rc);
		return rc;
	}

	rc = copy_to_user(buf, &val, len);
	if (rc)
		return rc;

	return len;
}

static ssize_t scom_write(struct file *filep, const char __user *buf,
			size_t len, loff_t *offset)
{
	int rc;
	struct miscdevice *mdev =
				(struct miscdevice *)filep->private_data;
	struct scom_device *scom = to_scom_dev(mdev);
	struct device *dev = &scom->fsi_dev->dev;
	uint64_t val;

	if (len != sizeof(uint64_t))
		return -EINVAL;

	rc = copy_from_user(&val, buf, len);
	if (rc)
		return -EINVAL;

	rc = fsi_scom_write(scom->fsi_dev, val, *offset);
	if (rc)
		dev_dbg(dev, "%s failed: %d\n", __func__, rc);

	return rc;
}

static loff_t scom_llseek(struct file *file, loff_t offset, int whence)
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

static const struct file_operations scom_fops = {
	.owner	= THIS_MODULE,
	.llseek	= scom_llseek,
	.read	= scom_read,
	.write	= scom_write,
};

static int scom_probe(struct device *dev)
{
	struct fsi_device *fsi_dev = to_fsi_dev(dev);
	struct scom_device *scom;

	scom = devm_kzalloc(dev, sizeof(*scom), GFP_KERNEL);
	if (!scom)
		return -ENOMEM;

	snprintf(scom->name, sizeof(scom->name),
			"scom%d", atomic_inc_return(&scom_idx));
	scom->fsi_dev = fsi_dev;
	scom->mdev.minor = MISC_DYNAMIC_MINOR;
	scom->mdev.fops = &scom_fops;
	scom->mdev.name = scom->name;
	scom->mdev.parent = dev;
	list_add(&scom->link, &scom_devices);

	return misc_register(&scom->mdev);
}

static struct fsi_device_id scom_ids[] = {
	{
		.engine_type = FSI_ENGID_SCOM,
		.version = FSI_VERSION_ANY,
	},
	{ 0 }
};

static struct fsi_driver scom_drv = {
	.id_table = scom_ids,
	.drv = {
		.name = "scom",
		.bus = &fsi_bus_type,
		.probe = scom_probe,
	}
};

static int scom_init(void)
{
	INIT_LIST_HEAD(&scom_devices);
	return fsi_driver_register(&scom_drv);
}

static void scom_exit(void)
{
	struct list_head *pos;
	struct scom_device *scom;

	list_for_each(pos, &scom_devices) {
		scom = list_entry(pos, struct scom_device, link);
		misc_deregister(&scom->mdev);
		devm_kfree(&scom->fsi_dev->dev, scom);
	}
	fsi_driver_unregister(&scom_drv);
}

module_init(scom_init);
module_exit(scom_exit);
MODULE_LICENSE("GPL");
