/*
 * Fake FSI master driver for FSI development
 *
 * Copyright (C) IBM Corporation 2016
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>

#include "fsi-master.h"

const uint8_t data[] = {
	0xc0, 0x02, 0x08, 0x03,	/* chip id */
	0x80, 0x01, 0x11, 0x00,	/* peek */
	0x80, 0x01, 0x20, 0x3e,	/* slave */
	0x00, 0x01, 0x10, 0xa5,	/* i2c */
};


static int fsi_master_fake_read(struct fsi_master *_master, int link,
		uint8_t slave, uint32_t addr, void *val, size_t size)
{
	if (link != 0)
		return -ENODEV;

	if (addr + size > sizeof(data))
		memset(val, 0, size);
	else
		memcpy(val, data + addr, size);

	return 0;
}

static int fsi_master_fake_write(struct fsi_master *_master, int link,
		uint8_t slave, uint32_t addr, const void *val, size_t size)
{
	if (link != 0)
		return -ENODEV;

	return -EACCES;
}

static int fsi_master_fake_probe(struct platform_device *pdev)
{
	struct fsi_master *master;

	master = devm_kzalloc(&pdev->dev, sizeof(*master), GFP_KERNEL);
	if (!master)
		return -ENOMEM;

	master->dev = &pdev->dev;
	master->n_links = 1;
	master->read = fsi_master_fake_read;
	master->write = fsi_master_fake_write;

	return fsi_master_register(master);
}

static const struct of_device_id fsi_master_fake_match[] = {
	{ .compatible = "ibm,fsi-master-fake" },
	{ },
};

static struct platform_driver fsi_master_fake_driver = {
	.driver = {
		.name		= "fsi-master-fake",
		.of_match_table	= fsi_master_fake_match,
	},
	.probe	= fsi_master_fake_probe,
};

static int __init fsi_master_fake_init(void)
{
	struct device_node *np;

	platform_driver_register(&fsi_master_fake_driver);

	for_each_compatible_node(np, NULL, "ibm,fsi-master-fake")
		of_platform_device_create(np, NULL, NULL);

	return 0;
}

module_init(fsi_master_fake_init);
