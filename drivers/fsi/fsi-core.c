/*
 * FSI core driver
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

#include <linux/device.h>
#include <linux/fsi.h>
#include <linux/module.h>
#include <linux/slab.h>

#include "fsi-master.h"

#define FSI_N_SLAVES	4
#define FSI_SLAVE_CONF_CRC_SHIFT        4
#define FSI_SLAVE_CONF_CRC_MASK         0x0000000f
#define FSI_SLAVE_CONF_DATA_BITS        28

static atomic_t master_idx = ATOMIC_INIT(-1);

struct fsi_slave {
	struct device		dev;
	struct fsi_master	*master;
	int			link;
	uint8_t			id;
};

#define to_fsi_slave(d) container_of(d, struct fsi_slave, dev)

/* crc helpers */
static const uint8_t crc4_tab[] = {
	0x0, 0x7, 0xe, 0x9, 0xb, 0xc, 0x5, 0x2,
	0x1, 0x6, 0xf, 0x8, 0xa, 0xd, 0x4, 0x3,
};

uint8_t fsi_crc4(uint8_t c, uint64_t x, int bits)
{
	int i;

	/* Align to 4-bits */
	bits = (bits + 3) & ~0x3;

	/* Calculate crc4 over four-bit nibbles, starting at the MSbit */
	for (i = bits; i >= 0; i -= 4)
		c = crc4_tab[c ^ ((x >> i) & 0xf)];

	return c;
}
EXPORT_SYMBOL_GPL(fsi_crc4);

/* FSI slave support */

static void fsi_slave_release(struct device *dev)
{
	struct fsi_slave *slave = to_fsi_slave(dev);

	kfree(slave);
}

static int fsi_slave_init(struct fsi_master *master,
		int link, uint8_t slave_id)
{
	struct fsi_slave *slave;
	uint32_t chip_id;
	int rc;
	uint8_t crc;

	rc = master->read(master, link, slave_id, 0, &chip_id, sizeof(chip_id));
	if (rc) {
		dev_warn(master->dev, "can't read slave %02x:%02x: %d\n",
				link, slave_id, rc);
		return -ENODEV;
	}
	crc = fsi_crc4(0, chip_id >> FSI_SLAVE_CONF_CRC_SHIFT,
			FSI_SLAVE_CONF_DATA_BITS);
	if (crc != (chip_id & FSI_SLAVE_CONF_CRC_MASK)) {
		dev_warn(master->dev, "slave %02x:%02x invalid chip id CRC!\n",
				link, slave_id);
		return -EIO;
	}

	pr_debug("fsi: found chip %08x at %02x:%02x:%02x\n",
			master->idx, chip_id, link, slave_id);

	/* we can communicate with a slave; create devices and scan */
	slave = kzalloc(sizeof(*slave), GFP_KERNEL);
	if (!slave)
		return -ENOMEM;

	slave->master = master;
	slave->id = slave_id;
	slave->dev.parent = master->dev;
	slave->dev.release = fsi_slave_release;

	dev_set_name(&slave->dev, "slave@%02x:%02x", link, slave_id);
	rc = device_register(&slave->dev);
	if (rc < 0) {
		dev_warn(master->dev, "failed to create slave device: %d\n",
				rc);
		put_device(&slave->dev);
		return rc;
	}

	return rc;
}

/* FSI master support */

static int fsi_master_scan(struct fsi_master *master)
{
	int link, slave_id;

	for (link = 0; link < master->n_links; link++)
		for (slave_id = 0; slave_id < FSI_N_SLAVES; slave_id++)
			fsi_slave_init(master, link, slave_id);

	return 0;

}

int fsi_master_register(struct fsi_master *master)
{
	master->idx = atomic_inc_return(&master_idx);
	get_device(master->dev);
	fsi_master_scan(master);
	return 0;
}
EXPORT_SYMBOL_GPL(fsi_master_register);

void fsi_master_unregister(struct fsi_master *master)
{
	put_device(master->dev);
}
EXPORT_SYMBOL_GPL(fsi_master_unregister);

/* FSI core & Linux bus type definitions */

static int fsi_bus_match(struct device *dev, struct device_driver *drv)
{
	struct fsi_device *fsi_dev = to_fsi_dev(dev);
	struct fsi_driver *fsi_drv = to_fsi_drv(drv);
	const struct fsi_device_id *id;

	if (!fsi_drv->id_table)
		return 0;

	for (id = fsi_drv->id_table; id->engine_type; id++) {
		if (id->engine_type != fsi_dev->engine_type)
			continue;
		if (id->version == FSI_VERSION_ANY ||
				id->version == fsi_dev->version)
			return 1;
	}

	return 0;
}

struct bus_type fsi_bus_type = {
	.name		= "fsi",
	.match		= fsi_bus_match,
};
EXPORT_SYMBOL_GPL(fsi_bus_type);

static int fsi_init(void)
{
	return bus_register(&fsi_bus_type);
}

static void fsi_exit(void)
{
	bus_unregister(&fsi_bus_type);
}

module_init(fsi_init);
module_exit(fsi_exit);
