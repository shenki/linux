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

#define FSI_SLAVE_CONF_NEXT_MASK	0x80000000
#define FSI_SLAVE_CONF_SLOTS_MASK	0x00ff0000
#define FSI_SLAVE_CONF_SLOTS_SHIFT	16
#define FSI_SLAVE_CONF_VERSION_MASK	0x0000f000
#define FSI_SLAVE_CONF_VERSION_SHIFT	12
#define FSI_SLAVE_CONF_TYPE_MASK	0x00000ff0
#define FSI_SLAVE_CONF_TYPE_SHIFT	4
#define FSI_SLAVE_CONF_CRC_SHIFT	4
#define FSI_SLAVE_CONF_CRC_MASK		0x0000000f
#define FSI_SLAVE_CONF_DATA_BITS	28

#define FSI_PEEK_BASE			0x410
#define	FSI_SLAVE_BASE			0x800

static const int engine_page_size = 0x400;

static atomic_t master_idx = ATOMIC_INIT(-1);

struct fsi_slave {
	struct list_head	list_link;	/* Master's list of slaves */
	struct list_head	my_engines;
	struct device		dev;
	struct fsi_master	*master;
	int			link;
	uint8_t			id;
};

#define to_fsi_slave(d) container_of(d, struct fsi_slave, dev)

static int fsi_slave_read(struct fsi_slave *slave, uint32_t addr,
		void *val, size_t size);
static int fsi_slave_write(struct fsi_slave *slave, uint32_t addr,
		const void *val, size_t size);

/*
 * FSI slave engine control register offsets
 */
#define	FSI_SMODE		0x0	/* R/W: Mode register */

/*
 * SMODE fields
 */
#define	FSI_SMODE_WSC		0x80000000	/* Warm start done */
#define	FSI_SMODE_ECRC		0x20000000	/* Hw CRC check */
#define	FSI_SMODE_SID_SHIFT	24		/* ID shift */
#define	FSI_SMODE_SID_MASK	3		/* ID Mask */
#define	FSI_SMODE_ED_SHIFT	20		/* Echo delay shift */
#define	FSI_SMODE_ED_MASK	0xf		/* Echo delay mask */
#define	FSI_SMODE_SD_SHIFT	16		/* Send delay shift */
#define	FSI_SMODE_SD_MASK	0xf		/* Send delay mask */
#define	FSI_SMODE_LBCRR_SHIFT	8		/* Clk ratio shift */
#define	FSI_SMODE_LBCRR_MASK	0xf		/* Clk ratio mask */

/* FSI endpoint-device support */
int fsi_device_read(struct fsi_device *dev, uint32_t addr, void *val,
		size_t size)
{
	if (addr > dev->size)
		return -EINVAL;

	if (addr + size > dev->size)
		return -EINVAL;

	return fsi_slave_read(dev->slave, dev->addr + addr, val, size);
}
EXPORT_SYMBOL_GPL(fsi_device_read);

int fsi_device_write(struct fsi_device *dev, uint32_t addr, const void *val,
		size_t size)
{
	if (addr > dev->size)
		return -EINVAL;

	if (addr + size > dev->size)
		return -EINVAL;

	return fsi_slave_write(dev->slave, dev->addr + addr, val, size);
}
EXPORT_SYMBOL_GPL(fsi_device_write);

int fsi_device_peek(struct fsi_device *dev, void *val)
{
	uint32_t addr = FSI_PEEK_BASE + ((dev->unit - 2) * sizeof(uint32_t));

	return fsi_slave_read(dev->slave, addr, val, sizeof(uint32_t));
}

static void fsi_device_release(struct device *_device)
{
	struct fsi_device *device = to_fsi_dev(_device);

	kfree(device);
}

static struct fsi_device *fsi_create_device(struct fsi_slave *slave)
{
	struct fsi_device *dev;

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return NULL;

	dev->dev.parent = &slave->dev;
	dev->dev.bus = &fsi_bus_type;
	dev->dev.release = fsi_device_release;

	return dev;
}

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

/* Encode slave local bus echo delay */
static inline uint32_t fsi_smode_echodly(int x)
{
	return (x & FSI_SMODE_ED_MASK) << FSI_SMODE_ED_SHIFT;
}

/* Encode slave local bus send delay */
static inline uint32_t fsi_smode_senddly(int x)
{
	return (x & FSI_SMODE_SD_MASK) << FSI_SMODE_SD_SHIFT;
}

/* Encode slave local bus clock rate ratio */
static inline uint32_t fsi_smode_lbcrr(int x)
{
	return (x & FSI_SMODE_LBCRR_MASK) << FSI_SMODE_LBCRR_SHIFT;
}

/* Encode slave ID */
static inline uint32_t fsi_smode_sid(int x)
{
	return (x & FSI_SMODE_SID_MASK) << FSI_SMODE_SID_SHIFT;
}

static int fsi_slave_read(struct fsi_slave *slave, uint32_t addr,
			void *val, size_t size)
{
	return slave->master->read(slave->master, slave->link,
			slave->id, addr, val, size);
}

static int fsi_slave_write(struct fsi_slave *slave, uint32_t addr,
			const void *val, size_t size)
{
	return slave->master->write(slave->master, slave->link,
			slave->id, addr, val, size);
}

static int fsi_slave_scan(struct fsi_slave *slave)
{
	uint32_t engine_addr;
	uint32_t conf;
	int rc, i;

	INIT_LIST_HEAD(&slave->my_engines);

	/*
	 * scan engines
	 *
	 * We keep the peek mode and slave engines for the core; so start
	 * at the third slot in the configuration table. We also need to
	 * skip the chip ID entry at the start of the address space.
	 */
	engine_addr = engine_page_size * 3;
	for (i = 2; i < engine_page_size / sizeof(uint32_t); i++) {
		uint8_t slots, version, type, crc;
		struct fsi_device *dev;

		rc = fsi_slave_read(slave, (i + 1) * sizeof(conf),
				&conf, sizeof(conf));
		if (rc) {
			dev_warn(&slave->dev,
				"error reading slave registers\n");
			return -1;
		}

		crc = fsi_crc4(0, conf >> FSI_SLAVE_CONF_CRC_SHIFT,
				FSI_SLAVE_CONF_DATA_BITS);
		if (crc != (conf & FSI_SLAVE_CONF_CRC_MASK)) {
			dev_warn(&slave->dev,
				"crc error in slave register at 0x%04x\n",
				i);
			return -1;
		}

		slots = (conf & FSI_SLAVE_CONF_SLOTS_MASK)
			>> FSI_SLAVE_CONF_SLOTS_SHIFT;
		version = (conf & FSI_SLAVE_CONF_VERSION_MASK)
			>> FSI_SLAVE_CONF_VERSION_SHIFT;
		type = (conf & FSI_SLAVE_CONF_TYPE_MASK)
			>> FSI_SLAVE_CONF_TYPE_SHIFT;

		/*
		 * Unused address areas are marked by a zero type value; this
		 * skips the defined address areas
		 */
		if (type != 0) {

			/* create device */
			dev = fsi_create_device(slave);
			if (!dev)
				return -ENOMEM;

			dev->slave = slave;
			dev->engine_type = type;
			dev->version = version;
			dev->unit = i;
			dev->addr = engine_addr;
			dev->size = slots * engine_page_size;

			dev_info(&slave->dev,
			"engine[%i]: type %x, version %x, addr %x size %x\n",
					dev->unit, dev->engine_type, version,
					dev->addr, dev->size);

			device_initialize(&dev->dev);
			dev_set_name(&dev->dev, "%02x:%02x:%02x:%02x",
					slave->master->idx, slave->link,
					slave->id, i - 2);

			rc = device_add(&dev->dev);
			if (rc) {
				dev_warn(&slave->dev, "add failed: %d\n", rc);
				put_device(&dev->dev);
				continue;
			}
			list_add(&dev->link, &slave->my_engines);
		}

		engine_addr += slots * engine_page_size;

		if (!(conf & FSI_SLAVE_CONF_NEXT_MASK))
			break;
	}

	return 0;
}

static void fsi_slave_release(struct device *dev)
{
	struct fsi_slave *slave = to_fsi_slave(dev);

	kfree(slave);
}

static uint32_t set_smode_defaults(struct fsi_master *master)
{
	return FSI_SMODE_WSC | FSI_SMODE_ECRC
		| fsi_smode_echodly(0xf) | fsi_smode_senddly(0xf)
		| fsi_smode_lbcrr(1);
}

static int fsi_slave_set_smode(struct fsi_master *master, int link, int id)
{
	uint32_t smode = set_smode_defaults(master);

	smode |= fsi_smode_sid(id);
	return master->write(master, link, 3, FSI_SLAVE_BASE + FSI_SMODE,
				&smode, sizeof(smode));
}

static int fsi_slave_init(struct fsi_master *master,
		int link, uint8_t slave_id)
{
	struct fsi_slave *slave;
	uint32_t chip_id;
	int rc;
	uint8_t crc;

	/*
	 * todo: Due to CFAM hardware issues related to BREAK commands we're
	 * limited to only one CFAM per link.  Once issues are resolved this
	 * restriction can be removed.
	 */
	if (slave_id > 0)
		return 0;

	rc = fsi_slave_set_smode(master, link, slave_id);
	if (rc) {
		dev_warn(master->dev, "can't set smode on slave:%02x:%02x %d\n",
				link, slave_id, rc);
		return -ENODEV;
	}

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
	list_add(&slave->list_link, &master->my_slaves);
	fsi_slave_scan(slave);
	return 0;
}

/* FSI master support */

static int fsi_master_link_enable(struct fsi_master *master, int link)
{
	if (master->link_enable)
		return master->link_enable(master, link);

	return 0;
}

/*
 * Issue a break command on this link
 */
static int fsi_master_break(struct fsi_master *master, int link)
{
	if (master->send_break)
		return master->send_break(master, link);

	return 0;
}

static int fsi_master_scan(struct fsi_master *master)
{
	int link, slave_id, rc;
	uint32_t smode;

	if (!master->slave_list) {
		INIT_LIST_HEAD(&master->my_slaves);
		master->slave_list = true;
	}

	for (link = 0; link < master->n_links; link++) {
		rc = fsi_master_link_enable(master, link);
		if (rc) {
			dev_dbg(master->dev,
				"enable link:%d failed with:%d\n", link, rc);
			continue;
		}
		rc = fsi_master_break(master, link);
		if (rc) {
			dev_dbg(master->dev,
				"Break to link:%d failed with:%d\n", link, rc);
			continue;
		}

		/*
		 * Verify can read slave at default ID location. If fails
		 * there must be nothing on other end of link
		 */
		rc = master->read(master, link, 3, FSI_SLAVE_BASE + FSI_SMODE,
				&smode, sizeof(smode));
		if (rc) {
			dev_dbg(master->dev,
				"Read link:%d smode default id failed:%d\n",
				link, rc);
			continue;
		}

		for (slave_id = 0; slave_id < FSI_N_SLAVES; slave_id++)
			fsi_slave_init(master, link, slave_id);

	}

	return 0;
}

static void fsi_master_unscan(struct fsi_master *master)
{
	struct fsi_slave *slave, *slave_tmp;
	struct fsi_device *fsi_dev, *fsi_dev_tmp;

	if (!master->slave_list)
		return;

	list_for_each_entry_safe(slave, slave_tmp, &master->my_slaves,
							list_link) {
		list_del(&slave->list_link);
		list_for_each_entry_safe(fsi_dev, fsi_dev_tmp,
					&slave->my_engines, link) {
			list_del(&fsi_dev->link);
			put_device(&fsi_dev->dev);
		}
		device_unregister(&slave->dev);
	}
	master->slave_list = false;
}

int fsi_master_register(struct fsi_master *master)
{
	master->idx = atomic_inc_return(&master_idx);
	master->slave_list = false;
	get_device(master->dev);
	fsi_master_scan(master);
	return 0;
}
EXPORT_SYMBOL_GPL(fsi_master_register);

void fsi_master_unregister(struct fsi_master *master)
{
	fsi_master_unscan(master);
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

int fsi_driver_register(struct fsi_driver *fsi_drv)
{
	if (!fsi_drv)
		return -EINVAL;
	if (!fsi_drv->id_table)
		return -EINVAL;

	return driver_register(&fsi_drv->drv);
}
EXPORT_SYMBOL_GPL(fsi_driver_register);

void fsi_driver_unregister(struct fsi_driver *fsi_drv)
{
	driver_unregister(&fsi_drv->drv);
}
EXPORT_SYMBOL_GPL(fsi_driver_unregister);

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
