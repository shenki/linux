/*
 *  Bogus Block Driver for PowerPC Full System Simulator
 *
 *  (C) Copyright IBM Corporation 2003-2005
 *
 *  Bogus Disk Driver
 *
 *  Author: Eric Van Hensbegren <ericvh@gmail.com>
 *
 *    inspired by drivers/block/nbd.c
 *    written by Pavel Machek and Steven Whitehouse
 *
 *  Some code is from the IBM Full System Simulator Group in ARL
 *  Author: Patrick Bohrer <IBM Austin Research Lab>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to:
 * Free Software Foundation
 * 51 Franklin Street, Fifth Floor
 * Boston, MA  02111-1301  USA
 *
 */

#define DEBUG

#include <linux/major.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/fs.h>
#include <linux/stat.h>
#include <linux/errno.h>
#include <linux/file.h>
#include <linux/ioctl.h>
#include <linux/blkdev.h>
#include <net/sock.h>
#include <asm/prom.h>
#include <asm/systemsim.h>
#include <asm/prom.h>

#include <asm/uaccess.h>
#include <asm/types.h>

#define MAJOR_NR		42
#define MAX_SYSTEMSIM_BD	4
#define BD_SECT_SZ		512

#define SYSTEMSIM_BD_SET_BLKSIZE	_IO( 0xab, 1 )
#define SYSTEMSIM_BD_SET_SIZE   	_IO( 0xab, 2 )
#define SYSTEMSIM_BD_SET_SIZE_BLOCKS    _IO( 0xab, 7 )
#define SYSTEMSIM_BD_DISCONNECT  	_IO( 0xab, 8 )

struct systemsim_bd_device {
	int initialized;
	int changed;
	int refcnt;
	int flags;
	struct gendisk *disk;
};

static struct systemsim_bd_device systemsim_bd_dev[MAX_SYSTEMSIM_BD];

#define BD_INFO_SYNC   0
#define BD_INFO_STATUS 1
#define BD_INFO_BLKSZ  2
#define BD_INFO_DEVSZ  3
#define BD_INFO_CHANGE 4

#define BOGUS_DISK_READ  116
#define BOGUS_DISK_WRITE 117
#define BOGUS_DISK_INFO  118

static inline int
systemsim_disk_read(int devno, void *buf, ulong sect, ulong nrsect)
{
	memset(buf, 0, nrsect*BD_SECT_SZ);
	return callthru3(BOGUS_DISK_READ, (unsigned long)buf,
			 (unsigned long)sect,
			 (unsigned long)((nrsect << 16) | devno));
}

static inline int
systemsim_disk_write(int devno, void *buf, ulong sect, ulong nrsect)
{
	return callthru3(BOGUS_DISK_WRITE, (unsigned long)buf,
			 (unsigned long)sect,
			 (unsigned long)((nrsect << 16) | devno));
}

static inline int systemsim_disk_info(int op, int devno)
{
	return callthru2(BOGUS_DISK_INFO, (unsigned long)op,
			 (unsigned long)devno);
}

static int systemsim_bd_init_disk(int devno)
{
	struct gendisk *disk = systemsim_bd_dev[devno].disk;
	unsigned int sz;
	int rc;

	/* check disk configured */
	rc = systemsim_disk_info(BD_INFO_STATUS, devno);
	pr_debug("mambobd%d: bd_init_disk, status = %d\n", devno, rc);
	if (rc <= 0)
		return 0;

	systemsim_bd_dev[devno].initialized++;
	systemsim_bd_dev[devno].changed = 0;

	sz = systemsim_disk_info(BD_INFO_DEVSZ, devno);

	printk("Initializing disk %d with devsz %u\n", devno, sz);

	set_capacity(disk, sz << 1);

	return 1;
}

static void do_systemsim_bd_request(struct request_queue * q)
{
	struct request *req;

	req = blk_fetch_request(q);
	while (req) {
		int minor = req->rq_disk->first_minor;
		int result = -EIO;

		if (systemsim_bd_dev[minor].changed)
			goto done;

		switch (rq_data_dir(req)) {
		case READ:
			result = systemsim_disk_read(minor,
						     bio_data(req->bio),
						     blk_rq_pos(req),
						     blk_rq_cur_sectors(req));
			break;
		case WRITE:
			result = systemsim_disk_write(minor,
						      bio_data(req->bio),
						      blk_rq_pos(req),
						      blk_rq_cur_sectors(req));
		};
	done:
		if (!__blk_end_request_cur(req, result ? -EIO : 0))
			req = blk_fetch_request(q);
	}
}

static void systemsim_bd_release(struct gendisk *disk, fmode_t mode)
{
	struct systemsim_bd_device *lo;
	int dev;

	if (!disk)
		return;
	dev = disk->first_minor;
	if (dev >= MAX_SYSTEMSIM_BD)
		return;
	if (systemsim_disk_info(BD_INFO_SYNC, dev) < 0) {
		printk(KERN_ALERT "systemsim_bd_release: unable to sync\n");
	}
	lo = &systemsim_bd_dev[dev];
	if (lo->refcnt <= 0)
		printk(KERN_ALERT "systemsim_bd_release: refcount(%d) <= 0\n",
		       lo->refcnt);
	lo->refcnt--;
}

static int systemsim_bd_revalidate(struct gendisk *disk)
{
	int devno = disk->first_minor;

	pr_debug("mambobd%d: revalidate\n", devno);

	systemsim_bd_init_disk(devno);

	return 0;
}

static int systemsim_bd_media_changed(struct gendisk *disk)
{
	int devno = disk->first_minor;
	int rc;

	rc = systemsim_disk_info(BD_INFO_CHANGE, devno);
	/* Disk not initialized ... no change */
	pr_debug("mambobd%d: media_changed, rc = %d\n", devno, rc);
	if (rc < 0)
		return 0;
	if (rc)
		systemsim_bd_dev[devno].changed = 1;
	return systemsim_bd_dev[devno].changed;
}

static int systemsim_bd_open(struct block_device *bdev, fmode_t mode)
{
	int dev;

	if (!bdev)
		return -EINVAL;
	dev = bdev->bd_disk->first_minor;
	if (dev >= MAX_SYSTEMSIM_BD)
		return -ENODEV;

	check_disk_change(bdev);

	if (!systemsim_bd_dev[dev].initialized && !systemsim_bd_init_disk(dev))
		return -ENOMEDIUM;
	if (systemsim_bd_dev[dev].changed)
		return -ENOMEDIUM;

	systemsim_bd_dev[dev].refcnt++;
	return 0;
}

static struct block_device_operations systemsim_bd_fops = {
      .owner		= THIS_MODULE,
      .open		= systemsim_bd_open,
      .release		= systemsim_bd_release,
      .media_changed	= systemsim_bd_media_changed,
      .revalidate_disk	= systemsim_bd_revalidate,
};

static DEFINE_SPINLOCK(systemsim_bd_lock);

static int __init systemsim_bd_init(void)
{
	struct device_node *systemsim;
	int err = -ENOMEM;
	int i;

	systemsim = of_find_node_by_path("/systemsim");
	if (systemsim == NULL) {
		printk("systemsim_bd: /systemsim not found in device-tree");
		return -1;
	}
	of_node_put(systemsim);

	/*
	 * We could detect which disks are configured in openfirmware
	 * but I think this unnecessarily limits us from being able to
	 * hot-plug bogus disks durning run-time.
	 *
	 */

	for (i = 0; i < MAX_SYSTEMSIM_BD; i++) {
		struct gendisk *disk = alloc_disk(1);
		if (!disk)
			goto out;
		systemsim_bd_dev[i].disk = disk;
		/*
		 * The new linux 2.5 block layer implementation requires
		 * every gendisk to have its very own request_queue struct.
		 * These structs are big so we dynamically allocate them.
		 */
		disk->queue =
		    blk_init_queue(do_systemsim_bd_request, &systemsim_bd_lock);
		if (!disk->queue) {
			put_disk(disk);
			goto out;
		}
	}

	if (register_blkdev(MAJOR_NR, "systemsim_bd")) {
		err = -EIO;
		goto out;
	}
#ifdef MODULE
	printk("systemsim bogus disk: registered device at major %d\n",
	       MAJOR_NR);
#else
	printk("systemsim bogus disk: compiled in with kernel\n");
#endif

	/*
	 * left device name alone for now as too much depends on it
	 * external to the kernel
	 *
	 */

	for (i = 0; i < MAX_SYSTEMSIM_BD; i++) {	/* load defaults */
		struct gendisk *disk = systemsim_bd_dev[i].disk;
		systemsim_bd_dev[i].initialized = 0;
		systemsim_bd_dev[i].refcnt = 0;
		systemsim_bd_dev[i].flags = 0;
		systemsim_bd_dev[i].changed = 0;
		disk->major = MAJOR_NR;
		disk->first_minor = i;
		disk->fops = &systemsim_bd_fops;
		disk->private_data = &systemsim_bd_dev[i];
		disk->flags |= GENHD_FL_REMOVABLE;
		sprintf(disk->disk_name, "mambobd%d", i);
		set_capacity(disk, 0x7ffffc00ULL << 1);	/* 2 TB */
		add_disk(disk);
	}

	return 0;
      out:
	while (i--) {
		if (systemsim_bd_dev[i].disk->queue)
			blk_cleanup_queue(systemsim_bd_dev[i].disk->queue);
		put_disk(systemsim_bd_dev[i].disk);
	}
	return -EIO;
}

static void __exit systemsim_bd_cleanup(void)
{

	unregister_blkdev(MAJOR_NR, "systemsim_bd");
}

module_init(systemsim_bd_init);
module_exit(systemsim_bd_cleanup);

MODULE_DESCRIPTION("Systemsim Block Device");
MODULE_LICENSE("GPL");
