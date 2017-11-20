/*
 * Copyright 2017 IBM Corp.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <asm/unaligned.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/fsi-sbefifo.h>
#include <linux/idr.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/uaccess.h>
#include <linux/wait.h>
#include <linux/workqueue.h>

#define OCC_SRAM_BYTES		4096
#define OCC_CMD_DATA_BYTES	4090
#define OCC_RESP_DATA_BYTES	4089

#define OCC_RESP_CMD_IN_PRG    0xFF

#define OCC_TIMEOUT_MS		1000
#define OCC_CMD_IN_PRG_WAIT_MS	50

struct occ {
	struct device *sbefifo;
	char name[32];
	int idx;
	struct list_head xfrs;
	spinlock_t list_lock;		/* lock access to the xfrs list */
	struct mutex occ_lock;		/* lock access to the hardware */
	struct work_struct work;
	bool cancel;
};

#define to_occ(x)	container_of((x), struct occ, mdev)

struct occ_response {
	u8 seq_no;
	u8 cmd_type;
	u8 return_status;
	__be16 data_length;
	u8 data[OCC_RESP_DATA_BYTES];
	__be16 checksum;
} __packed;

/*
 * transfer flags are NOT mutually exclusive
 *
 * Initial flags are none; transfer is created and queued from write(). All
 *  flags are cleared when the transfer is completed by closing the file or
 *  reading all of the available response data.
 * XFR_IN_PROGRESS is set when a transfer is started from occ_worker_putsram,
 *  and cleared if the transfer fails or occ_worker_getsram completes.
 * XFR_COMPLETE is set when a transfer fails or finishes occ_worker_getsram.
 * XFR_CANCELED is set when the transfer's client is released.
 */
enum {
	XFR_IN_PROGRESS,
	XFR_COMPLETE,
	XFR_CANCELED,
};

struct occ_xfr {
	struct list_head link;
	int rc;
	u8 buf[OCC_SRAM_BYTES];
	size_t cmd_data_length;
	size_t resp_data_length;
	unsigned long flags;
};

/*
 * client flags
 *
 * CLIENT_NONBLOCKING is set during open() if the file was opened with the
 *  O_NONBLOCK flag.
 * CLIENT_XFR_PENDING is set during write() and cleared when all data has been
 *  read.
 */
enum {
	CLIENT_NONBLOCKING,
	CLIENT_XFR_PENDING,
};

struct occ_client {
	struct kref kref;
	struct occ *occ;
	struct occ_xfr xfr;
	spinlock_t lock;		/* lock access to the client state */
	wait_queue_head_t wait;
	size_t read_offset;
	unsigned long flags;
};

#define to_client(x)	container_of((x), struct occ_client, xfr)

static struct workqueue_struct *occ_wq;

static DEFINE_IDA(occ_ida);

static void occ_get_client(struct occ_client *client)
{
	kref_get(&client->kref);
}

static void occ_client_release(struct kref *kref)
{
	struct occ_client *client = container_of(kref, struct occ_client,
						 kref);

	kfree(client);
}

static void occ_put_client(struct occ_client *client)
{
	kref_put(&client->kref, occ_client_release);
}

static int occ_write_sbefifo(struct sbefifo_client *client, const char *buf,
			     ssize_t len)
{
	int rc;
	ssize_t total = 0;

	do {
		rc = sbefifo_drv_write(client, &buf[total], len - total);
		if (rc < 0)
			return rc;
		else if (!rc)
			break;

		total += rc;
	} while (total < len);

	return (total == len) ? 0 : -ENOSPC;
}

static int occ_read_sbefifo(struct sbefifo_client *client, char *buf,
			    ssize_t len)
{
	int rc;
	ssize_t total = 0;

	do {
		rc = sbefifo_drv_read(client, &buf[total], len - total);
		if (rc < 0)
			return rc;
		else if (!rc)
			break;

		total += rc;
	} while (total < len);

	return (total == len) ? 0 : -ENODATA;
}

static int occ_getsram(struct device *sbefifo, u32 address, u8 *data,
		       ssize_t len)
{
	int rc;
	u8 *resp;
	__be32 buf[5];
	u32 data_len = ((len + 7) / 8) * 8;	/* must be multiples of 8 B */
	struct sbefifo_client *client;

	/*
	 * Magic sequence to do SBE getsram command. SBE will fetch data from
	 * specified SRAM address.
	 */
	buf[0] = cpu_to_be32(0x5);
	buf[1] = cpu_to_be32(0xa403);
	buf[2] = cpu_to_be32(1);
	buf[3] = cpu_to_be32(address);
	buf[4] = cpu_to_be32(data_len);

	client = sbefifo_drv_open(sbefifo, 0);
	if (!client)
		return -ENODEV;

	rc = occ_write_sbefifo(client, (const char *)buf, sizeof(buf));
	if (rc)
		goto done;

	resp = kzalloc(data_len, GFP_KERNEL);
	if (!resp) {
		rc = -ENOMEM;
		goto done;
	}

	rc = occ_read_sbefifo(client, (char *)resp, data_len);
	if (rc)
		goto free;

	/* check for good response */
	rc = occ_read_sbefifo(client, (char *)buf, 8);
	if (rc)
		goto free;

	if ((be32_to_cpu(buf[0]) == data_len) &&
	    (be32_to_cpu(buf[1]) == 0xC0DEA403))
		memcpy(data, resp, len);
	else
		rc = -EBADMSG;

free:
	kfree(resp);

done:
	sbefifo_drv_release(client);
	return rc;
}

static int occ_putsram(struct device *sbefifo, u32 address, u8 *data,
		       ssize_t len)
{
	int rc;
	__be32 *buf;
	u32 data_len = ((len + 7) / 8) * 8;	/* must be multiples of 8 B */
	size_t cmd_len = data_len + 20;
	struct sbefifo_client *client;

	buf = kzalloc(cmd_len, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	/*
	 * Magic sequence to do SBE putsram command. SBE will transfer
	 * data to specified SRAM address.
	 */
	buf[0] = cpu_to_be32(0x5 + (data_len / 4));
	buf[1] = cpu_to_be32(0xa404);
	buf[2] = cpu_to_be32(1);
	buf[3] = cpu_to_be32(address);
	buf[4] = cpu_to_be32(data_len);

	memcpy(&buf[5], data, len);

	client = sbefifo_drv_open(sbefifo, 0);
	if (!client) {
		rc = -ENODEV;
		goto free;
	}

	rc = occ_write_sbefifo(client, (const char *)buf, cmd_len);
	if (rc)
		goto done;

	rc = occ_read_sbefifo(client, (char *)buf, 8);
	if (rc)
		goto done;

	/* check for good response */
	if ((be32_to_cpu(buf[0]) != data_len) ||
	    (be32_to_cpu(buf[1]) != 0xC0DEA404))
		rc = -EBADMSG;

done:
	sbefifo_drv_release(client);
free:
	kfree(buf);
	return rc;
}

static int occ_trigger_attn(struct device *sbefifo)
{
	int rc;
	__be32 buf[6];
	struct sbefifo_client *client;

	/*
	 * Magic sequence to do SBE putscom command. SBE will write 8 bytes to
	 * specified SCOM address.
	 */
	buf[0] = cpu_to_be32(0x6);
	buf[1] = cpu_to_be32(0xa202);
	buf[2] = 0;
	buf[3] = cpu_to_be32(0x6D035);
	buf[4] = cpu_to_be32(0x20010000);	/* trigger occ attention */
	buf[5] = 0;

	client = sbefifo_drv_open(sbefifo, 0);
	if (!client)
		return -ENODEV;

	rc = occ_write_sbefifo(client, (const char *)buf, sizeof(buf));
	if (rc)
		goto done;

	rc = occ_read_sbefifo(client, (char *)buf, 8);
	if (rc)
		goto done;

	/* check for good response */
	if ((be32_to_cpu(buf[0]) != 0xC0DEA202) ||
	    (be32_to_cpu(buf[1]) & 0x0FFFFFFF))
		rc = -EBADMSG;

done:
	sbefifo_drv_release(client);

	return rc;
}

static void occ_worker(struct work_struct *work)
{
	int rc = 0, empty;
	u16 resp_data_length;
	unsigned long flags;
	unsigned long start;
	const unsigned long timeout = msecs_to_jiffies(OCC_TIMEOUT_MS);
	const long int wait_time = msecs_to_jiffies(OCC_CMD_IN_PRG_WAIT_MS);
	struct occ_xfr *xfr;
	struct occ_response *resp;
	struct occ_client *client;
	struct occ *occ = container_of(work, struct occ, work);
	struct device *sbefifo = occ->sbefifo;

again:
	if (occ->cancel)
		return;

	spin_lock_irqsave(&occ->list_lock, flags);

	xfr = list_first_entry_or_null(&occ->xfrs, struct occ_xfr, link);
	if (!xfr) {
		spin_unlock_irqrestore(&occ->list_lock, flags);
		return;
	}

	client = to_client(xfr);
	occ_get_client(client);
	resp = (struct occ_response *)xfr->buf;
	set_bit(XFR_IN_PROGRESS, &xfr->flags);

	spin_unlock_irqrestore(&occ->list_lock, flags);
	mutex_lock(&occ->occ_lock);

	start = jiffies;

	/* write occ command */
	rc = occ_putsram(sbefifo, 0xFFFBE000, xfr->buf,
			 xfr->cmd_data_length);
	if (rc)
		goto done;

	rc = occ_trigger_attn(sbefifo);
	if (rc)
		goto done;

	/* read occ response */
	do {
		rc = occ_getsram(sbefifo, 0xFFFBF000, xfr->buf, 8);
		if (rc)
			goto done;

		if (resp->return_status == OCC_RESP_CMD_IN_PRG) {
			rc = -EALREADY;

			if (time_after(jiffies, start + timeout))
				break;

			set_current_state(TASK_INTERRUPTIBLE);
			schedule_timeout(wait_time);
		}
	} while (rc);

	resp_data_length = get_unaligned_be16(&resp->data_length);
	if (resp_data_length > OCC_RESP_DATA_BYTES) {
		rc = -EMSGSIZE;
		goto done;
	}

	if (resp_data_length > 1) {
		/* already got 3 bytes resp, also need 2 bytes checksum */
		rc = occ_getsram(sbefifo, 0xFFFBF008, &xfr->buf[8],
				 resp_data_length - 1);
		if (rc)
			goto done;
	}

	xfr->resp_data_length = resp_data_length + 7;

done:
	mutex_unlock(&occ->occ_lock);

	xfr->rc = rc;
	set_bit(XFR_COMPLETE, &xfr->flags);

	spin_lock_irqsave(&occ->list_lock, flags);

	clear_bit(XFR_IN_PROGRESS, &xfr->flags);
	list_del(&xfr->link);
	empty = list_empty(&occ->xfrs);

	spin_unlock_irqrestore(&occ->list_lock, flags);

	wake_up_interruptible(&client->wait);
	occ_put_client(client);

	if (!empty)
		goto again;
}

static int occ_probe(struct platform_device *pdev)
{
	int rc;
	u32 reg;
	struct occ *occ;
	struct device *dev = &pdev->dev;

	occ = devm_kzalloc(dev, sizeof(*occ), GFP_KERNEL);
	if (!occ)
		return -ENOMEM;

	occ->sbefifo = dev->parent;
	INIT_LIST_HEAD(&occ->xfrs);
	spin_lock_init(&occ->list_lock);
	mutex_init(&occ->occ_lock);
	INIT_WORK(&occ->work, occ_worker);

	if (dev->of_node) {
		rc = of_property_read_u32(dev->of_node, "reg", &reg);
		if (!rc) {
			/* make sure we don't have a duplicate from dts */
			occ->idx = ida_simple_get(&occ_ida, reg, reg + 1,
						  GFP_KERNEL);
			if (occ->idx < 0)
				occ->idx = ida_simple_get(&occ_ida, 1, INT_MAX,
							  GFP_KERNEL);
		} else {
			occ->idx = ida_simple_get(&occ_ida, 1, INT_MAX,
						  GFP_KERNEL);
		}
	} else {
		occ->idx = ida_simple_get(&occ_ida, 1, INT_MAX, GFP_KERNEL);
	}

	platform_set_drvdata(pdev, occ);

	return 0;
}

static int occ_remove(struct platform_device *pdev)
{
	unsigned long flags;
	struct occ *occ = platform_get_drvdata(pdev);
	struct occ_xfr *xfr;
	struct occ_client *client;

	occ->cancel = true;

	spin_lock_irqsave(&occ->list_lock, flags);
	list_for_each_entry(xfr, &occ->xfrs, link) {
		client = to_client(xfr);
		wake_up_all(&client->wait);
	}
	spin_unlock_irqrestore(&occ->list_lock, flags);

	cancel_work_sync(&occ->work);

	ida_simple_remove(&occ_ida, occ->idx);

	return 0;
}

static const struct of_device_id occ_match[] = {
	{ .compatible = "ibm,p9-occ" },
	{ },
};

static struct platform_driver occ_driver = {
	.driver = {
		.name = "occ",
		.of_match_table	= occ_match,
	},
	.probe	= occ_probe,
	.remove = occ_remove,
};

static int occ_init(void)
{
	occ_wq = create_singlethread_workqueue("occ");
	if (!occ_wq)
		return -ENOMEM;

	return platform_driver_register(&occ_driver);
}

static void occ_exit(void)
{
	destroy_workqueue(occ_wq);

	platform_driver_unregister(&occ_driver);

	ida_destroy(&occ_ida);
}

module_init(occ_init);
module_exit(occ_exit);

MODULE_AUTHOR("Eddie James <eajames@us.ibm.com>");
MODULE_DESCRIPTION("BMC P9 OCC driver");
MODULE_LICENSE("GPL");
