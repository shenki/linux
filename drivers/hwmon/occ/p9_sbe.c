/*
 * Copyright 2017 IBM Corp.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/device.h>
#include <linux/errno.h>
#include <linux/fsi-occ.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>

#include "common.h"

struct p9_sbe_occ {
	struct occ occ;
	struct device *sbe;

	/*
	 * Pointer to occ device client. We store this so that we can cancel
	 * the client operations in remove() if necessary. We only need one
	 * pointer since we do one OCC operation (open, write, read, close) at
	 * a time (access to p9_sbe_occ_send_cmd is locked in the common code
	 * with occ.lock).
	 */
	struct occ_client *client;

	/*
	 * This lock controls access to the client pointer and ensures atomic
	 * open, close and NULL assignment. This prevents simultaneous opening
	 * and closing of the client, or closing multiple times.
	 */
	spinlock_t lock;
};

#define to_p9_sbe_occ(x)	container_of((x), struct p9_sbe_occ, occ)

static void p9_sbe_occ_close_client(struct p9_sbe_occ *occ)
{
	unsigned long flags;
	struct occ_client *tmp_client;

	spin_lock_irqsave(&occ->lock, flags);
	tmp_client = occ->client;
	occ->client = NULL;
	occ_drv_release(tmp_client);
	spin_unlock_irqrestore(&occ->lock, flags);
}

static int p9_sbe_occ_send_cmd(struct occ *occ, u8 *cmd)
{
	int rc;
	unsigned long flags;
	struct occ_response *resp = &occ->resp;
	struct p9_sbe_occ *p9_sbe_occ = to_p9_sbe_occ(occ);

	spin_lock_irqsave(&p9_sbe_occ->lock, flags);
	if (p9_sbe_occ->sbe)
		p9_sbe_occ->client = occ_drv_open(p9_sbe_occ->sbe, 0);
	spin_unlock_irqrestore(&p9_sbe_occ->lock, flags);

	if (!p9_sbe_occ->client)
		return -ENODEV;

	/* skip first byte (sequence number), OCC driver handles it */
	rc = occ_drv_write(p9_sbe_occ->client, (const char *)&cmd[1], 7);
	if (rc < 0)
		goto err;

	rc = occ_drv_read(p9_sbe_occ->client, (char *)resp, sizeof(*resp));
	if (rc < 0)
		goto err;

	/* check the OCC response */
	switch (resp->return_status) {
	case OCC_RESP_CMD_IN_PRG:
		rc = -ETIMEDOUT;
		break;
	case OCC_RESP_SUCCESS:
		rc = 0;
		break;
	case OCC_RESP_CMD_INVAL:
	case OCC_RESP_CMD_LEN_INVAL:
	case OCC_RESP_DATA_INVAL:
	case OCC_RESP_CHKSUM_ERR:
		rc = -EINVAL;
		break;
	case OCC_RESP_INT_ERR:
	case OCC_RESP_BAD_STATE:
	case OCC_RESP_CRIT_EXCEPT:
	case OCC_RESP_CRIT_INIT:
	case OCC_RESP_CRIT_WATCHDOG:
	case OCC_RESP_CRIT_OCB:
	case OCC_RESP_CRIT_HW:
		rc = -EREMOTEIO;
		break;
	default:
		rc = -EPROTO;
	}

err:
	p9_sbe_occ_close_client(p9_sbe_occ);
	return rc;
}

static int p9_sbe_occ_probe(struct platform_device *pdev)
{
	struct occ *occ;
	struct p9_sbe_occ *p9_sbe_occ = devm_kzalloc(&pdev->dev,
						     sizeof(*p9_sbe_occ),
						     GFP_KERNEL);
	if (!p9_sbe_occ)
		return -ENOMEM;

	p9_sbe_occ->sbe = pdev->dev.parent;
	occ = &p9_sbe_occ->occ;
	occ->bus_dev = &pdev->dev;
	platform_set_drvdata(pdev, occ);

	occ->poll_cmd_data = 0x20;		/* P9 OCC poll data */
	occ->send_cmd = p9_sbe_occ_send_cmd;

	return occ_setup(occ, "p9_occ");
}

static int p9_sbe_occ_remove(struct platform_device *pdev)
{
	struct occ *occ = platform_get_drvdata(pdev);
	struct p9_sbe_occ *p9_sbe_occ = to_p9_sbe_occ(occ);

	p9_sbe_occ->sbe = NULL;
	p9_sbe_occ_close_client(p9_sbe_occ);

	occ_shutdown(occ);

	return 0;
}

static struct platform_driver p9_sbe_occ_driver = {
	.driver = {
		.name = "occ-hwmon",
	},
	.probe	= p9_sbe_occ_probe,
	.remove = p9_sbe_occ_remove,
};

module_platform_driver(p9_sbe_occ_driver);

MODULE_AUTHOR("Eddie James <eajames@us.ibm.com>");
MODULE_DESCRIPTION("BMC P9 OCC hwmon driver");
MODULE_LICENSE("GPL");
