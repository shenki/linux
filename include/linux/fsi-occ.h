/*
 * Copyright (C) IBM Corporation 2017
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

#ifndef LINUX_FSI_OCC_H
#define LINUX_FSI_OCC_H

struct device;
struct occ_client;

#define OCC_RESP_CMD_IN_PRG		0xFF
#define OCC_RESP_SUCCESS		0
#define OCC_RESP_CMD_INVAL		0x11
#define OCC_RESP_CMD_LEN_INVAL		0x12
#define OCC_RESP_DATA_INVAL		0x13
#define OCC_RESP_CHKSUM_ERR		0x14
#define OCC_RESP_INT_ERR		0x15
#define OCC_RESP_BAD_STATE		0x16
#define OCC_RESP_CRIT_EXCEPT		0xE0
#define OCC_RESP_CRIT_INIT		0xE1
#define OCC_RESP_CRIT_WATCHDOG		0xE2
#define OCC_RESP_CRIT_OCB		0xE3
#define OCC_RESP_CRIT_HW		0xE4

extern struct occ_client *occ_drv_open(struct device *dev,
				       unsigned long flags);
extern int occ_drv_read(struct occ_client *client, char *buf, size_t len);
extern int occ_drv_write(struct occ_client *client, const char *buf,
			 size_t len);
extern void occ_drv_release(struct occ_client *client);

#endif /* LINUX_FSI_OCC_H */
