/*
 * SBEFIFO FSI Client device driver
 *
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

#ifndef LINUX_FSI_SBEFIFO_H
#define LINUX_FSI_SBEFIFO_H

struct device;
struct sbefifo_client;

extern struct sbefifo_client *sbefifo_drv_open(struct device *dev,
					       unsigned long flags);
extern int sbefifo_drv_read(struct sbefifo_client *client, char *buf,
			    size_t len);
extern int sbefifo_drv_write(struct sbefifo_client *client, const char *buf,
			     size_t len);
extern void sbefifo_drv_release(struct sbefifo_client *client);

#endif /* LINUX_FSI_SBEFIFO_H */
