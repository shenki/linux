/*
 * Copyright 2016 IBM Corp.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */

#ifndef _UAPI_LINUX_MBOX_HOST_H
#define _UAPI_LINUX_MBOX_HOST_H

#include <linux/ioctl.h>

#define __ASPEED_MBOX_IOCTL_MAGIC	0xb1
#define ASPEED_MBOX_IOCTL_ATN	_IOW(__ASPEED_MBOX_IOCTL_MAGIC, 0x00, uint8_t)

#endif /* _UAPI_LINUX_MBOX_HOST_H */
