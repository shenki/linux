/*
 * Copyright 2016 IBM Corp.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */

#ifndef _UAPI_LINUX_LPC_CTRL_H
#define _UAPI_LINUX_LPC_CTRL_H

#include <linux/ioctl.h>

struct lpc_mapping {
	uint32_t hostaddr;
	uint32_t size;
};

#define __LPC_CTRL_IOCTL_MAGIC	0xb2
#define LPC_CTRL_IOCTL_SIZE	_IOR(__LPC_CTRL_IOCTL_MAGIC, 0x00, uint32_t)
#define LPC_CTRL_IOCTL_MAP	_IOW(__LPC_CTRL_IOCTL_MAGIC, 0x01, struct lpc_mapping)
#define LPC_CTRL_IOCTL_UNMAP	_IO(__LPC_CTRL_IOCTL_MAGIC, 0x02)

#endif /* _UAPI_LINUX_LPC_CTRL_H */
