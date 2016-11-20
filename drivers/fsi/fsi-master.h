/*
 * FSI master definitions. These comprise the core <--> master interface,
 * to allow the core to interact with the (hardware-specific) masters.
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

#ifndef DRIVERS_FSI_MASTER_H
#define DRIVERS_FSI_MASTER_H

#include <linux/device.h>

struct fsi_master {
	struct list_head my_slaves;
	bool		slave_list;
	struct device	*dev;
	int		idx;
	int		n_links;
	int		(*read)(struct fsi_master *, int link,
				uint8_t slave, uint32_t addr,
				void *val, size_t size);
	int		(*write)(struct fsi_master *, int link,
				uint8_t slave, uint32_t addr,
				const void *val, size_t size);
	int		(*send_break)(struct fsi_master *, int link);
	int		(*link_enable)(struct fsi_master *, int link);
};

extern int fsi_master_register(struct fsi_master *master);
extern void fsi_master_unregister(struct fsi_master *master);

/**
 * crc4 helper: Given a starting crc4 state @c, calculate the crc4 vaue of @x,
 * which is @bits in length. This may be required by master implementations
 * that do not provide their own hardware checksums.
 *
 * The crc4 is performed on 4-bit chunks (which is all we need for FSI
 * calculations). Typically, we'll want a starting state of 0:
 *
 *  c = fsi_crc4(0, msg, len);
 *
 * To crc4 a message that includes a single start bit, initialise crc4 state
 * with:
 *
 *  c = fsi_crc4(0, 1, 1);
 *
 * Then update with message data:
 *
 *  c = fsi_crc4(c, msg, len);
 */
uint8_t fsi_crc4(uint8_t c, uint64_t x, int bits);

#endif /* DRIVERS_FSI_MASTER_H */
