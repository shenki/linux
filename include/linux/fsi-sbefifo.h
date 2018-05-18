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

int sbefifo_submit(struct device *dev, const __be32 *command, size_t cmd_len,
		   __be32 *response, size_t *resp_len);

int sbefifo_parse_status(u16 cmd, __be32 *response, size_t resp_len, size_t *data_len);

#endif /* LINUX_FSI_SBEFIFO_H */
