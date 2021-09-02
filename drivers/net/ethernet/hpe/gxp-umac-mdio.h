/* SPDX-License-Identifier: GPL-2.0
 * Copyright (C) 2019 Hewlett-Packard Development Company, L.P.
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _UMAC_MDIO_H_
#define _UMAC_MDIO_H_

#define UMAC_MII                0x00  //  R/W MII Register
#define UMAC_MII_NMRST          0x00008000
#define UMAC_MII_PHY_ADDR_MASK  0x001F0000
#define UMAC_MII_PHY_ADDR_SHIFT 16
#define UMAC_MII_MOWNER         0x00000200
#define UMAC_MII_MRNW           0x00000100
#define UMAC_MII_REG_ADDR_MASK  0x0000001F
#define UMAC_MII_DATA           0x04  //  R/W MII Data Register
#define UMAC_MII_LINK           0x08  //  R/W Link Register
#define UMAC_MII_CFG            0x0C  //  R/W LED Register

#endif
