/*
 * ast-ahbc.h - ahhc herader
 *
 * Copyright (C) ASPEED Technology Inc.
 * Ryan Chen <ryan_chen@aspeedtech.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 *
 */
#ifndef __AST_AHBC_H_INCLUDED
#define __AST_AHBC_H_INCLUDED
//only version soc
extern void ast_ahbc_boot_remap(void);
//ast-g5 soc
extern void ast_ahbc_lpc_plus_mapping(u8 enable);
extern void ast_ahbc_peie_mapping(u8 enable);
#endif
