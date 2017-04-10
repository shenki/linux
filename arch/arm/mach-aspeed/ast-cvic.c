/*
 *  ast-cvic.c
 *
 *
 *  Copyright (C) 2012-2020  ASPEED Technology Inc.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 *  History:
 *    2016.12.26: Initial version [Ryan Chen]
 */

#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <asm/io.h>
#include <mach/irqs.h>
#include <mach/hardware.h>

//#define AST_CVIC_DEBUG

#ifdef AST_CVIC_DEBUG
#define CVICDUG(fmt, args...) printk("%s() " fmt, __FUNCTION__, ## args)
#else
#define CVICDUG(fmt, args...)
#endif

void __iomem		*ast_cvic_reg_base;

/* *****************************************************************************/
void ast_trigger_coldfire(void) {
	writel((0x1 << 1), ast_cvic_reg_base + 0x18);
}

EXPORT_SYMBOL(ast_trigger_coldfire);

static int __init ast_cvic_init(void)
{

	ast_cvic_reg_base = ioremap(AST_CVIC_BASE, SZ_128);
	if (!ast_cvic_reg_base) {
		printk("ast_cvic_init error !\n");
		return 1;
	}
	return 0;
}
arch_initcall(ast_cvic_init);
