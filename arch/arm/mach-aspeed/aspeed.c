/*
 * Copyright IBM Corporation 2016
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/init.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/io.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include "ast2400.h"

static void __init aspeed_dt_init(void)
{
	of_platform_populate(NULL, of_default_bus_match_table, NULL, NULL);
}

#define AST_IO_VA	0xf0000000
#define AST_IO_PA	0x1e600000
#define AST_IO_SZ	0x00200000

#define AST_IO(__pa)	((void __iomem *)(((__pa) & 0x001fffff) | AST_IO_VA))

static struct map_desc aspeed_io_desc[] __initdata __maybe_unused = {
	{
		.virtual	=  AST_IO_VA,
		.pfn		= __phys_to_pfn(AST_IO_PA),
		.length		= AST_IO_SZ,
		.type		= MT_DEVICE
	},
};

#define SCU_PASSWORD	0x1688A8A8

static void __init aspeed_init_early(void)
{
	u32 reg;

	/*
	 * Unlock SCU
	 */
	writel(SCU_PASSWORD, AST_IO(AST_BASE_SCU));

	/* We enable the UART clock divisor in the SCU's misc control
	 * register, as the baud rates in aspeed.dtb all assume that the
	 * divisor is active
	 */
	reg = readl(AST_IO(AST_BASE_SCU | 0x2c));
	writel(reg | 0x00001000, AST_IO(AST_BASE_SCU | 0x2c));

	/*
	 * Disable the watchdogs
	 */
	writel(0, AST_IO(AST_BASE_WDT | 0x0c));
	writel(0, AST_IO(AST_BASE_WDT | 0x2c));
}

static void __init aspeed_map_io(void)
{
	debug_ll_io_init();
	iotable_init(aspeed_io_desc, ARRAY_SIZE(aspeed_io_desc));
}

static const char *const aspeed_dt_match[] __initconst = {
	"aspeed,ast2400",
	NULL,
};

DT_MACHINE_START(aspeed_dt, "ASpeed SoC")
	.map_io		= aspeed_map_io,
	.init_early	= aspeed_init_early,
	.init_machine	= aspeed_dt_init,
	.dt_compat	= aspeed_dt_match,
MACHINE_END
