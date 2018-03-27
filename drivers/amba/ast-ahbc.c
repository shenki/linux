/*
 * ast-ahbc.c - AHBC driver for the Aspeed SoC
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
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of.h>
#include <asm/io.h>
#include <mach/ast-ahbc.h>
/***********************  Registers for AHBC ***************************/
#define AST_AHBC_PROTECT		0x00	/* Protection Key Register */
#define AST_AHBC_PRIORITY_CTRL	0x80	/* Priority Cortrol Register */
#define AST_AHBC_ADDR_REMAP		0x8C	/* Address Remapping Register */

/* AST_AHBC_PROTECT	0x00		Protection Key Register 	*/
#define AHBC_PROTECT_UNLOCK		0xAEED1A03

/* AST_AHBC_ADDR_REMAP	0x8C		Address Remapping Register */
#define AHBC_PCI_REMAP1			(1 << 5)
#define AHBC_PCI_REMAP0			(1 << 4)

//support soc-g5
#define AHBC_PCIE_MAP			(1 << 5)
#define AHBC_LPC_PLUS_MAP		(1 << 4)
//support for old version
#define AHBC_BOOT_REMAP			1
/**************************************************************/
//#define AST_AHBC_DEBUG

#ifdef AST_AHBC_DEBUG
#define AHBCDBUG(fmt, args...) printf("%s() " fmt, __FUNCTION__, ## args)
#else
#define AHBCDBUG(fmt, args...)
#endif
/**************************************************************/
void __iomem	*ast_ahbc_base = 0;

static inline u32 
ast_ahbc_read(u32 reg)
{
	u32 val;
	val = readl(ast_ahbc_base + reg);
	AHBCDBUG("reg = 0x%08x, val = 0x%08x\n", reg, val);
	
	return val;
}

static inline void
ast_ahbc_write(u32 val, u32 reg) 
{
	AHBCDBUG("reg = 0x%08x, val = 0x%08x\n", reg, val);

#ifdef CONFIG_AST_AHBC_LOCK
	//unlock 
	writel(AHBC_PROTECT_UNLOCK, ast_ahbc_base);
	writel(val, ast_ahbc_base + reg);
	//lock
	writel(0xaa,ast_ahbc_base);	
#else
	writel(AHBC_PROTECT_UNLOCK, ast_ahbc_base);
	writel(val, ast_ahbc_base + reg);
#endif

}
//***********************************Information ***********************************
//old soc have remap
extern void ast_ahbc_boot_remap(void)
{
	ast_ahbc_write(ast_ahbc_read(AST_AHBC_ADDR_REMAP) | AHBC_BOOT_REMAP, AST_AHBC_ADDR_REMAP);	
}

//only support ast-g5
extern void ast_ahbc_peie_mapping(u8 enable)
{
	if(enable)
		ast_ahbc_write(ast_ahbc_read(AST_AHBC_ADDR_REMAP) | AHBC_PCIE_MAP, AST_AHBC_ADDR_REMAP);	
	else
		ast_ahbc_write(ast_ahbc_read(AST_AHBC_ADDR_REMAP) & ~AHBC_PCIE_MAP, AST_AHBC_ADDR_REMAP);	
}

//only support ast-g5
extern void ast_ahbc_lpc_plus_mapping(u8 enable)
{
	if(enable)
		ast_ahbc_write(ast_ahbc_read(AST_AHBC_ADDR_REMAP) | AHBC_LPC_PLUS_MAP, AST_AHBC_ADDR_REMAP);	
	else
		ast_ahbc_write(ast_ahbc_read(AST_AHBC_ADDR_REMAP) & ~AHBC_LPC_PLUS_MAP, AST_AHBC_ADDR_REMAP);	
}

static int ast_ahbc_probe(struct platform_device *pdev)
{
	struct resource *res;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	ast_ahbc_base = devm_ioremap_resource(&pdev->dev, res);

	return 0;
}

static const struct of_device_id ast_ahbc_of_match[] = {
	{ .compatible = "aspeed,ast-ahbc", },
	{ }
};

static struct platform_driver ast_ahbc_driver = {
	.probe = ast_ahbc_probe,
	.driver = {
		.name = "ast-ahbc",
		.of_match_table = ast_ahbc_of_match,
	},
};

static int ast_ahbc_init(void)
{
	return platform_driver_register(&ast_ahbc_driver);
}

subsys_initcall(ast_ahbc_init);

MODULE_AUTHOR("Ryan Chen");
MODULE_DESCRIPTION("ASPEED AHBC Driver");
MODULE_LICENSE("GPL");
