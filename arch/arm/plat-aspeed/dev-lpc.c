/********************************************************************************
* File Name     : linux/arch/arm/plat-aspeed/dev-lpc.c
* Author        : Ryan chen
* Description   : ASPEED LPC Controller
*
* Copyright (C) ASPEED Technology Inc.
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by the Free Software Foundation;
* either version 2 of the License, or (at your option) any later version.
* 
* This program is distributed in the hope that it will be useful,  but WITHOUT ANY WARRANTY;
* without even the implied warranty of MERCHANTABILITY or
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
* 
* You should have received a copy of the GNU General Public License
* along with this program; if not, write to the Free Software
* Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA

* History      :
*    1. 2012/11/29 ryan chen create this file
*
********************************************************************************/

#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/platform_device.h>
#include <mach/irqs.h>
#include <mach/platform.h>
#include <plat/devs.h>
#include <plat/ast-scu.h>
#include <plat/ast-lpc.h>
#include <mach/gpio.h>

/* --------------------------------------------------------------------
 *  LPC
 * -------------------------------------------------------------------- */
#if defined(CONFIG_AST_LPC) || defined(CONFIG_AST_LPC_MODULE)
static struct resource ast_lpc_resource[] = {
	[0] = {
		.start = AST_LPC_BASE,
		.end = AST_LPC_BASE + SZ_512 -1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_LPC,
		.end = IRQ_LPC,
		.flags = IORESOURCE_IRQ,
	},
};

static u64 ast_lpc_dma_mask = 0xffffffffUL;

static struct platform_device ast_lpc_device = {
	.name	= "ast-lpc",
    .id = 0,
    .dev = {
            .dma_mask = &ast_lpc_dma_mask,
            .coherent_dma_mask = 0xffffffff,
    },
	.resource = ast_lpc_resource,
	.num_resources = ARRAY_SIZE(ast_lpc_resource),
};
#ifdef AST_LPC_PLUS_BASE
static struct resource ast_lpc_plus_resource[] = {
	[0] = {
		.start = AST_LPC_PLUS_BASE,
		.end = AST_LPC_PLUS_BASE + SZ_4K,
		.flags = IORESOURCE_MEM,
	},
};

static struct platform_device ast_lpc_plus_device = {
	.name	= "ast-lpc_plus",
    .id = 0,
    .dev = {
            .dma_mask = &ast_lpc_dma_mask,
            .coherent_dma_mask = 0xffffffff,
    },
	.resource = ast_lpc_plus_resource,
	.num_resources = ARRAY_SIZE(ast_lpc_plus_resource),
};
#endif
void __init ast_add_device_lpc(void)
{
#if 0	
	//due to at init reset state is correct . 
	if(gpio_get_value(PIN_GPIOI1))
		printk("Use LPC+ Bus Access \n");
	else
		printk("Use LPC Bus Access \n");		
#endif

#ifdef CONFIG_ARCH_AST1070


#ifdef AST_LPC_PLUS_BASE	
	int cc_num;
	if(gpio_get_value(PIN_GPIOI2))
		cc_num = 2; //dual 1070
	else	
		cc_num = 1; //single 1070	

	if(ast_scu_get_lpc_plus_enable()) {
		ast_lpc_plus_info.scan_node = cc_num;
	} else {
		ast_lpc_info.lpc_bus_mode = 1;
		ast_lpc_info.scan_node = cc_num;
	}
#else
	ast_lpc_info.scan_node = 1;
#endif
	
	
#endif	//End AST1070

#ifndef CONFIG_AST_ESPI
	ast_scu_set_lpc_mode();
#endif 
	platform_device_register(&ast_lpc_device);
#ifdef AST_LPC_PLUS_BASE	
	if(ast_scu_get_lpc_plus_enable())
		platform_device_register(&ast_lpc_plus_device);
#endif
}
#else
void __init ast_add_device_lpc(void) {}
#endif
