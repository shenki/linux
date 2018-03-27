/*
 * Copyright 2017 IBM Corporation
 * Copyright ASPEED Technology Inc.
 * Ryan Chen <ryan_chen@aspeedtech.com>
 * 
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */

#include <linux/clk-provider.h>
#include <linux/io.h>
#include <linux/of_address.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>

#include "clk-aspeed.h"
#define AST_CLK_DEBUG

#ifdef AST_CLK_DEBUG
//#define CLK_DBUG(fmt, args...) printk(KERN_DEBUG "%s() " fmt, __FUNCTION__, ## args)
#define CLK_DBUG(fmt, args...) printk( "%s() " fmt, __FUNCTION__, ## args)

#else
#define CLK_DBUG(fmt, args...)
#endif

#define CLK_25M_IN	(0x1 << 23)
static unsigned long aspeed_clk_clkin_recalc_rate(struct clk_hw *hw,
						  unsigned long parent_rate)
{
	struct aspeed_clk *clkin = to_aspeed_clk(hw);
	unsigned long rate;
	int ret;
	u32 div;

	/* SCU70: Hardware Strapping Register  */
	ret = regmap_read(clkin->map, clkin->div, &div);
	if (ret) {
		pr_err("%s: regmap read failed\n", clk_hw_get_name(hw));
		return ret;
	}

	if (div & CLK_25M_IN)
		rate = 25 * 1000 * 1000;
	else
		rate = 24 * 1000 * 1000;

	return rate;
}

#define SCU_H_PLL_RESET			(0x1 << 21)
#define SCU_H_PLL_BYPASS_EN		(0x1 << 20)
#define SCU_H_PLL_OFF			(0x1 << 19)

#define SCU_H_PLL_GET_PNUM(x)	((x >> 13) & 0x3f)	//P = SCU24[18:13]
#define SCU_H_PLL_GET_MNUM(x)	((x >> 5) & 0xff)	//M = SCU24[12:5]
#define SCU_H_PLL_GET_NNUM(x)	(x & 0xf)			//N = SCU24[4:0]

static unsigned long aspeed_clk_hpll_recalc_rate(struct clk_hw *hw,
						 unsigned long clkin_rate)
{
	struct aspeed_clk *hpll = to_aspeed_clk(hw);
	unsigned long rate;
	int ret;
	u32 div;
	int p, m, n;

	/* SCU24: H-PLL Parameter Register */
	ret = regmap_read(hpll->map, hpll->div, &div);
	if (ret) {
		pr_err("%s: regmap read failed\n", clk_hw_get_name(hw));
		return ret;
	}

	if(div & SCU_H_PLL_OFF)
		return 0;

	if(div & SCU_H_PLL_BYPASS_EN)
		return clkin_rate;

	p = SCU_H_PLL_GET_PNUM(div); 
	m = SCU_H_PLL_GET_MNUM(div);
	n = SCU_H_PLL_GET_NNUM(div);

	//hpll = clkin * [(M+1) /(N+1)] / (P+1)
	rate = clkin_rate * ((m + 1) / (n + 1)) / (p + 1);

	return rate;
}

#define SCU_HW_STRAP_GET_AXI_AHB_RATIO(x)	((x >> 9) & 0x7)

static unsigned long aspeed_clk_ahb_recalc_rate(struct clk_hw *hw,
						unsigned long hpll_rate)
{
	struct aspeed_clk *ahb = to_aspeed_clk(hw);
	unsigned long rate;
	int ret;
	u32 div;
	u32 axi_div, ahb_div;
	

	/* Strap register SCU70 */
	ret = regmap_read(ahb->map, ahb->div, &div);
	if (ret) {
		pr_err("%s: regmap read failed\n", clk_hw_get_name(hw));
		return ret;
	}

	//AST2500 fix axi_div = 2
	axi_div = 2;
	ahb_div = (SCU_HW_STRAP_GET_AXI_AHB_RATIO(div) + 1); 

	rate = (hpll_rate / axi_div) / ahb_div;

	return rate;
}

#define SCU_PCLK_APB_DIV(x)			(x << 23)
#define SCU_GET_PCLK_DIV(x)			((x >> 23) & 0x7)
#define SCU_PCLK_APB_DIV_MASK		(0x7 << 23)		//limitation on PCLK .. PCLK > 0.5*LCLK (33Mhz)

static unsigned long aspeed_clk_apb_recalc_rate(struct clk_hw *hw,
						unsigned long hpll_rate)
{
	struct aspeed_clk *apb = to_aspeed_clk(hw);
	unsigned long rate;
	int ret;
	u32 div;
	u32 apb_div;

	/* Clock selection register SCU08 */
	ret = regmap_read(apb->map, apb->div, &div);
	if (ret) {
		pr_err("%s: regmap read failed\n", clk_hw_get_name(hw));
		return ret;
	}

	apb_div = SCU_GET_PCLK_DIV(div);
	apb_div = (apb_div+1) << 2;

	rate = hpll_rate / apb_div;

	return rate;
}

#define SCU_M_PLL_RESET					(0x1 << 21)
#define SCU_M_PLL_BYPASS				(0x1 << 20)
#define SCU_M_PLL_OFF					(0x1 << 19)

#define SCU_M_PLL_GET_PNUM(x)			((x >> 13) & 0x3f)	//P  == SCU20[13:18]
#define SCU_M_PLL_GET_MNUM(x)			((x >> 5) & 0xff)	//M  == SCU20[5:12] 
#define SCU_M_PLL_GET_NNUM(x)			(x & 0x1f)			//N  == SCU20[0:4]

static unsigned long aspeed_clk_mpll_recalc_rate(struct clk_hw *hw,
						unsigned long clkin_rate)
{
	struct aspeed_clk *mpll = to_aspeed_clk(hw);
	unsigned long rate;
	int ret;
	u32 div;
	int p, m, n;

	/* SCU20: M-PLL Parameter Register */
	ret = regmap_read(mpll->map, mpll->div, &div);
	if (ret) {
		pr_err("%s: regmap read failed\n", clk_hw_get_name(hw));
		return ret;
	}

	if(div & SCU_M_PLL_OFF)
		return 0;

	if(div & SCU_M_PLL_BYPASS)
		return clkin_rate;

	p = SCU_M_PLL_GET_PNUM(div); 
	m = SCU_M_PLL_GET_MNUM(div);
	n = SCU_M_PLL_GET_NNUM(div);

	//mpll = clkin * [(M+1) /(N+1)] / (P+1)
	rate = clkin_rate * ((m + 1) / (n + 1)) / (p + 1);

	return rate;
}

/*	AST_SCU_D_PLL : 0x28 - D-PLL Parameter  register	*/
#define SCU_D_PLL_GET_SIP(x)			((x >>27) & 0x1f)
#define SCU_D_PLL_GET_SIC(x)			((x >>22) & 0x1f)
#define SCU_D_PLL_GET_ODNUM(x)			((x >>19) & 0x7)
#define SCU_D_PLL_GET_PNUM(x)			((x >>13) & 0x3f)
#define SCU_D_PLL_GET_NNUM(x)			((x >>8) & 0x1f)
#define SCU_D_PLL_GET_MNUM(x)			(x & 0xff)

/*	AST_SCU_D_PLL_EXTEND : 0x130 - D-PLL Extended Parameter  register	*/
#define SCU_D_PLL_SET_MODE(x)			((x & 0x3) << 3)
#define SCU_D_PLL_RESET					(0x1 << 2)
#define SCU_D_PLL_BYPASS				(0x1 << 1)
#define SCU_D_PLL_OFF					(0x1)

static unsigned long aspeed_clk_dpll_recalc_rate(struct clk_hw *hw,
						unsigned long clkin_rate)
{
	struct aspeed_clk *dpll = to_aspeed_clk(hw);
	unsigned long rate;
	int ret;
	u32 div;
	u32 enable;
	int p, m, n, od;

	/* SCU28: D-PLL Parameter Register */
	ret = regmap_read(dpll->map, dpll->div, &div);
	if (ret) {
		pr_err("%s: regmap read failed\n", clk_hw_get_name(hw));
		return ret;
	}

	/* SCU130: D-PLL Parameter Register */
	ret = regmap_read(dpll->map, dpll->enable, &enable);
	if (ret) {
		pr_err("%s: regmap read failed\n", clk_hw_get_name(hw));
		return ret;
	}

	if(enable & SCU_D_PLL_OFF)
		return 0;

	if(enable & SCU_D_PLL_BYPASS)
		return clkin_rate;

	m = SCU_D_PLL_GET_MNUM(div);
	n = SCU_D_PLL_GET_NNUM(div);
	p = SCU_D_PLL_GET_PNUM(div);
	od = SCU_D_PLL_GET_ODNUM(div);

	//dpll = clkin * [(M + 1) /(N + 1)] / (P + 1) / (OD + 1)
	rate = (clkin_rate * (m + 1)) / (n + 1) / (p + 1) / (od + 1);

	return rate;
}

#define SCU_D2_PLL_SET_ODNUM(x)		(x << 19)
#define SCU_D2_PLL_GET_ODNUM(x)		((x >> 19) & 0x3)
#define SCU_D2_PLL_OD_MASK				(0x3 << 19)
#define SCU_D2_PLL_SET_PNUM(x)			(x << 13)
#define SCU_D2_PLL_GET_PNUM(x)			((x >>13)&0x3f)
#define SCU_D2_PLL_PNUM_MASK			(0x3f << 13)
#define SCU_D2_PLL_SET_NNUM(x)			(x << 8)
#define SCU_D2_PLL_GET_NNUM(x)			((x >>8)&0x1f)
#define SCU_D2_PLL_NNUM_MASK			(0x1f << 8)
#define SCU_D2_PLL_SET_MNUM(x)			(x)
#define SCU_D2_PLL_GET_MNUM(x)			(x & 0xff)
#define SCU_D2_PLL_MNUM_MASK			(0xff)

/*	AST_SCU_D2_PLL_EXTEND: 0x13C - D2-PLL Extender Parameter  register */
#define SCU_D2_PLL_PARAMETER0(x)		((x) << 5)
#define SCU_D2_PLL_SET_MODE(x)			((x) << 3)
#define SCU_D2_PLL_GET_MODE(x)			(((x) >> 3) & 0x3)
#define SCU_D2_PLL_RESET				(0x1 << 2)
#define SCU_D2_PLL_BYPASS				(0x1 << 1)
#define SCU_D2_PLL_OFF					(0x1)

static unsigned long aspeed_clk_d2pll_recalc_rate(struct clk_hw *hw,
						unsigned long clkin_rate)
{
	struct aspeed_clk *d2pll = to_aspeed_clk(hw);
	unsigned long rate;
	int ret;
	u32 div;
	u32 enable;
	int p, m, n, od;

	/* SCU1C: D2-PLL Parameter Register */
	ret = regmap_read(d2pll->map, d2pll->div, &div);
	if (ret) {
		pr_err("%s: regmap read failed\n", clk_hw_get_name(hw));
		return ret;
	}

	/* SCU13C: D2-PLL Parameter Register */
	ret = regmap_read(d2pll->map, d2pll->enable, &enable);
	if (ret) {
		pr_err("%s: regmap read failed\n", clk_hw_get_name(hw));
		return ret;
	}

	if(enable & SCU_D2_PLL_OFF)
		return 0;

	if(enable & SCU_D2_PLL_BYPASS) 
		return clkin_rate;

	m = SCU_D2_PLL_GET_MNUM(div);
	n = SCU_D2_PLL_GET_NNUM(div);
	p= SCU_D2_PLL_GET_PNUM(div);
	od = SCU_D2_PLL_GET_ODNUM(div);

	//d2pll = clkin * [(M + 1) /(N + 1)] / (P + 1) / (OD + 1)
	rate = (clkin_rate * (m + 1)) / (n + 1) / (p + 1) / (od + 1);

	return rate;
}

static long aspeed_clk_d2pll_round_rate(struct clk_hw *hw,
			unsigned long drate, unsigned long *prate)
{
//	struct aspeed_clk *d2pll = to_aspeed_clk(hw);
	printk("TODO aspeed_clk_d2pll_round_rate \n");
	return 0;
}
			
static int aspeed_clk_d2pll_set_rate(struct clk_hw *hw, unsigned long rate,
							unsigned long parent_rate)
{
	struct aspeed_clk *d2pll = to_aspeed_clk(hw);
	int ret;
	u32 div;
	u32 enable;
	int p, m, n, od;
	CLK_DBUG("TODO aspeed_clk_d2pll_set_rate \n");
	/* SCU1C: D2-PLL Parameter Register */
	ret = regmap_read(d2pll->map, d2pll->div, &div);
	if (ret) {
		pr_err("%s: regmap read failed\n", clk_hw_get_name(hw));
		return ret;
	}
	
	/* SCU13C: D2-PLL Parameter Register */
	ret = regmap_read(d2pll->map, d2pll->enable, &enable);
	if (ret) {
		pr_err("%s: regmap read failed\n", clk_hw_get_name(hw));
		return ret;
	}
	printk("TODO ~~ \n");
#if 0	
	//Off D2-PLL
//	ast_scu_write(ast_scu_read(AST_SCU_D2_PLL_EXTEND) |  SCU_D2_PLL_OFF | SCU_D2_PLL_RESET , AST_SCU_D2_PLL_EXTEND);
	ast_scu_write(0x585, AST_SCU_D2_PLL_EXTEND);

	//set D2-PLL parameter 
	ast_scu_write(pll_setting, AST_SCU_D2_PLL);

	//enable D2-PLL
//	ast_scu_write(ast_scu_read(AST_SCU_D2_PLL_EXTEND) &  ~(SCU_D2_PLL_OFF | SCU_D2_PLL_RESET) , AST_SCU_D2_PLL_EXTEND);
	ast_scu_write(0x580, AST_SCU_D2_PLL_EXTEND);
#endif	
	return 0;
}

#define SCU_GET_LHCLK_DIV(x)		((x >> 20) & 0x7)
#define SCU_SET_LHCLK_DIV(x)		(x << 20)
#define SCU_LHCLK_DIV_MASK			(0x7 << 20)
#define SCU_LHCLK_SOURCE_EN			(0x1 << 19)		//0: ext , 1:internel

static unsigned long aspeed_clk_lhpll_recalc_rate(struct clk_hw *hw,
						unsigned long clkin_rate)
{
	struct aspeed_clk *lhpll = to_aspeed_clk(hw);
	unsigned long rate;
	int ret;
	u32 div;
	u32 hlpll_div;

	/* SCU1C:  Parameter Register */
	ret = regmap_read(lhpll->map, lhpll->div, &div);
	
	if (ret) {
		pr_err("%s: regmap read failed\n", clk_hw_get_name(hw));
		return ret;
	}

	if(SCU_LHCLK_SOURCE_EN & div) {
		hlpll_div = SCU_GET_LHCLK_DIV(div);
		hlpll_div = (hlpll_div+1) << 2;
		rate = (clkin_rate/hlpll_div);
	} else { 
		printk("TODO come from external source lhpll\n");
		// come from external source
		rate = 0; 
	}
		
	return rate;
}

#define SCU_CLK_SD_DIV(x)			(x << 12)
#define SCU_CLK_SD_GET_DIV(x)		((x >> 12) & 0x7)
#define SCU_CLK_SD_MASK			(0x7 << 12)
//0x08
#define SCU_SDCLK_STOP_EN			(0x1 << 27)
//0x0C
#define SCU_CLK_SD_EN				(0x1 << 15)

static unsigned long aspeed_sdclk_recalc_rate(struct clk_hw *hw,
						unsigned long clkin_rate)
{
	struct aspeed_clk *sdclk = to_aspeed_clk(hw);
	unsigned long rate;
	int ret;
	u32 div;
	u32 enable;
	u32 sd_div;

	ret = regmap_read(sdclk->map, sdclk->div, &div);
	
	if (ret) {
		pr_err("%s: regmap read failed\n", clk_hw_get_name(hw));
		return ret;
	}

	/* SCU0C: SD EN Register */
	ret = regmap_read(sdclk->map, sdclk->enable, &enable);
	if (ret) {
		pr_err("%s: regmap read failed\n", clk_hw_get_name(hw));
		return ret;
	}

	if(enable & SCU_SDCLK_STOP_EN)
		return 0;

	sd_div = SCU_CLK_SD_GET_DIV(div);
	sd_div = (sd_div+1) << 2;
	rate = clkin_rate / sd_div;

	return rate;
}

static int aspeed_sdclk_prepare(struct clk_hw *hw)
{
	struct aspeed_clk *sdclk = to_aspeed_clk(hw);
	int ret;
	u32 div;
	CLK_DBUG("aspeed_sdclk_prepare \n");

	ret = regmap_read(sdclk->map, sdclk->div, &div);
	if (ret) {
		pr_err("%s: regmap read failed\n", clk_hw_get_name(hw));
		return ret;
	}

#ifdef CONFIG_ARCH_AST3200	
	// SDCLK = H-PLL / 12
	ret = regmap_write(sdclk->map, sdclk->div, (div & ~SCU_CLK_SD_MASK) | SCU_CLK_SD_DIV(7));
#else
	// SDCLK = G4  H-PLL / 4, G5 = H-PLL /8
	ret = regmap_write(sdclk->map, sdclk->div, (div & ~SCU_CLK_SD_MASK) | SCU_CLK_SD_DIV(1));
#endif
	if (ret) {
		pr_err("%s: regmap read failed\n", clk_hw_get_name(hw));
		return ret;
	}
	return 0;	
}

static int aspeed_sdclk_enable(struct clk_hw *hw)
{
	struct aspeed_clk *sdclk = to_aspeed_clk(hw);
	int ret;
	u32 div;	
	u32 enable;

	CLK_DBUG("aspeed_sdclk_enable \n");
	/* SCU0C: SD EN Register */
	ret = regmap_read(sdclk->map, sdclk->enable, &enable);
	if (ret) {
		pr_err("%s: regmap read failed\n", clk_hw_get_name(hw));
		return ret;
	}
	
	ret = regmap_write(sdclk->map, sdclk->enable, enable & ~SCU_SDCLK_STOP_EN);
	if (ret) {
		pr_err("%s: regmap read failed\n", clk_hw_get_name(hw));
		return ret;
	}

	ret = regmap_read(sdclk->map, sdclk->div, &div);
	if (ret) {
		pr_err("%s: regmap read failed\n", clk_hw_get_name(hw));
		return ret;
	}
	
	ret = regmap_write(sdclk->map, sdclk->div, div | SCU_CLK_SD_EN);
	if (ret) {
		pr_err("%s: regmap read failed\n", clk_hw_get_name(hw));
		return ret;
	}
	
	return 0;	
}

static void aspeed_sdclk_disable(struct clk_hw *hw)
{
	struct aspeed_clk *sdclk = to_aspeed_clk(hw);
	int ret;
	u32 div;		
	u32 enable;

	/* SCU0C: SD EN Register */
	ret = regmap_read(sdclk->map, sdclk->enable, &enable);
	if (ret) {
		pr_err("%s: regmap read failed\n", clk_hw_get_name(hw));
		return;
	}
	
	ret = regmap_write(sdclk->map, sdclk->enable, enable | SCU_SDCLK_STOP_EN);
	if (ret) {
		pr_err("%s: regmap read failed\n", clk_hw_get_name(hw));
		return;
	}

	ret = regmap_read(sdclk->map, sdclk->div, &div);
	if (ret) {
		pr_err("%s: regmap read failed\n", clk_hw_get_name(hw));
		return ret;
	}
	
	ret = regmap_write(sdclk->map, sdclk->div, div & ~SCU_CLK_SD_EN);
	if (ret) {
		pr_err("%s: regmap read failed\n", clk_hw_get_name(hw));
		return ret;
	}
	return 0;
}

#define SCU_USB11CLK_STOP_EN		(0x1 << 9)

static int aspeed_usb11clk_enable(struct clk_hw *hw)
{
	struct aspeed_clk *usb11clk = to_aspeed_clk(hw);
	int ret;
	u32 enable;

	CLK_DBUG("aspeed_usb11clk_enable \n");
	/* SCU0C: USB11 EN Register */
	ret = regmap_read(usb11clk->map, usb11clk->enable, &enable);
	if (ret) {
		pr_err("%s: regmap read failed\n", clk_hw_get_name(hw));
		return ret;
	}
	
	ret = regmap_write(usb11clk->map, usb11clk->enable, enable & ~SCU_USB11CLK_STOP_EN);
	if (ret) {
		pr_err("%s: regmap read failed\n", clk_hw_get_name(hw));
		return ret;
	}
	return 0;	
}

static void aspeed_usb11clk_disable(struct clk_hw *hw)
{
	struct aspeed_clk *usb11clk = to_aspeed_clk(hw);
	int ret;
	u32 enable;

	CLK_DBUG("aspeed_usb11clk_disable \n");
	/* SCU0C: usb11clk EN Register */
	ret = regmap_read(usb11clk->map, usb11clk->enable, &enable);
	if (ret) {
		pr_err("%s: regmap read failed\n", clk_hw_get_name(hw));
		return;
	}
	
	ret = regmap_write(usb11clk->map, usb11clk->enable, enable | SCU_USB11CLK_STOP_EN);
	if (ret) {
		pr_err("%s: regmap read failed\n", clk_hw_get_name(hw));
		return;
	}
}

#define SCU_CLK_VIDEO_SLOW_EN		(0x1 << 31)

#define SCU_ECLK_SOURCE(x)			(x << 2)
#define SCU_ECLK_SOURCE_MASK		(0x3 << 2)

#define SCU_CLK_VIDEO_SLOW_SET(x)	(x << 28)
#define SCU_CLK_VIDEO_SLOW_MASK		(0x7 << 28)

static int aspeed_eclk_prepare(struct clk_hw *hw)
{
	struct aspeed_clk *eclk = to_aspeed_clk(hw);
	int ret;
	u32 div;
	CLK_DBUG("aspeed_eclk_prepare \n");

	ret = regmap_read(eclk->map, eclk->div, &div);
	if (ret) {
		pr_err("%s: regmap read failed\n", clk_hw_get_name(hw));
		return ret;
	}
	//  Enable Clock & ECLK = inverse of (M-PLL / 2)
	if(of_machine_is_compatible("aspeed,ast2500")) {
		div =div & ~(SCU_ECLK_SOURCE_MASK | SCU_CLK_VIDEO_SLOW_MASK | SCU_CLK_VIDEO_SLOW_EN);
	} else {
		div = (div & ~(SCU_ECLK_SOURCE_MASK | SCU_CLK_VIDEO_SLOW_EN)) | SCU_ECLK_SOURCE(2);
	}

	ret = regmap_write(eclk->map, eclk->div, div);
	if (ret) {
		pr_err("%s: regmap read failed\n", clk_hw_get_name(hw));
		return ret;
	}
	return 0;	
}

#define SCU_ECLK_STOP_EN			(0x1 << 0)
#define SCU_VCLK_STOP_EN			(0x1 << 3)

static int aspeed_eclk_enable(struct clk_hw *hw)
{
	struct aspeed_clk *eclk = to_aspeed_clk(hw);
	int ret;
	u32 enable;

	CLK_DBUG("aspeed_eclk_enable \n");
	/* SCU0C: ECLK EN Register */
	ret = regmap_read(eclk->map, eclk->enable, &enable);
	if (ret) {
		pr_err("%s: regmap read failed\n", clk_hw_get_name(hw));
		return ret;
	}
	
	ret = regmap_write(eclk->map, eclk->enable, enable & ~(SCU_ECLK_STOP_EN | SCU_VCLK_STOP_EN));
	if (ret) {
		pr_err("%s: regmap read failed\n", clk_hw_get_name(hw));
		return ret;
	}
	return 0;	
}

static void aspeed_eclk_disable(struct clk_hw *hw)
{
	struct aspeed_clk *eclk = to_aspeed_clk(hw);
	int ret;
	u32 enable;

	/* SCU0C: ECLK DIS Register */
	ret = regmap_read(eclk->map, eclk->enable, &enable);
	if (ret) {
		pr_err("%s: regmap read failed\n", clk_hw_get_name(hw));
		return;
	}
	
	ret = regmap_write(eclk->map, eclk->enable, enable | SCU_ECLK_STOP_EN | SCU_VCLK_STOP_EN);

	if (ret) {
		pr_err("%s: regmap read failed\n", clk_hw_get_name(hw));
		return;
	}
}

#define SCU_RSACLK_STOP_EN			(0x1 << 24)
#define SCU_YCLK_STOP_EN			(0x1 << 16)

static unsigned long aspeed_yclk_recalc_rate(struct clk_hw *hw,
						unsigned long clkin_rate)
{
	struct aspeed_clk *yclk = to_aspeed_clk(hw);
	int ret;
	u32 enable;

	ret = regmap_read(yclk->map, yclk->enable, &enable);
	
	if (ret) {
		pr_err("%s: regmap read failed\n", clk_hw_get_name(hw));
		return ret;
	}

	if(enable & SCU_YCLK_STOP_EN) 
		return 0;
	else
		return clkin_rate;
}	

static int aspeed_yclk_enable(struct clk_hw *hw)
{
	struct aspeed_clk *yclk = to_aspeed_clk(hw);
	int ret;
	u32 enable;

	ret = regmap_read(yclk->map, yclk->enable, &enable);
	
	if (ret) {
		pr_err("%s: regmap read failed\n", clk_hw_get_name(hw));
		return ret;
	}

	ret = regmap_write(yclk->map, yclk->enable, enable & ~(SCU_YCLK_STOP_EN | SCU_RSACLK_STOP_EN));
	if (ret) {
		pr_err("%s: regmap read failed\n", clk_hw_get_name(hw));
		return ret;
	}
	return 0;		
}

static void aspeed_yclk_disable(struct clk_hw *hw)
{
	struct aspeed_clk *yclk = to_aspeed_clk(hw);
	int ret;
	u32 enable;

	ret = regmap_read(yclk->map, yclk->enable, &enable);
	if (ret) {
		pr_err("%s: regmap read failed\n", clk_hw_get_name(hw));
		return;
	}

	ret = regmap_write(yclk->map, yclk->enable, enable | SCU_YCLK_STOP_EN | SCU_RSACLK_STOP_EN);
	if (ret) {
		pr_err("%s: regmap read failed\n", clk_hw_get_name(hw));
		return;
	}	
}

#define SCU_MAC1CLK_STOP_EN		(0x1 << 21)

static int aspeed_mac1_clk_enable(struct clk_hw *hw)
{
	struct aspeed_clk *mac1_clk = to_aspeed_clk(hw);
	int ret;
	u32 enable;

	ret = regmap_read(mac1_clk->map, mac1_clk->enable, &enable);
	if (ret) {
		pr_err("%s: regmap read failed\n", clk_hw_get_name(hw));
		return ret;
	}

	ret = regmap_write(mac1_clk->map, mac1_clk->enable, enable & ~SCU_MAC1CLK_STOP_EN);
	if (ret) {
		pr_err("%s: regmap read failed\n", clk_hw_get_name(hw));
		return ret;
	}
	return 0;
}

static void aspeed_mac1_clk_disable(struct clk_hw *hw)
{
	struct aspeed_clk *mac1_clk = to_aspeed_clk(hw);
	int ret;
	u32 enable;

	ret = regmap_read(mac1_clk->map, mac1_clk->enable, &enable);
	if (ret) {
		pr_err("%s: regmap read failed\n", clk_hw_get_name(hw));
		return;
	}

	ret = regmap_write(mac1_clk->map, mac1_clk->enable, enable | SCU_MAC1CLK_STOP_EN);
	if (ret) {
		pr_err("%s: regmap read failed\n", clk_hw_get_name(hw));
		return;
	}
}

#define SCU_MAC0CLK_STOP_EN		(0x1 << 20)

static int aspeed_mac0_clk_enable(struct clk_hw *hw)
{
	struct aspeed_clk *mac0_clk = to_aspeed_clk(hw);
	int ret;
	u32 enable;

	ret = regmap_read(mac0_clk->map, mac0_clk->enable, &enable);
	
	if (ret) {
		pr_err("%s: regmap read failed\n", clk_hw_get_name(hw));
		return ret;
	}

	ret = regmap_write(mac0_clk->map, mac0_clk->enable, enable & ~SCU_MAC0CLK_STOP_EN);
	if (ret) {
		pr_err("%s: regmap read failed\n", clk_hw_get_name(hw));
		return ret;
	}
	return 0;
}

static void aspeed_mac0_clk_disable(struct clk_hw *hw)
{
	struct aspeed_clk *mac0_clk = to_aspeed_clk(hw);
	int ret;
	u32 enable;

	ret = regmap_read(mac0_clk->map, mac0_clk->enable, &enable);
	if (ret) {
		pr_err("%s: regmap read failed\n", clk_hw_get_name(hw));
		return;
	}

	ret = regmap_write(mac0_clk->map, mac0_clk->enable, enable | SCU_MAC0CLK_STOP_EN);
	if (ret) {
		pr_err("%s: regmap read failed\n", clk_hw_get_name(hw));
		return;
	}
}

#define SCU_USB20_PHY_CLK_EN		(0x1 << 14)

static int aspeed_usb20p1_clk_enable(struct clk_hw *hw)
{
	struct aspeed_clk *usb20p1_clk = to_aspeed_clk(hw);
	int ret;
	u32 enable;

	CLK_DBUG("aspeed_usb20p1_clk_enable \n");

	ret = regmap_read(usb20p1_clk->map, usb20p1_clk->enable, &enable);
	
	if (ret) {
		pr_err("%s: regmap read failed\n", clk_hw_get_name(hw));
		return ret;
	}

	ret = regmap_write(usb20p1_clk->map, usb20p1_clk->enable, enable | SCU_USB20_PHY_CLK_EN);
	if (ret) {
		pr_err("%s: regmap read failed\n", clk_hw_get_name(hw));
		return ret;
	}
	return 0;
}

static void aspeed_usb20p1_clk_disable(struct clk_hw *hw)
{
	struct aspeed_clk *usb20p1_clk = to_aspeed_clk(hw);
	int ret;
	u32 enable;

	ret = regmap_read(usb20p1_clk->map, usb20p1_clk->enable, &enable);
	if (ret) {
		pr_err("%s: regmap read failed\n", clk_hw_get_name(hw));
		return;
	}

	ret = regmap_write(usb20p1_clk->map, usb20p1_clk->enable, enable & ~SCU_USB20_PHY_CLK_EN);
	if (ret) {
		pr_err("%s: regmap read failed\n", clk_hw_get_name(hw));
		return;
	}	
}

#define SCU_USB_P1_STOP_EN			(0x1 << 7)

static int aspeed_usb20p2_clk_enable(struct clk_hw *hw)
{
	struct aspeed_clk *usb20p2_clk = to_aspeed_clk(hw);
	int ret;
	u32 enable;

	CLK_DBUG("aspeed_usb20p2_clk_enable \n");

	ret = regmap_read(usb20p2_clk->map, usb20p2_clk->enable, &enable);
	
	if (ret) {
		pr_err("%s: regmap read failed\n", clk_hw_get_name(hw));
		return ret;
	}

	ret = regmap_write(usb20p2_clk->map, usb20p2_clk->enable, enable & ~SCU_USB_P1_STOP_EN);
	if (ret) {
		pr_err("%s: regmap read failed\n", clk_hw_get_name(hw));
		return ret;
	}
	return 0;
}

static void aspeed_usb20p2_clk_disable(struct clk_hw *hw)
{
	struct aspeed_clk *usb20p2_clk = to_aspeed_clk(hw);
	int ret;
	u32 enable;

	ret = regmap_read(usb20p2_clk->map, usb20p2_clk->enable, &enable);
	if (ret) {
		pr_err("%s: regmap read failed\n", clk_hw_get_name(hw));
		return;
	}

	ret = regmap_write(usb20p2_clk->map, usb20p2_clk->enable, enable | SCU_USB_P1_STOP_EN);
	if (ret) {
		pr_err("%s: regmap read failed\n", clk_hw_get_name(hw));
		return;
	}	
}

static const struct clk_ops aspeed_clk_clkin_ops = {
	.recalc_rate = aspeed_clk_clkin_recalc_rate,
};
static const struct clk_ops aspeed_clk_hpll_ops = {
	.recalc_rate = aspeed_clk_hpll_recalc_rate,
};
static const struct clk_ops aspeed_clk_apb_ops = {
	.recalc_rate = aspeed_clk_apb_recalc_rate,
};
static const struct clk_ops aspeed_clk_ahb_ops = {
	.recalc_rate = aspeed_clk_ahb_recalc_rate,
};
static const struct clk_ops aspeed_clk_mpll_ops = {
	.recalc_rate = aspeed_clk_mpll_recalc_rate,
};	
static const struct clk_ops aspeed_clk_dpll_ops = {
	.recalc_rate = aspeed_clk_dpll_recalc_rate,
};
static const struct clk_ops aspeed_clk_d2pll_ops = {
	.recalc_rate = aspeed_clk_d2pll_recalc_rate,
	.round_rate = aspeed_clk_d2pll_round_rate,
	.set_rate = aspeed_clk_d2pll_set_rate,
};
static const struct clk_ops aspeed_clk_lhpll_ops = {
	.recalc_rate = aspeed_clk_lhpll_recalc_rate,
};
static const struct clk_ops aspeed_sdclk_ops = {
	.recalc_rate = aspeed_sdclk_recalc_rate,
	.prepare = aspeed_sdclk_prepare,
	.enable = aspeed_sdclk_enable,
	.disable = aspeed_sdclk_disable,
};
static const struct clk_ops aspeed_usb11clk_ops = {
	.enable = aspeed_usb11clk_enable,
	.disable = aspeed_usb11clk_disable,
};

static const struct clk_ops aspeed_eclk_ops = {
	.prepare = aspeed_eclk_prepare,
	.enable = aspeed_eclk_enable,
	.disable = aspeed_eclk_disable,
};
	
static const struct clk_ops aspeed_yclk_ops = {
	.recalc_rate = aspeed_yclk_recalc_rate,
	.enable = aspeed_yclk_enable,
	.disable = aspeed_yclk_disable,
};

static const struct clk_ops aspeed_mac1_clk_ops = {
	.enable = aspeed_mac1_clk_enable,
	.disable = aspeed_mac1_clk_disable,
};

static const struct clk_ops aspeed_mac0_clk_ops = {
	.enable = aspeed_mac0_clk_enable,
	.disable = aspeed_mac0_clk_disable,
};

static const struct clk_ops aspeed_usb20p1_clk_ops = {
	.enable = aspeed_usb20p1_clk_enable,
	.disable = aspeed_usb20p1_clk_disable,
};

static const struct clk_ops aspeed_usb20p2_clk_ops = {
	.enable = aspeed_usb20p2_clk_enable,
	.disable = aspeed_usb20p2_clk_disable,
};

static void __init aspeed_clk_clkin_init(struct device_node *node)
{
	aspeed_clk_common_init(node, &aspeed_clk_clkin_ops, 0);
}
CLK_OF_DECLARE(aspeed_clkin_clk, "aspeed,g5-clkin-clock", aspeed_clk_clkin_init);

static void __init aspeed_clk_hpll_init(struct device_node *node)
{
	aspeed_clk_common_init(node, &aspeed_clk_hpll_ops, 0);
}
CLK_OF_DECLARE(aspeed_hpll_clk, "aspeed,g5-hpll-clock", aspeed_clk_hpll_init);

static void __init aspeed_clk_apb_init(struct device_node *node)
{
	aspeed_clk_common_init(node, &aspeed_clk_apb_ops, 0);
}
CLK_OF_DECLARE(aspeed_apb_clk, "aspeed,g5-apb-clock", aspeed_clk_apb_init);

static void __init aspeed_clk_ahb_init(struct device_node *node)
{
	aspeed_clk_common_init(node, &aspeed_clk_ahb_ops, 0);
}
CLK_OF_DECLARE(aspeed_ahb_clk, "aspeed,g5-ahb-clock", aspeed_clk_ahb_init);
static void __init aspeed_clk_mpll_init(struct device_node *node)
{
	aspeed_clk_common_init(node, &aspeed_clk_mpll_ops, 0);
}
CLK_OF_DECLARE(aspeed_mpll_clk, "aspeed,g5-mpll-clock", aspeed_clk_mpll_init);
static void __init aspeed_clk_dpll_init(struct device_node *node)
{
	aspeed_clk_common_init(node, &aspeed_clk_dpll_ops, 0);
}
CLK_OF_DECLARE(aspeed_dpll_clk, "aspeed,g5-dpll-clock", aspeed_clk_dpll_init);
static void __init aspeed_clk_d2pll_init(struct device_node *node)
{
	aspeed_clk_common_init(node, &aspeed_clk_d2pll_ops, 0);
}
CLK_OF_DECLARE(aspeed_d2pll_clk, "aspeed,g5-d2pll-clock", aspeed_clk_d2pll_init);
static void __init aspeed_clk_lhpll_init(struct device_node *node)
{
	aspeed_clk_common_init(node, &aspeed_clk_lhpll_ops, 0);
}
CLK_OF_DECLARE(aspeed_lhpll_clk, "aspeed,g5-lhpll-clock", aspeed_clk_lhpll_init);
static void __init aspeed_sdclk_init(struct device_node *node)
{
	aspeed_clk_common_init(node, &aspeed_sdclk_ops, CLK_GET_RATE_NOCACHE);
}
CLK_OF_DECLARE(aspeed_sdclk, "aspeed,g5-sdclock", aspeed_sdclk_init);
static void __init aspeed_usb11clk_init(struct device_node *node)
{
	aspeed_clk_common_init(node, &aspeed_usb11clk_ops, 0);
}
CLK_OF_DECLARE(aspeed_usb11clk, "aspeed,g5-usb11clock", aspeed_usb11clk_init);
static void __init aspeed_eclk_init(struct device_node *node)
{
	aspeed_clk_common_init(node, &aspeed_eclk_ops, 0);
}
CLK_OF_DECLARE(aspeed_eclk, "aspeed,g5-eclock", aspeed_eclk_init);
static void __init aspeed_yclk_init(struct device_node *node)
{
	aspeed_clk_common_init(node, &aspeed_yclk_ops, 0);
}
CLK_OF_DECLARE(aspeed_yclk, "aspeed,g5-yclock", aspeed_yclk_init);
static void __init aspeed_mac1_clk_init(struct device_node *node)
{
	aspeed_clk_common_init(node, &aspeed_mac1_clk_ops, 0);
}
CLK_OF_DECLARE(aspeed_mac1_clk, "aspeed,g5-mac1-clock", aspeed_mac1_clk_init);
static void __init aspeed_mac0_clk_init(struct device_node *node)
{
	aspeed_clk_common_init(node, &aspeed_mac0_clk_ops, 0);
}
CLK_OF_DECLARE(aspeed_mac0_clk, "aspeed,g5-mac0-clock", aspeed_mac0_clk_init);
static void __init aspeed_usb20p1_clk_init(struct device_node *node)
{
	aspeed_clk_common_init(node, &aspeed_usb20p1_clk_ops, 0);
}
CLK_OF_DECLARE(aspeed_usb20p1_clk, "aspeed,g5-usb20p1clock", aspeed_usb20p1_clk_init);
static void __init aspeed_usb20p2_clk_init(struct device_node *node)
{
	aspeed_clk_common_init(node, &aspeed_usb20p2_clk_ops, 0);
}
CLK_OF_DECLARE(aspeed_usb20p2_clk, "aspeed,g5-usb20p2clock", aspeed_usb20p2_clk_init);
