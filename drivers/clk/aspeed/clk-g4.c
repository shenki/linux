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

#define CLK_25M_IN					(0x1 << 23)
#define CLK_25M_MASK(x)				((x >> 23) & 0x1)
#define USBCLK_48M_IN				(0x1 << 18)
#define USBCLK_48M_MASK(x)			((x >> 18) & 0x1)
#define CLK_24M_IN					0
#define CLK_48M_IN					1
#define CLK_25M_IN_24M_USB_CKI		2
#define CLK_25M_IN_48M_USB_CKI		3
static unsigned long aspeed_clk_clkin_recalc_rate(struct clk_hw *hw,
						  unsigned long parent_rate)
{
	struct aspeed_clk *clkin = to_aspeed_clk(hw);
	unsigned long rate;
	int ret;
	u32 div;
	u8 clk;

	/* SCU70: Hardware Strapping Register  */
	ret = regmap_read(clkin->map, clkin->div, &div);
	if (ret) {
		pr_err("%s: regmap read failed\n", clk_hw_get_name(hw));
		return ret;
	}
	
	clk = CLK_25M_MASK(div) | USBCLK_48M_MASK(div);
	
	switch(clk) {
		case 0:
			rate = 24 * 1000 * 1000;
			break;
		case 1:
			rate = 48 * 1000 * 1000;
			break;
		case 2:
		case 3:
			rate = 25 * 1000 * 1000;
			break;
	}

	return rate;
}

#define SCU_H_PLL_PARAMETER				(0x1 << 18)
#define SCU_H_PLL_BYPASS_EN				(0x1 << 17)
#define SCU_H_PLL_OFF					(0x1 << 16)
#define SCU_H_PLL_NUM(x)				(x << 5)
#define SCU_H_PLL_GET_NUM(x)			((x >> 5) & 0x3f)
#define SCU_H_PLL_NUM_MASK				(0x3f << 5)
#define SCU_H_PLL_OUT_DIV				(0x1 << 4)
#define SCU_H_PLL_GET_DIV(x)			((x >> 4) & 0x1)
#define SCU_H_PLL_DENUM(x)				(x)
#define SCU_H_PLL_GET_DENUM(x)			(x & 0xf)
#define SCU_H_PLL_DENUM_MASK			(0xf)

#define SCU_HW_STRAP_GET_H_PLL_CLK(x)	((x >> 8 )& 0x3)
#define SCU_HW_STRAP_H_PLL_CLK_MASK		(0x3 << 8)
#define SCU_HW_STRAP_25MHZ				(0x1 << 23)

static unsigned long aspeed_clk_hpll_recalc_rate(struct clk_hw *hw,
						 unsigned long clkin_rate)
{
	struct aspeed_clk *hpll = to_aspeed_clk(hw);
	unsigned long rate;
	int ret;
	u32 div;
	int clk;
	u32 scu70;
	int de, n, od;

	/* SCU70 */
	ret = regmap_read(hpll->map, hpll->enable, &scu70);
	if (ret) {
		pr_err("%s: regmap read failed\n", clk_hw_get_name(hw));
		return ret;
	}

	/* SCU24: HPLL */
	ret = regmap_read(hpll->map, hpll->div, &div);
	if (ret) {
		pr_err("%s: regmap read failed\n", clk_hw_get_name(hw));
		return ret;
	}

	if(div & SCU_H_PLL_OFF)
		return 0;

	if(div & SCU_H_PLL_PARAMETER) {
		if(div & SCU_H_PLL_BYPASS_EN) {
			return clkin_rate;
		} else {
			od = SCU_H_PLL_GET_DIV(div);	//OD == SCU24[4]
			n = SCU_H_PLL_GET_NUM(div);		//Numerator == SCU24[10:5]
			de = SCU_H_PLL_GET_DENUM(div);	//Denumerator == SCU24[3:0]
			//hpll = clkin * (2-OD) * ((Numerator+2)/(Denumerator+1))
			rate = ((clkin_rate * (2-od) * (n+2))/(de+1));
		}
	} else {	// HW Trap
		if(SCU_HW_STRAP_25MHZ & scu70) {
			clk = SCU_HW_STRAP_GET_H_PLL_CLK(scu70);
			switch (clk) {
				case 0:
					rate = 400 * 1000 * 1000; 
					break;
				case 1:
					rate = 375 * 1000 * 1000; 
					break;
				case 2:
					rate = 350 * 1000 * 1000; 
					break;
				case 3:
					rate = 425 * 1000 * 1000; 
					break;
				default:
					BUG(); 
					break;
			}			
		} else {
			clk = SCU_HW_STRAP_GET_H_PLL_CLK(scu70);
			switch (clk) {
				case 0:
					rate = 384 * 1000 * 1000; 
					break;
				case 1:
					rate = 360 * 1000 * 1000; 
					break;
				case 2:
					rate = 336 * 1000 * 1000; 
					break;
				case 3:
					rate = 408 * 1000 * 1000; 
					break;
				default:
					BUG(); 
					break;
			}
		}
	}

	return rate;
}

#define SCU_HW_STRAP_GET_CPU_AHB_RATIO(x)		((x >> 10) & 3)

static unsigned long aspeed_clk_ahb_recalc_rate(struct clk_hw *hw, 
						unsigned long hpll_rate)
{
	struct aspeed_clk *ahb = to_aspeed_clk(hw);
	unsigned long rate;
	int ret;
	u32 div;
	u32 ahb_div;


	/* Strap register SCU70 */
	ret = regmap_read(ahb->map, ahb->div, &div);
	if (ret) {
		pr_err("%s: regmap read failed\n", clk_hw_get_name(hw));
		return ret;
	}

	/* bits 11:10 define the CPU/AHB clock frequency ratio */
	ahb_div = SCU_HW_STRAP_GET_CPU_AHB_RATIO(div);

	rate = hpll_rate / (ahb_div + 1);

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

	/* SCU08: Clock Selection Register */
	ret = regmap_read(apb->map, apb->div, &div);
	if (ret) {
		pr_err("%s: regmap read failed\n", clk_hw_get_name(hw));
		return ret;
	}

	apb_div = SCU_GET_PCLK_DIV(div);
	apb_div = (apb_div+1) << 1;

	rate = hpll_rate / apb_div;

	return rate;
}

#define SCU_M_PLL_BYPASS				(0x1 << 17)
#define SCU_M_PLL_OFF					(0x1 << 16)
#define SCU_M_PLL_NUM(x)				(x << 5)
#define SCU_M_PLL_GET_NUM(x)			((x >> 5) & 0x3f)
#define SCU_M_PLL_NUM_MASK				(0x3f << 5)
#define SCU_M_PLL_OUT_DIV				(0x1 << 4)
#define SCU_M_PLL_GET_DIV(x)			((x >> 4) & 0x1)
#define SCU_M_PLL_DENUM(x)				(x)
#define SCU_M_PLL_GET_DENUM(x)			(x & 0xf)

static unsigned long aspeed_clk_mpll_recalc_rate(struct clk_hw *hw,
						unsigned long clkin_rate)
{
	struct aspeed_clk *mpll = to_aspeed_clk(hw);
	unsigned long rate;
	int ret;
	u32 div;
	int od, n, de;

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

	od = SCU_M_PLL_GET_DIV(div);	//OD == SCU24[4]
	n = SCU_M_PLL_GET_NUM(div);		//Numerator == SCU24[10:5]
	de = SCU_M_PLL_GET_DENUM(div);	//Denumerator == SCU24[3:0]
	
	//mpll = clkin * (2-OD) * ((n+2)/(de+1))
	rate = (clkin_rate * (2-od) * ((n+2)/(de+1)));

	return rate;
}

//AST2400 default : D1 is for VGA,  D2 is for GFX

#define SCU_D2_PLL_SET_PD2(x)			(x << 19)
#define SCU_D2_PLL_GET_PD2(x)			((x >> 19)&0x7)
#define SCU_D2_PLL_PD2_MASK				(0x7 << 19)
#define SCU_D2_PLL_BYPASS				(0x1 << 18)
#define SCU_D2_PLL_OFF					(0x1 << 17)
#define SCU_D2_PLL_SET_PD(x)			(x << 15)
#define SCU_D2_PLL_GET_PD(x)			((x >> 15) &0x3)
#define SCU_D2_PLL_PD_MASK				(0x3 << 15)
#define SCU_D2_PLL_SET_OD(x)			(x << 13)
#define SCU_D2_PLL_GET_OD(x)			((x >> 13) & 0x3)
#define SCU_D2_PLL_OD_MASK				(0x3 << 13)
#define SCU_D2_PLL_SET_DENUM(x)			(x << 8)
#define SCU_D2_PLL_GET_DENUM(x)			((x >>8)&0x1f)
#define SCU_D2_PLL_DENUM_MASK			(0x1f << 8)
#define SCU_D2_PLL_SET_NNUM(x)			(x)
#define SCU_D2_PLL_GET_NNUM(x)			(x & 0xff)
#define SCU_D2_PLL_NUM_MASK				(0xff)

static unsigned long aspeed_clk_d2pll_recalc_rate(struct clk_hw *hw,
						unsigned long clkin_rate)
{
	struct aspeed_clk *d2pll = to_aspeed_clk(hw);
	unsigned long rate;
	int ret;
	u32 div;
	u32 enable;
	int n, de, od, pd, pd2;

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

	n = SCU_D2_PLL_GET_NNUM(div);
	de = SCU_D2_PLL_GET_DENUM(div);
	od = 1 << (SCU_D2_PLL_GET_OD(div) - 1);
	pd= SCU_D2_PLL_GET_PD(div) + 1;
	pd2= SCU_D2_PLL_GET_PD2(div) + 1;

	//d2pll = clkin * (Numerator * 2) / (Denumerator * OD * PD * PD2)
	rate = (clkin_rate * n * 2) / (de * od * pd * pd2);

	return rate;
}

static long aspeed_clk_d2pll_round_rate(struct clk_hw *hw,
			unsigned long drate, unsigned long *prate)
{
	//struct aspeed_clk *d2pll = to_aspeed_clk(hw);
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
	printk("TODO aspeed_clk_d2pll_set_rate \n");
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

	/* SCU1C: D2-PLL Parameter Register */
	ret = regmap_read(lhpll->map, lhpll->div, &div);
	
	if (ret) {
		pr_err("%s: regmap read failed\n", clk_hw_get_name(hw));
		return ret;
	}

	if(SCU_LHCLK_SOURCE_EN & div) {
		hlpll_div = SCU_GET_LHCLK_DIV(div);
		hlpll_div = (hlpll_div+1) << 1;
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
#define SCU_CLK_SD_MASK				(0x7 << 12)
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
	sd_div = (sd_div+1) << 1;
	rate = clkin_rate /= sd_div;

	return rate;
}

static int aspeed_sdclk_prepare(struct clk_hw *hw)
{
	struct aspeed_clk *sdclk = to_aspeed_clk(hw);
	int ret;
	u32 div;

	ret = regmap_read(sdclk->map, sdclk->div, &div);
	if (ret) {
		pr_err("%s: regmap read failed\n", clk_hw_get_name(hw));
		return ret;
	}

	// SDCLK = G4  H-PLL / 4
	ret = regmap_write(sdclk->map, sdclk->div, (div & ~SCU_CLK_SD_MASK) | SCU_CLK_SD_DIV(1));

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
		return;
	}
	
	ret = regmap_write(sdclk->map, sdclk->div, div & ~SCU_CLK_SD_EN);
	if (ret) {
		pr_err("%s: regmap read failed\n", clk_hw_get_name(hw));
		return;
	}
	return;

}

#define SCU_USB11CLK_STOP_EN		(0x1 << 9)

static int aspeed_usb11clk_enable(struct clk_hw *hw)
{
	struct aspeed_clk *usb11clk = to_aspeed_clk(hw);
	int ret;
	u32 enable;

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

static void __init aspeed_clk_clkin_init(struct device_node *node)
{
	aspeed_clk_common_init(node, &aspeed_clk_clkin_ops, 0);
}
CLK_OF_DECLARE(aspeed_clkin_clk, "aspeed,g4-clkin-clock", aspeed_clk_clkin_init);

static void __init aspeed_clk_hpll_init(struct device_node *node)
{
	aspeed_clk_common_init(node, &aspeed_clk_hpll_ops, 0);
}
CLK_OF_DECLARE(aspeed_hpll_clk, "aspeed,g4-hpll-clock", aspeed_clk_hpll_init);

static void __init aspeed_clk_apb_init(struct device_node *node)
{
	aspeed_clk_common_init(node, &aspeed_clk_apb_ops, 0);
}
CLK_OF_DECLARE(aspeed_apb_clk, "aspeed,g4-apb-clock", aspeed_clk_apb_init);

static void __init aspeed_clk_ahb_init(struct device_node *node)
{
	aspeed_clk_common_init(node, &aspeed_clk_ahb_ops, 0);
}
CLK_OF_DECLARE(aspeed_ahb_clk, "aspeed,g4-ahb-clock", aspeed_clk_ahb_init);
static void __init aspeed_clk_mpll_init(struct device_node *node)
{
	aspeed_clk_common_init(node, &aspeed_clk_mpll_ops, 0);
}
CLK_OF_DECLARE(aspeed_mpll_clk, "aspeed,g4-mpll-clock", aspeed_clk_mpll_init);
static void __init aspeed_clk_d2pll_init(struct device_node *node)
{
	aspeed_clk_common_init(node, &aspeed_clk_d2pll_ops, 0);
}
CLK_OF_DECLARE(aspeed_d2pll_clk, "aspeed,g4-d2pll-clock", aspeed_clk_d2pll_init);
static void __init aspeed_clk_lhpll_init(struct device_node *node)
{
	aspeed_clk_common_init(node, &aspeed_clk_lhpll_ops, 0);
}
CLK_OF_DECLARE(aspeed_lhpll_clk, "aspeed,g4-lhpll-clock", aspeed_clk_lhpll_init);
static void __init aspeed_sdclk_init(struct device_node *node)
{
	aspeed_clk_common_init(node, &aspeed_sdclk_ops, CLK_GET_RATE_NOCACHE);
}
CLK_OF_DECLARE(aspeed_sdpll_clk, "aspeed,g4-sdclock", aspeed_sdclk_init);
static void __init aspeed_usb11clk_init(struct device_node *node)
{
	aspeed_clk_common_init(node, &aspeed_usb11clk_ops, 0);
}
CLK_OF_DECLARE(aspeed_usb11clk, "aspeed,g4-usb11clock", aspeed_usb11clk_init);
static void __init aspeed_mac1_clk_init(struct device_node *node)
{
	aspeed_clk_common_init(node, &aspeed_mac1_clk_ops, 0);
}
CLK_OF_DECLARE(aspeed_mac1_clk, "aspeed,g4-mac1-clock", aspeed_mac1_clk_init);
static void __init aspeed_mac0_clk_init(struct device_node *node)
{
	aspeed_clk_common_init(node, &aspeed_mac0_clk_ops, 0);
}
CLK_OF_DECLARE(aspeed_mac0_clk, "aspeed,g4-mac0-clock", aspeed_mac0_clk_init);
static void __init aspeed_usb20p1_clk_init(struct device_node *node)
{
	aspeed_clk_common_init(node, &aspeed_usb20p1_clk_ops, 0);
}
CLK_OF_DECLARE(aspeed_usb20p1_clk, "aspeed,g4-usb20p1clock", aspeed_usb20p1_clk_init);
