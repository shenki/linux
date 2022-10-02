// SPDX-License-Identifier: GPL-2.0
/*
 * Nuvoton NPCM8xx Clock Generator
 * All the clocks are initialized by the bootloader, so this driver allows only
 * reading of current settings directly from the hardware.
 *
 * Copyright (C) 2020 Nuvoton Technologies
 * Author: Tomer Maimon <tomer.maimon@nuvoton.com>
 */

#include <linux/bitfield.h>
#include <linux/clk-provider.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include <dt-bindings/clock/nuvoton,npcm845-clk.h>

#define NPCM8XX_REF_CLK		25000000

struct npcm8xx_clk_pll_data {
	unsigned int	id;
	const char	*name;
	u32		reg;
	unsigned long	flags;
};

struct npcm8xx_clk_pll {
	struct clk_hw hw;
	void __iomem *pllcon;
	struct npcm8xx_clk_pll_data pll;
	struct clk_init_data init;
};

#define to_npcm8xx_clk_pll(_hw) container_of(_hw, struct npcm8xx_clk_pll, hw)

#define PLLCON_LOKI	BIT(31)
#define PLLCON_LOKS	BIT(30)
#define PLLCON_FBDV	GENMASK(27, 16)
#define PLLCON_OTDV2	GENMASK(15, 13)
#define PLLCON_PWDEN	BIT(12)
#define PLLCON_OTDV1	GENMASK(10, 8)
#define PLLCON_INDV	GENMASK(5, 0)

static unsigned long npcm8xx_clk_pll_recalc_rate(struct clk_hw *hw,
						 unsigned long parent_rate)
{
	struct npcm8xx_clk_pll *pll = to_npcm8xx_clk_pll(hw);
	unsigned long fbdv, indv, otdv1, otdv2;
	unsigned int val;
	u64 ret;

	if (parent_rate == 0) {
		pr_debug("%s: parent rate is zero\n", __func__);
		return 0;
	}

	val = readl_relaxed(pll->pllcon + pll->pll.reg);

	indv = FIELD_GET(PLLCON_INDV, val);
	fbdv = FIELD_GET(PLLCON_FBDV, val);
	otdv1 = FIELD_GET(PLLCON_OTDV1, val);
	otdv2 = FIELD_GET(PLLCON_OTDV2, val);

	ret = (u64)parent_rate * fbdv;
	do_div(ret, indv * otdv1 * otdv2);

	return ret;
}

static const struct clk_ops npcm8xx_clk_pll_ops = {
	.recalc_rate = npcm8xx_clk_pll_recalc_rate,
};

static struct clk_hw *
npcm8xx_clk_register_pll(struct device *dev, struct npcm8xx_clk_pll *pll,
			 void __iomem *sys_base)
{
	struct clk_hw *hw;
	int ret;

	pll->pllcon = sys_base;
	hw = &pll->hw;

	ret = devm_clk_hw_register(dev, hw);
	if (ret)
		return ERR_PTR(ret);

	return hw;
}

#define NPCM8XX_CLKEN1          (0x00)
#define NPCM8XX_CLKEN2          (0x28)
#define NPCM8XX_CLKEN3          (0x30)
#define NPCM8XX_CLKEN4          (0x70)
#define NPCM8XX_CLKSEL          (0x04)
#define NPCM8XX_CLKDIV1         (0x08)
#define NPCM8XX_CLKDIV2         (0x2C)
#define NPCM8XX_CLKDIV3         (0x58)
#define NPCM8XX_CLKDIV4         (0x7C)
#define NPCM8XX_PLLCON0         (0x0C)
#define NPCM8XX_PLLCON1         (0x10)
#define NPCM8XX_PLLCON2         (0x54)
#define NPCM8XX_SWRSTR          (0x14)
#define NPCM8XX_IRQWAKECON      (0x18)
#define NPCM8XX_IRQWAKEFLAG     (0x1C)
#define NPCM8XX_IPSRST1         (0x20)
#define NPCM8XX_IPSRST2         (0x24)
#define NPCM8XX_IPSRST3         (0x34)
#define NPCM8XX_WD0RCR          (0x38)
#define NPCM8XX_WD1RCR          (0x3C)
#define NPCM8XX_WD2RCR          (0x40)
#define NPCM8XX_SWRSTC1         (0x44)
#define NPCM8XX_SWRSTC2         (0x48)
#define NPCM8XX_SWRSTC3         (0x4C)
#define NPCM8XX_SWRSTC4         (0x50)
#define NPCM8XX_CORSTC          (0x5C)
#define NPCM8XX_PLLCONG         (0x60)
#define NPCM8XX_AHBCKFI         (0x64)
#define NPCM8XX_SECCNT          (0x68)
#define NPCM8XX_CNTR25M         (0x6C)
#define NPCM8XX_THRTL_CNT       (0xC0)

struct npcm8xx_clk_mux_data {
	u8 shift;
	u8 mask;
	u32 *table;
	const char *name;
	const char * const *parent_names;
	u8 num_parents;
	unsigned long flags;
	/*
	 * If this clock is exported via DT, set onecell_idx to constant
	 * defined in include/dt-bindings/clock/nuvoton, NPCM8XX-clock.h for
	 * this specific clock.  Otherwise, set to -1.
	 */
	int onecell_idx;
};

struct npcm8xx_clk_div_data {
	u32 reg;
	u8 shift;
	u8 width;
	const char *name;
	const char *parent_name;
	u8 clk_divider_flags;
	unsigned long flags;
	/*
	 * If this clock is exported via DT, set onecell_idx to constant
	 * defined in include/dt-bindings/clock/nuvoton, NPCM8XX-clock.h for
	 * this specific clock.  Otherwise, set to -1.
	 */
	int onecell_idx;
};

/*
 * Single copy of strings used to refer to clocks within this driver indexed by
 * above enum.
 */
#define NPCM8XX_CLK_S_REFCLK      "refclk"
#define NPCM8XX_CLK_S_SYSBYPCK    "sysbypck"
#define NPCM8XX_CLK_S_MCBYPCK     "mcbypck"
#define NPCM8XX_CLK_S_PLL0        "pll0"
#define NPCM8XX_CLK_S_PLL1        "pll1"
#define NPCM8XX_CLK_S_PLL1_DIV2   "pll1_div2"
#define NPCM8XX_CLK_S_PLL2        "pll2"
#define NPCM8XX_CLK_S_PLL_GFX     "pll_gfx"
#define NPCM8XX_CLK_S_PLL2_DIV2   "pll2_div2"
#define NPCM8XX_CLK_S_PIX_MUX     "gfx_pixel"
#define NPCM8XX_CLK_S_MC_MUX      "mc_phy"
#define NPCM8XX_CLK_S_CPU_MUX     "cpu"  /* AKA system clock */
#define NPCM8XX_CLK_S_MC          "mc"
#define NPCM8XX_CLK_S_AXI         "axi"  /* AKA CLK2 */
#define NPCM8XX_CLK_S_AHB         "ahb"  /* AKA CLK4 */
#define NPCM8XX_CLK_S_CLKOUT_MUX  "clkout_mux"
#define NPCM8XX_CLK_S_UART_MUX    "uart_mux"
#define NPCM8XX_CLK_S_SD_MUX      "sd_mux"
#define NPCM8XX_CLK_S_GFXM_MUX    "gfxm_mux"
#define NPCM8XX_CLK_S_SU_MUX      "serial_usb_mux"
#define NPCM8XX_CLK_S_DVC_MUX     "dvc_mux"
#define NPCM8XX_CLK_S_GFX_MUX     "gfx_mux"
#define NPCM8XX_CLK_S_ADC_MUX     "adc_mux"
#define NPCM8XX_CLK_S_SPI0        "spi0"
#define NPCM8XX_CLK_S_SPI1        "spi1"
#define NPCM8XX_CLK_S_SPI3        "spi3"
#define NPCM8XX_CLK_S_SPIX        "spix"
#define NPCM8XX_CLK_S_APB1        "apb1"
#define NPCM8XX_CLK_S_APB2        "apb2"
#define NPCM8XX_CLK_S_APB3        "apb3"
#define NPCM8XX_CLK_S_APB4        "apb4"
#define NPCM8XX_CLK_S_APB5        "apb5"
#define NPCM8XX_CLK_S_APB19       "apb19"
#define NPCM8XX_CLK_S_TOCK        "tock"
#define NPCM8XX_CLK_S_CLKOUT      "clkout"
#define NPCM8XX_CLK_S_PRE_ADC     "pre adc"
#define NPCM8XX_CLK_S_UART        "uart"
#define NPCM8XX_CLK_S_UART2       "uart2"
#define NPCM8XX_CLK_S_TIMER       "timer"
#define NPCM8XX_CLK_S_MMC         "mmc"
#define NPCM8XX_CLK_S_SDHC        "sdhc"
#define NPCM8XX_CLK_S_ADC         "adc"
#define NPCM8XX_CLK_S_GFX         "gfx0_gfx1_mem"
#define NPCM8XX_CLK_S_USBIF       "serial_usbif"
#define NPCM8XX_CLK_S_USB_HOST    "usb_host"
#define NPCM8XX_CLK_S_USB_BRIDGE  "usb_bridge"
#define NPCM8XX_CLK_S_PCI         "pci"
#define NPCM8XX_CLK_S_TH          "th"
#define NPCM8XX_CLK_S_ATB         "atb"
#define NPCM8XX_CLK_S_PRE_CLK     "pre_clk"

#define NPCM8XX_CLK_S_RG_MUX	  "rg_mux"
#define NPCM8XX_CLK_S_RCP_MUX	  "rcp_mux"
#define NPCM8XX_CLK_S_RG	  "rg"
#define NPCM8XX_CLK_S_RCP	  "rcp"

static u32 pll_mux_table[] = {0, 1, 2, 3};
static const char * const pll_mux_parents[] = {
	NPCM8XX_CLK_S_PLL0,
	NPCM8XX_CLK_S_PLL1,
	NPCM8XX_CLK_S_REFCLK,
	NPCM8XX_CLK_S_PLL2_DIV2,
};

static u32 cpuck_mux_table[] = {0, 1, 2, 3, 7};
static const char * const cpuck_mux_parents[] = {
	NPCM8XX_CLK_S_PLL0,
	NPCM8XX_CLK_S_PLL1,
	NPCM8XX_CLK_S_REFCLK,
	NPCM8XX_CLK_S_SYSBYPCK,
	NPCM8XX_CLK_S_PLL2,
};

static u32 pixcksel_mux_table[] = {0, 2};
static const char * const pixcksel_mux_parents[] = {
	NPCM8XX_CLK_S_PLL_GFX,
	NPCM8XX_CLK_S_REFCLK,
};

static u32 sucksel_mux_table[] = {2, 3};
static const char * const sucksel_mux_parents[] = {
	NPCM8XX_CLK_S_REFCLK,
	NPCM8XX_CLK_S_PLL2_DIV2,
};

static u32 mccksel_mux_table[] = {0, 2, 3};
static const char * const mccksel_mux_parents[] = {
	NPCM8XX_CLK_S_PLL1_DIV2,
	NPCM8XX_CLK_S_REFCLK,
	NPCM8XX_CLK_S_MCBYPCK,
};

static u32 clkoutsel_mux_table[] = {0, 1, 2, 3, 4};
static const char * const clkoutsel_mux_parents[] = {
	NPCM8XX_CLK_S_PLL0,
	NPCM8XX_CLK_S_PLL1,
	NPCM8XX_CLK_S_REFCLK,
	NPCM8XX_CLK_S_PLL_GFX, // divided by 2
	NPCM8XX_CLK_S_PLL2_DIV2,
};

static u32 gfxmsel_mux_table[] = {2, 3};
static const char * const gfxmsel_mux_parents[] = {
	NPCM8XX_CLK_S_REFCLK,
	NPCM8XX_CLK_S_PLL2_DIV2,
};

static u32 dvcssel_mux_table[] = {2, 3};
static const char * const dvcssel_mux_parents[] = {
	NPCM8XX_CLK_S_REFCLK,
	NPCM8XX_CLK_S_PLL2,
};

#define CLK_PLL(_reg, _name, _parent, _flags, _id) {			\
		.pll.id = _id,						\
		.pll.name = _name,					\
		.pll.reg = _reg,					\
		.hw.init = CLK_HW_INIT_PARENTS_DATA(_name, _parent,	\
						    &npcm8xx_clk_pll_ops,	\
						    _flags),		\
	}

static struct clk_parent_data npcm8xx_pll_parent[] = {
	{ .fw_name = NPCM8XX_CLK_S_REFCLK, .name = NPCM8XX_CLK_S_REFCLK },
};

static struct npcm8xx_clk_pll npcm8xx_pll_clks[] = {
	CLK_PLL(NPCM8XX_PLLCON0, NPCM8XX_CLK_S_PLL0, npcm8xx_pll_parent, 0, -1),
	CLK_PLL(NPCM8XX_PLLCON1, NPCM8XX_CLK_S_PLL1, npcm8xx_pll_parent, 0, -1),
	CLK_PLL(NPCM8XX_PLLCON2, NPCM8XX_CLK_S_PLL2, npcm8xx_pll_parent, 0, -1),
	CLK_PLL(NPCM8XX_PLLCONG, NPCM8XX_CLK_S_PLL_GFX, npcm8xx_pll_parent, 0, -1),
};

static const struct npcm8xx_clk_mux_data npcm8xx_muxes[] = {
	{ 0, GENMASK(1, 0), cpuck_mux_table, NPCM8XX_CLK_S_CPU_MUX,
	cpuck_mux_parents, ARRAY_SIZE(cpuck_mux_parents), CLK_IS_CRITICAL,
	NPCM8XX_CLK_CPU },

	{ 4, GENMASK(1, 0), pixcksel_mux_table, NPCM8XX_CLK_S_PIX_MUX,
	pixcksel_mux_parents, ARRAY_SIZE(pixcksel_mux_parents), 0,
	NPCM8XX_CLK_GFX_PIXEL },

	{ 6, GENMASK(1, 0), pll_mux_table, NPCM8XX_CLK_S_SD_MUX,
	pll_mux_parents, ARRAY_SIZE(pll_mux_parents), 0, -1 },

	{ 8, GENMASK(1, 0), pll_mux_table, NPCM8XX_CLK_S_UART_MUX,
	pll_mux_parents, ARRAY_SIZE(pll_mux_parents), 0, -1 },

	{ 10, GENMASK(1, 0), sucksel_mux_table, NPCM8XX_CLK_S_SU_MUX,
	sucksel_mux_parents, ARRAY_SIZE(sucksel_mux_parents), 0, -1 },

	{ 12, GENMASK(1, 0), mccksel_mux_table, NPCM8XX_CLK_S_MC_MUX,
	mccksel_mux_parents, ARRAY_SIZE(mccksel_mux_parents), 0, -1 },

	{ 14, GENMASK(1, 0), pll_mux_table, NPCM8XX_CLK_S_ADC_MUX,
	pll_mux_parents, ARRAY_SIZE(pll_mux_parents), 0, -1 },

	{ 16, GENMASK(1, 0), pll_mux_table, NPCM8XX_CLK_S_GFX_MUX,
	pll_mux_parents, ARRAY_SIZE(pll_mux_parents), 0, -1 },

	{ 18, GENMASK(2, 0), clkoutsel_mux_table, NPCM8XX_CLK_S_CLKOUT_MUX,
	clkoutsel_mux_parents, ARRAY_SIZE(clkoutsel_mux_parents), 0, -1 },

	{ 21, GENMASK(1, 0), gfxmsel_mux_table, NPCM8XX_CLK_S_GFXM_MUX,
	gfxmsel_mux_parents, ARRAY_SIZE(gfxmsel_mux_parents), 0, -1 },

	{ 23, GENMASK(1, 0), dvcssel_mux_table, NPCM8XX_CLK_S_DVC_MUX,
	dvcssel_mux_parents, ARRAY_SIZE(dvcssel_mux_parents), 0, -1 },

	{ 25, GENMASK(1, 0), pll_mux_table, NPCM8XX_CLK_S_RG_MUX,
	pll_mux_parents, ARRAY_SIZE(pll_mux_parents), 0, -1 },

	{ 27, GENMASK(1, 0), pll_mux_table, NPCM8XX_CLK_S_RCP_MUX,
	pll_mux_parents, ARRAY_SIZE(pll_mux_parents), 0, -1 },
};

/* configurable dividers: */
static const struct npcm8xx_clk_div_data npcm8xx_divs[] = {
	{ NPCM8XX_CLKDIV1, 28, 3, NPCM8XX_CLK_S_ADC, NPCM8XX_CLK_S_PRE_ADC,
		CLK_DIVIDER_READ_ONLY | CLK_DIVIDER_POWER_OF_TWO, 0,
		NPCM8XX_CLK_ADC },
	/* bit 30-28 ADCCKDIV*/
	{ NPCM8XX_CLKDIV1, 26, 2, NPCM8XX_CLK_S_AHB, NPCM8XX_CLK_S_PRE_CLK,
		CLK_DIVIDER_READ_ONLY, CLK_IS_CRITICAL, NPCM8XX_CLK_AHB },
	/* bit 28-26 CLK4DIV*/
	{ NPCM8XX_CLKDIV1, 21, 5, NPCM8XX_CLK_S_PRE_ADC,
	NPCM8XX_CLK_S_ADC_MUX, CLK_DIVIDER_READ_ONLY, 0, NPCM8XX_CLK_PRE_ADC },
	/* bit 25-21 PRE-ADCCKDIV*/
	{ NPCM8XX_CLKDIV1, 16, 5, NPCM8XX_CLK_S_UART,
	NPCM8XX_CLK_S_UART_MUX, CLK_DIVIDER_READ_ONLY, 0, NPCM8XX_CLK_UART },
	/* bit 20-16 UARTDIV*/
	{ NPCM8XX_CLKDIV1, 11, 5, NPCM8XX_CLK_S_MMC,
	NPCM8XX_CLK_S_SD_MUX, CLK_DIVIDER_READ_ONLY, 0, NPCM8XX_CLK_MMC },
	/* bit 15-11 MMCCKDIV*/
	{ NPCM8XX_CLKDIV1, 6, 5, NPCM8XX_CLK_S_SPI3,
	NPCM8XX_CLK_S_AHB, 0, 0, NPCM8XX_CLK_SPI3 },
	/* bit 10-6 AHB3CKDIV*/
	{ NPCM8XX_CLKDIV1, 2, 4, NPCM8XX_CLK_S_PCI,
	NPCM8XX_CLK_S_GFX_MUX, CLK_DIVIDER_READ_ONLY, 0, NPCM8XX_CLK_PCI },
	/* bit 5-2 PCICKDIV*/

	{ NPCM8XX_CLKDIV2, 30, 2, NPCM8XX_CLK_S_APB4, NPCM8XX_CLK_S_AHB,
		CLK_DIVIDER_READ_ONLY | CLK_DIVIDER_POWER_OF_TWO, 0,
		NPCM8XX_CLK_APB4 },
	/* bit 31-30 APB4CKDIV*/
	{ NPCM8XX_CLKDIV2, 28, 2, NPCM8XX_CLK_S_APB3, NPCM8XX_CLK_S_AHB,
		CLK_DIVIDER_READ_ONLY | CLK_DIVIDER_POWER_OF_TWO, 0,
		NPCM8XX_CLK_APB3 },
	/* bit 29-28 APB3CKDIV*/
	{ NPCM8XX_CLKDIV2, 26, 2, NPCM8XX_CLK_S_APB2, NPCM8XX_CLK_S_AHB,
		CLK_DIVIDER_READ_ONLY | CLK_DIVIDER_POWER_OF_TWO, 0,
		NPCM8XX_CLK_APB2 },
	/* bit 28-26 APB2CKDIV*/
	{ NPCM8XX_CLKDIV2, 24, 2, NPCM8XX_CLK_S_APB1, NPCM8XX_CLK_S_AHB,
		CLK_DIVIDER_READ_ONLY | CLK_DIVIDER_POWER_OF_TWO, 0,
		NPCM8XX_CLK_APB1 },
	/* bit 25-24 APB1CKDIV*/
	{ NPCM8XX_CLKDIV2, 22, 2, NPCM8XX_CLK_S_APB5, NPCM8XX_CLK_S_AHB,
		CLK_DIVIDER_READ_ONLY | CLK_DIVIDER_POWER_OF_TWO, 0,
		NPCM8XX_CLK_APB5 },
	/* bit 23-22 APB5CKDIV*/
	{ NPCM8XX_CLKDIV2, 16, 5, NPCM8XX_CLK_S_CLKOUT, NPCM8XX_CLK_S_CLKOUT_MUX,
		 CLK_DIVIDER_READ_ONLY, 0, NPCM8XX_CLK_CLKOUT },
	/* bit 20-16 CLKOUTDIV*/
	{ NPCM8XX_CLKDIV2, 13, 3, NPCM8XX_CLK_S_GFX, NPCM8XX_CLK_S_GFX_MUX,
		CLK_DIVIDER_READ_ONLY, 0, NPCM8XX_CLK_GFX },
	/* bit 15-13 GFXCKDIV*/
	{ NPCM8XX_CLKDIV2, 8, 5, NPCM8XX_CLK_S_USB_BRIDGE, NPCM8XX_CLK_S_SU_MUX,
		CLK_DIVIDER_READ_ONLY, 0, NPCM8XX_CLK_SU },
	/* bit 12-8 SUCKDIV*/
	{ NPCM8XX_CLKDIV2, 4, 4, NPCM8XX_CLK_S_USB_HOST, NPCM8XX_CLK_S_SU_MUX,
		CLK_DIVIDER_READ_ONLY, 0, NPCM8XX_CLK_SU48 },
	/* bit 8-4 SU48CKDIV*/
	{ NPCM8XX_CLKDIV2, 0, 4, NPCM8XX_CLK_S_SDHC,
	NPCM8XX_CLK_S_SD_MUX, CLK_DIVIDER_READ_ONLY, 0, NPCM8XX_CLK_SDHC }
	,/* bit 3-0 SD1CKDIV*/

	{ NPCM8XX_CLKDIV3, 16, 8, NPCM8XX_CLK_S_SPI1,
	NPCM8XX_CLK_S_AHB, CLK_DIVIDER_READ_ONLY, 0, NPCM8XX_CLK_SPI1 },
	/* bit 23-16 SPI1CKDV*/
	{ NPCM8XX_CLKDIV3, 11, 5, NPCM8XX_CLK_S_UART2,
	NPCM8XX_CLK_S_UART_MUX, CLK_DIVIDER_READ_ONLY, 0, NPCM8XX_CLK_UART2 },
	/* bit 15-11 UARTDIV2*/
	{ NPCM8XX_CLKDIV3, 6, 5, NPCM8XX_CLK_S_SPI0,
	NPCM8XX_CLK_S_AHB, CLK_DIVIDER_READ_ONLY, 0, NPCM8XX_CLK_SPI0 },
	/* bit 10-6 SPI0CKDV*/
	{ NPCM8XX_CLKDIV3, 1, 5, NPCM8XX_CLK_S_SPIX,
	NPCM8XX_CLK_S_AHB, CLK_DIVIDER_READ_ONLY, 0, NPCM8XX_CLK_SPIX },
	/* bit 5-1 SPIXCKDV*/

	{ NPCM8XX_CLKDIV4, 28, 4, NPCM8XX_CLK_S_RG, NPCM8XX_CLK_S_RG_MUX,
	CLK_DIVIDER_READ_ONLY, 0, NPCM8XX_CLK_RG },
	/* bit 31-28 RGREFDIV*/
	{ NPCM8XX_CLKDIV4, 12, 4, NPCM8XX_CLK_S_RCP, NPCM8XX_CLK_S_RCP_MUX,
	CLK_DIVIDER_READ_ONLY, 0, NPCM8XX_CLK_RCP },
	/* bit 15-12 RCPREFDIV*/
	{ NPCM8XX_THRTL_CNT, 0, 2, NPCM8XX_CLK_S_TH, NPCM8XX_CLK_S_CPU_MUX,
	CLK_DIVIDER_READ_ONLY | CLK_DIVIDER_POWER_OF_TWO, 0, NPCM8XX_CLK_TH },
	/* bit 1-0 TH_DIV*/
};

static DEFINE_SPINLOCK(npcm8xx_clk_lock);

static int npcm8xx_clk_probe(struct platform_device *pdev)
{
	struct clk_hw_onecell_data *npcm8xx_clk_data;
	struct device *dev = &pdev->dev;
	void __iomem *clk_base;
	struct resource *res;
	struct clk_hw *hw;
	unsigned int i;
	int err;

	npcm8xx_clk_data = devm_kzalloc(dev, struct_size(npcm8xx_clk_data, hws,
							 NPCM8XX_NUM_CLOCKS),
					GFP_KERNEL);
	if (!npcm8xx_clk_data)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	clk_base = devm_ioremap(dev, res->start, resource_size(res));
	if (!clk_base) {
		dev_err(&pdev->dev, "Failed to remap I/O memory\n");
		return -ENOMEM;
	}

	npcm8xx_clk_data->num = NPCM8XX_NUM_CLOCKS;

	for (i = 0; i < NPCM8XX_NUM_CLOCKS; i++)
		npcm8xx_clk_data->hws[i] = ERR_PTR(-EPROBE_DEFER);

	/* Reference 25MHz clock */
	hw = clk_hw_register_fixed_rate(dev, "refclk", NULL, 0, NPCM8XX_REF_CLK);
	if (IS_ERR(hw))
		return PTR_ERR(hw);
	npcm8xx_clk_data->hws[NPCM8XX_CLK_REFCLK] = hw;

	/* Register plls */
	for (i = 0; i < ARRAY_SIZE(npcm8xx_pll_clks); i++) {
		struct npcm8xx_clk_pll *pll_clk = &npcm8xx_pll_clks[i];

		hw = npcm8xx_clk_register_pll(dev, pll_clk, clk_base);
		if (IS_ERR(hw)) {
			dev_err(dev, "npcm8xx_clk: Can't register pll\n");
			goto unregister_refclk;
		}
	}

	/* Register fixed dividers */
	hw = devm_clk_hw_register_fixed_factor(dev, NPCM8XX_CLK_S_PLL1_DIV2,
					       NPCM8XX_CLK_S_PLL1, 0, 1, 2);
	if (IS_ERR(hw)) {
		dev_err(dev, "npcm8xx_clk: Can't register fixed div\n");
		goto unregister_refclk;
	}

	hw = devm_clk_hw_register_fixed_factor(dev, NPCM8XX_CLK_S_PLL2_DIV2,
					       NPCM8XX_CLK_S_PLL2, 0, 1, 2);
	if (IS_ERR(hw)) {
		dev_err(dev, "npcm8xx_clk: Can't register pll div2\n");
		goto unregister_refclk;
	}

	hw = devm_clk_hw_register_fixed_factor(dev, NPCM8XX_CLK_S_PRE_CLK,
					       NPCM8XX_CLK_S_CPU_MUX, 0, 1, 2);
	if (IS_ERR(hw)) {
		dev_err(dev, "npcm8xx_clk: Can't register ckclk div2\n");
		goto unregister_refclk;
	}

	hw = devm_clk_hw_register_fixed_factor(dev, NPCM8XX_CLK_S_AXI,
					       NPCM8XX_CLK_S_TH, 0, 1, 2);
	if (IS_ERR(hw)) {
		dev_err(dev, "npcm8xx_clk: Can't register axi div2\n");
		goto unregister_refclk;
	}

	hw = devm_clk_hw_register_fixed_factor(dev, NPCM8XX_CLK_S_ATB,
					       NPCM8XX_CLK_S_AXI, 0, 1, 2);
	if (IS_ERR(hw)) {
		dev_err(dev, "npcm8xx_clk: Can't register atb div2\n");
		goto unregister_refclk;
	}

	/* Register clock dividers specified in npcm8xx_divs */
	for (i = 0; i < ARRAY_SIZE(npcm8xx_divs); i++) {
		const struct npcm8xx_clk_div_data *div_data = &npcm8xx_divs[i];

		hw = devm_clk_hw_register_divider(dev, div_data->name,
						  div_data->parent_name,
						  div_data->flags,
						  clk_base + div_data->reg,
						  div_data->shift,
						  div_data->width,
						  div_data->clk_divider_flags,
						  &npcm8xx_clk_lock);
		if (IS_ERR(hw)) {
			dev_err(dev, "npcm8xx_clk: Can't register div table\n");
			goto unregister_refclk;
		}

		if (div_data->onecell_idx >= 0)
			npcm8xx_clk_data->hws[div_data->onecell_idx] = hw;
	}

	/* Register muxes */
	for (i = 0; i < ARRAY_SIZE(npcm8xx_muxes); i++) {
		const struct npcm8xx_clk_mux_data *mux_data = &npcm8xx_muxes[i];

		hw = clk_hw_register_mux_table(dev, mux_data->name,
					       mux_data->parent_names,
					       mux_data->num_parents,
					       mux_data->flags,
					       clk_base + NPCM8XX_CLKSEL,
					       mux_data->shift,
					       mux_data->mask, 0,
					       mux_data->table,
					       &npcm8xx_clk_lock);

		if (IS_ERR(hw)) {
			dev_err(dev, "npcm8xx_clk: Can't register mux\n");
			goto err_mux_clk;
		}

		if (mux_data->onecell_idx >= 0)
			npcm8xx_clk_data->hws[mux_data->onecell_idx] = hw;
	}

	err = devm_of_clk_add_hw_provider(dev, of_clk_hw_onecell_get,
					  npcm8xx_clk_data);
	if (err) {
		dev_err(dev, "unable to add clk provider\n");
		hw = ERR_PTR(err);
		goto err_mux_clk;
	}

	return err;

err_mux_clk:
	while (i--) {
		if (npcm8xx_muxes[i].onecell_idx >= 0)
			clk_hw_unregister_mux(npcm8xx_clk_data->hws[npcm8xx_muxes[i].onecell_idx]);
	}
unregister_refclk:
	clk_hw_unregister(npcm8xx_clk_data->hws[NPCM8XX_CLK_REFCLK]);
	return PTR_ERR(hw);
}

static const struct of_device_id npcm8xx_clk_dt_ids[] = {
	{ .compatible = "nuvoton,npcm845-clk", },
	{ }
};

static struct platform_driver npcm8xx_clk_driver = {
	.probe  = npcm8xx_clk_probe,
	.driver = {
		.name = "npcm8xx_clk",
		.of_match_table = npcm8xx_clk_dt_ids,
	},
};

static int __init npcm8xx_clk_driver_init(void)
{
	return platform_driver_register(&npcm8xx_clk_driver);
}
arch_initcall(npcm8xx_clk_driver_init);

