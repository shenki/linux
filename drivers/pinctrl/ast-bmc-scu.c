/*
 * ast-bmc-scu.c - BMC SCU driver for the Aspeed SoC
 *
 * Copyright (C) ASPEED Technology Inc.
 * Ryan Chen <ryan_chen@aspeedtech.com>
 *
 *
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/init.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <mach/aspeed.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>

#include <mach/ast-bmc-scu.h>
#include <mach/regs-bmc-scu.h>
//#include <mach/ast_i2c.h>

//#define ASPEED_SCU_LOCK
//#define AST_BMC_SCU_DBG

#ifdef AST_BMC_SCU_DBG
#define BMC_SCUDBG(fmt, args...) printk(KERN_DEBUG "%s() " fmt,__FUNCTION__, ## args)
#else
#define BMC_SCUDBG(fmt, args...)
#endif

#define BMC_SCUMSG(fmt, args...) printk(fmt, ## args)

void __iomem *ast_scu_base;

spinlock_t ast_scu_lock;

static inline u32 
ast_scu_read(u32 reg)
{
	u32 val;
		
	val = readl(ast_scu_base + reg);
	
	BMC_SCUDBG("ast_scu_read : reg = 0x%08x, val = 0x%08x\n", reg, val);
	
	return val;
}

static inline void
ast_scu_write(u32 val, u32 reg) 
{
	BMC_SCUDBG("ast_scu_write : reg = 0x%08x, val = 0x%08x\n", reg, val);
#ifdef CONFIG_AST_SCU_LOCK
	//unlock 
	writel(SCU_PROTECT_UNLOCK, ast_scu_base);
	writel(val, ast_scu_base + reg);
	//lock
	writel(0xaa,ast_scu_base);	
#else
	writel(SCU_PROTECT_UNLOCK, ast_scu_base);
	writel(val, ast_scu_base + reg);
#endif
}

//SoC mapping Table 
struct soc_id {
	const char	* name;
	u32	rev_id;
};

#define SOC_ID(str, rev) { .name = str, .rev_id = rev, }

static struct soc_id soc_map_table[] = {
	SOC_ID("AST1100/AST2050-A0", 0x00000200),
	SOC_ID("AST1100/AST2050-A1", 0x00000201),
	SOC_ID("AST1100/AST2050-A2,3/AST2150-A0,1", 0x00000202),
	SOC_ID("AST1510/AST2100-A0", 0x00000300),
	SOC_ID("AST1510/AST2100-A1", 0x00000301),
	SOC_ID("AST1510/AST2100-A2,3", 0x00000302),
	SOC_ID("AST2200-A0,1", 0x00000102),
	SOC_ID("AST2300-A0", 0x01000003),
	SOC_ID("AST2300-A1", 0x01010303),
	SOC_ID("AST1300-A1", 0x01010003),
	SOC_ID("AST1050-A1", 0x01010203),
	SOC_ID("AST2400-A0", 0x02000303),
	SOC_ID("AST2400-A1", 0x02010303),
	SOC_ID("AST1010-A0", 0x03000003),
	SOC_ID("AST1010-A1", 0x03010003),
	SOC_ID("AST3200-A0", 0x04002003),
	SOC_ID("AST3200-A1", 0x04012003),
	SOC_ID("AST3200-A2", 0x04032003),
	SOC_ID("AST1520-A0", 0x03000203),	
	SOC_ID("AST1520-A1", 0x03010203),
	SOC_ID("AST2510-A0", 0x04000103),
	SOC_ID("AST2510-A1", 0x04010103),
	SOC_ID("AST2510-A2", 0x04030103),	
	SOC_ID("AST2520-A0", 0x04000203),
	SOC_ID("AST2520-A1", 0x04010203),
	SOC_ID("AST2520-A2", 0x04030203),
	SOC_ID("AST2500-A0", 0x04000303),	
	SOC_ID("AST2500-A1", 0x04010303),
	SOC_ID("AST2500-A2", 0x04030303),	
	SOC_ID("AST2530-A0", 0x04000403),
	SOC_ID("AST2530-A1", 0x04010403),
	SOC_ID("AST2530-A2", 0x04030403),	
};
//***********************************Initial control***********************************
#ifdef SCU_RESET_VIDEO
extern void
ast_scu_reset_video(void)
{
	ast_scu_write(ast_scu_read(AST_SCU_RESET) | SCU_RESET_VIDEO, AST_SCU_RESET);
	udelay(100);
	ast_scu_write(ast_scu_read(AST_SCU_RESET) & ~SCU_RESET_VIDEO, AST_SCU_RESET);
}

EXPORT_SYMBOL(ast_scu_reset_video);

extern void
ast_scu_init_video(u8 dynamic_en)
{
	//Video Engine Clock Enable and Reset
	//  Enable Clock & ECLK = inverse of (M-PLL / 2)
	if(dynamic_en)
		ast_scu_write((ast_scu_read(AST_SCU_CLK_SEL) & ~SCU_CLK_VIDEO_SLOW_MASK) | SCU_CLK_VIDEO_SLOW_EN | SCU_CLK_VIDEO_SLOW_SET(0), AST_SCU_CLK_SEL);
	else {
		if(GET_CHIP_REVISION(ast_scu_read(AST_SCU_REVISION_ID)) == 4)
			ast_scu_write((ast_scu_read(AST_SCU_CLK_SEL) & ~(SCU_ECLK_SOURCE_MASK | SCU_CLK_VIDEO_SLOW_MASK | SCU_CLK_VIDEO_SLOW_EN)), AST_SCU_CLK_SEL);
		else
			ast_scu_write((ast_scu_read(AST_SCU_CLK_SEL) & ~(SCU_ECLK_SOURCE_MASK | SCU_CLK_VIDEO_SLOW_EN)) | SCU_ECLK_SOURCE(2), AST_SCU_CLK_SEL);
	}
	
	// Enable CLK
	ast_scu_write(ast_scu_read(AST_SCU_CLK_STOP) & ~(SCU_ECLK_STOP_EN | SCU_VCLK_STOP_EN), AST_SCU_CLK_STOP);	
	mdelay(10);
	ast_scu_write(ast_scu_read(AST_SCU_RESET) | SCU_RESET_VIDEO, AST_SCU_RESET);
	udelay(100);
	ast_scu_write(ast_scu_read(AST_SCU_RESET) & ~SCU_RESET_VIDEO, AST_SCU_RESET);
}
EXPORT_SYMBOL(ast_scu_init_video);
#endif

#ifdef SCU_UART1CLK_STOP_EN
extern void
ast_scu_init_uart(u8 uart)
{
	u32 clk_stop_en = 0;	

	//uart 1
	if(uart & 0x2) {
		clk_stop_en |= SCU_UART1CLK_STOP_EN;
	}

	if(uart & 0x4) {
		clk_stop_en |= SCU_UART2CLK_STOP_EN;
	}

	if(uart & 0x8) {
		clk_stop_en |= SCU_UART3CLK_STOP_EN;
	}

	if(uart & 0x10) {
		clk_stop_en |= SCU_UART4CLK_STOP_EN;
	}
	
	ast_scu_write(ast_scu_read(AST_SCU_CLK_STOP) & ~(clk_stop_en), AST_SCU_CLK_STOP);
	
}
EXPORT_SYMBOL(ast_scu_init_uart);
#endif


#ifdef SCU_RESET_PCIE
extern void
ast_scu_init_pcie(void)
{
	if((ast_scu_read(AST_SCU_RESET) & SCU_RESET_PCIE_DIR) && (!(ast_scu_read(AST_SCU_RESET) & SCU_RESET_PCIE))) {
		//do nothing
		//printk("No need init PCIe \n");
	} else {
		//pcie host reset	
		ast_scu_write(ast_scu_read(AST_SCU_RESET) & ~SCU_RESET_PCIE, AST_SCU_RESET);
		ast_scu_write(ast_scu_read(AST_SCU_RESET) | SCU_RESET_PCIE_DIR, AST_SCU_RESET);
		ast_scu_write(ast_scu_read(AST_SCU_RESET) | SCU_RESET_PCIE, AST_SCU_RESET);
		mdelay(10); 
		ast_scu_write(ast_scu_read(AST_SCU_RESET) & ~SCU_RESET_PCIE, AST_SCU_RESET);

		//p2x reset
		ast_scu_write(ast_scu_read(AST_SCU_RESET) & ~SCU_RESET_P2X, AST_SCU_RESET);
		
		//use 0x7c for clr 
		ast_scu_write(SCU_HW_STRAP_VGA_MASK, AST_SCU_REVISION_ID);
		ast_scu_write(SCU_HW_STRAP_VGA_SIZE_SET(VGA_64M_DRAM), AST_SCU_HW_STRAP1);
		
		ast_scu_write(ast_scu_read(AST_SCU_MISC2_CTRL) | SCU_PCIE_MAPPING_HIGH | SCU_MALI_RC_MODE | SCU_MALI_DTY_MODE, AST_SCU_MISC2_CTRL); 		
	}

}
EXPORT_SYMBOL(ast_scu_init_pcie);
#endif

extern void
ast_scu_reset_lpc(void)
{
	//Note .. It have been enable in U-boot..... 
	ast_scu_write(ast_scu_read(AST_SCU_RESET) | SCU_RESET_LPC, AST_SCU_RESET);	

	ast_scu_write(ast_scu_read(AST_SCU_RESET) & ~SCU_RESET_LPC, AST_SCU_RESET);
}

EXPORT_SYMBOL(ast_scu_reset_lpc);

//////1 : lpc plus modes
extern u8
ast_scu_get_lpc_plus_enable(void)
{
	if(ast_scu_read(AST_SCU_FUN_PIN_CTRL5) & SCU_FUN_PIN_LPC_PLUS)
		return 1;
	else 
		return 0;
}

EXPORT_SYMBOL(ast_scu_get_lpc_plus_enable);

extern void
ast_scu_init_rfx(void)
{
#if 0
	ast_scu_write(ast_scu_read(AST_SCU_RESET) | SCU_RESET_RFX, AST_SCU_RESET);
	ast_scu_write(ast_scu_read(AST_SCU_RESET2) | (SCU_RESET_RFXDEC | SCU_RESET_RFXCMQ | SCU_RESET_BITBLT), AST_SCU_RESET2);	

	//Use D1-PLL
	ast_scu_write((ast_scu_read(AST_SCU_CLK_SEL) & ~(SCU_ECLK_SOURCE_MASK | SCU_CLK_VIDEO_SLOW_MASK | SCU_CLK_VIDEO_SLOW_EN)), AST_SCU_CLK_SEL);
	ast_scu_write(ast_scu_read(AST_SCU_CLK_SEL) | SCU_ECLK_SOURCE(2), AST_SCU_CLK_SEL);
	
	ast_scu_write((ast_scu_read(AST_SCU_MISC1_CTRL) & ~SCU_MISC_D_PLL_DIS) | SCU_MISC_D_PLL_SOURCE, AST_SCU_MISC1_CTRL);

	ast_scu_write(0x75402031, AST_SCU_D_PLL);
	ast_scu_write(0x00000580, AST_SCU_DPLL_PAR0);
	ast_scu_write(0x00000000, AST_SCU_DPLL_PAR1);
	ast_scu_write(0x0004AB1C, AST_SCU_DPLL_PAR2);
	
	ast_scu_write(ast_scu_read(AST_SCU_CLK_STOP) & 	~(SCU_RFX_CLK_STOP_EN), AST_SCU_CLK_STOP);	
	ast_scu_write(ast_scu_read(AST_SCU_CLK_STOP2) & ~(SCU_CMQCLK_STOP | SCU_RFXCLK_STOP | SCU_BITBLTCLK_STOP), AST_SCU_CLK_STOP2);	
	udelay(3);
	ast_scu_write(ast_scu_read(AST_SCU_RESET) & ~SCU_RESET_RFX, AST_SCU_RESET);
	ast_scu_write(ast_scu_read(AST_SCU_RESET2) & ~(SCU_RESET_RFXDEC | SCU_RESET_RFXCMQ | SCU_RESET_BITBLT), AST_SCU_RESET2);	

	//Multi fun pin
	ast_scu_write(ast_scu_read(AST_SCU_FUN_PIN_CTRL6) | SCU_FUN_PIN_DVO_24BIT, AST_SCU_FUN_PIN_CTRL6); 
#endif	
}
EXPORT_SYMBOL(ast_scu_init_rfx);

#ifdef SCU_RESET_H264
extern void
ast_scu_init_h264(void)
{
	ast_scu_write(ast_scu_read(AST_SCU_RESET) | SCU_RESET_H264, AST_SCU_RESET);
	ast_scu_write(ast_scu_read(AST_SCU_CLK_SEL2) & 	~SCU_H264CLK_MASK, AST_SCU_CLK_SEL2);
	ast_scu_write(ast_scu_read(AST_SCU_CLK_SEL2) | SCU_SET_H264CLK_DIV(3), AST_SCU_CLK_SEL2);	
	ast_scu_write(ast_scu_read(AST_SCU_CLK_STOP) & 	~SCU_H264_STOP_EN, AST_SCU_CLK_STOP);
	udelay(3);
	ast_scu_write(ast_scu_read(AST_SCU_RESET) & ~SCU_RESET_H264, AST_SCU_RESET);
}
EXPORT_SYMBOL(ast_scu_init_h264);
#endif

/* 0:disable spi 1: enable spi master 2:enable spi master and spi slave to ahb 3: enable spi pass-through*/
extern void
ast_scu_spi_master(u8 mode)
{
#ifdef AST_SOC_G5
	switch(mode) {
		case 0:
			ast_scu_write(SCU_HW_STRAP_SPI_MODE_MASK, AST_SCU_REVISION_ID);
			break;
		case 1:
			ast_scu_write(SCU_HW_STRAP_SPI_MODE_MASK, AST_SCU_REVISION_ID);
			ast_scu_write(SCU_HW_STRAP_SPI_MASTER, AST_SCU_HW_STRAP1);
			break;
		case 2:
			ast_scu_write(SCU_HW_STRAP_SPI_MODE_MASK, AST_SCU_REVISION_ID);
			ast_scu_write(SCU_HW_STRAP_SPI_M_S_EN, AST_SCU_HW_STRAP1);
			break;
		case 3:
			ast_scu_write(SCU_HW_STRAP_SPI_MODE_MASK, AST_SCU_REVISION_ID);
			ast_scu_write(SCU_HW_STRAP_SPI_PASS_THROUGH, AST_SCU_HW_STRAP1);			
			break;
	}
#else
	switch(mode) {
		case 0:			
			ast_scu_write(ast_scu_read(AST_SCU_HW_STRAP1) & ~SCU_HW_STRAP_SPI_MODE_MASK, AST_SCU_HW_STRAP1);
			break;
		case 1:
			ast_scu_write((ast_scu_read(AST_SCU_HW_STRAP1) & ~SCU_HW_STRAP_SPI_MODE_MASK) |SCU_HW_STRAP_SPI_MASTER, AST_SCU_HW_STRAP1);
			break;
		case 2:
			ast_scu_write((ast_scu_read(AST_SCU_HW_STRAP1) & ~SCU_HW_STRAP_SPI_MODE_MASK) |SCU_HW_STRAP_SPI_MASTER, AST_SCU_HW_STRAP1);
			break;
		case 3:
			ast_scu_write((ast_scu_read(AST_SCU_HW_STRAP1) & ~SCU_HW_STRAP_SPI_MODE_MASK) |SCU_HW_STRAP_SPI_PASS_THROUGH, AST_SCU_HW_STRAP1);
			break;
	}

#endif
}

EXPORT_SYMBOL(ast_scu_spi_master);

#ifdef SCU_RESET_CRT
extern void
ast_scu_init_crt(void)
{
	//ast2400 : VGA use D1 clk, CRT use D2 clk
	//ast2500 : VGA use D1 clk, CRT use 40Mhz 
	//ast3200/ast1520 : VGA use D1 clk, CRT use D1/D2 clk select L: SCU08[bit 8] - H SCU2C[bit 21]

#ifdef AST_SOC_G5

#ifdef CONFIG_ARCH_AST3200
	//Select D2 CLK source 00:D-PLL, 01: D2-PLL, 1x : 40Mhz
	//H: 2c[bit : 21], L: 08[bit : 8]
	//Select D2-PLL parameter source [01]
	ast_scu_write(ast_scu_read(AST_SCU_CLK_SEL) | SCU_CRT_CLK_L_SOURCE , AST_SCU_CLK_SEL);
	ast_scu_write(ast_scu_read(AST_SCU_MISC1_CTRL) & ~SCU_MISC_CRT_CLK_H_SOURCE , AST_SCU_MISC1_CTRL);
	
	//Off D2-PLL
//	ast_scu_write(ast_scu_read(AST_SCU_D2_PLL_EXTEND) |  SCU_D2_PLL_OFF | SCU_D2_PLL_RESET , AST_SCU_D2_PLL_EXTEND);
	ast_scu_write(0x585, AST_SCU_D2_PLL_EXTEND);

	//set D2-PLL parameter 
	ast_scu_write((0x15 << 27) | (0xE << 22) | (0x03D << 13) | (0x40), AST_SCU_D2_PLL);

	//enable D2-PLL
//	ast_scu_write(ast_scu_read(AST_SCU_D2_PLL_EXTEND) &  ~(SCU_D2_PLL_OFF | SCU_D2_PLL_RESET) , AST_SCU_D2_PLL_EXTEND);
	ast_scu_write(0x580, AST_SCU_D2_PLL_EXTEND);
	ast_scu_write(ast_scu_read(AST_SCU_RESET) & ~SCU_RESET_CRT, AST_SCU_RESET);	

	ast_scu_write(ast_scu_read(AST_SCU_RESET2) & ~(SCU_RESET_CRT0 | SCU_RESET_CRT1 | SCU_RESET_CRT2 | SCU_RESET_CRT3), AST_SCU_RESET2);

	//For DVO output timing
	ast_scu_write((ast_scu_read(AST_SCU_CLK_SEL2) & SCU_VIDEO1_OUTPUT_CLK_DELAY_MASK) | SCU_VIDEO1_OUTPUT_CLK_DELAY(5), AST_SCU_CLK_SEL2);	
	
#else
	//ast2500 use 40Mhz (init @ platform.S)
	ast_scu_write(ast_scu_read(AST_SCU_RESET) & ~SCU_RESET_CRT, AST_SCU_RESET);	

	ast_scu_write(ast_scu_read(AST_SCU_RESET2) & ~SCU_RESET_CRT0, AST_SCU_RESET2);	

#endif
	//enable CRT CLK
	ast_scu_write(ast_scu_read(AST_SCU_CLK_STOP) & ~SCU_D1CLK_STOP , AST_SCU_CLK_STOP);	

	ast_scu_write(0x1df, 0xd4); 

#else
	//SOC VER < G5
	/* Enable D2 - PLL */
	ast_scu_write(ast_scu_read(AST_SCU_MISC1_CTRL) & ~SCU_MISC_D2_PLL_DIS, AST_SCU_MISC1_CTRL);

	/* Reset CRT */ 
	ast_scu_write(ast_scu_read(AST_SCU_RESET) | SCU_RESET_CRT, AST_SCU_RESET);
	
	/* Set Delay 5 Compensation TODO ...*/
	ast_scu_write((ast_scu_read(AST_SCU_CLK_SEL) & ~SCU_CLK_VIDEO_DELAY_MASK) |
						SCU_CLK_VIDEO_DELAY(5), AST_SCU_CLK_SEL);

	//enable D2 CLK
	ast_scu_write(ast_scu_read(AST_SCU_CLK_STOP) & ~SCU_D2CLK_STOP_EN , AST_SCU_CLK_STOP);

	udelay(10);
	ast_scu_write(ast_scu_read(AST_SCU_RESET) & ~SCU_RESET_CRT, AST_SCU_RESET);
	
#endif

	
}
EXPORT_SYMBOL(ast_scu_init_crt);
#endif

extern void
ast_scu_uartx_init(void)
{
	//for UART (6-13) enable clock
	ast_scu_write(ast_scu_read(AST_SCU_CLK_STOP2) &
				~(SCU_UART_DIV13 | SCU_UARTXCLK_STOP),
		AST_SCU_CLK_STOP2); 
	
}

EXPORT_SYMBOL(ast_scu_uartx_init);

//***********************************CLK control***********************************
extern void
ast_scu_uart_div(void)
{
	ast_scu_write(ast_scu_read(AST_SCU_CLK_STOP2) &
				~SCU_UART_DIV13,
		AST_SCU_CLK_STOP2); 
}

EXPORT_SYMBOL(ast_scu_uart_div);
//***********************************CLK Information***********************************
extern void
ast_scu_osc_clk_output(void)
{
	//in ast3200 for usb audio code clock
//	if (!(ast_scu_read(AST_SCU_HW_STRAP1) & CLK_25M_IN))
//	{
		ast_scu_write(ast_scu_read(AST_SCU_MISC1_CTRL) | SCU_MISC_OSC_CLK_OUT_PIN, AST_SCU_MISC1_CTRL);
		ast_scu_write((ast_scu_read(AST_SCU_COUNT_CTRL) & ~SCU_FREQ_SOURCE_FOR_MEASU_MASK) | SCU_FREQ_SOURCE_FOR_MEASU(SCU_FREQ_SOURCE_FOR_MEASU_12MHZ), AST_SCU_COUNT_CTRL);
//	}
}

EXPORT_SYMBOL(ast_scu_osc_clk_output);

//Because value 0 is not allowed in SDIO12C D[15:8]: Host Control Settings #1 Register, we have to increase the maximum
//host's clock in case that system will not ask host to set 1 in the sdhci_set_clock() function
/*
SCU7C: Silicon Revision ID Register
D[31:24]: Chip ID
0: AST2050/AST2100/AST2150/AST2200/AST3000
1: AST2300

D[23:16] Silicon revision ID for AST2300 generation and later
0: A0
1: A1
2: A2
.
.
.
FPGA revision starts from 0x80


D[11:8] Bounding option

D[7:0] Silicon revision ID for AST2050/AST2100 generation (for software compatible)
0: A0
1: A1
2: A2
3: A3
.
.
FPGA revision starts from 0x08, 8~10 means A0, 11+ means A1, AST2300 should be assigned to 3
*/


extern void
ast_scu_set_lpc_mode(void)
{
#ifdef AST_SOC_G5
	ast_scu_write(SCU_HW_STRAP_ESPI_MODE , AST_SCU_REVISION_ID);
#endif
}
EXPORT_SYMBOL(ast_scu_set_lpc_mode);

extern void
ast_scu_show_system_info (void)
{
#if 0
#ifdef AST_SOC_G5
	u32 axi_div, ahb_div, h_pll, pclk_div;

	h_pll = ast_get_h_pll_clk();

	//AST2500 A1 fix 
	axi_div = 2;
	ahb_div = (SCU_HW_STRAP_GET_AXI_AHB_RATIO(ast_scu_read(AST_SCU_HW_STRAP1)) + 1);
	pclk_div = (SCU_GET_PCLK_DIV(ast_scu_read(AST_SCU_CLK_SEL)) + 1) * 4;

	BMC_SCUMSG("CPU = %d MHz , AXI = %d MHz, AHB = %d MHz, PCLK = %d Mhz (div: %d:%d:%d) \n", 
			h_pll/1000000, 
			h_pll/axi_div/1000000,
			h_pll/axi_div/ahb_div/1000000,
			h_pll/pclk_div/1000000, axi_div, ahb_div, pclk_div); 

#else
	u32 h_pll, ahb_div, pclk_div;

	h_pll = ast_get_h_pll_clk();

	ahb_div = SCU_HW_STRAP_GET_CPU_AHB_RATIO(ast_scu_read(AST_SCU_HW_STRAP1));
	ahb_div += 1;
	pclk_div = (SCU_GET_PCLK_DIV(ast_scu_read(AST_SCU_CLK_SEL)) + 1) * 4;
	
	BMC_SCUMSG("CPU = %d MHz ,AHB = %d MHz, PCLK = %d Mhz (div : %d:%d) \n", 
			h_pll/1000000, h_pll/ahb_div/1000000, h_pll/ahb_div/1000000, 
			ahb_div, pclk_div); 
#endif
#endif 
	return ;
}

EXPORT_SYMBOL(ast_scu_show_system_info);

//*********************************** Multi-function pin control ***********************************
extern void
ast_scu_multi_func_uart(u8 uart)
{
	switch(uart) {
		case 1:
			ast_scu_write(ast_scu_read(AST_SCU_FUN_PIN_CTRL2) |
						SCU_FUN_PIN_UART1_RXD |
						SCU_FUN_PIN_UART1_TXD |
						SCU_FUN_PIN_UART1_NRTS |
						SCU_FUN_PIN_UART1_NDTR |
						SCU_FUN_PIN_UART1_NRI |
						SCU_FUN_PIN_UART1_NDSR |
						SCU_FUN_PIN_UART1_NDCD |
						SCU_FUN_PIN_UART1_NCTS, 
				AST_SCU_FUN_PIN_CTRL2); 
			break;		
		case 2:
			ast_scu_write(ast_scu_read(AST_SCU_FUN_PIN_CTRL2) |
						SCU_FUN_PIN_UART2_RXD |
						SCU_FUN_PIN_UART2_TXD |
						SCU_FUN_PIN_UART2_NRTS |
						SCU_FUN_PIN_UART2_NDTR |
						SCU_FUN_PIN_UART2_NRI |
						SCU_FUN_PIN_UART2_NDSR |
						SCU_FUN_PIN_UART2_NDCD |
						SCU_FUN_PIN_UART2_NCTS, 
				AST_SCU_FUN_PIN_CTRL2); 
			break;		
		case 3:
			ast_scu_write(ast_scu_read(AST_SCU_FUN_PIN_CTRL1) |
						SCU_FUN_PIN_UART3_RXD |
						SCU_FUN_PIN_UART3_TXD |
						SCU_FUN_PIN_UART3_NRTS |
						SCU_FUN_PIN_UART3_NDTR |
						SCU_FUN_PIN_UART3_NRI |
						SCU_FUN_PIN_UART3_NDSR |
						SCU_FUN_PIN_UART3_NDCD |
						SCU_FUN_PIN_UART3_NCTS, 
				AST_SCU_FUN_PIN_CTRL1); 
			break;
		case 4:
			ast_scu_write(ast_scu_read(AST_SCU_FUN_PIN_CTRL1) |
						SCU_FUN_PIN_UART4_RXD |
						SCU_FUN_PIN_UART4_TXD |
						SCU_FUN_PIN_UART4_NRTS |
						SCU_FUN_PIN_UART4_NDTR |
						SCU_FUN_PIN_UART4_NRI |
						SCU_FUN_PIN_UART4_NDSR |
						SCU_FUN_PIN_UART4_NDCD |
						SCU_FUN_PIN_UART4_NCTS, 
				AST_SCU_FUN_PIN_CTRL1); 			
			break;
		case 6:
			ast_scu_write(ast_scu_read(AST_SCU_FUN_PIN_CTRL5) |
						SCU_FUC_PIN_UART6, 
				AST_SCU_FUN_PIN_CTRL5); 
			break;
#ifdef AST_SOC_G5			
		case 7:
			ast_scu_write(ast_scu_read(AST_SCU_FUN_PIN_CTRL6) |
						SCU_FUN_PIN_UART7,
				AST_SCU_FUN_PIN_CTRL6); 
			break;
		case 8:
			ast_scu_write(ast_scu_read(AST_SCU_FUN_PIN_CTRL6) |
						SCU_FUN_PIN_UART8,
				AST_SCU_FUN_PIN_CTRL6); 
			break;
		case 9:
			ast_scu_write(ast_scu_read(AST_SCU_FUN_PIN_CTRL6) |
						SCU_FUN_PIN_UART9,
				AST_SCU_FUN_PIN_CTRL6); 
			break;
		case 10:
			ast_scu_write(ast_scu_read(AST_SCU_FUN_PIN_CTRL2) &
						~(SCU_FUN_PIN_VGAVS | SCU_FUN_PIN_VGAHS),
				AST_SCU_FUN_PIN_CTRL2); 
			
			ast_scu_write(ast_scu_read(AST_SCU_FUN_PIN_CTRL6) |
						SCU_FUN_PIN_UART10,
				AST_SCU_FUN_PIN_CTRL6); 
			break;
		case 11:
			ast_scu_write(ast_scu_read(AST_SCU_FUN_PIN_CTRL2) &
						~(SCU_FUN_PIN_DDCDAT | SCU_FUN_PIN_DDCCLK),
				AST_SCU_FUN_PIN_CTRL2); 
			
			ast_scu_write(ast_scu_read(AST_SCU_FUN_PIN_CTRL6) |
						SCU_FUN_PIN_UART11,
				AST_SCU_FUN_PIN_CTRL6); 
			break;
		case 12:
			ast_scu_write(ast_scu_read(AST_SCU_FUN_PIN_CTRL6) |
						SCU_FUN_PIN_UART12,
				AST_SCU_FUN_PIN_CTRL6); 
			break;
		case 13:
			ast_scu_write(ast_scu_read(AST_SCU_FUN_PIN_CTRL6) |
						SCU_FUN_PIN_UART13,
				AST_SCU_FUN_PIN_CTRL6); 
			break;
#endif
	}


}

extern void
ast_scu_multi_func_eth(u8 num)
{
	switch(num) {
		case 0:
			if(ast_scu_read(AST_SCU_HW_STRAP1) & SCU_HW_STRAP_MAC0_RGMII) {
				BMC_SCUMSG("MAC0 : RGMII \n");
				ast_scu_write(ast_scu_read(AST_SCU_FUN_PIN_CTRL1) | 
							SCU_FUN_PIN_MAC0_PHY_LINK, 
					AST_SCU_FUN_PIN_CTRL1); 
			} else {
				BMC_SCUMSG("MAC0 : RMII/NCSI \n");			
				ast_scu_write(ast_scu_read(AST_SCU_FUN_PIN_CTRL1) &
							~SCU_FUN_PIN_MAC0_PHY_LINK, 
					AST_SCU_FUN_PIN_CTRL1); 
			}

#ifdef AST_SOC_G5
			ast_scu_write(ast_scu_read(AST_SCU_FUN_PIN_CTRL1) | 
						SCU_FUN_PIN_MAC0_PHY_LINK, 
				AST_SCU_FUN_PIN_CTRL1); 

#endif
			ast_scu_write(ast_scu_read(AST_SCU_FUN_PIN_CTRL3) | 
						SCU_FUN_PIN_MAC0_MDIO |
						SCU_FUN_PIN_MAC0_MDC, 
				AST_SCU_FUN_PIN_CTRL3); 
			
			break;
		case 1:
			if(ast_scu_read(AST_SCU_HW_STRAP1) & SCU_HW_STRAP_MAC1_RGMII) {
				BMC_SCUMSG("MAC1 : RGMII \n");
				ast_scu_write(ast_scu_read(AST_SCU_FUN_PIN_CTRL1) | 
							SCU_FUN_PIN_MAC1_PHY_LINK, 
					AST_SCU_FUN_PIN_CTRL1); 
			} else {
				BMC_SCUMSG("MAC1 : RMII/NCSI \n");
				ast_scu_write(ast_scu_read(AST_SCU_FUN_PIN_CTRL1) & 
						~SCU_FUN_PIN_MAC1_PHY_LINK, 
					AST_SCU_FUN_PIN_CTRL1); 
			}
		
			ast_scu_write(ast_scu_read(AST_SCU_FUN_PIN_CTRL1) | 
						SCU_FUN_PIN_MAC1_PHY_LINK, 
				AST_SCU_FUN_PIN_CTRL1); 
			
			ast_scu_write(ast_scu_read(AST_SCU_FUN_PIN_CTRL5) | 
						SCU_FUC_PIN_MAC1_MDIO,
				AST_SCU_FUN_PIN_CTRL5); 

			break;
	}
}



extern void
ast_scu_multi_func_romcs(u8 num)
{
	ast_scu_write(ast_scu_read(AST_SCU_FUN_PIN_CTRL3) | 
			SCU_FUN_PIN_ROMCS(num), 
		AST_SCU_FUN_PIN_CTRL3);
}

extern void
ast_scu_multi_func_i2c(u8 bus_no)
{
	//TODO check ... //In AST2400 Due to share pin with SD , please not enable I2C 10 ~14 
	// AST 2400 have 14 , AST 2300 9 ...
	u32 pin_ctrl = ast_scu_read(AST_SCU_FUN_PIN_CTRL5);
	switch(bus_no) {
		case 0:
			break;
		case 1:
			break;
		case 2:
			pin_ctrl |= SCU_FUC_PIN_I2C3;
			break;
		case 3:
			pin_ctrl |= SCU_FUC_PIN_I2C4;			
			break;
		case 4:
			pin_ctrl |= SCU_FUC_PIN_I2C5;			
			break;
		case 5:
			pin_ctrl |= SCU_FUC_PIN_I2C6;			
			break;
		case 6:
			pin_ctrl |= SCU_FUC_PIN_I2C7;			
			break;
		case 7:
			pin_ctrl |= SCU_FUC_PIN_I2C8;			
			break;
		case 8:
			pin_ctrl |= SCU_FUC_PIN_I2C9;	
			break;
		case 9:
			pin_ctrl |= SCU_FUC_PIN_I2C10;
			pin_ctrl &= ~SCU_FUC_PIN_SD1;
			break;
		case 10:
			pin_ctrl |= SCU_FUC_PIN_I2C11;			
			pin_ctrl &= ~SCU_FUC_PIN_SD1;
			break;
		case 11:
			pin_ctrl |= SCU_FUC_PIN_I2C12;			
			pin_ctrl &= ~SCU_FUC_PIN_SD1;		
			break;
		case 12:
			pin_ctrl |= SCU_FUC_PIN_I2C13;			
			pin_ctrl &= ~SCU_FUC_PIN_SD1;
			break;
		case 13:
			pin_ctrl |= SCU_FUC_PIN_I2C14;	
			break;
	}

	ast_scu_write(pin_ctrl, AST_SCU_FUN_PIN_CTRL5);
}	

EXPORT_SYMBOL(ast_scu_multi_func_i2c);

extern void
ast_scu_multi_func_pwm_tacho(void)
{
	u32 sts = 0;
	spin_lock(&ast_scu_lock);	

	sts = ast_scu_read(AST_SCU_FUN_PIN_CTRL3) &~0xcfffff;
	ast_scu_write(sts | 0xc000ff, AST_SCU_FUN_PIN_CTRL3);
	spin_unlock(&ast_scu_lock);	

}	

EXPORT_SYMBOL(ast_scu_multi_func_pwm_tacho);

//0 : usb 2.0 hub mode, 1:usb 2.0 host2 controller
extern void
ast_scu_multi_func_usb_port1_mode(u8 mode)
{
	if(mode)
		ast_scu_write(ast_scu_read(AST_SCU_FUN_PIN_CTRL5) | SCU_FUC_PIN_USB20_HOST, 
					AST_SCU_FUN_PIN_CTRL5);
	else
		ast_scu_write(ast_scu_read(AST_SCU_FUN_PIN_CTRL5) & ~SCU_FUC_PIN_USB20_HOST, 
					AST_SCU_FUN_PIN_CTRL5);
}	

EXPORT_SYMBOL(ast_scu_multi_func_usb_port1_mode);

//0 : 1.1 hid 1, 1.1 host , 2, 2.0 host 3, 2.0 device 
extern void
ast_scu_multi_func_usb_port2_mode(u8 mode)
{
#if defined(AST_SOC_G5)
	if(mode == 0)
		ast_scu_write((ast_scu_read(AST_SCU_FUN_PIN_CTRL6) & ~SCU_FUN_PIN_USBP1_MASK), 
					AST_SCU_FUN_PIN_CTRL6);
	else if ((mode == 1) || (mode == 2))
		ast_scu_write((ast_scu_read(AST_SCU_FUN_PIN_CTRL6) & ~SCU_FUN_PIN_USBP1_MASK) |
					SCU_FUN_PIN_USBP1_MODE(0x2), 
					AST_SCU_FUN_PIN_CTRL6);
	else if (mode == 3)
		ast_scu_write((ast_scu_read(AST_SCU_FUN_PIN_CTRL6) & ~SCU_FUN_PIN_USBP1_MASK) |
					SCU_FUN_PIN_USBP1_MODE(0x1), 
					AST_SCU_FUN_PIN_CTRL6);
	else {
		printk("nothing\n");
	}
#else
	if(mode == 0)
		ast_scu_write(ast_scu_read(AST_SCU_FUN_PIN_CTRL5) & ~SCU_FUC_PIN_USB11_PORT2, 
					AST_SCU_FUN_PIN_CTRL5);
	else if ((mode == 1) || (mode == 2))
		ast_scu_write(ast_scu_read(AST_SCU_FUN_PIN_CTRL5) | SCU_FUC_PIN_USB11_PORT2 | SCU_FUC_PIN_USB20_HOST, 
			AST_SCU_FUN_PIN_CTRL5);
	else {
		printk("nothing\n");
	}
#endif	
}	

EXPORT_SYMBOL(ast_scu_multi_func_usb_port2_mode);

//0 : gpioQ6,7 mode , 1: usb1.1 host port 4 mode
extern void
ast_scu_multi_func_usb_port34_mode(u8 mode)
{
#if defined(AST_SOC_G5)
	if(mode) {
		ast_scu_write(ast_scu_read(AST_SCU_FUN_PIN_CTRL5) | SCU_FUC_PIN_USB11_PORT4, 
					AST_SCU_FUN_PIN_CTRL5);
		ast_scu_write(ast_scu_read(AST_SCU_FUN_PIN_CTRL3) | 
					(SCU_FUN_PIN_USBP3_DP |SCU_FUN_PIN_USBP3_DN | SCU_FUN_PIN_USBP4_DP | SCU_FUN_PIN_USBP4_DN),
					AST_SCU_FUN_PIN_CTRL3);		
	} else {
		ast_scu_write(ast_scu_read(AST_SCU_FUN_PIN_CTRL5) & ~SCU_FUC_PIN_USB11_PORT4, 
					AST_SCU_FUN_PIN_CTRL5);
		ast_scu_write(ast_scu_read(AST_SCU_FUN_PIN_CTRL3) & 
					~(SCU_FUN_PIN_USBP3_DP |SCU_FUN_PIN_USBP3_DN | SCU_FUN_PIN_USBP4_DP | SCU_FUN_PIN_USBP4_DN),
					AST_SCU_FUN_PIN_CTRL3);		
	}
#else
	if(mode) {
		ast_scu_write(ast_scu_read(AST_SCU_FUN_PIN_CTRL5) | SCU_FUC_PIN_USB11_PORT4, 
					AST_SCU_FUN_PIN_CTRL5);
	} else {
		ast_scu_write(ast_scu_read(AST_SCU_FUN_PIN_CTRL5) & ~SCU_FUC_PIN_USB11_PORT4, 
					AST_SCU_FUN_PIN_CTRL5);
	}
#endif
}	

EXPORT_SYMBOL(ast_scu_multi_func_usb_port34_mode);

//0 : 1: SD1 function 
extern void
ast_scu_multi_func_sdhc_8bit_mode(void)
{
		ast_scu_write(ast_scu_read(AST_SCU_FUN_PIN_CTRL5) | SCU_FUC_PIN_SD1 | SCU_FUC_PIN_SD1_8BIT, 
					AST_SCU_FUN_PIN_CTRL5);
}	

EXPORT_SYMBOL(ast_scu_multi_func_sdhc_8bit_mode);

extern void
ast_scu_multi_func_sdhc_slot(u8 slot)
{
	switch(slot) {
		case 0:
			ast_scu_write(ast_scu_read(AST_SCU_FUN_PIN_CTRL5) | SCU_FUC_PIN_SD1, 
						AST_SCU_FUN_PIN_CTRL5);						
			break;
		case 1:
			ast_scu_write(ast_scu_read(AST_SCU_FUN_PIN_CTRL5) | SCU_FUC_PIN_SD2, 
						AST_SCU_FUN_PIN_CTRL5);									
			break;
		case 3:
			ast_scu_write(ast_scu_read(AST_SCU_FUN_PIN_CTRL5) | SCU_FUC_PIN_SD1 | SCU_FUC_PIN_SD2, 
						AST_SCU_FUN_PIN_CTRL5);						
			break;			
	}
}	

EXPORT_SYMBOL(ast_scu_multi_func_sdhc_slot);

//0: VGA , 1 : CRT, 2 : PASS through Port -A, 3 : PASS through Port -B  
extern void
ast_scu_set_crt_source(u8 dac_soource)
{
	ast_scu_write((ast_scu_read(AST_SCU_MISC1_CTRL) & ~SCU_MISC_DAC_MASK) | 
				SCU_MISC_SET_DAC_SOURCE(dac_soource)  , AST_SCU_MISC1_CTRL);

}

EXPORT_SYMBOL(ast_scu_set_crt_source);

extern void
ast_scu_multi_func_crt(void)
{	
	/* multi-pin for DVO enable DVO (bit18) is VGA , enable DAC (bit16) is CRT  */
#if defined(CONFIG_AST_DAC) || defined(CONFIG_AST_DVO)
		ast_scu_write((ast_scu_read(AST_SCU_MISC1_CTRL) & ~SCU_MISC_DAC_MASK)
					| SCU_MISC_DAC_SOURCE_CRT | SCU_MISC_DVO_SOURCE_CRT | SCU_MISC_2D_CRT_EN , AST_SCU_MISC1_CTRL);
#elif defined(CONFIG_AST_DVO) 
		ast_scu_write(ast_scu_read(AST_SCU_MISC1_CTRL) | SCU_MISC_DVO_SOURCE_CRT| SCU_MISC_2D_CRT_EN, AST_SCU_MISC1_CTRL);
#else //default(CONFIG_AST_DAC) 
		ast_scu_write((ast_scu_read(AST_SCU_MISC1_CTRL) & ~SCU_MISC_DAC_MASK) 
					| SCU_MISC_DAC_SOURCE_CRT | SCU_MISC_2D_CRT_EN, AST_SCU_MISC1_CTRL);
#endif

	//Digital vodeo input function pins : 00 disable, 10 24bits mode 888,
	ast_scu_write((ast_scu_read(AST_SCU_FUN_PIN_CTRL6) &
			~SCU_FUC_PIN_DIGI_V_OUT_MASK) |
			SCU_FUC_PIN_DIGI_V_OUT(VIDEO_24BITS), AST_SCU_FUN_PIN_CTRL6);
}

EXPORT_SYMBOL(ast_scu_multi_func_crt);


extern void
ast_scu_multi_func_sgpio(void)
{
	ast_scu_write(ast_scu_read(AST_SCU_FUN_PIN_CTRL2) |
			SCU_FUN_PIN_SGPMI |
			SCU_FUN_PIN_SGPMO |
			SCU_FUN_PIN_SGPMLD |
			SCU_FUN_PIN_SGPMCK, AST_SCU_FUN_PIN_CTRL2);	
}	

EXPORT_SYMBOL(ast_scu_multi_func_sgpio);

//***********************************Information ***********************************
extern u32
ast_scu_revision_id(void)
{
	int i;
	u32 rev_id = ast_scu_read(AST_SCU_REVISION_ID);
	for(i=0;i<ARRAY_SIZE(soc_map_table);i++) {
		if(rev_id == soc_map_table[i].rev_id)
			break;
	}
	if(i == ARRAY_SIZE(soc_map_table))
		BMC_SCUMSG("UnKnow-SOC : %x \n",rev_id);
	else
		BMC_SCUMSG("SOC : %4s \n",soc_map_table[i].name);
	
	return rev_id;
}	

EXPORT_SYMBOL(ast_scu_revision_id);

extern void
ast_scu_security_info(void)
{
	switch((ast_scu_read(AST_SCU_HW_STRAP2) >> 18) & 0x3) {
		case 1:
			printk("SEC : DSS Mode \n");
			break;
		case 2:
			printk("SEC : UnKnow \n");
			break;			
		case 3:
			printk("SEC : SPI2 Mode \n");
			break;						
	}

}	

extern void
ast_scu_sys_rest_info(void)
{
	u32 rest = ast_scu_read(AST_SCU_SYS_CTRL);

	if(rest & SCU_SYS_EXT_RESET_FLAG) {
		BMC_SCUMSG("RST : External \n");
		ast_scu_write(ast_scu_read(AST_SCU_SYS_CTRL) & ~SCU_SYS_EXT_RESET_FLAG, AST_SCU_SYS_CTRL);
	}
	if (rest & SCU_SYS_WDT1_RESET_FLAG) {
		BMC_SCUMSG("RST : WDT1 \n");		
		ast_scu_write(ast_scu_read(AST_SCU_SYS_CTRL) & ~SCU_SYS_WDT1_RESET_FLAG, AST_SCU_SYS_CTRL);
	}
#ifdef SCU_SYS_WDT2_RESET_FLAG		
	if (rest & SCU_SYS_WDT2_RESET_FLAG) {
		BMC_SCUMSG("RST : WDT2 - 2nd Boot \n");
		ast_scu_write(ast_scu_read(AST_SCU_SYS_CTRL) & ~SCU_SYS_WDT2_RESET_FLAG, AST_SCU_SYS_CTRL);
	}
#endif
#ifdef SCU_SYS_WDT3_RESET_FLAG
	if (rest & SCU_SYS_WDT3_RESET_FLAG) {
		BMC_SCUMSG("RST : WDT3 - Boot\n");
		ast_scu_write(ast_scu_read(AST_SCU_SYS_CTRL) & ~SCU_SYS_WDT3_RESET_FLAG, AST_SCU_SYS_CTRL);
	}
#endif		
	if (rest & SCU_SYS_PWR_RESET_FLAG) {
		BMC_SCUMSG("RST : Power On \n");
		ast_scu_write(ast_scu_read(AST_SCU_SYS_CTRL) & ~SCU_SYS_PWR_RESET_FLAG, AST_SCU_SYS_CTRL);
	}
}	

extern void
ast_scu_set_vga_display(u8 enable)
{
	if(enable)
		ast_scu_write(ast_scu_read(AST_SCU_MISC1_CTRL) & ~SCU_MISC_VGA_CRT_DIS, AST_SCU_MISC1_CTRL);
	else
		ast_scu_write(ast_scu_read(AST_SCU_MISC1_CTRL) | SCU_MISC_VGA_CRT_DIS, AST_SCU_MISC1_CTRL);
}

EXPORT_SYMBOL(ast_scu_set_vga_display);

extern u8
ast_scu_get_vga_display(void)
{
	if(ast_scu_read(AST_SCU_MISC1_CTRL) & SCU_MISC_VGA_CRT_DIS)
		return 0;
	else
		return 1;
}

EXPORT_SYMBOL(ast_scu_get_vga_display);

extern u32
ast_scu_get_vga_memsize(void)
{
	u32 size=0;

	switch(SCU_HW_STRAP_VGA_SIZE_GET(ast_scu_read(AST_SCU_HW_STRAP1))) {
		case VGA_8M_DRAM:
			size = 8*1024*1024;
			break;
		case VGA_16M_DRAM:
			size = 16*1024*1024;
			break;
		case VGA_32M_DRAM:
			size = 32*1024*1024;
			break;
		case VGA_64M_DRAM:
			size = 64*1024*1024;
			break;
		default:
			BMC_SCUMSG("error vga size \n");
			break;
	}
	return size;
}

EXPORT_SYMBOL(ast_scu_get_vga_memsize);

extern void
ast_scu_get_who_init_dram(void)
{
	switch(SCU_VGA_DRAM_INIT_MASK(ast_scu_read(AST_SCU_VGA0))) {
		case 0:
			BMC_SCUMSG("VBIOS init \n");
			break;
		case 1:
			BMC_SCUMSG("SOC init \n");
			break;
		default:
			BMC_SCUMSG("error vga size \n");
			break;
	}
}
EXPORT_SYMBOL(ast_scu_get_who_init_dram);

extern int
ast_scu_espi_mode(void)
{
#ifdef AST_SOC_G5
	return(ast_scu_read(AST_SCU_HW_STRAP1) & SCU_HW_STRAP_ESPI_MODE);
#else
	return 0;
#endif
}

extern int
ast_scu_2nd_wdt_mode(void)
{
	if(ast_scu_read(AST_SCU_HW_STRAP1) & SCU_HW_STRAP_2ND_BOOT_WDT)
		return 1;
	else 
		return 0;
}

extern u8
ast_scu_get_superio_addr_config(void)
{
	if(ast_scu_read(AST_SCU_HW_STRAP1) & SCU_HW_STRAP_SUPER_IO_CONFIG)
		return 0x4E;
	else
		return 0x2E;
}

extern u8
ast_scu_adc_trim_read(void)
{
	return (ast_scu_read(AST_SCU_OTP1) >> 28);
}
EXPORT_SYMBOL(ast_scu_adc_trim_read);

extern void
ast_scu_hw_random_enable(u8 enable)
{
	if(enable)
		ast_scu_write(ast_scu_read(AST_SCU_RAMDOM_GEN) | RNG_ENABLE, AST_SCU_RAMDOM_GEN);
	else
		ast_scu_write(ast_scu_read(AST_SCU_RAMDOM_GEN) & ~RNG_ENABLE, AST_SCU_RAMDOM_GEN);
}
EXPORT_SYMBOL(ast_scu_hw_random_enable);

extern u32
ast_scu_hw_random_read(void)
{
	return (ast_scu_read(AST_SCU_RAMDOM_DATA));
}
EXPORT_SYMBOL(ast_scu_hw_random_read);

extern u8
ast_scu_get_hw_random_type(void)
{
	return (RNG_GET_TYPE(ast_scu_read(AST_SCU_RAMDOM_GEN)));
}
EXPORT_SYMBOL(ast_scu_get_hw_random_type);

extern void
ast_scu_set_hw_random_type(u8 type)
{
	ast_scu_write(((ast_scu_read(AST_SCU_RAMDOM_GEN) & ~RNG_TYPE_MASK) | RNG_SET_TYPE(type)), AST_SCU_RAMDOM_GEN);
}
EXPORT_SYMBOL(ast_scu_set_hw_random_type);

#ifdef AST_SCU_OTP_READ_CTRL
extern u8
ast_scu_otp_read(u8 reg)
{
        ast_scu_write(SCU_OTP_TRIGGER | SCU_OTP_READ_ADDR(reg), AST_SCU_OTP_READ_CTRL);
        while(SCU_OTP_TRIGGER_STS & ast_scu_read(AST_SCU_OTP_READ_CTRL));
        return (SCU_OTP_READ_DATA(ast_scu_read(AST_SCU_OTP_READ_CTRL)));
}

EXPORT_SYMBOL(ast_scu_otp_read);
#endif

extern u32
ast_get_dram_base(void)
{
#ifdef AST_SOC_G5
	return 0x80000000;
#else
	return 0x40000000;
#endif
}

EXPORT_SYMBOL(ast_get_dram_base);


static struct regmap *ast_scu_map;

static int ast_bmc_scu_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct resource *res;
	struct device_node *np;
	u32 idx;
	u32 reset_mask;

	BMC_SCUDBG("\n");	
	ast_scu_map = syscon_regmap_lookup_by_compatible("aspeed,g5-scu");
	if (IS_ERR(ast_scu_map)) {
		printk("error \n");

	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	ast_scu_base = devm_ioremap_resource(&pdev->dev, res);

#ifdef CONFIG_AST_RUNTIME_DMA_UARTS
	if(of_machine_is_compatible("aspeed,ast2500")) {
		if((CONFIG_AST_RUNTIME_DMA_UARTS > 2) || (CONFIG_SERIAL_8250_RUNTIME_UARTS > 4)) {
			ast_scu_uartx_init();
		}
	}
#endif

#ifdef CONFIG_SERIAL_AST_DMA_UART
	//UART Setting 
	for_each_compatible_node(np, NULL, "aspeed,ast-sdma-uart") {
		BMC_SCUDBG("np->name %s %s \n", np->name, np->properties->name);
		idx = of_alias_get_id(np, "serial");
		if(idx > 0)
			ast_scu_multi_func_uart(idx);
	}
#endif 

	for_each_compatible_node(np, NULL, "ns16550a") {
		BMC_SCUDBG("np->name %s %s \n", np->name, np->properties->name);
		idx = of_alias_get_id(np, "serial");
		if(idx > 0)
			ast_scu_multi_func_uart(idx);
	}

	if(of_find_compatible_node(NULL, NULL, "aspeed,ast-pwm-tacho")) {
		BMC_SCUDBG("aspeed,ast-pwm-tacho found in SCU \n");
		//SCU Pin-MUX	//PWM & TACHO 
		ast_scu_multi_func_pwm_tacho();		
	}

	for_each_compatible_node(np, NULL, "aspeed,ast-mac") {
		BMC_SCUDBG("aspeed,ast-mac found in SCU, ");
		if (of_property_read_u32(np, "pinmux", &idx) == 0) {
			BMC_SCUDBG("pinmux = %d \n", idx);
			ast_scu_multi_func_eth(idx);
		}
	}

	for_each_compatible_node(np, NULL, "aspeed,ast-i2c") {
		BMC_SCUDBG("aspeed,ast-i2c found in SCU, ");
		idx = of_alias_get_id(np, "i2c");		
		BMC_SCUDBG("bus = %d ", idx);
		if(idx > 0)
			ast_scu_multi_func_i2c(idx);
		BMC_SCUDBG("\n");
	}

	for_each_compatible_node(np, NULL, "aspeed,sdhci-ast") {
		BMC_SCUDBG("aspeed,sdhci-ast found in SCU, ");
		if (of_property_read_u32(np, "slot", &idx) == 0) {
			BMC_SCUDBG("slot = %d ", idx);
			ast_scu_multi_func_sdhc_slot(idx);
		}
		BMC_SCUDBG("\n");
	}

	if(of_find_compatible_node(NULL, NULL, "aspeed,ast-udc")) {
		BMC_SCUDBG("aspeed,ast-udc found in SCU \n");
		ast_scu_multi_func_usb_port1_mode(0);
	}
#if 1
	if((of_find_compatible_node(NULL, NULL, "aspeed,ast-uhci")) || (of_find_compatible_node(NULL, NULL, "aspeed,ast-ehci"))) {
		BMC_SCUDBG("aspeed,usb host found in SCU \n");
		ast_scu_multi_func_usb_port1_mode(1);
		ast_scu_multi_func_usb_port2_mode(2);
#ifdef AST_SOC_G4		
		ast_scu_multi_func_usb_port34_mode(1);
#endif
	} 
#else
	ast_scu_multi_func_usb_port1_mode(0);

#endif

//TODO Fix for ast2400
#if 0
	if(of_find_compatible_node(np, NULL, "aspeed,ast-g4-wdt")) {
		if(of_property_read_u32(np, "reset_mask", &reset_mask))
			ast_scu_write(reset_mask, AST_SCU_WDT_RESET);
		BMC_SCUDBG("aspeed,ast-g4-wdt reset mask %x \n", reset_mask);
	}
#endif
	ast_scu_show_system_info();

#ifdef CONFIG_ARCH_AST3200
	//AST3200 usb audio codec clock
	ast_scu_osc_clk_output();
#endif

out:
	return ret;
}

static const struct of_device_id ast_bmc_scu_of_match[] = {
	{ .compatible = "aspeed,ast-bmc-scu", },
	{ }
};

static struct platform_driver ast_bmc_scu_driver = {
	.probe = ast_bmc_scu_probe,
	.driver = {
		.name = KBUILD_MODNAME,
		.of_match_table = ast_bmc_scu_of_match,
	},
};

static int ast_bmc_scu_init(void)
{
	return platform_driver_register(&ast_bmc_scu_driver);
}

core_initcall(ast_bmc_scu_init);
