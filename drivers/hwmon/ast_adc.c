/*
 * ast-adc.c - ADC driver for the Aspeed SoC
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
/* attr ADC sysfs 0~max adc channel 
*	 0 - show/store enable
*	 3 - show value
*	 1 - show/store alarm_en set enable 
*	 2 - show alarm   get statuse
*	 4 - show/store upper
*	 5 - show/store lower  */

#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/timer.h>
#include <linux/mutex.h>
#include <linux/hwmon-sysfs.h>
#include <linux/hwmon.h>
#include <linux/workqueue.h>
#include <linux/sysfs.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/reset.h>
#include <linux/of.h>
#include <mach/ast-scu.h>
/****************************************************************************************/
/*AST ADC Register Definition */
#define AST_ADC_CTRL			0x00
#define AST_ADC_IER				0x04
#define AST_ADC_VGA				0x08
#define AST_ADC_CLK				0x0c
#define AST_ADC_CH0_1			0x10
#define AST_ADC_CH2_3			0x14
#define AST_ADC_CH4_5			0x18
#define AST_ADC_CH6_7			0x1c
#define AST_ADC_CH8_9			0x20
#define AST_ADC_CH10_11			0x24
#define AST_ADC_CH12_13			0x28
#define AST_ADC_CH14_15			0x2c
#define AST_ADC_BOUND0			0x30
#define AST_ADC_BOUND1			0x34
#define AST_ADC_BOUND2			0x38
#define AST_ADC_BOUND3			0x3c
#define AST_ADC_BOUND4			0x40
#define AST_ADC_BOUND5			0x44
#define AST_ADC_BOUND6			0x48
#define AST_ADC_BOUND7			0x4c
#define AST_ADC_BOUND8			0x50
#define AST_ADC_BOUND9			0x54
#define AST_ADC_BOUND10			0x58
#define AST_ADC_BOUND11			0x5c
#define AST_ADC_BOUND12			0x60
#define AST_ADC_BOUND13			0x64
#define AST_ADC_BOUND14			0x68
#define AST_ADC_BOUND15			0x6c
#define AST_ADC_HYSTER0			0x70
#define AST_ADC_HYSTER1			0x74
#define AST_ADC_HYSTER2			0x78
#define AST_ADC_HYSTER3			0x7c
#define AST_ADC_HYSTER4			0x80
#define AST_ADC_HYSTER5			0x84
#define AST_ADC_HYSTER6			0x88
#define AST_ADC_HYSTER7			0x8c
#define AST_ADC_HYSTER8			0x90
#define AST_ADC_HYSTER9			0x94
#define AST_ADC_HYSTER10		0x98
#define AST_ADC_HYSTER11		0x9c
#define AST_ADC_HYSTER12		0xa0
#define AST_ADC_HYSTER13		0xa4
#define AST_ADC_HYSTER14		0xa8
#define AST_ADC_HYSTER15		0xac
#define AST_ADC_INTR_SEL		0xC0
#define AST_ADC_CH16			0xD0
#define AST_ADC_CH17			0xD4
#define AST_ADC_COMP_TRIM		0xC4

// AST_ADC_CTRL:0x00 - ADC Engine Control Register 
#define AST_ADC_CTRL_CH15_EN		(0x1 << 31)
#define AST_ADC_CTRL_CH14_EN		(0x1 << 30)
#define AST_ADC_CTRL_CH13_EN		(0x1 << 29)
#define AST_ADC_CTRL_CH12_EN		(0x1 << 28)
#define AST_ADC_CTRL_CH11_EN		(0x1 << 27)
#define AST_ADC_CTRL_CH10_EN		(0x1 << 26)
#define AST_ADC_CTRL_CH9_EN			(0x1 << 25)
#define AST_ADC_CTRL_CH8_EN			(0x1 << 24)
#define AST_ADC_CTRL_CH7_EN			(0x1 << 23)
#define AST_ADC_CTRL_CH6_EN			(0x1 << 22)
#define AST_ADC_CTRL_CH5_EN			(0x1 << 21)
#define AST_ADC_CTRL_CH4_EN			(0x1 << 20)
#define AST_ADC_CTRL_CH3_EN			(0x1 << 19)
#define AST_ADC_CTRL_CH2_EN			(0x1 << 18)
#define AST_ADC_CTRL_CH1_EN			(0x1 << 17)
#define AST_ADC_CTRL_CH0_EN			(0x1 << 16)

//AST_G3
#define AST_ADC_CTRL_COMPEN_CLR		(0x1 << 6)
#define AST_G3_ADC_CTRL_COMPEN		(0x1 << 5)
#define AST_ADC_CTRL_COMPEN			(0x1 << 4)
//AST_G5 support 
#define AST_ADC_CTRL_INIT_RDY		(0x1 << 8)
//AST_G5 support Auto Compesating
#define AST_ADC_CTRL_AUTO_COMPEN	(0x1 << 5)

#define AST_ADC_CTRL_NORMAL			(0x7 << 1)

#define AST_ADC_CTRL_EN				(0x1)


/* AST_ADC_IER : 0x04 - Interrupt Enable and Interrupt status	*/
#define AST_ADC_IER_CH15			(0x1 << 31)
#define AST_ADC_IER_CH14			(0x1 << 30)
#define AST_ADC_IER_CH13			(0x1 << 29)
#define AST_ADC_IER_CH12			(0x1 << 28)
#define AST_ADC_IER_CH11			(0x1 << 27)
#define AST_ADC_IER_CH10			(0x1 << 26)
#define AST_ADC_IER_CH9				(0x1 << 25)
#define AST_ADC_IER_CH8				(0x1 << 24)
#define AST_ADC_IER_CH7				(0x1 << 23)
#define AST_ADC_IER_CH6				(0x1 << 22)
#define AST_ADC_IER_CH5				(0x1 << 21)
#define AST_ADC_IER_CH4				(0x1 << 20)
#define AST_ADC_IER_CH3				(0x1 << 19)
#define AST_ADC_IER_CH2				(0x1 << 18)
#define AST_ADC_IER_CH1				(0x1 << 17)
#define AST_ADC_IER_CH0				(0x1 << 16)
#define AST_ADC_STS_CH15			(0x1 << 15)
#define AST_ADC_STS_CH14			(0x1 << 14)
#define AST_ADC_STS_CH13			(0x1 << 13)
#define AST_ADC_STS_CH12			(0x1 << 12)
#define AST_ADC_STS_CH11			(0x1 << 11)
#define AST_ADC_STS_CH10			(0x1 << 10)
#define AST_ADC_STS_CH9				(0x1 << 9)
#define AST_ADC_STS_CH8				(0x1 << 8)
#define AST_ADC_STS_CH7				(0x1 << 7)
#define AST_ADC_STS_CH6				(0x1 << 6)
#define AST_ADC_STS_CH5				(0x1 << 5)
#define AST_ADC_STS_CH4				(0x1 << 4)
#define AST_ADC_STS_CH3				(0x1 << 3)
#define AST_ADC_STS_CH2				(0x1 << 2)
#define AST_ADC_STS_CH1				(0x1 << 1)
#define AST_ADC_STS_CH0				(0x1)

/* AST_ADC_VGA	: 0x08 - VGA Detect Control */
#define AST_ADC_VGA_EN				(0x1 << 16)
#define AST_ADC_VGA_DIV_MASK		(0x3ff)

/* AST_ADC_CLK : 0x0c - ADC CLK Control */
#define AST_ADC_CLK_PRE_DIV_MASK	(0x7fff << 17)
#define AST_ADC_CLK_PRE_DIV			(0x1 << 17)
#define AST_ADC_CLK_INVERT			(0x1 << 16)		//only for ast2300
#define AST_ADC_CLK_DIV_MASK		(0x3ff)

#define AST_ADC_H_CH_MASK			(0x3ff << 16)
#define AST_ADC_L_CH_MASK			(0x3ff)

#define AST_ADC_H_BOUND				(0x3ff << 16)
#define AST_ADC_L_BOUND				(0x3ff)

#define AST_ADC_HYSTER_EN			(0x1 << 31)

//only support in ast2500
/* AST_ADC_CH16	 : 0xD0 - */
/* AST_ADC_CH17	 : 0xD4 - */
#define AST_TEMP_CH_RDY				(0x1 << 31)
#define AST_GET_TEMP_A_MASK(x)		((x >>16) & 0xfff)
#define AST_TEMP_CH_EN				(0x1 << 15)		
#define AST_GET_TEMP_B_MASK(x)		(x & 0xfff)

/******************************************************************************************/

struct adc_ch_vcc_data {
	int v2;
	int r1;
	int r2;	
};

static struct adc_ch_vcc_data vcc_ref[16] = {
	[0] = {
		.v2 = 0,
		.r1 = 5600,
		.r2 = 1000,
	},
	[1] = {
		.v2 = -12,
		.r1 = 1000,
		.r2 = 10000,
	},
	[2] = {
		.v2 = 0,
		.r1 = 1800,
		.r2 = 1000,
	},
	[3] = {
		.v2 = -5,
		.r1 = 2200,
		.r2 = 10000,
	},
	[4] = {
		.v2 = 0,
		.r1 = 56000,
		.r2 = 1000,
	},
};

struct ast_adc_config {
	u8		adc_version;	
	u8		adc_ch_num;
	u8		tmper_ch_num;
};

struct ast_adc_data {
	struct device			*dev;
	void __iomem			*reg_base;			/* virtual */
	int 	irq;			//ADC IRQ number 
	struct ast_adc_config	*config;
	int	compen_value;		//Compensating value
	struct reset_control *reset;	
	struct clk 			*clk;
	u32					pclk;
};

static inline void
ast_adc_write(struct ast_adc_data *ast_adc, u32 val, u32 reg)
{
	dev_dbg(ast_adc->dev, "write offset: %x, val: %x \n",reg,val);
	writel(val, ast_adc->reg_base+ reg);
}

static inline u32
ast_adc_read(struct ast_adc_data *ast_adc, u32 reg)
{
	u32 val = readl(ast_adc->reg_base + reg);
	dev_dbg(ast_adc->dev, "read offset: %x, val: %x \n",reg, val);
	return val;
}

static void ast_g5_adc_ctrl_init(struct ast_adc_data *ast_adc)
{
	//Auto Compensating Sensing Mode : do not use in AST-G5
	u8 trim;

	//Set wait a sensing cycle t (s) = 12 * (1/PCLK) * 2 * (ADC0c[31:17] + 1) * (ADC0c[9:0] +1)
	//ex : pclk = 48Mhz , ADC0c[31:17] = 0,  ADC0c[9:0] = 0x40 : 64,  ADC0c[31:17] = 0x3e7 : 999 
	// --> 0.0325s	= 12 * 2 * (0x3e7 + 1) *(64+1) / 48000000
	// --> 0.0005s	= 12 * 2 * (0x3e7 + 1) / 48000000	

//	TODO ... 
//	scu read trim 
//	write trim 0xC4 [3:0]
	trim = ast_scu_adc_trim_read();

	if((trim == 0x0))
		trim = 0x8;

	ast_adc_write(ast_adc, trim, AST_ADC_COMP_TRIM);

	ast_adc_write(ast_adc, 0x40, AST_ADC_CLK);

	ast_adc_write(ast_adc, AST_ADC_CTRL_NORMAL | AST_ADC_CTRL_EN, AST_ADC_CTRL);

	while(!(ast_adc_read(ast_adc, AST_ADC_CTRL) & AST_ADC_CTRL_INIT_RDY));

#if 0
	ast_adc_write(ast_adc, AST_ADC_CTRL_AUTO_COMPEN  | AST_ADC_CTRL_NORMAL | 
							AST_ADC_CTRL_EN, AST_ADC_CTRL);

	while(ast_adc_read(ast_adc, AST_ADC_CTRL) & AST_ADC_CTRL_AUTO_COMPEN);

	//compensating value = 0x200 - ADC10[9:0]
	ast_adc->compen_value = 0x200 - ((ast_adc_read(ast_adc, AST_ADC_COMP_TRIM) >> 16) & 0x3ff);
	dev_dbg(ast_adc->dev, "compensating value %d \n",ast_adc->compen_value);
#else
	ast_adc_write(ast_adc, AST_ADC_CTRL_CH0_EN | AST_ADC_CTRL_COMPEN | 
							AST_ADC_CTRL_NORMAL | AST_ADC_CTRL_EN, 
							AST_ADC_CTRL);

	ast_adc_read(ast_adc, AST_ADC_CTRL);

	mdelay(1);

	//compensating value = 0x200 - ADC10[9:0]
	ast_adc->compen_value = 0x200 - (ast_adc_read(ast_adc, AST_ADC_CH0_1) & AST_ADC_L_CH_MASK);
	dev_dbg(ast_adc->dev, "compensating value %d \n",ast_adc->compen_value);

	ast_adc_write(ast_adc, ~(AST_ADC_CTRL_COMPEN | AST_ADC_CTRL_CH0_EN) & ast_adc_read(ast_adc, AST_ADC_CTRL), AST_ADC_CTRL);	

#endif
		
}

static void ast_g4_adc_ctrl_init(struct ast_adc_data *ast_adc)
{
	//Compensating Sensing Mode
	//Set wait a sensing cycle t (s) = 12 * (1/PCLK) * 2 * (ADC0c[31:17] + 1) * (ADC0c[9:0] +1)
	//ex : pclk = 48Mhz , ADC0c[31:17] = 0,  ADC0c[9:0] = 0x40 : 64,  ADC0c[31:17] = 0x3e7 : 999 
	// --> 0.0325s	= 12 * 2 * (0x3e7 + 1) *(64+1) / 48000000
	// --> 0.0005s	= 12 * 2 * (0x3e7 + 1) / 48000000	
	//For AST2400 A0 workaround  ... ADC0c = 1 ;
//	ast_adc_write(ast_adc, 1, AST_ADC_CLK);
//	ast_adc_write(ast_adc, (0x3e7<< 17) | 0x40, AST_ADC_CLK);
	ast_adc_write(ast_adc, 0x40, AST_ADC_CLK);

	ast_adc_write(ast_adc, AST_ADC_CTRL_CH0_EN | AST_ADC_CTRL_COMPEN | 
							AST_ADC_CTRL_NORMAL | AST_ADC_CTRL_EN, 
							AST_ADC_CTRL);

	ast_adc_read(ast_adc, AST_ADC_CTRL);

	mdelay(1);

	//compensating value = 0x200 - ADC10[9:0]
	ast_adc->compen_value = 0x200 - (ast_adc_read(ast_adc, AST_ADC_CH0_1) & AST_ADC_L_CH_MASK);
	dev_dbg(ast_adc->dev, "compensating value %d \n",ast_adc->compen_value);

	ast_adc_write(ast_adc, ~AST_ADC_CTRL_COMPEN & ast_adc_read(ast_adc, AST_ADC_CTRL), AST_ADC_CTRL);	
}

static void ast_g3_adc_ctrl_init(struct ast_adc_data *ast_adc)
{
	//Set wait a sensing cycle t (s) = 12 * (1/PCLK) * 2 * (ADC0c[31:17] + 1) * (ADC0c[9:0] +1)
	//ex : pclk = 48Mhz , ADC0c[31:17] = 0,  ADC0c[9:0] = 0x40 : 64,  ADC0c[31:17] = 0x3e7 : 999 
	// --> 0.0325s	= 12 * 2 * (0x3e7 + 1) *(64+1) / 48000000
	// --> 0.0005s	= 12 * 2 * (0x3e7 + 1) / 48000000	
	
	ast_adc_write(ast_adc, 0x3e7, AST_ADC_CLK); 

	ast_adc_write(ast_adc, AST_ADC_CTRL_CH12_EN | AST_ADC_CTRL_COMPEN_CLR | 
							AST_G3_ADC_CTRL_COMPEN | AST_ADC_CTRL_NORMAL | 
							AST_ADC_CTRL_EN, AST_ADC_CTRL);

	mdelay(50);

	//compensating value = 0x200 - ADC10[9:0]
	if(ast_adc_read(ast_adc, AST_ADC_CH12_13) & (0x1 << 8))
		ast_adc->compen_value = 0x200 - (ast_adc_read(ast_adc, AST_ADC_CH12_13) & AST_ADC_L_CH_MASK);
	else
		ast_adc->compen_value = 0 - (ast_adc_read(ast_adc, AST_ADC_CH12_13) & AST_ADC_L_CH_MASK);

	dev_dbg(ast_adc->dev, "compensating value %d \n",ast_adc->compen_value);

	ast_adc_write(ast_adc, ~AST_G3_ADC_CTRL_COMPEN & ast_adc_read(ast_adc, AST_ADC_CTRL), AST_ADC_CTRL);	
}

static u16
ast_get_adc_hyster_lower(struct ast_adc_data *ast_adc, u8 adc_ch)
{
    u16 tmp=0;
	tmp = ast_adc_read(ast_adc, AST_ADC_HYSTER0 + (adc_ch *4)) & AST_ADC_L_BOUND;

	dev_dbg(ast_adc->dev, "read val = %d \n",tmp);

	return tmp;

}

static void
ast_set_adc_hyster_lower(struct ast_adc_data *ast_adc, u8 adc_ch, u16 value)
{
	ast_adc_write(ast_adc, 
			(ast_adc_read(ast_adc, AST_ADC_HYSTER0 + (adc_ch *4)) & ~AST_ADC_L_BOUND) |
			value, 
			AST_ADC_HYSTER0 + (adc_ch *4));

}

static u16
ast_get_adc_hyster_upper(struct ast_adc_data *ast_adc, u8 adc_ch)
{
    u16 tmp=0;
	tmp = ((ast_adc_read(ast_adc, AST_ADC_HYSTER0 + (adc_ch *4)) & AST_ADC_H_BOUND) >> 16);

	dev_dbg(ast_adc->dev, "read val = %d \n",tmp);

	return tmp;
}

static void
ast_set_adc_hyster_upper(struct ast_adc_data *ast_adc, u8 adc_ch, u32 value)
{
	ast_adc_write(ast_adc, 
			(ast_adc_read(ast_adc, AST_ADC_HYSTER0 + (adc_ch *4)) & ~AST_ADC_H_BOUND) |
			(value << 16), 
			AST_ADC_HYSTER0 + (adc_ch *4));

}

static u8
ast_get_adc_hyster_en(struct ast_adc_data *ast_adc, u8 adc_ch)
{
	//tacho source
	if(ast_adc_read(ast_adc, AST_ADC_HYSTER0 + (adc_ch *4)) & AST_ADC_HYSTER_EN)
		return 1;
	else
		return 0;
}

static void
ast_set_adc_hyster_en(struct ast_adc_data *ast_adc, u8 adc_ch, u8 enable)
{
	//tacho source 
	if(enable == 1)
		ast_adc_write(ast_adc,
			ast_adc_read(ast_adc, AST_ADC_HYSTER0 + (adc_ch *4)) | AST_ADC_HYSTER_EN,
			AST_ADC_HYSTER0 + (adc_ch *4));
	else
		ast_adc_write(ast_adc,
			ast_adc_read(ast_adc, AST_ADC_HYSTER0 + (adc_ch *4)) & ~AST_ADC_HYSTER_EN,
			AST_ADC_HYSTER0 + (adc_ch *4));
}

static u16
ast_get_adc_lower(struct ast_adc_data *ast_adc, u8 adc_ch)
{
    u16 tmp=0;
	tmp = ast_adc_read(ast_adc, AST_ADC_BOUND0 + (adc_ch *4)) & AST_ADC_L_BOUND;

	dev_dbg(ast_adc->dev, "read val = %d \n",tmp);

	return tmp;

}

static void
ast_set_adc_lower(struct ast_adc_data *ast_adc, u8 adc_ch, u16 value)
{
	ast_adc_write(ast_adc, 
			(ast_adc_read(ast_adc, AST_ADC_BOUND0 + (adc_ch *4)) & ~AST_ADC_L_BOUND) |
			value, 
			AST_ADC_BOUND0 + (adc_ch *4));

}

static u16
ast_get_adc_upper(struct ast_adc_data *ast_adc, u8 adc_ch)
{
    u16 tmp=0;
	tmp = ((ast_adc_read(ast_adc, AST_ADC_BOUND0 + (adc_ch *4)) & AST_ADC_H_BOUND) >> 16);

	dev_dbg(ast_adc->dev, "read val = %d \n",tmp);

	return tmp;


}

static void
ast_set_adc_upper(struct ast_adc_data *ast_adc, u8 adc_ch, u32 value)
{
	ast_adc_write(ast_adc, 
			(ast_adc_read(ast_adc, AST_ADC_BOUND0 + (adc_ch *4)) & ~AST_ADC_H_BOUND) |
			(value << 16), 
			AST_ADC_BOUND0 + (adc_ch *4));

}


static u8
ast_get_adc_alarm(struct ast_adc_data *ast_adc, u8 adc_ch)
{
	//adc ch source 
	if(ast_adc_read(ast_adc, AST_ADC_IER) & (0x1 << adc_ch))
		return 1;
	else
		return 0;
}

static u16
ast_get_adc_value(struct ast_adc_data *ast_adc, u8 adc_ch)
{
    int tmp = 0;

	switch(adc_ch) {
		case 0:
			tmp = ast_adc_read(ast_adc, AST_ADC_CH0_1) & AST_ADC_L_CH_MASK;
			break;
		case 1:
			tmp = (ast_adc_read(ast_adc, AST_ADC_CH0_1) & AST_ADC_H_CH_MASK) >> 16;
			break;	
		case 2:
			tmp = ast_adc_read(ast_adc, AST_ADC_CH2_3) & AST_ADC_L_CH_MASK;
			break;
		case 3:
			tmp = (ast_adc_read(ast_adc, AST_ADC_CH2_3) & AST_ADC_H_CH_MASK) >> 16;
			break;	
		case 4:
			tmp = ast_adc_read(ast_adc, AST_ADC_CH4_5) & AST_ADC_L_CH_MASK;
			break;
		case 5:
			tmp = (ast_adc_read(ast_adc, AST_ADC_CH4_5) & AST_ADC_H_CH_MASK) >> 16;
			break;	
		case 6:
			tmp = ast_adc_read(ast_adc, AST_ADC_CH6_7) & AST_ADC_L_CH_MASK;
			break;
		case 7:
			tmp = (ast_adc_read(ast_adc, AST_ADC_CH6_7) & AST_ADC_H_CH_MASK) >> 16;
			break;	
		case 8:
			tmp = ast_adc_read(ast_adc, AST_ADC_CH8_9) & AST_ADC_L_CH_MASK;
			break;
		case 9:
			tmp = (ast_adc_read(ast_adc, AST_ADC_CH8_9) & AST_ADC_H_CH_MASK) >> 16;
			break;	
		case 10:
			tmp = ast_adc_read(ast_adc, AST_ADC_CH10_11) & AST_ADC_L_CH_MASK;
			break;
		case 11:
			tmp = (ast_adc_read(ast_adc, AST_ADC_CH10_11) & AST_ADC_H_CH_MASK) >> 16;
			break;	
		case 12:
			tmp = ast_adc_read(ast_adc, AST_ADC_CH12_13) & AST_ADC_L_CH_MASK;
			break;
		case 13:
			tmp = (ast_adc_read(ast_adc, AST_ADC_CH12_13) & AST_ADC_H_CH_MASK) >> 16;
			break;	
		case 14:
			tmp = ast_adc_read(ast_adc, AST_ADC_CH14_15) & AST_ADC_L_CH_MASK;
			break;
		case 15:
			tmp = (ast_adc_read(ast_adc, AST_ADC_CH14_15) & AST_ADC_H_CH_MASK) >> 16;
			break;	

	}

	tmp += ast_adc->compen_value;
	if(tmp < 0)
		tmp = 0;

	dev_dbg(ast_adc->dev, "voltage raw = %d \n",tmp);
	
	return tmp;

}

static u8 
ast_get_adc_en(struct ast_adc_data *ast_adc, u8 adc_ch)
{
    u8 tmp=0;

	if(ast_adc_read(ast_adc, AST_ADC_CTRL) & (0x1 << (16+adc_ch)))
		tmp = 1;
	else
		tmp = 0;

	return tmp;

}

static void 
ast_set_adc_en(struct ast_adc_data *ast_adc, u8 adc_ch, u8 enable)
{
	if(enable)
		ast_adc_write(ast_adc, ast_adc_read(ast_adc, AST_ADC_CTRL) | (0x1 << (16+adc_ch)), AST_ADC_CTRL);
	else
		ast_adc_write(ast_adc, ast_adc_read(ast_adc, AST_ADC_CTRL) & ~(0x1 << (16+adc_ch)), AST_ADC_CTRL);
}


/* attr ADC sysfs 0~max adc channel 
*	 0 - show/store channel enable
*	 1 - show value 
*	 2 - show alarm   get statuse
*	 3 - show/store upper
*	 4 - show/store lower 
*	 5 - show/store hystersis enable  
*	 6 - show/store hystersis upper  
*	 7 - show/store hystersis low  
*/

static ssize_t 
ast_show_adc(struct device *dev, struct device_attribute *attr, char *sysfsbuf)
{
	struct ast_adc_data *ast_adc = dev_get_drvdata(dev);

	struct sensor_device_attribute_2 *sensor_attr = to_sensor_dev_attr_2(attr);
	u16 tmp;
	u32 voltage,tmp1, tmp2,tmp3;

	//sensor_attr->index : pwm_ch#
	//sensor_attr->nr : attr#
	switch(sensor_attr->nr) 
	{
		case 0: //channel enable, disable
			return sprintf(sysfsbuf, "%d : %s\n", ast_get_adc_en(ast_adc, sensor_attr->index),ast_get_adc_en(ast_adc,sensor_attr->index) ? "Enable":"Disable");
			break;
		case 1: //value
			tmp = ast_get_adc_value(ast_adc, sensor_attr->index);
			return sprintf(sysfsbuf, "%d \n",tmp);
			break;
		case 2: //alarm
			return sprintf(sysfsbuf, "%d \n", ast_get_adc_alarm(ast_adc,sensor_attr->index));
			break;			
		case 3: //upper
			return sprintf(sysfsbuf, "%d \n", ast_get_adc_upper(ast_adc,sensor_attr->index));
			break;			
		case 4: //lower
			return sprintf(sysfsbuf, "%d \n", ast_get_adc_lower(ast_adc,sensor_attr->index));
			break;			
		case 5: //hystersis enable 
			return sprintf(sysfsbuf, "%d : %s\n", ast_get_adc_hyster_en(ast_adc,sensor_attr->index),ast_get_adc_hyster_en(ast_adc,sensor_attr->index) ? "Enable":"Disable");
			break;			
		case 6: //hystersis upper
			return sprintf(sysfsbuf, "%d \n", ast_get_adc_hyster_upper(ast_adc,sensor_attr->index));
			break;			
		case 7: //hystersis lower
			return sprintf(sysfsbuf, "%d \n", ast_get_adc_hyster_lower(ast_adc,sensor_attr->index));
			break;			
		case 8: //voltage
			tmp = ast_get_adc_value(ast_adc, sensor_attr->index);		
			//Voltage Sense Method
			tmp1 = (vcc_ref[sensor_attr->index].r1 + vcc_ref[sensor_attr->index].r2) * tmp * 25 * 10;
			tmp2 = vcc_ref[sensor_attr->index].r2 * 1023 ;
		
			tmp3 = (vcc_ref[sensor_attr->index].r1 * vcc_ref[sensor_attr->index].v2) / vcc_ref[sensor_attr->index].r2;
		//	printk("tmp3 = %d \n",tmp3);
			voltage = (tmp1/tmp2) - tmp3;			
			return sprintf(sysfsbuf, "%d.%d (V)\n",voltage/100, voltage%100);
			break;			
		case 9: //r1
			return sprintf(sysfsbuf, "%d \n", vcc_ref[sensor_attr->index].r1);
			break;			
		case 10: //r2
			return sprintf(sysfsbuf, "%d \n", vcc_ref[sensor_attr->index].r2);
			break;			
		case 11: //v2
			return sprintf(sysfsbuf, "%d \n", vcc_ref[sensor_attr->index].v2);
			break;			

		default:
			return -EINVAL;
			break;
	}
}

static ssize_t 
ast_store_adc(struct device *dev, struct device_attribute *attr, const char *sysfsbuf, size_t count)
{
	u32 input_val;
	struct ast_adc_data *ast_adc = dev_get_drvdata(dev);
	struct sensor_device_attribute_2 *sensor_attr =
						to_sensor_dev_attr_2(attr);

	input_val = simple_strtoul(sysfsbuf, NULL, 10);

	//sensor_attr->index : pwm_ch#
	//sensor_attr->nr : attr#
	switch(sensor_attr->nr) 
	{
		case 0: //enable, disable
			ast_set_adc_en(ast_adc, sensor_attr->index, input_val);
			break;
		case 1: //value
			
			break;
		case 2: //alarm
			break;			
		case 3:
			ast_set_adc_upper(ast_adc, sensor_attr->index, input_val);
			break;
		case 4:
			ast_set_adc_lower(ast_adc, sensor_attr->index, input_val);
			break;
		case 5: //hystersis
			ast_set_adc_hyster_en(ast_adc, sensor_attr->index, input_val);
			break;			
		case 6:
			ast_set_adc_hyster_upper(ast_adc, sensor_attr->index, input_val);
			break;
		case 7:
			ast_set_adc_hyster_lower(ast_adc, sensor_attr->index, input_val);
			break;
		case 8:	//voltage
			break;
		case 9: //r1
			vcc_ref[sensor_attr->index].r1 = input_val;
			break;
		case 10: //r2
			vcc_ref[sensor_attr->index].r2 = input_val;
			break;
		case 11: //v2
			input_val = simple_strtol(sysfsbuf, NULL, 10);
			vcc_ref[sensor_attr->index].v2 = input_val;
			break;			
		default:
			return -EINVAL;
			break;
	}

	return count;
}

static u16
ast_get_temper_value(struct ast_adc_data *ast_adc, u8 temper_ch)
{
    u16 temper;
	dev_dbg(ast_adc->dev, "ast_get_temper_value %d \n",temper_ch);

	switch(temper_ch) {
		case 0:
			ast_adc_write(ast_adc, ast_adc_read(ast_adc, AST_ADC_CH16) | AST_TEMP_CH_EN, AST_ADC_CH16);
			while(!(ast_adc_read(ast_adc, AST_ADC_CH16) & AST_TEMP_CH_RDY)); 
			//A-B 
			temper = AST_GET_TEMP_A_MASK(ast_adc_read(ast_adc, AST_ADC_CH16)) - 
					AST_GET_TEMP_B_MASK(ast_adc_read(ast_adc, AST_ADC_CH16));
			temper = ((temper - 247) * 100 ) / (320 - 247);
			break;
		case 1:
			ast_adc_write(ast_adc, ast_adc_read(ast_adc, AST_ADC_CH17) | AST_TEMP_CH_EN, AST_ADC_CH17);
			while(!(ast_adc_read(ast_adc, AST_ADC_CH17) & AST_TEMP_CH_RDY)); 
			//A-B 
			temper = AST_GET_TEMP_A_MASK(ast_adc_read(ast_adc, AST_ADC_CH17)) - 
					AST_GET_TEMP_B_MASK(ast_adc_read(ast_adc, AST_ADC_CH17));

			temper = ((temper - 247) * 100 ) / (320 - 247);
			break;	
	}

	return temper;

}

static ssize_t 
ast_show_temper(struct device *dev, struct device_attribute *attr, char *sysfsbuf)
{
	struct ast_adc_data *ast_adc = dev_get_drvdata(dev);
	struct sensor_device_attribute_2 *sensor_attr = to_sensor_dev_attr_2(attr);

	dev_dbg(ast_adc->dev, "ast_show_temper %d \n",sensor_attr->nr);
	//sensor_attr->index : pwm_ch#
	//sensor_attr->nr : attr#
	switch(sensor_attr->nr) 
	{
		case 0: //value
			return sprintf(sysfsbuf, "%d\n",ast_get_temper_value(ast_adc, sensor_attr->index));
			break;
		default:
			return -EINVAL;
			break;
	}
}

/* attr temperature sysfs 0~max t channel 
*	 0 - show value 
*/

#define sysfs_temper_ch(index) \
static SENSOR_DEVICE_ATTR_2(temper##index##_value, S_IRUGO | S_IWUSR, \
	ast_show_temper, NULL, 0, index); \
\
static struct attribute *temper##index##_attributes[] = { \
	&sensor_dev_attr_temper##index##_value.dev_attr.attr, \
	NULL \
};

sysfs_temper_ch(0);
sysfs_temper_ch(1);

static const struct attribute_group temper_attribute_groups[] = {
	{ .attrs = temper0_attributes },
	{ .attrs = temper1_attributes },
};

/* attr ADC sysfs 0~max adc channel 
*	 0 - show/store channel enable
*	 1 - show value 
*	 2 - show alarm   get statuse
*	 3 - show/store upper
*	 4 - show/store lower 
*	 5 - show/store hystersis enable  
*	 6 - show/store hystersis upper  
*	 7 - show/store hystersis low  
*/

#define sysfs_adc_ch(index) \
static SENSOR_DEVICE_ATTR_2(adc##index##_en, S_IRUGO | S_IWUSR, \
	ast_show_adc, ast_store_adc, 0, index); \
\
static SENSOR_DEVICE_ATTR_2(adc##index##_value, S_IRUGO | S_IWUSR, \
	ast_show_adc, NULL, 1, index); \
\
static SENSOR_DEVICE_ATTR_2(adc##index##_alarm, S_IRUGO | S_IWUSR, \
	ast_show_adc, NULL, 2, index); \
\
static SENSOR_DEVICE_ATTR_2(adc##index##_upper, S_IRUGO | S_IWUSR, \
	ast_show_adc, ast_store_adc, 3, index); \
\
static SENSOR_DEVICE_ATTR_2(adc##index##_lower, S_IRUGO | S_IWUSR, \
	ast_show_adc, ast_store_adc, 4, index); \
\
static SENSOR_DEVICE_ATTR_2(adc##index##_hyster_en, S_IRUGO | S_IWUSR, \
	ast_show_adc, ast_store_adc, 5, index); \
\
static SENSOR_DEVICE_ATTR_2(adc##index##_hyster_upper, S_IRUGO | S_IWUSR, \
	ast_show_adc, ast_store_adc, 6, index); \
\
static SENSOR_DEVICE_ATTR_2(adc##index##_hyster_lower, S_IRUGO | S_IWUSR, \
	ast_show_adc, ast_store_adc, 7, index); \
\
static SENSOR_DEVICE_ATTR_2(adc##index##_voltage, S_IRUGO | S_IWUSR, \
	ast_show_adc, NULL, 8, index); \
\
static SENSOR_DEVICE_ATTR_2(adc##index##_r1, S_IRUGO | S_IWUSR, \
	ast_show_adc, ast_store_adc, 9, index); \
\
static SENSOR_DEVICE_ATTR_2(adc##index##_r2, S_IRUGO | S_IWUSR, \
	ast_show_adc, ast_store_adc, 10, index); \
\
static SENSOR_DEVICE_ATTR_2(adc##index##_v2, S_IRUGO | S_IWUSR, \
	ast_show_adc, ast_store_adc, 11, index); \
\
static struct attribute *adc##index##_attributes[] = { \
	&sensor_dev_attr_adc##index##_en.dev_attr.attr, \
	&sensor_dev_attr_adc##index##_value.dev_attr.attr, \
	&sensor_dev_attr_adc##index##_alarm.dev_attr.attr, \
	&sensor_dev_attr_adc##index##_upper.dev_attr.attr, \
	&sensor_dev_attr_adc##index##_lower.dev_attr.attr, \
	&sensor_dev_attr_adc##index##_hyster_en.dev_attr.attr, \
	&sensor_dev_attr_adc##index##_hyster_upper.dev_attr.attr, \
	&sensor_dev_attr_adc##index##_hyster_lower.dev_attr.attr, \
	&sensor_dev_attr_adc##index##_voltage.dev_attr.attr, \
	&sensor_dev_attr_adc##index##_r1.dev_attr.attr, \
	&sensor_dev_attr_adc##index##_r2.dev_attr.attr, \
	&sensor_dev_attr_adc##index##_v2.dev_attr.attr, \
	NULL \
};

/*
 * Create the needed functions for each pwm using the macro defined above
 * (4 pwms are supported)
 */
sysfs_adc_ch(0);
sysfs_adc_ch(1);
sysfs_adc_ch(2);
sysfs_adc_ch(3);
sysfs_adc_ch(4);
sysfs_adc_ch(5);
sysfs_adc_ch(6);
sysfs_adc_ch(7);
sysfs_adc_ch(8);
sysfs_adc_ch(9);
sysfs_adc_ch(10);
sysfs_adc_ch(11);
sysfs_adc_ch(12);
sysfs_adc_ch(13);
sysfs_adc_ch(14);
sysfs_adc_ch(15);

static const struct attribute_group adc_attribute_groups[] = {
	{ .attrs = adc0_attributes },
	{ .attrs = adc1_attributes },
	{ .attrs = adc2_attributes },
	{ .attrs = adc3_attributes },
	{ .attrs = adc4_attributes },
	{ .attrs = adc5_attributes },
	{ .attrs = adc6_attributes },
	{ .attrs = adc7_attributes },
	{ .attrs = adc8_attributes },
	{ .attrs = adc9_attributes },
	{ .attrs = adc10_attributes },	
	{ .attrs = adc11_attributes },
	{ .attrs = adc12_attributes },
	{ .attrs = adc13_attributes },
	{ .attrs = adc14_attributes },
	{ .attrs = adc15_attributes },
};


static ssize_t 
store_compen_value(struct device *dev, struct device_attribute *attr, const char *sysfsbuf, size_t count)
{
	struct ast_adc_data *ast_adc = dev_get_drvdata(dev);

	ast_adc->compen_value = simple_strtoul(sysfsbuf, NULL, 10);
	return count;
}

static ssize_t 
show_compen_value(struct device *dev, struct device_attribute *attr, char *sysfsbuf)
{
	struct ast_adc_data *ast_adc = dev_get_drvdata(dev);
	
	return sprintf(sysfsbuf, "%d \n", ast_adc->compen_value);
}

static DEVICE_ATTR(adc_compen_value, S_IRUGO | S_IWUSR, show_compen_value, store_compen_value);


static struct attribute *ast_adc_attributes[] = {
	&dev_attr_adc_compen_value.attr,
	NULL
};

static const struct attribute_group ast_adc_attribute = {
	.attrs = ast_adc_attributes,
};

static const struct ast_adc_config ast_g3_config = { 
	.adc_version = 3, 
	.adc_ch_num = 12, 
	.tmper_ch_num = 0, 
};

static const struct ast_adc_config ast_g4_config = { 
	.adc_version = 4, 
	.adc_ch_num =16, 
	.tmper_ch_num = 0, 
};

static const struct ast_adc_config ast_g5_config = { 
	.adc_version = 5, 
	.adc_ch_num =16, 
	.tmper_ch_num = 2, 
};

static const struct of_device_id ast_adc_matches[] = {
	{ .compatible = "aspeed,ast-g3-adc",	.data = &ast_g3_config, },
	{ .compatible = "aspeed,ast-g4-adc",	.data = &ast_g4_config, },
	{ .compatible = "aspeed,ast-g5-adc",	.data = &ast_g5_config, },	
	{},
};

MODULE_DEVICE_TABLE(of, ast_adc_matches);

static int 
ast_adc_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct ast_adc_data *ast_adc;
	const struct of_device_id *adc_dev_id;	
	int err;
	int ret=0;
	int i;

	dev_dbg(&pdev->dev, "ast_adc_probe \n");

	ast_adc = devm_kzalloc(&pdev->dev, sizeof(struct ast_adc_data), GFP_KERNEL);
	if (!ast_adc) {
		ret = -ENOMEM;
		goto out;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (NULL == res) {
		dev_err(&pdev->dev, "cannot get IORESOURCE_MEM\n");
		ret = -ENOENT;
		goto out_mem;
	}

	ast_adc->reg_base = devm_ioremap_resource(&pdev->dev, res);
	if (!ast_adc->reg_base) {
		ret = -EIO;
		goto out_region;
	}

	ast_adc->irq = platform_get_irq(pdev, 0);
	if (ast_adc->irq < 0) {
		dev_err(&pdev->dev, "no irq specified\n");
		ret = -ENOENT;
		goto out_region;
	}

	ast_adc->reset = devm_reset_control_get_exclusive(&pdev->dev, "adc");
	if (IS_ERR(ast_adc->reset)) {
		dev_err(&pdev->dev, "can't get adc reset\n");
		return PTR_ERR(ast_adc->reset);
	}

	//scu init
	reset_control_assert(ast_adc->reset);
	reset_control_deassert(ast_adc->reset);

	ast_adc->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(ast_adc->clk)) {
		dev_err(&pdev->dev, "no clock defined\n");
		return -ENODEV;
	}
	ast_adc->pclk = clk_get_rate(ast_adc->clk);

	adc_dev_id = of_match_node(ast_adc_matches, pdev->dev.of_node);
	if (!adc_dev_id)
		return -EINVAL;

	ast_adc->config = (struct ast_adc_config *) adc_dev_id->data;

	/* Register sysfs hooks */
	ast_adc->dev = hwmon_device_register(&pdev->dev);
	if (IS_ERR(ast_adc->dev)) {
		ret = PTR_ERR(ast_adc->dev);
		goto out_region;
	}

	platform_set_drvdata(pdev, ast_adc);

	for(i=0; i<ast_adc->config->adc_ch_num; i++) {
		err = sysfs_create_group(&pdev->dev.kobj, &adc_attribute_groups[i]);
		if (err)
			goto out_region;
	}

	if(ast_adc->config->tmper_ch_num) {
		for(i=0; i<ast_adc->config->tmper_ch_num; i++) {
			err = sysfs_create_group(&pdev->dev.kobj, &temper_attribute_groups[i]);
			if (err)
				goto out_region;
		}
	}

	ret = sysfs_create_group(&pdev->dev.kobj, &ast_adc_attribute);
	if (ret)
		goto out_region;

	if(ast_adc->config->adc_version == 3) {
		ast_g3_adc_ctrl_init(ast_adc);
	} else if (ast_adc->config->adc_version == 4) {
		ast_g4_adc_ctrl_init(ast_adc);
	} else if(ast_adc->config->adc_version == 5) {
		ast_g5_adc_ctrl_init(ast_adc);
	} else {
		dev_err(&pdev->dev, "adc config error \n");	
		return -ENODEV;
	}
	
	printk(KERN_INFO "ast_adc: driver successfully loaded.\n");

	return 0;


//out_irq:
//	free_irq(ast_adc->irq, NULL);
out_region:
	release_mem_region(res->start, res->end - res->start + 1);
out_mem:
	kfree(ast_adc);
out:
	printk(KERN_WARNING "ast_adc: driver init failed (ret=%d)!\n", ret);
	return ret;
}

static int 
ast_adc_remove(struct platform_device *pdev)
{
	int i=0;
	struct ast_adc_data *ast_adc = platform_get_drvdata(pdev);
	struct resource *res;
	printk(KERN_INFO "ast_adc: driver unloaded.\n");

	hwmon_device_unregister(ast_adc->dev);

	for(i=0; i<ast_adc->config->adc_ch_num; i++)
		sysfs_remove_group(&pdev->dev.kobj, &adc_attribute_groups[i]);

	if(ast_adc->config->tmper_ch_num) {
		for(i=0; i<ast_adc->config->tmper_ch_num; i++)
			sysfs_remove_group(&pdev->dev.kobj, &temper_attribute_groups[i]);
	}

	platform_set_drvdata(pdev, NULL);
//	free_irq(ast_adc->irq, ast_adc);
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	iounmap(ast_adc->reg_base);
	release_mem_region(res->start, res->end - res->start + 1);
	kfree(ast_adc);
	return 0;
}

#ifdef CONFIG_PM
static int 
ast_adc_suspend(struct platform_device *pdev, pm_message_t state)
{
	printk("ast_adc_suspend : TODO \n");
	return 0;
}

static int 
ast_adc_resume(struct platform_device *pdev)
{
	struct ast_adc_data *ast_adc = platform_get_drvdata(pdev);

	return 0;
}

#else
#define ast_adc_suspend        NULL
#define ast_adc_resume         NULL
#endif

static struct platform_driver ast_adc_driver = {
	.probe 		= ast_adc_probe,
	.remove 		= ast_adc_remove,
	.suspend        = ast_adc_suspend,
	.resume         = ast_adc_resume,
	.driver         = {
		.name   = KBUILD_MODNAME,
		.of_match_table = ast_adc_matches,
	},
};

module_platform_driver(ast_adc_driver);

MODULE_AUTHOR("Ryan Chen <ryan_chen@aspeedtech.com>");
MODULE_DESCRIPTION("AST ADC driver");
MODULE_LICENSE("GPL");
