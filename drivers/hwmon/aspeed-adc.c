/*
 * ADC driver for the Aspeed SoC
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
/****************************************************************************************/
/*AST ADC Register Definition */
#define ASPEED_ADC_CTRL			0x00
#define ASPEED_ADC_IER				0x04
#define ASPEED_ADC_VGA				0x08
#define ASPEED_ADC_CLK				0x0c
#define ASPEED_ADC_CH0_1			0x10
#define ASPEED_ADC_CH2_3			0x14
#define ASPEED_ADC_CH4_5			0x18
#define ASPEED_ADC_CH6_7			0x1c
#define ASPEED_ADC_CH8_9			0x20
#define ASPEED_ADC_CH10_11			0x24
#define ASPEED_ADC_CH12_13			0x28
#define ASPEED_ADC_CH14_15			0x2c
#define ASPEED_ADC_BOUND0			0x30
#define ASPEED_ADC_BOUND1			0x34
#define ASPEED_ADC_BOUND2			0x38
#define ASPEED_ADC_BOUND3			0x3c
#define ASPEED_ADC_BOUND4			0x40
#define ASPEED_ADC_BOUND5			0x44
#define ASPEED_ADC_BOUND6			0x48
#define ASPEED_ADC_BOUND7			0x4c
#define ASPEED_ADC_BOUND8			0x50
#define ASPEED_ADC_BOUND9			0x54
#define ASPEED_ADC_BOUND10			0x58
#define ASPEED_ADC_BOUND11			0x5c
#define ASPEED_ADC_BOUND12			0x60
#define ASPEED_ADC_BOUND13			0x64
#define ASPEED_ADC_BOUND14			0x68
#define ASPEED_ADC_BOUND15			0x6c
#define ASPEED_ADC_HYSTER0			0x70
#define ASPEED_ADC_HYSTER1			0x74
#define ASPEED_ADC_HYSTER2			0x78
#define ASPEED_ADC_HYSTER3			0x7c
#define ASPEED_ADC_HYSTER4			0x80
#define ASPEED_ADC_HYSTER5			0x84
#define ASPEED_ADC_HYSTER6			0x88
#define ASPEED_ADC_HYSTER7			0x8c
#define ASPEED_ADC_HYSTER8			0x90
#define ASPEED_ADC_HYSTER9			0x94
#define ASPEED_ADC_HYSTER10		0x98
#define ASPEED_ADC_HYSTER11		0x9c
#define ASPEED_ADC_HYSTER12		0xa0
#define ASPEED_ADC_HYSTER13		0xa4
#define ASPEED_ADC_HYSTER14		0xa8
#define ASPEED_ADC_HYSTER15		0xac
#define ASPEED_ADC_INTR_SEL		0xC0
#define ASPEED_ADC_CH16			0xD0
#define ASPEED_ADC_CH17			0xD4
#define ASPEED_ADC_COMP_TRIM		0xC4

// ASPEED_ADC_CTRL:0x00 - ADC Engine Control Register 
#define ASPEED_ADC_CTRL_CH15_EN		(0x1 << 31)
#define ASPEED_ADC_CTRL_CH14_EN		(0x1 << 30)
#define ASPEED_ADC_CTRL_CH13_EN		(0x1 << 29)
#define ASPEED_ADC_CTRL_CH12_EN		(0x1 << 28)
#define ASPEED_ADC_CTRL_CH11_EN		(0x1 << 27)
#define ASPEED_ADC_CTRL_CH10_EN		(0x1 << 26)
#define ASPEED_ADC_CTRL_CH9_EN			(0x1 << 25)
#define ASPEED_ADC_CTRL_CH8_EN			(0x1 << 24)
#define ASPEED_ADC_CTRL_CH7_EN			(0x1 << 23)
#define ASPEED_ADC_CTRL_CH6_EN			(0x1 << 22)
#define ASPEED_ADC_CTRL_CH5_EN			(0x1 << 21)
#define ASPEED_ADC_CTRL_CH4_EN			(0x1 << 20)
#define ASPEED_ADC_CTRL_CH3_EN			(0x1 << 19)
#define ASPEED_ADC_CTRL_CH2_EN			(0x1 << 18)
#define ASPEED_ADC_CTRL_CH1_EN			(0x1 << 17)
#define ASPEED_ADC_CTRL_CH0_EN			(0x1 << 16)

//ASPEED_G3
#define ASPEED_ADC_CTRL_COMPEN_CLR		(0x1 << 6)
#define ASPEED_G3_ADC_CTRL_COMPEN		(0x1 << 5)
#define ASPEED_ADC_CTRL_COMPEN			(0x1 << 4)
//ASPEED_G5 support 
#define ASPEED_ADC_CTRL_INIT_RDY		(0x1 << 8)
//ASPEED_G5 support Auto Compesating
#define ASPEED_ADC_CTRL_AUTO_COMPEN	(0x1 << 5)

#define ASPEED_ADC_CTRL_NORMAL			(0x7 << 1)

#define ASPEED_ADC_CTRL_EN				(0x1)


/* ASPEED_ADC_IER : 0x04 - Interrupt Enable and Interrupt status	*/
#define ASPEED_ADC_IER_CH15			(0x1 << 31)
#define ASPEED_ADC_IER_CH14			(0x1 << 30)
#define ASPEED_ADC_IER_CH13			(0x1 << 29)
#define ASPEED_ADC_IER_CH12			(0x1 << 28)
#define ASPEED_ADC_IER_CH11			(0x1 << 27)
#define ASPEED_ADC_IER_CH10			(0x1 << 26)
#define ASPEED_ADC_IER_CH9				(0x1 << 25)
#define ASPEED_ADC_IER_CH8				(0x1 << 24)
#define ASPEED_ADC_IER_CH7				(0x1 << 23)
#define ASPEED_ADC_IER_CH6				(0x1 << 22)
#define ASPEED_ADC_IER_CH5				(0x1 << 21)
#define ASPEED_ADC_IER_CH4				(0x1 << 20)
#define ASPEED_ADC_IER_CH3				(0x1 << 19)
#define ASPEED_ADC_IER_CH2				(0x1 << 18)
#define ASPEED_ADC_IER_CH1				(0x1 << 17)
#define ASPEED_ADC_IER_CH0				(0x1 << 16)
#define ASPEED_ADC_STS_CH15			(0x1 << 15)
#define ASPEED_ADC_STS_CH14			(0x1 << 14)
#define ASPEED_ADC_STS_CH13			(0x1 << 13)
#define ASPEED_ADC_STS_CH12			(0x1 << 12)
#define ASPEED_ADC_STS_CH11			(0x1 << 11)
#define ASPEED_ADC_STS_CH10			(0x1 << 10)
#define ASPEED_ADC_STS_CH9				(0x1 << 9)
#define ASPEED_ADC_STS_CH8				(0x1 << 8)
#define ASPEED_ADC_STS_CH7				(0x1 << 7)
#define ASPEED_ADC_STS_CH6				(0x1 << 6)
#define ASPEED_ADC_STS_CH5				(0x1 << 5)
#define ASPEED_ADC_STS_CH4				(0x1 << 4)
#define ASPEED_ADC_STS_CH3				(0x1 << 3)
#define ASPEED_ADC_STS_CH2				(0x1 << 2)
#define ASPEED_ADC_STS_CH1				(0x1 << 1)
#define ASPEED_ADC_STS_CH0				(0x1)

/* ASPEED_ADC_VGA	: 0x08 - VGA Detect Control */
#define ASPEED_ADC_VGA_EN				(0x1 << 16)
#define ASPEED_ADC_VGA_DIV_MASK		(0x3ff)

/* ASPEED_ADC_CLK : 0x0c - ADC CLK Control */
#define ASPEED_ADC_CLK_PRE_DIV_MASK	(0x7fff << 17)
#define ASPEED_ADC_CLK_PRE_DIV			(0x1 << 17)
#define ASPEED_ADC_CLK_INVERT			(0x1 << 16)		//only for ast2300
#define ASPEED_ADC_CLK_DIV_MASK		(0x3ff)

#define ASPEED_ADC_H_CH_MASK			(0x3ff << 16)
#define ASPEED_ADC_L_CH_MASK			(0x3ff)

#define ASPEED_ADC_H_BOUND				(0x3ff << 16)
#define ASPEED_ADC_L_BOUND				(0x3ff)

#define ASPEED_ADC_HYSTER_EN			(0x1 << 31)

//only support in ast2500
/* ASPEED_ADC_CH16	 : 0xD0 - */
/* ASPEED_ADC_CH17	 : 0xD4 - */
#define ASPEED_TEMP_CH_RDY				(0x1 << 31)
#define ASPEED_GET_TEMP_A_MASK(x)		((x >>16) & 0xfff)
#define ASPEED_TEMP_CH_EN				(0x1 << 15)		
#define ASPEED_GET_TEMP_B_MASK(x)		(x & 0xfff)

/******************************************************************************************/
extern u8 ast_scu_adc_trim_read(void);

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

struct aspeed_adc_config {
	u8		adc_version;	
	u8		adc_ch_num;
	u8		tmper_ch_num;
};

struct aspeed_adc_data {
	struct device			*dev;
	void __iomem			*reg_base;			/* virtual */
	int 	irq;			//ADC IRQ number 
	struct aspeed_adc_config	*config;
	int	compen_value;		//Compensating value
	struct reset_control *reset;	
	struct clk 			*clk;
	u32					pclk;
};

static inline void
aspeed_adc_write(struct aspeed_adc_data *aspeed_adc, u32 val, u32 reg)
{
	dev_dbg(aspeed_adc->dev, "write offset: %x, val: %x \n",reg,val);
	writel(val, aspeed_adc->reg_base+ reg);
}

static inline u32
aspeed_adc_read(struct aspeed_adc_data *aspeed_adc, u32 reg)
{
	u32 val = readl(aspeed_adc->reg_base + reg);
	dev_dbg(aspeed_adc->dev, "read offset: %x, val: %x \n",reg, val);
	return val;
}

static void aspeed_g5_adc_ctrl_init(struct aspeed_adc_data *aspeed_adc)
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

	aspeed_adc_write(aspeed_adc, trim, ASPEED_ADC_COMP_TRIM);

	aspeed_adc_write(aspeed_adc, 0x40, ASPEED_ADC_CLK);

	aspeed_adc_write(aspeed_adc, ASPEED_ADC_CTRL_NORMAL | ASPEED_ADC_CTRL_EN, ASPEED_ADC_CTRL);

	while(!(aspeed_adc_read(aspeed_adc, ASPEED_ADC_CTRL) & ASPEED_ADC_CTRL_INIT_RDY));

#if 0
	aspeed_adc_write(aspeed_adc, ASPEED_ADC_CTRL_AUTO_COMPEN  | ASPEED_ADC_CTRL_NORMAL | 
							ASPEED_ADC_CTRL_EN, ASPEED_ADC_CTRL);

	while(aspeed_adc_read(aspeed_adc, ASPEED_ADC_CTRL) & ASPEED_ADC_CTRL_AUTO_COMPEN);

	//compensating value = 0x200 - ADC10[9:0]
	aspeed_adc->compen_value = 0x200 - ((aspeed_adc_read(aspeed_adc, ASPEED_ADC_COMP_TRIM) >> 16) & 0x3ff);
	dev_dbg(aspeed_adc->dev, "compensating value %d \n",aspeed_adc->compen_value);
#else
	aspeed_adc_write(aspeed_adc, ASPEED_ADC_CTRL_CH0_EN | ASPEED_ADC_CTRL_COMPEN | 
							ASPEED_ADC_CTRL_NORMAL | ASPEED_ADC_CTRL_EN, 
							ASPEED_ADC_CTRL);

	aspeed_adc_read(aspeed_adc, ASPEED_ADC_CTRL);

	mdelay(1);

	//compensating value = 0x200 - ADC10[9:0]
	aspeed_adc->compen_value = 0x200 - (aspeed_adc_read(aspeed_adc, ASPEED_ADC_CH0_1) & ASPEED_ADC_L_CH_MASK);
	dev_dbg(aspeed_adc->dev, "compensating value %d \n",aspeed_adc->compen_value);

	aspeed_adc_write(aspeed_adc, ~(ASPEED_ADC_CTRL_COMPEN | ASPEED_ADC_CTRL_CH0_EN) & aspeed_adc_read(aspeed_adc, ASPEED_ADC_CTRL), ASPEED_ADC_CTRL);	

#endif
		
}

static void aspeed_g4_adc_ctrl_init(struct aspeed_adc_data *aspeed_adc)
{
	//Compensating Sensing Mode
	//Set wait a sensing cycle t (s) = 12 * (1/PCLK) * 2 * (ADC0c[31:17] + 1) * (ADC0c[9:0] +1)
	//ex : pclk = 48Mhz , ADC0c[31:17] = 0,  ADC0c[9:0] = 0x40 : 64,  ADC0c[31:17] = 0x3e7 : 999 
	// --> 0.0325s	= 12 * 2 * (0x3e7 + 1) *(64+1) / 48000000
	// --> 0.0005s	= 12 * 2 * (0x3e7 + 1) / 48000000	
	//For AST2400 A0 workaround  ... ADC0c = 1 ;
//	aspeed_adc_write(aspeed_adc, 1, ASPEED_ADC_CLK);
//	aspeed_adc_write(aspeed_adc, (0x3e7<< 17) | 0x40, ASPEED_ADC_CLK);
	aspeed_adc_write(aspeed_adc, 0x40, ASPEED_ADC_CLK);

	aspeed_adc_write(aspeed_adc, ASPEED_ADC_CTRL_CH0_EN | ASPEED_ADC_CTRL_COMPEN | 
							ASPEED_ADC_CTRL_NORMAL | ASPEED_ADC_CTRL_EN, 
							ASPEED_ADC_CTRL);

	aspeed_adc_read(aspeed_adc, ASPEED_ADC_CTRL);

	mdelay(1);

	//compensating value = 0x200 - ADC10[9:0]
	aspeed_adc->compen_value = 0x200 - (aspeed_adc_read(aspeed_adc, ASPEED_ADC_CH0_1) & ASPEED_ADC_L_CH_MASK);
	dev_dbg(aspeed_adc->dev, "compensating value %d \n",aspeed_adc->compen_value);

	aspeed_adc_write(aspeed_adc, ~ASPEED_ADC_CTRL_COMPEN & aspeed_adc_read(aspeed_adc, ASPEED_ADC_CTRL), ASPEED_ADC_CTRL);	
}

static void aspeed_g3_adc_ctrl_init(struct aspeed_adc_data *aspeed_adc)
{
	//Set wait a sensing cycle t (s) = 12 * (1/PCLK) * 2 * (ADC0c[31:17] + 1) * (ADC0c[9:0] +1)
	//ex : pclk = 48Mhz , ADC0c[31:17] = 0,  ADC0c[9:0] = 0x40 : 64,  ADC0c[31:17] = 0x3e7 : 999 
	// --> 0.0325s	= 12 * 2 * (0x3e7 + 1) *(64+1) / 48000000
	// --> 0.0005s	= 12 * 2 * (0x3e7 + 1) / 48000000	
	
	aspeed_adc_write(aspeed_adc, 0x3e7, ASPEED_ADC_CLK); 

	aspeed_adc_write(aspeed_adc, ASPEED_ADC_CTRL_CH12_EN | ASPEED_ADC_CTRL_COMPEN_CLR | 
							ASPEED_G3_ADC_CTRL_COMPEN | ASPEED_ADC_CTRL_NORMAL | 
							ASPEED_ADC_CTRL_EN, ASPEED_ADC_CTRL);

	mdelay(50);

	//compensating value = 0x200 - ADC10[9:0]
	if(aspeed_adc_read(aspeed_adc, ASPEED_ADC_CH12_13) & (0x1 << 8))
		aspeed_adc->compen_value = 0x200 - (aspeed_adc_read(aspeed_adc, ASPEED_ADC_CH12_13) & ASPEED_ADC_L_CH_MASK);
	else
		aspeed_adc->compen_value = 0 - (aspeed_adc_read(aspeed_adc, ASPEED_ADC_CH12_13) & ASPEED_ADC_L_CH_MASK);

	dev_dbg(aspeed_adc->dev, "compensating value %d \n",aspeed_adc->compen_value);

	aspeed_adc_write(aspeed_adc, ~ASPEED_G3_ADC_CTRL_COMPEN & aspeed_adc_read(aspeed_adc, ASPEED_ADC_CTRL), ASPEED_ADC_CTRL);	
}

static u16
aspeed_get_adc_hyster_lower(struct aspeed_adc_data *aspeed_adc, u8 adc_ch)
{
    u16 tmp=0;
	tmp = aspeed_adc_read(aspeed_adc, ASPEED_ADC_HYSTER0 + (adc_ch *4)) & ASPEED_ADC_L_BOUND;

	dev_dbg(aspeed_adc->dev, "read val = %d \n",tmp);

	return tmp;

}

static void
aspeed_set_adc_hyster_lower(struct aspeed_adc_data *aspeed_adc, u8 adc_ch, u16 value)
{
	aspeed_adc_write(aspeed_adc, 
			(aspeed_adc_read(aspeed_adc, ASPEED_ADC_HYSTER0 + (adc_ch *4)) & ~ASPEED_ADC_L_BOUND) |
			value, 
			ASPEED_ADC_HYSTER0 + (adc_ch *4));

}

static u16
aspeed_get_adc_hyster_upper(struct aspeed_adc_data *aspeed_adc, u8 adc_ch)
{
    u16 tmp=0;
	tmp = ((aspeed_adc_read(aspeed_adc, ASPEED_ADC_HYSTER0 + (adc_ch *4)) & ASPEED_ADC_H_BOUND) >> 16);

	dev_dbg(aspeed_adc->dev, "read val = %d \n",tmp);

	return tmp;
}

static void
aspeed_set_adc_hyster_upper(struct aspeed_adc_data *aspeed_adc, u8 adc_ch, u32 value)
{
	aspeed_adc_write(aspeed_adc, 
			(aspeed_adc_read(aspeed_adc, ASPEED_ADC_HYSTER0 + (adc_ch *4)) & ~ASPEED_ADC_H_BOUND) |
			(value << 16), 
			ASPEED_ADC_HYSTER0 + (adc_ch *4));

}

static u8
aspeed_get_adc_hyster_en(struct aspeed_adc_data *aspeed_adc, u8 adc_ch)
{
	//tacho source
	if(aspeed_adc_read(aspeed_adc, ASPEED_ADC_HYSTER0 + (adc_ch *4)) & ASPEED_ADC_HYSTER_EN)
		return 1;
	else
		return 0;
}

static void
aspeed_set_adc_hyster_en(struct aspeed_adc_data *aspeed_adc, u8 adc_ch, u8 enable)
{
	//tacho source 
	if(enable == 1)
		aspeed_adc_write(aspeed_adc,
			aspeed_adc_read(aspeed_adc, ASPEED_ADC_HYSTER0 + (adc_ch *4)) | ASPEED_ADC_HYSTER_EN,
			ASPEED_ADC_HYSTER0 + (adc_ch *4));
	else
		aspeed_adc_write(aspeed_adc,
			aspeed_adc_read(aspeed_adc, ASPEED_ADC_HYSTER0 + (adc_ch *4)) & ~ASPEED_ADC_HYSTER_EN,
			ASPEED_ADC_HYSTER0 + (adc_ch *4));
}

static u16
aspeed_get_adc_lower(struct aspeed_adc_data *aspeed_adc, u8 adc_ch)
{
    u16 tmp=0;
	tmp = aspeed_adc_read(aspeed_adc, ASPEED_ADC_BOUND0 + (adc_ch *4)) & ASPEED_ADC_L_BOUND;

	dev_dbg(aspeed_adc->dev, "read val = %d \n",tmp);

	return tmp;

}

static void
aspeed_set_adc_lower(struct aspeed_adc_data *aspeed_adc, u8 adc_ch, u16 value)
{
	aspeed_adc_write(aspeed_adc, 
			(aspeed_adc_read(aspeed_adc, ASPEED_ADC_BOUND0 + (adc_ch *4)) & ~ASPEED_ADC_L_BOUND) |
			value, 
			ASPEED_ADC_BOUND0 + (adc_ch *4));

}

static u16
aspeed_get_adc_upper(struct aspeed_adc_data *aspeed_adc, u8 adc_ch)
{
    u16 tmp=0;
	tmp = ((aspeed_adc_read(aspeed_adc, ASPEED_ADC_BOUND0 + (adc_ch *4)) & ASPEED_ADC_H_BOUND) >> 16);

	dev_dbg(aspeed_adc->dev, "read val = %d \n",tmp);

	return tmp;


}

static void
aspeed_set_adc_upper(struct aspeed_adc_data *aspeed_adc, u8 adc_ch, u32 value)
{
	aspeed_adc_write(aspeed_adc, 
			(aspeed_adc_read(aspeed_adc, ASPEED_ADC_BOUND0 + (adc_ch *4)) & ~ASPEED_ADC_H_BOUND) |
			(value << 16), 
			ASPEED_ADC_BOUND0 + (adc_ch *4));

}


static u8
aspeed_get_adc_alarm(struct aspeed_adc_data *aspeed_adc, u8 adc_ch)
{
	//adc ch source 
	if(aspeed_adc_read(aspeed_adc, ASPEED_ADC_IER) & (0x1 << adc_ch))
		return 1;
	else
		return 0;
}

static u16
aspeed_get_adc_value(struct aspeed_adc_data *aspeed_adc, u8 adc_ch)
{
    int tmp = 0;

	switch(adc_ch) {
		case 0:
			tmp = aspeed_adc_read(aspeed_adc, ASPEED_ADC_CH0_1) & ASPEED_ADC_L_CH_MASK;
			break;
		case 1:
			tmp = (aspeed_adc_read(aspeed_adc, ASPEED_ADC_CH0_1) & ASPEED_ADC_H_CH_MASK) >> 16;
			break;	
		case 2:
			tmp = aspeed_adc_read(aspeed_adc, ASPEED_ADC_CH2_3) & ASPEED_ADC_L_CH_MASK;
			break;
		case 3:
			tmp = (aspeed_adc_read(aspeed_adc, ASPEED_ADC_CH2_3) & ASPEED_ADC_H_CH_MASK) >> 16;
			break;	
		case 4:
			tmp = aspeed_adc_read(aspeed_adc, ASPEED_ADC_CH4_5) & ASPEED_ADC_L_CH_MASK;
			break;
		case 5:
			tmp = (aspeed_adc_read(aspeed_adc, ASPEED_ADC_CH4_5) & ASPEED_ADC_H_CH_MASK) >> 16;
			break;	
		case 6:
			tmp = aspeed_adc_read(aspeed_adc, ASPEED_ADC_CH6_7) & ASPEED_ADC_L_CH_MASK;
			break;
		case 7:
			tmp = (aspeed_adc_read(aspeed_adc, ASPEED_ADC_CH6_7) & ASPEED_ADC_H_CH_MASK) >> 16;
			break;	
		case 8:
			tmp = aspeed_adc_read(aspeed_adc, ASPEED_ADC_CH8_9) & ASPEED_ADC_L_CH_MASK;
			break;
		case 9:
			tmp = (aspeed_adc_read(aspeed_adc, ASPEED_ADC_CH8_9) & ASPEED_ADC_H_CH_MASK) >> 16;
			break;	
		case 10:
			tmp = aspeed_adc_read(aspeed_adc, ASPEED_ADC_CH10_11) & ASPEED_ADC_L_CH_MASK;
			break;
		case 11:
			tmp = (aspeed_adc_read(aspeed_adc, ASPEED_ADC_CH10_11) & ASPEED_ADC_H_CH_MASK) >> 16;
			break;	
		case 12:
			tmp = aspeed_adc_read(aspeed_adc, ASPEED_ADC_CH12_13) & ASPEED_ADC_L_CH_MASK;
			break;
		case 13:
			tmp = (aspeed_adc_read(aspeed_adc, ASPEED_ADC_CH12_13) & ASPEED_ADC_H_CH_MASK) >> 16;
			break;	
		case 14:
			tmp = aspeed_adc_read(aspeed_adc, ASPEED_ADC_CH14_15) & ASPEED_ADC_L_CH_MASK;
			break;
		case 15:
			tmp = (aspeed_adc_read(aspeed_adc, ASPEED_ADC_CH14_15) & ASPEED_ADC_H_CH_MASK) >> 16;
			break;	

	}

	tmp += aspeed_adc->compen_value;
	if(tmp < 0)
		tmp = 0;

	dev_dbg(aspeed_adc->dev, "voltage raw = %d \n",tmp);
	
	return tmp;

}

static u8 
aspeed_get_adc_en(struct aspeed_adc_data *aspeed_adc, u8 adc_ch)
{
    u8 tmp=0;

	if(aspeed_adc_read(aspeed_adc, ASPEED_ADC_CTRL) & (0x1 << (16+adc_ch)))
		tmp = 1;
	else
		tmp = 0;

	return tmp;

}

static void 
aspeed_set_adc_en(struct aspeed_adc_data *aspeed_adc, u8 adc_ch, u8 enable)
{
	if(enable)
		aspeed_adc_write(aspeed_adc, aspeed_adc_read(aspeed_adc, ASPEED_ADC_CTRL) | (0x1 << (16+adc_ch)), ASPEED_ADC_CTRL);
	else
		aspeed_adc_write(aspeed_adc, aspeed_adc_read(aspeed_adc, ASPEED_ADC_CTRL) & ~(0x1 << (16+adc_ch)), ASPEED_ADC_CTRL);
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
aspeed_show_adc(struct device *dev, struct device_attribute *attr, char *sysfsbuf)
{
	struct aspeed_adc_data *aspeed_adc = dev_get_drvdata(dev);

	struct sensor_device_attribute_2 *sensor_attr = to_sensor_dev_attr_2(attr);
	u16 tmp;
	u32 voltage,tmp1, tmp2,tmp3;

	//sensor_attr->index : pwm_ch#
	//sensor_attr->nr : attr#
	switch(sensor_attr->nr) 
	{
		case 0: //channel enable, disable
			return sprintf(sysfsbuf, "%d : %s\n", aspeed_get_adc_en(aspeed_adc, sensor_attr->index),aspeed_get_adc_en(aspeed_adc,sensor_attr->index) ? "Enable":"Disable");
			break;
		case 1: //value
			tmp = aspeed_get_adc_value(aspeed_adc, sensor_attr->index);
			return sprintf(sysfsbuf, "%d \n",tmp);
			break;
		case 2: //alarm
			return sprintf(sysfsbuf, "%d \n", aspeed_get_adc_alarm(aspeed_adc,sensor_attr->index));
			break;			
		case 3: //upper
			return sprintf(sysfsbuf, "%d \n", aspeed_get_adc_upper(aspeed_adc,sensor_attr->index));
			break;			
		case 4: //lower
			return sprintf(sysfsbuf, "%d \n", aspeed_get_adc_lower(aspeed_adc,sensor_attr->index));
			break;			
		case 5: //hystersis enable 
			return sprintf(sysfsbuf, "%d : %s\n", aspeed_get_adc_hyster_en(aspeed_adc,sensor_attr->index),aspeed_get_adc_hyster_en(aspeed_adc,sensor_attr->index) ? "Enable":"Disable");
			break;			
		case 6: //hystersis upper
			return sprintf(sysfsbuf, "%d \n", aspeed_get_adc_hyster_upper(aspeed_adc,sensor_attr->index));
			break;			
		case 7: //hystersis lower
			return sprintf(sysfsbuf, "%d \n", aspeed_get_adc_hyster_lower(aspeed_adc,sensor_attr->index));
			break;			
		case 8: //voltage
			tmp = aspeed_get_adc_value(aspeed_adc, sensor_attr->index);		
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
aspeed_store_adc(struct device *dev, struct device_attribute *attr, const char *sysfsbuf, size_t count)
{
	u32 input_val;
	struct aspeed_adc_data *aspeed_adc = dev_get_drvdata(dev);
	struct sensor_device_attribute_2 *sensor_attr =
						to_sensor_dev_attr_2(attr);

	input_val = simple_strtoul(sysfsbuf, NULL, 10);

	//sensor_attr->index : pwm_ch#
	//sensor_attr->nr : attr#
	switch(sensor_attr->nr) 
	{
		case 0: //enable, disable
			aspeed_set_adc_en(aspeed_adc, sensor_attr->index, input_val);
			break;
		case 1: //value
			
			break;
		case 2: //alarm
			break;			
		case 3:
			aspeed_set_adc_upper(aspeed_adc, sensor_attr->index, input_val);
			break;
		case 4:
			aspeed_set_adc_lower(aspeed_adc, sensor_attr->index, input_val);
			break;
		case 5: //hystersis
			aspeed_set_adc_hyster_en(aspeed_adc, sensor_attr->index, input_val);
			break;			
		case 6:
			aspeed_set_adc_hyster_upper(aspeed_adc, sensor_attr->index, input_val);
			break;
		case 7:
			aspeed_set_adc_hyster_lower(aspeed_adc, sensor_attr->index, input_val);
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
aspeed_get_temper_value(struct aspeed_adc_data *aspeed_adc, u8 temper_ch)
{
    u16 temper;
	dev_dbg(aspeed_adc->dev, "aspeed_get_temper_value %d \n",temper_ch);

	switch(temper_ch) {
		case 0:
			aspeed_adc_write(aspeed_adc, aspeed_adc_read(aspeed_adc, ASPEED_ADC_CH16) | ASPEED_TEMP_CH_EN, ASPEED_ADC_CH16);
			while(!(aspeed_adc_read(aspeed_adc, ASPEED_ADC_CH16) & ASPEED_TEMP_CH_RDY)); 
			//A-B 
			temper = ASPEED_GET_TEMP_A_MASK(aspeed_adc_read(aspeed_adc, ASPEED_ADC_CH16)) - 
					ASPEED_GET_TEMP_B_MASK(aspeed_adc_read(aspeed_adc, ASPEED_ADC_CH16));
			temper = ((temper - 247) * 100 ) / (320 - 247);
			break;
		case 1:
			aspeed_adc_write(aspeed_adc, aspeed_adc_read(aspeed_adc, ASPEED_ADC_CH17) | ASPEED_TEMP_CH_EN, ASPEED_ADC_CH17);
			while(!(aspeed_adc_read(aspeed_adc, ASPEED_ADC_CH17) & ASPEED_TEMP_CH_RDY)); 
			//A-B 
			temper = ASPEED_GET_TEMP_A_MASK(aspeed_adc_read(aspeed_adc, ASPEED_ADC_CH17)) - 
					ASPEED_GET_TEMP_B_MASK(aspeed_adc_read(aspeed_adc, ASPEED_ADC_CH17));

			temper = ((temper - 247) * 100 ) / (320 - 247);
			break;	
	}

	return temper;

}

static ssize_t 
aspeed_show_temper(struct device *dev, struct device_attribute *attr, char *sysfsbuf)
{
	struct aspeed_adc_data *aspeed_adc = dev_get_drvdata(dev);
	struct sensor_device_attribute_2 *sensor_attr = to_sensor_dev_attr_2(attr);

	dev_dbg(aspeed_adc->dev, "aspeed_show_temper %d \n",sensor_attr->nr);
	//sensor_attr->index : pwm_ch#
	//sensor_attr->nr : attr#
	switch(sensor_attr->nr) 
	{
		case 0: //value
			return sprintf(sysfsbuf, "%d\n",aspeed_get_temper_value(aspeed_adc, sensor_attr->index));
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
	aspeed_show_temper, NULL, 0, index); \
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
	aspeed_show_adc, aspeed_store_adc, 0, index); \
\
static SENSOR_DEVICE_ATTR_2(adc##index##_value, S_IRUGO | S_IWUSR, \
	aspeed_show_adc, NULL, 1, index); \
\
static SENSOR_DEVICE_ATTR_2(adc##index##_alarm, S_IRUGO | S_IWUSR, \
	aspeed_show_adc, NULL, 2, index); \
\
static SENSOR_DEVICE_ATTR_2(adc##index##_upper, S_IRUGO | S_IWUSR, \
	aspeed_show_adc, aspeed_store_adc, 3, index); \
\
static SENSOR_DEVICE_ATTR_2(adc##index##_lower, S_IRUGO | S_IWUSR, \
	aspeed_show_adc, aspeed_store_adc, 4, index); \
\
static SENSOR_DEVICE_ATTR_2(adc##index##_hyster_en, S_IRUGO | S_IWUSR, \
	aspeed_show_adc, aspeed_store_adc, 5, index); \
\
static SENSOR_DEVICE_ATTR_2(adc##index##_hyster_upper, S_IRUGO | S_IWUSR, \
	aspeed_show_adc, aspeed_store_adc, 6, index); \
\
static SENSOR_DEVICE_ATTR_2(adc##index##_hyster_lower, S_IRUGO | S_IWUSR, \
	aspeed_show_adc, aspeed_store_adc, 7, index); \
\
static SENSOR_DEVICE_ATTR_2(adc##index##_voltage, S_IRUGO | S_IWUSR, \
	aspeed_show_adc, NULL, 8, index); \
\
static SENSOR_DEVICE_ATTR_2(adc##index##_r1, S_IRUGO | S_IWUSR, \
	aspeed_show_adc, aspeed_store_adc, 9, index); \
\
static SENSOR_DEVICE_ATTR_2(adc##index##_r2, S_IRUGO | S_IWUSR, \
	aspeed_show_adc, aspeed_store_adc, 10, index); \
\
static SENSOR_DEVICE_ATTR_2(adc##index##_v2, S_IRUGO | S_IWUSR, \
	aspeed_show_adc, aspeed_store_adc, 11, index); \
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
	struct aspeed_adc_data *aspeed_adc = dev_get_drvdata(dev);

	aspeed_adc->compen_value = simple_strtoul(sysfsbuf, NULL, 10);
	return count;
}

static ssize_t 
show_compen_value(struct device *dev, struct device_attribute *attr, char *sysfsbuf)
{
	struct aspeed_adc_data *aspeed_adc = dev_get_drvdata(dev);
	
	return sprintf(sysfsbuf, "%d \n", aspeed_adc->compen_value);
}

static DEVICE_ATTR(adc_compen_value, S_IRUGO | S_IWUSR, show_compen_value, store_compen_value);


static struct attribute *aspeed_adc_attributes[] = {
	&dev_attr_adc_compen_value.attr,
	NULL
};

static const struct attribute_group aspeed_adc_attribute = {
	.attrs = aspeed_adc_attributes,
};

static const struct aspeed_adc_config ast2300_config = { 
	.adc_version = 3, 
	.adc_ch_num = 12, 
	.tmper_ch_num = 0, 
};

static const struct aspeed_adc_config ast2400_config = { 
	.adc_version = 4, 
	.adc_ch_num =16, 
	.tmper_ch_num = 0, 
};

static const struct aspeed_adc_config ast2500_config = { 
	.adc_version = 5, 
	.adc_ch_num =16, 
	.tmper_ch_num = 2, 
};

static const struct of_device_id aspeed_adc_matches[] = {
	{ .compatible = "aspeed,ast2300-adc",	.data = &ast2300_config, },
	{ .compatible = "aspeed,ast2400-adc",	.data = &ast2400_config, },
	{ .compatible = "aspeed,ast2500-adc",	.data = &ast2500_config, },	
	{},
};

MODULE_DEVICE_TABLE(of, aspeed_adc_matches);

static int 
aspeed_adc_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct aspeed_adc_data *aspeed_adc;
	const struct of_device_id *adc_dev_id;	
	int err;
	int ret=0;
	int i;

	dev_dbg(&pdev->dev, "aspeed_adc_probe \n");

	aspeed_adc = devm_kzalloc(&pdev->dev, sizeof(struct aspeed_adc_data), GFP_KERNEL);
	if (!aspeed_adc) {
		ret = -ENOMEM;
		goto out;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (NULL == res) {
		dev_err(&pdev->dev, "cannot get IORESOURCE_MEM\n");
		ret = -ENOENT;
		goto out_mem;
	}

	aspeed_adc->reg_base = devm_ioremap_resource(&pdev->dev, res);
	if (!aspeed_adc->reg_base) {
		ret = -EIO;
		goto out_region;
	}

	aspeed_adc->irq = platform_get_irq(pdev, 0);
	if (aspeed_adc->irq < 0) {
		dev_err(&pdev->dev, "no irq specified\n");
		ret = -ENOENT;
		goto out_region;
	}

	aspeed_adc->reset = devm_reset_control_get(&pdev->dev, NULL);
	if (IS_ERR(aspeed_adc->reset)) {
		dev_err(&pdev->dev, "can't get adc reset\n");
		return PTR_ERR(aspeed_adc->reset);
	}

	//scu init
	reset_control_assert(aspeed_adc->reset);
	reset_control_deassert(aspeed_adc->reset);

	aspeed_adc->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(aspeed_adc->clk)) {
		dev_err(&pdev->dev, "no clock defined\n");
		return -ENODEV;
	}
	aspeed_adc->pclk = clk_get_rate(aspeed_adc->clk);

	adc_dev_id = of_match_node(aspeed_adc_matches, pdev->dev.of_node);
	if (!adc_dev_id)
		return -EINVAL;

	aspeed_adc->config = (struct aspeed_adc_config *) adc_dev_id->data;

	/* Register sysfs hooks */
	aspeed_adc->dev = hwmon_device_register(&pdev->dev);
	if (IS_ERR(aspeed_adc->dev)) {
		ret = PTR_ERR(aspeed_adc->dev);
		goto out_region;
	}

	platform_set_drvdata(pdev, aspeed_adc);

	for(i=0; i<aspeed_adc->config->adc_ch_num; i++) {
		err = sysfs_create_group(&pdev->dev.kobj, &adc_attribute_groups[i]);
		if (err)
			goto out_region;
	}

	if(aspeed_adc->config->tmper_ch_num) {
		for(i=0; i<aspeed_adc->config->tmper_ch_num; i++) {
			err = sysfs_create_group(&pdev->dev.kobj, &temper_attribute_groups[i]);
			if (err)
				goto out_region;
		}
	}

	ret = sysfs_create_group(&pdev->dev.kobj, &aspeed_adc_attribute);
	if (ret)
		goto out_region;

	if(aspeed_adc->config->adc_version == 3) {
		aspeed_g3_adc_ctrl_init(aspeed_adc);
	} else if (aspeed_adc->config->adc_version == 4) {
		aspeed_g4_adc_ctrl_init(aspeed_adc);
	} else if(aspeed_adc->config->adc_version == 5) {
		aspeed_g5_adc_ctrl_init(aspeed_adc);
	} else {
		dev_err(&pdev->dev, "adc config error \n");	
		return -ENODEV;
	}
	
	printk(KERN_INFO "aspeed_adc: driver successfully loaded.\n");

	return 0;


//out_irq:
//	free_irq(aspeed_adc->irq, NULL);
out_region:
	release_mem_region(res->start, res->end - res->start + 1);
out_mem:
	kfree(aspeed_adc);
out:
	printk(KERN_WARNING "aspeed_adc: driver init failed (ret=%d)!\n", ret);
	return ret;
}

static int 
aspeed_adc_remove(struct platform_device *pdev)
{
	int i=0;
	struct aspeed_adc_data *aspeed_adc = platform_get_drvdata(pdev);
	struct resource *res;
	printk(KERN_INFO "aspeed_adc: driver unloaded.\n");

	hwmon_device_unregister(aspeed_adc->dev);

	for(i=0; i<aspeed_adc->config->adc_ch_num; i++)
		sysfs_remove_group(&pdev->dev.kobj, &adc_attribute_groups[i]);

	if(aspeed_adc->config->tmper_ch_num) {
		for(i=0; i<aspeed_adc->config->tmper_ch_num; i++)
			sysfs_remove_group(&pdev->dev.kobj, &temper_attribute_groups[i]);
	}

	platform_set_drvdata(pdev, NULL);
//	free_irq(aspeed_adc->irq, aspeed_adc);
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	iounmap(aspeed_adc->reg_base);
	release_mem_region(res->start, res->end - res->start + 1);
	kfree(aspeed_adc);
	return 0;
}

#ifdef CONFIG_PM
static int 
aspeed_adc_suspend(struct platform_device *pdev, pm_message_t state)
{
	printk("aspeed_adc_suspend : TODO \n");
	return 0;
}

static int 
aspeed_adc_resume(struct platform_device *pdev)
{
	struct aspeed_adc_data *aspeed_adc = platform_get_drvdata(pdev);

	return 0;
}

#else
#define aspeed_adc_suspend        NULL
#define aspeed_adc_resume         NULL
#endif

static struct platform_driver aspeed_adc_driver = {
	.probe 		= aspeed_adc_probe,
	.remove 		= aspeed_adc_remove,
	.suspend        = aspeed_adc_suspend,
	.resume         = aspeed_adc_resume,
	.driver         = {
		.name   = KBUILD_MODNAME,
		.of_match_table = aspeed_adc_matches,
	},
};

module_platform_driver(aspeed_adc_driver);

MODULE_AUTHOR("Ryan Chen <ryan_chen@aspeedtech.com>");
MODULE_DESCRIPTION("ASPEED ADC driver");
MODULE_LICENSE("GPL");
