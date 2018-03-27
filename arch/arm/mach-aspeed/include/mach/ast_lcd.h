/*
* ast_lcd.h - ASPEED LCD Panel Timing 
*
* Copyright (C) ASPEED Technology Inc.
* Ryan Chen <ryan_chen@aspeedtech.com>
*
* This program is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public License
* as published by the Free Software Foundation; either version
* 2 of the License, or (at your option) any later version.
*/
#include <linux/fb.h>

//# Define IO __ for control 
#define YUV_MODE 0x4630
#define CHANGE_YUV_ADDR 0x4631
#define CHANGE_ADDR 0x4632
#define OVERSCAN 0x4634


enum astfb_color_format {
	ASTFB_COLOR_NONE = 0,
	ASTFB_COLOR_RGB565,
	ASTFB_COLOR_RGB888,
	ASTFB_COLOR_YUV444,
	ASTFB_COLOR_YUV420,
	ASTFB_COLOR_YUV422,	
};

struct aspeed_lcd_panel {
	struct fb_videomode	mode;
	signed short		width;	/* width in mm */
	signed short		height;	/* height in mm */
};

struct ast_monitor_info {
	int status; //0: no data   1:get data
	int type;   //0:dvi  1:hdmi
	struct fb_monspecs specs;
	char edid[256];
};

struct ast_fb_plat_data {
#ifdef AST_SOC_G5
	void (*set_pll)(u32 pll_setting);
	u32 clock_src;	//0: 24Mhz, 1: 25Mhz
#endif
	int disp_dev_no;
};

int ast_vga_get_info(struct fb_info *fb_info);
int ast_hdmi_get_info(struct fb_info *fb_info);
void ast_hdmi_enable(int en);
int vga_read_edid(void);
struct fb_info *astfb_get_crt_screen(u8 crt);

struct fb_info *astfb_get_fb_info(u8 crt);
u8 astfb_get_crt_color_format(struct fb_info *info);
u32 astfb_get_crt_fb_addr(struct fb_info *info);
u16 astfb_get_crt_fb_line_offset(struct fb_info *info);
