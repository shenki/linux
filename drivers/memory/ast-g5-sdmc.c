/********************************************************************************
* File Name     : arch/arm/mach-aspeed/ast-sdmc.c 
* Author         : Ryan Chen
* Description   : AST SDRAM Memory Ctrl
* 
* Copyright (C) 2012-2020  ASPEED Technology Inc.
* This program is free software; you can redistribute it and/or modify 
* it under the terms of the GNU General Public License as published by the Free Software Foundation; 
* either version 2 of the License, or (at your option) any later version. 
* This program is distributed in the hope that it will be useful,  but WITHOUT ANY WARRANTY; 
* without even the implied warranty of MERCHANTABILITY or 
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details. 
* You should have received a copy of the GNU General Public License 
* along with this program; if not, write to the Free Software 
* Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA 


*   History      : 
*    1. 2013/03/15 Ryan Chen Create
* 
********************************************************************************/
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <asm/io.h>
#include <linux/hwmon-sysfs.h>

#include <mach/platform.h>
#include <mach/hardware.h>

#include <plat/ast-sdmc.h>
/************************************  Registers for SDMC ****************************************/ 
#define AST_SDMC_PROTECT				0x00		/*	protection key register	*/
#define AST_SDMC_CONFIG				0x04		/*	Configuration register */
#define AST_SDMC_MEM_REQ				0x08		/*	Graphics Memory Protection register */

#define AST_SDMC_ISR					0x50		/*	Interrupt Control/Status Register */

/*	AST_SDMC_PROTECT: 0x00  - protection key register */
#define SDMC_PROTECT_UNLOCK			0xFC600309

/*	AST_SDMC_CONFIG :0x04	 - Configuration register */
#define SDMC_CONFIG_VER_NEW			(0x1 << 28)
#define SDMC_CONFIG_MEM_GET(x)		(x & 0x3)

#define SDMC_CONFIG_CACHE_EN			(0x1 << 10)
#define SDMC_CONFIG_EEC_EN			(0x1 << 7)
#define SDMC_CONFIG_DDR4				(0x1 << 4)


/*	#define AST_SDMC_ISR	 : 0x50	- Interrupt Control/Status Register */
#define SDMC_ISR_CLR					(0x1 << 31)
#define SDMC_ISR_RW_ACCESS			(0x1 << 29)

#define SDMC_ISR_GET_ECC_RECOVER(x)	((x >> 16) & 0xff)
#define SDMC_ISR_GET_ECC_UNRECOVER(x)	((x >> 12) & 0xf)
/****************************************************************************************/

//#define AST_SDMC_LOCK
//#define AST_SDMC_DEBUG

#ifdef AST_SDMC_DEBUG
#define SDMCDBUG(fmt, args...) printk("%s() " fmt, __FUNCTION__, ## args)
#else
#define SDMCDBUG(fmt, args...)
#endif

#define SDMCMSG(fmt, args...) printk(fmt, ## args)
/****************************************************************************************************************/
void __iomem	*ast_sdmc_base = 0;

static inline u32 
ast_sdmc_read(u32 reg)
{
	u32 val;
		
	val = readl(ast_sdmc_base + reg);
	
	SDMCDBUG("ast_sdmc_read : reg = 0x%08x, val = 0x%08x\n", reg, val);
	
	return val;
}

static inline void
ast_sdmc_write(u32 val, u32 reg) 
{
	SDMCDBUG("ast_sdmc_write : reg = 0x%08x, val = 0x%08x\n", reg, val);
#ifdef CONFIG_AST_SDMC_LOCK
	//unlock 
	writel(SDMC_PROTECT_UNLOCK, ast_sdmc_base);
	writel(val, ast_sdmc_base + reg);
	//lock
	writel(0xaa,ast_sdmc_base);	
#else
	writel(SDMC_PROTECT_UNLOCK, ast_sdmc_base);

	writel(val, ast_sdmc_base + reg);
#endif
}
/****************************************************************************************************************/
extern u8
ast_sdmc_get_cache(void)
{
	if(ast_sdmc_read(AST_SDMC_CONFIG) & SDMC_CONFIG_CACHE_EN)
		return 1;
	else
		return 0;
}

extern void
ast_sdmc_set_cache(u8 enable)
{
	if(enable) 
		ast_sdmc_write(ast_sdmc_read(AST_SDMC_CONFIG) | SDMC_CONFIG_CACHE_EN, AST_SDMC_CONFIG);
	else
		ast_sdmc_write(ast_sdmc_read(AST_SDMC_CONFIG) & ~SDMC_CONFIG_CACHE_EN, AST_SDMC_CONFIG);
}

extern u8
ast_sdmc_get_ecc(void)
{
	if(ast_sdmc_read(AST_SDMC_CONFIG) & SDMC_CONFIG_EEC_EN)
		return 1;
	else
		return 0;
}

extern void
ast_sdmc_set_ecc(u8 enable)
{
	if(enable) 
		ast_sdmc_write(ast_sdmc_read(AST_SDMC_CONFIG) | SDMC_CONFIG_EEC_EN, AST_SDMC_CONFIG);
	else
		ast_sdmc_write(ast_sdmc_read(AST_SDMC_CONFIG) & ~SDMC_CONFIG_EEC_EN, AST_SDMC_CONFIG);	
}

extern u8
ast_sdmc_get_dram(void)
{
	if(ast_sdmc_read(AST_SDMC_CONFIG) & SDMC_CONFIG_DDR4)
		return 1;
	else
		return 0;
}

extern void
ast_sdmc_disable_mem_protection(u8 req)
{
	ast_sdmc_write(ast_sdmc_read(AST_SDMC_MEM_REQ) & ~(1<< req), AST_SDMC_MEM_REQ);
}

extern u32
ast_sdmc_get_mem_size(void)
{
	u32 size=0;
	u32 conf = ast_sdmc_read(AST_SDMC_CONFIG);
	
	if(conf & SDMC_CONFIG_VER_NEW) {
		switch(SDMC_CONFIG_MEM_GET(conf)) {
			case 0:
				size = 128*1024*1024;
				break;
			case 1:
				size = 256*1024*1024;
				break;
			case 2:
				size = 512*1024*1024;
				break;
			case 3:
				size = 1024*1024*1024;
				break;
				
			default:
				SDMCMSG("error ddr size \n");
				break;
		}
		
	} else {
		switch(SDMC_CONFIG_MEM_GET(conf)) {
			case 0:
				size = 64*1024*1024;
				break;
			case 1:
				size = 128*1024*1024;
				break;
			case 2:
				size = 256*1024*1024;
				break;
			case 3:
				size = 512*1024*1024;
				break;
				
			default:
				SDMCMSG("error ddr size \n");
				break;
		}
	}
	return size;
}

/************************************************** SYS FS **************************************************************/
static ssize_t show_cache(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d: %s\n", ast_sdmc_get_cache(), ast_sdmc_get_cache()? "Enable":"Disable");
}

static ssize_t store_cache(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	u32 val;	
	val = simple_strtoul(buf, NULL, 10);

	if(val)
		ast_sdmc_set_cache(1);
	else
		ast_sdmc_set_cache(0);
	
	return count;
}

static DEVICE_ATTR(cache, S_IRUGO | S_IWUSR, show_cache, store_cache); 

static ssize_t show_ecc(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d: %s\n", ast_sdmc_get_ecc(), ast_sdmc_get_ecc()? "Enable":"Disable");
}

static ssize_t store_ecc(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	u32 val;	

	val = simple_strtoul(buf, NULL, 10);

	if(val)
		ast_sdmc_set_ecc(1);
	else
		ast_sdmc_set_ecc(0);
	
	return count;
}

static DEVICE_ATTR(ecc, S_IRUGO | S_IWUSR, show_ecc, store_ecc); 

static ssize_t show_ecc_counter(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "recoverable count %d: un-recoverable count %d\n", 
			SDMC_ISR_GET_ECC_RECOVER(ast_sdmc_read(AST_SDMC_ISR)), 
			SDMC_ISR_GET_ECC_UNRECOVER(ast_sdmc_read(AST_SDMC_ISR)));
}

static ssize_t store_ecc_counter(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	u32 val;	

	val = simple_strtoul(buf, NULL, 10);

	if(val)
		ast_sdmc_write(ast_sdmc_read(AST_SDMC_ISR) | SDMC_ISR_CLR, AST_SDMC_ISR);
	
	return count;
}

static DEVICE_ATTR(ecc_counter, S_IRUGO | S_IWUSR, show_ecc_counter, store_ecc_counter); 

static ssize_t show_dram(struct device *dev,
	struct device_attribute *attr, char *buf)
{

	return sprintf(buf, "%d: %s\n", ast_sdmc_get_dram(), ast_sdmc_get_dram()? "DDR4":"DDR3");
}

static DEVICE_ATTR(dram, S_IRUGO, show_dram, NULL); 

static struct attribute *ast_sdmc_attributes[] = {
	&dev_attr_cache.attr,
	&dev_attr_ecc.attr,
	&dev_attr_ecc_counter.attr,	
	&dev_attr_dram.attr,	
	NULL
};
/************************************************** SYS FS **************************************************************/
static const struct attribute_group sdmc_attribute_group = {
	.attrs = ast_sdmc_attributes
};

//***********************************Information ***********************************
static struct kobject *ast_sdmc_kobj;

static int __init ast_sdmc_init(void)
{
	int ret;

	SDMCDBUG("\n");
	ast_sdmc_base = ioremap(AST_SDMC_BASE , SZ_256);

	//show in /sys/kernel/dram
	ast_sdmc_kobj = kobject_create_and_add("dram", kernel_kobj);
	if (!ast_sdmc_kobj)
		return -ENOMEM;
	ret = sysfs_create_group(ast_sdmc_kobj, &sdmc_attribute_group);
	if (ret)
			kobject_put(ast_sdmc_kobj);
	return ret;
}

core_initcall(ast_sdmc_init);
