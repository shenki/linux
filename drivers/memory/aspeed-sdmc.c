/*
 * ASPEED SDRAM Controller driver
 *
 * Copyright (C) ASPEED Technology Inc.
 * Ryan Chen <ryan_chen@aspeedtech.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/hwmon-sysfs.h>
#include <linux/aspeed-sdmc.h>

#define ASPEED_SDMC_PROTECT				0x00		/*	protection key register	*/
#define ASPEED_SDMC_CONFIG				0x04		/*	Configuration register */
#define ASPEED_SDMC_MEM_REQ				0x08		/*	Graphics Memory Protection register */

#define ASPEED_SDMC_ISR					0x50		/*	Interrupt Control/Status Register */

/*	ASPEED_SDMC_PROTECT: 0x00  - protection key register */
#define SDMC_PROTECT_UNLOCK			0xFC600309

/*	ASPEED_SDMC_CONFIG :0x04	 - Configuration register */
#define SDMC_CONFIG_VER_NEW			(0x1 << 28)
#define SDMC_CONFIG_MEM_GET(x)		(x & 0x3)

#define SDMC_CONFIG_CACHE_EN			(0x1 << 10)
#define SDMC_CONFIG_EEC_EN			(0x1 << 7)
#define SDMC_CONFIG_DDR4				(0x1 << 4)


/*	#define ASPEED_SDMC_ISR	 : 0x50	- Interrupt Control/Status Register */
#define SDMC_ISR_CLR					(0x1 << 31)
#define SDMC_ISR_RW_ACCESS			(0x1 << 29)

#define SDMC_ISR_GET_ECC_RECOVER(x)	((x >> 16) & 0xff)
#define SDMC_ISR_GET_ECC_UNRECOVER(x)	((x >> 12) & 0xf)

//#define ASPEED_SDMC_LOCK
//#define ASPEED_SDMC_DEBUG

#ifdef ASPEED_SDMC_DEBUG
#define SDMCDBUG(fmt, args...) printk("%s() " fmt, __FUNCTION__, ## args)
#else
#define SDMCDBUG(fmt, args...)
#endif

#define SDMCMSG(fmt, args...) printk(fmt, ## args)

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
#ifdef CONFIG_ASPEED_SDMC_LOCK
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

extern u8
ast_sdmc_get_ecc(void)
{
	if(ast_sdmc_read(ASPEED_SDMC_CONFIG) & SDMC_CONFIG_EEC_EN)
		return 1;
	else
		return 0;
}

#ifdef CONFIG_MACH_ASPEED_G5

extern u8
ast_sdmc_get_cache(void)
{
	if(ast_sdmc_read(ASPEED_SDMC_CONFIG) & SDMC_CONFIG_CACHE_EN)
		return 1;
	else
		return 0;
}

extern void
ast_sdmc_set_cache(u8 enable)
{
	if(enable) 
		ast_sdmc_write(ast_sdmc_read(ASPEED_SDMC_CONFIG) | SDMC_CONFIG_CACHE_EN, ASPEED_SDMC_CONFIG);
	else
		ast_sdmc_write(ast_sdmc_read(ASPEED_SDMC_CONFIG) & ~SDMC_CONFIG_CACHE_EN, ASPEED_SDMC_CONFIG);
}


extern u8
ast_sdmc_get_dram(void)
{
	if(ast_sdmc_read(ASPEED_SDMC_CONFIG) & SDMC_CONFIG_DDR4)
		return 1;
	else
		return 0;
}
#endif

extern void
ast_sdmc_disable_mem_protection(u8 req)
{
	ast_sdmc_write(ast_sdmc_read(ASPEED_SDMC_MEM_REQ) & ~(1<< req), ASPEED_SDMC_MEM_REQ);
}

extern u32
ast_sdmc_get_mem_size(void)
{
	u32 size = 0;
	u32 conf = ast_sdmc_read(ASPEED_SDMC_CONFIG);
	
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

EXPORT_SYMBOL(ast_sdmc_get_mem_size);

static ssize_t show_ecc(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d: %s\n", ast_sdmc_get_ecc(), ast_sdmc_get_ecc()? "Enable":"Disable");
}

static DEVICE_ATTR(ecc, S_IRUGO | S_IWUSR, show_ecc, NULL); 

static ssize_t show_ecc_counter(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "recoverable count %d: un-recoverable count %d\n", 
			SDMC_ISR_GET_ECC_RECOVER(ast_sdmc_read(ASPEED_SDMC_ISR)), 
			SDMC_ISR_GET_ECC_UNRECOVER(ast_sdmc_read(ASPEED_SDMC_ISR)));
}

static ssize_t store_ecc_counter(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	u32 val;	

	val = simple_strtoul(buf, NULL, 10);

	if(val)
		ast_sdmc_write(ast_sdmc_read(ASPEED_SDMC_ISR) | SDMC_ISR_CLR, ASPEED_SDMC_ISR);
	
	return count;
}

static DEVICE_ATTR(ecc_counter, S_IRUGO | S_IWUSR, show_ecc_counter, store_ecc_counter); 

#ifdef CONFIG_MACH_ASPEED_G5

static ssize_t show_cache(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d: %s\n", ast_sdmc_get_cache(), ast_sdmc_get_cache()? "Enable":"Disable");
}

static DEVICE_ATTR(cache, S_IRUGO | S_IWUSR, show_cache, NULL); 


static ssize_t show_dram(struct device *dev,
	struct device_attribute *attr, char *buf)
{

	return sprintf(buf, "%d: %s\n", ast_sdmc_get_dram(), ast_sdmc_get_dram()? "DDR4":"DDR3");
}

static DEVICE_ATTR(dram, S_IRUGO, show_dram, NULL); 

#endif

static struct attribute *ast_sdmc_attributes[] = {
	&dev_attr_ecc.attr,
	&dev_attr_ecc_counter.attr,	
#ifdef CONFIG_MACH_ASPEED_G5
	&dev_attr_cache.attr,
	&dev_attr_dram.attr,
#endif	
	NULL
};

static const struct attribute_group sdmc_attribute_group = {
	.attrs = ast_sdmc_attributes
};

static struct kobject *ast_sdmc_kobj;

static int aspeed_sdmc_probe(struct platform_device *pdev)
{
	struct resource *res;
	int ret;

	SDMCDBUG("\n");
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	ast_sdmc_base = devm_ioremap_resource(&pdev->dev, res);

	//show in /sys/kernel/dram
	ast_sdmc_kobj = kobject_create_and_add("dram", kernel_kobj);
	if (!ast_sdmc_kobj)
		return -ENOMEM;
	ret = sysfs_create_group(ast_sdmc_kobj, &sdmc_attribute_group);
	if (ret)
			kobject_put(ast_sdmc_kobj);
	return ret;
}

static const struct of_device_id aspeed_sdmc_of_match[] = {
	{ .compatible = "aspeed,ast2500-sdmc", },
	{ .compatible = "aspeed,ast2400-sdmc", },
	{ }
};

static struct platform_driver aspeed_sdmc_driver = {
	.probe = aspeed_sdmc_probe,
	.driver = {
		.name = KBUILD_MODNAME,
		.of_match_table = aspeed_sdmc_of_match,
	},
};

static int aspeed_sdmc_init(void)
{
	return platform_driver_register(&aspeed_sdmc_driver);
}

core_initcall(aspeed_sdmc_init);
