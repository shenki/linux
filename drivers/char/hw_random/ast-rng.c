/*
 * ast-rng.c - RNG driver for the Aspeed SoC
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
#include <linux/module.h>
#include <linux/init.h>
#include <linux/random.h>
#include <linux/platform_device.h>
#include <linux/hw_random.h>

#include <linux/ast-scu.h>

static int ast_rng_data_read(struct hwrng *rng, u32 *data)
{
	*data = ast_scu_hw_random_read();
//	printk("rng = %08x \n",*data);
	return 4;
}

static struct hwrng ast_rng_ops = {
	.name		= "ast",
	.data_read	= ast_rng_data_read,
};


static ssize_t show_rng_type(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "Type : %d \n", ast_scu_get_hw_random_type());
}

static ssize_t store_rng_type(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	u8 val;	
	val = simple_strtoul(buf, NULL, 5);
	ast_scu_set_hw_random_type(val);
	
	return count;
}

static DEVICE_ATTR(type, S_IRUGO | S_IWUSR, show_rng_type, store_rng_type); 

static struct attribute *rng_sysfs_entries[] = {
	&dev_attr_type.attr,		
	NULL
};

static struct attribute_group rng_attribute_group = {
	.attrs = rng_sysfs_entries,
};

static int ast_rng_probe(struct platform_device *pdev)
{
	int ret = hwrng_register(&ast_rng_ops);

	if (ret)
		return ret;

	ast_scu_hw_random_enable(1);

	ret = sysfs_create_group(&pdev->dev.kobj, &rng_attribute_group);
	if (ret) {
		printk(KERN_ERR "ast_jtag: failed to create sysfs device attributes.\n");
		return -1;
	}
	return 0;
}

static int ast_rng_remove(struct platform_device *pdev)
{
	hwrng_unregister(&ast_rng_ops);

	ast_scu_hw_random_enable(0);

	return 0;
}

#ifdef CONFIG_PM
static int ast_rng_suspend(struct device *dev)
{
	ast_scu_hw_random_enable(0);
	return 0;
}

static int ast_rng_resume(struct device *dev)
{
	ast_scu_hw_random_enable(1);
	return 0;
}
static const struct dev_pm_ops ast_rng_pm_ops = {
        .suspend        = ast_rng_suspend,
        .resume         = ast_rng_resume,
};
#endif

static const struct of_device_id ast_rng_dt_ids[] = {
        { .compatible = "aspeed,ast-rng" },
        { /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, ast_rng_dt_ids);

static struct platform_driver ast_rng_driver = {
	.probe          = ast_rng_probe,
	.remove         = ast_rng_remove,
	.driver         = {
		.name   = "ast-rng",
#ifdef CONFIG_PM			
		.pm     = &ast_rng_pm_ops,
#endif		
		.of_match_table = ast_rng_dt_ids,
	},
};

module_platform_driver(ast_rng_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Ryan Chen <ryan_chen@aspeedtech.com>");
MODULE_DESCRIPTION("ASPEED random number generator driver");
