// SPDX-License-Identifier: GPL-2.0
/*
 * Device driver for regulators in MAX5970 and MAX5978 IC
 *
 * Copyright (c) 2022 9elements GmbH
 *
 * Author: Patrick Rudolph <patrick.rudolph@9elements.com>
 */

#include <linux/bitops.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/leds.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/i2c.h>
#include <linux/mfd/core.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>
#include <linux/version.h>
#include <linux/mfd/max597x.h>

static const struct regmap_config max597x_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = MAX_REGISTERS,
};

static const struct mfd_cell max597x_devs[] = {
	{
	 .name = "max597x-regulator",
	 },
	{
	 .name = "max597x-iio",
	 },
	{
	 .name = "max597x-led",
	 },
};

static int max597x_probe(struct i2c_client *i2c, const struct i2c_device_id *id)
{
	struct max597x_data *max597x;
	struct regmap *regmap;
	enum max597x_chip_type chip = id->driver_data;

	max597x =
	    devm_kzalloc(&i2c->dev, sizeof(struct max597x_data), GFP_KERNEL);
	switch (chip) {
	case MAX597x_TYPE_MAX5970:
		max597x->num_switches = 2;
		break;
	case MAX597x_TYPE_MAX5978:
		max597x->num_switches = 1;
		break;
	}

	regmap = devm_regmap_init_i2c(i2c, &max597x_regmap_config);
	if (IS_ERR(regmap)) {
		dev_err(&i2c->dev, "No regmap\n");
		return -EINVAL;
	}
	max597x->regmap = regmap;
	max597x->irq = i2c->irq;
	max597x->dev = &i2c->dev;
	i2c_set_clientdata(i2c, max597x);

	return devm_mfd_add_devices(max597x->dev, PLATFORM_DEVID_AUTO,
				    max597x_devs, ARRAY_SIZE(max597x_devs),
				    NULL, 0, NULL);
}

static const struct i2c_device_id max597x_table[] = {
	{.name = "max5970", MAX597x_TYPE_MAX5970},
	{.name = "max5978", MAX597x_TYPE_MAX5978},
	{},
};

MODULE_DEVICE_TABLE(i2c, max597x_table);

static const struct of_device_id max597x_of_match[] = {
	{	.compatible = "maxim,max5970",
		.data = (void *)MAX597x_TYPE_MAX5970
	},
	{	.compatible = "maxim,max5978",
		.data = (void *)MAX597x_TYPE_MAX5978
	},
	{},
};

MODULE_DEVICE_TABLE(of, max597x_of_match);

static struct i2c_driver max597x_driver = {
	.id_table = max597x_table,
	.driver = {
		   .name = "max597x",
		   .of_match_table = of_match_ptr(max597x_of_match),
		   },
	.probe = max597x_probe,
};

module_i2c_driver(max597x_driver);

MODULE_AUTHOR("Patrick Rudolph <patrick.rudolph@9elements.com>");
MODULE_DESCRIPTION("MAX5970_hot-swap controller driver");
MODULE_LICENSE("GPL v2");
