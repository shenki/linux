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
#include <linux/mfd/max597x.h>
#include <linux/regmap.h>
#include <linux/version.h>
#include <linux/platform_device.h>

struct max597x_led {
	struct regmap *regmap;
	struct led_classdev led;
	unsigned int index;
};

static int max597x_led_set_brightness(struct led_classdev *cdev,
				      enum led_brightness brightness)
{
	struct max597x_led *led = cdev->driver_data;
	int ret;

	if (!led || !led->regmap)
		return -1;

	ret = regmap_update_bits(led->regmap, MAX5970_REG_LED_FLASH,
				 1 << led->index, ~brightness << led->index);
	if (ret < 0)
		dev_err(cdev->dev, "failed to set brightness %d\n", ret);
	return ret;
}

static int max597x_led(struct max597x_data *max597x, struct device_node *nc, u32 reg)
{
	struct max597x_led *led;
	const char *state;
	int ret = 0;

	led = devm_kzalloc(max597x->dev, sizeof(struct max597x_led),
				   GFP_KERNEL);
	if (!led)
		return -ENOMEM;

	if (of_property_read_string(nc, "label", &led->led.name))
		led->led.name = nc->name;

	led->led.max_brightness = 1;
	led->led.brightness_set_blocking = max597x_led_set_brightness;
	led->led.default_trigger = "none";
	led->index = reg;
	led->regmap = max597x->regmap;
	ret = led_classdev_register(max597x->dev, &led->led);
	if (ret) {
		dev_err(max597x->dev, "Error in initializing led %s", led->led.name);
		devm_kfree(max597x->dev, led);
		return ret;
	}
	led->led.driver_data = led;
	led->led.dev = max597x->dev;
	if (!of_property_read_string(nc, "default-state", &state)) {
		if (!strcmp(state, "on")) {
			led->led.brightness = 1;
			led_set_brightness(&led->led, led->led.brightness);
		}
	}
	return 0;
}

static int max597x_led_probe(struct platform_device *pdev)
{


	struct max597x_data *max597x = dev_get_drvdata(pdev->dev.parent);
	struct device_node *np = dev_of_node(pdev->dev.parent);
	struct device_node *led_node;
	struct device_node *child;
	int ret = 0;


	led_node = of_get_child_by_name(np, "leds");
	if (!led_node)
		return -ENODEV;

	for_each_available_child_of_node(led_node, child) {
		u32 reg;

		if (of_property_read_u32(child, "reg", &reg))
			continue;

		if (reg >= MAX597X_NUM_LEDS) {
			dev_err(max597x->dev, "invalid LED (%u >= %d)\n", reg,
				MAX597X_NUM_LEDS);
			continue;
		}

		ret = max597x_led(max597x, child, reg);
		if (ret < 0) {
			of_node_put(child);
			return ret;
		}
	}

	return ret;
}

static struct platform_driver max597x_led_driver = {
	.driver = {
		.name = "max597x-led",
	},
	.probe = max597x_led_probe,
};

module_platform_driver(max597x_led_driver);


MODULE_AUTHOR("Patrick Rudolph <patrick.rudolph@9elements.com>");
MODULE_DESCRIPTION("MAX5970_hot-swap controller driver");
MODULE_LICENSE("GPL v2");
