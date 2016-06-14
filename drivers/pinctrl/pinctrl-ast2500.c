/*
 * Copyright (C) 2016 IBM Corp.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/bitops.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/string.h>
#include <linux/types.h>

#include "core.h"
#include "pinctrl-utils.h"
#include "pinctrl-aspeed.h"

#define AST2500_NR_PINS 228

#define SCU80 0x80
#define SCU90 0x90
#define SCU94 0x94

#define COND1 FUNC_DESC_BIT(SCU90, 6, 0)
#define COND2 { SCU94, GENMASK(1, 0), 0, 0 }

FUNC_EXPR_DECL_SINGLE(I2C9, FUNC_DESC_SET(SCU90, 22), COND1);

#define C14 4
FUNC_EXPR_DECL_SINGLE(TIMER5, FUNC_DESC_SET(SCU80, 4), COND1);
MS_PIN_DECL(C14, GPIOA4, I2C9, TIMER5);

FUNC_GROUP_DECL(TIMER5, C14);

#define A13 5
FUNC_EXPR_DECL_SINGLE(TIMER6, FUNC_DESC_SET(SCU80, 5), COND1);
MS_PIN_DECL(A13, GPIOA4, I2C9, TIMER6);

FUNC_GROUP_DECL(TIMER6, A13);

FUNC_GROUP_DECL(I2C9, C14, A13);

FUNC_EXPR_DECL_SINGLE(I2C4, FUNC_DESC_SET(SCU90, 17));

#define A9 130
SS_PIN_DECL(A9, GPIOQ2, I2C4);

#define B9 131
SS_PIN_DECL(B9, GPIOQ3, I2C4);

FUNC_GROUP_DECL(I2C4, A9, B9);

/* I2C3 */

FUNC_EXPR_DECL_SINGLE(I2C3, FUNC_DESC_SET(SCU90, 16), COND1);

#define A11 128
SS_PIN_DECL(A11, GPIOQ0, I2C3);

#define A10 129
SS_PIN_DECL(A10, GPIOQ1, I2C3);

FUNC_GROUP_DECL(I2C3, A11, A10);

/* I2C12 */
#define D10 10
#define E12 11
FUNC_EXPR_DECL_SINGLE(I2C12, FUNC_DESC_SET(SCU90, 25));
FUNC_EXPR_DECL_SINGLE(SD1, FUNC_DESC_SET(SCU90, 0));
MS_PIN_DECL(D10, GPIOC4, SD1, I2C12);
MS_PIN_DECL(E12, GPIOC4, SD1, I2C12);

FUNC_GROUP_DECL(I2C12, D10, E12);
FUNC_GROUP_DECL(SD1, D10, E12, /* nees more */);


static struct pinctrl_pin_desc ast2500_pins[AST2500_NR_PINS] = {
	ASPEED_PINCTRL_PIN(C14),
	ASPEED_PINCTRL_PIN(A13),
	ASPEED_PINCTRL_PIN(A11),
	ASPEED_PINCTRL_PIN(A10),
	ASPEED_PINCTRL_PIN(A9),
	ASPEED_PINCTRL_PIN(B9),
	ASPEED_PINCTRL_PIN(D10),
	ASPEED_PINCTRL_PIN(E12),
};

static const struct aspeed_pin_group ast2500_groups[] = {
	ASPEED_PINCTRL_GROUP(I2C12),
	ASPEED_PINCTRL_GROUP(I2C9),
	ASPEED_PINCTRL_GROUP(I2C4),
	ASPEED_PINCTRL_GROUP(I2C3),
};

static const struct aspeed_pin_function ast2500_functions[] = {
	ASPEED_PINCTRL_FUNC(I2C12),
	ASPEED_PINCTRL_FUNC(I2C9),
	ASPEED_PINCTRL_FUNC(I2C4),
	ASPEED_PINCTRL_FUNC(I2C3),
};

static struct aspeed_pinctrl_data ast2500_pinctrl = {
	.pins = ast2500_pins,
	.npins = ARRAY_SIZE(ast2500_pins),
	.groups = ast2500_groups,
	.ngroups = ARRAY_SIZE(ast2500_groups),
	.functions = ast2500_functions,
	.nfunctions = ARRAY_SIZE(ast2500_functions),
};

static struct pinmux_ops ast2500_pinmux_ops = {
	.get_functions_count = aspeed_pinmux_get_fn_count,
	.get_function_name = aspeed_pinmux_get_fn_name,
	.get_function_groups = aspeed_pinmux_get_fn_groups,
	.set_mux = aspeed_pinmux_set_mux,
	.gpio_request_enable = aspeed_gpio_request_enable,
	.strict = true,
};

static struct pinctrl_ops ast2500_pinctrl_ops = {
	.get_groups_count = aspeed_pinctrl_get_groups_count,
	.get_group_name = aspeed_pinctrl_get_group_name,
	.get_group_pins = aspeed_pinctrl_get_group_pins,
	.pin_dbg_show = aspeed_pinctrl_pin_dbg_show,
	.dt_node_to_map = pinconf_generic_dt_node_to_map_pin,
	.dt_free_map = pinctrl_utils_free_map,
};

static struct pinconf_ops ast2500_pinconf_ops = {
	.pin_config_get = aspeed_pin_config_get,
	.pin_config_set = aspeed_pin_config_set,
};

static struct pinctrl_desc ast2500_pinctrl_desc = {
	.name = "pinctrl-ast2500",
	.pins = ast2500_pins,
	.npins = ARRAY_SIZE(ast2500_pins),
	.pctlops = &ast2500_pinctrl_ops,
	.pmxops = &ast2500_pinmux_ops,
	.confops = &ast2500_pinconf_ops,
	.owner = THIS_MODULE,
};

static struct pinctrl_gpio_range ast2500_gpios = {
	.name = "ast2500-pctrl-gpio-range",
	/* HACK: this is even auto-assigned by the GPIO subsystem, so there are
	 * no guarantees. BUT. It seems to pick 320 consistently.
	 */
	.base = 320,
	.npins = ARRAY_SIZE(ast2500_pins),
};

static int __init ast2500_pinctrl_probe(struct platform_device *pdev)
{
	struct aspeed_pinctrl_data *pdata = &ast2500_pinctrl;
	struct resource *res;
	struct pinctrl_dev *pctl;
	int i;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	pdata->reg_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(pdata->reg_base))
		return PTR_ERR(pdata->reg_base);

	/* We allocated space for all pins, make sure we initialise all pin
	 * numbers. Those pins we haven't defined won't yet have had their
	 * number initialised, and it's effectively a no-op for those which
	 * have.
	 */
	for (i = 0; i < ARRAY_SIZE(ast2500_pins); i++)
		ast2500_pins[i].number = i;

	pctl = pinctrl_register(&ast2500_pinctrl_desc, &pdev->dev, pdata);

	if (IS_ERR(pctl)) {
		dev_err(&pdev->dev, "Failed to register pinctrl\n");
		return PTR_ERR(pctl);
	}

	platform_set_drvdata(pdev, pdata);

	/* grange.name = "exynos5440-pctrl-gpio-range"; */
	pinctrl_add_gpio_range(pctl, &ast2500_gpios);

	return 0;
}

static const struct of_device_id ast2500_pinctrl_of_match[] = {
	{ .compatible = "aspeed,ast2500-pinctrl", },
	{ },
};

static struct platform_driver ast2500_pinctrl_driver = {
	.driver = {
		.name = "pinctrl-ast2500",
		.of_match_table = ast2500_pinctrl_of_match,
	},
};

module_platform_driver_probe(ast2500_pinctrl_driver, ast2500_pinctrl_probe);

MODULE_AUTHOR("Andrew Jeffery <andrew@aj.id.au>");
MODULE_DESCRIPTION("ASPEED AST2500 pinctrl driver");
MODULE_LICENSE("GPL v2");
