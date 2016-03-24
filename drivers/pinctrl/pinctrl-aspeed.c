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

#include <linux/io.h>
#include <linux/platform_device.h>
#include "core.h"
#include "pinctrl-aspeed.h"

int aspeed_pinctrl_get_groups_count(struct pinctrl_dev *pctldev)
{
	struct aspeed_pinctrl_data *pdata = pinctrl_dev_get_drvdata(pctldev);

	return pdata->ngroups;
}

const char *aspeed_pinctrl_get_group_name(struct pinctrl_dev *pctldev,
		unsigned group)
{
	struct aspeed_pinctrl_data *pdata = pinctrl_dev_get_drvdata(pctldev);

	return pdata->groups[group].name;
}

int aspeed_pinctrl_get_group_pins(struct pinctrl_dev *pctldev,
		unsigned group, const unsigned **pins, unsigned *npins)
{
	struct aspeed_pinctrl_data *pdata = pinctrl_dev_get_drvdata(pctldev);

	*pins = &pdata->groups[group].pins[0];
	*npins = pdata->groups[group].npins;

	return 0;
}

void aspeed_pinctrl_pin_dbg_show(struct pinctrl_dev *pctldev,
		struct seq_file *s, unsigned offset)
{
	seq_printf(s, " %s", dev_name(pctldev->dev));
}

int aspeed_pinmux_get_fn_count(struct pinctrl_dev *pctldev)
{
	struct aspeed_pinctrl_data *pdata = pinctrl_dev_get_drvdata(pctldev);

	return pdata->nfunctions;
}

const char *aspeed_pinmux_get_fn_name(struct pinctrl_dev *pctldev,
						unsigned function)
{
	struct aspeed_pinctrl_data *pdata = pinctrl_dev_get_drvdata(pctldev);

	return pdata->functions[function].name;
}

int aspeed_pinmux_get_fn_groups(struct pinctrl_dev *pctldev,
		unsigned function, const char * const **groups,
		unsigned * const num_groups)
{
	struct aspeed_pinctrl_data *pdata = pinctrl_dev_get_drvdata(pctldev);

	*groups = pdata->functions[function].groups;
	*num_groups = pdata->functions[function].ngroups;

	return 0;
}

bool aspeed_func_desc_eval(const struct aspeed_func_desc *desc,
		void __iomem *base)
{
	u32 raw = ioread32(base + desc->reg);

	return ((raw & desc->mask) >> __ffs(desc->mask)) == desc->enable;
}

bool aspeed_func_expr_eval(const struct aspeed_func_expr *expr, void __iomem *base)
{
	bool ret = true;
	int i;

	for (i = 0; i < expr->ndescs; i++) {
		const struct aspeed_func_desc *desc = &expr->descs[i];

		ret = ret && aspeed_func_desc_eval(desc, base);
	}

	return ret;
}

bool aspeed_func_expr_enable(const struct aspeed_func_expr *expr, void __iomem *base)
{
	int i;
	int ret;

	ret = aspeed_func_expr_eval(expr, base);
	if (ret)
		return ret;

	for (i = 0; i < expr->ndescs; i++) {
		const struct aspeed_func_desc *desc = &expr->descs[i];
		u32 val;

		/* FIXME: this is ast2400 specific */
		if (desc->reg == STRAP)
			continue;

		val = ioread32(base + desc->reg);
		val &= ~desc->mask;
		val |= desc->enable << __ffs(desc->mask);
		iowrite32(val, base + desc->reg);
	}

	ret = aspeed_func_expr_eval(expr, base);
	return ret;
}

bool aspeed_func_expr_disable(const struct aspeed_func_expr *expr, void __iomem *base)
{
	int i;
	bool ret;

	/* Ensure that we change any unsupported state to disabled */

	for (i = 0; i < expr->ndescs; i++) {
		const struct aspeed_func_desc *desc = &expr->descs[i];
		u32 val;

		if (desc->reg == STRAP)
			continue;

		val = ioread32(base + desc->reg);
		val &= ~desc->mask;
		val |= (desc->disable << __ffs(desc->mask));
		iowrite32(val, base + desc->reg);
	}

	ret = !aspeed_func_expr_eval(expr, base);

	return ret;
}

bool aspeed_maybe_disable(const struct aspeed_func_expr **expr, void __iomem *base)
{
	bool ret;

	if (!expr)
		return true;

	while (*expr) {
		ret |= aspeed_func_expr_disable(*expr, base);
		expr++;
	}
	return ret;
}

bool aspeed_sig_in_exprs(const struct aspeed_func_expr **exprs,
		const struct aspeed_func_expr *sig)
{
	if (!exprs)
		return false;

	while (*exprs) {
		if (sig == *exprs)
			return true;
		exprs++;
	}
	return false;
}

int aspeed_pinmux_set_mux(struct pinctrl_dev *pctldev, unsigned function,
	       	unsigned group)
{
	int i;
	const struct aspeed_pinctrl_data *pdata =
		pinctrl_dev_get_drvdata(pctldev);
	const struct aspeed_pin_group *pgroup = &pdata->groups[group];
	const struct aspeed_pin_function *pfunc =
		&pdata->functions[function];


	for (i = 0; i < pgroup->npins; i++) {
		int pin = pgroup->pins[i];
		const struct aspeed_pin_prio *pprio = pdata->pins[pin].drv_data;
		const struct aspeed_func_expr *func = pfunc->expr;

		if (!pprio)
			return -1;

		/* If low priority is requested, ensure high is disabled */
		if (aspeed_sig_in_exprs(pprio->low, func)) {
			if (!aspeed_maybe_disable(pprio->high, pdata->reg_base)) {
				pr_err("Failed to disable high priority function on pin %d\n",
						pin);
				return -1;
			}
		}

		/* Nothing to do if we're already enabled */
		if (aspeed_func_expr_eval(func, pdata->reg_base))
			continue;

		/* Configure the pin */
		if (!aspeed_func_expr_enable(func, pdata->reg_base))
			return -1;
	}

	return 0;
}

int aspeed_gpio_request_enable(struct pinctrl_dev *pctldev,
		struct pinctrl_gpio_range *range,
		unsigned offset)
{
	unsigned pin = range->base + offset;
	const struct aspeed_pinctrl_data *pdata =
		pinctrl_dev_get_drvdata(pctldev);
	const struct aspeed_pin_prio *pprio =
		pdata->pins[pin].drv_data;

	if (!pprio)
		return 0;

	if (!aspeed_maybe_disable(pprio->high, pdata->reg_base))
		return -1;

	if (!aspeed_maybe_disable(pprio->low, pdata->reg_base))
		return -1;

	return 0;
}

int aspeed_pin_config_get(struct pinctrl_dev *pctldev,
		unsigned pin,
		unsigned long *config)
{
	return 0;
}

int aspeed_pin_config_set(struct pinctrl_dev *pctldev,
		unsigned pin,
		unsigned long *configs,
		unsigned num_configs)
{
	return 0;
}
