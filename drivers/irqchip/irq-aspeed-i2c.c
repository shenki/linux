/*
 * irq-aspeed-i2c.c - I2C IRQCHIP driver for the Aspeed SoC
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
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/reset.h>
#include <linux/irq.h>
#include <linux/irqchip.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/irqdomain.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of.h>
#include <linux/io.h>
/*******************************************************************/
#define AST_I2CG_ISR		0x00
#define AST_I2CG_OWNER		0x08
#define AST_I2CG_CTRL		0x0C
#define I2C_SRAM_BUFF_EN	0x1
/*******************************************************************/
//#define AST_I2C_IRQ_DEBUG

#ifdef AST_I2C_IRQ_DEBUG
#define I2C_IRQ_DBUG(fmt, args...) printk(KERN_DEBUG "%s() " fmt, __FUNCTION__, ## args)
#else
#define I2C_IRQ_DBUG(fmt, args...)
#endif

/*******************************************************************/
struct ast_i2c_irq {
	void __iomem	*regs;
	int			parent_irq;
	int			bus_num;
	struct irq_domain	*irq_domain;
	struct reset_control *reset;	
};
/*******************************************************************/
static void ast_i2c_irq_handler(struct irq_desc *desc)
{
	struct ast_i2c_irq *i2c_irq = irq_desc_get_handler_data(desc);
	struct irq_chip *chip = irq_desc_get_chip(desc);
	unsigned long bit, status;
	unsigned int bus_irq;

	chained_irq_enter(chip, desc);
	status = readl(i2c_irq->regs + AST_I2CG_ISR);
	for_each_set_bit(bit, &status, i2c_irq->bus_num) {
		bus_irq = irq_find_mapping(i2c_irq->irq_domain, bit);
		generic_handle_irq(bus_irq);
	}
	chained_irq_exit(chip, desc);
}

static void noop(struct irq_data *data) { }

static unsigned int noop_ret(struct irq_data *data)
{
	return 0;
}

struct irq_chip i2c_irq_chip = {
	.name			= "i2c-irq",
	.irq_startup	= noop_ret,
	.irq_shutdown	= noop,
	.irq_enable		= noop,
	.irq_disable	= noop,
	.irq_ack		= noop,
	.irq_mask		= noop,
	.irq_unmask		= noop,
	.flags			= IRQCHIP_SKIP_SET_WAKE,
};

static int ast_i2c_map_irq_domain(struct irq_domain *domain,
					unsigned int irq, irq_hw_number_t hwirq)
{
	irq_set_chip_and_handler(irq, &i2c_irq_chip, handle_simple_irq);
	irq_set_chip_data(irq, domain->host_data);

	return 0;
}

static const struct irq_domain_ops ast_i2c_irq_domain_ops = {
	.map = ast_i2c_map_irq_domain,
};

static int irq_aspeed_i2c_probe(struct platform_device *pdev)
{
	struct ast_i2c_irq *i2c_irq;
	struct device_node *node = pdev->dev.of_node;
	u32 cmd_source;

	i2c_irq = kzalloc(sizeof(*i2c_irq), GFP_KERNEL);
	if (!i2c_irq)
		return -ENOMEM;	

	i2c_irq->regs = of_iomap(node, 0);
	if (IS_ERR(i2c_irq->regs))
		return PTR_ERR(i2c_irq->regs);

	node->data = i2c_irq;
	if (of_property_read_u32(node, "bus_num", &i2c_irq->bus_num) == 0) {
		I2C_IRQ_DBUG("i2c_irq->bus_num = %d \n", i2c_irq->bus_num);
	}

	i2c_irq->reset = devm_reset_control_get_exclusive(&pdev->dev, "i2c");
	if (IS_ERR(i2c_irq->reset)) {
		dev_err(&pdev->dev, "can't get i2c reset\n");
		return PTR_ERR(i2c_irq->reset);
	}

	//SCU I2C Reset 
	reset_control_assert(i2c_irq->reset);
	udelay(3);
	reset_control_deassert(i2c_irq->reset);

	/* only support in ast-g5 platform */
	writel(I2C_SRAM_BUFF_EN, i2c_irq->regs + AST_I2CG_CTRL);

	if (!of_property_read_u32(node, "cmd-source", &cmd_source)) {
		writel(cmd_source, i2c_irq->regs + AST_I2CG_OWNER);
	}

	i2c_irq->parent_irq = irq_of_parse_and_map(node, 0);
	if (i2c_irq->parent_irq < 0)
		return i2c_irq->parent_irq;

	i2c_irq->irq_domain = irq_domain_add_linear(
			node, i2c_irq->bus_num,
			&ast_i2c_irq_domain_ops, NULL);
	if (!i2c_irq->irq_domain)
		return -ENOMEM;

	i2c_irq->irq_domain->name = "ast-i2c-domain";

	irq_set_chained_handler_and_data(i2c_irq->parent_irq,
					 ast_i2c_irq_handler, i2c_irq);

	pr_info("i2c-irq controller registered, irq %d\n", i2c_irq->parent_irq);

	return 0;
}

static const struct of_device_id irq_aspeed_i2c_dt_ids[] = {
	{ .compatible = "aspeed,ast-i2c-irq", },
	{},
};
MODULE_DEVICE_TABLE(of, irq_aspeed_i2c_dt_ids);

static struct platform_driver irq_aspeed_i2c_device_driver = {
	.probe		= irq_aspeed_i2c_probe,
	.driver 	= {
		.name   = KBUILD_MODNAME,
		.of_match_table = irq_aspeed_i2c_dt_ids,
	}
};

static int __init irq_aspeed_i2c_init(void)
{
	return platform_driver_register(&irq_aspeed_i2c_device_driver);
}
core_initcall(irq_aspeed_i2c_init);

MODULE_AUTHOR("Ryan Chen");
MODULE_DESCRIPTION("AST SOC I2C IRQ Driver");
MODULE_LICENSE("GPL v2");
