/*
 * irq-aspeed-scu.c - SCU IRQCHIP driver for the Aspeed SoC
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
#include <linux/irq.h>
#include <linux/irqchip.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/irqdomain.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/io.h>
/*******************************************************************/
/*  AST_SCU_INTR_CTRL : 0x18 - Interrupt control and status register   */
#define INTR_LPC_H_L_RESET				(0x1 << 21)
#define INTR_LPC_L_H_RESET				(0x1 << 20)
#define INTR_PCIE_H_L_RESET				(0x1 << 19)
#define INTR_PCIE_L_H_RESET				(0x1 << 18)
#define INTR_VGA_SCRATCH_CHANGE			(0x1 << 17)
#define INTR_VGA_CURSOR_CHANGE			(0x1 << 16)
#define INTR_ISSUE_MSI					(0x1 << 6)
#define INTR_LPC_H_L_RESET_EN			(0x1 << 5)
#define INTR_LPC_L_H_RESET_EN			(0x1 << 4)
#define INTR_PCIE_H_L_RESET_EN			(0x1 << 3)
#define INTR_PCIE_L_H_RESET_EN			(0x1 << 2)
#define INTR_VGA_SCRATCH_CHANGE_EN		(0x1 << 1)
#define INTR_VGA_CURSOR_CHANGE_EN		(0x1 << 0)
/*******************************************************************/
//#define AST_SCU_IRQ_DEBUG

#ifdef AST_SCU_IRQ_DEBUG
#define SCU_IRQ_DBUG(fmt, args...) printk(KERN_DEBUG "%s() " fmt, __FUNCTION__, ## args)
#else
#define SCU_IRQ_DBUG(fmt, args...)
#endif
/*******************************************************************/
struct ast_scu_irq {
	void __iomem	*regs;
	int			parent_irq;
	int			irq_num;
	struct irq_domain	*irq_domain;
};
/*******************************************************************/
static void ast_scu_irq_handler(struct irq_desc *desc)
{
	struct ast_scu_irq *scu_irq = irq_desc_get_handler_data(desc);
	struct irq_chip *chip = irq_desc_get_chip(desc);
	unsigned long bit, status;
	unsigned int bus_irq;

	chained_irq_enter(chip, desc);
	status = readl(scu_irq->regs);
	for_each_set_bit(bit, &status, scu_irq->irq_num) {
		bus_irq = irq_find_mapping(scu_irq->irq_domain, bit);
		generic_handle_irq(bus_irq);
	}
	writel(status, scu_irq->regs);
	chained_irq_exit(chip, desc);
}

static void noop(struct irq_data *data) { }

static unsigned int noop_ret(struct irq_data *data)
{
	return 0;
}

struct irq_chip scu_irq_chip = {
	.name		= "scu-irq",
	.irq_startup	= noop_ret,
	.irq_shutdown	= noop,
	.irq_enable	= noop,
	.irq_disable	= noop,
	.irq_ack	= noop,
	.irq_mask	= noop,
	.irq_unmask	= noop,
	.flags		= IRQCHIP_SKIP_SET_WAKE,
};

static int ast_scu_map_irq_domain(struct irq_domain *domain,
					unsigned int irq, irq_hw_number_t hwirq)
{
	irq_set_chip_and_handler(irq, &scu_irq_chip, handle_simple_irq);
	irq_set_chip_data(irq, domain->host_data);

	return 0;
}

static const struct irq_domain_ops ast_scu_irq_domain_ops = {
	.map = ast_scu_map_irq_domain,
};

static int __init ast_scu_irq_of_init(struct device_node *node,
					struct device_node *parent)
{
	struct ast_scu_irq *scu_irq;
	u8 *buf_pool;

	scu_irq = kzalloc(sizeof(*scu_irq), GFP_KERNEL);
	if (!scu_irq)
		return -ENOMEM;	

	scu_irq->regs = of_iomap(node, 0);
	if (IS_ERR(scu_irq->regs))
		return PTR_ERR(scu_irq->regs);

	node->data = scu_irq;
	scu_irq->irq_num = 22;

	scu_irq->parent_irq = irq_of_parse_and_map(node, 0);
	if (scu_irq->parent_irq < 0)
		return scu_irq->parent_irq;

	scu_irq->irq_domain = irq_domain_add_linear(
			node, scu_irq->irq_num,
			&ast_scu_irq_domain_ops, NULL);
	if (!scu_irq->irq_domain)
		return -ENOMEM;

	scu_irq->irq_domain->name = "ast-scu-domain";

	irq_set_chained_handler_and_data(scu_irq->parent_irq,
					 ast_scu_irq_handler, scu_irq);

	pr_info("scu-irq controller registered, irq %d\n", scu_irq->parent_irq);

	return 0;
}

IRQCHIP_DECLARE(ast_scu_irq, "aspeed,aspeed-scu-ic", ast_scu_irq_of_init);
