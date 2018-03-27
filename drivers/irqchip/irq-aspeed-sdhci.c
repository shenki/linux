/*
 * irq-aspeed-sdhci.c - SDHCI IRQCHIP driver for the Aspeed SoC
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
#include <mach/ast-sdhci.h>
/*******************************************************************/
#define AST_SDHCI_INFO				0x00
#define AST_SDHCI_BLOCK				0x04
#define AST_SDHCI_CTRL				0xF0
#define AST_SDHCI_ISR				0xFC

/* #define AST_SDHCI_INFO			0x00*/
#define AST_SDHCI_S1MMC8			(1 << 25)
#define AST_SDHCI_S0MMC8			(1 << 24)
/*******************************************************************/
//#define AST_SDHCI_IRQ_DEBUG

#ifdef AST_SDHCI_IRQ_DEBUG
#define SDHCI_IRQ_DBUG(fmt, args...) printk(KERN_DEBUG "%s() " fmt, __FUNCTION__, ## args)
#else
#define SDHCI_IRQ_DBUG(fmt, args...)
#endif
/*******************************************************************/
void ast_sd_set_8bit_mode(struct ast_sdhci_irq *sdhci_irq, u8 mode)
{
	if (mode)
		writel((1 << 24) | readl(sdhci_irq->regs), sdhci_irq->regs);
	else
		writel(~(1 << 24) & readl(sdhci_irq->regs), sdhci_irq->regs);
}

EXPORT_SYMBOL(ast_sd_set_8bit_mode);

static void ast_sdhci_irq_handler(struct irq_desc *desc)
{
	struct ast_sdhci_irq *sdhci_irq = irq_desc_get_handler_data(desc);
	struct irq_chip *chip = irq_desc_get_chip(desc);
	unsigned long bit, status;
	unsigned int slot_irq;

	chained_irq_enter(chip, desc);
	status = readl(sdhci_irq->regs + AST_SDHCI_ISR) & 0x3;
//	printk("sdhci irq status %x \n", status);
	for_each_set_bit(bit, &status, sdhci_irq->slot_num) {
		slot_irq = irq_find_mapping(sdhci_irq->irq_domain, bit);
//		printk("slot_irq %x \n", slot_irq);
		generic_handle_irq(slot_irq);
	}
	chained_irq_exit(chip, desc);
}

static void noop(struct irq_data *data) { }

static unsigned int noop_ret(struct irq_data *data)
{
	return 0;
}

struct irq_chip sdhci_irq_chip = {
	.name		= "sdhci-irq",
	.irq_startup	= noop_ret,
	.irq_shutdown	= noop,
	.irq_enable	= noop,
	.irq_disable	= noop,
	.irq_ack	= noop,
	.irq_mask	= noop,
	.irq_unmask	= noop,
//	.irq_set_type	= noop_ret,
	.flags		= IRQCHIP_SKIP_SET_WAKE,
};

static int ast_sdhci_map_irq_domain(struct irq_domain *domain,
									unsigned int irq, irq_hw_number_t hwirq)
{
	irq_set_chip_and_handler(irq, &sdhci_irq_chip, handle_simple_irq);
	irq_set_chip_data(irq, domain->host_data);
	irq_set_irq_type(irq, IRQ_TYPE_LEVEL_HIGH);

	return 0;
}

static const struct irq_domain_ops ast_sdhci_irq_domain_ops = {
	.map = ast_sdhci_map_irq_domain,
};

static int irq_aspeed_sdhci_probe(struct platform_device *pdev)
{
	struct ast_sdhci_irq *sdhci_irq;
	u32 slot0_clk_delay, slot1_clk_delay;

	SDHCI_IRQ_DBUG("ast_sdhci_irq_init \n");

	sdhci_irq = kzalloc(sizeof(*sdhci_irq), GFP_KERNEL);
	if (!sdhci_irq)
		return -ENOMEM;

	platform_set_drvdata(pdev, sdhci_irq);
	//node->data = sdhci_irq;
	pdev->dev.of_node->data = sdhci_irq;

	if (!of_property_read_u32(pdev->dev.of_node, "slot_num", &sdhci_irq->slot_num)) {
		SDHCI_IRQ_DBUG("sdhci_irq->slot_num = %d \n", sdhci_irq->slot_num);
	}

	sdhci_irq->regs = of_iomap(pdev->dev.of_node, 0);
	if (IS_ERR(sdhci_irq->regs))
		return PTR_ERR(sdhci_irq->regs);

	sdhci_irq->reset = devm_reset_control_get_exclusive(&pdev->dev, "sdhci");
	if (IS_ERR(sdhci_irq->reset)) {
		dev_err(&pdev->dev, "can't get sdhci reset\n");
		return PTR_ERR(sdhci_irq->reset);
	}
	sdhci_irq->clk = devm_clk_get(&pdev->dev, NULL);

	if (IS_ERR(sdhci_irq->clk)) {
		dev_err(&pdev->dev, "no clock defined\n");
		return -ENODEV;
	}
	//SDHCI Host's Clock Enable and Reset
	reset_control_assert(sdhci_irq->reset);
	mdelay(10);
	clk_prepare_enable(sdhci_irq->clk);
	clk_enable(sdhci_irq->clk);
	mdelay(10);
	reset_control_deassert(sdhci_irq->reset);

	sdhci_irq->parent_irq = irq_of_parse_and_map(pdev->dev.of_node, 0);
	if (sdhci_irq->parent_irq < 0)
		return sdhci_irq->parent_irq;

	sdhci_irq->irq_domain = irq_domain_add_linear(
								pdev->dev.of_node, sdhci_irq->slot_num,
								&ast_sdhci_irq_domain_ops, NULL);
	if (!sdhci_irq->irq_domain)
		return -ENOMEM;

	sdhci_irq->irq_domain->name = "ast-sdhci-irq";

	irq_set_chained_handler_and_data(sdhci_irq->parent_irq,
									 ast_sdhci_irq_handler, sdhci_irq);


	//1e7600f0[17:16] = 0x3 //slot0 clock delay mode
	//1e7600f0[24:20] = 0x8 //slot0 delay
	if (!of_property_read_u32(pdev->dev.of_node, "slot0-clk-delay", &slot0_clk_delay)) {
		writel((readl(sdhci_irq->regs + AST_SDHCI_CTRL) & ~0x01f30000) | (0x3 << 16) | (slot0_clk_delay << 20), sdhci_irq->regs + AST_SDHCI_CTRL);
	}

	//1e7600f0[19:18] = 0x3 //slot1 clock delay mode
	//1e7600f0[29:25] = 0x8 //slot1 delay
	if (!of_property_read_u32(pdev->dev.of_node, "slot1-clk-delay", &slot1_clk_delay)) {
		writel((readl(sdhci_irq->regs + AST_SDHCI_CTRL) & ~0x3e0c0000) | (0x3 << 18) | (slot1_clk_delay << 25), sdhci_irq->regs + AST_SDHCI_CTRL);
	}

	pr_info("sdhci irq controller registered, irq %d\n", sdhci_irq->parent_irq);

	return 0;
}

static const struct of_device_id irq_aspeed_sdhci_dt_ids[] = {
	{ .compatible = "aspeed,ast-sdhci-irq", },
	{},
};
MODULE_DEVICE_TABLE(of, irq_aspeed_sdhci_dt_ids);

static struct platform_driver irq_aspeed_sdhci_device_driver = {
	.probe		= irq_aspeed_sdhci_probe,
	.driver		= {
		.name   = KBUILD_MODNAME,
		.of_match_table	= irq_aspeed_sdhci_dt_ids,
	}
};

static int __init irq_aspeed_sdhci_init(void)
{
	return platform_driver_register(&irq_aspeed_sdhci_device_driver);
}
core_initcall(irq_aspeed_sdhci_init);

MODULE_AUTHOR("Ryan Chen");
MODULE_DESCRIPTION("AST SOC SDHCI IRQ Driver");
MODULE_LICENSE("GPL v2");
