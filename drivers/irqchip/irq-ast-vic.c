/*
 * irq-ast-vic.c - VIC driver for the Aspeed SoC
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
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/irqchip.h>
#include <linux/irqdomain.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/device.h>

#include <asm/exception.h>
#include <asm/mach/irq.h>

struct aspeed_vic {
	void __iomem		*base;
	struct irq_domain *domain;
};

static struct aspeed_vic *ast_vic;
/***************************************************************************/
#define AST_VIC_NUM_IRQS	64
/***************************************************************************/
#define AST_IRQ_STS(x)				(ast_vic->base + 0x00 + (x*0x04))
#define AST_FIQ_STS(x)				(ast_vic->base + 0x04 + (x*0x04))
#define AST_RAW_STS(x)				(ast_vic->base + 0x10 + (x*0x04))
#define AST_INTR_SEL(x)				(ast_vic->base + 0x18 + (x*0x04))
#define AST_INTR_EN(x)				(ast_vic->base + 0x20 + (x*0x04))
#define AST_INTR_DIS(x)				(ast_vic->base + 0x28 + (x*0x04))
#define AST_INTR_SW_EN(x)			(ast_vic->base + 0x30 + (x*0x04))
#define AST_INTR_SW_CLR(x)			(ast_vic->base + 0x38 + (x*0x04))
#define AST_INTR_SENSE(x)			(ast_vic->base + 0x40 + (x*0x04))
#define AST_INTR_BOTH_EDGE(x)		(ast_vic->base + 0x48 + (x*0x04))
#define AST_INTR_EVENT(x)			(ast_vic->base + 0x50 + (x*0x04))
#define AST_INTR_EDGE_CLR(x)		(ast_vic->base + 0x58 + (x*0x04))
#define AST_INTR_EDGE_STS(x)		(ast_vic->base + 0x60 + (x*0x04))

#define IRQ_SET_LEVEL_TRIGGER(x, irq_no)   *((volatile unsigned long*)AST_INTR_SENSE(x)) |= 1 << (irq_no)
#define IRQ_SET_EDGE_TRIGGER(x, irq_no)    *((volatile unsigned long*)AST_INTR_SENSE(x)) &= ~(1 << (irq_no))
#define IRQ_SET_RISING_EDGE(x, irq_no)     *((volatile unsigned long*)AST_INTR_EVENT(x)) |= 1 << (irq_no)
#define IRQ_SET_BOTH_TRIGGER(x, irq_no)    *((volatile unsigned long*)AST_INTR_BOTH_EDGE(x)) &= ~(1 << (irq_no))
#define IRQ_SET_FALLING_EDGE(x, irq_no)    *((volatile unsigned long*)AST_INTR_EVENT(x)) &= ~(1 << (irq_no))
#define IRQ_SET_HIGH_LEVEL(x,irq_no)      *((volatile unsigned long*)AST_INTR_EVENT(x)) |= 1 << (irq_no)
#define IRQ_SET_LOW_LEVEL(x, irq_no)       *((volatile unsigned long*)AST_INTR_EVENT(x)) &= ~(1 << (irq_no))
#define IRQ_EDGE_CLEAR(x, irq_no)          *((volatile unsigned long*)AST_INTR_EDGE_CLR(x)) |= 1 << (irq_no)
#define IRQ_SW_CLEAR(x, irq_no)          *((volatile unsigned long*)AST_INTR_SW_CLR(x)) |= 1 << (irq_no)
/***************************************************************************/
asmlinkage void __exception_irq_entry ast_vic_handle_irq(struct pt_regs *regs)
{
	u32 stat, hwirq;

	for (;;) {
		hwirq = 0;
		stat = readl(AST_IRQ_STS(0));
		if (!stat) {
			stat = readl(AST_IRQ_STS(1));
			hwirq = 32;
		}
		if (stat == 0)
			break;
		hwirq += ffs(stat) - 1;
//		printk("handle irq %d \n", hwirq);
		handle_domain_irq(ast_vic->domain, hwirq, regs);
	}
}

static void ast_vic_ack_irq(struct irq_data *d)
{
        int idx = 0;
        int idx_irq = d->hwirq;

        if (d->hwirq & 0x20) {
                idx = 1;
                idx_irq &= 0x1f;
        }

        /*  check edge interrupt type */
        if(!((0x1 << idx_irq) & readl(AST_INTR_SENSE(idx))))
                IRQ_EDGE_CLEAR(idx, idx_irq);

        /*  check sw interrupt type */
        if((0x1 << idx_irq) & readl(AST_INTR_SW_EN(idx)))
                IRQ_SW_CLEAR(idx, idx_irq);
}

static void ast_vic_mask_irq(struct irq_data *d)
{
	int idx = 0;
	int idx_irq = d->hwirq;
	
	if (d->hwirq & 0x20) {
		idx = 1;
		idx_irq &= 0x1f;
	} 
	
	writel(readl(AST_INTR_DIS(idx)) | (1 << idx_irq), AST_INTR_DIS(idx));
}

static void ast_vic_unmask_irq(struct irq_data *d)
{
        int idx = 0;
        int idx_irq = d->hwirq;

        if (d->hwirq & 0x20) {
	        idx = 1;
	        idx_irq &= 0x1f;
        }

        writel((1 << idx_irq), AST_INTR_EN(idx));
}

static void ast_vic_mask_ack_irq(struct irq_data *d)
{
	int idx = 0;
	u32 idx_irq = d->hwirq;

	if (d->hwirq & 0x20) {
		idx=1;
		idx_irq &= 0x1f;
	} 
	
	writel((1 << idx_irq), AST_INTR_DIS(idx));

	/*	check edge interrupt type */
	if(!((0x1 << idx_irq) & readl(AST_INTR_SENSE(idx))))
		IRQ_EDGE_CLEAR(idx, idx_irq);
	
	/*	check sw interrupt type */
	if((0x1 << idx_irq) & readl(AST_INTR_SW_EN(idx)))
		IRQ_SW_CLEAR(idx, idx_irq);	
	
}

static int ast_irq_type(struct irq_data *d, unsigned int type)
{
	int idx = 0;
	u32 idx_irq = d->hwirq;

	if (d->hwirq & 0x20) {
		idx = 1;
		idx_irq &= 0x1f;
	} 
printk("ast_irq_type irq %d \n", d->hwirq);
	switch (type) {
		case IRQ_TYPE_NONE:
			break;
		case IRQ_TYPE_EDGE_RISING:
			IRQ_SET_EDGE_TRIGGER(idx, idx_irq);
			IRQ_SET_RISING_EDGE(idx, idx_irq);
			irq_set_handler(d->hwirq, handle_edge_irq);
			break;
		case IRQ_TYPE_EDGE_FALLING:
			IRQ_SET_EDGE_TRIGGER(idx, idx_irq);
			IRQ_SET_FALLING_EDGE(idx, idx_irq);
			irq_set_handler(d->hwirq, handle_edge_irq);
			break;		
		case IRQ_TYPE_EDGE_BOTH:
			IRQ_SET_EDGE_TRIGGER(idx, idx_irq);
			IRQ_SET_BOTH_TRIGGER(idx, idx_irq);
			irq_set_handler(d->hwirq, handle_edge_irq);
			break;
		case IRQ_TYPE_LEVEL_HIGH:
			irq_set_handler(d->hwirq, handle_level_irq);
			break;
		default:
			pr_err("No such irq type %d", type);
			return -EINVAL;
	}

	return 0;
}

static struct irq_chip ast_vic_chip = {
	.name		= "ast-vic",
	.irq_ack	= ast_vic_ack_irq,
	.irq_mask	= ast_vic_mask_irq,
	.irq_unmask = ast_vic_unmask_irq,
	.irq_mask_ack	= ast_vic_mask_ack_irq,		
	.irq_set_type	= ast_irq_type,	
};

static int ast_vic_map(struct irq_domain *d, unsigned int irq,
			irq_hw_number_t hwirq)
{
	int idx = 0;
	u32 idx_irq = hwirq;

	if (hwirq & 0x20) {
		idx = 1;
		idx_irq &= 0x1f;
	} 
#if 1
	if((0x1 << idx_irq) & readl(AST_INTR_SENSE(idx))) {
		irq_set_chip_and_handler(irq, &ast_vic_chip, handle_level_irq);
//		irq_set_irq_type(irq, IRQ_TYPE_LEVEL_HIGH);
	} else {
		irq_set_chip_and_handler(irq, &ast_vic_chip, handle_edge_irq);
//		irq_set_irq_type(irq, IRQ_TYPE_EDGE_BOTH);
	}
#else
	if(hwirq != 22) {
		irq_set_chip_and_handler(irq, &ast_vic_chip, handle_level_irq);
//		irq_set_irq_type(irq, IRQ_TYPE_LEVEL_HIGH);
	} else {
		irq_set_chip_and_handler(irq, &ast_vic_chip, handle_edge_irq);
//		irq_set_irq_type(irq, IRQ_TYPE_EDGE_BOTH);
	}

#endif
	irq_set_chip_data(irq, ast_vic);
	irq_set_probe(irq);

	return 0;

}

static struct irq_domain_ops ast_vic_dom_ops = {
	.map = ast_vic_map,
	.xlate = irq_domain_xlate_onetwocell,
};

static int __init ast_init_vic_of(struct device_node *node,
			       struct device_node *parent)
{
	int i = 0;
	void __iomem *regs;

	regs = of_iomap(node, 0);

	if (!regs) {
		pr_err("aspeed-vic: could not map irq registers\n");
		return -EINVAL;
	}
	
	ast_vic = kzalloc(sizeof(struct aspeed_vic), GFP_KERNEL);
	if (!ast_vic) {
		iounmap(regs);
		return -ENOMEM;
	}

	ast_vic->base = regs;

	/* VIC Init */
	for(i = 0; i < 2; i++) {
		writel(0, AST_INTR_EN(i));
		writel(0, AST_INTR_SEL(i));
		writel(0, AST_INTR_SW_CLR(i));
		writel(0xFFFFFFFF, AST_INTR_DIS(i));
		writel(0xFFFFFFFF, AST_INTR_EDGE_CLR(i));
	}

	set_handle_irq(ast_vic_handle_irq);

	ast_vic->domain = irq_domain_add_simple(node, AST_VIC_NUM_IRQS, 0,
					 &ast_vic_dom_ops, ast_vic);

#ifdef CONFIG_FIQ
	init_FIQ(0);
#endif
	return 0;
}

IRQCHIP_DECLARE(aspeed_vic, "aspeed,ast-vic", ast_init_vic_of);
