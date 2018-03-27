/*
 * ast-p2x.c - p2x driver for the Aspeed SoC
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
#include <linux/irq.h>
#include <linux/msi.h>
#include <linux/list.h>
#include <linux/pci_regs.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
	
#include <mach/platform.h>
#include <asm/io.h>

#include <linux/irqchip/chained_irq.h>	
#include <mach/hardware.h>
#include <mach/regs-p2x.h>
#include <mach/ast_p2x.h>
//***********************************Information ***********************************
/* Register for MCTP  */
#define AST_P2X_CTRL			0x00		/*	Engine Status and Engine Control	*/
#define AST_P2X_INT				0x04		/*	Interrupt Enable and Status Register */
#define AST_P2X_ID				0x08		/*	Target ID and Mask */
#define AST_P2X_TX_DESC3		0x10		/*	Sending Descriptor [127:96] */
#define AST_P2X_TX_DESC2		0x14		/*	Sending Descriptor [95:64] */
#define AST_P2X_TX_DESC1		0x18		/*	Sending Descriptor [63:32] */
#define AST_P2X_TX_DESC0		0x1C		/*	Sending Descriptor [31:0] */
#define AST_P2X_TX_DATA		0x20		/*	Sending Data Port */
#define AST_P2X_RX_DESC3		0x40		/*	Received Descriptor [127:96] */
#define AST_P2X_RX_DESC2		0x44		/*	Received Descriptor [95:64] */
#define AST_P2X_RX_DESC1		0x48		/*	Received Descriptor [63:32] */
#define AST_P2X_RX_DESC0		0x4C		/*	Received Descriptor [31:0] */
#define AST_P2X_RX_DATA		0x50		/*	Received Data Port */

#define AST_P2X_MSI_IER			0x70		/*	MSI interrupt enalbe */
#define AST_P2X_MSI_ISR			0x74		/*	MSI interrupt sts */

#define AST_P2X_DEC_ADDR		0x80		/*	ADDR */
#define AST_P2X_DEC_MASK		0x84		/*	MASK */
#define AST_P2X_DEC_TAG		0x88		/*	TAG */

/*	AST_P2X_CTRL			0x00		Engine Status and Engine Control	*/
#define P2X_CTRL_GET_RX_LEN(x)		(((x >> 18) & 0xf) * 4)
#define P2X_CTRL_RX_IDLE			(1 << 17)
#define P2X_CTRL_TX_IDLE			(1 << 16)

#define P2X_CTRL_RX_MSI_EN			(1 << 5)
#define P2X_CTRL_UNLOCK_RX_BUFF	(1 << 4)
#define P2X_CTRL_RX_MATCH_EN		(1 << 3)
#define P2X_CTRL_DROP_DIS			(1 << 2)
#define P2X_CTRL_TX_TRIGGER		(1 << 1)
#define P2X_CTRL_RX_EN				(1)

/*	AST_P2X_INT			0x04		Interrupt Enable and Status Register */
#define P2X_INTD_EN				(1 << 21)
#define P2X_INTC_EN				(1 << 20)
#define P2X_INTB_EN				(1 << 19)
#define P2X_INTA_EN				(1 << 18)
#define P2X_RX_INT_EN			(1 << 17)
#define P2X_TX_INT_EN			(1 << 16)

#define P2X_PCIE_MSI				(1 << 6)
#define P2X_PCIE_INTD				(1 << 5)
#define P2X_PCIE_INTC				(1 << 4)
#define P2X_PCIE_INTB				(1 << 3)
#define P2X_PCIE_INTA				(1 << 2)
#define P2X_RX_COMPLETE			(1 << 1)
#define P2X_TX_COMPLETE			(1)
//***********************************Information ***********************************
//#define AST_P2X_DEBUG 

#ifdef AST_P2X_DEBUG
#define P2XDBUG(fmt, args...) printk("%s() " fmt, __FUNCTION__, ## args)
#else
#define P2XDBUG(fmt, args...)
#endif

struct ast_p2x_irq {
	void __iomem	*ast_p2x_base;
	int			parent_irq;
	struct irq_domain	*irq_domain;
};

struct ast_p2x_irq 	*ast_p2x;

static u8 txTag = 0;
static inline u32 
ast_p2x_read(u32 reg)
{
	u32 val;
		
	val = readl(ast_p2x->ast_p2x_base + reg);
	
//	P2XDBUG("reg = 0x%08x, val = 0x%08x\n", reg, val);
	
	return val;
}

static inline void
ast_p2x_write(u32 val, u32 reg) 
{
//	P2XDBUG("reg = 0x%08x, val = 0x%08x\n", reg, val);

	writel(val, ast_p2x->ast_p2x_base + reg);
}

//***********************************Information ***********************************

extern void ast_pcie_cfg_read(u8 type, u32 bdf_offset, u32 *value)
{
	u32 timeout, desc3, desc2;
r_again:
	timeout = 0;
	txTag %= 0x7;
//	printk("Read: type = %d, busfunc = %x ",type, bdf_offset);
	if((ast_p2x_read(AST_P2X_INT) & P2X_RX_COMPLETE) != 0)
		printk("EEEEEEEE  \n");
	
	ast_p2x_write(0x4000001 | (type << 24), AST_P2X_TX_DESC3);	
	ast_p2x_write(0x200f | (txTag << 8), AST_P2X_TX_DESC2);
	ast_p2x_write(bdf_offset, AST_P2X_TX_DESC1);
	ast_p2x_write(0, AST_P2X_TX_DESC0);
//	ast_p2x_write(0, AST_P2X_TX_DATA);

	//trigger
	ast_p2x_write(ast_p2x_read(AST_P2X_CTRL) |P2X_CTRL_TX_TRIGGER | P2X_CTRL_RX_EN, AST_P2X_CTRL);	

	//wait 
//	printf("trigger \n");
	while(!(ast_p2x_read(AST_P2X_INT) & P2X_RX_COMPLETE)) {
		timeout++;
		if(timeout > 10000) {
			printk("time out \n");
			*value = 0xffffffff;
			goto out;
		}
	};

	//read 
	desc3 = ast_p2x_read(AST_P2X_RX_DESC3);	
	desc2 = ast_p2x_read(AST_P2X_RX_DESC2);
	ast_p2x_read(AST_P2X_RX_DESC1);	

	if( ((desc3 >> 24) == 0x4A) && 
		((desc3 & 0xfff) == 0x1) && 
		((desc2 & 0xe000) == 0)) {
		*value = ast_p2x_read(AST_P2X_RX_DATA);
        } else if ( ((desc3 >> 24) == 0x0A) &&
                        ((desc2 & 0xe000) == 0x4000)) {
//                        printk("cfg read re-try \n");
                        ast_p2x_write(ast_p2x_read(AST_P2X_CTRL) | P2X_CTRL_UNLOCK_RX_BUFF |P2X_CTRL_RX_EN, AST_P2X_CTRL);
                        ast_p2x_write(ast_p2x_read(AST_P2X_INT) | P2X_TX_COMPLETE | P2X_RX_COMPLETE, AST_P2X_INT);
                        //wait
                        while(ast_p2x_read(AST_P2X_INT) & P2X_RX_COMPLETE);
                        goto r_again;
	} else {
		*value = 0xffffffff;		
		
	}

out:
	txTag++;
	ast_p2x_write(ast_p2x_read(AST_P2X_CTRL) | P2X_CTRL_UNLOCK_RX_BUFF |P2X_CTRL_RX_EN, AST_P2X_CTRL);	
	ast_p2x_write(ast_p2x_read(AST_P2X_INT) | P2X_TX_COMPLETE | P2X_RX_COMPLETE, AST_P2X_INT);	
	//wait 
	while(ast_p2x_read(AST_P2X_INT) & P2X_RX_COMPLETE);
//	printk("value %x \n",* value);
}

extern void ast_pcie_cfg_write(u8 type, u8 byte_en, u32 bdf_offset, u32 data)
{
	txTag %= 0x7;

//	printk("Write byte_en : %x, offset: %x, value = %x \n",byte_en , bdf_offset, data);

	ast_p2x_write(0x44000001 | (type << 24), AST_P2X_TX_DESC3);	
	ast_p2x_write(0x2000 | (txTag << 8) | byte_en, AST_P2X_TX_DESC2);
	ast_p2x_write(bdf_offset, AST_P2X_TX_DESC1);
	ast_p2x_write(0, AST_P2X_TX_DESC0);
	ast_p2x_write(data, AST_P2X_TX_DATA);	

	//trigger
	ast_p2x_write(ast_p2x_read(AST_P2X_CTRL) |P2X_CTRL_TX_TRIGGER | P2X_CTRL_RX_EN, AST_P2X_CTRL);	
//	printf("trigger \n");	
	//wait 
	while(!(ast_p2x_read(AST_P2X_INT) & P2X_RX_COMPLETE));

	//read TODO Check TAG 
	ast_p2x_read(AST_P2X_RX_DESC3);	
	ast_p2x_read(AST_P2X_RX_DESC2);
	ast_p2x_read(AST_P2X_RX_DESC1);	
//	while(header && tag )
	txTag++;	
	ast_p2x_write(ast_p2x_read(AST_P2X_CTRL) | P2X_CTRL_UNLOCK_RX_BUFF |P2X_CTRL_RX_EN, AST_P2X_CTRL);	
	ast_p2x_write(ast_p2x_read(AST_P2X_INT) | P2X_TX_COMPLETE | P2X_RX_COMPLETE, AST_P2X_INT);	
	//wait 
	while(ast_p2x_read(AST_P2X_INT) & P2X_RX_COMPLETE);

}

extern void ast_p2x_addr_map(u32 mask, u32 addr)
{
	//Address mapping
	ast_p2x_write(addr, AST_P2X_DEC_ADDR);
	ast_p2x_write(mask, AST_P2X_DEC_MASK);
	ast_p2x_write(0x00000028, AST_P2X_DEC_TAG);
}

static void
ast_p2x_irq_handler(struct irq_desc *desc)
{
	u32 msi = 0;
	u32 i;
	printk("TODO \n");
#if 0	
	struct irq_chip *chip = irq_desc_get_chip(desc);
	u32 sts = ast_p2x_read(AST_P2X_INT);

	P2XDBUG("sts :%x \n",sts);

	chained_irq_enter(chip, desc);
	
	if(sts & P2X_PCIE_INTA) {
		P2XDBUG("INT_INTA\n");
		generic_handle_irq(IRQ_PCIE_INTA);
	}

	if(sts & P2X_PCIE_INTB) {
		P2XDBUG("INT_INTB\n");
		generic_handle_irq(IRQ_PCIE_INTB);
	}

	if(sts & P2X_PCIE_INTC) {
		P2XDBUG("INT_INTC\n");
		generic_handle_irq(IRQ_PCIE_INTC);
	}

	if(sts & P2X_PCIE_INTD) {
		P2XDBUG("INT_INTD\n");
		generic_handle_irq(IRQ_PCIE_INTD);
	}

	if(sts & P2X_PCIE_MSI) {
		P2XDBUG("INT_MSI\n");
		msi = ast_p2x_read(AST_P2X_MSI_ISR);
		for(i = 0; i < 32; i++) {
			if(msi & (1 << i)) {
				generic_handle_irq(IRQ_PCIE_MSI0 + i);
			}
		}
	}

	chained_irq_exit(chip, desc);
#endif	
}


static void ast_p2x_ack_irq(struct irq_data *d)
{
	unsigned int p2x_irq = d->hwirq;
	if(p2x_irq > 3) {
		p2x_irq -= 4;
		ast_p2x_write(ast_p2x_read(AST_P2X_MSI_ISR) | (1 << p2x_irq) , AST_P2X_MSI_ISR);	
		P2XDBUG("clr irq %d , msi irq %d \n", d->irq, p2x_irq);
	} else {
		ast_p2x_write(ast_p2x_read(AST_P2X_INT) | (1 << (p2x_irq + 2)) , AST_P2X_INT);		
		P2XDBUG("clr irq %d , intabcd %d \n", d->irq, p2x_irq);		
	}	
	
}

static void ast_p2x_mask_irq(struct irq_data *d)
{
	unsigned int p2x_irq = d->hwirq;

	if(p2x_irq > 3) {
		p2x_irq -= 4;
		ast_p2x_write(ast_p2x_read(AST_P2X_MSI_IER) & ~(1 << p2x_irq) , AST_P2X_MSI_IER);	
		P2XDBUG("disable irq %d , msi irq %d \n", d->irq, p2x_irq);
	} else {
		//disable irq
		ast_p2x_write(ast_p2x_read(AST_P2X_INT) & ~(1 << (p2x_irq + 18)) , AST_P2X_INT);	
		P2XDBUG("disable irq %d , intabcd %d \n", d->irq, p2x_irq);		
	}
	
}

static void ast_p2x_unmask_irq(struct irq_data *d)
{
	unsigned int p2x_irq = d->hwirq;
	
	if(p2x_irq > 3) {
		p2x_irq -= 4;
		ast_p2x_write(ast_p2x_read(AST_P2X_MSI_IER) | (1 << p2x_irq) , AST_P2X_MSI_IER);	
		P2XDBUG("enable irq %d , msi irq %d \n", d->irq, p2x_irq);			
	} else {
		//Enable IRQ ..
		ast_p2x_write(ast_p2x_read(AST_P2X_INT) | (1 << (p2x_irq + 18)) , AST_P2X_INT);	
		P2XDBUG("enalbe irq %d , intabcd %d \n", d->irq, p2x_irq);		
	}
	
}

static struct irq_chip ast_p2x_irq_chip = {
	.name		= "P2X",
	.irq_ack		= ast_p2x_ack_irq,
	.irq_mask		= ast_p2x_mask_irq,
	.irq_unmask	= ast_p2x_unmask_irq,
};

#ifdef CONFIG_PCI_MSI
static DECLARE_BITMAP(msi_irq_in_use, AST_NUM_MSI_IRQS);

static int
msi_create_irq(void)
{
	int irq, pos;
again:
	pos = find_first_zero_bit(msi_irq_in_use, AST_NUM_MSI_IRQS);
	irq = IRQ_PCIE_MSI0 + pos;

	printk("0: pos %d , irq %d \n", pos, irq);
	if (irq > NR_IRQS)
		return -ENOSPC;
	/* test_and_set_bit operates on 32-bits at a time */
	if (test_and_set_bit(pos, msi_irq_in_use))
		goto again;

	P2XDBUG("1 : pos = %d, irq = %d\n", pos, irq);
//	dynamic_irq_init(irq);

	return irq;	
}

static void
msi_destroy_irq(unsigned int irq)
{
	int pos = irq - IRQ_PCIE_MSI0;
	P2XDBUG("\n");

//	dynamic_irq_cleanup(irq);

	clear_bit(pos, msi_irq_in_use);
}

extern void
arch_teardown_msi_irq(unsigned int irq)
{
	P2XDBUG("\n");

	msi_destroy_irq(irq);
}

int arch_setup_msi_irq(struct pci_dev *pdev, struct msi_desc *desc)
{
	int irq = msi_create_irq();
	struct msi_msg msg;

	P2XDBUG("\n");
	if (irq < 0)
		return irq;

	irq_set_msi_desc(irq, desc);	

	msg.address_hi = 0;
	msg.address_lo = 0x1e6f00f0;

	msg.data = irq - IRQ_PCIE_MSI0;
	printk("msi.data = %d\n", msg.data);

	write_msi_msg(irq, &msg);
//	set_irq_chip_and_handler(irq, &ast_p2x_irq_chip, handle_simple_irq);
	
	ast_p2x_write(ast_p2x_read(AST_P2X_CTRL) | P2X_CTRL_RX_MSI_EN, AST_P2X_CTRL);

	return 0;
}


#endif	// CONFIG_PCI_MSI

#define PCIE_IRQ_NUM 4
/////////////////////////////////////////
#ifdef CONFIG_PCI_MSI
#define AST_NUM_MSI_IRQS				32
#else
#define AST_NUM_MSI_IRQS				0
#endif

static int ast_p2x_map_irq_domain(struct irq_domain *domain,
					unsigned int irq, irq_hw_number_t hwirq)
{
	irq_set_chip_and_handler(irq, &ast_p2x_irq_chip, ast_p2x_irq_handler);
	irq_set_chip_data(irq, domain->host_data);

	return 0;
}

static const struct irq_domain_ops ast_p2x_irq_domain_ops = {
	.map = ast_p2x_map_irq_domain,
};

static int __init ast_p2x_irq_of_init(void)
{
	P2XDBUG("\n");
	int	parent_irq;

	ast_p2x = kzalloc(sizeof(*i2c_irq), GFP_KERNEL);
	if (!ast_p2x)
		return -ENOMEM;
	
	ast_p2x->ast_p2x_base = of_iomap(node, 0);

	ast_p2x->parent_irq = irq_of_parse_and_map(node, 0);
	if (ast_p2x->parent_irq < 0)
		return parent_irq;

	ast_p2x->irq_domain = irq_domain_add_linear(
			node, AST_NUM_MSI_IRQS + PCIE_IRQ_NUM,
			&ast_p2x_irq_domain_ops, NULL);
	if (ast_p2x->irq_domain)
		return -ENOMEM;

	ast_p2x->irq_domain->name = "ast-p2x-domain";

	ast_p2x_write(P2X_CTRL_DROP_DIS, AST_P2X_CTRL);	


	ast_p2x->irq_domain->name = "ast-p2x-domain";

	irq_set_chained_handler_and_data(ast_p2x->parent_irq,
					 ast_p2x_irq_handler, ast_p2x);

	return 0;
}

IRQCHIP_DECLARE(ast_p2x, "aspeed,ast-p2x", ast_p2x_irq_of_init);
