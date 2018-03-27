/*
 * ast-gpio.c - GPIO driver for the Aspeed SoC
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
#include <linux/clk.h>
#include <linux/bitops.h>
#include <linux/gpio/driver.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/module.h>
#include <linux/irqdomain.h>
#include <linux/interrupt.h>

/*************************************************************/
#define GPIO_PER_PORT_PIN_NUM				8

#define GPIO_INPUT_MODE					0
#define GPIO_OUTPUT_MODE					1

#define GPIO_RISING_EDGE					1
#define GPIO_FALLING_EDGE					0

#define GPIO_LEVEL_HIGH						1
#define GPIO_LEVEL_LOW						1

#define GPIO_EDGE_MODE						0
#define GPIO_LEVEL_MODE					1

#define GPIO_EDGE_LEVEL_MODE				0
#define GPIO_DUAL_EDGE_MODE				1

#define GPIO_NO_DEBOUNCE					0
#define GPIO_DEBOUNCE_TIMER0				2		//GPIO 50 as debounce timer
#define GPIO_DEBOUNCE_TIMER1				1		//GPIO 54 as debounce timer
#define GPIO_DEBOUNCE_TIMER2				3		//GPIO 58 as debounce timer

#define GPIO_CMD_ARM						0
#define GPIO_CMD_LPC						1		
#define GPIO_CMD_COPROCESSOR				2
/*************************************************************/
//#define AST_GPIO_DEBUG

#ifdef AST_GPIO_DEBUG
#define GPIODBUG(fmt, args...) printk(KERN_DEBUG "%s() " fmt, __FUNCTION__, ## args)
#else
#define GPIODBUG(fmt, args...)
#endif
/*************************************************************/
//GPIO group structure 
struct ast_gpio_bank {
	int 		id;
	u32		data_offset;
	u32		dir_offset;
	u32		int_en_offset;	
	u32		int_type_offset;		
	u32		int_sts_offset;	
	u32		rst_tol_offset;		
	u32		debounce_offset;	
	u32		cmd_source_offset;
	struct ast_gpio	*gpio;
	u32 	timer0;
	u32 	timer1;
	u32 	timer2;
};

struct ast_gpio_config {
	unsigned int nr_gpio_ports;
	struct ast_gpio_bank *gpio_bank;
};

struct ast_gpio {
	void __iomem *reg_base;	
	int irq;	
	spinlock_t lock;	
	struct gpio_chip chip;
//	struct platform_device *pdev;	
	struct ast_gpio_config *config;
};


static inline u32
ast_gpio_read(struct ast_gpio_bank *ast_gpio ,u32 offset)
{
#ifdef AST_GPIO_DEBUG
	u32 v = readl(ast_gpio->gpio->reg_base + offset);
	GPIODBUG("offset = 0x%08x, val = %x \n", offset, v);
	return v;
#else
	return readl(ast_gpio->gpio->reg_base + offset);
#endif	
}

static inline void
ast_gpio_write(struct ast_gpio_bank *ast_gpio , u32 val, u32 offset)
{
    GPIODBUG("write offset = 0x%08x, val = 0x%08x\n", offset, val);
    writel(val, ast_gpio->gpio->reg_base + offset);
}

/***************************************************************************************/
static int
ast_gpio_direction_input(struct gpio_chip *chip, unsigned offset)
{
	struct ast_gpio *gpio = gpiochip_get_data(chip);
	struct ast_gpio_bank *gpio_bank = &gpio->config->gpio_bank[offset/8];
	unsigned long flags;
	u32 v;
	int ret = -1;

	GPIODBUG("dir_in [%d] \n", offset);

	local_irq_save(flags);

	v = ast_gpio_read(gpio_bank, gpio_bank->dir_offset);
	
	v &= ~(GPIO_OUTPUT_MODE << (offset % 32));
	ast_gpio_write(gpio_bank, v, gpio_bank->dir_offset);

	ret = 0;

	local_irq_restore(flags);

	return ret;

}

static int
ast_gpio_direction_output(struct gpio_chip *chip, unsigned offset, int val)
{
	struct ast_gpio *gpio = gpiochip_get_data(chip);
	struct ast_gpio_bank *gpio_bank = &gpio->config->gpio_bank[offset/8];
	unsigned long flags;
	u32 v;
	int ret = -1;
	GPIODBUG("dir_out pin %d val %d\n", offset, val);

	local_irq_save(flags);

	/* Set the value */
	v = ast_gpio_read(gpio_bank, gpio_bank->data_offset);
	if (val) 
		v |= (1 << (offset % 32));
	else
		v &= ~(1 << (offset % 32));
	ast_gpio_write(gpio_bank, v, gpio_bank->data_offset);

	/* Drive as an output */
	v = ast_gpio_read(gpio_bank, gpio_bank->dir_offset);
	v |= (GPIO_OUTPUT_MODE << (offset % 32));
	ast_gpio_write(gpio_bank, v, gpio_bank->dir_offset);

	local_irq_restore(flags);

	ret = 0;	

	return ret;
}

/*0=out, 1=in, */
static int ast_gpio_get_direction(struct gpio_chip *chip, unsigned int offset)
{
	struct ast_gpio *gpio = gpiochip_get_data(chip);
	struct ast_gpio_bank *gpio_bank = &gpio->config->gpio_bank[offset/8];
	unsigned long flags;
	int dir = 1;
	u32 v;

	local_irq_save(flags);

	v = ast_gpio_read(gpio_bank, gpio_bank->dir_offset);
	if(v & (GPIO_OUTPUT_MODE << (offset % 32)))
	 	dir = 0;
	else
		dir = 1;

	local_irq_restore(flags);
//	GPIODBUG("ast_gpio_get_direction offset %d dir %d [0=out, 1=in]\n", offset, dir);

	return dir;
}

static int
ast_gpio_get(struct gpio_chip *chip, unsigned offset)
{
	struct ast_gpio *gpio = gpiochip_get_data(chip);
	struct ast_gpio_bank *gpio_bank = &gpio->config->gpio_bank[offset/8];
	unsigned long flags;
	u32 v;

	GPIODBUG("Get [%d] \n", offset);

	local_irq_save(flags);

	v = ast_gpio_read(gpio_bank, gpio_bank->data_offset);

	v &= (1 << (offset % 32));

	if(v)
		v = 1;
	else
		v = 0;

	local_irq_restore(flags);

    return v;
}

static void
ast_gpio_set(struct gpio_chip *chip, unsigned offset, int val)
{
	struct ast_gpio *gpio = gpiochip_get_data(chip);
	struct ast_gpio_bank *gpio_bank = &gpio->config->gpio_bank[offset/8];

	unsigned long flags;
	u32 v;
	GPIODBUG("Set %s[%d] = %d\n",chip->label, offset, val);

	local_irq_save(flags);

	/* Set the value */

	v = ast_gpio_read(gpio_bank, gpio_bank->data_offset);

	if (val)
		v |= (1 << (offset % 32));
	else
		v &= ~(1 << (offset % 32));

	ast_gpio_write(gpio_bank, v, gpio_bank->data_offset);

	local_irq_restore(flags);

}

//mode 0 : no debounce , 1: set  0x50, 2: 0x54, 3: 0x58 as debonce timer
static int
ast_gpio_set_debounce(struct gpio_chip *chip, unsigned offset, unsigned debounce)
{
	struct ast_gpio *gpio = gpiochip_get_data(chip);
	struct ast_gpio_bank *gpio_bank = &gpio->config->gpio_bank[offset/8];
	u32 set0, set1, port;
	u32 timer0, timer1, timer2;

	set0 = ast_gpio_read(gpio_bank, gpio_bank->debounce_offset);	
	set1 = ast_gpio_read(gpio_bank, gpio_bank->debounce_offset + 0x04);

	port = offset % 32;

	if(debounce) {
		//find value near 
		timer0 = abs(debounce - gpio_bank->timer0);
		timer1 = abs(debounce - gpio_bank->timer1);
		timer2= abs(debounce - gpio_bank->timer2);
		if(timer0 < timer1) {
			if(timer0 < timer2) {
				debounce = 1;
			} else {
				debounce = 2;
			}
		} else {
			if(timer1 < timer2) {
				debounce = 2;
			} else {
				debounce = 3;
			}		
		}
	}
	switch(debounce) {
		case 0:	//no debonce 	
			set0 &= ~(1 << port);
			set1 &= ~(1 << port);
			break;
		case 1:	//GPIO 0x50 as debonce timer
			set0 &= ~(1 << port);
			set1 |= (1 << port);
			break;
		case 2: //GPIO 0x54 as debonce timer
			set0 |= (1 << port);
			set1 &= ~(1 << port);
			break;			
		case 3:	//GPIO 0x58 as debonce timer
			set0 |= (1 << port);
			set1 |= (1 << port);
		break;
		default:
			dev_err(chip->parent, "Debounce value %u not in range\n",
				debounce);
			return -EINVAL;
		break;

	}

	ast_gpio_write(gpio_bank, set0, gpio_bank->debounce_offset);	
	ast_gpio_write(gpio_bank, set1, gpio_bank->debounce_offset + 0x04);	
	GPIODBUG("ast_gpio_set_debounce [%d] = %d\n", offset, debounce);

	return 0;
}

/***************************************************************************************/
//timer 0/1/2
//Debounce time = PCLK * (val+1)
void ast_set_gpio_debounce_timer(int timer, int val)
{
#if 0
	struct ast_gpio_bank *ast_gpio = &ast_gpio_gp[0];

	switch(timer) {
		case 1:
			ast_gpio_write(ast_gpio, val, 0x50);	
			break;
		case 2:
			ast_gpio_write(ast_gpio, val, 0x54);	
			break;
		case 3:
			ast_gpio_write(ast_gpio, val, 0x58);	
			break;
	}
#endif	
}

EXPORT_SYMBOL(ast_set_gpio_debounce_timer);


//TODO ......
//
void ast_set_gpio_tolerant(int gpio_port, int mode)
{
#if 0 
	u32 set0, set1;
	u16 gp, port;
	gp = gpio_port / 4;
	port = gpio_port % 4;
	set0 = ast_gpio_read(&ast_gpio_gp[gp], ast_gpio_gp[gp].debounce_offset);
	set1 = ast_gpio_read(&ast_gpio_gp[gp], ast_gpio_gp[gp].debounce_offset + 0x04);

	switch(port) {
		case 0:		//A , H , ......
			set0 = port 
			ast_gpio_write(ast_gpio, val, 0x50);
			break;
		case 1:
			ast_gpio_write(ast_gpio, val, 0x54);
			break;
		case 2:
			ast_gpio_write(ast_gpio, val, 0x58);
			break;
		case 3:
			ast_gpio_write(ast_gpio, val, 0x58);
		break;
		default:
			GPIODBUG("not support \n");
			return;
		break;

	}

	ast_gpio_write(&ast_gpio_gp[gp], set0, ast_gpio_gp[gp].debounce_offset);
	ast_gpio_write(&ast_gpio_gp[gp], set1, ast_gpio_gp[gp].debounce_offset + 0x04);
#endif
}

EXPORT_SYMBOL(ast_set_gpio_tolerant);

/*
 * We need to unmask the GPIO bank interrupt as soon as possible to
 * avoid missing GPIO interrupts for other lines in the bank.
 * Then we need to mask-read-clear-unmask the triggered GPIO lines
 * in the bank to avoid missing nested interrupts for a GPIO line.
 * If we wait to unmask individual GPIO lines in the bank after the
 * line's interrupt handler has been run, we may miss some nested
 * interrupts.
 */
static void 
ast_gpio_irq_handler(struct irq_desc *desc)
{
	unsigned long isr;
	int i,j, irq;
	struct gpio_chip *g_chip = irq_desc_get_handler_data(desc);
	struct ast_gpio *gpio = gpiochip_get_data(g_chip);
	struct irq_chip *chip = irq_desc_get_chip(desc);
	struct ast_gpio_bank *gpio_bank;	

	chained_irq_enter(chip, desc);
	GPIODBUG("ast_gpio_irq_handler\n ");

	for (i = 0; i < gpio->config->nr_gpio_ports/4; i++) {
		gpio_bank = &gpio->config->gpio_bank[i*4];
		GPIODBUG("4 port gp bank [%d]  \n", i);
		isr = ast_gpio_read(gpio_bank, gpio_bank->int_sts_offset);
		GPIODBUG("isr %lx \n", isr);
		for_each_set_bit(j, &isr, 32) {
			irq = irq_find_mapping(g_chip->irqdomain, i * 32 + j);
			GPIODBUG("generic_handle_irq %d \n", irq);
			generic_handle_irq(irq);
		}
	}

	chained_irq_exit(chip, desc);
}

static void ast_gpio_ack_irq(struct irq_data *d)
{
	struct ast_gpio *gpio = gpiochip_get_data(irq_data_get_irq_chip_data(d));
	struct ast_gpio_bank *gpio_bank = &gpio->config->gpio_bank[d->hwirq / 8];
	u32 gpio_port_pin = d->hwirq % 32;

	GPIODBUG("ast_gpio_ack_irq irq [%d] hwirq [%ld] \n ",d->irq, d->hwirq);

	ast_gpio_write(gpio_bank, (1 << gpio_port_pin), gpio_bank->int_sts_offset);

	GPIODBUG("read sts %x\n ",ast_gpio_read(gpio_bank, gpio_bank->int_sts_offset));

}

static void ast_gpio_mask_irq(struct irq_data *d)
{
	struct ast_gpio *gpio = gpiochip_get_data(irq_data_get_irq_chip_data(d));
	struct ast_gpio_bank *gpio_bank = &gpio->config->gpio_bank[d->hwirq / 8];
	u32 gpio_port_pin = d->hwirq % 32;

	GPIODBUG("ast_gpio_mask_irq irq [%d] hwirq [%ld]\n ",d->irq, d->hwirq);

	//disable irq
	ast_gpio_write(gpio_bank, ast_gpio_read(gpio_bank, gpio_bank->int_en_offset) &
			~(1<< gpio_port_pin), gpio_bank->int_en_offset);

}

static void ast_gpio_unmask_irq(struct irq_data *d)
{
	struct ast_gpio *gpio = gpiochip_get_data(irq_data_get_irq_chip_data(d));
	struct ast_gpio_bank *gpio_bank = &gpio->config->gpio_bank[d->hwirq / 8];
	u32 gpio_port_pin = d->hwirq % 32;

	GPIODBUG("ast_gpio_unmask_irq irq [%d] hwirq [%ld] \n ",d->irq, d->hwirq);

	//Enable IRQ ..
	ast_gpio_write(gpio_bank, 1<< gpio_port_pin, gpio_bank->int_sts_offset);

	ast_gpio_write(gpio_bank, ast_gpio_read(gpio_bank, gpio_bank->int_en_offset) |
			(1<< gpio_port_pin), gpio_bank->int_en_offset);

}

static int
ast_gpio_irq_type(struct irq_data *d, unsigned int type)
{
	struct ast_gpio *gpio = gpiochip_get_data(irq_data_get_irq_chip_data(d));
	struct ast_gpio_bank *gpio_bank = &gpio->config->gpio_bank[d->hwirq / 8];
	u32 gpio_port_pin = d->hwirq % 32;
	u32 type0, type1, type2;

	GPIODBUG("ast_gpio_irq_type irq [%d] hwirq [%ld]\n ",d->irq, d->hwirq);

	if (type & ~IRQ_TYPE_SENSE_MASK)
		return -EINVAL;

	type0 = ast_gpio_read(gpio_bank, gpio_bank->int_type_offset);
	type1 = ast_gpio_read(gpio_bank, gpio_bank->int_type_offset + 0x04);
	type2 = ast_gpio_read(gpio_bank, gpio_bank->int_type_offset + 0x08);

	switch(type) {
		/* Edge rising type */
		case IRQ_TYPE_EDGE_RISING:
			type0 |=(1<<gpio_port_pin);
			type1 &=~(1<<gpio_port_pin);
			type2 &=~(1<<gpio_port_pin);
			break;
		/* Edge falling type */
		case IRQ_TYPE_EDGE_FALLING:
			type0 &= ~(1<<gpio_port_pin);
			type1 &=~(1<<gpio_port_pin);
			type2 &=~(1<<gpio_port_pin);
			break;
		case IRQ_TYPE_EDGE_BOTH:
			type2 |= (1<<gpio_port_pin);
			break;
		case IRQ_TYPE_LEVEL_HIGH:
			type0 |=(1<<gpio_port_pin);
			type1 |=(1<<gpio_port_pin);
			type2 &=~(1<<gpio_port_pin);
			break;
		case IRQ_TYPE_LEVEL_LOW:
			type0 &=~(1<<gpio_port_pin);
			type1 |=(1<<gpio_port_pin);
			type2 &=~(1<<gpio_port_pin);
			break;
		default:
			GPIODBUG("not support trigger");
			return -EINVAL;
			break;
	}

	ast_gpio_write(gpio_bank, type0, gpio_bank->int_type_offset);
	ast_gpio_write(gpio_bank, type1, gpio_bank->int_type_offset + 0x04);
	ast_gpio_write(gpio_bank, type2, gpio_bank->int_type_offset + 0x08);

	return 0;

}

static struct irq_chip ast_gpio_irq_chip = {
	.name		= "ast-gpio",
	.irq_ack		= ast_gpio_ack_irq,
	.irq_mask		= ast_gpio_mask_irq,
	.irq_unmask	= ast_gpio_unmask_irq,
	.irq_set_type	= ast_gpio_irq_type,
};

#define AST_GPIO_BANK(name, index_no, data, dir, int_en, int_type, int_sts, rst_tol, debounce, cmd_s)	\
{											\
	.data_offset = data,						\
	.dir_offset = dir, 							\
	.int_en_offset = int_en,					\
	.int_type_offset = int_type,				\
	.int_sts_offset = int_sts, 					\
	.rst_tol_offset = rst_tol,					\
	.debounce_offset = debounce,				\
	.cmd_source_offset = cmd_s,				\
}

static struct ast_gpio_bank ast_cam_bank[] = {
        AST_GPIO_BANK("GPIOA", 0, 0x000, 0x004, 0x008, 0x00c, 0x018, 0x01c, 0x020, 0x00),
        AST_GPIO_BANK("GPIOB", 1, 0x000, 0x004, 0x008, 0x00c, 0x018, 0x01c, 0x020, 0x00),
        AST_GPIO_BANK("GPIOC", 2, 0x000, 0x004, 0x008, 0x00c, 0x018, 0x01c, 0x020, 0x00),
        AST_GPIO_BANK("GPIOD", 3, 0x000, 0x004, 0x008, 0x00c, 0x018, 0x01c, 0x020, 0x00),
        AST_GPIO_BANK("GPIOE", 0, 0x030, 0x034, 0x038, 0x03c, 0x048, 0x04c, 0x050, 0x00),
        AST_GPIO_BANK("GPIOF", 1, 0x030, 0x034, 0x038, 0x03c, 0x048, 0x04c, 0x050, 0x00),
        AST_GPIO_BANK("GPIOG", 2, 0x030, 0x034, 0x038, 0x03c, 0x048, 0x04c, 0x050, 0x00),
        AST_GPIO_BANK("GPIOH", 3, 0x030, 0x034, 0x038, 0x03c, 0x048, 0x04c, 0x050, 0x00),
        AST_GPIO_BANK("GPIOI", 0, 0x070, 0x074, 0x078, 0x07c, 0x088, 0x08c, 0x00, 0x00),
        AST_GPIO_BANK("GPIOJ", 1, 0x070, 0x074, 0x078, 0x07c, 0x088, 0x08c, 0x00, 0x00),
        AST_GPIO_BANK("GPIOK", 2, 0x070, 0x074, 0x078, 0x07c, 0x088, 0x08c, 0x00, 0x00),
        AST_GPIO_BANK("GPIOL", 3, 0x070, 0x074, 0x078, 0x07c, 0x088, 0x08c, 0x00, 0x00),
};

static struct ast_gpio_bank ast_g4_bank[] = {
	AST_GPIO_BANK("GPIOA", 0, 0x000, 0x004, 0x008, 0x00c, 0x018, 0x01c, 0x040, 0x060),
	AST_GPIO_BANK("GPIOB", 1, 0x000, 0x004, 0x008, 0x00c, 0x018, 0x01c, 0x040, 0x060),
	AST_GPIO_BANK("GPIOC", 2, 0x000, 0x004, 0x008, 0x00c, 0x018, 0x01c, 0x040, 0x060),
	AST_GPIO_BANK("GPIOD", 3, 0x000, 0x004, 0x008, 0x00c, 0x018, 0x01c, 0x040, 0x060),
	AST_GPIO_BANK("GPIOE", 0, 0x020, 0x024, 0x028, 0x02c, 0x038, 0x03c, 0x048, 0x068),
	AST_GPIO_BANK("GPIOF", 1, 0x020, 0x024, 0x028, 0x02c, 0x038, 0x03c, 0x048, 0x068),
	AST_GPIO_BANK("GPIOG", 2, 0x020, 0x024, 0x028, 0x02c, 0x038, 0x03c, 0x048, 0x068),
	AST_GPIO_BANK("GPIOH", 3, 0x020, 0x024, 0x028, 0x02c, 0x038, 0x03c, 0x048, 0x068),
	AST_GPIO_BANK("GPIOI", 0, 0x070, 0x074, 0x098, 0x09c, 0x0a8, 0x0ac, 0x0b0, 0x090),
	AST_GPIO_BANK("GPIOJ", 1, 0x070, 0x074, 0x098, 0x09c, 0x0a8, 0x0ac, 0x0b0, 0x090),
	AST_GPIO_BANK("GPIOK", 2, 0x070, 0x074, 0x098, 0x09c, 0x0a8, 0x0ac, 0x0b0, 0x090),
	AST_GPIO_BANK("GPIOL", 3, 0x070, 0x074, 0x098, 0x09c, 0x0a8, 0x0ac, 0x0b0, 0x090),
	AST_GPIO_BANK("GPIOM", 0, 0x078, 0x07c, 0x0e8, 0x0ec, 0x0f8, 0x0fc, 0x100, 0x0e0),
	AST_GPIO_BANK("GPION", 1, 0x078, 0x07c, 0x0e8, 0x0ec, 0x0f8, 0x0fc, 0x100, 0x0e0),
	AST_GPIO_BANK("GPIOO", 2, 0x078, 0x07c, 0x0e8, 0x0ec, 0x0f8, 0x0fc, 0x100, 0x0e0),
	AST_GPIO_BANK("GPIOP", 3, 0x078, 0x07c, 0x0e8, 0x0ec, 0x0f8, 0x0fc, 0x100, 0x0e0),
	AST_GPIO_BANK("GPIOQ", 0, 0x080, 0x084, 0x118, 0x11c, 0x128, 0x12c, 0x130, 0x110),
	AST_GPIO_BANK("GPIOR", 1, 0x080, 0x084, 0x118, 0x11c, 0x128, 0x12c, 0x130, 0x110),
	AST_GPIO_BANK("GPIOS", 2, 0x080, 0x084, 0x118, 0x11c, 0x128, 0x12c, 0x130, 0x110),
	AST_GPIO_BANK("GPIOT", 3, 0x080, 0x084, 0x118, 0x11c, 0x128, 0x12c, 0x130, 0x110),
	AST_GPIO_BANK("GPIOU", 0, 0x088, 0x08c, 0x148, 0x14c, 0x158, 0x15c, 0x160, 0x140),
	AST_GPIO_BANK("GPIOV", 1, 0x088, 0x08c, 0x148, 0x14c, 0x158, 0x15c, 0x160, 0x140),
	AST_GPIO_BANK("GPIOW", 2, 0x088, 0x08c, 0x148, 0x14c, 0x158, 0x15c, 0x160, 0x140),
	AST_GPIO_BANK("GPIOX", 3, 0x088, 0x08c, 0x148, 0x14c, 0x158, 0x15c, 0x160, 0x140),
	AST_GPIO_BANK("GPIOY", 0, 0x1e0, 0x1e4, 0x178, 0x17c, 0x188, 0x18c, 0x190, 0x170),
	AST_GPIO_BANK("GPIOZ", 1, 0x1e0, 0x1e4, 0x178, 0x17c, 0x188, 0x18c, 0x190, 0x170),
	AST_GPIO_BANK("GPIOAA", 2, 0x1e0, 0x1e4, 0x178, 0x17c, 0x188, 0x18c, 0x190, 0x170),
	AST_GPIO_BANK("GPIOAB", 3, 0x1e0, 0x1e4, 0x178, 0x17c, 0x188, 0x18c, 0x190, 0x170),
};

static struct ast_gpio_bank ast_g5_bank[] = {
	AST_GPIO_BANK("GPIOA", 0, 0x000, 0x004, 0x008, 0x00c, 0x018, 0x01c, 0x040, 0x060),
	AST_GPIO_BANK("GPIOB", 1, 0x000, 0x004, 0x008, 0x00c, 0x018, 0x01c, 0x040, 0x060),
	AST_GPIO_BANK("GPIOC", 2, 0x000, 0x004, 0x008, 0x00c, 0x018, 0x01c, 0x040, 0x060),
	AST_GPIO_BANK("GPIOD", 3, 0x000, 0x004, 0x008, 0x00c, 0x018, 0x01c, 0x040, 0x060),
	AST_GPIO_BANK("GPIOE", 0, 0x020, 0x024, 0x028, 0x02c, 0x038, 0x03c, 0x048, 0x068),
	AST_GPIO_BANK("GPIOF", 1, 0x020, 0x024, 0x028, 0x02c, 0x038, 0x03c, 0x048, 0x068),
	AST_GPIO_BANK("GPIOG", 2, 0x020, 0x024, 0x028, 0x02c, 0x038, 0x03c, 0x048, 0x068),
	AST_GPIO_BANK("GPIOH", 3, 0x020, 0x024, 0x028, 0x02c, 0x038, 0x03c, 0x048, 0x068),
	AST_GPIO_BANK("GPIOI", 0, 0x070, 0x074, 0x098, 0x09c, 0x0a8, 0x0ac, 0x0b0, 0x090),
	AST_GPIO_BANK("GPIOJ", 1, 0x070, 0x074, 0x098, 0x09c, 0x0a8, 0x0ac, 0x0b0, 0x090),
	AST_GPIO_BANK("GPIOK", 2, 0x070, 0x074, 0x098, 0x09c, 0x0a8, 0x0ac, 0x0b0, 0x090),
	AST_GPIO_BANK("GPIOL", 3, 0x070, 0x074, 0x098, 0x09c, 0x0a8, 0x0ac, 0x0b0, 0x090),
	AST_GPIO_BANK("GPIOM", 0, 0x078, 0x07c, 0x0e8, 0x0ec, 0x0f8, 0x0fc, 0x100, 0x0e0),
	AST_GPIO_BANK("GPION", 1, 0x078, 0x07c, 0x0e8, 0x0ec, 0x0f8, 0x0fc, 0x100, 0x0e0),
	AST_GPIO_BANK("GPIOO", 2, 0x078, 0x07c, 0x0e8, 0x0ec, 0x0f8, 0x0fc, 0x100, 0x0e0),
	AST_GPIO_BANK("GPIOP", 3, 0x078, 0x07c, 0x0e8, 0x0ec, 0x0f8, 0x0fc, 0x100, 0x0e0),
	AST_GPIO_BANK("GPIOQ", 0, 0x080, 0x084, 0x118, 0x11c, 0x128, 0x12c, 0x130, 0x110),
	AST_GPIO_BANK("GPIOR", 1, 0x080, 0x084, 0x118, 0x11c, 0x128, 0x12c, 0x130, 0x110),
	AST_GPIO_BANK("GPIOS", 2, 0x080, 0x084, 0x118, 0x11c, 0x128, 0x12c, 0x130, 0x110),
	AST_GPIO_BANK("GPIOT", 3, 0x080, 0x084, 0x118, 0x11c, 0x128, 0x12c, 0x130, 0x110),
	AST_GPIO_BANK("GPIOU", 0, 0x088, 0x08c, 0x148, 0x14c, 0x158, 0x15c, 0x160, 0x140),
	AST_GPIO_BANK("GPIOV", 1, 0x088, 0x08c, 0x148, 0x14c, 0x158, 0x15c, 0x160, 0x140),
	AST_GPIO_BANK("GPIOW", 2, 0x088, 0x08c, 0x148, 0x14c, 0x158, 0x15c, 0x160, 0x140),
	AST_GPIO_BANK("GPIOX", 3, 0x088, 0x08c, 0x148, 0x14c, 0x158, 0x15c, 0x160, 0x140),
	AST_GPIO_BANK("GPIOY", 0, 0x1e0, 0x1e4, 0x178, 0x17c, 0x188, 0x18c, 0x190, 0x170),
	AST_GPIO_BANK("GPIOZ", 1, 0x1e0, 0x1e4, 0x178, 0x17c, 0x188, 0x18c, 0x190, 0x170),
	AST_GPIO_BANK("GPIOAA", 2, 0x1e0, 0x1e4, 0x178, 0x17c, 0x188, 0x18c, 0x190, 0x170),
	AST_GPIO_BANK("GPIOAB", 3, 0x1e0, 0x1e4, 0x178, 0x17c, 0x188, 0x18c, 0x190, 0x170),
	AST_GPIO_BANK("GPIOAC", 0, 0x1e8, 0x1ec, 0x1a8, 0x1ac, 0x1b8, 0x1bc, 0x1c0, 0x1a0),
};

static const struct ast_gpio_config ast_g4_config = { 
	.nr_gpio_ports = 28, 
	.gpio_bank = ast_g4_bank, 
};

static const struct ast_gpio_config ast_g5_config = { 
	.nr_gpio_ports = 29, 
	.gpio_bank = ast_g5_bank, 
};

static const struct ast_gpio_config ast_cam_config = { 
	.nr_gpio_ports = 12, 
	.gpio_bank = ast_cam_bank, 
};

static const struct of_device_id ast_gpio_of_table[] = {
	{ .compatible = "aspeed,ast-g4-gpio",	.data = &ast_g4_config, },
	{ .compatible = "aspeed,ast-g5-gpio",	.data = &ast_g5_config, },
	{ .compatible = "aspeed,ast-cam-gpio",	.data = &ast_cam_config, },	
	{}
};

MODULE_DEVICE_TABLE(of, ast_gpio_of_table);

static int __init
ast_gpio_probe(struct platform_device *pdev)
{
	int i, rc;
	const struct of_device_id *gpio_dev_id;
	struct resource *res;	
	struct ast_gpio *gpio;
	struct ast_gpio_bank *gpio_bank;
	struct clk 			*clk;
	u32					apb_clk;

	gpio = devm_kzalloc(&pdev->dev, sizeof(struct ast_gpio), GFP_KERNEL);
	if (!gpio)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENXIO;
	
	gpio->reg_base = devm_ioremap_resource(&pdev->dev, res);
	if (!gpio->reg_base)
		return -ENOMEM;

	gpio_dev_id = of_match_node(ast_gpio_of_table, pdev->dev.of_node);
	if (!gpio_dev_id)
		return -EINVAL;

	gpio->config = (struct ast_gpio_config *) gpio_dev_id->data;

	platform_set_drvdata(pdev, gpio);

	gpio->chip.parent = &pdev->dev;
	gpio->chip.ngpio = gpio->config->nr_gpio_ports * 8;
	gpio->chip.direction_input = ast_gpio_direction_input;
	gpio->chip.direction_output = ast_gpio_direction_output;
	gpio->chip.get_direction = ast_gpio_get_direction;
#if 0		
	gpio->chip.request = ast_gpio_request;
	gpio->chip.free = ast_gpio_free;
	gpio->chip.set_config = ast_gpio_set_config;		/* new kernel */
#endif	
	gpio->chip.get = ast_gpio_get;
	gpio->chip.set = ast_gpio_set;
	gpio->chip.set_debounce = ast_gpio_set_debounce;

	GPIODBUG("gpio port num %d, total gpio pin : %d\n", gpio->config->nr_gpio_ports, gpio->chip.ngpio);

	for (i = 0; i < gpio->config->nr_gpio_ports; i++) {
		gpio_bank = &gpio->config->gpio_bank[i];
		gpio_bank->id = i;
		gpio_bank->gpio = gpio;
		//Set Level Trigger
		ast_gpio_write(gpio_bank, 0xffffffff, gpio_bank->int_type_offset);
		ast_gpio_write(gpio_bank, 0xffffffff, gpio_bank->int_type_offset + 0x04);
		ast_gpio_write(gpio_bank, 0, gpio_bank->int_type_offset + 0x08);
		//Disable IRQ 
		ast_gpio_write(gpio_bank, 0x0, gpio_bank->int_en_offset);
	}

	//init debuance timer 0/1/2 [0x50,0x54,0x58]
	clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(clk)) {
		dev_err(&pdev->dev, "gpio no clock defined\n");
		return -ENODEV;
	}
	apb_clk = clk_get_rate(clk);
	//set debunce timer 
	gpio_bank->timer0 = 100; 		//100us 
	gpio_bank->timer1 = 10000; 		//10ms
	gpio_bank->timer2 = 100000; 	//100ms

//	printk("apb_clk %d \n", apb_clk);

	//debounce timer value = PCLK / debounce time 
	ast_gpio_write(gpio_bank, gpio_bank->timer0 * (apb_clk/1000000), 0x50);
	ast_gpio_write(gpio_bank, gpio_bank->timer1 * (apb_clk/1000000), 0x54);
	ast_gpio_write(gpio_bank, gpio_bank->timer2 * (apb_clk/1000000), 0x58);
//	printk("gpio debunce timer %x, %x, %x\n", ast_gpio_read(gpio_bank, 0x50), ast_gpio_read(gpio_bank, 0x54), ast_gpio_read(gpio_bank, 0x58));

	ast_gpio_write(gpio_bank, 0xffffffff, gpio_bank->int_type_offset);
	ast_gpio_write(gpio_bank, 0xffffffff, gpio_bank->int_type_offset + 0x04);
	ast_gpio_write(gpio_bank, 0, gpio_bank->int_type_offset + 0x08);

	rc = devm_gpiochip_add_data(&pdev->dev, &gpio->chip, gpio);
	if (rc < 0)
		return rc;

	gpio->irq = platform_get_irq(pdev, 0);
	if (gpio->irq < 0)
		return gpio->irq;

	rc = gpiochip_irqchip_add(&gpio->chip, &ast_gpio_irq_chip,
			0, handle_level_irq, IRQ_TYPE_NONE);
	if (rc) {
		dev_info(&pdev->dev, "Could not add irqchip\n");
		return rc;
	}

	gpiochip_set_chained_irqchip(&gpio->chip, &ast_gpio_irq_chip,
		gpio->irq, ast_gpio_irq_handler);

	printk("AST GPIO Driver successfully loaded \n");

	return 0;
}

static struct platform_driver ast_gpio_driver = {
	.driver		= {
		.name	= KBUILD_MODNAME,
		.of_match_table = ast_gpio_of_table,
	},
};

module_platform_driver_probe(ast_gpio_driver, ast_gpio_probe);

MODULE_AUTHOR("Ryan Chen <ryan_chen@aspeedtech.com>");
MODULE_DESCRIPTION("GPIO driver for AST processors");
MODULE_LICENSE("GPL");
