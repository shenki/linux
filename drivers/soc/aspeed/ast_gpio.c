/********************************************************************************
* File Name     : driver/char/asped/ast_gpio.c
* Author         : Ryan Chen
* Description   : AST GPIO IRQ test driver
*
* Copyright (C) 2012-2020  ASPEED Technology Inc.
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by the Free Software Foundation;
* either version 2 of the License, or (at your option) any later version.
* This program is distributed in the hope that it will be useful,  but WITHOUT ANY WARRANTY;
* without even the implied warranty of MERCHANTABILITY or
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
* You should have received a copy of the GNU General Public License
* along with this program; if not, write to the Free Software
* Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <asm/io.h>

#include <asm/uaccess.h>
#include <linux/delay.h>

#include <linux/init.h>      // initialization macros
#include <linux/module.h>    // dynamic loading of modules into the kernel
#include <linux/kernel.h>    // kernel stuff
#include <linux/gpio.h>      // GPIO functions/macros
#include <linux/interrupt.h> // interrupt functions/macros
#include <mach/gpio.h>      // GPIO functions/macros

#define GPIO_INT_NAME  "gpioB1_int"

static irqreturn_t gpio_handler(int irq, void *dev_instance)
{
	printk("GPIO IRQ .........\n");


	return (IRQ_HANDLED);
}

int __init gpio_module_init(void)
{
	int		ret;
	int 		gpio_irq;

	if (gpio_request(PIN_GPIOB1, GPIO_INT_NAME)) {
		printk("GPIO request failure: %s\n", GPIO_INT_NAME);
		return 0;
	}
	gpio_direction_input(PIN_GPIOB1);
//	gpio_set_debounce(GPIO, 1);
//	printk("gpio_export \n");
	gpio_export(PIN_GPIOB1, 1);
//	printk("gpio_to_irq \n");
	gpio_irq = gpio_to_irq(PIN_GPIOB1);        // map your GPIO to an IRQ
//	printk("gpio_to_irq %d \n", gpio_irq);
	ret = request_irq(gpio_irq,           // requested interrupt
					  (irq_handler_t) gpio_handler, // pointer to handler function
					  IRQF_TRIGGER_HIGH, // interrupt mode flag
					  "gpio_pin_irq",        // used in /proc/interrupts
					  NULL);               // the *dev_id shared interrupt lines, NULL is okay
	if (ret) {
		printk("AST Unable to request GPIO IRQ %d\n", ret);
		return ret;
	}

	return 0;       /* Return 1 to not load the module */
}

void __exit gpio_module_exit(void)
{
	return;
}

/* ------------------------- Module Information Follows -------------------- */
module_init(gpio_module_init);
module_exit(gpio_module_exit);

MODULE_AUTHOR("Ryan Chen");
MODULE_DESCRIPTION("GPIO IRQ test driver for BMC appliance");
MODULE_LICENSE("GPL");
