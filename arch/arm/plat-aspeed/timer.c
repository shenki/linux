/*
 *  linux/arch/arm/arch-ast2000/timer.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
 
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/clocksource.h>
#include <linux/clockchips.h>

#include <linux/irq.h>
#include <asm/io.h>
#include <mach/hardware.h>
#include <mach/irqs.h>
#include <plat/ast-scu.h>

#include <plat/core.h>

#include <asm/mach/time.h>

/***************************************************************************/
#define AST_TIMER_COUNT			0x00
#define AST_TIMER_RELOAD		0x04
#define AST_TIMER_MATCH1		0x08
#define AST_TIMER_MATCH2		0x0C
#define AST_TIMER_CTRL1			0x30
#define AST_TIMER_CTRL2			0x34
#define AST_TIMER_CTRL3			0x38
/***************************************************************************/
//Timer Ctrl 

#define TIMER_CTRL_T1_ENABLE	0x1
#define TIMER_CTRL_T1_EXT_REF	(0x1 << 1) 
#define TIMER_CTRL_T2_ENABLE	(0x1 << 4) 
#define TIMER_CTRL_T2_EXT_REF	(0x1 << 5) 
#define TIMER_CTRL_T3_ENABLE	(0x1 << 8) 
#define TIMER_CTRL_T3_EXT_REF	(0x1 << 9) 


/***************************************************************************/
#define AST_TIMER_EXT_CLK_1M		(1*1000*1000)				/* 1M */
#define TIMER_RELOAD					(AST_TIMER_EXT_CLK_1M / HZ)

/* Ticks */
#define TICKS_PER_uSEC                  		1		/* AST_TIMER_EXT_CLK_1M / 10 ^ 6 */

/* How long is the timer interval? */
#define TICKS2USECS(x)				((x) / TICKS_PER_uSEC)

/***************************************************************************/
static void __iomem *ast_timer_base;

#define ast_timer_write(value, reg) \
	__raw_writel(value, ast_timer_base + (reg))
#define ast_timer_read(reg) \
	__raw_readl(ast_timer_base + (reg))
/***************************************************************************/
static int ast_set_periodic(struct clock_event_device *evt)
{
	ast_timer_write(TIMER_RELOAD - 1, AST_TIMER_RELOAD);
	ast_timer_write(TIMER_RELOAD - 1, AST_TIMER_COUNT);
	ast_timer_write(TIMER_CTRL_T1_ENABLE | TIMER_CTRL_T1_EXT_REF, AST_TIMER_CTRL1);

	return 0;
}

static int ast_set_next_event(unsigned long cycles,
				struct clock_event_device *evt)
{
	ast_timer_write(cycles, AST_TIMER_RELOAD);
	ast_timer_write(TIMER_CTRL_T1_ENABLE | ast_timer_read(AST_TIMER_CTRL1), AST_TIMER_CTRL1);

	return 0;
}

static struct clock_event_device ast_clockevent = {
	.name				= "timer0",
	.shift				= 32,
	.features				= CLOCK_EVT_FEAT_PERIODIC | CLOCK_EVT_FEAT_ONESHOT,
	.set_next_event		= ast_set_next_event,
	.set_state_periodic	= ast_set_periodic,
	.rating		= 300,
	.cpumask	= cpu_all_mask,
};

/*
 * IRQ handler for the timer
 */
static irqreturn_t ast_timer_interrupt(int irq, void *dev_id)
{
	struct clock_event_device *evt = dev_id;

	evt->event_handler(evt);

	return IRQ_HANDLED;
}


static struct irqaction ast_timer_irq = {
	.name		= "ast timer",
	.flags		= IRQF_TIMER | IRQF_TRIGGER_RISING,
	.handler		= ast_timer_interrupt,
	.dev_id		= &ast_clockevent,
};

/*
 * Set up timer interrupt, and return the current time in seconds.
 */
void __init ast_init_timer(void)
{
	ast_timer_base = ioremap(AST_TIMER_BASE, 0x100);

	ast_timer_write(0, AST_TIMER_CTRL1);
	ast_timer_write(0, AST_TIMER_CTRL2);
	ast_timer_write(0, AST_TIMER_CTRL3);

	clockevents_config_and_register(&ast_clockevent, AST_TIMER_EXT_CLK_1M,
					1, TIMER_RELOAD - 1);

	/* Enable timer interrupts */	
	setup_irq(IRQ_TIMER0, &ast_timer_irq);
}

