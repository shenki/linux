/*
 * ASPEED timer driver
 */
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/clockchips.h>
#include <linux/clocksource.h>
#include <linux/sched_clock.h>
#include <linux/clk.h>
#include <linux/slab.h>
#include <linux/bitops.h>
#include <linux/delay.h>

/* 
 * Register definitions for the timers
 */
#define AST_TIMER_COUNT			0x00
#define AST_TIMER_RELOAD		0x04
#define AST_TIMER_MATCH1		0x08
#define AST_TIMER_MATCH2		0x0C
#define AST_TIMER_CTRL1			0x30
#define AST_TIMER_CTRL2			0x34
#define AST_TIMER_CTRL3			0x38

#define TIMER1_COUNT		(0x00)
#define TIMER1_LOAD			(0x04)
#define TIMER1_MATCH1		(0x08)
#define TIMER1_MATCH2		(0x0c)
#define TIMER2_COUNT		(0x10)
#define TIMER2_LOAD			(0x14)
#define TIMER2_MATCH1		(0x18)
#define TIMER2_MATCH2		(0x1c)
#define TIMER3_COUNT		(0x20)
#define TIMER3_LOAD			(0x24)
#define TIMER3_MATCH1		(0x28)
#define TIMER3_MATCH2		(0x2c)

//Timer Ctrl 

#define TIMER_CTRL_T1_ENABLE	(0x1)
#define TIMER_CTRL_T1_EXT_REF	(0x1 << 1) 
#define TIMER_CTRL_T1_OVER_INT	(0x1 << 2) 
#define TIMER_CTRL_T2_ENABLE	(0x1 << 4) 
#define TIMER_CTRL_T2_EXT_REF	(0x1 << 5) 
#define TIMER_CTRL_T3_ENABLE	(0x1 << 8) 
#define TIMER_CTRL_T3_EXT_REF	(0x1 << 9) 


struct ast_timer {
	void __iomem *base;
	unsigned int tick_rate;
	bool count_down;
	u32 t1_enable_val;
	struct clock_event_device clkevt;
#ifdef CONFIG_ARM
	struct delay_timer delay_timer;
#endif
};

#define AST_TIMER_EXT_CLK_1M		(1*1000*1000)				/* 1M */
#define TIMER_RELOAD					(AST_TIMER_EXT_CLK_1M / HZ)

/*
 * A local singleton used by sched_clock and delay timer reads, which are
 * fast and stateless
 */
static struct ast_timer *local_tmr;

static inline struct ast_timer *to_ast_timer(struct clock_event_device *evt)
{
	return container_of(evt, struct ast_timer, clkevt);
}

static unsigned long ast_timer_read_current_timer_up(void)
{
	return readl(local_tmr->base + TIMER2_COUNT);
}

static unsigned long ast_timer_read_current_timer_down(void)
{
	return ~readl(local_tmr->base + TIMER2_COUNT);
}

static u64 notrace ast_timer_read_sched_clock_up(void)
{
	return ast_timer_read_current_timer_up();
}

static u64 notrace ast_timer_read_sched_clock_down(void)
{
	return ast_timer_read_current_timer_down();
}

static int ast_timer_set_next_event(unsigned long cycles,
				       struct clock_event_device *evt)
{
	struct ast_timer *timer = to_ast_timer(evt);
#if 0
	u32 cr;
	/* Stop */
	cr = readl(timer->base + TIMER_CR);
	cr &= ~timer->t1_enable_val;
	writel(cr, timer->base + TIMER_CR);

	/* Setup the match register forward/backward in time */
	cr = readl(timer->base + TIMER1_COUNT);
	if (timer->count_down)
		cr -= cycles;
	else
		cr += cycles;
	writel(cr, timer->base + TIMER1_MATCH1);

	/* Start */
	cr = readl(timer->base + TIMER_CR);
	cr |= timer->t1_enable_val;
	writel(cr, timer->base + TIMER_CR);
#else
	writel(cycles, timer->base + AST_TIMER_RELOAD);
	writel(TIMER_CTRL_T1_ENABLE | readl(timer->base + AST_TIMER_CTRL1), timer->base + AST_TIMER_CTRL1);

#endif
	return 0;
}

static int ast_timer_shutdown(struct clock_event_device *evt)
{
	struct ast_timer *timer = to_ast_timer(evt);
	writel(~TIMER_CTRL_T1_ENABLE & readl(timer->base + AST_TIMER_CTRL1), timer->base + AST_TIMER_CTRL1);	

	return 0;
}

#if 0
static int ast_timer_set_oneshot(struct clock_event_device *evt)
{
	struct ast_timer *timer = to_ast_timer(evt);
	u32 cr;

	/* Stop */
	cr = readl(timer->base + TIMER_CR);
	cr &= ~timer->t1_enable_val;
	writel(cr, timer->base + TIMER_CR);

	/* Setup counter start from 0 or ~0 */
	writel(0, timer->base + TIMER1_COUNT);
	if (timer->count_down)
		writel(~0, timer->base + TIMER1_LOAD);
	else
		writel(0, timer->base + TIMER1_LOAD);

	/* Enable interrupt */
	cr = readl(timer->base + TIMER_INTR_MASK);
	cr &= ~(TIMER_1_INT_OVERFLOW | TIMER_1_INT_MATCH2);
	cr |= TIMER_1_INT_MATCH1;
	writel(cr, timer->base + TIMER_INTR_MASK);

	return 0;
}
#endif

static int ast_timer_set_periodic(struct clock_event_device *evt)
{
	struct ast_timer *timer = to_ast_timer(evt);
	
	u32 period = DIV_ROUND_CLOSEST(timer->tick_rate, HZ);
	printk("period %d , timer->tick_rate %d \n",period , timer->tick_rate);
#if 0	
	u32 cr;
	printk("__FUNCTION__ = %s,  timer->count_down %d \n", __FUNCTION__, timer->count_down);
	/* Stop */
	cr = readl(timer->base + TIMER_CR);
	cr &= ~timer->t1_enable_val;
	writel(cr, timer->base + TIMER_CR);

	/* Setup timer to fire at 1/HZ intervals. */
	if (timer->count_down) {
		printk("timer->count_down period %d \n", period);
		writel(period - 1, timer->base + TIMER1_LOAD);
		writel(0, timer->base + TIMER1_MATCH1);
	} else {
		cr = 0xffffffff - (period - 1);
		printk("! timer->count_down cr %x \n", cr);
		writel(cr, timer->base + TIMER1_COUNT);
		writel(cr, timer->base + TIMER1_LOAD);

		/* Enable interrupt on overflow */
		cr = readl(timer->base + TIMER_INTR_MASK);
		cr &= ~(TIMER_1_INT_MATCH1 | TIMER_1_INT_MATCH2);
		cr |= TIMER_1_INT_OVERFLOW;
		writel(cr, timer->base + TIMER_INTR_MASK);
	}

	/* Start the timer */
	cr = readl(timer->base + TIMER_CR);
	cr |= timer->t1_enable_val;
	writel(cr, timer->base + TIMER_CR);
#else
	writel(TIMER_RELOAD, timer->base + AST_TIMER_RELOAD);
	writel(TIMER_RELOAD, timer->base + AST_TIMER_COUNT);
	writel(TIMER_CTRL_T1_ENABLE | TIMER_CTRL_T1_EXT_REF, timer->base + AST_TIMER_CTRL1);

#endif
	return 0;
}

/*
 * IRQ handler for the timer
 */
 
static irqreturn_t ast_timer_interrupt(int irq, void *dev_id)
{
	struct clock_event_device *evt = dev_id;

	evt->event_handler(evt);
	return IRQ_HANDLED;
}

static int __init ast_timer_common_init(struct device_node *np, bool is_aspeed)
{
	struct ast_timer *timer;
	int irq;
	struct clk *clk;
	int ret;
	u32 val;

#if 0
	clk = of_clk_get_by_name(np, "PCLK");
	if (IS_ERR(clk)) {
		pr_err("could not get PCLK\n");
		return PTR_ERR(clk);
	}
	
	ret = clk_prepare_enable(clk);
	if (ret) {
		pr_err("failed to enable PCLK\n");
		return ret;
	}
#endif
	timer = kzalloc(sizeof(struct ast_timer), GFP_KERNEL);
	if (!timer) {
		ret = -ENOMEM;
		goto out_disable_clock;
	}

	timer->tick_rate = AST_TIMER_EXT_CLK_1M;
//	printk("timer->tick_rate %d \n", timer->tick_rate);

	timer->base = of_iomap(np, 0);
	if (!timer->base) {
		pr_err("Can't remap registers");
		ret = -ENXIO;
		goto out_free;
	}
	/* IRQ for timer 1 */
	irq = irq_of_parse_and_map(np, 0);
	if (irq <= 0) {
		pr_err("Can't parse IRQ");
		ret = -EINVAL;
		goto out_unmap;
	}

	timer->t1_enable_val = TIMER_CTRL_T1_ENABLE |
		TIMER_CTRL_T1_OVER_INT;

	writel(0, timer->base + AST_TIMER_CTRL1);
	writel(0, timer->base + AST_TIMER_CTRL2);
	writel(0, timer->base + AST_TIMER_CTRL3);

	ret = request_irq(irq, ast_timer_interrupt, IRQF_TIMER,
			  "timer0", &timer->clkevt);
	if (ret) {
		pr_err("timer0 no IRQ\n");
		goto out_unmap;
	}

	timer->clkevt.name = "ast-timer0";
	/* Reasonably fast and accurate clock event */
	timer->clkevt.rating = 300;
	timer->clkevt.features = CLOCK_EVT_FEAT_PERIODIC |
		CLOCK_EVT_FEAT_ONESHOT;
	timer->clkevt.set_next_event = ast_timer_set_next_event;
	timer->clkevt.set_state_shutdown = ast_timer_shutdown;
	timer->clkevt.set_state_periodic = ast_timer_set_periodic;
//	timer->clkevt.set_state_oneshot = ast_timer_set_oneshot;
	timer->clkevt.tick_resume = ast_timer_shutdown;
	timer->clkevt.cpumask = cpumask_of(0);
	timer->clkevt.irq = irq;
	clockevents_config_and_register(&timer->clkevt,
					timer->tick_rate,
					1, TIMER_RELOAD);	
#if 0
	/* Also use this timer for delays */
	if (timer->count_down)
		timer->delay_timer.read_current_timer =
			ast_timer_read_current_timer_down;
	else
		timer->delay_timer.read_current_timer =
			ast_timer_read_current_timer_up;
	timer->delay_timer.freq = timer->tick_rate;
	register_current_timer_delay(&timer->delay_timer);
#endif
	return 0;

out_unmap:
	iounmap(timer->base);
out_free:
	kfree(timer);
out_disable_clock:
	clk_disable_unprepare(clk);

	return ret;
}

static __init int ast_timer_init(struct device_node *np)
{
	return ast_timer_common_init(np, true);
}

CLOCKSOURCE_OF_DECLARE(ast, "aspeed,ast-timer", ast_timer_init);
