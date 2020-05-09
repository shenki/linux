/*
 * Microwatt FPGA-based SoC platform setup code.
 *
 * Copyright 2020 Paul Mackerras (paulus@ozlabs.org), IBM Corp.
 */

#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/stddef.h>
#include <linux/init.h>
#include <linux/of.h>
#include <asm/machdep.h>
#include <asm/udbg.h>
#include <asm/reg.h>
#include <asm/time.h>

static u64 potato_uart_base;

#define PROC_FREQ 100000000
#define UART_FREQ 115200
#define UART_BASE 0xc0002000

#define POTATO_CONSOLE_TX		0x00
#define POTATO_CONSOLE_RX		0x08
#define POTATO_CONSOLE_STATUS		0x10
#define   POTATO_CONSOLE_STATUS_RX_EMPTY		0x01
#define   POTATO_CONSOLE_STATUS_TX_EMPTY		0x02
#define   POTATO_CONSOLE_STATUS_RX_FULL			0x04
#define   POTATO_CONSOLE_STATUS_TX_FULL			0x08
#define POTATO_CONSOLE_CLOCK_DIV	0x18
#define POTATO_CONSOLE_IRQ_EN		0x20

static u64 potato_uart_reg_read(int offset)
{
	u64 val, msr;

	msr = mfmsr();
	__asm__ volatile("mtmsrd %3,0; ldcix %0,%1,%2; mtmsrd %4,0"
			 : "=r" (val) : "b" (potato_uart_base), "r" (offset),
			   "r" (msr & ~MSR_DR), "r" (msr));

	return val;
}

static void potato_uart_reg_write(int offset, u64 val)
{
	u64 msr;

	msr = mfmsr();
	__asm__ volatile("mtmsrd %3,0; stdcix %0,%1,%2; mtmsrd %4,0"
			 : : "r" (val), "b" (potato_uart_base), "r" (offset),
			   "r" (msr & ~MSR_DR), "r" (msr));
}

static int potato_uart_rx_empty(void)
{
	u64 val;

	val = potato_uart_reg_read(POTATO_CONSOLE_STATUS);

	if (val & POTATO_CONSOLE_STATUS_RX_EMPTY)
		return 1;

	return 0;
}

static int potato_uart_tx_full(void)
{
	u64 val;

	val = potato_uart_reg_read(POTATO_CONSOLE_STATUS);

	if (val & POTATO_CONSOLE_STATUS_TX_FULL)
		return 1;

	return 0;
}

static int potato_uart_read(void)
{
	while (potato_uart_rx_empty())
		;
	return potato_uart_reg_read(POTATO_CONSOLE_RX);
}

static int potato_uart_read_poll(void)
{
	if (potato_uart_rx_empty())
		return -1;
	return potato_uart_reg_read(POTATO_CONSOLE_RX);
}

static void potato_uart_write(char c)
{
	if (c == '\n')
		potato_uart_write('\r');
	while (potato_uart_tx_full())
		;
	potato_uart_reg_write(POTATO_CONSOLE_TX, c);
}

static unsigned long potato_uart_divisor(unsigned long proc_freq, unsigned long uart_freq)
{
	return proc_freq / (uart_freq * 16) - 1;
}

void potato_uart_init(void)
{
	potato_uart_base = UART_BASE;

	potato_uart_reg_write(POTATO_CONSOLE_CLOCK_DIV, potato_uart_divisor(PROC_FREQ, UART_FREQ));
}

void udbg_init_debug_microwatt(void)
{
	potato_uart_init();
	udbg_putc = potato_uart_write;
	udbg_getc = potato_uart_read;
	udbg_getc_poll = potato_uart_read_poll;
}

static void __init microwatt_calibrate_decr(void)
{
	ppc_tb_freq = 100000000;
	ppc_proc_freq = 100000000;
}

static void __init microwatt_setup_arch(void)
{
}

static void __init microwatt_init_IRQ(void)
{
}

static int __init microwatt_probe(void)
{
	return of_machine_is_compatible("microwatt-soc");
}

define_machine(microwatt) {
	.name			= "microwatt",
	.probe			= microwatt_probe,
	.setup_arch		= microwatt_setup_arch,
	.init_IRQ		= microwatt_init_IRQ,
	.progress		= udbg_progress,
	.calibrate_decr		= microwatt_calibrate_decr,
};
