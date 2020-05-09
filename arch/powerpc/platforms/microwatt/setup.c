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
#include <asm/time.h>

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
	.calibrate_decr		= microwatt_calibrate_decr,
};
