// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) ASPEED Technology Inc.
// Copyright IBM Corp.

#include <linux/of_address.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/smp.h>

#include <asm/proc-fns.h>

#define BOOT_ADDR	0x00
#define BOOT_SIG	0x04

static struct device_node *secboot_node;

static int aspeed_g6_boot_secondary(unsigned int cpu, struct task_struct *idle)
{
	void __iomem *base;

	base = of_iomap(secboot_node, 0);
	if (!base) {
		pr_err("could not map the secondary boot base!");
		return -ENODEV;
	}

	writel_relaxed(0, base + BOOT_ADDR);
	writel_relaxed(__pa_symbol(secondary_startup_arm), base + BOOT_ADDR);
	writel_relaxed((0xABBAAB00 | (cpu & 0xff)), base + BOOT_SIG);

	dsb_sev();

	iounmap(base);

	return 0;
}

static void __init aspeed_g6_smp_prepare_cpus(unsigned int max_cpus)
{
	void __iomem *base;

	secboot_node = of_find_compatible_node(NULL, NULL, "aspeed,ast2600-smpmem");
	if (!secboot_node) {
		pr_err("secboot device node found!!\n");
		return;
	}

	base = of_iomap(secboot_node, 0);
	if (!base) {
		pr_err("could not map the secondary boot base!");
		return;
	}
	__raw_writel(0xBADABABA, base + BOOT_SIG);

	iounmap(base);
}

#ifdef CONFIG_HOTPLUG_CPU
static void aspeed_g6_cpu_die(unsigned int cpu)
{
        /* Do WFI. If we wake up early, go back into WFI */
        while (1)
                cpu_do_idle();
}

static int aspeed_g6_cpu_kill(unsigned int cpu)
{
        return 1;
}
#endif

static const struct smp_operations aspeed_smp_ops __initconst = {
	.smp_prepare_cpus	= aspeed_g6_smp_prepare_cpus,
	.smp_boot_secondary	= aspeed_g6_boot_secondary,
#ifdef CONFIG_HOTPLUG_CPU
	.cpu_die		= aspeed_g6_cpu_die,
	.cpu_kill		= aspeed_g6_cpu_kill,
#endif
};

CPU_METHOD_OF_DECLARE(aspeed_smp, "aspeed,ast2600-smp", &aspeed_smp_ops);
