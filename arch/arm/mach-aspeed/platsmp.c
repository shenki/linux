// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) ASPEED Technology Inc.
// Copyright IBM Corp.

#include <linux/of_address.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/smp.h>

#define BOOT_ADDR	0x00
#define BOOT_SIG	0x04

void __iomem *secboot_base;

static int aspeed_g6_boot_secondary(unsigned int cpu, struct task_struct *idle)
{
	writel_relaxed(0, secboot_base + BOOT_ADDR);
	writel_relaxed(__pa_symbol(secondary_startup_arm),
			secboot_base + BOOT_ADDR);
	writel_relaxed((0xABBAAB00 | (cpu & 0xff)), secboot_base + BOOT_SIG);

	dsb_sev();

	iounmap(secboot_base);

	return 0;
}

static void __init aspeed_g6_smp_prepare_cpus(unsigned int max_cpus)
{
	struct device_node *secboot_node;

	secboot_node = of_find_compatible_node(NULL, NULL, "aspeed,ast2600-smpmem");
	if (!secboot_node) {
		pr_err("secboot device node found!!\n");
		return;
	}

	secboot_base = of_iomap(secboot_node, 0);
	if (!secboot_base) {
		pr_err("could not map the secondary boot base!");
		return;
	}
	__raw_writel(0xBADABABA, secboot_base + BOOT_SIG);
}

static const struct smp_operations aspeed_smp_ops __initconst = {
	.smp_prepare_cpus	= aspeed_g6_smp_prepare_cpus,
	.smp_boot_secondary	= aspeed_g6_boot_secondary,
};

CPU_METHOD_OF_DECLARE(aspeed_smp, "aspeed,ast2600-smp", &aspeed_smp_ops);
