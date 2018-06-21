/* SPDX-License-Identifier: GPL-2.0 */
#ifndef LINUX_MMC_SDHCI_ASPEED_DATA_H
#define LINUX_MMC_SDHCI_ASPEED_DATA_H

#include <linux/clk.h>
#include <linux/reset.h>

struct aspeed_sdhci_irq {
	void __iomem	*regs;
	int		parent_irq;
	struct irq_domain	*irq_domain;
	struct reset_control	*reset;
	struct clk 	*clk;
	u32		sd_clk;
};

extern void aspeed_sdhci_set_8bit_mode(struct aspeed_sdhci_irq *sdhci_irq, u8 mode);
#endif

