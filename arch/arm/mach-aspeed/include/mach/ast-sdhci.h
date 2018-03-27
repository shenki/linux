/********************************************************************************
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

*   History      :
*      1. 2017/08/03 Ryan Chen create this file
*
********************************************************************************/

#ifndef __AST_SDHCI_H_INCLUDED
#define __AST_SDHCI_H_INCLUDED

#include <linux/clk.h>
#include <linux/reset.h>

struct ast_sdhci_irq {
	void __iomem	*regs;
	int			parent_irq;
	int			slot_num;
	
	struct irq_domain	*irq_domain;
	struct reset_control *reset;
	struct clk 			*clk;
	u32				sd_clk;	
};

extern void ast_sd_set_8bit_mode(struct ast_sdhci_irq *sdhci_irq, u8 mode);
#endif

