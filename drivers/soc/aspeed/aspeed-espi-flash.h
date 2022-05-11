/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2021 ASPEED Technology Inc.
 */
#ifndef _ASPEED_ESPI_FLASH_H_
#define _ASPEED_ESPI_FLASH_H_

#include <linux/mtd/mtd.h>

enum aspeed_espi_flash_safs_mode {
	SAFS_MODE_MIX,
	SAFS_MODE_SW,
	SAFS_MODE_HW,
	SAFS_MODES,
};

struct aspeed_espi_flash_dma {
	void *tx_virt;
	dma_addr_t tx_addr;
	void *rx_virt;
	dma_addr_t rx_addr;
};

struct aspeed_espi_flash {

	unsigned short		page_offset;	/* offset in flash address */
	unsigned int		page_size;	/* of bytes per page */

	struct mutex			lock;
	struct aspeed_espi_flash_dma	dma;

	struct mtd_info			mtd;
	struct aspeed_espi_ctrl		*ctrl;
	uint8_t				erase_mask;

	uint32_t			tx_sts;
	uint32_t			rx_sts;

	wait_queue_head_t		wq;

	spinlock_t			spinlock;
};

void aspeed_espi_flash_event(uint32_t sts, struct aspeed_espi_flash *espi_flash);
void aspeed_espi_flash_enable(struct aspeed_espi_flash *espi_flash);
void *aspeed_espi_flash_alloc(struct device *dev, struct aspeed_espi_ctrl *espi_ctrl);
void aspeed_espi_flash_free(struct device *dev, struct aspeed_espi_flash *espi_flash);

#endif
