// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2021 ASPEED Technology Inc.
 */
#include <linux/fs.h>
#include <linux/of_device.h>
#include <linux/miscdevice.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>
#include <linux/uaccess.h>
#include <linux/vmalloc.h>
#include <linux/miscdevice.h>
#include <linux/dma-mapping.h>

#include "aspeed-espi-ioc.h"
#include "aspeed-espi-ctrl.h"
#include "aspeed-espi-flash.h"

#define FLASH_MDEV_NAME	"aspeed-espi-flash"

static long aspeed_espi_flash_rx(uint8_t *cyc, uint16_t *len,
				 uint32_t *addr, uint8_t *pkt, size_t *pkt_len,
				struct aspeed_espi_flash *espi_flash)
{
	unsigned long flags;
	uint32_t reg, rx_pkt_len, addr_be;
	struct aspeed_espi_ctrl *espi_ctrl = espi_flash->ctrl;
	void *rx_virt_ptr;
	ulong to;
	int ret;

	spin_lock_irqsave(&espi_flash->spinlock, flags);

	to = msecs_to_jiffies(100);
	ret = wait_event_interruptible_lock_irq_timeout(espi_flash->wq,
						       espi_flash->rx_sts,
						       espi_flash->spinlock,
						       to);
	
	spin_unlock_irqrestore(&espi_flash->spinlock, flags);

	if (ret == -ERESTARTSYS)
		return -EINTR;
	else if (!ret)
		return -ETIMEDOUT;
	else if (espi_flash->rx_sts & ESPI_INT_STS_FLASH_RX_ABT)
		return -EFAULT;
	else
		ret = 0;

	/* common header (i.e. cycle type, tag, and length) is taken by HW */
	regmap_read(espi_ctrl->map, ESPI_FLASH_RX_CTRL, &reg);
	*cyc = (reg & ESPI_FLASH_RX_CTRL_CYC_MASK) >> ESPI_FLASH_RX_CTRL_CYC_SHIFT;
	*len = (reg & ESPI_FLASH_RX_CTRL_LEN_MASK) >> ESPI_FLASH_RX_CTRL_LEN_SHIFT;

	/*
	 * calculate the length of the rest part of the
	 * eSPI packet to be read from HW and copied to
	 * user space.
	 */
	switch (*cyc) {
	case ESPI_FLASH_READ:
	case ESPI_FLASH_WRITE:
	case ESPI_FLASH_ERASE:
		// FIXME: is this every received ?
		/* R/W/E as an address field */
		rx_virt_ptr = espi_flash->dma.rx_virt + sizeof(addr_be);
		rx_pkt_len = (*len) ? *len : ESPI_PLD_LEN_MAX;

		if (addr) {
			memcpy(&addr_be, espi_flash->dma.rx_virt, sizeof(addr_be));

			*addr = ntohl(addr_be);
		}

		break;
	case ESPI_FLASH_SUC_CMPLT_D_MIDDLE:
	case ESPI_FLASH_SUC_CMPLT_D_FIRST:
	case ESPI_FLASH_SUC_CMPLT_D_LAST:
	case ESPI_FLASH_SUC_CMPLT_D_ONLY:
		rx_pkt_len = (*len) ? *len : ESPI_PLD_LEN_MAX;
		rx_virt_ptr = espi_flash->dma.rx_virt;
		break;
	case ESPI_FLASH_SUC_CMPLT:
	case ESPI_FLASH_UNSUC_CMPLT:
	case ESPI_FLASH_UNSUC_CMPLT_ONLY:
		/* No data received */
		rx_pkt_len = 0;
		rx_virt_ptr = NULL;
		break;
	default:
		rx_pkt_len = 0;
		ret = -EFAULT;
	}
	if (pkt_len)
		*pkt_len = rx_pkt_len;

	if (rx_pkt_len && pkt_len && pkt) {
		if (*pkt_len >= rx_pkt_len) {
			memcpy(pkt, rx_virt_ptr, rx_pkt_len);
		} else {
			ret = -EINVAL;
		}
	}

	spin_lock_irqsave(&espi_flash->spinlock, flags);

	regmap_write_bits(espi_ctrl->map, ESPI_FLASH_RX_CTRL,
			  ESPI_FLASH_RX_CTRL_PEND_SERV,
			  ESPI_FLASH_RX_CTRL_PEND_SERV);

	espi_flash->rx_sts = 0;

	spin_unlock_irqrestore(&espi_flash->spinlock, flags);

	return ret;
}

static long aspeed_espi_flash_rx_get_completion(struct aspeed_espi_flash *espi_flash)
{
	uint8_t cyc;
	uint16_t len;
	int ret;
	ret = aspeed_espi_flash_rx(&cyc, &len, NULL, NULL, 0, espi_flash);
	if (ret)
		return ret;

	if (cyc != ESPI_FLASH_SUC_CMPLT)
		return -EFAULT;
	return 0;
}

static long aspeed_espi_flash_put_tx(uint8_t cyc, uint16_t len, uint32_t addr,
				     const uint8_t *pkt, size_t pkt_len,
				     struct aspeed_espi_flash *espi_flash)
{
	struct aspeed_espi_ctrl *espi_ctrl = espi_flash->ctrl;
	unsigned long flags;
	uint32_t reg, addr_be;
	ulong to;
	int ret = 0;

	regmap_read(espi_ctrl->map, ESPI_FLASH_TX_CTRL, &reg);
	if (reg & ESPI_FLASH_TX_CTRL_TRIGGER)
		return -EBUSY;

	/*
	 * common header (i.e. cycle type, tag, and length)
	 * part is written to HW registers
	 */
	addr_be = htonl(addr);
	memcpy(espi_flash->dma.tx_virt, &addr_be, sizeof(addr_be));
	if (pkt && pkt_len) {
		memcpy(espi_flash->dma.tx_virt + sizeof(addr_be), pkt, pkt_len);
	}
	dma_wmb();

	spin_lock_irqsave(&espi_flash->spinlock, flags);
	espi_flash->tx_sts = 0;
	espi_flash->rx_sts = 0;

	reg = ((cyc << ESPI_FLASH_TX_CTRL_CYC_SHIFT) & ESPI_FLASH_TX_CTRL_CYC_MASK)
		| ((len << ESPI_FLASH_TX_CTRL_LEN_SHIFT) & ESPI_FLASH_TX_CTRL_LEN_MASK)
		| ESPI_FLASH_TX_CTRL_TRIGGER;

	regmap_write(espi_ctrl->map, ESPI_FLASH_TX_CTRL, reg);

	to = msecs_to_jiffies(100);
	ret = wait_event_interruptible_lock_irq_timeout(espi_flash->wq,
						       espi_flash->tx_sts,
						       espi_flash->spinlock,
						       to);
	if (ret == -ERESTARTSYS)
		ret = -EINTR;
	else if (!ret)
		ret = -ETIMEDOUT;
	else if (espi_flash->tx_sts & (ESPI_INT_STS_FLASH_TX_ERR | ESPI_INT_STS_FLASH_TX_ABT))
		ret = -EFAULT;
	else
		ret = 0;

	spin_unlock_irqrestore(&espi_flash->spinlock, flags);
	
	return ret;
}

void aspeed_espi_flash_event(uint32_t sts, struct aspeed_espi_flash *espi_flash)
{
	unsigned long flags;

	if (!(sts & ESPI_INT_STS_FLASH_BITS))
		return;

	spin_lock_irqsave(&espi_flash->spinlock, flags);
	espi_flash->rx_sts |= sts & (ESPI_INT_STS_FLASH_RX_ABT | ESPI_INT_STS_FLASH_RX_CMPLT);
	espi_flash->tx_sts |= sts & (ESPI_INT_STS_FLASH_TX_ERR | ESPI_INT_STS_FLASH_TX_ABT | ESPI_INT_STS_FLASH_TX_CMPLT);
	spin_unlock_irqrestore(&espi_flash->spinlock, flags);
	wake_up_interruptible(&espi_flash->wq);
}

static int aspeed_espi_flash_erase(struct mtd_info *mtd, struct erase_info *instr)
{
	struct aspeed_espi_flash *espi_flash = mtd->priv;
	int ret = 0;

	/* Sanity checks */
	if ((uint32_t)instr->len % espi_flash->mtd.erasesize)
		return -EINVAL;

	if ((uint32_t)instr->addr % espi_flash->mtd.erasesize)
		return -EINVAL;

	mutex_lock(&espi_flash->lock);

	while (instr->len) {
		ret = aspeed_espi_flash_put_tx(ESPI_FLASH_ERASE, espi_flash->erase_mask, instr->addr, NULL, 0, espi_flash);
		if (ret) {
			goto unlock_mtx_n_out;
		}
		ret = aspeed_espi_flash_rx_get_completion(espi_flash);
		if (ret) {
			goto unlock_mtx_n_out;
		}

		/* TODO: Read blank check */

		instr->len -= espi_flash->mtd.erasesize;
		instr->addr += espi_flash->mtd.erasesize;
	}

unlock_mtx_n_out:
	instr->fail_addr = instr->addr;

	mutex_unlock(&espi_flash->lock);
	return ret;
}

static int aspeed_espi_flash_read(struct mtd_info *mtd, loff_t from, size_t len,
			size_t * retlen, u_char * buf)
{
	struct aspeed_espi_flash *espi_flash = mtd->priv;
	uint16_t len_pt, len_rx;
	size_t pkt_len;
	uint8_t cyc;
	int ret = 0;

	mutex_lock(&espi_flash->lock);

	while (len) {
		len_pt = (len > ESPI_PLD_LEN_MIN)? ESPI_PLD_LEN_MIN : len;

		ret = aspeed_espi_flash_put_tx(ESPI_FLASH_READ, len_pt, from, NULL, 0, espi_flash);
		if (ret) {
			goto unlock_mtx_n_out;
		}

		pkt_len = len_pt;
		len_rx = 0;
		cyc = 0;
		ret = aspeed_espi_flash_rx(&cyc, &len_rx, NULL, buf, &pkt_len, espi_flash);
		if (ret)
			goto unlock_mtx_n_out;
		
		if (cyc != ESPI_FLASH_SUC_CMPLT_D_ONLY) {
			ret = -EFAULT;
			goto unlock_mtx_n_out;
		}
		if (pkt_len != len_pt) {
			ret = -ENODATA;
			goto unlock_mtx_n_out;
		}

		len -= len_pt;
		buf += len_pt;
		from += len_pt;
		*retlen += len_pt;
	}

unlock_mtx_n_out:
	mutex_unlock(&espi_flash->lock);
	return ret;
}

static int aspeed_espi_flash_write(struct mtd_info *mtd, loff_t to, size_t len,
			size_t * retlen, const u_char * buf)
{
	struct aspeed_espi_flash *espi_flash = mtd->priv;
	uint16_t len_pt;
	int ret = 0;

	mutex_lock(&espi_flash->lock);

	while (len) {
		len_pt = (len > ESPI_PLD_LEN_MIN)? ESPI_PLD_LEN_MIN : len;

		ret = aspeed_espi_flash_put_tx(ESPI_FLASH_WRITE, len_pt, to, buf, len_pt, espi_flash);
		if (ret) {
			goto unlock_mtx_n_out;
		}
		ret = aspeed_espi_flash_rx_get_completion(espi_flash);
		if (ret) {
			goto unlock_mtx_n_out;
		}

		len -= len_pt;
		buf += len_pt;
		to += len_pt;
		*retlen += len_pt;
	}

unlock_mtx_n_out:
	mutex_unlock(&espi_flash->lock);
	return ret;
}

void aspeed_espi_flash_enable(struct aspeed_espi_flash *espi_flash)
{
	struct aspeed_espi_flash_dma *dma = &espi_flash->dma;
	struct aspeed_espi_ctrl *espi_ctrl = espi_flash->ctrl;

	regmap_update_bits(espi_ctrl->map, ESPI_CTRL,
			   ESPI_CTRL_FLASH_SW_MODE_MASK,
			   0);

	regmap_write(espi_ctrl->map, ESPI_FLASH_TX_DMA, dma->tx_addr);
	regmap_write(espi_ctrl->map, ESPI_FLASH_RX_DMA, dma->rx_addr);
	regmap_update_bits(espi_ctrl->map, ESPI_CTRL,
				ESPI_CTRL_FLASH_TX_DMA_EN | ESPI_CTRL_FLASH_RX_DMA_EN,
				ESPI_CTRL_FLASH_TX_DMA_EN | ESPI_CTRL_FLASH_RX_DMA_EN);

	regmap_write(espi_ctrl->map, ESPI_INT_STS,
		     ESPI_INT_STS_FLASH_BITS);

	regmap_update_bits(espi_ctrl->map, ESPI_INT_EN,
			   ESPI_INT_EN_FLASH_BITS,
			   ESPI_INT_EN_FLASH_BITS);

	regmap_update_bits(espi_ctrl->map, ESPI_CTRL,
			   ESPI_CTRL_FLASH_SW_RDY,
			   ESPI_CTRL_FLASH_SW_RDY);
}

void *aspeed_espi_flash_alloc(struct device *dev, struct aspeed_espi_ctrl *espi_ctrl)
{
	struct aspeed_espi_flash_dma *dma;
	struct mtd_info *mtd;
	struct aspeed_espi_flash *espi_flash;
	u32 reg;

	espi_flash = devm_kzalloc(dev, sizeof(struct aspeed_espi_flash), GFP_KERNEL);
	if (!espi_flash) 
		return ERR_PTR(-ENOMEM);

	espi_flash->ctrl = espi_ctrl;

	/* Bus lock */
	mutex_init(&espi_flash->lock);

	init_waitqueue_head(&espi_flash->wq);

	spin_lock_init(&espi_flash->spinlock);

	dma = &espi_flash->dma;

	dma->tx_virt = dma_alloc_coherent(dev, PAGE_SIZE,
						&dma->tx_addr, GFP_KERNEL);
	if (!dma->tx_virt) {
		dev_err(dev, "cannot allocate DMA TX buffer\n");
		return ERR_PTR(-ENOMEM);
	}

	dma->rx_virt = dma_alloc_coherent(dev, PAGE_SIZE,
						&dma->rx_addr, GFP_KERNEL);
	if (!dma->rx_virt) {
		dev_err(dev, "cannot allocate DMA RX buffer\n");
		return ERR_PTR(-ENOMEM);
	}

	reg = 0;
	of_property_read_u32(dev->of_node, "flash,size", &reg);  // FIXME
	
	mtd = &espi_flash->mtd;
	mtd->dev.parent = dev;
	mtd->size = reg;
	mtd->flags = MTD_CAP_NORFLASH;
	mtd->_erase = aspeed_espi_flash_erase;
	mtd->_read = aspeed_espi_flash_read;
	mtd->_write = aspeed_espi_flash_write;
	mtd->type = MTD_NORFLASH;
	mtd->name = "aspeed-espi-ctrl";

	regmap_read(espi_ctrl->map, ESPI_CH3_CAP_N_CONF, &reg);
	reg = (reg & ESPI_CH3_CAP_N_CONF_ERASE_MASK) >> ESPI_CH3_CAP_N_CONF_ERASE_SHIFT;
	espi_flash->erase_mask = reg;
	switch (reg) {
	case ESPI_CH3_CAP_N_CONF_ERASE_SIZE_4KB:
	case ESPI_CH3_CAP_N_CONF_ERASE_SIZE_4KB_64KB:
		mtd->erasesize = 0x1000;
		break;
	case ESPI_CH3_CAP_N_CONF_ERASE_SIZE_64KB:
		mtd->erasesize = 0x10000;
		break;
	case ESPI_CH3_CAP_N_CONF_ERASE_SIZE_128KB:
		mtd->erasesize = 0x20000;
		break;
	case ESPI_CH3_CAP_N_CONF_ERASE_SIZE_256KB:
		mtd->erasesize = 0x40000;
		break;
	default:
		printk(KERN_NOTICE "Unknown erase size %x\n", reg);
		return ERR_PTR(-ENODEV);
	}

	mtd->writesize = 1;
	mtd->owner = THIS_MODULE;
	mtd->priv = espi_flash;

	if (mtd_device_register(mtd, NULL, 0)) {
		printk(KERN_NOTICE "aspeed-espi-mtd: Failed to register mtd device\n");
		return ERR_PTR(-ENODEV);
	}

	aspeed_espi_flash_enable(espi_flash);

	return espi_flash;
}

void aspeed_espi_flash_free(struct device *dev, struct aspeed_espi_flash *espi_flash)
{
	struct aspeed_espi_flash_dma *dma = &espi_flash->dma;

	dma_free_coherent(dev, PAGE_SIZE, dma->tx_virt, dma->tx_addr);
	dma_free_coherent(dev, PAGE_SIZE, dma->rx_virt, dma->rx_addr);

	mtd_device_unregister(&espi_flash->mtd);
}
