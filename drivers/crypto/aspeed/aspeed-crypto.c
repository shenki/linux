/*
 * Crypto driver for the Aspeed SoC
 *
 * Copyright (C) ASPEED Technology Inc.
 * Ryan Chen <ryan_chen@aspeedtech.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include "aspeed-crypto.h"

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/clk.h>
#include <linux/crypto.h>
#include <linux/reset.h>

#include <crypto/aes.h>
#include <crypto/algapi.h>
#include <crypto/authenc.h>
#include <crypto/des.h>
#include <crypto/md5.h>
#include <crypto/sha.h>
#include <crypto/scatterwalk.h>
#include <crypto/algapi.h>
#include <crypto/aes.h>
#include <crypto/hash.h>
#include <crypto/md5.h>
#include <crypto/internal/hash.h>
#include <crypto/internal/skcipher.h>

#include <linux/completion.h>
#include <linux/clk.h>
#include <linux/crypto.h>
#include <linux/cryptohash.h>
#include <linux/delay.h>
#include <linux/scatterlist.h>

#include <linux/dma-mapping.h>
#include <linux/dmapool.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/rtnetlink.h>
#include <linux/scatterlist.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/timer.h>
#include "aspeed-crypto.h"

//#define ASPEED_CRYPTO_DEBUG

#ifdef ASPEED_CRYPTO_DEBUG
//#define CRYPTO_DBUG(fmt, args...) printk(KERN_DEBUG "%s() " fmt, __FUNCTION__, ## args)
#define CRYPTO_DBUG(fmt, args...) printk("%s() " fmt, __FUNCTION__, ## args)
#else
#define CRYPTO_DBUG(fmt, args...)
#endif

/*************************************************************************************/
int aspeed_crypto_enqueue(struct aspeed_crypto_dev *crypto_dev,
				      struct ablkcipher_request *req)
{
	unsigned long flags;
	int err;

	spin_lock_irqsave(&crypto_dev->lock, flags);
	err = ablkcipher_enqueue_request(&crypto_dev->queue, req);
	spin_unlock_irqrestore(&crypto_dev->lock, flags);
	tasklet_schedule(&crypto_dev->crypto_tasklet);

	return err;
}

static void aspeed_crypto_tasklet(unsigned long data)
{
	struct aspeed_crypto_dev *crypto_dev = (struct aspeed_crypto_dev *)data;
	struct crypto_async_request *async_req, *backlog;
	unsigned long flags;
	int err = 0;

	CRYPTO_DBUG("\n");

	spin_lock_irqsave(&crypto_dev->lock, flags);
	backlog   = crypto_get_backlog(&crypto_dev->queue);
	async_req = crypto_dequeue_request(&crypto_dev->queue);
	spin_unlock_irqrestore(&crypto_dev->lock, flags);
	if (!async_req) {
		dev_err(crypto_dev->dev, "async_req is NULL !!\n");
		return;
	}
	if (backlog) {
		backlog->complete(backlog, -EINPROGRESS);
		backlog = NULL;
	}

	if (crypto_tfm_alg_type(async_req->tfm) == CRYPTO_ALG_TYPE_ABLKCIPHER) {
		CRYPTO_DBUG("ablkcipher_request_cast \n");
		crypto_dev->ablk_req = ablkcipher_request_cast(async_req);
#if 1		
		err = aspeed_crypto_ablkcipher_trigger(crypto_dev);
		crypto_dev->ablk_req->base.complete(&crypto_dev->ablk_req->base, err);
#else
		if(aspeed_crypto_ablkcipher_trigger(crypto_dev))
			crypto_dev->ablk_req->base.complete(&crypto_dev->ablk_req->base, 0);
#endif		
	} else {
		crypto_dev->ahash_req = ahash_request_cast(async_req);
		CRYPTO_DBUG("ahash_request_cast \n");
		err = aspeed_crypto_ahash_trigger(crypto_dev);
		crypto_dev->ahash_req->base.complete(&crypto_dev->ahash_req->base, err);
	}

}

static irqreturn_t aspeed_crypto_irq(int irq, void *dev)
{
	struct aspeed_crypto_dev *crypto_dev = (struct aspeed_crypto_dev *)dev;
	u32 sts = aspeed_crypto_read(crypto_dev, ASPEED_HACE_STS);

	CRYPTO_DBUG("aspeed_crypto_irq sts %x xxxxxxxxxx\n", sts);

	aspeed_crypto_write(crypto_dev, sts, ASPEED_HACE_STS);
	return IRQ_HANDLED;
}

static int aspeed_crypto_register(struct aspeed_crypto_dev *crypto_dev)
{
	aspeed_register_crypto_algs(crypto_dev);
	aspeed_register_ahash_algs(crypto_dev);
	return 0;
}

static void aspeed_crypto_unregister(void)
{
#if 0
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(aspeed_cipher_algs); i++) {
		if (aspeed_cipher_algs[i]->type == ALG_TYPE_CIPHER)
			crypto_unregister_alg(&aspeed_cipher_algs[i]->alg.crypto);
		else
			crypto_unregister_ahash(&aspeed_cipher_algs[i]->alg.hash);
	}
#endif	
}

static int aspeed_crypto_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct resource *res;
	int err;
	struct aspeed_crypto_dev *crypto_dev;

	crypto_dev = devm_kzalloc(&pdev->dev, sizeof(struct aspeed_crypto_dev), GFP_KERNEL);
	if (!crypto_dev) {
		dev_err(dev, "unable to alloc data struct.\n");
		return -ENOMEM;
	}

	crypto_dev->dev = dev;

	platform_set_drvdata(pdev, crypto_dev);

	spin_lock_init(&crypto_dev->lock);

	tasklet_init(&crypto_dev->crypto_tasklet, 
				aspeed_crypto_tasklet, (unsigned long)crypto_dev);
	crypto_init_queue(&crypto_dev->queue, 50);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(dev, "no MEM resource info\n");
		err = -ENODEV;
	}
	crypto_dev->regs = devm_ioremap_resource(&pdev->dev, res);
	if (!(crypto_dev->regs)) {
		dev_err(dev, "can't ioremap\n");
		return -ENOMEM;
	}
	crypto_dev->irq = platform_get_irq(pdev, 0);
	if (!crypto_dev->irq) {
		dev_err(&pdev->dev, "no memory/irq resource for crypto_dev\n");
		return -ENXIO;
	}

	crypto_dev->yclk = devm_clk_get(&pdev->dev, "yclk");
	if (IS_ERR(crypto_dev->yclk)) {
		dev_err(&pdev->dev, "no yclk clock defined\n");
		return -ENODEV;
	}

	clk_prepare_enable(crypto_dev->yclk);

	if (devm_request_irq(&pdev->dev, crypto_dev->irq, aspeed_crypto_irq, 0, dev_name(&pdev->dev), crypto_dev)) {
		dev_err(dev, "unable to request aes irq.\n");
		return -EBUSY;
	}

	// 8-byte aligned
	crypto_dev->ctx_buf = dma_alloc_coherent(&pdev->dev,
			      0xa000,
			      &crypto_dev->ctx_dma_addr, GFP_KERNEL);

	if (! crypto_dev->ctx_buf) {
		printk("error buff allocation\n");
		return -ENOMEM;
	}
	crypto_dev->buf_size = 0x1000;
	crypto_dev->buf_in = crypto_dev->ctx_buf + 0x1000;
	crypto_dev->dma_addr_in = crypto_dev->ctx_dma_addr + 0x1000;

	crypto_dev->buf_out = crypto_dev->buf_in + 0x1000;
	crypto_dev->dma_addr_out = crypto_dev->dma_addr_in + 0x1000;

	crypto_dev->hash_key = crypto_dev->buf_out + 0x1000;
	crypto_dev->hash_key_dma = crypto_dev->dma_addr_out + 0x1000;

	crypto_dev->hmac_key = crypto_dev->hash_key + 0x1000;
	crypto_dev->hmac_key_dma = crypto_dev->hash_key_dma + 0x1000;

	crypto_dev->hash_src = crypto_dev->hmac_key + 0x1000;
	crypto_dev->hash_src_dma = crypto_dev->hmac_key_dma + 0x1000;

	crypto_dev->hash_digst = crypto_dev->hash_src + ASPEED_HASH_BUFF_SIZE;
	crypto_dev->hash_digst_dma = crypto_dev->hash_src_dma + ASPEED_HASH_BUFF_SIZE;

	CRYPTO_DBUG("Crypto ctx %x , in : %x, out: %x\n", crypto_dev->ctx_dma_addr, crypto_dev->dma_addr_in, crypto_dev->dma_addr_out);

	CRYPTO_DBUG("Hash key %x , src : %x, digst: %x\n", crypto_dev->hash_key_dma, crypto_dev->hash_src_dma, crypto_dev->hash_digst_dma);

	aspeed_crypto_write(crypto_dev, crypto_dev->ctx_dma_addr, ASPEED_HACE_CONTEXT);

	err = aspeed_crypto_register(crypto_dev);
	if (err) {
		dev_err(dev, "err in register alg");
		return err;
	}

	printk("ASPEED Crypto Accelerator successfully registered \n");

	return 0;
}

static int aspeed_crypto_remove(struct platform_device *pdev)
{
	struct aspeed_crypto_dev *crypto_dev = platform_get_drvdata(pdev);

	//aspeed_crypto_unregister();
	tasklet_kill(&crypto_dev->crypto_tasklet);
	return 0;
}

#ifdef CONFIG_PM
static int aspeed_crypto_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct aspeed_crypto_dev *crypto_dev = platform_get_drvdata(pdev);

	/*
	 * We only support standby mode. All we have to do is gate the clock to
	 * the spacc. The hardware will preserve state until we turn it back
	 * on again.
	 */
//	clk_disable(crypto_dev->clk);

	return 0;
}

static int aspeed_crypto_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct aspeed_crypto_dev *crypto_dev = platform_get_drvdata(pdev);
	return 0;
//	return clk_enable(crypto_dev->clk);
}

#endif /* CONFIG_PM */



static const struct of_device_id aspeed_crypto_of_matches[] = {
	{ .compatible = "aspeed,ast2500-crypto", },
	{ .compatible = "aspeed,ast2400-crypto", },
	{},
};

MODULE_DEVICE_TABLE(of, aspeed_crypto_of_matches);

static struct platform_driver aspeed_crypto_driver = {
	.probe 		= aspeed_crypto_probe,
	.remove		= aspeed_crypto_remove,
#ifdef CONFIG_PM
	.suspend	= aspeed_crypto_suspend,
	.resume 	= aspeed_crypto_resume,
#endif
	.driver         = {
		.name   = KBUILD_MODNAME,
		.of_match_table = aspeed_crypto_of_matches,
	},
};

module_platform_driver(aspeed_crypto_driver);

MODULE_AUTHOR("Ryan Chen <ryan_chen@aspeedtech.com>");
MODULE_DESCRIPTION("ASPEED Crypto driver");
MODULE_LICENSE("GPL2");
