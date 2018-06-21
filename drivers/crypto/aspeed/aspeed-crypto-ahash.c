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

//#define ASPEED_AHASH_DEBUG 

#ifdef ASPEED_AHASH_DEBUG
//#define AHASH_DBG(fmt, args...) printk(KERN_DEBUG "%s() " fmt, __FUNCTION__, ## args)
#define AHASH_DBG(fmt, args...) printk("%s() " fmt, __FUNCTION__, ## args)
#else
#define AHASH_DBG(fmt, args...)
#endif

int aspeed_crypto_ahash_trigger(struct aspeed_crypto_dev *crypto_dev)
{
	struct ahash_request *req = crypto_dev->ahash_req;
	struct aspeed_ahash_ctx *ctx = crypto_tfm_ctx(req->base.tfm);	

	AHASH_DBG("ctx->ahash_cmd : %x total %d\n", ctx->ahash_cmd, ctx->total);

#ifdef ASPEED_CRYPTO_IRQ
	crypto_dev->cmd |= HASH_CMD_INT_ENABLE;
	crypto_dev->isr = 0;
#endif

	aspeed_crypto_write(crypto_dev, crypto_dev->hash_src_dma, ASPEED_HACE_HASH_SRC);
	aspeed_crypto_write(crypto_dev, crypto_dev->hash_digst_dma, ASPEED_HACE_HASH_DIGEST_BUFF);
	aspeed_crypto_write(crypto_dev, crypto_dev->hmac_key_dma, ASPEED_HACE_HASH_KEY_BUFF);
	aspeed_crypto_write(crypto_dev, ctx->total, ASPEED_HACE_HASH_DATA_LEN);

	aspeed_crypto_write(crypto_dev, ctx->ahash_cmd, ASPEED_HACE_HASH_CMD);

	AHASH_DBG("0x20 : %x 0x24 %x, 0x28 %x 0x2c %x 0x30 %x \n", crypto_dev->hash_src_dma, crypto_dev->hash_digst_dma, crypto_dev->hmac_key_dma, ctx->total, ctx->ahash_cmd);
#if 0
	if (!(crypto_dev->isr & HACE_HASH_ISR)) {
		printk("INTR ERROR crypto_dev->isr %x \n", crypto_dev->isr);
	}
	CDBUG("irq %x\n", crypto_dev->isr);
#endif
	while (aspeed_crypto_read(crypto_dev, ASPEED_HACE_STS) & HACE_HASH_BUSY);

	memcpy(req->result, crypto_dev->hash_digst, ctx->digcnt);

	AHASH_DBG("done : copy to result \n");
	return 0;
}

static int aspeed_sha_update(struct ahash_request *req)
{
	struct crypto_ahash *tfm = crypto_ahash_reqtfm(req);
	struct aspeed_ahash_ctx *ctx = crypto_ahash_ctx(tfm);
	struct aspeed_crypto_dev *crypto_dev = ctx->crypto_dev;
	unsigned long flags;	
	int err;
	
	AHASH_DBG("ctx->total %d req->nbytes %d req->src %x crypto dev %x ctx->flags %d\n", ctx->total, req->nbytes, req->src, crypto_dev, ctx->flags);

	if (ctx->bufcnt + ctx->total < ASPEED_HASH_BUFF_SIZE) {
		scatterwalk_map_and_copy(crypto_dev->hash_src + ctx->bufcnt, req->src,
					 0, req->nbytes, 0);
		ctx->bufcnt += req->nbytes;
		ctx->total += req->nbytes;
		AHASH_DBG("xx ctx->total %d req->nbytes %d ctx->bufcnt %d\n", ctx->total, req->nbytes, ctx->bufcnt);
	} else {
		printk("aspeed_sha_update TODO xxxxx...ctx->bufcnt %d, ctx->total %d \n", ctx->bufcnt, ctx->total);
	}

	if(ctx->flags) {
		AHASH_DBG("enqueue ctx->total %d req->nbytes %d req->src %x crypto dev %x ctx->flags %d\n", ctx->total, req->nbytes, req->src, crypto_dev, ctx->flags);
		spin_lock_irqsave(&crypto_dev->lock, flags);
		err = crypto_enqueue_request(&crypto_dev->queue, &req->base);
		spin_unlock_irqrestore(&crypto_dev->lock, flags);
		
		tasklet_schedule(&crypto_dev->crypto_tasklet);
		return err;
//		return crypto_enqueue_request(&crypto_dev->queue, &req->base);
	} else {
		AHASH_DBG("done \n");
		return 0;
	}
}

static int aspeed_sha_final(struct ahash_request *req)
{
	struct crypto_ahash *tfm = crypto_ahash_reqtfm(req);
	struct aspeed_ahash_ctx *ctx = crypto_ahash_ctx(tfm);
	struct aspeed_crypto_dev *crypto_dev = ctx->crypto_dev;
	int err;
	unsigned long flags;

	AHASH_DBG("req->nbytes %d req->src %x crypto dev %x ctx->bufcnt %d, ctx->total %d\n", req->nbytes, req->src, crypto_dev, ctx->bufcnt, ctx->total);

	spin_lock_irqsave(&crypto_dev->lock, flags);
	err = crypto_enqueue_request(&crypto_dev->queue, &req->base);
	spin_unlock_irqrestore(&crypto_dev->lock, flags);

	tasklet_schedule(&crypto_dev->crypto_tasklet);

	return err;
}

static int aspeed_sha_finup(struct ahash_request *req)
{
	struct crypto_ahash *tfm = crypto_ahash_reqtfm(req);
	struct aspeed_ahash_ctx *ctx = crypto_ahash_ctx(tfm);

	int err1, err2;

	AHASH_DBG("\n");
	ctx->flags = 1;

	err1 = aspeed_sha_update(req);
	if (err1 == -EINPROGRESS || err1 == -EBUSY)
		return err1;

	/*
	 * final() has to be always called to cleanup resources
	 * even if udpate() failed, except EINPROGRESS
	 */
	err2 = aspeed_sha_final(req);

	return err1 ? : err2;
}

static int aspeed_sham_setkey(struct crypto_ahash *tfm, const u8 *key,
			      unsigned int keylen)
{
	struct aspeed_ahash_ctx *ctx = crypto_ahash_ctx(tfm);
	struct aspeed_crypto_dev *crypto_dev = ctx->crypto_dev;
	int err = 0;

//	AHASH_DBG("keylen %d crypto_dev %x %.*s \n", keylen, crypto_dev, keylen, key);

	switch (crypto_ahash_digestsize(tfm)) {
	case MD5_DIGEST_SIZE:
		ctx->ahash_cmd = HASH_CMD_MD5 | HASH_CMD_MD5_SWAP;
		ctx->digcnt = MD5_DIGEST_SIZE;
		break;
	case SHA1_DIGEST_SIZE:
		ctx->ahash_cmd = HASH_CMD_SHA1 | HASH_CMD_SHA_SWAP;
		ctx->digcnt = SHA1_DIGEST_SIZE;
		break;
	case SHA224_DIGEST_SIZE:
		ctx->ahash_cmd = HASH_CMD_SHA224 | HASH_CMD_SHA_SWAP;
		ctx->digcnt = SHA224_DIGEST_SIZE;
		break;
	case SHA256_DIGEST_SIZE:
		ctx->ahash_cmd = HASH_CMD_SHA256 | HASH_CMD_SHA_SWAP;
		ctx->digcnt = SHA256_DIGEST_SIZE;
		break;
	default:
		printk("%d not support \n", crypto_ahash_digestsize(tfm));
		return -EINVAL;
		break;
	}

	memset(crypto_dev->hash_key, 0, 64);

#if 0
	printk("key : \n");
	print_hex_dump(KERN_CONT, "", DUMP_PREFIX_OFFSET,
			16, 1,
			key, keylen, false);
	printk("\n");
#endif	
	
	if(keylen > 64) {
		AHASH_DBG("gen hash key keylen %d crypto_dev->hash_key_dma trigger  %x  cmd %x \n",keylen, crypto_dev->hash_key_dma, ctx->ahash_cmd);
		//gen hash(key)
		memcpy(crypto_dev->hash_src, key, keylen);
		aspeed_crypto_write(crypto_dev, crypto_dev->hash_src_dma, ASPEED_HACE_HASH_SRC);
		aspeed_crypto_write(crypto_dev, crypto_dev->hash_key_dma, ASPEED_HACE_HASH_DIGEST_BUFF);
		aspeed_crypto_write(crypto_dev, keylen, ASPEED_HACE_HASH_DATA_LEN);
		aspeed_crypto_write(crypto_dev, ctx->ahash_cmd, ASPEED_HACE_HASH_CMD);		
		while (aspeed_crypto_read(crypto_dev, ASPEED_HACE_STS) & HACE_HASH_BUSY);
		//for workaround for SHA224 fill 32 bytes
		if(ctx->digcnt == SHA224_DIGEST_SIZE) {
			memset(crypto_dev->hash_key + 28, 0, 4);
		}
	} else {
		AHASH_DBG("keylen %d crypto_dev->hash_key_dma trigger  %x  cmd %x \n",keylen, crypto_dev->hash_key_dma, ctx->ahash_cmd);
		memcpy(crypto_dev->hash_key, key, keylen);
	}

	ctx->ahash_cmd |= HASH_CMD_HMAC;

	aspeed_crypto_write(crypto_dev, crypto_dev->hash_key_dma, ASPEED_HACE_HASH_SRC);
	aspeed_crypto_write(crypto_dev, crypto_dev->hmac_key_dma, ASPEED_HACE_HASH_KEY_BUFF);
	aspeed_crypto_write(crypto_dev, 0x40, ASPEED_HACE_HASH_DATA_LEN);
	aspeed_crypto_write(crypto_dev, ctx->ahash_cmd | BIT(8), ASPEED_HACE_HASH_CMD);
	while (aspeed_crypto_read(crypto_dev, ASPEED_HACE_STS) & HACE_HASH_BUSY);

	return err;
}
			  
static int aspeed_ahash_init(struct ahash_request *req)
{
	struct crypto_ahash *tfm = crypto_ahash_reqtfm(req);
	struct aspeed_ahash_ctx *ctx = crypto_tfm_ctx(req->base.tfm);

	ctx->bufcnt = 0;
	ctx->total = 0;
	ctx->flags = 0;

	switch (crypto_ahash_digestsize(tfm)) {
	case MD5_DIGEST_SIZE:
		ctx->ahash_cmd |= HASH_CMD_MD5 | HASH_CMD_MD5_SWAP;
		ctx->digcnt = MD5_DIGEST_SIZE;
		break;
	case SHA1_DIGEST_SIZE:
		ctx->ahash_cmd |= HASH_CMD_SHA1 | HASH_CMD_SHA_SWAP;
		ctx->digcnt = SHA1_DIGEST_SIZE;
		break;
	case SHA224_DIGEST_SIZE:
		ctx->ahash_cmd |= HASH_CMD_SHA224 | HASH_CMD_SHA_SWAP;
		ctx->digcnt = SHA224_DIGEST_SIZE;
		break;
	case SHA256_DIGEST_SIZE:
		ctx->ahash_cmd |= HASH_CMD_SHA256 | HASH_CMD_SHA_SWAP;
		ctx->digcnt = SHA256_DIGEST_SIZE;
		break;
	default:
		printk("%d not support \n", crypto_ahash_digestsize(tfm));
		return -EINVAL;
		break;
	}

	AHASH_DBG("req->nbytes %d digisize %d ctx cmd %x\n", req->nbytes, crypto_ahash_digestsize(tfm), ctx->ahash_cmd);

	return 0;
}

static int aspeed_sha_digest(struct ahash_request *req)
{
	return aspeed_ahash_init(req) ?: aspeed_sha_finup(req);
}

static int aspeed_cra_ahash_init(struct crypto_tfm *tfm)
{
	struct aspeed_ahash_ctx *ctx = crypto_tfm_ctx(tfm);
	struct aspeed_crypto_alg *algt;
	struct ahash_alg *alg = __crypto_ahash_alg(tfm->__crt_alg);

	const char *alg_name = crypto_tfm_alg_name(tfm);

	algt = container_of(alg, struct aspeed_crypto_alg, alg.ahash);

	ctx->crypto_dev = algt->crypto_dev;
	ctx->flags = 0;
	ctx->ahash_cmd = 0;

	AHASH_DBG("%s ctx->crypto_dev %x \n", alg_name, ctx->crypto_dev);

	/* for fallback */
	ctx->fallback_tfm = crypto_alloc_ahash(alg_name, 0,
					       CRYPTO_ALG_NEED_FALLBACK);
	if (IS_ERR(ctx->fallback_tfm)) {
		dev_err(ctx->crypto_dev->dev, "Could not load fallback driver.\n");
		return PTR_ERR(ctx->fallback_tfm);
	}
#if 0
	crypto_ahash_set_reqsize(__crypto_ahash_cast(tfm),
				 sizeof(struct atmel_sha_reqctx));
#else	
	crypto_ahash_set_reqsize(__crypto_ahash_cast(tfm),
				 sizeof(struct aspeed_ahash_ctx) +
				 crypto_ahash_reqsize(ctx->fallback_tfm));
#endif
	return 0;

}

static void aspeed_cra_hash_exit(struct crypto_tfm *tfm)
{
#if 0
	struct aspeed_sham_ctx *tctx = crypto_tfm_ctx(tfm);

	AHASH_DBG("\n");

	crypto_free_shash(tctx->fallback);
	tctx->fallback = NULL;

	if (tctx->base_hash) {
		crypto_free_shash(tctx->base_hash);
	}
#else
	AHASH_DBG("\n");
	return;
#endif
}

static int aspeed_sha_export(struct ahash_request *req, void *out)
{
	struct aspeed_ahash_rctx *rctx = ahash_request_ctx(req);
	AHASH_DBG("\n");

	memcpy(out, rctx, sizeof(*rctx));
	return 0;
}

static int aspeed_sha_import(struct ahash_request *req, const void *in)
{
	struct aspeed_ahash_rctx *rctx = ahash_request_ctx(req);
	AHASH_DBG("\n");

	memcpy(rctx, in, sizeof(*rctx));
	return 0;
}

struct aspeed_crypto_alg aspeed_ahash_algs[] = {
 	{
		.alg.ahash = {
			.init		= aspeed_ahash_init,
			.update 	= aspeed_sha_update,
			.final		= aspeed_sha_final,
			.finup		= aspeed_sha_finup,
			.digest 	= aspeed_sha_digest,
			.export 	= aspeed_sha_export,
			.import 	= aspeed_sha_import,
			.halg = {
				.digestsize = MD5_DIGEST_SIZE,
				.statesize	= sizeof(struct aspeed_sham_reqctx),
				.base	= {
					.cra_name		= "md5",
					.cra_driver_name	= "aspeed-md5",
					.cra_priority		= 300,
					.cra_flags		= CRYPTO_ALG_TYPE_AHASH |
					CRYPTO_ALG_ASYNC |
					CRYPTO_ALG_NEED_FALLBACK |
					CRYPTO_ALG_KERN_DRIVER_ONLY,
					.cra_blocksize		= MD5_HMAC_BLOCK_SIZE,
					.cra_ctxsize		= sizeof(struct aspeed_sham_ctx),
					.cra_alignmask	= 0,
					.cra_module 		= THIS_MODULE,
					.cra_init			= aspeed_cra_ahash_init,
					.cra_exit			= aspeed_cra_hash_exit,
				}
			}
		},
	},
	{
		.alg.ahash = {	
			.init		= aspeed_ahash_init,
			.update 	= aspeed_sha_update,
			.final		= aspeed_sha_final,
			.finup		= aspeed_sha_finup,
			.digest 		= aspeed_sha_digest,
			.export 	= aspeed_sha_export,
			.import 	= aspeed_sha_import,
			.halg = {
				.digestsize = SHA1_DIGEST_SIZE,
				.statesize = sizeof(struct aspeed_sham_reqctx),
				.base	= {
					.cra_name		= "sha1",
					.cra_driver_name	= "aspeed-sha1",
					.cra_priority		= 300,
					.cra_flags		= CRYPTO_ALG_TYPE_AHASH |
					CRYPTO_ALG_ASYNC |
					CRYPTO_ALG_NEED_FALLBACK |
					CRYPTO_ALG_KERN_DRIVER_ONLY,
					.cra_blocksize		= SHA1_BLOCK_SIZE,
					.cra_ctxsize		= sizeof(struct aspeed_sham_ctx),
					.cra_alignmask	= 0,
					.cra_module 		= THIS_MODULE,
					.cra_init			= aspeed_cra_ahash_init,
					.cra_exit			= aspeed_cra_hash_exit,
				}
			}
		},
	},
	{
		.alg.ahash = {	
			.init		= aspeed_ahash_init,
			.update 	= aspeed_sha_update,
			.final		= aspeed_sha_final,
			.finup		= aspeed_sha_finup,
			.digest 		= aspeed_sha_digest,
			.export 	= aspeed_sha_export,
			.import 	= aspeed_sha_import,
			.halg = {
				.digestsize = SHA256_DIGEST_SIZE,
				.statesize = sizeof(struct aspeed_sham_reqctx),
				.base	= {
					.cra_name		= "sha256",
					.cra_driver_name	= "aspeed-sha256",
					.cra_priority		= 300,
					.cra_flags		= CRYPTO_ALG_TYPE_AHASH |
					CRYPTO_ALG_ASYNC |
					CRYPTO_ALG_NEED_FALLBACK |
					CRYPTO_ALG_KERN_DRIVER_ONLY,
					.cra_blocksize		= SHA256_BLOCK_SIZE,
					.cra_ctxsize		= sizeof(struct aspeed_sham_ctx),
					.cra_alignmask	= 0,
					.cra_module 		= THIS_MODULE,
					.cra_init			= aspeed_cra_ahash_init,
					.cra_exit			= aspeed_cra_hash_exit,
				}
			}
		},
	},
	{
		.alg.ahash = {	
			.init		= aspeed_ahash_init,
			.update		= aspeed_sha_update,
			.final		= aspeed_sha_final,
			.finup		= aspeed_sha_finup,
			.digest		= aspeed_sha_digest,
			.export 	= aspeed_sha_export,
			.import 	= aspeed_sha_import,
			.halg = {
				.digestsize	= SHA224_DIGEST_SIZE,
				.statesize = sizeof(struct aspeed_sham_reqctx),
				.base	= {
					.cra_name		= "sha224",
					.cra_driver_name	= "aspeed-sha224",
					.cra_priority		= 300,
					.cra_flags		= CRYPTO_ALG_TYPE_AHASH |
					CRYPTO_ALG_ASYNC |
					CRYPTO_ALG_NEED_FALLBACK |
					CRYPTO_ALG_KERN_DRIVER_ONLY,
					.cra_blocksize		= SHA224_BLOCK_SIZE,
					.cra_ctxsize		= sizeof(struct aspeed_sham_ctx),
					.cra_alignmask	= 0,
					.cra_module		= THIS_MODULE,
					.cra_init			= aspeed_cra_ahash_init,
					.cra_exit			= aspeed_cra_hash_exit,
				}
			}
		},
	},
 	{
		.alg.ahash = {	
			.init		= aspeed_ahash_init,
			.update		= aspeed_sha_update,
			.final		= aspeed_sha_final,
			.finup		= aspeed_sha_finup,
			.digest		= aspeed_sha_digest,
			.setkey		= aspeed_sham_setkey,
			.export 	= aspeed_sha_export,
			.import 	= aspeed_sha_import,			
			.halg = {
				.digestsize	= MD5_DIGEST_SIZE,
				.statesize = sizeof(struct aspeed_sham_reqctx),
				.base	= {
					.cra_name		= "hmac(md5)",
					.cra_driver_name	= "aspeed-hmac-md5",
					.cra_priority		= 300,
					.cra_flags		= CRYPTO_ALG_TYPE_AHASH |
					CRYPTO_ALG_ASYNC |
					CRYPTO_ALG_NEED_FALLBACK |
					CRYPTO_ALG_KERN_DRIVER_ONLY,
					.cra_blocksize		= MD5_HMAC_BLOCK_SIZE,
					.cra_ctxsize		= sizeof(struct aspeed_sham_ctx),
					.cra_alignmask	= 0,
					.cra_module		= THIS_MODULE,
					.cra_init		= aspeed_cra_ahash_init,
					.cra_exit		= aspeed_cra_hash_exit,
				}
			}
		},
	},
	{
		.alg.ahash = {	
			.init		= aspeed_ahash_init,
			.update		= aspeed_sha_update,
			.final		= aspeed_sha_final,
			.finup		= aspeed_sha_finup,
			.digest		= aspeed_sha_digest,
			.setkey		= aspeed_sham_setkey,
			.export 	= aspeed_sha_export,
			.import 	= aspeed_sha_import,			
			.halg = {
				.digestsize	= SHA1_DIGEST_SIZE,
				.statesize = sizeof(struct aspeed_sham_reqctx),
				.base	= {
					.cra_name		= "hmac(sha1)",
					.cra_driver_name	= "aspeed-hmac-sha1",
					.cra_priority		= 300,
					.cra_flags		= CRYPTO_ALG_TYPE_AHASH |
					CRYPTO_ALG_ASYNC |
					CRYPTO_ALG_NEED_FALLBACK |
					CRYPTO_ALG_KERN_DRIVER_ONLY,
					.cra_blocksize		= SHA1_BLOCK_SIZE,
					.cra_ctxsize		= sizeof(struct aspeed_sham_ctx),
					.cra_alignmask	= 0,
					.cra_module		= THIS_MODULE,
					.cra_init		= aspeed_cra_ahash_init,
					.cra_exit		= aspeed_cra_hash_exit,
				}
			}
		},
	},
 	{
		.alg.ahash = {	
			.init		= aspeed_ahash_init,
			.update 	= aspeed_sha_update,
			.final		= aspeed_sha_final,
			.finup		= aspeed_sha_finup,
			.digest 	= aspeed_sha_digest,
			.setkey 	= aspeed_sham_setkey,
			.export 	= aspeed_sha_export,
			.import 	= aspeed_sha_import,
			.halg = {
				.digestsize	= SHA224_DIGEST_SIZE,
				.statesize = sizeof(struct aspeed_sham_reqctx),
				.base	= {
					.cra_name		= "hmac(sha224)",
					.cra_driver_name	= "aspeed-hmac-sha224",
					.cra_priority		= 300,
					.cra_flags		= CRYPTO_ALG_TYPE_AHASH |
					CRYPTO_ALG_ASYNC |
					CRYPTO_ALG_NEED_FALLBACK |
					CRYPTO_ALG_KERN_DRIVER_ONLY,
					.cra_blocksize		= SHA224_BLOCK_SIZE,
					.cra_ctxsize		= sizeof(struct aspeed_sham_ctx),
					.cra_alignmask	= 0,
					.cra_module 	= THIS_MODULE,
					.cra_init		= aspeed_cra_ahash_init,
					.cra_exit		= aspeed_cra_hash_exit,
				}
			}
		},
	},
	{
		.alg.ahash = {	
			.init			= aspeed_ahash_init,
			.update 		= aspeed_sha_update,
			.final		= aspeed_sha_final,
			.finup		= aspeed_sha_finup,
			.digest 		= aspeed_sha_digest,
			.setkey 		= aspeed_sham_setkey,
			.export 	= aspeed_sha_export,
			.import 	= aspeed_sha_import,
			.halg = {
				.digestsize	= SHA256_DIGEST_SIZE,
				.statesize = sizeof(struct aspeed_sham_reqctx),
				.base	= {
					.cra_name		= "hmac(sha256)",
					.cra_driver_name	= "aspeed-hmac-sha256",
					.cra_priority		= 300,
					.cra_flags		= CRYPTO_ALG_TYPE_AHASH |
					CRYPTO_ALG_ASYNC |
					CRYPTO_ALG_NEED_FALLBACK |
					CRYPTO_ALG_KERN_DRIVER_ONLY,
					.cra_blocksize		= SHA256_BLOCK_SIZE,
					.cra_ctxsize		= sizeof(struct aspeed_sham_ctx),
					.cra_alignmask	= 0,
					.cra_module 	= THIS_MODULE,
					.cra_init		= aspeed_cra_ahash_init,
					.cra_exit		= aspeed_cra_hash_exit,
				}
			}
		},
	},
};

int aspeed_register_ahash_algs(struct aspeed_crypto_dev *crypto_dev)
{
	int i;
	int err = 0;

	for (i = 0; i < ARRAY_SIZE(aspeed_ahash_algs); i++) {
		aspeed_ahash_algs[i].crypto_dev = crypto_dev;
		err = crypto_register_ahash(&aspeed_ahash_algs[i].alg.ahash);
		if (err)
			return err;
	}
	return 0;
}
