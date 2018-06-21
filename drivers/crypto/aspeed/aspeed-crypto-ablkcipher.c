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

//#define ASPEED_CIPHER_DEBUG

#ifdef ASPEED_CIPHER_DEBUG
//#define CIPHER_DBG(fmt, args...) printk(KERN_DEBUG "%s() " fmt, __FUNCTION__, ## args)
#define CIPHER_DBG(fmt, args...) printk("%s() " fmt, __FUNCTION__, ## args)

#else
#define CIPHER_DBG(fmt, args...)
#endif

int aspeed_crypto_ablkcipher_trigger(struct aspeed_crypto_dev *crypto_dev)
{
	struct crypto_ablkcipher *cipher = crypto_ablkcipher_reqtfm(crypto_dev->ablk_req);
	struct aspeed_cipher_ctx *ctx = crypto_ablkcipher_ctx(cipher);
	struct ablkcipher_request	*req = crypto_dev->ablk_req;
	struct scatterlist	*in_sg = req->src, *out_sg = req->dst;
	int nbytes = 0;

	if (ctx->enc_cmd & HACE_CMD_RC4) {
		*(u32 *)(crypto_dev->ctx_buf + 8) = 0x0001;
		memcpy(crypto_dev->ctx_buf + 16, ctx->key.arc4, 256);
	} else {
		if(ctx->enc_cmd & HACE_CMD_DES_SELECT) {
			if (ctx->iv) {
				memcpy(crypto_dev->ctx_buf + 8, ctx->iv, 8);
			}
			memcpy(crypto_dev->ctx_buf + 16, ctx->key.des, 24);
		} else {
			if (ctx->iv) {
				memcpy(crypto_dev->ctx_buf, ctx->iv, 16);
			}
			memcpy(crypto_dev->ctx_buf + 16, ctx->key.aes, 0xff);
		}
	}

	nbytes = sg_copy_to_buffer(in_sg, sg_nents(req->src), crypto_dev->buf_in, req->nbytes);
	CIPHER_DBG("copy nbytes %d, req->nbytes %d , nb_in_sg %d, nb_out_sg %d \n", nbytes, req->nbytes, sg_nents(req->src), sg_nents(req->dst));
	
	if (!nbytes) {
		printk("nbytes error \n");
		return -EINVAL;
	}

	if (nbytes != req->nbytes) {
		printk("~~~ EOOERR	nbytes %d , req->nbytes %d \n", nbytes, req->nbytes);
	}

#ifdef ASPEED_CRYPTO_IRQ
	crypto_dev->cmd |= HACE_CMD_ISR_EN;
	crypto_dev->irq = 0;
//	CDBUG("crypto cmd %x\n", crypto_dev->cmd);
#endif

	aspeed_crypto_write(crypto_dev, crypto_dev->dma_addr_in, ASPEED_HACE_SRC);
	aspeed_crypto_write(crypto_dev, crypto_dev->dma_addr_out, ASPEED_HACE_DEST);
	aspeed_crypto_write(crypto_dev, req->nbytes, ASPEED_HACE_DATA_LEN);
	aspeed_crypto_write(crypto_dev, ctx->enc_cmd, ASPEED_HACE_CMD);

	while (aspeed_crypto_read(crypto_dev, ASPEED_HACE_STS) & HACE_CRYPTO_BUSY);

	nbytes = sg_copy_from_buffer(out_sg, sg_nents(req->dst), crypto_dev->buf_out, req->nbytes);

	CIPHER_DBG("sg_copy_from_buffer nbytes %d req->nbytes %d, cmd %x\n",nbytes, req->nbytes, ctx->enc_cmd);

	if (!nbytes) {
		printk("nbytes %d req->nbytes %d\n", nbytes, req->nbytes);
		return -EINVAL;
	}

	return 0;
}

static u8 AESSBox[256] = {
	0x63, 0x7c, 0x77, 0x7b, 0xf2, 0x6b, 0x6f, 0xc5, 0x30, 0x01, 0x67, 0x2b, 0xfe, 0xd7, 0xab, 0x76,
	0xca, 0x82, 0xc9, 0x7d, 0xfa, 0x59, 0x47, 0xf0, 0xad, 0xd4, 0xa2, 0xaf, 0x9c, 0xa4, 0x72, 0xc0,
	0xb7, 0xfd, 0x93, 0x26, 0x36, 0x3f, 0xf7, 0xcc, 0x34, 0xa5, 0xe5, 0xf1, 0x71, 0xd8, 0x31, 0x15,
	0x04, 0xc7, 0x23, 0xc3, 0x18, 0x96, 0x05, 0x9a, 0x07, 0x12, 0x80, 0xe2, 0xeb, 0x27, 0xb2, 0x75,
	0x09, 0x83, 0x2c, 0x1a, 0x1b, 0x6e, 0x5a, 0xa0, 0x52, 0x3b, 0xd6, 0xb3, 0x29, 0xe3, 0x2f, 0x84,
	0x53, 0xd1, 0x00, 0xed, 0x20, 0xfc, 0xb1, 0x5b, 0x6a, 0xcb, 0xbe, 0x39, 0x4a, 0x4c, 0x58, 0xcf,
	0xd0, 0xef, 0xaa, 0xfb, 0x43, 0x4d, 0x33, 0x85, 0x45, 0xf9, 0x02, 0x7f, 0x50, 0x3c, 0x9f, 0xa8,
	0x51, 0xa3, 0x40, 0x8f, 0x92, 0x9d, 0x38, 0xf5, 0xbc, 0xb6, 0xda, 0x21, 0x10, 0xff, 0xf3, 0xd2,
	0xcd, 0x0c, 0x13, 0xec, 0x5f, 0x97, 0x44, 0x17, 0xc4, 0xa7, 0x7e, 0x3d, 0x64, 0x5d, 0x19, 0x73,
	0x60, 0x81, 0x4f, 0xdc, 0x22, 0x2a, 0x90, 0x88, 0x46, 0xee, 0xb8, 0x14, 0xde, 0x5e, 0x0b, 0xdb,
	0xe0, 0x32, 0x3a, 0x0a, 0x49, 0x06, 0x24, 0x5c, 0xc2, 0xd3, 0xac, 0x62, 0x91, 0x95, 0xe4, 0x79,
	0xe7, 0xc8, 0x37, 0x6d, 0x8d, 0xd5, 0x4e, 0xa9, 0x6c, 0x56, 0xf4, 0xea, 0x65, 0x7a, 0xae, 0x08,
	0xba, 0x78, 0x25, 0x2e, 0x1c, 0xa6, 0xb4, 0xc6, 0xe8, 0xdd, 0x74, 0x1f, 0x4b, 0xbd, 0x8b, 0x8a,
	0x70, 0x3e, 0xb5, 0x66, 0x48, 0x03, 0xf6, 0x0e, 0x61, 0x35, 0x57, 0xb9, 0x86, 0xc1, 0x1d, 0x9e,
	0xe1, 0xf8, 0x98, 0x11, 0x69, 0xd9, 0x8e, 0x94, 0x9b, 0x1e, 0x87, 0xe9, 0xce, 0x55, 0x28, 0xdf,
	0x8c, 0xa1, 0x89, 0x0d, 0xbf, 0xe6, 0x42, 0x68, 0x41, 0x99, 0x2d, 0x0f, 0xb0, 0x54, 0xbb, 0x16
};
static u32 aesRcon[10] = {
	0x01000000, 0x02000000, 0x04000000, 0x08000000,
	0x10000000, 0x20000000, 0x40000000, 0x80000000,
	0x1b000000, 0x36000000
};

u32 AESRotWord(u32 val)
{
	return ((val <<  8)) |
	       ((val >> 24) & 0xff) ;
};
u32 AESSubWord(u32 val)
{
	return (AESSBox[ val        & 0xff]) |
	       (AESSBox[(val >>  8) & 0xff] <<  8) |
	       (AESSBox[(val >> 16) & 0xff] << 16) |
	       (AESSBox[(val >> 24) & 0xff] << 24) ;
};
void AESKeyExpan(unsigned int bits, u32 *aeskeyin, u32 *aeskeydram)
{
	unsigned int aesNk = 0;
	unsigned int aesNr = 0;
	unsigned int i = 0;
	switch (bits) {
	case 128:
		aesNk = 4;
		aesNr = 10;
		break;
	case 192:
		aesNk = 6;
		aesNr = 12;
		break;
	case 256:
		aesNk = 8;
		aesNr = 14;
	}
	for (i = 0; i < aesNk; i++)
		aeskeydram[i] = be32_to_cpu(aeskeyin[i]);

	for (i = aesNk; i < 4 * (aesNr + 1); i++) {
		if (i % aesNk == 0)
			aeskeydram[i] = aeskeydram[i - aesNk] ^ AESSubWord(AESRotWord(aeskeydram[i - 1])) ^ aesRcon[(i / aesNk) - 1];
		else if ((aesNk > 6) && (i % aesNk == 4))
			aeskeydram[i] = aeskeydram[i - aesNk] ^ AESSubWord(aeskeydram[i - 1])                         ;
		else
			aeskeydram[i] = aeskeydram[i - aesNk] ^                       aeskeydram[i - 1]                           ;
	}

	for (i = 0; i < 4 * (aesNr + 1); i++) {
		aeskeydram[i] = be32_to_cpu(aeskeydram[i]);
	}

}

static int aspeed_rc4_crypt(struct ablkcipher_request *req, u32 cmd)
{
	struct aspeed_cipher_ctx *ctx = crypto_ablkcipher_ctx(crypto_ablkcipher_reqtfm(req));
	struct aspeed_crypto_dev *crypto_dev = ctx->crypto_dev;

	CIPHER_DBG("\n");

	cmd |= HACE_CMD_RI_WO_DATA_ENABLE | 
			HACE_CMD_CONTEXT_LOAD_ENABLE | HACE_CMD_CONTEXT_SAVE_ENABLE;

	ctx->enc_cmd = cmd;

	return aspeed_crypto_enqueue(crypto_dev, req);
}

static int aspeed_rc4_setkey(struct crypto_ablkcipher *cipher, const u8 *in_key,
			     unsigned int key_len)
{
	int i, j = 0, k = 0;

	struct aspeed_cipher_ctx *ctx = crypto_ablkcipher_ctx(cipher);

	CIPHER_DBG("keylen : %d : %s  \n", key_len, in_key);

	for (i = 0; i < 256; i++)
		ctx->key.arc4[i] = i;

	for (i = 0; i < 256; i++) {
		u32 a = ctx->key.arc4[i];
		j = (j + in_key[k] + a) & 0xff;
		ctx->key.arc4[i] = ctx->key.arc4[j];
		ctx->key.arc4[j] = a;
		if (++k >= key_len)
			k = 0;
	}

	ctx->key_len = 256;

	return 0;
}

static int aspeed_rc4_decrypt(struct ablkcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_rc4_crypt(req, HACE_CMD_DECRYPT | HACE_CMD_RC4);
}

static int aspeed_rc4_encrypt(struct ablkcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_rc4_crypt(req, HACE_CMD_ENCRYPT | HACE_CMD_RC4);
}


static int aspeed_des_crypt(struct ablkcipher_request *req, u32 cmd)
{
	struct aspeed_cipher_ctx *ctx = crypto_ablkcipher_ctx(crypto_ablkcipher_reqtfm(req));
	struct aspeed_crypto_dev *crypto_dev = ctx->crypto_dev;

	CIPHER_DBG("\n");

	cmd |= HACE_CMD_DES_SELECT | HACE_CMD_RI_WO_DATA_ENABLE |
	       HACE_CMD_CONTEXT_LOAD_ENABLE | HACE_CMD_CONTEXT_SAVE_ENABLE;

	ctx->iv = req->info;
	ctx->enc_cmd = cmd;

#if 0
	crypto_dev->ablk_req = req;
	aspeed_crypto_ablkcipher_trigger(crypto_dev);
	return 0;
#else	
	return aspeed_crypto_enqueue(crypto_dev, req);
#endif
}

static int aspeed_des_setkey(struct crypto_ablkcipher *cipher, const u8 *key,
			     unsigned int keylen)
{
	struct aspeed_cipher_ctx *ctx = crypto_ablkcipher_ctx(cipher);
	CIPHER_DBG("bits : %d : \n", keylen);

	if ((keylen != DES_KEY_SIZE) && (keylen != 2 * DES_KEY_SIZE) && (keylen != 3 * DES_KEY_SIZE)) {
		crypto_ablkcipher_set_flags(cipher, CRYPTO_TFM_RES_BAD_KEY_LEN);
		printk("keylen fail %d bits \n", keylen);
		return -EINVAL;
	}

	memcpy(ctx->key.des, key, keylen);
	ctx->key_len = keylen;

	return 0;
}

static int aspeed_tdes_ctr_decrypt(struct ablkcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_des_crypt(req, HACE_CMD_DECRYPT | HACE_CMD_CTR | HACE_CMD_TRIPLE_DES | HACE_CMD_DES);
}

static int aspeed_tdes_ctr_encrypt(struct ablkcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_des_crypt(req, HACE_CMD_ENCRYPT | HACE_CMD_CTR | HACE_CMD_TRIPLE_DES | HACE_CMD_DES);
}

static int aspeed_tdes_ofb_decrypt(struct ablkcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_des_crypt(req, HACE_CMD_DECRYPT | HACE_CMD_OFB | HACE_CMD_TRIPLE_DES | HACE_CMD_DES);
}

static int aspeed_tdes_ofb_encrypt(struct ablkcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_des_crypt(req, HACE_CMD_ENCRYPT | HACE_CMD_OFB | HACE_CMD_TRIPLE_DES | HACE_CMD_DES);
}

static int aspeed_tdes_cfb_decrypt(struct ablkcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_des_crypt(req, HACE_CMD_DECRYPT | HACE_CMD_CFB | HACE_CMD_TRIPLE_DES | HACE_CMD_DES);
}

static int aspeed_tdes_cfb_encrypt(struct ablkcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_des_crypt(req, HACE_CMD_ENCRYPT | HACE_CMD_CFB | HACE_CMD_TRIPLE_DES | HACE_CMD_DES);
}

static int aspeed_tdes_cbc_decrypt(struct ablkcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_des_crypt(req, HACE_CMD_DECRYPT | HACE_CMD_CBC | HACE_CMD_TRIPLE_DES | HACE_CMD_DES);
}

static int aspeed_tdes_cbc_encrypt(struct ablkcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_des_crypt(req, HACE_CMD_ENCRYPT | HACE_CMD_CBC | HACE_CMD_TRIPLE_DES | HACE_CMD_DES);
}

static int aspeed_tdes_ecb_decrypt(struct ablkcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_des_crypt(req, HACE_CMD_DECRYPT | HACE_CMD_ECB | HACE_CMD_TRIPLE_DES | HACE_CMD_DES);
}

static int aspeed_tdes_ecb_encrypt(struct ablkcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_des_crypt(req, HACE_CMD_ENCRYPT | HACE_CMD_ECB | HACE_CMD_TRIPLE_DES | HACE_CMD_DES);
}

static int aspeed_des_ctr_decrypt(struct ablkcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_des_crypt(req, HACE_CMD_DECRYPT | HACE_CMD_CTR | HACE_CMD_SINGLE_DES | HACE_CMD_DES);
}

static int aspeed_des_ctr_encrypt(struct ablkcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_des_crypt(req, HACE_CMD_ENCRYPT | HACE_CMD_CTR | HACE_CMD_SINGLE_DES | HACE_CMD_DES);
}

static int aspeed_des_ofb_decrypt(struct ablkcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_des_crypt(req, HACE_CMD_DECRYPT | HACE_CMD_OFB | HACE_CMD_SINGLE_DES | HACE_CMD_DES);
}

static int aspeed_des_ofb_encrypt(struct ablkcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_des_crypt(req, HACE_CMD_ENCRYPT | HACE_CMD_OFB | HACE_CMD_SINGLE_DES | HACE_CMD_DES);
}

static int aspeed_des_cfb_decrypt(struct ablkcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_des_crypt(req, HACE_CMD_DECRYPT | HACE_CMD_CFB | HACE_CMD_SINGLE_DES | HACE_CMD_DES);
}

static int aspeed_des_cfb_encrypt(struct ablkcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_des_crypt(req, HACE_CMD_ENCRYPT | HACE_CMD_CFB | HACE_CMD_SINGLE_DES | HACE_CMD_DES);
}

static int aspeed_des_cbc_decrypt(struct ablkcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_des_crypt(req, HACE_CMD_DECRYPT | HACE_CMD_CBC | HACE_CMD_SINGLE_DES | HACE_CMD_DES);
}

static int aspeed_des_cbc_encrypt(struct ablkcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_des_crypt(req, HACE_CMD_ENCRYPT | HACE_CMD_CBC | HACE_CMD_SINGLE_DES | HACE_CMD_DES);
}

static int aspeed_des_ecb_decrypt(struct ablkcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_des_crypt(req, HACE_CMD_DECRYPT | HACE_CMD_ECB | HACE_CMD_SINGLE_DES | HACE_CMD_DES);
}

static int aspeed_des_ecb_encrypt(struct ablkcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_des_crypt(req, HACE_CMD_ENCRYPT | HACE_CMD_ECB | HACE_CMD_SINGLE_DES | HACE_CMD_DES);
}

static int aspeed_aes_crypt(struct ablkcipher_request *req, u32 cmd)
{
	struct aspeed_cipher_ctx *ctx = crypto_ablkcipher_ctx(crypto_ablkcipher_reqtfm(req));
	struct aspeed_crypto_dev *crypto_dev = ctx->crypto_dev;

	ctx->iv = req->info;

	cmd |= HACE_CMD_AES_SELECT | HACE_CMD_RI_WO_DATA_ENABLE |
	       HACE_CMD_CONTEXT_LOAD_ENABLE | HACE_CMD_CONTEXT_SAVE_ENABLE;

	switch (ctx->key_len) {
	case AES_KEYSIZE_128:
		cmd |= HACE_CMD_AES128;
		break;
	case AES_KEYSIZE_192:
		cmd |= HACE_CMD_AES192;
		break;
	case AES_KEYSIZE_256:
		cmd |= HACE_CMD_AES256;
		break;
	}

	ctx->enc_cmd = cmd;

	return aspeed_crypto_enqueue(crypto_dev, req);
}

static int aspeed_aes_setkey(struct crypto_ablkcipher *cipher, const u8 *key,
			     unsigned int keylen)
{
	struct aspeed_cipher_ctx *ctx = crypto_ablkcipher_ctx(cipher);
	CIPHER_DBG("bits : %d \n", (keylen * 8));

	if (keylen != AES_KEYSIZE_128 && keylen != AES_KEYSIZE_192 &&
	    keylen != AES_KEYSIZE_256) {
		crypto_ablkcipher_set_flags(cipher, CRYPTO_TFM_RES_BAD_KEY_LEN);
		return -EINVAL;
	}

//	crypto_aes_expand_key(&ctx->key.aes, key, keylen);
	AESKeyExpan(keylen * 8, (u32 *) key, (u32 *)ctx->key.aes);
	ctx->key_len = keylen;

	return 0;
}

static int aspeed_aes_ctr_decrypt(struct ablkcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_aes_crypt(req, HACE_CMD_DECRYPT | HACE_CMD_CTR);
}

static int aspeed_aes_ctr_encrypt(struct ablkcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_aes_crypt(req, HACE_CMD_ENCRYPT | HACE_CMD_CTR);
}

static int aspeed_aes_ofb_decrypt(struct ablkcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_aes_crypt(req, HACE_CMD_DECRYPT | HACE_CMD_OFB);
}

static int aspeed_aes_ofb_encrypt(struct ablkcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_aes_crypt(req, HACE_CMD_ENCRYPT | HACE_CMD_OFB);
}

static int aspeed_aes_cfb_decrypt(struct ablkcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_aes_crypt(req, HACE_CMD_DECRYPT | HACE_CMD_CFB);
}

static int aspeed_aes_cfb_encrypt(struct ablkcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_aes_crypt(req, HACE_CMD_ENCRYPT | HACE_CMD_CFB);
}

static int aspeed_aes_ecb_decrypt(struct ablkcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_aes_crypt(req, HACE_CMD_DECRYPT | HACE_CMD_ECB);
}

static int aspeed_aes_ecb_encrypt(struct ablkcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_aes_crypt(req, HACE_CMD_ENCRYPT | HACE_CMD_ECB);
}

static int aspeed_aes_cbc_decrypt(struct ablkcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_aes_crypt(req, HACE_CMD_DECRYPT | HACE_CMD_CBC);
}

static int aspeed_aes_cbc_encrypt(struct ablkcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_aes_crypt(req, HACE_CMD_ENCRYPT | HACE_CMD_CBC);
}

static int aspeed_crypto_cra_init(struct crypto_tfm *tfm)
{
	struct aspeed_cipher_ctx *ctx = crypto_tfm_ctx(tfm);
	struct crypto_alg *alg = tfm->__crt_alg;
	struct aspeed_crypto_alg *crypto_alg;
	crypto_alg = container_of(alg, struct aspeed_crypto_alg, alg.crypto);

	ctx->crypto_dev = crypto_alg->crypto_dev;
	ctx->iv = NULL;

	return 0;
}

static void aspeed_crypto_cra_exit(struct crypto_tfm *tfm)
{
	//disable clk ??
}

struct aspeed_crypto_alg aspeed_crypto_algs[] = {
	{
		.alg.crypto = {
			.cra_name 		= "ecb(aes)",
			.cra_driver_name 	= "aspeed-ecb-aes",
			.cra_priority 		= 300,
			.cra_flags 		= CRYPTO_ALG_TYPE_ABLKCIPHER |
						CRYPTO_ALG_ASYNC,
			.cra_blocksize 	= AES_BLOCK_SIZE,
			.cra_ctxsize 		= sizeof(struct aspeed_cipher_ctx),
			.cra_alignmask	= 0x0f,
			.cra_type 		= &crypto_ablkcipher_type,
			.cra_module 		= THIS_MODULE,
			.cra_init 		= aspeed_crypto_cra_init,
			.cra_exit 		= aspeed_crypto_cra_exit,
			.cra_u.ablkcipher = {
				//no iv
				.min_keysize 	= AES_MIN_KEY_SIZE,
				.max_keysize 	= AES_MAX_KEY_SIZE,
				.setkey 		= aspeed_aes_setkey,
				.encrypt 	= aspeed_aes_ecb_encrypt,
				.decrypt	= aspeed_aes_ecb_decrypt,
			},
		},
	},
	{
		.alg.crypto = {
			.cra_name 		= "cbc(aes)",
			.cra_driver_name 	= "aspeed-cbc-aes",
			.cra_priority 		= 300,
			.cra_flags 		= CRYPTO_ALG_TYPE_ABLKCIPHER |
					CRYPTO_ALG_ASYNC,
			.cra_blocksize 	= AES_BLOCK_SIZE,
			.cra_ctxsize 		= sizeof(struct aspeed_cipher_ctx),
			.cra_alignmask	= 0xf,
			.cra_type 		= &crypto_ablkcipher_type,
			.cra_module 		= THIS_MODULE,
			.cra_init 		= aspeed_crypto_cra_init,
			.cra_exit 		= aspeed_crypto_cra_exit,
			.cra_u.ablkcipher = {
				.ivsize		= AES_BLOCK_SIZE,
				.min_keysize 	= AES_MIN_KEY_SIZE,
				.max_keysize 	= AES_MAX_KEY_SIZE,
				.setkey 		= aspeed_aes_setkey,
				.encrypt 	= aspeed_aes_cbc_encrypt,
				.decrypt	= aspeed_aes_cbc_decrypt,
			},
		},
	},
	{
		.alg.crypto = {
			.cra_name		= "cfb(aes)",
			.cra_driver_name	= "aspeed-cfb-aes",
			.cra_priority		= 300,
			.cra_flags		= CRYPTO_ALG_TYPE_ABLKCIPHER |
			CRYPTO_ALG_ASYNC,
			.cra_blocksize	= AES_BLOCK_SIZE,
			.cra_ctxsize		= sizeof(struct aspeed_cipher_ctx),
			.cra_alignmask	= 0xf,
			.cra_type		= &crypto_ablkcipher_type,
			.cra_module 		= THIS_MODULE,
			.cra_init		= aspeed_crypto_cra_init,
			.cra_exit		= aspeed_crypto_cra_exit,
			.cra_u.ablkcipher = {
				.ivsize		= AES_BLOCK_SIZE,
				.min_keysize	= AES_MIN_KEY_SIZE,
				.max_keysize	= AES_MAX_KEY_SIZE,
				.setkey 		= aspeed_aes_setkey,
				.encrypt	= aspeed_aes_cfb_encrypt,
				.decrypt	= aspeed_aes_cfb_decrypt,
			},
		},
	},
	{
		.alg.crypto = {
			.cra_name		= "ofb(aes)",
			.cra_driver_name	= "aspeed-ofb-aes",
			.cra_priority		= 300,
			.cra_flags		= CRYPTO_ALG_TYPE_ABLKCIPHER |
			CRYPTO_ALG_ASYNC,
			.cra_blocksize	= AES_BLOCK_SIZE,
			.cra_ctxsize		= sizeof(struct aspeed_cipher_ctx),
			.cra_alignmask	= 0xf,
			.cra_type		= &crypto_ablkcipher_type,
			.cra_module 		= THIS_MODULE,
			.cra_init		= aspeed_crypto_cra_init,
			.cra_exit		= aspeed_crypto_cra_exit,
			.cra_u.ablkcipher = {
				.ivsize		= AES_BLOCK_SIZE,
				.min_keysize	= AES_MIN_KEY_SIZE,
				.max_keysize	= AES_MAX_KEY_SIZE,
				.setkey 		= aspeed_aes_setkey,
				.encrypt	= aspeed_aes_ofb_encrypt,
				.decrypt	= aspeed_aes_ofb_decrypt,
			},
		},
	},
	{
		.alg.crypto = {
			.cra_name		= "ctr(aes)",
			.cra_driver_name	= "aspeed-ctr-aes",
			.cra_priority		= 300,
			.cra_flags		= CRYPTO_ALG_TYPE_ABLKCIPHER |
			CRYPTO_ALG_ASYNC,
			.cra_blocksize	= AES_BLOCK_SIZE,
			.cra_ctxsize		= sizeof(struct aspeed_cipher_ctx),
			.cra_alignmask	= 0xf,
			.cra_type		= &crypto_ablkcipher_type,
			.cra_module 		= THIS_MODULE,
			.cra_init		= aspeed_crypto_cra_init,
			.cra_exit		= aspeed_crypto_cra_exit,
			.cra_u.ablkcipher = {
				.ivsize		= AES_BLOCK_SIZE,
				.min_keysize	= AES_MIN_KEY_SIZE,
				.max_keysize	= AES_MAX_KEY_SIZE,
				.setkey 		= aspeed_aes_setkey,
				.encrypt	= aspeed_aes_ctr_encrypt,
				.decrypt	= aspeed_aes_ctr_decrypt,
			},
		},
	},
#if 0
	{
		.alg.crypto = {
			.cra_name		= "ecb(des)",
			.cra_driver_name	= "aspeed-ecb-des",
			.cra_priority		= 300,
			.cra_flags		= CRYPTO_ALG_TYPE_ABLKCIPHER |
			CRYPTO_ALG_ASYNC,
			.cra_blocksize		= DES_BLOCK_SIZE,
			.cra_ctxsize		= sizeof(struct aspeed_cipher_ctx),
			.cra_alignmask	= 0xf,
			.cra_type		= &crypto_ablkcipher_type,
			.cra_module 		= THIS_MODULE,
			.cra_init		= aspeed_crypto_cra_init,
			.cra_exit		= aspeed_crypto_cra_exit,
			.cra_u.ablkcipher = {
				//no iv
				.min_keysize	= DES_KEY_SIZE,
				.max_keysize	= DES_KEY_SIZE,
				.setkey		= aspeed_des_setkey,
				.encrypt	= aspeed_des_ecb_encrypt,
				.decrypt	= aspeed_des_ecb_decrypt,
			},
		},
	},
#endif	
	{
		.alg.crypto = {
			.cra_name		= "cbc(des)",
			.cra_driver_name	= "aspeed-cbc-des",
			.cra_priority		= 300,
			.cra_flags		= CRYPTO_ALG_TYPE_ABLKCIPHER |
			CRYPTO_ALG_ASYNC,
			.cra_blocksize		= DES_BLOCK_SIZE,
			.cra_ctxsize		= sizeof(struct aspeed_cipher_ctx),
			.cra_alignmask	= 0xf,
			.cra_type		= &crypto_ablkcipher_type,
			.cra_module 		= THIS_MODULE,
			.cra_init		= aspeed_crypto_cra_init,
			.cra_exit		= aspeed_crypto_cra_exit,
			.cra_u.ablkcipher = {
				.ivsize		= DES_BLOCK_SIZE,
				.min_keysize	= DES_KEY_SIZE,
				.max_keysize	= DES_KEY_SIZE,
				.setkey 	= aspeed_des_setkey,
				.encrypt	= aspeed_des_cbc_encrypt,
				.decrypt	= aspeed_des_cbc_decrypt,
			},
		},
	},
	{
		.alg.crypto = {
			.cra_name		= "cfb(des)",
			.cra_driver_name	= "aspeed-cfb-des",
			.cra_priority		= 300,
			.cra_flags		= CRYPTO_ALG_TYPE_ABLKCIPHER |
			CRYPTO_ALG_ASYNC,
			.cra_blocksize		= DES_BLOCK_SIZE,
			.cra_ctxsize		= sizeof(struct aspeed_cipher_ctx),
			.cra_alignmask	= 0xf,
			.cra_type		= &crypto_ablkcipher_type,
			.cra_module 		= THIS_MODULE,
			.cra_init		= aspeed_crypto_cra_init,
			.cra_exit		= aspeed_crypto_cra_exit,
			.cra_u.ablkcipher = {
				.ivsize		= DES_BLOCK_SIZE,
				.min_keysize	= DES_KEY_SIZE,
				.max_keysize	= DES_KEY_SIZE,
				.setkey 	= aspeed_des_setkey,
				.encrypt	= aspeed_des_cfb_encrypt,
				.decrypt	= aspeed_des_cfb_decrypt,
			},
		},
	},
	{
		.alg.crypto = {
			.cra_name		= "ofb(des)",
			.cra_driver_name	= "aspeed-ofb-des",
			.cra_priority		= 300,
			.cra_flags		= CRYPTO_ALG_TYPE_ABLKCIPHER |
					CRYPTO_ALG_ASYNC,
			.cra_blocksize		= DES_BLOCK_SIZE,
			.cra_ctxsize		= sizeof(struct aspeed_cipher_ctx),
			.cra_alignmask	= 0xf,
			.cra_type		= &crypto_ablkcipher_type,
			.cra_module 		= THIS_MODULE,
			.cra_init		= aspeed_crypto_cra_init,
			.cra_exit		= aspeed_crypto_cra_exit,
			.cra_u.ablkcipher = {
				.ivsize		= DES_BLOCK_SIZE,
				.min_keysize	= 2 * DES_KEY_SIZE,
				.max_keysize	= 3 * DES_KEY_SIZE,
				.setkey 	= aspeed_des_setkey,
				.encrypt	= aspeed_des_ofb_encrypt,
				.decrypt	= aspeed_des_ofb_decrypt,
			},
		},
	},
	{
		.alg.crypto = {
			.cra_name		= "ctr(des)",
			.cra_driver_name	= "aspeed-ctr-des",
			.cra_priority		= 300,
			.cra_flags		= CRYPTO_ALG_TYPE_ABLKCIPHER |
					CRYPTO_ALG_ASYNC,
			.cra_blocksize		= DES_BLOCK_SIZE,
			.cra_ctxsize		= sizeof(struct aspeed_cipher_ctx),
			.cra_alignmask	= 0xf,
			.cra_type		= &crypto_ablkcipher_type,
			.cra_module 		= THIS_MODULE,
			.cra_init		= aspeed_crypto_cra_init,
			.cra_exit		= aspeed_crypto_cra_exit,
			.cra_u.ablkcipher = {
				.ivsize		= DES_BLOCK_SIZE,
				.min_keysize	= 2 * DES_KEY_SIZE,
				.max_keysize	= 3 * DES_KEY_SIZE,
				.setkey 	= aspeed_des_setkey,
				.encrypt	= aspeed_des_ctr_encrypt,
				.decrypt	= aspeed_des_ctr_decrypt,
			},
		},
	},
	{
		.alg.crypto = {
			.cra_name		= "ecb(des3_ede)",
			.cra_driver_name	= "aspeed-ecb-tdes",
			.cra_priority		= 300,
			.cra_flags		= CRYPTO_ALG_TYPE_ABLKCIPHER |
			CRYPTO_ALG_ASYNC,
			.cra_blocksize		= DES_BLOCK_SIZE,
			.cra_ctxsize		= sizeof(struct aspeed_cipher_ctx),
			.cra_alignmask	= 0xf,
			.cra_type		= &crypto_ablkcipher_type,
			.cra_module 		= THIS_MODULE,
			.cra_init		= aspeed_crypto_cra_init,
			.cra_exit		= aspeed_crypto_cra_exit,
			.cra_u.ablkcipher = {
				.ivsize		= DES_BLOCK_SIZE,
				.min_keysize	= 2 * DES_KEY_SIZE,
				.max_keysize	= 3 * DES_KEY_SIZE,
				.setkey		= aspeed_des_setkey,
				.encrypt	= aspeed_tdes_ecb_encrypt,
				.decrypt	= aspeed_tdes_ecb_decrypt,
			},
		},
	},
	{
		.alg.crypto = {
			.cra_name		= "cbc(des3_ede)",
			.cra_driver_name	= "aspeed-cbc-tdes",
			.cra_priority		= 300,
			.cra_flags		= CRYPTO_ALG_TYPE_ABLKCIPHER |
			CRYPTO_ALG_ASYNC,
			.cra_blocksize		= DES_BLOCK_SIZE,
			.cra_ctxsize		= sizeof(struct aspeed_cipher_ctx),
			.cra_alignmask	= 0xf,
			.cra_type		= &crypto_ablkcipher_type,
			.cra_module 		= THIS_MODULE,
			.cra_init		= aspeed_crypto_cra_init,
			.cra_exit		= aspeed_crypto_cra_exit,
			.cra_u.ablkcipher = {
				.ivsize		= DES_BLOCK_SIZE,
				.min_keysize	= 2 * DES_KEY_SIZE,
				.max_keysize	= 3 * DES_KEY_SIZE,
				.setkey 	= aspeed_des_setkey,
				.encrypt	= aspeed_tdes_cbc_encrypt,
				.decrypt	= aspeed_tdes_cbc_decrypt,
			},
		},
	},
	{
		.alg.crypto = {
			.cra_name		= "cfb(des3_ede)",
			.cra_driver_name	= "aspeed-cfb-tdes",
			.cra_priority		= 300,
			.cra_flags		= CRYPTO_ALG_TYPE_ABLKCIPHER |
			CRYPTO_ALG_ASYNC,
			.cra_blocksize		= DES_BLOCK_SIZE,
			.cra_ctxsize		= sizeof(struct aspeed_cipher_ctx),
			.cra_alignmask	= 0xf,
			.cra_type		= &crypto_ablkcipher_type,
			.cra_module 		= THIS_MODULE,
			.cra_init		= aspeed_crypto_cra_init,
			.cra_exit		= aspeed_crypto_cra_exit,
			.cra_u.ablkcipher = {
				.ivsize		= DES_BLOCK_SIZE,
				.min_keysize	= 2 * DES_KEY_SIZE,
				.max_keysize	= 3 * DES_KEY_SIZE,
				.setkey 	= aspeed_des_setkey,
				.encrypt	= aspeed_tdes_cfb_encrypt,
				.decrypt	= aspeed_tdes_cfb_decrypt,
			},
		},
	},
	{
		.alg.crypto = {
			.cra_name		= "ofb(des3_ede)",
			.cra_driver_name	= "aspeed-ofb-tdes",
			.cra_priority		= 300,
			.cra_flags		= CRYPTO_ALG_TYPE_ABLKCIPHER |
			CRYPTO_ALG_ASYNC,
			.cra_blocksize		= DES_BLOCK_SIZE,
			.cra_ctxsize		= sizeof(struct aspeed_cipher_ctx),
			.cra_alignmask	= 0xf,
			.cra_type		= &crypto_ablkcipher_type,
			.cra_module 		= THIS_MODULE,
			.cra_init		= aspeed_crypto_cra_init,
			.cra_exit		= aspeed_crypto_cra_exit,
			.cra_u.ablkcipher = {
				.ivsize		= DES_BLOCK_SIZE,
				.min_keysize	= 2 * DES_KEY_SIZE,
				.max_keysize	= 3 * DES_KEY_SIZE,
				.setkey 	= aspeed_des_setkey,
				.encrypt	= aspeed_tdes_ofb_encrypt,
				.decrypt	= aspeed_tdes_ofb_decrypt,
			},
		},
	},
	{
		.alg.crypto = {
			.cra_name		= "ctr(des3_ede)",
			.cra_driver_name	= "aspeed-ctr-tdes",
			.cra_priority		= 300,
			.cra_flags		= CRYPTO_ALG_TYPE_ABLKCIPHER |
			CRYPTO_ALG_ASYNC,
			.cra_blocksize		= DES_BLOCK_SIZE,
			.cra_ctxsize		= sizeof(struct aspeed_cipher_ctx),
			.cra_alignmask	= 0xf,
			.cra_type		= &crypto_ablkcipher_type,
			.cra_module 		= THIS_MODULE,
			.cra_init		= aspeed_crypto_cra_init,
			.cra_exit		= aspeed_crypto_cra_exit,
			.cra_u.ablkcipher = {
				.ivsize		= DES_BLOCK_SIZE,
				.min_keysize	= 2 * DES_KEY_SIZE,
				.max_keysize	= 3 * DES_KEY_SIZE,
				.setkey 	= aspeed_des_setkey,
				.encrypt	= aspeed_tdes_ctr_encrypt,
				.decrypt	= aspeed_tdes_ctr_decrypt,
			},
		},
	},
	{
		.alg.crypto = {
			.cra_name		= "ecb(arc4)",
			.cra_driver_name	= "aspeed-ecb-arc4",
			.cra_priority		= 300,
			.cra_flags		= CRYPTO_ALG_TYPE_ABLKCIPHER |
									CRYPTO_ALG_ASYNC,
			.cra_blocksize		= 1,
			.cra_ctxsize		= sizeof(struct aspeed_cipher_ctx),
			.cra_alignmask	= 0xf,
			.cra_type		= &crypto_ablkcipher_type,
			.cra_module 		= THIS_MODULE,
			.cra_init		= aspeed_crypto_cra_init,
			.cra_exit		= aspeed_crypto_cra_exit,
			.cra_u.ablkcipher = {
				.min_keysize	= 1,
				.max_keysize	= 256,
				.setkey 	= aspeed_rc4_setkey,
				.encrypt	= aspeed_rc4_encrypt,
				.decrypt	= aspeed_rc4_decrypt,
			},
		},
	},
};

int aspeed_register_crypto_algs(struct aspeed_crypto_dev *crypto_dev)
{
	int i;
	int err = 0;
	for (i = 0; i < ARRAY_SIZE(aspeed_crypto_algs); i++) {
		aspeed_crypto_algs[i].crypto_dev = crypto_dev;
		err = crypto_register_alg(&aspeed_crypto_algs[i].alg.crypto);
		if (err)
			return err;
	}
	return 0;
}
