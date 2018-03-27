/*
 * ast-crypto.c - Crypto driver for the Aspeed SoC
 *
 * Copyright (C) ASPEED Technology Inc.
 * Ryan Chen <ryan_chen@aspeedtech.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
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
#include <linux/reset.h>
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

#define AST_CRYPTO_IRQ
//#define AST_CRYPTO_DEBUG
//#define AST_HASH_DEBUG

#define AST_AES_QUEUE_LENGTH	1
#define AST_HASH_BUFF_SIZE 	8192

//#define CDBUG(fmt, args...) printk(KERN_DEBUG "%s() " fmt, __FUNCTION__, ## args)

#ifdef AST_CRYPTO_DEBUG
#define CRYPTO_DBUG(fmt, args...) printk(KERN_DEBUG "%s() " fmt, __FUNCTION__, ## args)
#else
#define CRYPTO_DBUG(fmt, args...)
#endif

#ifdef AST_HASH_DEBUG
#define HASH_DBUG(fmt, args...) printk(KERN_DEBUG "%s() " fmt, __FUNCTION__, ## args)
#else
#define HASH_DBUG(fmt, args...)
#endif

/*************************************************************************************/
#define AST_HACE_SRC					0x00
#define AST_HACE_DEST					0x04
#define AST_HACE_CONTEXT				0x08
#define AST_HACE_DATA_LEN				0x0C
#define AST_HACE_CMD					0x10
#define AST_HACE_TAG					0x18
#define AST_HACE_STS					0x1C
#define AST_HACE_HASH_SRC				0x20
#define AST_HACE_HASH_DIGEST_BUFF	0x24
#define AST_HACE_HASH_KEY_BUFF		0x28
#define AST_HACE_HASH_DATA_LEN		0x2C
#define AST_HACE_HASH_CMD				0x30

#define AST_HACE_RSA_MD_EXP_BIT		0x40
#define AST_HACE_RSA_CMD				0x4C

#define AST_HACE_CMD_QUEUE			0x50
#define AST_HACE_CMD_QUEUE_EP		0x54
#define AST_HACE_CMD_QUEUE_WP		0x58
#define AST_HACE_CMD_QUEUE_RP		0x5C
#define AST_HACE_ENG_FEATURE			0x60

/*************************************************************************************/

#define HACE_HASH_BUSY				(0x1)
#define HACE_CRYPTO_BUSY				(0x1 << 1)
#define HACE_RSA_BUSY					(0x1 << 2)

#define HACE_HASH_ISR					(0x1 << 9)
#define HACE_CRYPTO_ISR				(0x1 << 12)
#define HACE_RSA_ISR					(0x1 << 13)

/* 	AST_HACE_CMD					0x10		*/
#define HACE_CMD_SINGLE_DES				(0)
#define HACE_CMD_TRIPLE_DES				(0x1 << 17)

#define HACE_CMD_AES_SELECT				(0)
#define HACE_CMD_DES_SELECT				(0x1 << 16)

#define HACE_CMD_INT_ENABLE				(0x1 << 12)
#define HACE_CMD_INT_DISABLE				(0)

#define HACE_CMD_RI_WO_DATA_ENABLE		(0)
#define HACE_CMD_RI_WO_DATA_DISABLE		(0x1 << 11)

#define HACE_CMD_CONTEXT_LOAD_ENABLE	(0)
#define HACE_CMD_CONTEXT_LOAD_DISABLE	(0x1 << 10)

#define HACE_CMD_CONTEXT_SAVE_ENABLE	(0)
#define HACE_CMD_CONTEXT_SAVE_DISABLE	(0x1 << 9)

#define HACE_CMD_AES						(0)
#define HACE_CMD_DES						(0)
#define HACE_CMD_RC4						(0x1 << 8)

#define HACE_CMD_DECRYPT					(0)
#define HACE_CMD_ENCRYPT					(0x1 << 7)

#define HACE_CMD_ECB						(0)
#define HACE_CMD_CBC						(0x1 << 4)
#define HACE_CMD_CFB						(0x1 << 5)
#define HACE_CMD_OFB						(0x3 << 4)
#define HACE_CMD_CTR						(0x1 << 6)

#define HACE_CMD_AES128					(0)
#define HACE_CMD_AES192					(0x1 << 2)
#define HACE_CMD_AES256					(0x1 << 3)

#define HACE_CMD_OP_CASCADE				(0x3)
#define HACE_CMD_OP_INDEPENDENT			(0x1)


/* AST_HACE_HASH_CMD				0x30		*/
#define HASH_CMD_INT_ENABLE				(0x1 << 9)
#define HASH_CMD_INT_DISABLE				(0)

#define HASH_CMD_HMAC						(0x1 << 7)

#define HASH_CMD_MD5						(0)
#define HASH_CMD_SHA1						(0x2 << 4)
#define HASH_CMD_SHA224					(0x4 << 4)
#define HASH_CMD_SHA256					(0x5 << 4)

#define HASH_CMD_MD5_SWAP				(0x1 << 2)
#define HASH_CMD_SHA_SWAP				(0x1 << 3)

/*
 * Asynchronous crypto request structure.
 *
 * This structure defines a request that is either queued for processing or
 * being processed.
 */
struct ast_crypto_req {
	struct list_head		list;
	struct ast_crypto_dev		*crypto_dev;
	struct crypto_async_request	*req;
	int				result;
	bool				is_encrypt;
	unsigned			ctx_id;
	dma_addr_t			src_addr, dst_addr;
	struct reset_control *reset;
	struct clk 			*clk;
	u32					apb_clk;	

	/* AEAD specific bits. */
	u8				*giv;
	size_t				giv_len;
	dma_addr_t			giv_pa;
};

struct ast_crypto_reqctx {
	unsigned long mode;
};

struct ast_crypto_dev {
	void __iomem			*regs;
	struct list_head		list;

	struct completion		cmd_complete;
	u32 					isr;
	struct reset_control *reset;
	struct clk 			*clk;
	u32					yclk;	
	
	spinlock_t			lock;

	//crypto
	struct ast_aes_ctx	*aes_ctx;
	struct ast_des_ctx	*des_ctx;
	struct ast_rc4_ctx		*rc4_ctx;

	//hash
	struct ast_sham_reqctx *sham_reqctx;

	struct crypto_queue	queue;

	struct tasklet_struct	done_task;
	struct tasklet_struct	queue_task;

	unsigned long			flags;	

	struct ablkcipher_request	*ablkcipher_req;
	struct ahash_request		*ahash_req;
	
	size_t	total;


	struct crypto_alg		*crypto_algs;
	struct ahash_alg		*ahash_algs;

	struct device			*dev;

	u32 		buf_size;	
	void	*ctx_buf;
	dma_addr_t	ctx_dma_addr;

	void	*buf_in;
	dma_addr_t	dma_addr_in;

	void	*buf_out;
	dma_addr_t	dma_addr_out;
	
	/* hash */
	void	*hash_key;
	dma_addr_t	hash_key_dma;

	void	*hash_src;
	dma_addr_t	hash_src_dma;

	void	*hash_digst;
	dma_addr_t	hash_digst_dma;

	u32 		cmd;	
};

/* Algorithm type mask. */
#define SPACC_CRYPTO_ALG_MASK		0x7

/* Block cipher context. */
struct ast_aes_ctx {
	struct ast_crypto_dev		*crypto_dev;
	u8				*iv;	
	u8				key[0xff];
	u8				key_len;	
	u16				block_size;	
	u8				done;
};

struct ast_des_ctx {
	struct ast_crypto_dev		*crypto_dev;
	u8				*iv;	
	u8				key[0x24];
	u8				key_len;	
	u8				done;
};

struct ast_rc4_ctx {
	struct ast_crypto_dev		*crypto_dev;
	u8				rc4_key[256];		
	int				key_len;	
	u8				done;
};

struct ast_cipher_context {
	struct ast_crypto_dev		*crypto_dev;	
	int			key_len;
	int			enc_type;
	union {
		u8		ast_aes[AES_MAX_KEY_SIZE];
		u8		ast_des[DES_KEY_SIZE];
		u8		ast_des3[3 * DES_KEY_SIZE];
		u8		ast_arc4[258]; /* S-box, X, Y */
	} key;	
	u8				done;	
};
/*************************************************************************************/
struct ast_sham_ctx {
	struct ast_crypto_dev	*crypto_dev;
	
	unsigned long		flags;	//hmac flag
	
	/* fallback stuff */
	struct crypto_shash	*fallback;
	struct crypto_shash 	*base_hash;		//for hmac
};

struct ast_sham_reqctx {
	struct ast_crypto_dev	*crypto_dev;
	unsigned long	flags;	//final update flag should no use
	u8			op; 	  	////0: init, 1 : upate , 2: final update	

	u32			cmd;

	u8	digest[SHA256_DIGEST_SIZE] __aligned(sizeof(u32));

	size_t			digcnt;
	
	size_t			bufcnt;

	/* walk state */
	struct scatterlist	*sg;
	unsigned int		offset;	/* offset in current sg */
	unsigned int		total;	/* total request */

	size_t 		block_size;

	u8	buffer[0] __aligned(sizeof(u32));
};

/*************************************************************************************/
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
	return ((val <<  8)       ) |
	       ((val >> 24) & 0xff) ;
};
u32 AESSubWord(u32 val)
{
	return ( AESSBox[ val        & 0xff]      ) |
	       ( AESSBox[(val >>  8) & 0xff] <<  8) |
	       ( AESSBox[(val >> 16) & 0xff] << 16) |
	       ( AESSBox[(val >> 24) & 0xff] << 24) ;
};
void AESKeyExpan(unsigned int bits, u32 *aeskeyin, u32* aeskeydram)
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

	for (i = aesNk; i < 4*(aesNr + 1); i++) {
		if (i % aesNk == 0)
			aeskeydram[i] = aeskeydram[i-aesNk] ^ AESSubWord(AESRotWord(aeskeydram[i-1])) ^ aesRcon[(i / aesNk)-1];
		else if ((aesNk > 6) && (i % aesNk == 4))
			aeskeydram[i] = aeskeydram[i-aesNk] ^ AESSubWord(           aeskeydram[i-1] )                         ;
		else
			aeskeydram[i] = aeskeydram[i-aesNk] ^                       aeskeydram[i-1]                           ;
	}

	for (i = 0; i < 4*(aesNr + 1); i++) {
		aeskeydram[i] = be32_to_cpu(aeskeydram[i]);
	}

}
/*************************************************************************************/
static inline void
ast_crypto_write(struct ast_crypto_dev *crypto, u32 val, u32 reg)
{
//	printk("uart dma write : val: %x , reg : %x \n",val,reg);	
	writel(val, crypto->regs + reg);
}

static inline u32
ast_crypto_read(struct ast_crypto_dev *crypto, u32 reg)
{
#if 0
	u32 val = readl(crypto->regs + reg);
	printk("R : reg %x , val: %x \n",reg, val);
	return val;
#else	
	return readl(crypto->regs + reg);
#endif
}

/*************************************************************************************/
struct ast_crypto_drv {
	struct list_head	dev_list;
	spinlock_t		lock;
};

static struct ast_crypto_drv ast_drv = {
	.dev_list = LIST_HEAD_INIT(ast_drv.dev_list),
	.lock = __SPIN_LOCK_UNLOCKED(ast_drv.lock),
};

/*************************************************************************************/
static int ast_crypto_sg_length(struct ablkcipher_request *req,
			struct scatterlist *sg)
{
	unsigned int total = req->nbytes;
	int sg_nb;
	unsigned int len;
	struct scatterlist *sg_list;

	sg_nb = 0;
	sg_list = sg;
	total = req->nbytes;

	while (total) {
		len = min(sg_list->length, total);

		sg_nb++;
		total -= len;

		sg_list = sg_next(sg_list);
		if (!sg_list)
			total = 0;
	}

	return sg_nb;
}

/*************************************************************************************/
static int ast_crypto_trigger(struct ast_crypto_dev *ast_crypto)
{
	u32 nbytes = 0;
	unsigned int	nb_in_sg = 0, nb_out_sg = 0;
	struct ablkcipher_request	*req = ast_crypto->ablkcipher_req;
//	struct scatterlist	*org_in_sg = req->src, *org_out_sg = req->dst;
	struct scatterlist	*in_sg = req->src, *out_sg = req->dst;

	if(ast_crypto->cmd & HACE_CMD_RC4) {
		//RC4		
		if(!ast_crypto->rc4_ctx->done) {
			 *(u32 *) (ast_crypto->ctx_buf + 8) = 0x0001;
			memcpy(ast_crypto->ctx_buf + 16, ast_crypto->rc4_ctx->rc4_key, 256);
			ast_crypto->rc4_ctx->done = 1;
		}
	} else { 
		if(ast_crypto->cmd & HACE_CMD_DES_SELECT) {
			//DES
			if(!ast_crypto->des_ctx->done) {
				if(ast_crypto->des_ctx->iv) {
					memcpy(ast_crypto->ctx_buf + 8, ast_crypto->des_ctx->iv, 8);
#if 0				
					for(i = 0; i < 8; i++) {
						ctxbuf[i + 8] = ast_crypto->des_ctx->iv[i];
					}
#endif 				
				}
#if 0			
				for(i = 0; i < 24; i++) {
					ctxbuf[0x10 + i] = ast_crypto->des_ctx->key[i];
				}
#else
				memcpy(ast_crypto->ctx_buf + 16, ast_crypto->des_ctx->key, 24);
#endif
				ast_crypto->des_ctx->done = 1;
			}
		} else {
			//AES
			if(!ast_crypto->aes_ctx->done) {

				if(ast_crypto->aes_ctx->iv) {		
#if 1				
					memcpy(ast_crypto->ctx_buf, ast_crypto->aes_ctx->iv, 16);
#else
					printk("Set iv: \n");
					for(i = 0; i < 0x10; i++) {
						ctxbuf[i] = ast_crypto->aes_ctx->iv[i];
						printk("%02x ", ctxbuf[i]);
					}		
					printk("\n");
#endif				
				}
#if 1
				memcpy(ast_crypto->ctx_buf + 16, ast_crypto->aes_ctx->key, 0xff);
#else
				printk("Set key: \n");			
				for(i = 0; i < 0xff; i++) {
					ctxbuf[0x10 + i] = ast_crypto->aes_ctx->key[i];
					printk("%02x ", ctxbuf[0x10 + i]);
				}
				printk("\n");
#endif			
				ast_crypto->aes_ctx->done = 1;
			}		
		}
	}

	nb_in_sg = ast_crypto_sg_length(req, in_sg);
	if (!nb_in_sg)
		return -EINVAL;

	nb_out_sg = ast_crypto_sg_length(req, out_sg);
	if (!nb_out_sg)
		return -EINVAL;

	if(nb_in_sg != nb_out_sg) {
		printk("ERROR !!!~~~~ \n");
	}
#if 0	
	err = dma_map_sg(ast_crypto->dev, org_in_sg, nb_in_sg,
			 DMA_TO_DEVICE);
	if (!err) {
		dev_err(ast_crypto->dev, "dma_map_sg() error\n");
		return -EINVAL;
	}

	err = dma_map_sg(ast_crypto->dev, org_out_sg, nb_out_sg,
			 DMA_FROM_DEVICE);

	if (!err) {
		dev_err(ast_crypto->dev, "dma_map_sg() error\n");
		return -EINVAL;
	}

//	printk("req->nbytes %d , nb_in_sg %d, nb_out_sg %d \n", req->nbytes, nb_in_sg, nb_out_sg);

	for(i = 0; i <nb_in_sg; i++)  {
//		printk("sg phy %x ", sg_phys(in_sg));

#ifdef AST_CRYPTO_IRQ
		ast_crypto->cmd |= HACE_CMD_INT_ENABLE;
		printk("ast_crypto->cmd %x \n", ast_crypto->cmd);
		ast_crypto->isr = 0;
		init_completion(&ast_crypto->cmd_complete); 	

#endif 
	
		ast_crypto_write(ast_crypto, sg_phys(in_sg), AST_HACE_SRC);
		ast_crypto_write(ast_crypto, sg_phys(out_sg), AST_HACE_DEST);
		ast_crypto_write(ast_crypto, in_sg->length, AST_HACE_DATA_LEN);
		printk("src : %x , dst : %x , len %d , out len %d \n", sg_phys(in_sg), sg_phys(out_sg), in_sg->length, out_sg->length);		
		
		ast_crypto_write(ast_crypto, ast_crypto->cmd, AST_HACE_CMD);
		

#ifdef AST_CRYPTO_IRQ
		wait_for_completion_interruptible(&ast_crypto->cmd_complete);
		printk("done \n");

#if 0
		if(!(ast_crypto->isr & HACE_CRYPTO_ISR)) {
			printk("INTR ERROR ast_crypto->isr %x \n", ast_crypto->isr);
		}
#endif			
#else
		while(ast_crypto_read(ast_crypto, AST_HACE_STS) & HACE_CRYPTO_BUSY);
#endif
	
		nbytes +=  in_sg->length;
		in_sg = sg_next(in_sg);
		out_sg = sg_next(out_sg);			
	}

	if(nbytes != req->nbytes) {
		printk("~~~ EOOERR  nbytes %d , req->nbytes %d \n", nbytes, req->nbytes);
	}
	dma_unmap_sg(ast_crypto->dev, org_in_sg, nb_in_sg, DMA_TO_DEVICE);
	dma_unmap_sg(ast_crypto->dev, org_out_sg, nb_out_sg, DMA_FROM_DEVICE);
#else
	nbytes = sg_copy_to_buffer(in_sg, nb_in_sg, 	ast_crypto->buf_in, req->nbytes);
//	printk("copy nbytes %d, req->nbytes %d , nb_in_sg %d, nb_out_sg %d \n", nbytes, req->nbytes, nb_in_sg, nb_out_sg);

	if (!nbytes)
		return -EINVAL;

	if(nbytes != req->nbytes) {
		printk("~~~ EOOERR  nbytes %d , req->nbytes %d \n", nbytes, req->nbytes);
	}

#ifdef AST_CRYPTO_IRQ
	ast_crypto->cmd |= HACE_CMD_INT_ENABLE;
	ast_crypto->isr = 0;
//	CDBUG("crypto cmd %x\n", ast_crypto->cmd);
	init_completion(&ast_crypto->cmd_complete); 	
#endif 
	
	ast_crypto_write(ast_crypto, ast_crypto->dma_addr_in, AST_HACE_SRC);
	ast_crypto_write(ast_crypto, ast_crypto->dma_addr_out, AST_HACE_DEST);
	ast_crypto_write(ast_crypto, req->nbytes, AST_HACE_DATA_LEN);

	ast_crypto_write(ast_crypto, ast_crypto->cmd, AST_HACE_CMD);
		
#ifdef AST_CRYPTO_IRQ
	wait_for_completion_interruptible(&ast_crypto->cmd_complete);
#if 0
	if(!(ast_crypto->isr & HACE_CRYPTO_ISR)) {
		printk("INTR ERROR ast_crypto->isr %x \n", ast_crypto->isr);
		printk("INTR ERROR ast_crypto->isr %x \n", ast_crypto->isr);
		printk("INTR ERROR ast_crypto->isr %x \n", ast_crypto->isr);
		printk("INTR ERROR ast_crypto->isr %x \n", ast_crypto->isr);
		printk("INTR ERROR ast_crypto->isr %x \n", ast_crypto->isr);
		printk("INTR ERROR ast_crypto->isr %x \n", ast_crypto->isr);
		printk("INTR ERROR ast_crypto->isr %x \n", ast_crypto->isr);
		printk("INTR ERROR ast_crypto->isr %x \n", ast_crypto->isr);
		printk("INTR ERROR ast_crypto->isr %x \n", ast_crypto->isr);		
	}
	if(ast_crypto->isr == 0)
		CDBUG("~~~ ast_crypto->isr %x \n", ast_crypto->isr);
#endif	
#else
	while(ast_crypto_read(ast_crypto, AST_HACE_STS) & HACE_CRYPTO_BUSY);
#endif

	nbytes = sg_copy_from_buffer(out_sg, nb_out_sg, ast_crypto->buf_out, req->nbytes);

//	printk("sg_copy_from_buffer nbytes %d req->nbytes %d\n",nbytes, req->nbytes);

	if (!nbytes) {
		printk("nbytes %d req->nbytes %d\n",nbytes, req->nbytes);
		return -EINVAL;
	}

#endif 

	return 0;
}

static int ast_crypto_handle_queue(struct ast_crypto_dev *ast_crypto,
				struct ablkcipher_request *req)
{
	struct crypto_async_request *async_req, *backlog;
//	struct ast_crypto_reqctx *rctx;
	unsigned long flags;
	int ret = 0;
	spin_lock_irqsave(&ast_crypto->lock, flags);
	if (req)
		ret = ablkcipher_enqueue_request(&ast_crypto->queue, req);

	if(ast_crypto_read(ast_crypto, AST_HACE_STS) & HACE_CRYPTO_BUSY) {
		printk("Engine Busy \n");
		spin_unlock_irqrestore(&ast_crypto->lock, flags);
		return ret;
	}
	backlog = crypto_get_backlog(&ast_crypto->queue);
	async_req = crypto_dequeue_request(&ast_crypto->queue);
	spin_unlock_irqrestore(&ast_crypto->lock, flags);

	if (!async_req)
		return ret;
	
	if (backlog)
		backlog->complete(backlog, -EINPROGRESS);

	req = ablkcipher_request_cast(async_req);

	/* assign new request to device */
	ast_crypto->ablkcipher_req = req;

	if(ast_crypto_trigger(ast_crypto) != 0)
		printk("ast_crypto_trigger error \n");

	req->base.complete(&req->base, 0);

	return 0;
}

static void ast_aes_done_task(unsigned long data)
{
#if 0	

	struct ast_crypto_dev *ast_crypto = (struct ast_crypto_dev *)data;
	int err;
	CRYPTO_DBUG("\n");

	atmel_aes_read_n(dd, AES_ODATAR(0), (u32 *) crypto_dev->buf_out,
			crypto_dev->bufcnt >> 2);

	if (sg_copy_from_buffer(crypto_dev->out_sg, crypto_dev->nb_out_sg,
		crypto_dev->buf_out, crypto_dev->bufcnt))
		err = 0;
	else
		err = -EINVAL;

	req->base.complete(&req->base, err);
#endif	
}

static void ast_aes_queue_task(unsigned long data)
{
	struct ast_crypto_dev *ast_crypto = (struct ast_crypto_dev *)data;
	CRYPTO_DBUG("\n");

	ast_crypto_handle_queue(ast_crypto, NULL);
}

/*************************************************************************************/
static int ast_rc4_crypt(struct ablkcipher_request *req, u32 cmd)
{
	struct ast_rc4_ctx *ctx = crypto_ablkcipher_ctx(crypto_ablkcipher_reqtfm(req));
//	struct ast_crypto_reqctx *rctx = ablkcipher_request_ctx(req);
	struct ast_crypto_dev *ast_crypto = NULL, *tmp;

	CRYPTO_DBUG("\n");	

	spin_lock_bh(&ast_drv.lock);
	if (!ctx->crypto_dev) {
		list_for_each_entry(tmp, &ast_drv.dev_list, list) {
			ast_crypto = tmp;
			break;
		}
		ctx->crypto_dev = ast_crypto;
	} else {
		ast_crypto = ctx->crypto_dev;
	}

	spin_unlock_bh(&ast_drv.lock);


	cmd |= HACE_CMD_RI_WO_DATA_ENABLE | 
			HACE_CMD_CONTEXT_LOAD_ENABLE | HACE_CMD_CONTEXT_SAVE_ENABLE;

	ast_crypto->cmd = cmd;
	ast_crypto->rc4_ctx = ctx;
	
	return ast_crypto_handle_queue(ast_crypto, req);
}

static int ast_rc4_setkey(struct crypto_ablkcipher *tfm, const u8 *in_key,
							unsigned int key_len)
			   
{	
	int i, j = 0, k = 0;
	struct ast_rc4_ctx *ctx = crypto_ablkcipher_ctx(tfm);
	CRYPTO_DBUG("keylen : %d \n", key_len);	

	for (i = 0; i < 256; i++)
		ctx->rc4_key[i] = i;

	for (i = 0; i < 256; i++) {
		u32 a = ctx->rc4_key[i];
		j = (j + in_key[k] + a) & 0xff;
		ctx->rc4_key[i] = ctx->rc4_key[j];
		ctx->rc4_key[j] = a;
		if (++k >= key_len)
			k = 0;
	}

	ctx->key_len = 256;
	ctx->done = 0;

	
	return 0;
}

static int ast_rc4_decrypt(struct ablkcipher_request *req)
{
	CRYPTO_DBUG("\n");	
	return ast_rc4_crypt(req, HACE_CMD_DECRYPT | HACE_CMD_RC4);
}

static int ast_rc4_encrypt(struct ablkcipher_request *req)
{
	CRYPTO_DBUG("\n");	
	return ast_rc4_crypt(req, HACE_CMD_ENCRYPT | HACE_CMD_RC4);
}

/*************************************************************************************/

static int ast_des_crypt(struct ablkcipher_request *req, u32 cmd)
{
	struct ast_des_ctx *ctx = crypto_ablkcipher_ctx(crypto_ablkcipher_reqtfm(req));
//	struct ast_crypto_reqctx *rctx = ablkcipher_request_ctx(req);
	struct ast_crypto_dev *ast_crypto = NULL, *tmp;

	CRYPTO_DBUG("\n");	

	spin_lock_bh(&ast_drv.lock);
	if (!ctx->crypto_dev) {
		list_for_each_entry(tmp, &ast_drv.dev_list, list) {
			ast_crypto = tmp;
			break;
		}
		ctx->crypto_dev = ast_crypto;
	} else {
		ast_crypto = ctx->crypto_dev;
	}

	spin_unlock_bh(&ast_drv.lock);
	
	ctx->iv = req->info;

	cmd |= HACE_CMD_DES_SELECT | HACE_CMD_RI_WO_DATA_ENABLE | 
			HACE_CMD_CONTEXT_LOAD_ENABLE | HACE_CMD_CONTEXT_SAVE_ENABLE;

	ast_crypto->cmd = cmd;
	ast_crypto->des_ctx = ctx;

	return ast_crypto_handle_queue(ast_crypto, req);
}

static int ast_des_setkey(struct crypto_ablkcipher *tfm, const u8 *key,
			   unsigned int keylen)
{
//	struct crypto_tfm *ctfm = crypto_ablkcipher_tfm(tfm);
	struct ast_aes_ctx *ctx = crypto_ablkcipher_ctx(tfm);

	CRYPTO_DBUG("bits : %d \n", (keylen * 8));

	if ((keylen != DES_KEY_SIZE) && (keylen != 2*DES_KEY_SIZE) && (keylen != 3*DES_KEY_SIZE)) {
		crypto_ablkcipher_set_flags(tfm, CRYPTO_TFM_RES_BAD_KEY_LEN);
		return -EINVAL;
	}

	memcpy(ctx->key, key, keylen);
	ctx->key_len = keylen;
	ctx->done = 0;

	return 0;
}

static int ast_tdes_ctr_decrypt(struct ablkcipher_request *req)
{
	CRYPTO_DBUG("\n");	
	return ast_des_crypt(req, HACE_CMD_DECRYPT | HACE_CMD_CTR | HACE_CMD_TRIPLE_DES | HACE_CMD_DES);
}

static int ast_tdes_ctr_encrypt(struct ablkcipher_request *req)
{
	CRYPTO_DBUG("\n");	
	return ast_des_crypt(req, HACE_CMD_ENCRYPT | HACE_CMD_CTR | HACE_CMD_TRIPLE_DES | HACE_CMD_DES);
}

static int ast_tdes_ofb_decrypt(struct ablkcipher_request *req)
{
	CRYPTO_DBUG("\n");	
	return ast_des_crypt(req, HACE_CMD_DECRYPT | HACE_CMD_OFB | HACE_CMD_TRIPLE_DES | HACE_CMD_DES);
}

static int ast_tdes_ofb_encrypt(struct ablkcipher_request *req)
{
	CRYPTO_DBUG("\n");	
	return ast_des_crypt(req, HACE_CMD_ENCRYPT | HACE_CMD_OFB | HACE_CMD_TRIPLE_DES | HACE_CMD_DES);
}

static int ast_tdes_cfb_decrypt(struct ablkcipher_request *req)
{
	CRYPTO_DBUG("\n");	
	return ast_des_crypt(req, HACE_CMD_DECRYPT | HACE_CMD_CFB | HACE_CMD_TRIPLE_DES | HACE_CMD_DES);
}

static int ast_tdes_cfb_encrypt(struct ablkcipher_request *req)
{
	CRYPTO_DBUG("\n");	
	return ast_des_crypt(req, HACE_CMD_ENCRYPT | HACE_CMD_CFB | HACE_CMD_TRIPLE_DES | HACE_CMD_DES);
}

static int ast_tdes_cbc_decrypt(struct ablkcipher_request *req)
{
	CRYPTO_DBUG("\n");	
	return ast_des_crypt(req, HACE_CMD_DECRYPT | HACE_CMD_CBC | HACE_CMD_TRIPLE_DES | HACE_CMD_DES);
}

static int ast_tdes_cbc_encrypt(struct ablkcipher_request *req)
{
	CRYPTO_DBUG("\n");	
	return ast_des_crypt(req, HACE_CMD_ENCRYPT | HACE_CMD_CBC | HACE_CMD_TRIPLE_DES | HACE_CMD_DES);
}

static int ast_tdes_ecb_decrypt(struct ablkcipher_request *req)
{
	CRYPTO_DBUG("\n");	
	return ast_des_crypt(req, HACE_CMD_DECRYPT | HACE_CMD_ECB | HACE_CMD_TRIPLE_DES | HACE_CMD_DES);
}

static int ast_tdes_ecb_encrypt(struct ablkcipher_request *req)
{
	CRYPTO_DBUG("\n");	
	return ast_des_crypt(req, HACE_CMD_ENCRYPT | HACE_CMD_ECB | HACE_CMD_TRIPLE_DES | HACE_CMD_DES);
}

static int ast_des_ctr_decrypt(struct ablkcipher_request *req)
{
	CRYPTO_DBUG("\n");	
	return ast_des_crypt(req, HACE_CMD_DECRYPT | HACE_CMD_CTR | HACE_CMD_SINGLE_DES | HACE_CMD_DES);
}

static int ast_des_ctr_encrypt(struct ablkcipher_request *req)
{
	CRYPTO_DBUG("\n");	
	return ast_des_crypt(req, HACE_CMD_ENCRYPT | HACE_CMD_CTR | HACE_CMD_SINGLE_DES | HACE_CMD_DES);
}

static int ast_des_ofb_decrypt(struct ablkcipher_request *req)
{
	CRYPTO_DBUG("\n");	
	return ast_des_crypt(req, HACE_CMD_DECRYPT | HACE_CMD_OFB | HACE_CMD_SINGLE_DES | HACE_CMD_DES);
}

static int ast_des_ofb_encrypt(struct ablkcipher_request *req)
{
	CRYPTO_DBUG("\n");	
	return ast_des_crypt(req, HACE_CMD_ENCRYPT | HACE_CMD_OFB | HACE_CMD_SINGLE_DES | HACE_CMD_DES);
}

static int ast_des_cfb_decrypt(struct ablkcipher_request *req)
{
	CRYPTO_DBUG("\n");	
	return ast_des_crypt(req, HACE_CMD_DECRYPT | HACE_CMD_CFB | HACE_CMD_SINGLE_DES | HACE_CMD_DES);
}

static int ast_des_cfb_encrypt(struct ablkcipher_request *req)
{
	CRYPTO_DBUG("\n");	
	return ast_des_crypt(req, HACE_CMD_ENCRYPT | HACE_CMD_CFB | HACE_CMD_SINGLE_DES | HACE_CMD_DES);
}

static int ast_des_cbc_decrypt(struct ablkcipher_request *req)
{
	CRYPTO_DBUG("\n");	
	return ast_des_crypt(req, HACE_CMD_DECRYPT | HACE_CMD_CBC | HACE_CMD_SINGLE_DES | HACE_CMD_DES);
}

static int ast_des_cbc_encrypt(struct ablkcipher_request *req)
{
	CRYPTO_DBUG("\n");	
	return ast_des_crypt(req, HACE_CMD_ENCRYPT | HACE_CMD_CBC | HACE_CMD_SINGLE_DES | HACE_CMD_DES);
}

static int ast_des_ecb_decrypt(struct ablkcipher_request *req)
{
	CRYPTO_DBUG("\n");	
	return ast_des_crypt(req, HACE_CMD_DECRYPT | HACE_CMD_ECB | HACE_CMD_SINGLE_DES | HACE_CMD_DES);
}

static int ast_des_ecb_encrypt(struct ablkcipher_request *req)
{
	CRYPTO_DBUG("\n");	
	return ast_des_crypt(req, HACE_CMD_ENCRYPT | HACE_CMD_ECB | HACE_CMD_SINGLE_DES | HACE_CMD_DES);
}
/* AES *************************************************************************************/
static int ast_aes_crypt(struct ablkcipher_request *areq, u32 cmd)
{
	struct ast_aes_ctx *ctx = crypto_ablkcipher_ctx(crypto_ablkcipher_reqtfm(areq));
//	struct ast_crypto_reqctx *rctx = ablkcipher_request_ctx(areq);
	struct ast_crypto_dev *ast_crypto = NULL, *tmp;
	
	CRYPTO_DBUG("\n");	

	spin_lock_bh(&ast_drv.lock);
	if (!ctx->crypto_dev) {
		list_for_each_entry(tmp, &ast_drv.dev_list, list) {
			ast_crypto = tmp;
			break;
		}
		ctx->crypto_dev = ast_crypto;
	} else {
		ast_crypto = ctx->crypto_dev;
	}

	spin_unlock_bh(&ast_drv.lock);

	ctx->iv = areq->info;	

	cmd |= HACE_CMD_AES_SELECT | HACE_CMD_RI_WO_DATA_ENABLE | 
			HACE_CMD_CONTEXT_LOAD_ENABLE | HACE_CMD_CONTEXT_SAVE_ENABLE;

	switch(ctx->key_len) {
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

	ast_crypto->cmd = cmd;

	ast_crypto->aes_ctx = ctx;

	return ast_crypto_handle_queue(ast_crypto, areq);

}

static int ast_aes_setkey(struct crypto_ablkcipher *tfm, const u8 *key,
			    unsigned int keylen)
{
	struct ast_aes_ctx *ctx = crypto_ablkcipher_ctx(tfm);
	CRYPTO_DBUG("bits : %d \n", (keylen * 8));

	if (keylen != AES_KEYSIZE_128 && keylen != AES_KEYSIZE_192 &&
		   keylen != AES_KEYSIZE_256) {
		crypto_ablkcipher_set_flags(tfm, CRYPTO_TFM_RES_BAD_KEY_LEN);
		return -EINVAL;
	}

	AESKeyExpan(keylen * 8, (u32*) key, (u32*)ctx->key);
	ctx->key_len = keylen;
	ctx->done = 0;
	return 0;
}

static int ast_aes_ctr_decrypt(struct ablkcipher_request *req)
{
	CRYPTO_DBUG("\n");	
	return ast_aes_crypt(req, HACE_CMD_DECRYPT | HACE_CMD_CTR);
}

static int ast_aes_ctr_encrypt(struct ablkcipher_request *req)
{
	CRYPTO_DBUG("\n");	
	return ast_aes_crypt(req, HACE_CMD_ENCRYPT | HACE_CMD_CTR);
}

static int ast_aes_ofb_decrypt(struct ablkcipher_request *req)
{
	CRYPTO_DBUG("\n");	
	return ast_aes_crypt(req, HACE_CMD_DECRYPT | HACE_CMD_OFB);
}

static int ast_aes_ofb_encrypt(struct ablkcipher_request *req)
{
	CRYPTO_DBUG("\n");	
	return ast_aes_crypt(req, HACE_CMD_ENCRYPT | HACE_CMD_OFB);
}

static int ast_aes_cfb_decrypt(struct ablkcipher_request *req)
{
	CRYPTO_DBUG("\n");	
	return ast_aes_crypt(req, HACE_CMD_DECRYPT | HACE_CMD_CFB);
}

static int ast_aes_cfb_encrypt(struct ablkcipher_request *req)
{
	CRYPTO_DBUG("\n");	
	return ast_aes_crypt(req, HACE_CMD_ENCRYPT | HACE_CMD_CFB);
}

static int ast_aes_ecb_decrypt(struct ablkcipher_request *req)
{
	CRYPTO_DBUG("\n");	
	return ast_aes_crypt(req, HACE_CMD_DECRYPT | HACE_CMD_ECB);
}

static int ast_aes_ecb_encrypt(struct ablkcipher_request *req)
{
	CRYPTO_DBUG("\n");	
	return ast_aes_crypt(req, HACE_CMD_ENCRYPT | HACE_CMD_ECB);
}

static int ast_aes_cbc_decrypt(struct ablkcipher_request *req)
{
	CRYPTO_DBUG("\n");	
	return ast_aes_crypt(req, HACE_CMD_DECRYPT | HACE_CMD_CBC);
}

static int ast_aes_cbc_encrypt(struct ablkcipher_request *req)
{
	CRYPTO_DBUG("\n");	
	return ast_aes_crypt(req, HACE_CMD_ENCRYPT | HACE_CMD_CBC);
}

/*************************************************************************************/
static int ast_crypto_cra_init(struct crypto_tfm *tfm)
{
	CRYPTO_DBUG("\n");

	tfm->crt_ablkcipher.reqsize = sizeof(struct ast_crypto_reqctx);

	return 0;
}

static void ast_crypto_cra_exit(struct crypto_tfm *tfm)
{
	CRYPTO_DBUG("\n");

}

/*************************************************************************************/
static size_t ast_sha_append_sg(struct ast_sham_reqctx *ctx)
{
	size_t count;
	struct ast_crypto_dev	*ast_crypto = ctx->crypto_dev;
	u8	*hash_buff = ast_crypto->hash_src;
	HASH_DBUG("ctx->bufcnt %d, ctx->offset %d, ctx->total %d\n", ctx->bufcnt, ctx->offset, ctx->total);

	while ((ctx->bufcnt < AST_HASH_BUFF_SIZE) && ctx->total) {
		count = min(ctx->sg->length - ctx->offset, ctx->total);
		count = min(count, AST_HASH_BUFF_SIZE - ctx->bufcnt);

		if (count <= 0) {
			/*
			 * Check if count <= 0 because the buffer is full or
			 * because the sg length is 0. In the latest case,
			 * check if there is another sg in the list, a 0 length
			 * sg doesn't necessarily mean the end of the sg list.
			 */
			if ((ctx->sg->length == 0) && !sg_is_last(ctx->sg)) {
				ctx->sg = sg_next(ctx->sg);
				continue;
			} else
				break;
		}

		scatterwalk_map_and_copy(hash_buff + ctx->bufcnt, ctx->sg,
			ctx->offset, count, 0);

		ctx->bufcnt += count;
		ctx->offset += count;
		ctx->total -= count;

		if (ctx->offset == ctx->sg->length) {
			ctx->sg = sg_next(ctx->sg);
			if (ctx->sg)
				ctx->offset = 0;
			else
				ctx->total = 0;
		}
	}
	HASH_DBUG("ctx->bufcnt %d, ctx->offset %d, ctx->total %d\n", ctx->bufcnt, ctx->offset, ctx->total);

	return 0;
}

static int ast_hash_trigger(struct ast_crypto_dev *ast_crypto)
{
	struct ahash_request *req = ast_crypto->ahash_req;
	struct ast_sham_reqctx *ctx = ahash_request_ctx(req);

	HASH_DBUG("ctx->bufcnt %d\n", ctx->bufcnt);

#ifdef AST_CRYPTO_IRQ
	ast_crypto->cmd |= HASH_CMD_INT_ENABLE;
	ast_crypto->isr = 0;
	init_completion(&ast_crypto->cmd_complete); 	
//	CDBUG("hash cmd %x\n", ast_crypto->cmd);
#endif 

	ast_crypto_write(ast_crypto, ast_crypto->hash_src_dma, AST_HACE_HASH_SRC);
	ast_crypto_write(ast_crypto, ast_crypto->hash_digst_dma, AST_HACE_HASH_DIGEST_BUFF);
	ast_crypto_write(ast_crypto, ast_crypto->hash_key_dma, AST_HACE_HASH_KEY_BUFF);	
	ast_crypto_write(ast_crypto, ctx->bufcnt, AST_HACE_HASH_DATA_LEN);

	ast_crypto_write(ast_crypto, ast_crypto->cmd, AST_HACE_HASH_CMD);


#ifdef AST_CRYPTO_IRQ
	wait_for_completion_interruptible(&ast_crypto->cmd_complete);
#if 0
	if(!(ast_crypto->isr & HACE_HASH_ISR)) {
		printk("INTR ERROR ast_crypto->isr %x \n", ast_crypto->isr);
	}
	CDBUG("irq %x\n", ast_crypto->isr);
#endif 
#else
	while(ast_crypto_read(ast_crypto, AST_HACE_STS) & HACE_HASH_BUSY);
#endif

#if 0
	printk("digst dma : %x \n",ast_crypto->hash_digst_dma);	
	for(i=0;i<ctx->digcnt;i++)
		printk("%02x ", digst[i]);

	printk("\n");		
#endif 

	memcpy(ast_crypto->ahash_req->result, ast_crypto->hash_digst, ctx->digcnt);

//	printk("copy to sg done \n");	
	return 0;
}

static int ast_hash_handle_queue(struct ast_crypto_dev *ast_crypto,
				  struct ahash_request *req) 
{
	struct crypto_async_request *async_req, *backlog;
	struct ast_sham_reqctx *ctx;
	unsigned long flags;
	int ret = 0;
	
	HASH_DBUG("nbytes : %d \n",req->nbytes);

	spin_lock_irqsave(&ast_crypto->lock, flags);
	if (req)
		ret = ahash_enqueue_request(&ast_crypto->queue, req);

	if(ast_crypto_read(ast_crypto, AST_HACE_STS) & HACE_HASH_BUSY) {
		printk("hash Engine Busy \n");
		spin_unlock_irqrestore(&ast_crypto->lock, flags);
		return -1;
	}

	backlog = crypto_get_backlog(&ast_crypto->queue);
	async_req = crypto_dequeue_request(&ast_crypto->queue);
	spin_unlock_irqrestore(&ast_crypto->lock, flags);

	if (!async_req)
		return -1;

	if (backlog)
		backlog->complete(backlog, -EINPROGRESS);

	req = ahash_request_cast(async_req);
	ast_crypto->ahash_req = req;
	ctx = ahash_request_ctx(req);

	ast_crypto->cmd = ctx->cmd;
	
	ast_hash_trigger(ast_crypto);		

	req->base.complete(&req->base, 0);

	return ret;
}

static int ast_sham_init(struct ahash_request *req)
{
	struct crypto_ahash *tfm = crypto_ahash_reqtfm(req);
	struct ast_sham_ctx *tctx = crypto_ahash_ctx(tfm);
	struct ast_sham_reqctx *ctx = ahash_request_ctx(req);
	struct ast_crypto_dev *ast_crypto = NULL, *tmp;
	
	HASH_DBUG("init: digest size: %d\n", crypto_ahash_digestsize(tfm));

	spin_lock_bh(&ast_drv.lock);
	if (!ctx->crypto_dev) {
		list_for_each_entry(tmp, &ast_drv.dev_list, list) {
			ast_crypto = tmp;
			break;
		}
		tctx->crypto_dev = ast_crypto;

	} else {
		ast_crypto = tctx->crypto_dev;
	}

	ctx->crypto_dev = ast_crypto;

	spin_unlock_bh(&ast_drv.lock);

	ctx->flags = 0;
	ctx->cmd = 0;

	switch (crypto_ahash_digestsize(tfm)) {
		case MD5_DIGEST_SIZE:
			ctx->cmd = HASH_CMD_MD5 | HASH_CMD_MD5_SWAP;
			ctx->block_size = SHA1_BLOCK_SIZE;
			ctx->digcnt = MD5_DIGEST_SIZE;
			break;		
		case SHA1_DIGEST_SIZE:
			ctx->cmd = HASH_CMD_SHA1 | HASH_CMD_SHA_SWAP;
			ctx->block_size = SHA1_BLOCK_SIZE;
			ctx->digcnt = SHA1_DIGEST_SIZE;			
			break;
		case SHA224_DIGEST_SIZE:
			ctx->cmd = HASH_CMD_SHA224 | HASH_CMD_SHA_SWAP;
			ctx->block_size = SHA224_BLOCK_SIZE;
			ctx->digcnt = SHA224_DIGEST_SIZE;						
			break;
		case SHA256_DIGEST_SIZE:
			ctx->cmd = HASH_CMD_SHA256 | HASH_CMD_SHA_SWAP;
			ctx->block_size = SHA256_BLOCK_SIZE;
			ctx->digcnt = SHA256_DIGEST_SIZE;						
			break;
		default:
			printk("%d not support \n", crypto_ahash_digestsize(tfm));		
			return -EINVAL;
			break;
	}

	ctx->bufcnt = 0;

	if(tctx->flags & HASH_CMD_HMAC) {
		HASH_DBUG("wHMAC \n");
		ctx->cmd |= HASH_CMD_HMAC;
	}
	return 0;
}

static int ast_sha_update(struct ahash_request *req)
{
	struct ast_sham_reqctx *ctx = ahash_request_ctx(req);
//	struct ast_crypto_dev *ast_crypto = ctx->crypto_dev;

	if (!req->nbytes)
		return -1;
	
	HASH_DBUG("req->nbytes %d\n", req->nbytes);

	ctx->total = req->nbytes;
	ctx->sg = req->src;
	ctx->offset = 0;

	if (ctx->bufcnt + ctx->total <= AST_HASH_BUFF_SIZE) {
		ast_sha_append_sg(ctx);
		return 0;
	} else {
		printk("ast_sha_update TODO ...ctx->bufcnt %d, ctx->total %d \n", ctx->bufcnt, ctx->total);
//		return ast_hash_handle_queue(ast_crypto, req);
	}

	return 0;
}

static int ast_sha_final(struct ahash_request *req)
{
	struct ast_sham_reqctx *ctx = ahash_request_ctx(req);
	struct ast_sham_ctx *tctx = crypto_tfm_ctx(req->base.tfm);
	struct ast_crypto_dev *ast_crypto = tctx->crypto_dev;

	ctx->flags = 1;
	ctx->op = 2;
	
	HASH_DBUG("ctx->bufcnt : %d , ctx->total : %d,  req->nbytes : %d \n",ctx->bufcnt, ctx->total, req->nbytes);
	if(req->nbytes) {
		printk("ast_sha_final ~~ TODO req->nbytes %d ~~~ \n", req->nbytes);
		ast_sha_update(req);
	}

	return ast_hash_handle_queue(ast_crypto, req);

}

static int ast_sha_finup(struct ahash_request *req)
{
//	struct ast_sham_reqctx *ctx = ahash_request_ctx(req);
	int err1, err2;

	HASH_DBUG("Final up TODO ...\n");

//	ctx->flags |= SHA_FLAGS_FINUP;

	err1 = ast_sha_update(req);
	if (err1 == -EINPROGRESS || err1 == -EBUSY)
		return err1;

	/*
	 * final() has to be always called to cleanup resources
	 * even if udpate() failed, except EINPROGRESS
	 */
	err2 = ast_sha_final(req);

	return err1 ?: err2;

}

static int ast_sha_digest(struct ahash_request *req)
{
	HASH_DBUG("TOTO~~~~~~\n");

	return ast_sham_init(req) ?: ast_sha_finup(req);
}

static int ast_sham_setkey(struct crypto_ahash *tfm, const u8 *key,
			  unsigned int keylen)
{
	struct ast_sham_ctx *ctx = crypto_ahash_ctx(tfm);
//	int bs = crypto_shash_blocksize(ctx->base_hash);
//	int ds = crypto_shash_digestsize(ctx->base_hash);
	struct ast_crypto_dev *ast_crypto = NULL, *tmp;
	int err=0;

	HASH_DBUG("keylen %d\n", keylen);

	spin_lock_bh(&ast_drv.lock);
	if (!ctx->crypto_dev) {
		list_for_each_entry(tmp, &ast_drv.dev_list, list) {
			ast_crypto = tmp;
			break;
		}
		ctx->crypto_dev = ast_crypto;

	} else {
		ast_crypto = ctx->crypto_dev;
	}

	spin_unlock_bh(&ast_drv.lock);

	err = crypto_shash_setkey(ctx->fallback, key, keylen);
	if (err)
		return err;

	memcpy(ast_crypto->hash_key, key, keylen);

	return err;
}

/*************************************************************************************/
static int ast_sha_cra_init_alg(struct crypto_tfm *tfm, const char *alg_base)
{
	struct ast_sham_ctx *tctx = crypto_tfm_ctx(tfm);
	const char *alg_name = crypto_tfm_alg_name(tfm);

	HASH_DBUG("%s \n", alg_name);

	/* Allocate a fallback and abort if it failed. */
	tctx->fallback = crypto_alloc_shash(alg_name, 0,
						CRYPTO_ALG_NEED_FALLBACK);
	if (IS_ERR(tctx->fallback)) {
		pr_err("ast-sham: fallback driver '%s' "
				"could not be loaded.\n", alg_name);
		return PTR_ERR(tctx->fallback);
	}

	crypto_ahash_set_reqsize(__crypto_ahash_cast(tfm),
				 sizeof(struct ast_sham_reqctx));

	tctx->flags = 0;
	
	if (alg_base) {
		tctx->flags = HASH_CMD_HMAC;
		tctx->base_hash = crypto_alloc_shash(alg_base, 0,
						CRYPTO_ALG_NEED_FALLBACK);
		if (IS_ERR(tctx->base_hash)) {
			pr_err("ast-sham: base driver '%s' "
					"could not be loaded.\n", alg_base);
			crypto_free_shash(tctx->fallback);
			return PTR_ERR(tctx->base_hash);
		}
	}

	return 0;

}

static int ast_sham_cra_init(struct crypto_tfm *tfm)
{
	return ast_sha_cra_init_alg(tfm, NULL);
}

static int ast_sha_cra_sha1_init(struct crypto_tfm *tfm)
{
	return ast_sha_cra_init_alg(tfm, "sha1");
}

static int ast_sha_cra_sha224_init(struct crypto_tfm *tfm)
{
	return ast_sha_cra_init_alg(tfm, "sha224");
}

static int ast_sha_cra_sha256_init(struct crypto_tfm *tfm)
{
	return ast_sha_cra_init_alg(tfm, "sha256");
}

static int ast_sha_cra_md5_init(struct crypto_tfm *tfm)
{
	return ast_sha_cra_init_alg(tfm, "md5");
}

static void ast_sham_cra_exit(struct crypto_tfm *tfm)
{
	struct ast_sham_ctx *tctx = crypto_tfm_ctx(tfm);

	HASH_DBUG("\n");

	crypto_free_shash(tctx->fallback);
	tctx->fallback = NULL;

	if (tctx->base_hash) {
		crypto_free_shash(tctx->base_hash);
	}
}

static struct crypto_alg ast_crypto_algs[] = {	
	{
		.cra_name 		= "ecb(aes)",
		.cra_driver_name 	= "ast-ecb-aes",
		.cra_priority 		= 300,
		.cra_flags 		= CRYPTO_ALG_TYPE_ABLKCIPHER | 
							CRYPTO_ALG_KERN_DRIVER_ONLY | 
							CRYPTO_ALG_ASYNC,
		.cra_blocksize 	= AES_BLOCK_SIZE,
		.cra_ctxsize 		= sizeof(struct ast_aes_ctx),
		.cra_alignmask	= 0xf,			
		.cra_type 		= &crypto_ablkcipher_type,
		.cra_module 		= THIS_MODULE,
		.cra_init 		= ast_crypto_cra_init,
		.cra_exit 		= ast_crypto_cra_exit,			
		.cra_u.ablkcipher = {
			//no iv
			.min_keysize 	= AES_MIN_KEY_SIZE,
			.max_keysize 	= AES_MAX_KEY_SIZE,			
			.setkey 		= ast_aes_setkey,
			.encrypt 	= ast_aes_ecb_encrypt,
			.decrypt	= ast_aes_ecb_decrypt,
		},
	},	
	{
		.cra_name 		= "cbc(aes)",
		.cra_driver_name 	= "ast-cbc-aes",
		.cra_priority 		= 300,
		.cra_flags 		= CRYPTO_ALG_TYPE_ABLKCIPHER | 
							CRYPTO_ALG_KERN_DRIVER_ONLY |
							CRYPTO_ALG_ASYNC,
		.cra_blocksize 	= AES_BLOCK_SIZE,
		.cra_ctxsize 		= sizeof(struct ast_aes_ctx),
		.cra_alignmask	= 0xf,			
		.cra_type 		= &crypto_ablkcipher_type,
		.cra_module 		= THIS_MODULE,
		.cra_init 		= ast_crypto_cra_init,
		.cra_exit 		= ast_crypto_cra_exit,			
		.cra_u.ablkcipher = {
			.ivsize		= AES_BLOCK_SIZE,	
			.min_keysize 	= AES_MIN_KEY_SIZE,
			.max_keysize 	= AES_MAX_KEY_SIZE,			
			.setkey 		= ast_aes_setkey,
			.encrypt 	= ast_aes_cbc_encrypt,
			.decrypt	= ast_aes_cbc_decrypt,
		},
	},	
	{
		.cra_name		= "cfb(aes)",
		.cra_driver_name	= "ast-cfb-aes",
		.cra_priority		= 300,
		.cra_flags		= CRYPTO_ALG_TYPE_ABLKCIPHER |
							CRYPTO_ALG_KERN_DRIVER_ONLY |
							CRYPTO_ALG_ASYNC,
		.cra_blocksize	= AES_BLOCK_SIZE,
		.cra_ctxsize		= sizeof(struct ast_aes_ctx),
		.cra_alignmask	= 0xf,			
		.cra_type		= &crypto_ablkcipher_type,
		.cra_module 		= THIS_MODULE,
		.cra_init		= ast_crypto_cra_init,
		.cra_exit		= ast_crypto_cra_exit, 		
		.cra_u.ablkcipher = {
			.ivsize		= AES_BLOCK_SIZE,		
			.min_keysize	= AES_MIN_KEY_SIZE,
			.max_keysize	= AES_MAX_KEY_SIZE, 		
			.setkey 		= ast_aes_setkey,
			.encrypt	= ast_aes_cfb_encrypt,
			.decrypt	= ast_aes_cfb_decrypt,
		},
	},	
	{
		.cra_name		= "ofb(aes)",
		.cra_driver_name	= "ast-ofb-aes",
		.cra_priority		= 300,
		.cra_flags		= CRYPTO_ALG_TYPE_ABLKCIPHER |
						CRYPTO_ALG_KERN_DRIVER_ONLY |
						CRYPTO_ALG_ASYNC,
		.cra_blocksize	= AES_BLOCK_SIZE,
		.cra_ctxsize		= sizeof(struct ast_aes_ctx),
		.cra_alignmask	= 0xf,			
		.cra_type		= &crypto_ablkcipher_type,
		.cra_module 		= THIS_MODULE,
		.cra_init		= ast_crypto_cra_init,
		.cra_exit		= ast_crypto_cra_exit, 		
		.cra_u.ablkcipher = {
			.ivsize		= AES_BLOCK_SIZE,		
			.min_keysize	= AES_MIN_KEY_SIZE,
			.max_keysize	= AES_MAX_KEY_SIZE, 		
			.setkey 		= ast_aes_setkey,
			.encrypt	= ast_aes_ofb_encrypt,
			.decrypt	= ast_aes_ofb_decrypt,
		},
	},	
	{
		.cra_name		= "ctr(aes)",
		.cra_driver_name	= "ast-ctr-aes",
		.cra_priority		= 300,
		.cra_flags		= CRYPTO_ALG_TYPE_ABLKCIPHER |
						CRYPTO_ALG_KERN_DRIVER_ONLY |
						CRYPTO_ALG_ASYNC,
		.cra_blocksize	= AES_BLOCK_SIZE,
		.cra_ctxsize		= sizeof(struct ast_aes_ctx),
		.cra_alignmask	= 0xf,			
		.cra_type		= &crypto_ablkcipher_type,
		.cra_module 		= THIS_MODULE,
		.cra_init		= ast_crypto_cra_init,
		.cra_exit		= ast_crypto_cra_exit, 		
		.cra_u.ablkcipher = {
			.ivsize		= AES_BLOCK_SIZE,		
			.min_keysize	= AES_MIN_KEY_SIZE,
			.max_keysize	= AES_MAX_KEY_SIZE, 		
			.setkey 		= ast_aes_setkey,
			.encrypt	= ast_aes_ctr_encrypt,
			.decrypt	= ast_aes_ctr_decrypt,
		},
	},
	{
		.cra_name		= "ecb(des)",
		.cra_driver_name	= "ast-ecb-des",
		.cra_priority		= 300,
		.cra_flags		= CRYPTO_ALG_TYPE_ABLKCIPHER |
							CRYPTO_ALG_KERN_DRIVER_ONLY |
							CRYPTO_ALG_ASYNC,
		.cra_blocksize		= DES_BLOCK_SIZE,
		.cra_ctxsize		= sizeof(struct ast_des_ctx),
		.cra_alignmask	= 0xf,
		.cra_type		= &crypto_ablkcipher_type,
		.cra_module 		= THIS_MODULE,
		.cra_init		= ast_crypto_cra_init,
		.cra_exit		= ast_crypto_cra_exit,
		.cra_u.ablkcipher = {
			//no iv
			.min_keysize	= DES_KEY_SIZE,
			.max_keysize	= DES_KEY_SIZE,
			.setkey		= ast_des_setkey,
			.encrypt	= ast_des_ecb_encrypt,
			.decrypt	= ast_des_ecb_decrypt,
		},
	},
	{
		.cra_name		= "cbc(des)",
		.cra_driver_name	= "ast-cbc-des",
		.cra_priority		= 300,
		.cra_flags		= CRYPTO_ALG_TYPE_ABLKCIPHER | 
							CRYPTO_ALG_KERN_DRIVER_ONLY |
							CRYPTO_ALG_ASYNC,
		.cra_blocksize		= DES_BLOCK_SIZE,
		.cra_ctxsize		= sizeof(struct ast_des_ctx),
		.cra_alignmask	= 0xf,
		.cra_type		= &crypto_ablkcipher_type,
		.cra_module 		= THIS_MODULE,
		.cra_init		= ast_crypto_cra_init,
		.cra_exit		= ast_crypto_cra_exit,
		.cra_u.ablkcipher = {
			.ivsize		= DES_BLOCK_SIZE,
			.min_keysize	= DES_KEY_SIZE,
			.max_keysize	= DES_KEY_SIZE,
			.setkey 	= ast_des_setkey,
			.encrypt	= ast_des_cbc_encrypt,
			.decrypt	= ast_des_cbc_decrypt,
		},
	},
	{
		.cra_name		= "cfb(des)",
		.cra_driver_name	= "ast-cfb-des",
		.cra_priority		= 300,
		.cra_flags		= CRYPTO_ALG_TYPE_ABLKCIPHER | 
							CRYPTO_ALG_KERN_DRIVER_ONLY |
							CRYPTO_ALG_ASYNC,
		.cra_blocksize		= DES_BLOCK_SIZE,
		.cra_ctxsize		= sizeof(struct ast_des_ctx),
		.cra_alignmask	= 0xf,
		.cra_type		= &crypto_ablkcipher_type,
		.cra_module 		= THIS_MODULE,
		.cra_init		= ast_crypto_cra_init,
		.cra_exit		= ast_crypto_cra_exit,
		.cra_u.ablkcipher = {
			.ivsize		= DES_BLOCK_SIZE,		
			.min_keysize	= DES_KEY_SIZE,
			.max_keysize	= DES_KEY_SIZE,
			.setkey 	= ast_des_setkey,
			.encrypt	= ast_des_cfb_encrypt,
			.decrypt	= ast_des_cfb_decrypt,
		},
	},
	{
		.cra_name		= "ofb(des)",
		.cra_driver_name	= "ast-ofb-des",
		.cra_priority		= 300,
		.cra_flags		= CRYPTO_ALG_TYPE_ABLKCIPHER | 
							CRYPTO_ALG_KERN_DRIVER_ONLY |
							CRYPTO_ALG_ASYNC,
		.cra_blocksize		= DES_BLOCK_SIZE,
		.cra_ctxsize		= sizeof(struct ast_des_ctx),
		.cra_alignmask	= 0xf,
		.cra_type		= &crypto_ablkcipher_type,
		.cra_module 		= THIS_MODULE,
		.cra_init		= ast_crypto_cra_init,
		.cra_exit		= ast_crypto_cra_exit,
		.cra_u.ablkcipher = {
			.ivsize		= DES_BLOCK_SIZE,		
			.min_keysize	= 2*DES_KEY_SIZE,
			.max_keysize	= 3*DES_KEY_SIZE,
			.setkey 	= ast_des_setkey,
			.encrypt	= ast_des_ofb_encrypt,
			.decrypt	= ast_des_ofb_decrypt,
		},
	},
	{
		.cra_name		= "ctr(des)",
		.cra_driver_name	= "ast-ctr-des",
		.cra_priority		= 300,
		.cra_flags		= CRYPTO_ALG_TYPE_ABLKCIPHER | 
							CRYPTO_ALG_KERN_DRIVER_ONLY |
							CRYPTO_ALG_ASYNC,
		.cra_blocksize		= DES_BLOCK_SIZE,
		.cra_ctxsize		= sizeof(struct ast_des_ctx),
		.cra_alignmask	= 0xf,
		.cra_type		= &crypto_ablkcipher_type,
		.cra_module 		= THIS_MODULE,
		.cra_init		= ast_crypto_cra_init,
		.cra_exit		= ast_crypto_cra_exit,
		.cra_u.ablkcipher = {
			.ivsize		= DES_BLOCK_SIZE,		
			.min_keysize	= 2*DES_KEY_SIZE,
			.max_keysize	= 3*DES_KEY_SIZE,
			.setkey 	= ast_des_setkey,
			.encrypt	= ast_des_ctr_encrypt,
			.decrypt	= ast_des_ctr_decrypt,
		},
	},	
	{
		.cra_name		= "ecb(des3_ede)",
		.cra_driver_name	= "ast-ecb-tdes",
		.cra_priority		= 300,
		.cra_flags		= CRYPTO_ALG_TYPE_ABLKCIPHER | 
							CRYPTO_ALG_KERN_DRIVER_ONLY |
							CRYPTO_ALG_ASYNC,
		.cra_blocksize		= DES_BLOCK_SIZE,
		.cra_ctxsize		= sizeof(struct ast_des_ctx),
		.cra_alignmask	= 0xf,
		.cra_type		= &crypto_ablkcipher_type,
		.cra_module 		= THIS_MODULE,
		.cra_init		= ast_crypto_cra_init,
		.cra_exit		= ast_crypto_cra_exit,
		.cra_u.ablkcipher = {
			.ivsize		= DES_BLOCK_SIZE,		
			.min_keysize	= 2*DES_KEY_SIZE,
			.max_keysize	= 3*DES_KEY_SIZE,
			.setkey		= ast_des_setkey,
			.encrypt	= ast_tdes_ecb_encrypt,
			.decrypt	= ast_tdes_ecb_decrypt,
		},
	},
	{
		.cra_name		= "cbc(des3_ede)",
		.cra_driver_name	= "ast-cbc-tdes",
		.cra_priority		= 300,
		.cra_flags		= CRYPTO_ALG_TYPE_ABLKCIPHER | 
							CRYPTO_ALG_KERN_DRIVER_ONLY |
							CRYPTO_ALG_ASYNC,
		.cra_blocksize		= DES_BLOCK_SIZE,
		.cra_ctxsize		= sizeof(struct ast_des_ctx),
		.cra_alignmask	= 0xf,
		.cra_type		= &crypto_ablkcipher_type,
		.cra_module 		= THIS_MODULE,
		.cra_init		= ast_crypto_cra_init,
		.cra_exit		= ast_crypto_cra_exit,
		.cra_u.ablkcipher = {
			.ivsize		= DES_BLOCK_SIZE,		
			.min_keysize	= 2*DES_KEY_SIZE,
			.max_keysize	= 3*DES_KEY_SIZE,
			.setkey 	= ast_des_setkey,
			.encrypt	= ast_tdes_cbc_encrypt,
			.decrypt	= ast_tdes_cbc_decrypt,
		},
	},
	{
		.cra_name		= "cfb(des3_ede)",
		.cra_driver_name	= "ast-cfb-tdes",
		.cra_priority		= 300,
		.cra_flags		= CRYPTO_ALG_TYPE_ABLKCIPHER | 
							CRYPTO_ALG_KERN_DRIVER_ONLY |
							CRYPTO_ALG_ASYNC,
		.cra_blocksize		= DES_BLOCK_SIZE,
		.cra_ctxsize		= sizeof(struct ast_des_ctx),
		.cra_alignmask	= 0xf,
		.cra_type		= &crypto_ablkcipher_type,
		.cra_module 		= THIS_MODULE,
		.cra_init		= ast_crypto_cra_init,
		.cra_exit		= ast_crypto_cra_exit,
		.cra_u.ablkcipher = {
			.ivsize		= DES_BLOCK_SIZE,		
			.min_keysize	= 2*DES_KEY_SIZE,
			.max_keysize	= 3*DES_KEY_SIZE,
			.setkey 	= ast_des_setkey,
			.encrypt	= ast_tdes_cfb_encrypt,
			.decrypt	= ast_tdes_cfb_decrypt,
		},
	},
	{
		.cra_name		= "ofb(des3_ede)",
		.cra_driver_name	= "ast-ofb-tdes",
		.cra_priority		= 300,
		.cra_flags		= CRYPTO_ALG_TYPE_ABLKCIPHER | 
							CRYPTO_ALG_KERN_DRIVER_ONLY |
							CRYPTO_ALG_ASYNC,
		.cra_blocksize		= DES_BLOCK_SIZE,
		.cra_ctxsize		= sizeof(struct ast_des_ctx),
		.cra_alignmask	= 0xf,
		.cra_type		= &crypto_ablkcipher_type,
		.cra_module 		= THIS_MODULE,
		.cra_init		= ast_crypto_cra_init,
		.cra_exit		= ast_crypto_cra_exit,
		.cra_u.ablkcipher = {
			.ivsize		= DES_BLOCK_SIZE,		
			.min_keysize	= 2*DES_KEY_SIZE,
			.max_keysize	= 3*DES_KEY_SIZE,
			.setkey 	= ast_des_setkey,
			.encrypt	= ast_tdes_ofb_encrypt,
			.decrypt	= ast_tdes_ofb_decrypt,
		},
	},
	{
		.cra_name		= "ctr(des3_ede)",
		.cra_driver_name	= "ast-ctr-tdes",
		.cra_priority		= 300,
		.cra_flags		= CRYPTO_ALG_TYPE_ABLKCIPHER | 
							CRYPTO_ALG_KERN_DRIVER_ONLY |
							CRYPTO_ALG_ASYNC,
		.cra_blocksize		= DES_BLOCK_SIZE,
		.cra_ctxsize		= sizeof(struct ast_des_ctx),
		.cra_alignmask	= 0xf,
		.cra_type		= &crypto_ablkcipher_type,
		.cra_module 		= THIS_MODULE,
		.cra_init		= ast_crypto_cra_init,
		.cra_exit		= ast_crypto_cra_exit,
		.cra_u.ablkcipher = {
			.ivsize		= DES_BLOCK_SIZE,		
			.min_keysize	= 2*DES_KEY_SIZE,
			.max_keysize	= 3*DES_KEY_SIZE,
			.setkey 	= ast_des_setkey,
			.encrypt	= ast_tdes_ctr_encrypt,
			.decrypt	= ast_tdes_ctr_decrypt,
		},
	},		
	{
		.cra_name		= "ecb(arc4)",
		.cra_driver_name	= "ast-ecb-arc4",
		.cra_priority		= 300,
		.cra_flags		= CRYPTO_ALG_TYPE_ABLKCIPHER | 
							CRYPTO_ALG_KERN_DRIVER_ONLY | 
							CRYPTO_ALG_ASYNC,
		.cra_blocksize		= 1,
		.cra_ctxsize		= sizeof(struct ast_rc4_ctx),
		.cra_alignmask	= 0xf,
		.cra_type		= &crypto_ablkcipher_type,
		.cra_module 		= THIS_MODULE,
		.cra_init		= ast_crypto_cra_init,
		.cra_exit		= ast_crypto_cra_exit,
		.cra_u.ablkcipher = {
			.min_keysize	= 1,
			.max_keysize	= 256,
			.setkey 	= ast_rc4_setkey,
			.encrypt	= ast_rc4_encrypt,
			.decrypt	= ast_rc4_decrypt,
		},
	},				
};
static struct ahash_alg ast_ahash_alg[] = {	
#if 1
	{
		.init			= ast_sham_init,
		.update 		= ast_sha_update,
		.final		= ast_sha_final,
		.finup		= ast_sha_finup,
		.digest 		= ast_sha_digest,
		.halg = {
			.digestsize = MD5_DIGEST_SIZE,
			.statesize	= sizeof(struct ast_sham_reqctx),
			.base	= {
				.cra_name		= "md5",
				.cra_driver_name	= "ast-md5",
				.cra_priority		= 300,
				.cra_flags		= CRYPTO_ALG_TYPE_AHASH | 
							CRYPTO_ALG_ASYNC |
							CRYPTO_ALG_NEED_FALLBACK |
							CRYPTO_ALG_KERN_DRIVER_ONLY,
				.cra_blocksize		= MD5_HMAC_BLOCK_SIZE,
				.cra_ctxsize		= sizeof(struct ast_sham_ctx),
				.cra_alignmask	= 0,
				.cra_module 		= THIS_MODULE,
				.cra_init			= ast_sham_cra_init,
				.cra_exit			= ast_sham_cra_exit,
			}
		}
	},
#endif	
	{
		.init			= ast_sham_init,
		.update 		= ast_sha_update,
		.final		= ast_sha_final,
		.finup		= ast_sha_finup,
		.digest 		= ast_sha_digest,
		.halg = {
			.digestsize = SHA1_DIGEST_SIZE,
			.statesize = sizeof(struct ast_sham_reqctx),
			.base	= {
				.cra_name		= "sha1",
				.cra_driver_name	= "ast-sha1",
				.cra_priority		= 300,
				.cra_flags		= CRYPTO_ALG_TYPE_AHASH |
							CRYPTO_ALG_ASYNC |
							CRYPTO_ALG_NEED_FALLBACK |
							CRYPTO_ALG_KERN_DRIVER_ONLY,
				.cra_blocksize		= SHA1_BLOCK_SIZE,
				.cra_ctxsize		= sizeof(struct ast_sham_ctx),
				.cra_alignmask	= 0,
				.cra_module 		= THIS_MODULE,
				.cra_init			= ast_sham_cra_init,
				.cra_exit			= ast_sham_cra_exit,				
			}
		}
	},
	{
		.init			= ast_sham_init,
		.update 		= ast_sha_update,
		.final		= ast_sha_final,
		.finup		= ast_sha_finup,
		.digest 		= ast_sha_digest,
		.halg = {
			.digestsize = SHA256_DIGEST_SIZE,
			.statesize = sizeof(struct ast_sham_reqctx),
			.base	= {
				.cra_name		= "sha256",
				.cra_driver_name	= "ast-sha256",
				.cra_priority		= 300,
				.cra_flags		= CRYPTO_ALG_TYPE_AHASH |
							CRYPTO_ALG_ASYNC |
							CRYPTO_ALG_NEED_FALLBACK |
							CRYPTO_ALG_KERN_DRIVER_ONLY,
				.cra_blocksize		= SHA256_BLOCK_SIZE,
				.cra_ctxsize		= sizeof(struct ast_sham_ctx),
				.cra_alignmask	= 0,
				.cra_module 		= THIS_MODULE,
				.cra_init			= ast_sham_cra_init,
				.cra_exit			= ast_sham_cra_exit,				
			}
		}
	},
	{
		.init			= ast_sham_init,
		.update		= ast_sha_update,
		.final		= ast_sha_final,
		.finup		= ast_sha_finup,
		.digest		= ast_sha_digest,
		.halg = {
			.digestsize	= SHA224_DIGEST_SIZE,
			.statesize = sizeof(struct ast_sham_reqctx),
			.base	= {
				.cra_name		= "sha224",
				.cra_driver_name	= "ast-sha224",
				.cra_priority		= 300,
				.cra_flags		= CRYPTO_ALG_TYPE_AHASH |
							CRYPTO_ALG_ASYNC |
							CRYPTO_ALG_NEED_FALLBACK |
							CRYPTO_ALG_KERN_DRIVER_ONLY,
				.cra_blocksize		= SHA224_BLOCK_SIZE,
				.cra_ctxsize		= sizeof(struct ast_sham_ctx),
				.cra_alignmask	= 0,
				.cra_module		= THIS_MODULE,
				.cra_init			= ast_sham_cra_init,
				.cra_exit			= ast_sham_cra_exit,				
			}
		}
	},
	{
		.init			= ast_sham_init,
		.update		= ast_sha_update,
		.final		= ast_sha_final,
		.finup		= ast_sha_finup,
		.digest		= ast_sha_digest,
		.setkey		= ast_sham_setkey,
		.halg = {
			.digestsize	= MD5_DIGEST_SIZE,
			.statesize = sizeof(struct ast_sham_reqctx),
			.base	= {
				.cra_name		= "hmac(md5)",
				.cra_driver_name	= "ast-hmac-md5",
				.cra_priority		= 300,
				.cra_flags		= CRYPTO_ALG_TYPE_AHASH |
								CRYPTO_ALG_ASYNC |
								CRYPTO_ALG_NEED_FALLBACK |
								CRYPTO_ALG_KERN_DRIVER_ONLY,
				.cra_blocksize		= MD5_HMAC_BLOCK_SIZE,
				.cra_ctxsize		= sizeof(struct ast_sham_ctx),
				.cra_alignmask	= 0,
				.cra_module		= THIS_MODULE,
				.cra_init		= ast_sha_cra_md5_init,
				.cra_exit		= ast_sham_cra_exit,
			}
		}
	},	
	{
		.init			= ast_sham_init,
		.update		= ast_sha_update,
		.final		= ast_sha_final,
		.finup		= ast_sha_finup,
		.digest		= ast_sha_digest,
		.setkey		= ast_sham_setkey,
		.halg = {
			.digestsize	= SHA1_DIGEST_SIZE,
			.statesize = sizeof(struct ast_sham_reqctx),
			.base	= {
				.cra_name		= "hmac(sha1)",
				.cra_driver_name	= "ast-hmac-sha1",
				.cra_priority		= 300,
				.cra_flags		= CRYPTO_ALG_TYPE_AHASH |
								CRYPTO_ALG_ASYNC |
								CRYPTO_ALG_NEED_FALLBACK |
								CRYPTO_ALG_KERN_DRIVER_ONLY,
				.cra_blocksize		= SHA1_BLOCK_SIZE,
				.cra_ctxsize		= sizeof(struct ast_sham_ctx),
				.cra_alignmask	= 0,
				.cra_module		= THIS_MODULE,
				.cra_init		= ast_sha_cra_sha1_init,
				.cra_exit		= ast_sham_cra_exit,
			}
		}
	},
	{
		.init			= ast_sham_init,
		.update 		= ast_sha_update,
		.final		= ast_sha_final,
		.finup		= ast_sha_finup,
		.digest 		= ast_sha_digest,
		.setkey 		= ast_sham_setkey,
		.halg = {
			.digestsize	= SHA224_DIGEST_SIZE,
			.statesize = sizeof(struct ast_sham_reqctx),
			.base	= {
				.cra_name		= "hmac(sha224)",
				.cra_driver_name	= "ast-hmac-sha224",
				.cra_priority		= 300,
				.cra_flags		= CRYPTO_ALG_TYPE_AHASH |
								CRYPTO_ALG_ASYNC |
								CRYPTO_ALG_NEED_FALLBACK |
								CRYPTO_ALG_KERN_DRIVER_ONLY,
				.cra_blocksize		= SHA224_BLOCK_SIZE,
				.cra_ctxsize		= sizeof(struct ast_sham_ctx),
				.cra_alignmask	= 0,
				.cra_module 	= THIS_MODULE,
				.cra_init		= ast_sha_cra_sha224_init,
				.cra_exit		= ast_sham_cra_exit,
			}
		}
	},
	{
		.init			= ast_sham_init,
		.update 		= ast_sha_update,
		.final		= ast_sha_final,
		.finup		= ast_sha_finup,
		.digest 		= ast_sha_digest,
		.setkey 		= ast_sham_setkey,
		.halg = {
			.digestsize	= SHA256_DIGEST_SIZE,
			.statesize = sizeof(struct ast_sham_reqctx),
			.base	= {
				.cra_name		= "hmac(sha256)",
				.cra_driver_name	= "ast-hmac-sha256",
				.cra_priority		= 300,
				.cra_flags		= CRYPTO_ALG_TYPE_AHASH |
								CRYPTO_ALG_ASYNC |
								CRYPTO_ALG_NEED_FALLBACK |
								CRYPTO_ALG_KERN_DRIVER_ONLY,
				.cra_blocksize		= SHA256_BLOCK_SIZE,
				.cra_ctxsize		= sizeof(struct ast_sham_ctx),
				.cra_alignmask	= 0,
				.cra_module 	= THIS_MODULE,
				.cra_init		= ast_sha_cra_sha256_init,
				.cra_exit		= ast_sham_cra_exit,
			}
		}
	},
};

/*************************************************************************************/
#ifdef AST_CRYPTO_IRQ
static irqreturn_t ast_crypto_irq(int irq, void *dev)
{
	struct ast_crypto_dev *ast_crypto = (struct ast_crypto_dev *)dev;
	ast_crypto->isr = ast_crypto_read(ast_crypto, AST_HACE_STS);

	CRYPTO_DBUG("sts %x \n",ast_crypto->isr);
//	CDBUG("irq %x\n", ast_crypto->isr);
	if(ast_crypto->isr == 0)
		printk("sts %x \n",ast_crypto->isr);
	ast_crypto_write(ast_crypto, ast_crypto->isr, AST_HACE_STS);
	complete(&ast_crypto->cmd_complete);
	return IRQ_HANDLED;
}
#endif

static int ast_crypto_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;	
	struct resource *res;	
	int i, err, ret = -EINVAL;
	int irq;
	struct ast_crypto_dev *crypto_dev;

	crypto_dev = devm_kzalloc(&pdev->dev, sizeof(struct ast_crypto_dev), GFP_KERNEL);
	if (!crypto_dev) {
		dev_err(dev, "unable to alloc data struct.\n");
		return -ENOMEM;
	}
	
	crypto_dev->dev = dev;

	platform_set_drvdata(pdev, crypto_dev);

	INIT_LIST_HEAD(&crypto_dev->list);
	spin_lock_init(&crypto_dev->lock);

	tasklet_init(&crypto_dev->done_task, ast_aes_done_task,
					(unsigned long)crypto_dev);
	tasklet_init(&crypto_dev->queue_task, ast_aes_queue_task,
					(unsigned long)crypto_dev);

	crypto_init_queue(&crypto_dev->queue, AST_AES_QUEUE_LENGTH);	

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
	irq = platform_get_irq(pdev, 0);
	if (!irq) {
		dev_err(&pdev->dev, "no memory/irq resource for crypto_dev\n");
		return -ENXIO;
	}

	crypto_dev->reset = devm_reset_control_get_exclusive(&pdev->dev, "crypto");
	//crypto_dev->reset = devm_reset_control_get(&pdev->dev, "crypto");
	if (IS_ERR(crypto_dev->reset)) {
		dev_err(&pdev->dev, "can't get crypto reset\n");
		return PTR_ERR(crypto_dev->reset);
	}

	crypto_dev->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(crypto_dev->clk)) {
		dev_err(&pdev->dev, "no clock defined\n");
		return -ENODEV;
	}

	//scu init
	reset_control_assert(crypto_dev->reset);
	clk_prepare_enable(crypto_dev->clk);
	mdelay(1);
	reset_control_deassert(crypto_dev->reset);
	
#ifdef AST_CRYPTO_IRQ
	if (devm_request_irq(&pdev->dev, irq, ast_crypto_irq, 0, dev_name(&pdev->dev), crypto_dev)) {
		dev_err(dev, "unable to request aes irq.\n");
		return -EBUSY;
	}
#endif 

	spin_lock(&ast_drv.lock);
	list_add_tail(&crypto_dev->list, &ast_drv.dev_list);
	spin_unlock(&ast_drv.lock);

	crypto_dev->crypto_algs		= &ast_crypto_algs;
	crypto_dev->ahash_algs		= &ast_ahash_alg;

	// 8-byte aligned
	crypto_dev->ctx_buf = dma_alloc_coherent(&pdev->dev,
					 0x8000,
					 &crypto_dev->ctx_dma_addr, GFP_KERNEL);

	if(! crypto_dev->ctx_buf) {
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

	crypto_dev->hash_src = crypto_dev->hash_key + 0x1000;
	crypto_dev->hash_src_dma = crypto_dev->hash_key_dma + 0x1000;

	crypto_dev->hash_digst = crypto_dev->hash_src + AST_HASH_BUFF_SIZE;
	crypto_dev->hash_digst_dma = crypto_dev->hash_src_dma + AST_HASH_BUFF_SIZE;

//	printk("Crypto ctx %x , in : %x, out: %x\n", crypto_dev->ctx_dma_addr, crypto_dev->dma_addr_in, crypto_dev->dma_addr_out);

//	printk("Hash key %x , src : %x, digst: %x\n", crypto_dev->hash_key_dma, crypto_dev->hash_src_dma, crypto_dev->hash_digst_dma);

	///Ctrl init 
	ast_crypto_write(crypto_dev, crypto_dev->ctx_dma_addr, AST_HACE_CONTEXT);

	////////////////////////	
	for (i = 0; i < ARRAY_SIZE(ast_crypto_algs); i++) {
		err = crypto_register_alg(&ast_crypto_algs[i]);
		if(err)
			printk("ast_crypto_algs ~~~ ERROR ~~~\n");
	}

	for (i = 0; i < ARRAY_SIZE(ast_ahash_alg); i++) {
		err = crypto_register_ahash(&ast_ahash_alg[i]);
		if(err)
			printk("ast_ahash_alg~~~ ERROR ~~~\n");
	}	

	printk(KERN_INFO "AST crypto driver successfully loaded \n");

	return 0;
}

static int ast_crypto_remove(struct platform_device *pdev)
{
	struct ast_crypto_alg *alg, *next;
	struct ast_crypto_dev *crypto_dev = platform_get_drvdata(pdev);


//	clk_disable(crypto_dev->clk);
//	clk_put(crypto_dev->clk);

	return 0;
}

#ifdef CONFIG_PM
static int ast_crypto_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct ast_crypto_dev *crypto_dev = platform_get_drvdata(pdev);

	/*
	 * We only support standby mode. All we have to do is gate the clock to
	 * the spacc. The hardware will preserve state until we turn it back
	 * on again.
	 */
//	clk_disable(crypto_dev->clk);

	return 0;
}

static int ast_crypto_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct ast_crypto_dev *crypto_dev = platform_get_drvdata(pdev);
	return 0;
//	return clk_enable(crypto_dev->clk);
}

#endif /* CONFIG_PM */



static const struct of_device_id ast_crypto_of_matches[] = {
	{ .compatible = "aspeed,ast-crypto", },
	{},
};

MODULE_DEVICE_TABLE(of, ast_crypto_of_matches);

static struct platform_driver ast_crypto_driver = {
	.probe 		= ast_crypto_probe,
	.remove		= ast_crypto_remove,
#ifdef CONFIG_PM	
	.suspend	= ast_crypto_suspend,
	.resume 	= ast_crypto_resume,
#endif	
	.driver         = {
		.name   = KBUILD_MODNAME,
		.of_match_table = ast_crypto_of_matches,
	},
};

module_platform_driver(ast_crypto_driver);

MODULE_AUTHOR("Ryan Chen <ryan_chen@aspeedtech.com>");
MODULE_DESCRIPTION("AST Crypto driver");
MODULE_LICENSE("GPL");

MODULE_AUTHOR("Ryan Chen <ryan_chen@aspeedtech.com>");
MODULE_DESCRIPTION("AST Crypto driver");
MODULE_LICENSE("GPL");
