#ifndef __ASPEED_CRYPTO_H__
#define __ASPEED_CRYPTO_H__

#include <crypto/aes.h>
#include <crypto/des.h>
#include <crypto/algapi.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <crypto/internal/hash.h>

#include <crypto/md5.h>
#include <crypto/sha.h>

/* Crypto control registers*/
#define ASPEED_HACE_SRC			0x00
#define ASPEED_HACE_DEST		0x04
#define ASPEED_HACE_CONTEXT		0x08	/* 8 byte aligned*/
#define ASPEED_HACE_DATA_LEN		0x0C
#define ASPEED_HACE_CMD			0x10
#define  HACE_CMD_SINGLE_DES	0
#define  HACE_CMD_TRIPLE_DES	BIT(17)
#define  HACE_CMD_AES_SELECT	0
#define  HACE_CMD_DES_SELECT	BIT(16)
#define  HACE_CMD_ISR_EN		BIT(12)
#define  HACE_CMD_RI_WO_DATA_ENABLE		(0)
#define  HACE_CMD_RI_WO_DATA_DISABLE	BIT(11)
#define  HACE_CMD_CONTEXT_LOAD_ENABLE	(0)
#define  HACE_CMD_CONTEXT_LOAD_DISABLE	BIT(10)
#define  HACE_CMD_CONTEXT_SAVE_ENABLE	(0)
#define  HACE_CMD_CONTEXT_SAVE_DISABLE	BIT(9)
#define  HACE_CMD_AES					(0)
#define  HACE_CMD_DES					(0)
#define  HACE_CMD_RC4					BIT(8)
#define  HACE_CMD_DECRYPT				(0)
#define  HACE_CMD_ENCRYPT				BIT(7)
#define  HACE_CMD_ECB					(0)
#define  HACE_CMD_CBC					(0x1 << 4)
#define  HACE_CMD_CFB					(0x1 << 5)
#define  HACE_CMD_OFB					(0x3 << 4)
#define  HACE_CMD_CTR					(0x1 << 6)
#define  HACE_CMD_AES128				(0)
#define  HACE_CMD_AES192				(0x1 << 2)
#define  HACE_CMD_AES256				(0x1 << 3)
#define  HACE_CMD_OP_CASCADE			(0x3)
#define  HACE_CMD_OP_INDEPENDENT		(0x1)
#define ASPEED_HACE_TAG			0x18
#define ASPEED_HACE_STS			0x1C
#define  HACE_RSA_ISR			BIT(13)
#define  HACE_CRYPTO_ISR		BIT(12)
#define  HACE_HASH_ISR			BIT(9)

#define  HACE_RSA_BUSY			BIT(2)
#define  HACE_CRYPTO_BUSY		BIT(1)
#define  HACE_HASH_BUSY			BIT(0)
#define ASPEED_HACE_HASH_SRC		0x20
#define ASPEED_HACE_HASH_DIGEST_BUFF	0x24
#define ASPEED_HACE_HASH_KEY_BUFF	0x28	/* 64 byte aligned*/
#define ASPEED_HACE_HASH_DATA_LEN	0x2C
#define ASPEED_HACE_HASH_CMD		0x30
#define  HASH_CMD_INT_ENABLE		BIT(9)
#define  HASH_CMD_INT_DISABLE		(0)
#define  HASH_CMD_HMAC				BIT(7)
#define  HASH_CMD_MD5				(0)
#define  HASH_CMD_SHA1				(0x2 << 4)
#define  HASH_CMD_SHA224			(0x4 << 4)
#define  HASH_CMD_SHA256			(0x5 << 4)
#define  HASH_CMD_MD5_SWAP				(0x1 << 2)
#define  HASH_CMD_SHA_SWAP				(0x1 << 3)
#define  HASH_CMD_CASCADED_CRYPTO_FIRST	(2)
#define  HASH_CMD_CASCADED_HASH_FIRST	(3)
#define ASPEED_HACE_RSA_MD_EXP_BIT	0x40
#define ASPEED_HACE_RSA_CMD		0x4C
#define ASPEED_HACE_CMD_QUEUE		0x50
#define ASPEED_HACE_CMD_QUEUE_EP	0x54
#define ASPEED_HACE_CMD_QUEUE_WP	0x58
#define ASPEED_HACE_CMD_QUEUE_RP	0x5C
#define ASPEED_HACE_ENG_FEATURE		0x60

/*
 * Asynchronous crypto request structure.
 *
 * This structure defines a request that is either queued for processing or
 * being processed.
 */

struct aspeed_cipher_ctx {
	struct aspeed_crypto_dev *crypto_dev;
	u8			*iv;
	int 		key_len;
	int 		enc_cmd;
	union {
		u8		aes[AES_MAX_KEY_SIZE];
		u8		des[DES_KEY_SIZE];
		u8		des3[3 * DES_KEY_SIZE];
		u8		arc4[256]; /* S-box, X, Y */
	} key;
};

struct aspeed_crypto_dev {
	void __iomem			*regs;

	int 					irq;
	struct clk 			*yclk;

	spinlock_t			lock;

	//hash
	struct aspeed_sham_reqctx *sham_reqctx;

	struct crypto_queue	queue;

	struct tasklet_struct	crypto_tasklet;

	unsigned long			flags;

	size_t	total;

	struct ablkcipher_request	*ablk_req;
	struct ahash_request		*ahash_req;


//	struct crypto_alg		*crypto_algs;
//	struct ahash_alg		*ahash_algs;
//	struct akcipher_alg 	*akcipher_alg;	//for rsa 

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

	void	*hmac_key;
	dma_addr_t	hmac_key_dma;

	void	*hash_src;
	dma_addr_t	hash_src_dma;

	void	*hash_digst;
	dma_addr_t	hash_digst_dma;


};

struct aspeed_crypto_alg {
	struct aspeed_crypto_dev	*crypto_dev;
	union {
		struct crypto_alg	crypto;
		struct ahash_alg	ahash;
	} alg;
};

/* the private variable of hash */
struct aspeed_ahash_ctx {
	struct aspeed_crypto_dev	*crypto_dev;
	u32 		ahash_cmd;
	size_t		digcnt;
	unsigned int	total;	/* total request */	
	struct scatterlist	*sg;	
	unsigned long	flags;	
	size_t	bufcnt;	
	/* for fallback */
	struct crypto_ahash		*fallback_tfm;
//	struct crypto_shash 	*base_hash;		//for hmac
};

/* the privete variable of hash for fallback */
struct aspeed_ahash_rctx {
	struct ahash_request		fallback_req;
};

/*************************************************************************************/
struct aspeed_sham_ctx {
	struct aspeed_crypto_dev	*crypto_dev;

	unsigned long		flags;	//hmac flag

	/* fallback stuff */
	struct crypto_shash	*fallback;
	struct crypto_shash 	*base_hash;		//for hmac
};

struct aspeed_sham_reqctx {
	struct aspeed_crypto_dev	*crypto_dev;
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

static inline void
aspeed_crypto_write(struct aspeed_crypto_dev *crypto, u32 val, u32 reg)
{
//	printk("write : val: %x , reg : %x \n",val,reg);
	writel(val, crypto->regs + reg);
}

static inline u32
aspeed_crypto_read(struct aspeed_crypto_dev *crypto, u32 reg)
{
#if 0
	u32 val = readl(crypto->regs + reg);
	printk("R : reg %x , val: %x \n", reg, val);
	return val;
#else
	return readl(crypto->regs + reg);
#endif
}

#define ASPEED_HASH_BUFF_SIZE 	8192

extern int aspeed_crypto_ahash_trigger(struct aspeed_crypto_dev *aspeed_crypto);

extern int aspeed_crypto_ablkcipher_trigger(struct aspeed_crypto_dev *aspeed_crypto);
extern int aspeed_hash_trigger(struct aspeed_crypto_dev *aspeed_crypto);
extern int aspeed_hash_handle_queue(struct aspeed_crypto_dev *aspeed_crypto, struct ahash_request *req);

extern int aspeed_register_crypto_algs(struct aspeed_crypto_dev *crypto_dev);
extern int aspeed_register_ahash_algs(struct aspeed_crypto_dev *crypto_dev);

extern int aspeed_crypto_enqueue(struct aspeed_crypto_dev *aspeed_crypto, struct ablkcipher_request *req);

#endif
