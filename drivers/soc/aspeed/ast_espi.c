/*
 * ast-espi.c - eSPI driver for the Aspeed SoC
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
#include <linux/sched.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/dma-mapping.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <mach/hardware.h>
#include <mach/gpio.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/reset.h>

#include <linux/irqdomain.h>

/*************************************************************************************/
#define AST_ESPI_CTRL			0x00		/* Engine Control */
#define AST_ESPI_STS				0x04		/* Engine Status */
#define AST_ESPI_ISR				0x08		/* Interrupt Status */
#define AST_ESPI_IER				0x0C		/* Interrupt Enable */
#define AST_ESPI_PCP_RX_DMA		0x10		/* DMA Address of Peripheral Channel Posted Rx Package */
#define AST_ESPI_PCP_RX_CTRL	0x14		/* Control of Peripheral Channel Posted Rx Package */
#define AST_ESPI_PCP_RX_DATA	0x18		/* Data Port of Peripheral Channel Posted Rx Package */
#define AST_ESPI_PCP_TX_DMA		0x20		/* DMA Address of Peripheral Channel Posted Tx Package */
#define AST_ESPI_PCP_TX_CTRL	0x24		/* Control of Peripheral Channel Posted Tx Package */
#define AST_ESPI_PCP_TX_DATA	0x28		/* Data Port of Peripheral Channel Posted Tx Package */
#define AST_ESPI_PCNP_TX_DMA	0x30		/* DMA Address of Peripheral Channel Non-Posted Tx Package */
#define AST_ESPI_PCNP_TX_CTRL	0x34		/* Control of Peripheral Channel Non-Posted Tx Package */
#define AST_ESPI_PCNP_TX_DATA	0x38		/* Data Port of Peripheral Channel Non-Posted Tx Package */
#define AST_ESPI_OOB_RX_DMA	0x40		/* DMA Address of OOB Channel Rx Package */
#define AST_ESPI_OOB_RX_CTRL	0x44		/* Control of OOB Channel Rx Package */
#define AST_ESPI_OOB_RX_DATA	0x48		/* Date port of OOB Channel Rx Package */
#define AST_ESPI_OOB_TX_DMA	0x50		/* DMA Address of OOB Channel Tx Package */
#define AST_ESPI_OOB_TX_CTRL	0x54		/* Control of OOB Channel Tx Package */
#define AST_ESPI_OOB_TX_DATA	0x58		/* Date port of OOB Channel Tx Package */
#define AST_ESPI_FLASH_RX_DMA	0x60		/* DMA Address of Flash Channel Rx Package */
#define AST_ESPI_FLASH_RX_CTRL	0x64		/* Control of Flash Channel Rx Package */
#define AST_ESPI_FLASH_RX_DATA	0x68		/* Date port of Flash Channel Rx Package */
#define AST_ESPI_FLASH_TX_DMA	0x70		/* DMA Address of Flash Channel Tx Package */
#define AST_ESPI_FLASH_TX_CTRL	0x74		/* Control of Flash Channel Tx Package */
#define AST_ESPI_FLASH_TX_DATA	0x78		/* Date port of Flash Channel Tx Package */

#define AST_ESPI_PC_RX_SADDR	0x84		/* Mapping Source Address of Peripheral Channel Rx Package */
#define AST_ESPI_PC_RX_TADDR	0x88		/* Mapping Target Address of Peripheral Channel Rx Package */
#define AST_ESPI_PC_RX_TADDRM	0x8c		/* Mapping Target Address Mask of Peripheral Channel Rx Package */
#define AST_ESPI_FLASH_TADDRM	0x90		/* Mapping Target Address Mask of Flash Channel */
#define AST_ESPI_SYS_IER			0x94		/* Interrupt enable of System Event from Master */
#define AST_ESPI_SYS_EVENT		0x98		/* System Event from and to Master */
#define AST_ESPI_GPIO_VIRTCH	0x9C		/* GPIO through Virtual Wire Cahnnel  */
#define AST_ESPI_GCAP_CONFIG	0xA0		/* General Capabilities and Configuration  */
#define AST_ESPI_CH0CAP_CONFIG	0xA4		/* Channel 0 Capabilities and Configuration  */
#define AST_ESPI_CH1CAP_CONFIG	0xA8		/* Channel 1 Capabilities and Configuration  */
#define AST_ESPI_CH2CAP_CONFIG	0xAC		/* Channel 2 Capabilities and Configuration  */
#define AST_ESPI_CH3CAP_CONFIG	0xB0		/* Channel 3 Capabilities and Configuration  */

#define AST_ESPI_GPIO_DIR_VIRTCH	0xB4		/* GPIO Direction of Virtual Wire Channel  */
#define AST_ESPI_GPIO_SEL_VIRTCH	0xB8		/* GPIO Selection of Virtual Wire Channel  */
#define AST_ESPI_GPIO_REST_VIRTCH	0xBC		/* GPIO Reset Selection of Virtual Wire Channel  */

#define AST_ESPI_SYS1_IER			0x100		/* Interrupt enable of System Event from Master */
#define AST_ESPI_SYS1_EVENT			0x104		/* Interrupt enable of System Event from Master */

#define AST_ESPI_SYS_INT_T0		0x110
#define AST_ESPI_SYS_INT_T1		0x114
#define AST_ESPI_SYS_INT_T2		0x118
#define AST_ESPI_SYS_EVENT_ISR		0x11C


#define AST_ESPI_SYS1_INT_T0		0x120
#define AST_ESPI_SYS1_INT_T1		0x124
#define AST_ESPI_SYS1_INT_T2		0x128
#define AST_ESPI_SYS1_INT_STS		0x12C

/* AST_ESPI_CTRL	-	0x00	:Engine Control */
#define ESPI_CTRL_FLASH_TX_SW_RESET	(0x1 << 31)
#define ESPI_CTRL_FLASH_RX_SW_RESET	(0x1 << 30)
#define ESPI_CTRL_OOB_TX_SW_RESET		(0x1 << 29)
#define ESPI_CTRL_OOB_RX_SW_RESET		(0x1 << 28)
#define ESPI_CTRL_PCNP_TX_SW_RESET		(0x1 << 27)
#define ESPI_CTRL_PCNP_RX_SW_RESET		(0x1 << 26)
#define ESPI_CTRL_PCP_TX_SW_RESET		(0x1 << 25)
#define ESPI_CTRL_PCP_RX_SW_RESET		(0x1 << 24)
#define ESPI_CTRL_FLASH_TX_DMA			(0x1 << 23)
#define ESPI_CTRL_FLASH_RX_DMA			(0x1 << 22)
#define ESPI_CTRL_OOB_TX_DMA			(0x1 << 21)
#define ESPI_CTRL_OOB_RX_DMA			(0x1 << 20)
#define ESPI_CTRL_PCNP_TX_DMA			(0x1 << 19)
/* */
#define ESPI_CTRL_PCP_TX_DMA			(0x1 << 17)
#define ESPI_CTRL_PCP_RX_DMA			(0x1 << 16)
/* */
#define ESPI_CTRL_DIR_RESET				(0x1 << 13)
#define ESPI_CTRL_VAL_RESET				(0x1 << 12)
#define ESPI_CTRL_SW_FLASH_READ		(0x1 << 10)
#define ESPI_CTRL_SW_GPIO_VIRTCH		(0x1 << 9)


/* AST_ESPI_ISR	- 0x08 : Interrupt Status */
#define ESPI_ISR_HW_RESET				(0x1 << 31)
/* */
#define ESPI_ISR_VIRTW_SYS1				(0x1 << 22)
#define ESPI_ISR_FLASH_TX_ERR			(0x1 << 21)
#define ESPI_ISR_OOB_TX_ERR				(0x1 << 20)
#define ESPI_ISR_FLASH_TX_ABORT			(0x1 << 19)
#define ESPI_ISR_OOB_TX_ABORT			(0x1 << 18)
#define ESPI_ISR_PCNP_TX_ABORT			(0x1 << 17)
#define ESPI_ISR_PCP_TX_ABORT			(0x1 << 16)
#define ESPI_ISR_FLASH_RX_ABORT			(0x1 << 15)
#define ESPI_ISR_OOB_RX_ABORT			(0x1 << 14)
#define ESPI_ISR_PCNP_RX_ABORT			(0x1 << 13)
#define ESPI_ISR_PCP_RX_ABORT			(0x1 << 12)
#define ESPI_ISR_PCNP_TX_ERR			(0x1 << 11)
#define ESPI_ISR_PCP_TX_ERR				(0x1 << 10)
#define ESPI_ISR_VIRTW_GPIO				(0x1 << 9)
#define ESPI_ISR_VIRTW_SYS				(0x1 << 8)
#define ESPI_ISR_FLASH_TX_COMP			(0x1 << 7)
#define ESPI_ISR_FLASH_RX_COMP			(0x1 << 6)
#define ESPI_ISR_OOB_TX_COMP			(0x1 << 5)
#define ESPI_ISR_OOB_RX_COMP			(0x1 << 4)
#define ESPI_ISR_PCNP_TX_COMP			(0x1 << 3)
/* */
#define ESPI_ISR_PCP_TX_COMP			(0x1 << 1)
#define ESPI_ISR_PCP_RX_COMP			(0x1)


/* AST_ESPI_PCP_RX_CTRL	-0x14	:	Control of Peripheral Channel Posted Rx Package */
#define ESPI_TRIGGER_PACKAGE			(0x1 << 31)

#define ESPI_GET_CYCLE_TYPE(x)			(x & 0xff)
#define ESPI_GET_TAG(x)					((x >> 8) & 0xf)
#define ESPI_GET_LEN(x)					((x >> 12) & 0xfff)


/* Cycle Type Define	*/

/* eSPI Peripheral Channel	*/
#define MEM_READ32				0x00
#define MEM_WRITE32				0x01
#define MEM_READ64				0x02
#define MEM_WRITE64				0x03

#define MESSAGE_TRCV			0x10
#define MESSAGED_TRCV			0x11

#define SUCCESS_M_COMPLETE		0x09
#define SUCCESS_F_COMPLETE		0x0B
#define SUCCESS_L_COMPLETE		0x0D
#define SUCCESS_O_COMPLETE		0x0F

#define USUCCESS_L_COMPLETE	0x0C
#define USUCCESS_O_COMPLETE	0x0E

#define FLASH_READ				0x00
#define FLASH_WRITE				0x01
#define FLASH_ERASE				0x02

/* OOB Message Channel	*/

/* AST_ESPI_SYS1_EVENT			0x104 : Interrupt enable of System Event from Master */
/* AST_ESPI_SYS1_INT_STS		0x12C		*/
#define ESPI_SYS_SUS_ACK		(0x1 << 20)

#define ESPI_SYS_SUS_WARN		(0x1)

/* AST_ESPI_SYS_EVENT			0x98		System Event from and to Master */
/* AST_ESPI_SYS_EVENT_ISR		0x11C	System Event from and to Master interrupt sts */

#define ESPI_HOST_REST_ACK		(0x1 << 27)

#define ESPI_REST_CPU_INIT		(0x1 << 26)

#define ESPI_BOOT_STS			(0x1 << 23)
#define ESPI_NFATEL_ERR			(0x1 << 22)
#define ESPI_FATEL_ERR			(0x1 << 21)
#define ESPI_BOOT_DWN			(0x1 << 20)
#define ESPI_OOB_REST_ACK		(0x1 << 16)

#define ESPI_HOST_NMI_OUT		(0x1 << 10)
#define ESPI_HOST_SMI_OUT		(0x1 << 9)

#define ESPI_HOST_RST_WARN		(0x1 << 8)

#define ESPI_OOB_RST_WARN		(0x1 << 6)

#define ESPI_SYS_S5_SLEEP		(0x1 << 2)
#define ESPI_SYS_S4_SLEEP		(0x1 << 1)
#define ESPI_SYS_S3_SLEEP		(0x1)

/* AST_ESPI_GCAP_CONFIG	0xA0		General Capabilities and Configuration  */
#define GET_GCAP_IO_MODE(x)		((x >> 26) & 0x3)
#define GET_GCAP_OP_FREQ(x)		((x >> 20) & 0x7)
#define GET_GCAP_CH_SUPPORT(x)	(x & 0xf)

/*************************************************************************************/
struct ast_espi_xfer {
	unsigned int header;		//[23:12] len, [11:8] tag, [7:0] cycle type
	unsigned int buf_len;		//xfer buff len
	unsigned char	*xfer_buf;
};

#define ESPIIOC_BASE       'P'

#define AST_ESPI_PERIP_IOCRX			_IOWR(ESPIIOC_BASE, 0x10, struct ast_espi_xfer)			//post rx
#define AST_ESPI_PERIP_IOCTX			_IOW(ESPIIOC_BASE, 0x11, struct ast_espi_xfer)				//post tx
#define AST_ESPI_PERINP_IOCTX		_IOW(ESPIIOC_BASE, 0x12, struct ast_espi_xfer)				//non-post tx
#define AST_ESPI_OOB_IOCRX			_IOWR(ESPIIOC_BASE, 0x13, struct ast_espi_xfer)
#define AST_ESPI_OOB_IOCTX			_IOW(ESPIIOC_BASE, 0x14, struct ast_espi_xfer)
#define AST_ESPI_FLASH_IOCRX		_IOWR(ESPIIOC_BASE, 0x15, struct ast_espi_xfer)
#define AST_ESPI_FLASH_IOCTX		_IOW(ESPIIOC_BASE, 0x16, struct ast_espi_xfer)
#define AST_ESPI_GPIO_IOCRX			_IOWR(ESPIIOC_BASE, 0x17, unsigned int)
#define AST_ESPI_SYS_IOCRX			_IOW(ESPIIOC_BASE, 0x18, unsigned int)
#define AST_ESPI_RX_IOCSTS			_IOR(ESPIIOC_BASE, 0x19, unsigned int)
/*************************************************************************************/
//#define AST_ESPI_DEBUG

#ifdef AST_ESPI_DEBUG
#define ESPI_DBUG(fmt, args...) printk(KERN_DEBUG "%s() " fmt,__FUNCTION__, ## args)
#else
#define ESPI_DBUG(fmt, args...)
#endif

//#define AST_ESPI_DMA_MODE

struct espi_ch_data {
	int 		full;
	u32		header;	//[23:12] len, [11:8] tag, [7:0] cycle type
	u32		buf_len;
	u8		*buff;
#ifdef AST_ESPI_DMA_MODE
	dma_addr_t dma_addr;
#endif
};

struct ast_espi_data {
	struct platform_device 	*pdev;
	void __iomem			*reg_base;			/* virtual */
	int 					irq;					//LPC IRQ number
	struct reset_control 	*reset;
	u32 					irq_sts;
	u32 					vw_gpio;
	u32					sys_event;
	bool 					is_open;
	struct espi_ch_data		p_rx_channel;
	struct espi_ch_data		p_tx_channel;
	struct espi_ch_data		np_tx_channel;
	struct espi_ch_data		oob_rx_channel;
	struct espi_ch_data		oob_tx_channel;
	struct espi_ch_data		flash_rx_channel;
	struct espi_ch_data		flash_tx_channel;
	struct fasync_struct 	*async_queue;
};

/******************************************************************************/
static DEFINE_SPINLOCK(espi_state_lock);
/******************************************************************************/

static inline u32
ast_espi_read(struct ast_espi_data *ast_espi, u32 reg)
{
#if 0
	u32 val;
	val = readl(ast_espi->reg_base + reg);
	ESPI_DBUG("ast_espi_read : reg = 0x%08x, val = 0x%08x\n", reg, val);
	return val;
#else
	return readl(ast_espi->reg_base + reg);
#endif
}

static inline void
ast_espi_write(struct ast_espi_data *ast_espi, u32 val, u32 reg)
{
//	ESPI_DBUG("ast_espi_write : reg = 0x%08x, val = 0x%08x\n", reg, val);
	writel(val, ast_espi->reg_base + reg);
}

/******************************************************************************/
#ifdef CONFIG_COLDFIRE_ESPI
#else
static void
ast_espi_flash_tx(struct ast_espi_data *ast_espi)
{
	int i = 0;
	ESPI_DBUG("\n");
#ifdef AST_ESPI_DMA_MODE

#else
	for (i = 0; i < ast_espi->flash_tx_channel.buf_len; i++)
		ast_espi_write(ast_espi, ast_espi->flash_tx_channel.buff[i] , AST_ESPI_FLASH_TX_DATA);

#endif
	ast_espi_write(ast_espi, ESPI_TRIGGER_PACKAGE | ast_espi->flash_tx_channel.header, AST_ESPI_FLASH_TX_CTRL);
}

static void
ast_espi_flash_rx(struct ast_espi_data *ast_espi)
{
	int i = 0;
	struct espi_ch_data	 *flash_rx = &ast_espi->flash_rx_channel;
	u32 ctrl = ast_espi_read(ast_espi, AST_ESPI_FLASH_RX_CTRL);
	ESPI_DBUG("cycle type = %x , tag = %x, len = %d byte \n", ESPI_GET_CYCLE_TYPE(ctrl), ESPI_GET_TAG(ctrl), ESPI_GET_LEN(ctrl));

	flash_rx->full = 1;
	flash_rx->header = ctrl;
	if ((ESPI_GET_CYCLE_TYPE(ctrl) == 0x00) || (ESPI_GET_CYCLE_TYPE(ctrl) == 0x02))
		flash_rx->buf_len = 4;
	else if (ESPI_GET_CYCLE_TYPE(ctrl) == 0x01)
		flash_rx->buf_len = ESPI_GET_LEN(ctrl) + 4;
	else if ((ESPI_GET_CYCLE_TYPE(ctrl) & 0x09) == 0x09)
		flash_rx->buf_len = ESPI_GET_LEN(ctrl);
	else
		flash_rx->buf_len = 0;

#ifdef AST_ESPI_DMA_MODE

#else
	for (i = 0; i < flash_rx->buf_len; i++)
		flash_rx->buff[i] = ast_espi_read(ast_espi, AST_ESPI_FLASH_RX_DATA);
#endif
}
#endif
static void
ast_espi_oob_rx(struct ast_espi_data *ast_espi)
{
	int i = 0;
	u32 ctrl = ast_espi_read(ast_espi, AST_ESPI_OOB_RX_CTRL);
	ESPI_DBUG("cycle type = %x , tag = %x, len = %d byte \n", ESPI_GET_CYCLE_TYPE(ctrl), ESPI_GET_TAG(ctrl), ESPI_GET_LEN(ctrl));

	ast_espi->oob_rx_channel.header = ctrl;
	ast_espi->oob_rx_channel.buf_len = ESPI_GET_LEN(ctrl);
#ifdef AST_ESPI_DMA_MODE
#else
	for (i = 0; i < ast_espi->oob_rx_channel.buf_len; i++)
		ast_espi->oob_rx_channel.buff[i] = ast_espi_read(ast_espi, AST_ESPI_OOB_RX_DATA);
#endif
}

static void
ast_espi_oob_tx(struct ast_espi_data *ast_espi)
{
	int i = 0;
	ESPI_DBUG("\n");

#ifdef AST_ESPI_DMA_MODE

#else
	for (i = 0; i < ast_espi->oob_tx_channel.buf_len; i++)
		ast_espi_write(ast_espi, ast_espi->oob_tx_channel.buff[i] , AST_ESPI_OOB_TX_DATA);
#endif

	ast_espi_write(ast_espi, ESPI_TRIGGER_PACKAGE | ast_espi->oob_tx_channel.header , AST_ESPI_OOB_TX_CTRL);
}

static void
ast_espi_pcp_rx(struct ast_espi_data *ast_espi)
{
	int i = 0;
	u32 ctrl = ast_espi_read(ast_espi, AST_ESPI_PCP_RX_CTRL);
	ESPI_DBUG("cycle type = %x , tag = %x, len = %d byte \n", ESPI_GET_CYCLE_TYPE(ctrl), ESPI_GET_TAG(ctrl), ESPI_GET_LEN(ctrl));

	ast_espi->p_rx_channel.header = ctrl;

	//Message
	if ((ESPI_GET_CYCLE_TYPE(ctrl) & 0x10) == 0x10) {		//message
		ast_espi->p_rx_channel.buf_len = 5;
		if (ESPI_GET_CYCLE_TYPE(ctrl) & 0x1)	//message with data
			ast_espi->p_rx_channel.buf_len += ESPI_GET_LEN(ctrl);
	} else if ((ESPI_GET_CYCLE_TYPE(ctrl) & 0x09) == 0x09)	//success com with data
		ast_espi->p_rx_channel.buf_len = ESPI_GET_LEN(ctrl);
	else
		ast_espi->p_rx_channel.buf_len = 0;

#ifdef AST_ESPI_DMA_MODE
#else
	for (i = 0; i < ast_espi->p_rx_channel.buf_len; i++)
		ast_espi->p_rx_channel.buff[i] = ast_espi_read(ast_espi, AST_ESPI_PCP_RX_DATA);
#endif
	//wait for up get package
}

static void
ast_espi_pcp_tx(struct ast_espi_data *ast_espi)
{
	int i = 0;
	ESPI_DBUG("\n");

#ifdef AST_ESPI_DMA_MODE

#else
	for (i = 0; i < ast_espi->p_tx_channel.buf_len; i++)
		ast_espi_write(ast_espi, ast_espi->p_tx_channel.buff[i] , AST_ESPI_PCP_TX_DATA);
#endif
	ast_espi_write(ast_espi, ESPI_TRIGGER_PACKAGE | ast_espi->p_tx_channel.header, AST_ESPI_PCP_TX_CTRL);
}

static void
ast_espi_pcnp_tx(struct ast_espi_data *ast_espi)
{
	int i = 0;
	ESPI_DBUG("\n");

#ifdef AST_ESPI_DMA_MODE

#else
	for (i = 0; i < ast_espi->np_tx_channel.buf_len; i++)
		ast_espi_write(ast_espi, ast_espi->np_tx_channel.buff[i] , AST_ESPI_PCNP_TX_DATA);
#endif
	ast_espi_write(ast_espi, ESPI_TRIGGER_PACKAGE | ast_espi->np_tx_channel.header, AST_ESPI_PCNP_TX_CTRL);
}

static void
ast_sys_event(struct ast_espi_data *ast_espi)
{
	u32 sts = ast_espi_read(ast_espi, AST_ESPI_SYS_EVENT_ISR);
	u32 sys_event = ast_espi_read(ast_espi, AST_ESPI_SYS_EVENT);
	ESPI_DBUG("sts %x, sys_event %x\n", sts, sys_event);

	if (sts & ESPI_HOST_RST_WARN) {
		if (sys_event & ESPI_HOST_RST_WARN)
			ast_espi_write(ast_espi, sys_event | ESPI_HOST_REST_ACK, AST_ESPI_SYS_EVENT);
		else
			ast_espi_write(ast_espi, sys_event & ~ESPI_HOST_REST_ACK, AST_ESPI_SYS_EVENT);
		ast_espi_write(ast_espi, ESPI_HOST_RST_WARN, AST_ESPI_SYS_EVENT_ISR);
	}

	if (sts & ESPI_OOB_RST_WARN) {
		if (sys_event & ESPI_OOB_RST_WARN)
			ast_espi_write(ast_espi, sys_event | ESPI_OOB_REST_ACK, AST_ESPI_SYS_EVENT);
		else
			ast_espi_write(ast_espi, sys_event & ~ESPI_OOB_REST_ACK, AST_ESPI_SYS_EVENT);
		ast_espi_write(ast_espi, ESPI_OOB_RST_WARN, AST_ESPI_SYS_EVENT_ISR);
	}

	if (sts & ~(ESPI_OOB_RST_WARN | ESPI_HOST_RST_WARN)) {
		printk("new sts %x \n", sts);
		ast_espi_write(ast_espi, sts, AST_ESPI_SYS_EVENT_ISR);
	}

}

static void
ast_sys1_event(struct ast_espi_data *ast_espi)
{
	u32 sts = ast_espi_read(ast_espi, AST_ESPI_SYS1_INT_STS);
	if (sts & ESPI_SYS_SUS_WARN) {
		ast_espi_write(ast_espi, ast_espi_read(ast_espi, AST_ESPI_SYS1_EVENT) | ESPI_SYS_SUS_ACK, AST_ESPI_SYS1_EVENT);
		//TODO  polling bit 20 is 1
		ast_espi_write(ast_espi, ESPI_SYS_SUS_WARN, AST_ESPI_SYS1_INT_STS);
	}

	if (sts & ~(ESPI_SYS_SUS_WARN)) {
		printk("new sys1 sts %x \n", sts);
		ast_espi_write(ast_espi, sts, AST_ESPI_SYS1_INT_STS);
	}

}

static void ast_espi_ctrl_init(struct ast_espi_data *ast_espi)
{

	//a1 espi intiial
	ast_espi_write(ast_espi, ast_espi_read(ast_espi, AST_ESPI_CTRL) | 0xff, AST_ESPI_CTRL);

	//TODO for function interrpt type
	ast_espi_write(ast_espi, 0, AST_ESPI_SYS_INT_T0);
	ast_espi_write(ast_espi, 0, AST_ESPI_SYS_INT_T1);
	ast_espi_write(ast_espi, ESPI_HOST_RST_WARN | ESPI_OOB_RST_WARN, AST_ESPI_SYS_INT_T2);

	ast_espi_write(ast_espi, 0xffffffff, AST_ESPI_IER);
	ast_espi_write(ast_espi, 0xffffffff, AST_ESPI_SYS_IER);

	//A1
	ast_espi_write(ast_espi, 0x1, AST_ESPI_SYS1_IER);
	ast_espi_write(ast_espi, 0x1, AST_ESPI_SYS1_INT_T0);


#ifdef AST_ESPI_DMA_MODE
	ast_espi_write(ast_espi, ast_espi->p_rx_channel.dma_addr, AST_ESPI_PCP_RX_DMA);
	ast_espi_write(ast_espi, ast_espi->p_tx_channel.dma_addr, AST_ESPI_PCP_TX_DMA);

	ast_espi_write(ast_espi, ast_espi->np_tx_channel.dma_addr, AST_ESPI_PCNP_TX_DMA);

	ast_espi_write(ast_espi, ast_espi->oob_rx_channel.dma_addr, AST_ESPI_OOB_RX_DMA);
	ast_espi_write(ast_espi, ast_espi->oob_tx_channel.dma_addr, AST_ESPI_OOB_TX_DMA);

	ast_espi_write(ast_espi, ast_espi->flash_rx_channel.dma_addr, AST_ESPI_FLASH_RX_DMA);
	ast_espi_write(ast_espi, ast_espi->flash_tx_channel.dma_addr, AST_ESPI_FLASH_TX_DMA);

	ast_espi_write(ast_espi, ast_espi_read(ast_espi, AST_ESPI_CTRL) |
				   ESPI_CTRL_FLASH_RX_DMA | ESPI_CTRL_FLASH_TX_DMA |
				   ESPI_CTRL_OOB_RX_DMA | ESPI_CTRL_OOB_TX_DMA |
				   ESPI_CTRL_PCNP_TX_DMA | ESPI_CTRL_PCP_RX_DMA | ESPI_CTRL_PCP_TX_DMA,  AST_ESPI_CTRL);
#endif


}

static irqreturn_t ast_espi_reset_isr(int this_irq, void *dev_id)
{
	u32 sw_mode = 0;
	struct ast_espi_data *ast_espi = dev_id;

	ESPI_DBUG("ast_espi_reset_isr\n");

	sw_mode = ast_espi_read(ast_espi, AST_ESPI_CTRL) & ESPI_CTRL_SW_FLASH_READ;

	//scu init
	reset_control_assert(ast_espi->reset);
	reset_control_deassert(ast_espi->reset);
	ast_espi_ctrl_init(ast_espi);

	ast_espi_write(ast_espi, ast_espi_read(ast_espi, AST_ESPI_CTRL) | sw_mode, AST_ESPI_CTRL);

	return IRQ_HANDLED;
}
static irqreturn_t ast_espi_isr(int this_irq, void *dev_id)
{
	struct ast_espi_data *ast_espi = dev_id;
	u32 sts = ast_espi_read(ast_espi, AST_ESPI_ISR);
	ESPI_DBUG("sts : %x\n", sts);

#ifdef CONFIG_COLDFIRE_ESPI
#else
	if (sts & ESPI_ISR_HW_RESET) {
		printk("ESPI_ISR_HW_RESET \n");
		ast_espi_write(ast_espi, ast_espi_read(ast_espi, AST_ESPI_SYS_EVENT) | ESPI_BOOT_STS | ESPI_BOOT_DWN, AST_ESPI_SYS_EVENT);

		//6:flash ready ,4: oob ready , 0: perp ready
//		ast_espi_write(ast_espi, ast_espi_read(ast_espi, AST_ESPI_CTRL) |(0x1 << 4) | (0x1 << 6), AST_ESPI_CTRL);
		ast_espi_write(ast_espi, ESPI_ISR_HW_RESET, AST_ESPI_ISR);
	}
#endif

	if (sts & ESPI_ISR_FLASH_TX_ERR) {
		printk("ESPI_ISR_FLASH_TX_ERR \n");
		ast_espi_write(ast_espi, ESPI_ISR_FLASH_TX_ERR, AST_ESPI_ISR);
	}

	if (sts & ESPI_ISR_OOB_TX_ERR) {
		printk("ESPI_ISR_OOB_TX_ERR \n");
		ast_espi_write(ast_espi, ESPI_ISR_OOB_TX_ERR, AST_ESPI_ISR);
	}

	if (sts & ESPI_ISR_FLASH_TX_ABORT) {
		printk("ESPI_ISR_FLASH_TX_ABORT \n");
		ast_espi_write(ast_espi, ESPI_ISR_FLASH_TX_ABORT, AST_ESPI_ISR);
	}

	if (sts & ESPI_ISR_OOB_TX_ABORT) {
		printk("ESPI_ISR_OOB_TX_ABORT \n");
		ast_espi_write(ast_espi, ESPI_ISR_OOB_TX_ABORT, AST_ESPI_ISR);
	}

	if (sts & ESPI_ISR_PCNP_TX_ABORT) {
		printk("ESPI_ISR_PCNP_TX_ABORT\n");
		ast_espi_write(ast_espi, ESPI_ISR_PCNP_TX_ABORT, AST_ESPI_ISR);
	}

	if (sts & ESPI_ISR_PCP_TX_ABORT) {
		printk("ESPI_ISR_PCP_TX_ABORT\n");
		ast_espi_write(ast_espi, ESPI_ISR_PCP_TX_ABORT, AST_ESPI_ISR);
	}

	if (sts & ESPI_ISR_FLASH_RX_ABORT) {
		printk("ESPI_ISR_FLASH_RX_ABORT \n");
		ast_espi_write(ast_espi, ESPI_ISR_FLASH_RX_ABORT, AST_ESPI_ISR);
	}

	if (sts & ESPI_ISR_OOB_RX_ABORT) {
		printk("ESPI_ISR_OOB_RX_ABORT");
		ast_espi_write(ast_espi, ESPI_ISR_OOB_RX_ABORT, AST_ESPI_ISR);
	}

	if (sts & ESPI_ISR_PCNP_RX_ABORT) {
		printk("ESPI_ISR_PCNP_RX_ABORT\n");
		ast_espi_write(ast_espi, ESPI_ISR_PCNP_RX_ABORT, AST_ESPI_ISR);
	}

	if (sts & ESPI_ISR_PCP_RX_ABORT) {
		printk("ESPI_ISR_PCP_RX_ABORT \n");
		ast_espi_write(ast_espi, ESPI_ISR_PCP_RX_ABORT, AST_ESPI_ISR);
	}

	if (sts & ESPI_ISR_PCNP_TX_ERR) {
		printk("ESPI_ISR_PCNP_TX_ERR \n");
		ast_espi_write(ast_espi, ESPI_ISR_PCNP_TX_ERR, AST_ESPI_ISR);
	}

	if (sts & ESPI_ISR_PCP_TX_ERR) {
		printk("ESPI_ISR_PCP_TX_ERR \n");
		ast_espi_write(ast_espi, ESPI_ISR_PCP_TX_ERR, AST_ESPI_ISR);
	}

	if (sts & ESPI_ISR_VIRTW_GPIO) {
		ESPI_DBUG("ESPI_ISR_VIRTW_GPIO \n");
		ast_espi->vw_gpio = ast_espi_read(ast_espi, AST_ESPI_GPIO_VIRTCH);
		ast_espi_write(ast_espi, ESPI_ISR_VIRTW_GPIO, AST_ESPI_ISR);
	}

	if (sts & ESPI_ISR_VIRTW_SYS) {
		ESPI_DBUG("ESPI_ISR_VIRTW_SYS \n");
//		ast_espi->sys_event = ast_espi_read(ast_espi, AST_ESPI_SYS_EVENT);
		ast_sys_event(ast_espi);
		ast_espi_write(ast_espi, ESPI_ISR_VIRTW_SYS, AST_ESPI_ISR);
	}

#ifdef CONFIG_COLDFIRE_ESPI
#else
	//AST2500 A1
	if (sts & ESPI_ISR_VIRTW_SYS1) {
		ESPI_DBUG("ESPI_ISR_VIRTW_SYS1 \n");
		ast_sys1_event(ast_espi);
		ast_espi_write(ast_espi, ESPI_ISR_VIRTW_SYS1, AST_ESPI_ISR);
	}

	if (sts & ESPI_ISR_FLASH_TX_COMP) {
		ESPI_DBUG("ESPI_ISR_FLASH_TX_COMP \n");
		ast_espi_write(ast_espi, ESPI_ISR_FLASH_TX_COMP, AST_ESPI_ISR);
	}

	if (sts & ESPI_ISR_FLASH_RX_COMP) {
		ESPI_DBUG("ESPI_ISR_FLASH_RX_COMP \n");
		ast_espi_flash_rx(ast_espi);
		ast_espi_write(ast_espi, ESPI_ISR_FLASH_RX_COMP, AST_ESPI_ISR);
	}
#endif
	if (sts & ESPI_ISR_OOB_TX_COMP) {
		ESPI_DBUG("ESPI_ISR_OOB_TX_COMP \n");
		ast_espi_write(ast_espi, ESPI_ISR_OOB_TX_COMP, AST_ESPI_ISR);
	}

	if (sts & ESPI_ISR_OOB_RX_COMP) {
		ESPI_DBUG("ESPI_ISR_OOB_RX_COMP \n");
		ast_espi_oob_rx(ast_espi);
		ast_espi_write(ast_espi, ESPI_ISR_OOB_RX_COMP, AST_ESPI_ISR);
	}

	if (sts & ESPI_ISR_PCNP_TX_COMP) {
		ESPI_DBUG("ESPI_ISR_PCNP_TX_COMP \n");
		ast_espi_write(ast_espi, ESPI_ISR_PCNP_TX_COMP, AST_ESPI_ISR);
	}

	if (sts & ESPI_ISR_PCP_TX_COMP) {
		ESPI_DBUG("ESPI_ISR_PCP_TX_COMP \n");
		ast_espi_write(ast_espi, ESPI_ISR_PCP_TX_COMP, AST_ESPI_ISR);
	}

	if (sts & ESPI_ISR_PCP_RX_COMP) {
		ESPI_DBUG("ESPI_ISR_PCP_RX_COMP \n");
		ast_espi_pcp_rx(ast_espi);
		ast_espi_write(ast_espi, ESPI_ISR_PCP_RX_COMP, AST_ESPI_ISR);
	}

#ifdef CONFIG_COLDFIRE_ESPI
#else
	ast_espi->irq_sts |= sts;
	if (ast_espi->async_queue)
		kill_fasync(&ast_espi->async_queue, SIGIO, POLL_IN);
#endif
	return IRQ_HANDLED;
}

static long
espi_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	struct miscdevice *c = file->private_data;
	struct ast_espi_data *ast_espi = dev_get_drvdata(c->this_device);
	struct ast_espi_xfer *xfer = (void __user *)arg;

	switch (cmd) {
	case AST_ESPI_PERIP_IOCRX:
		ESPI_DBUG(" \n");
		xfer->header = ast_espi->p_rx_channel.header;
		xfer->buf_len = ast_espi->p_rx_channel.buf_len;
		if (copy_to_user(xfer->xfer_buf, ast_espi->p_rx_channel.buff, ast_espi->p_rx_channel.buf_len))
			ret = -EFAULT;
		ast_espi_write(ast_espi, ESPI_TRIGGER_PACKAGE, AST_ESPI_PCP_RX_CTRL);
		break;
	case AST_ESPI_PERIP_IOCTX:
		ESPI_DBUG(" \n");
		ast_espi->p_tx_channel.header = xfer->header;
		ast_espi->p_tx_channel.buf_len = xfer->buf_len;
		if (copy_from_user(ast_espi->p_tx_channel.buff, xfer->xfer_buf, xfer->buf_len)) {
			ESPI_DBUG("copy_from_user  fail\n");
			ret = -EFAULT;
		} else
			ast_espi_pcp_tx(ast_espi);
		break;
	case AST_ESPI_PERINP_IOCTX:
		ESPI_DBUG(" \n");
		ast_espi->np_tx_channel.header = xfer->header;
		ast_espi->np_tx_channel.buf_len = xfer->buf_len;
		if (copy_from_user(ast_espi->np_tx_channel.buff, xfer->xfer_buf, xfer->buf_len)) {
			ESPI_DBUG("copy_from_user  fail\n");
			ret = -EFAULT;
		} else
			ast_espi_pcnp_tx(ast_espi);
		break;
	case AST_ESPI_OOB_IOCRX:
		ESPI_DBUG(" \n");
		xfer->header = ast_espi->oob_rx_channel.header;
		xfer->buf_len = ast_espi->oob_rx_channel.buf_len;
		if (copy_to_user(xfer->xfer_buf, ast_espi->oob_rx_channel.buff, ast_espi->oob_rx_channel.buf_len))
			ret = -EFAULT;
		ast_espi_write(ast_espi, ESPI_TRIGGER_PACKAGE, AST_ESPI_OOB_RX_CTRL);
		break;
	case AST_ESPI_OOB_IOCTX:
		ESPI_DBUG(" \n");
		ast_espi->oob_tx_channel.header = xfer->header;
		ast_espi->oob_tx_channel.buf_len = xfer->buf_len;
		if (copy_from_user(ast_espi->oob_tx_channel.buff, xfer->xfer_buf, xfer->buf_len)) {
			ESPI_DBUG("copy_from_user  fail\n");
			ret = -EFAULT;
		} else
			ast_espi_oob_tx(ast_espi);
		break;
#ifdef CONFIG_COLDFIRE_ESPI
#else
	case AST_ESPI_FLASH_IOCRX:
		ESPI_DBUG(" AST_ESPI_FLASH_IOCRX \n");
		if (!ast_espi->flash_rx_channel.full) {
			ret = -ENODATA;
			break;
		}
		xfer->header = ast_espi->flash_rx_channel.header;
		xfer->buf_len = ast_espi->flash_rx_channel.buf_len;
		if (copy_to_user(xfer->xfer_buf, ast_espi->flash_rx_channel.buff, ast_espi->flash_rx_channel.buf_len))
			ret = -EFAULT;

		ast_espi->flash_rx_channel.full = 0;
		ast_espi_write(ast_espi, ESPI_TRIGGER_PACKAGE, AST_ESPI_FLASH_RX_CTRL);
		break;
	case AST_ESPI_FLASH_IOCTX:
		ESPI_DBUG("header %x, buf_len = %d  \n", xfer->header, xfer->buf_len);
		ast_espi->flash_tx_channel.header = xfer->header;
		ast_espi->flash_tx_channel.buf_len = xfer->buf_len;
		if (xfer->buf_len) {
			if (copy_from_user(ast_espi->flash_tx_channel.buff, xfer->xfer_buf, xfer->buf_len)) {
				ESPI_DBUG("copy_from_user  fail\n");
				ret = -EFAULT;
			}
		}
		ast_espi_flash_tx(ast_espi);
		break;
#endif
	case AST_ESPI_GPIO_IOCRX:
		ESPI_DBUG(" \n");
		ast_espi->vw_gpio = ast_espi_read(ast_espi, AST_ESPI_GPIO_VIRTCH);
		ret = __put_user(ast_espi->vw_gpio, (u32 __user *)arg);
		ast_espi->vw_gpio = 0;
		break;
	case AST_ESPI_SYS_IOCRX:
		ESPI_DBUG(" \n");
		ast_espi->sys_event = ast_espi_read(ast_espi, AST_ESPI_SYS_EVENT) & 0xffff;
		ret = __put_user(ast_espi->sys_event, (u32 __user *)arg);
		ast_espi->sys_event = 0;
		break;
	case AST_ESPI_RX_IOCSTS:
		ret = __put_user(ast_espi->irq_sts, (u32 __user *)arg);
		ast_espi->irq_sts = 0;
		break;

	default:
		ESPI_DBUG("ERROR \n");
		return -ENOTTY;
	}

	return ret;
}

static int espi_open(struct inode *inode, struct file *file)
{
	struct miscdevice *c = file->private_data;
	struct ast_espi_data *ast_espi = dev_get_drvdata(c->this_device);

	ESPI_DBUG("\n");
	spin_lock(&espi_state_lock);

	if (ast_espi->is_open) {
		spin_unlock(&espi_state_lock);
		return -EBUSY;
	}

	ast_espi->is_open = true;

	spin_unlock(&espi_state_lock);

	return 0;
}

static int espi_fasync(int fd, struct file *file, int mode)
{
	struct miscdevice *c = file->private_data;
	struct ast_espi_data *ast_espi = dev_get_drvdata(c->this_device);
	return fasync_helper(fd, file, mode, &ast_espi->async_queue);
}

static int espi_release(struct inode *inode, struct file *file)
{
	struct miscdevice *c = file->private_data;
	struct ast_espi_data *ast_espi = dev_get_drvdata(c->this_device);

	ESPI_DBUG("\n");
	spin_lock(&espi_state_lock);

	ast_espi->is_open = false;
//	espi_fasync(-1, file, 0);
	spin_unlock(&espi_state_lock);

	return 0;
}

static ssize_t show_fatal_err(struct device *dev,
							  struct device_attribute *attr, char *buf)
{
	struct ast_espi_data *ast_espi = dev_get_drvdata(dev);
	return sprintf(buf, "%s\n", ast_espi_read(ast_espi, AST_ESPI_SYS_EVENT) & ESPI_FATEL_ERR ? "0" : "1");
}

static ssize_t store_fatal_err(struct device *dev,
							   struct device_attribute *attr, const char *buf, size_t count)
{
	u32 val;
	struct ast_espi_data *ast_espi = dev_get_drvdata(dev);

	val = simple_strtoul(buf, NULL, 5);
	if (val)
		ast_espi_write(ast_espi, ast_espi_read(ast_espi, AST_ESPI_SYS_EVENT) | ESPI_FATEL_ERR, AST_ESPI_SYS_EVENT);
	else
		ast_espi_write(ast_espi, ast_espi_read(ast_espi, AST_ESPI_SYS_EVENT) & ~ESPI_FATEL_ERR, AST_ESPI_SYS_EVENT);

	return count;
}

static DEVICE_ATTR(fatal_err, S_IWUSR | S_IWUSR, show_fatal_err, store_fatal_err);

static ssize_t show_nfatal_err(struct device *dev,
							   struct device_attribute *attr, char *buf)
{
	struct ast_espi_data *ast_espi = dev_get_drvdata(dev);
	return sprintf(buf, "%s\n", ast_espi_read(ast_espi, AST_ESPI_SYS_EVENT) & ESPI_NFATEL_ERR ? "0" : "1");
}

static ssize_t store_nfatal_err(struct device *dev,
								struct device_attribute *attr, const char *buf, size_t count)
{
	u32 val;
	struct ast_espi_data *ast_espi = dev_get_drvdata(dev);

	val = simple_strtoul(buf, NULL, 5);
	if (val)
		ast_espi_write(ast_espi, ast_espi_read(ast_espi, AST_ESPI_SYS_EVENT) | ESPI_NFATEL_ERR, AST_ESPI_SYS_EVENT);
	else
		ast_espi_write(ast_espi, ast_espi_read(ast_espi, AST_ESPI_SYS_EVENT) & ~ESPI_NFATEL_ERR, AST_ESPI_SYS_EVENT);

	return count;
}

static DEVICE_ATTR(nfatal_err, S_IWUSR | S_IWUSR, show_nfatal_err, store_nfatal_err);

static ssize_t show_rest_cpu(struct device *dev,
							 struct device_attribute *attr, char *buf)
{
	struct ast_espi_data *ast_espi = dev_get_drvdata(dev);
	return sprintf(buf, "%s\n", ast_espi_read(ast_espi, AST_ESPI_SYS_EVENT) & ESPI_REST_CPU_INIT ? "0" : "1");
}

static ssize_t store_rest_cpu(struct device *dev,
							  struct device_attribute *attr, const char *buf, size_t count)
{
	u32 val;
	struct ast_espi_data *ast_espi = dev_get_drvdata(dev);

	val = simple_strtoul(buf, NULL, 5);
	if (val)
		ast_espi_write(ast_espi, ast_espi_read(ast_espi, AST_ESPI_SYS_EVENT) | ESPI_REST_CPU_INIT, AST_ESPI_SYS_EVENT);
	else
		ast_espi_write(ast_espi, ast_espi_read(ast_espi, AST_ESPI_SYS_EVENT) & ~ESPI_REST_CPU_INIT, AST_ESPI_SYS_EVENT);

	return count;
}

static DEVICE_ATTR(rest_cpu,  S_IWUSR | S_IWUSR, show_rest_cpu, store_rest_cpu);

static ssize_t show_host_rest_ack(struct device *dev,
								  struct device_attribute *attr, char *buf)
{
	struct ast_espi_data *ast_espi = dev_get_drvdata(dev);
	return sprintf(buf, "%s\n", ast_espi_read(ast_espi, AST_ESPI_SYS_EVENT) & ESPI_HOST_REST_ACK ? "0" : "1");
}

static ssize_t store_host_rest_ack(struct device *dev,
								   struct device_attribute *attr, const char *buf, size_t count)
{
	u32 val;
	struct ast_espi_data *ast_espi = dev_get_drvdata(dev);

	val = simple_strtoul(buf, NULL, 5);
	if (val)
		ast_espi_write(ast_espi, ast_espi_read(ast_espi, AST_ESPI_SYS_EVENT) | ESPI_HOST_REST_ACK, AST_ESPI_SYS_EVENT);
	else
		ast_espi_write(ast_espi, ast_espi_read(ast_espi, AST_ESPI_SYS_EVENT) & ~ESPI_HOST_REST_ACK, AST_ESPI_SYS_EVENT);

	return count;
}

static DEVICE_ATTR(host_rest_ack, S_IWUSR | S_IWUSR, show_host_rest_ack, store_host_rest_ack);

static ssize_t show_oob_rest_ack(struct device *dev,
								 struct device_attribute *attr, char *buf)
{
	struct ast_espi_data *ast_espi = dev_get_drvdata(dev);
	return sprintf(buf, "%s\n", ast_espi_read(ast_espi, AST_ESPI_SYS_EVENT) & ESPI_OOB_REST_ACK ? "0" : "1");
}

static ssize_t store_oob_rest_ack(struct device *dev,
								  struct device_attribute *attr, const char *buf, size_t count)
{
	u32 val;
	struct ast_espi_data *ast_espi = dev_get_drvdata(dev);

	val = simple_strtoul(buf, NULL, 5);
	if (val)
		ast_espi_write(ast_espi, ast_espi_read(ast_espi, AST_ESPI_SYS_EVENT) | ESPI_OOB_REST_ACK, AST_ESPI_SYS_EVENT);
	else
		ast_espi_write(ast_espi, ast_espi_read(ast_espi, AST_ESPI_SYS_EVENT) & ~ESPI_OOB_REST_ACK, AST_ESPI_SYS_EVENT);

	return count;
}

static DEVICE_ATTR(oob_rest_ack, S_IWUSR | S_IWUSR, show_oob_rest_ack, store_oob_rest_ack);

static ssize_t show_boot_sts(struct device *dev,
							 struct device_attribute *attr, char *buf)
{
	struct ast_espi_data *ast_espi = dev_get_drvdata(dev);
	return sprintf(buf, "%s\n", ast_espi_read(ast_espi, AST_ESPI_SYS_EVENT) & ESPI_BOOT_STS ? "0" : "1");
}

static ssize_t store_boot_sts(struct device *dev,
							  struct device_attribute *attr, const char *buf, size_t count)
{
	u32 val;
	struct ast_espi_data *ast_espi = dev_get_drvdata(dev);

	val = simple_strtoul(buf, NULL, 5);
	if (val)
		ast_espi_write(ast_espi, ast_espi_read(ast_espi, AST_ESPI_SYS_EVENT) | ESPI_BOOT_STS, AST_ESPI_SYS_EVENT);
	else
		ast_espi_write(ast_espi, ast_espi_read(ast_espi, AST_ESPI_SYS_EVENT) & ~ESPI_BOOT_STS, AST_ESPI_SYS_EVENT);

	return count;
}

static DEVICE_ATTR(boot_sts, S_IWUSR | S_IWUSR, show_boot_sts, store_boot_sts);

static ssize_t show_boot_dwn(struct device *dev,
							 struct device_attribute *attr, char *buf)
{
	struct ast_espi_data *ast_espi = dev_get_drvdata(dev);
	return sprintf(buf, "%s\n", ast_espi_read(ast_espi, AST_ESPI_SYS_EVENT) & ESPI_BOOT_DWN ? "0" : "1");
}

static ssize_t store_boot_dwn(struct device *dev,
							  struct device_attribute *attr, const char *buf, size_t count)
{
	u32 val;
	struct ast_espi_data *ast_espi = dev_get_drvdata(dev);

	val = simple_strtoul(buf, NULL, 5);
	if (val)
		ast_espi_write(ast_espi, ast_espi_read(ast_espi, AST_ESPI_SYS_EVENT) | ESPI_BOOT_DWN, AST_ESPI_SYS_EVENT);
	else
		ast_espi_write(ast_espi, ast_espi_read(ast_espi, AST_ESPI_SYS_EVENT) & ~ESPI_BOOT_DWN, AST_ESPI_SYS_EVENT);

	return count;
}

static DEVICE_ATTR(boot_dwn, S_IWUSR | S_IWUSR, show_boot_dwn, store_boot_dwn);

static ssize_t show_op_freq(struct device *dev,
							struct device_attribute *attr, char *buf)
{
	struct ast_espi_data *ast_espi = dev_get_drvdata(dev);

	switch (GET_GCAP_OP_FREQ(ast_espi_read(ast_espi, AST_ESPI_GCAP_CONFIG))) {
	case 0:
		return sprintf(buf, "20Mhz\n");
		break;
	case 1:
		return sprintf(buf, "25Mhz\n");
		break;
	case 2:
		return sprintf(buf, "33Mhz\n");
		break;
	case 3:
		return sprintf(buf, "50Mhz\n");
		break;
	case 4:
		return sprintf(buf, "66Mhz\n");
		break;
	}
	return sprintf(buf, "Unknow\n");
}

static DEVICE_ATTR(op_freq, S_IRUGO, show_op_freq, NULL);

static ssize_t show_io_mode(struct device *dev,
							struct device_attribute *attr, char *buf)
{
	struct ast_espi_data *ast_espi = dev_get_drvdata(dev);

	switch (GET_GCAP_IO_MODE(ast_espi_read(ast_espi, AST_ESPI_GCAP_CONFIG))) {
	case 0:
		return sprintf(buf, "Single IO\n");
		break;
	case 1:
		return sprintf(buf, "Dual IO\n");
		break;
	case 2:
		return sprintf(buf, "Qual IO\n");
		break;
	}
	return sprintf(buf, "Unknow\n");
}

static DEVICE_ATTR(io_mode, S_IRUGO, show_io_mode, NULL);

static ssize_t show_sw_gpio(struct device *dev,
							struct device_attribute *attr, char *buf)
{
	struct ast_espi_data *ast_espi = dev_get_drvdata(dev);

	return sprintf(buf, "%s Mode\n", ast_espi_read(ast_espi, AST_ESPI_CTRL) & ESPI_CTRL_SW_GPIO_VIRTCH ? "1:SW" : "0:HW");
}

static ssize_t store_sw_gpio(struct device *dev,
							 struct device_attribute *attr, const char *buf, size_t count)
{
	u32 val;
	struct ast_espi_data *ast_espi = dev_get_drvdata(dev);

	val = simple_strtoul(buf, NULL, 5);
	if (val)
		ast_espi_write(ast_espi, ast_espi_read(ast_espi, AST_ESPI_CTRL) | ESPI_CTRL_SW_GPIO_VIRTCH, AST_ESPI_CTRL);
	else
		ast_espi_write(ast_espi, ast_espi_read(ast_espi, AST_ESPI_CTRL) & ~ESPI_CTRL_SW_GPIO_VIRTCH, AST_ESPI_CTRL);

	return count;
}

static DEVICE_ATTR(sw_gpio, S_IRUGO | S_IWUSR, show_sw_gpio, store_sw_gpio);

static ssize_t show_sw_flash_read(struct device *dev,
								  struct device_attribute *attr, char *buf)
{
	struct ast_espi_data *ast_espi = dev_get_drvdata(dev);

	return sprintf(buf, "%s Mode\n", ast_espi_read(ast_espi, AST_ESPI_CTRL) & ESPI_CTRL_SW_FLASH_READ ? "1:SW" : "0:HW");
}

static ssize_t store_sw_flash_read(struct device *dev,
								   struct device_attribute *attr, const char *buf, size_t count)
{
	u32 val;
	struct ast_espi_data *ast_espi = dev_get_drvdata(dev);

	val = simple_strtoul(buf, NULL, 5);
	if (val)
		ast_espi_write(ast_espi, ast_espi_read(ast_espi, AST_ESPI_CTRL) | ESPI_CTRL_SW_FLASH_READ, AST_ESPI_CTRL);
	else
		ast_espi_write(ast_espi, ast_espi_read(ast_espi, AST_ESPI_CTRL) & ~ESPI_CTRL_SW_FLASH_READ, AST_ESPI_CTRL);

	return count;
}

static DEVICE_ATTR(sw_flash_read, S_IRUGO | S_IWUSR, show_sw_flash_read, store_sw_flash_read);

static struct attribute *espi_sysfs_entries[] = {
	&dev_attr_sw_flash_read.attr,
	&dev_attr_sw_gpio.attr,
	&dev_attr_io_mode.attr,
	&dev_attr_op_freq.attr,
	&dev_attr_boot_sts.attr,
	&dev_attr_boot_dwn.attr,
	&dev_attr_oob_rest_ack.attr,
	&dev_attr_fatal_err.attr,
	&dev_attr_nfatal_err.attr,
	&dev_attr_rest_cpu.attr,
	&dev_attr_host_rest_ack.attr,
	NULL
};

static struct attribute_group espi_attribute_group = {
	.attrs = espi_sysfs_entries,
};

static const struct file_operations ast_espi_fops = {
	.owner			= THIS_MODULE,
	.unlocked_ioctl		= espi_ioctl,
	.open			= espi_open,
	.release			= espi_release,
	.fasync			= espi_fasync,
};

///////////////////////////////////
struct miscdevice ast_espi_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "ast-espi",
	.fops = &ast_espi_fops,
};

#define MAX_XFER_BUFF_SIZE	0x80		//128
///////////////////////////////////
static int ast_espi_probe(struct platform_device *pdev)
{
	static struct ast_espi_data *ast_espi;
	struct resource *res;
	int ret = 0;
	u32 gpio_irq;

	ESPI_DBUG("\n");

	ast_espi = devm_kzalloc(&pdev->dev, sizeof(struct ast_espi_data), GFP_KERNEL);
	if (ast_espi == NULL) {
		dev_err(&pdev->dev, "failed to allocate memory\n");
		return -ENOMEM;
	}

	ast_espi->pdev = pdev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "no memory resource defined\n");
		ret = -ENODEV;
		goto err_free;
	}

	ast_espi->reg_base = devm_ioremap_resource(&pdev->dev, res);
	if (ast_espi->reg_base == NULL) {
		dev_err(&pdev->dev, "failed to ioremap() registers\n");
		ret = -ENODEV;
		goto err_free_mem;
	}

#ifdef AST_ESPI_DMA_MODE
	ast_espi->p_rx_channel.buff = dma_alloc_coherent(NULL,
								  MAX_XFER_BUFF_SIZE * 7,
								  &ast_espi->p_rx_channel.dma_addr, GFP_KERNEL);

	ast_espi->p_tx_channel.buff = ast_espi->p_rx_channel.buff  + MAX_XFER_BUFF_SIZE;
	ast_espi->p_tx_channel.dma_addr = ast_espi->p_rx_channel.dma_addr + MAX_XFER_BUFF_SIZE;

	ast_espi->np_tx_channel.buff = ast_espi->p_tx_channel.buff  + MAX_XFER_BUFF_SIZE;
	ast_espi->np_tx_channel.dma_addr = ast_espi->p_tx_channel.dma_addr + MAX_XFER_BUFF_SIZE;

	ast_espi->oob_rx_channel.buff = ast_espi->np_tx_channel.buff + MAX_XFER_BUFF_SIZE;
	ast_espi->oob_rx_channel.dma_addr = ast_espi->np_tx_channel.dma_addr + MAX_XFER_BUFF_SIZE;

	ast_espi->oob_tx_channel.buff = ast_espi->oob_rx_channel.buff + MAX_XFER_BUFF_SIZE;
	ast_espi->oob_tx_channel.dma_addr = ast_espi->oob_rx_channel.dma_addr + MAX_XFER_BUFF_SIZE;

	ast_espi->flash_rx_channel.buff = ast_espi->oob_tx_channel.buff + MAX_XFER_BUFF_SIZE;
	ast_espi->flash_rx_channel.dma_addr = ast_espi->oob_tx_channel.dma_addr + MAX_XFER_BUFF_SIZE;

	ast_espi->flash_tx_channel.buff = ast_espi->flash_rx_channel.buff + MAX_XFER_BUFF_SIZE;
	ast_espi->flash_tx_channel.dma_addr = ast_espi->flash_rx_channel.dma_addr + MAX_XFER_BUFF_SIZE;

	ast_espi_write(ast_espi, ast_espi->p_rx_channel.dma_addr, AST_ESPI_PCP_RX_DMA);
	ast_espi_write(ast_espi, ast_espi->p_tx_channel.dma_addr, AST_ESPI_PCP_TX_DMA);

	ast_espi_write(ast_espi, ast_espi->np_tx_channel.dma_addr, AST_ESPI_PCNP_TX_DMA);

	ast_espi_write(ast_espi, ast_espi->oob_rx_channel.dma_addr, AST_ESPI_OOB_RX_DMA);
	ast_espi_write(ast_espi, ast_espi->oob_tx_channel.dma_addr, AST_ESPI_OOB_TX_DMA);

	ast_espi_write(ast_espi, ast_espi->flash_rx_channel.dma_addr, AST_ESPI_FLASH_RX_DMA);
	ast_espi_write(ast_espi, ast_espi->flash_tx_channel.dma_addr, AST_ESPI_FLASH_TX_DMA);

	ast_espi_write(ast_espi, ast_espi_read(ast_espi, AST_ESPI_CTRL) |
				   ESPI_CTRL_FLASH_RX_DMA | ESPI_CTRL_FLASH_TX_DMA |
				   ESPI_CTRL_OOB_RX_DMA | ESPI_CTRL_OOB_TX_DMA |
				   ESPI_CTRL_PCNP_TX_DMA | ESPI_CTRL_PCP_RX_DMA | ESPI_CTRL_PCP_TX_DMA,  AST_ESPI_CTRL);

#else
	ast_espi->p_rx_channel.buff = kzalloc(MAX_XFER_BUFF_SIZE * 7, GFP_KERNEL);
	ast_espi->p_tx_channel.buff = ast_espi->p_rx_channel.buff  + MAX_XFER_BUFF_SIZE;
	ast_espi->np_tx_channel.buff = ast_espi->p_tx_channel.buff  + MAX_XFER_BUFF_SIZE;
	ast_espi->oob_rx_channel.buff = ast_espi->np_tx_channel.buff + MAX_XFER_BUFF_SIZE;
	ast_espi->oob_tx_channel.buff = ast_espi->oob_rx_channel.buff + MAX_XFER_BUFF_SIZE;
	ast_espi->flash_rx_channel.buff = ast_espi->oob_tx_channel.buff + MAX_XFER_BUFF_SIZE;
	ast_espi->flash_tx_channel.buff = ast_espi->flash_rx_channel.buff + MAX_XFER_BUFF_SIZE;
#endif

	ast_espi->irq = platform_get_irq(pdev, 0);
	if (ast_espi->irq < 0) {
		dev_err(&pdev->dev, "no irq specified\n");
		ret = -ENOENT;
		goto err_free_mem;
	}

	ret = devm_request_irq(&pdev->dev, ast_espi->irq, ast_espi_isr,
						   0, dev_name(&pdev->dev), ast_espi);
	if (ret) {
		printk("AST ESPI Unable to get IRQ");
		goto err_free_mem;
	}

	ast_espi->reset = devm_reset_control_get_exclusive(&pdev->dev, "espi");
	if (IS_ERR(ast_espi->reset)) {
		dev_err(&pdev->dev, "can't get espi reset\n");
		return PTR_ERR(ast_espi->reset);
	}

#if 1
	if (gpio_request(PIN_GPIOAC7, "GPIOAC7")) {
		printk("GPIO request failure: GPIOAC7\n");
		goto err_free_mem;
	}
	gpio_direction_input(PIN_GPIOAC7);
	gpio_set_debounce(PIN_GPIOAC7, 0x1);
	gpio_export(PIN_GPIOAC7, 1);
	gpio_irq = gpio_to_irq(PIN_GPIOAC7);        // map your GPIO to an IRQ
//	printk("gpio_to_irq %d \n", gpio_irq);
	ret = request_irq(gpio_irq, ast_espi_reset_isr, IRQF_TRIGGER_FALLING, "gpioAC7", ast_espi);
	if (ret) {
		printk("AST ESPI Unable to get IRQ %d\n", ret);
		goto err_free_mem;
	}

#else
	ast_set_gpio_debounce(PIN_GPIOAC7, 0x1);
	ast_set_gpio_debounce_timer(1, 0x100);
	irq_set_irq_type(IRQ_GPIOAC7, IRQ_TYPE_EDGE_FALLING);
	ret = request_irq(IRQ_GPIOAC7, ast_espi_reset_isr, IRQF_SHARED, "gpioAC7", ast_espi);
	if (ret) {
		printk("AST ESPI Unable to get IRQ");
		goto err_free_mem;
	}
#endif
	ast_espi_ctrl_init(ast_espi);

	ret = misc_register(&ast_espi_misc);
	if (ret) {
		printk(KERN_ERR "ESPI : failed misc_register\n");
		goto err_free_irq;
	}

	platform_set_drvdata(pdev, ast_espi);
	dev_set_drvdata(ast_espi_misc.this_device, ast_espi);

	ret = sysfs_create_group(&pdev->dev.kobj, &espi_attribute_group);
	if (ret) {
		printk(KERN_ERR "ast_espi: failed to create sysfs device attributes.\n");
		return -1;
	}

	printk(KERN_INFO "ast_espi: driver successfully loaded.\n");

	return 0;

err_free_irq:
	free_irq(ast_espi->irq, pdev);

err_free_mem:
	release_mem_region(res->start, resource_size(res));
err_free:
	kfree(ast_espi);

	return ret;
}

static int ast_espi_remove(struct platform_device *pdev)
{
	struct ast_espi_data *ast_espi;
	struct resource *res;

	ESPI_DBUG("\n");
	ast_espi = platform_get_drvdata(pdev);
	if (ast_espi == NULL)
		return -ENODEV;

	free_irq(PIN_GPIOAC7, ast_espi);
	free_irq(ast_espi->irq, ast_espi);
	iounmap(ast_espi->reg_base);
	sysfs_remove_group(&pdev->dev.kobj, &espi_attribute_group);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	release_mem_region(res->start, resource_size(res));

	kfree(ast_espi);
	return 0;
}

static const struct of_device_id ast_espi_of_matches[] = {
	{ .compatible = "aspeed,ast-espi", },
	{},
};
MODULE_DEVICE_TABLE(of, ast_espi_of_matches);

static struct platform_driver ast_espi_driver = {
	.probe		= ast_espi_probe,
	.remove		= ast_espi_remove,
	.driver         = {
		.name   = KBUILD_MODNAME,
		.of_match_table = ast_espi_of_matches,
	},
};

module_platform_driver(ast_espi_driver);

MODULE_AUTHOR("Ryan Chen <ryan_chen@aspeedtech.com>");
MODULE_DESCRIPTION("AST eSPI driver");
MODULE_LICENSE("GPL");
