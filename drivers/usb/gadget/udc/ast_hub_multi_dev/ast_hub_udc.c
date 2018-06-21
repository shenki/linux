/*
*  ast_hub_udc.c -- driver for ast udc -series USB peripheral controller
*
*  HID UDC driver for AST SOC
*
*  Copyright (C) 2012-2020  ASPEED Technology Inc.
*
*  This program is free software; you can redistribute it and/or modify
*  it under the terms of the GNU General Public License version 2 as
*  published by the Free Software Foundation.
*
*  History:
*    2016.04.26: Initial version [Ryan Chen]
*    2017.10.03: Added support for multiple devices to support USB Hub and devices behing hub [Akshay Srinivas]
*/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/ioport.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/list.h>
#include <linux/interrupt.h>
#include <linux/proc_fs.h>
#include <linux/prefetch.h>
#include <linux/clk.h>
#include <linux/usb/ch9.h>

#include <linux/usb/gadget.h>
#include <linux/of.h>
#include <linux/of_gpio.h>

#include <asm/byteorder.h>
#include <asm/io.h>
#include <asm/irq.h>

#include <linux/dma-mapping.h>
#include <linux/ctype.h>


/*************************************************************************************/
#define AST_VHUB_CTRL 				0x00
#define AST_VHUB_CONF 				0x04
#define AST_VHUB_IER 				0x08
#define AST_VHUB_ISR 				0x0C
#define AST_VHUB_EP_ACK_IER 		0x10
#define AST_VHUB_EP_NAK_IER 		0x14
#define AST_VHUB_EP_ACK_ISR 		0x18
#define AST_VHUB_EP_NAK_ISR 		0x1C
#define AST_VHUB_DEV_RESET 			0x20
#define AST_VHUB_USB_STS 			0x24
#define AST_VHUB_EP_DATA 			0x28
#define AST_VHUB_ISO_TX_FAIL		0x2C
#define AST_VHUB_EP0_CTRL			0x30
#define AST_VHUB_EP0_DATA_BUFF		0x34
#define AST_VHUB_EP1_CTRL			0x38
#define AST_VHUB_EP1_STS_CHG		0x3C

#define AST_VHUB_SETUP_DATA0		0x80
#define AST_VHUB_SETUP_DATA1		0x84

/*  ************************************************************************************/
/* AST_VHUB_CTRL 		0x00 */
#define ROOT_PHY_CLK_EN				(1 << 31)
#define ROOT_PHY_SELF_TEST_EN			(1 << 25)
#define ROOT_DN_15K_EN					(1 << 24)
#define ROOT_DP_15K_EN					(1 << 23)
#define ROOT_FIFO_DYNP_EN				(1 << 19)
#define ROOT_EP_LONG_DESC				(1 << 18)
#define ROOT_ISO_IN_NULL_RESP			(1 << 17)
#define ROOT_SPLIT_IN					(1 << 16)
#define ROOT_LOOP_TEST_PASS			(1 << 15)
#define ROOT_LOOP_TEST_FINISH			(1 << 14)
#define ROOT_BIST_TEST_PASS				(1 << 13)
#define ROOT_BIST_ON					(1 << 12)
#define ROOT_PHY_RESET_DIS				(1 << 11)
#define ROOT_TEST_MODE(x)				((x) << 8)
#define ROOT_FORCE_TIMER_HS			(1 << 7)
#define ROOT_FORCE_HS					(1 << 6)
#define ROOT_REMOTE_WAKEUP_12MS		(1 << 5)
#define ROOT_REMOTE_WAKEUP_EN			(1 << 4)
#define ROOT_AUTO_REMOTE_WAKEUP_EN	(1 << 3)
#define ROOT_STOP_CLK_IN_SUPEND		(1 << 2)
#define ROOT_UPSTREAM_FS				(1 << 1)
#define ROOT_UPSTREAM_EN				(1)

/* AST_VHUB_CONF 		0x04 */
#define CONF_GET_DMA_STS(x)				((x >> 16) & 0xff)
#define CONF_GET_DEV_ADDR(x)			(x & 0x7f)

/* AST_VHUB_IER 		0x08	*/
#define ISR_USB_CMD_DEADLOCK			(1 << 18)

#define ISR_EP_NAK						(1 << 17)
#define ISR_EP_ACK_STALL				(1 << 16)
#define ISR_DEVICE5						(1 << 13)
#define ISR_DEVICE4						(1 << 12)
#define ISR_DEVICE3						(1 << 11)
#define ISR_DEVICE2						(1 << 10)
#define ISR_DEVICE1						(1 << 9)
#define ISR_SUSPEND_RESUME				(1 << 8)
#define ISR_BUS_SUSPEND 				(1 << 7)
#define ISR_BUS_RESET 					(1 << 6)
#define ISR_HUB_EP1_IN_DATA_ACK		(1 << 5)
#define ISR_HUB_EP0_IN_DATA_NAK		(1 << 4)
#define ISR_HUB_EP0_IN_ACK_STALL		(1 << 3)
#define ISR_HUB_EP0_OUT_NAK			(1 << 2)
#define ISR_HUB_EP0_OUT_ACK_STALL		(1 << 1)
#define ISR_HUB_EP0_SETUP				(1)

/* AST_VHUB_EP_ACK_IER 		0x10 */
#define EP14_ISR							(1 << 14)
#define EP13_ISR							(1 << 13)
#define EP12_ISR							(1 << 12)
#define EP11_ISR							(1 << 11)
#define EP10_ISR							(1 << 10)
#define EP9_ISR							(1 << 9)
#define EP8_ISR							(1 << 8)
#define EP7_ISR							(1 << 7)
#define EP6_ISR							(1 << 6)
#define EP5_ISR							(1 << 5)
#define EP4_ISR							(1 << 4)
#define EP3_ISR							(1 << 3)
#define EP2_ISR							(1 << 2)
#define EP1_ISR							(1 << 1)
#define EP0_ISR							(1)

/* AST_VHUB_DEV_RESET_ISR 		0x20 */
#define DEV5_SOFT_RESET					(1 << 5)
#define DEV4_SOFT_RESET					(1 << 4)
#define DEV3_SOFT_RESET					(1 << 3)
#define DEV2_SOFT_RESET					(1 << 2)
#define DEV1_SOFT_RESET					(1 << 1)
#define ROOT_HUB_SOFT_RESET			(1)


/* AST_VHUB_EP0_CTRL			0x30 */
#define EP0_GET_RX_LEN(x)				((x >> 16) & 0x7f)
#define EP0_TX_LEN(x)						((x & 0x7f) << 8)
#define EP0_RX_BUFF_RDY					(1 << 2)
#define EP0_TX_BUFF_RDY					(1 << 1)
#define EP0_STALL						(1)

#define AST_UDC_DEV_CTRL 				0x00
#define AST_UDC_DEV_ISR				0x04
#define AST_UDC_DEV_EP0_CTRL			0x08
#define AST_UDC_DEV_EP0_DATA_BUFF	0x0C

/*  ************************************************************************************/
//#define AST_UDC_DEV_CTRL 				0x00
#define DEV_CTRL_DEV_ADDR_MASK		(0x3f << 8)
#define DEV_CTRL_IN_NAK_EN				(1 << 6)
#define DEV_CTRL_IN_ACK_STALL_EN		(1 << 5)
#define DEV_CTRL_OUT_NAK_EN			(1 << 4)
#define DEV_CTRL_OUT_ACK_STALL_EN		(1 << 3)
#define DEV_CTRL_SETUP_EN				(1 << 2)
#define DEV_CTRL_HIGH_SPEED_MODE		(1 << 1)
#define DEV_CTRL_DEV_EN				(1)

//define AST_UDC_DEV_ISR					0x04
#define DEV_CTRL_IN_NAK_INT			(1 << 4)
#define DEV_CTRL_IN_ACK_STALL_INT		(1 << 3)
#define DEV_CTRL_OUT_NAK_INT			(1 << 2)
#define DEV_CTRL_OUT_ACK_STALL_INT	(1 << 1)
#define DEV_CTRL_SETUP_INT				(1)

//#define AST_UDC_DEV_EP0_CTRL			0x08
#define DEV_EP0_GET_RX_SIZE(x)			((x >> 16) & 0x7f)
#define DEV_EP0_TX_SIZE_MASK			(0x7f << 8)
#define DEV_EP0_SET_TX_SIZE(x)			((x & 0x7f) << 8)
#define DEV_EP0_RX_BUFF_RDY			(1 << 2)
#define DEV_EP0_TX_BUFF_RDY			(1 << 1)
#define DEV_EP0_STALL					(1)

/*************************************************************************************/
#define AST_EP_CONFIG					0x00
#define AST_EP_DMA_CTRL				0x04
#define AST_EP_DMA_BUFF				0x08
#define AST_EP_DMA_STS					0x0C

/*************************************************************************************/
//#define AST_EP_CONFIG					0x00
#define EP_SET_MAX_PKT(x)				((x & 0x3ff) << 16)

#define EP_AUTO_DATA_DISABLE			(0x1 << 13)
#define EP_SET_EP_STALL					(0x1 << 12)

#define EP_SET_EP_NUM(x)				((x & 0xf) << 8)

#define EP_SET_TYPE_MASK(x)				((x) << 4)
#define EP_TYPE_BULK_IN					(0x2 << 4)
#define EP_TYPE_BULK_OUT				(0x3 << 4)
#define EP_TYPE_INT_IN					(0x4 << 4)
#define EP_TYPE_INT_OUT					(0x5 << 4)
#define EP_TYPE_ISO_IN					(0x6 << 4)
#define EP_TYPE_ISO_OUT					(0x7 << 4)

#define EP_ALLOCATED_MASK				(0x7 << 1)

#define EP_ENABLE						(1)

//#define AST_EP_DMA_STS					0x0C


//#define AST_EP_DMA_CTRL				0x04

#define EP_SINGLE_DMA_MODE			(0x1 << 1)

/*************************************************************************************/

#define DEV_PORT_RESET (1 << 0)
#define DEV_PORT_RESET_DONE (1 << 1)
#define HOST_RESUME	(1 << 2)
#define DEV_PORT_SUSPEND (1 << 3)

static struct ast_udc *root_udc;
#define IS_HUB(udc) (udc == root_udc)
struct ast_udc_request {
	struct usb_request	req;
	struct list_head		queue;
	int zero;
	unsigned mapped:1;	
};
#define DMA_LIST_MODE 1
struct ast_udc_ep {
	struct usb_ep			ep;
	struct list_head		queue;
	struct ast_udc	*udc;
	void __iomem			*ep_reg;
	unsigned			stopped:1;
	u8				ep_dir;
	void					*ep_buf;
	dma_addr_t			ep_dma;
#ifdef DMA_LIST_MODE
	dma_addr_t			ep_desc_dma;
	dma_addr_t			ep_desc_buf;
#endif
	const struct usb_endpoint_descriptor	*desc;
	int ep_pool_num;
	int status_phase;
};
#ifdef DMA_LIST_MODE
static int use_list_mode;
dma_addr_t dma_desc_buf;
dma_addr_t dma_desc_dma;
#endif

#define AST_NUM_ENDPOINTS		16
/*
 * driver is non-SMP, and just blocks IRQs whenever it needs
 * access protection for chip registers or driver state
 */
struct ast_udc {
	void __iomem			*reg;
	void __iomem			*dev_reg;
	struct clk 				*clk;	
	struct usb_gadget			gadget;
	struct ast_udc_ep			ep[AST_NUM_ENDPOINTS];
	struct usb_gadget_driver	*driver;
	unsigned			suspended:1;
	unsigned			req_pending:1;
	unsigned			wait_for_addr_ack:1;
	unsigned			wait_for_config_ack:1;
	unsigned			active_suspend:1;
        unsigned                        stopping:1;
	u8				addr;
	struct platform_device	*pdev;
	struct proc_dir_entry	*pde;
	struct usb_ctrlrequest *root_setup;	
	int					irq;
	spinlock_t			lock;
	void					*ep0_ctrl_buf;
	dma_addr_t			ep0_ctrl_dma;
	int dev_num;
	struct tasklet_struct udc_tasklet;
	volatile u32 soft_irq_status;
	struct usb_ep *used_ep[6];//assume max 6 eps will be used per gadget.
	
};
#define AST_USB_DEV_SPEED 1
#ifdef AST_USB_DEV_SPEED
int hub_speed = USB_SPEED_FULL;
EXPORT_SYMBOL(hub_speed);
#endif

static const char * const ast_ep_name[] = {
	"ep0", "ep1", "ep2", "ep3", "ep4","ep5", "ep6", "ep7", "ep8", "ep9","ep10", "ep11", "ep12", "ep13", "ep14", "ep15"};
#if 0
#define AST_UDC_DEBUG
#define AST_BUS_DEBUG
#define AST_SETUP_DEBUG
#define AST_EP_DEBUG
#define AST_ISR_DEBUG
#define DMA_LIST_DEBUG
#endif
#ifdef AST_BUS_DEBUG
#define BUS_DBG(fmt, args...) printk(KERN_DEBUG "%s() " fmt,__FUNCTION__, ## args)
#else
#define BUS_DBG(fmt, args...)
#endif

#ifdef AST_SETUP_DEBUG
//#define SETUP_DBG(fmt, args...) printk(KERN_DEBUG "%s() " fmt,__FUNCTION__, ## args)
#define SETUP_DBG(fmt, args...) printk("%s() " fmt,__FUNCTION__, ## args)
#else
#define SETUP_DBG(fmt, args...)
#endif

#ifdef AST_EP_DEBUG
//#define EP_DBG(fmt, args...) printk(KERN_DEBUG "%s() " fmt,__FUNCTION__, ## args)
#define EP_DBG(fmt, args...) printk("%s() " fmt,__FUNCTION__, ## args)

#else
#define EP_DBG(fmt, args...)
#endif

#ifdef AST_UDC_DEBUG
#define UDC_DBG(fmt, args...) printk("%s() " fmt,__FUNCTION__, ## args)
#else
#define UDC_DBG(fmt, args...)
#endif

#ifdef AST_ISR_DEBUG
//#define ISR_DBG(fmt, args...) printk(KERN_DEBUG "%s() " fmt,__FUNCTION__, ## args)
#define ISR_DBG(fmt, args...) printk("%s() " fmt,__FUNCTION__, ## args)
#else
#define ISR_DBG(fmt, args...)
#endif

#ifdef DMA_LIST_DEBUG
#define LIST_MODE_DBG(fmt, args...) printk("%s() " fmt,__FUNCTION__, ## args)
#else
#define LIST_MODE_DBG(fmt, args...)
#endif

/*-------------------------------------------------------------------------*/
#define ast_udc_read(udc, offset) \
	__raw_readl((udc)->reg + (offset))
#define ast_udc_write(udc, val, offset) \
	__raw_writel((val), (udc)->reg + (offset))

#define ast_dev_udc_read(udc, offset) \
	__raw_readl((udc)->dev_reg + (offset))
#define ast_dev_udc_write(udc, val, offset) \
	__raw_writel((val), (udc)->dev_reg + (offset))

#define ast_ep_read(ep, reg) \
	__raw_readl((ep)->ep_reg + (reg))
#define ast_ep_write(ep, val, reg) \
	__raw_writel((val), (ep)->ep_reg + (reg))
/*-------------------------------------------------------------------------*/
#ifdef EP_DONE_DEBUG
int print_stop_debug;
#endif
static void ast_udc_done(struct ast_udc_ep *ep, struct ast_udc_request *req, int status)
{
	struct ast_udc *udc = ep->udc;

	EP_DBG("%s len (%d/%d) buf %p, dir %x\n",ep->ep.name, req->req.actual, req->req.length, req->req.buf, ep->ep_dir);
#ifdef EP_DONE_DEBUG
	if(1 || print_stop_debug) {
		printk("%s len (%d/%d) buf %p, dir %x dev %d udc %p\n",ep->ep.name, req->req.actual, req->req.length, req->req.buf, ep->ep_dir, udc->dev_num, udc);
		printk("ep %p q %p req %p %p\n", ep, &ep->queue, req, &req->req);
	}
#endif
		
	if(!ep->ep_dir && strcmp(ep->ep.name, "ep0") == 0) {
		//printk("Unmap\n");
		//usb_gadget_unmap_request(&udc->gadget, &req->req, 0);
	}
	list_del_init(&req->queue);
//	list_del(&req->queue);
	
	if (req->req.status == -EINPROGRESS)
		req->req.status = status;
	else
		status = req->req.status;
	
	if (status && status != -ESHUTDOWN)
		EP_DBG("%s done %p, status %d\n", ep->ep.name, req, status);

	spin_unlock(&udc->lock);
	usb_gadget_giveback_request(&ep->ep, &req->req);
	spin_lock(&udc->lock);
#ifdef EP_DONE_DEBUG
	if(1 || print_stop_debug) {
		printk("done end %d\n", list_empty(&ep->queue));
	}
#endif

}
static void ast_udc_nuke(struct ast_udc_ep *ep, int status)
{
        /* Sanity check */
        if (&ep->queue == NULL)
                return;

        while (!list_empty(&ep->queue)) {
                struct ast_udc_request *req;
#ifdef EP_DONE_DEBUG
		if(print_stop_debug)
			printk("Nuking ep %x %s req\n", ep->ep.address, ep->ep.name);
#endif
                req = list_entry(ep->queue.next, struct ast_udc_request,
                                queue);
                ast_udc_done(ep, req, status);
        }
}

/**
 * Stop activity on all endpoints.
 * Device controller for which EP activity is to be stopped.
 *
 * All the endpoints are stopped and any pending transfer requests if any on
 * the endpoint are terminated.
 */
static void ast_udc_stop_activity(struct ast_udc *udc)
{
	int epnum;
	struct ast_udc_ep *ep;

	for (epnum = 0; epnum < 2; epnum++) {
		ep = &udc->ep[epnum];
		ep->stopped = 1;
		ast_udc_nuke(ep, -ESHUTDOWN);
	}
}


static void ast_dev_udc_stop_activity(struct ast_udc *udc, int inform_higher_layer)
{
	int epnum;
	struct ast_udc_ep *ep;

	for (epnum = 0; epnum < AST_NUM_ENDPOINTS; epnum++) {
		ep = &udc->ep[epnum];
		ep->stopped = 1;
		ast_udc_nuke(ep, -ESHUTDOWN);
	}
	if (inform_higher_layer && udc->driver && udc->driver->reset) {
		spin_unlock(&udc->lock);
		udc->driver->reset(&udc->gadget);
		spin_lock(&udc->lock);
	}		
	
}

/*-------------------------------------------------------------------------*/
static struct ast_udc*ep_to_udc[AST_NUM_ENDPOINTS];
static int ast_udc_ep_enable(struct usb_ep *_ep,
				const struct usb_endpoint_descriptor *desc)
{
	struct ast_udc_ep	*ep = container_of(_ep, struct ast_udc_ep, ep);
	struct ast_udc *udc = ep->udc;
	u16	maxpacket = usb_endpoint_maxp(desc);
	u8	epnum = usb_endpoint_num(desc);	
	unsigned long	flags;
	u32 		ep_conf = 0;
	u8		type;
	u8		dir_in, num = 0;

	EP_DBG("%s, set ep #%d, maxpacket %d ,wmax %d \n",
		ep->ep.name, epnum, maxpacket, le16_to_cpu(desc->wMaxPacketSize));
	if (isdigit (_ep->name [2]))
		num = simple_strtoul (&_ep->name[2], NULL, 10);

	EP_DBG("ep->ep_pool_num %d dev %d ep_to_udc[%d] %p udc %p ep_reg %p\n",
		ep->ep_pool_num, udc->dev_num, num, ep_to_udc[num], udc, ep->ep_reg);
	if (!_ep || !ep || !desc || desc->bDescriptorType != USB_DT_ENDPOINT) {
		printk("bad ep or descriptor %s %d , maxpacket %d, ep maxpacket %d \n", _ep->name, desc->bDescriptorType, maxpacket, ep->ep.maxpacket);
		return -EINVAL;
	}

	if (!udc->driver) {
		printk("bogus device state\n");
		//return -ESHUTDOWN;
	}
	UDC_DBG("Enable:ep_pool_num %d dev %d ep_to_udc[%d] %p udc %p ep_reg %p ep %p %p %p\n",
		ep->ep_pool_num, udc->dev_num, num, ep_to_udc[num], udc, ep->ep_reg, ep, &udc->ep[ep->ep_pool_num], &udc->ep[ep->ep_pool_num + 1]);
	spin_lock_irqsave(&udc->lock, flags);
	ep_to_udc[num] = udc;
	ep->desc = desc;
	ep->stopped = 0;
	ep->ep.maxpacket = maxpacket;
	ast_ep_write(ep, 0, AST_EP_CONFIG);//Reset the endpoint
 	if(maxpacket > 1024) {
		printk("TODO check size ~~~~ \n");
		maxpacket = 1024;
	}
	
	if(maxpacket == 1024) {
		ep_conf = 0;
	} else {
		ep_conf = EP_SET_MAX_PKT(maxpacket);
	}
	
	ep_conf |= EP_SET_EP_NUM(epnum);
	if(IS_HUB(udc)) {
		ep_conf &= ~(0x7 << 1);
	} else  {
		ep_conf &= ~(0x7 << 1);
		ep_conf |= (udc->dev_num +1) << 1;
	}
	type = usb_endpoint_type(desc);
	dir_in = usb_endpoint_dir_in(desc);
	ep->ep_dir = dir_in;

	EP_DBG("epnum %d, type %d, dir_in %d \n", epnum, type, dir_in);
	switch(type) {
		case USB_ENDPOINT_XFER_ISOC:
			if(dir_in) {
				ep_conf |= EP_TYPE_ISO_IN;
			} else {
				ep_conf |= EP_TYPE_ISO_OUT;
			}
			break;
		case USB_ENDPOINT_XFER_BULK:
			if(dir_in) {
				ep_conf |= EP_TYPE_BULK_IN;
			} else {
				ep_conf |= EP_TYPE_BULK_OUT;
			}			
			break;
		case USB_ENDPOINT_XFER_INT:
			if(dir_in) {
				ep_conf |= EP_TYPE_INT_IN;
			} else {
				ep_conf |= EP_TYPE_INT_OUT;
			}			
			break;
	}

	ast_ep_write(ep, 0x4, AST_EP_DMA_CTRL);
	ast_ep_write(ep, 0x2, AST_EP_DMA_CTRL); 
	ast_ep_write(ep, 0, AST_EP_DMA_STS);
	ast_ep_write(ep, ep_conf | EP_ENABLE, AST_EP_CONFIG);
	ep->status_phase = 0;
	spin_unlock_irqrestore(&udc->lock, flags);

	return 0;
}

static int ast_udc_ep_disable (struct usb_ep * _ep)
{
	struct ast_udc_ep	*ep = container_of(_ep, struct ast_udc_ep, ep);
	struct ast_udc		*udc = ep->udc;	

	unsigned long	flags;

	EP_DBG("%s\n", _ep->name);

	spin_lock_irqsave(&udc->lock, flags);

	ep->ep.desc = NULL;
	ep->stopped = 1;

	ast_udc_nuke(ep, -ESHUTDOWN);

	ast_ep_write(ep, 0, AST_EP_CONFIG);
	spin_unlock_irqrestore(&udc->lock, flags);
	return 0;
}

static struct usb_request *
ast_udc_ep_alloc_request(struct usb_ep *_ep, gfp_t gfp_flags)
{
	struct ast_udc_request *req;
	EP_DBG("%s\n", _ep->name);

	req = kzalloc(sizeof (struct ast_udc_request), gfp_flags);
	if (!req) {
		printk("%s %d error\n", __FUNCTION__, __LINE__);
		return NULL;
	}
	INIT_LIST_HEAD(&req->queue);
	UDC_DBG("%s %d\n", __FUNCTION__, __LINE__);
	return &req->req;
}

static void ast_udc_ep_free_request(struct usb_ep *_ep, struct usb_request *_req)
{
	struct ast_udc_request *req;
	EP_DBG("%s\n", _ep->name);

	req = container_of(_req, struct ast_udc_request, req);
	kfree(req);
}

static void ast_udc_ep_dma(struct ast_udc_ep *ep, struct ast_udc_request *req)
{
	u16	tx_len;
#ifdef DMA_LIST_MODE
	int do_dma = 0;
#endif

	if((req->req.length - req->req.actual) > ep->ep.maxpacket) {
		tx_len = ep->ep.maxpacket; 
	} else {
		tx_len = req->req.length - req->req.actual;
	}

#if 1
	if(ep->ep_dir) {
//		u8 * buf = req->req.buf;
		EP_DBG("%s : tx dma %x - len %d:(%d/%d) :",ep->ep.name, req->req.dma + req->req.actual, tx_len, req->req.actual, req->req.length);
#if 0
		for(i = 0; i < tx_len; i++)
			printk(" %x", buf[i]);
		printk("\n");
#else
//		printk("\n");
#endif 			
	} else {
		EP_DBG("%s : rx dma %x - len %d:(%d/%d) \n",ep->ep.name, req->req.dma + req->req.actual, tx_len, req->req.actual, req->req.length);		
	}
#endif

#if 1
	if(tx_len > 1024) UDC_DBG("*************************************************8\n");
	EP_DBG("dma: %s : len : %d dir %x ep_reg %p\n", ep->ep.name, tx_len, ep->ep_dir, ep->ep_reg);
#ifdef DMA_LIST_MODE
	if(1 || req->req.length > 512)
		LIST_MODE_DBG("Sdma: %s : len : %d dir %x ep_reg %p req->req.length %d req->req.actual %d\n", ep->ep.name, tx_len, ep->ep_dir, ep->ep_reg, req->req.length, req->req.actual);
	if(use_list_mode && req->req.length >= 0 && ((req->req.dma % 4) == 0))
		do_dma = 1;
	if(use_list_mode && ((req->req.dma % 4) != 0) && req->req.length >= 0 && ep->ep_dir)
		do_dma = 1;
	if(do_dma){
		int i = 0;
		volatile u32 *temp;
		int num_des = 0;
		int per_desc_len;
		int remaining_len = 0;
		int long_pkt_mode = 0;
		if(ep->ep_dir && long_pkt_mode)
			per_desc_len = 3584;
		else
			per_desc_len = 512;
		tx_len = req->req.length;
		num_des = req->req.length/(per_desc_len);//7 * 512
		if((req->req.length%per_desc_len))
			num_des++;
		if(req->req.length == 0)
			num_des = 1;
		temp = (void*)ep->ep_desc_buf;
		ast_ep_write(ep, 0, AST_EP_DMA_CTRL);
		ast_ep_write(ep, 0, AST_EP_DMA_STS);

		ast_ep_write(ep, 0x4, AST_EP_DMA_CTRL);
		ast_ep_write(ep, 0, AST_EP_DMA_STS);
		ast_ep_write(ep, ep->ep_desc_dma, AST_EP_DMA_BUFF);
		ast_ep_write(ep, 0x1 | (long_pkt_mode << 3), AST_EP_DMA_CTRL);
		if((req->req.dma % 4) != 0 && !ep->ep_dir) {
			printk("Sdma: %s : len : %d dir %x ep_reg %p req->req.length %d req->req.actual %d\n",
				ep->ep.name, tx_len, ep->ep_dir, ep->ep_reg, req->req.length, req->req.actual);
			panic("DMA NOt ALIGNED> for OUT EP?!!\n");
		}

		LIST_MODE_DBG("num_des %d ep->ep_desc_dma %x ep->ep_dir %x\n", num_des, ep->ep_desc_dma, ep->ep_dir);
		for(i = 0;i < num_des;i++) {
			*temp = req->req.dma + (i * per_desc_len);
			temp++;
			if(ep->ep_dir)
				*temp = min(tx_len, per_desc_len);
			else
				*temp = 0;
			temp++;
			LIST_MODE_DBG("%s : rx dma %x - len %d:(%d/%d) \n",ep->ep.name, req->req.dma + (i*per_desc_len), min(tx_len, per_desc_len), req->req.actual, req->req.length);
			tx_len -= min(tx_len, per_desc_len);
		}
		temp--;
		*temp |= ( 1 << 31);//Enable Interrupt generation for Last desc
		if(ep->ep_dir)
			ast_ep_write(ep, num_des - 0, AST_EP_DMA_STS);
		else
			ast_ep_write(ep, (0x200 << 16) | num_des - 0, AST_EP_DMA_STS);
		return;
	}
#endif
	if((req->req.dma % 4) != 0) {
//		printk("1: %s : %x len (%d/%d) dir %x\n", ep->ep.name, req->req.dma, req->req.actual, req->req.length, ep->ep_dir);
		if((ep->ep_dir) && (!req->req.actual)) {
			memcpy(ep->ep_buf, req->req.buf, req->req.length);
		}
		ast_ep_write(ep, ep->ep_dma + req->req.actual, AST_EP_DMA_BUFF);
	} else {
		ast_ep_write(ep, req->req.dma + req->req.actual, AST_EP_DMA_BUFF);
	}
	//trigger
	ast_ep_write(ep, tx_len << 16, AST_EP_DMA_STS);
	ast_ep_write(ep, tx_len << 16 | 0x1, AST_EP_DMA_STS);		

#else
	ast_ep_write(ep, req->req.dma + req->req.actual, AST_EP_DMA_BUFF);
	//trigger
	ast_ep_write(ep, tx_len << 16, AST_EP_DMA_STS);
	ast_ep_write(ep, tx_len << 16 | 0x1, AST_EP_DMA_STS);	
#endif

}

static void ast_udc_ep0_queue(struct ast_udc_ep *ep, struct ast_udc_request *req)
{
	struct ast_udc		*udc = ep->udc;
	u16	tx_len;

//	if(req->req.length > 64) printk("ep0 queue len %d ***************************\n", req->req.length);

	if((req->req.length - req->req.actual) > ep->ep.maxpacket) {
		tx_len = ep->ep.maxpacket; 
	} else {
		tx_len = req->req.length - req->req.actual;
	}
	ast_udc_write(udc, req->req.dma + req->req.actual, AST_VHUB_EP0_DATA_BUFF);
	
	if(ep->ep_dir) {
		SETUP_DBG("ep0 in addr buf %x, dma %x , txlen %d:(%d/%d) ,dir %d \n", (u32)req->req.buf, req->req.dma + req->req.actual, tx_len, req->req.actual, req->req.length, ep->ep_dir);
		req->req.actual += tx_len;		
		ast_udc_write(udc, EP0_TX_LEN(tx_len), AST_VHUB_EP0_CTRL);
		ast_udc_write(udc, EP0_TX_LEN(tx_len) | EP0_TX_BUFF_RDY, AST_VHUB_EP0_CTRL);
	} else {
		SETUP_DBG("ep0 out ~~ addr buf %x, dma %x , (%d/%d) ,dir %d \n", (u32)req->req.buf, req->req.dma + req->req.actual, req->req.actual, req->req.length, ep->ep_dir);
		if(!req->req.length) {
			ast_udc_write(udc, EP0_TX_BUFF_RDY, AST_VHUB_EP0_CTRL);
			ep->ep_dir = 0x80;
		} else {
			ast_udc_write(udc, EP0_RX_BUFF_RDY, AST_VHUB_EP0_CTRL); 
		}

	}
}


static void ast_udc_dev_ep0_queue(struct ast_udc_ep *ep, struct ast_udc_request *req)
{
	struct ast_udc		*udc = ep->udc;
	u16	tx_len;
	u32 temp;
//	if(req->req.length > 64) printk("ep0 queue len %d ***************************\n", req->req.length);

	if((req->req.length - req->req.actual) > ep->ep.maxpacket) {
		tx_len = ep->ep.maxpacket; 
	} else {
		tx_len = req->req.length - req->req.actual;
	}

	req->zero = req->req.zero;
	ast_dev_udc_write(udc, req->req.dma + req->req.actual, 0xC);//DEV0C
	temp = ast_dev_udc_read(udc, 0x8);
	if(temp & EP0_TX_BUFF_RDY) {
		printk("DEV08 %x not expected EP0_TX_BUFF_RDY req %p %p\n", temp, req, &req->req);
		printk("ep0 queue len %d dev %d dir %x ***************************\n", req->req.length, udc->dev_num,ep->ep_dir);
		mdelay(4000);
	}
	if(temp & EP0_RX_BUFF_RDY) {
		printk("DEV08 %x not expected EP0_RX_BUFF_RDY req %p %p\n", temp, req, &req->req);
		printk("ep0 queue len %d dev %d dir %x ***************************\n", req->req.length, udc->dev_num,ep->ep_dir);
		mdelay(4000);
	}

	if(ep->ep_dir) {
		SETUP_DBG("ep0 in addr buf %x, dma %x , txlen %d:(%d/%d) ,dir %d dev %d\n",
			(u32)req->req.buf, req->req.dma + req->req.actual, tx_len, req->req.actual, req->req.length, ep->ep_dir, udc->dev_num);
#ifdef EP_DONE_DEBUG
		printk("ep0 in txlen %d:(%d/%d) dev %d dev08 %x status_phase %d\n",
			tx_len, req->req.actual, req->req.length, udc->dev_num, ast_dev_udc_read(udc, 0x8), ep->status_phase);
#endif
		req->req.actual += tx_len;		
		if(req->req.zero && req->req.length == 0) {
                        UDC_DBG("IN::req->req.zero is set and req->req.length is 0? dev %d\n", udc->dev_num);
			UDC_DBG("type : %x, req : %x, val : %x, idx: %x, len : %d  \n", 
				udc->root_setup->bRequestType, 
				udc->root_setup->bRequest, 
				udc->root_setup->wValue,
				udc->root_setup->wIndex,
				udc->root_setup->wLength);
		}
		if(req->req.length == 0) {
			ep->status_phase = 1;
			ast_dev_udc_write(udc, EP0_RX_BUFF_RDY, 0x8); 
			ep->ep_dir = 0;
		} else {
			ast_dev_udc_write(udc, EP0_TX_LEN(tx_len), 0x8);//DEV08
			ast_dev_udc_write(udc, EP0_TX_LEN(tx_len) | EP0_TX_BUFF_RDY, 0x8);
		}
	} else {
		SETUP_DBG("ep0 out ~~ addr buf %x, dma %x , (%d/%d) ,dir %d dev %d ep->status_phase %d\n",
			(u32)req->req.buf, req->req.dma + req->req.actual, req->req.actual, req->req.length, ep->ep_dir,udc->dev_num, ep->status_phase);
#ifdef EP_DONE_DEBUG
		printk("ep0 out (%d/%d) dev %d dev08 %x\n",
			req->req.actual, req->req.length, udc->dev_num, ast_dev_udc_read(udc, 0x8));
#endif
		if(!req->req.length) {
                    ep->status_phase = 1;
			ast_dev_udc_write(udc, EP0_TX_BUFF_RDY, 0x8);
			ep->ep_dir = 0x80;
		} else {
			ast_dev_udc_write(udc, EP0_RX_BUFF_RDY, 0x8); 
		}
	}
}
static int hub_remote_wakeup_set;
static int ast_udc_dev_ep_queue(struct usb_ep *_ep,
			struct usb_request *_req, gfp_t gfp_flags)
{
	struct ast_udc_request	*req = container_of(_req, struct ast_udc_request, req);
	struct ast_udc_request  *req1;
	struct ast_udc_ep		*ep = container_of(_ep, struct ast_udc_ep, ep);
	struct ast_udc		*udc = ep->udc;
	unsigned long		flags;
	if(unlikely(udc->stopping)) {
		printk("Error %s %d\n", __FUNCTION__, __LINE__);
		return -EINVAL;
	}
	if (unlikely(!_req || !_req->complete || !_req->buf || !_ep)) {
		printk("Error %s %d\n", __FUNCTION__, __LINE__);
		return -EINVAL;
	}
	if(hub_remote_wakeup_set) {
#ifndef AUTO_REMOTE_WAKEUP
                u32 temp;
#endif
		UDC_DBG("Dev Initiate Remote wakeup dev_num %d\n", udc->dev_num);
#ifndef AUTO_REMOTE_WAKEUP
                temp = ast_udc_read(udc, AST_VHUB_CTRL);
                temp |= (1 << 4);
                ast_udc_write(udc, temp, AST_VHUB_CTRL);
#endif

	}

	if(ep->stopped) {
		UDC_DBG("%s : is stop \n", _ep->name);
		return 1;
	}
	EP_DBG("%s : len : %d ep->ep_dir %d\n", _ep->name, _req->length, ep->ep_dir);
#ifdef EP_DONE_DEBUG
	printk("%s : len : %d ep->ep_dir %d dev %d QE %d\n", _ep->name, _req->length, ep->ep_dir, udc->dev_num, list_empty(&ep->queue));
	printk("ep %p q %p req %p %p actual %d _req %p\n", ep, &ep->queue, req, &req->req, _req->actual, _req);
#endif
	spin_lock_irqsave(&udc->lock, flags);
	if(!list_empty(&ep->queue)) {
		req1 = list_entry(ep->queue.next, struct ast_udc_request,
                                queue);
		if(req1 == req) {
			printk(KERN_CRIT "\n\nEarlier request didnot finish deleting from ep q list\n\n");
		        printk(KERN_CRIT "%s : len : %d ep->ep_dir %d dev %d QE %d\n", _ep->name, _req->length, ep->ep_dir, udc->dev_num, list_empty(&ep->queue));
       			printk("ep %p q %p req %p %p actual %d _req %p\n", ep, &ep->queue, req, &req->req, _req->actual, _req);
			list_del_init(&req->queue);
		}
	}
	list_add_tail(&req->queue, &ep->queue);
	req->req.actual = 0;
	req->req.status = -EINPROGRESS;

	if(usb_gadget_map_request(&udc->gadget, &req->req, ep->ep_dir)) {
		printk("map ERROR ~~ \n");
		spin_unlock_irqrestore(&udc->lock, flags);
		return 1;
	}

	if (ep->ep.desc == NULL) {	/* ep0 */
		if((req->req.dma % 4) != 0) {printk("EP0 dma error ~~~~ dma %x\n", req->req.dma); while(1); }
		ast_udc_dev_ep0_queue(ep, req);
	} else {
		if (list_is_singular(&ep->queue)) {
//			if(ep->ep.name == "ep1") printk("%s dma req %x \n", ep->ep.name, req);
			ast_udc_ep_dma(ep, req);
		} else {
//			printk("%s just add req %x buf %x \n", ep->ep.name, req, req->req.buf);
		}
	}

	spin_unlock_irqrestore(&udc->lock, flags);
	
	return 0;
}

static int ast_udc_ep_queue(struct usb_ep *_ep,
			struct usb_request *_req, gfp_t gfp_flags)
{
	struct ast_udc_request	*req = container_of(_req, struct ast_udc_request, req);
	struct ast_udc_ep		*ep = container_of(_ep, struct ast_udc_ep, ep);
	struct ast_udc		*udc = ep->udc;
	unsigned long		flags;

	if (unlikely(!_req || !_req->complete || !_req->buf || !_ep)) {
		printk("Error %s %d\n", __FUNCTION__, __LINE__);
		return -EINVAL;
	}

	if(ep->stopped) {
		printk("%s : is stop \n", _ep->name);
		return 1;
	}
	if(hub_remote_wakeup_set) {
#ifndef AUTO_REMOTE_WAKEUP
		u32 temp;
#endif
		UDC_DBG("Hub Initiating Remote wakeup\n");
#ifndef AUTO_REMOTE_WAKEUP
		temp = ast_udc_read(udc, AST_VHUB_CTRL);
		temp |= (1 << 4);
		ast_udc_write(udc, temp, AST_VHUB_CTRL);
#endif
	}

	EP_DBG("%s : len : %d buf %x\n", _ep->name, _req->length, *(int*)_req->buf);
	if(!list_empty(&req->queue)) {
		udc = ep->udc;
		printk("List not empty EP Q Hub udc %p AST_VHUB_EP1_STS_CHG %x\n", udc, ast_udc_read(udc, AST_VHUB_EP1_STS_CHG));
		mdelay(2000);
		return -EBUSY;
	}
	if (!udc->driver) {
		return -ESHUTDOWN;
	}

	spin_lock_irqsave(&udc->lock, flags);

	list_add_tail(&req->queue, &ep->queue);

	req->req.actual = 0;
	req->req.status = -EINPROGRESS;

	if(usb_gadget_map_request(&udc->gadget, &req->req, ep->ep_dir)) {
		printk("map ERROR ~~ \n");
		spin_unlock_irqrestore(&udc->lock, flags);
		return 1;
	}

	if (ep->ep.desc == NULL) {	/* ep0 */
		if((req->req.dma % 4) != 0) {printk("EP0 dma error ~~~~ dma %x\n", req->req.dma); while(1); }
		ast_udc_ep0_queue(ep, req);
	} else {
		if (list_is_singular(&ep->queue)) {
//			if(ep->ep.name == "ep1") printk("%s dma req %x \n", ep->ep.name, req);
			//ast_udc_ep_dma(ep, req);
			//ast_ep_write(ep, _req->length << 16, AST_EP_DMA_STS);
			ast_udc_write(udc, *(u8*)_req->buf, AST_VHUB_EP1_STS_CHG);
			UDC_DBG("wrote status change AST_VHUB_EP1_STS_CHG %x\n",ast_udc_read(udc, AST_VHUB_EP1_STS_CHG)); 
		} else {
//			printk("%s just add req %x buf %x \n", ep->ep.name, req, req->req.buf);
		}
	}

	spin_unlock_irqrestore(&udc->lock, flags);
	
	return 0;
}

static int ast_udc_ep_dequeue(struct usb_ep *_ep, struct usb_request *_req)
{
        struct ast_udc_ep               *ep = container_of(_ep, struct ast_udc_ep, ep);
        struct ast_udc_request  *req;
        unsigned long           flags;
        struct ast_udc          *udc = ep->udc;
        u32             ep_conf = 0;
        u32 data_toggle;

        UDC_DBG("%s \n", _ep->name);

        if (!_ep || ep->ep.name == ast_ep_name[0])
                return -EINVAL;

        spin_lock_irqsave(&udc->lock, flags);

        /* make sure it's actually queued on this endpoint */
        list_for_each_entry (req, &ep->queue, queue) {
                if (&req->req == _req) {
                        list_del_init(&req->queue);
                        _req->status = -ECONNRESET;
                        break;
                }
        }
        if (&req->req != _req) {
                spin_unlock_irqrestore(&udc->lock, flags);
                return -EINVAL;
        }
        data_toggle = ast_ep_read(ep, AST_EP_DMA_STS);
        ep_conf = ast_ep_read(ep, AST_EP_DMA_CTRL);
        //reset the DMA state machine and release buffer
        ast_ep_write(ep, ep_conf | ( 1 << 2), AST_EP_DMA_CTRL);
        ep_conf = ast_ep_read(ep, AST_EP_CONFIG);//copy ep configuration
        ast_ep_write(ep, 0, AST_EP_CONFIG);//disable/reset endpoint
        ast_udc_write(udc, 0x0, AST_VHUB_EP1_CTRL);
        data_toggle = data_toggle >> 28;
        data_toggle = data_toggle & 0x3;
        if(data_toggle == 0x2) {
                data_toggle = 1;
        }else if(data_toggle != 0) {
                printk("Unxpected data_toggle %x\n", data_toggle);
        }
        ast_ep_write(ep, ep_conf, AST_EP_CONFIG);//copy back endpoint conf
        ast_udc_write(udc, 0x1, AST_VHUB_EP1_CTRL);
        ast_udc_write(udc, ep->ep_pool_num, AST_VHUB_EP_DATA);
        ast_udc_write(udc, ep->ep_pool_num | (data_toggle << 8), AST_VHUB_EP_DATA);
        ep_conf = ast_ep_read(ep, AST_EP_CONFIG);
        UDC_DBG("DQ:ep_conf %x AST_EP_DMA_CTRL %x AST_EP_DMA_STS %x AST_VHUB_EP1_CTRL %x AST_VHUB_EP1_STS_CHG %x data_toggle %x epnum %d\n",
                ep_conf, ast_ep_read(ep, AST_EP_DMA_CTRL), ast_ep_read(ep, AST_EP_DMA_STS),ast_udc_read(udc, AST_VHUB_EP1_CTRL), ast_udc_read(udc,AST_VHUB_EP1_STS_CHG),
                data_toggle, ep->ep_pool_num);
        ast_udc_done(ep, req, -ESHUTDOWN);
        spin_unlock_irqrestore(&udc->lock, flags);

        return 0;
}
static int ast_udc_dev_ep_dequeue(struct usb_ep *_ep, struct usb_request *_req)
{
	struct ast_udc_ep		*ep = container_of(_ep, struct ast_udc_ep, ep);
	struct ast_udc_request	*req;
	unsigned long		flags;
	struct ast_udc		*udc = ep->udc;

	UDC_DBG("%s \n", _ep->name);

	if (!_ep || ep->ep.name == ast_ep_name[0])
		return -EINVAL;

	spin_lock_irqsave(&udc->lock, flags);

	/* make sure it's actually queued on this endpoint */
	list_for_each_entry (req, &ep->queue, queue) {
		if (&req->req == _req) {
			list_del_init(&req->queue);
			_req->status = -ECONNRESET;			
			break;
		}
	}
	if (&req->req != _req) {
		spin_unlock_irqrestore(&udc->lock, flags);
		return -EINVAL;
	}

	ast_udc_done(ep, req, -ESHUTDOWN);
	spin_unlock_irqrestore(&udc->lock, flags);

	return 0;
}

static int ast_udc_ep_set_halt(struct usb_ep *_ep, int value)
{
	struct ast_udc_ep	*ep = container_of(_ep, struct ast_udc_ep, ep);
	struct ast_udc	*udc = ep->udc;
	unsigned long	flags;
	int		status = -EAGAIN;

	UDC_DBG("%s \n", _ep->name);
	if (!_ep) {
		printk("ast_udc_ep_set_halt _ep NULL?\n");
		return -EINVAL;
	}
	UDC_DBG("%s : %d\n", _ep->name, value);

	spin_lock_irqsave(&udc->lock, flags);
	if(!list_empty(&ep->queue)) {
		printk("Set Halt:Ep Busy retry set halt later\n");
		goto done;
	}
	status = 0;

	if(_ep->name == ast_ep_name[0]) {
		if(value)
			ast_udc_write(udc, ast_udc_read(udc, AST_VHUB_EP0_CTRL) | EP0_STALL, AST_VHUB_EP0_CTRL);
		else
			ast_udc_write(udc, ast_udc_read(udc, AST_VHUB_EP0_CTRL) & ~EP0_STALL, AST_VHUB_EP0_CTRL);
	} else {
		if(value) {
			ast_ep_write(ep, ast_ep_read(ep, AST_EP_CONFIG) | EP_SET_EP_STALL, AST_EP_CONFIG);
		} else {
			ast_ep_write(ep, ast_ep_read(ep, AST_EP_CONFIG) & ~EP_SET_EP_STALL, AST_EP_CONFIG);
			UDC_DBG("Clear Stall for eppool %d ep %p %p\n", ep->ep_pool_num, &udc->ep[ep->ep_pool_num + 1],ep);
			ast_udc_write(udc, (ep->ep_pool_num - 1), AST_VHUB_EP_DATA);
 		}
	}
	ep->stopped = value ? 1:0;
	
done:
	spin_unlock_irqrestore(&ep->udc->lock, flags);
	return status;
}

static const struct usb_ep_ops ast_udc_ep_ops = {
	.enable		= ast_udc_ep_enable,
	.disable		= ast_udc_ep_disable,
	.alloc_request	= ast_udc_ep_alloc_request,
	.free_request	= ast_udc_ep_free_request,
	.queue		= ast_udc_ep_queue,
	.dequeue		= ast_udc_ep_dequeue,
	.set_halt		= ast_udc_ep_set_halt,
	/* there's only imprecise fifo status reporting */
};

static const struct usb_ep_ops ast_udc_dev_ep_ops = {
	.enable		= ast_udc_ep_enable,
	.disable		= ast_udc_ep_disable,
	.alloc_request	= ast_udc_ep_alloc_request,
	.free_request	= ast_udc_ep_free_request,
	.queue		= ast_udc_dev_ep_queue,
	.dequeue		= ast_udc_dev_ep_dequeue,
	.set_halt		= ast_udc_ep_set_halt,
	/* there's only imprecise fifo status reporting */
};


/*************************************************************************************************************************************************/
void ast_udc_ep0_rx(struct ast_udc *udc)
{
	SETUP_DBG("\n");

	ast_udc_write(udc, udc->ep0_ctrl_dma, AST_VHUB_EP0_DATA_BUFF);
	//trigger
	ast_udc_write(udc, EP0_RX_BUFF_RDY, AST_VHUB_EP0_CTRL); 		

}

void ast_dev_udc_ep0_rx(struct ast_udc *udc)
{
	SETUP_DBG("\n");

	ast_dev_udc_write(udc, udc->ep0_ctrl_dma, 0xC);
	//trigger
	ast_dev_udc_write(udc, EP0_RX_BUFF_RDY, 0x8); 		

}

void ast_udc_ep0_tx(struct ast_udc *udc)
{
	SETUP_DBG("\n");

	ast_udc_write(udc, udc->ep0_ctrl_dma, AST_VHUB_EP0_DATA_BUFF);
	//trigger
	ast_udc_write(udc, EP0_TX_BUFF_RDY, AST_VHUB_EP0_CTRL); 		

}

void ast_dev_udc_ep0_tx(struct ast_udc *udc)
{
	SETUP_DBG("\n");

	ast_dev_udc_write(udc, udc->ep0_ctrl_dma, 0xC);
	//trigger
	ast_dev_udc_write(udc, EP0_TX_BUFF_RDY, 0x8); 		

}

void ast_udc_ep0_out(struct ast_udc *udc)
{
	struct ast_udc_ep *ep = &udc->ep[0];
	struct ast_udc_request	*req;
	u16 rx_len = EP0_GET_RX_LEN(ast_udc_read(udc, AST_VHUB_EP0_CTRL));	
	u8 *buf;

	SETUP_DBG("\n");

	if (list_empty(&ep->queue)) {
		return;
	} else {
		req = list_entry(ep->queue.next, struct ast_udc_request, queue);
	}

	buf = req->req.buf;

	req->req.actual += rx_len;
	
	if((rx_len < ep->ep.maxpacket) || (req->req.actual == req->req.length)) {
		ast_udc_ep0_tx(udc);
		if(!ep->ep_dir) {
			ast_udc_done(ep, req, 0);
		}		
	} else {
		if(rx_len > req->req.length) {
//			printk("ep0 out Check %d %d %d dir %x \n",req->req.length,  rx_len,  ep->ep.maxpacket, ep->ep_dir);
#if 0
			printk("error ~~~~~ ~~~~~~ `~~~~~ %x \n", ast_udc_read(udc, AST_VHUB_EP0_CTRL));
			printk("%x \n", ast_udc_read(udc, AST_VHUB_EP0_CTRL));
			for(i = 0; i < rx_len; i++) {
				printk("%x ", buf[i]);
			}
			printk("\n");
#endif			
			//Issue Fix 
			printk("Issue Fix (%d/%d)\n", req->req.actual, req->req.length);
			ast_udc_ep0_tx(udc);
			ast_udc_done(ep, req, 0);
			return;
		}	
		ep->ep_dir = 0;
		ast_udc_ep0_queue(ep, req);
	}
}

void ast_dev_udc_ep0_out(struct ast_udc *udc)
{
	struct ast_udc_ep *ep = &udc->ep[0];
	struct ast_udc_request	*req;
	u16 rx_len = EP0_GET_RX_LEN(ast_dev_udc_read(udc, 0x8));	
	u8 *buf;

	SETUP_DBG("dev %d rx_len %d dev08 %x dev0c %x\n", udc->dev_num, rx_len,ast_dev_udc_read(udc, 0x8), ast_dev_udc_read(udc, 0xc));
#ifdef EP_DONE_DEBUG
	printk("ep0out: ");
#endif
	if (list_empty(&ep->queue)) {
		return;
	} else {
		req = list_entry(ep->queue.next, struct ast_udc_request, queue);
	}
	buf = req->req.buf;
#ifdef EP_DONE_DEBUG
	printk("(%d/%d) DEV08 %x dev %d ", req->req.actual, req->req.length, ast_dev_udc_read(udc, 0x8),udc->dev_num);
	printk("SP %d DIR %x zero %d\n\n", ep->status_phase, ep->ep_dir, req->zero);
#endif
	if(!ep->status_phase && (rx_len == 0)) {
		printk(KERN_CRIT "ISSUEOUT2:rx_len %d status phase Not set actual %d len %d req->zero %d\n", rx_len,
			req->req.actual, req->req.length, req->zero);
	}
	if(!ep->status_phase) {
		req->req.actual += rx_len;
	} else {
		if(rx_len) {
			printk(KERN_CRIT "ISSUEOUT:rx_len %d status phase set actual %d len %d \n", rx_len,
				req->req.actual, req->req.length);
		}
	}
	
	if((rx_len < ep->ep.maxpacket) || (req->req.actual == req->req.length)) {
		if(req->zero && (req->req.actual % ep->ep.maxpacket) == 0) {
			UDC_DBG("SENDING ZLP ep->status_phase %d\n", ep->status_phase);
			ast_dev_udc_ep0_rx(udc);
			req->zero = 0;
			return;
		}
		if(ep->status_phase) {
			ep->status_phase  = 0;
		} else {
			ep->status_phase  = 1;
			ep->ep_dir = 0x80;
			ast_dev_udc_ep0_tx(udc);
		}
		if(!ep->status_phase && !ep->ep_dir) {
			int i =0;
			//printk("%x \n", ast_udc_read(udc, AST_VHUB_EP0_CTRL));
			for(i = 0; i < 0; i++) {
				UDC_DBG("%x ", buf[i]);
			}
			//printk("\n");

			ast_udc_done(ep, req, 0);
		}		
	} else {
		if(rx_len > req->req.length) {
			//printk("ep0 out Check %d %d %d dir %x \n",req->req.length,  rx_len,  ep->ep.maxpacket, ep->ep_dir);
#if 0
			printk("error ~~~~~ ~~~~~~ `~~~~~ %x \n", ast_udc_read(udc, AST_VHUB_EP0_CTRL));
			printk("%x \n", ast_udc_read(udc, AST_VHUB_EP0_CTRL));
			for(i = 0; i < rx_len; i++) {
				printk("%x ", buf[i]);
			}
			printk("\n");
#endif			
			//Issue Fix 
			printk("Issue Fix (%d/%d)\n", req->req.actual, req->req.length);
			ast_dev_udc_ep0_tx(udc);
			ast_udc_done(ep, req, 0);
			return;
		}	
		ep->ep_dir = 0;
		ast_udc_dev_ep0_queue(ep, req);
	}
}

void ast_udc_ep0_in(struct ast_udc *udc)
{
	struct ast_udc_ep *ep = &udc->ep[0];
	struct ast_udc_request	*req;

	SETUP_DBG("\n");

	if (list_empty(&ep->queue)) {
		return;
	} else {
		req = list_entry(ep->queue.next, struct ast_udc_request, queue);
	}

	if(req->req.length == req->req.actual) {
		ast_udc_ep0_rx(udc);
		if(ep->ep_dir) {
			ast_udc_done(ep, req, 0);
		}
	} else {
		ast_udc_ep0_queue(ep, req);
	}
}


void ast_dev_udc_ep0_in(struct ast_udc *udc)
{
	struct ast_udc_ep *ep = &udc->ep[0];
	struct ast_udc_request	*req;

	SETUP_DBG("dev %d dev08 %x dev0c %x\n", udc->dev_num, ast_dev_udc_read(udc, 0x8), ast_dev_udc_read(udc, 0xc));
#ifdef EP_DONE_DEBUG
	printk("ep0in: ");
#endif
	if (list_empty(&ep->queue)) {
		return;
	} else {
		req = list_entry(ep->queue.next, struct ast_udc_request, queue);
	}
	if(ep->status_phase && req->req.length != req->req.actual) {
		printk(KERN_CRIT "\n\nISSUE?::(%d/%d) DEV08 %x DEV0c %x dev %d\n", req->req.actual, req->req.length, ast_dev_udc_read(udc, 0x8), ast_dev_udc_read(udc, 0xc),udc->dev_num);
	}
#ifdef EP_DONE_DEBUG
	printk("(%d/%d) DEV08 %x dev %d ", req->req.actual, req->req.length, ast_dev_udc_read(udc, 0x8),udc->dev_num);
	printk("SP %d DIR %x zero %d \n\n", ep->status_phase, ep->ep_dir, req->zero);
#endif
	if(ep->status_phase || req->req.length == req->req.actual) {
		if(req->zero && (req->req.actual % ep->ep.maxpacket) == 0) {
			UDC_DBG("Sending ZLP ep->status_phase %d\n", ep->status_phase);
			ast_dev_udc_ep0_tx(udc);
			req->zero = 0;
			return;
		}
		if(ep->status_phase) {
			ep->status_phase  = 0;
		} else {
			ep->status_phase  = 1;
			ep->ep_dir = 0;
			ast_dev_udc_ep0_rx(udc);
		}
		if(!ep->status_phase && ep->ep_dir) {
			//printk("Not Memset to 0 for IN only req->req.length %d\n", req->req.length);
			//memset(req->req.buf, 0xaa, req->req.length);
			ast_udc_done(ep, req, 0);
		}
	} else {
		ast_udc_dev_ep0_queue(ep, req);
	}
}

void ast_udc_ep_handle(struct ast_udc *udc, u16 ep_num)
{
	struct ast_udc_ep *ep = &udc->ep[ep_num];
	struct ast_udc_request	*req;
	u16 len = 0;
#ifdef DMA_LIST_MODE
	int do_dma = 0;
#endif

	if(list_empty(&ep->queue))
		return;

//	req = list_entry(ep->queue.next, struct ast_udc_request, queue);
	req = list_first_entry(&ep->queue, struct ast_udc_request, queue);

//	printk("req = %x \n",req);
//	buf = (u8 *) req->req.buf;

	len = (ast_ep_read(ep, AST_EP_DMA_STS) >> 16) & 0x7ff;
	//printk("DMACOMPL:%s handle req->req.length %d dma_len %d ep_num %u\n",
		//ep->ep.name, req->req.length, len, ep_num);
	if((len == 0) || (len < 1024)) {
		//printk("TODO ~~~~~~ len = %d\n", len);
	}
	req->req.actual += len;
#ifdef DMA_LIST_MODE
	if(use_list_mode && req->req.length >= 0 && ((req->req.dma % 4) == 0))
		do_dma = 1;
	if(use_list_mode && ((req->req.dma % 4) != 0) && req->req.length >= 0 && ep->ep_dir)
		do_dma = 1;
	//We use long desc mode and we assume all the data will be xferred in 1 shot
	if(use_list_mode && ep->ep_dir && do_dma) {
		req->req.actual = req->req.length;
	}
	if(req->req.length >= 0)
		LIST_MODE_DBG("DMACOMPL:%s handle req->req.length %d dma_len %d ep_num %u ep->ep_dir %x actual %d do_dma %d\n",
			ep->ep.name, req->req.length, len, ep_num, ep->ep_dir, req->req.actual, do_dma);
	if(1 && !ep->ep_dir && do_dma) { // && req->req.length > 512 && req->req.length < 16384) {
		u32 i = 0, rx_len;
		volatile u32 *temp;
		int num_des = 0;
		int per_desc_len;
		per_desc_len = 512;
		rx_len = req->req.length;
		num_des = req->req.length/(per_desc_len);//7 * 512
		if((req->req.length%per_desc_len))
			num_des++;
		req->req.actual = 0;
		temp = (void*)ep->ep_desc_buf;

		for(i = 0;i < num_des;i++) {
			temp++;
			req->req.actual += *temp & 0x7ff;
			if((*temp & 0x7ff) < 512) {
				len = *temp & 0x7ff;
				break;
			}
			temp++;
		}
		i = ast_ep_read(ep, AST_EP_DMA_CTRL);
		if(i & (1 << 13)) {
			LIST_MODE_DBG("Out short packet rcvd i %x\n", i);
			ast_ep_write(ep, (i | (1 << 13)), AST_EP_DMA_CTRL);
		}
		LIST_MODE_DBG("total rcvd req->req.actual %d len %d\n", req->req.actual, len);
	}

#endif
	
//	EP_DBG(" %s : actual %d \n", ep->ep.name, req->req.actual);
	
	if((req->req.length == req->req.actual)){
#if 1		
		usb_gadget_unmap_request(&udc->gadget, &req->req, ep->ep_dir);
		if((req->req.dma % 4) != 0) {
#ifdef DMA_LIST_MODE
			if(!ep->ep_dir && !do_dma) {//!ep->ep_dir && req->req.length <= 512 && req->req.length >= 16384) {
#else
			if(!ep->ep_dir) {
#endif
				prefetchw(req->req.buf);
//				printk("%s : rx done %x len (%d/%d) \n", ep->ep.name, req->req.dma, req->req.actual, req->req.length);
				memcpy(req->req.buf, ep->ep_buf, req->req.actual);
			} else {
//				printk("%s : tx done %x len (%d/%d) \n", ep->ep.name, req->req.dma, req->req.actual, req->req.length);
			}
		}
#else
		usb_gadget_unmap_request(&udc->gadget, &req->req, ep->ep_dir);

#endif
		ast_udc_done(ep, req, 0);
		if(!list_empty(&ep->queue)) {
//			req = list_entry(ep->queue.next, struct ast_udc_request, queue);
			req = list_first_entry(&ep->queue, struct ast_udc_request, queue);
//			printk("%s next req %x \n", ep->ep.name, req);
			ast_udc_ep_dma(ep, req);
		}		
	} else {
		if((!ep->ep_dir) && (len < ep->ep.maxpacket)) {
#if 1			
			usb_gadget_unmap_request(&udc->gadget, &req->req, ep->ep_dir);
			if((req->req.dma % 4) != 0) {
#ifdef DMA_LIST_MODE
			if(!ep->ep_dir && !do_dma) { //(!ep->ep_dir && req->req.length <= 512 && req->req.length >= 16384) {
#else
				if(!ep->ep_dir) {
#endif
//					printk("%s : rx done %x len (%d/%d) \n", ep->ep.name, req->req.dma, req->req.actual, req->req.length);
					memcpy(req->req.buf, ep->ep_buf, req->req.actual);
				} else {
//					printk("%s : tx done %x len (%d/%d) \n", ep->ep.name, req->req.dma, req->req.actual, req->req.length);
				}
			}
#else		
			usb_gadget_unmap_request(&udc->gadget, &req->req, ep->ep_dir);			

#endif		
			ast_udc_done(ep, req, 0);
			if(!list_empty(&ep->queue)) {
//				req = list_entry(ep->queue.next, struct ast_udc_request, queue);
				req = list_first_entry(&ep->queue, struct ast_udc_request, queue);
//				printk("%s next req %x \n", ep->ep.name, req);
				ast_udc_ep_dma(ep, req);
			}			
		} else {
			//next 			
			ast_udc_ep_dma(ep, req);
		}
	}

	

}

void ast_udc_setup_handle(struct ast_udc *udc)
{
	u16 ep_num = 0;
	int driver_delegate = 1;
	struct usb_ctrlrequest *ctrl = NULL;
#if 0
	struct usb_ep *ep;
	struct usb_gadget	*gadget1;
	struct ast_udc *dev_udc;
	static int print=0;
	dev_udc = udc;
	dev_udc++;
	gadget1 = &dev_udc->gadget;
#endif
	SETUP_DBG("type : %x, req : %x, val : %x, idx: %x, len : %d  \n", 
		udc->root_setup->bRequestType, 
		udc->root_setup->bRequest, 
		udc->root_setup->wValue,
		udc->root_setup->wIndex,
		udc->root_setup->wLength);
#if 0
	print++;
	if(0 && (print %4) == 0)
        list_for_each_entry (ep, &gadget1->ep_list, ep_list) {
                printk("ep %p\n", ep);
        }
#endif
	if(!udc) {
		printk("UDC IS NULL\n\n");
	}
	if(!udc->root_setup)
		printk("udc->root_setup NULL %d\n", udc->dev_num);
	if(!udc->driver->setup)
		printk("udc->driver->setup is NULL %d\n", udc->dev_num);

	if(unlikely(udc->stopping)) {
		printk("Stopping Error %s %d\n", __FUNCTION__, __LINE__);
		return;
	}
#ifdef EP_DONE_DEBUG
	if(udc->dev_num >= 0)
		printk("type : %x, req : %x, val : %x, idx: %x, len : %d dev08 %x \n", 
		udc->root_setup->bRequestType, 
		udc->root_setup->bRequest, 
		udc->root_setup->wValue,
		udc->root_setup->wIndex,
		udc->root_setup->wLength, ast_dev_udc_read(udc, 0x8));
#endif
	if(udc->ep[0].status_phase) {
		printk("Status phase not completed for previous setup packet dev %d\n", udc->dev_num);
		udc->ep[0].status_phase = 0;	
	}
	udc->ep[0].ep_dir = udc->root_setup->bRequestType & USB_DIR_IN;
	udc->ep[0].stopped = 0;
	ctrl = udc->root_setup;
	switch(udc->root_setup->bRequest) {
		case USB_REQ_SET_ADDRESS:
			UDC_DBG("udc->driver->setaddr %d\n", udc->dev_num);
				if (ast_udc_read(udc, AST_VHUB_USB_STS) & (0x1 << 27))
					udc->gadget.speed = USB_SPEED_HIGH;
				else
					udc->gadget.speed = USB_SPEED_FULL;
				hub_speed = udc->gadget.speed;
				
				SETUP_DBG("set addr %x is_hub %d?\n", udc->root_setup->wValue, IS_HUB(udc));
				if(IS_HUB(udc)) {
					ast_udc_write(udc, udc->root_setup->wValue, AST_VHUB_CONF); 
					ast_udc_write(udc, EP0_TX_BUFF_RDY, AST_VHUB_EP0_CTRL);
				} else {
					u32 dev00;
					dev00 = ast_dev_udc_read(udc,0);
					dev00 &= 0xFF;
					dev00 |= (udc->root_setup->wValue << 8);
					ast_dev_udc_write(udc, dev00, 0); 
					ast_dev_udc_write(udc, EP0_TX_BUFF_RDY, 0x8);//DEV08
					UDC_DBG("Dev set adress done\n");
					driver_delegate = 0;
				}
				break;
		case USB_REQ_CLEAR_FEATURE:
			if(udc->root_setup->bRequestType == (USB_TYPE_STANDARD|USB_RECIP_DEVICE) && udc->root_setup->wValue == USB_DEVICE_REMOTE_WAKEUP) {
                                u32 temp;
				UDC_DBG("USB_DEVICE_REMOTE_WAKEUP clear\n");
                                temp = ast_udc_read(udc, AST_VHUB_CTRL);
                                temp &= ~(1 << 3);
                                ast_udc_write(udc, temp, AST_VHUB_CTRL);
					hub_remote_wakeup_set = 0;
				goto finish_status;
			}
			if (udc->root_setup->bRequestType != USB_RECIP_ENDPOINT)
				goto delegate;
			if (udc->root_setup->wValue != USB_ENDPOINT_HALT
					|| udc->root_setup->wLength != 0){
				UDC_DBG("Invalid clear feature req wValue %d ctrl->wIndex 0x%x\n",
						udc->root_setup->wValue, udc->root_setup->wIndex);
				goto do_stall;
			}
			UDC_DBG("ClearFeatureHalt:type : %x, req : %x, val : %x, idx: %x, len : %d dev08 %x \n", 
				udc->root_setup->bRequestType, 
				udc->root_setup->bRequest, 
				udc->root_setup->wValue,
				udc->root_setup->wIndex,
				udc->root_setup->wLength, ast_dev_udc_read(udc, 0x8));
			ep_num = udc->root_setup->wIndex & 0xf;
			UDC_DBG("Clear Stall for eppool %d ep %p %p\n", udc->ep[ep_num].ep_pool_num, &udc->ep[ep_num], &udc->ep[udc->ep[ep_num].ep_pool_num + 1]);
			UDC_DBG("ep_dir %x bEndpointAddress %x name %s AST_EP_CONFIG %x\n",udc->ep[ep_num].ep_dir, udc->ep[ep_num].desc->bEndpointAddress, udc->ep[ep_num].ep.name,
				ast_ep_read(&udc->ep[ep_num], AST_EP_CONFIG));
			ast_udc_write(udc, (ep_num - 1), AST_VHUB_EP_DATA);
			ast_ep_write(&udc->ep[ep_num], ast_ep_read(&udc->ep[ep_num], AST_EP_CONFIG) & ~EP_SET_EP_STALL, AST_EP_CONFIG);
finish_status:
			if(IS_HUB(udc))
				ast_udc_write(udc, EP0_TX_BUFF_RDY, AST_VHUB_EP0_CTRL);				
			else 
				ast_dev_udc_write(udc, EP0_TX_BUFF_RDY, 0x8);//DEV08
			driver_delegate = 0;
			break;
			case USB_REQ_SET_FEATURE:
				UDC_DBG("USB_REQ_SET_FEATURE ep-%d\n", udc->root_setup->wIndex & USB_ENDPOINT_NUMBER_MASK);
				if(udc->root_setup->bRequestType == (USB_TYPE_STANDARD|USB_RECIP_DEVICE) && udc->root_setup->wValue == USB_DEVICE_REMOTE_WAKEUP) {
					UDC_DBG("USB_DEVICE_REMOTE_WAKEUP set\n");
					hub_remote_wakeup_set = 1;
					if(IS_HUB(udc))
						ast_udc_write(udc, EP0_TX_BUFF_RDY, AST_VHUB_EP0_CTRL);
					else
						ast_dev_udc_write(udc, EP0_TX_BUFF_RDY, 0x8);//DEV08
					driver_delegate = 0;
				}
				if (ctrl->bRequestType != USB_RECIP_ENDPOINT)
					goto delegate;
				if (ctrl->wValue != USB_ENDPOINT_HALT
						|| ctrl->wLength != 0){
					UDC_DBG("Invalid set feature req wValue %d ctrl->wIndex 0x%x\n",
							ctrl->wValue, ctrl->wIndex);
					goto do_stall;
				}
			UDC_DBG("SetFeature Halt:type : %x, req : %x, val : %x, idx: %x, len : %d dev08 %x \n", 
				udc->root_setup->bRequestType, 
				udc->root_setup->bRequest, 
				udc->root_setup->wValue,
				udc->root_setup->wIndex,
				udc->root_setup->wLength, ast_dev_udc_read(udc, 0x8));

				break;
		case USB_REQ_GET_STATUS:
			/* USB_ENDPOINT_HALT status? */
			if (ctrl->bRequestType != (USB_DIR_IN|USB_RECIP_ENDPOINT))
				goto delegate;

			UDC_DBG("USB_REQ_GET_STATUS req wValue %d ctrl->wIndex 0x%x\n",
				ctrl->wValue, ctrl->wIndex);
			/* ep0 never stalls */
			if (!(ctrl->wIndex & 0xf)) {
				goto delegate;
			}
			goto delegate;

			break;

	}	
	if(!driver_delegate)
		return;
delegate:
	spin_unlock(&udc->lock);
	if(udc->driver->setup(&udc->gadget, udc->root_setup) < 0) {
do_stall:
		if(IS_HUB(udc))
			ast_udc_write(udc, ast_udc_read(udc, AST_VHUB_EP0_CTRL) | EP0_STALL, AST_VHUB_EP0_CTRL);
		else
			ast_dev_udc_write(udc, ast_dev_udc_read(udc, 8) | EP0_STALL, 8);//DEV08
		printk("TODO XXXXXXXXXXXXXXXXXXXXXXXXXX udc->root_setup->bRequest %d dev %d udc->root_setup %p\n", udc->root_setup->bRequest, udc->dev_num, udc->root_setup);
		printk("type : %x, req : %x, val : %x, idx: %x, len : %d  \n", 
		udc->root_setup->bRequestType, 
		udc->root_setup->bRequest, 
		udc->root_setup->wValue,
		udc->root_setup->wIndex,
		udc->root_setup->wLength);
		UDC_DBG("ast_dev_udc_read(dev_udc, 0) %x dev_reg %p AST_VHUB_CTRL %x\n", ast_dev_udc_read(udc, 0),udc->dev_reg, ast_udc_read(root_udc, AST_VHUB_CTRL));
	}
	spin_lock(&udc->lock);
	
}

static int device_irq(struct ast_udc *udc)
{
	unsigned long flags;
	u32 isr;
	spin_lock_irqsave(&udc->lock, flags);
	isr = ast_dev_udc_read(udc, 0x4);
	if(isr & ISR_HUB_EP0_IN_ACK_STALL) {
		ISR_DBG("ISR_DEV_EP0_IN_ACK_STALL \n");
		ast_dev_udc_write(udc, ISR_HUB_EP0_IN_ACK_STALL, 0x4);
		ast_dev_udc_ep0_in(udc);
	}

	if(isr & ISR_HUB_EP0_OUT_ACK_STALL) {
		int i = 0;
		ISR_DBG("ISR_DEV_EP0_OUT_ACK_STALL \n");
		ast_dev_udc_write(udc, ISR_HUB_EP0_OUT_ACK_STALL, 0x4);
		for(i = 0;i < 100;i++) {
			if(!(ast_dev_udc_read(udc, 0x8) & EP0_RX_BUFF_RDY)) {
				ast_dev_udc_ep0_out(udc);
				break;
			}
			UDC_DBG("ast_dev_udc_read(udc, 0x8) %x\n", ast_dev_udc_read(udc, 0x8));
			mdelay(100);
		}
	}
	if(isr & ISR_HUB_EP0_SETUP) {
		ISR_DBG("ISR_DEV_EP0_SETUP \n");
		ast_dev_udc_write(udc, ISR_HUB_EP0_SETUP, 0x4);
		ast_udc_setup_handle(udc);
	}
	spin_unlock_irqrestore(&udc->lock, flags);
	return IRQ_HANDLED;
}

static irqreturn_t ast_udc_irq (int irq, void *data)
{
	struct ast_udc	*udc = (struct ast_udc*) data;
	u32 isr = ast_udc_read(udc, AST_VHUB_ISR);
	u32 ep_isr = 0;
	int i = 0;
	
	spin_lock(&udc->lock);

	if(isr & ISR_BUS_RESET) {
		BUS_DBG("ISR_BUS_RESET \n");
		hub_remote_wakeup_set = 0;
		ast_udc_write(udc, ISR_BUS_RESET, AST_VHUB_ISR);
		ast_udc_stop_activity(udc);
		udc->gadget.speed = USB_SPEED_UNKNOWN;
		for(i = 0; i < AST_NUM_ENDPOINTS;i++)
			ep_to_udc[i] = NULL;
		ep_to_udc[0] = udc;//always used
		if (udc->driver && udc->driver->reset) {
			spin_unlock(&udc->lock);
			udc->driver->reset(&udc->gadget);
			spin_lock(&udc->lock);
		}		
	}

	if(isr & ISR_SUSPEND_RESUME) {
		//Suspend, we don't handle this in sample
		BUS_DBG("ISR_SUSPEND_RESUME \n");
		ast_udc_write(udc, ISR_SUSPEND_RESUME, AST_VHUB_ISR);
		if (udc->driver && udc->driver->resume) {
			spin_unlock(&udc->lock);
			udc->driver->resume(&udc->gadget);
			spin_lock(&udc->lock);
		}
	}

	if(isr & ISR_HUB_EP0_IN_ACK_STALL) {
		ISR_DBG("ISR_HUB_EP0_IN_ACK_STALL \n");
		ast_udc_write(udc, ISR_HUB_EP0_IN_ACK_STALL, AST_VHUB_ISR);
		ast_udc_ep0_in(udc);
	}

	if(isr & ISR_HUB_EP0_OUT_ACK_STALL) {
		ISR_DBG("ISR_HUB_EP0_OUT_ACK_STALL \n");
		ast_udc_write(udc, ISR_HUB_EP0_OUT_ACK_STALL, AST_VHUB_ISR);
		ast_udc_ep0_out(udc);
	}

	if(isr & ISR_HUB_EP0_OUT_NAK) {
//		ISR_DBG("ISR_HUB_EP0_OUT_NAK \n");
		ast_udc_write(udc, ISR_HUB_EP0_OUT_NAK, AST_VHUB_ISR);
	}

	if(isr & ISR_HUB_EP0_IN_DATA_NAK) {
		//IN NAK, we don't handle this in sample
//		ISR_DBG("ISR_HUB_EP0_IN_DATA_ACK \n");
		ast_udc_write(udc, ISR_HUB_EP0_IN_DATA_NAK, AST_VHUB_ISR);
	}

	if(isr & ISR_HUB_EP0_SETUP) {
		ISR_DBG("SETUP udc %p root_udc %p\n", udc, root_udc);
		ast_udc_write(udc, ISR_HUB_EP0_SETUP, AST_VHUB_ISR);
		ast_udc_setup_handle(udc);
	}

	if(isr & ISR_HUB_EP1_IN_DATA_ACK) {
		//HUB Bitmap control
		ISR_DBG("ERROR EP1 IN\n");
		ast_udc_write(udc, ISR_HUB_EP1_IN_DATA_ACK, AST_VHUB_ISR);
		ast_udc_write(udc, 0x00, AST_VHUB_EP1_STS_CHG);
	}
	spin_unlock(&udc->lock);

	if(isr & ISR_DEVICE1) {
		ISR_DBG("ISR_DEVICE1\n");
		device_irq(udc + 1);
	}

	if(isr & ISR_DEVICE2) {
		ISR_DBG("ISR_DEVICE2\n");
		device_irq(udc + 2);
	}

	if(isr & ISR_DEVICE3) {
		ISR_DBG("ISR_DEVICE3 \n");
		device_irq(udc + 3);
	}

	if(isr & ISR_DEVICE4) {
		ISR_DBG("ISR_DEVICE4 \n");
		device_irq(udc + 4);
	}

	if(isr & ISR_DEVICE5) {
		ISR_DBG("ISR_DEVICE5 \n");
		device_irq(udc + 5);
	}
	spin_lock(&udc->lock);

	if(isr & ISR_EP_ACK_STALL) {
		EP_DBG("ISR_EP_ACK_STALL\n");
//		ast_udc_write(udc, ISR_EP_ACK_STALL, AST_VHUB_ISR); 
		ep_isr = ast_udc_read(udc, AST_VHUB_EP_ACK_ISR);
		for(i = 0; i < 15; i++) {
			if(ep_isr & (0x1 << i)) {
				EP_DBG("ep_to_udc[%d] %p hub_udc %p\n", i, ep_to_udc[i + 1], udc);
				ast_udc_write(ep_to_udc[i + 1], 0x1 <<  i, AST_VHUB_EP_ACK_ISR); 
				ast_udc_ep_handle(ep_to_udc[i + 1], i + 1);			
			}
		}
	}

	if(isr & ISR_EP_NAK) {
		ISR_DBG("ISR_EP_NAK ****************************************\n");
		ast_udc_write(udc, ISR_EP_NAK, AST_VHUB_ISR);
	}

	if(isr & ISR_BUS_SUSPEND) {
#ifdef AUTO_REMOTE_WAKEUP
		u32 temp;
#endif
		//Suspend, we don't handle this in sample
		BUS_DBG("ISR_BUS_SUSPEND \n");
		BUS_DBG("Suspend hub_remote_wakeup_set %d\n", hub_remote_wakeup_set);
		ast_udc_write(udc, ISR_BUS_SUSPEND, AST_VHUB_ISR);
#ifdef AUTO_REMOTE_WAKEUP
		if(hub_remote_wakeup_set) {
			temp = ast_udc_read(udc, AST_VHUB_CTRL);
			temp |= (1 << 3);
			ast_udc_write(udc, temp, AST_VHUB_CTRL);
		}
#endif
		if (udc->driver && udc->driver->suspend) {
			spin_unlock(&udc->lock);
			udc->driver->suspend(&udc->gadget);
			spin_lock(&udc->lock);
		}
	}
	spin_unlock(&udc->lock);
	return IRQ_HANDLED;

}
/*-------------------------------------------------------------------------*/
static int ast_udc_gadget_getframe(struct usb_gadget *gadget)
{
	struct ast_udc	*udc = container_of(gadget, struct ast_udc, gadget);

	UDC_DBG("\n");
	return (ast_udc_read(udc, AST_VHUB_USB_STS) >> 16) & 0x7ff;
}

static int ast_dev_udc_gadget_getframe(struct usb_gadget *gadget)
{
//	struct ast_udc	*udc = container_of(gadget, struct ast_udc, gadget);

	UDC_DBG("\n");
	return 0;
}

static int ast_udc_wakeup(struct usb_gadget *gadget)
{
	UDC_DBG("TODO ~~~~~\n");
	return 0;
}
static void device_deinit_hw(struct ast_udc	*udc);
extern void hub_device_connect_disconnect(int devnum, struct usb_gadget *gadget, int conn);
static int ast_dev_udc_pullup(struct usb_gadget *gadget, int is_on)
{
	struct ast_udc	*udc = container_of(gadget, struct ast_udc, gadget);

	UDC_DBG("dev %d on %d\n", udc->dev_num, is_on);
	UDC_DBG("pullup dev %d on %d\n", udc->dev_num, is_on);
	if(is_on) {
		hub_device_connect_disconnect(udc->dev_num, &root_udc->gadget, 1);
	} else {
		device_deinit_hw(udc);//Interrupts can happen and state of the driver may be undefined
		hub_device_connect_disconnect(udc->dev_num, &root_udc->gadget, 0);
	}
	return 0;
}
/*
 * activate/deactivate link with host; minimize power usage for
 * inactive links by cutting clocks and transceiver power.
 */

static int ast_udc_pullup(struct usb_gadget *gadget, int is_on)
{
	struct ast_udc	*udc = container_of(gadget, struct ast_udc, gadget);

	UDC_DBG("%d \n", is_on);

	if(is_on)
		ast_udc_write(udc, ast_udc_read(udc, AST_VHUB_CTRL) | ROOT_UPSTREAM_EN, AST_VHUB_CTRL);	
	else
		ast_udc_write(udc, ast_udc_read(udc, AST_VHUB_CTRL) & ~ROOT_UPSTREAM_EN, AST_VHUB_CTRL);	

	return 0;
}

int is_ep_free(struct usb_gadget *gadget, struct usb_ep *_ep)
{
	u8 num = 0;
	struct ast_udc_ep	*ep = container_of(_ep, struct ast_udc_ep, ep);
	if (isdigit (_ep->name [2]))
		num = simple_strtoul (&_ep->name [2], NULL, 10);
	UDC_DBG("is_ep_free %s num %d ep_pool_num %d free %p ep %p\n", _ep->name, num, ep->ep_pool_num, ep_to_udc[num], ep);
	if(!ep_to_udc[num]) {
		return 1;
	}
	return 0;
}
EXPORT_SYMBOL(is_ep_free);
void mark_ep(struct usb_gadget *gadget, struct usb_ep *_ep, int used)
{
	struct ast_udc	*udc = container_of(gadget, struct ast_udc, gadget);
	struct ast_udc_ep	*ep = container_of(_ep, struct ast_udc_ep, ep);
	int i;
	u8 num = 0;
	if (isdigit (_ep->name [2]))
		num = simple_strtoul (&_ep->name [2], NULL, 10);

	UDC_DBG("Marking ep pool %d as used num %d oldudc %p udc %p ep %p\n",
		ep->ep_pool_num, num, ep_to_udc[ep->ep_pool_num], udc, ep);
	ep_to_udc[num] = udc;
	
	for(i = 0;i < 6;i++) {
		if(!udc->used_ep[i]) {
			udc->used_ep[i] = _ep;
			break;
		}
	}
}
EXPORT_SYMBOL(mark_ep);

#define NUM_PORTS 5
static void device_init_hw(struct ast_udc	*udc)
{
	u32 temp;
	ast_dev_udc_write(udc, 0xff, 4);
	ast_dev_udc_write(udc, (1<<2) | (1 << 3) | (1 << 5) | 0, 0);//DEV00
	UDC_DBG("DEV00 %x\n", ast_dev_udc_read(udc, 0));
	temp = ast_udc_read(udc, AST_VHUB_IER);
	temp = temp | (1 << (9 + udc->dev_num));
	ast_udc_write(udc, temp, AST_VHUB_IER);
}

static void device_deinit_hw(struct ast_udc	*udc)
{
	u32 temp;
	ast_dev_udc_write(udc, 0, 0);//DEV00 Disable device
	ast_dev_udc_write(udc, 0xff, 4);
	UDC_DBG("DEV00 %x\n", ast_dev_udc_read(udc, 0));
	temp = ast_udc_read(udc, AST_VHUB_IER);
	temp = temp & ~(1 << (9 + udc->dev_num));
	ast_udc_write(udc, temp, AST_VHUB_IER);
}

static int ast_dev_udc_start(struct usb_gadget *gadget,
		struct usb_gadget_driver *driver)
{
	struct ast_udc	*udc = container_of(gadget, struct ast_udc, gadget);
	unsigned long	flags;

	/* basic sanity tests */
	UDC_DBG("\n");

	if (!udc) {
		printk("%s %d\n", __FUNCTION__, __LINE__);
		return -ENODEV;
	}
	if (!driver
		|| driver->max_speed < USB_SPEED_FULL
		|| !driver->setup) {
		printk("%s %d\n", __FUNCTION__, __LINE__);
		return -EINVAL;
	}
	spin_lock_irqsave(&udc->lock, flags);
	UDC_DBG("dev udcstart %d dev_reg %p udc %p\n", udc->dev_num, udc->dev_reg, udc);
	/* hook up the driver */
	driver->driver.bus = NULL;
	udc->driver = driver;
	udc->gadget.dev.driver = &driver->driver;
	device_init_hw(udc);
	spin_unlock_irqrestore(&udc->lock, flags);
	return 0;
}
static int ast_udc_start(struct usb_gadget *gadget,
		struct usb_gadget_driver *driver)
{
	struct ast_udc	*udc = container_of(gadget, struct ast_udc, gadget);

	UDC_DBG("\n");

	if (!udc)
		return -ENODEV;

	if (udc->driver)
		return -EBUSY;

	udc->driver = driver;

	udc->gadget.dev.of_node = udc->pdev->dev.of_node;
//	udc->gadget.is_selfpowered = 1;

	return 0;
}

static int ast_dev_udc_stop(struct usb_gadget *gadget)
{
	struct ast_udc *udc = container_of(gadget, struct ast_udc, gadget);
	unsigned long	flags;
	int i = 0;
	UDC_DBG("ast_dev_udc_stop udc->dev_num %d\n", udc->dev_num);
	udc->stopping = 1;
	spin_lock_irqsave(&udc->lock, flags);
	udc->stopping = 1;
	UDC_DBG("ast_dev_udc_stop1 udc->dev_num %d\n", udc->dev_num);
	device_deinit_hw(udc);
	ast_udc_write(udc, (1 << (udc->dev_num + 1)), AST_VHUB_DEV_RESET);
	ast_udc_write(udc, 0, AST_VHUB_DEV_RESET);
	UDC_DBG("%s %d\n", __FUNCTION__, __LINE__);
	ast_dev_udc_stop_activity(udc, 0);
	UDC_DBG("%s %d\n", __FUNCTION__, __LINE__);
	udc->driver = NULL;
	for(i = 0 ;i < AST_NUM_ENDPOINTS;i++) {
		if(ep_to_udc[i] == udc) {
			printk("Disabling Endpoint pool ep %d ep_reg %p\n",
				udc->ep[i].ep_pool_num, udc->ep[i].ep_reg);
			ast_ep_write(&udc->ep[i], 0, AST_EP_CONFIG);
			ep_to_udc[i] = NULL;
		}
	}
	UDC_DBG("ast_dev_udc_stopdone  udc->dev_num %d\n", udc->dev_num);
	udc->stopping = 0;
	spin_unlock_irqrestore(&udc->lock, flags);
	return 0;
}
static int ast_udc_stop(struct usb_gadget *gadget)
{
	struct ast_udc *udc = container_of(gadget, struct ast_udc, gadget);
	unsigned long	flags;

	UDC_DBG("\n");

	spin_lock_irqsave(&udc->lock, flags);
	ast_udc_write(udc, ast_udc_read(udc, AST_VHUB_CTRL) & ~ROOT_UPSTREAM_EN, AST_VHUB_CTRL);	
	udc->gadget.speed = USB_SPEED_UNKNOWN;
	ast_udc_stop_activity(udc);
	spin_unlock_irqrestore(&udc->lock, flags);

	udc->driver = NULL;

	return 0;
}

static struct usb_ep *ast_dev_match_ep(struct usb_gadget *g,
		struct usb_endpoint_descriptor *desc,
		struct usb_ss_ep_comp_descriptor *ep_comp)
{
	struct usb_ep *ep;
	list_for_each_entry (ep, &g->ep_list, ep_list) {
		if(is_ep_free(g, ep))
			break;
	}
	if (usb_gadget_ep_match_desc(g, ep, desc, ep_comp)) {
		mark_ep(g, ep, 1);
		return ep;
	}
	printk("EP Not found\n");
	return NULL;
}

static const struct usb_gadget_ops ast_dev_udc_ops = {
	.get_frame		= ast_dev_udc_gadget_getframe,	
	.wakeup			= ast_udc_wakeup,
	.pullup			= ast_dev_udc_pullup,
	.udc_start		= ast_dev_udc_start,
	.udc_stop		= ast_dev_udc_stop,
	.match_ep		= ast_dev_match_ep,
};


static const struct usb_gadget_ops ast_udc_ops = {
	.get_frame		= ast_udc_gadget_getframe,	
	.wakeup			= ast_udc_wakeup,
	.pullup			= ast_udc_pullup,
	.udc_start		= ast_udc_start,
	.udc_stop		= ast_udc_stop,
};
/*-------------------------------------------------------------------------*/
static void ast_udc_init(struct ast_udc *udc) 
{
#ifdef DMA_LIST_MODE
	if(use_list_mode) {
		ast_udc_write(udc, ROOT_PHY_CLK_EN | ROOT_PHY_RESET_DIS | ROOT_EP_LONG_DESC, AST_VHUB_CTRL);
	} else {
		ast_udc_write(udc, ROOT_PHY_CLK_EN | ROOT_PHY_RESET_DIS, AST_VHUB_CTRL);
	}
#else
	ast_udc_write(udc, ROOT_PHY_CLK_EN | ROOT_PHY_RESET_DIS, AST_VHUB_CTRL);
#endif
	
	udelay(1);
	ast_udc_write(udc, 0, AST_VHUB_DEV_RESET);

	ast_udc_write(udc, 0x101ff, AST_VHUB_IER);
	
	ast_udc_write(udc, 0x1ffff, AST_VHUB_ISR);

	ast_udc_write(udc, 0x7ffff, AST_VHUB_EP_ACK_ISR);		
	
	ast_udc_write(udc, 0x7ffff, AST_VHUB_EP_ACK_IER);	
	ast_udc_write(udc, 0, AST_VHUB_EP0_CTRL);
	ast_udc_write(udc, 0x1, AST_VHUB_EP1_CTRL);
//	ast_udc_write(udc, 0, AST_VHUB_EP1_CTRL);

}

int handle_host_resume(int devnum, struct usb_gadget *gadget)
{
	struct ast_udc *dev_udc;
        u32 val;

#ifdef ASPEED_USB_DEV_SPEED
#define HIGH_SPEED 0x80
#define FULL_SPEED 0x0
#endif
#ifdef ASPEED_USB_DEV_SPEED
	val = (hub_speed == USB_SPEED_HIGH)?HIGH_SPEED:0;
	val |= HOST_RESUME;
        //speed detection bit is Bit7 this is readonly for devices
	UDC_DBG("handle_host_resume dev %d val %d\n", devnum, val);
#else
	val = HOST_RESUME;
        //speed detection bit is Bit7 this is readonly for devices
	UDC_DBG("handle_host_resume dev %d val %d\n", devnum, val);
	dev_udc = root_udc;
	UDC_DBG("%s devnum %d\n", __FUNCTION__, devnum);
	dev_udc = dev_udc  + 1 + devnum;
	dev_udc->soft_irq_status |= HOST_RESUME;
	tasklet_schedule(&dev_udc->udc_tasklet);
#endif
	return 0;
}

int handle_port_reset(int devnum, struct usb_gadget *gadget)
{
	struct ast_udc *dev_udc;
	u32 temp;
	dev_udc = root_udc;
	UDC_DBG("%s devnum %d\n", __FUNCTION__, devnum);
	dev_udc = dev_udc  + 1 + devnum;
	dev_udc->soft_irq_status |= DEV_PORT_RESET;
	temp = ast_dev_udc_read(dev_udc, 0);
	temp = temp & ~(0x7F << 8);
	ast_dev_udc_write(dev_udc, temp, 0);//DEV00
	dev_udc->ep[0].status_phase = 0;//start fresh
	tasklet_schedule(&dev_udc->udc_tasklet);
	return 0;
}

int handle_port_suspend(int devnum, struct usb_gadget *gadget)
{
	struct ast_udc *dev_udc;
	dev_udc = root_udc;
	UDC_DBG("%s devnum %d\n", __FUNCTION__, devnum);
	dev_udc = dev_udc  + 1 + devnum;
	dev_udc->soft_irq_status |= DEV_PORT_SUSPEND;
	tasklet_schedule(&dev_udc->udc_tasklet);
	UDC_DBG("handle_port_suspend dev %d\n", devnum);
	return 0;
}


int get_far(int devnum)
{
	struct ast_udc *dev_udc;
	u32 temp;
	dev_udc = root_udc;
	dev_udc = dev_udc + devnum;
	if(devnum > NUM_PORTS) {
		panic("%s devnum %d > NUM_PORTS\n", __FUNCTION__, devnum);
	}
	temp = ast_dev_udc_read(dev_udc, 0);
	temp = temp >> 8;
	UDC_DBG("%s!devnum %d far %x\n", __FUNCTION__, devnum, (temp & 0x7f));
	//mdelay(10000);
	return (temp & 0x7f);
}

void port_enable(int devnum, struct usb_gadget *gadget)
{
	struct ast_udc *dev_udc;
	int i;
	u32 temp;
	dev_udc = root_udc;
	dev_udc++;
	if(devnum > NUM_PORTS) {
		panic("%s devnum %d > NUM_PORTS\n", __FUNCTION__, devnum);
	}
	for(i = 0; i < NUM_PORTS;i++) {
		UDC_DBG("port_enable dev_udc %p devnum %d dev_udc->dev_num %d\n", dev_udc, devnum, dev_udc->dev_num);
		if(dev_udc->dev_num == devnum)
			break;
		dev_udc++;
	}
	device_init_hw(dev_udc);
	temp = ast_dev_udc_read(dev_udc, 0);
	temp = temp | 1;//High speed and device enable
	ast_dev_udc_write(dev_udc, temp, 0);//DEV00
	temp = temp | ( 1 << 1) | 1;//High speed and device enable
	ast_dev_udc_write(dev_udc, temp, 0);//DEV00
	UDC_DBG("ast_dev_udc_read(dev_udc, 0) %x dev_reg %p AST_VHUB_CTRL %x\n", ast_dev_udc_read(dev_udc, 0),dev_udc->dev_reg, ast_udc_read(root_udc, AST_VHUB_CTRL));
}

void port_disable(int devnum, struct usb_gadget *gadget)
{
	struct ast_udc *dev_udc;
	int i;
	u32 temp = 0;
	dev_udc = root_udc;
	dev_udc++;
	for(i = 0; i < NUM_PORTS;i++) {
		//printk("port_disable dev_udc %p devnum %d dev_udc->dev_num %d\n", dev_udc, devnum, dev_udc->dev_num);
		if(dev_udc->dev_num == devnum)
			break;
		dev_udc++;
	}
	if(dev_udc->dev_num == devnum)
		ast_dev_udc_write(dev_udc, temp, 0);//DEV00
}

#define EP_DMA_SIZE		2048
void __iomem *usb_reg_space = NULL;
static const char       gadget_name[6][32] = {"ast_dev_udc_0", "ast_dev_udc_1", "ast_dev_udc_2", "ast_dev_udc_3", "ast_dev_udc_4",
										"ast_dev_udc_5"};
static const char       virt_dev_name[6][32] = {"ast_vdev_udc_0", "ast_vdev_udc_1", "ast_vdev_udc_2", "ast_vdev_udc_3", "ast_vdev_udc_4",
										"ast_vdev_udc_5"};

static volatile u8 reset_done[NUM_PORTS];
static void dev_soft_irq(unsigned long data)
{
	unsigned long flags;
	struct ast_udc *udc = (struct ast_udc *)data;
	int i = 0;
	extern void hub_reset_done(int devnum, struct usb_gadget *gadget);

	spin_lock_irqsave(&udc->lock, flags);
	if(udc->soft_irq_status & DEV_PORT_RESET) {
		if(root_udc == udc)
			WARN(1, "DEV_PORT_RESET For Hub is not possible\n");
		udc->soft_irq_status &= ~(DEV_PORT_RESET); 
		ast_dev_udc_stop_activity(udc, 1);
		root_udc->soft_irq_status |= DEV_PORT_RESET_DONE;
		reset_done[udc->dev_num] = 0x2F;
		tasklet_schedule(&root_udc->udc_tasklet);
		spin_unlock_irqrestore(&udc->lock, flags);
	}
	
	if(udc->soft_irq_status & DEV_PORT_RESET_DONE) {
		udc->soft_irq_status &= ~(DEV_PORT_RESET_DONE); 
		spin_unlock_irqrestore(&udc->lock, flags);
		for(i = 0;i < NUM_PORTS;i++) {
			if(reset_done[i] == 0x2F) {
				reset_done[i] = 0;
				hub_reset_done(i, &root_udc->gadget);
			}
		}
	}

	if(udc->soft_irq_status & HOST_RESUME) {
		udc->soft_irq_status &= ~(HOST_RESUME);
		spin_unlock_irqrestore(&udc->lock, flags);
		UDC_DBG("dev %d resume\n", udc->dev_num);
		if (udc->driver && udc->driver->resume) {
			udc->driver->resume(&udc->gadget);
		}

	}

	if(udc->soft_irq_status & DEV_PORT_SUSPEND) {
		udc->soft_irq_status &= ~(DEV_PORT_SUSPEND);
		spin_unlock_irqrestore(&udc->lock, flags);
		UDC_DBG("dev %d suspends\n", udc->dev_num);
		if (udc->driver && udc->driver->suspend) {
			udc->driver->suspend(&udc->gadget);
		}
	}
}
static struct device virt_dev[NUM_PORTS];
static int ast_udc_probe(struct platform_device *pdev)
{
	struct ast_udc	*udc;
	struct ast_udc_ep	*ep;		
	int				ret = 0;
	struct resource	*res;
	extern int ast_hub_init(int);
	int i, j;
	struct ast_udc	*dev_udc;
	char str[32];
	void					*ep_buf;
	dma_addr_t			ep_dma;
#ifdef DMA_LIST_MODE
	dma_addr_t desc_buf;
	dma_addr_t desc_dma;
#endif
#if defined (DMA_LIST_MODE) && defined (CONFIG_OF)
        struct device_node *np = pdev->dev.of_node;
	const __be32 *ip;
	int len;
#endif

	UDC_DBG(" \n");
	UDC_DBG("\nast_udc_probe!!\n");
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		pr_err("platform_get_resource error.\n");
		return -ENODEV;
	}
	root_udc = udc = devm_kzalloc(&pdev->dev, (sizeof(struct ast_udc) * (NUM_PORTS + 1)) + 32, GFP_KERNEL);
	if (udc == NULL)
		goto err_alloc;
	udc->dev_num = -1;//Hub
	udc->irq = platform_get_irq(pdev, 0);
	if (!udc->irq) {
		pr_err("platform_get_resource IORESOURCE_IRQ error.\n");
		return -ENODEV;
	}
	udc->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(udc->clk)) {
		dev_err(&pdev->dev, "no clock defined \n");
		return -ENODEV;
	}

	clk_prepare_enable(udc->clk);
	dev_udc = udc;
	dev_udc++;
	/* init software state */
	udc->pdev = pdev;
	spin_lock_init(&udc->lock);

	udc->gadget.speed = USB_SPEED_UNKNOWN;
	udc->gadget.max_speed = USB_SPEED_HIGH;
	udc->gadget.dev.parent = &pdev->dev;
	udc->gadget.ops = &ast_udc_ops;
	udc->gadget.ep0 = &udc->ep[0].ep;
	udc->gadget.name = "ast_udc";
	udc->gadget.dev.init_name = "gadget";

	udc->reg = usb_reg_space = ioremap(res->start, resource_size(res));
	udc->dev_reg = 0;
	if (udc->reg == NULL) {
		pr_err("ioremap error.\n");
		goto err_map;
	}
	for(i = 0; i < AST_NUM_ENDPOINTS;i++)
		ep_to_udc[i] = NULL;
	ep_to_udc[0] = udc;//always used
	udc->root_setup = udc->reg + 0x80;

	INIT_LIST_HEAD(&udc->gadget.ep_list);
	INIT_LIST_HEAD(&udc->gadget.ep0->ep_list);
#if defined (DMA_LIST_MODE) && defined (CONFIG_OF)
        ip = of_get_property(np,"list_mode", &len);
        if(ip) {
                use_list_mode = be32_to_cpu(*ip);
        }
        UDC_DBG("USB HUB use_list_mode %d len %d\n", use_list_mode, len);
#endif

	//1024 * 16
	udc->ep0_ctrl_buf = dma_alloc_coherent(udc->gadget.dev.parent,
					EP_DMA_SIZE * 2, &udc->ep0_ctrl_dma, GFP_KERNEL);
#ifdef DMA_LIST_MODE
	if(use_list_mode) {
		dma_desc_buf = dma_alloc_coherent(NULL,
					SZ_4K, &dma_desc_dma, GFP_KERNEL);
	}
#endif
	UDC_DBG("Hub udc->gadget %p\n", &udc->gadget);	
	tasklet_init(&udc->udc_tasklet, dev_soft_irq, (unsigned long)udc);
	for(i = 0;i < NUM_PORTS;i++) {
		dev_udc->dev_num = i;
		UDC_DBG("dev_udc->gadget %p\n", &dev_udc->gadget);
		dev_udc->irq = udc->irq;
		dev_udc->reg = udc->reg;
		dev_udc->dev_reg = udc->reg + 0x100 + ( i *16);
		dev_udc->gadget.speed		= udc->gadget.speed;
		dev_udc->gadget.max_speed	= udc->gadget.max_speed;
		dev_udc->gadget.dev.parent	= &udc->gadget.dev;
		dev_udc->gadget.ops			= &ast_dev_udc_ops;
		dev_udc->gadget.ep0			= &dev_udc->ep[0].ep;
		dev_udc->gadget.name			= gadget_name[i];	
		sprintf(str, "ast_dev_udc%d\n", i);
		dev_set_name(&dev_udc->gadget.dev, str);
		dev_udc->root_setup = udc->reg + 0x88 + (i*8);
		dev_udc->gadget.dev.init_name = gadget_name[i];
		virt_dev[i].init_name = virt_dev_name[i];
		//device_initialize(&virt_dev[i]);
		ret = device_register(&virt_dev[i]);
		dev_set_name(&virt_dev[i], virt_dev_name[i]);
		INIT_LIST_HEAD(&dev_udc->gadget.ep_list);
		INIT_LIST_HEAD(&dev_udc->gadget.ep0->ep_list);
		tasklet_init(&dev_udc->udc_tasklet, dev_soft_irq, (unsigned long)dev_udc);
		dev_udc++;
	}
	for (i = 0; i < 2; i++) {
		ep = &udc->ep[i];
		ep->ep.name = ast_ep_name[i];
//		ep->ep.caps = ep_info[i].caps;
		ep->ep.ops = &ast_udc_ep_ops;
		ep->udc = udc;
		if(i) {
			ep->ep_reg = udc->reg + 0x200 + (0x10 * (i - 1));
			ep->ep_pool_num = (i - 1);

			ep->ep_buf = udc->ep0_ctrl_buf + (i * EP_DMA_SIZE);
			ep->ep_dma = udc->ep0_ctrl_dma + (i * EP_DMA_SIZE);
			usb_ep_set_maxpacket_limit(&ep->ep, 1024);
		} else {
			ep->ep_reg = 0;
			usb_ep_set_maxpacket_limit(&ep->ep, 64);		
		}
		UDC_DBG("%s : maxpacket  %d, dma: %x \n ", ep->ep.name, ep->ep.maxpacket, ep->ep_dma);
		
		if(i) 
			list_add_tail(&ep->ep.ep_list, &udc->gadget.ep_list);
		
		INIT_LIST_HEAD(&ep->queue);
	}
	dev_udc = udc;
	dev_udc++;
	ep_buf = dma_alloc_coherent(udc->gadget.dev.parent,
					EP_DMA_SIZE * AST_NUM_ENDPOINTS, &ep_dma, GFP_KERNEL);

	for(j = 0;j < NUM_PORTS;j++) {
#ifdef DMA_LIST_MODE
		if(use_list_mode) {
			desc_buf = dma_desc_buf;
			desc_dma = dma_desc_dma;
		}
#endif
		dev_udc->ep0_ctrl_buf = dma_alloc_coherent(NULL,
					64, &dev_udc->ep0_ctrl_dma, GFP_KERNEL);
		for (i = 0; i < AST_NUM_ENDPOINTS; i++) {
			ep = &dev_udc->ep[i];
			ep->ep.name = ast_ep_name[i];
	//		ep->ep.caps = ep_info[i].caps;
			ep->ep.ops = &ast_udc_dev_ep_ops;
			ep->udc = dev_udc;
			if(i) {
				ep->ep_reg = dev_udc->reg + 0x200 + (0x10 * (i - 1));
				ep->ep_pool_num = (i - 1);
				ep->ep_buf = ep_buf + ((i -1) * EP_DMA_SIZE);
				ep->ep_dma = ep_dma + ((i -1) * EP_DMA_SIZE);
				usb_ep_set_maxpacket_limit(&ep->ep, 1024);
#ifdef DMA_LIST_MODE
				if(use_list_mode) {
					ep->ep_desc_buf = desc_buf;
					ep->ep_desc_buf = ALIGN(ep->ep_desc_buf, 8);
					ep->ep_desc_dma = desc_dma;
					ep->ep_desc_dma = ALIGN(desc_dma, 8);
					desc_dma += 200;
					desc_buf += 200;//space shd be enuf
				}
#endif
				ep->ep.caps.type_bulk = true;
				ep->ep.caps.dir_in = true;
				ep->ep.caps.dir_out = true;
			} else {
				ep->ep_reg = 0;
				ep->ep.caps.type_control = true;
				usb_ep_set_maxpacket_limit(&ep->ep, 64);
			}
			UDC_DBG("%s : maxpacket  %d, dma: %x ep %p\n ", ep->ep.name, ep->ep.maxpacket, ep->ep_dma, &ep->ep);
			
			if(i) 
				list_add_tail(&ep->ep.ep_list, &dev_udc->gadget.ep_list);
			
			INIT_LIST_HEAD(&ep->queue);
		}
		dev_udc++;
	}
	//ast udc initial 
	ast_udc_init(udc);

	/* request UDC and maybe VBUS irqs */
	ret = request_irq(udc->irq, ast_udc_irq, IRQF_SHARED, "ast_udc", udc);
	if (ret < 0) {
		printk("request irq %d failed\n", udc->irq);
		goto fail1;
	}

	ret = usb_add_gadget_udc(&pdev->dev, &udc->gadget);
	if (ret)
		goto fail1;
	dev_set_drvdata(&pdev->dev, udc);
	device_init_wakeup(&pdev->dev, 1);
	ast_hub_init(1);
	printk(KERN_INFO "ast_udc: HUB driver successfully loaded?.\n");
	dev_udc = udc;
	dev_udc++;
	for(j = 0;j < NUM_PORTS;j++) {
		UDC_DBG("add dev %d udc dev %d dev_udc %p kobj name %s\n", j, dev_udc->dev_num,dev_udc, kobject_name(&virt_dev[j].kobj));
		ret = usb_add_gadget_udc(&virt_dev[j], &dev_udc->gadget);
		if (ret) {
			printk("Add udc for dev %dfailed ret %d\n", j + 1, ret);
			goto fail1;
		}
		dev_udc++;
	}
	printk(KERN_INFO "ast_udc: HUB driver successfully loaded?.\n");
	return 0;

fail1:
	free_irq(udc->irq, udc);

err_map:
	if (udc->reg)
		iounmap(udc->reg);

err_alloc:
	kfree(udc);

	printk("ast udc probe failed, %d\n", ret);
	
	return ret;
}

static int __exit ast_udc_remove(struct platform_device *pdev)
{
	struct ast_udc *udc = platform_get_drvdata(pdev);
	struct resource *res;
	unsigned long	flags;
	//TODO Proper cleanup since Hub is builtin , removal can only happen during reboot
	usb_del_gadget_udc(&udc->gadget);
	if (udc->driver)
		return -EBUSY;

	spin_lock_irqsave(&udc->lock, flags);
	ast_udc_write(udc, ast_udc_read(udc, AST_VHUB_CTRL) & ~ROOT_UPSTREAM_EN, AST_VHUB_CTRL);
	spin_unlock_irqrestore(&udc->lock, flags);

	device_init_wakeup(&pdev->dev, 0);
	free_irq(udc->irq, udc);
	iounmap(udc->reg);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	release_mem_region(res->start, resource_size(res));


	return 0;
}

#ifdef CONFIG_PM
static int ast_udc_suspend(struct platform_device *pdev, pm_message_t mesg)
{
	struct ast_udc *udc = platform_get_drvdata(pdev);
	int		wake = udc->driver && device_may_wakeup(&pdev->dev);
	unsigned long	flags;
	printk("TODO ~~");
	/* Unless we can act normally to the host (letting it wake us up
	 * whenever it has work for us) force disconnect.  Wakeup requires
	 * PLLB for USB events (signaling for reset, wakeup, or incoming
	 * tokens) and VBUS irqs (on systems which support them).
	 */
	if ((!udc->suspended && udc->addr)
			|| !wake) {
		spin_lock_irqsave(&udc->lock, flags);
		pullup(udc, 0);
		wake = 0;
		spin_unlock_irqrestore(&udc->lock, flags);
	} else
		enable_irq_wake(udc->irq);

	udc->active_suspend = wake;
	if (gpio_is_valid(udc->board.vbus_pin) && !udc->board.vbus_polled && wake)
		enable_irq_wake(udc->board.vbus_pin);
	return 0;
}

static int ast_udc_resume(struct platform_device *pdev)
{
	struct ast_udc *udc = platform_get_drvdata(pdev);
	unsigned long	flags;
	printk("TODO ~~");
	if (!udc->board.vbus_polled &&
	    udc->active_suspend)
		disable_irq_wake(udc->board.vbus_pin);

	/* maybe reconnect to host; if so, clocks on */
	if (udc->active_suspend)
		disable_irq_wake(udc->irq);
	else {
		spin_lock_irqsave(&udc->lock, flags);
		pullup(udc, 1);
		spin_unlock_irqrestore(&udc->lock, flags);
	}
	return 0;
}
#else
#define	ast_udc_suspend	NULL
#define	ast_udc_resume	NULL
#endif

#if defined(CONFIG_OF)
static const struct of_device_id ast_udc_dt_ids[] = {
	{ .compatible = "aspeed,ast-udc" },
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, ast_udc_dt_ids);
#endif

static struct platform_driver ast_udc_driver = {
	.remove		= __exit_p(ast_udc_remove),
	.suspend		= ast_udc_suspend,
	.resume		= ast_udc_resume,
#ifdef CONFIG_OF
	.probe 		= ast_udc_probe,
#endif
	.driver		= {
		.name	= "ast_udc",
		.owner	= THIS_MODULE,
		.of_match_table	= of_match_ptr(ast_udc_dt_ids),
	},
};

module_platform_driver_probe(ast_udc_driver, ast_udc_probe);

MODULE_DESCRIPTION("AST UDC driver");
MODULE_AUTHOR("Ryan Chen");
MODULE_LICENSE("GPL");

