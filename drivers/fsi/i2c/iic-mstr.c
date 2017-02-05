/*
 *   Copyright (c) International Business Machines Corp., 2006, 2009, 2010
 *
 *   This program is free software;  you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY;  without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See
 *   the GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program;  if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/sysfs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/param.h>
#include <linux/sched.h>
#include <linux/list.h>
#include <linux/slab.h>
#include <asm/page.h>
#include <linux/pagemap.h>
#include <linux/aio.h>
#include <linux/i2cfsi.h>
#include "iic-int.h"
#include "iic-fsi.h"
#include <linux/fsi.h>
#include <linux/time.h>
#include <asm/io.h>
#include <linux/moduleparam.h>
#include <linux/interrupt.h>

typedef struct
{
	unsigned long type;
	iic_eng_ops_t *ops;
	struct list_head list;
} iic_eng_type_t;

typedef struct
{
	unsigned long type;
	iic_bus_t * bus;
	struct list_head list;
} iic_bus_type_t;

/* This is a DLL of engine ops for the engines that are supported */
LIST_HEAD(iic_eng_type_list);

/* DLL for bus ops for the busses that are supported */
LIST_HEAD(iic_bus_type_list);

iic_opts_t iic_dflt_opts =
{
	.xfr_opts =
	{
		.dev_addr = 0,		/* 8 bits with LSB ignored */
		.dev_width = 0,		/* offset width in bytes */
		.offset = 0,		/* offset in bytes */
		.inc_addr = 0,		/* address increment mask */
		.timeout = 5000,	/* transfer timeout in milliseconds */
		.wdelay = 0,		/* write delay in milliseconds */
		.rdelay = 0,		/* read delay in milliseconds */ 
		.wsplit = 0,		/* write split chunk size (bytes) */
		.rsplit = 0,		/* read split chunk size (bytes) */
		.flags = 0,
	},
	.recovery =
	{
		.redo_pol = 0,
		.redo_delay = 0,
	}
};

static const char iic_mstr_version[] = "3.0";

/* save off the default cdev type pointer so we can call the default cdev
 * release function in our own bus release function
 */
static struct kobj_type* cdev_dflt_type = 0;
struct kobj_type iic_bus_type;

/* funtion called when cdev object (embedded in bus object) ref count
 * reaches zero.  (prevents cdev memory from being freed to early)
 */
void iic_bus_release(struct kobject* kobj)
{
	struct cdev *p = container_of(kobj, struct cdev, kobj);
	iic_bus_t* bus = container_of(p, iic_bus_t, cdev);

	IFLDi(1, "deleting bus[%08lx]\n", bus->bus_id);
	if(cdev_dflt_type && cdev_dflt_type->release)
		cdev_dflt_type->release(kobj);
	kfree(bus);
}

int iic_open(struct inode* inode, struct file* filp);
int iic_release(struct inode* inode, struct file* filp);
ssize_t iic_read(struct file *file, char __user *buf, size_t count,
		 loff_t *offset);
ssize_t iic_write(struct file *file, const char __user *buf, size_t count,
	       	  loff_t *offset);
ssize_t iic_aio_read(struct kiocb *iocb, const struct iovec *buf, unsigned long count, loff_t pos);
ssize_t iic_aio_write(struct kiocb *iocb, const struct iovec *buf, unsigned long count, loff_t pos);
long iic_ioctl(struct file *file, unsigned int cmd,
              unsigned long arg);
static int iic_mmap(struct file* file, struct vm_area_struct* vma);
int iic_xfr(iic_client_t* client, char* buf, size_t count, loff_t* offset, 
		char read_flag);
loff_t iic_llseek(struct file *filp, loff_t off, int whence);

struct file_operations iic_fops = {
	.owner = THIS_MODULE,
	.open = iic_open,
	.release = iic_release,
	.read = iic_read,
	.write = iic_write,
	.unlocked_ioctl = iic_ioctl,
	.llseek = iic_llseek,
	.mmap = iic_mmap,
};

int iic_common_open(iic_client_t ** o_client, iic_bus_t * bus, int engine_num)
{
	int ret = 0;
	iic_client_t * client;
	IENTER();


	BUG_ON(in_atomic());

	/*
	 * Create a client with the default attributes and associate it
	 * with the file descriptor.
	 */
	client = (iic_client_t*)kzalloc(sizeof(*client), GFP_KERNEL);
	if(!client)
	{
		ret = -ENOMEM;
		goto exit;
	}
	memcpy(&client->opts, &iic_dflt_opts, sizeof(iic_dflt_opts));

	if(!bus)
	{
		ret = -ENOMEM;
		kfree(client);
		goto exit;
	}
	else
	{
		client->flags |= IIC_CLIENT_SOURCE_USER;
	}

	client->bus = bus;
	client->tgid = current->tgid;
	sema_init(&client->sem, 1);
	init_waitqueue_head(&client->wait);
	kobject_get(&bus->eng->kobj);
	*o_client = client;

exit:
	IEXIT(0);
	return ret;
}

int iic_sideways_open(iic_client_t ** o_client,
		      iic_bus_t * bus,
		      int engine_num)
{
	return iic_common_open(o_client, bus, engine_num);
}
EXPORT_SYMBOL(iic_sideways_open);

int iic_open(struct inode* inode, struct file* filp)
{
	int ret = 0;
	iic_client_t* client;
	iic_bus_t* bus = container_of(inode->i_cdev,
				      iic_bus_t,
				      cdev);
	IENTER();
	if(!bus)
	{
		ret = -ENODEV;
		goto exit;
	}

	ret = iic_common_open(&client, bus, 0);
	filp->private_data = client;
	IFLDs(2, "OPEN     client[%p] bus[%08lx]\n", client, bus->bus_id);

exit:
	IEXIT(ret);
	return ret;
}

/* Abort all pending xfrs for a client, or if client is 0, abort all
 * pending xfrs for the engine.  
 */
int iic_abort_all(iic_eng_t* eng, iic_client_t* client, int status)
{
	unsigned long flags;
	iic_xfr_t *iterator, *temp;

	IENTER();
	/* abort currently running xfr */
	spin_lock_irqsave(&eng->lock, flags);
	if(eng->cur_xfr && (!client || ( eng->cur_xfr->client == client)))
	{
		iic_xfr_t* cur_xfr = eng->cur_xfr;
		cur_xfr->status = status;
		iic_abort_xfr(cur_xfr);

		iic_xfr_complete(cur_xfr);
	}

	/* abort queued xfrs */
	list_for_each_entry_safe(iterator, temp, &eng->xfrq, q_entry)
	{
		if(!client || (iterator->client == client))
		{
			iterator->status = status;
			iic_abort_xfr(iterator);
			iic_xfr_complete(iterator);
		}
	}
	spin_unlock_irqrestore(&eng->lock, flags);
	IEXIT(0);
	return 0;
}
EXPORT_SYMBOL(iic_abort_all);

int iic_common_release(iic_client_t * client)
{
        int rc = 0;
        iic_bus_t * bus = client->bus;

        IENTER();

	BUG_ON(in_atomic());

        /* abort all pending transfers for this client */
        iic_abort_all(bus->eng, client, -EPIPE);

        /* unlock any address locks associated with this client */
        spin_lock_irq(&bus->eng->lock);
        iic_unlock_all(&bus->eng->lck_mgr, client);
        iic_start_next_xfr(bus->eng);
        spin_unlock_irq(&bus->eng->lock);

        client->bus = 0;
        kfree(client);
        kobject_put(&bus->eng->kobj);

        IEXIT(rc);
        return rc;
}

int iic_sideways_release(iic_client_t * client)
{
	return iic_common_release(client);
}
EXPORT_SYMBOL(iic_sideways_release);

int iic_release(struct inode* inode, struct file* filp)
{
	iic_client_t* client = (iic_client_t*)filp->private_data;
	IENTER();

	IFLDs(2, "CLOSE    client[%p] bus[%08lx]\n", client, bus->bus_id);

	iic_common_release(client);

	/* Delete the client object associated with the file descriptor */
	filp->private_data = 0;

	IEXIT(0);
	return 0;
}
EXPORT_SYMBOL(iic_release);


void iic_cleanup_xfr(iic_xfr_t* xfr, dd_ffdc_t ** o_ffdc)
{
	IENTER();

	del_timer(&xfr->delay);
	del_timer(&xfr->timeout);
	kfree(xfr);
	IEXIT(0);
}

#ifndef MSEC_PER_SEC
#define MSEC_PER_SEC 1000
#endif
int iic_create_xfr(iic_client_t* client, struct kiocb* iocb, 
		   void* buf, size_t len, unsigned long flags,
		   iic_xfr_t** new_xfr, dd_ffdc_t ** o_ffdc)
{
	int rc = 0;
	iic_xfr_t *xfr;
	iic_xfr_opts_t *t_opts;
	iic_eng_t *eng = client->bus->eng;
	unsigned short j = 0, count = 0, size = 0;

	IENTER();

	xfr = (iic_xfr_t*) kmalloc(sizeof(iic_xfr_t), GFP_KERNEL);
	if(!xfr)
	{
		*new_xfr = 0;
		rc = -ENOMEM;
		IFLDe(0, "kmalloc xfr failed\n");
		goto exit;
	}

	memset(xfr, 0, sizeof(iic_xfr_t));

	/* Copy all client attributes neccesary for doing the transfer
	 * into the xfr struct.
	 */
	memcpy(&xfr->opts, &client->opts, sizeof(iic_opts_t));
	xfr->client = client;
	xfr->iocb = iocb;
	xfr->flags = flags;
	xfr->buf = (char*)buf;
	xfr->size = len;
	xfr->pid = current->pid;

	/* modify the xfr opts for ease of use in the device driver */
	t_opts = &xfr->opts.xfr_opts;
	t_opts->inc_addr = (t_opts->inc_addr >> 1) << (t_opts->dev_width * 8);

	/* device driver code will only look at the rdelay and rsplit fields.
	 * wdelay and wsplit values will be copied to rdelay and rsplit if
	 * this is a write transfer.
	 */
	if(test_bit(IIC_XFR_RD, &xfr->flags))
	{
		if(t_opts->rsplit)
		{
			if (t_opts->rsplit > 0x8000)
			{
				t_opts->rsplit = 0x7FFF;
			}
			else {
				t_opts->rsplit = t_opts->rsplit - 1;
			}
		}
		else {
			t_opts->rsplit = 0x7FFF;
		}
	}
	else
	{
		unsigned long data_sz = xfr->size;
		unsigned long start;

		if(t_opts->wsplit)
		{
			t_opts->rsplit = t_opts->wsplit - 1;
			t_opts->rdelay = t_opts->wdelay;
		}

		/* store off the first 4 bytes of write transfers now
		 * for ffdc.
		 */
		if(data_sz > sizeof(long))
			data_sz = sizeof(long);
		start = sizeof(long) - data_sz;
	}

	/* prevent split numbers that just have one bit set (0x800,
	 * 0x20, etc) to avoid problems with split calculation
	 * in engine
	 */
	count = 0;
	size = sizeof(t_opts->rsplit) * 8;
	for (j = 0; j < size && count <= 1; j++) {
		if (t_opts->rsplit & (1 << j))
			count++;
	}

	if (count == 1 && t_opts->rsplit > 2)
		t_opts->rsplit = t_opts->rsplit - 1;

	if(test_bit(IIC_ENG_BLOCK, &eng->flags))
	{
		IFLDe(1, "eng[%08x] blocked\n", eng->id);
		rc = -ENODEV;
		if(test_bit(IIC_ENG_REMOVED, &eng->flags))
			rc = -ENOLINK;
		xfr->status = rc;
		goto error;
	}

	rc = 0;

	*new_xfr = xfr;
	goto exit;
		
error:
	kfree(xfr);
	*new_xfr = 0;

exit:
	IEXIT(rc);
	return rc;
}

/* called within a timer context to continue a delayed transfer */
void iic_continue_xfr(unsigned long data)
{
	iic_xfr_t *xfr = (iic_xfr_t*)data;
	IENTER();
	IFLDd(1, "CONTINUE xfr[%p]\n", xfr);
	clear_bit(IIC_XFR_DELAYED, &xfr->flags);

	iic_start_next_xfr(xfr->client->bus->eng);
	IEXIT(0);
}

/* Called by the engine when a transfer should only be continued after
 * a period of time has expired.
 * This is needed for implementing write delays.
 * Note: delay is in milliseconds!
 */
void iic_delay_xfr(iic_xfr_t* xfr, unsigned long delay)
{
	iic_eng_t *eng = xfr->client->bus->eng;
	IENTER();
	IFLDd(2, "DELAY    xfr[%p] time[%ld]\n", xfr, delay);
	eng->cur_xfr = 0;

	/* Get the next xfr started (if any) */
	iic_start_next_xfr(eng);

	/* Make sure the delayed bit is set */
	set_bit(IIC_XFR_DELAYED, &xfr->flags);

	/* Place this xfr back at the beginning of the queue */
	list_add(&xfr->q_entry, &eng->xfrq);
	
	/* Start a timer that will allow the transfer to start back up
	 * when it pops.
	 */
	xfr->delay.data = (unsigned long)xfr;
	xfr->delay.function = iic_continue_xfr;
	mod_timer(&xfr->delay, jiffies + msecs_to_jiffies( delay ) );
	IEXIT(0);

}
EXPORT_SYMBOL(iic_delay_xfr);

void iic_finish_complete(unsigned long data)
{
	iic_xfr_t *xfr = (iic_xfr_t*)data;
	IENTER();
	clear_bit(IIC_XFR_DELAYED, &xfr->flags);
	iic_xfr_complete(xfr);
	IEXIT(0);
}

#define NUM_RETRIES 15
#define RETRY_DELAY 5
#define BACKOFF_DELAY 500

/* Retry timeout fails for relatively long timeout periods */
static unsigned long allow_retry(iic_xfr_t* xfr)
{
	/* No retry allowed - use original timeout period */
	return (xfr->opts.xfr_opts.timeout);
}

unsigned long error_match(int status, unsigned long policy, iic_xfr_t* xfr)
{
	unsigned long error_bit = 0;
	unsigned long rc = 0;
	IENTER();
	switch(status)
	{
		case -ENXIO:
			/* Allow one retry for addr NACK */
			rc = 1;
			error_bit = IIC_VAL_ADDR_NOACK;
			break;
		case -ENODATA:
			error_bit = IIC_VAL_DATA_NOACK;
			break;
		case -ETIME:
			/* Allow one retry for long timeout periods */
			if (allow_retry(xfr) != xfr->opts.xfr_opts.timeout)
				rc = 1;
			error_bit = IIC_VAL_TIMEOUT;
			break;
		case -EALREADY:
			error_bit = IIC_VAL_LOST_ARB;

			/* More retries hardcoded for bus multimaster failure
			   This behavior can be overridden by user config
			   set delay to 5 ms */
			if (!(error_bit & policy)) {
				/* the return code should actually be the
				   number of retries. See comparison in
				   rec_retry */
				rc = NUM_RETRIES;
				xfr->opts.recovery.redo_delay = RETRY_DELAY;
				goto exit;
			}
			break;
		case -EIO:
			/* Allow retries for bus errors */
			rc = 3;
			error_bit = IIC_VAL_BUS_ERR;
			break;
		default:
			break;
	}
	if(error_bit & policy)
		rc = policy & ~IIC_VAL_ALL_ERRS;
exit:
	IEXIT((int)rc);
	return rc;
}

/* 1 means 1 retry, 0 means no retries. */
unsigned short rec_retry(iic_xfr_t* xfr)
{
	unsigned short rc;
	IENTER();
	rc = error_match(xfr->status, xfr->opts.recovery.redo_pol, xfr);
	if(rc <= xfr->retry_count)
		rc = 0;
	IEXIT(rc);
	return 0;
}

/* Returns the delay needed prior to retry.  If a read or write delay
 * was specified and is larger than the retry delay or the error
 * isn't a policy match, then the read/write delay will be returned.
 */
unsigned long rec_delay(iic_xfr_t* xfr)
{
	unsigned long rc;
	IENTER();
	rc = xfr->opts.recovery.redo_delay ;
	if(rc < xfr->opts.xfr_opts.rdelay)
		rc = xfr->opts.xfr_opts.rdelay;
	IEXIT((int)rc);
	return rc;
}

/* Keep IIC_XFR_RD, IIC_XFR_ASYNC, and IIC_XFR_FAST when retrying xfr. */
#define IIC_XFR_RESET_MASK 0x00000007

/* Called by the engine specific code to notify us that the transfer ended.
 * If the transfer requires a delay before starting a new transfer,
 * (i.e., the IIC_XFR_DELAYED bit is set), then unlocking the address,
 * notifying caller of completion, and cleanup of transfer will be delayed
 * using a kernel timer.
 */ 
void iic_xfr_complete(iic_xfr_t* xfr)
{
	unsigned short delay;
	int rc = 0;
	iic_eng_t *eng = 0;
	iic_xfr_opts_t *opts;

	IENTER();

	if(!xfr)
	{
		IFLDe(0, "iic_xfr_complete called on null xfr!\n");
		goto exit;
	}

	opts = &xfr->opts.xfr_opts;

	if(test_bit(IIC_XFR_ENDED, &xfr->flags) ||
	   test_bit(IIC_XFR_RETRY_IN_PROGRESS, &xfr->flags))
	{
		IFLDd(2, "iic_xfr_complete xfr[%p] flags[%08lx] no-op\n", 
				xfr, xfr->flags);
		goto exit;
	}
	
	eng = xfr->client->bus->eng;

	if(xfr->status == -ETIME && eng->ops->finish_rescue_timeout) {
		xfr->status =
			((rc = eng->ops->finish_rescue_timeout(eng, xfr)) >=
			(long int)xfr->size)
			? 0
			: -ETIME;
		if(xfr->status == 0 && xfr->bytes_xfrd < xfr->size)
			xfr->client->flags |= IIC_CLIENT_EOD;
		IFLDd(1, "xfr->status[%d]\n", xfr->status);
	}

	/* Check if we need to retry this transfer if it failed.
	 * Only the first failure's FFDC and status will be kept.
	 * If the transfer succeeds on a retry, the FFDC will be freed
	 * and no error will be reported.
	 */
	if(rec_retry(xfr))
	{
			
		IFLDi(7, "RETRY    client[%p], bus[%d.%d:%d.%d.%d.%d]\n", 
		      xfr->client, IIC_GET_PLINK(eng->id), IIC_GET_PCFAM(eng->id),
		      IIC_GET_LINK(eng->id), IIC_GET_CFAM(eng->id), 
		      IIC_GET_ENG(eng->id), xfr->client->bus->port);
		IFLDi(3, "  xfr[%p] count[%d] status[%d]\n", 
		      xfr, xfr->retry_count + 1, xfr->status);

		/* increment retry count */
		xfr->retry_count++;

		/* reset timeout timer */
		if(xfr->opts.xfr_opts.timeout)
		{
			mod_timer(&xfr->timeout, jiffies + 
				   msecs_to_jiffies( allow_retry(xfr) ) );
		}

		/* if xfr timed out before starting, just leave it
		 * on the queue and give it another chance to run.
		 */
		if(!test_bit(IIC_XFR_STARTED, &xfr->flags))
		{
			goto exit;
		}

		/* reset xfr to start at the beginning
		 * Note: can't do DMA on a retry because
		 * dma_setup can't be called from a
		 * interrupt handler.
		 */ 
		delay = rec_delay(xfr);

		/* Multi-master - Backoff an extended period after every four retries */
		if ((xfr->status == -EALREADY) && ((xfr->retry_count % 4) == 0))
		{
			delay += BACKOFF_DELAY;

			/* adjust timeout timer to include backoff */
			if(xfr->opts.xfr_opts.timeout)
			{
				mod_timer(&xfr->timeout, jiffies + 
					   msecs_to_jiffies( xfr->opts.xfr_opts.timeout +
								 BACKOFF_DELAY ) );
			}
		}

		xfr->status = 0;
		xfr->flags &= IIC_XFR_RESET_MASK;

		/* notify others that a retry is in progress, so don't
		 * call iic_xfr_complete until retry is attempted.
		 * This flag is cleared when an error occurs or the xfr
		 * completes successfully or is cancelled.  (Problem
		 * noticed in timeout function when dma xfrs were retried.)
		 */
		set_bit(IIC_XFR_RETRY_IN_PROGRESS, &xfr->flags);
		
		/* Always call iic_delay_xfr so that failed transfers
		 * are given time to be cleaned up before we try
		 * a new transfer.  If the delay is 0, the transfer
		 * is placed back on the queue and started as soon as
		 * cleanup of the previous attempt completes
		 */
		iic_delay_xfr(xfr, delay);
		goto exit;
	}


#ifdef DELAYED_COMPLETION
	if(!test_bit(IIC_XFR_DELAYED, &xfr->flags))
	{
#endif
		/* if this xfr currently owned the address lock, release it */
		if(xfr->addr_lck->cur_xfr == xfr)
		{
			xfr->addr_lck->cur_xfr = 0;
		}

		/* unlock this xfr's address lock or dequeue lock request */
		IDBGd(1, "xfr[%p] releasing lock\n", xfr);
		iic_unlock(&xfr->client->bus->eng->lck_mgr, xfr->addr_lck);
#ifdef DELAYED_COMPLETION
	}
#endif

	/* If a transfer isn't already running, check if one is ready and
	 * start it.
	 */
	if(eng->cur_xfr == xfr)
	{
		eng->cur_xfr = 0;
	}
	iic_start_next_xfr(eng);

	/* Once iic_xfr_complete is called, the timeout and delay timers are
	 * no longer needed.
	 */
	del_timer(&xfr->timeout);

#ifdef DELAYED_COMPLETION
	/* For transfers that require a delay, take care of unlocking
	 * the address, completion notification, and cleanup later.
	 */
	if(test_bit(IIC_XFR_DELAYED, &xfr->flags))
	{
		xfr->delay.data = (unsigned long)xfr;
		xfr->delay.function = iic_finish_complete;
		mod_timer(&xfr->delay, jiffies + msecs_to_jiffies( xfr->opts.xfr_opts.rdelay ) );
		IFLDd(2, "DELAYCOMP xfr[%p] time[%d]\n", xfr,
						xfr->opts.xfr_opts.rdelay);
		goto exit;
	}
#endif
	del_timer(&xfr->delay);

	set_bit(IIC_XFR_ENDED, &xfr->flags);

	IFLDi(7, "COMPLETE client[%p] bus[%d.%d:%d.%d.%d.%d]\n", 
	      xfr->client, IIC_GET_PLINK(eng->id), IIC_GET_PCFAM(eng->id),
	      IIC_GET_LINK(eng->id), IIC_GET_CFAM(eng->id), IIC_GET_ENG(eng->id),
	      xfr->client->bus->port);
	IFLDi(2, "  xfr[%p] status[%d]\n", xfr, xfr->status);

	/**
	 * defer queueing of ffdc to the calling thread
	 * or to iic_cleanup_xfr for async transfers.
	 */

	/* for async transfers, just call aio_complete and then cleanup
	 * the xfr object.
	 */
	if(test_bit(IIC_XFR_ASYNC, &xfr->flags))
	{
		xfr->status = (xfr->status)? xfr->status: xfr->bytes_xfrd;
		IFLDd(1, "aio_complete xfr[%p]\n", xfr);
//		aio_complete(xfr->iocb, xfr->status, 0);
		iic_cleanup_xfr(xfr, NULL);
	}
	
	/* for sync transfers, just wake up the calling thread.  The
	 * calling thread will handle any necessary cleanup.
	 */
	else
	{
		IFLDd(1, "wake xfr[%p] client\n", xfr);
		wake_up_interruptible(&xfr->client->wait);
	}
exit:
	IEXIT(0);
	return;
}
EXPORT_SYMBOL(iic_xfr_complete);

/* This function is either called within an interrupt context or when
 * interrupts are disabled.  This function is called as recovery from
 * various types of failures.  FFDC should already be filled in before
 * calling this function.  Failures caused by the abort are ignored.
 */
void iic_abort_xfr(iic_xfr_t* xfr)
{
	int rc = 0;
	iic_eng_t *eng = xfr->client->bus->eng;
	IENTER();
	IFLDi(1, "ABORTREQ xfr[%p]\n", xfr);

	if(test_bit(IIC_XFR_ABORT, &xfr->flags))
	{
		IDBGd(0, "abort already started!\n");
		goto exit;
	}

	/* If this was a retry, the retry completed.  clear flag so that
	 * iic_xfr_complete can do its work.
	 */	
	clear_bit(IIC_XFR_RETRY_IN_PROGRESS, &xfr->flags);
	
	set_bit(IIC_XFR_ABORT, &xfr->flags);
	/* If the xfr is still waiting to run, remove it from the queue */
	if(eng->cur_xfr != xfr)
	{
		list_del(&xfr->q_entry);
	}
	/* Otherwise, the xfr is running.  Lock the engine, Signal the hw 
	 * to halt the transfer.
	 */
	else
	{
		/* lock the engine so we don't try to start a new transfer
		 * until the current transfer is aborted
		 */
		set_bit(IIC_ENG_ABORT, &eng->flags);

		/* once the IIC_ENG_ABORT flag is set, the interrupt
		 * handler will no longer access the xfr data structure
		 * and it's safe to set the IIC_XFR_ENG_COMPLETE flag.
		 */
		set_bit(IIC_XFR_ENG_COMPLETED, &xfr->flags);

		/* don't access hw if failed due to a parent bus access error */
		if(!test_bit(IIC_NO_ACCESS, &eng->flags))
		{
			/* start the abort procedure */
			rc = eng->ops->start_abort(eng, 0/*ignore ffdc*/);
		}

		/* Finish off the abort inside a work queue context.  When
		 * the abort is completed, the engine will get unlocked and
		 * iic_start_next_xfr will get called.
		 */
		schedule_work(&eng->work);

	}
exit:
	IEXIT(0);
	return;
}
EXPORT_SYMBOL(iic_abort_xfr);

/* Work queue function that finishes an abort operation */
void iic_finish_abort(struct work_struct * work)
{
	unsigned long flags;
	iic_eng_t* eng = container_of(work, iic_eng_t, work);
	IENTER();
	/* don't access hw if we lost engine access */
	if(!test_bit(IIC_NO_ACCESS, &eng->flags))
	{
		eng->ops->finish_abort(eng, 0);
	}
	spin_lock_irqsave(&eng->lock, flags);
	clear_bit(IIC_ENG_ABORT, &eng->flags);
	iic_start_next_xfr(eng);
	spin_unlock_irqrestore(&eng->lock, flags);
	IFLDd(0, "ABORTREQ (completed)\n");
	IEXIT(0);
}

	
/* Timer function that handles the case where a transfer or abort takes
 * too long to complete.
 */
void iic_timeout(unsigned long data)
{
	iic_xfr_t *xfr = (iic_xfr_t*)data;
	iic_eng_t *eng  = xfr->client->bus->eng;
	unsigned long flags;

	spin_lock_irqsave(&eng->lock, flags);
	IENTER();
	IFLDi(1, "TIMEOUT  xfr[%p]\n", xfr);
	if(test_bit(IIC_XFR_ENDED, &xfr->flags))
		goto exit;

	if(eng->ops->start_rescue_timeout) {
		int rc;
		xfr->status =
			((rc = eng->ops->start_rescue_timeout(eng, xfr)) >=
			(long int)xfr->size)
			? xfr->status
			: -ETIME;
		if(xfr->status == 0 && xfr->bytes_xfrd < xfr->size)
			xfr->client->flags |= IIC_CLIENT_EOD;
	} else
		xfr->status = -ETIME;

	IFLDd(1, "xfr->status[%d]\n", xfr->status);

	/* makes sure xfr_complete gets called in dma callback function */
	set_bit(IIC_XFR_ENG_COMPLETED, &xfr->flags);

	/* for DMA, this will cause dma_notify to get called which
	 * calls our callback function, which calls
	 * abort_xfr / xfr_complete.
	 */
	iic_abort_xfr(xfr);

	/* Don't force users to wait for the abort to complete */
	iic_xfr_complete(xfr);
exit:
	spin_unlock_irqrestore(&eng->lock, flags);
	IEXIT(0);
}

int iic_xfr_ready(iic_xfr_t* xfr)
{
	int rc = 0;  //xfr not ready
	iic_lck_t *lck = xfr->addr_lck;
	IENTER();

	/* If this xfr owns the lock and isn't write delayed, and isn't
	 * blacklisted, the transfer is ready to run.
	 */
	if(!test_bit(IIC_XFR_DELAYED, &xfr->flags) &&
			(lck->count > 0) &&
			((lck->cur_xfr == 0) || (lck->cur_xfr == xfr)))
	{
		IDBGf(1, "xfr[%p] good to go\n", xfr);
		lck->cur_xfr = xfr;
		rc = 1;
	}

	IEXIT(rc);
	return rc;
}

int iic_start_next_xfr(iic_eng_t* eng)
{
	int rc = 0;
	iic_xfr_t *iterator, *xfr;
	IENTER();
	xfr = 0;

	/* if a xfr is already running, or there is an abort or reset then
	 * do nothing.
	 */
	if(eng->cur_xfr || 
	   test_bit(IIC_ENG_ABORT, &eng->flags) ||
	   test_bit(IIC_ENG_RESET, &eng->flags) ||
	   test_bit(IIC_ENG_BLOCK, &eng->flags))
	{
		/* Notify thread waiting to do reset that the engine might
		 * be idle now.
		 */
		if(test_bit(IIC_ENG_RESET, &eng->flags))
		{
			wake_up_interruptible(&eng->waitq);
		}
		goto exit;
	}

	IDBGl(0, "Looking for next xfr\n");
	/* scan the queue from the beginning for a transfer that's ready */
	/* if the process that submitted the xfr is black-listed, it will
	 * be skipped
	 */
	list_for_each_entry(iterator, &eng->xfrq, q_entry)
	{
		if(iic_xfr_ready(iterator))
		{
			xfr = iterator;
			break;
		}
	}

	/* If a xfr is ready to go,  start it */
	if(xfr)
	{
		/* set the delay bit here if necessary so that if the transfer
		 * is aborted other transfers to the same address will be
		 * delayed appropriately in iic_xfr_complete.
		 */
		if(xfr->opts.xfr_opts.rdelay)
		{
			set_bit(IIC_XFR_DELAYED, &xfr->flags);
		}
		eng->cur_xfr = xfr;
		list_del(&xfr->q_entry);
		if(!test_bit(IIC_XFR_STARTED, &xfr->flags))
			IFLDs(3, "START    client[%p] bus[%08lx] xfr[%p]\n",
				xfr->client, xfr->client->bus->bus_id, xfr);
		clear_bit(IIC_NO_ACCESS, &eng->flags);
		set_bit(IIC_XFR_STARTED, &xfr->flags);
		rc = eng->ops->start(xfr); 
		if(rc)
		{
			/* If this was a retry, the retry completed.  
			 * clear flag so that
			 * iic_xfr_complete can do its work.
			 */	
			clear_bit(IIC_XFR_RETRY_IN_PROGRESS, &xfr->flags);

			IFLDe(2, "xfr[%p] start failed: %d\n", xfr, rc);
			iic_abort_xfr(xfr); 
			set_bit(IIC_XFR_ENG_COMPLETED, &xfr->flags);

			iic_xfr_complete(xfr);
		}
	}

exit:
	IEXIT(rc);
	return rc;
}

/* Adds a xfr to the end of the queue */
int iic_enq_xfr(iic_xfr_t *xfr)
{
	int rc;
	unsigned long flags;
	iic_eng_t *eng = xfr->client->bus->eng;
	iic_xfr_opts_t *opts = &xfr->opts.xfr_opts;
	IENTER();
	spin_lock_irqsave(&eng->lock, flags);

	/* Submit a lock request for this xfr (non-blocking) */
	rc = iic_req_lock(&eng->lck_mgr,
			  opts->dev_addr,
			  (opts->inc_addr >> (opts->dev_width * 8)),
			  xfr->client,
			  &xfr->addr_lck);
	if(rc < 0)
	{
		goto exit;
	}

	/* enqueue this xfr */
	IFLDi(7, "SUBMIT   client[%p] bus[%d.%d:%d.%d.%d.%d]\n",
	      xfr->client, IIC_GET_PLINK(eng->id), IIC_GET_PCFAM(eng->id), 
	      IIC_GET_LINK(eng->id), IIC_GET_CFAM(eng->id), IIC_GET_ENG(eng->id), 
	      xfr->client->bus->port); 
	IFLDi(5, "  xfr[%p] addr[%04x:%04x] sz[%08lx] timeout[%ld]\n", 
	      xfr, opts->dev_addr + ((test_bit(IIC_XFR_RD, &xfr->flags))? 1:0), 
	      opts->rsplit, xfr->size, opts->timeout);
	list_add_tail(&xfr->q_entry, &eng->xfrq);
	set_bit(IIC_XFR_QUEUED, &xfr->flags);

	/* start a kernel timer that will abort the transfer 
	 * if it takes too long.
	 */
	init_timer(&xfr->timeout);
	xfr->timeout.data = (unsigned long)xfr;
	xfr->timeout.function = iic_timeout;
	if(opts->timeout)
	{
		xfr->timeout.expires = jiffies +
				msecs_to_jiffies( allow_retry(xfr) );
		add_timer(&xfr->timeout);
	}
	init_timer(&xfr->delay);

	/* If no transfers are currently active, scan the queue for the
	 * next transfer and start it
	 */
	iic_start_next_xfr(eng); 

	rc = -EIOCBQUEUED;
exit:
	spin_unlock_irqrestore(&eng->lock, flags);
	IEXIT(rc);

	return rc;
}

int iic_wait_xfr(iic_xfr_t *xfr)
{
	int rc = 0;
	unsigned long flags;

	IENTER();
	IFLDd(2, "WAIT     xfr[%p] time[%ld]\n", 
			xfr, xfr->opts.xfr_opts.timeout);
	rc = wait_event_interruptible(xfr->client->wait, 
			test_bit(IIC_XFR_ENDED, &xfr->flags));
	if(rc < 0)
	{
		/* EINTR is always retried at the adal level.  ADAL users
		 * will never see the EINTR errno and won't know to collect
		 * FFDC for it, so don't generate FFDC for EINTR but do
		 * trace it.
		 */
		spin_lock_irqsave(&xfr->client->bus->eng->lock, flags);
		if(!xfr->status)
			xfr->status = -EINTR;
		IFLDe(2, "aborting xfr[%p] due to signal. pid[%d]\n",
			xfr, xfr->pid);
		iic_abort_xfr(xfr);

		/* Don't force users to wait for the abort to complete */
		iic_xfr_complete(xfr);
		spin_unlock_irqrestore(&xfr->client->bus->eng->lock, flags);
	}
	IEXIT(rc);
	return rc;
}

/*
 * Shared read method between user space applications and sideways kernel
 * calls.
 */
ssize_t iic_common_read(iic_client_t * client, void * buf, size_t count,
                        loff_t *offset, dd_ffdc_t ** o_ffdc)
{
	ssize_t rc = count;
	iic_xfr_t *xfr;

	IENTER();

	BUG_ON(in_atomic());

	if(!count)
	{
		rc = -EINVAL;
		goto no_up;
	}

	if(down_interruptible(&client->sem))
	{
		rc = -EINTR;
		goto no_up;
	}

	rc = iic_create_xfr(client, 0, buf, count, (1 << IIC_XFR_RD), &xfr,
			o_ffdc);
	if(rc)
	{
		goto exit;
	}

	/* enqueue or start the xfr */
	rc = iic_enq_xfr(xfr);
	if(rc != -EIOCBQUEUED)
	{
		goto error;
	}

	/* wait for xfr to complete */
	iic_wait_xfr(xfr);

	/* set rc appropriately */
	if(xfr->status)
	{
		rc = xfr->status;
	}
	else
	{
		rc = xfr->bytes_xfrd;
		client->opts.xfr_opts.offset += rc;
	}

	/* Data is already in the user buffer at this point.
	 * Cleanup the transfer and return status to the user.
	 */

error:
	iic_cleanup_xfr(xfr, o_ffdc);
exit:
	up(&client->sem);
no_up:
	IEXIT(rc);
	return rc;
}

ssize_t iic_sideways_read(iic_client_t * client, void * buf, size_t count,
                         loff_t *offset, dd_ffdc_t ** o_ffdc)
{
	client->opts.xfr_opts.offset = *offset;
	return iic_common_read(client, buf, count, offset, o_ffdc);
}
EXPORT_SYMBOL(iic_sideways_read);

ssize_t iic_read(struct file *filp, char __user *buf, size_t count,
		 loff_t *offset)
{
	ssize_t rc = count;
	char *kbuf;
	iic_client_t *client = (iic_client_t*)filp->private_data;

	IENTER();

	if (client->flags & IIC_CLIENT_EOD) {
		client->flags &= ~(IIC_CLIENT_EOD);
		return 0;
	}

	if(filp->f_flags & O_NONBLOCK)
	{
		rc = -EAGAIN;
		goto exit;
	}

	if(!access_ok(VERIFY_READ, buf, count))
	{
		rc = -EFAULT;
		goto exit;
	}

	kbuf = kzalloc(count, GFP_KERNEL);
	if (!kbuf) {
		rc = -ENOMEM;
		goto exit;
	}

	rc = iic_common_read(client, kbuf, count, offset, NULL);
	if (rc < 0)
		goto free;

	rc = copy_to_user(buf, kbuf, count);

free:
	kfree(kbuf);

exit:
	IEXIT(rc);
	return rc;
}

/*
 * Shared write method between user space and kernel 'sideways' calls.
 */
ssize_t iic_common_write(iic_client_t * client, void * buf, size_t count,
                         loff_t * offset, dd_ffdc_t ** o_ffdc)
{
	ssize_t rc = count;
	iic_xfr_t *xfr;

	IENTER();

	BUG_ON(in_atomic());

	if(!count)
	{
		rc = -EINVAL;
		goto no_up;
	}

	if(down_interruptible(&client->sem))
	{
		rc = -EINTR;
		goto no_up;
	}

	rc = iic_create_xfr(client, 0, buf, count, 0, &xfr, o_ffdc);
	if(rc)
	{
		goto exit;
	}

	/* enqueue or start the xfr */
	rc = iic_enq_xfr(xfr);
	if(rc != -EIOCBQUEUED)
	{
		goto error;
	}

	/* wait for xfr to complete */
	iic_wait_xfr(xfr);

	/* set rc appropriately */
	if(xfr->status)
	{
		rc = xfr->status;
	}
	else
	{
		rc = xfr->bytes_xfrd;
		client->opts.xfr_opts.offset += rc;
	}

error:
	iic_cleanup_xfr(xfr, o_ffdc);
exit:
	up(&client->sem);
no_up:
	IEXIT(rc);
	return rc;
}

ssize_t iic_sideways_write(iic_client_t * client, void * buf, size_t count,
                          loff_t * offset, dd_ffdc_t ** o_ffdc)
{
	client->opts.xfr_opts.offset = *offset;
	return iic_common_write(client, buf, count, offset, o_ffdc);
}
EXPORT_SYMBOL(iic_sideways_write);

ssize_t iic_write(struct file *filp, const char __user *buf, size_t count,
	       	  loff_t *offset)
{
	ssize_t rc = count;
	char *kbuf;
	iic_client_t *client = (iic_client_t*)filp->private_data;

	IENTER();

	if (client->flags & IIC_CLIENT_EOD) {
		client->flags &= ~(IIC_CLIENT_EOD);
		return 0;
	}

	/* don't support posted writes at this time */
	if(filp->f_flags & O_NONBLOCK)
	{
		rc = -EAGAIN;
		goto exit;
	}

	if(!access_ok(VERIFY_WRITE, buf, count))
	{
		rc = -EFAULT;
		goto exit;
	}

	kbuf = kzalloc(count, GFP_KERNEL);
	if (!kbuf) {
		rc = -ENOMEM;
		goto exit;
	}

	rc = copy_from_user(kbuf, buf, count);
	if (rc)
		goto free;

	rc = iic_common_write(client, kbuf, count, offset, NULL);

free:
	kfree(kbuf);

exit:
	IEXIT(rc);
	return rc;
}

/* timout is in milliseconds! */
int iic_reset(iic_bus_t* bus, int timeout, iic_ffdc_t** ffdc)
{
	int rc;
	IENTER();
	//IFLDi(1, "bus[%08lx]: reset requested\n", bus->bus_id);
	/* block new transfers from starting on the engine */
	set_bit(IIC_ENG_RESET, &bus->eng->flags);

	/* wait for any pending operations on the engine to complete */
	/* Note - timeout must be in jiffies for wait_for_idle! */
	rc = bus->eng->ops->wait_for_idle(bus->eng, msecs_to_jiffies( timeout ), ffdc);
	if(!rc)
	{
		/* do the reset */
		rc = bus->eng->ops->reset_bus(bus, ffdc);
		if(!rc)
		{
			set_current_state(TASK_UNINTERRUPTIBLE);

			/* schedule_timeout requires its parameter in jiffies. */
			rc = schedule_timeout(IIC_RESET_DELAY);
		}
	}
	if(rc < 0)
	{
		IFLDe(2, "bus[%08lx] reset failed: %d\n", bus->bus_id, rc);
	}
	else
	{
		IFLDi(2, "bus[%08lx]: reset complete. stucked[%d]",
		      bus->bus_id, (rc == 1)? 1: 0);
	}

	/* restart processing of new transfers */
	spin_lock_irq(&bus->eng->lock);
	clear_bit(IIC_ENG_RESET, &bus->eng->flags);
	iic_start_next_xfr(bus->eng);
	spin_unlock_irq(&bus->eng->lock);

	IEXIT(rc);
	return rc;

}
EXPORT_SYMBOL(iic_reset);

/* We need to make sure no transfers are in progress before reading the state
 * of a bus in case we need to switch to a different bus.
 * Note: timeout is in milliseconds!
 */
int iic_get_bus_state(iic_bus_t* bus, unsigned long* state, int timeout, 
		iic_ffdc_t** ffdc)
{
	int rc;
	
	IENTER();
	/* block new transfers from starting on the engine */
	set_bit(IIC_ENG_RESET, &bus->eng->flags);
	
	/* wait for any pending operations on the engine to complete
	 * Note:  timeout must be in jiffies
	 */
	rc = bus->eng->ops->wait_for_idle(bus->eng, msecs_to_jiffies( timeout ), ffdc);
	if(!rc)
	{
	        /* check bus state */
	        rc = bus->eng->ops->get_bus_state(bus, state, ffdc);
		IDBGs(3, "get_bus_state[%08lx]: state=%08lx, rc=%d\n",
				bus->bus_id, *state, rc);
	}
	
	/* restart processing of new transfers */
	spin_lock_irq(&bus->eng->lock);
	clear_bit(IIC_ENG_RESET, &bus->eng->flags);
	iic_start_next_xfr(bus->eng);
	spin_unlock_irq(&bus->eng->lock);
	        
	IEXIT(rc);
	return rc;
}

#define IIC_W(a) _IOC(_IOC_WRITE, 0, a, 0)
#define IIC_R(a) _IOC(_IOC_READ, 0, a, 0)

/*
During an I2C transfer there is often the need to first send a command 
and then read back an answer right away. This has to be done without the 
risk of another (multimaster) device interrupting this atomic operation. 
The I2C protocol defines a so-called repeated start condition. After 
having sent the address byte (address and read/write bit) the master may 
send any number of bytes followed by a stop condition. Instead of sending 
the stop condition it is also allowed to send another start condition 
again followed by an address (and of course including a read/write bit) 
and more data. This is defined recursively allowing any number of start 
conditions to be sent. The purpose of this is to allow combined 
write/read operations to one or more devices without releasing the bus 
and thus with the guarantee that the operation is not interrupted.

Before reading data from the slave, you must tell it which of its
internal address (offset) you want to read.
So a read of the slave actually starts off by writing to it.
This is the same as when you want to write to it: You send the
start sequence, the I2C address of the slave with the R/W bit
and the internal register number (i.e offset) you want to write to.
Now you send another start sequence (sometimes called a restart)
and the I2C address again - this time
with the read bit set. You then read as many data bytes as you
wish and terminate the transaction with a stop sequence.
*/

int iic_repeated_xfer(iic_client_t *client, struct i2c_msg msgs[], int num)
{
	struct i2c_msg *pmsg;
	iic_xfr_t *xfr;
	iic_opts_t* opts;
	iic_xfr_opts_t* xfr_opts;
	u8 __user **data_ptrs;
	u8 *current_msg_buf_ptr;
	int rc = 0;
	int i;
	unsigned long options;

	opts = &client->opts;
	xfr_opts = &opts->xfr_opts;
	//unsigned long new_port = xfr->client->bus->port;

	data_ptrs = kmalloc(num * sizeof(u8 __user *), GFP_KERNEL);
        if (data_ptrs == NULL) {
                rc = -ENOMEM;
		goto exit;
        }

	// Get the offset from the configuration which is the default.
	// The default will be overridden by the message.
	for (i = 0; i < num; i++) {
		pmsg = &msgs[i];
		if (!pmsg->len) /* If length is zero */
                     continue;  /* on to the next request. */
		data_ptrs[i] = (u8 __user *)msgs[i].buf;
		current_msg_buf_ptr = kmalloc(pmsg->len, GFP_KERNEL); 
		if (current_msg_buf_ptr == NULL) 
		{
			rc = -ENOMEM;
			goto error;
		}
		// Bring over the user space buffer in order to
		// retrieve the offset.
		// We need to set up the offset before calling
		// iic_create_xfr. 
		// We still pass the user buffer to the iic_create_xfr because
		// the function will do its own conversion.
	  	if(copy_from_user(current_msg_buf_ptr,
                         data_ptrs[i],
                         msgs[i].len)) 
		{
                        rc = -EFAULT;
			kfree(current_msg_buf_ptr);
			goto error;
                }

		options = 0;
		if (i != num -1)
		{
			set_bit(IIC_REPEATED_START, &options);
			xfr_opts->flags |= options;
		}
		else
		{
			clear_bit(IIC_REPEATED_START, &options);
			xfr_opts->flags &= options;
		}

		// Need to set the slave address here
		xfr_opts->dev_addr =  pmsg->addr;
		// The offset is passed down by the adal_iic_config using 
		// the ADAL_IIC_CFG_OFFSET parameter.
		// Refer to the ioctl case IIC_W(IIC_IOC_OFFSET):
		// xfr_opts->offset = val;
		// The user should configure the dev_width for their specific
		// slave device before calling the ioctl.
		// If the dev_width = 0, then we set the option to default 2. 
		if (xfr_opts->dev_width == 0)
		{
			xfr_opts->dev_width = 2;
		}
		

		// This is the read command
		if (pmsg->flags & I2C_M_RD) 
		{
			//set_bit(IIC_XFR_RD, &options);
			rc = iic_create_xfr(client, 0, (void*)pmsg->buf,
					   pmsg->len, (1 << IIC_XFR_RD), &xfr,
					   NULL);
		}
		else
		{
			if (num > 1)
			{
				// Multiple messages recieved
				// The first msg contains the offset for 
				// the repeated start.
				// Otherwise it's just a regular write.
				if ( (i == 0) && (msgs[1].flags & I2C_M_RD) )
				{
					   xfr_opts->offset = *current_msg_buf_ptr;
					// Set the offset and don't do the write
					continue;
				}
			} 

			// This is a regular write

			rc = iic_create_xfr(client, 0, (void*)pmsg->buf,
					    pmsg->len, 0, &xfr, NULL);
		}
		if(rc)
		{
			goto exit;
		}
		/* enqueue or start the xfr */
		rc = iic_enq_xfr(xfr);
		if(rc != -EIOCBQUEUED)
		{
			goto error;
		}

		/* wait for xfr to complete */
		iic_wait_xfr(xfr);
		/* set rc appropriately */
		rc = xfr->status;
		kfree(current_msg_buf_ptr);
	}
	/* Data is already in the user buffer at this point.
	 * Cleanup the transfer and return status to the user.
	 */
	
error:
	iic_cleanup_xfr(xfr, NULL);
exit:
	kfree(data_ptrs);
	return rc;
}
long iic_ioctl(struct file *file, unsigned int cmd,
              unsigned long arg)
{
	iic_ffdc_t* ffdc = 0;
	int ret = 0;
	unsigned long val = 0;
	iic_client_t* client;
	iic_eng_t* eng;
	iic_opts_t* opts;
	iic_xfr_opts_t* xfr_opts;
	iic_rec_pol_t* recovery;
	int ioc_nr = _IOC_NR(cmd);
	struct i2c_msg *iic_msg_ptr;
	struct i2c_rdwr_ioctl_data iic_msg_arg;

	IENTER();

	client = (iic_client_t*)file->private_data;
	eng = client->bus->eng;
	
	/* Allow address unlock to occur even if blacklisted or blocked */
	if(ioc_nr == IIC_IOC_ULCK_ADDR)
		goto skip_check;
	if(test_bit(IIC_ENG_BLOCK, &eng->flags))
	{
		IFLDe(1, "IOCTL    eng[%08x] blocked\n", eng->id);
		ret = -ENODEV;
		if(test_bit(IIC_ENG_REMOVED, &eng->flags))
			ret = -ENOLINK;
		goto exit;
	}

skip_check:
	opts = &client->opts;
	xfr_opts = &opts->xfr_opts;
	recovery = &opts->recovery;

	if(down_interruptible(&client->sem))
	{
		IEXIT(-EINTR);
		return -EINTR;
	}

	if((_IOC_TYPE(cmd) != IIC_IOC_MAGIC) ||
	   (_IOC_NR(cmd) > IIC_IOC_MAXNR))
	{
		ret = -ENOTTY;
		goto exit;
	}

	/* strip the magic and size info from the command that we don't care
	 * about.
	 */
	cmd = _IOC(_IOC_DIR(cmd), 0, _IOC_NR(cmd), 0);

	/* Check if no data needs to be transfered for this ioctl */
	if(_IOC_DIR(cmd) == _IOC_NONE)
	{
		switch(_IOC_NR(cmd))
		{
			case IIC_IOC_RESET_FULL:
				/* reset xfr opts to default values */
				memcpy(opts, &iic_dflt_opts, sizeof(*opts));

			case IIC_IOC_RESET_LIGHT:
				/* only allow 1 user requested reset per engine
				 * at a time.
				 */
				IFLDi(2, "RESET    client[%p] bus[%08lx]\n",
						client, client->bus->bus_id);
				if(down_interruptible(&eng->sem))
				{
					ret = -EINTR;
					break;
				}
				ret = iic_reset(client->bus, xfr_opts->timeout, &ffdc);
				up(&eng->sem);

				break;
			case IIC_IOC_REPEATED_IO:
				// The buffer pointer is stored in arg.
				// Try to get the pointer out from the arg and
				// send the request out one by one using the
				// existing I/O method. 
				// Do not write "STOP" until the last I/O request
				// is done.
	            		ret = copy_from_user(&iic_msg_arg,
               		   		(struct i2c_rdwr_ioctl_data __user *)arg, sizeof(iic_msg_arg));
	            		if (ret)
				{
					ret = -EFAULT;
					break;
				}
				/* Put an arbitrary limit on the number of messages that can
                 		* be sent at once */
                		if (iic_msg_arg.nmsgs > I2C_RDRW_IOCTL_MAX_MSGS)
				{
					ret = -EFAULT;
					break;
				}
				iic_msg_ptr = (struct i2c_msg *)
                        	kmalloc(iic_msg_arg.nmsgs * sizeof(struct i2c_msg),
                        		GFP_KERNEL);
				if (iic_msg_ptr == NULL)
				{
					ret = -ENOMEM;
					break;
				}

		        	if (copy_from_user(iic_msg_ptr, iic_msg_arg.msgs,
                                   iic_msg_arg.nmsgs * sizeof(struct i2c_msg))) 				{
                            		kfree(iic_msg_ptr);
                            		ret =  -EFAULT;
					break;
                		}

				// We don't want to convert the data pointer here
				// because the set_iic_xfr will do the conversion 

				ret = iic_repeated_xfer(client, iic_msg_ptr, 
					iic_msg_arg.nmsgs);
				if (ret < 0)
				{
					ret = -EFAULT;
                			kfree(iic_msg_ptr);
					break;
				}
				else
					ret = 0;
                		kfree(iic_msg_ptr);

			break;
			default:
				ret = -EINVAL;
		}
		goto exit;
	}

	/* handle 4 byte args here */
	if(ioc_nr <= IIC_IOC_4_BYTES)
	{
		if((_IOC_DIR(cmd) == _IOC_WRITE) &&
	   	   (ret = get_user(val, (unsigned long*)arg)))
		{
			goto exit;
		}
		switch(cmd)
		{
			case IIC_W(IIC_IOC_SPEED):
				if((val < 1) || (val > 55))
				{
					ret = -EINVAL;
					break;
				}
				ret = eng->ops->set_speed(client->bus, 
							  val);
				break;
			case IIC_R(IIC_IOC_SPEED):
				val = eng->ops->get_speed(client->bus);
				break;
			case IIC_W(IIC_IOC_DEV_ADDR):
				xfr_opts->dev_addr = val;
				xfr_opts->offset = 0;
				break;
			case IIC_R(IIC_IOC_DEV_ADDR):
				val = xfr_opts->dev_addr;
				break;
			case IIC_W(IIC_IOC_DEV_WIDTH):
				xfr_opts->dev_width = val;
				xfr_opts->offset = 0;
				break;
			case IIC_R(IIC_IOC_DEV_WIDTH):
				val = xfr_opts->dev_width;
				break;
			case IIC_W(IIC_IOC_OFFSET):
				xfr_opts->offset = val;
				break;
			case IIC_R(IIC_IOC_OFFSET):
				val = xfr_opts->offset;
				break;
			case IIC_W(IIC_IOC_INC_ADDR):
				xfr_opts->inc_addr = val;
				break;
			case IIC_R(IIC_IOC_INC_ADDR):
				val = xfr_opts->inc_addr;
				break;
			case IIC_W(IIC_IOC_TIMEOUT):
				xfr_opts->timeout = val;
				break;
			case IIC_R(IIC_IOC_TIMEOUT):
				val = xfr_opts->timeout;
				break;
			case IIC_W(IIC_IOC_RDELAY):
				xfr_opts->rdelay = val;
				break;
			case IIC_R(IIC_IOC_RDELAY):
				val = xfr_opts->rdelay;
				break;
			case IIC_W(IIC_IOC_WDELAY):
				xfr_opts->wdelay = val;
				break;
			case IIC_R(IIC_IOC_WDELAY):
				val = xfr_opts->wdelay;
				break;
			case IIC_W(IIC_IOC_RSPLIT):
				xfr_opts->rsplit = val;
				break;
			case IIC_R(IIC_IOC_RSPLIT):
				val = xfr_opts->rsplit;
				break;
			case IIC_W(IIC_IOC_WSPLIT):
				xfr_opts->wsplit = val;
				break;
			case IIC_R(IIC_IOC_WSPLIT):
				val = xfr_opts->wsplit;
				break;
			case IIC_W(IIC_IOC_REDO_POL):
				recovery->redo_pol = val;
				break;
			case IIC_R(IIC_IOC_REDO_POL):
				val = recovery->redo_pol;
				break;
			case IIC_W(IIC_IOC_REDO_DELAY):
				recovery->redo_delay = val;
				break;
			case IIC_R(IIC_IOC_REDO_DELAY):
				val = recovery->redo_delay;
				break;
			case IIC_R(IIC_IOC_BUS_STATE):
				if(down_interruptible(&eng->sem))
				{
					ret = -EINTR;
					break;
				}
				ret = iic_get_bus_state(client->bus, &val,
							xfr_opts->timeout, 
							&ffdc);
				up(&eng->sem);
				break;
			case IIC_W(IIC_IOC_FLAGS):
				if(val & ~(IIC_FORCE_DMA | IIC_NO_DMA | 
							IIC_SPECIAL_RD))
				{
					ret = -EINVAL;
					break;
				}
				xfr_opts->flags = val;
				break;
			case IIC_R(IIC_IOC_FLAGS):
				val = xfr_opts->flags;
				break;
			default:
				ret = -EINVAL;
		}
		if((_IOC_DIR(cmd) == _IOC_READ) && !ret)
		{
			ret = put_user(val, (unsigned long*)arg);
		}
		goto exit;
	}

	/* handle objects larger than 4 bytes here */
	switch(cmd)
	{
		iic_lock_t ulck;
		iic_lck_t *klck;

		case IIC_W(IIC_IOC_LCK_ADDR):
		case IIC_W(IIC_IOC_LCK_ENG):
			if((ret = copy_from_user(&ulck, (void*)arg, 
							sizeof(ulck))))
			{
				ret = -EFAULT;
				break;
			}

			ret = iic_wait_lock(&eng->lck_mgr, ulck.addr,
					    (cmd == IIC_W(IIC_IOC_LCK_ENG))
					    ? ulck.mask
					    : ulck.mask >> 1,
					    client,
					    msecs_to_jiffies( ulck.timeout));
			break;
		case IIC_W(IIC_IOC_ULCK_ADDR):
		case IIC_W(IIC_IOC_ULCK_ENG):
			if((ret = copy_from_user(&ulck, (void*)arg, 
							sizeof(ulck))))
			{
				ret = -EFAULT;
				break;
			}
			spin_lock_irq(&eng->lock);
			klck = iic_find_handle(&eng->lck_mgr, client,
					       ulck.addr,
					       (cmd == IIC_W(IIC_IOC_ULCK_ENG))
					       ? ulck.mask
					       : ulck.mask >> 1);
			if(klck)
			{
				ret = iic_unlock(&eng->lck_mgr, klck);
				if(!ret)
					iic_start_next_xfr(eng);
			}
			spin_unlock_irq(&eng->lock);
			break;
		case IIC_W(IIC_IOC_ALL):
			if((ret = copy_from_user(opts, (void*)arg, 
							sizeof(*opts))))
			{
				ret = -EFAULT;
			}
			break;
		case IIC_R(IIC_IOC_ALL):
			if((ret = copy_to_user((void*)arg, opts, 
							sizeof(*opts))))
			{
				ret = -EFAULT;
			}
			break;
		case IIC_W(IIC_IOC_DISPLAY_REGS):
			eng->ops->display_regs(eng, 0);
			break;
		default:
			ret = -EINVAL;
			goto exit;
	}

exit:
	up(&client->sem);
	IFLDd(5, "IOCTL    client[%p] bus[%08lx] cmd[%08x] ptr[%08lx] val[%08lx]\n",
			client, client->bus->bus_id, cmd, arg, val);
	IEXIT(ret);
	return ret;
}

loff_t iic_llseek(struct file *filp, loff_t off, int whence)
{
	iic_client_t* client;
	iic_xfr_opts_t* xfr_opts;
	loff_t new_pos;

	IENTER();
	client = (iic_client_t*)filp->private_data;
	xfr_opts = &client->opts.xfr_opts;

	if(down_interruptible(&client->sem))
	{
		new_pos = -EINTR;
		goto exit;
	}
	switch(whence)
	{
		case 0: /* SEEK_SET */
			new_pos = off;
			break;
		case 1: /* SEEK_CUR */
			new_pos = xfr_opts->offset + off;
			break;
		case 2: /* SEEK_END, not supported */
		default:
			new_pos = -EINVAL;
	}

	if(new_pos >= 0)
	{
		xfr_opts->offset = new_pos;
	}
	up(&client->sem);
exit:
	IFLDd(2, "client[%p] seek: new_pos=%08lx\n", client, 
		(unsigned long)new_pos);
	IEXIT((int)new_pos);
	return new_pos;
}

static int iic_mmap(struct file* filp, struct vm_area_struct* vma)
{
	int rc = 0;
	iic_client_t *client = (iic_client_t*)filp->private_data;
	iic_eng_t* eng = client->bus->eng;
	// iopa doesn't exist in MCP6 kernel 
	unsigned long phys_base_addr = virt_to_phys((unsigned long *)eng->base);
	IENTER();
	printk("mmap\n");

	vma->vm_flags |= VM_DONTEXPAND | VM_DONTDUMP | VM_IO;
	printk(">>remap_page_range(%08lX, 0, %08lX, %08lX, )\n",
			vma->vm_start, phys_base_addr, vma->vm_end - vma->vm_start);
	rc = remap_pfn_range(vma, vma->vm_start, phys_base_addr,
			vma->vm_end - vma->vm_start,
			vma->vm_page_prot);
	printk("<<remap_page_range = %d\n", rc);
	if(rc){
		return -EINVAL;
	}
	IEXIT(0);
	return 0;
}

int iic_register_eng_ops(iic_eng_ops_t* new_ops, unsigned long type)
{
	iic_eng_type_t* new_type = (iic_eng_type_t*)
		kmalloc(sizeof(iic_eng_type_t), GFP_KERNEL);
	IENTER();
	if(!new_type)
	{
		return -ENOMEM;
	}
	
	new_type->type = type;
	new_type->ops = new_ops;

	/* Add this eng type object to beginning of engine type list*/
	list_add(&new_type->list, &iic_eng_type_list);
	IDBGd(1, "eng type %08lx registered\n", type);
	IEXIT(0);
	return 0;
}
EXPORT_SYMBOL(iic_register_eng_ops);

int iic_unregister_eng_ops(unsigned long type)
{
	iic_eng_type_t *iterator, *found;
	IENTER();
	found = 0;

	list_for_each_entry(iterator, &iic_eng_type_list, list)
	{
		if(iterator->type == type)
		{
			found = iterator;
			break;
		}
	}
	if(found)
	{
		list_del(&found->list);
		kfree(found);
		IDBGd(1, "engine type %08lx unregistered\n", type);
	}
	IEXIT(0);
	return 0;
}
EXPORT_SYMBOL(iic_unregister_eng_ops);

void iic_register_bus(iic_bus_t * new_bus, unsigned long type)
{
        iic_bus_type_t* new_type = (iic_bus_type_t*)
                kmalloc(sizeof(iic_bus_type_t), GFP_KERNEL);

        IENTER();

        new_type->type = type;
        new_type->bus = new_bus;
        list_add(&new_type->list, &iic_bus_type_list);

        IEXIT(0);
}
EXPORT_SYMBOL(iic_register_bus);

void iic_unregister_bus(iic_bus_t *bus, unsigned long type)
{
        iic_bus_type_t *iterator, *temp;

        IENTER();

        list_for_each_entry_safe(iterator, temp, &iic_bus_type_list, list)
        {
                if((iterator->type == type) && (iterator->bus == bus))
                {
                        list_del(&iterator->list);
                        kfree(iterator);
                }
        }
        IEXIT(0);
}
EXPORT_SYMBOL(iic_unregister_bus);

void iic_init_eng(iic_eng_t* eng)
{
	IENTER();
	spin_lock_init(&eng->lock);
	sema_init(&eng->sem, 1);
	INIT_LIST_HEAD(&eng->xfrq);
	eng->cur_xfr = 0;
	iic_lck_mgr_init(&eng->lck_mgr);
	init_waitqueue_head(&eng->waitq);
	INIT_WORK(&eng->work, iic_finish_abort);
	atomic_set(&eng->xfr_num, 0);
	IEXIT(0);
}
EXPORT_SYMBOL(iic_init_eng);

int iic_eng_ops_is_vaild(struct iic_eng_ops *ops)
{
	int found = 0;
	iic_eng_type_t *iterator;

	list_for_each_entry(iterator, &iic_eng_type_list, list)
	{
		if(iterator->ops == ops)
		{
			found = 1;
			break;
		}
	}

	return found;
}
EXPORT_SYMBOL(iic_eng_ops_is_vaild);

struct iic_eng_ops* iic_get_eng_ops(unsigned long type)
{
	iic_eng_type_t *iterator;
	iic_eng_ops_t *found = 0;
	IENTER();

	/* return the eng ops for the given type of engine */
	list_for_each_entry(iterator, &iic_eng_type_list, list)
	{
		if(iterator->type == type)
		{
			found = iterator->ops;
			break;
		}
	}
	IEXIT((int)found);
	return found;
}
EXPORT_SYMBOL(iic_get_eng_ops);

/* called when an ffdc q for a bus is unlocked */
void iic_ffdc_q_unlocked(int scope, void* data)
{
	iic_eng_t* eng = (iic_eng_t*)data;
	unsigned long flags;
	if(eng)
	{
		spin_lock_irqsave(&eng->lock, flags);
		iic_start_next_xfr(eng);
		spin_unlock_irqrestore(&eng->lock, flags);
	}
}

/* Register this bus's minor number with the kernel and add
 * it to the iic class in sysfs so that a hotplug event is
 * sent to udev.  The sysfs name needs to be unique because
 * all entries are placed in the same directory.  udev
 * will take care of creating the correct /dev name.
 */
#define IIC_BUS_MAX_FFDC 4
iic_bus_t*  iic_create_bus(struct class* classp, iic_eng_t* eng,
			   dev_t devnum, char* name, unsigned char port,
			   unsigned long bus_id)
{
	int rc = 0;
	iic_bus_t* bus = 0;

	IENTER();

	if(!eng)
	{
		goto exit;
	}
	bus = (iic_bus_t*)kmalloc(sizeof(iic_bus_t), GFP_KERNEL);
	if(!bus)
	{
		goto exit;
	}
	memset(bus, 0, sizeof(iic_bus_t));
	bus->port = port;
	bus->bus_id = bus_id;
	bus->eng = eng;
	bus->devnum = devnum;
	bus->i2c_hz = 400000;
	cdev_init(&bus->cdev, &iic_fops); // ref count = 1
	/* since cdev is embedded in our bus structure, override the cdev
	 * cleanup function with our own so that the bus object doesn't get
	 * freed until the cdev ref count goes to zero.
	 */
	if(!cdev_dflt_type)
	{
		cdev_dflt_type = bus->cdev.kobj.ktype;
		memcpy(&iic_bus_type, cdev_dflt_type, sizeof(iic_bus_type));
		iic_bus_type.release = iic_bus_release;
	}
	bus->cdev.kobj.ktype = &iic_bus_type;
	kobject_set_name(&bus->cdev.kobj, name);
	rc = cdev_add(&bus->cdev, devnum, 1);
	if(rc)
	{
		IFLDe(1, "cdev_add failed for bus %08lx\n", bus->bus_id);
		kobject_put(&bus->cdev.kobj);
		goto exit_cdev_add;
	}

	bus->class_dev = device_create(classp, NULL,
			devnum,
			bus->eng->dev,
			(const char *)"%s",
			name);
	if(!bus->class_dev)
	{
		IFLDe(1, "device create failed, %08lx\n", bus->bus_id);
		goto exit_class_add;
	}

	IFLDi(1, "bus[%08lx] created\n", bus->bus_id);
	goto exit;

	device_destroy(classp, bus->devnum);
exit_class_add:
	cdev_del(&bus->cdev);
exit_cdev_add:
	bus = 0;
exit:
	IEXIT((int)bus);
	return bus;
}
EXPORT_SYMBOL(iic_create_bus);

void iic_delete_bus(struct class* classp, iic_bus_t* bus)
{
	IENTER();

	if(!bus)
	{
		goto exit;
	}
	IFLDi(1, "cleanup bus[%08lx]\n", bus->bus_id);
	cdev_del(&bus->cdev);
exit:
	IEXIT(0);
	return;
}
EXPORT_SYMBOL(iic_delete_bus);

static int __init iic_init(void)
{
	int rc = 0;

	IENTER();
	printk("IIC: base support loaded  ver. %s\n", iic_mstr_version);
	IEXIT(rc);
	return rc;
}

static void __exit iic_exit(void)
{
	IENTER();
	printk("IIC: base support unloaded.\n");
	IEXIT(0);
}

module_init(iic_init);
module_exit(iic_exit);
MODULE_AUTHOR("Eddie James <eajames@us.ibm.com>");
MODULE_DESCRIPTION("Base IIC Driver");
MODULE_LICENSE("GPL");
