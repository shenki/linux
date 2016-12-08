/*
 * Copyright 2016 IBM Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/errno.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/spinlock.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/miscdevice.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/mbox-host.h>

#define DEVICE_NAME	"aspeed-mbox"

#define MBOX_NUM_REGS 16
#define MBOX_NUM_DATA_REGS 14

#define MBOX_DATA_0 0x00
#define MBOX_STATUS_0 0x40
#define MBOX_STATUS_1 0x44
#define MBOX_BMC_CTRL 0x48
#define   MBOX_CTRL_RECV BIT(7)
#define   MBOX_CTRL_MASK BIT(1)
#define   MBOX_CTRL_SEND BIT(0)
#define MBOX_HOST_CTRL 0x4c
#define MBOX_INTERRUPT_0 0x50
#define MBOX_INTERRUPT_1 0x54

struct mbox {
	struct miscdevice	miscdev;
	void __iomem		*base;
	int			irq;
	wait_queue_head_t	queue;
	struct timer_list	poll_timer;
};

static atomic_t mbox_open_count = ATOMIC_INIT(0);

static u8 mbox_inb(struct mbox *mbox, int reg)
{
	return ioread8(mbox->base + reg);
}

static void mbox_outb(struct mbox *mbox, u8 data, int reg)
{
	iowrite8(data, mbox->base + reg);
}

static struct mbox *file_mbox(struct file *file)
{
	return container_of(file->private_data, struct mbox, miscdev);
}

static int mbox_open(struct inode *inode, struct file *file)
{
	struct mbox *mbox = file_mbox(file);

	if (atomic_inc_return(&mbox_open_count) == 1) {
		/*
		 * Clear the interrupt status bit if it was left on and unmask
		 * interrupts.
		 * MBOX_CTRL_RECV bit is W1C, this also unmasks in 1 step
		 */
		mbox_outb(mbox, MBOX_CTRL_RECV, MBOX_BMC_CTRL);
		return 0;
	}

	atomic_dec(&mbox_open_count);
	return -EBUSY;
}

static ssize_t mbox_read(struct file *file, char __user *buf,
				size_t count, loff_t *ppos)
{
	struct mbox *mbox = file_mbox(file);
	char __user *p = buf;
	int i;

	if (!access_ok(VERIFY_WRITE, buf, count))
		return -EFAULT;

	WARN_ON(*ppos);

	if (wait_event_interruptible(mbox->queue,
		mbox_inb(mbox, MBOX_BMC_CTRL) & MBOX_CTRL_RECV))
		return -ERESTARTSYS;

	for (i = 0; i < MBOX_NUM_DATA_REGS; i++)
		if (__put_user(mbox_inb(mbox, MBOX_DATA_0 + (i * 4)), p++))
			return -EFAULT;

	/* MBOX_CTRL_RECV bit is W1C, this also unmasks in 1 step */
	mbox_outb(mbox, MBOX_CTRL_RECV, MBOX_BMC_CTRL);
	return p - buf;
}

static ssize_t mbox_write(struct file *file, const char __user *buf,
				size_t count, loff_t *ppos)
{
	struct mbox *mbox = file_mbox(file);
	const char __user *p = buf;
	char c;
	int i;

	if (!access_ok(VERIFY_READ, buf, count))
		return -EFAULT;

	WARN_ON(*ppos);

	for (i = 0; i < MBOX_NUM_DATA_REGS; i++) {
		if (__get_user(c, p))
			return -EFAULT;

		mbox_outb(mbox, c, MBOX_DATA_0 + (i * 4));
		p++;
	}

	mbox_outb(mbox, MBOX_CTRL_SEND, MBOX_BMC_CTRL);

	return p - buf;
}

static long mbox_ioctl(struct file *file, unsigned int cmd,
		unsigned long param)
{
	struct mbox *mbox = file_mbox(file);

	switch (cmd) {
	case ASPEED_MBOX_IOCTL_ATN:
		mbox_outb(mbox, param, MBOX_DATA_0 + 15);
		return 0;
	}

	return -EINVAL;
}

static unsigned int mbox_poll(struct file *file, poll_table *wait)
{
	struct mbox *mbox = file_mbox(file);
	unsigned int mask = 0;

	poll_wait(file, &mbox->queue, wait);

	if (mbox_inb(mbox, MBOX_BMC_CTRL) & MBOX_CTRL_RECV)
		mask |= POLLIN;

	return mask;
}

static const struct file_operations mbox_fops = {
	.owner		= THIS_MODULE,
	.open		= mbox_open,
	.read		= mbox_read,
	.write		= mbox_write,
	.poll		= mbox_poll,
	.unlocked_ioctl	= mbox_ioctl,
};

static void poll_timer(unsigned long data)
{
	struct mbox *mbox = (void *)data;

	mbox->poll_timer.expires += msecs_to_jiffies(500);
	wake_up(&mbox->queue);
	add_timer(&mbox->poll_timer);
}

static irqreturn_t mbox_irq(int irq, void *arg)
{
	struct mbox *mbox = arg;

	if (!(mbox_inb(mbox, MBOX_BMC_CTRL) & MBOX_CTRL_RECV))
		return IRQ_NONE;

	/*
	 * Leave the status bit set so that we know the data is for us,
	 * clear it once it has been read.
	 */

	/* Mask it off, we'll clear it when we the data gets read */
	mbox_outb(mbox, MBOX_CTRL_MASK, MBOX_BMC_CTRL);

	wake_up(&mbox->queue);
	return IRQ_HANDLED;
}

static int mbox_config_irq(struct mbox *mbox,
		struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	int rc;

	mbox->irq = irq_of_parse_and_map(dev->of_node, 0);
	if (!mbox->irq)
		return -ENODEV;

	rc = devm_request_irq(dev, mbox->irq, mbox_irq, IRQF_SHARED,
			DEVICE_NAME, mbox);
	if (rc < 0) {
		dev_warn(dev, "Unable to request IRQ %d\n", mbox->irq);
		mbox->irq = 0;
		return rc;
	}

	/*
	 * Disable all register based interrupts, we'll have to change
	 * this, protocol seemingly will require regs 0 and 15
	 */
	mbox_outb(mbox, 0x00, MBOX_INTERRUPT_0); /* regs 0 - 7 */
	mbox_outb(mbox, 0x00, MBOX_INTERRUPT_1); /* regs 8 - 15 */

	/* W1C */
	mbox_outb(mbox, 0xff, MBOX_STATUS_0);
	mbox_outb(mbox, 0xff, MBOX_STATUS_1);

	mbox_outb(mbox, MBOX_CTRL_RECV, MBOX_BMC_CTRL);
	return 0;
}

static int mbox_probe(struct platform_device *pdev)
{
	struct mbox *mbox;
	struct device *dev;
	struct resource *res;
	int rc;

	if (!pdev || !pdev->dev.of_node)
		return -ENODEV;

	dev = &pdev->dev;
	dev_info(dev, "Found mbox host device\n");

	mbox = devm_kzalloc(dev, sizeof(*mbox), GFP_KERNEL);
	if (!mbox)
		return -ENOMEM;

	dev_set_drvdata(&pdev->dev, mbox);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(dev, "Unable to find resources\n");
		rc = -ENXIO;
		goto out;
	}

	mbox->base = devm_ioremap_resource(&pdev->dev, res);
	if (!mbox->base) {
		rc = -ENOMEM;
		goto out;
	}
	init_waitqueue_head(&mbox->queue);

	mbox->miscdev.minor = MISC_DYNAMIC_MINOR;
	mbox->miscdev.name = DEVICE_NAME;
	mbox->miscdev.fops = &mbox_fops;
	mbox->miscdev.parent = dev;
	rc = misc_register(&mbox->miscdev);
	if (rc) {
		dev_err(dev, "Unable to register device\n");
		goto out;
	}

	mbox_config_irq(mbox, pdev);

	if (mbox->irq) {
		dev_info(dev, "Using IRQ %d\n", mbox->irq);
	} else {
		dev_info(dev, "No IRQ; using timer\n");
		setup_timer(&mbox->poll_timer, poll_timer,
				(unsigned long)mbox);
		mbox->poll_timer.expires = jiffies + msecs_to_jiffies(10);
		add_timer(&mbox->poll_timer);
	}

out:
	return rc;

}

static int mbox_remove(struct platform_device *pdev)
{
	struct mbox *mbox = dev_get_drvdata(&pdev->dev);

	misc_deregister(&mbox->miscdev);
	if (!mbox->irq)
		del_timer_sync(&mbox->poll_timer);
	mbox = NULL;

	return 0;
}

static const struct of_device_id mbox_match[] = {
	{ .compatible = "aspeed,ast2400-mbox" },
	{ },
};

static struct platform_driver mbox_driver = {
	.driver = {
		.name		= DEVICE_NAME,
		.of_match_table = mbox_match,
	},
	.probe = mbox_probe,
	.remove = mbox_remove,
};

module_platform_driver(mbox_driver);

MODULE_DEVICE_TABLE(of, mbox_match);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Cyril Bur <cyrilbur@gmail.com>");
MODULE_DESCRIPTION("Linux device interface to the Aspeed MBOX registers");
