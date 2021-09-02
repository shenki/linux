// SPDX-License-Identifier: GPL-2.0
/* Copyright (C) 2019 Hewlett-Packard Development Company, L.P.
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/errno.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/completion.h>
#include <linux/interrupt.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>

#define GXP_MAX_I2C_ENGINE 10
static const char * const gxp_i2c_name[] = {
	"gxp-i2c0", "gxp-i2c1", "gxp-i2c2", "gxp-i2c3",
	"gxp-i2c4", "gxp-i2c5", "gxp-i2c6", "gxp-i2c7",
	"gxp-i2c8",	"gxp-i2c9" };

/* Default value */
#define GXP_I2C_BIT_RATE	100000	/* 100kHz */

/* GXP I2C Global interrupt status/enable register*/
#define GXP_I2CINTSTAT		0x00
#define GXP_I2CINTEN		0x04

/* GXP I2C registers */
#define GXP_I2CSTAT		0x00
#define MASK_STOP_EVENT		0x20
#define MASK_ACK		0x08
#define MASK_RW			0x04
#define GXP_I2CEVTERR		0x01
#define MASK_SLAVE_CMD_EVENT	0x01
#define MASK_SLAVE_DATA_EVENT	0x02
#define MASK_MASTER_EVENT	0x10
#define GXP_I2CSNPDAT		0x02
#define GXP_I2CMCMD		0x04
#define GXP_I2CSCMD		0x06
#define GXP_I2CSNPAA		0x09
#define GXP_I2CADVFEAT		0x0A
#define GXP_I2COWNADR		0x0B
#define GXP_I2CFREQDIV		0x0C
#define GXP_I2CFLTFAIR		0x0D
#define GXP_I2CTMOEDG		0x0E
#define GXP_I2CCYCTIM		0x0F

static bool i2c_global_init_done;

enum {
	GXP_I2C_IDLE = 0,
	GXP_I2C_ADDR_PHASE,
	GXP_I2C_RDATA_PHASE,
	GXP_I2C_WDATA_PHASE,
	GXP_I2C_ADDR_NACK,
	GXP_I2C_DATA_NACK,
	GXP_I2C_ERROR,
	GXP_I2C_COMP
};

struct gxp_i2c_drvdata {
	struct device *dev;
	void __iomem	*base;
	uint32_t bus_frequency;
	int engine;
	int irq;
	struct completion completion;
	struct i2c_adapter	adapter;
	struct i2c_msg *curr_msg;
	int msgs_remaining;
	int msgs_num;
	uint8_t *buf;
	size_t buf_remaining;
	unsigned char		state;
	struct i2c_client *slave;
	unsigned char stopped;
};

static struct regmap *i2cg_map;

static void gxp_i2c_start(struct gxp_i2c_drvdata *drvdata)
{
	void __iomem *base = drvdata->base;
	uint16_t value;

	drvdata->buf = drvdata->curr_msg->buf;
	drvdata->buf_remaining = drvdata->curr_msg->len;

	value = drvdata->curr_msg->addr << 9; //addr in struct i2c_msg is 7 bits

	if (drvdata->curr_msg->flags & I2C_M_RD)
		value |= 0x05; // read
	else
		value |= 0x01;	//write
	/* dev_err(drvdata->dev,
	 *  "gxp_i2c_start:msg[%d/%d] addr=0x%02x flags=0x%02x len=%d\n",
	 *   drvdata->msgs_num - drvdata->msgs_remaining, drvdata->msgs_num,
	 *   drvdata->curr_msg->addr, drvdata->curr_msg->flags,
	 *   drvdata->curr_msg->len);
	 */

	drvdata->state = GXP_I2C_ADDR_PHASE;
	writew(value, base + GXP_I2CMCMD);
}

static int gxp_i2c_master_xfer(struct i2c_adapter *adapter,
		struct i2c_msg *msgs, int num)
{
	int ret;
	struct gxp_i2c_drvdata *drvdata = i2c_get_adapdata(adapter);
	unsigned long time_left;


	drvdata->msgs_remaining = num;
	drvdata->curr_msg = msgs;
	drvdata->msgs_num = num;
	reinit_completion(&drvdata->completion);

	gxp_i2c_start(drvdata);

	time_left = wait_for_completion_timeout(&drvdata->completion,
						adapter->timeout);
	ret = num - drvdata->msgs_remaining;
	if (time_left == 0) {
		switch (drvdata->state) {
		case GXP_I2C_WDATA_PHASE:
			dev_err(drvdata->dev,
			"gxp_i2c_start:write Data phase timeout at msg[%d]\n",
			ret);
			break;
		case GXP_I2C_RDATA_PHASE:
			dev_err(drvdata->dev,
			"gxp_i2c_start:read Data phase timeout at msg[%d]\n",
			ret);
			break;
		case GXP_I2C_ADDR_PHASE:
			dev_err(drvdata->dev,
			"gxp_i2c_start:Addr phase timeout\n");
			break;
		default:
			dev_err(drvdata->dev,
			"gxp_i2c_start:i2c transfer timeout state=%d\n",
			drvdata->state);
			break;
		}
		return -ETIMEDOUT;
	}

	if (drvdata->state == GXP_I2C_ADDR_NACK) {
		dev_dbg(drvdata->dev,
				"gxp_i2c_start:No ACK for address phase\n");
		return -EIO;
	} else if (drvdata->state == GXP_I2C_DATA_NACK) {
		dev_dbg(drvdata->dev, "gxp_i2c_start:No ACK for data phase\n");
		return -EIO;
	}

	return ret;
}

static u32 gxp_i2c_func(struct i2c_adapter *adap)
{
#if IS_ENABLED(CONFIG_I2C_SLAVE)
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL | I2C_FUNC_SLAVE;
#else
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
#endif
}

#if IS_ENABLED(CONFIG_I2C_SLAVE)
static int gxp_i2c_reg_slave(struct i2c_client *slave)
{
	struct gxp_i2c_drvdata *drvdata = i2c_get_adapdata(slave->adapter);
	void __iomem *base = drvdata->base;

	pr_info("[%s] I2C engine%d addr:0x%02x\n", __func__, drvdata->engine, slave->addr);
	if (drvdata->slave)
		return -EBUSY;

	if (slave->flags & I2C_CLIENT_TEN)
		return -EAFNOSUPPORT;

	drvdata->slave = slave;

	writeb(slave->addr<<1, base + GXP_I2COWNADR);
	writeb(0x69, base + GXP_I2CSCMD);

	return 0;
}

static int gxp_i2c_unreg_slave(struct i2c_client *slave)
{
	struct gxp_i2c_drvdata *drvdata = i2c_get_adapdata(slave->adapter);
	void __iomem *base = drvdata->base;

	pr_info("[%s] I2C engine%d\n", __func__, drvdata->engine);
	WARN_ON(!drvdata->slave);

	writeb(0x00, base + GXP_I2COWNADR);
	writeb(0xF0, base + GXP_I2CSCMD);

	drvdata->slave = NULL;

	return 0;
}
#endif

static const struct i2c_algorithm gxp_i2c_algo = {
	.master_xfer   = gxp_i2c_master_xfer,
	.functionality = gxp_i2c_func,
#if IS_ENABLED(CONFIG_I2C_SLAVE)
	.reg_slave     = gxp_i2c_reg_slave,
	.unreg_slave   = gxp_i2c_unreg_slave,
#endif
};

static void gxp_i2c_stop(struct gxp_i2c_drvdata *drvdata)
{
	void __iomem *base = drvdata->base;

	writeb(0x82, base + GXP_I2CMCMD); // clear event, send stop

	complete(&drvdata->completion);
}

static void gxp_i2c_restart(struct gxp_i2c_drvdata *drvdata)
{
	void __iomem *base = drvdata->base;
	uint16_t value;

	drvdata->buf = drvdata->curr_msg->buf;
	drvdata->buf_remaining = drvdata->curr_msg->len;

	value = drvdata->curr_msg->addr << 9;

	if (drvdata->curr_msg->flags & I2C_M_RD)
		value |= 0x85; // read and clear master event
	else
		value |= 0x81; // write and clear master event
	drvdata->state = GXP_I2C_ADDR_PHASE;
	/* dev_err(drvdata->dev,
	 * "gxp_i2c_restart:msg[%d/%d] addr=0x%02x flags=0x%02x len=%d\n",
	 *  drvdata->msgs_num - drvdata->msgs_remaining, drvdata->msgs_num,
	 *  drvdata->curr_msg->addr, drvdata->curr_msg->flags,
	 *  drvdata->curr_msg->len);
	 */

	writew(value, base + GXP_I2CMCMD);
}

static void gxp_i2c_chk_addr_ack(struct gxp_i2c_drvdata *drvdata)
{
	void __iomem *base = drvdata->base;
	uint16_t value;

	//if ack
	value = readb(base + GXP_I2CSTAT);
	if (!(value & MASK_ACK)) {
		//got no ack, stop
		drvdata->state = GXP_I2C_ADDR_NACK;
		gxp_i2c_stop(drvdata);
		return;
	}

	if (drvdata->curr_msg->flags & I2C_M_RD) {
		//start to read data from slave
		if (drvdata->buf_remaining == 0) {
			//no more data to read, stop
			drvdata->msgs_remaining--;
			drvdata->state = GXP_I2C_COMP;
			gxp_i2c_stop(drvdata);
			return;
		}
		drvdata->state = GXP_I2C_RDATA_PHASE;

		if (drvdata->buf_remaining == 1) {
			//the last data, don't ack
			writeb(0x84, base + GXP_I2CMCMD);
		} else {
			//read data and ack it
			writeb(0x8C, base + GXP_I2CMCMD);
		}
	} else {
		//start to write first data to slave
		if (drvdata->buf_remaining == 0) {
			//no more data to write, stop
			drvdata->msgs_remaining--;
			drvdata->state = GXP_I2C_COMP;
			gxp_i2c_stop(drvdata);
			return;
		}
		value = *drvdata->buf;
		value = value << 8;
		value |= 0x80; //clear master event
		drvdata->buf++;
		drvdata->buf_remaining--;
		drvdata->state = GXP_I2C_WDATA_PHASE;
		writew(value, base + GXP_I2CMCMD);
	}
}

static void gxp_i2c_ack_data(struct gxp_i2c_drvdata *drvdata)
{
	void __iomem *base = drvdata->base;
	uint8_t value;

	// store the data returned
	value = readb(base + GXP_I2CSNPDAT);
	*drvdata->buf = value;
	drvdata->buf++;
	drvdata->buf_remaining--;

	if (drvdata->buf_remaining == 0) {
		//no more data, this msg is completed.
		drvdata->msgs_remaining--;

		//if this msg is last one?
		if (drvdata->msgs_remaining == 0) {
			//no more msg, stop
			drvdata->state = GXP_I2C_COMP;
			gxp_i2c_stop(drvdata);
			return;
		}
		//move to next msg and start transfer
		drvdata->curr_msg++;
		gxp_i2c_restart(drvdata);
		return;
	}

	//ack the slave to make it send next byte
	drvdata->state = GXP_I2C_RDATA_PHASE;

	if (drvdata->buf_remaining == 1) {
		//the last data, don't ack
		writeb(0x84, base + GXP_I2CMCMD);
	} else {
		//read data and ack it
		writeb(0x8C, base + GXP_I2CMCMD);
	}
}

static void gxp_i2c_chk_data_ack(struct gxp_i2c_drvdata *drvdata)
{
	void __iomem *base = drvdata->base;
	uint16_t value;

	//if ack
	value = readb(base + GXP_I2CSTAT);
	if (!(value & MASK_ACK)) {
		//got no ack, stop
		drvdata->state = GXP_I2C_DATA_NACK;
		gxp_i2c_stop(drvdata);
		return;
	}

	//got ack
	//if there is more data to write?
	if (drvdata->buf_remaining == 0) {
		//no more data, this msg is completed.
		drvdata->msgs_remaining--;

		//if this msg is last one?
		if (drvdata->msgs_remaining == 0) {
			//no more msg, stop
			drvdata->state = GXP_I2C_COMP;
			gxp_i2c_stop(drvdata);
			return;
		}
		//move to next msg and start transfer
		drvdata->curr_msg++;
		gxp_i2c_restart(drvdata);
		return;
	}

	//write data to slave
	value = *drvdata->buf;
	value = value << 8;
	value |= 0x80; //clear master event
	drvdata->buf++;
	drvdata->buf_remaining--;
	drvdata->state = GXP_I2C_WDATA_PHASE;
	writew(value, base + GXP_I2CMCMD);
}

#if IS_ENABLED(CONFIG_I2C_SLAVE)
static bool gxp_i2c_slave_irq_handler(struct gxp_i2c_drvdata *drvdata)
{
	void __iomem *base = drvdata->base;
	uint16_t value;
	uint8_t buf;
	int ret;

	value = readb(base + GXP_I2CEVTERR);

	// Received start or stop event
	if (value & MASK_SLAVE_CMD_EVENT) {
		value = readb(base + GXP_I2CSTAT);
		// Master sent stop
		if (value & MASK_STOP_EVENT) {
			if (drvdata->stopped == 0)
				i2c_slave_event(drvdata->slave, I2C_SLAVE_STOP, &buf);
			writeb(0x69, base + GXP_I2CSCMD);
			drvdata->stopped = 1;
		} else {
			// Master sent start
			// Master wants to read from us
			drvdata->stopped = 0;
			if (value & MASK_RW) {
				i2c_slave_event(drvdata->slave,
						I2C_SLAVE_READ_REQUESTED, &buf);
				value = buf << 8 | 0x61;
				writew(value, base + GXP_I2CSCMD);
			} else {
				// Master wants to write to us
				ret = i2c_slave_event(drvdata->slave,
					I2C_SLAVE_WRITE_REQUESTED, &buf);
				if (!ret)
					writeb(0x69, base + GXP_I2CSCMD);	//ack next byte from master
				else
					writeb(0x61, base + GXP_I2CSCMD); //nack next byte from master
			}
		}
	}
	// Received data event
	else if (value & MASK_SLAVE_DATA_EVENT) {
		value = readb(base + GXP_I2CSTAT);
		// Master wants to read from us
		if (value & MASK_RW) {
			// Master wants another byte
			if (value & MASK_ACK) {
				i2c_slave_event(drvdata->slave,
						I2C_SLAVE_READ_PROCESSED, &buf);
				value = buf << 8 | 0x61;
				writew(value, base + GXP_I2CSCMD);
			} else {
				// No more bytes needed
				writew(0x69, base + GXP_I2CSCMD);
			}
		} else {
			// Master wants to write to us
			value = readb(base + GXP_I2CSNPDAT);
			buf = (uint8_t)value;
			ret = i2c_slave_event(drvdata->slave,
					I2C_SLAVE_WRITE_RECEIVED, &buf);
			if (!ret)
				writeb(0x69, base + GXP_I2CSCMD); //ack next byte from master
			else
				writeb(0x61, base + GXP_I2CSCMD); //nack next byte from master
		}
	} else {
		return false;
	}

	return true;
}
#endif

static irqreturn_t gxp_i2c_irq_handler(int irq, void *_drvdata)
{
	struct gxp_i2c_drvdata *drvdata = (struct gxp_i2c_drvdata *)_drvdata;
	uint32_t value;
	void __iomem *base = drvdata->base;

	regmap_read(i2cg_map, GXP_I2CINTSTAT, &value);
	if (!(value & (1 << drvdata->engine)))
		return IRQ_NONE;

	value = readb(base + GXP_I2CEVTERR);

	// Error
	if (value & ~(MASK_MASTER_EVENT | MASK_SLAVE_CMD_EVENT |
				MASK_SLAVE_DATA_EVENT)) {
		pr_alert("[%s] I2C Error, GXP_I2CEVTERR = 0x%x\n", __func__,
			value);
		writeb(0x00, base + GXP_I2CEVTERR); //clear all event
		drvdata->state = GXP_I2C_ERROR;
		gxp_i2c_stop(drvdata);
		return IRQ_HANDLED;
	}

#if IS_ENABLED(CONFIG_I2C_SLAVE)
	// Slave mode
	if (value & (MASK_SLAVE_CMD_EVENT | MASK_SLAVE_DATA_EVENT)) {
		if (gxp_i2c_slave_irq_handler(drvdata))
			return IRQ_HANDLED;
		pr_alert("[%s] I2C Error, GXP_I2CEVTERR = 0x%x\n",
			__func__, value);
		return IRQ_NONE;
	}
#endif

	// Master mode
	switch (drvdata->state) {
	case GXP_I2C_ADDR_PHASE:
		gxp_i2c_chk_addr_ack(drvdata);
		break;

	case GXP_I2C_RDATA_PHASE:
		gxp_i2c_ack_data(drvdata);
		break;

	case GXP_I2C_WDATA_PHASE:
		gxp_i2c_chk_data_ack(drvdata);
		break;
	}

	return IRQ_HANDLED;
}

static void gxp_i2c_init(struct gxp_i2c_drvdata *drvdata)
{
	void __iomem *base = drvdata->base;

	drvdata->state = GXP_I2C_IDLE;
	writeb(2000000/drvdata->bus_frequency, base + GXP_I2CFREQDIV);
	writeb(0x32, base + GXP_I2CFLTFAIR); //filter count=3,fairness count=2
	writeb(0x0a, base + GXP_I2CTMOEDG);
	writeb(0x00, base + GXP_I2CCYCTIM); //disable maximum cycle timeout
	writeb(0x00, base + GXP_I2CSNPAA); //disable snoop alert address
	writeb(0x00, base + GXP_I2CADVFEAT); //disable advanced feature
	writeb(0xF0, base + GXP_I2CSCMD);	//clear and mask slave event
	writeb(0x80, base + GXP_I2CMCMD);	//clear master event
	writeb(0x00, base + GXP_I2CEVTERR);	//clear error event
	writeb(0x00, base + GXP_I2COWNADR);	//slave not used
}

static int gxp_i2c_probe(struct platform_device *pdev)
{
	struct gxp_i2c_drvdata *drvdata;
	int rc;
	struct resource *res;
	struct i2c_adapter *adapter;

	if (!i2c_global_init_done) {
		pr_info("[%s] I2c global init\n", __func__);
		i2cg_map = syscon_regmap_lookup_by_phandle(pdev->dev.of_node,
				"i2cg-handle");
		if (IS_ERR(i2cg_map)) {
			dev_err(&pdev->dev, "failed to map i2cg_handle\n");
			return -ENODEV;
		}
		//disable interrupt
		regmap_update_bits(i2cg_map, GXP_I2CINTEN, 0x00000FFF, 0);
		i2c_global_init_done = true;
	}

	drvdata = devm_kzalloc(&pdev->dev, sizeof(struct gxp_i2c_drvdata),
			GFP_KERNEL);
	if (!drvdata)
		return -ENOMEM;

	platform_set_drvdata(pdev, drvdata);
	drvdata->dev = &pdev->dev;
	init_completion(&drvdata->completion);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	drvdata->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(drvdata->base))
		return PTR_ERR(drvdata->base);

	drvdata->engine = (res->start & 0xf00) >> 8;
	pr_info("%s: i2c engine%d\n", __func__, drvdata->engine);
	if (drvdata->engine >= GXP_MAX_I2C_ENGINE) {
		dev_err(&pdev->dev, "i2c engine% is unsupported\n",
				drvdata->engine);
		return -EINVAL;
	}

	rc = platform_get_irq(pdev, 0);
	if (rc < 0) {
		dev_err(&pdev->dev, "unable to obtain IRQ number\n");
		return rc;
	}

	drvdata->irq = rc;
	pr_info("[%s] i2c engine%d, rq = %d\n", __func__, drvdata->engine,
			drvdata->irq);

	rc = devm_request_irq(&pdev->dev, drvdata->irq, gxp_i2c_irq_handler,
			IRQF_SHARED, gxp_i2c_name[drvdata->engine], drvdata);
	if (rc < 0) {
		dev_err(&pdev->dev, "irq request failed\n");
		return rc;
	}

	rc = of_property_read_u32(pdev->dev.of_node,
				   "bus-frequency", &drvdata->bus_frequency);
	if (rc < 0) {
		dev_info(&pdev->dev,
		"Could not read bus-frequency property, use default frequency:100000\n");
		drvdata->bus_frequency = GXP_I2C_BIT_RATE;
	}

	gxp_i2c_init(drvdata);

	//enable interrupt
	regmap_update_bits(i2cg_map, GXP_I2CINTEN, BIT(drvdata->engine),
			BIT(drvdata->engine));

	adapter = &drvdata->adapter;
	i2c_set_adapdata(adapter, drvdata);

	adapter->owner = THIS_MODULE;
	adapter->class = I2C_CLASS_DEPRECATED;
	strlcpy(adapter->name, "HPE GXP I2C adapter", sizeof(adapter->name));
	adapter->algo = &gxp_i2c_algo;
	adapter->dev.parent = &pdev->dev;
	adapter->dev.of_node = pdev->dev.of_node;

	rc = i2c_add_adapter(adapter);
	if (rc)
		dev_err(&pdev->dev, "i2c add adapter failed\n");

	return rc;
}

static int gxp_i2c_remove(struct platform_device *pdev)
{
	struct gxp_i2c_drvdata *drvdata = platform_get_drvdata(pdev);

	pr_info("[%s] drvdata engine %d\n", __func__, drvdata->engine);
	i2c_del_adapter(&drvdata->adapter);

	return 0;
}

static const struct of_device_id gxp_i2c_of_match[] = {
	{ .compatible = "hpe,gxp-i2c" },
	{},
};
MODULE_DEVICE_TABLE(of, gxp_i2c_of_match);

static struct platform_driver gxp_i2c_driver = {
	.probe	= gxp_i2c_probe,
	.remove = gxp_i2c_remove,
	.driver = {
		.name	= "gxp-i2c",
		.of_match_table = gxp_i2c_of_match,
	},
};
module_platform_driver(gxp_i2c_driver);

MODULE_AUTHOR("Gilbert Chen <gilbert.chen@hpe.com>");
MODULE_DESCRIPTION("HPE GXP I2C bus driver");

