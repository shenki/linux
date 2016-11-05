/*
 *  I2C adapter for the ASPEED I2C bus.
 *
 *  Copyright (C) 2012-2020  ASPEED Technology Inc.
 *  Copyright 2016 IBM Corporation
 *  Copyright 2016 Google, Inc.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/completion.h>
#include <linux/slab.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/clk.h>

/* I2C Register */
#define ASPEED_I2C_FUN_CTRL_REG				0x00
#define ASPEED_I2C_AC_TIMING_REG1			0x04
#define ASPEED_I2C_AC_TIMING_REG2			0x08
#define ASPEED_I2C_INTR_CTRL_REG			0x0c
#define ASPEED_I2C_INTR_STS_REG				0x10
#define ASPEED_I2C_CMD_REG				0x14
#define ASPEED_I2C_DEV_ADDR_REG				0x18
#define ASPEED_I2C_BYTE_BUF_REG				0x20
#define ASPEED_I2C_OFFSET_START				0x40
#define ASPEED_I2C_OFFSET_INCREMENT			0x40

#define ASPEED_I2C_NUM_BUS 14

/* Global Register Definition */
/* 0x00 : I2C Interrupt Status Register  */
/* 0x08 : I2C Interrupt Target Assignment  */

/* Device Register Definition */
/* 0x00 : I2CD Function Control Register  */
#define ASPEED_I2CD_MULTI_MASTER_DIS			BIT(15)
#define ASPEED_I2CD_SDA_DRIVE_1T_EN			BIT(8)
#define ASPEED_I2CD_M_SDA_DRIVE_1T_EN			BIT(7)
#define ASPEED_I2CD_M_HIGH_SPEED_EN			BIT(6)
#define ASPEED_I2CD_SLAVE_EN				BIT(1)
#define ASPEED_I2CD_MASTER_EN				BIT(0)

/* 0x08 : I2CD Clock and AC Timing Control Register #2 */
#define ASPEED_NO_TIMEOUT_CTRL				0


/* 0x0c : I2CD Interrupt Control Register &
 * 0x10 : I2CD Interrupt Status Register
 *
 * These share bit definitions, so use the same values for the enable &
 * status bits.
 */
#define ASPEED_I2CD_INTR_SDA_DL_TIMEOUT			BIT(14)
#define ASPEED_I2CD_INTR_BUS_RECOVER_DONE		BIT(13)
#define ASPEED_I2CD_INTR_SLAVE_MATCH			BIT(7)
#define ASPEED_I2CD_INTR_SCL_TIMEOUT			BIT(6)
#define ASPEED_I2CD_INTR_ABNORMAL			BIT(5)
#define ASPEED_I2CD_INTR_NORMAL_STOP			BIT(4)
#define ASPEED_I2CD_INTR_ARBIT_LOSS			BIT(3)
#define ASPEED_I2CD_INTR_RX_DONE			BIT(2)
#define ASPEED_I2CD_INTR_TX_NAK				BIT(1)
#define ASPEED_I2CD_INTR_TX_ACK				BIT(0)

/* 0x14 : I2CD Command/Status Register   */
#define ASPEED_I2CD_SCL_LINE_STS			BIT(18)
#define ASPEED_I2CD_SDA_LINE_STS			BIT(17)
#define ASPEED_I2CD_BUS_BUSY_STS			BIT(16)
#define ASPEED_I2CD_BUS_RECOVER_CMD			BIT(11)

/* Command Bit */
#define ASPEED_I2CD_M_STOP_CMD				BIT(5)
#define ASPEED_I2CD_M_S_RX_CMD_LAST			BIT(4)
#define ASPEED_I2CD_M_RX_CMD				BIT(3)
#define ASPEED_I2CD_S_TX_CMD				BIT(2)
#define ASPEED_I2CD_M_TX_CMD				BIT(1)
#define ASPEED_I2CD_M_START_CMD				BIT(0)

/* 0x18 : I2CD Slave Device Address Register   */
#define ASPEED_I2CD_DEV_ADDR_MASK			GENMASK(6, 0)

enum aspeed_i2c_slave_state {
	ASPEED_I2C_SLAVE_START,
	ASPEED_I2C_SLAVE_READ_REQUESTED,
	ASPEED_I2C_SLAVE_READ_PROCESSED,
	ASPEED_I2C_SLAVE_WRITE_REQUESTED,
	ASPEED_I2C_SLAVE_WRITE_RECEIVED,
	ASPEED_I2C_SLAVE_STOP,
};

struct aspeed_i2c_bus {
	struct i2c_adapter		adap;
	struct device			*dev;
	void __iomem			*base;
	spinlock_t			lock;
	struct completion		cmd_complete;
	int				irq;
	/* Transaction state. */
	struct i2c_msg			*msg;
	int				msg_pos;
	u32				cmd_err;
#if IS_ENABLED(CONFIG_I2C_SLAVE)
	struct i2c_client		*slave;
	enum aspeed_i2c_slave_state	slave_state;
#endif
};

struct aspeed_i2c_controller {
	struct device		*dev;
	void __iomem		*base;
	int			irq;
	struct irq_domain	*irq_domain;
};

static inline void aspeed_i2c_write(struct aspeed_i2c_bus *bus, u32 val,
				    u32 reg)
{
	writel(val, bus->base + reg);
}

static inline u32 aspeed_i2c_read(struct aspeed_i2c_bus *bus, u32 reg)
{
	return readl(bus->base + reg);
}

static u8 aspeed_i2c_recover_bus(struct aspeed_i2c_bus *bus)
{
	u32 command;
	unsigned long time_left;
	unsigned long flags;
	int ret = 0;

	spin_lock_irqsave(&bus->lock, flags);
	command = aspeed_i2c_read(bus, ASPEED_I2C_CMD_REG);
	/* Bus is idle: no recovery needed. */
	if ((command & ASPEED_I2CD_SDA_LINE_STS) &&
	    (command & ASPEED_I2CD_SCL_LINE_STS))
		goto out;

	dev_dbg(bus->dev, "bus hung (state %x), attempting recovery\n",
		command);

	/* Bus held: put bus in stop state. */
	if ((command & ASPEED_I2CD_SDA_LINE_STS) &&
	    !(command & ASPEED_I2CD_SCL_LINE_STS)) {
		aspeed_i2c_write(bus, ASPEED_I2CD_M_STOP_CMD,
				 ASPEED_I2C_CMD_REG);
		reinit_completion(&bus->cmd_complete);
		spin_unlock_irqrestore(&bus->lock, flags);

		time_left = wait_for_completion_interruptible_timeout(
				&bus->cmd_complete, bus->adap.timeout * HZ);

		spin_lock_irqsave(&bus->lock, flags);
		if (time_left == 0)
			ret = -ETIMEDOUT;
		else if (bus->cmd_err)
			ret = -EIO;
	/* Bus error. */
	} else if (!(command & ASPEED_I2CD_SDA_LINE_STS)) {
		aspeed_i2c_write(bus, ASPEED_I2CD_BUS_RECOVER_CMD,
				 ASPEED_I2C_CMD_REG);
		reinit_completion(&bus->cmd_complete);
		spin_unlock_irqrestore(&bus->lock, flags);

		time_left = wait_for_completion_interruptible_timeout(
				&bus->cmd_complete, bus->adap.timeout * HZ);

		spin_lock_irqsave(&bus->lock, flags);
		if (time_left == 0)
			ret = -ETIMEDOUT;
		else if (bus->cmd_err)
			ret = -EIO;
		/* Recovery failed. */
		else if (!(aspeed_i2c_read(bus, ASPEED_I2C_CMD_REG) &
			   ASPEED_I2CD_SDA_LINE_STS))
			ret = -EIO;
	}

out:
	spin_unlock_irqrestore(&bus->lock, flags);
	return ret;
}

#if IS_ENABLED(CONFIG_I2C_SLAVE)
static bool aspeed_i2c_slave_irq(struct aspeed_i2c_bus *bus)
{
	bool irq_handled = true;
	u32 command;
	u32 irq_status;
	u32 status_ack = 0;
	u8 value;
	struct i2c_client *slave = bus->slave;

	spin_lock(&bus->lock);
	if (!slave) {
		irq_handled = false;
		goto out;
	}
	command = aspeed_i2c_read(bus, ASPEED_I2C_CMD_REG);
	irq_status = aspeed_i2c_read(bus, ASPEED_I2C_INTR_STS_REG);

	/* Slave was requested, restart state machine. */
	if (irq_status & ASPEED_I2CD_INTR_SLAVE_MATCH) {
		status_ack |= ASPEED_I2CD_INTR_SLAVE_MATCH;
		bus->slave_state = ASPEED_I2C_SLAVE_START;
	}
	/* Slave is not currently active, irq was for someone else. */
	if (bus->slave_state == ASPEED_I2C_SLAVE_STOP) {
		irq_handled = false;
		goto out;
	}

	dev_dbg(bus->dev, "slave irq status 0x%08x, cmd 0x%08x\n",
		irq_status, command);

	/* Slave was sent something. */
	if (irq_status & ASPEED_I2CD_INTR_RX_DONE) {
		value = aspeed_i2c_read(bus, ASPEED_I2C_BYTE_BUF_REG) >> 8;
		/* Handle address frame. */
		if (bus->slave_state == ASPEED_I2C_SLAVE_START) {
			if (value & 0x1)
				bus->slave_state =
						ASPEED_I2C_SLAVE_READ_REQUESTED;
			else
				bus->slave_state =
						ASPEED_I2C_SLAVE_WRITE_REQUESTED;
		}
		status_ack |= ASPEED_I2CD_INTR_RX_DONE;
	}

	/* Slave was asked to stop. */
	if (irq_status & ASPEED_I2CD_INTR_NORMAL_STOP) {
		status_ack |= ASPEED_I2CD_INTR_NORMAL_STOP;
		bus->slave_state = ASPEED_I2C_SLAVE_STOP;
	}
	if (irq_status & ASPEED_I2CD_INTR_TX_NAK) {
		status_ack |= ASPEED_I2CD_INTR_TX_NAK;
		bus->slave_state = ASPEED_I2C_SLAVE_STOP;
	}

	if (bus->slave_state == ASPEED_I2C_SLAVE_READ_REQUESTED) {
		if (irq_status & ASPEED_I2CD_INTR_TX_ACK)
			dev_err(bus->dev, "Unexpected ACK on read request.\n");
		bus->slave_state = ASPEED_I2C_SLAVE_READ_PROCESSED;

		i2c_slave_event(slave, I2C_SLAVE_READ_REQUESTED, &value);
		aspeed_i2c_write(bus, value, ASPEED_I2C_BYTE_BUF_REG);
		aspeed_i2c_write(bus, ASPEED_I2CD_S_TX_CMD, ASPEED_I2C_CMD_REG);
	} else if (bus->slave_state == ASPEED_I2C_SLAVE_READ_PROCESSED) {
		status_ack |= ASPEED_I2CD_INTR_TX_ACK;
		if (!(irq_status & ASPEED_I2CD_INTR_TX_ACK))
			dev_err(bus->dev,
				"Expected ACK after processed read.\n");
		i2c_slave_event(slave, I2C_SLAVE_READ_PROCESSED, &value);
		aspeed_i2c_write(bus, value, ASPEED_I2C_BYTE_BUF_REG);
		aspeed_i2c_write(bus, ASPEED_I2CD_S_TX_CMD, ASPEED_I2C_CMD_REG);
	} else if (bus->slave_state == ASPEED_I2C_SLAVE_WRITE_REQUESTED) {
		bus->slave_state = ASPEED_I2C_SLAVE_WRITE_RECEIVED;
		i2c_slave_event(slave, I2C_SLAVE_WRITE_REQUESTED, &value);
	} else if (bus->slave_state == ASPEED_I2C_SLAVE_WRITE_RECEIVED) {
		i2c_slave_event(slave, I2C_SLAVE_WRITE_RECEIVED, &value);
	} else if (bus->slave_state == ASPEED_I2C_SLAVE_STOP) {
		i2c_slave_event(slave, I2C_SLAVE_STOP, &value);
	}

	if (status_ack != irq_status)
		dev_err(bus->dev,
			"irq handled != irq. expected %x, but was %x\n",
			irq_status, status_ack);
	aspeed_i2c_write(bus, status_ack, ASPEED_I2C_INTR_STS_REG);

out:
	spin_unlock(&bus->lock);
	return irq_handled;
}
#endif

static bool aspeed_i2c_master_irq(struct aspeed_i2c_bus *bus)
{
	const u32 errs = ASPEED_I2CD_INTR_ARBIT_LOSS |
		ASPEED_I2CD_INTR_ABNORMAL |
		ASPEED_I2CD_INTR_SCL_TIMEOUT |
		ASPEED_I2CD_INTR_SDA_DL_TIMEOUT |
		ASPEED_I2CD_INTR_TX_NAK;
	u32 irq_status;

	spin_lock(&bus->lock);
	irq_status = aspeed_i2c_read(bus, ASPEED_I2C_INTR_STS_REG);
	bus->cmd_err = irq_status & errs;

	dev_dbg(bus->dev, "master irq status 0x%08x\n", irq_status);

	/* No message to transfer. */
	if (bus->cmd_err ||
	    (irq_status & ASPEED_I2CD_INTR_NORMAL_STOP) ||
	    (irq_status & ASPEED_I2CD_INTR_BUS_RECOVER_DONE)) {
		complete(&bus->cmd_complete);
		goto out;
	} else if (!bus->msg || bus->msg_pos >= bus->msg->len)
		goto out;

	if ((bus->msg->flags & I2C_M_RD) &&
	    (irq_status & ASPEED_I2CD_INTR_RX_DONE)) {
		bus->msg->buf[bus->msg_pos++] = aspeed_i2c_read(
				bus, ASPEED_I2C_BYTE_BUF_REG) >> 8;
		if (bus->msg_pos + 1 < bus->msg->len)
			aspeed_i2c_write(bus, ASPEED_I2CD_M_RX_CMD,
					 ASPEED_I2C_CMD_REG);
		else if (bus->msg_pos < bus->msg->len)
			aspeed_i2c_write(bus, ASPEED_I2CD_M_RX_CMD |
				      ASPEED_I2CD_M_S_RX_CMD_LAST,
				      ASPEED_I2C_CMD_REG);
	} else if (!(bus->msg->flags & I2C_M_RD) &&
		   (irq_status & ASPEED_I2CD_INTR_TX_ACK)) {
		aspeed_i2c_write(bus, bus->msg->buf[bus->msg_pos++],
			      ASPEED_I2C_BYTE_BUF_REG);
		aspeed_i2c_write(bus, ASPEED_I2CD_M_TX_CMD, ASPEED_I2C_CMD_REG);
	}

	/* Transmission complete: notify caller. */
	if (bus->msg_pos >= bus->msg->len)
		complete(&bus->cmd_complete);
out:
	aspeed_i2c_write(bus, irq_status, ASPEED_I2C_INTR_STS_REG);
	spin_unlock(&bus->lock);
	return true;
}

static irqreturn_t aspeed_i2c_bus_irq(int irq, void *dev_id)
{
	struct aspeed_i2c_bus *bus = dev_id;

#if IS_ENABLED(CONFIG_I2C_SLAVE)
	if (aspeed_i2c_slave_irq(bus)) {
		dev_dbg(bus->dev, "irq handled by slave.\n");
		return IRQ_HANDLED;
	}
#endif
	if (aspeed_i2c_master_irq(bus)) {
		dev_dbg(bus->dev, "irq handled by master.\n");
		return IRQ_HANDLED;
	}
	dev_err(bus->dev, "irq not handled properly!\n");
	return IRQ_HANDLED;
}

static int aspeed_i2c_master_single_xfer(struct i2c_adapter *adap,
				      struct i2c_msg *msg)
{
	struct aspeed_i2c_bus *bus = adap->algo_data;
	unsigned long flags;
	u8 slave_addr;
	u32 command = ASPEED_I2CD_M_START_CMD | ASPEED_I2CD_M_TX_CMD;
	int ret = msg->len;
	unsigned long time_left;

	spin_lock_irqsave(&bus->lock, flags);
	bus->msg = msg;
	bus->msg_pos = 0;
	slave_addr = msg->addr << 1;
	if (msg->flags & I2C_M_RD) {
		slave_addr |= 1;
		command |= ASPEED_I2CD_M_RX_CMD;
		if (msg->len == 1)
			command |= ASPEED_I2CD_M_S_RX_CMD_LAST;
	}
	aspeed_i2c_write(bus, slave_addr, ASPEED_I2C_BYTE_BUF_REG);
	aspeed_i2c_write(bus, command, ASPEED_I2C_CMD_REG);
	reinit_completion(&bus->cmd_complete);
	spin_unlock_irqrestore(&bus->lock, flags);

	time_left = wait_for_completion_interruptible_timeout(
			&bus->cmd_complete, bus->adap.timeout * HZ * msg->len);
	if (time_left == 0)
		return -ETIMEDOUT;

	spin_lock_irqsave(&bus->lock, flags);
	if (bus->cmd_err)
		ret = -EIO;
	bus->msg = NULL;
	spin_unlock_irqrestore(&bus->lock, flags);

	return ret;
}

static int aspeed_i2c_master_xfer(struct i2c_adapter *adap,
				  struct i2c_msg *msgs, int num)
{
	struct aspeed_i2c_bus *bus = adap->algo_data;
	int ret;
	int i;
	unsigned long flags;
	unsigned long time_left;

	/* If bus is busy, attempt recovery. We assume a single master
	 * environment.
	 */
	if (aspeed_i2c_read(bus, ASPEED_I2C_CMD_REG) &
	    ASPEED_I2CD_BUS_BUSY_STS) {
		ret = aspeed_i2c_recover_bus(bus);
		if (ret)
			return ret;
	}

	for (i = 0; i < num; i++) {
		ret = aspeed_i2c_master_single_xfer(adap, &msgs[i]);
		if (ret < 0)
			break;
		/* TODO: Support other forms of I2C protocol mangling. */
		if (msgs[i].flags & I2C_M_STOP) {
			spin_lock_irqsave(&bus->lock, flags);
			aspeed_i2c_write(bus, ASPEED_I2CD_M_STOP_CMD,
				      ASPEED_I2C_CMD_REG);
			reinit_completion(&bus->cmd_complete);
			spin_unlock_irqrestore(&bus->lock, flags);

			time_left = wait_for_completion_interruptible_timeout(
					&bus->cmd_complete,
					bus->adap.timeout * HZ);
			if (time_left == 0)
				return -ETIMEDOUT;
		}
	}

	spin_lock_irqsave(&bus->lock, flags);
	aspeed_i2c_write(bus, ASPEED_I2CD_M_STOP_CMD, ASPEED_I2C_CMD_REG);
	reinit_completion(&bus->cmd_complete);
	spin_unlock_irqrestore(&bus->lock, flags);

	time_left = wait_for_completion_interruptible_timeout(
			&bus->cmd_complete, bus->adap.timeout * HZ);
	if (time_left == 0)
		return -ETIMEDOUT;

	/* If nothing went wrong, return number of messages transferred. */
	if (ret < 0)
		return ret;
	else
		return i;
}

static u32 aspeed_i2c_functionality(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL | I2C_FUNC_SMBUS_BLOCK_DATA;
}

#if IS_ENABLED(CONFIG_I2C_SLAVE)
static int aspeed_i2c_reg_slave(struct i2c_client *client)
{
	struct aspeed_i2c_bus *bus;
	unsigned long flags;
	u32 addr_reg_val;
	u32 func_ctrl_reg_val;

	bus = client->adapter->algo_data;
	spin_lock_irqsave(&bus->lock, flags);
	if (bus->slave) {
		spin_unlock_irqrestore(&bus->lock, flags);
		return -EINVAL;
	}

	/* Set slave addr. */
	addr_reg_val = aspeed_i2c_read(bus, ASPEED_I2C_DEV_ADDR_REG);
	addr_reg_val &= ~ASPEED_I2CD_DEV_ADDR_MASK;
	addr_reg_val |= client->addr & ASPEED_I2CD_DEV_ADDR_MASK;
	aspeed_i2c_write(bus, addr_reg_val, ASPEED_I2C_DEV_ADDR_REG);

	/* Switch from master mode to slave mode. */
	func_ctrl_reg_val = aspeed_i2c_read(bus, ASPEED_I2C_FUN_CTRL_REG);
	func_ctrl_reg_val &= ~ASPEED_I2CD_MASTER_EN;
	func_ctrl_reg_val |= ASPEED_I2CD_SLAVE_EN;
	aspeed_i2c_write(bus, func_ctrl_reg_val, ASPEED_I2C_FUN_CTRL_REG);

	bus->slave = client;
	bus->slave_state = ASPEED_I2C_SLAVE_STOP;
	spin_unlock_irqrestore(&bus->lock, flags);
	return 0;
}

static int aspeed_i2c_unreg_slave(struct i2c_client *client)
{
	struct aspeed_i2c_bus *bus = client->adapter->algo_data;
	unsigned long flags;
	u32 func_ctrl_reg_val;

	spin_lock_irqsave(&bus->lock, flags);
	if (!bus->slave) {
		spin_unlock_irqrestore(&bus->lock, flags);
		return -EINVAL;
	}

	/* Switch from slave mode to master mode. */
	func_ctrl_reg_val = aspeed_i2c_read(bus, ASPEED_I2C_FUN_CTRL_REG);
	func_ctrl_reg_val &= ~ASPEED_I2CD_SLAVE_EN;
	func_ctrl_reg_val |= ASPEED_I2CD_MASTER_EN;
	aspeed_i2c_write(bus, func_ctrl_reg_val, ASPEED_I2C_FUN_CTRL_REG);

	bus->slave = NULL;
	spin_unlock_irqrestore(&bus->lock, flags);
	return 0;
}
#endif

static const struct i2c_algorithm aspeed_i2c_algo = {
	.master_xfer	= aspeed_i2c_master_xfer,
	.functionality	= aspeed_i2c_functionality,
#if IS_ENABLED(CONFIG_I2C_SLAVE)
	.reg_slave	= aspeed_i2c_reg_slave,
	.unreg_slave	= aspeed_i2c_unreg_slave,
#endif
};

static u32 aspeed_i2c_get_clk_reg_val(u32 divider_ratio)
{
	unsigned int inc = 0, div;
	u32 scl_low, scl_high, data;

	for (div = 0; divider_ratio >= 16; div++) {
		inc |= (divider_ratio & 1);
		divider_ratio >>= 1;
	}
	divider_ratio += inc;
	scl_low = (divider_ratio >> 1) - 1;
	scl_high = divider_ratio - scl_low - 2;
	data = 0x77700300 | (scl_high << 16) | (scl_low << 12) | div;
	return data;
}

static int aspeed_i2c_init_clk(struct aspeed_i2c_bus *bus,
			    struct platform_device *pdev)
{
	struct clk *pclk;
	u32 clk_freq;
	u32 divider_ratio;
	int ret;

	pclk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(pclk)) {
		dev_err(&pdev->dev, "clk_get failed\n");
		return PTR_ERR(pclk);
	}
	ret = of_property_read_u32(pdev->dev.of_node,
			"clock-frequency", &clk_freq);
	if (ret < 0) {
		dev_err(&pdev->dev,
				"Could not read clock-frequency property\n");
		clk_freq = 100000;
	}
	divider_ratio = clk_get_rate(pclk) / clk_freq;
	/* We just need the clock rate, we don't actually use the clk object. */
	devm_clk_put(&pdev->dev, pclk);

	/* Set AC Timing */
	if (clk_freq / 1000 > 400) {
		aspeed_i2c_write(bus, aspeed_i2c_read(bus,
						      ASPEED_I2C_FUN_CTRL_REG) |
				ASPEED_I2CD_M_HIGH_SPEED_EN |
				ASPEED_I2CD_M_SDA_DRIVE_1T_EN |
				ASPEED_I2CD_SDA_DRIVE_1T_EN,
				ASPEED_I2C_FUN_CTRL_REG);

		aspeed_i2c_write(bus, 0x3, ASPEED_I2C_AC_TIMING_REG2);
		aspeed_i2c_write(bus, aspeed_i2c_get_clk_reg_val(divider_ratio),
			      ASPEED_I2C_AC_TIMING_REG1);
	} else {
		aspeed_i2c_write(bus, aspeed_i2c_get_clk_reg_val(divider_ratio),
			      ASPEED_I2C_AC_TIMING_REG1);
		aspeed_i2c_write(bus, ASPEED_NO_TIMEOUT_CTRL,
				 ASPEED_I2C_AC_TIMING_REG2);
	}

	return 0;
}

static void noop(struct irq_data *data) { }

static struct irq_chip aspeed_i2c_irqchip = {
	.name		= "ast-i2c",
	.irq_unmask	= noop,
	.irq_mask	= noop,
};

static int aspeed_i2c_probe_bus(struct platform_device *pdev)
{
	struct aspeed_i2c_bus *bus;
	struct aspeed_i2c_controller *controller =
			dev_get_drvdata(pdev->dev.parent);
	struct resource *res;
	int ret, irq;
	u32 hwirq;

	bus = devm_kzalloc(&pdev->dev, sizeof(*bus), GFP_KERNEL);
	if (!bus)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	bus->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(bus->base))
		return PTR_ERR(bus->base);

	bus->irq = platform_get_irq(pdev, 0);
	if (bus->irq < 0)
		return -ENXIO;
	ret = of_property_read_u32(pdev->dev.of_node, "interrupts", &hwirq);
	if (ret < 0) {
		dev_err(&pdev->dev, "no I2C 'interrupts' property\n");
		return -ENXIO;
	}
	irq = irq_create_mapping(controller->irq_domain, hwirq);
	irq_set_chip_data(irq, controller);
	irq_set_chip_and_handler(irq, &aspeed_i2c_irqchip, handle_simple_irq);
	ret = devm_request_irq(&pdev->dev, bus->irq, aspeed_i2c_bus_irq,
			0, dev_name(&pdev->dev), bus);
	if (ret) {
		dev_err(&pdev->dev, "failed to request interrupt\n");
		return -ENXIO;
	}

	/* Initialize the I2C adapter */
	spin_lock_init(&bus->lock);
	init_completion(&bus->cmd_complete);
	bus->adap.owner = THIS_MODULE;
	bus->adap.retries = 0;
	bus->adap.timeout = 5;
	bus->adap.algo = &aspeed_i2c_algo;
	bus->adap.algo_data = bus;
	bus->adap.dev.parent = &pdev->dev;
	bus->adap.dev.of_node = pdev->dev.of_node;
	snprintf(bus->adap.name, sizeof(bus->adap.name), "Aspeed i2c");

	bus->dev = &pdev->dev;

	/* reset device: disable master & slave functions */
	aspeed_i2c_write(bus, 0, ASPEED_I2C_FUN_CTRL_REG);

	ret = aspeed_i2c_init_clk(bus, pdev);
	if (ret < 0)
		return ret;

	/* Enable Master Mode */
	aspeed_i2c_write(bus, aspeed_i2c_read(bus, ASPEED_I2C_FUN_CTRL_REG) |
		      ASPEED_I2CD_MASTER_EN |
		      ASPEED_I2CD_MULTI_MASTER_DIS, ASPEED_I2C_FUN_CTRL_REG);

	/* Set interrupt generation of I2C controller */
	aspeed_i2c_write(bus, ASPEED_I2CD_INTR_SDA_DL_TIMEOUT |
			ASPEED_I2CD_INTR_BUS_RECOVER_DONE |
			ASPEED_I2CD_INTR_SCL_TIMEOUT |
			ASPEED_I2CD_INTR_ABNORMAL |
			ASPEED_I2CD_INTR_NORMAL_STOP |
			ASPEED_I2CD_INTR_ARBIT_LOSS |
			ASPEED_I2CD_INTR_RX_DONE |
			ASPEED_I2CD_INTR_TX_NAK |
			ASPEED_I2CD_INTR_TX_ACK,
			ASPEED_I2C_INTR_CTRL_REG);

	ret = i2c_add_adapter(&bus->adap);
	if (ret < 0)
		return -ENXIO;

	platform_set_drvdata(pdev, bus);

	dev_info(bus->dev, "i2c bus %d registered, irq %d\n",
			bus->adap.nr, bus->irq);

	return 0;
}

static int aspeed_i2c_remove_bus(struct platform_device *pdev)
{
	struct aspeed_i2c_bus *bus = platform_get_drvdata(pdev);

	i2c_del_adapter(&bus->adap);
	return 0;
}

static const struct of_device_id aspeed_i2c_bus_of_table[] = {
	{ .compatible = "aspeed,ast2400-i2c-bus", },
	{ .compatible = "aspeed,ast2500-i2c-bus", },
	{ },
};
MODULE_DEVICE_TABLE(of, aspeed_i2c_bus_of_table);

static struct platform_driver aspeed_i2c_bus_driver = {
	.probe		= aspeed_i2c_probe_bus,
	.remove		= aspeed_i2c_remove_bus,
	.driver		= {
		.name		= "ast-i2c-bus",
		.of_match_table	= aspeed_i2c_bus_of_table,
	},
};

static void aspeed_i2c_controller_irq(struct irq_desc *desc)
{
	struct aspeed_i2c_controller *c = irq_desc_get_handler_data(desc);
	unsigned long p, status;
	unsigned int bus_irq;

	status = readl(c->base);
	for_each_set_bit(p, &status, ASPEED_I2C_NUM_BUS) {
		bus_irq = irq_find_mapping(c->irq_domain, p);
		generic_handle_irq(bus_irq);
	}
}

static int aspeed_i2c_probe_controller(struct platform_device *pdev)
{
	struct aspeed_i2c_controller *controller;
	struct device_node *np;
	struct resource *res;

	controller = kzalloc(sizeof(*controller), GFP_KERNEL);
	if (!controller)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	controller->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(controller->base))
		return PTR_ERR(controller->base);

	controller->irq = platform_get_irq(pdev, 0);
	if (controller->irq < 0)
		return -ENXIO;

	controller->irq_domain = irq_domain_add_linear(pdev->dev.of_node,
			ASPEED_I2C_NUM_BUS, &irq_domain_simple_ops, NULL);
	if (!controller->irq_domain)
		return -ENXIO;
	controller->irq_domain->name = "ast-i2c-domain";

	irq_set_chained_handler_and_data(controller->irq,
			aspeed_i2c_controller_irq, controller);

	controller->dev = &pdev->dev;

	platform_set_drvdata(pdev, controller);

	dev_info(controller->dev, "i2c controller registered, irq %d\n",
			controller->irq);

	for_each_child_of_node(pdev->dev.of_node, np) {
		of_platform_device_create(np, NULL, &pdev->dev);
		of_node_put(np);
	}

	return 0;
}

static int aspeed_i2c_remove_controller(struct platform_device *pdev)
{
	struct aspeed_i2c_controller *controller = platform_get_drvdata(pdev);

	irq_domain_remove(controller->irq_domain);
	return 0;
}

static const struct of_device_id aspeed_i2c_controller_of_table[] = {
	{ .compatible = "aspeed,ast2400-i2c-controller", },
	{ .compatible = "aspeed,ast2500-i2c-controller", },
	{ },
};
MODULE_DEVICE_TABLE(of, aspeed_i2c_controller_of_table);

static struct platform_driver aspeed_i2c_controller_driver = {
	.probe		= aspeed_i2c_probe_controller,
	.remove		= aspeed_i2c_remove_controller,
	.driver		= {
		.name		= "ast-i2c-controller",
		.of_match_table	= aspeed_i2c_controller_of_table,
	},
};

static int __init aspeed_i2c_driver_init(void)
{
	int ret;

	ret = platform_driver_register(&aspeed_i2c_controller_driver);
	if (ret < 0)
		return ret;
	return platform_driver_register(&aspeed_i2c_bus_driver);
}
module_init(aspeed_i2c_driver_init);

static void __exit aspeed_i2c_driver_exit(void)
{
	platform_driver_unregister(&aspeed_i2c_bus_driver);
	platform_driver_unregister(&aspeed_i2c_controller_driver);
}
module_exit(aspeed_i2c_driver_exit);

MODULE_AUTHOR("Brendan Higgins <brendanhiggins@google.com>");
MODULE_DESCRIPTION("Aspeed I2C Bus Driver");
MODULE_LICENSE("GPL");
