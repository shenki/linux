/*
 *  i2c_adap_ast.c
 *
 *  I2C adapter for the ASPEED I2C bus access.
 *
 *  Copyright (C) 2012-2020  ASPEED Technology Inc.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 *  History:
 *    2012.07.26: Initial version [Ryan Chen]
 */
//#define DEBUG

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/completion.h>
#include <linux/slab.h>

#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/clk.h>

#include <linux/dma-mapping.h>

#include <asm/irq.h>
#include <asm/io.h>

#if defined(CONFIG_COLDFIRE)
#include <asm/arch/regs-iic.h>
#include <asm/arch/ast_i2c.h>
#else
#include <plat/regs-iic.h>
#include <plat/ast_i2c.h>
#endif

#define CONFIG_AST_I2C_SLAVE_RDWR
//#define AST_I2C_S_DEBUG 1

#ifdef AST_I2C_S_DEBUG
#define I2C_S_DBUG(fmt, args...) printk(fmt, ## args)
#else
#define I2C_S_DBUG(fmt, args...)
#endif

//#define AST_I2C_M_DEBUG 1

#ifdef AST_I2C_M_DEBUG
#define I2C_M_DBUG(fmt, args...) printk(fmt, ## args)
#else
#define I2C_M_DBUG(fmt, args...)
#endif

//AST2400 buffer mode issue , force I2C slave write use byte mode , read use buffer mode
/* Use platform_data instead of module parameters */
/* Fast Mode = 400 kHz, Standard = 100 kHz */
//static int clock = 100; /* Default: 100 kHz */
/***************************************************************************/

#ifdef CONFIG_AST_I2C_SLAVE_RDWR
#define I2C_S_BUF_SIZE 		256
#define I2C_S_RX_BUF_NUM 	40
#endif

#define AST_LOCKUP_DETECTED (0x1 << 15)

// Enable SCL/SDA pull LOW detection for Yosemite platform
#ifdef CONFIG_YOSEMITE
#define AST_I2C_LOW_TIMEOUT 0x07
#else
#define AST_I2C_LOW_TIMEOUT 0x00
#endif //CONFIG_YOSEMITE

struct ast_i2c_dev {
	struct ast_i2c_driver_data *ast_i2c_data;
	struct device		*dev;
	void __iomem		*reg_base;			/* virtual */
	int 				irq;				//I2C IRQ number
	u32					bus_id;				//for i2c dev# IRQ number check
	u32					state;				//I2C xfer mode state matchine
	u32				bus_recover;
	struct i2c_adapter	adap;
	struct buf_page		*req_page;
//speed up
	u32				disable;
	u32				rx_page;
	
//dma or buff mode needed
	unsigned char		*dma_buf;
	dma_addr_t			dma_addr;

//master
	u8					master_operation;
	int					xfer_last;			//cur xfer is last msgs for stop msgs
	struct i2c_msg 		*master_msgs;		//cur xfer msgs
	int					master_xfer_len;			//cur xfer len
	int					master_xfer_cnt;			//total xfer count
	struct completion		cmd_complete;
	int					cmd_err;
	u8 					blk_r_flag; 		//for smbus block read
	void 				(*do_master_xfer)(struct ast_i2c_dev *i2c_dev);
//Slave structure
	u8					slave_operation;
	struct i2c_msg		*slave_msgs; 		//cur slave xfer msgs
	int 					slave_xfer_len;
	int 					slave_xfer_cnt;
	void					(*do_slave_xfer)(struct ast_i2c_dev *i2c_dev);
	u8					slave_en;
#ifdef CONFIG_AST_I2C_SLAVE_RDWR
	u8					slave_rx_full;	
	u8					slave_rx_idx;	
	u8					slave_ioctl_idx;
	struct i2c_msg		slave_rx_msg[I2C_S_RX_BUF_NUM];
	struct i2c_msg		slave_tx_msg;
#endif
};



static inline void
ast_i2c_write(struct ast_i2c_dev *i2c_dev, u32 val, u32 reg)
{
//	dev_dbg(i2c_dev->dev, "ast_i2c_write : val: %x , reg : %x \n",val,reg);
	writel(val, i2c_dev->reg_base+ reg);
}

static inline u32
ast_i2c_read(struct ast_i2c_dev *i2c_dev, u32 reg)
{
#if 0
	u32 val = readl(i2c_dev->reg_base + reg);
	printk("R : reg %x , val: %x \n",reg, val);
	return val;
#else
	return readl(i2c_dev->reg_base + reg);
#endif
}

static u32 select_i2c_clock(struct ast_i2c_dev *i2c_dev)
{

	unsigned int clk, inc = 0, div, divider_ratio;
	u32 SCL_Low, SCL_High, data;

  // hack: The calculated value for 1MHz does not match with measured value, so override
  if (i2c_dev->ast_i2c_data->bus_clk == 1000000) {
//    data = 0x77744302;
	data = 0x77744002;
    return data;
  }

	clk = i2c_dev->ast_i2c_data->get_i2c_clock();
//	printk("pclk = %d \n",clk);
	divider_ratio = clk / i2c_dev->ast_i2c_data->bus_clk;
	for (div = 0; divider_ratio >= 16; div++)
	{
		inc |= (divider_ratio & 1);
		divider_ratio >>= 1;
	}
	divider_ratio += inc;
	SCL_Low = (divider_ratio >> 1) - 1;
	SCL_High = divider_ratio - SCL_Low - 2;
	data = 0x77700300 | (SCL_High << 16) | (SCL_Low << 12) | div;
//	printk("I2CD04 for %d = %08X\n", target_speed, data);
	return data;
}

#ifdef CONFIG_AST_I2C_SLAVE_MODE
/* AST I2C Slave mode  */
static void ast_slave_issue_alert(struct ast_i2c_dev *i2c_dev, u8 enable)
{
	//only support dev0~3
	if(i2c_dev->bus_id > 3)
		return;
	else {
		if(enable)
			ast_i2c_write(i2c_dev, ast_i2c_read(i2c_dev,I2C_CMD_REG) | AST_I2CD_S_ALT_EN, I2C_CMD_REG);
		else
			ast_i2c_write(i2c_dev, ast_i2c_read(i2c_dev,I2C_CMD_REG) & ~AST_I2CD_S_ALT_EN, I2C_CMD_REG);
	}
}

static void ast_slave_mode_enable(struct ast_i2c_dev *i2c_dev, struct i2c_msg *msgs)
{
	if(msgs->buf[0] == 1) {
		i2c_dev->slave_en = 1;
		printk("ioctl slave enable %d\n", i2c_dev->slave_en);

		ast_i2c_write(i2c_dev, msgs->addr, I2C_DEV_ADDR_REG);
		if(i2c_dev->master_operation) {
			dev_dbg(i2c_dev->dev, "master is ongoing\n");
		} else {
			ast_i2c_write(i2c_dev, ast_i2c_read(i2c_dev,I2C_FUN_CTRL_REG) | AST_I2CD_SLAVE_EN, I2C_FUN_CTRL_REG);		
		}
	} else {
		i2c_dev->slave_en = 0;
		printk("ioctl slave disable %d\n", i2c_dev->slave_en);
		ast_i2c_write(i2c_dev, ast_i2c_read(i2c_dev,I2C_FUN_CTRL_REG) & ~AST_I2CD_SLAVE_EN, I2C_FUN_CTRL_REG);
	}
}

#endif

static void ast_i2c_dev_init(struct ast_i2c_dev *i2c_dev)
{
	//I2CG Reset
	ast_i2c_write(i2c_dev, 0, I2C_FUN_CTRL_REG);

	ast_i2c_write(i2c_dev, ast_i2c_read(i2c_dev, I2C_FUN_CTRL_REG) | AST_I2CD_MASTER_EN, I2C_FUN_CTRL_REG);

	//High SPEED mode 
#if 0	
	ast_i2c_write(i2c_dev, ast_i2c_read(i2c_dev, I2C_FUN_CTRL_REG) |
						AST_I2CD_M_HIGH_SPEED_EN |
						AST_I2CD_M_SDA_DRIVE_1T_EN |
						AST_I2CD_SDA_DRIVE_1T_EN 
						, I2C_FUN_CTRL_REG);
	
	/* Set AC Timing */
	ast_i2c_write(i2c_dev, 0x3, I2C_AC_TIMING_REG2);
	ast_i2c_write(i2c_dev, select_i2c_clock(i2c_dev), I2C_AC_TIMING_REG1);		
#endif 

	/* Set AC Timing */
	ast_i2c_write(i2c_dev, select_i2c_clock(i2c_dev), I2C_AC_TIMING_REG1);
//	ast_i2c_write(i2c_dev, AST_NO_TIMEOUT_CTRL, I2C_AC_TIMING_REG2);
	ast_i2c_write(i2c_dev, 0x5, I2C_AC_TIMING_REG2);
//	ast_i2c_write(i2c_dev, 0x77743335, I2C_AC_TIMING_REG1);

	//Clear Interrupt
	ast_i2c_write(i2c_dev, 0xfffffff, I2C_INTR_STS_REG);

	//TODO
//	ast_i2c_write(i2c_dev, 0xAF, I2C_INTR_CTRL_REG);
	//Enable Interrupt, STOP Interrupt has bug in AST2000

	/* Set interrupt generation of I2C controller */
	ast_i2c_write(i2c_dev,
				AST_I2CD_SDA_DL_TO_INTR_EN |
				AST_I2CD_BUS_RECOVER_INTR_EN |
				AST_I2CD_SMBUS_ALT_INTR_EN |
//				AST_I2CD_SLAVE_MATCH_INTR_EN |
				AST_I2CD_SCL_TO_INTR_EN |
				AST_I2CD_ABNORMAL_INTR_EN |
				AST_I2CD_NORMAL_STOP_INTR_EN |
				AST_I2CD_ARBIT_LOSS_INTR_EN |
				AST_I2CD_RX_DOWN_INTR_EN,
//				AST_I2CD_TX_NAK_INTR_EN,
//				AST_I2CD_TX_ACK_INTR_EN,
				I2C_INTR_CTRL_REG);

}

#ifdef CONFIG_AST_I2C_SLAVE_RDWR
//for memory buffer initial
static void ast_i2c_slave_buff_init(struct ast_i2c_dev *i2c_dev)
{
	int i;
	//Tx buf  1
	i2c_dev->slave_tx_msg.len = I2C_S_BUF_SIZE;
	i2c_dev->slave_tx_msg.buf = kzalloc(I2C_S_BUF_SIZE, GFP_KERNEL);
	//Rx buf 4
	for(i=0; i<I2C_S_RX_BUF_NUM; i++) {
		i2c_dev->slave_rx_msg[i].addr = 0;
		i2c_dev->slave_rx_msg[i].flags = 0;	//mean empty buffer
		i2c_dev->slave_rx_msg[i].len = I2C_S_BUF_SIZE;
		i2c_dev->slave_rx_msg[i].buf = kzalloc(I2C_S_BUF_SIZE, GFP_KERNEL);
	}
	//assign cur msg is #0
	i2c_dev->slave_msgs = &i2c_dev->slave_rx_msg[0];
	i2c_dev->slave_rx_full = 0;
	i2c_dev->slave_rx_idx = 0;
	i2c_dev->slave_ioctl_idx = 0;
	
}

static int ast_i2c_slave_ioctl_xfer(struct i2c_adapter *adap, struct i2c_msg *msgs)
{
	struct ast_i2c_dev *i2c_dev = adap->algo_data;
	struct i2c_msg *slave_rx_msg = &i2c_dev->slave_rx_msg[i2c_dev->slave_ioctl_idx];
	u8 tmp_rx_idx;
	int ret=0;

	switch(msgs->flags) {
		case 0:
			if((slave_rx_msg->addr == 0) && (slave_rx_msg->flags == BUFF_FULL)) {
//				printk("%d-slave read %d\n", i2c_dev->bus_id, slave_rx_msg->len);
				if(slave_rx_msg->buf[0] != 0x20) {
					dev_err(i2c_dev->dev, "ioctl buffer 0x10 [%x] error !! ~~~~\n", slave_rx_msg->buf[0]);
					while(1);
				}
				memcpy(msgs->buf, slave_rx_msg->buf, slave_rx_msg->len);
				msgs->len = slave_rx_msg->len;
				slave_rx_msg->len = 0;
				slave_rx_msg->flags = 0;	
				i2c_dev->slave_ioctl_idx++;
				i2c_dev->slave_ioctl_idx %= I2C_S_RX_BUF_NUM;
				if(i2c_dev->slave_rx_full) {
//					dev_err(i2c_dev->dev, "slave re-enable ioctl[%d], rx[%d]\n", i2c_dev->slave_ioctl_idx, i2c_dev->slave_rx_idx);
					if(ast_i2c_read(i2c_dev, I2C_FUN_CTRL_REG) & AST_I2CD_SLAVE_EN) {
						dev_err(i2c_dev->dev, "buffer handle error !! ~~~~\n");
					} else {
						if(i2c_dev->slave_rx_msg[i2c_dev->slave_ioctl_idx].flags != BUFF_FULL) {
							i2c_dev->slave_rx_full = 0;
							if((i2c_dev->slave_en) && (!i2c_dev->master_operation)) {
								dev_err(i2c_dev->dev, "slave re-enable \n");
								ast_i2c_write(i2c_dev, ast_i2c_read(i2c_dev, I2C_FUN_CTRL_REG) | AST_I2CD_SLAVE_EN, I2C_FUN_CTRL_REG);
							} 						
						}

					}
				}
			} else {
//				printk("%d-No Buff\n", i2c_dev->bus_id);
				msgs->len = 0;
				ret = -1;
			}	
			break;
		case I2C_M_RD:	//slave write
			dev_dbg(i2c_dev->dev, "slave write \n");
			memcpy(msgs->buf, i2c_dev->slave_tx_msg.buf, I2C_S_BUF_SIZE);
			break;
		case I2C_S_EN:
			if((msgs->addr < 0x1) || (msgs->addr > 0xff)) {
				ret = -1;
				printk("addrsss not correct !! \n");
				ret = -1;
			} else {
				if(msgs->len != 1) printk("ERROR \n");
				ast_slave_mode_enable(i2c_dev, msgs);
			}
			break;
		case I2C_S_ALT:
			printk("slave issue alt\n");
			if(msgs->len != 1) printk("ERROR \n");
			if(msgs->buf[0]==1)
				ast_slave_issue_alert(i2c_dev, 1);
			else
				ast_slave_issue_alert(i2c_dev, 0);
			break;
		default:
			printk("slave xfer error \n");
			break;

	}
	return ret;
}

#endif

static u8
ast_i2c_bus_error_recover(struct ast_i2c_dev *i2c_dev)
{
	u32 sts;
	int r;
	u32 i = 0;
	printk("ast_i2c_bus_error_recover \n");
	//Check 0x14's SDA and SCL status
	sts = ast_i2c_read(i2c_dev,I2C_CMD_REG);

	if ((sts & AST_I2CD_SDA_LINE_STS) && (sts & AST_I2CD_SCL_LINE_STS)) {
		//Means bus is idle.
		dev_dbg(i2c_dev->dev, "I2C bus (%d) is idle. I2C slave doesn't exist?!\n", i2c_dev->bus_id);
		return -1;
	}

	dev_dbg(i2c_dev->dev, "ERROR!! I2C(%d) bus hanged, try to recovery it!\n", i2c_dev->bus_id);


	if ((sts & AST_I2CD_SDA_LINE_STS) && !(sts & AST_I2CD_SCL_LINE_STS)) {
		//if SDA == 1 and SCL == 0, it means the master is locking the bus.
		//Send a stop command to unlock the bus.
		dev_dbg(i2c_dev->dev, "I2C's master is locking the bus, try to stop it.\n");
//
		init_completion(&i2c_dev->cmd_complete);
		i2c_dev->cmd_err = 0;
		i2c_dev->master_operation = 1;
		ast_i2c_write(i2c_dev, AST_I2CD_M_STOP_CMD, I2C_CMD_REG);

		r = wait_for_completion_interruptible_timeout(&i2c_dev->cmd_complete,
													   i2c_dev->adap.timeout*HZ);

		if(i2c_dev->cmd_err) {
			dev_dbg(i2c_dev->dev, "recovery error \n");
			return -1;
		}

		if (r == 0) {
			 dev_dbg(i2c_dev->dev, "recovery timed out\n");
			 return -1;
		} else {
			dev_dbg(i2c_dev->dev, "Recovery successfully\n");
			return 0;
		}


	} else if (!(sts & AST_I2CD_SDA_LINE_STS)) {
		//else if SDA == 0, the device is dead. We need to reset the bus
		//And do the recovery command.
		dev_dbg(i2c_dev->dev, "I2C's slave is dead, try to recover it\n");
		//Let's retry 10 times
		for (i = 0; i < 10; i++) {
//			ast_i2c_dev_init(i2c_dev);
			//Do the recovery command BIT11
			init_completion(&i2c_dev->cmd_complete);
			i2c_dev->master_operation = 1;
			ast_i2c_write(i2c_dev, AST_I2CD_BUS_RECOVER_CMD_EN, I2C_CMD_REG);
			i2c_dev->bus_recover = 1;
			r = wait_for_completion_interruptible_timeout(&i2c_dev->cmd_complete,
														   i2c_dev->adap.timeout*HZ);
			if (i2c_dev->cmd_err != 0) {
				dev_dbg(i2c_dev->dev, "ERROR!! Failed to do recovery command(0x%08x)\n", i2c_dev->cmd_err);
				return -1;
			}
			//Check 0x14's SDA and SCL status
			sts = ast_i2c_read(i2c_dev,I2C_CMD_REG);
			if (sts & AST_I2CD_SDA_LINE_STS) //Recover OK
				break;
		}
		if (i == 10) {
			dev_dbg(i2c_dev->dev, "ERROR!! recover failed\n");
			return -1;
		}
	} else {
		dev_dbg(i2c_dev->dev, "Don't know how to handle this case?!\n");
		return -1;
	}
  dev_dbg(i2c_dev->dev, "Recovery successfully\n");
	return 0;
}

static void ast_master_alert_recv(struct ast_i2c_dev *i2c_dev)
{
	printk("ast_master_alert_recv bus id %d, Disable Alt, Please Imple \n",i2c_dev->bus_id);
}

static int ast_i2c_wait_bus_not_busy(struct ast_i2c_dev *i2c_dev)
{
	int timeout = 10;

	while(ast_i2c_read(i2c_dev,I2C_CMD_REG) & AST_I2CD_BUS_BUSY_STS) {
//		ast_i2c_bus_error_recover(i2c_dev);
		if(timeout<=0) {
			printk("%d-bus busy %x \n",i2c_dev->bus_id, ast_i2c_read(i2c_dev,I2C_CMD_REG));
			return -EAGAIN;
		}
		timeout--;
		mdelay(10);
	}

	return 0;
}

static void ast_i2c_slave_xfer_done(struct ast_i2c_dev *i2c_dev)
{
	u32 xfer_len = 0;
	int i;
	u8 *rx_buf;

	if (i2c_dev->slave_msgs->flags & I2C_M_RD) {
		dev_err(i2c_dev->dev, "slave tx \n");
	} else {
		//rx done
		dev_dbg(i2c_dev->dev, "rx \n");
		xfer_len = AST_I2CD_RX_BUF_ADDR_GET(ast_i2c_read(i2c_dev, I2C_BUF_CTRL_REG));
		if(xfer_len == 0)
			xfer_len = AST_I2C_PAGE_SIZE;
		rx_buf = (u8 *)i2c_dev->req_page->page_addr;
		if((i2c_dev->slave_msgs->len + xfer_len) > I2C_S_BUF_SIZE) {
			printk("out of buff size ~~~~\n");
		}
		for(i = 0; i < xfer_len ;i++) {
			i2c_dev->slave_msgs->buf[i2c_dev->slave_msgs->len+i] = rx_buf[i];
			dev_dbg(i2c_dev->dev,"%d, [%x] \n", i ,i2c_dev->slave_msgs->buf[i]);
		}			
		i2c_dev->slave_msgs->len += xfer_len;
	}

}

static irqreturn_t i2c_ast_handler(int this_irq, void *dev_id)
{
	u32 xfer_len = 0;
	int i;
	u8 *pool_buf;
	struct ast_i2c_dev *i2c_dev = dev_id;
	u32 sts = ast_i2c_read(i2c_dev,I2C_INTR_STS_REG);
	i2c_dev->state = (ast_i2c_read(i2c_dev,I2C_CMD_REG) >> 19) & 0xf;

	dev_dbg(i2c_dev->dev,"ISR : %x\n",sts);

//	dev_dbg(i2c_dev->dev,"sts machine %x, slave_op %d \n", xfer_sts,i2c_dev->slave_operation);
	if(i2c_dev->slave_operation && i2c_dev->master_operation) {
		printk("%d-M-S error !!!! isr : %x, sts : %x ~~~~~~~~~~~~~~~~~~~\n ", i2c_dev->bus_id, sts, i2c_dev->state);
	}
		
	if(AST_I2CD_INTR_STS_SMBUS_ALT & sts) {
		printk("AST_I2CD_INTR_STS_SMBUS_ALT \n");
		dev_dbg(i2c_dev->dev, "M clear isr: AST_I2CD_INTR_STS_SMBUS_ALT= %x\n",sts);
		//Disable ALT INT
		ast_i2c_write(i2c_dev, ast_i2c_read(i2c_dev, I2C_INTR_CTRL_REG) &
					~AST_I2CD_SMBUS_ALT_INTR_EN,
					I2C_INTR_CTRL_REG);
		ast_i2c_write(i2c_dev, AST_I2CD_INTR_STS_SMBUS_ALT, I2C_INTR_STS_REG);
		ast_master_alert_recv(i2c_dev);
		sts &= ~AST_I2CD_SMBUS_ALT_INTR_EN;
		return IRQ_HANDLED;
	}

	if(AST_I2CD_INTR_STS_ARBIT_LOSS & sts) {
		if(i2c_dev->master_operation) {
			dev_dbg(i2c_dev->dev, "M clear isr: AST_I2CD_INTR_STS_ARBIT_LOSS = %x\n",sts);
			i2c_dev->cmd_err = AST_I2CD_INTR_STS_ARBIT_LOSS;
			complete(&i2c_dev->cmd_complete);
			ast_i2c_write(i2c_dev, AST_I2CD_INTR_STS_ARBIT_LOSS, I2C_INTR_STS_REG);
		} else {
			printk("%d-AST_I2CD_INTR_STS_ARBIT_LOSS M[%d] S[%d] TODO ERROR \n", i2c_dev->bus_id, i2c_dev->master_operation, i2c_dev->slave_operation);
		}
		return IRQ_HANDLED;
	}
	
	if(AST_I2CD_INTR_STS_BUS_RECOVER & sts) {
		dev_dbg(i2c_dev->dev, "M clear isr: AST_I2CD_INTR_STS_BUS_RECOVER= %x\n",sts);
		printk("AST_I2CD_INTR_STS_BUS_RECOVER \n");
		ast_i2c_write(i2c_dev, AST_I2CD_INTR_STS_BUS_RECOVER, I2C_INTR_STS_REG);
		i2c_dev->cmd_err = 0;
		if(i2c_dev->bus_recover) {
			complete(&i2c_dev->cmd_complete);				
			i2c_dev->bus_recover = 0;
		} else {
			printk("Error !! Bus revover\n");
		}
		return IRQ_HANDLED;
	}

	if(AST_I2CD_INTR_STS_SMBUS_DEF_ADDR & sts) {
		printk("AST_I2CD_INTR_STS_SMBUS_DEF_ADDR TODO \n");
		return IRQ_HANDLED;
	}

	if(AST_I2CD_INTR_STS_SMBUS_DEV_ALT & sts) {
		printk("AST_I2CD_INTR_STS_SMBUS_DEV_ALT TODO \n");		
		return IRQ_HANDLED;
	}

	if(AST_I2CD_INTR_STS_SMBUS_ARP_ADDR & sts) {
		printk("AST_I2CD_INTR_STS_SMBUS_ARP_ADDR TODO \n");		
		return IRQ_HANDLED;
	}

	if(AST_I2CD_INTR_STS_GCALL_ADDR & sts) {
		printk("AST_I2CD_INTR_STS_GCALL_ADDR TODO \n");		
		i2c_dev->cmd_err = AST_I2CD_INTR_STS_GCALL_ADDR;
		return IRQ_HANDLED;		
	}

	if(AST_I2CD_INTR_STS_ABNORMAL & sts) {
		dev_dbg(i2c_dev->dev, "AST_I2CD_INTR_STS_ABNORMAL M [%x], S [%x] \n", i2c_dev->master_operation, i2c_dev->slave_operation);		
		if(i2c_dev->master_operation) {				
			i2c_dev->cmd_err = AST_I2CD_INTR_STS_ABNORMAL;			
			complete(&i2c_dev->cmd_complete);
			ast_i2c_write(i2c_dev, AST_I2CD_INTR_STS_ABNORMAL, I2C_INTR_STS_REG);
		} else {
			//drop package
			i2c_dev->slave_msgs->addr = 0;
			i2c_dev->slave_msgs->len = 0;
			ast_i2c_write(i2c_dev, AST_I2CD_INTR_STS_ABNORMAL, I2C_INTR_STS_REG);	
		}
		return IRQ_HANDLED;
	}	

	
	sts &= (AST_I2CD_INTR_STS_SDA_DL_TO | AST_I2CD_INTR_STS_TX_ACK | AST_I2CD_INTR_STS_NORMAL_STOP | AST_I2CD_INTR_STS_SCL_TO |
			AST_I2CD_INTR_STS_TX_NAK | AST_I2CD_INTR_STS_RX_DOWN | AST_I2CD_INTR_STS_SLAVE_MATCH);

	if(i2c_dev->master_operation) {
		if(AST_I2CD_INTR_STS_SDA_DL_TO & sts) {
			dev_dbg(i2c_dev->dev, "AST_I2CD_INTR_STS_SDA_DL_TO M [%x], S [%x] \n", i2c_dev->master_operation, i2c_dev->slave_operation);						
			i2c_dev->cmd_err = AST_I2CD_INTR_STS_SDA_DL_TO;
			complete(&i2c_dev->cmd_complete);
			ast_i2c_write(i2c_dev, AST_I2CD_INTR_STS_SDA_DL_TO, I2C_INTR_STS_REG); 
			return IRQ_HANDLED;
		}
		
		if(AST_I2CD_INTR_STS_SCL_TO & sts) {
			dev_dbg(i2c_dev->dev, "AST_I2CD_INTR_STS_SCL_TO M [%x], S [%x] \n", i2c_dev->master_operation, i2c_dev->slave_operation);				
			i2c_dev->cmd_err = AST_I2CD_INTR_STS_SCL_TO;
			complete(&i2c_dev->cmd_complete);
			ast_i2c_write(i2c_dev, AST_I2CD_INTR_STS_SCL_TO, I2C_INTR_STS_REG); 
			return IRQ_HANDLED; 	
		}	
		
		sts &= (AST_I2CD_INTR_STS_TX_ACK | AST_I2CD_INTR_STS_NORMAL_STOP | AST_I2CD_INTR_STS_TX_NAK | AST_I2CD_INTR_STS_RX_DOWN);
		
		switch(sts) {
			case AST_I2CD_INTR_STS_TX_ACK:
				dev_dbg(i2c_dev->dev, "M clear isr: AST_I2CD_INTR_STS_TX_ACK %x\n",sts);
				//workaround for buffer pool,  use byte mode master read ack
				if (i2c_dev->master_msgs->flags & I2C_M_RD) {
					dev_dbg(i2c_dev->dev, "M RD \n");
					//just read ack
				} else {
					dev_dbg(i2c_dev->dev, "M WR \n");
					xfer_len = AST_I2CD_TX_DATA_BUF_GET(ast_i2c_read(i2c_dev, I2C_BUF_CTRL_REG));
					xfer_len++;
					if(xfer_len != i2c_dev->master_xfer_len) {
						//TODO..
						dev_err(i2c_dev->dev,"xfer_len = %d  !=  master_xfer_len = %d  \n", xfer_len, i2c_dev->master_xfer_len);
						i2c_dev->cmd_err = 1;
					} else {
						i2c_dev->master_xfer_cnt += i2c_dev->master_xfer_len;
					}				
				}
				ast_i2c_write(i2c_dev, ast_i2c_read(i2c_dev, I2C_INTR_CTRL_REG) & ~AST_I2CD_TX_ACK_INTR_EN, I2C_INTR_CTRL_REG);
				complete(&i2c_dev->cmd_complete);
				ast_i2c_write(i2c_dev, AST_I2CD_INTR_STS_TX_ACK, I2C_INTR_STS_REG);
				break;
			case AST_I2CD_INTR_STS_TX_ACK | AST_I2CD_INTR_STS_NORMAL_STOP:
				dev_dbg(i2c_dev->dev, "M clear isr: AST_I2CD_INTR_STS_TX_ACK | AST_I2CD_INTR_STS_NORMAL_STOP= %x\n",sts);
				xfer_len = AST_I2CD_TX_DATA_BUF_GET(ast_i2c_read(i2c_dev, I2C_BUF_CTRL_REG));
				xfer_len++;
				if(xfer_len != i2c_dev->master_xfer_len) {
					//TODO..
					dev_err(i2c_dev->dev,"xfer_len = %d  !=  master_xfer_len = %d  \n", xfer_len, i2c_dev->master_xfer_len);
					i2c_dev->cmd_err = 1;
				} else {
					i2c_dev->master_xfer_cnt += i2c_dev->master_xfer_len;
				}				
				complete(&i2c_dev->cmd_complete);
				ast_i2c_write(i2c_dev, AST_I2CD_INTR_STS_TX_ACK | AST_I2CD_INTR_STS_NORMAL_STOP, I2C_INTR_STS_REG);
				break;
			case AST_I2CD_INTR_STS_TX_NAK | AST_I2CD_INTR_STS_NORMAL_STOP:
				dev_dbg(i2c_dev->dev, "M clear isr: AST_I2CD_INTR_STS_TX_NAK| AST_I2CD_INTR_STS_NORMAL_STOP = %x\n",sts);
				i2c_dev->cmd_err = AST_I2CD_INTR_STS_TX_NAK | AST_I2CD_INTR_STS_NORMAL_STOP;
				complete(&i2c_dev->cmd_complete);
				ast_i2c_write(i2c_dev, AST_I2CD_INTR_STS_TX_NAK | AST_I2CD_INTR_STS_NORMAL_STOP, I2C_INTR_STS_REG);
				break;
			case AST_I2CD_INTR_STS_TX_NAK:
				dev_dbg(i2c_dev->dev, "M clear isr: AST_I2CD_INTR_STS_TX_NAK = %x\n",sts);
				i2c_dev->cmd_err = AST_I2CD_INTR_STS_TX_NAK;
				ast_i2c_write(i2c_dev, ast_i2c_read(i2c_dev, I2C_INTR_CTRL_REG) & ~AST_I2CD_TX_NAK_INTR_EN, I2C_INTR_CTRL_REG);	
				complete(&i2c_dev->cmd_complete);
				ast_i2c_write(i2c_dev, AST_I2CD_INTR_STS_TX_NAK, I2C_INTR_STS_REG);
				break;
			case AST_I2CD_INTR_STS_NORMAL_STOP:
				if(i2c_dev->master_msgs->flags & I2C_M_RD) {
					//for normal read device nak and send stop 
					complete(&i2c_dev->cmd_complete);
					ast_i2c_write(i2c_dev, AST_I2CD_INTR_STS_NORMAL_STOP, I2C_INTR_STS_REG);					
				} else {
					//mater write not have 
					printk("TODO .. master .. sts %x \n", sts);
				}
				break;
			case (AST_I2CD_INTR_STS_RX_DOWN | AST_I2CD_INTR_STS_NORMAL_STOP):	
				if(i2c_dev->master_msgs->flags & I2C_M_RD) {
					//master rx down
					//workaround for pool buffer mode
					dev_dbg(i2c_dev->dev, "M clear isr: AST_I2CD_INTR_STS_NORMAL_STOP = %x\n",sts);
					//take care
					ast_i2c_write(i2c_dev, ast_i2c_read(i2c_dev,I2C_INTR_CTRL_REG) | AST_I2CD_RX_DOWN_INTR_EN, I2C_INTR_CTRL_REG);
					pool_buf = (u8 *)i2c_dev->req_page->page_addr;				
					xfer_len = AST_I2CD_RX_BUF_ADDR_GET(ast_i2c_read(i2c_dev, I2C_BUF_CTRL_REG));
					if(xfer_len == 0)
						xfer_len = AST_I2C_PAGE_SIZE;
					for(i = 0; i< xfer_len; i++) {
						i2c_dev->master_msgs->buf[i2c_dev->master_xfer_cnt + i] = pool_buf[i];
//						dev_dbg(i2c_dev->dev, "rx %d buff[%x]\n",i2c_dev->master_xfer_cnt+i, i2c_dev->master_msgs->buf[i2c_dev->master_xfer_cnt+i]);
					}
					complete(&i2c_dev->cmd_complete);
					ast_i2c_write(i2c_dev, AST_I2CD_INTR_STS_RX_DOWN | AST_I2CD_INTR_STS_NORMAL_STOP, I2C_INTR_STS_REG);					
				} else {
					printk("TODO .. master .. sts %x \n", sts);
				}
				
				break;
			case AST_I2CD_INTR_STS_RX_DOWN:
				printk("TODO .. master .. sts %x \n", sts);
				break;
			default:
				printk("%d-Master No one care : isr :%x, state %x, ctrl : %x, M[%d], S[%d] TODO ~~~\n", i2c_dev->bus_id, sts, i2c_dev->state, ast_i2c_read(i2c_dev,I2C_FUN_CTRL_REG), i2c_dev->master_operation, i2c_dev->slave_operation);
				complete(&i2c_dev->cmd_complete);
				ast_i2c_write(i2c_dev, ast_i2c_read(i2c_dev, I2C_FUN_CTRL_REG) & ~AST_I2CD_MASTER_EN, I2C_FUN_CTRL_REG);
				ast_i2c_write(i2c_dev, ast_i2c_read(i2c_dev, I2C_FUN_CTRL_REG) | AST_I2CD_MASTER_EN, I2C_FUN_CTRL_REG);				
				break;
		}		
	} else {
		//slave 
		if(AST_I2CD_INTR_STS_SDA_DL_TO & sts) {
			dev_dbg(i2c_dev->dev, "SDA_DL_TO isr %x M [%x], S [%x] \n",sts, i2c_dev->master_operation, i2c_dev->slave_operation);
			//drop package			
			i2c_dev->slave_msgs->addr = 0;
			i2c_dev->slave_msgs->len = 0;			
			i2c_dev->slave_operation = 0;
			ast_i2c_write(i2c_dev, ast_i2c_read(i2c_dev, I2C_FUN_CTRL_REG) & ~(AST_I2CD_MASTER_EN | AST_I2CD_SLAVE_EN), I2C_FUN_CTRL_REG);
			if(!i2c_dev->slave_rx_full) {
				ast_i2c_write(i2c_dev, ast_i2c_read(i2c_dev, I2C_FUN_CTRL_REG) | AST_I2CD_MASTER_EN | AST_I2CD_SLAVE_EN, I2C_FUN_CTRL_REG);
			} else {
				ast_i2c_write(i2c_dev, ast_i2c_read(i2c_dev, I2C_FUN_CTRL_REG) | AST_I2CD_MASTER_EN, I2C_FUN_CTRL_REG);
			}
			
			
			return IRQ_HANDLED;
		}

		if(AST_I2CD_INTR_STS_SCL_TO & sts) {
			dev_dbg(i2c_dev->dev, "SCL_TO isr %x M [%x], S [%x] \n",sts, i2c_dev->master_operation, i2c_dev->slave_operation);
			//drop package			
			i2c_dev->slave_msgs->addr = 0;
			i2c_dev->slave_msgs->len = 0;			
			i2c_dev->slave_operation = 0;			
			ast_i2c_write(i2c_dev, ast_i2c_read(i2c_dev, I2C_FUN_CTRL_REG) & ~(AST_I2CD_MASTER_EN | AST_I2CD_SLAVE_EN), I2C_FUN_CTRL_REG);
			if(!i2c_dev->slave_rx_full) {
				ast_i2c_write(i2c_dev, ast_i2c_read(i2c_dev, I2C_FUN_CTRL_REG) | AST_I2CD_MASTER_EN | AST_I2CD_SLAVE_EN, I2C_FUN_CTRL_REG);
			} else {
				ast_i2c_write(i2c_dev, ast_i2c_read(i2c_dev, I2C_FUN_CTRL_REG) | AST_I2CD_MASTER_EN, I2C_FUN_CTRL_REG);
			}
			return IRQ_HANDLED;
		}
		printk("S isr %x \n", sts);
		sts &= (AST_I2CD_INTR_STS_NORMAL_STOP | AST_I2CD_INTR_STS_RX_DOWN | AST_I2CD_INTR_STS_SLAVE_MATCH);
		switch(sts) {
			case AST_I2CD_INTR_STS_TX_NAK:			
			case AST_I2CD_INTR_STS_TX_ACK:
			case (AST_I2CD_INTR_STS_TX_NAK | AST_I2CD_INTR_STS_NORMAL_STOP):				
			case (AST_I2CD_INTR_STS_TX_ACK | AST_I2CD_INTR_STS_NORMAL_STOP):
			case (AST_I2CD_INTR_STS_TX_NAK | AST_I2CD_INTR_STS_SLAVE_MATCH):
			case AST_I2CD_INTR_STS_NORMAL_STOP:	
				dev_err(i2c_dev->dev, " S  error sts %x ~~~~\n", sts);
				while(1);	
			case AST_I2CD_INTR_STS_RX_DOWN | AST_I2CD_INTR_STS_SLAVE_MATCH:
				//Slave rx byte data
				//ast_i2c_slave_xfer_byte_done
				i2c_dev->slave_operation = 1;
#if 0				
				i2c_dev->slave_msgs->buf[0] = ast_i2c_read(i2c_dev, I2C_BYTE_BUF_REG) >> 8;
				if(i2c_dev->slave_msgs->buf[0] != 0x20) {
					dev_err(i2c_dev->dev, "0 buffer 0x10 [%x] error !! ~~~~\n", i2c_dev->slave_msgs->buf[0]);
					i2c_dev->slave_msgs->buf[0] = 0x20;
				}
#else
				i2c_dev->slave_msgs->buf[0] = 0x20;
#endif
				i2c_dev->slave_msgs->len = 1;
				//Write Start : if repeat start, use over write buffer .. 
				i2c_dev->slave_msgs->addr = BUFF_ONGOING;

				//workaround for S|Rx|P | S send 
				if(i2c_dev->slave_rx_msg[(i2c_dev->slave_rx_idx + 3) % I2C_S_RX_BUF_NUM].flags == BUFF_FULL) {
					i2c_dev->slave_rx_full = 1;
					dev_dbg(i2c_dev->dev, "buffer full-disable [%x] \n",ast_i2c_read(i2c_dev, I2C_CMD_REG));		
//					ast_i2c_write(i2c_dev, ast_i2c_read(i2c_dev, I2C_FUN_CTRL_REG) & ~(AST_I2CD_SLAVE_EN), I2C_FUN_CTRL_REG);	
				}

#if 0				
				//will enable when slave rx stop					
				ast_i2c_write(i2c_dev, ast_i2c_read(i2c_dev, I2C_FUN_CTRL_REG) & ~(AST_I2CD_SLAVE_EN), I2C_FUN_CTRL_REG);	

				//Slave prepare for Rx buffer 
				ast_i2c_write(i2c_dev,
							AST_I2CD_RX_BUF_END_ADDR_SET((i2c_dev->req_page->page_size-1)) |
							AST_I2CD_BUF_BASE_ADDR_SET((i2c_dev->req_page->page_addr_point)),
							I2C_BUF_CTRL_REG);
#else
				ast_i2c_write(i2c_dev, i2c_dev->disable, I2C_FUN_CTRL_REG);
				ast_i2c_write(i2c_dev, i2c_dev->rx_page, I2C_BUF_CTRL_REG);
#endif
				ast_i2c_write(i2c_dev, AST_I2CD_RX_BUFF_ENABLE, I2C_CMD_REG);
	
				ast_i2c_write(i2c_dev, AST_I2CD_INTR_STS_RX_DOWN | AST_I2CD_INTR_STS_SLAVE_MATCH, I2C_INTR_STS_REG);
				break;
			case AST_I2CD_INTR_STS_RX_DOWN | AST_I2CD_INTR_STS_NORMAL_STOP | AST_I2CD_INTR_STS_SLAVE_MATCH :
				//workaround for next start addr
				dev_err(i2c_dev->dev, "Rx | P | Addr \n");
				ast_i2c_slave_xfer_done(i2c_dev);
				//slave rx stop for next msg
				i2c_dev->slave_msgs->addr = 0;
				i2c_dev->slave_msgs->flags = BUFF_FULL;
				//net slave msg
				i2c_dev->slave_rx_idx++;
				i2c_dev->slave_rx_idx %= I2C_S_RX_BUF_NUM;
				i2c_dev->slave_msgs = &i2c_dev->slave_rx_msg[i2c_dev->slave_rx_idx];

				//ast_i2c_slave_xfer_byte_done 
				i2c_dev->slave_msgs->buf[0] = ast_i2c_read(i2c_dev, I2C_BYTE_BUF_REG) >> 8;
				if(i2c_dev->slave_msgs->buf[0] != 0x20) {
					dev_err(i2c_dev->dev, "2 buffer 0x10 [%x] error !! ~~~~\n", i2c_dev->slave_msgs->buf[0]);
//					while(1);
					i2c_dev->slave_msgs->buf[0] = 0x20;
				}				
				i2c_dev->slave_msgs->len = 1;

				if(i2c_dev->slave_msgs->addr) {
					//use repeat start, use over write buffer .. 
#if 0					
					dev_err(i2c_dev->dev, "slave ERROR S[%d], sts[%x] 0x00[%x]~~ TODO ~~~~!!\n", i2c_dev->slave_operation, (ast_i2c_read(i2c_dev,I2C_CMD_REG) >> 19) & 0xf, ast_i2c_read(i2c_dev, I2C_FUN_CTRL_REG));
					dev_err(i2c_dev->dev, "slave msg flag [%x] len [%d]\n", i2c_dev->slave_msgs->flags, i2c_dev->slave_msgs->len);
					dev_err(i2c_dev->dev, "slave ioctl idx [%d] rx_idx [%d] rx_full [%d]\n", i2c_dev->slave_ioctl_idx, i2c_dev->slave_rx_idx, i2c_dev->slave_rx_full);
#endif					
				} else {
					i2c_dev->slave_msgs->addr = BUFF_ONGOING;
				}

				//workaround for S|Rx|P | S send 
				if(i2c_dev->slave_rx_msg[(i2c_dev->slave_rx_idx + 3) % I2C_S_RX_BUF_NUM].flags == BUFF_FULL) {
					i2c_dev->slave_rx_full = 1;
					ast_i2c_write(i2c_dev, ast_i2c_read(i2c_dev, I2C_FUN_CTRL_REG) & ~(AST_I2CD_SLAVE_EN), I2C_FUN_CTRL_REG);	
				}
				//prepare for new rx
				dev_dbg(i2c_dev->dev, "(-->) slave prepare rx buf \n");
				ast_i2c_write(i2c_dev,
							AST_I2CD_RX_BUF_END_ADDR_SET((i2c_dev->req_page->page_size-1)) |
							AST_I2CD_BUF_BASE_ADDR_SET((i2c_dev->req_page->page_addr_point)),
							I2C_BUF_CTRL_REG);
				
				ast_i2c_write(i2c_dev, AST_I2CD_RX_BUFF_ENABLE, I2C_CMD_REG);
	
				ast_i2c_write(i2c_dev, AST_I2CD_INTR_STS_RX_DOWN | AST_I2CD_INTR_STS_NORMAL_STOP | AST_I2CD_INTR_STS_SLAVE_MATCH, I2C_INTR_STS_REG);								
				break;
				case AST_I2CD_INTR_STS_RX_DOWN: 			
					//drop this, have handle via S|Rx|P | S condictuin
					ast_i2c_write(i2c_dev, AST_I2CD_INTR_STS_RX_DOWN , I2C_INTR_STS_REG);
					break;
				
			case (AST_I2CD_INTR_STS_RX_DOWN | AST_I2CD_INTR_STS_NORMAL_STOP):
				dev_dbg(i2c_dev->dev, "S-Rx|P \n");
				ast_i2c_write(i2c_dev, AST_I2CD_INTR_STS_NORMAL_STOP | AST_I2CD_INTR_STS_RX_DOWN, I2C_INTR_STS_REG);				
				ast_i2c_slave_xfer_done(i2c_dev);
				i2c_dev->slave_msgs->addr = 0;
				i2c_dev->slave_msgs->flags = BUFF_FULL;
				i2c_dev->slave_rx_idx++;
				i2c_dev->slave_rx_idx %= I2C_S_RX_BUF_NUM;
				i2c_dev->slave_msgs = &i2c_dev->slave_rx_msg[i2c_dev->slave_rx_idx];
				i2c_dev->slave_msgs->len = 0;				
				i2c_dev->slave_operation = 0;			
				//will enable when slave rx stop
				if(!i2c_dev->slave_rx_full) {
					ast_i2c_write(i2c_dev, i2c_dev->disable | (AST_I2CD_SLAVE_EN), I2C_FUN_CTRL_REG);	
				}
				break;
			default:
				printk("%d-Slave No one care : isr :%x, state %x\n", i2c_dev->bus_id, sts, i2c_dev->state);
				break;
		}	
	}

	return IRQ_HANDLED;

}

static int ast_i2c_do_msgs_xfer(struct ast_i2c_dev *i2c_dev, struct i2c_msg *msgs, int num)
{
	int i;
	int ret = 0;	
	u32 cmd = 0;
	long timeout = 0;
	int msg_num = 0;
	unsigned long flags;
	spinlock_t lock;
	
	u32 *tx_buf = (u32 *) i2c_dev->req_page->page_addr;
	u32 *rx_buf = (u32 *) i2c_dev->req_page->page_addr;

	i2c_dev->master_operation = 1;
	ast_i2c_write(i2c_dev, 0x10, I2C_AC_TIMING_REG2);

	for (msg_num=0; msg_num< num; msg_num++) {
		i2c_dev->blk_r_flag = 0;
		i2c_dev->master_msgs = &msgs[msg_num];
		if(num == msg_num+1)
			i2c_dev->xfer_last = 1;
		else
			i2c_dev->xfer_last = 0;

		i2c_dev->cmd_err = 0;
		i2c_dev->master_xfer_cnt = -1;
//		printk("[%d] msgs flags %x, len %d \n", msg_num, i2c_dev->master_msgs->flags, i2c_dev->master_msgs->len);
		dev_dbg(i2c_dev->dev,"[%d] msgs flags %x, len %d \n", msg_num, i2c_dev->master_msgs->flags, i2c_dev->master_msgs->len);

		if(i2c_dev->master_msgs->flags & I2C_M_RD) {
			//20161107 Ryan ADD
			//workaround .. HW can;t send start read addr with buff mode 
			i2c_dev->master_xfer_len = 1;
			dev_dbg(i2c_dev->dev,"M read addr %x cnt %d, xf len %d \n", i2c_dev->master_msgs->addr, i2c_dev->master_xfer_cnt, i2c_dev->master_msgs->len);
							
			ast_i2c_write(i2c_dev, (i2c_dev->master_msgs->addr <<1) |0x1, I2C_BYTE_BUF_REG);

			ast_i2c_write(i2c_dev, ast_i2c_read(i2c_dev, I2C_INTR_CTRL_REG) | AST_I2CD_TX_NAK_INTR_EN | AST_I2CD_TX_ACK_INTR_EN, I2C_INTR_CTRL_REG);	
			
			ast_i2c_write(i2c_dev, AST_I2CD_M_TX_CMD | AST_I2CD_M_START_CMD, I2C_CMD_REG);
			
			timeout = wait_for_completion_interruptible_timeout(&i2c_dev->cmd_complete,
														   i2c_dev->adap.timeout*HZ);
		
			if (timeout == 0) {
				dev_err(i2c_dev->dev, "controller read timed out\n");
				i2c_dev->state = (ast_i2c_read(i2c_dev,I2C_CMD_REG) >> 19) & 0xf;
				ast_i2c_write(i2c_dev, ast_i2c_read(i2c_dev, I2C_FUN_CTRL_REG) & ~AST_I2CD_MASTER_EN, I2C_FUN_CTRL_REG);
				ast_i2c_write(i2c_dev, ast_i2c_read(i2c_dev, I2C_FUN_CTRL_REG) | AST_I2CD_MASTER_EN, I2C_FUN_CTRL_REG);
				ret = -ETIMEDOUT;
				break;
			}
			
			if(i2c_dev->cmd_err != 0) {
				if(i2c_dev->cmd_err == AST_I2CD_INTR_STS_TX_NAK) {
					i2c_dev->master_xfer_len = 0;
					dev_dbg(i2c_dev->dev, "nak send stop \n");
					ast_i2c_write(i2c_dev, AST_I2CD_M_STOP_CMD, I2C_CMD_REG);
					timeout = wait_for_completion_interruptible_timeout(&i2c_dev->cmd_complete,
															   i2c_dev->adap.timeout*HZ);
					if (timeout == 0) {
						dev_dbg(i2c_dev->dev, "send stop timed out\n");
						i2c_dev->state = (ast_i2c_read(i2c_dev,I2C_CMD_REG) >> 19) & 0xf;
						dev_dbg(i2c_dev->dev, "sts [%x], isr sts [%x] \n",i2c_dev->state, ast_i2c_read(i2c_dev,I2C_INTR_STS_REG));
						ret = -ETIMEDOUT;
					}
					ret = -ETIMEDOUT;
					goto out;
				}
				ret = -EAGAIN;
				break;
			} 
			
			i2c_dev->master_xfer_cnt += i2c_dev->master_xfer_len;

			//do last rx buffer 
			
			i2c_dev->master_xfer_len = i2c_dev->master_msgs->len;
			if(i2c_dev->master_xfer_len == 0) goto stop;
			ast_i2c_write(i2c_dev, ast_i2c_read(i2c_dev,I2C_INTR_CTRL_REG) & ~AST_I2CD_RX_DOWN_INTR_EN, I2C_INTR_CTRL_REG);
//			printk("rx len %d \n", i2c_dev->master_xfer_len);
			ast_i2c_write(i2c_dev,
								AST_I2CD_RX_BUF_END_ADDR_SET((i2c_dev->master_xfer_len-1))|
								AST_I2CD_BUF_BASE_ADDR_SET((i2c_dev->req_page->page_addr_point)),
								I2C_BUF_CTRL_REG);

			ast_i2c_write(i2c_dev, AST_I2CD_M_RX_CMD | AST_I2CD_RX_BUFF_ENABLE | AST_I2CD_M_STOP_CMD | AST_I2CD_M_S_RX_CMD_LAST, I2C_CMD_REG);
			
		} else {
			tx_buf[0] = (i2c_dev->master_msgs->addr << 1);	//+1
			//next data write
			if((i2c_dev->master_msgs->len + 1) > i2c_dev->req_page->page_size)
				i2c_dev->master_xfer_len = i2c_dev->req_page->page_size;
			else
				i2c_dev->master_xfer_len = i2c_dev->master_msgs->len + 1;

			for(i = 1; i < i2c_dev->master_xfer_len; i++) {
				if(i%4 == 0)
					tx_buf[i/4] = 0;
				tx_buf[i/4] |= (i2c_dev->master_msgs->buf[i2c_dev->master_xfer_cnt + i] << ((i%4)*8)) ;
			}

			if(i2c_dev->xfer_last) {
				cmd = AST_I2CD_M_START_CMD | AST_I2CD_M_TX_CMD | AST_I2CD_TX_BUFF_ENABLE | AST_I2CD_M_STOP_CMD;
			} else {
				cmd = AST_I2CD_M_START_CMD | AST_I2CD_M_TX_CMD | AST_I2CD_TX_BUFF_ENABLE;
				ast_i2c_write(i2c_dev, ast_i2c_read(i2c_dev, I2C_INTR_CTRL_REG) | AST_I2CD_TX_ACK_INTR_EN, I2C_INTR_CTRL_REG);
			}

			ast_i2c_write(i2c_dev,
						AST_I2CD_TX_DATA_BUF_END_SET((i2c_dev->master_xfer_len - 1)) |
						AST_I2CD_BUF_BASE_ADDR_SET(i2c_dev->req_page->page_addr_point),
						I2C_BUF_CTRL_REG);
			ast_i2c_write(i2c_dev, cmd, I2C_CMD_REG);		
		}

		timeout = wait_for_completion_interruptible_timeout(&i2c_dev->cmd_complete, i2c_dev->adap.timeout*HZ);

		if (timeout == 0) {
			dev_err(i2c_dev->dev, "controller timed out\n");
			i2c_dev->state = (ast_i2c_read(i2c_dev,I2C_CMD_REG) >> 19) & 0xf;
			dev_err(i2c_dev->dev, "0x14 [%x] sts [%x], isr sts [%x] \n",ast_i2c_read(i2c_dev,I2C_CMD_REG) , i2c_dev->state, ast_i2c_read(i2c_dev,I2C_INTR_STS_REG));
			dev_err(i2c_dev->dev, "0x14 [%x] sts [%x], isr sts [%x] \n",ast_i2c_read(i2c_dev,I2C_CMD_REG) , i2c_dev->state, ast_i2c_read(i2c_dev,I2C_INTR_STS_REG));
			dev_err(i2c_dev->dev, "0x10 [%x] M [%x], S [%x] \n",ast_i2c_read(i2c_dev,I2C_INTR_STS_REG) , i2c_dev->master_operation, i2c_dev->slave_operation);
			dev_err(i2c_dev->dev, "0x00 [%x] M [%x], S [%x] \n",ast_i2c_read(i2c_dev,I2C_FUN_CTRL_REG) , i2c_dev->master_operation, i2c_dev->slave_operation);
			dev_err(i2c_dev->dev, "controller timed out ~~ reset \n");

			ast_i2c_write(i2c_dev, ast_i2c_read(i2c_dev, I2C_FUN_CTRL_REG) & ~AST_I2CD_MASTER_EN, I2C_FUN_CTRL_REG);
			ast_i2c_write(i2c_dev, ast_i2c_read(i2c_dev, I2C_FUN_CTRL_REG) | AST_I2CD_MASTER_EN, I2C_FUN_CTRL_REG);
			ret = -ETIMEDOUT;
			break;
		}

		if(i2c_dev->cmd_err != 0) {
			if(i2c_dev->cmd_err == AST_I2CD_INTR_STS_TX_NAK) {
stop:
				ast_i2c_write(i2c_dev, AST_I2CD_M_STOP_CMD, I2C_CMD_REG);
				timeout = wait_for_completion_interruptible_timeout(&i2c_dev->cmd_complete,
														   i2c_dev->adap.timeout*HZ);
				if (timeout == 0) {
					dev_dbg(i2c_dev->dev, "send stop timed out\n");
					i2c_dev->state = (ast_i2c_read(i2c_dev,I2C_CMD_REG) >> 19) & 0xf;
					dev_dbg(i2c_dev->dev, "sts [%x], isr sts [%x] \n",i2c_dev->state, ast_i2c_read(i2c_dev,I2C_INTR_STS_REG));
					ret = -ETIMEDOUT;
				}				
			}
			ret = -EAGAIN;
			break;
		} 
		dev_dbg(i2c_dev->dev,"[%d] msgs end \n", msg_num);

//		printk("[%d] msgs end \n", msg_num);
		ret++;
		
	}
	
out:

	return ret;

}

static int ast_i2c_xfer(struct i2c_adapter *adap, struct i2c_msg *msgs, int num)
{
	struct ast_i2c_dev *i2c_dev = adap->algo_data;
	int ret, i;
	
	for (i = adap->retries; i >= 0; i--) {
		if(i2c_dev->slave_operation) {
			goto next;
		} 
	
		if(ast_i2c_read(i2c_dev,I2C_CMD_REG) & AST_I2CD_BUS_BUSY_STS)
			goto next;
	
		ast_i2c_write(i2c_dev, i2c_dev->disable, I2C_FUN_CTRL_REG); //will enable when slave rx stop
		i2c_dev->state = (ast_i2c_read(i2c_dev,I2C_CMD_REG) >> 19) & 0xf;
		if (i2c_dev->state) {
			if(!i2c_dev->slave_rx_full)
				ast_i2c_write(i2c_dev, ast_i2c_read(i2c_dev, I2C_FUN_CTRL_REG) | AST_I2CD_SLAVE_EN, I2C_FUN_CTRL_REG);
			goto next;
		}
	
		ret = ast_i2c_do_msgs_xfer(i2c_dev, msgs, num);
		
		if (ret != -EAGAIN)
			goto out;

next:
//		udelay(10);
//		mdelay(1);
//		mdelay(3);
		mdelay(20);
	}

	ret = -EREMOTEIO;
	
out:
	dev_dbg(i2c_dev->dev, "%d-m: end \n", i2c_dev->bus_id);

	return ret;
}

static u32 ast_i2c_functionality(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL | I2C_FUNC_SMBUS_BLOCK_DATA;
}

static const struct i2c_algorithm i2c_ast_algorithm = {
	.master_xfer	= ast_i2c_xfer,
#ifdef CONFIG_AST_I2C_SLAVE_RDWR
	.slave_xfer		= ast_i2c_slave_ioctl_xfer,
#endif
	.functionality	= ast_i2c_functionality,
};

static int ast_i2c_probe(struct platform_device *pdev)
{
	struct ast_i2c_dev *i2c_dev;
	struct resource *res;
	int ret;

	dev_dbg(&pdev->dev, "ast_i2c_probe \n");

	i2c_dev = kzalloc(sizeof(struct ast_i2c_dev), GFP_KERNEL);
	if (!i2c_dev) {
		ret = -ENOMEM;
		goto err_no_mem;
	}

	i2c_dev->ast_i2c_data = pdev->dev.platform_data;
	if(i2c_dev->ast_i2c_data->master_dma == BUFF_MODE) {
		dev_dbg(&pdev->dev, "use buffer pool mode 256\n");

	} else if ((i2c_dev->ast_i2c_data->master_dma == INC_DMA_MODE) || (i2c_dev->ast_i2c_data->slave_dma == INC_DMA_MODE)) {
		dev_dbg(&pdev->dev, "use dma mode \n");
		if (!i2c_dev->dma_buf) {
			i2c_dev->dma_buf = dma_alloc_coherent(NULL, AST_I2C_DMA_SIZE, &i2c_dev->dma_addr, GFP_KERNEL);
			if (!i2c_dev->dma_buf) {
				printk("unable to allocate tx Buffer memory\n");
				ret = -ENOMEM;
				goto err_no_dma;
			}
			if(i2c_dev->dma_addr%4 !=0) {
				printk("not 4 byte boundary \n");
				ret = -ENOMEM;
				goto err_no_dma;
			}
//			printk("dma_buf = [0x%x] dma_addr = [0x%x], please check 4byte boundary \n",i2c_dev->dma_buf,i2c_dev->dma_addr);
			memset (i2c_dev->dma_buf, 0, AST_I2C_DMA_SIZE);
		}

	} else {
		//master_mode 0: use byte mode
		dev_dbg(&pdev->dev, "use default byte mode \n");
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (NULL == res) {
		dev_err(&pdev->dev, "cannot get IORESOURCE_MEM\n");
		ret = -ENOENT;
		goto err_no_io_res;
	}
	if (!request_mem_region(res->start, resource_size(res), res->name)) {
        dev_err(&pdev->dev, "cannot reserved region\n");
        ret = -ENXIO;
        goto err_no_io_res;
	}

	i2c_dev->reg_base = ioremap(res->start, resource_size(res));
	if (!i2c_dev->reg_base) {
		ret = -EIO;
		goto release_mem;
	}

	i2c_dev->irq = platform_get_irq(pdev, 0);
	if (i2c_dev->irq < 0) {
		dev_err(&pdev->dev, "no irq specified\n");
		ret = -ENOENT;
		goto ereqirq;
	}

	i2c_dev->dev = &pdev->dev;

	i2c_dev->bus_id = pdev->id;

 	/* Initialize the I2C adapter */
	i2c_dev->adap.owner   = THIS_MODULE;
//TODO
//	i2c_dev->adap.retries = 0;

//	i2c_dev->adap.retries = 3;
	i2c_dev->adap.retries = 5;

	i2c_dev->adap.timeout = 3;

	/*
	 * If "pdev->id" is negative we consider it as zero.
	 * The reason to do so is to avoid sysfs names that only make
	 * sense when there are multiple adapters.
	 */
	i2c_dev->adap.nr = pdev->id != -1 ? pdev->id : 0;
	snprintf(i2c_dev->adap.name, sizeof(i2c_dev->adap.name), "ast_ipmb.%u",
		 i2c_dev->adap.nr);

	i2c_dev->slave_operation = 0;
	i2c_dev->blk_r_flag = 0;
	i2c_dev->adap.algo = &i2c_ast_algorithm;

	ast_i2c_dev_init(i2c_dev);

	if(i2c_dev->ast_i2c_data->request_pool_buff_page(&(i2c_dev->req_page)))
		printk("request_pool_buff_page error ~~~~~~~~~~~~~~~~~~\n");

#if defined(CONFIG_ARCH_AST2400)
//muust after init 
	ast_i2c_write(i2c_dev,
					(ast_i2c_read(i2c_dev, I2C_FUN_CTRL_REG) &
					~AST_I2CD_BUFF_SEL_MASK) |
					AST_I2CD_BUFF_SEL(i2c_dev->req_page->page_no),
					I2C_FUN_CTRL_REG);


//
	i2c_dev->disable = ast_i2c_read(i2c_dev, I2C_FUN_CTRL_REG) | AST_I2CD_MASTER_EN;

	//Slave prepare for Rx buffer 
	i2c_dev->rx_page = AST_I2CD_RX_BUF_END_ADDR_SET((i2c_dev->req_page->page_size-1)) |
			AST_I2CD_BUF_BASE_ADDR_SET((i2c_dev->req_page->page_addr_point));

//
#endif

#ifdef CONFIG_AST_I2C_SLAVE_RDWR
	ast_i2c_slave_buff_init(i2c_dev);
#endif
	init_completion(&i2c_dev->cmd_complete);

	ret = request_irq(i2c_dev->irq, i2c_ast_handler, IRQF_SHARED,
			  i2c_dev->adap.name, i2c_dev);
	if (ret) {
		printk(KERN_INFO "I2C: Failed request irq %d\n", i2c_dev->irq);
		goto ereqirq;
	}

	i2c_dev->adap.algo_data = i2c_dev;
	i2c_dev->adap.dev.parent = &pdev->dev;

	//new kernel version
	i2c_dev->bus_id = pdev->id;

//	i2c_dev->adap.id = pdev->id;

	ret = i2c_add_numbered_adapter(&i2c_dev->adap);
	if (ret < 0) {
		printk(KERN_INFO "I2C: Failed to add bus\n");
		goto eadapt;
	}

	platform_set_drvdata(pdev, i2c_dev);

	printk(KERN_INFO "I2C: %s: AST I2C adapter [%d khz]\n",
	       i2c_dev->adap.name,i2c_dev->ast_i2c_data->bus_clk/1000);

	return 0;

eadapt:
	free_irq(i2c_dev->irq, i2c_dev);
ereqirq:
	iounmap(i2c_dev->reg_base);

release_mem:
	release_mem_region(res->start, resource_size(res));
err_no_io_res:
err_no_dma:
	kfree(i2c_dev);

err_no_mem:
	return ret;
}

static int ast_i2c_remove(struct platform_device *pdev)
{
	struct ast_i2c_dev *i2c_dev = platform_get_drvdata(pdev);
	struct resource *res;

	platform_set_drvdata(pdev, NULL);
	i2c_del_adapter(&i2c_dev->adap);

	free_irq(i2c_dev->irq, i2c_dev);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	iounmap(i2c_dev->reg_base);
	release_mem_region(res->start, res->end - res->start + 1);

	kfree(i2c_dev);

	return 0;
}

#ifdef CONFIG_PM
static int ast_i2c_suspend(struct platform_device *pdev, pm_message_t state)
{
	//TODO
//	struct ast_i2c_dev *i2c_dev = platform_get_drvdata(pdev);
	return 0;
}

static int ast_i2c_resume(struct platform_device *pdev)
{
	//TODO
//	struct ast_i2c_dev *i2c_dev = platform_get_drvdata(pdev);
	//Should reset i2c ???
	return 0;
}
#else
#define ast_i2c_suspend        NULL
#define ast_i2c_resume         NULL
#endif

static struct platform_driver i2c_ast_driver = {
	.probe			= ast_i2c_probe,
	.remove 		= ast_i2c_remove,
    .suspend        = ast_i2c_suspend,
    .resume         = ast_i2c_resume,
    .driver         = {
            .name   = "ast-ipmb",
            .owner  = THIS_MODULE,
    },
};

static int __init ast_i2c_init(void)
{
	return platform_driver_register(&i2c_ast_driver);
}

static void __exit ast_i2c_exit(void)
{
	platform_driver_unregister(&i2c_ast_driver);
}
//TODO : check module init sequence
module_init(ast_i2c_init);
module_exit(ast_i2c_exit);

MODULE_AUTHOR("Ryan Chen <ryan_chen@aspeedtech.com>");
MODULE_DESCRIPTION("ASPEED AST I2C Bus Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:ast_i2c");
