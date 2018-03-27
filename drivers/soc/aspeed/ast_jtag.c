/*
 * ast-jtag.c - JTAG driver for the Aspeed SoC
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
#include <linux/sysfs.h>
#include <linux/clk.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <asm/io.h>
#include <asm/uaccess.h>
/*************************************************************************************/
#define AST_JTAG_DATA			0x00
#define AST_JTAG_INST			0x04
#define AST_JTAG_CTRL			0x08
#define AST_JTAG_ISR				0x0C
#define AST_JTAG_SW				0x10
#define AST_JTAG_TCK				0x14
#define AST_JTAG_IDLE			0x18

/* AST_JTAG_CTRL - 0x08 : Engine Control */
#define JTAG_ENG_EN				(0x1 << 31)
#define JTAG_ENG_OUT_EN			(0x1 << 30)
#define JTAG_FORCE_TMS			(0x1 << 29)

#define JTAG_IR_UPDATE			(0x1 << 26)	//AST2500 only
#define JTAG_INST_LEN_MASK		(0x3f << 20)
#define JTAG_SET_INST_LEN(x)		(x << 20)
#define JTAG_SET_INST_MSB		(0x1 << 19)
#define JTAG_TERMINATE_INST		(0x1 << 18)
#define JTAG_LAST_INST			(0x1 << 17)
#define JTAG_INST_EN				(0x1 << 16)
#define JTAG_DATA_LEN_MASK		(0x3f << 4)

#define JTAG_DR_UPDATE			(0x1 << 10)	//AST2500 only
#define JTAG_DATA_LEN(x)			(x << 4)
#define JTAG_SET_DATA_MSB		(0x1 << 3)
#define JTAG_TERMINATE_DATA		(0x1 << 2)
#define JTAG_LAST_DATA			(0x1 << 1)
#define JTAG_DATA_EN				(0x1)

/* AST_JTAG_ISR	- 0x0C : INterrupt status and enable */
#define JTAG_INST_PAUSE			(0x1 << 19)
#define JTAG_INST_COMPLETE		(0x1 << 18)
#define JTAG_DATA_PAUSE			(0x1 << 17)
#define JTAG_DATA_COMPLETE		(0x1 << 16)

#define JTAG_INST_PAUSE_EN		(0x1 << 3)
#define JTAG_INST_COMPLETE_EN	(0x1 << 2)
#define JTAG_DATA_PAUSE_EN		(0x1 << 1)
#define JTAG_DATA_COMPLETE_EN	(0x1)


/* AST_JTAG_SW	- 0x10 : Software Mode and Status */
#define JTAG_SW_MODE_EN			(0x1 << 19)
#define JTAG_SW_MODE_TCK		(0x1 << 18)
#define JTAG_SW_MODE_TMS		(0x1 << 17)
#define JTAG_SW_MODE_TDIO		(0x1 << 16)
//
#define JTAG_STS_INST_PAUSE		(0x1 << 2)
#define JTAG_STS_DATA_PAUSE		(0x1 << 1)
#define JTAG_STS_ENG_IDLE		(0x1)

/* AST_JTAG_TCK	- 0x14 : TCK Control */
#define JTAG_TCK_INVERSE			(0x1 << 31)
#define JTAG_TCK_DIVISOR_MASK	(0x7ff)
#define JTAG_GET_TCK_DIVISOR(x)	(x & 0x7ff)

/*  AST_JTAG_IDLE - 0x18 : Ctroller set for go to IDLE */
#define JTAG_GO_IDLE				(0x1)
/*************************************************************************************/
typedef enum jtag_xfer_mode {
	HW_MODE = 0,
	SW_MODE
} xfer_mode;

struct runtest_idle {
	xfer_mode 	mode;		//0 :HW mode, 1: SW mode
	unsigned char 	reset;		//Test Logic Reset
	unsigned char 	end;			//o: idle, 1: ir pause, 2: drpause
	unsigned char 	tck;			//keep tck
};

struct sir_xfer {
	xfer_mode 	mode;		//0 :HW mode, 1: SW mode
	unsigned short length;	//bits
	unsigned int tdi;
	unsigned int tdo;
	unsigned char endir;	//0: idle, 1:pause
};

struct sdr_xfer {
	xfer_mode 	mode;		//0 :HW mode, 1: SW mode
	unsigned char 	direct; // 0 ; read , 1 : write
	unsigned short length;	//bits
	unsigned int *tdio;
	unsigned char enddr;	//0: idle, 1:pause
};

#define JTAGIOC_BASE       'T'

#define AST_JTAG_IOCRUNTEST		_IOW(JTAGIOC_BASE, 0, struct runtest_idle)
#define AST_JTAG_IOCSIR			_IOWR(JTAGIOC_BASE, 1, struct sir_xfer)
#define AST_JTAG_IOCSDR			_IOWR(JTAGIOC_BASE, 2, struct sdr_xfer)
#define AST_JTAG_SIOCFREQ		_IOW(JTAGIOC_BASE, 3, unsigned int)
#define AST_JTAG_GIOCFREQ		_IOR(JTAGIOC_BASE, 4, unsigned int)
/******************************************************************************/
//#define AST_JTAG_DEBUG

#ifdef AST_JTAG_DEBUG
#define JTAG_DBUG(fmt, args...) printk(KERN_DEBUG "%s() " fmt,__FUNCTION__, ## args)
#else
#define JTAG_DBUG(fmt, args...)
#endif

#define JTAG_MSG(fmt, args...) printk(fmt, ## args)

struct ast_jtag_info {
	void __iomem	*reg_base;
	u8 			sts;			//0: idle, 1:irpause 2:drpause
	int 			irq;				//JTAG IRQ number
	struct reset_control *reset;
	struct clk 			*clk;
	u32					apb_clk;
	u32 			flag;
	wait_queue_head_t jtag_wq;
	bool 			is_open;
};

/*************************************************************************************/
static DEFINE_SPINLOCK(jtag_state_lock);

/******************************************************************************/
static inline u32
ast_jtag_read(struct ast_jtag_info *ast_jtag, u32 reg)
{
#if 0
	u32 val;
	val = readl(ast_jtag->reg_base + reg);
	JTAG_DBUG("reg = 0x%08x, val = 0x%08x\n", reg, val);
	return val;
#else
	return readl(ast_jtag->reg_base + reg);;
#endif
}

static inline void
ast_jtag_write(struct ast_jtag_info *ast_jtag, u32 val, u32 reg)
{
	JTAG_DBUG("reg = 0x%08x, val = 0x%08x\n", reg, val);
	writel(val, ast_jtag->reg_base + reg);
}

/******************************************************************************/
void ast_jtag_set_freq(struct ast_jtag_info *ast_jtag, unsigned int freq)
{
	u16 i;
	for (i = 0; i < 0x7ff; i++) {
//		JTAG_DBUG("[%d] : freq : %d , target : %d \n", i, ast_get_pclk()/(i + 1), freq);
		if ((ast_jtag->apb_clk / (i + 1)) <= freq)
			break;
	}
//	printk("div = %x \n", i);
	ast_jtag_write(ast_jtag, ((ast_jtag_read(ast_jtag, AST_JTAG_TCK) & ~JTAG_TCK_DIVISOR_MASK) | i),  AST_JTAG_TCK);

}

unsigned int ast_jtag_get_freq(struct ast_jtag_info *ast_jtag)
{
	return ast_jtag->apb_clk / (JTAG_GET_TCK_DIVISOR(ast_jtag_read(ast_jtag, AST_JTAG_TCK)) + 1);
}
/******************************************************************************/
void dummy(struct ast_jtag_info *ast_jtag, unsigned int cnt)
{
	int i = 0;
	for (i = 0; i < cnt; i++)
		ast_jtag_read(ast_jtag, AST_JTAG_SW);
}

static u8 TCK_Cycle(struct ast_jtag_info *ast_jtag, u8 TMS, u8 TDI)
{
	u8 tdo;

	// TCK = 0
	ast_jtag_write(ast_jtag, JTAG_SW_MODE_EN | (TMS * JTAG_SW_MODE_TMS) | (TDI * JTAG_SW_MODE_TDIO), AST_JTAG_SW);

	dummy(ast_jtag, 10);

	// TCK = 1
	ast_jtag_write(ast_jtag, JTAG_SW_MODE_EN | JTAG_SW_MODE_TCK | (TMS * JTAG_SW_MODE_TMS) | (TDI * JTAG_SW_MODE_TDIO), AST_JTAG_SW);

	if (ast_jtag_read(ast_jtag, AST_JTAG_SW) & JTAG_SW_MODE_TDIO)
		tdo = 1;
	else
		tdo = 0;

	dummy(ast_jtag, 10);

	// TCK = 0
	ast_jtag_write(ast_jtag, JTAG_SW_MODE_EN | (TMS * JTAG_SW_MODE_TMS) | (TDI * JTAG_SW_MODE_TDIO), AST_JTAG_SW);

	return tdo;
}
#if 0
unsigned int IRScan(struct JTAG_XFER *jtag_cmd)
{

	unsigned int temp;

	printf("Length: %d, Terminate: %d, Last: %d\n", jtag_cmd->length, jtag_cmd->terminate, jtag_cmd->last);

	if (jtag_cmd->length > 32) {
		printf("Length should be less than or equal to 32");
		return 0;
	} else if (jtag_cmd->length == 0) {
		printf("Length should not be 0");
		return 0;
	}

	*((unsigned int *)(jtag_cmd->taddr + 0x4)) = jtag_cmd->data;

	temp = *((unsigned int *)(jtag_cmd->taddr + 0x08));
	temp &= 0xe0000000;
	temp |= (jtag_cmd->length & 0x3f) << 20;
	temp |= (jtag_cmd->terminate & 0x1) << 18;
	temp |= (jtag_cmd->last & 0x1) << 17;
	temp |= 0x10000;

	printf("IRScan : %x\n", temp);

	*((unsigned int *)(jtag_cmd->taddr + 0x8)) = temp;

	do {
		temp = *((unsigned int *)(jtag_cmd->taddr + 0xC));
		if (jtag_cmd->last | jtag_cmd->terminate)
			temp &= 0x00040000;
		else
			temp &= 0x00080000;
	} while (temp == 0);

	temp = *((unsigned int *)(jtag_cmd->taddr + 0x4));

	temp = temp >> (32 - jtag_cmd->length);

	jtag_cmd->data = temp;
}
#endif
/******************************************************************************/
void ast_jtag_wait_instruction_pause_complete(struct ast_jtag_info *ast_jtag)
{
	wait_event_interruptible(ast_jtag->jtag_wq, (ast_jtag->flag == JTAG_INST_PAUSE));
	JTAG_DBUG("\n");
	ast_jtag->flag = 0;
}

void ast_jtag_wait_instruction_complete(struct ast_jtag_info *ast_jtag)
{
	wait_event_interruptible(ast_jtag->jtag_wq, (ast_jtag->flag == JTAG_INST_COMPLETE));
	JTAG_DBUG("\n");
	ast_jtag->flag = 0;
}

void ast_jtag_wait_data_pause_complete(struct ast_jtag_info *ast_jtag)
{
	wait_event_interruptible(ast_jtag->jtag_wq, (ast_jtag->flag == JTAG_DATA_PAUSE));
	JTAG_DBUG("\n");
	ast_jtag->flag = 0;
}

void ast_jtag_wait_data_complete(struct ast_jtag_info *ast_jtag)
{
	wait_event_interruptible(ast_jtag->jtag_wq, (ast_jtag->flag == JTAG_DATA_COMPLETE));
	JTAG_DBUG("\n");
	ast_jtag->flag = 0;
}
/******************************************************************************/
/* JTAG_reset() is to generate at least 9 TMS high and
 * 1 TMS low to force devices into Run-Test/Idle State
 */
void ast_jtag_run_test_idle(struct ast_jtag_info *ast_jtag, struct runtest_idle *runtest)
{
	int i = 0;

	JTAG_DBUG(":%s mode\n", runtest->mode ? "SW" : "HW");

	if (runtest->mode) {
		//SW mode
		//from idle , from pause,  -- > to pause, to idle

		if (runtest->reset) {
			for (i = 0; i < 10; i++) {
				TCK_Cycle(ast_jtag, 1, 0);
			}
		}

		switch (ast_jtag->sts) {
		case 0:
			if (runtest->end == 1) {
				TCK_Cycle(ast_jtag, 1, 0);	 // go to DRSCan
				TCK_Cycle(ast_jtag, 1, 0);	 // go to IRSCan
				TCK_Cycle(ast_jtag, 0, 0);	 // go to IRCap
				TCK_Cycle(ast_jtag, 1, 0);	 // go to IRExit1
				TCK_Cycle(ast_jtag, 0, 0);	 // go to IRPause
				ast_jtag->sts = 1;
			} else if (runtest->end == 2) {
				TCK_Cycle(ast_jtag, 1, 0);	 // go to DRSCan
				TCK_Cycle(ast_jtag, 0, 0);	 // go to DRCap
				TCK_Cycle(ast_jtag, 1, 0);	 // go to DRExit1
				TCK_Cycle(ast_jtag, 0, 0);	 // go to DRPause
				ast_jtag->sts = 1;
			} else {
				TCK_Cycle(ast_jtag, 0, 0);	// go to IDLE
				ast_jtag->sts = 0;
			}
			break;
		case 1:
			//from IR/DR Pause
			if (runtest->end == 1) {
				TCK_Cycle(ast_jtag, 1, 0);	// go to Exit2 IR / DR
				TCK_Cycle(ast_jtag, 1, 0);	// go to Update IR /DR
				TCK_Cycle(ast_jtag, 1, 0);	 // go to DRSCan
				TCK_Cycle(ast_jtag, 1, 0);	 // go to IRSCan
				TCK_Cycle(ast_jtag, 0, 0);	 // go to IRCap
				TCK_Cycle(ast_jtag, 1, 0);	 // go to IRExit1
				TCK_Cycle(ast_jtag, 0, 0);	 // go to IRPause
				ast_jtag->sts = 1;
			} else if (runtest->end == 2) {
				TCK_Cycle(ast_jtag, 1, 0);	// go to Exit2 IR / DR
				TCK_Cycle(ast_jtag, 1, 0);	// go to Update IR /DR
				TCK_Cycle(ast_jtag, 1, 0);	 // go to DRSCan
				TCK_Cycle(ast_jtag, 0, 0);	 // go to DRCap
				TCK_Cycle(ast_jtag, 1, 0);	 // go to DRExit1
				TCK_Cycle(ast_jtag, 0, 0);	 // go to DRPause
				ast_jtag->sts = 1;
			} else {
				TCK_Cycle(ast_jtag, 1, 0);		// go to Exit2 IR / DR
				TCK_Cycle(ast_jtag, 1, 0);		// go to Update IR /DR
				TCK_Cycle(ast_jtag, 0, 0);	// go to IDLE
				ast_jtag->sts = 0;
			}
			break;
		default:
			printk("TODO check ERROR \n");
			break;
		}

		for (i = 0; i < runtest->tck; i++)
			TCK_Cycle(ast_jtag, 0, 0);	// stay on IDLE for at lease  TCK cycle

	} else {
		ast_jtag_write(ast_jtag, 0 , AST_JTAG_SW); //dis sw mode
		mdelay(1);
		if (runtest->reset)
			ast_jtag_write(ast_jtag, JTAG_ENG_EN | JTAG_ENG_OUT_EN | JTAG_FORCE_TMS , AST_JTAG_CTRL);	// x TMS high + 1 TMS low
		else
			ast_jtag_write(ast_jtag, JTAG_GO_IDLE , AST_JTAG_IDLE);
		mdelay(1);
		ast_jtag_write(ast_jtag, JTAG_SW_MODE_EN | JTAG_SW_MODE_TDIO, AST_JTAG_SW);
		ast_jtag->sts = 0;
	}
}

int ast_jtag_sir_xfer(struct ast_jtag_info *ast_jtag, struct sir_xfer *sir)
{
	int i = 0;
	JTAG_DBUG("%s mode, ENDIR : %d, len : %d \n", sir->mode ? "SW" : "HW", sir->endir, sir->length);

	if (sir->mode) {
		if (ast_jtag->sts) {
			//from IR/DR Pause
			TCK_Cycle(ast_jtag, 1, 0);		// go to Exit2 IR / DR
			TCK_Cycle(ast_jtag, 1, 0);		// go to Update IR /DR
		}

		TCK_Cycle(ast_jtag, 1, 0);		// go to DRSCan
		TCK_Cycle(ast_jtag, 1, 0);		// go to IRSCan
		TCK_Cycle(ast_jtag, 0, 0);		// go to CapIR
		TCK_Cycle(ast_jtag, 0, 0);		// go to ShiftIR

		sir->tdo = 0;
		for (i = 0; i < sir->length; i++) {
			if (i == (sir->length - 1)) {
				sir->tdo |= TCK_Cycle(ast_jtag, 1, sir->tdi & 0x1);	// go to IRExit1
			} else {
				sir->tdo |= TCK_Cycle(ast_jtag, 0, sir->tdi & 0x1);	// go to ShiftIR
				sir->tdi >>= 1;
				sir->tdo <<= 1;
			}
		}

		TCK_Cycle(ast_jtag, 0, 0);		// go to IRPause

		//stop pause
		if (sir->endir == 0) {
			//go to idle
			TCK_Cycle(ast_jtag, 1, 0);		// go to IRExit2
			TCK_Cycle(ast_jtag, 1, 0);		// go to IRUpdate
			TCK_Cycle(ast_jtag, 0, 0);		// go to IDLE
		}
	} else {
		//HW MODE

		ast_jtag_write(ast_jtag, 0 , AST_JTAG_SW); //dis sw mode
		ast_jtag_write(ast_jtag, sir->tdi, AST_JTAG_INST);

		if (sir->endir) {
			ast_jtag_write(ast_jtag, JTAG_ENG_EN | JTAG_ENG_OUT_EN | JTAG_SET_INST_LEN(sir->length), AST_JTAG_CTRL);
			ast_jtag_write(ast_jtag, JTAG_ENG_EN | JTAG_ENG_OUT_EN | JTAG_SET_INST_LEN(sir->length) | JTAG_INST_EN, AST_JTAG_CTRL);
			ast_jtag_wait_instruction_pause_complete(ast_jtag);
		} else {
			ast_jtag_write(ast_jtag, JTAG_ENG_EN | JTAG_ENG_OUT_EN | JTAG_LAST_INST | JTAG_SET_INST_LEN(sir->length), AST_JTAG_CTRL);
			ast_jtag_write(ast_jtag, JTAG_ENG_EN | JTAG_ENG_OUT_EN | JTAG_LAST_INST | JTAG_SET_INST_LEN(sir->length) | JTAG_INST_EN, AST_JTAG_CTRL);
			ast_jtag_wait_instruction_complete(ast_jtag);
		}

		sir->tdo = ast_jtag_read(ast_jtag, AST_JTAG_INST);

#if 0
		ast_jtag_write(ast_jtag, JTAG_SW_MODE_EN | JTAG_SW_MODE_TDIO, AST_JTAG_SW);
#else
		if (sir->endir == 0) {
			ast_jtag_write(ast_jtag, JTAG_SW_MODE_EN | JTAG_SW_MODE_TDIO, AST_JTAG_SW);
		}
#endif
	}
	ast_jtag->sts = sir->endir;
	return 0;
}

int ast_jtag_sdr_xfer(struct ast_jtag_info *ast_jtag, struct sdr_xfer *sdr)
{
	unsigned int index = 0;
	u32 shift_bits = 0;
	u32 tdo = 0;
	u32 remain_xfer = sdr->length;

	JTAG_DBUG("%s mode, len : %d \n", sdr->mode ? "SW" : "HW", sdr->length);

	if (sdr->mode) {
		//SW mode
		if (ast_jtag->sts) {
			//from IR/DR Pause
			TCK_Cycle(ast_jtag, 1, 0);		// go to Exit2 IR / DR
			TCK_Cycle(ast_jtag, 1, 0);		// go to Update IR /DR
		}

		TCK_Cycle(ast_jtag, 1, 0);		// go to DRScan
		TCK_Cycle(ast_jtag, 0, 0);		// go to DRCap
		TCK_Cycle(ast_jtag, 0, 0);		// go to DRShift

		if (!sdr->direct)
			sdr->tdio[index] = 0;
		while (remain_xfer) {
			if (sdr->direct) {
				//write
				if ((shift_bits % 32) == 0)
					JTAG_DBUG("W dr->dr_data[%d]: %x\n", index, sdr->tdio[index]);

				tdo = (sdr->tdio[index] >> (shift_bits % 32)) & (0x1);
				JTAG_DBUG("%d ", tdo);
				if (remain_xfer == 1) {
					TCK_Cycle(ast_jtag, 1, tdo);	// go to DRExit1
				} else {
					TCK_Cycle(ast_jtag, 0, tdo);	// go to DRShit
				}
			} else {
				//read
				if (remain_xfer == 1) {
					tdo = TCK_Cycle(ast_jtag, 1, tdo);	// go to DRExit1
				} else {
					tdo = TCK_Cycle(ast_jtag, 0, tdo);	// go to DRShit
				}
				JTAG_DBUG("%d ", tdo);
				sdr->tdio[index] |= (tdo << (shift_bits % 32));

				if ((shift_bits % 32) == 0)
					JTAG_DBUG("R dr->dr_data[%d]: %x\n", index, sdr->tdio[index]);
			}
			shift_bits++;
			remain_xfer--;
			if ((shift_bits % 32) == 0) {
				index ++;
				sdr->tdio[index] = 0;
			}

		}

		TCK_Cycle(ast_jtag, 0, 0);		// go to DRPause

		if (sdr->enddr == 0) {
			TCK_Cycle(ast_jtag, 1, 0);		// go to DRExit2
			TCK_Cycle(ast_jtag, 1, 0);		// go to DRUpdate
			TCK_Cycle(ast_jtag, 0, 0);		// go to IDLE
		}
	} else {
		//HW MODE
		ast_jtag_write(ast_jtag, 0, AST_JTAG_SW);
		while (remain_xfer) {
			if (sdr->direct) {
				JTAG_DBUG("W dr->dr_data[%d]: %x\n", index, sdr->tdio[index]);
				ast_jtag_write(ast_jtag, sdr->tdio[index], AST_JTAG_DATA);
			} else {
				ast_jtag_write(ast_jtag, 0, AST_JTAG_DATA);
			}

			if (remain_xfer > 32) {
				shift_bits = 32;
				// read bytes were not equals to column length ==> Pause-DR
				JTAG_DBUG("shit bits %d \n", shift_bits);
				ast_jtag_write(ast_jtag,
							   JTAG_ENG_EN | JTAG_ENG_OUT_EN |
							   JTAG_DATA_LEN(shift_bits), AST_JTAG_CTRL);
				ast_jtag_write(ast_jtag,
							   JTAG_ENG_EN | JTAG_ENG_OUT_EN |
							   JTAG_DATA_LEN(shift_bits) | JTAG_DATA_EN, AST_JTAG_CTRL);
				ast_jtag_wait_data_pause_complete(ast_jtag);
			} else {
				// read bytes equals to column length => Update-DR
				shift_bits = remain_xfer;
				JTAG_DBUG("shit bits %d with last \n", shift_bits);
				if (sdr->enddr) {
					JTAG_DBUG("DR Keep Pause \n");
					ast_jtag_write(ast_jtag,
								   JTAG_ENG_EN | JTAG_ENG_OUT_EN | JTAG_DR_UPDATE |
								   JTAG_DATA_LEN(shift_bits), AST_JTAG_CTRL);
					ast_jtag_write(ast_jtag,
								   JTAG_ENG_EN | JTAG_ENG_OUT_EN | JTAG_DR_UPDATE |
								   JTAG_DATA_LEN(shift_bits) | JTAG_DATA_EN, AST_JTAG_CTRL);
					ast_jtag_wait_data_pause_complete(ast_jtag);
				} else {
					JTAG_DBUG("DR go IDLE \n");
					ast_jtag_write(ast_jtag,
								   JTAG_ENG_EN | JTAG_ENG_OUT_EN | JTAG_LAST_DATA |
								   JTAG_DATA_LEN(shift_bits), AST_JTAG_CTRL);
					ast_jtag_write(ast_jtag,
								   JTAG_ENG_EN | JTAG_ENG_OUT_EN | JTAG_LAST_DATA |
								   JTAG_DATA_LEN(shift_bits) | JTAG_DATA_EN, AST_JTAG_CTRL);
					ast_jtag_wait_data_complete(ast_jtag);
				}
			}

			if (!sdr->direct) {
				//TODO check ....
				if (shift_bits < 32)
					sdr->tdio[index] = ast_jtag_read(ast_jtag, AST_JTAG_DATA) >> (32 - shift_bits);
				else
					sdr->tdio[index] = ast_jtag_read(ast_jtag, AST_JTAG_DATA);
				JTAG_DBUG("R dr->dr_data[%d]: %x\n", index, sdr->tdio[index]);
			}

			remain_xfer = remain_xfer - shift_bits;
			index ++;
			JTAG_DBUG("remain_xfer %d\n", remain_xfer);
		}

#if 1
		ast_jtag_write(ast_jtag, JTAG_SW_MODE_EN | JTAG_SW_MODE_TDIO, AST_JTAG_SW);
#else
//		if(sdr->enddr == 0) {
		mdelay(1);
		ast_jtag_write(ast_jtag, JTAG_SW_MODE_EN | JTAG_SW_MODE_TDIO, AST_JTAG_SW);
//		}
#endif
	}

	ast_jtag->sts = sdr->enddr;
	return 0;
}

/*************************************************************************************/
static irqreturn_t ast_jtag_interrupt(int this_irq, void *dev_id)
{
	u32 status;
	struct ast_jtag_info *ast_jtag = dev_id;

	status = ast_jtag_read(ast_jtag, AST_JTAG_ISR);
	JTAG_DBUG("sts %x \n", status);

	if (status & JTAG_INST_PAUSE) {
		ast_jtag_write(ast_jtag, JTAG_INST_PAUSE | (status & 0xf), AST_JTAG_ISR);
		ast_jtag->flag = JTAG_INST_PAUSE;
	}

	if (status & JTAG_INST_COMPLETE) {
		ast_jtag_write(ast_jtag, JTAG_INST_COMPLETE | (status & 0xf), AST_JTAG_ISR);
		ast_jtag->flag = JTAG_INST_COMPLETE;
	}

	if (status & JTAG_DATA_PAUSE) {
		ast_jtag_write(ast_jtag, JTAG_DATA_PAUSE | (status & 0xf), AST_JTAG_ISR);
		ast_jtag->flag = JTAG_DATA_PAUSE;
	}

	if (status & JTAG_DATA_COMPLETE) {
		ast_jtag_write(ast_jtag, JTAG_DATA_COMPLETE | (status & 0xf), AST_JTAG_ISR);
		ast_jtag->flag = JTAG_DATA_COMPLETE;
	}

	if (ast_jtag->flag) {
		wake_up_interruptible(&ast_jtag->jtag_wq);
		return IRQ_HANDLED;
	} else {
		printk("TODO Check JTAG's interrupt %x\n", status);
		return IRQ_NONE;
	}

}

/*************************************************************************************/
struct ast_jtag_info *ast_jtag;

static long jtag_ioctl(struct file *file, unsigned int cmd,
					   unsigned long arg)
{
	int ret = 0;
	struct ast_jtag_info *ast_jtag = file->private_data;
	void __user *argp = (void __user *)arg;
	struct sir_xfer sir;
	struct sdr_xfer sdr;
	struct runtest_idle run_idle;
//	unsigned int freq;

	switch (cmd) {
	case AST_JTAG_GIOCFREQ:
		ret = __put_user(ast_jtag_get_freq(ast_jtag), (unsigned int __user *)arg);
		break;
	case AST_JTAG_SIOCFREQ:
//			printk("set freq = %d , pck %d \n",config.freq, ast_get_pclk());
		if ((unsigned int)arg > ast_jtag->apb_clk)
			ret = -EFAULT;
		else
			ast_jtag_set_freq(ast_jtag, (unsigned int)arg);

		break;
	case AST_JTAG_IOCRUNTEST:
		if (copy_from_user(&run_idle, argp, sizeof(struct runtest_idle)))
			ret = -EFAULT;
		else
			ast_jtag_run_test_idle(ast_jtag, &run_idle);
		break;
	case AST_JTAG_IOCSIR:
		if (copy_from_user(&sir, argp, sizeof(struct sir_xfer)))
			ret = -EFAULT;
		else
			ast_jtag_sir_xfer(ast_jtag, &sir);

		if (copy_to_user(argp, &sir, sizeof(struct sdr_xfer)))
			ret = -EFAULT;
		break;
	case AST_JTAG_IOCSDR:
		if (copy_from_user(&sdr, argp, sizeof(struct sdr_xfer)))
			ret = -EFAULT;
		else
			ast_jtag_sdr_xfer(ast_jtag, &sdr);

		if (copy_to_user(argp, &sdr, sizeof(struct sdr_xfer)))
			ret = -EFAULT;
		break;
	default:
		return -ENOTTY;
	}

	return ret;
}

static int jtag_open(struct inode *inode, struct file *file)
{
//	struct ast_jtag_info *drvdata;

	spin_lock(&jtag_state_lock);

//	drvdata = container_of(inode->i_cdev, struct ast_jtag_info, cdev);

	if (ast_jtag->is_open) {
		spin_unlock(&jtag_state_lock);
		return -EBUSY;
	}

	ast_jtag->is_open = true;
	file->private_data = ast_jtag;

	spin_unlock(&jtag_state_lock);

	return 0;
}

static int jtag_release(struct inode *inode, struct file *file)
{
	struct ast_jtag_info *drvdata = file->private_data;

	spin_lock(&jtag_state_lock);

	drvdata->is_open = false;

	spin_unlock(&jtag_state_lock);

	return 0;
}

static ssize_t show_tdo(struct device *dev,
						struct device_attribute *attr, char *buf)
{
	struct ast_jtag_info *ast_jtag = dev_get_drvdata(dev);

	return sprintf(buf, "%s\n", ast_jtag_read(ast_jtag, AST_JTAG_SW) & JTAG_SW_MODE_TDIO ? "1" : "0");
}

static DEVICE_ATTR(tdo, S_IRUGO, show_tdo, NULL);

static ssize_t store_tdi(struct device *dev,
						 struct device_attribute *attr, const char *buf, size_t count)
{
	u32 tdi;
	struct ast_jtag_info *ast_jtag = dev_get_drvdata(dev);

	tdi = simple_strtoul(buf, NULL, 1);

	ast_jtag_write(ast_jtag, ast_jtag_read(ast_jtag, AST_JTAG_SW) | JTAG_SW_MODE_EN | (tdi * JTAG_SW_MODE_TDIO), AST_JTAG_SW);

	return count;
}

static DEVICE_ATTR(tdi, S_IWUSR, NULL, store_tdi);

static ssize_t store_tms(struct device *dev,
						 struct device_attribute *attr, const char *buf, size_t count)
{
	u32 tms;
	struct ast_jtag_info *ast_jtag = dev_get_drvdata(dev);

	tms = simple_strtoul(buf, NULL, 1);

	ast_jtag_write(ast_jtag, ast_jtag_read(ast_jtag, AST_JTAG_SW) | JTAG_SW_MODE_EN | (tms * JTAG_SW_MODE_TMS), AST_JTAG_SW);

	return count;
}

static DEVICE_ATTR(tms, S_IWUSR, NULL, store_tms);

static ssize_t store_tck(struct device *dev,
						 struct device_attribute *attr, const char *buf, size_t count)
{
	u32 tck;
	struct ast_jtag_info *ast_jtag = dev_get_drvdata(dev);

	tck = simple_strtoul(buf, NULL, 1);

	ast_jtag_write(ast_jtag, ast_jtag_read(ast_jtag, AST_JTAG_SW) | JTAG_SW_MODE_EN | (tck * JTAG_SW_MODE_TDIO), AST_JTAG_SW);

	return count;
}

static DEVICE_ATTR(tck, S_IWUSR, NULL, store_tck);

static ssize_t show_sts(struct device *dev,
						struct device_attribute *attr, char *buf)
{
	struct ast_jtag_info *ast_jtag = dev_get_drvdata(dev);

	return sprintf(buf, "%s\n", ast_jtag->sts ? "Pause" : "Idle");
}

static DEVICE_ATTR(sts, S_IRUGO, show_sts, NULL);

static ssize_t show_frequency(struct device *dev,
							  struct device_attribute *attr, char *buf)
{
	struct ast_jtag_info *ast_jtag = dev_get_drvdata(dev);
//	printk("PCLK = %d \n", ast_get_pclk());
//	printk("DIV  = %d \n", JTAG_GET_TCK_DIVISOR(ast_jtag_read(ast_jtag, AST_JTAG_TCK)) + 1);
	return sprintf(buf, "Frequency : %d\n", ast_jtag->apb_clk / (JTAG_GET_TCK_DIVISOR(ast_jtag_read(ast_jtag, AST_JTAG_TCK)) + 1));
}

static ssize_t store_frequency(struct device *dev,
							   struct device_attribute *attr, const char *buf, size_t count)
{
	u32 val;
	struct ast_jtag_info *ast_jtag = dev_get_drvdata(dev);

	val = simple_strtoul(buf, NULL, 20);
	ast_jtag_set_freq(ast_jtag, val);

	return count;
}

static DEVICE_ATTR(freq, S_IRUGO | S_IWUSR, show_frequency, store_frequency);

static struct attribute *jtag_sysfs_entries[] = {
	&dev_attr_freq.attr,
	&dev_attr_sts.attr,
	&dev_attr_tck.attr,
	&dev_attr_tms.attr,
	&dev_attr_tdi.attr,
	&dev_attr_tdo.attr,
	NULL
};

static struct attribute_group jtag_attribute_group = {
	.attrs = jtag_sysfs_entries,
};

static const struct file_operations ast_jtag_fops = {
	.owner		= THIS_MODULE,
	.unlocked_ioctl	= jtag_ioctl,
	.open		= jtag_open,
	.release		= jtag_release,
};

struct miscdevice ast_jtag_misc = {
	.minor 	= MISC_DYNAMIC_MINOR,
	.name 	= "ast-jtag",
	.fops 	= &ast_jtag_fops,
};

static int ast_jtag_probe(struct platform_device *pdev)
{
	struct resource *res;
	int ret = 0;

	JTAG_DBUG("ast_jtag_probe\n");

	if (!(ast_jtag = devm_kzalloc(&pdev->dev, sizeof(struct ast_jtag_info), GFP_KERNEL))) {
		return -ENOMEM;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (NULL == res) {
		dev_err(&pdev->dev, "cannot get IORESOURCE_MEM\n");
		ret = -ENOENT;
		goto out;
	}

	ast_jtag->reg_base = devm_ioremap_resource(&pdev->dev, res);
	if (!ast_jtag->reg_base) {
		ret = -EIO;
		goto out_region;
	}

	ast_jtag->irq = platform_get_irq(pdev, 0);
	if (ast_jtag->irq < 0) {
		dev_err(&pdev->dev, "no irq specified\n");
		ret = -ENOENT;
		goto out_region;
	}

	ast_jtag->reset = devm_reset_control_get_exclusive(&pdev->dev, "jtag");
	if (IS_ERR(ast_jtag->reset)) {
		dev_err(&pdev->dev, "can't get jtag reset\n");
		return PTR_ERR(ast_jtag->reset);
	}

	ast_jtag->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(ast_jtag->clk)) {
		dev_err(&pdev->dev, "no clock defined\n");
		return -ENODEV;
	}
	ast_jtag->apb_clk = clk_get_rate(ast_jtag->clk);

	//scu init
	reset_control_assert(ast_jtag->reset);
	udelay(3);
	reset_control_deassert(ast_jtag->reset);

	ast_jtag_write(ast_jtag, JTAG_ENG_EN | JTAG_ENG_OUT_EN , AST_JTAG_CTRL); //Eanble Clock
	ast_jtag_write(ast_jtag, JTAG_SW_MODE_EN | JTAG_SW_MODE_TDIO, AST_JTAG_SW);

	ret = devm_request_irq(&pdev->dev, ast_jtag->irq, ast_jtag_interrupt,
						   0, dev_name(&pdev->dev), ast_jtag);
	if (ret) {
		printk("JTAG Unable to get IRQ");
		goto out_region;
	}

	ast_jtag_write(ast_jtag, JTAG_INST_PAUSE | JTAG_INST_COMPLETE |
				   JTAG_DATA_PAUSE | JTAG_DATA_COMPLETE |
				   JTAG_INST_PAUSE_EN | JTAG_INST_COMPLETE_EN |
				   JTAG_DATA_PAUSE_EN | JTAG_DATA_COMPLETE_EN,
				   AST_JTAG_ISR);		//Eanble Interrupt

	ast_jtag->flag = 0;
	init_waitqueue_head(&ast_jtag->jtag_wq);

	ret = misc_register(&ast_jtag_misc);
	if (ret) {
		printk(KERN_ERR "JTAG : failed to request interrupt\n");
		goto out_irq;
	}

	platform_set_drvdata(pdev, ast_jtag);
	dev_set_drvdata(ast_jtag_misc.this_device, ast_jtag);

	ret = sysfs_create_group(&pdev->dev.kobj, &jtag_attribute_group);
	if (ret) {
		printk(KERN_ERR "ast_jtag: failed to create sysfs device attributes.\n");
		return -1;
	}

	printk(KERN_INFO "ast_jtag: driver successfully loaded.\n");

	return 0;

out_irq:
	free_irq(ast_jtag->irq, NULL);
out_region:
	release_mem_region(res->start, res->end - res->start + 1);
out:
	printk(KERN_WARNING "ast_jtag: driver init failed (ret=%d)!\n", ret);
	return ret;
}

static int ast_jtag_remove(struct platform_device *pdev)
{
	struct resource *res;
	struct ast_jtag_info *ast_jtag = platform_get_drvdata(pdev);

	JTAG_DBUG("ast_jtag_remove\n");

	sysfs_remove_group(&pdev->dev.kobj, &jtag_attribute_group);

	misc_deregister(&ast_jtag_misc);

	free_irq(ast_jtag->irq, ast_jtag);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	iounmap(ast_jtag->reg_base);

	platform_set_drvdata(pdev, NULL);

	release_mem_region(res->start, res->end - res->start + 1);

	return 0;
}

#ifdef CONFIG_PM
static int
ast_jtag_suspend(struct platform_device *pdev, pm_message_t state)
{
	return 0;
}

static int
ast_jtag_resume(struct platform_device *pdev)
{
	return 0;
}
#endif

static const struct of_device_id ast_jtag_of_matches[] = {
	{ .compatible = "aspeed,ast-jtag", },
	{},
};
MODULE_DEVICE_TABLE(of, ast_jtag_of_matches);

static struct platform_driver ast_jtag_driver = {
	.probe 		= ast_jtag_probe,
	.remove 		= ast_jtag_remove,
#ifdef CONFIG_PM
	.suspend        = ast_jtag_suspend,
	.resume         = ast_jtag_resume,
#endif
	.driver         = {
		.name   = KBUILD_MODNAME,
		.of_match_table = ast_jtag_of_matches,
	},
};

module_platform_driver(ast_jtag_driver);

MODULE_AUTHOR("Ryan Chen <ryan_chen@aspeedtech.com>");
MODULE_DESCRIPTION("AST JTAG LIB Driver");
MODULE_LICENSE("GPL");
