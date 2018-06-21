/*
 * JTAG driver for the Aspeed SoC
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
/******************************************************************************/
#define ASPEED_JTAG_DATA		0x00
#define ASPEED_JTAG_INST		0x04
#define ASPEED_JTAG_CTRL		0x08
#define ASPEED_JTAG_ISR			0x0C
#define ASPEED_JTAG_SW			0x10
#define ASPEED_JTAG_TCK			0x14
#define ASPEED_JTAG_IDLE		0x18

/* ASPEED_JTAG_CTRL - 0x08 : Engine Control */
#define JTAG_ENG_EN			(0x1 << 31)
#define JTAG_ENG_OUT_EN			(0x1 << 30)
#define JTAG_FORCE_TMS			(0x1 << 29)

#define JTAG_IR_UPDATE			(0x1 << 26)	//AST2500 only
#define JTAG_INST_LEN_MASK		(0x3f << 20)
#define JTAG_SET_INST_LEN(x)		(x << 20)
#define JTAG_SET_INST_MSB		(0x1 << 19)
#define JTAG_TERMINATE_INST		(0x1 << 18)
#define JTAG_LAST_INST			(0x1 << 17)
#define JTAG_INST_EN			(0x1 << 16)
#define JTAG_DATA_LEN_MASK		(0x3f << 4)

#define JTAG_DR_UPDATE			(0x1 << 10)	//AST2500 only
#define JTAG_DATA_LEN(x)		(x << 4)
#define JTAG_SET_DATA_MSB		(0x1 << 3)
#define JTAG_TERMINATE_DATA		(0x1 << 2)
#define JTAG_LAST_DATA			(0x1 << 1)
#define JTAG_DATA_EN			(0x1)

/* ASPEED_JTAG_ISR	- 0x0C : INterrupt status and enable */
#define JTAG_INST_PAUSE			(0x1 << 19)
#define JTAG_INST_COMPLETE		(0x1 << 18)
#define JTAG_DATA_PAUSE			(0x1 << 17)
#define JTAG_DATA_COMPLETE		(0x1 << 16)

#define JTAG_INST_PAUSE_EN		(0x1 << 3)
#define JTAG_INST_COMPLETE_EN		(0x1 << 2)
#define JTAG_DATA_PAUSE_EN		(0x1 << 1)
#define JTAG_DATA_COMPLETE_EN		(0x1)

/* ASPEED_JTAG_SW	- 0x10 : Software Mode and Status */
#define JTAG_SW_MODE_EN			(0x1 << 19)
#define JTAG_SW_MODE_TCK		(0x1 << 18)
#define JTAG_SW_MODE_TMS		(0x1 << 17)
#define JTAG_SW_MODE_TDIO		(0x1 << 16)
//
#define JTAG_STS_INST_PAUSE		(0x1 << 2)
#define JTAG_STS_DATA_PAUSE		(0x1 << 1)
#define JTAG_STS_ENG_IDLE		(0x1)

/* ASPEED_JTAG_TCK	- 0x14 : TCK Control */
#define JTAG_TCK_INVERSE		(0x1 << 31)
#define JTAG_TCK_DIVISOR_MASK		(0x7ff)
#define JTAG_GET_TCK_DIVISOR(x)		(x & 0x7ff)

/*  ASPEED_JTAG_IDLE - 0x18 : Ctroller set for go to IDLE */
#define JTAG_CTRL_TRSTn_HIGH		(0x1 << 31)
#define JTAG_GO_IDLE			(0x1)
/******************************************************************************/
typedef enum jtag_xfer_mode {
	HW_MODE = 0,
	SW_MODE
} xfer_mode;

struct runtest_idle {
	xfer_mode	mode;	//0 :HW mode, 1: SW mode
	unsigned char	reset;	//Test Logic Reset
	unsigned char	end;	//o: idle, 1: ir pause, 2: drpause
	unsigned char	tck;	//keep tck
};

struct sir_xfer {
	xfer_mode	mode;	//0 :HW mode, 1: SW mode
	unsigned short	length;	//bits
	unsigned int	*tdi;
	unsigned int	*tdo;
	unsigned char	endir;	//0: idle, 1:pause
};

struct sdr_xfer {
	xfer_mode	mode;	//0 :HW mode, 1: SW mode
	unsigned char	direct; // 0 ; read , 1 : write
	unsigned short	length;	//bits
	unsigned int	*tdio;
	unsigned char	enddr;	//0: idle, 1:pause
};

struct io_xfer {
	xfer_mode	mode;		//0 :HW mode, 1: SW mode
	unsigned long	Address;
	unsigned long	Data;
};

struct trst_reset {
	unsigned long	operation; // 0 ; read , 1 : write
	unsigned long	Data;	 // 0 means low, 1 means high - TRST pin
};

#define JTAGIOC_BASE	'T'

#define ASPEED_JTAG_IOCRUNTEST	_IOW(JTAGIOC_BASE, 0, struct runtest_idle)
#define ASPEED_JTAG_IOCSIR	_IOWR(JTAGIOC_BASE, 1, struct sir_xfer)
#define ASPEED_JTAG_IOCSDR	_IOWR(JTAGIOC_BASE, 2, struct sdr_xfer)
#define ASPEED_JTAG_SIOCFREQ	_IOW(JTAGIOC_BASE, 3, unsigned int)
#define ASPEED_JTAG_GIOCFREQ	_IOR(JTAGIOC_BASE, 4, unsigned int)
#define ASPEED_JTAG_IOWRITE	_IOW(JTAGIOC_BASE, 5, struct io_xfer)
#define ASPEED_JTAG_IOREAD	_IOR(JTAGIOC_BASE, 6, struct io_xfer)
#define ASPEED_JTAG_RESET	_IOW(JTAGIOC_BASE, 7, struct io_xfer)
#define ASPEED_JTAG_TRST_RESET	_IOW(JTAGIOC_BASE, 8, struct trst_reset)
#define ASPEED_JTAG_RUNTCK	_IOW(JTAGIOC_BASE, 12, struct io_xfer)
/******************************************************************************/
#define ASPEED_JTAG_DEBUG

#ifdef ASPEED_JTAG_DEBUG
#define JTAG_DBUG(fmt, args...) printk(KERN_DEBUG "%s() " fmt, __FUNCTION__, ## args)
#else
#define JTAG_DBUG(fmt, args...)
#endif

#define JTAG_MSG(fmt, args...) printk(fmt, ## args)

struct aspeed_jtag_info {
	void __iomem		*reg_base;
	u8			sts;	//0: idle, 1:irpause 2:drpause
	int			irq;		//JTAG IRQ number
	struct reset_control	*reset;
	struct clk		*clk;
	u32			apb_clk;
	u32			flag;
	wait_queue_head_t	jtag_wq;
	bool			is_open;
};

/******************************************************************************/
static DEFINE_SPINLOCK(jtag_state_lock);

/******************************************************************************/
static inline u32
aspeed_jtag_read(struct aspeed_jtag_info *aspeed_jtag, u32 reg)
{
#if 0
	u32 val;

	val = readl(aspeed_jtag->reg_base + reg);
	JTAG_DBUG("reg = 0x%08x, val = 0x%08x\n", reg, val);
	return val;
#else
	return readl(aspeed_jtag->reg_base + reg);
#endif
}

static inline void
aspeed_jtag_write(struct aspeed_jtag_info *aspeed_jtag, u32 val, u32 reg)
{
	JTAG_DBUG("reg = 0x%08x, val = 0x%08x\n", reg, val);
	writel(val, aspeed_jtag->reg_base + reg);
}

/******************************************************************************/
void aspeed_jtag_set_freq(struct aspeed_jtag_info *aspeed_jtag, unsigned int freq)
{
	u16 i;

	for (i = 0; i < 0x7ff; i++) {
//		JTAG_DBUG("[%d] : freq : %d , target : %d \n", i, aspeed_get_pclk()/(i + 1), freq);
		if ((aspeed_jtag->apb_clk / (i + 1)) <= freq)
			break;
	}
//	printk("div = %x \n", i);
	aspeed_jtag_write(aspeed_jtag, ((aspeed_jtag_read(aspeed_jtag, ASPEED_JTAG_TCK) & ~JTAG_TCK_DIVISOR_MASK) | i),  ASPEED_JTAG_TCK);

}

unsigned int aspeed_jtag_get_freq(struct aspeed_jtag_info *aspeed_jtag)
{
	return aspeed_jtag->apb_clk / (JTAG_GET_TCK_DIVISOR(aspeed_jtag_read(aspeed_jtag, ASPEED_JTAG_TCK)) + 1);
}
/******************************************************************************/
void dummy(struct aspeed_jtag_info *aspeed_jtag, unsigned int cnt)
{
	int i = 0;

	for (i = 0; i < cnt; i++)
		aspeed_jtag_read(aspeed_jtag, ASPEED_JTAG_SW);
}

static u8 TCK_Cycle(struct aspeed_jtag_info *aspeed_jtag, u8 TMS, u8 TDI)
{
	u8 tdo;

	// TCK = 0
	aspeed_jtag_write(aspeed_jtag, JTAG_SW_MODE_EN | (TMS * JTAG_SW_MODE_TMS) | (TDI * JTAG_SW_MODE_TDIO), ASPEED_JTAG_SW);

	dummy(aspeed_jtag, 10);

	// TCK = 1
	aspeed_jtag_write(aspeed_jtag, JTAG_SW_MODE_EN | JTAG_SW_MODE_TCK | (TMS * JTAG_SW_MODE_TMS) | (TDI * JTAG_SW_MODE_TDIO), ASPEED_JTAG_SW);

	if (aspeed_jtag_read(aspeed_jtag, ASPEED_JTAG_SW) & JTAG_SW_MODE_TDIO)
		tdo = 1;
	else
		tdo = 0;

	dummy(aspeed_jtag, 10);

	// TCK = 0
	aspeed_jtag_write(aspeed_jtag, JTAG_SW_MODE_EN | (TMS * JTAG_SW_MODE_TMS) | (TDI * JTAG_SW_MODE_TDIO), ASPEED_JTAG_SW);

	return tdo;
}

/******************************************************************************/
void aspeed_jtag_wait_instruction_pause_complete(struct aspeed_jtag_info *aspeed_jtag)
{
	wait_event_interruptible(aspeed_jtag->jtag_wq, (aspeed_jtag->flag == JTAG_INST_PAUSE));
	JTAG_DBUG("\n");
	aspeed_jtag->flag = 0;
}

void aspeed_jtag_wait_instruction_complete(struct aspeed_jtag_info *aspeed_jtag)
{
	wait_event_interruptible(aspeed_jtag->jtag_wq, (aspeed_jtag->flag == JTAG_INST_COMPLETE));
	JTAG_DBUG("\n");
	aspeed_jtag->flag = 0;
}

void aspeed_jtag_wait_data_pause_complete(struct aspeed_jtag_info *aspeed_jtag)
{
	wait_event_interruptible(aspeed_jtag->jtag_wq, (aspeed_jtag->flag == JTAG_DATA_PAUSE));
	JTAG_DBUG("\n");
	aspeed_jtag->flag = 0;
}

void aspeed_jtag_wait_data_complete(struct aspeed_jtag_info *aspeed_jtag)
{
	wait_event_interruptible(aspeed_jtag->jtag_wq, (aspeed_jtag->flag == JTAG_DATA_COMPLETE));
	JTAG_DBUG("\n");
	aspeed_jtag->flag = 0;
}
/******************************************************************************/
/* JTAG_reset() is to generate at leaspeed 9 TMS high and
 * 1 TMS low to force devices into Run-Test/Idle State
 */
void aspeed_jtag_run_test_idle(struct aspeed_jtag_info *aspeed_jtag, struct runtest_idle *runtest)
{
	int i = 0;

	JTAG_DBUG(":%s mode\n", runtest->mode ? "SW" : "HW");

	if (runtest->mode) {
		//SW mode
		//from idle , from pause,  -- > to pause, to idle

		if (runtest->reset) {
			for (i = 0; i < 10; i++)
				TCK_Cycle(aspeed_jtag, 1, 0);
		}

		switch (aspeed_jtag->sts) {
		case 0:
			if (runtest->end == 1) {
				TCK_Cycle(aspeed_jtag, 1, 0);	// go to DRSCan
				TCK_Cycle(aspeed_jtag, 1, 0);	// go to IRSCan
				TCK_Cycle(aspeed_jtag, 0, 0);	// go to IRCap
				TCK_Cycle(aspeed_jtag, 1, 0);	// go to IRExit1
				TCK_Cycle(aspeed_jtag, 0, 0);	// go to IRPause
				aspeed_jtag->sts = 1;
			} else if (runtest->end == 2) {
				TCK_Cycle(aspeed_jtag, 1, 0);	// go to DRSCan
				TCK_Cycle(aspeed_jtag, 0, 0);	// go to DRCap
				TCK_Cycle(aspeed_jtag, 1, 0);	// go to DRExit1
				TCK_Cycle(aspeed_jtag, 0, 0);	// go to DRPause
				aspeed_jtag->sts = 1;
			} else {
				TCK_Cycle(aspeed_jtag, 0, 0);	// go to IDLE
				aspeed_jtag->sts = 0;
			}
			break;
		case 1:
			//from IR/DR Pause
			if (runtest->end == 1) {
				TCK_Cycle(aspeed_jtag, 1, 0);	// go to Exit2 IR / DR
				TCK_Cycle(aspeed_jtag, 1, 0);	// go to Update IR /DR
				TCK_Cycle(aspeed_jtag, 1, 0);	// go to DRSCan
				TCK_Cycle(aspeed_jtag, 1, 0);	// go to IRSCan
				TCK_Cycle(aspeed_jtag, 0, 0);	// go to IRCap
				TCK_Cycle(aspeed_jtag, 1, 0);	// go to IRExit1
				TCK_Cycle(aspeed_jtag, 0, 0);	// go to IRPause
				aspeed_jtag->sts = 1;
			} else if (runtest->end == 2) {
				TCK_Cycle(aspeed_jtag, 1, 0);	// go to Exit2 IR / DR
				TCK_Cycle(aspeed_jtag, 1, 0);	// go to Update IR /DR
				TCK_Cycle(aspeed_jtag, 1, 0);	// go to DRSCan
				TCK_Cycle(aspeed_jtag, 0, 0);	// go to DRCap
				TCK_Cycle(aspeed_jtag, 1, 0);	// go to DRExit1
				TCK_Cycle(aspeed_jtag, 0, 0);	// go to DRPause
				aspeed_jtag->sts = 1;
			} else {
				TCK_Cycle(aspeed_jtag, 1, 0);	// go to Exit2 IR / DR
				TCK_Cycle(aspeed_jtag, 1, 0);	// go to Update IR /DR
				TCK_Cycle(aspeed_jtag, 0, 0);	// go to IDLE
				aspeed_jtag->sts = 0;
			}
			break;
		default:
			printk("TODO check ERROR \n");
			break;
		}

		for (i = 0; i < runtest->tck; i++)
			TCK_Cycle(aspeed_jtag, 0, 0);	// stay on IDLE for at lease  TCK cycle

	} else {
		aspeed_jtag_write(aspeed_jtag, 0, ASPEED_JTAG_SW);  //dis sw mode
		mdelay(1);
		if (runtest->reset)
			aspeed_jtag_write(aspeed_jtag, JTAG_ENG_EN | JTAG_ENG_OUT_EN | JTAG_FORCE_TMS, ASPEED_JTAG_CTRL);	// x TMS high + 1 TMS low
		else
			aspeed_jtag_write(aspeed_jtag, JTAG_GO_IDLE, ASPEED_JTAG_IDLE);
		mdelay(1);
		aspeed_jtag_write(aspeed_jtag, JTAG_SW_MODE_EN | JTAG_SW_MODE_TDIO, ASPEED_JTAG_SW);
		aspeed_jtag->sts = 0;
	}
}

int aspeed_jtag_sir_xfer(struct aspeed_jtag_info *aspeed_jtag, struct sir_xfer *sir)
{
	unsigned int index = 0;
	u32 shift_bits = 0;
	u32 tdi = 0;
	u32 remain_xfer = sir->length;

	JTAG_DBUG("%s mode, ENDIR : %d, len : %d \n", sir->mode ? "SW" : "HW", sir->endir, sir->length);

	if (sir->mode) {
		if (aspeed_jtag->sts) {
			//from IR/DR Pause
			TCK_Cycle(aspeed_jtag, 1, 0);	// go to Exit2 IR / DR
			TCK_Cycle(aspeed_jtag, 1, 0);	// go to Update IR /DR
		}

		TCK_Cycle(aspeed_jtag, 1, 0);		// go to DRSCan
		TCK_Cycle(aspeed_jtag, 1, 0);		// go to IRSCan
		TCK_Cycle(aspeed_jtag, 0, 0);		// go to IRCap
		TCK_Cycle(aspeed_jtag, 0, 0);		// go to IRShift

		sir->tdo[index] = 0;
		while (remain_xfer) {
			tdi = (sir->tdi[index]) >> (shift_bits % 32) & (0x1);
			if (remain_xfer == 1) {
				sir->tdo[index] |= TCK_Cycle(aspeed_jtag, 1, tdi);
			} else {
				sir->tdo[index] |= TCK_Cycle(aspeed_jtag, 0, tdi);
				sir->tdo[index] <<= 1;
			}
			shift_bits++;
			remain_xfer--;
			if ((shift_bits % 32) == 0) {
				index++;
				sir->tdo[index] = 0;
			}
		}

		TCK_Cycle(aspeed_jtag, 0, 0);		// go to IRPause

		//stop pause
		if (sir->endir == 0) {
			//go to idle
			TCK_Cycle(aspeed_jtag, 1, 0);		// go to IRExit2
			TCK_Cycle(aspeed_jtag, 1, 0);		// go to IRUpdate
			TCK_Cycle(aspeed_jtag, 0, 0);		// go to IDLE
		}
	} else {
		//HW MODE

		aspeed_jtag_write(aspeed_jtag, 0, ASPEED_JTAG_SW);  //dis sw mode

		while (remain_xfer) {
			aspeed_jtag_write(aspeed_jtag, sir->tdi[index], ASPEED_JTAG_INST);
			if (remain_xfer > 32) {
				shift_bits = 32;
				aspeed_jtag_write(aspeed_jtag,
					       JTAG_ENG_EN | JTAG_ENG_OUT_EN |
					       JTAG_SET_INST_LEN(shift_bits),
					       ASPEED_JTAG_CTRL);
				aspeed_jtag_write(aspeed_jtag,
					       JTAG_ENG_EN | JTAG_ENG_OUT_EN |
					       JTAG_SET_INST_LEN(shift_bits) |
					       JTAG_INST_EN, ASPEED_JTAG_CTRL);
				aspeed_jtag_wait_instruction_pause_complete(aspeed_jtag);
			} else {
				shift_bits = remain_xfer;
				if (sir->endir) {
					aspeed_jtag_write(aspeed_jtag,
						       JTAG_ENG_EN | JTAG_ENG_OUT_EN |
						       JTAG_SET_INST_LEN(shift_bits),
						       ASPEED_JTAG_CTRL);
					aspeed_jtag_write(aspeed_jtag,
						       JTAG_ENG_EN | JTAG_ENG_OUT_EN |
						       JTAG_SET_INST_LEN(shift_bits) |
						       JTAG_INST_EN, ASPEED_JTAG_CTRL);
					aspeed_jtag_wait_instruction_pause_complete(aspeed_jtag);
				} else {
					aspeed_jtag_write(aspeed_jtag,
						       JTAG_ENG_EN | JTAG_ENG_OUT_EN |
						       JTAG_LAST_INST | JTAG_SET_INST_LEN(shift_bits),
						       ASPEED_JTAG_CTRL);
					aspeed_jtag_write(aspeed_jtag,
						       JTAG_ENG_EN | JTAG_ENG_OUT_EN |
						       JTAG_LAST_INST | JTAG_SET_INST_LEN(shift_bits) |
						       JTAG_INST_EN, ASPEED_JTAG_CTRL);
					aspeed_jtag_wait_instruction_complete(aspeed_jtag);
				}
			}
			if (shift_bits < 32)
				sir->tdo[index] = aspeed_jtag_read(aspeed_jtag, ASPEED_JTAG_INST) >> (32 - shift_bits);
			else
				sir->tdo[index] = aspeed_jtag_read(aspeed_jtag, ASPEED_JTAG_INST);
			remain_xfer = remain_xfer - shift_bits;
			index++;
		}

		// sir->tdo = aspeed_jtag_read(aspeed_jtag, ASPEED_JTAG_INST);

#if 0
		aspeed_jtag_write(aspeed_jtag, JTAG_SW_MODE_EN | JTAG_SW_MODE_TDIO, ASPEED_JTAG_SW);
#else
		if (sir->endir == 0) {
			aspeed_jtag_write(aspeed_jtag, JTAG_SW_MODE_EN | JTAG_SW_MODE_TDIO, ASPEED_JTAG_SW);
		}
#endif
	}
	aspeed_jtag->sts = sir->endir;
	return 0;
}

int aspeed_jtag_sdr_xfer(struct aspeed_jtag_info *aspeed_jtag, struct sdr_xfer *sdr)
{
	unsigned int index = 0;
	u32 shift_bits = 0;
	u32 tdo = 0;
	u32 remain_xfer = sdr->length;

	JTAG_DBUG("%s mode, len : %d \n", sdr->mode ? "SW" : "HW", sdr->length);

	if (sdr->mode) {
		//SW mode
		if (aspeed_jtag->sts) {
			//from IR/DR Pause
			TCK_Cycle(aspeed_jtag, 1, 0);		// go to Exit2 IR / DR
			TCK_Cycle(aspeed_jtag, 1, 0);		// go to Update IR /DR
		}

		TCK_Cycle(aspeed_jtag, 1, 0);		// go to DRScan
		TCK_Cycle(aspeed_jtag, 0, 0);		// go to DRCap
		TCK_Cycle(aspeed_jtag, 0, 0);		// go to DRShift

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
					TCK_Cycle(aspeed_jtag, 1, tdo);	// go to DRExit1
				} else {
					TCK_Cycle(aspeed_jtag, 0, tdo);	// go to DRShit
				}
			} else {
				//read
				if (remain_xfer == 1) {
					tdo = TCK_Cycle(aspeed_jtag, 1, tdo);	// go to DRExit1
				} else {
					tdo = TCK_Cycle(aspeed_jtag, 0, tdo);	// go to DRShit
				}
				JTAG_DBUG("%d ", tdo);
				sdr->tdio[index] |= (tdo << (shift_bits % 32));

				if ((shift_bits % 32) == 0)
					JTAG_DBUG("R dr->dr_data[%d]: %x\n", index, sdr->tdio[index]);
			}
			shift_bits++;
			remain_xfer--;
			if ((shift_bits % 32) == 0) {
				index++;
				sdr->tdio[index] = 0;
			}

		}

		TCK_Cycle(aspeed_jtag, 0, 0);		// go to DRPause

		if (sdr->enddr == 0) {
			TCK_Cycle(aspeed_jtag, 1, 0);		// go to DRExit2
			TCK_Cycle(aspeed_jtag, 1, 0);		// go to DRUpdate
			TCK_Cycle(aspeed_jtag, 0, 0);		// go to IDLE
		}
	} else {
		//HW MODE
		aspeed_jtag_write(aspeed_jtag, 0, ASPEED_JTAG_SW);
		while (remain_xfer) {
			if (sdr->direct) {
				JTAG_DBUG("W dr->dr_data[%d]: %x\n", index, sdr->tdio[index]);
				aspeed_jtag_write(aspeed_jtag, sdr->tdio[index], ASPEED_JTAG_DATA);
			} else {
				aspeed_jtag_write(aspeed_jtag, 0, ASPEED_JTAG_DATA);
			}

			if (remain_xfer > 32) {
				shift_bits = 32;
				// read bytes were not equals to column length ==> Pause-DR
				JTAG_DBUG("shit bits %d \n", shift_bits);
				aspeed_jtag_write(aspeed_jtag,
					       JTAG_ENG_EN | JTAG_ENG_OUT_EN |
					       JTAG_DATA_LEN(shift_bits), ASPEED_JTAG_CTRL);
				aspeed_jtag_write(aspeed_jtag,
					       JTAG_ENG_EN | JTAG_ENG_OUT_EN |
					       JTAG_DATA_LEN(shift_bits) | JTAG_DATA_EN, ASPEED_JTAG_CTRL);
				aspeed_jtag_wait_data_pause_complete(aspeed_jtag);
			} else {
				// read bytes equals to column length => Update-DR
				shift_bits = remain_xfer;
				JTAG_DBUG("shit bits %d with last \n", shift_bits);
				if (sdr->enddr) {
					JTAG_DBUG("DR Keep Pause \n");
					aspeed_jtag_write(aspeed_jtag,
						       JTAG_ENG_EN | JTAG_ENG_OUT_EN |
						       JTAG_DATA_LEN(shift_bits), ASPEED_JTAG_CTRL);
					aspeed_jtag_write(aspeed_jtag,
						       JTAG_ENG_EN | JTAG_ENG_OUT_EN |
						       JTAG_DATA_LEN(shift_bits) | JTAG_DATA_EN, ASPEED_JTAG_CTRL);
					aspeed_jtag_wait_data_pause_complete(aspeed_jtag);
				} else {
					JTAG_DBUG("DR go IDLE \n");
					aspeed_jtag_write(aspeed_jtag,
						       JTAG_ENG_EN | JTAG_ENG_OUT_EN | JTAG_LAST_DATA |
						       JTAG_DATA_LEN(shift_bits), ASPEED_JTAG_CTRL);
					aspeed_jtag_write(aspeed_jtag,
						       JTAG_ENG_EN | JTAG_ENG_OUT_EN | JTAG_LAST_DATA |
						       JTAG_DATA_LEN(shift_bits) | JTAG_DATA_EN, ASPEED_JTAG_CTRL);
					aspeed_jtag_wait_data_complete(aspeed_jtag);
				}
			}

			if (!sdr->direct) {
				//TODO check ....
				if (shift_bits < 32)
					sdr->tdio[index] = aspeed_jtag_read(aspeed_jtag, ASPEED_JTAG_DATA) >> (32 - shift_bits);
				else
					sdr->tdio[index] = aspeed_jtag_read(aspeed_jtag, ASPEED_JTAG_DATA);
				JTAG_DBUG("R dr->dr_data[%d]: %x\n", index, sdr->tdio[index]);
			}

			remain_xfer = remain_xfer - shift_bits;
			index++;
			JTAG_DBUG("remain_xfer %d\n", remain_xfer);
		}

#if 0
		mdelay(1);
		aspeed_jtag_write(aspeed_jtag, JTAG_SW_MODE_EN | JTAG_SW_MODE_TDIO, ASPEED_JTAG_SW);
#else
		if(!sdr->enddr) {
			mdelay(1);
			aspeed_jtag_write(aspeed_jtag, JTAG_SW_MODE_EN | JTAG_SW_MODE_TDIO, ASPEED_JTAG_SW);
		}
#endif
	}

	aspeed_jtag->sts = sdr->enddr;
	return 0;
}

/*************************************************************************************/
static irqreturn_t aspeed_jtag_interrupt(int this_irq, void *dev_id)
{
	u32 status;
	struct aspeed_jtag_info *aspeed_jtag = dev_id;

	status = aspeed_jtag_read(aspeed_jtag, ASPEED_JTAG_ISR);
	JTAG_DBUG("sts %x \n", status);

	if (status & JTAG_INST_PAUSE) {
		aspeed_jtag_write(aspeed_jtag, JTAG_INST_PAUSE | (status & 0xf), ASPEED_JTAG_ISR);
		aspeed_jtag->flag = JTAG_INST_PAUSE;
	}

	if (status & JTAG_INST_COMPLETE) {
		aspeed_jtag_write(aspeed_jtag, JTAG_INST_COMPLETE | (status & 0xf), ASPEED_JTAG_ISR);
		aspeed_jtag->flag = JTAG_INST_COMPLETE;
	}

	if (status & JTAG_DATA_PAUSE) {
		aspeed_jtag_write(aspeed_jtag, JTAG_DATA_PAUSE | (status & 0xf), ASPEED_JTAG_ISR);
		aspeed_jtag->flag = JTAG_DATA_PAUSE;
	}

	if (status & JTAG_DATA_COMPLETE) {
		aspeed_jtag_write(aspeed_jtag, JTAG_DATA_COMPLETE | (status & 0xf), ASPEED_JTAG_ISR);
		aspeed_jtag->flag = JTAG_DATA_COMPLETE;
	}

	if (aspeed_jtag->flag) {
		wake_up_interruptible(&aspeed_jtag->jtag_wq);
		return IRQ_HANDLED;
	} else {
		printk("TODO Check JTAG's interrupt %x\n", status);
		return IRQ_NONE;
	}

}

void JTAG_reset(struct aspeed_jtag_info *aspeed_jtag)
{
	aspeed_jtag_write(aspeed_jtag, 0, ASPEED_JTAG_SW);
	aspeed_jtag_write(aspeed_jtag, JTAG_ENG_EN | JTAG_ENG_OUT_EN, ASPEED_JTAG_CTRL);
	aspeed_jtag_write(aspeed_jtag, JTAG_ENG_EN | JTAG_ENG_OUT_EN | JTAG_FORCE_TMS, ASPEED_JTAG_CTRL);
	mdelay(5);
	while (1) {
		if (aspeed_jtag_read(aspeed_jtag, ASPEED_JTAG_CTRL) & JTAG_FORCE_TMS)
			break;
	}
	aspeed_jtag_write(aspeed_jtag, JTAG_SW_MODE_EN | JTAG_SW_MODE_TDIO, ASPEED_JTAG_SW);
}

/*************************************************************************************/
struct aspeed_jtag_info *aspeed_jtag;

static long jtag_ioctl(struct file *file, unsigned int cmd,
		       unsigned long arg)
{
	int ret = 0;
	struct aspeed_jtag_info *aspeed_jtag = file->private_data;
	void __user *argp = (void __user *)arg;
	struct sir_xfer sir;
	struct sdr_xfer sdr;
	struct io_xfer io;
	struct trst_reset trst_pin;
	struct runtest_idle run_idle;

	switch (cmd) {
	case ASPEED_JTAG_GIOCFREQ:
		ret = __put_user(aspeed_jtag_get_freq(aspeed_jtag), (unsigned int __user *)arg);
		break;
	case ASPEED_JTAG_SIOCFREQ:
//			printk("set freq = %d , pck %d \n",config.freq, aspeed_get_pclk());
		if ((unsigned int)arg > aspeed_jtag->apb_clk)
			ret = -EFAULT;
		else
			aspeed_jtag_set_freq(aspeed_jtag, (unsigned int)arg);

		break;
	case ASPEED_JTAG_IOCRUNTEST:
		if (copy_from_user(&run_idle, argp, sizeof(struct runtest_idle)))
			ret = -EFAULT;
		else
			aspeed_jtag_run_test_idle(aspeed_jtag, &run_idle);
		break;
	case ASPEED_JTAG_IOCSIR:
		if (copy_from_user(&sir, argp, sizeof(struct sir_xfer))) {
			ret = -EFAULT;
		} else {
			unsigned int *kerneltdo = 0;
			unsigned int *kerneltdi = 0;
			unsigned int usertdi = (unsigned int)sir.tdi;
			unsigned int usertdo = (unsigned int)sir.tdo;

			kerneltdi = vmalloc((((sir.length + 7) >> 3) + 4));
			// if (kerneltdi == NULL) {
			// 	printk("ASPEED_JTAG : Can't allocate memory\n");
			// 	return -ENOMEM;
			// }
			kerneltdo = vmalloc((((sir.length + 7) >> 3) + 4));
			// if (kerneltdo == NULL) {
			// 	printk("ASPEED_JTAG : Can't allocate memory\n");
			// 	vfree(kerneltdi);
			// 	return -ENOMEM;
			// }

			memset(kerneltdi, 0, (((sir.length + 7) >> 3) + 4));
			memset(kerneltdo, 0, (((sir.length + 7) >> 3) + 4));

			if (copy_from_user(kerneltdi, sir.tdi, ((sir.length + 7) >> 3))) {
				ret = -EFAULT;
			} else {
				sir.tdi = kerneltdi;
				sir.tdo = kerneltdo;
				aspeed_jtag_sir_xfer(aspeed_jtag, &sir);
				sir.tdo = (unsigned int *)usertdo;
				sir.tdi = (unsigned int *)usertdi;
				if (copy_to_user(sir.tdo, kerneltdo, ((sir.length + 7) >> 3)))
					ret = -EFAULT;
				if (copy_to_user(argp, &sir, sizeof(struct sir_xfer)))
					ret = -EFAULT;
			}
			vfree(kerneltdi);
			vfree(kerneltdo);
		}
		break;
	case ASPEED_JTAG_IOCSDR:
		if (copy_from_user(&sdr, argp, sizeof(struct sdr_xfer))) {
			ret = -EFAULT;
		} else {
			unsigned int useraddress = (unsigned int)sdr.tdio;
			unsigned int *kernelAddess = vmalloc((((sdr.length + 7) >> 3) + 4));

			if (kernelAddess == NULL) {
				printk("ASPEED_JTAG : Can't allocate memory\n");
				return -ENOMEM;
			}
			memset(kernelAddess, 0, (((sdr.length + 7) >> 3) + 4));
			if (copy_from_user(kernelAddess, sdr.tdio, ((sdr.length + 7) >> 3))) {
				ret = -EFAULT;
			} else {
				sdr.tdio = (unsigned int *)kernelAddess;
				aspeed_jtag_sdr_xfer(aspeed_jtag, &sdr);
				sdr.tdio = (unsigned int *)useraddress;
				if (copy_to_user(sdr.tdio, kernelAddess, ((sdr.length + 7) >> 3)))
					ret = -EFAULT;
				if (copy_to_user(argp, &sdr, sizeof(struct sdr_xfer)))
					ret = -EFAULT;
			}
			vfree(kernelAddess);
		}
		break;
	case ASPEED_JTAG_IOWRITE:
		if (copy_from_user(&io, argp, sizeof(struct io_xfer))) {
			ret = -EFAULT;
		} else {
			void __iomem	*reg_add;

			reg_add = ioremap(io.Address, 4);
			writel(io.Data, reg_add);
			iounmap(reg_add);
		}

		break;
	case ASPEED_JTAG_IOREAD:
		if (copy_from_user(&io, argp, sizeof(struct io_xfer))) {
			ret = -EFAULT;
		} else {
			void __iomem	*reg_add;

			reg_add = ioremap(io.Address, 4);
			io.Data = readl(reg_add);
			iounmap(reg_add);
		}
		if (copy_to_user(argp, &io, sizeof(struct io_xfer)))
			ret = -EFAULT;
		break;
	case ASPEED_JTAG_RESET:
		JTAG_reset(aspeed_jtag);
		break;
	case ASPEED_JTAG_RUNTCK:
		if (copy_from_user(&io, argp, sizeof(struct io_xfer))) {
			ret = -EFAULT;
		} else {
			int i;

			for (i = 0; i < io.Address; i++)
				TCK_Cycle(aspeed_jtag, io.mode, io.Data);
		}
		break;
	case ASPEED_JTAG_TRST_RESET:
		if (copy_from_user(&trst_pin, argp, sizeof(struct trst_reset))) {
			ret = -EFAULT;
		} else {
			unsigned int regs = aspeed_jtag_read(aspeed_jtag, ASPEED_JTAG_IDLE);

			if (trst_pin.operation == 1) {
				if (trst_pin.Data == 1)
					aspeed_jtag_write(aspeed_jtag, regs | (1 << 31), ASPEED_JTAG_IDLE);
				else
					aspeed_jtag_write(aspeed_jtag, regs & (~(1 << 31)), ASPEED_JTAG_IDLE);
			} else
				trst_pin.Data = (regs >> 31);

		}
		if (copy_to_user(argp, &trst_pin, sizeof(struct trst_reset)))
			ret = -EFAULT;
		break;
	default:
		return -ENOTTY;
	}

	return ret;
}

static int jtag_open(struct inode *inode, struct file *file)
{
//	struct aspeed_jtag_info *drvdata;

	spin_lock(&jtag_state_lock);

//	drvdata = container_of(inode->i_cdev, struct aspeed_jtag_info, cdev);

	if (aspeed_jtag->is_open) {
		spin_unlock(&jtag_state_lock);
		return -EBUSY;
	}

	aspeed_jtag->is_open = true;
	file->private_data = aspeed_jtag;

	spin_unlock(&jtag_state_lock);

	return 0;
}

static int jtag_release(struct inode *inode, struct file *file)
{
	struct aspeed_jtag_info *drvdata = file->private_data;

	spin_lock(&jtag_state_lock);

	drvdata->is_open = false;

	spin_unlock(&jtag_state_lock);

	return 0;
}

static ssize_t show_tdo(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct aspeed_jtag_info *aspeed_jtag = dev_get_drvdata(dev);

	return sprintf(buf, "%s\n", aspeed_jtag_read(aspeed_jtag, ASPEED_JTAG_SW) & JTAG_SW_MODE_TDIO ? "1" : "0");
}

static DEVICE_ATTR(tdo, S_IRUGO, show_tdo, NULL);

static ssize_t store_tdi(struct device *dev,
			 struct device_attribute *attr, const char *buf, size_t count)
{
	u32 tdi;
	struct aspeed_jtag_info *aspeed_jtag = dev_get_drvdata(dev);

	tdi = simple_strtoul(buf, NULL, 1);

	aspeed_jtag_write(aspeed_jtag, aspeed_jtag_read(aspeed_jtag, ASPEED_JTAG_SW) | JTAG_SW_MODE_EN | (tdi * JTAG_SW_MODE_TDIO), ASPEED_JTAG_SW);

	return count;
}

static DEVICE_ATTR(tdi, S_IWUSR, NULL, store_tdi);

static ssize_t store_tms(struct device *dev,
			 struct device_attribute *attr, const char *buf, size_t count)
{
	u32 tms;
	struct aspeed_jtag_info *aspeed_jtag = dev_get_drvdata(dev);

	tms = simple_strtoul(buf, NULL, 1);

	aspeed_jtag_write(aspeed_jtag, aspeed_jtag_read(aspeed_jtag, ASPEED_JTAG_SW) | JTAG_SW_MODE_EN | (tms * JTAG_SW_MODE_TMS), ASPEED_JTAG_SW);

	return count;
}

static DEVICE_ATTR(tms, S_IWUSR, NULL, store_tms);

static ssize_t store_tck(struct device *dev,
			 struct device_attribute *attr, const char *buf, size_t count)
{
	u32 tck;
	struct aspeed_jtag_info *aspeed_jtag = dev_get_drvdata(dev);

	tck = simple_strtoul(buf, NULL, 1);

	aspeed_jtag_write(aspeed_jtag, aspeed_jtag_read(aspeed_jtag, ASPEED_JTAG_SW) | JTAG_SW_MODE_EN | (tck * JTAG_SW_MODE_TDIO), ASPEED_JTAG_SW);

	return count;
}

static DEVICE_ATTR(tck, S_IWUSR, NULL, store_tck);

static ssize_t show_sts(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct aspeed_jtag_info *aspeed_jtag = dev_get_drvdata(dev);

	return sprintf(buf, "%s\n", aspeed_jtag->sts ? "Pause" : "Idle");
}

static DEVICE_ATTR(sts, S_IRUGO, show_sts, NULL);

static ssize_t show_frequency(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	struct aspeed_jtag_info *aspeed_jtag = dev_get_drvdata(dev);
//	printk("PCLK = %d \n", aspeed_get_pclk());
//	printk("DIV  = %d \n", JTAG_GET_TCK_DIVISOR(aspeed_jtag_read(aspeed_jtag, ASPEED_JTAG_TCK)) + 1);
	return sprintf(buf, "Frequency : %d\n", aspeed_jtag->apb_clk / (JTAG_GET_TCK_DIVISOR(aspeed_jtag_read(aspeed_jtag, ASPEED_JTAG_TCK)) + 1));
}

static ssize_t store_frequency(struct device *dev,
			       struct device_attribute *attr, const char *buf, size_t count)
{
	u32 val;
	struct aspeed_jtag_info *aspeed_jtag = dev_get_drvdata(dev);

	val = simple_strtoul(buf, NULL, 20);
	aspeed_jtag_set_freq(aspeed_jtag, val);

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

static const struct file_operations aspeed_jtag_fops = {
	.owner		= THIS_MODULE,
	.unlocked_ioctl	= jtag_ioctl,
	.open		= jtag_open,
	.release		= jtag_release,
};

struct miscdevice aspeed_jtag_misc = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= "aspeed-jtag",
	.fops	= &aspeed_jtag_fops,
};

static int aspeed_jtag_probe(struct platform_device *pdev)
{
	struct resource *res;
	int ret = 0;

	JTAG_DBUG("aspeed_jtag_probe\n");

	aspeed_jtag = devm_kzalloc(&pdev->dev, sizeof(struct aspeed_jtag_info), GFP_KERNEL);
	if (!aspeed_jtag)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "cannot get IORESOURCE_MEM\n");
		ret = -ENOENT;
		goto out;
	}

	aspeed_jtag->reg_base = devm_ioremap_resource(&pdev->dev, res);
	if (!aspeed_jtag->reg_base) {
		ret = -EIO;
		goto out_region;
	}

	aspeed_jtag->irq = platform_get_irq(pdev, 0);
	if (aspeed_jtag->irq < 0) {
		dev_err(&pdev->dev, "no irq specified\n");
		ret = -ENOENT;
		goto out_region;
	}

	aspeed_jtag->reset = devm_reset_control_get_exclusive(&pdev->dev, "jtag");
	if (IS_ERR(aspeed_jtag->reset)) {
		dev_err(&pdev->dev, "can't get jtag reset\n");
		return PTR_ERR(aspeed_jtag->reset);
	}

	aspeed_jtag->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(aspeed_jtag->clk)) {
		dev_err(&pdev->dev, "no clock defined\n");
		return -ENODEV;
	}
	aspeed_jtag->apb_clk = clk_get_rate(aspeed_jtag->clk);

	//scu init
	reset_control_assert(aspeed_jtag->reset);
	udelay(3);
	reset_control_deassert(aspeed_jtag->reset);

	aspeed_jtag_write(aspeed_jtag, JTAG_ENG_EN | JTAG_ENG_OUT_EN, ASPEED_JTAG_CTRL);  //Eanble Clock
	//Enable sw mode for disable clk
	aspeed_jtag_write(aspeed_jtag, JTAG_SW_MODE_EN | JTAG_SW_MODE_TDIO, ASPEED_JTAG_SW);

	ret = devm_request_irq(&pdev->dev, aspeed_jtag->irq, aspeed_jtag_interrupt,
			       0, dev_name(&pdev->dev), aspeed_jtag);
	if (ret) {
		printk("JTAG Unable to get IRQ");
		goto out_region;
	}

	aspeed_jtag_write(aspeed_jtag, JTAG_INST_PAUSE | JTAG_INST_COMPLETE |
		       JTAG_DATA_PAUSE | JTAG_DATA_COMPLETE |
		       JTAG_INST_PAUSE_EN | JTAG_INST_COMPLETE_EN |
		       JTAG_DATA_PAUSE_EN | JTAG_DATA_COMPLETE_EN,
		       ASPEED_JTAG_ISR);		//Eanble Interrupt

	aspeed_jtag->flag = 0;
	init_waitqueue_head(&aspeed_jtag->jtag_wq);

	ret = misc_register(&aspeed_jtag_misc);
	if (ret) {
		printk(KERN_ERR "JTAG : failed to request interrupt\n");
		goto out_irq;
	}

	platform_set_drvdata(pdev, aspeed_jtag);
	dev_set_drvdata(aspeed_jtag_misc.this_device, aspeed_jtag);

	ret = sysfs_create_group(&pdev->dev.kobj, &jtag_attribute_group);
	if (ret) {
		printk(KERN_ERR "aspeed_jtag: failed to create sysfs device attributes.\n");
		return -1;
	}

	printk(KERN_INFO "aspeed_jtag: driver successfully loaded.\n");

	return 0;

out_irq:
	free_irq(aspeed_jtag->irq, NULL);
out_region:
	release_mem_region(res->start, res->end - res->start + 1);
out:
	printk(KERN_WARNING "aspeed_jtag: driver init failed (ret=%d)!\n", ret);
	return ret;
}

static int aspeed_jtag_remove(struct platform_device *pdev)
{
	struct resource *res;
	struct aspeed_jtag_info *aspeed_jtag = platform_get_drvdata(pdev);

	JTAG_DBUG("aspeed_jtag_remove\n");

	sysfs_remove_group(&pdev->dev.kobj, &jtag_attribute_group);

	misc_deregister(&aspeed_jtag_misc);

	free_irq(aspeed_jtag->irq, aspeed_jtag);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	iounmap(aspeed_jtag->reg_base);

	platform_set_drvdata(pdev, NULL);

	release_mem_region(res->start, res->end - res->start + 1);

	return 0;
}

#ifdef CONFIG_PM
static int
aspeed_jtag_suspend(struct platform_device *pdev, pm_message_t state)
{
	return 0;
}

static int
aspeed_jtag_resume(struct platform_device *pdev)
{
	return 0;
}
#endif

static const struct of_device_id aspeed_jtag_of_matches[] = {
	{ .compatible = "aspeed,aspeed-jtag", },
	{},
};
MODULE_DEVICE_TABLE(of, aspeed_jtag_of_matches);

static struct platform_driver aspeed_jtag_driver = {
	.probe		= aspeed_jtag_probe,
	.remove		= aspeed_jtag_remove,
#ifdef CONFIG_PM
	.suspend	= aspeed_jtag_suspend,
	.resume		= aspeed_jtag_resume,
#endif
	.driver		= {
		.name	= KBUILD_MODNAME,
		.of_match_table = aspeed_jtag_of_matches,
	},
};

module_platform_driver(aspeed_jtag_driver);

MODULE_AUTHOR("Ryan Chen <ryan_chen@aspeedtech.com>");
MODULE_DESCRIPTION("AST JTAG LIB Driver");
MODULE_LICENSE("GPL");
