/* SPDX-License-Identifier: GPL-2.0
 * Copyright (C) 2019 Hewlett-Packard Development Company, L.P.
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

// Gilbert Chen <gilbert.chen@hpe.com>
#ifndef _UMAC_H_
#define _UMAC_H_

#define UMAC_CONFIG_STATUS      0x00        /* R/W Configuration and Status
					     * Register I
					     */
#define UMAC_CFG_PROMISC        0x00008000  // promiscuous mode
#define UMAC_CFG_FE_LOOPBK      0x00004000  // front end loopback mode
#define UMAC_CFG_ACCEPT         0x00002000  // accept own broadcast
#define UMAC_CFG_TXEN           0x00001000  // transmit enable
#define UMAC_CFG_RXEN           0x00000800  // receive enable
#define UMAC_CFG_GTX_CLK_EN     0x00000400  // gigabit clock enable
#define UMAC_CFG_TX_CLK_EN      0x00000200  // 10/100 clock enable
#define UMAC_CFG_KEEP_CORRUPT   0x00000100  // keep corrupted packets
#define UMAC_STS_RX_PKT_MISSED  0x00000080  // no more rx buffers available
#define UMAC_STS_MAC_RX_BUSY    0x00000020  // rx channel receiving a packet
#define UMAC_STS_RX_FLOODED     0x00000010  // rx fifo reached high water mark
#define UMAC_CFG_GIGABIT_MODE   0x00000004  // MAC gigabit mode enable
#define UMAC_CFG_FIFO_LOOPBK    0x00000002  // FIFO loopback mode
#define UMAC_CFG_FULL_DUPLEX    0x00000001  // enable ignoring of collisions
#define UMAC_RING_PTR           0x04        //  R/W Ring Pointer Register
#define UMAC_TX_RING_PTR_MASK   0x7FFF0000  // transmit ring entry pointer
#define UMAC_TX_RING_PTR_SHIFT  16
#define UMAC_RX_RING_PTR_MASK   0x00007FFF  // receive ring entry pointer
#define UMAC_RX_RING_PTR_SHIFT  0

#define UMAC_RING_PROMPT      0x08	//  W	Ring Prompt Register
#define UMAC_CLEAR_STATUS     0x0C	//  W	Clear Status Register
#define UMAC_CKSUM_CONFIG     0x10	//  R/W	Checksum Config Register
#define UMAC_RING_SIZE        0x14	//  R/W	Ring Size Register
#define UMAC_TX_RING_SIZE_MASK  0xFF000000
#define UMAC_TX_RING_SIZE_SHIFT 24
#define UMAC_RX_RING_SIZE_MASK  0x00FF0000
#define UMAC_RX_RING_SIZE_SHIFT 16
#define UMAC_LAST_RX_PKT_SIZE   0x0000FFFF
// ring size values
#define UMAC_RING_SIZE_4        0x00
#define UMAC_RING_SIZE_8        0x01
#define UMAC_RING_SIZE_16       0x03
#define UMAC_RING_SIZE_32       0x07
#define UMAC_RING_SIZE_64       0x0F
#define UMAC_RING_SIZE_128      0x1F
#define UMAC_RING_SIZE_256      0x3F

#define UMAC_MAC_ADDR_HI      0x18	//  R/W	MAC Address[47:32] Register
#define UMAC_MAC_ADDR_MID     0x1C	//  R/W	MAC Address[31:16] Register
#define UMAC_MAC_ADDR_LO      0x20	//  R/W	MAC Address[15:0] Register
#define UMAC_MC_ADDR_FILT_HI  0x24	//  R/W	LAF[63:32] Register
#define UMAC_MC_ADDR_FILT_LO  0x28	//  R/W	LAF[31:0] Register
#define UMAC_CONFIG_STATUS2   0x2C	/*  R/W	Configuration and Status
					 *  Register II *
					 */
#define UMAC_INTERRUPT        0x30	//  R/W	MAC Interrupt Configuration
					// and Status Register
#define UMAC_RX_INT_OFLOW       0x00000080
#define UMAC_TX_INT_OFLOW       0x00000040
#define UMAC_OVERRRUN_INT       0x00000010
#define UMAC_RX_INTEN           0x00000008
#define UMAC_RX_INT             0x00000004
#define UMAC_TX_INTEN           0x00000002
#define UMAC_TX_INT             0x00000001

#define UMAC_OVERRUN_COUNT    0x34	//  R/W	Overrun Counter Register
#define UMAC_RX_INT_CONFIG    0x38	//  R/W	Rx Interrupt Config Register
#define UMAC_TX_INT_CONFIG    0x3C	//  R/W	Tx Interrupt Config Register
#define UMAC_PACKET_LENGTH    0x40	//  R/W	Packet Length Register
#define UMAC_BCAST_FILTER     0x44	//  R/W	Broadcast Filter Config Register
#define UMAC_BCAST_PROMPT     0x48	//  W	Broadcast Prompt Register
#define UMAC_RX_RING_ADDR     0x4C	//  R/W	Rx Ring Base Address Register
#define UMAC_TX_RING_ADDR     0x50	//  R/W	Tx Ring Base Address Register
#define UMAC_DMA_CONFIG       0x54	//  R/W	DMA Config Register
#define UMAC_BURST_CONFIG     0x58	//  R/W	Bursting Config Register
#define UMAC_PAUSE_CONFIG     0x5C	//  R/W	PAUSE Frame Config Register
#define UMAC_PAUSE_CONTROL    0x60	/*  R/W	PAUSE Frame Control and
					 *  Status Register
					 */
#define UMAC_CONGESTN_CONFIG  0x64	/*  R/W	Channel Congestion Config
					 *  Register
					 */

#define UMAC_MAX_TX_DESC_ENTRIES	0x100	/* 256,number of ring buffer
						 *  entries supported
						 */
#define UMAC_MAX_RX_DESC_ENTRIES	0x100
#define UMAC_MAX_TX_FRAME_SIZE		0x600
#define UMAC_MAX_RX_FRAME_SIZE		0x600

// ring status masks
#define UMAC_RING_ENTRY_HW_OWN		0x8000

// rx ring status masks
#define UMAC_RING_RX_FRAME_ERR      0x4000
#define UMAC_RING_RX_CRC_ERR        0x2000
#define UMAC_RING_RX_LEN_ERR        0x1000
#define UMAC_RING_RX_OVERRUN        0x0800
#define UMAC_RING_RX_PKT_TYPE_MASK  0x0700
#define UMAC_RING_RX_MII_ERR        0x0080
#define UMAC_RING_RX_CAR_EXT_ERR    0x0040
#define UMAC_RING_RX_BAD_PAUSE_ERR  0x0020
#define UMAC_RING_RX_ERR_MASK       0x38E0 /* frame err not included
					    * since it's "ok"
					    */

// maximum ethernet frame size
#define UMAC_MIN_FRAME_SIZE       60    // excludes preable, sfd, and fcs
#define UMAC_MAX_PAYLOAD_SIZE     1500
#define UMAC_MAX_FRAME_SIZE       1514  // excludes preable, sfd, and fcs
#define UMAC_MAX_PACKET_ROUNDED   0x600 // 1536, nicely aligned
#define MAX_PKT_SIZE		  1518

struct umac_rx_desc_entry {
	u32  dmaaddress;   // Start address for DMA operationg
	u16  status;       // Packet tx status and ownership flag
	u16  count;        // Number of bytes received
	u16  checksum;     // On-the-fly packet checksum
	u16  control;      // Checksum-in-time flag
	u32  reserved;
} __aligned(16);

struct umac_rx_descs {
	struct umac_rx_desc_entry entrylist[UMAC_MAX_RX_DESC_ENTRIES];
	u8 framelist[UMAC_MAX_RX_DESC_ENTRIES][UMAC_MAX_RX_FRAME_SIZE];
} __packed;

struct umac_tx_desc_entry {
	u32  dmaaddress;   // Start address for DMA operationg
	u16  status;       // Packet rx status, type, and ownership flag
	u16  count;        // Number of bytes received
	u32  cksumoffset;  // Specifies where to place packet checksum
	u32  reserved;
} __aligned(16);

struct umac_tx_descs {
	struct umac_tx_desc_entry entrylist[UMAC_MAX_TX_DESC_ENTRIES];
	u8 framelist[UMAC_MAX_TX_DESC_ENTRIES][UMAC_MAX_TX_FRAME_SIZE];
} __packed;

#endif
