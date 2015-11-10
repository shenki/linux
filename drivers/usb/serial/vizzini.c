/*
 * Based on vizzini driver:
 *
 * Copyright (c) 2013 Exar Corporation, Inc.
 *
 * Based on USB Serial "Simple" driver
 *
 * Copyright (C) 2001-2006,2008,2013 Greg Kroah-Hartman <greg@kroah.com>
 * Copyright (C) 2005 Arthur Huillet (ahuillet@users.sf.net)
 * Copyright (C) 2005 Thomas Hergenhahn <thomas.hergenhahn@suse.de>
 * Copyright (C) 2009 Outpost Embedded, LLC
 * Copyright (C) 2010 Zilogic Systems <code@zilogic.com>
 * Copyright (C) 2013 Wei Shuai <cpuwolf@gmail.com>
 * Copyright (C) 2013 Linux Foundation
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License version
 *	2 as published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/tty.h>
#include <linux/module.h>
#include <linux/usb.h>
#include <linux/usb/serial.h>

#define XR_SET_REG			0

#define URM_REG_BLOCK			4
#define EPLOCALS_REG_BLOCK		0x66

#define MEM_EP_LOCALS_SIZE_S		3
#define MEM_EP_LOCALS_SIZE		(1 << MEM_EP_LOCALS_SIZE_S)

#define EP_WIDE_MODE			0x03

#define UART_GPIO_MODE			0x01a

#define UART_GPIO_MODE_SEL_M		0x7
#define UART_GPIO_MODE_SEL_S		0
#define UART_GPIO_MODE_SEL		0x007

#define UART_GPIO_MODE_SEL_GPIO		(0x0 << UART_GPIO_MODE_SEL_S)
#define UART_GPIO_MODE_SEL_RTS_CTS	(0x1 << UART_GPIO_MODE_SEL_S)
#define UART_GPIO_MODE_SEL_DTR_DSR	(0x2 << UART_GPIO_MODE_SEL_S)

#define UART_ENABLE			0x003
#define UART_ENABLE_TX_M		0x1
#define UART_ENABLE_TX_S		0
#define UART_ENABLE_TX			0x001
#define UART_ENABLE_RX_M		0x1
#define UART_ENABLE_RX_S		1
#define UART_ENABLE_RX			0x002

#define UART_CLOCK_DIVISOR_0		0x004
#define UART_CLOCK_DIVISOR_1		0x005
#define UART_CLOCK_DIVISOR_2		0x006

#define UART_TX_CLOCK_MASK_0		0x007
#define UART_TX_CLOCK_MASK_1		0x008

#define UART_RX_CLOCK_MASK_0		0x009
#define UART_RX_CLOCK_MASK_1		0x00a

#define UART_FORMAT			0x00b

#define UART_FORMAT_SIZE_M		0xf
#define UART_FORMAT_SIZE_S		0
#define UART_FORMAT_SIZE		0x00f

#define UART_FORMAT_SIZE_7		(0x7 << UART_FORMAT_SIZE_S)
#define UART_FORMAT_SIZE_8		(0x8 << UART_FORMAT_SIZE_S)
#define UART_FORMAT_SIZE_9		(0x9 << UART_FORMAT_SIZE_S)

#define UART_FORMAT_PARITY_M		0x7
#define UART_FORMAT_PARITY_S		4
#define UART_FORMAT_PARITY		0x070

#define UART_FORMAT_PARITY_NONE		(0x0 << UART_FORMAT_PARITY_S)
#define UART_FORMAT_PARITY_ODD		(0x1 << UART_FORMAT_PARITY_S)
#define UART_FORMAT_PARITY_EVEN		(0x2 << UART_FORMAT_PARITY_S)
#define UART_FORMAT_PARITY_1		(0x3 << UART_FORMAT_PARITY_S)
#define UART_FORMAT_PARITY_0		(0x4 << UART_FORMAT_PARITY_S)

#define UART_FORMAT_STOP_M		0x1
#define UART_FORMAT_STOP_S		7
#define UART_FORMAT_STOP		0x080

#define UART_FORMAT_STOP_1		(0x0 << UART_FORMAT_STOP_S)
#define UART_FORMAT_STOP_2		(0x1 << UART_FORMAT_STOP_S)

#define UART_FLOW			0x00c

#define UART_FLOW_MODE_M		0x7
#define UART_FLOW_MODE_S		0
#define UART_FLOW_MODE			0x007

#define UART_FLOW_MODE_NONE		(0x0 << UART_FLOW_MODE_S)
#define UART_FLOW_MODE_HW		(0x1 << UART_FLOW_MODE_S)
#define UART_FLOW_MODE_SW		(0x2 << UART_FLOW_MODE_S)

#define UART_XON_CHAR			0x010
#define UART_XOFF_CHAR			0x011

#define URM_ENABLE_BASE			0x010
#define URM_ENABLE_0			0x010
#define URM_ENABLE_0_TX			0x001
#define URM_ENABLE_0_RX			0x002

struct vizzini_baud_rate
{
	unsigned int tx;
	unsigned int rx0;
	unsigned int rx1;
};

static struct vizzini_baud_rate vizzini_baud_rates[] = {
	{ 0x000, 0x000, 0x000 },
	{ 0x000, 0x000, 0x000 },
	{ 0x100, 0x000, 0x100 },
	{ 0x020, 0x400, 0x020 },
	{ 0x010, 0x100, 0x010 },
	{ 0x208, 0x040, 0x208 },
	{ 0x104, 0x820, 0x108 },
	{ 0x844, 0x210, 0x884 },
	{ 0x444, 0x110, 0x444 },
	{ 0x122, 0x888, 0x224 },
	{ 0x912, 0x448, 0x924 },
	{ 0x492, 0x248, 0x492 },
	{ 0x252, 0x928, 0x292 },
	{ 0X94A, 0X4A4, 0XA52 },
	{ 0X52A, 0XAA4, 0X54A },
	{ 0XAAA, 0x954, 0X4AA },
	{ 0XAAA, 0x554, 0XAAA },
	{ 0x555, 0XAD4, 0X5AA },
	{ 0XB55, 0XAB4, 0X55A },
	{ 0X6B5, 0X5AC, 0XB56 },
	{ 0X5B5, 0XD6C, 0X6D6 },
	{ 0XB6D, 0XB6A, 0XDB6 },
	{ 0X76D, 0X6DA, 0XBB6 },
	{ 0XEDD, 0XDDA, 0X76E },
	{ 0XDDD, 0XBBA, 0XEEE },
	{ 0X7BB, 0XF7A, 0XDDE },
	{ 0XF7B, 0XEF6, 0X7DE },
	{ 0XDF7, 0XBF6, 0XF7E },
	{ 0X7F7, 0XFEE, 0XEFE },
	{ 0XFDF, 0XFBE, 0X7FE },
	{ 0XF7F, 0XEFE, 0XFFE },
	{ 0XFFF, 0XFFE, 0XFFD },
};

static int vizzini_set_reg(struct usb_serial_port *port,
                           unsigned int block, unsigned int regnum,
			   unsigned int value)
{
	dev_dbg(&port->serial->dev->dev, "%s 0x%02x:0x%02x = 0x%02x\n",
		__func__, block, regnum, value);

        return usb_control_msg(port->serial->dev,
			       usb_sndctrlpipe(port->serial->dev, 0),
			       XR_SET_REG,
			       USB_DIR_OUT | USB_TYPE_VENDOR,
			       value, regnum | (block << 8),
			       NULL, 0,
			       5000);
}

static void vizzini_disable(struct usb_serial_port *port)
{
	unsigned int block = port->bulk_out_endpointAddress - 1;

        vizzini_set_reg(port, block, UART_ENABLE, 0);
        vizzini_set_reg(port, URM_REG_BLOCK, URM_ENABLE_BASE + block, 0);
}

static void vizzini_enable(struct usb_serial_port *port)
{
	unsigned int block = port->bulk_out_endpointAddress - 1;

        vizzini_set_reg(port, URM_REG_BLOCK, URM_ENABLE_BASE + block,
			URM_ENABLE_0_TX);
        vizzini_set_reg(port, block, UART_ENABLE,
			UART_ENABLE_TX | UART_ENABLE_RX);
        vizzini_set_reg(port, URM_REG_BLOCK, URM_ENABLE_BASE + block,
			URM_ENABLE_0_TX | URM_ENABLE_0_RX);
}

static void vizzini_set_baud_rate(struct usb_serial_port *port,
				  unsigned int rate)
{
	int 		block 	= port->bulk_out_endpointAddress - 1;
	unsigned int 	divisor = 48000000 / rate;
	unsigned int 	i 	= ((32 * 48000000) / rate) & 0x1f;
	unsigned int 	tx_mask = vizzini_baud_rates[i].tx;
	unsigned int 	rx_mask = ((divisor & 1) ? vizzini_baud_rates[i].rx1 :
				   vizzini_baud_rates[i].rx0);

	dev_dbg(&port->serial->dev->dev,
		"Setting baud rate to %d: i=%u div=%u tx=%03x rx=%03x\n",
		rate, i, divisor, tx_mask, rx_mask);

	vizzini_set_reg(port, block, UART_CLOCK_DIVISOR_0,
			(divisor >>  0) & 0xff);
	vizzini_set_reg(port, block, UART_CLOCK_DIVISOR_1,
			(divisor >>  8) & 0xff);
	vizzini_set_reg(port, block, UART_CLOCK_DIVISOR_2,
			(divisor >> 16) & 0xff);

	vizzini_set_reg(port, block, UART_TX_CLOCK_MASK_0,
			(tx_mask >>  0) & 0xff);
	vizzini_set_reg(port, block, UART_TX_CLOCK_MASK_1,
			(tx_mask >>  8) & 0xff);

	vizzini_set_reg(port, block, UART_RX_CLOCK_MASK_0,
			(rx_mask >>  0) & 0xff);
	vizzini_set_reg(port, block, UART_RX_CLOCK_MASK_1,
			(rx_mask >>  8) & 0xff);
}

static void vizzini_set_termios(struct tty_struct *tty,
				struct usb_serial_port *port,
				struct ktermios *termios_old)
{
	unsigned int             cflag, block;
        speed_t                  rate;
	unsigned int             format_size, format_parity, format_stop, flow, gpio_mode;

	cflag = tty->termios.c_cflag;

        block = port->bulk_out_endpointAddress - 1;

        vizzini_disable(port);

        if ((cflag & CSIZE) == CS7)
                format_size = UART_FORMAT_SIZE_7;
        else if ((cflag & CSIZE) == CS5)
                /* Enabling 5-bit mode is really 9-bit mode! */
                format_size = UART_FORMAT_SIZE_9;
        else
                format_size = UART_FORMAT_SIZE_8;

        if (cflag & PARENB) {
                if (cflag & PARODD) {
                        if (cflag & CMSPAR)
                                format_parity = UART_FORMAT_PARITY_1;
                        else
                                format_parity = UART_FORMAT_PARITY_ODD;
                } else {
                        if (cflag & CMSPAR)
                                format_parity = UART_FORMAT_PARITY_0;
                        else
                                format_parity = UART_FORMAT_PARITY_EVEN;
		}
        } else {
                format_parity = UART_FORMAT_PARITY_NONE;
        }

        if (cflag & CSTOPB)
                format_stop = UART_FORMAT_STOP_2;
        else
                format_stop = UART_FORMAT_STOP_1;

        vizzini_set_reg(port, block, UART_FORMAT,
			format_size | format_parity | format_stop);

        if (cflag & CRTSCTS) {
                flow      = UART_FLOW_MODE_HW;
                gpio_mode = UART_GPIO_MODE_SEL_RTS_CTS;
        } else if (I_IXOFF(tty) || I_IXON(tty)) {
                unsigned char   start_char = START_CHAR(tty);
                unsigned char   stop_char  = STOP_CHAR(tty);

                flow      = UART_FLOW_MODE_SW;
                gpio_mode = UART_GPIO_MODE_SEL_GPIO;

                vizzini_set_reg(port, block, UART_XON_CHAR, start_char);
                vizzini_set_reg(port, block, UART_XOFF_CHAR, stop_char);
        } else {
                flow      = UART_FLOW_MODE_NONE;
                gpio_mode = UART_GPIO_MODE_SEL_GPIO;
        }

        vizzini_set_reg(port, block, UART_FLOW, flow);
        vizzini_set_reg(port, block, UART_GPIO_MODE, gpio_mode);

	vizzini_set_reg(port, EPLOCALS_REG_BLOCK,
			(block * MEM_EP_LOCALS_SIZE) + EP_WIDE_MODE,
			format_size == UART_FORMAT_SIZE_9);

        rate = tty_get_baud_rate(tty);
	if (rate)
		vizzini_set_baud_rate(port, rate);

        vizzini_enable(port);
}

static const struct usb_device_id id_table[] = {
	{ USB_DEVICE(0x04e2, 0x1410), },
	{ USB_DEVICE(0x04e2, 0x1412), },
	{ USB_DEVICE(0x04e2, 0x1414), },
	{ },
};
MODULE_DEVICE_TABLE(usb, id_table);

static const struct usb_device_id vizzini_1410_id_table[] = {
	{ USB_DEVICE(0x04e2, 0x1410), },
	{ },
};
static struct usb_serial_driver vizzini_1410_device = {
	.driver = {
		.owner =	THIS_MODULE,
		.name =		"vizzini_1410",
	},
	.id_table =		vizzini_1410_id_table,
	.num_ports =		1,
	.set_termios =		vizzini_set_termios,
};

static const struct usb_device_id vizzini_1412_id_table[] = {
	{ USB_DEVICE(0x04e2, 0x1412), },
	{ },
};
static struct usb_serial_driver vizzini_1412_device = {
	.driver = {
		.owner =	THIS_MODULE,
		.name =		"vizzini_1412",
	},
	.id_table =		vizzini_1412_id_table,
	.num_ports =		2,
	.set_termios =		vizzini_set_termios,
};

static const struct usb_device_id vizzini_1414_id_table[] = {
	{ USB_DEVICE(0x04e2, 0x1414), },
	{ },
};
static struct usb_serial_driver vizzini_1414_device = {
	.driver = {
		.owner =	THIS_MODULE,
		.name =		"vizzini_1414",
	},
	.id_table =		vizzini_1414_id_table,
	.num_ports =		4,
	.set_termios =		vizzini_set_termios,
};

static struct usb_serial_driver * const serial_drivers[] = {
	&vizzini_1410_device,
	&vizzini_1412_device,
	&vizzini_1414_device,
	NULL
};

module_usb_serial_driver(serial_drivers, id_table);

MODULE_LICENSE("GPL");
