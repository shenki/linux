/*
*  ast_usb_hub.h -- Contains the defines and structures for Hub functionality 
*
*
*  Copyright (C) 2012-2020  ASPEED Technology Inc.
*
*  This program is free software; you can redistribute it and/or modify
*  it under the terms of the GNU General Public License version 2 as
*  published by the Free Software Foundation.
*
*  History:
*    2017.10.03: Created the file and added defines and structures [Akshay Srinivas]
*/

#ifndef  __AST_USB_HUB_H__
#define __AST_USB_HUB_H__

enum hub_state{
	HUB_IDLE,
	HUB_ENUM,
	HUB_STEADY,
	DEV_ENUM,
	DEV_STEADY,
};
/*
 * wPortStatus bit field
 * See USB 2.0 spec Table 11-21
 */
#define USB_PORT_STAT_CONNECTION	0x0001
#define USB_PORT_STAT_ENABLE		0x0002
#define USB_PORT_STAT_SUSPEND		0x0004
#define USB_PORT_STAT_OVERCURRENT	0x0008
#define USB_PORT_STAT_RESET		0x0010
#define USB_PORT_STAT_L1		0x0020
/* bits 6 to 7 are reserved */
#define USB_PORT_STAT_POWER		0x0100
#define USB_PORT_STAT_LOW_SPEED		0x0200
#define USB_PORT_STAT_HIGH_SPEED        0x0400
#define USB_PORT_STAT_TEST              0x0800
#define USB_PORT_STAT_INDICATOR         0x1000
/* bits 13 to 15 are reserved */

/*
 * Additions to wPortStatus bit field from USB 3.0
 * See USB 3.0 spec Table 10-10
 */
#define USB_PORT_STAT_LINK_STATE	0x01e0
#define USB_SS_PORT_STAT_POWER		0x0200
#define USB_SS_PORT_STAT_SPEED		0x1c00
#define USB_PORT_STAT_SPEED_5GBPS	0x0000
/* Valid only if port is enabled */
/* Bits that are the same from USB 2.0 */
#define USB_SS_PORT_STAT_MASK (USB_PORT_STAT_CONNECTION |	    \
				USB_PORT_STAT_ENABLE |	    \
				USB_PORT_STAT_OVERCURRENT | \
				USB_PORT_STAT_RESET)

/*
 * wPortChange bit field
 * See USB 2.0 spec Table 11-22 and USB 2.0 LPM ECN Table-4.10
 * Bits 0 to 5 shown, bits 6 to 15 are reserved
 */
#define USB_PORT_STAT_C_CONNECTION	0x0001
#define USB_PORT_STAT_C_ENABLE		0x0002
#define USB_PORT_STAT_C_SUSPEND		0x0004
#define USB_PORT_STAT_C_OVERCURRENT	0x0008
#define USB_PORT_STAT_C_RESET		0x0010
#define USB_PORT_STAT_C_L1		0x0020

extern void handle_port_reset(int port_ind, struct usb_gadget *);
extern int handle_host_resume(int devnum, struct usb_gadget *gadget);
extern void port_enable(int devnum, struct usb_gadget *gadget);
extern void clear_mb_code(int devnum, unsigned int code_to_clear,struct usb_gadget *gadget);
void port_disable(int devnum, struct usb_gadget *gadget);
/* (shifted) direction/type/recipient from the USB 2.0 spec, table 9.2 */
#define DeviceRequest \
	((USB_DIR_IN|USB_TYPE_STANDARD|USB_RECIP_DEVICE)<<8)
#define DeviceOutRequest \
	((USB_DIR_OUT|USB_TYPE_STANDARD|USB_RECIP_DEVICE)<<8)

#define InterfaceRequest \
	((USB_DIR_IN|USB_TYPE_STANDARD|USB_RECIP_INTERFACE)<<8)

#define EndpointRequest \
	((USB_DIR_IN|USB_TYPE_STANDARD|USB_RECIP_INTERFACE)<<8)
#define EndpointOutRequest \
	((USB_DIR_OUT|USB_TYPE_STANDARD|USB_RECIP_INTERFACE)<<8)

/* class requests from the USB 2.0 hub spec, table 11-15 */
/* GetBusState and SetHubDescriptor are optional, omitted */
#define ClearHubFeature		(0x2000 | USB_REQ_CLEAR_FEATURE)
#define ClearPortFeature	(0x2300 | USB_REQ_CLEAR_FEATURE)
#define GetHubDescriptor	(0xa000 | USB_REQ_GET_DESCRIPTOR)
#define GetHubStatus		(0xa000 | USB_REQ_GET_STATUS)
#define GetPortStatus		(0xa300 | USB_REQ_GET_STATUS)
#define SetHubFeature		(0x2000 | USB_REQ_SET_FEATURE)
#define SetPortFeature		(0x2300 | USB_REQ_SET_FEATURE)
/*
 * Port feature numbers
 * See USB 2.0 spec Table 11-17
 */
#define USB_PORT_FEAT_CONNECTION	0
#define USB_PORT_FEAT_ENABLE		1
#define USB_PORT_FEAT_SUSPEND		2	/* L2 suspend */
#define USB_PORT_FEAT_OVER_CURRENT	3
#define USB_PORT_FEAT_RESET		4
#define USB_PORT_FEAT_L1		5	/* L1 suspend */
#define USB_PORT_FEAT_POWER		8
#define USB_PORT_FEAT_LOWSPEED		9
#define USB_PORT_FEAT_HIGHSPEED		10
#define USB_PORT_FEAT_C_CONNECTION	16
#define USB_PORT_FEAT_C_ENABLE		17
#define USB_PORT_FEAT_C_SUSPEND		18
#define USB_PORT_FEAT_C_OVER_CURRENT	19
#define USB_PORT_FEAT_C_RESET		20
#define USB_PORT_FEAT_TEST              21
#define USB_PORT_FEAT_INDICATOR         22
#define USB_PORT_FEAT_C_PORT_L1         23

#define EP0_BUFSIZE	256
#define MAX_HUB_BUF	512
#define STRING_MANUFACTURER     1
#define STRING_PRODUCT          2
#define STRING_SERIAL           3
#define STRING_CONFIG           4
#define STRING_INTERFACE        5

//host will provide port ind starting from 1 and not from 0 
#define MAX_DS_PORTS	5
//#define USE_MAILBOX

#endif  //__AST_USB_HUB_H__

