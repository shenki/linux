/*
*  ast_usb_hub.c -- Contains the Hub functionality for the AST2500
*
*
*  Copyright (C) 2012-2020  ASPEED Technology Inc.
*
*  This program is free software; you can redistribute it and/or modify
*  it under the terms of the GNU General Public License version 2 as
*  published by the Free Software Foundation.
*
*  History:
*    2017.10.03: Created the file and added code to support Hub and devices behind hub [Akshay Srinivas]
*/

#include <linux/kallsyms.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/device.h>

#include <linux/usb/ch9.h>
 #include <linux/usb/gadget.h>

#include <linux/timer.h>
#include <linux/module.h>

#include "ast_usb_hub.h"
extern int hub_usb_gadget_config_buf(
        const struct usb_config_descriptor      *config,
        void                                    *buf,
        unsigned                                length,
        const struct usb_descriptor_header      **desc
);
extern int hub_usb_gadget_get_string (struct usb_gadget_strings *table, int id, u8 *buf);
extern int usb_hub_gadget_register_driver (struct usb_gadget_driver *driver);
extern int usb_hub_gadget_unregister_driver (struct usb_gadget_driver *driver);
extern void usb_ep_autoconfig_reset (struct usb_gadget *gadget);
//#define EXTRA_DEBUG 1
#define HUB_SUSPEND_RESUME
#ifdef HUB_SUSPEND_RESUME
//PORT_RESUME is defined only if HUB_SUSPEND_RESUME is defined
#define PORT_RESUME
extern int handle_port_suspend(int devnum, struct usb_gadget *gadget);
#endif
static void ast_hub_port_status(struct usb_gadget *gadget);
#ifndef USE_MAILBOX
static void ast_check_set_address(struct usb_gadget *gadget);
#endif
struct hub_descriptor{
	unsigned char	bDescLength;
	unsigned char	bDescriptorType;
	unsigned char	bNbrPorts;
	unsigned short  hub_char_logical_pwr_switching_mode:2; /*D1...D0: Logical Power Switching Mode
								00: Ganged power switching (all ports’ power at
								once)
								01: Individual port power switching
								1X: Reserved. Used only on 1.0 compliant hubs
								that implement no power switching*/
	unsigned short  hub_char_comp_device:1; /* D2: Identifies a Compound Device
						0: Hub is not part of a compound device.
						1: Hub is part of a compound device.*/
	unsigned short  hub_char_ovr_current_prot_mode:2; /*D4...D3: Over-current Protection Mode
							00: Global Over-current Protection. The hub
							reports over-current as a summation of all
							ports’ current draw, without a breakdown of
							individual port over-current status.
							01: Individual Port Over-current Protection. The
							hub reports over-current on a per-port basis.
							Each port has an over-current status.
							1X: No Over-current Protection. This option is
							allowed only for bus-powered hubs that do not
							implement over-current protection.*/
	unsigned short hub_char_TT_think_time:2; /* D6...D5: TT Think TIme
						00: TT requires at most 8 FS bit times of inter
						transaction gap on a full-/low-speed
						downstream bus.
						01: TT requires at most 16 FS bit times.
						10: TT requires at most 24 FS bit times.
						11: TT requires at most 32 FS bit times.*/
	unsigned short hub_char_port_indicators_support:1; /* D7: Port Indicators Supported
							0: Port Indicators are not supported on its
							downstream facing ports and the
							PORT_INDICATOR request has no effect.
							1: Port Indicators are supported on its
							downstream facing ports and the
							PORT_INDICATOR request controls the
							indicators. See Section 11.5.3.*/
	unsigned short hub_char_rsvd:8; /*D15...D8: Reserved*/

	unsigned char  bPwrOn2PwrGood; /*Time (in 2 ms intervals) from the time the power-on
					sequence begins on a port until power is good on that
					port. The USB System Software uses this value to
					determine how long to wait before accessing a
					powered-on port.*/ 
	unsigned char  bHubContrCurrent; /*Maximum current requirements of the Hub Controller
					electronics in mA.*/
	unsigned char DeviceRemovable;/* Indicates if a port has a removable device attached.
					This field is reported on byte-granularity. Within a
					byte, if no port exists for a given location, the field
					representing the port characteristics returns 0.
					Bit value definition:
					0B - Device is removable.
					1B - Device is non-removable
					This is a bitmap corresponding to the individual ports
					on the hub:
					Bit 0: Reserved for future use.
					Bit 1: Port 1
					Bit 2: Port 2
					....
					Bit n: Port n (implementation-dependent, up to a
					maximum of 255 ports).*/	
	unsigned char PortPwrCtrlMask;/* This field exists for reasons of compatibility with
					software written for 1.0 compliant devices. All bits in
					this field should be set to 1B. This field has one bit for
					each port on the hub with additional pad bits, if
					necessary, to make the number of bits in the field an
					integer multiple of 8.*/

}__attribute__((packed));
struct hub_descriptor ast_hub_desc = {
        .bDescLength = 9,
        .bDescriptorType = 0x29,
        //.bNbrPorts= (MAX_DS_PORTS - 1),//host will provide port ind starting from 1 and not from 0
        .bNbrPorts= (MAX_DS_PORTS),//host will provide port ind starting from 1 and not from 0
        .hub_char_logical_pwr_switching_mode=00,
	.hub_char_comp_device = 0,
	.hub_char_ovr_current_prot_mode= 0,
	.hub_char_TT_think_time =0,
	.hub_char_port_indicators_support=0,
	.hub_char_rsvd=0,
	.bPwrOn2PwrGood=10,
	.bHubContrCurrent=100,
	.DeviceRemovable=0,
	.PortPwrCtrlMask=0xFF,
};
#define AST_USB_DEV_SPEED 1
#ifdef AST_USB_DEV_SPEED
extern int hub_speed;
#endif
#if 1
#define PHUB_DBG(fmt...) do{}while(0);
#else
#define PHUB_DBG(fmt...) printk(fmt);
#endif
#define ENDP_ADDR 0x01
/* There is only one interface. */

static struct usb_interface_descriptor
intf_desc = {
        .bLength =              sizeof (intf_desc),
        .bDescriptorType =      USB_DT_INTERFACE,
	.bInterfaceNumber =	0,
	.bAlternateSetting = 0,
        
        .bNumEndpoints =        1,              // Adjusted during fsg_bind()
        .bInterfaceClass =      0x09,
        .bInterfaceSubClass =   0,    // Adjusted during fsg_bind()
        .bInterfaceProtocol =   0,    // Adjusted during fsg_bind()
        .iInterface =           STRING_INTERFACE,
};
struct usb_endpoint_descriptor hub_ep_desc= {
        .bLength =              0x07, 
        .bDescriptorType =      USB_DT_ENDPOINT,
 
        .bEndpointAddress =     USB_DIR_IN|ENDP_ADDR,
        .bmAttributes =         USB_ENDPOINT_XFER_INT,
        .wMaxPacketSize =       __constant_cpu_to_le16(64),
	 .bInterval = 16,
};

static const struct usb_descriptor_header *hub_function[] = {
#if 0
        (struct usb_descriptor_header *) &otg_desc,
#endif
        (struct usb_descriptor_header *) &intf_desc,
        (struct usb_descriptor_header *) &hub_ep_desc,
        NULL,
}; 


/* Static strings, in UTF-8 (for simplicity we use only ASCII characters) */
static struct usb_string                strings[] = {
        {STRING_MANUFACTURER,   "ASpeed Technologies Inc"},
        {STRING_PRODUCT,        "ASpeed AST2500 HighSpeed HUB"},
        {STRING_SERIAL,         "0xBABEFACE" },
        {STRING_CONFIG,         "Self-powered"},
        {STRING_INTERFACE,      "High Speed Hub"},
        {}
};
static struct usb_gadget_strings	stringtab = {
	.language	= 0x0409,		// en-us
	.strings	= strings,
};


struct usb_device_descriptor hub_dev_desc = {
	.bLength = 0x12,
	.bDescriptorType = 1,

	.bcdUSB = 0x200,
	.bDeviceClass = 0x09,
	.bDeviceSubClass = 0x00,
	.bDeviceProtocol = 0x01,
	.bMaxPacketSize0 = 64,
	.idVendor	= __constant_cpu_to_le16(0x2A4B),
	.idProduct	= __constant_cpu_to_le16(0x0400),
	.bcdDevice	= __constant_cpu_to_le16(0x0100),
	.iManufacturer	= STRING_MANUFACTURER,
	.iProduct	= STRING_PRODUCT,
	.iSerialNumber	= STRING_SERIAL,
	.bNumConfigurations = 1,
};

struct usb_config_descriptor hub_config_desc = {
	.bLength = 0x09,
	.bDescriptorType = 2,

	.wTotalLength = 0x09,
	.bNumInterfaces = 1,
	.bConfigurationValue = 1,
	.iConfiguration = 0,
	.bmAttributes = (USB_CONFIG_ATT_ONE |USB_CONFIG_ATT_SELFPOWER | USB_CONFIG_ATT_WAKEUP),
	.bMaxPower = 50,
};

 struct ast_usb_hub_port_status {
	unsigned short	Connection_Sts:1;
	unsigned short	Enable_Sts:1;
	unsigned short	Suspend_Sts:1;
	unsigned short	Overcurrent_Sts:1;
	unsigned short	Reset_Sts:1;
	unsigned short	Reserved1:3;
	unsigned short	Portpower_Sts:1;
	unsigned short	Lspeedattach_Sts:1;
	unsigned short	Hspeedattach_Sts:1;
	unsigned short	PortTestMode_Sts:1;
	unsigned short	PortIndicatorControl_Sts:1;
	unsigned short	Reserved2:3;
	unsigned short	Connection_Sts_Chng:1;
	unsigned short	Enable_Sts_Chng:1;
	unsigned short	Suspend_Sts_Chng:1;
	unsigned short	Overcurrent_Sts_Chng:1;
	unsigned short	Reset_Sts_Chng:1;
	unsigned short	Reserved3:11;
};

struct usb_ds_port_dev;
typedef int (*ast_ds_port_setup_fn)(struct usb_ds_port_dev * ds_dev, unsigned char * setup_buf);
struct usb_ds_port_dev{
	struct EP_BUFFS* ep_buffers[32];	
	unsigned char 	dev_state; //0 - Idle, 1 - Enumerating , 2 - Steady
	unsigned char	device_address;
	ast_ds_port_setup_fn * ds_port_setup_fn;	
};

struct usb_hub_dev {
	struct usb_gadget		*gadget;
	struct usb_request		*req;
	unsigned			bufsiz;

	struct usb_configuration	*config;

	/* internals */
	struct usb_device_descriptor	desc;
	struct list_head		configs;
	//struct usb_composite_driver	*driver;
	u8				next_string_id;

	/* the gadget driver won't enable the data pullup
	 * while the deactivation count is nonzero.
	 */
	unsigned			deactivations;

	/* protects at least deactivation count */
	spinlock_t			lock;

//Status EP details
	struct usb_ep		*status_ep;
	struct usb_request 	*status_req;
	struct usb_endpoint_descriptor	*status_ep_desc;
 	unsigned char 		ast_hub_state; //0 - Idle, 1 - HUB_ENUM, 2 - HUB_STEADY, 3 - DEV_ENUM
	unsigned char		hub_dev_address;
	unsigned char		current_dev_address;
	struct ast_usb_hub_port_status	hub_port_status[MAX_DS_PORTS];
   	u32                     port_status[MAX_DS_PORTS];
	u32	resetdone;
	/* This is set if for a particular device there is some port status change and 
	  * the same has not been informed to the HOST
	  */
	u32 device_status_pending;
        /* sometimes during plug out/plug in or rmmod.insmod there are chances that hub status change
         * is ignored by h/w. Basically s/w enqueues hub status change on interrupt endpoint , so ep_queue
         * retruens a success but now if we clear device_status_pending based on this assumption it may not
         * reach host. so it is better to clear it on completion.Right now it is used only by ast_hub_port_status
         */
	u32 device_clear_status_pending;
	/* This will be set till a device is addressed , then we move on to next device */
	u32 connect_pending;
	u32 reset_in_progress;
	u32 devices_present;
#ifdef HUB_SUSPEND_RESUME
	unsigned long	suspend_timeout[MAX_DS_PORTS];
#endif
	unsigned hub_disconnect:1;
#ifdef HUB_SUSPEND_RESUME
	u32 suspend_changed;
#endif
};
#ifdef DELETE_LATER
struct ast_usb_hub_port_status	hub_port_status[MAX_DS_PORTS];
static struct ast_usb_hub_port_status port_status_new[MAX_DS_PORTS];
#endif
unsigned char status_change_bitmap = 0;
static struct timer_list status_change_timer;
#ifndef USE_MAILBOX
static struct timer_list set_address_timer;
#endif
int ast_hub_dev_address;

struct usb_ds_port_dev *ast_dsport_device_register(ast_ds_port_setup_fn* setup_fn){
	return NULL;
}
struct usb_ds_port_dev *ast_dsport_device_unregister(void){
	return NULL;
}

#define PORT_C_MASK \
	((USB_PORT_STAT_C_CONNECTION \
	| USB_PORT_STAT_C_ENABLE \
	| USB_PORT_STAT_C_SUSPEND \
	| USB_PORT_STAT_C_OVERCURRENT \
	| USB_PORT_STAT_C_RESET) << 16)

//Enable it when required for debugging the causes of HUB reset 
#undef HUB_RESET_DEBUG
#ifdef HUB_RESET_DEBUG
#define MAX_INDEX 2
struct hub_reset_debug {
	int hub_status_changed;
	u32 device_status_pending;
	u32 connect_pending;
	u32 reset_in_progress;
	u32 resetdone;
	u32 devices_present;
	u32 port_status[MAX_DS_PORTS];
	u32 last_port_status[MAX_DS_PORTS];
	int from_where;
	int dev_num;
	int status;
	u64 timestamp;
};
static int last_ep_queue;
static int hub_reset_index;
static struct hub_reset_debug hub_debug[MAX_INDEX];
static struct hub_reset_debug hub_debug_devices[MAX_DS_PORTS];
static u32 last_port_status[MAX_DS_PORTS];
static int last_device;
static u64 last_time_in_jiffies;

static void update_debug_stats(struct usb_hub_dev *cdev, int who, int status, int devnum, int hub_status_changed)
{
	hub_debug[hub_reset_index].hub_status_changed = hub_status_changed;
	hub_debug[hub_reset_index].device_status_pending = cdev->device_status_pending;
	hub_debug[hub_reset_index].connect_pending = cdev->connect_pending;			
	hub_debug[hub_reset_index].reset_in_progress = cdev->reset_in_progress;
	hub_debug[hub_reset_index].devices_present = cdev->devices_present;
	hub_debug[hub_reset_index].from_where = who;
	hub_debug[hub_reset_index].status = status;
	hub_debug[hub_reset_index].dev_num = devnum;
	hub_debug[hub_reset_index].resetdone = cdev->resetdone;
	hub_debug[hub_reset_index].port_status[devnum] = cdev->port_status[devnum];
	hub_debug[hub_reset_index].last_port_status[devnum] = last_port_status[devnum];
	hub_debug[hub_reset_index].timestamp = jiffies;
	//printk("print %lu lastime %lu %llu\n", jiffies, hub_debug[hub_reset_index].timestamp, hub_debug[hub_reset_index].timestamp);

	hub_debug_devices[devnum].hub_status_changed = hub_status_changed;
	hub_debug_devices[devnum].device_status_pending = cdev->device_status_pending;
	hub_debug_devices[devnum].connect_pending = cdev->connect_pending;			
	hub_debug_devices[devnum].reset_in_progress = cdev->reset_in_progress;
	hub_debug_devices[devnum].devices_present = cdev->devices_present;
	hub_debug_devices[devnum].from_where = who;
	hub_debug_devices[devnum].status = status;
	hub_debug_devices[devnum].dev_num = devnum;
	hub_debug_devices[devnum].resetdone = cdev->resetdone;
	hub_debug_devices[devnum].port_status[devnum] = cdev->port_status[devnum];
	hub_debug_devices[devnum].last_port_status[devnum] = last_port_status[devnum];
	hub_debug_devices[devnum].timestamp = jiffies;

	hub_reset_index = (hub_reset_index + 1) % MAX_INDEX;
}

static void print_last_device_debug_stats(struct usb_hub_dev *cdev, int dev_num)
{
	int i = 0;
	int devnum = 0;
	printk("last devices details =%d\n",dev_num);
	printk("device_clear_status_pending %x device_status_pending %x resetdone %d connect pending %d\n",
                    cdev->device_clear_status_pending, cdev->device_status_pending, cdev->resetdone, cdev->connect_pending);
	for(i = 0; i < MAX_DS_PORTS;i++) {
		if(i == dev_num) {
			devnum = i;
			printk("hub_status_changed %x device_status_pending %x connect_pending %x devices_present %x\n", 
				hub_debug_devices[i].hub_status_changed, hub_debug_devices[i].device_status_pending, hub_debug_devices[i].connect_pending, hub_debug_devices[i].devices_present);
			printk("reset_in_progress %x from_where %d status %d devnum %d last_port_status %x curportstatus %x\n", 
				hub_debug_devices[i].reset_in_progress, hub_debug_devices[i].from_where, hub_debug_devices[i].status, devnum,
				last_port_status[devnum], cdev->port_status[devnum]);
			printk("lastportstatus[%d] %x port_status %x resetdone %x\n",
				hub_debug_devices[i].dev_num, hub_debug_devices[i].last_port_status[devnum], hub_debug_devices[i].port_status[devnum], hub_debug_devices[i].resetdone);
			printk(" curtime %lu lastime %llu last_time_in_jiffies %llu\n", jiffies, hub_debug_devices[i].timestamp, last_time_in_jiffies);
		}
	}
	printk("\n");
}
static void print_reset_debug_stats(struct usb_hub_dev *cdev)
{
	int i = 0;
	int devnum = 0;
	printk("\n");
	printk("last_ep_queue %d hub_reset_index %d last_time_in_jiffies %llu\n", last_ep_queue, hub_reset_index, last_time_in_jiffies);
	for(i = hub_reset_index; i < MAX_INDEX;i++) {
		devnum = hub_debug[i].dev_num;
		printk("hub_status_changed %x device_status_pending %x connect_pending %x devices_present %x\n", 
			hub_debug[i].hub_status_changed, hub_debug[i].device_status_pending, hub_debug[i].connect_pending, hub_debug[i].devices_present);
		printk("reset_in_progress %x from_where %d status %d devnum %d last_port_status %x curportstatus %x\n", 
			hub_debug[i].reset_in_progress, hub_debug[i].from_where, hub_debug[i].status, devnum,
			last_port_status[devnum], cdev->port_status[devnum]);
		printk("lastportstatus[%d] %x port_status %x resetdone %d\n",
			hub_debug[i].dev_num, hub_debug[i].last_port_status[devnum], hub_debug[i].port_status[devnum], hub_debug[i].resetdone);
		printk("cur time %lu lastime %lu %llu\n", jiffies, (unsigned long)hub_debug[i].timestamp, hub_debug[i].timestamp);
	}

	for(i = 0; i < hub_reset_index;i++) {
		devnum = hub_debug[i].dev_num;
		printk("hub_status_changed %x device_status_pending %x connect_pending %x devices_present %x\n", 
			hub_debug[i].hub_status_changed, hub_debug[i].device_status_pending, hub_debug[i].connect_pending, hub_debug[i].devices_present);
		printk("reset_in_progress %x from_where %d status %d devnum %d last_port_status %x curportstatus %x\n", 
			hub_debug[i].reset_in_progress, hub_debug[i].from_where, hub_debug[i].status, devnum,
			last_port_status[devnum], cdev->port_status[devnum]);
		printk("lastportstatus[%d] %x port_status %x resetdone %d\n",
			hub_debug[i].dev_num, hub_debug[i].last_port_status[devnum], hub_debug[i].port_status[devnum], hub_debug[i].resetdone);
		printk(" curtime %lu lastime %llu\n", jiffies, hub_debug[i].timestamp);
	}
	printk("last devices details=\n");
	for(i = 0; i < MAX_DS_PORTS;i++) {
		if(i == last_device) {
			devnum = hub_debug_devices[i].dev_num;
			printk("hub_status_changed %x device_status_pending %x connect_pending %x devices_present %x\n", 
				hub_debug_devices[i].hub_status_changed, hub_debug_devices[i].device_status_pending, hub_debug_devices[i].connect_pending, hub_debug_devices[i].devices_present);
			printk("reset_in_progress %x from_where %d status %d devnum %d last_port_status %x curportstatus %x\n", 
				hub_debug_devices[i].reset_in_progress, hub_debug_devices[i].from_where, hub_debug_devices[i].status, devnum,
				last_port_status[devnum], cdev->port_status[devnum]);
			printk("lastportstatus[%d] %x port_status %x resetdone %d\n",
				hub_debug_devices[i].dev_num, hub_debug_devices[i].last_port_status[devnum], hub_debug_devices[i].port_status[devnum], hub_debug_devices[i].resetdone);
			printk(" curtime %lu lastime %llu\n", jiffies, hub_debug_devices[i].timestamp);
		}
	}
	printk("\n");
}
#endif

static struct usb_hub_dev       *hub_cdev __read_mostly;
void ast_hub_status_ep_complete(struct usb_ep *ep, struct usb_request *req)
{               
        int devnum = 0, timeout = 0;
        struct usb_hub_dev      *cdev = hub_cdev;
	unsigned long flags;
	int locked = 1;

        PHUB_DBG("Entered %s\n", __FUNCTION__);
        if (req->status || req->actual != req->length)
                PHUB_DBG(
                                "setup complete --> %d, %d/%d\n",
                                req->status, req->actual, req->length);
	locked = spin_trylock_irqsave(&cdev->lock, flags);
	del_timer(&status_change_timer);
#ifdef EXTRA_DEBUG
	printk("complete req->status %d device_clear_status_pending %x cdev->port_status %x locked %d\n",
		req->status, cdev->device_clear_status_pending, cdev->port_status[4], locked);
#endif
        if(!req->status && cdev->device_clear_status_pending) {
                PHUB_DBG("cdev %p cdev->device_clear_status_pending %x cdev->device_status_pending %x req %p \n",
                        cdev, cdev->device_clear_status_pending, cdev->device_status_pending, req);
                devnum = ffs(cdev->device_clear_status_pending);
                if(devnum) {
                        cdev->device_status_pending &= ~(1 << (devnum - 1));
                }
                cdev->device_clear_status_pending = 0;
        }
       if(likely(!req->status)) {
               /*Give a longer delay so that host gets time to do a get port status
                * if we give a small delay then there can be spurious hub status change and depending
                * on host system s/w it may or may not reset Hub
                * Ex:- during rmmod and insmod tests we may get a reset done interrupt and we enqueue a
                * status change , now if we get completion before host deos get port status and we start the
                * timer with a shorter duration then we have enqueued 1 more hub status change(ideally this time)
                * there is no status change, now host decides to do get port status for previous status change, after this
                * host again does a get port status (due to 2nd status change) and this time hub says no change , If the host
                * is not resilient or a buggy host then he may decide to issue a Hub reset.
                * If the host doesnt give a get port status/does not do the next expected step then retry again after 250ms
               */
               if(cdev->device_status_pending)
			timeout = msecs_to_jiffies(50);//50 ms
		else
			timeout = msecs_to_jiffies(250);//250 ms
	} else {
		timeout = msecs_to_jiffies(100);//100 ms
#ifdef EXTRA_DEBUG
		printk("Hub status change failed req->status %d time %lu\n", req->status, jiffies);
		printk("device_clear_status_pending %x device_status_pending %x resetdone %d connect pending %d\n",
                        cdev->device_clear_status_pending, cdev->device_status_pending, cdev->resetdone, cdev->connect_pending);
#endif
	}
	if(locked)
		spin_unlock_irqrestore(&cdev->lock, flags);
	status_change_timer.expires = jiffies + timeout;
	status_change_timer.function = (void (*)(unsigned long))ast_hub_port_status;
	mod_timer(&status_change_timer, jiffies + timeout);
}

static void ast_hub_setup_complete(struct usb_ep *ep, struct usb_request *req)
{
	PHUB_DBG("Entered %s\n", __FUNCTION__);
        if (req->status || req->actual != req->length){
#ifdef EXTRA_DEBUG
                printk(
                                "setup complete --> %d, %d/%d\n",
                                req->status, req->actual, req->length);
#endif
	}
		req->status = 0;
}
#ifdef LINUX_3_4_41
#include "../epautoconf.c"
#endif
static void /* __init_or_exit */
ast_hub_unbind(struct usb_gadget *gadget)
{
	struct usb_hub_dev	*cdev = get_gadget_data(gadget);
	PHUB_DBG("Entered %s\n", __FUNCTION__);

	if (cdev->req) {
                kfree(cdev->req->buf);
                usb_ep_free_request(gadget->ep0, cdev->req);
        }
        kfree(cdev);
        set_gadget_data(gadget, NULL);
}
#ifndef LINUX_3_4_41
static void usb_ep_autoconfig_reset_(struct usb_gadget *gadget)
{
	struct usb_ep	*ep;

	list_for_each_entry (ep, &gadget->ep_list, ep_list) {
		ep->driver_data = NULL;
	}
	gadget->in_epnum = 0;
	gadget->out_epnum = 0;
}

static struct usb_ep * get_hub_ep(struct usb_gadget *gadget, struct usb_endpoint_descriptor	*desc)
{
	struct usb_ep	*ep;
	list_for_each_entry (ep, &gadget->ep_list, ep_list) {
#ifdef EXTRA_DEBUG
			printk("HUB:ep->name %s\n", ep->name);
#endif
		if (!strcmp(ep->name, "ep1"))
			goto found_ep;
	}
	/* Fail */
	return NULL;
found_ep:
	ep->address = desc->bEndpointAddress;
	ep->desc = NULL;
	ep->comp_desc = NULL;
	return ep;
}
#endif

//int __init ast_hub_bind(struct usb_gadget *gadget, struct usb_gadget_driver *driver)
int ast_hub_bind(struct usb_gadget *gadget, struct usb_gadget_driver *driver)
{ 
	struct usb_hub_dev        *cdev;
	int                             status = -ENOMEM;
	int i = 0;

	PHUB_DBG("Entered %s gadget %p\n", __FUNCTION__, gadget);
        cdev = kzalloc(sizeof *cdev, GFP_KERNEL);
        if (!cdev)
                return status;
        
        spin_lock_init(&cdev->lock);
        cdev->gadget = gadget;
        set_gadget_data(gadget, cdev);
	hub_cdev = cdev;
        INIT_LIST_HEAD(&cdev->configs);
	for(i = 0;i < MAX_DS_PORTS;i++)
		cdev->port_status[i] = USB_PORT_STAT_POWER;
        /* preallocate control response and buffer */
        cdev->req = usb_ep_alloc_request(gadget->ep0, GFP_KERNEL);

	        if (!cdev->req)
                goto fail;
        cdev->req->buf = kmalloc(MAX_HUB_BUF, GFP_KERNEL);
        if (!cdev->req->buf)
                goto fail;
        cdev->req->complete = ast_hub_setup_complete;
        gadget->ep0->driver_data = cdev;

        cdev->bufsiz = MAX_HUB_BUF;

	usb_gadget_set_selfpowered(gadget);
#ifdef LINUX_3_4_41
	usb_ep_autoconfig_reset(cdev->gadget);
#else
	usb_ep_autoconfig_reset_(cdev->gadget);
#endif
#ifdef EXTRA_DEBUG
	printk("%s %d gadget->ep0 %p\n", __FUNCTION__, __LINE__,gadget->ep0);
#endif
	gadget->ep0->driver_data = cdev;
#if 1 //This is for the Status change endpoint
#ifdef LINUX_3_4_41
	cdev->status_ep = usb_ep_autoconfig(cdev->gadget, &hub_ep_desc);
#else
	cdev->status_ep = get_hub_ep(cdev->gadget, &hub_ep_desc);
#endif
	if(cdev->status_ep == NULL) {
		printk("Aspeed Hub interrupt end point not found!\n");
		return 0;
	}
#ifdef EXTRA_DEBUG
	printk("%s %d\n", __FUNCTION__, __LINE__);
#endif
	cdev->status_req = usb_ep_alloc_request(cdev->status_ep, GFP_KERNEL);
#ifdef EXTRA_DEBUG
	printk("%s %d\n", __FUNCTION__, __LINE__);	
#endif
	cdev->status_req->buf = kzalloc(64, GFP_KERNEL);
	cdev->status_req->length=64;
	cdev->status_req->complete = ast_hub_status_ep_complete;
	cdev->status_ep->desc = &hub_ep_desc;
	status = usb_ep_enable(cdev->status_ep);
	if(status < 0){
		PHUB_DBG("could not Enable EP\n");
		goto fail;
	}
#endif
#ifndef USE_MAILBOX
	init_timer(&set_address_timer);
	set_address_timer.data = (unsigned long)cdev->gadget;
	set_address_timer.function = (void (*)(unsigned long))ast_check_set_address;
#endif
	init_timer(&status_change_timer);
	status_change_timer.expires = jiffies + msecs_to_jiffies(4000);
	status_change_timer.data = (unsigned long)cdev->gadget;
	status_change_timer.function = (void (*)(unsigned long))ast_hub_port_status;
	add_timer(&status_change_timer);
	return 0;
fail:
	ast_hub_unbind(gadget);
	return -1;
}

static void
ast_hub_suspend(struct usb_gadget *gadget)
{
#ifdef HUB_SUSPEND_RESUME
	struct usb_hub_dev        *cdev = get_gadget_data(gadget);
	int i = 0;
	unsigned long flags;
	spin_lock_irqsave(&cdev->lock, flags);

	for(i = 0;i < MAX_DS_PORTS;i++) {
		if(!(cdev->devices_present & (1 << i))) {
			continue;
		}
		/* If the port is enabled but not already suspended via SetPortFeature then call suspend */
		/* If there is a BUS suspend then only Hub can know in that case device suspend will
		 * be called as many times as Hub suspend handler gets called, unless the port is not already
		 * selectively suspended via SetPortFeature
		 */
		if(!(cdev->port_status[i] & USB_PORT_STAT_SUSPEND) && (cdev->port_status[i] & USB_PORT_STAT_ENABLE)) {
			handle_port_suspend(i, gadget);
		}
	}
	spin_unlock_irqrestore(&cdev->lock, flags);
#endif
}

static void
ast_hub_resume(struct usb_gadget *gadget)
{
}


void ast_hub_dump(unsigned char * buf, long size){

	int i = size/8;
	int rem = size%8;
	int j=0;
	return;
	if(size <= 0)
		return;

	for(i = 0; i< size; i +=8){
		for (j=0; j< 8; j++){
			printk("%x \t", *(buf+i+j));
		}
		printk("\n");
	}
	i -=8;
	for(j=0; j<rem; j++){
			printk("%x \t", *(buf+i+j));
	}
	printk("\n");
}
void print_desc_types(int value){
        int my_val = (value & (~(USB_TYPE_CLASS)));
	return;
        if((value & USB_TYPE_CLASS) == USB_TYPE_CLASS){
                printk("\nClass specific Desc");
        }

        switch (my_val){
                case USB_DT_DEVICE:
                        printk("USB_DT_DEVICE\n");
                break;
                case USB_DT_CONFIG:
                        printk("USB_DT_CONFIG\n");
                break;
                case USB_DT_STRING:
                        printk("USB_DT_STRING\n");
                break;
                case USB_DT_INTERFACE:
                        printk("USB_DT_INTERFACE\n");
                break;
                case USB_DT_ENDPOINT:
                        printk("USB_DT_ENDPOINT\n");
                break;
                case USB_DT_DEVICE_QUALIFIER:
                        printk("USB_DT_DEVICE_QUALIFIER\n");
                break;
                case USB_DT_OTHER_SPEED_CONFIG:
                        printk("USB_DT_OTHER_SPEED_CONFIG\n");
                break;
                case USB_DT_INTERFACE_POWER:
                        printk("USB_DT_INTERFACE_POWER\n");
                break;
                case USB_DT_INTERFACE_ASSOCIATION:
                        printk("USB_DT_INTERFACE_ASSOCIATION\n");
                break;
                case USB_DT_OTG:
                        printk("USB_DT_OTG\n");
                break;
                case USB_DT_DEVICE_CAPABILITY:
                        printk("USB_DT_DEVICE_CAPABILITY\n");
                break;
        }
}

 enum hub_port_feature{
	C_HUB_LOCAL_POWER = 0,
	C_HUB_OVER_CURRENT,
	PORT_CONNECTION = 0,
	PORT_ENABLE,
	PORT_SUSPEND,
	PORT_OVER_CURRENT,
	PORT_RESET,
	PORT_POWER,
	PORT_LOW_SPEED,
	C_PORT_CONNECTION = 16,
	C_PORT_ENABLE,
	C_PORT_SUSPEND,
	C_PORT_OVER_CURRENT,
	C_PORT_RESET,
	PORT_TEST,
	PORT_INDICATOR,
};
#ifdef LINUX_3_4_41
#include "../usbstring.c"
#include "../config.c"
#endif
int ast_virt_hub_control(
	struct usb_gadget *gadget,
	u16		typeReq,
	u16		wValue,
	u16		wIndex,
	char		*buf,
	u16		wLength)
{
	u32		temp;
	int		retval = 0;
	struct usb_hub_dev        *cdev = get_gadget_data(gadget);
	unsigned long flags;
	spin_lock_irqsave(&cdev->lock, flags);

	/* hub features:  always zero, setting is a NOP
	 * port features: reported, sometimes updated when host is active
	 * no indicators
	 */
	switch (typeReq) {
	case ClearHubFeature:
	case SetHubFeature:
		switch (wValue) {
		case C_HUB_OVER_CURRENT:
		case C_HUB_LOCAL_POWER:
			break;
		default:
			printk("Def err case typeReq %x wValue %x wLength %x wIndex %d line %d\n",
				typeReq, wValue, wLength, wIndex, __LINE__);
			goto error;
		}
		break;
	case ClearPortFeature:
		wIndex = wIndex  - 1;
#ifdef EXTRA_DEBUG
		printk("ClearPortFeature wIndex %d wValue %d\n", wIndex,wValue);
#endif

		switch (wValue) {
		case USB_PORT_FEAT_ENABLE:
#ifdef EXTRA_DEBUG
			printk("Hub:Disable %d port\n", wIndex);
#endif
			/*Should we make this port in disabled state?*/
			cdev->port_status[wIndex] &= ~USB_PORT_STAT_ENABLE;
#ifdef HUB_SUSPEND_RESUME
			cdev->port_status[wIndex] &= ~USB_PORT_STAT_SUSPEND;
#endif
#ifdef EXTRA_DEBUG
			printk("Disable:cdev->reset_in_progress %x connect_pending %x device_status_pending %x\n",
				cdev->reset_in_progress, cdev->connect_pending, cdev->device_status_pending);
#endif
			cdev->reset_in_progress &= ~(1 << (wIndex));
			port_disable(wIndex, gadget);
			break;
		case USB_PORT_FEAT_SUSPEND:
#ifdef EXTRA_DEBUG
			printk("clear USB_PORT_FEAT_SUSPEND Not handled yet\n");
			printk("typeReq %x wValue %x wLength %x wIndex %d line %d\n",
				typeReq, wValue, wLength, wIndex, __LINE__);
#endif
#ifdef HUB_SUSPEND_RESUME
			if (cdev->port_status[wIndex] & USB_PORT_STAT_SUSPEND) {
				/* 20msec resume signaling */
				cdev->suspend_timeout[wIndex] = jiffies + msecs_to_jiffies(20);
#ifdef PORT_RESUME
				(void)handle_host_resume(wIndex, gadget);
#endif
				cdev->suspend_changed |= (1 << wIndex);
#ifdef EXTRA_DEBUG
				printk("Dev resume %d %lx %lx\n", wIndex, cdev->suspend_timeout[wIndex], jiffies);
#endif
				//Fire the timer ASAP and check 20ms timeout later in hub_port_status
				mod_timer(&status_change_timer, jiffies + msecs_to_jiffies(1));
			}
#endif

			break;
		case USB_PORT_FEAT_POWER:
#ifdef EXTRA_DEBUG
			printk("clear USB_PORT_FEAT_POWER\n");
			printk("typeReq %x wValue %x wLength %x wIndex %d line %d\n",
				typeReq, wValue, wLength, wIndex, __LINE__);
#endif
			break;
		case USB_PORT_FEAT_C_CONNECTION:
			cdev->port_status[wIndex] &= ~(USB_PORT_STAT_C_CONNECTION << 16);
			//Clear port feature means host acknowledged that it saw our message
			//So if the status change for this port is set try to clear
			//there can still be race conditions though b/w we clearing status change and 
			//before host reads it, but alteast try to reduce the probability
			//TODO if some change is still set shoud we still go ahead and clear status change EP?
			if((1 << wIndex) == cdev->device_clear_status_pending) {
				int ret = -1;
				ret = usb_ep_dequeue(cdev->status_ep, cdev->status_req);
#ifdef EXTRA_DEBUG
				printk("clear conn Dequed existing request ret %d port_status[%d] %x\n",
					ret, wIndex, cdev->port_status[wIndex]); 
#endif
			}
			break;
		case USB_PORT_FEAT_C_ENABLE:
			cdev->port_status[wIndex] &= ~(USB_PORT_STAT_C_ENABLE << 16);
			break;
		case USB_PORT_FEAT_C_OVER_CURRENT:
			break;
		case USB_PORT_FEAT_C_RESET:
			cdev->port_status[wIndex] &= ~(USB_PORT_STAT_C_RESET << 16);
			//Clear port feature means host acknowledged that it saw our message
			//So if the status change for this port is set try to clear
			//there can still be race conditions though b/w we clearing status change and 
			//before host reads it, but alteast try to reduce the probability
			//TODO if some change is still set shoud we still go ahead and clear status change EP?
			if((1 << wIndex) == cdev->device_clear_status_pending) {
				int ret = -1;
				ret = usb_ep_dequeue(cdev->status_ep, cdev->status_req);
#ifdef EXTRA_DEBUG
				printk("clear reset Dequed existing request ret %d port_status[%d] %x\n",
					ret, wIndex, cdev->port_status[wIndex]); 
#endif
			}
			break;
		case USB_PORT_FEAT_C_SUSPEND:
#ifdef HUB_SUSPEND_RESUME
			cdev->port_status[wIndex] &= ~(USB_PORT_STAT_C_SUSPEND << 16);
#endif
			break;
		default:
			printk("Def err case typeReq %x wValue %x wLength %x wIndex %d line %d\n",
				typeReq, wValue, wLength, wIndex, __LINE__);
			goto error;
		}
		break;
	case GetHubDescriptor:
		memcpy(buf, &ast_hub_desc, sizeof(ast_hub_desc));
		break;
	case GetHubStatus:
		temp = 0;
		*(__le32 *) buf = cpu_to_le32(temp);
		break;
	case GetPortStatus:
		wIndex = wIndex  - 1;
#ifdef EXTRA_DEBUG
		printk("GetPortStatus cdev->port_status[%d] %x reset_in_progress %x\n", wIndex, cdev->port_status[wIndex],
				cdev->reset_in_progress);
#endif
		if(cdev->port_status[wIndex] & USB_PORT_STAT_RESET) {
#ifdef EXTRA_DEBUG
			printk("cdev->resetdone %x wIndex %d reset_in_progress %x device_clear_status_pending %x\n",
				cdev->resetdone, wIndex, cdev->reset_in_progress, cdev->device_clear_status_pending);
#endif
		}
#if 0
		if((((cdev->port_status[wIndex]) & (USB_PORT_STAT_RESET) ) && ((cdev->resetdone) & (1 << wIndex)))
			/*
			  * This is because if there was a reset in progress set and Port is enabled tat means reset has actually happend
			  * but no SET_ADDRESS still i believe it is safe to inform host that reset is done without wasting time
			  */
			|| (((cdev->port_status[wIndex]) & (USB_PORT_STAT_RESET)) && (((cdev->reset_in_progress) & (1 << wIndex)) == 1) && ((cdev->port_status[wIndex]) & (USB_PORT_STAT_ENABLE)))){
			/*
				Enable status change should be set only on a PORT Disable 
				Some USB Host controllers are very sensitive to this and if 
				this bit is set that means port is disabled.
			*/
			cdev->resetdone = 0;
//			cdev->connect_or_reset_in_progress &= ~(1 << wIndex);
			cdev->port_status[wIndex] &= ~USB_PORT_STAT_RESET;
			cdev->port_status[wIndex] |= USB_PORT_STAT_ENABLE | USB_PORT_STAT_HIGH_SPEED
						| (USB_PORT_STAT_C_RESET << 16);
#ifdef EXTRA_DEBUG
			printk("Clear reset\n");
#endif
		}
#endif
#ifdef HUB_RESET_DEBUG
		last_port_status[wIndex] = cdev->port_status[wIndex];
#endif
		memcpy(buf, &cdev->port_status[wIndex], sizeof(u32));
		break;
	case SetPortFeature:
		switch (wValue) {
		case USB_PORT_FEAT_POWER:
#ifdef EXTRA_DEBUG
			printk("set USB_PORT_FEAT_POWER\n");
#endif
			wIndex = wIndex  - 1;
#ifdef EXTRA_DEBUG
			printk("typeReq %x wValue %x wLength %x wIndex %d line %d\n",
				typeReq, wValue, wLength, wIndex, __LINE__);
#endif

			cdev->port_status[wIndex] |= USB_PORT_STAT_POWER;
			status_change_timer.data = (unsigned long)cdev->gadget;
			break;
		case USB_PORT_FEAT_RESET:
			wIndex = wIndex  - 1;
#ifdef EXTRA_DEBUG
			printk("USB_PORT_FEAT_RESET wIndex %d reset_in_progress %x port_status %x resetdone %d\n",
					wIndex, cdev->reset_in_progress, cdev->port_status[wIndex], cdev->resetdone);
#endif
			if(cdev->reset_in_progress) {
				/* If there was a reset in progress , and host has asked to reset a different
				  * devive , Take care of that.
				  */
				if((cdev->reset_in_progress & (1 << wIndex)) == 0) {
					extern int get_far(int );
					int reset_devnum = ffs(cdev->reset_in_progress);
					if(get_far((reset_devnum)) == 0) {
						printk("There was a previous reset disabling device %d\n",
							reset_devnum - 1);
						port_disable(reset_devnum - 1, gadget);
					} else {
						printk("Not disabling device %d Hub is assuming device is up "
							"and running based on FAR!\n",  reset_devnum - 1);
					}
					cdev->reset_in_progress &= ~(1 << (reset_devnum - 1));
				}
			}

			if(cdev->port_status[wIndex] & USB_PORT_STAT_CONNECTION &&
				(cdev->reset_in_progress & (1 << wIndex)) == 0) {
				cdev->reset_in_progress |= (1 << wIndex);
#ifndef USE_MAILBOX
				mod_timer(&set_address_timer, jiffies + msecs_to_jiffies(40));
#endif
				if((cdev->connect_pending & (1 << wIndex)) == 0)
					printk("cdev->connect_pending %x is 0 ? wIndex %d\n",
						cdev->connect_pending, wIndex);
				/* Ideally only after a successful reset shd we reset connect_pending? */
				cdev->connect_pending &= ~(1 << wIndex);
				cdev->port_status[wIndex] |= USB_PORT_STAT_RESET;
				cdev->port_status[wIndex] &= ~USB_PORT_STAT_ENABLE;
#ifdef HUB_SUSPEND_RESUME
				cdev->port_status[wIndex] &= ~USB_PORT_STAT_SUSPEND;
#endif
				handle_port_reset(wIndex, gadget);
			} else {
#ifdef HUB_RESET_DEBUG
				printk("USB_PORT_FEAT_RESET:connect_pending %x port_status %x wIndex %d\n",
					cdev->connect_pending, cdev->port_status[wIndex], wIndex);
#endif
#if 1 
				/* reset_in_progress will be set till a setaddress is received for that device, now if host has issued a port reset
				  * and it has timed out because device reset took more than expected time , then host will again issue a port reset
				  * this time if previous reset has completed and if port is enabled then i believe it is safe to inform host that reset 
				  * is done so that host wont timeout
				  */
				if((cdev->reset_in_progress & (1 << wIndex)) == (1 << wIndex) && (cdev->port_status[wIndex] & USB_PORT_STAT_ENABLE)) {
#ifdef EXTRA_DEBUG
					printk("Behave like reset is done\n");
#endif
					cdev->port_status[wIndex] |= USB_PORT_STAT_RESET;
					cdev->resetdone = (1 << wIndex);
					mod_timer(&status_change_timer, jiffies+1);
				}
#endif
			}
			break;
		case USB_PORT_FEAT_SUSPEND:
#ifdef HUB_SUSPEND_RESUME
			wIndex = wIndex  - 1;
			if(!(cdev->port_status[wIndex] & USB_PORT_STAT_SUSPEND) && (cdev->port_status[wIndex] & USB_PORT_STAT_ENABLE)) {
				cdev->port_status[wIndex] |= USB_PORT_STAT_SUSPEND;
				handle_port_suspend(wIndex, gadget);
			}
#endif
#ifdef EXTRA_DEBUG
			printk("typeReq %x wValue %x wLength %x wIndex %d line %d\n",
				typeReq, wValue, wLength, wIndex, __LINE__);
#endif
			break;
		case USB_PORT_FEAT_TEST:
			wIndex >>= 8;
			switch (wIndex) {
			case 1:
				printk("TEST_J\n");
				break;
			case 2:
				printk("TEST_K\n");
				break;
			case 3:
				printk("TEST_SE0_NAK\n");
				break;
			case 4:
				printk("TEST_PACKET\n");
				break;
			case 5:
				printk("TEST_FORCE_ENABLE\n");
				break;
			case 6:
				printk("TEST_FIFO_ACCESS\n");
				break;
			default:
				printk("Def err case typeReq %x wValue %x wLength %x wIndex %d line %d\n",
					typeReq, wValue, wLength, wIndex, __LINE__);
				goto error;
				break;
			}
			break;
		default:
			printk("Def err case typeReq %x wValue %x wLength %x wIndex %d line %d\n",
				typeReq, wValue, wLength, wIndex, __LINE__);
			goto error;
			break;
		}
		break;

	default:
error:
		/* "protocol stall" on error */
		retval = -EPIPE;
	}
	spin_unlock_irqrestore(&cdev->lock, flags);

	return retval;
}
int
ast_hub_setup(struct usb_gadget *gadget, const struct usb_ctrlrequest *ctrl)
{
	struct usb_hub_dev        *cdev = get_gadget_data(gadget);
	struct usb_request              *req = cdev->req;
	const struct usb_ctrlrequest *cmd = ctrl;
 	u16		typeReq, wValue, wIndex, wLength;
	char * tbuf = req->buf;
	int		len = 0;
	int		status = 0;
	u8		patch_wakeup = 0;
	u8		patch_protocol = 0;
	const struct usb_descriptor_header	**function;

	typeReq  = (cmd->bRequestType << 8) | cmd->bRequest;
	wValue   = le16_to_cpu (cmd->wValue);
	wIndex   = le16_to_cpu (cmd->wIndex);
	wLength  = le16_to_cpu (cmd->wLength);
#ifdef EXTRA_DEBUG
	printk("ast_hub_setup typeReq %x\n", typeReq);
#endif
	switch (typeReq) {

	/* DEVICE REQUESTS */

	/* The root hub's remote wakeup enable bit is implemented using
	 * driver model wakeup flags.  If this system supports wakeup
	 * through USB, userspace may change the default "allow wakeup"
	 * policy through sysfs or these calls.
	 *
	 * Most root hubs support wakeup from downstream devices, for
	 * runtime power management (disabling USB clocks and reducing
	 * VBUS power usage).  However, not all of them do so; silicon,
	 * board, and BIOS bugs here are not uncommon, so these can't
	 * be treated quite like external hubs.
	 *
	 * Likewise, not all root hubs will pass wakeup events upstream,
	 * to wake up the whole system.  So don't assume root hub and
	 * controller capabilities are identical.
	 */

	case DeviceRequest | USB_REQ_GET_STATUS:
		tbuf [0] = (1 << USB_DEVICE_REMOTE_WAKEUP)
				| (1 << USB_DEVICE_SELF_POWERED);
		tbuf [1] = 0;
		len = 2;
		break;
	case DeviceOutRequest | USB_REQ_CLEAR_FEATURE:
		if (wValue == USB_DEVICE_REMOTE_WAKEUP)
			printk("USB_DEVICE_REMOTE_WAKEUP clear\n");
		else
			printk("Unknown wValue %x line %d\n", __LINE__, wValue);
		break;
	case DeviceOutRequest | USB_REQ_SET_FEATURE:
		if (wValue == USB_DEVICE_REMOTE_WAKEUP) {
#ifdef EXTRA_DEBUG
			printk("USB_DEVICE_REMOTE_WAKEUP set but cannot wakeup\n");
#endif
		}
		else
			printk("Unknown wValue %x line %d\n", __LINE__, wValue);
		break;
	case DeviceRequest | USB_REQ_GET_CONFIGURATION:
		tbuf [0] = 1;
		len = 1;
			/* FALLTHROUGH */
	case DeviceOutRequest | USB_REQ_SET_CONFIGURATION:
		status = usb_ep_disable(cdev->status_ep);
		if(status < 0){
			printk("could not disable EP!\n");
		}
		cdev->status_ep->desc = &hub_ep_desc;
		status = usb_ep_enable(cdev->status_ep);
		if(status < 0){
			printk("could not Enable EP!\n");
		}
//		cdev->hub_disconnect = 0;
		cdev->ast_hub_state = HUB_STEADY;
#ifdef HUB_RESET_DEBUG
		printk("Hub:Setconfig:connect_pending %x devices_present %x reset_in_progress %x\n",
			cdev->connect_pending, cdev->devices_present, cdev->reset_in_progress);
#endif
                req->length = 0;
                req->zero = len < wLength;
                len = usb_ep_queue(gadget->ep0, req, GFP_ATOMIC);
//		mod_timer(&test_timer, jiffies + msecs_to_jiffies(2000));
		break;
	case DeviceRequest | USB_REQ_GET_DESCRIPTOR:
		switch (wValue & 0xff00) {
		case USB_DT_DEVICE << 8:
			len = min(wLength, (u16) sizeof hub_dev_desc);
			memcpy(req->buf, &hub_dev_desc, len);
			ast_hub_dump(req->buf, len);
			patch_protocol = 0;//Set only for Hub having TT No idea what it is and we dont use it
			break;
		case USB_DT_CONFIG << 8:
			function = hub_function;
			if ((wValue & 0xff) > 0) {
				printk("Error %s %d wValue %x\n", __FUNCTION__, __LINE__, wValue);
				return -EINVAL;
			}
			len = hub_usb_gadget_config_buf(&hub_config_desc, req->buf, EP0_BUFSIZE, function);
			if(len < 0) {
				printk("USB_DT_CONFIG error typeReq %x wValue %x wLength %x wIndex %d line %d\n",
					typeReq, wValue, wLength, wIndex, __LINE__);
				status = len;
			}
			if (len >= 0) {
				len = min(wLength, (u16) len);
				ast_hub_dump(req->buf, len);
			}
			patch_wakeup = 0;//Related to Wakeup
			break;
		case USB_DT_STRING << 8:
			len = hub_usb_gadget_get_string(&stringtab,
					wValue & 0xff, req->buf);
			if(len < 0) {
				printk("USB_DT_STRING error typeReq %x wValue %x wLength %x wIndex %d line %d\n",
					typeReq, wValue, wLength, wIndex, __LINE__);
				status = len;
			}
			if (len >= 0) {
				len = min(wLength, (u16) len);
				ast_hub_dump(req->buf, len);
			}
			break;
		default:
			printk("Default case error typeReq %x wValue %x wLength %x wIndex %d line %d\n",
				typeReq, wValue, wLength, wIndex, __LINE__);
			goto error;
		}
		break;
	case DeviceRequest | USB_REQ_GET_INTERFACE:
		tbuf [0] = 0;
		len = 1;
			/* FALLTHROUGH */
	case DeviceOutRequest | USB_REQ_SET_INTERFACE:
		break;
	case DeviceOutRequest | USB_REQ_SET_ADDRESS:
#ifdef EXTRA_DEBUG
		printk("USB_REQ_SET_ADDRESS wValue %x\n", wValue);
#endif
		cdev->hub_dev_address = wValue;
		cdev->ast_hub_state = HUB_STEADY;
		/*TODO: Start the timer as of now its always started*/
		len = 0;
		req->length = len;
		break;

	/* INTERFACE REQUESTS (no defined feature/status flags) */

	/* ENDPOINT REQUESTS */

	case EndpointRequest | USB_REQ_GET_STATUS:
		// ENDPOINT_HALT flag
		tbuf [0] = 0;
		tbuf [1] = 0;
		len = 2;
			/* FALLTHROUGH */
	case EndpointOutRequest | USB_REQ_CLEAR_FEATURE:
	case EndpointOutRequest | USB_REQ_SET_FEATURE:
		printk("no endpoint features yet\n");
		break;

	/* CLASS REQUESTS (and errors) */

	default:
		/* non-generic request */
		switch (typeReq) {
		case GetHubStatus:
		case GetPortStatus:
			len = 4;
			break;
		case GetHubDescriptor:
			len = sizeof (ast_hub_desc);
			len = min(wLength, (u16) len);
			break;
		}
		status = ast_virt_hub_control(gadget, typeReq, wValue, 
			wIndex, tbuf, wLength);
		if(len == 0) {
                req->length = 0;
                req->zero = len < wLength;
                len = usb_ep_queue(gadget->ep0, req, GFP_ATOMIC);
		}
		break;
error:
		/* "protocol stall" on error */
		status = -EPIPE;
	}

	if (status) {
		len = 0;
		printk("status %d error typeReq %x wValue %x wLength %x wIndex %d line %d\n",
			status, typeReq, wValue, wLength, wIndex, __LINE__);
		if (status != -EPIPE) {
		}
	}
	if (len) {
		if(len > wLength) {
			printk(">len%d error typeReq %x wValue %x wLength %x wIndex %d line %d\n",
						len, typeReq, wValue, wLength, wIndex, __LINE__);
		}
                req->length = len;
                req->zero = len < wLength;
                 len = usb_ep_queue(gadget->ep0, req, GFP_ATOMIC);
                if (len < 0) {
                        PHUB_DBG("ep_queue --> %d\n", len);
                        req->status = 0;
                        ast_hub_setup_complete(gadget->ep0, req);
                }
                return 1;
	}

	return status;
}

static void ast_hub_port_status(struct usb_gadget *gadget)
{
	struct usb_hub_dev        *cdev = get_gadget_data(gadget);
	int status = -1, i = 0, devnum = 0;
	int hub_status_changed = 0;
	unsigned long flags;
	int time_in_jiffies = msecs_to_jiffies(1000);

 	 if(cdev->ast_hub_state < HUB_STEADY || cdev->hub_disconnect) {
		//printk("cdev->ast_hub_state %x cdev->hub_disconnect %d cdev->resetdone %x\n", cdev->ast_hub_state, cdev->hub_disconnect, cdev->resetdone);
	 	goto retryAgain;
 	 }
	//printk("cdev->ast_hub_state %x cdev->hub_disconnect %d cdev->resetdone %x\n", cdev->ast_hub_state, cdev->hub_disconnect, cdev->resetdone);
	spin_lock_irqsave(&cdev->lock, flags);
	 /* Take a rough guess dont aquire the lock unnecessarily */
#ifdef HUB_SUSPEND_RESUME
	if(cdev->resetdone || cdev->suspend_changed) {
#else
	if(cdev->resetdone) {
#endif
#ifdef EXTRA_DEBUG
		printk("cdev->resetdone %x\n", cdev->resetdone);
#endif
		goto handleit;
	}
	if(cdev->connect_pending) {
		int retry = 1;
		 /* We can check reset_in_progress to ensure 1 device is complete before we proceed to next
		   */
		if(cdev->reset_in_progress) {
#ifdef EXTRA_DEBUG
			printk("Rerty:cdev->reset_in_progress %x connect_pending %x device_status_pending %x\n",
				cdev->reset_in_progress, cdev->connect_pending, cdev->device_status_pending);
#endif
		 	goto retryAgainLock;
		}
#ifdef EXTRA_DEBUG
	 	printk("Timer:connect_pending %x device_status_pending %x\n",
			cdev->connect_pending, cdev->device_status_pending);
#endif
		for(i = 0;i < MAX_DS_PORTS;i++) {
			if(cdev->connect_pending & (1 << i)) {
				if((cdev->device_status_pending & (1 << i))) {
					retry = 0;
				 	break;
				}
			}
		}
		if(retry) {
			goto retryAgainLock;
		}
	}

	if(cdev->device_status_pending)
		devnum = ffs(cdev->device_status_pending);
 	if(devnum == 0) {
		time_in_jiffies = msecs_to_jiffies(750);
	 	goto retryAgainLock;
 	}
handleit:
	if(cdev->resetdone) {
	 	devnum = ffs(cdev->resetdone);
		cdev->resetdone = 0;
		if((cdev->port_status[(devnum - 1)]) & (USB_PORT_STAT_RESET) ) {
			cdev->port_status[(devnum - 1)] &= ~USB_PORT_STAT_RESET;
#ifdef AST_USB_DEV_SPEED
                    cdev->port_status[(devnum - 1)] |= USB_PORT_STAT_ENABLE | (USB_PORT_STAT_C_RESET << 16);
                    if(hub_speed == USB_SPEED_HIGH)
                            cdev->port_status[(devnum - 1)] |= USB_PORT_STAT_HIGH_SPEED;
#else
			cdev->port_status[(devnum - 1)] |= USB_PORT_STAT_ENABLE | USB_PORT_STAT_HIGH_SPEED
						| (USB_PORT_STAT_C_RESET << 16);
#endif
		} else {
			printk("portstatus:previously reset was not set? status %x\n", cdev->port_status[(devnum - 1)]);
		}

		goto statusSend;
	}
#ifdef HUB_SUSPEND_RESUME
	if(cdev->suspend_changed) {
	 	devnum = ffs(cdev->suspend_changed);
#ifndef PORT_RESUME
		cdev->suspend_changed  &= ~(1 << (devnum - 1));
#else
		if(time_after_eq(jiffies, cdev->suspend_timeout[(devnum - 1)])) {
#ifdef EXTRA_DEBUG
			printk("Clearing %d device suspend_timeout %lx jiffies %lx %d\n",
				devnum - 1, cdev->suspend_timeout[(devnum - 1)], jiffies,
				cdev->port_status[(devnum - 1)] & USB_PORT_STAT_SUSPEND);
#endif
			cdev->suspend_changed  &= ~(1 << (devnum - 1));
		}
#endif
#ifdef PORT_RESUME
		if(time_after_eq(jiffies, cdev->suspend_timeout[(devnum - 1)]) && 
			(cdev->port_status[(devnum - 1)] & USB_PORT_STAT_SUSPEND) ) {
#else
		if((cdev->port_status[(devnum - 1)]) & (USB_PORT_STAT_SUSPEND) ) {
#endif
#ifdef EXTRA_DEBUG
			printk("Resuming %d device\n", devnum - 1);
#endif
			cdev->port_status[(devnum - 1)] |= (USB_PORT_STAT_C_SUSPEND << 16);
			cdev->port_status[(devnum - 1)] &= ~USB_PORT_STAT_SUSPEND;
		} else {
			printk("devnum %d suspend was not set %x?? cdev->suspend_changed %x jiffies %lx %lx\n",
				devnum - 1, cdev->port_status[(devnum - 1)], cdev->suspend_changed,
				jiffies, cdev->suspend_timeout[(devnum - 1)]);
		}
		if(cdev->suspend_changed)//If still suspend changed is set we have to notify the host again in next iteration
			time_in_jiffies = msecs_to_jiffies(1);
		goto statusSend;
	}
#endif
	devnum = 0;
	if(cdev->connect_pending) {
#ifdef EXTRA_DEBUG
	 	printk("Timer:connect_pending %x cdev->device_status_pending %x\n",
			cdev->connect_pending, cdev->device_status_pending);
#endif
		for(i = 0;i < MAX_DS_PORTS;i++) {
			if(cdev->connect_pending & (1 << i)) {
				if((cdev->device_status_pending & (1 << i))) {
				 	devnum = i + 1;
				}
				/* There is already connect pending so dont handle it now */
				/* Basically this is to ensure that 1 port is reset completely
				  * before we inform host about next device connect, In this 
				  * way we maintain order
				  */
				if((cdev->device_status_pending & (1 << i)) == 0)
				 	goto retryAgainLock;
			}
		}
		if(devnum) {
			if((cdev->port_status[devnum - 1] & USB_PORT_STAT_CONNECTION) == 0)
				cdev->port_status[devnum - 1] |= USB_PORT_STAT_CONNECTION | (USB_PORT_STAT_C_CONNECTION << 16);
			else
				cdev->port_status[devnum - 1] |= USB_PORT_STAT_CONNECTION;
		}
	}
	/* Any other status? */
	if(devnum == 0 && cdev->device_status_pending)
		devnum = ffs(cdev->device_status_pending);
statusSend:
	 if(devnum != 0) {
//	 	cdev->connect_or_reset_in_progress = 1;
		hub_status_changed = 1 << (devnum + 0);
#ifdef EXTRA_DEBUG
		printk("hub_status_changed %x devnum %d\n", hub_status_changed, devnum);
#endif
		memcpy(cdev->status_req->buf, &hub_status_changed, 1);
		cdev->status_req->length = 1;
		if(0 && cdev->status_req->status && cdev->status_req->status != -ECONNRESET) {
			printk("previous status still pending:cdev->status_req->status %d devnum %d time %lu\n",
				cdev->status_req->status, devnum - 1, jiffies);
		}
		if((cdev->port_status[(devnum - 1)] & PORT_C_MASK) != 0) {
//				if((cdev->port_status[(devnum - 1)] & PORT_C_MASK) == 0)
				status = usb_ep_queue(cdev->status_ep, cdev->status_req,  GFP_KERNEL);
#ifdef HUB_RESET_DEBUG
				if(last_port_status[(devnum - 1)] == cdev->port_status[(devnum - 1)]) {
					printk("HubIssue?:%d %x == %x last_ep_queue %d devnum %d device_clear_status_pending %x\n",
						__LINE__, last_port_status[(devnum - 1)], cdev->port_status[(devnum - 1)],
						last_ep_queue, (devnum - 1), cdev->device_clear_status_pending);
					printk("last/current status %d jiffies %lu last_time_in_jiffies %llu\n", status , jiffies, last_time_in_jiffies);
					last_device = (devnum - 1);
					print_last_device_debug_stats(cdev, (devnum - 1));
				}
				update_debug_stats(cdev, 1, status, (devnum - 1), hub_status_changed);
				last_ep_queue = 1;
#endif
			if(status < 0) {
				printk("ast_hub_port_status: usb_ep_queue failed!!status %d\n", status);
		                printk("ast_hub_disconnect cdev->devices_present %x cdev->connect_pending %x cdev->reset_in_progress %x\n",
	       	                 cdev->devices_present, cdev->connect_pending, cdev->reset_in_progress);
				/* Try after 5s or 20ms, after completion immmediately timer will be called though */
				if(status == -EBUSY)
					time_in_jiffies = msecs_to_jiffies(5000);
				else
					time_in_jiffies = msecs_to_jiffies(20);
				cdev->device_status_pending |= (1 << (devnum - 1));
			} else {
#ifdef EXTRA_DEBUG
	                        if(cdev->device_clear_status_pending)
	                                printk("cdev->device_clear_status_pending already set %x devnum %d cdev->device_status_pending %x\n",
	                                        cdev->device_clear_status_pending, devnum, cdev->device_status_pending);
				printk("portstatus:ep_q cdev->resetdone %0x devnum %d port_status %x\n",
					cdev->resetdone, (devnum - 1), cdev->port_status[(devnum - 1)]);
#endif
	                        //Make sure to clear device_status_pending only on completion
				//cdev->device_status_pending &= ~(1 << (devnum - 1));
	                        cdev->device_clear_status_pending = (1 << (devnum - 1));
			}
		 } else {
#ifdef EXTRA_DEBUG
			printk("portstatus not changed?:%d %x devnum %d device_status_pending %x resetdone 0x%x device_clear_status_pending %x\n",
						__LINE__, cdev->port_status[(devnum - 1)], (devnum - 1), 
						cdev->device_status_pending, cdev->resetdone, cdev->device_clear_status_pending);
			printk("last/current status %d jiffies %lu\n", status , jiffies);
#endif
			cdev->device_status_pending &= ~(1 << (devnum - 1));
		 }
	 }
retryAgainLock:
#ifdef HUB_RESET_DEBUG
	last_time_in_jiffies = jiffies;
#endif
	spin_unlock_irqrestore(&cdev->lock, flags);
retryAgain:

	status_change_timer.data = (unsigned long)cdev->gadget;
	status_change_timer.function = (void (*)(unsigned long))ast_hub_port_status;

	mod_timer(&status_change_timer, jiffies + time_in_jiffies);

	return;
}
static int hub_status_change_issues_with_host = 1;
static void ast_hub_disconnect(struct usb_gadget *gadget)
{
	struct usb_hub_dev        *cdev = get_gadget_data(gadget);
	int i = 0;
	unsigned long flags;

#ifdef EXTRA_DEBUG
 	printk("ast_hub_disconnect cdev->devices_present %x\n", cdev->devices_present);
#endif
	if(cdev->hub_disconnect) {
		printk("ast_hub_disconnect cdev->devices_present %x cdev->connect_pending %x cdev->reset_in_progress %x\n",
			cdev->devices_present, cdev->connect_pending, cdev->reset_in_progress);
	}
	spin_lock_irqsave(&cdev->lock, flags);
	cdev->ast_hub_state = HUB_IDLE;
#ifdef HUB_RESET_DEBUG
	print_reset_debug_stats(cdev);
#endif
 	cdev->hub_disconnect = 1;
	for(i = 0;i < MAX_DS_PORTS;i++) {
		if(cdev->devices_present & (1 << i)) {
			if(!hub_status_change_issues_with_host)
				handle_port_reset(i, gadget);
		}
		/* Disable all the ports under the Hub */
		port_disable(i , gadget);
		cdev->port_status[i] = USB_PORT_STAT_POWER;
		if(hub_status_change_issues_with_host && (cdev->devices_present & (1 << i))) {
			cdev->port_status[i] |= USB_PORT_STAT_CONNECTION | (USB_PORT_STAT_C_CONNECTION << 16);
			cdev->device_status_pending |= (1 << i);
		}
	}
	cdev->resetdone = 0;
	cdev->reset_in_progress = 0;
	cdev->connect_pending = 0;
	if(!hub_status_change_issues_with_host)
		cdev->device_status_pending = 0;
	cdev->device_clear_status_pending = 0;
	if(!cdev->devices_present || hub_status_change_issues_with_host)
		cdev->hub_disconnect = 0;
	if(hub_status_change_issues_with_host && cdev->devices_present)
		mod_timer(&status_change_timer, jiffies + msecs_to_jiffies(500));
	spin_unlock_irqrestore(&cdev->lock, flags);
	/* TODO start timer , as of now timer is always running */
}
static struct usb_gadget_driver ast_hub_driver = {
        .max_speed          = USB_SPEED_HIGH,
	  .bind            = ast_hub_bind,
        .unbind         = ast_hub_unbind,
        .setup          = ast_hub_setup,
        .disconnect     = ast_hub_disconnect,
        .reset     = ast_hub_disconnect,
        .suspend        = ast_hub_suspend,
        .resume         = ast_hub_resume,
        .driver = {
	  .name		= (char *) "Aspeed Usb Hub",
	  .owner          = THIS_MODULE,
        },
};
int hub_registered;
void ast_hub_reset(struct usb_gadget *gadget)
{
	struct usb_hub_dev        *cdev = get_gadget_data(gadget);
	if(hub_registered)
		cdev->ast_hub_state = HUB_IDLE;
	return;	
}
void hub_reset_done(int devnum, struct usb_gadget *gadget)
{
	struct usb_hub_dev        *cdev = get_gadget_data(gadget);
	unsigned long flags;
	int hub_status_changed = 0;

	hub_status_changed = 1 << (devnum + 1);
#ifdef EXTRA_DEBUG
	printk("hub_reset_done: devnum %d cdev->hub_disconnect %d reset_in_progress %x\n",
		devnum, cdev->hub_disconnect, cdev->reset_in_progress);
#endif
	spin_lock_irqsave(&cdev->lock, flags);
	if(cdev->hub_disconnect) {
		/* Wait for all the devices to finish reset before we inform Host */
		/* TODO: If there is a faulty device then no device will be enumerated
		  * So put a large timer for the same, will do later.
		  */
		cdev->connect_pending |= (1 << devnum);
		if(cdev->devices_present == cdev->connect_pending){
			cdev->hub_disconnect = 0;
			cdev->device_status_pending = cdev->devices_present;
		}
		goto handleLater;
	}
	if(cdev->resetdone & ~(1 << devnum))
		printk("previos cdev->resetdone %x???\n", cdev->resetdone);
	if(cdev->reset_in_progress & (1 << devnum)) {
		mod_timer(&status_change_timer, jiffies + msecs_to_jiffies(1));
		cdev->resetdone = (1 << devnum);
		port_enable(devnum, gadget);
	}

handleLater:
	spin_unlock_irqrestore(&cdev->lock, flags);
}
#ifdef USE_MAILBOX

void hub_2_dev_mb_intr(int devnum, unsigned int mb_code, struct usb_gadget *gadget)
{
#define DEVICE_UP_AND_RUNNING 0x1
#ifdef HUB_SUSPEND_RESUME
#define DEVICE_RESUME 0x2
#endif
	struct usb_hub_dev        *cdev = get_gadget_data(gadget);
	unsigned long flags;
#ifdef HUB_RESET_DEBUG
	printk("Mailbox intr rcvd from dev %d code %x\n", devnum, mb_code);
#endif
	spin_lock_irqsave(&cdev->lock, flags);

	if(mb_code & DEVICE_UP_AND_RUNNING) {
#ifdef HUB_RESET_DEBUG
		printk("for dev %d connect_pending %x reset_in_progress %x\n", 
			devnum, cdev->connect_pending, cdev->reset_in_progress);
#endif
		cdev->reset_in_progress &= ~(1 << devnum);
		clear_mb_code(devnum, DEVICE_UP_AND_RUNNING, gadget);
	}

#ifdef HUB_SUSPEND_RESUME
	if(mb_code & DEVICE_RESUME) {
		if((cdev->port_status[devnum] & USB_PORT_STAT_SUSPEND)) {
			cdev->suspend_changed |= (1 << devnum);
			cdev->suspend_timeout[devnum] = jiffies + msecs_to_jiffies(0);
			mod_timer(&status_change_timer, jiffies + msecs_to_jiffies(1));
		}
		clear_mb_code(devnum, DEVICE_RESUME, gadget);
	}
#endif
	spin_unlock_irqrestore(&cdev->lock, flags);

}
#else
void set_address_rcvd(int devnum, struct usb_gadget *gadget)
{
	struct usb_hub_dev        *cdev = get_gadget_data(gadget);
	unsigned long flags;
	devnum -= 1;
#ifdef EXTRA_DEBUG
	printk("Hub:set_address_rcvd for dev %d connect_pending %x reset_in_progress %x\n", 
		devnum, cdev->connect_pending, cdev->reset_in_progress);
#endif
	spin_lock_irqsave(&cdev->lock, flags);
	cdev->reset_in_progress &= ~(1 << devnum);
	spin_unlock_irqrestore(&cdev->lock, flags);
	/* TODO We can start a timer immediately to process other updates */
}

static void ast_check_set_address(struct usb_gadget *gadget)
{
	struct usb_hub_dev        *cdev = get_gadget_data(gadget);
	extern int get_far(int );
#ifdef EXTRA_DEBUG
	printk("ast_check_set_address cdev->reset_in_progress %x\n", cdev->reset_in_progress);
#endif
	if(cdev->reset_in_progress) {
		int reset_devnum = ffs(cdev->reset_in_progress);
		int far = get_far(reset_devnum);
		if(far) {
			set_address_rcvd(reset_devnum, gadget);
		}
	}
	/* if there is reset in progress keep on polling till there is no reset */
	if(cdev->reset_in_progress) {
		mod_timer(&set_address_timer, jiffies + msecs_to_jiffies(20));
	}
}
#endif

void hub_device_connect_disconnect(int devnum, struct usb_gadget *gadget, int conn)
{
	struct usb_hub_dev        *cdev = get_gadget_data(gadget);
	unsigned long flags;
	int hub_status_changed = 0;
	u32 temp;

	hub_status_changed = 1 << (devnum + 1);
#ifdef EXTRA_DEBUG
	printk("hub_device_connect_disconnect:cdev->hub_disconnect %d devnum %d connect_pending %x devices_present %x conn %d\n",
		cdev->hub_disconnect, devnum, cdev->connect_pending, cdev->devices_present, conn);
#endif
	spin_lock_irqsave(&cdev->lock, flags);
	if(unlikely(cdev->ast_hub_state < HUB_STEADY)) {
		if(conn) {
			cdev->connect_pending |= (1 << devnum);
			cdev->devices_present |= (1 << devnum);
		}else {
			cdev->connect_pending &= ~(1 << devnum);
			cdev->devices_present &= ~(1 << devnum);
		}
		if(!cdev->hub_disconnect)
		 	cdev->device_status_pending |= (1 << devnum);
		spin_unlock_irqrestore(&cdev->lock, flags);
		return;
	}
	if(unlikely(cdev->hub_disconnect)) {
		printk("someone insmoded a device while there was a reset conn %d devnum %d\n", conn, devnum);
	}
	if(conn) {
		temp = cdev->connect_pending;
		cdev->devices_present |= (1 << devnum);
		cdev->connect_pending |= (1 << devnum);
#ifdef EXTRA_DEBUG
		printk("cdev->reset_in_progress %d temp %d %d\n", cdev->reset_in_progress, temp, cdev->hub_disconnect);
#endif
		if(!temp && !cdev->reset_in_progress && !cdev->hub_disconnect) {
#ifdef EXTRA_DEBUG
			printk("cdev->port_status[%d] %x\n", devnum, cdev->port_status[devnum]);
#endif
			if(!cdev->resetdone) {
				mod_timer(&status_change_timer, jiffies + msecs_to_jiffies(1));
			}
		 	cdev->device_status_pending |= (1 << devnum);
			goto handleLater;
		} else {
			if(!cdev->hub_disconnect)
				cdev->device_status_pending |= (1 << devnum);
		}
	} else {
		cdev->devices_present &= ~(1 << devnum);
		if((cdev->port_status[devnum] & USB_PORT_STAT_CONNECTION)) {
			if(cdev->connect_pending & (1 << devnum))
				printk("connect_pending set for dev %d?\n", devnum);
			cdev->connect_pending &= ~(1 << devnum);
			cdev->port_status[devnum] &= ~(USB_PORT_STAT_CONNECTION |
						USB_PORT_STAT_ENABLE |
						USB_PORT_STAT_LOW_SPEED |
						USB_PORT_STAT_HIGH_SPEED |
						USB_PORT_STAT_SUSPEND);
			cdev->port_status[devnum] |= (USB_PORT_STAT_C_CONNECTION << 16);
			if(!cdev->resetdone) {
				mod_timer(&status_change_timer, jiffies + msecs_to_jiffies(200));
			}
		 	cdev->device_status_pending |= (1 << devnum);
			goto handleLater;
		} else {
			/* Sometimes is there is a previous reset_in_progress and we do a insmod of a new module(device)
			  * and then u do rmmod of new device then we may need to clear connect_pending and
			  * device_status_pending ,otherwise s/w will think there is still new connect pending
			  */
			if(cdev->connect_pending & (1 << devnum)) {
#ifdef EXTRA_DEBUG
				printk("connect_pending set for dev %d? connect_pending %x device_status_pending %x\n",
					devnum, cdev->connect_pending, cdev->device_status_pending);
#endif
			}
			cdev->connect_pending &= ~(1 << devnum);
			cdev->device_status_pending &= ~(1 << devnum);
		}
		if(cdev->reset_in_progress  & (1 << devnum)) {
#ifdef EXTRA_DEBUG
			printk("There was a resetinprogress set for device %d but now got disconnect"
				"before hub could get MB interrupt\n", devnum);
#endif
			cdev->reset_in_progress &= ~(1 << devnum);
		}
		port_disable(devnum, gadget);
	}
handleLater:
	spin_unlock_irqrestore(&cdev->lock, flags);
}

//int __init ast_hub_init(int reg)
int ast_hub_init(int reg)
{
	int i = 0;
	if(reg) {
		hub_registered = 1 ;
		i = usb_gadget_probe_driver(&ast_hub_driver);
#ifdef EXTRA_DEBUG
		printk("Hub usb_gadget_probe_driver ret %d\n", i);
#endif
	}
	return 0;
}

void __exit ast_hub_exit(void)
{
	usb_gadget_unregister_driver(&ast_hub_driver);
} 
