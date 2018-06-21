/*
*  support.c - support functions for AST USB HUB
*
*
*  Copyright (C) 2017-2020  ASPEED Technology Inc.
*
*  This program is free software; you can redistribute it and/or modify
*  it under the terms of the GNU General Public License version 2 as
*  published by the Free Software Foundation.
*
*  History:
*    2017.10.24: Initial version [Akshay Srinivas]
*/

#include <linux/nls.h>
#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>
#include <linux/usb/composite.h>

int
hub_usb_descriptor_fillbuf(void *buf, unsigned buflen,
		const struct usb_descriptor_header **src)
{
	u8	*dest = buf;

	if (!src)
		return -EINVAL;

	/* fill buffer from src[] until null descriptor ptr */
	for (; NULL != *src; src++) {
		unsigned		len = (*src)->bLength;

		if (len > buflen)
			return -EINVAL;
		memcpy(dest, *src, len);
		buflen -= len;
		dest += len;
	}
	return dest - (u8 *)buf;
}

int hub_usb_gadget_config_buf(
	const struct usb_config_descriptor	*config,
	void					*buf,
	unsigned				length,
	const struct usb_descriptor_header	**desc
)
{
	struct usb_config_descriptor		*cp = buf;
	int					len;

	/* config descriptor first */
	if (length < USB_DT_CONFIG_SIZE || !desc)
		return -EINVAL;
	*cp = *config;

	/* then interface/endpoint/class/vendor/... */
	len = hub_usb_descriptor_fillbuf(USB_DT_CONFIG_SIZE + (u8*)buf,
			length - USB_DT_CONFIG_SIZE, desc);
	if (len < 0)
		return len;
	len += USB_DT_CONFIG_SIZE;
	if (len > 0xffff)
		return -EINVAL;

	/* patch up the config descriptor */
	cp->bLength = USB_DT_CONFIG_SIZE;
	cp->bDescriptorType = USB_DT_CONFIG;
	cp->wTotalLength = cpu_to_le16(len);
	cp->bmAttributes |= USB_CONFIG_ATT_ONE;
	return len;
}

int
hub_usb_gadget_get_string (struct usb_gadget_strings *table, int id, u8 *buf)
{
	struct usb_string	*s;
	int			len;

	/* descriptor 0 has the language id */
	if (id == 0) {
		buf [0] = 4;
		buf [1] = USB_DT_STRING;
		buf [2] = (u8) table->language;
		buf [3] = (u8) (table->language >> 8);
		return 4;
	}
	for (s = table->strings; s && s->s; s++)
		if (s->id == id)
			break;

	/* unrecognized: stall. */
	if (!s || !s->s) {
		printk("hub_usb_gadget_get_string error %p id %d %x\n", s, id, id);
		for (s = table->strings; s && s->s; s++) {
			printk("s->id %u  s->s %s\n", s->id, s->s);
		}
		return -EINVAL;
	}

	/* string descriptors have length, tag, then UTF16-LE text */
	len = min ((size_t) 126, strlen (s->s));
	len = utf8s_to_utf16s(s->s, len, UTF16_LITTLE_ENDIAN,
			(wchar_t *) &buf[2], 126);
	if (len < 0) {
		printk("hub_usb_gadget_get_string error len  %d id %d %x\n", len, id, id);
		for (s = table->strings; s && s->s; s++) {
			printk("s->id %u  s->s %s\n", s->id, s->s);
		}
		return -EINVAL;
	}
	buf [0] = (len + 1) * 2;
	buf [1] = USB_DT_STRING;
	return buf [0];
}

