/*
 * Copyright (C) Hasjim Williams
 * Authors:  Hasjim Williams <@>
 * License terms:  GNU General Public License (GPL), version 2
 */

#define DRIVER_NAME "uart-litex"

struct litex_uart_offsets {
	u8 txfull;
	u8 rxempty;
	u8 ev_status;
	u8 ev_pending;
	u8 ev_enable;
};

struct litex_uart_config {
};

struct litex_uart_info {
	struct litex_uart_offsets ofs;
#if 0
	struct litex_uart_config cfg;
#endif
};

#define UNDEF_REG 0xff

/* Register offsets */
struct litex_uart_info litex_info = {
	.ofs = {
		.txfull = 0x04,
		.rxempty = 0x08,
		.ev_status = 0x0c,
		.ev_pending = 0x10,
		.ev_enable = 0x14,
	},
#if 0
	.cfg = {
	}
#endif
};

#define LITEX_SERIAL_NAME "ttySLX"
#define LITEX_MAX_PORTS 16

struct litex_port {
	struct uart_port port;
	struct litex_uart_info *info;
};

static struct litex_port litex_ports[LITEX_MAX_PORTS];
static struct uart_driver litex_uart_driver;
