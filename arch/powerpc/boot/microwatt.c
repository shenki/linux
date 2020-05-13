// SPDX-License-Identifier: GPL-2.0-or-later

#include <stddef.h>
#include "stdio.h"
#include "types.h"
#include "io.h"
#include "ops.h"

BSS_STACK(8192);

#define UART_BASE       0xc0002000
#define POTATO_CONSOLE_TX               0x00
#define POTATO_CONSOLE_RX               0x08
#define POTATO_CONSOLE_STATUS           0x10
#define   POTATO_CONSOLE_STATUS_RX_EMPTY                0x01
#define   POTATO_CONSOLE_STATUS_TX_EMPTY                0x02
#define   POTATO_CONSOLE_STATUS_RX_FULL                 0x04
#define   POTATO_CONSOLE_STATUS_TX_FULL                 0x08
#define POTATO_CONSOLE_CLOCK_DIV        0x18
#define POTATO_CONSOLE_IRQ_EN           0x20


static inline void writeq(uint64_t val, unsigned long addr)
{
        __asm__ volatile("sync; stdcix %0,0,%1" : : "r" (val), "r" (addr) : "memory");
}


static inline uint64_t readq(unsigned long addr)
{
        uint64_t val;
        __asm__ volatile("sync; ldcix %0,0,%1" : "=r" (val) : "r" (addr) : "memory");
        return val;
}


static uint64_t potato_uart_base = 0xc0002000;

static uint64_t potato_uart_reg_read(int offset)
{
        return readq(potato_uart_base + offset);
}

static void potato_uart_reg_write(int offset, uint64_t val)
{
        writeq(val, potato_uart_base + offset);
}

static void potato_uart_write(char c)
{
        uint64_t val;

        val = c;

        potato_uart_reg_write(POTATO_CONSOLE_TX, val);
}


static int potato_uart_tx_full(void)
{
        uint64_t val;

        val = potato_uart_reg_read(POTATO_CONSOLE_STATUS);

        if (val & POTATO_CONSOLE_STATUS_TX_FULL)
                return 1;

        return 0;
}

static void putchar(unsigned char c)
{
        while (potato_uart_tx_full())
                /* Do Nothing */;

        potato_uart_write(c);
}


static void putstr(const char *str, int len)
{
        for (unsigned long i = 0; i < len; i++) {
                putchar(str[i]);
        }
}

void platform_init(unsigned long r3, unsigned long r4, unsigned long r5)
{
	unsigned long heapsize = 16*1024*1024 - (unsigned long)_end;

	simple_alloc_init(_end, heapsize, 32, 64);
	fdt_init(_dtb_start);

	console_ops.write = putstr;
}

