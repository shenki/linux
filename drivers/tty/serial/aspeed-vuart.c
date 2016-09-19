/*
 *  Serial Port driver for Aspeed VUART device
 *
 *    Copyright (C) 2016 Jeremy Kerr <jk@ozlabs.org>, IBM Corp.
 *    Copyright (C) 2006 Arnd Bergmann <arnd@arndb.de>, IBM Corp.
 *
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License
 *  as published by the Free Software Foundation; either version
 *  2 of the License, or (at your option) any later version.
 *
 */
#include <linux/device.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/clk.h>

#include "8250/8250.h"

#define AST_VUART_GCRA		0x20
#define AST_VUART_GCRA_VUART_EN		0x01
#define AST_VUART_GCRA_HOST_TX_DISCARD	0x20
#define AST_VUART_GCRB		0x24
#define AST_VUART_GCRB_HOST_SIRQ_MASK	0xf0
#define AST_VUART_GCRB_HOST_SIRQ_SHIFT	4
#define AST_VUART_ADDRL		0x28
#define AST_VUART_ADDRH		0x2c

struct ast_vuart {
	struct platform_device *pdev;
	void __iomem		*regs;
	struct clk		*clk;
	int			line;
};

static ssize_t ast_vuart_show_addr(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct ast_vuart *vuart = dev_get_drvdata(dev);
	u16 addr;

	addr = (readb(vuart->regs + AST_VUART_ADDRH) << 8) |
		(readb(vuart->regs + AST_VUART_ADDRL));

	return snprintf(buf, PAGE_SIZE - 1, "0x%x\n", addr);
}

static ssize_t ast_vuart_set_addr(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct ast_vuart *vuart = dev_get_drvdata(dev);
	unsigned long val;
	int err;

	err = kstrtoul(buf, 0, &val);
	if (err)
		return err;

	writeb((val >> 8) & 0xff, vuart->regs + AST_VUART_ADDRH);
	writeb((val >> 0) & 0xff, vuart->regs + AST_VUART_ADDRL);

	return count;
}

static DEVICE_ATTR(lpc_address, S_IWUSR | S_IRUGO,
		ast_vuart_show_addr, ast_vuart_set_addr);

static ssize_t ast_vuart_show_sirq(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct ast_vuart *vuart = dev_get_drvdata(dev);
	u8 reg;

	reg = readb(vuart->regs + AST_VUART_GCRB);
	reg &= AST_VUART_GCRB_HOST_SIRQ_MASK;
	reg >>= AST_VUART_GCRB_HOST_SIRQ_SHIFT;

	return snprintf(buf, PAGE_SIZE - 1, "%u\n", reg);
}

static ssize_t ast_vuart_set_sirq(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct ast_vuart *vuart = dev_get_drvdata(dev);
	unsigned long val;
	int err;
	u8 reg;

	err = kstrtoul(buf, 0, &val);
	if (err)
		return err;

	val <<= AST_VUART_GCRB_HOST_SIRQ_SHIFT;
	val &= AST_VUART_GCRB_HOST_SIRQ_MASK;

	reg = readb(vuart->regs + AST_VUART_GCRB);
	reg &= ~AST_VUART_GCRB_HOST_SIRQ_MASK;
	reg |= val;
	writeb(reg, vuart->regs + AST_VUART_GCRB);

	return count;
}

static DEVICE_ATTR(sirq, S_IWUSR | S_IRUGO,
		ast_vuart_show_sirq, ast_vuart_set_sirq);

static void ast_vuart_set_enabled(struct ast_vuart *vuart, bool enabled)
{
	u8 reg;

	reg = readb(vuart->regs + AST_VUART_GCRA);
	reg &= ~AST_VUART_GCRA_VUART_EN;
	if (enabled)
		reg |= AST_VUART_GCRA_VUART_EN;
	writeb(reg, vuart->regs + AST_VUART_GCRA);
}

static void ast_vuart_set_host_tx_discard(struct ast_vuart *vuart, bool discard)
{
	u8 reg;

	reg = readb(vuart->regs + AST_VUART_GCRA);

	/* if the HOST_TX_DISCARD bit is set, discard is *disabled* */
	reg &= ~AST_VUART_GCRA_HOST_TX_DISCARD;
	if (!discard)
		reg |= AST_VUART_GCRA_HOST_TX_DISCARD;

	writeb(reg, vuart->regs + AST_VUART_GCRA);
}

static int ast_vuart_startup(struct uart_port *uart_port)
{
	struct uart_8250_port *uart_8250_port = up_to_u8250p(uart_port);
	struct ast_vuart *vuart = uart_8250_port->port.private_data;
	int rc;

	rc = serial8250_do_startup(uart_port);
	if (rc)
		return rc;

	ast_vuart_set_host_tx_discard(vuart, false);

	return 0;
}

static void ast_vuart_shutdown(struct uart_port *uart_port)
{
	struct uart_8250_port *uart_8250_port = up_to_u8250p(uart_port);
	struct ast_vuart *vuart = uart_8250_port->port.private_data;

	ast_vuart_set_host_tx_discard(vuart, true);

	serial8250_do_shutdown(uart_port);
}


/**
 * The device tree parsing code here is heavily based on that of the of_serial
 * driver, but we have a few core differences, as we need to use our own
 * ioremapping for extra register support
 */
static int ast_vuart_probe(struct platform_device *pdev)
{
	struct uart_8250_port port;
	struct resource resource;
	struct ast_vuart *vuart;
	struct device_node *np;
	u32 clk, prop;
	int rc;

	np = pdev->dev.of_node;

	vuart = devm_kzalloc(&pdev->dev, sizeof(*vuart), GFP_KERNEL);
	if (!vuart)
		return -ENOMEM;

	vuart->pdev = pdev;
	rc = of_address_to_resource(np, 0, &resource);
	if (rc) {
		dev_warn(&pdev->dev, "invalid address\n");
		return rc;
	}

	/* create our own mapping for VUART-specific registers */
	vuart->regs = devm_ioremap_resource(&pdev->dev, &resource);
	if (IS_ERR(vuart->regs)) {
		dev_warn(&pdev->dev, "failed to map registers\n");
		return PTR_ERR(vuart->regs);
	}

	memset(&port, 0, sizeof(port));
	port.port.private_data = vuart;
	port.port.membase = vuart->regs;
	port.port.mapbase = resource.start;
	port.port.mapsize = resource_size(&resource);
	port.port.startup = ast_vuart_startup;
	port.port.shutdown = ast_vuart_shutdown;

	if (of_property_read_u32(np, "clock-frequency", &clk)) {
		vuart->clk = devm_clk_get(&pdev->dev, NULL);
		if (IS_ERR(vuart->clk)) {
			dev_warn(&pdev->dev,
				"clk or clock-frequency not defined\n");
			return PTR_ERR(vuart->clk);
		}

		rc = clk_prepare_enable(vuart->clk);
		if (rc < 0)
			return rc;

		clk = clk_get_rate(vuart->clk);
	}

	/* If current-speed was set, then try not to change it. */
	if (of_property_read_u32(np, "current-speed", &prop) == 0)
		port.port.custom_divisor = clk / (16 * prop);

	/* Check for shifted address mapping */
	if (of_property_read_u32(np, "reg-offset", &prop) == 0)
		port.port.mapbase += prop;

	/* Check for registers offset within the devices address range */
	if (of_property_read_u32(np, "reg-shift", &prop) == 0)
		port.port.regshift = prop;

	/* Check for fifo size */
	if (of_property_read_u32(np, "fifo-size", &prop) == 0)
		port.port.fifosize = prop;

	/* Check for a fixed line number */
	rc = of_alias_get_id(np, "serial");
	if (rc >= 0)
		port.port.line = rc;

	port.port.irq = irq_of_parse_and_map(np, 0);
	port.port.iotype = UPIO_MEM;
	if (of_property_read_u32(np, "reg-io-width", &prop) == 0) {
		switch (prop) {
		case 1:
			port.port.iotype = UPIO_MEM;
			break;
		case 4:
			port.port.iotype = of_device_is_big_endian(np) ?
				       UPIO_MEM32BE : UPIO_MEM32;
			break;
		default:
			dev_warn(&pdev->dev, "unsupported reg-io-width (%d)\n",
				 prop);
			rc = -EINVAL;
			goto err_clk_disable;
		}
	}

	port.port.type = PORT_16550A;
	port.port.uartclk = clk;
	port.port.flags = UPF_SHARE_IRQ | UPF_BOOT_AUTOCONF
		| UPF_FIXED_PORT | UPF_FIXED_TYPE;

	if (of_find_property(np, "no-loopback-test", NULL))
		port.port.flags |= UPF_SKIP_TEST;

	port.port.dev = &pdev->dev;

	if (port.port.fifosize)
		port.capabilities = UART_CAP_FIFO;

	if (of_property_read_bool(pdev->dev.of_node,
				  "auto-flow-control"))
		port.capabilities |= UART_CAP_AFE;

	rc = serial8250_register_8250_port(&port);
	if (rc < 0)
		goto err_clk_disable;


	vuart->line = rc;
	ast_vuart_set_enabled(vuart, true);
	ast_vuart_set_host_tx_discard(vuart, true);
	platform_set_drvdata(pdev, vuart);

	/* extra sysfs control */
	rc = device_create_file(&pdev->dev, &dev_attr_lpc_address);
	if (rc)
		dev_warn(&pdev->dev, "can't create lpc_address file\n");
	rc = device_create_file(&pdev->dev, &dev_attr_sirq);
	if (rc)
		dev_warn(&pdev->dev, "can't create sirq file\n");

	return 0;

err_clk_disable:
	if (vuart->clk)
		clk_disable_unprepare(vuart->clk);

	irq_dispose_mapping(port.port.irq);
	return rc;
}

static int ast_vuart_remove(struct platform_device *pdev)
{
	struct ast_vuart *vuart = platform_get_drvdata(pdev);

	ast_vuart_set_enabled(vuart, false);

	if (vuart->clk)
		clk_disable_unprepare(vuart->clk);
	return 0;
}

static const struct of_device_id ast_vuart_table[] = {
	{ .compatible = "aspeed,vuart" },
	{ },
};

static struct platform_driver ast_vuart_driver = {
	.driver = {
		.name = "aspeed-vuart",
		.of_match_table = ast_vuart_table,
	},
	.probe = ast_vuart_probe,
	.remove = ast_vuart_remove,
};

module_platform_driver(ast_vuart_driver);

MODULE_AUTHOR("Jeremy Kerr <jk@ozlabs.org>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Driver for Aspeed VUART device");
