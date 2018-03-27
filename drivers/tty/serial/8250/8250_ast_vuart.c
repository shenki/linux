/*
 *  Serial Port driver for Aspeed VUART device
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

#include "8250.h"
/*********************************************************************************************/
/*AST VUART Register Definition */
#define AST_VUART_CTRLA			0x20
#define AST_VUART_CTRLB			0x24
#define AST_VUART_ADDRL			0x28
#define AST_VUART_ADDRH			0x2C
#define AST_VUART_CTRLE				0x30
#define AST_VUART_CTRLF				0x34
#define AST_VUART_CTRLG			0x38
#define AST_VUART_CTRLH			0x3C

/* AST_VUART_CTRLA			0x20 */
#define VUART_ENABLE					(1 << 0)
#define VUART_SIRQ_POLARITY			(1 << 1)
#define VUART_DISABLE_H_TX_DISCARD	(1 << 5)

/* AST_VUART_CTRLB			0x24 */
#define VUART_SET_SIRQ(x)			(x << 4)
#define VUART_SIRQ_MASK			(0xf << 4)
#define VUART_GET_SIRQ(x)			((x >> 4) & 0xf)
/*********************************************************************************************/
struct ast_vuart {
	struct platform_device *pdev;
	void __iomem		*regs;
	struct clk		*clk;
	int				line;
};

static ssize_t ast_vuart_show_addr(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct ast_vuart *vuart = dev_get_drvdata(dev);

	return sprintf(buf, "0x%x\n", (readl(vuart->regs + AST_VUART_ADDRH) << 8) | (readl(vuart->regs + AST_VUART_ADDRL)));
}

static ssize_t ast_vuart_store_addr(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct ast_vuart *vuart = dev_get_drvdata(dev);
	u32 val;
	
	val = simple_strtoul(buf, NULL, 10);

	writel((val >> 8) & 0xff, vuart->regs + AST_VUART_ADDRH);
	writel((val >> 0) & 0xff, vuart->regs + AST_VUART_ADDRL);

	return count;
}

static DEVICE_ATTR(port_address, S_IWUSR | S_IRUGO,
		ast_vuart_show_addr, ast_vuart_store_addr);

static ssize_t ast_vuart_show_sirq(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct ast_vuart *vuart = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", VUART_GET_SIRQ(readl(vuart->regs + AST_VUART_CTRLB)));
}

static ssize_t ast_vuart_store_sirq(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct ast_vuart *vuart = dev_get_drvdata(dev);
	u32 val;

	val = simple_strtoul(buf, NULL, 10);
	writel((readl(vuart->regs + AST_VUART_CTRLB) & ~VUART_SIRQ_MASK) | VUART_SET_SIRQ(val), vuart->regs);

	return count;
}

static DEVICE_ATTR(sirq, S_IWUSR | S_IRUGO,
		ast_vuart_show_sirq, ast_vuart_store_sirq);

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
	u32 port_address, sirq;
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

	//vuart initial
	writel(0x0, vuart->regs + AST_VUART_CTRLA);

	if (of_property_read_u32(np, "serial_irq", &sirq) == 0) {
		writel((readl(vuart->regs + AST_VUART_CTRLB) & ~VUART_SIRQ_MASK) | VUART_SET_SIRQ(sirq) | 0x3, vuart->regs + AST_VUART_CTRLB);
		printk("serial irq %d, ", sirq);
	} else {
		writel((readl(vuart->regs + AST_VUART_CTRLB) & ~VUART_SIRQ_MASK) | VUART_SET_SIRQ(4) | 0x3, vuart->regs + AST_VUART_CTRLB);
		printk("default serial irq 4, ");
	}
	
	if (of_property_read_u32(np, "port_address", &port_address) == 0) {
		writel((port_address >> 8) & 0xff, vuart->regs + AST_VUART_ADDRH);
		writel((port_address >> 0) & 0xff, vuart->regs + AST_VUART_ADDRL);
		printk("port address %x\n", port_address);
	} else {
		writel((0x3f8 >> 8) & 0xff, vuart->regs + AST_VUART_ADDRH);
		writel((0x3f8 >> 0) & 0xff, vuart->regs + AST_VUART_ADDRL);
		printk("default port address 0x3f8\n");
	}
	
	writel(0x0, vuart->regs + AST_VUART_CTRLF);
	writel(VUART_ENABLE | VUART_SIRQ_POLARITY | VUART_DISABLE_H_TX_DISCARD, vuart->regs + AST_VUART_CTRLA);	

	memset(&port, 0, sizeof(port));
	port.port.private_data = vuart;
	port.port.membase = vuart->regs;
	port.port.mapbase = resource.start;
	port.port.mapsize = resource_size(&resource);

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
	port.port.type = PORT_16550A;
	port.port.uartclk = clk;
	port.port.flags = UPF_SHARE_IRQ | UPF_BOOT_AUTOCONF
		| UPF_FIXED_PORT | UPF_FIXED_TYPE;
//	port.port.flags = UPF_SKIP_TEST | UPF_SHARE_IRQ | UPF_BOOT_AUTOCONF;

	if (of_find_property(np, "no-loopback-test", NULL))
		port.port.flags |= UPF_SKIP_TEST;

	port.port.dev = &pdev->dev;

	if (port.port.fifosize)
		port.capabilities = UART_CAP_FIFO;

	rc = serial8250_register_8250_port(&port);
	if (rc < 0)
		goto err_clk_disable;

	vuart->line = rc;
	platform_set_drvdata(pdev, vuart);

	/* extra sysfs control */
	rc = device_create_file(&pdev->dev, &dev_attr_port_address);
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

static const struct of_device_id ast_vuart_table[] = {
	{ .compatible = "aspeed,ast-vuart" },
	{ },
};

static struct platform_driver ast_vuart_driver = {
	.driver = {
		.name	= KBUILD_MODNAME,
		.of_match_table = ast_vuart_table,
	},
	.probe = ast_vuart_probe,
};

module_platform_driver(ast_vuart_driver);
MODULE_DESCRIPTION("VUART driver for AST SOC");
MODULE_AUTHOR("Ryan Chen <ryan_chen@aspeedtech.com>");
MODULE_LICENSE("GPL v2");
