// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for Microwatt SPI Flash Controller
 *
 * Copyright (c) 2021, IBM Corporation.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/spi/spi-mem.h>

#define DRIVER_NAME "microwatt-spi"

#define MSPI_DATA		0x00 /* Byte access: single wire transfer */
#define MSPI_DATA_DUAL		0x01 /* Byte access: dual wire transfer */
#define MSPI_DATA_QUAD		0x02 /* Byte access: quad wire transfer */

#define MSPI_CTRL		0x04 /* Reset and manual mode control */
#define	  MSPI_CTRL_RESET		0x01  /* reset all registers */
#define	  MSPI_CTRL_MANUAL_CS		0x02  /* assert CS, enable manual mode */
#define	  MSPI_CTRL_CKDIV_SHIFT		8     /* clock div */
#define	  MSPI_CTRL_CKDIV_MASK		(0xff << MSPI_CTRL_CKDIV_SHIFT)

#define MSPI_CFG		0x08 /* Automatic map configuration */
#define	  MSPI_CFG_CMD_SHIFT	     0	   /* Command to use for reads */
#define	  MSPI_CFG_CMD_MASK	     (0xff << MSPI_CFG_CMD_SHIFT)
#define	  MSPI_CFG_DUMMIES_SHIFT     8	   /* # dummy cycles */
#define	  MSPI_CFG_DUMMIES_MASK	     (0x7  << MSPI_CFG_DUMMIES_SHIFT)
#define	  MSPI_CFG_MODE_SHIFT	     11	   /* SPI wire mode */
#define	  MSPI_CFG_MODE_MASK	     (0x3  << MSPI_CFG_MODE_SHIFT)
#define	    MSPI_CFG_MODE_SINGLE	  (0 << 11)
#define	    MSPI_CFG_MODE_DUAL		  (2 << 11)
#define	    MSPI_CFG_MODE_QUAD		  (3 << 11)
#define	  MSPI_CFG_ADDR4	     BIT(13) /* 3 or 4 addr bytes */
#define	  MSPI_CFG_CKDIV_SHIFT	     16	   /* clock div */
#define	  MSPI_CFG_CKDIV_MASK	     (0xff << MSPI_CFG_CKDIV_SHIFT)
#define	  MSPI_CFG_CSTOUT_SHIFT	     24	   /* CS timeout */
#define	  MSPI_CFG_CSTOUT_MASK	     (0x3f << MSPI_CFG_CSTOUT_SHIFT)

struct mw_spi {
	void __iomem		*regs;
	void __iomem		*mem;
	struct clk		*pclk;
	struct platform_device	*pdev;
	resource_size_t		mmap_size;
};

struct mw_spi_mode {
	u8 cmd_buswidth;
	u8 addr_buswidth;
	u8 data_buswidth;
	u32 config;
};

static const struct mw_spi_mode mw_spi_modes[] = {
	{ 1, 1, 1, MSPI_CFG_MODE_SINGLE },
	{ 1, 1, 2, MSPI_CFG_MODE_DUAL   },
	{ 1, 1, 4, MSPI_CFG_MODE_QUAD   },
};

static u32 mspi_read(struct mw_spi *mspi, u32 offset)
{
	u32 value = readl_relaxed(mspi->regs + offset);

	dev_vdbg(&mspi->pdev->dev, "read @0x%02x 0x%08x\n", offset, value);
	return value;
}

static void mspi_write(struct mw_spi *mspi, u32 offset, u32 value)
{
	dev_vdbg(&mspi->pdev->dev, "write @0x%02x 0x%08x\n", offset, value);
	writel_relaxed(value, mspi->regs + offset);
}

static inline bool mw_spi_is_compatible(const struct spi_mem_op *op,
					const struct mw_spi_mode *mode)
{
	if (op->cmd.buswidth != mode->cmd_buswidth)
		return false;

	if (op->addr.nbytes && op->addr.buswidth != mode->addr_buswidth)
		return false;

	if (op->data.nbytes && op->data.buswidth != mode->data_buswidth)
		return false;

	return true;
}

static int mw_spi_find_mode(const struct spi_mem_op *op)
{
	u32 i;

	for (i = 0; i < ARRAY_SIZE(mw_spi_modes); i++)
		if (mw_spi_is_compatible(op, &mw_spi_modes[i]))
			return i;

	return -EOPNOTSUPP;
}

static bool mw_spi_supports_op(struct spi_mem *mem,
			       const struct spi_mem_op *op)
{
	if (mw_spi_find_mode(op) < 0)
		return false;

	/* special case not supported by hardware */
	if (op->addr.nbytes == 2 && op->cmd.buswidth != op->addr.buswidth &&
	    op->dummy.nbytes == 0)
		return false;

	/* DTR ops not supported. */
	if (op->cmd.dtr || op->addr.dtr || op->dummy.dtr || op->data.dtr)
		return false;
	if (op->cmd.nbytes != 1)
		return false;

	return true;
}

static int mw_spi_set_cfg(struct mw_spi *mspi, const struct spi_mem_op *op)
{
	u32 cfg;
	u32 dummy_cycles = 0;
	int mode;

	/* TODO: preserve default clock settings (HW or initial FW) */
	cfg = mspi_read(mspi, MSPI_CFG);
	cfg &= MSPI_CFG_CKDIV_MASK;

	mode = mw_spi_find_mode(op);
	if (mode < 0)
		return mode;
	cfg |= mw_spi_modes[mode].config;

	/* 32 cycles CS timeout for quad */
	if (mw_spi_modes[mode].config == MSPI_CFG_MODE_QUAD)
		cfg |= 0x20 << MSPI_CFG_CSTOUT_SHIFT;

	cfg |= op->cmd.opcode << MSPI_CFG_CMD_SHIFT;

	if (op->dummy.buswidth && op->dummy.nbytes)
		dummy_cycles = op->dummy.nbytes * 8 / op->dummy.buswidth;

	/* TODO: clarify dummies */
	if (dummy_cycles)
		cfg |= 0x07 << MSPI_CFG_DUMMIES_SHIFT;

	if (op->addr.buswidth && op->addr.nbytes == 4)
		cfg |= MSPI_CFG_ADDR4;

	mspi_write(mspi, MSPI_CFG, cfg);
	return 0;
}

static int mw_spi_mem_read(struct mw_spi *mspi, const struct spi_mem_op *op)
{
	int err;

	/* TODO: check that the accesses wrap around when reaching the
	 * flash size.
	 */
	if (op->addr.val + op->data.nbytes > mspi->mmap_size)
		return -EOPNOTSUPP;

	err = mw_spi_set_cfg(mspi, op);
	if (err)
		return err;

	memcpy_fromio(op->data.buf.in, mspi->mem + op->addr.val, op->data.nbytes);
	return 0;
}

static int mw_spi_command_read(struct mw_spi *mspi, const struct spi_mem_op *op)
{
	mspi_write(mspi, MSPI_CTRL, MSPI_CTRL_MANUAL_CS);

	writeb(op->cmd.opcode, mspi->regs + MSPI_DATA);
	ioread8_rep(mspi->regs + MSPI_DATA, op->data.buf.in, op->data.nbytes);

	mspi_write(mspi, MSPI_CTRL, 0);
	return 0;
}

static int mw_spi_command_write(struct mw_spi *mspi, const struct spi_mem_op *op)
{
	mspi_write(mspi, MSPI_CTRL, MSPI_CTRL_MANUAL_CS);

	writeb(op->cmd.opcode, mspi->regs + MSPI_DATA);
	iowrite8_rep(mspi->regs + MSPI_DATA, op->data.buf.out, op->data.nbytes);

	mspi_write(mspi, MSPI_CTRL, 0);
	return 0;
}

static int mw_spi_write(struct mw_spi *mspi, const struct spi_mem_op *op)
{
	int i = op->addr.nbytes;

	mspi_write(mspi, MSPI_CTRL, MSPI_CTRL_MANUAL_CS);

	/* push command + address */
	writeb(op->cmd.opcode, mspi->regs + MSPI_DATA);
	while (i--)
		writeb((op->addr.val >> (i * 8)) & 0xff, mspi->regs + MSPI_DATA);
	/* then data */
	iowrite8_rep(mspi->regs + MSPI_DATA, op->data.buf.out, op->data.nbytes);

	mspi_write(mspi, MSPI_CTRL, 0);
	return 0;
}

static int __mw_spi_exec_op(struct spi_mem *mem, const struct spi_mem_op *op)
{
	struct mw_spi *mspi = spi_controller_get_devdata(mem->spi->master);

	if (op->data.dir == SPI_MEM_DATA_IN && op->data.buf.in) {
		/* Use command mode for Read SFDP */
		if (!op->addr.nbytes || op->cmd.opcode == 0x5a)
			return mw_spi_command_read(mspi, op);

		return mw_spi_mem_read(mspi, op);
	} else { /* TODO: check write conditions */
		if (!op->addr.nbytes)
			return mw_spi_command_write(mspi, op);

		return mw_spi_write(mspi, op);
	}

	return -EOPNOTSUPP;
}

static int mw_spi_exec_op(struct spi_mem *mem, const struct spi_mem_op *op)
{
	int ret;

	ret = __mw_spi_exec_op(mem, op);
	if (ret)
		dev_err(&mem->spi->dev, "operation failed with %d\n", ret);

	return ret;
}

static const char *mw_spi_get_name(struct spi_mem *spimem)
{
	return dev_name(spimem->spi->dev.parent);
}

static const struct spi_controller_mem_ops mw_spi_mem_ops = {
	.supports_op = mw_spi_supports_op,
	.exec_op = mw_spi_exec_op,
	.get_name = mw_spi_get_name
};

static int mw_spi_setup(struct spi_device *spi)
{
	struct spi_controller *ctrl = spi->master;
	struct mw_spi *mspi = spi_controller_get_devdata(ctrl);
//	unsigned long clk_rate;

	if (ctrl->busy)
		return -EBUSY;

	if (!spi->max_speed_hz)
		return -EINVAL;

	dev_info(&mspi->pdev->dev, "Using %d MHz SPI frequency\n",
		 spi->max_speed_hz / 1000000);

#ifdef MSPI_HAS_A_CLOCK
	clk_rate = clk_get_rate(mspi->pclk);
	if (!clk_rate)
		return -EINVAL;
#endif
	/* TODO: set clock rate */
	return 0;
}

static void mw_spi_reset(struct mw_spi *mspi)
{
	mspi_write(mspi, MSPI_CTRL, MSPI_CTRL_RESET);
}

static int mw_spi_probe(struct platform_device *pdev)
{
	struct spi_controller *ctrl;
	struct mw_spi *mspi;
	struct resource *res;
	int err = 0;

	ctrl = devm_spi_alloc_master(&pdev->dev, sizeof(*mspi));
	if (!ctrl)
		return -ENOMEM;

	ctrl->mode_bits = SPI_RX_DUAL | SPI_RX_QUAD | SPI_TX_DUAL | SPI_TX_QUAD;
	ctrl->setup = mw_spi_setup;
	ctrl->bus_num = -1;
	ctrl->mem_ops = &mw_spi_mem_ops;
	ctrl->num_chipselect = 1;
	ctrl->dev.of_node = pdev->dev.of_node;
	platform_set_drvdata(pdev, ctrl);

	mspi = spi_controller_get_devdata(ctrl);

	mspi->pdev = pdev;

	/* Map the registers */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	mspi->regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(mspi->regs)) {
		dev_err(&pdev->dev, "missing registers\n");
		return PTR_ERR(mspi->regs);
	}

	/* Map the flash window */
	res =  platform_get_resource(pdev, IORESOURCE_MEM, 1);
	mspi->mem = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(mspi->mem)) {
		dev_err(&pdev->dev, "missing flash window region\n");
		return PTR_ERR(mspi->mem);
	}

	mspi->mmap_size = resource_size(res);

	/* Get the peripheral clock */
	mspi->pclk = devm_clk_get(&pdev->dev, "pclk");
	if (IS_ERR(mspi->pclk))
		mspi->pclk = devm_clk_get(&pdev->dev, NULL);

	if (IS_ERR(mspi->pclk)) {
		dev_err(&pdev->dev, "missing peripheral clock\n");
		return PTR_ERR(mspi->pclk);
	}

	/* Enable the peripheral clock */
	err = clk_prepare_enable(mspi->pclk);
	if (err) {
		dev_err(&pdev->dev, "failed to enable the peripheral clock\n");
		return err;
	}

	mw_spi_reset(mspi);

	err = spi_register_controller(ctrl);
	if (err)
		goto disable_pclk;

	return 0;

disable_pclk:
	clk_disable_unprepare(mspi->pclk);

	return err;
}

static int mw_spi_remove(struct platform_device *pdev)
{
	struct spi_controller *ctrl = platform_get_drvdata(pdev);
	struct mw_spi *mspi = spi_controller_get_devdata(ctrl);

	spi_unregister_controller(ctrl);
	clk_disable_unprepare(mspi->pclk);
	return 0;
}

static const struct of_device_id mw_spi_of_match[] = {
	{ .compatible = "openpower,microwatt-spi", },
	{ },
};

MODULE_DEVICE_TABLE(of, mw_spi_dt_ids);

static struct platform_driver mw_spi_driver = {
	.probe		= mw_spi_probe,
	.remove		= mw_spi_remove,
	.driver = {
		.name	= DRIVER_NAME,
		.of_match_table	= mw_spi_of_match,
	},
};
module_platform_driver(mw_spi_driver);

MODULE_AUTHOR("Cédric Le Goater <clg@kaod.org>");
MODULE_DESCRIPTION("Microwatt SPI Flash Controller driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" DRIVER_NAME);
