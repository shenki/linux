/*
 * Copyright 2017 IBM Corporation
 *
 * Joel Stanley <joel@jms.id.au>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/fsi.h>

#include "i2c-fsi.h"

#define I2C_RESET_DELAY_MS      5 /* 5 msecs */
#define I2C_FIFO_HI_LVL         4
#define I2C_FIFO_LO_LVL         4

#define I2C_DEFAULT_CLK_HZ	125000000

/* PPC bit number conversion */
#define PPC_BIT(bit)            (0x8000000000000000ULL >> (bit))
#define PPC_BIT32(bit)          (0x80000000UL >> (bit))
#define PPC_BIT16(bit)          (0x8000UL >> (bit))
#define PPC_BIT8(bit)           (0x80UL >> (bit))
#define PPC_BITMASK(bs,be)      ((PPC_BIT(bs) - PPC_BIT(be)) | PPC_BIT(bs))
#define PPC_BITMASK32(bs,be)    ((PPC_BIT32(bs) - PPC_BIT32(be))|PPC_BIT32(bs)) 
#define PPC_BITMASK16(bs,be)    ((PPC_BIT16(bs) - PPC_BIT16(be))|PPC_BIT16(bs)) 
#define PPC_BITMASK8(bs,be)     ((PPC_BIT8(bs) - PPC_BIT8(be))|PPC_BIT8(bs))
#define PPC_BITLSHIFT(be)       (63 - (be))
#define PPC_BITLSHIFT32(be)     (31 - (be))

/*
 * PPC bitmask field manipulation
 */

/* Find left shift from first set bit in mask */
#define MASK_TO_LSH(m)          (__builtin_ffsll(m) - 1ULL)

/* Extract field fname from val */
#define GETFIELD(m, v)          (((v) & (m)) >> MASK_TO_LSH(m))

/* Set field fname of oval to fval
 * NOTE: oval isn't modified, the combined result is returned
 */
#define SETFIELD(m, v, val)                             \
        (((v) & ~(m)) | ((((typeof(v))(val)) << MASK_TO_LSH(m)) & (m)))


#define FSI_ENGID_I2CM		0x1

/* SCOM engine register set */
#define SCOM_DATA0_REG		0x00
#define SCOM_DATA1_REG		0x04
#define SCOM_CMD_REG		0x08
#define SCOM_RESET_REG		0x1C

#define SCOM_RESET_CMD		0x80000000
#define SCOM_WRITE_CMD		0x80000000

enum request_state {
	state_idle,
	state_occache_dis,
	state_offset,
	state_data,
	state_error,
	state_recovery,
};

struct fsi_i2c_bus {
	struct i2c_adapter	adapter;
	struct fsi_device	*fsi;
	u8			*rx_buf;
	u8			*tx_buf;

	u32			fifo_size;
        enum request_state	state;
};

#define fsii2c_to_fsi_dev(x) container_of((x), struct fsi_i2c_bus, fsi)

static int fsi_i2c_prog_watermark(struct fsi_i2c_bus *i2c)
{
        uint64_t watermark;
	struct device *dev = &i2c->adapter.dev;
        int rc;

	rc = fsi_scom_read(i2c->fsi, &watermark, I2C_WATERMARK_REG);
        if (rc) {
                dev_err(dev, "Failed to read the WATERMARK_REG\n");
                return rc;
        }

        /* Set the high/low watermark */
        watermark = SETFIELD(I2C_WATERMARK_HIGH, watermark, I2C_FIFO_HI_LVL);
        watermark = SETFIELD(I2C_WATERMARK_LOW, watermark, I2C_FIFO_LO_LVL);
	rc = fsi_scom_write(i2c->fsi, watermark, I2C_WATERMARK_REG);
        if (rc)
                dev_err(dev, "Failed to set high/low watermark level\n"); 

        return rc;
}

static int fsi_i2c_xfer(struct i2c_adapter *adap,
			struct i2c_msg *msgs, int num)
{
	struct fsi_i2c_bus *i2c = adap->algo_data;

	return 0;
}

static uint32_t fsi_i2c_get_bit_rate_divisor(uint32_t lb_freq,
					     uint32_t bus_speed)
{
	return (((lb_freq / bus_speed) - 1) / 4);
}

static u32 ast_i2c_functionality(struct i2c_adapter *adap)
{
	/* TODO: Fill out the appropriate functions */
	return I2C_FUNC_I2C;
}

static const struct i2c_algorithm fsi_i2c_algorithm = {
	.master_xfer = fsi_i2c_xfer,
	.functionality = fsi_i2c_functionality,
};

static int fsi_i2c_probe(struct device *dev)
{
	struct fsi_i2c_bus *i2c;
	int bus_clk_rate;
	u64 ex_stat;
	int rc;

	i2c = devm_kzalloc(dev, sizeof(*i2c), GFP_KERNEL);
	if (!i2c)
		return -ENOMEM;

	i2c->fsi = to_fsi_dev(dev);
	i2c->state = state_idle;

	i2c->adapter.owner = THIS_MODULE;
	i2c->adapter.dev.parent = dev;
	i2c->adapter.algo = &fsi_i2c_algorithm;
	i2c->adapter.algo_data = i2c;
	snprintf(i2c->adapter.name, sizeof(bus->adapter.name),
			"fsi-i2c.%d", bus_num);
	i2c->dev = dev;

	for_each_compatible_node(np, "ibm,cfam-i2cm") {
		/* Find node that matches the FSI address */
	}

	dev->clk = devm_clk_get(dev->dev, NULL);
	if (IS_ERR(dev->clk)) {
		dev_err(dev->dev, "no clock defined\n");
		return -ENODEV;
	}
	clk_prepare_enable(dev->clk);

	ret = of_property_read_u32(of_node,
			"clock-frequency", &i2c->bus_clk);
	if (ret < 0)
		bus->bus_clk = 400000;

	rc = fsi_scom_read(i2c->fsi, &ex_stat, I2C_EXTD_STAT_REG);
	if (rc) {
		dev_err(dev, "Failed to read EXTD_STAT_REG\n");
		return rc;
	}
	i2c->fifo_size = GETFIELD(I2C_EXTD_STAT_FIFO_SIZE, ex_stat);

	rc = fsi_i2c_prog_watermark(i2c);
	if (rc)
		return rc;

	rc = i2c_add_adapter(&i2c->adapter);
	if (ret < 0)
		return -ENXIO;

	return 0;
}

static const struct fsi_device_id fsi_i2c_ids[] = {
	{ FSI_ENGID_I2CM, FSI_VERSION_ANY },
	{ }
};

static struct fsi_driver fsi_i2c_driver = {
	.id_table = fsi_i2c_ids,
	.drv = {
		.name = "power8_i2cm",
		.bus = &fsi_bus_type,
		.probe = fsi_i2c_probe,
	},
};

static int fsi_i2c_init(void)
{
	return fsi_driver_register(&fsi_i2c_driver);
}

static void fsi_i2c_exit(void)
{
	fsi_driver_unregister(&fsi_i2c_driver);
}

module_init(fsi_i2c_init);
module_exit(fsi_i2c_exit);


MODULE_DESCRIPTION("FSI attached I2C master");
MODULE_LICENSE("GPL");
