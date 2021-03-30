// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright 2021 Joel Stanley, IBM Corp.
 */

#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/iopoll.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>

#define DEVICE_NAME	"aspeed-otp"

#define OTP_PASSWD		0x349fe38a
#define OTP_TRIGGER_PROGRAM     0x23b1e364
#define OTP_TRIGGER_READ        0x23b1e361
#define OTP_TRIGGER_WRITE_REG   0x23b1e362

#define OTP_PROTECT_KEY		0x0
#define OTP_COMMAND		0x4
#define OTP_TIMING		0x8
#define OTP_ADDR		0x10
#define OTP_STATUS		0x14
#define  OTP_STATUS_IDLE 	0x6
#define OTP_COMPARE_1		0x20
#define OTP_COMPARE_2		0x24
#define OTP_COMPARE_3		0x28
#define OTP_COMPARE_4		0x2c

#define NUM_OTP_CONF    	16
#define NUM_PROG_TRIES  	16

enum otp_region {
    otp_region_strap,
    otp_region_conf,
};

struct otp_params {
	uint32_t timings[3];
	uint32_t soak_parameters[3][3];
};

const struct otp_params rev_a2 = {
	.timings[0] = 0x04190760,
	.timings[1] = 0x04191388,
	.timings[2] = 0x04193a98,

	.soak_parameters[0][0] = 0x0210,
	.soak_parameters[0][1] = 0x2000,
	.soak_parameters[0][2] = 0x0,

	.soak_parameters[1][0] = 0x1200,
	.soak_parameters[1][1] = 0x107f,
	.soak_parameters[1][2] = 0x1024,

	.soak_parameters[2][0] = 0x1220,
	.soak_parameters[2][1] = 0x2074,
	.soak_parameters[2][2] = 0x08a4,
};

const struct otp_params rev_a1 = {
	.timings[0] = 0x04190760,
	.timings[1] = 0x04190760,
	.timings[2] = 0x041930d4,

	.soak_parameters[0][0] = 0x0,
	.soak_parameters[0][1] = 0x0,
	.soak_parameters[0][2] = 0x0,

	.soak_parameters[1][0] = 0x4021,
	.soak_parameters[1][1] = 0x302f,
	.soak_parameters[1][2] = 0x4020,

	.soak_parameters[2][0] = 0x4021,
	.soak_parameters[2][1] = 0x1027,
	.soak_parameters[2][2] = 0x4820,
};

struct aspeed_otp {
	struct miscdevice miscdev;
	struct device *dev;
	void *base;

	const struct otp_params *params;
};

static int otp_wait_complete(struct aspeed_otp *otp)
{
    uint32_t reg;

    return readl_relaxed_poll_timeout(otp->base + OTP_STATUS, reg,
		    (reg & OTP_STATUS_IDLE) == OTP_STATUS_IDLE,
		    10,
		    10000);
}

static int otp_program(struct aspeed_otp *otp, uint32_t addr, uint32_t val)
{
	writel(addr, otp->base + OTP_ADDR);
	writel(val, otp->base + OTP_COMPARE_1);
	writel(OTP_TRIGGER_PROGRAM, otp->base + OTP_COMMAND);

	return otp_wait_complete(otp);
}

static int otp_read_reg(struct aspeed_otp *otp, uint32_t addr, uint32_t *val)
{
	int rc;

	writel(addr, otp->base + OTP_ADDR);
	writel(OTP_TRIGGER_READ, otp->base + OTP_COMMAND);

	rc = otp_wait_complete(otp);
	if (rc)
		return rc;

	*val = readl(otp->base + OTP_COMPARE_1);

	return 0;
}

static int otp_read_config(struct aspeed_otp *otp, int offset, uint32_t *val)
{
	uint32_t config_offset = 0x800;

	config_offset |= (offset / 8) * 0x200;
	config_offset |= (offset % 8) * 2;

	return otp_read_reg(otp, config_offset, val);
}

static int otp_write_reg(struct aspeed_otp *otp, uint32_t addr, uint32_t val)
{
	writel(addr, otp->base + OTP_ADDR);
	writel(val, otp->base + OTP_COMPARE_1);
	writel(OTP_TRIGGER_WRITE_REG, otp->base + OTP_COMMAND);

	return otp_wait_complete(otp);
}

static int otp_set_soak(struct aspeed_otp *otp, unsigned int soak)
{
    int rc;

    if (soak > 2)
        return -EINVAL;

    rc = otp_write_reg(otp, 0x3000, otp->params->soak_parameters[soak][0]);
    if (rc)
        return rc;

    rc = otp_write_reg(otp, 0x5000, otp->params->soak_parameters[soak][1]);
    if (rc)
        return rc;

    rc = otp_write_reg(otp, 0x1000, otp->params->soak_parameters[soak][2]);
    if (rc)
        return rc;

    writel(otp->params->timings[soak], otp->base + OTP_TIMING);

    return 0;
}

static int otp_write(struct aspeed_otp *otp, uint32_t address, uint32_t bitmask)
{
    int rc;
    int tries = 0;
    uint32_t prog;
    uint32_t readback;

    rc = otp_set_soak(otp, 1);
    if (rc)
        return rc;

    prog = ~bitmask;
    rc = otp_program(otp, address, prog);
    if (rc)
        goto undo_soak;

    do {
        rc = otp_read_reg(otp, address, &readback);
        if (rc)
            goto undo_soak;

        if (readback & bitmask)
            break;

        rc = otp_set_soak(otp, (tries % 2) ? 1 : 2);
        if (rc)
            goto undo_soak;

        rc = otp_program(otp, address, prog);
        if (rc)
            goto undo_soak;
    } while (tries++ < NUM_PROG_TRIES);

    if (tries == NUM_PROG_TRIES) {
        dev_err(otp->dev, "Failed to program OTP\n");
        rc = -EREMOTEIO;
    } else
        dev_dbg(otp->dev, "Success!\n");

undo_soak:
    otp_set_soak(otp, 0);

    return rc;
}

int otp_read_conf(struct aspeed_otp *otp)
{
    uint32_t conf[NUM_OTP_CONF];
    int i;
    int rc;

    writel(OTP_PASSWD, otp->base + OTP_PROTECT_KEY);

    for (i = 0; i < NUM_OTP_CONF; ++i) {
	    rc = otp_read_config(otp, i, &conf[i]);
	    if (rc)
		    goto done;
    }

    dev_dbg(otp->dev, "OTP configuration:\n");
    for (i = 0; i < NUM_OTP_CONF; ++i)
	    dev_dbg(otp->dev, "%02u: %08x\n", i, conf[i]);

done:
    writel(0, otp->base + OTP_PROTECT_KEY);
    return rc;
}

int otp_read_strap(struct aspeed_otp *otp)
{
    uint32_t res[2] = { 0, 0};
    uint32_t strap[6][2];
    uint32_t protect[2];
    uint32_t scu_protect[2];
    int rc;
    int i;

    writel(OTP_PASSWD, otp->base + OTP_PROTECT_KEY);

    rc = otp_read_config(otp, 28, &scu_protect[0]);
    if (rc)
	    goto done;

    rc = otp_read_config(otp, 29, &scu_protect[1]);
    if (rc)
	    goto done;

    rc = otp_read_config(otp, 30, &protect[0]);
    if (rc)
	    goto done;

    rc = otp_read_config(otp, 31, &protect[1]);
    if (rc)
	    goto done;

    for (i = 0; i < 6; ++i) {
	    int o = 16 + (i * 2);

	    rc = otp_read_config(otp, o, &strap[i][0]);
	    if (rc)
		    goto done;

	    rc = otp_read_config(otp, o + 1, &strap[i][1]);
	    if (rc)
		    goto done;

	    res[0] ^= strap[i][0];
	    res[1] ^= strap[i][1];
    }

    dev_dbg(otp->dev, "OTP straps:\n");
    dev_dbg(otp->dev, "Protect SCU: %08x %08x\n", scu_protect[0], scu_protect[1]);
    dev_dbg(otp->dev, "Protect:     %08x %08x\n", protect[0], protect[1]);

    for (i = 0; i < 6; ++i)
	    dev_dbg(otp->dev, "Option %u:    %08x %08x\n", i, strap[i][0], strap[i][1]);

    dev_dbg(otp->dev, "Result:      %08x %08x\n", res[0], res[1]);
done:
    writel(0, otp->base + OTP_PROTECT_KEY);
    return rc;
}

int otp_write_conf(struct aspeed_otp *otp, unsigned int word, unsigned int bit)
{
    int rc;
    uint32_t conf;
    uint32_t address;
    uint32_t bitmask;

    if (word >= NUM_OTP_CONF || bit >= 32)
        return -EINVAL;

    bitmask = 1 << bit;

    writel(OTP_PASSWD, otp->base + OTP_PROTECT_KEY);

    rc = otp_read_config(otp, word, &conf);
    if (rc)
        goto done;

    if (conf & bitmask) {
        dev_err(otp->dev, "Configuration bit already set\n");
        rc = -EALREADY;
        goto done;
    }

    address = 0x800;
    address |= (word / 8) * 0x200;
    address |= (word % 8) * 2;

    dev_dbg(otp->dev, "Writing configuration at OTP %04x with %08x\n", address, bitmask);
    rc = otp_write(otp, address, bitmask);

done:
    writel(0, otp->base + OTP_PROTECT_KEY);
    return rc;
}

int otp_write_strap(struct aspeed_otp *otp, unsigned int bit, unsigned int val)
{
    int i;
    int rc;
    int f = -1;
    uint32_t address;
    uint32_t bitmask;
    uint32_t protect;
    uint32_t res = 0;
    uint32_t word = 0;
    uint32_t strap[6];

    if (bit >= 64 || val > 1)
        return -EINVAL;

    writel(OTP_PASSWD, otp->base + OTP_PROTECT_KEY);

    if (bit > 31) {
        word = 1;
        bit -= 32;
    }

    bitmask = 1 << bit;

    rc = otp_read_config(otp, 30 + word, &protect);
    if (rc)
        goto done;

    if (protect & bitmask) {
        dev_err(otp->dev, "Cannot write strap; bit is protected\n");
        rc = -EACCES;
        goto done;
    }

    for (i = 0; i < 6; ++i) {
        uint32_t o = 16 + (i * 2);

        rc = otp_read_config(otp, o + word, &strap[i]);
        if (rc)
            goto done;

        res ^= strap[i];
        if (f < 0 && !(strap[i] & bitmask))
            f = i;
    }

    if (f < 0) {
        dev_err(otp->dev, "Strap cannot be configured further\n");
        rc = -EPERM;
        goto done;
    }

    if (((res & bitmask) && val) || (!val && !(res & bitmask))) {
        dev_err(otp->dev, "Strap already in desired configuration\n");
        rc = -EALREADY;
        goto done;
    }

    i = (16 + (f * 2)) + word;
    address = 0x800;
    address |= (i / 8) * 0x200;
    address |= (i % 8) * 2;

    dev_dbg(otp->dev, "Writing strap at OTP %04x with %08x\n", address, bitmask);
    rc = otp_write(otp, address, bitmask);

done:
    writel(otp->base + OTP_PROTECT_KEY, 0);
    return rc;
}

static const struct file_operations aspeed_otp_fops = {
	.owner		= THIS_MODULE,
};

static int aspeed_otp_probe(struct platform_device *pdev)
{
        struct device_node *np = pdev->dev.of_node;
	struct device *dev = &pdev->dev;
	struct aspeed_otp *otp;
	struct regmap *scu;
	uint32_t rev;
	int rc;

	otp = devm_kzalloc(dev, sizeof(*otp), GFP_KERNEL);
	if (!otp)
		return -ENOMEM;

	otp->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(otp->base))
		return PTR_ERR(otp->base);

	platform_set_drvdata(pdev, otp);

        scu = syscon_regmap_lookup_by_phandle(np, "syscon");
	if (IS_ERR(scu)) {
		dev_err(dev, "failed to find SCU regmap\n");
		return PTR_ERR(scu);
	}
	rc = regmap_read(scu, 0x14, &rev);
	if (rc)
		return rc;

	rev >>= 24;
	switch (rev >> 24) {
	case 0:
	case 1:
		dev_dbg(dev, "Detected AST2600 A%d\n", rev);
		otp->params = &rev_a1;
		break;
	case 2:
		dev_dbg(dev, "Detected AST2600 A2\n");
		otp->params = &rev_a2;
		break;
	default:
		dev_dbg(dev, "Unknown AST2600 revision: %d\n", rev);
		return -EINVAL;
	}

	otp->miscdev.minor = MISC_DYNAMIC_MINOR;
	otp->miscdev.name = DEVICE_NAME;
	otp->miscdev.fops = &aspeed_otp_fops;
	otp->miscdev.parent = dev;
	rc = misc_register(&otp->miscdev);
	if (rc) {
		dev_err(dev, "Unable to register device\n");
		return rc;
	}

	return 0;
}

static int aspeed_otp_remove(struct platform_device *pdev)
{

	struct aspeed_otp *otp = dev_get_drvdata(&pdev->dev);

	misc_deregister(&otp->miscdev);

	return 0;
}

static const struct of_device_id aspeed_otp_match[] = {
	{ .compatible = "aspeed,ast2600-otp" },
	{ },
};

static struct platform_driver aspeed_otp_driver = {
	.driver = {
		.name		= DEVICE_NAME,
		.of_match_table = aspeed_otp_match,
	},
	.probe = aspeed_otp_probe,
	.remove = aspeed_otp_remove,
};

module_platform_driver(aspeed_otp_driver);

MODULE_DEVICE_TABLE(of, aspeed_otp_match);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Joel Stanley <joel@jms.id.au>");
MODULE_DESCRIPTION("ASPEED OTP configuration");
