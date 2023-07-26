// SPDX-License-Identifier: GPL-2.0+
/*
 * Hardware monitoring driver for Infineon TDA38640
 *
 * Copyright (c) 2023 9elements GmbH
 *
 */

#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/regulator/driver.h>
#include "pmbus.h"

static const struct regulator_desc __maybe_unused tda38640_reg_desc[] = {
	PMBUS_REGULATOR("vout", 0),
};

struct tda38640_data {
        struct pmbus_driver_info info;
        u32 en_pin_lvl;
};
#define to_tda38640_data(x)  container_of(x, struct tda38640_data, info)

/*
 * Map PB_ON_OFF_CONFIG_POLARITY_HIGH to PB_OPERATION_CONTROL_ON.
 */
static int tda38640_read_byte_data(struct i2c_client *client, int page, int reg)
{
	const struct pmbus_driver_info *info = pmbus_get_driver_info(client);
	struct tda38640_data *data = to_tda38640_data(info);
	int ret, on_off_config, enabled;

	if (reg != PMBUS_OPERATION)
		return -ENODATA;

	ret = pmbus_read_byte_data(client, page, reg);
	if (ret < 0)
		return ret;

	on_off_config = pmbus_read_byte_data(client, page,
					     PMBUS_ON_OFF_CONFIG);
	if (on_off_config < 0)
		return on_off_config;

	enabled = !!(on_off_config & PB_ON_OFF_CONFIG_POLARITY_HIGH);

	enabled ^= data->en_pin_lvl;
	if (enabled)
		ret &= ~PB_OPERATION_CONTROL_ON;
	else
		ret |= PB_OPERATION_CONTROL_ON;

	return ret;
}

/*
 * Map PB_OPERATION_CONTROL_ON to PB_ON_OFF_CONFIG_POLARITY_HIGH.
 */
static int tda38640_write_byte_data(struct i2c_client *client, int page,
				    int reg, u8 byte)
{
	const struct pmbus_driver_info *info = pmbus_get_driver_info(client);
	struct tda38640_data *data = to_tda38640_data(info);
	int enable, ret;

	if (reg != PMBUS_OPERATION)
		return -ENODATA;

	enable = !!(byte & PB_OPERATION_CONTROL_ON);

	byte &= ~PB_OPERATION_CONTROL_ON;
	ret = pmbus_write_byte_data(client, page, reg, byte);
	if (ret < 0)
		return ret;

	enable ^= data->en_pin_lvl;

	return pmbus_update_byte_data(client, page, PMBUS_ON_OFF_CONFIG,
				      PB_ON_OFF_CONFIG_POLARITY_HIGH,
				      enable ? 0 : PB_ON_OFF_CONFIG_POLARITY_HIGH);
}

static struct pmbus_driver_info tda38640_info = {
	.pages = 1,
	.format[PSC_VOLTAGE_IN] = linear,
	.format[PSC_VOLTAGE_OUT] = linear,
	.format[PSC_CURRENT_OUT] = linear,
	.format[PSC_CURRENT_IN] = linear,
	.format[PSC_POWER] = linear,
	.format[PSC_TEMPERATURE] = linear,
	.func[0] = PMBUS_HAVE_VIN | PMBUS_HAVE_STATUS_INPUT
	    | PMBUS_HAVE_TEMP | PMBUS_HAVE_STATUS_TEMP
	    | PMBUS_HAVE_IIN
	    | PMBUS_HAVE_VOUT | PMBUS_HAVE_STATUS_VOUT
	    | PMBUS_HAVE_IOUT | PMBUS_HAVE_STATUS_IOUT
	    | PMBUS_HAVE_POUT | PMBUS_HAVE_PIN,
#if IS_ENABLED(CONFIG_SENSORS_TDA38640_REGULATOR)
	.num_regulators = 1,
	.reg_desc = tda38640_reg_desc,
#endif
};

static int tda38640_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct device_node *np = dev_of_node(dev);
	struct tda38640_data *data;

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;
	memcpy(&data->info, &tda38640_info, sizeof(tda38640_info));

	if (!CONFIG_SENSORS_TDA38640_REGULATOR || !np ||
	    of_property_read_u32(np, "infineon,en-pin-fixed-level", &data->en_pin_lvl))
		return pmbus_do_probe(client, &data->info);

	/*
	 * Apply ON_OFF_CONFIG workaround as enabling the regulator using the
	 * OPERATION register doesn't work in SVID mode.
	 */
	data->info.read_byte_data = tda38640_read_byte_data;
	data->info.write_byte_data = tda38640_write_byte_data;
	/*
	 * One should configure PMBUS_ON_OFF_CONFIG here, but
	 * PB_ON_OFF_CONFIG_POWERUP_CONTROL, PB_ON_OFF_CONFIG_EN_PIN_REQ and
	 * PB_ON_OFF_CONFIG_EN_PIN_REQ are ignored by the device.
	 * Only PB_ON_OFF_CONFIG_POLARITY_HIGH has an effect.
	 */

	return pmbus_do_probe(client, &data->info);
}

static const struct i2c_device_id tda38640_id[] = {
	{"tda38640", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, tda38640_id);

static const struct of_device_id __maybe_unused tda38640_of_match[] = {
	{ .compatible = "infineon,tda38640"},
	{ },
};
MODULE_DEVICE_TABLE(of, tda38640_of_match);

/* This is the driver that will be inserted */
static struct i2c_driver tda38640_driver = {
	.driver = {
		.name = "tda38640",
		.of_match_table = of_match_ptr(tda38640_of_match),
	},
	.probe_new = tda38640_probe,
	.id_table = tda38640_id,
};

module_i2c_driver(tda38640_driver);

MODULE_AUTHOR("Patrick Rudolph <patrick.rudolph@9elements.com>");
MODULE_DESCRIPTION("PMBus driver for Infineon TDA38640");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS(PMBUS);
