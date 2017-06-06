/*
 * max31785.c - Part of lm_sensors, Linux kernel modules for hardware
 *	       monitoring.
 *
 * (C) 2016 Raptor Engineering, LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/err.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/slab.h>

/* MAX31785 device IDs */
#define MAX31785_MFR_ID				0x4d
#define MAX31785_MFR_MODEL			0x53

/* MAX31785 registers */
#define MAX31785_REG_PAGE			0x00
#define MAX31785_PAGE_FAN_CONFIG(ch)		(0x00 + (ch))
#define MAX31785_REG_FAN_CONFIG_1_2		0x3a
#define MAX31785_REG_FAN_COMMAND_1		0x3b
#define MAX31785_REG_STATUS_FANS_1_2		0x81
#define MAX31785_REG_FAN_SPEED_1		0x90
#define MAX31785_REG_MFR_ID			0x99
#define MAX31785_REG_MFR_MODEL			0x9a
#define MAX31785_REG_MFR_REVISION		0x9b
#define MAX31785_REG_MFR_FAN_CONFIG		0xf1
#define	 MAX31785_REG_MFR_FAN_CONFIG_DUAL_TACH	BIT(12)
#define MAX31785_REG_READ_FAN_PWM		0xf3

/* Fan Config register bits */
#define MAX31785_FAN_CFG_PWM_ENABLE		0x80
#define MAX31785_FAN_CFG_CONTROL_MODE_RPM	0x40
#define MAX31785_FAN_CFG_PULSE_MASK		0x30
#define MAX31785_FAN_CFG_PULSE_SHIFT		4
#define MAX31785_FAN_CFG_PULSE_OFFSET		1

/* Fan Status register bits */
#define MAX31785_FAN_STATUS_FAULT_MASK		0x80

/* Fan Command constants */
#define MAX31785_FAN_COMMAND_PWM_RATIO		40

#define NR_CHANNEL				6

/* Addresses to scan */
static const unsigned short normal_i2c[] = {
	0x52, 0x53, 0x54, 0x55,
	I2C_CLIENT_END
};

#define MAX31785_CAP_FAST_ROTOR BIT(0)

/*
 * Client data (each client gets its own)
 *
 * @lock:		Protects device access and access to cached values
 * @valid:		False until fields below it are valid
 * @last_updated:	Last update time in jiffies
 */
struct max31785 {
	struct i2c_client	*client;
	struct mutex		lock;
	bool			valid;
	unsigned long		last_updated;
	u32			capabilities;

	/* Registers */
	u8	fan_config[NR_CHANNEL];
	u16	fan_command[NR_CHANNEL];
	u16	mfr_fan_config[NR_CHANNEL];
	u8	fault_status[NR_CHANNEL];
	u16	pwm[NR_CHANNEL];
	u16	tach_rpm[NR_CHANNEL * 2];
};

static inline bool max31785_has_dual_rotor(struct max31785 *data)
{
	return !!(data->capabilities & MAX31785_CAP_FAST_ROTOR);
}

static int max31785_set_page(struct i2c_client *client,
				u8 page)
{
	return i2c_smbus_write_byte_data(client, MAX31785_REG_PAGE, page);
}

static int read_fan_data(struct i2c_client *client, u8 fan, u8 reg,
				  s32 (*read)(const struct i2c_client *, u8))
{
	int rv;

	rv = max31785_set_page(client, MAX31785_PAGE_FAN_CONFIG(fan));
	if (rv < 0)
		return rv;

	return read(client, reg);
}

static inline int max31785_read_fan_byte(struct i2c_client *client, u8 fan,
					 u8 reg)
{
	return read_fan_data(client, fan, reg, i2c_smbus_read_byte_data);
}

static inline int max31785_read_fan_word(struct i2c_client *client, u8 fan,
					 u8 reg)
{
	return read_fan_data(client, fan, reg, i2c_smbus_read_word_data);
}

static int max31785_write_fan_byte(struct i2c_client *client, u8 fan,
					 u8 reg, u8 data)
{
	int err;

	err = max31785_set_page(client, MAX31785_PAGE_FAN_CONFIG(fan));
	if (err < 0)
		return err;

	return i2c_smbus_write_byte_data(client, reg, data);
}

static int max31785_write_fan_word(struct i2c_client *client, u8 fan,
					 u8 reg, u16 data)
{
	int err;

	err = max31785_set_page(client, MAX31785_PAGE_FAN_CONFIG(fan));
	if (err < 0)
		return err;

	return i2c_smbus_write_word_data(client, reg, data);
}

/* Cut down version of i2c_smbus_xfer_emulated(), reading 4 bytes */
static s64 max31785_smbus_read_long_data(struct i2c_client *client, u8 command)
{
	unsigned char cmdbuf[1];
	unsigned char rspbuf[4];
	s64 rc;

	struct i2c_msg msg[2] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = sizeof(cmdbuf),
			.buf = cmdbuf,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = sizeof(rspbuf),
			.buf = rspbuf,
		},
	};

	cmdbuf[0] = command;

	rc = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
	if (rc < 0)
		return rc;

	rc = (rspbuf[0] << (0 * 8)) | (rspbuf[1] << (1 * 8)) |
	     (rspbuf[2] << (2 * 8)) | (rspbuf[3] << (3 * 8));

	return rc;
}

static int max31785_update_fan_speed(struct max31785 *data, u8 fan)
{
	s64 rc;

	rc = max31785_set_page(data->client, MAX31785_PAGE_FAN_CONFIG(fan));
	if (rc)
		return rc;

	if (max31785_has_dual_rotor(data)) {
		rc = max31785_smbus_read_long_data(data->client,
				MAX31785_REG_FAN_SPEED_1);
		if (rc < 0)
			return rc;

		data->tach_rpm[fan] = rc & 0xffff;
		data->tach_rpm[NR_CHANNEL + fan] = (rc >> 16) & 0xffff;

		return rc;
	}

	rc = i2c_smbus_read_word_data(data->client, MAX31785_REG_FAN_SPEED_1);
	if (rc < 0)
		return rc;

	data->tach_rpm[fan] = rc;

	return rc;
}

static inline bool is_automatic_control_mode(struct max31785 *data,
			int index)
{
	return data->fan_command[index] > 0x7fff;
}

static struct max31785 *max31785_update_device(struct device *dev)
{
	struct max31785 *data = dev_get_drvdata(dev);
	struct i2c_client *client = data->client;
	struct max31785 *ret = data;
	int rv;
	int i;

	mutex_lock(&data->lock);

	if (!time_after(jiffies, data->last_updated + HZ) && data->valid) {
		mutex_unlock(&data->lock);

		return ret;
	}

	for (i = 0; i < NR_CHANNEL; i++) {
		rv = max31785_read_fan_byte(client, i,
				MAX31785_REG_STATUS_FANS_1_2);
		if (rv < 0)
			goto abort;
		data->fault_status[i] = rv;

		rv = max31785_update_fan_speed(data, i);
		if (rv < 0)
			goto abort;

		if ((data->fan_config[i] & MAX31785_FAN_CFG_CONTROL_MODE_RPM)
				|| is_automatic_control_mode(data, i)) {
			rv = max31785_read_fan_word(client, i,
					MAX31785_REG_READ_FAN_PWM);
			if (rv < 0)
				goto abort;
			data->pwm[i] = rv;
		}

		if (!is_automatic_control_mode(data, i)) {
			/*
			 * Poke watchdog for manual fan control
			 *
			 * XXX (AJ): This isn't documented in the MAX31785
			 * datasheet, or anywhere else it seems.
			 */
			rv = max31785_write_fan_word(client,
					i, MAX31785_REG_FAN_COMMAND_1,
					data->fan_command[i]);
			if (rv < 0)
				goto abort;
		}
	}

	data->last_updated = jiffies;
	data->valid = true;

	mutex_unlock(&data->lock);

	return ret;

abort:
	data->valid = false;

	mutex_unlock(&data->lock);

	return ERR_PTR(rv);

}

static ssize_t max31785_fan_set_target(struct max31785 *data, int channel,
		long rpm)
{
	int rc;

	if (rpm > 0x7fff)
		return -EINVAL;

	mutex_lock(&data->lock);

	/* Write new RPM value */
	data->fan_command[channel] = rpm;
	rc = max31785_write_fan_word(data->client, channel,
				MAX31785_REG_FAN_COMMAND_1,
				data->fan_command[channel]);

	mutex_unlock(&data->lock);

	return rc;
}

static ssize_t max31785_fan_set_pulses(struct max31785 *data, int channel,
		long pulses)
{
	int rc;

	if (pulses > 4)
		return -EINVAL;

	mutex_lock(&data->lock);

	/* XXX (AJ): This sequence disables the fan and sets in PWM mode */
	data->fan_config[channel] &= MAX31785_FAN_CFG_PULSE_MASK;
	data->fan_config[channel] |= ((pulses - MAX31785_FAN_CFG_PULSE_OFFSET)
					<< MAX31785_FAN_CFG_PULSE_SHIFT);

	/* Write new pulse value */
	rc = max31785_write_fan_byte(data->client, channel,
				MAX31785_REG_FAN_CONFIG_1_2,
				data->fan_config[channel]);

	mutex_unlock(&data->lock);

	return rc;
}

static ssize_t max31785_pwm_set(struct max31785 *data, int channel, long pwm)
{
	int rc;

	if (pwm > 255)
		return -EINVAL;

	mutex_lock(&data->lock);

	/* Write new PWM value */
	data->fan_command[channel] = pwm * MAX31785_FAN_COMMAND_PWM_RATIO;
	rc = max31785_write_fan_word(data->client, channel,
				MAX31785_REG_FAN_COMMAND_1,
				data->fan_command[channel]);

	mutex_unlock(&data->lock);

	return rc;
}

static ssize_t max31785_pwm_enable(struct max31785 *data, int channel,
		long mode)
{
	struct i2c_client *client = data->client;
	int rc;

	mutex_lock(&data->lock);

	switch (mode) {
	case 0:
		data->fan_config[channel] =
			data->fan_config[channel]
			& ~MAX31785_FAN_CFG_PWM_ENABLE;
		break;
	case 1: /* fallthrough */
	case 2: /* fallthrough */
	case 3:
		data->fan_config[channel] =
			data->fan_config[channel]
			 | MAX31785_FAN_CFG_PWM_ENABLE;
		break;
	default:
		rc = -EINVAL;
		goto done;

	}

	switch (mode) {
	case 0:
		break;
	case 1:
		data->fan_config[channel] =
			data->fan_config[channel]
			& ~MAX31785_FAN_CFG_CONTROL_MODE_RPM;
		break;
	case 2:
		data->fan_config[channel] =
			data->fan_config[channel]
			| MAX31785_FAN_CFG_CONTROL_MODE_RPM;
		break;
	case 3:
		data->fan_command[channel] = 0xffff;
		break;
	default:
		rc = -EINVAL;
		goto done;
	}

	rc = max31785_write_fan_byte(client, channel,
				MAX31785_REG_FAN_CONFIG_1_2,
				data->fan_config[channel]);

	if (!rc)
		rc = max31785_write_fan_word(client, channel,
				MAX31785_REG_FAN_COMMAND_1,
				data->fan_command[channel]);

done:
	mutex_unlock(&data->lock);

	return rc;
}

static int max31785_init_fans(struct max31785 *data)
{
	struct i2c_client *client = data->client;
	int i, rv;

	for (i = 0; i < NR_CHANNEL; i++) {
		rv = max31785_read_fan_byte(client, i,
				MAX31785_REG_FAN_CONFIG_1_2);
		if (rv < 0)
			return rv;
		data->fan_config[i] = rv;

		rv = max31785_read_fan_word(client, i,
				MAX31785_REG_FAN_COMMAND_1);
		if (rv < 0)
			return rv;
		data->fan_command[i] = rv;

		rv = max31785_read_fan_word(client, i,
				MAX31785_REG_MFR_FAN_CONFIG);
		if (rv < 0)
			return rv;
		data->mfr_fan_config[i] = rv;

		if (max31785_has_dual_rotor(data)) {
			rv |= MAX31785_REG_MFR_FAN_CONFIG_DUAL_TACH;
			data->mfr_fan_config[i] = rv;

			rv = max31785_write_fan_word(client, i,
					MAX31785_REG_MFR_FAN_CONFIG,
					data->mfr_fan_config[i]);
			if (rv < 0)
				return rv;
		}

		if (!((data->fan_config[i]
			& MAX31785_FAN_CFG_CONTROL_MODE_RPM)
			|| is_automatic_control_mode(data, i))) {
			data->pwm[i] = 0;
		}
	}

	return rv;
}

/* Return 0 if detection is successful, -ENODEV otherwise */
static int max31785_detect(struct i2c_client *client,
			  struct i2c_board_info *info)
{
	struct i2c_adapter *adapter = client->adapter;
	int rv;

	if (!i2c_check_functionality(adapter,
			I2C_FUNC_SMBUS_BYTE_DATA | I2C_FUNC_SMBUS_WORD_DATA))
		return -ENODEV;

	/* Probe manufacturer / model registers */
	rv = i2c_smbus_read_byte_data(client, MAX31785_REG_MFR_ID);
	if (rv < 0)
		return -ENODEV;
	if (rv != MAX31785_MFR_ID)
		return -ENODEV;

	rv = i2c_smbus_read_byte_data(client, MAX31785_REG_MFR_MODEL);
	if (rv < 0)
		return -ENODEV;
	if (rv != MAX31785_MFR_MODEL)
		return -ENODEV;

	strlcpy(info->type, "max31785", I2C_NAME_SIZE);

	return 0;
}

static const u32 max31785_fan_config_0x3030[] = {
	HWMON_F_INPUT | HWMON_F_PULSES | HWMON_F_TARGET | HWMON_F_FAULT,
	HWMON_F_INPUT | HWMON_F_PULSES | HWMON_F_TARGET | HWMON_F_FAULT,
	HWMON_F_INPUT | HWMON_F_PULSES | HWMON_F_TARGET | HWMON_F_FAULT,
	HWMON_F_INPUT | HWMON_F_PULSES | HWMON_F_TARGET | HWMON_F_FAULT,
	HWMON_F_INPUT | HWMON_F_PULSES | HWMON_F_TARGET | HWMON_F_FAULT,
	HWMON_F_INPUT | HWMON_F_PULSES | HWMON_F_TARGET | HWMON_F_FAULT,
	0
};

static const struct hwmon_channel_info max31785_fan_0x3030 = {
	.type = hwmon_fan,
	.config = max31785_fan_config_0x3030,
};

static const u32 max31785_fan_config_0x3040[] = {
	HWMON_F_INPUT | HWMON_F_PULSES | HWMON_F_TARGET | HWMON_F_FAULT,
	HWMON_F_INPUT | HWMON_F_PULSES | HWMON_F_TARGET | HWMON_F_FAULT,
	HWMON_F_INPUT | HWMON_F_PULSES | HWMON_F_TARGET | HWMON_F_FAULT,
	HWMON_F_INPUT | HWMON_F_PULSES | HWMON_F_TARGET | HWMON_F_FAULT,
	HWMON_F_INPUT | HWMON_F_PULSES | HWMON_F_TARGET | HWMON_F_FAULT,
	HWMON_F_INPUT | HWMON_F_PULSES | HWMON_F_TARGET | HWMON_F_FAULT,
	HWMON_F_INPUT,
	HWMON_F_INPUT,
	HWMON_F_INPUT,
	HWMON_F_INPUT,
	HWMON_F_INPUT,
	HWMON_F_INPUT,
	0
};

static const struct hwmon_channel_info max31785_fan_0x3040 = {
	.type = hwmon_fan,
	.config = max31785_fan_config_0x3040,
};

static const u32 max31785_pwm_config[] = {
	HWMON_PWM_INPUT | HWMON_PWM_ENABLE,
	HWMON_PWM_INPUT | HWMON_PWM_ENABLE,
	HWMON_PWM_INPUT | HWMON_PWM_ENABLE,
	HWMON_PWM_INPUT | HWMON_PWM_ENABLE,
	HWMON_PWM_INPUT | HWMON_PWM_ENABLE,
	HWMON_PWM_INPUT | HWMON_PWM_ENABLE,
	0,
};

static const struct hwmon_channel_info max31785_pwm = {
	.type = hwmon_pwm,
	.config = max31785_pwm_config
};

static const struct hwmon_channel_info *max31785_info_0x3030[] = {
	&max31785_fan_0x3030,
	&max31785_pwm,
	NULL,
};

static const struct hwmon_channel_info *max31785_info_0x3040[] = {
	&max31785_fan_0x3040,
	&max31785_pwm,
	NULL,
};

static int max31785_read_fan(struct max31785 *data, u32 attr, int channel,
		long *val)
{
	int rc = 0;

	switch (attr) {
	case hwmon_fan_pulses:
	{
		long pulses;

		pulses = data->fan_config[channel];
		pulses &= MAX31785_FAN_CFG_PULSE_MASK;
		pulses >>= MAX31785_FAN_CFG_PULSE_SHIFT;
		pulses += MAX31785_FAN_CFG_PULSE_OFFSET;

		*val = pulses;
		break;
	}
	case hwmon_fan_target:
	{
		long target;

		mutex_lock(&data->lock);

		target = data->fan_command[channel];

		if (!(data->fan_config[channel] &
				MAX31785_FAN_CFG_CONTROL_MODE_RPM))
			target /= MAX31785_FAN_COMMAND_PWM_RATIO;

		*val = target;

		mutex_unlock(&data->lock);

		break;
	}
	case hwmon_fan_input:
		*val = data->tach_rpm[channel];
		break;
	case hwmon_fan_fault:
		*val = !!(data->fault_status[channel] &
				MAX31785_FAN_STATUS_FAULT_MASK);
		break;
	default:
		rc = -EOPNOTSUPP;
		break;
	};

	return rc;
}

static int max31785_read_pwm(struct max31785 *data, u32 attr, int channel,
		long *val)
{
	bool is_auto;
	bool is_rpm;
	int rc = 0;

	mutex_lock(&data->lock);

	is_rpm = !!(data->fan_config[channel] &
			MAX31785_FAN_CFG_CONTROL_MODE_RPM);
	is_auto = is_automatic_control_mode(data, channel);

	switch (attr) {
	case hwmon_pwm_enable:
	{
		bool pwm_enabled;

		pwm_enabled = (data->fan_config[channel] &
				MAX31785_FAN_CFG_PWM_ENABLE);

		if (!pwm_enabled)
			*val = 0;
		else if (is_auto)
			*val = 3;
		else if (is_rpm)
			*val = 2;
		else
			*val = 1;
		break;
	}
	case hwmon_pwm_input:
		if (is_rpm || is_auto)
			*val = data->pwm[channel] / 100;
		else
			*val = data->fan_command[channel]
				/ MAX31785_FAN_COMMAND_PWM_RATIO;
		break;
	default:
		rc = -EOPNOTSUPP;
	};

	mutex_unlock(&data->lock);

	return rc;
}

static int max31785_read(struct device *dev, enum hwmon_sensor_types type,
		u32 attr, int channel, long *val)
{
	struct max31785 *data;
	int rc = -EOPNOTSUPP;

	data = max31785_update_device(dev);

	if (IS_ERR(data))
		return PTR_ERR(data);

	switch (type) {
	case hwmon_fan:
		return max31785_read_fan(data, attr, channel, val);
	case hwmon_pwm:
		return max31785_read_pwm(data, attr, channel, val);
	default:
		break;
	}

	return rc;
}

static int max31785_write_fan(struct max31785 *data, u32 attr, int channel,
		long val)
{
	int rc = -EOPNOTSUPP;

	switch (attr) {
		break;
	case hwmon_fan_pulses:
		return max31785_fan_set_pulses(data, channel, val);
	case hwmon_fan_target:
		return max31785_fan_set_target(data, channel, val);
	default:
		break;
	};

	return rc;
}

static int max31785_write_pwm(struct max31785 *data, u32 attr, int channel,
		long val)
{
	int rc = -EOPNOTSUPP;

	switch (attr) {
	case hwmon_pwm_enable:
		return max31785_pwm_enable(data, channel, val);
	case hwmon_pwm_input:
		return max31785_pwm_set(data, channel, val);
	default:
		break;
	};

	return rc;
}

static int max31785_write(struct device *dev, enum hwmon_sensor_types type,
		u32 attr, int channel, long val)
{
	struct max31785 *data;
	int rc = -EOPNOTSUPP;

	data = dev_get_drvdata(dev);

	switch (type) {
	case hwmon_fan:
		return max31785_write_fan(data, attr, channel, val);
	case hwmon_pwm:
		return max31785_write_pwm(data, attr, channel, val);
	default:
		break;
	}

	return rc;

}

static umode_t max31785_is_visible(const void *_data,
		enum hwmon_sensor_types type, u32 attr, int channel)
{
	switch (type) {
	case hwmon_fan:
		switch (attr) {
		case hwmon_fan_input:
		case hwmon_fan_fault:
			return 0444;
		case hwmon_fan_pulses:
		case hwmon_fan_target:
			return 0644;
		default:
			break;
		};
		break;
	case hwmon_pwm:
		return 0644;
	default:
		break;
	};

	return 0;
}

static const struct hwmon_ops max31785_hwmon_ops = {
	.is_visible = max31785_is_visible,
	.read = max31785_read,
	.write = max31785_write,
};

static const struct hwmon_chip_info max31785_chip_info_0x3030 = {
	.ops = &max31785_hwmon_ops,
	.info = max31785_info_0x3030,
};

static const struct hwmon_chip_info max31785_chip_info_0x3040 = {
	.ops = &max31785_hwmon_ops,
	.info = max31785_info_0x3040,
};


static int max31785_get_capabilities(struct max31785 *data)
{
	s32 rc;

	rc = i2c_smbus_read_word_data(data->client, MAX31785_REG_MFR_REVISION);
	if (rc < 0)
		return rc;

	if (rc == 0x3040)
		data->capabilities |= MAX31785_CAP_FAST_ROTOR;

	return 0;
}

static int max31785_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = client->adapter;
	const struct hwmon_chip_info *chip;
	struct device *dev = &client->dev;
	struct device *hwmon_dev;
	struct max31785 *data;
	int rc;

	if (!i2c_check_functionality(adapter,
			I2C_FUNC_SMBUS_BYTE_DATA | I2C_FUNC_SMBUS_WORD_DATA))
		return -ENODEV;

	data = devm_kzalloc(dev, sizeof(struct max31785), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->client = client;
	mutex_init(&data->lock);

	rc = max31785_init_fans(data);
	if (rc)
		return rc;

	rc = max31785_get_capabilities(data);
	if (rc < 0)
		return rc;

	if (max31785_has_dual_rotor(data))
		chip = &max31785_chip_info_0x3040;
	else
		chip = &max31785_chip_info_0x3030;

	hwmon_dev = devm_hwmon_device_register_with_info(dev,
			client->name, data, chip, NULL);

	return PTR_ERR_OR_ZERO(hwmon_dev);
}

static const struct i2c_device_id max31785_id[] = {
	{ "max31785", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, max31785_id);

static struct i2c_driver max31785_driver = {
	.class		= I2C_CLASS_HWMON,
	.probe		= max31785_probe,
	.driver = {
		.name	= "max31785",
	},
	.id_table	= max31785_id,
	.detect		= max31785_detect,
	.address_list	= normal_i2c,
};

module_i2c_driver(max31785_driver);

MODULE_AUTHOR("Timothy Pearson <tpearson@raptorengineering.com>");
MODULE_DESCRIPTION("MAX31785 sensor driver");
MODULE_LICENSE("GPL");
