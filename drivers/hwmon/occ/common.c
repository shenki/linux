/*
 * Copyright 2017 IBM Corp.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/device.h>
#include <linux/hwmon-sysfs.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/mutex.h>
#include <asm/unaligned.h>

#include "common.h"

#define OCC_UPDATE_FREQUENCY		msecs_to_jiffies(1000)

#define OCC_TEMP_SENSOR_FAULT		0xFF

#define OCC_FRU_TYPE_VRM		0x3

/* OCC sensor type and version definitions */

struct temp_sensor_1 {
	u16 sensor_id;
	u16 value;
} __packed;

struct temp_sensor_2 {
	u32 sensor_id;
	u8 fru_type;
	u8 value;
} __packed;

struct freq_sensor_1 {
	u16 sensor_id;
	u16 value;
} __packed;

struct freq_sensor_2 {
	u32 sensor_id;
	u16 value;
} __packed;

struct power_sensor_1 {
	u16 sensor_id;
	u32 update_tag;
	u32 accumulator;
	u16 value;
} __packed;

struct power_sensor_2 {
	u32 sensor_id;
	u8 function_id;
	u8 apss_channel;
	u16 reserved;
	u32 update_tag;
	u64 accumulator;
	u16 value;
} __packed;

struct power_sensor_data {
	u16 value;
	u32 update_tag;
	u64 accumulator;
} __packed;

struct power_sensor_data_and_time {
	u16 update_time;
	u16 value;
	u32 update_tag;
	u64 accumulator;
} __packed;

struct power_sensor_a0 {
	u32 sensor_id;
	struct power_sensor_data_and_time system;
	u32 reserved;
	struct power_sensor_data_and_time proc;
	struct power_sensor_data vdd;
	struct power_sensor_data vdn;
} __packed;

struct caps_sensor_1 {
	u16 curr_powercap;
	u16 curr_powerreading;
	u16 norm_powercap;
	u16 max_powercap;
	u16 min_powercap;
	u16 user_powerlimit;
} __packed;

struct caps_sensor_2 {
	u16 curr_powercap;
	u16 curr_powerreading;
	u16 norm_powercap;
	u16 max_powercap;
	u16 min_powercap;
	u16 user_powerlimit;
	u8 user_powerlimit_source;
} __packed;

struct caps_sensor_3 {
	u16 curr_powercap;
	u16 curr_powerreading;
	u16 norm_powercap;
	u16 max_powercap;
	u16 hard_min_powercap;
	u16 soft_min_powercap;
	u16 user_powerlimit;
	u8 user_powerlimit_source;
} __packed;

struct extended_sensor {
	u8 name[4];
	u8 flags;
	u8 reserved;
	u8 data[6];
} __packed;

static int occ_poll(struct occ *occ)
{
	u16 checksum = occ->poll_cmd_data + 1;
	u8 cmd[8];

	/* big endian */
	cmd[0] = 0;			/* sequence number */
	cmd[1] = 0;			/* cmd type */
	cmd[2] = 0;			/* data length msb */
	cmd[3] = 1;			/* data length lsb */
	cmd[4] = occ->poll_cmd_data;	/* data */
	cmd[5] = checksum >> 8;		/* checksum msb */
	cmd[6] = checksum & 0xFF;	/* checksum lsb */
	cmd[7] = 0;

	/* mutex should already be locked if necessary */
	return occ->send_cmd(occ, cmd);
}

static int occ_set_user_power_cap(struct occ *occ, u16 user_power_cap)
{
	int rc;
	u8 cmd[8];
	u16 checksum = 0x24;
	__be16 user_power_cap_be = cpu_to_be16(user_power_cap);

	cmd[0] = 0;
	cmd[1] = 0x22;
	cmd[2] = 0;
	cmd[3] = 2;

	memcpy(&cmd[4], &user_power_cap_be, 2);

	checksum += cmd[4] + cmd[5];
	cmd[6] = checksum >> 8;
	cmd[7] = checksum & 0xFF;

	rc = mutex_lock_interruptible(&occ->lock);
	if (rc)
		return rc;

	rc = occ->send_cmd(occ, cmd);

	mutex_unlock(&occ->lock);

	return rc;
}

static int occ_update_response(struct occ *occ)
{
	int rc = mutex_lock_interruptible(&occ->lock);

	if (rc)
		return rc;

	/* limit the maximum rate of polling the OCC */
	if (time_after(jiffies, occ->last_update + OCC_UPDATE_FREQUENCY)) {
		rc = occ_poll(occ);
		occ->last_update = jiffies;
	}

	mutex_unlock(&occ->lock);
	return rc;
}

static ssize_t occ_show_temp_1(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	int rc;
	u16 val = 0;
	struct temp_sensor_1 *temp;
	struct occ *occ = dev_get_drvdata(dev);
	struct occ_sensors *sensors = &occ->sensors;
	struct sensor_device_attribute_2 *sattr = to_sensor_dev_attr_2(attr);

	rc = occ_update_response(occ);
	if (rc)
		return rc;

	temp = ((struct temp_sensor_1 *)sensors->temp.data) + sattr->index;

	switch (sattr->nr) {
	case 0:
		val = get_unaligned_be16(&temp->sensor_id);
		break;
	case 1:
		/* millidegrees */
		val = get_unaligned_be16(&temp->value) * 1000;
		break;
	default:
		return -EINVAL;
	}

	return snprintf(buf, PAGE_SIZE - 1, "%u\n", val);
}

static ssize_t occ_show_temp_2(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	int rc;
	u32 val = 0;
	struct temp_sensor_2 *temp;
	struct occ *occ = dev_get_drvdata(dev);
	struct occ_sensors *sensors = &occ->sensors;
	struct sensor_device_attribute_2 *sattr = to_sensor_dev_attr_2(attr);

	rc = occ_update_response(occ);
	if (rc)
		return rc;

	temp = ((struct temp_sensor_2 *)sensors->temp.data) + sattr->index;

	switch (sattr->nr) {
	case 0:
		val = get_unaligned_be32(&temp->sensor_id);
		break;
	case 1:
		val = temp->value;
		if (val == OCC_TEMP_SENSOR_FAULT)
			return -EREMOTEIO;

		if (temp->fru_type != OCC_FRU_TYPE_VRM) {
			/* sensor not ready */
			if (val == 0)
				return -EAGAIN;

			val *= 1000;	/* millidegrees */
		}
		break;
	case 2:
		val = temp->fru_type;
		break;
	case 3:
		val = temp->value == OCC_TEMP_SENSOR_FAULT;
		break;
	default:
		return -EINVAL;
	}

	return snprintf(buf, PAGE_SIZE - 1, "%u\n", val);
}

static ssize_t occ_show_freq_1(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	int rc;
	u16 val = 0;
	struct freq_sensor_1 *freq;
	struct occ *occ = dev_get_drvdata(dev);
	struct occ_sensors *sensors = &occ->sensors;
	struct sensor_device_attribute_2 *sattr = to_sensor_dev_attr_2(attr);

	rc = occ_update_response(occ);
	if (rc)
		return rc;

	freq = ((struct freq_sensor_1 *)sensors->freq.data) + sattr->index;

	switch (sattr->nr) {
	case 0:
		val = get_unaligned_be16(&freq->sensor_id);
		break;
	case 1:
		val = get_unaligned_be16(&freq->value);
		break;
	default:
		return -EINVAL;
	}

	return snprintf(buf, PAGE_SIZE - 1, "%u\n", val);
}

static ssize_t occ_show_freq_2(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	int rc;
	u32 val = 0;
	struct freq_sensor_2 *freq;
	struct occ *occ = dev_get_drvdata(dev);
	struct occ_sensors *sensors = &occ->sensors;
	struct sensor_device_attribute_2 *sattr = to_sensor_dev_attr_2(attr);

	rc = occ_update_response(occ);
	if (rc)
		return rc;

	freq = ((struct freq_sensor_2 *)sensors->freq.data) + sattr->index;

	switch (sattr->nr) {
	case 0:
		val = get_unaligned_be32(&freq->sensor_id);
		break;
	case 1:
		val = get_unaligned_be16(&freq->value);
		break;
	default:
		return -EINVAL;
	}

	return snprintf(buf, PAGE_SIZE - 1, "%u\n", val);
}

static ssize_t occ_show_power_1(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int rc;
	u32 val = 0;
	struct power_sensor_1 *power;
	struct occ *occ = dev_get_drvdata(dev);
	struct occ_sensors *sensors = &occ->sensors;
	struct sensor_device_attribute_2 *sattr = to_sensor_dev_attr_2(attr);

	rc = occ_update_response(occ);
	if (rc)
		return rc;

	power = ((struct power_sensor_1 *)sensors->power.data) + sattr->index;

	switch (sattr->nr) {
	case 0:
		val = get_unaligned_be16(&power->sensor_id);
		break;
	case 1:
		val = get_unaligned_be32(&power->update_tag);
		break;
	case 2:
		val = get_unaligned_be32(&power->accumulator);
		break;
	case 3:
		/* microwatts */
		val = get_unaligned_be16(&power->value) * 1000000;
		break;
	default:
		return -EINVAL;
	}

	return snprintf(buf, PAGE_SIZE - 1, "%u\n", val);
}

static ssize_t occ_show_power_2(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int rc;
	u64 val = 0;
	struct power_sensor_2 *power;
	struct occ *occ = dev_get_drvdata(dev);
	struct occ_sensors *sensors = &occ->sensors;
	struct sensor_device_attribute_2 *sattr = to_sensor_dev_attr_2(attr);

	rc = occ_update_response(occ);
	if (rc)
		return rc;

	power = ((struct power_sensor_2 *)sensors->power.data) + sattr->index;

	switch (sattr->nr) {
	case 0:
		val = get_unaligned_be32(&power->sensor_id);
		break;
	case 1:
		val = get_unaligned_be32(&power->update_tag);
		break;
	case 2:
		val = get_unaligned_be64(&power->accumulator);
		break;
	case 3:
		/* microwatts */
		val = get_unaligned_be16(&power->value) * 1000000;
		break;
	case 4:
		val = power->function_id;
		break;
	case 5:
		val = power->apss_channel;
		break;
	default:
		return -EINVAL;
	}

	return snprintf(buf, PAGE_SIZE - 1, "%llu\n", val);
}

static ssize_t occ_show_power_a0(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	int rc;
	u64 val = 0;
	struct power_sensor_a0 *power;
	struct occ *occ = dev_get_drvdata(dev);
	struct occ_sensors *sensors = &occ->sensors;
	struct sensor_device_attribute_2 *sattr = to_sensor_dev_attr_2(attr);

	rc = occ_update_response(occ);
	if (rc)
		return rc;

	power = ((struct power_sensor_a0 *)sensors->power.data) + sattr->index;

	switch (sattr->nr) {
	case 0:
		val = get_unaligned_be32(&power->sensor_id);
		break;
	case 1:
		return snprintf(buf, PAGE_SIZE - 1, "system\n");
	case 2:
		val = get_unaligned_be16(&power->system.update_time);
		break;
	case 3:
		/* microwatts */
		val = get_unaligned_be16(&power->system.value) * 1000000;
		break;
	case 4:
		val = get_unaligned_be32(&power->system.update_tag);
		break;
	case 5:
		val = get_unaligned_be64(&power->system.accumulator);
		break;
	case 6:
		return snprintf(buf, PAGE_SIZE - 1, "proc\n");
	case 7:
		val = get_unaligned_be16(&power->proc.update_time);
		break;
	case 8:
		/* microwatts */
		val = get_unaligned_be16(&power->proc.value) * 1000000;
		break;
	case 9:
		val = get_unaligned_be32(&power->proc.update_tag);
		break;
	case 10:
		val = get_unaligned_be64(&power->proc.accumulator);
		break;
	case 11:
		return snprintf(buf, PAGE_SIZE - 1, "vdd\n");
	case 12:
		/* microwatts */
		val = get_unaligned_be16(&power->vdd.value) * 1000000;
		break;
	case 13:
		val = get_unaligned_be32(&power->vdd.update_tag);
		break;
	case 14:
		val = get_unaligned_be64(&power->vdd.accumulator);
		break;
	case 15:
		return snprintf(buf, PAGE_SIZE - 1, "vdn\n");
	case 16:
		/* microwatts */
		val = get_unaligned_be16(&power->vdn.value) * 1000000;
		break;
	case 17:
		val = get_unaligned_be32(&power->vdn.update_tag);
		break;
	case 18:
		val = get_unaligned_be64(&power->vdn.accumulator);
		break;
	default:
		return -EINVAL;
	}

	return snprintf(buf, PAGE_SIZE - 1, "%llu\n", val);
}

static ssize_t occ_show_caps_1(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	int rc;
	u16 val = 0;
	struct caps_sensor_1 *caps;
	struct occ *occ = dev_get_drvdata(dev);
	struct occ_sensors *sensors = &occ->sensors;
	struct sensor_device_attribute_2 *sattr = to_sensor_dev_attr_2(attr);

	rc = occ_update_response(occ);
	if (rc)
		return rc;

	caps = ((struct caps_sensor_1 *)sensors->caps.data) + sattr->index;

	switch (sattr->nr) {
	case 0:
		val = get_unaligned_be16(&caps->curr_powercap);
		break;
	case 1:
		val = get_unaligned_be16(&caps->curr_powerreading);
		break;
	case 2:
		val = get_unaligned_be16(&caps->norm_powercap);
		break;
	case 3:
		val = get_unaligned_be16(&caps->max_powercap);
		break;
	case 4:
		val = get_unaligned_be16(&caps->min_powercap);
		break;
	case 5:
		val = get_unaligned_be16(&caps->user_powerlimit);
		break;
	default:
		return -EINVAL;
	}

	return snprintf(buf, PAGE_SIZE - 1, "%u\n", val);
}

static ssize_t occ_show_caps_2(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	int rc;
	u16 val = 0;
	struct caps_sensor_2 *caps;
	struct occ *occ = dev_get_drvdata(dev);
	struct occ_sensors *sensors = &occ->sensors;
	struct sensor_device_attribute_2 *sattr = to_sensor_dev_attr_2(attr);

	rc = occ_update_response(occ);
	if (rc)
		return rc;

	caps = ((struct caps_sensor_2 *)sensors->caps.data) + sattr->index;

	switch (sattr->nr) {
	case 0:
		val = get_unaligned_be16(&caps->curr_powercap);
		break;
	case 1:
		val = get_unaligned_be16(&caps->curr_powerreading);
		break;
	case 2:
		val = get_unaligned_be16(&caps->norm_powercap);
		break;
	case 3:
		val = get_unaligned_be16(&caps->max_powercap);
		break;
	case 4:
		val = get_unaligned_be16(&caps->min_powercap);
		break;
	case 5:
		val = get_unaligned_be16(&caps->user_powerlimit);
		break;
	case 6:
		val = caps->user_powerlimit_source;
		break;
	default:
		return -EINVAL;
	}

	return snprintf(buf, PAGE_SIZE - 1, "%u\n", val);
}

static ssize_t occ_show_caps_3(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	int rc;
	u16 val = 0;
	struct caps_sensor_3 *caps;
	struct occ *occ = dev_get_drvdata(dev);
	struct occ_sensors *sensors = &occ->sensors;
	struct sensor_device_attribute_2 *sattr = to_sensor_dev_attr_2(attr);

	rc = occ_update_response(occ);
	if (rc)
		return rc;

	caps = ((struct caps_sensor_3 *)sensors->caps.data) + sattr->index;

	switch (sattr->nr) {
	case 0:
		val = get_unaligned_be16(&caps->curr_powercap);
		break;
	case 1:
		val = get_unaligned_be16(&caps->curr_powerreading);
		break;
	case 2:
		val = get_unaligned_be16(&caps->norm_powercap);
		break;
	case 3:
		val = get_unaligned_be16(&caps->max_powercap);
		break;
	case 4:
		val = get_unaligned_be16(&caps->hard_min_powercap);
		break;
	case 5:
		val = get_unaligned_be16(&caps->user_powerlimit);
		break;
	case 6:
		val = caps->user_powerlimit_source;
		break;
	case 7:
		val = get_unaligned_be16(&caps->soft_min_powercap);
		break;
	default:
		return -EINVAL;
	}

	return snprintf(buf, PAGE_SIZE - 1, "%u\n", val);
}

static ssize_t occ_store_caps_user(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	int rc;
	u16 user_power_cap;
	struct occ *occ = dev_get_drvdata(dev);

	rc = kstrtou16(buf, 0, &user_power_cap);
	if (rc)
		return rc;

	rc = occ_set_user_power_cap(occ, user_power_cap);
	if (rc)
		return rc;

	return count;
}

static ssize_t occ_show_extended(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	int rc;
	struct extended_sensor *extn;
	struct occ *occ = dev_get_drvdata(dev);
	struct occ_sensors *sensors = &occ->sensors;
	struct sensor_device_attribute_2 *sattr = to_sensor_dev_attr_2(attr);

	rc = occ_update_response(occ);
	if (rc)
		return rc;

	extn = ((struct extended_sensor *)sensors->extended.data) +
		sattr->index;

	switch (sattr->nr) {
	case 0:
		rc = snprintf(buf, PAGE_SIZE - 1, "%02x%02x%02x%02x\n",
			      extn->name[0], extn->name[1], extn->name[2],
			      extn->name[3]);
		break;
	case 1:
		rc = snprintf(buf, PAGE_SIZE - 1, "%02x\n", extn->flags);
		break;
	case 2:
		rc = snprintf(buf, PAGE_SIZE - 1, "%02x%02x%02x%02x%02x%02x\n",
			      extn->data[0], extn->data[1], extn->data[2],
			      extn->data[3], extn->data[4], extn->data[5]);
		break;
	default:
		return -EINVAL;
	}

	return rc;
}

/* only need to do this once at startup, as OCC won't change sensors on us */
static void occ_parse_poll_response(struct occ *occ)
{
	unsigned int i, offset = 0, size = 0;
	struct occ_sensor *sensor;
	struct occ_sensors *sensors = &occ->sensors;
	struct occ_response *resp = &occ->resp;
	struct occ_poll_response *poll =
		(struct occ_poll_response *)&resp->data[0];
	struct occ_poll_response_header *header = &poll->header;
	struct occ_sensor_data_block *block = &poll->block;

	for (i = 0; i < header->num_sensor_data_blocks; ++i) {
		block = (struct occ_sensor_data_block *)((u8 *)block + offset);
		offset = (block->header.num_sensors *
			  block->header.sensor_length) + sizeof(block->header);
		size += offset;

		/* validate all the length/size fields */
		if ((size + sizeof(*header)) >= OCC_RESP_DATA_BYTES) {
			dev_warn(occ->bus_dev, "exceeded response buffer\n");
			return;
		}

		/* match sensor block type */
		if (strncmp(block->header.eye_catcher, "TEMP", 4) == 0)
			sensor = &sensors->temp;
		else if (strncmp(block->header.eye_catcher, "FREQ", 4) == 0)
			sensor = &sensors->freq;
		else if (strncmp(block->header.eye_catcher, "POWR", 4) == 0)
			sensor = &sensors->power;
		else if (strncmp(block->header.eye_catcher, "CAPS", 4) == 0)
			sensor = &sensors->caps;
		else if (strncmp(block->header.eye_catcher, "EXTN", 4) == 0)
			sensor = &sensors->extended;
		else {
			dev_warn(occ->bus_dev, "sensor not supported %.4s\n",
				 block->header.eye_catcher);
			continue;
		}

		sensor->num_sensors = block->header.num_sensors;
		sensor->version = block->header.sensor_format;
		sensor->data = &block->data;
	}
}

int occ_setup(struct occ *occ, const char *name)
{
	int rc;

	mutex_init(&occ->lock);

	/* no need to lock */
	rc = occ_poll(occ);
	if (rc < 0) {
		dev_err(occ->bus_dev, "failed to get OCC poll response: %d\n",
			rc);
		return rc;
	}

	occ_parse_poll_response(occ);

	return 0;
}
