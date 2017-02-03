/*
 * A FSI master controller, using a simple GPIO bit-banging interface
 */

#include <linux/platform_device.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/fsi.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/spinlock.h>

#include "fsi-master.h"

#define	FSI_GPIO_STD_DLY	1	/* Standard pin delay in nS */
#define	FSI_ECHO_DELAY_CLOCKS	16	/* Number clocks for echo delay */
#define	FSI_PRE_BREAK_CLOCKS	50	/* Number clocks to prep for break */
#define	FSI_BREAK_CLOCKS	256	/* Number of clocks to issue break */
#define	FSI_POST_BREAK_CLOCKS	16000	/* Number clocks to set up cfam */
#define	FSI_INIT_CLOCKS		5000	/* Clock out any old data */
#define	FSI_GPIO_STD_DELAY	10	/* Standard GPIO delay in nS */
					/* todo: adjust down as low as */
					/* possible or eliminate */
#define	FSI_GPIO_CMD_DPOLL	0x000000000000002AULL
#define	FSI_GPIO_CMD_DPOLL_SIZE	9
#define	FSI_GPIO_DPOLL_CLOCKS	100      /* < 21 will cause slave to hang */
#define	FSI_GPIO_CMD_DEFAULT	0x2000000000000000ULL
#define	FSI_GPIO_CMD_WRITE	0
#define	FSI_GPIO_CMD_READ	0x0400000000000000ULL
#define	FSI_GPIO_CMD_SLAVE_MASK	0xC000000000000000ULL
#define	FSI_GPIO_CMD_ADDR_SHIFT	37
#define	FSI_GPIO_CMD_ADDR_MASK	0x001FFFFF
#define	FSI_GPIO_CMD_SLV_SHIFT	62
#define	FSI_GPIO_CMD_SIZE_16	0x0000001000000000ULL
#define	FSI_GPIO_CMD_SIZE_32	0x0000003000000000ULL
#define	FSI_GPIO_CMD_DT32_SHIFT	4
#define	FSI_GPIO_CMD_DT16_SHIFT	20
#define	FSI_GPIO_CMD_DT8_SHIFT	28
#define	FSI_GPIO_CMD_DFLT_LEN	28
#define	FSI_GPIO_CMD_CRC_SHIFT	60

/* Bus errors */
#define	FSI_GPIO_ERR_BUSY	1	/* Slave stuck in busy state */
#define	FSI_GPIO_RESP_ERRA	2	/* Any (misc) Error */
#define	FSI_GPIO_RESP_ERRC	3	/* Slave reports master CRC error */
#define	FSI_GPIO_MTOE		4	/* Master time out error */
#define	FSI_GPIO_CRC_INVAL	5	/* Master reports slave CRC error */

/* Normal slave responses */
#define	FSI_GPIO_RESP_BUSY	1
#define	FSI_GPIO_RESP_ACK	0
#define	FSI_GPIO_RESP_ACKD	4

#define	FSI_GPIO_MAX_BUSY	100
#define	FSI_GPIO_MTOE_COUNT	1000
#define	FSI_GPIO_DRAIN_BITS	20
#define	FSI_GPIO_CRC_SIZE	4
#define	FSI_GPIO_MSG_ID_SIZE		2
#define	FSI_GPIO_MSG_RESPID_SIZE	2
#define	FSI_GPIO_PRIME_SLAVE_CLOCKS	100
#define	FSI_GPIO_WAKE_CLOCKS		32	/* >= 32 ok, 16 fails */

static DEFINE_SPINLOCK(fsi_gpio_cmd_lock);	/* lock around fsi commands */

struct fsi_master_gpio {
	struct fsi_master	master;
	struct gpio_desc	*gpio_clk;
	struct gpio_desc	*gpio_data;
	struct gpio_desc	*gpio_trans;	/* Voltage translator */
	struct gpio_desc	*gpio_enable;	/* FSI enable */
	struct gpio_desc	*gpio_mux;	/* Mux control */
};

#define to_fsi_master_gpio(m) container_of(m, struct fsi_master_gpio, master)

struct fsi_gpio_msg {
	uint64_t	msg;
	uint8_t		bits;
};

static void clock_toggle(struct fsi_master_gpio *master, int count)
{
	int i;

	for (i = 0; i < count; i++) {
		ndelay(FSI_GPIO_STD_DLY);
		gpiod_set_value(master->gpio_clk, 0);
		ndelay(FSI_GPIO_STD_DLY);
		gpiod_set_value(master->gpio_clk, 1);
	}
}

static int sda_in(struct fsi_master_gpio *master)
{
	int in;

	ndelay(FSI_GPIO_STD_DLY);
	in = gpiod_get_value(master->gpio_data);
	return in ? 1 : 0;
}

static void sda_out(struct fsi_master_gpio *master, int value)
{
	gpiod_set_value(master->gpio_data, value);
}

static void set_sda_input(struct fsi_master_gpio *master)
{
	gpiod_direction_input(master->gpio_data);
	if (master->gpio_trans)
		gpiod_set_value(master->gpio_trans, 0);
}

static void set_sda_output(struct fsi_master_gpio *master, int value)
{
	if (master->gpio_trans)
		gpiod_set_value(master->gpio_trans, 1);
	gpiod_direction_output(master->gpio_data, value);
}

static void serial_in(struct fsi_master_gpio *master, struct fsi_gpio_msg *cmd,
			uint8_t num_bits)
{
	uint8_t bit;
	uint64_t msg = 0;
	uint8_t in_bit = 0;

	set_sda_input(master);

	for (bit = 0; bit < num_bits; bit++) {
		clock_toggle(master, 1);
		in_bit = sda_in(master);
		msg <<= 1;
		msg |= ~in_bit & 0x1;	/* Data is negative active */
	}
	cmd->bits = num_bits;
	cmd->msg = msg;
}

static void serial_out(struct fsi_master_gpio *master,
			const struct fsi_gpio_msg *cmd)
{
	uint8_t bit;
	uint64_t msg = ~cmd->msg;	/* Data is negative active */
	uint64_t sda_mask = 0x1ULL << (cmd->bits - 1);
	uint64_t last_bit = ~0;
	int next_bit;

	if (!cmd->bits) {
		dev_warn(master->master.dev, "trying to output 0 bits\n");
		return;
	}
	set_sda_output(master, 0);

	/* Send the start bit */
	sda_out(master, 0);
	clock_toggle(master, 1);

	/* Send the message */
	for (bit = 0; bit < cmd->bits; bit++) {
		next_bit = (msg & sda_mask) >> (cmd->bits - 1);
		if (last_bit ^ next_bit) {
			sda_out(master, next_bit);
			last_bit = next_bit;
		}
		clock_toggle(master, 1);
		msg <<= 1;
	}
}

/*
 * Clock out some 0's after every message to ride out line reflections
 */
static void echo_delay(struct fsi_master_gpio *master)
{
	set_sda_output(master, 1);
	clock_toggle(master, FSI_ECHO_DELAY_CLOCKS);
}

/*
 * Wake up the slave by clocking it a number of times prior to sending the next
 * master command.  Needed when the bus has been idle for a while.
 */
static void wake_slave(struct fsi_master_gpio *master)
{
	set_sda_output(master, 1);
	clock_toggle(master, FSI_GPIO_WAKE_CLOCKS);
}

/*
 * Used in bus error cases only.  Clears out any remaining data the slave
 * is attempting to send
 */
static void drain_response(struct fsi_master_gpio *master)
{
	struct fsi_gpio_msg msg;

	serial_in(master, &msg, FSI_GPIO_DRAIN_BITS);
}

/*
 * Store information on master errors so handler can detect and clean
 * up the bus
 */
static void fsi_master_gpio_error(struct fsi_master_gpio *master, int error)
{

}

static int poll_for_response(struct fsi_master_gpio *master, uint8_t expected,
			uint8_t size, void *data)
{
	int busy_count = 0, i;
	struct fsi_gpio_msg response, cmd;
	int bits_remaining = 0, bit_count, response_id, id;
	uint64_t resp = 0;
	uint8_t bits_received = FSI_GPIO_MSG_ID_SIZE +
				FSI_GPIO_MSG_RESPID_SIZE;
	uint8_t crc_in;

	do {
		for (i = 0; i < FSI_GPIO_MTOE_COUNT; i++) {
			serial_in(master, &response, 1);
			if (response.msg)
				break;
		}
		if (i >= FSI_GPIO_MTOE_COUNT) {
			dev_dbg(master->master.dev,
				"Master time out waiting for response\n");
			drain_response(master);
			fsi_master_gpio_error(master, FSI_GPIO_MTOE);
			return -EIO;
		}

		/* Response received */
		bit_count = FSI_GPIO_MSG_ID_SIZE + FSI_GPIO_MSG_RESPID_SIZE;
		serial_in(master, &response, bit_count);

		response_id = response.msg & 0x3;
		id = (response.msg >> FSI_GPIO_MSG_RESPID_SIZE) & 0x3;
		dev_dbg(master->master.dev, "id:%d resp:%d\n", id, response_id);

		resp = response.msg;

		switch (response_id) {
		case FSI_GPIO_RESP_ACK:
			if (expected == FSI_GPIO_RESP_ACKD)
				bits_remaining = 8 * size;
			break;

		case FSI_GPIO_RESP_BUSY:
			/*
			 * Its necessary to clock slave before issuing
			 * d-poll, not indicated in the hardware protocol
			 * spec. < 20 clocks causes slave to hang, 21 ok.
			 */
			set_sda_output(master, 1);
			clock_toggle(master, FSI_GPIO_DPOLL_CLOCKS);
			cmd.msg = FSI_GPIO_CMD_DPOLL;
			cmd.bits = FSI_GPIO_CMD_DPOLL_SIZE;
			serial_out(master, &cmd);
			echo_delay(master);
			continue;

		case FSI_GPIO_RESP_ERRA:
		case FSI_GPIO_RESP_ERRC:
			dev_dbg(master->master.dev, "ERR received: %d\n",
				(int)response.msg);
			/*
			 * todo: Verify crc from slave and in general
			 * only act on any response if crc is correct
			 */
			clock_toggle(master, FSI_GPIO_CRC_SIZE);
			fsi_master_gpio_error(master, response.msg);
			return -EIO;
		}

		/* Read in the data field if applicable */
		if (bits_remaining) {
			serial_in(master, &response, bits_remaining);
			resp <<= bits_remaining;
			resp |= response.msg;
			bits_received += bits_remaining;
			*((uint32_t *)data) = response.msg;
		}

		crc_in = fsi_crc4(0, resp | (0x1ULL << bits_received),
					bits_received + 1);

		/* Read in the crc and check it */
		serial_in(master, &response, FSI_GPIO_CRC_SIZE);
		if (crc_in != response.msg) {
			dev_dbg(master->master.dev, "ERR response CRC\n");
			fsi_master_gpio_error(master, FSI_GPIO_CRC_INVAL);
			return -EIO;
		}
		/* Clock the slave enough to be ready for next operation */
		clock_toggle(master, FSI_GPIO_PRIME_SLAVE_CLOCKS);
		return 0;

	} while (busy_count++ < FSI_GPIO_MAX_BUSY);

	dev_dbg(master->master.dev, "ERR slave is stuck in busy state\n");
	fsi_master_gpio_error(master, FSI_GPIO_ERR_BUSY);

	return -EIO;
}

static void build_abs_ar_command(struct fsi_gpio_msg *cmd, uint64_t mode,
		uint8_t slave, uint32_t addr, size_t size,
		const void *data)
{
	uint8_t crc;

	cmd->bits = FSI_GPIO_CMD_DFLT_LEN;
	cmd->msg = FSI_GPIO_CMD_DEFAULT;
	cmd->msg |= mode;
	cmd->msg &= ~FSI_GPIO_CMD_SLAVE_MASK;
	cmd->msg |= (((uint64_t)slave) << FSI_GPIO_CMD_SLV_SHIFT);
	addr &= FSI_GPIO_CMD_ADDR_MASK;
	cmd->msg |= (((uint64_t)addr) << FSI_GPIO_CMD_ADDR_SHIFT);
	if (size == sizeof(uint8_t)) {
		if (data) {
			uint8_t cmd_data = *((uint8_t *)data);

			cmd->msg |=
				((uint64_t)cmd_data) << FSI_GPIO_CMD_DT8_SHIFT;
		}
	} else if (size == sizeof(uint16_t)) {
		cmd->msg |= FSI_GPIO_CMD_SIZE_16;
		if (data) {
			uint16_t cmd_data;

			memcpy(&cmd_data, data, size);
			cmd->msg |=
				((uint64_t)cmd_data) << FSI_GPIO_CMD_DT16_SHIFT;
		}
	} else {
		cmd->msg |= FSI_GPIO_CMD_SIZE_32;
		if (data) {
			uint32_t cmd_data;

			memcpy(&cmd_data, data, size);
			cmd->msg |=
				((uint64_t)cmd_data) << FSI_GPIO_CMD_DT32_SHIFT;
		}
	}

	if (mode == FSI_GPIO_CMD_WRITE)
		cmd->bits += (8 * size);

	/* Include start bit */
	crc = fsi_crc4(0,
			(cmd->msg >> (64 - cmd->bits)) | (0x1ULL << cmd->bits),
			cmd->bits + 1);
	cmd->msg |= ((uint64_t)crc) << (FSI_GPIO_CMD_CRC_SHIFT - cmd->bits);
	cmd->bits += FSI_GPIO_CRC_SIZE;

	/* Right align message */
	cmd->msg >>= (64 - cmd->bits);
}

static int fsi_master_gpio_read(struct fsi_master *_master, int link,
		uint8_t slave, uint32_t addr, void *val, size_t size)
{
	struct fsi_master_gpio *master = to_fsi_master_gpio(_master);
	struct fsi_gpio_msg cmd;
	int rc;
	unsigned long flags;

	if (link != 0)
		return -ENODEV;

	build_abs_ar_command(&cmd, FSI_GPIO_CMD_READ, slave, addr, size, NULL);

	spin_lock_irqsave(&fsi_gpio_cmd_lock, flags);
	wake_slave(master);
	serial_out(master, &cmd);
	echo_delay(master);
	rc = poll_for_response(master, FSI_GPIO_RESP_ACKD, size, val);
	spin_unlock_irqrestore(&fsi_gpio_cmd_lock, flags);

	return rc;
}

static int fsi_master_gpio_write(struct fsi_master *_master, int link,
		uint8_t slave, uint32_t addr, const void *val, size_t size)
{
	struct fsi_master_gpio *master = to_fsi_master_gpio(_master);
	struct fsi_gpio_msg cmd;
	int rc;
	unsigned long flags;

	if (link != 0)
		return -ENODEV;

	build_abs_ar_command(&cmd, FSI_GPIO_CMD_WRITE, slave, addr, size, val);

	spin_lock_irqsave(&fsi_gpio_cmd_lock, flags);
	wake_slave(master);
	serial_out(master, &cmd);
	echo_delay(master);
	rc = poll_for_response(master, FSI_GPIO_RESP_ACK, size, NULL);
	spin_unlock_irqrestore(&fsi_gpio_cmd_lock, flags);

	return rc;
}

/*
 * Issue a break command on link
 */
static int fsi_master_gpio_break(struct fsi_master *_master, int link)
{
	struct fsi_master_gpio *master = to_fsi_master_gpio(_master);

	if (link != 0)
		return -ENODEV;

	set_sda_output(master, 1);
	clock_toggle(master, FSI_PRE_BREAK_CLOCKS);
	sda_out(master, 0);
	clock_toggle(master, FSI_BREAK_CLOCKS);
	echo_delay(master);
	sda_out(master, 1);
	clock_toggle(master, FSI_POST_BREAK_CLOCKS);

	/* Wait for logic reset to take effect */
	udelay(200);

	return 0;
}

static void fsi_master_gpio_init(struct fsi_master_gpio *master)
{
	if (master->gpio_mux)
		gpiod_direction_output(master->gpio_mux, 1);
	if (master->gpio_trans)
		gpiod_direction_output(master->gpio_trans, 1);
	if (master->gpio_enable)
		gpiod_direction_output(master->gpio_enable, 1);
	gpiod_direction_output(master->gpio_clk, 1);
	gpiod_direction_output(master->gpio_data, 1);

	/* todo: evaluate if clocks can be reduced */
	clock_toggle(master, FSI_INIT_CLOCKS);
}

static int fsi_master_gpio_link_enable(struct fsi_master *_master, int link)
{
	struct fsi_master_gpio *master = to_fsi_master_gpio(_master);

	if (link != 0)
		return -ENODEV;
	if (master->gpio_enable)
		gpiod_set_value(master->gpio_enable, 1);

	return 0;
}

static ssize_t store_scan(struct device *dev,
				struct device_attribute *attr,
				const char *buf,
				size_t count)
{
	struct fsi_master_gpio *master = dev_get_drvdata(dev);

	fsi_master_gpio_init(master);

	/* clear out any old scan data if present */
	fsi_master_unregister(&master->master);
	fsi_master_register(&master->master);
	fsi_master_start_ipoll(&master->master);

	return count;
}

static DEVICE_ATTR(scan, 0200, NULL, store_scan);

static int fsi_master_gpio_probe(struct platform_device *pdev)
{
	struct fsi_master_gpio *master;
	struct gpio_desc *gpio;

	master = devm_kzalloc(&pdev->dev, sizeof(*master), GFP_KERNEL);
	if (!master)
		return -ENOMEM;
	master->master.dev = &pdev->dev;

	gpio = devm_gpiod_get(&pdev->dev, "clock", 0);
	if (IS_ERR(gpio)) {
		dev_dbg(&pdev->dev, "probe: failed to get clock pin\n");
		return PTR_ERR(gpio);
	}
	master->gpio_clk = gpio;

	gpio = devm_gpiod_get(&pdev->dev, "data", 0);
	if (IS_ERR(gpio)) {
		dev_dbg(&pdev->dev, "probe: failed to get data pin\n");
		return PTR_ERR(gpio);
	}
	master->gpio_data = gpio;

	/* Optional pins */

	gpio = devm_gpiod_get(&pdev->dev, "trans", 0);
	if (IS_ERR(gpio))
		dev_dbg(&pdev->dev, "probe: failed to get trans pin\n");
	else
		master->gpio_trans = gpio;

	gpio = devm_gpiod_get(&pdev->dev, "enable", 0);
	if (IS_ERR(gpio))
		dev_dbg(&pdev->dev, "probe: failed to get enable pin\n");
	else
		master->gpio_enable = gpio;

	gpio = devm_gpiod_get(&pdev->dev, "mux", 0);
	if (IS_ERR(gpio))
		dev_dbg(&pdev->dev, "probe: failed to get mux pin\n");
	else
		master->gpio_mux = gpio;

	master->master.n_links = 1;
	master->master.read = fsi_master_gpio_read;
	master->master.write = fsi_master_gpio_write;
	master->master.send_break = fsi_master_gpio_break;
	master->master.link_enable = fsi_master_gpio_link_enable;
	platform_set_drvdata(pdev, master);

	return device_create_file(&pdev->dev, &dev_attr_scan);
}


static int fsi_master_gpio_remove(struct platform_device *pdev)
{
	struct fsi_master_gpio *master = platform_get_drvdata(pdev);

	devm_gpiod_put(&pdev->dev, master->gpio_clk);
	devm_gpiod_put(&pdev->dev, master->gpio_data);
	if (master->gpio_trans)
		devm_gpiod_put(&pdev->dev, master->gpio_trans);
	if (master->gpio_enable)
		devm_gpiod_put(&pdev->dev, master->gpio_enable);
	if (master->gpio_mux)
		devm_gpiod_put(&pdev->dev, master->gpio_mux);
	fsi_master_unregister(&master->master);

	return 0;
}

static const struct of_device_id fsi_master_gpio_match[] = {
	{ .compatible = "ibm,fsi-master-gpio" },
	{ },
};

static struct platform_driver fsi_master_gpio_driver = {
	.driver = {
		.name		= "fsi-master-gpio",
		.of_match_table	= fsi_master_gpio_match,
	},
	.probe	= fsi_master_gpio_probe,
	.remove = fsi_master_gpio_remove,
};

module_platform_driver(fsi_master_gpio_driver);
MODULE_LICENSE("GPL");
