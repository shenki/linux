#include <linux/init.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/io.h>
#include <linux/pinctrl/machine.h>
#include <linux/pinctrl/consumer.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

// XXX TEMP HACKERY
//
// To be replaced by proper clock, pinmux and syscon drivers operating
// from DT parameters
static struct pinctrl_map palmetto_mapping[] __initdata = {
	PIN_MAP_MUX_GROUP_DEFAULT("i2c-3", "1e6e2000.pinmux", NULL, "I2C3"),
	PIN_MAP_MUX_GROUP_DEFAULT("i2c-4", "1e6e2000.pinmux", NULL, "I2C4"),
	PIN_MAP_MUX_GROUP_DEFAULT("i2c-5", "1e6e2000.pinmux", NULL, "I2C5"),
	PIN_MAP_MUX_GROUP_DEFAULT("i2c-6", "1e6e2000.pinmux", NULL, "I2C6"),
	PIN_MAP_MUX_GROUP_DEFAULT("i2c-7", "1e6e2000.pinmux", NULL, "I2C7"),
	PIN_MAP_MUX_GROUP_DEFAULT("i2c-8", "1e6e2000.pinmux", NULL, "I2C8"),
	PIN_MAP_MUX_GROUP_HOG_DEFAULT("1e6e2000.pinmux", NULL, "NCTS4"),
	PIN_MAP_MUX_GROUP_HOG_DEFAULT("1e6e2000.pinmux", NULL, "NDCD4"),
	PIN_MAP_MUX_GROUP_HOG_DEFAULT("1e6e2000.pinmux", NULL, "NRI4"),
	PIN_MAP_MUX_GROUP_HOG_DEFAULT("1e6e2000.pinmux", NULL, "TXD4"),
	PIN_MAP_MUX_GROUP_HOG_DEFAULT("1e6e2000.pinmux", NULL, "RXD4"),
	PIN_MAP_MUX_GROUP_HOG_DEFAULT("1e6e2000.pinmux", NULL, "FLBUSY"),
	PIN_MAP_MUX_GROUP_HOG_DEFAULT("1e6e2000.pinmux", NULL, "FLWP"),
	PIN_MAP_MUX_GROUP_HOG_DEFAULT("1e6e2000.pinmux", NULL, "VGAHS"),
	PIN_MAP_MUX_GROUP_HOG_DEFAULT("1e6e2000.pinmux", NULL, "VGAVS"),
	PIN_MAP_MUX_GROUP_HOG_DEFAULT("1e6e2000.pinmux", NULL, "DDCCLK"),
	PIN_MAP_MUX_GROUP_HOG_DEFAULT("1e6e2000.pinmux", NULL, "DDCDAT"),
	PIN_MAP_MUX_GROUP_HOG_DEFAULT("1e6e2000.pinmux", NULL, "NCTS1"),
	PIN_MAP_MUX_GROUP_HOG_DEFAULT("1e6e2000.pinmux", NULL, "NDCD1"),
	PIN_MAP_MUX_GROUP_HOG_DEFAULT("1e6e2000.pinmux", NULL, "NDSR1"),
	PIN_MAP_MUX_GROUP_HOG_DEFAULT("1e6e2000.pinmux", NULL, "NRI1"),
	PIN_MAP_MUX_GROUP_HOG_DEFAULT("1e6e2000.pinmux", NULL, "NDTR1"),
	PIN_MAP_MUX_GROUP_HOG_DEFAULT("1e6e2000.pinmux", NULL, "NRTS1"),
	PIN_MAP_MUX_GROUP_HOG_DEFAULT("1e6e2000.pinmux", NULL, "TXD1"),
	PIN_MAP_MUX_GROUP_HOG_DEFAULT("1e6e2000.pinmux", NULL, "RXD1"),
	PIN_MAP_MUX_GROUP_HOG_DEFAULT("1e6e2000.pinmux", NULL, "PWM0"),
	PIN_MAP_MUX_GROUP_HOG_DEFAULT("1e6e2000.pinmux", NULL, "PWM1"),
	PIN_MAP_MUX_GROUP_HOG_DEFAULT("1e6e2000.pinmux", NULL, "PWM2"),
	PIN_MAP_MUX_GROUP_HOG_DEFAULT("1e6e2000.pinmux", NULL, "PWM3"),
	PIN_MAP_MUX_GROUP_HOG_DEFAULT("1e6e2000.pinmux", NULL, "PWM4"),
	PIN_MAP_MUX_GROUP_HOG_DEFAULT("1e6e2000.pinmux", NULL, "PWM5"),
	PIN_MAP_MUX_GROUP_HOG_DEFAULT("1e6e2000.pinmux", NULL, "PWM6"),
	PIN_MAP_MUX_GROUP_HOG_DEFAULT("1e6e2000.pinmux", NULL, "PWM7"),
	PIN_MAP_MUX_GROUP_HOG_DEFAULT("1e6e2000.pinmux", NULL, "BMCINT"),
	PIN_MAP_MUX_GROUP_HOG_DEFAULT("1e6e2000.pinmux", NULL, "FLACK"),
	PIN_MAP_MUX_GROUP_HOG_DEFAULT("1e6e2000.pinmux", NULL, "ROM8"),
};

static struct pinctrl_map ast2500evb_mapping[] __initdata = {
	PIN_MAP_MUX_GROUP_DEFAULT("i2c-11", "1e6e2000.pinmux", NULL, "I2C12"),
	PIN_MAP_MUX_GROUP_DEFAULT("i2c-8", "1e6e2000.pinmux", NULL, "I2C9"),
	PIN_MAP_MUX_GROUP_DEFAULT("i2c-3", "1e6e2000.pinmux", NULL, "I2C4"),
	PIN_MAP_MUX_GROUP_DEFAULT("i2c-2", "1e6e2000.pinmux", NULL, "I2C3"),
};


static struct pinctrl_map witherspoon_mapping[] __initdata = {
	PIN_MAP_MUX_GROUP_DEFAULT("i2c-11", "1e6e2000.pinmux", NULL, "I2C12"),
	PIN_MAP_MUX_GROUP_DEFAULT("i2c-8", "1e6e2000.pinmux", NULL, "I2C9"),
	PIN_MAP_MUX_GROUP_DEFAULT("i2c-3", "1e6e2000.pinmux", NULL, "I2C4"),
	PIN_MAP_MUX_GROUP_DEFAULT("i2c-2", "1e6e2000.pinmux", NULL, "I2C3"),
};

static void __init aspeed_dt_init(void)
{
	int ret = 0;

	if (of_machine_is_compatible("tyan,palmetto-bmc"))
		ret = pinctrl_register_mappings(palmetto_mapping,
				ARRAY_SIZE(palmetto_mapping));

	if (of_machine_is_compatible("ibm,witherspoon"))
		ret = pinctrl_register_mappings(witherspoon_mapping,
				ARRAY_SIZE(witherspoon_mapping));

	if (of_machine_is_compatible("aspeed,ast2500evb"))
		ret = pinctrl_register_mappings(ast2500evb_mapping,
				ARRAY_SIZE(ast2500evb_mapping));
	if (ret)
		printk("Failed to register mappings with pinmux :(\n");

	of_platform_populate(NULL, of_default_bus_match_table, NULL, NULL);
}

#define AST_IO_VA	0xf0000000
#define AST_IO_PA	0x1e600000
#define AST_IO_SZ	0x00200000

#define AST_IO(__pa)	((void __iomem *)(((__pa) & 0x001fffff) | AST_IO_VA))

#define AST_BASE_LPC		0x1E789000 /* LPC Controller */
#define AST_BASE_SPI		0x1E630000 /* SPI Memory Controller */
#define AST_BASE_MAC0		0X1E660000 /* MAC 1 */
#define AST_BASE_SCU		0x1E6E2000 /* System Control Unit (SCU) */
#define AST_BASE_GPIO		0x1E780000 /* GPIO Controller */

static struct map_desc aspeed_io_desc[] __initdata __maybe_unused = {
	{
		.virtual	=  AST_IO_VA,
		.pfn		= __phys_to_pfn(AST_IO_PA),
		.length		= AST_IO_SZ,
		.type		= MT_DEVICE
	},
};

static void __init do_common_setup(void)
{
	/* Enable LPC FWH cycles, Enable LPC to AHB bridge */
	writel(0x00000500, AST_IO(AST_BASE_LPC | 0x80));

	/* Flash controller */
	writel(0x00000003, AST_IO(AST_BASE_SPI | 0x00));
	writel(0x00002404, AST_IO(AST_BASE_SPI | 0x04));

	/* Set UART routing */
	writel(0x00000000, AST_IO(AST_BASE_LPC | 0x9c));

	/* SCU setup
	 *  - GPIOC{4,5,6} are FUNC_MODE{0,1,2}. These nets are
	 *    connected to the Ethernet phy and to ensure the correct
	 *    operation they all need to be in GPIO mode (SCU90[0] = 0)
	 *    and then pulled down
	 */
	writel(0x01C000FF, AST_IO(AST_BASE_SCU | 0x88));
	writel(0xC1C000FF, AST_IO(AST_BASE_SCU | 0x8c));
	writel(0x0000A008, AST_IO(AST_BASE_SCU | 0x90));

	/* Setup scratch registers */
	writel(0x00000042, AST_IO(AST_BASE_LPC | 0x170));
	writel(0x00004000, AST_IO(AST_BASE_LPC | 0x174));
}

static void __init do_barreleye_setup(void)
{
	u32 reg;

	do_common_setup();

	/* Setup PNOR address mapping for 64M flash */
	writel(0x30000C00, AST_IO(AST_BASE_LPC | 0x88));
	writel(0xFC0003FF, AST_IO(AST_BASE_LPC | 0x8C));

	/* GPIO setup */
	writel(0x9E82FCE7, AST_IO(AST_BASE_GPIO | 0x00));
	writel(0x0370E677, AST_IO(AST_BASE_GPIO | 0x04));

	/* SCU setup
	 *  - GPION must be set to GPIO mode (SCU88[0:7] = 0) on
	 *    Barreleye so they can be used to read the PCIe inventory
	 *    status
	 */

	writel(0x01C00000, AST_IO(AST_BASE_SCU | 0x88));

	/*
	 * Do read/modify/write on power gpio to prevent resetting power on
	 * reboot
	 */
	reg = readl(AST_IO(AST_BASE_GPIO | 0x20));
	reg |= 0xCFC8F7FD;
	writel(reg, AST_IO(AST_BASE_GPIO | 0x20));
	writel(0xC738F20A, AST_IO(AST_BASE_GPIO | 0x24));
	writel(0x0031FFAF, AST_IO(AST_BASE_GPIO | 0x80));
}

static void __init do_palmetto_setup(void)
{
	do_common_setup();

	/* Setup PNOR address mapping for 32M flash */
	writel(0x30000E00, AST_IO(AST_BASE_LPC | 0x88));
	writel(0xFE0001FF, AST_IO(AST_BASE_LPC | 0x8C));

	/* GPIO setup */
	writel(0x13008CE7, AST_IO(AST_BASE_GPIO | 0x00));
	writel(0x0370E677, AST_IO(AST_BASE_GPIO | 0x04));
	writel(0xDF48F7FF, AST_IO(AST_BASE_GPIO | 0x20));
	writel(0xC738F202, AST_IO(AST_BASE_GPIO | 0x24));

	/* SCU setup */
	writel(0x01C0007F, AST_IO(AST_BASE_SCU | 0x88));
}

static void __init do_garrison_setup(void)
{
	do_common_setup();

	/* Setup PNOR address mapping for 64M flash */
	writel(0x30000C00, AST_IO(AST_BASE_LPC | 0x88));
	writel(0xFC0003FF, AST_IO(AST_BASE_LPC | 0x8C));

	/* SCU setup */
	writel(0xd7000000, AST_IO(AST_BASE_SCU | 0x88));
}

static void __init do_ast2500evb_setup(void)
{
	unsigned long reg;

	/* Reset AHB bridges */
	writel(0x02, AST_IO(AST_BASE_SCU | 0x04));

	/* Set old MDIO interface */
	reg = readl(AST_IO(AST_BASE_MAC0 | 0x40));
	reg &= ~0x80000000;
	writel(reg, AST_IO(AST_BASE_MAC0 | 0x40));
}

#define SCU_PASSWORD	0x1688A8A8

static void __init aspeed_init_early(void)
{
	// XXX UART stuff to fix to pinmux & co
	writel(0x02010023, AST_IO(AST_BASE_LPC | 0x9c));

	/* Unlock SCU */
	writel(SCU_PASSWORD, AST_IO(AST_BASE_SCU));

	/* Enable UART4 RXD4, TXD4, NRI4, NDCD4, NCTS4 */
	writel(0xcb000000, AST_IO(AST_BASE_SCU | 0x80));

	/* Enable
	 *  - UART1 RXD1, RXD1, NRTS1, NDTR1, NRI1, NDSR1, NDCD1, NCTS1.
	 *  - VGA DDCDAT, DDCCLK, VGAVS, VGAHS.
	 *  - NAND flash FLWP#, FLBUSY#
	 */
	writel(0x00fff0c0, AST_IO(AST_BASE_SCU | 0x84));

	/* Enables all the clocks except D2CLK, USB1.1 Host, USB1.1, LHCLK */
	writel(0x10CC5E80, AST_IO(AST_BASE_SCU | 0x0c));

	/*
	 * Ensure all IPs except GPIO and LPC are reset on watchdog expiry
	 */
	writel(0x001fdff3, AST_IO(AST_BASE_SCU | 0x9c));

	/*
	 * Temporary setup of AST registers until pinmux driver is complete
	 */
	if (of_machine_is_compatible("rackspace,barreleye-bmc"))
		do_barreleye_setup();
	if (of_machine_is_compatible("tyan,palmetto-bmc"))
		do_palmetto_setup();
	if (of_machine_is_compatible("ibm,garrison-bmc"))
		do_garrison_setup();

	if (of_machine_is_compatible("aspeed,ast2500"))
		do_ast2500evb_setup();

}

static void __init aspeed_map_io(void)
{
	iotable_init(aspeed_io_desc, ARRAY_SIZE(aspeed_io_desc));
	debug_ll_io_init();

	printk("SOC Rev: %08x\n", readl(AST_IO(AST_BASE_SCU | 0x7c)));
}

static const char *const aspeed_dt_match[] __initconst = {
		"aspeed,ast2400",
		"aspeed,ast2500",
		NULL,
};

DT_MACHINE_START(aspeed_dt, "ASpeed SoC")
	.init_early	= aspeed_init_early,
	.dt_compat	= aspeed_dt_match,
	.map_io		= aspeed_map_io,
	.init_machine	= aspeed_dt_init,
MACHINE_END
