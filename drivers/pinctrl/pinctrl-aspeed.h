/*
 * Copyright (C) 2016 IBM Corp.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef PINCTRL_ASPEED
#define PINCTRL_ASPEED

#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/pinconf-generic.h>

/* Challenges in the ASPEED SoC Multifunction Pin Design
 * --------------------------------------------------
 *
 * * Reasonable number of pins (216)
 *
 * * The SoC function enabled on a pin is determined on a priority basis
 *
 * * Using the priority system, pins can provide up to 3 different signals
 *   types
 *
 * * The signal active on a pin is often described by compound logical
 *   expressions involving multiple operators, registers and bits
 *
 *   * The high and low priority functions' bit masks are frequently not the
 *     same
 *     (i.e. cannot just flip a bit to change from high to low), or even in the
 *     same register.
 *
 *   * Not all functions are disabled by unsetting bits, some require setting
 *     bits
 *
 *   * Not all signals can be deactivated as some expressions depend on values
 *     in the hardware strapping register, which is treated as read-only.
 *
 * SoC Multi-function Pin Expression Examples
 * ------------------------------------------
 *
 * Here are some sample mux configurations from the AST2400 datasheet to
 * illustrate the corner cases, roughly in order of least to most corner. In
 * the table, HP and LP are high- and low- priority respectively.
 *
 * D6 is a pin with a single (i.e. aside from GPIO), high priority signal
 * that participates in one function:
 *
 * Ball | Default | HP Signal | HP Expression               | LP Signal | LP Expression | GPIO Name
 * -----+---------+-----------+-----------------------------+-----------+---------------+----------
 *  D6    GPIOA0    MAC1LINK    SCU80[0]=1                                                GPIOA0
 * -----+---------+-----------+-----------------------------+-----------+---------------+----------
 *
 * C5 is a multi-signal pin (both high and low priority signals). Here we touch
 * different registers for the different functions that enable each signal:
 *
 * -----+---------+-----------+-----------------------------+-----------+---------------+----------
 *  C5    GPIOA4    SCL9        SCU90[22]=1                   TIMER5      SCU80[4]=1      GPIOA4
 * -----+---------+-----------+-----------------------------+-----------+---------------+----------
 *
 * E19 is a single-signal pin with two functions that influence the active
 * signal. In this case both bits have the same meaning - enable a dedicated
 * LPC reset pin. However it's not always the case that the bits in the
 * OR-relationship have the same meaning.
 *
 * -----+---------+-----------+-----------------------------+-----------+---------------+----------
 *  E19   GPIOB4    LPCRST#     SCU80[12]=1 | Strap[14]=1                                 GPIOB4
 * -----+---------+-----------+-----------------------------+-----------+---------------+----------
 *
 * For example, pin B19 has a low-priority signal that's enabled by two
 * distinct SoC functions: A specific SIOPBI bit in SCUA4, and an ACPI bit
 * in the STRAP register. The ACPI bit configures signals on pins in addition
 * to B19. Both of the low priority functions as well as the high priority
 * function must be disabled for GPIOF1 to be used.
 *
 * Ball | Default | HP Signal | HP Expression                           | LP Signal | LP Expression                          | GPIO Name
 * -----+---------+-----------+-----------------------------------------+-----------+----------------------------------------+----------
 *  B19   GPIOF1    NDCD4       SCU80[25]=1                               SIOPBI#     SCUA4[12]=1 | Strap[19]=0                GPIOF1
 * -----+---------+-----------+-----------------------------------------+-----------+----------------------------------------+----------
 *
 * For pin E18, the SoC ANDs the expected state of three bits to determine the
 * pin's active signal:
 *
 * * SCU3C[3]: Enable external SOC reset function
 * * SCU80[15]: Enable SPICS1# or EXTRST# function pin
 * * SCU90[31]: Select SPI interface CS# output
 *
 * -----+---------+-----------+-----------------------------------------+-----------+----------------------------------------+----------
 *  E18   GPIOB7    EXTRST#     SCU3C[3]=1 & SCU80[15]=1 & SCU90[31]=0    SPICS1#     SCU3C[3]=1 & SCU80[15]=1 & SCU90[31]=1   GPIOB7
 * -----+---------+-----------+-----------------------------------------+-----------+----------------------------------------+----------
 *
 * (Bits SCU3C[3] and SCU80[15] appear to only be used in the expressions for
 * selecting the signals on pin E18)
 *
 * Pin T5 is a multi-signal pin with a more complex configuration:
 *
 * Ball | Default | HP Signal | HP Expression                | LP Signal | LP Expression | GPIO Name
 * -----+---------+-----------+------------------------------+-----------+---------------+----------
 *  T5    GPIOL1    VPIDE       SCU90[5:4]!=0 & SCU84[17]=1    NDCD1       SCU84[17]=1     GPIOL1
 * -----+---------+-----------+------------------------------+-----------+---------------+----------
 *
 * The high priority signal configuration is best thought of in terms of its
 * exploded form, with reference to the SCU90[5:4] bits:
 *
 * * SCU90[5:4]=00: disable
 * * SCU90[5:4]=01: 18 bits (R6/G6/B6) video mode.
 * * SCU90[5:4]=10: 24 bits (R8/G8/B8) video mode.
 * * SCU90[5:4]=11: 30 bits (R10/G10/B10) video mode.
 *
 * -----+---------+-----------+------------------------------+-----------+---------------+----------
 *  T5    GPIOL1    VPIDE      (SCU90[5:4]=1 & SCU84[17]=1)    NDCD1       SCU84[17]=1     GPIOL1
 *                             | (SCU90[5:4]=2 & SCU84[17]=1)
 *                             | (SCU90[5:4]=3 & SCU84[17]=1)
 * -----+---------+-----------+------------------------------+-----------+---------------+----------
 *
 * For reference the SCU84[17] bit enables the "UART1 NDCD1 or Video VPIDE
 * function pin", where the signal itself is determined by whether SCU94[5:4]
 * is disabled or in one of the 18, 24 or 30bit video modes.
 *
 * Other video-input-related pins require an explicit state in SCU90[5:4]:
 *
 * -----+---------+-----------+------------------------------+-----------+---------------+----------
 *  W1    GPIOL6    VPIB0       SCU90[5:4]=3 & SCU84[22]=1     TXD1        SCU84[22]=1     GPIOL6
 * -----+---------+-----------+------------------------------+-----------+---------------+----------
 *
 * The examples of T5 and W1 are particularly fertile, as they also demonstrate
 * that despite operating as part of the video input bus each signal needs to
 * be enabled individually via it's own SCU84 (in the cases of T5 and W1)
 * register bit. This is a little crazy if the bus doesn't have optional
 * signals, but is used to decent effect with some of the UARTs where not all
 * signals are required. However, this isn't done consistently - UART1 is
 * enabled on a per-pin basis, and by contrast, all signals for UART6 are
 * enabled by a single bit.
 *
 * Ultimately the requirement to control pins like T5 drive the design:
 *
 * * Pins provide signals according to functions activated in the mux
 *   configuration
 *
 * * Pins provide two to three signals in a priority order
 *
 * * For priorities levels defined on a pin, each priority provides one signal
 *
 * * Enabling lower priority signals requires higher priority signals be
 *   disabled
 *
 * * A function represents a set of signals; functions are distinct if their
 *   sets of signals are not equal
 *
 * * Signals participate in one or more functions
 *
 * * A function is described by an expression of one or more function
 *   descriptors, which compare bit values in a register
 *
 * * A function expression is the smallest set of function descriptors whose
 *   comparisons must evaluate 'true' for a function to be enabled.
 *
 * * A function is active if evaluating every function descriptor in the
 *   function expression yields a 'true' result
 *
 * * A signal at a given priority on a given pin is active if any of the
 *   functions in which the signal participates are active, and no higher
 *   priority signal on the pin is active
 *
 * * GPIO is configured per-pin
 *
 * And so:
 *
 * * To disable a signal, any function(s) activating the signal must be
 *   disabled
 *
 * * Each pin must know the expressions of functions in which it participates,
 *   for the purpose of enabling GPIO. This is done by deactivating all
 *   functions that activate higher priority signals on the pin (except in the
 *   case of GPIO[TUV], where GPIO is the high priority function).
 *
 * As a concrete example:
 *
 * * T5 provides three signals types: VPIDE, NDCD1 and GPIO
 *
 * * The VPIDE signal participates in 3 functions: VPI18, VPI24 and VPI30
 *
 * * The NDCD1 signal participates in just its own NDCD1 function
 *
 * * VPIDE is high priority, NDCD1 is low priority, and GPIOL1 is the least
 *   prioritised
 *
 * * The prerequisit for activating the NDCD1 signal is that the VPI18, VPI24
 *   and VPI30 functions all be disabled
 *
 * * Similarly, all of VPI18, VPI24, VPI30 and NDCD1 functions must be disabled
 *   to provide GPIOL6
 *
 * Considerations
 * --------------
 *
 * * If pinctrl allows us to allocate a pin we can configure a function without
 *   concern for the function of already allocated pins, if pin groups are
 *   created with respect to the SoC functions in which they participate (I felt
 *   this was worth mentioning as it didn't seem obvious from the bit/pin
 *   relationships; I've done some network analysis on the problem).
 *
 *     * Conversely, failing to allocate all pins in a group indicates some
 *       bits (as well as pins) required for the group's configuration will
 *       already be in use, likely in a way that's inconsistent with the
 *       requirements of the failed group.
 *
 * * Implement data structures such that we can lean on the compiler to avoid
 *   or catch errors
 *
 *   * Define symbols for structures so they can be referenced by name
 *
 *     * Compiler enforces unique symbol names, detecting copy/paste errors
 *     * Compiler errors on undefined symbol names, detecting typos or missing
 *       information
 *
 *   * Compiler can calculate array sizes for us
 *
 * * Implement macros such that we can lean on the CPP to
 *
 *   * Avoid duplicate specification of information where possible
 *   * Reduce line noise of type declaration and assignment
 */

 /**
  * A function descriptor, which describes the register, bits and the
  * enable/disable values that should be compared or written.
  *
  * @reg: The register offset from base in bytes
  * @mask: The mask to apply to the register. The lowest set bit of the mask is
  *        used to derive the shift value.
  * @enable: The value that enables the function. Value should be in the LSBs,
  *          not at the position of the mask.
  * @disable: The value that disables the function. Value should be in the
  *           LSBs, not at the position of the mask.
  */
struct aspeed_func_desc {
	unsigned reg;
	u32 mask;
	u32 enable;
	u32 disable;
};

/**
 * Describes a function expression. The expression is evaluated by ANDing the
 * evaluation of the descriptors.
 *
 * @name: The function name
 * @ndescs: The number of function descriptors in the expression
 * @descs: Pointer to an array of function descriptors that comprise the
 *         function expression
 */
struct aspeed_func_expr {
	const char *name;
	int ndescs;
	const struct aspeed_func_desc *descs;
};

/**
 * A struct capturing the function expressions enabling signals at each
 * priority for a given pin. A high or low priority signal configuration is
 * evaluated by ORing the evaluation of the function expressions in the
 * respective priority's list.
 *
 * @high: A NULL-terminated list of function expressions that enable the
 *        high-priority signal
 * @low: A NULL-terminated list of function expressions that enable the
 *       low-priority signal
 * @other: The name of the "other" function, enabled by disabling the high and
 *         low priority signals
 */
struct aspeed_pin_prio {
	const struct aspeed_func_expr **high;
	const struct aspeed_func_expr **low;
	const char *other;
};

/* Macro hell */

/**
 * Short-hand macro describing a mux function enabled by the state of one bit.
 * The disable value is derived.
 *
 * @reg: The function's associated register, offset from base
 * @idx: The function's bit index in the register
 * @val: The value (0 or 1) that enables the function
 */
#define FUNC_DESC_BIT(reg, idx, val) \
	{ reg, BIT_MASK(idx), val, ((val + 1) & 1) }

/**
 * A further short-hand macro describing a mux function enabled with a set bit.
 *
 * @reg: The function's associated register, offset from base
 * @idx: The function's bit index in the register
 */
#define FUNC_DESC_SET(reg, idx) FUNC_DESC_BIT(reg, idx, 1)

#define FUNC_DESC_LIST_SYM(func) func_descs_ ## func
#define FUNC_DESC_LIST_DECL(func, ...) \
	static const struct aspeed_func_desc FUNC_DESC_LIST_SYM(func)[] = \
		{ __VA_ARGS__ }

#define FUNC_EXPR_SYM(func) func_expr_ ## func
#define FUNC_EXPR_DECL_(func) \
	static const struct aspeed_func_expr FUNC_EXPR_SYM(func) = \
	{ \
		.name = #func, \
		.ndescs = ARRAY_SIZE(FUNC_DESC_LIST_SYM(func)), \
		.descs = &(FUNC_DESC_LIST_SYM(func))[0], \
	}

/**
 * Declare a function expression.
 *
 * @func: A macro symbol name for the function (is subjected to stringification
 *        and token pasting)
 * @...: Function descriptors that define the function expression
 *
 * For example, the following declares the ACPI function:
 *
 *     FUNC_EXPR_DECL(ACPI, FUNC_DESC_BIT(STRAP, 19, 0));
 *
 * And (with a few defines left out for brevity) the following declares the
 * 8-signal ROM function:
 *
 *     FUNC_EXPR_DECL(ROM8,
 *         VPOOFF0_DESC,
 *         ROMEN_0_DESC,
 *         ROMEN_1_DESC,
 *         ROMEN_2_DESC);
 */
#define FUNC_EXPR_DECL(func, ...) \
	FUNC_DESC_LIST_DECL(func, __VA_ARGS__); \
	FUNC_EXPR_DECL_(func)

/**
 * Declare a pointer to a function expression
 *
 * @func: The macro symbol name for the function (subjected to token pasting)
 */
#define FUNC_EXPR_PTR(func) (&FUNC_EXPR_SYM(func))

#define FUNC_EXPR_LIST_SYM(func) func_exprs_ ## func

/**
 * Declare a function list for reference in a struct pin_prio.
 *
 * @func: A macro symbol name for the function (is subjected to token pasting)
 * @...: Function expression structure pointers (use FUNC_EXPR_POINTER())
 */
#define FUNC_EXPR_LIST_DECL(func, ...) \
	static const struct aspeed_func_expr *FUNC_EXPR_LIST_SYM(func)[] = \
		{ __VA_ARGS__ }

/**
 * A short-hand macro for declaring an expression list consisting of a single
 * function.
 *
 * @func: A macro symbol name for the function (is subjected to token pasting)
 */
#define FUNC_EXPR_LIST_DECL_SINGLE(func) \
	FUNC_EXPR_LIST_DECL(func, FUNC_EXPR_PTR(func), NULL)

#define FUNC_EXPR_LIST_PTR(func) (&FUNC_EXPR_LIST_SYM(func)[0])

/**
 * A short-hand macro for declaring a function expression and an expression
 * list with a single function.
 *
 * @func: A macro symbol name for the function (is subjected to token pasting)
 * @...: Function descriptors that define the function expression
 */
#define FUNC_EXPR_DECL_SINGLE(func, ...) \
	FUNC_DESC_LIST_DECL(func, __VA_ARGS__); \
	FUNC_EXPR_DECL_(func); \
	FUNC_EXPR_LIST_DECL(func, FUNC_EXPR_PTR(func), NULL)

#define PIN_SYM(pin) pin_ ## pin

#define MS_PIN_DECL_(pin, other, high, low) \
	static const struct aspeed_pin_prio PIN_SYM(pin) = { high, low, #other }

/**
 * Declare a multi-signal pin
 *
 * @pin: The pin number
 * @other: Macro name for "other" functionality (subjected to stringification)
 * @high: Macro name for the high signal functions
 * @low: Macro name for the low signal functions
 *
 * For example:
 *
 *    FUNC_EXPR_DECL_SINGLE(SD1, FUNC_DESC_SET(SCU90, 0));
 *    FUNC_EXPR_DECL_SINGLE(I2C10, FUNC_DESC_SET(SCU90, 23));
 *
 *    #define C4 16
 *    MS_PIN_DECL(C4, GPIOC0, SD1, I2C10);
 *    #define B3 17
 *    MS_PIN_DECL(B3, GPIOC1, SD1, I2C10);
 */
#define MS_PIN_DECL(pin, other, high, low) \
	MS_PIN_DECL_(pin, other, \
			FUNC_EXPR_LIST_PTR(high), \
			FUNC_EXPR_LIST_PTR(low))

#define PIN_GROUP_SYM(func) pins_ ## func
#define FUNC_GROUP_SYM(func) groups_ ## func
#define FUNC_GROUP_DECL(func, ...) \
	static const int PIN_GROUP_SYM(func)[] = { __VA_ARGS__ }; \
	static const char *const FUNC_GROUP_SYM(func)[] = { #func }

/**
 * Declare a single signal pin
 *
 * @pin: The pin number
 * @other: Macro name for "other" functionality (subjected to stringification)
 * @func: Macro name for the function
 *
 * For example:
 *
 *    FUNC_EXPR_DECL_SINGLE(I2C5, FUNC_DESC_SET(SCU90, 18));
 *
 *    #define E3 80
 *    SS_PIN_DECL(E3, GPIOK0, I2C5);
 *
 *    #define D2 81
 *    SS_PIN_DECL(D2, GPIOK1, I2C5);
 */
#define SS_PIN_DECL(pin, other, func) \
	MS_PIN_DECL_(pin, other, FUNC_EXPR_LIST_PTR(func), NULL)

/**
 * Single signal, single function pin declaration
 *
 * @pin: The pin number
 * @other: Macro name for "other" functionality (subjected to stringification)
 * @func: Macro name for the function
 * @...: Function descriptors that define the function expression
 *
 * For example:
 *
 *    SSSF_PIN_DECL(A4, GPIOA2, TIMER3, FUNC_DESC_SET(SCU80, 2));
 */
#define SSSF_PIN_DECL(pin, other, func, ...) \
	FUNC_EXPR_DECL(func, __VA_ARGS__); \
	FUNC_EXPR_LIST_DECL(func, FUNC_EXPR_PTR(func), NULL); \
	MS_PIN_DECL_(pin, other, FUNC_EXPR_LIST_PTR(func), NULL); \
	FUNC_GROUP_DECL(func, pin)

struct aspeed_pinctrl_data {
	void __iomem *reg_base;

	const struct pinctrl_pin_desc *pins;
	const unsigned npins;

	const struct aspeed_pin_group *groups;
	const unsigned ngroups;

	const struct aspeed_pin_function *functions;
	const unsigned nfunctions;
};

#define ASPEED_PINCTRL_PIN(name_) \
	[name_] = { \
		.number = name_, \
		.name = #name_, \
		.drv_data = (void *) &(PIN_SYM(name_)) \
	}

struct aspeed_pin_group {
	const char *name;
	const unsigned int *pins;
	const unsigned npins;
};

#define ASPEED_PINCTRL_GROUP(name_) { \
	.name = #name_, \
	.pins = &(PIN_GROUP_SYM(name_))[0], \
	.npins = ARRAY_SIZE(PIN_GROUP_SYM(name_)), \
}

struct aspeed_pin_function {
	const char *name;
	const char *const *groups;
	unsigned ngroups;
	const struct aspeed_func_expr *expr;
};

#define ASPEED_PINCTRL_FUNC(name_, ...) { \
	.name = #name_, \
	.groups = &FUNC_GROUP_SYM(name_)[0], \
	.ngroups = ARRAY_SIZE(FUNC_GROUP_SYM(name_)), \
	.expr = FUNC_EXPR_PTR(name_), \
}

int aspeed_pinctrl_get_groups_count(struct pinctrl_dev *pctldev);
const char *aspeed_pinctrl_get_group_name(struct pinctrl_dev *pctldev,
		unsigned group);
int aspeed_pinctrl_get_group_pins(struct pinctrl_dev *pctldev,
		unsigned group, const unsigned **pins, unsigned *npins);
void aspeed_pinctrl_pin_dbg_show(struct pinctrl_dev *pctldev,
		struct seq_file *s, unsigned offset);
int aspeed_pinmux_get_fn_count(struct pinctrl_dev *pctldev);
const char *aspeed_pinmux_get_fn_name(struct pinctrl_dev *pctldev,
		unsigned function);
int aspeed_pinmux_get_fn_groups(struct pinctrl_dev *pctldev,
		unsigned function, const char * const **groups,
		unsigned * const num_groups);
void aspeed_func_desc_print_val(const struct aspeed_func_desc *desc, u32 rv);
bool aspeed_func_desc_eval(const struct aspeed_func_desc *desc,
		void __iomem *base);
bool aspeed_func_expr_eval(const struct aspeed_func_expr *expr,
		void __iomem *base);
bool aspeed_func_expr_enable(const struct aspeed_func_expr *expr,
		void __iomem *base);
bool aspeed_func_expr_disable(const struct aspeed_func_expr *expr,
		void __iomem *base);
bool aspeed_maybe_disable(const struct aspeed_func_expr **expr,
		void __iomem *base);
bool aspeed_sig_in_exprs(const struct aspeed_func_expr **exprs,
		const struct aspeed_func_expr *sig);
int aspeed_pinmux_set_mux(struct pinctrl_dev *pctldev, unsigned function,
		unsigned group);
int aspeed_gpio_request_enable(struct pinctrl_dev *pctldev,
		struct pinctrl_gpio_range *range,
		unsigned offset);
int aspeed_pin_config_get(struct pinctrl_dev *pctldev,
		unsigned pin,
		unsigned long *config);
int aspeed_pin_config_set(struct pinctrl_dev *pctldev,
		unsigned pin,
		unsigned long *configs,
		unsigned num_configs);

/* FIXME: This is ast2400 specific */
#define STRAP 0x70

#endif /* PINCTRL_ASPEED */
