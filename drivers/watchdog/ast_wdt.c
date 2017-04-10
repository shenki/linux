/********************************************************************************
* File Name     : ast_wdt
* 
* Copyright (C) 2012-2020  ASPEED Technology Inc.
* This program is free software; you can redistribute it and/or modify 
* it under the terms of the GNU General Public License as published by the Free Software Foundation; 
* either version 2 of the License, or (at your option) any later version. 
* This program is distributed in the hope that it will be useful,  but WITHOUT ANY WARRANTY; 
* without even the implied warranty of MERCHANTABILITY or 
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details. 
* You should have received a copy of the GNU General Public License 
* along with this program; if not, write to the Free Software 
* Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA 
********************************************************************************/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/poll.h>
#include <linux/interrupt.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/string.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/miscdevice.h>
#include <linux/watchdog.h>
#include <linux/fs.h>
#include <linux/notifier.h>
#include <linux/reboot.h>
#include <linux/init.h>
#include <linux/semaphore.h>
#include <linux/of.h>
#include <linux/of_device.h>


#include <linux/platform_device.h>
#include <asm/io.h>
#include <mach/hardware.h>

#ifdef CONFIG_COLDFIRE
#include <asm/arch/irqs.h>
#include <asm/arch/ast_wdt.h>
#include <asm/arch/platform.h>
#else
#include <mach/irqs.h>
#include <mach/ast_wdt.h>
#include <mach/platform.h>
#endif

#define TICKS_PER_uSEC                  1 

typedef unsigned char bool_T;

#ifdef TRUE
#undef TRUE
#endif

#ifdef FALSE
#undef FALSE
#endif

#define TRUE	1
#define FALSE	0

void __iomem *wdt_reg_base;


#define WDT_CntSts              	(0x00)
#define WDT_Reload              	(0x04)
#define WDT_Restart            	(0x08)
#define WDT_Ctrl                	(0x0C)
#define WDT_TimeOut             	(0x10)
#define WDT_Clr                 	(0x14)
#define WDT_RstWd               	(0x18)

#define WDT_CLK_SRC_EXT		0
#define WDT_CLK_SRC_PCLK	1

//Global Variables
#define WD_TIMO 6			/* Default heartbeat = 6 seconds */

static int heartbeat = WD_TIMO;
module_param(heartbeat, int, 0);
MODULE_PARM_DESC(heartbeat, "Watchdog heartbeat in seconds. (0<heartbeat<65536, default=" __MODULE_STRING(WD_TIMO) ")");

static int nowayout = WATCHDOG_NOWAYOUT;
module_param(nowayout, int, 0);
MODULE_PARM_DESC(nowayout, "Watchdog cannot be stopped once started (default=CONFIG_WATCHDOG_NOWAYOUT)");

static unsigned long wdt_is_open;
static char expect_close;

static inline void ast_wdt_w32(unsigned reg, u32 val)
{
	writel(val, wdt_reg_base + reg);
}

static inline u32 ast_wdt_r32(unsigned reg)
{
	return readl(wdt_reg_base + reg);
}

//Function Declaration
static irqreturn_t wdt_isr(int irq, void *devid)
{
	/* clear timeout */
	ast_wdt_w32(WDT_Clr, 1);

	return (IRQ_HANDLED);
}

void wdt_disable(void)
{
    register unsigned int regVal;
                                                                                     
    /* reset WDT_Ctrl[0] as 0 */
    regVal = ast_wdt_r32(WDT_Ctrl);
    regVal &= 0xFFFFFFFE;
    ast_wdt_w32(WDT_Ctrl, regVal);
}

void wdt_sel_clk_src(unsigned char sourceClk)
{
    register unsigned int regVal;
                                                                                     
    regVal = ast_wdt_r32(WDT_Ctrl);
    if (sourceClk == WDT_CLK_SRC_PCLK)
    {
        /* reset WDT_Ctrl[4] as 0 */
        regVal &= 0xFFFFFFEF;
    }
    else
    {
        /* set WDT_Ctrl[4] as 1 */
        regVal |= 0x00000010;
    }
    ast_wdt_w32(WDT_Ctrl, regVal);
}

void wdt_set_timeout_action(bool_T bResetOut, bool_T bIntrSys, bool_T bResetSys)
{
	register unsigned int regVal;

	regVal = ast_wdt_r32(WDT_Ctrl);

	if (bResetOut)
	{
		/* set WDT_Ctrl[3] = 1 */
		regVal |= 0x00000008;
	}
	else
	{
		/* reset WDT_Ctrl[3] = 0 */
		regVal &= 0xFFFFFFF7;
	}

	if (bIntrSys)
	{
		/* set WDT_Ctrl[2] = 1 */
		regVal |= 0x00000004;
	}
	else
	{
		/* reset WDT_Ctrl[2] = 0 */
		regVal &= 0xFFFFFFFB;
	}

	if (bResetSys)
	{
		/* set WDT_Ctrl[1] = 1 */
		regVal |= 0x00000002;
	}
	else
	{
		/* reset WDT_Ctrl[1] = 0 */
		regVal &= 0xFFFFFFFD;
	}

	ast_wdt_w32(WDT_Ctrl, regVal);
}

void wdt_enable(void)
{
	register unsigned int regVal;

	/* set WDT_Ctrl[0] as 1 */
	regVal = ast_wdt_r32(WDT_Ctrl);
	regVal |= 1;
	ast_wdt_w32(WDT_Ctrl, regVal);
}

void wdt_restart_new(unsigned int nPeriod, int sourceClk, bool_T bResetOut, bool_T bIntrSys, bool_T bResetSys, bool_T bUpdated)
{
	wdt_disable();

	ast_wdt_w32(WDT_Reload, nPeriod);

	wdt_sel_clk_src(sourceClk);

	wdt_set_timeout_action(bResetOut, bIntrSys, bResetSys);

	ast_wdt_w32(WDT_Restart, 0x4755);	/* reload! */

	if (!bUpdated)
  	  wdt_enable();
}

void wdt_restart(void)
{
	wdt_disable();
	ast_wdt_w32(WDT_Restart, 0x4755);	/* reload! */
	wdt_enable();
}


/**
 *	wdt_set_heartbeat:
 *	@t:		the new heartbeat value that needs to be set.
 *
 *	Set a new heartbeat value for the watchdog device. If the heartbeat value is
 *	incorrect we keep the old value and return -EINVAL. If successfull we
 *	return 0.
 */
static int wdt_set_heartbeat(int t)
{
  if ((t < 1) || (t > 1000))
      return -EINVAL;
      
  heartbeat=t;
      
  wdt_restart_new(TICKS_PER_uSEC*1000000*t, WDT_CLK_SRC_EXT, FALSE, TRUE, FALSE, FALSE);
  return 0;
}

/*
   Kernel Interfaces
*/

/**
 *	ast_wdt_write: 
 *	@file: file handle to the watchdog
 *	@buf: buffer to write (unused as data does not matter here
 *	@count: count of bytes
 *	@ppos: pointer to the position to write. No seeks allowed
 *
 *	A write to a watchdog device is defined as a keepalive signal. Any
 *	write of data will do, as we we don't define content meaning.
 */
         
 static ssize_t ast_wdt_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
 {
   if(count) 
   {
     if (!nowayout)
     {
        size_t i;
             
        /* In case it was set long ago */
        expect_close = 0;
                         
        for (i = 0; i != count; i++) 
        {
          char c;
          if (get_user(c, buf + i))
            return -EFAULT;
          if (c == 'V')
            expect_close = 42;
        }
      }
      wdt_restart();
   }
   return count; 
 }
 
/**
 *	ast_wdt_ioctl:
 *	@inode: inode of the device
 *	@file: file handle to the device
 *	@cmd: watchdog command
 *	@arg: argument pointer
 * *	The watchdog API defines a common set of functions for all watchdogs
 *	according to their available features. We only actually usefully support
 *	querying capabilities and current status.
 */
static long ast_wdt_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
  void __user *argp = (void __user *)arg;
  int __user *p = argp;
  int new_heartbeat;
      
  static struct watchdog_info ident = 
  {
    .options 		= WDIOF_SETTIMEOUT|
                          WDIOF_MAGICCLOSE|
                          WDIOF_KEEPALIVEPING,
    .firmware_version 	= 1,
    .identity 		= "AST WDT",
  };
  
  switch(cmd)
  {
    default:
      return -ENOIOCTLCMD;
    case WDIOC_GETSUPPORT:
      return copy_to_user(argp, &ident, sizeof(ident))?-EFAULT:0;                      
    case WDIOC_GETSTATUS:
    case WDIOC_GETBOOTSTATUS:
      return put_user(0, p);
    case WDIOC_KEEPALIVE:
      wdt_restart();
      return 0;
    case WDIOC_SETTIMEOUT:
      if (get_user(new_heartbeat, p))
        return -EFAULT;

      if (wdt_set_heartbeat(new_heartbeat))
        return -EINVAL;

      /* Fall */
    case WDIOC_GETTIMEOUT:
      return put_user(heartbeat, p);      
  }
}
/**
*	ast_wdt_open:
*	@inode: inode of device
*	@file: file handle to device
*
*	The watchdog device has been opened. The watchdog device is single
*	open and on opening we load the counters. Counter zero is a 100Hz
*	cascade, into counter 1 which downcounts to reboot. When the counter
*	triggers counter 2 downcounts the length of the reset pulse which
*	set set to be as long as possible.
*/
          
static int ast_wdt_open(struct inode *inode, struct file *file)
{
  if(test_and_set_bit(0, &wdt_is_open))
      return -EBUSY;
  /*
   *	Activate
   */

  wdt_restart();
  return nonseekable_open(inode, file);
} 

/**
*	ast_wdt_release:
*	@inode: inode to board
*	@file: file handle to board
*
*	The watchdog has a configurable API. There is a religious dispute
*	between people who want their watchdog to be able to shut down and
*	those who want to be sure if the watchdog manager dies the machine
*	reboots. In the former case we disable the counters, in the latter
*	case you have to open it again very soon.
*/
          
static int ast_wdt_release(struct inode *inode, struct file *file)
{
  if (expect_close == 42 || !nowayout) 
  {
      wdt_disable();
      clear_bit(0, &wdt_is_open);
  } 
  else 
  {
      printk(KERN_CRIT "wdt: WDT device closed unexpectedly.  WDT will not stop!\n");
      wdt_restart();
  }
  expect_close = 0;
  return 0;
}

/**
*	notify_sys:
*	@this: our notifier block
*	@code: the event being reported
*	@unused: unused
*
*	Our notifier is called on system shutdowns. We want to turn the card
*	off at reboot otherwise the machine will reboot again during memory
*	test or worse yet during the following fsck. This would suck, in fact
*	trust me - if it happens it does suck.
*/
          
static int ast_wdt_notify_sys(struct notifier_block *this, unsigned long code, void *unused)
{
   if(code==SYS_DOWN || code==SYS_HALT) 
   {
     /* Turn the WDT off */
     wdt_disable();
   }
   return NOTIFY_DONE;
}

extern void ast_soc_wdt_reset(enum reboot_mode mode, const char *cmd)	
{
	writel(0x10 , (void *)(wdt_reg_base+0x04));
	writel(0x4755, (void *)(wdt_reg_base+0x08));
	writel(0x3, (void *)(wdt_reg_base+0x0c));
}

EXPORT_SYMBOL(ast_soc_wdt_reset);

static struct file_operations ast_wdt_fops = 
{
	.owner	= THIS_MODULE,
	.llseek	= no_llseek,
	.write	= ast_wdt_write,
	.unlocked_ioctl	= ast_wdt_ioctl,
	.open		= ast_wdt_open,
	.release	= ast_wdt_release,
};

static struct miscdevice ast_wdt_miscdev = 
{
   .minor	= WATCHDOG_MINOR,
   .name	= "watchdog",
   .fops	= &ast_wdt_fops,
};
     
static struct notifier_block ast_wdt_notifier = 
{
   .notifier_call=ast_wdt_notify_sys,
};

static int ast_wdt_probe(struct platform_device *pdev)
{
	int ret;  

	wdt_reg_base = ioremap(AST_WDT_BASE, SZ_256);
	if (!wdt_reg_base) {
		printk("ast_i2c_irq_init ERROR \n");
		return -1;
	}

	wdt_disable();
	wdt_sel_clk_src(WDT_CLK_SRC_EXT);
	wdt_set_timeout_action(FALSE, FALSE, FALSE);

	/* register ISR */
	if(request_irq(IRQ_WDT, wdt_isr, 0, "WDT", NULL)) {
		printk("unable to register interrupt INT_WDT = %d\n", IRQ_WDT);
		return (-1);
	} else
		printk("success to register interrupt for INT_WDT (%d)\n", IRQ_WDT);

	ret = register_reboot_notifier(&ast_wdt_notifier);
	if(ret) {
		printk(KERN_ERR "wdt: cannot register reboot notifier (err=%d)\n", ret);
		free_irq(IRQ_WDT, NULL);
		return ret;
	}

	ret = misc_register(&ast_wdt_miscdev);
	if (ret) 
	{
	  printk(KERN_ERR "wdt: cannot register miscdev on minor=%d (err=%d)\n",WATCHDOG_MINOR, ret);
	  unregister_reboot_notifier(&ast_wdt_notifier);   
	  return ret;
	}

	/* interrupt the system while WDT timeout */
	wdt_restart_new(TICKS_PER_uSEC*1000000*heartbeat, WDT_CLK_SRC_EXT, FALSE, TRUE, FALSE, TRUE);

	printk(KERN_INFO "AST WDT is installed.(irq = %d, heartbeat = %d secs, nowayout = %d)\n",IRQ_WDT,heartbeat,nowayout);

	return (0);
}

static int ast_wdt_remove(struct platform_device *dev)
{
	wdt_disable();
	misc_deregister(&ast_wdt_miscdev);
	disable_irq(IRQ_WDT);
	free_irq(IRQ_WDT, NULL);		
	return 0;
}

static const struct of_device_id ast_wdt_dt_ids[] = {
	{ .compatible = "aspeed,ast-wdt" },
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, ast_wdt_dt_ids);

static struct platform_driver ast_wdt_driver = {
	.probe 		= ast_wdt_probe,
	.remove		= ast_wdt_remove,
#if 0        
	.suspend		= ast_wdt_suspend,
	.resume 		= ast_wdt_resume,
#endif        
		
	.driver		= {
		.name	= "ast-wdt",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(ast_wdt_dt_ids),
	},
};

static struct platform_device *ast_wdt_device;

static int __init ast_wdt_init(void)
{
	int ret;

	static const struct resource ast_wdt0_resource[] = {
		[0] = {
			.start	= AST_WDT_BASE,
			.end		= AST_WDT_BASE + (SZ_16*2) - 1,
			.flags	= IORESOURCE_MEM,
		},
		[1] = {
			.start 	= IRQ_WDT,
			.end 	= IRQ_WDT,
			.flags 	= IORESOURCE_IRQ,
		},	
	};

#if 0
	static struct resource ast_wdt1_resource[] = {
		[0] = {
			.start	= AST_WDT_BASE + (SZ_16*2),
			.end	= AST_WDT_BASE + (SZ_16*4) - 1,
			.flags	= IORESOURCE_MEM,
		},
		[1] = {
				.start = IRQ_WDT,
				.end = IRQ_WDT,
				.flags = IORESOURCE_IRQ,
		},	
	};
#endif

	ret = platform_driver_register(&ast_wdt_driver);

	if (!ret) {
		ast_wdt_device = platform_device_register_simple("ast-wdt", 0,
								ast_wdt0_resource, ARRAY_SIZE(ast_wdt0_resource));
		if (IS_ERR(ast_wdt_device)) {
			platform_driver_unregister(&ast_wdt_driver);
			ret = PTR_ERR(ast_wdt_device);
		}
	}

	return ret;
}

static void __exit ast_wdt_exit(void)
{
	platform_device_unregister(ast_wdt_device);
	platform_driver_unregister(&ast_wdt_driver);
}

module_init(ast_wdt_init);
module_exit(ast_wdt_exit);

MODULE_AUTHOR("Ryan Chen <ryan_chen@aspeedtech.com>");
MODULE_DESCRIPTION("Watchdog driver for AST processors");
MODULE_LICENSE("GPL");
