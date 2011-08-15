/* linux/arch/arm/mach-s5pv210/include/mach/system.h
 *
 * Copyright (c) 2010 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * S5PV210 - system support header
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#ifndef __ASM_ARCH_SYSTEM_H
#define __ASM_ARCH_SYSTEM_H __FILE__

#include <mach/idle.h>
#include <mach/map.h>
#include <linux/clk.h>
#include <linux/io.h>

extern void (*s5pv21x_idle)(void);


static void arch_idle(void)
{
	if(s5pv21x_idle != NULL)
		(s5pv21x_idle)();
	else
		s5pv210_default_idle();
}

static void arch_reset(char mode, const char *cmd)
{
#if 0	
	void __iomem * swreset ;

	swreset = ioremap(S5PC11X_PA_SWRESET,0x400);
	printk(" arch_reset()\n");
	
	if (swreset == NULL) 
	{
                printk("failed to ioremap() region in arch_reset()\n");
                return ;
        }
        
	__raw_writel(0x1,swreset);
	
	mdelay(5000);

#else
	int ret;
	void __iomem * wdt ;
	static struct clk	*wdt_clock; /*  Watch Reset is used */

	printk("MIDAS@%s: +\n", __func__);
	
	wdt_clock = clk_get(NULL, "watchdog");
	if (IS_ERR(wdt_clock)) {
		printk("failed to find watchdog clock source\n");
		ret = PTR_ERR(wdt_clock);
                return ;
	}

	clk_enable(wdt_clock);

	wdt = ioremap(S5PV210_PA_WDT,0x400);
        
	if (wdt == NULL) 
	{
		printk("failed to ioremap() region in arch_reset()\n");
		return ;
	}

	__raw_writel(0x0,wdt); /* Disable WDT */
	__raw_writel(0x8000,wdt+0x4); 
	__raw_writel(0x1,wdt+4); 
	__raw_writel(0x8000,wdt+8); 
	__raw_writel(0x8,wdt+8); 
	__raw_writel(0x8021,wdt); 
	
	mdelay(500);	

	printk("MIDAS@%s: -\n", __func__);
#endif
}

#endif /* __ASM_ARCH_SYSTEM_H */
