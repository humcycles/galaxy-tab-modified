/*
 * Copyright (C) 2008 Google, Inc.
 * Author: Nick Pelly <npelly@google.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

/* Control bluetooth power for Jupiter platform */

#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/rfkill.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/wakelock.h>
#include <linux/irq.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <mach/gpio.h>
#include <mach/hardware.h>
#include <plat/gpio-cfg.h>
#include <plat/irqs.h>
#include <linux/io.h>
#include <mach/dma.h>
#include <plat/regs-otg.h>
#include <mach/regs-gpio.h>
#include <plat/gpio-cfg.h>

#ifdef CONFIG_S5P_LPAUDIO
#include <mach/cpuidle.h>
#endif /* CONFIG_S5P_LPAUDIO */
#define BT_SLEEP_ENABLER

#define IRQ_BT_HOST_WAKE      IRQ_EINT(21)

static struct wake_lock rfkill_wake_lock;
#ifdef BT_SLEEP_ENABLER
static struct wake_lock bt_wake_lock;
#endif 

#ifndef	GPIO_LEVEL_LOW
#define GPIO_LEVEL_LOW		0
#define GPIO_LEVEL_HIGH		1
#endif

extern unsigned int HWREV;

void rfkill_switch_all(enum rfkill_type type, enum rfkill_user_states state);

extern void s3c_setup_uart_cfg_gpio(unsigned char port);
//extern void s3c_reset_uart_cfg_gpio(unsigned char port);

static struct rfkill *bt_rfk;
static const char bt_name[] = "bcm4329";

static bool lpaudio_lock = 0;
static bool bt_init_complete = 0;

static unsigned int bt_gpio_table[][4] = {
	{GPIO_BT_nRST, 1, GPIO_LEVEL_LOW, S3C_GPIO_PULL_NONE},
	{GPIO_BT_HOST_WAKE, GPIO_BT_HOST_WAKE_AF, GPIO_LEVEL_NONE, S3C_GPIO_PULL_DOWN},
	{GPIO_BT_WAKE, 1, GPIO_LEVEL_LOW, S3C_GPIO_PULL_DOWN},
};

static unsigned int bt_gpio_table_rev06[][4] = {
	{GPIO_BT_nRST, 1, GPIO_LEVEL_LOW, S3C_GPIO_PULL_NONE},
	{GPIO_BT_HOST_WAKE, GPIO_BT_HOST_WAKE_AF, GPIO_LEVEL_NONE, S3C_GPIO_PULL_DOWN},
	{GPIO_BT_WAKE_REV06, 1, GPIO_LEVEL_LOW, S3C_GPIO_PULL_DOWN},
};

/*
static unsigned int bt_uart_on_table[][4] = {
	{GPIO_BT_RXD, GPIO_BT_RXD_AF, GPIO_LEVEL_NONE, S3C_GPIO_PULL_NONE},
	{GPIO_BT_TXD, GPIO_BT_TXD_AF, GPIO_LEVEL_NONE, S3C_GPIO_PULL_NONE},
	{GPIO_BT_CTS, GPIO_BT_CTS_AF, GPIO_LEVEL_NONE, S3C_GPIO_PULL_NONE},
	{GPIO_BT_RTS, GPIO_BT_RTS_AF, GPIO_LEVEL_NONE, S3C_GPIO_PULL_NONE},
};
*/

static unsigned int bt_uart_off_table[][4] = {
	{GPIO_BT_RXD, 1, GPIO_LEVEL_NONE, S3C_GPIO_PULL_NONE},
	{GPIO_BT_TXD, 1, GPIO_LEVEL_NONE, S3C_GPIO_PULL_NONE},
	{GPIO_BT_CTS, 1, GPIO_LEVEL_NONE, S3C_GPIO_PULL_NONE},
	{GPIO_BT_RTS, 1, GPIO_LEVEL_NONE, S3C_GPIO_PULL_NONE},
};

void bt_config_gpio_table(int array_size, int (*gpio_table)[4])
{
	u32 i, gpio;

	for (i = 0; i < array_size; i++) {
		gpio = gpio_table[i][0];
		s3c_gpio_cfgpin(gpio, S3C_GPIO_SFN(gpio_table[i][1]));
		s3c_gpio_setpull(gpio, gpio_table[i][3]);
		if (gpio_table[i][2] != GPIO_LEVEL_NONE)
			gpio_set_value(gpio, gpio_table[i][2]);
	}
}

void bt_uart_rts_ctrl(int flag)
{
	unsigned int gpa0_bt_con;
	unsigned int gpa0_bt_pud;
	unsigned int gpa0_bt_dat;
	
	if(flag)
	{
		// BT RTS Set to HIGH
		s3c_gpio_cfgpin(S5PV210_GPA0(3), S3C_GPIO_OUTPUT);
		s3c_gpio_setpull(S5PV210_GPA0(3), S3C_GPIO_PULL_NONE);
		gpa0_bt_dat = __raw_readl(S5PV210_GPA0DAT);
		gpa0_bt_dat |= (1 << 3);
		__raw_writel(gpa0_bt_dat, S5PV210_GPA0DAT);

		gpa0_bt_con = __raw_readl(S5PV210_GPA0CONPDN);
		gpa0_bt_con |= (1 << 6);
		gpa0_bt_con &= ~(1 << 7);
		__raw_writel(gpa0_bt_con, S5PV210_GPA0CONPDN);

		gpa0_bt_pud = __raw_readl(S5PV210_GPA0PUDPDN);
		gpa0_bt_pud &= ~(1 << 7 | 1 << 6);
		__raw_writel(gpa0_bt_pud, S5PV210_GPA0PUDPDN);
	}
	else
	{
		// BT RTS Set to LOW
		s3c_gpio_cfgpin(S5PV210_GPA0(3), S3C_GPIO_OUTPUT);
		gpa0_bt_dat = __raw_readl(S5PV210_GPA0DAT);
		gpa0_bt_dat &= ~(1 << 3);
		__raw_writel(gpa0_bt_dat, S5PV210_GPA0DAT);

		s3c_gpio_cfgpin(S5PV210_GPA0(3), S3C_GPIO_SFN(2));
		s3c_gpio_setpull(S5PV210_GPA0(3), S3C_GPIO_PULL_NONE);
	}
}
EXPORT_SYMBOL(bt_uart_rts_ctrl);

static int bluetooth_set_power(void *data, enum rfkill_user_states state)
{
	int ret = 0;
	unsigned int gpio_wlan_rst;
	
	if (HWREV >= 12)
		gpio_wlan_rst = GPIO_WLAN_nRST_REV06;
	else
		gpio_wlan_rst = GPIO_WLAN_nRST;
	
	switch (state) {

		case RFKILL_USER_STATE_UNBLOCKED:
			printk(KERN_DEBUG "[BT] Device Powering ON \n");
			s3c_setup_uart_cfg_gpio(0);

			if (gpio_is_valid(GPIO_WLAN_BT_EN))
			{
				ret = gpio_request(GPIO_WLAN_BT_EN, "GPB");
				if (ret < 0) {
					printk(KERN_ERR "[BT] Failed to request GPIO_WLAN_BT_EN!\n");
					return ret;
				}
				gpio_direction_output(GPIO_WLAN_BT_EN, GPIO_LEVEL_HIGH);
			}

			if (gpio_is_valid(GPIO_BT_nRST))
			{
				ret = gpio_request(GPIO_BT_nRST, "GPB");
				if (ret < 0) {
					gpio_free(GPIO_WLAN_BT_EN);
					printk(KERN_ERR "[BT] Failed to request GPIO_BT_nRST\n");
					return ret;			
				}
				gpio_direction_output(GPIO_BT_nRST, GPIO_LEVEL_LOW);
			}
			printk(KERN_DEBUG "[BT] GPIO_BT_nRST = %d\n", gpio_get_value(GPIO_BT_nRST));

			/* Set GPIO_BT_WLAN_REG_ON high */ 
			s3c_gpio_setpull(GPIO_WLAN_BT_EN, S3C_GPIO_PULL_NONE);
			gpio_set_value(GPIO_WLAN_BT_EN, GPIO_LEVEL_HIGH);

			s3c_gpio_slp_cfgpin(GPIO_WLAN_BT_EN, S3C_GPIO_SLP_OUT1);  
			s3c_gpio_slp_setpull_updown(GPIO_WLAN_BT_EN, S3C_GPIO_PULL_NONE);

			printk( "[BT] GPIO_WLAN_BT_EN = %d\n", gpio_get_value(GPIO_WLAN_BT_EN));		
			/*FIXME sleep should be enabled disabled since the device is not booting 
			 * 			if its enabled*/
			msleep(100);  // 100msec, delay  between reg_on & rst. (bcm4329 powerup sequence)

			/* Set GPIO_BT_nRST high */
			s3c_gpio_setpull(GPIO_BT_nRST, S3C_GPIO_PULL_NONE);
			gpio_set_value(GPIO_BT_nRST, GPIO_LEVEL_HIGH);

			s3c_gpio_slp_cfgpin(GPIO_BT_nRST, S3C_GPIO_SLP_OUT1);
			s3c_gpio_slp_setpull_updown(GPIO_BT_nRST, S3C_GPIO_PULL_NONE);

			printk("[BT] GPIO_BT_nRST = %d\n", gpio_get_value(GPIO_BT_nRST));

			gpio_free(GPIO_BT_nRST);
			gpio_free(GPIO_WLAN_BT_EN);

			break;

		case RFKILL_USER_STATE_SOFT_BLOCKED:
			printk(KERN_DEBUG "[BT] Device Powering OFF \n");
//			s3c_reset_uart_cfg_gpio(0);

			s3c_gpio_setpull(GPIO_BT_nRST, S3C_GPIO_PULL_NONE);
			gpio_set_value(GPIO_BT_nRST, GPIO_LEVEL_LOW);

			s3c_gpio_slp_cfgpin(GPIO_BT_nRST, S3C_GPIO_SLP_OUT0);
			s3c_gpio_slp_setpull_updown(GPIO_BT_nRST, S3C_GPIO_PULL_NONE);

			printk("[BT] GPIO_BT_nRST = %d\n",gpio_get_value(GPIO_BT_nRST));

			if(gpio_get_value(gpio_wlan_rst) == 0)
			{		
				s3c_gpio_setpull(GPIO_WLAN_BT_EN, S3C_GPIO_PULL_NONE);
				gpio_set_value(GPIO_WLAN_BT_EN, GPIO_LEVEL_LOW);

				s3c_gpio_slp_cfgpin(GPIO_WLAN_BT_EN, S3C_GPIO_SLP_OUT0);
				s3c_gpio_slp_setpull_updown(GPIO_WLAN_BT_EN, S3C_GPIO_PULL_NONE);

				printk("[BT] GPIO_WLAN_BT_EN = %d\n", gpio_get_value(GPIO_WLAN_BT_EN));
			}

			bt_config_gpio_table(ARRAY_SIZE(bt_uart_off_table), bt_uart_off_table);

			gpio_free(GPIO_BT_nRST);
			gpio_free(GPIO_WLAN_BT_EN);

			break;

		default:
			printk(KERN_ERR "[BT] Bad bluetooth rfkill state %d\n", state);
	}

	return 0;
}


static void bt_host_wake_work_func(struct work_struct *ignored)
{
	printk(KERN_DEBUG "[BT] wake_lock timeout = 5 sec\n");
	wake_lock_timeout(&rfkill_wake_lock, 5*HZ);
	
	if(bt_init_complete && !lpaudio_lock)
	{
		s5p_set_lpaudio_lock(1);
		lpaudio_lock = 1;
	}
	
	enable_irq(IRQ_BT_HOST_WAKE);
}
static DECLARE_WORK(bt_host_wake_work, bt_host_wake_work_func);


irqreturn_t bt_host_wake_irq_handler(int irq, void *dev_id)
{
	//printk(KERN_DEBUG "[BT] bt_host_wake_irq_handler start\n");

	disable_irq_nosync(IRQ_BT_HOST_WAKE);

	schedule_work(&bt_host_wake_work);

	return IRQ_HANDLED;
}

static int bt_rfkill_set_block(void *data, bool blocked)
{
	int ret =0;
	
	ret = bluetooth_set_power(data, blocked? RFKILL_USER_STATE_SOFT_BLOCKED : RFKILL_USER_STATE_UNBLOCKED);
		
	return ret;
}

static const struct rfkill_ops bt_rfkill_ops = {
	.set_block = bt_rfkill_set_block,
};

static int __init jupiter_rfkill_probe(struct platform_device *pdev)
{
	int rc = 0;
	int irq,ret;

	//Initialize wake locks
	wake_lock_init(&rfkill_wake_lock, WAKE_LOCK_SUSPEND, "board-rfkill");
#ifdef BT_SLEEP_ENABLER
	wake_lock_init(&bt_wake_lock, WAKE_LOCK_SUSPEND, "bt-rfkill");
#endif

	//BT Host Wake IRQ
	irq = IRQ_BT_HOST_WAKE;
	
	if (HWREV >= 12)
		bt_config_gpio_table(ARRAY_SIZE(bt_gpio_table_rev06), bt_gpio_table_rev06);
	else
		bt_config_gpio_table(ARRAY_SIZE(bt_gpio_table), bt_gpio_table);

	set_irq_type(irq, IRQ_TYPE_EDGE_RISING);

	ret = request_irq(irq, bt_host_wake_irq_handler, 0, "bt_host_wake_irq_handler", NULL);
	if(ret < 0)
		printk(KERN_ERR "[BT] Request_irq failed \n");

//	enable_irq(IRQ_BT_HOST_WAKE);

	//RFKILL init - default to bluetooth off
	//rfkill_switch_all(RFKILL_TYPE_BLUETOOTH, RFKILL_USER_STATE_SOFT_BLOCKED);

	bt_rfk = rfkill_alloc(bt_name, &pdev->dev, RFKILL_TYPE_BLUETOOTH, &bt_rfkill_ops, NULL);
	if (!bt_rfk)
		return -ENOMEM;

	rfkill_init_sw_state(bt_rfk, 0);

	printk(KERN_DEBUG "[BT] rfkill_register(bt_rfk) \n");

	rc = rfkill_register(bt_rfk);
	if (rc)
	{
		printk ("***********ERROR IN REGISTERING THE RFKILL***********\n");
		rfkill_destroy(bt_rfk);
	}

	rfkill_set_sw_state(bt_rfk, 1);
	bluetooth_set_power(NULL, RFKILL_USER_STATE_SOFT_BLOCKED);

	return rc;
}

static struct platform_driver jupiter_device_rfkill = {
	.probe = jupiter_rfkill_probe,
	.driver = {
		.name = "bt_rfkill",
		.owner = THIS_MODULE,
	},
};

#ifdef BT_SLEEP_ENABLER
static struct rfkill *bt_sleep;

static int bluetooth_set_sleep(void *data, enum rfkill_user_states state)
{	
	int ret =0;
	unsigned int gpio_bt_wake;
	
	if (HWREV >= 12)
		gpio_bt_wake = GPIO_BT_WAKE_REV06;
	else
		gpio_bt_wake = GPIO_BT_WAKE;
	
	switch (state) {

		case RFKILL_USER_STATE_UNBLOCKED:
			printk ("[BT] In the unblocked state of the sleep\n");
			if (gpio_is_valid(gpio_bt_wake))
			{
				ret = gpio_request(gpio_bt_wake, "GPG3");
				if(ret < 0) {
					printk(KERN_ERR "[BT] Failed to request GPIO_BT_WAKE!\n");
					return ret;
				}
				gpio_direction_output(gpio_bt_wake, GPIO_LEVEL_LOW);
			}

			s3c_gpio_setpull(gpio_bt_wake, S3C_GPIO_PULL_NONE);
			gpio_set_value(gpio_bt_wake, GPIO_LEVEL_LOW);
			
			printk(KERN_DEBUG "[BT] GPIO_BT_WAKE = %d\n", gpio_get_value(gpio_bt_wake) );
			gpio_free(gpio_bt_wake);
			
			printk("[BT] wake_unlock(bt_wake_lock)\n");
			wake_unlock(&bt_wake_lock);
			
			if(bt_init_complete && lpaudio_lock)
			{
				s5p_set_lpaudio_lock(0);
				lpaudio_lock = 0;
			}
						
			break;

		case RFKILL_USER_STATE_SOFT_BLOCKED:
			printk ("[BT] In the soft blocked state of the sleep\n");
			if (gpio_is_valid(gpio_bt_wake))
			{
				ret = gpio_request(gpio_bt_wake, "GPG3");
				if(ret < 0) {
					printk(KERN_ERR "[BT] Failed to request GPIO_BT_WAKE!\n");
					return ret;
				}
				gpio_direction_output(gpio_bt_wake, GPIO_LEVEL_HIGH);
			}

			s3c_gpio_setpull(gpio_bt_wake, S3C_GPIO_PULL_NONE);
			gpio_set_value(gpio_bt_wake, GPIO_LEVEL_HIGH);

			printk(KERN_DEBUG "[BT] GPIO_BT_WAKE = %d\n", gpio_get_value(GPIO_BT_WAKE) );
			gpio_free(gpio_bt_wake);
			printk("[BT] wake_lock(bt_wake_lock)\n");
			wake_lock(&bt_wake_lock);
			break;

		default:
			printk(KERN_ERR "[BT] bad bluetooth rfkill state %d\n", state);
	}
	return 0;
}

static int btsleep_rfkill_set_block(void *data, bool blocked)
{
	int ret =0;
	
	ret = bluetooth_set_sleep(data, blocked? RFKILL_USER_STATE_SOFT_BLOCKED : RFKILL_USER_STATE_UNBLOCKED);
		
	return ret;
}

static const struct rfkill_ops btsleep_rfkill_ops = {
	.set_block = btsleep_rfkill_set_block,
};

static int __init jupiter_btsleep_probe(struct platform_device *pdev)
{
	int rc = 0;
	
	bt_sleep = rfkill_alloc(bt_name, &pdev->dev, RFKILL_TYPE_BLUETOOTH, &btsleep_rfkill_ops, NULL);
	if (!bt_sleep)
		return -ENOMEM;

	rfkill_set_sw_state(bt_sleep, 1);

	rc = rfkill_register(bt_sleep);
	if (rc)
		rfkill_destroy(bt_sleep);

	//bluetooth_set_sleep(NULL, RFKILL_USER_STATE_UNBLOCKED);

	return rc;
}

static struct platform_driver jupiter_device_btsleep = {
	.probe = jupiter_btsleep_probe,
	.driver = {
		.name = "bt_sleep",
		.owner = THIS_MODULE,
	},
};
#endif

static int __init jupiter_rfkill_init(void)
{
	int rc = 0;
	rc = platform_driver_register(&jupiter_device_rfkill);

#ifdef BT_SLEEP_ENABLER
	platform_driver_register(&jupiter_device_btsleep);
#endif

	bt_init_complete = 1;
	return rc;
}

module_init(jupiter_rfkill_init);
MODULE_DESCRIPTION("jupiter rfkill");
MODULE_LICENSE("GPL");
