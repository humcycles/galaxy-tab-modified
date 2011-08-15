/*
 * Driver for keys on GPIO lines capable of generating interrupts.
 *
 * Copyright 2005 Phil Blundell
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/sched.h>
#include <linux/pm.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <asm/gpio.h>
#include <plat/gpio-cfg.h>
#include <linux/miscdevice.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#define IRQ_TOUCHKEY_INT                IRQ_EINT(19)
#define DEVICE_NAME                          "melfas-touchkey"

/* Melfas touchkey register */
#define KEYCODE_REG                          0x00
#define FIRMWARE_VERSION                0x01
//#define TOUCHKEY_MODULE_VERSION  0x02
//#define TOUCHKEY_ADDRESS	             0x20

#define MAX_KEYS	                           3
#define TK_STATUS_PRESS			1
#define TK_STATUS_RELEASE		       0
#define UPDOWN_EVENT_BIT                0x08
#define KEYCODE_BIT                           0x07

static int touchkey_keycode[MAX_KEYS] = {NULL, KEY_MENU, KEY_BACK };
char *touchkeycode_name[MAX_KEYS]=
{
        "NULL",
        "Menu",
        "Back"
};

static int touchkey_status[MAX_KEYS] = {0,};

//extern symbol from mach-p1.c
extern unsigned int HWREV;

struct melfas_touchkey_data{
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct work_struct work;
	struct early_suspend earlysuspend;
};

struct melfas_touchkey_data *touchkey_data = NULL;

#ifdef CONFIG_HAS_EARLYSUSPEND
static void led_early_suspend(struct early_suspend *h);
static void led_early_resume(struct early_suspend *h);
#endif

static int i2c_touchkey_read(u8 reg, u8 *val, unsigned int len)
{
	int 	 err;
	struct 	 i2c_msg msg[1];

	if( (touchkey_data->client == NULL) || (!touchkey_data->client->adapter) )
	{
		return -ENODEV;
	}

	msg->addr 	= touchkey_data->client->addr;
	msg->flags = I2C_M_RD;
	msg->len   = len;
	msg->buf   = val;
	err = i2c_transfer(touchkey_data->client->adapter, msg, 1);

	if (err >= 0)
	{
		return 0;
	}
	printk("[TouchKey] %s %d i2c transfer error\n", __func__, __LINE__);/* add by inter.park */

	return err;

}

static void i2c_touchkey_write( u8 val )
{
    int ret;

    ret = i2c_smbus_xfer(touchkey_data->client->adapter, touchkey_data->client->addr,
            touchkey_data ->client->flags, I2C_SMBUS_WRITE, val, I2C_SMBUS_BYTE, NULL);

    if (ret < 0)
    {
        printk("[TouchKey] Failed to write i2c.");
    }

}

void  touchkey_work_func(struct work_struct * p)
{
	u8 data;
	int i, code;
	int retry =10;

       while(retry--)
       {
        	if(i2c_touchkey_read(KEYCODE_REG, &data, 1) < 0)
        	{
        		printk("[TouchKey] %s %d i2c transfer error retry = %d\n", __func__, __LINE__, retry);
        		mdelay(10);
        	}
        	else
        	{
        	    break;
        	}
	}

	if(data & UPDOWN_EVENT_BIT)
	{
		/* press */
		code = data & KEYCODE_BIT;
		if(code >= MAX_KEYS)
		{
			printk("[TouchKey]  unknown touch key pressed: 0x%x", code);
		}
		else
		{
			touchkey_status[code] = TK_STATUS_PRESS;
			input_report_key(touchkey_data->input_dev, touchkey_keycode[code],1);
			input_sync(touchkey_data->input_dev);
			printk("[TouchKey] %s key is pressed , Keycode: %d\n", touchkeycode_name[code], touchkey_keycode[code]);
		}
	}
	else
	{
		for(i=0;i<MAX_KEYS;i++)
		{
			if(touchkey_status[i] == TK_STATUS_PRESS)
			{
				input_report_key(touchkey_data->input_dev, touchkey_keycode[i], 0);
				input_sync(touchkey_data->input_dev);
				printk("[TouchKey] %s key is released , Keycode: %d\n", touchkeycode_name[i], touchkey_keycode[i]);
				touchkey_status[i] = TK_STATUS_RELEASE;
			}
		}
	}

}

static irqreturn_t touchkey_interrupt(int irq, void *dev_id)
{
    if (!work_pending(&touchkey_data->work))
        schedule_work(&touchkey_data->work);

    return IRQ_HANDLED;
}

static int __devinit touchkey_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    struct melfas_touchkey_data *pdata;
    struct input_dev *input_dev;
    int err = 0;
//    u8 data[3];
    int key;

    pdata = kzalloc(sizeof(struct melfas_touchkey_data), GFP_KERNEL);
    input_dev = input_allocate_device();

    if (!pdata || !input_dev)
    {
        kfree(pdata);
        input_free_device(input_dev);
        return -ENOMEM;
    }

    pdata->client = client;

    pdata->input_dev = input_dev;

    input_dev->name = DEVICE_NAME;
    input_dev->phys = "melfas-touchkey/input0";
    input_dev->id.bustype = BUS_HOST;

    set_bit(EV_SYN, input_dev->evbit);
    set_bit(EV_KEY, input_dev->evbit);
    set_bit(touchkey_keycode[1], input_dev->keybit);
    set_bit(touchkey_keycode[2], input_dev->keybit);

    err = input_register_device(input_dev);
    if (err)
    {
        input_free_device(input_dev);
        return err;
    }

    // for touch key
    for(key = 0; key < MAX_KEYS; key++)
    {
        touchkey_status[key] = TK_STATUS_RELEASE;
    }

    INIT_WORK(&pdata->work, touchkey_work_func);
    touchkey_data = pdata;

    if (request_irq(IRQ_TOUCHKEY_INT, touchkey_interrupt, IRQF_SAMPLE_RANDOM
            | IRQF_TRIGGER_RISING |IRQF_TRIGGER_FALLING, client->dev.driver->name, pdata))
    {
        printk("[TouchKey] %s Can't allocate irq ..\n", __func__);
        return -EBUSY;
    }

#ifdef CONFIG_HAS_EARLYSUSPEND
    touchkey_data->earlysuspend.suspend =led_early_suspend;
    touchkey_data->earlysuspend.resume = led_early_resume;
    register_early_suspend(&touchkey_data->earlysuspend);
#endif

//    i2c_touchkey_read(FIRMWARE_VERSION, data,3);
//    printk("[TouchKey] %s F/W version: 0x%x, Module version:0x%x\n",__func__, data[1],data[2]);

    return err;
}

static int __devexit touchkey_remove(struct i2c_client *client)
{
  	i2c_set_clientdata(client, NULL);
	return 0;
}

static void init_hw(void)
{
#ifndef CONFIG_TARGET_LOCALE_KOR
    s3c_gpio_setpull(GPIO_TOUCH_KEY_INT, S3C_GPIO_PULL_NONE);
    set_irq_type(IRQ_TOUCHKEY_INT, IRQ_TYPE_EDGE_FALLING);
#endif
}

static ssize_t touch_led_control(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    unsigned int data;

    sscanf(buf, "%u", &data);

    if(data == 1)
    {
        printk("[TouchKey] LED on\n");
        // Turn on melfas led
        i2c_touchkey_write(0x1);
    }
    else
    {
        printk("[TouchKey] LED off\n");
        // Turn off melfas led
        i2c_touchkey_write(0x2);
    }
    return size;
}

static int touchkey_suspend(struct i2c_client *client, pm_message_t mesg)
{
    //LED off
    i2c_touchkey_write(0x2);    // 1 : on , 2: off
    return 0;
}
static int touchkey_resume(struct i2c_client *client)
{
    //LED on
    i2c_touchkey_write(0x1);    // 1 : on , 2: off
    return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void led_early_suspend(struct early_suspend *h)
{
    printk("[TouchKey] %s \n",__func__);
    touchkey_suspend(touchkey_data->client, PMSG_SUSPEND);
}

static void led_early_resume(struct early_suspend *h)
{
    printk("[TouchKey] %s \n",__func__);
    touchkey_resume(touchkey_data->client);
}
#endif	// End of CONFIG_HAS_EARLYSUSPEND


static DEVICE_ATTR(brightness, S_IRUGO | S_IWUSR | S_IWOTH | S_IXOTH, NULL, touch_led_control);

struct file_operations keyled_control_fops =
{
	.owner   = THIS_MODULE,
};

static struct miscdevice keyled_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "melfas_touchkey",
	.fops = &keyled_control_fops,
};

static const struct i2c_device_id touchkey_id[] = {
	{ "melfas_touchkey_i2c", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, touchkey_id);

static struct i2c_driver touchkey_driver = {
	.driver = {
		.name = "melfas_touchkey_i2c",
	},
	.probe          = touchkey_probe,
	.remove        = __devexit_p(touchkey_remove),
	.suspend      =   touchkey_suspend,
	.resume        = touchkey_resume,
	.id_table      = touchkey_id,
};


static int __init touchkey_init(void)
{
    int ret = 0;

    if( HWREV >= 0x5)
    {
        return ret;
    }

    ret = misc_register(&keyled_device);
    if (ret) {
        printk("[TouchKey] %s misc_register fail\n",__func__);
    }

    if (device_create_file(keyled_device.this_device, &dev_attr_brightness) < 0)
    {
        printk("[TouchKey] %s device_create_file fail dev_attr_touch_update\n",__func__);
        pr_err("[TouchKey] Failed to create device file(%s)!\n", dev_attr_brightness.attr.name);
    }

    init_hw();
    ret = i2c_add_driver(&touchkey_driver);
    if(ret)
    {
    	printk("[TouchKey] melfas touch keypad registration failed, module not inserted.ret= %d\n",ret);
    }
    return ret;
}

static void __exit touchkey_exit(void)
{
	i2c_del_driver(&touchkey_driver);
	misc_deregister(&keyled_device);
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&touchkey_data->earlysuspend);
#endif	/* CONFIG_HAS_EARLYSUSPEND */
}

late_initcall(touchkey_init);
module_exit(touchkey_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("@@@");
MODULE_DESCRIPTION("melfas touch keypad");

