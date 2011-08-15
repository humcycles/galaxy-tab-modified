/* 
 *
 * Sain touch driver
 *
 * Copyright (C) 2009 Sain, Inc.
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

#include <linux/module.h>
#include <linux/input.h>
#include <linux/i2c.h>		// I2C_M_NOSTART
#include <linux/miscdevice.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/hrtimer.h>
#include <linux/ioctl.h>
#include <linux/earlysuspend.h>
#include <linux/string.h>
#include <linux/semaphore.h>
#include <linux/kthread.h>
#include "sain_touch_reg.h"

#include <asm/io.h>
#include <linux/delay.h>
//#include <mach/vreg.h>          /* set a vreg */
#include <mach/gpio.h>

//#include <asm/gpio.h>

#if TOUCH_UPGRADE_MODULE
#include <linux/errno.h>
#include <linux/miscdevice.h>
#include <linux/ioport.h>
#include <linux/fcntl.h>
#include <linux/spinlock.h>
#include <asm/uaccess.h>
#include <asm/system.h>
#endif

#include "mach/gpio-p1.h"

#if TOUCH_ONESHOT_UPGRADE
#include "sain_touch_firmware.h"
#endif

#define	SAIN_DEBUG	1

#define SAIN_DRIVER_NAME        "sain_touch"	//"sain_touch"

//#define GPIO_TOUCH_IRQ  	109	// interrupt pin number
//#define GPIO_TOUCH_IRQ  	GPIO_TOUCH_IRQ  		// interrupt pin number	// interrupt pin number

#define IRQ_TOUCH_INT       IRQ_EINT_GROUP(18, 5)	// group 18 : J0

#define	SAIN_INIT_RETRY_CNT	10

#if	SAIN_DEBUG
#define	sain_debug_msg(fmt, args...)	printk(KERN_INFO "[%-18s:%5d]" fmt, __FUNCTION__, __LINE__, ## args)
#else
#define	sain_debug_msg(fmt, args...)	do{}while(0)
#endif

#define swap_v(a, b, t)	((t) = (a), (a) = (b), (b) = (t))
#define swap_16(s) (((((s) & 0xff) << 8) | (((s) >> 8) & 0xff))) 

//typedef	int	bool;

#define	TRUE					1
#define	FALSE					0

#define	VALID_EVENT				1
#define	INVALID_EVENT				0

#define	USE_DEFAULT_ID				1
#define	SUPPORTED_FINGER_NUM			1
#define	MULTI_TOUCH_SUPPORT			1
#define	GESTURE_SUPPORT				0
#define	SUPPORTED_BUTTON_NUM			4


#define	ICON_BUTTON_UNCHANGE			0
#define	ICON_BUTTON_DOWN			1
#define	ICON_BUTTON_UP				2


#define	I2C_SUCCESS				0

#define ts_write_cmd(client,reg) i2c_smbus_write_byte(client, reg)
#define ts_write_reg(client,reg,value) i2c_smbus_write_word_data(client, reg, value)
#define ts_select_reg(client,reg) i2c_smbus_write_byte(client, reg)
#define ts_read_reg(client,reg) i2c_smbus_read_word_data(client, reg)

// include/linux/i2c.h	-> I2C_SMBUS_BLOCK_MAX °ª º¯°æ
#if 0
#define ts_read_data(client,reg,values,length) i2c_smbus_read_i2c_block_data(client, reg, length, values)
#else
inline s32 ts_read_data(struct i2c_client *client, u8 reg, u8 *values, u8 length)
{
	s32 ret;
	u8  register_num;

	register_num = reg;
	
	if((ret = i2c_master_send(client , &register_num , 1)) < 0)	return ret;	// select register
	ndelay(100);		// for setup tx transaction.
//        udelay(20);		// for setup tx transaction.
	if((ret = i2c_master_recv(client , values , length)) < 0)	return ret;

	return length;
}

#endif

#define ts_write_data(client,reg,values,length) i2c_smbus_write_i2c_block_data(client, reg, length, values)

// default vallues
//-------------------------------------------------------
#define	TOUCH_MODE				0
#define	SENSITIVITY				300 // for 7"

#define	HOLD_THRESHOLD				32
#define	REACTION_TIME				2
#define PALM_REJECT_THRESHHOLD			50	// tunning point
#define NOISE_REJECT_THRESHHOLD			80	// tunning point
#define STYLUS_EDGE_COEF			110
#define STYLUS_HW_THRESHHOLD 			1500	// tuning point

//-------------------------------------------------------

// define event type
#define	EVENT_UP				1
#define	EVENT_DOWN				2
#define	EVENT_MOVE				3
#define	EVENT_HOLD				4
#define	EVENT_LONG_HOLD				5
#define	EVENT_PINCH_IN				6
#define	EVENT_PINCH_OUT				7
#define	EVENT_FLICK_UP				8
#define	EVENT_FLICK_DOWN			9
#define	EVENT_FLICK_RIGHT			10
#define	EVENT_FLICK_LEFT			11


#define	UPGRADE_DATA_SIZE			512
#define	TOUCH_UPGRADE_MINOR			150	// include/linux/miscdevices.h

typedef	struct	
{
	u16	address;
	u8	data[UPGRADE_DATA_SIZE];
}_sain_upgrade_info;

typedef	struct	
{
	u16	x;
	u16	y;
	u16	width;	
}_ts_sain_coord;

typedef	struct	
{
	u16	status;
	u8	finger_cnt;
	u8	time_stamp;
	_ts_sain_coord	coord[SUPPORTED_FINGER_NUM];

}_ts_sain_point_info;


#define	TOUCH_V_FLIP	0x01
#define	TOUCH_H_FLIP	0x02
#define	TOUCH_XY_SWAP	0x04

typedef	struct
{
	u32 chip_revision; 
	u32 chip_firmware_version;
			  
	u32 MaxX;
	u32 MaxY;
	u32 MinX;
	u32 MinY;
	u32 Orientation;
	u8 gesture_support;
}_ts_capa_info;

typedef struct
{
	struct input_dev *input_dev;
	struct task_struct *task;
	struct i2c_client *client;
	struct semaphore update_lock;
    
	u32 i2c_dev_addr;
	_ts_capa_info	cap_info;
    	
	bool is_valid_event;
	_ts_sain_point_info touch_info;
	_ts_sain_point_info prev_touch_info;    
	u16 onefinger_gesture_reg;
	u16 twofinger_gesture_reg;
	u16 icon_event_reg;
	u16 chip_int_mask;    
	u16 event_type;
	u32 int_gpio_num;
	u32 irq;
	u8 button[SUPPORTED_BUTTON_NUM];
} sain_touch_dev;

#define	TS_DRIVER_NAME	"sain_touch"	// for i2c
static struct i2c_device_id sain_idtable[] = {
    {TS_DRIVER_NAME, 0},  
    { }   
};

u32 BUTTON_MAPPING_KEY[SUPPORTED_BUTTON_NUM] = {KEY_SEARCH, KEY_BACK, KEY_HOME, KEY_MENU};

static int sain_touch_probe(struct i2c_client *client, const struct i2c_device_id *i2c_id);
static int sain_touch_remove(struct i2c_client *client);

// id -> include/linux/i2c-id.h 
static struct i2c_driver sain_touch_driver = {
    .probe     = sain_touch_probe,
    .remove    = sain_touch_remove,

    .id_table  = sain_idtable,
    .driver    = {
        .name  = SAIN_DRIVER_NAME,
    },
};

#if TOUCH_UPGRADE_MODULE
sain_touch_dev *m_upgrade_dev;
#endif



static bool ts_get_samples (sain_touch_dev* touch_dev)
{
	u32 event_type;
	int i;

//	sain_debug_msg("ts_get_samples+\r\n");
	
	if (gpio_get_value(touch_dev->int_gpio_num))
	{
        	//interrupt pin is high, not valid data.
//		printk(KERN_WARNING "woops... inturrpt pin is high\r\n");
        	return FALSE;
    	}
    
	touch_dev->is_valid_event = INVALID_EVENT;

	if (ts_read_data (touch_dev->client, SAIN_POINT_STATUS_REG, (u8*)(&touch_dev->touch_info), sizeof(_ts_sain_point_info))< 0)
	{
		sain_debug_msg("error read point info using i2c.-\r\n");
        	return FALSE;
	}
#if 1	
	sain_debug_msg("status reg = 0x%x , point cnt = %d, time stamp = %d, one gesture = %x, two gesture = %x\r\n", touch_dev->touch_info.status, 
                           touch_dev->touch_info.finger_cnt, touch_dev->touch_info.time_stamp, touch_dev->onefinger_gesture_reg, touch_dev->twofinger_gesture_reg);
        sain_debug_msg("x:%d, y:%d\n", touch_dev->touch_info.coord[0].x , touch_dev->touch_info.coord[0].y);
#endif

	for(i=0; i< SUPPORTED_BUTTON_NUM; i++)	touch_dev->button[i] = ICON_BUTTON_UNCHANGE;

	if(sain_bit_test(touch_dev->touch_info.status, BIT_ICON_EVENT))
	{
	        sain_debug_msg("status reg : 0x%x", touch_dev->touch_info.status);
                udelay(1);
		if (ts_read_data (touch_dev->client, SAIN_ICON_STATUS_REG, (u8*)(&touch_dev->icon_event_reg), 2) < 0)
		{
			printk(KERN_INFO "error read icon info using i2c.\n");
        		return FALSE;
		}	
                sain_debug_msg("icon_event_reg : 0x%x", touch_dev->icon_event_reg);
		for(i=0; i<SUPPORTED_BUTTON_NUM; i++)
		{
			if(sain_bit_test(touch_dev->icon_event_reg, (BIT_O_ICON0_DOWN+i)))
			{
				touch_dev->button[i] = ICON_BUTTON_DOWN;
				touch_dev->is_valid_event = VALID_EVENT;				
			}
		}

		for(i=0; i<SUPPORTED_BUTTON_NUM; i++)
		{
			if(sain_bit_test(touch_dev->icon_event_reg, (BIT_O_ICON0_UP+i)))
			{
				touch_dev->button[i] = ICON_BUTTON_UP;	
				touch_dev->is_valid_event = VALID_EVENT;				
			}
		}

	}
  	
  	
	if(sain_bit_test(touch_dev->touch_info.status, BIT_PT_NO_CHANGE))
	{
		touch_dev->touch_info.finger_cnt = SUPPORTED_FINGER_NUM + 1;
		return TRUE;
	}

	
	if(!sain_bit_test(touch_dev->touch_info.status, BIT_PT_EXIST))
	{
		if(touch_dev->prev_touch_info.finger_cnt > 0)
		{
			event_type = EVENT_UP;	
			touch_dev->is_valid_event = VALID_EVENT;
			touch_dev->event_type = event_type;
		}

		//memset((char *)&touch_dev->touch_info, 0x0, sizeof(_ts_sain_point_info));		
		memset((char *)&touch_dev->prev_touch_info, 0x0, sizeof(_ts_sain_point_info));
		return TRUE;
	}

		
	event_type = 0xFFFFFFFF;
	if(sain_bit_test(touch_dev->touch_info.status, BIT_DOWN))
	{				
		event_type = EVENT_DOWN;
	}
	else if(sain_bit_test(touch_dev->touch_info.status, BIT_MOVE))
	{
		event_type = EVENT_MOVE;
	}
	
	if(event_type != 0xFFFFFFFF)
	{
		touch_dev->is_valid_event = VALID_EVENT;
		touch_dev->event_type = event_type;
	}
	
	memcpy((char*)&touch_dev->prev_touch_info, (char*)&touch_dev->touch_info, sizeof(_ts_sain_point_info));	

//	sain_debug_msg("ts_get_samples-\r\n");
	
	return TRUE;
}


static int ts_int_handler(int irq, void *dev)
{
	sain_touch_dev* touch_dev = (sain_touch_dev*)dev;
//	sain_debug_msg("interrupt occured +\r\n");
//	disable_irq(irq);	
        disable_irq_nosync(irq);
	up(&touch_dev->update_lock); 	
	return IRQ_HANDLED;
}

bool ts_read_coord (sain_touch_dev * hDevice)
{
    sain_touch_dev* touch_dev = (sain_touch_dev*)hDevice;

//    sain_debug_msg("ts_read_coord+\r\n");

    if(ts_get_samples(touch_dev)==FALSE)
    	return FALSE;

    ts_write_cmd(touch_dev->client, SAIN_CLEAR_INT_STATUS_CMD);

    
    return TRUE;
}

#if	TOUCH_ONESHOT_UPGRADE
bool ts_check_need_upgrade(u16 curVersion)
{
	u16	newVersion;
	newVersion = (u16) (m_Firmware_data_4002[0] | (m_Firmware_data_4002[1]<<8));

	printk(KERN_INFO "cur Version = 0x%x, new Version = 0x%x\n", curVersion, newVersion);
	
	if(curVersion >= newVersion)
		return FALSE;

	return TRUE;
}

#if SAIN_TS_SERIES
#define	INFO_START	0x1000
#define	INFO_END	0x10FF

#define	CODE_START	0x8000
#define	CODE_END	0xF9FF

NvBool ts_upgrade_firmware(sain_touch_dev* touch_dev)
{
	u16 flash_addr;
	_sain_upgrade_info	up_info;

	printk(KERN_INFO "Erase Flash\n");
	if (ts_write_reg(touch_dev->client, SAIN_ERASE_FIRMWARE, 0xaaaa)==I2C_SUCCESS)
		goto fail_upgrade;
	
	mdelay(2000);

	printk(KERN_INFO "Writing Information Data\n");
			
	for(flash_addr= INFO_START; flash_addr< INFO_END; flash_addr+=UPGRADE_DATA_SIZE)
	{
		printk(KERN_INFO " info start = %x, info end = %x, addr = %x\n", INFO_START, INFO_END, flash_addr);

		up_info.address = flash_addr;
		swap_16	(up_info.address);	
		memcpy(up_info.data , &m_Firmware_data_4002[flash_addr], UPGRADE_DATA_SIZE);
		
		if(ts_write_data(m_upgrade_dev->client,SAIN_WRITE_FIRMWARE,(char*)&up_info,sizeof(_sain_upgrade_info))<0)
		{
			printk(KERN_ERR"sain_upgrade_ioctl : cannot write updage data (addr = %x)!!\r\n", up_info.address);
			goto fail_upgrade;
		}	
		while(!gpio_get_value(m_upgrade_dev->int_gpio_num));		

	}


	for(flash_addr= CODE_START; flash_addr< CODE_END; flash_addr+=UPGRADE_DATA_SIZE)
	{
		printk(KERN_INFO " code start = %x, code end = %x, addr = %x\n", CODE_START, CODE_END, flash_addr);
		up_info.address = flash_addr;
		swap_16	(up_info.address);	
		memcpy(up_info.data , &m_Firmware_data_4002[flash_addr], UPGRADE_DATA_SIZE);
		
		if(ts_write_data(m_upgrade_dev->client,SAIN_WRITE_FIRMWARE,(char*)&up_info,sizeof(_sain_upgrade_info))<0)
		{
			printk(KERN_ERR"sain_upgrade_ioctl : cannot write updage data (addr = %x)!!\r\n", up_info.address);
			goto fail_upgrade;
		}	
		while(!gpio_get_value(m_upgrade_dev->int_gpio_num));		

	}

	printk(KERN_INFO"Upgrade Finished\n");

	return TRUE;

fail_upgrade:
	return FALSE;
	
}
#else

#define	TC_PAGE_SZ		64
#define	TC_SECTOR_SZ		8

u8 ts_upgrade_firmware(sain_touch_dev* touch_dev)
{
	u16 flash_addr;
	_sain_upgrade_info	up_info;
	u32 i;


	printk(KERN_INFO "reset command\n");
	if (ts_write_cmd(touch_dev->client, SAIN_REST_CMD)!=I2C_SUCCESS)
	{
		printk(KERN_INFO "failed to reset\n");
		goto fail_upgrade;
	}
	
	printk(KERN_INFO "Erase Flash\n");
	if (ts_write_reg(touch_dev->client, SAIN_ERASE_FLASH, 0xaaaa)!=I2C_SUCCESS)
	{
		printk(KERN_INFO "failed to erase flash\n");
		goto fail_upgrade;
	}
	
	mdelay(2000);

	printk(KERN_INFO "writing firmware data\n");			
		
	for(flash_addr= 0; flash_addr< 16*1024; )
	{
			
		for(i=0; i< TC_PAGE_SZ/TC_SECTOR_SZ; i++)
		{
			printk(KERN_INFO "addr = %04x, len = %d\n", flash_addr, TC_SECTOR_SZ);
			if(ts_write_data(touch_dev->client,SAIN_WRITE_FLASH,&m_Firmware_data_4002[flash_addr+2],TC_SECTOR_SZ)<0)
			{
				printk(KERN_INFO"error : write sain tc firmare\n");
				goto fail_upgrade;
			}		
			flash_addr+= TC_SECTOR_SZ;			
		}
		mdelay(20);			
	}
	printk(KERN_INFO"upgrade finished\n");

	return TRUE;

fail_upgrade:
	return FALSE;
	
}

#endif

#endif

static volatile u16 max_x = 1023;
static volatile u16 max_y = 599;
//static volatile u32 orient = 0;
static volatile u32 orient = TOUCH_V_FLIP + TOUCH_H_FLIP;	 

bool ts_init_touch(sain_touch_dev* touch_dev)
{
	u16	reg_val;
	int	i;
	u16 	SetMaxX = max_x; //Max Position range from 0x0002 to 0x1fff
	u16 	SetMaxY = max_y; //Max Position range from 0x0002 to 0x1fff  

	u16 	CurMaxX = 1024;
	u16 	CurMaxY = 1920;      
	u16 	chip_revision;
	u16 	chip_firmware_version;


	sain_debug_msg("sain touch init +\r\n");

	if(touch_dev == NULL)
	{
		printk(KERN_ERR "error touch_dev == null?\r\n");
		goto fail_init;
	}

	sain_debug_msg("disable interrupt\r\n");
	for(i=0; i<SAIN_INIT_RETRY_CNT; i++)
	{
		if (ts_write_reg(touch_dev->client, SAIN_INT_ENABLE_FLAG, 0x0)==I2C_SUCCESS)
			break;
	}

	if(i==SAIN_INIT_RETRY_CNT)
	{
		printk(KERN_INFO "fail to write interrupt register\r\n");
		goto fail_init;
	}
	sain_debug_msg("successfully disable int. (retry cnt = %d)\r\n", i);

	sain_debug_msg("send reset command\r\n");
	if (ts_write_cmd(touch_dev->client, SAIN_REST_CMD)!=I2C_SUCCESS)
		goto fail_init;

	/* get chip revision id */
	if (ts_read_data(touch_dev->client, SAIN_CHIP_REVISION, (u8*)&chip_revision, 2)<0)
	{
		printk(KERN_INFO "fail to read chip revision\r\n");
		goto fail_init;
	}


	touch_dev->cap_info.chip_revision = (u32)chip_revision;
	printk(KERN_INFO "sain touch chip revision id = %x\r\n", touch_dev->cap_info.chip_revision);

	/* get chip firmware version */
	if (ts_read_data(touch_dev->client, SAIN_FIRMWARE_VERSION, (u8*)&chip_firmware_version, 2)<0) goto fail_init;

	touch_dev->cap_info.chip_firmware_version = (u32)chip_firmware_version;
	printk(KERN_INFO "sain touch chip firmware version = %x\r\n", touch_dev->cap_info.chip_firmware_version);

#if 0//	TOUCH_ONESHOT_UPGRADE

 	if(ts_check_need_upgrade(touch_dev->cap_info.chip_firmware_version)==TRUE)
   	{
   		printk(KERN_INFO "start upgrade firmware\n");
		ts_upgrade_firmware(touch_dev);
		mdelay(10);  

 #if 	SAIN_TS_SERIES
		// H/W CAlIBRATION --------------------------------------------->
		if (ts_write_cmd(touch_dev->client, SAIN_REST_CMD)!=I2C_SUCCESS)
			goto fail_init;
		if (ts_write_cmd(touch_dev->client, SAIN_WAKEUP_CMD)!=I2C_SUCCESS)
			goto fail_init;

		mdelay(2000);    
#else
		// h/w reset (cold reset)
		
#endif		

		// h/w calibration
		if (ts_write_reg(touch_dev->client, SAIN_TOUCH_MODE, 0x07)!=I2C_SUCCESS) goto fail_init;
		if (ts_write_cmd(touch_dev->client, SAIN_CALIBRATE_CMD)!=I2C_SUCCESS)
			goto fail_init;
		if (ts_write_cmd(touch_dev->client, SAIN_REST_CMD)!=I2C_SUCCESS)
			goto fail_init;
		
		mdelay(3000); 
		//<----------------------------------------------------------

		if (ts_write_reg(touch_dev->client, SAIN_TOUCH_MODE, 0x00)!=I2C_SUCCESS) goto fail_init;
		if (ts_write_cmd(touch_dev->client, SAIN_REST_CMD)!=I2C_SUCCESS)
			goto fail_init;
		/* get chip firmware version */
		if (ts_read_data(touch_dev->client, SAIN_FIRMWARE_VERSION, (u8*)&chip_firmware_version, 2)<0) goto fail_init;

		touch_dev->cap_info.chip_firmware_version = (u32)chip_firmware_version;
		printk(KERN_INFO "sain touch chip renewed firmware version = %x\r\n", touch_dev->cap_info.chip_firmware_version);
 
	  }
 #endif

#if 0 // calibration
        sain_debug_msg("calibration start", CurMaxX);
		// h/w calibration
	if (ts_write_reg(touch_dev->client, SAIN_TOUCH_MODE, 0x07)!=I2C_SUCCESS) goto fail_init;
	if (ts_write_cmd(touch_dev->client, SAIN_CALIBRATE_CMD)!=I2C_SUCCESS)
		goto fail_init;
	if (ts_write_cmd(touch_dev->client, SAIN_REST_CMD)!=I2C_SUCCESS)
		goto fail_init;
	mdelay(3000);
#endif	
	sain_debug_msg("calibration end", CurMaxX);
	/* initialize */	
	if (ts_write_reg(touch_dev->client, SAIN_X_RESOLUTION, (u16)(SetMaxX))!=I2C_SUCCESS) goto fail_init;
	if (ts_write_reg(touch_dev->client, SAIN_Y_RESOLUTION, (u16)(SetMaxY))!=I2C_SUCCESS) goto fail_init;
    
	if (ts_read_data(touch_dev->client, SAIN_X_RESOLUTION, (u8*)&CurMaxX, 2)<0) goto fail_init;
	sain_debug_msg("touch max x = %d\r\n", CurMaxX);
	if (ts_read_data(touch_dev->client, SAIN_Y_RESOLUTION, (u8*)&CurMaxY, 2)<0) goto fail_init;
	sain_debug_msg("touch min y = %d\r\n", CurMaxY);    

	touch_dev->cap_info.MinX = (u32)0;
	touch_dev->cap_info.MinY = (u32)0;
	touch_dev->cap_info.MaxX = (u32)CurMaxX;
	touch_dev->cap_info.MaxY = (u32)CurMaxY;
	
//	touch_dev->cap_info.Orientation = TOUCH_V_FLIP + TOUCH_H_FLIP;	 
	touch_dev->cap_info.Orientation = orient;	 	
	
	touch_dev->cap_info.gesture_support = 0;
    		
	sain_debug_msg("set other configuration\r\n");

	reg_val = HOLD_THRESHOLD;
	if (ts_write_reg(touch_dev->client, SAIN_HOLD_THRESHOLD, reg_val)!=I2C_SUCCESS)	goto fail_init;
	reg_val = REACTION_TIME;
	if (ts_write_reg(touch_dev->client, SAIN_REACTION_TIME, reg_val)!=I2C_SUCCESS)	goto fail_init;

#if 0	// use default values
	reg_val = PALM_REJECT_THRESHHOLD;
	if (ts_write_reg(touch_dev->client, SAIN_PALM_REJECT_THRESHHOLD, reg_val)!=I2C_SUCCESS)	goto fail_init;
	reg_val = NOISE_REJECT_THRESHHOLD;	
	if (ts_write_reg(touch_dev->client, SAIN_NOISE_REJECT_THRESHHOLD, reg_val)!=I2C_SUCCESS)	goto fail_init;
	reg_val = STYLUS_EDGE_COEF; 
	if (ts_write_reg(touch_dev->client, SAIN_STYLUS_EDGE_COEF, reg_val)!=I2C_SUCCESS)	goto fail_init;
	reg_val = STYLUS_HW_THRESHHOLD;
	if (ts_write_reg(touch_dev->client, SAIN_STYLUS_HW_THRESHHOLD, reg_val)!=I2C_SUCCESS)	goto fail_init;		
#endif

	reg_val = SENSITIVITY;
	if (ts_write_reg(touch_dev->client, SAIN_SENSITIVITY, reg_val)!=I2C_SUCCESS)	goto fail_init;	
	reg_val = TOUCH_MODE;
	if (ts_write_reg(touch_dev->client, SAIN_TOUCH_MODE, reg_val)!=I2C_SUCCESS)	goto fail_init;		

	// soft calibration
	if (ts_write_cmd(touch_dev->client, SAIN_CALIBRATE_CMD)!=I2C_SUCCESS)
		goto fail_init;


	reg_val = 0;
	sain_bit_set(reg_val, BIT_PT_CNT_CHANGE);			
	sain_bit_set(reg_val, BIT_DOWN);
	sain_bit_set(reg_val, BIT_MOVE);
	sain_bit_set(reg_val, BIT_UP);
	if(SUPPORTED_BUTTON_NUM > 0)
		sain_bit_set(reg_val, BIT_ICON_EVENT);

	
	touch_dev->chip_int_mask = reg_val;

	if (ts_write_reg(touch_dev->client, SAIN_INT_ENABLE_FLAG, touch_dev->chip_int_mask)!=I2C_SUCCESS)	goto fail_init;		


	//---------------------------------------------------------------------
	// read garbage data
	for(i=0; i<10; i++)
	{
		ts_write_cmd(touch_dev->client, SAIN_CLEAR_INT_STATUS_CMD);
	}
	sain_debug_msg("successfully initialized\r\n");
	return TRUE;
	
fail_init:
	sain_debug_msg("failed initiallize\r\n");
	if (ts_write_cmd(touch_dev->client, SAIN_REST_CMD)!=I2C_SUCCESS);
	return FALSE;

}

static int sain_touch_thread(void *pdata)
{
	sain_touch_dev *touch_dev = (sain_touch_dev*)pdata;   
	bool read_coord_continued;
	int i;
	u32 x[SUPPORTED_FINGER_NUM], y[SUPPORTED_FINGER_NUM], width[SUPPORTED_FINGER_NUM];
	u32 tmp;
	u8 finger_down[SUPPORTED_FINGER_NUM]={0,};
	u8 prev_finger_down[SUPPORTED_FINGER_NUM]={0,};
	u8 reported = FALSE;
	u8 prev_finger_cnt = 0;
	u8 loop_n = 0;

	printk(KERN_INFO "touch thread started.. \r\n");

	for (;;)
	{
		down(&touch_dev->update_lock);
//		sain_debug_msg("sain_touch_thread : semaphore signalled\r\n");
		read_coord_continued = TRUE;
		while(read_coord_continued)
		{
			prev_finger_cnt = 0;
			
			for(i=0; i<SUPPORTED_FINGER_NUM; i++)
			{
				prev_finger_down[i] = finger_down[i];
				if(prev_finger_down[i] == TRUE)	prev_finger_cnt++;					
			}
	        	
			if (ts_read_coord(touch_dev)==FALSE)
			{
				sain_debug_msg("couldn't read touch_dev sample\r\n");
				goto continue_read_samples;
			}

			if (touch_dev->is_valid_event == INVALID_EVENT)
			{
				sain_debug_msg(": invalid sample\r\n");
				goto continue_read_samples;
			}	

			reported = FALSE;


			for(i=0; i< SUPPORTED_BUTTON_NUM; i++)
			{
				if(touch_dev->button[i]  == ICON_BUTTON_DOWN)
				{
					printk("button %d down\n", i);					
					input_report_key(touch_dev->input_dev, BUTTON_MAPPING_KEY[i], 1);					
					reported = TRUE;
				}
				else if(touch_dev->button[i]  == ICON_BUTTON_UP)
				{
					printk("button %d up\n", i);
					input_report_key(touch_dev->input_dev, BUTTON_MAPPING_KEY[i], 0);
					reported = TRUE;
				}
			}


			if(touch_dev->prev_touch_info.finger_cnt > SUPPORTED_FINGER_NUM)
			{
				if(reported == TRUE)
				{
					input_sync(touch_dev->input_dev);	
				}
				goto continue_read_samples;
			}

			loop_n = (touch_dev->touch_info.finger_cnt > 0)?touch_dev->touch_info.finger_cnt:prev_finger_cnt;

			for (i = 0; i < loop_n; i++)
			{
				sain_debug_msg("finger %d down \r\n", i+1);

				x[i] = touch_dev->touch_info.coord[i].x;
				y[i] = touch_dev->touch_info.coord[i].y;
				width[i] = touch_dev->touch_info.coord[i].width;
				
				if(i < touch_dev->touch_info.finger_cnt)
					finger_down[i] = TRUE;
				else
					finger_down[i] = FALSE;
#if 1					
				 /* transformation from touch to screen orientation */
				if (touch_dev->cap_info.Orientation & TOUCH_V_FLIP)
				{
					y[i] = touch_dev->cap_info.MaxY + touch_dev->cap_info.MinY - y[i];			               
				}
				if (touch_dev->cap_info.Orientation & TOUCH_H_FLIP)
				{
					x[i] = touch_dev->cap_info.MaxX + touch_dev->cap_info.MinX - x[i];			               
				}
				if (touch_dev->cap_info.Orientation & TOUCH_XY_SWAP)
				{					
			                 swap_v(x[i], y[i], tmp);
				}
#else
                                {
                                     u16 new_x, new_y;

                                     new_x = 1023 - y[i];
                                     new_y =  x[i];

                                     x[i]=new_x;
                                     y[i]=new_y;
                                }
#endif
	 			       
			}

			for(;i<SUPPORTED_FINGER_NUM; i++)	finger_down[i] = 0;
		

			if(loop_n > 0)
			{
				input_report_abs(touch_dev->input_dev, ABS_X, x[0]);
		                input_report_abs(touch_dev->input_dev, ABS_Y, y[0]);
				sain_debug_msg("finger1 x = %d, y = %d \r\n", x[0], y[0]);
				input_report_abs(touch_dev->input_dev, ABS_PRESSURE, width[0]);
			}

			if(loop_n > 0 && touch_dev->touch_info.finger_cnt == 0)
			{
				input_report_key(touch_dev->input_dev, BTN_TOUCH, TRUE);
				input_sync(touch_dev->input_dev);
			}
			/* Report down or up flag */
			input_report_key(touch_dev->input_dev, BTN_TOUCH, finger_down[0]);

#if MULTI_TOUCH_SUPPORT			
						 	
			// report co-ordinates for the 2nd finger
			if (touch_dev->touch_info.finger_cnt >= 2)
			{
				input_report_abs(touch_dev->input_dev, ABS_HAT0X, x[1]); 
		               	input_report_abs(touch_dev->input_dev, ABS_HAT0Y, y[1]); 
				sain_debug_msg("finger2 x = %d, y = %d \r\n", x[1], y[1]);	               	
		               	input_report_key(touch_dev->input_dev, BTN_2, finger_down[1]);
			}
				
			if (touch_dev->touch_info.finger_cnt == 2)
			{
				if(prev_finger_down[2])
					input_report_key(touch_dev->input_dev, BTN_3, 0);
				if(prev_finger_down[3])
					input_report_key(touch_dev->input_dev, BTN_4, 0);
			}
				

			if (touch_dev->touch_info.finger_cnt >= 3)
			{
				input_report_abs(touch_dev->input_dev, ABS_HAT1X, x[2]); 
		               	input_report_abs(touch_dev->input_dev, ABS_HAT1Y, y[2]); 
				sain_debug_msg("finger3 x = %d, y = %d \r\n", x[2], y[2]);              				
		               	input_report_key(touch_dev->input_dev, BTN_3, finger_down[2]);
			}

			if (touch_dev->touch_info.finger_cnt == 2)
			{
				if(prev_finger_down[3])
					input_report_key(touch_dev->input_dev, BTN_4, 0);
			}


			if (touch_dev->touch_info.finger_cnt >= 4)
			{
				input_report_abs(touch_dev->input_dev, ABS_HAT2X, x[3]); 
		               	input_report_abs(touch_dev->input_dev, ABS_HAT2Y, y[3]); 
				sain_debug_msg("finger4 x = %d, y = %d \r\n", x[3], y[3]);
		               	input_report_key(touch_dev->input_dev, BTN_4, finger_down[3]);
			}			

#endif			
			input_sync(touch_dev->input_dev);
			//prev_finger_cnt = touch_dev->touch_info.finger_cnt;

		continue_read_samples:
			
			//check_interrupt_pin, if high, enable int & wait signal
			if (gpio_get_value(touch_dev->int_gpio_num))
			{
//			        sain_debug_msg("int pin is already high or have been changed to high\n");
				enable_irq(touch_dev->client->irq);
				read_coord_continued = FALSE;
			} 
			else
			{
//				sain_debug_msg("interrupt pin is still low, so continue read \r\n");			
			}
				
		}
	}

    return 0;
}

#if TOUCH_UPGRADE_MODULE

#define	TOUCH_UPGRADE_MINOR	150	// include/linux/miscdevices.h

#define	IOCTL_ERASE_FIRMWARE	_IO('u', 0x0)
#define	IOCTL_WRITE_FIRMWARE	_IOW('u', 0x01, _sain_upgrade_info)
#define	IOCTL_READ_FIRMWARE	_IOR('u', 0x02, _sain_upgrade_info)
#define	IOCTL_RESTART_TOUCH	_IO('u', 0x3)

static const struct file_operations sain_upgrade_fops = {
	.ioctl		= sain_upgrade_ioctl,
	.open		= sain_upgrade_open,
	.release	= sain_upgrade_release,
};

static struct miscdevice sain_upgrade_dev=
{
	.minor	= TOUCH_UPGRADE_MINOR,
	.name	= "sain_ts_upgrade",
	.fops	= &sain_upgrade_fops,
};

static int sain_upgrade_ioctl(struct inode *inode, struct file *file,
                        unsigned int cmd, unsigned long arg)
{

	_sain_upgrade_info	up_info;
	u8 *swap_addr;
	u8 ch;	

	if(m_upgrade_dev==NULL)
	{
		printk(KERN_ERR"sain_upgrade_ioctl : updage device == null!!\r\n");
		return -EACCES;
	}
	
	switch (cmd) {
	case IOCTL_ERASE_FIRMWARE:	/* Mask alarm int. enab. bit	*/
	{
		disable_irq(m_upgrade_dev->irq);
		// 0xaaaa is signature
		ts_write_reg(m_upgrade_dev->client,SAIN_ERASE_FIRMWARE, 0xaaaa);		
		while(!gpio_get_value(m_upgrade_dev->int_gpio_num));
		return 0;
	}
	case IOCTL_RESTART_TOUCH:	/* Allow alarm interrupts.	*/
	{
		disable_irq(m_upgrade_dev->irq);
		ts_init_touch(m_upgrade_dev);	
		enable_irq(m_upgrade_dev->irq);
		return 0;
	}
	case IOCTL_WRITE_FIRMWARE:	/* Mask watchdog int. enab. bit	*/
	{
		if (copy_from_user(&up_info, (_sain_upgrade_info *)arg, sizeof(_sain_upgrade_info)))
		{
			printk(KERN_ERR"sain_upgrade_ioctl : cannot read buffer for write updage command!!\r\n");
			return -EFAULT;
		}

		disable_irq(m_upgrade_dev->irq);

		// swap
		sain_debug_msg("sain_upgrade_ioctl : upgrade write address = %x\r\n", up_info.address);
		// address is big endian
		swap_addr = (u8*)&up_info.address;		
		ch = swap_addr[0];
		swap_addr[0] = swap_addr[1];
		swap_addr[1] = ch;

		if(ts_write_data(m_upgrade_dev->client,SAIN_WRITE_FIRMWARE,(char*)&up_info,sizeof(_sain_upgrade_info))<0)
		{
			printk(KERN_ERR"sain_upgrade_ioctl : cannot write updage data (addr = %x)!!\r\n", up_info.address);
		}	
		while(!gpio_get_value(m_upgrade_dev->int_gpio_num));

		return 0;
	}
	case IOCTL_READ_FIRMWARE:	/* Allow watchdog interrupts.	*/
	{
		_sain_upgrade_info * up_pt;

		if (copy_from_user(&up_info, (_sain_upgrade_info *)arg, sizeof(_sain_upgrade_info)))
		{
			printk(KERN_ERR"sain_upgrade_ioctl : cannot read buffer for write updage command!!\r\n");
			return -EFAULT;
		}
		sain_debug_msg("sain_upgrade_ioctl : upgrade read address = %x\r\n", up_info.address);
		// address is big endian		
		swap_addr = (u8*)&up_info.address;		
		ch = swap_addr[0];
		swap_addr[0] = swap_addr[1];
		swap_addr[1] = ch;

		disable_irq(m_upgrade_dev->irq);
		ts_write_reg(m_upgrade_dev->client,SAIN_READ_FIRMWARE, up_info.address);		
		
		if(! access_ok(VERIFY_WRITE, (_sain_upgrade_info *)arg, sizeof(_sain_upgrade_info)))
		{		
		 	return -EFAULT;
		}
		up_pt = (_sain_upgrade_info *)arg;
		if (ts_read_data (m_upgrade_dev->client, SAIN_READ_FIRMWARE, (u8*)(up_info.data), UPGRADE_DATA_SIZE)< 0)
		{
			sain_debug_msg("error read upgrade data.\r\n");
        		return FALSE;
		}
	
		return copy_to_user((void *)arg, (u8*)&up_info, sizeof(_sain_upgrade_info)) ? -EFAULT : 0;
	}
	

	return 0;
}

static int sain_upgrade_open(struct inode *inode, struct file *file)
{
	lock_kernel();

	unlock_kernel();
	return 0;

out_busy:

	unlock_kernel();
	return -EBUSY;
}

static int sain_upgrade_release(struct inode *inode, struct file *file)
{
	return 0;
}


#endif

static int sain_touch_probe(struct i2c_client *client, const struct i2c_device_id *i2c_id)
{
	int ret;
	sain_touch_dev* touch_dev;
	int i;

  	sain_debug_msg("sain_touch_probe+ \r\n");

  	sain_debug_msg("i2c check function \r\n");	
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk(KERN_ERR "error : not compatible i2c function \r\n");
		ret = -ENODEV;
		goto err_check_functionality;
	}

  	sain_debug_msg("touch data alloc \r\n");	
	touch_dev = kzalloc(sizeof(sain_touch_dev), GFP_KERNEL);
	if (!touch_dev) {
	  	printk(KERN_ERR "unabled to allocate touch data \r\n");	
		ret = -ENOMEM;
		goto err_alloc_dev_data;
	}
	touch_dev->client = client;
	i2c_set_clientdata(client, touch_dev);

	
    	sema_init(&touch_dev->update_lock, 0);

	if(I2C_SMBUS_BLOCK_MAX < sizeof(_ts_sain_point_info))
	{
		printk(KERN_WARNING "max size error : i2c max size = %d, sain packet size = %d \r\n", I2C_SMBUS_BLOCK_MAX, sizeof(_ts_sain_point_info));	
		printk(KERN_WARNING "must modify I2C_SMBUS_BLOCK_MAX field in include/linux/i2c.h\r\n");
	}
	

	sain_debug_msg("touch thread create \r\n");	
	touch_dev->task = kthread_create(sain_touch_thread, touch_dev, "sain_touch_thread");
	if(touch_dev->task == NULL)
	{
		printk(KERN_ERR "unabled to create touch thread \r\n");
		ret = -1;
		goto err_kthread_create_failed;
	}	
	//wake_up_process( touch_dev->task );
	sain_debug_msg("allocate input device \r\n");
	touch_dev->input_dev = input_allocate_device();
	if (touch_dev->input_dev == 0) {
		printk(KERN_ERR "unabled to allocate input device \r\n");
		ret = -ENOMEM;
		goto err_input_allocate_device;
	}

	//initialize sain touch ic
	touch_dev->int_gpio_num = GPIO_TOUCH_INT;	// for upgrade	
	ts_init_touch(touch_dev);
	
	touch_dev->input_dev->name = SAIN_DRIVER_NAME;
	touch_dev->input_dev->phys = "sain_touch/input0";	// <- for compatability
   	touch_dev->input_dev->id.bustype = BUS_HOST;
   	touch_dev->input_dev->id.vendor = 0x0001;

   	touch_dev->input_dev->id.product = 0x0002;
   	touch_dev->input_dev->id.version = 0x0100;   	
    
	set_bit(EV_SYN, touch_dev->input_dev->evbit);
	set_bit(EV_KEY, touch_dev->input_dev->evbit);
	set_bit(BTN_TOUCH, touch_dev->input_dev->keybit);
	set_bit(EV_ABS, touch_dev->input_dev->evbit);

	if(SUPPORTED_BUTTON_NUM > 0)
	{
		for(i=0; i< SUPPORTED_BUTTON_NUM; i++)	
	    		set_bit(BUTTON_MAPPING_KEY[i], touch_dev->input_dev->keybit);
	}	


	if (touch_dev->cap_info.Orientation & TOUCH_XY_SWAP)
	{

		input_set_abs_params(touch_dev->input_dev, ABS_Y, touch_dev->cap_info.MinX, touch_dev->cap_info.MaxX, 0, 0);
		input_set_abs_params(touch_dev->input_dev, ABS_X, touch_dev->cap_info.MinY, touch_dev->cap_info.MaxY, 0, 0);

		for(i=0; i<(SUPPORTED_FINGER_NUM-1)*2; i+=2)
		{
			input_set_abs_params(touch_dev->input_dev, ABS_HAT0X +i, touch_dev->cap_info.MinY, touch_dev->cap_info.MaxY, 0, 0);		
			input_set_abs_params(touch_dev->input_dev, ABS_HAT0Y +i+1, touch_dev->cap_info.MinX, touch_dev->cap_info.MaxX, 0, 0);
		}		
	}
	else
	{
		input_set_abs_params(touch_dev->input_dev, ABS_X, touch_dev->cap_info.MinX, touch_dev->cap_info.MaxX, 0, 0);
		input_set_abs_params(touch_dev->input_dev, ABS_Y, touch_dev->cap_info.MinY, touch_dev->cap_info.MaxY, 0, 0);
	
		for(i=0; i<(SUPPORTED_FINGER_NUM-1)*2; i+=2)
		{
			input_set_abs_params(touch_dev->input_dev, ABS_HAT0X +i, touch_dev->cap_info.MinX, touch_dev->cap_info.MaxX, 0, 0);		
			input_set_abs_params(touch_dev->input_dev, ABS_HAT0Y +i+1, touch_dev->cap_info.MinY, touch_dev->cap_info.MaxY, 0, 0);
		}		
	
	}
	
        input_set_abs_params(touch_dev->input_dev, ABS_PRESSURE, 0, 100, 0, 0);
        input_set_abs_params(touch_dev->input_dev, ABS_TOOL_WIDTH, 0, 255, 0, 0);	
	

	sain_debug_msg("register %s input device \r\n", touch_dev->input_dev->name);
	ret = input_register_device(touch_dev->input_dev);
	if(ret) {
		printk(KERN_ERR "unable to register %s input device\r\n", touch_dev->input_dev->name);
        	goto err_input_register_device;
	}
	
	touch_dev->int_gpio_num = GPIO_TOUCH_INT;

	sain_debug_msg("request irq (pin = %d) \r\n", touch_dev->int_gpio_num);
	
	if (touch_dev->client->irq) {
//		ret = request_irq(touch_dev->client->irq, ts_int_handler, IRQF_TRIGGER_LOW, SAIN_DRIVER_NAME, touch_dev);
                ret = request_irq(IRQ_TOUCH_INT, ts_int_handler, IRQF_TRIGGER_LOW, SAIN_DRIVER_NAME, touch_dev);
		if (ret) {
			printk(KERN_ERR "unable to register irq.(%s)\r\n", touch_dev->input_dev->name);
			goto err_request_irq;
		}
	}
	touch_dev->irq = touch_dev->client->irq;
	//enable_irq(touch_dev->client->irq);	// need debug
	dev_info(&client->dev, "sain touch probe.\r\n");
	wake_up_process( touch_dev->task );

#if TOUCH_UPGRADE_MODULE
	ret = misc_register(&sain_upgrade_dev);
	if (ret)
	{
		printk(KERN_ERR "cannot register upgrade device\r\n");
		m_upgrade_dev= NULL;
		return 0;
	}
	m_upgrade_dev = touch_dev;	
	printk(KERN_INFO "successfully registered upgrade device\r\n");
#endif
	
	return 0;

err_request_irq:
	input_unregister_device(touch_dev->input_dev);
err_input_register_device:
	input_free_device(touch_dev->input_dev);
err_kthread_create_failed:	
err_input_allocate_device:	
	kfree(touch_dev);
err_alloc_dev_data:	
err_check_functionality:

	
#if TOUCH_UPGRADE_MODULE	
	m_upgrade_dev= NULL;
#endif

	return ret;
}


static int sain_touch_remove(struct i2c_client *client)
{
	sain_touch_dev *touch_dev = i2c_get_clientdata(client);

	sain_debug_msg("sain_touch_remove+ \r\n");	

	if (touch_dev->client->irq) {
		free_irq(touch_dev->client->irq, touch_dev);
	}

	input_unregister_device(touch_dev->input_dev);
	input_free_device(touch_dev->input_dev);
	kfree(touch_dev);

#if TOUCH_UPGRADE_MODULE
	misc_deregister(&sain_upgrade_dev);
	m_upgrade_dev= NULL;
#endif
	return 0;
}

static int __devinit sain_touch_init(void)
{
        printk("%s\n", __FUNCTION__ );
	return i2c_add_driver(&sain_touch_driver);    
}

static void __exit sain_touch_exit(void)
{
	i2c_del_driver(&sain_touch_driver);
}

module_init(sain_touch_init);
module_exit(sain_touch_exit);

MODULE_DESCRIPTION("touch-screen device driver using i2c interface");
MODULE_AUTHOR("sohnet <swjang@sain.co.kr>");
MODULE_LICENSE("GPL");
