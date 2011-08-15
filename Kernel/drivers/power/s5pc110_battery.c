/*
 * linux/drivers/power/s3c6410_battery.c
 *
 * Battery measurement code for S3C6410 platform.
 *
 * based on palmtx_battery.c
 *
 * Copyright (C) 2009 Samsung Electronics.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */


#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/power_supply.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/irq.h>
#include <linux/wakelock.h>
#include <asm/mach-types.h>
#include <mach/hardware.h>
#include <mach/battery.h>
#include <plat/gpio-cfg.h>
#include <linux/earlysuspend.h>
#include <linux/io.h>
#include <mach/regs-clock.h>
#include <asm/gpio.h>

#include "s5pc110_battery.h"

static struct wake_lock vbus_wake_lock;
static struct wake_lock temp_wake_lock;
#if (defined __TEST_DEVICE_DRIVER__  || defined __ALWAYS_AWAKE_DEVICE__)
static struct wake_lock wake_lock_for_dev;
#endif /* __TEST_DEVICE_DRIVER__ || __ALWAYS_AWAKE_DEVICE__ */

#ifdef __FUEL_GAUGES_IC__
#include <linux/i2c.h>
#include "fuel_gauge.c"
#endif /* __FUEL_GAUGES_IC__ */

#ifdef __SMB136_CHARGER_IC__
#include <linux/i2c.h>
#include "smb136_charger.c"
#endif 


/*define IRQ */
#define IRQ_nCHG	IRQ_EINT9
#define IRQ_FUEL_ALRT	IRQ_EINT14

extern unsigned int HWREV;  // HWREV_MODE

#ifdef MAX17042
int fg_alert_init(void);
int fg_check_status_reg(void);
#endif

/* Prototypes */
extern void FSA9480_ChangePathToAudio(u8 enable);
//extern u8 FSA9480_Get_JIG_Status(void);
extern int s3c_adc_get_adc_data(int channel);
extern void MAX8998_IRQ_init(void);
extern void maxim_charging_control(unsigned int dev_type  , unsigned int cmd);
extern unsigned char maxim_vf_status(void);
//extern unsigned char maxim_lpm_chg_status(void);
unsigned char maxim_chg_status(void);
unsigned char maxim_charging_enable_status(void);
#ifdef __CHECK_BATTERY_V_F__
static unsigned int s3c_bat_check_v_f(void);
#endif

#define LPM_MODE


static ssize_t s3c_bat_show_property(struct device *dev,
                                      struct device_attribute *attr,
                                      char *buf);
static ssize_t s3c_bat_store(struct device *dev, 
			     struct device_attribute *attr,
			     const char *buf, size_t count);
static void s3c_set_chg_en(int enable);

static bool check_UV_charging_case(void);

#ifdef __TEST_DEVICE_DRIVER__
static ssize_t s3c_test_show_property(struct device *dev,
                                      struct device_attribute *attr,
                                      char *buf);
static ssize_t s3c_test_store(struct device *dev, 
			     struct device_attribute *attr,
			     const char *buf, size_t count);
static int bat_temper_state = 0;
#endif /* __TEST_DEVICE_DRIVER__ */

#ifdef __TEST_MODE_INTERFACE__
static struct power_supply *s3c_power_supplies_test = NULL;
static void polling_timer_func(unsigned long unused);
static void s3c_bat_status_update(struct power_supply *bat_ps);
#endif /* __TEST_MODE_INTERFACE__ */

#define TRUE	1
#define FALSE	0

#define ADC_DATA_ARR_SIZE	6
#define ADC_TOTAL_COUNT		10
#define POLLING_INTERVAL	2000
#ifdef __TEST_MODE_INTERFACE__
#define POLLING_INTERVAL_TEST	1000
#endif /* __TEST_MODE_INTERFACE__ */

#ifdef __BATTERY_COMPENSATION__
/* Offset Bit Value */
#define OFFSET_VIBRATOR_ON		(0x1 << 0)
#define OFFSET_CAMERA_ON		(0x1 << 1)
#define OFFSET_MP3_PLAY			(0x1 << 2)
#define OFFSET_VIDEO_PLAY		(0x1 << 3)
#define OFFSET_VOICE_CALL_2G		(0x1 << 4)
#define OFFSET_VOICE_CALL_3G		(0x1 << 5)
#define OFFSET_DATA_CALL		(0x1 << 6)
#define OFFSET_LCD_ON			(0x1 << 7)
#define OFFSET_TA_ATTACHED		(0x1 << 8)
#define OFFSET_CAM_FLASH		(0x1 << 9)
#define OFFSET_BOOTING			(0x1 << 10)
#else
#define OFFSET_VOICE_CALL_2G		(0x1 << 4)
#define OFFSET_VOICE_CALL_3G		(0x1 << 5)
#endif /* __BATTERY_COMPENSATION__ */

#ifdef __9BITS_RESOLUTION__
#define INVALID_VOL_ADC		20
#else /* __9BITS_RESOLUTION__ */
#define INVALID_VOL_ADC		160
#endif /* __9BITS_RESOLUTION__ */

static struct work_struct bat_work;
static struct work_struct cable_work;
static struct delayed_work fuelgauge_work;
static struct delayed_work fullcharging_work;
static struct delayed_work full_comp_work;
static struct device *dev;
static struct timer_list polling_timer;
static struct timer_list cable_timer;
static int s3c_battery_initial;
static int force_update;
static int full_charge_flag;
static int old_charging_source;
static int low_batt_boot_flag = 0;
static u32 is_recharging = 0;
int low_batt_comp_flag = 0;
int soc_restart_flag = 0;

#ifndef __FUEL_GAUGES_IC__
static int batt_max;
static int batt_full;
static int batt_safe_rech;
static int batt_almost;
static int batt_high;
static int batt_medium;
static int batt_low;
static int batt_critical;
static int batt_min;
static int batt_off;
#endif
#ifdef __ADJUST_RECHARGE_ADC__
static int batt_recharging;
#endif /* __ADJUST_RECHARGE_ADC__ */
#ifdef __BATTERY_COMPENSATION__
static int batt_compensation;
#endif /* __BATTERY_COMPENSATION__ */

static unsigned int start_time_msec;
static unsigned int total_time_msec;

#ifdef __BOARD_REV_ADC__
static int board_rev_adc;
static int is_end_board_rev_adc;
#endif /* __BOARD_REV_ADC__ */

static char *status_text[] = {
	[POWER_SUPPLY_STATUS_UNKNOWN] =		"Unknown",
	[POWER_SUPPLY_STATUS_CHARGING] =	"Charging",
	[POWER_SUPPLY_STATUS_DISCHARGING] =	"Discharging",
	[POWER_SUPPLY_STATUS_NOT_CHARGING] =	"Not Charging",
	[POWER_SUPPLY_STATUS_FULL] =		"Full",
};

int call_state = 0;

typedef enum {
	CHARGER_BATTERY = 0,
	CHARGER_USB,
	CHARGER_AC,
	CHARGER_DISCHARGE
} charger_type_t;

struct battery_info {
	u32 batt_id;		/* Battery ID from ADC */
	s32 batt_vol;		/* Battery voltage from ADC */
	s32 batt_vol_adc;	/* Battery ADC value */
	s32 batt_vol_adc_cal;	/* Battery ADC value (calibrated)*/
	s32 batt_temp;		/* Battery Temperature (C) from ADC */
	s32 batt_temp_adc;	/* Battery Temperature ADC value */
	s32 batt_temp_adc_cal;	/* Battery Temperature ADC value (calibrated) */
	s32 batt_current;	/* Battery current from ADC */
	u32 level;		/* formula */
	u32 charging_source;	/* 0: no cable, 1:usb, 2:AC */
	u32 charging_enabled;	/* 0: Disable, 1: Enable */
	u32 batt_health;	/* Battery Health (Authority) */
	u32 batt_is_full;       /* 0 : Not full 1: Full */
	u32 batt_is_recharging; /* 0 : Not recharging 1: Recharging */
	s32 batt_vol_adc_aver;	/* batt vol adc average */
	u32 batt_improper_ta; /* 1: improper ta */
#ifdef __TEST_MODE_INTERFACE__
	u32 batt_test_mode;	/* test mode */
	s32 batt_vol_aver;	/* batt vol average */
	s32 batt_temp_aver;	/* batt temp average */
	s32 batt_temp_adc_aver;	/* batt temp adc average */
	s32 batt_v_f_adc;	/* batt V_F adc */
        s32 batt_slate_mode; /* slate mode */
     
#endif /* __TEST_MODE_INTERFACE__ */
};

/* lock to protect the battery info */
static DEFINE_MUTEX(work_lock);

struct s3c_battery_info {
	int present;
	int polling;
	unsigned int polling_interval;
	unsigned int device_state;

	struct battery_info bat_info;
#ifdef LPM_MODE
	unsigned int charging_mode_booting;
#endif
};
static struct s3c_battery_info s3c_bat_info;

struct adc_sample_info {
	unsigned int cnt;
	int total_adc;
	int average_adc;
	int adc_arr[ADC_TOTAL_COUNT];
	int index;
};
#ifndef __FUEL_GAUGES_IC__
static struct adc_sample_info adc_sample[ENDOFADC];
#endif

struct battery_driver 
{
	struct early_suspend	early_suspend;
};
struct battery_driver *battery = NULL;


extern charging_device_type curent_device_type;

#ifdef LPM_MODE
void charging_mode_set(unsigned int val)
{
	s3c_bat_info.charging_mode_booting=val;
}
unsigned int charging_mode_get(void)
{
	return s3c_bat_info.charging_mode_booting;
}
#endif

static int get_usb_power_state(void)
{
	if(curent_device_type==PM_CHARGER_USB_INSERT)
		return 1;
	else
		return 0;
}	

static inline int s3c_adc_get_adc_data_ex(int channel) {
#ifndef __9BITS_RESOLUTION__
	return s3c_adc_get_adc_data(channel);
#else
	return s3c_adc_get_adc_data(channel) / 8;
#endif /* __9BITS_RESOLUTION__ */
}

#ifndef __FUEL_GAUGES_IC__
static unsigned long calculate_average_adc(adc_channel_type channel, int adc)
{
	unsigned int cnt = 0;
	int total_adc = 0;
	int average_adc = 0;
	int index = 0;

	cnt = adc_sample[channel].cnt;
	total_adc = adc_sample[channel].total_adc;

	if (adc < 0 || adc == 0) {
		dev_err(dev, "%s: invalid adc : %d\n", __func__, adc);
		adc = adc_sample[channel].average_adc;
	}

	if( cnt < ADC_TOTAL_COUNT ) {
		adc_sample[channel].adc_arr[cnt] = adc;
		adc_sample[channel].index = cnt;
		adc_sample[channel].cnt = ++cnt;

		total_adc += adc;
		average_adc = total_adc / cnt;
	} else {
#if 0
		if (channel == S3C_ADC_VOLTAGE &&
				!s3c_bat_info.bat_info.charging_enabled && 
				adc > adc_sample[channel].average_adc) {
			dev_dbg(dev, "%s: adc over avg : %d\n", __func__, adc);
			return adc_sample[channel].average_adc;
		}
#endif
		index = adc_sample[channel].index;
		if (++index >= ADC_TOTAL_COUNT)
			index = 0;

		total_adc = (total_adc - adc_sample[channel].adc_arr[index]) + adc;
		average_adc = total_adc / ADC_TOTAL_COUNT;

		adc_sample[channel].adc_arr[index] = adc;
		adc_sample[channel].index = index;
	}

	adc_sample[channel].total_adc = total_adc;
	adc_sample[channel].average_adc = average_adc;

	dev_dbg(dev, "%s: ch:%d adc=%d, avg_adc=%d\n",
			__func__, channel, adc, average_adc);
	return average_adc;
}
#endif /* __FUEL_GAUGES_IC__ */

static int s3c_bat_get_adc_data(adc_channel_type adc_ch)
{
	int adc_arr[ADC_DATA_ARR_SIZE];
	int adc_max = 0;
	int adc_min = 0;
	int adc_total = 0;
	int i;

	for (i = 0; i < ADC_DATA_ARR_SIZE; i++) {
		adc_arr[i] = s3c_adc_get_adc_data_ex(adc_ch);
		dev_dbg(dev, "%s: adc_arr = %d\n", __func__, adc_arr[i]);
		if ( i != 0) {
			if (adc_arr[i] > adc_max) 
				adc_max = adc_arr[i];
			else if (adc_arr[i] < adc_min)
				adc_min = adc_arr[i];
		} else {
			adc_max = adc_arr[0];
			adc_min = adc_arr[0];
		}
		adc_total += adc_arr[i];
	}

	dev_dbg(dev, "%s: adc_max = %d, adc_min = %d\n",
			__func__, adc_max, adc_min);
	return (adc_total - adc_max - adc_min) / (ADC_DATA_ARR_SIZE - 2);
}


#ifndef __FUEL_GAUGES_IC__
static unsigned long s3c_read_bat(struct power_supply *bat_ps)
{
	int adc = 0;
	static int cnt = 0;

	dev_dbg(dev, "%s\n", __func__);

	adc = s3c_bat_get_adc_data(S3C_ADC_VOLTAGE);
	dev_dbg(dev, "%s: adc = %d\n", __func__, adc);

#ifdef __BATTERY_COMPENSATION__
	adc += batt_compensation;
#endif /* __BATTERY_COMPENSATION__ */
	if (adc < s3c_bat_info.bat_info.batt_vol_adc_aver - INVALID_VOL_ADC
			&& cnt < 10) {
		dev_err(dev, "%s: invaild adc = %d\n", __func__, adc);
		adc = s3c_bat_info.bat_info.batt_vol_adc_aver;
		cnt++;
	} else {
		cnt = 0;
	}
	s3c_bat_info.bat_info.batt_vol_adc =  adc;

	return calculate_average_adc(S3C_ADC_VOLTAGE, adc);
}
#endif /* __FUEL_GAUGES_IC__ */

static unsigned long s3c_read_temp(struct power_supply *bat_ps)
{
	int adc = 0;

	dev_dbg(dev, "%s\n", __func__);
	if(HWREV >= 0x5)  // Rev0.5
		adc = s3c_bat_get_adc_data(ADC_TEMPERATURE);
	else
		adc=929;
	dev_dbg(dev, "%s: adc = %d\n", __func__, adc);

#ifdef __TEST_DEVICE_DRIVER__
	switch (bat_temper_state) {
	case 0:
		break;
	case 1:
		adc = TEMP_HIGH_BLOCK;
		break;
	case 2:
		adc = TEMP_LOW_BLOCK;
		break;
	default:
		break;
	}
#endif /* __TEST_DEVICE_DRIVER__ */

	s3c_bat_info.bat_info.batt_temp_adc = adc;

#if 0
	return calculate_average_adc(ADC_TEMPERATURE, adc);
#else
	return adc;
#endif
}

static int is_over_abs_time(void)
{
	  unsigned int total_time;
	  
	  if (!start_time_msec)
		  return 0;

	if (s3c_bat_info.bat_info.batt_is_recharging)
		total_time = TOTAL_RECHARGING_TIME;
	else
		total_time = TOTAL_CHARGING_TIME;
			
	if(time_after((unsigned long)jiffies, (unsigned long)(start_time_msec + total_time))) 
	{ 	 
		printk("%s abs time over (abs time: %u, start time: %u)\n",__func__, total_time, start_time_msec);
		return 1;
	}	
	else
		return 0;
}

#ifdef __BATTERY_COMPENSATION__
static void s3c_bat_set_compesation(int mode, 
				    int offset,
				    int compensate_value)
{
	if (mode) {
		if (!(s3c_bat_info.device_state & offset)) {
			s3c_bat_info.device_state |= offset;
			batt_compensation += compensate_value;
		}
	} else {
		if (s3c_bat_info.device_state & offset) {
			s3c_bat_info.device_state &= ~offset;
			batt_compensation -= compensate_value;
		}
	}
	dev_dbg(dev, "%s: device_state=0x%x, compensation=%d\n", __func__,
			s3c_bat_info.device_state, batt_compensation);
}
#endif /* __BATTERY_COMPENSATION__ */

#ifdef __CHECK_CHG_CURRENT__
static void check_chg_current(struct power_supply *bat_ps)
{
	static int cnt = 0;
	unsigned long chg_current = 0; 

	chg_current = s3c_bat_get_adc_data(ADC_CHG_CURRENT);
	s3c_bat_info.bat_info.batt_current = chg_current;
	if (chg_current <= CURRENT_OF_FULL_CHG) {
		cnt++;
		if (cnt >= 10) {
			dev_info(dev, "%s: battery full\n", __func__);
			s3c_set_chg_en(0);
			s3c_bat_info.bat_info.batt_is_full = 1;
			force_update = 1;
			full_charge_flag = 1;
			cnt = 0;
		}
	} else {
		cnt = 0;
	}
}
#endif /* __CHECK_CHG_CURRENT__ */

#ifdef __ADJUST_RECHARGE_ADC__
static void check_recharging_bat(int bat_vol)
{
	static int cnt = 0;

	if (s3c_bat_info.bat_info.batt_is_full && 
		!s3c_bat_info.bat_info.charging_enabled &&
		batt_recharging != -1 && bat_vol < batt_recharging) {
		if (++cnt >= 10) {
			dev_info(dev, "%s: recharging\n", __func__);
			s3c_bat_info.bat_info.batt_is_recharging = 1;
			s3c_set_chg_en(1);
			cnt = 0;
		}
	} else {
		cnt = 0;
	}
}
#endif /* __ADJUST_RECHARGE_ADC__ */

#ifndef __FUEL_GAUGES_IC__
#ifdef __ANDROID_BAT_LEVEL_CONCEPT__
static int s3c_get_bat_level(struct power_supply *bat_ps)
{
	int bat_level = 0;
	int bat_vol = s3c_read_bat(bat_ps);

	s3c_bat_info.bat_info.batt_vol_adc_aver = bat_vol;

	if(is_over_abs_time()) {
		bat_level = 100;
		s3c_bat_info.bat_info.batt_is_full = 1;
		dev_info(dev, "%s: charging time is over\n", __func__);
		s3c_set_chg_en(0);
		goto __end__;
	}

#ifdef __BATTERY_COMPENSATION__
	if (s3c_bat_info.bat_info.charging_enabled) {
		if (bat_vol > batt_almost - COMPENSATE_TA) {
			s3c_bat_set_compesation(0, OFFSET_TA_ATTACHED,
					COMPENSATE_TA);
		}
	}
#endif /* __BATTERY_COMPENSATION__ */

	if (bat_vol > batt_full) {
		int temp = (batt_max - batt_full) / 5;
		if (bat_vol > (batt_full + temp) || 
				s3c_bat_info.bat_info.batt_is_full)
			bat_level = 100;
		else
			bat_level = 90;

#ifdef __CHECK_CHG_CURRENT__
		if (s3c_bat_info.bat_info.charging_enabled) {
			check_chg_current(bat_ps);
			if (!s3c_bat_info.bat_info.batt_is_full)
				bat_level = 90;
		}
#endif /* __CHECK_CHG_CURRENT__ */
#ifdef __ADJUST_RECHARGE_ADC__
		check_recharging_bat(bat_vol);
#endif /* __ADJUST_RECHARGE_ADC__ */
		dev_dbg(dev, "%s: (full)level = %d\n", __func__, bat_level );
	} else if (batt_full >= bat_vol && bat_vol > batt_almost) {
		int temp = (batt_full - batt_almost) / 2;
		if (bat_vol > (batt_almost + temp))
			bat_level = 80;
		else
			bat_level = 70;

		if (s3c_bat_info.bat_info.batt_is_recharging)
			bat_level = 100;

		if (s3c_bat_info.bat_info.batt_is_full &&
			!s3c_bat_info.bat_info.charging_enabled) {
			dev_info(dev, "%s: recharging(under full)\n", __func__);
			s3c_bat_info.bat_info.batt_is_recharging = 1;
			s3c_set_chg_en(1);
			bat_level = 100;
		}
		dev_dbg(dev, "%s: (almost)level = %d\n", __func__, bat_level);
	} else if (batt_almost >= bat_vol && bat_vol > batt_high) {
		int temp = (batt_almost - batt_high) / 2;
		if (bat_vol > (batt_high + temp))
			bat_level = 60;
		else
			bat_level = 50;
		dev_dbg(dev, "%s: (high)level = %d\n", __func__, bat_level );
	} else if (batt_high >= bat_vol && bat_vol > batt_medium) {
		int temp = (batt_high - batt_medium) / 2;
		if (bat_vol > (batt_medium + temp))
			bat_level = 40;
		else
			bat_level = 30;
		dev_dbg(dev, "%s: (med)level = %d\n", __func__, bat_level);
	} else if (batt_medium >= bat_vol && bat_vol > batt_low) {
		bat_level = 15;
		dev_dbg(dev, "%s: (low)level = %d\n", __func__, bat_level);
	} else if (batt_low >= bat_vol && bat_vol > batt_critical) {
		bat_level = 5;
		dev_dbg(dev, "%s: (cri)level = %d, vol = %d\n", __func__,
				bat_level, bat_vol);
	} else if (batt_critical >= bat_vol && bat_vol > batt_min) {
		bat_level = 3;
		dev_info(dev, "%s: (min)level = %d, vol = %d\n", __func__,
				bat_level, bat_vol);
	} else if (batt_min >= bat_vol && bat_vol > batt_off) {
		bat_level = 1;
		dev_info(dev, "%s: (off)level = %d, vol = %d\n", __func__,
				bat_level, bat_vol);
	} else if (batt_off >= bat_vol)  {
		bat_level = 0;
		dev_info(dev, "%s: (off)level = %d, vol = %d", __func__,
				bat_level, bat_vol);
	}
	dev_dbg(dev, "%s: level = %d\n", __func__, bat_level);

__end__:
	dev_dbg(dev, "%s: bat_vol = %d, level = %d, is_full = %d\n",
			__func__, bat_vol, bat_level, 
			s3c_bat_info.bat_info.batt_is_full);
#ifdef __TEMP_ADC_VALUE__
	return 80;
#else
	return bat_level;
#endif /* __TEMP_ADC_VALUE__ */
}
#else /* __ANDROID_BAT_LEVEL_CONCEPT__ */
static int s3c_get_bat_level(struct power_supply *bat_ps)
{
	int bat_level = 0;
	int bat_vol = s3c_read_bat(bat_ps);

	s3c_bat_info.bat_info.batt_vol_adc_aver = bat_vol;

	if(is_over_abs_time()) {
		bat_level = 100;
		s3c_bat_info.bat_info.batt_is_full = 1;
		dev_info(dev, "%s: charging time is over\n", __func__);
		s3c_set_chg_en(0);
		goto __end__;
	}

#ifdef __BATTERY_COMPENSATION__
	if (s3c_bat_info.bat_info.charging_enabled) {
		if (bat_vol > batt_almost - COMPENSATE_TA) {
			s3c_bat_set_compesation(0, OFFSET_TA_ATTACHED,
					COMPENSATE_TA);
		}
	}
#endif /* __BATTERY_COMPENSATION__ */

	if (bat_vol > batt_full) {
		bat_level = 100;
#ifdef __CHECK_CHG_CURRENT__
		if (s3c_bat_info.bat_info.charging_enabled) {
			check_chg_current(bat_ps);
			if (!s3c_bat_info.bat_info.batt_is_full)
				bat_level = 90;
		}
#endif /* __CHECK_CHG_CURRENT__ */
#ifdef __ADJUST_RECHARGE_ADC__
		check_recharging_bat(bat_vol);
#endif /* __ADJUST_RECHARGE_ADC__ */
		dev_dbg(dev, "%s: (full)level = %d\n", __func__, bat_level );
	} else if (batt_full >= bat_vol && bat_vol > batt_almost) {
		int temp = (batt_full - batt_almost) / 3;
		if (bat_vol > (batt_almost + temp * 2))
			bat_level = 90;
		else if (bat_vol > (batt_almost + temp))
			bat_level = 80;
		else
			bat_level = 70;

		if (s3c_bat_info.bat_info.batt_is_recharging)
			bat_level = 100;

		if (s3c_bat_info.bat_info.batt_is_full &&
			!s3c_bat_info.bat_info.charging_enabled) {
			dev_info(dev, "%s: recharging(under full)\n", __func__);
			s3c_bat_info.bat_info.batt_is_recharging = 1;
			s3c_set_chg_en(1);
			bat_level = 100;
		}
		dev_dbg(dev, "%s: (almost)level = %d\n", __func__, bat_level);
	} else if (batt_almost >= bat_vol && bat_vol > batt_high) {
		bat_level = 50;
		dev_dbg(dev, "%s: (high)level = %d\n", __func__, bat_level );
	} else if (batt_high >= bat_vol && bat_vol > batt_medium) {
		bat_level = 30;
		dev_dbg(dev, "%s: (med)level = %d\n", __func__, bat_level);
	} else if (batt_medium >= bat_vol && bat_vol > batt_low) {
		bat_level = 15;
		dev_dbg(dev, "%s: (low)level = %d\n", __func__, bat_level);
	} else if (batt_low >= bat_vol && bat_vol > batt_critical) {
		bat_level = 5;
		dev_info(dev, "%s: (cri)level = %d, vol = %d\n", __func__,
				bat_level, bat_vol);
	} else if (batt_critical >= bat_vol && bat_vol > batt_min) {
		bat_level = 3;
		dev_info(dev, "%s: (min)level = %d, vol = %d\n", __func__,
				bat_level, bat_vol);
	} else if (batt_min >= bat_vol && bat_vol > batt_off) {
		bat_level = 1;
		dev_info(dev, "%s: (off)level = %d, vol = %d\n", __func__,
				bat_level, bat_vol);
	} else if (batt_off >= bat_vol)  {
		bat_level = 0;
		dev_info(dev, "%s: (off)level = %d, vol = %d", __func__,
				bat_level, bat_vol);
	}
	dev_dbg(dev, "%s: level = %d\n", __func__, bat_level);

__end__:
	dev_dbg(dev, "%s: bat_vol = %d, level = %d, is_full = %d\n",
			__func__, bat_vol, bat_level, 
			s3c_bat_info.bat_info.batt_is_full);
#ifdef __TEMP_ADC_VALUE__
	return 80;
#else
	return bat_level;
#endif /* __TEMP_ADC_VALUE__ */
}
#endif /* __ANDROID_BAT_LEVEL_CONCEPT__ */

static int s3c_get_bat_vol(struct power_supply *bat_ps)
{
	int bat_vol = 0;
	int adc = s3c_bat_info.bat_info.batt_vol_adc;
#ifdef __TEST_MODE_INTERFACE__
	int batt_vol_adc_aver = s3c_bat_info.bat_info.batt_vol_adc_aver;
#ifndef __9BITS_RESOLUTION__
	s3c_bat_info.bat_info.batt_vol_aver =
		(batt_vol_adc_aver - 2170 ) * 10 / 7 + 3200;
#else
	s3c_bat_info.bat_info.batt_vol_aver = batt_vol_adc_aver * 10000 / 848;
#endif /* __9BITS_RESOLUTION__ */
#endif /* __TEST_MODE_INTERFACE__ */

#ifndef __9BITS_RESOLUTION__
	bat_vol = ((adc / 100) * 10 - 217) * 100 / 7 + 3200;
#if defined(CONFIG_MACH_SPICA) 
	bat_vol = (((adc - 2170) * 10 / 7)/100)*100 + 3200;
#endif
#else
	bat_vol = adc * 100 / 848 * 100;
#endif /* __9BITS_RESOLUTION__ */

	dev_dbg(dev, "%s: adc = %d, bat_vol = %d\n",
			__func__, adc, bat_vol);

	return bat_vol;
}
#else /* __FUEL_GAUGES_IC__ */

#define LOW_BATT_COMP_RANGE_NUM	5
#define MAX_LOW_BATT_CHECK_CNT	10  /* 20 seconds */
static int check_start_vol = 0;
static int low_batt_comp_cnt[LOW_BATT_COMP_RANGE_NUM][2] = { {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0} };

static void display_low_batt_comp_cnt(void)
{
	u8 type_str[10];

	if(battery_type == SDI_BATTERY_TYPE)
		sprintf(type_str, "SDI");
	else if(battery_type == ATL_BATTERY_TYPE)
		sprintf(type_str, "ATL");
	else
		sprintf(type_str, "Unknown");

	printk("Check Array(%s) : [%d, %d], [%d, %d], [%d, %d], [%d, %d], [%d, %d]\n", type_str,
			low_batt_comp_cnt[0][0], low_batt_comp_cnt[0][1], low_batt_comp_cnt[1][0], low_batt_comp_cnt[1][1],
			low_batt_comp_cnt[2][0], low_batt_comp_cnt[2][1], low_batt_comp_cnt[3][0], low_batt_comp_cnt[3][1],
			low_batt_comp_cnt[4][0], low_batt_comp_cnt[4][1]);
}

static void add_low_batt_comp_cnt(int range, int level)
{
	int i = 0;
	int j = 0;

	// Increase the requested count value, and reset others.
	low_batt_comp_cnt[range-1][level/2] ++;

	for(i = 0; i < LOW_BATT_COMP_RANGE_NUM; i++)
	{
		for(j = 0; j < 2; j++)
		{
			if(i == range-1 && j == level/2)
				continue;  // keep the count value.
			else
				low_batt_comp_cnt[i][j] = 0;  // reset
		}
	}
}

static void reset_low_batt_comp_cnt(void)
{
	memset(low_batt_comp_cnt, 0x0, sizeof(low_batt_comp_cnt));
//	printk("%s : Reset check array count.\n", __func__);
}

static int check_low_batt_comp_condtion(int* nLevel)
{
	int i = 0;
	int j = 0;
	int ret = 0;

	for(i = 0; i < LOW_BATT_COMP_RANGE_NUM; i++)
		{
		for(j = 0; j < 2; j++)
		{
			if(low_batt_comp_cnt[i][j] >= MAX_LOW_BATT_CHECK_CNT)
			{
				display_low_batt_comp_cnt();

				ret = 1;
				*nLevel = j*2 + 1;  // 0->1%, 1->3%
				break;
			}
		}
	}

	return ret;
}

static int get_low_batt_threshold(int range, int level, int nCurrent)
{
	int ret = 0;

	if(battery_type == SDI_BATTERY_TYPE)
	{
		switch(range) {
		case 4:
			if(level == 1)
				ret = SDI_Range4_1_Offset + ((nCurrent * SDI_Range4_1_Slope) / 1000);
			else if(level == 3)
				ret = SDI_Range4_3_Offset + ((nCurrent * SDI_Range4_3_Slope) / 1000);
			break;
		
		case 3:
			if(level == 1)
				ret = SDI_Range3_1_Offset + ((nCurrent * SDI_Range3_1_Slope) / 1000);
			else if(level == 3)
				ret = SDI_Range3_3_Offset + ((nCurrent * SDI_Range3_3_Slope) / 1000);
			break;
		
		case 2:
			if(level == 1)
				ret = SDI_Range2_1_Offset + ((nCurrent * SDI_Range2_1_Slope) / 1000);
			else if(level == 3)
				ret = SDI_Range2_3_Offset + ((nCurrent * SDI_Range2_3_Slope) / 1000);
			break;
		
		case 1:
			if(level == 1)
				ret = SDI_Range1_1_Offset + ((nCurrent * SDI_Range1_1_Slope) / 1000);
			else if(level == 3)
				ret = SDI_Range1_3_Offset + ((nCurrent * SDI_Range1_3_Slope) / 1000);
			break;
		
		default:
			break;
		}
	}
	else if(battery_type == ATL_BATTERY_TYPE)
	{
		switch(range) {
		case 5:
			if(level == 1)
				ret = ATL_Range5_1_Offset + ((nCurrent * ATL_Range5_1_Slope) / 1000);
			else if(level == 3)
				ret = ATL_Range5_3_Offset + ((nCurrent * ATL_Range5_3_Slope) / 1000);
			break;

		case 4:
			if(level == 1)
				ret = ATL_Range4_1_Offset + ((nCurrent * ATL_Range4_1_Slope) / 1000);
			else if(level == 3)
				ret = ATL_Range4_3_Offset + ((nCurrent * ATL_Range4_3_Slope) / 100000);  // Slope value range is different
			break;
		
		case 3:
			if(level == 1)
				ret = ATL_Range3_1_Offset + ((nCurrent * ATL_Range3_1_Slope) / 1000);
			else if(level == 3)
				ret = ATL_Range3_3_Offset + ((nCurrent * ATL_Range3_3_Slope) / 1000);
			break;
		
		case 2:
			if(level == 1)
				ret = ATL_Range2_1_Offset + ((nCurrent * ATL_Range2_1_Slope) / 1000);
			else if(level == 3)
				ret = ATL_Range2_3_Offset + ((nCurrent * ATL_Range2_3_Slope) / 1000);
			break;
		
		case 1:
			if(level == 1)
				ret = ATL_Range1_1_Offset + ((nCurrent * ATL_Range1_1_Slope) / 1000);
			else if(level == 3)
				ret = ATL_Range1_3_Offset + ((nCurrent * ATL_Range1_3_Slope) / 1000);
			break;
		
		default:
			break;
		}
	}
	
//	printk("%s : Range%d, Level%d, Avg_Current(%d) -> Threshold(%d)\n", __func__, range, level, avg_current, ret);

	return ret;
}

static int s3c_low_batt_compensation(int fg_soc,int fg_vcell, int fg_current)
{
	int nRet=0;
	int fg_avg_current=0;
	int fg_min_current=0;
	int new_level = 0;
	int bCntReset=0;
	
	if(!check_start_vol)  // If not initialized yet, then init threshold values.
	{
		if(battery_type == SDI_BATTERY_TYPE)
			check_start_vol = 3550;  // Under 3.55V
		else if(battery_type == ATL_BATTERY_TYPE)
			check_start_vol = 3450;  // Under 3.45V
	}
	
	/////////////////////////////////////////////////////////////////
	if(!s3c_bat_info.bat_info.charging_enabled
		&& !low_batt_comp_flag
		&& (fg_vcell <= check_start_vol))  // Not charging, flag is none, Under 3.55V or 3.45V
	{
		fg_avg_current = fg_read_avg_current();
		fg_min_current = min(fg_avg_current, fg_current);
	
		if(battery_type == SDI_BATTERY_TYPE)
	{
			if(fg_min_current < -1250)  // I > 1.25A
		{		
				if(fg_soc >= 2 && fg_vcell < get_low_batt_threshold(4, 1, fg_min_current)) {  // 1%
					add_low_batt_comp_cnt(4, 1);
				}
				else if(fg_soc >= 4 && fg_vcell < get_low_batt_threshold(4, 3, fg_min_current)) {  // 3%
					add_low_batt_comp_cnt(4, 3);
				}
				else
				{
					bCntReset=1;
				}
			}
			else if(fg_min_current >= -1250 && fg_min_current < -750)  // 0.75A < I <= 1.25A
			{		
				if(fg_soc >= 2 && fg_vcell < get_low_batt_threshold(3, 1, fg_min_current)) {  // 1%
					add_low_batt_comp_cnt(3, 1);
				}
				else if(fg_soc >= 4 && fg_vcell < get_low_batt_threshold(3, 3, fg_min_current)) {  // 3%
					add_low_batt_comp_cnt(3, 3);
				}
				else
				{
					bCntReset=1;
				}
			}
			else if(fg_min_current >= -750 && fg_min_current < -100)  // 0.1A < I <= 0.75A
			{
				if(fg_soc >= 2 && fg_vcell < get_low_batt_threshold(2, 1, fg_min_current)) {  // 1%
					add_low_batt_comp_cnt(2, 1);
				}
				else if(fg_soc >= 4 && fg_vcell < get_low_batt_threshold(2, 3, fg_min_current)) {  // 3%
					add_low_batt_comp_cnt(2, 3);
				}
				else
				{
					bCntReset=1;
				}
			}
			else if(fg_min_current >= -100 && fg_min_current < 0)  // I <= 0.1A
			{
				if(fg_soc >= 2 && fg_vcell < get_low_batt_threshold(1, 1, fg_min_current)) {  // 1%
					add_low_batt_comp_cnt(1, 1);
				}
				else if(fg_soc >= 4 && fg_vcell < get_low_batt_threshold(1, 3, fg_min_current)) {  // 3%
					add_low_batt_comp_cnt(1, 3);
				}
				else
				{
					bCntReset=1;
				}
			}
		}
		else if(battery_type == ATL_BATTERY_TYPE)
		{
			if(fg_min_current < -1000)  // I > 1A
			{
				if(fg_soc >= 2 && fg_vcell < get_low_batt_threshold(5, 1, fg_min_current)) {  // 1%
					add_low_batt_comp_cnt(5, 1);
				}
				else if(fg_soc >= 4 && fg_vcell < get_low_batt_threshold(5, 3, fg_min_current)) {  // 3%
					add_low_batt_comp_cnt(5, 3);
				}
				else
				{
					bCntReset=1;
				}
			}
			else if(fg_min_current >= -1000 && fg_min_current < -700)  // 0.7A < I <= 1A
			{
				if(fg_soc >= 2 && fg_vcell < get_low_batt_threshold(4, 1, fg_min_current)) {  // 1%
					add_low_batt_comp_cnt(4, 1);
				}
				else if(fg_soc >= 4 && fg_vcell < get_low_batt_threshold(4, 3, fg_min_current)) {  // 3%
					add_low_batt_comp_cnt(4, 3);
				}
				else
				{
					bCntReset=1;
				}
			}
			else if(fg_min_current >= -700 && fg_min_current < -500)  // 0.5A < I <= 0.7A
			{		
				if(fg_soc >= 2 && fg_vcell < get_low_batt_threshold(3, 1, fg_min_current)) {  // 1%
				add_low_batt_comp_cnt(3, 1);
			}
				else if(fg_soc >= 4 && fg_vcell < get_low_batt_threshold(3, 3, fg_min_current)) {  // 3%
				add_low_batt_comp_cnt(3, 3);
				}
				else
				{
					bCntReset=1;
			}
		}
			else if(fg_min_current >= -500 && fg_min_current < -250)  // 0.25A < I <= 0.5A
		{
				if(fg_soc >= 2 && fg_vcell < get_low_batt_threshold(2, 1, fg_min_current)) {  // 1%
				add_low_batt_comp_cnt(2, 1);
			}
				else if(fg_soc >= 4 && fg_vcell < get_low_batt_threshold(2, 3, fg_min_current)) {  // 3%
				add_low_batt_comp_cnt(2, 3);
			}
				else
				{
					bCntReset=1;
		}
			}
			else if(fg_min_current >= -250 && fg_min_current < 0)  // I <= 0.25A
		{
				if(fg_soc >= 2 && fg_vcell < get_low_batt_threshold(1, 1, fg_min_current)) {  // 1%
				add_low_batt_comp_cnt(1, 1);
				}
				else if(fg_soc >= 4 && fg_vcell < get_low_batt_threshold(1, 3, fg_min_current)) {  // 3%
				add_low_batt_comp_cnt(1, 3);
				}
				else
				{
					bCntReset=1;
				}
			}
		}

		if(check_low_batt_comp_condtion(&new_level))
		{
			fg_low_batt_compensation(new_level);
			reset_low_batt_comp_cnt();
		}

		if (bCntReset)
			reset_low_batt_comp_cnt();

		// if compensation finished, then read SOC again!!
		if(low_batt_comp_flag)
		{
			printk("%s : MIN_CURRENT(%d), AVG_CURRENT(%d), CURRENT(%d), SOC(%d), VCELL(%d)\n",
				__func__, fg_min_current, fg_avg_current, fg_current, fg_soc, fg_vcell);
			fg_soc = fg_read_soc();
			printk("%s : SOC is set to %d\n", __func__, fg_soc);		
		}
	}
	/////////////////////////////////////////////////////////////////

	nRet=fg_soc;
	
	return nRet;
}


extern int fuel_guage_init;
static int chk_cnt = 0;
static int s3c_get_bat_level(struct power_supply *bat_ps)
{
	int fg_soc = -1;
	int fg_vcell = -1;
	int fg_current = 0;
	int recover_flag = 0;
	static int fg_skip = 0;
	static int skip_cnt = 0;

	static int cnt = 0;

	if(!fuel_guage_init)
		return 50;

/////////////////////////////////////////////////////////////////
	if(!(chk_cnt % 15))  // every 30 seconds.
		recover_flag = fg_check_cap_corruption();

	if(!(chk_cnt++ % 150)) {  // every 5 minutes
		fg_check_vf_fullcap_range();
		chk_cnt = 1;
	}

	if ((fg_soc = fg_read_soc()) < 0) {
		dev_err(dev, "%s: Can't read soc!!!\n", __func__);
		fg_soc = s3c_bat_info.bat_info.level;
	}

	if(FSA9480_Get_JIG_Status() || low_batt_comp_flag) {
		// Do nothing...
	}
	else {
		if( ((fg_soc+5) < prevRepSOC) || (fg_soc > (prevRepSOC+5)) )
			fg_skip = 1;
	}
 
//	printk("%s: fg_soc(%d), chk_cnt(%d), fg_skip(%d), recover_flag(%d)\n", __func__, fg_soc, chk_cnt, fg_skip, recover_flag);

	if(fg_skip) {  // skip one time (maximum 30 seconds) because of corruption.
		printk("%s: skip update until corruption check is done (skip_cnt:%d)\n", __func__, ++skip_cnt);
		fg_soc = s3c_bat_info.bat_info.level;
		if(recover_flag || skip_cnt > 150) {
			fg_skip = 0;
			skip_cnt = 0;
		}
	}
/////////////////////////////////////////////////////////////////

	if(low_batt_boot_flag) {
		fg_soc = 0;

		if(maxim_lpm_chg_status() && !check_UV_charging_case()) {
			fg_adjust_capacity();
			low_batt_boot_flag = 0;
		}

		if(!maxim_lpm_chg_status())
			low_batt_boot_flag = 0;
	}

	if ((fg_vcell = fg_read_vcell()) < 0) {
		dev_err(dev, "%s: Can't read vcell!!!\n", __func__);
		fg_vcell = s3c_bat_info.bat_info.batt_vol;
	} else
		s3c_bat_info.bat_info.batt_vol = fg_vcell;

	fg_current = fg_read_current();  // check battery current

	if(s3c_bat_info.bat_info.charging_source == CHARGER_AC && s3c_bat_info.bat_info.batt_improper_ta==0)
	{
		if (is_over_abs_time()) {
			fg_soc = 100;
			s3c_bat_info.bat_info.batt_is_full = 1;
			dev_info(dev, "%s: charging time is over\n", __func__);
			printk( "%s: fg_vcell = %d, fg_soc = %d, is_full = %d\n",
					__func__, fg_vcell, fg_soc, 
					s3c_bat_info.bat_info.batt_is_full);
			s3c_set_chg_en(0);
			goto __end__;
		}
	}

	if(fg_vcell <= RECHARGE_COND_VOLTAGE) {
		if (s3c_bat_info.bat_info.batt_is_full &&
			!s3c_bat_info.bat_info.charging_enabled) {
			if(++cnt >10)
			{
				dev_info(dev, "%s: recharging(under full)\n", __func__);
				s3c_bat_info.bat_info.batt_is_recharging = 1;
				s3c_set_chg_en(1);
				cnt=0;
				printk( "%s: fg_vcell = %d, fg_soc = %d, is_recharging = %d\n",
						__func__, fg_vcell, fg_soc, 
						s3c_bat_info.bat_info.batt_is_recharging);
				
			}
		}
		else
		{
			cnt=0;
		}	
	}
	else
	{
		cnt=0;
	}

	if (fg_soc > 100)
		fg_soc = 100;

	/*	Checks vcell level and tries to compensate SOC if needed.
	*/
	 if(!FSA9480_Get_JIG_Status())  // If jig cable is connected, then skip low batt compensation check.
		fg_soc=s3c_low_batt_compensation(fg_soc,fg_vcell,fg_current);

#ifdef __CHECK_CHG_CURRENT__
	if (fg_vcell >= FULL_CHARGE_COND_VOLTAGE) {
		if (s3c_bat_info.bat_info.charging_enabled) {
			check_chg_current(bat_ps);
			if (s3c_bat_info.bat_info.batt_is_full)
				fg_soc = 100;
		}
	}
#endif /* __CHECK_CHG_CURRENT__ */
#ifdef __ADJUST_RECHARGE_ADC__
	check_recharging_bat(fg_vcell);
#endif /* __ADJUST_RECHARGE_ADC__ */


__end__:
	dev_dbg(dev, "%s: fg_vcell = %d, fg_soc = %d, is_full = %d\n",
			__func__, fg_vcell, fg_soc, 
			s3c_bat_info.bat_info.batt_is_full);
	
	if (s3c_bat_info.bat_info.batt_is_full && (s3c_bat_info.bat_info.charging_source!=CHARGER_USB))
		fg_soc = 100;
	else
	{
		if(fg_soc >=100)
			fg_soc = 99;
	}

	return fg_soc;
}

static int s3c_get_bat_vol(struct power_supply *bat_ps)
{
	return s3c_bat_info.bat_info.batt_vol;
}

static int get_fg_average_vcell(void)
{
	int fg_vcell = -1;
	int i = 0;
	int min, max;
	int total = 0;

	if(!fuel_guage_init)
		return 4000;

	for(i=0; i<10; i++)
	{
		fg_vcell = fg_read_vcell();
//		printk("%s: vcell(%d)\n", __func__, fg_vcell);
		if(i == 0) {  // reset min, max
			min = fg_vcell;
			max = fg_vcell;
		}

		if(fg_vcell <= min)
			min = fg_vcell;

		if(fg_vcell >= max)
			max = fg_vcell;

		total += fg_vcell;

		mdelay(50);
	}

	return ( (total - (min + max)) / 8);	
}
#endif /* __FUEL_GAUGES_IC__ */

#ifdef __BOARD_REV_ADC__
static void s3c_get_board_rev_adc(struct power_supply *bat_ps)
{
	static unsigned int cnt = 0;
	int adc = 0;

	adc = s3c_bat_get_adc_data(S3C_ADC_BOARD_REV);
	board_rev_adc = calculate_average_adc(S3C_ADC_BOARD_REV, adc);
	dev_dbg(dev, "%s: adc=%d, avg_adc=%d\n", __func__, adc, board_rev_adc);

	if (++cnt >= 10)
		is_end_board_rev_adc = 1;
}
#endif /* __BOARD_REV_ADC__ */

#if (defined __TEST_MODE_INTERFACE__ && defined __BATTERY_V_F_ADC__)
static void s3c_get_v_f_adc(void)
{
	s3c_bat_info.bat_info.batt_v_f_adc
		= s3c_bat_get_adc_data(ADC_BATTERY_V_F);
	dev_info(dev, "%s: vf=%d\n", __func__,
			s3c_bat_info.bat_info.batt_v_f_adc);
}
#endif /* __TEST_MODE_INTERFACE__ && __BATTERY_V_F_ADC__ */

static u32 s3c_get_bat_health(void)
{
	return s3c_bat_info.bat_info.batt_health;
}

static void s3c_set_bat_health(u32 batt_health)
{
	s3c_bat_info.bat_info.batt_health = batt_health;
}

static inline int gpio_get_value_ex(unsigned int pin)
{
	return gpio_get_value(pin);
}

static inline void gpio_set_value_ex(unsigned int pin, unsigned int level)
{
	gpio_set_value(pin, level);
}


static bool check_jig_cable(void)
{
	int acc_id, acc_vol = 0;
	int check_jig = 1;
	int i = 0;

	for(i=0;i<3;i++)
	{
		acc_id = s3c_bat_get_adc_data(ADC_ACCESSORY_ID);
		acc_vol = acc_id* 3300 / 4095;

		printk("%s: acc_vol = %d !!\n", __func__, acc_vol);

		if(acc_vol<2600 || acc_vol > 2900)
		{
			check_jig=0;
			break;
		}
	}	
	return check_jig;
}


static bool check_samsung_charger(void)
{
	//1.Read ADC value
	int adc_1, adc_2, vol_1, vol_2;
	int i = 0;

	adc_1 = adc_2 = vol_1 = vol_2 = 0;

	FSA9480_ChangePathToAudio(TRUE);

	for ( i=0; i<3; i++)
	{
		adc_1 = s3c_bat_get_adc_data(ADC_AP_CHECK_1);
		adc_2 = s3c_bat_get_adc_data(ADC_AP_CHECK_2);

		//2. Change ADC value to Voltage
		vol_1= adc_1* 3300 / 4095;
		vol_2= adc_2* 3300 / 4095;
		printk("%s: vol_1 = %d, vol_2 = %d!!\n", __func__, vol_1, vol_2);

		//3. Check range of the voltage
		if( (vol_1 < 800) || (vol_1 > 1470) || (vol_2 < 800) || (vol_2 > 1470) )
		{
			FSA9480_ChangePathToAudio(FALSE);			
			return FALSE;
		}	
	}
	
	FSA9480_ChangePathToAudio(FALSE);
	return TRUE;
}


static bool check_UV_charging_case(void)
{
	int fg_vcell = fg_read_vcell();
	int fg_current = fg_read_current();
	int threshold = 0;

	if(battery_type == SDI_BATTERY_TYPE)
		threshold = 3300 + ((fg_current * 17) / 100);
	else if(battery_type == ATL_BATTERY_TYPE)
		threshold = 3300 + ((fg_current * 13) / 100);

	if(fg_vcell <= threshold)
		return TRUE;
	else
		return FALSE;
}


static void s3c_set_time_for_charging(int mode) {
	if (mode) {
		/* record start time for abs timer */
		start_time_msec = jiffies;
		dev_info(dev, "%s: start_time(%u)\n", __func__,
				start_time_msec);
	} else {
		/* initialize start time for abs timer */
		start_time_msec = 0;
		total_time_msec = 0;
		dev_info(dev, "%s: reset abs timer\n", __func__);
	}
}


static void s3c_set_chg_en(int enable)
{
	//int chg_en_val = gpio_get_value_ex(gpio_chg_en);
	int chg_en_val = maxim_chg_status();

	if (enable) {
		//if (chg_en_val == GPIO_LEVEL_HIGH) {
		if (chg_en_val) {
			//gpio_set_value_ex(gpio_chg_en, GPIO_LEVEL_LOW);
			if(curent_device_type==PM_CHARGER_TA)
			{
#ifdef __EXTERNEL_CHARGER_IC__
				if(HWREV >= 0x4)  // Rev0.4
				{
					maxim_charging_control(PM_CHARGER_DEFAULT, FALSE);  // do not use MAX8998 charger

					if(check_samsung_charger() == TRUE)
					{
						printk("%s: samsung charger!!\n", __func__);
						if(HWREV >= 0x8)							
							smb136_charging(DEVICE_TA);  // Set current to High
						else
							gpio_set_value(GPIO_CURR_ADJ, 1);  // Set current to High
					}
					else
					{
						printk("%s: improper charger!!\n", __func__);
						s3c_bat_info.bat_info.batt_improper_ta = 1;
						if(HWREV >= 0x8)
							smb136_charging(DEVICE_USB);  // Set current to Low 				
						else	
							gpio_set_value(GPIO_CURR_ADJ, 0);  // Set current to Low
					}
					gpio_set_value(GPIO_TA_EN, 0);  // External charger enable
				}
				else  // Under Rev0.3
				{
					if(check_samsung_charger() == TRUE)
					{
						printk("%s: samsung charger!!\n", __func__);
						maxim_charging_control(PM_CHARGER_DEFAULT, FALSE);  // do not use MAX8998 charger
						gpio_set_value(GPIO_TA_EN, 0);  // External charger enable
					}
					else
					{
						printk("%s: improper charger!!\n", __func__);
						s3c_bat_info.bat_info.batt_improper_ta = 1;
						gpio_set_value(GPIO_TA_EN, 1);  // External charger disable
						maxim_charging_control(PM_CHARGER_USB_INSERT, TRUE);  // use PMIC charger
					}
				}
#else /* __EXTERNEL_CHARGER_IC__ */
				maxim_charging_control(PM_CHARGER_TA, TRUE);
#endif /* __EXTERNEL_CHARGER_IC__ */
			}
			else if (curent_device_type==PM_CHARGER_USB_INSERT)
			{
#ifdef __EXTERNEL_CHARGER_IC__
				if(HWREV >= 0x4)  // Rev0.4
				{
					maxim_charging_control(PM_CHARGER_DEFAULT, FALSE);	 // do not use MAX8998 charger
					if(HWREV >=0x8)
					{
						if(check_jig_cable()==TRUE)
						{
							printk("%s: Jig cable insert!! Start high current charging~\n", __func__);
							smb136_charging(DEVICE_TA);  // Set current to Low
						}	
						else
							smb136_charging(DEVICE_USB);  // Set current to Low
					}
					else	
						gpio_set_value(GPIO_CURR_ADJ, 0);  // Set current to Low
					gpio_set_value(GPIO_TA_EN, 0);  // External charger enable
				}
				else  // Under Rev0.4
				{
					gpio_set_value(GPIO_TA_EN, 1);  // External charger disable
					maxim_charging_control(PM_CHARGER_USB_INSERT, TRUE);  // enable MAX8998 charger
				}
#else /* __EXTERNEL_CHARGER_IC__ */
				maxim_charging_control(PM_CHARGER_USB_INSERT, TRUE);  // enable MAX8998 charger
#endif /* __EXTERNEL_CHARGER_IC__ */
			}
			else
			{
#ifdef __EXTERNEL_CHARGER_IC__
				gpio_set_value(GPIO_TA_EN, 1);  // External charger disable
#endif /* __EXTERNEL_CHARGER_IC__ */
				maxim_charging_control(PM_CHARGER_DEFAULT, FALSE);
				printk("%s: unknown charger!!\n", __func__);
			}
			if(HWREV >=0x8)
				smb136_test_read();

			dev_info(dev, "%s: gpio_chg_en(1)\n", __func__);
			s3c_set_time_for_charging(1);
#ifdef __BATTERY_COMPENSATION__
			s3c_bat_set_compesation(1, OFFSET_TA_ATTACHED,
					COMPENSATE_TA);
#endif /* __BATTERY_COMPENSATION__ */
		}
	} 
	else
	{
		maxim_charging_control(PM_CHARGER_DEFAULT, FALSE);
#ifdef __EXTERNEL_CHARGER_IC__
		gpio_set_value(GPIO_TA_EN, 1);
#endif /* __EXTERNEL_CHARGER_IC__ */
		dev_info(dev, "%s: gpio_chg_en(0)\n", __func__);
		s3c_set_time_for_charging(0);
		s3c_bat_info.bat_info.batt_is_recharging = 0;
		s3c_bat_info.bat_info.batt_improper_ta = 0;
#ifdef __BATTERY_COMPENSATION__
		s3c_bat_set_compesation(0, OFFSET_TA_ATTACHED,
				COMPENSATE_TA);
#endif /* __BATTERY_COMPENSATION__ */
	}

#if 1
	if (enable) {
		if(curent_device_type==PM_CHARGER_TA)	{
			if(HWREV >= 0x4)  {
				if(!smb136_is_charging_active() && !smb136_is_fullcharging())
				{
					s3c_bat_info.bat_info.batt_improper_ta = 1;
					printk("%s: charging not active!! improper charger!!\n", __func__);
				}
			}
		}
	}
#endif					
	
	s3c_bat_info.bat_info.charging_enabled = enable;
}

static void s3c_temp_control(int mode) {
	switch (mode) {
	case POWER_SUPPLY_HEALTH_GOOD:
		dev_info(dev, "%s: GOOD\n", __func__);
		s3c_set_bat_health(mode);
		break;
	case POWER_SUPPLY_HEALTH_OVERHEAT:
		dev_info(dev, "%s: OVERHEAT\n", __func__);
		s3c_set_bat_health(mode);
		break;
	case POWER_SUPPLY_HEALTH_FREEZE:
		dev_info(dev, "%s: FREEZE\n", __func__);
		s3c_set_bat_health(mode);
		break;
	default:
		break;
	}
	schedule_work(&cable_work);
}

static int s3c_get_bat_temp(struct power_supply *bat_ps)
{
	int temp = 0;
	int array_size = 0;
	int i = 0;
	int temp_adc = s3c_read_temp(bat_ps);
	int health = s3c_get_bat_health();
	int update=0;
	s32 fuelgauge_temp=0;
#ifdef __BATTERY_COMPENSATION__
	unsigned int ex_case = 0;
#endif /* __BATTERY_COMPENSATION__ */
#ifdef __TEST_MODE_INTERFACE__
	s3c_bat_info.bat_info.batt_temp_adc_aver = temp_adc;
#endif /* __TEST_MODE_INTERFACE__ */

#ifdef __BATTERY_COMPENSATION__
	ex_case = OFFSET_MP3_PLAY | OFFSET_VOICE_CALL_2G | OFFSET_VOICE_CALL_3G
		| OFFSET_DATA_CALL | OFFSET_VIDEO_PLAY;
	if (s3c_bat_info.device_state & ex_case) {
		if (health == POWER_SUPPLY_HEALTH_OVERHEAT ||
				health == POWER_SUPPLY_HEALTH_FREEZE)
			s3c_temp_control(POWER_SUPPLY_HEALTH_GOOD);

		goto __map_temperature__;
	}
#endif /* __BATTERY_COMPENSATION__ */

#if 0 // charging block of thermistor temperature
	if (temp_adc <= TEMP_HIGH_BLOCK) {
		if (health != POWER_SUPPLY_HEALTH_OVERHEAT &&
				health != POWER_SUPPLY_HEALTH_UNSPEC_FAILURE)
		{
			s3c_temp_control(POWER_SUPPLY_HEALTH_OVERHEAT);			
			update=1;
		}
	} else if (temp_adc >= TEMP_HIGH_RECOVER &&
			temp_adc <= TEMP_LOW_RECOVER) {
		if (health == POWER_SUPPLY_HEALTH_OVERHEAT ||
				health == POWER_SUPPLY_HEALTH_FREEZE)
		{
			s3c_temp_control(POWER_SUPPLY_HEALTH_GOOD);
			update=1;
		}
	} else if (temp_adc >= TEMP_LOW_BLOCK) {
		if (health != POWER_SUPPLY_HEALTH_FREEZE &&
				health != POWER_SUPPLY_HEALTH_UNSPEC_FAILURE)
		{
			s3c_temp_control(POWER_SUPPLY_HEALTH_FREEZE);
			update=1;
		}
	}
#endif

	if(fg_check_battery_present())
		fuelgauge_temp = fg_read_batt_temp();
	else
		fuelgauge_temp=20000;

#ifdef __TEST_DEVICE_DRIVER__
		switch (bat_temper_state) {
		case 0:
			break;
		case 1:
			fuelgauge_temp = TEMP_FG_HIGH_BLOCK;
			break;
		case 2:
			fuelgauge_temp = TEMP_FG_LOW_BLOCK;
			break;
		default:
			break;
		}
#endif /* __TEST_DEVICE_DRIVER__ */

	if (fuelgauge_temp >= TEMP_FG_HIGH_BLOCK) {
		if (health != POWER_SUPPLY_HEALTH_OVERHEAT &&
				health != POWER_SUPPLY_HEALTH_UNSPEC_FAILURE)
		{
			s3c_temp_control(POWER_SUPPLY_HEALTH_OVERHEAT); 		
			update=1;
		}
	} else if (fuelgauge_temp <= TEMP_FG_HIGH_RECOVER &&
			fuelgauge_temp >= TEMP_FG_LOW_RECOVER) {
		if (health == POWER_SUPPLY_HEALTH_OVERHEAT ||
				health == POWER_SUPPLY_HEALTH_FREEZE)
		{
			s3c_temp_control(POWER_SUPPLY_HEALTH_GOOD);
			update=1;
		}
	} else if (fuelgauge_temp <= TEMP_FG_LOW_BLOCK) {
		if (health != POWER_SUPPLY_HEALTH_FREEZE &&
				health != POWER_SUPPLY_HEALTH_UNSPEC_FAILURE)
		{
			s3c_temp_control(POWER_SUPPLY_HEALTH_FREEZE);
			update=1;
		}
	}

#ifdef __BATTERY_COMPENSATION__
__map_temperature__:	
#endif /* __BATTERY_COMPENSATION__ */
	array_size = ARRAY_SIZE(temper_table);
	for (i = 0; i < (array_size - 1); i++) {
		if (i == 0) {
			if (temp_adc >= temper_table[0][0]) {
				temp = temper_table[0][1];
				break;
			} else if (temp_adc <= temper_table[array_size-1][0]) {
				temp = temper_table[array_size-1][1];
				break;
			}
		}

		if (temper_table[i][0] > temp_adc &&
				temper_table[i+1][0] <= temp_adc) {
			temp = temper_table[i+1][1];
		}
	}


	if(update)
		printk("%s:fuel_temp = %d, adc_temp = %d, adc = %d\n", __func__, fuelgauge_temp, temp/10, temp_adc);

#ifdef __TEST_MODE_INTERFACE__
       	s3c_bat_info.bat_info.batt_temp_aver = fuelgauge_temp/100;
#endif /* __TEST_MODE_INTERFACE__ */
	return fuelgauge_temp/100;
}
static int s3c_bat_get_charging_status(void)
{
        charger_type_t charger = CHARGER_BATTERY; 
        int ret = 0;
        charger = s3c_bat_info.bat_info.charging_source;
        
        switch (charger) {
        case CHARGER_BATTERY:
		ret = POWER_SUPPLY_STATUS_NOT_CHARGING;
		break;
	case CHARGER_USB:			
		if(s3c_bat_info.bat_info.batt_slate_mode){
		charger = CHARGER_BATTERY;
                ret = POWER_SUPPLY_STATUS_NOT_CHARGING;
                break;
	}
        case CHARGER_AC:
		if(s3c_bat_info.bat_info.batt_improper_ta)
			ret = POWER_SUPPLY_STATUS_NOT_CHARGING;
                else if (s3c_bat_info.bat_info.batt_is_full)
                        ret = POWER_SUPPLY_STATUS_FULL;
		else
                        ret = POWER_SUPPLY_STATUS_CHARGING;
                break;
	case CHARGER_DISCHARGE:
		ret = POWER_SUPPLY_STATUS_DISCHARGING;
		break;
        default:
                ret = POWER_SUPPLY_STATUS_UNKNOWN;
        }

	dev_dbg(dev, "%s: %s\n", __func__, status_text[ret]);
        return ret;
}



static int s3c_bat_get_property(struct power_supply *bat_ps, 
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	dev_dbg(bat_ps->dev, "%s: psp = %d\n", __func__, psp);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = s3c_bat_get_charging_status();
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = s3c_get_bat_health();
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = s3c_bat_info.present;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = s3c_bat_info.bat_info.level;
		dev_dbg(dev, "%s: level = %d\n", __func__, 
				val->intval);
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = s3c_bat_info.bat_info.batt_temp;
		dev_dbg(bat_ps->dev, "%s: temp = %d\n", __func__, 
				val->intval);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int s3c_power_get_property(struct power_supply *bat_ps, 
		enum power_supply_property psp, 
		union power_supply_propval *val)
{
       charger_type_t charger = CHARGER_BATTERY;
       charger = s3c_bat_info.bat_info.charging_source;
	
	dev_dbg(bat_ps->dev, "%s: psp = %d\n", __func__, psp);

	charger = s3c_bat_info.bat_info.charging_source;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		if (bat_ps->type == POWER_SUPPLY_TYPE_MAINS)
			val->intval = (charger == CHARGER_AC ? 1 : 0);
		else if (bat_ps->type == POWER_SUPPLY_TYPE_USB){
			if(s3c_bat_info.bat_info.batt_slate_mode){
			    charger = CHARGER_BATTERY;			
	                }
			val->intval = (charger == CHARGER_USB ? 1 : 0);
		}
		else
			val->intval = 0;
		break;
	default:
		return -EINVAL;
	}
	
	return 0;
}

#define SEC_BATTERY_ATTR(_name)								\
{											\
        .attr = { .name = #_name, .mode = S_IRUGO | S_IWUGO, .owner = THIS_MODULE },	\
        .show = s3c_bat_show_property,							\
        .store = s3c_bat_store,								\
}

static struct device_attribute s3c_battery_attrs[] = {
	SEC_BATTERY_ATTR(batt_vol),
	SEC_BATTERY_ATTR(batt_vol_adc),
	SEC_BATTERY_ATTR(batt_vol_adc_cal),
	SEC_BATTERY_ATTR(batt_temp),
	SEC_BATTERY_ATTR(batt_temp_adc),
	SEC_BATTERY_ATTR(batt_temp_adc_cal),
	SEC_BATTERY_ATTR(batt_vol_adc_aver),
#ifdef __BOARD_REV_ADC__
        SEC_BATTERY_ATTR(board_rev_adc),
#endif /* __BOARD_REV_ADC__ */
#ifdef __TEST_MODE_INTERFACE__
	/* test mode */
	SEC_BATTERY_ATTR(batt_test_mode),
	/* average */
	SEC_BATTERY_ATTR(batt_vol_aver),
	SEC_BATTERY_ATTR(batt_temp_aver),
	SEC_BATTERY_ATTR(batt_temp_adc_aver),
	SEC_BATTERY_ATTR(batt_v_f_adc),
#endif /* __TEST_MODE_INTERFACE__ */
#ifdef __CHECK_CHG_CURRENT__
	SEC_BATTERY_ATTR(batt_chg_current),
#endif /* __CHECK_CHG_CURRENT__ */
	SEC_BATTERY_ATTR(charging_source),
#ifdef __BATTERY_COMPENSATION__
	SEC_BATTERY_ATTR(vibrator),
	SEC_BATTERY_ATTR(camera),
	SEC_BATTERY_ATTR(mp3),
	SEC_BATTERY_ATTR(video),
	SEC_BATTERY_ATTR(talk_gsm),
	SEC_BATTERY_ATTR(talk_wcdma),
	SEC_BATTERY_ATTR(data_call),
	SEC_BATTERY_ATTR(device_state),
	SEC_BATTERY_ATTR(batt_compensation),
	SEC_BATTERY_ATTR(is_booting),
#else
	SEC_BATTERY_ATTR(talk_gsm),
	SEC_BATTERY_ATTR(talk_wcdma),
#endif /* __BATTERY_COMPENSATION__ */
#ifdef __FUEL_GAUGES_IC__
	SEC_BATTERY_ATTR(fg_soc),
	SEC_BATTERY_ATTR(fg_vsoc),
	SEC_BATTERY_ATTR(fg_vcell_aver),
	SEC_BATTERY_ATTR(fg_check),
	SEC_BATTERY_ATTR(reset_soc),
	SEC_BATTERY_ATTR(reset_cap),
	SEC_BATTERY_ATTR(fg_reg),
	SEC_BATTERY_ATTR(batt_type),
#endif /* __FUEL_GAUGES_IC__ */
#ifdef LPM_MODE
	SEC_BATTERY_ATTR(charging_mode_booting),
	SEC_BATTERY_ATTR(batt_temp_check),
	SEC_BATTERY_ATTR(batt_full_check),
#endif
 SEC_BATTERY_ATTR(batt_slate_mode),

};

enum {
        BATT_VOL = 0,
        BATT_VOL_ADC,
        BATT_VOL_ADC_CAL,
        BATT_TEMP,
        BATT_TEMP_ADC,
        BATT_TEMP_ADC_CAL,
	BATT_VOL_ADC_AVER,
#ifdef __BOARD_REV_ADC__
        BOARD_REV_ADC,
#endif /* __BOARD_REV_ADC__ */
#ifdef __TEST_MODE_INTERFACE__
	BATT_TEST_MODE,
	BATT_VOL_AVER,
	BATT_TEMP_AVER,
	BATT_TEMP_ADC_AVER,
	BATT_V_F_ADC,
#endif /* __TEST_MODE_INTERFACE__ */
#ifdef __CHECK_CHG_CURRENT__
	BATT_CHG_CURRENT,	
#endif /* __CHECK_CHG_CURRENT__ */
	BATT_CHARGING_SOURCE,
#ifdef __BATTERY_COMPENSATION__
	BATT_VIBRATOR,
	BATT_CAMERA,
	BATT_MP3,
	BATT_VIDEO,
	BATT_VOICE_CALL_2G,
	BATT_VOICE_CALL_3G,
	BATT_DATA_CALL,
	BATT_DEV_STATE,
	BATT_COMPENSATION,
	BATT_BOOTING,
#else
	BATT_VOICE_CALL_2G,
	BATT_VOICE_CALL_3G,
#endif /* __BATTERY_COMPENSATION__ */
#ifdef __FUEL_GAUGES_IC__
	BATT_FG_SOC,
	BATT_FG_VSOC,
	BATT_FG_VCELL_AVER,
	BATT_FG_CHECK,
	BATT_RESET_SOC,
	BATT_RESET_CAP,
	BATT_FG_REG,
	BATT_BATT_TYPE,
#endif /* __FUEL_GAUGES_IC__ */
#ifdef LPM_MODE
	CHARGING_MODE_BOOTING,
	BATT_TEMP_CHECK,
	BATT_FULL_CHECK,
#endif
        BATT_SLATE_MODE,
};

static int s3c_bat_create_attrs(struct device * dev)
{
        int i, rc;
        
        for (i = 0; i < ARRAY_SIZE(s3c_battery_attrs); i++) {
                rc = device_create_file(dev, &s3c_battery_attrs[i]);
                if (rc)
                        goto s3c_attrs_failed;
        }
        goto succeed;
        
s3c_attrs_failed:
        while (i--)
                device_remove_file(dev, &s3c_battery_attrs[i]);
succeed:        
        return rc;
}

static ssize_t s3c_bat_show_property(struct device *dev,
                                      struct device_attribute *attr,
                                      char *buf)
{
        int i = 0;
	u8 batt_str[5];
        const ptrdiff_t off = attr - s3c_battery_attrs;

        switch (off) {
        case BATT_VOL:
                i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
                               s3c_bat_info.bat_info.batt_vol);
                break;
        case BATT_VOL_ADC:
#ifndef __FUEL_GAUGES_IC__
		s3c_bat_info.bat_info.batt_vol_adc = 
			s3c_bat_get_adc_data(S3C_ADC_VOLTAGE);
#else
		s3c_bat_info.bat_info.batt_vol_adc = 0;
#endif /* __FUEL_GAUGES_IC__ */
                i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
                               s3c_bat_info.bat_info.batt_vol_adc);
                break;
        case BATT_VOL_ADC_CAL:
                i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
                               s3c_bat_info.bat_info.batt_vol_adc_cal);
                break;
        case BATT_TEMP:
                i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
                               s3c_bat_info.bat_info.batt_temp);
                break;
        case BATT_TEMP_ADC:
		s3c_bat_info.bat_info.batt_temp_adc = 
			s3c_bat_get_adc_data(ADC_TEMPERATURE);
                i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
                               s3c_bat_info.bat_info.batt_temp_adc);
                break;	
#ifdef __TEST_MODE_INTERFACE__
	case BATT_TEST_MODE:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
		s3c_bat_info.bat_info.batt_test_mode);
		break;
#endif /* __TEST_MODE_INTERFACE__ */
        case BATT_TEMP_ADC_CAL:
                i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
                               s3c_bat_info.bat_info.batt_temp_adc_cal);
                break;
        case BATT_VOL_ADC_AVER:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			s3c_bat_info.bat_info.batt_vol_adc_aver);
		break;
#ifdef __BOARD_REV_ADC__
        case BOARD_REV_ADC:
                i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", board_rev_adc);
                break;
#endif /* __BOARD_REV_ADC__ */
#ifdef __TEST_MODE_INTERFACE__
	case BATT_VOL_AVER:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			s3c_bat_info.bat_info.batt_vol_aver);
		break;
	case BATT_TEMP_AVER:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			s3c_bat_info.bat_info.batt_temp_aver);
		break;
	case BATT_TEMP_ADC_AVER:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			s3c_bat_info.bat_info.batt_temp_adc_aver);
		break;
	case BATT_V_F_ADC:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			s3c_bat_info.bat_info.batt_v_f_adc);
		break;
#endif /* __TEST_MODE_INTERFACE__ */
#ifdef __CHECK_CHG_CURRENT__
	case BATT_CHG_CURRENT:
		s3c_bat_info.bat_info.batt_current = 
			s3c_bat_get_adc_data(ADC_CHG_CURRENT);
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
				s3c_bat_info.bat_info.batt_current);
		break;
#endif /* __CHECK_CHG_CURRENT__ */
	case BATT_CHARGING_SOURCE:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			s3c_bat_info.bat_info.charging_source);
		break;
#ifdef __BATTERY_COMPENSATION__
	case BATT_DEV_STATE:
		i += scnprintf(buf + i, PAGE_SIZE - i, "0x%08x\n",
			s3c_bat_info.device_state);
		break;
	case BATT_COMPENSATION:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			batt_compensation);
		break;
#endif /* __BATTERY_COMPENSATION__ */
#ifdef __FUEL_GAUGES_IC__
	case BATT_FG_SOC:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			fg_read_soc());
		break;
	case BATT_FG_VSOC:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			fg_read_vfsoc());
		break;		
	case BATT_FG_VCELL_AVER:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			get_fg_average_vcell());
		break;
	case BATT_FG_CHECK:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			fg_check_chip_state());
		break;
	case BATT_BATT_TYPE:
		if(battery_type == SDI_BATTERY_TYPE)
			sprintf(batt_str, "SDI");
		else if(battery_type == ATL_BATTERY_TYPE)
			sprintf(batt_str, "ATL");
		i += scnprintf(buf + i, PAGE_SIZE - i, "%s_%s\n",
			batt_str, batt_str);
		break;
#endif /* __FUEL_GAUGES_IC__ */
#ifdef LPM_MODE
	case CHARGING_MODE_BOOTING:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			charging_mode_get());
		break;		
	case BATT_TEMP_CHECK:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			s3c_bat_info.bat_info.batt_health);
		break;		
	case BATT_FULL_CHECK:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			s3c_bat_info.bat_info.batt_is_full );
		break;			
#endif
        case BATT_SLATE_MODE:
			i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			s3c_bat_info.bat_info.batt_slate_mode);
			break;
        default:
                i = -EINVAL;
        }       
        
        return i;
}

#ifndef __FUEL_GAUGES_IC__
static void s3c_bat_set_vol_cal(int batt_cal)
{
	int max_cal = 4096;
#ifdef __9BITS_RESOLUTION__
	max_cal = 512;
#endif /* __9BITS_RESOLUTION__ */

	if (!batt_cal)
		return;

	if (batt_cal >= max_cal) {
		dev_err(dev, "%s: invalid battery_cal(%d)\n", __func__, batt_cal);
		return;
	}

	batt_max = batt_cal + BATT_MAXIMUM;
	batt_full = batt_cal + BATT_FULL;
	batt_safe_rech = batt_cal + BATT_SAFE_RECHARGE;
	batt_almost = batt_cal + BATT_ALMOST_FULL;
	batt_high = batt_cal + BATT_HIGH;
	batt_medium = batt_cal + BATT_MED;
	batt_low = batt_cal + BATT_LOW;
	batt_critical = batt_cal + BATT_CRITICAL;
	batt_min = batt_cal + BATT_MINIMUM;
	batt_off = batt_cal + BATT_OFF;
}
#endif

static ssize_t s3c_bat_store(struct device *dev, 
			     struct device_attribute *attr,
			     const char *buf, size_t count)
{
	int x = 0;
	int ret = 0;
	const ptrdiff_t off = attr - s3c_battery_attrs;

        switch (off) {
        case BATT_VOL_ADC_CAL:
#if 0 /* sone */
		if (sscanf(buf, "%d\n", &x) == 1) {
			s3c_bat_info.bat_info.batt_vol_adc_cal = x;
			s3c_bat_set_vol_cal(x);
			ret = count;
		}
		dev_info(dev, "%s: batt_vol_adc_cal = %d\n", __func__, x);
#endif
                break;
        case BATT_TEMP_ADC_CAL:
		if (sscanf(buf, "%d\n", &x) == 1) {
			s3c_bat_info.bat_info.batt_temp_adc_cal = x;
			ret = count;
		}
		dev_info(dev, "%s: batt_temp_adc_cal = %d\n", __func__, x);
                break;
#ifdef __TEST_MODE_INTERFACE__
	case BATT_TEST_MODE:
		if (sscanf(buf, "%d\n", &x) == 1) {
			s3c_bat_info.bat_info.batt_test_mode = x;
			ret = count;
		}
		
		if (s3c_bat_info.bat_info.batt_test_mode) {
			s3c_bat_info.polling_interval = POLLING_INTERVAL_TEST;			
			if (s3c_bat_info.polling) {
				del_timer_sync(&polling_timer);
			}
			mod_timer(&polling_timer, jiffies + 
				msecs_to_jiffies(s3c_bat_info.polling_interval));
			s3c_bat_status_update(&s3c_power_supplies_test[CHARGER_BATTERY]);
		} else {
			s3c_bat_info.polling_interval = POLLING_INTERVAL;		
			if (s3c_bat_info.polling) {
				del_timer_sync(&polling_timer);
			}
			mod_timer(&polling_timer,jiffies + 
				msecs_to_jiffies(s3c_bat_info.polling_interval));
			s3c_bat_status_update(&s3c_power_supplies_test[CHARGER_BATTERY]);
		}

#ifdef CONFIG_TARGET_LOCALE_KOR		
		/* 2010.08.06 
		   This was required by a charger verifier in Gumi.
		   He asked that Key LED should be turned off 
		   when BatteryStatus APP was running.

		   The number of GPIO depends on the target system.
		*/
		gpio_set_value(GPIO_KEY_LED_EN_R05, !s3c_bat_info.bat_info.batt_test_mode);
#endif		
				
		dev_info(dev, "%s: batt_test_mode = %d\n", __func__, x);
		break;
#endif /* __TEST_MODE_INTERFACE__ */
#ifdef __BATTERY_COMPENSATION__
	case BATT_VIBRATOR:
		if (sscanf(buf, "%d\n", &x) == 1) {
			s3c_bat_set_compesation(x, OFFSET_VIBRATOR_ON,
					COMPENSATE_VIBRATOR);
			ret = count;
		}
		dev_info(dev, "%s: vibrator = %d\n", __func__, x);
                break;
	case BATT_CAMERA:
		if (sscanf(buf, "%d\n", &x) == 1) {
			s3c_bat_set_compesation(x, OFFSET_CAMERA_ON,
					COMPENSATE_CAMERA);
			ret = count;
		}
		dev_info(dev, "%s: camera = %d\n", __func__, x);
                break;
	case BATT_MP3:
		if (sscanf(buf, "%d\n", &x) == 1) {
			s3c_bat_set_compesation(x, OFFSET_MP3_PLAY,
					COMPENSATE_MP3);
			ret = count;
		}
		dev_info(dev, "%s: mp3 = %d\n", __func__, x);
                break;
	case BATT_VIDEO:
		if (sscanf(buf, "%d\n", &x) == 1) {
			s3c_bat_set_compesation(x, OFFSET_VIDEO_PLAY,
					COMPENSATE_VIDEO);
			ret = count;
		}
		dev_info(dev, "%s: video = %d\n", __func__, x);
                break;
	case BATT_VOICE_CALL_2G:
		if (sscanf(buf, "%d\n", &x) == 1) {
			if(x)
				call_state |= OFFSET_VOICE_CALL_2G;
			else
				call_state &= ~OFFSET_VOICE_CALL_2G;
			s3c_bat_set_compesation(x, OFFSET_VOICE_CALL_2G,
					COMPENSATE_VOICE_CALL_2G);
			ret = count;
		}
		dev_info(dev, "%s: voice call 2G = %d\n", __func__, x);
                break;
	case BATT_VOICE_CALL_3G:
		if (sscanf(buf, "%d\n", &x) == 1) {
			if(x)
				call_state |= OFFSET_VOICE_CALL_3G;
			else
				call_state &= ~OFFSET_VOICE_CALL_3G;
			s3c_bat_set_compesation(x, OFFSET_VOICE_CALL_3G,
					COMPENSATE_VOICE_CALL_3G);
			ret = count;
		}
		dev_info(dev, "%s: voice call 3G = %d\n", __func__, x);
                break;
	case BATT_DATA_CALL:
		if (sscanf(buf, "%d\n", &x) == 1) {
			s3c_bat_set_compesation(x, OFFSET_DATA_CALL,
					COMPENSATE_DATA_CALL);
			ret = count;
		}
		dev_info(dev, "%s: data call = %d\n", __func__, x);
                break;
#ifdef COMPENSATE_BOOTING
	case BATT_BOOTING:
		if (sscanf(buf, "%d\n", &x) == 1) {
			s3c_bat_set_compesation(x, OFFSET_BOOTING,
					COMPENSATE_BOOTING);
			ret = count;
		}
		dev_info(dev, "%s: boot complete = %d\n", __func__, x);
                break;
#endif /* COMPENSATE_BOOTING */
#else /* __BATTERY_COMPENSATION__ */
	case BATT_VOICE_CALL_2G:
		if (sscanf(buf, "%d\n", &x) == 1) {
			if(x)
				call_state |= OFFSET_VOICE_CALL_2G;
			else
				call_state &= ~OFFSET_VOICE_CALL_2G;
			ret = count;
		}
		dev_info(dev, "%s: voice call 2G = %d\n", __func__, x);
                break;
	case BATT_VOICE_CALL_3G:
		if (sscanf(buf, "%d\n", &x) == 1) {
			if(x)
				call_state |= OFFSET_VOICE_CALL_3G;
			else
				call_state &= ~OFFSET_VOICE_CALL_3G;
			ret = count;
		}
		dev_info(dev, "%s: voice call 3G = %d\n", __func__, x);
                break;
#endif /* __BATTERY_COMPENSATION__ */
#ifdef __FUEL_GAUGES_IC__
	case BATT_RESET_SOC:
		if (sscanf(buf, "%d\n", &x) == 1) {
			if (x == 1)
				fg_reset_soc();
			ret = count;
		}
		dev_info(dev, "%s: Reset SOC:%d\n", __func__, x);
		break;
	case BATT_RESET_CAP:
		if (sscanf(buf, "%d\n", &x) == 1 || x==2 || x==3 || x==4) {
			if (x==1 || x== 2 || x==3 || x==4)
				fg_reset_capacity(x);
			ret = count;
		}
		dev_info(dev, "%s: Reset CAP:%d\n", __func__, x);
		break;
	case BATT_FG_REG:
		if (sscanf(buf, "%d\n", &x) == 1) {
			if (x == 1)
				fg_periodic_read();
			ret = count;
		}
		dev_info(dev, "%s: FG Register:%d\n", __func__, x);
		break;
#endif /* __FUEL_GAUGES_IC__ */
#ifdef LPM_MODE
	case CHARGING_MODE_BOOTING:
		if (sscanf(buf, "%d\n", &x) == 1) {
			charging_mode_set(x);
			ret = count;
		}
		dev_info(dev, "%s: CHARGING_MODE_BOOTING:%d\n", __func__, x);
		break;		
#endif
        case BATT_SLATE_MODE:
			if (sscanf(buf, "%d\n", &x) == 1) {
				s3c_bat_info.bat_info.batt_slate_mode = x;
				ret = count;
			}
				dev_info(dev, "%s: batt_slate_mode = %d\n", __func__, x);
			break;
        default:
                ret = -EINVAL;
        }       

	return ret;
}

#ifdef __BATTERY_COMPENSATION__
void s3c_bat_set_compensation_for_drv(int mode, int offset)
{
	switch(offset) {
	case OFFSET_VIBRATOR_ON:
		dev_dbg(dev, "%s: vibrator = %d\n", __func__, mode);
		s3c_bat_set_compesation(mode, offset, COMPENSATE_VIBRATOR);
		break;
	case OFFSET_LCD_ON:
		dev_info(dev, "%s: LCD On = %d\n", __func__, mode);
		s3c_bat_set_compesation(mode, offset, COMPENSATE_LCD);
		break;
	case OFFSET_CAM_FLASH:
		dev_info(dev, "%s: flash = %d\n", __func__, mode);
		s3c_bat_set_compesation(mode, offset, COMPENSATE_CAM_FALSH);
		break;
	default:
		break;
	}

}
EXPORT_SYMBOL(s3c_bat_set_compensation_for_drv);
#endif /* __BATTERY_COMPENSATION__ */

#ifdef __TEST_DEVICE_DRIVER__
#define SEC_TEST_ATTR(_name)								\
{											\
        .attr = { .name = #_name, .mode = S_IRUGO | S_IWUGO, .owner = THIS_MODULE },	\
        .show = s3c_test_show_property,							\
        .store = s3c_test_store,							\
}

static struct device_attribute s3c_test_attrs[] = {
	SEC_TEST_ATTR(suspend_lock),
	SEC_TEST_ATTR(control_tmp),
};

enum {
	SUSPEND_LOCK = 0,
	CTRL_TMP,
};

static int s3c_test_create_attrs(struct device * dev)
{
        int i, rc;
        
        for (i = 0; i < ARRAY_SIZE(s3c_test_attrs); i++) {
                rc = device_create_file(dev, &s3c_test_attrs[i]);
                if (rc)
                        goto s3c_attrs_failed;
        }
        goto succeed;
        
s3c_attrs_failed:
        while (i--)
                device_remove_file(dev, &s3c_test_attrs[i]);
succeed:        
        return rc;
}

static ssize_t s3c_test_show_property(struct device *dev,
                                      struct device_attribute *attr,
                                      char *buf)
{
	int i = 0;
	const ptrdiff_t off = attr - s3c_test_attrs;

	switch (off) {
//	case TEST_PM:  // example
//		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", 0);
//		s3c_test_pm();
//		break;
	default:
		i = -EINVAL;
	}       

	return i;
}

static ssize_t s3c_test_store(struct device *dev, 
			     struct device_attribute *attr,
			     const char *buf, size_t count)
{
	int mode = 0;
	int ret = 0;
	const ptrdiff_t off = attr - s3c_test_attrs;

	switch (off) {
	case SUSPEND_LOCK:
		if (sscanf(buf, "%d\n", &mode) == 1) {
			dev_dbg(dev, "%s: suspend_lock(%d)\n", __func__, mode);
			if (mode) 
				wake_lock(&wake_lock_for_dev);
			else
				wake_lock_timeout(&wake_lock_for_dev, HZ / 2);
			ret = count;
		}
		break;

	case CTRL_TMP:
		if (sscanf(buf, "%d\n", &mode) == 1) {
			dev_info(dev, "%s: control tmp(%d)\n", __func__, mode);
			bat_temper_state = mode;
			ret = count;
		}
		break;

	default:
		ret = -EINVAL;
	}       

	return ret;
}
#endif /* __TEST_DEVICE_DRIVER__ */

static enum power_supply_property s3c_battery_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CAPACITY,
};

static enum power_supply_property s3c_power_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static char *supply_list[] = {
	"battery",
};

static struct power_supply s3c_power_supplies[] = {
	{
		.name = "battery",
		.type = POWER_SUPPLY_TYPE_BATTERY,
		.properties = s3c_battery_properties,
		.num_properties = ARRAY_SIZE(s3c_battery_properties),
		.get_property = s3c_bat_get_property,
	},
	{
		.name = "usb",
		.type = POWER_SUPPLY_TYPE_USB,
		.supplied_to = supply_list,
		.num_supplicants = ARRAY_SIZE(supply_list),
		.properties = s3c_power_properties,
		.num_properties = ARRAY_SIZE(s3c_power_properties),
		.get_property = s3c_power_get_property,
	},
	{
		.name = "ac",
		.type = POWER_SUPPLY_TYPE_MAINS,
		.supplied_to = supply_list,
		.num_supplicants = ARRAY_SIZE(supply_list),
		.properties = s3c_power_properties,
		.num_properties = ARRAY_SIZE(s3c_power_properties),
		.get_property = s3c_power_get_property,
	},
};

static int s3c_cable_status_update(int status)
{
	int ret = 0;
	charger_type_t source = CHARGER_BATTERY;

	dev_dbg(dev, "%s\n", __func__);

	if(!s3c_battery_initial)
		return -EPERM;

	switch(status) {
	case CHARGER_BATTERY:
		dev_dbg(dev, "%s: cable NOT PRESENT\n", __func__);
		s3c_bat_info.bat_info.charging_source = CHARGER_BATTERY;
		break;
	case CHARGER_USB:
		dev_dbg(dev, "%s: cable USB\n", __func__);
		if(s3c_bat_info.bat_info.batt_slate_mode){
			s3c_bat_info.bat_info.charging_source = CHARGER_BATTERY;			
		}else{
		s3c_bat_info.bat_info.charging_source = CHARGER_USB;
		}	
		break;
	case CHARGER_AC:
		dev_dbg(dev, "%s: cable AC\n", __func__);
		s3c_bat_info.bat_info.charging_source = CHARGER_AC;
		break;
	case CHARGER_DISCHARGE:
		dev_dbg(dev, "%s: Discharge\n", __func__);
		s3c_bat_info.bat_info.charging_source = CHARGER_DISCHARGE;
		break;
	default:
		dev_err(dev, "%s: Nat supported status\n", __func__);
		ret = -EINVAL;
	}
	source = s3c_bat_info.bat_info.charging_source;

	if (source == CHARGER_USB || source == CHARGER_AC) {
		wake_lock(&vbus_wake_lock);
	} else {
		/* give userspace some time to see the uevent and update
		* LED state or whatnot...
		*/
		if(!maxim_chg_status()) {
			if(s3c_bat_info.charging_mode_booting)  // LPM
				wake_lock_timeout(&vbus_wake_lock, 2 * HZ);  // 2secs
			else
				wake_lock_timeout(&vbus_wake_lock, HZ / 2);  // 500ms
		}
	}
	/* if the power source changes, all power supplies may change state */
	power_supply_changed(&s3c_power_supplies[CHARGER_BATTERY]);
/*
	power_supply_changed(&s3c_power_supplies[CHARGER_USB]);
	power_supply_changed(&s3c_power_supplies[CHARGER_AC]);
*/
//	printk("%s: call power_supply_changed\n", __func__);
	return ret;
}



#ifdef CONFIG_TARGET_LOCALE_KOR 
static void s3c_bat_recheck_improper_TA(void)
{
	printk("%s: curent_device_type = %d \n", __func__,curent_device_type);
	if(curent_device_type==PM_CHARGER_TA && s3c_bat_info.bat_info.batt_improper_ta == 1)
	{
		printk("%s: rechecking Improper TA !!\n", __func__);

#ifdef __EXTERNEL_CHARGER_IC__
		if(HWREV >= 0x4)  // Rev0.4
		{
			if(check_samsung_charger() == TRUE)
			{
				printk("%s: samsung TA!!\n", __func__);
				s3c_bat_info.bat_info.batt_improper_ta = 0;
				if(HWREV >= 0x8)							
					smb136_charging(DEVICE_TA);  // Set current to High
				else
					gpio_set_value(GPIO_CURR_ADJ, 1);  // Set current to High
			}			
		}
		else  // Under Rev0.3
		{
			if(check_samsung_charger() == TRUE)
			{
				printk("%s: samsung TA!!\n", __func__);
				s3c_bat_info.bat_info.batt_improper_ta = 0;
				maxim_charging_control(PM_CHARGER_DEFAULT, FALSE);  // do not use MAX8998 charger
				gpio_set_value(GPIO_TA_EN, 0);  // External charger enable
			}			
		}
#else /* __EXTERNEL_CHARGER_IC__ */
		maxim_charging_control(PM_CHARGER_TA, TRUE);
#endif /* __EXTERNEL_CHARGER_IC__ */
	}
}
#endif

static void s3c_bat_status_update(struct power_supply *bat_ps)
{
	int old_level, old_temp, old_health, old_is_full;

#ifdef CONFIG_TARGET_LOCALE_KOR 
	static int nCnt=0;
	static bool bFirst=TRUE;
#endif

	dev_dbg(dev, "%s ++\n", __func__);

	if(!s3c_battery_initial)
		return;

	mutex_lock(&work_lock);
	old_temp = s3c_bat_info.bat_info.batt_temp;
	old_health = s3c_bat_info.bat_info.batt_health;
	old_level = s3c_bat_info.bat_info.level; 
	old_is_full = s3c_bat_info.bat_info.batt_is_full;
	s3c_bat_info.bat_info.batt_temp = s3c_get_bat_temp(bat_ps);

	s3c_bat_info.bat_info.level = s3c_get_bat_level(bat_ps);
	if (!s3c_bat_info.bat_info.charging_enabled &&
			!s3c_bat_info.bat_info.batt_is_full &&
			!FSA9480_Get_JIG_Status() ) {
		if (s3c_bat_info.bat_info.level > old_level)
			s3c_bat_info.bat_info.level = old_level;
	}
	s3c_bat_info.bat_info.batt_vol = s3c_get_bat_vol(bat_ps);

#ifdef __BOARD_REV_ADC__
	if (!is_end_board_rev_adc)
		s3c_get_board_rev_adc(bat_ps);
#endif /* __BOARD_REV_ADC__ */

#if (defined __TEST_MODE_INTERFACE__ && defined __BATTERY_V_F_ADC__)
	if (s3c_bat_info.bat_info.batt_test_mode == 1)
		s3c_get_v_f_adc();
#endif /* __TEST_MODE_INTERFACE__ && __BATTERY_V_F_ADC__ */
	if (old_level != s3c_bat_info.bat_info.level 
			|| (old_temp/10) != (s3c_bat_info.bat_info.batt_temp/10)
			|| old_is_full != s3c_bat_info.bat_info.batt_is_full
			|| old_charging_source != s3c_bat_info.bat_info.charging_source  // cable changed
			|| old_health != s3c_bat_info.bat_info.batt_health
			|| force_update) {
		old_charging_source = s3c_bat_info.bat_info.charging_source;
		force_update = 0;
		power_supply_changed(bat_ps);
//		printk("%s: call power_supply_changed\n", __func__);
	}

#ifdef CONFIG_TARGET_LOCALE_KOR 
	/* Try to recheck if it is an improper TA or not 
	when this function is called the fifth time
	It is related to a problem in which Samsung TA is recognized 
	as an improper TA on bootup time. */
	if (bFirst)
	{
		nCnt++;

		if (nCnt==5)
		{	
			bFirst=FALSE;
			s3c_bat_recheck_improper_TA();
		}
	}
#endif

	mutex_unlock(&work_lock);
	dev_dbg(dev, "%s --\n", __func__);
}

#ifdef __CHECK_BATTERY_V_F__
#if defined(__BATTERY_V_F_ADC__)
static unsigned int s3c_bat_check_v_f(void)
{
	unsigned int rc = 0;
	int adc = 0;

	s3c_get_v_f_adc();  // Get battery VF adc
	adc = s3c_bat_info.bat_info.batt_v_f_adc;

	dev_info(dev, "%s: V_F ADC = %d\n", __func__, adc);

	if (adc <= BATT_VF_MAX && adc >= BATT_VF_MIN) {
		/* s3c_set_bat_health(POWER_SUPPLY_HEALTH_GOOD); */
		rc = 1;
	} else {
		dev_info(dev, "%s: Unauthorized battery!\n", __func__);
		s3c_set_bat_health(POWER_SUPPLY_HEALTH_UNSPEC_FAILURE);
		rc = 0;
	}
	return rc;
}
#elif defined(__PMIC_HAS_V_F__)
static unsigned int s3c_bat_check_v_f(void)
{
	unsigned int ret = 0;
	if( maxim_vf_status() )	// bat detected
	{
		ret = 1;
	}
	else
	{
		dev_err(dev, "%s: VF error!\n", __func__);
		s3c_set_bat_health(POWER_SUPPLY_HEALTH_UNSPEC_FAILURE);
		ret = 0;
	}
	return ret;
}
#endif /* __PMIC_HAS_V_F__ */
#endif /* __CHECK_BATTERY_V_F__ */

static void s3c_cable_check_status(void)
{
 	charger_type_t status = 0;
	old_charging_source = s3c_bat_info.bat_info.charging_source;

	printk("%s: health(%d)\n", __func__, s3c_bat_info.bat_info.batt_health);

	mutex_lock(&work_lock);

	if (maxim_chg_status()) {
		if (s3c_get_bat_health() != POWER_SUPPLY_HEALTH_GOOD) {
			dev_info(dev, "%s: Unhealth battery state!\n", __func__);
			status = CHARGER_DISCHARGE;
			s3c_set_chg_en(0);
			goto __end__;
		}

		if (get_usb_power_state())
			status = CHARGER_USB;
		else
			status = CHARGER_AC;

		if((status == CHARGER_USB)&&(s3c_bat_info.bat_info.batt_slate_mode)){
			        status = CHARGER_BATTERY;
			        s3c_set_chg_en(0);					
		        } else
		s3c_set_chg_en(1);

		low_batt_comp_flag = 0;  // reset by charging (0904)
		reset_low_batt_comp_cnt();  // reset low_batt_comp_cnt array value.

		dev_info(dev, "%s: status : %s\n", __func__, 
				(status == CHARGER_USB) ? "USB" : "AC");
	} else {
		status = CHARGER_BATTERY;
		s3c_set_chg_en(0);
	}
__end__:
	//dev_info(dev, "%s: gpio_chg_en %s\n", __func__, 
	//	maxim_charging_enable_status()?"enabled":"disabled");
	s3c_cable_status_update(status);
	mutex_unlock(&work_lock);
}

static void s3c_bat_work(struct work_struct *work)
{
#ifdef __ADJUST_RECHARGE_ADC__
	static int pre_rechar = 0;
	static int cnt = 0;
	if (s3c_bat_info.bat_info.batt_is_full == 1 && full_charge_flag == 1) {
			batt_recharging = -1;
			pre_rechar = 1;
			full_charge_flag = 0;
	}

	if (pre_rechar) {
		if (++cnt > 10) {
#ifndef __FUEL_GAUGES_IC__
			int adc = s3c_bat_get_adc_data(S3C_ADC_VOLTAGE);
			batt_recharging = adc - BATT_RECHARGE_CODE;
#else /* __FUEL_GAUGES_IC__ */
			int fg_vcell = fg_read_vcell();
			batt_recharging = fg_vcell - BATT_RECHARGE_CODE;
#endif /* __FUEL_GAUGES_IC__ */
			dev_info(dev, "%s: batt_recharging=%d\n", __func__,
					batt_recharging);
			pre_rechar = 0;
			cnt = 0;
		}
	}
#endif /* __ADJUST_RECHARGE_ADC__ */

	dev_dbg(dev, "%s\n", __func__);

	s3c_bat_status_update(&s3c_power_supplies[CHARGER_BATTERY]);
}

static void s3c_cable_work(struct work_struct *work)
{
	//dev_info(dev, "%s\n", __func__);
	s3c_cable_check_status();
}

#ifdef CONFIG_PM
static int s3c_bat_suspend(struct platform_device *pdev, 
		pm_message_t state)
{
	dev_info(dev, "%s\n", __func__);

	if (s3c_bat_info.polling)
		del_timer_sync(&polling_timer);

	flush_scheduled_work();
#if 0 /* sone */
	disable_irq(IRQ_TA_CONNECTED_N);
	disable_irq(IRQ_TA_CHG_N);
#endif
	return 0;
}

static int s3c_bat_resume(struct platform_device *pdev)
{
	dev_info(dev, "%s\n", __func__);
//	wake_lock(&vbus_wake_lock);
#if 0 /* sone */
	enable_irq(IRQ_TA_CONNECTED_N);
	enable_irq(IRQ_TA_CHG_N);
#endif

	if(maxim_lpm_chg_status())
		wake_lock_timeout(&temp_wake_lock, 2 * HZ);

	schedule_work(&bat_work);
//	cable_timer.expires = jiffies + msecs_to_jiffies(50);
//	add_timer(&cable_timer);
/*
	if (timer_pending(&cable_timer))
		del_timer(&cable_timer);

	if(maxim_chg_status())
	{
		cable_timer.expires = jiffies + msecs_to_jiffies(50);
		add_timer(&cable_timer);
	}	
	else
		schedule_work(&cable_work);
*/
	if (s3c_bat_info.polling)
		mod_timer(&polling_timer,
			  jiffies + msecs_to_jiffies(s3c_bat_info.polling_interval));
	return 0;
}
#else
#define s3c_bat_suspend NULL
#define s3c_bat_resume NULL
#endif /* CONFIG_PM */

static void polling_timer_func(unsigned long unused)
{
	pr_debug("s3c_battery : %s\n", __func__);
	schedule_work(&bat_work);

	mod_timer(&polling_timer,
		  jiffies + msecs_to_jiffies(s3c_bat_info.polling_interval));
}

#if 0 /* sone */
static irqreturn_t s3c_cable_changed_isr(int irq, void *power_supply)
{
	//dev_info(dev, "%s: irq=0x%x, gpio_ta_connected=%x\n", __func__, irq, gpio_get_value(gpio_ta_connected));

	if (!s3c_battery_initial)
		return IRQ_HANDLED;

	s3c_bat_info.bat_info.batt_is_full = 0;
#ifdef __ADJUST_RECHARGE_ADC__
	batt_recharging = -1;
#endif /* __ADJUST_RECHARGE_ADC__ */

	schedule_work(&cable_work);
	/*
	 * Wait a bit before reading ac/usb line status and setting charger,
	 * because ac/usb status readings may lag from irq.
	 */
	mod_timer(&polling_timer,
		  jiffies + msecs_to_jiffies(s3c_bat_info.polling_interval));

	return IRQ_HANDLED;
}
#endif

extern int uUSB_check_finished;
static void cable_timer_func(unsigned long unused)
{
#if 1
	static int cnt = 0;
	dev_info(dev, "%s %d\n", __func__,cnt);

	if(uUSB_check_finished || cnt > 50)
	{
		printk("%s : uUSB_check_finished = %d\n",__func__,uUSB_check_finished);
		uUSB_check_finished =0;  // reset
		cnt =0;
		schedule_work(&cable_work);
		del_timer(&cable_timer);
	}
	else
	{
		cnt ++;
		cable_timer.expires = jiffies + msecs_to_jiffies(50);
		add_timer(&cable_timer);
	}
#else
	dev_info(dev, "%s\n", __func__);

	if(!uUSB_check_finished)  // USB cheching is not finished yey
	{
		cable_timer.expires = jiffies + msecs_to_jiffies(50);
		add_timer(&cable_timer);
	}
	else
	{
		uUSB_check_finished =0;  // reset
		schedule_work(&cable_work);
		del_timer(&cable_timer);
	}
#endif	
}

void s3c_cable_changed(void)
{
	//dev_info(dev, "%s: irq=0x%x, gpio_ta_connected=%x\n", __func__, irq,
	//		gpio_get_value(gpio_ta_connected));

	dev_info(dev, "%s: charger changed\n", __func__);

	if (!s3c_battery_initial)
		return ;

	if(!s3c_bat_info.charging_mode_booting)
		s3c_bat_info.bat_info.batt_is_full = 0;

	// Reset battery health to GOOD.
	s3c_bat_info.bat_info.batt_health = POWER_SUPPLY_HEALTH_GOOD;

#ifdef __ADJUST_RECHARGE_ADC__
	batt_recharging = -1;
#endif /* __ADJUST_RECHARGE_ADC__ */

	if (timer_pending(&cable_timer))
		del_timer(&cable_timer);

	if(maxim_chg_status())
	{
		cable_timer.expires = jiffies + msecs_to_jiffies(50);
		add_timer(&cable_timer);
	}	
	else
		schedule_work(&cable_work);
		
#ifdef CONFIG_TARGET_LOCALE_USAGSM 
	fg_test_read();
#endif

	/*
	 * Wait a bit before reading ac/usb line status and setting charger,
	 * because ac/usb status readings may lag from irq.
	 */
	mod_timer(&polling_timer,
		  jiffies + msecs_to_jiffies(s3c_bat_info.polling_interval));
}

#if 0 /* sone */
static irqreturn_t s3c_cable_charging_isr(int irq, void *power_supply)
{
	int chg_ing = gpio_get_value(gpio_chg_ing);
	dev_info(dev, "%s: irq=0x%x, gpio_chg_ing=%d\n", __func__, irq, chg_ing);

	if (!s3c_battery_initial)
		return IRQ_HANDLED;
#ifndef __DISABLE_CHG_ING_INTR__
	if (chg_ing && !gpio_get_value(gpio_ta_connected) &&
			s3c_bat_info.bat_info.charging_enabled &&
			s3c_get_bat_health() == POWER_SUPPLY_HEALTH_GOOD) {
		s3c_set_chg_en(0);
		s3c_bat_info.bat_info.batt_is_full = 1;
		force_update = 1;
		full_charge_flag = 1;
	}

	schedule_work(&bat_work);
	/*
	 * Wait a bit before reading ac/usb line status and setting charger,
	 * because ac/usb status readings may lag from irq.
	 */
	mod_timer(&polling_timer,
		  jiffies + msecs_to_jiffies(s3c_bat_info.polling_interval));
#endif /* __DISABLE_CHG_ING_INTR__ */

	return IRQ_HANDLED;
}
#endif

void s3c_cable_charging(void)
{
	is_recharging = s3c_bat_info.bat_info.batt_is_recharging;
	//int chg_ing = gpio_get_value(gpio_chg_ing);
	//dev_info(dev, "%s: irq=0x%x, gpio_chg_ing=%d\n", __func__, irq, chg_ing);

	if (!s3c_battery_initial)
		return ;

#ifndef __DISABLE_CHG_ING_INTR__
/*
	if (chg_ing && !gpio_get_value(gpio_ta_connected) &&
			s3c_bat_info.bat_info.charging_enabled &&
			s3c_get_bat_health() == POWER_SUPPLY_HEALTH_GOOD) {
*/
	if (s3c_bat_info.bat_info.charging_enabled &&
			s3c_get_bat_health() == POWER_SUPPLY_HEALTH_GOOD) {
		s3c_set_chg_en(0);
		s3c_bat_info.bat_info.batt_is_full = 1;
		force_update = 1;
		full_charge_flag = 1;

		// 2010.09.13 full charge compensation algorithm
		fg_fullcharged_compensation(is_recharging, 1);

		cancel_delayed_work(&full_comp_work);
		schedule_delayed_work(&full_comp_work, 100);  // after 1sec

		printk("%s battery is full charged\n",__func__);
	}

	schedule_work(&bat_work);
	/*
	 * Wait a bit before reading ac/usb line status and setting charger,
	 * because ac/usb status readings may lag from irq.
	 */
	mod_timer(&polling_timer,
		  jiffies + msecs_to_jiffies(s3c_bat_info.polling_interval));
#endif /* __DISABLE_CHG_ING_INTR__ */
}


extern int s3c_usb_cable(int connected);
static void battery_early_suspend(struct early_suspend *h)
{
	u32 con;

	/*hsmmc clock disable*/
	con = __raw_readl(S5P_CLKGATE_IP2);
	con &= ~0xF0000;
	__raw_writel(con, S5P_CLKGATE_IP2);

	/*usb clock disable*/
	s3c_usb_cable(0);

	return ;
}

static void battery_late_resume(struct early_suspend *h)
{
}


#ifdef MAX17042
static irqreturn_t low_battery_isr( int irq, void *_di )
{
	pr_info("%s: low battery isr\n", __func__);
	cancel_delayed_work( &fuelgauge_work );
	schedule_delayed_work( &fuelgauge_work, 0 );

	return IRQ_HANDLED;
}

int _low_battery_alarm_(void)
{	
	if(!maxim_chg_status())
	{
		pr_info("%s: soc = 0! Power Off!!\n", __func__);
		s3c_bat_info.bat_info.level=0;
		wake_lock_timeout( &vbus_wake_lock , HZ );
		power_supply_changed(&s3c_power_supplies[CHARGER_BATTERY]);
	}

	return 0;
}

static void fuelgauge_work_handler( struct work_struct *work )
{	
	pr_info("%s: low battery alert!\n", __func__);
	if(fg_check_status_reg())
		_low_battery_alarm_();
}
#endif

static void full_comp_work_handler( struct work_struct *work )
{
	int avg_current = fg_read_avg_current();

	if(avg_current >= 25) {  // Real threshold is 25.625mA
		cancel_delayed_work(&full_comp_work);
		schedule_delayed_work(&full_comp_work, 100);  // after 1sec
	}
	else {
		printk("%s : full charge compensation start (avg_current : %d)\n", __func__, avg_current);
		fg_fullcharged_compensation(is_recharging, 0);
	}
}

#ifdef __EXTERNEL_CHARGER_IC__
irqreturn_t ISL9220_STAT2_interrupt(int irq, void *ptr)
{
	cancel_delayed_work( &fullcharging_work );
	schedule_delayed_work( &fullcharging_work, 0 );
	
	return IRQ_HANDLED; 
}


#ifdef __SMB136_CHARGER_IC__
irqreturn_t SMB136_STAT_interrupt(int irq, void *ptr)
{
	cancel_delayed_work( &fullcharging_work );
	schedule_delayed_work( &fullcharging_work, 0 );

	return IRQ_HANDLED; 
}

void check_fullchaged(void)
{
	maxim_chg_status();
	if(curent_device_type==PM_CHARGER_TA && smb136_is_already_fullcharged()==1 && s3c_bat_info.bat_info.level >= 70)
	{	
		printk("%s: already fullcharged!!\n", __func__);
		s3c_cable_charging();
	}
}

#endif


static void fullcharging_work_handler(struct work_struct * work)
{
	unsigned int TA_nSTAT=0, TA_nCHG=0;
	int is_fullcharing=0;
	pr_info("nCHG rising intr!!\n");

	if(HWREV >= 0x8)
	{
		smb136_test_read();
		TA_nCHG = gpio_get_value_ex(GPIO_TA_nCHG);
		
		if( TA_nCHG==1 && maxim_chg_status()==1)
		{
			if(smb136_is_fullcharging()==1 && s3c_bat_info.bat_info.level >= 70)
			{
				is_fullcharing=1;
			}
		}
	}
	else
	{
		TA_nSTAT = gpio_get_value_ex(GPIO_TA_nSTAT);
		TA_nCHG = gpio_get_value_ex(GPIO_TA_nCHG);

		dev_info(dev,"TA_nSTAT = %d,  TA_nCHG = %d!!\n", TA_nSTAT,TA_nCHG );
		if(TA_nSTAT==0 && TA_nCHG==1)
			is_fullcharing=1;
	}

	if(is_fullcharing==1 &&curent_device_type != PM_CHARGER_NULL)
	{	
		s3c_cable_charging();
	}
}
#endif /* __EXTERNEL_CHARGER_IC__ */


static void s3c_bat_interrupt_init(void)
{	
#ifdef MAX17042
	dev_info(dev, "%s: low battery interrupt setting!\n", __func__);
	set_irq_type(IRQ_FUEL_ALRT, IRQ_TYPE_EDGE_FALLING);
	if(request_irq(IRQ_FUEL_ALRT, low_battery_isr, IRQF_DISABLED, "fg alert irq", NULL ))
		pr_err("%s: Can NOT request irq %d!\n", __func__, IRQ_FUEL_ALRT);
#endif

#ifdef __EXTERNEL_CHARGER_IC__
	if(HWREV >= 0x08)
	{
		set_irq_type(IRQ_nCHG, IRQ_TYPE_EDGE_RISING);
		if (request_irq(IRQ_nCHG, SMB136_STAT_interrupt, IRQF_DISABLED, "TA_nCHG intr", NULL)) 
			pr_err("%s: Can NOT request irq %d!\n", __func__, IRQ_nCHG);
	}	
	else	
	{
		set_irq_type(IRQ_nCHG, IRQ_TYPE_EDGE_RISING);
		if (request_irq(IRQ_nCHG, ISL9220_STAT2_interrupt, IRQF_DISABLED, "TA_nCHG intr", NULL)) 
			pr_err("%s: Can NOT request irq %d!\n", __func__, IRQ_nCHG);
	}	
#endif /* __EXTERNEL_CHARGER_IC__ */
}

static int __devinit s3c_bat_probe(struct platform_device *pdev)
{
	int i;
	int ret = 0;

	dev = &pdev->dev;
	dev_info(dev, "%s HWREV = 0x%x\n", __func__,HWREV);

	s3c_bat_info.present = 1;
	s3c_bat_info.polling = 1;
	s3c_bat_info.polling_interval = POLLING_INTERVAL;
	s3c_bat_info.device_state = 0;

	s3c_bat_info.bat_info.batt_vol_adc_aver = 0;
#ifdef __TEST_MODE_INTERFACE__
	s3c_bat_info.bat_info.batt_vol_aver = 0;
	s3c_bat_info.bat_info.batt_temp_aver = 0;
	s3c_bat_info.bat_info.batt_temp_adc_aver = 0;
	s3c_bat_info.bat_info.batt_v_f_adc = 0;

	s3c_bat_info.bat_info.batt_test_mode = 0;
 	s3c_power_supplies_test = s3c_power_supplies;
#endif /* __TEST_MODE_INTERFACE__ */
	s3c_bat_info.bat_info.batt_id = 0;
	s3c_bat_info.bat_info.batt_vol = 0;
	s3c_bat_info.bat_info.batt_vol_adc = 0;
	s3c_bat_info.bat_info.batt_vol_adc_cal = 0;
	s3c_bat_info.bat_info.batt_temp = 0;
	s3c_bat_info.bat_info.batt_temp_adc = 0;
	s3c_bat_info.bat_info.batt_temp_adc_cal = 0;
	s3c_bat_info.bat_info.batt_current = 0;
	s3c_bat_info.bat_info.level = 100;
	s3c_bat_info.bat_info.charging_source = CHARGER_BATTERY;
	s3c_bat_info.bat_info.charging_enabled = 0;
	s3c_bat_info.bat_info.batt_health = POWER_SUPPLY_HEALTH_GOOD;
        s3c_bat_info.bat_info.batt_slate_mode = 0;
#ifndef __FUEL_GAUGES_IC__
	memset(adc_sample, 0x00, sizeof adc_sample);

	batt_max = BATT_CAL + BATT_MAXIMUM;
	batt_full = BATT_CAL + BATT_FULL;
	batt_safe_rech = BATT_CAL + BATT_SAFE_RECHARGE;
	batt_almost = BATT_CAL + BATT_ALMOST_FULL;
	batt_high = BATT_CAL + BATT_HIGH;
	batt_medium = BATT_CAL + BATT_MED;
	batt_low = BATT_CAL + BATT_LOW;
	batt_critical = BATT_CAL + BATT_CRITICAL;
	batt_min = BATT_CAL + BATT_MINIMUM;
	batt_off = BATT_CAL + BATT_OFF;
#endif

#ifdef __ADJUST_RECHARGE_ADC__
	batt_recharging = -1;
#endif /* __ADJUST_RECHARGE_ADC__ */

#ifdef __BATTERY_COMPENSATION__
	batt_compensation = 0;
#ifdef COMPENSATE_BOOTING
	s3c_bat_set_compesation(1, OFFSET_BOOTING, COMPENSATE_BOOTING);
	s3c_bat_set_compesation(1, OFFSET_LCD_ON, COMPENSATE_LCD);
#endif /* COMPENSATE_BOOTING */
#endif /* __BATTERY_COMPENSATION__ */

	INIT_WORK(&bat_work, s3c_bat_work);
	INIT_WORK(&cable_work, s3c_cable_work);

#ifdef __ALWAYS_AWAKE_DEVICE__
	dev_info(dev, "%s: always awake(wake_lock)\n", __func__);
	wake_lock(&wake_lock_for_dev);
#endif /* __ALWAYS_AWAKE_DEVICE__ */

	/* init power supplier framework */
	for (i = 0; i < ARRAY_SIZE(s3c_power_supplies); i++) {
		ret = power_supply_register(&pdev->dev, 
				&s3c_power_supplies[i]);
		if (ret) {
			dev_err(dev, "Failed to register"
					"power supply %d,%d\n", i, ret);
			goto __end__;
		}
	}

	/* create sec detail attributes */
	s3c_bat_create_attrs(s3c_power_supplies[CHARGER_BATTERY].dev);

#ifdef __TEST_DEVICE_DRIVER__
	s3c_test_create_attrs(s3c_power_supplies[CHARGER_AC].dev);
#endif /* __TEST_DEVICE_DRIVER__ */

	if (s3c_bat_info.polling) {
		dev_dbg(dev, "%s: will poll for status\n", 
				__func__);
		setup_timer(&polling_timer, polling_timer_func, 0);
		mod_timer(&polling_timer,
			  jiffies + msecs_to_jiffies(s3c_bat_info.polling_interval));
	}
	setup_timer(&cable_timer, cable_timer_func, 0);
	
	s3c_battery_initial = 1;
	force_update = 0;
	full_charge_flag = 0;
	soc_restart_flag = 0;

	if(maxim_lpm_chg_status() && check_UV_charging_case())
		low_batt_boot_flag = 1;

#ifdef MAX17042
	mutex_lock(&work_lock);
	fg_alert_init(); // Set Alert init value in the bootloader
	mutex_unlock(&work_lock);

	INIT_DELAYED_WORK(&fuelgauge_work, fuelgauge_work_handler);
#endif
	
	INIT_DELAYED_WORK(&fullcharging_work, fullcharging_work_handler);
	INIT_DELAYED_WORK(&full_comp_work, full_comp_work_handler);

	s3c_bat_status_update(&s3c_power_supplies[CHARGER_BATTERY]);

#ifdef __CHECK_BATTERY_V_F__
//	s3c_bat_check_v_f();
#endif /* __CHECK_BATTERY_V_F__ */

	s3c_cable_check_status();
	if(HWREV >= 0x8)							
		check_fullchaged();

	/* Request IRQ */ 
	MAX8998_IRQ_init();
	s3c_bat_interrupt_init();


	if(charging_mode_get())
	{
		battery = kzalloc(sizeof(struct battery_driver), GFP_KERNEL);
		battery->early_suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB + 1;
		battery->early_suspend.suspend = battery_early_suspend;
		battery->early_suspend.resume = battery_late_resume;
		register_early_suspend(&battery->early_suspend);
	}

__end__:
	return ret;
}

static int __devexit s3c_bat_remove(struct platform_device *pdev)
{
	int i;
	dev_info(dev, "%s\n", __func__);

	if (s3c_bat_info.polling)
		del_timer_sync(&polling_timer);

#if 0 /* sone */
	free_irq(IRQ_TA_CONNECTED_N, &s3c_power_supplies[CHARGER_BATTERY]);
	free_irq(IRQ_TA_CHG_N, &s3c_power_supplies[CHARGER_BATTERY]);
#endif

	for (i = 0; i < ARRAY_SIZE(s3c_power_supplies); i++) {
		power_supply_unregister(&s3c_power_supplies[i]);
	}
 
	return 0;
}

static struct platform_driver s3c_bat_driver = {
	.driver.name	= DRIVER_NAME,
	.driver.owner	= THIS_MODULE,
	.probe		= s3c_bat_probe,
	.remove		= __devexit_p(s3c_bat_remove),
	.suspend	= s3c_bat_suspend,
	.resume		= s3c_bat_resume,
};

/* Initailize GPIO */
static void s3c_bat_init_hw(void)
{
#if 0  // old charger IC
	s3c_gpio_cfgpin(gpio_ta_connected, S3C_GPIO_SFN(gpio_ta_connected_af));
	s3c_gpio_setpull(gpio_ta_connected, S3C_GPIO_PULL_UP);

	s3c_gpio_cfgpin(gpio_chg_ing, S3C_GPIO_SFN(gpio_chg_ing_af));
	s3c_gpio_setpull(gpio_chg_ing, S3C_GPIO_PULL_UP);
	gpio_set_value_ex(gpio_chg_en, GPIO_LEVEL_HIGH);
#endif

#ifdef __EXTERNEL_CHARGER_IC__
	s3c_gpio_cfgpin(GPIO_TA_EN, S3C_GPIO_OUTPUT);
	s3c_gpio_setpull(GPIO_TA_EN, S3C_GPIO_PULL_NONE);

	s3c_gpio_cfgpin(GPIO_TA_nSTAT, S3C_GPIO_INPUT);
	s3c_gpio_setpull(GPIO_TA_nSTAT, S3C_GPIO_PULL_NONE);

	s3c_gpio_cfgpin(GPIO_TA_nCHG, S3C_GPIO_SFN(GPIO_TA_nCHG_AF));
	s3c_gpio_setpull(GPIO_TA_nCHG, S3C_GPIO_PULL_NONE);
#endif /* __EXTERNEL_CHARGER_IC__ */

	s3c_gpio_cfgpin(GPIO_FUEL_ARLT, S3C_GPIO_SFN(GPIO_FUEL_ARLT_AF));
	s3c_gpio_setpull(GPIO_FUEL_ARLT, S3C_GPIO_PULL_NONE);

}

static int __init s3c_bat_init(void)
{
	pr_info("%s\n", __func__);
	s3c_bat_init_hw();

	wake_lock_init(&vbus_wake_lock, WAKE_LOCK_SUSPEND, "vbus_present");
	wake_lock_init(&temp_wake_lock, WAKE_LOCK_SUSPEND, "temp_lock");
#if (defined __TEST_DEVICE_DRIVER__  || defined __ALWAYS_AWAKE_DEVICE__)
	wake_lock_init(&wake_lock_for_dev, WAKE_LOCK_SUSPEND, "wake_lock_dev");
#endif /* __TEST_DEVICE_DRIVER__ || __ALWAYS_AWAKE_DEVICE__ */

#ifdef __SMB136_CHARGER_IC__
	if(HWREV >=0x08)
	{
		if (i2c_add_driver(&smb136_i2c_driver))
			pr_err("%s: Can't add smb136 i2c drv\n", __func__);
	}
#endif

#ifdef __FUEL_GAUGES_IC__
	if (i2c_add_driver(&fg_i2c_driver))
		pr_err("%s: Can't add fg i2c drv\n", __func__);
#endif /* __FUEL_GAUGES_IC__ */
	return platform_driver_register(&s3c_bat_driver);
}

static void __exit s3c_bat_exit(void)
{
	pr_info("%s\n", __func__);

#ifdef __SMB136_CHARGER_IC__
	if(HWREV >=0x08)
	{
		i2c_del_driver(&smb136_i2c_driver);
	}
		
#endif 
	
#ifdef __FUEL_GAUGES_IC__
	i2c_del_driver(&fg_i2c_driver);
#endif /* __FUEL_GAUGES_IC__ */
	platform_driver_unregister(&s3c_bat_driver);
}

late_initcall(s3c_bat_init);
module_exit(s3c_bat_exit);

MODULE_AUTHOR("Minsung Kim <ms925.kim@samsung.com>");
MODULE_DESCRIPTION("S3C6410 battery driver");
MODULE_LICENSE("GPL");

