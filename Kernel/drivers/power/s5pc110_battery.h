/*
 * linux/drivers/power/s3c6410_battery.h
 *
 * Battery measurement code for S3C6410 platform.
 *
 * Copyright (C) 2009 Samsung Electronics.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#define DRIVER_NAME	"p1-battery"

/*
 * Spica Rev00 board Battery Table
 */
#define BATT_CAL		2447	/* 3.60V */

#define BATT_MAXIMUM		406	/* 4.176V */
#define BATT_FULL		353	/* 4.10V  */
#define BATT_SAFE_RECHARGE 353	/* 4.10V */
#define BATT_ALMOST_FULL	188 /* 3.8641V */	//322	/* 4.066V */
#define BATT_HIGH		112 /* 3.7554V */ 		//221	/* 3.919V */
#define BATT_MED		66 /* 3.6907V */ 		//146	/* 3.811V */
#define BATT_LOW		43 /* 3.6566V */		//112	/* 3.763V */
#define BATT_CRITICAL		8 /* 3.6037V */ 	//(74)	/* 3.707V */
#define BATT_MINIMUM		(-28) /* 3.554V */	//(38)	/* 3.655V */
#define BATT_OFF		(-128) /* 3.4029V */	//(-103)	/* 3.45V  */

#ifdef CONFIG_TARGET_LOCALE_KOR 
const int temper_table[][2] =  {  /* Updated on July 7th 2010 */
	/* ADC, Temperature (C) */
	{ 1836, 		-200}, 
	{ 1824, 		-190},
	{ 1813, 		-180},
	{ 1802, 		-170},
	{ 1791, 		-160},
	{ 1780, 		-150},
	{ 1766, 		-140},
	{ 1752, 		-130},
	{ 1739, 		-120},
	{ 1725, 		-110},
	{ 1712, 		-100},
	{ 1694, 		-90 },
	{ 1676, 		-80 },
	{ 1658, 		-70 },
	{ 1640, 		-60 },
	{ 1623, 		-50 },
	{ 1603, 		-40 },
	{ 1584, 		-30 },
	{ 1564, 		-20 },
	{ 1545, 		-10 },
	{ 1526, 		0	},
	{ 1505, 		10	},
	{ 1484, 		20	},
	{ 1464, 		30	},
	{ 1443, 		40	},
	{ 1423, 		50	},
	{ 1400, 		60	},
	{ 1377, 		70	},
	{ 1354, 		80	},
	{ 1331, 		90	},
	{ 1308, 		100 },
	{ 1282, 		110 },
	{ 1256, 		120 },
	{ 1231, 		130 },
	{ 1205, 		140 },
	{ 1180, 		150 },
	{ 1157, 		160 },
	{ 1135, 		170 },
	{ 1113, 		180 },
	{ 1091, 		190 },
	{ 1069, 		200 },
	{ 1046, 		210 },
	{ 1023, 		220 },
	{ 1000, 		230 },
	{ 977,		240 },
	{ 955,		250 },
	{ 932,		260 },
	{ 909,		270 },
	{ 886,		280 },
	{ 863,		290 },
	{ 840,		300 },
	{ 818,		310 },
	{ 796,		320 },
	{ 775,		330 },
	{ 753,		340 },
	{ 732,		350 },
	{ 712,		360 },
	{ 693,		370 },
	{ 673,		380 },
	{ 654,		390 },
	{ 635,		400 },
	{ 617,		410 },
	{ 600,		420 },
	{ 583,		430 },
	{ 566,		440 },
	{ 549,		450 },
	{ 533,		460 },
	{ 517,		470 },
	{ 501,		480 },
	{ 485,		490 },
	{ 470,		500 },
	{ 456,		510 },
	{ 442,		520 },
	{ 429,		530 },
	{ 415,		540 },
	{ 402,		550 },
	{ 389,		560 },
	{ 376,		570 },
	{ 364,		580 },
	{ 351,		590 },
	{ 339,		600 },
	{ 329,		610 },
	{ 319,		620 },
	{ 309,		630 },
	{ 299,		640 },
	{ 289,		650 },
	{ 273,		660 },
	{ 263,		670 },
	{ 252,		680 },
	{ 242,		690 },
	{ 232,		700 },
};

#else /* CONFIG_TARGET_LOCALE_KOR */
/*
 * P1 Rev05 board Temperature Table
 */
const int temper_table[][2] =  {
	/* ADC, Temperature (C) */
	{ 1830, 		-200},
	{ 1819, 		-190},
	{ 1809, 		-180},
	{ 1798, 		-170},
	{ 1788, 		-160},
	{ 1778, 		-150},
	{ 1767, 		-140},
	{ 1756, 		-130},
	{ 1745, 		-120},
	{ 1734, 		-110},
	{ 1723, 		-100},
	{ 1710, 		-90 },
	{ 1697, 		-80 },
	{ 1685, 		-70 },
	{ 1672, 		-60 },
	{ 1660, 		-50 },
	{ 1638, 		-40 },
	{ 1616, 		-30 },
	{ 1594, 		-20 },
	{ 1572, 		-10 },
	{ 1550, 		0	},
	{ 1532, 		10	},
	{ 1514, 		20	},
	{ 1494, 		30	},
	{ 1478, 		40	},
	{ 1461, 		50	},
	{ 1437, 		60	},
	{ 1414, 		70	},
	{ 1390, 		80	},
	{ 1367, 		90	},
	{ 1344, 		100 },
	{ 1322, 		110 },
	{ 1300, 		120 },
	{ 1279, 		130 },
	{ 1257, 		140 },
	{ 1236, 		150 },
	{ 1208, 		160 },
	{ 1181, 		170 },
	{ 1153, 		180 },
	{ 1126, 		190 },
	{ 1099, 		200 },
	{ 1076, 		210 },
	{ 1054, 		220 },
	{ 1031, 		230 },
	{ 1009,		240 },
	{ 987,		250 },
	{ 965,		260 },
	{ 943,		270 },
	{ 921,		280 },
	{ 899,		290 },
	{ 877,		300 },
	{ 852,		310 },
	{ 828,		320 },
	{ 803,		330 },
	{ 779,		340 },
	{ 755,		350 },
	{ 734,		360 },
	{ 713,		370 },
	{ 693,		380 },
	{ 672,		390 },
	{ 652,		400 },
	{ 634,		410 },
	{ 617,		420 },
	{ 600,		430 },
	{ 583,		440 },
	{ 566,		450 },
	{ 547,		460 },
	{ 528,		470 },
	{ 509,		480 },
	{ 490,		490 },
	{ 471,		500 },
	{ 457,		510 },
	{ 443,		520 },
	{ 429,		530 },
	{ 415,		540 },
	{ 402,		550 },
	{ 389,		560 },
	{ 376,		570 },
	{ 364,		580 },
	{ 351,		590 },
	{ 339,		600 },
	{ 328,		610 },
	{ 317,		620 },
	{ 306,		630 },
	{ 295,		640 },
	{ 284,		650 },
	{ 273,		660 },
	{ 263,		670 },
	{ 252,		680 },
	{ 242,		690 },
	{ 232,		700 },
};
#endif /* CONFIG_TARGET_LOCALE_KOR */

//2010.7.15 Removed P1_KOR feature to keep the same values for the followings
//2010.7.7 request by HW
#ifdef CONFIG_TARGET_LOCALE_KOR 
#define TEMP_HIGH_BLOCK		temper_table[67][0] /* 47C */
#define TEMP_HIGH_RECOVER	temper_table[64][0] /* 44C */
#define TEMP_LOW_BLOCK		temper_table[20][0] /* 0C */
#define TEMP_LOW_RECOVER	temper_table[23][0] /* 3C */
#else
#define TEMP_HIGH_BLOCK		temper_table[72][0] /* 52C */
#define TEMP_HIGH_RECOVER	temper_table[69][0] /* 49C */
#define TEMP_LOW_BLOCK		temper_table[20][0] /* 0C */
#define TEMP_LOW_RECOVER	temper_table[23][0] /* 3C */
#endif

#define TEMP_FG_HIGH_BLOCK		55000
#define TEMP_FG_HIGH_RECOVER	45000
#define TEMP_FG_LOW_BLOCK		0
#define TEMP_FG_LOW_RECOVER		3000

typedef enum adc_channel {
	ADC_TEMPERATURE = 0,
	ADC_BATT_MON,
	ADC_CHG_CURRENT,
	ADC_EAR_ADC_35,
	ADC_ACCESSORY_ID,
	ADC_REMOTE_SENSE,
	ADC_AP_CHECK_2,
	ADC_AP_CHECK_1,
	ADC_CH8,
	ADC_CH9,
	ENDOFADC
} adc_channel_type;


/******************************************************************************
 * Battery driver features
 * ***************************************************************************/
/* #define __TEMP_ADC_VALUE__ */
/* #define __CHECK_BATTERY_V_F__ */
/* #define __PMIC_HAS_V_F__ */
/* #define __BATTERY_V_F_ADC__ */
/* #define __BATTERY_COMPENSATION__ */
/* #define __CHECK_BOARD_REV__ */
/* #define __BOARD_REV_ADC__ */
#define __TEST_DEVICE_DRIVER__
/* #define __ALWAYS_AWAKE_DEVICE__  */
#define __TEST_MODE_INTERFACE__
#define __FUEL_GAUGES_IC__ 
#define __EXTERNEL_CHARGER_IC__
#define __SMB136_CHARGER_IC__
/*****************************************************************************/

#define TOTAL_CHARGING_TIME		(6*60*60*HZ)	/* 6 hours */
#define TOTAL_RECHARGING_TIME	(2*60*60*HZ)	/* 2 hours */

#ifdef __BATTERY_COMPENSATION__
#define COMPENSATE_VIBRATOR		19
#define COMPENSATE_CAMERA		25
#define COMPENSATE_MP3			17
#define COMPENSATE_VIDEO		28
#define COMPENSATE_VOICE_CALL_2G	13
#define COMPENSATE_VOICE_CALL_3G	14
#define COMPENSATE_DATA_CALL		25
#define COMPENSATE_LCD			0
#define COMPENSATE_TA			0
#define COMPENSATE_CAM_FALSH		0
#define COMPENSATE_BOOTING		52
#endif /* __BATTERY_COMPENSATION__ */

#ifdef __FUEL_GAUGES_IC__
//#define SOC_LB_FOR_POWER_OFF		27

#define FULL_CHARGE_COND_VOLTAGE		4000
#define RECHARGE_COND_VOLTAGE			4135
#ifdef CONFIG_TARGET_LOCALE_KOR 	
#define RECHARGE_COND_BACKUP_VOLTAGE	4000
#endif 
#define CURRENT_OF_FULL_CHG 		225 /* 150mA => (150*1.5)mV */
#endif /* __FUEL_GAUGES_IC__ */

