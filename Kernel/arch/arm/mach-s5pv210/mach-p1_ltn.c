/* linux/arch/arm/mach-s5pv210/mach-smdkc110.c
 *
 * Copyright (c) 2010 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/serial_core.h>
#include <linux/gpio.h>
#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/regulator/max8998.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/usb/ch9.h>
#include <linux/pwm_backlight.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/setup.h>
#include <asm/mach-types.h>

#include <mach/map.h>
#include <mach/regs-clock.h>
#include <mach/regs-mem.h>
#include <mach/regs-gpio.h>
#include <mach/gpio-bank.h>
#include <mach/adcts.h>
#include <mach/ts.h>
#include <mach/param.h>

#include <media/s5k3ba_platform.h>
#include <media/s5k4ba_platform.h>
#include <media/s5k4ea_platform.h>
#include <media/s5k6aa_platform.h>

#include <plat/regs-serial.h>
#include <plat/s5pv210.h>
#include <plat/devs.h>
#include <plat/cpu.h>
#include <plat/fb.h>
#include <plat/gpio-cfg.h>
#include <plat/iic.h>

#include <plat/fimc.h>
#include <plat/regs-fimc.h>
#include <plat/csis.h>
#include <plat/mfc.h>
#include <plat/sdhci.h>
#include <plat/ide.h>
#include <plat/regs-otg.h>
#include <plat/clock.h>
#include <plat/gpio-core.h>

#include <mach/gpio-p1.ltn.h>

#ifdef CONFIG_ANDROID_PMEM
#include <linux/android_pmem.h>
#include <plat/media.h>
#endif

#if defined(CONFIG_PM)
#include <plat/pm.h>
#endif

#include <mach/max8998_function.h>

#include <mach/sec_jack.h>

/* additional feature - start >> */
#include <linux/input.h>
#include <linux/gpio_keys.h>
#include <mach/system.h>
#if CONFIG_FB_S3C
#include <video/s3cfb.h>
#endif
#include <linux/spi/spi.h>
#include <plat/spi.h>
#include <linux/swi.h>
#include <linux/spi/spi_gpio.h>
#include <linux/i2c-gpio.h>
#include <linux/i2c/l3g4200d.h>
#include <linux/bh1721.h>
#include <linux/regulator/max8998.h>
#include <linux/reboot.h>

#include <media/isx005_platform.h>
#include <media/s5k6aafx_platform.h>
#ifdef CONFIG_VIDEO_NM6XX
#include <media/nm6xx_platform.h> //johnny.kim
#endif

#include <linux/modemctl.h>
#include <linux/onedram.h>

#include <linux/irq.h>

#ifdef CONFIG_KERNEL_DEBUG_SEC
#include <linux/kernel_sec_common.h>
#endif

struct class *sec_class;
EXPORT_SYMBOL(sec_class);

struct device *switch_dev;
EXPORT_SYMBOL(switch_dev);

struct device *gps_dev = NULL;
EXPORT_SYMBOL(gps_dev);

void (*sec_set_param_value)(int idx, void *value);
EXPORT_SYMBOL(sec_set_param_value);

void (*sec_get_param_value)(int idx, void *value);
EXPORT_SYMBOL(sec_get_param_value);

unsigned int HWREV=0;
EXPORT_SYMBOL(HWREV);

extern void qt602240_set_amoled_display(int mode);

typedef enum {
	MACHINE_UNKNOWN		= -1,
	MACHINE_P1_AMOLED	= 0x0,
	MACHINE_P2			= 0x1,
	MACHINE_AQUILA		= 0x2,
	MACHINE_P1_TFT		= 0x3,
} machine_type_t;

static machine_type_t machine_type = MACHINE_UNKNOWN;

static machine_type_t get_machine_type(void);

static struct platform_device s3c_device_qtts = {
	.name = "qt602240-ts",
	.id = -1,
};

/* << additional feature - end */

/* Following are default values for UCON, ULCON and UFCON UART registers */
#define S5PV210_UCON_DEFAULT	(S3C2410_UCON_TXILEVEL |	\
				 S3C2410_UCON_RXILEVEL |	\
				 S3C2410_UCON_TXIRQMODE |	\
				 S3C2410_UCON_RXIRQMODE |	\
				 S3C2410_UCON_RXFIFO_TOI |	\
				 S3C2443_UCON_RXERR_IRQEN)

#define S5PV210_ULCON_DEFAULT	S3C2410_LCON_CS8

#define S5PV210_UFCON_DEFAULT	(S3C2410_UFCON_FIFOMODE |	\
				 S5PV210_UFCON_TXTRIG4 |	\
				 S5PV210_UFCON_RXTRIG4)


extern void s5pv210_reserve_bootmem(void);
extern void s3c_sdhci_set_platdata(void);


static struct s3c2410_uartcfg smdkv210_uartcfgs[] __initdata = {
	[0] = {
		.hwport		= 0,
		.flags		= 0,
		.ucon		= S5PV210_UCON_DEFAULT,
		.ulcon		= S5PV210_ULCON_DEFAULT,
		.ufcon		= S5PV210_UFCON_DEFAULT,
	},
	[1] = {
		.hwport		= 1,
		.flags		= 0,
		.ucon		= S5PV210_UCON_DEFAULT,
		.ulcon		= S5PV210_ULCON_DEFAULT,
		.ufcon		= S5PV210_UFCON_DEFAULT,
	},
	[2] = {
		.hwport		= 2,
		.flags		= 0,
		.ucon		= S5PV210_UCON_DEFAULT,
		.ulcon		= S5PV210_ULCON_DEFAULT,
		.ufcon		= S5PV210_UFCON_DEFAULT,
	},
	[3] = {
		.hwport		= 3,
		.flags		= 0,
		.ucon		= S5PV210_UCON_DEFAULT,
		.ulcon		= S5PV210_ULCON_DEFAULT,
		.ufcon		= S5PV210_UFCON_DEFAULT,
	},
};

/* PMIC */
static struct regulator_consumer_supply dcdc1_consumers[] = {
	{
		.supply		= "vddarm",
	},
};

static struct regulator_init_data max8998_dcdc1_data = {
	.constraints	= {
		.name		= "VCC_ARM",
		.min_uV		=  750000,
		.max_uV		= 1500000,
		.always_on	= 1,
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE,
	},
	.num_consumer_supplies	= ARRAY_SIZE(dcdc1_consumers),
	.consumer_supplies	= dcdc1_consumers,
};

static struct regulator_consumer_supply dcdc2_consumers[] = {
	{
		.supply		= "vddint",
	},
};

static struct regulator_init_data max8998_dcdc2_data = {
	.constraints	= {
		.name		= "VCC_INTERNAL",
		.min_uV		=  750000,
		.max_uV		= 1500000,
		.always_on	= 1,
//		.apply_uV	= 1,
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE,
	},
	.num_consumer_supplies	= ARRAY_SIZE(dcdc2_consumers),
	.consumer_supplies	= dcdc2_consumers,
};

static struct regulator_init_data max8998_dcdc4_data = {
	.constraints	= {
		.name		= "DCDC4",
		.min_uV		=  800000,
		.max_uV		= 2300000,
		//.always_on	= 1,
//		.apply_uV	= 1,
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE,
	},
};

#ifdef CONFIG_USB_SAMSUNG_LTE_MODEM
// for USB HOST always on
static struct regulator_init_data max8998_ldo3_data = {
	.constraints	= {
		.name		= "MIPI_1.1V",
		.min_uV		= 1100000,
		.max_uV		= 1100000,
		.always_on	= 1,
		.apply_uV	= 1,
		.valid_ops_mask	= REGULATOR_CHANGE_STATUS,
		},
};
#endif	//def CONFIG_USB_SAMSUNG_LTE_MODEM

static struct regulator_init_data max8998_ldo4_data = {
        .constraints    = {
                .name           = "VCC_DAC",
                .min_uV         = 3300000,
                .max_uV         = 3300000,
		.always_on	= 1,
                .apply_uV       = 1,
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE,
        },
};

#ifdef CONFIG_USB_SAMSUNG_LTE_MODEM
// for USB HOST always on
static struct regulator_init_data max8998_ldo8_data = {
	.constraints	= {
		.name		= "VUSB/VADC_3.3V",
		.min_uV		= 3300000,
		.max_uV		= 3300000,
		.apply_uV	= 1,
		.always_on	= 1,
		.valid_ops_mask	= REGULATOR_CHANGE_STATUS,
		},
};
#endif	//def CONFIG_USB_SAMSUNG_LTE_MODEM

static struct regulator_init_data max8998_ldo11_data = {
        .constraints    = {
                .name           = "CAM_IO_2.8V",
                .min_uV         = 2800000,
                .max_uV         = 2800000,
		//.always_on	= 1,
               // .apply_uV       = 1,
		//.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE,
        },
};

static struct regulator_init_data max8998_ldo12_data = {
        .constraints    = {
                .name           = "CAM_5M_1.8V",
                .min_uV         = 1800000,
                .max_uV         = 1800000,
		//.always_on	= 1,
                //.apply_uV       = 1,
		//.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE,
        },
};

static struct regulator_init_data max8998_ldo13_data = {
        .constraints    = {
                .name           = "CAM_A_2.8V",
                .min_uV         = 2800000,
                .max_uV         = 2800000,
		//.always_on	= 1,
                //.apply_uV       = 1,
		//.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE,
        },
};

static struct regulator_init_data max8998_ldo14_data = {
        .constraints    = {
                .name           = "CAM_CIF_1.8V",
                .min_uV         = 1800000,
                .max_uV         = 1800000,
		//.always_on	= 1,
                //.apply_uV       = 1,
		//.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE,
        },
};

static struct regulator_init_data max8998_ldo15_data = {
        .constraints    = {
                .name           = "CAM_AF_2.8V",
                .min_uV         = 2800000,
                .max_uV         = 2800000,
		//.always_on	= 1,
                //.apply_uV       = 1,
		//.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE,
        },
};

static struct regulator_init_data max8998_ldo6_data = {
        .constraints    = {
                .name           = "VTF_2.8V",
                .min_uV         = 2800000,
                .max_uV         = 2800000,
		.always_on	= 1,
		.apply_uV	= 1,
//		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE,
        },
};

static struct regulator_init_data max8998_ldo7_data = {
        .constraints    = {
                .name           = "VCC_LCD",
                .min_uV         = 1600000,
                .max_uV         = 3600000,
//		.always_on	= 1,		// MIDAS[2010.08.04]@Disable VTF_2.8V
                //.apply_uV	= 1,
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE,
        },
};

static struct regulator_init_data max8998_ldo17_data = {
        .constraints    = {
                .name           = "PM_LVDS_VDD",
                .min_uV         = 1600000,
                .max_uV         = 3600000,
		.always_on	= 1,
                //.apply_uV	= 1,
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE,
        },
};


static struct max8998_subdev_data universal_regulators[] = {
	{ MAX8998_DCDC1, &max8998_dcdc1_data },
	{ MAX8998_DCDC2, &max8998_dcdc2_data },
//	{ MAX8998_DCDC4, &max8998_dcdc4_data },
#ifdef CONFIG_USB_SAMSUNG_LTE_MODEM
	{ MAX8998_LDO3, &max8998_ldo3_data },
#endif //def CONFIG_USB_SAMSUNG_LTE_MODEM	
	{ MAX8998_LDO4, &max8998_ldo4_data },
#ifdef CONFIG_USB_SAMSUNG_LTE_MODEM
	{ MAX8998_LDO8, &max8998_ldo8_data },
#endif //def CONFIG_USB_SAMSUNG_LTE_MODEM	
	{ MAX8998_LDO11, &max8998_ldo11_data },
//	{ MAX8998_LDO12, &max8998_ldo12_data },
	{ MAX8998_LDO13, &max8998_ldo13_data },
	{ MAX8998_LDO14, &max8998_ldo14_data },
//	{ MAX8998_LDO15, &max8998_ldo15_data },
//	{ MAX8998_LDO6, &max8998_ldo6_data },
	{ MAX8998_LDO7, &max8998_ldo7_data },
	{ MAX8998_LDO17, &max8998_ldo17_data },
};

static struct max8998_platform_data max8998_platform_data = {
	.num_regulators	= ARRAY_SIZE(universal_regulators),
	.regulators	= universal_regulators,
};

#ifdef CONFIG_S5PV210_ADCTS
static struct s3c_adcts_plat_info s3c_adcts_cfgs __initdata = {
	.channel = {
		{ /* 0 */
			.delay = 0xFF,
			.presc = 49,
			.resol = S3C_ADCCON_RESSEL_12BIT,
		},{ /* 1 */
			.delay = 0xFF,
			.presc = 49,
			.resol = S3C_ADCCON_RESSEL_12BIT,
		},{ /* 2 */
			.delay = 0xFF,
			.presc = 49,
			.resol = S3C_ADCCON_RESSEL_12BIT,
		},{ /* 3 */
			.delay = 0xFF,
			.presc = 49,
			.resol = S3C_ADCCON_RESSEL_12BIT,
		},{ /* 4 */
			.delay = 0xFF,
			.presc = 49,
			.resol = S3C_ADCCON_RESSEL_12BIT,
		},{ /* 5 */
			.delay = 0xFF,
			.presc = 49,
			.resol = S3C_ADCCON_RESSEL_12BIT,
		},{ /* 6 */
			.delay = 0xFF,
			.presc = 49,
			.resol = S3C_ADCCON_RESSEL_12BIT,
		},{ /* 7 */
			.delay = 0xFF,
			.presc = 49,
			.resol = S3C_ADCCON_RESSEL_12BIT,
		},
	},
};
#endif

#ifdef CONFIG_TOUCHSCREEN_S3C
static struct s3c_ts_mach_info s3c_ts_platform __initdata = {
	.adcts = {
		.delay = 0xFF,
	.presc                  = 49,
		.resol = S3C_ADCCON_RESSEL_12BIT,
	},
	.sampling_time = 18,
	.sampling_interval_ms = 20,
	.x_coor_min	= 180,
	.x_coor_max = 4000,
	.x_coor_fuzz = 32,
	.y_coor_min = 300,
	.y_coor_max = 3900,
	.y_coor_fuzz = 32,
	.use_tscal = false,
	.tscal = {0, 0, 0, 0, 0, 0, 0},
};
#endif

#ifndef CONFIG_S5PV210_ADCTS
static struct s3c_adc_mach_info s3c_adc_platform __initdata = {
	/* s5pc110 support 12-bit resolution */
	.delay  = 10000,
	.presc  = 49,
	.resolution = 12,
};
#endif

#ifdef CONFIG_VIDEO_FIMC
/*
 * External camera reset
 * Because the most of cameras take i2c bus signal, so that
 * you have to reset at the boot time for other i2c slave devices.
 * This function also called at fimc_init_camera()
 * Do optimization for cameras on your platform.
*/
static int smdkv210_cam0_power(int onoff)
{
	int err;
	/* Camera A */
	err = gpio_request(S5PV210_GPH0(2), "GPH0");
	if (err)
		printk(KERN_ERR "#### failed to request GPH0 for CAM_2V8\n");

	s3c_gpio_setpull(S5PV210_GPH0(2), S3C_GPIO_PULL_NONE);
	gpio_direction_output(S5PV210_GPH0(2), 0);
	gpio_direction_output(S5PV210_GPH0(2), 1);
	gpio_free(S5PV210_GPH0(2));

	return 0;
}

static int smdkv210_cam1_power(int onoff)
{
	int err;

	/* Camera B */
	err = gpio_request(S5PV210_GPH0(3), "GPH0");
	if (err)
		printk(KERN_ERR "#### failed to request GPH0 for CAM_2V8\n");

	s3c_gpio_setpull(S5PV210_GPH0(3), S3C_GPIO_PULL_NONE);
	gpio_direction_output(S5PV210_GPH0(3), 0);
	gpio_direction_output(S5PV210_GPH0(3), 1);
	gpio_free(S5PV210_GPH0(3));

	return 0;
}

static int smdkv210_mipi_cam_reset(void)
{
	int err;

	err = gpio_request(S5PV210_GPH0(3), "GPH0");
	if (err)
		printk(KERN_ERR "#### failed to reset(GPH0) for MIPI CAM\n");

	s3c_gpio_setpull(S5PV210_GPH0(3), S3C_GPIO_PULL_NONE);
	gpio_direction_output(S5PV210_GPH0(3), 0);
	gpio_direction_output(S5PV210_GPH0(3), 1);
	gpio_free(S5PV210_GPH0(3));

	return 0;
}

static int smdkv210_mipi_cam_power(int onoff)
{
	int err;

	/* added for V210 CAM power */
	err = gpio_request(S5PV210_GPH1(2), "GPH1");
	if (err)
		printk(KERN_ERR "#### failed to request(GPH1)for CAM_2V8\n");

	s3c_gpio_setpull(S5PV210_GPH1(2), S3C_GPIO_PULL_NONE);
	gpio_direction_output(S5PV210_GPH1(2), onoff);
	gpio_free(S5PV210_GPH1(2));

	return 0;
}

/*
 * Guide for Camera Configuration for smdkv210
 * ITU channel must be set as A or B
 * ITU CAM CH A: S5K3BA only
 * ITU CAM CH B: one of S5K3BA and S5K4BA
 * MIPI: one of S5K4EA and S5K6AA
 *
 * NOTE1: if the S5K4EA is enabled, all other cameras must be disabled
 * NOTE2: currently, only 1 MIPI camera must be enabled
 * NOTE3: it is possible to use both one ITU cam and
 * 	  one MIPI cam except for S5K4EA case
 *
*/
#undef CAM_ITU_CH_A
#undef S5K3BA_ENABLED
#define S5K4BA_ENABLED
#undef S5K4EA_ENABLED
#undef S5K6AA_ENABLED
#define WRITEBACK_ENABLED

/* External camera module setting */
/* 2 ITU Cameras */
#ifdef S5K3BA_ENABLED
static struct s5k3ba_platform_data s5k3ba_plat = {
	.default_width = 640,
	.default_height = 480,
	.pixelformat = V4L2_PIX_FMT_VYUY,
	.freq = 24000000,
	.is_mipi = 0,
};

static struct i2c_board_info  s5k3ba_i2c_info = {
	I2C_BOARD_INFO("S5K3BA", 0x2d),
	.platform_data = &s5k3ba_plat,
};

static struct s3c_platform_camera s5k3ba = {
#ifdef CAM_ITU_CH_A
	.id		= CAMERA_PAR_A,
#else
	.id		= CAMERA_PAR_B,
#endif
	.type		= CAM_TYPE_ITU,
	.fmt		= ITU_601_YCBCR422_8BIT,
	.order422	= CAM_ORDER422_8BIT_CRYCBY,
	.i2c_busnum	= 0,
	.info		= &s5k3ba_i2c_info,
	.pixelformat	= V4L2_PIX_FMT_VYUY,
	.srclk_name	= "mout_epll",
	.clk_name	= "sclk_cam1",
	.clk_rate	= 24000000,
	.line_length	= 1920,
	.width		= 640,
	.height		= 480,
	.window		= {
		.left	= 0,
		.top	= 0,
		.width	= 640,
		.height	= 480,
	},

	/* Polarity */
	.inv_pclk	= 0,
	.inv_vsync	= 1,
	.inv_href	= 0,
	.inv_hsync	= 0,

	.initialized	= 0,
#ifdef CAM_ITU_CH_A
	.cam_power	= smdkv210_cam0_power,
#else
	.cam_power	= smdkv210_cam1_power,
#endif
};
#endif

#ifdef S5K4BA_ENABLED
static struct s5k4ba_platform_data s5k4ba_plat = {
	.default_width = 800,
	.default_height = 600,
	.pixelformat = V4L2_PIX_FMT_UYVY,
	.freq = 44000000,
	.is_mipi = 0,
};

static struct i2c_board_info  s5k4ba_i2c_info = {
	I2C_BOARD_INFO("S5K4BA", 0x2d),
	.platform_data = &s5k4ba_plat,
};

static struct s3c_platform_camera s5k4ba = {
#ifdef CAM_ITU_CH_A
	.id		= CAMERA_PAR_A,
#else
	.id		= CAMERA_PAR_B,
#endif
	.type		= CAM_TYPE_ITU,
	.fmt		= ITU_601_YCBCR422_8BIT,
	.order422	= CAM_ORDER422_8BIT_CBYCRY,
	.i2c_busnum	= 0,
	.info		= &s5k4ba_i2c_info,
	.pixelformat	= V4L2_PIX_FMT_UYVY,
	.srclk_name	= "mout_mpll",
	.clk_name	= "sclk_cam1",
	.clk_rate	= 44000000,
	.line_length	= 1920,
	.width		= 800,
	.height		= 600,
	.window		= {
		.left	= 0,
		.top	= 0,
		.width	= 800,
		.height	= 600,
	},

	/* Polarity */
	.inv_pclk	= 0,
	.inv_vsync	= 1,
	.inv_href	= 0,
	.inv_hsync	= 0,

	.initialized	= 0,
#ifdef CAM_ITU_CH_A
	.cam_power	= smdkv210_cam0_power,
#else
	.cam_power	= smdkv210_cam1_power,
#endif
};
#endif

/* 2 MIPI Cameras */
#ifdef S5K4EA_ENABLED
static struct s5k4ea_platform_data s5k4ea_plat = {
	.default_width = 1920,
	.default_height = 1080,
	.pixelformat = V4L2_PIX_FMT_UYVY,
	.freq = 24000000,
	.is_mipi = 1,
};

static struct i2c_board_info  s5k4ea_i2c_info = {
	I2C_BOARD_INFO("S5K4EA", 0x2d),
	.platform_data = &s5k4ea_plat,
};

static struct s3c_platform_camera s5k4ea = {
	.id		= CAMERA_CSI_C,
	.type		= CAM_TYPE_MIPI,
	.fmt		= MIPI_CSI_YCBCR422_8BIT,
	.order422	= CAM_ORDER422_8BIT_CBYCRY,
	.i2c_busnum	= 0,
	.info		= &s5k4ea_i2c_info,
	.pixelformat	= V4L2_PIX_FMT_UYVY,
	.srclk_name	= "mout_mpll",
	.clk_name	= "sclk_cam0",
	.clk_rate	= 48000000,
	.line_length	= 1920,
	.width		= 1920,
	.height		= 1080,
	.window		= {
		.left	= 0,
		.top	= 0,
		.width	= 1920,
		.height	= 1080,
	},

	.mipi_lanes	= 2,
	.mipi_settle	= 12,
	.mipi_align	= 32,

	/* Polarity */
	.inv_pclk	= 0,
	.inv_vsync	= 1,
	.inv_href	= 0,
	.inv_hsync	= 0,

	.initialized	= 0,
	.cam_power	= smdkv210_mipi_cam_power,
};
#endif

#ifdef S5K6AA_ENABLED
static struct s5k6aa_platform_data s5k6aa_plat = {
	.default_width = 640,
	.default_height = 480,
	.pixelformat = V4L2_PIX_FMT_UYVY,
	.freq = 24000000,
	.is_mipi = 1,
};

static struct i2c_board_info  s5k6aa_i2c_info = {
	I2C_BOARD_INFO("S5K6AA", 0x3c),
	.platform_data = &s5k6aa_plat,
};

static struct s3c_platform_camera s5k6aa = {
	.id		= CAMERA_CSI_C,
	.type		= CAM_TYPE_MIPI,
	.fmt		= MIPI_CSI_YCBCR422_8BIT,
	.order422	= CAM_ORDER422_8BIT_CBYCRY,
	.i2c_busnum	= 0,
	.info		= &s5k6aa_i2c_info,
	.pixelformat	= V4L2_PIX_FMT_UYVY,
	.srclk_name	= "mout_epll",
	.clk_name	= "sclk_cam0",
	.clk_rate	= 24000000,
	.line_length	= 1920,
	/* default resol for preview kind of thing */
	.width		= 640,
	.height		= 480,
	.window		= {
		.left	= 0,
		.top	= 0,
		.width	= 640,
		.height	= 480,
	},

	.mipi_lanes	= 1,
	.mipi_settle	= 6,
	.mipi_align	= 32,

	/* Polarity */
	.inv_pclk	= 0,
	.inv_vsync	= 1,
	.inv_href	= 0,
	.inv_hsync	= 0,

	.initialized	= 0,
	.cam_power	= smdkv210_mipi_cam_power,
};
#endif

#ifdef WRITEBACK_ENABLED
static struct i2c_board_info  writeback_i2c_info = {
	I2C_BOARD_INFO("WriteBack", 0x0),
};

static struct s3c_platform_camera writeback = {
	.id		= CAMERA_WB,
	.fmt		= ITU_601_YCBCR422_8BIT,
	.order422	= CAM_ORDER422_8BIT_CBYCRY,
	.i2c_busnum	= 0,
	.info		= &writeback_i2c_info,
	.pixelformat	= V4L2_PIX_FMT_YUV444,
	.line_length	= 800,
	.width		= 480,
	.height		= 800,
	.window		= {
		.left	= 0,
		.top	= 0,
		.width	= 480,
		.height	= 800,
	},

	.initialized	= 0,
};
#endif

/* External camera module setting */
#ifdef CONFIG_VIDEO_S5K6AAFX
#if 0
static void s5ka3dfx_ldo_en(bool onoff)
{
	if(onoff){
		pmic_ldo_enable(LDO_CAM_IO);
		pmic_ldo_enable(LDO_CAM_A);
		pmic_ldo_enable(LDO_CAM_CIF);
	} else {
		pmic_ldo_disable(LDO_CAM_IO);
		pmic_ldo_disable(LDO_CAM_A);
		pmic_ldo_disable(LDO_CAM_CIF);
	}
}

static int s5ka3dfx_cam_stdby(bool onoff)
{
	int err;
	/* CAM_VGA_nSTBY - GPB(0) */
	err = gpio_request(S5PV210_GPB(0), "GPB");
	if (err) {
		printk(KERN_ERR "failed to request GPJ0 for camera control\n");
		return err;
	}

	gpio_direction_output(S5PV210_GPB(0), 0); 
	msleep(1);
	gpio_direction_output(S5PV210_GPB(0), 1); 
	msleep(1);

	if(onoff){
		gpio_set_value(S5PV210_GPB(0), 1); 
	} else {
		gpio_set_value(S5PV210_GPB(0), 0); 
	}
	msleep(1);

	gpio_free(S5PV210_GPB(0));

	return 0;
}

static int s5ka3dfx_cam_nrst(bool onoff)
{
	int err;

	/* CAM_VGA_nRST - GPB(2)*/
	err = gpio_request(S5PV210_GPB(2), "GPB");
	if (err) {
		printk(KERN_ERR "failed to request GPJ1 for camera control\n");
		return err;
	}

	gpio_direction_output(S5PV210_GPB(2), 0);
	msleep(1);
	gpio_direction_output(S5PV210_GPB(2), 1);
	msleep(1);

	gpio_set_value(S5PV210_GPB(2), 0);
	msleep(1);

	if(onoff){
		gpio_set_value(S5PV210_GPB(2), 1);
		msleep(1);
	}
	gpio_free(S5PV210_GPB(2));

	return 0;
}

#endif

static inline int s5k6aafx_power_on()
{
	int err;

	printk(KERN_ERR "s5k6aafx_power_on\n");

	/* CAM_VGA_nSTBY - GPB(0)  */
	err = gpio_request(GPIO_CAM_VGA_nSTBY, "GPB");
	if (err) {
		printk(KERN_ERR "failed to request GPIO for camera nSTBY pin\n");
		return err;
	}

	/* CAM_VGA_nRST - GPB(2) */
	err = gpio_request(GPIO_CAM_VGA_nRST, "GPB");
	if (err) {
		printk(KERN_ERR "failed to request GPIO for camera nRST pin\n");
		return err;
	}

	/* Turn CAM_A_2.8V on */
	Set_MAX8998_PM_OUTPUT_Voltage(LDO13, VCC_2p800);
	Set_MAX8998_PM_REG(ELDO13, 1);
	udelay(50);

	/* Turn CAM_CIF_1.5V on */
	if(HWREV <= 0x03){ // Under Rev0.3
		Set_MAX8998_PM_OUTPUT_Voltage(LDO14, VCC_1p500);
		Set_MAX8998_PM_REG(ELDO14, 1);
		udelay(50);
	} else if(HWREV >= 0x04){ //Over Rev0.4
		Set_MAX8998_PM_OUTPUT_Voltage(LDO12, VCC_1p500);
		Set_MAX8998_PM_REG(ELDO12, 1);
		udelay(50);
	}

	/* Turn CAM_IO_2.8V on */
	Set_MAX8998_PM_OUTPUT_Voltage(LDO11, VCC_2p800);
	Set_MAX8998_PM_REG(ELDO11, 1);
	udelay(50);

	/* CAM_VGA_nSTBY  HIGH */
	gpio_direction_output(GPIO_CAM_VGA_nSTBY, 0);
	gpio_set_value(GPIO_CAM_VGA_nSTBY, 1);
	udelay(500);

	/* Mclk enable */
	s3c_gpio_cfgpin(GPIO_CAM_MCLK, S5PV210_GPE1_3_CAM_A_CLKOUT);
	udelay(200);

	/* CAM_VGA_nRST  HIGH */
	gpio_direction_output(GPIO_CAM_VGA_nRST, 0);
	gpio_set_value(GPIO_CAM_VGA_nRST, 1);		
	udelay(500);

	gpio_free(GPIO_CAM_VGA_nSTBY);
	gpio_free(GPIO_CAM_VGA_nRST);	

	return 0;
}



static inline int s5k6aafx_power_off()
{
	int err;

	printk(KERN_ERR "s5ka6aax_power_off\n");

	/* CAM_VGA_nSTBY - GPB(0)  */
	err = gpio_request(GPIO_CAM_VGA_nSTBY, "GPB");
	if (err) {
		printk(KERN_ERR "failed to request GPIO for camera nSTBY pin\n");
		return err;
	}

	/* CAM_VGA_nRST - GPB(2) */
	err = gpio_request(GPIO_CAM_VGA_nRST, "GPB");
	if (err) {
		printk(KERN_ERR "failed to request GPIO for camera nRST pin\n");
		return err;
	}

	/* CAM_VGA_nRST  LOW */
	gpio_direction_output(GPIO_CAM_VGA_nRST, 1);
	gpio_set_value(GPIO_CAM_VGA_nRST, 0);
	udelay(200);

	/* Mclk disable */
	s3c_gpio_cfgpin(GPIO_CAM_MCLK, 0);
	udelay(50);

	/* CAM_VGA_nSTBY  LOW */
	gpio_direction_output(GPIO_CAM_VGA_nSTBY, 1);
	gpio_set_value(GPIO_CAM_VGA_nSTBY, 0);
	udelay(200);

	/* Turn CAM_IO_2.8V off */
	Set_MAX8998_PM_REG(ELDO11, 0);
	udelay(50);

	/* Turn CAM_CIF_1.8V off */
	if(HWREV <= 0x03){ // Under Rev0.3
		Set_MAX8998_PM_REG(ELDO14, 0);
	} else if(HWREV >= 0x04){ //Over Rev0.4
		Set_MAX8998_PM_REG(ELDO12, 0);
	}
	udelay(50);

	/* Turn CAM_A_2.8V off */
	Set_MAX8998_PM_REG(ELDO13, 0);

	gpio_free(GPIO_CAM_VGA_nSTBY);
	gpio_free(GPIO_CAM_VGA_nRST);	

	return 0;
}

static int s5k6aafx_power_en(int onoff)
{
#if 0
	if(onoff){
		s5ka3dfx_ldo_en(true);
		s3c_gpio_cfgpin(S5PV210_GPE1(3), S5PV210_GPE1_3_CAM_A_CLKOUT);
		s5ka3dfx_cam_stdby(true);
		s5ka3dfx_cam_nrst(true);
		mdelay(100);
	} else {
		s5ka3dfx_cam_stdby(false);
		s5ka3dfx_cam_nrst(false);
		s3c_gpio_cfgpin(S5PV210_GPE1(3), 0);
		s5ka3dfx_ldo_en(false);
	}

	return 0;
#endif

	if (onoff)
		s5k6aafx_power_on();
	else
	{
		s5k6aafx_power_off();
		s3c_i2c0_force_stop();	
	}

	return 0;
}

int s5k6aafx_power_reset(void)
{
	s5k6aafx_power_en(0);
	s5k6aafx_power_en(1);

	return 0;
}
static struct s5k6aafx_platform_data s5k6aafx_plat = {
	.default_width = 800,
	.default_height = 600,
	.pixelformat = V4L2_PIX_FMT_UYVY,
};

static struct i2c_board_info s5k6aafx_i2c_info = {
	I2C_BOARD_INFO("S5K6AAFX", 0x78 >> 1),
	.platform_data = &s5k6aafx_plat,
};

static struct s3c_platform_camera s5k6aafx = {
	.id		= CAMERA_PAR_A,
	.type		= CAM_TYPE_ITU,
	.fmt		= ITU_601_YCBCR422_8BIT,
	.order422	= CAM_ORDER422_8BIT_YCBYCR,
	.i2c_busnum	= 0,
	.info		= &s5k6aafx_i2c_info,
	.pixelformat	= V4L2_PIX_FMT_UYVY,
	.srclk_name	= "xusbxti",
	.clk_name	= "sclk_cam0",
	.clk_rate	= 24000000,
	.line_length	= 800,
	.width		= 800,
	.height		= 600,
	.window		= {
		.left	= 0,
		.top	= 0,
		.width	= 800,
		.height	= 600,
	},

	/* Polarity */
	.inv_pclk	= 0,
	.inv_vsync 	= 1,
	.inv_href	= 0,
	.inv_hsync	= 0,

	.initialized 	= 0,
	.cam_power	= s5k6aafx_power_en,
};
#endif

#ifdef CONFIG_VIDEO_ISX005
/**
 * isx005_ldo_en()
 * camera???Ü¨Í≥£Î´ó??????????ñÑÎµ????LDO???Íπ???Î±ÄÏ™??
 *
 * @param     en             LDO enable ???
 * @return    void
 * @remark    Function     Set_MAX8998_PM_OUTPUT_Voltage, Set_MAX8998_PM_REG, 
 *
 * Dec 07, 2009  initial revision
 */
static void isx005_ldo_en(bool en)
{
	printk("<MACHINE> isx005_ldo_en(%d)\n", en);

	if(en){
			
		/* Turn CAM_3M_1.2V on */
		if(HWREV <= 0x3) { // Under Rev0.3 
			Set_MAX8998_PM_OUTPUT_Voltage(LDO12, VCC_1p200);
			Set_MAX8998_PM_REG(ELDO12, 1);
			udelay(50);
		} else if(HWREV >=0x4) { //Over Rev0.4
			Set_MAX8998_PM_OUTPUT_Voltage(LDO14, VCC_1p200);
			Set_MAX8998_PM_REG(ELDO14, 1);
			udelay(50);
		}

		/* Turn CAM_IO_2.8V on */
		Set_MAX8998_PM_OUTPUT_Voltage(LDO11, VCC_2p800);
		Set_MAX8998_PM_REG(ELDO11, 1);
//cam-i2c		s3c_i2c0_cfg_gpio_pull_none();		
		udelay(50);

		/* Turn CAM_A_2.8V on */		
		Set_MAX8998_PM_OUTPUT_Voltage(LDO13, VCC_2p800);
		Set_MAX8998_PM_REG(ELDO13, 1);
		udelay(50);

		/* Turn CAM_AF_2.8V or 3.0V on */	
		if(HWREV <= 0x3) {  // Under Rev0.3
			Set_MAX8998_PM_OUTPUT_Voltage(LDO15, VCC_2p800);
			Set_MAX8998_PM_REG(ELDO15, 1);
		} else if(HWREV >= 0x4) {  //Over Rev0.4
			Set_MAX8998_PM_OUTPUT_Voltage(LDO15, VCC_3p000);
			Set_MAX8998_PM_REG(ELDO15, 1);
		}
	} else {
//cam-i2c		s3c_i2c0_cfg_gpio_pull_up();
//cam-i2c		mdelay(50);
		Set_MAX8998_PM_REG(ELDO13, 0);
		Set_MAX8998_PM_REG(ELDO15, 0);
		Set_MAX8998_PM_REG(ELDO11, 0);
		udelay(50);		
		if(HWREV <= 0x3) { // Under Rev0.3
			Set_MAX8998_PM_REG(ELDO12, 0);
		}
		else if(HWREV >= 0x4) {  //Over Rev0.4
			Set_MAX8998_PM_REG(ELDO14, 0);
		}

	}
}
/**
 * isx005_cam_stdby()
 * camera??enable ??ÔßêÔΩã?
 *
 * @param     en             enable ???
 * @return    err              gpio ??Ôß?ÍΩ?????çâ???Î°™Ìçî???err???Íæ©Îáë???ÔßêÔΩã?
 * @remark    Function     gpio_request, gpio_direction_output, gpio_set_value, msleep, gpio_free
 *
 * Dec 07, 2009  initial revision
 */
 
//static int isx005_cam_stdby(bool en)
int isx005_cam_stdby(bool en)
{
	int err;

	printk("<MACHINE> stdby(%d)\n", en);

	/* CAM_MEGA_EN - GPJ1(2) */
	err = gpio_request(S5PV210_GPJ1(2), "GPJ1");
	if (err)
	{
		printk(KERN_ERR "failed to request GPJ1 for camera control\n");
		return err;
	}

	gpio_direction_output(S5PV210_GPJ1(2), 0);
	msleep(1);
	gpio_direction_output(S5PV210_GPJ1(2), 1);
	msleep(1);

	if(en)
	{
		gpio_set_value(S5PV210_GPJ1(2), 1);
	} 
	else 
	{
		gpio_set_value(S5PV210_GPJ1(2), 0); 
	}
	msleep(1);

	gpio_free(S5PV210_GPJ1(2));

	return 0;
}
/**
 * isx005_cam_nrst()
 * camera??reset ??ÔßêÔΩã?
 *
 * @param     nrst             reset ???
 * @return    err              gpio ??Ôß?ÍΩ?????çâ???Î°™Ìçî???err???Íæ©Îáë???ÔßêÔΩã?
 * @remark    Function     gpio_request, gpio_direction_output, gpio_set_value, msleep, gpio_free
 *
 * Dec 07, 2009  initial revision
 */
static int isx005_cam_nrst(bool nrst)
{
	int err;

	printk("isx005_cam_nrst(%d)\n", nrst);

	/* CAM_MEGA_nRST - GPJ1(5)*/
	err = gpio_request(S5PV210_GPJ1(5), "GPJ1");
	if (err) 
	{
		printk(KERN_ERR "failed to request GPJ1 for camera control\n");
		return err;
	}

	gpio_direction_output(S5PV210_GPJ1(5), 0);
	msleep(1);
	gpio_direction_output(S5PV210_GPJ1(5), 1);
	msleep(1);

	//gpio_set_value(S5PV210_GPJ1(5), 0);
	
	msleep(1);

	if (nrst)
	{
		gpio_set_value(S5PV210_GPJ1(5), 1);
		msleep(1);
	}
	else
	{
		gpio_set_value(S5PV210_GPJ1(5), 0);	
		msleep(1);

	}
	
	gpio_free(S5PV210_GPJ1(5));

	return 0;
}
/**
 * isx005_power_on()
 * camera??????Î°Ïæ??Áπûë?????πÏäñ????Íπ??ÔßêÔΩã?
 *
 * @param     en             enable ???
 * @return    err              gpio ??Ôß?ÍΩ?????çâ???Î°™Ìçî???err???Íæ©Îáë???ÔßêÔΩã?
 * @remark    Function     isx005_setup_port, isx005_ldo_en, s3c_gpio_cfgpin,
 *                                   isx005_cam_stdby, isx005_cam_nrst
 *
 * Dec 07, 2009  initial revision
 */
static int isx005_power_on(bool en)
{
	int err;

	isx005_ldo_en(en);

	msleep(2);

	s3c_gpio_cfgpin(S5PV210_GPE1(3), S5PV210_GPE1_3_CAM_A_CLKOUT * en);

	msleep(2);

	isx005_cam_nrst(en);

	return 0;
	}

static int isx005_power_off(void)
{
	int err;

	printk("isx005_power_off\n");

	/* CAM_MEGA_nRST - GPJ1(5) */
	err = gpio_request(GPIO_CAM_MEGA_nRST, "GPJ1");
	
	if(err) {
		printk(KERN_ERR "failed to request GPJ1 for camera control\n");
	
		return err;
	}

	/* CAM_MEGA_EN - GPJ1(2) */
	err = gpio_request(GPIO_CAM_MEGA_EN, "GPJ1");

	if(err) {
		printk(KERN_ERR "failed to request GPJ1 for camera control\n");
	
		return err;
	}

	// CAM_MEGA_EN - GPJ1(2) LOW
	gpio_direction_output(GPIO_CAM_MEGA_EN, 1);
	
	gpio_set_value(GPIO_CAM_MEGA_EN, 0);
	
	msleep(3);//mdelay(150);	

	// CAM_MEGA_nRST - GPJ1(5) LOW
	gpio_direction_output(GPIO_CAM_MEGA_nRST, 1);
	
	gpio_set_value(GPIO_CAM_MEGA_nRST, 0);
	
	mdelay(1);

	// Mclk disable
	s3c_gpio_cfgpin(GPIO_CAM_MCLK, 0);

	mdelay(1);

	isx005_ldo_en(FALSE);

	mdelay(1);
	
	gpio_free(GPIO_CAM_MEGA_nRST);
	gpio_free(GPIO_CAM_MEGA_EN);
	
	msleep(10);

	return 0;
}


static int isx005_power_en(int onoff)
{
	if(onoff){
		isx005_power_on(TRUE);
	} else {
		isx005_power_off();
		s3c_i2c0_force_stop();
	}

	return 0;
}

int isx005_power_reset(void)
{
	isx005_power_en(0);
	isx005_power_en(1);

	return 0;
}

static struct isx005_platform_data isx005_plat = {
	.default_width = 800,
	.default_height = 600,
	.pixelformat = V4L2_PIX_FMT_UYVY,
	.freq = 24000000,
	.is_mipi = 0,
};

static struct i2c_board_info  isx005_i2c_info = {	
//	I2C_BOARD_INFO("ISX005", 0x78 >> 1),
// I2C_BOARD_INFO("ISX005", 0x1A >> 1),
I2C_BOARD_INFO("ISX005", 0x1A ),
	.platform_data = &isx005_plat,
};

static struct s3c_platform_camera isx005 = {
	.id = CAMERA_PAR_A,
	.type = CAM_TYPE_ITU,
	.fmt = ITU_601_YCBCR422_8BIT,
	.order422 = CAM_ORDER422_8BIT_CBYCRY,
	.i2c_busnum = 0,
	.info = &isx005_i2c_info,
	.pixelformat = V4L2_PIX_FMT_UYVY,
	.srclk_name = "xusbxti",
	.clk_name = "sclk_cam0",
	.clk_rate = 24000000,
	.line_length = 1536,
	.width = 800,
	.height = 600,
	.window = {
		.left 	= 0,
		.top = 0,
		.width = 800,
		.height = 600,
	},

	/* Polarity */
	.inv_pclk = 0,
	.inv_vsync = 1,
	.inv_href = 0,
	.inv_hsync = 0,
	.initialized = 0,
	.cam_power = isx005_power_en,
};
#endif /* CONFIG_VIDEO_ISX005 */


#ifdef CONFIG_VIDEO_NM6XX //johnny.kim

static int nm6xx_power_en(int onoff)
{
	printk("==============  NM6XX Tuner Sensor ============== \n");

	return 0;
}

static struct nm6xx_platform_data nm6xx_plat = {
	.default_width = 320,
	.default_height = 240,
	.pixelformat = V4L2_PIX_FMT_YUYV,
	.freq = 13500000,
	.is_mipi = 0,
};

static struct i2c_board_info  nm6xx_i2c_info = {
//	I2C_BOARD_INFO("ISX005", 0x78 >> 1),
// I2C_BOARD_INFO("ISX005", 0x1A >> 1),
  I2C_BOARD_INFO("NM6XX", 0x1A ),
	.platform_data = &nm6xx_plat,
};

static struct s3c_platform_camera nm6xx = {
	.id = CAMERA_PAR_A,
	.type = CAM_TYPE_ITU,
	.fmt = ITU_601_YCBCR422_8BIT,//ITU_656_YCBCR422_8BIT,
	.order422 = CAM_ORDER422_8BIT_YCBYCR,//CAM_ORDER422_8BIT_CBYCRY,
	.i2c_busnum = 0,
	.info = &nm6xx_i2c_info,
	.pixelformat = V4L2_PIX_FMT_YUYV,
	.srclk_name = "xusbxti",
	.clk_name = "sclk_cam0",
	.clk_rate = 6750000,
	.line_length = 320,
	.width = 320,
	.height = 240,
	.window = {
		.left 	= 0,
		.top = 0,
		.width = 320,
		.height = 240,
	},

	/* Polarity */
	.inv_pclk = 0,
	.inv_vsync = 1,
	.inv_href = 0,
	.inv_hsync = 0,
	.initialized = 0,
	.cam_power = nm6xx_power_en,
};
#endif

/* Interface setting */
static struct s3c_platform_fimc fimc_plat = {
	.srclk_name     = "mout_mpll",
	.clk_name       = "sclk_fimc_lclk",
      	.clk_rate       = 166750000,
#if 0
#if defined(S5K4EA_ENABLED) || defined(S5K6AA_ENABLED)
	.default_cam	= CAMERA_CSI_C,
#else

#ifdef WRITEBACK_ENABLED
	.default_cam	= CAMERA_WB,
#elif CAM_ITU_CH_A
	.default_cam	= CAMERA_PAR_A,
#else
	.default_cam	= CAMERA_PAR_B,
#endif

#endif

#endif
	.camera		= {
#ifdef CONFIG_VIDEO_ISX005
		&isx005,
#endif /* CONFIG_VIDEO_ISX005 */
#ifdef CONFIG_VIDEO_S5K6AAFX
		&s5k6aafx,
#endif
#ifdef CONFIG_VIDEO_NM6XX //johnny.kim
    &nm6xx,
#endif
	},
	.hw_ver		= 0x43,
};
#endif

#if defined(CONFIG_HAVE_PWM)
static struct platform_pwm_backlight_data smdk_backlight_data = {
	.pwm_id  = 3,
	.max_brightness = 255,
	.dft_brightness = 255,
	.pwm_period_ns  = 78770,
};

static struct platform_device smdk_backlight_device = {
	.name      = "pwm-backlight",
	.id        = -1,
	.dev        = {
		.parent = &s3c_device_timer[3].dev,
		.platform_data = &smdk_backlight_data,
	},
};
static void __init smdk_backlight_register(void)
{
	int ret = platform_device_register(&smdk_backlight_device);
	if (ret)
		printk(KERN_ERR "smdk: failed to register backlight device: %d\n", ret);
}
#endif

#if defined(CONFIG_BLK_DEV_IDE_S3C)
static struct s3c_ide_platdata smdkv210_ide_pdata __initdata = {
	.setup_gpio     = s3c_ide_setup_gpio,
};
#endif

/* I2C0 */
static struct i2c_board_info i2c_devs0[] __initdata = {
#ifdef CONFIG_SND_SOC_WM8580
	{
		I2C_BOARD_INFO("wm8580", 0x1b),
	},
#endif
};

static struct i2c_board_info i2c_devs4[] __initdata = {
#ifdef CONFIG_SND_SOC_WM8994 
	{
		I2C_BOARD_INFO("wm8994", (0x34>>1)),
	},
#endif
#ifdef CONFIG_MHL_SII9234
	{
		I2C_BOARD_INFO("SII9234", 0x72>>1),
	},
	{
		I2C_BOARD_INFO("SII9234A", 0x7A>>1),
	},
	{
		I2C_BOARD_INFO("SII9234B", 0x92>>1),
	},
	{
		I2C_BOARD_INFO("SII9234C", 0xC8>>1),
	},
#endif
};

/* I2C1 */
static struct i2c_board_info i2c_devs1[] __initdata = {
    {
        I2C_BOARD_INFO("s5p_ddc", (0x74>>1)),
    },

};

/* I2C2 */
static struct i2c_board_info i2c_devs6[] __initdata = {
	{
		/* The address is 0xCC used since SRAD = 0 */
		I2C_BOARD_INFO("max8998", (0xCC >> 1)),
		.platform_data = &max8998_platform_data,
	},

	{
		I2C_BOARD_INFO("rtc_max8998", (0x0D >> 1)),
	},
};

static struct i2c_board_info i2c_devs7[] __initdata = {
	{
		I2C_BOARD_INFO("smb136", 0x9A >> 1),
	}, 
};

static struct i2c_board_info i2c_devs8[] __initdata = {
#if defined(CONFIG_FB_S3C_CMC623)
	{
		I2C_BOARD_INFO("sec_tune_cmc623_i2c", 0x38),
	}, 
#endif
};

static struct i2c_board_info i2c_devs9[] __initdata = {
	{
		I2C_BOARD_INFO("fuelgauge", (0x6D >> 1)),
	},
};

static struct i2c_board_info i2c_devs12[] __initdata = {
	{
		I2C_BOARD_INFO("fsa9480", (0x4A >> 1)),
	},
};

#if defined(CONFIG_TARGET_LOCALE_LTN)  // CYS_ 2010.05.07
	static struct i2c_board_info i2c_devs13[] __initdata = {
		{
			I2C_BOARD_INFO("nmi625", 0x61),
		},
	};
#endif


#ifdef CONFIG_DM9000
static void __init smdkv210_dm9000_set(void)
{
	unsigned int tmp;

	tmp = ((0<<28)|(0<<24)|(5<<16)|(0<<12)|(0<<8)|(0<<4)|(0<<0));
	__raw_writel(tmp, (S5P_SROM_BW+0x18));

	tmp = __raw_readl(S5P_SROM_BW);
	tmp &= ~(0xf << 20);

#ifdef CONFIG_DM9000_16BIT
	tmp |= (0x1 << 20);
#else
	tmp |= (0x2 << 20);
#endif
	__raw_writel(tmp, S5P_SROM_BW);

	tmp = __raw_readl(S5PV210_MP01CON);
	tmp &= ~(0xf << 20);
	tmp |= (2 << 20);

	__raw_writel(tmp, S5PV210_MP01CON);
}
#endif

#ifdef CONFIG_ANDROID_PMEM
static struct android_pmem_platform_data pmem_pdata = {
	.name = "pmem",
	.no_allocator = 1,
	.cached = 1,
	.start = 0, // will be set during proving pmem driver.
	.size = 0 // will be set during proving pmem driver.
};

static struct android_pmem_platform_data pmem_gpu1_pdata = {
   .name = "pmem_gpu1",
   .no_allocator = 1,
   .cached = 1,
   .buffered = 1,
   .start = 0,
   .size = 0,
};

static struct android_pmem_platform_data pmem_adsp_pdata = {
   .name = "pmem_adsp",
   .no_allocator = 1,
   .cached = 1,
   .buffered = 1,
   .start = 0,
   .size = 0,
};

static struct platform_device pmem_device = {
   .name = "android_pmem",
   .id = 0,
   .dev = { .platform_data = &pmem_pdata },
};

static struct platform_device pmem_gpu1_device = {
	.name = "android_pmem",
	.id = 1,
	.dev = { .platform_data = &pmem_gpu1_pdata },
};

static struct platform_device pmem_adsp_device = {
	.name = "android_pmem",
	.id = 2,
	.dev = { .platform_data = &pmem_adsp_pdata },
};

static void __init android_pmem_set_platdata(void)
{
	pmem_pdata.start = (u32)s3c_get_media_memory_bank(S3C_MDEV_PMEM, 0);
	pmem_pdata.size = (u32)s3c_get_media_memsize_bank(S3C_MDEV_PMEM, 0);

	pmem_gpu1_pdata.start = (u32)s3c_get_media_memory_bank(S3C_MDEV_PMEM_GPU1, 0);
	pmem_gpu1_pdata.size = (u32)s3c_get_media_memsize_bank(S3C_MDEV_PMEM_GPU1, 0);

	pmem_adsp_pdata.start = (u32)s3c_get_media_memory_bank(S3C_MDEV_PMEM_ADSP, 0);
	pmem_adsp_pdata.size = (u32)s3c_get_media_memsize_bank(S3C_MDEV_PMEM_ADSP, 0);
}
#endif



/**********************************/
/* add lcd related component - start >> */
#ifdef CONFIG_FB_S3C_LVDS
#ifdef CONFIG_TARGET_LOCALE_USAGSM
//P1_ATT PCLK -> 47.6MHz
static struct s3cfb_lcd lvds = {
        .width = 1024,
        .height = 600,
		.p_width = 154,
		.p_height = 90,
		.bpp = 32,
        .freq = 60,

        .timing = {
                .h_fp = 100,//50,	//179,	//.h_fp = 79,
                .h_bp = 80,//30,	//225,	//.h_bp = 200,
                .h_sw = 50,//20,	//40,
                .v_fp = 10,//6,	//10,
                .v_fpe = 1,
                .v_bp = 11,//5,	//11,
                .v_bpe = 1,
                .v_sw = 10,// 4,	//10,

        },

        .polarity = {
                .rise_vclk = 0,
                .inv_hsync = 1,
                .inv_vsync = 1,
                .inv_vden = 0,
        },
};
#else
static struct s3cfb_lcd lvds = {
        .width = 1024,
        .height = 600,
		.p_width = 154,
		.p_height = 90,
		.bpp = 32,
        .freq = 60,

        .timing = {
                .h_fp = 142,	//50,	//179,	//.h_fp = 79,
                .h_bp = 210,	//30,	//225,	//.h_bp = 200,
                .h_sw = 50,		//20,	//40,
                .v_fp = 10,	//6,	//10,
                .v_fpe = 1,
                .v_bp = 11,	//5,	//11,
                .v_bpe = 1,
                .v_sw = 10,	// 4,	//10,

        },

        .polarity = {
                .rise_vclk = 0,
                .inv_hsync = 1,
                .inv_vsync = 1,
                .inv_vden = 0,
        },
};
#endif
static void lvds_cfg_gpio(struct platform_device *pdev)
{
        int i,err;

        for (i = 0; i < 8; i++) {
                s3c_gpio_cfgpin(S5PV210_GPF0(i), S3C_GPIO_SFN(2));
                s3c_gpio_setpull(S5PV210_GPF0(i), S3C_GPIO_PULL_NONE);
        }

        for (i = 0; i < 8; i++) {
                s3c_gpio_cfgpin(S5PV210_GPF1(i), S3C_GPIO_SFN(2));
                s3c_gpio_setpull(S5PV210_GPF1(i), S3C_GPIO_PULL_NONE);
        }

        for (i = 0; i < 8; i++) {
                s3c_gpio_cfgpin(S5PV210_GPF2(i), S3C_GPIO_SFN(2));
                s3c_gpio_setpull(S5PV210_GPF2(i), S3C_GPIO_PULL_NONE);
        }

        for (i = 0; i < 4; i++) {
                s3c_gpio_cfgpin(S5PV210_GPF3(i), S3C_GPIO_SFN(2));
                s3c_gpio_setpull(S5PV210_GPF3(i), S3C_GPIO_PULL_NONE);
        }

        /* mDNIe SEL: why we shall write 0x2 ? */
		
#ifndef CONFIG_FB_S3C_MDNIE
        writel(0x2, S5P_MDNIE_SEL);
#else
        writel(0x1, S5P_MDNIE_SEL);
#endif

        /* drive strength to max */
//        writel(0xffffffff, S5P_VA_GPIO + 0x12c);
//        writel(0xffffffff, S5P_VA_GPIO + 0x14c);
//        writel(0xffffffff, S5P_VA_GPIO + 0x16c);
//        writel(0x000000ff, S5P_VA_GPIO + 0x18c);
        writel(0x5555557f, S5P_VA_GPIO + 0x12c);
        writel(0x55555555, S5P_VA_GPIO + 0x14c);
        writel(0x55555555, S5P_VA_GPIO + 0x16c);
        writel(0x00000055, S5P_VA_GPIO + 0x18c);

/*
        err = gpio_request(S5PV210_MP04(1), "LVDS_RST");
        if (err) {
                printk(KERN_ERR "failed to request MP04(1) for "
                        "lcd reset control\n");
                return;
        }

        gpio_direction_output(S5PV210_MP04(1), 1);
        msleep(100);

        gpio_set_value(S5PV210_MP04(1), 0);
        msleep(10);

	gpio_free(S5PV210_MP04(1));
*/
}

static void lvds_backlight_start(void)
{
	if(HWREV < 0xE)
	{
	        gpio_direction_output(GPIO_LCD_BACKLIGHT_EN, 1);
	        udelay(30);

	        gpio_set_value(GPIO_LCD_BACKLIGHT_EN, 1);
	        udelay(500);
	}
}

static void lvds_backlight_setaddress(int j)
{
 	if(HWREV < 0xE)
	{
	        while(j>0) {
	                gpio_set_value(GPIO_LCD_BACKLIGHT_EN, 0);
	                udelay(30);

	                gpio_set_value(GPIO_LCD_BACKLIGHT_EN, 1);
	                udelay(30);

	                j--;
	        }
	}
}

static void lvds_backlight_setdata(int j)
{
	if(HWREV < 0xE)
	{
	        while(j>0) {
	                gpio_set_value(GPIO_LCD_BACKLIGHT_EN, 0);
	                udelay(30);

	                gpio_set_value(GPIO_LCD_BACKLIGHT_EN, 1);
	                udelay(30);

	                j--;
	        }
	}
}




static int lvds_backlight_on(struct platform_device *pdev)
{
        int err;

	err = gpio_request(GPIO_LCD_LDO_EN, "LCD_LDO_EN");

	if (err) {
		printk(KERN_ERR "failed to request GPJ2_6 for "
				"lcd_ldo_en control\n");
		return err;
	}

#if !defined(CONFIG_TARGET_LOCALE_LTN) // CYS_ 2010.07.02 for Camera Preview
	err = gpio_request(GPIO_MLCD_ON, "MLCD_ON");

	if (err) {
		printk(KERN_ERR "failed to request GPJ3 for "
				"lcd backlight control\n");
		gpio_free(GPIO_LCD_LDO_EN);
		return err;
	}

        gpio_direction_output(GPIO_MLCD_ON, 1);
        gpio_set_value(GPIO_MLCD_ON, 1);
#endif
	gpio_direction_output(GPIO_LCD_LDO_EN, 1);
        gpio_set_value(GPIO_LCD_LDO_EN, 1);
        msleep(500);

#if !defined(CONFIG_TARGET_LOCALE_LTN) // CYS_ 2010.07.02 for Camera Preview
	gpio_free(GPIO_MLCD_ON);
#endif
	gpio_free(GPIO_LCD_LDO_EN);

        return 0;
}

static int lvds_reset_lcd(struct platform_device *pdev)
{
        int err;

/*
	err = gpio_request(GPIO_MLCD_RST, "MLCD_RST");
	if (err) {
		printk(KERN_ERR "failed to request MP0(5) for "
				"lcd reset control\n");
		return err;
	}

        gpio_direction_output(GPIO_MLCD_RST, 1);
        msleep(100);

        gpio_set_value(GPIO_MLCD_RST, 0);
        msleep(10);

        gpio_set_value(GPIO_MLCD_RST, 1);
        msleep(10);

        gpio_set_value(GPIO_MLCD_RST, 0);
        msleep(10);

	gpio_free(GPIO_MLCD_RST);
*/
/*
        err = gpio_request(S5PV210_MP04(1), "LVDS_RST");
        if (err) {
                printk(KERN_ERR "failed to request MP04(1) for "
                        "lcd reset control\n");
                return err;
        }

        gpio_set_value(S5PV210_MP04(1), 1);
        msleep(10);

	gpio_free(S5PV210_MP04(1));
*/
#if 0
    err = gpio_request(GPIO_LCD_BACKLIGHT_EN, "GPIO_LCD_BACKLIGHT_EN");
    if (err) {
            printk(KERN_ERR "failed to request GPIO_LCD_BACKLIGHT_EN for "
                    "lcd reset control\n");
            return err;
    }
#endif

	// if LCD is PWM type, it doesn't control 
	if(get_machine_type() == MACHINE_P1_TFT && HWREV < 0x9)
		{
        printk(KERN_NOTICE "start LVDS Backlight control\n");
        lvds_backlight_start();
        lvds_backlight_setaddress(17);
        lvds_backlight_start();
        lvds_backlight_setdata(3);

        lvds_backlight_start();
        lvds_backlight_setaddress(18);
        lvds_backlight_start();
        lvds_backlight_setdata(3);
        lvds_backlight_start();
        lvds_backlight_setaddress(19);
        lvds_backlight_start();
        lvds_backlight_setdata(4);

        lvds_backlight_start();
        lvds_backlight_setaddress(20);
        lvds_backlight_start();
        lvds_backlight_setdata(7);
        lvds_backlight_start();
        printk(KERN_NOTICE "end LVDS Backlight control\n");
		}

	//gpio_free(GPIO_LCD_BACKLIGHT_EN);

        return 0;
}





static struct s3c_platform_fb lvds_data __initdata = {
	.hw_ver = 0x60,
	.clk_name = "sclk_fimd",		//"lcd",
	.nr_wins = 5,
	.default_win = CONFIG_FB_S3C_DEFAULT_WINDOW,
	.swap = FB_SWAP_HWORD | FB_SWAP_WORD,

	.lcd = &lvds,
	.cfg_gpio = lvds_cfg_gpio,
	.backlight_on = lvds_backlight_on,
	.reset_lcd = lvds_reset_lcd,
};

#define P1P2_BACKLIGHT_EN       S5PV210_MP05(1)

static void backlight_init(void)
{

}

/* nt39411 backlight */
static struct swi_board_info swi_board_info[] __initdata = {
        {
                .name = "nt39411b_bl",
                .controller_data = P1P2_BACKLIGHT_EN,
                .low_period = 30,
                .high_period = 30,
                .init = backlight_init,
        },
};

static struct platform_device sec_device_lms700 = {
	.name   = "lms700",
	.id		= -1,
};

static struct platform_device mdnie_pwm_backlight = {
	.name   = "mdnie_pwm_bl",
	.id		= -1,
};

#if defined(CONFIG_FB_S3C_CMC623)
static struct platform_device cmc623_pwm_backlight = {
	.name   = "cmc623_pwm_bl",
	.id		= -1,
};
#endif

#endif

#ifdef CONFIG_FB_S3C_AMS701KA

static struct s3cfb_lcd ams701ka = {
        .width = 1024,
        .height = 600,
		.p_width = 0,
		.p_height = 0,
        .bpp = 24,
        .freq = 60,

        .timing = {
                .h_fp = 48,
                .h_bp = 114,
                .h_sw = 30,
                .v_fp = 8,
                .v_fpe = 1,
                .v_bp = 6,
                .v_bpe = 1,
                .v_sw = 2,
        },

        .polarity = {
                .rise_vclk = 0,
                .inv_hsync = 1,
                .inv_vsync = 1,
                .inv_vden = 0,
        },
};

static void ams701ka_cfg_gpio(struct platform_device *pdev)
{
        int i;

	printk("AMS701KA_CFG_GPIO!!!!!!!\n");

        for (i = 0; i < 8; i++) {
                s3c_gpio_cfgpin(S5PV210_GPF0(i), S3C_GPIO_SFN(2));
                s3c_gpio_setpull(S5PV210_GPF0(i), S3C_GPIO_PULL_NONE);
        }

        for (i = 0; i < 8; i++) {
                s3c_gpio_cfgpin(S5PV210_GPF1(i), S3C_GPIO_SFN(2));
                s3c_gpio_setpull(S5PV210_GPF1(i), S3C_GPIO_PULL_NONE);
        }

        for (i = 0; i < 8; i++) {
                s3c_gpio_cfgpin(S5PV210_GPF2(i), S3C_GPIO_SFN(2));
                s3c_gpio_setpull(S5PV210_GPF2(i), S3C_GPIO_PULL_NONE);
        }

        for (i = 0; i < 4; i++) {
                s3c_gpio_cfgpin(S5PV210_GPF3(i), S3C_GPIO_SFN(2));
                s3c_gpio_setpull(S5PV210_GPF3(i), S3C_GPIO_PULL_NONE);
        }

        /* mDNIe SEL: why we shall write 0x2 ? */
#ifndef CONFIG_FB_S3C_MDNIE
        writel(0x2, S5P_MDNIE_SEL);
#endif

        /* drive strength to max */
        writel(0xffffffff, S5P_VA_GPIO + 0x12c);
        writel(0xffffffff, S5P_VA_GPIO + 0x14c);
        writel(0xffffffff, S5P_VA_GPIO + 0x16c);
        writel(readl(S5P_VA_GPIO + 0x18c) | 0xffffff,
	                        S5P_VA_GPIO + 0x18c);

        /* DISPLAY_CS */
        s3c_gpio_cfgpin(S5PV210_MP01(1), S3C_GPIO_OUTPUT);
        s3c_gpio_setpull(S5PV210_MP01(1), S3C_GPIO_PULL_NONE);
        /* DISPLAY_CLK */
        s3c_gpio_cfgpin(S5PV210_MP04(1), S3C_GPIO_OUTPUT);
        s3c_gpio_setpull(S5PV210_MP04(1), S3C_GPIO_PULL_NONE);
        /* DISPLAY_SO */
        s3c_gpio_cfgpin(S5PV210_MP04(2), S3C_GPIO_INPUT);
        s3c_gpio_setpull(S5PV210_MP04(2), S3C_GPIO_PULL_NONE);
        /* DISPLAY_SI */
        s3c_gpio_cfgpin(S5PV210_MP04(3), S3C_GPIO_OUTPUT);
        s3c_gpio_setpull(S5PV210_MP04(3), S3C_GPIO_PULL_NONE);

}

static int ams701ka_backlight_on(struct platform_device *pdev)
{
	printk("AMS701KA_BACKLIGHT_ON!!!!!!!\n");

	gpio_direction_output(GPIO_LCD_LDO_EN, 1);
#if !defined(CONFIG_TARGET_LOCALE_LTN) // CYS_ 2010.07.02 for Camera Preview
	gpio_direction_output(GPIO_MLCD_ON, 1);
#endif
	mdelay(100);

	return 0;
}

static int ams701ka_reset_lcd(struct platform_device *pdev)
{
	static int first = 1;

	printk("AMS701KA_RESET_LCD!!!!!!!\n");
	if(first)	{
		gpio_direction_output(GPIO_MLCD_RST, 1);
	        msleep(10);

	        gpio_set_value(GPIO_MLCD_RST, 0);
	        msleep(10);

	        gpio_set_value(GPIO_MLCD_RST, 1);
	        msleep(30);
	}

	first = 0;

	return 0;
}

static struct s3c_platform_fb ams701ka_data __initdata = {
        .hw_ver = 0x60,
        .clk_name = "sclk_fimd",
        .nr_wins = 5,
        .default_win = CONFIG_FB_S3C_DEFAULT_WINDOW,
        .swap = FB_SWAP_HWORD | FB_SWAP_WORD,

	.lcd = &ams701ka,
        .cfg_gpio = ams701ka_cfg_gpio,
        .backlight_on = ams701ka_backlight_on,
        .reset_lcd = ams701ka_reset_lcd,
};

#define LCD_BUS_NUM 	3
#define DISPLAY_CS	S5PV210_MP01(1)
#define SUB_DISPLAY_CS	S5PV210_MP01(2)
#define DISPLAY_CLK	S5PV210_MP04(1)
#define DISPLAY_SI	S5PV210_MP04(3)

static struct spi_board_info spi_board_info_ams701ka[] __initdata = {
    	{
	    	.modalias	= "ams701ka",
		.platform_data	= NULL,
		.max_speed_hz	= 1200000,
		.bus_num	= LCD_BUS_NUM,
		.chip_select	= 0,
		.mode		= SPI_MODE_3,
		.controller_data = (void *)DISPLAY_CS,
	},
};

static struct spi_gpio_platform_data ams701ka_spi_gpio_data = {
	.sck	= DISPLAY_CLK,
	.mosi	= DISPLAY_SI,
	.miso	= -1,
	.num_chipselect	= 2,
};

static struct platform_device s3c_device_ams701ka_spi_gpio = {
	.name	= "spi_gpio",
	.id	= LCD_BUS_NUM,
	.dev	= {
		.parent		= &s3c_device_fb.dev,
		.platform_data	= &ams701ka_spi_gpio_data,
	},
};

#endif

/* << add lcd related component - end >> */
/************************************/

struct platform_device sec_device_battery = {
	.name	= "p1-battery",
	.id		= -1,
};

#ifdef CONFIG_30PIN_CONN
struct platform_device sec_device_connector = {
		.name	= "acc_con",
		.id 	= -1,
};
#endif
/**********************************/
/* added i2c- gpio-emulation - start >> */

/* i2c-gpio emulation platform_data */
static	struct	i2c_gpio_platform_data	i2c4_platdata = {
	.sda_pin		= GPIO_AP_SDA_18V,
	.scl_pin		= GPIO_AP_SCL_18V,
	.udelay			= 2,	/* 250KHz */		
	.sda_is_open_drain	= 0,
	.scl_is_open_drain	= 0,
	.scl_is_output_only	= 0,
//	.scl_is_output_only	= 1,
};

static struct platform_device s3c_device_i2c4 = {
	.name				= "i2c-gpio",
	.id					= 4,
	.dev.platform_data	= &i2c4_platdata,
};

static	struct	i2c_gpio_platform_data	i2c5_platdata = {
	.sda_pin		= GPIO_AP_I2C_SDA_28V,
	.scl_pin		= GPIO_AP_I2C_SCL_28V,
	.udelay			= 2,	/* 250KHz */		
//	.udelay			= 4,
	.sda_is_open_drain	= 0,
	.scl_is_open_drain	= 0,
	.scl_is_output_only	= 0,
//	.scl_is_output_only	= 1,
};

static struct platform_device s3c_device_i2c5 = {
	.name				= "i2c-gpio",
	.id					= 5,
	.dev.platform_data	= &i2c5_platdata,
};

static	struct	i2c_gpio_platform_data	i2c6_platdata = {
	.sda_pin		= GPIO_AP_PMIC_SDA,
	.scl_pin		= GPIO_AP_PMIC_SCL,
	.udelay			= 2,	/* 250KHz */		
	.sda_is_open_drain	= 0,
	.scl_is_open_drain	= 0,
	.scl_is_output_only	= 0,
};

static struct platform_device s3c_device_i2c6 = {
	.name				= "i2c-gpio",
	.id					= 6,
	.dev.platform_data	= &i2c6_platdata,
};

static	struct	i2c_gpio_platform_data	i2c7_platdata = {
#if defined(CONFIG_TARGET_LOCALE_LTN)  // CYS_ 2010.05.07
	.sda_pin		= GPIO_CHARGER_SDA_2_8V,
	.scl_pin		= GPIO_CHARGER_SCL_2_8V,
#else
	.sda_pin		= GPIO_MOT_R_SDA_2_8V,
	.scl_pin		= GPIO_MOT_R_SCL_2_8V,
#endif
	.udelay			= 2,	/* 250KHz */		
	.sda_is_open_drain	= 0,
	.scl_is_open_drain	= 0,
	.scl_is_output_only	= 0,
};

static struct platform_device s3c_device_i2c7 = {
	.name				= "i2c-gpio",
	.id					= 7,
	.dev.platform_data	= &i2c7_platdata,
};

static	struct	i2c_gpio_platform_data	i2c8_platdata = {  // P1_LSJ : DE10
	.sda_pin		= GPIO_CMC_SDA_18V,
	.scl_pin		= GPIO_CMC_SCL_18V,
	.udelay			= 1,	/* 500KHz */		
	.sda_is_open_drain	= 0,
	.scl_is_open_drain	= 0,
	.scl_is_output_only	= 0,
};

static struct platform_device s3c_device_i2c8 = {
	.name				= "i2c-gpio",
	.id					= 8,
	.dev.platform_data	= &i2c8_platdata,
};

#if defined(CONFIG_FB_S3C_CMC623)
static struct platform_device sec_device_tune_cmc623 = { // P1_LSJ : DE06
		.name			= "sec_tune_cmc623",
		.id 			= -1,
};
#endif

static	struct	i2c_gpio_platform_data	i2c9_platdata = {
	.sda_pin		= GPIO_FUEL_AP_SDA,
	.scl_pin		= GPIO_FUEL_AP_SCL,
	.udelay			= 2,	/* 250KHz */		
	.sda_is_open_drain	= 0,
	.scl_is_open_drain	= 0,
	.scl_is_output_only	= 0,
};

static struct platform_device s3c_device_i2c9 = {
	.name				= "i2c-gpio",
	.id					= 9,
	.dev.platform_data	= &i2c9_platdata,
};

static	struct	i2c_gpio_platform_data	i2c10_platdata = {
	.sda_pin		= GPIO_AP_SDA_2_8V,
	.scl_pin		= GPIO_AP_SCL_2_8V,
	.udelay			= 2,	/* 250KHz */		
	.sda_is_open_drain	= 0,
	.scl_is_open_drain	= 0,
	.scl_is_output_only	= 0,
};

static struct platform_device s3c_device_i2c10 = {
	.name				= "i2c-gpio",
	.id					= 10,
	.dev.platform_data	= &i2c10_platdata,
};

static	struct	i2c_gpio_platform_data	i2c11_platdata = {
	.sda_pin		= GPIO_TOUCH_KEY_SDA,
	.scl_pin		= GPIO_TOUCH_KEY_SCL,
	.udelay			= 2,	/* 250KHz */		
	.sda_is_open_drain	= 0,
	.scl_is_open_drain	= 0,
	.scl_is_output_only	= 0,
};

static struct platform_device s3c_device_i2c11 = {
	.name				= "i2c-gpio",
	.id					= 11,
	.dev.platform_data	= &i2c11_platdata,
};

static	struct	i2c_gpio_platform_data	i2c12_platdata = {
	.sda_pin		= GPIO_USB_SW_SDA,
	.scl_pin		= GPIO_USB_SW_SCL,
	.udelay			= 2,	/* 250KHz */		
	.sda_is_open_drain	= 0,
	.scl_is_open_drain	= 0,
	.scl_is_output_only	= 0,
};
static struct platform_device s3c_device_i2c12 = {
	.name				= "i2c-gpio",
	.id					= 12,
	.dev.platform_data	= &i2c12_platdata,
};

#if defined(CONFIG_TARGET_LOCALE_LTN) 
static	struct	i2c_gpio_platform_data	i2c13_platdata = {
	.sda_pin		= GPIO_ISDBT_SDA,
	.scl_pin		= GPIO_ISDBT_SCL,
	.udelay			= 2/*5*/, /* 250KHz */ //johnny.kim 100khz
	.sda_is_open_drain	= 0,
	.scl_is_open_drain	= 0,
	.scl_is_output_only	= 0,
};

static struct platform_device s3c_device_i2c13 = {
	.name				= "i2c-gpio",
	.id					= 13,
	.dev.platform_data	= &i2c13_platdata,
};
#endif



static struct platform_device bma020_accel = {
       .name  = "bma020-accelerometer",
       .id    = -1,
};

#if defined(CONFIG_KEYBOARD_P1)
static struct platform_device p1_keyboard = {
        .name  = "p1_keyboard",
        .id    = -1,
};
#endif

static struct sec_jack_port sec_jack_port[] = {
		{
		{ // HEADSET detect info
			.eint		=IRQ_EINT6, 
			.gpio		= GPIO_DET_35,	 
			.gpio_af	= GPIO_DET_35_AF  , 
			.low_active 	= 1
		},
		{ // SEND/END info
			.eint		= IRQ_EINT(30),
			.gpio		= GPIO_EAR_SEND_END, 
			.gpio_af	= GPIO_EAR_SEND_END_AF, 
			.low_active = 1
		}			
		}
};

static struct sec_jack_platform_data sec_jack_data = {
		.port			= sec_jack_port,
		.nheadsets		= ARRAY_SIZE(sec_jack_port),
};

static struct platform_device sec_device_jack = {
		.name			= "sec_jack",
		.id 			= -1,
		.dev			= {
				.platform_data	= &sec_jack_data,
		},
};


static struct sec_jack_port sec_jack_port_r04[] = {
		{
		{ // HEADSET detect info
			.eint		=IRQ_EINT8, 
			.gpio		= GPIO_DET_35_R04,	 
			.gpio_af	= GPIO_DET_35_R04_AF  , 
			.low_active 	= 1
		},
		{ // SEND/END info
			.eint		= IRQ_EINT12,
			.gpio		= GPIO_EAR_SEND_END_R04, 
			.gpio_af	= GPIO_EAR_SEND_END_R04_AF, 
			.low_active = 1
		}			
		}
};

static struct sec_jack_platform_data sec_jack_data_r04 = {
		.port			= sec_jack_port_r04,
		.nheadsets		= ARRAY_SIZE(sec_jack_port_r04),
};

static struct platform_device sec_device_jack_r04 = {
		.name			= "sec_jack",
		.id 			= -1,
		.dev			= {
				.platform_data	= &sec_jack_data_r04,
		},
};


/* << added i2c- gpio-emulation - end */
/*********************************/

/********************/
/* gpio-key - start >> */

static struct gpio_keys_button button_data[] = {
		{ KEY_POWER, GPIO_nPOWER, 1, "Power", EV_KEY, 0, 0, IRQ_EINT(22)},
		{ KEY_HOME, GPIO_HOME_KEY, 1, "Home", EV_KEY, 0, 0, IRQ_EINT(29)},
		{ KEY_VOLUMEUP, GPIO_KBR0, 1, "Volume Up", EV_KEY, 0, 0, IRQ_EINT(24)},
		{ KEY_VOLUMEDOWN, GPIO_KBR1, 1, "Volume Down", EV_KEY, 0, 0, IRQ_EINT(25)},
};

static struct gpio_keys_platform_data gpio_keys_data = {
		.buttons 	= button_data,
		.nbuttons	= ARRAY_SIZE(button_data),
		.rep		= 0,
};

static struct platform_device gpio_keys_device = {
        .name           = "gpio-keys",
        .id             = -1,
        .dev            = {
                .platform_data  = &gpio_keys_data,
        },
};

static struct gpio_keys_button button_data_rev05[] = {
		{ KEY_POWER, GPIO_nPOWER, 1, "Power", EV_KEY, 0, 0, IRQ_EINT(22)},
		{ KEY_VOLUMEUP, GPIO_KBR0, 1, "Volume Up", EV_KEY, 0, 0, IRQ_EINT(24)},
		{ KEY_VOLUMEDOWN, GPIO_KBR1, 1, "Volume Down", EV_KEY, 0, 0, IRQ_EINT(25)},
};

static struct gpio_keys_platform_data gpio_keys_data_rev05 = {
		.buttons 	= button_data_rev05,
		.nbuttons	= ARRAY_SIZE(button_data_rev05),
		.rep		= 0,
};

static struct platform_device gpio_keys_device_rev05 = {
        .name           = "gpio-keys",
        .id             = -1,
        .dev            = {
                .platform_data  = &gpio_keys_data_rev05,
        },
};

static void gpio_keys_cfg_gpio()
{

    if(HWREV <= 0x4)
    {
        s3c_gpio_cfgpin(GPIO_KBC1, S3C_GPIO_OUTPUT);
        s3c_gpio_setpull(GPIO_KBC1, S3C_GPIO_PULL_NONE);
        gpio_set_value(GPIO_KBC1, 0);

        s3c_gpio_cfgpin(GPIO_HOME_KEY, S3C_GPIO_INPUT);
        s3c_gpio_setpull(GPIO_HOME_KEY, S3C_GPIO_PULL_NONE);
    }
    
    s3c_gpio_cfgpin(GPIO_nPOWER, S3C_GPIO_INPUT);            
    s3c_gpio_setpull(GPIO_nPOWER, S3C_GPIO_PULL_NONE);	

    s3c_gpio_cfgpin(GPIO_KBR0, S3C_GPIO_INPUT);
    s3c_gpio_setpull(GPIO_KBR0, S3C_GPIO_PULL_NONE);

    s3c_gpio_cfgpin(GPIO_KBR1, S3C_GPIO_INPUT);
    s3c_gpio_setpull(GPIO_KBR1, S3C_GPIO_PULL_NONE);

}

/* << gpio-key -end */
/******************/

/**********************/
/* dpram platform device*/
struct platform_device sec_device_dpram = {
	.name	= "dpram-device",
	.id		= -1,
};
/*********************/

/********************/
/* bluetooth - start >> */

struct platform_device s3c_device_8998consumer = {
        .name             = "max8998-consumer",
        .id               = 0,
  	.dev = { .platform_data = &max8998_platform_data },
};

static struct platform_device	sec_device_rfkill = {
	.name = "bt_rfkill",
	.id	  = -1,
};

static struct platform_device	sec_device_btsleep = {
	.name = "bt_sleep",
	.id	  = -1,
};


/* << bluetooth -end */
/******************/

/**/
/* onedram */
static void onedram_cfg_gpio(void)
{
	//unsigned gpio_onedram_int_ap = S5PC11X_GPH1(3);
	s3c_gpio_cfgpin(GPIO_nINT_ONEDRAM_AP, S3C_GPIO_SFN(GPIO_nINT_ONEDRAM_AP_AF));
	s3c_gpio_setpull(GPIO_nINT_ONEDRAM_AP, S3C_GPIO_PULL_UP);
	set_irq_type(GPIO_nINT_ONEDRAM_AP, IRQ_TYPE_LEVEL_LOW);
}

static struct onedram_platform_data onedram_data = {
		.cfg_gpio = onedram_cfg_gpio,
		};

static struct resource onedram_res[] = {
	[0] = {
		.start = (S5PV210_PA_SDRAM + 0x05000000),
		.end = (S5PV210_PA_SDRAM + 0x05000000 + SZ_16M - 1),
		.flags = IORESOURCE_MEM,
		},
	[1] = {
		.start = IRQ_EINT11,
		.end = IRQ_EINT11,
		.flags = IORESOURCE_IRQ,
		},
	};

static struct platform_device onedram = {
		.name = "onedram",
		.id = -1,
		.num_resources = ARRAY_SIZE(onedram_res),
		.resource = onedram_res,
		.dev = {
			.platform_data = &onedram_data,
			},
		};

/* Modem control */
static void modemctl_cfg_gpio(void);

static struct modemctl_platform_data mdmctl_data = {
	.name = "xmm",
	.gpio_phone_on = GPIO_PHONE_ON,
	.gpio_phone_active = GPIO_PHONE_ACTIVE,
	.gpio_pda_active = GPIO_PDA_ACTIVE,
	.gpio_cp_reset = GPIO_CP_RST,
	.gpio_sim_ndetect = GPIO_SIM_nDETECT,
	.cfg_gpio = modemctl_cfg_gpio,
	};

static struct resource mdmctl_res[] = {
	[0] = {
		.start = IRQ_EINT15,
		.end = IRQ_EINT15,
		.flags = IORESOURCE_IRQ,
		},
	[1] = {
		.start = IRQ_EINT(27),
		.end = IRQ_EINT(27),
		.flags = IORESOURCE_IRQ,
		},
	};

static struct platform_device modemctl = {
		.name = "modemctl",
		.id = -1,
		.num_resources = ARRAY_SIZE(mdmctl_res),
		.resource = mdmctl_res,
		.dev = {
			.platform_data = &mdmctl_data,
			},
		};

static void modemctl_cfg_gpio(void)
{
	int err = 0;
	
	unsigned gpio_phone_on;
	
	if (HWREV < 7) {
		gpio_phone_on = mdmctl_data.gpio_phone_on;
	} else {
		mdmctl_data.gpio_phone_on =  NULL;
	}
	
	unsigned gpio_phone_active = mdmctl_data.gpio_phone_active;
	unsigned gpio_cp_rst = mdmctl_data.gpio_cp_reset;
	unsigned gpio_pda_active = mdmctl_data.gpio_pda_active;
	unsigned gpio_sim_ndetect = mdmctl_data.gpio_sim_ndetect;

	if (HWREV < 7) {
	err = gpio_request(gpio_phone_on, "PHONE_ON");
	if (err) {
		printk("fail to request gpio %s\n","PHONE_ON");
	} else {
		gpio_direction_output(gpio_phone_on, GPIO_LEVEL_LOW);
		s3c_gpio_setpull(gpio_phone_on, S3C_GPIO_PULL_NONE);
	}
	}
	
	err = gpio_request(gpio_cp_rst, "CP_RST");
	if (err) {
		printk("fail to request gpio %s\n","CP_RST");
	} else {
		gpio_direction_output(gpio_cp_rst, GPIO_LEVEL_LOW);
		s3c_gpio_setpull(gpio_cp_rst, S3C_GPIO_PULL_NONE);
	}
	err = gpio_request(gpio_pda_active, "PDA_ACTIVE");
	if (err) {
		printk("fail to request gpio %s\n","PDA_ACTIVE");
	} else {
		gpio_direction_output(gpio_pda_active, GPIO_LEVEL_HIGH);
		s3c_gpio_setpull(gpio_pda_active, S3C_GPIO_PULL_NONE);
	}

	s3c_gpio_cfgpin(gpio_phone_active, S3C_GPIO_SFN(0xF));
	s3c_gpio_setpull(gpio_phone_active, S3C_GPIO_PULL_NONE);
	set_irq_type(gpio_phone_active, IRQ_TYPE_EDGE_BOTH);

	s3c_gpio_cfgpin(gpio_sim_ndetect, S3C_GPIO_SFN(0xF));
	s3c_gpio_setpull(gpio_sim_ndetect, S3C_GPIO_PULL_NONE);
	set_irq_type(gpio_sim_ndetect, IRQ_TYPE_EDGE_BOTH);
}

static struct platform_device *smdkc110_devices[] __initdata = {
#ifdef CONFIG_MTD_ONENAND
	&s3c_device_onenand,
#endif
	&s3c_device_8998consumer,
#ifdef CONFIG_FB_S3C
	&s3c_device_fb,
#endif
#if defined(CONFIG_KEYBOARD_P1)
	&p1_keyboard,
#endif	
#ifdef CONFIG_TOUCHSCREEN_S3C
	&s3c_device_ts,
#endif
#ifdef CONFIG_S5PV210_ADCTS
	&s3c_device_adcts,
#endif
#ifdef CONFIG_DM9000
	&s5p_device_dm9000,
#endif
#ifdef CONFIG_S3C2410_WATCHDOG
	&s3c_device_wdt,
#endif
#if defined(CONFIG_BLK_DEV_IDE_S3C)
	&s3c_device_cfcon,
#endif
#ifdef CONFIG_RTC_DRV_S3C
	&s5p_device_rtc,
#endif

#ifdef CONFIG_HAVE_PWM
//	&s3c_device_timer[0],
	&s3c_device_timer[1],
	&s3c_device_timer[2],
	&s3c_device_timer[3],
#endif	

#ifdef CONFIG_SND_S3C24XX_SOC
	&s3c64xx_device_iis0,
#endif
#ifdef CONFIG_VIDEO_FIMC
	&s3c_device_fimc0,
	&s3c_device_fimc1,
	&s3c_device_fimc2,
	&s3c_device_csis,
	&s3c_device_ipc,
#endif
#ifdef CONFIG_VIDEO_MFC50
	&s3c_device_mfc,
#endif

#ifdef CONFIG_VIDEO_JPEG_V2
	&s3c_device_jpeg,
#endif

#ifdef CONFIG_VIDEO_ROTATOR
	&s5p_device_rotator,
#endif

#ifdef CONFIG_USB
#ifndef CONFIG_USB_S3C_OTG_HOST
		&s3c_device_usb_ehci,
		&s3c_device_usb_ohci,
#endif
#endif
#ifdef CONFIG_USB_S3C_OTG_HOST
		&s3c_device_usb_otghcd,
#endif
#ifdef CONFIG_USB_GADGET
		&s3c_device_usbgadget,
#endif

	&s3c_device_qtts,
#ifdef CONFIG_USB_ANDROID
	&s3c_device_android_usb,
	&s3c_device_usb_mass_storage,
#endif

//	&modemctl,
//	&onedram,
//	&sec_device_dpram,

	&s3c_device_i2c0,
	&s3c_device_i2c1,
	&s3c_device_i2c2,

	&s3c_device_i2c4,
	&s3c_device_i2c5,
	&s3c_device_i2c6,
    &s3c_device_i2c8,
	&s3c_device_i2c9,
	&s3c_device_i2c10,
	&s3c_device_i2c12,
#if defined(CONFIG_TARGET_LOCALE_LTN) 
	&s3c_device_i2c13,
#endif
#ifdef CONFIG_S3C_DEV_HSMMC
	&s3c_device_hsmmc0,
#endif
#ifdef CONFIG_S3C_DEV_HSMMC2
	&s3c_device_hsmmc2,
#endif
#ifdef CONFIG_S3C_DEV_HSMMC3
	&s3c_device_hsmmc3,
#endif
/*
#ifdef CONFIG_VIDEO_TV20
	&s5p_device_tvout,
	&s5p_device_cec,
	&s5p_device_hpd,
#endif
*/
#ifdef CONFIG_ANDROID_PMEM
	&pmem_device,
	&pmem_gpu1_device,
	&pmem_adsp_device,
#endif
	&sec_device_battery,
#ifdef CONFIG_VIDEO_G2D
	&s5p_device_g2d,
#endif
	&sec_device_rfkill,
	&sec_device_btsleep,
/*	
#ifdef CONFIG_30PIN_CONN
	&sec_device_connector,
#endif
*/
#ifdef CONFIG_VIDEO_TSI
	&s3c_device_tsi,
#endif
};

static struct platform_device *smdkc110_old_devices[] __initdata = {
#ifdef CONFIG_MTD_ONENAND
	&s3c_device_onenand,
#endif
	&s3c_device_8998consumer,
#ifdef CONFIG_FB_S3C
	&s3c_device_fb,
#endif
#if defined(CONFIG_KEYBOARD_P1)
	&p1_keyboard,
#endif	
#ifdef CONFIG_TOUCHSCREEN_S3C
	&s3c_device_ts,
#endif
#ifdef CONFIG_S5PV210_ADCTS
	&s3c_device_adcts,
#endif
#ifdef CONFIG_DM9000
	&s5p_device_dm9000,
#endif
#ifdef CONFIG_S3C2410_WATCHDOG
	&s3c_device_wdt,
#endif
#if defined(CONFIG_BLK_DEV_IDE_S3C)
	&s3c_device_cfcon,
#endif
#ifdef CONFIG_RTC_DRV_S3C
	&s5p_device_rtc,
#endif

#ifdef CONFIG_HAVE_PWM
//	&s3c_device_timer[0],
	&s3c_device_timer[1],
	&s3c_device_timer[2],
	&s3c_device_timer[3],
#endif	

#ifdef CONFIG_SND_S3C24XX_SOC
	&s3c64xx_device_iis0,
#endif
#ifdef CONFIG_VIDEO_FIMC
	&s3c_device_fimc0,
	&s3c_device_fimc1,
	&s3c_device_fimc2,
	&s3c_device_csis,
	&s3c_device_ipc,
#endif
#ifdef CONFIG_VIDEO_MFC50
	&s3c_device_mfc,
#endif

#ifdef CONFIG_VIDEO_JPEG_V2
	&s3c_device_jpeg,
#endif

#ifdef CONFIG_VIDEO_ROTATOR
	&s5p_device_rotator,
#endif

#ifdef CONFIG_USB
#ifndef CONFIG_USB_S3C_OTG_HOST
		&s3c_device_usb_ehci,
		&s3c_device_usb_ohci,
#endif
#endif
#ifdef CONFIG_USB_S3C_OTG_HOST
		&s3c_device_usb_otghcd,
#endif
#ifdef CONFIG_USB_GADGET
		&s3c_device_usbgadget,
#endif

	&s3c_device_qtts,
#ifdef CONFIG_USB_ANDROID
	&s3c_device_android_usb,
	&s3c_device_usb_mass_storage,
#endif

//	&modemctl,
//	&onedram,
//	&sec_device_dpram,

	&s3c_device_i2c0,
	&s3c_device_i2c1,
	&s3c_device_i2c2,

	&s3c_device_i2c4,
	&s3c_device_i2c5,
	&s3c_device_i2c6,
    &s3c_device_i2c8,
	&s3c_device_i2c9,
	&s3c_device_i2c10,
	&s3c_device_i2c12,

#ifdef CONFIG_S3C_DEV_HSMMC
	&s3c_device_hsmmc0,
#endif
#ifdef CONFIG_S3C_DEV_HSMMC1
	&s3c_device_hsmmc1,
#endif
#ifdef CONFIG_S3C_DEV_HSMMC2
	&s3c_device_hsmmc2,
#endif
#ifdef CONFIG_S3C_DEV_HSMMC3
	&s3c_device_hsmmc3,
#endif
/*
#ifdef CONFIG_VIDEO_TV20
	&s5p_device_tvout,
	&s5p_device_cec,
	&s5p_device_hpd,
#endif
*/
#ifdef CONFIG_ANDROID_PMEM
	&pmem_device,
	&pmem_gpu1_device,
	&pmem_adsp_device,
#endif
	&sec_device_battery,
#ifdef CONFIG_VIDEO_G2D
	&s5p_device_g2d,
#endif
	&sec_device_rfkill,
	&sec_device_btsleep,
/*	
#ifdef CONFIG_30PIN_CONN
	&sec_device_connector,
#endif
*/
#ifdef CONFIG_VIDEO_TSI
	&s3c_device_tsi,
#endif
};

static void __init smdkc110_fixup(struct machine_desc *desc,
                                       struct tag *tags, char **cmdline,
                                       struct meminfo *mi)
{
       mi->bank[0].start = 0x30000000;
       mi->bank[0].size = 80 * SZ_1M;
       mi->bank[0].node = 0;

       mi->bank[1].start = 0x40000000;
       mi->bank[1].size = 256 * SZ_1M;
       mi->bank[1].node = 1;

#if !defined (CONFIG_DDR_CONFIG_4G)
	mi->nr_banks = 2;
#else
	mi->bank[2].start = 0x50000000;
	mi->bank[2].size = 256 * SZ_1M;
	mi->bank[2].node = 2;
	
	mi->nr_banks = 3;
#endif
}

static void __init smdkc110_map_io(void)
{
	s5p_init_io(NULL, 0, S5P_VA_CHIPID);
	s3c24xx_init_clocks(24000000);
	s5pv210_gpiolib_init();
	s3c24xx_init_uarts(smdkv210_uartcfgs, ARRAY_SIZE(smdkv210_uartcfgs));
	s5pv210_reserve_bootmem();

#ifdef CONFIG_MTD_ONENAND
	s3c_device_onenand.name = "s5pc110-onenand";
#endif
}

#ifdef CONFIG_S3C_SAMSUNG_PMEM
static void __init s3c_pmem_set_platdata(void)
{
	pmem_pdata.start = s3c_get_media_memory_bank(S3C_MDEV_PMEM, 1);
	pmem_pdata.size = s3c_get_media_memsize_bank(S3C_MDEV_PMEM, 1);
}
#endif

/* this function are used to detect s5pc110 chip version temporally */

int s5pc110_version ;

void _hw_version_check(void)
{
	void __iomem * phy_address ;
	int temp; 

	phy_address = ioremap (0x40,1);

	temp = __raw_readl(phy_address);


	if (temp == 0xE59F010C)
	{
		s5pc110_version = 0;
	}
	else
	{
		s5pc110_version=1 ;
	}
	printk("S5PC110 Hardware version : EVT%d \n",s5pc110_version);
	
	iounmap(phy_address);
}

/* Temporally used
 * return value 0 -> EVT 0
 * value 1 -> evt 1
 */

int hw_version_check(void)
{
	return s5pc110_version ;
}
EXPORT_SYMBOL(hw_version_check);

/*************************************/
/* add board specific component - start >> */

static machine_type_t get_machine_type(void)
{
	int err;

	if (machine_type != MACHINE_UNKNOWN)
		return machine_type;

	err = gpio_request(GPIO_LCD_ID, "LCD_ID");

	if (err)	{
		printk(KERN_ERR "failed to request LCD_ID\n");
		return MACHINE_UNKNOWN;
	}

        gpio_direction_input(GPIO_LCD_ID);
        s3c_gpio_setpull(GPIO_LCD_ID, S3C_GPIO_PULL_NONE);

		udelay(10);

	if (gpio_get_value(GPIO_LCD_ID))
		machine_type = MACHINE_P1_TFT;
	else
		machine_type = MACHINE_P1_AMOLED;

//	machine_type = MACHINE_P1_TFT;

	gpio_free(GPIO_LCD_ID);

	return machine_type;
}

static int arise_notifier_call(struct notifier_block *this, unsigned long code, void *_cmd)
{

	int mode = REBOOT_MODE_NONE;

	if ((code == SYS_RESTART) && _cmd) {
		if (!strcmp((char *)_cmd, "arm11_fota"))
			mode = REBOOT_MODE_ARM11_FOTA;
		else if (!strcmp((char *)_cmd, "arm9_fota"))
			mode = REBOOT_MODE_ARM9_FOTA;
		else if (!strcmp((char *)_cmd, "recovery")) 
			mode = REBOOT_MODE_RECOVERY;
		else if (!strcmp((char *)_cmd, "download")) 
			mode = REBOOT_MODE_DOWNLOAD;
	}

	if(code != SYS_POWER_OFF) {
		if(sec_set_param_value)	{
			sec_set_param_value(__REBOOT_MODE, &mode);
		}
	}

	return NOTIFY_DONE;
}


static struct notifier_block arise_reboot_notifier = {
	.notifier_call = arise_notifier_call,
};

/* sec class init  - qt touch uses sec_class*/
static void p1_switch_init(void)
{
	sec_class = class_create(THIS_MODULE, "sec");
	
	if (IS_ERR(sec_class))
		pr_err("Failed to create class(sec)!\n");

	switch_dev = device_create(sec_class, NULL, 0, NULL, "switch");
	
	if (IS_ERR(switch_dev))
		pr_err("Failed to create device(switch)!\n");

};


#define S3C_GPIO_SETPIN_ZERO	0
#define S3C_GPIO_SETPIN_ONE		1
#define S3C_GPIO_SETPIN_NONE	2  // dont set the data pin

/*
 *
 * GPIO Initialization table. It has the following format
 * { pin number, pin configuration, pin value, pullup/down config,
 * 		driver strength, slew rate, sleep mode pin conf, sleep mode pullup/down config }
 * 
 * The table can be modified with the appropriate value for each pin. 
 */

// >>  define added for prevent error
// will be removed, when real functions are implemented
#define S3C_GPIO_DRVSTR_1X	(0)
#define S3C_GPIO_DRVSTR_2X	(1)
#define S3C_GPIO_DRVSTR_3X	(2)
#define S3C_GPIO_DRVSTR_4X	(3)

#define S3C_GPIO_SLEWRATE_FAST	(0)
#define S3C_GPIO_SLEWRATE_SLOW	(1)

#define S3C_GPIO_SLP_OUT0       ((__force s3c_gpio_pull_t)0x00)
#define S3C_GPIO_SLP_OUT1       ((__force s3c_gpio_pull_t)0x01)
#define S3C_GPIO_SLP_INPUT      ((__force s3c_gpio_pull_t)0x02)
#define S3C_GPIO_SLP_PREV       ((__force s3c_gpio_pull_t)0x03)
// << end

static unsigned int p1_gpio_table[][8] = {
	/* Off part */	
	// GPA0 ~ GPA1 : is done by UART driver early, so not modifying.
#if 0	
	{S5PV210_GPA0(0), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN, 
			S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_GPA0(1), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_GPA0(2), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_GPA0(3), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_GPA0(4), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_GPA0(5), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_GPA0(6), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_GPA0(7), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 

	// uart2 rx and tx..  done by uboot. So not modifying
	//{S5PV210_GPA1(0), 2, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
	//		 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
	//{S5PV210_GPA1(1), 2, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
	//		 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 

	{S5PV210_GPA1(2), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_GPA1(3), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
#endif
	{S5PV210_GPB(0), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_GPB(1), S3C_GPIO_OUTPUT, S3C_GPIO_SETPIN_ZERO, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
        {S5PV210_GPB(2), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_GPB(3), S3C_GPIO_OUTPUT, S3C_GPIO_SETPIN_ZERO, S3C_GPIO_PULL_NONE,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_PREV, S3C_GPIO_PULL_NONE}, 
        {S5PV210_GPB(4), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_GPB(5), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_GPB(6), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_GPB(7), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 

	{S5PV210_GPC0(0), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_GPC0(1), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_GPC0(2), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_GPC0(3), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_GPC0(4), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
#if defined(CONFIG_TARGET_LOCALE_LTN) // CYS_ 2010.06.24 move to cmc623_setting to bootloader 
	{S5PV210_GPC1(0), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_GPC1(1), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_GPC1(2), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_GPC1(3), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_GPC1(4), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
#endif
//	{S5PV210_GPD0(0), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
//			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_GPD0(1), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_GPD0(2), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
//        {S5PV210_GPD0(3), S3C_GPIO_OUTPUT, S3C_GPIO_SETPIN_ZERO, S3C_GPIO_PULL_NONE,
//			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPD1(0), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_GPD1(1), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_GPD1(2), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_GPD1(3), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_GPD1(4), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_GPD1(5), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 

	{S5PV210_GPE0(0), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_GPE0(1), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_GPE0(2), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_GPE0(3), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_GPE0(4), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_GPE0(5), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_GPE0(6), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_GPE0(7), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 

	{S5PV210_GPE1(0), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_GPE1(1), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_GPE1(2), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_GPE1(3), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_GPE1(4), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
#if 0
	{S5PV210_GPF0(0), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_GPF0(1), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_GPF0(2), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_GPF0(3), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_GPF0(4), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_GPF0(5), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_GPF0(6), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_GPF0(7), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 

	{S5PV210_GPF1(0), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_GPF1(1), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_GPF1(2), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_GPF1(3), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_GPF1(4), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_GPF1(5), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_GPF1(6), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_GPF1(7), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 

	{S5PV210_GPF2(0), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_GPF2(1), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_GPF2(2), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_GPF2(3), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_GPF2(4), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_GPF2(5), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_GPF2(6), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_GPF2(7), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 

	{S5PV210_GPF3(0), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_GPF3(1), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_GPF3(2), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_GPF3(3), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
#endif			 
        {S5PV210_GPF3(4), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_GPF3(5), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 

	{S5PV210_GPG0(0), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_GPG0(1), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_GPG0(2), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_GPG0(3), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_GPG0(4), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_GPG0(5), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_GPG0(6), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 

	{S5PV210_GPG1(0), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_GPG1(1), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_GPG1(2), S3C_GPIO_OUTPUT, S3C_GPIO_SETPIN_ZERO, S3C_GPIO_PULL_NONE,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_PREV, S3C_GPIO_PULL_NONE}, 
        {S5PV210_GPG1(3), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_GPG1(4), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_GPG1(5), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_GPG1(6), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 

	{S5PV210_GPG2(0), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_GPG2(1), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_GPG2(2), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_GPG2(3), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_GPG2(4), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_GPG2(5), S3C_GPIO_OUTPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_GPG2(6), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 

	{S5PV210_GPG3(0), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_GPG3(1), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
//        {S5PV210_GPG3(2), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
//			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_GPG3(3), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_GPG3(4), S3C_GPIO_OUTPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_NONE,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_GPG3(5), S3C_GPIO_OUTPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_NONE,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_GPG3(6), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 

	/* Alive part */
        {S5PV210_GPH0(0), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, 0, 0}, 
        {S5PV210_GPH0(1), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, 0, 0}, 
        {S5PV210_GPH0(2), S3C_GPIO_OUTPUT, S3C_GPIO_SETPIN_ZERO, S3C_GPIO_PULL_NONE,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, 0, 0}, 
        {S5PV210_GPH0(3), S3C_GPIO_OUTPUT, S3C_GPIO_SETPIN_ZERO, S3C_GPIO_PULL_NONE,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, 0, 0}, 
        {S5PV210_GPH0(4), S3C_GPIO_OUTPUT, S3C_GPIO_SETPIN_ZERO, S3C_GPIO_PULL_NONE,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, 0, 0}, 
        {S5PV210_GPH0(5), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, 0, 0}, 
        {S5PV210_GPH0(6), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, 0, 0}, 
        {S5PV210_GPH0(7), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, 0, 0}, 

        {S5PV210_GPH1(0), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, 0, 0}, 
        {S5PV210_GPH1(1), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, 0, 0}, 
        {S5PV210_GPH1(2), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, 0, 0}, 
        {S5PV210_GPH1(3), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, 0, 0}, 
        {S5PV210_GPH1(4), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, 0, 0}, 
        {S5PV210_GPH1(5), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, 0, 0}, 
        {S5PV210_GPH1(6), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_NONE,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, 0, 0}, 
        {S5PV210_GPH1(7), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, 0, 0}, 

        {S5PV210_GPH2(0), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, 0, 0}, 
        {S5PV210_GPH2(1), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, 0, 0}, 
        {S5PV210_GPH2(2), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, 0, 0}, 
        {S5PV210_GPH2(3), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, 0, 0}, 
        {S5PV210_GPH2(4), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, 0, 0}, 
        {S5PV210_GPH2(5), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, 0, 0}, 
        {S5PV210_GPH2(6), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_NONE,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, 0, 0}, 
        {S5PV210_GPH2(7), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, 0, 0}, 

        {S5PV210_GPH3(0), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_NONE,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, 0, 0}, 
        {S5PV210_GPH3(1), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_NONE,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, 0, 0}, 
        {S5PV210_GPH3(2), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, 0, 0}, 
        {S5PV210_GPH3(3), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, 0, 0}, 
        {S5PV210_GPH3(4), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, 0, 0}, 
        {S5PV210_GPH3(5), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_NONE,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, 0, 0}, 
        {S5PV210_GPH3(6), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, 0, 0}, 
        {S5PV210_GPH3(7), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, 0, 0}, 


	/* Alive part ending and off part start*/
	{S5PV210_GPI(0), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_GPI(1), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_GPI(2), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_GPI(3), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_GPI(4), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_GPI(5), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_GPI(6), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
#if defined(CONFIG_TARGET_LOCALE_LTN)  // CYS_ 2010.05.07 WLAN_BT_EN, GPJ0(0)->GPB(5) (in rev01) GPJ1(0) (in rev02)
        {S5PV210_GPJ0(0), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
#else
	{S5PV210_GPJ0(0), S3C_GPIO_OUTPUT, S3C_GPIO_SETPIN_ZERO, S3C_GPIO_PULL_NONE,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_PREV, S3C_GPIO_PULL_NONE}, 
#endif
        {S5PV210_GPJ0(1), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_GPJ0(2), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_GPJ0(3), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_GPJ0(4), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_GPJ0(5), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_GPJ0(6), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_GPJ0(7), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 

#if defined(CONFIG_TARGET_LOCALE_LTN)// CYS_ 2010.05.07 WLAN_BT_EN, GPJ0(0)->GPJ1(0)
        {S5PV210_GPJ1(0), S3C_GPIO_OUTPUT, S3C_GPIO_SETPIN_ZERO, S3C_GPIO_PULL_NONE,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_PREV, S3C_GPIO_PULL_NONE}, 
#else
	{S5PV210_GPJ1(0), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
#endif
//        {S5PV210_GPJ1(1), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
//			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_GPJ1(2), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_GPJ1(4), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_GPJ1(5), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 

//	{S5PV210_GPJ2(0), S3C_GPIO_OUTPUT, S3C_GPIO_SETPIN_ONE, S3C_GPIO_PULL_NONE,
//			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
//        {S5PV210_GPJ2(1), S3C_GPIO_OUTPUT, S3C_GPIO_SETPIN_ONE, S3C_GPIO_PULL_NONE,
//			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
        {S5PV210_GPJ2(2), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_GPJ2(3), S3C_GPIO_OUTPUT, S3C_GPIO_SETPIN_ONE, S3C_GPIO_PULL_NONE,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
//        {S5PV210_GPJ2(4), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
//			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_GPJ2(5), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_GPJ2(7), S3C_GPIO_OUTPUT, S3C_GPIO_SETPIN_ONE, S3C_GPIO_PULL_NONE,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPJ3(0), S3C_GPIO_OUTPUT, S3C_GPIO_SETPIN_ONE, S3C_GPIO_PULL_NONE,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
        {S5PV210_GPJ3(1), S3C_GPIO_OUTPUT, S3C_GPIO_SETPIN_ONE, S3C_GPIO_PULL_NONE,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
        {S5PV210_GPJ3(2), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_GPJ3(3), S3C_GPIO_OUTPUT, S3C_GPIO_SETPIN_ONE, S3C_GPIO_PULL_NONE,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
        {S5PV210_GPJ3(4), S3C_GPIO_OUTPUT, S3C_GPIO_SETPIN_ONE, S3C_GPIO_PULL_NONE,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
        {S5PV210_GPJ3(5), S3C_GPIO_OUTPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_NONE,  // TA_EN
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_PREV, S3C_GPIO_PULL_NONE}, 
        {S5PV210_GPJ3(6), S3C_GPIO_OUTPUT, S3C_GPIO_SETPIN_ONE, S3C_GPIO_PULL_NONE,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
        {S5PV210_GPJ3(7), S3C_GPIO_OUTPUT, S3C_GPIO_SETPIN_ONE, S3C_GPIO_PULL_NONE,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPJ4(0), S3C_GPIO_OUTPUT, S3C_GPIO_SETPIN_ONE, S3C_GPIO_PULL_NONE,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
        {S5PV210_GPJ4(1), S3C_GPIO_OUTPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_NONE,  // CURR_ADJ
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_PREV, S3C_GPIO_PULL_NONE}, 
        {S5PV210_GPJ4(2), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_GPJ4(3), S3C_GPIO_OUTPUT, S3C_GPIO_SETPIN_ONE, S3C_GPIO_PULL_NONE,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
        {S5PV210_GPJ4(4), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
	/* memory part */
#if !defined(CONFIG_TARGET_LOCALE_LTN) // CYS_ 2010.06.24 move to cmc623_setting to bootloader 
	{S5PV210_MP01(0), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_MP01(1), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
#endif
        {S5PV210_MP01(2), S3C_GPIO_OUTPUT, S3C_GPIO_SETPIN_ZERO, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_4X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
#if !defined(CONFIG_TARGET_LOCALE_LTN) // CYS_ 2010.06.24 move to cmc623_setting to bootloader 
        {S5PV210_MP01(3), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
#endif
        {S5PV210_MP01(4), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_MP01(5), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_MP01(6), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_MP01(7), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
 
	{S5PV210_MP02(0), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_MP02(1), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_MP02(2), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_MP02(3), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 

	{S5PV210_MP03(0), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_MP03(1), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_MP03(2), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_MP03(3), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_MP03(4), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_MP03(5), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_MP03(6), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_MP03(7), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 

	{S5PV210_MP04(0), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_MP04(1), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_MP04(2), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_MP04(3), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
#if !defined(CONFIG_TARGET_LOCALE_LTN) // CYS_ 2010.06.24 move to cmc623_setting to bootloader 
        {S5PV210_MP04(4), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
#endif
        {S5PV210_MP04(5), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_MP04(6), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_MP04(7), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 

	{S5PV210_MP05(0), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_MP05(1), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_MP05(2), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_MP05(3), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_MP05(4), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
#if !defined(CONFIG_TARGET_LOCALE_LTN) // CYS_ 2010.06.24 move to cmc623_setting to bootloader 
        {S5PV210_MP05(6), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
#endif
	{S5PV210_MP05(7), S3C_GPIO_OUTPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_NONE,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_PREV, S3C_GPIO_PULL_NONE}, 

	{S5PV210_MP06(0), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_MP06(1), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_MP06(2), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_MP06(3), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_MP06(4), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_MP06(5), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_MP06(6), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_MP06(7), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 

	{S5PV210_MP07(0), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_MP07(1), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_MP07(2), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_MP07(3), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_MP07(4), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_MP07(5), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_MP07(6), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
        {S5PV210_MP07(7), S3C_GPIO_INPUT, S3C_GPIO_SETPIN_NONE, S3C_GPIO_PULL_DOWN,
			 S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 

	/* Memory part ending and off part ending */

};

static unsigned int p1_motor_i2c_gpio_table[][8] = {
	{S5PV210_GPJ2(0), S3C_GPIO_OUTPUT, S3C_GPIO_SETPIN_ONE, S3C_GPIO_PULL_NONE,
			S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ2(1), S3C_GPIO_OUTPUT, S3C_GPIO_SETPIN_ONE, S3C_GPIO_PULL_NONE,
			S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
};

static unsigned int p1_touch_i2c_gpio_table[][8] = {
	{S5PV210_GPD0(0), S3C_GPIO_OUTPUT, S3C_GPIO_SETPIN_ONE, S3C_GPIO_PULL_NONE,
			S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE},
#if !defined(CONFIG_TARGET_LOCALE_LTN)  // CYS_ 2010.05.07		
	{S5PV210_GPG3(2), S3C_GPIO_OUTPUT, S3C_GPIO_SETPIN_ONE, S3C_GPIO_PULL_NONE,
			S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE},
#endif	
};

static unsigned int p1_lcd_amoled_gpio_table[][8] = {
        {S5PV210_GPJ1(3), S3C_GPIO_OUTPUT, S3C_GPIO_SETPIN_ONE, S3C_GPIO_PULL_NONE,
		S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_OUT1, S3C_GPIO_PULL_NONE},
	{S5PV210_GPJ2(6), S3C_GPIO_OUTPUT, S3C_GPIO_SETPIN_ONE, S3C_GPIO_PULL_NONE,
		S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_OUT1, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP05(5), S3C_GPIO_OUTPUT, S3C_GPIO_SETPIN_ONE, S3C_GPIO_PULL_NONE,
		S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_OUT1, S3C_GPIO_PULL_NONE}, 
};

static unsigned int p1_lcd_tft_gpio_table[][8] = {
#if !defined(CONFIG_TARGET_LOCALE_LTN) // CYS_ 2010.07.02 for Camera Preview
        {GPIO_MLCD_ON, S3C_GPIO_OUTPUT, S3C_GPIO_SETPIN_ONE, S3C_GPIO_PULL_DOWN,
		S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE},
#endif
	{GPIO_LCD_LDO_EN, S3C_GPIO_OUTPUT, S3C_GPIO_SETPIN_ONE, S3C_GPIO_PULL_DOWN,
		S3C_GPIO_DRVSTR_1X, S3C_GPIO_SLEWRATE_FAST, S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
};

void s3c_config_gpio_table(int array_size, unsigned int (*gpio_table)[8])
{
	u32 i, gpio;
	for (i = 0; i < array_size; i++) {
		gpio = gpio_table[i][0];
		/* Off part */
		if((gpio <= S5PV210_GPG3(6)) ||
			((gpio <= S5PV210_GPJ4(7)) && (gpio >= S5PV210_GPI(0)))) {

			s3c_gpio_cfgpin(gpio, S3C_GPIO_SFN(gpio_table[i][1]));
			s3c_gpio_setpull(gpio, gpio_table[i][3]);

			if (gpio_table[i][2] != S3C_GPIO_SETPIN_NONE)
				gpio_set_value(gpio, gpio_table[i][2]);

#if 0	// currently not supported, should be enabled later
			s3c_gpio_set_drvstrength(gpio, gpio_table[i][4]);
			s3c_gpio_set_slewrate(gpio, gpio_table[i][5]);
#endif
		}
	}
}

void s3c_config_gpio_alive_table(int array_size, int (*gpio_table)[4])
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


#define S5PV210_PS_HOLD_CONTROL_REG (S3C_VA_SYS+0xE81C)
//extern void arch_reset(char mode);
extern u8 FSA9480_Get_JIG_Status(void);

static void smdkc110_power_off (void) 
{
#if 1
	int	mode = REBOOT_MODE_NONE;
	char reset_mode = 'r';
	char* str_ptr = "reset\n";
	int cnt = 0;

	printk("MIDAS@%s: +\n", __func__);
//	kernel_sec_clear_upload_magic_number();

	if (maxim_chg_status()) {	/* Reboot Charging */
		mode = REBOOT_MODE_CHARGING;
		if (sec_set_param_value)
			sec_set_param_value(__REBOOT_MODE, &mode);

		if(HWREV <= 0x3)
		{
			gpio_set_value(GPIO_TA_EN, 1);  // diasble externel charger IC
			maxim_charging_control(0x3, TRUE);  // enable MAX8998 charger
		}
		
		/* Watchdog Reset */
		printk(KERN_EMERG "%s: TA is connected, rebooting...\n", __func__);
#ifdef CONFIG_KERNEL_DEBUG_SEC
		kernel_sec_hw_reset(true);
#else
		arch_reset(reset_mode, str_ptr);
#endif
		printk(KERN_EMERG "%s: waiting for reset!\n", __func__);
	}
	else /* Power Off or Reboot */
	{	
//		if (sec_set_param_value)
//			sec_set_param_value(__REBOOT_MODE, &mode);

#if 1 //if JIG is connected, reset
		if (FSA9480_Get_JIG_Status()) {
			/* Watchdog Reset */
			printk(KERN_EMERG "%s: JIG is connected, rebooting...\n", __func__);
#ifdef CONFIG_KERNEL_DEBUG_SEC
			kernel_sec_hw_reset(true);
#else
			arch_reset(reset_mode, str_ptr);
#endif
			printk(KERN_EMERG "%s: waiting for reset!\n", __func__);
		} else
#endif			
		{
			/* POWER_N -> Input */
			s3c_gpio_cfgpin(GPIO_nPOWER, S3C_GPIO_INPUT);
			s3c_gpio_setpull(GPIO_nPOWER, S3C_GPIO_PULL_NONE);
			/* PHONE_ACTIVE -> Input */
			s3c_gpio_cfgpin(GPIO_PHONE_ACTIVE, S3C_GPIO_INPUT);
			s3c_gpio_setpull(GPIO_PHONE_ACTIVE, S3C_GPIO_PULL_NONE);

			/* Check Power Off Condition */
			if (!gpio_get_value(GPIO_nPOWER) || gpio_get_value(GPIO_PHONE_ACTIVE)) {
				/* Wait Power Button Release */
				printk(KERN_EMERG "%s: waiting for GPIO_nPOWER high.\n", __func__);
				
#if 0 // add later for checking of nPower and Phone_active 
				while (!gpio_get_value(GPIO_N_POWER)); 

				/* Wait Phone Power Off */
				printk(KERN_EMERG "%s: waiting for GPIO_PHONE_ACTIVE low.\n", __func__);
				while (gpio_get_value(GPIO_PHONE_ACTIVE)) {
					if (cnt++ < 5) {
						printk(KERN_EMERG "%s: GPIO_PHONE_ACTIVE is high(%d)\n", __func__, cnt);
						mdelay(1000);
					} else {
						printk(KERN_EMERG "%s: GPIO_PHONE_ACTIVE TIMED OUT!!!\n", __func__);
						break;
					}
				}	
#endif
			}
			/* PS_HOLD -> Output Low */
			printk(KERN_EMERG "%s: setting GPIO_PDA_PS_HOLD low.\n", __func__);

			/*PS_HOLD high  PS_HOLD_CONTROL, R/W, 0xE010_E81C*/
			writel(readl(S5PV210_PS_HOLD_CONTROL_REG) & 0xFFFFFEFF, S5PV210_PS_HOLD_CONTROL_REG);
			//gpio_direction_output(GPIO_AP_PS_HOLD, 1);
			//s3c_gpio_setpull(GPIO_AP_PS_HOLD, S3C_GPIO_PULL_NONE);
			//gpio_set_value(GPIO_AP_PS_HOLD, 0);

			printk(KERN_EMERG "%s: should not reach here!\n", __func__);
		}
	}

	printk("MIDAS@%s: -\n", __func__);
	while (1);
#else
	char reset_mode = 'r';
	char* str_ptr = "reset\n";
	arch_reset(reset_mode, str_ptr);
	while (1);
#endif
}


static unsigned int p1_sleep_gpio_table[][3] = {

	{S5PV210_GPA0(0),
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPA0(1),
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPA0(2),
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPA0(3),
			S3C_GPIO_SLP_OUT1, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPA0(4), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPA0(5),
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPA0(6), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPA0(7),
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPA1(0), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPA1(1),
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPA1(2), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPA1(3),
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPB(0), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPB(1),
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPB(2),
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
//	{S5PV210_GPB(3),  // GPIO_BT_nRST
//			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPB(4), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
#if !defined(CONFIG_TARGET_LOCALE_LTN)  // CYS_ 2010.05.07
	{S5PV210_GPB(5), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
#endif
	{S5PV210_GPB(6), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPB(7),
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPC0(0), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPC0(1), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPC0(2), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPC0(3), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPC0(4), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPC1(0), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPC1(1), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPC1(2), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPC1(3), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPC1(4), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPD0(0), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPD0(1), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPD0(2), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPD0(3), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPD1(0), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPD1(1), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPD1(2), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPD1(3), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPD1(4), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPD1(5), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPE0(0), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPE0(1), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPE0(2), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPE0(3), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPE0(4), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPE0(5), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPE0(6), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPE0(7),
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPE1(0), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPE1(1), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPE1(2), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPE1(3), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPE1(4), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPF0(0), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF0(1), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF0(2), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF0(3), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF0(4), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF0(5), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF0(6), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF0(7), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPF1(0), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF1(1), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF1(2), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF1(3), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF1(4), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF1(5), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF1(6), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF1(7), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPF2(0), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF2(1), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF2(2), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF2(3), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF2(4), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF2(5), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF2(6), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF2(7), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPF3(0), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF3(1), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF3(2), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF3(3), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
// This pin is controlled by sound. - GPIO_CODEC_LDO_EN
	{S5PV210_GPF3(4), 
			S3C_GPIO_SLP_PREV, S3C_GPIO_PULL_NONE},  // audio
	{S5PV210_GPF3(5), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPG0(0), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG0(1), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG0(2), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG0(3), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG0(4), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG0(5), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG0(6), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPG1(0), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG1(1), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
//	{S5PV210_GPG1(2),  // GPIO_WLAN_nRST
//			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE},
	{S5PV210_GPG1(3), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG1(4), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG1(5), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG1(6), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPG2(0), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG2(1), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG2(2), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG2(3), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG2(4), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG2(5), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG2(6), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPG3(0), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG3(1), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG3(2), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG3(3), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG3(4), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG3(5), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG3(6), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE},

	{S5PV210_GPI(0), 
			S3C_GPIO_SLP_PREV, S3C_GPIO_PULL_NONE},  // audio
	{S5PV210_GPI(1), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPI(2), 
			S3C_GPIO_SLP_PREV, S3C_GPIO_PULL_NONE},  // audio
	{S5PV210_GPI(3), 
			S3C_GPIO_SLP_PREV, S3C_GPIO_PULL_NONE},  // audio
	{S5PV210_GPI(4), 
			S3C_GPIO_SLP_PREV, S3C_GPIO_PULL_NONE},  // audio
	{S5PV210_GPI(5), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPI(6), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
#if defined(CONFIG_TARGET_LOCALE_LTN)  // CYS_ 2010.05.07
	{S5PV210_GPJ0(0),  // ISDBT_SCL
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
#else
//	{S5PV210_GPJ0(0),  // GPIO_WLAN_BT_EN
//			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
#endif
	{S5PV210_GPJ0(1), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ0(2), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ0(3), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ0(4), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ0(5), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ0(6), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ0(7), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
#if defined(CONFIG_TARGET_LOCALE_LTN)
//	{S5PV210_GPJ1(0),  // GPIO_WLAN_BT_EN
//			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
#else
	{S5PV210_GPJ1(0), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
#endif
	{S5PV210_GPJ1(1), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ1(2), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ1(4), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ1(5), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPJ2(0), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ2(1), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ2(2), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ2(3), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ2(4), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ2(5), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ2(7), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPJ3(0), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ3(1), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ3(2), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ3(3), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ3(4), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ3(5),  // TA_EN
			S3C_GPIO_SLP_PREV, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ3(6), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ3(7), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPJ4(0), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ4(1), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
// This pin is controlled by sound. - GPIO_MICBIAS_EN
	{S5PV210_GPJ4(2), 
			S3C_GPIO_SLP_PREV, S3C_GPIO_PULL_NONE},  // audio
	{S5PV210_GPJ4(3), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ4(4), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 

	/* memory part */
	{S5PV210_MP01(0), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP01(1), 
			S3C_GPIO_SLP_OUT1, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP01(2), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP01(3), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP01(4), 
			S3C_GPIO_SLP_OUT1, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP01(5), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE},
	{S5PV210_MP01(6), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP01(7), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 

	{S5PV210_MP02(0), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP02(1), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP02(2), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP02(3), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 

	{S5PV210_MP03(0), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_MP03(1), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_MP03(2), 
			S3C_GPIO_SLP_OUT1, S3C_GPIO_PULL_NONE},
	{S5PV210_MP03(3), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP03(4), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP03(5), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP03(6), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
 	{S5PV210_MP03(6), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP03(7), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 

	{S5PV210_MP04(0), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP04(1), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP04(2), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP04(3), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP04(4), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP04(5), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP04(6), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP04(7), // MHL_RST
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 

	{S5PV210_MP05(0), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP05(1), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP05(2), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP05(3), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP05(4), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP05(6), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP05(7), 
			S3C_GPIO_SLP_PREV, S3C_GPIO_PULL_NONE}, //UART_SEL prev

	{S5PV210_MP06(0), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP06(1), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP06(2), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP06(3), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP06(4), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP06(5), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP06(6), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP06(7), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 

	{S5PV210_MP07(0), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP07(1), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP07(2), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP07(3), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP07(4), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP07(5), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP07(6), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP07(7), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	/* Memory part ending and off part ending */
};

static unsigned int p1_r05_sleep_gpio_table[][3] = {

	{S5PV210_GPA0(0),
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPA0(1),
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPA0(2),
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPA0(3),
			S3C_GPIO_SLP_OUT1, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPA0(4), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPA0(5),
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPA0(6), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPA0(7),
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPA1(0), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPA1(1),
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPA1(2), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPA1(3),
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPB(0), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPB(1),
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPB(2),
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
//	{S5PV210_GPB(3),  // GPIO_BT_nRST
//			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPB(4), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
#if !defined(CONFIG_TARGET_LOCALE_LTN) // CYS_ 2010.05.07
	{S5PV210_GPB(5), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
#endif
	{S5PV210_GPB(6), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPB(7),
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPC0(0), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPC0(1), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPC0(2), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPC0(3), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPC0(4), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPC1(0), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPC1(1), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPC1(2), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPC1(3), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPC1(4), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPD0(0),  // KEY_LED_EN
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPD0(1), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPD0(2), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPD0(3),  // LCD_CABC_PWM
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPD1(0), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPD1(1), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPD1(2), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPD1(3), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPD1(4), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPD1(5), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPE0(0), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPE0(1), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPE0(2), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPE0(3), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPE0(4), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPE0(5), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPE0(6), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPE0(7),
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPE1(0), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPE1(1), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPE1(2), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPE1(3), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPE1(4), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPF0(0), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF0(1), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF0(2), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF0(3), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF0(4), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF0(5), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF0(6), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF0(7), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPF1(0), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF1(1), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF1(2), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF1(3), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF1(4), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF1(5), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF1(6), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF1(7), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPF2(0), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF2(1), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF2(2), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF2(3), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF2(4), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF2(5), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF2(6), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF2(7), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPF3(0), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF3(1), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF3(2), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF3(3), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
// This pin is controlled by sound. - GPIO_CODEC_LDO_EN
	{S5PV210_GPF3(4), 
			S3C_GPIO_SLP_PREV, S3C_GPIO_PULL_NONE},  // audio
	{S5PV210_GPF3(5), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPG0(0), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG0(1), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG0(2), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG0(3), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG0(4), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG0(5), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG0(6), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPG1(0), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG1(1), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
//	{S5PV210_GPG1(2),  // GPIO_WLAN_nRST
//			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE},
	{S5PV210_GPG1(3), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG1(4), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG1(5), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG1(6), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPG2(0), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG2(1), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG2(2), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG2(3), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG2(4), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG2(5), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG2(6), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPG3(0), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG3(1), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG3(2),  // NC
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG3(3), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG3(4), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG3(5), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG3(6), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE},

	{S5PV210_GPI(0), 
			S3C_GPIO_SLP_PREV, S3C_GPIO_PULL_NONE},  // audio
	{S5PV210_GPI(1), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPI(2), 
			S3C_GPIO_SLP_PREV, S3C_GPIO_PULL_NONE},  // audio
	{S5PV210_GPI(3), 
			S3C_GPIO_SLP_PREV, S3C_GPIO_PULL_NONE},  // audio
	{S5PV210_GPI(4), 
			S3C_GPIO_SLP_PREV, S3C_GPIO_PULL_NONE},  // audio
	{S5PV210_GPI(5), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPI(6), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
#if defined(CONFIG_TARGET_LOCALE_LTN)  // CYS_ 2010.05.07
	{S5PV210_GPJ0(0),  // ISDBT_SCL
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
#else
//	{S5PV210_GPJ0(0),  // GPIO_WLAN_BT_EN
//			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
#endif
	{S5PV210_GPJ0(1), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ0(2), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ0(3), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ0(4), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ0(5),  // NC
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ0(6), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ0(7), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 

#if defined(CONFIG_TARGET_LOCALE_LTN)
//	{S5PV210_GPJ1(0),  // GPIO_WLAN_BT_EN
//			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
#else
	{S5PV210_GPJ1(0), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
#endif
	{S5PV210_GPJ1(1),  // VIBTONE_EN
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ1(2), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ1(4), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ1(5), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPJ2(0), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ2(1),  // GYRO_CS (NC)
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ2(2), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ2(3), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ2(4),  // NC
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ2(5),  // NC
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ2(7), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPJ3(0), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ3(1), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ3(2), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ3(3), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ3(4), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ3(5),  // TA_EN
			S3C_GPIO_SLP_PREV, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ3(6), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ3(7), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPJ4(0), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ4(1),  // CURR_ADJ
			S3C_GPIO_SLP_PREV, S3C_GPIO_PULL_NONE}, 
// This pin is controlled by sound. - GPIO_MICBIAS_EN
	{S5PV210_GPJ4(2), 
			S3C_GPIO_SLP_PREV, S3C_GPIO_PULL_NONE},  // audio
	{S5PV210_GPJ4(3), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ4(4), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 

	/* memory part */
	{S5PV210_MP01(0), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP01(1), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP01(2), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP01(3), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP01(4), 
			S3C_GPIO_SLP_OUT1, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP01(5), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE},
	{S5PV210_MP01(6), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP01(7), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 

	{S5PV210_MP02(0), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP02(1), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP02(2), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP02(3), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 

	{S5PV210_MP03(0), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_MP03(1), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_MP03(2), 
			S3C_GPIO_SLP_OUT1, S3C_GPIO_PULL_NONE},
	{S5PV210_MP03(3), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP03(4), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP03(5), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP03(6), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
 	{S5PV210_MP03(6), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP03(7), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 

	{S5PV210_MP04(0), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP04(1), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP04(2), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP04(3), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP04(4), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP04(5), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP04(6), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP04(7), // MHL_RST
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE},

	{S5PV210_MP05(0), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP05(1), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP05(2), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP05(3), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP05(4), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP05(6), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP05(7), 
			S3C_GPIO_SLP_PREV, S3C_GPIO_PULL_NONE}, //UART_SEL prev

	{S5PV210_MP06(0), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP06(1), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP06(2), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP06(3), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP06(4), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP06(5), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP06(6), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP06(7), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 

	{S5PV210_MP07(0), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP07(1), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP07(2), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP07(3), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP07(4), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP07(5), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP07(6), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP07(7), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	/* Memory part ending and off part ending */
};

static unsigned int p1_r08_sleep_gpio_table[][3] = {
	{S5PV210_GPJ1(4),  // NC
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP04(3),  // NC
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP04(5),  // NC
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
};

static unsigned int p1_r09_sleep_gpio_table[][3] = {

	{S5PV210_GPA0(0),
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPA0(1),
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPA0(2),
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPA0(3),
			S3C_GPIO_SLP_OUT1, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPA0(4), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPA0(5),
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPA0(6), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPA0(7),
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPA1(0), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPA1(1),
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPA1(2), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPA1(3),
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPB(0), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPB(1),
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPB(2),
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
//	{S5PV210_GPB(3),  // GPIO_BT_nRST
//			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPB(4), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPB(5), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPB(6), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPB(7),
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPC0(0), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPC0(1), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPC0(2), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPC0(3), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPC0(4), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPC1(0),  // CMC_SLEEP
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPC1(1),  // CMC_EN
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPC1(2),  // CMC_RST
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPC1(3),  // CMC_SHDN
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPC1(4),  // CMC_BYPASS
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPD0(0),  // KEY_LED_EN
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPD0(1), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPD0(2), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPD0(3),  // LCD_CABC_PWM
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPD1(0), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPD1(1), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPD1(2), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPD1(3), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPD1(4), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPD1(5), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPE0(0), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPE0(1), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPE0(2), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPE0(3), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPE0(4), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPE0(5), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPE0(6), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPE0(7),
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPE1(0), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPE1(1), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPE1(2), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPE1(3), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPE1(4), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPF0(0), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF0(1), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF0(2), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF0(3), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF0(4), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF0(5), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF0(6), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF0(7), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPF1(0), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF1(1), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF1(2), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF1(3), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF1(4), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF1(5), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF1(6), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF1(7), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPF2(0), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF2(1), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF2(2), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF2(3), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF2(4), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF2(5), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF2(6), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF2(7), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPF3(0), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF3(1), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF3(2), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF3(3), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
// This pin is controlled by sound. - GPIO_CODEC_LDO_EN
	{S5PV210_GPF3(4), 
			S3C_GPIO_SLP_PREV, S3C_GPIO_PULL_NONE},  // audio
	{S5PV210_GPF3(5), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPG0(0), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG0(1), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG0(2), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG0(3), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG0(4), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG0(5), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG0(6), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPG1(0), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG1(1), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
//	{S5PV210_GPG1(2),  // GPIO_WLAN_nRST
//			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE},
	{S5PV210_GPG1(3), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG1(4), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG1(5), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG1(6), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPG2(0), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG2(1), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG2(2), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG2(3), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG2(4), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG2(5), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG2(6), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPG3(0), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG3(1), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG3(2),  // NC
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG3(3),  // NC
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG3(4), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG3(5), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG3(6), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE},

	{S5PV210_GPI(0), 
			S3C_GPIO_SLP_PREV, S3C_GPIO_PULL_NONE},  // audio
	{S5PV210_GPI(1), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPI(2), 
			S3C_GPIO_SLP_PREV, S3C_GPIO_PULL_NONE},  // audio
	{S5PV210_GPI(3), 
			S3C_GPIO_SLP_PREV, S3C_GPIO_PULL_NONE},  // audio
	{S5PV210_GPI(4), 
			S3C_GPIO_SLP_PREV, S3C_GPIO_PULL_NONE},  // audio
	{S5PV210_GPI(5), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPI(6), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 

//	{S5PV210_GPJ0(0),  // GPIO_WLAN_BT_EN
//			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ0(1), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ0(2), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ0(3), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ0(4), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ0(5),  // TOUCH_INT
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ0(6), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ0(7), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPJ1(0), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ1(1),  // MESSMEMORY_EN
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ1(2), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ1(4),  // OVF_FLAG
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ1(5), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPJ2(0),  // CHARGER_SDA_2.8V
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ2(1),  // CHARGER_SCL_2.8V
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ2(2),  // HDMI_EN1
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ2(3), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ2(4),  // MESSMEMORY_EN2
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ2(5),  // NC
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ2(7), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPJ3(0), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ3(1), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ3(2), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ3(3), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ3(4), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ3(5),  // TA_EN
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ3(6), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ3(7), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPJ4(0), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ4(1),  // NC
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
// This pin is controlled by sound. - GPIO_MICBIAS_EN
	{S5PV210_GPJ4(2), 
			S3C_GPIO_SLP_PREV, S3C_GPIO_PULL_NONE},  // audio
	{S5PV210_GPJ4(3), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ4(4), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 

	/* memory part */
	{S5PV210_MP01(0), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP01(1), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP01(2), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP01(3), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP01(4), 
			S3C_GPIO_SLP_OUT1, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP01(5), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE},
	{S5PV210_MP01(6), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP01(7), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 

	{S5PV210_MP02(0), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP02(1), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP02(2), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP02(3), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 

	{S5PV210_MP03(0), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_MP03(1), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_MP03(2), 
			S3C_GPIO_SLP_OUT1, S3C_GPIO_PULL_NONE},
	{S5PV210_MP03(3), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP03(4), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP03(5), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP03(6), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
 	{S5PV210_MP03(6), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP03(7), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 

	{S5PV210_MP04(0), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP04(1), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP04(2), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP04(3),  // CMC_SCL_1.8V
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP04(4), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP04(5),  // CMC_SDA_1.8V
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP04(6),  // GPS_CNBTL
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP04(7), // MHL_RST
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE},

	{S5PV210_MP05(0), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP05(1), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP05(2), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP05(3), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP05(4), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP05(6), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP05(7), 
			S3C_GPIO_SLP_PREV, S3C_GPIO_PULL_NONE}, //UART_SEL prev

	{S5PV210_MP06(0), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP06(1), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP06(2), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP06(3), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP06(4), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP06(5), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP06(6), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP06(7), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 

	{S5PV210_MP07(0), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP07(1), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP07(2), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP07(3), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP07(4), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP07(5), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP07(6), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP07(7), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	/* Memory part ending and off part ending */
};

static unsigned int p1_r11_sleep_gpio_table[][3] = {
	{S5PV210_GPJ1(4),  // NC
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
};

static unsigned int p1_r12_sleep_gpio_table[][3] = {

	{S5PV210_GPA0(0),
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPA0(1),
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPA0(2),
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPA0(3),
			S3C_GPIO_SLP_OUT1, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPA0(4), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPA0(5),
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPA0(6), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPA0(7),
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPA1(0), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_GPA1(1),
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPA1(2), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPA1(3),
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPB(0), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPB(1),
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPB(2),
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
//	{S5PV210_GPB(3),  // GPIO_BT_nRST
//			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
#if defined(CONFIG_TARGET_LOCALE_LTN)
	{S5PV210_GPB(4),  // HWREV_MODE3
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPB(5),  // HWREV_MODE2
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPB(6),  // HWREV_MODE1
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPB(7),  // HWREV_MODE0
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
#else
	{S5PV210_GPB(4),  // NC
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_GPB(5),  // NC
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_GPB(6),  // NC
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_GPB(7),  // NC
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
#endif
	{S5PV210_GPC0(0), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPC0(1),  // NC
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_GPC0(2), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPC0(3), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPC0(4), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 

#if defined(CONFIG_TARGET_LOCALE_LTN)
	{S5PV210_GPC1(0),  // I2S_SCLK_1.8V
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_GPC1(1),  // I2S_MCLK_1.8V
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_GPC1(2),  // I2S_LRCLK_1.8V
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_GPC1(3),  // I2S_DATA_1.8V
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_GPC1(4),  // NC
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
#else
	{S5PV210_GPC1(0),  // CMC_SLEEP
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPC1(1),  // CMC_EN
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPC1(2),  // CMC_RST
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPC1(3),  // CMC_SHDN
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPC1(4),  // CMC_BYPASS
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
#endif
	{S5PV210_GPD0(0),  // KEY_LED_EN
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPD0(1),  // NC
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 

#if defined(CONFIG_TARGET_LOCALE_LTN)
	{S5PV210_GPD0(2),   // HWREV_MODE4
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
#else
	{S5PV210_GPD0(2),  // NC
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
#endif
	{S5PV210_GPD0(3),  // LCD_CABC_PWM
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPD1(0), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_GPD1(1), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_GPD1(2), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPD1(3), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPD1(4), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_GPD1(5), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 

	{S5PV210_GPE0(0), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPE0(1), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPE0(2), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPE0(3), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPE0(4), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPE0(5), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPE0(6), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPE0(7),
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPE1(0), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPE1(1), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPE1(2), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPE1(3), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPE1(4),  // NC
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 

	{S5PV210_GPF0(0), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF0(1), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF0(2), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF0(3), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF0(4), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF0(5), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF0(6), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF0(7), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPF1(0), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF1(1), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF1(2), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF1(3), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF1(4), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF1(5), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF1(6), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF1(7), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPF2(0), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF2(1), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF2(2), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF2(3), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF2(4), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF2(5), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF2(6), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF2(7), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPF3(0), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF3(1), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF3(2), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPF3(3), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
// This pin is controlled by sound. - GPIO_CODEC_LDO_EN
	{S5PV210_GPF3(4), 
			S3C_GPIO_SLP_PREV, S3C_GPIO_PULL_NONE},  // audio
	{S5PV210_GPF3(5), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPG0(0), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG0(1),  // NAND_CMD
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
#if defined(CONFIG_TARGET_LOCALE_LTN)
	{S5PV210_GPG0(2),   // TOUCH_INT
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
#else
	{S5PV210_GPG0(2),  // NC
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN},
#endif
	{S5PV210_GPG0(3),  // NAND_D(0)
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_GPG0(4),  // NAND_D(1)
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_GPG0(5),  // NAND_D(2)
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_GPG0(6),  // NAND_D(3)
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 

	{S5PV210_GPG1(0),  // GPS_nRST
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG1(1),  // GPS_PWR_EN
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
#if defined(CONFIG_TARGET_LOCALE_LTN)
	{S5PV210_GPG1(2), // ISDBT_RSTn 
			  S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_DOWN}, 
#else
	{S5PV210_GPG1(2),  // NC
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN},
#endif
	{S5PV210_GPG1(3),  // NAND_D(4)
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_GPG1(4),  // NAND_D(5)
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_GPG1(5),  // NAND_D(6)
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_GPG1(6),  // NAND_D(7)
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 

	{S5PV210_GPG2(0), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG2(1),  // T_FLASH_CLK
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_GPG2(2), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG2(3),  // T_FLASH_D(0)
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_GPG2(4),  // T_FLASH_D(1)
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_GPG2(5),  // T_FLASH_D(2)
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_GPG2(6),  // T_FLASH_D(3)
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 

	{S5PV210_GPG3(0),  // WLAN_SDIO_CLK
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG3(1),  // WLAN_SDIO_CMD
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
//	{S5PV210_GPG3(2),  // WLAN_nRST
//			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG3(3),  // WLAN_SDIO_D(0)
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG3(4),  // WLAN_SDIO_D(1)
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG3(5),  // WLAN_SDIO_D(2)
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPG3(6),  // WLAN_SDIO_D(3)
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE},

	{S5PV210_GPI(0), 
			S3C_GPIO_SLP_PREV, S3C_GPIO_PULL_NONE},  // audio
	{S5PV210_GPI(1),  // NC
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_GPI(2), 
			S3C_GPIO_SLP_PREV, S3C_GPIO_PULL_NONE},  // audio
	{S5PV210_GPI(3), 
			S3C_GPIO_SLP_PREV, S3C_GPIO_PULL_NONE},  // audio
	{S5PV210_GPI(4), 
			S3C_GPIO_SLP_PREV, S3C_GPIO_PULL_NONE},  // audio
	{S5PV210_GPI(5),  // NC
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_GPI(6),  // NC
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 

#if defined(CONFIG_TARGET_LOCALE_LTN)
	{S5PV210_GPJ0(0),  // ISDBT_SCL
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ0(1),  // ISDBT_SDA
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ0(2), // ISDBT_CLK
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_GPJ0(3), // ISDBT_SYNC
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_GPJ0(4), // ISDBT_VALID
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_DOWN},  
	{S5PV210_GPJ0(5),  // ISDBT_DATA
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_DOWN},  
	{S5PV210_GPJ0(6), // ISDBT_ERR
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_DOWN}, 
#else
//	{S5PV210_GPJ0(0),  // GPIO_WLAN_BT_EN
//			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ0(1), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ0(2), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ0(3), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ0(4), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ0(5),  // TOUCH_INT
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN},
	{S5PV210_GPJ0(6), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
#endif
	{S5PV210_GPJ0(7), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
#if defined(CONFIG_TARGET_LOCALE_LTN)
//	{S5PV210_GPJ1(0),   // GPIO_WLAN_BT_EN
//			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
#else
	{S5PV210_GPJ1(0),  // NC
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
#endif
	{S5PV210_GPJ1(1),  // MESSMEMORY_EN
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ1(2), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
#if defined(CONFIG_TARGET_LOCALE_LTN)
	{S5PV210_GPJ1(3), // ISDBT_PWR_EN
			  S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_DOWN}, 
#else
	{S5PV210_GPJ1(3),  // NC
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
#endif
	{S5PV210_GPJ1(5), 
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPJ2(0),  // CHARGER_SDA_2.8V
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ2(1),  // CHARGER_SCL_2.8V
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ2(2),  // HDMI_EN1
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ2(3), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ2(4),  // BT_WAKE
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ2(5),  // WLAN_WAKE
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ2(7), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPJ3(0), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ3(1), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
#if defined(CONFIG_TARGET_LOCALE_LTN)
	{S5PV210_GPJ3(2),  // ATV_RSTn (Latin Rev0.3)
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
#else
	{S5PV210_GPJ3(2),  // NC(Rev0.6), CAM_LDO_EN(Rev0.7)
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
#endif
	{S5PV210_GPJ3(3), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ3(4), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ3(5),  // TA_EN
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ3(6), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ3(7), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 

	{S5PV210_GPJ4(0), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ4(1),  // MASSMEMORY_EN2
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
// This pin is controlled by sound. - GPIO_MICBIAS_EN
	{S5PV210_GPJ4(2), 
			S3C_GPIO_SLP_PREV, S3C_GPIO_PULL_NONE},  // audio
	{S5PV210_GPJ4(3), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ4(4), 
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 

	/* memory part */
#if defined(CONFIG_TARGET_LOCALE_LTN)//lhs_2010.08.04 for MP sleep 4mA 
	{S5PV210_MP01(0),  // CMC_SHDN
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN},
	{S5PV210_MP01(1),  // CMC_SLEEP
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
#else
	{S5PV210_MP01(0),  // NC
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_MP01(1),  // NC
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
#endif
	{S5PV210_MP01(2),  // RESET_REQ_N
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
#if defined(CONFIG_TARGET_LOCALE_LTN)
	{S5PV210_MP01(3),  // CMC_RST
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
#else
	{S5PV210_MP01(3),  // NC
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
#endif
	{S5PV210_MP01(4),  // AP_NANDCS
			S3C_GPIO_SLP_OUT1, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP01(5),  // NC
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN},
	{S5PV210_MP01(6),  // NC
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_MP01(7),  // NC
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 

	{S5PV210_MP02(0),  // NC
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_MP02(1),  // NC
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_MP02(2),  // VCC_1.8V_PDA
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP02(3),  // NC
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 

	{S5PV210_MP03(0),  // LVDS_SHDN
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_MP03(1),  // NC
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_MP03(2),  // NC (*)
			S3C_GPIO_SLP_OUT1, S3C_GPIO_PULL_NONE},
	{S5PV210_MP03(3),  // PDA_ACTIVE
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP03(4),  // VCC_1.8V_PDA
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP03(5),  // NC
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
 	{S5PV210_MP03(6),  // NC
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_MP03(7),  // NC
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 

	{S5PV210_MP04(0),  // GPS_CNTL
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP04(1),  // NC
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_MP04(2),  // NC
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_MP04(3),  // NC
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
#if defined(CONFIG_TARGET_LOCALE_LTN)
	{S5PV210_MP04(4),   // CMC_EN
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
#else
	{S5PV210_MP04(4),  // NC
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
#endif
	{S5PV210_MP04(5),  // CMC_SDA_1.8V
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP04(6),  // CMC_SCL_1.8V
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP04(7), // MHL_RST
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE},

	{S5PV210_MP05(0),  // LCD_ID
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP05(1),  // NC
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_MP05(2),  // AP_SCL
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP05(3),  // AP_SDA
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP05(4),  // NC
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
#if defined(CONFIG_TARGET_LOCALE_LTN) 
	{S5PV210_MP05(6),  // CMC_BYPASS
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
#else
	{S5PV210_MP05(6),  // NC
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
#endif
	{S5PV210_MP05(7),  //UART_SEL
			S3C_GPIO_SLP_PREV, S3C_GPIO_PULL_NONE},

	{S5PV210_MP06(0),  // NC
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_MP06(1),   // NC
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_MP06(2),   // NC
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_MP06(3),   // NC
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_MP06(4),   // NC
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_MP06(5),   // NC
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_MP06(6),   // NC
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_MP06(7),   // NC
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 

	{S5PV210_MP07(0),   // NC
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_MP07(1),   // NC
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_MP07(2),   // NC
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_MP07(3),   // NC
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_MP07(4),   // NC
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_MP07(5),   // NC
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_MP07(6),   // NC
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_MP07(7),   // NC
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
	/* Memory part ending and off part ending */
};

static unsigned int p1_r15_sleep_gpio_table[][3] = {
	{S5PV210_GPD0(1),  // VIBTONE_PWM
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPD0(3),  // NC
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_GPJ1(3),  // VIBTONE_EN
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_GPJ3(2),  // NC
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 

	{S5PV210_MP05(0),  // NC
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
	{S5PV210_MP05(1),  // EAR_MICBIAS_EN
			S3C_GPIO_SLP_PREV, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP05(5),  // NC
			S3C_GPIO_SLP_INPUT, S3C_GPIO_PULL_DOWN}, 
};

static unsigned int p1_r16_sleep_gpio_table[][3] = {
	{S5PV210_MP04(2),  // FLASH_EN1
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
	{S5PV210_MP04(3),  // FLASH_EN2
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE}, 
};

static unsigned int p1_r18_sleep_gpio_table[][3] = {
	{S5PV210_MP01(5),  // EAR_MICBIAS_EN
			S3C_GPIO_SLP_PREV, S3C_GPIO_PULL_NONE}, 
};

static unsigned int p1_lcd_amoled_sleep_gpio_table[][3] = {
	{S5PV210_GPJ1(3),
			S3C_GPIO_SLP_OUT1, S3C_GPIO_PULL_NONE},
	{S5PV210_GPJ2(6),
			S3C_GPIO_SLP_OUT1, S3C_GPIO_PULL_NONE},
	{S5PV210_MP05(5),
			S3C_GPIO_SLP_OUT1, S3C_GPIO_PULL_NONE},
};

static unsigned int p1_lcd_tft_sleep_gpio_table[][3] = {
	{S5PV210_GPJ1(3),
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE},
	{S5PV210_GPJ2(6),
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE},
	{S5PV210_MP05(5),
			S3C_GPIO_SLP_OUT0, S3C_GPIO_PULL_NONE},
};

#if defined(CONFIG_KEYBOARD_P1)
static unsigned int p1_keyboard_sleep_gpio_table[][3] = {
	{S5PV210_GPJ1(4),  // ACCESSORY_EN
		S3C_GPIO_SLP_PREV, S3C_GPIO_PULL_UP},
};
#endif

void s3c_config_sleep_gpio_table(int array_size, unsigned int (*gpio_table)[3])
{
        u32 i, gpio;
		
        for (i = 0; i < array_size; i++) {
		gpio = gpio_table[i][0];
               	s3c_gpio_slp_cfgpin(gpio, gpio_table[i][1]);
               	s3c_gpio_slp_setpull_updown(gpio, gpio_table[i][2]);
	}
}
// just for ref.. 	
//
void s3c_config_sleep_gpio(void)
{
	// Setting the alive mode registers
	s3c_gpio_cfgpin(GPIO_ACC_INT, S3C_GPIO_INPUT);
	s3c_gpio_setpull(GPIO_ACC_INT, S3C_GPIO_PULL_DOWN);
//	s3c_gpio_setpin(GPIO_ACC_INT, 0);

	s3c_gpio_cfgpin(GPIO_BUCK_1_EN_A, S3C_GPIO_OUTPUT);
	s3c_gpio_setpull(GPIO_BUCK_1_EN_A, S3C_GPIO_PULL_NONE);
	s3c_gpio_setpin(GPIO_BUCK_1_EN_A, 0);

	s3c_gpio_cfgpin(GPIO_BUCK_1_EN_B, S3C_GPIO_OUTPUT);
	s3c_gpio_setpull(GPIO_BUCK_1_EN_B, S3C_GPIO_PULL_NONE);
	s3c_gpio_setpin(GPIO_BUCK_1_EN_B, 0);

	s3c_gpio_cfgpin(GPIO_BUCK_2_EN, S3C_GPIO_OUTPUT);
	s3c_gpio_setpull(GPIO_BUCK_2_EN, S3C_GPIO_PULL_NONE);
	s3c_gpio_setpin(GPIO_BUCK_2_EN, 0);

	s3c_gpio_cfgpin(GPIO_ACCESSORY_INT, S3C_GPIO_INPUT);
	s3c_gpio_setpull(GPIO_ACCESSORY_INT, S3C_GPIO_PULL_NONE);
//	s3c_gpio_setpin(GPIO_ACCESSORY_INT, 0);

	if(HWREV >= 0x4) {  // NC
		if(HWREV == 14 || HWREV == 15) {  // RF_TOUCH_INT (P1000 Rev0.8, Rev0.9)
			s3c_gpio_cfgpin(GPIO_GPH06, S3C_GPIO_INPUT);
			s3c_gpio_setpull(GPIO_GPH06, S3C_GPIO_PULL_NONE);
			//s3c_gpio_setpin(GPIO_GPH06, 0);
		}
		else {  // NC
			s3c_gpio_cfgpin(GPIO_GPH06, S3C_GPIO_INPUT);
			s3c_gpio_setpull(GPIO_GPH06, S3C_GPIO_PULL_DOWN);
			//s3c_gpio_setpin(GPIO_GPH06, 0);
		}
	}
	else {  // DET_3.5
		//s3c_gpio_cfgpin(GPIO_DET_35, S3C_GPIO_INPUT);
		//s3c_gpio_setpull(GPIO_DET_35, S3C_GPIO_PULL_NONE);
		//s3c_gpio_setpin(GPIO_DET_35, 0);
	}

	//s3c_gpio_cfgpin(GPIO_AP_PMIC_IRQ, S3C_GPIO_INPUT);
	//s3c_gpio_setpull(GPIO_AP_PMIC_IRQ, S3C_GPIO_PULL_NONE);
	//s3c_gpio_setpin(GPIO_AP_PMIC_IRQ, 0);

	if(HWREV >= 0x4) {  // DET_3.5
		//s3c_gpio_cfgpin(GPIO_DET_35_R04, S3C_GPIO_INPUT);
		//s3c_gpio_setpull(GPIO_DET_35_R04, S3C_GPIO_PULL_NONE);
		//s3c_gpio_setpin(GPIO_DET_35_R04, 0);
	}
	else {  // NC
		s3c_gpio_cfgpin(GPIO_GPH10, S3C_GPIO_OUTPUT);
		s3c_gpio_setpull(GPIO_GPH10, S3C_GPIO_PULL_NONE);
		s3c_gpio_setpin(GPIO_GPH10, 0);
	}

	//s3c_gpio_cfgpin(GPIO_TA_nCHG, S3C_GPIO_INPUT);
	//s3c_gpio_setpull(GPIO_TA_nCHG, S3C_GPIO_PULL_NONE);
	//s3c_gpio_setpin(GPIO_TA_nCHG, 0);

	s3c_gpio_cfgpin(GPIO_MHL_INT, S3C_GPIO_INPUT);
	s3c_gpio_setpull(GPIO_MHL_INT, S3C_GPIO_PULL_DOWN);
//	s3c_gpio_setpin(GPIO_MHL_INT, 1);

	//s3c_gpio_cfgpin(GPIO_nINT_ONEDRAM_AP, S3C_GPIO_INPUT);
	//s3c_gpio_setpull(GPIO_nINT_ONEDRAM_AP, S3C_GPIO_PULL_NONE);
	//s3c_gpio_setpin(GPIO_nINT_ONEDRAM_AP, 0);

	if(HWREV >= 0x4) {  // SEND_END
		//s3c_gpio_cfgpin(GPIO_EAR_SEND_END_R04, S3C_GPIO_INPUT);
		//s3c_gpio_setpull(GPIO_EAR_SEND_END_R04, S3C_GPIO_PULL_NONE);
		//s3c_gpio_setpin(GPIO_EAR_SEND_END_R04, 0);
	}
	else {  // NC
		s3c_gpio_cfgpin(GPIO_GPH14, S3C_GPIO_OUTPUT);
		s3c_gpio_setpull(GPIO_GPH14, S3C_GPIO_PULL_NONE);
		s3c_gpio_setpin(GPIO_GPH14, 0);
	}

	s3c_gpio_cfgpin(GPIO_HDMI_HPD, S3C_GPIO_INPUT);
	s3c_gpio_setpull(GPIO_HDMI_HPD, S3C_GPIO_PULL_DOWN);
//	s3c_gpio_setpin(GPIO_HDMI_HPD,0);

	//s3c_gpio_cfgpin(GPIO_FUEL_ARLT, S3C_GPIO_INPUT);
	//s3c_gpio_setpull(GPIO_FUEL_ARLT, S3C_GPIO_PULL_NONE);
	//s3c_gpio_setpin(GPIO_FUEL_ARLT, 0);

	//s3c_gpio_cfgpin(GPIO_PHONE_ACTIVE, S3C_GPIO_INPUT);
	//s3c_gpio_setpull(GPIO_PHONE_ACTIVE, S3C_GPIO_PULL_NONE);
	//s3c_gpio_setpin(GPIO_PHONE_ACTIVE, 0);

	if(HWREV >= 12) {  // REMOTE_SENSE_IRQ (GT-P1000 Rev0.6)
		s3c_gpio_cfgpin(GPIO_REMOTE_SENSE_IRQ, S3C_GPIO_INPUT);
		s3c_gpio_setpull(GPIO_REMOTE_SENSE_IRQ, S3C_GPIO_PULL_NONE);
		//s3c_gpio_setpin(GPIO_REMOTE_SENSE_IRQ, 0);
	}
	else {
		s3c_gpio_cfgpin(GPIO_GPH20, S3C_GPIO_OUTPUT);
		s3c_gpio_setpull(GPIO_GPH20, S3C_GPIO_PULL_NONE);
		s3c_gpio_setpin(GPIO_GPH20, 0);
	}

	if(HWREV >= 12) {  // TOUCH_EN (GT-P1000 Rev0.6)
		s3c_gpio_cfgpin(GPIO_TOUCH_EN_REV06, S3C_GPIO_OUTPUT);
		s3c_gpio_setpull(GPIO_TOUCH_EN_REV06, S3C_GPIO_PULL_NONE);
		s3c_gpio_setpin(GPIO_TOUCH_EN_REV06, 0);
	}
	else {  // NC
		s3c_gpio_cfgpin(GPIO_GPH21, S3C_GPIO_OUTPUT);
		s3c_gpio_setpull(GPIO_GPH21, S3C_GPIO_PULL_NONE);
		s3c_gpio_setpin(GPIO_GPH21, 0);
	}

	if(HWREV >= 12) {  // GYRO_INT (GT-P1000 Rev0.6)
		s3c_gpio_cfgpin(GPIO_GYRO_INT, S3C_GPIO_INPUT);
		s3c_gpio_setpull(GPIO_GYRO_INT, S3C_GPIO_PULL_DOWN);
		//s3c_gpio_setpin(GPIO_GYRO_INT, 0);
	}
	else {  // NC
		s3c_gpio_cfgpin(GPIO_GPH22, S3C_GPIO_OUTPUT);
		s3c_gpio_setpull(GPIO_GPH22, S3C_GPIO_PULL_NONE);
		s3c_gpio_setpin(GPIO_GPH22, 0);
	}

	if(HWREV >= 0x5) {  // NC
		if(HWREV == 15) {  // WAKEUP_KEY(P1000 Rev0.9)
			s3c_gpio_cfgpin(GPIO_GPH23, S3C_GPIO_INPUT);
			s3c_gpio_setpull(GPIO_GPH23, S3C_GPIO_PULL_NONE);
			//s3c_gpio_setpin(GPIO_GPH23, 0);
		}
		else {  // NC
			s3c_gpio_cfgpin(GPIO_GPH23, S3C_GPIO_INPUT);
			s3c_gpio_setpull(GPIO_GPH23, S3C_GPIO_PULL_DOWN);
			//s3c_gpio_setpin(GPIO_GPH23, 0);
		}
	}
	else {  // TOUCH_KEY_INT
		s3c_gpio_cfgpin(GPIO_TOUCH_KEY_INT, S3C_GPIO_INPUT);
		s3c_gpio_setpull(GPIO_TOUCH_KEY_INT, S3C_GPIO_PULL_NONE);
		//s3c_gpio_setpin(GPIO_GPH23, 0);
	}

	//s3c_gpio_cfgpin(GPIO_WLAN_HOST_WAKE, S3C_GPIO_OUTPUT);
	//s3c_gpio_setpull(GPIO_WLAN_HOST_WAKE, S3C_GPIO_PULL_NONE);
	//s3c_gpio_setpin(GPIO_WLAN_HOST_WAKE, 0);

	//s3c_gpio_cfgpin(GPIO_BT_HOST_WAKE, S3C_GPIO_OUTPUT);
	//s3c_gpio_setpull(GPIO_BT_HOST_WAKE, S3C_GPIO_PULL_NONE);
	//s3c_gpio_setpin(GPIO_BT_HOST_WAKE, 0);

	//s3c_gpio_cfgpin(GPIO_nPOWER, S3C_GPIO_INPUT);
	//s3c_gpio_setpull(GPIO_nPOWER, S3C_GPIO_PULL_NONE);
	//s3c_gpio_setpin(GPIO_nPOWER, 0);

	//s3c_gpio_cfgpin(GPIO_JACK_nINT, S3C_GPIO_INPUT);
	//s3c_gpio_setpull(GPIO_JACK_nINT, S3C_GPIO_PULL_NONE);
	//s3c_gpio_setpin(GPIO_JACK_nINT, 0);

#if 0 // keypad
	s3c_gpio_cfgpin(GPIO_KBR0, S3C_GPIO_INPUT);
	s3c_gpio_setpull(GPIO_KBR0, S3C_GPIO_PULL_NONE);
	//s3c_gpio_setpin(GPIO_KBR0, 0);

	s3c_gpio_cfgpin(GPIO_KBR1, S3C_GPIO_INPUT);
	s3c_gpio_setpull(GPIO_KBR1, S3C_GPIO_PULL_NONE);
	//s3c_gpio_setpin(GPIO_KBR1, 0);
#endif

	s3c_gpio_cfgpin(GPIO_MSENSE_IRQ, S3C_GPIO_INPUT);
	s3c_gpio_setpull(GPIO_MSENSE_IRQ, S3C_GPIO_PULL_NONE);
	//s3c_gpio_setpin(GPIO_MSENSE_IRQ, 0);

	if(HWREV >= 0x6) {  // SIM_DETECT
		//s3c_gpio_cfgpin(GPIO_SIM_nDETECT, S3C_GPIO_INPUT);
		//s3c_gpio_setpull(GPIO_SIM_nDETECT, S3C_GPIO_PULL_NONE);
		//s3c_gpio_setpin(GPIO_SIM_nDETECT, 0);
	}
	else {  // NC
		s3c_gpio_cfgpin(GPIO_GPH33, S3C_GPIO_OUTPUT);
		s3c_gpio_setpull(GPIO_GPH33, S3C_GPIO_PULL_NONE);
		s3c_gpio_setpin(GPIO_GPH33, 0);
	}

	//s3c_gpio_cfgpin(GPIO_T_FLASH_DETEC, S3C_GPIO_INPUT);
	//s3c_gpio_setpull(GPIO_T_FLASH_DETECT, S3C_GPIO_PULL_NONE);
	//s3c_gpio_setpin(GPIO_T_FLASH_DETECT, 0);

	if(HWREV >= 11) {   // DOCK_INT (GT-P1000 Rev0.5)
		s3c_gpio_cfgpin(GPIO_DOCK_INT, S3C_GPIO_INPUT);
		s3c_gpio_setpull(GPIO_DOCK_INT, S3C_GPIO_PULL_NONE);
		//s3c_gpio_setpin(GPIO_DOCK_INT, 0);
	}
	else {  // NC
		s3c_gpio_cfgpin(GPIO_GPH35, S3C_GPIO_OUTPUT);
		s3c_gpio_setpull(GPIO_GPH35, S3C_GPIO_PULL_NONE);
		s3c_gpio_setpin(GPIO_GPH35, 0);
	}

	if(HWREV >= 0x4) {  // NC
		s3c_gpio_cfgpin(GPIO_GPH36, S3C_GPIO_INPUT);
		s3c_gpio_setpull(GPIO_GPH36, S3C_GPIO_PULL_DOWN);
		//s3c_gpio_setpin(GPIO_GPH36, 0);
	}
	else {  // SEND_END
		//s3c_gpio_cfgpin(GPIO_EAR_SEND_END, S3C_GPIO_INPUT);
		//s3c_gpio_setpull(GPIO_EAR_SEND_END, S3C_GPIO_PULL_NONE);
		//s3c_gpio_setpin(GPIO_EAR_SEND_END, 0);
	}

	s3c_gpio_cfgpin(GPIO_CP_RST, S3C_GPIO_OUTPUT);
	s3c_gpio_setpull(GPIO_CP_RST, S3C_GPIO_PULL_UP);
	//s3c_gpio_setpin(GPIO_CP_RST, 1);

	if(HWREV >= 12) {  // Above P1000 Rev0.6 (1.2)
		s3c_config_sleep_gpio_table(ARRAY_SIZE(p1_r12_sleep_gpio_table),
			p1_r12_sleep_gpio_table);

		if(HWREV >= 15) {  // Above P1000 Rev0.9
			s3c_config_sleep_gpio_table(ARRAY_SIZE(p1_r15_sleep_gpio_table),
				p1_r15_sleep_gpio_table);
		}
		
		if(HWREV >= 16) {  // Above P1000 Rev1.0
			s3c_config_sleep_gpio_table(ARRAY_SIZE(p1_r16_sleep_gpio_table),
				p1_r16_sleep_gpio_table);
		}

		if(HWREV >= 18) {  // Above P1000 Rev1.2
			s3c_config_sleep_gpio_table(ARRAY_SIZE(p1_r18_sleep_gpio_table),
				p1_r18_sleep_gpio_table);
		}
#if defined(CONFIG_KEYBOARD_P1)
            if((HWREV >= 13)&&(HWREV <= 15)||(HWREV >= 18))
            {
                    s3c_config_sleep_gpio_table(ARRAY_SIZE(p1_keyboard_sleep_gpio_table),
				p1_keyboard_sleep_gpio_table);
            }
#endif
	}
	else if(HWREV >= 8) {  // Above P1000 Rev0.2 (0.8)
		s3c_config_sleep_gpio_table(ARRAY_SIZE(p1_r09_sleep_gpio_table),
			p1_r09_sleep_gpio_table);

		if(HWREV == 11) {  // Rev0.5 : only 1 gpio status is different from Rev0.3 (0.9)
			s3c_config_sleep_gpio_table(ARRAY_SIZE(p1_r11_sleep_gpio_table),
				p1_r11_sleep_gpio_table);
		}

		if(HWREV == 8) {  // Rev0.2 : only 3 gpio status is different from Rev0.3 (0.9)
			s3c_config_sleep_gpio_table(ARRAY_SIZE(p1_r08_sleep_gpio_table),
				p1_r08_sleep_gpio_table);
		}
	}
	else if(HWREV >= 5) {  // Above Rev0.5
		s3c_config_sleep_gpio_table(ARRAY_SIZE(p1_r05_sleep_gpio_table),
			p1_r05_sleep_gpio_table);
	}
	else {  // Under Rev0.4
		s3c_config_sleep_gpio_table(ARRAY_SIZE(p1_sleep_gpio_table),
			p1_sleep_gpio_table);
	}


	if (get_machine_type() == MACHINE_P1_AMOLED) {
		if(HWREV < 0x5)
			{
			s3c_config_sleep_gpio_table(ARRAY_SIZE(p1_lcd_amoled_sleep_gpio_table),
				p1_lcd_amoled_sleep_gpio_table);
			}
		else
			{
			//PWM
			s3c_config_sleep_gpio_table(ARRAY_SIZE(p1_lcd_tft_sleep_gpio_table),
				p1_lcd_tft_sleep_gpio_table);
			}
	}
	else if (get_machine_type() == MACHINE_P1_TFT) {
		s3c_config_sleep_gpio_table(ARRAY_SIZE(p1_lcd_tft_sleep_gpio_table),
			p1_lcd_tft_sleep_gpio_table);
	}

    //Derek: REV10 Sleep GPIO
	if(HWREV >= 16) {  
		s3c_gpio_cfgpin(GPIO_ATV_RSTn_REV10, S3C_GPIO_INPUT);
		s3c_gpio_setpull(GPIO_ATV_RSTn_REV10, S3C_GPIO_PULL_DOWN);

		s3c_gpio_cfgpin(GPIO_ISDBT_PWR_EN_REV10, S3C_GPIO_INPUT);
		s3c_gpio_setpull(GPIO_ISDBT_PWR_EN_REV10, S3C_GPIO_PULL_DOWN);
	}

	if(HWREV == 16) {  

        s3c_gpio_cfgpin(GPIO_TV_CLK_EN, S3C_GPIO_OUTPUT);
		s3c_gpio_setpull(GPIO_TV_CLK_EN, S3C_GPIO_PULL_NONE);
	}
}

EXPORT_SYMBOL(s3c_config_sleep_gpio);


extern void set_pmic_gpio(void);

void p1_init_gpio(void)
{
	s3c_config_gpio_table(ARRAY_SIZE(p1_gpio_table),
			p1_gpio_table);

	if(HWREV <= 0x3) {  // Under Rev0.3
		s3c_config_gpio_table(ARRAY_SIZE(p1_motor_i2c_gpio_table),
			p1_motor_i2c_gpio_table);
	}
	if(HWREV <= 0x4) {  // Under Rev0.4
		s3c_config_gpio_table(ARRAY_SIZE(p1_touch_i2c_gpio_table),
			p1_touch_i2c_gpio_table);
	}

	if (get_machine_type() == MACHINE_P1_AMOLED)	{
		if(HWREV < 0x5)
			{
			s3c_config_gpio_table(ARRAY_SIZE(p1_lcd_amoled_gpio_table),
				p1_lcd_amoled_gpio_table);
			}
		else
			{
			//PWM
			s3c_config_gpio_table(ARRAY_SIZE(p1_lcd_tft_gpio_table),
				p1_lcd_tft_gpio_table);
			}
	} else if (get_machine_type() == MACHINE_P1_TFT)	{
		s3c_config_gpio_table(ARRAY_SIZE(p1_lcd_tft_gpio_table),
			p1_lcd_tft_gpio_table);
	}

	/*Adding pmic gpio(GPH2, GPH3, GPH4) initialisation*/
	printk("MIDAS@%s(%d): set_pmic_gpio\n", __func__, __LINE__);
	set_pmic_gpio();
}

static struct i2c_board_info i2c_devs3[] __initdata = {
    {
        I2C_BOARD_INFO("qt602240_ts", 0x4a),
    },
};

static struct i2c_board_info i2c_devs11[] __initdata = {
    {
        I2C_BOARD_INFO("melfas_touchkey_i2c", 0x20),
    },
};

#ifdef CONFIG_GYRO_L3G4200D
static struct l3g4200d_platform_data l3g4200d_p1p2_platform_data = {
};
#endif

static struct i2c_board_info i2c_devs5[] __initdata = {
	{
		I2C_BOARD_INFO("bma020", 0x38),
	},
#ifdef CONFIG_GYRO_L3G4200D
	{
		I2C_BOARD_INFO("l3g4200d", 0x68),
		.platform_data = &l3g4200d_p1p2_platform_data,
	},
#endif
};

#if 0		// module temp. removed
static struct haptic_platform_data aquila_haptic_L_data = {
	.name		= "vibetones_L",
	.pwm_timer	= 1,			/* Use PWM 3 */
	.gpio		= GPIO_HAPTIC_EN_L, //S5PV210_GPJ1(1),	/* XMSMADDR_9 */
	.ldo_level	= ISA1200_LDOADJ_36V,	/* LDOADJ : 3.6V */
};

static struct i2c_board_info i2c_devs7[] __initdata = {
	{
		I2C_BOARD_INFO("isa1200_R", 0x90 >> 1),
		.platform_data = &aquila_haptic_R_data,
	}, 
};
#endif

static void ambient_light_sensor_reset(void)
{
	int gpio;

	/* BH1721FVC */
	gpio = S5PV210_GPG2(2);		/* XMMC2CDn */
	s3c_gpio_cfgpin(gpio, S3C_GPIO_OUTPUT);
	gpio_request(gpio, "ALS_nRST");
	gpio_direction_output(gpio, 0);

	gpio_set_value(gpio, 0);
	/* More than 1us */
	udelay(2);
	gpio_set_value(gpio, 1);
}

static struct bh1721_platform_data bh1721_p1p2_platform_data = {
	.reset = ambient_light_sensor_reset,
};

#if 0		// module temp. removed
static struct haptic_platform_data aquila_haptic_R_data = {
	.name		= "vibetones_R",
	.pwm_timer	= 2,			/* Use PWM 2 */
	.gpio		= GPIO_HAPTIC_EN_R, //S5PV210_GPJ1(1),	/* XMSMADDR_9 */
	.ldo_level	= ISA1200_LDOADJ_36V,	/* LDOADJ : 3.6V */
};
#endif

static struct i2c_board_info i2c_devs10[] __initdata = {
	{
		I2C_BOARD_INFO("ak8973", 0x1c),
	},
	{
		I2C_BOARD_INFO("bh1721", 0x23),
		.platform_data	= &bh1721_p1p2_platform_data,
	},
#if 0		// module temp. removed
	{
		I2C_BOARD_INFO("isa1200_L", 0x90 >> 1),
		.platform_data = &aquila_haptic_L_data,
	},
#endif	
};

/* touch screen device init */
static void __init qt_touch_init(void)
{
    int gpio, irq;

    if(HWREV >= 0xc)
        gpio = S5PV210_GPH2(1);
    else
        gpio = S5PV210_GPG3(6);			/* XMMC3DATA_3 */
    
    gpio_request(gpio, "TOUCH_EN");
    s3c_gpio_cfgpin(gpio, S3C_GPIO_OUTPUT);
    gpio_direction_output(gpio, 1);

#if defined(CONFIG_TARGET_LOCALE_LTN)
    gpio = S5PV210_GPG0(2);			
#else
    gpio = S5PV210_GPJ0(5);				/* XMSMADDR_5 */
#endif
    gpio_request(gpio, "TOUCH_INT");
    s3c_gpio_cfgpin(gpio, S3C_GPIO_SFN(0xf));
    s3c_gpio_setpull(gpio, S3C_GPIO_PULL_UP);
    irq = gpio_to_irq(gpio);
    i2c_devs3[0].irq = irq;
}

static void __init cmc623_setting(void)
{
    printk("**************************************\n");
    printk("**** < cmc623_setting >          *****\n");
    printk("**************************************\n");

	if (gpio_is_valid(GPIO_CMC_SLEEP)) {
		if (gpio_request(GPIO_CMC_SLEEP, "CMC_SLEEP"))
			printk(KERN_ERR "Filed to request GPIO_CMC_SLEEP!\n");
		gpio_direction_output(GPIO_CMC_SLEEP, GPIO_LEVEL_LOW);
	}
	s3c_gpio_setpull(GPIO_CMC_SLEEP, S3C_GPIO_PULL_NONE);

	if (gpio_is_valid(GPIO_CMC_EN)) {
		if (gpio_request(GPIO_CMC_EN, "CMC_EN"))
			printk(KERN_ERR "Filed to request GPIO_CMC_EN!\n");
		gpio_direction_output(GPIO_CMC_EN, GPIO_LEVEL_HIGH);
	}
	s3c_gpio_setpull(GPIO_CMC_EN, S3C_GPIO_PULL_NONE);

	if (gpio_is_valid(GPIO_CMC_RST)) {
		if (gpio_request(GPIO_CMC_RST, "CMC_RST"))
        {      
			printk(KERN_ERR "Filed to request GPIO_CMC_RST!\n");
        }
        else 
        {
            printk(KERN_ERR "Success to request GPIO_CMC_RST!\n");
        }
		gpio_direction_output(GPIO_CMC_RST, GPIO_LEVEL_HIGH);
	}
	s3c_gpio_setpull(GPIO_CMC_RST, S3C_GPIO_PULL_NONE);

	if (gpio_is_valid(GPIO_CMC_SHDN)) {
		if (gpio_request(GPIO_CMC_SHDN, "CMC_SHDN"))
			printk(KERN_ERR "Filed to request GPIO_CMC_SHDN!\n");
		gpio_direction_output(GPIO_CMC_SHDN, GPIO_LEVEL_HIGH);
	}
	s3c_gpio_setpull(GPIO_CMC_SHDN, S3C_GPIO_PULL_NONE);

	if (gpio_is_valid(GPIO_CMC_BYPASS)) {
		if (gpio_request(GPIO_CMC_BYPASS, "CMC_BYPASS"))
        {      
			printk(KERN_ERR "Filed to request GPIO_CMC_BYPASS!\n");
        }
        else 
        {
            printk(KERN_ERR "Success to request GPIO_CMC_BYPASS!\n");
        }
		gpio_direction_output(GPIO_CMC_BYPASS, GPIO_LEVEL_LOW);
	}
	s3c_gpio_setpull(GPIO_CMC_BYPASS, S3C_GPIO_PULL_NONE);

	// BYPASS MODE
	gpio_set_value(GPIO_CMC_EN, GPIO_LEVEL_HIGH);
	gpio_set_value(GPIO_CMC_RST, GPIO_LEVEL_HIGH);
	gpio_set_value(GPIO_CMC_SHDN, GPIO_LEVEL_HIGH);
	gpio_set_value(GPIO_CMC_BYPASS, GPIO_LEVEL_LOW);
	gpio_set_value(GPIO_CMC_SLEEP, GPIO_LEVEL_HIGH);
    mdelay(1);
	gpio_set_value(GPIO_CMC_RST, GPIO_LEVEL_LOW);
    mdelay(4);
	gpio_set_value(GPIO_CMC_RST, GPIO_LEVEL_HIGH);
}


#if defined(CONFIG_TARGET_LOCALE_LTN)  // CYS_ 2010.06.29 for ATV Gpio Config
static void __init nmi_i2s_cfg_gpio_init(void)
{
	s3c_gpio_cfgpin(GPIO_I2S_SCLK_18V, S3C_GPIO_SFN(0x4));
	s3c_gpio_cfgpin(GPIO_I2S_MCLK_18V, S3C_GPIO_SFN(0x4));
	s3c_gpio_cfgpin(GPIO_I2S_LRCLK_18V, S3C_GPIO_SFN(0x4));
	s3c_gpio_cfgpin(GPIO_I2S_DATA_18V, S3C_GPIO_SFN(0x4));

	s3c_gpio_setpull(S5PV210_GPC1(0), S3C_GPIO_PULL_NONE);
	s3c_gpio_setpull(S5PV210_GPC1(1), S3C_GPIO_PULL_NONE);
	s3c_gpio_setpull(S5PV210_GPC1(2), S3C_GPIO_PULL_NONE);
	s3c_gpio_setpull(S5PV210_GPC1(3), S3C_GPIO_PULL_NONE);
}
#endif


/* << add board specific component - end */
/************************************/

static void __init smdkc110_machine_init(void)
{
	unsigned int model_rev = 0;
	unsigned char model_str[12];

	printk("MIDAS@%s: +\n", __func__);
	/* Find out S5PC110 chip version */
	_hw_version_check();

	// Read HWREV_MODE gpio status
	s3c_gpio_cfgpin(GPIO_HWREV_MODE0, S3C_GPIO_INPUT);
	s3c_gpio_setpull( GPIO_HWREV_MODE0, S3C_GPIO_PULL_NONE); 
	s3c_gpio_cfgpin(GPIO_HWREV_MODE1, S3C_GPIO_INPUT);
	s3c_gpio_setpull( GPIO_HWREV_MODE1, S3C_GPIO_PULL_NONE);  
	s3c_gpio_cfgpin(GPIO_HWREV_MODE2, S3C_GPIO_INPUT);
	s3c_gpio_setpull( GPIO_HWREV_MODE2, S3C_GPIO_PULL_NONE); 
	s3c_gpio_cfgpin(GPIO_HWREV_MODE3, S3C_GPIO_INPUT);
	s3c_gpio_setpull( GPIO_HWREV_MODE3, S3C_GPIO_PULL_NONE); 
	s3c_gpio_cfgpin(GPIO_HWREV_MODE4, S3C_GPIO_INPUT);
	s3c_gpio_setpull( GPIO_HWREV_MODE4, S3C_GPIO_PULL_NONE); 
	s3c_gpio_cfgpin(GPIO_HWREV_MODE5, S3C_GPIO_INPUT);
	s3c_gpio_setpull( GPIO_HWREV_MODE5, S3C_GPIO_PULL_NONE); 

	HWREV = gpio_get_value(GPIO_HWREV_MODE0);
	HWREV = HWREV | (gpio_get_value(GPIO_HWREV_MODE1) <<1);
	HWREV = HWREV | (gpio_get_value(GPIO_HWREV_MODE2) <<2);
	HWREV = HWREV | (gpio_get_value(GPIO_HWREV_MODE3) <<3);

	model_rev = (gpio_get_value(GPIO_HWREV_MODE4) << 1) | gpio_get_value(GPIO_HWREV_MODE5);
	switch(model_rev)
	{
		case 0 :
			sprintf(model_str, "P1_AMOLED");
			break;
		case 1:
			sprintf(model_str, "P2");
			break;
		case 2:
#if defined(CONFIG_TARGET_LOCALE_LTN)
			sprintf(model_str, "GT-P1000L");
#else
			sprintf(model_str, "GT-P1000");
#endif				
			break;
		case 3:
			sprintf(model_str, "P1");
			break;
		default:
			sprintf(model_str, "Unknown");
			break;
	}

	if(model_rev == 0x2)  // GT-P1000
		HWREV += 6;  // Rev0.6

	printk("%s: HWREV (0x%x), Model (%s)\n", __func__, HWREV, model_str);

	p1_init_gpio();
	
	qt_touch_init();
	
	/* OneNAND */
#ifdef CONFIG_MTD_ONENAND
	//s3c_device_onenand.dev.platform_data = &s5p_onenand_data;
#endif

#ifdef CONFIG_DM9000
	smdkv210_dm9000_set();
#endif

#ifdef CONFIG_ANDROID_PMEM
	android_pmem_set_platdata();
#endif
	/* i2c */
	s3c_i2c0_set_platdata(NULL);
	s3c_i2c1_set_platdata(NULL);
	s3c_i2c2_set_platdata(NULL);
	i2c_register_board_info(0, i2c_devs0, ARRAY_SIZE(i2c_devs0));
	i2c_register_board_info(1, i2c_devs1, ARRAY_SIZE(i2c_devs1));
	i2c_register_board_info(2, i2c_devs3, ARRAY_SIZE(i2c_devs3));
	i2c_register_board_info(4, i2c_devs4, ARRAY_SIZE(i2c_devs4));
	if(HWREV < 0x0F)
		i2c_register_board_info(5, i2c_devs5, ARRAY_SIZE(i2c_devs5));
	else
	{
		i2c_devs5[ARRAY_SIZE(i2c_devs5)-1].addr += 1;						// From HW rev 0.9, slave addres is changed from 0x68 to 0x69
		i2c_register_board_info(5, i2c_devs5, ARRAY_SIZE(i2c_devs5));
	}
		
	i2c_register_board_info(6, i2c_devs6, ARRAY_SIZE(i2c_devs6));
	i2c_register_board_info(8, i2c_devs8, ARRAY_SIZE(i2c_devs8));
	i2c_register_board_info(9, i2c_devs9, ARRAY_SIZE(i2c_devs9));
	i2c_register_board_info(10, i2c_devs10, ARRAY_SIZE(i2c_devs10));
	i2c_register_board_info(12, i2c_devs12, ARRAY_SIZE(i2c_devs12));
#if defined(CONFIG_TARGET_LOCALE_LTN)  // CYS_ 2010.05.07
	i2c_register_board_info(13, i2c_devs13, ARRAY_SIZE(i2c_devs13));
#endif

	if(HWREV >= 0x8)
		i2c_register_board_info(7, i2c_devs7, ARRAY_SIZE(i2c_devs7));

	if(HWREV <= 0x4)
	{
        	i2c_register_board_info(11, i2c_devs11, ARRAY_SIZE(i2c_devs11));
	}

	pm_power_off = smdkc110_power_off;

#ifdef CONFIG_FB_S3C
//	s3cfb_set_platdata(NULL);
#endif

#if defined(CONFIG_BLK_DEV_IDE_S3C)
	s3c_ide_set_platdata(&smdkv210_ide_pdata);
#endif

#if defined(CONFIG_TOUCHSCREEN_S3C)
	s3c_ts_set_platdata(&s3c_ts_platform);
#endif

#if defined(CONFIG_S5PV210_ADCTS)
	s3c_adcts_set_platdata(&s3c_adcts_cfgs);
#endif

	if(HWREV >= 0x9) {
#ifdef CONFIG_FB_S3C_LVDS
#if defined(CONFIG_FB_S3C_CMC623)
		platform_device_register(&cmc623_pwm_backlight);
#endif
		platform_device_register(&sec_device_lms700);
		s3cfb_set_platdata(&lvds_data);
#endif
	}
	else if (get_machine_type() == MACHINE_P1_TFT) {
#ifdef CONFIG_FB_S3C_LVDS
		swi_register_board_info(&swi_board_info[0], 1);
//     	platform_device_register(&mdnie_pwm_backlight);
		platform_device_register(&sec_device_lms700);
		s3cfb_set_platdata(&lvds_data);
#endif
	} else if (get_machine_type() == MACHINE_P1_AMOLED) {
		if(HWREV < 0x5) {
			qt602240_set_amoled_display(1);
#ifdef CONFIG_FB_S3C_AMS701KA
			spi_register_board_info(spi_board_info_ams701ka, ARRAY_SIZE(spi_board_info_ams701ka));
			s3cfb_set_platdata(&ams701ka_data);
#endif
		} else {
			//PWM
#ifdef CONFIG_FB_S3C_LVDS
			platform_device_register(&mdnie_pwm_backlight);
			platform_device_register(&sec_device_lms700);
			s3cfb_set_platdata(&lvds_data);
#endif
		}
	}

#if defined(CONFIG_PM)
	s3c_pm_init();
#endif

#ifdef CONFIG_VIDEO_FIMC
	/* fimc */
	s3c_fimc0_set_platdata(&fimc_plat);
	s3c_fimc1_set_platdata(&fimc_plat);
	s3c_fimc2_set_platdata(&fimc_plat);
	s3c_csis_set_platdata(NULL);

#if defined(CONFIG_MACH_SMDKC110) && !defined(CONFIG_MACH_S5PC110_P1)
	smdkv210_cam0_power(1);
	smdkv210_cam1_power(1);
	smdkv210_mipi_cam_reset();
#endif

#endif

#ifdef CONFIG_VIDEO_MFC50
	/* mfc */
	s3c_mfc_set_platdata(NULL);
#endif

#ifdef CONFIG_S3C_DEV_HSMMC
	s5pv210_default_sdhci0();
#endif
#ifdef CONFIG_S3C_DEV_HSMMC1
	if (HWREV < 12)
		s5pv210_default_sdhci1();
#endif
#ifdef CONFIG_S3C_DEV_HSMMC2
	s5pv210_default_sdhci2();
#endif
#ifdef CONFIG_S3C_DEV_HSMMC3
	s5pv210_default_sdhci3();
#endif
#ifdef CONFIG_S5PV210_SETUP_SDHCI
	s3c_sdhci_set_platdata();
#endif

	if(HWREV >= 0xc)
        	i2c8_platdata.scl_pin = GPIO_CMC_SCL_18V_REV06;

	if (HWREV >= 12)
		platform_add_devices(smdkc110_devices, ARRAY_SIZE(smdkc110_devices));
	else
		platform_add_devices(smdkc110_old_devices, ARRAY_SIZE(smdkc110_old_devices));

	if(HWREV != 0x7) {
		platform_device_register(&modemctl);
		platform_device_register(&onedram);
	}	

#if 0 // defined(CONFIG_HAVE_PWM)
	smdk_backlight_register();
#endif

	if(HWREV <= 0x3)  // Under Rev0.3
	{
		platform_device_register(&s3c_device_i2c7);
		platform_device_register(&sec_device_jack);
	}
	else {
		platform_device_register(&sec_device_jack_r04);
	}

#if defined(CONFIG_FB_S3C_CMC623)
	if(HWREV >= 0x09)
	    platform_device_register(&sec_device_tune_cmc623);
#endif

	
	if(HWREV >= 0x08)
		platform_device_register(&s3c_device_i2c7);

	if(HWREV <= 0x4)  // Under Rev0.4
		platform_device_register(&s3c_device_i2c11);

#ifdef CONFIG_FB_S3C_AMS701KA
	if (get_machine_type() == MACHINE_P1_AMOLED && HWREV < 0x5)	{
		platform_device_register(&s3c_device_ams701ka_spi_gpio);
	}
#endif

	/* GPIO Keys */
	if(HWREV >= 0x5)
            platform_device_register(&gpio_keys_device_rev05);
	else
            platform_device_register(&gpio_keys_device);

	gpio_keys_cfg_gpio();

	if(HWREV >= 0x9)
	{
//		cmc623_setting();   // P1_LSJ : DE10
	}

//Tvout driver (over rev02)
	if(HWREV >= 0x8)
	{
#ifdef CONFIG_VIDEO_TV20
		platform_device_register(&s5p_device_tvout);
		platform_device_register(&s5p_device_cec);
		platform_device_register(&s5p_device_hpd);
#endif
#ifdef CONFIG_30PIN_CONN
		platform_device_register(&sec_device_connector);
#endif
	}
	
#if defined(CONFIG_TARGET_LOCALE_LTN)  // CYS_ 2010.06.29 for ATV Gpio Config
	nmi_i2s_cfg_gpio_init();
#endif

	register_reboot_notifier(&arise_reboot_notifier);
	
	p1_switch_init();

	BUG_ON(!sec_class);
	gps_dev = device_create(sec_class, NULL, 0, NULL, "gps");
	if (IS_ERR(gps_dev))
		pr_err("Failed to create device(gps)!\n");

	if(HWREV < 0x9)
		{
		lvds_backlight_on(NULL);
		lvds_reset_lcd(NULL);
		}

	printk("MIDAS@%s: -\n", __func__);
}

#ifdef CONFIG_USB_SUPPORT
/* Initializes OTG Phy. */
void otg_phy_init(void)
{
	__raw_writel(__raw_readl(S5P_USB_PHY_CONTROL)
		|(0x1<<0), S5P_USB_PHY_CONTROL); /*USB PHY0 Enable */
	__raw_writel((__raw_readl(S3C_USBOTG_PHYPWR)
		&~(0x3<<3)&~(0x1<<0))|(0x1<<5), S3C_USBOTG_PHYPWR);
	__raw_writel((__raw_readl(S3C_USBOTG_PHYCLK)
		&~(0x5<<2))|(0x3<<0), S3C_USBOTG_PHYCLK);
	__raw_writel((__raw_readl(S3C_USBOTG_RSTCON)
		&~(0x3<<1))|(0x1<<0), S3C_USBOTG_RSTCON);
	udelay(10);
	__raw_writel(__raw_readl(S3C_USBOTG_RSTCON)
		&~(0x7<<0), S3C_USBOTG_RSTCON);
	udelay(10);
}
EXPORT_SYMBOL(otg_phy_init);

/* USB Control request data struct must be located here for DMA transfer */
struct usb_ctrlrequest usb_ctrl __attribute__((aligned(8)));
EXPORT_SYMBOL(usb_ctrl);

/* OTG PHY Power Off */
void otg_phy_off(void)
{
	__raw_writel(__raw_readl(S3C_USBOTG_PHYPWR)
		|(0x3<<3), S3C_USBOTG_PHYPWR);
	__raw_writel(__raw_readl(S5P_USB_PHY_CONTROL)
		&~(1<<0), S5P_USB_PHY_CONTROL);
}
EXPORT_SYMBOL(otg_phy_off);

void usb_host_phy_init(void)
{
	struct clk *otg_clk;

	otg_clk = clk_get(NULL, "otg");
	clk_enable(otg_clk);

	if (readl(S5P_USB_PHY_CONTROL) & (0x1<<1))
		return;

	__raw_writel(__raw_readl(S5P_USB_PHY_CONTROL)
		|(0x1<<1), S5P_USB_PHY_CONTROL);
	__raw_writel((__raw_readl(S3C_USBOTG_PHYPWR)
		&~(0x1<<7)&~(0x1<<6))|(0x1<<8)|(0x1<<5), S3C_USBOTG_PHYPWR);
	__raw_writel((__raw_readl(S3C_USBOTG_PHYCLK)
		&~(0x1<<7))|(0x3<<0), S3C_USBOTG_PHYCLK);
	__raw_writel((__raw_readl(S3C_USBOTG_RSTCON))
		|(0x1<<4)|(0x1<<3), S3C_USBOTG_RSTCON);
	__raw_writel(__raw_readl(S3C_USBOTG_RSTCON)
		&~(0x1<<4)&~(0x1<<3), S3C_USBOTG_RSTCON);
}
EXPORT_SYMBOL(usb_host_phy_init);

void usb_host_phy_off(void)
{
	__raw_writel(__raw_readl(S3C_USBOTG_PHYPWR)
		|(0x1<<7)|(0x1<<6), S3C_USBOTG_PHYPWR);
	__raw_writel(__raw_readl(S5P_USB_PHY_CONTROL)
		&~(1<<1), S5P_USB_PHY_CONTROL);
}
EXPORT_SYMBOL(usb_host_phy_off);

/* For OTG host mode */
#ifdef CONFIG_USB_S3C_OTG_HOST

/* Initializes OTG Phy */
void otg_host_phy_init(void) 
{
	__raw_writel(__raw_readl(S5P_USB_PHY_CONTROL)
		|(0x1<<0), S5P_USB_PHY_CONTROL); /*USB PHY0 Enable */
	__raw_writel((__raw_readl(S3C_USBOTG_PHYPWR)
		&~(0x3<<3)&~(0x1<<0))|(0x1<<5), S3C_USBOTG_PHYPWR);
	__raw_writel((__raw_readl(S3C_USBOTG_PHYCLK)
		&~(0x1<<4))|(0x7<<0), S3C_USBOTG_PHYCLK);

	__raw_writel((__raw_readl(S3C_USBOTG_RSTCON)
		&~(0x3<<1))|(0x1<<0), S3C_USBOTG_RSTCON);
	udelay(10);
	__raw_writel((__raw_readl(S3C_USBOTG_RSTCON)
		&~(0x7<<0)), S3C_USBOTG_RSTCON);
	udelay(10);

	__raw_writel((__raw_readl(S3C_UDC_OTG_GUSBCFG)
		|(0x3<<8)), S3C_UDC_OTG_GUSBCFG);

//	smb136_set_otg_mode(1);

	printk("otg_host_phy_int : USBPHYCTL=0x%x,PHYPWR=0x%x,PHYCLK=0x%x,USBCFG=0x%x\n", 
		readl(S5P_USB_PHY_CONTROL), 
		readl(S3C_USBOTG_PHYPWR),
		readl(S3C_USBOTG_PHYCLK), 
		readl(S3C_UDC_OTG_GUSBCFG)
		);
}
EXPORT_SYMBOL(otg_host_phy_init);
#endif

#endif

#if defined(CONFIG_KEYPAD_S3C) || defined(CONFIG_KEYPAD_S3C_MODULE)
#if defined(CONFIG_KEYPAD_S3C_MSM)
void s3c_setup_keypad_cfg_gpio(void)
{
	unsigned int gpio;
	unsigned int end;

	/* gpio setting for KP_COL0 */
	s3c_gpio_cfgpin(S5PV210_GPJ1(5), S3C_GPIO_SFN(3));
	s3c_gpio_setpull(S5PV210_GPJ1(5), S3C_GPIO_PULL_NONE);

	/* gpio setting for KP_COL1 ~ KP_COL7 and KP_ROW0 */
	end = S5PV210_GPJ2(8);
	for (gpio = S5PV210_GPJ2(0); gpio < end; gpio++) {
		s3c_gpio_cfgpin(gpio, S3C_GPIO_SFN(3));
		s3c_gpio_setpull(gpio, S3C_GPIO_PULL_NONE);
	}

	/* gpio setting for KP_ROW1 ~ KP_ROW8 */
	end = S5PV210_GPJ3(8);
	for (gpio = S5PV210_GPJ3(0); gpio < end; gpio++) {
		s3c_gpio_cfgpin(gpio, S3C_GPIO_SFN(3));
		s3c_gpio_setpull(gpio, S3C_GPIO_PULL_NONE);
	}

	/* gpio setting for KP_ROW9 ~ KP_ROW13 */
	end = S5PV210_GPJ4(5);
	for (gpio = S5PV210_GPJ4(0); gpio < end; gpio++) {
		s3c_gpio_cfgpin(gpio, S3C_GPIO_SFN(3));
		s3c_gpio_setpull(gpio, S3C_GPIO_PULL_NONE);
	}
}
#else
void s3c_setup_keypad_cfg_gpio(int rows, int columns)
{
	unsigned int gpio;
	unsigned int end;

	end = S5PV210_GPH3(rows);

	/* Set all the necessary GPH2 pins to special-function 0 */
	for (gpio = S5PV210_GPH3(0); gpio < end; gpio++) {
		s3c_gpio_cfgpin(gpio, S3C_GPIO_SFN(3));
		s3c_gpio_setpull(gpio, S3C_GPIO_PULL_UP);
	}

	end = S5PV210_GPH2(columns);

	/* Set all the necessary GPK pins to special-function 0 */
	for (gpio = S5PV210_GPH2(0); gpio < end; gpio++) {
		s3c_gpio_cfgpin(gpio, S3C_GPIO_SFN(3));
		s3c_gpio_setpull(gpio, S3C_GPIO_PULL_NONE);
	}
}
#endif /* if defined(CONFIG_KEYPAD_S3C_MSM)*/
EXPORT_SYMBOL(s3c_setup_keypad_cfg_gpio);
#endif

/*****************************/
/* add uart gpio config - start >> */
void s3c_setup_uart_cfg_gpio(unsigned char port)
{
	switch(port)
	{
	case 0:
		s3c_gpio_cfgpin(GPIO_BT_RXD, S3C_GPIO_SFN(GPIO_BT_RXD_AF));
		s3c_gpio_setpull(GPIO_BT_RXD, S3C_GPIO_PULL_NONE);
		s3c_gpio_cfgpin(GPIO_BT_TXD, S3C_GPIO_SFN(GPIO_BT_TXD_AF));
		s3c_gpio_setpull(GPIO_BT_TXD, S3C_GPIO_PULL_NONE);
		s3c_gpio_cfgpin(GPIO_BT_CTS, S3C_GPIO_SFN(GPIO_BT_CTS_AF));
		s3c_gpio_setpull(GPIO_BT_CTS, S3C_GPIO_PULL_NONE);
		s3c_gpio_cfgpin(GPIO_BT_RTS, S3C_GPIO_SFN(GPIO_BT_RTS_AF));
		s3c_gpio_setpull(GPIO_BT_RTS, S3C_GPIO_PULL_NONE);
#if 0		
		s3c_gpio_slp_cfgpin(GPIO_BT_RXD, S3C_GPIO_SLP_PREV);
		s3c_gpio_slp_setpull_updown(GPIO_BT_RXD, S3C_GPIO_PULL_NONE);
		s3c_gpio_slp_cfgpin(GPIO_BT_TXD, S3C_GPIO_SLP_PREV);
		s3c_gpio_slp_setpull_updown(GPIO_BT_TXD, S3C_GPIO_PULL_NONE);
		s3c_gpio_slp_cfgpin(GPIO_BT_CTS, S3C_GPIO_SLP_PREV);
		s3c_gpio_slp_setpull_updown(GPIO_BT_CTS, S3C_GPIO_PULL_NONE);
		s3c_gpio_slp_cfgpin(GPIO_BT_RTS, S3C_GPIO_SLP_PREV);
		s3c_gpio_slp_setpull_updown(GPIO_BT_RTS, S3C_GPIO_PULL_NONE);
#endif		
		break;
	case 1:
		s3c_gpio_cfgpin(GPIO_GPS_RXD, S3C_GPIO_SFN(GPIO_GPS_RXD_AF));
		s3c_gpio_setpull(GPIO_GPS_RXD, S3C_GPIO_PULL_UP);
		s3c_gpio_cfgpin(GPIO_GPS_TXD, S3C_GPIO_SFN(GPIO_GPS_TXD_AF));
		s3c_gpio_setpull(GPIO_GPS_TXD, S3C_GPIO_PULL_NONE);
		s3c_gpio_cfgpin(GPIO_GPS_CTS, S3C_GPIO_SFN(GPIO_GPS_CTS_AF));
		s3c_gpio_setpull(GPIO_GPS_CTS, S3C_GPIO_PULL_NONE);
		s3c_gpio_cfgpin(GPIO_GPS_RTS, S3C_GPIO_SFN(GPIO_GPS_RTS_AF));
		s3c_gpio_setpull(GPIO_GPS_RTS, S3C_GPIO_PULL_NONE);
		break;
	case 2:
		s3c_gpio_cfgpin(GPIO_AP_RXD, S3C_GPIO_SFN(GPIO_AP_RXD_AF));
		s3c_gpio_setpull(GPIO_AP_RXD, S3C_GPIO_PULL_NONE);
		s3c_gpio_cfgpin(GPIO_AP_TXD, S3C_GPIO_SFN(GPIO_AP_TXD_AF));
		s3c_gpio_setpull(GPIO_AP_TXD, S3C_GPIO_PULL_NONE);
		break;
	case 3:
		s3c_gpio_cfgpin(GPIO_AP_FLM_RXD, S3C_GPIO_SFN(GPIO_AP_FLM_RXD_AF));
		s3c_gpio_setpull(GPIO_AP_FLM_RXD, S3C_GPIO_PULL_NONE);
		s3c_gpio_cfgpin(GPIO_AP_FLM_TXD, S3C_GPIO_SFN(GPIO_AP_FLM_TXD_AF));
		s3c_gpio_setpull(GPIO_AP_FLM_TXD, S3C_GPIO_PULL_NONE);
		break;
	default:
		break;
	}
}
EXPORT_SYMBOL(s3c_setup_uart_cfg_gpio);
EXPORT_SYMBOL(s3c_config_gpio_table);

/* << add uart gpio config - end */
/****************************/

static unsigned long wlan_reglock_flags = 0;
static spinlock_t wlan_reglock = SPIN_LOCK_UNLOCKED;

static unsigned int wlan_gpio_table[][4] = {	
	{GPIO_WLAN_nRST, GPIO_WLAN_nRST_AF, GPIO_LEVEL_LOW, S3C_GPIO_PULL_NONE},
	{GPIO_WLAN_HOST_WAKE, GPIO_WLAN_HOST_WAKE_AF, GPIO_LEVEL_NONE, S3C_GPIO_PULL_DOWN},
	{GPIO_WLAN_WAKE, GPIO_WLAN_WAKE_AF, GPIO_LEVEL_LOW, S3C_GPIO_PULL_NONE},
};

static unsigned int wlan_sdio_on_table[][4] = {
	{GPIO_WLAN_SDIO_CLK, GPIO_WLAN_SDIO_CLK_AF, GPIO_LEVEL_NONE, S3C_GPIO_PULL_NONE},
	{GPIO_WLAN_SDIO_CMD, GPIO_WLAN_SDIO_CMD_AF, GPIO_LEVEL_NONE, S3C_GPIO_PULL_NONE},
	{GPIO_WLAN_SDIO_D0, GPIO_WLAN_SDIO_D0_AF, GPIO_LEVEL_NONE, S3C_GPIO_PULL_NONE},
	{GPIO_WLAN_SDIO_D1, GPIO_WLAN_SDIO_D1_AF, GPIO_LEVEL_NONE, S3C_GPIO_PULL_NONE},
	{GPIO_WLAN_SDIO_D2, GPIO_WLAN_SDIO_D2_AF, GPIO_LEVEL_NONE, S3C_GPIO_PULL_NONE},
	{GPIO_WLAN_SDIO_D3, GPIO_WLAN_SDIO_D3_AF, GPIO_LEVEL_NONE, S3C_GPIO_PULL_NONE},
};

static unsigned int wlan_sdio_off_table[][4] = {
	{GPIO_WLAN_SDIO_CLK, 1, GPIO_LEVEL_LOW, S3C_GPIO_PULL_NONE},
	{GPIO_WLAN_SDIO_CMD, 0, GPIO_LEVEL_NONE, S3C_GPIO_PULL_NONE},
	{GPIO_WLAN_SDIO_D0, 0, GPIO_LEVEL_NONE, S3C_GPIO_PULL_NONE},
	{GPIO_WLAN_SDIO_D1, 0, GPIO_LEVEL_NONE, S3C_GPIO_PULL_NONE},
	{GPIO_WLAN_SDIO_D2, 0, GPIO_LEVEL_NONE, S3C_GPIO_PULL_NONE},
	{GPIO_WLAN_SDIO_D3, 0, GPIO_LEVEL_NONE, S3C_GPIO_PULL_NONE},
};

static unsigned int wlan_gpio_table_rev06[][4] = {	
	{GPIO_WLAN_nRST_REV06, GPIO_WLAN_nRST_REV06_AF, GPIO_LEVEL_LOW, S3C_GPIO_PULL_NONE},
	{GPIO_WLAN_HOST_WAKE, GPIO_WLAN_HOST_WAKE_AF, GPIO_LEVEL_NONE, S3C_GPIO_PULL_DOWN},
	{GPIO_WLAN_WAKE, GPIO_WLAN_WAKE_AF, GPIO_LEVEL_LOW, S3C_GPIO_PULL_NONE},
};

static unsigned int wlan_sdio_on_table_rev06[][4] = {
	{GPIO_WLAN_SDIO_CLK_REV06, GPIO_WLAN_SDIO_CLK_REV06_AF, GPIO_LEVEL_NONE, S3C_GPIO_PULL_NONE},
	{GPIO_WLAN_SDIO_CMD_REV06, GPIO_WLAN_SDIO_CMD_REV06_AF, GPIO_LEVEL_NONE, S3C_GPIO_PULL_NONE},
	{GPIO_WLAN_SDIO_D0_REV06, GPIO_WLAN_SDIO_D0_REV06_AF, GPIO_LEVEL_NONE, S3C_GPIO_PULL_NONE},
	{GPIO_WLAN_SDIO_D1_REV06, GPIO_WLAN_SDIO_D1_REV06_AF, GPIO_LEVEL_NONE, S3C_GPIO_PULL_NONE},
	{GPIO_WLAN_SDIO_D2_REV06, GPIO_WLAN_SDIO_D2_REV06_AF, GPIO_LEVEL_NONE, S3C_GPIO_PULL_NONE},
	{GPIO_WLAN_SDIO_D3_REV06, GPIO_WLAN_SDIO_D3_REV06_AF, GPIO_LEVEL_NONE, S3C_GPIO_PULL_NONE},
};

static unsigned int wlan_sdio_off_table_rev06[][4] = {
	{GPIO_WLAN_SDIO_CLK_REV06, 1, GPIO_LEVEL_LOW, S3C_GPIO_PULL_NONE},
	{GPIO_WLAN_SDIO_CMD_REV06, 0, GPIO_LEVEL_NONE, S3C_GPIO_PULL_NONE},
	{GPIO_WLAN_SDIO_D0_REV06, 0, GPIO_LEVEL_NONE, S3C_GPIO_PULL_NONE},
	{GPIO_WLAN_SDIO_D1_REV06, 0, GPIO_LEVEL_NONE, S3C_GPIO_PULL_NONE},
	{GPIO_WLAN_SDIO_D2_REV06, 0, GPIO_LEVEL_NONE, S3C_GPIO_PULL_NONE},
	{GPIO_WLAN_SDIO_D3_REV06, 0, GPIO_LEVEL_NONE, S3C_GPIO_PULL_NONE},
};

void wlan_setup_power(int on, int flag)
{
	unsigned int gpio_wlan_rst;
	if (HWREV >= 12)
		gpio_wlan_rst = GPIO_WLAN_nRST_REV06;
	else
		gpio_wlan_rst = GPIO_WLAN_nRST;
	
	printk(/*KERN_INFO*/ "%s %s", __func__, on ? "on" : "down");
	if (flag != 1) {	
		printk(/*KERN_DEBUG*/ " --reset(flag=%d)\n", flag);
		if (on)
			gpio_set_value(gpio_wlan_rst, GPIO_LEVEL_HIGH);
		else
			gpio_set_value(gpio_wlan_rst, GPIO_LEVEL_LOW);			
		return;
	}	
	printk(/*KERN_INFO*/ " --enter\n");
		
	if (on) {		
		if (HWREV >= 12) {
			s3c_config_gpio_alive_table(ARRAY_SIZE(wlan_gpio_table_rev06), wlan_gpio_table_rev06);
			s3c_config_gpio_alive_table(ARRAY_SIZE(wlan_sdio_on_table_rev06), wlan_sdio_on_table_rev06);
		}
		else {
			s3c_config_gpio_alive_table(ARRAY_SIZE(wlan_gpio_table), wlan_gpio_table);
			s3c_config_gpio_alive_table(ARRAY_SIZE(wlan_sdio_on_table), wlan_sdio_on_table);
		}
		
		/* PROTECT this check under spinlock.. No other thread should be touching
		 * GPIO_BT_REG_ON at this time.. If BT is operational, don't touch it. */
		spin_lock_irqsave(&wlan_reglock, wlan_reglock_flags);	
		
		gpio_set_value(GPIO_WLAN_BT_EN, GPIO_LEVEL_HIGH);
		s3c_gpio_slp_cfgpin(GPIO_WLAN_BT_EN, S3C_GPIO_SLP_OUT1);

		msleep(100);
		
		gpio_set_value(gpio_wlan_rst, GPIO_LEVEL_HIGH);
		s3c_gpio_slp_cfgpin(gpio_wlan_rst, S3C_GPIO_SLP_OUT1);
		
		printk(KERN_DEBUG "WLAN: GPIO_WLAN_BT_EN = %d, GPIO_WLAN_nRST = %d\n", 
			   gpio_get_value(GPIO_WLAN_BT_EN), gpio_get_value(gpio_wlan_rst));
		
		spin_unlock_irqrestore(&wlan_reglock, wlan_reglock_flags);
	}
	else {
		/* PROTECT this check under spinlock.. No other thread should be touching
		 * GPIO_BT_REG_ON at this time.. If BT is operational, don't touch it. */
		spin_lock_irqsave(&wlan_reglock, wlan_reglock_flags);	
		/* need delay between v_bat & reg_on for 2 cycle @ 38.4MHz */
		udelay(5);
		
		if (gpio_get_value(GPIO_BT_nRST) == 0) {
			gpio_set_value(GPIO_WLAN_BT_EN, GPIO_LEVEL_LOW);	
			s3c_gpio_slp_cfgpin(GPIO_WLAN_BT_EN, S3C_GPIO_SLP_OUT0);
		}
		
		gpio_set_value(gpio_wlan_rst, GPIO_LEVEL_LOW);
		s3c_gpio_slp_cfgpin(gpio_wlan_rst, S3C_GPIO_SLP_OUT0);
		
		printk(KERN_DEBUG "WLAN: GPIO_WLAN_BT_EN = %d, GPIO_WLAN_nRST = %d\n", 
			   gpio_get_value(GPIO_WLAN_BT_EN), gpio_get_value(gpio_wlan_rst));
		
		spin_unlock_irqrestore(&wlan_reglock, wlan_reglock_flags);

		
		if (HWREV >= 12)
			s3c_config_gpio_alive_table(ARRAY_SIZE(wlan_sdio_off_table_rev06), wlan_sdio_off_table_rev06);
		else
			s3c_config_gpio_alive_table(ARRAY_SIZE(wlan_sdio_off_table), wlan_sdio_off_table);	
	}
	msleep(100);
	
	/* mmc_rescan*/
	
	if (HWREV >= 12)
		sdhci_s3c_force_presence_change(&s3c_device_hsmmc3);
	else
		sdhci_s3c_force_presence_change(&s3c_device_hsmmc1);
}
EXPORT_SYMBOL(wlan_setup_power);

#if defined(CONFIG_TARGET_LOCALE_LTN)
MACHINE_START(SMDKC110, "GT-P1000L")
#else
MACHINE_START(SMDKC110, "GT-P1000")
#endif		
	/* Maintainer: Kukjin Kim <kgene.kim@samsung.com> */
	.phys_io	= S3C_PA_UART & 0xfff00000,
	.io_pg_offst	= (((u32)S3C_VA_UART) >> 18) & 0xfffc,
	.boot_params	= S5P_PA_SDRAM + 0x100,
	.fixup		= smdkc110_fixup,
	.init_irq	= s5pv210_init_irq,
	.map_io		= smdkc110_map_io,
	.init_machine	= smdkc110_machine_init,
	.timer		= &s5p_systimer,
MACHINE_END
