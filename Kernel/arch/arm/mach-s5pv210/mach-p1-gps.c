#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <mach/gpio.h>
#include <plat/gpio-cfg.h>

extern unsigned int HWREV;
extern struct device *gps_dev;

static void __init gps_gpio_init(void)
{
#if 0  // for B04 board only
	gpio_request(GPIO_IrDA_SHUTDOWN, "IrDA_SHUTDOWN"); /* XMSMDATA_12 */
	s3c_gpio_cfgpin(GPIO_IrDA_SHUTDOWN, S3C_GPIO_OUTPUT);
	s3c_gpio_setpull(GPIO_IrDA_SHUTDOWN, S3C_GPIO_PULL_NONE);
	gpio_direction_output(GPIO_IrDA_SHUTDOWN, 1);
	gpio_export(GPIO_IrDA_SHUTDOWN, 1);
#endif

	if(HWREV >= 12)	{
		gpio_request(GPIO_GPS_nRST_REV06, "GPS_nRST");	/* XMMC3CLK */
		s3c_gpio_setpull(GPIO_GPS_nRST_REV06, S3C_GPIO_PULL_NONE);
		s3c_gpio_cfgpin(GPIO_GPS_nRST_REV06, S3C_GPIO_OUTPUT);
		gpio_direction_output(GPIO_GPS_nRST_REV06, 1);
		
		gpio_request(GPIO_GPS_PWR_EN_REV06, "GPS_PWR_EN");	/* XMMC3CLK */
		s3c_gpio_setpull(GPIO_GPS_PWR_EN_REV06, S3C_GPIO_PULL_NONE);
		s3c_gpio_cfgpin(GPIO_GPS_PWR_EN_REV06, S3C_GPIO_OUTPUT);
		gpio_direction_output(GPIO_GPS_PWR_EN_REV06, 0);
		
		s3c_gpio_setpull(GPIO_GPS_RXD, S3C_GPIO_PULL_UP);
		gpio_export(GPIO_GPS_nRST_REV06, 1);
		gpio_export(GPIO_GPS_PWR_EN_REV06, 1);

		BUG_ON(!gps_dev);
		gpio_export_link(gps_dev, "GPS_nRST", GPIO_GPS_nRST_REV06);
		gpio_export_link(gps_dev, "GPS_PWR_EN", GPIO_GPS_PWR_EN_REV06);
	}
	else{
		gpio_request(GPIO_GPS_nRST, "GPS_nRST");	/* XMMC3CLK */
		s3c_gpio_setpull(GPIO_GPS_nRST, S3C_GPIO_PULL_NONE);
		s3c_gpio_cfgpin(GPIO_GPS_nRST, S3C_GPIO_OUTPUT);
		gpio_direction_output(GPIO_GPS_nRST, 1);
		
		gpio_request(GPIO_GPS_PWR_EN, "GPS_PWR_EN");	/* XMMC3CLK */
		s3c_gpio_setpull(GPIO_GPS_PWR_EN, S3C_GPIO_PULL_NONE);
		s3c_gpio_cfgpin(GPIO_GPS_PWR_EN, S3C_GPIO_OUTPUT);
		gpio_direction_output(GPIO_GPS_PWR_EN, 0);
		
		s3c_gpio_setpull(GPIO_GPS_RXD, S3C_GPIO_PULL_UP);
		gpio_export(GPIO_GPS_nRST, 1);
		gpio_export(GPIO_GPS_PWR_EN, 1);

		BUG_ON(!gps_dev);
		gpio_export_link(gps_dev, "GPS_nRST", GPIO_GPS_nRST);
		gpio_export_link(gps_dev, "GPS_PWR_EN", GPIO_GPS_PWR_EN);
	}
}


arch_initcall(gps_gpio_init);
