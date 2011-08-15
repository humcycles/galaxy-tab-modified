# arch/arm/plat-s3c64xx/Makefile
#
# Copyright 2009 Simtec Electronics
#
# Licensed under GPLv2

obj-y				:=
obj-m				:=
obj-n				:= dummy.o
obj-				:=

# Objects we always build independent of SoC choice

obj-y				+= init.o
ifndef CONFIG_S5P_HIGH_RES_TIMERS
obj-y				+= time.o
endif
obj-y				+= clock.o
obj-y				+= pwm-clock.o
obj-y				+= gpio.o
obj-y				+= gpio-config.o

obj-$(CONFIG_SAMSUNG_GPIOLIB_4BIT)	+= gpiolib.o
obj-$(CONFIG_SAMSUNG_CLKSRC)	+= clock-clksrc.o

obj-$(CONFIG_SAMSUNG_IRQ_UART)	+= irq-uart.o
obj-$(CONFIG_SAMSUNG_IRQ_VIC_TIMER) += irq-vic-timer.o

# ADC

obj-$(CONFIG_S3C_ADC)	+= adc.o

# devices

obj-$(CONFIG_S3C_DEV_HSMMC)	+= dev-hsmmc.o
obj-$(CONFIG_S3C_DEV_HSMMC1)	+= dev-hsmmc1.o
obj-$(CONFIG_S3C_DEV_HSMMC2)	+= dev-hsmmc2.o
obj-$(CONFIG_S3C_DEV_HSMMC3)	+= dev-hsmmc3.o
obj-y				+= dev-i2c0.o
obj-$(CONFIG_S3C_DEV_I2C1)	+= dev-i2c1.o
obj-$(CONFIG_S3C_DEV_I2C2)      += dev-i2c2.o
obj-$(CONFIG_S3C_DEV_FB)	+= dev-fb.o
obj-y				+= dev-uart.o
obj-$(CONFIG_S3C_DEV_USB_HOST)	+= dev-usb.o
obj-$(CONFIG_S3C_DEV_USB_HSOTG)	+= dev-usb-hsotg.o
obj-$(CONFIG_S3C_DEV_NAND)	+= dev-nand.o
obj-$(CONFIG_BLK_DEV_IDE_S3C)   += dev-ide.o
ifeq ($(CONFIG_TARGET_LOCALE_KOR),y)
obj-$(CONFIG_SPI_S5PV210)        += dev-spi.o
endif

# DMA support

obj-$(CONFIG_S3C_DMA)		+= dma.o

# PM support

obj-$(CONFIG_PM)		+= pm.o
obj-$(CONFIG_PM)		+= pm-gpio.o
obj-$(CONFIG_SAMSUNG_PM_CHECK)	+= pm-check.o

ifeq ($(CONFIG_REGULATOR_MAX8998),y)
obj-$(CONFIG_TARGET_LOCALE_EUR) += max8998_consumer.o
obj-$(CONFIG_TARGET_LOCALE_HKTW) += max8998_consumer.o
obj-$(CONFIG_TARGET_LOCALE_HKTW_FET) += max8998_consumer.o
obj-$(CONFIG_TARGET_LOCALE_KOR) += max8998_consumer_kor.o
obj-$(CONFIG_TARGET_LOCALE_VZW) += max8998_consumer.o
obj-$(CONFIG_TARGET_LOCALE_USAGSM) += max8998_consumer.o
endif
obj-$(CONFIG_CPU_FREQ)          += s5pc11x-dvfs.o

# PWM support

obj-$(CONFIG_HAVE_PWM)		+= pwm.o
