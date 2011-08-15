# arch/arm/mach-s5pv210/Makefile
#
# Copyright (c) 2010 Samsung Electronics Co., Ltd.
# 		http://www.samsung.com/
#
# Licensed under GPLv2

obj-y				:=
obj-m				:=
obj-n				:=
obj-				:=

# Core support for S5PV210 system

obj-$(CONFIG_CPU_S5PV210)	+= cpu.o init.o clock.o gpio.o dma.o pd.o irq-eint-group.o irq-eint.o

# machine support

obj-$(CONFIG_MACH_SMDKV210)	+= mach-smdkv210.o
# obj-$(CONFIG_MACH_SMDKC110)	+= mach-smdkc110.o

ifeq ($(CONFIG_MACH_S5PC110_P1),y)
ifeq ($(CONFIG_TARGET_LOCALE_LTN),y)
obj-$(CONFIG_TARGET_LOCALE_LTN)	+= mach-p1_ltn.o p1-rfkill.o mach-p1-gps.o
else
obj-$(CONFIG_TARGET_LOCALE_EUR)	+= mach-p1.o p1-rfkill.o mach-p1-gps.o
endif
obj-$(CONFIG_TARGET_LOCALE_HKTW) += mach-p1.o p1-rfkill.o mach-p1-gps.o
obj-$(CONFIG_TARGET_LOCALE_HKTW_FET) += mach-p1.o p1-rfkill.o mach-p1-gps.o
obj-$(CONFIG_TARGET_LOCALE_KOR)	+= mach-p1_kor.o p1-rfkill_kor.o mach-p1-gps_kor.o
obj-$(CONFIG_TARGET_LOCALE_VZW)	+= mach-p1_vzw.o p1-rfkill.o mach-p1-gps.o
obj-$(CONFIG_TARGET_LOCALE_USAGSM) += mach-p1.o p1-rfkill.o mach-p1-gps.o
endif

obj-$(CONFIG_S5PV210_PM)	+= pm.o
obj-$(CONFIG_PM_PWR_GATING)	+= powergating.o
obj-$(CONFIG_S5PV210_SETUP_SDHCI)	+= setup-sdhci.o

# Device setup - ADCTS
obj-$(CONFIG_S5PV210_ADCTS)		+= s5pc11x-adcts.o

obj-$(CONFIG_S5PV210_SETUP_FB)		+= setup-fb.o
obj-$(CONFIG_S5PV210_SETUP_FIMC0)	+= setup-fimc0.o
obj-$(CONFIG_S5PV210_SETUP_FIMC1)	+= setup-fimc1.o
obj-$(CONFIG_S5PV210_SETUP_FIMC2)	+= setup-fimc2.o
obj-$(CONFIG_S5PV210_SETUP_CSIS)	+= setup-csis.o
ifeq ($(CONFIG_SEC_HEADSET),y)
obj-$(CONFIG_TARGET_LOCALE_EUR)	+= sec_jack.o
obj-$(CONFIG_TARGET_LOCALE_HKTW) += sec_jack.o
obj-$(CONFIG_TARGET_LOCALE_HKTW_FET) += sec_jack.o
obj-$(CONFIG_TARGET_LOCALE_KOR)	+= sec_jack_kor.o
obj-$(CONFIG_TARGET_LOCALE_VZW)	+= sec_jack.o
obj-$(CONFIG_TARGET_LOCALE_USAGSM)	+= sec_jack.o
endif

obj-$(CONFIG_CPU_IDLE)		+= cpuidle.o idle2.o
