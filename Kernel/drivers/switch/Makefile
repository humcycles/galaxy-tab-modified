
ifeq ($(X_BUILD_LOCALE),P1_SPR)
	EXTRA_CFLAGS		+= -DCONFIG_TARGET_LOCALE_SPR
endif

# Switch Class Driver
obj-$(CONFIG_SWITCH)		+= switch_class.o
obj-$(CONFIG_SWITCH_GPIO)	+= switch_gpio.o
obj-$(CONFIG_MACH_S5PC110_P1) += fsa9480_i2c.o user_switch.o
