# 
# Makefile for TDMB Device Drivers.
#

EXTRA_CFLAGS += -Idrivers/media/tdmb
ifeq ($(CONFIG_TDMB_GDM7024),y)
EXTRA_CFLAGS += -Idrivers/media/tdmb/gdm7024
endif
ifeq ($(CONFIG_TDMB_T3700),y)
EXTRA_CFLAGS += -Idrivers/media/tdmb/t3700
endif
# EXTRA_CFLAGS += -v

obj-$(CONFIG_TDMB) += tdmb_module.o

tdmb_module-y := tdmb.o tdmb_drv.o
ifeq ($(CONFIG_TDMB_T3700),y)
tdmb_module-y += t3700/INC_DAB_DLS.o t3700/INC_FICDEC.o t3700/INC_INTERFACE.o t3700/INC_PROCESS.o t3700/INC_RF_CTRL500.o
endif
ifeq ($(CONFIG_TDMB_GDM7024),y)
tdmb_module-y += gdm7024/gdm_hal.o gdm7024/gdm_api_common.o gdm7024/gdm_api_dab.o gdm7024/gdm_drv_common.o gdm7024/gdm_drv_dab.o gdm7024/fic_decode.o tdmb_drv_gct.o
endif
