# SPDX-License-Identifier: GPL-2.0
#
# Makefile for DWC_otg Highspeed USB controller driver
#

BUS_INTERFACE = -DLM_INTERFACE
#EXTRA_CFLAGS    += -DDEBUG
EXTRA_CFLAGS	+= -Dlinux -DDWC_HS_ELECT_TST
#EXTRA_CFLAGS	+= -DDWC_EN_ISOC
#EXTRA_CFLAGS   	+= -I$(PWD)/dwc_common_port
#EXTRA_CFLAGS   	+= -I$(PORTLIB)
EXTRA_CFLAGS   	+= -DDWC_LINUX
EXTRA_CFLAGS   	+= $(CFI)
EXTRA_CFLAGS	+= $(BUS_INTERFACE)
#EXTRA_CFLAGS	+= -DDWC_DEV_SRPCAP


dwc_otg-objs	:= dwc_otg_driver.o dwc_otg_attr.o
dwc_otg-objs	+= dwc_otg_cil.o dwc_otg_cil_intr.o
dwc_otg-objs	+= dwc_otg_pcd_linux.o dwc_otg_pcd.o dwc_otg_pcd_intr.o
ifeq ($(CONFIG_USB),y)
dwc_otg-objs	+= dwc_otg_hcd.o dwc_otg_hcd_linux.o dwc_otg_hcd_intr.o dwc_otg_hcd_queue.o dwc_otg_hcd_ddma.o
endif
dwc_otg-objs	+= dwc_otg_adp.o
ifneq ($(CFI),)
dwc_otg-objs	+= dwc_otg_cfi.o
endif
dwc_otg-objs	+= common_port/dwc_common_linux.o

#objs relative to RK platform
ifeq ($(CONFIG_CPU_RK3188),y)
dwc_otg-objs 	+= usbdev_rk30.o
endif
ifeq ($(CONFIG_CPU_RK3288),y)
dwc_otg-objs 	+= usbdev_rk32.o
endif
ifeq ($(CONFIG_CPU_RK3036),y)
dwc_otg-objs 	+= usbdev_rk3036.o
endif
ifeq ($(CONFIG_CPU_RK312X),y)
dwc_otg-objs 	+= usbdev_rk3126.o
endif
ifeq ($(CONFIG_CPU_RK322X),y)
dwc_otg-objs	+= usbdev_rk322x.o
endif
ifeq ($(CONFIG_CPU_RV110X),y)
dwc_otg-objs	+= usbdev_rv1108.o
endif
ifeq ($(CONFIG_ARM64),y)
dwc_otg-objs	+= usbdev_rk3368.o
dwc_otg-objs	+= usbdev_rk322xh.o
endif
dwc_otg-objs	+= usbdev_bc.o
obj-$(CONFIG_DWC_OTG_310) := dwc_otg.o
