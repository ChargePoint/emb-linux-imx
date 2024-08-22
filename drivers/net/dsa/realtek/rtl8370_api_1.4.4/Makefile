# SPDX-License-Identifier: GPL-2.0-only
#
# Makefile for the RTL8370MB Switch device drivers.
#

obj-$(CONFIG_NET_DSA_REALTEK_RTL8370MB) += rtl8370_api.o module_info.o

ccflags-y += -I$(srctree)/$(src) -I$(srctree)/$(src)/dal/rtl8367c

rtl8370_api-objs := vlan.o \
		chip.o \
		l2.o \
		module_info.o \
		port.o \
		rate.o \
		rtk_switch.o \
		cpu.o \
		stat.o \
		interrupt.o \
		led.o \
		dal/rtl8367c/rtl8367c_asicdrv_vlan.o \
		dal/rtl8367c/rtl8367c_asicdrv_unknownMulticast.o \
		dal/rtl8367c/rtl8367c_asicdrv_misc.o \
		dal/rtl8367c/rtl8367c_asicdrv.o \
		dal/rtl8367c/dal_rtl8367c_cpu.o \
		dal/rtl8367c/dal_rtl8367c_rate.o \
		dal/rtl8367c/dal_rtl8367c_port.o \
		dal/rtl8367c/rtl8367c_asicdrv_rma.o \
		dal/rtl8367c/dal_rtl8367c_mapper.o \
		dal/rtl8367c/rtl8367c_asicdrv_portIsolation.o \
		dal/rtl8367c/dal_rtl8367c_l2.o \
		dal/rtl8367c/rtl8367c_asicdrv_dot1x.o \
		dal/rtl8367c/dal_rtl8367c_interrupt.o \
		dal/rtl8367c/rtl8367c_asicdrv_green.o \
		dal/rtl8367c/rtl8367c_asicdrv_lut.o \
		dal/rtl8367c/rtl8367c_asicdrv_port.o \
		dal/rtl8367c/rtl8367c_asicdrv_cputag.o \
		dal/rtl8367c/dal_rtl8367c_led.o \
		dal/rtl8367c/dal_rtl8367c_stat.o \
		dal/rtl8367c/dal_rtl8367c_vlan.o \
		dal/rtl8367c/rtl8367c_asicdrv_led.o \
		dal/rtl8367c/rtl8367c_asicdrv_mib.o \
		dal/rtl8367c/rtl8367c_asicdrv_interrupt.o \
		dal/rtl8367c/rtl8367c_asicdrv_scheduling.o \
		dal/rtl8367c/rtl8367c_asicdrv_meter.o \
		dal/rtl8367c/rtl8367c_asicdrv_mirror.o \
		dal/rtl8367c/rtl8367c_asicdrv_phy.o \
		dal/rtl8367c/rtl8367c_asicdrv_inbwctrl.o \
		dal/rtl8367c/dal_rtl8367c_switch.o \
		dal/dal_mgmt.o \
# JimmyLiu
