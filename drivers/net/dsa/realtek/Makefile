#SPDX-License-Identifier: GPL-2.0
obj-$(CONFIG_NET_DSA_REALTEK_MDIO) 	+= realtek-mdio.o
obj-$(CONFIG_NET_DSA_REALTEK_SMI) 	+= realtek-smi.o
obj-$(CONFIG_NET_DSA_REALTEK_RTL8366RB) += rtl8366.o
rtl8366-objs 				:= rtl8366-core.o rtl8366rb.o
obj-$(CONFIG_NET_DSA_REALTEK_RTL8365MB) += rtl8365mb.o
ccflags-y += -I$(src)/rtl8370_api -I$(src)/rtl8370_api/dal -I$(src)/rtl8370_api/dal/rtl8367c
obj-$(CONFIG_NET_DSA_REALTEK_RTL8370MB) += rtl8370mb.o
obj-$(CONFIG_NET_DSA_REALTEK_RTL_API)   += rtl8370_api/
#rtl8370mb-objs				:= rtl8370_api/acl.o \
				rtl8370_api/chip.o \
				rtl8370_api/cpu.o \
				rtl8370_api/dot1x.o \
				rtl8370_api/eee.o \
				rtl8370_api/gpio.o \
				rtl8370_api/i2c.o \
				rtl8370_api/igmp.o \
				rtl8370_api/interrupt.o \
				rtl8370_api/l2.o \
				rtl8370_api/leaky.o \
				rtl8370_api/led.o \
				rtl8370_api/mirror.o \
				rtl8370_api/oam.o \
				rtl8370_api/port.o \
				rtl8370_api/ptp.o \
				rtl8370_api/qos.o \
				rtl8370_api/rate.o \
				rtl8370_api/rldp.o \
				rtl8370_api/rtk_switch.o \
				rtl8370_api/stat.o \
				rtl8370_api/storm.o \
				rtl8370_api/svlan.o \
				rtl8370_api/trap.o \
				rtl8370_api/trunk.o \
				rtl8370_api/vlan.o \
				rtl8370_api/dal/dal_mgmt.o \
				rtl8370_api/dal/rtl8367c/dal_rtl8367c_acl.o \
				rtl8370_api/dal/rtl8367c/dal_rtl8367c_cpu.o \
				rtl8370_api/dal/rtl8367c/dal_rtl8367c_dot1x.o \
				rtl8370_api/dal/rtl8367c/dal_rtl8367c_eee.o \
				rtl8370_api/dal/rtl8367c/dal_rtl8367c_gpio.o \
				rtl8370_api/dal/rtl8367c/dal_rtl8367c_i2c.o \
				rtl8370_api/dal/rtl8367c/dal_rtl8367c_igmp.o \
				rtl8370_api/dal/rtl8367c/dal_rtl8367c_interrupt.o \
				rtl8370_api/dal/rtl8367c/dal_rtl8367c_l2.o \
				rtl8370_api/dal/rtl8367c/dal_rtl8367c_leaky.o \
				rtl8370_api/dal/rtl8367c/dal_rtl8367c_led.o \
				rtl8370_api/dal/rtl8367c/dal_rtl8367c_mapper.o \
				rtl8370_api/dal/rtl8367c/dal_rtl8367c_mirror.o \
				rtl8370_api/dal/rtl8367c/dal_rtl8367c_oam.o \
				rtl8370_api/dal/rtl8367c/dal_rtl8367c_port.o \
				rtl8370_api/dal/rtl8367c/dal_rtl8367c_ptp.o \
				rtl8370_api/dal/rtl8367c/dal_rtl8367c_qos.o \
				rtl8370_api/dal/rtl8367c/dal_rtl8367c_rate.o \
				rtl8370_api/dal/rtl8367c/dal_rtl8367c_rldp.o \
				rtl8370_api/dal/rtl8367c/dal_rtl8367c_stat.o \
				rtl8370_api/dal/rtl8367c/dal_rtl8367c_storm.o \
				rtl8370_api/dal/rtl8367c/dal_rtl8367c_svlan.o \
				rtl8370_api/dal/rtl8367c/dal_rtl8367c_switch.o \
