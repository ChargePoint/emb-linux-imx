# SPDX-License-Identifier: GPL-2.0
obj-$(CONFIG_NET_DSA_REALTEK_MDIO) 	+= realtek-mdio.o
obj-$(CONFIG_NET_DSA_REALTEK_SMI) 	+= realtek-smi.o
obj-$(CONFIG_NET_DSA_REALTEK_RTL8366RB) += rtl8366.o
rtl8366-objs 				:= rtl8366-core.o rtl8366rb.o
obj-$(CONFIG_NET_DSA_REALTEK_RTL8365MB) += rtl8365mb.o
ccflags-y += -I$(src)/rtl8370_api_1.4.4 -I$(src)/rtl8370_api_1.4.4/dal -I$(src)/rtl8370_api_1.4.4/dal/rtl8367c
obj-$(CONFIG_NET_DSA_REALTEK_RTL8370MB) += rtl8370mb.o rtl8370_common.o
obj-$(CONFIG_NET_DSA_REALTEK_RTL_API) 	+= rtl8370_api_1.4.4/
