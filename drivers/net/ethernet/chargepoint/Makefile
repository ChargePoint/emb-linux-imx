# SPDX-License-Identifier: GPL-2.0
#
# Makefile for the ChargePoint ethernet over serial network device drivers.
#

obj-$(CONFIG_ETH_SERIAL) += qca_7k_common.o
obj-$(CONFIG_ETH_SPI) += qcaspi.o
qcaspi-objs := qca_7k.o qca_debug.o qca_spi.o
obj-$(CONFIG_ETH_UART) += qcauart.o
qcauart-objs := qca_uart.o

obj-y += emac/

obj-$(CONFIG_RMNET) += rmnet/
