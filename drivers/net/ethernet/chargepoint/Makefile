# SPDX-License-Identifier: GPL-2.0
#
# Makefile for the ChargePoint network device drivers.
#

obj-$(CONFIG_ETH_SPI) += qca_7k_common.o qca_7k.o qca_debug.o qca_spi.o
obj-$(CONFIG_CHPT_ETH_SPI)                  := eth_spi_master.o
ccflags-$(CONFIG_CHPT_ETH_SPI_DEBUG)        += -DDEBUG
ccflags-$(CONFIG_CHPT_ETH_SPI_VERBOSE)      += -DVERBOSE_DEBUG
