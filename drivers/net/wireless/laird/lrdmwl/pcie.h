/*
 * Copyright (C) 2018-2020 Laird Connectivity
 *
 * This software file (the "File") is distributed by Marvell International
 * Ltd. under the terms of the GNU General Public License Version 2, June 1991
 * (the "License").  You may use, redistribute and/or modify this File in
 * accordance with the terms and conditions of the License, a copy of which
 * is available by writing to the Free Software Foundation, Inc.
 *
 * THE FILE IS DISTRIBUTED AS-IS, WITHOUT WARRANTY OF ANY KIND, AND THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE
 * ARE EXPRESSLY DISCLAIMED.  The License provides additional details about
 * this warranty disclaimer.
 */

#ifndef _PCIE_H_
#define _PCIE_H_

#include <linux/pci.h>
#include <linux/interrupt.h>

#include "main.h"

#define MAC_REG_ADDR(offset)      (offset + 0xA000)
#define MAC_REG_ADDR_PCI(offset)  ((card->iobase1 + 0xA000) + offset)

#define MCU_CCA_CNT               MAC_REG_ADDR(0x06A0)
#define MCU_TXPE_CNT              MAC_REG_ADDR(0x06A4)
#define MCU_LAST_READ             MAC_REG_ADDR(0x06A8)

#define MAC_REG_TCQ0_WRPTR        MAC_REG_ADDR(0x0040)
#define MAC_REG_TCQ0_RDPTR        MAC_REG_ADDR(0x0044)

#define MAC_REG_CW0_MIN           MAC_REG_ADDR(0x00a0)
#define MAC_REG_CW0_MAX           MAC_REG_ADDR(0x00a4)
#define MAC_REG_TXOP0             MAC_REG_ADDR(0x0260)
#define MAC_REG_AIFSN0            MAC_REG_ADDR(0x0680)

#define MWIFIEX_PCIE_FLR_HAPPENS  0xFEDCBABA

struct mwl_pcie_card {
	struct mwl_priv *priv;
	struct pci_dev *pdev;
	bool surprise_removed;
	int chip_type;
	void __iomem *iobase0; /* MEM Base Address Register 0  */
	void __iomem *iobase1; /* MEM Base Address Register 1  */
	u32 next_bar_num;
	struct mwl_desc_data desc_data[SYSADPT_NUM_OF_DESC_DATA];
	/* number of descriptors owned by fw at any one time */
	int fw_desc_cnt[SYSADPT_NUM_OF_DESC_DATA];
	int intr_mode;
	spinlock_t intr_status_lock;

	struct tasklet_struct tx_task;
	struct tasklet_struct tx_done_task;
	struct tasklet_struct qe_task;

	struct work_struct fw_reset_work;
};

void mwl_pcie_tx_done(unsigned long data);
void mwl_pcie_rx_recv(unsigned long data);

#endif
