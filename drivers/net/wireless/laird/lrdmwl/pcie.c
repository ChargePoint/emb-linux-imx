/*
 * Copyright (C) 2018-2020 Laird Connectivity
 *
 * This software file (the "File") is distributed by Laird, PLC.
 * under the terms of the GNU General Public License Version 2, June 1991
 * (the "License").  You may use, redistribute and/or modify this File in
 * accordance with the terms and conditions of the License, a copy of which
 * is available by writing to the Free Software Foundation, Inc.
 *
 * THE FILE IS DISTRIBUTED AS-IS, WITHOUT WARRANTY OF ANY KIND, AND THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE
 * ARE EXPRESSLY DISCLAIMED.  The License provides additional details about
 * this warranty disclaimer.
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/pci.h>
#include <linux/interrupt.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include "sysadpt.h"
#include "dev.h"
#include "fwcmd.h"
#include "hostcmd.h"
#include "main.h"
#include "pcie.h"
#include "isr.h"
#include "tx.h"
#include "rx.h"

/*Note PCIEDRV_VERSION define was taken from SDIO since there
 *     was no PCI version defined in reference driver */
#define MWL_PCIEDRV_VERSION  "10.3.0.16-20160105"
#define LRD_PCIE_VERSION     LRD_BLD_VERSION "-" MWL_PCIEDRV_VERSION
#define LRD_PCIE_DESC        "Laird Connectivity 60 Series Wireless PCIE Network Driver"

#define INTF_HEADER_LEN      0
#define INTF_HEADER_LEN_MFG  4


/* PCIe interrupt mode = MSI (default) */
int pcie_intr_mode = 1;

static struct mwl_chip_info mwl_chip_tbl[] = {
	[MWL8864] = {
		.part_name	= "88W8864",
		.fw_image	= MWL_FW_ROOT"/88W8864_pcie.bin",
		.antenna_tx	= ANTENNA_TX_4_AUTO,
		.antenna_rx	= ANTENNA_RX_4_AUTO,
	},
	[MWL8897] = {
		.part_name	= "88W8897",
		.fw_image	= MWL_FW_ROOT"/88W8897_pcie.bin",
		.antenna_tx	= ANTENNA_TX_2,
		.antenna_rx	= ANTENNA_RX_2,
	},
	[MWL8964] = {
		.part_name	= "88W8964",
		.fw_image	= MWL_FW_ROOT"/88W8964_pcie.bin",
		.antenna_tx	= ANTENNA_TX_4_AUTO,
		.antenna_rx	= ANTENNA_RX_4_AUTO,
	},
	[MWL8997] = {
		.part_name	= "88W8997",
		.fw_image	= MWL_FW_ROOT"/88W8997_pcie.bin",
		.mfg_image	= MWL_FW_ROOT"/88W8997_pcie_mfg.bin",
		.antenna_tx	= ANTENNA_TX_2,
		.antenna_rx	= ANTENNA_RX_2,
	},
};

static void mwl_pcie_tx_flush_amsdu(unsigned long data);
static int mwl_tx_ring_alloc(struct mwl_priv *priv);
static int mwl_tx_ring_init(struct mwl_priv *priv);
static void mwl_tx_ring_cleanup(struct mwl_priv *priv);
static void mwl_tx_ring_free(struct mwl_priv *priv);
static void mwl_set_bit(struct mwl_priv *priv, int bit_num, volatile void * addr);
static void mwl_clear_bit(struct mwl_priv *priv, int bit_num, volatile void * addr);
static int mwl_pcie_restart_handler(struct mwl_priv *priv);
static void mwl_pcie_up_dev(struct mwl_priv *priv);
static void mwl_pcie_down_dev(struct mwl_priv *priv);
static bool mwl_pcie_check_fw_status(struct mwl_priv *priv);

#define MAX_WAIT_CMD_RESPONSE_ITERATIONS         (10*4000)

static irqreturn_t mwl_pcie_isr(int irq, void *dev_id);
static struct pci_device_id mwl_pci_id_tbl[] = {
	{ PCI_VDEVICE(MARVELL_EXT, 0x2b42), .driver_data = MWL8997, },
	{ },
};

static const struct of_device_id mwl_pcie_of_match_table[] = {
	{ .compatible = "pci1b4b,2b42" },
	{ }
};

static int mwl_pcie_probe_of(struct device *dev)
{
	if (!of_match_node(mwl_pcie_of_match_table, dev_of_node(dev))) {
		dev_err(dev, "required compatible string missing\n");
		return -EINVAL;
	}

	return 0;
}



static void lrd_pci_fw_reset_workfn(struct work_struct *work)
{
	int rc;
	struct ieee80211_hw  *hw;
	struct mwl_pcie_card *card = container_of(work,
				struct mwl_pcie_card, fw_reset_work);

	hw = pci_get_drvdata(card->pdev);

	rc = pci_reset_function(card->pdev);

	if (rc != 0)
	{
		wiphy_err(hw->wiphy, "%s: PCI reset failed! %d\n",
			  MWL_DRV_NAME, rc);
	}
}


static void mwl_pcie_reset_prepare(struct pci_dev *pdev)
{
	struct mwl_priv      *priv;
	struct ieee80211_hw  *hw;

	hw = pci_get_drvdata(pdev);

	if (!hw || !hw->priv)
		return;

	priv = hw->priv;

	wiphy_err(hw->wiphy, "%s: Prepare for reset...\n", __func__);

	if (priv->recovery_in_progress)
		mwl_shutdown_sw(priv, false);

	wiphy_err(hw->wiphy, "%s: Resetting...\n", __func__);
}


static void mwl_pcie_reset_done(struct pci_dev *pdev)
{
	struct mwl_priv      *priv;
	struct ieee80211_hw  *hw;

	hw = pci_get_drvdata(pdev);

	if (!hw || !hw->priv)
		return;

	priv = hw->priv;

	wiphy_err(hw->wiphy, "%s: Reset complete...\n", __func__);

	// Give FLR handler in firmware opportunity to run
	// Typically takes ~125ms for FW Ready signature to be cleared after FLR
	msleep(150);

	if (priv->recovery_in_progress)
		mwl_reinit_sw(priv, false);
	else
	{
		int rc;
		rc = mwl_fw_dnld_and_init(priv);
		if (rc)
		{
			wiphy_err(hw->wiphy, "%s: Initialization failed after reset!! %d\n",
				MWL_DRV_NAME, rc);
		}
	}
}

/*
 * Cleanup all software without cleaning anything related to PCIe and HW.
*/
static void mwl_free_pci_resource(struct mwl_priv *priv)
{
#if 0
	struct mwl_pcie_card *card = priv->intf;
	struct pci_dev *pdev = card->pdev;

	/* priv->pcmd_buf will be automatically freed on driver unload */
	if (priv->pcmd_buf)
		dma_free_coherent(priv->dev,
			CMD_BUF_SIZE,
			priv->pcmd_buf,
			priv->pphys_cmd_buf);

	if (pdev) {
		iounmap((volatile void __iomem *)&pdev->resource[0]);
		iounmap((volatile void __iomem *)&pdev->resource[card->next_bar_num]);
	}

#endif
}

static int mwl_alloc_pci_resource(struct mwl_priv *priv)
{
	struct mwl_pcie_card *card = priv->intf;
	struct pci_dev *pdev = card->pdev;
	void __iomem *addr;

	card->next_bar_num = 1;	/* 32-bit */
	if (pci_resource_flags(pdev, 0) & 0x04)
		card->next_bar_num = 2;	/* 64-bit */

	addr = devm_ioremap_resource(priv->dev, &pdev->resource[0]);
	if (IS_ERR(addr)) {
		wiphy_err(priv->hw->wiphy,
			  "%s: cannot reserve PCI memory region 0\n",
			  MWL_DRV_NAME);
		goto err;
	}
	card->iobase0 = addr;
	wiphy_debug(priv->hw->wiphy, "card->iobase0 = %p\n", card->iobase0);

	addr = devm_ioremap_resource(priv->dev,
				     &pdev->resource[card->next_bar_num]);
	if (IS_ERR(addr)) {
		wiphy_err(priv->hw->wiphy,
			  "%s: cannot reserve PCI memory region 1\n",
			  MWL_DRV_NAME);
		goto err;
	}
	card->iobase1 = addr;
	wiphy_debug(priv->hw->wiphy, "card->iobase1 = %p\n", card->iobase1);

	priv->pcmd_buf =
		(unsigned short *)dmam_alloc_coherent(priv->dev,
						      CMD_BUF_SIZE,
						      &priv->pphys_cmd_buf,
						      GFP_KERNEL);
	if (!priv->pcmd_buf) {
		wiphy_err(priv->hw->wiphy,
			  "%s: cannot alloc memory for command buffer\n",
			  MWL_DRV_NAME);
		goto err;
	}
	wiphy_debug(priv->hw->wiphy,
		    "priv->pcmd_buf = %p  priv->pphys_cmd_buf = %p\n",
		    priv->pcmd_buf,
		    (void *)priv->pphys_cmd_buf);
	memset(priv->pcmd_buf, 0x00, CMD_BUF_SIZE);

	return 0;

err:
	wiphy_err(priv->hw->wiphy, "pci alloc fail\n");

	return -EIO;
}

int mwl_tx_init(struct ieee80211_hw *hw)
{
	struct mwl_priv *priv = hw->priv;
	int rc;

	wiphy_info(hw->wiphy, "%s() called: ctype=%d\n", __FUNCTION__, priv->chip_type);

	if (IS_PFU_ENABLED(priv->chip_type)) {
		rc = wlan_pcie_create_txbd_ring(hw);
		if (rc) {
			wiphy_err(hw->wiphy, "wlan_pcie_create_txbd_ring() failed\n");
			return rc;
		}
	} else {
		rc = mwl_tx_ring_alloc(priv);
		if (rc) {
			wiphy_err(hw->wiphy, "allocating TX ring failed\n");
			return rc;
		}
	}

	rc = mwl_tx_ring_init(priv);

	if (rc) {
		if (!IS_PFU_ENABLED(priv->chip_type)) {

			mwl_tx_ring_free(priv);
			wiphy_err(hw->wiphy, "initializing TX ring failed\n");
			return rc;
		}
	}

	return 0;
}

/* rx */
#define MAX_NUM_RX_RING_BYTES  (SYSADPT_MAX_NUM_RX_DESC * \
				sizeof(struct mwl_rx_desc))

#define MAX_NUM_RX_HNDL_BYTES  (SYSADPT_MAX_NUM_RX_DESC * \
				sizeof(struct mwl_rx_hndl))

static int mwl_rx_ring_alloc(struct mwl_priv *priv)
{
	struct mwl_desc_data *desc;

	desc = &priv->desc_data[0];
	desc->prx_ring = (struct mwl_rx_desc *)
		dma_alloc_coherent(priv->dev,
				   MAX_NUM_RX_RING_BYTES,
				   &desc->pphys_rx_ring,
				   GFP_KERNEL);
	if (!desc->prx_ring) {
		wiphy_err(priv->hw->wiphy, "cannot alloc mem\n");
		return -ENOMEM;
	}

	memset(desc->prx_ring, 0x00, MAX_NUM_RX_RING_BYTES);

	desc->rx_hndl = kmalloc(MAX_NUM_RX_HNDL_BYTES, GFP_KERNEL);

	if (!desc->rx_hndl) {
		dma_free_coherent(priv->dev,
				  MAX_NUM_RX_RING_BYTES,
				  desc->prx_ring,
				  desc->pphys_rx_ring);
		return -ENOMEM;
	}

	memset(desc->rx_hndl, 0x00, MAX_NUM_RX_HNDL_BYTES);

	return 0;
}

static int mwl_rx_ring_init(struct mwl_priv *priv)
{
	struct mwl_pcie_card *card = priv->intf;
	struct mwl_desc_data *desc;
	int i;
	struct mwl_rx_hndl *rx_hndl;
	dma_addr_t dma;
	u32 val;

	desc = &priv->desc_data[0];

	if (desc->prx_ring) {
		desc->rx_buf_size = SYSADPT_MAX_AGGR_SIZE;

		for (i = 0; i < SYSADPT_MAX_NUM_RX_DESC; i++) {
			rx_hndl = &desc->rx_hndl[i];
			rx_hndl->psk_buff =
				dev_alloc_skb(desc->rx_buf_size);

			if (!rx_hndl->psk_buff) {
				wiphy_err(priv->hw->wiphy,
					  "rxdesc %i: no skbuff available\n",
					  i);
				return -ENOMEM;
			}

			skb_reserve(rx_hndl->psk_buff,
				    SYSADPT_MIN_BYTES_HEADROOM);
			desc->prx_ring[i].rx_control =
				EAGLE_RXD_CTRL_DRIVER_OWN;
			desc->prx_ring[i].status = EAGLE_RXD_STATUS_OK;
			desc->prx_ring[i].qos_ctrl = 0x0000;
			desc->prx_ring[i].channel = 0x00;
			desc->prx_ring[i].rssi = 0x00;
			desc->prx_ring[i].pkt_len =
				cpu_to_le16(SYSADPT_MAX_AGGR_SIZE);
			dma = dma_map_single(&card->pdev->dev,
					     rx_hndl->psk_buff->data,
					     desc->rx_buf_size,
					     DMA_FROM_DEVICE);
			if (dma_mapping_error(&card->pdev->dev, dma)) {
				wiphy_err(priv->hw->wiphy,
					  "failed to map pci memory!\n");
				dev_kfree_skb_any(rx_hndl->psk_buff);
				rx_hndl->psk_buff = NULL;
				// Previously initialized ring entries handled in mwl_rx_ring_cleanup
				return -ENOMEM;
			}

#ifdef CONFIG_ARCH_DMA_ADDR_T_64BIT
			// Some platforms have been found to return 64 bit DMA addresses
			// even though 32 bit addresses are required by the call to
			// pcie_set_dma_mask() in mwl_pcie_init()
			// Check for that here and fail gracefully with a hint instead of
			// crashing later
			if (dma >> 32) {
				wiphy_err(priv->hw->wiphy,
					  "dma_map_single() returned 64 bit DMA address %llx\n",
					  (long long unsigned int)dma);
				wiphy_err(priv->hw->wiphy,
					  "64 bit DMA is not supported, failing!!\n");
				dma_unmap_single(&card->pdev->dev,
						 dma,
						 desc->rx_buf_size,
						 DMA_FROM_DEVICE);
				dev_kfree_skb_any(rx_hndl->psk_buff);
				rx_hndl->psk_buff = NULL;
				// Previously initialized ring entries handled in mwl_rx_ring_cleanup
				return -ENOMEM;
			}
#endif

			desc->prx_ring[i].pphys_buff_data = cpu_to_le32(dma);
			val = (u32)desc->pphys_rx_ring +
			      ((i + 1) * sizeof(struct mwl_rx_desc));
			desc->prx_ring[i].pphys_next = cpu_to_le32(val);
			rx_hndl->pdesc = &desc->prx_ring[i];
			if (i < (SYSADPT_MAX_NUM_RX_DESC - 1))
				rx_hndl->pnext = &desc->rx_hndl[i + 1];
		}
		desc->prx_ring[SYSADPT_MAX_NUM_RX_DESC - 1].pphys_next =
			cpu_to_le32((u32)desc->pphys_rx_ring);
		desc->rx_hndl[SYSADPT_MAX_NUM_RX_DESC - 1].pnext =
			&desc->rx_hndl[0];
		desc->pnext_rx_hndl = &desc->rx_hndl[0];

		return 0;
	}

	wiphy_err(priv->hw->wiphy, "no valid RX mem\n");

	return -ENOMEM;
}

static void mwl_rx_ring_cleanup(struct mwl_priv *priv)
{
	struct mwl_pcie_card *card = priv->intf;
	struct mwl_desc_data *desc;
	int i;
	struct mwl_rx_hndl *rx_hndl;

	desc = &priv->desc_data[0];

	if (desc->prx_ring) {
		for (i = 0; i < SYSADPT_MAX_NUM_RX_DESC; i++) {
			rx_hndl = &desc->rx_hndl[i];
			if (!rx_hndl->psk_buff)
				continue;

			dma_unmap_single(&card->pdev->dev,
					 le32_to_cpu
					 (rx_hndl->pdesc->pphys_buff_data),
					 desc->rx_buf_size,
					 DMA_FROM_DEVICE);

			wiphy_dbg(priv->hw->wiphy,
				   "Rx: unmapped+free'd %i 0x%p 0x%x %i\n",
				   i, rx_hndl->psk_buff->data,
				   le32_to_cpu(rx_hndl->pdesc->pphys_buff_data),
				   desc->rx_buf_size);

			dev_kfree_skb_any(rx_hndl->psk_buff);
			rx_hndl->psk_buff = NULL;
		}
	}
}

static void mwl_rx_ring_free(struct mwl_priv *priv)
{
	struct mwl_desc_data *desc;

	desc = &priv->desc_data[0];

	if (desc->prx_ring) {
		mwl_rx_ring_cleanup(priv);

		dma_free_coherent(priv->dev,
				  MAX_NUM_RX_RING_BYTES,
				  desc->prx_ring,
				  desc->pphys_rx_ring);

		desc->prx_ring = NULL;
	}

	kfree(desc->rx_hndl);

	desc->pnext_rx_hndl = NULL;
}

static int mwl_rx_refill(struct mwl_priv *priv, struct mwl_rx_hndl *rx_hndl)
{
	struct mwl_pcie_card *card = priv->intf;
	struct mwl_desc_data *desc;
	dma_addr_t dma;

	desc = &priv->desc_data[0];

	rx_hndl->psk_buff = dev_alloc_skb(desc->rx_buf_size);

	if (!rx_hndl->psk_buff)
		return -ENOMEM;

	skb_reserve(rx_hndl->psk_buff, SYSADPT_MIN_BYTES_HEADROOM);

	rx_hndl->pdesc->status = EAGLE_RXD_STATUS_OK;
	rx_hndl->pdesc->qos_ctrl = 0x0000;
	rx_hndl->pdesc->channel = 0x00;
	rx_hndl->pdesc->rssi = 0x00;
	rx_hndl->pdesc->pkt_len = cpu_to_le16(desc->rx_buf_size);

	dma = dma_map_single(&card->pdev->dev,
			     rx_hndl->psk_buff->data,
			     desc->rx_buf_size,
			     DMA_FROM_DEVICE);
	if (dma_mapping_error(&card->pdev->dev, dma)) {
		dev_kfree_skb_any(rx_hndl->psk_buff);
		wiphy_err(priv->hw->wiphy,
			  "failed to map pci memory!\n");
		return -ENOMEM;
	}

	rx_hndl->pdesc->pphys_buff_data = cpu_to_le32(dma);

	return 0;
}


static void mwl_set_bit(struct mwl_priv *priv, int bit_num, volatile void * addr)
{
	struct mwl_pcie_card *card = priv->intf;
	u32 intr_status;
	unsigned long flags;
	spin_lock_irqsave(&card->intr_status_lock,flags);
	intr_status = readl(addr);
	intr_status = intr_status | (1 << bit_num);
	writel(intr_status, addr);
	spin_unlock_irqrestore(&card->intr_status_lock,flags);
}

static void mwl_clear_bit(struct mwl_priv *priv, int bit_num, volatile void *addr)
{
	struct mwl_pcie_card *card = priv->intf;
	u32 intr_status;
	unsigned long flags;
	spin_lock_irqsave(&card->intr_status_lock, flags);
	intr_status = readl(addr);
	intr_status = intr_status & ~(1 << bit_num);
	writel(intr_status, addr);
	spin_unlock_irqrestore(&card->intr_status_lock, flags);
}

void mwl_pcie_rx_recv(unsigned long data)
{
	struct ieee80211_hw *hw = (struct ieee80211_hw *)data;
	struct mwl_priv *priv = hw->priv;
	struct mwl_pcie_card *card = priv->intf;
	struct mwl_desc_data *desc;
	struct mwl_rx_hndl *curr_hndl;
	int work_done = 0;
	struct sk_buff *prx_skb = NULL;
	int pkt_len;
	struct ieee80211_rx_status status;
	struct mwl_vif *mwl_vif = NULL;
	struct ieee80211_hdr *wh;
	struct mwl_rx_event_data *rx_evnt;

	desc = &priv->desc_data[0];
	curr_hndl = desc->pnext_rx_hndl;

	if (!curr_hndl) {
		mwl_set_bit(priv, MACREG_A2HRIC_BIT_NUM_RX_RDY, card->iobase1 + MACREG_REG_A2H_INTERRUPT_STATUS_MASK);

		wiphy_warn(hw->wiphy, "busy or no receiving packets\n");
		return;
	}

	while ((curr_hndl->pdesc->rx_control == EAGLE_RXD_CTRL_DMA_OWN) &&
	       (work_done < priv->recv_limit)) {
		prx_skb = curr_hndl->psk_buff;
		if (!prx_skb)
			goto out;
		dma_unmap_single(&card->pdev->dev,
				 le32_to_cpu(curr_hndl->pdesc->pphys_buff_data),
				 desc->rx_buf_size,
				 DMA_FROM_DEVICE);
		pkt_len = le16_to_cpu(curr_hndl->pdesc->pkt_len);

		if (skb_tailroom(prx_skb) < pkt_len) {
			dev_kfree_skb_any(prx_skb);
			goto out;
		}

		if (curr_hndl->pdesc->payldType == RX_PAYLOAD_TYPE_EVENT_INFO) {
			rx_evnt = (struct mwl_rx_event_data *)prx_skb->data;
			mwl_handle_rx_event(hw, rx_evnt);
			dev_kfree_skb_any(prx_skb);
			goto out;
		}

		if ((curr_hndl->pdesc->channel != hw->conf.chandef.chan->hw_value) &&
			!(priv->roc.tmr_running && priv->roc.in_progress &&
				(curr_hndl->pdesc->channel == priv->roc.chan))) {
			dev_kfree_skb_any(prx_skb);
			goto out;
		}

		mwl_rx_prepare_status(curr_hndl->pdesc, &status);

		priv->noise = curr_hndl->pdesc->noise_floor;

		wh = &((struct mwl_dma_data *)prx_skb->data)->wh;

		if (ieee80211_has_protected(wh->frame_control)) {
			/* Check if hw crypto has been enabled for
			 * this bss. If yes, set the status flags
			 * accordingly
			 */
			if (ieee80211_has_tods(wh->frame_control)) {
				mwl_vif = mwl_rx_find_vif_bss(priv, wh->addr1);
				if (!mwl_vif && ieee80211_has_a4(wh->frame_control))
					mwl_vif = mwl_rx_find_vif_bss(priv,wh->addr2);
			}
			else if (ieee80211_has_fromds(wh->frame_control))
				mwl_vif = mwl_rx_find_vif_bss(priv, wh->addr2);
			else
				mwl_vif = mwl_rx_find_vif_bss(priv, wh->addr3);

			if  (mwl_vif && mwl_vif->is_hw_crypto_enabled) {
				/* When MMIC ERROR is encountered
				 * by the firmware, payload is
				 * dropped and only 32 bytes of
				 * mwlwifi Firmware header is sent
				 * to the host.
				 *
				 * We need to add four bytes of
				 * key information.  In it
				 * MAC80211 expects keyidx set to
				 * 0 for triggering Counter
				 * Measure of MMIC failure.
				 */
				if (status.flag & RX_FLAG_MMIC_ERROR) {
					struct mwl_dma_data *tr;

					tr = (struct mwl_dma_data *)
					     prx_skb->data;
					memset((void *)&tr->data, 0, 4);
					pkt_len += 4;
					/* IV is stripped in this case
					* Indicate that, so mac80211 doesn't attempt to decrypt the
					* packet and fail prior to handling the MMIC error indication
					*/
					status.flag |= RX_FLAG_IV_STRIPPED;
				}

				if (!ieee80211_is_auth(wh->frame_control))
					/* For WPA2 frames, AES header/MIC are
					** present to enable mac80211 to check
					** for replay attacks
					 */
					status.flag |= RX_FLAG_DECRYPTED |
						       RX_FLAG_MMIC_STRIPPED;
			}
		}

		skb_put(prx_skb, pkt_len);
		mwl_rx_remove_dma_header(prx_skb, curr_hndl->pdesc->qos_ctrl);

		wh = (struct ieee80211_hdr *)prx_skb->data;

		if (ieee80211_is_mgmt(wh->frame_control)) {
			struct ieee80211_mgmt *mgmt;
			__le16 capab;

			mgmt = (struct ieee80211_mgmt *)prx_skb->data;

			if (unlikely(ieee80211_is_action(wh->frame_control) &&
				     mgmt->u.action.category ==
				     WLAN_CATEGORY_BACK &&
				     mgmt->u.action.u.addba_resp.action_code ==
				     WLAN_ACTION_ADDBA_RESP)) {
				capab = mgmt->u.action.u.addba_resp.capab;
				if (le16_to_cpu(capab) & 1)
					mwl_rx_enable_sta_amsdu(priv, mgmt->sa);
			}
		}

#if 0 //def CONFIG_MAC80211_MESH
		if (ieee80211_is_data_qos(wh->frame_control) &&
		    ieee80211_has_a4(wh->frame_control)) {
			u8 *qc = ieee80211_get_qos_ctl(wh);

			if (*qc & IEEE80211_QOS_CTL_A_MSDU_PRESENT)
				if (mwl_rx_process_mesh_amsdu(priv, prx_skb,
							      &status))
					goto out;
		}
#endif
		memcpy(IEEE80211_SKB_RXCB(prx_skb), &status, sizeof(status));
		mwl_rx_upload_pkt(hw, prx_skb);
out:
		mwl_rx_refill(priv, curr_hndl);
		curr_hndl->pdesc->rx_control = EAGLE_RXD_CTRL_DRIVER_OWN;
		curr_hndl->pdesc->qos_ctrl = 0;
		curr_hndl = curr_hndl->pnext;
		work_done++;
	}

	desc->pnext_rx_hndl = curr_hndl;

	mwl_set_bit(priv, MACREG_A2HRIC_BIT_NUM_RX_RDY, card->iobase1 + MACREG_REG_A2H_INTERRUPT_STATUS_MASK);

	return;
}


int mwl_rx_init(struct ieee80211_hw *hw)
{
	struct mwl_priv *priv = hw->priv;
	int rc;

	rc = mwl_rx_ring_alloc(priv);
	if (rc) {
		wiphy_err(hw->wiphy, "allocating RX ring failed\n");
		return rc;
	}

	rc = mwl_rx_ring_init(priv);
	if (rc) {
		mwl_rx_ring_free(priv);
		wiphy_err(hw->wiphy,
			  "initializing RX ring failed\n");
		return rc;
	}

	return 0;
}

void mwl_rx_deinit(struct ieee80211_hw *hw)
{
	struct mwl_priv *priv = hw->priv;

	mwl_rx_ring_cleanup(priv);
	mwl_rx_ring_free(priv);
}

/* tx */
#define MAX_NUM_TX_RING_BYTES  (SYSADPT_MAX_NUM_TX_DESC * \
				sizeof(struct mwl_tx_desc))

#define MAX_NUM_TX_HNDL_BYTES   (SYSADPT_MAX_NUM_TX_DESC * \
				sizeof(struct mwl_tx_hndl))
static int mwl_tx_ring_alloc(struct mwl_priv *priv)
{
	struct mwl_desc_data *desc;
	int num;
	u8 *mem;

	desc = &priv->desc_data[0];

	mem = dma_alloc_coherent(priv->dev,
				 MAX_NUM_TX_RING_BYTES *
				 SYSADPT_NUM_OF_DESC_DATA,
				 &desc->pphys_tx_ring,
				 GFP_KERNEL);

	if (!mem) {
		wiphy_err(priv->hw->wiphy, "cannot alloc mem\n");
		return -ENOMEM;
	}

	for (num = 0; num < SYSADPT_NUM_OF_DESC_DATA; num++) {
		desc = &priv->desc_data[num];

		desc->ptx_ring = (struct mwl_tx_desc *)
			(mem + num * MAX_NUM_TX_RING_BYTES);

		desc->pphys_tx_ring = (dma_addr_t)
			((u32)priv->desc_data[0].pphys_tx_ring +
			num * MAX_NUM_TX_RING_BYTES);

		memset(desc->ptx_ring, 0x00,
		       MAX_NUM_TX_RING_BYTES);
	}

	mem = kmalloc(MAX_NUM_TX_HNDL_BYTES * SYSADPT_NUM_OF_DESC_DATA,
		      GFP_KERNEL);

	if (!mem) {
		wiphy_err(priv->hw->wiphy, "cannot alloc mem\n");
		dma_free_coherent(priv->dev,
				  MAX_NUM_TX_RING_BYTES *
				  SYSADPT_NUM_OF_DESC_DATA,
				  priv->desc_data[0].ptx_ring,
				  priv->desc_data[0].pphys_tx_ring);
		return -ENOMEM;
	}

	for (num = 0; num < SYSADPT_NUM_OF_DESC_DATA; num++) {
		desc = &priv->desc_data[num];

		desc->tx_hndl = (struct mwl_tx_hndl *)
			(mem + num * MAX_NUM_TX_HNDL_BYTES);

		memset(desc->tx_hndl, 0x00,
		       MAX_NUM_TX_HNDL_BYTES);
	}

	return 0;
}

static int mwl_tx_ring_init(struct mwl_priv *priv)
{
	int num;
	int i;
	struct mwl_desc_data *desc;

	for (num = 0; num < SYSADPT_NUM_OF_DESC_DATA; num++) {
		skb_queue_head_init(&priv->txq[num]);
		priv->fw_desc_cnt[num] = 0;

		if (!IS_PFU_ENABLED(priv->chip_type)) {
			desc = &priv->desc_data[num];

			if (desc->ptx_ring) {
				for (i = 0; i < SYSADPT_MAX_NUM_TX_DESC; i++) {
				desc->ptx_ring[i].status =
					cpu_to_le32(EAGLE_TXD_STATUS_IDLE);
					desc->ptx_ring[i].pphys_next =
					cpu_to_le32((u32)desc->pphys_tx_ring +
					((i + 1) * sizeof(struct mwl_tx_desc)));
					desc->tx_hndl[i].pdesc =
						&desc->ptx_ring[i];
					if (i < SYSADPT_MAX_NUM_TX_DESC - 1)
						desc->tx_hndl[i].pnext =
						&desc->tx_hndl[i + 1];
				}
				desc->ptx_ring[SYSADPT_MAX_NUM_TX_DESC - 1].pphys_next =
					cpu_to_le32((u32)desc->pphys_tx_ring);
				desc->tx_hndl[SYSADPT_MAX_NUM_TX_DESC - 1].pnext =
					&desc->tx_hndl[0];

				desc->pstale_tx_hndl = &desc->tx_hndl[0];
				desc->pnext_tx_hndl  = &desc->tx_hndl[0];
			} else {
				wiphy_err(priv->hw->wiphy, "no valid TX mem\n");
				return -ENOMEM;
			}
		}
	}

	return 0;
}

static void mwl_tx_ring_cleanup(struct mwl_priv *priv)
{
	struct mwl_pcie_card *card = priv->intf;
	int cleaned_tx_desc = 0;
	int num, i;
	struct mwl_desc_data *desc;

	for (num = 0; num < SYSADPT_NUM_OF_DESC_DATA; num++) {
		skb_queue_purge(&priv->txq[num]);
		priv->fw_desc_cnt[num] = 0;

		if (!IS_PFU_ENABLED(priv->chip_type)) {
			desc = &priv->desc_data[num];

			if (desc->ptx_ring) {
				for (i = 0; i < SYSADPT_MAX_NUM_TX_DESC; i++) {
					if (!desc->tx_hndl[i].psk_buff)
						continue;

					wiphy_dbg(priv->hw->wiphy,
							"Tx: unmapped and free'd %i 0x%p 0x%x\n",
							i,
							desc->tx_hndl[i].psk_buff->data,
							le32_to_cpu(
								desc->ptx_ring[i].pkt_ptr));
					dma_unmap_single(&card->pdev->dev,
							le32_to_cpu(
								desc->ptx_ring[i].pkt_ptr),
							desc->tx_hndl[i].psk_buff->len,
							DMA_TO_DEVICE);
					dev_kfree_skb_any(desc->tx_hndl[i].psk_buff);
					desc->ptx_ring[i].status =
						cpu_to_le32(EAGLE_TXD_STATUS_IDLE);
					desc->ptx_ring[i].pkt_ptr = 0;
					desc->ptx_ring[i].pkt_len = 0;
					desc->tx_hndl[i].psk_buff = NULL;
					cleaned_tx_desc++;
				}
			}
		}
	}

	wiphy_info(priv->hw->wiphy, "cleaned %i TX descr\n", cleaned_tx_desc);
}

static void mwl_tx_ring_free(struct mwl_priv *priv)
{
	int num;

	if (priv->desc_data[0].ptx_ring) {
		dma_free_coherent(priv->dev,
				  MAX_NUM_TX_RING_BYTES *
				  SYSADPT_NUM_OF_DESC_DATA,
				  priv->desc_data[0].ptx_ring,
				  priv->desc_data[0].pphys_tx_ring);
	}

	for (num = 0; num < SYSADPT_NUM_OF_DESC_DATA; num++) {
		if (priv->desc_data[num].ptx_ring)
			priv->desc_data[num].ptx_ring = NULL;
		priv->desc_data[num].pstale_tx_hndl = NULL;
		priv->desc_data[num].pnext_tx_hndl = NULL;
	}

	kfree(priv->desc_data[0].tx_hndl);
}
void mwl_tx_deinit(struct ieee80211_hw *hw)
{
	struct mwl_priv *priv = hw->priv;

	mwl_tx_ring_cleanup(priv);

	if (IS_PFU_ENABLED(priv->chip_type))
		wlan_pcie_delete_txbd_ring(hw);
	else
		mwl_tx_ring_free(priv);
}

static bool mwl_pcie_is_tx_available(struct mwl_priv *priv, int desc_num)
{
	struct mwl_pcie_card *card = priv->intf;
	struct mwl_tx_hndl *tx_hndl;

	if (IS_PFU_ENABLED(priv->chip_type))
		return PCIE_TXBD_NOT_FULL(priv->txbd_wrptr, priv->txbd_rdptr);

	tx_hndl = priv->desc_data[desc_num].pnext_tx_hndl;

	if (!tx_hndl->pdesc)
		return false;

	if (tx_hndl->pdesc->status != EAGLE_TXD_STATUS_IDLE) {
		/* Interrupt F/W anyway */
		if (tx_hndl->pdesc->status &
		    cpu_to_le32(EAGLE_TXD_STATUS_FW_OWNED))
			writel(MACREG_H2ARIC_BIT_PPA_READY,
			       card->iobase1 +
			       MACREG_REG_H2A_INTERRUPT_EVENTS);
		return false;
	}

	return true;
}
static int mwl_pcie_init_post(struct mwl_priv *priv)
{
	struct mwl_pcie_card *card = priv->intf;
	int i;

	// Stop Shutdown mode not supported
	priv->stop_shutdown = false;

	if (priv->mfg_mode) {
		// Assume ST 60 with one interface
		priv->radio_caps.capability = 0;
		priv->radio_caps.num_mac = 1;
	}

	if (!IS_PFU_ENABLED(priv->chip_type)) {
		writel(priv->desc_data[0].pphys_tx_ring,
			card->iobase0 + priv->desc_data[0].wcb_base);
		for (i = 1; i < SYSADPT_TOTAL_TX_QUEUES; i++)
			writel(priv->desc_data[i].pphys_tx_ring,
				card->iobase0 + priv->desc_data[i].wcb_base);
	}

	writel(priv->desc_data[0].pphys_rx_ring,
	       card->iobase0 + priv->desc_data[0].rx_desc_read);
	writel(priv->desc_data[0].pphys_rx_ring,
	       card->iobase0 + priv->desc_data[0].rx_desc_write);

	return 0;
}

/*
 * This function initializes the PCI-E host memory space, WCB rings, etc.
 *
 * The following initializations steps are followed -
 *      - Allocate TXBD ring buffers
 *      - Allocate RXBD ring buffers
 *      - Allocate event BD ring buffers
 *      - Allocate command response ring buffer
 *      - Allocate sleep cookie buffer
 */
static int mwl_pcie_init(struct mwl_priv *priv)
{
	struct mwl_pcie_card *card = priv->intf;
	struct pci_dev *pdev = card->pdev;
	struct ieee80211_hw *hw;
	int rc = 0;

	priv->chip_type = card->chip_type;
	priv->host_if = MWL_IF_PCIE;

	hw = priv->hw;
	priv->irq=-1;
	card->priv = priv;

	rc = pci_enable_device(pdev);
	if (rc) {
		wiphy_err(hw->wiphy, "%s: cannot enable new PCI device.\n",
			MWL_DRV_NAME);
		goto err_enable_dev;
	}

	pci_set_master(pdev);

	rc = dma_set_mask(&pdev->dev, DMA_BIT_MASK(32));
	if (rc) {
		wiphy_err(hw->wiphy, "%s: 32-bit PCI DMA not supported by host",
			MWL_DRV_NAME);
		goto err_set_dma;
	}

	rc = dma_set_coherent_mask(&pdev->dev, DMA_BIT_MASK(32));
	if (rc) {
		wiphy_err(hw->wiphy, "%s: 32-bit consistent PCI DMA not supported by host",
			MWL_DRV_NAME);
		goto err_set_dma;
	}

	pci_set_drvdata(pdev, hw);
	priv->dev = &pdev->dev;
	rc = mwl_alloc_pci_resource(priv);
	if (rc) {
		wiphy_err(hw->wiphy, "%s: fail to allocate pci resource.\n",
			MWL_DRV_NAME);
		goto err_alloc_resource;
	}

	spin_lock_init(&card->intr_status_lock);
	rc = mwl_tx_init(hw);
	if (rc) {
		wiphy_err(hw->wiphy, "%s: fail to initialize TX\n",
			  MWL_DRV_NAME);
		goto err_mwl_tx_init;
	}

	rc = mwl_rx_init(hw);
	if (rc) {
		wiphy_err(hw->wiphy, "%s: fail to initialize RX\n",
			  MWL_DRV_NAME);
		goto err_mwl_rx_init;
	}

	return rc;

err_mwl_rx_init:
	mwl_tx_deinit(hw);

err_mwl_tx_init:
	mwl_free_pci_resource(priv);

err_alloc_resource:
	pci_set_drvdata(pdev, NULL);

err_set_dma:
	pci_disable_device(pdev);

err_enable_dev:
	return rc;
}


static int mwl_pcie_intr_init(struct mwl_priv *priv)
{
	int rc = 0, try_legacy_irq = 1;
	struct mwl_pcie_card *card = priv->intf;
	struct ieee80211_hw *hw = priv->hw;

	card->intr_mode = 0;

	if (pcie_intr_mode == 1) {

		rc = pci_enable_msi(card->pdev);

		if (rc) {
			wiphy_err(hw->wiphy, "%s: pci_enable_msi failed %d\n",
					MWL_DRV_NAME, rc);
		} else {
			rc = request_irq(card->pdev->irq, mwl_pcie_isr,
					0, MWL_DRV_NAME, priv->hw);
			if (rc == 0) {
				card->intr_mode = 1;
				try_legacy_irq = 0;
			} else {
				wiphy_err(hw->wiphy,
					"%s: request_irq MSI failed %d\n",
					MWL_DRV_NAME, rc);

				pci_disable_msi(card->pdev);

			}
		}
	}

	if (try_legacy_irq) {
		rc = request_irq(card->pdev->irq, mwl_pcie_isr,
				IRQF_SHARED, MWL_DRV_NAME, priv->hw);
		if (rc) {
			priv->irq = -1;
			wiphy_err(hw->wiphy,
					"%s: request_irq Legacy failed %d\n",
					MWL_DRV_NAME, rc);

			return rc;
		}
	}

	priv->irq = card->pdev->irq;

	return 0;
}


static int mwl_pcie_intr_deinit(struct mwl_priv *priv)
{
	struct mwl_pcie_card *card = (struct mwl_pcie_card *)priv->intf;

	if (priv->irq != -1) {
		free_irq(priv->irq, priv->hw);
		priv->irq = -1;

		if(card->intr_mode == 1) {
			pci_disable_msi(card->pdev);
			card->intr_mode = 0;
		}
	}

	return 0;
}

static void mwl_pcie_cleanup(struct mwl_priv *priv)
{
	struct mwl_pcie_card *card = priv->intf;
	struct pci_dev *pdev = card->pdev;

	mwl_rx_deinit(priv->hw);
	mwl_tx_deinit(priv->hw);

	mwl_pcie_intr_deinit(priv);

	mwl_free_pci_resource(priv);
	if (pdev) {
		pci_disable_device(pdev);
		pci_set_drvdata(pdev, NULL);
	}
}

static void mwl_fwdl_trig_pcicmd(struct mwl_priv *priv)
{
	struct mwl_pcie_card *card = (struct mwl_pcie_card *)priv->intf;

	writel(priv->pphys_cmd_buf, card->iobase1 + MACREG_REG_GEN_PTR);

	writel((u32)((u64)priv->pphys_cmd_buf >> 32), card->iobase1 + MACREG_REG_INT_CODE);

	writel(MACREG_H2ARIC_BIT_DOOR_BELL,
	       card->iobase1 + MACREG_REG_H2A_INTERRUPT_EVENTS);
}

static void mwl_fwdl_trig_pcicmd_bootcode(struct mwl_priv *priv)
{
	struct mwl_pcie_card *card = (struct mwl_pcie_card *) priv->intf;

	/* Write the lower 32bits of the physical address to low command
	 * address scratch register
	 */
	writel((u32)priv->pphys_cmd_buf, card->iobase1 + MACREG_REG_GEN_PTR);

	/* Write the upper 32bits of the physical address to high command
	 * address scratch register
	 */
	writel((u32)((u64)priv->pphys_cmd_buf >> 32), card->iobase1 + MACREG_REG_INT_CODE);

	/* Ring the door bell */
	writel(MACREG_H2ARIC_BIT_DOOR_BELL,
	       card->iobase1 + MACREG_REG_H2A_INTERRUPT_EVENTS);
}

static bool mwl_pcie_check_fw_status(struct mwl_priv *priv)
{
	struct mwl_pcie_card *card;
	u32 int_code = 0;
	bool rc = false;

	u32 fwreadyReg = MACREG_REG_FW_STATUS;
	u32 curr_iteration = 0;

	card = (struct mwl_pcie_card *)priv->intf;

	/* TBD do we need to disable interrupts before this */
	if (priv->mfg_mode) {
		u32 fwreadysignature = MFG_FW_READY_SIGNATURE;

		writel(fwreadysignature, card->iobase1 + MACREG_REG_DRV_READY);
		curr_iteration = 10;

		do {
			curr_iteration--;
			lrdmwl_delay(10);

			int_code = readl(card->iobase1 + fwreadyReg);
		} while ((curr_iteration) &&
				((int_code != MFG_FW_READY_SIGNATURE) && (int_code != HOSTCMD_SOFTAP_FWRDY_SIGNATURE)));

		if (curr_iteration) {
			rc = true;
		}
	}
	else
	{
		int_code = readl(card->iobase1 + fwreadyReg);

		if ((int_code == MFG_FW_READY_SIGNATURE) ||
			(int_code == HOSTCMD_SOFTAP_FWRDY_SIGNATURE))
		{
			rc = true;
		}
	}

	return rc;
}


/* Combo firmware image is a combination of
 * (1) combo crc heaer, start with CMD5
 * (2) bluetooth image, start with CMD7, end with CMD6, data wrapped in CMD1.
 * (3) wifi image.
 *
 * This function bypass the header and bluetooth part, return
 * the offset of tail wifi-only part. If the image is already wifi-only,
 * that is start with CMD1, return 0.
 */

static int mwl_extract_wifi_fw(struct mwl_priv *priv) {
	struct ieee80211_hw *hw;
	const void *firmware;
	u32 firmware_len;
	const struct mwifiex_fw_data *fwdata;
	u32 offset = 0, data_len, dnld_cmd;
	int ret = 0;
	bool cmd7_before = false, first_cmd = false;

	hw = priv->hw;

	firmware = priv->fw_ucode->data;
	firmware_len = priv->fw_ucode->size;

	while (1) {
		/* Check for integer and buffer overflow */
		if (offset + sizeof(fwdata->header) < sizeof(fwdata->header) ||
		    offset + sizeof(fwdata->header) >= firmware_len) {
			wiphy_err(hw->wiphy,
				    "extract wifi-only fw failure!\n");
			ret = -1;
			goto done;
		}

		fwdata = firmware + offset;
		dnld_cmd = le32_to_cpu(fwdata->header.dnld_cmd);
		data_len = le32_to_cpu(fwdata->header.data_length);

		/* Skip past header */
		offset += sizeof(fwdata->header);

		switch (dnld_cmd) {
		case MWIFIEX_FW_DNLD_CMD_1:
			if (offset + data_len < data_len) {
				wiphy_err(hw->wiphy, "bad FW parse\n");
				ret = -1;
				goto done;
			}

			/* Image start with cmd1, already wifi-only firmware */
			if (!first_cmd) {
				wiphy_info(hw->wiphy,
					    "input wifi-only firmware\n");
				return 0;
			}

			if (!cmd7_before) {
				wiphy_err(hw->wiphy,
					    "no cmd7 before cmd1!\n");
				ret = -1;
				goto done;
			}
			offset += data_len;
			break;
		case MWIFIEX_FW_DNLD_CMD_5:
			first_cmd = true;
			/* Check for integer overflow */
			if (offset + data_len < data_len) {
				wiphy_err(hw->wiphy, "bad FW parse\n");
				ret = -1;
				goto done;
			}
			offset += data_len;
			break;
		case MWIFIEX_FW_DNLD_CMD_6:
			first_cmd = true;
			/* Check for integer overflow */
			if (offset + data_len < data_len) {
				wiphy_err(hw->wiphy, "bad FW parse\n");
				ret = -1;
				goto done;
			}
			offset += data_len;
			if (offset >= firmware_len) {
				wiphy_err(hw->wiphy,
					    "extract wifi-only fw failure!\n");
				ret = -1;
			} else {
				ret = offset;
			}
			goto done;
		case MWIFIEX_FW_DNLD_CMD_7:
			first_cmd = true;
			cmd7_before = true;
			break;
		default:
			wiphy_err(hw->wiphy, "unknown dnld_cmd %d\n",
				    dnld_cmd);
			ret = -1;
			goto done;
		}
	}

done:
	return ret;
}


static int mwl_pcie_program_firmware(struct mwl_priv *priv)
{
	const struct firmware *fw;
	struct ieee80211_hw *hw;
	struct mwl_pcie_card *card;
	u32 curr_iteration;
	u32 size_fw_downloaded = 0;
	u32 int_code = 0;
	u32 len = 0;
	u32 regval = MACREG_A2HRIC_BIT_MASK;


	u32 fwreadysignature = priv->mfg_mode ?
		MFG_FW_READY_SIGNATURE : HOSTCMD_SOFTAP_FWRDY_SIGNATURE;
	u32 fwreadyReg = MACREG_REG_FW_STATUS;

	fw = priv->fw_ucode;
	card = (struct mwl_pcie_card *)priv->intf;
	hw = priv->hw;

	if (priv->mfg_mode) {
		wiphy_info(hw->wiphy, "mfg_mode: overriding I/F header len to %d\n",INTF_HEADER_LEN_MFG);
		priv->if_ops.inttf_head_len = INTF_HEADER_LEN_MFG;
	}

	/* FW before jumping to boot rom, it will enable PCIe transaction retry,
	 * wait for boot code to stop it.
	 */
	lrdmwl_delay(5*1000);

	if(mwl_pcie_check_fw_status(priv))
	{
		wiphy_err(hw->wiphy, "%s: FW already running - resetting\n", __func__);
		INIT_WORK(&card->fw_reset_work, lrd_pci_fw_reset_workfn);
		schedule_work(&card->fw_reset_work);
		return -EINPROGRESS;
	}

	if (!priv->tx_amsdu_enable)
		regval &= ~MACREG_A2HRIC_BIT_QUE_EMPTY;

	writel(regval, card->iobase1 + MACREG_REG_A2H_INTERRUPT_CLEAR_SEL);
	writel(0x00, card->iobase1 + MACREG_REG_A2H_INTERRUPT_CAUSE);
	writel(0x00, card->iobase1 + MACREG_REG_A2H_INTERRUPT_MASK);
	writel(regval, card->iobase1 + MACREG_REG_A2H_INTERRUPT_STATUS_MASK);

	/* this routine interacts with SC2 bootrom to download firmware binary
	 * to the device. After DMA'd to SC2, the firmware could be deflated to
	 * reside on its respective blocks such as ITCM, DTCM, SQRAM,
	 * (or even DDR, AFTER DDR is init'd before fw download
	 */
	wiphy_info(hw->wiphy, "Starting fw download\n");

	/* make sure SCRATCH2 C40 (MACREG_REG_CMD_SIZE) is clear, in case we are too quick
	 * Worst case observed is ~150ms after FLR reset, which should already have been
	 * accounted for by the time we reach here.
	 */
	curr_iteration = 10;
	while (((readl(card->iobase1 + MACREG_REG_CMD_SIZE) == 0) ||
			(readl(card->iobase1 + MACREG_REG_CMD_SIZE) == 0xffffffff)) && curr_iteration)
	{
		msleep(50);
		curr_iteration--;
	}

	if (!curr_iteration)
	{
		wiphy_err(hw->wiphy, "err polling MACREG_REG_CMD_SIZE!\n");
		goto err_download;
	}

	int_code = readl(card->iobase1 + PCIE_SCRATCH_13_REG);
	if (int_code == MWIFIEX_PCIE_FLR_HAPPENS)
	{
		int ret;

		wiphy_info(hw->wiphy, "Function Level Reset detected!  Downloading WiFi FW only...\n");
		ret = mwl_extract_wifi_fw(priv);
		if (ret < 0)
		{
			wiphy_err(hw->wiphy, "Failed to extract wifi fw\n");
			goto err_download;
		}

		size_fw_downloaded = ret;
		wiphy_info(hw->wiphy, "info: dnld wifi firmware from %d bytes\n", size_fw_downloaded);
	}

	while (size_fw_downloaded < fw->size) {
		len = readl(card->iobase1 + MACREG_REG_CMD_SIZE);

		if (!len) {
			break;
		}

		/* this copies the next chunk of fw binary to be delivered */
		memcpy((char *)&priv->pcmd_buf[
			INTF_CMDHEADER_LEN(INTF_HEADER_LEN)],
		       (fw->data + size_fw_downloaded), len);

		/* Write the command length to cmd_size scratch register */
		writel(len, card->iobase1 + MACREG_REG_CMD_SIZE);

		/* this function writes pdata to c10, then write 2 to c18 */
		mwl_fwdl_trig_pcicmd_bootcode(priv);

		curr_iteration = 1000;
		/* Wait for cmd done
		 * Worst case observed is ~1ms
		 */
		do {
			int_code = readl(card->iobase1 + MACREG_REG_H2A_INTERRUPT_CAUSE);
			if ((int_code & MACREG_H2ARIC_BIT_DOOR_BELL) !=
			    MACREG_H2ARIC_BIT_DOOR_BELL)
				break;

			lrdmwl_delay(10);
			curr_iteration--;
		} while (curr_iteration);

		if (curr_iteration == 0) {
			/* This limited loop check allows you to exit gracefully
			 * without locking up your entire system just because fw
			 * download failed
			 */
			wiphy_err(hw->wiphy,
				  "\nExhausted fw segment download\n");
			goto err_download;
		}

		size_fw_downloaded += len;
	}

	wiphy_info(hw->wiphy,
		    "FwSize = %d downloaded Size = %d\n",
		    (int)fw->size, size_fw_downloaded);

	/* Now firware is downloaded successfully, so this part is to check
	 * whether fw can properly execute to an extent that write back
	 * signature to indicate its readiness to the host. NOTE: if your
	 * downloaded fw crashes, this signature checking will fail. This
	 * part is similar as SC1
		 */

	if (!priv->mfg_mode) {
		*((u32 *)&priv->pcmd_buf[INTF_CMDHEADER_LEN(INTF_HEADER_LEN)+1]) = 0;
		mwl_fwdl_trig_pcicmd(priv);
	}
	else {
		writel(fwreadysignature, card->iobase1 + MACREG_REG_DRV_READY);
	}

	// Firmware initialization has been observed to take ~2.5 seconds
	curr_iteration = 200;
	do {
		curr_iteration--;
		if (!priv->mfg_mode) {
			writel(HOSTCMD_SOFTAP_MODE, card->iobase1 + MACREG_REG_GEN_PTR);
		}

		msleep(50);

		int_code = readl(card->iobase1 + fwreadyReg);

	} while ((curr_iteration) && (int_code != fwreadysignature));

	if (curr_iteration == 0) {
		wiphy_err(hw->wiphy, "No response from firmware!  sig = 0x%x\n", int_code);
		goto err_download;
	}

	wiphy_info(hw->wiphy, "fw download complete\n");
	writel(0x00, card->iobase1 + MACREG_REG_INT_CODE);

	return 0;

err_download:

	return -EIO;
}

static bool mwl_pcie_check_card_status(struct mwl_priv *priv)
{
	struct mwl_pcie_card *card = (struct mwl_pcie_card *)priv->intf;
	u32 regval;

	regval = readl(card->iobase1 + MACREG_REG_INT_CODE);
	if (regval == 0xffffffff) {
		wiphy_err(priv->hw->wiphy, "adapter does not exist\n");
		return false;
	}

	return true;
}

static void mwl_pcie_enable_int(struct mwl_priv *priv)
{
	struct mwl_pcie_card *card = (struct mwl_pcie_card *)priv->intf;
	u32 regval = MACREG_A2HRIC_BIT_MASK;

	if (mwl_pcie_check_card_status(priv)) {
		writel(0x00, card->iobase1 + MACREG_REG_A2H_INTERRUPT_MASK);

		if (!priv->tx_amsdu_enable)
			regval &= ~MACREG_A2HRIC_BIT_QUE_EMPTY;
		writel(regval, card->iobase1 + MACREG_REG_A2H_INTERRUPT_MASK);
	}
}

static void mwl_pcie_disable_int(struct mwl_priv *priv)
{
	struct mwl_pcie_card *card = (struct mwl_pcie_card *)priv->intf;

	if (mwl_pcie_check_card_status(priv))
		writel(0x00, card->iobase1 + MACREG_REG_A2H_INTERRUPT_MASK);
}

static int mwl_pcie_send_command(struct mwl_priv *priv)
{
	struct mwl_pcie_card *card = (struct mwl_pcie_card *)priv->intf;
	struct hostcmd_header *pcmd;

	pcmd = (struct hostcmd_header *)&priv->pcmd_buf[INTF_CMDHEADER_LEN(priv->if_ops.inttf_head_len)];


	if (priv->mfg_mode) {

		if (pcmd->cmd != cpu_to_le16(HOSTCMD_CMD_MFG)) {
			return 0;
		}

		// XXX: Keeping i/f hdr as 0s for now
		writel(le16_to_cpu(pcmd->len) + priv->if_ops.inttf_head_len, card->iobase1 + MACREG_REG_CMD_SIZE);
		writel(priv->pphys_cmd_buf, card->iobase1 + MACREG_REG_CMDRSP_BUF_LO);
		writel(0x0, card->iobase1 + MACREG_REG_CMDRSP_BUF_HI);
	}

	writel(priv->pphys_cmd_buf, card->iobase1 + MACREG_REG_GEN_PTR);

	writel(MACREG_H2ARIC_BIT_DOOR_BELL, card->iobase1 + MACREG_REG_H2A_INTERRUPT_EVENTS);
	return 0;
}

/* Check command response back or not */
static int mwl_pcie_cmd_resp_wait_completed(struct mwl_priv *priv,
		unsigned short cmd)
{
	unsigned int curr_iteration = MAX_WAIT_CMD_RESPONSE_ITERATIONS;
	unsigned short int_code = 0;

	if (priv->mfg_mode && cmd != HOSTCMD_CMD_MFG) {
		usleep_range(250, 500);
		return 0;
	}

	do {
        usleep_range(250, 500);
		int_code = le16_to_cpu(*((__le16 *)&priv->pcmd_buf[
				INTF_CMDHEADER_LEN(priv->if_ops.inttf_head_len)]));
	} while ((int_code != cmd) && (--curr_iteration));

	if (curr_iteration == 0) {
		wiphy_err(priv->hw->wiphy, "cmd 0x%04x=%s timed out\n",
			  cmd, mwl_fwcmd_get_cmd_string(cmd));
		wiphy_err(priv->hw->wiphy, "return code: 0x%04x\n", int_code);
		return -EIO;
	}

	if (priv->chip_type != MWL8997)
		usleep_range(3000, 5000);

	return 0;
}

static int mwl_pcie_host_to_card(struct mwl_priv *priv, int desc_num,
		struct sk_buff *tx_skb)
{
	struct mwl_pcie_card *card = priv->intf;
	struct mwl_tx_hndl *tx_hndl = NULL;
	struct mwl_tx_desc *tx_desc;
	struct mwl_tx_ctrl *tx_ctrl;
	struct ieee80211_tx_info *tx_info;

	dma_addr_t dma;
        unsigned int wrindx;
        const unsigned int num_tx_buffs = MLAN_MAX_TXRX_BD << PCIE_TX_START_PTR;
	tx_info = IEEE80211_SKB_CB(tx_skb);
	tx_ctrl = (struct mwl_tx_ctrl *)&IEEE80211_SKB_CB(tx_skb)->status;

	if (!IS_PFU_ENABLED(priv->chip_type)) {
		tx_hndl = priv->desc_data[desc_num].pnext_tx_hndl;
		tx_hndl->psk_buff = tx_skb;
		tx_desc = tx_hndl->pdesc;
	} else {
		struct mwl_tx_pfu_dma_data *dma_data =
			(struct mwl_tx_pfu_dma_data *)tx_skb->data;
		tx_desc = &dma_data->tx_desc;
	}

	// clear flags then set if required
	tx_desc->flags = 0;
	if (tx_info->flags & IEEE80211_TX_INTFL_DONT_ENCRYPT) {
		tx_desc->flags |= cpu_to_le32(MWL_TX_WCB_FLAGS_DONT_ENCRYPT);
	}

	if (tx_info->flags & IEEE80211_TX_CTL_NO_CCK_RATE) {
		tx_desc->flags |= cpu_to_le32(MWL_TX_WCB_FLAGS_NO_CCK_RATE);
	}

	tx_desc->tx_priority = tx_ctrl->tx_priority;
	tx_desc->qos_ctrl = cpu_to_le16(tx_ctrl->qos_ctrl);
	tx_desc->pkt_len = cpu_to_le16(tx_skb->len);
	tx_desc->packet_info = 0;
	tx_desc->data_rate = 0;
	tx_desc->type = tx_ctrl->type;
	tx_desc->xmit_control = tx_ctrl->xmit_control;
	tx_desc->sap_pkt_info = 0;

	// Note - When PFU is enabled tx_skb contains the tx_desc
	// Therefore must not touch the descriptor after the call to dma_map_single()
	//
	// This limitation does not exist when PFU is not enabled since the
	// tx_desc is located in a separate coherent memory buffer
	if (IS_PFU_ENABLED(priv->chip_type))
		tx_desc->pkt_ptr = cpu_to_le32(sizeof(struct mwl_tx_desc));

	tx_desc->status = cpu_to_le32(EAGLE_TXD_STATUS_FW_OWNED);

	dma = dma_map_single(&card->pdev->dev, tx_skb->data, tx_skb->len, DMA_TO_DEVICE);
	if (dma_mapping_error(&card->pdev->dev, dma)) {
		dev_kfree_skb_any(tx_skb);
		wiphy_err(priv->hw->wiphy,
			  "failed to map pci memory!\n");
		return -ENOMEM;
	}

	if (!IS_PFU_ENABLED(priv->chip_type))
		tx_desc->pkt_ptr = cpu_to_le32(dma);

	if (IS_PFU_ENABLED(priv->chip_type)) {
		wrindx = (priv->txbd_wrptr & MLAN_TXBD_MASK) >> PCIE_TX_START_PTR;
#if 0
		wiphy_err(priv->hw->wiphy,
		"SEND DATA: Attach pmbuf %p at txbd_wridx=%d\n", tx_skb, wrindx);
#endif
		priv->tx_buf_list[wrindx] = tx_skb;
		priv->txbd_ring[wrindx]->paddr = cpu_to_le64(dma);
		priv->txbd_ring[wrindx]->len = cpu_to_le16((unsigned short)tx_skb->len);
		priv->txbd_ring[wrindx]->flags = cpu_to_le16(MLAN_BD_FLAG_FIRST_DESC | MLAN_BD_FLAG_LAST_DESC);

		priv->txbd_ring[wrindx]->frag_len = cpu_to_le16((unsigned short)tx_skb->len);
		priv->txbd_ring[wrindx]->offset = 0;
		priv->txbd_wrptr += MLAN_BD_FLAG_TX_START_PTR;

		if ((priv->txbd_wrptr & MLAN_TXBD_MASK) == num_tx_buffs)
			priv->txbd_wrptr = ((priv->txbd_wrptr & MLAN_BD_FLAG_TX_ROLLOVER_IND) ^
					MLAN_BD_FLAG_TX_ROLLOVER_IND);

		/*
		 * Memory barrier required to ensure write to PCI does not pass writes to memory
		 */
		wmb();

		/* Write the TX ring write pointer in to REG_TXBD_WRPTR */
		writel(priv->txbd_wrptr, card->iobase1 + REG_TXBD_WRPTR);

#if 0
		wiphy_err(priv->hw->wiphy,
		"SEND DATA: Updated <Rd: %#x, Wr: %#x>\n",
				priv->txbd_rdptr, priv->txbd_wrptr);
#endif

	} else {
		/*
		 * Memory barrier required to ensure write to PCI does not pass writes to memory
		 */
		wmb();

		writel(MACREG_H2ARIC_BIT_PPA_READY, card->iobase1 + MACREG_REG_H2A_INTERRUPT_EVENTS);
		priv->desc_data[desc_num].pnext_tx_hndl = tx_hndl->pnext;
		priv->fw_desc_cnt[desc_num]++;
	}

	return 0;
}

void mwl_non_pfu_tx_done(unsigned long data)
{
	struct ieee80211_hw *hw = (struct ieee80211_hw *)data;
	struct mwl_priv *priv = hw->priv;
	struct mwl_pcie_card *card = priv->intf;
	int num;
	struct mwl_desc_data *desc;
	struct mwl_tx_hndl *tx_hndl;
	struct mwl_tx_desc *tx_desc;
	struct sk_buff *done_skb;
	u32 rate;
	struct mwl_dma_data *tr;
	struct ieee80211_tx_info *info;
	struct mwl_tx_ctrl *tx_ctrl;
	struct sk_buff_head *amsdu_pkts;
	int hdrlen;

	spin_lock_bh(&priv->tx_desc_lock);
	for (num = 0; num < SYSADPT_TX_WMM_QUEUES; num++) {
		desc = &priv->desc_data[num];
		tx_hndl = desc->pstale_tx_hndl;
		tx_desc = tx_hndl->pdesc;

		if ((tx_desc->status &
				cpu_to_le32(EAGLE_TXD_STATUS_FW_OWNED)) &&
		    (tx_hndl->pnext->pdesc->status &
		    cpu_to_le32(EAGLE_TXD_STATUS_OK)))
			tx_desc->status = cpu_to_le32(EAGLE_TXD_STATUS_OK);

		while (tx_hndl &&
		       (tx_desc->status & cpu_to_le32(EAGLE_TXD_STATUS_OK)) &&
		       (!(tx_desc->status &
		       cpu_to_le32(EAGLE_TXD_STATUS_FW_OWNED)))) {
			dma_unmap_single(&card->pdev->dev,
					 le32_to_cpu(tx_desc->pkt_ptr),
					 le16_to_cpu(tx_desc->pkt_len),
					 DMA_TO_DEVICE);
			done_skb = tx_hndl->psk_buff;
			rate = le32_to_cpu(tx_desc->rate_info);
			tx_desc->pkt_ptr = 0;
			tx_desc->pkt_len = 0;
			tx_desc->status =
				cpu_to_le32(EAGLE_TXD_STATUS_IDLE);
			tx_hndl->psk_buff = NULL;
			wmb(); /* memory barrier */

			tr = (struct mwl_dma_data *)done_skb->data;
			info = IEEE80211_SKB_CB(done_skb);

			if (ieee80211_is_data(tr->wh.frame_control) ||
			    ieee80211_is_data_qos(tr->wh.frame_control)) {
				tx_ctrl = (struct mwl_tx_ctrl *)&info->status;
				amsdu_pkts = (struct sk_buff_head *)
					tx_ctrl->amsdu_pkts;
				if (amsdu_pkts) {
					mwl_tx_ack_amsdu_pkts(hw, rate,
							      amsdu_pkts);
					dev_kfree_skb_any(done_skb);
					done_skb = NULL;
				} else {
					mwl_tx_prepare_info(hw, rate, info);
				}
			} else {
				mwl_tx_prepare_info(hw, 0, info);
			}

			if (done_skb) {
				/* Remove H/W dma header */
				hdrlen = ieee80211_hdrlen(tr->wh.frame_control);
				memmove(tr->data - hdrlen, &tr->wh, hdrlen);
				skb_pull(done_skb, sizeof(*tr) - hdrlen);
				info->flags &= ~IEEE80211_TX_CTL_AMPDU;
				info->flags |= IEEE80211_TX_STAT_ACK;
				ieee80211_tx_status(hw, done_skb);
			}

			tx_hndl = tx_hndl->pnext;
			tx_desc = tx_hndl->pdesc;
			priv->fw_desc_cnt[num]--;
		}

		desc->pstale_tx_hndl = tx_hndl;
	}
	spin_unlock_bh(&priv->tx_desc_lock);

	if (priv->is_tx_done_schedule) {

		mwl_set_bit(priv, MACREG_A2HRIC_BIT_NUM_TX_DONE, card->iobase1 + MACREG_REG_A2H_INTERRUPT_STATUS_MASK);

		tasklet_schedule(priv->if_ops.ptx_task);
		priv->is_tx_done_schedule = false;
	}

	return;
}

/* XXX: Tx Rate to be indicated to mac80211 - For KF2 PCIe & SDIO,
** driver has no way of knowing the rate at which the pkt was Tx'ed.
** Use hardcoded max value for this
*/
static void mwl_tx_complete_skb(struct sk_buff *done_skb,
		struct _mlan_pcie_data_buf *tx_ring_entry,
		struct ieee80211_hw *hw)
{
	struct mwl_tx_desc *tx_desc;
	struct mwl_priv *priv = hw->priv;
	struct mwl_pcie_card *card = priv->intf;
	struct ieee80211_tx_info *info;
	struct mwl_tx_ctrl *tx_ctrl;
	struct sk_buff_head *amsdu_pkts;
	u32 rate;
	struct mwl_tx_pfu_dma_data *tr =
		(struct mwl_tx_pfu_dma_data *)done_skb->data;
	int hdrlen;

	tx_desc = &tr->tx_desc;

#if 0
wiphy_err(priv->hw->wiphy, "unmap: skb=%p vdata=%p pdata=%p len=%d!\n",
			done_skb,
			done_skb->data,
			le64_to_cpu(tx_ring_entry->paddr),
			le16_to_cpu(tx_ring_entry->len));
#endif

	dma_unmap_single(&card->pdev->dev,
			le64_to_cpu(tx_ring_entry->paddr),
			le16_to_cpu(tx_ring_entry->len),
			DMA_TO_DEVICE);

#if 0
	rate = le32_to_cpu(tx_desc->rate_info);
#endif

	tx_desc->pkt_ptr = 0;
	tx_desc->pkt_len = 0;
	tx_desc->status = cpu_to_le32(EAGLE_TXD_STATUS_IDLE);
	wmb(); /* memory barrier */

	info = IEEE80211_SKB_CB(done_skb);

	if (ieee80211_is_data(tr->wh.frame_control) ||
			ieee80211_is_data_qos(tr->wh.frame_control)) {
		rate = TX_COMP_RATE_FOR_DATA;
		tx_ctrl = (struct mwl_tx_ctrl *)&info->status;
		amsdu_pkts = (struct sk_buff_head *)
			tx_ctrl->amsdu_pkts;
		if (amsdu_pkts) {
			mwl_tx_ack_amsdu_pkts(hw, rate,
					amsdu_pkts);
			dev_kfree_skb_any(done_skb);
			done_skb = NULL;
		} else {
			mwl_tx_prepare_info(hw, rate, info);
		}
	} else {
		mwl_tx_prepare_info(hw, 0, info);
	}

	if (done_skb) {
		/* Remove H/W dma header */
		hdrlen = ieee80211_hdrlen(tr->wh.frame_control);
		memmove(tr->data - hdrlen, &tr->wh, hdrlen);
		skb_pull(done_skb, sizeof(*tr) - hdrlen);
		info->flags &= ~IEEE80211_TX_CTL_AMPDU;
		info->flags |= IEEE80211_TX_STAT_ACK;
		ieee80211_tx_status(hw, done_skb);
	}
}

void mwl_pfu_tx_done(unsigned long data)
{
	struct ieee80211_hw *hw = (struct ieee80211_hw *)data;
	struct mwl_priv *priv = hw->priv;
	struct mwl_pcie_card *card = priv->intf;
	struct sk_buff *done_skb;
	u32 wrdoneidx, rdptr;
	const unsigned int num_tx_buffs = MLAN_MAX_TXRX_BD << PCIE_TX_START_PTR;

	spin_lock_bh(&priv->tx_desc_lock);

	/* Read the TX ring read pointer set by firmware */
	rdptr = readl(card->iobase1 + REG_TXBD_RDPTR);

#if 0
	wiphy_err(hw->wiphy,  "SEND DATA COMP:  rdptr_prev=0x%x, rdptr=0x%x\n",
		priv->txbd_rdptr, rdptr);
#endif

	/* free from previous txbd_rdptr to current txbd_rdptr */
	while (((priv->txbd_rdptr & MLAN_TXBD_MASK)
			   != (rdptr & MLAN_TXBD_MASK))
		 || ((priv->txbd_rdptr & MLAN_BD_FLAG_TX_ROLLOVER_IND)
			   != (rdptr & MLAN_BD_FLAG_TX_ROLLOVER_IND))) {
		wrdoneidx = priv->txbd_rdptr & MLAN_TXBD_MASK;
		wrdoneidx >>= PCIE_TX_START_PTR;

		done_skb = priv->tx_buf_list[wrdoneidx];
		if (done_skb)
			mwl_tx_complete_skb(done_skb, priv->txbd_ring[wrdoneidx], hw);

		priv->tx_buf_list[wrdoneidx] = MNULL;
		priv->txbd_ring[wrdoneidx]->paddr = 0;
		priv->txbd_ring[wrdoneidx]->len = 0;
		priv->txbd_ring[wrdoneidx]->flags = 0;
		priv->txbd_ring[wrdoneidx]->frag_len = 0;
		priv->txbd_ring[wrdoneidx]->offset = 0;
		priv->txbd_rdptr += MLAN_BD_FLAG_TX_START_PTR;
		if ((priv->txbd_rdptr & MLAN_TXBD_MASK) == num_tx_buffs)
			priv->txbd_rdptr = ((priv->txbd_rdptr &
				  MLAN_BD_FLAG_TX_ROLLOVER_IND) ^
				  MLAN_BD_FLAG_TX_ROLLOVER_IND);
	}

	spin_unlock_bh(&priv->tx_desc_lock);
	 if (priv->is_tx_done_schedule) {

		mwl_set_bit(priv, MACREG_A2HRIC_BIT_NUM_TX_DONE, card->iobase1 + MACREG_REG_A2H_INTERRUPT_STATUS_MASK);

		tasklet_schedule(priv->if_ops.ptx_task);
		priv->is_tx_done_schedule = false;
	}
}

void mwl_pcie_tx_done(unsigned long data)
{
	struct ieee80211_hw *hw = (struct ieee80211_hw *)data;
	struct mwl_priv *priv = hw->priv;

	/* Return all skbs to mac80211 */
	if (IS_PFU_ENABLED(priv->chip_type))
		mwl_pfu_tx_done((unsigned long)hw);
	else
		mwl_non_pfu_tx_done((unsigned long)hw);
}


#ifdef BG4CT_A0_WORKAROUND
#define MAX_ISR_ITERATION 2
#endif
static
irqreturn_t mwl_pcie_isr(int irq, void *dev_id)
{
	struct ieee80211_hw *hw = dev_id;
	struct mwl_priv *priv = hw->priv;
	void __iomem *int_status_mask;
	struct mwl_pcie_card *card = (struct mwl_pcie_card *)priv->intf;
	unsigned int int_status;

	if (card->surprise_removed || priv->recovery_in_progress) {
		return IRQ_HANDLED;
	}

	int_status_mask = card->iobase1 +
		MACREG_REG_A2H_INTERRUPT_STATUS_MASK;

	int_status = readl(card->iobase1 +
		MACREG_REG_A2H_INTERRUPT_CAUSE);

	if (int_status == 0x00000000)
		return IRQ_NONE;

	if (int_status == 0xffffffff) {
		wiphy_warn(hw->wiphy, "card unplugged?\n");
	} else {
		writel(~int_status,
		       card->iobase1 + MACREG_REG_A2H_INTERRUPT_CAUSE);

#ifdef CONFIG_DEBUG_FS
		priv->valid_interrupt_cnt++;
#endif

		if (int_status & MACREG_A2HRIC_BIT_TX_DONE) {
			if (!priv->is_tx_done_schedule) {
				mwl_clear_bit(priv, MACREG_A2HRIC_BIT_NUM_TX_DONE, card->iobase1 + MACREG_REG_A2H_INTERRUPT_STATUS_MASK);
				tasklet_schedule(priv->if_ops.ptx_done_task);
				priv->is_tx_done_schedule = true;
			}
		}

		if (int_status & MACREG_A2HRIC_BIT_RX_RDY) {
			if (priv->mac_started) {
				mwl_clear_bit(priv, MACREG_A2HRIC_BIT_NUM_RX_RDY, card->iobase1 + MACREG_REG_A2H_INTERRUPT_STATUS_MASK);
				tasklet_schedule(&priv->rx_task);
			}
		}

		if (int_status & MACREG_A2HRIC_BIT_RADAR_DETECT) {
			wiphy_info(hw->wiphy, "radar detected by firmware\n");
			ieee80211_radar_detected(hw);
		}

		if (int_status & MACREG_A2HRIC_BIT_QUE_EMPTY) {
			if (priv->mac_started) {
				if (time_after(jiffies, (priv->qe_trigger_time + 1))) {
					mwl_clear_bit(priv, MACREG_A2HRIC_BIT_NUM_QUE_EMPTY, card->iobase1 + MACREG_REG_A2H_INTERRUPT_STATUS_MASK);
					if (priv->tx_amsdu_enable) {
						tasklet_schedule(priv->if_ops.pqe_task);
						priv->qe_trigger_num++;
						priv->qe_trigger_time = jiffies;
					}
				}
			}
		}

		if (int_status & MACREG_A2HRIC_BIT_CHAN_SWITCH)
			ieee80211_queue_work(hw, &priv->chnl_switch_handle);

		if (int_status & MACREG_A2HRIC_BA_WATCHDOG)
			ieee80211_queue_work(hw, &priv->watchdog_ba_handle);
	}

	return IRQ_HANDLED;
}

static void mwl_pcie_read_register(struct mwl_priv *priv,
		int index, int reg, u32 *data)
{
	struct mwl_pcie_card *card = priv->intf;

	if (index == 0)
		*data = readl(card->iobase0 + reg);
	else
		*data = readl(card->iobase1 + reg);
}

static void mwl_pcie_write_register(struct mwl_priv *priv,
		int index, int reg, u32 data)
{
	struct mwl_pcie_card *card = (struct mwl_pcie_card *)priv->intf;

	if (index == 0)
		writel(data, card->iobase0 + reg);
	else
		writel(data, card->iobase1 + reg);
}

static int mwl_pcie_register_dev(struct mwl_priv *priv)
{
	struct mwl_pcie_card *card;
	int rc = 0;

	card = (struct mwl_pcie_card *)priv->intf;

	rc = mwl_pcie_intr_init(priv);

	if (rc) {
		return rc;
	}

#ifndef NEW_DP
	priv->if_ops.ptx_task = &card->tx_task;
	tasklet_init(priv->if_ops.ptx_task, (void *)mwl_tx_skbs,
		(unsigned long)priv->hw);
	tasklet_disable(priv->if_ops.ptx_task);

	priv->if_ops.ptx_done_task = &card->tx_done_task;
	tasklet_init(priv->if_ops.ptx_done_task,
		(void *)mwl_pcie_tx_done, (unsigned long)priv->hw);
	tasklet_disable(priv->if_ops.ptx_done_task);

	if (priv->tx_amsdu_enable) {
		priv->if_ops.pqe_task = &card->qe_task;
		tasklet_init(priv->if_ops.pqe_task, (void *)mwl_pcie_tx_flush_amsdu,
			(unsigned long)priv->hw);
		tasklet_disable(priv->if_ops.pqe_task);
	}
#endif

	tasklet_init(&priv->rx_task, (void *)mwl_pcie_rx_recv,
		(unsigned long)priv->hw);
	tasklet_disable(&priv->rx_task);

	return rc;
}

static void mwl_pcie_unregister_dev(struct mwl_priv *priv)
{
	wiphy_warn(priv->hw->wiphy,
			"%s(): tasklet_pending[txd=%d]\n",
			__FUNCTION__,
			priv->is_tx_done_schedule);

#ifndef NEW_DP
	if (priv->if_ops.ptx_task != NULL)
		tasklet_kill(priv->if_ops.ptx_task);

	if (priv->if_ops.ptx_done_task != NULL)
		tasklet_kill(priv->if_ops.ptx_done_task);

	if (priv->if_ops.pqe_task != NULL)
		tasklet_kill(priv->if_ops.pqe_task);
#endif /* NEW_DP */

	tasklet_kill(&priv->rx_task);
}

static void mwl_pcie_tx_flush_amsdu(unsigned long data)
{
	struct ieee80211_hw *hw = (struct ieee80211_hw *)data;
	struct mwl_priv *priv = hw->priv;
	struct mwl_pcie_card *card = (struct mwl_pcie_card *)priv->intf;

	struct mwl_sta *sta_info;
	int i;
	struct mwl_amsdu_frag *amsdu_frag;

// TODO: RR: Take spin_lock_bh() here ??
	spin_lock(&priv->tx_desc_lock);
	spin_lock(&priv->sta_lock);
	list_for_each_entry(sta_info, &priv->sta_list, list) {
		spin_lock(&sta_info->amsdu_lock);
		for (i = 0; i < SYSADPT_TX_WMM_QUEUES; i++) {
			amsdu_frag = &sta_info->amsdu_ctrl.frag[i];
			if (amsdu_frag->num) {
				if (time_after(jiffies,
					       (amsdu_frag->jiffies + 1))) {
					if (mwl_pcie_is_tx_available(priv, i)) {
						/* wiphy_err(priv->hw->wiphy,
						* "%s()\n", __FUNCTION__);
						*/
						mwl_tx_skb(priv, i,
							   amsdu_frag->skb);
						amsdu_frag->num = 0;
						amsdu_frag->cur_pos = NULL;
					}
				}
			}
		}
		spin_unlock(&sta_info->amsdu_lock);
	}
	spin_unlock(&priv->sta_lock);
	spin_unlock(&priv->tx_desc_lock);

	mwl_set_bit(priv, MACREG_A2HRIC_BIT_NUM_QUE_EMPTY, card->iobase1 + MACREG_REG_A2H_INTERRUPT_STATUS_MASK);

}

#ifdef CONFIG_DEBUG_FS
static int mwl_pcie_dbg_info(struct mwl_priv *priv, char *p, int size, int len)
{
	struct mwl_pcie_card *card = (struct mwl_pcie_card *)priv->intf;

	len += scnprintf(p + len, size - len, "irq number: %d\n", priv->irq);
	len += scnprintf(p + len, size - len, "iobase0: %p\n", card->iobase0);
	len += scnprintf(p + len, size - len, "iobase1: %p\n", card->iobase1);

	return len;
}

static int mwl_pcie_debugfs_reg_access(struct mwl_priv *priv, bool write)
{
	struct ieee80211_hw *hw = priv->hw;
	struct mwl_pcie_card *card = (struct mwl_pcie_card *)priv->intf;
	u8 set;
	u32 *addr_val;
	int ret = 0;

	set = write ? WL_SET : WL_GET;

	switch (priv->reg_type) {
	case MWL_ACCESS_ADDR0:
		if (set == WL_GET)
			priv->reg_value = readl(card->iobase0 + priv->reg_offset);
		else
			writel(priv->reg_value, card->iobase0 + priv->reg_offset);
		break;
	case MWL_ACCESS_ADDR1:
		if (set == WL_GET)
			priv->reg_value = readl(card->iobase1 + priv->reg_offset);
		else
			writel(priv->reg_value, card->iobase1 + priv->reg_offset);
		break;
	case MWL_ACCESS_ADDR:
		addr_val = kmalloc(64 * sizeof(u32), GFP_KERNEL);
		if (addr_val) {
			memset(addr_val, 0, 64 * sizeof(u32));
			addr_val[0] = priv->reg_value;
			ret = mwl_fwcmd_get_addr_value(hw, priv->reg_offset, 4, addr_val, set);

			if ((!ret) && (set == WL_GET)) {
				priv->reg_value = addr_val[0];
			}

			kfree(addr_val);
		} else {
			ret = -ENOMEM;
		}
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}
#endif

static int mwl_pcie_wakeup_card(struct mwl_priv *priv)
{
	return 0;
}

static int mwl_pcie_is_deepsleep(struct mwl_priv * priv)
{
	return 0;
}

static void mwl_pcie_enter_deepsleep(struct mwl_priv *priv)
{
	return;
}

/*
 * This function initializes the PCI-E host memory space, WCB rings, etc.,
 * similar to mwl_pcie_init(), but without resetting PCI-E state.
 */
static void mwl_pcie_up_dev(struct mwl_priv *priv)
{
	struct ieee80211_hw *hw = priv->hw;
	struct mwl_pcie_card *card = priv->intf;
	struct pci_dev *pdev = card->pdev;

	wiphy_info(priv->hw->wiphy, "%s: Bringing up adapter...\n", MWL_DRV_NAME);

	/*
	 * Initialize tx/rx buffers, enable bus mastering
	 */
	mwl_tx_init(hw);
	mwl_rx_init(hw);
	pci_set_master(pdev);

	return;
}

/* This function cleans up the PCI-E host memory space. */
static void mwl_pcie_down_dev(struct mwl_priv *priv)
{
	struct ieee80211_hw *hw = priv->hw;
	struct mwl_pcie_card *card = priv->intf;
	struct pci_dev *pdev = card->pdev;

	wiphy_info(priv->hw->wiphy, "%s: Taking down adapter...\n", MWL_DRV_NAME);

	pci_clear_master(pdev);
	mwl_rx_deinit(hw);
	mwl_tx_deinit(hw);
}


static struct mwl_if_ops pcie_ops = {
	.inttf_head_len = INTF_HEADER_LEN,
	//Tasklets are assigned per instance during device registration
	.ptx_task          = NULL,
	.ptx_done_task     = NULL,
	.pqe_task          = NULL,
	.init_if           = mwl_pcie_init,
	.init_if_post      = mwl_pcie_init_post,
	.cleanup_if        = mwl_pcie_cleanup,
	.check_card_status = mwl_pcie_check_card_status,
	.prog_fw           = mwl_pcie_program_firmware,
	.enable_int        = mwl_pcie_enable_int,
	.disable_int       =  mwl_pcie_disable_int,
	.send_cmd          = mwl_pcie_send_command,
	.cmd_resp_wait_completed = mwl_pcie_cmd_resp_wait_completed,
	.register_dev      = mwl_pcie_register_dev,
//	.unregister_dev    = mwl_pcie_unregister_dev,
	.is_tx_available   = mwl_pcie_is_tx_available,
	.host_to_card      = mwl_pcie_host_to_card,
	.read_reg          = mwl_pcie_read_register,
	.write_reg         = mwl_pcie_write_register,
	.tx_done           = mwl_pcie_tx_done,
#ifdef CONFIG_DEBUG_FS
	.dbg_info          = mwl_pcie_dbg_info,
	.dbg_reg_access    = mwl_pcie_debugfs_reg_access,
#endif
	.enter_deepsleep   = mwl_pcie_enter_deepsleep,
	.wakeup_card       = mwl_pcie_wakeup_card,
	.is_deepsleep      = mwl_pcie_is_deepsleep,
	.down_dev          = mwl_pcie_down_dev,
	.up_dev            = mwl_pcie_up_dev,

	.hardware_restart  = mwl_pcie_restart_handler,
};


static int mwl_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	static bool printed_version;
	struct mwl_pcie_card *card;
	int rc = 0;

	if (id->driver_data >= MWLUNKNOWN)
		return -ENODEV;

	if (!printed_version) {
		pr_info("<<%s version %s>>",
			LRD_DESC, LRD_DRV_VERSION);
		printed_version = true;
	}

	card = kzalloc(sizeof(struct mwl_pcie_card), GFP_KERNEL);
	if (!card)
		return -ENOMEM;

	card->chip_type = id->driver_data;
	card->pdev = pdev;
	card->surprise_removed = false;

	memcpy(&pcie_ops.mwl_chip_tbl, &mwl_chip_tbl[card->chip_type],
		sizeof(struct mwl_chip_info));

	/* device tree node parsing and platform specific configuration*/
	if (dev_of_node(&pdev->dev)) {
		rc = mwl_pcie_probe_of(&pdev->dev);
		if (rc)
			goto err_add_card;
	}

	rc = mwl_add_card(card, &pcie_ops, dev_of_node(&pdev->dev));
	if (rc) {
		pr_err("%s: add card failed", MWL_DRV_NAME);
		goto err_add_card;
	}
	return rc;

err_add_card:
	kfree(card);
	return rc;

}

static void mwl_remove(struct pci_dev *pdev)
{
	struct ieee80211_hw *hw = pci_get_drvdata(pdev);
	struct mwl_priv *priv;
	struct mwl_pcie_card *card;

	if (!hw)
		return;

	priv = hw->priv;
	card = (struct mwl_pcie_card *)priv->intf;

	card->surprise_removed = true;

	if (priv->init_complete)
		mwl_wl_deinit(priv);

	mwl_pcie_cleanup(priv);

	mwl_pcie_unregister_dev(priv);

	mwl_ieee80211_free_hw(priv);
	kfree(card);

#if 0
	pci_disable_device(pdev);
	pci_set_drvdata(pdev, NULL);
#endif

}

static int mwl_pcie_restart_handler(struct mwl_priv *priv)
{
	int rc = 0;
	struct mwl_pcie_card *card = (struct mwl_pcie_card *)priv->intf;

	/* We can't afford to wait here; remove() might be waiting on us. If we
	* can't grab the device lock, maybe we'll get another chance later.
	*/
	rc = pci_reset_function(card->pdev);

	if (rc != 0)
		pr_err("%s: PCI reset attempt failed with error %d!", MWL_DRV_NAME, rc);

	return rc;
}


#if 0
static u32 pci_read_mac_reg(struct mwl_priv *priv, u32 offset)
{
	if (priv->chip_type == MWL8964) {
		u32 *addr_val = kmalloc(64 * sizeof(u32), GFP_ATOMIC);
		u32 val;

		if (addr_val) {
			mwl_fwcmd_get_addr_value(priv->hw,
						 0x8000a000 + offset, 4,
						 addr_val, 0);
			val = addr_val[0];
			kfree(addr_val);
			return val;
		}
		return 0;
	} else
		return le32_to_cpu(*(__le32 * __force)
		       (MAC_REG_ADDR_PCI(offset)));
}
#endif

static const struct pci_error_handlers mwl_pcie_err_handler = {
	.reset_prepare = mwl_pcie_reset_prepare,
	.reset_done    = mwl_pcie_reset_done,
};

static struct pci_driver mwl_pci_driver = {
	.name     = MWL_DRV_NAME,
	.id_table = mwl_pci_id_tbl,
	.probe    = mwl_probe,
	.remove   = mwl_remove,

	.err_handler = &mwl_pcie_err_handler,
};

module_pci_driver(mwl_pci_driver);
MODULE_DEVICE_TABLE(pci, mwl_pci_id_tbl);


MODULE_DESCRIPTION(LRD_PCIE_DESC);
MODULE_VERSION(LRD_PCIE_VERSION);
MODULE_AUTHOR(LRD_AUTHOR);
MODULE_LICENSE("GPL v2");

module_param(pcie_intr_mode, int, 0);
MODULE_PARM_DESC(pcie_intr_mode, "0: Legacy, 1: MSI");
