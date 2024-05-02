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

#include "sysadpt.h"
#include "dev.h"
#include "pcie.h"

int wlan_pcie_create_txbd_ring(struct ieee80211_hw *hw)
{
	int ret = 0;
	unsigned int i;
	struct mwl_priv *priv = hw->priv;
	struct mwl_pcie_card *card = priv->intf;

	/*
	 * driver maintaines the write pointer and firmware maintaines the read
	 * pointer.
	 */
	priv->txbd_wrptr = 0;
	priv->txbd_rdptr = 0;

	/* allocate shared memory for the BD ring and divide the same in to
	 * several descriptors
	 */
	priv->txbd_ring_size =
		sizeof(struct _mlan_pcie_data_buf) * MLAN_MAX_TXRX_BD;
	wiphy_info(hw->wiphy, "TX ring: allocating %d bytes\n",
		priv->txbd_ring_size);

	priv->txbd_ring_vbase = dma_alloc_coherent(&card->pdev->dev,
		priv->txbd_ring_size, (dma_addr_t *)&priv->txbd_ring_pbase,
		GFP_ATOMIC);

	if (!priv->txbd_ring_vbase) {
		wiphy_err(hw->wiphy,
			"%s: No free moal_malloc_consistent\n", __func__);
	    return MLAN_STATUS_FAILURE;
	}

	wiphy_info(hw->wiphy,
		"TX ring: - base: %p, pbase: %#x:%x,len: %x\n",
		priv->txbd_ring_vbase,
		(unsigned int)((unsigned long long)priv->txbd_ring_pbase >> 32),
		(unsigned int)priv->txbd_ring_pbase,
		priv->txbd_ring_size);

	for (i = 0; i < MLAN_MAX_TXRX_BD; i++) {
		priv->txbd_ring[i] = (struct _mlan_pcie_data_buf *)
			(priv->txbd_ring_vbase +
				(sizeof(struct _mlan_pcie_data_buf) * i));
		priv->tx_buf_list[i] = MNULL;
		priv->txbd_ring[i]->paddr = 0;
		priv->txbd_ring[i]->len = 0;
		priv->txbd_ring[i]->flags = 0;
		priv->txbd_ring[i]->frag_len = 0;
		priv->txbd_ring[i]->offset = 0;
	}

	return ret;
}

int wlan_pcie_delete_txbd_ring(struct ieee80211_hw *hw)
{
	unsigned int i;
	struct sk_buff *skb;
	struct mwl_tx_desc *tx_desc;
	struct mwl_priv *priv = hw->priv;
	struct mwl_pcie_card *card = priv->intf;

	for (i = 0; i < MLAN_MAX_TXRX_BD; i++) {
		if (priv->tx_buf_list[i]) {
			skb = priv->tx_buf_list[i];
			tx_desc = (struct mwl_tx_desc *)skb->data;

			dma_unmap_single(&card->pdev->dev,
					le32_to_cpu(tx_desc->pkt_ptr),
					le16_to_cpu(tx_desc->pkt_len),
					DMA_TO_DEVICE);
			dev_kfree_skb_any(skb);
		}
		priv->tx_buf_list[i] = MNULL;
		priv->txbd_ring[i]->paddr = 0;
		priv->txbd_ring[i]->len = 0;
		priv->txbd_ring[i]->flags = 0;
		priv->txbd_ring[i]->frag_len = 0;
		priv->txbd_ring[i]->offset = 0;
		priv->txbd_ring[i] = MNULL;
	}

	if (priv->txbd_ring_vbase) {
		dma_free_coherent(&card->pdev->dev,
				priv->txbd_ring_size,
				priv->txbd_ring_vbase,
				priv->txbd_ring_pbase);
	}
	priv->txbd_ring_size = 0;
	priv->txbd_wrptr = 0;
	priv->txbd_rdptr = 0;
	priv->txbd_ring_vbase = MNULL;
	priv->txbd_ring_pbase = 0;

	return MLAN_STATUS_SUCCESS;
}
