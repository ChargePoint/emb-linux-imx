/*
 * Copyright (C) 2006-2017, Marvell International Ltd.
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

/* Description:  This file implements receive related functions. */

#include <linux/etherdevice.h>
#include <linux/skbuff.h>

#include "sysadpt.h"
#include "dev.h"
#include "rx.h"
#include "main.h"


#define DECRYPT_ERR_MASK        0x80
#define GENERAL_DECRYPT_ERR     0xFF
#define TKIP_DECRYPT_MIC_ERR    0x02
#define WEP_DECRYPT_ICV_ERR     0x04
#define TKIP_DECRYPT_ICV_ERR    0x08

#define W8997_RSSI_OFFSET       0

/* Receive rate information constants */
#define RX_RATE_INFO_FORMAT_11A       0
#define RX_RATE_INFO_FORMAT_11B       1
#define RX_RATE_INFO_FORMAT_11N       2
#define RX_RATE_INFO_FORMAT_11AC      4

#define RX_RATE_INFO_HT20             0
#define RX_RATE_INFO_HT40             1
#define RX_RATE_INFO_HT80             2
#define RX_RATE_INFO_HT160            3

#define RX_RATE_INFO_LONG_INTERVAL    0
#define RX_RATE_INFO_SHORT_INTERVAL   1

struct mwl_vif *mwl_find_first_sta(struct mwl_priv *priv);

void mwl_rx_prepare_status(struct mwl_rx_desc *pdesc,
					 struct ieee80211_rx_status *status)
{
	u16 rate, format, nss, bw, gi, rt;

	memset(status, 0, sizeof(*status));

	status->signal = pdesc->rssi - W8997_RSSI_OFFSET;

	rate = le16_to_cpu(pdesc->rate);
	format = rate & MWL_RX_RATE_FORMAT_MASK;
	nss = (rate & MWL_RX_RATE_NSS_MASK) >> MWL_RX_RATE_NSS_SHIFT;
	bw = (rate & MWL_RX_RATE_BW_MASK) >> MWL_RX_RATE_BW_SHIFT;
	gi = (rate & MWL_RX_RATE_GI_MASK) >> MWL_RX_RATE_GI_SHIFT;
	rt = (rate & MWL_RX_RATE_RT_MASK) >> MWL_RX_RATE_RT_SHIFT;

	switch (format) {
	case RX_RATE_INFO_FORMAT_11N:
		status->encoding = RX_ENC_HT;
		if (bw == RX_RATE_INFO_HT40)
			status->bw |= RATE_INFO_BW_40;
		if (gi == RX_RATE_INFO_SHORT_INTERVAL)
			status->enc_flags |= RX_ENC_FLAG_SHORT_GI;
		break;
	case RX_RATE_INFO_FORMAT_11AC:
		status->encoding |= RX_ENC_VHT;
		if (bw == RX_RATE_INFO_HT40)
			status->bw |= RATE_INFO_BW_40;
		if (bw == RX_RATE_INFO_HT80)
			status->bw |= RATE_INFO_BW_80;
		if (bw == RX_RATE_INFO_HT160)
			status->bw |= RATE_INFO_BW_160;
		if (gi == RX_RATE_INFO_SHORT_INTERVAL)
			status->enc_flags |= RX_ENC_FLAG_SHORT_GI;
		status->nss = (nss + 1);
		break;
	}

	// Firmware is encoding legacy rates using an internal low level
	// value instead of the sequential table index value the driver
	// is expecting.

	// Translate the values the firmware is sending to the value the driver was expecting to see here
	switch (format) {
	case RX_RATE_INFO_FORMAT_11B:
		switch(rt) {
		case 10: status->rate_idx = 0; break;
		case  4: status->rate_idx = 1; break;
		case  7: status->rate_idx = 2; break;
		case 14: status->rate_idx = 3; break;
		// Note - Firmware supports 22Mbps and reports it as 12
		// at what should be index  4.  We should never see that
		// and don't have a rate entry for it
		case 12:
		default: status->rate_idx = 0; break;
		};
		break;
	case RX_RATE_INFO_FORMAT_11A:
		// Note - accounting for the additional 22Mbps rate definition
		// in firmware here by removing it from the adjusted firmware value
		switch(rt) {
		case 11: status->rate_idx = 5-1; break;
		case 15: status->rate_idx = 6-1; break;
		case 10: status->rate_idx = 7-1; break;
		case 14: status->rate_idx = 8-1; break;
		case  9: status->rate_idx = 9-1; break;
		case 13: status->rate_idx = 10-1; break;
		case  8: status->rate_idx = 11-1; break;
		case  12: status->rate_idx = 12-1; break;
		// Note - Firmware suports 72Mbps and reports it as 7
		// at what should be index 13.  We should never see that
		// and don't have a rate entry for it
		case 7:
		default: status->rate_idx = 5-1; break;
		};
		break;
	case RX_RATE_INFO_FORMAT_11N:
	case RX_RATE_INFO_FORMAT_11AC:
		// These values are sent as expected
		status->rate_idx = rt;
		break;
	};

	if (pdesc->channel > BAND_24_CHANNEL_NUM) {
		status->band = NL80211_BAND_5GHZ;
		if ((!(status->encoding & RX_ENC_HT)) &&
		    (!(status->encoding & RX_ENC_VHT))) {
			// Translate to 5G rate table index which does not
			// include 802.11b rate entries
			status->rate_idx -= 4;
			if (status->rate_idx >= BAND_50_RATE_NUM)
				status->rate_idx = BAND_50_RATE_NUM - 1;
		}
	} else {
		status->band = NL80211_BAND_2GHZ;
		if ((!(status->encoding & RX_ENC_HT)) &&
		    (!(status->encoding & RX_ENC_VHT))) {
			if (status->rate_idx >= BAND_24_RATE_NUM)
				status->rate_idx = BAND_24_RATE_NUM - 1;
		}
	}

	status->freq = ieee80211_channel_to_frequency(pdesc->channel,
						      status->band);

	/* check if status has a specific error bit (bit 7) set or indicates
	 * a general decrypt error
	 */
	if ((pdesc->status == GENERAL_DECRYPT_ERR) ||
	    (pdesc->status & DECRYPT_ERR_MASK)) {
		/* check if status is not equal to 0xFF
		 * the 0xFF check is for backward compatibility
		 */
		if (pdesc->status != GENERAL_DECRYPT_ERR) {
			if (((pdesc->status & (~DECRYPT_ERR_MASK)) &
			    TKIP_DECRYPT_MIC_ERR) && !((pdesc->status &
			    (WEP_DECRYPT_ICV_ERR | TKIP_DECRYPT_ICV_ERR)))) {
				status->flag |= RX_FLAG_MMIC_ERROR;
			}
		}
	}
}
EXPORT_SYMBOL_GPL(mwl_rx_prepare_status);

static inline unsigned int lrd_elapsed_jiffies_msecs(unsigned long start)
{
	unsigned long end = jiffies;

	if (end >= start)
		return jiffies_to_msecs(end - start);

	return jiffies_to_msecs(end + (ULONG_MAX - start) + 1);
}

void mwl_handle_rx_event(struct ieee80211_hw *hw,
					struct mwl_rx_event_data *rx_evnt)
{
	struct mwl_priv *priv = hw->priv;

	if (rx_evnt->event_id == MWL_RX_EVNT_RADAR_DETECT) {
		wiphy_info(hw->wiphy, "radar detected by firmware\n");
		ieee80211_radar_detected(hw);
	}
	else if (rx_evnt->event_id == MWL_RX_EVENT_LINKLOSS_DETECT) {
		wiphy_info(hw->wiphy, "link loss detected by firmware\n");
	}
	else if (rx_evnt->event_id == MWL_RX_EVENT_REG) {
		wiphy_info(hw->wiphy, "regulatory event %x\n", rx_evnt->event_info);

		cancel_work_sync(&priv->reg.awm);
		mod_timer(&priv->reg.timer_awm, jiffies + msecs_to_jiffies(1));
	}
#ifdef CONFIG_PM
	else if (rx_evnt->event_id == MWL_RX_EVENT_WOW_LINKLOSS_DETECT ||
	         rx_evnt->event_id == MWL_RX_EVENT_WOW_AP_DETECT ||
	         rx_evnt->event_id == MWL_RX_EVENT_WOW_RX_DETECT) {
		/* WOW event */
		priv->wow.results.reason  = rx_evnt->event_id;

		/* ToDo:  Revisit when FW support returing interface */
		priv->wow.results.mwl_vif = mwl_find_first_sta(priv);

		if (priv->wow.state & WOWLAN_STATE_HS_SENT) {
			/* report event after resume notificaiton is sent */
		}
		else if (priv->wow.jiffies && lrd_elapsed_jiffies_msecs(priv->wow.jiffies) < WOWLAN_JIFFIES) {
			priv->wow.jiffies = 0;
			lrd_report_wowlan_wakeup(priv);
		}
	}
#endif
}
EXPORT_SYMBOL_GPL(mwl_handle_rx_event);

void mwl_rx_enable_sta_amsdu(struct mwl_priv *priv,
					   u8 *sta_addr)
{
	struct mwl_sta *sta_info;
	struct ieee80211_sta *sta;

	spin_lock_bh(&priv->sta_lock);
	list_for_each_entry(sta_info, &priv->sta_list, list) {
		sta = container_of((void *)sta_info, struct ieee80211_sta,
				   drv_priv);
		if (ether_addr_equal(sta->addr, sta_addr)) {
			sta_info->is_amsdu_allowed = true;
			break;
		}
	}
	spin_unlock_bh(&priv->sta_lock);
}
EXPORT_SYMBOL_GPL(mwl_rx_enable_sta_amsdu);

struct mwl_vif *mwl_find_first_sta(struct mwl_priv *priv)
{
	struct mwl_vif *mwl_vif;

	spin_lock_bh(&priv->vif_lock);
		list_for_each_entry(mwl_vif, &priv->vif_list, list) {
		if (NL80211_IFTYPE_STATION == mwl_vif->vif->type) {
			spin_unlock_bh(&priv->vif_lock);
			return mwl_vif;
		}
	}

	spin_unlock_bh(&priv->vif_lock);

	return NULL;
}

struct mwl_vif *mwl_rx_find_vif_bss(struct mwl_priv *priv,
						  u8 *bssid)
{
	struct mwl_vif *mwl_vif;

	spin_lock_bh(&priv->vif_lock);
	list_for_each_entry(mwl_vif, &priv->vif_list, list) {
		if (ether_addr_equal(bssid, mwl_vif->bssid)) {
			spin_unlock_bh(&priv->vif_lock);
			return mwl_vif;
		}
	}
	spin_unlock_bh(&priv->vif_lock);

	return NULL;
}
EXPORT_SYMBOL_GPL(mwl_rx_find_vif_bss);

void mwl_rx_remove_dma_header(struct sk_buff *skb, __le16 qos)
{
	struct mwl_dma_data *tr;
	int hdrlen;

	tr = (struct mwl_dma_data *)skb->data;
	hdrlen = ieee80211_hdrlen(tr->wh.frame_control);

	if (hdrlen != sizeof(tr->wh)) {
		if (ieee80211_is_data_qos(tr->wh.frame_control)) {
			memmove(tr->data - hdrlen, &tr->wh, hdrlen - 2);
			*((__le16 *)(tr->data - 2)) = qos;
		} else {
			memmove(tr->data - hdrlen, &tr->wh, hdrlen);
		}
	}

	if (hdrlen != sizeof(*tr))
		skb_pull(skb, sizeof(*tr) - hdrlen);
}
EXPORT_SYMBOL_GPL(mwl_rx_remove_dma_header);

void mwl_rx_defered_handler(struct work_struct *work)
{
	struct ieee80211_hw *hw;
	struct mwl_priv *priv = container_of(work,
			struct mwl_priv, rx_defer_work);
	struct sk_buff *rx_skb;

	hw = priv->hw;

	priv->is_rx_defer_schedule = false;

	while ((rx_skb = skb_dequeue(&priv->rx_defer_skb_q))) {
		struct ieee80211_hdr *wh;
		wh = (struct ieee80211_hdr *)rx_skb->data;

		wiphy_err(hw->wiphy, "%s(): mgmt=%d stype=%d\n",
			__FUNCTION__,
			ieee80211_is_mgmt(wh->frame_control),
			(wh->frame_control & IEEE80211_FCTL_STYPE));

		/* TODO: Add defered processing code here */
		kfree_skb(rx_skb);
	}
}

inline bool mwl_rx_needs_defered_processing(struct sk_buff *rx_skb)
{
	/* TODO: Choose conditions for selecting defered pkts here */
	return 0;
}

#if 1
unsigned int dbgRxPrbResp, dbgRxBcn;
#endif

void mwl_rx_upload_pkt(struct ieee80211_hw *hw,
		struct sk_buff *rx_skb)
{
	struct ieee80211_hdr *wh;
	struct mwl_priv *priv = hw->priv;
	struct sk_buff *skb_save;

	wh = (struct ieee80211_hdr *)rx_skb->data;

	// We've received at least one packet
	// Clear the scan count to prevent recovery logic from kicking in
	if (atomic_read(&priv->null_scan_count)) {
		atomic_set(&priv->null_scan_count, 0);
	}

	if (unlikely(ieee80211_is_mgmt(wh->frame_control)) &&
		mwl_rx_needs_defered_processing(rx_skb) &&
		((skb_save = skb_copy(rx_skb, GFP_ATOMIC)) != NULL)){
		skb_queue_tail(&priv->rx_defer_skb_q, skb_save);

		if (!priv->is_rx_defer_schedule) {
			priv->is_rx_defer_schedule = true;
			queue_work(priv->rx_defer_workq, &priv->rx_defer_work);
		}
	}

#if 1
if (ieee80211_is_mgmt(wh->frame_control)) {

	if (ieee80211_is_probe_resp(wh->frame_control)) {
		dbgRxPrbResp++;
	} else if (ieee80211_is_beacon(wh->frame_control)) {
		dbgRxBcn++;
	}
}
#endif

	/* Upload pkts to mac80211 */
	ieee80211_rx(hw, rx_skb);
}
EXPORT_SYMBOL_GPL(mwl_rx_upload_pkt);
