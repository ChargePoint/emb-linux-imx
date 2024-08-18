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

/* Description:  This file implements debug fs related functions. */

#include <linux/debugfs.h>

#include "sysadpt.h"
#include "dev.h"
#include "hostcmd.h"
#include "fwcmd.h"
#include "thermal.h"
#include "debugfs.h"

#include "pcie.h"
#include "sdio.h"

#define MWLWIFI_DEBUGFS_ADD_FILE(name) do { \
	if (!debugfs_create_file(#name, 0644, priv->debugfs_phy, \
				 priv, &mwl_debugfs_##name##_fops)) \
		return; \
} while (0)

#define MWLWIFI_DEBUGFS_FILE_OPS(name) \
static const struct file_operations mwl_debugfs_##name##_fops = { \
	.read = mwl_debugfs_##name##_read, \
	.write = mwl_debugfs_##name##_write, \
	.open = simple_open, \
}

#define MWLWIFI_DEBUGFS_FILE_READ_OPS(name) \
static const struct file_operations mwl_debugfs_##name##_fops = { \
	.read = mwl_debugfs_##name##_read, \
	.open = simple_open, \
}

#define MWLWIFI_DEBUGFS_FILE_WRITE_OPS(name) \
static const struct file_operations mwl_debugfs_##name##_fops = { \
	.write = mwl_debugfs_##name##_write, \
	.open = simple_open, \
}

static const char chipname[MWLUNKNOWN][8] = {
	"88W8864",
	"88W8897",
	"88W8964",
	"88W8997",
};

static const char chipbus[5][5] = {
	"?",
	"?",
	"SDIO",
	"PCIE",
	"USB"
};

static void dump_data(char *p, int size, int *len, u8 *data,
		      int data_len, char *title)
{
	int cur_byte = 0;
	int i;

	*len += scnprintf(p + *len, size - *len, "%s\n", title);

	for (cur_byte = 0; cur_byte < data_len; cur_byte += 8) {
		if ((cur_byte + 8) < data_len) {
			for (i = 0; i < 8; i++) {
				*len += scnprintf(p + *len, size - *len, "0x%02x ", *(data + cur_byte + i));
			}
			*len += scnprintf(p + *len, size - *len, "\n");
		} else {
			for (i = 0; i < (data_len - cur_byte); i++) {
				*len += scnprintf(p + *len, size - *len, "0x%02x ", *(data + cur_byte + i));
			}
			*len += scnprintf(p + *len, size - *len, "\n");
			break;
		}
	}
}

static ssize_t mwl_debugfs_info_read(struct file *file, char __user *ubuf,
				     size_t count, loff_t *ppos)
{
	struct mwl_priv *priv = (struct mwl_priv *)file->private_data;
	unsigned long    page = get_zeroed_page(GFP_KERNEL);
	char     *p = (char *)page;
	int       i = 0;
	int     len = 0;
	int    size = PAGE_SIZE;
	ssize_t ret = 0;

	if (!p) {
		return -ENOMEM;
	}

	len += scnprintf(p + len, size - len, "\n");
	len += scnprintf(p + len, size - len, "Driver name: %s\n", MWL_DRV_NAME);
	len += scnprintf(p + len, size - len, "Chip type: %s-%s\n", chipname[priv->chip_type], chipbus[priv->host_if]);
	len += scnprintf(p + len, size - len, "HW  version: %X\n", priv->hw_data.hw_version);
	len += scnprintf(p + len, size - len, "DRV version: %s\n", LRD_BLD_VERSION);
	len += scnprintf(p + len, size - len, "FW  version: %d.%d.%d.%d\n",
											((priv->hw_data.fw_release_num >> 24) & 0xff),
											((priv->hw_data.fw_release_num >> 16) & 0xff),
											((priv->hw_data.fw_release_num >> 8) & 0xff),
											((priv->hw_data.fw_release_num >> 0) & 0xff));
	len += scnprintf(p + len, size - len, "OTP version: %d\n", priv->radio_caps.version);
	len += scnprintf(p + len, size - len, "OTP region: 0x%x\n", priv->reg.otp.region);
	len += scnprintf(p + len, size - len, "OTP cap: 0x%x\n", priv->radio_caps.capability);
	len += scnprintf(p + len, size - len, "OTP mac: %d\n", priv->radio_caps.num_mac);
	len += scnprintf(p + len, size - len, "Radio Type: %s%s%s\n",
											(priv->radio_caps.capability & LRD_CAP_SU60) ? "SU":"ST",
											(priv->radio_caps.capability & LRD_CAP_440)  ? "-440":"",
											(priv->radio_caps.capability & LRD_CAP_SOM8MP)  ? "-SOM8MP":"");
	len += scnprintf(p + len, size - len, "BT Offset  : %s\n",
											(priv->radio_caps.capability & LRD_CAP_BT_SEQ) ? "1":
											(priv->radio_caps.capability & LRD_CAP_BT_DUP) ? "0":"Default");
	len += scnprintf(p + len, size - len, "MAC address: %pM\n", priv->hw_data.mac_addr);
	len += scnprintf(p + len, size - len, "Region code: 0x%02x (0x%02x)\n", priv->reg.cc.region, priv->reg.otp.region);
	len += scnprintf(p + len, size - len, "Country code: '%c%c' ('%c%c')\n",
												priv->reg.cc.alpha2[0],priv->reg.cc.alpha2[1],
												priv->reg.otp.alpha2[0],priv->reg.otp.alpha2[1]);
	len += scnprintf(p + len, size - len, "Radio: %s\n", priv->radio_on ? "enable" : "disable");

	len += scnprintf(p + len, size - len, "2g: %s\n", priv->disable_2g ? "disable" : "enable");
	len += scnprintf(p + len, size - len, "5g: %s\n", priv->disable_5g ? "disable" : "enable");
	len += scnprintf(p + len, size - len, "TX antenna: %d\n", priv->ant_tx_num);
	len += scnprintf(p + len, size - len, "RX antenna: %d\n", priv->ant_rx_num);
	len += scnprintf(p + len, size - len, "Antenna Gain: 0x%08x\n", priv->ant_gain_adjust);
	len += scnprintf(p + len, size - len, "Deep Sleep: 0x%x\n", priv->ds_enable);

	//Card Specific debug info
	if (priv->if_ops.dbg_info != NULL) {
		len += priv->if_ops.dbg_info(priv , p, size, len);
	}

	len += scnprintf(p + len, size - len, "TX limit: %d\n", priv->txq_limit);
	len += scnprintf(p + len, size - len, "RX limit: %d\n", priv->recv_limit);
	len += scnprintf(p + len, size - len, "AP  macid support: %08x\n",  priv->ap_macids_supported);
	len += scnprintf(p + len, size - len, "STA macid support: %08x\n", priv->sta_macids_supported);
	len += scnprintf(p + len, size - len, "macid used: %08x\n", priv->macids_used);
	len += scnprintf(p + len, size - len, "qe trigger number: %d\n", priv->qe_trigger_num);

	len += scnprintf(p + len, size - len,"OS TxQ status = [ %d:%d %d:%d %d:%d %d:%d ]\n",
			ieee80211_queue_stopped(priv->hw, 0), skb_queue_len(&priv->txq[0]),
			ieee80211_queue_stopped(priv->hw, 1), skb_queue_len(&priv->txq[1]),
			ieee80211_queue_stopped(priv->hw, 2), skb_queue_len(&priv->txq[2]),
			ieee80211_queue_stopped(priv->hw, 3), skb_queue_len(&priv->txq[3]));

	len += scnprintf(p + len, size - len,"tx_mgmt_cnt=%lu, tx_data_cnt=%lu\n", priv->tx_mgmt_cnt, priv->tx_data_cnt);
	if (priv->host_if == MWL_IF_PCIE) {
		len += scnprintf(p + len, size - len,"num_valid_interrupts = %lu\n", priv->valid_interrupt_cnt);
	}
	else  if (priv->host_if == MWL_IF_SDIO) {
		struct mwl_sdio_card *card = priv->intf;
		len += scnprintf(p + len, size - len, "tx_pkt_unaligned_cnt: %d\n",card->tx_pkt_unaligned_cnt);
	}

	// Dump PFU regs
	if (((priv->host_if == MWL_IF_PCIE) && IS_PFU_ENABLED(priv->chip_type))) {
		struct mwl_pcie_card *card = priv->intf;

		len += scnprintf(p + len, size - len, "PCIe Tx Rd/Wr Ptr (Sw) = (0x%x / 0x%x)\n",
			priv->txbd_rdptr, priv->txbd_wrptr);

		len += scnprintf(p + len, size - len, "PCIe Tx Rd/Wr Ptr (Hw) = (0x%x / 0x%x)\n",
			readl(card->iobase1 + REG_TXBD_RDPTR),
			readl(card->iobase1 + REG_TXBD_WRPTR));

		len += scnprintf(p + len, size - len, "PCIe IntMask = 0x%x\n",
			readl(card->iobase1 +
			MACREG_REG_A2H_INTERRUPT_STATUS_MASK));
	}

	// Dump WMM regs
	for (i=0; i<7; i++){
		u32 cwmin, cwmax, txop, aifsn;
		if(mwl_fwcmd_reg_mac(priv->hw, WL_GET, MAC_REG_CW0_MIN+(i*8), &cwmin))
			cwmin = 0xdead;
		if(mwl_fwcmd_reg_mac(priv->hw, WL_GET, MAC_REG_CW0_MAX+(i*8), &cwmax))
			cwmax = 0xdead;
		if(mwl_fwcmd_reg_mac(priv->hw, WL_GET, MAC_REG_TXOP0+(i*4), &txop))
			cwmax = 0xdead;
		if(mwl_fwcmd_reg_mac(priv->hw, WL_GET, MAC_REG_AIFSN0+(i*4), &aifsn))
			cwmax = 0xdead;
		len += scnprintf(p + len, size - len,"TCQ%d : cwmin=%d cwmax=%d txop=%d aifsn=%d\n", i, cwmin, cwmax, txop, aifsn);
	}

	len += scnprintf(p + len, size - len, "\n");

	ret = simple_read_from_buffer(ubuf, count, ppos, p, len);
	free_page(page);

	return ret;
}

static ssize_t mwl_debugfs_vif_read(struct file *file, char __user *ubuf,
				    size_t count, loff_t *ppos)
{
	struct mwl_priv *priv = (struct mwl_priv *)file->private_data;
	struct mwl_vif       *mwl_vif;
	struct ieee80211_vif *vif;
	char ssid[IEEE80211_MAX_SSID_LEN + 1];

	unsigned long page = get_zeroed_page(GFP_KERNEL);
	char     *p = (char *)page;
	int     len = 0;
	int    size = PAGE_SIZE;
	ssize_t ret = 0;

	if (!p) {
		return -ENOMEM;
	}

	len += scnprintf(p + len, size - len, "\n");

	spin_lock_bh(&priv->vif_lock);
	list_for_each_entry(mwl_vif, &priv->vif_list, list) {
		vif = container_of((void *)mwl_vif, struct ieee80211_vif, drv_priv);
		len += scnprintf(p + len, size - len, "macid: %d\n", mwl_vif->macid);
		switch (vif->type) {
		case NL80211_IFTYPE_AP:
			len += scnprintf(p + len, size - len, "type: ap\n");
			memcpy(ssid, vif->cfg.ssid, vif->cfg.ssid_len);
			ssid[vif->cfg.ssid_len] = 0;
			len += scnprintf(p + len, size - len, "ssid: %s\n", ssid);
			len += scnprintf(p + len, size - len, "mac address: %pM\n", mwl_vif->bssid);
			break;
		case NL80211_IFTYPE_STATION:
			len += scnprintf(p + len, size - len, "type: sta\n");
			len += scnprintf(p + len, size - len, "mac address: %pM\n", mwl_vif->sta_mac);
			break;
		default:
			len += scnprintf(p + len, size - len, "type: unknown\n");
			break;
		}
		len += scnprintf(p + len, size - len, "hw_crypto_enabled: %s\n", mwl_vif->is_hw_crypto_enabled ? "true" : "false");
		len += scnprintf(p + len, size - len, "key idx: %d\n",  mwl_vif->keyidx);
		len += scnprintf(p + len, size - len, "IV: %08x%04x\n", mwl_vif->iv32, mwl_vif->iv16);
		dump_data(p, size, &len, mwl_vif->beacon_info.ie_wmm_ptr,   mwl_vif->beacon_info.ie_wmm_len, "WMM:");
		dump_data(p, size, &len, mwl_vif->beacon_info.ie_rsn_ptr,   mwl_vif->beacon_info.ie_rsn_len, "RSN:");
		dump_data(p, size, &len, mwl_vif->beacon_info.ie_rsn48_ptr, mwl_vif->beacon_info.ie_rsn48_len, "RSN48:");
		dump_data(p, size, &len, mwl_vif->beacon_info.ie_ht_ptr,    mwl_vif->beacon_info.ie_ht_len, "HT:");
		dump_data(p, size, &len, mwl_vif->beacon_info.ie_vht_ptr,   mwl_vif->beacon_info.ie_vht_len, "VHT:");
		dump_data(p, size, &len, mwl_vif->beacon_info.ie_11k_ptr,   mwl_vif->beacon_info.ie_11k_len, "11K:");
		len += scnprintf(p + len, size - len, "\n");
	}
	spin_unlock_bh(&priv->vif_lock);

	ret = simple_read_from_buffer(ubuf, count, ppos, p, len);
	free_page(page);

	return ret;
}

static ssize_t mwl_debugfs_sta_read(struct file *file, char __user *ubuf,
				    size_t count, loff_t *ppos)
{
	struct mwl_priv *priv = (struct mwl_priv *)file->private_data;
	struct mwl_sta       *sta_info;
	struct ieee80211_sta *sta;

	unsigned long page = get_zeroed_page(GFP_KERNEL);
	char     *p = (char *)page;
	int     len = 0;
	int    size = PAGE_SIZE;
	ssize_t ret = 0;

	if (!p) {
		return -ENOMEM;
	}

	len += scnprintf(p + len, size - len, "\n");

	spin_lock_bh(&priv->sta_lock);
	list_for_each_entry(sta_info, &priv->sta_list, list) {
		sta = container_of((void *)sta_info, struct ieee80211_sta, drv_priv);
		len += scnprintf(p + len, size - len, "mac address: %pM\n", sta->addr);
		len += scnprintf(p + len, size - len, "aid: %u\n",   sta->aid);
		len += scnprintf(p + len, size - len, "ampdu: %s\n", sta_info->is_ampdu_allowed ? "true" : "false");
		len += scnprintf(p + len, size - len, "amsdu: %s\n", sta_info->is_amsdu_allowed ? "true" : "false");
		if (sta_info->is_amsdu_allowed) {
			len += scnprintf(p + len, size - len, "amsdu cap: 0x%02x\n", sta_info->amsdu_ctrl.cap);
		}
		if (sta->deflink.ht_cap.ht_supported) {
			len += scnprintf(p + len, size - len, "ht_cap: 0x%04x, ampdu: %02x, %02x\n",
					 sta->deflink.ht_cap.cap,
					 sta->deflink.ht_cap.ampdu_factor,
					 sta->deflink.ht_cap.ampdu_density);
			len += scnprintf(p + len, size - len, "rx_mask: 0x%02x, %02x, %02x, %02x\n",
					 sta->deflink.ht_cap.mcs.rx_mask[0],
					 sta->deflink.ht_cap.mcs.rx_mask[1],
					 sta->deflink.ht_cap.mcs.rx_mask[2],
					 sta->deflink.ht_cap.mcs.rx_mask[3]);
		}
		if (sta->deflink.vht_cap.vht_supported) {
			len += scnprintf(p + len, size - len, "vht_cap: 0x%08x, mcs: %02x, %02x\n",
					 sta->deflink.vht_cap.cap,
					 sta->deflink.vht_cap.vht_mcs.rx_mcs_map,
					 sta->deflink.vht_cap.vht_mcs.tx_mcs_map);
		}
		len += scnprintf(p + len, size - len, "rx_bw: %d, rx_nss: %d\n",    sta->deflink.bandwidth, sta->deflink.rx_nss);
		len += scnprintf(p + len, size - len, "tdls:  %d, tdls_init: %d\n", sta->tdls, sta->tdls_initiator);
		len += scnprintf(p + len, size - len, "wme:   %d, mfp: %d\n",       sta->wme, sta->mfp);
		len += scnprintf(p + len, size - len, "IV: %08x%04x\n",             sta_info->iv32, sta_info->iv16);
		len += scnprintf(p + len, size - len, "\n");
	}
	spin_unlock_bh(&priv->sta_lock);

	ret = simple_read_from_buffer(ubuf, count, ppos, p, len);
	free_page(page);

	return ret;
}

static ssize_t mwl_debugfs_ampdu_read(struct file *file, char __user *ubuf,
				      size_t count, loff_t *ppos)
{
	struct mwl_priv *priv = (struct mwl_priv *)file->private_data;
	struct mwl_ampdu_stream *stream;
	struct mwl_sta       *sta_info;
	struct ieee80211_sta *sta;

	unsigned long page = get_zeroed_page(GFP_KERNEL);
	char     *p = (char *)page;
	int       i = 0;
	int     len = 0;
	int    size = PAGE_SIZE;
	ssize_t ret = 0;

	if (!p) {
		return -ENOMEM;
	}

	len += scnprintf(p + len, size - len, "\n");
	spin_lock_bh(&priv->stream_lock);
	for (i = 0; i < SYSADPT_TX_AMPDU_QUEUES; i++) {
		stream = &priv->ampdu[i];
		if (!stream->state)
			continue;
		len += scnprintf(p + len, size - len, "stream: %d\n", i);
		len += scnprintf(p + len, size - len, "idx: %u\n",   stream->idx);
		len += scnprintf(p + len, size - len, "state: %u\n", stream->state);
		if (stream->sta) {
			len += scnprintf(p + len, size - len, "mac address: %pM\n", stream->sta->addr);
			len += scnprintf(p + len, size - len, "tid: %u\n", stream->tid);
		}
	}
	spin_unlock_bh(&priv->stream_lock);

	spin_lock_bh(&priv->sta_lock);
	list_for_each_entry(sta_info, &priv->sta_list, list) {
		for (i = 0; i < MWL_MAX_TID; i++) {
			if (sta_info->check_ba_failed[i]) {
				sta = container_of((void *)sta_info, struct ieee80211_sta, drv_priv);
				len += scnprintf(p + len, size - len, "%pM(%d): %d\n", sta->addr, i, sta_info->check_ba_failed[i]);
			}
		}
	}
	spin_unlock_bh(&priv->sta_lock);

	len += scnprintf(p + len, size - len, "\n");

	ret = simple_read_from_buffer(ubuf, count, ppos, p, len);
	free_page(page);

	return ret;
}

static ssize_t mwl_debugfs_dfs_channel_read(struct file *file,
					    char __user *ubuf,
					    size_t count, loff_t *ppos)
{
	struct mwl_priv *priv = (struct mwl_priv *)file->private_data;
	struct ieee80211_supported_band *sband;
	struct ieee80211_channel        *channel;

	unsigned long page = get_zeroed_page(GFP_KERNEL);
	char     *p = (char *)page;
	int       i = 0;
	int     len = 0;
	int    size = PAGE_SIZE;
	ssize_t ret = 0;

	if (!p) {
		return -ENOMEM;
	}

	sband = priv->hw->wiphy->bands[NL80211_BAND_5GHZ];
	if (!sband) {
		ret = -EINVAL;
		goto err;
	}

	len += scnprintf(p + len, size - len, "\n");
	for (i = 0; i < sband->n_channels; i++) {
		channel = &sband->channels[i];
		if (channel->flags & IEEE80211_CHAN_RADAR) {
			len += scnprintf(p + len, size - len, "%d(%d): flags: %08x dfs_state: %d\n",
					 channel->hw_value,
					 channel->center_freq,
					 channel->flags, channel->dfs_state);
			len += scnprintf(p + len, size - len, "cac timer: %d ms\n", channel->dfs_cac_ms);
		}
	}
	len += scnprintf(p + len, size - len, "\n");

	ret = simple_read_from_buffer(ubuf, count, ppos, p, len);

err:
	free_page(page);
	return ret;
}

static ssize_t mwl_debugfs_dfs_channel_write(struct file *file,
					     const char __user *ubuf,
					     size_t count, loff_t *ppos)
{
	struct mwl_priv *priv = (struct mwl_priv *)file->private_data;
	struct ieee80211_supported_band *sband;
	struct ieee80211_channel       *channel;

	unsigned long page = get_zeroed_page(GFP_KERNEL);
	char   *p = (char *)page;
	int     i = 0;
	int dfs_state = 0;
	int cac_time  = -1;
	ssize_t ret = 0;

	if (!p) {
		return -ENOMEM;
	}

	sband = priv->hw->wiphy->bands[NL80211_BAND_5GHZ];
	if (!sband) {
		ret = -EINVAL;
		goto err;
	}

	if (copy_from_user(p, ubuf, min_t(size_t, count, PAGE_SIZE - 1))) {
		ret = -EFAULT;
		goto err;
	}

	ret = sscanf(p, "%d %d", &dfs_state, &cac_time);

	if ((ret < 1) || (ret > 2)) {
		ret = -EINVAL;
		goto err;
	}

	for (i = 0; i < sband->n_channels; i++) {
		channel = &sband->channels[i];
		if (channel->flags & IEEE80211_CHAN_RADAR) {
			channel->dfs_state = dfs_state;
			if (cac_time != -1) {
				channel->dfs_cac_ms = cac_time * 1000;
			}
		}
	}
	ret = count;

err:
	free_page(page);
	return ret;
}

static ssize_t mwl_debugfs_dfs_radar_read(struct file *file, char __user *ubuf,
					  size_t count, loff_t *ppos)
{
	struct mwl_priv *priv = (struct mwl_priv *)file->private_data;
	unsigned long    page = get_zeroed_page(GFP_KERNEL);
	char  *p = (char *)page;
	int  len = 0;
	int size = PAGE_SIZE;
	ssize_t ret;

	if (!p) {
		return -ENOMEM;
	}

	len += scnprintf(p + len, size - len, "\n");
	len += scnprintf(p + len, size - len, "csa_active: %d\n", priv->csa_active);
	len += scnprintf(p + len, size - len, "dfs_region: %d\n", priv->reg.dfs_region);
	len += scnprintf(p + len, size - len, "pw_filter: %d\n",     priv->dfs_pw_filter);
	len += scnprintf(p + len, size - len, "min_num_radar: %d\n", priv->dfs_min_num_radar);
	len += scnprintf(p + len, size - len, "min_pri_count: %d\n", priv->dfs_min_pri_count);
	len += scnprintf(p + len, size - len, "chirp_count_min: %d\n",     priv->dfs_chirp_count_min);
	len += scnprintf(p + len, size - len, "chirp_time_interval: %d\n", priv->dfs_chirp_time_interval);
	len += scnprintf(p + len, size - len, "\n");

	ret = simple_read_from_buffer(ubuf, count, ppos, p, len);
	free_page(page);

	return ret;
}

static ssize_t mwl_debugfs_dfs_radar_write(struct file *file,
					   const char __user *ubuf,
					   size_t count, loff_t *ppos)
{
	struct mwl_priv *priv = (struct mwl_priv *)file->private_data;

	wiphy_info(priv->hw->wiphy, "simulate radar detected\n");
	ieee80211_radar_detected(priv->hw);

	return count;
}

static ssize_t mwl_debugfs_thermal_read(struct file *file,
					char __user *ubuf,
					size_t count, loff_t *ppos)
{
	struct mwl_priv *priv = (struct mwl_priv *)file->private_data;
	unsigned long    page = get_zeroed_page(GFP_KERNEL);
	char     *p = (char *)page;
	int     len = 0;
	int    size = PAGE_SIZE;
	ssize_t ret = 0;

	if (!p) {
		return -ENOMEM;
	}

	mwl_fwcmd_get_temp(priv->hw, &priv->temperature);

	len += scnprintf(p + len, size - len, "\n");
	len += scnprintf(p + len, size - len, "quiet period: %d\n",   priv->quiet_period);
	len += scnprintf(p + len, size - len, "throttle state: %d\n", priv->throttle_state);
	len += scnprintf(p + len, size - len, "temperature: %d\n",    priv->temperature);
	len += scnprintf(p + len, size - len, "\n");

	ret = simple_read_from_buffer(ubuf, count, ppos, p, len);
	free_page(page);

	return ret;
}

static ssize_t mwl_debugfs_thermal_write(struct file *file,
					 const char __user *ubuf,
					 size_t count, loff_t *ppos)
{
	struct mwl_priv *priv = (struct mwl_priv *)file->private_data;
	unsigned long    page = get_zeroed_page(GFP_KERNEL);
	char   *p = (char *)page;
	int    throttle_state = 0;
	ssize_t ret = 0;

	if (!p) {
		return -ENOMEM;
	}

	if (copy_from_user(p, ubuf, min_t(size_t, count, PAGE_SIZE - 1))) {
		ret = -EFAULT;
		goto err;
	}

	if (kstrtoint(p, 0, &throttle_state)) {
		ret = -EINVAL;
		goto err;
	}

	if (throttle_state > SYSADPT_THERMAL_THROTTLE_MAX) {
		wiphy_warn(priv->hw->wiphy,"throttle state %d is exceeding the limit %d\n",
			   throttle_state, SYSADPT_THERMAL_THROTTLE_MAX);
		ret = -EINVAL;
		goto err;
	}

	priv->throttle_state = throttle_state;
	mwl_thermal_set_throttling(priv);
	ret = count;

err:
	free_page(page);
	return ret;
}

static int mwl_debugfs_reg_access(struct mwl_priv *priv, bool write)
{
	struct ieee80211_hw *hw = priv->hw;
	u8  set = 0;
	int ret = -EINVAL;

	set = write ? WL_SET : WL_GET;

	switch (priv->reg_type) {
	case MWL_ACCESS_MAC:
		ret = mwl_fwcmd_reg_mac(hw, set, priv->reg_offset, &priv->reg_value);
		break;
	case MWL_ACCESS_RF:
		ret = mwl_fwcmd_reg_rf(hw, set, priv->reg_offset,  &priv->reg_value);
		break;
	case MWL_ACCESS_BBP:
		ret = mwl_fwcmd_reg_bb(hw, set, priv->reg_offset,  &priv->reg_value);
		break;
	case MWL_ACCESS_CAU:
		ret = mwl_fwcmd_reg_cau(hw, set, priv->reg_offset, &priv->reg_value);
		break;
	default:
		// Interface specific
		if (priv->if_ops.dbg_reg_access != NULL) {
			ret = priv->if_ops.dbg_reg_access(priv, write);
		}
	}

	return ret;
}

static ssize_t mwl_debugfs_regrdwr_read(struct file *file, char __user *ubuf,
					size_t count, loff_t *ppos)
{
	struct mwl_priv *priv = (struct mwl_priv *)file->private_data;
	unsigned long page;
	char *p;
	int  len = 0;
	int size = PAGE_SIZE;
	ssize_t ret;

	if (*ppos) {
		return len;
	}

	page = get_zeroed_page(GFP_KERNEL);
	if (!page) {
		return -ENOMEM;
	}
	p = (char *)page;

	if (!priv->reg_type) {
		/* No command has been given */
		len += scnprintf(p + len, size - len, "0");
		ret = -EINVAL;
		goto none;
	}

	/* Get command has been given */
	ret = mwl_debugfs_reg_access(priv, false);

//done:
	if (!ret)
		len += scnprintf(p + len, size - len, "%u 0x%08x 0x%08x\n",
				 priv->reg_type, priv->reg_offset, priv->reg_value);
	else
		len += scnprintf(p + len, size - len, "error: %lu(%u 0x%08x 0x%08x)\n",
				 (long unsigned int)ret, priv->reg_type, priv->reg_offset, priv->reg_value);

	ret = simple_read_from_buffer(ubuf, count, ppos, p, len);

none:
	free_page(page);
	return ret;
}

static ssize_t mwl_debugfs_regrdwr_write(struct file *file,
					 const char __user *ubuf,
					 size_t count, loff_t *ppos)
{
	struct mwl_priv *priv = (struct mwl_priv *)file->private_data;
	unsigned long    page = get_zeroed_page(GFP_KERNEL);
	char *p   = (char *)page;
	u32 reg_type   = 0;
	u32 reg_offset = 0;
	u32 reg_value  = UINT_MAX;
	ssize_t ret = 0;

	if (!p) {
		return -ENOMEM;
	}

	if (copy_from_user(p, ubuf, min_t(size_t, count, PAGE_SIZE - 1))) {
		ret = -EFAULT;
		goto err;
	}

	ret = sscanf(p, "%u %x %x", &reg_type, &reg_offset, &reg_value);

	if ((ret != 2) && (ret != 3)) {
		ret = -EINVAL;
		goto err;
	}

	if (!reg_type) {
		ret = -EINVAL;
		goto err;
	} else {
		priv->reg_type   = reg_type;
		priv->reg_offset = reg_offset;

		if (ret == 3) {
			priv->reg_value = reg_value;
			ret = mwl_debugfs_reg_access(priv, true);
			if (ret) {
				ret = -EINVAL;
				goto err;
			}
		}

		ret = count;
	}

err:
	free_page(page);
	return ret;
}

static ssize_t mwl_debugfs_otp_data_read(struct file *file,
		char __user *ubuf,
		size_t count, loff_t *ppos)
{
	struct mwl_priv *priv = (struct mwl_priv *)file->private_data;

	return simple_read_from_buffer(ubuf, count, ppos, priv->otp_data.buf, priv->otp_data.len);
}

static ssize_t mwl_debugfs_device_recovery_read(struct file *file,
                char __user *ubuf,
                size_t count, loff_t *ppos)
{
	int ret = 0;
	char ctrl[2];

	ctrl[0]='0';
	ctrl[1]='\n';

	ret = simple_read_from_buffer(ubuf, count, ppos, ctrl, sizeof(ctrl));
	return ret;
}


static ssize_t mwl_debugfs_device_recovery_write(struct file *file,
                                         const char __user *ubuf,
                                         size_t count, loff_t *ppos)
{
	struct mwl_priv *priv = (struct mwl_priv *)file->private_data;
	int ret = 0;
	char recover = 0;

	if (count > 2) {
		pr_err("Invalid Arguments\n\n");
		return -EINVAL;
	}

	if (copy_from_user(&recover, ubuf, 1)) {
		ret = -EFAULT;
		return ret;
	}

	switch (recover)
	{
		case '1':
			lrd_radio_recovery(priv);
		break;
		default :
			pr_err("Invalid argument : 1 needed\n");
			return -EINVAL;
		break;
	}

	return count;
}

static ssize_t mwl_debugfs_pwrtable_dump_write(struct file *file,
                                         const char __user *ubuf,
                                         size_t count, loff_t *ppos)
{
	struct mwl_priv *priv = (struct mwl_priv *)file->private_data;
	int ret = 0;
	char dump = 0;

	if(count > 2) {
		pr_err("Invalid Arguments\n\n");
		return -EINVAL;
	}

	if (copy_from_user(&dump, ubuf, 1)) {
		ret = -EFAULT;
		return ret;
	}

	switch(dump)
	{
		case '1':
			lrd_dump_max_pwr_table(priv);
		break;
		default :
			pr_err("Invalid argument : 1 needed\n");
			return -EINVAL;
		break;
	}

	return count;
}

static ssize_t mwl_debugfs_restart_required_read(struct file *file,
                char __user *ubuf,
                size_t count, loff_t *ppos)
{
	struct mwl_priv *priv = (struct mwl_priv *)file->private_data;
	int ret = 0;
	char ctrl[2];

	if (priv->recovery_in_progress)
		ctrl[0]='1';
	else
		ctrl[0]='0';

	ctrl[1]='\n';

	ret = simple_read_from_buffer(ubuf, count, ppos, ctrl, sizeof(ctrl));
	return ret;
}

static ssize_t mwl_debugfs_ds_status_read(struct file *file,
                char __user *ubuf,
                size_t count, loff_t *ppos)
{
	struct mwl_priv *priv = (struct mwl_priv *)file->private_data;
	int ret = 0;
	char ds_status[2];

	ds_status[0]= priv->if_ops.is_deepsleep(priv) ? '1' :'0';
	ds_status[1]='\n';

	ret = simple_read_from_buffer(ubuf, count, ppos, ds_status, sizeof(ds_status));
	return ret;
}

static ssize_t mwl_debugfs_ds_ctrl_read(struct file *file,
                char __user *ubuf,
                size_t count, loff_t *ppos)
{
	struct mwl_priv *priv = (struct mwl_priv *)file->private_data;
	int ret = 0;
	char ctrl[2];

	ctrl[0]= priv->ds_enable ? '1' :'0';
	ctrl[1]='\n';

	ret = simple_read_from_buffer(ubuf, count, ppos, ctrl, sizeof(ctrl));
	return ret;
}

static ssize_t mwl_debugfs_ds_ctrl_write(struct file *file,
                                         const char __user *ubuf,
                                         size_t count, loff_t *ppos)
{
	struct mwl_priv *priv = (struct mwl_priv *)file->private_data;
	int  ret = 0;
	char deepsleep;

	if (count > 2) {
		pr_err("Invalid Arguments\n\n");
		return -EINVAL;
	}

	if (priv->host_if != MWL_IF_SDIO ) {
		ret = -EACCES;
		return ret;
	}

	if (copy_from_user(&deepsleep, ubuf, 1)) {
		ret = -EFAULT;
		return ret;
	}

	switch(deepsleep) {
		case '1' : mwl_enable_ds(priv);
			break;
		case '0' : mwl_disable_ds(priv);
			break;
		default : pr_err("Invalid argument : 1/0 needed\n");
			return -EINVAL;
			break;
	}

	return count;
}


MWLWIFI_DEBUGFS_FILE_READ_OPS(info);
MWLWIFI_DEBUGFS_FILE_READ_OPS(vif);
MWLWIFI_DEBUGFS_FILE_READ_OPS(sta);
MWLWIFI_DEBUGFS_FILE_READ_OPS(ds_status);
MWLWIFI_DEBUGFS_FILE_OPS(ds_ctrl);
MWLWIFI_DEBUGFS_FILE_READ_OPS(ampdu);
MWLWIFI_DEBUGFS_FILE_OPS(dfs_channel);
MWLWIFI_DEBUGFS_FILE_OPS(dfs_radar);
MWLWIFI_DEBUGFS_FILE_OPS(thermal);
MWLWIFI_DEBUGFS_FILE_OPS(regrdwr);
MWLWIFI_DEBUGFS_FILE_READ_OPS(otp_data);
MWLWIFI_DEBUGFS_FILE_OPS(device_recovery);
MWLWIFI_DEBUGFS_FILE_READ_OPS(restart_required);
MWLWIFI_DEBUGFS_FILE_WRITE_OPS(pwrtable_dump);

void mwl_debugfs_init(struct ieee80211_hw *hw)
{
	struct mwl_priv *priv = hw->priv;

	if (!priv->debugfs_phy)
		priv->debugfs_phy = debugfs_create_dir("lrdwifi", hw->wiphy->debugfsdir);

	if (!priv->debugfs_phy)
		return;

	MWLWIFI_DEBUGFS_ADD_FILE(info);
	MWLWIFI_DEBUGFS_ADD_FILE(ds_status);
	MWLWIFI_DEBUGFS_ADD_FILE(ds_ctrl);
	MWLWIFI_DEBUGFS_ADD_FILE(vif);
	MWLWIFI_DEBUGFS_ADD_FILE(sta);
	MWLWIFI_DEBUGFS_ADD_FILE(ampdu);
	MWLWIFI_DEBUGFS_ADD_FILE(dfs_channel);
	MWLWIFI_DEBUGFS_ADD_FILE(dfs_radar);
	MWLWIFI_DEBUGFS_ADD_FILE(thermal);
	MWLWIFI_DEBUGFS_ADD_FILE(regrdwr);
	MWLWIFI_DEBUGFS_ADD_FILE(otp_data);
	MWLWIFI_DEBUGFS_ADD_FILE(device_recovery);
	MWLWIFI_DEBUGFS_ADD_FILE(restart_required);
	MWLWIFI_DEBUGFS_ADD_FILE(pwrtable_dump);
}

void mwl_debugfs_remove(struct ieee80211_hw *hw)
{
	struct mwl_priv *priv = hw->priv;

	debugfs_remove_recursive(priv->debugfs_phy);
	priv->debugfs_phy = NULL;
}
