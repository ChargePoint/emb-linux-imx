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

/* Description:  This file implements main functions of this module. */

#include <linux/module.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/suspend.h>
#include <linux/fips.h>
#include <linux/firmware.h>
#include <linux/rtnetlink.h>
#include <net/netlink.h>


#include "sysadpt.h"
#include "dev.h"
#include "pcie.h"
#include "fwcmd.h"
#include "tx.h"
#include "rx.h"
#include "isr.h"
#include "thermal.h"
#include "vendor_cmd.h"
#include "hostcmd.h"

#ifdef CONFIG_SYSFS
#include "sysfs.h"
#endif

#ifdef CONFIG_DEBUG_FS
#include "debugfs.h"
#endif

#include "main.h"
#include "vendor_cmd.h"
#define FILE_PATH_LEN    64

#define NOT_LRD_HW  0x214C5244
#define NUM_UNII3_CHANNELS  5
#define NUM_WW_CHANNELS     3
#define REG_PWR_DB_NAME     MWL_FW_ROOT"/regpwr.db"

static const struct ieee80211_channel mwl_channels_24[] = {
	{ .band = NL80211_BAND_2GHZ, .center_freq = 2412, .hw_value = 1, },
	{ .band = NL80211_BAND_2GHZ, .center_freq = 2417, .hw_value = 2, },
	{ .band = NL80211_BAND_2GHZ, .center_freq = 2422, .hw_value = 3, },
	{ .band = NL80211_BAND_2GHZ, .center_freq = 2427, .hw_value = 4, },
	{ .band = NL80211_BAND_2GHZ, .center_freq = 2432, .hw_value = 5, },
	{ .band = NL80211_BAND_2GHZ, .center_freq = 2437, .hw_value = 6, },
	{ .band = NL80211_BAND_2GHZ, .center_freq = 2442, .hw_value = 7, },
	{ .band = NL80211_BAND_2GHZ, .center_freq = 2447, .hw_value = 8, },
	{ .band = NL80211_BAND_2GHZ, .center_freq = 2452, .hw_value = 9, },
	{ .band = NL80211_BAND_2GHZ, .center_freq = 2457, .hw_value = 10, },
	{ .band = NL80211_BAND_2GHZ, .center_freq = 2462, .hw_value = 11, },
	{ .band = NL80211_BAND_2GHZ, .center_freq = 2467, .hw_value = 12, },
	{ .band = NL80211_BAND_2GHZ, .center_freq = 2472, .hw_value = 13, },
	{ .band = NL80211_BAND_2GHZ, .center_freq = 2484, .hw_value = 14, },
};

const struct ieee80211_rate mwl_rates_24[] = {
	{ .bitrate = 10, .hw_value = 2, },
	{ .bitrate = 20, .hw_value = 4, },
	{ .bitrate = 55, .hw_value = 11, },
	{ .bitrate = 110, .hw_value = 22, },
	{ .bitrate = 60, .hw_value = 12, },
	{ .bitrate = 90, .hw_value = 18, },
	{ .bitrate = 120, .hw_value = 24, },
	{ .bitrate = 180, .hw_value = 36, },
	{ .bitrate = 240, .hw_value = 48, },
	{ .bitrate = 360, .hw_value = 72, },
	{ .bitrate = 480, .hw_value = 96, },
	{ .bitrate = 540, .hw_value = 108, },
};

static const struct ieee80211_channel mwl_channels_50[] = {
	{ .band = NL80211_BAND_5GHZ, .center_freq = 5180, .hw_value = 36, },
	{ .band = NL80211_BAND_5GHZ, .center_freq = 5200, .hw_value = 40, },
	{ .band = NL80211_BAND_5GHZ, .center_freq = 5220, .hw_value = 44, },
	{ .band = NL80211_BAND_5GHZ, .center_freq = 5240, .hw_value = 48, },
	{ .band = NL80211_BAND_5GHZ, .center_freq = 5260, .hw_value = 52, },
	{ .band = NL80211_BAND_5GHZ, .center_freq = 5280, .hw_value = 56, },
	{ .band = NL80211_BAND_5GHZ, .center_freq = 5300, .hw_value = 60, },
	{ .band = NL80211_BAND_5GHZ, .center_freq = 5320, .hw_value = 64, },
	{ .band = NL80211_BAND_5GHZ, .center_freq = 5500, .hw_value = 100, },
	{ .band = NL80211_BAND_5GHZ, .center_freq = 5520, .hw_value = 104, },
	{ .band = NL80211_BAND_5GHZ, .center_freq = 5540, .hw_value = 108, },
	{ .band = NL80211_BAND_5GHZ, .center_freq = 5560, .hw_value = 112, },
	{ .band = NL80211_BAND_5GHZ, .center_freq = 5580, .hw_value = 116, },
	{ .band = NL80211_BAND_5GHZ, .center_freq = 5600, .hw_value = 120, },
	{ .band = NL80211_BAND_5GHZ, .center_freq = 5620, .hw_value = 124, },
	{ .band = NL80211_BAND_5GHZ, .center_freq = 5640, .hw_value = 128, },
	{ .band = NL80211_BAND_5GHZ, .center_freq = 5660, .hw_value = 132, },
	{ .band = NL80211_BAND_5GHZ, .center_freq = 5680, .hw_value = 136, },
	{ .band = NL80211_BAND_5GHZ, .center_freq = 5700, .hw_value = 140, },
	{ .band = NL80211_BAND_5GHZ, .center_freq = 5720, .hw_value = 144, },
	{ .band = NL80211_BAND_5GHZ, .center_freq = 5745, .hw_value = 149, },
	{ .band = NL80211_BAND_5GHZ, .center_freq = 5765, .hw_value = 153, },
	{ .band = NL80211_BAND_5GHZ, .center_freq = 5785, .hw_value = 157, },
	{ .band = NL80211_BAND_5GHZ, .center_freq = 5805, .hw_value = 161, },
	{ .band = NL80211_BAND_5GHZ, .center_freq = 5825, .hw_value = 165, },
};

const struct ieee80211_rate mwl_rates_50[] = {
	{ .bitrate = 60, .hw_value = 12, },
	{ .bitrate = 90, .hw_value = 18, },
	{ .bitrate = 120, .hw_value = 24, },
	{ .bitrate = 180, .hw_value = 36, },
	{ .bitrate = 240, .hw_value = 48, },
	{ .bitrate = 360, .hw_value = 72, },
	{ .bitrate = 480, .hw_value = 96, },
	{ .bitrate = 540, .hw_value = 108, },
};

static const struct ieee80211_iface_limit ibss_if_limits[] = {
	{ .max = 1,	.types = BIT(NL80211_IFTYPE_ADHOC) }
};

static const struct ieee80211_iface_limit ap_if_su_limits[] = {
	{ .max = SYSADPT_NUM_OF_SU_AP, .types = BIT(NL80211_IFTYPE_AP) },
	{ .max = SYSADPT_NUM_OF_STA,   .types = BIT(NL80211_IFTYPE_STATION) |
	                                        BIT(NL80211_IFTYPE_P2P_GO) |
	                                        BIT(NL80211_IFTYPE_P2P_CLIENT)},
};

static const struct ieee80211_iface_combination if_su_comb[] = {
	{
		.limits                 = ap_if_su_limits,
		.n_limits               = ARRAY_SIZE(ap_if_su_limits),
		.max_interfaces         = SYSADPT_NUM_OF_SU_AP,
		.num_different_channels = 1,
		.radar_detect_widths    = 0,
	},
	{
		.limits                 = ibss_if_limits,
		.n_limits               = ARRAY_SIZE(ibss_if_limits),
		.max_interfaces         = 1,
		.num_different_channels = 1,
		.radar_detect_widths    = 0,
	}
};


static const struct ieee80211_iface_limit if_st_limits[] = {
	{ .max = SYSADPT_NUM_OF_ST_AP, .types = BIT(NL80211_IFTYPE_AP) },
	{ .max = SYSADPT_NUM_OF_STA,   .types = BIT(NL80211_IFTYPE_STATION) |
	                                        BIT(NL80211_IFTYPE_P2P_GO) |
	                                        BIT(NL80211_IFTYPE_P2P_CLIENT)},
};

static const struct ieee80211_iface_combination if_st_comb[] = {
	{
		.limits                 = if_st_limits,
		.n_limits               = ARRAY_SIZE(if_st_limits),
		.max_interfaces         = SYSADPT_NUM_OF_ST_AP + SYSADPT_NUM_OF_STA,
		.num_different_channels = 1,
		.radar_detect_widths    = 0,
	},
	{
		.limits                 = ibss_if_limits,
		.n_limits               = ARRAY_SIZE(ibss_if_limits),
		.max_interfaces         = 1,
		.num_different_channels = 1,
		.radar_detect_widths    = 0
	}

};

#ifdef CONFIG_PM
static const struct wiphy_wowlan_support lrd_wowlan_support = {
	.flags = WIPHY_WOWLAN_ANY        |
	         WIPHY_WOWLAN_DISCONNECT |
	         WIPHY_WOWLAN_NET_DETECT,
	.n_patterns      = 0,
	.pattern_min_len = 0,
	.pattern_max_len = 0,
};
#endif

/* CAL data config file */
static char *cal_data_cfg;

/* WMM Turbo mode */
int wmm_turbo = 1;

/* EDMAC Control */
int EDMAC_Ctrl = 0x0;

/* Tx AMSDU control*/
int tx_amsdu_enable = 0;

int ds_enable = DS_ENABLE_ON;
int enable_bt_dup = 0;

/* MFG mode control*/
int mfg_mode = 0;

/* Laird additions */
int SISO_mode = 0;
int lrd_debug = 0;
int null_scan_count = 0;
unsigned int stop_shutdown = 0;
unsigned int stop_shutdown_timeout = 12000;
unsigned int enable_24_40mhz = 0;
unsigned int ant_gain_adjust = 0;
unsigned int host_crypto_mode = 0;

static int lrd_send_fw_event(struct device *dev, bool on)
{
	static char *env_on[] = { "FIRMWARE=on", NULL };
	static char *env_off[] = { "FIRMWARE=off", NULL };

	return kobject_uevent_env(&dev->kobj, KOBJ_CHANGE, on ? env_on : env_off);
}

static bool mwl_is_world_region(struct cc_info *cc)
{
	if (cc->region == REG_CODE_WW) {
		return true;
	}

	return false;
}

static bool mwl_is_en440_region(struct cc_info *cc)
{
	if (cc->region == REG_CODE_ETSI0) {
		return true;
	}
	else if ( cc->region == REG_CODE_ETSI ) {
		if ((cc->alpha2[0] == 'A' && cc->alpha2[1] == 'T') ||
		    (cc->alpha2[0] == 'B' && cc->alpha2[1] == 'E') ||
		    (cc->alpha2[0] == 'B' && cc->alpha2[1] == 'G') ||
		    (cc->alpha2[0] == 'C' && cc->alpha2[1] == 'Y') ||
		    (cc->alpha2[0] == 'C' && cc->alpha2[1] == 'Z') ||
		    (cc->alpha2[0] == 'D' && cc->alpha2[1] == 'E') ||
		    (cc->alpha2[0] == 'D' && cc->alpha2[1] == 'K') ||
		    (cc->alpha2[0] == 'E' && cc->alpha2[1] == 'E') ||
		    (cc->alpha2[0] == 'E' && cc->alpha2[1] == 'S') ||
		    (cc->alpha2[0] == 'F' && cc->alpha2[1] == 'I') ||
		    (cc->alpha2[0] == 'F' && cc->alpha2[1] == 'R') ||
		    (cc->alpha2[0] == 'G' && cc->alpha2[1] == 'B') ||
		    (cc->alpha2[0] == 'G' && cc->alpha2[1] == 'R') ||
		    (cc->alpha2[0] == 'H' && cc->alpha2[1] == 'R') ||
		    (cc->alpha2[0] == 'H' && cc->alpha2[1] == 'U') ||
		    (cc->alpha2[0] == 'I' && cc->alpha2[1] == 'E') ||
		    (cc->alpha2[0] == 'I' && cc->alpha2[1] == 'T') ||
		    (cc->alpha2[0] == 'L' && cc->alpha2[1] == 'T') ||
		    (cc->alpha2[0] == 'L' && cc->alpha2[1] == 'U') ||
		    (cc->alpha2[0] == 'L' && cc->alpha2[1] == 'V') ||
		    (cc->alpha2[0] == 'M' && cc->alpha2[1] == 'T') ||
		    (cc->alpha2[0] == 'N' && cc->alpha2[1] == 'L') ||
		    (cc->alpha2[0] == 'P' && cc->alpha2[1] == 'L') ||
		    (cc->alpha2[0] == 'P' && cc->alpha2[1] == 'T') ||
		    (cc->alpha2[0] == 'R' && cc->alpha2[1] == 'O') ||
 		    (cc->alpha2[0] == 'S' && cc->alpha2[1] == 'E') ||
		    (cc->alpha2[0] == 'S' && cc->alpha2[1] == 'K') ||
		    (cc->alpha2[0] == 'S' && cc->alpha2[1] == 'I')) {
			return true;
		}
	}

	return false;
}

static bool mwl_is_unknown_country(struct cc_info *cc)
{
	if (cc->alpha2[0] == '9' && cc->alpha2[1] == '9') {
		return true;
	}

	return false;
}

static int lrd_fixup_channels(struct mwl_priv *priv, struct cc_info *info)
{
	if (mwl_is_world_region(info)) {
		/* when configured for WW, firmware does not allow
		 * channels 12-14 to be configured, remove them here
		 * to keep mac80211 in sync with FW.
		 */
		if (priv->band_24.n_channels == ARRAY_SIZE(mwl_channels_24)) {
			priv->band_24.n_channels -= NUM_WW_CHANNELS;
		}

		priv->band_50.n_channels = ARRAY_SIZE(mwl_channels_50);
	}
	else {
		priv->band_24.n_channels = ARRAY_SIZE(mwl_channels_24);;
		priv->band_50.n_channels = ARRAY_SIZE(mwl_channels_50);

		if (mwl_is_en440_region(info) ) {
			if (0 == (priv->radio_caps.capability & LRD_CAP_440) ) {
				//If 440 isn't set in caps fw does not allow
				//channels 149-165 to be configured.  Remove
				//them here to keep mac80211 in sync with FW.
				priv->band_50.n_channels -= NUM_UNII3_CHANNELS;
			}
		}
	}

	return 0;
}


static int mwl_init_firmware(struct mwl_priv *priv)
{
	int rc = 0;
	const char *fw_name;

	/* Attempt to load mfg firmware first.
	 *
	 * If mfg_mode module parameter is set, mfg firmware is expected to be
	 * present so request_firmware() can be safely used.
	 * Otherwise, attempt to load mfg firmware from standard location
	 */
	fw_name = priv->if_ops.mwl_chip_tbl.mfg_image;

	if (mfg_mode) {
		wiphy_info(priv->hw->wiphy, "%s: Manufacturing mode forced on!\n", MWL_DRV_NAME);

		priv->mfg_mode = true;

		/* request_firmware() may hang if firmware is not present and
		 * FW user mode helper support is enabled in kernel
		 */
		rc = request_firmware((const struct firmware **)&priv->fw_ucode,
				       fw_name, priv->dev);
	}
	else {
		/* request_firmware_direct() will only find firmware if it is present in
		 * standard locations, but will not hang
		 */
		rc = request_firmware_direct((const struct firmware **)&priv->fw_ucode,
				      fw_name, priv->dev);
		if (!rc)
		{
			/* mfg firmware found and loaded directly */
			priv->mfg_mode = true;
		}
	}

	if (rc) {

		if (!mfg_mode) {

			fw_name = priv->if_ops.mwl_chip_tbl.fw_image;

			rc = request_firmware((const struct firmware **)&priv->fw_ucode,
								fw_name, priv->dev);
		}

		if (rc) {
			wiphy_err(priv->hw->wiphy,
				  "%s: cannot find firmware image <%s>\n",
				  MWL_DRV_NAME, fw_name);

			goto err_load_fw;
		}
	}

	wiphy_info(priv->hw->wiphy, "%s: found firmware image <%s>\n",
		   MWL_DRV_NAME, fw_name);

	rc = priv->if_ops.prog_fw(priv);
	if (rc) {
		if (rc != -EINPROGRESS)
			wiphy_err(priv->hw->wiphy,
				"%s: firmware download/init failed! <%s> %d\n",
				MWL_DRV_NAME, fw_name, rc);
		goto err_download_fw;
	}

	if (cal_data_cfg) {

		wiphy_info(priv->hw->wiphy,
			"Looking for cal file <%s>\n", cal_data_cfg);

		if ((request_firmware((const struct firmware **)&priv->cal_data,
		     cal_data_cfg, priv->dev)) < 0)
			wiphy_info(priv->hw->wiphy,
				  "Cal data request_firmware() failed\n");
	}

err_download_fw:
	release_firmware(priv->fw_ucode);

err_load_fw:
	return rc;
}

static void mwl_reg_notifier(struct wiphy *wiphy,
			     struct regulatory_request *request)
{
	struct ieee80211_hw *hw;
	struct mwl_priv *priv;

	hw = (struct ieee80211_hw *)wiphy_priv(wiphy);
	priv = hw->priv;

	wiphy_debug(hw->wiphy, "mwl_reg_notifier set=%d %s %c%c\n", priv->reg.regulatory_set, reg_initiator_name(request->initiator), request->alpha2[0], request->alpha2[1]);

	if (request->initiator == NL80211_REGDOM_SET_BY_DRIVER) {

		if ( 0 == memcmp(priv->reg.hint.alpha2, request->alpha2, sizeof(priv->reg.hint.alpha2)) ) {
			memcpy(&priv->reg.cc, &priv->reg.hint, sizeof(priv->reg.cc));
			memset(priv->reg.hint.alpha2, 'x', sizeof(priv->reg.hint.alpha2));
			priv->reg.hint.region =(u32)-1;
			priv->reg.dfs_region = request->dfs_region;
		}
	}
}

#pragma pack(push,1)
typedef struct module_signature {
	u8     sig[8];
	__le32 len;
} module_signature_t;

typedef struct ww_pwr_entry
{
	u8     cc[2];
	__le16 rc;
	__le32 len;
	u8     data[0];
} ww_pwr_entry_t;

#pragma pack(pop)

#define MODULE_SIG_STRING    "~Module signature appended~\n"

static int lrd_validate_db(const struct firmware *fw, struct mwl_priv *priv, int * payload_size)
{
	u8   *buf = (uint8_t*)fw->data;
	int bSize = fw->size;
	int markerlen = sizeof(MODULE_SIG_STRING)-1;
	int x = 0;
	struct ww_pwr_entry *entry;
	struct module_signature *info;
	struct ieee80211_hw *hw = priv->hw;

	wiphy_debug(hw->wiphy,"Validating %s\n", REG_PWR_DB_NAME);

	/* Verify signature marker is present */
	if (bSize < markerlen) {
		wiphy_err(hw->wiphy, "%s too small to contain signature\n", REG_PWR_DB_NAME);
		return -1;
	}

	bSize -= markerlen;

	if (memcmp(buf+ bSize, MODULE_SIG_STRING, markerlen) != 0 ) {
		wiphy_err(hw->wiphy, "Module signature marker not found %d\n", bSize);
		return -1;
	}

	/* Verify module signature info is present */
	if (bSize < sizeof(module_signature_t)) {
		wiphy_err(hw->wiphy, "Module signature not found %d\n", bSize);
		return -1;
	}

	bSize -= sizeof(module_signature_t);
	info = (module_signature_t*)(buf + bSize);

	/*Verify actual signature is present */
	if (bSize <  le32_to_cpu(info->len)) {
		wiphy_err(hw->wiphy, "Complete signature not present %d(%d)\n", bSize, le32_to_cpu(info->len));
		return -1;
	}

	bSize -= le32_to_cpu(info->len);

	for (x=0; x < bSize;) {
		entry = (ww_pwr_entry_t*)(buf + x);

		//Check header present
		if ( (bSize - x) < sizeof(ww_pwr_entry_t)) {
			wiphy_err(hw->wiphy, "Incomplete Tx Pw Entry header\n");
			return -1;
		}

		//Move past header
		x += sizeof(ww_pwr_entry_t);

		//Check data size
		if ( (bSize - x) <  le32_to_cpu(entry->len)) {
			wiphy_err(hw->wiphy, "Incomplete Tx Pw Entry \n");
			return -1;
		}

		//Move to next entry
		x += le32_to_cpu(entry->len);
	}

	if (x == bSize) {
		wiphy_debug(hw->wiphy, "Successfully validated %s %d\n", REG_PWR_DB_NAME, bSize);
		*payload_size = bSize;
		return 0;
	}

	wiphy_err(hw->wiphy, "Validation of %s failed.\n", REG_PWR_DB_NAME);

	return -1;
}


static int find_pwr_entry(struct mwl_priv *priv, u8 *cc, u16 rc, ww_pwr_entry_t**entry)
{
	struct ww_pwr_entry *tmp  = NULL;
	bool bMatch = false;
	int x   = 0;
	int y   = 0;
	int idx = -1;

	//See if RC/CC is found in DB
	for (x=0, y = 0; x < priv->reg.db_size; y++) {
		tmp = (ww_pwr_entry_t*)(priv->reg.db + x);

		if (rc < 256) {
			if (rc == le16_to_cpu(tmp->rc)) {
				bMatch = true;
			}
		}
		else {
			u16  region = cpu_to_le16(rc);
			if (0 == memcmp(tmp->cc, &region, sizeof(tmp->cc))) {
				bMatch = true;
			}
		}

		if (bMatch) {
			if (cc == NULL || 0 == memcmp(tmp->cc, cc, sizeof(tmp->cc))) {
				if (entry) {
					*entry = tmp;
				}
				idx = y;
				break;
			}
		}

		//Move to next entry
		x += sizeof(ww_pwr_entry_t) + le32_to_cpu(tmp->len);
	}

	return idx;
}

static void lrd_send_hint_no_lock(struct mwl_priv *priv, struct cc_info *cc)
{
	//Check region and CCin case either has changed.
	if (0 == memcmp(&priv->reg.cc, cc, sizeof(priv->reg.cc))) {
		//Aleady using, don't send again
		return;
	}

	wiphy_debug(priv->hw->wiphy,"Sending regulatory hint for %c%c\n", cc->alpha2[0], cc->alpha2[1]);

	//Fixup the channel set based on country/region
	lrd_fixup_channels(priv, cc);

	memcpy(&priv->reg.hint, cc, sizeof(priv->reg.hint));

	//Send driver hint
	priv->reg.regulatory_set = true;
	regulatory_hint(priv->hw->wiphy, cc->alpha2);
}

static void lrd_send_hint(struct mwl_priv *priv, struct cc_info *cc)
{
	mutex_lock(&priv->reg.mutex);
	lrd_send_hint_no_lock(priv, cc);
	mutex_unlock(&priv->reg.mutex);
}

int check_regdb(const char *alpha2);

static void lrd_cc_cb(const struct firmware *fw, void *context)
{
	struct mwl_priv *priv = (struct mwl_priv*)context;
	struct ww_pwr_entry *entry = NULL;
	int idx = -1;
	int payload_size;
	int rc = 0;

	if (!fw) {
		wiphy_info(priv->hw->wiphy,"/lib/firmware/%s not found.\n", REG_PWR_DB_NAME);
		goto exit;
	}

	if (priv->reg.db) {
		kfree(priv->reg.db);
		priv->reg.db      = NULL;
		priv->reg.db_size = 0;
	}

	if (!lrd_validate_db(fw, priv, &payload_size)) {
		priv->reg.db = kmemdup(fw->data, payload_size, GFP_KERNEL);
		priv->reg.db_size = priv->reg.db ? payload_size : 0;
	}

	release_firmware(fw);

	/* Find entry */
	wiphy_debug(priv->hw->wiphy,
		"checking %s for region:%x country:%c%c\n", REG_PWR_DB_NAME,
		priv->reg.otp.region, priv->reg.otp.alpha2[0],
		priv->reg.otp.alpha2[1]);

	idx = find_pwr_entry(priv, mwl_is_unknown_country(&priv->reg.otp) ?
		NULL : priv->reg.otp.alpha2, priv->reg.otp.region, &entry);

	if (!entry || le32_to_cpu(entry->len) <= 0) {
		if (idx < 0) {
			wiphy_warn(priv->hw->wiphy,
				"Region %x country %c%c not found in %s.\n",
				priv->reg.otp.region, priv->reg.otp.alpha2[0],
				priv->reg.otp.alpha2[1], REG_PWR_DB_NAME);
		}
		goto exit;
	}

	mutex_lock(&priv->reg.mutex);
	if (!priv->reg.pn) {
		rtnl_lock();
		rc = check_regdb(entry->cc);
		rtnl_unlock();
		if (rc >= 0) {
			/* Send to firmware */
			rc = lrd_fwcmd_lrd_set_power_table(priv->hw, idx, entry->data,
				le32_to_cpu(entry->len));
			if (!rc) {
				/* Queue event work */
				queue_work(priv->lrd_workq, &priv->reg.event);
			} else {
				idx = -100;
			}
		}
		else {
			wiphy_err(priv->hw->wiphy,
				"Country Code %c%c not present in CRDA database.\n",
				entry->cc[0], entry->cc[1]);
			idx = -101;
		}
	}
	mutex_unlock(&priv->reg.mutex);

exit:
	if (!mwl_is_unknown_country(&priv->reg.otp)) {
		/* Send driver hint */
		lrd_send_hint(priv, &priv->reg.otp);
	} else if (idx < 0) {
		wiphy_err(priv->hw->wiphy,
			"Failed to resolve region mapping %d, will not enable transmitter.\n", idx);
	}
}

void lrd_dump_max_pwr_table(struct mwl_priv *priv)
{
	u16 powlist[HAL_TRPC_ID_MAX];
	int ch_index, rate_index;
	int edmac;
	int rc;

	memset(powlist, 0, sizeof(powlist));
	pr_info("Power Table - Max per RateId:\n");
	pr_info(" CH,   0   1   2   3   4   5   6   7   8   9  10  11  12  13  14  15  16  17  18  19  20  21  22  23  24  25  26  27\n");
	for (ch_index = 0; ch_index < ARRAY_SIZE(mwl_channels_24); ch_index++) {
		int ch = mwl_channels_24[ch_index].hw_value;
		rc = mwl_fwcmd_get_tx_powers(priv, &powlist[0], HOSTCMD_ACT_GET_MAX_TX_PWR,
						ch, FREQ_BAND_2DOT4GHZ, CH_20_MHZ_WIDTH, NO_EXT_CHANNEL);
		if (rc) {
			pr_err("Failed to get max power for channel %d!\n", ch);
			return;
		}
		pr_info("%3d, {", ch);
		for (rate_index = 0; rate_index < HAL_TRPC_ID_2G_MAX; rate_index++) {
			pr_cont("%2d%s", powlist[rate_index], rate_index == HAL_TRPC_ID_2G_MAX-1 ? "}\n" : ", ");
		}
	}

	pr_info("\n");
	memset(powlist, 0, sizeof(powlist));
	for (ch_index = 0; ch_index < ARRAY_SIZE(mwl_channels_50); ch_index++) {
		int ch = mwl_channels_50[ch_index].hw_value;
		rc = mwl_fwcmd_get_tx_powers(priv, &powlist[0], HOSTCMD_ACT_GET_MAX_TX_PWR,
						ch, FREQ_BAND_5GHZ, CH_20_MHZ_WIDTH, NO_EXT_CHANNEL);
		if (rc) {
			pr_err("Failed to get max power for channel %d!\n", ch);
			return;
		}
		pr_info("%3d, {", ch);
		for (rate_index = 0; rate_index < HAL_TRPC_ID_5G_MAX; rate_index++) {
			pr_cont("%2d%s", powlist[rate_index], rate_index == HAL_TRPC_ID_5G_MAX-1 ? "}\n" : ", ");
		}
	}

	if (!mwl_fwcmd_getEDMAC(priv, &edmac)) {
		pr_info("EDMAC: %s\n", edmac ? "Enabled" : "Disabled");
	}
	return;
}

void mwl_set_ht_caps(struct mwl_priv *priv,
			    struct ieee80211_supported_band *band)
{
	struct ieee80211_hw *hw;

	hw = priv->hw;

	band->ht_cap.ht_supported = 1;

	band->ht_cap.cap |= IEEE80211_HT_CAP_LDPC_CODING;
	band->ht_cap.cap |= IEEE80211_HT_CAP_SM_PS;
	band->ht_cap.cap |= IEEE80211_HT_CAP_SGI_20;

	if (band->band == NL80211_BAND_5GHZ || enable_24_40mhz) {
		band->ht_cap.cap |= IEEE80211_HT_CAP_SGI_40;
		band->ht_cap.cap |= IEEE80211_HT_CAP_SUP_WIDTH_20_40;

		if (band->band == NL80211_BAND_2GHZ) {
			band->ht_cap.cap |= IEEE80211_HT_CAP_DSSSCCK40;
		}
	}

	if ((priv->chip_type == MWL8997) &&
		(priv->ant_tx_num > 1)){
		band->ht_cap.cap |= (1 << IEEE80211_HT_CAP_RX_STBC_SHIFT);
	}

	ieee80211_hw_set(hw, AMPDU_AGGREGATION);
	ieee80211_hw_set(hw, SUPPORTS_AMSDU_IN_AMPDU);
	band->ht_cap.ampdu_factor = IEEE80211_HT_MAX_AMPDU_64K;
	band->ht_cap.ampdu_density = IEEE80211_HT_MPDU_DENSITY_4;

	band->ht_cap.mcs.rx_mask[0] = 0xff;

	if (priv->ant_rx_num > 1)
		band->ht_cap.mcs.rx_mask[1] = 0xff;

#if 0
	if (priv->antenna_rx == ANTENNA_RX_4_AUTO)
		band->ht_cap.mcs.rx_mask[2] = 0xff;
#endif
	band->ht_cap.mcs.rx_mask[4] = 0x01;

	band->ht_cap.mcs.tx_params = IEEE80211_HT_MCS_TX_DEFINED;
}

void mwl_set_vht_caps(struct mwl_priv *priv,
			     struct ieee80211_supported_band *band)
{
	band->vht_cap.vht_supported = 1;

	band->vht_cap.cap |= IEEE80211_VHT_CAP_MAX_MPDU_LENGTH_3895;
	band->vht_cap.cap |= IEEE80211_VHT_CAP_RXLDPC;
	band->vht_cap.cap |= IEEE80211_VHT_CAP_SHORT_GI_80;
	band->vht_cap.cap |= IEEE80211_VHT_CAP_RXSTBC_1;

	if (priv->ant_tx_num > 1)
		band->vht_cap.cap |= IEEE80211_VHT_CAP_SU_BEAMFORMER_CAPABLE;

	band->vht_cap.cap |= IEEE80211_VHT_CAP_SU_BEAMFORMEE_CAPABLE;
	band->vht_cap.cap |= IEEE80211_VHT_CAP_MAX_A_MPDU_LENGTH_EXPONENT_MASK;
	band->vht_cap.cap |= IEEE80211_VHT_CAP_RX_ANTENNA_PATTERN;
	band->vht_cap.cap |= IEEE80211_VHT_CAP_TX_ANTENNA_PATTERN;

	if (priv->chip_type == MWL8964) {
		band->vht_cap.cap |= IEEE80211_VHT_CAP_SHORT_GI_160;
		band->vht_cap.cap |= IEEE80211_VHT_CAP_SUPP_CHAN_WIDTH_160MHZ;
	}

	if (priv->ant_rx_num == 1)
		band->vht_cap.vht_mcs.rx_mcs_map = cpu_to_le16(0xfffe);
	else if (priv->ant_rx_num == 2)
		band->vht_cap.vht_mcs.rx_mcs_map = cpu_to_le16(0xfffa);
	else
		band->vht_cap.vht_mcs.rx_mcs_map = cpu_to_le16(0xffea);

	if (priv->ant_tx_num == 1) {
		band->vht_cap.vht_mcs.tx_mcs_map = cpu_to_le16(0xfffe);
	} else if (priv->ant_tx_num == 2) {
		band->vht_cap.vht_mcs.tx_mcs_map = cpu_to_le16(0xfffa);
	} else
		band->vht_cap.vht_mcs.tx_mcs_map = cpu_to_le16(0xffea);

	if (band->vht_cap.cap & (IEEE80211_VHT_CAP_SU_BEAMFORMEE_CAPABLE |
	    IEEE80211_VHT_CAP_MU_BEAMFORMEE_CAPABLE)) {
		band->vht_cap.cap |=
			((priv->ant_tx_num - 1) <<
			IEEE80211_VHT_CAP_BEAMFORMEE_STS_SHIFT) &
			IEEE80211_VHT_CAP_BEAMFORMEE_STS_MASK;
	}

	if (band->vht_cap.cap & (IEEE80211_VHT_CAP_SU_BEAMFORMER_CAPABLE |
	    IEEE80211_VHT_CAP_MU_BEAMFORMER_CAPABLE)) {
		band->vht_cap.cap |=
			((priv->ant_tx_num - 1) <<
			IEEE80211_VHT_CAP_SOUNDING_DIMENSIONS_SHIFT) &
			IEEE80211_VHT_CAP_SOUNDING_DIMENSIONS_MASK;
	}
}

static struct ieee80211_iface_combination*
mwl_alloc_intf_combos(const struct ieee80211_iface_combination *c, int n_c)
{
	int size = 0;
	int    x = 0;
	u8  *end = 0;
	struct ieee80211_iface_combination *combo = NULL;

	if (!c) {
		return NULL;
	}

	size  = sizeof(struct ieee80211_iface_combination) * n_c;
	for (x = 0; x < n_c; x++) {
		//figure out size of limits
		size += sizeof(struct ieee80211_iface_limit) * c[x].n_limits;
	}

	//Allocate memory
	combo = (struct ieee80211_iface_combination *)kzalloc(size, GFP_KERNEL);

	if (combo) {
		//Copy interface combination array
		memcpy(combo, c, sizeof(struct ieee80211_iface_combination) * n_c);

		end = ((u8*)combo) + (sizeof(struct ieee80211_iface_combination) * n_c);

		//Copy limit arrays
		for (x = 0; x < combo->n_limits; x++) {
			//Fix limits array to point to end of combo array
			combo[x].limits = (struct ieee80211_iface_limit*)end;
			memcpy((void*)combo[x].limits, c[x].limits, sizeof(struct ieee80211_iface_limit)*combo[x].n_limits);
			end += sizeof(struct ieee80211_iface_limit) * combo[x].n_limits;
		}
	}

	return combo;
}

static void lrd_adjust_iface_combo(struct mwl_priv *priv, struct ieee80211_iface_combination *combos, int n_c)
{
	int x = 0;
	int y = 0;
	u16 max_intf = (u16)(priv->radio_caps.num_mac);

	if (NULL == combos) {
		return;
	}

	for (x = 0; x < n_c; x++) {
		if (combos[x].max_interfaces > max_intf) {
			wiphy_info(priv->hw->wiphy,"Adjusting combo %d's number of supported interfaces to %d\n", x, max_intf);

			combos[x].max_interfaces = max_intf;

			for (y = 0; y < combos[x].n_limits; y++) {
				if (combos[x].limits[y].max > max_intf) {
					((struct ieee80211_iface_limit*)combos[x].limits)[y].max = max_intf;
				}
			}
		}
	}
}

static int lrd_set_su_caps(struct mwl_priv *priv)
{
	struct ieee80211_hw *hw = priv->hw;

	//Allocate and adjust interface combinations
	hw->wiphy->iface_combinations   = mwl_alloc_intf_combos(if_su_comb, ARRAY_SIZE(if_su_comb));
	if (hw->wiphy->iface_combinations) {
		hw->wiphy->n_iface_combinations = ARRAY_SIZE(if_su_comb);

		lrd_adjust_iface_combo( priv,
		                       (struct ieee80211_iface_combination *)hw->wiphy->iface_combinations,
		                        hw->wiphy->n_iface_combinations);
	}

#ifdef CONFIG_PM
	if (priv->wow.capable) {
		hw->wiphy->wowlan = &lrd_wowlan_support;
		/* max number of SSIDs device can scan for */
		hw->wiphy->max_sched_scan_ssids = 1;
	}
#endif

	/* Register for monitor interfaces, to work around mac80211 bug */
	ieee80211_hw_set(hw, WANT_MONITOR_VIF);

	if (hw->wiphy->iface_combinations) {
		/* Caller will do actual allocation */
		/*Note:  This assumes first combo element contains the max number of interfaces */
		hw->wiphy->n_addresses =  hw->wiphy->iface_combinations[0].max_interfaces;
	}

	return 0;
}

static int lrd_set_st_caps(struct mwl_priv *priv)
{
	struct ieee80211_hw *hw = priv->hw;
	int x = 0;

	//Allocate and adjust interface combinations
	hw->wiphy->iface_combinations   = mwl_alloc_intf_combos(if_st_comb, ARRAY_SIZE(if_st_comb));;
	if (hw->wiphy->iface_combinations) {
		hw->wiphy->n_iface_combinations = ARRAY_SIZE(if_st_comb);

		lrd_adjust_iface_combo( priv,
		                        (struct ieee80211_iface_combination *)hw->wiphy->iface_combinations,
		                        hw->wiphy->n_iface_combinations);
	}

	/* sterling fw supports fewer AP interfaces - adjust for that here*/
	priv->ap_macids_supported = 0;
	for (x = 0; x < SYSADPT_NUM_OF_ST_AP; x++) {
		priv->ap_macids_supported |= 1 << x;
	}

	if (hw->wiphy->iface_combinations) {
		/* Caller will do actual allocation */
		/*Note:  This assumes first combo element contains the max number of interfaces */
		hw->wiphy->n_addresses =  hw->wiphy->iface_combinations[0].max_interfaces;
	}

	return 0;
}

void mwl_ieee80211_free_hw(struct mwl_priv *priv)
{
	struct ieee80211_hw *hw = priv->hw;

	if (hw) {
		//Free driver allocated memory
		if (hw->wiphy->iface_combinations) {
			kfree(hw->wiphy->iface_combinations);
			hw->wiphy->iface_combinations = NULL;
		}

		if (hw->wiphy->n_addresses) {
			kfree(hw->wiphy->addresses);
			hw->wiphy->addresses = NULL;
		}
	}

	ieee80211_free_hw(hw);

}
EXPORT_SYMBOL(mwl_ieee80211_free_hw);

static void mwl_set_ieee_hw_caps(struct mwl_priv *priv)
{
	struct ieee80211_hw *hw = priv->hw;
	int    x = 0;
	u64 addr = 0;
	u64 addr1 = 0;

	SET_IEEE80211_DEV(hw, priv->dev);
	SET_IEEE80211_PERM_ADDR(hw, priv->hw_data.mac_addr);

	hw->extra_tx_headroom   = SYSADPT_TX_MIN_BYTES_HEADROOM;
	hw->queues              = SYSADPT_TX_WMM_QUEUES;

	/* Set rssi values to dBm */
	ieee80211_hw_set(hw, SIGNAL_DBM);
	ieee80211_hw_set(hw, HAS_RATE_CONTROL);

	/* Ask mac80211 not to trigger PS mode
	 * based on PM bit of incoming frames.
	 */
	ieee80211_hw_set(hw, AP_LINK_PS);
	ieee80211_hw_set(hw, SUPPORTS_PER_STA_GTK);
	ieee80211_hw_set(hw, MFP_CAPABLE);
	ieee80211_hw_set(hw, SPECTRUM_MGMT);
	if (priv->chip_type == MWL8997) {
		ieee80211_hw_set(hw, SUPPORTS_PS);
	}

	/* Ask mac80211 to use standard NULL data packets
	 * instead of QOS NULL data packets.
	 * This works around issues in both mac80211 and the driver
	 * that prevent QOS NULL data packets from being transmitted correctly
	 */
	ieee80211_hw_set(hw, DOESNT_SUPPORT_QOS_NDP);

	hw->wiphy->flags |= WIPHY_FLAG_IBSS_RSN;
	hw->wiphy->flags |= WIPHY_FLAG_HAS_CHANNEL_SWITCH;
	hw->wiphy->flags |= WIPHY_FLAG_HAS_REMAIN_ON_CHANNEL;
	hw->wiphy->flags |= WIPHY_FLAG_SUPPORTS_TDLS;

	hw->wiphy->flags |= WIPHY_FLAG_4ADDR_AP;
	hw->wiphy->flags |= WIPHY_FLAG_4ADDR_STATION;

	hw->wiphy->features |=  NL80211_FEATURE_NEED_OBSS_SCAN;
	hw->wiphy->features |=  NL80211_FEATURE_AP_SCAN;
	hw->wiphy->features &= ~NL80211_FEATURE_MAC_ON_CREATE;

	hw->wiphy->max_remain_on_channel_duration = 5000;

	hw->vif_data_size = sizeof(struct mwl_vif);
	hw->sta_data_size = sizeof(struct mwl_sta);

	hw->wiphy->available_antennas_tx = MWL_8997_DEF_TX_ANT_BMP;
	hw->wiphy->available_antennas_rx = MWL_8997_DEF_RX_ANT_BMP;

	hw->wiphy->interface_modes = 0;
	hw->wiphy->interface_modes |= BIT(NL80211_IFTYPE_AP);
	hw->wiphy->interface_modes |= BIT(NL80211_IFTYPE_STATION);
	hw->wiphy->interface_modes |= BIT(NL80211_IFTYPE_P2P_GO);
	hw->wiphy->interface_modes |= BIT(NL80211_IFTYPE_P2P_CLIENT);
	hw->wiphy->interface_modes |= BIT(NL80211_IFTYPE_ADHOC);

	/* set up band information for 2.4G */
	if (!priv->disable_2g) {
		mwl_set_ht_caps(priv,  &priv->band_24);
		mwl_set_vht_caps(priv, &priv->band_24);

		hw->wiphy->bands[NL80211_BAND_2GHZ] = &priv->band_24;
	}

	/* set up band information for 5G */
	if (!priv->disable_5g) {
		mwl_set_ht_caps(priv,  &priv->band_50);
		mwl_set_vht_caps(priv, &priv->band_50);

		hw->wiphy->bands[NL80211_BAND_5GHZ] = &priv->band_50;
	}

	if (priv->radio_caps.capability & LRD_CAP_SU60) {
		lrd_set_su_caps(priv);
	}
	else {
		lrd_set_st_caps(priv);
	}

	/* Allocated interface address pool */
	hw->wiphy->addresses = (struct mac_address*)kzalloc(sizeof(struct mac_address) * hw->wiphy->n_addresses, GFP_KERNEL);
	addr = be64_to_cpup((u64*)priv->hw_data.mac_addr);
	addr = addr >> 16;

	for (x=0; x < hw->wiphy->n_addresses; x++) {
		addr1 = cpu_to_be64(addr<<16);
		memcpy(hw->wiphy->addresses[x].addr, &addr1, sizeof(struct mac_address));

		//First address in pool must be the same as permanent adddress, increment after copy
		if (priv->radio_caps.capability & LRD_CAP_BT_SEQ) {
			if (x) {
				//Add Locally Administered Bit
				hw->wiphy->addresses[x].addr[0] |= 0x02;
				addr++;
			}
		}
		else {
			if (priv->radio_caps.capability & LRD_CAP_BT_DUP && enable_bt_dup ) {
				//Add Locally Administered Bit
				hw->wiphy->addresses[x].addr[0] |= 0x02;

				//Fix up permanent address
				hw->wiphy->perm_addr[0] |= 0x02;
			}
			addr++;
		}
	}

	lrd_set_vendor_commands(hw->wiphy);
	lrd_set_vendor_events(hw->wiphy);
}

static int lrd_regd_init(struct mwl_priv *priv)
{
	struct ieee80211_hw *hw = priv->hw;
	int  rc = 0;

	/* hook regulatory domain change notification */
	hw->wiphy->reg_notifier      = mwl_reg_notifier;
	hw->wiphy->regulatory_flags |= REGULATORY_STRICT_REG;

	/*Reset alpha so we send driver hint if this is a reinit*/
	memset(priv->reg.cc.alpha2, 0, sizeof(priv->reg.cc.alpha2));
	priv->reg.regulatory_set = false;
	priv->reg.pn = 0;

	if (mwl_fwcmd_get_fw_region_code(hw, &priv->reg.otp.region)) {
		/* If we fail to retrieve region, default to WW */
		wiphy_err(priv->hw->wiphy, "Failed to retrieve regulatory region.\n");

		priv->reg.otp.region = REG_CODE_WW;
		memset(priv->reg.otp.alpha2,'9', sizeof(priv->reg.otp.alpha2));

		if (!priv->mfg_mode) {
			rc = -1;
		}

		goto done;
	}

	if (mwl_fwcmd_get_region_mapping(priv->hw, (struct mwl_region_mapping*)&priv->reg.otp.alpha2)) {
		/* If we fail to retrieve mapping, default to 99 */
		wiphy_err(priv->hw->wiphy, "Failed to retrieve region mapping\n");
		memset(priv->reg.otp.alpha2,'9', sizeof(priv->reg.otp.alpha2));
	}

	if (!mwl_is_unknown_country(&priv->reg.otp)) {
		//If we know the country, fixup channel set now and not
		//wait for the callback.
		lrd_fixup_channels(priv, &priv->reg.otp);
	}

done:
	return rc;
}

static void lrd_request_cc_db(struct mwl_priv *priv)
{
	if (!mwl_is_unknown_country(&priv->reg.otp)) {
		lrd_send_hint(priv, &priv->reg.otp);
	}

	request_firmware_nowait( THIS_MODULE, true, REG_PWR_DB_NAME, priv->dev, GFP_KERNEL, priv, lrd_cc_cb);
}

static void remain_on_channel_expire(struct timer_list *t)
{
    struct mwl_priv *priv = from_timer(priv, t, roc.roc_timer);
	struct ieee80211_hw *hw = priv->hw;

	priv->roc.tmr_running = false;
	if (!priv->roc.in_progress)
		return;

	ieee80211_remain_on_channel_expired(hw);
}

void timer_routine(struct timer_list *t)
{
	struct mwl_priv *priv = from_timer(priv, t, period_timer);
	struct mwl_ampdu_stream *stream;
	struct mwl_sta *sta_info;
	struct mwl_tx_info *tx_stats;
	int i;

	spin_lock_bh(&priv->stream_lock);
	for (i = 0; i < SYSADPT_TX_AMPDU_QUEUES; i++) {
		stream = &priv->ampdu[i];

		if (stream->state == AMPDU_STREAM_ACTIVE) {
			sta_info = mwl_dev_get_sta(stream->sta);
			tx_stats = &sta_info->tx_stats[stream->tid];

			if ((jiffies - tx_stats->start_time > HZ) &&
			    (tx_stats->pkts < SYSADPT_AMPDU_PACKET_THRESHOLD)) {
				ieee80211_stop_tx_ba_session(stream->sta, stream->tid);
			}

			if (jiffies - tx_stats->start_time > HZ) {
				tx_stats->pkts = 0;
				tx_stats->start_time = jiffies;
			}
		}
	}
	spin_unlock_bh(&priv->stream_lock);

	mod_timer(&priv->period_timer, jiffies +
		  msecs_to_jiffies(SYSADPT_TIMER_WAKEUP_TIME));
}

void ds_routine(struct timer_list *t)
{
	struct mwl_priv *priv = from_timer(priv, t, ds_timer);
	struct ieee80211_hw *hw = priv->hw;
	struct ieee80211_conf *conf = &hw->conf;

	if( priv->ds_enable != DS_ENABLE_ON || priv->shutdown) {
		return;
	}

	if (conf->flags & IEEE80211_CONF_IDLE) {
		queue_work(priv->lrd_workq, &priv->ds_work);
	}
}

void lrd_cc_event(struct work_struct *work)
{
	struct mwl_priv     *priv = container_of(work,struct mwl_priv, reg.event);
	struct ieee80211_hw *hw   = priv->hw;
	struct cc_info info;

	u32 pn     = 0;
	u32 result = 0;
	bool valid = false;

	wiphy_debug(hw->wiphy, "Regulatory Event running...\n");

	mutex_lock(&priv->reg.mutex);

	/* Add delay to allow firmware to decrypt power table blob sent
	this especially important on multicore CPU where workqueue
	is scheduled immediately on the other core */
	lrdmwl_delay(1000);

	if (!lrd_fwcmd_lrd_get_power_table_result(hw, &result, &pn))
		valid = !result && pn >= priv->reg.pn;

	if (!valid) {
		wiphy_err(hw->wiphy,
			"Power table entry failed firmware validation %s.\n",
			priv->reg.regulatory_set ? "reverting to OTP" : "");
		goto exit;
	}

	/* Get current country code and region */
	if (mwl_fwcmd_get_region_mapping(hw, (struct mwl_region_mapping *)info.alpha2)) {
		/* If we fail to retrieve mapping, default to WW */
		wiphy_err(hw->wiphy, "Failed to retrieve region mapping.\n");
		valid = false;
		goto exit;
	}

	/* Get region code while we hold reg.mutex */
	if (mwl_fwcmd_get_fw_region_code(hw, &info.region)) {
		/* If we fail to retrieve region, default to WW */
		wiphy_err(hw->wiphy, "Failed to retrieve regulatory region.\n");
		valid = false;
		goto exit;
	}

	/* Validate CC exists in CRDA */
	wiphy_debug(hw->wiphy, "Checking CRDA for Country Code %c%c.\n",
		info.alpha2[0], info.alpha2[1]);

	rtnl_lock();

	if (check_regdb(info.alpha2) < 0) {
		wiphy_err(hw->wiphy,
			"Country Code %x%x does not exist in CRDA database.\n",
			info.alpha2[0], info.alpha2[1]);
		valid = false;
	}

	rtnl_unlock();

exit:
	if (valid) {
		wiphy_debug(hw->wiphy, "Country Code %c%c is valid.\n",
			info.alpha2[0], info.alpha2[1]);

		/* Update driver hint */
		lrd_send_hint_no_lock(priv, &info);
	}
	else if (priv->reg.regulatory_set) {
		/* Reset power table */
		lrd_fwcmd_lrd_reset_power_table(priv->hw);

		/* Update driver hint */
		lrd_send_hint_no_lock(priv, &priv->reg.otp);
	}

	//lrd_dump_max_pwr_table(priv);

	mutex_unlock(&priv->reg.mutex);
}

void lrd_awm_routine(struct timer_list *t)
{
	struct mwl_priv *priv = from_timer(priv, t, reg.timer_awm);

	queue_work(priv->lrd_workq, &priv->reg.awm);
}

static void lrd_awm_expire(struct work_struct *work)
{
	struct mwl_priv  *priv = container_of(work,
                        struct mwl_priv, reg.awm);

	wiphy_debug(priv->hw->wiphy, "AWM Event running %d...\n",
		priv->reg.regulatory_set);

	mutex_lock(&priv->reg.mutex);

	if (priv->reg.regulatory_set) {
		/* Reset power table */
		lrd_fwcmd_lrd_reset_power_table(priv->hw);

		lrd_send_hint_no_lock(priv, &priv->reg.otp);
	}

	//lrd_dump_max_pwr_table(priv);

	mutex_unlock(&priv->reg.mutex);
}

static void mwl_ds_work(struct work_struct *work)
{
	struct mwl_priv  *priv = container_of(work,
                        struct mwl_priv, ds_work);

	if (priv->mfg_mode || priv->recovery_in_progress) {
		return;
	}

	mwl_fwcmd_enter_deepsleep(priv->hw);
}

static void lrd_stop_shutdown_workq(struct work_struct *work)
{
	struct mwl_priv  *priv = container_of(work, struct mwl_priv,
		stop_shutdown_work.work);

	mwl_shutdown_sw(priv, true);

	if (priv->if_ops.down_pwr != NULL)
		priv->if_ops.down_pwr(priv);
}

static int mwl_wl_init(struct mwl_priv *priv)
{
	struct ieee80211_hw *hw   = priv->hw;
	struct mwl_if_ops *if_ops = &priv->if_ops;
	int rc = 0;

	/* Setup queues */
	priv->rx_defer_workq = alloc_workqueue("lrdwifi-rx_defer_workq",
	                       WQ_HIGHPRI | WQ_MEM_RECLAIM | WQ_UNBOUND, 1);
	INIT_WORK(&priv->rx_defer_work, mwl_rx_defered_handler);
	skb_queue_head_init(&priv->rx_defer_skb_q);

	priv->lrd_workq = alloc_workqueue("lrdwifi-workq",
	                 WQ_HIGHPRI | WQ_MEM_RECLAIM | WQ_UNBOUND, 1);
	INIT_WORK(&priv->ds_work, mwl_ds_work);
	INIT_DELAYED_WORK(&priv->stop_shutdown_work, lrd_stop_shutdown_workq);
	INIT_WORK(&priv->reg.event, lrd_cc_event);
	INIT_WORK(&priv->reg.awm, lrd_awm_expire);

	priv->reg.regulatory_set = false;
	priv->disable_2g         = false;
	priv->disable_5g         = false;

	if (!SISO_mode)
		priv->ant_tx_bmp = if_ops->mwl_chip_tbl.antenna_tx;
	else
		priv->ant_tx_bmp = SISO_mode & MWL_8997_DEF_TX_ANT_BMP;
	priv->ant_tx_num = MWL_TXANT_BMP_TO_NUM(priv->ant_tx_bmp);

	if (!SISO_mode)
		priv->ant_rx_bmp = if_ops->mwl_chip_tbl.antenna_rx;
	else
		priv->ant_rx_bmp = SISO_mode & MWL_8997_DEF_RX_ANT_BMP;
	priv->ant_rx_num = MWL_RXANT_BMP_TO_NUM(priv->ant_rx_bmp);

	priv->ds_enable = priv->host_if == MWL_IF_SDIO ? ds_enable : DS_ENABLE_OFF;
	priv->ps_mode   = 0;

	priv->ap_macids_supported    = 0x0000ffff;
	priv->sta_macids_supported   = ((1UL << SYSADPT_NUM_OF_STA) - 1) << 16;
	priv->adhoc_macids_supported = 0x00000001;
	priv->macids_used            = 0;

	INIT_LIST_HEAD(&priv->vif_list);
	INIT_LIST_HEAD(&priv->sta_list);

	/* Set default radio state, preamble and wmm */
	priv->radio_on             = false;
	priv->radio_short_preamble = false;
	priv->wmm_enabled          = false;

	priv->powinited               = 0;
	priv->csa_active              = false;
	priv->dfs_chirp_count_min     = 5;
	priv->dfs_chirp_time_interval = 1000;
	priv->dfs_pw_filter           = 0;
	priv->dfs_min_num_radar       = 5;
	priv->dfs_min_pri_count       = 4;

	/* Handle watchdog ba events */
	INIT_WORK(&priv->watchdog_ba_handle, mwl_watchdog_ba_events);
	INIT_WORK(&priv->chnl_switch_handle, mwl_chnl_switch_event);

	priv->is_tx_done_schedule  = false;
	priv->qe_trigger_num       = 0;
	priv->qe_trigger_time      = jiffies;
	priv->txq_limit            = SYSADPT_TX_QUEUE_LIMIT;

	priv->is_rx_defer_schedule = false;
	priv->recv_limit           = SYSADPT_RECEIVE_LIMIT;

	priv->cmd_timeout          = false;

	mutex_init(&priv->fwcmd_mutex);
	mutex_init(&priv->reg.mutex);
	spin_lock_init(&priv->tx_desc_lock);
	spin_lock_init(&priv->vif_lock);
	spin_lock_init(&priv->sta_lock);
	spin_lock_init(&priv->stream_lock);

	init_waitqueue_head(&priv->tx_flush_wq);

	atomic_set(&priv->null_scan_count, null_scan_count);

	/* set up band information for 2.4G */
	memset(&priv->band_24, 0,sizeof(struct ieee80211_supported_band));
	if (!priv->disable_2g) {
		BUILD_BUG_ON(sizeof(priv->channels_24) != sizeof(mwl_channels_24));
		memcpy(priv->channels_24, mwl_channels_24,sizeof(mwl_channels_24));

		BUILD_BUG_ON(sizeof(priv->rates_24) != sizeof(mwl_rates_24));
		memcpy(priv->rates_24, mwl_rates_24, sizeof(mwl_rates_24));

		priv->band_24.band       = NL80211_BAND_2GHZ;
		priv->band_24.channels   = priv->channels_24;
		priv->band_24.n_channels = ARRAY_SIZE(mwl_channels_24);
		priv->band_24.bitrates   = priv->rates_24;
		priv->band_24.n_bitrates = ARRAY_SIZE(mwl_rates_24);
	}

	/* set up band information for 5G */
	memset(&priv->band_50, 0, sizeof(struct ieee80211_supported_band));
	if (!priv->disable_5g) {
		BUILD_BUG_ON(sizeof(priv->channels_50) != sizeof(mwl_channels_50));
		memcpy(priv->channels_50, mwl_channels_50,sizeof(mwl_channels_50));

		BUILD_BUG_ON(sizeof(priv->rates_50) != sizeof(mwl_rates_50));
		memcpy(priv->rates_50, mwl_rates_50, sizeof(mwl_rates_50));

		priv->band_50.band       = NL80211_BAND_5GHZ;
		priv->band_50.channels   = priv->channels_50;
		priv->band_50.n_channels = ARRAY_SIZE(mwl_channels_50);
		priv->band_50.bitrates   = priv->rates_50;
		priv->band_50.n_bitrates = ARRAY_SIZE(mwl_rates_50);
	}

	/* bus specific device registration */
	if (priv->if_ops.register_dev)
		rc = priv->if_ops.register_dev(priv);
	else
		rc = -ENXIO;
	if (rc) {
		wiphy_err(hw->wiphy, "%s: fail to register device\n",
			  MWL_DRV_NAME);
		goto err_wl_init;
	}

	/* Begin querying FW for information */
	rc = mwl_fwcmd_get_hw_specs(hw);
	if (rc) {
		wiphy_err(hw->wiphy, "%s: fail to get HW specifications\n",
			  MWL_DRV_NAME);
		goto err_wl_init;
	}
	else {
		if (priv->hw_data.fw_release_num == NOT_LRD_HW) {
			wiphy_err(hw->wiphy,
			     "Detected non Laird hardware: 0x%x\n", priv->hw_data.fw_release_num);
			rc = -ENODEV;
			goto err_wl_init;
		}
	}

	/* interface specific initialization after fw is loaded and hw specs retrieved .. */
	if (priv->if_ops.init_if_post) {
		if (priv->if_ops.init_if_post(priv)) {
			goto err_wl_init;
		}
	}

	if (!priv->mfg_mode) {
		rc = lrd_fwcmd_lrd_get_caps(hw, &priv->radio_caps);
		if (rc) {
			wiphy_err(hw->wiphy, "Fail to retrieve radio capabilities %x\n", rc);
			memset(&priv->radio_caps, 0, sizeof(priv->radio_caps));
		}

		if (priv->ant_gain_adjust) {
			rc = lrd_fwcmd_lrd_set_ant_gain_adjust(hw, priv->ant_gain_adjust);
			if (rc) {
				wiphy_err(hw->wiphy, "Antenna gain adjustment 0x%x specified but failed to send!\n", priv->ant_gain_adjust);
				goto err_wl_init;
			}
		}
	}

	if (priv->radio_caps.capability & LRD_CAP_SU60) {
		rc = mwl_thermal_register(priv);
		if (rc) {
			wiphy_err(hw->wiphy, "%s: fail to register thermal framework\n",
				MWL_DRV_NAME);
			goto err_wl_init;
		}
	}

	rc = mwl_fwcmd_set_hw_specs(priv->hw);
	if (rc) {
		wiphy_err(priv->hw->wiphy, "%s: fail to set HW specifications\n",
			  MWL_DRV_NAME);
		goto err_wl_init;
	}

	rc = mwl_fwcmd_set_cfg_data(hw, 2);
	if(rc) {
		wiphy_err(hw->wiphy, "%s: fail to download calibaration data\n",
			MWL_DRV_NAME);
	}

	/* Initialize regulatory info */
	if (lrd_regd_init(priv)) {
		wiphy_err(hw->wiphy, "%s: fail to register regulatory\n", MWL_DRV_NAME);
		goto err_wl_init;
	}

	rc = mwl_fwcmd_dump_otp_data(hw);
	if (rc) {
		wiphy_info(hw->wiphy, "OTP Dump failed\n");
	}

	mwl_fwcmd_radio_disable(hw);
	mwl_fwcmd_rf_antenna(hw, priv->ant_tx_bmp, priv->ant_rx_bmp);

	if (priv->stop_shutdown)
		queue_delayed_work(priv->lrd_workq, &priv->stop_shutdown_work,
			msecs_to_jiffies(stop_shutdown_timeout));

	/* Set IEEE HW Capabilities */
	mwl_set_ieee_hw_caps(priv);

	/* Register with MAC80211 */
	rc = ieee80211_register_hw(hw);
	if (rc) {
		wiphy_err(hw->wiphy, "%s: fail to register hw\n",
			  MWL_DRV_NAME);
		goto err_wl_init;
	}

	/* Request Regulatory DB */
	lrd_request_cc_db(priv);

	/* Success - dump relevant info to log */
	wiphy_info(hw->wiphy, "Radio Type %s%s%s (0x%x)\n", (priv->radio_caps.capability & LRD_CAP_SU60)?"SU60":"ST60",
	                                                  (priv->radio_caps.capability & LRD_CAP_440)?"_440":"",
	                                                  (priv->radio_caps.capability & LRD_CAP_SOM8MP)?"_SOM8MP":"",
	                                                   priv->radio_caps.capability);
	wiphy_info(hw->wiphy, "Num mac %d : OTP Version (%d)\n", priv->radio_caps.num_mac, priv->radio_caps.version);
	wiphy_info(hw->wiphy, "Firmware version: %d.%d.%d.%d\n", ((priv->hw_data.fw_release_num >> 24) & 0xff),
															 ((priv->hw_data.fw_release_num >> 16) & 0xff),
															 ((priv->hw_data.fw_release_num >> 8) & 0xff),
															 ((priv->hw_data.fw_release_num >> 0) & 0xff));
	wiphy_info(hw->wiphy, "Firmware OTP region: %x, country: %c%c\n", priv->reg.otp.region, priv->reg.otp.alpha2[0], priv->reg.otp.alpha2[1]);
	wiphy_info(priv->hw->wiphy, "Deep Sleep is %s\n",
		   priv->ds_enable == DS_ENABLE_ON ? "enabled": "disabled");

	wiphy_info(priv->hw->wiphy, "2G %s, 5G %s\n",
		   priv->disable_2g ? "disabled" : "enabled",
		   priv->disable_5g ? "disabled" : "enabled");

	wiphy_info(priv->hw->wiphy, "%d TX antennas, %d RX antennas. (%08x)/(%08x)\n",
		   priv->ant_tx_num, priv->ant_rx_num, priv->ant_tx_bmp, priv->ant_rx_bmp);

	if (!priv->mfg_mode && priv->ant_gain_adjust)
		wiphy_info(hw->wiphy, "Antenna gain adjustment in effect: 0x%x\n", priv->ant_gain_adjust);

	lrd_send_fw_event(priv->dev, true);

	return rc;

err_wl_init:
	destroy_workqueue(priv->lrd_workq);
	destroy_workqueue(priv->rx_defer_workq);

	return rc;
}

void mwl_wl_deinit(struct mwl_priv *priv)
{
	struct ieee80211_hw *hw = priv->hw;

#ifdef CONFIG_DEBUG_FS
	mwl_debugfs_remove(hw);
#endif

	cancel_delayed_work_sync(&priv->stop_shutdown_work);

	device_init_wakeup(priv->dev, false);
	lrd_send_fw_event(priv->dev, false);

	// priv->shutdown must be set prior to ieee80211_unregister_hw() so that
	// it is set before call to mwl_mac80211_stop()
	priv->shutdown = true;

	//Stop Timers
	del_timer_sync(&priv->period_timer);
	del_timer_sync(&priv->ds_timer);
	del_timer_sync(&priv->roc.roc_timer);
	del_timer_sync(&priv->reg.timer_awm);

	ieee80211_unregister_hw(hw);

	mwl_thermal_unregister(priv);

	/*Cancel iee80211 queue work itmes */
	cancel_work_sync(&priv->watchdog_ba_handle);
	cancel_work_sync(&priv->chnl_switch_handle);

	/* Cancel/Destroy allocated queues */
	if (priv->restart_workq) {
		cancel_work_sync(&priv->restart_work);
		destroy_workqueue(priv->restart_workq);
	}

	cancel_work_sync(&priv->rx_defer_work);
	destroy_workqueue(priv->rx_defer_workq);
	skb_queue_purge(&priv->rx_defer_skb_q);

	mwl_fwcmd_reset(hw);

	//cancel everything running on lrd work queue and destroy it
	cancel_work_sync(&priv->ds_work);
	cancel_work_sync(&priv->reg.event);
	cancel_work_sync(&priv->reg.awm);
	destroy_workqueue(priv->lrd_workq);

	if (priv->reg.db) {
		kfree(priv->reg.db);
	}

#ifdef CONFIG_SYSFS
	lrd_sysfs_remove(hw);
#endif
}
EXPORT_SYMBOL_GPL(mwl_wl_deinit);


void mwl_restart_ds_timer(struct mwl_priv *priv, bool force)
{
	struct ieee80211_conf *conf = &priv->hw->conf;

	if(priv->ds_enable != DS_ENABLE_ON || priv->shutdown || priv->recovery_in_progress) {
		return;
	}

	if ((conf->flags & IEEE80211_CONF_IDLE) || force) {
		mod_timer(&priv->ds_timer, jiffies + msecs_to_jiffies(1000));
	}
}
EXPORT_SYMBOL_GPL(mwl_restart_ds_timer);

void mwl_delete_ds_timer(struct mwl_priv *priv)
{
	del_timer_sync(&priv->ds_timer);

	//Make sure no work item outstanding
	cancel_work_sync(&priv->ds_work);
}
EXPORT_SYMBOL_GPL(mwl_delete_ds_timer);

void mwl_enable_ds(struct mwl_priv * priv)
{
	if (priv->ds_enable == DS_ENABLE_ON || priv->mfg_mode) {
		return;
	}

	priv->ds_enable = DS_ENABLE_ON;
	mwl_restart_ds_timer(priv,false);
	wiphy_info(priv->hw->wiphy, "Enabled DS\n");
}
EXPORT_SYMBOL_GPL(mwl_enable_ds);

void mwl_resume_ds(struct mwl_priv * priv)
{
	if (priv->ds_enable != DS_ENABLE_PAUSE || priv->mfg_mode) {
		return;
	}

	priv->ds_enable = DS_ENABLE_ON;
	mwl_restart_ds_timer(priv,false);
	wiphy_info(priv->hw->wiphy, "Resumed DS\n");
}
EXPORT_SYMBOL_GPL(mwl_resume_ds);

void mwl_pause_ds(struct mwl_priv* priv)
{
	struct ieee80211_hw *hw = priv->hw;

	if(priv->ds_enable != DS_ENABLE_ON) {
		return;
	}

	priv->ds_enable = DS_ENABLE_PAUSE;

	mwl_delete_ds_timer(priv);

	//Make sure card is awake when this function returns
	mwl_fwcmd_exit_deepsleep(hw);

	wiphy_info(priv->hw->wiphy, "Paused DS\n");
}
EXPORT_SYMBOL_GPL(mwl_pause_ds);

void mwl_disable_ds(struct mwl_priv * priv)
{
	struct ieee80211_hw *hw = priv->hw;

	if(priv->ds_enable == DS_ENABLE_OFF) {
		return;
	}

	priv->ds_enable = DS_ENABLE_OFF;

	//Kill timer
	mwl_delete_ds_timer(priv);

	// Don't try to access the radio while non-responsive
	if (!priv->recovery_in_progress) {
		//Make sure card is awake when this function returns
		mwl_fwcmd_exit_deepsleep(hw);
	}

	wiphy_info(priv->hw->wiphy, "Disabled DS\n");
}

EXPORT_SYMBOL_GPL(mwl_disable_ds);

static void lrd_radio_recovery_work(struct work_struct *work)
{
	struct mwl_priv *priv = container_of(work,
				struct mwl_priv, restart_work);
	int ret;

	wiphy_debug(priv->hw->wiphy, "%s: Initiating radio recovery!!\n",
		  MWL_DRV_NAME);

	// Restart radio hardware
	ret = priv->if_ops.hardware_restart(priv);
	if (ret) {
		wiphy_err(priv->hw->wiphy, "%s: Unable to restart radio!!\n",
			MWL_DRV_NAME);
		priv->recovery_in_progress = false;
		priv->recovery_owner = NULL;
	}
}

void lrd_radio_recovery(struct mwl_priv *priv)
{
	struct ieee80211_hw *hw = priv->hw;

	wiphy_info(priv->hw->wiphy, "%s: Radio recovery requested!\n", __func__);
	if (priv->recovery_in_progress)
	{
		wiphy_info(priv->hw->wiphy, "recovery_in_progress, skipping\n");
		return;
	}

	priv->recovery_in_progress = true;
	priv->recovery_in_progress_full = true;

	if (!priv->if_ops.hardware_restart)
	{
		wiphy_info(hw->wiphy, "%s: Radio recovery requested but no restart handler configured!\n",
			MWL_DRV_NAME);
		return;
	}

	// Initialize workq if it hasn't been already
	if (!priv->restart_workq)
	{
		priv->restart_workq = alloc_workqueue("lrdwifi-restart_workq",
						WQ_MEM_RECLAIM | WQ_UNBOUND | WQ_HIGHPRI, 1);
		INIT_WORK(&priv->restart_work, lrd_radio_recovery_work);
	}

	queue_work(priv->restart_workq, &priv->restart_work);
}
EXPORT_SYMBOL_GPL(lrd_radio_recovery);

int mwl_add_card(void *card, struct mwl_if_ops *if_ops,
	struct device_node *of_node)
{
	struct ieee80211_hw *hw;
	struct mwl_priv *priv;
	int rc = 0;

	hw = ieee80211_alloc_hw(sizeof(*priv), &mwl_mac80211_ops);
	if (!hw) {
		pr_err("%s: ieee80211 alloc failed\n",
		       MWL_DRV_NAME);
		rc = -ENOMEM;
		goto err_alloc_hw;
	}

	priv       = hw->priv;
	priv->hw   = hw;
	priv->intf = card;

	priv->init_complete = false;

	priv->tx_amsdu_enable = tx_amsdu_enable;

#ifdef CONFIG_LRDMWL_FIPS
	priv->host_crypto = fips_enabled && fips_wifi_enabled;
	if (priv->host_crypto)
		host_crypto_mode = 1;
#endif

	// BZ19005: host_crypto_mode module parameter to force host crypto
	// in most cases, driver will now automatically select hw/host crypto
	// exception: station mode with invalid cipher combinations
	// station mode with UC:HW/MC:Host (e.g. UC:CCMP-128, MC:GCMP-256)
	if (host_crypto_mode)
		priv->host_crypto = 1;

	if (priv->host_crypto)
		wiphy_info(priv->hw->wiphy, "Using host crypto.\n");

	/* Save interface specific operations in adapter */
	memmove(&priv->if_ops, if_ops, sizeof(struct mwl_if_ops));

	/* card specific initialization has been deferred until now .. */
	if (priv->if_ops.init_if) {
		if (priv->if_ops.init_if(priv))
			goto err_init_if;
	}

	rc = lrd_probe_of_wowlan(priv, of_node);
	if (rc == -EPROBE_DEFER)
		goto err_of;

	priv->ant_gain_adjust = ant_gain_adjust;
	if (of_node) {
		priv->stop_shutdown = stop_shutdown ||
			of_property_read_bool(of_node, "remove-power-on-link-down");

		/*
		 * If antenna gain adjustment is specified as both module parameter and
		 * in device tree, device tree wins
		 */
		of_property_read_u32(of_node, "ant-gain-adjust", &priv->ant_gain_adjust);
	}

	/* Setup timers */
	timer_setup(&priv->roc.roc_timer, remain_on_channel_expire, 0);
	timer_setup(&priv->period_timer,  timer_routine, 0);
	timer_setup(&priv->ds_timer, ds_routine, 0);
	timer_setup(&priv->reg.timer_awm, lrd_awm_routine, 0);

	rc = mwl_fw_dnld_and_init(priv);

	// mwl_fw_dnld_and_init returns -EINPROGRESS when target is reset
	// as part of the initialization sequence.  This is normal behavior
	// for USB at initial powerup, and for PCI in scenarios where firmware
	// has already been loaded and a Function Level Reset is initiated to restart.
	// In the case of PCI, the reset/restart handler is responsible for
	// calling mwl_fw_dnld_and_init again, but remainder of this init sequence
	// must complete successfully so driver remains loaded.
	if (rc) {
		if ((rc == -EINPROGRESS) && (priv->host_if == MWL_IF_PCIE))
			rc = 0;
		else
			goto err_dnld_and_init;
	}

	return rc;

err_dnld_and_init:
	device_init_wakeup(priv->dev, false);

err_of:
	priv->if_ops.cleanup_if(priv);

err_init_if:
	mwl_ieee80211_free_hw(priv);

err_alloc_hw:
	return rc;
}
EXPORT_SYMBOL_GPL(mwl_add_card);

int mwl_fw_dnld_and_init(struct mwl_priv *priv)
{
	int rc;
	struct ieee80211_hw *hw;

	hw = priv->hw;

	rc = mwl_init_firmware(priv);
	if (rc) {
		if (rc != -EINPROGRESS)
			wiphy_err(hw->wiphy, "%s: failed to initialize firmware\n",
				MWL_DRV_NAME);
		goto err_init_firmware;
	}

	rc = mwl_wl_init(priv);
	if (rc) {
		wiphy_err(hw->wiphy, "%s: failed to initialize wireless lan\n",
			  MWL_DRV_NAME);
		goto err_wl_init;
	}

	/* Start Timers */
	mod_timer(&priv->period_timer, jiffies + msecs_to_jiffies(SYSADPT_TIMER_WAKEUP_TIME));

	/* Start DS timer after init where radio is registered with mac802.11
	 * and IEEE80211_CONF_IDLE flag is setup
	 */
	mwl_restart_ds_timer(priv, true);

#ifdef CONFIG_SYSFS
	lrd_sysfs_init(hw);
#endif

#ifdef CONFIG_DEBUG_FS
	mwl_debugfs_init(hw);
#endif

	priv->init_complete = true;
	return 0;

err_wl_init:
	mwl_disable_ds(priv);

err_init_firmware:
	return rc;
}
EXPORT_SYMBOL_GPL(mwl_fw_dnld_and_init);


/*
 * This function gets called during restart
 */
int mwl_shutdown_sw(struct mwl_priv *priv, bool suspend)
{
	struct ieee80211_hw *hw = priv->hw;
	struct mwl_vif *mwl_vif, *tmp_vif;
	struct mwl_sta *mwl_sta, *tmp_sta;

	if (suspend) {
		priv->recovery_in_progress = true;
		priv->recovery_in_progress_full = true;
		priv->mac_init_complete = true;
	}
	else
		cancel_delayed_work_sync(&priv->stop_shutdown_work);

	WARN_ON(!priv->recovery_in_progress);

	lrd_send_fw_event(priv->dev, false);

	wiphy_debug(priv->hw->wiphy, "%s: Shutting down software...\n", MWL_DRV_NAME);

	/*
	 * Disable radio in error recovery scenarios
	 *  Stop queues
	 *  Disable tasklets
	 *  Return completed TX SKBs
	 * Note - mwl_mac80211_stop is called directly by mac80211 in
	 * standard non-error recovery scenarios
	 */
	if (!suspend && priv->mac_started)
		mwl_mac80211_stop(hw);

	del_timer_sync(&priv->roc.roc_timer);
	del_timer_sync(&priv->period_timer);
	del_timer_sync(&priv->reg.timer_awm);
	mwl_disable_ds(priv);

	cancel_work_sync(&priv->watchdog_ba_handle);
	cancel_work_sync(&priv->chnl_switch_handle);
	cancel_work_sync(&priv->rx_defer_work);
	cancel_work_sync(&priv->reg.event);
	cancel_work_sync(&priv->reg.awm);
	skb_queue_purge(&priv->rx_defer_skb_q);

	/*
	 * Existing interfaces and stations are re-added by ieee80211_reconfig
	 * with the expectation the driver is in a clean state.
	 * Remove private driver stations/interfaces here
	 */
	list_for_each_entry_safe(mwl_sta, tmp_sta, &priv->sta_list, list)
	{
		mwl_mac80211_sta_remove(hw, mwl_sta->vif, mwl_sta->sta);
	}

	list_for_each_entry_safe(mwl_vif, tmp_vif, &priv->vif_list, list)
	{
		mwl_mac80211_remove_vif(priv, mwl_vif->vif);
	}

	if (priv->if_ops.down_dev)
		priv->if_ops.down_dev(priv);

	return 0;
}
EXPORT_SYMBOL_GPL(mwl_shutdown_sw);

void lrd_send_restart_event(struct wiphy *wiphy, struct ieee80211_vif *vif, uint32_t reason)
{
	struct sk_buff *skb;
	struct wireless_dev *wdev = 0;

	if (vif)
		wdev = ieee80211_vif_to_wdev(vif);

	//Send Restart Event
	skb = cfg80211_vendor_event_alloc(wiphy, wdev, 100, (LRD_VENDOR_EVENT_RESTART & 0x7FFF), GFP_KERNEL);

	if (skb) {
		nla_put_u32(skb, LRD_ATTR_RESTART_REASON, reason);

		/* Send the event */
		cfg80211_vendor_event(skb, GFP_KERNEL);
	}
}

/* This function gets called during interface restart. Required
 * code is extracted from mwl_add_card()/mwl_wl_init()
 */
int mwl_reinit_sw(struct mwl_priv *priv, bool suspend)
{
	struct ieee80211_hw *hw = priv->hw;
	int rc;

	if (!priv->recovery_in_progress) {
		cancel_delayed_work_sync(&priv->stop_shutdown_work);
		return 0;
	}

	wiphy_info(priv->hw->wiphy, "%s: Re-initializing software...\n", MWL_DRV_NAME);

	// Save task context as mechanism to identify calls made in context
	// of restart sequence
	priv->recovery_owner = current;

	if (priv->if_ops.up_dev)
		priv->if_ops.up_dev(priv);

	// Re-initialize priv members that may have changed since initialization
	priv->ps_mode = 0;
	priv->radio_on = false;
	priv->radio_short_preamble = false;
	priv->wmm_enabled = false;
	priv->csa_active = false;
	priv->ds_enable = priv->host_if == MWL_IF_SDIO ? ds_enable : DS_ENABLE_OFF;
	priv->is_tx_done_schedule = false;
	priv->qe_trigger_num = 0;
	priv->qe_trigger_time = jiffies;
	priv->running_bsses = 0;
	priv->sw_scanning = false;

	priv->cmd_timeout = false;

	atomic_set(&priv->null_scan_count, null_scan_count);

	rc = mwl_init_firmware(priv);
	if (rc) {
		wiphy_err(hw->wiphy, "%s: fail to initialize firmware\n", MWL_DRV_NAME);
		goto err_init;
	}

	rc = mwl_fwcmd_get_hw_specs(hw);
	if (rc) {
		wiphy_err(hw->wiphy, "%s: fail to get HW specifications\n",
			  MWL_DRV_NAME);
		goto err_init;
	}
	else {
		if (priv->hw_data.fw_release_num == NOT_LRD_HW) {
			wiphy_err(hw->wiphy,
			     "Detected non Laird hardware: 0x%x\n", priv->hw_data.fw_release_num);
			rc = -ENODEV;
			goto err_init;
		}
		else {
			wiphy_info(hw->wiphy,
			     "firmware version: 0x%x\n", priv->hw_data.fw_release_num);
		}
	}

	/* card specific initialization after fw is loaded .. */
	if (priv->if_ops.init_if_post) {
		if (priv->if_ops.init_if_post(priv)) {
			goto err_init;
		}
	}

	if (!priv->mfg_mode) {
		rc = lrd_fwcmd_lrd_get_caps(hw, &priv->radio_caps);

		if (rc) {
			wiphy_err(hw->wiphy, "Fail to retrieve radio capabilities %x\n", rc);
			memset(&priv->radio_caps, 0, sizeof(priv->radio_caps));
		}

		if (priv->ant_gain_adjust) {
			rc = lrd_fwcmd_lrd_set_ant_gain_adjust(hw, priv->ant_gain_adjust);
			if (rc) {
				wiphy_err(hw->wiphy, "Antenna gain adjustment 0x%x specified but failed to send!\n", priv->ant_gain_adjust);
				goto err_init;
			}
		}
	}

	wiphy_info(hw->wiphy, "Radio Type %s%s%s (0x%x)\n", (priv->radio_caps.capability & LRD_CAP_SU60)?"SU60":"ST60",
	                                                  (priv->radio_caps.capability & LRD_CAP_440)?"_440":"",
	                                                  (priv->radio_caps.capability & LRD_CAP_SOM8MP)?"_SOM8MP":"",
	                                                   priv->radio_caps.capability);

	rc = mwl_fwcmd_set_hw_specs(priv->hw);
	if (rc) {
		wiphy_err(priv->hw->wiphy, "%s: fail to set HW specifications\n",
			  MWL_DRV_NAME);
		goto err_init;
	}

	/* Initialize regulatory info */
	if (lrd_regd_init(priv)) {
		wiphy_err(hw->wiphy, "%s: fail to register regulatory\n", MWL_DRV_NAME);
		goto err_init;
	}

	mwl_fwcmd_radio_disable(hw);
	mwl_fwcmd_rf_antenna(hw, priv->ant_tx_bmp, priv->ant_rx_bmp);

	priv->recovery_in_progress = false;
	priv->recovery_owner = NULL;

	if (!suspend) {
		wiphy_err(priv->hw->wiphy, "%s: Restarting mac80211...\n",
				  MWL_DRV_NAME);

		ieee80211_restart_hw(hw);
	}
	else {
		lrd_send_restart_event(priv->hw->wiphy, 0, LRD_REASON_RESUME);
	}

	/* Request Regulatory DB */
	lrd_request_cc_db(priv);

	mod_timer(&priv->period_timer, jiffies +
		  msecs_to_jiffies(SYSADPT_TIMER_WAKEUP_TIME));
	mwl_restart_ds_timer(priv, true);

	lrd_send_fw_event(priv->dev, true);

	if (priv->stop_shutdown  && !priv->mac_init_complete)
		queue_delayed_work(priv->lrd_workq, &priv->stop_shutdown_work,
			msecs_to_jiffies(2000));

err_init:

	if (rc) {
		if (priv->if_ops.down_dev)
			priv->if_ops.down_dev(priv);

		wiphy_err(hw->wiphy, "%s: fail to re-initialize wireless lan!\n",
			  MWL_DRV_NAME);
	}

	return rc;
}
EXPORT_SYMBOL_GPL(mwl_reinit_sw);

#ifdef CONFIG_PM
static irqreturn_t lrd_irq_wakeup_handler(int irq, void *dev_id)
{
	struct mwl_priv *priv = dev_id;

	/* Notify PM core we are wakeup source */
	pm_wakeup_event(priv->dev, 0);

	wiphy_dbg(priv->hw->wiphy, "Wake by Wi-Fi\n");

	return IRQ_HANDLED;
}

int lrd_probe_of_wowlan(struct mwl_priv *priv, struct device_node *of_node)
{
	int ret;

	if ((priv->host_if != MWL_IF_SDIO) || !of_node) {
		priv->wow.irq_wakeup = -ENODEV;
		return 0;
	}

	ret = of_irq_get(of_node, 0);
	if (ret <= 0) {
		if (ret != -EPROBE_DEFER)
			wiphy_err(priv->hw->wiphy,
				"Fail to parse irq_wakeup from device tree\n");
		priv->wow.irq_wakeup = -ENODEV;
		return ret;
	}

	priv->wow.irq_wakeup = ret;

	irq_set_status_flags(priv->wow.irq_wakeup, IRQ_NOAUTOEN);
	ret = devm_request_threaded_irq(priv->dev, priv->wow.irq_wakeup, NULL,
			   lrd_irq_wakeup_handler, IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
			   "wifi_wake", priv);
	if (ret) {
		wiphy_err(priv->hw->wiphy, "Failed to request irq_wakeup %d (%d)\n",
			priv->wow.irq_wakeup, ret);
		return ret;
	}

	ret = device_init_wakeup(priv->dev, true);
	if (ret) {
		wiphy_err(priv->hw->wiphy, "Fail to init wake source (%d)\n", ret);
		return ret;
	}

	wiphy_info(priv->hw->wiphy, "Added WoWLan interrupt\n");

	return 0;
}

/* Disable platform specific wakeup interrupt */
void lrd_disable_wowlan(struct mwl_priv *priv)
{
	if (priv->wow.irq_wakeup >= 0) {
		disable_irq_wake(priv->wow.irq_wakeup);
		disable_irq_nosync(priv->wow.irq_wakeup);
	}
}
EXPORT_SYMBOL_GPL(lrd_disable_wowlan);

/* Enable platform specific wakeup interrupt */
void lrd_enable_wowlan(struct mwl_priv *priv)
{
	/* Enable platform specific wakeup interrupt */
	if (priv->wow.irq_wakeup >= 0) {
		enable_irq(priv->wow.irq_wakeup);
		enable_irq_wake(priv->wow.irq_wakeup);
	}
}
EXPORT_SYMBOL_GPL(lrd_enable_wowlan);

void lrd_report_wowlan_wakeup(struct mwl_priv *priv)
{
	int x = 0;
	struct ieee80211_vif *vif;
	struct ieee80211_hw *hw = priv->hw;
	struct cfg80211_wowlan_wakeup    wakeup;
	struct cfg80211_wowlan_nd_info  *nd_info  = NULL;
	struct cfg80211_wowlan_nd_match *nd_match = NULL;;

	memset(&wakeup, 0, sizeof(wakeup));
	wakeup.pattern_idx = -1;

	switch( priv->wow.results.reason) {
		case MWL_RX_EVENT_WOW_LINKLOSS_DETECT:
			wiphy_info(hw->wiphy, "WOW link loss detected\n");
			wakeup.disconnect = true;
		break;

		case MWL_RX_EVENT_WOW_AP_DETECT:
			nd_info = kzalloc(sizeof(struct cfg80211_wowlan_nd_info)    +
			                  sizeof(struct cfg80211_wowlan_nd_match *) +
			                  sizeof(struct cfg80211_wowlan_nd_match)   +
			                  sizeof(u32) * priv->wow.results.n_channels,
			                  GFP_KERNEL);

			wiphy_info(hw->wiphy, "WOW AP in range detected\n");

			/* Fill in nd_info */
			nd_info->n_matches = 1;
			nd_match =  (struct cfg80211_wowlan_nd_match*)((u8*)nd_info->matches + sizeof(struct cfg80211_wowlan_nd_info*));
			nd_info->matches[0] = nd_match;

			/* nd_match -> ssid*/
			nd_match->ssid.ssid_len = min(priv->wow.ssidList[0].ssidLen, (u8)sizeof(nd_match->ssid.ssid));
			memcpy(nd_match->ssid.ssid, priv->wow.ssidList[0].ssid, nd_match->ssid.ssid_len);

			/* nd_match->channels */
			nd_match->n_channels = priv->wow.results.n_channels;
			for (x = 0; x < nd_match->n_channels; x++) {
				nd_match->channels[x] = priv->wow.results.channels[x];
			}

			wakeup.net_detect = nd_info;
		break;

		case MWL_RX_EVENT_WOW_RX_DETECT:
			/* We are treating the packet as a flag, rather than data */
			wiphy_info(hw->wiphy, "WOW rx packet detected\n");
			wakeup.packet = (void*)true;
			wakeup.packet_80211 = true;
			wakeup.packet_present_len = 0;
		break;
	}

	vif = priv->wow.results.mwl_vif->vif;
	ieee80211_report_wowlan_wakeup(vif, &wakeup, GFP_KERNEL);

	if (nd_info) {
		kfree(nd_info);
	}
}
#endif

module_param(cal_data_cfg, charp, 0);
MODULE_PARM_DESC(cal_data_cfg, "Calibration data file name");

module_param(wmm_turbo, int, 0);
MODULE_PARM_DESC(wmm_turbo, "WMM Turbo mode 0:Disable 1:Enable");

module_param(EDMAC_Ctrl, int, 0);
MODULE_PARM_DESC(EDMAC_Ctrl, "EDMAC CFG: BIT0:2G_enbl, BIT1:5G_enbl, " \
                             "BIT[4:11]: 2G_Offset, BIT[12:19]:5G_offset, " \
                             "BIT[20:27]:Queue_lock, BIT[28]: MCBC_QLock, " \
                             "BIT[29]: BCN_DSBL");

module_param(tx_amsdu_enable, int, 0);
MODULE_PARM_DESC(tx_amsdu_enable, "Tx AMSDU enable/disable");

module_param(SISO_mode, uint, 0444);
MODULE_PARM_DESC(SISO_mode, "SISO mode 0:Disable 1:Ant0 2:Ant1");

module_param(lrd_debug, uint, 0644);
MODULE_PARM_DESC(lrd_debug, "Debug mode 0:Disable 1:Enable");

module_param(ds_enable, uint, 0444);
MODULE_PARM_DESC(ds_enable, "Deep Sleep mode 0:Disable 1:Enable");

module_param(enable_bt_dup, uint, 0444);
MODULE_PARM_DESC(enable_bt_dup, "Enable coexistence with duplicate BT address 0:Disable 1:Enable");

module_param(mfg_mode, int, 0);
MODULE_PARM_DESC(mfg_mode, "MFG mode 0:disable 1:enable");

module_param(null_scan_count, int, 0);
MODULE_PARM_DESC(null_scan_count, "Null scan response recovery count");

module_param(ant_gain_adjust, uint, 0444);
MODULE_PARM_DESC(ant_gain_adjust, "Antenna gain adjustment");

module_param(stop_shutdown, uint, 0444);
MODULE_PARM_DESC(stop_shutdown, "Power off when stopped 0:Disable 1:Enable");

module_param(stop_shutdown_timeout, uint, 0644);
MODULE_PARM_DESC(stop_shutdown_timeout, "Timeout waiting for connection at boot, ms");

module_param(host_crypto_mode, uint, 0444);
MODULE_PARM_DESC(host_crypto_mode, "Use only host cryptography 0:hw/host 1:host");

/*Note: Enabling 2.4 band's 40MHz channels results in noncompliant WFA certification */
module_param(enable_24_40mhz, uint, 0);
MODULE_PARM_DESC(enable_24_40mhz, "Enable 2.4 support for 40MHz channels 0:Disable 1:Enable");


MODULE_DESCRIPTION(LRD_DESC);
MODULE_VERSION(LRD_DRV_VERSION);
MODULE_AUTHOR(LRD_AUTHOR);
MODULE_LICENSE("GPL v2");
