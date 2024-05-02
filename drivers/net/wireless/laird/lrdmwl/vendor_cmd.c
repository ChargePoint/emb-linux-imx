/*
 * Copyright (C) 2018-2020 Laird Connectivity
 *
 * This software file (the "File") is distributed by Laird Connectivity
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

#include <net/mac80211.h>
#include <linux/netlink.h>
#include <net/genetlink.h>

#include "sysadpt.h"
#include "dev.h"
#include "fwcmd.h"
#include "vendor_cmd.h"
#include "hostcmd.h"

#define PWR_TABLE_ID_OFFSET 36

void mwl_hex_dump(const void *buf, size_t len);

static int
lrd_vendor_cmd_mfg_start(struct wiphy *wiphy, struct wireless_dev *wdev,
			       const void *data, int data_len)
{
	struct ieee80211_hw *hw = wiphy_to_ieee80211_hw(wiphy);
	struct mwl_priv *priv = hw->priv;
	struct sk_buff  *msg  = NULL;
	int rc  = -ENOSYS;
	u32 rsp = 0;

	//If Deep Sleep enabled, pause it
	mwl_pause_ds(priv);

	//Send
	rc = lrd_fwcmd_mfg_start(hw, &rsp);

	//Respond
	msg = cfg80211_vendor_cmd_alloc_reply_skb(wiphy, sizeof(u32) * 2);

	if (msg) {
		nla_put_u32(msg, LRD_ATTR_CMD_RSP, rc);
		nla_put(msg, LRD_ATTR_DATA, sizeof(rsp), &rsp);
		rc = cfg80211_vendor_cmd_reply(msg);
	}
	else {
		rc = -ENOMEM;
	}

	return rc;
}

static int
lrd_vendor_cmd_mfg_write(struct wiphy *wiphy, struct wireless_dev *wdev,
			       const void *data, int data_len)
{
	struct ieee80211_hw *hw = wiphy_to_ieee80211_hw(wiphy);
	struct sk_buff  *msg = NULL;
	int rc = -ENOSYS;

	//Send
	rc = lrd_fwcmd_mfg_write(hw, (void*)data, data_len);

	//Respond
	msg = cfg80211_vendor_cmd_alloc_reply_skb(wiphy, sizeof(u32));

	if (msg) {
		nla_put_u32(msg, LRD_ATTR_CMD_RSP, rc);
		rc = cfg80211_vendor_cmd_reply(msg);
	}
	else {
		rc = -ENOMEM;
	}

	return rc;
}

static int
lrd_vendor_cmd_mfg_stop(struct wiphy *wiphy, struct wireless_dev *wdev,
			       const void *data, int data_len)
{
	struct ieee80211_hw *hw = wiphy_to_ieee80211_hw(wiphy);
	struct mwl_priv *priv = hw->priv;
	struct sk_buff  *msg  = NULL;
	int rc = -ENOSYS;

	//Send
	rc = lrd_fwcmd_mfg_end(hw);

	//Respond
	msg = cfg80211_vendor_cmd_alloc_reply_skb(wiphy, sizeof(u32));

	if (msg) {
		nla_put_u32(msg, LRD_ATTR_CMD_RSP, rc);
		rc = cfg80211_vendor_cmd_reply(msg);
	}
	else {
		rc = -ENOMEM;
	}

	//If Deep Sleep was paused, resume now
	mwl_resume_ds(priv);

	return rc;
}

static int
lrd_vendor_cmd_lru_start(struct wiphy *wiphy, struct wireless_dev *wdev,
			       const void *data, int data_len)
{
	struct sk_buff     *msg = NULL;
	int rc = 0;

	//Respond
	msg = cfg80211_vendor_cmd_alloc_reply_skb(wiphy, sizeof(u32));

	if (msg) {
		nla_put_u32(msg, LRD_ATTR_CMD_RSP, rc);
		rc = cfg80211_vendor_cmd_reply(msg);
	}
	else {
		rc = -ENOMEM;
	}

	return rc;

}

static int
lrd_vendor_cmd_lru_end(struct wiphy *wiphy, struct wireless_dev *wdev,
			       const void *data, int data_len)
{
	struct sk_buff  *msg = NULL;
	int rc = 0;

	//Respond
	msg = cfg80211_vendor_cmd_alloc_reply_skb(wiphy, sizeof(u32));

	if (msg) {
		nla_put_u32(msg, LRD_ATTR_CMD_RSP, rc);
		rc = cfg80211_vendor_cmd_reply(msg);
	}
	else {
		rc = -ENOMEM;
	}

	return rc;
}


static int
lrd_vendor_cmd_lru_write(struct wiphy *wiphy, struct wireless_dev *wdev,
			       const void *data, int data_len)
{
	struct ieee80211_hw     *hw = wiphy_to_ieee80211_hw(wiphy);
	struct sk_buff         *msg = NULL;
	struct lrd_vndr_header *rsp = NULL;
	int                      rc = -ENOSYS;

	//Send
	rc = lrd_fwcmd_lru_write(hw, (void*)data, data_len, (void*)&rsp);

	if (rc < 0 ) {
		goto fail;
	}

	//Respond
	msg = cfg80211_vendor_cmd_alloc_reply_skb(wiphy, sizeof(u32) + (rsp ? rsp->len:0));

	if (msg) {
		nla_put_u32(msg, LRD_ATTR_CMD_RSP, rsp ? rsp->result : 0);

		if (rsp) {
			nla_put(msg, LRD_ATTR_DATA, rsp->len - sizeof(struct lrd_vndr_header), ((u8*)rsp) + sizeof(struct lrd_vndr_header) );
		}

		rc = cfg80211_vendor_cmd_reply(msg);
	}
	else {
		rc = -ENOMEM;
		goto fail;
	}

fail:
	if (rsp) {
		kfree(rsp);
	}

	return rc;
}

static int
lrd_vendor_cmd_lrd_write(struct wiphy *wiphy, struct wireless_dev *wdev,
			       const void *data, int data_len)
{
	struct ieee80211_hw     *hw = wiphy_to_ieee80211_hw(wiphy);
	struct mwl_priv       *priv = hw->priv;
	struct sk_buff         *msg = NULL;
	struct lrd_vndr_header *rsp = NULL;
	struct lrdcmd_header   *hdr = (struct lrdcmd_header*)data;
	int                      rc = -ENOSYS;

	//Send
	if (NULL == hdr || data_len < sizeof(struct lrdcmd_header)) {
		rc = -EINVAL;
		goto fail;
	}

	if (hdr->lrd_cmd == cpu_to_le16(LRD_CMD_PWR_TABLE)) {
		if (priv->recovery_in_progress || priv->shutdown) {
			rc = -ENETDOWN;
			goto fail;
		}

		//Restart timers
		mod_timer(&priv->reg.timer_awm, jiffies + msecs_to_jiffies(CC_AWM_TIMER));
		cancel_work_sync(&priv->reg.awm);
		cancel_work_sync(&priv->reg.event);

		mutex_lock(&priv->reg.mutex);

		//Send
		rc = lrd_fwcmd_lrd_write(hw, (void*)data, data_len, (void*)&rsp);

		//Queue event work
		if (!rc && !rsp->result) {
			priv->reg.pn =  le32_to_cpu(*(u32*)(((u8*)data) + sizeof(*hdr) + PWR_TABLE_ID_OFFSET));

			queue_work(priv->lrd_workq, &priv->reg.event);
		}
		mutex_unlock(&priv->reg.mutex);
	}
	else if ((hdr->lrd_cmd == cpu_to_le16(LRD_CMD_CC_OTP_INFO)) ||
	         (hdr->lrd_cmd == cpu_to_le16(LRD_CMD_CC_INFO)) ) {
		rsp = kzalloc(sizeof(struct lrdcmd_cmd_cc_info) + sizeof(struct lrd_vndr_header), GFP_KERNEL);

		if (rsp) {
			struct lrdcmd_cmd_cc_info *cc;
			rsp->command = HOSTCMD_LRD_CMD;
			rsp->result  = 0;
			rsp->len     = sizeof(struct lrdcmd_cmd_cc_info) + sizeof(struct lrd_vndr_header);

			cc = (struct lrdcmd_cmd_cc_info*)(((u8*)rsp) + sizeof(struct lrd_vndr_header));
		    cc->hdr.result  = 0;
			if (hdr->lrd_cmd == cpu_to_le16(LRD_CMD_CC_OTP_INFO)) {
				cc->hdr.lrd_cmd = LRD_CMD_CC_OTP_INFO;
				cc->region = priv->reg.otp.region;
				memcpy(cc->alpha2, priv->reg.otp.alpha2, sizeof(cc->alpha2));
			}
			else {
				cc->hdr.lrd_cmd = LRD_CMD_CC_INFO;
				cc->region = priv->reg.cc.region;
				memcpy(cc->alpha2, priv->reg.cc.alpha2, sizeof(cc->alpha2));
			}

			rc = 0;
		}
		else {
			wiphy_err(hw->wiphy, "lrd_cmd_cc_info failed allocation response %zu\n", sizeof(struct cc_info));
			rc = -ENOMEM;
		}
	}
	else {
		if (priv->recovery_in_progress) {
			wiphy_info(hw->wiphy,
				"Vendor command \"%d\" rejected: Wi-Fi off\n",
				hdr->lrd_cmd);
			rc = -ENETDOWN;
			goto fail;
		}

		//Send
		rc = lrd_fwcmd_lrd_write(hw, (void*)data, data_len, (void*)&rsp);
	}

	if (rc < 0 ) {
		goto fail;
	}

	//Respond
	msg = cfg80211_vendor_cmd_alloc_reply_skb(wiphy, sizeof(uint32_t) + (rsp ? rsp->len:0));

	if (msg) {
		nla_put_u32(msg, LRD_ATTR_CMD_RSP, rsp ? rsp->result : 0);

		if (rsp) {
			nla_put(msg, LRD_ATTR_DATA, rsp->len - sizeof(struct lrd_vndr_header), ((u8*)rsp) + sizeof(struct lrd_vndr_header) );
		}

		rc = cfg80211_vendor_cmd_reply(msg);
	}
	else {
		rc = -ENOMEM;
		goto fail;
	}

fail:
	if (rsp) {
		kfree(rsp);
	}

	return rc;
}

static const struct wiphy_vendor_command lrd_vendor_commands[] = {
	{
		.info = {
			.vendor_id = LRD_OUI,
			.subcmd    = LRD_VENDOR_CMD_MFG_START,
		},
		.flags = 0,
		.doit  = lrd_vendor_cmd_mfg_start,
		.policy = VENDOR_CMD_RAW_DATA,
	},
	{
		.info = {
			.vendor_id = LRD_OUI,
			.subcmd    = LRD_VENDOR_CMD_MFG_WRITE,
		},
		.flags = 0,
		.doit  = lrd_vendor_cmd_mfg_write,
		.policy = VENDOR_CMD_RAW_DATA,
	},
	{
		.info = {
			.vendor_id = LRD_OUI,
			.subcmd    = LRD_VENDOR_CMD_MFG_STOP,
		},
		.flags = 0,
		.doit  = lrd_vendor_cmd_mfg_stop,
		.policy = VENDOR_CMD_RAW_DATA,
	},
	{
		.info = {
			.vendor_id = LRD_OUI,
			.subcmd    = LRD_VENDOR_CMD_LRU_START,
		},
		.flags = 0,
		.doit  = lrd_vendor_cmd_lru_start,
		.policy = VENDOR_CMD_RAW_DATA,
	},

	{
		.info = {
			.vendor_id = LRD_OUI,
			.subcmd    = LRD_VENDOR_CMD_LRU_WRITE,
		},
		.flags = 0,
		.doit  = lrd_vendor_cmd_lru_write,
		.policy = VENDOR_CMD_RAW_DATA,
	},

	{
		.info = {
			.vendor_id = LRD_OUI,
			.subcmd    = LRD_VENDOR_CMD_LRU_STOP,
		},
		.flags = 0,
		.doit  = lrd_vendor_cmd_lru_end,
		.policy = VENDOR_CMD_RAW_DATA,
	},

	{
		.info = {
			.vendor_id = LRD_OUI,
			.subcmd    = LRD_VENDOR_CMD_LRD_WRITE,
		},
		.flags = 0,
		.doit  = lrd_vendor_cmd_lrd_write,
		.policy = VENDOR_CMD_RAW_DATA,
	},

};


void lrd_set_vendor_commands(struct wiphy *wiphy)
{
	wiphy->vendor_commands   = lrd_vendor_commands;
	wiphy->n_vendor_commands = ARRAY_SIZE(lrd_vendor_commands);
}


static const struct nl80211_vendor_cmd_info lrd_vendor_events[] =
{
	{
		.vendor_id = LRD_OUI,
		.subcmd    = LRD_VENDOR_EVENT_RESTART,
	},
};

void lrd_set_vendor_events(struct wiphy *wiphy)
{
	wiphy->vendor_events   = lrd_vendor_events;
	wiphy->n_vendor_events = ARRAY_SIZE(lrd_vendor_events);
}
