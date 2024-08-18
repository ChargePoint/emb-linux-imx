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

/* Description:  This file implements firmware host command related
 * functions.
 */

#include <linux/etherdevice.h>
#include <linux/ctype.h>

#include "sysadpt.h"
#include "dev.h"
#include "fwcmd.h"
#include "hostcmd.h"
#include "debugfs.h"

#include "sdio.h"
#include "pcie.h"

#define MAX_WAIT_FW_COMPLETE_ITERATIONS         2000
#define MAX_WAIT_GET_HW_SPECS_ITERATONS         3

extern int lrd_debug;

static bool mwl_fwcmd_chk_adapter(struct mwl_priv *priv)
{
	bool rc;

	rc = priv->if_ops.check_card_status(priv);

	return rc;
}

static int mwl_fwcmd_send_cmd(struct mwl_priv *priv)
{
	return priv->if_ops.send_cmd(priv);
}

#define ENTRY(a) { case a: ptr= #a; break; }

char *mwl_fwcmd_get_cmd_string(unsigned short cmd)
{
	static char buf[2 * sizeof(unsigned short) + 3];
	char *ptr = buf;

	cmd &= ~HOSTCMD_RESP_BIT;

	switch (cmd) {
		ENTRY(HOSTCMD_CMD_GET_HW_SPEC)
		ENTRY(HOSTCMD_CMD_SET_HW_SPEC)
		ENTRY(HOSTCMD_CMD_802_11_GET_STAT)
		ENTRY(HOSTCMD_CMD_MAC_REG_ACCESS)
		ENTRY(HOSTCMD_CMD_BBP_REG_ACCESS)
		ENTRY(HOSTCMD_CMD_RF_REG_ACCESS)
		ENTRY(HOSTCMD_CMD_802_11_RADIO_CONTROL)
		ENTRY(HOSTCMD_CMD_MEM_ADDR_ACCESS)
		ENTRY(HOSTCMD_CMD_802_11_TX_POWER)
		ENTRY(HOSTCMD_CMD_802_11_RF_ANTENNA)
		ENTRY(HOSTCMD_CMD_802_11_PS_MODE)
		ENTRY(HOSTCMD_CMD_802_11_RF_ANTENNA_V2)
		ENTRY(HOSTCMD_CMD_BROADCAST_SSID_ENABLE)
		ENTRY(HOSTCMD_CMD_MFG)
		ENTRY(HOSTCMD_CMD_SET_CFG)
		ENTRY(HOSTCMD_CMD_SET_PRE_SCAN)
		ENTRY(HOSTCMD_CMD_SET_POST_SCAN)
		ENTRY(HOSTCMD_CMD_SET_RF_CHANNEL)
		ENTRY(HOSTCMD_CMD_SET_AID)
		ENTRY(HOSTCMD_CMD_SET_INFRA_MODE)
		ENTRY(HOSTCMD_CMD_802_11_RTS_THSD)
		ENTRY(HOSTCMD_CMD_SET_EDCA_PARAMS)
		ENTRY(HOSTCMD_CMD_802_11H_DETECT_RADAR)
		ENTRY(HOSTCMD_CMD_SET_WMM_MODE)
		ENTRY(HOSTCMD_CMD_HT_GUARD_INTERVAL)
		ENTRY(HOSTCMD_CMD_SET_FIXED_RATE)
		ENTRY(HOSTCMD_CMD_SET_IES)
		ENTRY(HOSTCMD_CMD_SET_LINKADAPT_CS_MODE)
		ENTRY(HOSTCMD_CMD_SET_MAC_ADDR)
		ENTRY(HOSTCMD_CMD_SET_RATE_ADAPT_MODE)
		ENTRY(HOSTCMD_CMD_GET_WATCHDOG_BITMAP)
		ENTRY(HOSTCMD_CMD_DEL_MAC_ADDR)
		ENTRY(HOSTCMD_CMD_BSS_START)
		ENTRY(HOSTCMD_CMD_AP_BEACON)
		ENTRY(HOSTCMD_CMD_SET_NEW_STN)
		ENTRY(HOSTCMD_CMD_SET_APMODE)
		ENTRY(HOSTCMD_CMD_SET_SWITCH_CHANNEL)
		ENTRY(HOSTCMD_CMD_UPDATE_ENCRYPTION)
		ENTRY(HOSTCMD_CMD_BASTREAM)
		ENTRY(HOSTCMD_CMD_SET_SPECTRUM_MGMT)
		ENTRY(HOSTCMD_CMD_SET_POWER_CONSTRAINT)
		ENTRY(HOSTCMD_CMD_SET_COUNTRY_CODE)
		ENTRY(HOSTCMD_CMD_SET_OPTIMIZATION_LEVEL)
		ENTRY(HOSTCMD_CMD_SET_MIMOPSHT)
		ENTRY(HOSTCMD_CMD_SET_WSC_IE)
		ENTRY(HOSTCMD_CMD_DWDS_ENABLE)
		ENTRY(HOSTCMD_CMD_FW_FLUSH_TIMER)
		ENTRY(HOSTCMD_CMD_SET_CDD)
		ENTRY(HOSTCMD_CMD_CAU_REG_ACCESS)
		ENTRY(HOSTCMD_CMD_GET_TEMP)
		ENTRY(HOSTCMD_CMD_GET_FW_REGION_CODE)
		ENTRY(HOSTCMD_CMD_GET_DEVICE_PWR_TBL)
		ENTRY(HOSTCMD_CMD_GET_FW_REGION_CODE_SC4)
		ENTRY(HOSTCMD_CMD_GET_DEVICE_PWR_TBL_SC4)
		ENTRY(HOSTCMD_CMD_QUIET_MODE)
		ENTRY(HOSTCMD_CMD_802_11_SLOT_TIME)
		ENTRY(HOSTCMD_CMD_EDMAC_CTRL)
		ENTRY(HOSTCMD_CMD_DUMP_OTP_DATA)
		ENTRY(HOSTCMD_CMD_HOSTSLEEP_CTRL)
		ENTRY(HOSTCMD_CMD_WOWLAN_AP_INRANGE_CFG)
		ENTRY(HOSTCMD_CMD_MONITOR_MODE)
		ENTRY(HOSTCMD_CMD_DEEPSLEEP)
		ENTRY(HOSTCMD_CMD_CONFIRM_PS)
		ENTRY(HOSTCMD_CMD_IBSS_START)
		ENTRY(HOSTCMD_LRD_CMD)
		ENTRY(HOSTCMD_LRD_MFG)
		ENTRY(HOSTCMD_LRD_REGION_MAPPING)

		default:
			sprintf(buf, "0x%02x", cmd);
		break;
	}

	return ptr;
}
EXPORT_SYMBOL_GPL(mwl_fwcmd_get_cmd_string);

void mwl_hex_dump(const void *buf, size_t len)
{
	const char *level = "";
	const char *prefix_str = "";
	int prefix_type = DUMP_PREFIX_OFFSET;
	int rowsize = 16;
	int groupsize = 1;
	bool ascii = false;
	const u8 *ptr = buf;
	int i, linelen, remaining = len;
	unsigned char linebuf[32 * 3 + 2 + 32 + 1];

	for (i = 0; i < len; i += rowsize) {
		linelen = min(remaining, rowsize);
		remaining -= rowsize;

		hex_dump_to_buffer(ptr + i, linelen, rowsize, groupsize,
			linebuf, sizeof(linebuf), ascii);

		switch (prefix_type) {
		case DUMP_PREFIX_ADDRESS:
			pr_info("%s%s%p: %s\n",
				level, prefix_str, ptr + i, linebuf);
			break;
		case DUMP_PREFIX_OFFSET:
			pr_info("%s%s%.8x: %s\n",
				level, prefix_str, i, linebuf);
			break;
		default:
			pr_info("%s%s%s\n",
				level, prefix_str, linebuf);
			break;
		}
	}
	return;
}
EXPORT_SYMBOL_GPL(mwl_hex_dump);

/*Note:  When calling this function, it is expected that the fwcmd_mutex is already held */
static int mwl_fwcmd_exec_cmd(struct mwl_priv *priv, unsigned short cmd)
{
	int rc = -EIO;
	struct hostcmd_header *presp;
	struct hostcmd_header *pcmd;

	might_sleep();

	// If recovery is in progress, firmware is hung or device is restarting
	// Don't attempt to access it unless coming from the restart owner
	if ((priv->recovery_in_progress) && (priv->recovery_owner != current))
		return -ENETDOWN;

	if(priv->if_ops.is_deepsleep(priv)) {
		priv->if_ops.wakeup_card(priv);
	}

	if (!mwl_fwcmd_chk_adapter(priv)) {
		wiphy_err(priv->hw->wiphy, "adapter does not exist\n");
		return -EIO;
	}

	if (priv->cmd_timeout) {
		if (lrd_debug) {
			wiphy_debug(priv->hw->wiphy,
			"Skip CMD(%04xh, %s) - due to prev cmd_timeout\n",
			cmd, mwl_fwcmd_get_cmd_string(cmd));
		}
		return -EIO;
	}

	pcmd = (struct hostcmd_header *)&priv->pcmd_buf[
		INTF_CMDHEADER_LEN(priv->if_ops.inttf_head_len)];

	priv->cmd_seq_num++;
	pcmd->seq_num = priv->cmd_seq_num;

	if (lrd_debug) {
		wiphy_debug(priv->hw->wiphy, "DNLD_CMD(# %02x)=> (%04xh, %s)\n",
			pcmd->seq_num, cmd, mwl_fwcmd_get_cmd_string(cmd));
		mwl_hex_dump((char*)pcmd, le16_to_cpu(pcmd->len));
	}

	rc = mwl_fwcmd_send_cmd(priv);
	if (rc) {
		if (priv->cmd_timeout) {
			wiphy_err(priv->hw->wiphy, "DNLD_CMD (# %02x)=> (%04xh, %s) timeout\n",
				pcmd->seq_num, cmd, mwl_fwcmd_get_cmd_string(cmd));

			mwl_hex_dump((char*)pcmd, le16_to_cpu(pcmd->len));
			// Fatal error, initiate firmware recovery
			lrd_radio_recovery(priv);
		}

		return rc;
	}

	if (priv->if_ops.cmd_resp_wait_completed) {
		rc = priv->if_ops.cmd_resp_wait_completed(priv,
			HOSTCMD_RESP_BIT | cmd);
	}

	if (rc) {
		if (rc == -ENETDOWN)
			return rc;

		wiphy_err(priv->hw->wiphy, "CMD_RESP (# %02x)=> (%04xh, %s) timeout\n",
			pcmd->seq_num, cmd, mwl_fwcmd_get_cmd_string(cmd));

		mwl_hex_dump((char*)pcmd, le16_to_cpu(pcmd->len));

		if (cmd != HOSTCMD_CMD_GET_HW_SPEC) {
			priv->cmd_timeout = true;

			// Fatal error, initiate firmware recovery
			lrd_radio_recovery(priv);
		}
		return -EIO;
	}
	presp = (struct hostcmd_header *)&priv->pcmd_buf[
		INTF_CMDHEADER_LEN(priv->if_ops.inttf_head_len)];

	if (lrd_debug) {
		wiphy_debug(priv->hw->wiphy, " CMD_RESP(# %02x)=> (%04xh)\n",
			presp->seq_num, le16_to_cpu(presp->cmd));
			mwl_hex_dump((char*)pcmd, le16_to_cpu(pcmd->len));
	}

	if(cmd != HOSTCMD_CMD_DEEPSLEEP) {
		mwl_restart_ds_timer(priv, false);
	}

	return 0;
}

int mwl_fwcmd_set_slot_time(struct ieee80211_hw *hw, bool short_slot)
{
	struct hostcmd_cmd_802_11_slot_time *pcmd;
	struct mwl_priv *priv = hw->priv;

//	wiphy_err(priv->hw->wiphy, "%s(): short_slot_time=%d\n", __FUNCTION__, short_slot);

	pcmd = (struct hostcmd_cmd_802_11_slot_time *)&priv->pcmd_buf[
			INTF_CMDHEADER_LEN(priv->if_ops.inttf_head_len)];

	mutex_lock(&priv->fwcmd_mutex);

	memset(pcmd, 0x00, sizeof(*pcmd));
	pcmd->cmd_hdr.cmd = cpu_to_le16(HOSTCMD_CMD_802_11_SLOT_TIME);
	pcmd->cmd_hdr.len = cpu_to_le16(sizeof(*pcmd));
	pcmd->action = cpu_to_le16(WL_SET);
	pcmd->short_slot = cpu_to_le16(short_slot?1:0);

	if (mwl_fwcmd_exec_cmd(priv, HOSTCMD_CMD_802_11_SLOT_TIME)) {
		mutex_unlock(&priv->fwcmd_mutex);
		return -EIO;
	}

	mutex_unlock(&priv->fwcmd_mutex);

	return 0;
}

/*Note:  When calling this function, it is expected that the fwcmd_mutex is already held */
int mwl_fwcmd_confirm_ps(struct ieee80211_hw *hw)
{
	struct hostcmd_cmd_confirm_ps *pcmd;
	struct mwl_priv *priv = hw->priv;

	if(priv->if_ops.is_deepsleep(priv))
		return 0;

	pcmd = (struct hostcmd_cmd_confirm_ps *)&priv->pcmd_buf[
			INTF_CMDHEADER_LEN(priv->if_ops.inttf_head_len)];

	memset(pcmd, 0x00, sizeof(*pcmd));
	pcmd->cmd_hdr.cmd = cpu_to_le16(HOSTCMD_CMD_CONFIRM_PS);
	pcmd->cmd_hdr.len = cpu_to_le16(sizeof(*pcmd));

	if (mwl_fwcmd_exec_cmd(priv, HOSTCMD_CMD_CONFIRM_PS)) {
		wiphy_err(hw->wiphy, " %s failed execution\n",
		          mwl_fwcmd_get_cmd_string(HOSTCMD_CMD_CONFIRM_PS));
		return -EIO;
	}

	if (pcmd->status == 0) {
		priv->if_ops.enter_deepsleep(priv);
		return 0;
	}

	return -1;
}
EXPORT_SYMBOL_GPL(mwl_fwcmd_confirm_ps);

int mwl_fwcmd_enter_deepsleep(struct ieee80211_hw *hw)
{
	struct hostcmd_cmd_deepsleep *pcmd;
	struct mwl_priv *priv = hw->priv;

	if (priv->mfg_mode) {
		return 0;
	}

	pcmd = (struct hostcmd_cmd_deepsleep *)&priv->pcmd_buf[
			INTF_CMDHEADER_LEN(priv->if_ops.inttf_head_len)];

	mutex_lock(&priv->fwcmd_mutex);

	if(priv->if_ops.is_deepsleep(priv)) {
		mutex_unlock(&priv->fwcmd_mutex);
		wiphy_err(priv->hw->wiphy,"radio already asleep\n");
		return 0;
	}

	memset(pcmd, 0x00, sizeof(*pcmd));
	pcmd->cmd_hdr.cmd = cpu_to_le16(HOSTCMD_CMD_DEEPSLEEP);
	pcmd->cmd_hdr.len = cpu_to_le16(sizeof(*pcmd));
	pcmd->enableFlag = 1;

	if (mwl_fwcmd_exec_cmd(priv, HOSTCMD_CMD_DEEPSLEEP)) {
		mutex_unlock(&priv->fwcmd_mutex);
		wiphy_err(hw->wiphy, " %s failed execution\n",
		          mwl_fwcmd_get_cmd_string(HOSTCMD_CMD_DEEPSLEEP));
		return -EIO;
	}

	priv->if_ops.enter_deepsleep(priv);

	mutex_unlock(&priv->fwcmd_mutex);

	wiphy_dbg(priv->hw->wiphy, "Entered deepsleep\n");
	return 0;
}

int mwl_fwcmd_exit_deepsleep(struct ieee80211_hw *hw)
{
	struct mwl_priv *priv = hw->priv;
	int rc = 0;

	mutex_lock(&priv->fwcmd_mutex);

	if (priv->if_ops.is_deepsleep(priv)) {
		rc = priv->if_ops.wakeup_card(priv);
	}

	mutex_unlock(&priv->fwcmd_mutex);

	return rc;
}

int mwl_fwcmd_config_EDMACCtrl(struct ieee80211_hw *hw, int EDMAC_Ctrl)
{
	struct hostcmd_cmd_edmac_ctrl *pcmd;
	struct mwl_priv *priv = hw->priv;

	pcmd = (struct hostcmd_cmd_edmac_ctrl *)&priv->pcmd_buf[
			INTF_CMDHEADER_LEN(priv->if_ops.inttf_head_len)];

	mutex_lock(&priv->fwcmd_mutex);

	memset(pcmd, 0x00, sizeof(*pcmd));
	pcmd->cmd_hdr.cmd = cpu_to_le16(HOSTCMD_CMD_EDMAC_CTRL);
	pcmd->cmd_hdr.len = cpu_to_le16(sizeof(*pcmd));
	pcmd->action = cpu_to_le16(WL_SET);
	pcmd->ed_ctrl_2g = ((EDMAC_Ctrl & EDMAC_2G_ENABLE_MASK)
						>> EDMAC_2G_ENABLE_SHIFT);
	pcmd->ed_ctrl_5g = ((EDMAC_Ctrl & EDMAC_5G_ENABLE_MASK)
						>> EDMAC_5G_ENABLE_SHIFT);
	pcmd->ed_offset_2g = (s8)((EDMAC_Ctrl & EDMAC_2G_THRESHOLD_OFFSET_MASK)
						>> EDMAC_2G_THRESHOLD_OFFSET_SHIFT);
	pcmd->ed_offset_5g = (s8)((EDMAC_Ctrl & EDMAC_5G_THRESHOLD_OFFSET_MASK)
						>> EDMAC_5G_THRESHOLD_OFFSET_SHIFT);
	pcmd->ed_bitmap_txq_lock = cpu_to_le16((EDMAC_Ctrl & EDMAC_QLOCK_BITMAP_MASK)
								>> EDMAC_QLOCK_BITMAP_SHIFT);

	if (mwl_fwcmd_exec_cmd(priv, HOSTCMD_CMD_EDMAC_CTRL)) {
		mutex_unlock(&priv->fwcmd_mutex);
		return -EIO;
	}

	mutex_unlock(&priv->fwcmd_mutex);

	return 0;
}

int mwl_fwcmd_getEDMAC(struct mwl_priv *priv, int * pEdmac)
{
	struct hostcmd_cmd_edmac_ctrl *pcmd;
	int rc = 0;

	if (!pEdmac) {
		rc = -EINVAL;
		goto error_return;
	}

	pcmd = (struct hostcmd_cmd_edmac_ctrl *)&priv->pcmd_buf[
			INTF_CMDHEADER_LEN(priv->if_ops.inttf_head_len)];

	mutex_lock(&priv->fwcmd_mutex);

	memset(pcmd, 0x00, sizeof(*pcmd));
	pcmd->cmd_hdr.cmd = cpu_to_le16(HOSTCMD_CMD_EDMAC_CTRL);
	pcmd->cmd_hdr.len = cpu_to_le16(sizeof(*pcmd));
	pcmd->action = cpu_to_le16(WL_GET);

	if (mwl_fwcmd_exec_cmd(priv, HOSTCMD_CMD_EDMAC_CTRL)) {
		mutex_unlock(&priv->fwcmd_mutex);
		rc = -EIO;
		goto error_return;
	}

	if (pcmd->cmd_hdr.result == HOSTCMD_RESULT_NOT_SUPPORT) {
		mutex_unlock(&priv->fwcmd_mutex);
		rc = -ENOTSUPP;
		goto error_return;
	}

	*pEdmac = pcmd->ed_ctrl_2g << EDMAC_2G_ENABLE_SHIFT;
	*pEdmac |= pcmd->ed_ctrl_5g << EDMAC_5G_ENABLE_SHIFT;

	mutex_unlock(&priv->fwcmd_mutex);

error_return:
	return rc;
}

static int mwl_fwcmd_802_11_radio_control(struct mwl_priv *priv,
					  bool enable, bool force)
{
	struct hostcmd_cmd_802_11_radio_control *pcmd;

	if (enable == priv->radio_on && !force)
		return 0;

	pcmd = (struct hostcmd_cmd_802_11_radio_control *)&priv->pcmd_buf[
			INTF_CMDHEADER_LEN(priv->if_ops.inttf_head_len)];

	mutex_lock(&priv->fwcmd_mutex);

	memset(pcmd, 0x00, sizeof(*pcmd));
	pcmd->cmd_hdr.cmd = cpu_to_le16(HOSTCMD_CMD_802_11_RADIO_CONTROL);
	pcmd->cmd_hdr.len = cpu_to_le16(sizeof(*pcmd));
	pcmd->action = cpu_to_le16(WL_SET);
	pcmd->control = cpu_to_le16(priv->radio_short_preamble ?
		WL_AUTO_PREAMBLE : WL_LONG_PREAMBLE);
	pcmd->radio_on = cpu_to_le16(enable ? WL_ENABLE : WL_DISABLE);

	if (mwl_fwcmd_exec_cmd(priv, HOSTCMD_CMD_802_11_RADIO_CONTROL)) {
		mutex_unlock(&priv->fwcmd_mutex);
		return -EIO;
	}

	priv->radio_on = enable;

	mutex_unlock(&priv->fwcmd_mutex);

	return 0;
}

int mwl_fwcmd_get_tx_powers(struct mwl_priv *priv, u16 *powlist,
				   u8 action, u16 ch, u16 band, u16 width, u16 sub_ch)
{
	struct hostcmd_cmd_802_11_tx_power *pcmd;
	int i;

	pcmd = (struct hostcmd_cmd_802_11_tx_power *)&priv->pcmd_buf[
			INTF_CMDHEADER_LEN(priv->if_ops.inttf_head_len)];

	mutex_lock(&priv->fwcmd_mutex);

	memset(pcmd, 0x00, sizeof(*pcmd));
	pcmd->cmd_hdr.cmd = cpu_to_le16(HOSTCMD_CMD_802_11_TX_POWER);
	pcmd->cmd_hdr.len = cpu_to_le16(sizeof(*pcmd));
	pcmd->action = cpu_to_le16(action);
	pcmd->ch = cpu_to_le16(ch);
	pcmd->bw = cpu_to_le16(width);
	pcmd->band = cpu_to_le16(band);
	pcmd->sub_ch = cpu_to_le16(sub_ch);

	if (mwl_fwcmd_exec_cmd(priv, HOSTCMD_CMD_802_11_TX_POWER)) {
		mutex_unlock(&priv->fwcmd_mutex);
		return -EIO;
	}

	for (i = 0; i < SYSADPT_TX_GRP_PWR_LEVEL_TOTAL; i++)
		powlist[i] = le16_to_cpu(pcmd->power_level_list[i]);

	mutex_unlock(&priv->fwcmd_mutex);

	return 0;
}

static int mwl_fwcmd_set_tx_powers(struct mwl_priv *priv, u16 txpow[],
				   u8 action, u16 ch, u16 band,
				   u16 width, u16 sub_ch)
{
	struct hostcmd_cmd_802_11_tx_power *pcmd;
	int i;

	pcmd = (struct hostcmd_cmd_802_11_tx_power *)&priv->pcmd_buf[
			INTF_CMDHEADER_LEN(priv->if_ops.inttf_head_len)];

	mutex_lock(&priv->fwcmd_mutex);

	memset(pcmd, 0x00, sizeof(*pcmd));
	pcmd->cmd_hdr.cmd = cpu_to_le16(HOSTCMD_CMD_802_11_TX_POWER);
	pcmd->cmd_hdr.len = cpu_to_le16(sizeof(*pcmd));
	pcmd->action = cpu_to_le16(action);
	pcmd->ch = cpu_to_le16(ch);
	pcmd->bw = cpu_to_le16(width);
	pcmd->band = cpu_to_le16(band);
	pcmd->sub_ch = cpu_to_le16(sub_ch);

	for (i = 0; i < SYSADPT_TX_GRP_PWR_LEVEL_TOTAL; i++)
		pcmd->power_level_list[i] = cpu_to_le16(txpow[i]);

	if (mwl_fwcmd_exec_cmd(priv, HOSTCMD_CMD_802_11_TX_POWER)) {
		mutex_unlock(&priv->fwcmd_mutex);
		return -EIO;
	}

	mutex_unlock(&priv->fwcmd_mutex);

	return 0;
}

static u8 mwl_fwcmd_get_80m_pri_chnl(u8 channel)
{
	u8 act_primary = ACT_PRIMARY_CHAN_0;

	switch (channel) {
	case 36:
		act_primary = ACT_PRIMARY_CHAN_0;
		break;
	case 40:
		act_primary = ACT_PRIMARY_CHAN_1;
		break;
	case 44:
		act_primary = ACT_PRIMARY_CHAN_2;
		break;
	case 48:
		act_primary = ACT_PRIMARY_CHAN_3;
		break;
	case 52:
		act_primary = ACT_PRIMARY_CHAN_0;
		break;
	case 56:
		act_primary = ACT_PRIMARY_CHAN_1;
		break;
	case 60:
		act_primary = ACT_PRIMARY_CHAN_2;
		break;
	case 64:
		act_primary = ACT_PRIMARY_CHAN_3;
		break;
	case 100:
		act_primary = ACT_PRIMARY_CHAN_0;
		break;
	case 104:
		act_primary = ACT_PRIMARY_CHAN_1;
		break;
	case 108:
		act_primary = ACT_PRIMARY_CHAN_2;
		break;
	case 112:
		act_primary = ACT_PRIMARY_CHAN_3;
		break;
	case 116:
		act_primary = ACT_PRIMARY_CHAN_0;
		break;
	case 120:
		act_primary = ACT_PRIMARY_CHAN_1;
		break;
	case 124:
		act_primary = ACT_PRIMARY_CHAN_2;
		break;
	case 128:
		act_primary = ACT_PRIMARY_CHAN_3;
		break;
	case 132:
		act_primary = ACT_PRIMARY_CHAN_0;
		break;
	case 136:
		act_primary = ACT_PRIMARY_CHAN_1;
		break;
	case 140:
		act_primary = ACT_PRIMARY_CHAN_2;
		break;
	case 144:
		act_primary = ACT_PRIMARY_CHAN_3;
		break;
	case 149:
		act_primary = ACT_PRIMARY_CHAN_0;
		break;
	case 153:
		act_primary = ACT_PRIMARY_CHAN_1;
		break;
	case 157:
		act_primary = ACT_PRIMARY_CHAN_2;
		break;
	case 161:
		act_primary = ACT_PRIMARY_CHAN_3;
		break;
	}

	return act_primary;
}

static u8 mwl_fwcmd_get_160m_pri_chnl(u8 channel)
{
	u8 act_primary = ACT_PRIMARY_CHAN_0;

	switch (channel) {
	case 36:
		act_primary = ACT_PRIMARY_CHAN_0;
		break;
	case 40:
		act_primary = ACT_PRIMARY_CHAN_1;
		break;
	case 44:
		act_primary = ACT_PRIMARY_CHAN_2;
		break;
	case 48:
		act_primary = ACT_PRIMARY_CHAN_3;
		break;
	case 52:
		act_primary = ACT_PRIMARY_CHAN_4;
		break;
	case 56:
		act_primary = ACT_PRIMARY_CHAN_5;
		break;
	case 60:
		act_primary = ACT_PRIMARY_CHAN_6;
		break;
	case 64:
		act_primary = ACT_PRIMARY_CHAN_7;
		break;
	case 100:
		act_primary = ACT_PRIMARY_CHAN_0;
		break;
	case 104:
		act_primary = ACT_PRIMARY_CHAN_1;
		break;
	case 108:
		act_primary = ACT_PRIMARY_CHAN_2;
		break;
	case 112:
		act_primary = ACT_PRIMARY_CHAN_3;
		break;
	case 116:
		act_primary = ACT_PRIMARY_CHAN_4;
		break;
	case 120:
		act_primary = ACT_PRIMARY_CHAN_5;
		break;
	case 124:
		act_primary = ACT_PRIMARY_CHAN_6;
		break;
	case 128:
		act_primary = ACT_PRIMARY_CHAN_7;
		break;
	case 149:
		act_primary = ACT_PRIMARY_CHAN_0;
		break;
	case 153:
		act_primary = ACT_PRIMARY_CHAN_1;
		break;
	case 157:
		act_primary = ACT_PRIMARY_CHAN_2;
		break;
	case 161:
		act_primary = ACT_PRIMARY_CHAN_3;
		break;
	case 165:
		act_primary = ACT_PRIMARY_CHAN_4;
		break;
	case 169:
		act_primary = ACT_PRIMARY_CHAN_5;
		break;
	case 173:
		act_primary = ACT_PRIMARY_CHAN_6;
		break;
	case 177:
		act_primary = ACT_PRIMARY_CHAN_7;
		break;
	}

	return act_primary;
}

static int laird_is_host_cipher(const u8 *buf)
{
	u32 cipher;
	cipher = get_unaligned_be32(buf);
	switch (cipher) {
	case WLAN_CIPHER_SUITE_GCMP:
	case WLAN_CIPHER_SUITE_GCMP_256:
	case WLAN_CIPHER_SUITE_CCMP_256:
		return 1;
	default:
		break;
	}
	return 0;
}

// returns true if the beacon rsnie is present and any ciphers use host crypto
static int laird_ap_select_host_crypto(struct mwl_vif *mwl_vif)
{
	const u8 *rsn_ie;
	size_t rsn_ie_len;
	int cnt;

	rsn_ie = mwl_vif->beacon_info.ie_rsn48_ptr;
	rsn_ie_len = mwl_vif->beacon_info.ie_rsn48_len;
	if (!rsn_ie)
		return 0;

	/* skip tag, len, version */
	if (rsn_ie_len < 4)
		return 0;
	rsn_ie += 4;
	rsn_ie_len -= 4;

	/* group cipher suite */
	if (rsn_ie_len < 4)
		return 0;
	if (laird_is_host_cipher(rsn_ie))
		return 1;
	rsn_ie += 4;
	rsn_ie_len -= 4;

	/* pairwise cipher suite(s) */
	if (rsn_ie_len < 2)
		return 0;
	cnt = get_unaligned_le16(rsn_ie);
	rsn_ie += 2;
	rsn_ie_len -= 2;
	if (rsn_ie_len < (cnt * 4))
		return 0;
	while (cnt--) {
		if (laird_is_host_cipher(rsn_ie))
			return 1;
		rsn_ie += 4;
		rsn_ie_len -= 4;
	}
	return 0;
}

static void mwl_fwcmd_parse_beacon(struct mwl_priv *priv,
				   struct mwl_vif *vif, u8 *beacon, int len)
{
	struct ieee80211_mgmt *mgmt;
	struct beacon_info *beacon_info;
	int baselen;
	u8 *pos;
	size_t left;
	bool elem_parse_failed = false;
	bool elem_11k_failed   = false;

	mgmt = (struct ieee80211_mgmt *)beacon;

	memcpy(vif->bssid, mgmt->bssid, ETH_ALEN);

	baselen = (u8 *)mgmt->u.beacon.variable - (u8 *)mgmt;
	if (baselen > len)
		return;

	beacon_info = &vif->beacon_info;
	memset(beacon_info, 0, sizeof(struct beacon_info));
	beacon_info->valid = false;
	beacon_info->ie_ht_ptr  = &beacon_info->ie_list_ht[0];
	beacon_info->ie_vht_ptr = &beacon_info->ie_list_vht[0];
	beacon_info->ie_11k_ptr = &beacon_info->ie_list_11k[0];

	beacon_info->cap_info = le16_to_cpu(mgmt->u.beacon.capab_info);
	beacon_info->power_constraint = 0;

	pos = (u8 *)mgmt->u.beacon.variable;
	left = len - baselen;

	while (left >= 2) {
		u8 id, elen;

		id = *pos++;
		elen = *pos++;
		left -= 2;

		if (elen > left) {
			elem_parse_failed = true;
			break;
		}

		switch (id) {
		case WLAN_EID_SSID:
			beacon_info->ie_ssid_len = (elen + 2);
			beacon_info->ie_ssid_ptr = (pos - 2);
			break;
		case WLAN_EID_IBSS_PARAMS:
			beacon_info->ie_ibss_parms_len = (elen + 2);
			beacon_info->ie_ibss_parms_ptr = (pos - 2);
			break;
		case WLAN_EID_COUNTRY:
			beacon_info->ie_country_len = (elen + 2);
			beacon_info->ie_country_ptr = (pos - 2);
			break;
		case WLAN_EID_SUPP_RATES:
		case WLAN_EID_EXT_SUPP_RATES:
			{
			int idx, bi, oi;
			u8 rate;

			for (bi = 0; bi < SYSADPT_MAX_DATA_RATES_G;
			     bi++) {
				if (beacon_info->b_rate_set[bi] == 0)
					break;
			}

			for (oi = 0; oi < SYSADPT_MAX_DATA_RATES_G;
			     oi++) {
				if (beacon_info->op_rate_set[oi] == 0)
					break;
			}

			for (idx = 0; idx < elen; idx++) {
				rate = pos[idx];
				if ((rate & 0x80) != 0) {
					if (bi < SYSADPT_MAX_DATA_RATES_G)
						beacon_info->b_rate_set[bi++]
							= rate & 0x7f;
					else {
						elem_parse_failed = true;
						break;
					}
				}
				if (oi < SYSADPT_MAX_DATA_RATES_G)
					beacon_info->op_rate_set[oi++] =
						rate & 0x7f;
				else {
					elem_parse_failed = true;
					break;
				}
			}
			}
			break;
		case WLAN_EID_PWR_CONSTRAINT:
			if (elen == 1)
				beacon_info->power_constraint = *pos;
			break;
		case WLAN_EID_RSN:
			beacon_info->ie_rsn48_len = (elen + 2);
			beacon_info->ie_rsn48_ptr = (pos - 2);
			break;
		case WLAN_EID_HT_CAPABILITY:
		case WLAN_EID_HT_OPERATION:
		case WLAN_EID_OVERLAP_BSS_SCAN_PARAM:
		case WLAN_EID_EXT_CAPABILITY:
			beacon_info->ie_ht_len += (elen + 2);
			if (beacon_info->ie_ht_len >
			    sizeof(beacon_info->ie_list_ht)) {
				elem_parse_failed = true;
			} else {
				*beacon_info->ie_ht_ptr++ = id;
				*beacon_info->ie_ht_ptr++ = elen;
				memcpy(beacon_info->ie_ht_ptr, pos, elen);
				beacon_info->ie_ht_ptr += elen;
			}
			break;
		case WLAN_EID_VHT_CAPABILITY:
		case WLAN_EID_VHT_OPERATION:
		case WLAN_EID_OPMODE_NOTIF:
			beacon_info->ie_vht_len += (elen + 2);
			if (beacon_info->ie_vht_len >
			    sizeof(beacon_info->ie_list_vht)) {
				elem_parse_failed = true;
			} else {
				*beacon_info->ie_vht_ptr++ = id;
				*beacon_info->ie_vht_ptr++ = elen;
				memcpy(beacon_info->ie_vht_ptr, pos, elen);
				beacon_info->ie_vht_ptr += elen;
			}
			break;

		case WLAN_EID_AP_CHAN_REPORT:
		case WLAN_EID_BSS_AVG_ACCESS_DELAY:
		case WLAN_EID_ANTENNA_INFO:
		case WLAN_EID_MEASUREMENT_PILOT_TX_INFO:
		case WLAN_EID_BSS_AVAILABLE_CAPACITY:
		case WLAN_EID_BSS_AC_ACCESS_DELAY:
		case WLAN_EID_RRM_ENABLED_CAPABILITIES:
		case WLAN_EID_MULTIPLE_BSSID:
			beacon_info->ie_11k_len += (elen + 2);
			if (beacon_info->ie_11k_len > sizeof(beacon_info->ie_list_11k)) {
				elem_11k_failed = true;
			}
			else {
				*beacon_info->ie_11k_ptr++ = id;
				*beacon_info->ie_11k_ptr++ = elen;
				memcpy(beacon_info->ie_11k_ptr, pos, elen);
				beacon_info->ie_11k_ptr += elen;
			}
			break;
		case WLAN_EID_VENDOR_SPECIFIC:
			if ((pos[0] == 0x00) && (pos[1] == 0x50) &&
			    (pos[2] == 0xf2)) {
				if (pos[3] == 0x01) {
					beacon_info->ie_rsn_len = (elen + 2);
					beacon_info->ie_rsn_ptr = (pos - 2);
				}

				if (pos[3] == 0x02) {
					beacon_info->ie_wmm_len = (elen + 2);
					beacon_info->ie_wmm_ptr = (pos - 2);
				}

				if (pos[3] == 0x04) {
					beacon_info->ie_wsc_len = (elen + 2);
					beacon_info->ie_wsc_ptr = (pos - 2);
				}
			}

			if ((pos[0] == 0x50) && (pos[1] == 0x6F) &&
				(pos[2] == 0x9A)) {
				if (pos[3] == 0x09) {
					beacon_info->ie_wfd_len = (elen + 2);
					beacon_info->ie_wfd_ptr = (pos - 2);
				}
			}
			break;
		default:
			break;
		}

		left -= elen;
		pos += elen;
	}

	if (!elem_parse_failed) {
		beacon_info->ie_ht_ptr = &beacon_info->ie_list_ht[0];
		beacon_info->ie_vht_ptr = &beacon_info->ie_list_vht[0];
		beacon_info->valid = true;
	}

	if (!elem_11k_failed) {
		beacon_info->ie_11k_ptr = &beacon_info->ie_list_11k[0];
	}
	else {
		wiphy_err(priv->hw->wiphy, "11k IEs will not fit in remaining buffer, dropping.");
		beacon_info->ie_11k_len = 0;
	}

	/* force host crypto if any rsnie ciphers require host crypto */
	vif->force_host_crypto = laird_ap_select_host_crypto(vif);
}

static int mwl_fwcmd_set_ies(struct mwl_priv *priv, struct mwl_vif *mwl_vif)
{
	struct hostcmd_cmd_set_ies *pcmd;
	struct beacon_info *beacon = &mwl_vif->beacon_info;

	if (beacon->ie_ht_len > sizeof(pcmd->ie_list_ht))
		goto einval;

	if (beacon->ie_vht_len > sizeof(pcmd->ie_list_vht))
		goto einval;

	if (beacon->ie_11k_len > sizeof(pcmd->ie_list_11k))
		goto einval;

	pcmd = (struct hostcmd_cmd_set_ies *)&priv->pcmd_buf[
			INTF_CMDHEADER_LEN(priv->if_ops.inttf_head_len)];

	mutex_lock(&priv->fwcmd_mutex);

	memset(pcmd, 0x00, sizeof(*pcmd));
	pcmd->cmd_hdr.cmd = cpu_to_le16(HOSTCMD_CMD_SET_IES);
	pcmd->cmd_hdr.len = cpu_to_le16(sizeof(*pcmd));
	pcmd->cmd_hdr.macid = mwl_vif->macid;

	pcmd->action = cpu_to_le16(HOSTCMD_ACT_GEN_SET);

	memcpy(pcmd->ie_list_ht, beacon->ie_ht_ptr, beacon->ie_ht_len);
	pcmd->ie_list_len_ht = cpu_to_le16(beacon->ie_ht_len);

	memcpy(pcmd->ie_list_vht, beacon->ie_vht_ptr, beacon->ie_vht_len);
	pcmd->ie_list_len_vht = cpu_to_le16(beacon->ie_vht_len);

	memcpy(pcmd->ie_list_11k, beacon->ie_11k_ptr, beacon->ie_11k_len);
	pcmd->ie_list_len_11k = cpu_to_le16(beacon->ie_11k_len);

	if (mwl_fwcmd_exec_cmd(priv, HOSTCMD_CMD_SET_IES)) {
		mutex_unlock(&priv->fwcmd_mutex);
		return -EIO;
	}

	mutex_unlock(&priv->fwcmd_mutex);

	return 0;

einval:

	wiphy_err(priv->hw->wiphy, "length of IE is too long\n");

	return -EINVAL;
}

static int mwl_fwcmd_set_ap_beacon(struct mwl_priv *priv,
				   struct ieee80211_vif *vif)
{
	struct mwl_vif *mwl_vif;
	struct hostcmd_cmd_ap_beacon *pcmd;
	struct ds_params *phy_ds_param_set;


	mwl_vif = mwl_dev_get_vif(vif);
	/* wmm structure of start command is defined less one byte,
	 * due to following field country is not used, add byte one
	 * to bypass the check.
	 */
	if (mwl_vif->beacon_info.ie_wmm_len >
	    (sizeof(pcmd->start_cmd.wmm_param) + 1)) {
		wiphy_err(priv->hw->wiphy,
		          "WMM IE is longer than radio supports");
		goto ielenerr;
	}

	if (mwl_vif->beacon_info.ie_rsn48_len > sizeof(pcmd->start_cmd.rsn_data)) {
		wiphy_err(priv->hw->wiphy,
		          "RSN IE is longer than radio supports");
		goto ielenerr;
	}

	if (mwl_vif->beacon_info.ie_ibss_parms_len >
	    sizeof(pcmd->start_cmd.ss_param_set.ibss_param_set)) {
		wiphy_err(priv->hw->wiphy,
		          "IBSS Parameter Set IE is longer than radio supports");
		goto ielenerr;
	}

	pcmd = (struct hostcmd_cmd_ap_beacon *)&priv->pcmd_buf[
			INTF_CMDHEADER_LEN(priv->if_ops.inttf_head_len)];

	mutex_lock(&priv->fwcmd_mutex);

	memset(pcmd, 0x00, sizeof(*pcmd));
	pcmd->cmd_hdr.cmd = cpu_to_le16(HOSTCMD_CMD_AP_BEACON);
	pcmd->cmd_hdr.len = cpu_to_le16(sizeof(*pcmd));
	pcmd->cmd_hdr.macid = mwl_vif->macid;

	if(vif->type == NL80211_IFTYPE_ADHOC) {
		ether_addr_copy(pcmd->start_cmd.sta_mac_addr, mwl_vif->sta_mac);
	}else {
		ether_addr_copy(pcmd->start_cmd.sta_mac_addr, mwl_vif->bssid);
	}

	ether_addr_copy(pcmd->start_cmd.Bssid, mwl_vif->bssid);
	memcpy(pcmd->start_cmd.ssid, vif->cfg.ssid, vif->cfg.ssid_len);
	pcmd->start_cmd.bss_type = 1;
	pcmd->start_cmd.bcn_period  = cpu_to_le16(vif->bss_conf.beacon_int);
	pcmd->start_cmd.dtim_period = vif->bss_conf.dtim_period; /* 8bit */

	phy_ds_param_set = &pcmd->start_cmd.phy_param_set.ds_param_set;
	phy_ds_param_set->elem_id = WLAN_EID_DS_PARAMS;
	phy_ds_param_set->len = sizeof(phy_ds_param_set->current_chnl);
	phy_ds_param_set->current_chnl = vif->bss_conf.chandef.chan->hw_value;

	pcmd->start_cmd.probe_delay = cpu_to_le16(10);
	pcmd->start_cmd.cap_info = cpu_to_le16(mwl_vif->beacon_info.cap_info);

	memcpy(&pcmd->start_cmd.ss_param_set.ibss_param_set, mwl_vif->beacon_info.ie_ibss_parms_ptr,
	       mwl_vif->beacon_info.ie_ibss_parms_len);

	memcpy(&pcmd->start_cmd.wmm_param, mwl_vif->beacon_info.ie_wmm_ptr,
	       mwl_vif->beacon_info.ie_wmm_len);

	//rsn IE
	if (mwl_vif->beacon_info.ie_rsn48_len) {
		memcpy(pcmd->start_cmd.rsn_data,
		       mwl_vif->beacon_info.ie_rsn48_ptr,
		       mwl_vif->beacon_info.ie_rsn48_len);

		pcmd->start_cmd.offset_StaRsnIE48 = pcmd->start_cmd.rsn_data - (u8*)&pcmd->start_cmd;
	}

	//wpa IE
	if (mwl_vif->beacon_info.ie_rsn_len) {

		if ( mwl_vif->beacon_info.ie_rsn48_len + mwl_vif->beacon_info.ie_rsn_len <= sizeof(pcmd->start_cmd.rsn_data)){

			pcmd->start_cmd.offset_StaRsnIE = pcmd->start_cmd.rsn_data - (u8*)&pcmd->start_cmd;

			if (pcmd->start_cmd.offset_StaRsnIE48) {
				pcmd->start_cmd.offset_StaRsnIE += mwl_vif->beacon_info.ie_rsn48_len;
			}

			memcpy(((u8*)&pcmd->start_cmd) + pcmd->start_cmd.offset_StaRsnIE,
			              mwl_vif->beacon_info.ie_rsn_ptr,
			              mwl_vif->beacon_info.ie_rsn_len);
		}
		else {
			wiphy_err(priv->hw->wiphy, "WPA IE will not fit in remaining buffer, dropping.");
			pcmd->start_cmd.offset_StaRsnIE = 0;
		}
	}

	//Convert WPA/RSN IE offsets to little endian
	pcmd->start_cmd.offset_StaRsnIE   = cpu_to_le16(pcmd->start_cmd.offset_StaRsnIE);
	pcmd->start_cmd.offset_StaRsnIE48 = cpu_to_le16(pcmd->start_cmd.offset_StaRsnIE48);

	memcpy(pcmd->start_cmd.b_rate_set, mwl_vif->beacon_info.b_rate_set,
	       SYSADPT_MAX_DATA_RATES_G);

	memcpy(pcmd->start_cmd.op_rate_set, mwl_vif->beacon_info.op_rate_set,
	       SYSADPT_MAX_DATA_RATES_G);

	if (mwl_fwcmd_exec_cmd(priv, HOSTCMD_CMD_AP_BEACON)) {
		mutex_unlock(&priv->fwcmd_mutex);
		return -EIO;
	}

	mutex_unlock(&priv->fwcmd_mutex);

	return 0;

ielenerr:

	return -EINVAL;
}

static int mwl_fwcmd_set_spectrum_mgmt(struct mwl_priv *priv, bool enable)
{
	struct hostcmd_cmd_set_spectrum_mgmt *pcmd;

	pcmd = (struct hostcmd_cmd_set_spectrum_mgmt *)&priv->pcmd_buf[
			INTF_CMDHEADER_LEN(priv->if_ops.inttf_head_len)];

	mutex_lock(&priv->fwcmd_mutex);

	memset(pcmd, 0x00, sizeof(*pcmd));
	pcmd->cmd_hdr.cmd = cpu_to_le16(HOSTCMD_CMD_SET_SPECTRUM_MGMT);
	pcmd->cmd_hdr.len = cpu_to_le16(sizeof(*pcmd));
	pcmd->spectrum_mgmt = cpu_to_le32(enable);

	if (mwl_fwcmd_exec_cmd(priv, HOSTCMD_CMD_SET_SPECTRUM_MGMT)) {
		mutex_unlock(&priv->fwcmd_mutex);
		return -EIO;
	}

	mutex_unlock(&priv->fwcmd_mutex);

	return 0;
}

static int mwl_fwcmd_set_power_constraint(struct mwl_priv *priv,
					  u32 power_constraint)
{
	struct hostcmd_cmd_set_power_constraint *pcmd;

	pcmd = (struct hostcmd_cmd_set_power_constraint *)&priv->pcmd_buf[
			INTF_CMDHEADER_LEN(priv->if_ops.inttf_head_len)];

	mutex_lock(&priv->fwcmd_mutex);

	memset(pcmd, 0x00, sizeof(*pcmd));
	pcmd->cmd_hdr.cmd = cpu_to_le16(HOSTCMD_CMD_SET_POWER_CONSTRAINT);
	pcmd->cmd_hdr.len = cpu_to_le16(sizeof(*pcmd));
	pcmd->power_constraint = cpu_to_le32(power_constraint);

	if (mwl_fwcmd_exec_cmd(priv, HOSTCMD_CMD_SET_POWER_CONSTRAINT)) {
		mutex_unlock(&priv->fwcmd_mutex);
		return -EIO;
	}

	mutex_unlock(&priv->fwcmd_mutex);

	return 0;
}

static int mwl_fwcmd_set_country_code(struct mwl_priv *priv,
				      struct mwl_vif *mwl_vif,
				      struct ieee80211_bss_conf *bss_conf)
{
	struct hostcmd_cmd_set_country_code *pcmd;
	struct beacon_info *b_inf = &mwl_vif->beacon_info;
	u8 chnl_len;
	bool a_band;
	bool enable = false;

	if (b_inf->ie_country_ptr) {
		if (bss_conf->chandef.chan->band == NL80211_BAND_2GHZ)
			a_band = false;
		else if (bss_conf->chandef.chan->band == NL80211_BAND_5GHZ)
			a_band = true;
		else
			return -EINVAL;

		chnl_len = b_inf->ie_country_len - 5;
		if (a_band) {
			if (chnl_len > sizeof(pcmd->domain_info.domain_entry_a))
				return -EINVAL;
		} else {
			if (chnl_len > sizeof(pcmd->domain_info.domain_entry_g))
				return -EINVAL;
		}

		enable = true;
	}

	pcmd = (struct hostcmd_cmd_set_country_code *)&priv->pcmd_buf[
			INTF_CMDHEADER_LEN(priv->if_ops.inttf_head_len)];

	mutex_lock(&priv->fwcmd_mutex);

	memset(pcmd, 0x00, sizeof(*pcmd));
	pcmd->cmd_hdr.cmd = cpu_to_le16(HOSTCMD_CMD_SET_COUNTRY_CODE);
	pcmd->cmd_hdr.len = cpu_to_le16(sizeof(*pcmd));
	pcmd->action = cpu_to_le32(enable);
	if (enable) {
		memcpy(pcmd->domain_info.country_string,
		       b_inf->ie_country_ptr + 2, 3);
		if (a_band) {
			pcmd->domain_info.g_chnl_len = 0;
			pcmd->domain_info.a_chnl_len = chnl_len;
			memcpy(pcmd->domain_info.domain_entry_a,
			       b_inf->ie_country_ptr + 5, chnl_len);
		} else {
			pcmd->domain_info.a_chnl_len = 0;
			pcmd->domain_info.g_chnl_len = chnl_len;
			memcpy(pcmd->domain_info.domain_entry_g,
			       b_inf->ie_country_ptr + 5, chnl_len);
		}
	}

	if (mwl_fwcmd_exec_cmd(priv, HOSTCMD_CMD_SET_COUNTRY_CODE)) {
		mutex_unlock(&priv->fwcmd_mutex);
		return -EIO;
	}

	mutex_unlock(&priv->fwcmd_mutex);

	return 0;
}

static int mwl_fwcmd_encryption_set_cmd_info(struct hostcmd_cmd_set_key *cmd,
					     struct ieee80211_vif *vif,
					     u8 *addr,
					     struct ieee80211_key_conf *key)
{
	struct mwl_vif *mwl_vif;

	mwl_vif = mwl_dev_get_vif(vif);

	cmd->cmd_hdr.cmd = cpu_to_le16(HOSTCMD_CMD_UPDATE_ENCRYPTION);
	cmd->cmd_hdr.len = cpu_to_le16(sizeof(*cmd));
	cmd->key_param.length = cpu_to_le16(sizeof(*cmd) -
		offsetof(struct hostcmd_cmd_set_key, key_param));
	cmd->key_param.key_index = cpu_to_le32(key->keyidx);
	cmd->key_param.key_len = cpu_to_le16(key->keylen);
	ether_addr_copy(cmd->key_param.mac_addr, addr);

	switch (key->cipher) {
	case WLAN_CIPHER_SUITE_WEP40:
	case WLAN_CIPHER_SUITE_WEP104:
		cmd->key_param.key_type_id = cpu_to_le16(KEY_TYPE_ID_WEP);
		if ( (key->flags & IEEE80211_KEY_FLAG_PAIRWISE)||
		     (key->keyidx == mwl_vif->tx_key_idx) ) {
			cmd->key_param.key_info = cpu_to_le32(ENCR_KEY_FLAG_WEP_TXKEY);

			if (key->flags & IEEE80211_KEY_FLAG_PAIRWISE) {
				mwl_vif->tx_key_idx = key->keyidx;
			}
		}
		break;
	case WLAN_CIPHER_SUITE_TKIP:
		cmd->key_param.key_type_id = cpu_to_le16(KEY_TYPE_ID_TKIP);
		cmd->key_param.key_info =
			(key->flags & IEEE80211_KEY_FLAG_PAIRWISE) ?
			cpu_to_le32(ENCR_KEY_FLAG_PAIRWISE) :
			cpu_to_le32(ENCR_KEY_FLAG_TXGROUPKEY);
		cmd->key_param.key_info |=
			cpu_to_le32(ENCR_KEY_FLAG_MICKEY_VALID |
				      ENCR_KEY_FLAG_TSC_VALID);
		break;
	case WLAN_CIPHER_SUITE_CCMP:
		cmd->key_param.key_type_id = cpu_to_le16(KEY_TYPE_ID_AES);
		cmd->key_param.key_info =
			(key->flags & IEEE80211_KEY_FLAG_PAIRWISE) ?
			cpu_to_le32(ENCR_KEY_FLAG_PAIRWISE) :
			cpu_to_le32(ENCR_KEY_FLAG_TXGROUPKEY);
		break;
	case WLAN_CIPHER_SUITE_AES_CMAC:
		return 1;
	default:
		return -ENOTSUPP;
	}

	return 0;
}

void mwl_fwcmd_reset(struct ieee80211_hw *hw)
{
	struct mwl_priv *priv = hw->priv;

	if (priv->if_ops.card_reset)
		priv->if_ops.card_reset(priv);
}
EXPORT_SYMBOL_GPL(mwl_fwcmd_reset);

void mwl_fwcmd_int_enable(struct ieee80211_hw *hw)
{
	struct mwl_priv *priv = hw->priv;

	if (priv->if_ops.enable_int)
		priv->if_ops.enable_int(priv);
}

void mwl_fwcmd_int_disable(struct ieee80211_hw *hw)
{
	struct mwl_priv *priv = hw->priv;

	if (priv->if_ops.disable_int)
		priv->if_ops.disable_int(priv);
}

int mwl_fwcmd_get_hw_specs(struct ieee80211_hw *hw)
{
	struct mwl_priv *priv = hw->priv;
	struct hostcmd_cmd_get_hw_spec *pcmd;
	int retry;
	int i;

	pcmd = (struct hostcmd_cmd_get_hw_spec *)&priv->pcmd_buf[
		INTF_CMDHEADER_LEN(priv->if_ops.inttf_head_len)];

	mutex_lock(&priv->fwcmd_mutex);
	for (retry = 0; retry < MAX_WAIT_GET_HW_SPECS_ITERATONS; retry++)
	{
		memset(pcmd, 0x00, sizeof(*pcmd));
		eth_broadcast_addr(pcmd->permanent_addr);
		pcmd->cmd_hdr.cmd = cpu_to_le16(HOSTCMD_CMD_GET_HW_SPEC);
		pcmd->cmd_hdr.len = cpu_to_le16(sizeof(*pcmd));
		pcmd->fw_awake_cookie = cpu_to_le32(priv->pphys_cmd_buf + 2048);

		if (!mwl_fwcmd_exec_cmd(priv, HOSTCMD_CMD_GET_HW_SPEC))
			break;

		msleep(1000);
	}

	if (retry == MAX_WAIT_GET_HW_SPECS_ITERATONS) {
		wiphy_err(hw->wiphy, "can't get hw specs\n");
		mutex_unlock(&priv->fwcmd_mutex);
		return -EIO;
	}

	if (!priv->mfg_mode) {
		//MFG firmware does not return consistent permanent mac, so only
		//use it if running protocol firmware
		ether_addr_copy(&priv->hw_data.mac_addr[0], pcmd->permanent_addr);
	}

	priv->desc_data[0].wcb_base =
		le32_to_cpu(pcmd->wcb_base0) & 0x0000ffff;
	for (i = 1; i < SYSADPT_TOTAL_TX_QUEUES; i++)
		priv->desc_data[i].wcb_base =
			le32_to_cpu(pcmd->wcb_base[i - 1]) & 0x0000ffff;
	priv->desc_data[0].rx_desc_read =
		le32_to_cpu(pcmd->rxpd_rd_ptr) & 0x0000ffff;
	priv->desc_data[0].rx_desc_write =
		le32_to_cpu(pcmd->rxpd_wr_ptr) & 0x0000ffff;
	priv->hw_data.region_code = le16_to_cpu(pcmd->region_code) & 0x00ff;
	priv->hw_data.fw_release_num = le32_to_cpu(pcmd->fw_release_num);
	priv->hw_data.max_num_tx_desc = le16_to_cpu(pcmd->num_wcb);
	priv->hw_data.max_num_mc_addr = le16_to_cpu(pcmd->num_mcast_addr);
	priv->hw_data.num_antennas = le16_to_cpu(pcmd->num_antenna);
	priv->hw_data.hw_version = pcmd->version;
	priv->hw_data.host_interface = pcmd->host_if;

	mutex_unlock(&priv->fwcmd_mutex);

	return 0;
}

int mwl_fwcmd_set_hw_specs(struct ieee80211_hw *hw)
{
	struct mwl_priv *priv = hw->priv;
	struct hostcmd_cmd_set_hw_spec *pcmd;
	int i;

	pcmd = (struct hostcmd_cmd_set_hw_spec *)&priv->pcmd_buf[
			INTF_CMDHEADER_LEN(priv->if_ops.inttf_head_len)];

	mutex_lock(&priv->fwcmd_mutex);

	memset(pcmd, 0x00, sizeof(*pcmd));
	pcmd->cmd_hdr.cmd = cpu_to_le16(HOSTCMD_CMD_SET_HW_SPEC);
	pcmd->cmd_hdr.len = cpu_to_le16(sizeof(*pcmd));

	if ((priv->host_if == MWL_IF_PCIE) &&
		IS_PFU_ENABLED(priv->chip_type)) {
		pcmd->wcb_base[0] = cpu_to_le32(priv->txbd_ring_pbase);
/*  host_spec.txbd_addr_hi =
*       (unsigned int)(((unsigned long long)pmadapter->txbd_ring_pbase)>>32);
*/
		pcmd->tx_wcb_num_per_queue = cpu_to_le32(MLAN_MAX_TXRX_BD);
		pcmd->num_tx_queues = cpu_to_le32(SYSADPT_PFU_NUM_OF_DESC_DATA);
	} else {
		pcmd->wcb_base[0] =
			cpu_to_le32(priv->desc_data[0].pphys_tx_ring);
		for (i = 1; i < SYSADPT_TOTAL_TX_QUEUES; i++)
			pcmd->wcb_base[i] =
				cpu_to_le32(priv->desc_data[i].pphys_tx_ring);
		pcmd->tx_wcb_num_per_queue =
			cpu_to_le32(SYSADPT_MAX_NUM_TX_DESC);
		pcmd->num_tx_queues = cpu_to_le32(SYSADPT_NUM_OF_DESC_DATA);

	}

	pcmd->total_rx_wcb = cpu_to_le32(SYSADPT_MAX_NUM_RX_DESC);
	pcmd->rxpd_wr_ptr = cpu_to_le32(priv->desc_data[0].pphys_rx_ring);
	pcmd->features |= cpu_to_le32(HW_SET_PARMS_FEATURES_HOST_PROBE_RESP);

	if (priv->host_crypto)
		pcmd->features |= cpu_to_le32(HW_SET_PARMS_FEATURES_HOST_ENCRDECRMGT);

	if (mwl_fwcmd_exec_cmd(priv, HOSTCMD_CMD_SET_HW_SPEC)) {
		mutex_unlock(&priv->fwcmd_mutex);
		return -EIO;
	}

	mutex_unlock(&priv->fwcmd_mutex);

	return 0;
}

static u16 wlan_parse_cal_cfg(const u8 *src, size_t len, u8 *dst)
{
	const u8 *ptr;
	u8 *dptr;

	ptr = src;
	dptr = dst;

	while (ptr - src < len) {
		if (*ptr && (isspace(*ptr) || iscntrl(*ptr))) {
			ptr++;
			continue;
		}

		if (isxdigit(*ptr)) {
			*dptr++ = simple_strtol(ptr, NULL, 16);
			ptr += 2;
		} else {
			ptr++;
		}
	}

	return (dptr - dst);
}

int mwl_fwcmd_set_cfg_data(struct ieee80211_hw *hw, u16 type)
{
	struct mwl_priv *priv = hw->priv;
	struct hostcmd_cmd_set_cfg *pcmd;

	if(!priv->cal_data)
		return 0;
	pcmd = (struct hostcmd_cmd_set_cfg *)&priv->pcmd_buf[
				INTF_CMDHEADER_LEN(priv->if_ops.inttf_head_len)];

	mutex_lock(&priv->fwcmd_mutex);

	memset(pcmd, 0x00, sizeof(*pcmd));

	pcmd->data_len = cpu_to_le16(wlan_parse_cal_cfg(priv->cal_data->data,
		priv->cal_data->size, pcmd->data));
	pcmd->cmd_hdr.cmd = cpu_to_le16(HOSTCMD_CMD_SET_CFG);
	pcmd->cmd_hdr.len = cpu_to_le16(sizeof(*pcmd) +
		le16_to_cpu(pcmd->data_len) - sizeof(pcmd->data));
	pcmd->action = cpu_to_le16(HOSTCMD_ACT_GEN_SET);
	pcmd->type = cpu_to_le16(type);

	if (mwl_fwcmd_exec_cmd(priv, HOSTCMD_CMD_SET_CFG)) {
		mutex_unlock(&priv->fwcmd_mutex);
		release_firmware(priv->cal_data);
		priv->cal_data = NULL;
		return -EIO;
	}

	mutex_unlock(&priv->fwcmd_mutex);

	release_firmware(priv->cal_data);
	priv->cal_data = NULL;

	return 0;

}

int mwl_fwcmd_get_stat(struct ieee80211_hw *hw,
		       struct ieee80211_low_level_stats *stats)
{
	struct mwl_priv *priv = hw->priv;
	struct hostcmd_cmd_802_11_get_stat *pcmd;

	pcmd = (struct hostcmd_cmd_802_11_get_stat *)&priv->pcmd_buf[
			INTF_CMDHEADER_LEN(priv->if_ops.inttf_head_len)];

	mutex_lock(&priv->fwcmd_mutex);

	memset(pcmd, 0x00, sizeof(*pcmd));
	pcmd->cmd_hdr.cmd = cpu_to_le16(HOSTCMD_CMD_802_11_GET_STAT);
	pcmd->cmd_hdr.len = cpu_to_le16(sizeof(*pcmd));

	if (mwl_fwcmd_exec_cmd(priv, HOSTCMD_CMD_802_11_GET_STAT)) {
		mutex_unlock(&priv->fwcmd_mutex);
		return -EIO;
	}

	stats->dot11ACKFailureCount =
		le32_to_cpu(pcmd->ack_failures);
	stats->dot11RTSFailureCount =
		le32_to_cpu(pcmd->rts_failures);
	stats->dot11FCSErrorCount =
		le32_to_cpu(pcmd->rx_fcs_errors);
	stats->dot11RTSSuccessCount =
		le32_to_cpu(pcmd->rts_successes);

	mutex_unlock(&priv->fwcmd_mutex);

	return 0;
}

int mwl_fwcmd_reg_mac(struct ieee80211_hw *hw, u8 flag, u32 reg, u32 *val)
{
	struct mwl_priv *priv = hw->priv;
	struct hostcmd_cmd_mac_reg_access *pcmd;

	pcmd = (struct hostcmd_cmd_mac_reg_access *)&priv->pcmd_buf[
				INTF_CMDHEADER_LEN(priv->if_ops.inttf_head_len)];

	mutex_lock(&priv->fwcmd_mutex);

	memset(pcmd, 0x00, sizeof(*pcmd));
	pcmd->cmd_hdr.cmd = cpu_to_le16(HOSTCMD_CMD_MAC_REG_ACCESS);
	pcmd->cmd_hdr.len = cpu_to_le16(sizeof(*pcmd));
	pcmd->offset = cpu_to_le16(reg);
	pcmd->action = cpu_to_le16(flag);
	pcmd->value  = cpu_to_le32(*val);

    if (mwl_fwcmd_exec_cmd(priv, HOSTCMD_CMD_MAC_REG_ACCESS)) {
        mutex_unlock(&priv->fwcmd_mutex);
        return -EIO;
    }

	*val = le32_to_cpu(pcmd->value);

	mutex_unlock(&priv->fwcmd_mutex);
	return 0;
}
EXPORT_SYMBOL_GPL(mwl_fwcmd_reg_mac);

int mwl_fwcmd_reg_bb(struct ieee80211_hw *hw, u8 flag, u32 reg, u32 *val)
{
	struct mwl_priv *priv = hw->priv;
	struct hostcmd_cmd_bbp_reg_access *pcmd;

	pcmd = (struct hostcmd_cmd_bbp_reg_access *)&priv->pcmd_buf[
				INTF_CMDHEADER_LEN(priv->if_ops.inttf_head_len)];

	mutex_lock(&priv->fwcmd_mutex);

	memset(pcmd, 0x00, sizeof(*pcmd));
	pcmd->cmd_hdr.cmd = cpu_to_le16(HOSTCMD_CMD_BBP_REG_ACCESS);
	pcmd->cmd_hdr.len = cpu_to_le16(sizeof(*pcmd));
	pcmd->offset = cpu_to_le16(reg);
	pcmd->action = cpu_to_le16(flag);
	pcmd->value  = (u8)cpu_to_le32(*val);

	if (mwl_fwcmd_exec_cmd(priv, HOSTCMD_CMD_BBP_REG_ACCESS)) {
		mutex_unlock(&priv->fwcmd_mutex);
		return -EIO;
	}

	*val = (u32)pcmd->value;

	mutex_unlock(&priv->fwcmd_mutex);
	return 0;
}
EXPORT_SYMBOL_GPL(mwl_fwcmd_reg_bb);

int mwl_fwcmd_reg_rf(struct ieee80211_hw *hw, u8 flag, u32 reg, u32 *val)
{
	struct mwl_priv *priv = hw->priv;
	struct hostcmd_cmd_rf_reg_access *pcmd;

	pcmd = (struct hostcmd_cmd_rf_reg_access *)&priv->pcmd_buf[
				INTF_CMDHEADER_LEN(priv->if_ops.inttf_head_len)];

	mutex_lock(&priv->fwcmd_mutex);

	memset(pcmd, 0x00, sizeof(*pcmd));
	pcmd->cmd_hdr.cmd = cpu_to_le16(HOSTCMD_CMD_RF_REG_ACCESS);
	pcmd->cmd_hdr.len = cpu_to_le16(sizeof(*pcmd));
	pcmd->offset = cpu_to_le16(reg);
	pcmd->action = cpu_to_le16(flag);
	pcmd->value  = (u8)cpu_to_le32(*val);

	if (mwl_fwcmd_exec_cmd(priv, HOSTCMD_CMD_RF_REG_ACCESS)) {
		mutex_unlock(&priv->fwcmd_mutex);
		return -EIO;
	}

	*val = (u32)pcmd->value;

	mutex_unlock(&priv->fwcmd_mutex);

	return 0;
}
EXPORT_SYMBOL_GPL(mwl_fwcmd_reg_rf);

int mwl_fwcmd_radio_enable(struct ieee80211_hw *hw)
{
	return mwl_fwcmd_802_11_radio_control(hw->priv, true, false);
}

int mwl_fwcmd_radio_disable(struct ieee80211_hw *hw)
{
	return mwl_fwcmd_802_11_radio_control(hw->priv, false, false);
}

int mwl_fwcmd_set_radio_preamble(struct ieee80211_hw *hw, bool short_preamble)
{
	struct mwl_priv *priv = hw->priv;
	int rc;

	priv->radio_short_preamble = short_preamble;
	rc = mwl_fwcmd_802_11_radio_control(priv, true, true);

	return rc;
}

int mwl_fwcmd_get_addr_value(struct ieee80211_hw *hw, u32 addr, u32 len,
			     u32 *val, u16 set)
{
	struct mwl_priv *priv = hw->priv;
	struct hostcmd_cmd_mem_addr_access *pcmd;
	int i;

	pcmd = (struct hostcmd_cmd_mem_addr_access *)&priv->pcmd_buf[
			INTF_CMDHEADER_LEN(priv->if_ops.inttf_head_len)];

	mutex_lock(&priv->fwcmd_mutex);

	memset(pcmd, 0x00, sizeof(*pcmd));
	pcmd->cmd_hdr.cmd = cpu_to_le16(HOSTCMD_CMD_MEM_ADDR_ACCESS);
	pcmd->cmd_hdr.len = cpu_to_le16(sizeof(*pcmd));
	pcmd->address = cpu_to_le32(addr);
	pcmd->length = cpu_to_le16(len);
	pcmd->value[0] = cpu_to_le32(*val);
	pcmd->reserved = cpu_to_le16(set);

	if (mwl_fwcmd_exec_cmd(priv, HOSTCMD_CMD_MEM_ADDR_ACCESS)) {
		mutex_unlock(&priv->fwcmd_mutex);
		return -EIO;
	}

	for (i = 0; i < len; i++)
		val[i] = le32_to_cpu(pcmd->value[i]);

	mutex_unlock(&priv->fwcmd_mutex);

	return 0;
}
EXPORT_SYMBOL_GPL(mwl_fwcmd_get_addr_value);

int mwl_fwcmd_tx_power(struct ieee80211_hw *hw,
		       struct ieee80211_conf *conf, u16 level)
{
	struct ieee80211_channel *channel = conf->chandef.chan;
	struct mwl_priv *priv = hw->priv;
	u16 band = 0, width = 0, sub_ch = 0;
	u16 txpow[SYSADPT_TX_GRP_PWR_LEVEL_TOTAL];
	int i  = 0;
	int rc = 0;

	if (channel->band == NL80211_BAND_2GHZ)
		band = FREQ_BAND_2DOT4GHZ;
	else if (channel->band == NL80211_BAND_5GHZ)
		band = FREQ_BAND_5GHZ;

	switch (conf->chandef.width) {
	case NL80211_CHAN_WIDTH_20_NOHT:
	case NL80211_CHAN_WIDTH_20:
		width = CH_20_MHZ_WIDTH;
		sub_ch = NO_EXT_CHANNEL;
		break;
	case NL80211_CHAN_WIDTH_40:
		width = CH_40_MHZ_WIDTH;
		if (conf->chandef.center_freq1 > channel->center_freq)
			sub_ch = EXT_CH_ABOVE_CTRL_CH;
		else
			sub_ch = EXT_CH_BELOW_CTRL_CH;
		break;
	case NL80211_CHAN_WIDTH_80:
		width = CH_80_MHZ_WIDTH;
		if (conf->chandef.center_freq1 > channel->center_freq)
			sub_ch = EXT_CH_ABOVE_CTRL_CH;
		else
			sub_ch = EXT_CH_BELOW_CTRL_CH;
		break;
	default:
		return -EINVAL;
	}

	for (i = 0; i < SYSADPT_TX_GRP_PWR_LEVEL_TOTAL; i++) {
		txpow[i] = level;
	}

	rc = mwl_fwcmd_set_tx_powers(priv, txpow, HOSTCMD_ACT_SET_TARGET_TX_PWR,
				     channel->hw_value, band, width, sub_ch);

	return rc;
}

int mwl_fwcmd_rf_antenna(struct ieee80211_hw *hw, int ant_tx_bmp,
		int ant_rx_bmp)
{
	struct mwl_priv *priv = hw->priv;
	struct hostcmd_cmd_802_11_rf_antenna_v2 *pcmd;
	int retval = 0;

	pcmd = (struct hostcmd_cmd_802_11_rf_antenna_v2 *)&priv->pcmd_buf[
			INTF_CMDHEADER_LEN(priv->if_ops.inttf_head_len)];

	mutex_lock(&priv->fwcmd_mutex);

	memset(pcmd, 0x00, sizeof(*pcmd));
	pcmd->cmd_hdr.cmd =
		cpu_to_le16(HOSTCMD_CMD_802_11_RF_ANTENNA_V2);
	pcmd->cmd_hdr.len = cpu_to_le16(sizeof(*pcmd));

	pcmd->action = cpu_to_le16(0);  //Not need - just making things obvious

	pcmd->ant_tx_bmp = cpu_to_le16(ant_tx_bmp);
	pcmd->ant_rx_bmp = cpu_to_le16(ant_rx_bmp);

	if (mwl_fwcmd_exec_cmd(priv, HOSTCMD_CMD_802_11_RF_ANTENNA_V2)) {
		mutex_unlock(&priv->fwcmd_mutex);
		return -EIO;
	}

	if (pcmd->cmd_hdr.result != 0) {
		wiphy_err(hw->wiphy, "Command %s Rejected by FW\n",
		mwl_fwcmd_get_cmd_string(HOSTCMD_CMD_802_11_RF_ANTENNA_V2));
		retval = -EINVAL;
	}

	mutex_unlock(&priv->fwcmd_mutex);

	return retval;
}

int mwl_fwcmd_broadcast_ssid_enable(struct ieee80211_hw *hw,
				   struct mwl_vif *mwl_vif,
				   struct ieee80211_vif *vif)
{
	struct mwl_priv *priv = hw->priv;
	struct hostcmd_cmd_broadcast_ssid_enable *pcmd;

	pcmd = (struct hostcmd_cmd_broadcast_ssid_enable *)&priv->pcmd_buf[
			INTF_CMDHEADER_LEN(priv->if_ops.inttf_head_len)];

	mutex_lock(&priv->fwcmd_mutex);

	memset(pcmd, 0x00, sizeof(*pcmd));
	pcmd->cmd_hdr.cmd = cpu_to_le16(HOSTCMD_CMD_BROADCAST_SSID_ENABLE);
	pcmd->cmd_hdr.len = cpu_to_le16(sizeof(*pcmd));
	pcmd->cmd_hdr.macid = mwl_vif->macid;

	if ((vif->cfg.ssid[0] != '\0') &&
	    (vif->cfg.ssid_len != 0) &&
	    (!vif->bss_conf.hidden_ssid))
		pcmd->enable = cpu_to_le32(true);
	else
		pcmd->enable = cpu_to_le32(false);

	if (vif->bss_conf.hidden_ssid) {
		if((mwl_vif->beacon_info.ie_ssid_len) - 2)
			pcmd->hidden_ssid_info = cpu_to_le32(NL80211_HIDDEN_SSID_ZERO_CONTENTS);
		else
			pcmd->hidden_ssid_info = cpu_to_le32(NL80211_HIDDEN_SSID_ZERO_LEN);
	}
	else
		pcmd->hidden_ssid_info = cpu_to_le32(NL80211_HIDDEN_SSID_NOT_IN_USE);

	if (mwl_fwcmd_exec_cmd(priv, HOSTCMD_CMD_BROADCAST_SSID_ENABLE)) {
		mutex_unlock(&priv->fwcmd_mutex);
		return -EIO;
	}

	mutex_unlock(&priv->fwcmd_mutex);

	return 0;
}

int mwl_fwcmd_powersave_EnblDsbl(struct ieee80211_hw *hw,
				struct ieee80211_conf *conf)
{
	struct mwl_priv *priv = hw->priv;
	struct hostcmd_cmd_802_11_ps_mode *pcmd;

	pcmd = (struct hostcmd_cmd_802_11_ps_mode *)&priv->pcmd_buf[
			INTF_CMDHEADER_LEN(priv->if_ops.inttf_head_len)];


	mutex_lock(&priv->fwcmd_mutex);

	memset(pcmd, 0x00, sizeof(*pcmd));
	pcmd->cmd_hdr.cmd = cpu_to_le16(HOSTCMD_CMD_802_11_PS_MODE);
	pcmd->cmd_hdr.len = cpu_to_le16(sizeof(*pcmd));
	pcmd->action = cpu_to_le16(WL_SET);
	pcmd->powermode = cpu_to_le16(conf->flags & IEEE80211_CONF_PS);
	priv->ps_mode = le16_to_cpu(pcmd->powermode);

	if (mwl_fwcmd_exec_cmd(priv, HOSTCMD_CMD_802_11_PS_MODE)) {
		mutex_unlock(&priv->fwcmd_mutex);
		return -EIO;
	}

	mutex_unlock(&priv->fwcmd_mutex);

	return 0;
}

#ifdef CONFIG_PM
int mwl_fwcmd_hostsleep_control(struct ieee80211_hw *hw, bool hs_enable, bool ds_enable, int wakeupCond)
{
	struct mwl_priv *priv = hw->priv;
	struct hostcmd_cmd_hostsleep_ctrl *pcmd;

 	pcmd = (struct hostcmd_cmd_hostsleep_ctrl *)&priv->pcmd_buf[
			INTF_CMDHEADER_LEN(priv->if_ops.inttf_head_len)];

	mutex_lock(&priv->fwcmd_mutex);

	memset(pcmd, 0x00, sizeof(*pcmd));
	pcmd->cmd_hdr.cmd = cpu_to_le16(HOSTCMD_CMD_HOSTSLEEP_CTRL);
	pcmd->cmd_hdr.len = cpu_to_le16(sizeof(*pcmd));
	pcmd->HS_enable = hs_enable;
	pcmd->DS_enable = ds_enable;
	pcmd->gap = cpu_to_le16(WOWLAN_WAKEUP_GAP_CFG);
	pcmd->wakeupSignal = priv->wow.wakeSigType;
	pcmd->wakeUpConditions = cpu_to_le32(wakeupCond);
	//Enable below code only for debug purpose.
	//It allows FW to unconditionally upload Rxed beacons.
	//pcmd->options = cpu_to_le32(0x1);

	if (mwl_fwcmd_exec_cmd(priv, HOSTCMD_CMD_HOSTSLEEP_CTRL) ||
	    (pcmd->cmd_hdr.result != cpu_to_le16(HOSTCMD_RESULT_OK))) {
		mutex_unlock(&priv->fwcmd_mutex);
		wiphy_err(hw->wiphy, " %s failed execution %x\n",
		          mwl_fwcmd_get_cmd_string(HOSTCMD_CMD_HOSTSLEEP_CTRL),
		          pcmd->cmd_hdr.result);
		return -EIO;
	}

	if (ds_enable) {
		//We told firmware to enter deepsleep - set ds state accordingly
		priv->if_ops.enter_deepsleep(priv);
	}

	mutex_unlock(&priv->fwcmd_mutex);

	return 0;
}
#endif

#ifdef CONFIG_PM
int mwl_fwcmd_wowlan_apinrange_config(struct ieee80211_hw *hw)
{
	struct mwl_priv *priv = hw->priv;
	struct hostcmd_cmd_wowlan_ap_inrange_cfg *pcmd;

	pcmd = (struct hostcmd_cmd_wowlan_ap_inrange_cfg *)&priv->pcmd_buf[
			INTF_CMDHEADER_LEN(priv->if_ops.inttf_head_len)];

	mutex_lock(&priv->fwcmd_mutex);

	memset(pcmd, 0, sizeof(*pcmd));

	pcmd->cmd_hdr.cmd = cpu_to_le16(HOSTCMD_CMD_WOWLAN_AP_INRANGE_CFG);
	pcmd->cmd_hdr.len = cpu_to_le16(sizeof(*pcmd));

	if (priv->wow.addrListCnt) {
		/* Fill in BSSID */
		pcmd->addrIeList_Len = cpu_to_le16(priv->wow.addrListCnt * sizeof(struct wowlan_apinrange_addrIe));
		memcpy(&pcmd->addrIeList, priv->wow.addrList, sizeof(struct wowlan_apinrange_addrIe));
	}

	if (priv->wow.ssidListCnt) {
		/* Fill in SSID */
		pcmd->ssidIeList_Len = cpu_to_le16(priv->wow.ssidListCnt * sizeof(struct wowlan_apinrange_ssidIe));
		memcpy(&pcmd->ssidIeList, priv->wow.ssidList, sizeof(struct wowlan_apinrange_ssidIe));
	}

	/* Fill in the list of channels to scan */
	pcmd->chanListCnt = cpu_to_le16(min(priv->wow.channelCnt, (u16)sizeof(pcmd->chanList)));
	memcpy(pcmd->chanList, priv->wow.channels, le16_to_cpu(pcmd->chanListCnt));

	if (mwl_fwcmd_exec_cmd(priv, HOSTCMD_CMD_WOWLAN_AP_INRANGE_CFG)) {
		mutex_unlock(&priv->fwcmd_mutex);
		wiphy_err(hw->wiphy, " %s failed execution\n", mwl_fwcmd_get_cmd_string(HOSTCMD_CMD_WOWLAN_AP_INRANGE_CFG));
		return -EIO;
	}

	mutex_unlock(&priv->fwcmd_mutex);

	return 0;
}
#endif

int mwl_fwcmd_set_roc_channel(struct ieee80211_hw *hw,
					struct ieee80211_channel *channel)
{
	struct mwl_priv *priv = hw->priv;
	struct hostcmd_cmd_set_rf_channel *pcmd;
	u32 chnl_flags, freq_band = FREQ_BAND_2DOT4GHZ;
	u32 chnl_width, act_primary;

	if (channel->band == NL80211_BAND_2GHZ) {
		freq_band = FREQ_BAND_2DOT4GHZ;
	} else if (channel->band == NL80211_BAND_5GHZ) {
		freq_band = FREQ_BAND_5GHZ;
	}

	chnl_width = CH_20_MHZ_WIDTH;
	act_primary = ACT_PRIMARY_CHAN_0;
	chnl_flags = (freq_band & FREQ_BAND_MASK) |
					((chnl_width << CHNL_WIDTH_SHIFT) & CHNL_WIDTH_MASK) |
					((act_primary << ACT_PRIMARY_SHIFT) & ACT_PRIMARY_MASK);

	pcmd = (struct hostcmd_cmd_set_rf_channel *)&priv->pcmd_buf[
			INTF_CMDHEADER_LEN(priv->if_ops.inttf_head_len)];

	mutex_lock(&priv->fwcmd_mutex);

	memset(pcmd, 0x00, sizeof(*pcmd));
	pcmd->cmd_hdr.cmd = cpu_to_le16(HOSTCMD_CMD_SET_RF_CHANNEL);
	pcmd->cmd_hdr.len = cpu_to_le16(sizeof(*pcmd));
	pcmd->action = cpu_to_le16(WL_SET);
	pcmd->curr_chnl = channel->hw_value;
	pcmd->remain_on_chan = 1;
	pcmd->chnl_flags = cpu_to_le32(chnl_flags);

	if (mwl_fwcmd_exec_cmd(hw->priv, HOSTCMD_CMD_SET_RF_CHANNEL)) {
		mutex_unlock(&priv->fwcmd_mutex);
		return -EIO;
	}

	if (pcmd->cmd_hdr.result != 0) {
		mutex_unlock(&priv->fwcmd_mutex);
		return -EINVAL;
	}

	mutex_unlock(&priv->fwcmd_mutex);
	return 0;
}

int mwl_config_remain_on_channel(struct ieee80211_hw *hw,
					struct ieee80211_channel *channel,
					bool remain_on_channel, int duration,
					enum ieee80211_roc_type type)
{
	struct mwl_priv *priv = hw->priv;
	struct ieee80211_conf *conf = &hw->conf;
	int rc = 0;

	if (remain_on_channel) {
		rc = mwl_fwcmd_radio_enable(hw);
	} else {
		channel = conf->chandef.chan;
		if (conf->flags & IEEE80211_CONF_IDLE)
			rc = mwl_fwcmd_radio_disable(hw);
	}

	if (rc)
		goto out;

	if (channel->band == NL80211_BAND_2GHZ) {
		mwl_fwcmd_set_apmode(hw, AP_MODE_2_4GHZ_11AC_MIXED);
		mwl_fwcmd_set_linkadapt_cs_mode(hw,
						LINK_CS_STATE_CONSERV);
	} else if (channel->band == NL80211_BAND_5GHZ) {
		mwl_fwcmd_set_apmode(hw, AP_MODE_11AC);
		mwl_fwcmd_set_linkadapt_cs_mode(hw,
						LINK_CS_STATE_AUTO);
	}

	if (remain_on_channel)
		rc = mwl_fwcmd_set_roc_channel(hw, channel);
	else
		rc = mwl_fwcmd_set_rf_channel(hw, conf);

	if (rc)
		goto out;

	priv->roc.in_progress = remain_on_channel;
	priv->roc.chan = channel->hw_value;
	priv->roc.duration = duration;
	priv->roc.type = type;
out:
	return rc;
}

int mwl_fwcmd_set_rf_channel(struct ieee80211_hw *hw,
			struct ieee80211_conf *conf)
{
	struct ieee80211_channel *channel = conf->chandef.chan;
	struct mwl_priv *priv = hw->priv;
	struct hostcmd_cmd_set_rf_channel *pcmd;
	u32 chnl_flags, freq_band, chnl_width, act_primary;

	pcmd = (struct hostcmd_cmd_set_rf_channel *)&priv->pcmd_buf[
			INTF_CMDHEADER_LEN(priv->if_ops.inttf_head_len)];

	mutex_lock(&priv->fwcmd_mutex);

	memset(pcmd, 0x00, sizeof(*pcmd));
	pcmd->cmd_hdr.cmd = cpu_to_le16(HOSTCMD_CMD_SET_RF_CHANNEL);
	pcmd->cmd_hdr.len = cpu_to_le16(sizeof(*pcmd));
	pcmd->action = cpu_to_le16(WL_SET);
	pcmd->curr_chnl = channel->hw_value;

	if (channel->band == NL80211_BAND_2GHZ) {
		freq_band = FREQ_BAND_2DOT4GHZ;
	} else if (channel->band == NL80211_BAND_5GHZ) {
		freq_band = FREQ_BAND_5GHZ;
	} else {
		mutex_unlock(&priv->fwcmd_mutex);
		return -EINVAL;
	}

	switch (conf->chandef.width) {
	case NL80211_CHAN_WIDTH_20_NOHT:
	case NL80211_CHAN_WIDTH_20:
		chnl_width = CH_20_MHZ_WIDTH;
		act_primary = ACT_PRIMARY_CHAN_0;
		break;
	case NL80211_CHAN_WIDTH_40:
		chnl_width = CH_40_MHZ_WIDTH;
		if (conf->chandef.center_freq1 > channel->center_freq)
			act_primary = ACT_PRIMARY_CHAN_0;
		else
			act_primary = ACT_PRIMARY_CHAN_1;
		break;
	case NL80211_CHAN_WIDTH_80:
		chnl_width = CH_80_MHZ_WIDTH;
		act_primary =
			mwl_fwcmd_get_80m_pri_chnl(pcmd->curr_chnl);
		break;
	case NL80211_CHAN_WIDTH_160:
		chnl_width = CH_160_MHZ_WIDTH;
		act_primary =
			mwl_fwcmd_get_160m_pri_chnl(pcmd->curr_chnl);
		break;
	default:
		mutex_unlock(&priv->fwcmd_mutex);
		return -EINVAL;
	}

	chnl_flags = (freq_band & FREQ_BAND_MASK) |
		((chnl_width << CHNL_WIDTH_SHIFT) & CHNL_WIDTH_MASK) |
		((act_primary << ACT_PRIMARY_SHIFT) & ACT_PRIMARY_MASK);

	pcmd->chnl_flags = cpu_to_le32(chnl_flags);

	if (mwl_fwcmd_exec_cmd(priv, HOSTCMD_CMD_SET_RF_CHANNEL)) {
		mutex_unlock(&priv->fwcmd_mutex);
		return -EIO;
	}

	if (pcmd->cmd_hdr.result != 0) {
		mutex_unlock(&priv->fwcmd_mutex);
		return -EINVAL;
	}

	mutex_unlock(&priv->fwcmd_mutex);

	if (priv->roc.in_progress)
		return 0;

	if (priv->sw_scanning && priv->cur_survey_info.filled) {
		int i;
		for (i = 0; i < priv->survey_info_idx; i++) {
			if (priv->cur_survey_info.channel.hw_value
					== priv->survey_info[i].channel.hw_value) {
				break;
			}
		}
		memcpy(&priv->survey_info[i], &priv->cur_survey_info,
			   sizeof(struct mwl_survey_info));
		if (i == priv->survey_info_idx)
			priv->survey_info_idx++;
	}

	memset(&priv->cur_survey_info, 0x0, sizeof(struct mwl_survey_info));
	memcpy(&priv->cur_survey_info.channel, conf->chandef.chan,
		   sizeof(struct ieee80211_channel));
	priv->cur_survey_valid = true;

	if(mwl_fwcmd_get_survey(hw, 1)) {
		return -EIO;
	}

	return 0;
}

int mwl_fwcmd_set_aid(struct ieee80211_hw *hw,
		      struct ieee80211_vif *vif, u8 *bssid, u16 aid)
{
	struct mwl_priv *priv = hw->priv;
	struct mwl_vif *mwl_vif;
	struct hostcmd_cmd_set_aid *pcmd;

	mwl_vif = mwl_dev_get_vif(vif);

	pcmd = (struct hostcmd_cmd_set_aid *)&priv->pcmd_buf[
			INTF_CMDHEADER_LEN(priv->if_ops.inttf_head_len)];

	mutex_lock(&priv->fwcmd_mutex);

	memset(pcmd, 0x00, sizeof(*pcmd));
	pcmd->cmd_hdr.cmd = cpu_to_le16(HOSTCMD_CMD_SET_AID);
	pcmd->cmd_hdr.len = cpu_to_le16(sizeof(*pcmd));
	pcmd->cmd_hdr.macid = mwl_vif->macid;
	pcmd->aid = cpu_to_le16(aid);
	ether_addr_copy(pcmd->mac_addr, bssid);

	if (mwl_fwcmd_exec_cmd(priv, HOSTCMD_CMD_SET_AID)) {
		mutex_unlock(&priv->fwcmd_mutex);
		return -EIO;
	}

	mutex_unlock(&priv->fwcmd_mutex);

	return 0;
}

int mwl_fwcmd_set_infra_mode(struct ieee80211_hw *hw,
			     struct ieee80211_vif *vif)
{
	struct mwl_priv *priv = hw->priv;
	struct mwl_vif *mwl_vif;
	struct hostcmd_cmd_set_infra_mode *pcmd;

	mwl_vif = mwl_dev_get_vif(vif);

	pcmd = (struct hostcmd_cmd_set_infra_mode *)&priv->pcmd_buf[
			INTF_CMDHEADER_LEN(priv->if_ops.inttf_head_len)];

	mutex_lock(&priv->fwcmd_mutex);

	memset(pcmd, 0x00, sizeof(*pcmd));
	pcmd->cmd_hdr.cmd = cpu_to_le16(HOSTCMD_CMD_SET_INFRA_MODE);
	pcmd->cmd_hdr.len = cpu_to_le16(sizeof(*pcmd));
	pcmd->cmd_hdr.macid = mwl_vif->macid;

	if (mwl_fwcmd_exec_cmd(priv, HOSTCMD_CMD_SET_INFRA_MODE)) {
		mutex_unlock(&priv->fwcmd_mutex);
		return -EIO;
	}

	mutex_unlock(&priv->fwcmd_mutex);

	return 0;
}

int mwl_fwcmd_set_rts_threshold(struct ieee80211_hw *hw, int threshold)
{
	struct mwl_priv *priv = hw->priv;
	struct hostcmd_cmd_802_11_rts_thsd *pcmd;

	pcmd = (struct hostcmd_cmd_802_11_rts_thsd *)&priv->pcmd_buf[
			INTF_CMDHEADER_LEN(priv->if_ops.inttf_head_len)];

	mutex_lock(&priv->fwcmd_mutex);

	memset(pcmd, 0x00, sizeof(*pcmd));
	pcmd->cmd_hdr.cmd = cpu_to_le16(HOSTCMD_CMD_802_11_RTS_THSD);
	pcmd->cmd_hdr.len = cpu_to_le16(sizeof(*pcmd));
	pcmd->action  = cpu_to_le16(WL_SET);
	pcmd->threshold = cpu_to_le16(threshold);

	if (mwl_fwcmd_exec_cmd(priv, HOSTCMD_CMD_802_11_RTS_THSD)) {
		mutex_unlock(&priv->fwcmd_mutex);
		return -EIO;
	}

	mutex_unlock(&priv->fwcmd_mutex);

	return 0;
}

int mwl_fwcmd_set_edca_params(struct ieee80211_hw *hw, u8 index,
			      u16 cw_min, u16 cw_max, u8 aifs, u16 txop)
{
	struct mwl_priv *priv = hw->priv;
	struct hostcmd_cmd_set_edca_params *pcmd;

	pcmd = (struct hostcmd_cmd_set_edca_params *)&priv->pcmd_buf[
			INTF_CMDHEADER_LEN(priv->if_ops.inttf_head_len)];

	mutex_lock(&priv->fwcmd_mutex);

	memset(pcmd, 0x00, sizeof(*pcmd));
	pcmd->cmd_hdr.cmd = cpu_to_le16(HOSTCMD_CMD_SET_EDCA_PARAMS);
	pcmd->cmd_hdr.len = cpu_to_le16(sizeof(*pcmd));

	pcmd->action = cpu_to_le16(0xffff);
	pcmd->txop = cpu_to_le16(txop);
	pcmd->cw_max = cpu_to_le32(cw_max);
	pcmd->cw_min = cpu_to_le32(cw_min);
	pcmd->aifsn = aifs;
	pcmd->txq_num = index;

	if (mwl_fwcmd_exec_cmd(priv, HOSTCMD_CMD_SET_EDCA_PARAMS)) {
		mutex_unlock(&priv->fwcmd_mutex);
		return -EIO;
	}

	mutex_unlock(&priv->fwcmd_mutex);

	return 0;
}

int mwl_fwcmd_set_radar_detect(struct ieee80211_hw *hw, u16 action)
{
	struct mwl_priv *priv = hw->priv;
	struct hostcmd_cmd_802_11h_detect_radar *pcmd;
	u16 radar_type = RADAR_TYPE_CODE_0;
	u8 channel = hw->conf.chandef.chan->hw_value;

	pcmd = (struct hostcmd_cmd_802_11h_detect_radar *)&priv->pcmd_buf[
			INTF_CMDHEADER_LEN(priv->if_ops.inttf_head_len)];

	if (priv->reg.dfs_region == NL80211_DFS_JP) {
		if (channel >= 52 && channel <= 64)
			radar_type = RADAR_TYPE_CODE_53;
		else if (channel >= 100 && channel <= 140)
			radar_type = RADAR_TYPE_CODE_56;
		else
			radar_type = RADAR_TYPE_CODE_0;
	} else if (priv->reg.dfs_region == NL80211_DFS_ETSI) {
		radar_type = RADAR_TYPE_CODE_ETSI;
	}

	mutex_lock(&priv->fwcmd_mutex);

	memset(pcmd, 0x00, sizeof(*pcmd));
	pcmd->cmd_hdr.cmd = cpu_to_le16(HOSTCMD_CMD_802_11H_DETECT_RADAR);
	pcmd->cmd_hdr.len = cpu_to_le16(sizeof(*pcmd));
	pcmd->action = cpu_to_le16(action);
	pcmd->radar_type_code = cpu_to_le16(radar_type);
	pcmd->min_chirp_cnt = cpu_to_le16(priv->dfs_chirp_count_min);
	pcmd->chirp_time_intvl = cpu_to_le16(priv->dfs_chirp_time_interval);
	pcmd->pw_filter = cpu_to_le16(priv->dfs_pw_filter);
	pcmd->min_num_radar = cpu_to_le16(priv->dfs_min_num_radar);
	pcmd->pri_min_num = cpu_to_le16(priv->dfs_min_pri_count);

	if (mwl_fwcmd_exec_cmd(priv, HOSTCMD_CMD_802_11H_DETECT_RADAR)) {
		mutex_unlock(&priv->fwcmd_mutex);
		return -EIO;
	}

	mutex_unlock(&priv->fwcmd_mutex);

	return 0;
}

int mwl_fwcmd_set_wmm_mode(struct ieee80211_hw *hw, bool enable)
{
	struct mwl_priv *priv = hw->priv;
	struct hostcmd_cmd_set_wmm_mode *pcmd;

	pcmd = (struct hostcmd_cmd_set_wmm_mode *)&priv->pcmd_buf[
			INTF_CMDHEADER_LEN(priv->if_ops.inttf_head_len)];

	mutex_lock(&priv->fwcmd_mutex);

	memset(pcmd, 0x00, sizeof(*pcmd));
	pcmd->cmd_hdr.cmd = cpu_to_le16(HOSTCMD_CMD_SET_WMM_MODE);
	pcmd->cmd_hdr.len = cpu_to_le16(sizeof(*pcmd));
	pcmd->action = cpu_to_le16(enable ? WL_ENABLE : WL_DISABLE);

	if (mwl_fwcmd_exec_cmd(priv, HOSTCMD_CMD_SET_WMM_MODE)) {
		mutex_unlock(&priv->fwcmd_mutex);
		return -EIO;
	}

	mutex_unlock(&priv->fwcmd_mutex);

	return 0;
}

int mwl_fwcmd_ht_guard_interval(struct ieee80211_hw *hw, u32 gi_type)
{
	struct mwl_priv *priv = hw->priv;
	struct hostcmd_cmd_ht_guard_interval *pcmd;

	pcmd = (struct hostcmd_cmd_ht_guard_interval *)&priv->pcmd_buf[
			INTF_CMDHEADER_LEN(priv->if_ops.inttf_head_len)];

	mutex_lock(&priv->fwcmd_mutex);

	memset(pcmd, 0x00, sizeof(*pcmd));
	pcmd->cmd_hdr.cmd = cpu_to_le16(HOSTCMD_CMD_HT_GUARD_INTERVAL);
	pcmd->cmd_hdr.len = cpu_to_le16(sizeof(*pcmd));
	pcmd->action = cpu_to_le32(WL_SET);
	pcmd->gi_type = cpu_to_le32(gi_type);

	if (mwl_fwcmd_exec_cmd(priv, HOSTCMD_CMD_HT_GUARD_INTERVAL)) {
		mutex_unlock(&priv->fwcmd_mutex);
		return -EIO;
	}

	mutex_unlock(&priv->fwcmd_mutex);

	return 0;
}

int mwl_fwcmd_use_fixed_rate(struct ieee80211_hw *hw, int mcast, int mgmt)
{
	struct mwl_priv *priv = hw->priv;
	struct hostcmd_cmd_set_fixed_rate *pcmd;

	pcmd = (struct hostcmd_cmd_set_fixed_rate *)&priv->pcmd_buf[
			INTF_CMDHEADER_LEN(priv->if_ops.inttf_head_len)];

	mutex_lock(&priv->fwcmd_mutex);

	memset(pcmd, 0x00, sizeof(*pcmd));
	pcmd->cmd_hdr.cmd = cpu_to_le16(HOSTCMD_CMD_SET_FIXED_RATE);
	pcmd->cmd_hdr.len = cpu_to_le16(sizeof(*pcmd));

	pcmd->action = cpu_to_le32(HOSTCMD_ACT_NOT_USE_FIXED_RATE);
	pcmd->multicast_rate = mcast;
	pcmd->management_rate = mgmt;

	if (mwl_fwcmd_exec_cmd(priv, HOSTCMD_CMD_SET_FIXED_RATE)) {
		mutex_unlock(&priv->fwcmd_mutex);
		return -EIO;
	}

	mutex_unlock(&priv->fwcmd_mutex);

	return 0;
}

int mwl_fwcmd_set_linkadapt_cs_mode(struct ieee80211_hw *hw, u16 cs_mode)
{
	struct mwl_priv *priv = hw->priv;
	struct hostcmd_cmd_set_linkadapt_cs_mode *pcmd;

	pcmd = (struct hostcmd_cmd_set_linkadapt_cs_mode *)&priv->pcmd_buf[
			INTF_CMDHEADER_LEN(priv->if_ops.inttf_head_len)];

	mutex_lock(&priv->fwcmd_mutex);

	memset(pcmd, 0x00, sizeof(*pcmd));
	pcmd->cmd_hdr.cmd = cpu_to_le16(HOSTCMD_CMD_SET_LINKADAPT_CS_MODE);
	pcmd->cmd_hdr.len = cpu_to_le16(sizeof(*pcmd));
	pcmd->action  = cpu_to_le16(HOSTCMD_ACT_GEN_SET);
	pcmd->cs_mode = cpu_to_le16(cs_mode);

	if (mwl_fwcmd_exec_cmd(priv, HOSTCMD_CMD_SET_LINKADAPT_CS_MODE)) {
		mutex_unlock(&priv->fwcmd_mutex);
		return -EIO;
	}

	mutex_unlock(&priv->fwcmd_mutex);

	return 0;
}

int mwl_fwcmd_set_rate_adapt_mode(struct ieee80211_hw *hw, u16 mode)
{
	struct mwl_priv *priv = hw->priv;
	struct hostcmd_cmd_set_rate_adapt_mode *pcmd;

	pcmd = (struct hostcmd_cmd_set_rate_adapt_mode *)&priv->pcmd_buf[
			INTF_CMDHEADER_LEN(priv->if_ops.inttf_head_len)];

	mutex_lock(&priv->fwcmd_mutex);

	memset(pcmd, 0x00, sizeof(*pcmd));
	pcmd->cmd_hdr.cmd = cpu_to_le16(HOSTCMD_CMD_SET_RATE_ADAPT_MODE);
	pcmd->cmd_hdr.len = cpu_to_le16(sizeof(*pcmd));
	pcmd->action = cpu_to_le16(WL_SET);
	pcmd->rate_adapt_mode = cpu_to_le16(mode);

	if (mwl_fwcmd_exec_cmd(priv, HOSTCMD_CMD_SET_RATE_ADAPT_MODE)) {
		mutex_unlock(&priv->fwcmd_mutex);
		return -EIO;
	}

	mutex_unlock(&priv->fwcmd_mutex);

	return 0;
}

int mwl_fwcmd_set_mac_addr_client(struct ieee80211_hw *hw,
				  struct ieee80211_vif *vif, u8 *mac_addr)
{
	struct mwl_priv *priv = hw->priv;
	struct mwl_vif *mwl_vif;
	struct hostcmd_cmd_set_mac_addr *pcmd;

	mwl_vif = mwl_dev_get_vif(vif);

	pcmd = (struct hostcmd_cmd_set_mac_addr *)&priv->pcmd_buf[
			INTF_CMDHEADER_LEN(priv->if_ops.inttf_head_len)];

	mutex_lock(&priv->fwcmd_mutex);

	memset(pcmd, 0x00, sizeof(*pcmd));
	pcmd->cmd_hdr.cmd = cpu_to_le16(HOSTCMD_CMD_SET_MAC_ADDR);
	pcmd->cmd_hdr.len = cpu_to_le16(sizeof(*pcmd));
	pcmd->cmd_hdr.macid = mwl_vif->macid;
	pcmd->mac_type = cpu_to_le16(WL_MAC_TYPE_SECONDARY_CLIENT);
	ether_addr_copy(pcmd->mac_addr, mac_addr);

	if (mwl_fwcmd_exec_cmd(priv, HOSTCMD_CMD_SET_MAC_ADDR) ||
	    (pcmd->cmd_hdr.result != cpu_to_le16(HOSTCMD_RESULT_OK))) {
		mutex_unlock(&priv->fwcmd_mutex);
		return -EIO;
	}

	mutex_unlock(&priv->fwcmd_mutex);

	return 0;
}

int mwl_fwcmd_get_watchdog_bitmap(struct ieee80211_hw *hw, u8 *bitmap)
{
	struct mwl_priv *priv = hw->priv;
	struct hostcmd_cmd_get_watchdog_bitmap *pcmd;

	pcmd = (struct hostcmd_cmd_get_watchdog_bitmap *)&priv->pcmd_buf[
			INTF_CMDHEADER_LEN(priv->if_ops.inttf_head_len)];

	mutex_lock(&priv->fwcmd_mutex);

	memset(pcmd, 0x00, sizeof(*pcmd));
	pcmd->cmd_hdr.cmd = cpu_to_le16(HOSTCMD_CMD_GET_WATCHDOG_BITMAP);
	pcmd->cmd_hdr.len = cpu_to_le16(sizeof(*pcmd));

	if (mwl_fwcmd_exec_cmd(priv, HOSTCMD_CMD_GET_WATCHDOG_BITMAP)) {
		mutex_unlock(&priv->fwcmd_mutex);
		return -EIO;
	}

	*bitmap = pcmd->watchdog_bitmap;

	mutex_unlock(&priv->fwcmd_mutex);

	return 0;
}

int mwl_fwcmd_remove_mac_addr(struct ieee80211_hw *hw,
			      struct ieee80211_vif *vif, u8 *mac_addr)
{
	struct mwl_priv *priv = hw->priv;
	struct mwl_vif *mwl_vif;
	struct hostcmd_cmd_set_mac_addr *pcmd;

	mwl_vif = mwl_dev_get_vif(vif);

	pcmd = (struct hostcmd_cmd_set_mac_addr *)&priv->pcmd_buf[
			INTF_CMDHEADER_LEN(priv->if_ops.inttf_head_len)];

	mutex_lock(&priv->fwcmd_mutex);

	memset(pcmd, 0x00, sizeof(*pcmd));
	pcmd->cmd_hdr.cmd = cpu_to_le16(HOSTCMD_CMD_DEL_MAC_ADDR);
	pcmd->cmd_hdr.len = cpu_to_le16(sizeof(*pcmd));
	pcmd->cmd_hdr.macid = mwl_vif->macid;
	ether_addr_copy(pcmd->mac_addr, mac_addr);

	if (mwl_fwcmd_exec_cmd(priv, HOSTCMD_CMD_DEL_MAC_ADDR)) {
		mutex_unlock(&priv->fwcmd_mutex);
		return -EIO;
	}

	mutex_unlock(&priv->fwcmd_mutex);

	return 0;
}

int mwl_fwcmd_ibss_start(struct ieee80211_hw *hw,
            struct ieee80211_vif *vif, bool enable)
{
	struct mwl_priv *priv = hw->priv;
	struct mwl_vif *mwl_vif;
	struct hostcmd_cmd_bss_start *pcmd;
	mwl_vif = mwl_dev_get_vif(vif);
	if(enable)
	wiphy_debug(priv->hw->wiphy, "CMD IBSS Start Enable,Mac Id %d\n", mwl_vif->macid);
	else
	wiphy_debug(priv->hw->wiphy, "CMD IBSS Start Disable\n");


	if (enable && (priv->running_bsses & (1 << mwl_vif->macid)))
		return 0;

	if (!enable && !(priv->running_bsses & (1 << mwl_vif->macid)))
		return 0;

	pcmd = (struct hostcmd_cmd_bss_start *)&priv->pcmd_buf[
		INTF_CMDHEADER_LEN(priv->if_ops.inttf_head_len)];

	mutex_lock(&priv->fwcmd_mutex);

	memset(pcmd, 0x00, sizeof(*pcmd));
	pcmd->cmd_hdr.cmd = cpu_to_le16(HOSTCMD_CMD_IBSS_START);
	pcmd->cmd_hdr.len = cpu_to_le16(sizeof(*pcmd));
	pcmd->cmd_hdr.macid = mwl_vif->macid;

	if (enable) {
		pcmd->enable = cpu_to_le32(WL_ENABLE);
	} else {
		if (mwl_vif->macid == 0)
			pcmd->enable = cpu_to_le32(WL_DISABLE);
		else
			pcmd->enable = cpu_to_le32(WL_DISABLE_VMAC);
	}

	if (mwl_fwcmd_exec_cmd(priv, HOSTCMD_CMD_IBSS_START)) {
		mutex_unlock(&priv->fwcmd_mutex);
		wiphy_err(hw->wiphy, "failed execution\n");
		return -EIO;
	}

	if (enable)
		priv->running_bsses |= (1 << mwl_vif->macid);
	else
		priv->running_bsses &= ~(1 << mwl_vif->macid);

	mutex_unlock(&priv->fwcmd_mutex);

	return 0;
}

int mwl_fwcmd_bss_start(struct ieee80211_hw *hw,
			struct ieee80211_vif *vif, bool enable)
{
	struct mwl_priv *priv = hw->priv;
	struct mwl_vif *mwl_vif;
	struct hostcmd_cmd_bss_start *pcmd;

	mwl_vif = mwl_dev_get_vif(vif);

	if (enable && (priv->running_bsses & (1 << mwl_vif->macid)))
		return 0;

	if (!enable && !(priv->running_bsses & (1 << mwl_vif->macid)))
		return 0;

	pcmd = (struct hostcmd_cmd_bss_start *)&priv->pcmd_buf[
		INTF_CMDHEADER_LEN(priv->if_ops.inttf_head_len)];

	mutex_lock(&priv->fwcmd_mutex);

	memset(pcmd, 0x00, sizeof(*pcmd));
	pcmd->cmd_hdr.cmd = cpu_to_le16(HOSTCMD_CMD_BSS_START);
	pcmd->cmd_hdr.len = cpu_to_le16(sizeof(*pcmd));
	pcmd->cmd_hdr.macid = mwl_vif->macid;

	if (enable) {
		pcmd->enable = cpu_to_le32(WL_ENABLE);
	} else {
		if (mwl_vif->macid == 0)
			pcmd->enable = cpu_to_le32(WL_DISABLE);
		else
			pcmd->enable = cpu_to_le32(WL_DISABLE_VMAC);
	}

	if (mwl_fwcmd_exec_cmd(priv, HOSTCMD_CMD_BSS_START)) {
		mutex_unlock(&priv->fwcmd_mutex);
		return -EIO;
	}

	if (enable)
		priv->running_bsses |= (1 << mwl_vif->macid);
	else
		priv->running_bsses &= ~(1 << mwl_vif->macid);

	mutex_unlock(&priv->fwcmd_mutex);

	return 0;
}

int mwl_fwcmd_set_beacon(struct ieee80211_hw *hw,
			 struct ieee80211_vif *vif, u8 *beacon, int len)
{
	struct mwl_priv *priv = hw->priv;
	struct mwl_vif *mwl_vif;
	struct beacon_info *b_inf;
	int rc;

	mwl_vif = mwl_dev_get_vif(vif);
	b_inf = &mwl_vif->beacon_info;

	mwl_fwcmd_parse_beacon(priv, mwl_vif, beacon, len);

	if (!b_inf->valid)
		goto err;

	if (mwl_fwcmd_broadcast_ssid_enable(hw, mwl_vif, vif))
		goto err;

	if (mwl_fwcmd_set_ies(priv, mwl_vif))
		goto err;

	if (mwl_fwcmd_set_wsc_ie(hw, b_inf->ie_wsc_len, b_inf->ie_wsc_ptr))
		goto err;

	if (mwl_fwcmd_set_wfd_ie(hw, b_inf->ie_wfd_len, b_inf->ie_wfd_ptr))
		goto err;

	if (mwl_fwcmd_set_ap_beacon(priv, vif))
		goto err;

	if(vif->type == NL80211_IFTYPE_ADHOC) {
		if (mwl_fwcmd_ibss_start(hw, vif, true))
			goto err;
	} else {
		if (mwl_fwcmd_bss_start(hw, vif, true))
			goto err;
	}

	if (b_inf->cap_info & WLAN_CAPABILITY_SPECTRUM_MGMT)
		rc = mwl_fwcmd_set_spectrum_mgmt(priv, true);
	else
		rc = mwl_fwcmd_set_spectrum_mgmt(priv, false);
	if (rc)
		goto err;

	if (b_inf->power_constraint)
		rc = mwl_fwcmd_set_power_constraint(priv,
						    b_inf->power_constraint);
	if (rc)
		goto err;

	if (mwl_fwcmd_set_country_code(priv, mwl_vif, &vif->bss_conf))
		goto err;

	b_inf->valid = false;

	return 0;

err:

	b_inf->valid = false;

	return -EIO;
}

int mwl_fwcmd_set_new_stn_add(struct ieee80211_hw *hw,
			      struct ieee80211_vif *vif,
			      struct ieee80211_sta *sta)
{
	struct mwl_priv *priv = hw->priv;
	struct mwl_vif *mwl_vif;
	struct hostcmd_cmd_set_new_stn *pcmd;
	u32 legacy_rates = 0;

	mwl_vif = mwl_dev_get_vif(vif);

	pcmd = (struct hostcmd_cmd_set_new_stn *)&priv->pcmd_buf[
		INTF_CMDHEADER_LEN(priv->if_ops.inttf_head_len)];

	mutex_lock(&priv->fwcmd_mutex);

	memset(pcmd, 0x00, sizeof(*pcmd));
	pcmd->cmd_hdr.cmd = cpu_to_le16(HOSTCMD_CMD_SET_NEW_STN);
	pcmd->cmd_hdr.len = cpu_to_le16(sizeof(*pcmd));
	pcmd->cmd_hdr.macid = mwl_vif->macid;
	pcmd->if_type = cpu_to_le16(vif->type);

	pcmd->action = cpu_to_le16(HOSTCMD_ACT_STA_ACTION_ADD);
	if ((vif->type == NL80211_IFTYPE_STATION) ||
		(vif->type == NL80211_IFTYPE_P2P_CLIENT)) {
		pcmd->aid = cpu_to_le16(1);
		pcmd->stn_id = cpu_to_le16(1);
	} else {
		pcmd->aid = cpu_to_le16(sta->aid);
		pcmd->stn_id = cpu_to_le16(sta->aid);
	}
	ether_addr_copy(pcmd->mac_addr, sta->addr);

	if (hw->conf.chandef.chan->band == NL80211_BAND_2GHZ) {
		/*
		 * 2.4 legacy rate bit definitions
		 *   Driver:
		 *     Modulation    Bits    Rate(Mbps)
		 *         CCK       3 - 0   11, 5.5, 2, 1
		 *        OFDM      11 - 4   54, 48, 36, 24, 18, 12, 9, 6
		 *   Radio firmware:
		 *         CCK       4 - 0   22, 11, 5.5, 2, 1
		 *        OFDM      12 - 5   54, 48, 36, 24, 18, 12, 9, 6
		 */
		legacy_rates = sta->deflink.supp_rates[NL80211_BAND_2GHZ] & 0xF;

		/* shift OFDM rates by 1 */
		legacy_rates |= (sta->deflink.supp_rates[NL80211_BAND_2GHZ] & 0xFF0) << 1;
	} else {
		/*
		 * 5.0 legacy rate bit definitions
		 *   Driver:
		 *     Modulation    Bits    Rate(Mbps)
		 *        OFDM       7 - 0   54, 48, 36, 24, 18, 12, 9, 6
		 *   Radio firmware:
		 *        OFDM      12 - 5   54, 48, 36, 24, 18, 12, 9, 6
		 *
		 * Shift by 5
		 */
		legacy_rates = sta->deflink.supp_rates[NL80211_BAND_5GHZ] << 5;
	}
	pcmd->peer_info.legacy_rate_bitmap = cpu_to_le32(legacy_rates);

	if (sta->deflink.ht_cap.ht_supported) {
		int i;

		for (i = 0; i < 4; i++) {
			if (i < sta->deflink.rx_nss) {
				pcmd->peer_info.ht_rates[i] =
					sta->deflink.ht_cap.mcs.rx_mask[i];
			} else {
				pcmd->peer_info.ht_rates[i] = 0;
			}
		}
		pcmd->peer_info.ht_cap_info = cpu_to_le16(sta->deflink.ht_cap.cap);
		pcmd->peer_info.mac_ht_param_info =
			(sta->deflink.ht_cap.ampdu_factor & 3) |
			((sta->deflink.ht_cap.ampdu_density & 7) << 2);
	}

	if (sta->deflink.vht_cap.vht_supported) {
		u32 rx_mcs_map_mask = 0;

		rx_mcs_map_mask = ((0x0000FFFF) >> (sta->deflink.rx_nss * 2))
			<< (sta->deflink.rx_nss * 2);
		pcmd->peer_info.vht_max_rx_mcs =
			cpu_to_le32((*((u32 *)
			&sta->deflink.vht_cap.vht_mcs.rx_mcs_map)) | rx_mcs_map_mask);
		pcmd->peer_info.vht_cap = cpu_to_le32(sta->deflink.vht_cap.cap);
		pcmd->peer_info.vht_rx_channel_width = sta->deflink.bandwidth;
	}

	pcmd->is_qos_sta = sta->wme;
	pcmd->qos_info = ((sta->uapsd_queues << 4) | (sta->max_sp << 1));

	if (mwl_fwcmd_exec_cmd(priv, HOSTCMD_CMD_SET_NEW_STN)) {
		mutex_unlock(&priv->fwcmd_mutex);
		return -EIO;
	}
	pcmd->cmd_hdr.cmd = cpu_to_le16(HOSTCMD_CMD_SET_NEW_STN);
	pcmd->cmd_hdr.len = cpu_to_le16(sizeof(*pcmd));
	pcmd->cmd_hdr.macid = mwl_vif->macid;

	if ((vif->type == NL80211_IFTYPE_STATION) ||
		(vif->type == NL80211_IFTYPE_P2P_CLIENT)) {
		ether_addr_copy(pcmd->mac_addr, mwl_vif->sta_mac);
		pcmd->aid = cpu_to_le16(2);
		pcmd->stn_id = cpu_to_le16(2);
		if (mwl_fwcmd_exec_cmd(priv, HOSTCMD_CMD_SET_NEW_STN)) {
			mutex_unlock(&priv->fwcmd_mutex);
			return -EIO;
		}
	}

	mutex_unlock(&priv->fwcmd_mutex);

	return 0;
}

int mwl_fwcmd_set_new_stn_add_self(struct ieee80211_hw *hw,
				   struct ieee80211_vif *vif)
{
	struct mwl_priv *priv = hw->priv;
	struct mwl_vif *mwl_vif;
	struct hostcmd_cmd_set_new_stn *pcmd;

	mwl_vif = mwl_dev_get_vif(vif);

	pcmd = (struct hostcmd_cmd_set_new_stn *)&priv->pcmd_buf[
			INTF_CMDHEADER_LEN(priv->if_ops.inttf_head_len)];

	mutex_lock(&priv->fwcmd_mutex);

	memset(pcmd, 0x00, sizeof(*pcmd));
	pcmd->cmd_hdr.cmd = cpu_to_le16(HOSTCMD_CMD_SET_NEW_STN);
	pcmd->cmd_hdr.len = cpu_to_le16(sizeof(*pcmd));
	pcmd->cmd_hdr.macid = mwl_vif->macid;
	pcmd->if_type = cpu_to_le16(vif->type);

	pcmd->action = cpu_to_le16(HOSTCMD_ACT_STA_ACTION_ADD);
	ether_addr_copy(pcmd->mac_addr, vif->addr);

	if (mwl_fwcmd_exec_cmd(priv, HOSTCMD_CMD_SET_NEW_STN) ||
	    (pcmd->cmd_hdr.result != cpu_to_le16(HOSTCMD_RESULT_OK))) {
		mutex_unlock(&priv->fwcmd_mutex);
		return -EIO;
	}

	mutex_unlock(&priv->fwcmd_mutex);

	return 0;
}

int mwl_fwcmd_set_new_stn_del(struct ieee80211_hw *hw,
			      struct ieee80211_vif *vif, u8 *addr)
{
	struct mwl_priv *priv = hw->priv;
	struct mwl_vif *mwl_vif;
	struct hostcmd_cmd_set_new_stn *pcmd;

	mwl_vif = mwl_dev_get_vif(vif);

	pcmd = (struct hostcmd_cmd_set_new_stn *)&priv->pcmd_buf[
			INTF_CMDHEADER_LEN(priv->if_ops.inttf_head_len)];

	mutex_lock(&priv->fwcmd_mutex);

	memset(pcmd, 0x00, sizeof(*pcmd));
	pcmd->cmd_hdr.cmd = cpu_to_le16(HOSTCMD_CMD_SET_NEW_STN);
	pcmd->cmd_hdr.len = cpu_to_le16(sizeof(*pcmd));
	pcmd->cmd_hdr.macid = mwl_vif->macid;

	pcmd->action = cpu_to_le16(HOSTCMD_ACT_STA_ACTION_REMOVE);
	ether_addr_copy(pcmd->mac_addr, addr);

	if (mwl_fwcmd_exec_cmd(priv, HOSTCMD_CMD_SET_NEW_STN)) {
		mutex_unlock(&priv->fwcmd_mutex);
		return -EIO;
	}

	if ((vif->type == NL80211_IFTYPE_STATION) ||
		(vif->type == NL80211_IFTYPE_P2P_CLIENT)) {
		ether_addr_copy(pcmd->mac_addr, mwl_vif->sta_mac);
	pcmd->cmd_hdr.cmd = cpu_to_le16(HOSTCMD_CMD_SET_NEW_STN);
	pcmd->cmd_hdr.len = cpu_to_le16(sizeof(*pcmd));

		if (mwl_fwcmd_exec_cmd(priv, HOSTCMD_CMD_SET_NEW_STN)) {
			mutex_unlock(&priv->fwcmd_mutex);
			return -EIO;
		}
	}

	mutex_unlock(&priv->fwcmd_mutex);

	return 0;
}

int mwl_fwcmd_set_apmode(struct ieee80211_hw *hw, u8 apmode)
{
	struct mwl_priv *priv = hw->priv;
	struct hostcmd_cmd_set_apmode *pcmd;

	pcmd = (struct hostcmd_cmd_set_apmode *)&priv->pcmd_buf[
			INTF_CMDHEADER_LEN(priv->if_ops.inttf_head_len)];

	mutex_lock(&priv->fwcmd_mutex);

	memset(pcmd, 0x00, sizeof(*pcmd));
	pcmd->cmd_hdr.cmd = cpu_to_le16(HOSTCMD_CMD_SET_APMODE);
	pcmd->cmd_hdr.len = cpu_to_le16(sizeof(*pcmd));
	pcmd->apmode = apmode;

	if (mwl_fwcmd_exec_cmd(priv, HOSTCMD_CMD_SET_APMODE)) {
		mutex_unlock(&priv->fwcmd_mutex);
		return -EIO;
	}

	mutex_unlock(&priv->fwcmd_mutex);

	return 0;
}

int mwl_fwcmd_set_switch_channel(struct mwl_priv *priv,
				 struct ieee80211_channel_switch *ch_switch)
{
	struct hostcmd_cmd_set_switch_channel *pcmd;
	struct cfg80211_chan_def *chandef = &ch_switch->chandef;
	struct ieee80211_channel *channel = chandef->chan;
	u32 chnl_flags, freq_band, chnl_width, act_primary, sec_chnl_offset;

	if (priv->csa_active)
		return 0;

	if (channel->band == NL80211_BAND_2GHZ)
		freq_band = FREQ_BAND_2DOT4GHZ;
	else if (channel->band == NL80211_BAND_5GHZ)
		freq_band = FREQ_BAND_5GHZ;
	else
		return -EINVAL;

	switch (chandef->width) {
	case NL80211_CHAN_WIDTH_20_NOHT:
	case NL80211_CHAN_WIDTH_20:
		chnl_width = CH_20_MHZ_WIDTH;
		act_primary = ACT_PRIMARY_CHAN_0;
		sec_chnl_offset = IEEE80211_HT_PARAM_CHA_SEC_NONE;
		break;
	case NL80211_CHAN_WIDTH_40:
		chnl_width = CH_40_MHZ_WIDTH;
		if (chandef->center_freq1 > channel->center_freq) {
			act_primary = ACT_PRIMARY_CHAN_0;
			sec_chnl_offset = IEEE80211_HT_PARAM_CHA_SEC_ABOVE;
		} else {
			act_primary = ACT_PRIMARY_CHAN_1;
			sec_chnl_offset = IEEE80211_HT_PARAM_CHA_SEC_BELOW;
		}
		break;
	case NL80211_CHAN_WIDTH_80:
		chnl_width = CH_80_MHZ_WIDTH;
		act_primary =
			mwl_fwcmd_get_80m_pri_chnl(channel->hw_value);
		if ((act_primary == ACT_PRIMARY_CHAN_0) ||
		    (act_primary == ACT_PRIMARY_CHAN_2))
			sec_chnl_offset = IEEE80211_HT_PARAM_CHA_SEC_ABOVE;
		else
			sec_chnl_offset = IEEE80211_HT_PARAM_CHA_SEC_BELOW;
		break;
	default:
		return -EINVAL;
	}

	chnl_flags = (freq_band & FREQ_BAND_MASK) |
		((chnl_width << CHNL_WIDTH_SHIFT) & CHNL_WIDTH_MASK) |
		((act_primary << ACT_PRIMARY_SHIFT) & ACT_PRIMARY_MASK);

	pcmd = (struct hostcmd_cmd_set_switch_channel *)&priv->pcmd_buf[
			INTF_CMDHEADER_LEN(priv->if_ops.inttf_head_len)];

	mutex_lock(&priv->fwcmd_mutex);

	memset(pcmd, 0x00, sizeof(*pcmd));
	pcmd->cmd_hdr.cmd = cpu_to_le16(HOSTCMD_CMD_SET_SWITCH_CHANNEL);
	pcmd->cmd_hdr.len = cpu_to_le16(sizeof(*pcmd));
	pcmd->next_11h_chnl = cpu_to_le32(channel->hw_value);
	pcmd->mode = cpu_to_le32(ch_switch->block_tx);
	pcmd->init_count = cpu_to_le32(ch_switch->count + 1);
	pcmd->chnl_flags = cpu_to_le32(chnl_flags);
	pcmd->next_ht_extchnl_offset = cpu_to_le32(sec_chnl_offset);

	if (mwl_fwcmd_exec_cmd(priv, HOSTCMD_CMD_SET_SWITCH_CHANNEL)) {
		mutex_unlock(&priv->fwcmd_mutex);
		return -EIO;
	}

	priv->csa_active = true;

	mutex_unlock(&priv->fwcmd_mutex);

	return 0;
}

int mwl_fwcmd_update_encryption_enable(struct ieee80211_hw *hw,
				       struct ieee80211_vif *vif,
				       u8 *addr, u8 encr_type)
{
	struct mwl_priv *priv = hw->priv;
	struct mwl_vif *mwl_vif;
	struct hostcmd_cmd_update_encryption *pcmd;

	mwl_vif = mwl_dev_get_vif(vif);

	pcmd = (struct hostcmd_cmd_update_encryption *)&priv->pcmd_buf[
			INTF_CMDHEADER_LEN(priv->if_ops.inttf_head_len)];

	mutex_lock(&priv->fwcmd_mutex);

	memset(pcmd, 0x00, sizeof(*pcmd));
	pcmd->cmd_hdr.cmd = cpu_to_le16(HOSTCMD_CMD_UPDATE_ENCRYPTION);
	pcmd->cmd_hdr.len = cpu_to_le16(sizeof(*pcmd));
	pcmd->cmd_hdr.macid = mwl_vif->macid;

	pcmd->action_type = cpu_to_le32(ENCR_ACTION_ENABLE_HW_ENCR);
	ether_addr_copy(pcmd->mac_addr, addr);
	pcmd->action_data[0] = encr_type;

	if (mwl_fwcmd_exec_cmd(priv, HOSTCMD_CMD_UPDATE_ENCRYPTION)) {
		mutex_unlock(&priv->fwcmd_mutex);
		return -EIO;
	}

	if ((vif->type == NL80211_IFTYPE_STATION) ||
		(vif->type == NL80211_IFTYPE_P2P_CLIENT)) {
		if (ether_addr_equal(mwl_vif->bssid, addr))
			ether_addr_copy(pcmd->mac_addr, mwl_vif->sta_mac);
		else
			ether_addr_copy(pcmd->mac_addr, mwl_vif->bssid);

		pcmd->cmd_hdr.cmd = cpu_to_le16(HOSTCMD_CMD_UPDATE_ENCRYPTION);
		pcmd->cmd_hdr.len = cpu_to_le16(sizeof(*pcmd));
		pcmd->cmd_hdr.macid = mwl_vif->macid;

		if (mwl_fwcmd_exec_cmd(priv, HOSTCMD_CMD_UPDATE_ENCRYPTION)) {
			mutex_unlock(&priv->fwcmd_mutex);
			return -EIO;
		}
	}

	mutex_unlock(&priv->fwcmd_mutex);

	return 0;
}

int mwl_fwcmd_encryption_set_key(struct ieee80211_hw *hw,
				 struct ieee80211_vif *vif, u8 *addr,
				 struct ieee80211_key_conf *key)
{
	struct mwl_priv *priv = hw->priv;
	struct mwl_vif *mwl_vif;
	struct hostcmd_cmd_set_key *pcmd;
	int rc;
	int keymlen;
	u32 action;
	u8 idx;

	mwl_vif = mwl_dev_get_vif(vif);

	pcmd = (struct hostcmd_cmd_set_key *)&priv->pcmd_buf[
			INTF_CMDHEADER_LEN(priv->if_ops.inttf_head_len)];

	mutex_lock(&priv->fwcmd_mutex);

	memset(pcmd, 0x00, sizeof(*pcmd));
	pcmd->cmd_hdr.cmd = cpu_to_le16(HOSTCMD_CMD_UPDATE_ENCRYPTION);
	pcmd->cmd_hdr.len = cpu_to_le16(sizeof(*pcmd));
	pcmd->cmd_hdr.macid = mwl_vif->macid;

	rc = mwl_fwcmd_encryption_set_cmd_info(pcmd, vif, addr, key);
	if (rc) {
		mutex_unlock(&priv->fwcmd_mutex);
		if (rc < 0)
			wiphy_err(hw->wiphy, "encryption not supported\n");
		return rc;
	}

	idx = key->keyidx;

	if (key->flags & IEEE80211_KEY_FLAG_PAIRWISE)
		action = ENCR_ACTION_TYPE_SET_KEY;
	else
		action = ENCR_ACTION_TYPE_SET_GROUP_KEY;

/* TODO: This code is missing in git ToT now - security mayn't work */
#if 0
{
		action = ENCR_ACTION_TYPE_SET_GROUP_KEY;

		/* if (vif->type == NL80211_IFTYPE_MESH_POINT &&
		    !ether_addr_equal(mwl_vif->bssid, addr)) */
			pcmd->key_param.key_info |=
				cpu_to_le32(ENCR_KEY_FLAG_RXGROUPKEY);
}
#endif

	switch (key->cipher) {
	case WLAN_CIPHER_SUITE_WEP40:
	case WLAN_CIPHER_SUITE_WEP104:
		if (!mwl_vif->wep_key_conf[idx].enabled) {
			memcpy(mwl_vif->wep_key_conf[idx].key, key,
			       sizeof(*key) + key->keylen);
			mwl_vif->wep_key_conf[idx].enabled = 1;
		}

		if (vif->type == NL80211_IFTYPE_STATION) {
			ether_addr_copy(mwl_vif->bssid, vif->bss_conf.bssid);
		} else if(vif->type == NL80211_IFTYPE_ADHOC) {
			if(!ether_addr_equal(vif->addr, addr)) {
				/* To handle a race condition when mac80211 doesnt populate
				 * keys for the newly added STA. Requesting FW to populate
				 * Keys for all the IBSS STA added till now */
				pcmd->key_param.cfg_flags = cpu_to_le32(0x1);
			}
		}
		keymlen = key->keylen;
		action = ENCR_ACTION_TYPE_SET_KEY;
		break;
	case WLAN_CIPHER_SUITE_TKIP:
		keymlen = MAX_ENCR_KEY_LENGTH + 2 * MIC_KEY_LENGTH;
		break;
	case WLAN_CIPHER_SUITE_CCMP:
		keymlen = key->keylen;
		break;
	default:
		mutex_unlock(&priv->fwcmd_mutex);
		wiphy_err(hw->wiphy, "encryption not support\n");
		return -ENOTSUPP;
	}

	memcpy((void *)&pcmd->key_param.key, key->key, keymlen);
	pcmd->action_type = cpu_to_le32(action);

	if (mwl_fwcmd_exec_cmd(priv, HOSTCMD_CMD_UPDATE_ENCRYPTION)) {
		mutex_unlock(&priv->fwcmd_mutex);
		return -EIO;
	}

	if ((vif->type == NL80211_IFTYPE_STATION) ||
		(vif->type == NL80211_IFTYPE_P2P_CLIENT)) {
		if (ether_addr_equal(mwl_vif->bssid, addr))
			ether_addr_copy(pcmd->key_param.mac_addr,
					mwl_vif->sta_mac);
		else
			ether_addr_copy(pcmd->key_param.mac_addr,
					mwl_vif->bssid);

		pcmd->cmd_hdr.cmd = cpu_to_le16(HOSTCMD_CMD_UPDATE_ENCRYPTION);
		pcmd->cmd_hdr.len = cpu_to_le16(sizeof(*pcmd));
		pcmd->cmd_hdr.macid = mwl_vif->macid;

		if (mwl_fwcmd_exec_cmd(priv, HOSTCMD_CMD_UPDATE_ENCRYPTION)) {
			mutex_unlock(&priv->fwcmd_mutex);
			return -EIO;
		}
	}

	mutex_unlock(&priv->fwcmd_mutex);

	return 0;
}

int mwl_fwcmd_encryption_remove_key(struct ieee80211_hw *hw,
				    struct ieee80211_vif *vif, u8 *addr,
				    struct ieee80211_key_conf *key)
{
	struct mwl_priv *priv = hw->priv;
	struct mwl_vif *mwl_vif;
	struct hostcmd_cmd_set_key *pcmd;
	int rc;

	mwl_vif = mwl_dev_get_vif(vif);

	pcmd = (struct hostcmd_cmd_set_key *)&priv->pcmd_buf[
			INTF_CMDHEADER_LEN(priv->if_ops.inttf_head_len)];

	mutex_lock(&priv->fwcmd_mutex);

	memset(pcmd, 0x00, sizeof(*pcmd));
	pcmd->cmd_hdr.cmd = cpu_to_le16(HOSTCMD_CMD_UPDATE_ENCRYPTION);
	pcmd->cmd_hdr.len = cpu_to_le16(sizeof(*pcmd));
	pcmd->cmd_hdr.macid = mwl_vif->macid;

	rc = mwl_fwcmd_encryption_set_cmd_info(pcmd, vif, addr, key);
	if (rc) {
		if (rc < 0)
			wiphy_err(hw->wiphy, "encryption not supported\n");
		goto out;
	}

	pcmd->action_type = cpu_to_le32(ENCR_ACTION_TYPE_REMOVE_KEY);

	rc = mwl_fwcmd_exec_cmd(priv, HOSTCMD_CMD_UPDATE_ENCRYPTION);

out:
	mutex_unlock(&priv->fwcmd_mutex);
	return rc;
}

int mwl_fwcmd_check_ba(struct ieee80211_hw *hw,
		       struct mwl_ampdu_stream *stream,
		       struct ieee80211_vif *vif)
{
	struct mwl_priv *priv = hw->priv;
	struct mwl_vif *mwl_vif;
	struct hostcmd_cmd_bastream *pcmd;
	u32 ba_flags, ba_type, ba_direction;

	mwl_vif = mwl_dev_get_vif(vif);

	pcmd = (struct hostcmd_cmd_bastream *)&priv->pcmd_buf[
			INTF_CMDHEADER_LEN(priv->if_ops.inttf_head_len)];

	mutex_lock(&priv->fwcmd_mutex);

	memset(pcmd, 0x00, sizeof(*pcmd));
	pcmd->cmd_hdr.cmd = cpu_to_le16(HOSTCMD_CMD_BASTREAM);
	pcmd->cmd_hdr.len = cpu_to_le16(sizeof(*pcmd));
	pcmd->cmd_hdr.macid = mwl_vif->macid;
	pcmd->cmd_hdr.result = cpu_to_le16(0xffff);

	pcmd->action_type = cpu_to_le32(BA_CHECK_STREAM);
	ether_addr_copy(&pcmd->ba_info.create_params.peer_mac_addr[0],
			stream->sta->addr);
	pcmd->ba_info.create_params.tid = stream->tid;
	ba_type = BASTREAM_FLAG_IMMEDIATE_TYPE;
	ba_direction = BASTREAM_FLAG_DIRECTION_UPSTREAM;
	ba_flags = (ba_type & BA_TYPE_MASK) |
		((ba_direction << BA_DIRECTION_SHIFT) & BA_DIRECTION_MASK);
	pcmd->ba_info.create_params.flags = cpu_to_le32(ba_flags);
	pcmd->ba_info.create_params.queue_id = stream->idx;

	if (mwl_fwcmd_exec_cmd(priv, HOSTCMD_CMD_BASTREAM)) {
		mutex_unlock(&priv->fwcmd_mutex);
		return -EIO;
	}

	if (pcmd->cmd_hdr.result != 0) {
		mutex_unlock(&priv->fwcmd_mutex);
		return -EINVAL;
	}

	mutex_unlock(&priv->fwcmd_mutex);

	return 0;
}

int mwl_fwcmd_create_ba(struct ieee80211_hw *hw,
			struct mwl_ampdu_stream *stream,
			u8 buf_size, struct ieee80211_vif *vif)
{
	struct mwl_priv *priv = hw->priv;
	struct mwl_vif *mwl_vif;
	struct hostcmd_cmd_bastream *pcmd;
	u32 ba_flags, ba_type, ba_direction;

	mwl_vif = mwl_dev_get_vif(vif);

	pcmd = (struct hostcmd_cmd_bastream *)&priv->pcmd_buf[
			INTF_CMDHEADER_LEN(priv->if_ops.inttf_head_len)];

	mutex_lock(&priv->fwcmd_mutex);

	memset(pcmd, 0x00, sizeof(*pcmd));
	pcmd->cmd_hdr.cmd = cpu_to_le16(HOSTCMD_CMD_BASTREAM);
	pcmd->cmd_hdr.len = cpu_to_le16(sizeof(*pcmd));
	pcmd->cmd_hdr.macid = mwl_vif->macid;
	pcmd->cmd_hdr.result = cpu_to_le16(0xffff);

	pcmd->action_type = cpu_to_le32(BA_CREATE_STREAM);
	pcmd->ba_info.create_params.bar_thrs = cpu_to_le32(buf_size);
	pcmd->ba_info.create_params.window_size = cpu_to_le32(buf_size);
	ether_addr_copy(&pcmd->ba_info.create_params.peer_mac_addr[0],
			stream->sta->addr);
	pcmd->ba_info.create_params.tid = stream->tid;
	ba_type = BASTREAM_FLAG_IMMEDIATE_TYPE;
	ba_direction = BASTREAM_FLAG_DIRECTION_UPSTREAM;
	ba_flags = (ba_type & BA_TYPE_MASK) |
		((ba_direction << BA_DIRECTION_SHIFT) & BA_DIRECTION_MASK);
	pcmd->ba_info.create_params.flags = cpu_to_le32(ba_flags);
	pcmd->ba_info.create_params.queue_id = stream->idx;
	pcmd->ba_info.create_params.param_info =
		(stream->sta->deflink.ht_cap.ampdu_factor &
		 IEEE80211_HT_AMPDU_PARM_FACTOR) |
		((stream->sta->deflink.ht_cap.ampdu_density << 2) &
		 IEEE80211_HT_AMPDU_PARM_DENSITY);

#if 0
	pcmd->ba_info.create_params.reset_seq_no = 1;
	pcmd->ba_info.create_params.current_seq = cpu_to_le16(0);
#endif

	if (mwl_fwcmd_exec_cmd(priv, HOSTCMD_CMD_BASTREAM)) {
		mutex_unlock(&priv->fwcmd_mutex);
		return -EIO;
	}

	if (pcmd->cmd_hdr.result != 0) {
		mutex_unlock(&priv->fwcmd_mutex);
		wiphy_err(hw->wiphy, "create ba result error %d\n",
			  le16_to_cpu(pcmd->cmd_hdr.result));
		return -EINVAL;
	}

	mutex_unlock(&priv->fwcmd_mutex);

	return 0;
}

int mwl_fwcmd_destroy_ba(struct ieee80211_hw *hw,
			 u8 idx)
{
	struct mwl_priv *priv = hw->priv;
	struct hostcmd_cmd_bastream *pcmd;
	u32 ba_flags, ba_type, ba_direction;

	pcmd = (struct hostcmd_cmd_bastream *)&priv->pcmd_buf[
			INTF_CMDHEADER_LEN(priv->if_ops.inttf_head_len)];

	mutex_lock(&priv->fwcmd_mutex);

	memset(pcmd, 0x00, sizeof(*pcmd));
	pcmd->cmd_hdr.cmd = cpu_to_le16(HOSTCMD_CMD_BASTREAM);
	pcmd->cmd_hdr.len = cpu_to_le16(sizeof(*pcmd));

	pcmd->action_type = cpu_to_le32(BA_DESTROY_STREAM);
	ba_type = BASTREAM_FLAG_IMMEDIATE_TYPE;
	ba_direction = BASTREAM_FLAG_DIRECTION_UPSTREAM;
	ba_flags = (ba_type & BA_TYPE_MASK) |
		((ba_direction << BA_DIRECTION_SHIFT) & BA_DIRECTION_MASK);
	pcmd->ba_info.destroy_params.flags = cpu_to_le32(ba_flags);
	pcmd->ba_info.destroy_params.fw_ba_context.context = cpu_to_le32(idx);

	if (mwl_fwcmd_exec_cmd(priv, HOSTCMD_CMD_BASTREAM)) {
		mutex_unlock(&priv->fwcmd_mutex);
		return -EIO;
	}

	mutex_unlock(&priv->fwcmd_mutex);

	return 0;
}

/* caller must hold priv->stream_lock when calling the stream functions */
struct mwl_ampdu_stream *mwl_fwcmd_add_stream(struct ieee80211_hw *hw,
					      struct ieee80211_sta *sta,
					      u8 tid)
{
	struct mwl_priv *priv = hw->priv;
	struct mwl_ampdu_stream *stream;
	int i;

	for (i = 0; i < SYSADPT_TX_AMPDU_QUEUES; i++) {
		stream = &priv->ampdu[i];

		if (stream->state == AMPDU_NO_STREAM) {
			stream->sta = sta;
			stream->state = AMPDU_STREAM_NEW;
			stream->tid = tid;
			stream->idx = i;
			return stream;
		}
	}

	return NULL;
}

void mwl_fwcmd_del_sta_streams(struct ieee80211_hw *hw,
			       struct ieee80211_sta *sta)
{
	struct mwl_priv *priv = hw->priv;
	struct mwl_ampdu_stream *stream;
	int i;

	spin_lock_bh(&priv->stream_lock);
	for (i = 0; i < SYSADPT_TX_AMPDU_QUEUES; i++) {
		stream = &priv->ampdu[i];

		if (stream->sta == sta) {
			mwl_fwcmd_remove_stream(hw, stream);
			spin_unlock_bh(&priv->stream_lock);
			mwl_fwcmd_destroy_ba(hw, stream->idx);
			spin_lock_bh(&priv->stream_lock);
		}
	}
	spin_unlock_bh(&priv->stream_lock);
}

int mwl_fwcmd_start_stream(struct ieee80211_hw *hw,
			   struct mwl_ampdu_stream *stream)
{
	/* if the stream has already been started, don't start it again */
	if (stream->state != AMPDU_STREAM_NEW)
		return 0;

	return ieee80211_start_tx_ba_session(stream->sta, stream->tid, 0);
}

void mwl_fwcmd_remove_stream(struct ieee80211_hw *hw,
			     struct mwl_ampdu_stream *stream)
{
	memset(stream, 0, sizeof(*stream));
}

struct mwl_ampdu_stream *mwl_fwcmd_lookup_stream(struct ieee80211_hw *hw,
						 u8 *addr, u8 tid)
{
	struct mwl_priv *priv = hw->priv;
	struct mwl_ampdu_stream *stream;
	int i;

	for (i = 0; i < SYSADPT_TX_AMPDU_QUEUES; i++) {
		stream = &priv->ampdu[i];

		if (stream->state == AMPDU_NO_STREAM)
			continue;

		if (ether_addr_equal(stream->sta->addr, addr) &&
		    stream->tid == tid)
			return stream;
	}

	return NULL;
}

bool mwl_fwcmd_ampdu_allowed(struct ieee80211_sta *sta, u8 tid)
{
	struct mwl_sta *sta_info;
	struct mwl_tx_info *tx_stats;

	if (WARN_ON(tid >= SYSADPT_MAX_TID))
		return false;

	sta_info = mwl_dev_get_sta(sta);

	tx_stats = &sta_info->tx_stats[tid];

	return (sta_info->is_ampdu_allowed &&
		tx_stats->pkts > SYSADPT_AMPDU_PACKET_THRESHOLD);
}

int mwl_fwcmd_set_optimization_level(struct ieee80211_hw *hw, u8 opt_level)
{
	struct mwl_priv *priv = hw->priv;
	struct hostcmd_cmd_set_optimization_level *pcmd;

	pcmd = (struct hostcmd_cmd_set_optimization_level *)&priv->pcmd_buf[
			INTF_CMDHEADER_LEN(priv->if_ops.inttf_head_len)];

	mutex_lock(&priv->fwcmd_mutex);

	memset(pcmd, 0x00, sizeof(*pcmd));
	pcmd->cmd_hdr.cmd = cpu_to_le16(HOSTCMD_CMD_SET_OPTIMIZATION_LEVEL);
	pcmd->cmd_hdr.len = cpu_to_le16(sizeof(*pcmd));
	pcmd->opt_level = opt_level;

	wiphy_info(hw->wiphy, "WMM Turbo=%d\n", opt_level);


	if (mwl_fwcmd_exec_cmd(priv, HOSTCMD_CMD_SET_OPTIMIZATION_LEVEL)) {
		mutex_unlock(&priv->fwcmd_mutex);
		return -EIO;
	}

	mutex_unlock(&priv->fwcmd_mutex);

	return 0;
}

int mwl_fwcmd_set_mimops_ht(struct ieee80211_hw *hw, u8 *addr, u8 smps_ctrl)
{
	struct mwl_priv *priv = hw->priv;
	struct hostcmd_cmd_set_mimops_ht *pcmd;

	pcmd = (struct hostcmd_cmd_set_mimops_ht *)&priv->pcmd_buf[
			INTF_CMDHEADER_LEN(priv->if_ops.inttf_head_len)];

	mutex_lock(&priv->fwcmd_mutex);

	memset(pcmd, 0x00, sizeof(*pcmd));
	pcmd->cmd_hdr.cmd = cpu_to_le16(HOSTCMD_CMD_SET_MIMOPSHT);
	pcmd->cmd_hdr.len = cpu_to_le16(sizeof(*pcmd));
	ether_addr_copy(pcmd->addr, addr);

	pcmd->enbl = (smps_ctrl & 0x1);
	pcmd->mode = (smps_ctrl >> 1);

	if (mwl_fwcmd_exec_cmd(priv, HOSTCMD_CMD_SET_MIMOPSHT)) {
		mutex_unlock(&priv->fwcmd_mutex);
		return -EIO;
	}

	mutex_unlock(&priv->fwcmd_mutex);

	return 0;
}

int mwl_fwcmd_set_wfd_ie(struct ieee80211_hw *hw, u8 len, u8 *data)
{
	struct mwl_priv *priv = hw->priv;
	struct hostcmd_cmd_set_wfd_ie *pcmd;

	if (len > sizeof(pcmd->data)) {
		wiphy_err(priv->hw->wiphy,
		          "WFD IE is longer than radio supports");
		return -EIO;
	}

	pcmd = (struct hostcmd_cmd_set_wfd_ie *)&priv->pcmd_buf[
			INTF_CMDHEADER_LEN(priv->if_ops.inttf_head_len)];

	mutex_lock(&priv->fwcmd_mutex);
	memset(pcmd, 0x00, sizeof(*pcmd));
	pcmd->cmd_hdr.cmd = cpu_to_le16(HOSTCMD_CMD_SET_WFD_IE);
	pcmd->cmd_hdr.len = cpu_to_le16(sizeof(*pcmd));
	pcmd->len = cpu_to_le16(len);
	memcpy(pcmd->data, data, len);

	if (mwl_fwcmd_exec_cmd(priv, HOSTCMD_CMD_SET_WFD_IE)) {
		mutex_unlock(&priv->fwcmd_mutex);
		return -EIO;
	}

	pcmd->ie_type = cpu_to_le16(WFD_IE_SET_PROBE_RESPONSE);
	pcmd->cmd_hdr.cmd = cpu_to_le16(HOSTCMD_CMD_SET_WFD_IE);
	pcmd->cmd_hdr.len = cpu_to_le16(sizeof(*pcmd));
	pcmd->len = cpu_to_le16(len);

	if (mwl_fwcmd_exec_cmd(priv, HOSTCMD_CMD_SET_WFD_IE)) {
		mutex_unlock(&priv->fwcmd_mutex);
		return -EIO;
	}

	mutex_unlock(&priv->fwcmd_mutex);

	return 0;
}

int mwl_fwcmd_set_wsc_ie(struct ieee80211_hw *hw, u8 len, u8 *data)
{
	struct mwl_priv *priv = hw->priv;
	struct hostcmd_cmd_set_wsc_ie *pcmd;

	if (len > sizeof(pcmd->data)) {
		wiphy_err(priv->hw->wiphy,
		          "WSC IE is longer than radio supports");
		return -EIO;
	}

	pcmd = (struct hostcmd_cmd_set_wsc_ie *)&priv->pcmd_buf[
			INTF_CMDHEADER_LEN(priv->if_ops.inttf_head_len)];

	mutex_lock(&priv->fwcmd_mutex);

	memset(pcmd, 0x00, sizeof(*pcmd));
	pcmd->cmd_hdr.cmd = cpu_to_le16(HOSTCMD_CMD_SET_WSC_IE);
	pcmd->cmd_hdr.len = cpu_to_le16(sizeof(*pcmd));
	pcmd->len = cpu_to_le16(len);
	memcpy(pcmd->data, data, len);

	if (mwl_fwcmd_exec_cmd(priv, HOSTCMD_CMD_SET_WSC_IE)) {
		mutex_unlock(&priv->fwcmd_mutex);
		return -EIO;
	}

	pcmd->ie_type = cpu_to_le16(WSC_IE_SET_PROBE_RESPONSE);
	pcmd->cmd_hdr.cmd = cpu_to_le16(HOSTCMD_CMD_SET_WSC_IE);
	pcmd->cmd_hdr.len = cpu_to_le16(sizeof(*pcmd));
	pcmd->len = cpu_to_le16(len);

	if (mwl_fwcmd_exec_cmd(priv, HOSTCMD_CMD_SET_WSC_IE)) {
		mutex_unlock(&priv->fwcmd_mutex);
		return -EIO;
	}

	mutex_unlock(&priv->fwcmd_mutex);

	return 0;
}

int mwl_fwcmd_set_dwds_stamode(struct ieee80211_hw *hw, bool enable)
{
	struct mwl_priv *priv = hw->priv;
	struct hostcmd_cmd_dwds_enable *pcmd;

	pcmd = (struct hostcmd_cmd_dwds_enable *)&priv->pcmd_buf[
			INTF_CMDHEADER_LEN(priv->if_ops.inttf_head_len)];

	mutex_lock(&priv->fwcmd_mutex);

	memset(pcmd, 0x00, sizeof(*pcmd));
	pcmd->cmd_hdr.cmd = cpu_to_le16(HOSTCMD_CMD_DWDS_ENABLE);
	pcmd->cmd_hdr.len = cpu_to_le16(sizeof(*pcmd));
	pcmd->enable = cpu_to_le32(enable);

	if (mwl_fwcmd_exec_cmd(priv, HOSTCMD_CMD_DWDS_ENABLE)) {
		mutex_unlock(&priv->fwcmd_mutex);
		return -EIO;
	}

	mutex_unlock(&priv->fwcmd_mutex);

	return 0;
}

int mwl_fwcmd_set_fw_flush_timer(struct ieee80211_hw *hw, u32 value)
{
	struct mwl_priv *priv = hw->priv;
	struct hostcmd_cmd_fw_flush_timer *pcmd;

	pcmd = (struct hostcmd_cmd_fw_flush_timer *)&priv->pcmd_buf[
		INTF_CMDHEADER_LEN(priv->if_ops.inttf_head_len)];

	mutex_lock(&priv->fwcmd_mutex);

	memset(pcmd, 0x00, sizeof(*pcmd));
	pcmd->cmd_hdr.cmd = cpu_to_le16(HOSTCMD_CMD_FW_FLUSH_TIMER);
	pcmd->cmd_hdr.len = cpu_to_le16(sizeof(*pcmd));
	pcmd->value = cpu_to_le32(value);

	if (mwl_fwcmd_exec_cmd(priv, HOSTCMD_CMD_FW_FLUSH_TIMER)) {
		mutex_unlock(&priv->fwcmd_mutex);
		return -EIO;
	}

	mutex_unlock(&priv->fwcmd_mutex);

	return 0;
}

int mwl_fwcmd_set_cdd(struct ieee80211_hw *hw)
{
	struct mwl_priv *priv = hw->priv;
	struct hostcmd_cmd_set_cdd *pcmd;

	pcmd = (struct hostcmd_cmd_set_cdd *)&priv->pcmd_buf[
		INTF_CMDHEADER_LEN(priv->if_ops.inttf_head_len)];

	mutex_lock(&priv->fwcmd_mutex);

	memset(pcmd, 0x00, sizeof(*pcmd));
	pcmd->cmd_hdr.cmd = cpu_to_le16(HOSTCMD_CMD_SET_CDD);
	pcmd->cmd_hdr.len = cpu_to_le16(sizeof(*pcmd));
	pcmd->enable = cpu_to_le32(priv->cdd);

	if (mwl_fwcmd_exec_cmd(priv, HOSTCMD_CMD_SET_CDD)) {
		mutex_unlock(&priv->fwcmd_mutex);
		return -EIO;
	}

	mutex_unlock(&priv->fwcmd_mutex);

	return 0;
}

int mwl_fwcmd_reg_cau(struct ieee80211_hw *hw, u8 flag, u32 reg, u32 *val)
{
	struct mwl_priv *priv = hw->priv;
	struct hostcmd_cmd_bbp_reg_access *pcmd;

	pcmd = (struct hostcmd_cmd_bbp_reg_access *)&priv->pcmd_buf[
				INTF_CMDHEADER_LEN(priv->if_ops.inttf_head_len)];

	mutex_lock(&priv->fwcmd_mutex);

	memset(pcmd, 0x00, sizeof(*pcmd));
	pcmd->cmd_hdr.cmd = cpu_to_le16(HOSTCMD_CMD_CAU_REG_ACCESS);
	pcmd->cmd_hdr.len = cpu_to_le16(sizeof(*pcmd));
	pcmd->offset = cpu_to_le16(reg);
	pcmd->action = cpu_to_le16(flag);
	pcmd->value  = (u8)cpu_to_le32(*val);

	if (mwl_fwcmd_exec_cmd(priv, HOSTCMD_CMD_CAU_REG_ACCESS)) {
		mutex_unlock(&priv->fwcmd_mutex);
		return -EIO;
	}

	*val = (u32)pcmd->value;

	mutex_unlock(&priv->fwcmd_mutex);

	return 0;
}
EXPORT_SYMBOL_GPL(mwl_fwcmd_reg_cau);

int mwl_fwcmd_get_temp(struct ieee80211_hw *hw, s32 *temp)
{
	struct mwl_priv *priv = hw->priv;
	struct hostcmd_cmd_get_temp *pcmd;

	pcmd = (struct hostcmd_cmd_get_temp *)&priv->pcmd_buf[
		INTF_CMDHEADER_LEN(priv->if_ops.inttf_head_len)];

	mutex_lock(&priv->fwcmd_mutex);

	memset(pcmd, 0x00, sizeof(*pcmd));
	pcmd->cmd_hdr.cmd = cpu_to_le16(HOSTCMD_CMD_GET_TEMP);
	pcmd->cmd_hdr.len = cpu_to_le16(sizeof(*pcmd));

	if (mwl_fwcmd_exec_cmd(priv, HOSTCMD_CMD_GET_TEMP)) {
		mutex_unlock(&priv->fwcmd_mutex);
		return -EIO;
	}

	*temp = le32_to_cpu(pcmd->celcius);

	mutex_unlock(&priv->fwcmd_mutex);

	return 0;
}

int mwl_fwcmd_get_fw_region_code(struct ieee80211_hw *hw,
				 u32 *fw_region_code)
{
	struct mwl_priv *priv = hw->priv;
	struct hostcmd_cmd_get_fw_region_code *pcmd;
	u16 cmd;
	int status;

	pcmd = (struct hostcmd_cmd_get_fw_region_code *)&priv->pcmd_buf[
				INTF_CMDHEADER_LEN(priv->if_ops.inttf_head_len)];

	mutex_lock(&priv->fwcmd_mutex);

	memset(pcmd, 0x00, sizeof(*pcmd));
	cmd = HOSTCMD_CMD_GET_FW_REGION_CODE;
	pcmd->cmd_hdr.cmd = cpu_to_le16(cmd);
	pcmd->cmd_hdr.len = cpu_to_le16(sizeof(*pcmd));

	if (mwl_fwcmd_exec_cmd(priv, cmd)) {
		mutex_unlock(&priv->fwcmd_mutex);
		return -EIO;
	}

	if (pcmd->cmd_hdr.result != 0) {
		mutex_unlock(&priv->fwcmd_mutex);
		return -EINVAL;
	}

	status = le32_to_cpu(pcmd->status);

	if (!status)
		*fw_region_code = le32_to_cpu(pcmd->fw_region_code);

	mutex_unlock(&priv->fwcmd_mutex);

	return status;
}

int mwl_fwcmd_get_device_pwr_tbl(struct ieee80211_hw *hw,
				 struct mwl_device_pwr_tbl *device_ch_pwrtbl,
				 u8 *region_code,
				 u8 *number_of_channels,
				 u32 channel_index)
{
	struct mwl_priv *priv = hw->priv;
	struct hostcmd_cmd_get_device_pwr_tbl *pcmd;
	int status;
	u16 cmd;

	pcmd = (struct hostcmd_cmd_get_device_pwr_tbl *)&priv->pcmd_buf[
				INTF_CMDHEADER_LEN(priv->if_ops.inttf_head_len)];

	mutex_lock(&priv->fwcmd_mutex);

	memset(pcmd, 0x00, sizeof(*pcmd));
	cmd = HOSTCMD_CMD_GET_DEVICE_PWR_TBL;
	pcmd->cmd_hdr.cmd = cpu_to_le16(cmd);
	pcmd->cmd_hdr.len = cpu_to_le16(sizeof(*pcmd));
	pcmd->status = cpu_to_le16(cmd);
	pcmd->current_channel_index = cpu_to_le32(channel_index);

	if (mwl_fwcmd_exec_cmd(priv, cmd)) {
		mutex_unlock(&priv->fwcmd_mutex);
		return -EIO;
	}

	device_ch_pwrtbl->channel = pcmd->channel_pwr_tbl.channel;
	memcpy(device_ch_pwrtbl->tx_pwr, pcmd->channel_pwr_tbl.tx_pwr,
	       SYSADPT_TX_GRP_PWR_LEVEL_TOTAL);
	device_ch_pwrtbl->dfs_capable = pcmd->channel_pwr_tbl.dfs_capable;
	device_ch_pwrtbl->ax_ant = pcmd->channel_pwr_tbl.ax_ant;
	device_ch_pwrtbl->cdd = pcmd->channel_pwr_tbl.cdd;
	*region_code = pcmd->region_code;
	*number_of_channels = pcmd->number_of_channels;
	status = le16_to_cpu(pcmd->status);

	mutex_unlock(&priv->fwcmd_mutex);

	return status;
}

int mwl_fwcmd_get_fw_region_code_sc4(struct ieee80211_hw *hw,
				     u32 *fw_region_code)
{
	struct mwl_priv *priv = hw->priv;
	struct hostcmd_cmd_get_fw_region_code_sc4 *pcmd;
	u16 cmd;

	pcmd = (struct hostcmd_cmd_get_fw_region_code_sc4 *)&priv->pcmd_buf[
				INTF_CMDHEADER_LEN(priv->if_ops.inttf_head_len)];

	mutex_lock(&priv->fwcmd_mutex);

	memset(pcmd, 0x00, sizeof(*pcmd));
	cmd = HOSTCMD_CMD_GET_FW_REGION_CODE_SC4;
	pcmd->cmd_hdr.cmd = cpu_to_le16(cmd);
	pcmd->cmd_hdr.len = cpu_to_le16(sizeof(*pcmd));

	if (mwl_fwcmd_exec_cmd(priv, cmd)) {
		mutex_unlock(&priv->fwcmd_mutex);
		return -EIO;
	}

	if (pcmd->cmd_hdr.result != 0) {
		mutex_unlock(&priv->fwcmd_mutex);
		return -EINVAL;
	}

	*fw_region_code = le32_to_cpu(pcmd->fw_region_code);

	mutex_unlock(&priv->fwcmd_mutex);

	return 0;
}

int mwl_fwcmd_get_pwr_tbl_sc4(struct ieee80211_hw *hw,
			      struct mwl_device_pwr_tbl *device_ch_pwrtbl,
			      u8 *region_code,
			      u8 *number_of_channels,
			      u32 channel_index)
{
	struct mwl_priv *priv = hw->priv;
	struct hostcmd_cmd_get_device_pwr_tbl_sc4 *pcmd;
	int status;
	u16 cmd;

	pcmd = (struct hostcmd_cmd_get_device_pwr_tbl_sc4 *)&priv->pcmd_buf[
				INTF_CMDHEADER_LEN(priv->if_ops.inttf_head_len)];

	mutex_lock(&priv->fwcmd_mutex);

	memset(pcmd, 0x00, sizeof(*pcmd));
	cmd = HOSTCMD_CMD_GET_DEVICE_PWR_TBL_SC4;
	pcmd->cmd_hdr.cmd = cpu_to_le16(cmd);
	pcmd->cmd_hdr.len = cpu_to_le16(sizeof(*pcmd));
	pcmd->status = cpu_to_le16(cmd);
	pcmd->current_channel_index = cpu_to_le32(channel_index);

	if (mwl_fwcmd_exec_cmd(priv, cmd)) {
		mutex_unlock(&priv->fwcmd_mutex);
		return -EIO;
	}

	device_ch_pwrtbl->channel = pcmd->channel_pwr_tbl.channel;
	memcpy(device_ch_pwrtbl->tx_pwr, pcmd->channel_pwr_tbl.tx_pwr,
	       SYSADPT_TX_PWR_LEVEL_TOTAL_SC4);
	device_ch_pwrtbl->dfs_capable = pcmd->channel_pwr_tbl.dfs_capable;
	device_ch_pwrtbl->ax_ant = pcmd->channel_pwr_tbl.ax_ant;
	device_ch_pwrtbl->cdd = pcmd->channel_pwr_tbl.cdd;
	*region_code = pcmd->region_code;
	*number_of_channels = pcmd->number_of_channels;
	status = le16_to_cpu(pcmd->status);

	mutex_unlock(&priv->fwcmd_mutex);

	return status;
}

int mwl_fwcmd_quiet_mode(struct ieee80211_hw *hw, bool enable, u32 period,
			 u32 duration, u32 next_offset)
{
	struct mwl_priv *priv = hw->priv;
	struct hostcmd_cmd_quiet_mode *pcmd;

	pcmd = (struct hostcmd_cmd_quiet_mode *)&priv->pcmd_buf[
		INTF_CMDHEADER_LEN(priv->if_ops.inttf_head_len)];

	mutex_lock(&priv->fwcmd_mutex);

	memset(pcmd, 0x00, sizeof(*pcmd));
	pcmd->cmd_hdr.cmd = cpu_to_le16(HOSTCMD_CMD_QUIET_MODE);
	pcmd->cmd_hdr.len = cpu_to_le16(sizeof(*pcmd));
	pcmd->action = cpu_to_le16(WL_SET);
	pcmd->enable = cpu_to_le32(enable);
	if (enable) {
		pcmd->period = cpu_to_le32(period);
		pcmd->duration = cpu_to_le32(duration);
		pcmd->next_offset = cpu_to_le32(next_offset);
	}

	if (mwl_fwcmd_exec_cmd(priv, HOSTCMD_CMD_QUIET_MODE)) {
		mutex_unlock(&priv->fwcmd_mutex);
		return -EIO;
	}

	mutex_unlock(&priv->fwcmd_mutex);

	return 0;
}

int mwl_fwcmd_get_survey(struct ieee80211_hw *hw, int rstReg)
{
	struct mwl_priv *priv = hw->priv;
	struct mwl_survey_info *survey_info;
	int last_read_val = 0, cca_cnt_val = 0, txpe_cnt_val = 0;

	if(!priv->cur_survey_valid)
		return 0;

	survey_info = &priv->cur_survey_info;

	if(mwl_fwcmd_reg_mac(hw, WL_GET, MCU_LAST_READ, &last_read_val))
		return -EIO;
	if(mwl_fwcmd_reg_mac(hw, WL_GET, MCU_CCA_CNT, &cca_cnt_val))
		return -EIO;
	if(mwl_fwcmd_reg_mac(hw, WL_GET, MCU_TXPE_CNT, &txpe_cnt_val))
		return -EIO;

	if(!rstReg) {
		survey_info->filled = SURVEY_INFO_TIME |
					SURVEY_INFO_TIME_BUSY |
					SURVEY_INFO_TIME_TX |
					SURVEY_INFO_NOISE_DBM;

		survey_info->time_period += last_read_val;
		survey_info->time_busy += cca_cnt_val;
		survey_info->time_tx += txpe_cnt_val;
		survey_info->noise = priv->noise;
	}
	return 0;
}

int mwl_fwcmd_dump_otp_data(struct ieee80211_hw *hw)
{
	int otp_data_len;
	struct mwl_priv *priv = hw->priv;
	struct hostcmd_cmd_dump_otp_data *pcmd;

	pcmd = (struct hostcmd_cmd_dump_otp_data *)&priv->pcmd_buf[
		INTF_CMDHEADER_LEN(priv->if_ops.inttf_head_len)];

	mutex_lock(&priv->fwcmd_mutex);

	memset(pcmd, 0x00, sizeof(*pcmd));
	pcmd->cmd_hdr.cmd = cpu_to_le16(HOSTCMD_CMD_DUMP_OTP_DATA);
	pcmd->cmd_hdr.len = cpu_to_le16(sizeof(*pcmd));

	if (mwl_fwcmd_exec_cmd(priv, HOSTCMD_CMD_DUMP_OTP_DATA)) {
		mutex_unlock(&priv->fwcmd_mutex);
		return -EIO;
	}

	otp_data_len = pcmd->cmd_hdr.len - cpu_to_le16(sizeof(*pcmd));

	if (otp_data_len <= MWL_OTP_BUF_SIZE) {
		wiphy_info(hw->wiphy, "OTP data len = %d\n", otp_data_len);
		priv->otp_data.len = otp_data_len;
		memcpy(priv->otp_data.buf, pcmd->pload, otp_data_len);
		mwl_hex_dump(priv->otp_data.buf, priv->otp_data.len);
	} else {
		wiphy_err(hw->wiphy, "Driver OTP buf size is less\n");
	}

	mutex_unlock(&priv->fwcmd_mutex);

	return 0;
}

int mwl_fwcmd_set_pre_scan(struct ieee80211_hw *hw)
{
	struct hostcmd_cmd_pre_scan *pcmd;
	struct mwl_priv *priv = hw->priv;

	pcmd = (struct hostcmd_cmd_pre_scan*)&priv->pcmd_buf[
		INTF_CMDHEADER_LEN(priv->if_ops.inttf_head_len)];

	mutex_lock(&priv->fwcmd_mutex);

	memset(pcmd, 0x00, sizeof(*pcmd));
	pcmd->cmd_hdr.cmd = cpu_to_le16(HOSTCMD_CMD_SET_PRE_SCAN);
	pcmd->cmd_hdr.len = cpu_to_le16(sizeof(*pcmd));

	if (mwl_fwcmd_exec_cmd(priv, HOSTCMD_CMD_SET_PRE_SCAN)) {
		mutex_unlock(&priv->fwcmd_mutex);
		return -EIO;
	}

	mutex_unlock(&priv->fwcmd_mutex);
	return 0;
}

int mwl_fwcmd_set_post_scan(struct ieee80211_hw *hw)
{
	struct hostcmd_cmd_post_scan *pcmd;
	struct mwl_priv *priv = hw->priv;

	pcmd = (struct hostcmd_cmd_post_scan*)&priv->pcmd_buf[
		INTF_CMDHEADER_LEN(priv->if_ops.inttf_head_len)];

	mutex_lock(&priv->fwcmd_mutex);

	memset(pcmd, 0x00, sizeof(*pcmd));
	pcmd->cmd_hdr.cmd = cpu_to_le16(HOSTCMD_CMD_SET_POST_SCAN);
	pcmd->cmd_hdr.len = cpu_to_le16(sizeof(*pcmd));

	if (mwl_fwcmd_exec_cmd(priv, HOSTCMD_CMD_SET_POST_SCAN)) {
		mutex_unlock(&priv->fwcmd_mutex);
		return -EIO;
	}

	mutex_unlock(&priv->fwcmd_mutex);
	return 0;
}

int mwl_fwcmd_get_region_mapping(struct ieee80211_hw *hw,
			struct mwl_region_mapping *map)
{
	struct hostcmd_cmd_region_mapping *pcmd;
	struct mwl_priv *priv = hw->priv;

	pcmd = (struct hostcmd_cmd_region_mapping*)&priv->pcmd_buf[
			INTF_CMDHEADER_LEN(priv->if_ops.inttf_head_len)];

	mutex_lock(&priv->fwcmd_mutex);

	memset(pcmd, 0x00, sizeof(*pcmd));
	pcmd->cmd_hdr.cmd = cpu_to_le16(HOSTCMD_LRD_REGION_MAPPING);
	pcmd->cmd_hdr.len = cpu_to_le16(sizeof(*pcmd));

	if (mwl_fwcmd_exec_cmd(priv, HOSTCMD_LRD_REGION_MAPPING) ||
	    pcmd->cmd_hdr.result != cpu_to_le16(HOSTCMD_RESULT_OK)) {
		mutex_unlock(&priv->fwcmd_mutex);
		wiphy_err(hw->wiphy, "mwl_fwcmd_get_region_mapping failed execution\n");
		return -EIO;
	}

	memcpy(map->cc, pcmd->cc, sizeof(map->cc));

	mutex_unlock(&priv->fwcmd_mutex);

	return 0;
}

int mwl_fwcmd_encryption_set_tx_key(struct ieee80211_hw *hw,
				 struct ieee80211_vif *vif,
				 struct ieee80211_key_conf *key)
{
	struct mwl_priv *priv = hw->priv;
	struct mwl_vif *mwl_vif;
	struct hostcmd_cmd_set_key *pcmd;
	struct hostcmd_cmd_update_encryption *pcmd2;
	int rc;

	if (key->cipher != WLAN_CIPHER_SUITE_WEP40 &&
	    key->cipher != WLAN_CIPHER_SUITE_WEP104) {
		wiphy_err(hw->wiphy, "encryption not support\n");
		return -ENOTSUPP;
	}

	mwl_vif = mwl_dev_get_vif(vif);

	pcmd = (struct hostcmd_cmd_set_key *)&priv->pcmd_buf[
			INTF_CMDHEADER_LEN(priv->if_ops.inttf_head_len)];

	mutex_lock(&priv->fwcmd_mutex);

	memset(pcmd, 0x00, sizeof(*pcmd));
	pcmd->cmd_hdr.cmd   = cpu_to_le16(HOSTCMD_CMD_UPDATE_ENCRYPTION);
	pcmd->cmd_hdr.len   = cpu_to_le16(sizeof(*pcmd));
	pcmd->cmd_hdr.macid = mwl_vif->macid;

	rc = mwl_fwcmd_encryption_set_cmd_info(pcmd, vif, mwl_vif->bssid, key);
	if (rc) {
		mutex_unlock(&priv->fwcmd_mutex);
		if (rc < 0)
			wiphy_err(hw->wiphy, "encryption not supported\n");
		return rc;
	}

	memcpy((void *)&pcmd->key_param.key, key->key, key->keylen);
	pcmd->action_type = cpu_to_le32(ENCR_ACTION_TYPE_SET_KEY);

	if (mwl_fwcmd_exec_cmd(priv, HOSTCMD_CMD_UPDATE_ENCRYPTION)) {
		mutex_unlock(&priv->fwcmd_mutex);
		return -EIO;
	}

	pcmd2 = (struct hostcmd_cmd_update_encryption *)&priv->pcmd_buf[
			INTF_CMDHEADER_LEN(priv->if_ops.inttf_head_len)];

	memset(pcmd2, 0x00, sizeof(*pcmd2));
	pcmd2->cmd_hdr.cmd = cpu_to_le16(HOSTCMD_CMD_UPDATE_ENCRYPTION);
	pcmd2->cmd_hdr.len = cpu_to_le16(sizeof(*pcmd));
	pcmd2->cmd_hdr.macid = mwl_vif->macid;

	pcmd2->action_type = cpu_to_le32(ENCR_ACTION_ENABLE_HW_ENCR);
	pcmd2->action_data[0] = ENCR_TYPE_WEP;

	ether_addr_copy(pcmd2->mac_addr, mwl_vif->bssid);

	if (mwl_fwcmd_exec_cmd(priv, HOSTCMD_CMD_UPDATE_ENCRYPTION)) {
		mutex_unlock(&priv->fwcmd_mutex);
		return -EIO;
	}

	mutex_unlock(&priv->fwcmd_mutex);
	return 0;
}

int lrd_fwcmd_mfg_start(struct ieee80211_hw *hw, u32 *data)
{
	struct hostcmd_cmd_mfg *pcmd;
	struct mwl_priv *priv = hw->priv;

	pcmd = (struct hostcmd_cmd_mfg*)&priv->pcmd_buf[
			INTF_CMDHEADER_LEN(priv->if_ops.inttf_head_len)];

	mutex_lock(&priv->fwcmd_mutex);

	memset(pcmd, 0x00, sizeof(*pcmd) + sizeof(u32));
	pcmd->cmd_hdr.cmd = cpu_to_le16(HOSTCMD_LRD_MFG);
	pcmd->cmd_hdr.len = cpu_to_le16(sizeof(*pcmd) + sizeof(u32));
	pcmd->action      = cpu_to_le32(MFG_TYPE_START);

	if (mwl_fwcmd_exec_cmd(priv, HOSTCMD_LRD_MFG) ||
	    pcmd->cmd_hdr.result != cpu_to_le16(HOSTCMD_RESULT_OK)) {
		mutex_unlock(&priv->fwcmd_mutex);
		wiphy_err(hw->wiphy, "lrd_fwcmd_mfg_start failed execution %x\n", le16_to_cpu(pcmd->cmd_hdr.result));
		return -EIO;
	}

	if (data ) {
		*data =  le32_to_cpu(*(u32*)pcmd->data);
	}

	mutex_unlock(&priv->fwcmd_mutex);
	return 0;
}

int lrd_fwcmd_mfg_end(struct ieee80211_hw *hw)
{
	struct hostcmd_cmd_mfg *pcmd;
	struct mwl_priv *priv = hw->priv;

	pcmd = (struct hostcmd_cmd_mfg*)&priv->pcmd_buf[
			INTF_CMDHEADER_LEN(priv->if_ops.inttf_head_len)];

	mutex_lock(&priv->fwcmd_mutex);

	memset(pcmd, 0x00, sizeof(*pcmd));
	pcmd->cmd_hdr.cmd = cpu_to_le16(HOSTCMD_LRD_MFG);
	pcmd->cmd_hdr.len = cpu_to_le16(sizeof(*pcmd) + sizeof(u32));
	pcmd->action      = cpu_to_le32(MFG_TYPE_END);

	if (mwl_fwcmd_exec_cmd(priv, HOSTCMD_LRD_MFG) ||
	    pcmd->cmd_hdr.result != cpu_to_le16(HOSTCMD_RESULT_OK)) {
		mutex_unlock(&priv->fwcmd_mutex);
		wiphy_err(hw->wiphy, "lrd_fwcmd_mfg_end failed execution %x\n", le16_to_cpu(pcmd->cmd_hdr.result));
		return -EIO;
	}

	mutex_unlock(&priv->fwcmd_mutex);

	return 0;
}

int lrd_fwcmd_mfg_write(struct ieee80211_hw *hw, void *data, int data_len)
{
	struct hostcmd_cmd_mfg *pcmd;
	struct mwl_priv *priv = hw->priv;

	//Validate data is multiple of 32
	if ((data_len % sizeof(u32))) {
		return -EINVAL;
	}

	pcmd = (struct hostcmd_cmd_mfg*)&priv->pcmd_buf[
			INTF_CMDHEADER_LEN(priv->if_ops.inttf_head_len)];

	mutex_lock(&priv->fwcmd_mutex);

	memset(pcmd, 0x00, sizeof(*pcmd) + data_len);
	pcmd->cmd_hdr.cmd = cpu_to_le16(HOSTCMD_LRD_MFG);
	pcmd->cmd_hdr.len = cpu_to_le16(sizeof(*pcmd) + data_len);
	pcmd->action      = cpu_to_le32(MFG_TYPE_WRITE);

	memcpy(pcmd->data, data, data_len);

	if (mwl_fwcmd_exec_cmd(priv, HOSTCMD_LRD_MFG) ||
	    pcmd->cmd_hdr.result != cpu_to_le16(HOSTCMD_RESULT_OK)) {
		mutex_unlock(&priv->fwcmd_mutex);
		wiphy_err(hw->wiphy, "lrd_fwcmd_mfg_write failed execution %x\n", le16_to_cpu(pcmd->cmd_hdr.result));
		return -EIO;
	}

	mutex_unlock(&priv->fwcmd_mutex);
	return 0;
}

int lrd_fwcmd_lru_write(struct ieee80211_hw *hw, void *data, int len, void **rsp)
{
	struct hostcmd_mfgfw_header *pcmd;
	struct mwl_priv *priv = hw->priv;

	pcmd = (struct hostcmd_mfgfw_header*)&priv->pcmd_buf[
			INTF_CMDHEADER_LEN(priv->if_ops.inttf_head_len)];

	mutex_lock(&priv->fwcmd_mutex);

	memset(pcmd, 0x00, sizeof(*pcmd));
	pcmd->cmd = cpu_to_le16(HOSTCMD_CMD_MFG);
	pcmd->len = cpu_to_le16(sizeof(*pcmd) + len);

	memcpy(((u8*)pcmd) + sizeof(struct hostcmd_mfgfw_header), data, len);

	if (mwl_fwcmd_exec_cmd(priv, HOSTCMD_CMD_MFG)) {
		mutex_unlock(&priv->fwcmd_mutex);
		wiphy_err(hw->wiphy, "lrd_fwcmd_lru_write failed execution %x\n", le16_to_cpu(pcmd->result));
		return -EIO;
	}

	if (le16_to_cpu(pcmd->len)) {
		/* To keep structures somewhat encapsulated we are going to squash
		 * part of the hostcmd_mfgfw_header so that we are left only with
		 * lrd_vndr_header and data response
		 */
		u16 len = le16_to_cpu(pcmd->len) - sizeof(struct hostcmd_mfgfw_header) + sizeof(struct lrd_vndr_header);

		*rsp = kzalloc(len, GFP_KERNEL);
		if (!*rsp) {
			mutex_unlock(&priv->fwcmd_mutex);
			wiphy_err(hw->wiphy, "lrd_fwcmd_lru_write failed allocation response %d\n", le16_to_cpu(pcmd->len));
			return -EIO;
		}

		((struct lrd_vndr_header*)*rsp)->command = le16_to_cpu(pcmd->cmd);
		((struct lrd_vndr_header*)*rsp)->result  = le16_to_cpu(pcmd->result);
		((struct lrd_vndr_header*)*rsp)->len     = len;
		memcpy(((u8*)*rsp) + (u8)sizeof(struct lrd_vndr_header), ((u8*)pcmd) + sizeof(struct hostcmd_mfgfw_header) , len - sizeof(struct lrd_vndr_header));
	}

	mutex_unlock(&priv->fwcmd_mutex);

	return 0;
}

int lrd_fwcmd_lrd_write(struct ieee80211_hw *hw, void *data, int len, void **rsp)
{
	struct hostcmd_header *pcmd;
	struct mwl_priv *priv = hw->priv;

	pcmd = (struct hostcmd_header*)&priv->pcmd_buf[
		INTF_CMDHEADER_LEN(priv->if_ops.inttf_head_len)];

	mutex_lock(&priv->fwcmd_mutex);

	memset(pcmd, 0x00, sizeof(*pcmd));
	pcmd->cmd = cpu_to_le16(HOSTCMD_LRD_CMD);
	pcmd->len = cpu_to_le16(sizeof(*pcmd) + len);

	memcpy(((u8*)pcmd) + sizeof(struct hostcmd_header), data, len);

	if (mwl_fwcmd_exec_cmd(priv, HOSTCMD_LRD_CMD)) {
		mutex_unlock(&priv->fwcmd_mutex);
		wiphy_err(hw->wiphy, "lrd_fwcmd_lrd_write failed execution %x\n", le16_to_cpu(pcmd->result));
		return -EIO;
	}

	if (pcmd->len) {
		/* To keep structures somewhat encapsulated we are going to squash
		 * part of the hostcmd_cmd_lrd so that we are left only with
		 * lrd_vndr_header and data response
		 */
		u16 len = le16_to_cpu(pcmd->len) - sizeof(struct hostcmd_header) + sizeof(struct lrd_vndr_header);

		*rsp = kzalloc(len, GFP_KERNEL);
		if (!*rsp) {
			mutex_unlock(&priv->fwcmd_mutex);
			wiphy_err(hw->wiphy, "lrd_fwcmd_lrd_write failed allocation response %d\n", le16_to_cpu(pcmd->len));
			return -EIO;
		}

		((struct lrd_vndr_header*)*rsp)->command = le16_to_cpu(pcmd->cmd);
		((struct lrd_vndr_header*)*rsp)->result  = le16_to_cpu(pcmd->result);
		((struct lrd_vndr_header*)*rsp)->len     = len;

		memcpy(((u8*)*rsp) + sizeof(struct lrd_vndr_header), ((u8*)pcmd) + sizeof(struct hostcmd_header) , len - sizeof(struct lrd_vndr_header));
	}

	mutex_unlock(&priv->fwcmd_mutex);
	return 0;
}

int lrd_fwcmd_lrd_get_caps(struct ieee80211_hw *hw, struct lrd_radio_caps *r_caps)
{
	struct hostcmd_header *pcmd;
	struct lrdcmd_cmd_cap *caps;
	struct mwl_priv       *priv = hw->priv;

	pcmd = (struct hostcmd_header*)&priv->pcmd_buf[
		INTF_CMDHEADER_LEN(priv->if_ops.inttf_head_len)];

	mutex_lock(&priv->fwcmd_mutex);

	memset(pcmd, 0x00, sizeof(*pcmd) + sizeof(*caps));

	pcmd->cmd = cpu_to_le16(HOSTCMD_LRD_CMD);
	pcmd->len = cpu_to_le16(sizeof(*pcmd) + sizeof(*caps));

	//Fill in capability struct
	caps = (struct lrdcmd_cmd_cap*) (((u8*)pcmd) + sizeof(struct hostcmd_header));
	caps->hdr.lrd_cmd = cpu_to_le16(LRD_CMD_CAPS);

	if (mwl_fwcmd_exec_cmd(priv, HOSTCMD_LRD_CMD) ||
		pcmd->result != cpu_to_le16(HOSTCMD_RESULT_OK)) {
		mutex_unlock(&priv->fwcmd_mutex);
		wiphy_err(hw->wiphy, "lrd_fwcmd_lrd_get_caps failed execution %x\n", le16_to_cpu(pcmd->result));
		return -EIO;
	}

	r_caps->capability = le16_to_cpu(caps->capability);
	r_caps->num_mac    = le16_to_cpu(caps->num_mac_addr);
	r_caps->version    = le16_to_cpu(caps->version);

	mutex_unlock(&priv->fwcmd_mutex);
	return 0;
}

int lrd_fwcmd_lrd_set_ant_gain_adjust(struct ieee80211_hw *hw, u32 adjust)
{
	struct hostcmd_header *pcmd;
	struct lrdcmd_cmd_ant_gain_adjust *adjust_cmd;
	struct mwl_priv       *priv = hw->priv;

	if (adjust & ~ANT_GAIN_VALID_MASK) {
		wiphy_err(hw->wiphy, "Invalid antenna gain adjust value (0x%x)!!\n", adjust);
		return -EINVAL;
	}

	pcmd = (struct hostcmd_header*)&priv->pcmd_buf[
		INTF_CMDHEADER_LEN(priv->if_ops.inttf_head_len)];

	mutex_lock(&priv->fwcmd_mutex);

	memset(pcmd, 0x00, sizeof(*pcmd) + sizeof(*adjust_cmd));

	pcmd->cmd = cpu_to_le16(HOSTCMD_LRD_CMD);
	pcmd->len = cpu_to_le16(sizeof(*pcmd) + sizeof(*adjust_cmd));

	//Fill in pwr struct
	adjust_cmd = (struct lrdcmd_cmd_ant_gain_adjust*) (((u8*)pcmd) + sizeof(struct hostcmd_header));
	adjust_cmd->hdr.lrd_cmd = cpu_to_le16(LRD_CMD_ANT_GAIN_ADJUST);
	adjust_cmd->ant_gain_adjust = cpu_to_le32(adjust);

	if (mwl_fwcmd_exec_cmd(priv, HOSTCMD_LRD_CMD) ||
		pcmd->result != cpu_to_le16(HOSTCMD_RESULT_OK)) {
		mutex_unlock(&priv->fwcmd_mutex);
		wiphy_err(hw->wiphy, "lrd_fwcmd_lrd_set_ant_gain_adjust failed execution %x\n", le16_to_cpu(pcmd->result));
		return -EIO;
	}

	mutex_unlock(&priv->fwcmd_mutex);
	return 0;
}

int lrd_fwcmd_lrd_reset_power_table(struct ieee80211_hw *hw)
{
	struct hostcmd_header *pcmd;
	struct mwl_priv       *priv = hw->priv;
	struct lrdcmd_header  *pwr_cmd;


	pcmd = (struct hostcmd_header*)&priv->pcmd_buf[
		INTF_CMDHEADER_LEN(priv->if_ops.inttf_head_len)];

	mutex_lock(&priv->fwcmd_mutex);

	memset(pcmd, 0x00, sizeof(*pcmd) + sizeof(*pwr_cmd));

	pcmd->cmd = cpu_to_le16(HOSTCMD_LRD_CMD);
	pcmd->len = cpu_to_le16(sizeof(*pcmd) + sizeof(*pwr_cmd));

	//Fill in pwr struct
	pwr_cmd = (struct lrdcmd_header*) (((u8*)pcmd) + sizeof(struct hostcmd_header));
	pwr_cmd->lrd_cmd = cpu_to_le16(LRD_CMD_PWR_TABLE_RESET);

	if (mwl_fwcmd_exec_cmd(priv, HOSTCMD_LRD_CMD) ||
		pcmd->result != cpu_to_le16(HOSTCMD_RESULT_OK)) {
		mutex_unlock(&priv->fwcmd_mutex);
		wiphy_err(hw->wiphy, "lrd_fwcmd_lrd_reset_pwr_table failed execution %x\n", le16_to_cpu(pcmd->result));
		return -EIO;
	}

	mutex_unlock(&priv->fwcmd_mutex);
	return 0;
}

int lrd_fwcmd_lrd_set_power_table(struct ieee80211_hw *hw, u16 index, void *data, u32 data_len)
{
	struct hostcmd_header *pcmd;
	struct lrdcmd_cmd_pwr_table *pwr_cmd;
	struct mwl_priv       *priv = hw->priv;
	u8 a1[] = {0xC0,0xEE,0x40,0x40,0x02,0x30};
	u8 a2[] = {0xC0,0xEE,0x40,0x42,0x7F,0x6C};

	if (NULL == data || data_len < MIN_AWM_SIZE ||
	    data_len >= (CMD_BUF_SIZE - INTF_CMDHEADER_LEN(priv->if_ops.inttf_head_len)) ) {
		wiphy_err(hw->wiphy, "Invalid power table (0x%x)!!\n", data_len);
		return -EINVAL;
	}

	pcmd = (struct hostcmd_header*)&priv->pcmd_buf[
		INTF_CMDHEADER_LEN(priv->if_ops.inttf_head_len)];

	mutex_lock(&priv->fwcmd_mutex);

	memset(pcmd, 0x00, sizeof(*pcmd) + sizeof(*pwr_cmd) + data_len);

	pcmd->cmd = cpu_to_le16(HOSTCMD_LRD_CMD);
	pcmd->len = cpu_to_le16(sizeof(*pcmd) + sizeof(*pwr_cmd) + data_len);

	//Fill in pwr struct
	pwr_cmd = (struct lrdcmd_cmd_pwr_table*) (((u8*)pcmd) + sizeof(struct hostcmd_header));
	pwr_cmd->hdr.lrd_cmd = cpu_to_le16(LRD_CMD_PWR_TABLE);
	pwr_cmd->len = cpu_to_le16(data_len + sizeof(struct lrdcmd_cmd_pwr_table) - offsetof(struct lrdcmd_cmd_pwr_table, len));

	//Add header
	pwr_cmd->frm.frame_control = cpu_to_le16(0x4208);
	pwr_cmd->frm.duration_id   = cpu_to_le16(44);
	pwr_cmd->frm.seq_ctrl      = cpu_to_le16(index << 4);
	memcpy(pwr_cmd->frm.addr1, a1, sizeof(pwr_cmd->frm.addr1));
	memcpy(pwr_cmd->frm.addr2, a2, sizeof(pwr_cmd->frm.addr2));
	memcpy(pwr_cmd->frm.addr3, a2, sizeof(pwr_cmd->frm.addr3));

	// It is expected caller has dealt with any endian issues in data.
	memcpy(pwr_cmd->data, data, data_len);

	if (mwl_fwcmd_exec_cmd(priv, HOSTCMD_LRD_CMD) ||
		pcmd->result != cpu_to_le16(HOSTCMD_RESULT_OK)) {
		mutex_unlock(&priv->fwcmd_mutex);
		wiphy_err(hw->wiphy, "lrd_fwcmd_lrd_set_pwr_table failed execution %x\n", le16_to_cpu(pcmd->result));
		return -EIO;
	}

	mutex_unlock(&priv->fwcmd_mutex);
	return 0;
}

int lrd_fwcmd_lrd_get_power_table_result(struct ieee80211_hw *hw, u32 *result, u32 *pn)
{
	struct hostcmd_header *pcmd;
	struct lrdcmd_cmd_pwr_table_result *pwr_result;
	struct mwl_priv       *priv = hw->priv;

	pcmd = (struct hostcmd_header*)&priv->pcmd_buf[
		INTF_CMDHEADER_LEN(priv->if_ops.inttf_head_len)];

	mutex_lock(&priv->fwcmd_mutex);

	memset(pcmd, 0x00, sizeof(*pcmd) + sizeof(*pwr_result));

	pcmd->cmd = cpu_to_le16(HOSTCMD_LRD_CMD);
	pcmd->len = cpu_to_le16(sizeof(*pcmd) + sizeof(*pwr_result));

	//Fill in pwr struct
	pwr_result = (struct lrdcmd_cmd_pwr_table_result*) (((u8*)pcmd) + sizeof(struct hostcmd_header));
	pwr_result->hdr.lrd_cmd = cpu_to_le16(LRD_CMD_PWR_TABLE_RESULT);

	if (mwl_fwcmd_exec_cmd(priv, HOSTCMD_LRD_CMD) ||
		pcmd->result != cpu_to_le16(HOSTCMD_RESULT_OK)) {
		mutex_unlock(&priv->fwcmd_mutex);
		wiphy_err(hw->wiphy, "lrd_fwcmd_lrd_set_pwr_table failed execution %x\n", le16_to_cpu(pcmd->result));
		return -EIO;
	}

	*result = le32_to_cpu(pwr_result->result);
	*pn     = le32_to_cpu(pwr_result->pn);

	mutex_unlock(&priv->fwcmd_mutex);
	return 0;
}

int mwl_fwcmd_set_monitor_mode(struct ieee80211_hw *hw, bool enable)
{
	struct hostcmd_cmd_monitor_mode *pcmd;
	struct mwl_priv *priv = hw->priv;
	pcmd = (struct hostcmd_cmd_monitor_mode*)&priv->pcmd_buf[
		INTF_CMDHEADER_LEN(priv->if_ops.inttf_head_len)];

	mutex_lock(&priv->fwcmd_mutex);

	memset(pcmd, 0x00, sizeof(*pcmd));
	pcmd->cmd_hdr.cmd = cpu_to_le16(HOSTCMD_CMD_MONITOR_MODE);
	pcmd->cmd_hdr.len = cpu_to_le16(sizeof(*pcmd));
	pcmd->enableFlag = enable;

	if (mwl_fwcmd_exec_cmd(priv, HOSTCMD_CMD_MONITOR_MODE)) {
		mutex_unlock(&priv->fwcmd_mutex);
		return -EIO;
	}

	mutex_unlock(&priv->fwcmd_mutex);
	return 0;
}
