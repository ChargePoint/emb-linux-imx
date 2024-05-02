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

#ifndef _MAIN_H_
#define _MAIN_H_

#define MWL_TXANT_BMP_TO_NUM(bmp)	\
(((bmp & MWL_8997_DEF_TX_ANT_BMP) == MWL_8997_DEF_TX_ANT_BMP)? 2 : 1)

#define MWL_RXANT_BMP_TO_NUM(bmp)	\
(((bmp & MWL_8997_DEF_RX_ANT_BMP) == MWL_8997_DEF_RX_ANT_BMP)? 2 : 1)


/* WMM Turbo mode */
extern int wmm_turbo;

extern int EDMAC_Ctrl;
extern int tx_amsdu_enable;

int mwl_add_card(void *, struct mwl_if_ops *, struct device_node *of_node);
void mwl_wl_deinit(struct mwl_priv *);
void mwl_set_ht_caps (struct mwl_priv *priv, struct ieee80211_supported_band *band);
void mwl_set_vht_caps(struct mwl_priv *priv, struct ieee80211_supported_band *band);
void mwl_ieee80211_free_hw(struct mwl_priv *);
extern void timer_routine(struct timer_list *t);
extern void mwl_restart_ds_timer(struct mwl_priv *priv, bool force);
extern void mwl_delete_ds_timer(struct mwl_priv *priv);
extern int mwl_fw_dnld_and_init(struct mwl_priv *priv);
extern int mwl_shutdown_sw(struct mwl_priv *priv, bool suspend);
extern int mwl_reinit_sw(struct mwl_priv *priv, bool suspend);
extern void mwl_mac80211_stop(struct ieee80211_hw *hw);
extern void mwl_mac80211_remove_vif(struct mwl_priv *priv, struct ieee80211_vif *vif);
extern int mwl_mac80211_sta_remove(struct ieee80211_hw *hw, struct ieee80211_vif *vif, struct ieee80211_sta *sta);

#ifdef CONFIG_SCHED_HRTICK
	#define	lrdmwl_delay(d) usleep_range(d, d + 10);
#else
	#define	lrdmwl_delay(d) (d < (MAX_UDELAY_MS * 1000) ? udelay(d) : mdelay(d / 1000))
#endif

#ifdef CONFIG_PM

extern void lrd_enable_wowlan(struct mwl_priv *priv);
extern void lrd_disable_wowlan(struct mwl_priv *priv);
int lrd_probe_of_wowlan(struct mwl_priv *priv, struct device_node *of_node);

extern void lrd_report_wowlan_wakeup(struct mwl_priv *priv);

#else

static inline void lrd_enable_wowlan(struct mwl_priv *priv) {}
static inline void lrd_disable_wowlan(struct mwl_priv *priv) {}
static inline int lrd_probe_of_wowlan(struct mwl_priv *priv,
	struct device_node *of_node) { return 0; }

#endif

void lrd_radio_recovery(struct mwl_priv *priv);
void lrd_dump_max_pwr_table(struct mwl_priv *priv);

extern void lrd_cc_event(struct work_struct *work);
extern void lrd_send_restart_event(struct wiphy *wiphy, struct ieee80211_vif *vif, uint32_t reason);

#endif
