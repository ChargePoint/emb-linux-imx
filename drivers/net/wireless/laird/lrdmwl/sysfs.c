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

/* Description:  This file implements sysfs related functions. */

#include <linux/sysfs.h>

#include "sysadpt.h"
#include "dev.h"

static const char chipname[MWLUNKNOWN][8] = {
	"88W8864",
	"88W8897",
	"88W8964",
	"88W8997"
};

static const char chipbus[5][5] = {
	"?",
	"?",
	"SDIO",
	"PCIE",
	"USB"
};
static ssize_t info_show(struct device *d, struct device_attribute *attr, char *buf)
{
	struct ieee80211_hw  *hw = dev_get_drvdata(d);
	struct mwl_priv      *priv;
	char  *p = (char *)buf;
	int  len = 0;
	int size = PAGE_SIZE;

	if (!p || !hw || !hw->priv) {
		return 0;
	}

	priv = hw->priv;

	len += scnprintf(p + len, size - len, "\n");
	len += scnprintf(p + len, size - len, "Driver name : %s\n", MWL_DRV_NAME);
	len += scnprintf(p + len, size - len, "Chip type   : %s-%s\n", chipname[priv->chip_type], chipbus[priv->host_if]);
	len += scnprintf(p + len, size - len, "HW  version : %d\n", priv->hw_data.hw_version);
	len += scnprintf(p + len, size - len, "FW  version : %d.%d.%d.%d\n",
			 ((priv->hw_data.fw_release_num >> 24) & 0xff),
			 ((priv->hw_data.fw_release_num >> 16) & 0xff),
			 ((priv->hw_data.fw_release_num >> 8) & 0xff),
			 ((priv->hw_data.fw_release_num >> 0) & 0xff));
	len += scnprintf(p + len, size - len, "DRV version : %s\n", LRD_BLD_VERSION);
	len += scnprintf(p + len, size - len, "OTP version : %d\n", priv->radio_caps.version);
	len += scnprintf(p + len, size - len, "OTP cap     : 0x%x\n", priv->radio_caps.capability);
	len += scnprintf(p + len, size - len, "OTP num mac : %d\n", priv->radio_caps.num_mac);
	len += scnprintf(p + len, size - len, "Radio Type  : %s%s%s\n",
			 (priv->radio_caps.capability & LRD_CAP_SU60) ? "SU":"ST",
			 (priv->radio_caps.capability & LRD_CAP_440)  ? "-440":"",
			 (priv->radio_caps.capability & LRD_CAP_SOM8MP)  ? "-SOM8MP":"");
	len += scnprintf(p + len, size - len, "BT Offset   : %s\n",
			 (priv->radio_caps.capability & LRD_CAP_BT_SEQ) ? "1":
			 (priv->radio_caps.capability & LRD_CAP_BT_DUP) ? "0":"Default");
	len += scnprintf(p + len, size - len, "MAC address : %pM\n", priv->hw_data.mac_addr);
	len += scnprintf(p + len, size - len, "Region code : 0x%02x (0x%02x)\n", priv->reg.cc.region, priv->reg.otp.region);
	len += scnprintf(p + len, size - len, "Country code: '%c%c' ('%c%c')\n",
			 priv->reg.cc.alpha2[0],priv->reg.cc.alpha2[1],
			 priv->reg.otp.alpha2[0],priv->reg.otp.alpha2[1]);
	len += scnprintf(p + len, size - len, "TX antenna  : %d\n", priv->ant_tx_num);
	len += scnprintf(p + len, size - len, "RX antenna  : %d\n", priv->ant_rx_num);
	len += scnprintf(p + len, size - len, "\n");

	return len;
}

static ssize_t mfg_mode_show(struct device *d, struct device_attribute *attr, char *buf)
{
	struct ieee80211_hw  *hw = dev_get_drvdata(d);
	struct mwl_priv      *priv;

	if (!buf || !hw || !hw->priv)
		return 0;

	priv = hw->priv;

	return scnprintf(buf, PAGE_SIZE, "%d\n", priv->mfg_mode);
}

static DEVICE_ATTR(info, 0444, info_show, NULL);
static DEVICE_ATTR(mfg_mode, 0444, mfg_mode_show, NULL);

static struct attribute *lrd_sys_status_entries[] = {
	&dev_attr_info.attr,
	&dev_attr_mfg_mode.attr,
	NULL
};

static const struct attribute_group lrd_attribute_group = {
	.name  = "lrd",
	.attrs = lrd_sys_status_entries,
};

void lrd_sysfs_init(struct ieee80211_hw *hw)
{
	struct mwl_priv *priv;
	int ret;

	if (!hw || !hw->priv ) {
		return;
	}

	priv = hw->priv;

	if (priv->dev) {
		ret = sysfs_create_group(&priv->dev->kobj, &lrd_attribute_group);
		if (ret)
			wiphy_err(priv->hw->wiphy, "%s: Unable to create attribute group!\n", __func__);
	}
}

void lrd_sysfs_remove(struct ieee80211_hw *hw)
{
	struct mwl_priv *priv;

	if (!hw || !hw->priv) {
		return;
	}

	priv = hw->priv;

	if (priv) {
		sysfs_remove_group(&priv->dev->kobj, &lrd_attribute_group);
	}
}
