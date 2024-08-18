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

/* Description:  This file defines debug fs related functions. */

#ifndef _LRD_SYSFS_H_
#define _LRD_SYSFS_H_

void lrd_sysfs_init(struct ieee80211_hw *hw);
void lrd_sysfs_remove(struct ieee80211_hw *hw);

#endif /* _LRD_SYSFS_H_ */
