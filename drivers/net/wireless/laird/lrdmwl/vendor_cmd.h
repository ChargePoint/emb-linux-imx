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

#ifndef _LRD_VENDOR_CMD_H_
#define _LRD_VENDOR_CMD_H_

#define LRD_OUI 0xC0EE40

enum lrd_vendor_commands {
	LRD_VENDOR_CMD_MFG_START = 1,
	LRD_VENDOR_CMD_MFG_WRITE,
	LRD_VENDOR_CMD_MFG_STOP,
	LRD_VENDOR_CMD_LRU_START,
	LRD_VENDOR_CMD_LRU_WRITE,
	LRD_VENDOR_CMD_LRU_STOP,
	LRD_VENDOR_CMD_LRD_WRITE,
	LRD_VENDOR_CMD_MAX,
};

enum lrd_vendor_events {
	LRD_VENDOR_EVENT_RESTART = 0x8000,
};

enum lrd_nlattrs {
	LRD_ATTR_CMD_RSP = 1,    //BZ13280 Some Android NL libraries discard attributes that are 0
	LRD_ATTR_DATA,
	LRD_ATTR_RESTART_REASON,
	LRD_ATTR_MAX
};

enum lrd_attr_restart_reason {
	LRD_REASON_RESET = 1,
	LRD_REASON_RESUME
};


void lrd_set_vendor_commands(struct wiphy *wiphy);
void lrd_set_vendor_events  (struct wiphy *wiphy);
#endif
