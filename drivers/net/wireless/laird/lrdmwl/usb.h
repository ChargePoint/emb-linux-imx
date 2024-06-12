/*
 * This file contains definitions for mwifiex USB interface driver.
 *
 * Copyright (C) 2012-2014, Marvell International Ltd.
 * Copyright (C) 2018-2020 Laird Connectivity
 *
 * This software file (the "File") is distributed by Marvell International
 * Ltd. under the terms of the GNU General Public License Version 2, June 1991
 * (the "License").  You may use, redistribute and/or modify this File in
 * accordance with the terms and conditions of the License, a copy of which
 * is available by writing to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA or on the
 * worldwide web at http://www.gnu.org/licenses/old-licenses/gpl-2.0.txt.
 *
 * THE FILE IS DISTRIBUTED AS-IS, WITHOUT WARRANTY OF ANY KIND, AND THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE
 * ARE EXPRESSLY DISCLAIMED.  The License provides additional details about
 * this warranty disclaimer.
 */

#ifndef _LRDMWL_USB_H
#define _LRDMWL_USB_H

#include <linux/completion.h>
#include <linux/skbuff.h>
#include <linux/usb.h>
#include <linux/etherdevice.h>

#define USB8XXX_VID		0x1286

#define USB8766_PID_1		0x2041
#define USB8766_PID_2		0x2042
#define USB8797_PID_1		0x2043
#define USB8797_PID_2		0x2044
#define USB8801_PID_1		0x2049
#define USB8801_PID_2		0x204a
#define USB8997_PID_1		0x2052
#define USB8997_PID_2		0x204e


#define USB8XXX_FW_DNLD		1
#define USB8XXX_FW_READY	2
#define USB8XXX_FW_MAX_RETRY	3

#define MWIFIEX_TX_DATA_URB	6
#define MWIFIEX_RX_DATA_URB	6
#define MWIFIEX_USB_TIMEOUT	100

#define USB8997_DEFAULT_FW_NAME	"mwlwifi/88W8997_usb.bin"

#define FW_DNLD_TX_BUF_SIZE	4096
#define FW_DNLD_RX_BUF_SIZE	2048
#define FW_HAS_LAST_BLOCK	0x00000004
#define FW_CMD_7		0x00000007

#define MWIFIEX_RX_DATA_BUF_SIZE     (4 * 1024)
#define MWIFIEX_RX_CMD_BUF_SIZE      (4 * 1024)

#define HIGH_RX_PENDING     50
#define LOW_RX_PENDING      20


#define FW_DATA_XMIT_SIZE \
	(sizeof(struct fw_header) + dlen + sizeof(u32))

struct urb_context {
	struct mwl_priv *priv;
	struct sk_buff *skb;
	struct urb *urb;
	u8 ep;
	struct list_head list;
};

struct usb_tx_data_port {
	u8 tx_data_ep;
	atomic_t tx_data_urb_pending;
	int tx_data_ix;
	struct urb_context tx_data_list[MWIFIEX_TX_DATA_URB];
};

struct mwl_wait_queue {
        wait_queue_head_t wait;
        int status;
};



struct usb_card_rec {
	struct mwl_priv *priv;
	struct usb_device *udev;
	struct usb_interface *intf;
	struct completion fw_done;
	u8 rx_cmd_ep;
	struct urb_context rx_cmd;

	struct urb_context rx_data_list[MWIFIEX_RX_DATA_URB];
	struct list_head rx_urb_pending_list;
	spinlock_t rx_urb_lock;
	struct usb_anchor rx_urb_anchor;

	u8 usb_boot_state;
	u8 rx_data_ep;
	u8 tx_cmd_ep;

	int bulk_out_maxpktsize;
	struct urb_context tx_cmd;
	struct usb_tx_data_port port;
	int rx_cmd_ep_type;
	u8 rx_cmd_interval;
	int tx_cmd_ep_type;
	u8 tx_cmd_interval;
	int chip_type;

	bool cmd_resp_recvd;
	struct mwl_wait_queue cmd_wait_q;
	u16 cmd_id;

	struct sk_buff_head rx_data_q;
	atomic_t rx_pending;
	struct tasklet_struct tx_task;

	int reset_pwd_gpio;

	bool dying;
};

struct fw_header {
	__le32 dnld_cmd;
	__le32 base_addr;
	__le32 data_len;
	__le32 crc;
};

struct fw_sync_header {
	__le32 cmd;
	__le32 seq_num;
} __packed;

struct fw_data {
	struct fw_header fw_hdr;
	__le32 seq_num;
	u8 data[1];
} __packed;

enum mwifiex_usb_ep {
        MWIFIEX_USB_EP_CMD_EVENT = 1,
        MWIFIEX_USB_EP_DATA = 2,
        MWIFIEX_USB_EP_DATA_CH2 = 3,
};

enum RESPONSES {
        OK = 0,
        ERROR = 1
};

struct mwl_rxinfo {
        struct sk_buff *parent;
        u8 bss_num;
        u8 bss_type;
        u8 use_count;
        u8 buf_type;
};

struct mwl_txinfo {
        u32 status_code;
        u8 flags;
        u8 bss_num;
        u8 bss_type;
        u8 aggr_num;
        u32 pkt_len;
        u8 ack_frame_id;
        u64 cookie;
};

struct mwl_cb {
	union {
		struct mwl_rxinfo rx_info;
		struct mwl_txinfo tx_info;
	};
};

static inline struct mwl_rxinfo *MWL_SKB_RXCB(struct sk_buff *skb)
{
        struct mwl_cb *cb = (struct mwl_cb *)skb->cb;

        BUILD_BUG_ON(sizeof(struct mwl_cb) > sizeof(skb->cb));
        return &cb->rx_info;
}

#endif /*_LRDMWL_USB_H */
