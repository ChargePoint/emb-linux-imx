/*
 * Marvell Wireless LAN device driver: USB specific handling
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

#include "sysadpt.h"
#include "usb.h"
#include "dev.h"
#include "main.h"
#include "fwcmd.h"
#include "tx.h"
#include "rx.h"
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/of.h>

#define MWL_USBDRV_VERSION  "1.0-20171201"
#define LRD_USB_VERSION     LRD_BLD_VERSION "-" MWL_USBDRV_VERSION
#define LRD_USB_DESC        "Laird 60 Series Wireless USB Network Driver"

static struct mwl_if_ops usb_ops1;
#define INTF_HEADER_LEN	4

/* 88W8997 datasheet requires PMIC_EN/PMU_EN to remain de-asserted for a minimum of 100ms */
#define USB_DEFAULT_POWERDOWN_DELAY_MS		100


static const struct usb_device_id mwl_usb_table[] = {
	/* 8997 */
	{USB_DEVICE(USB8XXX_VID, USB8997_PID_1)},
	{USB_DEVICE_AND_INTERFACE_INFO(USB8XXX_VID, USB8997_PID_2,
				       USB_CLASS_VENDOR_SPEC,
				       USB_SUBCLASS_VENDOR_SPEC, 0xff)},
	{ }	/* Terminating entry */
};

MODULE_DEVICE_TABLE(usb, mwl_usb_table);

static const struct of_device_id mwl_usb_of_match_table[] = {
	{ .compatible = "usb1286,204e" },
	{ .compatible = "usbif1286,204e.config1.2" },
	{ }
};


static unsigned int reset_pwd_gpio = ARCH_NR_GPIOS;


static struct mwl_chip_info mwl_chip_tbl[] = {
	[MWL8997] = {
		.part_name      = "88W8997",
		.fw_image       = MWL_FW_ROOT"/88W8997_usb.bin",
		.mfg_image      = MWL_FW_ROOT"/88W8997_usb_mfg.bin",
		.antenna_tx     = ANTENNA_TX_2,
		.antenna_rx     = ANTENNA_RX_2,
	},
};


#define WARNING 3
#define MSG 5
#define INFO 5
#define CMD 0x10
#define DATA 0x8
#define EVENT 0x20

#define MWIFIEX_TYPE_LEN                        4
#define MWIFIEX_USB_TYPE_CMD                    0xF00DFACE
#define MWIFIEX_USB_TYPE_DATA                   0xBEADC0DE
#define MWIFIEX_USB_TYPE_EVENT                  0xBEEFFACE

static int mwl_usb_submit_rx_urb(struct urb_context *ctx);
static void mwl_usb_free(struct usb_card_rec *card);
static int mwl_usb_restart_handler(struct mwl_priv *priv);
/* This function probes an mwl device and registers it. It allocates
 * the card structure, initiates the device registration and initialization
 * procedure by adding a logical interface.
 */
static int mwl_usb_probe(struct usb_interface *intf,
			     const struct usb_device_id *id)
{
	struct usb_device *udev = interface_to_usbdev(intf);
	struct usb_host_interface *iface_desc = intf->cur_altsetting;
	struct usb_endpoint_descriptor *epd;
	int gpio, rc, i;
	struct usb_card_rec *card;
	u16 id_vendor, id_product, bcd_device;
	struct device_node *of_node = NULL;
	enum of_gpio_flags flags;

	card = devm_kzalloc(&intf->dev, sizeof(*card), GFP_KERNEL);
	if (!card) {
		pr_err("allocate usb_card_rec structure failed\n");
		return -ENOMEM;
	}

	init_completion(&card->fw_done);

	id_vendor  = le16_to_cpu(udev->descriptor.idVendor);
	id_product = le16_to_cpu(udev->descriptor.idProduct);
	bcd_device = le16_to_cpu(udev->descriptor.bcdDevice);
	pr_debug("info: VID/PID = %X/%X, Boot2 version = %X\n",
		 id_vendor, id_product, bcd_device);

	card->chip_type = MWL8997;

	/* PID_1 is used for firmware downloading only */
	switch (id_product) {
	case USB8997_PID_1:
		card->usb_boot_state = USB8XXX_FW_DNLD;
		break;
	case USB8997_PID_2:
		card->usb_boot_state = USB8XXX_FW_READY;
		break;
	default:
		pr_warn("unknown id_product %#x\n", id_product);
		card->usb_boot_state = USB8XXX_FW_DNLD;
		break;
	}

	card->udev = udev;
	card->intf = intf;

	for (i = 0; i < iface_desc->desc.bNumEndpoints; ++i) {
		epd = &iface_desc->endpoint[i].desc;
		if (usb_endpoint_dir_in(epd) &&
		    usb_endpoint_num(epd) == MWIFIEX_USB_EP_CMD_EVENT &&
		    (usb_endpoint_xfer_bulk(epd) ||
		     usb_endpoint_xfer_int(epd))) {

			card->rx_cmd_ep_type  = usb_endpoint_type(epd);
			card->rx_cmd_interval = epd->bInterval;
			card->rx_cmd_ep       = usb_endpoint_num(epd);

			pr_debug("info: Rx CMD/EVT:: max pkt size: %d, addr: %d, ep_type: %d\n",
				 le16_to_cpu(epd->wMaxPacketSize), epd->bEndpointAddress, card->rx_cmd_ep_type);
		}

		if (usb_endpoint_dir_in(epd) &&
		    usb_endpoint_num(epd) == MWIFIEX_USB_EP_DATA &&
		    usb_endpoint_xfer_bulk(epd)) {

			card->rx_data_ep = usb_endpoint_num(epd);

			pr_debug("info: bulk IN: max pkt size: %d, addr: %d\n",
				 le16_to_cpu(epd->wMaxPacketSize), epd->bEndpointAddress);
		}

		if (usb_endpoint_dir_out(epd) &&
		    usb_endpoint_num(epd) == MWIFIEX_USB_EP_DATA &&
		    usb_endpoint_xfer_bulk(epd)) {

			card->port.tx_data_ep = usb_endpoint_num(epd);
			atomic_set(&card->port.tx_data_urb_pending, 0);

			pr_debug("info: bulk OUT 0: max pkt size: %d, addr: %d\n",
				 le16_to_cpu(epd->wMaxPacketSize), epd->bEndpointAddress);
		}

		if (usb_endpoint_dir_out(epd) &&
		    usb_endpoint_num(epd) == MWIFIEX_USB_EP_DATA_CH2 &&
		    usb_endpoint_xfer_bulk(epd)) {

			pr_debug("info: bulk OUT chan2 (not supported):\t max pkt size: %d, addr: %d\n",
				 le16_to_cpu(epd->wMaxPacketSize), epd->bEndpointAddress);
		}

		if (usb_endpoint_dir_out(epd) &&
		    usb_endpoint_num(epd) == MWIFIEX_USB_EP_CMD_EVENT &&
		    (usb_endpoint_xfer_bulk(epd) ||
		     usb_endpoint_xfer_int(epd))) {

			card->tx_cmd_ep_type      = usb_endpoint_type(epd);
			card->tx_cmd_interval     = epd->bInterval;
			card->tx_cmd_ep           = usb_endpoint_num(epd);
			card->bulk_out_maxpktsize = le16_to_cpu(epd->wMaxPacketSize);

			pr_debug("info: bulk OUT: max pkt size: %d, addr: %d\n",
				 le16_to_cpu(epd->wMaxPacketSize), epd->bEndpointAddress);
		}
	}

	usb_set_intfdata(intf, card);
	memcpy(&usb_ops1.mwl_chip_tbl, &mwl_chip_tbl[card->chip_type],
                sizeof(struct mwl_chip_info));

	// Initialize reset gpio to value provided by mod parameter if it exists
	card->reset_pwd_gpio = reset_pwd_gpio;
	flags = 0;

	/* device tree node parsing and platform specific configuration */
	if (dev_of_node(&intf->dev)) {
		if (of_match_node(mwl_usb_of_match_table, dev_of_node(&intf->dev))) {
			of_node = dev_of_node(&intf->dev);

			// Override gpio reset with value provided in devicetree if it exists
			gpio = of_get_named_gpio_flags(of_node, "pmu-en-gpios", 0, &flags);
			if (!gpio_is_valid(gpio))
				gpio = of_get_named_gpio_flags(of_node, "reset-gpios", 0, &flags);

			if (gpio_is_valid(gpio))
				card->reset_pwd_gpio = gpio;
		}
	}

	if (gpio_is_valid(card->reset_pwd_gpio)) {
		rc = devm_gpio_request_one(&intf->dev, card->reset_pwd_gpio,
			flags | GPIOF_OUT_INIT_HIGH, "wifi_pmu_en");
		if (!rc) {
			pr_info("PMU_EN GPIO %d configured\n", card->reset_pwd_gpio);
			usb_ops1.hardware_restart = mwl_usb_restart_handler;
		}
		else
			pr_err("Failed to configure PMU_EN gpio %d!!\n", card->reset_pwd_gpio);
	}
	else
		pr_info("PMU_EN GPIO not configured\n");

	rc = mwl_add_card(card, &usb_ops1, of_node);

	if (rc) {
		if (rc != -EINPROGRESS)
			pr_err("%s: Failed to add card: %d\n", __func__, rc);
		goto err_add_card;
	}

	usb_get_dev(udev);

	return 0;

err_add_card:
	mwl_usb_free(card);
	return rc;
}

static void mwl_usb_free(struct usb_card_rec *card)
{
	struct usb_tx_data_port *port;
	struct mwl_priv * priv = card->priv;
	int i;

	if (card->rx_cmd.urb) {
		usb_kill_urb(card->rx_cmd.urb);
		usb_free_urb(card->rx_cmd.urb);
		card->rx_cmd.urb = NULL;
	}

	if (card->rx_cmd.skb) {
		dev_kfree_skb_any(card->rx_cmd.skb);
		card->rx_cmd.skb = NULL;
	}

	if(likely(!priv->mfg_mode) && card->rx_data_list[0].urb) {
		usb_kill_anchored_urbs(&card->rx_urb_anchor);
		for (i = 0; i < MWIFIEX_RX_DATA_URB; i++) {
			if (card->rx_data_list[i].urb) {
				usb_free_urb(card->rx_data_list[i].urb);
				card->rx_data_list[i].urb = NULL;
			}
			if (card->rx_data_list[i].skb) {
				dev_kfree_skb_any(card->rx_data_list[i].skb);
				card->rx_data_list[i].skb = NULL;
			}
		}

		port = &card->port;
		for (i = 0; i < MWIFIEX_TX_DATA_URB; i++) {
			if (port->tx_data_list[i].urb) {
				// Note - only urbs that had been submitted have skbs
				// skb is free'd in tx complete handler when urb is killed
				usb_kill_urb(port->tx_data_list[i].urb);
				usb_free_urb(port->tx_data_list[i].urb);
				port->tx_data_list[i].urb = NULL;
			}
		}
	}

	if (card->tx_cmd.urb) {
		usb_kill_urb(card->tx_cmd.urb);
		usb_free_urb(card->tx_cmd.urb);
		card->tx_cmd.urb = NULL;
	}

	return;
}


static int mwl_usb_resume(struct usb_interface *intf)
{
	return 0;
}

static int mwl_usb_suspend(struct usb_interface *intf,pm_message_t message)
{
	return 0;
}

static void mwl_usb_disconnect(struct usb_interface *intf)
{
	struct usb_card_rec *card = usb_get_intfdata(intf);
	struct mwl_priv *adapter;

	/*TODO	wait_for_completion(&card->fw_done);*/
	adapter = card->priv;

	mwl_wl_deinit(adapter);

	/*TODO : deauthenticate and shutdown firmware*/

	card->dying = true;

	mwl_usb_free(card);

	/* TODO mwl_remove_card(adapter);*/

	if (adapter->if_ops.ptx_task != NULL)
		tasklet_kill(adapter->if_ops.ptx_task);

	tasklet_kill(&adapter->rx_task);

	usb_put_dev(interface_to_usbdev(intf));
}

static struct usb_driver mwl_usb_driver = {
	.name        = MWL_DRV_NAME,
	.probe       = mwl_usb_probe,
	.disconnect  = mwl_usb_disconnect,
	.id_table    = mwl_usb_table,
	.suspend     = mwl_usb_suspend,
	.resume      = mwl_usb_resume,
	.soft_unbind = 1,
};



static bool mwl_usb_check_card_status(struct mwl_priv *priv)
{
	return true;
}

/*
  Packet format (sdio interface):
  [TYPE:4][mwl_rx_desc:44][mwl_dma_data:32][payload wo 802.11 header]
*/
void mwl_handle_rx_packet(struct mwl_priv *priv, struct sk_buff *skb)
{
	struct ieee80211_hw *hw = priv->hw;
	struct mwl_rx_desc *pdesc;
	struct mwl_dma_data *dma;
	struct sk_buff *prx_skb = skb;
	struct ieee80211_rx_status status;
	struct mwl_vif *mwl_vif = NULL;
	struct ieee80211_hdr *wh;
	struct mwl_rx_event_data *rx_evnt;

	pdesc = (struct mwl_rx_desc *)prx_skb->data;

	/* => todo:
	// Save the rate info back to card
	//card->rate_info = pdesc->rate;
	//=> rateinfo--
	*/
	if (pdesc->payldType == RX_PAYLOAD_TYPE_EVENT_INFO) {
		skb_pull(prx_skb, sizeof(struct mwl_rx_desc));
		rx_evnt = (struct mwl_rx_event_data *)prx_skb->data;
		mwl_handle_rx_event(hw, rx_evnt);
		dev_kfree_skb_any(prx_skb);
		return;
	}

	if ((pdesc->channel != hw->conf.chandef.chan->hw_value) &&
		!(priv->roc.tmr_running && priv->roc.in_progress &&
			(pdesc->channel == priv->roc.chan))) {
		dev_kfree_skb_any(prx_skb);
//		wiphy_debug(priv->hw->wiphy,
//			"<= %s(), not accepted channel (%d, %d)\n", __func__,
//			pdesc->channel, hw->conf.chandef.chan->hw_value);
		return;
	}

	mwl_rx_prepare_status(pdesc, &status);
	priv->noise = pdesc->noise_floor;

	skb_pull(prx_skb, sizeof(struct mwl_rx_desc));
	dma = (struct mwl_dma_data *)prx_skb->data;
	wh = &dma->wh;

	if (ieee80211_has_protected(wh->frame_control)) {
		/* Check if hw crypto has been enabled for
		 * this bss. If yes, set the status flags
		 * accordingly
		 */
		if (ieee80211_has_tods(wh->frame_control))
		{
			mwl_vif = mwl_rx_find_vif_bss(priv, wh->addr1);
			if (!mwl_vif && ieee80211_has_a4(wh->frame_control))
				mwl_vif = mwl_rx_find_vif_bss(priv,wh->addr2);
		}
		else if (ieee80211_has_fromds(wh->frame_control))
			mwl_vif = mwl_rx_find_vif_bss(priv, wh->addr2);
		else
			mwl_vif = mwl_rx_find_vif_bss(priv, wh->addr3);

		if  (mwl_vif && mwl_vif->is_hw_crypto_enabled) {
			/* When MMIC ERROR is encountered
			 * by the firmware, payload is
			 * dropped and only 32 bytes of
			 * mwlwifi Firmware header is sent
			 * to the host.
			 *
			 * We need to add four bytes of
			 * key information.  In it
			 * MAC80211 expects keyidx set to
			 * 0 for triggering Counter
			 * Measure of MMIC failure.
			 */
			if (status.flag & RX_FLAG_MMIC_ERROR) {
				memset((void *)&dma->data, 0, 4);
				skb_put(prx_skb, 4);
				/* IV is stripped in this case
				 * Indicate that, so mac80211 doesn't attempt to decrypt the
				 * packet and fail prior to handling the MMIC error indication
				 */
				status.flag |= RX_FLAG_IV_STRIPPED;
			}

			if (!ieee80211_is_auth(wh->frame_control))
				/* For WPA2 frames, AES header/MIC are
				 ** present to enable mac80211 to check
				 ** for replay attacks
				 */
				status.flag |= RX_FLAG_DECRYPTED |
					       RX_FLAG_MMIC_STRIPPED;
		}
	}

	/*
	    Remove the DMA header (dma->fwlen)
	*/
	mwl_rx_remove_dma_header(prx_skb, pdesc->qos_ctrl);

	/* Update the pointer of wifi header,
		which may be different after mwl_rx_remove_dma_header()
	*/
	wh = (struct ieee80211_hdr *)prx_skb->data;

#if 0 //def CONFIG_MAC80211_MESH
		if (ieee80211_is_data_qos(wh->frame_control) &&
		    ieee80211_has_a4(wh->frame_control)) {
			u8 *qc = ieee80211_get_qos_ctl(wh);

			if (*qc & IEEE80211_QOS_CTL_A_MSDU_PRESENT)
				if (mwl_rx_process_mesh_amsdu(priv, prx_skb,
							      &status))
					return;
		}
#endif
	memcpy(IEEE80211_SKB_RXCB(prx_skb), &status, sizeof(status));

	/* Packet to indicate => Will indicate AMPDU/AMSDU packets */
	mwl_rx_upload_pkt(hw, prx_skb);

	return;
}


static void mwl_usb_flush_rx_urb_queue(struct mwl_priv *priv)
{
	struct usb_card_rec *card = (struct usb_card_rec *)priv->intf;
	struct urb_context * rx_urb, * tmp;
	struct list_head local;
	unsigned long flags;

	if (list_empty(&card->rx_urb_pending_list))
		return;

	INIT_LIST_HEAD(&local);

	spin_lock_irqsave(&card->rx_urb_lock, flags);
	list_splice_init(&card->rx_urb_pending_list, &local);
	spin_unlock_irqrestore(&card->rx_urb_lock, flags);

	list_for_each_entry_safe(rx_urb, tmp, &local, list) {
		list_del_init(&rx_urb->list);
		mwl_usb_submit_rx_urb(rx_urb);
	}
}


static void mwl_usb_rx_recv(unsigned long data)
{
	struct ieee80211_hw *hw = (struct ieee80211_hw *)data;
	struct mwl_priv *priv = hw->priv;
	struct usb_card_rec *card = (struct usb_card_rec *)priv->intf;
	struct sk_buff *prx_skb = NULL;

	while (!skb_queue_empty(&card->rx_data_q)) {
		prx_skb = skb_dequeue(&card->rx_data_q);
		if (prx_skb == NULL) {
			break;
		}
		atomic_dec(&card->rx_pending);
		mwl_handle_rx_packet(priv, prx_skb);

		// If we hit the low theshold, empty the rx urb pending queue
		if (atomic_read(&card->rx_pending) == LOW_RX_PENDING)
			mwl_usb_flush_rx_urb_queue(priv);
	}

	return;
}

static int mwl_usb_init_post(struct mwl_priv *priv)
{
	// Stop Shutdown mode not supported
	priv->stop_shutdown = false;

	if (priv->mfg_mode) {
		// Assume ST 60 with one interface
		priv->radio_caps.capability = 0;
		priv->radio_caps.num_mac = 1;
	}

	return 0;
}

static int mwl_usb_init(struct mwl_priv *priv)
{
	struct usb_card_rec *card = (struct usb_card_rec *)priv->intf;
	int num;

	card->priv    = priv;
	priv->host_if = MWL_IF_USB;

	priv->dev       = &card->udev->dev;
	priv->chip_type = card->chip_type;
	priv->pcmd_buf  = kzalloc(CMD_BUF_SIZE, GFP_KERNEL);

	if (!priv->pcmd_buf) {
		wiphy_err(priv->hw->wiphy,
				  "%s: cannot alloc memory for command buffer\n",
				  MWL_DRV_NAME);
		return -ENOMEM;
	}

	dev_set_drvdata(&card->udev->dev, priv->hw);

	wiphy_debug(priv->hw->wiphy, "priv->pcmd_buf = %p\n", priv->pcmd_buf);

	memset(priv->pcmd_buf, 0x00, CMD_BUF_SIZE);
	init_waitqueue_head(&card->cmd_wait_q.wait);
	card->cmd_wait_q.status = 0;
	card->cmd_resp_recvd = false;
	card->cmd_id = 0;

	if (priv->mfg_mode) {
		return 0;
	}

	skb_queue_head_init(&card->rx_data_q);

	/* Init the tasklet first in case there are tx/rx interrupts */
	tasklet_init(&priv->rx_task, (void *)mwl_usb_rx_recv, (unsigned long)priv->hw);
	tasklet_disable(&priv->rx_task);

	for (num = 0; num < SYSADPT_NUM_OF_DESC_DATA; num++)
		skb_queue_head_init(&priv->txq[num]);

	return 0;
}

static int mwl_usb_register_dev(struct mwl_priv *priv)
{
	struct usb_card_rec *card = (struct usb_card_rec *)priv->intf;

	card->priv = priv;

	priv->if_ops.ptx_task = &card->tx_task;
	tasklet_init(priv->if_ops.ptx_task, (void *)mwl_tx_skbs, (unsigned long)priv->hw);
	tasklet_disable(priv->if_ops.ptx_task);

	return 0;
}

/* This function handles received packet. Necessary action is taken based on
 * cmd/event/data.
 */
static int mwl_usb_recv(struct mwl_priv *adapter,
			    struct sk_buff *skb, u8 ep)
{
	u32 recv_type;
	struct usb_card_rec *card = (struct usb_card_rec *)adapter->intf;
	struct sk_buff *newskb;
	struct cmd_header cmd_hdr_tmp;
	__le32 tmp;

	/*TODO if (adapter->hs_activated)
		mwifiex_process_hs_config(adapter);*/

	if (skb->len < INTF_HEADER_LEN) {
		wiphy_err(adapter->hw->wiphy, "%s: invalid skb->len\n", __func__);
		return -1;
	}

	switch (ep) {
	case MWIFIEX_USB_EP_CMD_EVENT:
		skb_copy_from_linear_data(skb, &tmp, INTF_HEADER_LEN);
		recv_type = le32_to_cpu(tmp);

		switch (recv_type) {
		case MWIFIEX_USB_TYPE_CMD:
			skb_copy_from_linear_data_offset(skb, INTF_HEADER_LEN, &cmd_hdr_tmp, sizeof(cmd_hdr_tmp));

			if ((!card->cmd_resp_recvd) &&
			    (le16_to_cpu(cmd_hdr_tmp.command) == (card->cmd_id | HOSTCMD_RESP_BIT))) {
				skb_copy_from_linear_data(skb, adapter->pcmd_buf, skb->len);
				card->cmd_wait_q.status = 0;
				card->cmd_resp_recvd = true;
				card->cmd_id = 0;
				wake_up(&card->cmd_wait_q.wait);
			}
			break;

		case MWIFIEX_USB_TYPE_EVENT:
			if (skb->len < sizeof(u32)) {
				wiphy_err(adapter->hw->wiphy, "EVENT: skb->len too small\n");
				return -1;
			}
			skb_copy_from_linear_data(skb, &tmp, sizeof(u32));
			wiphy_debug(adapter->hw->wiphy, "event_cause %#x\n", le32_to_cpu(tmp));
			break;
		default:
			wiphy_err(adapter->hw->wiphy, "unknown recv_type %#x\n", recv_type);
			return -1;
		}
		break;
	case MWIFIEX_USB_EP_DATA:
		if (skb->len > MWIFIEX_RX_DATA_BUF_SIZE) {
			wiphy_err(adapter->hw->wiphy, "DATA: skb->len too large\n");
			return -1;
		}
		newskb = dev_alloc_skb(skb->len);
		if (!newskb) {
			wiphy_err(adapter->hw->wiphy, "DATA: skb allocation failure!\n");
			return -1;
		}
		skb_put_data(newskb, skb->data, skb->len);
		skb_pull(newskb, INTF_HEADER_LEN);
		skb_queue_tail(&card->rx_data_q, newskb);
		atomic_inc(&card->rx_pending);
		tasklet_schedule(&adapter->rx_task);
		break;
	default:
		wiphy_err(adapter->hw->wiphy, "%s: unknown endport %#x\n", __func__, ep);
		return -1;
	}

	return -EINPROGRESS;
}


static void  mwl_usb_cleanup(struct mwl_priv *adapter)
{

}


/*
 * Packet send completion callback handler.
 *
 * It either frees the buffer directly or forwards it to another
 * completion callback which checks conditions, updates statistics,
 * wakes up stalled traffic queue if required, and then frees the buffer.
 */
static int mwl_write_data_complete(struct mwl_priv *priv,
				struct sk_buff *skb)
{
	struct ieee80211_hw *hw = (struct ieee80211_hw *)priv->hw;
	struct mwl_tx_ctrl *tx_ctrl;
	struct ieee80211_tx_info *info;
	struct sk_buff_head *amsdu_pkts;
	struct mwl_dma_data *dma_data;
	struct ieee80211_hdr *wh;
	u8 *data = skb->data;
	u32 rate;

	if (skb == NULL)
		return 0;
	dma_data = (struct mwl_dma_data *)
		&data[INTF_HEADER_LEN + sizeof(struct mwl_tx_desc)];
	wh = &dma_data->wh;
	info = IEEE80211_SKB_CB(skb);

	tx_ctrl = (struct mwl_tx_ctrl *)&info->status;

	if (ieee80211_is_data(wh->frame_control) ||
		ieee80211_is_data_qos(wh->frame_control)) {
		rate = TX_COMP_RATE_FOR_DATA;
		tx_ctrl = (struct mwl_tx_ctrl *)&info->status;
		amsdu_pkts = (struct sk_buff_head *)
					tx_ctrl->amsdu_pkts;
		if (amsdu_pkts) {
			mwl_tx_ack_amsdu_pkts(hw, rate, amsdu_pkts);
			dev_kfree_skb_any(skb);
			skb = NULL;
		} else
			mwl_tx_prepare_info(hw, rate, info);
	 } else
			mwl_tx_prepare_info(hw, 0, info);

	if (skb != NULL) {
		info->flags &= ~IEEE80211_TX_CTL_AMPDU;
		info->flags |= IEEE80211_TX_STAT_ACK;

		if (ieee80211_is_data(wh->frame_control) ||
		ieee80211_is_data_qos(wh->frame_control)) {
//			wiphy_err(hw->wiphy, "fr_data_skb=%p\n", skb);
		}
		// USB completion handlers are sometimes called in irq context
		ieee80211_tx_status_irqsafe(hw, skb);
	}
	return 0;
}


static void mwl_usb_tx_complete(struct urb *urb)
{
	struct urb_context *context = (struct urb_context *)(urb->context);
	struct mwl_priv *priv = context->priv;
	struct usb_card_rec *card = (struct usb_card_rec *)priv->intf;
	struct usb_tx_data_port *port;

	if (context->ep != card->tx_cmd_ep) {
		if (urb->status == -ENOENT) {
			// urb was canceled, free the tx skb
			ieee80211_free_txskb(priv->hw, context->skb);
		}
		else
			mwl_write_data_complete(priv, context->skb);

		port = &card->port;
		atomic_dec(&port->tx_data_urb_pending);

		if (!priv->recovery_in_progress)
			tasklet_schedule(priv->if_ops.ptx_task);
	}

	return;
}

static int mwl_usb_tx_init(struct mwl_priv *adapter)
{
	struct usb_card_rec *card = (struct usb_card_rec *)adapter->intf;
	struct usb_tx_data_port *port;
	int i;

	card->tx_cmd.priv = adapter;
	card->tx_cmd.ep = card->tx_cmd_ep;

	card->tx_cmd.urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!card->tx_cmd.urb)
		return -ENOMEM;

	if(unlikely(adapter->mfg_mode))
		return 0;

	port = &card->port;

	if (!port->tx_data_ep)
		return -1;

	port->tx_data_ix = 0;

	for (i = 0; i < MWIFIEX_TX_DATA_URB; i++) {
		port->tx_data_list[i].priv = adapter;
		port->tx_data_list[i].ep = port->tx_data_ep;
		port->tx_data_list[i].urb = usb_alloc_urb(0, GFP_KERNEL);
		if (!port->tx_data_list[i].urb)
			return -ENOMEM;
	}

	return 0;
}

static void mwl_usb_rx_complete(struct urb *urb)
{
	struct urb_context *context = (struct urb_context *)urb->context;
	struct mwl_priv *adapter = context->priv;
	struct sk_buff *skb = context->skb;
	struct usb_card_rec *card;
	int recv_length = urb->actual_length;
	int status;
	unsigned long flags;

	if (!adapter || !adapter->intf) {
		pr_err("mwl adapter or card structure is not valid\n");
		return;
	}

	card = (struct usb_card_rec *)adapter->intf;

	if (adapter->recovery_in_progress)
		return;

	if ((urb->status) || !recv_length) {
		// Do not print when urb is killed or unlinked or recovery in progress
		if (!adapter->recovery_in_progress) {
			if (urb->status && (urb->status != -ENOENT) && (urb->status != -ECONNRESET))
				pr_err("%s: %s URB failed! status %d, length %d\n", __func__,
					card->rx_cmd_ep == context->ep ? "command" : "data",
					urb->status, recv_length);
		}

		goto setup_for_next;
	}

	if (recv_length) {
		if (skb->len > recv_length)
			skb_trim(skb, recv_length);
		else
			skb_put(skb, recv_length - skb->len);

	 	status = mwl_usb_recv(adapter, skb, context->ep);

		if (status != -EINPROGRESS) {
			if (status == -1)
				pr_err("received data processing failed!\n");
		}
	}

setup_for_next:

	switch(urb->status) {
	case -ENOENT:
	case -ECONNRESET:
	case -ENODEV:
	case -ESHUTDOWN:
		goto done;
	default:
		break;
	}

	if (card->dying)
		goto done;

	if (atomic_read(&card->rx_pending) > HIGH_RX_PENDING) {
		spin_lock_irqsave(&card->rx_urb_lock, flags);
		list_add_tail(&context->list, &card->rx_urb_pending_list);
		spin_unlock_irqrestore(&card->rx_urb_lock, flags);
	}
	else
		mwl_usb_submit_rx_urb(context);

done:
	return;
}

static int mwl_usb_submit_rx_urb(struct urb_context *ctx)
{
	struct mwl_priv *adapter = ctx->priv;
	struct usb_card_rec *card = (struct usb_card_rec *)adapter->intf;
	int size;

	if (card->rx_data_ep == ctx->ep)
		size = MWIFIEX_RX_DATA_BUF_SIZE;
	else
		size = MWIFIEX_RX_CMD_BUF_SIZE;

	if (card->rx_cmd_ep == ctx->ep &&
		card->rx_cmd_ep_type == USB_ENDPOINT_XFER_INT)
	{
		usb_fill_int_urb(ctx->urb, card->udev,
		                 usb_rcvintpipe(card->udev, ctx->ep),
		                 ctx->skb->data, size, mwl_usb_rx_complete,
		                 (void *)ctx, card->rx_cmd_interval);
	}
	else
	{
		usb_fill_bulk_urb(ctx->urb, card->udev,
		                  usb_rcvbulkpipe(card->udev, ctx->ep),
		                  ctx->skb->data, size, mwl_usb_rx_complete,
		                  (void *)ctx);
	}

	if (card->rx_data_ep == ctx->ep)
		usb_anchor_urb(ctx->urb, &card->rx_urb_anchor);
		
	if (usb_submit_urb(ctx->urb, GFP_ATOMIC)) {
		wiphy_err(adapter->hw->wiphy, "usb_submit_urb failed\n");
		if (card->rx_data_ep == ctx->ep)
			usb_unanchor_urb(ctx->urb);
		return -1;
	}

	return 0;
}

static int mwl_usb_rx_init(struct mwl_priv *adapter)
{
	struct usb_card_rec *card = (struct usb_card_rec *)adapter->intf;
	int i;

	card->rx_cmd.priv = adapter;
	card->rx_cmd.ep = card->rx_cmd_ep;

	card->rx_cmd.urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!card->rx_cmd.urb)
		return -ENOMEM;

	card->rx_cmd.skb = dev_alloc_skb(MWIFIEX_RX_CMD_BUF_SIZE);
	if (!card->rx_cmd.skb)
		return -ENOMEM;

	if (mwl_usb_submit_rx_urb(&card->rx_cmd))
		return -1;

	if(unlikely(adapter->mfg_mode))
		return 0;

	INIT_LIST_HEAD(&card->rx_urb_pending_list);
	spin_lock_init(&card->rx_urb_lock);
	init_usb_anchor(&card->rx_urb_anchor);

	for (i = 0; i < MWIFIEX_RX_DATA_URB; i++) {
		card->rx_data_list[i].priv = adapter;
		card->rx_data_list[i].ep   = card->rx_data_ep;
		card->rx_data_list[i].urb  = usb_alloc_urb(0, GFP_KERNEL);

		if (!card->rx_data_list[i].urb)
			return -1;

		card->rx_data_list[i].skb = dev_alloc_skb(MWIFIEX_RX_DATA_BUF_SIZE);
		if (!card->rx_data_list[i].skb) {
			wiphy_err(adapter->hw->wiphy, "%s: dev_alloc_skb failed\n", __func__);
			return -ENOMEM;
		}

		if (mwl_usb_submit_rx_urb(&card->rx_data_list[i]))
			return -1;
	}

	return 0;
}

static int mwl_read_data_sync(struct mwl_priv *priv, u8 *pbuf,
				  u32 *len, u8 ep, u32 timeout)
{
	struct usb_card_rec *card = priv->intf;
	int actual_length, ret;

	/* Receive the data response */
	ret = usb_bulk_msg(card->udev, usb_rcvbulkpipe(card->udev, ep), pbuf,
			   *len, &actual_length, timeout);
	if (ret) {
		wiphy_err(priv->hw->wiphy, "usb_bulk_msg for rx failed: %d\n", ret);
		return ret;
	}

	*len = actual_length;

	return ret;
}

static int mwl_write_data_sync(struct mwl_priv *priv, u8 *pbuf,
				   u32 *len, u8 ep, u32 timeout)
{
	struct usb_card_rec *card = priv->intf;
	int actual_length, ret;

	if (!(*len % card->bulk_out_maxpktsize))
		(*len)++;

	/* Send the data block */
	ret = usb_bulk_msg(card->udev, usb_sndbulkpipe(card->udev, ep), pbuf,
			   *len, &actual_length, timeout);
	if (ret) {
		wiphy_err(priv->hw->wiphy,"usb_bulk_msg for tx failed: %d\n", ret);
		return ret;
	}

	*len = actual_length;

	return ret;
}

static int mwl_prog_fw_w_helper(struct mwl_priv * priv)
{
	int ret = 0;
	const u8 *firmware = priv->fw_ucode->data;
	u8 *recv_buff = NULL;
	u32 retries = USB8XXX_FW_MAX_RETRY + 1;
	u32 dlen;
	u32 fw_seqnum = 0, tlen = 0, dnld_cmd = 0;
	struct fw_data *fwdata = NULL;
	struct fw_sync_header sync_fw;
	u8 check_winner = 1;

	if (!firmware) {
		wiphy_err(priv->hw->wiphy, "No firmware image found! Terminating download\n");
		ret = -1;
		goto fw_exit;
	}

	/* Allocate memory for transmit */
	fwdata = kzalloc(FW_DNLD_TX_BUF_SIZE, GFP_KERNEL);
	if (!fwdata) {
		ret = -ENOMEM;
		goto fw_exit;
	}

	/* Allocate memory for receive */
	recv_buff = kzalloc(FW_DNLD_RX_BUF_SIZE, GFP_KERNEL);
	if (!recv_buff) {
		ret = -ENOMEM;
		goto cleanup;
	}

	do {
		/* Send pseudo data to check winner status first */
		if (check_winner) {
			memset(&fwdata->fw_hdr, 0, sizeof(struct fw_header));
			dlen = 0;
		} else {
			/* copy the header of the fw_data to get the length */
			memcpy(&fwdata->fw_hdr, &firmware[tlen], sizeof(struct fw_header));

			dlen = le32_to_cpu(fwdata->fw_hdr.data_len);
			dnld_cmd = le32_to_cpu(fwdata->fw_hdr.dnld_cmd);
			tlen += sizeof(struct fw_header);

			/* Command 7 doesn't have data length field */
			if (dnld_cmd == FW_CMD_7)
				dlen = 0;

			if (dlen > (FW_DNLD_TX_BUF_SIZE - (sizeof(struct fw_data) - 1)))
			{
				pr_err("Firmware data length chunk (%d bytes) too large!\n", dlen);
				ret = -1;
				goto cleanup;
			}

			memcpy(fwdata->data, &firmware[tlen], dlen);

			fwdata->seq_num = cpu_to_le32(fw_seqnum);
			tlen += dlen;
		}

		/* If the send/receive fails or CRC occurs then retry */
		while (--retries) {
			u8 *buf = (u8 *)fwdata;
			u32 len = FW_DATA_XMIT_SIZE;

			/* send the firmware block */
			ret = mwl_write_data_sync(priv, buf, &len,
						MWIFIEX_USB_EP_CMD_EVENT,
						MWIFIEX_USB_TIMEOUT);
			if (ret) {
				wiphy_err(priv->hw->wiphy, "write_data_sync: failed: %d\n", ret);
				continue;
			}

			buf = recv_buff;
			len = FW_DNLD_RX_BUF_SIZE;

			/* Receive the firmware block response */
			ret = mwl_read_data_sync(priv, buf, &len,
						MWIFIEX_USB_EP_CMD_EVENT,
						MWIFIEX_USB_TIMEOUT);
			if (ret) {
				wiphy_err(priv->hw->wiphy, "read_data_sync: failed: %d\n", ret);
				continue;
			}

			memcpy(&sync_fw, recv_buff,
			       sizeof(struct fw_sync_header));

			/* check 1st firmware block resp for highest bit set */
			if (check_winner) {
				if (le32_to_cpu(sync_fw.cmd) & 0x80000000) {
					wiphy_info(priv->hw->wiphy, "USB is not the winner %#x\n", sync_fw.cmd);

					/* returning success */
					ret = 0;
					goto cleanup;
				}

				wiphy_info(priv->hw->wiphy, "start to download FW...\n");

				check_winner = 0;
				break;
			}

			/* check the firmware block response for CRC errors */
			if (sync_fw.cmd) {
				wiphy_err(priv->hw->wiphy, "FW received block with CRC %#x\n", sync_fw.cmd);
				ret = -1;
				continue;
			}

			retries = USB8XXX_FW_MAX_RETRY + 1;
			break;
		}
		fw_seqnum++;
	} while ((dnld_cmd != FW_HAS_LAST_BLOCK) && retries);

cleanup:
	wiphy_info(priv->hw->wiphy, "info: FW download over, size %d bytes, ret %d\n", tlen, ret);

	if (recv_buff)
		kfree(recv_buff);

	if (fwdata)
		kfree(fwdata);

fw_exit:
	return ret;
}


static int mwl_usb_dnld_fw(struct mwl_priv *priv)
{
	int ret;
	struct usb_card_rec *card = (struct usb_card_rec *)priv->intf;

	if (card->usb_boot_state == USB8XXX_FW_DNLD) {
		ret = mwl_prog_fw_w_helper(priv);
		if (ret)  {
			return -1;
		}

		/* Boot state changes after successful firmware download */
		wiphy_info(priv->hw->wiphy, "Firmware download complete, port will reset with new interface...\n");
		return -EINPROGRESS;
	}
	else
		wiphy_info(priv->hw->wiphy, "Skipping FW download, continuing with initialization...\n");

	ret = mwl_usb_rx_init(priv);
	if (!ret) {
		ret = mwl_usb_tx_init(priv);
	}

	return ret;
}

static int mwl_usb_send_cmd(struct mwl_priv * priv)
{
	struct usb_card_rec *card = (struct usb_card_rec *)priv->intf;
	struct urb_context *context = NULL;
	int len;
	int ret;
	struct urb *tx_urb;

	__le32 *pbuf = (__le32 *)priv->pcmd_buf;
	struct cmd_header *cmd_hdr = (struct cmd_header *)&priv->pcmd_buf[
                INTF_CMDHEADER_LEN(INTF_HEADER_LEN)];

/* TODO if (card->is_suspended) {
			wiphy_err(priv->hw->wiphy, "%s: not allowed while suspended\n", __func__);
			return -1;
		}
*/
/* TODO if (adapter->surprise_removed) {
			wiphy_err(priv->hw->wiphy, "%s: device removed\n", __func__);
			return -1;
		}
*/

	// Single context available for tx command urb
	// Commands are synchronous and protected by fwcmd mutex
	context = &card->tx_cmd;
	tx_urb = context->urb;
	len = le16_to_cpu(cmd_hdr->len)  +
		INTF_CMDHEADER_LEN(INTF_HEADER_LEN)*sizeof(unsigned short);

	*pbuf=cpu_to_le32(MWIFIEX_USB_TYPE_CMD);

	if (card->tx_cmd_ep_type == USB_ENDPOINT_XFER_INT)
	{
		usb_fill_int_urb(tx_urb, card->udev,
		                 usb_sndintpipe(card->udev,card->tx_cmd_ep),
		                 priv->pcmd_buf,
		                 len, mwl_usb_tx_complete,
		                 (void *)context, card->tx_cmd_interval);
	}
	else
		usb_fill_bulk_urb(tx_urb, card->udev,
		                  usb_sndbulkpipe(card->udev, card->tx_cmd_ep),
		                  priv->pcmd_buf, len,
		                  mwl_usb_tx_complete, (void *)context);

	tx_urb->transfer_flags |= URB_ZERO_PACKET;
	card->cmd_resp_recvd = false;
	card->cmd_id = (u16)(le16_to_cpu(cmd_hdr->command) & ~HOSTCMD_RESP_BIT);

	ret = usb_submit_urb(tx_urb, GFP_ATOMIC);
	if (ret) {
		wiphy_err(priv->hw->wiphy, "%s: usb_submit_urb failed\n", __func__);
	}

	return ret;
}

static int mwl_usb_cmd_resp_wait_completed(struct mwl_priv *priv,
        unsigned short cmd)
{
	struct usb_card_rec *card = (struct usb_card_rec *)priv->intf;
	int status;

	/* Wait for completion */
	status = wait_event_timeout(card->cmd_wait_q.wait,
				  (card->cmd_resp_recvd == true),
				  (12 * HZ));
	if (status == 0) {
		wiphy_err(priv->hw->wiphy, "timeout, cmd_wait_q terminated: %d\n", status);

		card->cmd_wait_q.status = -ETIMEDOUT;
	}

	return card->cmd_wait_q.status;
}


/*
 * Adds TxPD to AMSDU header.
 *
 * Each AMSDU packet will contain one TxPD at the beginning,
 * followed by multiple AMSDU subframes.
 */
static int
mwl_process_txdesc(struct mwl_priv *priv,
			    struct sk_buff *skb)
{
	struct mwl_tx_desc *tx_desc;
	struct mwl_tx_ctrl *tx_ctrl;
	struct ieee80211_tx_info *tx_info;
	u8 *ptr;
	u32* head;
	int size_needed;

	tx_info = IEEE80211_SKB_CB(skb);
	tx_ctrl = (struct mwl_tx_ctrl *)&IEEE80211_SKB_CB(skb)->status;

	size_needed = sizeof(struct mwl_tx_desc) + INTF_HEADER_LEN;

	/* existing pointers into skb buffer may not be valid after this call */
	if (skb_cow_head(skb, size_needed)) {
		WARN_ON(skb_headroom(skb) < (size_needed));
		return -ENOMEM;
	}

	ptr = (u8 *)skb->data;

	skb_push(skb, sizeof(struct mwl_tx_desc));
	tx_desc = (struct mwl_tx_desc *) skb->data;
	memset(tx_desc, 0, sizeof(struct mwl_tx_desc));

	skb_push(skb, INTF_HEADER_LEN);
	head =(u32 *)skb->data;
	*head = cpu_to_le32(MWIFIEX_USB_TYPE_DATA);
	tx_desc->tx_priority = tx_ctrl->tx_priority;
	tx_desc->qos_ctrl = cpu_to_le16(tx_ctrl->qos_ctrl);
	tx_desc->pkt_len = cpu_to_le16(skb->len);

	if (tx_info->flags & IEEE80211_TX_INTFL_DONT_ENCRYPT) {
		tx_desc->flags |= cpu_to_le32(MWL_TX_WCB_FLAGS_DONT_ENCRYPT);
	}

	if (tx_info->flags & IEEE80211_TX_CTL_NO_CCK_RATE) {
		tx_desc->flags |= cpu_to_le32(MWL_TX_WCB_FLAGS_NO_CCK_RATE);
	}

	tx_desc->packet_info = 0;
	tx_desc->data_rate = 0;
	tx_desc->type = tx_ctrl->type;
	tx_desc->xmit_control = tx_ctrl->xmit_control;
	tx_desc->sap_pkt_info = 0;
	tx_desc->pkt_ptr = cpu_to_le32((u8 *)skb->data - ptr);
	tx_desc->status = 0;

	return 0;
}

static int mwl_usb_host_to_card(struct mwl_priv *priv, int desc_num,
                struct sk_buff *tx_skb)
{
	struct urb_context *context = NULL;
	struct usb_tx_data_port *port = NULL;
	u8  ep = MWIFIEX_USB_EP_DATA;
	struct urb *tx_urb;
	int ret;

	struct usb_card_rec *card =  (struct usb_card_rec *) (priv->intf);

/* TODO if (card->is_suspended) {
			wiphy_err(priv->hw->wiphy, "%s: not allowed while suspended\n", __func__);
			return -1;
		}
*/
/* TODO if (adapter->surprise_removed) {
			wiphy_err(priv->hw->wiphy, "%s: device removed\n", __func__);
			return -1;
		}
*/

	if (priv->recovery_in_progress) {
		ret = -1;
		goto done;
	}

	/* Existing pointers into skb buffer are not valid after this call */
	ret = mwl_process_txdesc(priv,tx_skb);
	if (ret) {
		wiphy_err(priv->hw->wiphy, "%s: Failed to send packet! ret = %d\n", __func__, ret);
		dev_kfree_skb_any(tx_skb);
		goto done;
	}

	port = &card->port;
	if (atomic_read(&port->tx_data_urb_pending) >= MWIFIEX_TX_DATA_URB) {
		ret = -EBUSY;
		goto done;
	}
	if (port->tx_data_ix >= MWIFIEX_TX_DATA_URB)
		port->tx_data_ix = 0;
	context = &port->tx_data_list[port->tx_data_ix++];

	context->priv = priv;
	context->ep = ep;
	context->skb = tx_skb;
	tx_urb = context->urb;

	usb_fill_bulk_urb(tx_urb, card->udev, usb_sndbulkpipe(card->udev, ep),
					  tx_skb->data, tx_skb->len, mwl_usb_tx_complete,
					  (void *)context);

	tx_urb->transfer_flags |= URB_ZERO_PACKET;

	atomic_inc(&port->tx_data_urb_pending);

	if (usb_submit_urb(tx_urb, GFP_ATOMIC)) {
		wiphy_err(priv->hw->wiphy, "%s: usb_submit_urb failed\n", __func__);
		atomic_dec(&port->tx_data_urb_pending);
		if (port->tx_data_ix)
			port->tx_data_ix--;
		else
			port->tx_data_ix = MWIFIEX_TX_DATA_URB;

		ret =-1;
		goto done;
	}

	ret = -EINPROGRESS;

done:
	return ret;
}

static bool mwl_usb_is_tx_available(struct mwl_priv *priv, int desc_num)
{
	struct usb_tx_data_port *port = NULL;
	struct usb_card_rec *card =  (struct usb_card_rec *) (priv->intf);

	if (priv->recovery_in_progress)
		return false;

	port = &card->port;
	if (atomic_read(&port->tx_data_urb_pending) >= MWIFIEX_TX_DATA_URB) {
		return false;
	}

	return true;
}

static void mwl_usb_enter_deepsleep(struct mwl_priv *priv)
{
	return;
}

static int mwl_usb_wakeup_card(struct mwl_priv *priv)
{
	return 0;
}

static int mwl_usb_is_deepsleep(struct mwl_priv * priv)
{
	return 0;
}

static int mwl_usb_restart_handler(struct mwl_priv *priv)
{
	struct usb_card_rec *card = priv->intf;

	if (!gpio_is_valid(card->reset_pwd_gpio))
		return -ENOSYS;

	gpio_set_value(card->reset_pwd_gpio, 0);
	msleep(USB_DEFAULT_POWERDOWN_DELAY_MS);
	gpio_set_value(card->reset_pwd_gpio, 1);

	return 0;
}

#ifdef CONFIG_DEBUG_FS
static int mwl_usb_dbg_info (struct mwl_priv *priv, char *p, int size, int len)
{
	struct usb_card_rec *card = priv->intf;

	if (gpio_is_valid(card->reset_pwd_gpio)) {
		len += scnprintf(p + len, size - len, "PMU_EN gpio: %d\n", card->reset_pwd_gpio);
	}
	return len;
}
#endif


static struct mwl_if_ops usb_ops1 = {
	.inttf_head_len    = INTF_HEADER_LEN,
	.register_dev      = mwl_usb_register_dev,
	.cleanup_if        = mwl_usb_cleanup,
	.prog_fw           = mwl_usb_dnld_fw,
	.init_if           = mwl_usb_init,
	.init_if_post      = mwl_usb_init_post,
	.check_card_status = mwl_usb_check_card_status,
	.send_cmd          = mwl_usb_send_cmd,
	.cmd_resp_wait_completed = mwl_usb_cmd_resp_wait_completed,
	.host_to_card      = mwl_usb_host_to_card,
	.is_tx_available   = mwl_usb_is_tx_available,
	//Tasklets are assigned per instance during device registration
	.ptx_task          = NULL,
	.enter_deepsleep   = mwl_usb_enter_deepsleep,
	.wakeup_card       = mwl_usb_wakeup_card,
	.is_deepsleep      = mwl_usb_is_deepsleep,
#ifdef CONFIG_DEBUG_FS
	.dbg_info          = mwl_usb_dbg_info,
#endif
};

module_usb_driver(mwl_usb_driver);

#ifdef CONFIG_GPIOLIB
module_param(reset_pwd_gpio, uint, 0644);
MODULE_PARM_DESC(reset_pwd_gpio, "WIFI CHIP_PWD reset pin GPIO (deprecated)");
#endif


MODULE_AUTHOR(LRD_AUTHOR);
MODULE_DESCRIPTION(LRD_USB_DESC);
MODULE_VERSION(LRD_USB_VERSION);
MODULE_LICENSE("GPL v2");
