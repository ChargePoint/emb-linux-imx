/*
 * Marvell Wireless LAN device driver: SDIO specific handling
 *
 * Copyright (C) 2011-2014, Marvell International Ltd.
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
#include <linux/module.h>
#include <linux/of.h>
#include <linux/firmware.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include "sysadpt.h"
#include "dev.h"
#include "main.h"
#include "fwcmd.h"
#include "rx.h"
#include "tx.h"
#include "sdio.h"
#include "hostcmd.h"

#define MMC_STATE_SUSPENDED	(1<<5)		/* card is suspended */
#define mmc_card_clr_suspended(c) ((c)->state &= ~MMC_STATE_SUSPENDED)

#define MWL_SDIODRV_VERSION  "10.3.0.16-20160105"
#define LRD_SDIO_VERSION     LRD_BLD_VERSION "-" MWL_SDIODRV_VERSION
#define LRD_SDIO_DESC        "Laird 60 Series Wireless SDIO Network Driver"

#define INTF_HEADER_LEN         4

enum mwl_pm_action {
	MWL_PM_PREPARE,
	MWL_PM_COMPLETE,
	MWL_PM_RESUME,
	MWL_PM_RESUME_EARLY,
	MWL_PM_RESTORE_EARLY,
	MWL_PM_SUSPEND,
	MWL_PM_SUSPEND_LATE,
	MWL_PM_FREEZE_LATE,
	MWL_PM_POWEROFF
};

static struct mwl_chip_info mwl_chip_tbl[] = {
	[MWL8864] = {
		.part_name	= "88W8864",
		.fw_image	= MWL_FW_ROOT"/88W8864_sdio.bin",
		.antenna_tx	= ANTENNA_TX_4_AUTO,
		.antenna_rx	= ANTENNA_RX_4_AUTO,
	},
	[MWL8897] = {
		.part_name	= "88W8897",
		.fw_image	= MWL_FW_ROOT"/88W8897_sdio.bin",
		.antenna_tx	= ANTENNA_TX_2,
		.antenna_rx	= ANTENNA_RX_2,
	},
	[MWL8997] = {
		.part_name	= "88W8997",
		.fw_image	= MWL_FW_ROOT"/88W8997_sdio.bin",
		.mfg_image	= MWL_FW_ROOT"/88W8997_sdio_mfg.bin",
		.antenna_tx	= ANTENNA_TX_2,
		.antenna_rx	= ANTENNA_RX_2,
	},
};

static unsigned int reset_pwd_gpio = ARCH_NR_GPIOS;

static int mwl_write_data_sync(struct mwl_priv *priv,
			u8 *buffer, u32 pkt_len, u32 port);
static int mwl_sdio_enable_int(struct mwl_priv *priv, bool enable);

static int mwl_sdio_event(struct mwl_priv *priv);
static void mwl_sdio_tx_workq(struct work_struct *work);
static void mwl_sdio_rx_recv(unsigned long data);
static int mwl_sdio_read_fw_status(struct mwl_priv *priv, u16 *dat);

static int mwl_sdio_reset(struct mwl_priv *priv);
static int mwl_sdio_set_gpio(struct mwl_sdio_card *card, int value);
static int mwl_sdio_restart_handler(struct mwl_priv *priv);

/* Device ID for SD8897 */
#define SDIO_DEVICE_ID_MARVELL_8897   (0x912d)
#define SDIO_DEVICE_ID_MARVELL_8997   (0x9141)

/* 88W8997 datasheet requires PMIC_EN/PMU_EN to remain de-asserted for a minimum of 100ms */
#define SDIO_DEFAULT_POWERDOWN_DELAY_MS		100

static void
mwl_sdio_interrupt(struct sdio_func *func);

/* WLAN IDs */
static const struct sdio_device_id mwl_sdio_id_tbl[] = {
	{SDIO_DEVICE(SDIO_VENDOR_ID_MARVELL, SDIO_DEVICE_ID_MARVELL_8997),
		.driver_data = MWL8997},
	{ },
};

static const struct of_device_id mwl_sdio_of_match_table[] = {
	{ .compatible = "marvell,sd8997" },
	{ }
};

/*
 * This function reads data from SDIO card register.
 */
inline int mwl_read_reg(struct mwl_sdio_card *card, u32 reg, u8 *data)
{
	int rc = -1;

	*data = sdio_readb(card->func, reg, &rc);

	return rc;
}

/*
 * This function writes data into SDIO card register.
 */
inline int mwl_write_reg(struct mwl_sdio_card *card, u32 reg, u8 data)
{
	int rc = -1;

	sdio_writeb(card->func, data, reg, &rc);

	return rc;
}


static void mwl_free_sdio_mpa_buffers(struct mwl_priv *priv)
{
	struct mwl_sdio_card *card = priv->intf;

	kfree(card->mpa_tx.buf);
	card->mpa_tx.buf = NULL;
	card->mpa_tx.buf_size = 0;

	kfree(card->mpa_rx.buf);
	card->mpa_rx.buf = NULL;
	card->mpa_rx.buf_size = 0;
}


/*
 * This function allocates the MPA Tx and Rx buffers.
 */
static int mwl_alloc_sdio_mpa_buffers(struct mwl_priv *priv,
				   u32 mpa_tx_buf_size, u32 mpa_rx_buf_size)
{
	struct mwl_sdio_card *card = priv->intf;
	u32 rx_buf_size;

	card->mpa_tx.buf = kzalloc(mpa_tx_buf_size, GFP_KERNEL);
	if (!card->mpa_tx.buf)
		goto error;

	card->mpa_tx.buf_size = mpa_tx_buf_size;

	rx_buf_size = max_t(u32, mpa_rx_buf_size,
			    (u32)SDIO_MAX_AGGR_BUF_SIZE);
	card->mpa_rx.buf = kzalloc(rx_buf_size, GFP_KERNEL);
	if (!card->mpa_rx.buf)
		goto error;

	card->mpa_rx.buf_size = rx_buf_size;

	card->tx_pkt_unaligned_cnt = 0;

	return 0;

error:
	mwl_free_sdio_mpa_buffers(priv);
	return -ENOMEM;
}

static void *mwl_alloc_dma_align_buf(int rx_len, gfp_t flags)
{
	struct sk_buff *skb;
	int buf_len, pad;

	buf_len = rx_len + MWL_RX_HEADROOM + MWL_DMA_ALIGN_SZ;

	skb = __dev_alloc_skb(buf_len, flags);

	if (!skb)
		return NULL;

	skb_reserve(skb, MWL_RX_HEADROOM);

	pad = MWL_ALIGN_ADDR(skb->data, MWL_DMA_ALIGN_SZ) -
	      (long)skb->data;

	skb_reserve(skb, pad);

	return skb;
}

/*
 * This function polls the card status.
 */
static int
mwl_sdio_poll_card_status(struct mwl_priv *priv, u8 bits)
{
	struct mwl_sdio_card *card = priv->intf;
	u32 tries;
	u8 cs;

	for (tries = 0; tries < MAX_POLL_TRIES; tries++) {
		if (mwl_read_reg(card, card->reg->poll_reg, &cs))
			break;

		if ((cs & bits) == bits)
			return 0;

		lrdmwl_delay(10);
	}

	wiphy_err(priv->hw->wiphy,
		    "poll card status failed, tries = %d\n", tries);

	return -1;
}



/*
 * This function is used to initialize IO ports for the
 * chipsets supporting SDIO new mode eg SD8897.
 */
static int mwl_init_sdio_new_mode(struct mwl_priv *priv)
{
	struct mwl_sdio_card *card = priv->intf;
	int rc = 0;
	u8 reg;

	card->ioport = MEM_PORT;

	/* enable sdio new mode */
	if (mwl_read_reg(card, card->reg->card_cfg_2_1_reg, &reg)) {
		rc = -EIO;
		goto err_new_mode;
	}

	if (mwl_write_reg(card, card->reg->card_cfg_2_1_reg,
			      reg | CMD53_NEW_MODE)) {
		rc = -EIO;
		goto err_new_mode;
	}

	/* Configure cmd port and enable reading rx length from the register */
	if (mwl_read_reg(card, card->reg->cmd_cfg_0, &reg)) {
		rc = -EIO;
		goto err_new_mode;
	}

	if (mwl_write_reg(card, card->reg->cmd_cfg_0,
			      reg | CMD_PORT_RD_LEN_EN)) {
		rc = -EIO;
		goto err_new_mode;
	}

	/* Enable Dnld/Upld ready auto reset for cmd port after cmd53 is
	 * completed
	 */
	if (mwl_read_reg(card, card->reg->cmd_cfg_1, &reg)) {
		rc = -EIO;
		goto err_new_mode;
	}

	if (mwl_write_reg(card, card->reg->cmd_cfg_1,
			      reg | CMD_PORT_AUTO_EN)) {
		rc = -EIO;
		goto err_new_mode;
	}

err_new_mode:
	return rc;
}



/* This function initializes the IO ports.
 *
 * The following operations are performed -
 *      - Read the IO ports (0, 1 and 2)
 *      - Set host interrupt Reset-To-Read to clear
 *      - Set auto re-enable interrupt
 */
static int mwl_init_sdio_ioport(struct mwl_priv *priv)
{
	struct mwl_sdio_card *card = priv->intf;
	int rc;
	u8 reg;

	card->ioport = 0;

	rc = mwl_init_sdio_new_mode(priv);
	if (rc)
		goto cont;

	/* Read the IO port */
	rc = mwl_read_reg(card, card->reg->io_port_0_reg, &reg);
	if (rc)
		goto err_init_ioport;

	card->ioport |= (reg & 0xff);

	rc = mwl_read_reg(card, card->reg->io_port_1_reg, &reg);
	if (rc)
		goto err_init_ioport;

	card->ioport |= ((reg & 0xff) << 8);

	rc = mwl_read_reg(card, card->reg->io_port_2_reg, &reg);
	if (rc)
		goto err_init_ioport;

	card->ioport |= ((reg & 0xff) << 16);

cont:
	wiphy_info(priv->hw->wiphy, "%s: SDIO FUNC1 IO port: %#x\n",
		MWL_DRV_NAME, card->ioport);

	/* Set Host interrupt reset to read to clear */
	rc = mwl_read_reg(card, card->reg->host_int_rsr_reg, &reg);
	if (rc)
		goto err_init_ioport;

	mwl_write_reg(card, card->reg->host_int_rsr_reg,
				  reg | card->reg->sdio_int_mask);

	/* Dnld/Upld ready set to auto reset */
	rc = mwl_read_reg(card, card->reg->card_misc_cfg_reg, &reg);
	if (rc)
		goto err_init_ioport;

	mwl_write_reg(card, card->reg->card_misc_cfg_reg,
				  reg | AUTO_RE_ENABLE_INT);

err_init_ioport:
	return rc;
}

static int mwl_sdio_enable_int(struct mwl_priv *priv, bool enable)
{
	struct mwl_sdio_card *card = priv->intf;
	struct sdio_func *func = card->func;
	int ret;

	sdio_claim_host(func);
	sdio_writeb(func, card->reg->host_int_enable,
				       card->reg->host_int_mask_reg ^
					   (enable ? 0 : card->reg->host_int_mask_reg),
					   &ret);
	sdio_release_host(func);

	wiphy_dbg(priv->hw->wiphy,
			"=>%s(): %s host interrupt %s\n", __func__,
			enable ? "enable" : "disable", ret ? "failed" : "ok");

	return ret;
}

static int mwl_sdio_init_irq(struct mwl_priv *priv)
{
	struct mwl_sdio_card *card = priv->intf;
	struct sdio_func *func = card->func;
	int ret;

	wiphy_info(priv->hw->wiphy,
			    "%s, register IRQ\n", __func__);

	sdio_claim_host(func);
	/* Request the SDIO IRQ */
	ret = sdio_claim_irq(func, mwl_sdio_interrupt);
	if (ret)
		wiphy_err(priv->hw->wiphy,
			    "claim irq failed: ret=%d\n", ret);

	sdio_release_host(func);
	return ret;
}

static int mwl_sdio_init_post(struct mwl_priv *priv)
{
	struct mwl_sdio_card *card = priv->intf;
	struct sdio_func     *func = card->func;

	if (priv->stop_shutdown && (mmc_card_is_removable(func->card->host) ||
		!gpio_is_valid(card->reset_pwd_gpio))) {
		priv->stop_shutdown = false;

		dev_err(&func->dev,
			"Power down when network down mode supported only, "
			"when card set to non-removable and driver has direct control "
			"over PMU Enable GPIO\n");
	}

	if (priv->mfg_mode) {
		priv->ds_enable = DS_ENABLE_OFF;

		//Assume ST 60 with one interface
		priv->radio_caps.capability = 0;
		priv->radio_caps.num_mac = 1;
		priv->stop_shutdown = false;
	}

	return 0;
}

static int mwl_sdio_init(struct mwl_priv *priv)
{
	struct mwl_sdio_card *card = priv->intf;
	struct sdio_func     *func = card->func;
	const struct mwl_sdio_card_reg *reg = card->reg;
	int rc;
	u8 sdio_ireg;
	int num;

	/* not sure this patch is needed or not?? */
	func->card->quirks |= MMC_QUIRK_BLKSZ_FOR_BYTE_MODE;

	priv->host_if = MWL_IF_SDIO;
	card->priv = priv;
	priv->dev = &func->dev;
	sdio_set_drvdata(card->func, priv->hw);

	priv->if_ops.hardware_restart = mwl_sdio_restart_handler;

	sdio_claim_host(func);

	rc = sdio_enable_func(func);
	if (rc != 0) {
		sdio_release_host(func);
		dev_err(&func->dev, "%s: failed to enable sdio function\n", __func__);
		return rc;
	}

	/*
	 * Read the host_int_status_reg for ACK the first interrupt got
	 * from the bootloader. If we don't do this we get a interrupt
	 * as soon as we register the irq.
	 */
	mwl_read_reg(card, card->reg->host_int_status_reg, &sdio_ireg);

	/* Get SDIO ioport */
	mwl_init_sdio_ioport(priv);

	/* Set block size */
	rc = sdio_set_block_size(card->func, MWL_SDIO_BLOCK_SIZE);
	if (rc) {
		sdio_release_host(func);
		wiphy_err(priv->hw->wiphy,
			"cannot set SDIO block size rc 0x%04x\n", rc);
		return rc;
	}

	sdio_release_host(func);

	/* Initialize SDIO variables in card */
	card->mp_rd_bitmap = 0;
	card->mp_wr_bitmap = 0;
	card->curr_rd_port = reg->start_rd_port;
	card->curr_wr_port = reg->start_wr_port;

	card->mp_data_port_mask = reg->data_port_mask;

	card->mpa_tx.buf_len = 0;
	card->mpa_tx.pkt_cnt = 0;
	card->mpa_tx.start_port = 0;

	card->mpa_tx.enabled = 1;
	card->mpa_tx.pkt_aggr_limit = card->mp_agg_pkt_limit;

	card->mpa_rx.buf_len = 0;
	card->mpa_rx.pkt_cnt = 0;
	card->mpa_rx.start_port = 0;

	card->mpa_rx.enabled = 1;
	card->mpa_rx.pkt_aggr_limit = card->mp_agg_pkt_limit;

	/* Allocate buffers for SDIO MP-A */
	card->mp_regs = kzalloc(reg->max_mp_regs, GFP_KERNEL);
	if (!card->mp_regs) {
		rc = -ENOMEM;
		goto err_return;
	}

	/* Allocate skb pointer buffers */
	card->mpa_rx.skb_arr = kzalloc((sizeof(void *)) *
				       card->mp_agg_pkt_limit, GFP_KERNEL);
	if (!card->mpa_rx.skb_arr) {
		rc = -ENOMEM;
		goto err_return;
	}

	card->mpa_rx.len_arr = kzalloc(sizeof(*card->mpa_rx.len_arr) *
				       card->mp_agg_pkt_limit, GFP_KERNEL);

	if (!card->mpa_rx.len_arr) {
		rc = -ENOMEM;
		goto err_return;
	}

	rc = mwl_alloc_sdio_mpa_buffers(priv,
					     card->mp_tx_agg_buf_size,
					     card->mp_rx_agg_buf_size);

	/* Allocate 32k MPA Tx/Rx buffers if 64k memory allocation fails */
	if (rc && (card->mp_tx_agg_buf_size == MWL_MP_AGGR_BUF_SIZE_MAX ||
		    card->mp_rx_agg_buf_size == MWL_MP_AGGR_BUF_SIZE_MAX)) {
		/* Disable rx single port aggregation */
		card->host_disable_sdio_rx_aggr = true;

		rc = mwl_alloc_sdio_mpa_buffers
			(priv, MWL_MP_AGGR_BUF_SIZE_32K,
			 MWL_MP_AGGR_BUF_SIZE_32K);
		if (rc) {
			/* Disable multi port aggregation */
			card->mpa_tx.enabled = 0;
			card->mpa_rx.enabled = 0;
		}
	}

	spin_lock_init(&card->int_lock);
	spin_lock_init(&card->rx_proc_lock);
	init_waitqueue_head(&card->cmd_wait_q.wait);
	skb_queue_head_init(&card->rx_data_q);

	card->cmd_wait_q.status = 0;
	card->cmd_resp_recvd = false;
	card->cmd_id = 0;
	card->int_status = 0;
	init_waitqueue_head(&card->wait_deepsleep);

	priv->chip_type = card->chip_type;

	/* This routine is called during restart scenarios, but we keep the
	 * original pcmd_buf/pcmd_event_buf until driver unload
	 */
	if (!priv->pcmd_buf) {
		priv->pcmd_buf = kzalloc(CMD_BUF_SIZE, GFP_KERNEL);
		if (!priv->pcmd_buf) {
			wiphy_err(priv->hw->wiphy,
				  "%s: cannot alloc memory for command buffer\n",
				  MWL_DRV_NAME);
			rc = -ENOMEM;
			goto err_return;
		}
	}
	else
		memset(priv->pcmd_buf, 0x00, CMD_BUF_SIZE);

	if (!priv->pcmd_event_buf) {
		priv->pcmd_event_buf = kzalloc(CMD_BUF_SIZE, GFP_KERNEL);
		if (!priv->pcmd_event_buf) {
			wiphy_err(priv->hw->wiphy,"%s: cannot alloc memory for command_event buffer\n",
				  MWL_DRV_NAME);
			rc = -ENOMEM;
			goto err_return;
		}
	}
	else
		memset(priv->pcmd_event_buf, 0x00, CMD_BUF_SIZE);

	/* Initialize the tasklet first in case there are tx/rx interrupts */
	if (!priv->recovery_in_progress) {
		// Do not reinitialize the tasklet; it could be on the CPU queue
		// Tasklet has been disabled in mwl_mac80211_stop
		tasklet_init(&priv->rx_task, (void *)mwl_sdio_rx_recv,
			(unsigned long)priv->hw);
		tasklet_disable(&priv->rx_task);
	}

	for (num = 0; num < SYSADPT_NUM_OF_DESC_DATA; num++)
		skb_queue_head_init(&priv->txq[num]);

#ifdef CONFIG_PM
	priv->wow.capable =
		(sdio_get_host_pm_caps(func) & MMC_PM_KEEP_POWER) != 0;
#endif

	// Re-initialization needed in case of restart
	card->is_deepsleep = 0;

	card->dnld_cmd_failure = 0;
	card->is_running = true;

	return 0;

err_return:
	mwl_free_sdio_mpa_buffers(priv);
	kfree(priv->pcmd_buf); priv->pcmd_buf = NULL;
	kfree(priv->pcmd_event_buf); priv->pcmd_event_buf = NULL;
	kfree(card->mpa_rx.skb_arr); card->mpa_rx.skb_arr = NULL;
	kfree(card->mpa_rx.len_arr); card->mpa_rx.len_arr = NULL;
	kfree(card->mp_regs); card->mp_regs = NULL;

	return rc;
}

/*
 * This function sends data to the card.
 */
static int mwl_write_data_to_card(struct mwl_priv *priv,
				      u8 *payload, u32 pkt_len, u32 port)
{
	struct mwl_sdio_card *card = priv->intf;
	int i, ret;

	sdio_claim_host(card->func);

	for (i = 0; i <= MAX_WRITE_IOMEM_RETRY; ++i) {
		ret = mwl_write_data_sync(priv, payload, pkt_len, port);
		if (!ret || ret == -ENOMEDIUM || ret == -ENETDOWN)
			break;

		wiphy_err(priv->hw->wiphy,
			"host_to_card, write iomem (%d) failed: %d\n", i, ret);

		if (mwl_write_reg(card, CONFIGURATION_REG, 0x04))
			wiphy_err(priv->hw->wiphy, "write CFG reg failed\n");

		ret = -1;
	}

	sdio_release_host(card->func);

	return ret;
}

static int mwl_sdio_send_command(struct mwl_priv *priv)
{
	struct mwl_sdio_card *card = priv->intf;
	struct cmd_header *cmd_hdr = (struct cmd_header *)&priv->pcmd_buf[
		INTF_CMDHEADER_LEN(INTF_HEADER_LEN)];
	u32 buf_block_len;
	u32 blk_size;
	u16 len;
	u32 pkt_len;
	u32 port;
	int rc;
	__le16 *pbuf = (__le16 *)priv->pcmd_buf;
	int status;
	unsigned long flags;

	/* Wait till the card informs CMD_DNLD_RDY interrupt except
	 * for get HW spec command */
	if (cmd_hdr->command != cpu_to_le16(HOSTCMD_CMD_GET_HW_SPEC)) {
		status = wait_event_timeout(card->cmd_wait_q.wait,
			(card->int_status & DN_LD_CMD_PORT_HOST_INT_STATUS),
			(12 * HZ));
		if (!card->is_running)
			return -ENETDOWN;

		if (status == 0) {
			wiphy_err(priv->hw->wiphy, "CMD_DNLD failure\n");
			priv->cmd_timeout = true;
			return -1;
		} else {
			spin_lock_irqsave(&card->int_lock, flags);
			card->int_status &= ~DN_LD_CMD_PORT_HOST_INT_STATUS;
			spin_unlock_irqrestore(&card->int_lock, flags);
		}
	}

	len = le16_to_cpu(cmd_hdr->len) +
	    INTF_CMDHEADER_LEN(INTF_HEADER_LEN)*sizeof(unsigned short);
	port = CMD_PORT_SLCT;
	blk_size = MWL_SDIO_BLOCK_SIZE;
	buf_block_len = (len + blk_size - 1) / blk_size;
	pkt_len = buf_block_len * blk_size;
	card->cmd_resp_recvd = false;
	card->cmd_id = (u16)(le16_to_cpu(cmd_hdr->command) & ~HOSTCMD_RESP_BIT);

	pbuf[0] = cpu_to_le16(pkt_len);
	pbuf[1] = cpu_to_le16(MWL_TYPE_CMD);

	rc = mwl_write_data_to_card(priv, (u8 *)&priv->pcmd_buf[0],
		pkt_len, (card->ioport + port));

	if (rc < 0) {
		/* If command fails to write, reset the int status so next command can be processed. */
		spin_lock_irqsave(&card->int_lock, flags);
		card->int_status |= DN_LD_CMD_PORT_HOST_INT_STATUS;
		spin_unlock_irqrestore(&card->int_lock, flags);

		card->dnld_cmd_failure++;
		if (card->dnld_cmd_failure > MAX_DNLD_CMD_FAILURES) {
			wiphy_err(priv->hw->wiphy, "CMD_DNLD threshold failure\n");
			priv->cmd_timeout = true;
		}
	}
	else  {
		card->dnld_cmd_failure = 0;
	}

	return rc;
}


static void mwl_sdio_cleanup(struct mwl_priv *priv)
{
	int num;
	struct mwl_sdio_card *card = priv->intf;

	/* Disable Interrupt before tx/rx cleanup */
	sdio_claim_host(card->func);
	sdio_release_irq(card->func);
	sdio_release_host(card->func);
	wiphy_info(priv->hw->wiphy, "%s, unregister IRQ\n", __func__);

	/* Abandon wait on queue */
	card->is_running = false;
	card->int_status |= DN_LD_CMD_PORT_HOST_INT_STATUS;
	card->cmd_resp_recvd = true;
	wake_up_all(&card->cmd_wait_q.wait);

	/* mwl_sdio_cleanup is called both in driver shutdown and recovery scenarios
	   Only kill tasklet (which just means remove from CPU queue if it exists there) in shutdown scenario
	   Ensure tasklet is not both disabled and queued to avoid CPU hang */
	if (priv->shutdown) {
		/* delay to allow threads to exit */
		mdelay(10);
		tasklet_enable(&priv->rx_task);
		tasklet_kill(&priv->rx_task);
	}

	/* Free Tx bufs */
	for (num = 0; num < SYSADPT_NUM_OF_DESC_DATA; num++) {
		skb_queue_purge(&priv->txq[num]);
		priv->fw_desc_cnt[num] = 0;
	}

	/* Free Rx bufs */
	skb_queue_purge(&card->rx_data_q);
	mwl_free_sdio_mpa_buffers(priv);
	kfree(card->mpa_rx.skb_arr); card->mpa_rx.skb_arr = NULL;
	kfree(card->mpa_rx.len_arr); card->mpa_rx.len_arr = NULL;
	kfree(card->mp_regs); card->mp_regs = NULL;

	sdio_claim_host(card->func);
	sdio_disable_func(card->func);
	sdio_release_host(card->func);
}

static bool mwl_sdio_check_card_status(struct mwl_priv *priv)
{
	return true;
}

/*
 * This function writes multiple data into SDIO card memory.
 *
 * This does not work in suspended mode.
 */
static int
mwl_write_data_sync(struct mwl_priv *priv,
			u8 *buffer, u32 pkt_len, u32 port)
{
	struct mwl_sdio_card *card = priv->intf;
	int ret;
	u8 blk_mode  = (port & MWL_SDIO_BYTE_MODE_MASK) ? BYTE_MODE : BLOCK_MODE;
	u32 blk_size = (blk_mode == BLOCK_MODE) ? MWL_SDIO_BLOCK_SIZE : 1;
	u32 blk_cnt  = (blk_mode == BLOCK_MODE) ? (pkt_len / MWL_SDIO_BLOCK_SIZE) : pkt_len;
	u32 ioport   = (port & MWL_SDIO_IO_PORT_MASK);

	if (card->is_suspended && !priv->recovery_in_progress) {
		wiphy_err(priv->hw->wiphy,
			    "%s: not allowed while suspended\n", __func__);
		return -ENETDOWN;
	}

	ret = sdio_writesb(card->func, ioport, buffer, blk_cnt * blk_size);

	if (ret && ret != -ENOMEDIUM) {
		wiphy_err(priv->hw->wiphy," %s : sdio_writesb failed,buffer ptr = 0x%p ioport = 0x%x,"
				" size = %d, error = %d\n", __func__, buffer, ioport, blk_cnt * blk_size,ret);
	}

	return ret;
}

/*
 * This function reads the firmware status.
 */
static int
mwl_sdio_read_fw_status(struct mwl_priv *priv, u16 *dat)
{
	struct mwl_sdio_card *card = priv->intf;
	const struct mwl_sdio_card_reg *reg = card->reg;
	int rc;
	u8 fws0, fws1;

	sdio_claim_host(card->func);

	rc = mwl_read_reg(card, reg->status_reg_0, &fws0);
	if (rc)
		goto err;

	rc = mwl_read_reg(card, reg->status_reg_1, &fws1);
	if (rc)
		goto err;

	*dat = (u16) ((fws1 << 8) | fws0);

err:
	sdio_release_host(card->func);

	return rc;
}


/*
 * This function checks the firmware status in card.
 *
 * The winner interface is also determined by this function.
 */
static int mwl_check_fw_status(struct mwl_priv *priv, u32 poll_num)
{
	int ret = 0, tries;
	u16 firmware_stat = 0;

	wiphy_info(priv->hw->wiphy,"Checking fw status %d\n", poll_num);

	/* Wait for firmware initialization event */
	for (tries = 1; tries <= poll_num; tries++) {
		ret = mwl_sdio_read_fw_status(priv, &firmware_stat);
		if (ret)
		{
			if (!(tries % 10) ) {
				wiphy_err(priv->hw->wiphy, "fw read failed %d\n", ret);
			}
			continue;
		}

		if (firmware_stat == FIRMWARE_READY_SDIO) {
			ret = 0;
			wiphy_info(priv->hw->wiphy,
				"firmware is ready %d\n", tries);
			break;
		}

		if (!(tries % 10))
			wiphy_info(priv->hw->wiphy,
				"Waiting on fw status %d 0x%x\n",
				tries, firmware_stat);

		msleep(100);
		ret = -1;
	}

	return ret;
}



/*
 * This function downloads the firmware to the card.
 *
 * Firmware is downloaded to the card in blocks. Every block download
 * is tested for CRC errors, and retried a number of times before
 * returning failure.
 */
static int mwl_sdio_program_firmware(struct mwl_priv *priv)
{
	struct mwl_sdio_card *card = priv->intf;
	const struct mwl_sdio_card_reg *reg = card->reg;
	const struct firmware *fw;
	u8 *fw_data;
	u32 fw_len;
	int ret;
	u32 offset = 0;
	u8 base0, base1;
	u8 *fwbuf;
	u16 len = 0;
	u32 txlen, tx_bytes = 0, tries;
	u32 i = 0;
	u32 ioport = (card->ioport & MWL_SDIO_IO_PORT_MASK);
	u16 firmware_status = 0;

	fw = priv->fw_ucode;
	fw_len = fw->size;
	fw_data = (u8 *)fw->data;

	if (!fw_len) {
		wiphy_err(priv->hw->wiphy,
			    "Firmware image not found! Terminating download\n");
		return -1;
	}

	/* Assume that the allocated buffer is 8-byte aligned */
	fwbuf = kzalloc(MWL_UPLD_SIZE, GFP_KERNEL);
	if (!fwbuf) {
		wiphy_err(priv->hw->wiphy,"buffer allocation failed\n");
		return -ENOMEM;
	}

	wiphy_info(priv->hw->wiphy,
		    "Downloading FW image (%d bytes)\n", fw_len);

	mwl_sdio_enable_int(priv, false);

	for (i = 0; i < 2; ++i) {
		ret = mwl_sdio_read_fw_status(priv, &firmware_status);

		if (ret  == 0 && firmware_status == FIRMWARE_READY_SDIO) {
			wiphy_err(priv->hw->wiphy,
				"Firmware already initialized! Resetting radio...\n");

			if (priv->if_ops.down_dev)
				priv->if_ops.down_dev(priv);

			ret = mwl_sdio_reset(priv);

			if (priv->if_ops.up_dev)
				priv->if_ops.up_dev(priv);

			if (ret) {
				kfree(fwbuf);
				return ret;
			}
		}
		else
			break;
	}

	if (i >= 2) {
		kfree(fwbuf);
		return -1;
	}

	sdio_claim_host(card->func);

	/* Perform firmware data transfer */
	i = 0;
	for(;;) {
		/* The host polls for the DN_LD_CARD_RDY and CARD_IO_READY
		   bits */
		ret = mwl_sdio_poll_card_status(priv, CARD_IO_READY |
						    DN_LD_CARD_RDY);
		if (ret) {
			wiphy_err(priv->hw->wiphy,
				    "FW downloading \t"
				    "poll status timeout @ %d\n", offset);
			goto err_dnld;
		}

		/* More data? */
		if (offset >= fw_len)
			break;

		for (tries = 0; tries < MAX_POLL_TRIES; tries++) {
			ret = mwl_read_reg(card, reg->base_0_reg,
					       &base0);
			if (ret) {
				wiphy_err(priv->hw->wiphy,
					    "dev BASE0 register read failed:\t"
					    "base0=%#04X(%d). Terminating dnld\n",
					    base0, base0);
				goto err_dnld;
			}
			ret = mwl_read_reg(card, reg->base_1_reg,
					       &base1);
			if (ret) {
				wiphy_err(priv->hw->wiphy,
					    "dev BASE1 register read failed:\t"
					    "base1=%#04X(%d). Terminating dnld\n",
					    base1, base1);
				goto err_dnld;
			}
			len = (u16) (((base1 & 0xff) << 8) | (base0 & 0xff));

			if (len)
				break;

			lrdmwl_delay(10);
		}

		if (!len) {
			break;
		} else if (len > MWL_UPLD_SIZE) {
			wiphy_err(priv->hw->wiphy,
				    "FW dnld failed @ %d, invalid length %d\n",
				    offset, len);
			ret = -1;
			goto err_dnld;
		}

		txlen = len;

		if (len & BIT(0)) {
			if (++i > MAX_WRITE_IOMEM_RETRY) {
				wiphy_err(priv->hw->wiphy,
					    "FW dnld failed @ %d, over max retry\n",
					    offset);
				ret = -1;
				goto err_dnld;
			}
			wiphy_err(priv->hw->wiphy,
				    "CRC indicated by the helper:\t"
				    "len = 0x%04X, txlen = %d\n", len, txlen);
			len &= ~BIT(0);
			/* Setting this to 0 to resend from same offset */
			txlen = 0;
		} else {
			i = 0;

			/* Set blocksize to transfer - checking for last
			   block */
			if (fw_len - offset < txlen)
				txlen = fw_len - offset;

			tx_bytes = ALIGN(txlen, MWL_SDIO_BLOCK_SIZE);

			/* Copy payload to buffer */
			memcpy(fwbuf, &fw_data[offset], txlen);
		}

		ret = sdio_writesb(card->func, ioport, fwbuf, tx_bytes);
		if (ret) {
			if (ret != -ENOMEDIUM) {
				wiphy_err(priv->hw->wiphy,
						"FW download, write iomem (%d) failed @ %d\n",
						i, offset);
				if (mwl_write_reg(card, CONFIGURATION_REG, 0x04))
					wiphy_err(priv->hw->wiphy, "write CFG reg failed\n");
			}

			ret = -1;
			goto err_dnld;
		}

		offset += txlen;
	}

	sdio_release_host(card->func);

	wiphy_info(priv->hw->wiphy,
		    "FW download over, size %d bytes\n", offset);

	kfree(fwbuf);

	ret = mwl_check_fw_status(priv, MAX_FIRMWARE_POLL_TRIES);
	if (ret) {
		wiphy_err(priv->hw->wiphy,
			"FW status is not ready\n");
		return ret;
	}

	mwl_sdio_init_irq(priv);

	/* Enabling interrupt after firmware is ready.
	 * Otherwise there may be abnormal interrupt DN_LD_HOST_INT_MASK
	 */
	mwl_sdio_enable_int(priv, true);

	return ret;


err_dnld:
	sdio_release_host(card->func);
	kfree(fwbuf);
	return ret;
}

/*
 * This function reads multiple data from SDIO card memory.
 */
static int mwl_read_data_sync(struct mwl_priv *priv, u8 *buffer,
			      u32 len, u32 port)
{
	struct mwl_sdio_card *card = priv->intf;
	int ret;
	u8 blk_mode = (port & MWL_SDIO_BYTE_MODE_MASK) ? BYTE_MODE
		       : BLOCK_MODE;
	u32 blk_size = (blk_mode == BLOCK_MODE) ? MWL_SDIO_BLOCK_SIZE : 1;
	u32 blk_cnt = (blk_mode == BLOCK_MODE) ? (len / MWL_SDIO_BLOCK_SIZE)
			: len;
	u32 ioport = (port & MWL_SDIO_IO_PORT_MASK);

	ret = sdio_readsb(card->func, buffer, ioport, blk_cnt * blk_size);

	if (ret)
		wiphy_err(priv->hw->wiphy," %s : sdio_readsb buffer ptr = 0x%p ioport: 0x%x"
				" for size %d failed with error %d\n",
				__func__, buffer, ioport,blk_cnt * blk_size, ret);
	return ret;
}

static char *mwl_sdio_event_strn(u16 event_id)
{
	int max_entries = 0;
	int curr_id = 0;

	static const struct {
		u16 id;
		char *id_string;
	} events[] = {
		{ SDEVENT_RADAR_DETECT, "SDEVENT_RADAR_DETECT" },
		{ SDEVENT_CHNL_SWITCH,  "SDEVENT_CHNL_SWITCH" },
		{ SDEVENT_BA_WATCHDOG,  "SDEVENT_BA_WATCHDOG" },
		{ SDEVENT_WAKEUP,       "SDEVENT_WAKEUP" },
		{ SDEVENT_PS_SLEEP,     "SDEVENT_PS_SLEEP" },
		{ SDEVENT_IBSS_LAST_BCN,"SDEVENT_IBSS_LAST_BCN_TXTSF" },
	};

	max_entries = ARRAY_SIZE(events);

	for (curr_id = 0; curr_id < max_entries; curr_id++)
		if ((event_id & 0x7fff) == events[curr_id].id)
			return events[curr_id].id_string;

	return "unknown";
}

/*Note:  When calling this function, it is expected that the fwcmd_mutex is already held */
int mwl_sdio_wakeup_card(struct mwl_priv *priv)
{
	int status;
	struct mwl_sdio_card *card = priv->intf;
	u8 cr;

	sdio_claim_host(card->func);

	if (mwl_read_reg(card, CONFIGURATION_REG, &cr))
		wiphy_err(priv->hw->wiphy, "read CFG reg failed\n");

	wiphy_dbg(priv->hw->wiphy, "Initiate Card wakeup\n");

	if (mwl_write_reg(card, CONFIGURATION_REG, (cr | 0x2))) {
		sdio_release_host(card->func);
		wiphy_err(priv->hw->wiphy, "write CFG reg failed\n");
		return -EIO;
	}

	if (mwl_read_reg(card, CONFIGURATION_REG, &cr))
		wiphy_err(priv->hw->wiphy, "read CFG reg failed\n");

	sdio_release_host(card->func);

	status = wait_event_timeout(card->wait_deepsleep,(
					card->is_deepsleep == 0), 12 * HZ);
	if (!status) {
		wiphy_err(priv->hw->wiphy, "info: Card Wakeup failed\n");
		return -EIO;
	}

	mwl_restart_ds_timer(priv, false);

	wiphy_dbg(priv->hw->wiphy, "info: Card Wakeup complete\n");
	return 0;
}

/*Note: The thread that triggered the wakeup request is holding the fwcmd_mutex */
void mwl_sdio_wakeup_complete(struct work_struct *work)
{
	struct mwl_sdio_card *card = container_of(work,
				struct mwl_sdio_card, event_work);
	struct mwl_priv *priv = card->priv;

	sdio_claim_host(card->func);

	if (mwl_write_reg(card, CONFIGURATION_REG, 0))
		wiphy_err(priv->hw->wiphy, "write CFG reg failed\n");

	sdio_release_host(card->func);

	card->is_deepsleep = 0;
	wake_up(&card->wait_deepsleep);
}

void mwl_sdio_enter_ps_sleep(struct work_struct *work)
{
	struct mwl_sdio_card *card = container_of(work,
					struct mwl_sdio_card, cmd_work);
	struct mwl_priv *priv = card->priv;
	int num,ret;

	if (priv->recovery_in_progress)
		return;

	/* Grab fwcmd mutex here so we can be sure nobody else changes the devices sleep state */
	if (!mutex_trylock(&priv->fwcmd_mutex))
		return;

	wiphy_dbg(priv->hw->wiphy,"In ps sleep enter:\n");

	/* check if tx empty,if not do not enter PS */
	for (num = SYSADPT_TX_WMM_QUEUES - 1; num >= 0; num--) {
		if (skb_queue_len(&priv->txq[num]) > 0) {
			wiphy_dbg(priv->hw->wiphy,
				"Sleep fail due to tx not empty %d\n",
				skb_queue_len(&priv->txq[num]));
			goto done;
		}
	}

	/* check if Rx has any packets */
	if (skb_queue_len(&card->rx_data_q) > 0) {
		wiphy_dbg(priv->hw->wiphy, "Sleep fail due to rx not empty\n");
		goto done;
	}

	ret = mwl_fwcmd_confirm_ps(priv->hw);

done:
	mutex_unlock(&priv->fwcmd_mutex);
}

static int mwl_sdio_event(struct mwl_priv *priv)
{
	struct ieee80211_hw *hw = priv->hw;
	struct mwl_sdio_card *card = priv->intf;
#if 0
	struct mwl_hostevent *host_event = (struct mwl_hostevent *)(
		&priv->pcmd_buf[INTF_CMDHEADER_LEN(INTF_HEADER_LEN)]);
#else
	struct mwl_hostevent *host_event = (struct mwl_hostevent *)(
		&priv->pcmd_event_buf[0]);
#endif
	u16	event_id = le16_to_cpu(host_event->mac_event.event_id);

	wiphy_dbg(hw->wiphy,"=> sd_event: %s\n", mwl_sdio_event_strn(event_id));

	switch (event_id) {
	case SDEVENT_RADAR_DETECT:
		ieee80211_radar_detected(hw);
		break;
	case SDEVENT_CHNL_SWITCH:
		ieee80211_queue_work(hw, &priv->chnl_switch_handle);
		break;
	case SDEVENT_BA_WATCHDOG:
		ieee80211_queue_work(hw, &priv->watchdog_ba_handle);
		break;
	case SDEVENT_WAKEUP:
		queue_work(card->cmd_workq, &card->event_work);
		break;
	case SDEVENT_PS_SLEEP:
		queue_work(card->cmd_workq, &card->cmd_work);
		break;
	case SDEVENT_IBSS_LAST_BCN:
		if(host_event->BcnPayload.event)
		{
			priv->LastBeaconTime = get_unaligned_le64(host_event->BcnPayload.bssTsf);
		}
		break;
	default:
		wiphy_info(hw->wiphy,"Unknown event, id=%04xh\n", event_id);
	}

	return 0;
}

/*
 * This function sends a data buffer to the card.
 */
static int mwl_sdio_card_to_host(struct mwl_priv *priv, u32 *type,
				     u8 *buffer, u32 npayload, u32 ioport)
{
	int ret;

	if (!buffer) {
		wiphy_err(priv->hw->wiphy,
		        "%s: buffer is NULL\n", __func__);
		return -1;
	}

	ret = mwl_read_data_sync(priv, buffer, npayload, ioport);
	if (type != NULL)
		*type = le16_to_cpu(*(__le16 *)(buffer + 2));

	return ret;
}


/*
 * This function gets the read port.
 *
 * If control port bit is set in MP read bitmap, the control port
 * is returned, otherwise the current read port is returned and
 * the value is increased (provided it does not reach the maximum
 * limit, in which case it is reset to 1)
 */
static int mwl_get_rd_port(struct mwl_priv *priv, u8 *port)
{
	struct mwl_sdio_card *card = priv->intf;
	const struct mwl_sdio_card_reg *reg = card->reg;
	u32 rd_bitmap = card->mp_rd_bitmap;

	if (!(rd_bitmap & reg->data_port_mask))
		return -1;

	if (!(card->mp_rd_bitmap & (1 << card->curr_rd_port)))
		return -1;

	/* We are now handling the SDIO data ports */
	card->mp_rd_bitmap &= (u32)(~(1 << card->curr_rd_port));
	*port = card->curr_rd_port;

	/*
	 * card->curr_rd_port is 0 ~ 31 (= start_rd_port ~ card->max_ports-1)
	 */
	if (++card->curr_rd_port == card->max_ports)
		card->curr_rd_port = reg->start_rd_port;

	return 0;
}

/*
 * This function decode sdio aggreation pkt.
 *
 * Based on the the data block size and pkt_len,
 * skb data will be decoded to few packets.
 */
static void mwl_deaggr_sdio_pkt(struct mwl_priv *priv,
				    struct sk_buff *skb)
{
	struct mwl_sdio_card *card = priv->intf;
	u32 total_pkt_len, pkt_len;
	struct sk_buff *skb_deaggr;
	u32 pkt_type;
	u16 blk_size;
	u8 blk_num;
	u8 *data;

	data = skb->data;
	total_pkt_len = skb->len;

	while (total_pkt_len >= (SDIO_HEADER_OFFSET + INTF_HEADER_LEN)) {
		if (total_pkt_len < card->sdio_rx_block_size)
			break;
		blk_num = *(data + BLOCK_NUMBER_OFFSET);
		blk_size = card->sdio_rx_block_size * blk_num;
		if (blk_size > total_pkt_len) {
			wiphy_err(priv->hw->wiphy,
				"%s: error in blk_size,\t"
				"blk_num=%d, blk_size=%d, total_pkt_len=%d\n",
				__func__, blk_num, blk_size, total_pkt_len);
			break;
		}
		pkt_len = le16_to_cpu(*(__le16 *)(data + SDIO_HEADER_OFFSET));
		pkt_type = le16_to_cpu(*(__le16 *)(data + SDIO_HEADER_OFFSET +
					 2));
		if ((pkt_len + SDIO_HEADER_OFFSET) > blk_size) {
			wiphy_err(priv->hw->wiphy,
				"%s: error in pkt_len,\t"
				"pkt_len=%d, blk_size=%d\n",
				__func__, pkt_len, blk_size);
			break;
		}
		skb_deaggr = mwl_alloc_dma_align_buf(pkt_len, GFP_KERNEL);
		if (!skb_deaggr)
			break;
		skb_put(skb_deaggr, pkt_len);
		memcpy(skb_deaggr->data, data + SDIO_HEADER_OFFSET, pkt_len);
		skb_pull(skb_deaggr, INTF_HEADER_LEN);

		mwl_handle_rx_packet(priv, skb_deaggr);
		data += blk_size;
		total_pkt_len -= blk_size;
	}
}


/*
 * This function decodes a received packet.
 *
 * Based on the type, the packet is treated as either a data, or
 * a command response, or an event, and the correct handler
 * function is invoked.
 */
static int mwl_decode_rx_packet(struct mwl_priv *priv,
				    struct sk_buff *skb, u32 upld_typ)
{
	struct mwl_sdio_card *card = priv->intf;
	__le16 *curr_ptr = (__le16 *)skb->data;
	u16 pkt_len = le16_to_cpu(*curr_ptr);
	struct mwl_rxinfo *rx_info;

	switch (upld_typ) {
	case MWL_TYPE_AGGR_DATA:
		rx_info = MWL_SKB_RXCB(skb);
		rx_info->buf_type = MWL_TYPE_AGGR_DATA;
		break;
	case MWL_TYPE_DATA:
		skb_trim(skb, pkt_len);
		/* Remove the header (len:2 + type:2) */
		skb_pull(skb, INTF_HEADER_LEN);

		break;
	case MWL_TYPE_MGMT:
		skb_trim(skb, pkt_len);
		/* Remove the header (len:2 + type:2) */
		skb_pull(skb, INTF_HEADER_LEN);
		rx_info = MWL_SKB_RXCB(skb);
		rx_info->buf_type = MWL_TYPE_MGMT;
		break;
	case MWL_TYPE_BEACON:
		skb_trim(skb, pkt_len);
		/* Remove the header (len:2 + type:2) */
		skb_pull(skb, INTF_HEADER_LEN);
		rx_info = MWL_SKB_RXCB(skb);
		rx_info->buf_type = MWL_TYPE_BEACON;
		break;
	default:
		wiphy_err(priv->hw->wiphy,
			    "unknown upload type %#x\n", upld_typ);
		goto error;
		break;
	}
	skb_queue_tail(&card->rx_data_q, skb);
	atomic_inc(&card->rx_pending);
	card->data_received = true;

error:
	return 0;
}


/*
 * This function transfers received packets from card to driver, performing
 * aggregation if required.
 *
 * For data received on control port, or if aggregation is disabled, the
 * received buffers are uploaded as separate packets. However, if aggregation
 * is enabled and required, the buffers are copied onto an aggregation buffer,
 * provided there is space left, processed and finally uploaded.
 */
static int mwl_sdio_card_to_host_mp_aggr(struct mwl_priv *priv,
					     u16 rx_len, u8 port)
{
	struct mwl_sdio_card *card = priv->intf;
	s32 f_do_rx_aggr = 0;
	s32 f_do_rx_cur = 0;
	s32 f_aggr_cur = 0;
	s32 f_post_aggr_cur = 0;
	struct sk_buff *skb_deaggr;
	struct sk_buff *skb = NULL;
	u32 pkt_len, pkt_type, mport, pind;
	u8 *curr_ptr;
	int i;
	u32 port_count;

	if (!card->mpa_rx.enabled) {
		wiphy_err(priv->hw->wiphy,
			    "info: %s: rx aggregation disabled\n",
			    __func__);

		f_do_rx_cur = 1;
		goto rx_curr_single;
	}

	if (card->mp_rd_bitmap &
		card->reg->data_port_mask) {
		/* Some more data RX pending */

		if (MP_RX_AGGR_IN_PROGRESS(card)) {
			if (MP_RX_AGGR_BUF_HAS_ROOM(card, rx_len)) {
				f_aggr_cur = 1;
			} else {
				/* No room in Aggr buf, do rx aggr now */
				f_do_rx_aggr = 1;
				f_post_aggr_cur = 1;
			}
		} else {
			/* Rx aggr not in progress */
			f_aggr_cur = 1;
		}

	} else {
		/* No more data RX pending */
		if (MP_RX_AGGR_IN_PROGRESS(card)) {
			f_do_rx_aggr = 1;
			if (MP_RX_AGGR_BUF_HAS_ROOM(card, rx_len))
				f_aggr_cur = 1;
			else
				/* No room in Aggr buf, do rx aggr now */
				f_do_rx_cur = 1;
		} else {
			f_do_rx_cur = 1;
		}
	}

	if (f_aggr_cur != 0) {
		/* Curr pkt can be aggregated */
		mp_rx_aggr_setup(card, rx_len, port);

		if (MP_RX_AGGR_PKT_LIMIT_REACHED(card) ||
		    mp_rx_aggr_port_limit_reached(card)) {
			/* wiphy_err(priv->hw->wiphy,
				    "info: %s: aggregated packet\t"
				    "limit reached\n", __func__);*/
			/* No more pkts allowed in Aggr buf, rx it */
			f_do_rx_aggr = 1;
		}
	}

	if (f_do_rx_aggr) {
		/* do aggr RX now */
		for (i = 0, port_count = 0; i < card->max_ports; i++)
			if (card->mpa_rx.ports & BIT(i))
				port_count++;
			/* Reading data from "start_port + 0" to "start_port +
			 * port_count -1", so decrease the count by 1
			 */
		port_count--;
		mport = (card->ioport | SDIO_MPA_ADDR_BASE |
				 (port_count << 8)) + card->mpa_rx.start_port;

		if (mwl_read_data_sync(priv, card->mpa_rx.buf,
					   card->mpa_rx.buf_len, mport))
			goto error;


		/*
		* Get the data from bus (in mpa_rx.buf)
		* => put to the buffer array packet by packet
		*/
		curr_ptr = card->mpa_rx.buf;
		for (pind = 0; pind < card->mpa_rx.pkt_cnt; pind++) {
			u32 *len_arr = card->mpa_rx.len_arr;

			/* get curr PKT len & type */
			pkt_len = le16_to_cpu(*(__le16 *) &curr_ptr[0]);
			pkt_type = le16_to_cpu(*(__le16 *) &curr_ptr[2]);

			/* copy pkt to deaggr buf */
			skb_deaggr = mwl_alloc_dma_align_buf(len_arr[pind],
								 GFP_KERNEL);
			if (!skb_deaggr) {
				wiphy_err(priv->hw->wiphy,
					"skb allocation failure\t"\
					"drop pkt len=%d type=%d\n",
					pkt_len, pkt_type);
				curr_ptr += len_arr[pind];
				continue;
			}

			skb_put(skb_deaggr, len_arr[pind]);

			if (((pkt_type == MWL_TYPE_DATA) ||
			     (pkt_type == MWL_TYPE_AGGR_DATA &&
				card->sdio_rx_aggr_enable) ||
			     (pkt_type == MWL_TYPE_MGMT) ||
			     (pkt_type == MWL_TYPE_BEACON)
			     ) &&
			    (pkt_len <= len_arr[pind])) {

				memcpy(skb_deaggr->data, curr_ptr, pkt_len);

				skb_trim(skb_deaggr, pkt_len);
				/* Process de-aggr packet */
				mwl_decode_rx_packet(priv, skb_deaggr,
							 pkt_type);
			} else {
				wiphy_err(priv->hw->wiphy,
					    "drop wrong aggr pkt:\t"
					    "sdio_single_port_rx_aggr=%d\t"
					    "type=%d len=%d max_len=%d\n",
					    card->sdio_rx_aggr_enable,
					    pkt_type, pkt_len, len_arr[pind]);
				dev_kfree_skb_any(skb_deaggr);
			}
			curr_ptr += len_arr[pind];
		}
		MP_RX_AGGR_BUF_RESET(card);
	}

rx_curr_single:
	if (f_do_rx_cur) {
		skb = mwl_alloc_dma_align_buf(rx_len, GFP_KERNEL);
		if (!skb) {
			wiphy_err(priv->hw->wiphy,
				    "single skb allocated fail,\t"
				    "drop pkt port=%d len=%d\n", port, rx_len);
			if (mwl_sdio_card_to_host(priv, &pkt_type,
				card->mpa_rx.buf, rx_len, card->ioport + port))
				goto error;

			return 0;
		}
		skb_put(skb, rx_len);

		if (mwl_sdio_card_to_host(priv, &pkt_type, skb->data, skb->len,
					      card->ioport + port))
			goto error;
		mwl_decode_rx_packet(priv, skb, pkt_type);
	}
	if (f_post_aggr_cur) {
		wiphy_err(priv->hw->wiphy,
			    "info: current packet aggregation\n");
		/* Curr pkt can be aggregated */
		mp_rx_aggr_setup(card, rx_len, port);
	}

	return 0;

error:
	if (MP_RX_AGGR_IN_PROGRESS(card))
		MP_RX_AGGR_BUF_RESET(card);

	if (f_do_rx_cur && skb)
		/* Single transfer pending. Free curr buff also */
		dev_kfree_skb_any(skb);

	return -1;
}

/*
static char *mwl_pktstrn(char* pkt)
{
	static char msg[80];
	char pkthd = pkt[0];
	char pkt_type = (pkthd&0x0c)>>2;
	char pkt_subtype = (pkthd&0xf0) >> 4;

	memset(msg, 0, sizeof(msg));
	if (pkt_type == 0) {    //mgmt pkt
		switch (pkt_subtype) {
		case 0x0:
			strcpy(msg, "assoc_req");
			break;
		case 0x01:
			strcpy(msg, "assoc_resp");
			break;
		case 0x2:
			strcpy(msg, "reassoc_req");
			break;
		case 0x3:
			strcpy(msg, "reassoc_resp");
			break;
		case 0x4:
			strcpy(msg, "probe_req");
			break;
		case 0x5:
			strcpy(msg, "prob_resp");
			break;
		case 0x8:
			strcpy(msg, "beacon");
			break;
		case 0xa:
			strcpy(msg, "disassoc");
			break;
		case 0xb:
			strcpy(msg, "auth");
			break;
		case 0xc:
			strcpy(msg, "deauth");
			break;
		case 0xd:
			strcpy(msg, "action");
			break;
		default:
			break;
		}
	}

	if (pkt_type == 1) {    //ctrl pkt
		switch (pkt_subtype) {
		case 0xb:
			strcpy(msg, "rts");
			break;
		case 0xc:
			strcpy(msg, "cts");
			break;
		case 0xd:
			strcpy(msg, "ack");
			break;
		case 0x8:
			strcpy(msg, "BAR");
			break;
		case 0x9:
			strcpy(msg, "BA");
			break;
		default:
			break;
		}
	}

	if (pkt_type == 2) {    //data pkt
		char* pllc;
		switch (pkt_subtype) {
		case 0x8:
			strcpy(msg, "QOS");
			break;
		case 0x00:
			strcpy(msg, "data");
			break;
		case 0x4:
			strcpy(msg, "null_data");
			return msg;
		default:
			strcpy(msg, "data");
			return msg;
		}
		if (pkt_subtype == 0x00) {
			pllc = &pkt[24];
		} else if (pkt_subtype == 0x8) {
			pllc = &pkt[26];
		}

		if ((pllc[0] == '\xaa') && (pllc[1]='\xaa') && (pllc[2]=='\x03')) {
			if ((pllc[6]=='\x08')&&(pllc[7]=='\x06')) {
				strcat(msg, " - ARP");
			}
			if ((pllc[6]=='\x08') && (pllc[7] == '\x00')) {
				char *pip = &pllc[8];
				strcat(msg, " - IP");
				if (pip[9] == '\x01') {
					char *picmp = &pip[20];
					strcat(msg, " - ICMP");
					if (picmp[0] == '\x00') {
						strcat(msg, " - echo_rply");
					}
					if (picmp[0] == '\x08') {
						strcat(msg, " - echo_req");
					}
				}
				if (pip[9] == '\x11') {
					//char *pudp = &pip[20];
					strcat(msg, " - UDP");
				}
			}
		}
	}

	return msg;
}
*/


/*
  Packet format (sdio interface):
  [len:2][type:2][mwl_rx_desc:44][mwl_dma_data:32][payload wo 802.11 header]
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
	if (ieee80211_is_mgmt(wh->frame_control)) {
		struct ieee80211_mgmt *mgmt;
		__le16 capab;

		mgmt = (struct ieee80211_mgmt *)prx_skb->data;

		if (unlikely(ieee80211_is_action(wh->frame_control) &&
			mgmt->u.action.category == WLAN_CATEGORY_BACK &&
			mgmt->u.action.u.addba_resp.action_code ==
				WLAN_ACTION_ADDBA_RESP)) {
			capab = mgmt->u.action.u.addba_resp.capab;
			if (le16_to_cpu(capab) & 1)
				mwl_rx_enable_sta_amsdu(priv, mgmt->sa);
		}
	}

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


static void mwl_sdio_rx_recv(unsigned long data)
{
	struct ieee80211_hw *hw = (struct ieee80211_hw *)data;
	struct mwl_priv *priv = hw->priv;
	struct mwl_sdio_card *card = priv->intf;
	struct mwl_rxinfo *rx_info;
	struct sk_buff *prx_skb = NULL;
	int work_done = 0;

	mwl_restart_ds_timer(priv, false);

	while (work_done < priv->recv_limit) {

		// shutdown flag set in other thread
		// Ensure compiler doesn't play games with the read...
		if (READ_ONCE(priv->shutdown))
			break;

		prx_skb = skb_dequeue(&card->rx_data_q);
		if (prx_skb == NULL) {
			break;
		}

		rx_info = MWL_SKB_RXCB(prx_skb);


		if (rx_info->buf_type == MWL_TYPE_AGGR_DATA)
			mwl_deaggr_sdio_pkt(priv, prx_skb);
		else
			mwl_handle_rx_packet(priv, prx_skb);
		work_done++;
	}

	return;
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
	struct mwl_tx_desc *tx_wcb;

	if (skb == NULL)
		return 0;

	tx_wcb = (struct mwl_tx_desc *) &data[INTF_HEADER_LEN];
	dma_data = (struct mwl_dma_data *) &data[tx_wcb->pkt_ptr];

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

		ieee80211_tx_status(hw, skb);
	}

	return 0;
}

static void mwl_sdio_flush_amsdu(unsigned long data)
{
	struct ieee80211_hw *hw = (struct ieee80211_hw *)data;
	struct mwl_priv *priv = hw->priv;
	struct mwl_amsdu_frag *amsdu_frag;

	struct mwl_sta *sta_info;
	int i;

	list_for_each_entry(sta_info, &priv->sta_list, list) {
		for (i = 0; i < SYSADPT_TX_WMM_QUEUES; i++) {
			amsdu_frag = &sta_info->amsdu_ctrl.frag[i];
			if (amsdu_frag->num) {
				spin_unlock_bh(&sta_info->amsdu_lock);
				mwl_tx_skb(priv, i,
					   amsdu_frag->skb);
				spin_lock_bh(&sta_info->amsdu_lock);
				amsdu_frag->num = 0;
				amsdu_frag->cur_pos = NULL;
			}
		}
	}
}

static void mwl_sdio_flush_amsdu_no_lock(unsigned long data)
{
	struct ieee80211_hw *hw = (struct ieee80211_hw *)data;
	struct mwl_priv *priv = hw->priv;
	struct mwl_amsdu_frag *amsdu_frag;

	struct mwl_sta *sta_info;
	int i;

	list_for_each_entry(sta_info, &priv->sta_list, list) {
		if (sta_info == NULL) {
			return;
		}
		for (i = 0; i < SYSADPT_TX_WMM_QUEUES; i++) {
			amsdu_frag = &sta_info->amsdu_ctrl.frag[i];
			if (amsdu_frag->num) {
				mwl_tx_skb(priv, i,
					   amsdu_frag->skb);
				amsdu_frag->num = 0;
				amsdu_frag->cur_pos = NULL;
			}
		}
	}
}

static void mwl_sdio_tx_workq(struct work_struct *work)
{
	struct mwl_sdio_card *card = container_of(work,
			struct mwl_sdio_card, tx_work);
	struct mwl_priv *priv = card->priv;
	struct ieee80211_hw *hw = (struct ieee80211_hw *)priv->hw;

	/* We wakeup here instead of mwl_mac80211_tx since this call runs in atomic context
		and tx needs sdio write call which is blocking call */
	mutex_lock(&priv->fwcmd_mutex);
	if (priv->if_ops.is_deepsleep(priv)) {
		priv->if_ops.wakeup_card(priv);
	}
	mutex_unlock(&priv->fwcmd_mutex);


	mwl_tx_skbs((unsigned long)hw);
	mwl_sdio_flush_amsdu_no_lock((unsigned long)hw);
}


/*
 * This function aggregates transmission buffers in driver and downloads
 * the aggregated packet to card.
 *
 * The individual packets are aggregated by copying into an aggregation
 * buffer and then downloaded to the card. Previous unsent packets in the
 * aggregation buffer are pre-copied first before new packets are added.
 * Aggregation is done till there is space left in the aggregation buffer,
 * or till new packets are available.
 *
 * The function will only download the packet to the card when aggregation
 * stops, otherwise it will just aggregate the packet in aggregation buffer
 * and return.
 */
static int mwl_host_to_card_mp_aggr(struct mwl_priv *priv,
					u8 *payload, u32 pkt_len, u32 port,
					u32 desc_num)
{
	struct mwl_sdio_card *card = priv->intf;
	int ret = 0;
	struct sk_buff *next_skb;
	s32 f_send_aggr_buf = 0;
	s32 f_send_cur_buf = 0;
	s32 f_precopy_cur_buf = 0;
	s32 f_postcopy_cur_buf = 0;
	u32 mport;
	u32 port_count, next_pkt_len;
	int i;

//	wiphy_err(priv->hw->wiphy, "%s() called\n", __FUNCTION__);
	next_skb = skb_peek(&priv->txq[desc_num]);
	if (next_skb != NULL) {
		next_pkt_len = next_skb->len +
			sizeof(struct mwl_tx_desc);
	}
	else
		next_pkt_len = 0;

	if (next_pkt_len) {
		/* More pkt in TX queue */
		if (MP_TX_AGGR_IN_PROGRESS(card)) {
			if (MP_TX_AGGR_BUF_HAS_ROOM(card, pkt_len)) {
				f_precopy_cur_buf = 1;

				if (((card->mp_wr_bitmap &
					(1 << card->curr_wr_port)) == 0) ||
				    !MP_TX_AGGR_BUF_HAS_ROOM(card,
					pkt_len + next_pkt_len)) {
					f_send_aggr_buf = 1;
				}
			} else {
				/* No room in Aggr buf, send it */
				f_send_aggr_buf = 1;

				if ((card->mp_wr_bitmap &
					(1 << card->curr_wr_port)) == 0)
					f_send_cur_buf = 1;
				else
					f_postcopy_cur_buf = 1;
			}
		} else {
			if (MP_TX_AGGR_BUF_HAS_ROOM(card, pkt_len) &&
			    (card->mp_wr_bitmap & (1 << card->curr_wr_port))) {
				f_precopy_cur_buf = 1;
			} else {
				f_send_cur_buf = 1;
			}
		}
	} else {
		/* Last pkt in TX queue */
		if (MP_TX_AGGR_IN_PROGRESS(card)) {
			/* some packs in Aggr buf already */
			f_send_aggr_buf = 1;

			if (MP_TX_AGGR_BUF_HAS_ROOM(card, pkt_len)) {
				f_precopy_cur_buf = 1;
			} else {
				/* No room in Aggr buf, send it */
				f_send_cur_buf = 1;
			}
		} else {
			f_send_cur_buf = 1;
		}
	}

	if (f_precopy_cur_buf) {
		MP_TX_AGGR_BUF_PUT(card, payload, pkt_len, port);

		if (MP_TX_AGGR_PKT_LIMIT_REACHED(card) ||
		    mp_tx_aggr_port_limit_reached(card))
			/* No more pkts allowed in Aggr buf, send it */
			f_send_aggr_buf = 1;
	}

	if (f_send_aggr_buf) {
		for (i = 0, port_count = 0; i < card->max_ports; i++)
				if (card->mpa_tx.ports & BIT(i))
					port_count++;

		/* Writing data from "start_port + 0" to "start_port +
		 * port_count -1", so decrease the count by 1
		 */
		port_count--;
		mport = (card->ioport | SDIO_MPA_ADDR_BASE | (port_count << 8))
			+ card->mpa_tx.start_port;
		ret = mwl_write_data_to_card(priv, card->mpa_tx.buf,
						 card->mpa_tx.buf_len, mport);

		MP_TX_AGGR_BUF_RESET(card);
	}

	if (f_send_cur_buf != 0) {
		ret = mwl_write_data_to_card(priv, payload, pkt_len,
						 card->ioport + port);
	}

	if (f_postcopy_cur_buf != 0)
		MP_TX_AGGR_BUF_PUT(card, payload, pkt_len, port);

	return ret;
}

/*
 * This function gets the write port for data.
 *
 * The current write port is returned if available and the value is
 * increased (provided it does not reach the maximum limit, in which
 * case it is reset to 1)
 */
static int mwl_get_wr_port_data(struct mwl_priv *priv, u32 *port)
{
	struct mwl_sdio_card *card = priv->intf;
	const struct mwl_sdio_card_reg *reg = card->reg;
	u32 wr_bitmap = card->mp_wr_bitmap;

	if (!(wr_bitmap & card->mp_data_port_mask)) {
		card->data_sent = true;
		return -EBUSY;
	}

	if (card->mp_wr_bitmap & (1 << card->curr_wr_port)) {
		card->mp_wr_bitmap &= (u32) (~(1 << card->curr_wr_port));
		*port = card->curr_wr_port;
		if (++card->curr_wr_port == card->mp_end_port)
			card->curr_wr_port = reg->start_wr_port;
	} else {
		card->data_sent = true;
		return -EBUSY;
	}

	return 0;
}


static bool mwl_sdio_is_tx_available(struct mwl_priv *priv, int desc_num)
{
	struct mwl_sdio_card *card = priv->intf;
	u32 wr_bitmap = card->mp_wr_bitmap;

	if ((wr_bitmap & card->mp_data_port_mask) == 0)
		return false;

	if ((card->mp_wr_bitmap & (1 << card->curr_wr_port)) == 0)
		return false;

	return true;
}

/*
 * Adds TxPD to AMSDU header.
 *
 * Each AMSDU packet will contain one TxPD at the beginning,
 * followed by multiple AMSDU subframes.
 */
static int
mwl_process_txdesc(struct mwl_priv *priv, struct sk_buff *skb, struct mwl_dma_data **dma_data)
{
	struct mwl_tx_desc *tx_desc;
	struct mwl_tx_ctrl *tx_ctrl;
	struct ieee80211_tx_info *tx_info;
	u8 *ptr;
	int align_pad;
	int size_needed;
	struct mwl_sdio_card *card = priv->intf;

	tx_info = IEEE80211_SKB_CB(skb);
	tx_ctrl = (struct mwl_tx_ctrl *)&IEEE80211_SKB_CB(skb)->status;

	align_pad = ((void *)skb->data - (sizeof(struct mwl_tx_desc) + INTF_HEADER_LEN)-
			NULL) & (MWL_DMA_ALIGN_SZ - 1);

	if (align_pad)
	{
		card->tx_pkt_unaligned_cnt++;
	}

	size_needed = sizeof(struct mwl_tx_desc) + INTF_HEADER_LEN + align_pad;

	/* existing pointers into skb buffer may not be valid after this call */
	if (skb_cow_head(skb, size_needed)) {
		WARN_ON(skb_headroom(skb) < (size_needed));
		return -ENOMEM;
	}

	ptr = (u8 *)skb->data;

	*dma_data = (struct mwl_dma_data *) skb->data;

	skb_push(skb, sizeof(struct mwl_tx_desc)+align_pad);
	tx_desc = (struct mwl_tx_desc *) skb->data;
	memset(tx_desc, 0, sizeof(struct mwl_tx_desc));

	skb_push(skb, INTF_HEADER_LEN);
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
	tx_desc->pkt_ptr = cpu_to_le32(ptr - (u8 *)skb->data);
	tx_desc->status = 0;

	return 0;
}

/*

*/
static int mwl_sdio_host_to_card(struct mwl_priv *priv,
	int desc_num, struct sk_buff *tx_skb)
{
	struct mwl_sdio_card *card = priv->intf;
	int ret;
	u32 buf_block_len;
	u32 blk_size;
	u32 port;
	u8 *payload;
	u32 pkt_len;
	struct mwl_dma_data *dma_data;
	struct ieee80211_hdr *wh;

	if (priv->recovery_in_progress) {
		dev_kfree_skb_any(tx_skb);
		ret = -ENETDOWN;
		goto done;
	}

	/* get port number. */
	ret = mwl_get_wr_port_data(priv, &port);
	if (ret) {
//		wiphy_err(priv->hw->wiphy, "%s: no wr_port available\n", __func__);
		dev_kfree_skb_any(tx_skb);
		goto done;
	}


//	wiphy_err(priv->hw->wiphy, "curr wr_port = %d\n", port);

	/* hard code rate_info here, will get this information from FW later. */
	card->rate_info = 0x0F4F8762;  /* VHT, SGI-80M, MCS7, 3SS.*/

	/* Push INTF_HEADER_LEN & mwl_tx_desc
	 *
	 * Existing pointers into skb buffer are not valid after this call
	 */
	ret = mwl_process_txdesc(priv, tx_skb, &dma_data);
	if (ret) {
		wiphy_err(priv->hw->wiphy, "%s: Failed to send packet! ret = %d\n", __func__, ret);
		dev_kfree_skb_any(tx_skb);
		goto done;
	}

	wh = &dma_data->wh;

	payload = (u8 *)tx_skb->data;
	pkt_len = tx_skb->len;

	/* Transfer data to card */
	blk_size = MWL_SDIO_BLOCK_SIZE;
	buf_block_len = (pkt_len + blk_size - 1) / blk_size;
	*(__le16 *)&payload[0] = cpu_to_le16((u16)pkt_len);
	if (ieee80211_is_data(wh->frame_control))
		*(__le16 *)&payload[2] = cpu_to_le16(MWL_TYPE_DATA);
	else
		*(__le16 *)&payload[2] = cpu_to_le16(MWL_TYPE_MGMT);

	pkt_len = buf_block_len * blk_size;
	ret = mwl_host_to_card_mp_aggr(priv, payload, pkt_len,
						   port, desc_num);

	if (ret != 0) {
		card->curr_wr_port = port;
		card->mp_wr_bitmap |= (u32)(1<<card->curr_wr_port);
	}
	mwl_write_data_complete(priv, tx_skb);

done:
	return ret;
}

/*
 * SDIO interrupt handler.
 *
 * This function reads the interrupt status from firmware and handles
 * the interrupt in current thread (ksdioirqd) right away.
*
 * The following interrupts are checked and handled by this function -
 *      - Data sent
 *      - Command sent
 *      - Packets received
 *
 * Since the firmware does not generate download ready interrupt if the
 * port updated is command port only, command sent interrupt checking
 * should be done manually, and for every SDIO interrupt.
 *
 * In case of Rx packets received, the packets are uploaded from card to
 * host and processed accordingly.
 */
static void
mwl_sdio_interrupt(struct sdio_func *func)
{
	struct mwl_priv      *priv;
	struct ieee80211_hw  *hw;
	struct mwl_sdio_card *card;
	u8 sdio_ireg;
	u32 rx_blocks;
	u16 rx_len;
	unsigned long flags;
	u32 bitmap;
	u8 cr;

	hw = sdio_get_drvdata(func);

	if (!hw || !hw->priv)
		return;

	priv = hw->priv;
	card = priv->intf;

	if (mwl_read_data_sync(priv, card->mp_regs, card->reg->max_mp_regs, REG_PORT)) {
		wiphy_err(priv->hw->wiphy, "read mp_regs failed\n");
		return;
	}

	/*
	 * DN_LD_HOST_INT_STATUS and/or UP_LD_HOST_INT_STATUS
	 * For SDIO new mode CMD port interrupts
	 *	DN_LD_CMD_PORT_HOST_INT_STATUS and/or
	 *	UP_LD_CMD_PORT_HOST_INT_STATUS
	 * Clear the interrupt status register
	 */
	spin_lock_irqsave(&card->int_lock, flags);
	card->int_status |= card->mp_regs[card->reg->host_int_status_reg];
	sdio_ireg = card->int_status;
	spin_unlock_irqrestore(&card->int_lock, flags);

	if (!sdio_ireg)
		return;

	/* Following interrupt is only for SDIO new mode */
	if (sdio_ireg & DN_LD_CMD_PORT_HOST_INT_STATUS) {
		card->cmd_wait_q.status = 0;
		wake_up(&card->cmd_wait_q.wait);
	}

	/* Command Response / Event is back */
	if (sdio_ireg & UP_LD_CMD_PORT_HOST_INT_STATUS) {
		struct cmd_header *cmd_hdr_resp = (struct cmd_header *)
			&priv->pcmd_event_buf[INTF_CMDHEADER_LEN(INTF_HEADER_LEN)];
		__le16 *pRspBuf = (__le16 *)priv->pcmd_event_buf;

		spin_lock_irqsave(&card->int_lock, flags);
		card->int_status &= ~UP_LD_CMD_PORT_HOST_INT_STATUS;
		spin_unlock_irqrestore(&card->int_lock, flags);

		/* read the len of control packet */
		rx_len = card->mp_regs[card->reg->cmd_rd_len_1] << 8;
		rx_len |= (u16)card->mp_regs[card->reg->cmd_rd_len_0];
		rx_blocks = DIV_ROUND_UP(rx_len, MWL_SDIO_BLOCK_SIZE);
		if ((rx_blocks * MWL_SDIO_BLOCK_SIZE) > CMD_BUF_SIZE)
			return;

		rx_len = (u16) (rx_blocks * MWL_SDIO_BLOCK_SIZE);

		if (mwl_sdio_card_to_host(priv, NULL, (u8 *)priv->pcmd_event_buf,
						rx_len, card->ioport | CMD_PORT_SLCT)) {
			wiphy_err(hw->wiphy,
			    "%s: failed to card_to_host", __func__);
			goto term_cmd;
		}

		/*
		* If command has been sent & cmd_code = 0x8xxx => It's cmd_resp
		* Otherwise, it's event (new added)
		*/
		if ((!card->cmd_resp_recvd) &&
		    (le16_to_cpu(cmd_hdr_resp->command) == (card->cmd_id | HOSTCMD_RESP_BIT)) &&
		    (pRspBuf[1] ==  cpu_to_le16(MWL_TYPE_CMD))) {
			spin_lock_irqsave(&card->int_lock, flags);
			card->cmd_id = 0;
			memcpy(priv->pcmd_buf,priv->pcmd_event_buf,rx_len);
			card->cmd_wait_q.status = 0;
			card->cmd_resp_recvd = true;
			spin_unlock_irqrestore(&card->int_lock, flags);

			wake_up(&card->cmd_wait_q.wait);

		}
		else if (pRspBuf[1] ==  cpu_to_le16(MWL_TYPE_EVENT)) {
			mwl_sdio_event(priv);
		}
		else {
			wiphy_err(hw->wiphy,
			    "%s: Unexpected cmd/resp! Type 0x%x, cmd 0x%x\n",
			    __func__, le16_to_cpu(pRspBuf[1]), le16_to_cpu(cmd_hdr_resp->command));
		}
	}

	/* Tx-Done interrupt */
	if (sdio_ireg & DN_LD_HOST_INT_STATUS) {
		bitmap = (u32) card->mp_regs[card->reg->wr_bitmap_l];
		bitmap |= ((u32) card->mp_regs[card->reg->wr_bitmap_u]) << 8;
		bitmap |= ((u32) card->mp_regs[card->reg->wr_bitmap_1l]) << 16;
		bitmap |= ((u32) card->mp_regs[card->reg->wr_bitmap_1u]) << 24;

		spin_lock_irqsave(&card->int_lock, flags);
		card->int_status &= ~DN_LD_HOST_INT_STATUS;
		spin_unlock_irqrestore(&card->int_lock, flags);

		card->mp_wr_bitmap = bitmap;

		if (card->data_sent &&
		    (card->mp_wr_bitmap & card->mp_data_port_mask)) {
#if 0
			wiphy_err(hw->wiphy,
				    "error:  <--- Tx DONE Interrupt bmp=0x%x --->\n", card->mp_wr_bitmap);
#endif
			card->data_sent = false;
		}

		queue_work(card->tx_workq, &card->tx_work);
	}

	/* Rx process */
	if (sdio_ireg & UP_LD_HOST_INT_STATUS) {
		bitmap = (u32) card->mp_regs[card->reg->rd_bitmap_l];
		bitmap |= ((u32) card->mp_regs[card->reg->rd_bitmap_u]) << 8;
		bitmap |= ((u32) card->mp_regs[card->reg->rd_bitmap_1l]) << 16;
		bitmap |= ((u32) card->mp_regs[card->reg->rd_bitmap_1u]) << 24;
		card->mp_rd_bitmap = bitmap;

		spin_lock_irqsave(&card->int_lock, flags);
		card->int_status &= ~UP_LD_HOST_INT_STATUS;
		spin_unlock_irqrestore(&card->int_lock, flags);

		while (true) {
			u8 port;
			u32 len_reg_l, len_reg_u;

			if (mwl_get_rd_port(priv, &port))
				break;

			len_reg_l = card->reg->rd_len_p0_l + (port << 1);
			len_reg_u = card->reg->rd_len_p0_u + (port << 1);
			rx_len = ((u16) card->mp_regs[len_reg_u]) << 8;
			rx_len |= (u16) card->mp_regs[len_reg_l];

			rx_blocks =
				(rx_len + MWL_SDIO_BLOCK_SIZE -
				 1) / MWL_SDIO_BLOCK_SIZE;

			if (card->mpa_rx.enabled &&
			     ((rx_blocks * MWL_SDIO_BLOCK_SIZE) >
			      card->mpa_rx.buf_size)) {
				wiphy_err(hw->wiphy,
					    "invalid rx_len=%d\n",
					    rx_len);
				return;
			}

			rx_len = (u16) (rx_blocks * MWL_SDIO_BLOCK_SIZE);
			if (mwl_sdio_card_to_host_mp_aggr(priv, rx_len,
							      port)) {
				wiphy_err(hw->wiphy,
					    "card_to_host_mpa failed: int status=%#x\n",
					    sdio_ireg);
				goto term_cmd;
			}
		}

		/* Indicate the received packets (card->rx_data_q)to MAC80211 */
		if (!priv->shutdown) {
			tasklet_schedule(&priv->rx_task);
		}
	}
	return;

term_cmd:
	/* terminate cmd */
	if (mwl_read_reg(card, CONFIGURATION_REG, &cr))
		wiphy_err(hw->wiphy, "read CFG reg failed\n");
	else
		wiphy_err(hw->wiphy,
			    "info: CFG reg val = %d\n", cr);

	if (mwl_write_reg(card, CONFIGURATION_REG, (cr | 0x04)))
		wiphy_err(hw->wiphy,
			    "write CFG reg failed\n");
	else
		wiphy_err(hw->wiphy, "info: write success\n");

	if (mwl_read_reg(card, CONFIGURATION_REG, &cr))
		wiphy_err(hw->wiphy,
			    "read CFG reg failed\n");
	else
		wiphy_err(hw->wiphy,
			    "info: CFG reg val =%x\n", cr);
}

/* Check command response back or not */
static int mwl_sdio_cmd_resp_wait_completed(struct mwl_priv *priv,
	unsigned short cmd)
{
	struct mwl_sdio_card *card = priv->intf;
	int status;

	/* Wait for completion */
	status = wait_event_timeout(card->cmd_wait_q.wait,
						  (card->cmd_resp_recvd == true),
						  (12 * HZ));

	if (!card->is_running)
		return -ENETDOWN;

	if (status == 0) {
		status = -ETIMEDOUT;
		wiphy_err(priv->hw->wiphy, "timeout, cmd_wait_q terminated: %d\n",
			    status);
		card->cmd_wait_q.status = status;
		return status;
	}

	status = card->cmd_wait_q.status;
	card->cmd_wait_q.status = 0;

  /* status is command response value */
	return status;
}

/*
 * This function enables the host interrupt.
 *
 * The host interrupt enable mask is written to the card
 * host interrupt mask register.
 */
static int mwl_sdio_register_dev(struct mwl_priv *priv)
{
	int rc  = 0;

	return rc;
}

/*
 * This function unregisters the SDIO device.
 *
 * The SDIO IRQ is released, the function is disabled and driver
 * data is set to null.
 */
static void
mwl_sdio_unregister_dev(struct mwl_priv *priv)
{
	struct mwl_sdio_card *card = priv->intf;

	mwl_sdio_cleanup(priv);

	cancel_work_sync(&card->tx_work);
	destroy_workqueue(card->tx_workq);

	cancel_work_sync(&card->cmd_work);
	cancel_work_sync(&card->event_work);

	destroy_workqueue(card->cmd_workq);

	/* Free pcmd_buf/pcmd_event_buf */
	kfree(priv->pcmd_buf); priv->pcmd_buf = NULL;
	kfree(priv->pcmd_event_buf); priv->pcmd_event_buf = NULL;
}

/*Note:  When calling this function, it is expected that the fwcmd_mutex is already held */
static void mwl_sdio_enter_deepsleep(struct mwl_priv * priv)
{
	struct mwl_sdio_card *card = priv->intf;
	card->is_deepsleep = 1;
}

static int mwl_sdio_is_deepsleep(struct mwl_priv * priv)
{

	struct mwl_sdio_card *card = priv->intf;
	return card->is_deepsleep;
}

static void mwl_sdio_up_dev(struct mwl_priv *priv)
{
	wiphy_info(priv->hw->wiphy, "%s: Bringing up adapter...\n", MWL_DRV_NAME);

	mwl_sdio_init(priv);
}

static void mwl_sdio_down_dev(struct mwl_priv *priv)
{
	wiphy_info(priv->hw->wiphy, "%s: Taking down adapter...\n", MWL_DRV_NAME);

	mwl_sdio_cleanup(priv);
}

static int mwl_sdio_mmc_hw_reset(struct sdio_func *func)
{
	int rc = mmc_hw_reset(func->card);
	if (rc == 0)
		mmc_card_clr_suspended(func->card);

	return rc;
}

static int mwl_sdio_up_pwr(struct mwl_priv *priv)
{
	struct mwl_sdio_card * card = (struct mwl_sdio_card *)priv->intf;
	int rc;

	if (!priv->mac_init_complete)
		return 0;

	rc = mwl_sdio_set_gpio(card, 1);
	if (rc)
		return rc;

	sdio_claim_host(card->func);
	rc = mwl_sdio_mmc_hw_reset(card->func);
	sdio_release_host(card->func);

	return rc;
}

static void mwl_sdio_down_pwr(struct mwl_priv *priv)
{
	struct mwl_sdio_card * card = (struct mwl_sdio_card *)priv->intf;

	mwl_sdio_set_gpio(card, 0);
}

static int mwl_sdio_restart_handler(struct mwl_priv *priv)
{
	int ret, i;

	wiphy_debug(priv->hw->wiphy, "%s: Restarting adapter...\n", MWL_DRV_NAME);

	mwl_shutdown_sw(priv, false);

	for (i = 0; i < 2; ++i) {
		ret = mwl_sdio_reset(priv);
		if (ret)
			continue;

		if (priv->stop_shutdown) {
			ieee80211_restart_hw(priv->hw);
			ret = 0;
			break;
		}

		ret = mwl_reinit_sw(priv, false);
		if (!ret)
			break;

		wiphy_err(priv->hw->wiphy,
			"%s: Re-initialization failed with error %d\n",
			MWL_DRV_NAME, ret);
	}

	return ret;
}

#ifdef CONFIG_DEBUG_FS
static int mwl_sdio_dbg_info (struct mwl_priv *priv, char *p, int size, int len)
{
	struct mwl_sdio_card *card = priv->intf;

#ifdef CONFIG_PM
	len += scnprintf(p + len, size - len, "WOW Capable: %s\n", priv->wow.capable?"yes":"no");
#endif

	len += scnprintf(p + len, size - len, "Link down power off: %s\n", priv->stop_shutdown ? "enable":"disable");

	if (gpio_is_valid(card->reset_pwd_gpio)) {
		len += scnprintf(p + len, size - len, "PMU_EN gpio: %d\n", card->reset_pwd_gpio);
	}

	return len;
}
#endif

MODULE_DEVICE_TABLE(sdio, mwl_sdio_id_tbl);
static struct mwl_if_ops sdio_ops = {
	.inttf_head_len          = INTF_HEADER_LEN,
	.init_if                 = mwl_sdio_init,
	.init_if_post            = mwl_sdio_init_post,
	.cleanup_if              = mwl_sdio_cleanup,
	.check_card_status       = mwl_sdio_check_card_status,
	.prog_fw                 = mwl_sdio_program_firmware,
	.register_dev            = mwl_sdio_register_dev,
//	.unregister_dev          = mwl_sdio_unregister_dev,
	.send_cmd                = mwl_sdio_send_command,
	.cmd_resp_wait_completed = mwl_sdio_cmd_resp_wait_completed,
	.host_to_card            = mwl_sdio_host_to_card,
	.is_tx_available         = mwl_sdio_is_tx_available,
	.flush_amsdu             = mwl_sdio_flush_amsdu,
	.enter_deepsleep         = mwl_sdio_enter_deepsleep,
	.wakeup_card             = mwl_sdio_wakeup_card,
	.is_deepsleep            = mwl_sdio_is_deepsleep,
	.up_dev                  = mwl_sdio_up_dev,
	.down_dev                = mwl_sdio_down_dev,
	.up_pwr                  = mwl_sdio_up_pwr,
	.down_pwr                = mwl_sdio_down_pwr,
#ifdef CONFIG_DEBUG_FS
	.dbg_info                = mwl_sdio_dbg_info,
#endif
};

static int mwl_sdio_probe(struct sdio_func *func,
	const struct sdio_device_id *id)
{
	static bool printed_version;
	struct mwl_sdio_card *card;
	struct device_node *np, *of_node = NULL;
	int gpio, rc = 0;
	enum of_gpio_flags flags;

	if (id->driver_data >= MWLUNKNOWN)
		return -ENODEV;

	if (!printed_version) {
		dev_info(&func->dev, "<<%s version %s>>",
			LRD_DESC, LRD_DRV_VERSION);
		printed_version = true;
	}

	/* device tree node parsing and platform specific configuration */
	if (dev_of_node(&func->dev)) {
		if (of_match_node(mwl_sdio_of_match_table, dev_of_node(&func->dev)))
			of_node = dev_of_node(&func->dev);
	}

	if (!of_node && dev_of_node(&func->card->dev)) {
		for_each_child_of_node(dev_of_node(&func->card->dev), np) {
			if (of_match_node(mwl_sdio_of_match_table, np)) {
				of_node = np;
				break;
			}
		}
	}

	if (!of_node && dev_of_node(func->card->host->parent)) {
		for_each_child_of_node(dev_of_node(func->card->host->parent), np) {
			if (of_match_node(mwl_sdio_of_match_table, np)) {
				of_node = np;
				break;
			}
		}
	}

	card = kzalloc(sizeof(struct mwl_sdio_card), GFP_KERNEL);
	if (!card) {
		dev_err(&func->dev,  ": allocate mwl_sdio_card structure failed");
		return -ENOMEM;
	}

	card->func = func;
	card->dev_id = id;

	if ((id->driver_data == MWL8897) || (id->driver_data == MWL8997)){
		if (id->driver_data == MWL8897) {
			card->reg = &mwl_reg_sd8897;
			card->chip_type = MWL8897;
		} else {
			card->reg = &mwl_reg_sd8997;
			card->chip_type = MWL8997;
		}

		card->max_ports = 32;
		card->mp_agg_pkt_limit = 16;
		card->tx_buf_size = MWL_TX_DATA_BUF_SIZE_4K;
		card->mp_tx_agg_buf_size = MWL_MP_AGGR_BUF_SIZE_MAX;
		card->mp_rx_agg_buf_size = MWL_MP_AGGR_BUF_SIZE_MAX;
		card->mp_end_port = 0x0020;
	}

	card->tx_workq = alloc_workqueue("lrdwifi-tx_workq",
		WQ_HIGHPRI | WQ_MEM_RECLAIM | WQ_UNBOUND, 1);
	card->cmd_workq = alloc_workqueue("lrdwifi-cmd_workq",
		WQ_HIGHPRI | WQ_MEM_RECLAIM | WQ_UNBOUND, 1);

	if (!card->tx_workq || !card->cmd_workq) {
		rc = -ENOMEM;
		goto err_return;
	}

	INIT_WORK(&card->tx_work, mwl_sdio_tx_workq);
	INIT_WORK(&card->cmd_work, mwl_sdio_enter_ps_sleep);
	INIT_WORK(&card->event_work, mwl_sdio_wakeup_complete);

	sdio_ops.ptx_work  = &card->tx_work;
	sdio_ops.ptx_workq = card->tx_workq;

	memcpy(&sdio_ops.mwl_chip_tbl, &mwl_chip_tbl[card->chip_type],
		sizeof(struct mwl_chip_info));

	card->reset_pwd_gpio = reset_pwd_gpio;

	if (of_node) {
		gpio = of_get_named_gpio_flags(of_node, "pmu-en-gpios", 0, &flags);

		if (!gpio_is_valid(gpio))
			gpio = of_get_named_gpio_flags(of_node, "reset-gpios", 0, &flags);

		if (gpio_is_valid(gpio)) {
			rc = devm_gpio_request_one(&func->dev, gpio,
				flags | GPIOF_OUT_INIT_HIGH, "wifi_pmu_en");

			if (!rc)
				card->reset_pwd_gpio = gpio;
		}
	}

	if (gpio_is_valid(card->reset_pwd_gpio))
		dev_info(&func->dev, "PMU_EN GPIO %d configured\n", card->reset_pwd_gpio);
	else
		dev_info(&func->dev, "PMU_EN GPIO not configured\n");

	rc = mwl_add_card(card, &sdio_ops, of_node);

err_return:

	if (rc != 0) {
		if (rc != -EPROBE_DEFER)
			dev_err(&func->dev, "Failed to add_card %d\n", rc);

		if (card->cmd_workq)
			destroy_workqueue(card->cmd_workq);
		if (card->tx_workq)
			destroy_workqueue(card->tx_workq);
		kfree(card);
	}

	return rc;
}

static void mwl_sdio_remove(struct sdio_func *func)
{
	struct mwl_priv     *priv;
	struct ieee80211_hw *hw;

	dev_info(&func->dev, "Card removal initiated!\n");
	hw = sdio_get_drvdata(func);
	if (!hw || !hw->priv) {
		dev_err(&func->dev, "Data structures invalid, exiting...");
		return;
	}

	priv = hw->priv;

	mwl_wl_deinit(priv);

	mwl_sdio_unregister_dev(priv);

	mwl_sdio_reset(priv);

	mwl_ieee80211_free_hw(priv);
	kfree(priv->intf);

	dev_info(&func->dev, "Card removal complete!\n");
}

#ifdef CONFIG_PM
static int mwl_sdio_pm_worker(struct device *dev, int action)
{
	struct sdio_func     *func;
	struct ieee80211_hw  *hw;
	struct mwl_priv      *priv;
	struct mwl_sdio_card *card;

	func = dev_to_sdio_func(dev);
	if (!func)
		return 0;

	hw = sdio_get_drvdata(func);
	if (!hw || !hw->priv)
		return 0;

	priv = (struct mwl_priv*)hw->priv;
	card = (struct mwl_sdio_card *)priv->intf;
	if (!card)
		return 0;

	// Since we do not own reset gpio, we need to restore it direction
	// on resume from hibernate manually
	if (action == MWL_PM_RESTORE_EARLY) {
		if (gpio_is_valid(card->reset_pwd_gpio))
			gpio_direction_output(card->reset_pwd_gpio, 0);
	}

	// When we are in stop shutdown mode ignore PM callbacks
	// mac80211 start/stop will control power state
	if (priv->stop_shutdown && !(priv->wow.state & WOWLAN_STATE_ENABLED))
		return 0;

	switch (action) {
	case MWL_PM_PREPARE:
		if (mmc_card_is_removable(func->card->host)) {
			card->caps_fixups      |= MMC_CAP_NONREMOVABLE;
			func->card->host->caps |= MMC_CAP_NONREMOVABLE;
		}
		break;

	case MWL_PM_COMPLETE:
		if (card->caps_fixups)
			func->card->host->caps &= ~(card->caps_fixups);
		break;

	case MWL_PM_RESUME_EARLY:
		if (!(sdio_get_host_pm_caps(func) & MMC_PM_KEEP_POWER)) {
			mwl_sdio_set_gpio(card, 1);
			card->expect_recovery = true;
		} else if (priv->wow.state & WOWLAN_STATE_ENABLED)
			lrd_disable_wowlan(priv);
		break;

	case MWL_PM_RESTORE_EARLY:
		if (sdio_get_host_pm_caps(func) & MMC_PM_KEEP_POWER)
			mwl_shutdown_sw(priv, true);

		mwl_sdio_set_gpio(card, 1);
		card->expect_recovery = true;
		break;

	case MWL_PM_RESUME:
		if (card->expect_recovery) {
			if (sdio_get_host_pm_caps(func) & MMC_PM_KEEP_POWER) {
				sdio_claim_host(func);
				mwl_sdio_mmc_hw_reset(card->func);
				sdio_release_host(func);
			}

			card->is_suspended = false;
			mwl_reinit_sw(priv, true);

			card->expect_recovery = false;
		}
		else {
			card->is_suspended = false;
			mwl_restart_ds_timer(priv, false);
		}

		break;

	case MWL_PM_SUSPEND:
		if (!(sdio_get_host_pm_caps(func) & MMC_PM_KEEP_POWER))
			mwl_shutdown_sw(priv, true);
		else {
			sdio_set_host_pm_flags(func, MMC_PM_KEEP_POWER);
			mwl_delete_ds_timer(priv);
		}
		card->is_suspended = true;
		break;

	case MWL_PM_SUSPEND_LATE:
		if (!(sdio_get_host_pm_caps(func) & MMC_PM_KEEP_POWER))
			mwl_sdio_set_gpio(card, 0);
		else if (priv->wow.state & WOWLAN_STATE_ENABLED)
			lrd_enable_wowlan(priv);
		break;

	case MWL_PM_POWEROFF:
		mwl_sdio_set_gpio(card, 0);
		break;
	}

	return 0;
}

static int mwl_sdio_prepare(struct device *dev)
{
	return mwl_sdio_pm_worker(dev, MWL_PM_PREPARE);
}

static void mwl_sdio_complete(struct device *dev)
{
	mwl_sdio_pm_worker(dev, MWL_PM_COMPLETE);
}

static int mwl_sdio_resume(struct device *dev)
{
	return mwl_sdio_pm_worker(dev, MWL_PM_RESUME);
}

static int mwl_sdio_suspend(struct device *dev)
{
	return mwl_sdio_pm_worker(dev, MWL_PM_SUSPEND);
}

static int mwl_sdio_suspend_late(struct device *dev)
{
	return mwl_sdio_pm_worker(dev, MWL_PM_SUSPEND_LATE);
}

static int mwl_sdio_resume_early(struct device *dev)
{
	return mwl_sdio_pm_worker(dev, MWL_PM_RESUME_EARLY);
}

static int mwl_sdio_restore_early(struct device *dev)
{
	return mwl_sdio_pm_worker(dev, MWL_PM_RESTORE_EARLY);
}

static int mwl_sdio_poweroff(struct device *dev)
{
	return mwl_sdio_pm_worker(dev, MWL_PM_POWEROFF);
}

static const struct dev_pm_ops mwl_sdio_pm_ops = {
	.prepare        = mwl_sdio_prepare,
	.complete       = mwl_sdio_complete,
	.resume         = mwl_sdio_resume,
	.suspend        = mwl_sdio_suspend,
	.resume_early   = mwl_sdio_resume_early,
	.thaw_early     = mwl_sdio_resume_early,
	.restore_early  = mwl_sdio_restore_early,
	.suspend_late   = mwl_sdio_suspend_late,
	.freeze_late    = mwl_sdio_suspend_late,
	.poweroff_late  = mwl_sdio_poweroff,
};
#endif

static struct sdio_driver mwl_sdio_driver = {
	.name     = MWL_DRV_NAME,
	.id_table = mwl_sdio_id_tbl,
	.probe    = mwl_sdio_probe,
	.remove   = mwl_sdio_remove,
	.drv = {
		.owner = THIS_MODULE,
#ifdef CONFIG_PM
		.pm    = &mwl_sdio_pm_ops,
#endif
	}
};

#ifdef CONFIG_GPIOLIB
module_param(reset_pwd_gpio, uint, 0644);
MODULE_PARM_DESC(reset_pwd_gpio, "WIFI CHIP_PWD reset pin GPIO (deprecated)");
#endif

static int mwl_sdio_init_gpio_legacy(void)
{
	int ret;

	if (!gpio_is_valid(reset_pwd_gpio))
		return 0;

	ret = gpio_request_one(reset_pwd_gpio, GPIOF_OUT_INIT_HIGH, "WIFI_RESET");

	/* Only return failure code if GPIO is configured but
	 * request fails
	 */
	if (ret)
		pr_err("lrdmwl: Unable to obtain WIFI power gpio. %d\n", ret);
	else
		gpio_export(reset_pwd_gpio, false);

	return ret;
}

static int mwl_sdio_set_gpio(struct mwl_sdio_card *card, int value)
{
	if (!gpio_is_valid(card->reset_pwd_gpio))
		return -ENOSYS;

	gpio_set_value(card->reset_pwd_gpio, value);

	if (value)
		lrdmwl_delay(1500);

	return 0;
}

static int mwl_sdio_reset(struct mwl_priv *priv)
{
	struct mwl_sdio_card *card = priv->intf;
	struct sdio_func *func = card->func;
	int rc;

	sdio_claim_host(func);

	if (!mwl_sdio_set_gpio(card, 0)) {
		msleep(SDIO_DEFAULT_POWERDOWN_DELAY_MS);
		mwl_sdio_set_gpio(card, 1);
	}

	rc = mwl_sdio_mmc_hw_reset(card->func);

	sdio_release_host(func);

	return rc;
}

static int mwl_sdio_release_gpio_legacy(void)
{
	if (gpio_is_valid(reset_pwd_gpio)) {
		/* Be sure we release GPIO and leave the chip in reset
		 * when we unload
		 */
		gpio_set_value(reset_pwd_gpio, 0);
		gpio_free(reset_pwd_gpio);
	}

	return 0;
}

static int __init mwl_sdio_driver_init(void)
{
	int ret;

	ret = mwl_sdio_init_gpio_legacy();
	if (ret)
		return ret;

	ret = sdio_register_driver(&mwl_sdio_driver);
	if (ret)
		mwl_sdio_release_gpio_legacy();

	return ret;
}

static void __exit mwl_sdio_driver_exit(void)
{
	sdio_unregister_driver(&mwl_sdio_driver);

	mwl_sdio_release_gpio_legacy();
}

module_init(mwl_sdio_driver_init);
module_exit(mwl_sdio_driver_exit);

MODULE_DESCRIPTION(LRD_SDIO_DESC);
MODULE_VERSION(LRD_SDIO_VERSION);
MODULE_AUTHOR(LRD_AUTHOR);
MODULE_LICENSE("GPL v2");
