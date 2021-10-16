// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2021, ChargePoint Inc.
 */

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>

#include <video/mipi_display.h>

#include <drm/drm_crtc.h>
#include <drm/drm_device.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>
#include <drm/drm_print.h>

struct ampire_panel {
	struct drm_panel base;
	struct mipi_dsi_device *link;

	struct backlight_device *backlight;
	struct regulator *supply;
	struct gpio_desc *enable_gpio;
	struct i2c_client *i2c;

	bool prepared;
	bool enabled;
};

struct ampire_panel_cmd {
	char cmd;
	char data;
};

static const struct ampire_panel_cmd init_code[] = {
	{ 0x7A, 0xC1 }, /* enable MIPI register config */
	{ 0x20, 0x80 }, /* HACTIVE_L */
	{ 0x21, 0xE0 }, /* VACTIVE_L */
	{ 0x22, 0x12 }, /* VACTIVE_HACTIVE_H */
	{ 0x23, 0x72 }, /* HFP_L */
	{ 0x24, 0x1E }, /* HSW_L */
	{ 0x25, 0x24 }, /* HBP_L */
	{ 0x26, 0x00 }, /* HFP_HSW_HBP_H */
	{ 0x27, 0x20 }, /* VFP */
	{ 0x28, 0x03 }, /* VSW */
	{ 0x29, 0x0A }, /* VBP */
	{ 0x34, 0x80 }, /* SYNC_EVENT_DLY */
	{ 0x36, 0x72 }, /* HFP_MIN */
	{ 0xB5, 0xA0 }, /* MIPI_PD_CK_LANE */
	{ 0x5C, 0xFF }, /* PLL_WT_LOCK */
	{ 0x2A, 0x07 }, /* BIST_POL: HS_POL=1, VS_POL=1, DE_POL=1 */
	{ 0x56, 0x92 }, /* PLL_CTRL_6: PLL_REFSEL=MIPI Clk */
	{ 0x6B, 0x71 }, /* PLL_REF_DIV */
	{ 0x69, 0x2B }, /* PLL_INT */
	{ 0x86, 0x07 }, /* DSI_LANES=4 */
	{ 0x10, 0xD5 }, /* FRC=1, BIT_SWAP=001, RGB_SWAP=101 */
	{ 0x11, 0x88 }, /* SYS_CTRL_1: Phase-adjust */
	{ 0xB6, 0x20 }, /* MIPI_FORCE_0 */
	{ 0x51, 0x20 }, /* PLL_CTRL_1 */
	{ 0x09, 0x10 }, /* CONFIG_FINISH */
};

static inline
struct ampire_panel *to_ampire_panel(struct drm_panel *panel)
{
	return container_of(panel, struct ampire_panel, base);
}

static int ampire_panel_disable(struct drm_panel *panel)
{
	struct ampire_panel *ampire = to_ampire_panel(panel);
	int err;

	if (!ampire->enabled)
		return 0;

	if (ampire->backlight)
		backlight_disable(ampire->backlight);

	if (!ampire->i2c) {
		err = mipi_dsi_dcs_set_display_off(ampire->link);
		if (err < 0)
			DRM_DEV_ERROR(panel->dev,
						  "failed to set display off: %d\n", err);
	}

	ampire->enabled = false;

	return 0;
}

static int ampire_panel_unprepare(struct drm_panel *panel)
{
	struct ampire_panel *ampire = to_ampire_panel(panel);
	int err;

	if (!ampire->prepared)
		return 0;

	if (!ampire->i2c) {
		err = mipi_dsi_dcs_enter_sleep_mode(ampire->link);
		if (err < 0) {
			DRM_DEV_ERROR(panel->dev,
						  "failed to enter sleep mode: %d\n", err);
			return err;
		}
		msleep(120);
	}

	gpiod_set_value_cansleep(ampire->enable_gpio, 0);

	err = regulator_disable(ampire->supply);
	if (err < 0)
		return err;

	ampire->prepared = false;

	return 0;
}

static int ampire_panel_prepare(struct drm_panel *panel)
{
	struct ampire_panel *ampire = to_ampire_panel(panel);
	int err, regulator_err;
	unsigned int i;

	if (ampire->prepared)
		return 0;

	gpiod_set_value_cansleep(ampire->enable_gpio, 0);

	err = regulator_enable(ampire->supply);
	if (err < 0)
		return err;

	usleep_range(15000, 16000);

	gpiod_set_value_cansleep(ampire->enable_gpio, 1);

	usleep_range(150000, 160000);

	if (ampire->i2c) {
		// skip 1st entry which is for MIPI-DSI mode
		for (i = 1; i < ARRAY_SIZE(init_code); i++) {
			err = i2c_master_send(ampire->i2c, (const char *) &init_code[i],
								  sizeof(struct ampire_panel_cmd));
			if (err < 0) {
				DRM_DEV_ERROR(panel->dev,
							  "failed write i2c init cmds: %d\n", err);
				goto poweroff;
			}
		}
	} else {
		for (i = 0; i < ARRAY_SIZE(init_code); i++) {
			err = mipi_dsi_generic_write(ampire->link, &init_code[i],
						sizeof(struct ampire_panel_cmd));
			if (err < 0) {
				DRM_DEV_ERROR(panel->dev, "failed write init cmds: %d\n",
					      err);
				goto poweroff;
			}
		}
	}

	usleep_range(10000, 11000);

	ampire->prepared = true;

	return 0;

poweroff:
	gpiod_set_value_cansleep(ampire->enable_gpio, 0);

	regulator_err = regulator_disable(ampire->supply);
	if (regulator_err) DRM_DEV_ERROR(panel->dev,
						 "failed to disable regulator: %d\n", regulator_err);

	return err;
}

static int ampire_panel_enable(struct drm_panel *panel)
{
	struct ampire_panel *ampire = to_ampire_panel(panel);
	int ret;

	if (ampire->enabled)
		return 0;

	if (ampire->backlight) {
		ret = backlight_enable(ampire->backlight);
		if (ret) {
			DRM_DEV_ERROR(panel->dev,
				      "Failed to enable backlight %d\n", ret);
			return ret;
		}
	}

	ampire->enabled = true;

	return 0;
}

static const struct drm_display_mode default_mode = {
	.clock = 25175,
	.hdisplay = 640,
	.hsync_start = 640 + 16,
	.hsync_end = 640 + 16 + 30,
	.htotal = 640 + 16 + 30 + 114,
	.vdisplay = 480,
	.vsync_start = 480 + 10,
	.vsync_end = 480 + 10 + 3,
	.vtotal = 480 + 10 + 3 + 32,
	.flags = DRM_MODE_FLAG_PVSYNC | DRM_MODE_FLAG_PHSYNC,
};

static int ampire_panel_get_modes(struct drm_panel *panel, struct drm_connector *connector)
{
	struct drm_display_mode *mode;

	mode = drm_mode_duplicate(connector->dev, &default_mode);
	if (!mode) {
		DRM_DEV_ERROR(panel->dev, "failed to add mode %ux%ux\n",
			      default_mode.hdisplay, default_mode.vdisplay);
		return -ENOMEM;
	}

	drm_mode_set_name(mode);

	connector->display_info.width_mm = 117;
	connector->display_info.height_mm = 92;

	drm_mode_probed_add(connector, mode);

	return 1;
}

static const struct drm_panel_funcs ampire_panel_funcs = {
	.disable = ampire_panel_disable,
	.unprepare = ampire_panel_unprepare,
	.prepare = ampire_panel_prepare,
	.enable = ampire_panel_enable,
	.get_modes = ampire_panel_get_modes,
};

static const struct of_device_id ampire_of_match[] = {
	{ .compatible = "ampire,am640480-icn6211", },
	{ }
};
MODULE_DEVICE_TABLE(of, ampire_of_match);

static int ampire_panel_add(struct ampire_panel *ampire)
{
	struct device *dev = &ampire->link->dev;
	int err;

	ampire->supply = devm_regulator_get(dev, "power");
	if (IS_ERR(ampire->supply))
		return PTR_ERR(ampire->supply);

	ampire->enable_gpio = devm_gpiod_get_optional(dev, "enable",
							   GPIOD_OUT_HIGH);
	if (IS_ERR(ampire->enable_gpio)) {
		err = PTR_ERR(ampire->enable_gpio);
		dev_dbg(dev, "failed to get enable gpio: %d\n", err);
		ampire->enable_gpio = NULL;
	}

	ampire->backlight = devm_of_find_backlight(dev);

	drm_panel_init(&ampire->base, &ampire->link->dev, &ampire_panel_funcs, DRM_MODE_CONNECTOR_DSI);

	drm_panel_add(&ampire->base);
	return 0;
}

static void ampire_panel_del(struct ampire_panel *ampire)
{
	drm_panel_remove(&ampire->base);
}

static int ampire_panel_probe(struct mipi_dsi_device *dsi)
{
	struct ampire_panel *ampire;
	struct device_node *np;
	int err;

	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO |
		MIPI_DSI_MODE_VIDEO_SYNC_PULSE | MIPI_DSI_MODE_LPM;

	ampire = devm_kzalloc(&dsi->dev, sizeof(*ampire), GFP_KERNEL);
	if (!ampire)
		return -ENOMEM;

	mipi_dsi_set_drvdata(dsi, ampire);
	ampire->link = dsi;

	np = of_parse_phandle(dsi->dev.of_node, "i2c-dev", 0);
	if (!IS_ERR_OR_NULL(np)) {
		ampire->i2c = of_find_i2c_device_by_node(np);
	}

	err = ampire_panel_add(ampire);
	if (err < 0)
		return err;

	//return mipi_dsi_attach(dsi);
	err = mipi_dsi_attach(dsi);
	if(err < 0)
		ampire_panel_del(ampire);

	return err;
}

static int ampire_panel_remove(struct mipi_dsi_device *dsi)
{
	struct ampire_panel *ampire = mipi_dsi_get_drvdata(dsi);
	int err;

	err = ampire_panel_unprepare(&ampire->base);
	if (err < 0)
		DRM_DEV_ERROR(&dsi->dev, "failed to unprepare panel: %d\n",
			      err);

	err = ampire_panel_disable(&ampire->base);
	if (err < 0)
		DRM_DEV_ERROR(&dsi->dev, "failed to disable panel: %d\n", err);

	err = mipi_dsi_detach(dsi);
	if (err < 0)
		DRM_DEV_ERROR(&dsi->dev, "failed to detach from DSI host: %d\n",
			      err);

	ampire_panel_del(ampire);

	return 0;
}

static void ampire_panel_shutdown(struct mipi_dsi_device *dsi)
{
	struct ampire_panel *ampire = mipi_dsi_get_drvdata(dsi);

	ampire_panel_unprepare(&ampire->base);
	ampire_panel_disable(&ampire->base);
}

static struct mipi_dsi_driver ampire_panel_driver = {
	.driver = {
		.name = "panel-ampire-am640480",
		.of_match_table = ampire_of_match,
	},
	.probe = ampire_panel_probe,
	.remove = ampire_panel_remove,
	.shutdown = ampire_panel_shutdown,
};

module_mipi_dsi_driver(ampire_panel_driver);

MODULE_AUTHOR("Michael Fairman <michael.fairman@chargepoint.com>");
MODULE_DESCRIPTION("Ampire AM640480/ICN6211 panel driver");
MODULE_LICENSE("GPL v2");
