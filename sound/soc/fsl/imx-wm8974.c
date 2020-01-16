/*
 * Copyright (C) 2013-2016 Freescale Semiconductor, Inc.
 *
 * Based on imx-sgtl5000.c
 * Copyright (C) 2012 Freescale Semiconductor, Inc.
 * Copyright (C) 2012 Linaro Ltd.
 * Copyright 2017 NXP
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/i2c.h>
#include <linux/clk.h>
#include <sound/soc.h>

#include "../codecs/wm8974.h"
#include "imx-audmux.h"

#define DAI_NAME_SIZE	32

struct imx_wm8974_data {
	struct snd_soc_dai_link dai;
	struct snd_soc_card card;
	char codec_dai_name[DAI_NAME_SIZE];
	char platform_name[DAI_NAME_SIZE];
	struct clk *codec_clk;
	unsigned int clk_frequency;
};

static int imx_wm8974_dai_init(struct snd_soc_pcm_runtime *rtd)
{
	struct imx_wm8974_data *data = snd_soc_card_get_drvdata(rtd->card);
	struct device *dev = rtd->card->dev;
	int ret;
	ret = snd_soc_dai_set_sysclk(rtd->codec_dai, 0,
				     data->clk_frequency, SND_SOC_CLOCK_IN);
	if (ret) {
		dev_err(dev, "could not set codec driver MCLK clock(%u) params\n",data->clk_frequency);
		return ret;
	}

	return 0;
}

static const struct snd_soc_dapm_widget imx_wm8974_dapm_widgets[] = {
	SND_SOC_DAPM_MIC("Mic", NULL),
	SND_SOC_DAPM_SPK("Ext Spk", NULL),
};

static int imx_wm8974_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct device_node *ssi_np, *codec_np;
	struct platform_device *ssi_pdev;
	struct i2c_client *codec_dev;
	struct imx_wm8974_data *data = NULL;
	int ret;

	ssi_np = of_parse_phandle(pdev->dev.of_node, "cpu-dai", 0);
	codec_np = of_parse_phandle(pdev->dev.of_node, "audio-codec", 0);
	if (!ssi_np || !codec_np) {
		dev_err(&pdev->dev, "phandle missing or invalid\n");
		ret = -EINVAL;
		goto fail;
	}

	ssi_pdev = of_find_device_by_node(ssi_np);
	if (!ssi_pdev) {
		dev_err(&pdev->dev, "failed to find SSI platform device\n");
		ret = -EPROBE_DEFER;
		goto fail;
	}
	codec_dev = of_find_i2c_device_by_node(codec_np);
	if (!codec_dev) {
		dev_err(&pdev->dev, "failed to find codec platform device\n");
		return -EPROBE_DEFER;
	}

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data) {
		ret = -ENOMEM;
		goto fail;
	}

	data->codec_clk = devm_clk_get(&codec_dev->dev, NULL);
	if (IS_ERR(data->codec_clk)) {
		ret = PTR_ERR(data->codec_clk);
		goto fail;
	}

	data->clk_frequency = clk_get_rate(data->codec_clk);
	//clk_prepare_enable(data->codec_clk);//enabled by fsl sai driver

	data->dai.name = "HiFi";
	data->dai.stream_name = "HiFi";
	data->dai.codec_dai_name = "wm8974-hifi";
	data->dai.codec_of_node = codec_np;
	data->dai.cpu_of_node = ssi_np;
	data->dai.platform_of_node = ssi_np;
	data->dai.init = &imx_wm8974_dai_init;
	data->dai.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
			    SND_SOC_DAIFMT_CBS_CFS;

	data->card.dev = &pdev->dev;
	ret = snd_soc_of_parse_card_name(&data->card, "model");
	if (ret)
		goto fail;
	ret = snd_soc_of_parse_audio_routing(&data->card, "audio-routing");
	if (ret)
		goto fail;
	data->card.num_links = 1;
	data->card.owner = THIS_MODULE;
	data->card.dai_link = &data->dai;
	data->card.dapm_widgets = imx_wm8974_dapm_widgets;
	data->card.num_dapm_widgets = ARRAY_SIZE(imx_wm8974_dapm_widgets);

	platform_set_drvdata(pdev, &data->card);
	snd_soc_card_set_drvdata(&data->card, data);

	ret = devm_snd_soc_register_card(&pdev->dev, &data->card);
	if (ret) {
		dev_err(&pdev->dev, "snd_soc_register_card failed (%d)\n", ret);
		goto fail;
	}

	of_node_put(ssi_np);
	of_node_put(codec_np);

	return 0;

fail:
	if (data && !IS_ERR(data->codec_clk))
		clk_put(data->codec_clk);
	of_node_put(ssi_np);
	of_node_put(codec_np);

	return ret;
}

static int imx_wm8974_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);
	struct imx_wm8974_data *data = snd_soc_card_get_drvdata(card);

	clk_put(data->codec_clk);

	return 0;
}

static const struct of_device_id imx_wm8974_dt_ids[] = {
	{ .compatible = "fsl,imx-audio-wm8974", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx_wm8974_dt_ids);

static struct platform_driver imx_wm8974_driver = {
	.driver = {
		.name = "imx-wm8974",
		.pm = &snd_soc_pm_ops,
		.of_match_table = imx_wm8974_dt_ids,
	},
	.probe = imx_wm8974_probe,
	.remove = imx_wm8974_remove,
};
module_platform_driver(imx_wm8974_driver);

MODULE_AUTHOR("Vikas Gahlan <vikas.gahlan@chargepoint.com>");
MODULE_DESCRIPTION("Freescale i.MX wm8974 ASoC machine driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:imx-wm8974");
