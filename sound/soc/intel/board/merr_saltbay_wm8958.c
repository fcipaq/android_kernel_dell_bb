/*
 *  merr_saltbay_wm8958.c - ASoc Machine driver for Intel Merrfield MID platform
 *
 *  Copyright (C) 2013 Intel Corp
 *  Author: Vinod Koul <vinod.koul@intel.com>
 *  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */
#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/async.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <asm/intel_mid_rpmsg.h>
#include <asm/platform_mrfld_audio.h>
#include <asm/intel_sst_mrfld.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/jack.h>
#include <linux/input.h>
#include <asm/intel-mid.h>

#include <linux/mfd/wm8994/core.h>
#include <linux/mfd/wm8994/registers.h>
#include <linux/mfd/wm8994/pdata.h>
#include "../../codecs/wm8994.h"

#include <linux/i2c.h>

#define I2C_ADAPTER 0x01
#define I2C_ADDRESS_LEFT 0x34
#define I2C_ADDRESS_RIGHT 0x35

static struct i2c_board_info tfa9890_board_info_left = {
	I2C_BOARD_INFO("tfa9890_i2c_left", I2C_ADDRESS_LEFT)
};

static struct i2c_board_info tfa9890_board_info_right = {
	I2C_BOARD_INFO("tfa9890_i2c_right", I2C_ADDRESS_RIGHT)
};

static struct i2c_client *i2c_client_left;
static struct i2c_client *i2c_client_right;

static void tfa9890_register(void)
{
	struct i2c_adapter *adapter;
	adapter = i2c_get_adapter(I2C_ADAPTER);
	i2c_client_left = i2c_new_device(adapter, &tfa9890_board_info_left);
	i2c_client_right = i2c_new_device(adapter, &tfa9890_board_info_right);
}

/* Codec PLL output clk rate */
#define CODEC_SYSCLK_RATE			24576000
/* Input clock to codec at MCLK1 PIN */
#define CODEC_IN_MCLK1_RATE			19200000
/* Input clock to codec at MCLK2 PIN */
#define CODEC_IN_MCLK2_RATE			32768
/*  define to select between MCLK1 and MCLK2 input to codec as its clock */
#define CODEC_IN_MCLK1				1
#define CODEC_IN_MCLK2				2

/* Register address for OSC Clock */
#define MERR_OSC_CLKOUT_CTRL0_REG_ADDR  0xFF00BC04
/* Size of osc clock register */
#define MERR_OSC_CLKOUT_CTRL0_REG_SIZE  4

struct mrfld_8958_mc_private {
	struct snd_soc_jack jack;
	int jack_retry;
	void __iomem    *osc_clk0_reg;
	int spk_gpio;
	/* External amplifier */
	int ext_amp_init;
	int ext_amp_reset_gpio;
};

/* TODO: find better way of doing this */
static struct snd_soc_dai *find_codec_dai(struct snd_soc_card *card, const char *dai_name)
{
	int i;
	for (i = 0; i < card->num_rtd; i++) {
			if (!strcmp(card->rtd[i].codec_dai->name, dai_name))
					return card->rtd[i].codec_dai;
	}
	pr_err("%s: unable to find codec dai\n", __func__);
	/* this should never occur */
	WARN_ON(1);
	return NULL;
}

/* set_osc_clk0-	enable/disables the osc clock0
 * addr:		address of the register to write to
 * enable:		bool to enable or disable the clock
 */
static inline void set_soc_osc_clk0(void __iomem *addr, bool enable)
{
	u32 osc_clk_ctrl;

	osc_clk_ctrl = readl(addr);
	if (enable)
		osc_clk_ctrl |= BIT(31);
	else
		osc_clk_ctrl &= ~(BIT(31));

	pr_debug("%s: enable:%d val 0x%x\n", __func__, enable, osc_clk_ctrl);

	writel(osc_clk_ctrl, addr);
}


static inline struct snd_soc_codec *mrfld_8958_get_codec(struct snd_soc_card *card)
{
	bool found = false;
	struct snd_soc_codec *codec;

	list_for_each_entry(codec, &card->codec_dev_list, card_list) {
		if (!strstr(codec->name, "wm8994-codec")) {
			pr_debug("codec was %s", codec->name);
			continue;
		} else {
			found = true;
			break;
		}
	}
	if (found == false) {
		pr_err("%s: cant find codec", __func__);
		return NULL;
	}
	return codec;
}

/* Function to switch the input clock for codec,  When audio is in
 * progress input clock to codec will be through MCLK1 which is 19.2MHz
 * while in off state input clock to codec will be through 32KHz through
 * MCLK2
 * card	: Sound card structure
 * src	: Input clock source to codec
 */
static int mrfld_8958_set_codec_clk(struct snd_soc_card *card, int src, const char *codec_dai_name)
{
	struct snd_soc_dai *aif_dai = NULL;
	int ret;

	pr_debug("%s called", __func__);

	/* Assume card->rtd[0] = AIF1 by default */
	if (codec_dai_name) {
		aif_dai = find_codec_dai(card, codec_dai_name);
	} else {
		aif_dai = card->rtd[0].codec_dai;
	}
	if(!aif_dai){
		pr_err("Failed to aif_dai\n");
		return -EINVAL;
	}

	/* Turn ON the PLL to generate required sysclk rate
	 * from MCLK1 */
	ret = snd_soc_dai_set_pll(aif_dai,
				  WM8994_FLL1, WM8994_FLL_SRC_MCLK1,
				  CODEC_IN_MCLK1_RATE, CODEC_SYSCLK_RATE);
	if (ret < 0) {
		pr_err("Failed to start FLL: %d\n", ret);
		return ret;
	}
	/* Switch to MCLK1 input */
	ret = snd_soc_dai_set_sysclk(aif_dai, WM8994_SYSCLK_FLL1,
				     CODEC_SYSCLK_RATE, SND_SOC_CLOCK_IN);
	if (ret < 0) {
		pr_err("Failed to set codec sysclk configuration %d\n", ret);
		return ret;
	}

	return 0;
}

static int mrfld_wm8958_set_clk_fmt(struct snd_soc_dai *codec_dai)
{
	unsigned int fmt;
	int ret = 0;
	struct snd_soc_card *card = codec_dai->card;
	struct mrfld_8958_mc_private *ctx = snd_soc_card_get_drvdata(card);

	pr_debug("%s called", __func__);

	/* Enable the osc clock at start so that it gets settling time */
	set_soc_osc_clk0(ctx->osc_clk0_reg, true);

	if (codec_dai->id == 1) {
		/* Set TDM slot available only on AIF 1 */
		ret = snd_soc_dai_set_tdm_slot(codec_dai, 0, 0, 4, SNDRV_PCM_FORMAT_S24_LE);
		if (ret < 0) {
			pr_err("can't set codec pcm format %d\n", ret);
			return ret;
		}

		/* WM8958 Slave Mode */
		fmt =   SND_SOC_DAIFMT_DSP_B | SND_SOC_DAIFMT_IB_NF
			| SND_SOC_DAIFMT_CBS_CFS;
		ret = snd_soc_dai_set_fmt(codec_dai, fmt);
		if (ret < 0) {
			pr_err("can't set codec DAI 1 configuration %d\n", ret);
			return ret;
		}

		/* FIXME: move this to SYS_CLOCK event handler when codec driver
		 * dependency is clean.
		 */
		/* Switch to 19.2MHz MCLK1 input clock for codec */
		ret = mrfld_8958_set_codec_clk(card, CODEC_IN_MCLK1, "wm8994-aif1");

	} else if (codec_dai->id == 2) {
		pr_debug("setting I2S to master mode in AIF2");
		/* WM8958 Master I2S Mode */
		fmt = SND_SOC_DAIFMT_CBM_CFM | SND_SOC_DAIFMT_I2S
			| SND_SOC_DAIFMT_NB_NF;
		ret = snd_soc_dai_set_fmt(codec_dai, fmt);
		if (ret < 0) {
			pr_err("can't set codec DAI 2 format %d\n", ret);
			return ret;
		}
		/* Switch to 19.2MHz MCLK1 input clock for codec */
		ret = mrfld_8958_set_codec_clk(card, CODEC_IN_MCLK1, "wm8994-aif2");
		if (ret < 0) {
			pr_err("can't set codec DAI 2 clock %d\n", ret);
			return ret;
		}

	} else {
		return -EINVAL;
	}

	return ret;
}

static int mrfld_8958_hw_params(struct snd_pcm_substream *substream,
			   struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;

	return mrfld_wm8958_set_clk_fmt(codec_dai);
}

static int mrfld_8958_hw_params_ext_amp(struct snd_pcm_substream *substream,
			   struct snd_pcm_hw_params *params)
{
	int ret = 0;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;

	pr_debug("%s called", __func__);

	ret = mrfld_wm8958_set_clk_fmt(codec_dai);
	if (ret < 0) {
		pr_err("can't set codec clk format %d\n", ret);
		return ret;
	}
	return ret;
}

static int mrfld_wm8958_compr_set_params(struct snd_compr_stream *cstream)
{
	struct snd_soc_pcm_runtime *rtd = cstream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;

	return mrfld_wm8958_set_clk_fmt(codec_dai);
}

static int mrfld_8958_set_bias_level(struct snd_soc_card *card,
		struct snd_soc_dapm_context *dapm,
		enum snd_soc_bias_level level)
{
	struct snd_soc_dai *aif1_dai = card->rtd[0].codec_dai;
	int ret = 0;

	if (dapm->dev != aif1_dai->dev)
		return 0;
	switch (level) {
	case SND_SOC_BIAS_PREPARE:
		if (card->dapm.bias_level == SND_SOC_BIAS_STANDBY)
			ret = mrfld_wm8958_set_clk_fmt(aif1_dai);
		break;
	default:
		break;
	}
	pr_debug("%s card(%s)->bias_level %u\n", __func__, card->name,
			card->dapm.bias_level);
	return ret;
}
static int mrfld_8958_set_bias_level_post(struct snd_soc_card *card,
		 struct snd_soc_dapm_context *dapm,
		 enum snd_soc_bias_level level)
{
	struct snd_soc_dai *aif1_dai = card->rtd[0].codec_dai;
	struct mrfld_8958_mc_private *ctx = snd_soc_card_get_drvdata(card);
	int ret = 0;

	if (dapm->dev != aif1_dai->dev)
		return 0;

	switch (level) {
	case SND_SOC_BIAS_STANDBY:
		/* We are in stabdba down so */
		/* Switch to 32KHz MCLK2 input clock for codec
		 */
		ret = mrfld_8958_set_codec_clk(card, CODEC_IN_MCLK2, "wm8994-aif1");
		/* Turn off 19.2MHz soc osc clock */
		set_soc_osc_clk0(ctx->osc_clk0_reg, false);
		break;
	default:
		break;
	}
	card->dapm.bias_level = level;
	pr_debug("%s card(%s)->bias_level %u\n", __func__, card->name,
			card->dapm.bias_level);
	return ret;
}

static int mrfld_8958_set_spk_boost(struct snd_soc_dapm_widget *w,
				struct snd_kcontrol *k, int event)
{
	struct snd_soc_dapm_context *dapm = w->dapm;
	struct snd_soc_card *card = dapm->card;
	struct mrfld_8958_mc_private *ctx = snd_soc_card_get_drvdata(card);
	int ret = 0;

	pr_debug("%s: ON? %d\n", __func__, SND_SOC_DAPM_EVENT_ON(event));

	if (SND_SOC_DAPM_EVENT_ON(event))
		gpio_set_value((unsigned)ctx->spk_gpio, 1);
	else if (SND_SOC_DAPM_EVENT_OFF(event))
		gpio_set_value((unsigned)ctx->spk_gpio, 0);

	return ret;
}

static int wm8958_ext_amp_cal_get(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct mrfld_8958_mc_private *ctx =
			snd_soc_card_get_drvdata(codec->card);

	pr_debug("%s: ctx->ext_amp_init=%d\n", __func__, ctx->ext_amp_init);
	ucontrol->value.enumerated.item[0] = ctx->ext_amp_init;

	return 0;
}

static int wm8958_ext_amp_cal_set(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct mrfld_8958_mc_private *ctx =
		snd_soc_card_get_drvdata(codec->card);

	int value = ucontrol->value.enumerated.item[0];
	pr_debug("%s: ctx->ext_amp_init=%d, new value=%d\n", __func__,
		 ctx->ext_amp_init, value);

	if (value < 0 || value > 1)
		return -EINVAL;
	else
		ctx->ext_amp_init = value;

	return 0;
}

static int ext_amp_clock_control(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *k, int event)
{
	struct snd_soc_card *card = w->dapm->card;
	int ret;

	pr_debug("In %s\n", __func__);

	if (SND_SOC_DAPM_EVENT_ON(event)) {
		pr_debug("Enabling AIF2 Clock\n");
		ret = mrfld_8958_set_codec_clk(card, CODEC_IN_MCLK1, "wm8994-aif2");

#ifdef DEBUG_TFA_STATUS
		short data;
		i2c_master_send(i2c_client_left, "\x00", 1);
		i2c_master_recv(i2c_client_left, (char *) &data, 2);
		pr_debug("SND_SOC_DAPM_EVENT_ON: TFA LEFT: %x", data);
		i2c_master_send(i2c_client_right, "\x00", 1);
		i2c_master_recv(i2c_client_right, (char *) &data, 2);
		pr_debug("SND_SOC_DAPM_EVENT_ON: TFA RIGHT: %x", data);
#endif

		return ret;
	} else { /* SND_SOC_DAPM_EVENT_OFF */

#ifdef DEBUG_TFA_STATUS
		short data;
		i2c_master_send(i2c_client_left, "\x00", 1);
		i2c_master_recv(i2c_client_left, (char *) &data, 2);
		pr_debug("SND_SOC_DAPM_EVENT_OFF: TFA LEFT: %x", data);
		i2c_master_send(i2c_client_right, "\x00", 1);
		i2c_master_recv(i2c_client_right, (char *) &data, 2);
		pr_debug("SND_SOC_DAPM_EVENT_OFF: TFA RIGHT: %x", data);
#endif

		pr_debug("Muting TFA9890s\n");
		/* AMPE -> 0 */
		if (i2c_master_send(i2c_client_left, "\x09\x82\x74", 3) != 3)
			pr_err("i2c_client_left: failed to set AMPE to 0");
		if (i2c_master_send(i2c_client_right, "\x09\x82\x74", 3) != 3)
			pr_err("i2c_client_right: failed to set AMPE to 0");

		/* Soft mute sleep */
		msleep(100);

		pr_debug("Switching off TFA9890s\n");
		/* PWDN => 1 */
		if (i2c_master_send(i2c_client_left, "\x09\x82\x75", 3) != 3)
			pr_err("i2c_client_left: failed to set PWDN  1");
		if (i2c_master_send(i2c_client_right, "\x09\x82\x75", 3) != 3)
			pr_err("i2c_client_right: failed to set PWDN  to 1");

		/* Sleep to accommodate for processing time (possibly unnecessary) */
		usleep_range(5 * USEC_PER_MSEC, 5 * USEC_PER_MSEC);

		pr_debug("AIF2 path disabled\n");
		return 0;
	}
}

static int ext_amp_i2s_clk_control(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *k, int event)
{
	struct snd_soc_card *card = w->dapm->card;
	struct mrfld_8958_mc_private *ctx = snd_soc_card_get_drvdata(card);

	if (ctx->ext_amp_init == 1) {
		pr_debug("Powering up TFA9890s\n");

		/* PWDN => 0 and AMPE => 1 */
		if (i2c_master_send(i2c_client_left, "\x09\x82\x7c", 3) != 3)
			pr_err("i2c_client_left: failed to set AMPE to 1 and PWDN => 0");
		if (i2c_master_send(i2c_client_right, "\x09\x82\x7c", 3) != 3)
			pr_err("i2c_client_right: failed to set AMPE to 1 and PWDN => 0");
	} else {
		pr_debug("Not Powering up TFA9890s - pending amplifier init\n");
	}

	return 0;
}

static const struct snd_soc_dapm_widget widgets[] = {
	SND_SOC_DAPM_HP("Headphones", NULL),
	SND_SOC_DAPM_MIC("AMIC", NULL),
	SND_SOC_DAPM_MIC("DMIC", NULL),
	SND_SOC_DAPM_SUPPLY("EXT_AMP_CONTROL", SND_SOC_NOPM, 0, 0,
			ext_amp_clock_control,
			SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_POST_SUPPLY("EXT_AMP_I2S_CONTROL", SND_SOC_NOPM, 0, 0,
			ext_amp_i2s_clk_control,
			SND_SOC_DAPM_POST_PMU),
};
static const struct snd_soc_dapm_widget spk_boost_widget[] = {
	/* DAPM route is added only for Moorefield */
	SND_SOC_DAPM_SUPPLY("SPK_BOOST", SND_SOC_NOPM, 0, 0,
			mrfld_8958_set_spk_boost,
			SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
};

static const struct snd_kcontrol_new wm8958_ext_amp_controls[] = {
	SOC_SINGLE_BOOL_EXT("External Amplifiers Initialized", 0,
			    wm8958_ext_amp_cal_get, wm8958_ext_amp_cal_set),
};

static const struct snd_soc_dapm_route map[] = {
	{ "Headphones", NULL, "HPOUT1L" },
	{ "Headphones", NULL, "HPOUT1R" },

	/* saltbay uses 2 DMICs, other configs may use more so change below
	 * accordingly
	 */
	{ "DMIC", NULL, "MICBIAS1" },
	{ "DMIC1DAT", NULL, "DMIC" },
	{ "DMIC2DAT", NULL, "DMIC" },
	/*{ "DMIC3DAT", NULL, "DMIC" },*/
	/*{ "DMIC4DAT", NULL, "DMIC" },*/

	/* MICBIAS2 is connected as Bias for AMIC so we link it
	 * here. Also AMIC wires up to IN1LP pin.
	 * DMIC is externally connected to 1.8V rail, so no link rqd.
	 */
	{ "AMIC", NULL, "MICBIAS2" },
	{ "IN1LP", NULL, "AMIC" },

	/* SWM map link the SWM outs to codec AIF */
	{ "AIF1DAC1L", NULL, "Codec OUT0"  },
	{ "AIF1DAC1R", NULL, "Codec OUT0"  },
	{ "AIF1DAC2L", NULL, "Codec OUT1"  },
	{ "AIF1DAC2R", NULL, "Codec OUT1"  },
	{ "Codec IN0", NULL, "AIF1ADC1L" },
	{ "Codec IN0", NULL, "AIF1ADC1R" },
	{ "Codec IN1", NULL, "AIF1ADC1L" },
	{ "Codec IN1", NULL, "AIF1ADC1R" },
};

static const struct snd_soc_dapm_route map_bb[] = {
	{ "Headphones", NULL, "HPOUT1L" },
	{ "Headphones", NULL, "HPOUT1R" },

	/* saltbay uses 2 DMICs, other configs may use more so change below
	 * accordingly
	 */
	{ "DMIC1DAT", NULL, "DMIC" },
	{ "DMIC2DAT", NULL, "DMIC" },
	/*{ "DMIC3DAT", NULL, "DMIC" },*/
	/*{ "DMIC4DAT", NULL, "DMIC" },*/

	/* MICBIAS2 is connected as Bias for AMIC so we link it
	 * here. Also AMIC wires up to IN1LP pin.
	 * DMIC is externally connected to 1.8V rail, so no link rqd.
	 */
	{ "AMIC", NULL, "MICBIAS2" },
	{ "IN1LP", NULL, "AMIC" },

	/* SWM map link the SWM outs to codec AIF */
	{ "AIF1DAC1L", NULL, "Codec OUT0"  },
	{ "AIF1DAC1R", NULL, "Codec OUT0"  },
	{ "AIF1DAC2L", NULL, "Codec OUT1"  },
	{ "AIF1DAC2R", NULL, "Codec OUT1"  },
	{ "Codec IN0", NULL, "AIF1ADC1L" },
	{ "Codec IN0", NULL, "AIF1ADC1R" },
	{ "Codec IN1", NULL, "AIF1ADC1L" },
	{ "Codec IN1", NULL, "AIF1ADC1R" },
	{ "AIF2 Capture", NULL, "EXT_AMP_CONTROL" },
	{ "AIF2CLK", NULL, "EXT_AMP_I2S_CONTROL" },
};

static const struct snd_soc_dapm_route mofd_spk_boost_map[] = {
	{"SPKOUTLP", NULL, "SPK_BOOST"},
	{"SPKOUTLN", NULL, "SPK_BOOST"},
	{"SPKOUTRP", NULL, "SPK_BOOST"},
	{"SPKOUTRN", NULL, "SPK_BOOST"},
};

static const struct wm8958_micd_rate micdet_rates[] = {
	{ 32768,       true,  1, 4 },
	{ 32768,       false, 1, 1 },
	{ 44100 * 256, true,  7, 10 },
	{ 44100 * 256, false, 7, 10 },
};

static void wm8958_custom_micd_set_rate(struct snd_soc_codec *codec)
{
	struct wm8994_priv *wm8994 = snd_soc_codec_get_drvdata(codec);
	struct wm8994 *control = dev_get_drvdata(codec->dev->parent);
	int best, i, sysclk, val;
	bool idle;
	const struct wm8958_micd_rate *rates;
	int num_rates;

	idle = !wm8994->jack_mic;

	sysclk = snd_soc_read(codec, WM8994_CLOCKING_1);
	if (sysclk & WM8994_SYSCLK_SRC)
		sysclk = wm8994->aifclk[1];
	else
		sysclk = wm8994->aifclk[0];

	if (control->pdata.micd_rates) {
		rates = control->pdata.micd_rates;
		num_rates = control->pdata.num_micd_rates;
	} else {
		rates = micdet_rates;
		num_rates = ARRAY_SIZE(micdet_rates);
	}

	best = 0;
	for (i = 0; i < num_rates; i++) {
		if (rates[i].idle != idle)
			continue;
		if (abs(rates[i].sysclk - sysclk) <
		    abs(rates[best].sysclk - sysclk))
			best = i;
		else if (rates[best].idle != idle)
			best = i;
	}

	val = rates[best].start << WM8958_MICD_BIAS_STARTTIME_SHIFT
		| rates[best].rate << WM8958_MICD_RATE_SHIFT;

	dev_dbg(codec->dev, "MICD rate %d,%d for %dHz %s\n",
		rates[best].start, rates[best].rate, sysclk,
		idle ? "idle" : "active");

	snd_soc_update_bits(codec, WM8958_MIC_DETECT_1,
			    WM8958_MICD_BIAS_STARTTIME_MASK |
			    WM8958_MICD_RATE_MASK, val);
}

static void wm8958_custom_mic_id(void *data, u16 status)
{
	struct snd_soc_codec *codec = data;
	struct wm8994_priv *wm8994 = snd_soc_codec_get_drvdata(codec);

	dev_dbg(codec->dev, "wm8958 custom mic id called with status %x\n",
		status);

	/* Either nothing present or just starting detection */
	if (!(status & WM8958_MICD_STS)) {
		/* If nothing present then clear our statuses */
		dev_dbg(codec->dev, "Detected open circuit\n");

		schedule_delayed_work(&wm8994->open_circuit_work,
				      msecs_to_jiffies(2500));
		return;
	}

	schedule_delayed_work(&wm8994->micd_set_custom_rate_work,
		msecs_to_jiffies(wm8994->wm8994->pdata.micb_en_delay));

	/* If the measurement is showing a high impedence we've got a
	 * microphone.
	 */
	if (status & 0x600) {
		dev_dbg(codec->dev, "Detected microphone\n");

		wm8994->mic_detecting = false;
		wm8994->jack_mic = true;
		wm8994->headphone_detected = false;

		snd_soc_jack_report(wm8994->micdet[0].jack, SND_JACK_HEADSET,
				    SND_JACK_HEADSET);
	}


	if (status & 0xfc) {
		dev_dbg(codec->dev, "Detected headphone\n");

		/* Partial inserts of headsets with complete insert
		 * after an indeterminate amount of time require
		 * continouous micdetect enabled (until open circuit
		 * or headset is detected)
		 * */
		wm8994->mic_detecting = true;

		wm8994->jack_mic = false;
		wm8994->headphone_detected = true;

		snd_soc_jack_report(wm8994->micdet[0].jack, SND_JACK_HEADPHONE,
				    SND_JACK_HEADSET);
	}
}

static int mrfld_8958_init(struct snd_soc_pcm_runtime *runtime)
{
	int ret;
	unsigned int fmt;
	struct snd_soc_codec *codec = runtime->codec;
	struct snd_soc_dapm_context *dapm = &codec->dapm;
	struct snd_soc_card *card = runtime->card;
	struct snd_soc_dapm_context *card_dapm = &card->dapm;
	struct snd_soc_dai *aif1_dai = card->rtd[0].codec_dai;
	struct mrfld_8958_mc_private *ctx = snd_soc_card_get_drvdata(card);

	pr_debug("Entry %s\n", __func__);

	/* External amplifier initialization */
	ctx->ext_amp_init = 0;
	snd_soc_add_codec_controls(codec, wm8958_ext_amp_controls,
				   ARRAY_SIZE(wm8958_ext_amp_controls));
	tfa9890_register();

	/* Add missing codec pointer to card dapm context, which will be initialized
	 * to all widgets defined in this file */
	card->dapm.codec = codec;

	ret = mrfld_8958_set_codec_clk(card, CODEC_IN_MCLK1, "wm8994-aif1");
	ret = mrfld_8958_set_codec_clk(card, CODEC_IN_MCLK1, "wm8994-aif2");

	ret = snd_soc_dai_set_tdm_slot(aif1_dai, 0, 0, 4, SNDRV_PCM_FORMAT_S24_LE);
	if (ret < 0) {
		pr_err("can't set codec pcm format %d\n", ret);
		return ret;
	}

	/* WM8958 slave Mode */
	fmt =   SND_SOC_DAIFMT_DSP_B | SND_SOC_DAIFMT_IB_NF
		| SND_SOC_DAIFMT_CBS_CFS;
	ret = snd_soc_dai_set_fmt(aif1_dai, fmt);
	if (ret < 0) {
		pr_err("can't set codec DAI configuration %d\n", ret);
		return ret;
	}

	mrfld_8958_set_bias_level(card, dapm, SND_SOC_BIAS_OFF);
	card->dapm.idle_bias_off = true;

	/* these pins are not used in SB config so mark as nc
	 *
	 * LINEOUT1, 2
	 * IN1R
	 * DMICDAT2
	 */
	snd_soc_dapm_nc_pin(dapm, "DMIC2DAT");
	snd_soc_dapm_nc_pin(dapm, "LINEOUT1P");
	snd_soc_dapm_nc_pin(dapm, "LINEOUT1N");
	snd_soc_dapm_nc_pin(dapm, "LINEOUT2P");
	snd_soc_dapm_nc_pin(dapm, "LINEOUT2N");
	snd_soc_dapm_nc_pin(dapm, "IN1RN");
	snd_soc_dapm_nc_pin(dapm, "IN1RP");

	if (ctx->spk_gpio >= 0) {
		snd_soc_dapm_new_controls(card_dapm, spk_boost_widget,
					ARRAY_SIZE(spk_boost_widget));
		snd_soc_dapm_add_routes(card_dapm, mofd_spk_boost_map,
				ARRAY_SIZE(mofd_spk_boost_map));
	}
	/* Force enable VMID to avoid cold latency constraints */
	snd_soc_dapm_force_enable_pin(dapm, "VMID");
	snd_soc_dapm_sync(dapm);

	ctx->jack_retry = 0;
	ret = snd_soc_jack_new(codec, "Intel MID Audio Jack",
			       SND_JACK_HEADSET | SND_JACK_HEADPHONE |
				SND_JACK_BTN_0 | SND_JACK_BTN_1,
				&ctx->jack);
	if (ret) {
		pr_err("jack creation failed\n");
		return ret;
	}

	snd_jack_set_key(ctx->jack.jack, SND_JACK_BTN_1, KEY_MEDIA);
	snd_jack_set_key(ctx->jack.jack, SND_JACK_BTN_0, KEY_MEDIA);

	snd_soc_update_bits(codec, WM8958_MICBIAS2, WM8958_MICB2_LVL_MASK,
				WM8958_MICB2_LVL_2P6V << WM8958_MICB2_LVL_SHIFT);

	wm8958_mic_detect(codec, &ctx->jack, NULL, NULL,
			  wm8958_custom_mic_id, codec);

	wm8958_micd_set_custom_rate(codec, wm8958_custom_micd_set_rate, codec);

	snd_soc_update_bits(codec, WM8994_AIF1_DAC1_FILTERS_1, WM8994_AIF1DAC1_MUTE, 0);
	snd_soc_update_bits(codec, WM8994_AIF1_DAC2_FILTERS_1, WM8994_AIF1DAC2_MUTE, 0);

	/* Micbias1 is always off, so for pm optimizations make sure the micbias1
	 * discharge bit is set to floating to avoid discharge in disable state
	 */
	snd_soc_update_bits(codec, WM8958_MICBIAS1, WM8958_MICB1_DISCH, 0);

	return 0;
}

static unsigned int rates_8000_16000[] = {
	8000,
	16000,
};

static struct snd_pcm_hw_constraint_list constraints_8000_16000 = {
	.count = ARRAY_SIZE(rates_8000_16000),
	.list  = rates_8000_16000,
};

static unsigned int rates_48000[] = {
	48000,
};

static struct snd_pcm_hw_constraint_list constraints_48000 = {
	.count = ARRAY_SIZE(rates_48000),
	.list  = rates_48000,
};

static int mrfld_8958_startup(struct snd_pcm_substream *substream)
{
	return snd_pcm_hw_constraint_list(substream->runtime, 0,
			SNDRV_PCM_HW_PARAM_RATE,
			&constraints_48000);
}

static struct snd_soc_ops mrfld_8958_ops = {
	.startup = mrfld_8958_startup,
	.hw_params = mrfld_8958_hw_params,
};

static int mrfld_8958_8k_16k_startup(struct snd_pcm_substream *substream)
{
	return snd_pcm_hw_constraint_list(substream->runtime, 0,
			SNDRV_PCM_HW_PARAM_RATE,
			&constraints_8000_16000);
}

static struct snd_soc_ops mrfld_8958_8k_16k_ops = {
	.startup = mrfld_8958_8k_16k_startup,
	.hw_params = mrfld_8958_hw_params,
};

static struct snd_soc_compr_ops mrfld_compr_ops = {
	.set_params = mrfld_wm8958_compr_set_params,
};

static struct snd_soc_ops mrfld_8958_ext_amp_ops = {
	.hw_params = mrfld_8958_hw_params_ext_amp,
};

static const struct snd_soc_pcm_stream mrfld_wm8958_ext_amp_params = {
	/* .stream_name = "UNSPECIFIED" */
	.formats = SNDRV_PCM_FMTBIT_S16_LE,
	.rate_min = 48000,
	.rate_max = 48000,
	.channels_min = 2,
	.channels_max = 2,
	.rates = SNDRV_PCM_RATE_48000,
	.sig_bits = 16,
};

struct snd_soc_dai_link mrfld_8958_msic_dailink[] = {
	[MERR_SALTBAY_AUDIO] = {
		.name = "Merrifield Audio Port",
		.stream_name = "Audio",
		.cpu_dai_name = "Headset-cpu-dai",
		.codec_dai_name = "wm8994-aif1",
		.codec_name = "wm8994-codec",
		.platform_name = "sst-platform",
		.init = mrfld_8958_init,
		.ignore_suspend = 1,
		.ops = &mrfld_8958_ops,
		.playback_count = 3,
	},
	[MERR_SALTBAY_COMPR] = {
		.name = "Merrifield Compress Port",
		.stream_name = "Compress",
		.platform_name = "sst-platform",
		.cpu_dai_name = "Compress-cpu-dai",
		.codec_dai_name = "wm8994-aif1",
		.codec_name = "wm8994-codec",
		.compr_ops = &mrfld_compr_ops,
	},
	[MERR_SALTBAY_VOIP] = {
		.name = "Merrifield VOIP Port",
		.stream_name = "Voip",
		.cpu_dai_name = "Voip-cpu-dai",
		.codec_dai_name = "wm8994-aif1",
		.codec_name = "wm8994-codec",
		.platform_name = "sst-platform",
		.init = NULL,
		.ignore_suspend = 1,
		.ops = &mrfld_8958_8k_16k_ops,
	},
	[MERR_SALTBAY_PROBE] = {
		.name = "Merrifield Probe Port",
		.stream_name = "Probe",
		.cpu_dai_name = "Probe-cpu-dai",
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.platform_name = "sst-platform",
		.playback_count = 8,
		.capture_count = 8,
	},
	[MERR_SALTBAY_AWARE] = {
		.name = "Merrifield Aware Port",
		.stream_name = "Aware",
		.cpu_dai_name = "Loopback-cpu-dai",
		.codec_dai_name = "wm8994-aif1",
		.codec_name = "wm8994-codec",
		.platform_name = "sst-platform",
		.init = NULL,
		.ignore_suspend = 1,
		.ops = &mrfld_8958_8k_16k_ops,
	},
	[MERR_SALTBAY_VAD] = {
		.name = "Merrifield VAD Port",
		.stream_name = "Vad",
		.cpu_dai_name = "Loopback-cpu-dai",
		.codec_dai_name = "wm8994-aif1",
		.codec_name = "wm8994-codec",
		.platform_name = "sst-platform",
		.init = NULL,
		.ignore_suspend = 1,
		.ops = &mrfld_8958_8k_16k_ops,
	},
	[MERR_SALTBAY_POWER] = {
		.name = "Virtual Power Port",
		.stream_name = "Power",
		.cpu_dai_name = "Power-cpu-dai",
		.platform_name = "sst-platform",
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
	},
	/* CODEC<->CODEC links */
	{
		.name = "Audio AIF1-to-AIF2 Loop",
		.stream_name = "NXP External Amplifiers",
		.cpu_dai_name = "snd-soc-dummy-dai",
		.platform_name = "snd-soc-dummy",
		.codec_dai_name = "wm8994-aif2",
		.codec_name = "wm8994-codec",
		.dai_fmt = SND_SOC_DAIFMT_CBM_CFM | SND_SOC_DAIFMT_I2S
						| SND_SOC_DAIFMT_NB_NF,
		.params = &mrfld_wm8958_ext_amp_params,
		.ops = &mrfld_8958_ext_amp_ops,
		.dsp_loopback = false,
	},
};

#ifdef CONFIG_PM_SLEEP
static int snd_mrfld_8958_prepare(struct device *dev)
{
	struct snd_soc_card *card = dev_get_drvdata(dev);
	struct snd_soc_codec *codec;
	struct snd_soc_dapm_context *dapm;

	pr_debug("In %s\n", __func__);

	codec = mrfld_8958_get_codec(card);
	if (!codec) {
		pr_err("%s: couldn't find the codec pointer!\n", __func__);
		return -EAGAIN;
	}

	pr_debug("found codec %s\n", codec->name);
	dapm = &codec->dapm;

	snd_soc_dapm_disable_pin(dapm, "VMID");
	snd_soc_dapm_sync(dapm);

	snd_soc_suspend(dev);
	return 0;
}

static void snd_mrfld_8958_complete(struct device *dev)
{
	struct snd_soc_card *card = dev_get_drvdata(dev);
	struct snd_soc_codec *codec;
	struct snd_soc_dapm_context *dapm;

	pr_debug("In %s\n", __func__);

	codec = mrfld_8958_get_codec(card);
	if (!codec) {
		pr_err("%s: couldn't find the codec pointer!\n", __func__);
		return;
	}

	pr_debug("found codec %s\n", codec->name);
	dapm = &codec->dapm;

	snd_soc_dapm_force_enable_pin(dapm, "VMID");
	snd_soc_dapm_sync(dapm);

	snd_soc_resume(dev);
	return;
}

static int snd_mrfld_8958_poweroff(struct device *dev)
{
	pr_debug("In %s\n", __func__);
	snd_soc_poweroff(dev);
	return 0;
}
#else
#define snd_mrfld_8958_prepare NULL
#define snd_mrfld_8958_complete NULL
#define snd_mrfld_8958_poweroff NULL
#endif

/* SoC card */
static struct snd_soc_card snd_soc_card_mrfld = {
	.name = "wm8958-audio",
	.dai_link = mrfld_8958_msic_dailink,
	.num_links = ARRAY_SIZE(mrfld_8958_msic_dailink),
	.set_bias_level = mrfld_8958_set_bias_level,
	.set_bias_level_post = mrfld_8958_set_bias_level_post,
	.dapm_widgets = widgets,
	.num_dapm_widgets = ARRAY_SIZE(widgets),
	.dapm_routes = map,
	.num_dapm_routes = ARRAY_SIZE(map),
};

static int snd_mrfld_8958_config_gpio(struct platform_device *pdev,
					struct mrfld_8958_mc_private *drv)
{
	int ret = 0;

	if (drv->spk_gpio >= 0) {
		/* Set GPIO as output and init it with high value. So
		 * spk boost is disable by default */
		ret = devm_gpio_request_one(&pdev->dev, (unsigned)drv->spk_gpio,
				GPIOF_INIT_LOW, "spk_boost");
		if (ret) {
			pr_err("GPIO request failed\n");
			return ret;
		}
	}
	return ret;
}

static int snd_mrfld_8958_mc_probe(struct platform_device *pdev)
{
	int ret_val = 0;
	struct mrfld_8958_mc_private *drv;
	struct mrfld_audio_platform_data *mrfld_audio_pdata = pdev->dev.platform_data;

	pr_debug("Entry %s\n", __func__);

	if (!mrfld_audio_pdata) {
		pr_err("Platform data not provided\n");
		return -EINVAL;
	}
	drv = kzalloc(sizeof(*drv), GFP_ATOMIC);
	if (!drv) {
		pr_err("allocation failed\n");
		return -ENOMEM;
	}
	drv->spk_gpio = mrfld_audio_pdata->spk_gpio;
	drv->ext_amp_reset_gpio = get_gpio_by_name("audiocodec_rst");
	if (drv->ext_amp_reset_gpio < 0) {
		pr_info("audiocodec_rst not found in the SFI table.");
	} else {
		if (gpio_request(drv->ext_amp_reset_gpio, "snd_mrfld_8958")) {
			pr_err("req audiocodec_rst failed.");
		} else {
			if (gpio_direction_output(drv->ext_amp_reset_gpio, 1))
				pr_err("Failed to set ext_amp_reset_gpio high");
			usleep_range(10,10);
			if (gpio_direction_output(drv->ext_amp_reset_gpio, 0))
				pr_err("Failed to set ext_amp_reset_gpio low");
			gpio_free(drv->ext_amp_reset_gpio);
			pr_debug("Reset TFA9890s succesfull");
		}
	}

	ret_val = snd_mrfld_8958_config_gpio(pdev, drv);
	if (ret_val) {
		pr_err("GPIO configuration failed\n");
		goto unalloc;
	}
	/* ioremap the register */
	drv->osc_clk0_reg = devm_ioremap_nocache(&pdev->dev,
					MERR_OSC_CLKOUT_CTRL0_REG_ADDR,
					MERR_OSC_CLKOUT_CTRL0_REG_SIZE);
	if (!drv->osc_clk0_reg) {
		pr_err("osc clk0 ctrl ioremap failed\n");
		ret_val = -1;
		goto unalloc;
	}

	if (SPID_PRODUCT_ID(INTEL, MOFD, TABLET, BB, PRO) ||
	   SPID_PRODUCT_ID(INTEL, MOFD, TABLET, BB, ENG)) {
		pr_debug("Selecting BB specific dapm route map\n");
		snd_soc_card_mrfld.dapm_routes = map_bb;
		snd_soc_card_mrfld.num_dapm_routes = ARRAY_SIZE(map_bb);
	}

	/* register the soc card */
	snd_soc_card_mrfld.dev = &pdev->dev;
	snd_soc_card_set_drvdata(&snd_soc_card_mrfld, drv);
	ret_val = snd_soc_register_card(&snd_soc_card_mrfld);
	if (ret_val) {
		pr_err("snd_soc_register_card failed %d\n", ret_val);
		goto unalloc;
	}
	platform_set_drvdata(pdev, &snd_soc_card_mrfld);
	pr_info("%s successful\n", __func__);
	return ret_val;

unalloc:
	kfree(drv);
	return ret_val;
}

static int snd_mrfld_8958_mc_remove(struct platform_device *pdev)
{
	struct snd_soc_card *soc_card = platform_get_drvdata(pdev);
	struct mrfld_8958_mc_private *drv = snd_soc_card_get_drvdata(soc_card);

	pr_debug("In %s\n", __func__);
	kfree(drv);
	snd_soc_card_set_drvdata(soc_card, NULL);
	snd_soc_unregister_card(soc_card);
	platform_set_drvdata(pdev, NULL);
	return 0;
}

static void snd_mrfld_8958_mc_shutdown(struct platform_device *pdev)
{
	struct device dev = pdev->dev;
	struct snd_soc_card *soc_card = platform_get_drvdata(pdev);
	struct mrfld_8958_mc_private *drv = snd_soc_card_get_drvdata(soc_card);

	pr_debug("In %s\n", __func__);
	if (drv->ext_amp_reset_gpio >= 0) {
		if (gpio_request(drv->ext_amp_reset_gpio, "snd_mrfld_8958")) {
			pr_err("req audiocodec_rst failed.");
		} else {
			if (gpio_direction_output(drv->ext_amp_reset_gpio, 1))
				pr_err("Failed to set ext_amp_reset_gpio high");
			gpio_free(drv->ext_amp_reset_gpio);
			pr_debug("Reset TFA9890s succesfull");
		}
	}

	snd_mrfld_8958_poweroff(&dev);
}

const struct dev_pm_ops snd_mrfld_8958_mc_pm_ops = {
	.prepare = snd_mrfld_8958_prepare,
	.complete = snd_mrfld_8958_complete,
	.poweroff = snd_mrfld_8958_poweroff,
};

static struct platform_driver snd_mrfld_8958_mc_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "mrfld_wm8958",
		.pm = &snd_mrfld_8958_mc_pm_ops,
	},
	.probe = snd_mrfld_8958_mc_probe,
	.remove = snd_mrfld_8958_mc_remove,
	.shutdown = snd_mrfld_8958_mc_shutdown,
};

static int snd_mrfld_8958_driver_init(void)
{
	pr_info("Merrifield Machine Driver mrfld_wm8958 registerd\n");
	return platform_driver_register(&snd_mrfld_8958_mc_driver);
}

static void snd_mrfld_8958_driver_exit(void)
{
	pr_debug("In %s\n", __func__);
	platform_driver_unregister(&snd_mrfld_8958_mc_driver);
}

static int snd_mrfld_8958_rpmsg_probe(struct rpmsg_channel *rpdev)
{
	int ret = 0;

	if (rpdev == NULL) {
		pr_err("rpmsg channel not created\n");
		ret = -ENODEV;
		goto out;
	}

	dev_info(&rpdev->dev, "Probed snd_mrfld wm8958 rpmsg device\n");

	ret = snd_mrfld_8958_driver_init();

out:
	return ret;
}

static void snd_mrfld_8958_rpmsg_remove(struct rpmsg_channel *rpdev)
{
	snd_mrfld_8958_driver_exit();
	dev_info(&rpdev->dev, "Removed snd_mrfld wm8958 rpmsg device\n");
}

static void snd_mrfld_8958_rpmsg_cb(struct rpmsg_channel *rpdev, void *data,
				int len, void *priv, u32 src)
{
	dev_warn(&rpdev->dev, "unexpected, message\n");

	print_hex_dump(KERN_DEBUG, __func__, DUMP_PREFIX_NONE, 16, 1,
			data, len,  true);
}

static struct rpmsg_device_id snd_mrfld_8958_rpmsg_id_table[] = {
	{ .name = "rpmsg_mrfld_wm8958_audio" },
	{ },
};
MODULE_DEVICE_TABLE(rpmsg, snd_mrfld_8958_rpmsg_id_table);

static struct rpmsg_driver snd_mrfld_8958_rpmsg = {
	.drv.name	= KBUILD_MODNAME,
	.drv.owner	= THIS_MODULE,
	.id_table	= snd_mrfld_8958_rpmsg_id_table,
	.probe		= snd_mrfld_8958_rpmsg_probe,
	.callback	= snd_mrfld_8958_rpmsg_cb,
	.remove		= snd_mrfld_8958_rpmsg_remove,
};

static int __init snd_mrfld_8958_rpmsg_init(void)
{
	return register_rpmsg_driver(&snd_mrfld_8958_rpmsg);
}
late_initcall(snd_mrfld_8958_rpmsg_init);

static void __exit snd_mrfld_8958_rpmsg_exit(void)
{
	return unregister_rpmsg_driver(&snd_mrfld_8958_rpmsg);
}
module_exit(snd_mrfld_8958_rpmsg_exit);

MODULE_DESCRIPTION("ASoC Intel(R) Merrifield MID Machine driver");
MODULE_AUTHOR("Vinod Koul <vinod.koul@linux.intel.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:mrfld_wm8958");
