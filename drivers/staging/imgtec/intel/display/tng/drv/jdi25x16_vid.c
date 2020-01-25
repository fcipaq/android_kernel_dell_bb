/*
 * Copyright (C) 2010 Intel Corporation
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the next
 * paragraph) shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 * Authors:
 * Faxing Lu <faxing.lu@intel.com>
 */

#include <asm/intel_scu_pmic.h>

#include "displays/jdi25x16_vid.h"

static int mipi_reset_gpio;
static int bias_en_gpio;

static u8 jdi25x16_pixel_format[] = {0x3a, 0x77};
static u8 jdi25x16_clumn_addr[] = {
            0x2a, 0x00, 0x00, 0x04, 0xff};
static u8 jdi25x16_page_addr[] = {
            0x2b, 0x00, 0x00, 0x06, 0x3f};
static u8 jdi25x16_set_tear_on[] = {0x35, 0x00};
static u8 jdi25x16_tear_scanline[] = {
            0x44, 0x00, 0x00};
static u8 jdi25x16_set_brightness[] = {0x51, 0xFF};
static u8 jdi25x16_turn_on_backlight[] = {0x53, 0x24};
static u8 jdi25x16_set_mode[] = {0xb3, 0x14};

int mdfld_dsi_jdi25x16_ic_init(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_pkg_sender *sender
		= mdfld_dsi_get_pkg_sender(dsi_config);
	int err = 0;
	int i = 0;

	PSB_DEBUG_ENTRY("\n");
	if (!sender) {
		DRM_ERROR("Cannot get sender\n");
		return -EINVAL;
	}
	for(i = 0; i < 2; i++) {
		if (i == 0)
			sender->work_for_slave_panel = false;
		else
			sender->work_for_slave_panel = true;
		err = mdfld_dsi_send_mcs_short_hs(sender, soft_reset, 0, 0,
		        MDFLD_DSI_SEND_PACKAGE);
		if (err) {
		    DRM_ERROR("%s: %d: Panel software reset\n", __func__, __LINE__);
		    goto ic_init_err;
		}
		mdelay(20);
		err = mdfld_dsi_send_mcs_short_hs(sender, jdi25x16_pixel_format[0],
				jdi25x16_pixel_format[1], 1, 0);
		if (err) {
		    DRM_ERROR("%s: %d: Set pixel format\n", __func__, __LINE__);
		    goto ic_init_err;
		}

		err = mdfld_dsi_send_mcs_long_hs(sender,
		        jdi25x16_clumn_addr,
				5, MDFLD_DSI_SEND_PACKAGE);
		if (err) {
			DRM_ERROR("%s: %d: Set Clumn Address\n",
			__func__, __LINE__);
			goto ic_init_err;
		}

		err = mdfld_dsi_send_mcs_long_hs(sender,
		        jdi25x16_page_addr,
				5, MDFLD_DSI_SEND_PACKAGE);
		if (err) {
			DRM_ERROR("%s: %d: Set Page Address\n",
			__func__, __LINE__);
			goto ic_init_err;
		}
		err = mdfld_dsi_send_mcs_short_hs(sender, jdi25x16_set_tear_on[0],
				jdi25x16_set_tear_on[1], 1, 0);
		if (err) {
		    DRM_ERROR("%s: %d: Set tear on\n", __func__, __LINE__);
		    goto ic_init_err;
		}
		err = mdfld_dsi_send_mcs_long_hs(sender,
		        jdi25x16_tear_scanline,
				3, MDFLD_DSI_SEND_PACKAGE);
		if (err) {
			DRM_ERROR("%s: %d: Set tear scanline\n",
			__func__, __LINE__);
			goto ic_init_err;
		}
		err = mdfld_dsi_send_mcs_short_hs(sender, jdi25x16_set_brightness[0],
				jdi25x16_set_brightness[1], 1, 0);
		if (err) {
		    DRM_ERROR("%s: %d: Set brightness\n", __func__, __LINE__);
		    goto ic_init_err;
		}
	}
	sender->work_for_slave_panel = false;
	return 0;

ic_init_err:
	sender->work_for_slave_panel = false;
	err = -EIO;
	return err;
}

int mdfld_dsi_jdi25x16_set_mode(struct mdfld_dsi_config *dsi_config)
{

	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	int err;
	int i = 0;

	PSB_DEBUG_ENTRY("\n");

	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}
	for (i = 0; i < 2; i++) {
		if (i == 0)
			sender->work_for_slave_panel = false;
		else
			sender->work_for_slave_panel = true;
		mdelay(20);
		err = mdfld_dsi_send_gen_short_hs(sender,
			access_protect, 0, 2,
			MDFLD_DSI_SEND_PACKAGE);
		if (err) {
			DRM_ERROR("%s: %d: Set MCAP\n",
			__func__, __LINE__);
			goto set_mode_err;
		}

		err = mdfld_dsi_send_gen_long_hs(sender, jdi25x16_set_mode,
				2,
				MDFLD_DSI_SEND_PACKAGE);
		if (err) {
			DRM_ERROR("%s: %d: Set Mode\n", __func__, __LINE__);
			goto set_mode_err;
		}
		err = mdfld_dsi_send_gen_short_hs(sender,
			access_protect, 3, 2,
			MDFLD_DSI_SEND_PACKAGE);
		if (err) {
			DRM_ERROR("%s: %d: Set MCAP\n",
			__func__, __LINE__);
			goto set_mode_err;
		}
		/* Set Display on 0x29 */
		err = mdfld_dsi_send_mcs_short_hs(sender, set_display_on, 0, 0,
				MDFLD_DSI_SEND_PACKAGE);
		if (err) {
			DRM_ERROR("%s: %d: Set Display On\n", __func__, __LINE__);
			goto set_mode_err;
		}
	}

        msleep(20);
	err = mdfld_dsi_send_mcs_short_hs(sender, jdi25x16_turn_on_backlight[0],
			jdi25x16_turn_on_backlight[1], 1, 0);
	if (err) {
		DRM_ERROR("%s: %d: Turn on backlight\n", __func__, __LINE__);
		goto set_mode_err;
	}

	sender->work_for_slave_panel = false;
	return 0;

set_mode_err:
	sender->work_for_slave_panel = false;
	err = -EIO;
	return err;
}
static
void mdfld_dsi_jdi25x16_dsi_controller_init(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_hw_context *hw_ctx =
		&dsi_config->dsi_hw_context;
	/* Virtual channel number */
	int mipi_vc = 0;
	int mipi_pixel_format = 0x4;
	/* BURST_MODE */

	PSB_DEBUG_ENTRY("\n");

	/*reconfig lane configuration*/
	dsi_config->lane_count = 4;
	dsi_config->lane_config = MDFLD_DSI_DATA_LANE_4_0;
	hw_ctx->pll_bypass_mode = 0;
	/* This is for 400 mhz.  Set it to 0 for 800mhz */
	hw_ctx->cck_div = 1;

	hw_ctx->mipi_control = 0;
	hw_ctx->intr_en = 0xFFFFFFFF;
	hw_ctx->hs_tx_timeout = 0xFFFFFF;
	hw_ctx->lp_rx_timeout = 0xFFFFFF;
	hw_ctx->device_reset_timer = 0xffff;
	hw_ctx->turn_around_timeout = 0x3f;
	hw_ctx->high_low_switch_count = 0x40;
	hw_ctx->clk_lane_switch_time_cnt =  0x16002d;
	hw_ctx->lp_byteclk = 0x5;
	hw_ctx->dphy_param = 0x3c1fc51f;
	hw_ctx->eot_disable = 0x2;
	hw_ctx->init_count = 0xfa0;
	hw_ctx->dbi_bw_ctrl = 0x820;

	/*setup video mode format*/
	hw_ctx->video_mode_format = 0xf;

	/*set up func_prg*/
	hw_ctx->dsi_func_prg = ((mipi_pixel_format << 7) | (mipi_vc << 3) |
			dsi_config->lane_count);

	/*setup mipi port configuration*/
	hw_ctx->mipi = MIPI_PORT_EN | PASS_FROM_SPHY_TO_AFE |
		dsi_config->lane_config |
		DUAL_LINK_ENABLE | DUAL_LINK_CAPABLE;

}

static int mdfld_dsi_jdi25x16_detect(struct mdfld_dsi_config *dsi_config)
{
	int status;
	int pipe = dsi_config->pipe;

	PSB_DEBUG_ENTRY("\n");

	if (pipe == 0) {
		status = MDFLD_DSI_PANEL_CONNECTED;
	} else {
		DRM_INFO("%s: do NOT support dual panel\n", __func__);
		status = MDFLD_DSI_PANEL_DISCONNECTED;
	}

	return status;
}

static int mdfld_dsi_jdi25x16_power_on(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	int err;
	int i;

	PSB_DEBUG_ENTRY("\n");

	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}
	for (i = 0; i < 2; i++) {
		if (i == 0)
			sender->work_for_slave_panel = false;
		else
			sender->work_for_slave_panel = true;
		/* Sleep Out */
		err = mdfld_dsi_send_mcs_short_hs(sender, exit_sleep_mode, 0, 0,
				MDFLD_DSI_SEND_PACKAGE);
		if (err) {
			DRM_ERROR("%s: %d: Exit Sleep Mode\n", __func__, __LINE__);
			goto power_on_err;
		}
		msleep(100);
		/* Send TURN_ON packet */
		err = mdfld_dsi_send_dpi_spk_pkg_hs(sender, MDFLD_DSI_DPI_SPK_TURN_ON);
		if (err) {
			DRM_ERROR("Failed to send turn on packet\n");
			goto power_on_err;
		}
	}
	sender->work_for_slave_panel = false;
	return 0;

power_on_err:
	sender->work_for_slave_panel = false;
	err = -EIO;
	return err;
}

static void __vpro2_power_ctrl(bool on)
{
	u8 addr, value;
	addr = 0xad;
	if (intel_scu_ipc_ioread8(addr, &value))
		DRM_ERROR("%s: %d: failed to read vPro2\n", __func__, __LINE__);

	/* Control vPROG2 power rail with 2.85v. */
	if (on)
		value |= 0x1;
	else
		value &= ~0x1;

	if (intel_scu_ipc_iowrite8(addr, value))
		DRM_ERROR("%s: %d: failed to write vPro2\n",
				__func__, __LINE__);
}

static int mdfld_dsi_jdi25x16_power_off(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	int err;
	int i;

	PSB_DEBUG_ENTRY("\n");

	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}
	for (i = 0; i < 2; i++) {
		if (i == 0)
			sender->work_for_slave_panel = false;
		else
			sender->work_for_slave_panel = true;
		/*send SHUT_DOWN packet */
		err = mdfld_dsi_send_dpi_spk_pkg_hs(sender,
				MDFLD_DSI_DPI_SPK_SHUT_DOWN);
		if (err) {
			DRM_ERROR("Failed to send turn off packet\n");
			goto power_off_err;
		}
		/* Set Display off */
		err = mdfld_dsi_send_mcs_short_hs(sender, set_display_off, 0, 0,
				MDFLD_DSI_SEND_PACKAGE);
		if (err) {
			DRM_ERROR("%s: %d: Set Display On\n", __func__, __LINE__);
			goto power_off_err;
		}
		msleep(20);
		/* Sleep In */
		err = mdfld_dsi_send_mcs_short_hs(sender, enter_sleep_mode, 0, 0,
				MDFLD_DSI_SEND_PACKAGE);
		if (err) {
			DRM_ERROR("%s: %d: Exit Sleep Mode\n", __func__, __LINE__);
			goto power_off_err;
		}
	}
	sender->work_for_slave_panel = false;
	msleep(80);
	return 0;

power_off_err:
	sender->work_for_slave_panel = false;
	err = -EIO;
	return err;
}

static int mdfld_dsi_jdi25x16_panel_reset(struct mdfld_dsi_config *dsi_config)
{
	int ret;
	__vpro2_power_ctrl(true);
	usleep_range(2000, 2500);

	if (bias_en_gpio == 0) {
		bias_en_gpio = 189;
		ret = gpio_request(bias_en_gpio, "bias_enable");
		if (ret) {
			DRM_ERROR("Faild to request bias_enable gpio\n");
			return -EINVAL;
		}
		gpio_direction_output(bias_en_gpio, 0);
	}
	if (mipi_reset_gpio == 0) {
		ret = get_gpio_by_name("disp0_rst");
		if (ret < 0) {
			DRM_ERROR("Faild to get panel reset gpio, " \
				  "use default reset pin\n");
			return 0;
		}
		mipi_reset_gpio = ret;
		ret = gpio_request(mipi_reset_gpio, "mipi_display");
		if (ret) {
			DRM_ERROR("Faild to request panel reset gpio\n");
			return 0;
		}
		gpio_direction_output(mipi_reset_gpio, 0);
	}
	gpio_direction_output(bias_en_gpio, 0);
	gpio_direction_output(mipi_reset_gpio, 0);
	gpio_set_value_cansleep(bias_en_gpio, 0);
	gpio_set_value_cansleep(mipi_reset_gpio, 0);
	usleep_range(2000, 2500);
	gpio_set_value_cansleep(bias_en_gpio, 1);
	usleep_range(2000, 2500);
	gpio_set_value_cansleep(mipi_reset_gpio, 1);
	usleep_range(2000, 2500);
	return 0;
}

static struct drm_display_mode *jdi25x16_vid_get_config_mode(void)
{
	struct drm_display_mode *mode;

	PSB_DEBUG_ENTRY("\n");

	mode = kzalloc(sizeof(*mode), GFP_KERNEL);
	if (!mode)
		return NULL;

	mode->hdisplay = 2560;

	mode->hsync_start = mode->hdisplay + 8;
	mode->hsync_end = mode->hsync_start + 20;
	mode->htotal = mode->hsync_end + 32;

	mode->vdisplay = 1600;
	mode->vsync_start = mode->vdisplay + 12;
	mode->vsync_end = mode->vsync_start + 4;
	mode->vtotal = mode->vsync_end + 4;

	mode->vrefresh = 60;
	mode->clock =  mode->vrefresh * mode->vtotal *
		mode->htotal / 1000;

	drm_mode_set_name(mode);
	drm_mode_set_crtcinfo(mode, 0);

	mode->type |= DRM_MODE_TYPE_PREFERRED;

	return mode;
}

static int mdfld_dsi_jdi25x16_set_brightness(struct mdfld_dsi_config *dsi_config,
		int level)
{
	return 0;
}

static void jdi25x16_vid_get_panel_info(int pipe, struct panel_info *pi)
{
	if (!pi)
		return;

	if (pipe == 0) {
		pi->width_mm = 192;
		pi->height_mm = 120;
	}

	return;
}

void jdi25x16_vid_init(struct drm_device *dev, struct panel_funcs *p_funcs)
{
	PSB_DEBUG_ENTRY("\n");

	p_funcs->get_config_mode = jdi25x16_vid_get_config_mode;
	p_funcs->get_panel_info = jdi25x16_vid_get_panel_info;
	p_funcs->reset = mdfld_dsi_jdi25x16_panel_reset;
	p_funcs->drv_ic_init = mdfld_dsi_jdi25x16_ic_init;
	p_funcs->dsi_controller_init = mdfld_dsi_jdi25x16_dsi_controller_init;
	p_funcs->detect = mdfld_dsi_jdi25x16_detect;
	p_funcs->power_on = mdfld_dsi_jdi25x16_power_on;
	p_funcs->power_off = mdfld_dsi_jdi25x16_power_off;
	p_funcs->set_brightness = mdfld_dsi_jdi25x16_set_brightness;
	p_funcs->drv_set_panel_mode = mdfld_dsi_jdi25x16_set_mode;
}
