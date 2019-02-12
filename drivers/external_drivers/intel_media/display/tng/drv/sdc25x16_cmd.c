/*
 * Copyright © 2014 Intel Corporation
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

#include "mdfld_dsi_dbi.h"
#include "mdfld_output.h"
#include "mdfld_dsi_pkg_sender.h"
#include <asm/intel_scu_pmic.h>
#include "displays/sdc25x16_cmd.h"
#include <linux/random.h>

#define WIDTH 2560
#define HEIGHT 1600

//#define PIXEL_SHIFT_MAX_X       10
//#define PIXEL_SHIFT_MAX_Y       8
#define PIXEL_SHIFT_MAX_X       64
#define PIXEL_SHIFT_MAX_Y       64

#define PIXEL_SHIFT_INITIAL_X       0
#define PIXEL_SHIFT_INITIAL_Y       1

/* Set the panel update delay to 8 ms */
#define DEFAULT_PANEL_DELAY 8000

static int vdd_1_8v_gpio;
static int tcon_rdy_gpio;
static int bias_en_gpio;

static u8 sdc_column_addr[] = {
			0x2a, 0x00, 0x00, 0x04, 0xff};
static u8 sdc_page_addr[] = {
			0x2b, 0x00, 0x00, 0x06, 0x3f};
static	u8 sdc_set_300nit[34] = { 0x83,
								0x80, 0x80,
								0x80, 0x80,
								0x80, 0x80,
								0x80, 0x00,
								0x80, 0x80,
								0x00,
								0x80, 0x80,
								0x80, 0x80,
								0x80, 0x80,
								0x80, 0x00,
								0x80, 0x80,
								0x00,
								0x80, 0x80,
								0x80, 0x80,
								0x80, 0x80,
								0x80, 0x00,
								0x80, 0x80,
								0x00};
static	u8 sdc_set_AID[] = { 0x85, 0x06, 0x00 };

static	u8 sdc_gamma_setting[] = { 0x82, 0x1f};
static	u8 sdc_AOR_setting[] = { 0x85, 0x6, 0};
static	u8 sdc_global_para_53[] = { 0xb0, 0x34};
static	u8 sdc_ELVSS_para[] = { 0xbb, 0xF};
static	u8 sdc_global_para_47[] = { 0xb0, 0x2e};
static	u8 sdc_gamma_update[] = { 0xbb, 0x1};

static	u8 sdc_300_ELVSS[] = { 0xbb, 0x19};


static	u8 sdc_global_para_70[] = { 0xb0, 0x45};
static	u8 sdc_set_ACL_off[] = { 0xbb, 0x10};
static	u8 sdc_set_ACL_on[] = { 0xbb, 0x12};


static u8 sdc_brightness_list[30][5] = {
			{0x1f, 0x06, 0x00, 0x00, 0x0f},
			{0x00, 0x06, 0x00, 0x19, 0x1b},
			{0x01, 0x06, 0x00, 0x1a, 0x1c},
			{0x02, 0x06, 0x00, 0x1c, 0x1e},
			{0x03, 0x06, 0x00, 0x1d, 0x1f},
			{0x04, 0x58, 0x00, 0x1d, 0x1f},
			{0x05, 0x87, 0x00, 0x1d, 0x1f},
			{0x06, 0xad, 0x00, 0x1d, 0x1f},
			{0x07, 0xad, 0x00, 0x1f, 0x1f},
			{0x08, 0xad, 0x00, 0x1f, 0x1f},
			{0x09, 0xad, 0x00, 0x1f, 0x1f},
			{0x0a, 0xad, 0x00, 0x1f, 0x1f},
			{0x0b, 0xad, 0x00, 0x1f, 0x1f},
			{0x0c, 0xad, 0x00, 0x1f, 0x1f},
			{0x0d, 0xb3, 0x00, 0x1f, 0x1f},
			{0x0e, 0xd2, 0x00, 0x1f, 0x1f},
			{0x0f, 0xe8, 0x00, 0x1f, 0x1f},
			{0x10, 0xfe, 0x00, 0x1f, 0x1f},
			{0x11, 0x11, 0x01, 0x1f, 0x1f},
			{0x12, 0x23, 0x01, 0x1f, 0x1f},
			{0x13, 0x32, 0x01, 0x1f, 0x1f},
/* fcipaq: low brightness for night time reading */
			{0x14, 0x3c, 0x01, 0x1f, 0x1f},
			{0x15, 0x50, 0x01, 0x1f, 0x1f},
			{0x16, 0x64, 0x01, 0x1f, 0x1f},
			{0x17, 0x78, 0x01, 0x1f, 0x1f},
			{0x18, 0x82, 0x01, 0x1f, 0x1f},
			{0x19, 0x87, 0x01, 0x1f, 0x1f},
			{0x1a, 0x8c, 0x01, 0x1f, 0x1f},
			{0x1b, 0x91, 0x01, 0x1f, 0x1f},
			{0x1c, 0x96, 0x01, 0x1f, 0x1f}
/* fcipaq end */
			};

static
int sdc25x16_set_dimming(struct mdfld_dsi_pkg_sender *sender,
	int index)
{
	int err = 0;

	PSB_DEBUG_ENTRY("\n");
	sdc_gamma_setting[1] = sdc_brightness_list[index][0];
	sdc_AOR_setting[1] = sdc_brightness_list[index][1];
	sdc_AOR_setting[2] = sdc_brightness_list[index][2];
	sdc_ELVSS_para[1] = sdc_brightness_list[index][3];
	err = mdfld_dsi_send_gen_long_hs(sender,
			sdc_gamma_setting,
			2, MDFLD_DSI_SEND_PACKAGE);
	
	if (err) {
		DRM_ERROR("%s: %d: sdc_gamma_setting\n",
		__func__, __LINE__);
		goto set_dimming_err;
	}
	err = mdfld_dsi_send_gen_long_hs(sender,
			sdc_AOR_setting,
			3, MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("%s: %d: sdc_gamma_setting\n",
		__func__, __LINE__);
		goto set_dimming_err;
	}
	err = mdfld_dsi_send_gen_long_hs(sender,
			sdc_global_para_53,
			2, MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("%s: %d: sdc_global_para_53\n",
		__func__, __LINE__);
		goto set_dimming_err;
	}
	err = mdfld_dsi_send_gen_long_hs(sender,
			sdc_ELVSS_para,
			2, MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("%s: %d: sdc_ELVSS_para\n",
		__func__, __LINE__);
		goto set_dimming_err;
	}
	err = mdfld_dsi_send_gen_long_hs(sender,
			sdc_global_para_47,
			2, MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("%s: %d: sdc_global_para_47\n",
		__func__, __LINE__);
		goto set_dimming_err;
	}
	err = mdfld_dsi_send_gen_long_hs(sender,
			sdc_gamma_update,
			2, MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("%s: %d: sdc_gamma_update\n",
		__func__, __LINE__);
		goto set_dimming_err;
	}

	return 0;
set_dimming_err:
	err = -EIO;
	return err;
}

static
int sdc25x16_cmd_drv_ic_init(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_pkg_sender *sender
		= mdfld_dsi_get_pkg_sender(dsi_config);
	int err = 0;
	int i;
/***/
	u8 set_teline[] = {0x44,
			(drm_psb_te_timer_delay >> 8) & 0xff,
			0xff & drm_psb_te_timer_delay };
/***/
	PSB_DEBUG_ENTRY("\n");
	if (!sender) {
		DRM_ERROR("Cannot get sender\n");
		return -EINVAL;
	}
	i = 10;
	err = sdc25x16_set_dimming(sender, i);
	if (err) {
		DRM_ERROR("%s: %d: brightness setting error\n",
		__func__, __LINE__);
		goto ic_init_err;
	}
/***/
	err = mdfld_dsi_send_gen_long_hs(sender,
					set_teline,
					sizeof(set_teline),
					MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("%s: %d: set_teline\n",
		__func__, __LINE__);
		goto ic_init_err;
	}
/***/
	err = mdfld_dsi_send_gen_long_hs(sender,
			sdc_global_para_70,
			2, MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("%s: %d: sdc_global_para_70\n",
		__func__, __LINE__);
		goto ic_init_err;
	}
	err = mdfld_dsi_send_gen_long_hs(sender,
			sdc_set_ACL_on,
			2, MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("%s: %d: sdc_set_ACL_on\n",
		__func__, __LINE__);
		goto ic_init_err;
	}
	err = mdfld_dsi_send_gen_long_hs(sender,
			sdc_global_para_47,
			2, MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("%s: %d: sdc_global_para_47\n",
		__func__, __LINE__);
		goto ic_init_err;
	}
	err = mdfld_dsi_send_gen_long_hs(sender,
			sdc_gamma_update,
			2, MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("%s: %d: sdc_gamma_update\n",
		__func__, __LINE__);
		goto ic_init_err;
	}
	err = mdfld_dsi_send_mcs_long_hs(sender,
			sdc_column_addr,
			5, MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("%s: %d: Set Column Address\n",
		__func__, __LINE__);
		goto ic_init_err;
	}
	err = mdfld_dsi_send_mcs_long_hs(sender,
			sdc_page_addr,
			5, MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("%s: %d: Set Page Address\n",
		__func__, __LINE__);
		goto ic_init_err;
	}
	err = mdfld_dsi_send_mcs_short_hs(sender,
			set_tear_on, 0x0, 1,
			MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("%s: %d: Set Tear On\n",
		__func__, __LINE__);
		goto ic_init_err;
	}
	return 0;
ic_init_err:
	err = -EIO;
	return err;
}

static
void sdc25x16_cmd_controller_init(
		struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_hw_context *hw_ctx =
				&dsi_config->dsi_hw_context;

	PSB_DEBUG_ENTRY("\n");

	/*reconfig lane configuration*/
	dsi_config->lane_count = 4;
	dsi_config->lane_config = MDFLD_DSI_DATA_LANE_4_0;
	hw_ctx->cck_div = 1;
	hw_ctx->pll_bypass_mode = 0;

	hw_ctx->mipi_control = 0x0;
	hw_ctx->intr_en = 0xFFFFFFFF;
	hw_ctx->hs_tx_timeout = 0xFFFFFF;
	hw_ctx->lp_rx_timeout = 0xFFFFFF;
	hw_ctx->device_reset_timer = 0xffff;
	hw_ctx->turn_around_timeout = 0x3f;
	hw_ctx->high_low_switch_count = 0x2b;
	hw_ctx->clk_lane_switch_time_cnt =  0x2b0014;
	hw_ctx->lp_byteclk = 0x6;
	hw_ctx->dphy_param = 0x2a18681f;
	hw_ctx->eot_disable = 0x1 | BIT8;
	hw_ctx->init_count = 0xf0;
	hw_ctx->dbi_bw_ctrl = 1024;
	hw_ctx->hs_ls_dbi_enable = 0x0;
	hw_ctx->dsi_func_prg = ((DBI_DATA_WIDTH_OPT2 << 13) |
				dsi_config->lane_count);

	hw_ctx->mipi = SEL_FLOPPED_HSTX	| PASS_FROM_SPHY_TO_AFE |
		DUAL_LINK_ENABLE | DUAL_LINK_CAPABLE | TE_TRIGGER_GPIO_PIN | BANDGAP_CHICKEN_BIT;
	hw_ctx->video_mode_format = 0xf;
}
static
int sdc25x16_cmd_panel_connection_detect(
	struct mdfld_dsi_config *dsi_config)
{
	int status;
	int pipe = dsi_config->pipe;

	PSB_DEBUG_ENTRY("\n");

	if (pipe == 0) {
		status = MDFLD_DSI_PANEL_CONNECTED;
	} else {
		DRM_INFO("%s: do NOT support dual panel\n",
		__func__);
		status = MDFLD_DSI_PANEL_DISCONNECTED;
	}

	return status;
}

static
void vdd3_power_ctrl(bool on)
{
        u8 addr, value;
        addr = 0xaa;

        if (intel_scu_ipc_ioread8(addr, &value))
                DRM_ERROR("%s: %d: failed to read VDD3\n", __func__, __LINE__);

        /* Control vDD3 power rail with 3.3v */
        value &= ~0x60;
        value |= 0x60;
        if (on)
                value |= 0x1;
        else
                value &= 0x01;

        if (intel_scu_ipc_iowrite8(addr, value))
                DRM_ERROR("%s: %d: failed to write VDD3\n", __func__, __LINE__);
}

static
int sdc25x16_cmd_power_on(
	struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
        struct drm_device *dev = dsi_config->dev;
        struct drm_psb_private *dev_priv =
                (struct drm_psb_private *) dev->dev_private;
	/* I think the following variable should be global... :) */
        static bool init_power_on = true;
	int err = 0;

	PSB_DEBUG_ENTRY("\n");

	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}
	/* Set Display on 0x29 */
	err = mdfld_dsi_send_mcs_short_hs(sender, set_display_on, 0, 0,
			MDFLD_DSI_SEND_PACKAGE);

	if (init_power_on) {
		dev_priv->amoled_shift.dir_x = 0;
		dev_priv->amoled_shift.dir_y = 0;

		/* Start pixel shift at a random position. */

		get_random_bytes(&dev_priv->amoled_shift.curr_x, sizeof(dev_priv->amoled_shift.curr_x));
		dev_priv->amoled_shift.curr_x = dev_priv->amoled_shift.curr_x % PIXEL_SHIFT_MAX_X;

		get_random_bytes(&dev_priv->amoled_shift.curr_y, sizeof(dev_priv->amoled_shift.curr_y));
		dev_priv->amoled_shift.curr_y = dev_priv->amoled_shift.curr_y % PIXEL_SHIFT_MAX_Y;

		dev_priv->amoled_shift.flip_done = 0;

		init_power_on = false;
	}
/* controlled by psb_intel_display2.c --> amoled worker */
/*
        else {
                if (dev_priv->amoled_shift.curr_x < dev_priv->amoled_shift.max_x)
                        dev_priv->amoled_shift.curr_x++;
                else
                        dev_priv->amoled_shift.curr_x = 0;

                if (dev_priv->amoled_shift.curr_y < dev_priv->amoled_shift.max_y)
                        dev_priv->amoled_shift.curr_y++;

                else
                        dev_priv->amoled_shift.curr_y = 0;
        }
*/	
	if (err) {
		DRM_ERROR("%s: %d: Set Display On\n", __func__, __LINE__);
		goto power_err;
	}
power_err:
	return err;
}

static int sdc25x16_cmd_power_off(
		struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	int err;

	PSB_DEBUG_ENTRY("\n");

	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}

	err = mdfld_dsi_send_mcs_short_hs(sender,
			set_display_off, 0, 0,
			MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("%s: %d: Set Display Off\n",
		__func__, __LINE__);
		goto power_off_err;
	}
	msleep(150);

	err = mdfld_dsi_send_mcs_short_hs(sender,
			enter_sleep_mode, 0, 0,
			MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("%s: %d: Enter Sleep Mode\n",
		__func__, __LINE__);
		goto power_off_err;
	}
	msleep(120);

        if (vdd_1_8v_gpio)
                gpio_set_value_cansleep(vdd_1_8v_gpio, 0);
        msleep(10);

        if (bias_en_gpio)
                gpio_set_value_cansleep(bias_en_gpio, 0);

	return 0;
power_off_err:
	err = -EIO;
	return err;
}

static
int sdc25x16_cmd_enable_pixel_shift(int *max_x,
                         int *max_y)
{
        int val_x = PIXEL_SHIFT_MAX_X;
        int val_y = PIXEL_SHIFT_MAX_Y;

        *max_x = val_x;
        *max_y = val_y;

        return true;
}

static
int sdc25x16_cmd_set_brightness(
		struct mdfld_dsi_config *dsi_config,
		int level)
{
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	int i = 0;

	PSB_DEBUG_ENTRY("level = %d\n", level);

	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}
	/* fcipaq: low brightness for night time reading */
	i = 29 - (29 * level)/ 255;
 
	sdc25x16_set_dimming(sender, i);
	pr_err("================%d.%d\n",i,level);
	return 0;
}

static
int sdc25x16_cmd_panel_reset(
		struct mdfld_dsi_config *dsi_config)
{
        struct mdfld_dsi_pkg_sender *sender =
                mdfld_dsi_get_pkg_sender(dsi_config);

	int ret = 0;

	msleep(30);

	PSB_DEBUG_ENTRY("\n");

        if (vdd_1_8v_gpio == 0) {
                vdd_1_8v_gpio = 45;
                ret = gpio_request(vdd_1_8v_gpio, "vdd_1_8v_gpio");
                if (ret) {
                        DRM_ERROR("Faild to request vdd_1_8v gpio\n");
                        return -EINVAL;
                }
                ret = gpio_direction_output(vdd_1_8v_gpio, 0);
                if (ret) {
                        DRM_ERROR("Failed to set output direction for vdd_1_8v__gpio\n");
                        return -EINVAL;
                }
        }

	if (bias_en_gpio == 0) {
                bias_en_gpio = 189;
                ret = gpio_request(bias_en_gpio, "bias_enable");
                if (ret) {
                        DRM_ERROR("Failed to request bias_enable gpio\n");
                        return -EINVAL;
                }
                ret = gpio_direction_output(bias_en_gpio, 0);
                if (ret) {
                        DRM_ERROR("Failed to set output direction for bias_en_gpio\n");
                        return -EINVAL;
                }
        }

        if (tcon_rdy_gpio == 0) {
                tcon_rdy_gpio = 190;
                ret = gpio_request(tcon_rdy_gpio, "tcon_rdy_gpio");
                if (ret) {
                        DRM_ERROR("Faild to request tcon_rdy_gpio\n");
                        return -EINVAL;
                }
                ret = gpio_direction_input(tcon_rdy_gpio);
                if (ret) {
                        DRM_ERROR("Failed to set input direction for tcon_rdy_gpio\n");
                        return -EINVAL;
                }
        }

        gpio_direction_output(bias_en_gpio, 0);
        gpio_direction_output(vdd_1_8v_gpio, 0);
        gpio_set_value_cansleep(bias_en_gpio, 0);
        gpio_set_value_cansleep(vdd_1_8v_gpio, 0);
        msleep(15);
        vdd3_power_ctrl(true);
        msleep(10);
        gpio_set_value_cansleep(bias_en_gpio, 1);
        msleep(10);
        gpio_set_value_cansleep(vdd_1_8v_gpio, 1);
        msleep(10);

        /* wait for tcon_rdy_gpio signal: L->H */
        if (tcon_rdy_gpio) {
                while (!gpio_get_value(tcon_rdy_gpio)) {
                        ;
                }
        }
	
	return 0;
}

static
int sdc25x16_cmd_exit_deep_standby(
		struct mdfld_dsi_config *dsi_config)
{
	static bool bFirst = true;

	PSB_DEBUG_ENTRY("\n");
	if (bFirst) bFirst = false;
	else {
		msleep(30);
                vdd3_power_ctrl(true);
                msleep(10);
                gpio_set_value_cansleep(bias_en_gpio, 0);
    		usleep_range(2000, 2500);
                gpio_set_value_cansleep(bias_en_gpio, 1);
                msleep(10);
		gpio_set_value_cansleep(vdd_1_8v_gpio, 0);
		usleep_range(2000, 2500);
		gpio_set_value_cansleep(vdd_1_8v_gpio, 1);
		usleep_range(2000, 2500);

                /* wait for tcon_rdy_gpio signal: L->H */
                if (tcon_rdy_gpio) {
                        while (!gpio_get_value(tcon_rdy_gpio)) {
                                ;
                        }
                }
	}
	return 0;
}

static
struct drm_display_mode *sdc25x16_cmd_get_config_mode(void)
{
	struct drm_display_mode *mode;

	PSB_DEBUG_ENTRY("\n");

	mode = kzalloc(sizeof(*mode), GFP_KERNEL);
	if (!mode)
		return NULL;

	mode->hdisplay = WIDTH;

	mode->hsync_start = mode->hdisplay + 48;
	mode->hsync_end = mode->hsync_start + 32;
	mode->htotal = mode->hsync_end + 80;

	mode->vdisplay = HEIGHT;
	mode->vsync_start = mode->vdisplay + 3;
	mode->vsync_end = mode->vsync_start + 33;
	mode->vtotal = mode->vsync_end + 10;


	mode->vrefresh = 60;
	mode->clock =  mode->vrefresh * mode->vtotal * mode->htotal / 1000;
	mode->type |= DRM_MODE_TYPE_PREFERRED;

	mode->hdisplay = WIDTH - PIXEL_SHIFT_MAX_X;
	mode->vdisplay = HEIGHT - PIXEL_SHIFT_MAX_Y;

	drm_mode_set_name(mode);
	drm_mode_set_crtcinfo(mode, 0);

	return mode;
}

static
void sdc25x16_cmd_get_panel_info(int pipe,
		struct panel_info *pi)
{
	PSB_DEBUG_ENTRY("\n");

	if (pipe == 0) {
		pi->width_mm = 224;
		pi->height_mm = 140;
	}
}

void sdc25x16_cmd_init(struct drm_device *dev,
		struct panel_funcs *p_funcs)
{
	if (!dev || !p_funcs) {
		DRM_ERROR("Invalid parameters\n");
		return;
	}
	PSB_DEBUG_ENTRY("\n");
	p_funcs->reset = sdc25x16_cmd_panel_reset;
	p_funcs->power_on = sdc25x16_cmd_power_on;
	p_funcs->power_off = sdc25x16_cmd_power_off;
	p_funcs->drv_ic_init = sdc25x16_cmd_drv_ic_init;
	p_funcs->get_config_mode = sdc25x16_cmd_get_config_mode;
	p_funcs->get_panel_info = sdc25x16_cmd_get_panel_info;
	p_funcs->dsi_controller_init =
			sdc25x16_cmd_controller_init;
	p_funcs->detect =
			sdc25x16_cmd_panel_connection_detect;
	p_funcs->set_brightness =
			sdc25x16_cmd_set_brightness;
	p_funcs->exit_deep_standby =
				sdc25x16_cmd_exit_deep_standby;
	p_funcs->enable_pixel_shift = sdc25x16_cmd_enable_pixel_shift;
	drm_psb_te_timer_delay = DEFAULT_PANEL_DELAY;
}

