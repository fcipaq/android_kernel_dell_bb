/*****************************************************************************
 *
 * Copyright © 2010 Intel Corporation
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
 ******************************************************************************/
#include <linux/console.h>

#include "psb_drv.h"
#include "pmu_tng.h"
#include "psb_fb.h"
#include "psb_intel_reg.h"
#include "displayclass_interface.h"
#include "mdfld_dsi_output.h"
#include "pwr_mgmt.h"
#include "mdfld_dsi_dbi_dsr.h"
#include "dc_maxfifo.h"
#include <linux/kernel.h>
//#include <string.h>
#include "android_hdmi.h"
#include "mdfld_dsi_dbi_dsr.h"

#ifdef CONFIG_AMOLED_SUPPORT
#include <drm/drmP.h>
extern struct drm_pixel_shift wl_amoled_shift;
#endif

#define KEEP_UNUSED_CODE 0

static u32 primary_offset_x;
static u32 primary_offset_y;
static u32 primary_width;
static u32 primary_height;

#if KEEP_UNUSED_CODE
static int FindCurPipe(struct drm_device *dev)
{
	struct drm_crtc *crtc;

	list_for_each_entry(crtc, &dev->mode_config.crtc_list, head) {
		if (drm_helper_crtc_in_use(crtc)) {
			struct psb_intel_crtc *psb_intel_crtc =
			    to_psb_intel_crtc(crtc);
			return psb_intel_crtc->pipe;
		}
	}

	return 0;
}
#endif /* if KEEP_UNUSED_CODE */

static void user_mode_start(struct drm_psb_private *dev_priv)
{
	if (!dev_priv->um_start) {
		dev_priv->um_start = true;
		dev_priv->b_async_flip_enable = true;
		if (dev_priv->b_dsr_enable_config)
			dev_priv->b_dsr_enable = true;
	}
}

static void DCWriteReg(struct drm_device *dev, unsigned long ulOffset,
		       unsigned long ulValue)
{
	struct drm_psb_private *dev_priv;
	void *pvRegAddr;

	dev_priv = (struct drm_psb_private *)dev->dev_private;
	pvRegAddr = (void *)(dev_priv->vdc_reg + ulOffset);
	mb();
	iowrite32(ulValue, pvRegAddr);
}

void DCCBGetFramebuffer(struct drm_device *dev, struct psb_framebuffer **ppsb)
{
	struct drm_psb_private *dev_priv;
	struct psb_fbdev *fbdev;

	dev_priv = (struct drm_psb_private *)dev->dev_private;
	fbdev = dev_priv->fbdev;
	if (fbdev != NULL)
		*ppsb = fbdev->pfb;
}

int DCChangeFrameBuffer(struct drm_device *dev,
			struct psb_framebuffer *psbfb)
{
	return 0;
}

int DCCBEnableVSyncInterrupt(struct drm_device *dev, int pipe)
{
	struct drm_psb_private *dev_priv;
	int ret = 0;

	dev_priv = (struct drm_psb_private *)dev->dev_private;
	if (drm_vblank_get(dev, pipe)) {
		DRM_DEBUG("Couldn't enable vsync interrupt\n");
		ret = -EINVAL;
	}

	return ret;
}

void DCCBDisableVSyncInterrupt(struct drm_device *dev, int pipe)
{
	struct drm_psb_private *dev_priv;

	dev_priv = (struct drm_psb_private *)dev->dev_private;
	drm_vblank_put(dev, pipe);
}

void DCCBInstallVSyncISR(struct drm_device *dev,
			 pfn_vsync_handler pVsyncHandler)
{
	struct drm_psb_private *dev_priv;

	dev_priv = (struct drm_psb_private *)dev->dev_private;
	dev_priv->psb_vsync_handler = pVsyncHandler;
}

void DCCBUninstallVSyncISR(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv =
	    (struct drm_psb_private *)dev->dev_private;

	dev_priv->psb_vsync_handler = NULL;
}

void DCCBFlipToSurface(struct drm_device *dev, unsigned long uiAddr,
				unsigned long uiFormat, unsigned long uiStride,
		       unsigned int pipeflag)
{
	struct drm_psb_private *dev_priv =
	    (struct drm_psb_private *)dev->dev_private;
	u32 dspsurf = (dev_priv->cur_pipe == 0 ? DSPASURF : DSPBSURF);
	u32 dspcntr;
	u32 dspstride;
	u32 reg_offset;
	u32 val = 0;
	struct mdfld_dsi_config *dsi_config = NULL;
	struct mdfld_dsi_hw_context *dsi_ctx;

	DRM_DEBUG("%s %s %d, uiAddr = 0x%lx\n", __FILE__, __func__,
			  __LINE__, uiAddr);

	user_mode_start(dev_priv);

	if (pipeflag == 0) {
		dsi_config = dev_priv->dsi_configs[0];
		reg_offset = 0;
	} else if (pipeflag == 2) {
		dsi_config = dev_priv->dsi_configs[1];
		reg_offset = 0x2000;
	} else if (pipeflag == 1) {
		dsi_config = NULL;
		reg_offset = 0x1000;
	} else {
		DRM_ERROR("%s: invalid pipe %u\n", __func__, pipeflag);
		return;
	}

	/*update format*/
	val = (0x80000000 | uiFormat);

	if (dsi_config) {
		dsi_ctx = &dsi_config->dsi_hw_context;
		dsi_ctx->dspstride = uiStride;
		dsi_ctx->dspcntr = val;
		dsi_ctx->dspsurf = uiAddr;
	}

	dspsurf = DSPASURF + reg_offset;
	dspcntr = DSPACNTR + reg_offset;
	dspstride = DSPASTRIDE + reg_offset;

	DCWriteReg(dev, dspcntr, val);
	/*update stride*/
	DCWriteReg(dev, dspstride, uiStride);
	/*update surface address*/
	DCWriteReg(dev, dspsurf, uiAddr);
}

static bool is_vblank_period(struct drm_device *dev, int pipe)
{
	struct drm_psb_private *dev_priv = NULL;
	struct mdfld_dsi_config *dsi_config = NULL;
	struct android_hdmi_priv *hdmi_priv = NULL;
	u32 reg_offset = 0;
	int vdisplay = 0, vrefresh = 0;
	int delay_us = 5, dsl_threshold = 0, plane_flip_time = 200;
	int retry = 0;
	int dsl = 0;

	if (!dev || !dev->dev_private)
		return false;

	dev_priv = (struct drm_psb_private *)dev->dev_private;

	switch (pipe) {
	case PIPEA:
		reg_offset = 0;
		dsi_config = dev_priv->dsi_configs[0];
		if (dsi_config && dsi_config->mode) {
			vdisplay = dsi_config->mode->vdisplay;
			vrefresh = dsi_config->mode->vrefresh;
		}
		break;
	case PIPEB:
		reg_offset = 0x1000;
		hdmi_priv = dev_priv->hdmi_priv;
		if (hdmi_priv && hdmi_priv->current_mode) {
			vdisplay = hdmi_priv->current_mode->vdisplay;
			vrefresh = hdmi_priv->current_mode->vrefresh;
		}
		break;
	case PIPEC:
		reg_offset = 0x2000;
		dsi_config = dev_priv->dsi_configs[1];
		if (dsi_config && dsi_config->mode) {
			vdisplay = dsi_config->mode->vdisplay;
			vrefresh = dsi_config->mode->vrefresh;
		}
		break;
	default:
		DRM_ERROR("Invalid pipe %d\n", pipe);
		return false;
	}

	if (vdisplay <= 0) {
		DRM_ERROR("Invalid vertical active region for pipe %d.\n", pipe);
		return false;
	}

	retry = (int)(1000000 / (vrefresh * delay_us));
	dsl_threshold = vdisplay - (int)(1000000 / (vrefresh * plane_flip_time));
	while (--retry && ((REG_READ(PIPEADSL + reg_offset) & PIPE_LINE_CNT_MASK) >= dsl_threshold))
		udelay(delay_us);

	if (!retry) {
		DRM_ERROR("Pipe %d DSL is sticky.\n", pipe);
		return false;
	}

	dsl = REG_READ(PIPEADSL + reg_offset) & PIPE_LINE_CNT_MASK;
	if (dsl >= dsl_threshold)
		DRM_INFO("DSL is at line %u for pipe %d.\n", dsl, pipe);

	return true;
}

void DCCBFlipOverlay(struct drm_device *dev,
			struct intel_dc_overlay_ctx *ctx)
{
	struct drm_psb_private *dev_priv;
	struct mdfld_dsi_config *dsi_config = NULL;
	struct mdfld_dsi_hw_context *dsi_ctx;
	u32 ovadd_reg = OV_OVADD;
	u32 ovadd;

	if (!dev || !ctx)
		return;

	dev_priv = (struct drm_psb_private *)dev->dev_private;

	user_mode_start(dev_priv);

	if (ctx->index == 1)
		ovadd_reg = OVC_OVADD;

	ctx->ovadd |= 1;

	if (ctx->pipe == 0)
		dsi_config = dev_priv->dsi_configs[0];
	else if (ctx->pipe == 2)
		dsi_config = dev_priv->dsi_configs[1];

	if (dsi_config) {
		dsi_ctx = &dsi_config->dsi_hw_context;
		if (ctx->index == 0)
			dsi_ctx->ovaadd = ctx->ovadd;
		else if (ctx->index == 1)
			dsi_ctx->ovcadd = ctx->ovadd;
	}

	ovadd = PSB_RVDC32(ovadd_reg);
	if ((ovadd & BIT15) &&
		((ovadd & OV_PIPE_SELECT) != (ctx->ovadd & OV_PIPE_SELECT))) {
		PSB_WVDC32(ovadd & (~BIT15), ovadd_reg);
		DRM_INFO("overlay dynamic switch, disable first, ovadd 0x%x, flip 0x%x\n",
				ovadd, ctx->ovadd);
	} else {
		PSB_WVDC32(ctx->ovadd, ovadd_reg);
	}
}

void DCCBFlipSprite(struct drm_device *dev,
			struct intel_dc_sprite_ctx *ctx)
{
	struct drm_psb_private *dev_priv;
	struct mdfld_dsi_config *dsi_config = NULL;
	struct mdfld_dsi_hw_context *dsi_ctx;
	struct drm_display_mode *fixed_mode = NULL;
	u32 reg_offset = 0x3000;
	u32 reg_val_pos;
	u32 reg_val_size;
	u32 ovadd_reg = OV_OVADD;
	u32 ovadd;

	if (!dev || !ctx)
		return;

	dev_priv = (struct drm_psb_private *)dev->dev_private;

	user_mode_start(dev_priv);

	if (ctx->index == 1) {
		reg_offset = 0x4000;
	} else if (ctx->index == 2) {
		reg_offset = 0x5000;
	} else if (ctx->index == 0) {
		reg_offset = 0x3000;
	} else {
		DRM_ERROR("%s: invalid index\n", __func__);
	}

	/* assign sprite to pipe */
	ctx->cntr &= ~DISPPLANE_SEL_PIPE_MASK;

	if (ctx->pipe == 1)
		ctx->cntr |= DISPPLANE_SEL_PIPE_B;
	else if (ctx->pipe == 0) {
		ctx->cntr |= DISPPLANE_SEL_PIPE_A;
		dsi_config = dev_priv->dsi_configs[0];
	} else if (ctx->pipe == 2) {
		ctx->cntr |= DISPPLANE_SEL_PIPE_C;
		dsi_config = dev_priv->dsi_configs[1];
	}

	if (dsi_config)
	    fixed_mode = dsi_config->fixed_mode;
	ovadd = PSB_RVDC32(ovadd_reg);

#ifdef CONFIG_AMOLED_SUPPORT
	if (wl_amoled_shift.max_x || wl_amoled_shift.max_y) {
                u32 sprite_offset_x;
                u32 sprite_offset_y;
                u32 sprite_width;
                u32 sprite_height;
		u32 w, h;

		int tmp_hdisplay;
		int tmp_vdisplay;
		switch (get_panel_type(dev, 0)) {
		case SDC_25x16_CMD:
			tmp_hdisplay = 2560;
			tmp_vdisplay = 1600;
			break;
		case SDC_16x25_CMD:
			tmp_hdisplay = 1600;
			tmp_vdisplay = 2560;
			break;
		default:
			tmp_hdisplay = fixed_mode->hdisplay;
			tmp_vdisplay = fixed_mode->vdisplay;
			break;
		}

                reg_val_pos = ctx->pos;
                sprite_offset_x = reg_val_pos & 0x00000fff;
                sprite_offset_y = (reg_val_pos >> 16) & 0x00000fff;
                reg_val_size = ctx->size;
                w = sprite_width = reg_val_size & 0x00000fff;
                h = sprite_height = (reg_val_size >> 16) & 0x00000fff;

	        sprite_offset_x += wl_amoled_shift.curr_x;
	        sprite_offset_y += wl_amoled_shift.curr_y;

		wl_amoled_shift.flip_done = 1;

		if ((ctx->update_mask & SPRITE_UPDATE_POSITION)) {
			if (ovadd == 0x0)
                        	reg_val_pos = (reg_val_pos & 0xf000f000) | sprite_offset_x | (sprite_offset_y << 16);
			else 
                        	reg_val_pos = (reg_val_pos & 0xf000f000) | w | (h << 16);
			PSB_WVDC32(reg_val_pos, DSPAPOS + reg_offset);
                }

                if ((ctx->update_mask & SPRITE_UPDATE_SIZE)) {
			if (fixed_mode) {
                        	if ((sprite_offset_x + sprite_width + 1) > tmp_hdisplay)
					sprite_width = (tmp_hdisplay - 1) - sprite_offset_x;

                        	if ((sprite_offset_y + sprite_height + 1) > tmp_vdisplay)
					sprite_height = (tmp_vdisplay -1) - sprite_offset_y;
			}

			if (ovadd == 0x0) 
                        	reg_val_size = (reg_val_size & 0xf000f000) | sprite_width | (sprite_height << 16);
			else
                        	reg_val_size = (reg_val_size & 0xf000f000) | w | (h << 16);
                        PSB_WVDC32(reg_val_size, DSPASIZE + reg_offset);
                        PSB_WVDC32(ctx->stride, DSPASTRIDE + reg_offset);
                }
	} 
#else
	if ((ctx->update_mask & SPRITE_UPDATE_POSITION))
		PSB_WVDC32(ctx->pos, DSPAPOS + reg_offset);

        if ((ctx->update_mask & SPRITE_UPDATE_SIZE)) {
                PSB_WVDC32(ctx->size, DSPASIZE + reg_offset);
                PSB_WVDC32(ctx->stride, DSPASTRIDE + reg_offset);
        }
#endif
	if ((ctx->update_mask & SPRITE_UPDATE_CONSTALPHA)) {
		PSB_WVDC32(ctx->contalpa, DSPACONSTALPHA + reg_offset);
	}

	if ((ctx->update_mask & SPRITE_UPDATE_CONTROL)){
                if(drm_psb_set_gamma_success)
			PSB_WVDC32(ctx->cntr | DISPPLANE_GAMMA_ENABLE, DSPACNTR + reg_offset);
                else
                        PSB_WVDC32(ctx->cntr, DSPACNTR + reg_offset);
        }

	if ((ctx->update_mask & SPRITE_UPDATE_SURFACE)) {
		PSB_WVDC32(ctx->linoff, DSPALINOFF + reg_offset);
		PSB_WVDC32(ctx->tileoff, DSPATILEOFF + reg_offset);
		PSB_WVDC32(ctx->surf, DSPASURF + reg_offset);
	}

	if (dsi_config) {
		dsi_ctx = &dsi_config->dsi_hw_context;
#ifdef CONFIG_AMOLED_SUPPORT
		if (wl_amoled_shift.max_x || wl_amoled_shift.max_y) {
			dsi_ctx->sprite_dsppos = reg_val_pos;
			dsi_ctx->sprite_dspsize = reg_val_size;
		}
		else {
			dsi_ctx->sprite_dsppos = ctx->pos;
			dsi_ctx->sprite_dspsize = ctx->size;
		}
#else
		dsi_ctx->sprite_dsppos = ctx->pos;
		dsi_ctx->sprite_dspsize = ctx->size;
#endif
		dsi_ctx->sprite_dspstride = ctx->stride;
		dsi_ctx->sprite_dspcntr = ctx->cntr | ((PSB_RVDC32(DSPACNTR + reg_offset) & DISPPLANE_GAMMA_ENABLE));
		dsi_ctx->sprite_dsplinoff = ctx->linoff;
		dsi_ctx->sprite_dspsurf = ctx->surf;
	}
}

void DCCBFlipPrimary(struct drm_device *dev,
			struct intel_dc_primary_ctx *ctx)
{
	struct drm_psb_private *dev_priv;
	struct mdfld_dsi_config *dsi_config = NULL;
	struct mdfld_dsi_hw_context *dsi_ctx;
	struct drm_display_mode *fixed_mode = NULL;
	u32 reg_offset;
	int pipe;
	u32 reg_val_pos;
	u32 reg_val_size;

	if (!dev || !ctx)
		return;

	dev_priv = (struct drm_psb_private *)dev->dev_private;

	user_mode_start(dev_priv);

	if (ctx->index == 0) {
		reg_offset = 0;
		dsi_config = dev_priv->dsi_configs[0];
		pipe = 0;
	} else if (ctx->index == 1) {
		reg_offset = 0x1000;
		pipe = 1;
	} else if (ctx->index == 2) {
		reg_offset = 0x2000;
		dsi_config = dev_priv->dsi_configs[1];
		pipe = 2;
	} else
		return;

	if (dsi_config)
	    fixed_mode = dsi_config->fixed_mode;
	
#ifdef CONFIG_AMOLED_SUPPORT
	if (wl_amoled_shift.max_x || wl_amoled_shift.max_y) {
                u32 sprite_offset_x;
                u32 sprite_offset_y;
                u32 sprite_width;
                u32 sprite_height;

		int tmp_hdisplay;
		int tmp_vdisplay;

		switch (get_panel_type(dev, 0)) {
			case SDC_25x16_CMD:
				tmp_hdisplay = 2560;
				tmp_vdisplay = 1600;
				break;
			case SDC_16x25_CMD:
				tmp_hdisplay = 1600;
				tmp_vdisplay = 2560;
				break;
			default:
				tmp_hdisplay = fixed_mode->hdisplay;
				tmp_vdisplay = fixed_mode->vdisplay;
				break;
		}

                reg_val_pos = ctx->pos;
                primary_offset_x = sprite_offset_x = reg_val_pos & 0x00000fff;
                primary_offset_y = sprite_offset_y = (reg_val_pos >> 16) & 0x00000fff;
                reg_val_size = ctx->size;
                
		sprite_width = reg_val_size & 0x00000fff;
                sprite_height = (reg_val_size >> 16) & 0x00000fff;
 
                primary_width = tmp_hdisplay;
                primary_height = tmp_vdisplay;

	        sprite_offset_x = wl_amoled_shift.curr_x;
	        sprite_offset_y = wl_amoled_shift.curr_y;

		wl_amoled_shift.flip_done = 1;

		if ((ctx->update_mask & SPRITE_UPDATE_POSITION)) {
			reg_val_pos = (reg_val_pos & 0xf000f000) | sprite_offset_x | (sprite_offset_y << 16);
			PSB_WVDC32(reg_val_pos, DSPAPOS + reg_offset);
                }

                if ((ctx->update_mask & SPRITE_UPDATE_SIZE)) {
			if (fixed_mode) {
                        	if ((sprite_offset_x + sprite_width + 1) > tmp_hdisplay)
					sprite_width = (tmp_hdisplay - 1) - sprite_offset_x;

                        	if ((sprite_offset_y + sprite_height + 1) > tmp_vdisplay)
					sprite_height = (tmp_vdisplay -1) - sprite_offset_y;
			}
                        reg_val_size  = (reg_val_size & 0xf000f000) | sprite_width | (sprite_height << 16);
                        PSB_WVDC32(reg_val_size, DSPASIZE + reg_offset);
                        PSB_WVDC32(ctx->stride, DSPASTRIDE + reg_offset);
                }
	} 
#else
	if ((ctx->update_mask & SPRITE_UPDATE_POSITION))
		PSB_WVDC32(ctx->pos, DSPAPOS + reg_offset);

        if ((ctx->update_mask & SPRITE_UPDATE_SIZE)) {
                PSB_WVDC32(ctx->size, DSPASIZE + reg_offset);
                PSB_WVDC32(ctx->stride, DSPASTRIDE + reg_offset);
        }
#endif

	if ((ctx->update_mask & SPRITE_UPDATE_CONSTALPHA)) {
		PSB_WVDC32(ctx->contalpa, DSPACONSTALPHA + reg_offset);
	}

	if ((ctx->update_mask & SPRITE_UPDATE_CONTROL)){
                if(drm_psb_set_gamma_success)
                        PSB_WVDC32(ctx->cntr | DISPPLANE_GAMMA_ENABLE, DSPACNTR + reg_offset);
                else
                        PSB_WVDC32(ctx->cntr, DSPACNTR + reg_offset);
        }

	/* enable/disable double scan */
/*
#ifdef CONFIG_PSB_ZOOM
	ctx->cntr |= (0x1 << 20);
	PSB_WVDC32(ctx->cntr, DSPACNTR + reg_offset);
#else
	ctx->cntr &= ~(1UL << 20);
#endif
*/

	if ((ctx->update_mask & SPRITE_UPDATE_SURFACE)) {
		PSB_WVDC32(ctx->linoff, DSPALINOFF + reg_offset);
		PSB_WVDC32(ctx->tileoff, DSPATILEOFF + reg_offset);
		PSB_WVDC32(ctx->surf, DSPASURF + reg_offset);
	}

	if (dsi_config) {
		dsi_ctx = &dsi_config->dsi_hw_context;
#ifdef CONFIG_AMOLED_SUPPORT
		if (wl_amoled_shift.max_x || wl_amoled_shift.max_y) {
			dsi_ctx->dsppos = reg_val_pos;
			dsi_ctx->dspsize = reg_val_size;
		}
		else {
			dsi_ctx->dsppos = ctx->pos;
			dsi_ctx->dspsize = ctx->size;
		}
#else
		dsi_ctx->dsppos = ctx->pos;
		dsi_ctx->dspsize = ctx->size;
#endif
		dsi_ctx->dspstride = ctx->stride;
		dsi_ctx->dspcntr = ctx->cntr | ((PSB_RVDC32(DSPACNTR + reg_offset) & DISPPLANE_GAMMA_ENABLE));
		dsi_ctx->dsplinoff = ctx->linoff;
		dsi_ctx->dspsurf = ctx->surf;
	}
}

void DCCBFlipCursor(struct drm_device *dev,
			struct intel_dc_cursor_ctx *ctx)
{
	struct drm_psb_private *dev_priv;
	u32 reg_offset = 0;

	if (!dev || !ctx)
		return;

	dev_priv = (struct drm_psb_private *)dev->dev_private;

	user_mode_start(dev_priv);

	switch (ctx->pipe) {
	case 0:
		reg_offset = 0;
		break;
	case 1:
		reg_offset = 0x40;
		break;
	case 2:
		reg_offset = 0x60;
		break;
	}

	PSB_WVDC32(ctx->cntr, CURACNTR + reg_offset);
	PSB_WVDC32(ctx->pos, CURAPOS + reg_offset);
	PSB_WVDC32(ctx->surf, CURABASE + reg_offset);
}

void DCCBSetupZorder(struct drm_device *dev,
			struct intel_dc_plane_zorder *zorder,
			int pipe)
{

}

void DCCBSetPipeToOvadd(u32 *ovadd, int pipe)
{
	switch (pipe) {
	case 0:
		*ovadd |= OV_PIPE_A << OV_PIPE_SELECT_POS;
		break;
	case 1:
		*ovadd |= OV_PIPE_B << OV_PIPE_SELECT_POS;
		break;
	case 2:
		*ovadd |= OV_PIPE_C << OV_PIPE_SELECT_POS;
		break;
	}

	return;
}

static int _GetPipeFromOvadd(u32 ovadd)
{
	int ov_pipe_sel = (ovadd & OV_PIPE_SELECT) >> OV_PIPE_SELECT_POS;
	int pipe = 0;
	switch (ov_pipe_sel) {
	case OV_PIPE_A:
		pipe = 0;
		break;
	case OV_PIPE_B:
		pipe = 1;
		break;
	case OV_PIPE_C:
		pipe = 2;
		break;
	}

	return pipe;
}

static void _OverlayWaitVblank(struct drm_device *dev, int pipe)
{
	union drm_wait_vblank vblwait;
	int ret;

	vblwait.request.type =
		(_DRM_VBLANK_RELATIVE |
		 _DRM_VBLANK_NEXTONMISS);
	vblwait.request.sequence = 1;

	if (pipe == 1)
		vblwait.request.type |=
			_DRM_VBLANK_SECONDARY;

	ret = drm_wait_vblank(dev, (void *)&vblwait, 0);
	if (ret) {
		DRM_ERROR("%s: fail to wait vsync of pipe %d\n", __func__, pipe);
	}
}

static void _OverlayWaitFlip(struct drm_device *dev, u32 ovstat_reg,
			int index, int pipe)
{
	int retry;
	int ret = -EBUSY;

	/* HDMI pipe can run as low as 24Hz */
	retry = 600;
	if (pipe != 1) {
		retry = 200;  /* 60HZ for MIPI */
		DCCBDsrForbid(dev, pipe);
	}
	/**
	 * make sure overlay command buffer
	 * was copied before updating the system
	 * overlay command buffer.
	 */
	while (--retry) {
		if (pipe != 1 && ret == -EBUSY) {
			ret = DCCBUpdateDbiPanel(dev, pipe);
		}
		if (BIT31 & PSB_RVDC32(ovstat_reg))
			break;
		udelay(100);
	}

	if (pipe != 1)
		DCCBDsrAllow(dev, pipe);

	if (!retry)
		DRM_ERROR("OVADD %d flip timeout on pipe %d!\n", index, pipe);
}

int DCCBOverlayDisableAndWait(struct drm_device *dev, u32 ctx,
			int index)
{
	u32 ovadd_reg = OV_OVADD;
	u32 ovstat_reg = OV_DOVASTA;
	u32 power_islands = OSPM_DISPLAY_A;
	int pipe;

	if (index != 0 && index != 1) {
		DRM_ERROR("Invalid overlay index %d\n", index);
		return -EINVAL;
	}

	if (index) {
		ovadd_reg = OVC_OVADD;
		ovstat_reg = OVC_DOVCSTA;
		power_islands |= OSPM_DISPLAY_C;
	}

	pipe = _GetPipeFromOvadd(ctx);

	if (power_island_get(power_islands)) {
		PSB_WVDC32(ctx, ovadd_reg);

		/*wait for overlay flipped*/
		_OverlayWaitFlip(dev, ovstat_reg, index, pipe);

		power_island_put(power_islands);
	}
	return 0;
}

int DCCBOverlayEnable(struct drm_device *dev, u32 ctx,
			int index, int enabled)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	struct mdfld_dsi_config *dsi_config = NULL;
	struct mdfld_dsi_hw_context *dsi_ctx;
	u32 ovadd_reg = OV_OVADD;
	u32 ovstat_reg = OV_DOVASTA;
	u32 power_islands = OSPM_DISPLAY_A;
	int pipe;
	uint32_t pipeconf = PIPEACONF;

	if (index != 0 && index != 1) {
		DRM_ERROR("Invalid overlay index %d\n", index);
		return -EINVAL;
	}

	if (index) {
		ovadd_reg = OVC_OVADD;
		ovstat_reg = OVC_DOVCSTA;
		power_islands |= OSPM_DISPLAY_C;
	}

	pipe = _GetPipeFromOvadd(ctx);

	if (!enabled) {
		if (pipe == 0)
			dsi_config = dev_priv->dsi_configs[0];
		else if (pipe == 2)
			dsi_config = dev_priv->dsi_configs[1];

		if (dsi_config) {
			dsi_ctx = &dsi_config->dsi_hw_context;
			if (index == 0)
				dsi_ctx->ovaadd = 0;
			else if (index == 1)
				dsi_ctx->ovcadd = 0;
		}
	}

	if (pipe == 1)
		power_islands |= OSPM_DISPLAY_B;
	else if (pipe == 2)
		power_islands |= OSPM_DISPLAY_C;

	if (power_island_get(power_islands)) {
		/*make sure previous flip was done*/
		//_OverlayWaitFlip(dev, ovstat_reg, index, pipe);
		//_OverlayWaitVblank(dev, pipe);

		pipeconf = PIPEACONF + 0x1000 * pipe;
		if (PSB_RVDC32(pipeconf) & BIT31) {
			/* avoid writing ovadd during vblank perioid */
			DCCBAvoidFlipInVblankInterval(dev, pipe);
		}

		PSB_WVDC32(ctx, ovadd_reg);

		power_island_put(power_islands);
	}
	return 0;
}

int DCCBSpriteEnable(struct drm_device *dev, u32 ctx,
			int index, int enabled)
{
	u32 power_islands = (OSPM_DISPLAY_A | OSPM_DISPLAY_C);
	struct drm_psb_private *dev_priv = dev->dev_private;
	struct mdfld_dsi_config *dsi_config = NULL;
	struct mdfld_dsi_hw_context *dsi_ctx = NULL;
	u32 reg_offset;
	u32 dspcntr_reg = DSPACNTR;
	u32 dspsurf_reg = DSPASURF;

	switch (index) {
	case 0:
		reg_offset = 0x3000;
		break;
	case 1:
		reg_offset = 0x4000;
		break;
	case 2:
		reg_offset = 0x5000;
		break;
	default:
		DRM_ERROR("Invalid sprite index %d\n", index);
		return -EINVAL;
	}

	/* FIXME: need to check pipe info here. */
	dsi_config = dev_priv->dsi_configs[0];

	if (dsi_config)
		dsi_ctx = &dsi_config->dsi_hw_context;

	dspcntr_reg += reg_offset;
	dspsurf_reg += reg_offset;

	if (power_island_get(power_islands)) {
		if (dsi_ctx)
			dsi_ctx->sprite_dspcntr &= ~DISPLAY_PLANE_ENABLE;
		PSB_WVDC32((PSB_RVDC32(dspcntr_reg) & ~DISPLAY_PLANE_ENABLE),
				dspcntr_reg);
		PSB_WVDC32((PSB_RVDC32(dspsurf_reg)), dspsurf_reg);
		power_island_put(power_islands);
	}

	return 0;
}

int DCCBPrimaryEnable(struct drm_device *dev, u32 ctx,
			int index, int enabled)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	struct psb_gtt *pg = dev_priv->pg;
	struct mdfld_dsi_config *dsi_config = NULL;
	struct mdfld_dsi_hw_context *dsi_ctx = NULL;
	u32 reg_offset;

	if (index < 0 || index > 2) {
		DRM_ERROR("Invalid primary index %d\n", index);
		return -EINVAL;
	}

	if (index == 0) {
		dsi_config = dev_priv->dsi_configs[0];
		reg_offset = 0;
	} else if (index == 1) {
		reg_offset = 0x1000;
	} else if (index == 2) {
		dsi_config = dev_priv->dsi_configs[1];
		reg_offset = 0x2000;
	}

	if (dsi_config) {
		dsi_ctx = &dsi_config->dsi_hw_context;
		dsi_ctx->dsppos = 0;
		dsi_ctx->dspsize = (63 << 16) | 63;
		dsi_ctx->dspstride = (64 << 2);
		dsi_ctx->dspcntr = DISPPLANE_32BPP;
		dsi_ctx->dspcntr |= DISPPLANE_PREMULT_DISABLE;
		dsi_ctx->dspcntr |= (BIT31 & PSB_RVDC32(DSPACNTR + reg_offset));
		dsi_ctx->dsplinoff = 0;
		dsi_ctx->dspsurf = pg->reserved_gtt_start;
	}

	PSB_WVDC32(0, DSPAPOS + reg_offset);
	PSB_WVDC32((63 << 16) | 63, DSPASIZE + reg_offset);
	PSB_WVDC32((64 << 2), DSPASTRIDE + reg_offset);
	PSB_WVDC32(0x1c800000 | (BIT31 & PSB_RVDC32(DSPACNTR + reg_offset)),
		DSPACNTR + reg_offset);
	PSB_WVDC32(0, DSPALINOFF + reg_offset);
	PSB_WVDC32(0, DSPATILEOFF + reg_offset);
	PSB_WVDC32(pg->reserved_gtt_start, DSPASURF + reg_offset);

	return 0;
}

int DCCBCursorDisable(struct drm_device *dev, int index)
{
	u32 reg_offset;

	if (index < 0 || index > 2) {
		DRM_ERROR("Invalid cursor index %d\n", index);
		return -EINVAL;
	}

	switch (index) {
	case 0:
		reg_offset = 0;
		break;
	case 1:
		reg_offset = 0x40;
		break;
	case 2:
		reg_offset = 0x60;
		break;
	}

	PSB_WVDC32(0, CURACNTR + reg_offset);
	PSB_WVDC32(0, CURAPOS + reg_offset);
	PSB_WVDC32(0, CURABASE + reg_offset);

	return 0;
}

int DCCBUpdateDbiPanel(struct drm_device *dev, int pipe)
{
	struct drm_psb_private *dev_priv =
		(struct drm_psb_private *)dev->dev_private;
	struct mdfld_dsi_config *dsi_config = NULL;

	if ((pipe != 0) && (pipe != 2))
		return -EINVAL;

	if (dev_priv && dev_priv->dsi_configs)
		dsi_config = (pipe == 0) ?
			dev_priv->dsi_configs[0] : dev_priv->dsi_configs[1];

	return mdfld_dsi_dsr_update_panel_fb(dsi_config);
}

void DCCBWaitForDbiFifoEmpty(struct drm_device *dev, int pipe)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	struct mdfld_dsi_config *dsi_config;
	int retry;

	if ((pipe != 0) && (pipe != 2))
		return;

	dsi_config = (pipe == 0) ? dev_priv->dsi_configs[0] :
				   dev_priv->dsi_configs[1];

	if (!dsi_config || dsi_config->type != MDFLD_DSI_ENCODER_DBI)
		return;

	/* shall we use FLIP_DONE on ANN? */
	if (IS_TNG_B0(dev)) {
		retry = wait_event_interruptible_timeout(dev_priv->eof_wait,
				(REG_READ(MIPIA_GEN_FIFO_STAT_REG) & BIT27),
				msecs_to_jiffies(1000));
	} else {
		retry = 1000;
		while (retry-- && !(REG_READ(MIPIA_GEN_FIFO_STAT_REG)))
			udelay(500);
	}

	if (retry == 0)
		DRM_ERROR("DBI FIFO not empty\n");
}

void DCCBAvoidFlipInVblankInterval(struct drm_device *dev, int pipe)
{
	if ((pipe == PIPEB) ||
		(is_panel_vid_or_cmd(dev) == MDFLD_DSI_ENCODER_DPI))
		is_vblank_period(dev, pipe);
}

void DCCBUnblankDisplay(struct drm_device *dev)
{
	int res;
	struct psb_framebuffer *psb_fb = NULL;

	DCCBGetFramebuffer(dev, &psb_fb);

	if (!psb_fb)
		return;

	console_lock();
	res = fb_blank(psb_fb->fbdev, 0);
	console_unlock();
	if (res != 0) {
		DRM_ERROR("fb_blank failed (%d)", res);
	}
}

void DCCBFlipDSRCb(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv =
	    (struct drm_psb_private *)dev->dev_private;

	if (!dev_priv->um_start) {
		dev_priv->um_start = true;

		if (dev_priv->b_dsr_enable_config)
			dev_priv->b_dsr_enable = true;
	}

	if (dev_priv->b_dsr_enable && dev_priv->b_is_in_idle) {
		dev_priv->exit_idle(dev, MDFLD_DSR_2D_3D, NULL, true);
	}
}

u32 DCCBGetPipeCount(void)
{
	/* FIXME */
	return 3;
}

bool DCCBIsSuspended(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv =
		(struct drm_psb_private *)dev->dev_private;
	bool ret = false;

	if (!dev_priv)
		return false;

	mutex_lock(&dev->mode_config.mutex);
	ret = dev_priv->early_suspended;
	mutex_unlock(&dev->mode_config.mutex);

	return ret;
}

int DCCBIsPipeActive(struct drm_device *dev, int pipe)
{
	struct drm_psb_private *dev_priv =
		(struct drm_psb_private *)dev->dev_private;
	struct mdfld_dsi_config *dsi_config = NULL;
	u32 pipeconf_reg;
	int active = 0;

	if (pipe == 0)
		pipeconf_reg = PIPEACONF;
	else if (pipe == 1)
		pipeconf_reg = PIPEBCONF;
	else {
		DRM_ERROR("%s: unsupported pipe %d\n", __func__, pipe);
		return 0;
	}

	/* FIXME: need to remove the suspended state checking. */
	if (dev_priv->early_suspended)
		return 0;

	/* get display a for register reading */
	if (power_island_get(OSPM_DISPLAY_A)) {
		if ((pipe != 1) && dev_priv->dsi_configs) {
			dsi_config = (pipe == 0) ? dev_priv->dsi_configs[0] :
				dev_priv->dsi_configs[1];
		}

		mdfld_dsi_dsr_forbid(dsi_config);

		active = (PSB_RVDC32(pipeconf_reg) & BIT31) ? 1 : 0 ;

		mdfld_dsi_dsr_allow(dsi_config);

		power_island_put(OSPM_DISPLAY_A);
	}

	return active;
}

void DCCBDsrForbid(struct drm_device *dev, int pipe)
{
	struct drm_psb_private *dev_priv =
		(struct drm_psb_private *)dev->dev_private;
	struct mdfld_dsi_config *dsi_config = NULL;

	if ((pipe != 0) && (pipe != 2))
		return;

	if (dev_priv && dev_priv->dsi_configs)
		dsi_config = (pipe == 0) ?
			dev_priv->dsi_configs[0] : dev_priv->dsi_configs[1];

	mdfld_dsi_dsr_forbid(dsi_config);
}

void DCCBDsrAllow(struct drm_device *dev, int pipe)
{
	struct drm_psb_private *dev_priv =
		(struct drm_psb_private *)dev->dev_private;
	struct mdfld_dsi_config *dsi_config = NULL;

	if ((pipe != 0) && (pipe != 2))
		return;

	if (dev_priv && dev_priv->dsi_configs)
		dsi_config = (pipe == 0) ?
			dev_priv->dsi_configs[0] : dev_priv->dsi_configs[1];

	mdfld_dsi_dsr_allow(dsi_config);
}

int DCCBUpdateCursorPos(struct drm_device *dev, int pipe, uint32_t pos)
{
	u32 power_island = 0;
	u32 reg_offset = 0;

	switch (pipe) {
	case 0:
		power_island = OSPM_DISPLAY_A;
		reg_offset = 0;
		break;
	case 1:
		power_island = OSPM_DISPLAY_B;
		reg_offset = 0x40;
		break;
	case 2:
		power_island = OSPM_DISPLAY_C;
		reg_offset = 0x60;
		break;
	default:
		DRM_ERROR("%s: invalid pipe %d\n", __func__, pipe);
		return -1;
	}

	if (!power_island_get(power_island)) {
		DRM_ERROR("%s: failed to get power island for pipe %d\n", __func__, pipe);
		return -1;
	}

	PSB_WVDC32(pos, CURAPOS + reg_offset);
	power_island_put(power_island);
	return 0;
}

int DCCBDumpPipeStatus(struct drm_device *dev, int pipe)
{
	u32 power_island = 0;
	u32 reg_offset = 0;

	switch (pipe) {
	case 0:
		power_island = OSPM_DISPLAY_A;
		reg_offset = 0;
		break;
	case 1:
		power_island = OSPM_DISPLAY_B;
		reg_offset = 0x1000;
		break;
	case 2:
		power_island = OSPM_DISPLAY_C;
		reg_offset = 0x2000;
		break;
	default:
		DRM_ERROR("%s: invalid pipe %d\n", __func__, pipe);
		return -1;
	}

	if (!power_island_get(power_island)) {
		DRM_ERROR("%s: failed to get power island for pipe %d\n", __func__, pipe);
		return -1;
	}

	DRM_INFO("========= status on pipe%d ========\n", pipe);
	DRM_INFO("vblank_refcount = %u\n", atomic_read(&dev->vblank_refcount[pipe]));
	DRM_INFO("vblank_count = %u\n", drm_vblank_count(dev, pipe));
	DRM_INFO("PIPECONF = 0x%08x\n", REG_READ(PIPEACONF+reg_offset));
	DRM_INFO("PIPESTAT = 0x%08x\n\n", REG_READ(PIPEASTAT+reg_offset));
	DRM_INFO("===================================\n");

	power_island_put(power_island);
	return 0;
}
