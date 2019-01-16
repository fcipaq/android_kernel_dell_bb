/*
 * Support for Intel Camera Imaging ISP subsystem.
 *
 * Copyright (c) 2010 - 2014 Intel Corporation. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

#ifndef __IA_CSS_EED1_8_PARAM_H
#define __IA_CSS_EED1_8_PARAM_H

#include "type_support.h"

#include "ia_css_eed1_8_types.h" /* IA_CSS_NUMBER_OF_DEW_ENHANCE_SEGMENTS */

/* Define size of the state..... TODO: check if this is the correct place */
/* 4 planes : GR, R, B, GB */
#define NUM_PLANES	4
/* 5 lines state per color plane input_line_state */
#define EED1_8_STATE_INPUT_BUFFER_HEIGHT	(4 * NUM_PLANES)

/* ToDo: Move this to testsetup */
#define MAX_FRAME_SIMDWIDTH	30

/* Each plane has width equal to half frame line */
#define EED1_8_STATE_INPUT_BUFFER_WIDTH	CEIL_DIV(MAX_FRAME_SIMDWIDTH, 2)

/* 3 lines state for R and B (= 2 planes) rb_zipped_state */
#define EED1_8_STATE_RB_ZIPPED_HEIGHT	(2 * 2)
#define EED1_8_STATE_RB_ZIPPED_WIDTH	CEIL_DIV(MAX_FRAME_SIMDWIDTH, 2)

/* 1 full input line (GR-R color line) for Yc state */
#define EED1_8_STATE_YC_HEIGHT	1
#define EED1_8_STATE_YC_WIDTH	MAX_FRAME_SIMDWIDTH

/* 2 lines state per color plane Cg_state */
#define EED1_8_STATE_CG_HEIGHT	(2 * NUM_PLANES)
#define EED1_8_STATE_CG_WIDTH	CEIL_DIV(MAX_FRAME_SIMDWIDTH, 2)

/* 2 lines state per color plane Co_state */
#define EED1_8_STATE_CO_HEIGHT	(2 * NUM_PLANES)
#define EED1_8_STATE_CO_WIDTH	CEIL_DIV(MAX_FRAME_SIMDWIDTH, 2)

/* 1 full input line (GR-R color line) for AbsK state */
#define EED1_8_STATE_ABSK_HEIGHT	1
#define EED1_8_STATE_ABSK_WIDTH		MAX_FRAME_SIMDWIDTH


/* EED (Edge Enhancing Demosaic) ISP parameters */
struct ia_css_isp_eed1_8_params {
	int32_t rbzp_strength;

	int32_t fcstrength;
	int32_t fcthres_0;
	int32_t fc_sat_coef;
	int32_t fc_coring_prm;
	int32_t fc_slope;

	int32_t aerel_thres0;
	int32_t aerel_gain0;
	int32_t aerel_thres_diff;
	int32_t aerel_gain_diff;

	int32_t derel_thres0;
	int32_t derel_gain0;
	int32_t derel_thres1;
	int32_t derel_gain1;

	int32_t coring_pos0;
	int32_t coring_pos_diff;
	int32_t coring_neg0;
	int32_t coring_neg_diff;

	int32_t gain;
	int32_t gain_pos0;
	int32_t gain_pos_diff;
	int32_t gain_neg0;
	int32_t gain_neg_diff;

	int32_t pos_margin0;
	int32_t margin_pos_diff;
	int32_t neg_margin0;
	int32_t margin_neg_diff;

	int32_t dew_enhance_seg_x[IA_CSS_NUMBER_OF_DEW_ENHANCE_SEGMENTS];
	int32_t dew_enhance_seg_y[IA_CSS_NUMBER_OF_DEW_ENHANCE_SEGMENTS];
	int32_t dedgew_max;
};

#endif /* __IA_CSS_EED1_8_PARAM_H */
