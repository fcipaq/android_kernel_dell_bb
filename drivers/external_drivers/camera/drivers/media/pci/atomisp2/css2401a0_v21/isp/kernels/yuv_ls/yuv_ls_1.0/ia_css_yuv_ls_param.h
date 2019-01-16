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

#ifndef __IA_CSS_YUV_LS_PARAM_H
#define __IA_CSS_YUV_LS_PARAM_H

#include "type_support.h"

/* The number of load/store kernels in a pipeline can be greater than one.
 * A kernel can consume more than one input or can produce more
 * than one output.
 */
#define NUM_YUV_LS 2

/** YUV load/store */
struct sh_css_isp_yuv_ls_isp_config {
	unsigned base_address[NUM_YUV_LS];
	unsigned width[NUM_YUV_LS];
	unsigned height[NUM_YUV_LS];
	unsigned stride[NUM_YUV_LS];
};


#endif /* __IA_CSS_YUV_LS_PARAM_H */
