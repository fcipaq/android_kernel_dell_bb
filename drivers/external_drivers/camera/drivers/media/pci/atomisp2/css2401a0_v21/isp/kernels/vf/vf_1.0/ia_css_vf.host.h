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

#ifndef __IA_CSS_VF_HOST_H
#define __IA_CSS_VF_HOST_H

#include "ia_css_frame_public.h"
#include "ia_css_binary.h"

#include "ia_css_vf_types.h"
#include "ia_css_vf_param.h"

/* compute the log2 of the downscale factor needed to get closest
 * to the requested viewfinder resolution on the upper side. The output cannot
 * be smaller than the requested viewfinder resolution.
 */
enum ia_css_err
sh_css_vf_downscale_log2(
	const struct ia_css_frame_info *out_info,
	const struct ia_css_frame_info *vf_info,
	unsigned int *downscale_log2);

void
ia_css_vf_config(
	struct sh_css_isp_vf_isp_config *to,
	const struct ia_css_vf_configuration *from,
	unsigned size);

enum ia_css_err
ia_css_vf_configure(
	const struct ia_css_binary *binary,
	const struct ia_css_frame_info *out_info,
	struct ia_css_frame_info *vf_info,
	unsigned int *downscale_log2);

#endif /* __IA_CSS_VF_HOST_H */
