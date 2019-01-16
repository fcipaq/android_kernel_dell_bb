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

#ifndef _IA_CSS_PROGRAM_GROUP_PARAM_PRIVATE_H_
#define _IA_CSS_PROGRAM_GROUP_PARAM_PRIVATE_H_

#include <ia_css_program_group_param.h>
#include <ia_css_psys_manifest_types.h>
#include <ia_css_psys_program_group_manifest.h>
#include <ia_css_psys_terminal_manifest.h>
#include <ia_css_kernel_bitmap.h>
#include <ia_css_program_group_data.h>
#include <type_support.h>

#define SIZE_OF_PROGRAM_GROUP_PARAM_STRUCT_IN_BITS \
	(IA_CSS_KERNEL_BITMAP_BITS \
	+ (3 * IA_CSS_UINT32_T_BITS) \
	+ IA_CSS_UINT16_T_BITS \
	+ (2 * IA_CSS_UINT8_T_BITS))

/* tentative; co-design with ISP algorithm */
struct ia_css_program_group_param_s {
	ia_css_kernel_bitmap_t				kernel_enable_bitmap;						/**< The enable bits for each individual kernel */
	uint32_t							size;										/**< Size of this structure */
	uint32_t		program_param_offset;
	uint32_t		terminal_param_offset;
	uint16_t							fragment_count;								/**< Number of (explicit) fragments to use in a frame */
	uint8_t								program_count;								/**< Number of active programs */
	uint8_t								terminal_count;								/**< Number of active terminals */
};

#define SIZE_OF_PROGRAM_PARAM_STRUCT_IN_BITS \
	(IA_CSS_KERNEL_BITMAP_BITS \
	+ IA_CSS_UINT32_T_BITS \
	+ IA_CSS_INT32_T_BITS)

/* private */
struct ia_css_program_param_s {
	ia_css_kernel_bitmap_t				kernel_enable_bitmap;						/**< What to use this one for ? */
	uint32_t							size;										/**< Size of this structure */
	int32_t 		parent_offset; /**< offset to add to reach parent. This is negative value.*/
};

#define SIZE_OF_TERMINAL_PARAM_STRUCT_IN_BITS \
	(IA_CSS_UINT32_T_BITS \
	+ IA_CSS_FRAME_FORMAT_TYPE_BITS \
	+ IA_CSS_INT32_T_BITS \
	+ (IA_CSS_UINT16_T_BITS * IA_CSS_N_DATA_DIMENSION) \
	+ (IA_CSS_UINT16_T_BITS * IA_CSS_N_DATA_DIMENSION) \
	+ IA_CSS_INT32_T_BITS \
	+ IA_CSS_UINT16_T_BITS \
	+ IA_CSS_UINT8_T_BITS \
	+ (IA_CSS_UINT8_T_BITS * 1))

#endif /*_IA_CSS_PROGRAM_GROUP_PARAM_PRIVATE_H_*/
