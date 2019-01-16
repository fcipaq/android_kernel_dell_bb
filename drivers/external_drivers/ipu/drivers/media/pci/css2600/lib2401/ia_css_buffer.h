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

#ifndef __IA_CSS_BUFFER_H
#define __IA_CSS_BUFFER_H

/** @file
 * This file contains datastructures and types for buffers used in CSS
 */

#include <type_support.h>
#include "ia_css_types.h"
#include "ia_css_timer.h"

/** Enumeration of buffer types. Buffers can be queued and de-queued
 *  to hand them over between IA and ISP.
 */
enum ia_css_buffer_type {
	IA_CSS_BUFFER_TYPE_INVALID = -1,
	IA_CSS_BUFFER_TYPE_3A_STATISTICS = 0,
	IA_CSS_BUFFER_TYPE_DIS_STATISTICS,
	IA_CSS_BUFFER_TYPE_LACE_STATISTICS,
	IA_CSS_BUFFER_TYPE_INPUT_FRAME,
	IA_CSS_BUFFER_TYPE_OUTPUT_FRAME,
	IA_CSS_BUFFER_TYPE_SEC_OUTPUT_FRAME,
	IA_CSS_BUFFER_TYPE_VF_OUTPUT_FRAME,
	IA_CSS_BUFFER_TYPE_SEC_VF_OUTPUT_FRAME,
	IA_CSS_BUFFER_TYPE_RAW_OUTPUT_FRAME,
	IA_CSS_BUFFER_TYPE_CUSTOM_INPUT,
	IA_CSS_BUFFER_TYPE_CUSTOM_OUTPUT,
	IA_CSS_BUFFER_TYPE_METADATA,
	IA_CSS_BUFFER_TYPE_PARAMETER_SET,
	IA_CSS_BUFFER_TYPE_PER_FRAME_PARAMETER_SET,
	IA_CSS_NUM_DYNAMIC_BUFFER_TYPE,
	IA_CSS_NUM_BUFFER_TYPE
};

/** Buffer structure. This is a container structure that enables content
 *  independent buffer queues and access functions.
 */
struct ia_css_buffer {
	enum ia_css_buffer_type type; /**< Buffer type. */
	unsigned int exp_id; /**< exposure id for this buffer; 0 = not available; currently only implemented for buffered sensor mode */
	union {
		struct ia_css_isp_3a_statistics  *stats_3a;    /**< 3A statistics & optionally RGBY statistics. */
		struct ia_css_isp_dvs_statistics *stats_dvs;   /**< DVS statistics. */
		struct ia_css_isp_skc_dvs_statistics *stats_skc_dvs;  /**< SKC DVS statistics. */
		struct ia_css_isp_lace_statistics *stats_lace; /**< LACE statistics. */
		struct ia_css_frame              *frame;       /**< Frame buffer. */
		struct ia_css_acc_param          *custom_data; /**< Custom buffer. */
		struct ia_css_metadata           *metadata;    /**< Sensor metadata. */
	} data; /**< Buffer data pointer. */
	uint64_t driver_cookie; /**< cookie for the driver */
	struct ia_css_time_meas timing_data; /**< timing data (readings from the timer) */
};

/** @brief Dequeue param buffers from sp2host_queue
 *
 * @return                                       None
 *
 * This function must be called at every driver interrupt handler to prevent
 * overflow of sp2host_queue.
 */
void
ia_css_dequeue_param_buffers(void);

#endif /* __IA_CSS_BUFFER_H */
