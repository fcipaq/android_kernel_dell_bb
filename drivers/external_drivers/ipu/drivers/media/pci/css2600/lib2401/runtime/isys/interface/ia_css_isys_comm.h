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

#ifndef __IA_CSS_ISYS_COMM_H
#define __IA_CSS_ISYS_COMM_H

#include <type_support.h>
#include <input_system.h>

#ifdef USE_INPUT_SYSTEM_VERSION_2401
#include <input_system_global.h>

/*
 * a) ia_css_isys_stream_h & ia_css_isys_stream_cfg_t come from host.
 *
 * b) Here it is better  to use actual structures for stream handle
 * instead of opaque handles. Otherwise, we need to have another
 * communication channel to interpret that opaque handle(this handle is
 * maintained by host and needs to be populated to sp for every stream open)
 * */
typedef virtual_input_system_stream_t		*ia_css_isys_stream_h;
typedef virtual_input_system_stream_cfg_t	ia_css_isys_stream_cfg_t;

/*
 * error check for ISYS APIs.
 * */
typedef bool ia_css_isys_error_t;

#endif  /* USE_INPUT_SYSTEM_VERSION_2401*/
#endif  /*_IA_CSS_ISYS_COMM_H */
