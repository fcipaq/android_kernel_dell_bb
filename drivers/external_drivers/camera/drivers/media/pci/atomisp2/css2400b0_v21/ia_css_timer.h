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

#ifndef __IA_CSS_TIMER_H
#define __IA_CSS_TIMER_H

/** @file
 * Timer interface definitions
 */
#include <type_support.h>		/* for uint32_t */
#include "ia_css_err.h"

/** @brief timer reading definition */
typedef uint32_t clock_value_t;

/** @brief 32 bit clock tick,(measured time in ticks)*/
struct ia_css_clock_tick {
	clock_value_t ticks; /**< measured time in ticks.*/
};

/** @brief  code measurement common struct */
struct ia_css_time_meas {
	clock_value_t	start_timer_value;	/**< measured time in ticks */
	clock_value_t	end_timer_value;	/**< measured time in ticks */
};


/**@brief SIZE_OF_IA_CSS_CLOCK_TICK_STRUCT checks to ensure correct alignment for struct ia_css_clock_tick. */
#define SIZE_OF_IA_CSS_CLOCK_TICK_STRUCT sizeof(clock_value_t)
/** @brief checks to ensure correct alignment for ia_css_time_meas. */
#define SIZE_OF_IA_CSS_TIME_MEAS_STRUCT (sizeof(clock_value_t) \
					+ sizeof(clock_value_t))

/** @brief API to fetch timer count directly
*
* @param curr_ts [out] measured count value
* @return IA_CSS_SUCCESS if success
*
*/
enum ia_css_err
ia_css_timer_get_current_tick(
	struct ia_css_clock_tick *curr_ts);

#endif  /* __IA_CSS_TIMER_H */
