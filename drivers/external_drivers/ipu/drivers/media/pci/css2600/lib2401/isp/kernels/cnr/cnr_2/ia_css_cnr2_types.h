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

#ifndef __IA_CSS_CNR2_TYPES_H
#define __IA_CSS_CNR2_TYPES_H

/** Chroma Noise Reduction configuration.
 *
 *  Small sensitivity of edge means strong smoothness and NR performance.
 *  If you see blurred color on vertical edges,
 *  set higher values on sense_gain_h*.
 *  If you see blurred color on horizontal edges,
 *  set higher values on sense_gain_v*.
 *
 *  ISP block: CNR2
 * (ISP1: CNR1 is used.)
 * (ISP2: CNR1 is used for Preview/Video.)
 *  ISP2: CNR2 is used for Still.
 */
struct ia_css_cnr_config {
	uint16_t coring_u;	/**< Coring level of U.
				u0.13, [0,8191], default/ineffective 0 */
	uint16_t coring_v;	/**< Coring level of V.
				u0.13, [0,8191], default/ineffective 0 */
	uint16_t sense_gain_vy;	/**< Sensitivity of horizontal edge of Y.
				u13.0, [0,8191], default 100, ineffective 0 */
	uint16_t sense_gain_vu;	/**< Sensitivity of horizontal edge of U.
				u13.0, [0,8191], default 100, ineffective 0 */
	uint16_t sense_gain_vv;	/**< Sensitivity of horizontal edge of V.
				u13.0, [0,8191], default 100, ineffective 0 */
	uint16_t sense_gain_hy;	/**< Sensitivity of vertical edge of Y.
				u13.0, [0,8191], default 50, ineffective 0 */
	uint16_t sense_gain_hu;	/**< Sensitivity of vertical edge of U.
				u13.0, [0,8191], default 50, ineffective 0 */
	uint16_t sense_gain_hv;	/**< Sensitivity of vertical edge of V.
				u13.0, [0,8191], default 50, ineffective 0 */
};

#endif /* __IA_CSS_CNR2_TYPES_H */

