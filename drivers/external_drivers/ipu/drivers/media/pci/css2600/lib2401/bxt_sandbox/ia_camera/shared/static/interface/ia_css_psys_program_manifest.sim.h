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

#ifndef __IA_CSS_PSYS_PROGRAM_MANIFEST_SIM_H_INCLUDED__
#define __IA_CSS_PSYS_PROGRAM_MANIFEST_SIM_H_INCLUDED__

/*! \file */

/** @file ia_css_psys_program_manifest.sim.h
 *
 * Define the methods on the program manifest object: Simulation only
 */

#include <ia_css_psys_manifest_types.h>

#include <type_support.h>					/* uint8_t */

/*! Compute the size of storage required for allocating the program manifest object

 @param	program_dependency_count[in]	Number of programs this one depends on
 @param	terminal_dependency_count[in]	Number of terminals this one depends on

 @return 0 on error
 */
extern size_t ia_css_sizeof_program_manifest(
	const uint8_t							program_dependency_count,
	const uint8_t							terminal_dependency_count);

/*! Create (the storage for) the program manifest object

 @param	program_dependency_count[in]	Number of programs this one depends on
 @param	terminal_dependency_count[in]	Number of terminals this one depends on

 @return NULL on error
 */
extern ia_css_program_manifest_t *ia_css_program_manifest_alloc(
	const uint8_t							program_dependency_count,
	const uint8_t							terminal_dependency_count);

/*! Destroy (the storage of) the program manifest object

 @param	manifest[in]			program manifest

 @return NULL
 */
extern ia_css_program_manifest_t *ia_css_program_manifest_free(
	ia_css_program_manifest_t				*manifest);

#endif /* __IA_CSS_PSYS_PROGRAM_MANIFEST_SIM_H_INCLUDED__ */
