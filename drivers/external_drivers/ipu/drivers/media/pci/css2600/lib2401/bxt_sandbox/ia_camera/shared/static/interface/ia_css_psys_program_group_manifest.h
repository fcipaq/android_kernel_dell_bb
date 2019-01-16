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

#ifndef __IA_CSS_PSYS_PROGRAM_GROUP_MANIFEST_H_INCLUDED__
#define __IA_CSS_PSYS_PROGRAM_GROUP_MANIFEST_H_INCLUDED__

/*! \file */

/** @file ia_css_psys_program_group_manifest.h
 *
 * Define the methods on the program group manifest object that are not part of a single interface
 */

#include <ia_css_psys_manifest_types.h>

#include <type_support.h>						/* uint8_t */

#include <ia_css_psys_program_group_manifest.sim.h>

#include <ia_css_psys_program_group_manifest.hsys.user.h>

#include <ia_css_kernel_bitmap.h>				/* ia_css_kernel_bitmap_t */

/*! Get the stored size of the program group manifest object

 @param	manifest[in]			program group manifest object

 @return size, 0 on error
 */
extern size_t ia_css_program_group_manifest_get_size(
	const ia_css_program_group_manifest_t	*manifest);

/*! Get the program group ID of the program group manifest object

 @param	manifest[in]			program group manifest object

 @return program group ID, 0 on error
 */
extern ia_css_program_group_ID_t ia_css_program_group_manifest_get_program_group_ID(
	const ia_css_program_group_manifest_t	*manifest);

/*! Set the program group ID of the program group manifest object

 @param	manifest[in]			program group manifest object

 @param program group ID

 @return 0 on success, -1 on error
 */
extern int ia_css_program_group_manifest_set_program_group_ID(
	ia_css_program_group_manifest_t	*manifest,
	ia_css_program_group_ID_t id);

/*! Get the storage alignment constraint of the program group binary data

 @param	manifest[in]			program group manifest object

 @return alignment, 0 on error
 */
extern uint8_t ia_css_program_group_manifest_get_alignment(
	const ia_css_program_group_manifest_t	*manifest);

/*! Set the storage alignment constraint of the program group binary data

 @param	manifest[in]			program group manifest object
 @param	alignment[in]			alignment desired

 @return non zero value on error
 */
extern int ia_css_program_group_manifest_set_alignment(
	ia_css_program_group_manifest_t	*manifest,
	const uint8_t alignment);

/*! Get the kernel enable bitmap of the program group

 @param	manifest[in]			program group manifest object

 @return bitmap, 0 on error
 */
extern ia_css_kernel_bitmap_t ia_css_program_group_manifest_get_kernel_bitmap(
	const ia_css_program_group_manifest_t	*manifest);

/*! Set the kernel enable bitmap of the program group

 @param	manifest[in]			program group manifest object
 @param	kernel bitmap[in]		kernel enable bitmap

 @return non-zero on error
 */
extern int ia_css_program_group_manifest_set_kernel_bitmap(
	ia_css_program_group_manifest_t	*manifest,
	const ia_css_kernel_bitmap_t	bitmap);

/*! Get the number of programs in the program group manifest object

 @param	manifest[in]			program group manifest object

 @return program count, 0 on error
 */
extern uint8_t ia_css_program_group_manifest_get_program_count(
	const ia_css_program_group_manifest_t	*manifest);

/*! Get the number of terminals in the program group manifest object

 @param	manifest[in]			program group manifest object

 @return terminal count, 0 on error
 */
extern uint8_t ia_css_program_group_manifest_get_terminal_count(
	const ia_css_program_group_manifest_t	*manifest);

/*! Get the (pointer to) indexed program manifest in the program group manifest object

 @param	manifest[in]			program group manifest object
 @param	program_index[in]		index of the program manifest object

 @return program manifest, NULL on error
 */
extern ia_css_program_manifest_t *ia_css_program_group_manifest_get_program_manifest(
	const ia_css_program_group_manifest_t	*manifest,
	const unsigned int						program_index);

/*! Get the (pointer to) indexed terminal manifest in the program group manifest object

 @param	manifest[in]			program group manifest object
 @param	program_index[in]		index of the terminal manifest object

 @return terminal manifest, NULL on error
 */
extern ia_css_terminal_manifest_t *ia_css_program_group_manifest_get_terminal_manifest(
	const ia_css_program_group_manifest_t	*manifest,
	const unsigned int						terminal_index);

/*! Get the (pointer to) indexed data terminal manifest in the program group manifest object

 @param	manifest[in]			program group manifest object
 @param	program_index[in]		index of the terminal manifest object

 @return data terminal manifest, NULL on error
 */
extern ia_css_data_terminal_manifest_t *ia_css_program_group_manifest_get_data_terminal_manifest(
	const ia_css_program_group_manifest_t	*manifest,
	const unsigned int						terminal_index);

/*! Get the (pointer to) indexed parameter terminal manifest in the program group manifest object

 @param	manifest[in]			program group manifest object
 @param	program_index[in]		index of the terminal manifest object

 @return parameter terminal manifest, NULL on error
 */
extern ia_css_param_terminal_manifest_t *ia_css_program_group_manifest_get_param_terminal_manifest(
	const ia_css_program_group_manifest_t	*manifest,
	const unsigned int						terminal_index);

/*!	initialize program group manifest

 @param	manifest[in]			program group manifest object
 @param	program_count[in]		number of programs.
 @param	terminal_count[in]		number of terminals.
 @param	program_deps[in]		program dependencies for programs in pg.
 @param	terminal_deps[in]		terminal dependencies for programs in pg.
 @param	terminal_type[in]		array of terminal types, binary specific
					static frame data
@param  section_count[in]		section count(cached param terminal's sections).
					i.e., number of kernel parameter sections.
 @return none;
 */
extern void ia_css_program_group_manifest_init(
	ia_css_program_group_manifest_t *blob,
	const uint8_t	program_count,
	const uint8_t	terminal_count,
	const uint8_t   *program_dependencies,
	const uint8_t   *terminal_dependencies,
	const ia_css_terminal_type_t	*terminal_type,
	const uint16_t	section_count);
#endif /* __IA_CSS_PSYS_PROGRAM_GROUP_MANIFEST_H_INCLUDED__ */



