#ifndef __IA_CSS_PSYS_PROGRAM_MANIFEST_H_INCLUDED__
#define __IA_CSS_PSYS_PROGRAM_MANIFEST_H_INCLUDED__

/*! \file */

/** @file ia_css_psys_program_manifest.h
 *
 * Define the methods on the program manifest object that are not part of a single interface
 */

#include <ia_css_psys_manifest_types.h>

#include <type_support.h>						/* uint8_t */

#include <ia_css_psys_program_manifest.sim.h>

#include <ia_css_psys_program_manifest.hsys.user.h>

#include <ia_css_kernel_bitmap.h>				/* ia_css_kernel_bitmap_t */

/*
 * Resources needs
 */
#include <ia_css_psys_program_manifest.hsys.kernel.h>

/*! Check if the program manifest object specifies a fixed cell allocation

 @param	manifest[in]			program manifest object

 @return has_fixed_cell, false on error
 */
extern bool ia_css_has_program_manifest_fixed_cell(
	const ia_css_program_manifest_t			*manifest);

/*! Get the stored size of the program manifest object

 @param	manifest[in]			program manifest object

 @return size, 0 on error
 */
extern size_t ia_css_program_manifest_get_size(
	const ia_css_program_manifest_t			*manifest);

/*! Get the program ID of the program manifest object

 @param	manifest[in]			program manifest object

 @return program ID, 0 on error
 */
extern ia_css_program_ID_t ia_css_program_manifest_get_program_ID(
	const ia_css_program_manifest_t			*manifest);

/*! Set the program ID of the program manifest object

 @param	manifest[in]			program manifest object

 @param program ID

 @return 0 on success, -1 on error
 */
extern int ia_css_program_manifest_set_program_ID(
	ia_css_program_manifest_t			*manifest,
	ia_css_program_ID_t id);

/*! Get the (pointer to) the program group manifest parent of the program manifest object

 @param	manifest[in]			program manifest object

 @return the pointer to the parent, NULL on error
 */
extern ia_css_program_group_manifest_t *ia_css_program_manifest_get_parent(
	const ia_css_program_manifest_t			*manifest);

/*! Set the (pointer to) the program group manifest parent of the program manifest object

 @param	manifest[in]			program manifest object
 @param	program_offset[in]		this program's offset from program_group_manifest's base address.

 @return < 0 on error
 */
extern int ia_css_program_manifest_set_parent_offset(
	ia_css_program_manifest_t				*manifest,
	int32_t program_offset);

/*! Get the type of the program manifest object

 @param	manifest[in]			program manifest object

 @return program type, limit value on error
 */
extern ia_css_program_type_t ia_css_program_manifest_get_type(
	const ia_css_program_manifest_t			*manifest);

/*! Set the type of the program manifest object

 @param	manifest[in]			program manifest object
 @param	program_type[in]		program type

 @return < 0 on error
 */
extern int ia_css_program_manifest_set_type(
	ia_css_program_manifest_t				*manifest,
	const ia_css_program_type_t				program_type);

/*! Get the kernel composition of the program manifest object

 @param	manifest[in]			program manifest object

 @return bitmap, 0 on error
 */
extern ia_css_kernel_bitmap_t ia_css_program_manifest_get_kernel_bitmap(
	const ia_css_program_manifest_t			*manifest);

/*! Set the kernel dependency of the program manifest object

 @param	manifest[in]			program manifest object
 @param	kernel_bitmap[in]		kernel composition bitmap

 @return < 0 on error
 */
extern int ia_css_program_manifest_set_kernel_bitmap(
	ia_css_program_manifest_t				*manifest,
	const ia_css_kernel_bitmap_t			kernel_bitmap);

/*! Get the number of programs this programs depends on from the program group manifest object

 @param	manifest[in]			program manifest object

 @return program dependency count
 */
extern uint8_t ia_css_program_manifest_get_program_dependency_count(
	const ia_css_program_manifest_t			*manifest);

/*! Get the index of the program which the programs at this index depends on
    from the program manifest object

 @param	manifest[in]			program manifest object

 @return program dependency
 */
extern uint8_t ia_css_program_manifest_get_program_dependency(
	const ia_css_program_manifest_t			*manifest,
	const unsigned int						index);

/*! Set the index of the program which the programs at this index depends on
    in the program manifest object

 @param	manifest[in]			program manifest object

 @return program dependency
 */
extern int ia_css_program_manifest_set_program_dependency(
	ia_css_program_manifest_t				*manifest,
	const uint8_t							program_dependency,
	const unsigned int						index);

/*! Get the number of terminals this programs depends on from the program group manifest object

 @param	manifest[in]			program manifest object

 @return program dependency count
 */
extern uint8_t ia_css_program_manifest_get_terminal_dependency_count(
	const ia_css_program_manifest_t			*manifest);

/*! Get the index of the terminal which the programs at this index depends on
    from the program manifest object

 @param	manifest[in]			program manifest object

 @return terminal dependency
 */
uint8_t ia_css_program_manifest_get_terminal_dependency(
	const ia_css_program_manifest_t			*manifest,
	const unsigned int						index);

/*! Set the index of the terminal which the programs at this index depends on
    in the program manifest object

 @param	manifest[in]			program manifest object

 @return program dependency
 */
extern int ia_css_program_manifest_set_terminal_dependency(
	ia_css_program_manifest_t				*manifest,
	const uint8_t							terminal_dependency,
	const unsigned int						index);

/*! Check if the program manifest object specifies a subnode program

 @param	manifest[in]			program manifest object

 @return is_subnode, false on error
 */
extern bool ia_css_is_program_manifest_subnode_program_type(
	const ia_css_program_manifest_t			*manifest);

/*! Check if the program manifest object specifies a supernode program

 @param	manifest[in]			program manifest object

 @return is_supernode, false on error
 */
extern bool ia_css_is_program_manifest_supernode_program_type(
	const ia_css_program_manifest_t			*manifest);
/*! Check if the program manifest object specifies a singular program

 @param	manifest[in]			program manifest object

 @return is_singular, false on error
 */
extern bool ia_css_is_program_manifest_singular_program_type(
	const ia_css_program_manifest_t			*manifest);

#endif /* __IA_CSS_PSYS_PROGRAM_MANIFEST_H_INCLUDED__ */
