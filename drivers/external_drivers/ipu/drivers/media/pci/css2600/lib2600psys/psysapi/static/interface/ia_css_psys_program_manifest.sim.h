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
