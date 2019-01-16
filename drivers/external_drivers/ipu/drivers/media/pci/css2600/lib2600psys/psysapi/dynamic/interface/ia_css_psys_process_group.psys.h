#ifndef __IA_CSS_PSYS_PROCESS_GROUP_PSYS_H_INCLUDED__
#define __IA_CSS_PSYS_PROCESS_GROUP_PSYS_H_INCLUDED__

/*! \file */

/** @file ia_css_psys_process_group.psys.h
 *
 * Define the methods on the process group object: Psys embedded interface
 */

#include <ia_css_psys_process_types.h>

/*
 * Dispatcher
 */

/*! Perform the run command on the process group

 @param	process_group[in]		process group object

 Note: Run indicates that the process group will execute

 Precondition: The process group must be started or
 suspended and the processes have acquired the necessary
 internal resources

 @return < 0 on error
 */
extern int ia_css_process_group_run(
	ia_css_process_group_t					*process_group);

/*! Perform the stop command on the process group

 @param	process_group[in]		process group object

 Note: Stop indicates that the process group has completed execution

 Postcondition: The external resoruces can now be detached

 @return < 0 on error
 */
extern int ia_css_process_group_stop(
	ia_css_process_group_t					*process_group);


#endif /* __IA_CSS_PSYS_PROCESS_GROUP_PSYS_H_INCLUDED__ */
