#ifndef _HRT_VIED_TYPES_H
#define _HRT_VIED_TYPES_H

/** Types shared by VIED interfaces */

#include "type_support.h"

/** \brief An address within a VIED subsystem
 *
 * This will eventually replace teh vied_memory_address_t and  vied_subsystem_address_t
 */
typedef uint32_t vied_address_t;

/** \brief Memory address type
 *
 * A memory address is an offset within a memory.
 */  
typedef uint32_t   vied_memory_address_t;

/** \brief Master port id */
typedef int   vied_master_port_id_t;

/**
 * \brief Require the existence of a certain type
 *
 * This macro can be used in interface header files to ensure that
 * an implementation define type with a specified name exists.
 */
#define _VIED_REQUIRE_TYPE(T) enum { _VIED_SIZEOF_##T = sizeof(T) }


#endif /*  _HRT_VIED_TYPES_H */
