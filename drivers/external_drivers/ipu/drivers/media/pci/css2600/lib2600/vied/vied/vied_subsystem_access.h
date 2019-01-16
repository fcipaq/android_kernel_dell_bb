#ifndef _HRT_VIED_SUBSYSTEM_ACCESS_H
#define _HRT_VIED_SUBSYSTEM_ACCESS_H

#include "type_support.h"
#include "vied_subsystem_access_types.h"

#if !defined(CFG_VIED_SUBSYSTEM_ACCESS_INLINE_IMPL) && \
    !defined(CFG_VIED_SUBSYSTEM_ACCESS_LIB_IMPL)
#error Implementation selection macro for vied subsystem access not defined
#endif

#if defined(CFG_VIED_SUBSYSTEM_ACCESS_INLINE_IMPL)
#define _VIED_SUBSYSTEM_ACCESS_INLINE static inline
#include "vied_subsystem_access_impl.h"
#else
#define _VIED_SUBSYSTEM_ACCESS_INLINE
#endif

_VIED_SUBSYSTEM_ACCESS_INLINE
void vied_subsystem_store_8 (vied_subsystem_t dev, 
                             vied_subsystem_address_t addr, uint8_t  data);

_VIED_SUBSYSTEM_ACCESS_INLINE
void vied_subsystem_store_16(vied_subsystem_t dev, 
                             vied_subsystem_address_t addr, uint16_t data);

_VIED_SUBSYSTEM_ACCESS_INLINE
void vied_subsystem_store_32(vied_subsystem_t dev, 
                             vied_subsystem_address_t addr, uint32_t data);

_VIED_SUBSYSTEM_ACCESS_INLINE
void vied_subsystem_store(vied_subsystem_t dev,
                          vied_subsystem_address_t addr,
                          const void *data, unsigned int size);

_VIED_SUBSYSTEM_ACCESS_INLINE
uint8_t  vied_subsystem_load_8 (vied_subsystem_t dev, 
                                vied_subsystem_address_t addr);

_VIED_SUBSYSTEM_ACCESS_INLINE
uint16_t vied_subsystem_load_16(vied_subsystem_t dev, 
                                vied_subsystem_address_t addr);

_VIED_SUBSYSTEM_ACCESS_INLINE
uint32_t vied_subsystem_load_32(vied_subsystem_t dev, 
                                vied_subsystem_address_t addr);

_VIED_SUBSYSTEM_ACCESS_INLINE
void vied_subsystem_load(vied_subsystem_t dev,
                         vied_subsystem_address_t addr, 
                         void *data, unsigned int size);

#endif /* _HRT_VIED_SUBSYSTEM_ACCESS_H */
