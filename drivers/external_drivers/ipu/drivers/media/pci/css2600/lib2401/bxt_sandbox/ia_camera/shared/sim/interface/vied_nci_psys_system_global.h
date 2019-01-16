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

#ifndef __PSYS_SYSTEM_GLOBAL_H_INCLUDED__
#define __PSYS_SYSTEM_GLOBAL_H_INCLUDED__

#include <type_support.h>

/*
 * Key system types
 */
/* Subsystem internal physical address */
#define VIED_ADDRESS_BITS					32

//typedef uint32_t							vied_address_t;

/* Subsystem internal virtual address */
#define VIED_VADDRESS_BITS					32
typedef uint32_t							vied_vaddress_t;
/* Subsystem internal data bus */
#define VIED_DATA_BITS						32
typedef uint32_t							vied_data_t;

#define VIED_NULL							((vied_vaddress_t)0)

typedef enum {
	VIED_NCI_SP0_ID = 0,
	VIED_NCI_SP1_ID,
	VIED_NCI_SP2_ID,
	VIED_NCI_SP3_ID,
	VIED_NCI_VP0_ID,
	VIED_NCI_VP1_ID,
	VIED_NCI_VP2_ID,
	VIED_NCI_VP3_ID,
	VIED_NCI_ACC0_ID,
	VIED_NCI_ACC1_ID,
	VIED_NCI_ACC2_ID,
	VIED_NCI_ACC3_ID,
	VIED_NCI_ACC4_ID,
	VIED_NCI_ACC5_ID,
	VIED_NCI_ACC6_ID,
	VIED_NCI_N_CELL_ID
} vied_nci_cell_ID_t;

typedef enum {
	VIED_NCI_BARRIER0_ID = 0,
	VIED_NCI_BARRIER1_ID,
	VIED_NCI_BARRIER2_ID,
	VIED_NCI_BARRIER3_ID,
	VIED_NCI_BARRIER4_ID,
	VIED_NCI_BARRIER5_ID,
	VIED_NCI_BARRIER6_ID,
	VIED_NCI_BARRIER7_ID,
	VIED_NCI_N_BARRIER_ID
} vied_nci_barrier_ID_t;

typedef enum {
	VIED_NCI_SP_CTRL_TYPE_ID = 0,
	VIED_NCI_SP_SERVER_TYPE_ID,
	VIED_NCI_SP_FLOAT_TYPE_ID,
	VIED_NCI_VP_TYPE_ID,
	VIED_NCI_ACC_PSA_TYPE_ID,
	VIED_NCI_ACC_ISA_TYPE_ID,
	VIED_NCI_ACC_OSA_TYPE_ID,
	VIED_NCI_N_CELL_TYPE_ID
} vied_nci_cell_type_ID_t;

typedef enum {
	VIED_NCI_DEV_CHN_DMA__ID = 0,
	VIED_NCI_DEV_CHN_GDC_ID,
	VIED_NCI_DEV_CHN_ACC_ID,
	VIED_NCI_DEV_CHN_CELL_ID,
	VIED_NCI_N_DEV_CHN_ID
} vied_nci_dev_chn_ID_t;

typedef enum {
	VIED_NCI_VMEM0_ID = 0,
	VIED_NCI_VMEM1_ID,
	VIED_NCI_VMEM2_ID,
	VIED_NCI_VMEM3_ID,
	VIED_NCI_VMEM4_ID,
	VIED_NCI_BAMEM0_ID,
	VIED_NCI_BAMEM1_ID,
	VIED_NCI_BAMEM2_ID,
	VIED_NCI_BAMEM3_ID,
	VIED_NCI_DMEM0_ID,
	VIED_NCI_DMEM1_ID,
	VIED_NCI_DMEM2_ID,
	VIED_NCI_DMEM3_ID,
	VIED_NCI_DMEM4_ID,
	VIED_NCI_DMEM5_ID,
	VIED_NCI_DMEM6_ID,
	VIED_NCI_DMEM7_ID,
	VIED_NCI_PMEM0_ID,
	VIED_NCI_PMEM1_ID,
	VIED_NCI_PMEM2_ID,
	VIED_NCI_PMEM3_ID,
	VIED_NCI_N_MEM_ID
} vied_nci_mem_ID_t;

typedef enum {
	VIED_NCI_GMEM_TYPE_ID = 0,
	VIED_NCI_DMEM_TYPE_ID,
	VIED_NCI_VMEM_TYPE_ID,
	VIED_NCI_BAMEM_TYPE_ID,
	VIED_NCI_PMEM_TYPE_ID,
	VIED_NCI_N_MEM_TYPE_ID
} vied_nci_mem_type_ID_t;

/* Excluding PMEM */
#define VIED_NCI_N_DATA_MEM_TYPE_ID		(VIED_NCI_N_MEM_TYPE_ID - 1)

#define VIED_NCI_N_SP_CTRL_MEM		2
#define VIED_NCI_N_SP_SERVER_MEM	2
#define VIED_NCI_N_SP_FLOAT_MEM		2
#define VIED_NCI_N_VP_MEM			4
#define VIED_NCI_N_ACC_PSA_MEM		0
#define VIED_NCI_N_ACC_ISA_MEM		0
#define VIED_NCI_N_ACC_OSA_MEM		0
#define VIED_NCI_N_CELL_MEM_MAX		4

/*
 * Storage of the resource and resource type enumerators
 */
#define VIED_NCI_RESOURCE_ID_BITS			8
typedef uint8_t								vied_nci_resource_id_t;

#define VIED_NCI_RESOURCE_SIZE_BITS			16
typedef uint16_t							vied_nci_resource_size_t;

#define VIED_NCI_RESOURCE_BITMAP_BITS		32
typedef uint32_t							vied_nci_resource_bitmap_t;

extern vied_nci_resource_bitmap_t vied_nci_bit_mask(
	const unsigned int						index);

extern vied_nci_resource_bitmap_t vied_nci_bitmap_set(
	const vied_nci_resource_bitmap_t		bitmap,
	const vied_nci_resource_bitmap_t		bit_mask);

extern vied_nci_resource_bitmap_t vied_nci_bitmap_clear(
	const vied_nci_resource_bitmap_t		bitmap,
	const vied_nci_resource_bitmap_t		bit_mask);

extern bool vied_nci_is_bitmap_empty(
	const vied_nci_resource_bitmap_t		bitmap);

extern bool vied_nci_is_bitmap_set(
	const vied_nci_resource_bitmap_t		bitmap,
	const vied_nci_resource_bitmap_t		bit_mask);

extern bool vied_nci_is_bitmap_clear(
	const vied_nci_resource_bitmap_t		bitmap,
	const vied_nci_resource_bitmap_t		bit_mask);

extern int vied_nci_bitmap_compute_weight(
	const vied_nci_resource_bitmap_t		bitmap);

extern vied_nci_resource_bitmap_t vied_nci_bitmap_set_unique(
	const vied_nci_resource_bitmap_t		bitmap,
	const vied_nci_resource_bitmap_t		bit_mask);

extern vied_nci_resource_bitmap_t vied_nci_bit_mask_set_unique(
	const vied_nci_resource_bitmap_t		bitmap,
	const unsigned int						index);

extern vied_nci_resource_bitmap_t vied_nci_cell_bit_mask(
	const vied_nci_cell_ID_t		cell_id);

extern vied_nci_resource_bitmap_t vied_nci_barrier_bit_mask(
	const vied_nci_barrier_ID_t		barrier_id);

extern vied_nci_cell_type_ID_t vied_nci_cell_get_type(
	const vied_nci_cell_ID_t		cell_id);

extern vied_nci_mem_type_ID_t vied_nci_mem_get_type(
	const vied_nci_mem_ID_t			mem_id);

extern uint16_t vied_nci_mem_get_size(
	const vied_nci_mem_ID_t			mem_id);

extern bool vied_nci_is_cell_of_type(
	const vied_nci_cell_ID_t		cell_id,
	const vied_nci_cell_type_ID_t	cell_type_id);

extern bool vied_nci_is_mem_of_type(
	const vied_nci_mem_ID_t			mem_id,
	const vied_nci_mem_type_ID_t	mem_type_id);

extern bool vied_nci_is_cell_mem_of_type(
	const vied_nci_cell_ID_t		cell_id,
	const uint16_t					mem_index,
	const vied_nci_mem_type_ID_t	mem_type_id);

extern bool vied_nci_has_cell_mem_of_id(
	const vied_nci_cell_ID_t		cell_id,
	const vied_nci_mem_ID_t			mem_id);

extern uint16_t vied_nci_cell_get_mem_count(
	const vied_nci_cell_ID_t		cell_id);

extern vied_nci_mem_type_ID_t vied_nci_cell_get_mem_type(
	const vied_nci_cell_ID_t		cell_id,
	const uint16_t					mem_index);

extern vied_nci_mem_ID_t vied_nci_cell_get_mem(
	const vied_nci_cell_ID_t		cell_id,
	const uint16_t					mem_index);

extern vied_nci_mem_type_ID_t vied_nci_cell_type_get_mem_type(
	const vied_nci_cell_type_ID_t	cell_type_id,
	const uint16_t					mem_index);

#endif /* __PSYS_SYSTEM_GLOBAL_H_INCLUDED__ */
