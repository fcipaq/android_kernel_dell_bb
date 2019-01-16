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

#ifndef __IBUF_CTRL_GLOBAL_H_INCLUDED__
#define __IBUF_CTRL_GLOBAL_H_INCLUDED__

#include <type_support.h>

#include <ibuf_cntrl_defs.h>	/* _IBUF_CNTRL_RECALC_WORDS_STATUS,
				 * _IBUF_CNTRL_ARBITERS_STATUS,
				 * _IBUF_CNTRL_PROC_REG_ALIGN,
				 * etc.
				 */

typedef struct ib_buffer_s	ib_buffer_t;
struct	ib_buffer_s {
	uint32_t	start_addr;	/* start address of the buffer in the
					 * "input-buffer hardware block"
					 */

	uint32_t	stride;		/* stride per buffer line (in bytes) */
	uint32_t	lines;		/* lines in the buffer */
};

typedef struct ibuf_ctrl_cfg_s ibuf_ctrl_cfg_t;
struct ibuf_ctrl_cfg_s {

	bool online;

	struct {
		/* DMA configuration */
		uint32_t channel;
		uint32_t cmd; /* must be _DMA_V2_MOVE_A2B_NO_SYNC_CHK_COMMAND */

		/* DMA reconfiguration */
		uint32_t shift_returned_items;
		uint32_t elems_per_word_in_ibuf;
		uint32_t elems_per_word_in_dest;
	} dma_cfg;

	ib_buffer_t ib_buffer;

	struct {
		uint32_t stride;
		uint32_t start_addr;
		uint32_t lines;
	} dest_buf_cfg;

	uint32_t items_per_store;
	uint32_t stores_per_frame;

	struct {
		uint32_t sync_cmd;	/* must be _STREAM2MMIO_CMD_TOKEN_SYNC_FRAME */
		uint32_t store_cmd;	/* must be _STREAM2MMIO_CMD_TOKEN_STORE_PACKETS */
	} stream2mmio_cfg;
};

extern const uint32_t N_IBUF_CTRL_PROCS[N_IBUF_CTRL_ID];

#endif /* __IBUF_CTRL_GLOBAL_H_INCLUDED__ */
