/*

  This file is provided under a dual BSD/GPLv2 license.  When using or
  redistributing this file, you may do so under either license.

  GPL LICENSE SUMMARY

  Copyright(c) 2014 - 2015 Intel Corporation.

  This program is free software; you can redistribute it and/or modify
  it under the terms of version 2 of the GNU General Public License as
  published by the Free Software Foundation.

  This program is distributed in the hope that it will be useful, but
  WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  General Public License for more details.

  Contact Information:
  SoC Watch Developer Team <socwatchdevelopers@intel.com>
  Intel Corporation,
  1906 Fox Drive,
  Champaign, IL 61820

  BSD LICENSE

  Copyright(c) 2014 - 2015 Intel Corporation.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions
  are met:

    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in
      the documentation and/or other materials provided with the
      distribution.
    * Neither the name of Intel Corporation nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/


#include <asm/local.h>
#include <linux/sched.h>
#include <linux/slab.h>

#include <linux/mm.h> // for "remap_pfn_range"
#include <asm/io.h> // for "virt_to_phys"
#include <asm/uaccess.h> // for "copy_to_user"

#include <linux/hardirq.h> // for "in_atomic"

#include "sw_structs.h"
#include "sw_output_buffer.h"
#include "sw_defines.h"
#include "sw_mem.h"
#include "sw_lock_defs.h"
#include "sw_overhead_measurements.h"

/* -------------------------------------------------
 * Compile time constants and macros.
 * -------------------------------------------------
 */
#define NUM_SEGS_PER_BUFFER 2 /* MUST be pow 2! */
#define NUM_SEGS_PER_BUFFER_MASK (NUM_SEGS_PER_BUFFER - 1)
/*
 * The size of the 'buffer' data array in each segment.
 */
#define SW_SEG_DATA_SIZE (sw_buffer_alloc_size)
/*
 * Min size of per-cpu output buffers.
 */
#define SW_MIN_SEG_SIZE_BYTES (1 << 10) /* 1kB */
#define SW_MIN_OUTPUT_BUFFER_SIZE (SW_MIN_SEG_SIZE_BYTES * NUM_SEGS_PER_BUFFER)
/*
 * A symbolic constant for an empty buffer index.
 */
#define EMPTY_SEG (-1)
/*
 * How much space is available in a given segment?
 */
#define EMPTY_TSC ((u64)-1)
#define SEG_IS_FULL(seg) ({bool __full = false; \
        smp_mb(); \
        __full = ((seg)->is_full != EMPTY_TSC); \
        __full;})
#define SEG_SET_FULL(seg, tsc) do { \
    (seg)->is_full = (tsc); \
    smp_mb(); \
} while(0)
#define SEG_SET_EMPTY(seg) do { \
    barrier(); \
    (seg)->bytes_written = 0; \
    SEG_SET_FULL(seg, EMPTY_TSC); \
    /*smp_mb(); */ \
} while(0)
#define SPACE_AVAIL(seg) (SW_SEG_DATA_SIZE - (seg)->bytes_written )
#define SEG_IS_EMPTY(seg) (SPACE_AVAIL(seg) == SW_SEG_DATA_SIZE)

#define GET_OUTPUT_BUFFER(cpu) &per_cpu_output_buffers[(cpu)]
#define GET_POLLED_CPU() (sw_max_num_cpus)
/*
 * Convenience macro: iterate over each segment in a per-cpu output buffer.
 */
#define for_each_segment(i) for (i=0; i<NUM_SEGS_PER_BUFFER; ++i)
#define for_each_seg(buffer, seg) for (int i=0; i<NUM_SEGS_PER_BUFFER && (seg=(buffer)->segments[i]); ++i)
/*
 * How many buffers are we using?
 */
#define GET_NUM_OUTPUT_BUFFERS() (sw_max_num_cpus + 1)
/*
 * Convenience macro: iterate over each per-cpu output buffer.
 */
#define for_each_output_buffer(i) for (i=0; i<GET_NUM_OUTPUT_BUFFERS(); ++i)

/* -------------------------------------------------
 * Local data structures.
 * -------------------------------------------------
 */
typedef struct sw_data_buffer sw_data_buffer_t;
typedef struct sw_output_buffer sw_output_buffer_t;
struct sw_data_buffer {
    u64 is_full;
    u32 bytes_written;
    char *buffer;
} __attribute__((packed));
#define SW_SEG_HEADER_SIZE() (sizeof(struct sw_data_buffer) - sizeof(char *))

struct sw_output_buffer {
    sw_data_buffer_t buffers[NUM_SEGS_PER_BUFFER];
    int buff_index;
    u32 produced_samples;
    u32 dropped_samples;
    int last_seg_read;
    unsigned long free_pages;
    unsigned long mem_alloc_size;
} ____cacheline_aligned_in_smp;


/* -------------------------------------------------
 * Variable definitions.
 * -------------------------------------------------
 */
u64 sw_num_samples_produced = 0, sw_num_samples_dropped = 0;
int sw_max_num_cpus = -1;

DECLARE_OVERHEAD_VARS(sw_produce_generic_msg_i);
/*
 * The alarm queue.
 */
wait_queue_head_t sw_reader_queue;
/*
 * Per-cpu output buffers.
 */
sw_output_buffer_t *per_cpu_output_buffers = NULL;
/*
 * Variables for book keeping.
 */
volatile int sw_last_cpu_read = -1;
volatile s32 sw_last_mask = -1;
/*
 * Lock for the polled buffer.
 */
static DEFINE_SPINLOCK(sw_polled_lock);
/*
 * Buffer allocation size.
 */
unsigned long sw_buffer_alloc_size = (1 << 16); // 64 KB

/* -------------------------------------------------
 * Function definitions.
 * -------------------------------------------------
 */
static inline u64 tscval(void)
{
    u64 tsc = 0;
    rdmsrl(0x10, tsc);

    return tsc;
};


static char *reserve_seg_space_i(size_t size, int cpu, bool *should_wakeup, u64 *reservation_tsc)
{
    sw_output_buffer_t *buffer = GET_OUTPUT_BUFFER(cpu);
    int i=0;
    int buff_index = buffer->buff_index;
    char *dst = NULL;

    if (buff_index < 0 || buff_index >= NUM_SEGS_PER_BUFFER) {
        goto prod_seg_done;
    }
    for_each_segment(i) {
        sw_data_buffer_t *seg = &buffer->buffers[buff_index];
        if (SEG_IS_FULL(seg) == false) {
            if (SPACE_AVAIL(seg) >= size) {
                *reservation_tsc = tscval();
                dst = &seg->buffer[seg->bytes_written];
                seg->bytes_written += size;
                smp_mb();
                buffer->buff_index = buff_index;
                buffer->produced_samples++;
                goto prod_seg_done;
            }
            SEG_SET_FULL(seg, tscval());
        }
        buff_index = CIRCULAR_INC(buff_index, NUM_SEGS_PER_BUFFER_MASK);
        *should_wakeup = true;
    }
prod_seg_done:
    if (!dst) {
        buffer->dropped_samples++;
    }
    return dst;
};


int producePolledMsg(struct sw_driver_msg *msg, bool allow_wakeup)
{
    int cpu = GET_POLLED_CPU();
    bool should_wakeup = false;
    int retVal = PW_SUCCESS;

    if (!msg) {
        return -PW_ERROR;
    }
    pw_pr_debug("POLLED! cpu = %d\n", cpu);
    LOCK(sw_polled_lock);
    {
        size_t size = SW_DRIVER_MSG_HEADER_SIZE() + msg->payload_len;
        char *dst = reserve_seg_space_i(size, cpu, &should_wakeup, &msg->tsc);
        if (dst) {
            /*
             * Assign a special CPU number to this CPU.
             * This is OK, because messages enqueued in this buffer
             * are always CPU agnostic (otherwise they would
             * be invoked from within a preempt_disable()d context
             * in 'sw_handle_collector_node_i()', which ensures they
             * will be enqueued within the 'sw_produce_generic_msg_on_cpu()'
             * function).
             */
            msg->cpuidx = cpu;
            memcpy(dst, msg, SW_DRIVER_MSG_HEADER_SIZE()); dst += SW_DRIVER_MSG_HEADER_SIZE();
            memcpy(dst, msg->p_payload, msg->payload_len);
        } else {
            pw_pr_debug("NO space in polled msg!\n");
            retVal = -PW_ERROR;
        }
    }
    UNLOCK(sw_polled_lock);
    if (unlikely(should_wakeup && allow_wakeup && waitqueue_active(&sw_reader_queue))) {
        wake_up_interruptible(&sw_reader_queue);
    }
    return retVal;
};

int sw_produce_generic_msg_i(struct sw_driver_msg *msg, bool allow_wakeup)
{
    int retval = PW_SUCCESS;
    bool should_wakeup = false;
    int cpu = -1;
    unsigned long flags = 0;

    if (!msg) {
        pw_pr_error("ERROR: CANNOT produce a NULL msg!\n");
        return -PW_ERROR;
    }

#ifdef CONFIG_PREEMPT_COUNT
    if (!in_atomic()) {
        return producePolledMsg(msg, allow_wakeup);
    }
#endif

    local_irq_save(flags);
    cpu = get_cpu();
    {

        size_t size = msg->payload_len + SW_DRIVER_MSG_HEADER_SIZE();
        char *dst = reserve_seg_space_i(size, cpu, &should_wakeup, &msg->tsc);
        if (likely(dst)) {
            memcpy(dst, msg, SW_DRIVER_MSG_HEADER_SIZE()); dst += SW_DRIVER_MSG_HEADER_SIZE();
            memcpy(dst, msg->p_payload, msg->payload_len);
        } else {
            retval = -PW_ERROR;
        }
    }
    put_cpu();
    local_irq_restore(flags);


    if (unlikely(should_wakeup && allow_wakeup && waitqueue_active(&sw_reader_queue))) {
        wake_up_interruptible(&sw_reader_queue);
    }

    return retval;
};

int sw_produce_generic_msg(struct sw_driver_msg *msg, bool allow_wakeup)
{
    return DO_PER_CPU_OVERHEAD_FUNC_RET(int, sw_produce_generic_msg_i, msg, allow_wakeup);
};

int sw_init_per_cpu_buffers_i(unsigned long per_cpu_mem_size)
{
    int cpu = -1;
    per_cpu_output_buffers = (sw_output_buffer_t *)sw_kmalloc(sizeof(sw_output_buffer_t) * GET_NUM_OUTPUT_BUFFERS(), GFP_KERNEL | __GFP_ZERO);
    if (per_cpu_output_buffers == NULL) {
        pw_pr_error("ERROR allocating space for per-cpu output buffers!\n");
        sw_destroy_per_cpu_buffers();
        return -PW_ERROR;
    }
    for_each_output_buffer(cpu) {
        sw_output_buffer_t *buffer = &per_cpu_output_buffers[cpu];
        char *buff = NULL;
        int i=0;
        buffer->mem_alloc_size = per_cpu_mem_size;
        buffer->free_pages = __get_free_pages(GFP_KERNEL | __GFP_ZERO, get_order(per_cpu_mem_size));
        if (buffer->free_pages == 0) {
            pw_pr_error("ERROR allocating pages for buffer [%d]!\n", cpu);
            sw_destroy_per_cpu_buffers();
            return -PW_ERROR;
        }
        buff = (char *)buffer->free_pages;
        for_each_segment(i) {
            buffer->buffers[i].buffer = (char *)buff;
            buff += SW_SEG_DATA_SIZE;
        }
    }
    pw_pr_debug("PER_CPU_MEM_SIZE = %lu, order = %u\n", per_cpu_mem_size, get_order(per_cpu_mem_size));
    return PW_SUCCESS;
};

int sw_init_per_cpu_buffers(void)
{
    unsigned long per_cpu_mem_size = sw_get_output_buffer_size();

    pw_pr_debug("Buffer alloc size = %ld\n", sw_buffer_alloc_size);

    if (GET_NUM_OUTPUT_BUFFERS() <= 0) {
        pw_pr_error("ERROR: max # output buffers= %d\n", GET_NUM_OUTPUT_BUFFERS());
        return -PW_ERROR;
    }

    pw_pr_debug("DEBUG: sw_max_num_cpus = %d, num output buffers = %d\n", sw_max_num_cpus, GET_NUM_OUTPUT_BUFFERS());

    /*
     * Try to allocate per-cpu buffers. If allocation fails, decrease buffer size and retry.
     * Stop trying if size drops below 2KB (which means 1KB for each buffer).
     */
    while (per_cpu_mem_size >= SW_MIN_OUTPUT_BUFFER_SIZE && sw_init_per_cpu_buffers_i(per_cpu_mem_size)) {
        printk(KERN_INFO "WARNING: couldn't allocate per-cpu buffers with size %lu -- trying smaller size!\n", per_cpu_mem_size);
        sw_buffer_alloc_size >>= 1;
        per_cpu_mem_size = sw_get_output_buffer_size();
    }

    if (unlikely(per_cpu_output_buffers == NULL)) {
        printk(KERN_ERR "ERROR: couldn't allocate space for per-cpu output buffers!\n");
        return -PW_ERROR;
    }

    pw_pr_debug("OK, allocated per-cpu buffers with size = %lu\n", per_cpu_mem_size);

    init_waitqueue_head(&sw_reader_queue);

    return PW_SUCCESS;
};

void sw_destroy_per_cpu_buffers(void)
{
    int cpu = -1;

    if (per_cpu_output_buffers != NULL) {
        for_each_output_buffer(cpu) {
            sw_output_buffer_t *buffer = &per_cpu_output_buffers[cpu];
            if (buffer->free_pages != 0) {
                free_pages(buffer->free_pages, get_order(buffer->mem_alloc_size));
                buffer->free_pages = 0;
            }
        }
        sw_kfree(per_cpu_output_buffers);
        per_cpu_output_buffers = NULL;
    }
};

void sw_reset_per_cpu_buffers(void)
{
    int cpu = 0, i=0;
    for_each_output_buffer(cpu) {
        sw_output_buffer_t *buffer = GET_OUTPUT_BUFFER(cpu);
        buffer->buff_index = buffer->dropped_samples = buffer->produced_samples = 0;
        buffer->last_seg_read = -1;

        for_each_segment(i) {
            sw_data_buffer_t *seg = &buffer->buffers[i];
            memset(seg->buffer, 0, SW_SEG_DATA_SIZE);
            SEG_SET_EMPTY(seg);
        }
    }
    sw_last_cpu_read = -1;
    sw_last_mask = -1;
    pw_pr_debug("OK, reset per-cpu output buffers!\n");
};

int sw_map_per_cpu_buffers(struct vm_area_struct *vma, unsigned long *total_size)
{
    int cpu = -1;
    unsigned long start = vma->vm_start;
    /*
     * We have a number of output buffers. Each (per-cpu) output buffer is one contiguous memory
     * area, but the individual buffers are disjoint from each other. We therefore need to
     * loop over each buffer and map in its memory area.
     */
    for_each_output_buffer(cpu) {
        sw_output_buffer_t *buffer = &per_cpu_output_buffers[cpu];
        unsigned long buff_size = buffer->mem_alloc_size;
        int ret = remap_pfn_range(vma, start, virt_to_phys((void *)buffer->free_pages) >> PAGE_SHIFT, buff_size, vma->vm_page_prot);
        if (ret < 0) {
            return ret;
        }
        *total_size += buff_size;
        start += buff_size;
    }

    return PW_SUCCESS;
};

bool sw_any_seg_full(u32 *val, const bool *is_flush_mode)
{
    int num_visited = 0, i = 0;

    if (!val || !is_flush_mode) {
        pw_pr_error("ERROR: NULL ptrs in sw_any_seg_full!\n");
        return false;
    }

    *val = SW_NO_DATA_AVAIL_MASK;
    pw_pr_debug(KERN_INFO "Checking for full seg: val = %u, flush = %s\n", *val, GET_BOOL_STRING(*is_flush_mode));
    for_each_output_buffer(num_visited) {
        int min_seg = EMPTY_SEG, non_empty_seg = EMPTY_SEG;
        u64 min_tsc = EMPTY_TSC;
        sw_output_buffer_t *buffer = NULL;
        if (++sw_last_cpu_read >= GET_NUM_OUTPUT_BUFFERS()) {
            sw_last_cpu_read = 0;
        }
        buffer = GET_OUTPUT_BUFFER(sw_last_cpu_read);
        for_each_segment(i) {
            sw_data_buffer_t *seg = &buffer->buffers[i];
            u64 seg_tsc = seg->is_full;
            if (SEG_IS_EMPTY(seg)) {
                continue;
            }
            non_empty_seg = i;
            if (seg_tsc < min_tsc) {
                /*
                 * Can only happen if seg was full, provided 'EMPTY_TSC' is set to "(u64)-1"
                 */
                min_tsc = seg_tsc;
                min_seg = i;
            }
        }
        if (min_seg != EMPTY_SEG) {
            *val = (sw_last_cpu_read & 0xffff) << 16 | (min_seg & 0xffff);
            return true;
        } else if (*is_flush_mode && non_empty_seg != EMPTY_SEG) {
            *val = (sw_last_cpu_read & 0xffff) << 16 | (non_empty_seg & 0xffff);
            return true;
        }
    }
    /*
     * Reaches here only if there's no data to be read.
     */
    if (*is_flush_mode) {
        /*
         * We've drained all buffers and need to tell the userspace application there
         * isn't any data. Unfortunately, we can't just return a 'zero' value for the
         * mask (because that could also indicate that segment # 0 of cpu #0 has data).
         */
        *val = SW_ALL_WRITES_DONE_MASK;
        return true;
    }
    return false;
};

/*
 * Has semantics of 'copy_to_user()' -- returns # of bytes that could NOT be copied
 * (On success ==> returns 0).
 */
unsigned long sw_consume_data(u32 mask, char __user *buffer, size_t bytes_to_read, size_t *bytes_read)
{
    int which_cpu = -1, which_seg = -1;
    unsigned long bytes_not_copied = 0;
    sw_output_buffer_t *buff = NULL;
    sw_data_buffer_t *seg = NULL;

    if (!buffer || !bytes_read) {
        pw_pr_error("ERROR: NULL ptrs in sw_consume_data!\n");
        return -PW_ERROR;
    }

    if (bytes_to_read < SW_SEG_DATA_SIZE) {
        pw_pr_error("Error: bytes_to_read = %u, required to be at least %lu\n", (unsigned)bytes_to_read, (unsigned long)SW_SEG_DATA_SIZE);
        return bytes_to_read;
    }
    which_cpu = mask >> 16; which_seg = mask & 0xffff;
    pw_pr_debug("CONSUME: cpu = %d, seg = %d\n", which_cpu, which_seg);
    if (which_seg >= NUM_SEGS_PER_BUFFER) {
        pw_pr_error("Error: which_seg (%d) >= NUM_SEGS_PER_BUFFER (%d)\n", which_seg, NUM_SEGS_PER_BUFFER);
        return bytes_to_read;
    }
    /*
     * OK to access unlocked; either the segment is FULL, or no collection
     * is ongoing. In either case, we're GUARANTEED no producer is touching
     * this segment.
     */
    buff = GET_OUTPUT_BUFFER(which_cpu);
    seg = &buff->buffers[which_seg];
    bytes_not_copied = copy_to_user(buffer, seg->buffer, seg->bytes_written); // dst,src
    if (likely(bytes_not_copied == 0)) {
      *bytes_read = seg->bytes_written;
    } else {
        pw_pr_warn("Warning: couldn't copy %lu bytes\n", bytes_not_copied);
    }
    SEG_SET_EMPTY(seg);
    return bytes_not_copied;
};

unsigned long sw_get_output_buffer_size(void)
{
    return (sw_buffer_alloc_size * NUM_SEGS_PER_BUFFER);
};

void sw_count_samples_produced_dropped(void)
{
    int cpu = 0;
    sw_num_samples_produced = sw_num_samples_dropped = 0;
    if (per_cpu_output_buffers == NULL) {
        return;
    }
    for_each_output_buffer(cpu) {
        sw_output_buffer_t *buff = GET_OUTPUT_BUFFER(cpu);
        sw_num_samples_dropped += buff->dropped_samples;
        sw_num_samples_produced += buff->produced_samples;
    }
#if DO_OVERHEAD_MEASUREMENTS
    printk(KERN_INFO "DEBUG: there were %llu samples produced and %llu samples dropped in buffer v5!\n", sw_num_samples_produced, sw_num_samples_dropped);
#endif // DO_OVERHEAD_MEASUREMENTS
};

void sw_print_output_buffer_overheads(void)
{
    PRINT_CUMULATIVE_OVERHEAD_PARAMS(sw_produce_generic_msg_i, "PRODUCE_GENERIC_MSG");
};
