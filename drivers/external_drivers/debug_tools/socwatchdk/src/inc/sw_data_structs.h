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

/*
 * Description: file containing internal data structures used by the
 * power driver.
 */

#ifndef __SW_DATA_STRUCTS_H__
#define __SW_DATA_STRUCTS_H__ 1

/*
 * We're the PWR kernel device driver.
 * Flag needs to be set BEFORE
 * including 'pw_ioctl.h'
 */
#define PW_KERNEL_MODULE 1

#include "sw_ioctl.h" // For IOCTL mechanism
#include <linux/fs.h>
#include <linux/bitops.h> // for "test_and_set_bit(...)" atomic functionality


/*
 * Used to indicate whether
 * a new IRQ mapping was
 * created.
 */
typedef enum {
	OK_IRQ_MAPPING_EXISTS,
	OK_NEW_IRQ_MAPPING_CREATED,
	ERROR_IRQ_MAPPING
} irq_mapping_types_t;

typedef enum {
    PW_MAPPING_EXISTS,
    PW_NEW_MAPPING_CREATED,
    PW_MAPPING_ERROR
} pw_mapping_type_t;

/*
 * Structure to hold current CMD state
 * of the device driver. Constantly evolving, but
 * that's OK -- this is internal to the driver
 * and is NOT exported.
 */
typedef struct {
    sw_driver_collection_cmd_t cmd; // indicates which command was specified last e.g. START, STOP etc.
    /*
     * Should we write to our per-cpu output buffers?
     * YES if we're actively collecting.
     * NO if we're not.
     */
    bool write_to_buffers;
    /*
     * Should we "drain/flush" the per-cpu output buffers?
     * (See "device_read" for an explanation)
     */
    bool drain_buffers;
    /*
     * Current methodology for generating kernel-space call
     * stacks relies on following frame pointer: has
     * the kernel been compiled with frame pointers?
     */
    bool have_kernel_frame_pointers;
    /*
     * On some archs, C-state residency MSRS do NOT count at TSC frequency.
     * For these, we need to apply a "clock multiplier". Record that
     * here.
     */
    unsigned int residency_count_multiplier;
    /*
     * Store the bus clock frequency.
     */
    unsigned int bus_clock_freq_khz;
    /*
     * What switches should the device driver collect?
     * Note: different from interface spec:
     * We're moving from bitwise OR to bitwise OR of (1 << switch) values.
     * Use the "POWER_XXX_MASK" masks to set/test switch residency.
     */
    // int collection_switches;
    u64 collection_switches;
    /*
     * Total time elapsed for
     * all collections.
     * Aggregated over all collections -- useful
     * in multiple PAUSE/RESUME scenarios
     */
    unsigned long totalCollectionTime;
    /*
     * Start and stop jiffy values for
     * the current collection.
     */
    unsigned long collectionStartJIFF, collectionStopJIFF;
    /*
     * This is the knob to control the frequency of D-state data sampling
     * to adjust their collection overhead. By default, they are sampled
     * in power_start traceevent after 100 msec is passed from the previous sample.
     */
    u32 d_state_sample_interval;
    /*
     * New MSR storage scheme.
     */
    struct list_head msr_list;
    int num_msrs;
    sw_msr_addr_t *msr_addrs;
    /*
     * Platform residency information.
     */
    struct {
        /*
         * IPC commands for platform residency
         * Valid ONLY if 'collection_type' == 'PW_IO_IPC'
         */
        u32 ipc_start_command, ipc_start_sub_command; // START IPC command, sub-cmd
        u32 ipc_stop_command, ipc_stop_sub_command; // STOP IPC command, sub-cmd
        u32 ipc_dump_command, ipc_dump_sub_command; // DUMP IPC command, sub-cmd
        u16 num_addrs; // The number of addresses encoded in the 'platform_res_addrs', 'platform_remapped_addrs' and 'init_platform_res_values' arrays, below
        u8 collection_type; // One of 'sw_io_type_t'
        u8 counter_size_in_bytes; // Usually either 4 (for 32b counters) or 8 (for 64b counters)
        u64 *platform_res_addrs; // Addresses from which to read the various S0iX values; will be remapped (via 'ioremap_nocache()') into 'platform_remapped_addrs'
        u64 *platform_remapped_addrs; // Required for MMIO-based access; remapped addresses -- use this to do the actual reads
        u64 *init_platform_res_values; // Store the INITIAL values here
        // s_res_msg_t *platform_residency_msg; // Used to send messages back to Ring-3; 'platform_residency_msg->residencies' usually has 'num_addrs+2' entries (+1 for S0i0, +1 for S3)
    };

    // Others...
} internal_state_t;

/*
 * Per-cpu structure holding MSR residency counts,
 * timer-TSC values etc.
 */
typedef struct per_cpu_struct {
	u32 was_timer_hrtimer_softirq; // 4 bytes
	void *sched_timer_addr; // 4/8 bytes (arch dependent)
} per_cpu_t;

/*
 * Per-cpu structure holding wakeup event causes, tscs
 * etc. Set by the first non-{TPS, TPE} event to occur
 * after a processor wakes up.
 */
struct wakeup_event {
    u64 event_tsc; // TSC at which the event occurred
    u64 event_val; // Event value -- domain-specific
    s32 init_cpu; // CPU on which a timer was initialized; valid ONLY for wakeups caused by timers!
    u32 event_type; // one of c_break_type_t enum values
    pid_t event_tid, event_pid;
};

/*
 * Convenience macros for accessing per-cpu residencies
 */
#define RESIDENCY(p,i) ( (p)->residencies[(i)] )
#define PREV_MSR_VAL(p,i) ( (p)->prev_msr_vals[(i)] )

/*
 * Per-cpu structure holding stats information.
 * Eventually, we may want to incorporate these fields within
 * the "per_cpu_t" structure.
 */
typedef struct {
    local_t c_breaks, timer_c_breaks, inters_c_breaks;
    local_t p_trans;
    local_t num_inters, num_timers;
} stats_t;

typedef struct sw_msr_info_set sw_msr_info_set_t;
struct sw_msr_info_set {
    /*
     * Previous values of the various MSRs. Required to enable residency
     * computation.
     */
    sw_msr_val_t *prev_msr_vals;
    /*
     * Which 'Cx' MSRs counted during the previous C-state quantum(s)?
     * (We could have more than one in an HT environment -- it is NOT
     * guaranteed that a core wakeup will cause both threads to wakeup.)
     */
    sw_msr_val_t *curr_msr_count;
    /*
     * Scratch memory required for "C_MULTI_MSG" sample support.
     */
    u8 *c_multi_msg_mem;
    /*
     * The number of MSRs we're currently tracking.
     */
    u32 num_msrs;
    /*
     * What was the last C-state the OS requested?
     */
    u32 prev_req_cstate;
    /*
     * Have we sent the boundary C-state sample?
     * Required for an initial MSR set snapshot.
     */
    u8 init_msr_set_sent;
};

/*
 * Struct to hold old IA32_FIXED_CTR_CTRL and IA32_PERF_GLOBAL_CTRL MSR
 * MSR values to enable restoring after the socwatch driver terminates.
 * These are used to toggle CPU_CLK_UNHALTED.REF counting.
 */
typedef struct {
    u32 fixed_data[2], perf_data[2];
} CTRL_values_t;

/*
 * Helper struct to extract (user-supplied)
 * IOCTL input and output lengths.
 */
typedef struct {
	int in_len, out_len;
	char data[1];
} ioctl_args_stub_t;

#if defined(HAVE_COMPAT_IOCTL) && defined(CONFIG_X86_64)
#include <linux/compat.h>
/*
 * Helper struct for use in translating
 * IOCTLs from 32b user programs in 64b
 * kernels.
 */
typedef struct sw_driver_ioctl_arg32 sw_driver_ioctl_arg32_t;
#pragma pack(push, 1)
struct sw_driver_ioctl_arg32 {
    pw_s32_t in_len;
    pw_s32_t out_len;
    compat_caddr_t in_arg;
    compat_caddr_t out_arg;
};
#pragma pack(pop)
#endif // COMPAT && x64

#endif // __SW_DATA_STRUCTS_H__
