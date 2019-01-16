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
#ifndef _PW_KERNEL_DEFINES_H_
#define _PW_KERNEL_DEFINES_H_ 1

#ifdef __KERNEL__

/* ***************************************************
 * The following is only valid for kernel code.
 * ***************************************************
 */

#define CPU() (raw_smp_processor_id())
#define RAW_CPU() (raw_smp_processor_id())
#define TID() (current->pid)
#define PID() (current->tgid)
#define NAME() (current->comm)
#define PKG(c) ( cpu_data(c).phys_proc_id )
#define IT_REAL_INCR() (current->signal->it_real_incr.tv64)

#define ATOMIC_CAS(ptr, old_val, new_val) ( cmpxchg( (ptr), (old_val), (new_val) ) == (old_val) )

/*
 * Should we measure overheads?
 * '1' ==> YES
 * '0' ==> NO
 */
#define DO_OVERHEAD_MEASUREMENTS 0
/*
 * Should we track memory usage?
 * '1' ==> YES
 * '0' ==> NO
 */
#define DO_TRACK_MEMORY_USAGE 0
/*
 * Are we compiling with driver profiling support
 * turned ON? If YES then force 'DO_OVERHEAD_MEASUREMENTS'
 * and 'DO_TRACK_MEMORY_USAGE' to be TRUE.
 */
#if DO_DRIVER_PROFILING
    #if !DO_OVERHEAD_MEASUREMENTS
        #undef DO_OVERHEAD_MEASUREMENTS
        #define DO_OVERHEAD_MEASUREMENTS 1
    #endif // DO_OVERHEAD_MEASUREMENTS
    #if !DO_TRACK_MEMORY_USAGE
        #undef DO_TRACK_MEMORY_USAGE
        #define DO_TRACK_MEMORY_USAGE 1
    #endif // DO_TRACK_MEMORY_USAGE
#endif // DO_DRIVER_PROFILING
/*
 * Should we allow debug output.
 * Set to: "1" ==> 'OUTPUT' is enabled.
 *         "0" ==> 'OUTPUT' is disabled.
 */
#define DO_DEBUG_OUTPUT 0
/*
 * Control whether to output driver ERROR messages.
 * These are independent of the 'OUTPUT' macro
 * (which controls debug messages).
 * Set to '1' ==> Print driver error messages (to '/var/log/messages')
 *        '0' ==> Do NOT print driver error messages
 */
#define DO_PRINT_DRIVER_ERROR_MESSAGES 1
/*
 * Macros to control output printing.
 */
#if DO_DEBUG_OUTPUT
    #define pw_pr_debug(...) printk(KERN_INFO __VA_ARGS__)
    #define pw_pr_warn(...) printk(KERN_WARNING __VA_ARGS__)
#else
    #define pw_pr_debug(...)
    #define pw_pr_warn(...)
#endif
/*
 * Macro for driver error messages.
 */
#if (DO_PRINT_DRIVER_ERROR_MESSAGES || DO_DEBUG_OUTPUT)
    #define pw_pr_error(...) printk(KERN_ERR __VA_ARGS__)
#else
    #define pw_pr_error(...)
#endif

#endif // __KERNEL__

#endif // _PW_KERNEL_DEFINES_H_
