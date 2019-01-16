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
#ifndef __SW_TRACEPOINT_HANDLERS_H__
#define __SW_TRACEPOINT_HANDLERS_H__

enum sw_trace_data_type {
    SW_TRACE_COLLECTOR_TRACEPOINT,
    SW_TRACE_COLLECTOR_NOTIFIER
};

struct sw_trace_notifier_name {
    const char *kernel_name; // The tracepoint name; used by the kernel to identify tracepoints
    const char *abstract_name; // An abstract name used by plugins to specify tracepoints-of-interest; shared with Ring-3
};

typedef struct sw_trace_notifier_data sw_trace_notifier_data_t;
typedef int (*sw_trace_notifier_register_func)(struct sw_trace_notifier_data *node);
typedef int (*sw_trace_notifier_unregister_func)(struct sw_trace_notifier_data *node);

struct sw_trace_notifier_data {
    enum sw_trace_data_type type; // Tracepoint or Notifier
    const struct sw_trace_notifier_name *name; // Tracepoint name(s)
    sw_trace_notifier_register_func probe_register; // probe register function
    sw_trace_notifier_unregister_func probe_unregister; // probe unregister function
    struct tracepoint *tp;
    bool was_registered;
    struct list_head list; // List of 'sw_collector_data' instances for this tracepoint or notifier
};

int sw_extract_tracepoints(void);
int sw_register_trace_notifiers(void);
int sw_unregister_trace_notifiers(void);

void sw_reset_trace_notifier_lists(void);

void sw_print_trace_notifier_overheads(void);

int sw_for_each_tracepoint_node(int (*func)(struct sw_trace_notifier_data *node, void *priv), void *priv, bool return_on_error);
int sw_for_each_notifier_node(int (*func)(struct sw_trace_notifier_data *node, void *priv), void *priv, bool return_on_error);

int sw_get_trace_notifier_id(struct sw_trace_notifier_data *node);

const char *sw_get_trace_notifier_kernel_name(struct sw_trace_notifier_data *node);
const char *sw_get_trace_notifier_abstract_name(struct sw_trace_notifier_data *node);

#endif // __SW_TRACEPOINT_HANDLERS_H__
