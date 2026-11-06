/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef __RTLMQ_TRACE_H__
#define __RTLMQ_TRACE_H__

#include "rtlmq_base.h"

struct sk_buff;
struct rtlmq_irq_source;
struct rtlmq_vector;
struct rtlmq_vector_tx_poll;
struct rtlmq_tx_ring;

/*
 * Private tracepoints are debug/KUnit-only. Release macros do not evaluate
 * arguments. Production service events remain available in every build.
 */

#endif /* __RTLMQ_TRACE_H__ */

/*
 * define_trace.h rereads this file for each TRACE_SYSTEM. Select only that
 * system during each recursive pass; ordinary types and interfaces above
 * retain their single-read guard.
 */
#ifndef RTLMQ_TRACE_DEBUG_PASS
#undef TRACE_SYSTEM
#define TRACE_SYSTEM rtlmq_service

#if !defined(__RTLMQ_SERVICE_EVENT_H__) || defined(TRACE_HEADER_MULTI_READ)
#define __RTLMQ_SERVICE_EVENT_H__

#include <linux/tracepoint.h>

/*
 * A request that failed napi_schedule_prep() may already have an owner or
 * may be unavailable. The return value cannot distinguish those cases.
 */
TRACE_EVENT(rtlmq_service_request,

	TP_PROTO(const char *dev, u16 vector, u32 generation, u8 source,
		 bool admitted),
	TP_ARGS(dev, vector, generation, source, admitted),
	TP_STRUCT__entry(
		__string(dev, dev)
		__field(u16, vector)
		__field(u32, generation)
		__field(u8, source)
		__field(bool, admitted)
	),
	TP_fast_assign(
		__assign_str(dev);
		__entry->vector = vector;
		__entry->generation = generation;
		__entry->source = source;
		__entry->admitted = admitted;
	),
	TP_printk("dev=%s vector=%u generation=%u source=%u admitted=%u",
		  __get_str(dev), __entry->vector, __entry->generation,
		  __entry->source, __entry->admitted)
);

/*
 * first/end are monotonic descriptor counts within one ring generation,
 * not masked ring indices. Hardware OWN and the reclaim frontier are both
 * published before this event; it does not imply a doorbell or completion.
 */
TRACE_EVENT(rtlmq_tx_publish_range,

	TP_PROTO(const char *dev, u16 queue, u32 generation,
		 u32 first, u32 end, u32 packets),
	TP_ARGS(dev, queue, generation, first, end, packets),
	TP_STRUCT__entry(
		__string(dev, dev)
		__field(u16, queue)
		__field(u32, generation)
		__field(u32, first)
		__field(u32, end)
		__field(u32, packets)
	),
	TP_fast_assign(
		__assign_str(dev);
		__entry->queue = queue;
		__entry->generation = generation;
		__entry->first = first;
		__entry->end = end;
		__entry->packets = packets;
	),
	TP_printk("dev=%s queue=%u generation=%u first=%u end=%u packets=%u",
		  __get_str(dev), __entry->queue, __entry->generation,
		  __entry->first, __entry->end, __entry->packets)
);

/* A poll's GRO submissions and driver discards, not core delivery or XDP TX. */
TRACE_EVENT(rtlmq_rx_result,

	TP_PROTO(const char *dev, u16 queue, u32 generation, bool fast,
		 u32 skb_handoff, u32 driver_drops),
	TP_ARGS(dev, queue, generation, fast, skb_handoff, driver_drops),
	TP_STRUCT__entry(
		__string(dev, dev)
		__field(u16, queue)
		__field(u32, generation)
		__field(bool, fast)
		__field(u32, skb_handoff)
		__field(u32, driver_drops)
	),
	TP_fast_assign(
		__assign_str(dev);
		__entry->queue = queue;
		__entry->generation = generation;
		__entry->fast = fast;
		__entry->skb_handoff = skb_handoff;
		__entry->driver_drops = driver_drops;
	),
	TP_printk("dev=%s queue=%u generation=%u fast=%u skb_handoff=%u driver_drops=%u",
		  __get_str(dev), __entry->queue, __entry->generation,
		  __entry->fast, __entry->skb_handoff, __entry->driver_drops)
);

/* State was accepted under control_lock; OPEN alone is not runtime publication. */
TRACE_EVENT(rtlmq_lifecycle_transition,

	TP_PROTO(const char *dev, u8 from, u8 to, u32 generation),
	TP_ARGS(dev, from, to, generation),
	TP_STRUCT__entry(
		__string(dev, dev)
		__field(u8, from)
		__field(u8, to)
		__field(u32, generation)
	),
	TP_fast_assign(
		__assign_str(dev);
		__entry->from = from;
		__entry->to = to;
		__entry->generation = generation;
	),
	TP_printk("dev=%s from=%u to=%u generation=%u",
		  __get_str(dev), __entry->from, __entry->to,
		  __entry->generation)
);

#endif /* __RTLMQ_SERVICE_EVENT_H__ */

#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH .
#undef TRACE_INCLUDE_FILE
#define TRACE_INCLUDE_FILE rtlmq_trace

#ifndef TRACE_HEADER_MULTI_READ
#define RTLMQ_TRACE_SERVICE_PASS
#include <trace/define_trace.h>
#undef RTLMQ_TRACE_SERVICE_PASS
#endif
#endif
