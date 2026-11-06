/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef __RTLMQ_TX_H__
#define __RTLMQ_TX_H__

/*
 * TX descriptor bookkeeping, ring state, and transmit entry points.
 */
#include <linux/atomic.h>
#include <linux/build_bug.h>
#include <linux/cache.h>
#include <linux/dma-mapping.h>
#include <linux/netdevice.h>
#include <linux/overflow.h>
#include <linux/scatterlist.h>
#include <linux/spinlock.h>
#include <linux/u64_stats_sync.h>
#include "rtlmq_chip.h"
#include "rtlmq_desc.h"
#include "rtlmq_base.h"

struct rtlmq_private;
struct rtlmq_tx_ring;
struct rtlmq_vector;

enum rtlmq_serial_tx_phase {
	RTLMQ_SERIAL_IDLE,
	RTLMQ_SERIAL_ACTIVE,
	RTLMQ_SERIAL_DRAIN,
	RTLMQ_SERIAL_COMMIT,
	RTLMQ_SERIAL_RECOVERY,
};

enum rtlmq_serial_tx_reason {
	RTLMQ_SERIAL_FIRST,
	RTLMQ_SERIAL_EMPTY,
	RTLMQ_SERIAL_LIMIT,
	RTLMQ_SERIAL_FAIRNESS,
	RTLMQ_SERIAL_TIMEOUT,
	RTLMQ_SERIAL_INVALID,
};

/* Changed only under this ring's producer lock; never inferred from OWN. */
enum rtlmq_serial_tx_permit {
	RTLMQ_SERIAL_NONE,
	RTLMQ_SERIAL_DIRECT,
	RTLMQ_SERIAL_LIMITED,
	RTLMQ_SERIAL_DRAINING,
};

#define RTLMQ_SERIAL_EPOCH_DESCS 128U
/* Fixed dual-busy default; a larger runway must not mask missing TX service. */
#define RTLMQ_SERIAL_RUNWAY_DESCS 32U
/* CBS uses one-packet turns; indivisible packets may exceed this quota. */
#define RTLMQ_SERIAL_CBS_QUOTA_DESCS 1U
#define RTLMQ_SERIAL_EPOCH_PACKETS 128U
#define RTLMQ_SERIAL_FAIRNESS_MS 2U
#define RTLMQ_SERIAL_DRAIN_MS 100U
#define RTLMQ_SERIAL_REKICK_MS 10U
#define RTLMQ_SERIAL_NO_QUEUE U8_MAX

struct rtlmq_serial_tx_policy {
	u32 descs;
	u32 packets;
	u32 runway;
	bool cbs;
};

struct rtlmq_serial_tx_epoch {
	u64 bytes;
	u64 descs;
	u64 packets;
};

/* Stored only at packet heads in the existing software-info slot array. */
struct rtlmq_serial_tx_packet {
	/* Existing completion/BQL charge (SKB length for SKBs), not wire bytes. */
	u32 bytes;
	u16 descs;
	u16 queue;
};

struct rtlmq_serial_tx_snapshot {
	enum rtlmq_serial_tx_phase phase;
	enum rtlmq_serial_tx_reason reason;
	u32 dirty[RTLMQ_IMPL_MAX_TX_QUEUES], hw[RTLMQ_IMPL_MAX_TX_QUEUES];
	u32 commit[RTLMQ_IMPL_MAX_TX_QUEUES], stage[RTLMQ_IMPL_MAX_TX_QUEUES];
	u32 generation[RTLMQ_IMPL_MAX_TX_QUEUES];
	u32 published[RTLMQ_IMPL_MAX_TX_QUEUES], owned[RTLMQ_IMPL_MAX_TX_QUEUES];
	u32 frozen[RTLMQ_IMPL_MAX_TX_QUEUES], staged[RTLMQ_IMPL_MAX_TX_QUEUES];
	enum rtlmq_serial_tx_permit permit[RTLMQ_IMPL_MAX_TX_QUEUES];
	bool pending[RTLMQ_IMPL_MAX_TX_QUEUES];
	bool doorbell_pending[RTLMQ_IMPL_MAX_TX_QUEUES];
	struct rtlmq_serial_tx_epoch usage;
	u64 epoch;
	u64 handoffs;
	u64 handoff_ns;
	u64 drain_timeouts;
	u64 invariant_failures;
	u64 doorbells[RTLMQ_IMPL_MAX_TX_QUEUES];
	u8 active;
	u8 last_publication;
	u8 last_doorbell;
	bool paused;
};

/*
 * Both producer CPUs prepare independently. NAPI may borrow one queue lock
 * for publication/refill or revocation. Only diagnostics lock both queues.
 * The producer never takes the arbiter lock.
 */
struct rtlmq_serial_tx_producer {
	u32 stage;
	u32 commit;
	u32 stage_packets;
	u32 hw_packets;
	struct rtlmq_serial_tx_epoch epoch;
} ____cacheline_aligned;

struct rtlmq_serial_tx_control {
	enum rtlmq_serial_tx_permit permit;
	enum rtlmq_serial_tx_permit resume_permit;
	u32 generation;
	u32 watch_dirty;
	unsigned long watch_at;
	unsigned long timer_at;
	unsigned long fairness_at;
	u64 pending_since;
	u16 stage_high;
	u16 stage_low;
	u16 packet_high;
	u16 packet_low;
	bool paused;
	bool pending;
	bool watching;
	bool timer_armed;
	bool timer_due;
} ____cacheline_aligned;

struct rtlmq_serial_tx_arbiter {
	/*
	 * Serializes shared TX NAPI transitions against lifecycle and snapshots.
	 * Acquired before TX queue locks; producers never take this lock.
	 */
	spinlock_t lock;
	bool initialized;
	bool paused;
	bool dual_busy;
	u8 num_queues;
	enum rtlmq_serial_tx_phase phase;
	u8 active;
	u64 drain_start;
	struct rtlmq_serial_tx_policy policy;
	struct rtlmq_serial_tx_snapshot snapshot ____cacheline_aligned;
} ____cacheline_aligned;

static inline u32 rtlmq_tx_ring_distance(u32 end, u32 start)
{
	return end - start;
}

bool rtlmq_serial_tx_window_valid(u32 dirty, u32 hw, u32 commit, u32 stage,
				  u32 size);
bool rtlmq_tx_epoch_should_yield(const struct rtlmq_serial_tx_epoch *epoch,
				 const struct rtlmq_serial_tx_policy *policy);

void rtlmq_serial_tx_init(struct rtlmq_tx_ring *ring);
bool rtlmq_serial_tx_uses_cbs(struct rtlmq_private *tp);
void rtlmq_serial_tx_request(struct rtlmq_tx_ring *ring);
void rtlmq_serial_tx_watch(struct rtlmq_tx_ring *ring);
void rtlmq_serial_tx_stage_pending(struct rtlmq_tx_ring *ring);
void rtlmq_serial_tx_work(struct rtlmq_tx_ring *ring);
void rtlmq_serial_tx_service(struct rtlmq_vector *vector);
bool rtlmq_serial_tx_service_wanted(const struct rtlmq_vector *vector);
void rtlmq_serial_tx_pause(struct rtlmq_private *tp);
void rtlmq_serial_tx_resume(struct rtlmq_private *tp);
void rtlmq_serial_tx_cancel_staged(struct rtlmq_tx_ring *ring);
void rtlmq_serial_tx_cleanup_published(struct rtlmq_tx_ring *ring);
void rtlmq_serial_tx_snapshot(struct rtlmq_private *tp,
			      struct rtlmq_serial_tx_snapshot *snapshot);
void rtlmq_serial_tx_dump(struct rtlmq_private *tp);
bool rtlmq_serial_tx_capture_locked(struct rtlmq_private *tp,
				    struct rtlmq_serial_tx_snapshot *snapshot);
bool rtlmq_serial_tx_recovering(struct rtlmq_private *tp);
/* Queue lock required. DRAIN may notify, NONE must never notify. */
bool rtlmq_serial_tx_notify(struct rtlmq_tx_ring *ring);

#define RTLMQ_XSK_MAX_SEGS		16U
#define RTLMQ_MIN_TX_DESC		256U
#define RTLMQ_MAX_TX_DESC		1024U
#define RTLMQ_NUM_TX_DESC		RTLMQ_MAX_TX_DESC
#define RTLMQ_TX_DMA_SG_NENTS		(MAX_SKB_FRAGS + 1)
#define RTLMQ_TX_SMALL_COPY_BYTES	1536U
#define RTLMQ_TX_DESC_PRESSURE_MAX_BYTES	256U
#define RTLMQ_TX_LARGE_COPY_MIN_BYTES	768U
#define RTLMQ_TX_COPY_EARLY_IRQ_BYTES	4096U
#define RTLMQ_TX_MAPPED_EARLY_IRQ_BYTES	32768U
static_assert(SIZE_MAX / sizeof(struct rtlmq_tx_desc) >= RTLMQ_NUM_TX_DESC);
/*
 * Reserve a descriptor for the head and each fragment. The higher wake
 * threshold leaves room for several such skbs before the next stop.
 */
#define RTLMQ_TX_STOP_DESC_LOW		(MAX_SKB_FRAGS + 1)
#define RTLMQ_TX_WAKE_DESC_MIN_HEADROOM	(3 * RTLMQ_TX_STOP_DESC_LOW)
#define RTLMQ_TX_WAKE_DESC_HIGH		(RTLMQ_NUM_TX_DESC / 4)
static_assert(RTLMQ_TX_WAKE_DESC_HIGH >=
	      RTLMQ_TX_WAKE_DESC_MIN_HEADROOM);
static_assert(RTLMQ_TX_WAKE_DESC_HIGH < RTLMQ_NUM_TX_DESC);
/* Hardware request ceiling; profiles may advertise a smaller scheduling unit. */
#define RTLMQ_TSO_MAX_SKB_BYTES		GSO_LEGACY_MAX_SIZE

struct page;

enum {
	RTLMQ_TX_INFOF_MAPPED = BIT(0),
	RTLMQ_TX_INFOF_EOP = BIT(1),
	RTLMQ_TX_INFOF_PAGE = BIT(2),
	RTLMQ_TX_INFOF_UNMAP_OWNER = BIT(3),
	RTLMQ_TX_INFOF_TSO = BIT(4),
	RTLMQ_TX_INFOF_DMA_SG = BIT(5),
	RTLMQ_TX_INFOF_SMALL_COPY = BIT(6),
};

struct rtlmq_tx_info {
	dma_addr_t unmap_dma;
	u32 desc_len;
	u32 unmap_len;
	u32 generation;
	u16 completion_packets;
	u16 flags;
	struct rtlmq_serial_tx_packet packet;
};

struct rtlmq_tx_completion {
	union {
		struct sk_buff *skb;
		struct xdp_frame *xdpf;
		struct xsk_buff_pool *xsk_pool;
	};
	u32 completion_bytes;
	u32 timestamp_generation;
	u16 xsk_descs;
	u8 type;
	u8 flags;
};

enum {
	RTLMQ_TX_COMPLETIONF_HW_TSTAMP = BIT(0),
};

enum rtlmq_tx_own_recheck_reason {
	RTLMQ_TX_OWN_RECHECK_IRQ_BIT = 0,
	RTLMQ_TX_OWN_RECHECK_LIVENESS_BIT,
};

/*
 * Scatterlist and mapping state for one TSO skb. Stored at the final
 * descriptor slot before dma_map_sg() and kept at the same address until
 * that EOP slot is reclaimed. Descriptors share the mapping.
 */
struct rtlmq_tx_dma_txn {
	struct scatterlist sg[RTLMQ_TX_DMA_SG_NENTS];
	u16 orig_nents;
	bool mapped;
};

static_assert(SIZE_MAX / sizeof(struct rtlmq_tx_info) >= RTLMQ_NUM_TX_DESC);
static_assert(SIZE_MAX / sizeof(struct rtlmq_tx_completion) >= RTLMQ_NUM_TX_DESC);
static_assert(SIZE_MAX / sizeof(struct rtlmq_tx_dma_txn) >= RTLMQ_NUM_TX_DESC);

struct rtlmq_tx_ring_read_mostly {
	struct net_device *ndev;
	struct rtlmq_tx_desc *desc;
	dma_addr_t desc_dma;
	union {
		struct rtlmq_tx_info *buf_infos;
		struct rtlmq_tx_info *infos;
	};
	struct rtlmq_tx_completion *completions;
	struct rtlmq_tx_dma_txn *dma_txns;
	u8 *small_copy;
	dma_addr_t small_copy_dma;
	union {
		struct netdev_queue *txq;
		struct netdev_queue *ndev_queue;
	};
	void __iomem *doorbell;
	u16 doorbell_value;
	u16 tx_stop_desc_low;
	u16 tx_wake_desc_high;
	u16 ring_count;
	u16 ring_mask;
	u16 queue_index;
	/* Immutable profile fact cached when the committed ring is refreshed. */
	enum rtlmq_tx_desc_format desc_format;
	u8 desc_stride;
} ____cacheline_aligned;

struct rtlmq_tx_ring_producer {
	u16 next_to_use;
	u32 next_to_use_count;
	/*
	 * Set after new descriptors have been published and cleared by the
	 * single matching doorbell. Outstanding descriptors are tracked by the
	 * use/clean counts, not by this flag.
	 */
	bool tx_doorbell_pending;
	/*
	 * Reject an over-segmented AF_XDP packet through EOP.  This parser
	 * state survives NAPI budgets so a tail fragment cannot be published
	 * as a new packet.
	 */
	bool xsk_tx_discarding;
	/*
	 * Producer-only monotonic totals. Pending BQL work is their unsigned
	 * difference from the matching consumer totals.
	 */
	u32 bql_queued_packets;
	u32 bql_queued_bytes;
	u64 doorbells;
} ____cacheline_aligned;

struct rtlmq_tx_ring_consumer {
	u16 next_to_clean;
	u32 next_to_clean_count;
	/*
	 * NAPI consumes tx_reclaim_hint atomically. Published pending descriptors
	 * are next_to_use_count - next_to_clean_count.
	 */
	u32 tx_reclaim_hint;
	unsigned long own_recheck_reasons;
	u32 bql_completed_packets;
	u32 bql_completed_bytes;
} ____cacheline_aligned;

struct rtlmq_tx_timeout_snapshot {
	unsigned int queue;
	unsigned int pending;
	u64 clean_count;
	u32 ring_generation;
	bool generation_stable;
	u16 ring_vector_index;
};

/* Borrow the core batch until every reserved descriptor has an ordered owner. */
struct rtlmq_xsk_tx_batch {
	struct xsk_buff_pool *pool;
	u32 count;
	u32 cursor;
	u32 rejected;
	u32 retire_after;
};

struct rtlmq_tx_ring_cold {
	struct u64_stats_sync syncp;
	struct u64_stats_sync comp_syncp;
	struct rtlmq_ring_stats stats;
	struct delayed_work tx_liveness_work;
	struct rtlmq_serial_tx_arbiter serial;
	/*
	 * Teardown drains tx_liveness_work with cancel_delayed_work_sync()
	 * before freeing the ring.
	 */
	u32 liveness_work_generation;
	bool liveness_work_initialized;
	struct rtlmq_xsk_tx_batch xsk_batch;
};

struct rtlmq_tx_ring {
	struct rtlmq_private *priv;
	/*
	 * Named structs and anonymous members share storage, allowing both
	 * grouped and direct field access. Keep their field lists identical;
	 * the assertions below check sizes, alignment, and offsets.
	 */
	union {
		struct rtlmq_tx_ring_read_mostly read_mostly;
		struct {
			struct net_device *ndev;
			struct rtlmq_tx_desc *desc;
			dma_addr_t desc_dma;
			union {
				struct rtlmq_tx_info *buf_infos;
				struct rtlmq_tx_info *infos;
			};
			struct rtlmq_tx_completion *completions;
			struct rtlmq_tx_dma_txn *dma_txns;
			u8 *small_copy;
			dma_addr_t small_copy_dma;
				union {
					struct netdev_queue *txq;
					struct netdev_queue *ndev_queue;
				};
				void __iomem *doorbell;
				u16 doorbell_value;
			u16 tx_stop_desc_low;
			u16 tx_wake_desc_high;
			u16 ring_count;
			u16 ring_mask;
			u16 queue_index;
			enum rtlmq_tx_desc_format desc_format;
			u8 desc_stride;
		};
	};
	enum rtlmq_tx_subqueue_state {
		RTLMQ_TX_SUBQUEUE_DISABLED = 0,
		RTLMQ_TX_SUBQUEUE_BACKPRESSURED,
		RTLMQ_TX_SUBQUEUE_READY,
	} tx_subqueue_state;
	/*
	 * ndo_start_xmit(), ndo_xdp_xmit(), and native XDP_TX serialize new
	 * descriptors with the target netdev queue's xmit lock. NAPI updates
	 * next_to_clean. tx_subqueue_state records whether the queue is
	 * disabled, backpressured, or ready.
	 */
	union {
		struct rtlmq_tx_ring_producer producer;
		struct {
			u16 next_to_use;
			u32 next_to_use_count;
			bool tx_doorbell_pending;
			bool xsk_tx_discarding;
			u32 bql_queued_packets;
			u32 bql_queued_bytes;
			u64 doorbells;
		};
	};
	union {
		struct rtlmq_tx_ring_consumer consumer;
		struct {
			/*
			 * Vector-owned NAPI reclaims completed descriptors
			 * from this cursor.
			 */
			u16 next_to_clean;
			u32 next_to_clean_count;
			/*
			 * NAPI reclaim state; see rtlmq_tx_ring_consumer.
			 */
			u32 tx_reclaim_hint;
			unsigned long own_recheck_reasons;
			u32 bql_completed_packets;
			u32 bql_completed_bytes;
		};
	};
	struct rtlmq_serial_tx_producer serial;
	struct rtlmq_serial_tx_control serial_control;
	u16 vector_index;
	u32 generation;
	union {
		struct rtlmq_tx_ring_cold cold;
		struct {
			struct u64_stats_sync syncp;
			struct u64_stats_sync comp_syncp;
			struct rtlmq_ring_stats stats;
		};
	};
};

static_assert(__alignof__(struct rtlmq_tx_ring_read_mostly) >=
	      SMP_CACHE_BYTES);
static_assert(__alignof__(struct rtlmq_tx_ring_producer) >=
	      SMP_CACHE_BYTES);
static_assert(__alignof__(struct rtlmq_tx_ring_consumer) >=
	      SMP_CACHE_BYTES);
/* Guard cacheline budgets without requiring identical packing on every ABI. */
static_assert(sizeof(struct rtlmq_tx_ring_read_mostly) <=
	      2 * SMP_CACHE_BYTES);
static_assert(sizeof(struct rtlmq_tx_ring_producer) <=
	      1 * SMP_CACHE_BYTES);
static_assert(sizeof(struct rtlmq_tx_ring_consumer) <=
	      2 * SMP_CACHE_BYTES);

/* Only the first 16 bytes are touched for a short wire descriptor. */
static inline u8 rtlmq_tx_ring_desc_stride(const struct rtlmq_tx_ring *ring)
{
	return ring->desc_stride ? ring->desc_stride :
	       sizeof(struct rtlmq_tx_desc);
}

static inline struct rtlmq_tx_desc *
rtlmq_tx_desc_at_base(struct rtlmq_tx_desc *base, u8 stride, u16 entry)
{
	return (struct rtlmq_tx_desc *)((u8 *)base + (size_t)entry * stride);
}

static inline struct rtlmq_tx_desc *
rtlmq_tx_desc_at(const struct rtlmq_tx_ring *ring, u16 entry)
{
	return rtlmq_tx_desc_at_base(ring->desc,
				     rtlmq_tx_ring_desc_stride(ring), entry);
}

static_assert(!(offsetof(struct rtlmq_tx_ring, read_mostly) %
		SMP_CACHE_BYTES));
static_assert(!(offsetof(struct rtlmq_tx_ring, producer) %
		SMP_CACHE_BYTES));
static_assert(!(offsetof(struct rtlmq_tx_ring, consumer) %
		SMP_CACHE_BYTES));
static_assert(offsetof(struct rtlmq_tx_ring, producer) >=
	      offsetofend(struct rtlmq_tx_ring, read_mostly));
static_assert(offsetof(struct rtlmq_tx_ring, consumer) >=
	      offsetofend(struct rtlmq_tx_ring, producer));
static_assert(offsetof(struct rtlmq_tx_ring, doorbell_value) ==
	      offsetof(struct rtlmq_tx_ring, read_mostly.doorbell_value));
static_assert(offsetof(struct rtlmq_tx_ring, ring_count) ==
	      offsetof(struct rtlmq_tx_ring, read_mostly.ring_count));
static_assert((offsetof(struct rtlmq_tx_ring, read_mostly) /
	       SMP_CACHE_BYTES) !=
	      (offsetof(struct rtlmq_tx_ring, producer) /
	       SMP_CACHE_BYTES));
static_assert((offsetof(struct rtlmq_tx_ring, read_mostly) /
	       SMP_CACHE_BYTES) !=
	      (offsetof(struct rtlmq_tx_ring, consumer) /
	       SMP_CACHE_BYTES));
static_assert((offsetof(struct rtlmq_tx_ring, producer) /
	       SMP_CACHE_BYTES) !=
	      (offsetof(struct rtlmq_tx_ring, consumer) /
	       SMP_CACHE_BYTES));

/* Ring-local publication helpers; no device policy or arbiter access. */

static inline u32 rtlmq_tx_published_count(const struct rtlmq_tx_ring *ring)
{
	/* Reclaim/readers acquire the publisher's complete hardware interval. */
	u32 hw = smp_load_acquire(&ring->next_to_use_count);

	return rtlmq_tx_ring_distance(hw, READ_ONCE(ring->next_to_clean_count));
}

static inline u32 rtlmq_tx_unpublished_count(const struct rtlmq_tx_ring *ring)
{
	/* Staging may be observed outside the queue lock by NAPI capacity checks. */
	return rtlmq_tx_ring_distance(smp_load_acquire(&ring->serial.stage),
				      READ_ONCE(ring->next_to_use_count));
}

static inline u32 rtlmq_tx_stage_space(const struct rtlmq_tx_ring *ring)
{
	const struct rtlmq_serial_tx_control *c = &ring->serial_control;
	u32 packets =
		rtlmq_tx_ring_distance(READ_ONCE(ring->serial.stage_packets),
				       READ_ONCE(ring->serial.hw_packets));
	u32 descs = rtlmq_tx_unpublished_count(ring);

	if (READ_ONCE(c->permit) == RTLMQ_SERIAL_DIRECT && !descs)
		return ring->ring_count - 1;
	if (descs >= c->stage_high || packets >= c->packet_high)
		return 0;
	/*
	 * A free packet slot is a conservative one-descriptor reservation for
	 * the existing stop/recheck/wake interface. Physical space is separate.
	 */
	return min_t(u32, c->stage_high - descs, c->packet_high - packets);
}

static inline bool rtlmq_tx_staging_below_low(const struct rtlmq_tx_ring *ring)
{
	u32 packets =
		rtlmq_tx_ring_distance(READ_ONCE(ring->serial.stage_packets),
				       READ_ONCE(ring->serial.hw_packets));

	return rtlmq_tx_unpublished_count(ring) <=
		       ring->serial_control.stage_low &&
	       packets <= ring->serial_control.packet_low;
}

static inline bool
rtlmq_tx_ring_has_stage_space(const struct rtlmq_tx_ring *ring, u32 needed)
{
	return needed <= rtlmq_tx_stage_space(ring);
}

static __always_inline bool
rtlmq_tx_can_publish_fast(const struct rtlmq_tx_ring *ring)
{
	const struct rtlmq_serial_tx_control *c = &ring->serial_control;

	/* Caller holds the queue lock from construction through final OWN/kick. */
	if (unlikely(c->permit != RTLMQ_SERIAL_DIRECT ||
		     ring->serial.stage != ring->next_to_use_count))
		return false;

	return true;
}

static __always_inline void
rtlmq_write_tx_desc_staged(struct rtlmq_tx_ring *ring, unsigned int entry,
			   dma_addr_t addr, u32 len, u32 opts1, u32 opts2)
{
	struct rtlmq_tx_desc *desc = rtlmq_tx_desc_at(ring, entry);

	if (WARN_ON_ONCE(!len || len > RTLMQ_TX_DESC_LEN_MASK))
		return;
	/* OWN is not a construction option, even for native-style suffix callers. */
	opts1 &= ~(RTLMQ_DESC_OWN | RTLMQ_TX_DESC_LEN_MASK |
		   RTLMQ_DESC_RING_END);
	opts1 |= len;
	if (entry == ring->ring_count - 1)
		opts1 |= RTLMQ_DESC_RING_END;
	desc->addr = cpu_to_le64(addr);
	desc->opts2 = cpu_to_le32(opts2);
	WRITE_ONCE(desc->opts1, cpu_to_le32(opts1));
}

static __always_inline void
rtlmq_tx_epoch_account_packet(struct rtlmq_serial_tx_epoch *epoch,
			      const struct rtlmq_serial_tx_packet *packet)
{
	epoch->descs += packet->descs;
	epoch->bytes += packet->bytes;
	epoch->packets++;
}

static __always_inline void rtlmq_serial_tx_stage(struct rtlmq_tx_ring *ring,
						  u16 descs, u32 bytes)
{
	struct rtlmq_serial_tx_producer *p = &ring->serial;
	struct rtlmq_serial_tx_packet *packet =
		&ring->infos[p->stage & ring->ring_mask].packet;

	packet->descs = descs;
	packet->bytes = bytes;
	packet->queue = ring->queue_index;
	p->stage_packets++;
	/* Complete immutable packet, including metadata and BQL acceptance. */
	smp_store_release(&p->stage, p->stage + descs);
}

/*
 * Publication mechanism shared by direct packets and NAPI batches. Complete
 * opts are already in coherent memory; do not rediscover format or packet ends.
 * Suffix access is sequential (including ring wrap), followed by the gate.
 */
static __always_inline void
rtlmq_tx_publish_packet_own(struct rtlmq_tx_ring *ring, u32 first, u16 descs)
{
	struct rtlmq_tx_desc *gate =
		rtlmq_tx_desc_at(ring, first & ring->ring_mask);
	u32 end = first + descs;
	u32 i;

	if (descs > 1) {
		dma_wmb();
		for (i = first + 1; i != end; i++) {
			struct rtlmq_tx_desc *desc =
				rtlmq_tx_desc_at(ring, i & ring->ring_mask);

			WRITE_ONCE(desc->opts1,
				   desc->opts1 | cpu_to_le32(RTLMQ_DESC_OWN));
		}
	}
	/* Orders descriptor contents and suffix ownership before releasing gate. */
	rtlmq_tx_desc_device_release(gate, le32_to_cpu(gate->opts1));
}

static __always_inline void rtlmq_tx_publish_fast(struct rtlmq_tx_ring *ring,
						  u16 descs)
{
	u32 first = ring->next_to_use_count;
	const struct rtlmq_serial_tx_packet *packet =
		&ring->infos[first & ring->ring_mask].packet;

	ring->serial.commit = ring->serial.stage;
	rtlmq_tx_epoch_account_packet(&ring->serial.epoch, packet);
	rtlmq_tx_publish_packet_own(ring, first, descs);
	ring->serial.hw_packets++;
	WRITE_ONCE(ring->next_to_use, ring->serial.commit & ring->ring_mask);
	/* Reclaim's acquire sees metadata and BQL only after OWN publication. */
	smp_store_release(&ring->next_to_use_count, ring->serial.commit);
	ring->tx_doorbell_pending = true;
}

static_assert(sizeof(struct rtlmq_serial_tx_packet) == 8);
static_assert(!(offsetof(struct rtlmq_tx_ring, serial) % SMP_CACHE_BYTES));
static_assert(!(offsetof(struct rtlmq_tx_ring, serial_control) %
		SMP_CACHE_BYTES));
static_assert(!(offsetof(struct rtlmq_tx_ring, cold.serial) % SMP_CACHE_BYTES));

struct net_device;
struct work_struct;
struct rtlmq_vector;
struct sk_buff;

static __always_inline unsigned int
rtlmq_tx_desc_advance(const struct rtlmq_tx_ring *ring, unsigned int entry)
{
	return (entry + 1) & ring->ring_mask;
}

/* TX queue control and skb/XDP transmit entry points. */
void rtlmq_sync_tx_subqueues(struct rtlmq_private *tp);
bool rtlmq_tx_flush_doorbell_if_pending(struct rtlmq_tx_ring *ring);
bool rtlmq_tx_flush_pending_ring(struct rtlmq_tx_ring *ring);
void rtlmq_flush_pending_tx_doorbells(struct rtlmq_private *tp);
void rtlmq_flush_vector_pending_tx_doorbells(struct rtlmq_vector *vector);
bool rtlmq_tx_ring_boundary_ready(struct rtlmq_tx_ring *ring);
netdev_tx_t rtlmq_start_xmit(struct sk_buff *skb, struct net_device *ndev);

static inline const struct rtlmq_tx_ring_read_mostly *
rtlmq_tx_ring_rm(const struct rtlmq_tx_ring *ring)
{
	return &ring->read_mostly;
}

static inline u32 rtlmq_tx_accepted_pending(const struct rtlmq_tx_ring *ring)
{
	/* Pairs with serial staging's release of complete accepted packets. */
	u32 stage = smp_load_acquire(&ring->serial.stage);
	/* Acquire NAPI's later retirement so reused space is not overcounted. */
	u32 dirty = smp_load_acquire(&ring->next_to_clean_count);

	return rtlmq_tx_ring_distance(stage, dirty);
}

static inline unsigned int rtlmq_tx_desc_avail(const struct rtlmq_tx_ring *ring)
{
	const struct rtlmq_tx_ring_read_mostly *rm = rtlmq_tx_ring_rm(ring);
	unsigned int ring_count = rm->ring_count;
	unsigned int used;

	used = rtlmq_tx_accepted_pending(ring);
	return min(used < ring_count ? ring_count - used - 1 : 0,
		   rtlmq_tx_stage_space(ring));
}

static inline u16 rtlmq_tx_stop_desc_low(const struct rtlmq_tx_ring *ring)
{
	return rtlmq_tx_ring_rm(ring)->tx_stop_desc_low;
}

static inline u16 rtlmq_tx_wake_desc_high(const struct rtlmq_tx_ring *ring)
{
	return rtlmq_tx_ring_rm(ring)->tx_wake_desc_high;
}

static inline bool rtlmq_tx_ring_is_empty(const struct rtlmq_tx_ring *ring)
{
	return !rtlmq_tx_accepted_pending(ring);
}

/*
 * TX reclaim entry points and results for NAPI and timeout diagnostics.
 */

/*
 * Poll-local location, not cached ownership. Serialized packet preflight may
 * stop beyond dirty; the same NAPI owner rechecks that blocker before releasing
 * NAPI. Never retain this across polls or use it to authorize DMA retirement.
 */
struct rtlmq_tx_reclaim_wait {
	u32 clean_count;
	u32 blocked_count;
	u32 generation;
	bool valid;
};

struct rtlmq_tx_poll_result {
	unsigned int pkts_cleaned;
	struct rtlmq_tx_reclaim_wait wait;
};

struct rtlmq_tx_poll_result
rtlmq_poll_tx_ring_with_hint(struct rtlmq_tx_ring *ring, int budget);

void rtlmq_tx_finish_eop(struct rtlmq_tx_ring *ring, unsigned int entry,
			 int budget, unsigned int *pkts, unsigned int *bytes,
			 unsigned int *events);
void rtlmq_tx_unmap_sg_txn_owned(struct rtlmq_tx_ring *ring,
				 unsigned int entry);
void rtlmq_unmap_tx_info(struct rtlmq_tx_ring *ring, unsigned int entry);

void rtlmq_free_tx_info(struct rtlmq_tx_ring *ring, unsigned int entry);
void rtlmq_unwind_tx_entries(struct rtlmq_tx_ring *ring, unsigned int start,
			     unsigned int mapped);

/*
 * TX subqueue state updates and netdev queue readiness checks.
 */
static inline bool
rtlmq_netstack_tx_queue_live(const struct rtlmq_tx_ring *ring)
{
	const struct rtlmq_tx_ring_read_mostly *rm;
	const struct net_device *ndev;

	if (!ring || !ring->priv)
		return false;

	rm = rtlmq_tx_ring_rm(ring);
	if (!rm->ndev_queue)
		return false;

	ndev = rm->ndev;
	if (!ndev)
		return false;

	/*
	 * Queue objects exist before registration, but core wake/reset handling
	 * is live only after registration.
	 */
	return ndev->reg_state == NETREG_REGISTERED;
}

static inline bool
rtlmq_netdev_tx_queue_stack_xoff(const struct netdev_queue *txq)
{
	return txq &&
	       !!(READ_ONCE(txq->state) & BIT(__QUEUE_STATE_STACK_XOFF));
}

void rtlmq_sync_tx_subqueue(struct rtlmq_tx_ring *ring);
int rtlmq_tx_maybe_stop_subqueue(struct rtlmq_tx_ring *ring,
				 unsigned int stop_threshold);
bool rtlmq_tx_completion_wake_allowed(const struct rtlmq_tx_ring *ring);
void rtlmq_sync_tx_datapath_subqueue(struct rtlmq_tx_ring *ring);
void rtlmq_publish_tx_datapath_subqueue_state(struct rtlmq_tx_ring *ring,
					      enum rtlmq_tx_subqueue_state state);
void rtlmq_sync_tx_subqueue_after_clean(struct rtlmq_tx_ring *ring,
					unsigned int descs_cleaned,
					u32 pending_before, u32 pending_after);
void rtlmq_publish_tx_subqueue_state(struct rtlmq_tx_ring *ring,
				     enum rtlmq_tx_subqueue_state state);

/*
 * TX watchdog callback, reset suppression, and timeout snapshot helpers.
 */
void rtlmq_tx_timeout(struct net_device *ndev, unsigned int txqueue);
bool rtlmq_tx_timeout_maybe_suppress_reset(struct rtlmq_private *tp, u32 reset_reasons,
					   unsigned int timeout_events,
					   const struct rtlmq_tx_timeout_snapshot *timeout_snap);

/*
 * Per-ring delayed work requests TX service. Cancel synchronously before
 * freeing the ring; the callback never frees descriptors itself.
 */
void rtlmq_tx_ring_cancel_liveness(struct rtlmq_tx_ring *ring);
void rtlmq_tx_liveness_work_cb(struct work_struct *work);

/* The reset queue owner holds state_lock across payload and event metadata. */
void rtlmq_store_tx_timeout_snapshot_locked(struct rtlmq_private *tp,
					    const struct rtlmq_tx_timeout_snapshot *snap);
bool rtlmq_load_tx_timeout_snapshot_locked(struct rtlmq_private *tp,
					   struct rtlmq_tx_timeout_snapshot *snap);

#endif /* __RTLMQ_TX_H__ */
