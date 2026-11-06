/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef __RTLMQ_TX_PRIVATE_H__
#define __RTLMQ_TX_PRIVATE_H__

/*
 * Inline TX ring lookup and reclaim checks for IRQ and NAPI. These helpers
 * need the private device struct; the public TX API does not.
 */
#include "rtlmq.h"
#include "rtlmq_irq.h"
#include "rtlmq_tx.h"

/*
 * Caller must keep the device, vector, and ring storage alive. This helper
 * checks only the vector-array index and allocated-ring bounds.
 */
static __always_inline struct rtlmq_tx_ring *
rtlmq_vector_tx_ring_at(const struct rtlmq_private *tp,
			const struct rtlmq_vector *vector, unsigned int index,
			u16 ring_slots)
{
	u16 tx_queue;

	if (unlikely(index >= ARRAY_SIZE(vector->tx_ring_indices)))
		return NULL;

	tx_queue = READ_ONCE(vector->tx_ring_indices[index]);
	if (unlikely(tx_queue >= ring_slots))
		return NULL;

	return &tp->tx_rings[tx_queue];
}

static __always_inline u32 rtlmq_tx_desc_status_read(const struct rtlmq_tx_desc *desc)
{
	return le32_to_cpu(READ_ONCE(desc->opts1));
}

/*
 * Do not read completed descriptor data until OWN clears. dma_rmb() orders
 * subsequent reads after the status check. Status-only readiness probes do
 * not need this barrier because they do not consume descriptor data.
 */
static __always_inline bool
rtlmq_tx_desc_acquire(const struct rtlmq_tx_desc *desc, u32 *opts1)
{
	u32 status = rtlmq_tx_desc_status_read(desc);

	if (unlikely(status & RTLMQ_DESC_OWN)) {
		*opts1 = status;
		return false;
	}

	dma_rmb();
	*opts1 = status;
	return true;
}

static __always_inline u32
rtlmq_tx_ring_desc_status_hint_window(const struct rtlmq_tx_ring *ring)
{
	u32 clean_count;
	u32 use_count;

	/* Acquire the publisher's descriptor stores and hardware frontier. */
	use_count = smp_load_acquire(&ring->next_to_use_count);
	clean_count = READ_ONCE(ring->next_to_clean_count);

	return use_count - clean_count;
}

static __always_inline void
rtlmq_tx_ring_arm_own_recheck(struct rtlmq_tx_ring *ring,
			      enum rtlmq_tx_own_recheck_reason reason)
{
	if (!ring)
		return;

	if (!rtlmq_tx_ring_desc_status_hint_window(ring))
		return;

	set_bit(reason, &ring->own_recheck_reasons);
}

static __always_inline u32
rtlmq_tx_bql_pending_packets(const struct rtlmq_tx_ring *ring)
{
	return READ_ONCE(ring->bql_queued_packets) -
	       READ_ONCE(ring->bql_completed_packets);
}

static __always_inline u32
rtlmq_tx_bql_pending_bytes(const struct rtlmq_tx_ring *ring)
{
	return READ_ONCE(ring->bql_queued_bytes) -
	       READ_ONCE(ring->bql_completed_bytes);
}

static __always_inline void
rtlmq_tx_ring_note_desc_status_pending(struct rtlmq_tx_ring *ring)
{
	if (!ring)
		return;

	if (!rtlmq_tx_ring_desc_status_hint_window(ring))
		return;

	/*
	 * Request a scan, not a descriptor count. Reclaim computes the pending
	 * count and checks each descriptor's OWN bit before cleaning it.
	 */
	smp_store_release(&ring->tx_reclaim_hint, 1);
}

static __always_inline bool
rtlmq_tx_ring_queue_stopped_and_pending(const struct rtlmq_tx_ring *ring)
{
	const struct rtlmq_tx_ring_read_mostly *rm;
	struct netdev_queue *txq;
	u32 clean_count;
	u32 pending;

	if (!ring)
		return false;

	rm = rtlmq_tx_ring_rm(ring);
	txq = READ_ONCE(rm->txq);
	if (!txq || !netif_tx_queue_stopped(txq))
		return false;

	/* Pairs with rtlmq_tx_publish_clean_batch() after next_to_clean. */
	clean_count = smp_load_acquire(&ring->next_to_clean_count);
	pending = READ_ONCE(ring->next_to_use_count) - clean_count;

	return pending != 0;
}

static __always_inline bool
rtlmq_tx_ring_tx_irq_wanted(const struct rtlmq_tx_ring *ring)
{
	const struct rtlmq_private *tp;
	bool shared_sources;
	u16 vector_index;

	if (!ring)
		return false;

	tp = ring->priv;
	if (!tp)
		return false;

	if (!tp->vectors)
		return !rtlmq_coupled_tx_irq_suppression_active(tp);

	vector_index = READ_ONCE(ring->vector_index);
	if (vector_index >= READ_ONCE(tp->allocated_vectors))
		return false;

	shared_sources =
		rtlmq_vector_has_shared_data_sources(&tp->vectors[vector_index]);
	if (!rtlmq_coupled_tx_irq_suppression_active(tp) &&
	    !shared_sources)
		return true;

	return READ_ONCE(tp->vectors[vector_index].tx_irq_armed_wanted);
}

static __always_inline bool
rtlmq_tx_ring_single_desc_pressure(const struct rtlmq_tx_ring *ring)
{
	const struct rtlmq_tx_ring_read_mostly *rm;
	unsigned int entry;
	u16 flags;

	if (!ring)
		return false;

	rm = rtlmq_tx_ring_rm(ring);
	if (unlikely(!rm->infos))
		return false;

	/*
	 * Single-descriptor traffic can outrun completion service before core
	 * XOFF. Byte pressure catches larger packets before BQL throttles them;
	 * descriptor pressure preserves the established small-packet batching.
	 */
	entry = READ_ONCE(ring->next_to_clean);
	if (entry >= READ_ONCE(ring->ring_count))
		return false;

	flags = READ_ONCE(rm->infos[entry].flags);
	if (flags == (RTLMQ_TX_INFOF_SMALL_COPY | RTLMQ_TX_INFOF_EOP)) {
		if (READ_ONCE(rm->infos[entry].desc_len) >
		    RTLMQ_TX_DESC_PRESSURE_MAX_BYTES &&
		    rtlmq_tx_bql_pending_bytes(ring) >=
			    RTLMQ_TX_COPY_EARLY_IRQ_BYTES)
			return true;
		return rtlmq_tx_desc_avail(ring) <=
		       READ_ONCE(ring->ring_count) / 2;
	}
	if (flags != (RTLMQ_TX_INFOF_MAPPED |
		      RTLMQ_TX_INFOF_UNMAP_OWNER |
		      RTLMQ_TX_INFOF_EOP))
		return false;

	return rtlmq_tx_bql_pending_bytes(ring) >=
		       RTLMQ_TX_MAPPED_EARLY_IRQ_BYTES ||
	       rtlmq_tx_desc_avail(ring) <=
		       READ_ONCE(ring->ring_count) / 2;
}

#endif /* __RTLMQ_TX_PRIVATE_H__ */
