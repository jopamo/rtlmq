// SPDX-License-Identifier: GPL-2.0-only

#include <linux/smp.h>

#include "rtlmq_irq.h"
#include "rtlmq.h"
#include "rtlmq_regs.h"
#include "rtlmq_runtime_accessors.h"

static bool
rtlmq_vector_has_cached_data_irq_masks(const struct rtlmq_vector *vector)
{
	return READ_ONCE(vector->data_irq_mask_rx_tx) ||
	       READ_ONCE(vector->data_irq_mask_rx_only) ||
	       READ_ONCE(vector->data_irq_mask_tx_only);
}

bool rtlmq_runtime_vector_uses_msix_message_routing(const struct rtlmq_vector *vector)
{
	const struct rtlmq_private *tp;

	if (!vector)
		return false;

	tp = vector->priv;
	if (!tp || !tp->active_profile.valid)
		return false;

	/*
	 * The routing bit is set when building the vector layout. Require an
	 * active profile before returning it so callers cannot use a staged
	 * layout before commit.
	 */
	return READ_ONCE(vector->uses_msix_message_routing);
}

static u32
rtlmq_filter_empty_rx_irq_mask_for_tx(const struct rtlmq_vector *vector,
				      u32 mask, bool tx_armed)
{
	const struct rtlmq_private *tp;
	bool uses_message_routing;
	bool empty_rx = false;
	u16 ring_slots;
	unsigned int i;

	if (!vector || !mask || !READ_ONCE(vector->num_rx_rings))
		return mask;

	tp = vector->priv;
	if (!tp || !tp->rx_rings)
		return mask;

	uses_message_routing =
		rtlmq_runtime_vector_uses_msix_message_routing(vector);
	ring_slots = rtlmq_owned_rx_ring_slots(tp);
	for (i = 0; i < READ_ONCE(vector->num_rx_rings); i++) {
		const struct rtlmq_rx_ring *ring;
		u16 ring_index;
		u8 message_id;

		if (unlikely(i >= ARRAY_SIZE(vector->rx_ring_indices)))
			break;
		ring_index = READ_ONCE(vector->rx_ring_indices[i]);
		if (ring_index >= ring_slots)
			continue;

		ring = &tp->rx_rings[ring_index];
		if (READ_ONCE(ring->rx_posted))
			continue;

		empty_rx = true;
		if (!uses_message_routing)
			continue;

		/*
		 * Suppress an empty queue's RX capability. If TX currently needs a
		 * shared source, keep that source enabled for TX completion.
		 */
		message_id = READ_ONCE(vector->rx_message_ids[i]);
		if (message_id < 32) {
			u32 suppressible =
				READ_ONCE(vector->data_irq_mask_rx_only);

			if (tx_armed)
				suppressible &=
					~READ_ONCE(vector->data_irq_mask_tx_only);
			mask &= ~(BIT(message_id) & suppressible);
		}
	}

	/*
	 * Legacy MSI/INTx exposes one shared status word rather than
	 * queue-addressable messages. Keep RX completion and error events
	 * armed for populated queues, but suppress the level-like descriptor
	 * unavailable cause while any active queue has no descriptors.
	 * AF_XDP wakeup or the page-refill retry supplies missing descriptors.
	 */
	if (empty_rx && !uses_message_routing)
		mask &= ~RTLMQ_INT_RX_DESC_UNAVAIL;

	return mask;
}

u32 rtlmq_filter_empty_rx_irq_mask(const struct rtlmq_vector *vector, u32 mask)
{
	bool tx_armed = vector && READ_ONCE(vector->tx_irq_armed_wanted);

	return rtlmq_filter_empty_rx_irq_mask_for_tx(vector, mask, tx_armed);
}

u32 rtlmq_vector_rx_data_irq_wanted(const struct rtlmq_vector *vector)
{
	u32 rx_mask;

	if (!vector)
		return 0;

	rx_mask = READ_ONCE(vector->data_irq_mask_rx_only);
	return rtlmq_filter_empty_rx_irq_mask_for_tx(vector, rx_mask, false);
}

u32 rtlmq_vector_data_irq_mask_wanted_cached(const struct rtlmq_vector *vector,
					     bool tx_armed)
{
	return tx_armed ? READ_ONCE(vector->data_irq_mask_rx_tx) :
			  READ_ONCE(vector->data_irq_mask_rx_only);
}

u32 rtlmq_vector_data_irq_wanted_for_unmask(const struct rtlmq_vector *vector)
{
	u32 wanted_mask = READ_ONCE(vector->data_irq_mask_wanted);
	bool tx_armed = READ_ONCE(vector->tx_irq_armed_wanted);
	u32 cached_mask =
		rtlmq_vector_data_irq_mask_wanted_cached(vector, tx_armed);

	if (unlikely(rtlmq_vector_has_cached_data_irq_masks(vector) &&
		     wanted_mask != cached_mask))
		wanted_mask = cached_mask;

	/* Partial fallback may leave an active source table without cached masks. */
	if (unlikely(rtlmq_runtime_vector_uses_msix_message_routing(vector) &&
		     !READ_ONCE(vector->msix_data_mask)))
		wanted_mask = rtlmq_vector_wake_data_mask(vector);

	return rtlmq_filter_empty_rx_irq_mask(vector, wanted_mask);
}

bool rtlmq_coupled_tx_irq_suppression_active(const struct rtlmq_private *tp)
{
	const struct rtlmq_runtime_config *cfg;

	/*
	 * Derive this from the active configuration. OPENING needs it after
	 * active commit but before runtime_published is set, so a published
	 * accessor would return it too late.
	 */
	cfg = rtlmq_active_runtime_config(tp);
	return cfg && cfg->irq_mode == RTLMQ_IRQ_MODE_MSIX &&
	       cfg->routing_mode == RTLMQ_ROUTING_RX_PER_VECTOR_TX_COUPLED &&
	       cfg->num_vectors > cfg->num_tx_queues;
}

u32 rtlmq_vector_requested_source_mask(const struct rtlmq_vector *vector,
				       enum rtlmq_irq_source_role role)
{
	unsigned int i;
	u32 mask = 0;

	for (i = 0; i < vector->num_irq_sources; i++) {
		const struct rtlmq_irq_source *source = &vector->irq_sources[i];

		if (!source->requested || source->role != role)
			continue;
		if (source->message_id >= 32)
			continue;

		mask |= BIT(source->message_id);
	}

	return mask;
}

static u32 rtlmq_vector_data_irq_mask(const struct rtlmq_vector *vector)
{
	u32 mask = 0;

	if (!vector || !READ_ONCE(vector->irq_requested))
		return 0;

	if (rtlmq_runtime_vector_uses_msix_message_routing(vector))
		mask = READ_ONCE(vector->msix_data_mask);
	else
		mask = READ_ONCE(vector->irq_mask);
	if (mask)
		return mask;

	return rtlmq_vector_requested_source_mask(vector, RTLMQ_IRQ_SOURCE_DATA);
}

static u32 rtlmq_vector_kind_irq_mask(const struct rtlmq_vector *vector,
				      u8 kind)
{
	unsigned int i;
	u32 mask = 0;

	if (!vector)
		return 0;

	for (i = 0; i < vector->num_irq_sources; i++) {
		const struct rtlmq_irq_source *source = &vector->irq_sources[i];

		if (source->role != RTLMQ_IRQ_SOURCE_DATA ||
		    source->message_id >= BITS_PER_TYPE(mask) ||
		    !(source->kind_flags & kind))
			continue;

		mask |= BIT(source->message_id);
	}

	return mask;
}

void rtlmq_vector_refresh_data_irq_masks(struct rtlmq_vector *vector)
{
	bool staged_uses_msix_message_routing;
	u32 data_mask;

	if (!vector)
		return;

	staged_uses_msix_message_routing =
		READ_ONCE(vector->uses_msix_message_routing);

	if (staged_uses_msix_message_routing)
		data_mask = READ_ONCE(vector->msix_data_mask);
	else
		data_mask = READ_ONCE(vector->irq_mask);
	if (!data_mask)
		data_mask = rtlmq_vector_data_irq_mask(vector);
	WRITE_ONCE(vector->data_irq_mask_rx_tx, data_mask);
	if (staged_uses_msix_message_routing) {
		u32 tx_mask = READ_ONCE(vector->tx_irq_mask);
		u32 rx_mask =
			rtlmq_vector_kind_irq_mask(vector, RTLMQ_IRQ_SOURCE_KIND_RX);

		WRITE_ONCE(vector->data_irq_mask_tx_only, data_mask & tx_mask);
		WRITE_ONCE(vector->data_irq_mask_rx_only, data_mask & rx_mask);
		return;
	}

	WRITE_ONCE(vector->data_irq_mask_tx_only, data_mask);
	WRITE_ONCE(vector->data_irq_mask_rx_only, data_mask);
}

static void rtlmq_vector_publish_tx_irq_wanted(struct rtlmq_vector *vector,
					       bool tx_armed)
{
	u32 wanted_mask;

	if (!vector)
		return;

	wanted_mask =
		rtlmq_vector_data_irq_mask_wanted_cached(vector, tx_armed);
	if (likely(READ_ONCE(vector->tx_irq_armed_wanted) == tx_armed &&
		   READ_ONCE(vector->data_irq_mask_wanted) == wanted_mask))
		return;

	if (READ_ONCE(vector->tx_irq_armed_wanted) != tx_armed)
		WRITE_ONCE(vector->tx_irq_armed_wanted, tx_armed);
	if (READ_ONCE(vector->data_irq_mask_wanted) != wanted_mask)
		WRITE_ONCE(vector->data_irq_mask_wanted, wanted_mask);
}

static bool rtlmq_vector_tx_irq_should_arm(const struct rtlmq_vector *vector)
{
	if (!vector || !READ_ONCE(vector->tx_irq_mask))
		return false;

	/*
	 * Close mode has no producer cursor to poll. Keep its completion
	 * message armed for every outstanding batch so BQL can release
	 * sparse traffic without waiting for unrelated RX or the
	 * liveness backstop.
	 */
	return rtlmq_vector_has_tx_pending_desc(vector);
}

u32 rtlmq_vector_resolve_data_irq_wanted(struct rtlmq_vector *vector)
{
	struct rtlmq_private *tp;
	u32 wanted_mask;

	if (!vector)
		return 0;

	tp = vector->priv;
	if (!tp)
		return 0;

	wanted_mask = rtlmq_vector_data_irq_wanted_for_unmask(vector);
	if (READ_ONCE(vector->data_irq_mask_wanted) != wanted_mask)
		WRITE_ONCE(vector->data_irq_mask_wanted, wanted_mask);

	return wanted_mask;
}

u32 rtlmq_vector_data_irq_hw_enable_mask(const struct rtlmq_vector *vector,
					 u32 wanted_mask)
{
	u32 enable_mask;

	if (!vector)
		return 0;

	enable_mask = wanted_mask & ~READ_ONCE(vector->data_irq_mask_current);

	/*
	 * A CLEAR/SET posted-write race on IMR_V2 can leave the TX message
	 * bit cleared in silicon while the software shadow still reports it
	 * enabled. Reassert the TX-only bit whenever TX is wanted so a stale
	 * shadow can never paper over a lost hardware source. IMR_V2_SET is
	 * idempotent, so the redundant write is harmless.
	 */
	if (READ_ONCE(vector->tx_irq_armed_wanted))
		enable_mask |= READ_ONCE(vector->data_irq_mask_tx_only);

	return enable_mask;
}

static bool rtlmq_vector_napi_service_pending(const struct rtlmq_vector *vector)
{
	/* NAPI scheduled or running: its poll reclaims TX. */
	if (READ_ONCE(vector->napi.state) & NAPIF_STATE_SCHED)
		return true;

	/*
	 * The core refused the last completion, so it owns the next poll:
	 * the core NAPI timer, a missed schedule, or an external poll owner.
	 * That service path is guaranteed, so a producer must not open the TX
	 * completion source and turn it into a hard IRQ. The latch is cleared
	 * when a poll starts, and a missed schedule sets SCHED above.
	 */
	return READ_ONCE(vector->napi_service_pending);
}

void rtlmq_vector_note_napi_service_pending(struct rtlmq_vector *vector,
					    bool pending)
{
	unsigned long flags;

	if (!vector)
		return;

	spin_lock_irqsave(&vector->napi_rearm_lock, flags);
	WRITE_ONCE(vector->napi_service_pending, pending);
	spin_unlock_irqrestore(&vector->napi_rearm_lock, flags);
}

static void rtlmq_sync_vector_tx_irq_arming(struct rtlmq_vector *vector,
					    bool tx_armed)
{
	struct rtlmq_private *tp;
	unsigned long flags;
	u32 current_mask;
	u32 disable_mask;
	u32 wanted_mask;
	u32 enable_mask;

	if (!vector)
		return;

	spin_lock_irqsave(&vector->napi_rearm_lock, flags);
	lockdep_assert_held(&vector->napi_rearm_lock);

	if (!tx_armed) {
		rtlmq_vector_publish_tx_irq_wanted(vector, false);
		/*
		 * Pair with the producers' barriers before they read IRQ intent.
		 * Either they see the disarm and rearm TX, or this rescan sees
		 * their updated next_to_use_count.
		 */
		smp_mb();
		if (rtlmq_vector_tx_irq_should_arm(vector))
			tx_armed = true;
	}

	rtlmq_vector_publish_tx_irq_wanted(vector, tx_armed);

	tp = vector->priv;
	if (!tp || !rtlmq_runtime_vector_uses_msix_message_routing(vector) ||
	    !rtlmq_mmio_ordinary_authorized(tp) || !vector->irq_requested ||
	    !vector->napi_enabled || !READ_ONCE(vector->napi_rearm_allowed) ||
	    rtlmq_vector_napi_service_pending(vector))
		goto out_unlock;

	wanted_mask = rtlmq_vector_resolve_data_irq_wanted(vector);
	current_mask = READ_ONCE(vector->data_irq_mask_current);
	if (current_mask == wanted_mask && !tx_armed)
		goto out_unlock;

	enable_mask = rtlmq_vector_data_irq_hw_enable_mask(vector,
							   wanted_mask);
	disable_mask = current_mask & ~wanted_mask;
	if (disable_mask)
		rtlmq_disable_msix_mask(tp, disable_mask);
	if (enable_mask)
		rtlmq_enable_msix_mask(tp, enable_mask);
	WRITE_ONCE(vector->data_irq_mask_current, wanted_mask);

out_unlock:
	spin_unlock_irqrestore(&vector->napi_rearm_lock, flags);
}

bool rtlmq_uses_msix_message_routing(const struct rtlmq_private *tp)
{
	const struct rtlmq_runtime_config *cfg =
		rtlmq_active_runtime_config(tp);

	/*
	 * IRQ/NAPI callers use only the current runtime here. Candidate routing
	 * is checked separately in rtlmq_msix.c.
	 */
	return cfg &&
	       cfg->routing_mode == RTLMQ_ROUTING_RX_PER_VECTOR_TX_COUPLED &&
	       cfg->irq_mode == RTLMQ_IRQ_MODE_MSIX;
}

u32 rtlmq_vector_wake_data_mask(const struct rtlmq_vector *vector)
{
	if (!vector)
		return 0;

	if (READ_ONCE(vector->msix_data_mask))
		return READ_ONCE(vector->msix_data_mask);

	return rtlmq_vector_data_irq_mask(vector);
}

u32 rtlmq_vector_tx_irq_mask(const struct rtlmq_vector *vector)
{
	return rtlmq_vector_kind_irq_mask(vector, RTLMQ_IRQ_SOURCE_KIND_TX);
}

bool rtlmq_vector_has_shared_data_sources(const struct rtlmq_vector *vector)
{
	if (!vector)
		return false;

	return READ_ONCE(vector->data_irq_mask_rx_only) &
	       READ_ONCE(vector->data_irq_mask_tx_only);
}

void rtlmq_sync_tx_irq_arming(struct rtlmq_private *tp)
{
	unsigned int i;

	if (!tp || !tp->vectors)
		return;

	for (i = 0; i < tp->allocated_vectors; i++) {
		rtlmq_sync_vector_tx_irq_arming(&tp->vectors[i],
						rtlmq_vector_tx_irq_should_arm(&tp->vectors[i]));
	}
}

void rtlmq_sync_tx_ring_irq_arming(struct rtlmq_tx_ring *ring)
{
	struct rtlmq_vector *vector;
	struct rtlmq_private *tp;
	u16 vector_index;
	bool tx_armed;

	if (!ring)
		return;

	tp = ring->priv;
	if (!tp || !tp->vectors)
		return;

	vector_index = READ_ONCE(ring->vector_index);
	if (vector_index >= tp->allocated_vectors)
		return;

	vector = &tp->vectors[vector_index];
	if (!rtlmq_coupled_tx_irq_suppression_active(tp) &&
	    !rtlmq_vector_has_shared_data_sources(vector))
		return;
	tx_armed = rtlmq_vector_tx_irq_should_arm(vector);
	rtlmq_sync_vector_tx_irq_arming(vector, tx_armed);
}
