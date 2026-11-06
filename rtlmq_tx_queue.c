// SPDX-License-Identifier: GPL-2.0-only

#include <linux/jiffies.h>
#include <net/netdev_queues.h>

#include "rtlmq_runtime_accessors.h"
#include "rtlmq_irq.h"
#include "rtlmq.h"
#include "rtlmq_tx.h"
#include "rtlmq_tx_private.h"
#include "rtlmq_lifecycle.h"
#include "rtlmq_trace.h"

static unsigned int
rtlmq_tx_desc_avail_from_pending(const struct rtlmq_tx_ring *ring, u32 pending)
{
	unsigned int ring_count = rtlmq_tx_ring_rm(ring)->ring_count;

	if (pending >= ring_count)
		return 0;

	return ring_count - pending - 1;
}

static bool
rtlmq_tx_desc_avail_reaches_wake_desc_high(const struct rtlmq_tx_ring *ring,
					   unsigned int desc_avail)
{
	return desc_avail >= rtlmq_tx_wake_desc_high(ring);
}

/*
 * BQL owns STACK_XOFF.  A completion normally clears it through
 * netif_txq_completed_wake(), but the watchdog can race the final
 * completion and observe an empty driver ring before that wake reaches the
 * core.  Only repair the core bit when both driver accounting and the core
 * DQL agree that no TX work remains; never use this as a general queue wake.
 */
static bool rtlmq_tx_queue_bql_quiescent(const struct rtlmq_tx_ring *ring)
{
	const struct netdev_queue *txq;

	if (!ring)
		return false;

	txq = rtlmq_tx_ring_rm(ring)->ndev_queue;
	if (!txq ||
	    rtlmq_tx_bql_pending_packets(ring) ||
	    rtlmq_tx_bql_pending_bytes(ring))
		return false;

#ifdef CONFIG_BQL
	return READ_ONCE(txq->dql.num_queued) ==
	       READ_ONCE(txq->dql.num_completed);
#else
	return true;
#endif
}

static bool
rtlmq_tx_queue_empty_stack_xoff_repair_allowed(const struct rtlmq_tx_ring *ring,
					       bool stack_xoff)
{
	return stack_xoff && rtlmq_tx_ring_is_empty(ring) &&
	       rtlmq_tx_queue_bql_quiescent(ring);
}

static bool rtlmq_tx_subqueue_slot_inactive_for_stack_state(const struct rtlmq_tx_ring *ring)
{
	const struct rtlmq_tx_ring_read_mostly *rm = rtlmq_tx_ring_rm(ring);
	u16 published_txq = rtlmq_published_num_tx_queues(ring->priv);

	/*
	 * Use the published queue count, not the count being prepared for a
	 * rebuild. A zero published count makes every slot inactive.
	 */
	return rm->queue_index >= published_txq;
}

static enum rtlmq_tx_subqueue_state
rtlmq_tx_subqueue_target_state(const struct rtlmq_tx_ring *ring)
{
	const struct rtlmq_private *tp = ring->priv;
	const struct rtlmq_tx_ring_read_mostly *rm = rtlmq_tx_ring_rm(ring);
	const struct net_device *ndev = rm->ndev;
	u16 num_tx_queues = rtlmq_published_num_tx_queues(tp);
	unsigned int desc_avail;

	if (rm->queue_index >= num_tx_queues)
		return RTLMQ_TX_SUBQUEUE_DISABLED;

	if (!rm->desc || !rm->infos || !rm->completions || !rm->dma_txns)
		return RTLMQ_TX_SUBQUEUE_DISABLED;

	if (READ_ONCE(tp->state) != RTLMQ_STATE_OPEN)
		return RTLMQ_TX_SUBQUEUE_DISABLED;

	if (!netif_running(ndev) || !netif_device_present(ndev) ||
	    !netif_carrier_ok(ndev))
		return RTLMQ_TX_SUBQUEUE_DISABLED;

	/*
	 * tx_subqueue_state tracks device readiness and descriptor space.
	 * BQL controls STACK_XOFF independently.
	 */

	desc_avail = rtlmq_tx_desc_avail(ring);
	if (READ_ONCE(ring->tx_subqueue_state) ==
	    RTLMQ_TX_SUBQUEUE_BACKPRESSURED) {
		if (!rtlmq_tx_desc_avail_reaches_wake_desc_high(ring,
								desc_avail))
			return RTLMQ_TX_SUBQUEUE_BACKPRESSURED;
		return RTLMQ_TX_SUBQUEUE_READY;
	}

	if (desc_avail < rtlmq_tx_stop_desc_low(ring))
		return RTLMQ_TX_SUBQUEUE_BACKPRESSURED;

	return RTLMQ_TX_SUBQUEUE_READY;
}

bool rtlmq_tx_completion_wake_allowed(const struct rtlmq_tx_ring *ring)
{
	return rtlmq_tx_subqueue_target_state(ring) ==
	       RTLMQ_TX_SUBQUEUE_READY;
}

static void rtlmq_wake_netdev_tx_queue(struct rtlmq_tx_ring *ring)
{
	struct netdev_queue *ndev_queue = rtlmq_tx_ring_rm(ring)->ndev_queue;

	txq_trans_cond_update(ndev_queue);
	netif_tx_wake_queue(ndev_queue);
}

static void rtlmq_refresh_netdev_tx_queue(struct rtlmq_tx_ring *ring,
					  bool clear_stack_xoff)
{
	struct netdev_queue *ndev_queue = rtlmq_tx_ring_rm(ring)->ndev_queue;
	bool stack_xoff_cleared = false;

	if (!rtlmq_netstack_tx_queue_live(ring))
		return;

	clear_bit(__QUEUE_STATE_FROZEN, &ndev_queue->state);
	if (clear_stack_xoff) {
		/*
		 * Serialize the exceptional stack-XOFF repair with ndo_start_xmit.
		 * The ordinary BQL completion path already provides its own
		 * lockless ordering; this path is only for the empty-ring watchdog
		 * race and must not clear a newly asserted XOFF.
		 */
		__netif_tx_lock_bh(ndev_queue);
		if (rtlmq_tx_ring_is_empty(ring) &&
		    rtlmq_tx_queue_bql_quiescent(ring))
			stack_xoff_cleared =
				test_and_clear_bit(__QUEUE_STATE_STACK_XOFF,
						   &ndev_queue->state);
		__netif_tx_unlock_bh(ndev_queue);
	}
	rtlmq_wake_netdev_tx_queue(ring);
	/*
	 * netif_tx_wake_queue() only schedules qdisc work when it clears the
	 * driver XOFF bit. A BQL-only STACK_XOFF repair therefore needs the
	 * explicit schedule after the queue is fully runnable.
	 */
	if (stack_xoff_cleared)
		netif_schedule_queue(ndev_queue);
}

static void rtlmq_reset_inactive_netdev_tx_queue(struct rtlmq_tx_ring *ring)
{
	struct netdev_queue *ndev_queue = rtlmq_tx_ring_rm(ring)->ndev_queue;

	if (!rtlmq_netstack_tx_queue_live(ring))
		return;

	/*
	 * Reset BQL only for inactive queues. Resetting it on an ordinary wake
	 * would discard the learned DQL limit.
	 */
	clear_bit(__QUEUE_STATE_FROZEN, &ndev_queue->state);
	netdev_tx_reset_queue(ndev_queue);
	rtlmq_wake_netdev_tx_queue(ring);
}

static bool rtlmq_netdev_tx_queue_driver_blocked(struct netdev_queue *txq)
{
	unsigned long state;

	if (!txq)
		return false;

	state = READ_ONCE(txq->state);
	return !!(state & (BIT(__QUEUE_STATE_DRV_XOFF) |
			  BIT(__QUEUE_STATE_FROZEN)));
}

static void rtlmq_stop_netdev_tx_queue(struct rtlmq_tx_ring *ring)
{
	struct netdev_queue *ndev_queue = rtlmq_tx_ring_rm(ring)->ndev_queue;

	if (!rtlmq_netstack_tx_queue_live(ring))
		return;

	netif_tx_stop_queue(ndev_queue);
}

int rtlmq_tx_maybe_stop_subqueue(struct rtlmq_tx_ring *ring,
				 unsigned int stop_threshold)
{
	struct netdev_queue *txq = rtlmq_tx_ring_rm(ring)->ndev_queue;
	unsigned int start_threshold;
	int result;

	if (unlikely(!txq || !rtlmq_netstack_tx_queue_live(ring)))
		return 0;

	start_threshold = max_t(unsigned int, stop_threshold,
				rtlmq_tx_wake_desc_high(ring));
	/*
	 * The core helper pairs the stop with completion's wake and rechecks
	 * descriptor availability to avoid a lost wakeup.
	 */
	result = netif_txq_maybe_stop(txq, rtlmq_tx_desc_avail(ring),
				      stop_threshold, start_threshold);
	return result;
}

static __always_inline void
rtlmq_account_tx_subqueue_transition(struct rtlmq_tx_ring *ring,
				     enum rtlmq_tx_subqueue_state state)
{
	if (state == RTLMQ_TX_SUBQUEUE_READY)
		atomic64_inc(&ring->stats.queue_wake);
	else
		atomic64_inc(&ring->stats.queue_stopped);
}

void rtlmq_publish_tx_datapath_subqueue_state(struct rtlmq_tx_ring *ring,
					      enum rtlmq_tx_subqueue_state state)
{
	enum rtlmq_tx_subqueue_state old_state =
		READ_ONCE(ring->tx_subqueue_state);
	bool inactive =
		rtlmq_tx_subqueue_slot_inactive_for_stack_state(ring);

	if (old_state != state) {
		WRITE_ONCE(ring->tx_subqueue_state, state);
		if (!inactive)
			rtlmq_account_tx_subqueue_transition(ring, state);
	}

	/*
	 * Update driver state and IRQ arming here. The transmit path stops the
	 * core queue and completion wakes it; lifecycle repair is separate.
	 */
	rtlmq_sync_tx_ring_irq_arming(ring);
}

void rtlmq_sync_tx_datapath_subqueue(struct rtlmq_tx_ring *ring)
{
	enum rtlmq_tx_subqueue_state state =
		rtlmq_tx_subqueue_target_state(ring);

	rtlmq_publish_tx_datapath_subqueue_state(ring, state);
}

void rtlmq_publish_tx_subqueue_state(struct rtlmq_tx_ring *ring,
				     enum rtlmq_tx_subqueue_state state)
{
	const struct rtlmq_tx_ring_read_mostly *rm = rtlmq_tx_ring_rm(ring);
	struct netdev_queue *ndev_queue = rm->ndev_queue;
	enum rtlmq_tx_subqueue_state old_state;
	bool xmit_frozen = false;
	bool stack_xoff = false;
	bool netstack_txq_live;
	bool inactive;

	inactive = rtlmq_tx_subqueue_slot_inactive_for_stack_state(ring);
	old_state = READ_ONCE(ring->tx_subqueue_state);
	netstack_txq_live = rtlmq_netstack_tx_queue_live(ring);
	if (netstack_txq_live) {
		/*
		 * Reset/channel changes can leave a core TX queue frozen.
		 * Clear that bit before applying the new queue state.
		 */
		xmit_frozen = test_bit(__QUEUE_STATE_FROZEN,
				       &ndev_queue->state);
		if (xmit_frozen)
			clear_bit(__QUEUE_STATE_FROZEN, &ndev_queue->state);
		stack_xoff = rtlmq_netdev_tx_queue_stack_xoff(ndev_queue);
	}

	if (old_state == state && !inactive) {
		if (netstack_txq_live) {
			if (state == RTLMQ_TX_SUBQUEUE_READY) {
				if (rtlmq_tx_queue_empty_stack_xoff_repair_allowed(ring,
										   stack_xoff)) {
					rtlmq_refresh_netdev_tx_queue(ring, true);
				} else if (rtlmq_netdev_tx_queue_driver_blocked(ndev_queue) &&
					   rtlmq_tx_desc_avail_reaches_wake_desc_high(
						   ring, rtlmq_tx_desc_avail(ring))) {
					rtlmq_refresh_netdev_tx_queue(ring, false);
				}
			} else if (xmit_frozen ||
				   (!stack_xoff &&
				    !netif_tx_queue_stopped(ndev_queue))) {
				rtlmq_stop_netdev_tx_queue(ring);
			}
		}
		rtlmq_sync_tx_ring_irq_arming(ring);
		return;
	}

	WRITE_ONCE(ring->tx_subqueue_state, state);
	if (!inactive && old_state != state)
		rtlmq_account_tx_subqueue_transition(ring, state);

	if (!netstack_txq_live) {
		rtlmq_sync_tx_ring_irq_arming(ring);
		return;
	}

	if (inactive) {
		/*
		 * Withdrawing stack visibility does not revoke NAPI's retained
		 * completion maps. Preserve DQL debt until that generation is
		 * quiesced; a late completion still has to balance its charge.
		 */
		if (!rtlmq_release_allowed(ring->priv)) {
			if (rtlmq_tx_ring_is_empty(ring) &&
			    rtlmq_tx_queue_bql_quiescent(ring))
				rtlmq_refresh_netdev_tx_queue(ring, false);
			else
				rtlmq_stop_netdev_tx_queue(ring);
			rtlmq_sync_tx_ring_irq_arming(ring);
			return;
		}
		/*
		 * dev_watchdog() scans every allocated netdev_queue, not only
		 * the currently published real_num_tx_queues. Keep inactive
		 * queue slots awake so they stay watchdog-inert while
		 * core queue selection and real queue-count publication keep
		 * all live traffic on the active subset.
		 */
		rtlmq_reset_inactive_netdev_tx_queue(ring);
		rtlmq_sync_tx_ring_irq_arming(ring);
		return;
	}

	if (state == RTLMQ_TX_SUBQUEUE_READY) {
		/*
		 * Refresh trans_start when activating a queue, even if it is
		 * already awake. An unused slot may still have its registration
		 * timestamp, causing an immediate timeout when BQL stops it.
		 *
		 * Completion may already have woken the queue through the core
		 * completed-wake helper. The explicit wake here repairs queue
		 * state after lifecycle changes.
		 */
		txq_trans_cond_update(ndev_queue);
		if (netif_xmit_frozen_or_stopped(ndev_queue)) {
			if (rtlmq_tx_ring_is_empty(ring))
				rtlmq_refresh_netdev_tx_queue(ring, true);
			else
				rtlmq_refresh_netdev_tx_queue(ring, false);
		}
	} else {
		rtlmq_stop_netdev_tx_queue(ring);
	}

	rtlmq_sync_tx_ring_irq_arming(ring);
}

void rtlmq_sync_tx_subqueue(struct rtlmq_tx_ring *ring)
{
	enum rtlmq_tx_subqueue_state state;

	state = rtlmq_tx_subqueue_target_state(ring);
	rtlmq_publish_tx_subqueue_state(ring, state);
}

void rtlmq_sync_tx_subqueue_after_clean(struct rtlmq_tx_ring *ring,
					unsigned int descs_cleaned,
					u32 pending_before, u32 pending_after)
{
	struct netdev_queue *txq = rtlmq_tx_ring_rm(ring)->ndev_queue;
	enum rtlmq_tx_subqueue_state current_state;
	enum rtlmq_tx_subqueue_state state;
	bool threshold_crossed = false;
	unsigned int avail_after = 0;
	unsigned long txq_state = 0;
	bool driver_blocked = false;

	if (!descs_cleaned)
		return;

	current_state = READ_ONCE(ring->tx_subqueue_state);
	if (txq && rtlmq_netstack_tx_queue_live(ring)) {
		txq_state = READ_ONCE(txq->state);
		driver_blocked =
			!!(txq_state & (BIT(__QUEUE_STATE_DRV_XOFF) |
					BIT(__QUEUE_STATE_FROZEN)));
	}
	if (current_state != RTLMQ_TX_SUBQUEUE_READY || driver_blocked) {
		unsigned int tx_wake_desc_high = rtlmq_tx_wake_desc_high(ring);
		unsigned int avail_before =
			rtlmq_tx_desc_avail_from_pending(ring, pending_before);

		avail_after = rtlmq_tx_desc_avail_from_pending(ring,
							       pending_after);

		threshold_crossed = avail_before < tx_wake_desc_high &&
				    rtlmq_tx_desc_avail_reaches_wake_desc_high(ring, avail_after);
	}

	if (!driver_blocked &&
	    current_state == RTLMQ_TX_SUBQUEUE_READY &&
	    !threshold_crossed)
		return;

	state = rtlmq_tx_subqueue_target_state(ring);
	if (driver_blocked && state == RTLMQ_TX_SUBQUEUE_READY &&
	    !rtlmq_tx_desc_avail_reaches_wake_desc_high(ring, rtlmq_tx_desc_avail(ring)))
		return;
	if (!driver_blocked && state == current_state)
		return;

	rtlmq_publish_tx_datapath_subqueue_state(ring, state);
}

void rtlmq_sync_tx_subqueues(struct rtlmq_private *tp)
{
	u16 tx_slots = rtlmq_owned_tx_ring_slots(tp);
	unsigned int i;

	if (!tp->tx_rings)
		return;

	for (i = 0; i < tx_slots; i++)
		rtlmq_sync_tx_subqueue(&tp->tx_rings[i]);
}
