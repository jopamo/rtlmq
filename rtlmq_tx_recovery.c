// SPDX-License-Identifier: GPL-2.0-only

#include <linux/bitops.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/rcupdate.h>
#include <linux/string.h>
#include <linux/workqueue.h>

#include "rtlmq_diag.h"
#include "rtlmq_irq.h"
#include "rtlmq_lifecycle.h"
#include "rtlmq.h"
#include "rtlmq_regs.h"
#include "rtlmq_ring.h"
#include "rtlmq_runtime.h"
#include "rtlmq_runtime_accessors.h"
#include "rtlmq_tx.h"
#include "rtlmq_tx_internal.h"
#include "rtlmq_tx_private.h"
#include "rtlmq_trace.h"

#define RTLMQ_TX_TIMEOUT_RESET_DELAY	4

static unsigned int
rtlmq_tx_timeout_addressable_queue_count(const struct rtlmq_private *tp);
static void
rtlmq_take_tx_timeout_watchdog_snapshot(struct rtlmq_tx_ring *ring,
					struct rtlmq_tx_timeout_snapshot *snap);
static void
rtlmq_take_tx_timeout_snapshot(struct rtlmq_tx_ring *ring, struct rtlmq_tx_timeout_snapshot *snap);

void rtlmq_tx_ring_cancel_liveness(struct rtlmq_tx_ring *ring)
{
	if (!ring)
		return;

	/*
	 * Probe failure may reach teardown before INIT_DELAYED_WORK().
	 */
	if (!READ_ONCE(ring->cold.liveness_work_initialized))
		return;

	cancel_delayed_work_sync(&ring->cold.tx_liveness_work);
	WRITE_ONCE(ring->cold.liveness_work_generation, 0);
}

void rtlmq_tx_liveness_work_cb(struct work_struct *work)
{
	struct rtlmq_tx_ring_cold *cold =
		container_of(work, struct rtlmq_tx_ring_cold,
			     tx_liveness_work.work);
	struct rtlmq_tx_ring *ring = container_of(cold, struct rtlmq_tx_ring,
						  cold);
	{
		rtlmq_serial_tx_work(ring);
		return;
	}
}

static bool rtlmq_tx_timeout_force_reclaim_eligible(const struct rtlmq_private *tp,
						    const struct rtlmq_tx_ring *ring,
						    const struct rtlmq_tx_timeout_snapshot *snap)
{
	const struct rtlmq_vector *vector;
	unsigned int i;

	if (!tp || !ring || !snap)
		return false;

	if (!snap->pending || !snap->generation_stable)
		return false;

	if (!tp->vectors ||
	    snap->ring_vector_index >= READ_ONCE(tp->allocated_vectors))
		return false;

	vector = &tp->vectors[snap->ring_vector_index];
	if (!READ_ONCE(vector->napi_enabled) ||
	    !READ_ONCE(vector->irq_requested))
		return false;

	for (i = 0; i < READ_ONCE(vector->num_tx_rings) &&
	     i < ARRAY_SIZE(vector->tx_ring_indices);
	     i++) {
		if (READ_ONCE(vector->tx_ring_indices[i]) ==
		    ring->queue_index)
			return true;
	}

	return false;
}

void rtlmq_tx_timeout(struct net_device *ndev, unsigned int txqueue)
{
	struct rtlmq_private *tp = netdev_priv(ndev);
	struct rtlmq_tx_timeout_snapshot snap = {};
	bool forced_resweep = false;
	struct rtlmq_tx_ring *ring;
	u32 epoch;

	/*
	 * Snapshot the TX state, then queue bounded reset validation. Do not try
	 * to repair the ring in-place from ndo_tx_timeout(), and do not detach
	 * the device here. The actual rebuild stays in reset_work under
	 * rtnl+control_lock.
	 */
	/*
	 * The watchdog runs without control_lock. Hold RCU while looking up
	 * and reading a published ring. Revocation blocks new readers before
	 * live replacement or teardown drains existing readers.
	 */
	rcu_read_lock();
	if (unlikely(!rtlmq_published_runtime_config(tp))) {
		rcu_read_unlock();
		return;
	}
	if (unlikely(!tp->tx_rings ||
		     txqueue >= rtlmq_tx_timeout_addressable_queue_count(tp))) {
		netdev_warn(ndev,
			    "ignoring TX timeout for invalid queue %u (owned slots %u)\n",
			    txqueue,
			    rtlmq_tx_timeout_addressable_queue_count(tp));
		rcu_read_unlock();
		return;
	}

	ring = &tp->tx_rings[txqueue];
	/*
	 * The watchdog freezes the netdev queues before entering this callback.
	 * Consume any deferred xmit_more publication now, including the path
	 * where reset_work later suppresses destructive recovery after progress.
	 */
	rtlmq_tx_flush_pending_ring(ring);

	/*
	 * Capture the fixed descriptor window without register reads.
	 * Reset work formats it later under RTNL and control_lock.
	 */
	epoch = READ_ONCE(tp->reset_epoch);
	rtlmq_take_tx_timeout_watchdog_snapshot(ring, &snap);

	if (!snap.pending) {
		/*
		 * The watchdog can observe a stale trans_start after the final
		 * completion has already reclaimed the ring. There is no hardware
		 * work to reset in that state; repair queue state instead.
		 */
		rtlmq_sync_tx_subqueue(ring);
		rtlmq_sync_tx_irq_arming(tp);
		netdev_warn(ndev,
			    "TX timeout q%u suppressed: ring empty pending=0\n",
			    txqueue);
		rcu_read_unlock();
		return;
	}

	if (rtlmq_tx_timeout_force_reclaim_eligible(tp, ring, &snap)) {
		/*
		 * First schedule NAPI to reclaim missed completions. Delayed
		 * reset validation checks for progress before rebuilding the
		 * device under RTNL and control_lock.
		 */
		rtlmq_force_vector_tx_resweep(tp, snap.ring_vector_index);
		forced_resweep = true;
	}

	netdev_warn(ndev,
		    "TX timeout q%u pending=%u generation=%u recovery=%s; reset validation in %u jiffies\n",
		    txqueue, snap.pending, snap.ring_generation,
		    forced_resweep ? "forced_napi_resweep" : "deferred",
		    RTLMQ_TX_TIMEOUT_RESET_DELAY);
	/*
	 * Give an already-raised completion interrupt/NAPI run a few jiffies
	 * before reset_work decides whether the device needs rebuilding.
	 */
	rtlmq_request_tx_timeout_validation(tp, &snap, epoch, RTLMQ_TX_TIMEOUT_RESET_DELAY);
	rcu_read_unlock();
}

/* Timeout snapshots. */

static unsigned int
rtlmq_tx_timeout_addressable_queue_count(const struct rtlmq_private *tp)
{
	if (!tp || !tp->tx_rings)
		return 0;

	/*
	 * dev_watchdog() can still target netdev_queue slots outside the
	 * current published real_num_tx_queues. Keep timeout diagnostics and
	 * snapshots addressable across all allocated TX ring slots, including
	 * queues deactivated by a channel change.
	 */
	return rtlmq_owned_tx_ring_slots(tp);
}

static void
rtlmq_take_tx_timeout_snapshot(struct rtlmq_tx_ring *ring, struct rtlmq_tx_timeout_snapshot *snap)
{
	unsigned int attempt;

	memset(snap, 0, sizeof(*snap));
	if (!ring)
		return;

	for (attempt = 0; attempt < 3; attempt++) {
		u32 generation;
		u64 use_count;

		memset(snap, 0, sizeof(*snap));
		snap->ring_vector_index = 0xffff;
		/* Pairs with rtlmq_tx_ring_publish_generation() before slot reuse. */
		generation = smp_load_acquire(&ring->generation);

		snap->queue = ring->queue_index;
		use_count = READ_ONCE(ring->next_to_use_count);
		snap->clean_count = READ_ONCE(ring->next_to_clean_count);
		snap->pending = use_count - snap->clean_count;
		snap->ring_generation = generation;
		snap->ring_vector_index = READ_ONCE(ring->vector_index);

		/* Pairs with rtlmq_tx_ring_publish_generation() before slot reuse. */
		if (likely(generation == smp_load_acquire(&ring->generation))) {
			snap->generation_stable = true;
			return;
		}
	}

	/* Pairs with rtlmq_tx_ring_publish_generation() before slot reuse. */
	snap->ring_generation = smp_load_acquire(&ring->generation);
}

static void
rtlmq_take_tx_timeout_watchdog_snapshot(struct rtlmq_tx_ring *ring,
					struct rtlmq_tx_timeout_snapshot *snap)
{
	/*
	 * ndo_tx_timeout() runs from the networking watchdog timer while
	 * net-core holds its global TX lock. Capture one bounded software
	 * snapshot, including only the fixed descriptor window at the consumer
	 * cursor. Register reads and verbose formatting belong to reset_work.
	 */
	rtlmq_take_tx_timeout_snapshot(ring, snap);
}

void rtlmq_store_tx_timeout_snapshot_locked(struct rtlmq_private *tp,
					    const struct rtlmq_tx_timeout_snapshot *snap)
{
	lockdep_assert_held(&tp->state_lock);
	tp->last_tx_timeout = *snap;
	tp->last_tx_timeout_valid = true;
}

bool rtlmq_load_tx_timeout_snapshot_locked(struct rtlmq_private *tp,
					   struct rtlmq_tx_timeout_snapshot *snap)
{
	lockdep_assert_held(&tp->state_lock);
	if (!tp->last_tx_timeout_valid)
		return false;
	*snap = tp->last_tx_timeout;
	return true;
}

/* Timeout reset suppression. */

bool rtlmq_tx_timeout_maybe_suppress_reset(struct rtlmq_private *tp, u32 reset_reasons,
					   unsigned int timeout_events,
					   const struct rtlmq_tx_timeout_snapshot *timeout)
{
	struct rtlmq_tx_timeout_snapshot current_snap;
	struct rtlmq_vector *vector;
	struct rtlmq_tx_ring *ring;
	bool generation_change;
	bool clean_progress;
	bool vector_change;

	if (!tp || !tp->ndev ||
	    reset_reasons != BIT(RTLMQ_RESET_REASON_TX_TIMEOUT) ||
	    !tp->tx_rings)
		return false;

	/* A coalesced watchdog event cannot reopen revoked publication authority. */
	if (rtlmq_serial_tx_recovering(tp))
		return false;

	if (!timeout)
		return false;

	if (timeout->queue >= rtlmq_tx_timeout_addressable_queue_count(tp))
		return false;

	ring = &tp->tx_rings[timeout->queue];
	rtlmq_take_tx_timeout_snapshot(ring, &current_snap);
	clean_progress = current_snap.clean_count != timeout->clean_count;
	generation_change =
		current_snap.ring_generation != timeout->ring_generation;
	vector_change =
		current_snap.ring_vector_index != timeout->ring_vector_index;
	vector = tp->vectors &&
		 current_snap.ring_vector_index < tp->allocated_vectors ?
		 &tp->vectors[current_snap.ring_vector_index] : NULL;

	if (timeout_events == 1 &&
	    timeout->generation_stable && current_snap.generation_stable &&
	    !vector_change && !generation_change &&
	    vector && READ_ONCE(vector->napi_enabled) &&
	    READ_ONCE(vector->irq_requested) &&
	    clean_progress) {
		rtlmq_force_vector_tx_resweep(tp, current_snap.ring_vector_index);
		rtlmq_sync_tx_subqueue(ring);
		rtlmq_sync_tx_irq_arming(tp);

		netdev_warn(tp->ndev,
			    "tx_timeout q%u reset suppressed after descriptor cleanup pending=%u->%u clean=%llu->%llu\n",
			    timeout->queue,
			    timeout->pending, current_snap.pending,
			    (unsigned long long)timeout->clean_count,
			    (unsigned long long)current_snap.clean_count);
		return true;
	}

	netdev_warn(tp->ndev,
		    "tx_timeout q%u reset escalated events=%u pending=%u->%u clean=%llu->%llu\n",
		    timeout->queue,
		    timeout_events,
		    timeout->pending, current_snap.pending,
		    (unsigned long long)timeout->clean_count,
		    (unsigned long long)current_snap.clean_count);
	return false;
}
