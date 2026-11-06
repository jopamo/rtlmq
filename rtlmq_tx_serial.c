// SPDX-License-Identifier: GPL-2.0-only

#include <linux/ktime.h>
#include <linux/minmax.h>
#include <linux/prefetch.h>
#include <linux/workqueue.h>

#include "rtlmq.h"
#include "rtlmq_irq.h"
#include "rtlmq_lifecycle.h"
#include "rtlmq_runtime_accessors.h"
#include "rtlmq_runtime.h"
#include "rtlmq_tx.h"
#include "rtlmq_tx_private.h"
#include "rtlmq_trace.h"

/* Peer selection uses q ^ 1; a wider topology needs a different arbiter. */
static_assert(RTLMQ_IMPL_MAX_TX_QUEUES == 2);

bool rtlmq_serial_tx_window_valid(u32 dirty, u32 hw, u32 commit, u32 stage,
				  u32 size)
{
	u32 frozen = rtlmq_tx_ring_distance(commit, dirty);
	u32 published = rtlmq_tx_ring_distance(hw, dirty);
	u32 total = rtlmq_tx_ring_distance(stage, dirty);

	return size && total < size && published <= total &&
	       frozen >= published && frozen <= total;
}

bool rtlmq_tx_epoch_should_yield(const struct rtlmq_serial_tx_epoch *epoch,
				 const struct rtlmq_serial_tx_policy *policy)
{
	return (policy->descs && epoch->descs >= policy->descs) ||
	       (policy->packets && epoch->packets >= policy->packets);
}

/*
 * Publication: target queue lock, ring-local permit.
 * Transitions: shared TX NAPI, arbiter lock -> one target queue lock.
 * Diagnostics: arbiter -> q0 -> q1 (nested), only for explicit snapshots.
 * Lifecycle: arbiter -> each queue to pause/revoke, then cancel work unlocked.
 *
 * No producer acquires the arbiter lock. Once its permit becomes NONE, a
 * producer cannot expose OWN or issue TX_POLL. Work only requests NAPI.
 */
static struct rtlmq_tx_ring *rtlmq_serial_owner(struct rtlmq_private *tp)
{
	struct rtlmq_tx_ring *ring;

	if (!tp || !tp->tx_rings || !rtlmq_owned_tx_ring_slots(tp))
		return NULL;
	ring = &tp->tx_rings[0];
	if (!ring->cold.serial.initialized)
		return NULL;
	return ring;
}

bool rtlmq_serial_tx_uses_cbs(struct rtlmq_private *tp)
{
	struct rtlmq_tx_ring *owner = rtlmq_serial_owner(tp);

	/* First/last CBS admission rebuilds rings with NAPI quiesced. */
	return owner && owner->cold.serial.policy.cbs;
}

static u64 rtlmq_serial_now(void)
{
	return ktime_get_ns();
}

static bool rtlmq_serial_pending(const struct rtlmq_tx_ring *ring)
{
	/* Acquire the timestamp/frontier published by the empty -> pending edge. */
	return smp_load_acquire(&ring->serial_control.pending);
}

void rtlmq_serial_tx_init(struct rtlmq_tx_ring *ring)
{
	struct rtlmq_serial_tx_arbiter *s = &ring->cold.serial;
	struct rtlmq_serial_tx_control *c = &ring->serial_control;
	bool cbs = rtlmq_tc_cbs_enabled(&ring->priv->tc.requested);

	memset(&ring->serial, 0, sizeof(ring->serial));
	memset(c, 0, sizeof(*c));
	memset(s, 0, sizeof(*s));
	spin_lock_init(&s->lock);
	c->paused = true;
	c->stage_high = ring->ring_count * 3 / 4;
	c->stage_low = ring->ring_count / 2;
	c->packet_high = ring->ring_count / 2;
	c->packet_low = ring->ring_count / 4;
	s->paused = true;
	s->initialized = true;
	s->active = RTLMQ_SERIAL_NO_QUEUE;
	s->snapshot.active = RTLMQ_SERIAL_NO_QUEUE;
	s->snapshot.paused = true;
	s->snapshot.last_publication = RTLMQ_SERIAL_NO_QUEUE;
	s->snapshot.last_doorbell = RTLMQ_SERIAL_NO_QUEUE;
	s->policy = (struct rtlmq_serial_tx_policy){
		/* Hardware CBS uses one-packet turns. */
		.descs = cbs ? RTLMQ_SERIAL_CBS_QUOTA_DESCS : RTLMQ_SERIAL_EPOCH_DESCS,
		.packets = RTLMQ_SERIAL_EPOCH_PACKETS,
		.runway = cbs ? RTLMQ_SERIAL_CBS_QUOTA_DESCS : RTLMQ_SERIAL_RUNWAY_DESCS,
		.cbs = cbs,
	};
	s->policy.runway = min_t(u32, s->policy.runway,
				 ring->ring_count ? ring->ring_count - 1 : 0);
}

/* Queue lock protects requests against lifecycle pause, not the global lock. */
void rtlmq_serial_tx_request(struct rtlmq_tx_ring *ring)
{
	lockdep_assert_held(&ring->txq->_xmit_lock);
	if (ring->serial_control.paused ||
	    READ_ONCE(ring->priv->state) != RTLMQ_STATE_OPEN)
		return;
	rtlmq_force_vector_tx_resweep(ring->priv, ring->vector_index);
}

static void rtlmq_serial_arm_at(struct rtlmq_tx_ring *ring, unsigned long at)
{
	struct rtlmq_serial_tx_control *c = &ring->serial_control;
	unsigned long delay;

	lockdep_assert_held(&ring->txq->_xmit_lock);
	if (c->paused || (c->timer_armed && time_before_eq(c->timer_at, at)))
		return;
	c->timer_at = at;
	c->timer_armed = true;
	ring->cold.liveness_work_generation = c->generation;
	delay = time_after(at, jiffies) ? at - jiffies : 0;
	mod_delayed_work(system_dfl_wq, &ring->cold.tx_liveness_work, delay);
}

void rtlmq_serial_tx_watch(struct rtlmq_tx_ring *ring)
{
	struct rtlmq_serial_tx_control *c = &ring->serial_control;

	lockdep_assert_held(&ring->txq->_xmit_lock);
	if (c->watching || c->paused)
		return;
	c->watching = true;
	c->watch_dirty = READ_ONCE(ring->next_to_clean_count);
	c->watch_at = jiffies;
	rtlmq_serial_arm_at(ring, c->watch_at + msecs_to_jiffies(RTLMQ_SERIAL_REKICK_MS));
}

void rtlmq_serial_tx_stage_pending(struct rtlmq_tx_ring *ring)
{
	struct rtlmq_serial_tx_control *c = &ring->serial_control;

	lockdep_assert_held(&ring->txq->_xmit_lock);
	if (c->pending)
		return;
	/* One clock and one request on empty -> pending, not on every acceptance. */
	c->pending_since = ktime_get_ns();
	c->fairness_at = jiffies + msecs_to_jiffies(RTLMQ_SERIAL_FAIRNESS_MS);
	/* NAPI acquires pending before reading its first-wait timestamp. */
	smp_store_release(&c->pending, true);
	rtlmq_serial_arm_at(ring, c->fairness_at);
	rtlmq_serial_tx_request(ring);
}

void rtlmq_serial_tx_work(struct rtlmq_tx_ring *ring)
{
	struct rtlmq_serial_tx_control *c = &ring->serial_control;
	u32 generation;

	__netif_tx_lock_bh(ring->txq);
	/* Pairs with quiesced ring-generation publication before slot reuse. */
	generation = smp_load_acquire(&ring->generation);
	/*
	 * An older callback can be waiting for this lock while NAPI replaces
	 * its deadline. Do not consume that newer, still-future one-shot.
	 * Generation acquire pairs with quiesced ring-generation publication.
	 */
	if (!c->paused && c->timer_armed &&
	    time_after_eq(jiffies, c->timer_at) &&
	    ring->cold.liveness_work_generation == c->generation &&
	    c->generation == generation) {
		c->timer_armed = false;
		c->timer_due = true;
		rtlmq_serial_tx_request(ring);
	}
	__netif_tx_unlock_bh(ring->txq);
}

static void rtlmq_serial_cancel_timer(struct rtlmq_tx_ring *ring)
{
	struct rtlmq_serial_tx_control *c = &ring->serial_control;

	c->timer_armed = false;
	c->timer_due = false;
	c->watching = false;
	cancel_delayed_work(&ring->cold.tx_liveness_work);
}

static bool rtlmq_serial_window_valid(struct rtlmq_tx_ring *ring)
{
	/* Pair with rtlmq_tx_ring_publish_generation() before checking the window. */
	u32 generation = smp_load_acquire(&ring->generation);

	return ring->serial_control.generation == generation &&
	       rtlmq_serial_tx_window_valid(READ_ONCE(ring->next_to_clean_count),
					    ring->next_to_use_count, ring->serial.commit,
					    ring->serial.stage, ring->ring_count);
}

static void rtlmq_tx_epoch_begin(struct rtlmq_tx_ring *ring)
{
	struct rtlmq_serial_tx_arbiter *s =
		&ring->priv->tx_rings[0].cold.serial;

	ring->serial.epoch = (struct rtlmq_serial_tx_epoch){
	};
	s->snapshot.epoch++;
}

static void rtlmq_tx_epoch_end(struct rtlmq_tx_ring *ring)
{
	struct rtlmq_serial_tx_arbiter *s =
		&ring->priv->tx_rings[0].cold.serial;

	s->snapshot.usage = ring->serial.epoch;
}

static bool rtlmq_serial_packet_valid(struct rtlmq_tx_ring *ring, u32 first,
				      u32 available)
{
	const struct rtlmq_tx_info *info =
		&ring->infos[first & ring->ring_mask];
	const struct rtlmq_serial_tx_packet *packet = &info->packet;
	u32 last = (first + packet->descs - 1) & ring->ring_mask;

	return packet->descs && packet->descs <= available &&
	       packet->queue == ring->queue_index &&
	       info->generation == ring->serial_control.generation &&
	       (ring->infos[last].flags & RTLMQ_TX_INFOF_EOP);
}

static void rtlmq_serial_wake_staged(struct rtlmq_tx_ring *ring)
{
	/* NAPI, once per publication batch, under the same lock as try-stop. */
	if (READ_ONCE(ring->tx_subqueue_state) !=
		    RTLMQ_TX_SUBQUEUE_BACKPRESSURED ||
	    !rtlmq_tx_staging_below_low(ring) ||
	    rtlmq_tx_desc_avail(ring) < rtlmq_tx_wake_desc_high(ring))
		return;
	rtlmq_publish_tx_datapath_subqueue_state(ring, RTLMQ_TX_SUBQUEUE_READY);
	if (rtlmq_netstack_tx_queue_live(ring) &&
	    netif_tx_queue_stopped(ring->txq)) {
		netif_tx_wake_queue(ring->txq);
	}
}

/*
 * Frozen prefix is packet aligned by construction. A packet bigger than the
 * runway may run alone; no other packet can make a nonempty runway exceed its
 * target. Never split a packet for either the runway or the epoch quantum.
 */
static bool
rtlmq_tx_publish_committed_batch(struct rtlmq_tx_ring *ring,
				 const struct rtlmq_serial_tx_policy *policy,
				 bool limited)
{
	u32 hw = ring->next_to_use_count;
	u32 first = hw;
	u32 flight = rtlmq_tx_published_count(ring);
	struct rtlmq_serial_tx_epoch epoch = ring->serial.epoch;
	u32 packets = 0;
	u32 planned;

	lockdep_assert_held(&ring->txq->_xmit_lock);
	/* The queue lock freezes a fully constructed, packet-aligned prefix. */
	ring->serial.commit = ring->serial.stage;
	while (hw != ring->serial.commit) {
		const struct rtlmq_serial_tx_packet *packet =
			&ring->infos[hw & ring->ring_mask].packet;
		u32 available = rtlmq_tx_ring_distance(ring->serial.commit, hw);

		if (!rtlmq_serial_packet_valid(ring, hw, available))
			return false;
		if (limited &&
		    (rtlmq_tx_epoch_should_yield(&epoch, policy) ||
		     (flight && (flight >= policy->runway ||
				 packet->descs > policy->runway - flight))))
			break;
		rtlmq_tx_epoch_account_packet(&epoch, packet);
		hw += packet->descs;
		flight += packet->descs;
	}
	/*
	 * Finish all fallible boundary validation before exposing any OWN.
	 */
	planned = hw;
	epoch = ring->serial.epoch;
	for (hw = first; hw != planned;) {
		const struct rtlmq_serial_tx_packet *packet =
			&ring->infos[hw & ring->ring_mask].packet;

		prefetch(&ring->infos[(hw + packet->descs) & ring->ring_mask]);
		rtlmq_tx_epoch_account_packet(&epoch, packet);
		packets++;
		rtlmq_tx_publish_packet_own(ring, hw, packet->descs);
		hw += packet->descs;
	}
	if (hw != first) {
		ring->serial.epoch = epoch;
		ring->serial.hw_packets += packets;
		WRITE_ONCE(ring->next_to_use, hw & ring->ring_mask);
		/* Reclaim acquires hw only after the entire batch is DMA-published. */
		smp_store_release(&ring->next_to_use_count, hw);
		if (trace_rtlmq_tx_publish_range_enabled())
			trace_rtlmq_tx_publish_range(ring->priv->ndev->name, ring->queue_index,
						     READ_ONCE(ring->generation), first, hw,
						     packets);
		ring->tx_doorbell_pending = true;
		ring->priv->tx_rings[0].cold.serial.snapshot.last_publication =
			ring->queue_index;
		rtlmq_serial_tx_watch(ring);
		/*
		 * Reclaim may have disarmed TX before this refill or handoff.
		 * As on native publication, restore the completion IRQ request
		 * after publishing hw and before the kick. While NAPI is polling,
		 * the shared helper updates its eventual unmask, not hardware.
		 * Pair with the disarm barrier before testing its wanted flag;
		 * the frontier's release store alone does not order this load.
		 */
		smp_mb();
		if (!rtlmq_tx_ring_tx_irq_wanted(ring))
			rtlmq_sync_tx_ring_irq_arming(ring);
		if (!rtlmq_serial_tx_notify(ring))
			return false;
		ring->priv->tx_rings[0].cold.serial.snapshot.last_doorbell =
			ring->queue_index;
		rtlmq_serial_wake_staged(ring);
	}
	if (hw == ring->serial.stage) {
		/* Publish exhaustion before another empty -> pending event. */
		smp_store_release(&ring->serial_control.pending, false);
	}
	return true;
}

/*
 * dirty advances only after every published descriptor in a complete packet
 * has returned OWN. With authority revoked, dirty == hw therefore proves the
 * owned count is zero without a ring scan. Staged storage cannot set OWN.
 * No documented register proves hidden RTL8125B scheduler/fetch quiescence.
 */
static bool rtlmq_serial_hardware_idle(struct rtlmq_tx_ring *ring)
{
	return ring->serial_control.permit == RTLMQ_SERIAL_DRAINING &&
	       !rtlmq_tx_published_count(ring) && !ring->tx_doorbell_pending;
}

static bool rtlmq_serial_fairness_due(const struct rtlmq_serial_tx_control *c)
{
	u64 since, now;

	/* The coarse timer is a wake hint, not proof of elapsed waiting time. */
	if (time_before(jiffies, READ_ONCE(c->fairness_at)))
		return false;
	since = READ_ONCE(c->pending_since);
	now = rtlmq_serial_now();
	return now >= since &&
	       now - since >= RTLMQ_SERIAL_FAIRNESS_MS * NSEC_PER_MSEC;
}

static void rtlmq_serial_begin_drain(struct rtlmq_tx_ring *ring,
				     enum rtlmq_serial_tx_reason reason,
				     u64 now)
{
	struct rtlmq_serial_tx_arbiter *s =
		&ring->priv->tx_rings[0].cold.serial;
	struct rtlmq_serial_tx_control *c = &ring->serial_control;

	lockdep_assert_held(&ring->txq->_xmit_lock);
	/*
	 * The producer lock excludes even a producer which checked DIRECT
	 * before we arrived. After unlock no old producer can expose OWN.
	 * Deferred notification is still authorized until full drain.
	 */
	c->permit = RTLMQ_SERIAL_DRAINING;
	s->phase = RTLMQ_SERIAL_DRAIN;
	s->drain_start = now;
	s->snapshot.reason = reason;
	rtlmq_tx_epoch_end(ring);
	if (c->pending) {
		c->pending_since = now;
		c->fairness_at =
			jiffies + msecs_to_jiffies(RTLMQ_SERIAL_FAIRNESS_MS);
	}
	if (ring->tx_doorbell_pending)
		rtlmq_serial_tx_notify(ring);
	rtlmq_serial_arm_at(ring,
			    jiffies + msecs_to_jiffies(RTLMQ_SERIAL_DRAIN_MS));
}

static void rtlmq_serial_recover_locked(struct rtlmq_private *tp,
					enum rtlmq_serial_tx_reason reason)
{
	struct rtlmq_serial_tx_arbiter *s = &tp->tx_rings[0].cold.serial;
	unsigned int q;

	lockdep_assert_held(&s->lock);
	/* Revoke both permits, without ever granting concurrent authority. */
	for (q = 0; q < s->num_queues; q++) {
		struct rtlmq_tx_ring *ring = &tp->tx_rings[q];

		__netif_tx_lock_bh(ring->txq);
		ring->serial_control.permit = RTLMQ_SERIAL_NONE;
		ring->serial_control.resume_permit = RTLMQ_SERIAL_NONE;
		ring->serial_control.paused = true;
		rtlmq_serial_cancel_timer(ring);
		__netif_tx_unlock_bh(ring->txq);
	}
	s->phase = RTLMQ_SERIAL_RECOVERY;
	s->snapshot.reason = reason;
	if (reason == RTLMQ_SERIAL_TIMEOUT)
		s->snapshot.drain_timeouts++;
	else
		s->snapshot.invariant_failures++;
}

static bool rtlmq_serial_watch_service(struct rtlmq_tx_ring *ring)
{
	struct rtlmq_serial_tx_control *c = &ring->serial_control;
	u32 dirty = READ_ONCE(ring->next_to_clean_count);

	if (!c->timer_due)
		return true;
	c->timer_due = false;
	if (!rtlmq_tx_published_count(ring)) {
		c->watching = false;
		return true;
	}
	/* A one-shot samples progress; it does not permanently poll at 1 ms. */
	if (time_after_eq(jiffies,
			  c->watch_at +
				  msecs_to_jiffies(RTLMQ_SERIAL_DRAIN_MS))) {
		if (dirty == c->watch_dirty)
			return false;
		c->watch_dirty = dirty;
		c->watch_at = jiffies;
	}
	rtlmq_serial_tx_notify(ring);
	rtlmq_serial_arm_at(ring, c->watch_at + msecs_to_jiffies(RTLMQ_SERIAL_DRAIN_MS));
	return true;
}

static bool rtlmq_serial_step_idle_locked(struct rtlmq_private *tp)
{
	struct rtlmq_serial_tx_arbiter *s = &tp->tx_rings[0].cold.serial;
	unsigned int q;

	lockdep_assert_held(&s->lock);
	for (q = 0; q < s->num_queues; q++)
		if (rtlmq_serial_pending(&tp->tx_rings[q]))
			break;
	if (q == s->num_queues)
		return false;
	s->active = q;
	s->phase = RTLMQ_SERIAL_COMMIT;
	s->snapshot.reason = RTLMQ_SERIAL_FIRST;
	return true;
}

static void rtlmq_serial_step_active_locked(struct rtlmq_tx_ring *ring,
					    unsigned int other
)
{
	struct rtlmq_private *tp = ring->priv;
	struct rtlmq_serial_tx_arbiter *s = &tp->tx_rings[0].cold.serial;
	struct rtlmq_serial_tx_control *c = &ring->serial_control;

	lockdep_assert_held(&s->lock);
	lockdep_assert_held(&ring->txq->_xmit_lock);
	if (!s->dual_busy) {
		/* Previously deep native flight cannot be retroactively bounded. */
		c->permit = RTLMQ_SERIAL_LIMITED;
		s->dual_busy = true;
		rtlmq_tx_epoch_begin(ring);
	}
	if (!rtlmq_tx_published_count(ring) &&
	    !rtlmq_tx_unpublished_count(ring))
		rtlmq_serial_begin_drain(ring,
					 RTLMQ_SERIAL_EMPTY,
					 rtlmq_serial_now());
	else if (rtlmq_tx_epoch_should_yield(&ring->serial.epoch,
					     &s->policy))
		rtlmq_serial_begin_drain(ring,
					 RTLMQ_SERIAL_LIMIT,
					 rtlmq_serial_now());
	else if (rtlmq_serial_fairness_due(&tp->tx_rings[other].serial_control))
		rtlmq_serial_begin_drain(ring,
					 RTLMQ_SERIAL_FAIRNESS,
					 rtlmq_serial_now());
}

enum rtlmq_serial_drain_result {
	RTLMQ_SERIAL_DRAIN_WAIT,
	RTLMQ_SERIAL_DRAIN_READY,
	RTLMQ_SERIAL_DRAIN_TIMEOUT,
};

static enum rtlmq_serial_drain_result
rtlmq_serial_step_drain_locked(struct rtlmq_tx_ring *ring, bool other_pending)
{
	struct rtlmq_serial_tx_arbiter *s = &ring->priv->tx_rings[0].cold.serial;
	struct rtlmq_serial_tx_control *c = &ring->serial_control;
	u64 now;

	lockdep_assert_held(&s->lock);
	lockdep_assert_held(&ring->txq->_xmit_lock);
	if (ring->tx_doorbell_pending)
		rtlmq_serial_tx_notify(ring);
	if (!rtlmq_serial_hardware_idle(ring)) {
		now = rtlmq_serial_now();
		if (now - s->drain_start >=
		    RTLMQ_SERIAL_DRAIN_MS * NSEC_PER_MSEC)
			return RTLMQ_SERIAL_DRAIN_TIMEOUT;
		if (c->timer_due) {
			c->timer_due = false;
			rtlmq_serial_tx_notify(ring);
		}
		rtlmq_serial_arm_at(ring, jiffies +
			nsecs_to_jiffies(RTLMQ_SERIAL_DRAIN_MS * NSEC_PER_MSEC -
					  (now - s->drain_start)));
		return RTLMQ_SERIAL_DRAIN_WAIT;
	}
	c->permit = RTLMQ_SERIAL_NONE;
	rtlmq_serial_cancel_timer(ring);
	return RTLMQ_SERIAL_DRAIN_READY;
}

static bool rtlmq_serial_step_commit_locked(struct rtlmq_tx_ring *ring,
					    unsigned int other,
					    bool other_pending)
{
	struct rtlmq_private *tp = ring->priv;
	struct rtlmq_serial_tx_arbiter *s = &tp->tx_rings[0].cold.serial;
	struct rtlmq_serial_tx_control *c = &ring->serial_control;

	lockdep_assert_held(&s->lock);
	lockdep_assert_held(&ring->txq->_xmit_lock);
	/* Grant OWN only after the revoked queue is fully retired. */
	if (other < s->num_queues &&
	    (rtlmq_tx_published_count(&tp->tx_rings[other]) ||
	     READ_ONCE(tp->tx_rings[other].serial_control.permit) !=
		     RTLMQ_SERIAL_NONE))
		return false;
	s->dual_busy = other_pending;
	/* Its inactive fairness one-shot no longer describes this epoch. */
	rtlmq_serial_cancel_timer(ring);
	c->permit = other_pending ? RTLMQ_SERIAL_LIMITED :
				    RTLMQ_SERIAL_DIRECT;
	rtlmq_tx_epoch_begin(ring);
	s->phase = RTLMQ_SERIAL_ACTIVE;
	return true;
}

/* No hardware waits, reclaim or reset while holding these software locks. */
static void rtlmq_serial_step_locked(struct rtlmq_private *tp)
{
	struct rtlmq_serial_tx_arbiter *s = &tp->tx_rings[0].cold.serial;
	enum rtlmq_serial_tx_reason failure = RTLMQ_SERIAL_INVALID;
	bool other_pending, handoff = false;
	struct rtlmq_serial_tx_control *c;
	struct rtlmq_tx_ring *ring;
	unsigned int q, other;

	lockdep_assert_held(&s->lock);
	if (s->paused || s->phase == RTLMQ_SERIAL_RECOVERY)
		return;
	if (s->phase != RTLMQ_SERIAL_IDLE && s->phase != RTLMQ_SERIAL_ACTIVE &&
	    s->phase != RTLMQ_SERIAL_DRAIN && s->phase != RTLMQ_SERIAL_COMMIT)
		goto invalid;
	if (s->phase == RTLMQ_SERIAL_IDLE && !rtlmq_serial_step_idle_locked(tp))
		return;
	q = s->active;
	if (q >= s->num_queues || q >= RTLMQ_IMPL_MAX_TX_QUEUES)
		goto invalid;
	other = q ^ 1;
	other_pending = other < s->num_queues &&
			rtlmq_serial_pending(&tp->tx_rings[other]);
	ring = &tp->tx_rings[q];
	c = &ring->serial_control;
	__netif_tx_lock_bh(ring->txq);
	if (!rtlmq_serial_window_valid(ring))
		goto unlock_invalid;

	if (s->phase == RTLMQ_SERIAL_ACTIVE && other_pending)
		rtlmq_serial_step_active_locked(ring, other);
	if (s->phase == RTLMQ_SERIAL_DRAIN) {
		enum rtlmq_serial_drain_result drained;

		drained = rtlmq_serial_step_drain_locked(ring, other_pending);
		if (drained == RTLMQ_SERIAL_DRAIN_TIMEOUT) {
			failure = RTLMQ_SERIAL_TIMEOUT;
			goto unlock_invalid;
		}
		if (drained == RTLMQ_SERIAL_DRAIN_WAIT) {
			__netif_tx_unlock_bh(ring->txq);
			return;
		}
		handoff = true;
		__netif_tx_unlock_bh(ring->txq);
		/* Revoke old notification authority before taking the other queue lock. */
		q = other;
		other = q ^ 1;
		if (q >= s->num_queues)
			goto invalid;
		s->active = q;
		s->phase = RTLMQ_SERIAL_COMMIT;
		ring = &tp->tx_rings[q];
		c = &ring->serial_control;
		__netif_tx_lock_bh(ring->txq);
		if (!rtlmq_serial_window_valid(ring))
			goto unlock_invalid;
		other_pending = rtlmq_serial_pending(&tp->tx_rings[other]);
	}
	if (s->phase == RTLMQ_SERIAL_COMMIT &&
	    !rtlmq_serial_step_commit_locked(ring, other, other_pending))
		goto unlock_invalid;
	if (!other_pending) {
		s->dual_busy = false;
		c->permit = RTLMQ_SERIAL_DIRECT;
	}
	if (!rtlmq_serial_watch_service(ring)) {
		failure = RTLMQ_SERIAL_TIMEOUT;
		goto unlock_invalid;
	}
	if (!rtlmq_tx_publish_committed_batch(ring, &s->policy, s->dual_busy))
		goto unlock_invalid;
	if (handoff) {
		s->snapshot.handoffs++;
		s->snapshot.handoff_ns =
			rtlmq_serial_now() - s->drain_start;
	}
	if (s->dual_busy) {
		if (rtlmq_tx_epoch_should_yield(&ring->serial.epoch, &s->policy)) {
			rtlmq_serial_begin_drain(ring,
						 RTLMQ_SERIAL_LIMIT,
						 rtlmq_serial_now());
		} else {
			const struct rtlmq_serial_tx_control *waiter =
				&tp->tx_rings[other].serial_control;
			unsigned long now_jiffies = jiffies;
			unsigned long at = READ_ONCE(waiter->fairness_at);

			/* An early tick must not requeue an already-expired timer. */
			if (time_before_eq(at, now_jiffies))
				at = now_jiffies + 1;
			rtlmq_serial_arm_at(ring, at);
		}
	}
	__netif_tx_unlock_bh(ring->txq);
	return;
unlock_invalid:
	__netif_tx_unlock_bh(ring->txq);
invalid:
	rtlmq_serial_recover_locked(tp, failure);
}

bool rtlmq_serial_tx_service_wanted(const struct rtlmq_vector *vector)
{
	struct rtlmq_tx_ring *owner = rtlmq_serial_owner(vector->priv);
	struct rtlmq_serial_tx_arbiter *s;
	unsigned int q;

	if (!owner || vector->index != owner->vector_index)
		return false;
	s = &owner->cold.serial;
	if (READ_ONCE(s->paused) ||
	    READ_ONCE(s->phase) == RTLMQ_SERIAL_RECOVERY)
		return false;
	if (READ_ONCE(s->dual_busy) ||
	    READ_ONCE(s->phase) == RTLMQ_SERIAL_DRAIN)
		return true;
	for (q = 0; q < READ_ONCE(s->num_queues); q++)
		if (rtlmq_serial_pending(&vector->priv->tx_rings[q]) ||
		    READ_ONCE(vector->priv->tx_rings[q].serial_control.timer_due))
			return true;
	return false;
}

void rtlmq_serial_tx_service(struct rtlmq_vector *vector)
{
	struct rtlmq_private *tp = vector->priv;
	struct rtlmq_serial_tx_arbiter *s;
	bool recover, invalid;

	if (!rtlmq_serial_tx_service_wanted(vector))
		return;
	s = &tp->tx_rings[0].cold.serial;
	spin_lock_bh(&s->lock);
	if (READ_ONCE(tp->state) == RTLMQ_STATE_OPEN &&
	    rtlmq_mmio_ordinary_authorized(tp))
		rtlmq_serial_step_locked(tp);
	recover = s->phase == RTLMQ_SERIAL_RECOVERY;
	invalid = recover && s->snapshot.reason == RTLMQ_SERIAL_INVALID;
	spin_unlock_bh(&s->lock);
	WARN_ON_ONCE(invalid);
	if (recover) {
		rtlmq_serial_tx_dump(tp);
		rtlmq_request_reset(tp, BIT(RTLMQ_RESET_REASON_TX_TIMEOUT)
		);
	}
}

bool rtlmq_serial_tx_recovering(struct rtlmq_private *tp)
{
	struct rtlmq_tx_ring *owner = rtlmq_serial_owner(tp);

	return owner &&
	       READ_ONCE(owner->cold.serial.phase) == RTLMQ_SERIAL_RECOVERY;
}

void rtlmq_serial_tx_pause(struct rtlmq_private *tp)
{
	struct rtlmq_tx_ring *owner = rtlmq_serial_owner(tp);
	struct rtlmq_serial_tx_arbiter *s;
	unsigned int q;

	if (!owner)
		return;
	s = &owner->cold.serial;
	spin_lock_bh(&s->lock);
	if (!s->paused) {
		/* Retain the phase/permit for non-destructive live-service resume. */
		for (q = 0; q < s->num_queues; q++) {
			struct rtlmq_tx_ring *ring = &tp->tx_rings[q];

			__netif_tx_lock_bh(ring->txq);
			if (ring->tx_doorbell_pending)
				rtlmq_serial_tx_notify(ring);
			ring->serial_control.resume_permit =
				ring->serial_control.permit;
			ring->serial_control.permit = RTLMQ_SERIAL_NONE;
			ring->serial_control.paused = true;
			__netif_tx_unlock_bh(ring->txq);
		}
		rtlmq_serial_tx_capture_locked(tp, &s->snapshot);
	}
	s->paused = true;
	s->snapshot.paused = true;
	spin_unlock_bh(&s->lock);
	for (q = 0; q < s->num_queues; q++)
		rtlmq_tx_ring_cancel_liveness(&tp->tx_rings[q]);
}

void rtlmq_serial_tx_resume(struct rtlmq_private *tp)
{
	struct rtlmq_tx_ring *owner = rtlmq_serial_owner(tp);
	struct rtlmq_serial_tx_arbiter *s;
	unsigned int q, permits = 0;
	bool invalid = false;

	if (!owner)
		return;
	s = &owner->cold.serial;
	spin_lock_bh(&s->lock);
	s->num_queues = min_t(u16, RTLMQ_IMPL_MAX_TX_QUEUES,
			      rtlmq_published_num_tx_queues(tp));
	/* Validate the whole resume before restoring any cached authority. */
	for (q = 0; q < s->num_queues; q++) {
		struct rtlmq_tx_ring *ring = &tp->tx_rings[q];
		struct rtlmq_serial_tx_control *c = &ring->serial_control;
		enum rtlmq_serial_tx_permit permit;
		u32 generation;

		__netif_tx_lock_bh(ring->txq);
		permit = c->paused ? c->resume_permit : c->permit;
		/* Acquire a completed generation replacement before replaying a grant. */
		generation = smp_load_acquire(&ring->generation);
		if (ring->vector_index != owner->vector_index ||
		    permit > RTLMQ_SERIAL_DRAINING ||
		    (permit != RTLMQ_SERIAL_NONE &&
		     (c->generation != generation || s->active != q ||
		      s->phase == RTLMQ_SERIAL_RECOVERY || ++permits > 1)))
			invalid = true;
		__netif_tx_unlock_bh(ring->txq);
	}
	if (invalid) {
		rtlmq_serial_recover_locked(tp, RTLMQ_SERIAL_INVALID);
		goto unlock;
	}
	s->paused = false;
	for (q = 0; q < s->num_queues; q++) {
		struct rtlmq_tx_ring *ring = &tp->tx_rings[q];
		struct rtlmq_serial_tx_control *c = &ring->serial_control;

		__netif_tx_lock_bh(ring->txq);
		if (!c->paused) {
			__netif_tx_unlock_bh(ring->txq);
			continue;
		}
		/* Pin the committed generation before re-enabling producers and work. */
		c->generation = smp_load_acquire(&ring->generation);
		c->paused = false;
		c->permit = c->resume_permit;
		rtlmq_serial_cancel_timer(ring);
		if (rtlmq_tx_published_count(ring))
			rtlmq_serial_tx_watch(ring);
		if (c->pending) {
			c->fairness_at =
				jiffies +
				msecs_to_jiffies(RTLMQ_SERIAL_FAIRNESS_MS);
			rtlmq_serial_arm_at(ring, c->fairness_at);
			rtlmq_serial_tx_request(ring);
		}
		__netif_tx_unlock_bh(ring->txq);
	}
	if (s->phase == RTLMQ_SERIAL_DRAIN)
		s->drain_start = ktime_get_ns();
unlock:
	spin_unlock_bh(&s->lock);
	if (invalid)
		rtlmq_request_reset(tp, BIT(RTLMQ_RESET_REASON_TX_TIMEOUT)
		);
}

static void rtlmq_serial_cleanup_range(struct rtlmq_tx_ring *ring, u32 first,
				       u32 end)
{
	u32 i;

	if (WARN_ON_ONCE(rtlmq_tx_ring_distance(end, first) >=
			 ring->ring_count))
		return;
	/*
	 * Boundaries are packet aligned. Logical order reaches every packet's
	 * mappings before its EOP free, even across wrap; no preliminary ring
	 * scan/unmap pass is needed. It also preserves AF_XDP FIFO returns.
	 */
	for (i = first; i != end; i++)
		rtlmq_free_tx_info(ring, i & ring->ring_mask);
}

void rtlmq_serial_tx_cleanup_published(struct rtlmq_tx_ring *ring)
{
	/* The lifecycle caller establishes rtlmq_release_allowed(). */
	rtlmq_serial_cleanup_range(ring, ring->next_to_clean_count,
				   ring->next_to_use_count);
}

void rtlmq_serial_tx_cancel_staged(struct rtlmq_tx_ring *ring)
{
	/*
	 * Never-published packets need only software exclusion for DMA safety.
	 * AF_XDP CQ is positional: cancel its older published reservations first
	 * with lifecycle release permission. Caller excludes all ring users.
	 */
	rtlmq_serial_cleanup_range(ring, ring->next_to_use_count,
				   ring->serial.stage);
}

/* Explicit diagnostic snapshots only, never steady-state service scans. */
static u32 rtlmq_serial_owned(const struct rtlmq_tx_ring *ring)
{
	unsigned int i;
	u32 owned = 0;

	for (i = 0; i < ring->ring_count; i++)
		owned += !!(le32_to_cpu(READ_ONCE(rtlmq_tx_desc_at(ring, i)->opts1)) &
			    RTLMQ_DESC_OWN);
	return owned;
}

bool rtlmq_serial_tx_capture_locked(struct rtlmq_private *tp,
				    struct rtlmq_serial_tx_snapshot *snap)
{
	struct rtlmq_serial_tx_arbiter *s = &tp->tx_rings[0].cold.serial;
	bool valid = true;
	unsigned int q;

	lockdep_assert_held(&s->lock);
	/*
	 * Exclude both producers for one epoch-coherent snapshot. This nesting
	 * exists only here; ordinary NAPI publication takes one queue at a time.
	 * This reader never transmits, calls out or logs under these locks: do
	 * not claim a netdev transmit CPU owner or mutate producer bookkeeping.
	 */
	for (q = 0; q < s->num_queues; q++) {
		struct netdev_queue *txq = tp->tx_rings[q].txq;

		if (!q)
			spin_lock_bh(&txq->_xmit_lock);
		else
			spin_lock_nested(&txq->_xmit_lock,
					 SINGLE_DEPTH_NESTING);
	}
	*snap = s->snapshot;
	snap->phase = s->phase;
	snap->active = s->active;
	snap->paused = s->paused;
	for (q = 0; q < s->num_queues; q++) {
		struct rtlmq_tx_ring *ring = &tp->tx_rings[q];
		u32 dirty, hw, commit, stage;

		/* Producer frontiers are frozen; NAPI retirement may still advance. */
		stage = smp_load_acquire(&ring->serial.stage);
		/* Pair with NAPI's release of packet-atomic retirement. */
		dirty = smp_load_acquire(&ring->next_to_clean_count);
		hw = ring->next_to_use_count;
		commit = ring->serial.commit;
		snap->dirty[q] = dirty;
		snap->hw[q] = hw;
		snap->commit[q] = commit;
		snap->stage[q] = stage;
		snap->generation[q] = READ_ONCE(ring->generation);
		snap->permit[q] = ring->serial_control.permit;
		snap->pending[q] = ring->serial_control.pending;
		snap->doorbell_pending[q] = ring->tx_doorbell_pending;
		snap->published[q] = rtlmq_tx_ring_distance(hw, dirty);
		snap->frozen[q] = rtlmq_tx_ring_distance(commit, hw);
		snap->staged[q] = rtlmq_tx_ring_distance(stage, commit);
		snap->owned[q] = rtlmq_serial_owned(ring);
		snap->doorbells[q] = ring->doorbells;
		if (q == s->active)
			snap->usage = ring->serial.epoch;
		valid &= rtlmq_serial_tx_window_valid(dirty, hw, commit, stage,
						      ring->ring_count);
		valid &= snap->owned[q] <= snap->published[q];
		valid &= !snap->owned[q] || q == s->active;
	}
	for (q = s->num_queues; q; q--) {
		struct netdev_queue *txq = tp->tx_rings[q - 1].txq;

		if (q == 1)
			spin_unlock_bh(&txq->_xmit_lock);
		else
			spin_unlock(&txq->_xmit_lock);
	}
	/* Hardware may clear OWN while sampled, but no publisher can set it. */
	return valid && !(snap->owned[0] && snap->owned[1]) &&
	       !(snap->published[0] && snap->published[1]);
}

void rtlmq_serial_tx_snapshot(struct rtlmq_private *tp,
			      struct rtlmq_serial_tx_snapshot *snapshot)
{
	struct rtlmq_serial_tx_arbiter *s;
	bool valid = true;

	if (!tp || !tp->tx_rings || !rtlmq_owned_tx_ring_slots(tp))
		return;
	s = &tp->tx_rings[0].cold.serial;
	if (!s->initialized)
		return;
	spin_lock_bh(&s->lock);
	*snapshot = s->snapshot;
	if (!s->paused)
		valid = rtlmq_serial_tx_capture_locked(tp, snapshot);
	if (!valid) {
		s->snapshot.invariant_failures++;
		snapshot->invariant_failures = s->snapshot.invariant_failures;
	}
	spin_unlock_bh(&s->lock);
	WARN_ON_ONCE(!valid);
}

void rtlmq_serial_tx_dump(struct rtlmq_private *tp)
{
	struct rtlmq_serial_tx_snapshot snap = {};
	unsigned int q;

	if (!tp || !tp->tx_rings)
		return;
	rtlmq_serial_tx_snapshot(tp, &snap);
	netdev_info(tp->ndev,
		    "serialized TX phase=%u paused=%u active=%u epoch=%llu published=%u/%u owned=%u/%u frozen=%u/%u staged=%u/%u publication=%u doorbell=%u kicks=%llu/%llu reason=%u handoffs=%llu latency_ns=%llu timeouts=%llu invalid=%llu epoch_bytes=%llu descs=%llu packets=%llu\n",
		    snap.phase, snap.paused, snap.active, snap.epoch, snap.published[0],
		    snap.published[1], snap.owned[0], snap.owned[1], snap.frozen[0], snap.frozen[1],
		    snap.staged[0], snap.staged[1], snap.last_publication, snap.last_doorbell,
		    snap.doorbells[0], snap.doorbells[1], snap.reason, snap.handoffs,
		    snap.handoff_ns, snap.drain_timeouts, snap.invariant_failures, snap.usage.bytes,
		    snap.usage.descs, snap.usage.packets);
	for (q = 0; q < RTLMQ_IMPL_MAX_TX_QUEUES; q++)
		netdev_info(tp->ndev,
			    "serialized TX q%u generation=%u permit=%u dirty=%u hw=%u commit=%u stage=%u pending=%u deferred=%u\n",
			    q, snap.generation[q], snap.permit[q], snap.dirty[q], snap.hw[q],
			    snap.commit[q], snap.stage[q], snap.pending[q],
			    snap.doorbell_pending[q]);
}
