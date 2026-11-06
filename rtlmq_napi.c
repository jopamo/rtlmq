// SPDX-License-Identifier: GPL-2.0-only

#include <linux/bitops.h>
#include <linux/rtnetlink.h>

#include "rtlmq_irq.h"
#include "rtlmq.h"
#include "rtlmq_runtime.h"
#include "rtlmq_runtime_accessors.h"
#include "rtlmq_ptp.h"
#include "rtlmq_rx.h"
#include "rtlmq_ring.h"
#include "rtlmq_trace.h"
#include "rtlmq_tx.h"
#include "rtlmq_tx_private.h"
#include "rtlmq_xdp.h"

#define RTLMQ_RX_SERVICE_SMALL_BATCH_THRESHOLD	16
#define RTLMQ_RX_TX_YIELD_BATCH_THRESHOLD	4

static bool rtlmq_rx_tiny_batch(int rx_done)
{
	return rx_done > 0 &&
	       rx_done < RTLMQ_RX_TX_YIELD_BATCH_THRESHOLD;
}

static bool rtlmq_small_positive_hw_owned_should_yield_to_tx(const struct rtlmq_vector *vector)
{
	const struct rtlmq_private *tp;
	unsigned int num_tx_rings;
	u16 ring_slots;
	unsigned int i;

	if (!vector)
		return false;

	if (rtlmq_vector_has_tx_status_pending(vector) ||
	    rtlmq_vector_has_tx_reclaimable_desc(vector, NULL))
		return true;

	num_tx_rings = READ_ONCE(vector->num_tx_rings);
	if (!num_tx_rings)
		return false;

	tp = vector->priv;
	if (!tp || !tp->tx_rings)
		return false;

	ring_slots = rtlmq_owned_tx_ring_slots(tp);
	for (i = 0; i < num_tx_rings; i++) {
		const struct rtlmq_tx_ring *ring;

		ring = rtlmq_vector_tx_ring_at(tp, vector, i, ring_slots);
		if (!ring)
			continue;

		if (READ_ONCE(ring->tx_doorbell_pending) ||
		    rtlmq_tx_ring_queue_stopped_and_pending(ring))
			return true;
	}

	return false;
}

static bool rtlmq_rx_needs_autonomous_refill(const struct rtlmq_rx_ring *ring)
{
	return READ_ONCE(ring->rx_posted) < ring->ring_count &&
	       (!rtlmq_rx_ring_uses_xsk_pool(ring) ||
		!rtlmq_xsk_uses_need_wakeup(ring->cold->xsk_pool));
}

void rtlmq_restore_vector_rx_refill(struct rtlmq_vector *vector)
{
	struct rtlmq_private *tp = vector->priv;
	unsigned int i;

	lockdep_assert_held(&tp->control_lock);

	for (i = 0; i < vector->num_rx_rings; i++) {
		struct rtlmq_rx_ring *ring =
			&tp->rx_rings[vector->rx_ring_indices[i]];

		if (rtlmq_rx_needs_autonomous_refill(ring)) {
			rtlmq_retry_rx_refill(vector);
			break;
		}
	}
}

static int
rtlmq_poll_vector_rx_ring_fast(struct rtlmq_vector *vector,
			       struct rtlmq_rx_ring *ring, int budget,
			       bool skip_entry_own_mmio,
			       enum rtlmq_rx_clean_exit_reason *exit_reason_out)
{
	enum rtlmq_rx_clean_exit_reason exit_reason =
		RTLMQ_RX_CLEAN_EXIT_UNKNOWN;
	int work;

	if (likely(!rtlmq_ptp_rx_scan_maybe_required(ring->priv) &&
		   rtlmq_rx_skb_fast_entry_ready(ring)))
		work = rtlmq_poll_rx_ring_skb_fast(vector, ring, budget,
						   skip_entry_own_mmio,
						   &exit_reason);
	else
		work = rtlmq_poll_rx_ring(vector, ring, budget,
					  skip_entry_own_mmio, &exit_reason);

	if (unlikely(rtlmq_rx_needs_autonomous_refill(ring)))
		rtlmq_retry_rx_refill(vector);

	if (exit_reason_out)
		*exit_reason_out = exit_reason;

	return work;
}

static bool
rtlmq_rx_positive_hw_owned_continue_candidate(int work,
					      enum rtlmq_rx_clean_exit_reason exit_reason)
{
	return work > 0 &&
	       work < RTLMQ_RX_SERVICE_SMALL_BATCH_THRESHOLD &&
	       exit_reason == RTLMQ_RX_CLEAN_EXIT_HW_OWNED;
}

static int rtlmq_continue_positive_hw_owned_rx_ring(struct rtlmq_vector *vector,
						    struct rtlmq_rx_ring *ring,
						    int rx_done, int budget,
						    bool skip_entry_own_mmio)
{
	unsigned int repolls = 0;
	bool yield_exact_tiny_to_tx =
		rtlmq_rx_tiny_batch(rx_done) &&
		rtlmq_small_positive_hw_owned_should_yield_to_tx(vector);

	if (yield_exact_tiny_to_tx)
		return rx_done;

	/*
	 * After each small batch stops on OWN, recheck the same RX queue.
	 * Stop on the first unsuccessful check; re-enter the cleaner only for
	 * ready work. The threshold limits re-entry, not the cleaner's budget.
	 *
	 * For 1-3 packet batches, yield when this vector's TX needs service.
	 */
	while (rx_done > 0 && rx_done < budget &&
	       rx_done < RTLMQ_RX_SERVICE_SMALL_BATCH_THRESHOLD &&
	       repolls < RTLMQ_RX_SERVICE_SMALL_BATCH_THRESHOLD) {
		enum rtlmq_rx_clean_exit_reason exit_reason =
			RTLMQ_RX_CLEAN_EXIT_UNKNOWN;
		int work;

		if (!rtlmq_rx_ring_head_ready(ring))
			break;

		work = rtlmq_poll_vector_rx_ring_fast(vector, ring,
						      budget - rx_done,
						      skip_entry_own_mmio,
						      &exit_reason);
		repolls++;

		if (work <= 0)
			break;

		rx_done += work;

		if (!rtlmq_rx_positive_hw_owned_continue_candidate(work, exit_reason))
			break;
	}

	return rx_done;
}

static int rtlmq_poll_vector_rx_fast(struct rtlmq_private *tp,
				     struct rtlmq_vector *vector, int budget,
				     unsigned long rx_own_recheck_mask)
{
	struct rtlmq_rx_ring *positive_continue_ring = NULL;
	unsigned int num_rx_rings;
	unsigned int start;
	int rx_done = 0;
	unsigned int i;

	if (unlikely(budget <= 0))
		return 0;

	num_rx_rings = READ_ONCE(vector->num_rx_rings);
	if (unlikely(!num_rx_rings))
		return 0;

	/*
	 * Share one RX budget across the vector's rings. Each ring receives
	 * only the budget left by rings visited earlier in this poll. Rotate
	 * the first ring so repeated budget exhaustion cannot starve a peer.
	 * Apart from ring-local continuation, do not repoll RX before TX;
	 * allow TX reclaim and queue wakeups between RX service windows.
	 */
	if (likely(num_rx_rings == 1)) {
		u16 rx_queue = READ_ONCE(vector->rx_ring_indices[0]);
		struct rtlmq_rx_ring *ring;
		enum rtlmq_rx_clean_exit_reason exit_reason =
			RTLMQ_RX_CLEAN_EXIT_UNKNOWN;
		bool skip_entry_own_mmio;
		int work;

		ring = &tp->rx_rings[rx_queue];
		skip_entry_own_mmio =
			!(rx_own_recheck_mask & BIT(rx_queue));
		work = rtlmq_poll_vector_rx_ring_fast(vector, ring, budget,
						      skip_entry_own_mmio,
						      &exit_reason);

		if (rtlmq_rx_positive_hw_owned_continue_candidate(work, exit_reason))
			work = rtlmq_continue_positive_hw_owned_rx_ring(vector, ring, work, budget,
									skip_entry_own_mmio);

		return work;
	}

	start = READ_ONCE(vector->rx_poll_start);
	if (unlikely(start >= num_rx_rings))
		start = 0;
	WRITE_ONCE(vector->rx_poll_start,
		   start + 1 < num_rx_rings ? start + 1 : 0);

	for (i = 0; i < num_rx_rings && rx_done < budget; i++) {
		unsigned int ring_offset = start + i;
		struct rtlmq_rx_ring *ring;
		u16 rx_queue;
		enum rtlmq_rx_clean_exit_reason exit_reason =
			RTLMQ_RX_CLEAN_EXIT_UNKNOWN;
		bool skip_entry_own_mmio;
		int work;

		if (ring_offset >= num_rx_rings)
			ring_offset -= num_rx_rings;
		rx_queue = READ_ONCE(vector->rx_ring_indices[ring_offset]);
		ring = &tp->rx_rings[rx_queue];
		skip_entry_own_mmio =
			!(rx_own_recheck_mask & BIT(rx_queue));
		work = rtlmq_poll_vector_rx_ring_fast(vector, ring,
						      budget - rx_done,
						      skip_entry_own_mmio,
						      &exit_reason);

		if (rtlmq_rx_positive_hw_owned_continue_candidate(work, exit_reason) &&
		    !positive_continue_ring) {
			positive_continue_ring = ring;
		}

		rx_done += work;
	}

	if (positive_continue_ring && rx_done < budget)
		rx_done = rtlmq_continue_positive_hw_owned_rx_ring(
			vector, positive_continue_ring, rx_done, budget,
			!(rx_own_recheck_mask &
			  BIT(positive_continue_ring->queue_index)));

	return rx_done;
}

enum rtlmq_rx_entry_own_mmio_bypass_reason {
	RTLMQ_RX_ENTRY_OWN_MMIO_BYPASS_NONE = 0,
	RTLMQ_RX_ENTRY_OWN_MMIO_BYPASS_TX_IRQ,
	RTLMQ_RX_ENTRY_OWN_MMIO_BYPASS_NO_CAUSE_TX_PENDING,
};

static enum rtlmq_rx_entry_own_mmio_bypass_reason
rtlmq_rx_entry_own_mmio_bypass_reason(struct rtlmq_vector *vector, int budget,
				      u64 irq_cause)
{
	if (budget <= 0 || !READ_ONCE(vector->num_rx_rings))
		return RTLMQ_RX_ENTRY_OWN_MMIO_BYPASS_NONE;

	/*
	 * Keep the stale-OWN flush for RX or unknown IRQ causes. Otherwise a TX
	 * IRQ or nonempty TX ring is enough to classify this as TX service.
	 * Ring occupancy also covers repolls in coupled mode with TX IRQs masked.
	 */
	if (irq_cause & (BIT_ULL(RTLMQ_RX_SERVICE_REFILL_BIT) |
			 RTLMQ_RX_SERVICE_IRQ_CAUSE_RX |
			 RTLMQ_RX_SERVICE_IRQ_CAUSE_UNKNOWN))
		return RTLMQ_RX_ENTRY_OWN_MMIO_BYPASS_NONE;

	if (irq_cause & RTLMQ_RX_SERVICE_IRQ_CAUSE_TX)
		return RTLMQ_RX_ENTRY_OWN_MMIO_BYPASS_TX_IRQ;

	if (!irq_cause && rtlmq_vector_has_tx_pending_desc(vector))
		return RTLMQ_RX_ENTRY_OWN_MMIO_BYPASS_NO_CAUSE_TX_PENDING;

	return RTLMQ_RX_ENTRY_OWN_MMIO_BYPASS_NONE;
}

int rtlmq_poll_vector_rx_first(struct rtlmq_private *tp,
			       struct rtlmq_vector *vector, int budget,
			       u64 rx_service_entry_irq_cause,
			       unsigned long rx_own_recheck_mask)
{
	enum rtlmq_rx_entry_own_mmio_bypass_reason bypass_reason;

	bypass_reason =
		rtlmq_rx_entry_own_mmio_bypass_reason(vector, budget, rx_service_entry_irq_cause);
	/*
	 * Only a prospective TX-only bypass needs a readiness precheck.
	 * Otherwise the RX cleaner acquires OWN itself; do not read the same
	 * DMA cacheline once for admission and again for consumption.
	 */
	if (bypass_reason != RTLMQ_RX_ENTRY_OWN_MMIO_BYPASS_NONE &&
	    !rtlmq_rx_ready(vector))
		return 0;

	return rtlmq_poll_vector_rx_fast(tp, vector, budget,
					 rx_own_recheck_mask);
}

static u64 rtlmq_vector_next_tx_poll_seq(struct rtlmq_vector *vector);

static unsigned int rtlmq_vector_tx_ring_offset(unsigned int num_tx_rings,
						unsigned int ordinal,
						u64 poll_seq)
{
	/*
	 * Runtime TX queue counts are validated powers of two and this helper
	 * is reached only for the multi-ring case. Rotate the first ring
	 * without putting integer division in NAPI.
	 */
	return ((unsigned int)poll_seq + ordinal) & (num_tx_rings - 1);
}

static void
rtlmq_vector_tx_poll_merge_ring(struct rtlmq_vector_tx_poll *poll,
				const struct rtlmq_tx_poll_result *ring_poll)
{
	if (!poll || !ring_poll)
		return;

	poll->pkts_cleaned += ring_poll->pkts_cleaned;
}

static struct rtlmq_tx_poll_result
rtlmq_poll_tx_ring_napi(struct rtlmq_tx_ring *ring, int budget)
{
	if (!ring)
		return (struct rtlmq_tx_poll_result) {};

	/*
	 * The NAPI owner requests one sweep of the published interval. OWN is
	 * checked by reclaim, not by a speculative admission probe first.
	 * This does not arm the separately latched MMIO/OWN workaround.
	 */
	return rtlmq_poll_tx_ring_with_hint(ring, budget);
}

struct rtlmq_vector_tx_poll
rtlmq_poll_vector_tx_desc_fast(struct rtlmq_private *tp,
			       struct rtlmq_vector *vector, int budget)
{
	struct rtlmq_vector_tx_poll poll = {};
	unsigned int num_tx_rings;
	u16 ring_slots;
	unsigned int i;
	u64 poll_seq;

	if (!tp || !tp->tx_rings || !vector)
		return poll;

	num_tx_rings = READ_ONCE(vector->num_tx_rings);
	if (!num_tx_rings)
		return poll;

	ring_slots = rtlmq_owned_tx_ring_slots(tp);

	if (likely(num_tx_rings == 1)) {
		struct rtlmq_tx_poll_result ring_poll;
		struct rtlmq_tx_ring *ring;

		ring = rtlmq_vector_tx_ring_at(tp, vector, 0, ring_slots);
		if (!ring)
			return poll;

		ring_poll = rtlmq_poll_tx_ring_napi(ring, budget);

		poll.pkts_cleaned = ring_poll.pkts_cleaned;

		poll.wait[ring->queue_index] = ring_poll.wait;
		rtlmq_serial_tx_service(vector);
		return poll;
	}

	poll_seq = rtlmq_vector_next_tx_poll_seq(vector);
	for (i = 0; i < num_tx_rings; i++) {
		struct rtlmq_tx_poll_result ring_poll;
		struct rtlmq_tx_ring *ring;
		unsigned int ring_offset;

		ring_offset = rtlmq_vector_tx_ring_offset(num_tx_rings, i, poll_seq);
		ring = rtlmq_vector_tx_ring_at(tp, vector, ring_offset,
					       ring_slots);
		if (!ring)
			continue;

		ring_poll = rtlmq_poll_tx_ring_napi(ring, budget);
		rtlmq_vector_tx_poll_merge_ring(&poll, &ring_poll);
		poll.wait[ring->queue_index] = ring_poll.wait;
	}

	/* Same NAPI invocation: reclaim -> refill OR full-drain handoff. */
	rtlmq_serial_tx_service(vector);
	return poll;
}

static bool
rtlmq_vector_tx_desc_fast_service_wanted(struct rtlmq_vector *vector)
{
	if (rtlmq_serial_tx_service_wanted(vector))
		return true;
	/*
	 * Published work merits one reclaim pass, even if its IRQ hint was
	 * consumed while RX kept NAPI busy. Pending is not completion evidence:
	 * reclaim checks OWN and NAPI rearms rather than spinning on HW-owned
	 * work. Staged-only work is serviced solely by the serialized owner.
	 */
	return rtlmq_vector_has_tx_pending_desc(vector);
}

static void rtlmq_poll_vector_tx_on_rx_budget_full(struct rtlmq_private *tp,
						   struct rtlmq_vector *vector,
						   int budget)
{
	if (!vector)
		return;

	if (!READ_ONCE(vector->num_tx_rings) ||
	    !rtlmq_vector_tx_desc_fast_service_wanted(vector)) {
		return;
	}

	(void)rtlmq_poll_vector_tx_desc_fast(tp, vector, budget);
}

static u64 rtlmq_vector_next_tx_poll_seq(struct rtlmq_vector *vector)
{
	u64 poll_seq;

	if (!vector)
		return 1;

	poll_seq = READ_ONCE(vector->tx_poll_seq) + 1;
	if (unlikely(!poll_seq))
		poll_seq = 1;

	WRITE_ONCE(vector->tx_poll_seq, poll_seq);

	return poll_seq;
}

static void rtlmq_consume_tx_resweep_hints(struct rtlmq_vector *vector)
{
	/*
	 * Consume advisory service hints, not descriptor ownership evidence.
	 * Avoid an empty exchange on RX-only and continuing polls. A racing
	 * hint remains for the next poll if it arrives after the exchange.
	 */
	if (!READ_ONCE(vector->tx_resweep_hints))
		return;

	(void)xchg(&vector->tx_resweep_hints, 0);
}

static struct rtlmq_napi_poll_result
rtlmq_poll_vector_desc_fast(struct rtlmq_private *tp, struct rtlmq_vector *vector, int budget,
			    u64 rx_service_entry_irq_cause, unsigned long rx_own_recheck_mask)
{
	struct rtlmq_napi_poll_result result = {};
	struct rtlmq_vector_tx_poll tx_poll = {};
	int rx_done;

	rx_done = rtlmq_poll_vector_rx_first(tp, vector, budget,
					     rx_service_entry_irq_cause,
					     rx_own_recheck_mask);
	result.rx_done = rx_done;
	if (budget > 0 && rx_done == budget) {
		result.rx_budget_exhausted = true;
		/*
		 * RX used the full budget. Return budget and poll again even if
		 * this emptied the ring; core cannot distinguish that case from
		 * more work pending. TX reclaim does not consume RX budget, so
		 * run it and BQL wake handling once before returning.
		 */
		rtlmq_poll_vector_tx_on_rx_budget_full(tp, vector, budget);
		rtlmq_xsk_tx_poll_vector(tp, vector, budget);
		return result;
	}

	/*
	 * Poll RX before TX when budget is positive. TX reclaim uses descriptor
	 * OWN and pending counts, independent of RX budget. Budget-zero polls
	 * may reclaim ordinary TX but must defer XDP-frame returns.
	 */
	rtlmq_consume_tx_resweep_hints(vector);
	if (READ_ONCE(vector->num_tx_rings) &&
	    rtlmq_vector_tx_desc_fast_service_wanted(vector)) {
		tx_poll = rtlmq_poll_vector_tx_desc_fast(tp, vector, budget);
	}

	if (budget > 0)
		rtlmq_xsk_tx_poll_vector(tp, vector, budget);

	result.tx_done = tx_poll.pkts_cleaned;
	memcpy(result.tx_wait, tx_poll.wait, sizeof(result.tx_wait));
	return result;
}

static int rtlmq_poll_zero_budget_tx_only(struct rtlmq_private *tp,
					  struct rtlmq_vector *vector)
{
	/*
	 * netpoll may invoke NAPI with no RX budget. Do not even sample RX
	 * descriptor/IRQ readiness here: page-pool, XDP, AF_XDP, and NAPI
	 * completion APIs are forbidden. Only ordinary TX completion reclaim
	 * and its BQL/queue-wake side effects are permitted.
	 */
	rtlmq_consume_tx_resweep_hints(vector);
	if (READ_ONCE(vector->num_tx_rings) &&
	    rtlmq_vector_tx_desc_fast_service_wanted(vector)) {
		(void)rtlmq_poll_vector_tx_desc_fast(tp, vector, 0);
	}

	return 0;
}

static int rtlmq_poll(struct napi_struct *napi, int budget)
{
	struct rtlmq_vector *vector = container_of(napi, struct rtlmq_vector,
						  napi);
	struct rtlmq_private *tp = vector->priv;
	struct rtlmq_napi_poll_result result;
	unsigned long rx_own_recheck_mask;
	u64 rx_service_entry_irq_cause;

	/*
	 * A deferred service is running now. The latch only gates producers
	 * between a refused completion and the next poll entry, so clear it
	 * before any work; producers observe NAPI SCHED while this poll runs.
	 */
	WRITE_ONCE(vector->napi_service_pending, false);

	if (unlikely(budget == 0))
		return rtlmq_poll_zero_budget_tx_only(tp, vector);

	rx_service_entry_irq_cause =
		(u64)xchg(&vector->rx_service_irq_cause_pending, 0UL);
	rx_own_recheck_mask =
		xchg(&vector->rx_own_recheck_pending_mask, 0UL);
	result = rtlmq_poll_vector_desc_fast(tp, vector, budget,
					     rx_service_entry_irq_cause,
					     rx_own_recheck_mask);

	return rtlmq_napi_complete_vector(napi, vector, budget, &result);
}

int rtlmq_napi_complete_vector(struct napi_struct *napi,
			       struct rtlmq_vector *vector, int budget,
			       const struct rtlmq_napi_poll_result *result)
{
	const struct rtlmq_tx_reclaim_wait *wait = NULL;
	struct rtlmq_napi_rearm_plan rearm_plan = {};
	int work_done;
	int rx_done;

	if (WARN_ON_ONCE(!napi || !vector || !result))
		return 0;

	wait = result->tx_wait;
	rx_done = result->rx_done;

	if (budget <= 0)
		return rx_done;

	if (WARN_ON_ONCE(rx_done > budget))
		return budget;

	/*
	 * xmit_more may have deferred a TX doorbell even if NAPI was scheduled
	 * for other work. Flush under each TX queue lock before completion so
	 * queued descriptors cannot remain invisible to hardware.
	 */
	rtlmq_flush_vector_pending_tx_doorbells(vector);

	/*
	 * Return budget for an exactly-budget poll. Check descriptors again in
	 * a later under-budget poll before completing and rearming interrupts.
	 */
	if (result->rx_budget_exhausted) {
		if (WARN_ON_ONCE(rx_done != budget))
			return budget;
		return rx_done;
	}

	/*
	 * Finish descriptor/runtime reads before completion. Once it succeeds,
	 * napi_disable() on another CPU may return and tear down rings, layout,
	 * IRQs, and the active profile.
	 */
	if (rtlmq_rx_ready(vector))
		return budget;

	/*
	 * A TX descriptor may clear OWN after the last sweep, with its interrupt
	 * already consumed. Force a resweep and keep NAPI scheduled so reclaim
	 * cannot wait for a lost interrupt. The next poll requests reclaim
	 * from the published interval independently of IRQ-wanted/xoff state.
	 */
	if (rtlmq_vector_has_tx_reclaimable_desc(vector, wait)) {
		rtlmq_force_vector_tx_resweep(vector->priv, vector->index);
		return budget;
	}

	if (!rtlmq_prepare_napi_rearm(vector, &rearm_plan))
		return budget;

	/*
	 * Ordinary TX reclaim replenishes core software deferral. CBS uses
	 * one-packet turns: do not pace each handoff with that timer. RX work
	 * still drives deferral on coupled vectors; core settings stay intact.
	 */
	work_done = rx_done;
	if (result->tx_done && !rtlmq_serial_tx_uses_cbs(vector->priv))
		work_done += (int)result->tx_done;
	if (!napi_complete_done(napi, work_done)) {
		/*
		 * The core owns the next poll now. Publish that guaranteed
		 * service path so TX producers keep the completion source
		 * masked until it runs.
		 */
		rtlmq_vector_note_napi_service_pending(vector, true);
		return rx_done;
	}

	rearm_plan.completion_succeeded = true;
	rtlmq_apply_napi_rearm(vector, &rearm_plan);
	return min(rx_done, budget - 1);
}

static void rtlmq_napi_enable_core(struct rtlmq_vector *vector)
{
	napi_enable(&vector->napi);
}

static void rtlmq_rx_refill_work(struct work_struct *work)
{
	struct rtlmq_vector *vector = container_of(to_delayed_work(work),
		struct rtlmq_vector, rx_refill_work);
	unsigned long flags;

	spin_lock_irqsave(&vector->napi_rearm_lock, flags);
	if (vector->napi_rearm_allowed) {
		/* A software refill request must not take the TX-only RX bypass. */
		set_bit(RTLMQ_RX_SERVICE_REFILL_BIT,
			&vector->rx_service_irq_cause_pending);
		rtlmq_schedule_vector_napi(vector, RTLMQ_NAPI_SCHED_OTHER_DEFERRED);
	}
	spin_unlock_irqrestore(&vector->napi_rearm_lock, flags);
}

void rtlmq_retry_rx_refill(struct rtlmq_vector *vector)
{
	unsigned long flags;

	/*
	 * Allocation failure can leave no descriptor able to generate an IRQ.
	 * Retry through the same NAPI owner, at a bounded rate, until refill
	 * succeeds or teardown closes the rearm gate.
	 */
	spin_lock_irqsave(&vector->napi_rearm_lock, flags);
	if (vector->napi_rearm_allowed)
		mod_delayed_work(system_dfl_wq, &vector->rx_refill_work,
				 msecs_to_jiffies(10));
	spin_unlock_irqrestore(&vector->napi_rearm_lock, flags);
}

static void rtlmq_napi_disable_core(struct rtlmq_vector *vector)
{
	napi_disable(&vector->napi);
}

static void rtlmq_napi_delete_core(struct rtlmq_vector *vector)
{
	netif_napi_del(&vector->napi);
}

static void rtlmq_napi_set_irq_core(struct rtlmq_vector *vector, int irq)
{
	/*
	 * Automatic core IRQ affinity requires persistent NAPI configuration.
	 * The unhashed TX-only owner has none; its two IRQs remain owned by
	 * irq_sources, not by the RX affinity notifier/configuration machinery.
	 */
	if (test_bit(NAPI_STATE_NO_BUSY_POLL, &vector->napi.state))
		return;

	netif_napi_set_irq(&vector->napi, irq);
}

static void rtlmq_napi_add_config_core(struct rtlmq_private *tp,
				       struct rtlmq_vector *vector,
				       unsigned int index)
{
	netif_napi_add_config(tp->ndev, &vector->napi, rtlmq_poll, index);
}

static void
rtlmq_clear_vector_napi_registration_owner(struct rtlmq_vector *vector)
{
	struct rtlmq_private *tp;
	u64 vector_mask;

	if (!vector || !vector->priv)
		return;

	tp = vector->priv;
	if (WARN_ON_ONCE(vector->index >= RTLMQ_IMPL_MAX_NAPI))
		return;

	vector_mask = BIT_ULL(vector->index);
	tp->transaction.napi_registered_mask &= ~vector_mask;
}

int rtlmq_enable_vector_napi(struct rtlmq_vector *vector)
{
	unsigned long flags;

	if (!vector)
		return -EINVAL;

	if (!vector->napi_added)
		return -EINVAL;
	if (vector->napi_enabled)
		return -EALREADY;

	INIT_DELAYED_WORK(&vector->rx_refill_work, rtlmq_rx_refill_work);
	rtlmq_napi_enable_core(vector);
	spin_lock_irqsave(&vector->napi_rearm_lock, flags);
	lockdep_assert_held(&vector->napi_rearm_lock);
	WRITE_ONCE(vector->napi_rearm_allowed, true);
	vector->napi_enabled = true;
	spin_unlock_irqrestore(&vector->napi_rearm_lock, flags);
	return 0;
}

void rtlmq_withdraw_vector_napi_rearm(struct rtlmq_vector *vector)
{
	unsigned long flags;

	if (!vector)
		return;

	spin_lock_irqsave(&vector->napi_rearm_lock, flags);
	lockdep_assert_held(&vector->napi_rearm_lock);
	WRITE_ONCE(vector->napi_rearm_allowed, false);
	spin_unlock_irqrestore(&vector->napi_rearm_lock, flags);
}

int rtlmq_disable_vector_napi(struct rtlmq_vector *vector)
{
	if (!vector)
		return -EINVAL;

	if (!vector->napi_enabled)
		return -EALREADY;

	rtlmq_withdraw_vector_napi_rearm(vector);
	if (vector->rx_refill_work.work.func)
		cancel_delayed_work_sync(&vector->rx_refill_work);

	/* Reject inconsistent registration state without calling napi_disable(). */
	if (!vector->napi_added) {
		vector->napi_enabled = false;
		return -EINVAL;
	}

	rtlmq_napi_disable_core(vector);
	vector->napi_enabled = false;
	return 0;
}

void rtlmq_delete_vector_napi(struct rtlmq_vector *vector)
{
	if (!vector)
		return;

	if (vector->napi_enabled)
		WARN_ON_ONCE(rtlmq_disable_vector_napi(vector));

	if (!vector->napi_added) {
		rtlmq_clear_vector_napi_registration_owner(vector);
		return;
	}

	rtlmq_napi_delete_core(vector);
	vector->napi_added = false;
	rtlmq_clear_vector_napi_registration_owner(vector);
}

int rtlmq_validate_napi_enable_state(const struct rtlmq_private *tp,
				     const struct rtlmq_runtime_config *cfg,
				     const struct rtlmq_runtime_layout_state *layout)
{
	unsigned int i;

	if (!tp || !cfg || !layout || !layout->valid || !tp->vectors)
		return -EINVAL;

	for (i = 0; i < cfg->num_vectors; i++) {
		const struct rtlmq_vector *vector = &tp->vectors[i];
		const struct rtlmq_vector_layout *vector_layout;

		vector_layout = rtlmq_runtime_layout_vector(cfg, layout, i);
		if (!vector_layout ||
		    !rtlmq_vector_layout_published(vector_layout))
			continue;
		if (!vector->napi_added || vector->napi_enabled)
			return -EINVAL;
	}

	return 0;
}

void rtlmq_enable_active_napi(struct rtlmq_private *tp,
			      const struct rtlmq_runtime_config *cfg,
			      const struct rtlmq_runtime_layout_state *layout)
{
	unsigned int i;

	/*
	 * Candidate registration leaves NAPI disabled. Install active queue
	 * mappings and the resource generation before enabling it here.
	 * Keep interrupts masked until all active vectors are enabled.
	 */
	for (i = 0; i < cfg->num_vectors; i++) {
		const struct rtlmq_vector_layout *vector_layout;
		struct rtlmq_vector *vector = &tp->vectors[i];

		vector_layout = rtlmq_runtime_layout_vector(cfg, layout, i);
		if (!vector_layout ||
		    !rtlmq_vector_layout_published(vector_layout))
			continue;

		WARN_ON_ONCE(rtlmq_enable_vector_napi(vector));
	}
}

void rtlmq_clear_candidate_napi(struct rtlmq_private *tp)
{
	unsigned int i;

	if (!tp)
		return;
	if (!tp->vectors) {
		tp->transaction.napi_registered_mask = 0;
		return;
	}

	for (i = 0; i < tp->allocated_vectors; i++) {
		struct rtlmq_vector *vector = &tp->vectors[i];
		u64 vector_mask = BIT_ULL(i);

		/*
		 * The transaction mask identifies candidate registrations.
		 * Enabled/IRQ flags only determine whether teardown is safe.
		 */
		if (!(tp->transaction.napi_registered_mask & vector_mask))
			continue;
		if (vector->irq_requested || vector->napi_enabled ||
		    vector->napi_rearm_allowed)
			continue;

		rtlmq_delete_vector_napi(vector);
	}
}

static int rtlmq_sync_candidate_napi(struct rtlmq_private *tp)
{
	const struct rtlmq_runtime_layout_state *layout;
	const struct rtlmq_runtime_config *cfg;
	unsigned int i;

	if (!tp || !tp->vectors)
		return -EINVAL;

	cfg = rtlmq_candidate_runtime_config(tp);
	if (!cfg)
		return -EINVAL;

	layout = rtlmq_candidate_runtime_layout(tp);
	if (!layout)
		return -EINVAL;

	rtlmq_withdraw_napi_bindings(tp);

	for (i = 0; i < tp->allocated_vectors; i++) {
		const struct rtlmq_vector_layout *vector_layout;
		struct rtlmq_vector *vector = &tp->vectors[i];
		bool should_have_napi;

		vector_layout = rtlmq_runtime_layout_vector(cfg, layout, i);
		should_have_napi = vector_layout &&
				   rtlmq_vector_layout_published(vector_layout);

		if (should_have_napi) {
			if (i >= tp->ndev->num_napi_configs &&
			    vector_layout->num_rx_rings)
				return -EINVAL;

			if (!vector->napi_added) {
				/*
				 * RX vectors retain their config index across
				 * narrower generations, preserving NAPI identity
				 * and core settings. TX-only owners are unhashed
				 * and have no persistent config.
				 */
				if (!vector_layout->num_rx_rings)
					netif_napi_add_tx(tp->ndev, &vector->napi, rtlmq_poll);
				else
					rtlmq_napi_add_config_core(tp, vector, i);
				vector->napi_added = true;
				tp->transaction.napi_registered_mask |=
					BIT_ULL(i);
			}

			rtlmq_napi_set_irq_core(vector, -1);
			continue;
		}

		if (tp->transaction.napi_registered_mask & BIT_ULL(i))
			rtlmq_delete_vector_napi(vector);
	}

	for (i = 0; i < cfg->num_vectors; i++) {
		const struct rtlmq_vector_layout *vector_layout;
		struct rtlmq_vector *vector = &tp->vectors[i];

		vector_layout = rtlmq_runtime_layout_vector(cfg, layout, i);
		if (!vector_layout ||
		    !rtlmq_vector_layout_published(vector_layout))
			continue;

		if (!vector->napi_added)
			return -EINVAL;
	}

	return 0;
}

int rtlmq_register_candidate_napi(struct rtlmq_private *tp)
{
	int ret;

	ret = rtlmq_sync_candidate_napi(tp);
	if (ret)
		return ret;

	return 0;
}

static void rtlmq_withdraw_napi_bindings_locked(struct rtlmq_private *tp)
{
	unsigned int i;

	ASSERT_RTNL();

	if (!tp || !tp->ndev)
		return;

	for (i = 0; i < tp->ndev->num_rx_queues; i++)
		netif_queue_set_napi(tp->ndev, i, NETDEV_QUEUE_TYPE_RX, NULL);

	for (i = 0; i < tp->ndev->num_tx_queues; i++)
		netif_queue_set_napi(tp->ndev, i, NETDEV_QUEUE_TYPE_TX, NULL);

	if (tp->rx_rings) {
		for (i = 0; i < rtlmq_owned_rx_ring_slots(tp); i++)
			tp->rx_rings[i].napi = NULL;
	}

	if (!tp->vectors)
		return;

	for (i = 0; i < tp->allocated_vectors; i++) {
		struct rtlmq_vector *vector = &tp->vectors[i];

		if (!vector->napi_added)
			continue;

		rtlmq_napi_set_irq_core(vector, -1);
	}
}

void rtlmq_withdraw_napi_bindings(struct rtlmq_private *tp)
{
	if (!tp || !tp->vectors)
		return;

	rtlmq_withdraw_napi_bindings_locked(tp);
}

int rtlmq_validate_napi_bindings_layout(const struct rtlmq_private *tp,
					const struct rtlmq_runtime_config *cfg,
					const struct rtlmq_runtime_layout_state *layout)
{
	u16 num_rx_queues;
	u16 num_tx_queues;
	u16 num_vectors;
	unsigned int i;

	if (!tp || !cfg || !layout || !layout->valid || !tp->vectors)
		return -EINVAL;

	if (cfg->num_vectors > tp->allocated_vectors)
		return -EINVAL;

	num_rx_queues = cfg->num_rx_queues;
	num_tx_queues = cfg->num_tx_queues;
	num_vectors = cfg->num_vectors;

	for (i = 0; i < tp->allocated_vectors; i++) {
		const struct rtlmq_vector *vector = &tp->vectors[i];
		const struct rtlmq_vector_layout *vector_layout;

		if (!vector->napi_added)
			continue;

		vector_layout = rtlmq_runtime_layout_vector(cfg, layout, i);
		if (!vector_layout ||
		    !rtlmq_vector_layout_published(vector_layout))
			continue;

		if (vector->irq_requested && vector->irq >= 0)
			continue;

		return -EINVAL;
	}

	for (i = 0; i < num_rx_queues; i++) {
		u16 vector_index = rtlmq_runtime_layout_rx_vector_index(cfg, layout,
									i);

		if (vector_index >= num_vectors)
			return -EINVAL;

		if (tp->vectors[vector_index].napi_added)
			continue;

		return -EINVAL;
	}

	for (i = 0; i < num_tx_queues; i++) {
		u16 vector_index = rtlmq_runtime_layout_tx_vector_index(cfg, layout,
									i);

		if (vector_index >= num_vectors)
			return -EINVAL;

		if (tp->vectors[vector_index].napi_added)
			continue;

		return -EINVAL;
	}

	return 0;
}

int rtlmq_publish_napi_bindings_locked(struct rtlmq_private *tp,
				       const struct rtlmq_runtime_config *cfg,
				       const struct rtlmq_runtime_layout_state *layout)
{
	u16 num_rx_queues;
	u16 num_tx_queues;
	unsigned int i;

	ASSERT_RTNL();

	if (!tp || !tp->ndev || !cfg || !layout)
		return -EINVAL;

	lockdep_assert_held(&tp->control_lock);

	if (rtlmq_validate_napi_bindings_layout(tp, cfg, layout))
		return -EINVAL;

	num_rx_queues = cfg->num_rx_queues;
	num_tx_queues = cfg->num_tx_queues;

	/*
	 * Validate queue-to-NAPI and NAPI-to-IRQ bindings against the committed
	 * layout and allocated IRQ numbers before installing them.
	 */
	rtlmq_withdraw_napi_bindings_locked(tp);

	for (i = 0; i < tp->allocated_vectors; i++) {
		const struct rtlmq_vector_layout *vector_layout;
		struct rtlmq_vector *vector = &tp->vectors[i];

		if (!vector->napi_added)
			continue;

		vector_layout = rtlmq_runtime_layout_vector(cfg, layout, i);
		if (!vector_layout ||
		    !rtlmq_vector_layout_published(vector_layout))
			continue;

		rtlmq_napi_set_irq_core(vector, vector->irq);
	}

	for (i = 0; i < num_rx_queues; i++) {
		u16 vector_index = rtlmq_runtime_layout_rx_vector_index(cfg, layout,
									i);
		struct rtlmq_vector *vector = &tp->vectors[vector_index];

		netif_queue_set_napi(tp->ndev, i, NETDEV_QUEUE_TYPE_RX,
				     &vector->napi);

		if (tp->rx_rings && i < rtlmq_owned_rx_ring_slots(tp))
			tp->rx_rings[i].napi = &vector->napi;
	}

	for (i = 0; i < num_tx_queues; i++) {
		u16 vector_index = rtlmq_runtime_layout_tx_vector_index(cfg, layout,
									i);
		struct rtlmq_vector *vector = &tp->vectors[vector_index];

		netif_queue_set_napi(tp->ndev, i, NETDEV_QUEUE_TYPE_TX,
				     &vector->napi);
	}

	return 0;
}
