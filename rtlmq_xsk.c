// SPDX-License-Identifier: GPL-2.0-only

#include <linux/rcupdate.h>
#include <linux/bitops.h>
#include <linux/etherdevice.h>
#include <linux/if_vlan.h>
#include <linux/overflow.h>
#include <linux/rtnetlink.h>
#include <net/xdp_sock_drv.h>

#include "rtlmq_hw.h"
#include "rtlmq_lifecycle.h"
#include "rtlmq_irq.h"
#include "rtlmq_ptp.h"
#include "rtlmq_runtime_accessors.h"
#include "rtlmq.h"
#include "rtlmq_ring.h"
#include "rtlmq_runtime.h"
#include "rtlmq_rx.h"
#include "rtlmq_tx.h"
#include "rtlmq_xdp.h"

#define RTLMQ_XSK_TX_BUDGET	64U

static bool rtlmq_xsk_pool_runtime_owned_locked(struct rtlmq_private *tp,
						const struct xsk_buff_pool *pool);
static struct rtlmq_xsk_dma_pool_state *
rtlmq_find_xsk_dma_pool_state_locked(struct rtlmq_private *tp,
				     const struct xsk_buff_pool *pool);

void rtlmq_xsk_tx_completed(struct xsk_buff_pool *pool, u32 count)
{
	xsk_tx_completed(pool, count);
}

static enum rtlmq_tx_publish_result
rtlmq_xsk_publish_tx(struct rtlmq_tx_ring *ring, struct xsk_buff_pool *pool,
		     const struct xdp_desc *descs, unsigned int num_descs)
{
	return rtlmq_publish_tx_xsk_batch(ring, pool, descs, num_descs, false);
}

static void rtlmq_xsk_flush_tx(struct rtlmq_tx_ring *ring)
{
	rtlmq_tx_flush_doorbell_if_pending(ring);
}

static struct xsk_buff_pool *
rtlmq_active_xsk_pool(const struct rtlmq_private *tp, u16 queue_id)
{
	const struct rtlmq_rx_ring *ring;

	if (!tp || queue_id >= rtlmq_published_num_rx_queues(tp))
		return NULL;

	ring = &tp->rx_rings[queue_id];
	if (!ring->cold)
		return NULL;

	return READ_ONCE(ring->cold->xsk_pool);
}

static void rtlmq_xsk_complete_ordered(struct rtlmq_tx_ring *ring,
				       struct xsk_buff_pool *pool, u32 count)
{
	if (count && !rtlmq_ptp_xsk_tx_complete(ring->priv, pool, count, 0))
		rtlmq_xsk_tx_completed(pool, count);
}

void rtlmq_xsk_cancel_tx_batch(struct rtlmq_tx_ring *ring)
{
	struct rtlmq_xsk_tx_batch *batch = &ring->cold.xsk_batch;

	/* Full TX teardown has retired all earlier published/staged EOPs. */
	if (batch->pool && !rtlmq_xsk_pool_detached(ring->priv, batch->pool))
		rtlmq_xsk_complete_ordered(ring, batch->pool,
					   batch->count - batch->cursor + batch->rejected);
	memset(batch, 0, sizeof(*batch));
	ring->xsk_tx_discarding = false;
}

static bool rtlmq_xsk_retire_rejected(struct rtlmq_tx_ring *ring)
{
	struct rtlmq_xsk_tx_batch *batch = &ring->cold.xsk_batch;

	if (!batch->rejected)
		return true;
	/* The bounded outstanding ring interval makes the wrap comparison valid. */
	if ((s32)(smp_load_acquire(&ring->next_to_clean_count) -
		  batch->retire_after) < 0)
		return false;
	rtlmq_xsk_complete_ordered(ring, batch->pool, batch->rejected);
	batch->rejected = 0;
	return true;
}

static unsigned int rtlmq_xsk_tx_poll_ring(struct rtlmq_tx_ring *ring,
					   struct xsk_buff_pool *pool,
					   unsigned int budget)
{
	struct rtlmq_xsk_tx_batch *batch;
	unsigned int published = 0;
	struct netdev_queue *nq;
	unsigned int available;
	unsigned int max_segs;

	if (!ring || !pool || !budget)
		return 0;

	max_segs = min_t(unsigned int, RTLMQ_XSK_MAX_SEGS,
			 pool->xdp_zc_max_segs);
	if (!max_segs)
		return 0;

	nq = rtlmq_tx_ring_rm(ring)->ndev_queue;
	if (!nq)
		return 0;

	/* One NAPI owner consumes this pool; producers share the queue lock. */
	__netif_tx_lock_bh(nq);
	txq_trans_cond_update(nq);
	if (rtlmq_xsk_uses_need_wakeup(pool))
		rtlmq_xsk_clear_tx_need_wakeup(pool);

	batch = &ring->cold.xsk_batch;
	if (WARN_ON_ONCE(batch->pool && batch->pool != pool))
		goto out;
	if (!batch->pool) {
		available = min(budget, rtlmq_tx_desc_avail(ring));
		batch->count =
			rtlmq_ptp_xsk_tx_peek_release_desc_batch(ring->priv, pool, available);
		if (!batch->count)
			goto out;
		batch->pool = pool;
	}
	while (rtlmq_xsk_retire_rejected(ring) && batch->cursor < batch->count) {
		struct xdp_desc *descs = &pool->tx_descs[batch->cursor];
		enum rtlmq_tx_publish_result ret;
		unsigned int num_descs = 0;
		bool eop = false;

		/* Keep the unprocessed suffix in the core buffer; never peek over it. */
		if (ring->xsk_tx_discarding) {
			while (batch->cursor + num_descs < batch->count) {
				if (xsk_is_eop_desc(&descs[num_descs++])) {
					ring->xsk_tx_discarding = false;
					break;
				}
			}
			ret = RTLMQ_TX_DROPPED;
		} else {
			while (batch->cursor + num_descs < batch->count &&
			       num_descs < max_segs) {
				eop = xsk_is_eop_desc(&descs[num_descs]);
				num_descs++;
				if (eop)
					break;
			}
			/* A retained packet must progress even if the next poll budget shrinks. */
			if (published && (published >= budget ||
					  num_descs > budget - published))
				break;
			ret = eop ? rtlmq_xsk_publish_tx(ring, pool, descs, num_descs) :
				    RTLMQ_TX_DROPPED;
			if (ret == RTLMQ_TX_BUSY)
				break;
			if (!eop)
				ring->xsk_tx_discarding = true;
		}
		batch->cursor += num_descs;
		if (ret == RTLMQ_TX_PUBLISHED) {
			published += num_descs;
			continue;
		}

		/* No later CQ position may pass this earlier accepted TX frontier. */
		batch->rejected = num_descs;
		{
			batch->retire_after = ring->serial.stage;
		}
	}
	if (batch->cursor == batch->count && !batch->rejected)
		memset(batch, 0, sizeof(*batch));
	if (published)
		rtlmq_xsk_flush_tx(ring);
out:
	if (rtlmq_xsk_uses_need_wakeup(pool)) {
		u32 pending = READ_ONCE(ring->next_to_use_count);

		/* Pair with rtlmq_tx_publish_clean_batch() before reporting TX idle. */
		pending -= smp_load_acquire(&ring->next_to_clean_count);
		if (rtlmq_xsk_tx_needs_wakeup(pending))
			rtlmq_xsk_set_tx_need_wakeup(pool);
	}
	__netif_tx_unlock_bh(nq);

	return published;
}

static u16 rtlmq_xsk_tx_queue_count(const struct rtlmq_private *tp)
{
	return min(rtlmq_published_num_tx_queues(tp),
		   rtlmq_qualified_max_xsk_tx_queues(tp));
}

void rtlmq_xsk_start_tx_locked(struct rtlmq_private *tp)
{
	u16 queue_id;

	lockdep_assert_held(&tp->control_lock);

	for (queue_id = 0; queue_id < rtlmq_xsk_tx_queue_count(tp); queue_id++) {
		struct xsk_buff_pool *pool = rtlmq_active_xsk_pool(tp, queue_id);
		struct rtlmq_tx_ring *ring = &tp->tx_rings[queue_id];
		struct netdev_queue *nq = rtlmq_tx_ring_rm(ring)->ndev_queue;

		if (!pool || !rtlmq_xsk_uses_need_wakeup(pool))
			continue;

		/* Rebuilt idle rings have no completion to start TX polling. */
		__netif_tx_lock_bh(nq);
		if (!rtlmq_tx_accepted_pending(ring))
			rtlmq_xsk_set_tx_need_wakeup(pool);
		__netif_tx_unlock_bh(nq);
	}
}

unsigned int rtlmq_xsk_tx_poll_vector(struct rtlmq_private *tp,
				      struct rtlmq_vector *vector,
				      unsigned int budget)
{
	unsigned int tx_budget = min_t(unsigned int, budget,
				       RTLMQ_XSK_TX_BUDGET);
	unsigned int num_tx_rings;
	unsigned int total = 0;
	unsigned int start;
	unsigned int i;
	u16 tx_queues;

	if (!tp || !vector || !tx_budget)
		return 0;

	num_tx_rings = READ_ONCE(vector->num_tx_rings);
	if (!num_tx_rings)
		return 0;

	/* Submission needs its own rotation before work reaches the TX arbiter. */
	start = READ_ONCE(vector->xsk_tx_poll_start);
	if (unlikely(start >= num_tx_rings))
		start = 0;
	WRITE_ONCE(vector->xsk_tx_poll_start,
		   start + 1 < num_tx_rings ? start + 1 : 0);

	/*
	 * Teardown disables NAPI and waits for RCU readers before freeing
	 * RX cold state and TX ring storage used by this lookup.
	 */
	rcu_read_lock();
	tx_queues = rtlmq_xsk_tx_queue_count(tp);
	for (i = 0; i < num_tx_rings; i++) {
		unsigned int ring_offset = start + i;
		struct xsk_buff_pool *pool;
		u16 queue_id;

		if (ring_offset >= num_tx_rings)
			ring_offset -= num_tx_rings;
		queue_id = READ_ONCE(vector->tx_ring_indices[ring_offset]);
		if (queue_id >= tx_queues)
			continue;
		pool = rtlmq_active_xsk_pool(tp, queue_id);
		if (!pool)
			continue;

		total += rtlmq_xsk_tx_poll_ring(&tp->tx_rings[queue_id], pool,
						tx_budget - total);
		if (total >= tx_budget)
			break;
	}
	rcu_read_unlock();

	return total;
}

static int rtlmq_xsk_schedule_queue_vector(struct rtlmq_private *tp,
					   u16 vector_index)
{
	struct rtlmq_vector *vector;
	unsigned long flags;
	int ret;

	if (vector_index >= rtlmq_published_num_vectors(tp))
		return -ENXIO;

	vector = &tp->vectors[vector_index];
	/*
	 * Serialize wakeups with IRQ/NAPI rearming and teardown. Once teardown
	 * disables rearming, a wakeup must not schedule NAPI again.
	 */
	spin_lock_irqsave(&vector->napi_rearm_lock, flags);
	lockdep_assert_held(&vector->napi_rearm_lock);
	if (!READ_ONCE(vector->napi_enabled) ||
	    !READ_ONCE(vector->napi_rearm_allowed)) {
		ret = -ENETDOWN;
		goto out_unlock;
	}

	rtlmq_schedule_vector_napi(vector, RTLMQ_NAPI_SCHED_XSK_WAKEUP);
	ret = 0;

out_unlock:
	spin_unlock_irqrestore(&vector->napi_rearm_lock, flags);
	return ret;
}

struct rtlmq_xsk_wakeup_request {
	u16 queue_id;
	u32 flags;
	struct xsk_buff_pool *pool;
};

static int
rtlmq_normalize_xsk_wakeup_request(struct rtlmq_private *tp, u32 queue_id,
				   u32 flags,
				   struct rtlmq_xsk_wakeup_request *request)
{
	u32 admitted_flags = flags;

	if (!request)
		return -EINVAL;
	if (flags & ~(XDP_WAKEUP_RX | XDP_WAKEUP_TX))
		return -EINVAL;
	if (!netif_running(tp->ndev) || !netif_device_present(tp->ndev) ||
	    READ_ONCE(tp->state) != RTLMQ_STATE_OPEN)
		return -ENETDOWN;
	if (!(READ_ONCE(tp->ndev->xdp_features) &
	      NETDEV_XDP_ACT_XSK_ZEROCOPY))
		return -EOPNOTSUPP;
	if ((flags & XDP_WAKEUP_RX) &&
	    queue_id >= rtlmq_published_num_rx_queues(tp))
		return -EINVAL;
	if ((flags & XDP_WAKEUP_TX) &&
	    queue_id >= rtlmq_xsk_tx_queue_count(tp)) {
		/*
		 * AF_XDP core initializes TX need-wakeup before it knows whether
		 * this socket will use TX, so the first poll of an RX-only queue
		 * can carry RX|TX even when policy admits only q0 for XSK TX.
		 * Admit the valid RX refill hint and discard only the unavailable
		 * TX hint. A TX-only request for a nonexistent queue remains an
		 * error.
		 */
		if (!(flags & XDP_WAKEUP_RX))
			return -EINVAL;
		admitted_flags &= ~XDP_WAKEUP_TX;
	}
	if (queue_id > U16_MAX)
		return -EINVAL;

	request->queue_id = queue_id;
	request->flags = admitted_flags;
	request->pool = rtlmq_active_xsk_pool(tp, request->queue_id);
	if (!request->pool)
		return -ENXIO;

	return 0;
}

int rtlmq_xsk_wakeup(struct net_device *ndev, u32 queue_id, u32 flags)
{
	struct rtlmq_private *tp = netdev_priv(ndev);
	struct rtlmq_xsk_wakeup_request request;
	int ret;

	/*
	 * Wakeup does not run under NAPI. Hold RCU while reading ring->cold
	 * and the vector index; the rearm lock prevents scheduling after
	 * teardown disables the vector.
	 */
	rcu_read_lock();
	ret = rtlmq_normalize_xsk_wakeup_request(tp, queue_id, flags,
						 &request);
	if (ret)
		goto out_rcu;

	if (request.flags & XDP_WAKEUP_RX) {
		ret = rtlmq_xsk_schedule_queue_vector(
			tp, READ_ONCE(tp->rx_rings[request.queue_id].vector_index));
		if (ret)
			goto out_rcu;
	}

	if (request.flags & XDP_WAKEUP_TX) {
		ret = rtlmq_xsk_schedule_queue_vector(
			tp, READ_ONCE(tp->tx_rings[request.queue_id].vector_index));
		goto out_rcu;
	}

	ret = 0;
out_rcu:
	rcu_read_unlock();
	return ret;
}

/* AF_XDP requested/candidate/active state accessors. */

struct xsk_buff_pool *rtlmq_requested_xsk_pool(const struct rtlmq_private *tp,
					       u16 queue_index)
{
	if (!tp || queue_index >= RTLMQ_IMPL_MAX_RX_QUEUES)
		return NULL;

	return tp->requested_xsk_pools[queue_index];
}

bool rtlmq_xsk_tx_queue_count_allowed(const struct rtlmq_private *tp,
				      u16 num_tx_queues)
{
	const struct rtlmq_runtime_config *cfg;
	u16 admitted, queue;

	if (!tp)
		return true;
	cfg = rtlmq_active_runtime_config(tp);
	if (!cfg)
		cfg = rtlmq_requested_runtime_config(tp);
	if (!cfg)
		return true;
	admitted = min(cfg->num_tx_queues,
		       rtlmq_qualified_max_xsk_tx_queues(tp));
	/*
	 * Control admission precedes requested mutation; RTNL serializes core
	 * bind with these topology changes. Core may add TX to a shared RX pool
	 * without another driver attach. tx_descs persists until pool destruction,
	 * unlike current TX activity, so an idle pool still owns its capability.
	 * Use the committed shape, not a preference preserved across fallback.
	 * Pools above the old admitted TX range remain valid RX-only bindings.
	 */
	for (queue = num_tx_queues; queue < admitted; queue++) {
		struct xsk_buff_pool *pool = rtlmq_requested_xsk_pool(tp, queue);

		if (pool && READ_ONCE(pool->tx_descs))
			return false;
	}
	return true;
}

struct xsk_buff_pool *rtlmq_candidate_xsk_pool(const struct rtlmq_private *tp,
					       u16 queue_index)
{
	if (!tp || !tp->transaction.profile.valid ||
	    queue_index >= RTLMQ_IMPL_MAX_RX_QUEUES)
		return NULL;

	return tp->transaction.xsk_pools[queue_index];
}

u16 rtlmq_owned_xsk_pool_dma_users(const struct rtlmq_private *tp,
				   const struct xsk_buff_pool *pool)
{
	u16 slot;

	if (!tp || !pool)
		return 0;

	for (slot = 0; slot < RTLMQ_IMPL_MAX_RX_QUEUES; slot++) {
		if (READ_ONCE(tp->xsk_dma_pools[slot].pool) != pool)
			continue;

		return READ_ONCE(tp->xsk_dma_pools[slot].users);
	}

	return 0;
}

bool rtlmq_xsk_pool_dma_owned(const struct rtlmq_private *tp,
			      const struct xsk_buff_pool *pool)
{
	return rtlmq_owned_xsk_pool_dma_users(tp, pool) != 0;
}

bool rtlmq_xsk_pool_detached(const struct rtlmq_private *tp,
			     const struct xsk_buff_pool *pool)
{
	u16 slot;

	for (slot = 0; slot < RTLMQ_IMPL_MAX_RX_QUEUES; slot++)
		if (READ_ONCE(tp->xsk_dma_pools[slot].pool) == pool)
			return READ_ONCE(tp->xsk_dma_pools[slot].detached);
	return false;
}

u16 rtlmq_candidate_xsk_pool_bindings(const struct rtlmq_private *tp,
				      const struct xsk_buff_pool *pool)
{
	u16 count = 0;
	u16 i;

	if (!tp || !pool)
		return 0;

	for (i = 0; i < RTLMQ_IMPL_MAX_RX_QUEUES; i++) {
		if (rtlmq_candidate_xsk_pool(tp, i) == pool)
			count++;
	}

	return count;
}

u16 rtlmq_active_xsk_queue_count(const struct rtlmq_private *tp)
{
	const struct rtlmq_runtime_config *cfg;
	u16 count = 0;
	u16 i;

	if (!tp || !tp->rx_rings)
		return 0;

	cfg = rtlmq_active_runtime_config(tp);
	if (!cfg)
		return 0;

	for (i = 0; i < cfg->num_rx_queues; i++) {
		const struct rtlmq_rx_ring_cold *cold = tp->rx_rings[i].cold;

		if (cold && cold->xsk_pool)
			count++;
	}

	return count;
}

/* RX pool bindings and need-wakeup updates. */

bool rtlmq_rx_ring_xsk_binding_matches(const struct rtlmq_rx_ring *ring,
				       const struct xdp_rxq_info *expected_rxq)
{
	const struct xsk_buff_pool *pool;
	u32 i;

	if (!ring || !ring->cold || !ring->cold->xsk_pool)
		return false;

	pool = ring->cold->xsk_pool;
	if (pool->heads_cnt && !pool->heads)
		return false;

	for (i = 0; i < pool->heads_cnt; i++)
		if (READ_ONCE(pool->heads[i].xdp.rxq) != expected_rxq)
			return false;

	return true;
}

void rtlmq_publish_active_xsk_bindings_locked(struct rtlmq_private *tp,
					      const struct rtlmq_runtime_config *cfg,
					      const struct rtlmq_runtime_layout_state *layout)
{
	unsigned int i;

	lockdep_assert_held(&tp->control_lock);

	if (!tp || !cfg || !tp->rx_rings || !layout)
		return;

	for (i = 0; i < cfg->num_rx_queues; i++) {
		struct rtlmq_rx_ring *ring = &tp->rx_rings[i];

		if (!rtlmq_rx_ring_uses_xsk_pool(ring))
			continue;
		xsk_pool_set_rxq_info(ring->cold->xsk_pool,
				      &ring->cold->xdp_rxq);
	}
}

void rtlmq_withdraw_active_xsk_bindings_locked(struct rtlmq_private *tp)
{
	const struct rtlmq_runtime_config *cfg;
	unsigned int i;

	lockdep_assert_held(&tp->control_lock);

	cfg = rtlmq_active_runtime_config(tp);
	if (!cfg || !tp->rx_rings)
		return;

	for (i = 0; i < cfg->num_rx_queues; i++) {
		struct rtlmq_rx_ring *ring = &tp->rx_rings[i];

		if (!rtlmq_rx_ring_uses_xsk_pool(ring) ||
		    rtlmq_xsk_pool_detached(tp, ring->cold->xsk_pool))
			continue;
		xsk_pool_set_rxq_info(ring->cold->xsk_pool, NULL);
	}
}

void rtlmq_xsk_update_rx_need_wakeup(struct rtlmq_rx_ring *ring,
				     unsigned int requested,
				     unsigned int posted)
{
	struct xsk_buff_pool *pool;

	if (!rtlmq_rx_ring_uses_xsk_pool(ring))
		return;

	pool = ring->cold->xsk_pool;
	if (!rtlmq_xsk_uses_need_wakeup(pool))
		return;

	if (rtlmq_xsk_rx_needs_wakeup(requested, posted))
		rtlmq_xsk_set_rx_need_wakeup(pool);
	else
		rtlmq_xsk_clear_rx_need_wakeup(pool);
}

void rtlmq_xsk_release_rx_ring_state(struct rtlmq_rx_ring *ring)
{
	struct rtlmq_rx_ring_cold *cold;

	if (!ring || !ring->cold || !ring->cold->xsk_pool)
		return;

	cold = ring->cold;
	if (!rtlmq_xsk_pool_detached(ring->priv, cold->xsk_pool) &&
	    rtlmq_xsk_uses_need_wakeup(cold->xsk_pool)) {
		rtlmq_xsk_clear_rx_need_wakeup(cold->xsk_pool);
		rtlmq_xsk_clear_tx_need_wakeup(cold->xsk_pool);
	}
	if (cold->xsk_mb_head) {
		xsk_buff_free(cold->xsk_mb_head);
		cold->xsk_mb_head = NULL;
		cold->xsk_mb_len = 0;
		cold->xsk_mb_segs = 0;
	}
}

/* AF_XDP pool reconfiguration. */

static const struct rtlmq_rx_ring_cold *
rtlmq_active_rx_ring_cold(const struct rtlmq_private *tp, u16 queue_index)
{
	const struct rtlmq_runtime_config *cfg;

	if (!tp || !tp->rx_rings || READ_ONCE(tp->state) != RTLMQ_STATE_OPEN)
		return NULL;

	cfg = rtlmq_active_runtime_config(tp);
	if (!cfg || queue_index >= cfg->num_rx_queues)
		return NULL;

	return tp->rx_rings[queue_index].cold;
}

static bool rtlmq_active_rx_queue_has_mem_model(const struct rtlmq_private *tp,
						u16 queue_index)
{
	const struct rtlmq_runtime_config *cfg;
	const struct rtlmq_rx_ring_cold *cold;
	const struct rtlmq_rx_ring *ring;

	if (!tp || !tp->rx_rings)
		return false;

	cfg = rtlmq_active_runtime_config(tp);
	if (!cfg || queue_index >= cfg->num_rx_queues)
		return false;

	cold = rtlmq_active_rx_ring_cold(tp, queue_index);
	if (!cold)
		return false;

	ring = &tp->rx_rings[queue_index];
	if (!rtlmq_rx_ring_xdp_mem_model_matches(ring, tp->ndev,
						 queue_index))
		return false;
	if (cold->xsk_pool)
		return rtlmq_rx_ring_xsk_binding_matches(ring,
							 &cold->xdp_rxq);

	return true;
}

bool rtlmq_active_xsk_queue_eligible_locked(const struct rtlmq_private *tp,
					    u16 queue_index)
{
	lockdep_assert_held(&tp->control_lock);

	if (READ_ONCE(tp->state) != RTLMQ_STATE_OPEN)
		return false;

	return rtlmq_active_rx_queue_has_mem_model(tp, queue_index);
}

static void
rtlmq_forget_xsk_pool_requests_locked(struct rtlmq_private *tp,
				      const struct xsk_buff_pool *pool)
{
	unsigned int i;

	lockdep_assert_held(&tp->control_lock);

	/* Saved requests borrow pool pointers; no future rebuild may revive it. */
	for (i = 0; i < RTLMQ_IMPL_MAX_RX_QUEUES; i++) {
		if (tp->requested_xsk_pools[i] == pool)
			tp->requested_xsk_pools[i] = NULL;
		if (tp->transaction.xsk_pools[i] == pool)
			tp->transaction.xsk_pools[i] = NULL;
		if (tp->platform_recovery_committed.requested_xsk_pools[i] == pool)
			tp->platform_recovery_committed.requested_xsk_pools[i] = NULL;
	}
}

static bool rtlmq_paused_tx_owns_xsk_pool(const struct rtlmq_private *tp,
					  const struct xsk_buff_pool *pool)
{
	unsigned int q;

	lockdep_assert_held(&tp->control_lock);

	/*
	 * NAPI is disabled. External NDO_XDP_XMIT still takes the TX queue lock;
	 * netif_tx_disable() alone does not exclude that producer.
	 */
	for (q = 0; tp->tx_rings && q < rtlmq_owned_tx_ring_slots(tp); q++) {
		const struct rtlmq_tx_ring *ring = &tp->tx_rings[q];
		struct netdev_queue *nq = rtlmq_tx_ring_rm(ring)->ndev_queue;
		bool owned = false;
		u32 first;
		u32 count;
		u32 i;

		if (!nq) {
			if (ring->completions || ring->cold.xsk_batch.pool)
				return true;
			continue;
		}
		__netif_tx_lock_bh(nq);
		if (ring->cold.xsk_batch.pool == pool) {
			owned = true;
			goto unlock;
		}
		first = ring->next_to_clean_count;
		{
			count = rtlmq_tx_accepted_pending(ring);
		}
		if (!count)
			goto unlock;
		/* Invalid frontiers cannot authorize queue-local DMA unmap. */
		if (!ring->completions || count >= ring->ring_count) {
			owned = true;
			goto unlock;
		}
		for (i = 0; i < count; i++) {
			const struct rtlmq_tx_completion *completion =
				&ring->completions[(first + i) & ring->ring_mask];

			if (completion->type == RTLMQ_TX_BUFFER_XSK &&
			    completion->xsk_pool == pool) {
				owned = true;
				break;
			}
		}
unlock:
		__netif_tx_unlock_bh(nq);
		if (owned)
			return true;
	}
	return false;
}

static int rtlmq_stop_xsk_runtime_locked(struct rtlmq_private *tp)
{
	int ret;

	lockdep_assert_held(&tp->control_lock);

	/* Close revokes publication and retires all TX queues before RX mappings. */
	netif_device_detach(tp->ndev);
	ret = rtlmq_close_device(tp);
	if (!rtlmq_quiesce_contract_satisfied(tp)) {
		netdev_crit(tp->ndev,
			    "AF_XDP full shutdown incomplete: %d; pool retirement not established\n",
			    ret);
		return ret ? ret : -EIO;
	}
	/*
	 * Down-state restore may fail after the old runtime was destroyed.
	 * Rebuilding ordinary service is independent of final pool removal.
	 */
	if (ret)
		netdev_err(tp->ndev,
			   "AF_XDP runtime stopped but down-state restore failed: %d\n",
			   ret);
	/* An in-progress platform transition owns any later reopening. */
	if (tp->state == RTLMQ_STATE_PROBED &&
	    tp->platform_quiesce_owner == RTLMQ_PLATFORM_QUIESCE_NONE)
		rtlmq_request_reset(tp, BIT(RTLMQ_RESET_REASON_RUNTIME_RECONFIG)
		);
	return 0;
}

static int rtlmq_stop_xsk_detach_runtime_locked(struct rtlmq_private *tp,
						struct xsk_buff_pool *pool)
{
	int ret;

	/*
	 * No saved request may revive a departing pool. Full close does not
	 * require ordinary queue allocation to retire the old resources.
	 */
	rtlmq_forget_xsk_pool_requests_locked(tp, pool);
	/* Frozen PCI cannot run normal stop or wait for recovery under RTNL. */
	ret = rtlmq_mmio_ordinary_authorized(tp) ?
		rtlmq_stop_xsk_runtime_locked(tp) : -ENODEV;
	if (ret)
		ret = rtlmq_finalize_failed_runtime_locked(tp);
	if (!ret && rtlmq_xsk_pool_runtime_owned_locked(tp, pool))
		ret = -EBUSY;
	return ret;
}

static int
rtlmq_reconfigure_xsk_pool_inner_locked(struct rtlmq_private *tp,
					u16 queue_index,
					struct xsk_buff_pool *new_pool,
					enum rtlmq_live_change_outcome *outcome)
{
	struct rtlmq_rx_ring_replacement *replacement;
	struct rtlmq_live_service_pause pause;
	struct xsk_buff_pool *old_pool;
	bool detach;
	int ret;

	if (!tp || !tp->ndev || !tp->rx_rings)
		return -EINVAL;
	lockdep_assert_held(&tp->control_lock);
	ASSERT_RTNL();

	if (READ_ONCE(tp->state) != RTLMQ_STATE_OPEN ||
	    !netif_running(tp->ndev) ||
	    queue_index >= rtlmq_published_num_rx_queues(tp) ||
	    queue_index >= rtlmq_owned_rx_ring_slots(tp) ||
	    !tp->rx_rings[queue_index].cold)
		return -EBUSY;

	old_pool = tp->rx_rings[queue_index].cold->xsk_pool;
	detach = !new_pool;
	if (old_pool == new_pool) {
		tp->requested_xsk_pools[queue_index] = new_pool;
		return 0;
	}

	/*
	 * Allocate replacement buffers and prepare descriptors while the old
	 * queue runs. Commit replaces the buffers and memory model but reuses
	 * the active coherent descriptor allocation and hardware base.
	 */
	replacement = rtlmq_prepare_xsk_rx_ring_replacement(tp, queue_index, new_pool);
	if (IS_ERR(replacement)) {
		ret = PTR_ERR(replacement);
		if (!detach)
			return ret;
		netdev_err(tp->ndev,
			   "AF_XDP queue %u normal RX candidate failed: %d; closing runtime\n",
			   queue_index, ret);
		return rtlmq_stop_xsk_detach_runtime_locked(tp, old_pool);
	}

	ret = rtlmq_pause_live_service_locked(tp, &pause, RTLMQ_LIVE_MAC_PAUSE_RX_ONLY, detach);
	if (ret && !detach)
		goto err_discard_replacement;
	if (detach && (ret || rtlmq_paused_tx_owns_xsk_pool(tp, old_pool))) {
		if (ret)
			netdev_err(tp->ndev,
				   "AF_XDP queue %u pause failed: %d; closing runtime\n",
				   queue_index, ret);
		rtlmq_discard_xsk_rx_ring_replacement(tp, replacement);
		return rtlmq_stop_xsk_detach_runtime_locked(tp, old_pool);
	}

	/*
	 * RX pause affects all queues. Validate every active ring before
	 * replacing the target queue. On failure, leave RX and host service
	 * stopped and request reset without rewriting descriptor bases.
	 */
	ret = rtlmq_validate_active_rx_rings_for_restart_locked(tp);
	if (ret) {
		rtlmq_discard_xsk_rx_ring_replacement(tp, replacement);
		netdev_err(tp->ndev,
			   "AF_XDP queue %u found invalid active RX restart set: %d\n",
			   queue_index, ret);
		rtlmq_request_reset(tp, BIT(RTLMQ_RESET_REASON_RUNTIME_RECONFIG)
		);
		if (!detach)
			return ret;

		return rtlmq_stop_xsk_detach_runtime_locked(tp, old_pool);
	}

	ret = rtlmq_align_xsk_rx_ring_replacement_locked(tp, replacement);
	if (ret) {
		if (!detach)
			goto err_restore_and_discard_replacement;
		rtlmq_discard_xsk_rx_ring_replacement(tp, replacement);
		return rtlmq_stop_xsk_detach_runtime_locked(tp, old_pool);
	}

	/*
	 * RX DMA is drained, NAPI disabled, IRQ handlers synchronized, and netdev
	 * TX stopped. Replace the buffers without allocating memory, resetting
	 * the device, or changing PHY, RSS, IRQ allocation, or carrier state.
	 */
	rtlmq_commit_xsk_rx_ring_replacement(tp, replacement);
	tp->requested_xsk_pools[queue_index] = new_pool;
	*outcome = RTLMQ_LIVE_CHANGE_RECOVERY_PENDING;

	/*
	 * Clearing CmdRxEnb resets every hardware RX cursor. Rebuild all active
	 * RX rings from descriptor zero and write their bases while RX remains
	 * disabled. TX stays enabled. Other RX queues keep their buffers and
	 * memory models, but their software cursors must also restart at zero.
	 */
	ret = rtlmq_restart_active_rx_rings_from_zero_locked(tp);
	if (ret) {
		rtlmq_finish_xsk_rx_ring_replacement(tp, replacement);
		netdev_err(tp->ndev,
			   "AF_XDP queue %u committed but all-RX restart failed: %d\n",
			   queue_index, ret);
		return ret;
	}

	rtlmq_finish_xsk_rx_ring_replacement(tp, replacement);
	ret = rtlmq_restore_live_service_locked(tp, &pause);
	if (ret) {
		/*
		 * Keep the committed pool binding. Service may have resumed only
		 * partially; queue reset recovery rather than rebuilding here.
		 */
		netdev_err(tp->ndev,
			   "AF_XDP queue %u committed but MAC service resume failed: %d\n",
			   queue_index, ret);
		return ret;
	}

	*outcome = RTLMQ_LIVE_CHANGE_COMMITTED;
	return 0;

err_discard_replacement:
	rtlmq_discard_xsk_rx_ring_replacement(tp, replacement);
	return ret;

err_restore_and_discard_replacement:
	{
		int restore_ret =
			rtlmq_restore_live_service_locked(tp, &pause);

		if (restore_ret) {
			/* Attach failed: preserve the old request, discard only candidate. */
			rtlmq_discard_xsk_rx_ring_replacement(tp, replacement);
			rtlmq_stop_xsk_runtime_locked(tp);
			return ret;
		}
	}
	rtlmq_discard_xsk_rx_ring_replacement(tp, replacement);
	return ret;
}

int rtlmq_reconfigure_xsk_pool_locked(struct rtlmq_private *tp, u16 queue_index,
				      struct xsk_buff_pool *new_pool)
{
	enum rtlmq_live_change_outcome outcome = RTLMQ_LIVE_CHANGE_NOT_COMMITTED;
	struct rtlmq_netpoll_guard netpoll_guard;
	int ret;

	rtlmq_netpoll_guard_acquire_locked(tp, &netpoll_guard);
	ret = rtlmq_reconfigure_xsk_pool_inner_locked(tp, queue_index, new_pool, &outcome);
	ret = rtlmq_complete_live_change_locked(tp, outcome, ret, "xsk_pool");
	rtlmq_netpoll_guard_release_locked(tp, &netpoll_guard);

	return ret;
}

static int rtlmq_retain_detached_xsk_pool_locked(struct rtlmq_private *tp,
						struct xsk_buff_pool *pool,
						int detach_ret)
{
#ifdef XSK_POOL_DMA_RETENTION_API
	struct rtlmq_xsk_dma_pool_state *state;
	struct rtlmq_netpoll_guard guard;
	int ret;

	state = rtlmq_find_xsk_dma_pool_state_locked(tp, pool);
	if (!state || !state->ref)
		return detach_ret;

	rtlmq_forget_xsk_pool_requests_locked(tp, pool);
	if (!rtlmq_final_release_allowed(tp)) {
		rtlmq_netpoll_guard_acquire_locked(tp, &guard);
		ret = rtlmq_finalize_failed_runtime_locked(tp);
		rtlmq_netpoll_guard_release_locked(tp, &guard);
		if (!ret)
			return 0;
	}
	if (!rtlmq_final_release_allowed(tp))
		return detach_ret;

	/*
	 * Terminal host retirement drains NAPI, TX producers and PTP work.
	 * Keep RX heads and DMA ownership, but never touch the socket rings
	 * during later TX teardown. Core clears pool->netdev after we return.
	 */
	state = rtlmq_find_xsk_dma_pool_state_locked(tp, pool);
	if (WARN_ON_ONCE(!state || !state->ref))
		return detach_ret;
	if (!state->detached) {
		xsk_pool_dma_unmap(pool, 0);
		state->detached = true;
	}
	return 0;
#else
	return detach_ret;
#endif
}

int rtlmq_detach_xsk_pool_locked(struct rtlmq_private *tp, u16 queue_index)
{
	struct xsk_buff_pool *pool;
	int ret;

	lockdep_assert_held(&tp->control_lock);
	ASSERT_RTNL();

	pool = rtlmq_requested_xsk_pool(tp, queue_index);
	if (!pool && tp->rx_rings &&
	    queue_index < rtlmq_owned_rx_ring_slots(tp) &&
	    tp->rx_rings[queue_index].cold)
		pool = tp->rx_rings[queue_index].cold->xsk_pool;
	if (!pool)
		return -EINVAL;

	/*
	 * Final detach is a release operation, not a new configuration request.
	 * Removal and PM must allow it when no runtime can still use the pool.
	 */
	if (!rtlmq_candidate_release_allowed(tp) ||
	    rtlmq_xsk_pool_runtime_owned_locked(tp, pool)) {
		if (tp->state != RTLMQ_STATE_OPEN ||
		    tp->platform_quiesce_owner != RTLMQ_PLATFORM_QUIESCE_NONE ||
		    !rtlmq_published_runtime_config(tp)) {
			struct rtlmq_netpoll_guard guard;

			/*
			 * PM drops locks before close; PCI remove enters REMOVING
			 * before unregister takes RTNL. Retire a core-owned pool
			 * in either interval. Revoked MMIO requires terminal cleanup,
			 * so a later recovery cannot revive the departing pool.
			 * A failed open/reset/close may also retain non-OPEN owners.
			 * An unpublished OPEN owner awaiting recovery cannot use
			 * queue-local replacement either.
			 */
			rtlmq_netpoll_guard_acquire_locked(tp, &guard);
			ret = rtlmq_stop_xsk_detach_runtime_locked(tp, pool);
			rtlmq_netpoll_guard_release_locked(tp, &guard);
		} else {
			ret = rtlmq_admit_control_change_locked(tp, true);
			if (ret)
				return rtlmq_retain_detached_xsk_pool_locked(tp, pool, ret);
			ret = rtlmq_reconfigure_xsk_pool_locked(tp, queue_index, NULL);
		}
		if (ret)
			return rtlmq_retain_detached_xsk_pool_locked(tp, pool, ret);
	}

	rtlmq_forget_xsk_pool_requests_locked(tp, pool);
	if (!rtlmq_xsk_pool_runtime_owned_locked(tp, pool))
		rtlmq_release_xsk_pool_dma_locked(tp, pool);
	return 0;
}

static struct rtlmq_xsk_dma_pool_state *
rtlmq_find_xsk_dma_pool_state_locked(struct rtlmq_private *tp,
				     const struct xsk_buff_pool *pool)
{
	u16 slot;

	lockdep_assert_held(&tp->control_lock);

	if (!tp || !pool)
		return NULL;

	for (slot = 0; slot < RTLMQ_IMPL_MAX_RX_QUEUES; slot++) {
		if (tp->xsk_dma_pools[slot].pool == pool)
			return &tp->xsk_dma_pools[slot];
	}

	return NULL;
}

static struct rtlmq_xsk_dma_pool_state *
rtlmq_find_free_xsk_dma_pool_state_locked(struct rtlmq_private *tp)
{
	u16 slot;

	lockdep_assert_held(&tp->control_lock);

	if (!tp)
		return NULL;

	for (slot = 0; slot < RTLMQ_IMPL_MAX_RX_QUEUES; slot++) {
		if (!tp->xsk_dma_pools[slot].pool)
			return &tp->xsk_dma_pools[slot];
	}

	return NULL;
}

static struct device *rtlmq_xsk_dma_device(const struct rtlmq_private *tp)
{
	if (!tp)
		return NULL;

	if (tp->pdev)
		return &tp->pdev->dev;
	if (tp->ndev && tp->ndev->dev.parent)
		return tp->ndev->dev.parent;
	if (tp->ndev)
		return &tp->ndev->dev;

	return NULL;
}

static bool rtlmq_xsk_pool_needs_dma_map(const struct xsk_buff_pool *pool)
{
	if (!pool)
		return false;
	return !!pool->umem;
}

static bool rtlmq_xsk_pool_dma_ready(const struct rtlmq_private *tp,
				     const struct xsk_buff_pool *pool)
{
	struct device *dma_dev;

	if (!pool)
		return false;
	if (!pool->umem)
		return false;

	dma_dev = rtlmq_xsk_dma_device(tp);
	return dma_dev && pool->dev == dma_dev &&
	       pool->dma_pages && pool->dma_pages_cnt;
}

static bool
rtlmq_xsk_pool_runtime_owned_locked(struct rtlmq_private *tp,
				    const struct xsk_buff_pool *pool)
{
	unsigned int i;

	lockdep_assert_held(&tp->control_lock);

	if (!tp || !pool)
		return false;

	/*
	 * Check active/replacement RX rings and retained TX batches. A saved
	 * requested binding alone does not mean a ring is using the pool.
	 */
	for (i = 0; i < rtlmq_owned_rx_ring_slots(tp); i++) {
		struct rtlmq_rx_ring *candidate =
			tp->transaction.rx_rings[i];

		if (tp->rx_rings && tp->rx_rings[i].cold &&
		    tp->rx_rings[i].cold->xsk_pool == pool)
			return true;
		if (candidate && candidate->cold &&
		    candidate->cold->xsk_pool == pool)
			return true;
	}

	for (i = 0; tp->tx_rings && i < rtlmq_owned_tx_ring_slots(tp); i++)
		if (READ_ONCE(tp->tx_rings[i].cold.xsk_batch.pool) == pool)
			return true;

	return false;
}

int rtlmq_acquire_xsk_pool_dma_locked(struct rtlmq_private *tp,
				      struct xsk_buff_pool *pool)
{
	struct rtlmq_xsk_dma_pool_state *state;
	struct device *dma_dev;
	int ret;

	lockdep_assert_held(&tp->control_lock);

	if (!tp || !pool)
		return -EINVAL;

	state = rtlmq_find_xsk_dma_pool_state_locked(tp, pool);
	if (state) {
		if (state->detached)
			return -ENODEV;
		if (!rtlmq_xsk_pool_dma_ready(tp, pool))
			return -EIO;
		state->users++;
		return 0;
	}

	state = rtlmq_find_free_xsk_dma_pool_state_locked(tp);
	if (!state)
		return -ENOSPC;

	/* A pool without UMEM-backed pages has no DMA mapping work. */
	if (rtlmq_xsk_pool_needs_dma_map(pool)) {
		dma_dev = rtlmq_xsk_dma_device(tp);
		if (!dma_dev)
			return -ENODEV;

		ret = xsk_pool_dma_map(pool, dma_dev, 0);
		if (ret)
			return ret;
	}

	if (!rtlmq_xsk_pool_dma_ready(tp, pool)) {
		if (pool->umem &&
		    !rtlmq_xsk_pool_runtime_owned_locked(tp, pool))
			xsk_pool_dma_unmap(pool, 0);
		return -EIO;
	}

#ifdef XSK_POOL_DMA_RETENTION_API
	if (rtlmq_xsk_pool_needs_dma_map(pool)) {
		ASSERT_RTNL();
		state->ref = xsk_pool_dma_get(pool);
		if (!state->ref) {
			xsk_pool_dma_unmap(pool, 0);
			return -EIO;
		}
	}
#endif
	state->pool = pool;
	state->users = 1;

	return 0;
}

static bool
rtlmq_xsk_pool_geometry_model_valid(const struct xsk_buff_pool *pool)
{
	if (!pool)
		return false;
	if (pool->umem)
		return true;
	return false;
}

u32 rtlmq_xsk_pool_admission_max_segs(const struct rtlmq_private *tp,
				      const struct xsk_buff_pool *pool)
{
	u32 committed;

	if (!tp || !pool)
		return 0;
	if (!pool->umem ||
	    !(pool->umem->flags & XDP_UMEM_SG_FLAG))
		return 1;

	/*
	 * The core sets the UMEM SG flag before XDP_SETUP_XSK_POOL, but does
	 * not publish pool->xdp_zc_max_segs until ndo_bpf() succeeds. Candidate
	 * validation therefore uses the netdev limit while that field is zero.
	 * Once the pool is bound, use its negotiated limit for later rebuilds.
	 */
	committed = READ_ONCE(pool->xdp_zc_max_segs);
	return committed ? committed : rtlmq_xdp_zc_max_segs(tp);
}

static void
rtlmq_unmap_xsk_dma_state_locked(struct rtlmq_private *tp,
				 struct rtlmq_xsk_dma_pool_state *state)
{
	struct xsk_buff_pool *pool = state->pool;

	lockdep_assert_held(&tp->control_lock);

	if (rtlmq_xsk_pool_runtime_owned_locked(tp, pool) ||
	    (state->ref && (tp->retained_live_rx || tp->retained_live_tx)) ||
	    (state->detached && !rtlmq_release_allowed(tp))) {
		netdev_err(tp->ndev,
			   "refusing XSK DMA unmap while pool %p remains ring-owned\n",
			   pool);
		return;
	}
	if (!state->detached && pool->umem && rtlmq_xsk_pool_dma_ready(tp, pool))
		xsk_pool_dma_unmap(pool, 0);
#ifdef XSK_POOL_DMA_RETENTION_API
	if (state->ref) {
		ASSERT_RTNL();
		xsk_pool_dma_put(state->ref);
	}
#endif
	memset(state, 0, sizeof(*state));
}

void rtlmq_release_xsk_pool_dma_locked(struct rtlmq_private *tp,
				       const struct xsk_buff_pool *pool)
{
	struct rtlmq_xsk_dma_pool_state *state;

	lockdep_assert_held(&tp->control_lock);

	if (!tp || !pool)
		return;

	state = rtlmq_find_xsk_dma_pool_state_locked(tp, pool);
	if (!state)
		return;

	if (state->users > 1) {
		state->users--;
		return;
	}

	rtlmq_unmap_xsk_dma_state_locked(tp, state);
}

void rtlmq_release_all_xsk_pool_dma_locked(struct rtlmq_private *tp)
{
	u16 slot;

	lockdep_assert_held(&tp->control_lock);

	if (!tp)
		return;

	for (slot = 0; slot < RTLMQ_IMPL_MAX_RX_QUEUES; slot++) {
		struct rtlmq_xsk_dma_pool_state *state = &tp->xsk_dma_pools[slot];

		if (!state->pool)
			continue;

		rtlmq_unmap_xsk_dma_state_locked(tp, state);
	}
}

bool rtlmq_rx_xsk_frame_size_valid_pool_geometry(const struct rtlmq_private *tp,
						 const struct xsk_buff_pool *pool, u32 frame_size,
						 const struct rtlmq_rx_geometry *geometry)
{
	if (!tp || !pool || !geometry)
		return false;

	if (pool->umem && (pool->umem->flags & XDP_UMEM_SG_FLAG))
		return rtlmq_rx_xsk_sg_frame_size_valid_geometry(frame_size,
				rtlmq_xsk_pool_admission_max_segs(tp, pool), geometry);

	return rtlmq_rx_xsk_frame_size_valid_geometry(frame_size, geometry);
}

int rtlmq_validate_xsk_pool_geometry_locked(const struct rtlmq_private *tp,
					    struct xsk_buff_pool *pool,
					    u16 queue_index,
					    u32 *frame_size_out)
{
	struct rtlmq_rx_geometry geometry;
	u32 chunk_size;
	u32 frame_size;
	u32 frag_step;
	u32 headroom;
	u32 tailroom;

	lockdep_assert_held(&tp->control_lock);

	if (!tp || !pool || !frame_size_out)
		return -EINVAL;
	*frame_size_out = 0;
	if (!rtlmq_chip_supports_af_xdp(tp))
		return -EOPNOTSUPP;

	if (!tp->ndev ||
	    rtlmq_rx_geometry_normalize((u32)tp->ndev->mtu, &geometry))
		return -EINVAL;
	if (queue_index >= rtlmq_advertised_max_rx_queues(tp))
		return -EINVAL;
	if (!pool->heads || !pool->heads_cnt)
		return -EINVAL;

	/* Recheck UMEM geometry before saving or preparing a pool binding. */
	if (!rtlmq_xsk_pool_geometry_model_valid(pool))
		return -EINVAL;
	if (pool->umem) {
		if (!tp->ndev || pool->netdev != tp->ndev ||
		    pool->queue_id != queue_index)
			return -EINVAL;
		if (!!(pool->umem->flags & XDP_UMEM_UNALIGNED_CHUNK_FLAG) !=
		    pool->unaligned)
			return -EINVAL;
	}

	if (pool->xdp_zc_max_segs > rtlmq_xdp_zc_max_segs(tp))
		return -EOPNOTSUPP;
	if (pool->headroom > U32_MAX - geometry.xsk_headroom)
		return -EINVAL;

	chunk_size = xsk_pool_get_chunk_size(pool);
	headroom = geometry.xsk_headroom + pool->headroom;
#if IS_ENABLED(CONFIG_XDP_SOCKETS)
	tailroom = xsk_pool_get_tailroom(pool->dev && pool->umem &&
				       (pool->umem->flags & XDP_UMEM_SG_FLAG));
#else
	tailroom = 0;
#endif
	if (!chunk_size || headroom >= chunk_size ||
	    tailroom > chunk_size - headroom)
		return -EINVAL;

	frag_step = xsk_pool_get_rx_frag_step(pool);
	if (pool->unaligned) {
		if (frag_step)
			return -EINVAL;
	} else if (!is_power_of_2(chunk_size) || frag_step != chunk_size) {
		return -EINVAL;
	}

	/*
	 * Call the kernel helper only after proving that its unsigned
	 * headroom/tailroom subtractions cannot underflow.
	 */
	frame_size = xsk_pool_get_rx_frame_size(pool);
	if (!frame_size || !IS_ALIGNED(frame_size, 128))
		return -EINVAL;
	if (frame_size > rtlmq_max_xsk_frame_len())
		return -EOPNOTSUPP;
	if (frame_size > chunk_size - headroom - tailroom)
		return -EINVAL;
	*frame_size_out = frame_size;
	return 0;
}

bool rtlmq_requested_xsk_pool_bound_elsewhere_locked(const struct rtlmq_private *tp,
						     const struct xsk_buff_pool *pool,
						     u16 queue_index)
{
	u16 slot;

	lockdep_assert_held(&tp->control_lock);

	if (!tp || !pool)
		return false;

	for (slot = 0; slot < RTLMQ_IMPL_MAX_RX_QUEUES; slot++) {
		if (slot != queue_index &&
		    rtlmq_requested_xsk_pool(tp, slot) == pool)
			return true;
	}

	return false;
}

int rtlmq_validate_xsk_frame_sizes_locked(struct rtlmq_private *tp,
					  const struct rtlmq_rx_geometry *geometry,
					  u16 *conflict_queue, u32 *conflict_frame_size)
{
	u16 queue_index;
	int ret;

	lockdep_assert_held(&tp->control_lock);

	for (queue_index = 0; queue_index < RTLMQ_IMPL_MAX_RX_QUEUES;
	     queue_index++) {
		struct xsk_buff_pool *pool;
		u32 frame_size;

		pool = rtlmq_requested_xsk_pool(tp, queue_index);
		if (!pool)
			continue;
		if (rtlmq_requested_xsk_pool_bound_elsewhere_locked(tp, pool, queue_index)) {
			if (conflict_queue)
				*conflict_queue = queue_index;
			if (conflict_frame_size)
				*conflict_frame_size = 0;
			return -EBUSY;
		}

		ret = rtlmq_validate_xsk_pool_geometry_locked(tp, pool, queue_index, &frame_size);
		if (ret) {
			if (conflict_queue)
				*conflict_queue = queue_index;
			if (conflict_frame_size)
				*conflict_frame_size = frame_size;
			return ret;
		}
		if (pool->umem && !rtlmq_xsk_pool_dma_owned(tp, pool)) {
			if (conflict_queue)
				*conflict_queue = queue_index;
			if (conflict_frame_size)
				*conflict_frame_size = frame_size;
			return -EIO;
		}
		if (!rtlmq_rx_xsk_frame_size_valid_pool_geometry(tp, pool, frame_size, geometry)) {
			if (conflict_queue)
				*conflict_queue = queue_index;
			if (conflict_frame_size)
				*conflict_frame_size = frame_size;
			return frame_size > rtlmq_max_xsk_frame_len() ?
			       -EOPNOTSUPP : -EBUSY;
		}
	}

	return 0;
}
