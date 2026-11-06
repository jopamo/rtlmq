// SPDX-License-Identifier: GPL-2.0-only

#include <linux/rcupdate.h>
#include <linux/slab.h>
#include <linux/topology.h>

#include "rtlmq_hw.h"
#include "rtlmq_runtime_accessors.h"
#include "rtlmq.h"
#include "rtlmq_diag.h"
#include "rtlmq_lifecycle.h"
#include "rtlmq_regs.h"
#include "rtlmq_ring.h"
#include "rtlmq_runtime.h"
#include "rtlmq_rx.h"
#include "rtlmq_tx.h"
#include "rtlmq_xdp.h"

int rtlmq_validate_ring_counts(u32 rx_ring_count, u32 tx_ring_count)
{
	if (rx_ring_count < RTLMQ_MIN_RX_DESC ||
	    rx_ring_count > RTLMQ_MAX_RX_DESC ||
	    !is_power_of_2(rx_ring_count))
		return -EINVAL;

	if (tx_ring_count < RTLMQ_MIN_TX_DESC ||
	    tx_ring_count > RTLMQ_MAX_TX_DESC ||
	    !is_power_of_2(tx_ring_count))
		return -EINVAL;

	return 0;
}

static const struct rtlmq_runtime_config *
rtlmq_ring_build_config(const struct rtlmq_private *tp)
{
	const struct rtlmq_runtime_config *cfg;

	cfg = rtlmq_candidate_runtime_config(tp);
	if (cfg)
		return cfg;

	return rtlmq_active_or_requested_runtime_config(tp);
}

static void rtlmq_init_rx_ring_slot(struct rtlmq_private *tp,
				    struct rtlmq_rx_ring *ring,
				    unsigned int queue_index)
{
	const struct rtlmq_runtime_config *cfg = rtlmq_ring_build_config(tp);
	u16 ring_count = cfg ? cfg->rx_ring_count : RTLMQ_NUM_RX_DESC;

	ring->priv = tp;
	ring->ndev = tp->ndev;
	ring->napi = NULL;
	ring->page_pool = NULL;
	ring->ring_count = ring_count;
	ring->ring_mask = ring_count - 1;
	ring->refill_batch = ring_count;
	ring->rx_fast_skb_refill = false;
	ring->queue_index = queue_index;
	ring->vector_index = 0xffff;
	ring->cold = &tp->rx_ring_cold[queue_index];
	rtlmq_init_rx_desc_profile(ring);
}

void rtlmq_refresh_tx_ring_slot(struct rtlmq_private *tp,
				struct rtlmq_tx_ring *ring,
				unsigned int queue_index)
{
	u16 owned_vector_slots = rtlmq_owned_vector_slots(tp);
	const struct rtlmq_runtime_config *cfg = rtlmq_ring_build_config(tp);
	u16 ring_count = cfg ? cfg->tx_ring_count : RTLMQ_NUM_TX_DESC;

	ring->priv = tp;
	ring->ndev = tp->ndev;
	ring->doorbell = rtlmq_mmio_ordinary_authorized(tp) ?
		tp->mmio_addr + RTLMQ_TX_POLL : NULL;
	/*
	 * TPPOLL is a one-shot command, not an enabled-queue mask. Match the
	 * vendor BIT(ring->index) write and keep that value unchanged across
	 * completion-mode refreshes.
	 */
	ring->doorbell_value = queue_index < RTLMQ_TX_POLL_KICK_MAX_QUEUES ?
		(u16)(RTLMQ_TX_POLL_KICK_Q0 << queue_index) : 0;
	ring->tx_stop_desc_low = RTLMQ_TX_STOP_DESC_LOW;
	ring->tx_wake_desc_high = ring_count / 4;
	ring->ring_count = ring_count;
	ring->ring_mask = ring_count - 1;
	ring->queue_index = queue_index;
	ring->txq = netdev_get_tx_queue(ring->ndev, ring->queue_index);
	ring->desc_format = rtlmq_hw_tx_desc_fmt(tp);
	ring->desc_stride = rtlmq_hw_tx_desc_stride(tp);
	/*
	 * Check the saved vector index against allocated vector storage, not
	 * the hardware limit. Clear it if the vector no longer exists.
	 */
	if (!owned_vector_slots || ring->vector_index >= owned_vector_slots)
		ring->vector_index = 0xffff;
}

static u16 rtlmq_tx_ring_storage_slots(const struct rtlmq_private *tp)
{
	/*
	 * Size TX storage from the hardware limit, not current runtime queue
	 * counts. Supported user-visible queue counts are checked separately.
	 */
	return rtlmq_hw_max_tx_queues(tp);
}

int rtlmq_ring_preferred_nid(const struct rtlmq_private *tp)
{
	struct device *dev;
	int nid;

	if (!tp || !tp->pdev)
		return NUMA_NO_NODE;

	dev = &tp->pdev->dev;
	nid = dev_to_node(dev);
	return nid;
}

int rtlmq_sync_rx_desc_format(struct rtlmq_private *tp)
{
	unsigned int i;
	unsigned int j;
	size_t needed;
	u16 rx_slots;
	int ret;

	lockdep_assert_held(&tp->control_lock);

	/* Do not change RX descriptor format while OPEN. */
	if (WARN_ON_ONCE(READ_ONCE(tp->state) == RTLMQ_STATE_OPEN))
		return -EBUSY;

	if (!tp->rx_rings)
		return 0;

	rx_slots = rtlmq_owned_rx_ring_slots(tp);
	if (!rx_slots)
		return 0;

	ret = rtlmq_rx_desc_ring_size(tp->rx_rings[0].ring_count, READ_ONCE(tp->rx_desc_fmt),
				      &needed);
	if (ret)
		return ret;

	for (i = 0; i < rx_slots; i++) {
		struct rtlmq_rx_ring *ring = &tp->rx_rings[i];

		rtlmq_init_rx_desc_profile(ring);
		if (!ring->desc)
			continue;
		if (ring->desc_alloc_size < needed)
			return -EINVAL;

		rtlmq_reset_rx_ring_posting(ring);
		if (!ring->buffers)
			continue;

		for (j = 0; j < ring->ring_count; j++) {
			if (!ring->buffers[j].opaque)
				break;

			if (!rtlmq_publish_rx_desc(ring, j))
				return -EIO;
			ring->rx_posted++;
		}
		ring->next_to_alloc = ring->rx_posted & ring->ring_mask;
	}

	return 0;
}

int rtlmq_alloc_rings(struct rtlmq_private *tp)
{
	u16 max_rx_queues = rtlmq_hw_max_rx_queues(tp);
	u16 tx_ring_slots = rtlmq_tx_ring_storage_slots(tp);
	const struct rtlmq_runtime_config *cfg = rtlmq_active_or_requested_runtime_config(tp);
	unsigned int i;

	if (!rtlmq_hw_contract_present(tp) ||
	    !rtlmq_qualified_policy_present(tp) || !cfg ||
	    !cfg->num_rx_queues || !cfg->num_tx_queues)
		return -EINVAL;

	/*
	 * Allocate ring arrays at probe and free them at remove. Size them for
	 * the maximum supported storage, not current queue counts. Open/close
	 * and channel changes initialize only the selected subset.
	 */
	tp->rx_rings = kcalloc(max_rx_queues, sizeof(*tp->rx_rings),
			       GFP_KERNEL);
	if (!tp->rx_rings)
		return -ENOMEM;

	tp->rx_ring_cold = kcalloc(max_rx_queues,
				   sizeof(*tp->rx_ring_cold), GFP_KERNEL);
	if (!tp->rx_ring_cold)
		goto err_free_rx;

	tp->allocated_rx_ring_slots = max_rx_queues;

	for (i = 0; i < max_rx_queues; i++) {
		struct rtlmq_rx_ring *ring = &tp->rx_rings[i];

		u64_stats_init(&tp->rx_ring_cold[i].syncp);
		rtlmq_init_rx_ring_slot(tp, ring, i);
	}

	tp->tx_rings =
		kcalloc(tx_ring_slots, sizeof(*tp->tx_rings), GFP_KERNEL);
	if (!tp->tx_rings)
		goto err_free_debug;
	tp->allocated_tx_ring_slots = tx_ring_slots;

	for (i = 0; i < tx_ring_slots; i++) {
		u64_stats_init(&tp->tx_rings[i].syncp);
		u64_stats_init(&tp->tx_rings[i].comp_syncp);
		atomic64_set(&tp->tx_rings[i].stats.queue_stopped, 0);
		atomic64_set(&tp->tx_rings[i].stats.queue_wake, 0);
		rtlmq_refresh_tx_ring_slot(tp, &tp->tx_rings[i], i);
	}

	return 0;

err_free_debug:
	kfree(tp->tx_rings);
	tp->tx_rings = NULL;
	tp->allocated_tx_ring_slots = 0;
	kfree(tp->rx_ring_cold);
	tp->rx_ring_cold = NULL;
	tp->allocated_rx_ring_slots = 0;
err_free_rx:
	kfree(tp->rx_rings);
	tp->rx_rings = NULL;

	return -ENOMEM;
}

int rtlmq_init_rings(struct rtlmq_private *tp)
{
	const struct rtlmq_runtime_config *cfg = rtlmq_ring_build_config(tp);
	u16 num_rx_queues = rtlmq_candidate_or_published_num_rx_queues(tp);
	u16 num_tx_queues = rtlmq_candidate_or_published_num_tx_queues(tp);
	unsigned int i;
	int ret;

	if (!tp->rx_rings || !tp->tx_rings || !cfg)
		return -EINVAL;
	if (num_rx_queues > rtlmq_owned_rx_ring_slots(tp) ||
	    num_tx_queues > rtlmq_owned_tx_ring_slots(tp))
		return -EINVAL;

	WRITE_ONCE(tp->rx_desc_fmt, rtlmq_wanted_rx_desc_fmt(tp));

	for (i = 0; i < num_rx_queues; i++) {
		struct rtlmq_rx_ring *ring;
		struct xsk_buff_pool *xsk_pool =
			rtlmq_candidate_xsk_pool(tp, i);

		ring = rtlmq_prepare_candidate_rx_ring(tp, i);
		if (IS_ERR(ring))
			return PTR_ERR(ring);
		ring->ring_count = cfg->rx_ring_count;
		ring->ring_mask = ring->ring_count - 1;
		ring->refill_batch = ring->ring_count;

		/*
		 * Apply requested XSK bindings to private candidate rings only.
		 * The datapath keeps using committed ring->cold state until commit.
		 */
		ring->cold->xsk_pool = xsk_pool;
		ret = rtlmq_alloc_rx_ring(ring);
		if (ret)
			return ret;

		ret = rtlmq_init_rx_ring(ring);
		if (ret)
			return ret;
	}

	for (i = 0; i < num_tx_queues; i++) {
		struct rtlmq_tx_ring *ring = &tp->tx_rings[i];

		ret = rtlmq_alloc_tx_ring(ring);
		if (ret)
			return ret;

		ret = rtlmq_init_tx_ring(ring);
		if (ret)
			return ret;
	}

	return 0;
}

/*
 * Keep candidate RX rings and TX DMA handles in the transaction until commit
 * moves them to persistent ring slots. Derive active DMA masks from those
 * slots rather than maintain a second bitmap through rollback and teardown.
 * The descriptor pointer records allocation presence; DMA address zero is valid.
 */
u64 rtlmq_derive_active_rx_dma_ring_mask(const struct rtlmq_private *tp)
{
	unsigned int i;
	u64 mask = 0;

	if (!tp || !tp->rx_rings ||
	    !READ_ONCE(tp->active_resource_generation))
		return 0;

	for (i = 0; i < rtlmq_owned_rx_ring_slots(tp); i++) {
		const struct rtlmq_rx_ring *ring = &tp->rx_rings[i];

		if (READ_ONCE(ring->desc))
			mask |= BIT_ULL(i);
	}

	return mask;
}

u64 rtlmq_derive_active_tx_dma_ring_mask(const struct rtlmq_private *tp)
{
	unsigned int i;
	u64 mask = 0;

	if (!tp || !tp->tx_rings ||
	    !READ_ONCE(tp->active_resource_generation))
		return 0;

	for (i = 0; i < rtlmq_owned_tx_ring_slots(tp); i++) {
		const struct rtlmq_tx_ring *ring = &tp->tx_rings[i];

		if (READ_ONCE(ring->desc))
			mask |= BIT_ULL(i);
	}

	return mask;
}

void rtlmq_stop_rings(struct rtlmq_private *tp)
{
	u16 tx_slots = rtlmq_owned_tx_ring_slots(tp);
	u16 rx_slots = rtlmq_owned_rx_ring_slots(tp);
	unsigned int i;

	if (!tp || !rtlmq_release_allowed(tp))
		return;

	/*
	 * Repeated teardown is safe; partial-open rollback leaves empty rings
	 * for the next open.
	 */
	/*
	 * Queue-stat readers use RCU. Callers clear runtime_published first;
	 * wait here for existing readers before releasing ring storage.
	 */
	synchronize_rcu();
	rtlmq_mark_quiesce_phase(tp, RTLMQ_QUIESCE_RCU_DRAINED);
	rtlmq_retire_queue_stats(tp);
	rtlmq_clear_pending_tx_doorbells(tp);

	if (tp->tx_rings) {
		for (i = 0; i < tx_slots; i++)
			rtlmq_stop_tx_ring(&tp->tx_rings[i]);
	}

	if (tp->rx_rings) {
		for (i = 0; i < rx_slots; i++)
			rtlmq_stop_rx_ring(&tp->rx_rings[i]);
	}

	if (tp->tx_rings) {
		for (i = 0; i < tx_slots; i++)
			rtlmq_free_tx_ring(&tp->tx_rings[i]);
	}

	if (tp->rx_rings) {
		for (i = 0; i < rx_slots; i++)
			rtlmq_free_rx_ring(&tp->rx_rings[i]);
	}
}

void rtlmq_free_rings(struct rtlmq_private *tp)
{
	if (!tp || !rtlmq_release_allowed(tp))
		return;

	/*
	 * Discard transaction resources explicitly before stopping persistent
	 * active rings. Active ring cleanup does not free candidate resources.
	 */
	rtlmq_discard_candidate_tx_resources(tp);
	rtlmq_discard_candidate_rx_rings(tp);
	rtlmq_stop_rings(tp);
	rtlmq_release_active_rx_cold(tp);
	if (WARN_ON_ONCE(!rtlmq_ring_storage_release_allowed(tp)))
		return;

	kfree(tp->tx_rings);
	tp->tx_rings = NULL;
	tp->allocated_tx_ring_slots = 0;

	kfree(tp->rx_ring_cold);
	tp->rx_ring_cold = NULL;

	kfree(tp->rx_rings);
	tp->rx_rings = NULL;
	tp->allocated_rx_ring_slots = 0;
}
