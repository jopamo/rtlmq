// SPDX-License-Identifier: GPL-2.0-only

#include <linux/slab.h>

#include "rtlmq_runtime.h"
#include "rtlmq_lifecycle.h"
#include "rtlmq_regs.h"
#include "rtlmq.h"
#include "rtlmq_ring.h"
#include "rtlmq_runtime_accessors.h"
#include "rtlmq_tx.h"
#include "rtlmq_xdp.h"

struct rtlmq_live_tx_ring_resources {
	struct rtlmq_tx_desc *desc;
	dma_addr_t desc_dma;
	struct rtlmq_tx_info *infos;
	struct rtlmq_tx_completion *completions;
	struct rtlmq_tx_dma_txn *dma_txns;
	u16 ring_count;
	u8 desc_stride;
};

struct rtlmq_live_tx_ring_set {
	struct rtlmq_live_tx_ring_resources rings[RTLMQ_IMPL_MAX_TX_QUEUES];
	u64 owned_mask;
};

static void rtlmq_stop_tx_ring_quiesced(struct rtlmq_tx_ring *ring);

static void rtlmq_tx_ring_publish_generation(struct rtlmq_tx_ring *ring)
{
	u32 gen;

	if (!ring)
		return;
	if (WARN_ON_ONCE(!rtlmq_generation_reuse_allowed(ring->priv)))
		return;

	gen = READ_ONCE(ring->generation) + 1;
	if (!gen)
		gen++;
	/* Publish initialized cursors before acquire readers observe the generation. */
	smp_store_release(&ring->generation, gen);
}

size_t rtlmq_tx_desc_ring_bytes_for_stride(u16 ring_count, u8 stride)
{
	if (!ring_count || ring_count > RTLMQ_MAX_TX_DESC)
		return 0;

	if (stride != 16 && stride != sizeof(struct rtlmq_tx_desc))
		return 0;
	return array_size(ring_count, stride);
}

size_t rtlmq_tx_ring_allocation_bytes_for_stride(u16 ring_count, u8 stride)
{
	size_t desc_bytes = rtlmq_tx_desc_ring_bytes_for_stride(ring_count,
								stride);
	size_t small_copy_bytes;

	if (!desc_bytes ||
	    check_mul_overflow((size_t)ring_count,
			       (size_t)RTLMQ_TX_SMALL_COPY_BYTES,
			       &small_copy_bytes) ||
	    check_add_overflow(desc_bytes, small_copy_bytes, &desc_bytes))
		return 0;

	return desc_bytes;
}

/*
 * Reset producer/consumer cursors and reclaim snapshots. Callers stop ring
 * access first and publish the new generation after initialization or teardown.
 */
static void rtlmq_tx_ring_reset_cursor_state(struct rtlmq_tx_ring *ring)
{
	WRITE_ONCE(ring->next_to_use, 0);
	WRITE_ONCE(ring->next_to_clean, 0);
	WRITE_ONCE(ring->next_to_use_count, 0);
	WRITE_ONCE(ring->next_to_clean_count, 0);
	WRITE_ONCE(ring->tx_reclaim_hint, 0);
	WRITE_ONCE(ring->own_recheck_reasons, 0);
}

struct rtlmq_tx_desc *
rtlmq_candidate_or_active_tx_desc(const struct rtlmq_private *tp,
				  u16 queue_index)
{
	if (!tp || queue_index >= rtlmq_owned_tx_ring_slots(tp))
		return NULL;
	if (tp->transaction.tx_descs[queue_index])
		return tp->transaction.tx_descs[queue_index];

	return tp->tx_rings[queue_index].desc;
}

dma_addr_t rtlmq_candidate_or_active_tx_desc_dma(const struct rtlmq_private *tp,
						 u16 queue_index)
{
	if (!tp || queue_index >= rtlmq_owned_tx_ring_slots(tp))
		return 0;
	if (tp->transaction.tx_descs[queue_index])
		return tp->transaction.tx_desc_dmas[queue_index];

	return tp->tx_rings[queue_index].desc_dma;
}

int rtlmq_alloc_tx_ring(struct rtlmq_tx_ring *ring)
{
	struct rtlmq_runtime_transaction *transaction;
	struct rtlmq_tx_completion *completions;
	const struct rtlmq_runtime_config *cfg;
	struct rtlmq_tx_dma_txn *dma_txns;
	struct rtlmq_tx_info *infos;
	struct rtlmq_private *tp;
	size_t allocation_bytes;
	struct device *dev;
	int ret = -ENOMEM;
	u16 queue_index;
	u16 ring_count;
	int nid;

	if (!ring)
		return -EINVAL;
	tp = ring->priv;
	if (!tp || !rtlmq_candidate_discard_allowed(tp))
		return -EBUSY;

	queue_index = ring->queue_index;
	if (queue_index >= rtlmq_owned_tx_ring_slots(tp))
		return -EINVAL;
	cfg = rtlmq_candidate_runtime_config(tp);
	if (!cfg || queue_index >= cfg->num_tx_queues ||
	    rtlmq_validate_ring_counts(cfg->rx_ring_count,
				       cfg->tx_ring_count))
		return -EINVAL;
	ring_count = cfg->tx_ring_count;
	transaction = &tp->transaction;
	dev = &tp->pdev->dev;
	nid = rtlmq_ring_preferred_nid(tp);
	allocation_bytes =
		rtlmq_tx_ring_allocation_bytes_for_stride(ring_count, rtlmq_hw_tx_desc_stride(tp));
	if (!allocation_bytes)
		return -EINVAL;

	if (transaction->tx_descs[queue_index] ||
	    transaction->tx_infos[queue_index] ||
	    transaction->tx_completions[queue_index] ||
	    transaction->tx_dma_txns[queue_index])
		return -EBUSY;

	/*
	 * Metadata and scatterlists need not be physically contiguous.
	 * Allow a vmalloc fallback for large rings.
	 */
	infos = kvcalloc_node(ring_count, sizeof(*infos), GFP_KERNEL, nid);
	if (!infos)
		return -ENOMEM;
	transaction->tx_infos[queue_index] = infos;

	completions = kvcalloc_node(ring_count, sizeof(*completions),
				    GFP_KERNEL, nid);
	if (!completions)
		goto err_free_infos;
	transaction->tx_completions[queue_index] = completions;

	dma_txns =
		kvcalloc_node(ring_count, sizeof(*dma_txns), GFP_KERNEL, nid);
	if (!dma_txns)
		goto err_free_completions;
	transaction->tx_dma_txns[queue_index] = dma_txns;

	transaction->tx_descs[queue_index] =
		dma_alloc_coherent(dev, allocation_bytes,
				   &transaction->tx_desc_dmas[queue_index],
				   GFP_KERNEL);
	if (!transaction->tx_descs[queue_index])
		goto err_free_dma_txns;
	if (!rtlmq_dma_range_fits_coherent(dev, transaction->tx_desc_dmas[queue_index],
					   allocation_bytes)) {
		dma_free_coherent(dev, allocation_bytes,
				  transaction->tx_descs[queue_index],
				  transaction->tx_desc_dmas[queue_index]);
		transaction->tx_descs[queue_index] = NULL;
		transaction->tx_desc_dmas[queue_index] = 0;
		ret = -EIO;
		goto err_free_dma_txns;
	}
	tp->transaction.tx_dma_ring_mask |= BIT_ULL(ring->queue_index);

	return 0;

err_free_dma_txns:
	kvfree(dma_txns);
	transaction->tx_dma_txns[queue_index] = NULL;
err_free_completions:
	kvfree(completions);
	transaction->tx_completions[queue_index] = NULL;
err_free_infos:
	kvfree(infos);
	transaction->tx_infos[queue_index] = NULL;
	return ret;
}

bool rtlmq_candidate_tx_storage_present(const struct rtlmq_private *tp,
					unsigned int queue_index)
{
	return tp->transaction.tx_descs[queue_index] ||
	       tp->transaction.tx_infos[queue_index] ||
	       tp->transaction.tx_completions[queue_index] ||
	       tp->transaction.tx_dma_txns[queue_index];
}

int rtlmq_validate_candidate_tx_storage(const struct rtlmq_private *tp,
					u16 num_tx_queues)
{
	unsigned int i;

	if (!tp || num_tx_queues > RTLMQ_IMPL_MAX_TX_QUEUES)
		return -EINVAL;

	for (i = 0; i < num_tx_queues; i++) {
		if (!tp->transaction.tx_descs[i] ||
		    !tp->transaction.tx_infos[i] ||
		    !tp->transaction.tx_completions[i] ||
		    !tp->transaction.tx_dma_txns[i])
			return -EINVAL;
	}

	return 0;
}

int rtlmq_init_tx_ring(struct rtlmq_tx_ring *ring)
{
	struct rtlmq_private *tp = ring ? ring->priv : NULL;
	struct rtlmq_runtime_transaction *transaction;
	const struct rtlmq_runtime_config *cfg;
	size_t small_copy_bytes;
	size_t ring_bytes;
	u16 queue_index;
	u8 *small_copy;
	u16 ring_count;

	if (!tp || !rtlmq_candidate_discard_allowed(tp))
		return -EBUSY;
	/*
	 * Initialize only the replacement storage. Leave the active ring's
	 * geometry, cursors, generation, and MMIO cache unchanged until commit.
	 */
	queue_index = ring->queue_index;
	cfg = rtlmq_candidate_runtime_config(tp);
	if (!cfg || queue_index >= cfg->num_tx_queues ||
	    rtlmq_validate_ring_counts(cfg->rx_ring_count,
				       cfg->tx_ring_count))
		return -EINVAL;
	ring_count = cfg->tx_ring_count;
	ring_bytes = rtlmq_tx_desc_ring_bytes_for_stride(ring_count, rtlmq_hw_tx_desc_stride(tp));
	small_copy_bytes = array_size(ring_count,
				      RTLMQ_TX_SMALL_COPY_BYTES);
	if (!ring_bytes || small_copy_bytes == SIZE_MAX)
		return -EINVAL;
	transaction = &tp->transaction;
	if (!transaction->tx_descs[queue_index] ||
	    !transaction->tx_infos[queue_index] ||
	    !transaction->tx_completions[queue_index] ||
	    !transaction->tx_dma_txns[queue_index])
		return -EINVAL;
	small_copy = (u8 *)transaction->tx_descs[queue_index] + ring_bytes;

	memset(transaction->tx_descs[queue_index], 0, ring_bytes);
	memset(small_copy, 0, small_copy_bytes);
	memset(transaction->tx_infos[queue_index], 0,
	       sizeof(*transaction->tx_infos[queue_index]) * ring_count);
	memset(transaction->tx_completions[queue_index], 0,
	       sizeof(*transaction->tx_completions[queue_index]) * ring_count);
	memset(transaction->tx_dma_txns[queue_index], 0,
	       sizeof(*transaction->tx_dma_txns[queue_index]) * ring_count);

	return 0;
}

static void rtlmq_tx_ring_reset_accounting(struct rtlmq_tx_ring *ring)
{
	ring->bql_queued_packets = 0;
	ring->bql_queued_bytes = 0;
	ring->bql_completed_packets = 0;
	ring->bql_completed_bytes = 0;
	ring->doorbells = 0;
}

static void rtlmq_init_committed_tx_ring(struct rtlmq_tx_ring *ring)
{
	u32 pre_use = READ_ONCE(ring->next_to_use_count);
	u32 pre_clean = READ_ONCE(ring->next_to_clean_count);
	u16 pre_ntu = READ_ONCE(ring->next_to_use);
	u16 pre_ntc = READ_ONCE(ring->next_to_clean);

	INIT_DELAYED_WORK(&ring->cold.tx_liveness_work,
			  rtlmq_tx_liveness_work_cb);
	WRITE_ONCE(ring->cold.liveness_work_initialized, true);
	WRITE_ONCE(ring->cold.liveness_work_generation, 0);
	rtlmq_tx_ring_reset_cursor_state(ring);
	rtlmq_serial_tx_init(ring);
	WRITE_ONCE(ring->tx_doorbell_pending, false);
	rtlmq_tx_ring_reset_accounting(ring);
	WRITE_ONCE(ring->tx_subqueue_state, RTLMQ_TX_SUBQUEUE_DISABLED);
	rtlmq_tx_ring_publish_generation(ring);

	WARN_ON(pre_ntu || pre_ntc || pre_use || pre_clean);
}

static void rtlmq_clear_transferred_tx_candidate(struct rtlmq_private *tp,
						 unsigned int queue_index)
{
	tp->transaction.tx_descs[queue_index] = NULL;
	tp->transaction.tx_desc_dmas[queue_index] = 0;
	tp->transaction.tx_infos[queue_index] = NULL;
	tp->transaction.tx_completions[queue_index] = NULL;
	tp->transaction.tx_dma_txns[queue_index] = NULL;
}

void rtlmq_commit_candidate_tx_resources(struct rtlmq_private *tp)
{
	u16 num_tx_queues;
	unsigned int i;

	if (!tp || !tp->tx_rings)
		return;
	if (!tp->transaction.resource_generation)
		return;

	num_tx_queues = tp->transaction.profile.config.num_tx_queues;
	for (i = 0; i < num_tx_queues; i++) {
		struct rtlmq_tx_ring *ring = &tp->tx_rings[i];

		if (!tp->transaction.tx_infos[i] ||
		    !tp->transaction.tx_completions[i] ||
		    !tp->transaction.tx_dma_txns[i] ||
		    !tp->transaction.tx_descs[i])
			continue;

		/*
		 * Install replacement storage and update ring geometry and state
		 * together.
		 */
		rtlmq_refresh_tx_ring_slot(tp, ring, i);
		ring->desc = tp->transaction.tx_descs[i];
		ring->desc_dma = tp->transaction.tx_desc_dmas[i];
		ring->small_copy =
			(u8 *)tp->transaction.tx_descs[i] +
			rtlmq_tx_desc_ring_bytes_for_stride(ring->ring_count, ring->desc_stride);
		ring->small_copy_dma =
			tp->transaction.tx_desc_dmas[i] +
			rtlmq_tx_desc_ring_bytes_for_stride(ring->ring_count, ring->desc_stride);
		ring->buf_infos = tp->transaction.tx_infos[i];
		ring->completions = tp->transaction.tx_completions[i];
		ring->dma_txns = tp->transaction.tx_dma_txns[i];
		rtlmq_clear_transferred_tx_candidate(tp, i);
		rtlmq_init_committed_tx_ring(ring);
	}
}

struct rtlmq_live_tx_ring_set *rtlmq_alloc_live_tx_ring_set(void)
{
	return kzalloc(sizeof(struct rtlmq_live_tx_ring_set), GFP_KERNEL);
}

int rtlmq_stop_active_tx_rings_for_live_swap(struct rtlmq_private *tp,
					     const struct rtlmq_live_service_pause *pause)
{
	const struct rtlmq_runtime_config *cfg;
	unsigned int i;

	if (!tp || !pause || !pause->mac_paused ||
	    pause->scope != RTLMQ_LIVE_MAC_PAUSE_ALL_ENGINES ||
	    !rtlmq_retired_generation_release_allowed(tp))
		return -EBUSY;
	cfg = rtlmq_active_runtime_config(tp);
	if (!cfg || !tp->tx_rings)
		return -EINVAL;

	for (i = 0; i < cfg->num_tx_queues; i++)
		rtlmq_stop_tx_ring_quiesced(&tp->tx_rings[i]);

	return 0;
}

static void
rtlmq_capture_live_tx_resources(struct rtlmq_live_tx_ring_resources *resources,
				const struct rtlmq_tx_ring *ring)
{
	resources->desc = ring->desc;
	resources->desc_dma = ring->desc_dma;
	resources->infos = ring->infos;
	resources->completions = ring->completions;
	resources->dma_txns = ring->dma_txns;
	resources->ring_count = ring->ring_count;
	resources->desc_stride = rtlmq_tx_ring_desc_stride(ring);
}

static void rtlmq_install_live_tx_resources(struct rtlmq_tx_ring *ring,
					    const struct rtlmq_live_tx_ring_resources *resources)
{
	size_t ring_bytes =
		rtlmq_tx_desc_ring_bytes_for_stride(resources->ring_count, resources->desc_stride);

	ring->tx_wake_desc_high = resources->ring_count / 4;
	ring->ring_count = resources->ring_count;
	ring->ring_mask = resources->ring_count - 1;
	ring->desc_stride = resources->desc_stride;
	ring->desc = resources->desc;
	ring->desc_dma = resources->desc_dma;
	ring->small_copy = (u8 *)resources->desc + ring_bytes;
	ring->small_copy_dma = resources->desc_dma + ring_bytes;
	ring->buf_infos = resources->infos;
	ring->completions = resources->completions;
	ring->dma_txns = resources->dma_txns;
}

int rtlmq_swap_live_candidate_tx_rings(struct rtlmq_private *tp,
				       struct rtlmq_live_tx_ring_set *retired)
{
	const struct rtlmq_runtime_config *cfg;
	unsigned int i;
	int ret;

	if (!tp || !tp->tx_rings || !retired || retired->owned_mask)
		return -EINVAL;
	if (!rtlmq_retired_generation_release_allowed(tp))
		return -EBUSY;
	cfg = rtlmq_candidate_runtime_config(tp);
	if (!cfg)
		return -EINVAL;

	ret = rtlmq_validate_candidate_tx_storage(tp, cfg->num_tx_queues);
	if (ret)
		return ret;

	for (i = 0; i < cfg->num_tx_queues; i++) {
		struct rtlmq_live_tx_ring_resources candidate = {
			.desc = tp->transaction.tx_descs[i],
			.desc_dma = tp->transaction.tx_desc_dmas[i],
			.infos = tp->transaction.tx_infos[i],
			.completions = tp->transaction.tx_completions[i],
			.dma_txns = tp->transaction.tx_dma_txns[i],
			.ring_count = cfg->tx_ring_count,
			.desc_stride = rtlmq_hw_tx_desc_stride(tp),
		};
		struct rtlmq_tx_ring *ring = &tp->tx_rings[i];

		rtlmq_capture_live_tx_resources(&retired->rings[i], ring);
		retired->owned_mask |= BIT_ULL(i);
		rtlmq_refresh_tx_ring_slot(tp, ring, i);
		rtlmq_install_live_tx_resources(ring, &candidate);
		rtlmq_clear_transferred_tx_candidate(tp, i);
		rtlmq_init_committed_tx_ring(ring);
	}
	tp->transaction.tx_dma_ring_mask = 0;

	return 0;
}

void rtlmq_rollback_live_tx_rings(struct rtlmq_private *tp,
				  struct rtlmq_live_tx_ring_set *retired)
{
	unsigned int i;

	if (!tp || !tp->tx_rings || !retired ||
	    !rtlmq_retired_generation_release_allowed(tp))
		return;

	for (i = 0; i < RTLMQ_IMPL_MAX_TX_QUEUES; i++) {
		struct rtlmq_live_tx_ring_resources replacement;
		struct rtlmq_tx_ring *ring;

		if (!(retired->owned_mask & BIT_ULL(i)))
			continue;
		ring = &tp->tx_rings[i];
		rtlmq_capture_live_tx_resources(&replacement, ring);
		rtlmq_refresh_tx_ring_slot(tp, ring, i);
		rtlmq_install_live_tx_resources(ring, &retired->rings[i]);
		retired->rings[i] = replacement;
		rtlmq_init_committed_tx_ring(ring);
	}
}

bool rtlmq_release_live_tx_ring_set(struct rtlmq_private *tp,
				    struct rtlmq_live_tx_ring_set *retired)
{
	struct device *dev;
	unsigned int i;

	if (!retired)
		return true;
	if (!retired->owned_mask) {
		kfree(retired);
		return true;
	}
	if (!tp || !rtlmq_retired_generation_release_allowed(tp))
		return false;
	dev = tp->pdev ? &tp->pdev->dev : NULL;

	for (i = 0; i < RTLMQ_IMPL_MAX_TX_QUEUES; i++) {
		struct rtlmq_live_tx_ring_resources *ring =
			&retired->rings[i];
		size_t allocation_bytes;

		if (!(retired->owned_mask & BIT_ULL(i)))
			continue;
		allocation_bytes = rtlmq_tx_ring_allocation_bytes_for_stride(ring->ring_count,
									     ring->desc_stride);
		if (ring->desc) {
			if (dev)
				dma_free_coherent(dev, allocation_bytes,
						  ring->desc,
						  ring->desc_dma);
			else
				kfree(ring->desc);
		}
		kvfree(ring->infos);
		kvfree(ring->completions);
		kvfree(ring->dma_txns);
		memset(ring, 0, sizeof(*ring));
		retired->owned_mask &= ~BIT_ULL(i);
	}

	if (retired->owned_mask)
		return false;
	kfree(retired);
	return true;
}

void rtlmq_discard_candidate_tx_resources(struct rtlmq_private *tp)
{
	struct device *dev;
	unsigned int i;

	if (!tp || !rtlmq_candidate_discard_allowed(tp))
		return;
	dev = tp->pdev ? &tp->pdev->dev : NULL;

	for (i = 0; i < RTLMQ_IMPL_MAX_TX_QUEUES; i++) {
		if (tp->transaction.tx_descs[i]) {
			const struct rtlmq_runtime_config *cfg =
				rtlmq_candidate_runtime_config(tp);
			size_t allocation_bytes = cfg ?
				rtlmq_tx_ring_allocation_bytes_for_stride(
					cfg->tx_ring_count,
					rtlmq_hw_tx_desc_stride(tp)) : 0;

			if (dev)
				dma_free_coherent(dev, allocation_bytes,
						  tp->transaction.tx_descs[i],
						  tp->transaction.tx_desc_dmas[i]);
			else
				kfree(tp->transaction.tx_descs[i]);
			tp->transaction.tx_descs[i] = NULL;
			tp->transaction.tx_desc_dmas[i] = 0;
		}
		kvfree(tp->transaction.tx_infos[i]);
		tp->transaction.tx_infos[i] = NULL;
		kvfree(tp->transaction.tx_completions[i]);
		tp->transaction.tx_completions[i] = NULL;
		kvfree(tp->transaction.tx_dma_txns[i]);
		tp->transaction.tx_dma_txns[i] = NULL;
	}

	/*
	 * This mask tracks uncommitted allocations only. Unwinding them must
	 * not change storage already installed in the active rings.
	 */
	tp->transaction.tx_dma_ring_mask = 0;
}

void rtlmq_clear_pending_tx_doorbells(struct rtlmq_private *tp)
{
	unsigned int i;
	u16 tx_slots;

	if (!tp || !tp->tx_rings)
		return;

	tx_slots = rtlmq_owned_tx_ring_slots(tp);
	for (i = 0; i < tx_slots; i++)
		WRITE_ONCE(tp->tx_rings[i].tx_doorbell_pending, false);
}

static void rtlmq_stop_tx_ring_quiesced(struct rtlmq_tx_ring *ring)
{
	bool has_resources;

	rtlmq_serial_tx_pause(ring->priv);
	/*
	 * Stop the liveness backstop before cursor state is torn down so a
	 * concurrently running handler cannot force a resweep against a
	 * closing ring.
	 */
	rtlmq_tx_ring_cancel_liveness(ring);
	WRITE_ONCE(ring->tx_doorbell_pending, false);
	has_resources = ring->desc || ring->infos || ring->completions ||
			ring->dma_txns || ring->cold.xsk_batch.pool;
	if (!has_resources)
		return;

	WRITE_ONCE(ring->tx_subqueue_state, RTLMQ_TX_SUBQUEUE_DISABLED);
	{
		/* CQ and DMA lifetimes retire in acceptance order, not slot order. */
		rtlmq_serial_tx_cleanup_published(ring);
		rtlmq_serial_tx_cancel_staged(ring);
		rtlmq_serial_tx_init(ring);
	}
	rtlmq_tx_ring_reset_cursor_state(ring);
	rtlmq_tx_ring_publish_generation(ring);

	rtlmq_xsk_cancel_tx_batch(ring);

	rtlmq_tx_ring_reset_accounting(ring);

	/* Free slots may retain stale OWN-clear data; reuse overwrites it. */
	if (ring->txq)
		netdev_tx_reset_queue(ring->txq);
}

void rtlmq_stop_tx_ring(struct rtlmq_tx_ring *ring)
{
	if (!ring || !rtlmq_release_allowed(ring->priv))
		return;

	rtlmq_stop_tx_ring_quiesced(ring);
}

void rtlmq_free_tx_ring(struct rtlmq_tx_ring *ring)
{
	struct rtlmq_private *tp = ring ? ring->priv : NULL;
	struct device *dev = NULL;
	size_t allocation_bytes;

	if (!tp)
		return;
	allocation_bytes =
		rtlmq_tx_ring_allocation_bytes_for_stride(ring->ring_count,
							  rtlmq_tx_ring_desc_stride(ring));
	if (!rtlmq_release_allowed(tp))
		return;

	if (tp->pdev)
		dev = &tp->pdev->dev;

	rtlmq_stop_tx_ring(ring);

	if (ring->desc) {
		if (dev)
			dma_free_coherent(dev, allocation_bytes, ring->desc,
					  ring->desc_dma);
		else
			kfree(ring->desc);
		ring->desc = NULL;
		ring->desc_dma = 0;
	}
	ring->small_copy = NULL;
	ring->small_copy_dma = 0;

	kvfree(ring->buf_infos);
	ring->buf_infos = NULL;
	kvfree(ring->completions);
	ring->completions = NULL;
	kvfree(ring->dma_txns);
	ring->dma_txns = NULL;
	WRITE_ONCE(ring->tx_subqueue_state, RTLMQ_TX_SUBQUEUE_DISABLED);
	ring->doorbell = NULL;
}
