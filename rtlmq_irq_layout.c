// SPDX-License-Identifier: GPL-2.0-only

#include <linux/smp.h>

#include "rtlmq_irq.h"
#include "rtlmq_lifecycle.h"
#include "rtlmq_regs.h"
#include "rtlmq_ring.h"

static void rtlmq_vector_publish_generation(struct rtlmq_vector *vector)
{
	u32 gen;

	if (!vector)
		return;
	if (WARN_ON_ONCE(!rtlmq_generation_reuse_allowed(vector->priv)))
		return;

	gen = READ_ONCE(vector->generation) + 1;
	if (!gen)
		gen = 1;

	/* Invalidate rearm snapshots from the previous vector mapping. */
	smp_store_release(&vector->generation, gen);
}

static int rtlmq_validate_vector_queue_layout(const struct rtlmq_private *tp,
					      const struct rtlmq_runtime_config *cfg,
					      const struct rtlmq_runtime_layout_state *layout)
{
	u16 num_rx_queues;
	u16 num_tx_queues;
	u16 num_vectors;
	unsigned int i;

	if (!cfg || !layout || !layout->valid || !tp->vectors)
		return -EINVAL;
	if (!tp->rx_rings || !tp->tx_rings)
		return -EINVAL;
	if (cfg->num_vectors > tp->allocated_vectors)
		return -EINVAL;
	if (cfg->num_rx_queues > rtlmq_owned_rx_ring_slots(tp) ||
	    cfg->num_tx_queues > rtlmq_owned_tx_ring_slots(tp))
		return -EINVAL;

	num_vectors = cfg->num_vectors;
	num_rx_queues = cfg->num_rx_queues;
	num_tx_queues = cfg->num_tx_queues;

	for (i = 0; i < num_rx_queues; i++) {
		if (layout->rx_to_vector[i] < num_vectors)
			continue;

		return -EINVAL;
	}

	for (i = 0; i < num_tx_queues; i++) {
		if (layout->tx_to_vector[i] < num_vectors)
			continue;

		return -EINVAL;
	}

	for (i = 0; i < num_vectors; i++) {
		const struct rtlmq_vector_layout *vector = &layout->vectors[i];

		if (vector->num_rx_rings > ARRAY_SIZE(vector->rx_ring_indices) ||
		    vector->num_tx_rings > ARRAY_SIZE(vector->tx_ring_indices) ||
		    vector->num_irq_sources > ARRAY_SIZE(vector->irq_sources))
			return -EINVAL;
	}

	return 0;
}

int rtlmq_validate_runtime_queue_ownership(struct rtlmq_private *tp,
					   const struct rtlmq_runtime_config *cfg,
					   const struct rtlmq_runtime_layout_state *layout)
{
	ASSERT_RTNL();

	if (rtlmq_validate_napi_bindings_layout(tp, cfg, layout))
		return -EINVAL;

	return rtlmq_validate_vector_queue_layout(tp, cfg, layout);
}

void rtlmq_reset_vector_irq_sources(struct rtlmq_vector *vector)
{
	vector->num_irq_sources = 0;
	vector->msix_data_mask = 0;
	vector->msix_event_mask = 0;
	rtlmq_reset_irq_sources(vector->irq_sources,
				ARRAY_SIZE(vector->irq_sources));
}

static void rtlmq_reset_vector_irq_routing(struct rtlmq_vector *vector)
{
	if (!vector ||
	    !rtlmq_generation_reuse_allowed(vector->priv))
		return;

	rtlmq_vector_publish_generation(vector);
	vector->irq_mask = RTLMQ_INT_BOOTSTRAP_MASK;
	vector->uses_msix_message_routing = false;
	vector->tx_irq_armed_wanted = false;
	vector->napi_service_pending = false;
	vector->tx_irq_mask = 0;
	vector->data_irq_mask_current = 0;
	vector->data_irq_mask_wanted = 0;
	vector->data_irq_mask_rx_only = 0;
	vector->data_irq_mask_tx_only = 0;
	vector->data_irq_mask_rx_tx = 0;
	vector->tx_resweep_hints = 0;
	vector->tx_poll_seq = 0;
	vector->rx_poll_start = 0;
	vector->xsk_tx_poll_start = 0;
	WRITE_ONCE(vector->xdp_tx_pending_mask, 0);
	WRITE_ONCE(vector->rx_service_irq_cause_pending, 0UL);
	WRITE_ONCE(vector->rx_own_recheck_pending_mask, 0UL);
	rtlmq_reset_vector_irq_sources(vector);
}

static void rtlmq_reset_vector_ring_map_state(struct rtlmq_vector *vector)
{
	if (!vector ||
	    !rtlmq_generation_reuse_allowed(vector->priv))
		return;

	rtlmq_vector_publish_generation(vector);
	vector->num_rx_rings = 0;
	vector->num_tx_rings = 0;
	vector->tx_irq_armed_wanted = false;
	vector->napi_service_pending = false;
	rtlmq_vector_refresh_data_irq_masks(vector);
	vector->data_irq_mask_wanted =
		rtlmq_vector_data_irq_mask_wanted_cached(vector, false);
	vector->tx_resweep_hints = 0;
	vector->tx_poll_seq = 0;
	vector->rx_poll_start = 0;
	vector->xsk_tx_poll_start = 0;
	WRITE_ONCE(vector->rx_service_irq_cause_pending, 0UL);
	WRITE_ONCE(vector->rx_own_recheck_pending_mask, 0UL);
	memset(vector->rx_ring_indices, 0xff, sizeof(vector->rx_ring_indices));
	memset(vector->tx_ring_indices, 0xff, sizeof(vector->tx_ring_indices));
	memset(vector->rx_message_ids, RTLMQ_IRQ_SOURCE_NONE,
	       sizeof(vector->rx_message_ids));
	memset(vector->tx_message_ids, RTLMQ_IRQ_SOURCE_NONE,
	       sizeof(vector->tx_message_ids));
}

void rtlmq_reset_vector_queue_map(struct rtlmq_vector *vector)
{
	if (!vector ||
	    !rtlmq_generation_reuse_allowed(vector->priv))
		return;

	vector->irq_mask = RTLMQ_INT_BOOTSTRAP_MASK;
	rtlmq_reset_vector_ring_map_state(vector);
	rtlmq_reset_vector_irq_sources(vector);
	vector->tx_irq_mask = 0;
	vector->data_irq_mask_current = 0;
	vector->data_irq_mask_wanted = 0;
	vector->data_irq_mask_rx_only = 0;
	vector->data_irq_mask_tx_only = 0;
	vector->data_irq_mask_rx_tx = 0;
}

static void rtlmq_reset_vector_ring_map(struct rtlmq_vector *vector)
{
	rtlmq_reset_vector_ring_map_state(vector);
}

static void rtlmq_reset_ring_vector_indices(struct rtlmq_private *tp,
					    u16 max_rx_queues,
					    u16 max_tx_queues)
{
	unsigned int i;

	if (tp->rx_rings) {
		for (i = 0; i < max_rx_queues; i++)
			tp->rx_rings[i].vector_index = 0xffff;
	}

	if (tp->tx_rings) {
		for (i = 0; i < max_tx_queues; i++)
			tp->tx_rings[i].vector_index = 0xffff;
	}
}

void rtlmq_clear_unpublished_vector_queue_maps(struct rtlmq_private *tp)
{
	unsigned int i;

	if (!tp->vectors || !rtlmq_hw_contract_present(tp))
		return;

	for (i = 0; i < tp->allocated_vectors; i++) {
		struct rtlmq_vector *vector = &tp->vectors[i];

		if (vector->irq_requested || vector->napi_enabled)
			continue;

		rtlmq_reset_vector_queue_map(vector);
	}

	rtlmq_reset_ring_vector_indices(tp, rtlmq_owned_rx_ring_slots(tp),
					rtlmq_owned_tx_ring_slots(tp));
}

static void
rtlmq_apply_vector_queue_layout(struct rtlmq_vector *vector,
				const struct rtlmq_vector_layout *layout)
{
	unsigned int i;

	if (!vector ||
	    !rtlmq_generation_reuse_allowed(vector->priv))
		return;

	memcpy(vector->rx_ring_indices, layout->rx_ring_indices,
	       sizeof(vector->rx_ring_indices));
	memcpy(vector->tx_ring_indices, layout->tx_ring_indices,
	       sizeof(vector->tx_ring_indices));
	for (i = 0; i < ARRAY_SIZE(vector->rx_message_ids); i++)
		vector->rx_message_ids[i] =
			i < layout->num_rx_rings ?
				rtlmq_hw_msix_rx_message_id(vector->priv,
							    layout->rx_ring_indices[i]) :
				RTLMQ_IRQ_SOURCE_NONE;
	for (i = 0; i < ARRAY_SIZE(vector->tx_message_ids); i++)
		vector->tx_message_ids[i] =
			i < layout->num_tx_rings ?
				rtlmq_hw_msix_tx_message_id(vector->priv,
							    layout->tx_ring_indices[i]) :
				RTLMQ_IRQ_SOURCE_NONE;
	vector->num_rx_rings = layout->num_rx_rings;
	vector->num_tx_rings = layout->num_tx_rings;
	rtlmq_vector_publish_generation(vector);
}

static void
rtlmq_apply_vector_irq_layout(struct rtlmq_vector *vector,
			      const struct rtlmq_vector_layout *layout)
{
	if (!vector ||
	    !rtlmq_generation_reuse_allowed(vector->priv))
		return;

	vector->irq_mask = layout->irq_mask;
	vector->num_irq_sources = layout->num_irq_sources;
	vector->msix_data_mask = layout->msix_data_mask;
	vector->msix_event_mask = layout->msix_event_mask;
	rtlmq_reset_irq_sources(vector->irq_sources,
				ARRAY_SIZE(vector->irq_sources));
	memcpy(vector->irq_sources, layout->irq_sources,
	       sizeof(layout->irq_sources));
	vector->uses_msix_message_routing =
		rtlmq_candidate_uses_msix_message_routing(vector->priv);
	vector->tx_irq_mask = rtlmq_vector_tx_irq_mask(vector);
	rtlmq_vector_refresh_data_irq_masks(vector);
	vector->tx_irq_armed_wanted = false;
	vector->napi_service_pending = false;
	vector->data_irq_mask_current = 0;
	vector->data_irq_mask_wanted =
		rtlmq_vector_data_irq_mask_wanted_cached(vector, false);
}

static int rtlmq_validate_staged_vector_irq_layout(const struct rtlmq_private *tp,
						   const struct rtlmq_runtime_config *cfg,
						   const struct rtlmq_runtime_layout_state *layout)
{
	bool uses_msix_message_routing;
	unsigned int i;

	if (!tp || !cfg || !layout || !layout->valid || !tp->vectors)
		return -EINVAL;

	uses_msix_message_routing =
		rtlmq_candidate_uses_msix_message_routing(tp);
	for (i = 0; i < cfg->num_vectors; i++) {
		const struct rtlmq_vector_layout *expected =
			&layout->vectors[i];
		const struct rtlmq_vector *vector = &tp->vectors[i];

		if (vector->irq_mask != expected->irq_mask ||
		    vector->num_irq_sources != expected->num_irq_sources ||
		    vector->msix_data_mask != expected->msix_data_mask ||
		    vector->msix_event_mask != expected->msix_event_mask ||
		    vector->uses_msix_message_routing !=
			    uses_msix_message_routing ||
		    memcmp(vector->irq_sources, expected->irq_sources,
			   sizeof(expected->irq_sources)))
			return -EINVAL;
	}

	return 0;
}

static int
rtlmq_validate_staged_vector_queue_layout(const struct rtlmq_private *tp,
					  const struct rtlmq_runtime_config *cfg,
					  const struct rtlmq_runtime_layout_state *layout)
{
	unsigned int i;

	if (!tp || !cfg || !layout || !layout->valid || !tp->vectors)
		return -EINVAL;

	for (i = 0; i < tp->allocated_vectors; i++) {
		const struct rtlmq_vector *vector = &tp->vectors[i];

		if (i < cfg->num_vectors) {
			const struct rtlmq_vector_layout *expected =
				&layout->vectors[i];

			if (vector->num_rx_rings != expected->num_rx_rings ||
			    vector->num_tx_rings != expected->num_tx_rings ||
			    memcmp(vector->rx_ring_indices,
				   expected->rx_ring_indices,
				   sizeof(vector->rx_ring_indices)) ||
			    memcmp(vector->tx_ring_indices,
				   expected->tx_ring_indices,
				   sizeof(vector->tx_ring_indices)))
				return -EINVAL;
			continue;
		}

		if (vector->num_rx_rings || vector->num_tx_rings)
			return -EINVAL;
	}

	return 0;
}

int rtlmq_stage_candidate_irq_routing(struct rtlmq_private *tp,
				      const struct rtlmq_runtime_config *cfg,
				      const struct rtlmq_runtime_layout_state *layout)
{
	unsigned int i;
	int ret;

	if (!cfg || !layout || !layout->valid || !tp->vectors)
		return -EINVAL;
	if (cfg->num_vectors > tp->allocated_vectors)
		return -EINVAL;
	ret = rtlmq_validate_vector_queue_layout(tp, cfg, layout);
	if (ret)
		return ret;
	if (!rtlmq_generation_reuse_allowed(tp))
		return -EBUSY;

	/*
	 * Build vector metadata with DMA stopped and before request_irq().
	 * Leave ring->vector, queue->NAPI, txq2ring, and the active profile
	 * unchanged until commit. Do not reuse generation storage after
	 * enabling bus mastering.
	 */
	for (i = 0; i < tp->allocated_vectors; i++) {
		rtlmq_reset_vector_irq_routing(&tp->vectors[i]);
		rtlmq_reset_vector_ring_map(&tp->vectors[i]);
	}

	for (i = 0; i < cfg->num_vectors; i++) {
		rtlmq_apply_vector_irq_layout(&tp->vectors[i],
					      &layout->vectors[i]);
		rtlmq_apply_vector_queue_layout(&tp->vectors[i],
						&layout->vectors[i]);
	}

	ret = rtlmq_validate_staged_vector_irq_layout(tp, cfg, layout);
	if (ret)
		return ret;

	return rtlmq_validate_staged_vector_queue_layout(tp, cfg, layout);
}

/*
 * Install queue-to-vector mappings only. Preserve IRQ routing and request
 * state from open_irqs() until close_irqs() frees every requested source
 * before pci_free_irq_vectors().
 */
int rtlmq_publish_vector_queue_layout(struct rtlmq_private *tp,
				      const struct rtlmq_runtime_config *cfg,
				      const struct rtlmq_runtime_layout_state *layout)
{
	unsigned int i;
	int ret;

	ret = rtlmq_validate_vector_queue_layout(tp, cfg, layout);
	if (ret)
		return ret;

	/*
	 * Normal open builds this map before DMA. Cold setup may enter without
	 * it; build it only while generation reuse is safe.
	 */
	ret = rtlmq_validate_staged_vector_queue_layout(tp, cfg, layout);
	if (ret) {
		if (!rtlmq_generation_reuse_allowed(tp))
			return -EBUSY;
		for (i = 0; i < tp->allocated_vectors; i++)
			rtlmq_reset_vector_ring_map(&tp->vectors[i]);
		for (i = 0; i < cfg->num_vectors; i++)
			rtlmq_apply_vector_queue_layout(&tp->vectors[i], &layout->vectors[i]);
		ret = rtlmq_validate_staged_vector_queue_layout(tp, cfg, layout);
		if (ret)
			return ret;
	}

	rtlmq_reset_ring_vector_indices(tp, rtlmq_owned_rx_ring_slots(tp),
					rtlmq_owned_tx_ring_slots(tp));
	memset(tp->txq2ring, 0, sizeof(tp->txq2ring));

	for (i = 0; i < cfg->num_rx_queues; i++)
		tp->rx_rings[i].vector_index = layout->rx_to_vector[i];

	for (i = 0; i < cfg->num_tx_queues; i++) {
		tp->tx_rings[i].vector_index = layout->tx_to_vector[i];
		tp->txq2ring[i] = &tp->tx_rings[i];
	}

	return 0;
}

int rtlmq_publish_runtime_queue_ownership(struct rtlmq_private *tp,
					  const struct rtlmq_runtime_config *cfg,
					  const struct rtlmq_runtime_layout_state *layout)
{
	int ret;

	ASSERT_RTNL();

	ret = rtlmq_validate_runtime_queue_ownership(tp, cfg, layout);
	if (ret)
		return ret;

	/*
	 * Validate both maps before changing either one. The install helpers
	 * validate again, but RTNL and control_lock prevent intervening changes.
	 */
	ret = rtlmq_publish_napi_bindings_locked(tp, cfg, layout);
	if (ret)
		return ret;

	ret = rtlmq_publish_vector_queue_layout(tp, cfg, layout);
	if (ret)
		rtlmq_withdraw_napi_bindings(tp);

	return ret;
}

void rtlmq_withdraw_runtime_queue_ownership(struct rtlmq_private *tp)
{
	unsigned int i;

	ASSERT_RTNL();

	if (tp->vectors)
		rtlmq_withdraw_napi_bindings(tp);

	rtlmq_clear_pending_tx_doorbells(tp);

	if (!tp->vectors)
		return;

	rtlmq_reset_ring_vector_indices(tp, rtlmq_owned_rx_ring_slots(tp),
					rtlmq_owned_tx_ring_slots(tp));
	memset(tp->txq2ring, 0, sizeof(tp->txq2ring));

	for (i = 0; i < tp->allocated_vectors; i++)
		rtlmq_reset_vector_ring_map(&tp->vectors[i]);
}
