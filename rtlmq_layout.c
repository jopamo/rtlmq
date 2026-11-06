// SPDX-License-Identifier: GPL-2.0-only

#include <linux/bitops.h>
#include <linux/kernel.h>
#include <linux/string.h>

#include "rtlmq_hw.h"
#include "rtlmq_runtime_accessors.h"
#include "rtlmq_irq.h"
#include "rtlmq_runtime.h"
#include "rtlmq.h"
#include "rtlmq_regs.h"
#include "rtlmq_xdp.h"

/*
 * Build and validate queue/vector routing for a runtime configuration.
 * Keep mapping changes here; callers request rebuilds or format summaries.
 */

static u32
rtlmq_vector_layout_irq_mask(const struct rtlmq_runtime_config *cfg,
			     unsigned int vector_index,
			     const struct rtlmq_vector_layout *vector)
{
	u32 mask = RTLMQ_INT_BOOT_MASK;
	unsigned int i;

	if (!rtlmq_vector_layout_published(vector))
		return mask;

	if (vector->num_rx_rings)
		mask |= RTLMQ_INT_RX_OK | RTLMQ_INT_RX_ERR |
			RTLMQ_INT_RX_DESC_UNAVAIL;

	if (vector->num_tx_rings)
		mask |= RTLMQ_INT_TX_OK | RTLMQ_INT_TX_ERR;

	for (i = 0; i < vector->num_irq_sources; i++) {
		const struct rtlmq_irq_source *source =
			&vector->irq_sources[i];

		if (source->role == RTLMQ_IRQ_SOURCE_EVENT)
			mask |= RTLMQ_INT_LINK_CHG;
	}

	/*
	 * Legacy/MSI vector 0 handles the whole shared status word, even when
	 * link state is polled and has no dedicated event source.
	 */
	if (cfg->routing_mode == RTLMQ_ROUTING_SINGLE_VECTOR &&
	    vector_index == 0)
		mask |= RTLMQ_INT_BOOTSTRAP_MASK;

	return mask;
}

static int
rtlmq_finalize_runtime_layout(const struct rtlmq_private *tp,
			      const struct rtlmq_runtime_config *cfg,
			      struct rtlmq_runtime_layout_state *layout)
{
	u32 assigned_sources = 0;
	unsigned int i;

	if (!tp || !cfg || !layout)
		return -EINVAL;

	for (i = 0; i < cfg->num_vectors; i++) {
		struct rtlmq_vector_layout *vector = &layout->vectors[i];
		unsigned int j;

		vector->irq_mask = rtlmq_vector_layout_irq_mask(cfg, i, vector);
		if (!rtlmq_vector_layout_published(vector))
			continue;

		if (!vector->irq_mask) {
			netdev_err(tp->ndev,
				   "vector %u published without interrupt-mask coverage\n",
				   i);
			return -EINVAL;
		}

		if (vector->num_rx_rings &&
		    !(vector->irq_mask &
		      (RTLMQ_INT_RX_OK | RTLMQ_INT_RX_ERR |
		       RTLMQ_INT_RX_DESC_UNAVAIL))) {
			netdev_err(tp->ndev,
				   "vector %u missing RX interrupt-mask coverage\n",
				   i);
			return -EINVAL;
		}

		if (vector->num_tx_rings &&
		    !(vector->irq_mask &
		      (RTLMQ_INT_TX_OK | RTLMQ_INT_TX_ERR))) {
			netdev_err(tp->ndev,
				   "vector %u missing TX interrupt-mask coverage\n",
				   i);
			return -EINVAL;
		}

		for (j = 0; j < vector->num_irq_sources; j++) {
			u8 message_id = vector->irq_sources[j].message_id;

			if (message_id >= BITS_PER_TYPE(assigned_sources) ||
			    assigned_sources & BIT(message_id)) {
				netdev_err(tp->ndev,
					   "MSI-X message %u has multiple service owners\n",
					   message_id);
				return -EINVAL;
			}
			assigned_sources |= BIT(message_id);
		}
	}

	return 0;
}

void rtlmq_reset_irq_sources(struct rtlmq_irq_source *sources,
			     unsigned int count)
{
	unsigned int i;

	for (i = 0; i < count; i++) {
		sources[i].irq = -1;
		sources[i].message_id = RTLMQ_IRQ_SOURCE_NONE;
		sources[i].role = RTLMQ_IRQ_SOURCE_DATA;
		sources[i].kind_flags = 0;
		sources[i].requested = false;
	}
}

static void
rtlmq_reset_vector_layout_irq_sources(struct rtlmq_vector_layout *vector)
{
	vector->num_irq_sources = 0;
	vector->msix_data_mask = 0;
	vector->msix_event_mask = 0;
	rtlmq_reset_irq_sources(vector->irq_sources,
				ARRAY_SIZE(vector->irq_sources));
}

static void rtlmq_reset_vector_layout(struct rtlmq_vector_layout *vector)
{
	vector->irq_mask = RTLMQ_INT_BOOTSTRAP_MASK;
	vector->num_rx_rings = 0;
	vector->num_tx_rings = 0;
	memset(vector->rx_ring_indices, 0xff, sizeof(vector->rx_ring_indices));
	memset(vector->tx_ring_indices, 0xff, sizeof(vector->tx_ring_indices));
	rtlmq_reset_vector_layout_irq_sources(vector);
}

static void
rtlmq_reset_runtime_layout(struct rtlmq_runtime_layout_state *layout)
{
	unsigned int i;

	memset(layout, 0, sizeof(*layout));
	memset(layout->rx_to_vector, 0xff, sizeof(layout->rx_to_vector));
	memset(layout->tx_to_vector, 0xff, sizeof(layout->tx_to_vector));
	for (i = 0; i < ARRAY_SIZE(layout->vectors); i++)
		rtlmq_reset_vector_layout(&layout->vectors[i]);
}

static bool rtlmq_needs_link_irq_source(const struct rtlmq_private *tp)
{
	/*
	 * Use a dedicated MAC link event when phylib is not polling this PHY.
	 */
	return tp->phydev && tp->phydev->irq != PHY_POLL;
}

static bool
rtlmq_wants_msix_message_routing_cfg(const struct rtlmq_private *tp,
				     const struct rtlmq_runtime_config *cfg)
{
	return cfg && rtlmq_hw_has_msix_message_routing(tp) &&
	       cfg->routing_mode == RTLMQ_ROUTING_RX_PER_VECTOR_TX_COUPLED;
}

static int rtlmq_add_vector_irq_source(struct rtlmq_vector_layout *vector,
				       u8 message_id,
				       enum rtlmq_irq_source_role role,
				       u8 kind_flags)
{
	struct rtlmq_irq_source *source;
	unsigned int i;

	if (message_id == RTLMQ_IRQ_SOURCE_NONE)
		return 0;

	for (i = 0; i < vector->num_irq_sources; i++) {
		source = &vector->irq_sources[i];
		if (source->message_id != message_id)
			continue;
		if (source->role == role) {
			source->kind_flags |= kind_flags;
			return 0;
		}
		return -EINVAL;
	}

	if (vector->num_irq_sources >= ARRAY_SIZE(vector->irq_sources))
		return -EINVAL;

	source = &vector->irq_sources[vector->num_irq_sources++];
	source->irq = -1;
	source->message_id = message_id;
	source->role = role;
	source->kind_flags = kind_flags;
	source->requested = false;

	if (role == RTLMQ_IRQ_SOURCE_DATA)
		vector->msix_data_mask |= BIT(message_id);
	else
		vector->msix_event_mask |= BIT(message_id);

	return 0;
}

static int
rtlmq_build_vector_irq_sources(const struct rtlmq_private *tp,
			       const struct rtlmq_runtime_config *cfg,
			       struct rtlmq_runtime_layout_state *layout)
{
	struct rtlmq_vector_layout *link_vector = &layout->vectors[0];
	unsigned int i;
	int ret;

	if (!rtlmq_wants_msix_message_routing_cfg(tp, cfg))
		return 0;

	for (i = 0; i < cfg->num_rx_queues; i++) {
		u16 vector_index = layout->rx_to_vector[i];
		struct rtlmq_vector_layout *vector;

		if (vector_index >= cfg->num_vectors)
			return -EINVAL;
		vector = &layout->vectors[vector_index];

		ret = rtlmq_add_vector_irq_source(vector,
						  rtlmq_hw_msix_rx_message_id(tp, i),
						  RTLMQ_IRQ_SOURCE_DATA,
						  RTLMQ_IRQ_SOURCE_KIND_RX);
		if (ret)
			return ret;
	}

	for (i = 0; i < cfg->num_tx_queues; i++) {
		u16 vector_index = layout->tx_to_vector[i];
		struct rtlmq_vector_layout *vector;

		if (vector_index >= cfg->num_vectors)
			return -EINVAL;
		vector = &layout->vectors[vector_index];

		ret = rtlmq_add_vector_irq_source(vector,
						  rtlmq_hw_msix_tx_message_id(tp, i),
						  RTLMQ_IRQ_SOURCE_DATA,
						  RTLMQ_IRQ_SOURCE_KIND_TX);
		if (ret)
			return ret;
	}

	if (rtlmq_needs_link_irq_source(tp)) {
		ret = rtlmq_add_vector_irq_source(link_vector,
						  rtlmq_hw_msix_link_message_id(tp),
						  RTLMQ_IRQ_SOURCE_EVENT,
						  RTLMQ_IRQ_SOURCE_KIND_EVENT);
		if (ret)
			return ret;
	}

	return 0;
}

static int
rtlmq_build_queue_service_owners(const struct rtlmq_private *tp,
				 const struct rtlmq_runtime_config *cfg,
				 struct rtlmq_runtime_layout_state *layout)
{
	const struct rtlmq_hw_contract *hw = rtlmq_selected_hw_contract(tp);
	bool owner_has_rx[RTLMQ_IMPL_MAX_NAPI] = {};
	u16 message_owner[BITS_PER_TYPE(u32)];
	u16 service_owners;
	bool dedicated_tx;
	unsigned int i;
	u16 tx_owner;

	if (cfg->routing_mode == RTLMQ_ROUTING_SINGLE_VECTOR) {
		for (i = 0; i < cfg->num_rx_queues; i++)
			layout->rx_to_vector[i] = 0;
		for (i = 0; i < cfg->num_tx_queues; i++)
			layout->tx_to_vector[i] = 0;
		return 0;
	}
	if (cfg->routing_mode != RTLMQ_ROUTING_RX_PER_VECTOR_TX_COUPLED ||
	    !hw)
		return -EINVAL;

	/*
	 * All TX message sources have one serialized owner. Build a bounded
	 * message-to-owner map once; an RX source that shares a message with TX
	 * necessarily joins that owner. Profiles without overlap may retain the
	 * existing extra TX-only owner.
	 */
	service_owners =
		rtlmq_msix_service_owner_count(hw, cfg->num_rx_queues,
					       cfg->num_tx_queues);
	if (!service_owners)
		return -EINVAL;
	dedicated_tx = cfg->num_vectors == service_owners + 1;
	if (cfg->num_vectors != service_owners && !dedicated_tx)
		return -EINVAL;
	if (dedicated_tx &&
	    rtlmq_msix_rx_tx_sources_overlap(hw, cfg->num_rx_queues,
					     cfg->num_tx_queues))
		return -EINVAL;

	tx_owner = dedicated_tx ? service_owners : 0;
	if (tx_owner >= cfg->num_vectors)
		return -EINVAL;

	memset(message_owner, 0xff, sizeof(message_owner));
	for (i = 0; i < cfg->num_tx_queues; i++) {
		u8 message_id = hw->msix_tx_message_ids[i];

		if (message_id >= ARRAY_SIZE(message_owner))
			return -EINVAL;
		message_owner[message_id] = tx_owner;
		layout->tx_to_vector[i] = tx_owner;
	}

	for (i = 0; i < cfg->num_rx_queues; i++) {
		u8 message_id = hw->msix_rx_message_ids[i];
		unsigned int candidate;
		u16 owner;

		if (message_id >= ARRAY_SIZE(message_owner))
			return -EINVAL;
		owner = message_owner[message_id];
		if (owner != U16_MAX) {
			layout->rx_to_vector[i] = owner;
			owner_has_rx[owner] = true;
			continue;
		}

		if (!dedicated_tx && !owner_has_rx[tx_owner]) {
			owner = tx_owner;
		} else {
			owner = U16_MAX;
			for (candidate = 0; candidate < cfg->num_vectors;
			     candidate++) {
				if (candidate == tx_owner ||
				    owner_has_rx[candidate])
					continue;
				owner = candidate;
				break;
			}
			if (owner == U16_MAX)
				return -EINVAL;
		}

		message_owner[message_id] = owner;
		layout->rx_to_vector[i] = owner;
		owner_has_rx[owner] = true;
	}

	return 0;
}

static int rtlmq_build_runtime_layout(const struct rtlmq_private *tp,
				      const struct rtlmq_runtime_config *cfg,
				      struct rtlmq_runtime_layout_state *layout)
{
	unsigned int i;
	int ret;

	if (!tp || !cfg || !layout)
		return -EINVAL;

	if (!cfg->num_vectors || cfg->num_vectors > ARRAY_SIZE(layout->vectors) ||
	    cfg->num_rx_queues > ARRAY_SIZE(layout->rx_to_vector) ||
	    cfg->num_tx_queues > ARRAY_SIZE(layout->tx_to_vector))
		return -EINVAL;

	rtlmq_reset_runtime_layout(layout);
	ret = rtlmq_build_queue_service_owners(tp, cfg, layout);
	if (ret)
		return ret;

	for (i = 0; i < cfg->num_rx_queues; i++) {
		u16 vector_index = layout->rx_to_vector[i];
		struct rtlmq_vector_layout *vector;

		if (vector_index >= cfg->num_vectors)
			return -EINVAL;

		vector = &layout->vectors[vector_index];
		if (vector->num_rx_rings >= ARRAY_SIZE(vector->rx_ring_indices))
			return -EINVAL;

		vector->rx_ring_indices[vector->num_rx_rings++] = i;
	}

	for (i = 0; i < cfg->num_tx_queues; i++) {
		u16 vector_index = layout->tx_to_vector[i];
		struct rtlmq_vector_layout *vector;

		if (vector_index >= cfg->num_vectors)
			return -EINVAL;

		vector = &layout->vectors[vector_index];
		if (vector->num_tx_rings >= ARRAY_SIZE(vector->tx_ring_indices))
			return -EINVAL;

		vector->tx_ring_indices[vector->num_tx_rings++] = i;
	}

	ret = rtlmq_build_vector_irq_sources(tp, cfg, layout);
	if (ret)
		return ret;

	ret = rtlmq_finalize_runtime_layout(tp, cfg, layout);
	if (ret)
		return ret;

	layout->valid = true;
	return 0;
}

int rtlmq_validate_candidate_layout(struct rtlmq_private *tp)
{
	const struct rtlmq_runtime_layout_state *layout;
	const struct rtlmq_runtime_config *cfg;
	u16 max_rx_queues;
	u16 max_tx_queues;
	unsigned int i;
	unsigned int j;

	if (!tp->transaction.profile.valid)
		return -EINVAL;

	cfg = &tp->transaction.profile.config;
	layout = rtlmq_candidate_runtime_layout(tp);
	if (!layout)
		return -EINVAL;

	max_rx_queues = rtlmq_hw_max_rx_queues(tp);
	max_tx_queues = rtlmq_hw_max_tx_queues(tp);

	if (!cfg->num_vectors || cfg->num_vectors > tp->allocated_vectors)
		return -EINVAL;
	for (i = 0; i < max_rx_queues; i++) {
		if (i < cfg->num_rx_queues) {
			if (layout->rx_to_vector[i] < cfg->num_vectors)
				continue;
			netdev_err(tp->ndev,
				   "candidate layout invalid: active rx ring %u has vector %u (vectors=%u)\n",
				   i, layout->rx_to_vector[i], cfg->num_vectors);
			return -EINVAL;
		}

		if (layout->rx_to_vector[i] == 0xffff)
			continue;
		netdev_err(tp->ndev,
			   "candidate layout invalid: inactive rx ring %u has vector %u\n",
			   i, layout->rx_to_vector[i]);
		return -EINVAL;
	}

	for (i = 0; i < max_tx_queues; i++) {
		if (i < cfg->num_tx_queues) {
			if (layout->tx_to_vector[i] < cfg->num_vectors)
				continue;
			netdev_err(tp->ndev,
				   "candidate layout invalid: active tx ring %u has vector %u (vectors=%u)\n",
				   i, layout->tx_to_vector[i], cfg->num_vectors);
			return -EINVAL;
		}

		if (layout->tx_to_vector[i] == 0xffff)
			continue;
		netdev_err(tp->ndev,
			   "candidate layout invalid: inactive tx ring %u has vector %u\n",
			   i, layout->tx_to_vector[i]);
		return -EINVAL;
	}

	for (i = 0; i < tp->allocated_vectors &&
	     i < ARRAY_SIZE(layout->vectors); i++) {
		const struct rtlmq_vector_layout *vector = &layout->vectors[i];

		if (vector->num_rx_rings > ARRAY_SIZE(vector->rx_ring_indices) ||
		    vector->num_tx_rings > ARRAY_SIZE(vector->tx_ring_indices))
			return -EINVAL;

		if (i >= cfg->num_vectors &&
		    (vector->num_rx_rings || vector->num_tx_rings)) {
			netdev_err(tp->ndev,
				   "candidate layout invalid: inactive vector %u references queues rx=%u tx=%u\n",
				   i, vector->num_rx_rings,
				   vector->num_tx_rings);
			return -EINVAL;
		}

		for (j = 0; j < vector->num_rx_rings; j++) {
			u16 ring_index = vector->rx_ring_indices[j];

			if (ring_index >= cfg->num_rx_queues) {
				netdev_err(tp->ndev,
					   "candidate layout invalid: vector %u references inactive rx ring %u\n",
					   i, ring_index);
				return -EINVAL;
			}
			if (layout->rx_to_vector[ring_index] == i)
				continue;

			netdev_err(tp->ndev,
				   "candidate layout invalid: vector %u references rx ring %u owned by vector %u\n",
				   i, ring_index,
				   layout->rx_to_vector[ring_index]);
			return -EINVAL;
		}

		for (j = 0; j < vector->num_tx_rings; j++) {
			u16 ring_index = vector->tx_ring_indices[j];

			if (ring_index >= cfg->num_tx_queues) {
				netdev_err(tp->ndev,
					   "candidate layout invalid: vector %u references inactive tx ring %u\n",
					   i, ring_index);
				return -EINVAL;
			}
			if (layout->tx_to_vector[ring_index] == i)
				continue;

			netdev_err(tp->ndev,
				   "candidate layout invalid: vector %u references tx ring %u owned by vector %u\n",
				   i, ring_index,
				   layout->tx_to_vector[ring_index]);
			return -EINVAL;
		}
	}

	return 0;
}

int rtlmq_rebuild_vector_layout(struct rtlmq_private *tp)
{
	struct rtlmq_runtime_layout_state layout;
	const struct rtlmq_runtime_config *cfg;
	struct rtlmq_runtime_layout_state *dst;
	int ret;

	if (rtlmq_candidate_runtime_config(tp)) {
		cfg = rtlmq_candidate_runtime_config(tp);
		dst = &tp->transaction.profile.layout;
	} else if (!tp->active_profile.valid) {
		cfg = rtlmq_requested_runtime_config(tp);
		dst = &tp->requested_profile.layout;
	} else {
		/*
		 * Rebuild the active profile and layout through a candidate
		 * transaction, not the requested-profile helper.
		 */
		return -EINVAL;
	}

	if (!tp->vectors || !tp->allocated_vectors || !cfg ||
	    !cfg->num_vectors || cfg->num_vectors > tp->allocated_vectors)
		return -EINVAL;

	ret = rtlmq_build_runtime_layout(tp, cfg, &layout);
	if (ret)
		return ret;

	*dst = layout;

	return 0;
}
