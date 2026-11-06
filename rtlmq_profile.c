// SPDX-License-Identifier: GPL-2.0-only

#include <linux/bitops.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#if defined(MODULE) && defined(RTLMQ_HAVE_UTSVERSION)
#include <generated/utsversion.h>
#define RTLMQ_KERNEL_BUILD_ID	UTS_VERSION
#else
#define RTLMQ_KERNEL_BUILD_ID	"unavailable"
#endif

#include "rtlmq_hw.h"
#include "rtlmq_runtime_accessors.h"
#include "rtlmq_runtime.h"
#include "rtlmq_irq.h"
#include "rtlmq_ring.h"
#include "rtlmq.h"
#include "rtlmq_xdp.h"

/*
 * Select supported topologies from chip profiles, subject to hardware limits.
 * Runtime commit and publication are handled separately.
 */

/* Build profile and target-kernel identity embedded in the module. */
MODULE_INFO(rtlmq_build_profile, "production");
MODULE_INFO(rtlmq_kernel_build_id, RTLMQ_KERNEL_BUILD_ID);
MODULE_INFO(rtlmq_kernel_config_sha256, RTLMQ_KERNEL_CONFIG_SHA256);

u16 rtlmq_qualified_topology_count(const struct rtlmq_private *tp)
{
	const struct rtlmq_qualified_policy *qualified =
		rtlmq_qualified_policy_for(tp);

	return qualified && qualified->topologies ? qualified->num_topologies :
						    0;
}

const struct rtlmq_topology *
rtlmq_find_qualified_topology(const struct rtlmq_private *tp, u16 rx_queues,
			      u16 tx_queues)
{
	u16 i;

	for (i = 0; i < rtlmq_qualified_topology_count(tp); i++) {
		const struct rtlmq_topology *topology =
			rtlmq_qualified_topology_at(tp, i);

		if (topology && topology->rx_queues == rx_queues &&
		    topology->tx_queues == tx_queues)
			return topology;
	}

	return NULL;
}

const struct rtlmq_topology *
rtlmq_qualified_default_topology(const struct rtlmq_private *tp)
{
	const struct rtlmq_qualified_policy *qualified =
		rtlmq_qualified_policy_for(tp);
	u16 i;

	if (!qualified || !qualified->default_topology)
		return NULL;

	/* The default must appear in the supported topology table. */
	for (i = 0; i < rtlmq_qualified_topology_count(tp); i++) {
		const struct rtlmq_topology *topology =
			rtlmq_qualified_topology_at(tp, i);

		if (topology == qualified->default_topology)
			return topology;
	}

	return NULL;
}

u16 rtlmq_validated_max_tx_queues(const struct rtlmq_private *tp)
{
	u16 max_queues = RTLMQ_BOOT_TX_QUEUES;
	u16 i;

	for (i = 0; i < rtlmq_qualified_topology_count(tp); i++) {
		const struct rtlmq_topology *topology =
			rtlmq_qualified_topology_at(tp, i);

		if (topology)
			max_queues = max(max_queues, topology->tx_queues);
	}

	return min_t(u16, rtlmq_hw_max_tx_queues(tp), max_queues);
}

u16 rtlmq_validated_max_rx_queues(const struct rtlmq_private *tp)
{
	u16 max_queues = RTLMQ_BOOT_RX_QUEUES;
	u16 i;

	for (i = 0; i < rtlmq_qualified_topology_count(tp); i++) {
		const struct rtlmq_topology *topology =
			rtlmq_qualified_topology_at(tp, i);

		if (topology)
			max_queues = max(max_queues, topology->rx_queues);
	}

	return min_t(u16, rtlmq_hw_max_rx_queues(tp), max_queues);
}

bool rtlmq_validated_queue_shape_allowed(const struct rtlmq_private *tp,
					 u16 num_rx_queues, u16 num_tx_queues)
{
	return num_rx_queues <= rtlmq_hw_max_rx_queues(tp) &&
	       num_tx_queues <= rtlmq_hw_max_tx_queues(tp) &&
	       rtlmq_find_qualified_topology(tp, num_rx_queues, num_tx_queues);
}

u16 rtlmq_advertised_max_rx_queues(const struct rtlmq_private *tp)
{
	return rtlmq_validated_max_rx_queues(tp);
}

u16 rtlmq_advertised_max_tx_queues(const struct rtlmq_private *tp)
{
	return rtlmq_validated_max_tx_queues(tp);
}

static bool
rtlmq_runtime_topology_has_irq_mode(const struct rtlmq_private *tp,
				    const struct rtlmq_topology *topology)
{
	if (topology->irq_modes & BIT(RTLMQ_IRQ_MODE_MSIX) &&
	    rtlmq_chip_irq_mode_admitted(tp, RTLMQ_IRQ_MODE_MSIX))
		return true;
	if (topology->irq_modes & BIT(RTLMQ_IRQ_MODE_MSI) &&
	    rtlmq_chip_irq_mode_admitted(tp, RTLMQ_IRQ_MODE_MSI))
		return true;

	return topology->irq_modes & BIT(RTLMQ_IRQ_MODE_INTX) &&
	       rtlmq_chip_irq_mode_admitted(tp, RTLMQ_IRQ_MODE_INTX);
}

static bool rtlmq_chip_can_route_topology(const struct rtlmq_private *tp,
					  const struct rtlmq_topology *topology)
{
	u16 i;

	if (!rtlmq_hw_contract_present(tp) ||
	    !rtlmq_qualified_policy_present(tp) || !topology)
		return false;
	if (topology->rx_queues > rtlmq_hw_max_rx_queues(tp) ||
	    topology->tx_queues > rtlmq_hw_max_tx_queues(tp) ||
	    topology->vectors > rtlmq_hw_max_runtime_vectors(tp))
		return false;
	if (!rtlmq_runtime_topology_has_irq_mode(tp, topology) ||
	    !rtlmq_chip_routing_mode_admitted(tp, topology->routing))
		return false;
	if (topology->rx_queues > RTLMQ_BOOT_RX_QUEUES &&
	    !rtlmq_chip_supports_rss(tp))
		return false;
	if (topology->tx_queues > RTLMQ_BOOT_TX_QUEUES &&
	    !rtlmq_hw_has_q1_tx_desc_base_regs(tp))
		return false;

	/*
	 * Coupled multiqueue routing requires a hardware message ID for every
	 * TX queue. Reject topologies missing a TX interrupt source.
	 */
	if (topology->vectors > RTLMQ_BOOT_VECTORS &&
	    rtlmq_hw_has_msix_message_routing(tp)) {
		for (i = 0; i < topology->tx_queues; i++) {
			if (rtlmq_hw_msix_tx_message_id(tp, i) ==
			    RTLMQ_IRQ_SOURCE_NONE)
				return false;
		}
	}

	return true;
}

static bool
rtlmq_runtime_topology_supported(const struct rtlmq_private *tp,
				 const struct rtlmq_topology *topology)
{
	return topology &&
	       rtlmq_find_qualified_topology(tp, topology->rx_queues,
					     topology->tx_queues) == topology &&
	       rtlmq_chip_can_route_topology(tp, topology);
}

static void rtlmq_apply_runtime_topology(struct rtlmq_runtime_config *cfg,
					 const struct rtlmq_topology *topology)
{
	cfg->rss_enabled = topology->rx_queues > RTLMQ_BOOT_RX_QUEUES;
	cfg->num_vectors = topology->vectors;
	cfg->routing_mode = topology->routing;
}

static bool
rtlmq_runtime_config_matches_topology(const struct rtlmq_runtime_config *cfg,
				      const struct rtlmq_topology *topology)
{
	if (!cfg || !topology ||
	    cfg->num_rx_queues != topology->rx_queues ||
	    cfg->num_tx_queues != topology->tx_queues ||
	    cfg->num_vectors != topology->vectors ||
	    cfg->routing_mode != topology->routing ||
	    cfg->rss_enabled != (topology->rx_queues > RTLMQ_BOOT_RX_QUEUES))
		return false;

	return cfg->irq_mode == RTLMQ_IRQ_MODE_NONE ||
	       (cfg->irq_mode > RTLMQ_IRQ_MODE_NONE &&
		cfg->irq_mode < BITS_PER_TYPE(topology->irq_modes) &&
		(topology->irq_modes & BIT(cfg->irq_mode)));
}

bool rtlmq_runtime_config_matches_qualified_topology(const struct rtlmq_private *tp,
						     const struct rtlmq_runtime_config *cfg)
{
	const struct rtlmq_topology *topology;

	if (!cfg)
		return false;

	topology = rtlmq_find_qualified_topology(tp, cfg->num_rx_queues,
						 cfg->num_tx_queues);
	return rtlmq_runtime_config_matches_topology(cfg, topology);
}

static int
rtlmq_resolve_requested_topology(const struct rtlmq_private *tp,
				 u16 num_rx_queues, u16 num_tx_queues,
				 const struct rtlmq_topology **resolved)
{
	const struct rtlmq_topology *topology;

	if (!resolved)
		return -EINVAL;
	*resolved = NULL;
	if (!rtlmq_hw_contract_present(tp) ||
	    !rtlmq_qualified_policy_present(tp))
		return -EINVAL;
	if (!num_rx_queues || !num_tx_queues)
		return -EINVAL;
	if (!is_power_of_2(num_rx_queues) || !is_power_of_2(num_tx_queues))
		return -EINVAL;

	topology =
		rtlmq_find_qualified_topology(tp, num_rx_queues, num_tx_queues);
	if (!rtlmq_runtime_topology_supported(tp, topology))
		return -EOPNOTSUPP;

	*resolved = topology;
	return 0;
}

static int rtlmq_validate_requested_runtime_queue_counts(const struct rtlmq_private *tp,
							 u16 num_rx_queues, u16 num_tx_queues)
{
	const struct rtlmq_topology *topology;

	return rtlmq_resolve_requested_topology(tp, num_rx_queues, num_tx_queues, &topology);
}

int rtlmq_normalize_set_channels_request(const struct rtlmq_private *tp,
					 const struct ethtool_channels *channels,
					 struct rtlmq_channel_request *request)
{
	if (!channels)
		return -EINVAL;
	if (!request)
		return -EINVAL;
	if (channels->rx_count > U16_MAX || channels->tx_count > U16_MAX)
		return -EINVAL;

	if (!channels->rx_count || !channels->tx_count)
		return -EINVAL;

	if (channels->combined_count || channels->other_count)
		return -EOPNOTSUPP;

	/*
	 * Check ethtool-specific fields here, then use the same queue-count
	 * validation as internal runtime requests.
	 */
	return rtlmq_normalize_runtime_queue_request(tp, channels->rx_count, channels->tx_count,
						     request);
}

int rtlmq_normalize_runtime_queue_request(const struct rtlmq_private *tp,
					  u16 rx_count, u16 tx_count,
					  struct rtlmq_channel_request *request)
{
	if (!request)
		return -EINVAL;

	request->rx_count = rx_count;
	request->tx_count = tx_count;

	/*
	 * Validate queue counts identically for down-state changes and live
	 * rebuilds. Only their application paths differ.
	 */
	return rtlmq_validate_requested_runtime_queue_counts(tp, request->rx_count,
							     request->tx_count);
}

int rtlmq_validate_runtime_config(const struct rtlmq_private *tp,
				  const struct rtlmq_runtime_config *cfg)
{
	const struct rtlmq_topology *topology;
	u16 max_rx_queues;
	u16 max_tx_queues;
	u16 max_vectors;

	if (!cfg || !rtlmq_hw_contract_present(tp) ||
	    !rtlmq_qualified_policy_present(tp))
		return -EINVAL;

	max_rx_queues = rtlmq_hw_max_rx_queues(tp);
	max_tx_queues = rtlmq_hw_max_tx_queues(tp);
	max_vectors = rtlmq_hw_max_runtime_vectors(tp);

	if (!cfg->num_rx_queues || cfg->num_rx_queues > max_rx_queues)
		return -EINVAL;

	if (!cfg->num_tx_queues || cfg->num_tx_queues > max_tx_queues)
		return -EINVAL;
	if (cfg->num_tx_queues > RTLMQ_TX_POLL_KICK_MAX_QUEUES)
		return -EINVAL;

	if (!is_power_of_2(cfg->num_rx_queues) ||
	    !is_power_of_2(cfg->num_tx_queues))
		return -EINVAL;

	if (rtlmq_validate_ring_counts(cfg->rx_ring_count,
				       cfg->tx_ring_count))
		return -EINVAL;

	if (!cfg->num_vectors || cfg->num_vectors > max_vectors)
		return -EINVAL;

	/*
	 * Re-derive the complete topology from queue counts. This rejects a
	 * staged config whose vector, routing, RSS, or IRQ fields drift from
	 * the one qualified record before it can become active or visible.
	 */
	topology = rtlmq_find_qualified_topology(tp, cfg->num_rx_queues,
						 cfg->num_tx_queues);
	if (!rtlmq_runtime_topology_supported(tp, topology) ||
	    !rtlmq_runtime_config_matches_qualified_topology(tp, cfg))
		return -EOPNOTSUPP;
	if (cfg->irq_mode != RTLMQ_IRQ_MODE_NONE &&
	    !rtlmq_chip_irq_mode_admitted(tp, cfg->irq_mode))
		return -EOPNOTSUPP;
	if (cfg->xsk_tx_metadata_ops != rtlmq_get_xsk_tx_metadata_ops(tp))
		return -EOPNOTSUPP;

	return 0;
}

int rtlmq_plan_runtime_topology(const struct rtlmq_private *tp,
				u16 num_rx_queues, u16 num_tx_queues,
				struct rtlmq_runtime_topology_plan *plan)
{
	const struct rtlmq_runtime_config *requested_cfg;
	const struct rtlmq_topology *topology;
	struct rtlmq_runtime_config cfg;
	int ret;

	if (!plan)
		return -EINVAL;

	memset(plan, 0, sizeof(*plan));
	ret = rtlmq_resolve_requested_topology(tp, num_rx_queues, num_tx_queues, &topology);
	if (ret)
		return ret;

	memset(&cfg, 0, sizeof(cfg));
	requested_cfg = rtlmq_requested_runtime_config(tp);
	cfg.requested_num_rx_queues = rtlmq_requested_num_rx_queues(tp);
	cfg.requested_num_tx_queues = rtlmq_requested_num_tx_queues(tp);
	cfg.num_rx_queues = num_rx_queues;
	cfg.num_tx_queues = num_tx_queues;
	cfg.rx_ring_count = requested_cfg ?
		requested_cfg->rx_ring_count : RTLMQ_NUM_RX_DESC;
	cfg.tx_ring_count = requested_cfg ?
		requested_cfg->tx_ring_count : RTLMQ_NUM_TX_DESC;
	cfg.irq_mode = RTLMQ_IRQ_MODE_NONE;
	rtlmq_apply_runtime_topology(&cfg, topology);
	cfg.xsk_tx_metadata_ops = rtlmq_get_xsk_tx_metadata_ops(tp);

	ret = rtlmq_validate_runtime_config(tp, &cfg);
	if (ret)
		return ret;

	plan->profile = tp->profile;
	plan->topology = topology;
	plan->config = cfg;
	return 0;
}

int rtlmq_construct_runtime_profile_state(const struct rtlmq_runtime_topology_plan *plan,
					  struct rtlmq_runtime_profile_state *profile)
{
	if (!plan || !profile || !plan->profile || !plan->topology ||
	    !rtlmq_runtime_config_matches_topology(&plan->config,
						    plan->topology))
		return -EINVAL;

	memset(profile, 0, sizeof(*profile));
	profile->config = plan->config;
	profile->valid = true;
	return 0;
}

void rtlmq_clear_candidate_profile(struct rtlmq_private *tp)
{
	rtlmq_clear_candidate_napi(tp);
	rtlmq_discard_candidate_tx_resources(tp);
	rtlmq_discard_candidate_rx_rings(tp);
	rtlmq_discard_candidate_xdp(tp);
	memset(tp->transaction.xsk_pools, 0,
	       sizeof(tp->transaction.xsk_pools));
	tp->transaction.profile.valid = false;
	tp->transaction.profile.layout.valid = false;
	tp->transaction.resource_generation = 0;
	tp->transaction.rx_dma_ring_mask = 0;
	tp->transaction.tx_dma_ring_mask = 0;
	tp->transaction.irq_vectors_allocated = false;
	rtlmq_clear_candidate_rss(tp);
}

int rtlmq_select_candidate_runtime_profile(struct rtlmq_private *tp,
					   u16 num_rx_queues, u16 num_tx_queues)
{
	struct rtlmq_runtime_topology_plan plan;
	int ret;

	/*
	 * Validate the complete topology before constructing candidate state.
	 * A planning or construction error discards any existing candidate.
	 */
	ret = rtlmq_plan_runtime_topology(tp, num_rx_queues, num_tx_queues,
					  &plan);
	if (ret)
		goto err_clear_candidate;

	ret = rtlmq_construct_runtime_profile_state(&plan, &tp->transaction.profile);
	if (ret)
		goto err_clear_candidate;

	return 0;

err_clear_candidate:
	rtlmq_clear_candidate_profile(tp);
	return ret;
}
