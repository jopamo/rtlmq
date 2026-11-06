/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef __RTLMQ_RUNTIME_ACCESSORS_H__
#define __RTLMQ_RUNTIME_ACCESSORS_H__

/*
 * Inline readers inspect software state, not MMIO/OCP/MDIO registers.
 * Runtime and RSS owners perform mutation, commit, publication and rollback.
 */
#include "rtlmq_chip.h"
#include "rtlmq.h"

static inline u16 rtlmq_owned_rx_ring_slots(const struct rtlmq_private *tp)
{
	return tp ? tp->allocated_rx_ring_slots : 0;
}

static inline u16 rtlmq_owned_tx_ring_slots(const struct rtlmq_private *tp)
{
	return tp ? tp->allocated_tx_ring_slots : 0;
}

static inline u16 rtlmq_owned_vector_slots(const struct rtlmq_private *tp)
{
	return tp ? tp->allocated_vectors : 0;
}

static inline u16 rtlmq_requested_num_rx_queues(const struct rtlmq_private *tp)
{
	return tp && tp->requested_profile.valid ?
		tp->requested_profile.config.requested_num_rx_queues : 0;
}

static inline u16 rtlmq_requested_num_tx_queues(const struct rtlmq_private *tp)
{
	return tp && tp->requested_profile.valid ?
		tp->requested_profile.config.requested_num_tx_queues : 0;
}

static inline u16 rtlmq_requested_rx_ring_count(const struct rtlmq_private *tp)
{
	return tp && tp->requested_profile.valid ?
		tp->requested_profile.config.rx_ring_count : 0;
}

static inline u16 rtlmq_requested_tx_ring_count(const struct rtlmq_private *tp)
{
	return tp && tp->requested_profile.valid ?
		tp->requested_profile.config.tx_ring_count : 0;
}

static inline const struct rtlmq_rss *
rtlmq_requested_rss_state(const struct rtlmq_private *tp)
{
	if (!tp)
		return NULL;

	return &tp->rss;
}

static inline const struct rtlmq_rss *
rtlmq_active_rss_state(const struct rtlmq_private *tp)
{
	/*
	 * Active RSS commits with active_profile. Use its validity bit rather
	 * than a separate RSS flag that could become stale.
	 */
	if (!tp || !tp->active_profile.valid)
		return NULL;

	return &tp->active_rss;
}

const struct rtlmq_rss *
rtlmq_active_or_requested_rss_state(const struct rtlmq_private *tp);
/* Datapath and runtime statistics exclude unpublished configurations. */
const struct rtlmq_rss *
rtlmq_published_rss_state(const struct rtlmq_private *tp);

static inline const struct rtlmq_runtime_config *
rtlmq_active_runtime_config(const struct rtlmq_private *tp)
{
	if (!tp || !tp->active_profile.valid)
		return NULL;

	return &tp->active_profile.config;
}

static inline const struct rtlmq_runtime_config *
rtlmq_requested_runtime_config(const struct rtlmq_private *tp)
{
	/*
	 * Requested settings survive close, IRQ fallback, and candidate setup.
	 * Use candidate/active/published accessors for staged or live settings.
	 */
	if (!tp || !tp->requested_profile.valid)
		return NULL;

	return &tp->requested_profile.config;
}

static inline const struct rtlmq_runtime_layout_state *
rtlmq_active_runtime_layout(const struct rtlmq_private *tp)
{
	/*
	 * active_profile.valid covers the committed configuration and layout.
	 * layout.valid checks requested/candidate construction only.
	 */
	if (!tp || !tp->active_profile.valid)
		return NULL;

	return &tp->active_profile.layout;
}

static inline const struct rtlmq_runtime_layout_state *
rtlmq_requested_runtime_layout(const struct rtlmq_private *tp)
{
	if (!tp || !tp->requested_profile.layout.valid)
		return NULL;

	return &tp->requested_profile.layout;
}

const struct rtlmq_runtime_layout_state *
rtlmq_active_or_requested_runtime_layout(const struct rtlmq_private *tp);
const struct rtlmq_runtime_layout_state *
rtlmq_published_runtime_layout(const struct rtlmq_private *tp);

const struct rtlmq_runtime_config *
rtlmq_active_or_requested_runtime_config(const struct rtlmq_private *tp);

static inline const struct rtlmq_runtime_config *
rtlmq_published_runtime_config(const struct rtlmq_private *tp)
{
	if (unlikely(!tp))
		return NULL;

	/*
	 * Pairs with rtlmq_publish_runtime_publication_locked(). Acquiring
	 * publication does not pin the returned embedded profile.
	 */
	if (likely(smp_load_acquire(&tp->runtime_published) &&
		   tp->active_profile.valid &&
		   READ_ONCE(tp->state) == RTLMQ_STATE_OPEN))
		return &tp->active_profile.config;

	return NULL;
}

/*
 * Userspace current-setting getters, including ethtool, must not fall back
 * to requested settings when no active runtime is published.
 */

static inline u16 rtlmq_published_num_rx_queues(const struct rtlmq_private *tp)
{
	const struct rtlmq_runtime_config *cfg = rtlmq_published_runtime_config(tp);

	return cfg ? cfg->num_rx_queues : 0;
}

static inline u16 rtlmq_published_num_tx_queues(const struct rtlmq_private *tp)
{
	const struct rtlmq_runtime_config *cfg = rtlmq_published_runtime_config(tp);

	return cfg ? cfg->num_tx_queues : 0;
}

static inline u16 rtlmq_published_rx_ring_count(const struct rtlmq_private *tp)
{
	const struct rtlmq_runtime_config *cfg = rtlmq_published_runtime_config(tp);

	return cfg ? cfg->rx_ring_count : 0;
}

static inline u16 rtlmq_published_tx_ring_count(const struct rtlmq_private *tp)
{
	const struct rtlmq_runtime_config *cfg = rtlmq_published_runtime_config(tp);

	return cfg ? cfg->tx_ring_count : 0;
}

static inline u16 rtlmq_published_num_vectors(const struct rtlmq_private *tp)
{
	const struct rtlmq_runtime_config *cfg = rtlmq_published_runtime_config(tp);

	return cfg ? cfg->num_vectors : 0;
}

static inline enum rtlmq_irq_mode
rtlmq_published_irq_mode(const struct rtlmq_private *tp)
{
	const struct rtlmq_runtime_config *cfg = rtlmq_published_runtime_config(tp);

	return cfg ? cfg->irq_mode : RTLMQ_IRQ_MODE_NONE;
}

static inline enum rtlmq_routing_mode
rtlmq_published_routing_mode(const struct rtlmq_private *tp)
{
	const struct rtlmq_runtime_config *cfg = rtlmq_published_runtime_config(tp);

	return cfg ? cfg->routing_mode : RTLMQ_ROUTING_SINGLE_VECTOR;
}

/* Candidate readers belong to setup/programming paths, not current getters. */
static inline const struct rtlmq_rss *
rtlmq_candidate_rss_state(const struct rtlmq_private *tp)
{
	if (!tp || !tp->transaction.rss_valid)
		return NULL;

	return &tp->transaction.rss;
}

static inline const struct rtlmq_runtime_config *
rtlmq_candidate_runtime_config(const struct rtlmq_private *tp)
{
	if (!tp || !tp->transaction.profile.valid)
		return NULL;

	return &tp->transaction.profile.config;
}

static inline const struct rtlmq_runtime_layout_state *
rtlmq_candidate_runtime_layout(const struct rtlmq_private *tp)
{
	if (!tp || !tp->transaction.profile.layout.valid)
		return NULL;

	return &tp->transaction.profile.layout;
}

static inline struct bpf_prog *
rtlmq_candidate_xdp_prog(const struct rtlmq_private *tp)
{
	if (!tp || !tp->transaction.xdp_staged)
		return NULL;

	return READ_ONCE(tp->transaction.xdp_prog);
}

/*
 * Candidate-or-published selectors are for private rebuild/program paths
 * only. Userspace current-setting getters must not read candidate state.
 */
const struct rtlmq_runtime_config *
rtlmq_candidate_or_published_runtime_config(const struct rtlmq_private *tp);

static inline u16
rtlmq_candidate_or_published_num_rx_queues(const struct rtlmq_private *tp)
{
	const struct rtlmq_runtime_config *cfg = rtlmq_candidate_or_published_runtime_config(tp);

	return cfg ? cfg->num_rx_queues : 0;
}

static inline u16
rtlmq_candidate_or_published_num_tx_queues(const struct rtlmq_private *tp)
{
	const struct rtlmq_runtime_config *cfg = rtlmq_candidate_or_published_runtime_config(tp);

	return cfg ? cfg->num_tx_queues : 0;
}

static inline u16
rtlmq_candidate_or_published_num_vectors(const struct rtlmq_private *tp)
{
	const struct rtlmq_runtime_config *cfg = rtlmq_candidate_or_published_runtime_config(tp);

	return cfg ? cfg->num_vectors : 0;
}

static inline enum rtlmq_irq_mode
rtlmq_candidate_or_published_irq_mode(const struct rtlmq_private *tp)
{
	const struct rtlmq_runtime_config *cfg = rtlmq_candidate_or_published_runtime_config(tp);

	return cfg ? cfg->irq_mode : RTLMQ_IRQ_MODE_NONE;
}

static inline bool
rtlmq_candidate_or_published_rss_enabled(const struct rtlmq_private *tp)
{
	const struct rtlmq_runtime_config *cfg = rtlmq_candidate_or_published_runtime_config(tp);

	return cfg ? cfg->rss_enabled : false;
}

/* Raw hardware limits do not grant driver-supported features. */
static inline const struct rtlmq_hw_contract *
rtlmq_selected_hw_contract(const struct rtlmq_private *tp)
{
	return tp ? rtlmq_chip_profile_hw(tp->profile) : NULL;
}

static inline bool rtlmq_hw_contract_present(const struct rtlmq_private *tp)
{
	return rtlmq_selected_hw_contract(tp);
}

static inline u16 rtlmq_hw_max_rx_queues(const struct rtlmq_private *tp)
{
	const struct rtlmq_hw_contract *hw = rtlmq_selected_hw_contract(tp);
	u16 max_rx_queues = hw ? hw->max_rx_queues : 0;

	return max_rx_queues ?
	       min_t(u16, max_rx_queues, RTLMQ_IMPL_MAX_RX_QUEUES) :
	       RTLMQ_BOOT_RX_QUEUES;
}

static inline u16 rtlmq_hw_max_tx_queues(const struct rtlmq_private *tp)
{
	const struct rtlmq_hw_contract *hw = rtlmq_selected_hw_contract(tp);
	u16 max_tx_queues = hw ? hw->max_tx_queues : 0;

	return max_tx_queues ?
	       min_t(u16, max_tx_queues, RTLMQ_IMPL_MAX_TX_QUEUES) :
	       RTLMQ_BOOT_TX_QUEUES;
}

static inline u8 rtlmq_hw_tx_desc_stride(const struct rtlmq_private *tp)
{
	const struct rtlmq_hw_contract *hw = rtlmq_selected_hw_contract(tp);

	return hw && hw->tx_desc_stride ? hw->tx_desc_stride : 32;
}

static inline bool
rtlmq_hw_has_q1_tx_desc_base_regs(const struct rtlmq_private *tp)
{
	const struct rtlmq_hw_contract *hw = rtlmq_selected_hw_contract(tp);

	return hw && hw->has_q1_tx_desc_base_regs;
}

static inline bool
rtlmq_hw_requires_tx_engine_runtime_latch(const struct rtlmq_private *tp)
{
	const struct rtlmq_hw_contract *hw = rtlmq_selected_hw_contract(tp);

	return hw && hw->requires_tx_engine_runtime_latch;
}

static inline u16 rtlmq_hw_max_runtime_vectors(const struct rtlmq_private *tp)
{
	const struct rtlmq_hw_contract *hw = rtlmq_selected_hw_contract(tp);
	u16 max_vectors = hw ? hw->max_runtime_vectors : 0;

	return max_vectors ?
	       min_t(u16, max_vectors, RTLMQ_IMPL_MAX_NAPI) :
	       RTLMQ_BOOT_VECTORS;
}

static inline u16 rtlmq_hw_max_irq_vectors(const struct rtlmq_private *tp)
{
	const struct rtlmq_hw_contract *hw = rtlmq_selected_hw_contract(tp);
	u16 max_vectors = hw ? hw->max_irq_vectors : 0;

	return max_vectors ? max_vectors : RTLMQ_BOOT_VECTORS;
}

static inline bool rtlmq_hw_has_msi(const struct rtlmq_private *tp)
{
	const struct rtlmq_hw_contract *hw = rtlmq_selected_hw_contract(tp);

	return hw && hw->has_msi;
}

static inline bool rtlmq_hw_has_msix(const struct rtlmq_private *tp)
{
	const struct rtlmq_hw_contract *hw = rtlmq_selected_hw_contract(tp);

	return hw && hw->has_msix;
}

static inline bool rtlmq_hw_has_rss(const struct rtlmq_private *tp)
{
	const struct rtlmq_hw_contract *hw = rtlmq_selected_hw_contract(tp);

	return hw && hw->has_rss;
}

static inline bool
rtlmq_hw_has_msix_message_routing(const struct rtlmq_private *tp)
{
	const struct rtlmq_hw_contract *hw = rtlmq_selected_hw_contract(tp);

	return hw && hw->has_msix && hw->has_msix_message_routing;
}

static inline u8 rtlmq_hw_msix_rx_message_id(const struct rtlmq_private *tp,
					     u16 queue_index)
{
	const struct rtlmq_hw_contract *hw = rtlmq_selected_hw_contract(tp);

	if (queue_index >= RTLMQ_IMPL_MAX_RX_QUEUES)
		return 0xff;
	return hw ? hw->msix_rx_message_ids[queue_index] : 0xff;
}

static inline u8 rtlmq_hw_msix_tx_message_id(const struct rtlmq_private *tp,
					     u16 queue_index)
{
	const struct rtlmq_hw_contract *hw = rtlmq_selected_hw_contract(tp);

	if (queue_index >= RTLMQ_IMPL_MAX_TX_QUEUES)
		return 0xff;
	return hw ? hw->msix_tx_message_ids[queue_index] : 0xff;
}

static inline u8 rtlmq_hw_msix_link_message_id(const struct rtlmq_private *tp)
{
	const struct rtlmq_hw_contract *hw = rtlmq_selected_hw_contract(tp);

	return hw ? hw->msix_link_message_id : 0xff;
}

static inline u8
rtlmq_hw_msix_int_cfg0_clear_mask(const struct rtlmq_private *tp)
{
	const struct rtlmq_hw_contract *hw = rtlmq_selected_hw_contract(tp);

	return hw ? hw->msix_int_cfg0_clear_mask : 0;
}

static inline enum rtlmq_rx_desc_format
rtlmq_hw_rss_rx_desc_fmt(const struct rtlmq_private *tp)
{
	const struct rtlmq_hw_contract *hw = rtlmq_selected_hw_contract(tp);

	return hw ? hw->rss_rx_desc_fmt : RX_DESC_FMT_V1;
}

static inline enum rtlmq_rss_version
rtlmq_hw_rss_version(const struct rtlmq_private *tp)
{
	const struct rtlmq_hw_contract *hw = rtlmq_selected_hw_contract(tp);

	return hw ? hw->rss_version : RTLMQ_RSS_VERSION_NONE;
}

static inline enum rtlmq_irq_moderation_version
rtlmq_hw_irq_moderation_version(const struct rtlmq_private *tp)
{
	const struct rtlmq_hw_contract *hw = rtlmq_selected_hw_contract(tp);

	return hw ? hw->irq_moderation_version : RTLMQ_IRQ_MODERATION_NONE;
}

static inline enum rtlmq_tx_desc_format
rtlmq_hw_tx_desc_fmt(const struct rtlmq_private *tp)
{
	const struct rtlmq_hw_contract *hw = rtlmq_selected_hw_contract(tp);

	return hw ? hw->tx_desc_fmt : RTLMQ_TX_DESC_FORMAT_LEGACY;
}

/* Driver support comes from the selected qualified policy. */
static inline const struct rtlmq_qualified_policy *
rtlmq_qualified_policy_for(const struct rtlmq_private *tp)
{
	return tp ? rtlmq_chip_profile_qualified(tp->profile) : NULL;
}

static inline bool
rtlmq_qualified_policy_present(const struct rtlmq_private *tp)
{
	return rtlmq_qualified_policy_for(tp);
}

static inline u32 rtlmq_qualified_mtu_limit(const struct rtlmq_private *tp)
{
	const struct rtlmq_qualified_policy *qualified =
		rtlmq_qualified_policy_for(tp);

	return qualified ? qualified->max_mtu : 0;
}

static inline u16
rtlmq_qualified_max_xsk_tx_queues(const struct rtlmq_private *tp)
{
	const struct rtlmq_qualified_policy *qualified =
		rtlmq_qualified_policy_for(tp);

	return qualified && qualified->supports_af_xdp ?
		       qualified->max_xsk_tx_queues : 0;
}

static inline u32 rtlmq_qualified_tso_size_limit(const struct rtlmq_private *tp)
{
	const struct rtlmq_qualified_policy *qualified =
		rtlmq_qualified_policy_for(tp);

	return qualified ? qualified->max_tso_size : 0;
}

static inline const struct rtlmq_topology *
rtlmq_qualified_topology_at(const struct rtlmq_private *tp, u16 index)
{
	const struct rtlmq_qualified_policy *qualified =
		rtlmq_qualified_policy_for(tp);

	if (!qualified || !qualified->topologies ||
	    index >= qualified->num_topologies)
		return NULL;

	return &qualified->topologies[index];
}

static inline netdev_features_t
rtlmq_qualified_base_netdev_features(const struct rtlmq_private *tp)
{
	const struct rtlmq_qualified_policy *qualified =
		rtlmq_qualified_policy_for(tp);

	return qualified ? qualified->base_netdev_features : 0;
}

static inline bool
rtlmq_qualified_irq_mode_allowed(const struct rtlmq_private *tp,
				 enum rtlmq_irq_mode mode)
{
	u16 i;

	if (mode <= RTLMQ_IRQ_MODE_NONE || mode >= BITS_PER_TYPE(u8))
		return false;

	for (i = 0; i < rtlmq_qualified_topology_count(tp); i++) {
		const struct rtlmq_topology *topology =
			rtlmq_qualified_topology_at(tp, i);

		if (topology && topology->irq_modes & BIT(mode))
			return true;
	}

	return false;
}

static inline bool
rtlmq_qualified_routing_mode_allowed(const struct rtlmq_private *tp,
				     enum rtlmq_routing_mode mode)
{
	u16 i;

	for (i = 0; i < rtlmq_qualified_topology_count(tp); i++) {
		const struct rtlmq_topology *topology =
			rtlmq_qualified_topology_at(tp, i);

		if (topology && topology->routing == mode)
			return true;
	}

	return false;
}

static inline bool rtlmq_qualified_supports_rss(const struct rtlmq_private *tp)
{
	const struct rtlmq_qualified_policy *qualified =
		rtlmq_qualified_policy_for(tp);

	return qualified && qualified->supports_rss;
}

#define RTLMQ_QUALIFIED_BOOL_ACCESSOR(_name, _field) \
static inline bool _name(const struct rtlmq_private *tp) \
{ \
	const struct rtlmq_qualified_policy *qualified = \
		rtlmq_qualified_policy_for(tp); \
	\
	return qualified && qualified->_field; \
}

RTLMQ_QUALIFIED_BOOL_ACCESSOR(rtlmq_qualified_use_core_napi_defaults,
			      use_core_napi_defaults)
RTLMQ_QUALIFIED_BOOL_ACCESSOR(rtlmq_qualified_supports_vlan_ctag_tx,
			      supports_vlan_ctag_tx)
RTLMQ_QUALIFIED_BOOL_ACCESSOR(rtlmq_qualified_supports_vlan_ctag_rx,
			      supports_vlan_ctag_rx)
RTLMQ_QUALIFIED_BOOL_ACCESSOR(rtlmq_qualified_supports_rx_fcs,
			      supports_rx_fcs)
RTLMQ_QUALIFIED_BOOL_ACCESSOR(rtlmq_qualified_supports_rx_all,
			      supports_rx_all)
RTLMQ_QUALIFIED_BOOL_ACCESSOR(rtlmq_qualified_supports_live_mac_change,
			      supports_live_mac_change)
RTLMQ_QUALIFIED_BOOL_ACCESSOR(rtlmq_qualified_supports_inplace_ring_resize,
			      supports_inplace_ring_resize)
RTLMQ_QUALIFIED_BOOL_ACCESSOR(rtlmq_qualified_supports_eee, supports_eee)
RTLMQ_QUALIFIED_BOOL_ACCESSOR(rtlmq_qualified_supports_hw_tally,
			      supports_hw_tally)
RTLMQ_QUALIFIED_BOOL_ACCESSOR(rtlmq_qualified_supports_leds, supports_leds)
RTLMQ_QUALIFIED_BOOL_ACCESSOR(rtlmq_qualified_supports_cbs, supports_cbs)
RTLMQ_QUALIFIED_BOOL_ACCESSOR(rtlmq_qualified_wol_link_v3, wol_link_v3)

#undef RTLMQ_QUALIFIED_BOOL_ACCESSOR

static inline u32 rtlmq_qualified_wol_opts(const struct rtlmq_private *tp)
{
	const struct rtlmq_qualified_policy *qualified =
		rtlmq_qualified_policy_for(tp);

	return qualified ? qualified->wol_opts : 0;
}

static inline bool
rtlmq_qualified_supports_highdma(const struct rtlmq_private *tp)
{
	const struct rtlmq_qualified_policy *qualified =
		rtlmq_qualified_policy_for(tp);

	return qualified && qualified->supports_highdma;
}

static inline bool
rtlmq_qualified_supports_af_xdp(const struct rtlmq_private *tp)
{
	const struct rtlmq_qualified_policy *qualified =
		rtlmq_qualified_policy_for(tp);

	return qualified && qualified->supports_af_xdp;
}

static inline netdev_features_t
rtlmq_qualified_netdev_hw_features(const struct rtlmq_private *tp)
{
	netdev_features_t features =
		rtlmq_qualified_base_netdev_features(tp);

	if (rtlmq_chip_supports_rss(tp))
		features |= NETIF_F_RXHASH;
	if (rtlmq_qualified_supports_vlan_ctag_tx(tp))
		features |= NETIF_F_HW_VLAN_CTAG_TX;
	if (rtlmq_qualified_supports_vlan_ctag_rx(tp))
		features |= NETIF_F_HW_VLAN_CTAG_RX;
	if (rtlmq_qualified_supports_rx_fcs(tp))
		features |= NETIF_F_RXFCS;
	if (rtlmq_qualified_supports_rx_all(tp))
		features |= NETIF_F_RXALL;

	return features;
}

static inline netdev_features_t
rtlmq_qualified_netdev_default_features(const struct rtlmq_private *tp)
{
	/* RXFCS and RXALL require explicit opt-in; leave both off by default. */
	return rtlmq_qualified_netdev_hw_features(tp) &
	       ~(NETIF_F_RXFCS | NETIF_F_RXALL);
}

static inline enum rtlmq_ptp_backend
rtlmq_qualified_ptp_backend(const struct rtlmq_private *tp)
{
	const struct rtlmq_qualified_policy *qualified =
		rtlmq_qualified_policy_for(tp);

	return qualified ? qualified->ptp_backend : RTLMQ_PTP_BACKEND_NONE;
}

/* Mutable RSS views are for probe initialization and control_lock owners. */
struct rtlmq_rss *rtlmq_requested_rss_state_owned_mut(struct rtlmq_private *tp);
struct rtlmq_rss *rtlmq_candidate_rss_state_owned_mut(struct rtlmq_private *tp);

u32 rtlmq_get_rxfh_key_size(struct net_device *ndev);
u32 rtlmq_get_rxfh_indir_size(struct net_device *ndev);
int rtlmq_get_rxfh(struct net_device *ndev, struct ethtool_rxfh_param *rxfh);
int rtlmq_set_rxfh(struct net_device *ndev, struct ethtool_rxfh_param *rxfh,
		   struct netlink_ext_ack *extack);
void rtlmq_init_rss(struct rtlmq_private *tp);
void rtlmq_stage_candidate_rss_from_requested(struct rtlmq_private *tp);
void rtlmq_stage_candidate_rss_for_config(struct rtlmq_private *tp,
					  const struct rtlmq_runtime_config *cfg);
void rtlmq_clear_candidate_rss(struct rtlmq_private *tp);
/*
 * Apply an ethtool RSS update staged in tp->transaction.rss. While OPEN,
 * program hardware before committing requested and active settings. While
 * PROBED, commit only the request. A programming error clears the candidate
 * without changing the saved software settings.
 */
int rtlmq_apply_candidate_rss_transaction(struct rtlmq_private *tp);
void rtlmq_publish_rss_active(struct rtlmq_private *tp);
void rtlmq_unwind_active_rss(struct rtlmq_private *tp);
void rtlmq_rebalance_rss(struct rtlmq_private *tp);
void rtlmq_rebalance_requested_rss(struct rtlmq_private *tp);
void rtlmq_rebalance_rss_for_config(struct rtlmq_private *tp,
				    const struct rtlmq_runtime_config *cfg);

#endif /* __RTLMQ_RUNTIME_ACCESSORS_H__ */
