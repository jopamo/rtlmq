// SPDX-License-Identifier: GPL-2.0-only

#include <linux/bug.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/string.h>

#include "rtlmq_chip.h"
#include "rtlmq_regs.h"
#include "rtlmq_hw.h"
#include "rtlmq_runtime_accessors.h"
#include "rtlmq_irq.h"
#include "rtlmq_runtime.h"
#include "rtlmq.h"

const char *rtlmq_chip_profile_name(const struct rtlmq_chip_profile *profile)
{
	return profile ? profile->name : NULL;
}

const struct rtlmq_chip_match *
rtlmq_chip_profile_match(const struct rtlmq_chip_profile *profile)
{
	return profile ? profile->match : NULL;
}

const struct rtlmq_hw_contract *
rtlmq_chip_profile_hw(const struct rtlmq_chip_profile *profile)
{
	return profile ? profile->hw : NULL;
}

const struct rtlmq_qualified_policy *
rtlmq_chip_profile_qualified(const struct rtlmq_chip_profile *profile)
{
	return profile ? profile->qualified : NULL;
}

const struct rtlmq_chip_ops *
rtlmq_chip_profile_ops(const struct rtlmq_chip_profile *profile)
{
	return profile ? profile->ops : NULL;
}

const struct rtlmq_fw_profile *
rtlmq_chip_profile_firmware(const struct rtlmq_chip_profile *profile)
{
	return profile ? profile->firmware : NULL;
}

const struct rtlmq_regdump_schema *
rtlmq_chip_profile_regdump(const struct rtlmq_chip_profile *profile)
{
	return profile ? profile->regdump : NULL;
}

/*
 * Only profiles in this table are supported in normal builds. Leave
 * unsupported or unqualified silicon out of the table instead of adding
 * another flag that lookup code might ignore.
 */
static const struct rtlmq_chip_profile * const rtlmq_production_registry[] = {
	&rtlmq_8125b_profile,
	&rtlmq_8126a2_profile,
	&rtlmq_8126a3_profile,
	&rtlmq_5000a2_profile,
	&rtlmq_5000a3_profile,
};

#define RTLMQ_PROFILE_CONFIG_METHODS					\
	(BIT(RTLMQ_CFG_METHOD_2) | BIT(RTLMQ_CFG_METHOD_3) |		\
	 BIT(RTLMQ_CFG_METHOD_4) | BIT(RTLMQ_CFG_METHOD_5))
#define RTLMQ_PROFILE_IRQ_MODES						\
	(BIT(RTLMQ_IRQ_MODE_INTX) | BIT(RTLMQ_IRQ_MODE_MSI) |	\
	 BIT(RTLMQ_IRQ_MODE_MSIX))

static bool rtlmq_profile_message_id_valid(const struct rtlmq_hw_contract *hw,
					   u8 message_id)
{
	return message_id != RTLMQ_IRQ_SOURCE_NONE &&
	       message_id < hw->max_irq_vectors &&
	       message_id < BITS_PER_TYPE(u32);
}

static bool rtlmq_profile_rss_format_valid(enum rtlmq_rx_desc_format format)
{
	return format == RX_DESC_FMT_V3 || format == RX_DESC_FMT_V4;
}

bool rtlmq_msix_rx_tx_sources_overlap(const struct rtlmq_hw_contract *hw,
				      u16 num_rx_queues, u16 num_tx_queues)
{
	unsigned int rx;
	unsigned int tx;

	if (!hw || !num_rx_queues || !num_tx_queues ||
	    num_rx_queues > ARRAY_SIZE(hw->msix_rx_message_ids) ||
	    num_tx_queues > ARRAY_SIZE(hw->msix_tx_message_ids))
		return false;

	for (rx = 0; rx < num_rx_queues; rx++) {
		for (tx = 0; tx < num_tx_queues; tx++) {
			if (hw->msix_rx_message_ids[rx] ==
			    hw->msix_tx_message_ids[tx])
				return true;
		}
	}

	return false;
}

u16 rtlmq_msix_service_owner_count(const struct rtlmq_hw_contract *hw,
				   u16 num_rx_queues, u16 num_tx_queues)
{
	u32 non_tx_rx_sources = 0;
	bool overlaps = false;
	unsigned int rx;
	unsigned int tx;

	if (!hw || !num_rx_queues || !num_tx_queues ||
	    num_rx_queues > ARRAY_SIZE(hw->msix_rx_message_ids) ||
	    num_tx_queues > ARRAY_SIZE(hw->msix_tx_message_ids))
		return 0;

	for (rx = 0; rx < num_rx_queues; rx++) {
		u8 message_id = hw->msix_rx_message_ids[rx];
		bool tx_source = false;

		if (message_id >= BITS_PER_TYPE(u32))
			return 0;
		for (tx = 0; tx < num_tx_queues; tx++) {
			if (message_id != hw->msix_tx_message_ids[tx])
				continue;
			tx_source = true;
			overlaps = true;
			break;
		}
		if (!tx_source)
			non_tx_rx_sources |= BIT(message_id);
	}

	/*
	 * All TX rings share a service owner. RX sources that
	 * overlap TX join that owner; otherwise the TX owner can also service
	 * one RX source, preserving the existing compact RTL8125B topology.
	 */
	if (overlaps)
		return 1 + hweight32(non_tx_rx_sources);

	return max_t(u16, 1, hweight32(non_tx_rx_sources));
}

static int
rtlmq_validate_profile_topology(const struct rtlmq_chip_profile *profile,
				const struct rtlmq_topology *topology)
{
	const struct rtlmq_qualified_policy *qualified = profile->qualified;
	const struct rtlmq_hw_contract *hw = profile->hw;
	bool sources_overlap;
	u16 service_owners;
	unsigned int i;

	if (!topology->rx_queues || !topology->tx_queues ||
	    !is_power_of_2(topology->rx_queues) ||
	    !is_power_of_2(topology->tx_queues))
		return -EINVAL;
	if (topology->rx_queues > hw->max_rx_queues ||
	    topology->rx_queues > RTLMQ_IMPL_MAX_RX_QUEUES ||
	    topology->tx_queues > hw->max_tx_queues ||
	    topology->tx_queues > RTLMQ_IMPL_MAX_TX_QUEUES)
		return -EINVAL;
	if (!topology->vectors ||
	    topology->vectors > hw->max_runtime_vectors ||
	    topology->vectors > hw->max_irq_vectors ||
	    topology->vectors > RTLMQ_IMPL_MAX_NAPI)
		return -EINVAL;
	if (!topology->irq_modes ||
	    topology->irq_modes & ~RTLMQ_PROFILE_IRQ_MODES)
		return -EINVAL;
	if ((topology->irq_modes & BIT(RTLMQ_IRQ_MODE_MSI)) &&
	    !hw->has_msi)
		return -EINVAL;
	if ((topology->irq_modes & BIT(RTLMQ_IRQ_MODE_MSIX)) &&
	    !hw->has_msix)
		return -EINVAL;

	switch (topology->routing) {
	case RTLMQ_ROUTING_SINGLE_VECTOR:
		if (topology->vectors != RTLMQ_BOOT_VECTORS)
			return -EINVAL;
		break;
	case RTLMQ_ROUTING_RX_PER_VECTOR_TX_COUPLED:
		service_owners =
			rtlmq_msix_service_owner_count(hw,
						       topology->rx_queues,
						       topology->tx_queues);
		sources_overlap =
			rtlmq_msix_rx_tx_sources_overlap(hw,
							 topology->rx_queues,
							 topology->tx_queues);
		if (!service_owners ||
		    (topology->vectors != service_owners &&
		     (sources_overlap ||
		      topology->vectors != service_owners + 1)) ||
		    topology->irq_modes != BIT(RTLMQ_IRQ_MODE_MSIX) ||
		    !hw->has_msix || !hw->has_msix_message_routing)
			return -EINVAL;
		for (i = 0; i < topology->rx_queues; i++) {
			if (!rtlmq_profile_message_id_valid(hw,
							    hw->msix_rx_message_ids[i]))
				return -EINVAL;
		}
		for (i = 0; i < topology->tx_queues; i++) {
			if (!rtlmq_profile_message_id_valid(hw,
							    hw->msix_tx_message_ids[i]))
				return -EINVAL;
		}
		break;
	default:
		return -EINVAL;
	}

	if (topology->rx_queues > RTLMQ_BOOT_RX_QUEUES &&
	    (!qualified->supports_rss || !hw->has_rss ||
	     !rtlmq_profile_rss_format_valid(hw->rss_rx_desc_fmt) ||
	     topology->routing !=
		     RTLMQ_ROUTING_RX_PER_VECTOR_TX_COUPLED))
		return -EINVAL;
	if (topology->tx_queues > RTLMQ_BOOT_TX_QUEUES &&
	    (!hw->has_q1_tx_desc_base_regs ||
	     topology->routing !=
		     RTLMQ_ROUTING_RX_PER_VECTOR_TX_COUPLED))
		return -EINVAL;

	return 0;
}

static int rtlmq_validate_chip_profile(const struct rtlmq_chip_profile *profile)
{
	const struct rtlmq_qualified_policy *qualified;
	const struct rtlmq_chip_match *match;
	const struct rtlmq_hw_contract *hw;
	bool rss_topology_admitted = false;
	const struct rtlmq_chip_ops *ops;
	bool default_admitted = false;
	bool tx2_admitted = false;
	unsigned int i;
	unsigned int j;

	if (!profile || !profile->name || !profile->name[0] ||
	    !profile->match || !profile->hw || !profile->qualified ||
	    !profile->ops)
		return -EINVAL;

	match = profile->match;
	hw = profile->hw;
	qualified = profile->qualified;
	ops = profile->ops;

	if (!match->pci_vendor || !match->pci_device || !match->xid_mask ||
	    match->xid_mask & ~RTLMQ_TX_CONFIG_XID_MASK ||
	    match->xid & ~RTLMQ_TX_CONFIG_XID_MASK ||
	    !match->accepted_config_methods ||
	    match->accepted_config_methods & ~RTLMQ_PROFILE_CONFIG_METHODS)
		return -EINVAL;
	if (hw->mac_version == RTLMQ_MAC_NONE || !hw->max_rx_queues ||
	    !hw->max_tx_queues || !hw->max_irq_vectors ||
	    !hw->max_runtime_vectors ||
	    hw->max_runtime_vectors > hw->max_irq_vectors ||
	    hw->tx_desc_fmt > RTLMQ_TX_DESC_FORMAT_V3 ||
	    (hw->tx_desc_stride && hw->tx_desc_stride != 16 &&
	     hw->tx_desc_stride != 32) ||
	    (hw->irq_moderation_version != RTLMQ_IRQ_MODERATION_V2 &&
	     hw->irq_moderation_version != RTLMQ_IRQ_MODERATION_V5
	    ) ||
	    (hw->has_msix_message_routing && !hw->has_msix))
		return -EINVAL;
	if (!ops->prepare_mac || !ops->prepare_reset ||
	    !ops->configure_mac || !ops->configure_phy)
		return -EINVAL;
	if (!qualified->topologies || !qualified->num_topologies ||
	    qualified->num_topologies >
		    RTLMQ_IMPL_MAX_RX_QUEUES * RTLMQ_IMPL_MAX_TX_QUEUES ||
	    !qualified->default_topology)
		return -EINVAL;
	if (qualified->supports_rss &&
	    (!hw->has_rss ||
	     hw->rss_version != RTLMQ_RSS_VERSION_5 ||
	     !rtlmq_profile_rss_format_valid(hw->rss_rx_desc_fmt)))
		return -EINVAL;
	if (qualified->supports_eee &&
	    (hw->eee_tx_lpi_layout <= RTLMQ_EEE_TX_LPI_NONE ||
	     hw->eee_tx_lpi_layout > RTLMQ_EEE_TX_LPI_E040_EB62))
		return -EINVAL;
	if (qualified->ptp_backend > RTLMQ_PTP_BACKEND_PHY_V3)
		return -EINVAL;
	if (qualified->supports_af_xdp && !qualified->supports_xdp)
		return -EINVAL;
	if (qualified->supports_cbs && !qualified->supports_tc)
		return -EINVAL;
	if (qualified->wol_opts & ~RTLMQ_WOL_SUPPORTED_OPTS)
		return -EINVAL;
	if (qualified->max_mtu &&
	    (qualified->max_mtu < ETH_MIN_MTU ||
	     qualified->max_mtu > rtlmq_max_supported_mtu()))
		return -EINVAL;
	if (qualified->max_tso_size &&
	    (qualified->max_tso_size <
		     rtlmq_max_supported_mtu() + VLAN_ETH_HLEN ||
	     qualified->max_tso_size > RTLMQ_TSO_MAX_SKB_BYTES))
		return -EINVAL;

	for (i = 0; i < qualified->num_topologies; i++) {
		const struct rtlmq_topology *topology =
			&qualified->topologies[i];

		if (rtlmq_validate_profile_topology(profile, topology))
			return -EINVAL;
		if (topology == qualified->default_topology)
			default_admitted = true;
		if (topology->rx_queues > RTLMQ_BOOT_RX_QUEUES)
			rss_topology_admitted = true;
		if (topology->tx_queues > RTLMQ_BOOT_TX_QUEUES)
			tx2_admitted = true;

		for (j = 0; j < i; j++) {
			const struct rtlmq_topology *other =
				&qualified->topologies[j];

			if (topology->rx_queues == other->rx_queues &&
			    topology->tx_queues == other->tx_queues)
				return -EINVAL;
		}
	}

	if (!default_admitted ||
	    (qualified->supports_rss && !rss_topology_admitted) ||
	    (qualified->supports_tc && !tx2_admitted) ||
	    qualified->max_xsk_tx_queues > hw->max_tx_queues ||
	    qualified->max_xsk_tx_queues > RTLMQ_IMPL_MAX_TX_QUEUES ||
	    (qualified->max_xsk_tx_queues > 1 && !tx2_admitted))
		return -EINVAL;

	return 0;
}

static bool rtlmq_chip_matches_overlap(const struct rtlmq_chip_match *a,
				       const struct rtlmq_chip_match *b)
{
	if (a->pci_vendor != b->pci_vendor ||
	    a->pci_device != b->pci_device)
		return false;

	/*
	 * Lookup matches the PCI ID and XID before checking the config method.
	 * If two entries match the same raw XID, whichever appears first wins,
	 * even if their accepted config methods do not overlap.
	 */
	return !((a->xid ^ b->xid) & a->xid_mask & b->xid_mask);
}

static int rtlmq_validate_chip_profile_registry(const struct rtlmq_chip_profile *const *registry,
						size_t count)
{
	unsigned int i;
	unsigned int j;

	if (!registry || !count)
		return -EINVAL;

	for (i = 0; i < count; i++) {
		const struct rtlmq_chip_profile *profile = registry[i];

		if (rtlmq_validate_chip_profile(profile))
			return -EINVAL;
		for (j = 0; j < i; j++) {
			const struct rtlmq_chip_profile *other = registry[j];

			if (rtlmq_chip_matches_overlap(profile->match,
						       other->match))
				return -EEXIST;
		}
	}

	return 0;
}

static int rtlmq_seed_bootstrap_runtime_config(struct rtlmq_runtime_config *cfg,
					       const struct rtlmq_private *tp)
{
	const struct rtlmq_topology *topology =
		rtlmq_qualified_default_topology(tp);

	/*
	 * Profile validation already checked that the default topology is
	 * allowed by both policy and hardware. If it is invalid, fail instead
	 * of silently falling back to a smaller bootstrap topology.
	 */
	if (!topology ||
	    !rtlmq_validated_queue_shape_allowed(tp, topology->rx_queues,
						 topology->tx_queues))
		return -EINVAL;

	cfg->requested_num_rx_queues = topology->rx_queues;
	cfg->requested_num_tx_queues = topology->tx_queues;

	cfg->num_rx_queues = cfg->requested_num_rx_queues;
	cfg->num_tx_queues = cfg->requested_num_tx_queues;
	cfg->rx_ring_count = RTLMQ_NUM_RX_DESC;
	cfg->tx_ring_count = RTLMQ_NUM_TX_DESC;
	cfg->rss_enabled = topology->rx_queues > RTLMQ_BOOT_RX_QUEUES;
	cfg->num_vectors = topology->vectors;
	cfg->routing_mode = topology->routing;
	cfg->irq_mode = RTLMQ_IRQ_MODE_NONE;
	return 0;
}

const char *rtlmq_config_method_name(enum rtlmq_config_method method)
{
	switch (method) {
	case RTLMQ_CFG_METHOD_2:
		return "CFG_METHOD_2";
	case RTLMQ_CFG_METHOD_3:
		return "CFG_METHOD_3";
	case RTLMQ_CFG_METHOD_4:
		return "CFG_METHOD_4";
	case RTLMQ_CFG_METHOD_5:
		return "CFG_METHOD_5";
	case RTLMQ_CFG_METHOD_UNKNOWN:
	default:
		return "unknown";
	}
}

static enum rtlmq_config_method rtlmq_detect_config_method(u32 txconfig)
{
	u32 ic_version = txconfig & RTLMQ_TX_CONFIG_IC_VERSION_MASK;

	switch (txconfig & RTLMQ_TX_CONFIG_CFG_FAMILY_MASK) {
	case RTLMQ_TX_CONFIG_CFG_FAMILY_8125B:
		switch (ic_version) {
		case RTLMQ_TX_CONFIG_IC_VERSION_METHOD_4:
			return RTLMQ_CFG_METHOD_4;
		case RTLMQ_TX_CONFIG_IC_VERSION_METHOD_5:
			return RTLMQ_CFG_METHOD_5;
		}
		break;
	case RTLMQ_TX_CONFIG_CFG_FAMILY_8126A:
		switch (ic_version) {
		case RTLMQ_8126_TX_CONFIG_IC_VERSION_METHOD_2:
			return RTLMQ_CFG_METHOD_2;
		case RTLMQ_8126_TX_CONFIG_IC_VERSION_METHOD_3:
			return RTLMQ_CFG_METHOD_3;
		}
		break;
	case RTLMQ_TX_CONFIG_CFG_FAMILY_8127A:
		if (ic_version == RTLMQ_8127_TX_CONFIG_IC_VERSION_METHOD_2)
			return RTLMQ_CFG_METHOD_2;
		break;
	default:
		break;
	}

	return RTLMQ_CFG_METHOD_UNKNOWN;
}

static bool rtlmq_chip_match_pci(const struct rtlmq_chip_match *match,
				 const struct pci_dev *pdev)
{
	return match && pdev &&
	       match->pci_vendor == pdev->vendor &&
	       match->pci_device == pdev->device;
}

static bool rtlmq_chip_match_xid(const struct rtlmq_chip_match *match,
				 u16 raw_xid)
{
	return match && (raw_xid & match->xid_mask) ==
			(match->xid & match->xid_mask);
}

static bool rtlmq_supported_pci_profile_exists(const struct pci_dev *pdev)
{
	unsigned int i;

	if (!pdev)
		return false;

	for (i = 0; i < ARRAY_SIZE(rtlmq_production_registry); i++) {
		const struct rtlmq_chip_profile *profile =
			rtlmq_production_registry[i];
		const struct rtlmq_chip_match *match =
			rtlmq_chip_profile_match(profile);

		if (rtlmq_chip_match_pci(match, pdev))
			return true;
	}

	return false;
}

static const struct rtlmq_chip_profile *
rtlmq_find_chip_profile(const struct pci_dev *pdev, u16 raw_xid)
{
	unsigned int i;

	if (!pdev)
		return NULL;

	for (i = 0; i < ARRAY_SIZE(rtlmq_production_registry); i++) {
		const struct rtlmq_chip_profile *profile =
			rtlmq_production_registry[i];
		const struct rtlmq_chip_match *match =
			rtlmq_chip_profile_match(profile);

		if (!rtlmq_chip_match_pci(match, pdev))
			continue;
		if (rtlmq_chip_match_xid(match, raw_xid))
			return profile;
	}

	return NULL;
}

static bool
rtlmq_match_accepts_config_method(const struct rtlmq_chip_match *match,
				  enum rtlmq_config_method config_method)
{
	if (!match || config_method <= RTLMQ_CFG_METHOD_UNKNOWN ||
	    config_method >= sizeof(match->accepted_config_methods) *
				     BITS_PER_BYTE)
		return false;

	return match->accepted_config_methods & BIT(config_method);
}

static bool
rtlmq_profile_accepts_config_method(const struct rtlmq_chip_profile *profile,
				    enum rtlmq_config_method config_method)
{
	return rtlmq_match_accepts_config_method(rtlmq_chip_profile_match(profile), config_method);
}

bool rtlmq_chip_profile_is_authoritative(const struct rtlmq_private *tp)
{
	const struct rtlmq_qualified_policy *qualified;
	const struct rtlmq_chip_profile *registered;
	const struct rtlmq_chip_match *match;
	const struct rtlmq_hw_contract *hw;

	if (!tp || !tp->pdev || !tp->profile)
		return false;

	registered = rtlmq_find_chip_profile(tp->pdev, tp->raw_xid);
	if (registered != tp->profile)
		return false;

	match = rtlmq_chip_profile_match(tp->profile);
	hw = rtlmq_chip_profile_hw(tp->profile);
	qualified = rtlmq_chip_profile_qualified(tp->profile);
	if (!match || !hw || !qualified ||
	    tp->xid != match->xid ||
	    !rtlmq_profile_accepts_config_method(tp->profile,
						 tp->config_method))
		return false;

	return true;
}

static const struct rtlmq_chip_ops *
rtlmq_selected_chip_ops(const struct rtlmq_private *tp)
{
	return tp ? rtlmq_chip_profile_ops(tp->profile) : NULL;
}

bool rtlmq_chip_supports_rss(const struct rtlmq_private *tp)
{
	return rtlmq_qualified_supports_rss(tp) && rtlmq_hw_has_rss(tp);
}

bool rtlmq_chip_supports_xdp(const struct rtlmq_private *tp)
{
	const struct rtlmq_qualified_policy *qualified =
		rtlmq_qualified_policy_for(tp);

	return qualified && qualified->supports_xdp;
}

bool rtlmq_chip_supports_af_xdp(const struct rtlmq_private *tp)
{
	return rtlmq_chip_supports_xdp(tp) &&
	       rtlmq_qualified_supports_af_xdp(tp);
}

bool rtlmq_chip_supports_tc(const struct rtlmq_private *tp)
{
	const struct rtlmq_qualified_policy *qualified =
		rtlmq_qualified_policy_for(tp);

	return qualified && qualified->supports_tc &&
	       rtlmq_validated_max_tx_queues(tp) > RTLMQ_BOOT_TX_QUEUES &&
	       rtlmq_hw_has_q1_tx_desc_base_regs(tp);
}

bool rtlmq_chip_supports_cbs(const struct rtlmq_private *tp)
{
	const struct rtlmq_qualified_policy *qualified =
		rtlmq_qualified_policy_for(tp);

	return qualified && qualified->supports_cbs &&
	       rtlmq_chip_supports_tc(tp);
}

bool rtlmq_chip_irq_mode_admitted(const struct rtlmq_private *tp,
				  enum rtlmq_irq_mode mode)
{
	if (!rtlmq_qualified_irq_mode_allowed(tp, mode))
		return false;

	switch (mode) {
	case RTLMQ_IRQ_MODE_INTX:
		return true;
	case RTLMQ_IRQ_MODE_MSI:
		return rtlmq_hw_has_msi(tp);
	case RTLMQ_IRQ_MODE_MSIX:
		return rtlmq_hw_has_msix(tp);
	case RTLMQ_IRQ_MODE_NONE:
	default:
		return false;
	}
}

bool rtlmq_chip_routing_mode_admitted(const struct rtlmq_private *tp,
				      enum rtlmq_routing_mode mode)
{
	if (!rtlmq_qualified_routing_mode_allowed(tp, mode))
		return false;

	switch (mode) {
	case RTLMQ_ROUTING_SINGLE_VECTOR:
		return true;
	case RTLMQ_ROUTING_RX_PER_VECTOR_TX_COUPLED:
		return rtlmq_hw_has_msix_message_routing(tp);
	default:
		return false;
	}
}

int rtlmq_chip_prepare_mac(struct rtlmq_private *tp)
{
	const struct rtlmq_chip_ops *ops = rtlmq_selected_chip_ops(tp);

	return ops && ops->prepare_mac ? ops->prepare_mac(tp) : -EOPNOTSUPP;
}

int rtlmq_chip_prepare_reset(struct rtlmq_private *tp, const char *where)
{
	const struct rtlmq_chip_ops *ops = rtlmq_selected_chip_ops(tp);

	return ops && ops->prepare_reset ?
	       ops->prepare_reset(tp, where) : -EOPNOTSUPP;
}

int rtlmq_chip_configure_mac(struct rtlmq_private *tp)
{
	const struct rtlmq_chip_ops *ops = rtlmq_selected_chip_ops(tp);

	return ops && ops->configure_mac ?
	       ops->configure_mac(tp) : -EOPNOTSUPP;
}

int rtlmq_chip_configure_phy(struct rtlmq_private *tp)
{
	const struct rtlmq_chip_ops *ops = rtlmq_selected_chip_ops(tp);

	return ops && ops->configure_phy ?
	       ops->configure_phy(tp) : -EOPNOTSUPP;
}

int rtlmq_program_tx_engine_runtime(struct rtlmq_private *tp, const char *where)
{
	const struct rtlmq_hw_contract *hw;
	const struct rtlmq_chip_ops *ops;

	if (!tp || !rtlmq_mmio_ordinary_authorized(tp))
		return -ENODEV;
	lockdep_assert_held(&tp->control_lock);
	hw = rtlmq_chip_profile_hw(tp->profile);
	ops = rtlmq_selected_chip_ops(tp);
	if (!hw || !ops)
		return -EOPNOTSUPP;
	if (!hw->requires_tx_engine_runtime_latch)
		return 0;

	return ops->latch_tx_engine ?
	       ops->latch_tx_engine(tp, where) : -EOPNOTSUPP;
}

bool rtlmq_chip_needs_conservative_aspm(const struct rtlmq_private *tp)
{
	const struct rtlmq_hw_contract *hw =
		tp ? rtlmq_chip_profile_hw(tp->profile) : NULL;

	return hw && hw->requires_conservative_aspm;
}

int rtlmq_init_runtime_config(struct rtlmq_private *tp)
{
	struct rtlmq_runtime_config *cfg;
	int ret;

	if (!tp)
		return -EINVAL;

	memset(&tp->requested_profile, 0, sizeof(tp->requested_profile));
	memset(&tp->transaction, 0, sizeof(tp->transaction));
	memset(&tp->active_profile, 0, sizeof(tp->active_profile));

	tp->active_irq_vectors_allocated = false;
	tp->retired_generation_quiesced = false;

	cfg = &tp->requested_profile.config;
	ret = rtlmq_seed_bootstrap_runtime_config(cfg, tp);
	if (ret)
		return ret;

	rtlmq_set_requested_queue_counts(tp, cfg->requested_num_rx_queues,
					 cfg->requested_num_tx_queues);
	if (rtlmq_hw_irq_moderation_version(tp) ==
		    RTLMQ_IRQ_MODERATION_V5
	   ) {
		/*
		 * Disabled banks have verified replay but no ethtool time
		 * or frame conversion. Keep them disabled by default.
		 */
		rtlmq_set_requested_irq_coalesce_usecs(tp, 0);
		rtlmq_set_requested_irq_coalesce_frames(tp, 0);
		rtlmq_set_requested_tx_irq_coalesce_usecs(tp, 0);
		rtlmq_set_requested_tx_irq_coalesce_frames(tp, 0);
	} else {
		u32 rx_usecs = rtlmq_default_rx_irq_coalesce_usecs();
		u32 rx_frames = rtlmq_default_rx_irq_coalesce_frames();
		u32 tx_usecs = rtlmq_default_tx_irq_coalesce_usecs();
		u32 tx_frames = rtlmq_default_tx_irq_coalesce_frames();

		rtlmq_set_requested_irq_coalesce_usecs(tp, rx_usecs);
		rtlmq_set_requested_irq_coalesce_frames(tp, rx_frames);
		rtlmq_set_requested_tx_irq_coalesce_usecs(tp, tx_usecs);
		rtlmq_set_requested_tx_irq_coalesce_frames(tp, tx_frames);
	}

	WRITE_ONCE(tp->rx_desc_fmt, RX_DESC_FMT_V1);

	tp->requested_profile.valid = true;
	return 0;
}

int rtlmq_identify_chip(struct rtlmq_private *tp)
{
	const struct rtlmq_qualified_policy *qualified;
	const struct rtlmq_chip_profile *profile;
	const struct rtlmq_runtime_config *cfg;
	enum rtlmq_config_method config_method;
	const struct rtlmq_chip_match *match;
	const struct rtlmq_hw_contract *hw;
	u32 txconfig;
	u16 raw_xid;
	int ret;

	if (!tp || !rtlmq_mmio_ordinary_authorized(tp) || !tp->pdev)
		return -ENODEV;

	/*
	 * Clear the previous identity before probing this device. If
	 * identification fails, chip-specific code must not be able to reuse
	 * the old profile.
	 */
	tp->xid = 0;
	tp->raw_xid = 0;
	tp->config_method = RTLMQ_CFG_METHOD_UNKNOWN;
	tp->profile = NULL;

	ret = rtlmq_validate_chip_profile_registry(rtlmq_production_registry,
						   ARRAY_SIZE(rtlmq_production_registry));
	if (ret) {
		netdev_err(tp->ndev,
			   "invalid production chip profile registry: %d\n",
			   ret);
		return ret;
	}

	if (!rtlmq_supported_pci_profile_exists(tp->pdev)) {
		netdev_info(tp->ndev,
			    "unsupported PCI device %04x:%04x left unclaimed\n",
			    tp->pdev->vendor, tp->pdev->device);
		return -ENODEV;
	}

	txconfig = readl(tp->mmio_addr + RTLMQ_TX_CONFIG);
	if (txconfig == ~0U)
		return -EIO;

	raw_xid = (txconfig >> RTLMQ_TX_CONFIG_XID_SHIFT) &
		  RTLMQ_TX_CONFIG_XID_MASK;

	/*
	 * Decode the family and IC version together. PCI revision is not
	 * configuration-method authority; unknown IC versions never fall back.
	 */
	config_method = rtlmq_detect_config_method(txconfig);

	profile = rtlmq_find_chip_profile(tp->pdev, raw_xid);
	if (!profile) {
		netdev_info(tp->ndev,
			    "unsupported device %04x:%04x XID %#x left unclaimed\n",
			    tp->pdev->vendor, tp->pdev->device, raw_xid);
		return -ENODEV;
	}

	match = rtlmq_chip_profile_match(profile);
	hw = rtlmq_chip_profile_hw(profile);
	qualified = rtlmq_chip_profile_qualified(profile);
	if (WARN_ON_ONCE(!match || !hw || !qualified))
		return -EINVAL;

	if (!rtlmq_profile_accepts_config_method(profile, config_method)) {
		netdev_info(tp->ndev,
			    "unsupported %s config method for XID %#x\n",
			    rtlmq_chip_profile_name(profile), raw_xid);
		return -ENODEV;
	}

	tp->raw_xid = raw_xid;
	tp->xid = match->xid;
	tp->config_method = config_method;
	tp->profile = profile;

	ret = rtlmq_init_runtime_config(tp);
	if (ret)
		return ret;

	cfg = rtlmq_requested_runtime_config(tp);
	if (WARN_ON_ONCE(!cfg))
		return -EINVAL;

	return 0;
}

enum rtlmq_rx_desc_format
rtlmq_wanted_rx_desc_fmt(const struct rtlmq_private *tp)
{
	const struct rtlmq_runtime_config *cfg;

	cfg = rtlmq_candidate_runtime_config(tp);
	if (!cfg)
		cfg = rtlmq_active_or_requested_runtime_config(tp);

	/*
	 * Realtek 9.018.00 uses V3 for CFG_METHOD_4/5 whenever PHY PTP is
	 * enabled, even when RSS is off. PHY ingress timestamps do not use V3
	 * companion descriptors, but the PTP-capable hardware setup still
	 * expects V3 descriptors.
	 */
	if (rtlmq_qualified_ptp_backend(tp) == RTLMQ_PTP_BACKEND_PHY_V3)
		return RX_DESC_FMT_V3;

	if (!cfg || !cfg->rss_enabled)
		return RX_DESC_FMT_V1;

	return rtlmq_hw_rss_rx_desc_fmt(tp);
}
