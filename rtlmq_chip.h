/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef __RTLMQ_CHIP_H__
#define __RTLMQ_CHIP_H__

#include <linux/netdev_features.h>

#include "rtlmq_base.h"
#include "rtlmq_desc.h"

struct pci_dev;

enum rtlmq_rss_version {
	RTLMQ_RSS_VERSION_NONE = 0,
	RTLMQ_RSS_VERSION_5 = 5,
};

enum rtlmq_tx_desc_format {
	RTLMQ_TX_DESC_FORMAT_LEGACY = 0,
	RTLMQ_TX_DESC_FORMAT_V3,
};

enum rtlmq_irq_moderation_version {
	RTLMQ_IRQ_MODERATION_NONE = 0,
	RTLMQ_IRQ_MODERATION_V2 = 2,
	RTLMQ_IRQ_MODERATION_V5 = 5,
};

/*
 * Keep chip identity, hardware capabilities, and production policy separate.
 * Hardware limits can further restrict a validated profile, but hardware
 * support alone does not enable a feature.
 */
struct rtlmq_chip_match {
	u16 pci_vendor;
	u16 pci_device;
	u16 xid;
	u16 xid_mask;
	u16 accepted_config_methods;
};

enum rtlmq_8126_match_id {
	RTLMQ_8126_A2,
	RTLMQ_8126_A3,
	RTLMQ_5000_A2,
	RTLMQ_5000_A3,
	RTLMQ_8126_MATCH_COUNT,
};

/* Exact family matches shared by the production profiles and identity tests. */
extern const struct rtlmq_chip_match rtlmq_8126_matches[RTLMQ_8126_MATCH_COUNT];

struct rtlmq_hw_contract {
	enum rtlmq_mac_version mac_version;
	enum rtlmq_rx_desc_format rss_rx_desc_fmt;
	enum rtlmq_tx_desc_format tx_desc_fmt;
	/* Zero retains the existing 32-byte descriptor backing. */
	u8 tx_desc_stride;
	enum rtlmq_eee_tx_lpi_layout eee_tx_lpi_layout;
	enum rtlmq_rss_version rss_version;
	enum rtlmq_irq_moderation_version irq_moderation_version;
	u16 max_rx_queues;
	u16 max_tx_queues;
	u16 max_irq_vectors;
	u16 max_runtime_vectors;
	bool requires_tx_engine_runtime_latch;
	bool has_q1_tx_desc_base_regs;
	bool has_msi;
	bool has_msix;
	bool has_rss;
	bool has_msix_message_routing;
	bool requires_conservative_aspm;
	u8 msix_rx_message_ids[RTLMQ_IMPL_MAX_RX_QUEUES];
	u8 msix_tx_message_ids[RTLMQ_IMPL_MAX_TX_QUEUES];
	u8 msix_link_message_id;
	u8 msix_int_cfg0_clear_mask;
};

bool rtlmq_msix_rx_tx_sources_overlap(const struct rtlmq_hw_contract *hw,
				      u16 num_rx_queues, u16 num_tx_queues);
u16 rtlmq_msix_service_owner_count(const struct rtlmq_hw_contract *hw,
				   u16 num_rx_queues, u16 num_tx_queues);

/*
 * Each topology is a queue, vector, and IRQ combination reviewed together.
 * Do not mix queue counts with a different vector layout or routing mode.
 * Multiqueue RX requires RSS; single-RX topologies leave it disabled.
 * Service contexts follow physical message identity. RX sources overlapping
 * TX share its owner; disjoint sources may use an additional TX-only context.
 * All TX rings share a service owner.
 */
struct rtlmq_topology {
	u16 rx_queues;
	u16 tx_queues;
	u16 vectors;
	enum rtlmq_routing_mode routing;
	u8 irq_modes;
};

struct rtlmq_qualified_policy {
	netdev_features_t base_netdev_features;
	/* Zero uses the shared RX geometry limit. */
	u32 max_mtu;
	/* Zero uses the hardware TSO request ceiling. */
	u32 max_tso_size;
	enum rtlmq_ptp_backend ptp_backend;
	const struct rtlmq_topology *topologies;
	const struct rtlmq_topology *default_topology;
	u16 num_topologies;
	u16 max_xsk_tx_queues;
	bool use_core_napi_defaults;
	bool supports_rss;
	bool supports_xdp;
	bool supports_af_xdp;
	bool supports_tc;
	bool supports_vlan_ctag_tx;
	bool supports_vlan_ctag_rx;
	bool supports_rx_fcs;
	bool supports_rx_all;
	bool supports_highdma;
	bool supports_hw_tally;
	bool supports_live_mac_change;
	bool supports_inplace_ring_resize;
	bool supports_eee;
	bool supports_leds;
	u32 wol_opts;
	bool wol_link_v3;
	bool supports_cbs;
};

struct rtlmq_chip_ops {
	/* Complete bootstrap, including reset, without enabling DMA engines. */
	int (*prepare_mac)(struct rtlmq_private *tp);
	int (*prepare_reset)(struct rtlmq_private *tp, const char *where);
	int (*configure_mac)(struct rtlmq_private *tp);
	int (*configure_phy)(struct rtlmq_private *tp);
	/* After descriptor bases and queue counts, before DMA enable. */
	int (*latch_tx_engine)(struct rtlmq_private *tp, const char *where);
};

/*
 * Firmware profile data is immutable. The parser builds its own normalized
 * action list and must stay within these limits.
 */
struct rtlmq_fw_profile {
	const char *name;
	const char *version_prefix;
	u32 max_actions;
	u32 max_steps;
	u32 max_total_delay_ms;
};

struct rtlmq_regdump_register {
	u16 offset;
	u8 width;
};

struct rtlmq_regdump_schema {
	const struct rtlmq_regdump_register *registers;
	u16 register_count;
	const struct rtlmq_regdump_register *moderation;
	u16 moderation_count;
};

struct rtlmq_chip_profile {
	const char *name;
	const struct rtlmq_chip_match *match;
	const struct rtlmq_hw_contract *hw;
	const struct rtlmq_qualified_policy *qualified;
	const struct rtlmq_chip_ops *ops;
	const struct rtlmq_fw_profile *firmware;
	const struct rtlmq_regdump_schema *regdump;
};

extern const struct rtlmq_fw_profile rtlmq_8125b_firmware;
extern const struct rtlmq_fw_profile rtlmq_8126a2_firmware;
extern const struct rtlmq_fw_profile rtlmq_8126a3_firmware;
extern const struct rtlmq_regdump_schema rtlmq_8125b_regdump_schema;
extern const struct rtlmq_regdump_schema rtlmq_8126a_regdump_schema;
extern const struct rtlmq_chip_profile rtlmq_8125b_profile;
extern const struct rtlmq_chip_profile rtlmq_8126a2_profile;
extern const struct rtlmq_chip_profile rtlmq_8126a3_profile;
extern const struct rtlmq_chip_profile rtlmq_5000a2_profile;
extern const struct rtlmq_chip_profile rtlmq_5000a3_profile;

bool rtlmq_chip_profile_is_authoritative(const struct rtlmq_private *tp);
bool rtlmq_chip_supports_rss(const struct rtlmq_private *tp);
bool rtlmq_chip_supports_xdp(const struct rtlmq_private *tp);
bool rtlmq_chip_supports_af_xdp(const struct rtlmq_private *tp);
bool rtlmq_chip_supports_tc(const struct rtlmq_private *tp);
bool rtlmq_chip_supports_cbs(const struct rtlmq_private *tp);
bool rtlmq_chip_irq_mode_admitted(const struct rtlmq_private *tp,
				  enum rtlmq_irq_mode mode);
bool rtlmq_chip_routing_mode_admitted(const struct rtlmq_private *tp,
				      enum rtlmq_routing_mode mode);
int rtlmq_chip_prepare_mac(struct rtlmq_private *tp);
int rtlmq_chip_prepare_reset(struct rtlmq_private *tp, const char *where);
int rtlmq_chip_configure_mac(struct rtlmq_private *tp);
int rtlmq_chip_configure_phy(struct rtlmq_private *tp);
const char *rtlmq_chip_profile_name(const struct rtlmq_chip_profile *profile);
const struct rtlmq_chip_match *
rtlmq_chip_profile_match(const struct rtlmq_chip_profile *profile);
const struct rtlmq_hw_contract *
rtlmq_chip_profile_hw(const struct rtlmq_chip_profile *profile);
const struct rtlmq_qualified_policy *
rtlmq_chip_profile_qualified(const struct rtlmq_chip_profile *profile);
const struct rtlmq_chip_ops *
rtlmq_chip_profile_ops(const struct rtlmq_chip_profile *profile);
const struct rtlmq_fw_profile *
rtlmq_chip_profile_firmware(const struct rtlmq_chip_profile *profile);
const struct rtlmq_regdump_schema *
rtlmq_chip_profile_regdump(const struct rtlmq_chip_profile *profile);
u16 rtlmq_qualified_topology_count(const struct rtlmq_private *tp);
const struct rtlmq_topology *
rtlmq_qualified_default_topology(const struct rtlmq_private *tp);
const struct rtlmq_topology *
rtlmq_find_qualified_topology(const struct rtlmq_private *tp, u16 rx_queues,
			      u16 tx_queues);

#endif /* __RTLMQ_CHIP_H__ */
