/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef __RTLMQ_DIAG_H__
#define __RTLMQ_DIAG_H__

#include <linux/build_bug.h>
#include <linux/types.h>

#include "rtlmq_base.h"

struct ethtool_stats;
struct ethtool_regs;
struct ethtool_coalesce;
struct ethtool_eth_ctrl_stats;
struct ethtool_eth_mac_stats;
struct ethtool_pause_stats;
struct ethtool_pauseparam;
struct ethtool_rmon_hist_range;
struct ethtool_rmon_stats;
struct kernel_ethtool_coalesce;
struct net_device;
struct netlink_ext_ack;
struct netdev_stat_ops;
struct rtlmq_private;
struct rtlmq_queue_stats;

extern const struct netdev_stat_ops rtlmq_stat_ops;

int rtlmq_alloc_counter_memory(struct rtlmq_private *tp);
void rtlmq_init_counter_offsets(struct rtlmq_private *tp);
int rtlmq_update_counters(struct rtlmq_private *tp);
void rtlmq_tally_reset_epoch(struct rtlmq_private *tp);
void rtlmq_get_queue_stats_totals(const struct rtlmq_private *tp,
				  struct rtlmq_queue_stats *stats);
void rtlmq_retire_queue_stats(struct rtlmq_private *tp);
int rtlmq_ethtool_get_coalesce(struct net_device *ndev,
			       struct ethtool_coalesce *ec,
			       struct kernel_ethtool_coalesce *kernel_coal,
			       struct netlink_ext_ack *extack);
int rtlmq_ethtool_set_coalesce(struct net_device *ndev,
			       struct ethtool_coalesce *ec,
			       struct kernel_ethtool_coalesce *kernel_coal,
			       struct netlink_ext_ack *extack);
void rtlmq_ethtool_get_pause_stats(struct net_device *ndev,
				   struct ethtool_pause_stats *pause_stats);
void rtlmq_ethtool_get_eth_mac_stats(struct net_device *ndev,
				     struct ethtool_eth_mac_stats *mac_stats);
void rtlmq_ethtool_get_eth_ctrl_stats(struct net_device *ndev,
				      struct ethtool_eth_ctrl_stats *ctrl_stats);
void rtlmq_ethtool_get_rmon_stats(struct net_device *ndev,
				  struct ethtool_rmon_stats *rmon_stats,
				  const struct ethtool_rmon_hist_range **ranges);
int rtlmq_ethtool_get_sset_count(struct net_device *ndev, int sset);
void rtlmq_ethtool_get_strings(struct net_device *ndev, u32 stringset,
			       u8 *data);
void rtlmq_ethtool_get_stats(struct net_device *ndev,
			     struct ethtool_stats *stats, u64 *data);
u32 rtlmq_ethtool_get_rx_ring_count(struct net_device *ndev);

#define RTLMQ_REG_DUMP_MAGIC		0x514d5452U
#define RTLMQ_REG_DUMP_FORMAT_VERSION	7U
#define RTLMQ_REG_DUMP_MAX_MODERATION_SLOTS	34

enum rtlmq_reg_dump_status {
	RTLMQ_REG_DUMP_VALID = 0,
	RTLMQ_REG_DUMP_UNSUPPORTED_PROFILE,
	RTLMQ_REG_DUMP_UNAVAILABLE_STATE,
	RTLMQ_REG_DUMP_MMIO_REVOKED,
	RTLMQ_REG_DUMP_PCI_UNAVAILABLE,
	RTLMQ_REG_DUMP_READ_FAILED,
};

/*
 * Stable little-endian format returned by ETHTOOL_GREGS. The header is
 * followed by entry_count register records, two CBS queue records, and one
 * TX service record, identity record, ownership map, moderation state, then
 * passive tally status.
 * Version 7 appends passive tally status to the version 6 layout.
 * Status qualifies MMIO readback only; software state is always captured.
 */
struct rtlmq_reg_dump_header {
	__le32 magic;
	__le16 format_version;
	__le16 header_length;
	__le16 entry_length;
	__le16 entry_count;
	__le32 total_length;
	__le32 status;
	__le16 pci_vendor;
	__le16 pci_device;
	__le16 subsystem_vendor;
	__le16 subsystem_device;
	__le16 xid;
	__le16 raw_xid;
	u8 mac_version;
	u8 config_method;
	u8 lifecycle_state;
	u8 mmio_authority;
	__le32 reserved[3];
} __packed;

struct rtlmq_reg_dump_entry {
	__le16 offset;
	u8 width;
	u8 reserved;
	__le32 value;
} __packed;

struct rtlmq_reg_dump_cbs {
	u8 queue;
	u8 enabled;
	u8 registers_valid;
	u8 active_valid;
	u8 link_waiting;
	u8 reserved[3];
	__le32 idleslope;
	__le32 sendslope;
	__le32 hicredit;
	__le32 locredit;
	__le32 encoded_idle_credit;
	/* Last committed value, meaningful only when registers_valid is set. */
	__le32 programmed_idle_credit;
} __packed;

struct rtlmq_reg_dump_tx_service {
	u8 requested_single_frame;
	u8 tso_constrained;
	u8 published;
	u8 reserved;
	/* Generation policy and committed moderation; zero when unpublished. */
	__le32 epoch_descs;
	__le32 runway_descs;
	__le32 programmed_tx_moderation;
	/* ethtool coalesce remains the administrative get/merge/set interface. */
	__le32 requested_tx_usecs;
	__le32 requested_tx_frames;
	/* Core/user GSO hint and device capability are not a CBS save/restore. */
	__le32 gso_max_segs;
	__le32 tso_max_segs;
	__le64 wanted_features;
	__le64 effective_features;
} __packed;

struct rtlmq_reg_dump_identity {
	/* Selected immutable profile and firmware image, not hardware readback. */
	char profile_name[32];
	char firmware_name[40];
	/* Cached parsed image version; empty if no image is cached. */
	char firmware_version[32];
	/* Valid only when published; zero otherwise. */
	__le32 resource_generation;
	u8 published;
	u8 firmware_cached;
	u8 rx_desc_format;
	u8 tx_desc_format;
} __packed;

struct rtlmq_reg_dump_source {
	u8 message_id;
	u8 role;
	u8 kind_flags;
	u8 reserved;
} __packed;

struct rtlmq_reg_dump_vector {
	__le32 legacy_irq_mask;
	__le16 rx_queue_mask;
	__le16 tx_queue_mask;
	u8 source_count;
	u8 reserved[3];
	struct rtlmq_reg_dump_source sources[RTLMQ_MAX_IRQ_SOURCES];
} __packed;

struct rtlmq_reg_dump_ownership {
	u8 valid;
	u8 vector_count;
	u8 irq_mode;
	u8 routing_mode;
	struct rtlmq_reg_dump_vector vectors[RTLMQ_IMPL_MAX_NAPI];
} __packed;

struct rtlmq_reg_dump_moderation {
	__le32 requested_rx_usecs;
	__le32 requested_rx_frames;
	__le32 requested_tx_usecs;
	__le32 requested_tx_frames;
	/* Encoded image from the committed request, not register readback. */
	__le16 applied_rx_image;
	__le16 applied_tx_image;
	u8 applied_valid;
	u8 readback_status;
	u8 version;
	u8 slot_count;
	/* Fixed profile allowlist; values valid only on VALID readback_status. */
	struct rtlmq_reg_dump_entry slots[RTLMQ_REG_DUMP_MAX_MODERATION_SLOTS];
} __packed;

struct rtlmq_reg_dump_tally {
	/* Cached tally state; reading this record does not request a dump. */
	u8 sample_state;
	u8 committed_valid;
	u8 raw_valid;
	u8 reserved;
	__le64 command_failures;
} __packed;

static_assert(sizeof(struct rtlmq_reg_dump_header) == 48);
static_assert(sizeof(struct rtlmq_reg_dump_entry) == 8);
static_assert(sizeof(struct rtlmq_reg_dump_cbs) == 32);
static_assert(sizeof(struct rtlmq_reg_dump_tx_service) == 48);
static_assert(sizeof(struct rtlmq_reg_dump_identity) == 112);
static_assert(sizeof(struct rtlmq_reg_dump_source) == 4);
static_assert(sizeof(struct rtlmq_reg_dump_vector) == 36);
static_assert(sizeof(struct rtlmq_reg_dump_ownership) == 184);
static_assert(RTLMQ_IMPL_MAX_RX_QUEUES <= 16);
static_assert(RTLMQ_IMPL_MAX_TX_QUEUES <= 16);
static_assert(sizeof(struct rtlmq_reg_dump_moderation) == 296);
static_assert(sizeof(struct rtlmq_reg_dump_tally) == 12);

int rtlmq_get_regs_len(struct net_device *ndev);
void rtlmq_get_regs(struct net_device *ndev, struct ethtool_regs *regs,
		    void *data);

#endif /* __RTLMQ_DIAG_H__ */
