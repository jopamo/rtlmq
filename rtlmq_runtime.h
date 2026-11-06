/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef __RTLMQ_RUNTIME_H__
#define __RTLMQ_RUNTIME_H__

#include <linux/dma-mapping.h>
#include <linux/netdevice.h>

#include "rtlmq_base.h"

struct ethtool_channels;

/*
 * Runtime types shared by device, vector, and control code. Candidate-state
 * accessors are in rtlmq_runtime_accessors.h, not this header.
 */
struct rtlmq_tx_completion;
struct rtlmq_tx_desc;
struct rtlmq_tx_info;
struct rtlmq_rx_ring;
struct rtlmq_chip_profile;
struct rtlmq_topology;
struct xsk_tx_metadata_ops;
struct xsk_dma_ref;

struct rtlmq_runtime_config {
	u16 num_rx_queues;
	u16 num_tx_queues;
	/*
	 * Ring geometry follows the same requested -> candidate -> active
	 * lifecycle as queue shape. It is uniform across queues in one runtime
	 * generation, matching the standard ethtool ringparam contract.
	 */
	u16 rx_ring_count;
	u16 tx_ring_count;
	/*
	 * num_*, rss_enabled, and routing_mode describe the selected runtime.
	 * requested_num_* records the original queue request and may remain
	 * larger than num_* after IRQ fallback.
	 */
	u16 requested_num_rx_queues;
	u16 requested_num_tx_queues;
	u16 num_vectors;
	enum rtlmq_irq_mode irq_mode;
	bool rss_enabled;
	enum rtlmq_routing_mode routing_mode;
	/*
	 * Select metadata callbacks during topology planning so packet paths
	 * do not repeat chip-policy checks.
	 */
	const struct xsk_tx_metadata_ops *xsk_tx_metadata_ops;
};

struct page_pool;
struct xsk_buff_pool;

struct rtlmq_irq_source {
	int irq;
	u8 message_id;
	u8 role;
	u8 kind_flags;
	bool requested;
};

struct rtlmq_vector_layout {
	u32 irq_mask;
	u8 num_irq_sources;
	u32 msix_data_mask;
	u32 msix_event_mask;
	struct rtlmq_irq_source irq_sources[RTLMQ_MAX_LAYOUT_IRQ_SOURCES];
	u16 rx_ring_indices[RTLMQ_IMPL_MAX_RX_QUEUES];
	u16 tx_ring_indices[RTLMQ_IMPL_MAX_TX_QUEUES];
	u16 num_rx_rings;
	u16 num_tx_rings;
};

struct rtlmq_runtime_layout_state {
	u16 rx_to_vector[RTLMQ_IMPL_MAX_RX_QUEUES];
	u16 tx_to_vector[RTLMQ_IMPL_MAX_TX_QUEUES];
	struct rtlmq_vector_layout vectors[RTLMQ_IMPL_MAX_NAPI];
	bool valid;
};

/*
 * Keep each configuration with its layout. Requested/candidate configuration
 * may be valid before layout construction finishes; check layout.valid there.
 * For active state, profile.valid covers the committed pair.
 */
struct rtlmq_runtime_profile_state {
	struct rtlmq_runtime_config config;
	struct rtlmq_runtime_layout_state layout;
	bool valid;
};

struct rtlmq_rss {
	bool enabled;
	u32 indir[RTLMQ_RSS_INDIR_SIZE];
	u8 hash_key[RTLMQ_RSS_KEY_SIZE];
	u32 hash_options;
	u16 programmed_queue_count;
};

struct rtlmq_xsk_dma_pool_state {
	struct xsk_buff_pool *pool;
	struct xsk_dma_ref *ref;
	u16 users;
	/* Socket rings are gone; only retained buffer return is permitted. */
	bool detached;
};

struct rtlmq_requested_runtime_snapshot {
	/*
	 * Rollback copy of requested state. Queue counts are already stored in
	 * profile.config.requested_num_*; do not duplicate them here.
	 */
	struct rtlmq_runtime_profile_state profile;
	struct rtlmq_rss rss;
	struct bpf_prog *requested_xdp_prog;
	struct xsk_buff_pool *requested_xsk_pools[RTLMQ_IMPL_MAX_RX_QUEUES];
	u32 mtu;
	u32 irq_coalesce_usecs;
	u32 irq_coalesce_frames;
	u32 tx_irq_coalesce_usecs;
	u32 tx_irq_coalesce_frames;
};

struct rtlmq_live_ring_profile_commit {
	struct rtlmq_runtime_profile_state old_active_profile;
	struct rtlmq_requested_runtime_snapshot old_requested;
	u32 old_resource_generation;
	u32 new_resource_generation;
	bool staged;
};

struct rtlmq_runtime_transaction {
	struct rtlmq_runtime_profile_state profile;
	struct rtlmq_rss rss;
	struct bpf_prog *xdp_prog;
	struct xsk_buff_pool *xsk_pools[RTLMQ_IMPL_MAX_RX_QUEUES];
	/*
	 * Zero during resource setup; after validation, transfer this candidate
	 * generation to active state at commit.
	 * It must never be an alias of active_resource_generation.
	 */
	u32 resource_generation;
	u64 rx_dma_ring_mask;
	u64 tx_dma_ring_mask;
	struct rtlmq_tx_info *tx_infos[RTLMQ_IMPL_MAX_TX_QUEUES];
	struct rtlmq_tx_completion *tx_completions[RTLMQ_IMPL_MAX_TX_QUEUES];
	struct rtlmq_tx_dma_txn *tx_dma_txns[RTLMQ_IMPL_MAX_TX_QUEUES];
	struct rtlmq_tx_desc *tx_descs[RTLMQ_IMPL_MAX_TX_QUEUES];
	dma_addr_t tx_desc_dmas[RTLMQ_IMPL_MAX_TX_QUEUES];
	struct rtlmq_rx_ring *rx_rings[RTLMQ_IMPL_MAX_RX_QUEUES];
	u64 napi_registered_mask;
	u64 irq_requested_mask;
	bool irq_vectors_allocated;
	bool rss_valid;
	bool xdp_staged;
};

/* Runtime transition and publication API. */
struct rtlmq_private;

/*
 * Prepare and commit runtime changes, capture/restore requested snapshots,
 * expose active state to userspace, and format fallback diagnostics.
 * Other subsystems should use these helpers rather than update state directly.
 */
const char *rtlmq_state_name(enum rtlmq_lifecycle_state state);
const char *rtlmq_irq_mode_name(enum rtlmq_irq_mode mode);
void rtlmq_set_requested_queue_counts(struct rtlmq_private *tp,
				      u16 num_rx_queues, u16 num_tx_queues);
void rtlmq_set_requested_ring_counts(struct rtlmq_private *tp,
				     u16 rx_ring_count, u16 tx_ring_count);
struct rtlmq_runtime_config *
rtlmq_candidate_runtime_config_owned_mut(struct rtlmq_private *tp);
void rtlmq_capture_requested_runtime(struct rtlmq_private *tp,
				     struct rtlmq_requested_runtime_snapshot *snapshot);
/* Restore settings without discarding resources after a failed stop. */
void rtlmq_restore_requested_settings(struct rtlmq_private *tp,
				      const struct rtlmq_requested_runtime_snapshot *snapshot);
void rtlmq_restore_requested_runtime(struct rtlmq_private *tp,
				     const struct rtlmq_requested_runtime_snapshot *snapshot);
/*
 * Apply staged requested settings for probe, down-state changes, refresh, or
 * rollback. Prepare and validate a candidate, then commit requested state.
 * Restore the supplied snapshot on failure. If NULL, capture the current
 * request before preparation.
 */
int rtlmq_apply_requested_runtime_transaction(
	struct rtlmq_private *tp,
	const struct rtlmq_requested_runtime_snapshot *rollback_requested,
	bool reset_rss);
int rtlmq_prepare_candidate_runtime_profile(struct rtlmq_private *tp,
					    u16 num_rx_queues,
					    u16 num_tx_queues);
void rtlmq_discard_candidate_xdp(struct rtlmq_private *tp);
int rtlmq_prepare_requested_runtime_profile(struct rtlmq_private *tp);
int rtlmq_validate_candidate_runtime_profile(struct rtlmq_private *tp);
int rtlmq_program_candidate_runtime_state(struct rtlmq_private *tp,
					  const char *rss_where);
int rtlmq_commit_requested_runtime_profile(struct rtlmq_private *tp);
int rtlmq_refresh_active_runtime_from_requested_same_shape(struct rtlmq_private *tp);
int rtlmq_publish_runtime_profile_active(struct rtlmq_private *tp);
int rtlmq_validate_open_runtime_ownership(struct rtlmq_private *tp);
int rtlmq_build_live_ring_candidate_locked(struct rtlmq_private *tp);
int
rtlmq_stage_live_ring_profile_locked(struct rtlmq_private *tp,
				     const struct rtlmq_requested_runtime_snapshot *old_requested,
				     u32 old_resource_generation,
				     struct rtlmq_live_ring_profile_commit *commit);
void
rtlmq_restore_live_ring_profile_unpublished_locked(struct rtlmq_private *tp,
						   struct rtlmq_live_ring_profile_commit *commit);
int rtlmq_refresh_live_ring_bindings_unpublished_locked(struct rtlmq_private *tp);
int rtlmq_publish_live_ring_generation_locked(struct rtlmq_private *tp,
					      u32 resource_generation);
int rtlmq_publish_live_ring_profile_locked(struct rtlmq_private *tp,
					   struct rtlmq_live_ring_profile_commit *commit,
					   bool use_candidate);
int rtlmq_validate_open_scaling_state(struct rtlmq_private *tp,
				      const struct rtlmq_runtime_config *cfg,
				      const struct rtlmq_runtime_layout_state *layout);
int rtlmq_validate_published_real_queue_counts(const struct rtlmq_private *tp,
					       const struct rtlmq_runtime_config *cfg);
void rtlmq_unwind_active_profile(struct rtlmq_private *tp);
void rtlmq_publish_runtime_publication_locked(struct rtlmq_private *tp);
void rtlmq_revoke_runtime_publication_locked(struct rtlmq_private *tp);
int rtlmq_publish_netdev_queue_counts_for_config(struct rtlmq_private *tp,
						 const struct rtlmq_runtime_config *cfg);
int rtlmq_publish_netdev_queue_counts(struct rtlmq_private *tp);
int rtlmq_withdraw_netdev_queue_counts(struct rtlmq_private *tp);

/* Runtime profile policy API. */
/*
 * Validate queue requests and select a topology supported by both chip
 * hardware and driver policy.
 */
struct rtlmq_channel_request {
	u16 rx_count;
	u16 tx_count;
};

/*
 * Validated topology plan. Build candidates from this result, not directly
 * from unchecked queue counts or requested fields.
 */
struct rtlmq_runtime_topology_plan {
	const struct rtlmq_chip_profile *profile;
	const struct rtlmq_topology *topology;
	struct rtlmq_runtime_config config;
};

int rtlmq_normalize_runtime_queue_request(const struct rtlmq_private *tp,
					  u16 rx_count, u16 tx_count,
					  struct rtlmq_channel_request *request);
int rtlmq_normalize_set_channels_request(const struct rtlmq_private *tp,
					 const struct ethtool_channels *channels,
					 struct rtlmq_channel_request *request);
u16 rtlmq_validated_max_rx_queues(const struct rtlmq_private *tp);
u16 rtlmq_advertised_max_rx_queues(const struct rtlmq_private *tp);
u16 rtlmq_validated_max_tx_queues(const struct rtlmq_private *tp);
bool rtlmq_validated_queue_shape_allowed(const struct rtlmq_private *tp,
					 u16 num_rx_queues, u16 num_tx_queues);
bool rtlmq_runtime_config_matches_qualified_topology(const struct rtlmq_private *tp,
						     const struct rtlmq_runtime_config *cfg);
u16 rtlmq_advertised_max_tx_queues(const struct rtlmq_private *tp);
int rtlmq_validate_runtime_config(const struct rtlmq_private *tp,
				  const struct rtlmq_runtime_config *cfg);
int rtlmq_plan_runtime_topology(const struct rtlmq_private *tp,
				u16 num_rx_queues, u16 num_tx_queues,
				struct rtlmq_runtime_topology_plan *plan);
int rtlmq_construct_runtime_profile_state(const struct rtlmq_runtime_topology_plan *plan,
					  struct rtlmq_runtime_profile_state *profile);
int rtlmq_select_candidate_runtime_profile(struct rtlmq_private *tp,
					   u16 num_rx_queues,
					   u16 num_tx_queues);
void rtlmq_clear_candidate_profile(struct rtlmq_private *tp);

/* Runtime queue/vector layout API. */
/*
 * Queue/vector layout:
 * - queue-to-vector routing
 * - vector layout validation and rebuild
 * - layout-oriented runtime summaries
 *
 * Hardware mapping record (not CPU-placement policy):
 * - RXQ q -> layout->rx_to_vector[q] -> that vector's NAPI
 * - TXQ q -> layout->tx_to_vector[q] -> that vector's NAPI
 * - vector->irq is the primary Linux IRQ associated with that NAPI; MSI-X
 *   source entries on the vector resolve their own Linux IRQs to the same
 *   vector callback
 * - single-vector mode maps all queues to vector 0 and uses MSI or shared
 *   INTx fallback; coupled MSI-X assigns queues by hardware message identity.
 *   RX/TX sources sharing a message share a NAPI owner. TX queues use the
 *   extra TX-only context when present, otherwise vector 0.
 */

static inline const struct rtlmq_vector_layout *
rtlmq_runtime_layout_vector(const struct rtlmq_runtime_config *cfg,
			    const struct rtlmq_runtime_layout_state *layout,
			    u16 vector_index)
{
	if (!cfg || !layout || !layout->valid ||
	    vector_index >= cfg->num_vectors)
		return NULL;

	return &layout->vectors[vector_index];
}

static inline u16
rtlmq_runtime_layout_rx_vector_index(const struct rtlmq_runtime_config *cfg,
				     const struct rtlmq_runtime_layout_state *layout,
				     u16 queue_index)
{
	if (!cfg || !layout || !layout->valid ||
	    queue_index >= cfg->num_rx_queues)
		return 0xffff;

	return layout->rx_to_vector[queue_index];
}

static inline u16
rtlmq_runtime_layout_tx_vector_index(const struct rtlmq_runtime_config *cfg,
				     const struct rtlmq_runtime_layout_state *layout,
				     u16 queue_index)
{
	if (!cfg || !layout || !layout->valid ||
	    queue_index >= cfg->num_tx_queues)
		return 0xffff;

	return layout->tx_to_vector[queue_index];
}

int rtlmq_validate_candidate_layout(struct rtlmq_private *tp);
int rtlmq_rebuild_vector_layout(struct rtlmq_private *tp);

void rtlmq_reset_irq_sources(struct rtlmq_irq_source *sources,
			     unsigned int count);

struct rtlmq_cbs_config {
	/* Linux qdisc parameters and the derived idle-credit register value. */
	s32 hicredit;
	s32 locredit;
	s32 idleslope;
	s32 sendslope;
	u32 idle_credit;
	bool enabled;
};

struct rtlmq_tc_config {
	struct rtlmq_cbs_config cbs[RTLMQ_IMPL_MAX_TX_QUEUES];
	bool mqprio_enabled;
	/* One bit per Linux priority: the admitted mapping has only TC0/TC1. */
	u16 priority_tc1;
};

struct rtlmq_tc_state {
	struct rtlmq_tc_config requested;
	struct rtlmq_tc_config active;
	bool active_valid;
	bool link_waiting;
	/* MAC credit contents survive link events, but not MAC state loss. */
	bool registers_valid;
	u32 blocked_epoch;
};

int rtlmq_setup_tc(struct net_device *ndev, enum tc_setup_type type,
		   void *type_data);
int rtlmq_tc_restore_locked(struct rtlmq_private *tp);
void rtlmq_tc_invalidate_active_locked(struct rtlmq_private *tp);
void rtlmq_tc_publish_runtime_locked(struct rtlmq_private *tp);
void rtlmq_tc_sync_link_locked(struct rtlmq_private *tp);
bool rtlmq_tc_tx_queue_count_allowed(const struct rtlmq_private *tp,
				     u16 num_tx_queues);
int rtlmq_cbs_encode_idleslope(s32 idleslope, s32 sendslope, u32 *value);
int rtlmq_tc_sync_queue_mapping(struct rtlmq_private *tp, u16 num_tx_queues);
bool rtlmq_tc_link_recovery_owned(const struct rtlmq_private *tp);
bool rtlmq_tc_cbs_enabled(const struct rtlmq_tc_config *config);

#endif /* __RTLMQ_RUNTIME_H__ */
