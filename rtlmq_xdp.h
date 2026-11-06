/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef __RTLMQ_XDP_H__
#define __RTLMQ_XDP_H__

#include <linux/bitops.h>
#include <net/xdp.h>
#include <net/xdp_sock_drv.h>

#include "rtlmq_base.h"

struct rtlmq_xdp_metadata_snapshot {
	u64 rx_timestamp;
	u32 rss_hash;
	enum xdp_rss_hash_type rss_type;
	u16 vlan_tci;
	bool timestamp_valid;
	bool rss_valid;
	bool vlan_valid;
};

/* Keep xdp first; on XSK buffers, metadata occupies xdp_buff_xsk::cb. */
struct rtlmq_xdp_buff {
	struct xdp_buff xdp;
	struct rtlmq_xdp_metadata_snapshot metadata;
};

enum rtlmq_xdp_metadata_capability {
	RTLMQ_XDP_METADATA_HASH = BIT(0),
	RTLMQ_XDP_METADATA_VLAN = BIT(1),
	RTLMQ_XDP_METADATA_TIMESTAMP = BIT(2),
};

enum rtlmq_xsk_metadata_capability {
	RTLMQ_XSK_METADATA_CHECKSUM = BIT(0),
	RTLMQ_XSK_METADATA_TIMESTAMP = BIT(1),
};

struct net_device;
struct netdev_bpf;
struct rtlmq_private;
struct rtlmq_rx_geometry;
struct rtlmq_rx_ring;
struct rtlmq_runtime_config;
struct rtlmq_runtime_layout_state;
struct xdp_frame;

/*
 * XDP/AF_XDP datapath: native XDP transmit,
 * zero-copy socket TX polling/completion, and need-wakeup coordination.
 */
xdp_features_t rtlmq_xdp_feature_mask(const struct rtlmq_private *tp);
u32 rtlmq_xdp_zc_max_segs(const struct rtlmq_private *tp);
int rtlmq_validate_xsk_pool_geometry_locked(const struct rtlmq_private *tp,
					    struct xsk_buff_pool *pool,
					    u16 queue_index, u32 *frame_size);
bool rtlmq_rx_xsk_frame_size_valid_pool_geometry(const struct rtlmq_private *tp,
						 const struct xsk_buff_pool *pool, u32 frame_size,
						 const struct rtlmq_rx_geometry *geometry);
const struct xdp_metadata_ops *
rtlmq_get_xdp_metadata_ops(const struct rtlmq_private *tp);
const struct xsk_tx_metadata_ops *
rtlmq_get_xsk_tx_metadata_ops(const struct rtlmq_private *tp);
const struct xdp_metadata_ops *
rtlmq_xdp_metadata_ops_for_capabilities(u8 capabilities);
const struct xsk_tx_metadata_ops *
rtlmq_xsk_tx_metadata_ops_for_capabilities(u8 capabilities);
int rtlmq_xdp_setup(struct net_device *ndev, struct netdev_bpf *xdp);
int rtlmq_acquire_xsk_pool_dma_locked(struct rtlmq_private *tp,
				      struct xsk_buff_pool *pool);
void rtlmq_release_xsk_pool_dma_locked(struct rtlmq_private *tp,
				       const struct xsk_buff_pool *pool);
void rtlmq_release_all_xsk_pool_dma_locked(struct rtlmq_private *tp);
bool rtlmq_xsk_pool_detached(const struct rtlmq_private *tp,
			     const struct xsk_buff_pool *pool);
struct xsk_buff_pool *rtlmq_requested_xsk_pool(const struct rtlmq_private *tp,
					       u16 queue_index);
bool rtlmq_xsk_tx_queue_count_allowed(const struct rtlmq_private *tp,
				      u16 num_tx_queues);
struct xsk_buff_pool *rtlmq_candidate_xsk_pool(const struct rtlmq_private *tp,
					       u16 queue_index);
u16 rtlmq_candidate_xsk_pool_bindings(const struct rtlmq_private *tp,
				      const struct xsk_buff_pool *pool);
u16 rtlmq_owned_xsk_pool_dma_users(const struct rtlmq_private *tp,
				   const struct xsk_buff_pool *pool);
bool rtlmq_xsk_pool_dma_owned(const struct rtlmq_private *tp,
			      const struct xsk_buff_pool *pool);
u32 rtlmq_xsk_pool_admission_max_segs(const struct rtlmq_private *tp,
				      const struct xsk_buff_pool *pool);
bool rtlmq_rx_ring_xsk_binding_matches(const struct rtlmq_rx_ring *ring,
				       const struct xdp_rxq_info *expected_rxq);
void rtlmq_publish_active_xsk_bindings_locked(struct rtlmq_private *tp,
					      const struct rtlmq_runtime_config *cfg,
					      const struct rtlmq_runtime_layout_state *layout);
void rtlmq_withdraw_active_xsk_bindings_locked(struct rtlmq_private *tp);
void rtlmq_xsk_update_rx_need_wakeup(struct rtlmq_rx_ring *ring,
				     unsigned int requested,
				     unsigned int posted);
void rtlmq_xsk_release_rx_ring_state(struct rtlmq_rx_ring *ring);
u16 rtlmq_active_xsk_queue_count(const struct rtlmq_private *tp);
bool rtlmq_active_xsk_queue_eligible_locked(const struct rtlmq_private *tp,
					    u16 queue_index);
int rtlmq_reconfigure_xsk_pool_locked(struct rtlmq_private *tp, u16 queue_index,
				      struct xsk_buff_pool *new_pool);
int rtlmq_detach_xsk_pool_locked(struct rtlmq_private *tp, u16 queue_index);
bool rtlmq_requested_xsk_pool_bound_elsewhere_locked(const struct rtlmq_private *tp,
						     const struct xsk_buff_pool *pool,
						     u16 queue_index);
int rtlmq_validate_xsk_frame_sizes_locked(struct rtlmq_private *tp,
					  const struct rtlmq_rx_geometry *geometry,
					  u16 *conflict_queue, u32 *conflict_frame_size);
enum rtlmq_tx_publish_result
rtlmq_publish_tx_xdp_frame(struct rtlmq_tx_ring *ring,
			   struct xdp_frame *xdpf,
			   enum rtlmq_tx_buffer_type type,
			   bool dma_map, bool flush_doorbell);
int rtlmq_xdp_xmit(struct net_device *ndev, int n, struct xdp_frame **frames,
		   u32 flags);
enum rtlmq_tx_publish_result
rtlmq_publish_tx_xsk_batch(struct rtlmq_tx_ring *ring,
			   struct xsk_buff_pool *pool,
			   const struct xdp_desc *descs, unsigned int num_descs,
			   bool flush_doorbell);

unsigned int rtlmq_xsk_tx_poll_vector(struct rtlmq_private *tp,
				      struct rtlmq_vector *vector,
				      unsigned int budget);
void rtlmq_xsk_start_tx_locked(struct rtlmq_private *tp);
int rtlmq_xsk_wakeup(struct net_device *ndev, u32 queue_id, u32 flags);

static inline bool rtlmq_xsk_uses_need_wakeup(struct xsk_buff_pool *pool)
{
	return xsk_uses_need_wakeup(pool);
}

static inline void rtlmq_xsk_set_rx_need_wakeup(struct xsk_buff_pool *pool)
{
	xsk_set_rx_need_wakeup(pool);
}

static inline void rtlmq_xsk_set_tx_need_wakeup(struct xsk_buff_pool *pool)
{
	xsk_set_tx_need_wakeup(pool);
}

static inline void rtlmq_xsk_clear_rx_need_wakeup(struct xsk_buff_pool *pool)
{
	xsk_clear_rx_need_wakeup(pool);
}

static inline void rtlmq_xsk_clear_tx_need_wakeup(struct xsk_buff_pool *pool)
{
	xsk_clear_tx_need_wakeup(pool);
}

void rtlmq_xsk_cancel_tx_batch(struct rtlmq_tx_ring *ring);
void rtlmq_xsk_tx_completed(struct xsk_buff_pool *pool, u32 count);

static inline bool rtlmq_xsk_rx_needs_wakeup(unsigned int requested,
					     unsigned int posted)
{
	return requested && posted < requested;
}

static inline bool rtlmq_xsk_tx_needs_wakeup(unsigned int outstanding)
{
	return !outstanding;
}

#endif /* __RTLMQ_XDP_H__ */
