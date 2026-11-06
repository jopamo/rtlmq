// SPDX-License-Identifier: GPL-2.0-only

#include <linux/bitops.h>
#include <linux/etherdevice.h>
#include <linux/if_vlan.h>
#include <linux/rtnetlink.h>
#include <net/xdp_sock_drv.h>

#include "rtlmq_chip.h"
#include "rtlmq_hw.h"
#include "rtlmq_runtime_accessors.h"
#include "rtlmq_lifecycle.h"
#include "rtlmq.h"
#include "rtlmq_runtime.h"
#include "rtlmq_rx.h"
#include "rtlmq_xdp.h"

/*
 * Handle serialized MTU, XDP, and AF_XDP requests here. AF_XDP pool
 * geometry, binding, and DMA lifetime are handled by rtlmq_xsk.c.
 */
struct rtlmq_mtu_request {
	unsigned int mtu;
};

struct rtlmq_xdp_request {
	enum bpf_netdev_command command;
	struct bpf_prog *prog;
	struct xsk_buff_pool *pool;
	u16 queue_id;
};

static struct netlink_ext_ack *
rtlmq_xdp_request_extack(const struct netdev_bpf *xdp)
{
	if (!xdp || xdp->command != XDP_SETUP_PROG)
		return NULL;

	return xdp->extack;
}

u32 rtlmq_xdp_zc_max_segs(const struct rtlmq_private *tp)
{
	if (!rtlmq_chip_supports_af_xdp(tp))
		return 0;

	return RTLMQ_XDP_MAX_SEGS;
}

xdp_features_t rtlmq_xdp_feature_mask(const struct rtlmq_private *tp)
{
	xdp_features_t features;

	if (!rtlmq_chip_supports_xdp(tp))
		return 0;

	features = NETDEV_XDP_ACT_BASIC |
		   NETDEV_XDP_ACT_REDIRECT |
		   NETDEV_XDP_ACT_NDO_XMIT |
		   NETDEV_XDP_ACT_RX_SG |
		   NETDEV_XDP_ACT_NDO_XMIT_SG;
	if (rtlmq_chip_supports_af_xdp(tp))
		features |= NETDEV_XDP_ACT_XSK_ZEROCOPY;

	return features;
}

const struct xdp_metadata_ops *
rtlmq_get_xdp_metadata_ops(const struct rtlmq_private *tp)
{
	u8 capabilities = 0;

	if (!rtlmq_xdp_feature_mask(tp))
		return NULL;
	if (rtlmq_chip_supports_rss(tp))
		capabilities |= RTLMQ_XDP_METADATA_HASH;
	if (rtlmq_qualified_supports_vlan_ctag_rx(tp))
		capabilities |= RTLMQ_XDP_METADATA_VLAN;
	if (rtlmq_qualified_ptp_backend(tp) == RTLMQ_PTP_BACKEND_PHY_V3)
		capabilities |= RTLMQ_XDP_METADATA_TIMESTAMP;

	return rtlmq_xdp_metadata_ops_for_capabilities(capabilities);
}

const struct xsk_tx_metadata_ops *
rtlmq_get_xsk_tx_metadata_ops(const struct rtlmq_private *tp)
{
	netdev_features_t netdev_features;
	u8 capabilities = 0;

	if (!rtlmq_chip_supports_af_xdp(tp))
		return NULL;

	netdev_features = rtlmq_qualified_base_netdev_features(tp);
	if (netdev_features & (NETIF_F_IP_CSUM | NETIF_F_IPV6_CSUM))
		capabilities |= RTLMQ_XSK_METADATA_CHECKSUM;
	if (rtlmq_qualified_ptp_backend(tp) == RTLMQ_PTP_BACKEND_PHY_V2 ||
	    rtlmq_qualified_ptp_backend(tp) == RTLMQ_PTP_BACKEND_PHY_V3)
		capabilities |= RTLMQ_XSK_METADATA_TIMESTAMP;

	return rtlmq_xsk_tx_metadata_ops_for_capabilities(capabilities);
}

static int rtlmq_normalize_xdp_request(struct rtlmq_private *tp,
				       const struct netdev_bpf *xdp,
				       struct rtlmq_xdp_request *request,
				       struct netlink_ext_ack *extack)
{
	if (!xdp || !request)
		return -EINVAL;

	memset(request, 0, sizeof(*request));
	request->command = xdp->command;
	switch (xdp->command) {
	case XDP_SETUP_PROG:
		if (!rtlmq_xdp_feature_mask(tp)) {
			NL_SET_ERR_MSG_MOD(extack,
					   "selected rtlmq chip profile does not qualify XDP");
			return -EOPNOTSUPP;
		}
		request->prog = xdp->prog;
		break;
	case XDP_SETUP_XSK_POOL:
		if (!rtlmq_chip_supports_af_xdp(tp)) {
			NL_SET_ERR_MSG_MOD(extack,
					   "selected rtlmq chip profile does not qualify AF_XDP");
			return -EOPNOTSUPP;
		}
		request->pool = xdp->xsk.pool;
		request->queue_id = xdp->xsk.queue_id;
		break;
	default:
		NL_SET_ERR_MSG_MOD(extack, "rtlmq supports only XDP program and AF_XDP pool setup");
		return -EOPNOTSUPP;
	}

	return 0;
}

static int rtlmq_normalize_mtu_request_locked(struct rtlmq_private *tp,
					      int new_mtu,
					      struct rtlmq_mtu_request *request)
{
	struct rtlmq_rx_geometry geometry;
	u16 conflict_queue = 0xffff;
	u32 conflict_frame_size = 0;
	int ret;

	lockdep_assert_held(&tp->control_lock);

	if (!request)
		return -EINVAL;
	if (rtlmq_qualified_mtu_limit(tp) &&
	    new_mtu > rtlmq_qualified_mtu_limit(tp))
		return -EINVAL;
	ret = rtlmq_rx_geometry_normalize((u32)new_mtu, &geometry);
	if (ret)
		return ret;

	ret = rtlmq_validate_xsk_frame_sizes_locked(tp, &geometry, &conflict_queue,
						    &conflict_frame_size);
	if (ret) {
		netdev_err(tp->ndev,
			   "mtu=%d conflicts with AF_XDP queue %u frame_size=%u required_dma=%u max_frame=%u\n",
			   new_mtu, conflict_queue, conflict_frame_size,
			   geometry.dma_writable_len,
			   rtlmq_max_xsk_frame_len());
		return ret;
	}

	request->mtu = new_mtu;
	return 0;
}

static int rtlmq_validate_mtu_locked(struct rtlmq_private *tp, int new_mtu)
{
	struct rtlmq_mtu_request request;

	return rtlmq_normalize_mtu_request_locked(tp, new_mtu, &request);
}

static int rtlmq_validate_xdp_attach_locked(struct rtlmq_private *tp)
{
	const struct rtlmq_runtime_layout_state *layout;
	const struct rtlmq_runtime_config *cfg;
	bool open;
	int ret;

	lockdep_assert_held(&tp->control_lock);

	if (!tp->rx_rings)
		return -EINVAL;

	ret = rtlmq_validate_mtu_locked(tp, tp->ndev->mtu);
	if (ret)
		return ret;

	open = READ_ONCE(tp->state) == RTLMQ_STATE_OPEN;
	if (open) {
		cfg = rtlmq_active_runtime_config(tp);
		layout = rtlmq_active_runtime_layout(tp);
	} else {
		cfg = rtlmq_active_or_requested_runtime_config(tp);
		layout = rtlmq_active_or_requested_runtime_layout(tp);
	}
	if (!cfg || !layout || !layout->valid || !cfg->num_rx_queues)
		return -EINVAL;

	return open ? rtlmq_validate_open_scaling_state(tp, cfg, layout) : 0;
}

static int
rtlmq_apply_live_admin_rebuild_locked(struct rtlmq_private *tp,
				      const struct rtlmq_requested_runtime_snapshot *saved_requested
)
{
	lockdep_assert_held(&tp->control_lock);

	if (!saved_requested)
		return -EINVAL;
	if (!rtlmq_can_apply_live_admin_rebuild_locked(tp))
		return -EBUSY;

	return rtlmq_apply_live_reconfiguration_locked(tp,
						       BIT(RTLMQ_RESET_REASON_RUNTIME_RECONFIG),
						       saved_requested
	);
}

static int rtlmq_apply_requested_xdp_admin_change_locked(
	struct rtlmq_private *tp,
	bool live_rebuild,
	bool down_change,
	const struct rtlmq_requested_runtime_snapshot *saved_requested,
	bool reset_rss)
{
	lockdep_assert_held(&tp->control_lock);

	if (!saved_requested)
		return -EINVAL;

	/*
	 * XDP and AF_XDP changes must not alter queue configuration. If queue-count
	 * bookkeeping or the requested layout/RSS state is inconsistent, restore
	 * the saved request and fail the change.
	 */
	if (rtlmq_requested_runtime_config(tp) &&
	    rtlmq_requested_runtime_layout(tp)) {
		const struct rtlmq_runtime_layout_state *layout;
		const struct rtlmq_runtime_config *cfg;
		const struct rtlmq_rss *rss;
		u16 requested_rxq;
		u16 requested_txq;

		cfg = rtlmq_requested_runtime_config(tp);
		layout = rtlmq_requested_runtime_layout(tp);
		rss = rtlmq_requested_rss_state(tp);
		requested_rxq = rtlmq_requested_num_rx_queues(tp);
		requested_txq = rtlmq_requested_num_tx_queues(tp);
		if (!cfg || !layout || !layout->valid || !rss ||
		    cfg->num_rx_queues != requested_rxq ||
		    cfg->num_tx_queues != requested_txq ||
		    cfg->requested_num_rx_queues != requested_rxq ||
		    cfg->requested_num_tx_queues != requested_txq ||
		    (rss->programmed_queue_count &&
		     rss->programmed_queue_count != cfg->num_rx_queues)) {
			rtlmq_restore_requested_runtime(tp, saved_requested);
			return -EINVAL;
		}
	}

	if (live_rebuild) {
		if (reset_rss)
			rtlmq_rebalance_requested_rss(tp);

		return rtlmq_apply_live_admin_rebuild_locked(tp, saved_requested);
	}

	if (down_change)
		return rtlmq_apply_requested_runtime_transaction(tp,
								 saved_requested,
								 reset_rss);

	return -EBUSY;
}

static void rtlmq_publish_live_xdp_program_locked(struct rtlmq_private *tp,
						  struct bpf_prog *new_prog)
{
	struct bpf_prog *old_prog;
	u16 num_rx_queues;
	u16 i;

	lockdep_assert_held(&tp->control_lock);

	/*
	 * The caller has checked that pool DMA direction and geometry still
	 * match the new program. Disable the skb-only RX fast path before
	 * attach. On detach, drain old readers before enabling it again.
	 */
	num_rx_queues = rtlmq_published_num_rx_queues(tp);
	if (new_prog)
		for (i = 0; i < num_rx_queues; i++)
			WRITE_ONCE(tp->rx_rings[i].rx_fast_skb_refill, false);

	if (new_prog)
		bpf_prog_inc(new_prog);
	old_prog = xchg(&tp->xdp_prog, new_prog);
	synchronize_net();
	if (old_prog)
		bpf_prog_put(old_prog);

	if (!new_prog)
		for (i = 0; i < num_rx_queues; i++)
			WRITE_ONCE(tp->rx_rings[i].rx_fast_skb_refill,
				   !rtlmq_rx_ring_uses_xsk_pool(&tp->rx_rings[i]) &&
				   rtlmq_rx_ring_uses_single_descriptor(&tp->rx_rings[i]));
}

static void
rtlmq_revoke_xdp_program_for_removal_locked(struct rtlmq_private *tp)
{
	struct bpf_prog *requested_prog;
	struct bpf_prog *active_prog;
	u16 num_rx_queues;
	u16 i;

	lockdep_assert_held(&tp->control_lock);

	/*
	 * During unregister, core calls ndo_bpf to detach the program after
	 * normal admin changes are already blocked. Allow this detach or core
	 * will warn. Stop new execution of the program, wait for existing readers,
	 * then drop the requested and active references separately.
	 */
	num_rx_queues = rtlmq_owned_rx_ring_slots(tp);
	for (i = 0; tp->rx_rings && i < num_rx_queues; i++)
		WRITE_ONCE(tp->rx_rings[i].rx_fast_skb_refill, false);

	requested_prog = xchg(&tp->requested_xdp_prog, NULL);
	active_prog = xchg(&tp->xdp_prog, NULL);
	synchronize_net();
	if (active_prog)
		bpf_prog_put(active_prog);
	if (requested_prog)
		bpf_prog_put(requested_prog);
}

int rtlmq_change_mtu(struct net_device *ndev, int new_mtu)
{
	struct rtlmq_requested_runtime_snapshot saved_requested;
	struct rtlmq_private *tp = netdev_priv(ndev);
	struct rtlmq_mtu_request request;
	bool mtu_committed = false;
	bool live_rebuild;
	int ret;

	mutex_lock(&tp->control_lock);
	ret = rtlmq_admit_control_change_locked(tp, true);
	if (ret)
		goto out_unlock;

	if (new_mtu == ndev->mtu) {
		ret = 0;
		goto out_unlock;
	}

	if (!rtlmq_can_apply_live_admin_rebuild_locked(tp) &&
	    !rtlmq_can_apply_down_admin_change_locked(tp)) {
		ret = -EBUSY;
		goto out_unlock;
	}

	/*
	 * Live MTU changes require full MAC reset and RX reconfiguration.
	 * Do not use the in-place ring-depth transaction here: TX engine state
	 * must be rebuilt for the new geometry. Rollback restores the old MTU
	 * and requested profile.
	 */
	ret = rtlmq_normalize_mtu_request_locked(tp, new_mtu, &request);
	if (ret)
		goto out_unlock;

	live_rebuild = rtlmq_can_apply_live_admin_rebuild_locked(tp);
	if (live_rebuild)
		rtlmq_capture_requested_runtime(tp, &saved_requested);

	WRITE_ONCE(ndev->mtu, request.mtu);
	if (live_rebuild) {
		ret = rtlmq_apply_live_admin_rebuild_locked(tp, &saved_requested);
		if (ret)
			goto out_unlock;
	}

	mtu_committed = true;
	ret = 0;

out_unlock:
	mutex_unlock(&tp->control_lock);
	/*
	 * ndo_fix_features reads ndev->mtu. Update features after dropping
	 * control_lock because ndo_set_features takes the same lock. Leave
	 * wanted_features unchanged so TSO can return at MTU 1500.
	 */
	if (mtu_committed)
		netdev_update_features(ndev);

	return ret;
}

static int rtlmq_setup_xdp_program_locked(struct rtlmq_private *tp,
					  struct bpf_prog *prog,
					  bool live_rebuild, bool down_change,
					  struct netlink_ext_ack *extack)
{
	struct rtlmq_requested_runtime_snapshot saved_requested = {};
	struct bpf_prog *old_prog;
	int ret = 0;

	lockdep_assert_held(&tp->control_lock);

	if (prog) {
		ret = rtlmq_validate_xdp_attach_locked(tp);
		if (ret) {
			NL_SET_ERR_MSG_MOD(extack,
					   "rtlmq XDP attach requires MTU, RX buffer, and AF_XDP frame-size compatibility");
			return ret;
		}
	}

	old_prog = READ_ONCE(tp->requested_xdp_prog);
	if (old_prog == prog)
		return 0;

	if (live_rebuild || down_change)
		rtlmq_capture_requested_runtime(tp, &saved_requested);

	WRITE_ONCE(tp->requested_xdp_prog, prog);
	if (live_rebuild &&
	    !rtlmq_rx_xdp_requires_rebuild(tp, prog))
		rtlmq_publish_live_xdp_program_locked(tp, prog);
	else
		ret = rtlmq_apply_requested_xdp_admin_change_locked(tp,
								    live_rebuild, down_change,
								    &saved_requested,
								    false);
	if (ret) {
		/*
		 * On error, the caller still owns the new program reference.
		 * Restoring the old request leaves nothing to release here.
		 */
		return ret;
	}

	if (old_prog)
		bpf_prog_put(old_prog);
	return 0;
}

static int rtlmq_setup_xsk_pool_locked(struct rtlmq_private *tp,
				       struct xsk_buff_pool *pool, u16 queue_index,
				       bool live_rebuild,
				       struct netlink_ext_ack *extack)
{
	struct net_device *ndev = tp->ndev;
	struct rtlmq_rx_geometry geometry;
	struct xsk_buff_pool *old_pool;
	struct xsk_buff_pool *new_pool;
	struct rtlmq_rx_ring *ring;
	u32 frame_size;
	int ret;

	lockdep_assert_held(&tp->control_lock);

	/*
	 * AF_XDP queue IDs are limited by the qualified profile. Hardware
	 * capabilities may restrict them further, but must never expose
	 * queue IDs outside that profile.
	 */
	if (queue_index >= rtlmq_advertised_max_rx_queues(tp))
		return -EINVAL;

	old_pool = rtlmq_requested_xsk_pool(tp, queue_index);

	if (old_pool == pool)
		return 0;

	if (rtlmq_requested_xsk_pool_bound_elsewhere_locked(tp, pool,
							    queue_index)) {
		ret = -EBUSY;
		NL_SET_ERR_MSG_MOD(extack,
				   "rtlmq AF_XDP pool is already bound to another RX queue");
		return ret;
	}

	if (live_rebuild) {
		if (!rtlmq_active_xsk_queue_eligible_locked(tp, queue_index)) {
			ret = -EINVAL;
			NL_SET_ERR_MSG_MOD(extack,
					   "AF_XDP queue is unavailable in the active RX configuration");
			return ret;
		}
	} else if (queue_index >= rtlmq_requested_num_rx_queues(tp)) {
		return -EINVAL;
	}
	if (!tp->rx_rings ||
	    queue_index >= rtlmq_owned_rx_ring_slots(tp))
		return -EINVAL;
	ring = &tp->rx_rings[queue_index];
	if (!ring->cold)
		return -EINVAL;

	ret = rtlmq_validate_xdp_attach_locked(tp);
	if (ret) {
		NL_SET_ERR_MSG_MOD(extack,
				   "AF_XDP queue is incompatible with the active device geometry");
		return ret;
	}

	new_pool = xsk_get_pool_from_qid(ndev, queue_index);
	if (!new_pool || new_pool != pool)
		return -EINVAL;

	ret = rtlmq_validate_xsk_pool_geometry_locked(tp, new_pool, queue_index,
						      &frame_size);
	if (ret) {
		NL_SET_ERR_MSG_MOD(extack,
				   "rtlmq AF_XDP pool geometry or segment configuration is unsupported");
		return ret;
	}
	ret = rtlmq_rx_geometry_normalize((u32)ndev->mtu, &geometry);
	if (ret)
		return ret;
	if (!rtlmq_rx_xsk_frame_size_valid_pool_geometry(tp, new_pool,
							 frame_size, &geometry)) {
		ret = -EOPNOTSUPP;
		NL_SET_ERR_MSG_MOD(extack,
				   "rtlmq AF_XDP pool cannot cover the current MTU-backed RX frame requirement");
		return ret;
	}

	ret = rtlmq_acquire_xsk_pool_dma_locked(tp, new_pool);
	if (ret)
		return ret;
	if (!rtlmq_xsk_pool_dma_owned(tp, new_pool)) {
		ret = -EIO;
		NL_SET_ERR_MSG_MOD(extack,
				   "rtlmq AF_XDP pool DMA ownership was not established");
		rtlmq_release_xsk_pool_dma_locked(tp, new_pool);
		return ret;
	}

	if (live_rebuild) {
		ret = rtlmq_reconfigure_xsk_pool_locked(tp, queue_index, new_pool);
	} else {
		/*
		 * When the device is down there is no live ring or RSS state to
		 * rebuild. Save the queue-local request for the next open.
		 */
		tp->requested_xsk_pools[queue_index] = new_pool;
		ret = 0;
	}
	if (ret) {
		rtlmq_release_xsk_pool_dma_locked(tp, new_pool);
		return ret;
	}

	if (old_pool && old_pool != new_pool)
		rtlmq_release_xsk_pool_dma_locked(tp, old_pool);
	return 0;
}

int rtlmq_xdp_setup(struct net_device *ndev, struct netdev_bpf *xdp)
{
	struct rtlmq_private *tp = netdev_priv(ndev);
	struct rtlmq_xdp_request request;
	struct netlink_ext_ack *extack;
	bool live_rebuild;
	bool down_change;
	int ret = 0;

	extack = rtlmq_xdp_request_extack(xdp);
	ret = rtlmq_normalize_xdp_request(tp, xdp, &request, extack);
	if (ret)
		return ret;

	mutex_lock(&tp->control_lock);

	if (request.command == XDP_SETUP_XSK_POOL && !request.pool) {
		ret = rtlmq_detach_xsk_pool_locked(tp, request.queue_id);
		goto out_unlock;
	}

	if (request.command == XDP_SETUP_PROG && !request.prog &&
	    (READ_ONCE(tp->state) == RTLMQ_STATE_REMOVING ||
	     READ_ONCE(tp->state) == RTLMQ_STATE_DEAD)) {
		rtlmq_revoke_xdp_program_for_removal_locked(tp);
		goto out_unlock;
	}

	ret = rtlmq_admit_control_change_locked(tp, true);
	if (ret)
		goto out_unlock;

	live_rebuild = rtlmq_can_apply_live_admin_rebuild_locked(tp);
	down_change = rtlmq_can_apply_down_admin_change_locked(tp);

	/*
	 * Keep requested XDP program and AF_XDP pool state under control_lock.
	 * Updating the live XDP program and replacing an XSK pool use separate
	 * paths. Binding an XSK pool must not rebuild the PHY or device.
	 */
	if (!live_rebuild && !down_change) {
		ret = -EBUSY;
		if (request.command == XDP_SETUP_PROG)
			NL_SET_ERR_MSG_MOD(extack,
					   "rtlmq XDP and AF_XDP changes require a stable OPEN or quiesced PROBED state");
		goto out_unlock;
	}

	switch (request.command) {
	case XDP_SETUP_PROG:
		ret = rtlmq_setup_xdp_program_locked(tp, request.prog,
						     live_rebuild, down_change,
						     extack);
		break;
	case XDP_SETUP_XSK_POOL:
		ret = rtlmq_setup_xsk_pool_locked(tp, request.pool,
						  request.queue_id, live_rebuild,
						  extack);
		break;
	default:
		/* rtlmq_normalize_xdp_request() rejects this before the lock. */
		ret = -EOPNOTSUPP;
		break;
	}

out_unlock:
	mutex_unlock(&tp->control_lock);

	return ret;
}
