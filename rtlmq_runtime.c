// SPDX-License-Identifier: GPL-2.0-only

#include <linux/etherdevice.h>
#include <linux/filter.h>
#include <linux/if_vlan.h>
#include <linux/kernel.h>
#include <linux/rcupdate.h>
#include <linux/rtnetlink.h>
#include <linux/string.h>
#include <net/page_pool/types.h>
#include <net/xdp_sock_drv.h>

#include "rtlmq_diag.h"
#include "rtlmq_hw.h"
#include "rtlmq_runtime_accessors.h"
#include "rtlmq_lifecycle.h"
#include "rtlmq_runtime.h"
#include "rtlmq.h"
#include "rtlmq_irq.h"
#include "rtlmq_ring.h"
#include "rtlmq_rx.h"
#include "rtlmq_tx.h"
#include "rtlmq_xdp.h"

/*
 * Runtime snapshots, candidate preparation, active commit, and publication.
 * Other subsystems build and validate inputs; use these helpers to commit,
 * expose, or restore runtime state.
 */
static int rtlmq_validate_runtime_rss_state(const struct rtlmq_runtime_config *cfg,
					    const struct rtlmq_rss *rss);

static int rtlmq_stage_candidate_xdp(struct rtlmq_private *tp)
{
	struct bpf_prog *requested_prog;

	lockdep_assert_held(&tp->control_lock);

	requested_prog = READ_ONCE(tp->requested_xdp_prog);
	if (tp->transaction.xdp_staged)
		return tp->transaction.xdp_prog == requested_prog ? 0 : -EBUSY;

	if (requested_prog)
		bpf_prog_inc(requested_prog);
	WRITE_ONCE(tp->transaction.xdp_prog, requested_prog);
	tp->transaction.xdp_staged = true;
	return 0;
}

void rtlmq_discard_candidate_xdp(struct rtlmq_private *tp)
{
	struct bpf_prog *candidate_prog;

	if (!tp)
		return;

	lockdep_assert_held(&tp->control_lock);

	candidate_prog = xchg(&tp->transaction.xdp_prog, NULL);
	tp->transaction.xdp_staged = false;
	if (candidate_prog)
		bpf_prog_put(candidate_prog);
}

static void rtlmq_commit_candidate_xdp(struct rtlmq_private *tp)
{
	struct bpf_prog *active_prog = READ_ONCE(tp->xdp_prog);
	struct bpf_prog *candidate_prog = tp->transaction.xdp_prog;

	lockdep_assert_held(&tp->control_lock);
	WARN_ON_ONCE(!tp->transaction.xdp_staged);

	/*
	 * Same-shape recommit keeps the active reference and drops the duplicate
	 * candidate reference. A new open transfers the candidate reference
	 * rather than acquiring another from requested_xdp_prog.
	 */
	if (active_prog) {
		WARN_ON_ONCE(active_prog != candidate_prog);
		if (candidate_prog)
			bpf_prog_put(candidate_prog);
	} else {
		WRITE_ONCE(tp->xdp_prog, candidate_prog);
	}
	tp->transaction.xdp_prog = NULL;
	tp->transaction.xdp_staged = false;
}

static int rtlmq_validate_candidate_commit_state(struct rtlmq_private *tp,
						 bool require_rss)
{
	const struct rtlmq_runtime_config *cfg;
	struct bpf_prog *candidate_prog;
	struct bpf_prog *requested_prog;
	struct bpf_prog *active_prog;
	const struct rtlmq_rss *rss;
	int ret;

	if (!tp || !tp->transaction.profile.valid)
		return -EINVAL;

	ret = rtlmq_validate_candidate_runtime_profile(tp);
	if (ret)
		return ret;

	/*
	 * Updating requested configuration does not replace ring storage.
	 * Only a resource commit needs the old heap-backed RX state quiesced.
	 */
	if (!require_rss)
		return 0;

	if (tp->transaction.resource_generation &&
	    rtlmq_active_rx_cold_heap_present(tp) &&
	    !rtlmq_retired_generation_release_allowed(tp))
		return -EAGAIN;

	requested_prog = READ_ONCE(tp->requested_xdp_prog);
	candidate_prog = READ_ONCE(tp->transaction.xdp_prog);
	active_prog = READ_ONCE(tp->xdp_prog);
	if (!tp->transaction.xdp_staged ||
	    candidate_prog != requested_prog ||
	    (active_prog && active_prog != candidate_prog))
		return -EINVAL;

	cfg = rtlmq_candidate_runtime_config(tp);
	rss = rtlmq_candidate_rss_state(tp);
	if (!cfg || !rss)
		return -EINVAL;

	return rtlmq_validate_runtime_rss_state(cfg, rss);
}

void rtlmq_set_requested_queue_counts(struct rtlmq_private *tp,
				      u16 num_rx_queues, u16 num_tx_queues)
{
	if (!tp)
		return;

	/*
	 * Store requested queue counts in requested_profile.config. Rollback
	 * snapshots preserve the pre-change values without another device-wide
	 * copy.
	 */
	tp->requested_profile.config.requested_num_rx_queues = num_rx_queues;
	tp->requested_profile.config.requested_num_tx_queues = num_tx_queues;
}

void rtlmq_set_requested_ring_counts(struct rtlmq_private *tp,
				     u16 rx_ring_count, u16 tx_ring_count)
{
	if (!tp)
		return;

	tp->requested_profile.config.rx_ring_count = rx_ring_count;
	tp->requested_profile.config.tx_ring_count = tx_ring_count;
}

struct rtlmq_runtime_config *
rtlmq_candidate_runtime_config_owned_mut(struct rtlmq_private *tp)
{
	if (!tp || !tp->transaction.profile.valid)
		return NULL;

	return &tp->transaction.profile.config;
}

const struct rtlmq_rss *
rtlmq_active_or_requested_rss_state(const struct rtlmq_private *tp)
{
	const struct rtlmq_rss *rss;

	rss = rtlmq_active_rss_state(tp);
	if (rss)
		return rss;

	return rtlmq_requested_rss_state(tp);
}

const struct rtlmq_rss *
rtlmq_published_rss_state(const struct rtlmq_private *tp)
{
	if (!rtlmq_published_runtime_config(tp))
		return NULL;

	return rtlmq_active_rss_state(tp);
}

const struct rtlmq_runtime_layout_state *
rtlmq_active_or_requested_runtime_layout(const struct rtlmq_private *tp)
{
	if (!tp)
		return NULL;

	if (tp->active_profile.valid)
		return &tp->active_profile.layout;

	return rtlmq_requested_runtime_layout(tp);
}

const struct rtlmq_runtime_layout_state *
rtlmq_published_runtime_layout(const struct rtlmq_private *tp)
{
	if (!rtlmq_published_runtime_config(tp))
		return NULL;

	return rtlmq_active_runtime_layout(tp);
}

const struct rtlmq_runtime_config *
rtlmq_active_or_requested_runtime_config(const struct rtlmq_private *tp)
{
	if (!tp)
		return NULL;

	if (tp->active_profile.valid)
		return &tp->active_profile.config;

	return rtlmq_requested_runtime_config(tp);
}

const struct rtlmq_runtime_config *
rtlmq_candidate_or_published_runtime_config(const struct rtlmq_private *tp)
{
	const struct rtlmq_runtime_config *cfg;

	cfg = rtlmq_candidate_runtime_config(tp);
	if (cfg)
		return cfg;

	return rtlmq_published_runtime_config(tp);
}

void rtlmq_capture_requested_runtime(struct rtlmq_private *tp,
				     struct rtlmq_requested_runtime_snapshot *snapshot)
{
	if (!tp || !snapshot)
		return;

	snapshot->profile = tp->requested_profile;
	snapshot->rss = tp->rss;
	snapshot->requested_xdp_prog = READ_ONCE(tp->requested_xdp_prog);
	memcpy(snapshot->requested_xsk_pools, tp->requested_xsk_pools,
	       sizeof(snapshot->requested_xsk_pools));
	snapshot->mtu = tp->ndev ? READ_ONCE(tp->ndev->mtu) : 0;
	snapshot->irq_coalesce_usecs = tp->requested_irq_coalesce_usecs;
	snapshot->irq_coalesce_frames = tp->requested_irq_coalesce_frames;
	snapshot->tx_irq_coalesce_usecs =
		tp->requested_tx_irq_coalesce_usecs;
	snapshot->tx_irq_coalesce_frames =
		tp->requested_tx_irq_coalesce_frames;
}

void rtlmq_restore_requested_settings(struct rtlmq_private *tp,
				      const struct rtlmq_requested_runtime_snapshot *snapshot)
{
	if (!tp || !snapshot)
		return;

	tp->requested_profile = snapshot->profile;
	tp->rss = snapshot->rss;
	WRITE_ONCE(tp->requested_xdp_prog, snapshot->requested_xdp_prog);
	memcpy(tp->requested_xsk_pools, snapshot->requested_xsk_pools,
	       sizeof(tp->requested_xsk_pools));
	if (tp->ndev)
		WRITE_ONCE(tp->ndev->mtu, snapshot->mtu);
	tp->requested_irq_coalesce_usecs = snapshot->irq_coalesce_usecs;
	tp->requested_irq_coalesce_frames = snapshot->irq_coalesce_frames;
	tp->requested_tx_irq_coalesce_usecs = snapshot->tx_irq_coalesce_usecs;
	tp->requested_tx_irq_coalesce_frames = snapshot->tx_irq_coalesce_frames;
}

void rtlmq_restore_requested_runtime(struct rtlmq_private *tp,
				     const struct rtlmq_requested_runtime_snapshot *snapshot)
{
	if (!tp || !snapshot)
		return;

	rtlmq_restore_requested_settings(tp, snapshot);
	rtlmq_clear_candidate_profile(tp);
}

int rtlmq_commit_requested_runtime_profile(struct rtlmq_private *tp)
{
	struct rtlmq_runtime_profile_state requested;
	struct rtlmq_runtime_topology_plan plan;
	int ret;

	if (tp->transaction.profile.valid) {
		ret = rtlmq_validate_candidate_commit_state(tp, false);
		if (ret)
			return ret;

		tp->requested_profile = tp->transaction.profile;
		tp->requested_profile.valid = true;
		rtlmq_clear_candidate_profile(tp);
		return 0;
	}

	ret = rtlmq_plan_runtime_topology(tp,
					  rtlmq_requested_num_rx_queues(tp),
					  rtlmq_requested_num_tx_queues(tp),
					  &plan);
	if (ret)
		return ret;

	ret = rtlmq_construct_runtime_profile_state(&plan, &requested);
	if (ret)
		return ret;
	tp->requested_profile = requested;
	tp->requested_profile.valid = true;
	tp->requested_profile.layout.valid = false;
	rtlmq_clear_candidate_profile(tp);

	return 0;
}

int rtlmq_apply_requested_runtime_transaction(
	struct rtlmq_private *tp,
	const struct rtlmq_requested_runtime_snapshot *rollback_requested,
	bool reset_rss)
{
	const struct rtlmq_requested_runtime_snapshot *rollback = rollback_requested;
	struct rtlmq_requested_runtime_snapshot saved_requested;
	int ret;

	lockdep_assert_held(&tp->control_lock);

	if (!rollback) {
		/*
		 * Callers that changed requested settings must supply the old
		 * snapshot. Otherwise capture it here for prepare/validate rollback.
		 */
		rtlmq_capture_requested_runtime(tp, &saved_requested);
		rollback = &saved_requested;
	}

	ret = rtlmq_prepare_requested_runtime_profile(tp);
	if (ret)
		goto restore_requested;

	ret = rtlmq_commit_requested_runtime_profile(tp);
	if (ret)
		goto restore_requested;

	if (reset_rss)
		rtlmq_rebalance_requested_rss(tp);

	return 0;

restore_requested:
	rtlmq_restore_requested_runtime(tp, rollback);
	return ret;
}

static int
rtlmq_validate_runtime_xsk_queues(const struct rtlmq_private *tp,
				  const struct rtlmq_runtime_config *cfg)
{
	u16 queue_index;

	if (!tp || !tp->ndev || !cfg)
		return -EINVAL;
	lockdep_assert_held(&tp->control_lock);

	for (queue_index = 0; queue_index < cfg->num_rx_queues;
	     queue_index++) {
		struct xsk_buff_pool *pool =
			rtlmq_candidate_xsk_pool(tp, queue_index);
		u32 frame_size;
		int ret;

		if (!pool)
			continue;
		if (rtlmq_candidate_xsk_pool_bindings(tp, pool) != 1)
			return -EBUSY;
		ret = rtlmq_validate_xsk_pool_geometry_locked(tp, pool, queue_index, &frame_size);
		if (ret)
			return ret;
		if (pool->umem && !rtlmq_xsk_pool_dma_owned(tp, pool))
			return -EIO;
	}

	/*
	 * Reject queue-count changes that remove a requested XSK binding,
	 * including while the interface is down.
	 */
	for (queue_index = cfg->num_rx_queues;
	     queue_index < RTLMQ_IMPL_MAX_RX_QUEUES;
	     queue_index++) {
		if (!rtlmq_candidate_xsk_pool(tp, queue_index))
			continue;

		netdev_err(tp->ndev,
			   "runtime profile rxq=%u conflicts with AF_XDP pool on queue %u\n",
			   cfg->num_rx_queues, queue_index);
		return -EBUSY;
	}

	return 0;
}

int rtlmq_validate_candidate_runtime_profile(struct rtlmq_private *tp)
{
	int ret;

	if (!tp || !tp->transaction.profile.valid)
		return -EINVAL;

	ret = rtlmq_validate_runtime_config(tp, &tp->transaction.profile.config);
	if (ret)
		return ret;

	ret = rtlmq_validate_runtime_xsk_queues(tp, &tp->transaction.profile.config);
	if (ret)
		return ret;

	return rtlmq_validate_candidate_layout(tp);
}

static int rtlmq_validate_visible_queue_publication_policy(const struct rtlmq_private *tp,
							   const struct rtlmq_runtime_config *cfg)
{
	if (!tp || !cfg)
		return -EINVAL;

	if (!rtlmq_runtime_config_matches_qualified_topology(tp, cfg))
		return -EOPNOTSUPP;
	if (!rtlmq_chip_routing_mode_admitted(tp, cfg->routing_mode))
		return -EOPNOTSUPP;
	if (cfg->irq_mode != RTLMQ_IRQ_MODE_NONE &&
	    !rtlmq_chip_irq_mode_admitted(tp, cfg->irq_mode))
		return -EOPNOTSUPP;
	if (cfg->rss_enabled && !rtlmq_chip_supports_rss(tp))
		return -EOPNOTSUPP;

	return 0;
}

static int rtlmq_validate_real_queue_publication_scope(const struct rtlmq_private *tp,
						       const struct rtlmq_runtime_config *cfg)
{
	const struct rtlmq_runtime_config *active_cfg;
	enum rtlmq_lifecycle_state state;

	if (!tp || !cfg)
		return -EINVAL;

	active_cfg = rtlmq_active_runtime_config(tp);
	if (!active_cfg) {
		/*
		 * Keep real_num_{rx,tx}_queues at bootstrap 1x1 until active
		 * commit. Requested or candidate counts must not create extra
		 * sysfs queue directories.
		 */
		if (cfg->num_rx_queues == RTLMQ_BOOT_RX_QUEUES &&
		    cfg->num_tx_queues == RTLMQ_BOOT_TX_QUEUES)
			return 0;

		return -EINVAL;
	}

	state = READ_ONCE(tp->state);

	/*
	 * Real queue counts must match active state, not a wider requested or
	 * candidate configuration. During teardown, after leaving OPEN, allow
	 * bootstrap 1x1 counts before the active profile is fully cleared.
	 */
	if (state != RTLMQ_STATE_OPEN &&
	    cfg->num_rx_queues == RTLMQ_BOOT_RX_QUEUES &&
	    cfg->num_tx_queues == RTLMQ_BOOT_TX_QUEUES)
		return 0;

	if (cfg->num_rx_queues != active_cfg->num_rx_queues ||
	    cfg->num_tx_queues != active_cfg->num_tx_queues)
		return -EINVAL;

	return 0;
}

int rtlmq_prepare_candidate_runtime_profile(struct rtlmq_private *tp,
					    u16 num_rx_queues,
					    u16 num_tx_queues)
{
	int ret;

	if (!tp)
		return -EINVAL;

	lockdep_assert_held(&tp->control_lock);

	/* IRQ fallback bypasses the administrative queue-request entry points. */
	if (!rtlmq_tc_tx_queue_count_allowed(tp, num_tx_queues) ||
	    !rtlmq_xsk_tx_queue_count_allowed(tp, num_tx_queues))
		return -EBUSY;

	ret = rtlmq_select_candidate_runtime_profile(tp, num_rx_queues,
						     num_tx_queues);
	if (ret)
		return ret;

	ret = rtlmq_rebuild_vector_layout(tp);
	if (ret)
		goto err_clear_candidate;

	memcpy(tp->transaction.xsk_pools, tp->requested_xsk_pools,
	       sizeof(tp->transaction.xsk_pools));

	ret = rtlmq_stage_candidate_xdp(tp);
	if (ret)
		goto err_clear_candidate;

	ret = rtlmq_validate_candidate_runtime_profile(tp);
	if (ret)
		goto err_clear_candidate;

	return 0;

err_clear_candidate:
	rtlmq_clear_candidate_profile(tp);
	return ret;
}

int rtlmq_prepare_requested_runtime_profile(struct rtlmq_private *tp)
{
	return rtlmq_prepare_candidate_runtime_profile(tp, rtlmq_requested_num_rx_queues(tp),
						       rtlmq_requested_num_tx_queues(tp));
}

int rtlmq_program_candidate_runtime_state(struct rtlmq_private *tp,
					  const char *rss_where)
{
	const struct rtlmq_runtime_config *cfg;
	const char *where;
	int ret;

	if (!tp)
		return -EINVAL;

	lockdep_assert_held(&tp->control_lock);

	cfg = rtlmq_candidate_runtime_config(tp);
	if (!cfg)
		return -EINVAL;

	ret = rtlmq_validate_candidate_runtime_profile(tp);
	if (ret)
		return ret;

	where = rss_where ? rss_where : "runtime_build";

	ret = rtlmq_program_ring_registers(tp);
	if (ret)
		return ret;

	rtlmq_stage_candidate_rss_for_config(tp, cfg);
	ret = rtlmq_program_rss(tp);
	if (ret)
		return ret;

	ret = rtlmq_program_queue_state(tp, cfg);
	if (ret)
		return ret;

	ret = rtlmq_program_tx_engine_runtime(tp, where);
	if (ret)
		return ret;

	return 0;
}

static void rtlmq_copy_candidate_profiles(struct rtlmq_private *tp)
{
	tp->requested_profile = tp->transaction.profile;
	tp->requested_profile.valid = true;
	tp->active_profile = tp->transaction.profile;
	tp->active_profile.valid = true;
}

static void rtlmq_commit_runtime_profile_active(struct rtlmq_private *tp)
{
	rtlmq_publish_rss_active(tp);
	rtlmq_commit_candidate_tx_resources(tp);
	rtlmq_commit_candidate_rx_rings(tp);
	rtlmq_commit_candidate_xdp(tp);
	/*
	 * Validation is complete and these copies cannot fail. Update requested
	 * and active profiles together so requested queue counts keep their
	 * matching configuration and layout after a channel change.
	 */
	rtlmq_copy_candidate_profiles(tp);
	tp->transaction.napi_registered_mask = 0;
	tp->transaction.irq_requested_mask = 0;
	if (tp->transaction.resource_generation) {
		tp->active_resource_generation =
			tp->transaction.resource_generation;
		tp->active_irq_vectors_allocated =
			tp->transaction.irq_vectors_allocated;
		/*
		 * The token now belongs to active state. Clear the transaction's
		 * copy before candidate cleanup.
		 */
		tp->transaction.resource_generation = 0;
	}
	rtlmq_clear_candidate_profile(tp);
}

int rtlmq_publish_runtime_profile_active(struct rtlmq_private *tp)
{
	int ret;

	ret = rtlmq_validate_candidate_commit_state(tp, true);
	if (ret)
		return ret;

	ret = rtlmq_validate_visible_queue_publication_policy(tp, &tp->transaction.profile.config);
	if (ret)
		return ret;

	/*
	 * Validation is complete. Commit only copies state, transfers resources,
	 * and refreshes derived caches; none of these steps may fail.
	 */
	rtlmq_commit_runtime_profile_active(tp);

	return 0;
}

static int
rtlmq_inherit_active_irq_mode(struct rtlmq_private *tp,
			      const struct rtlmq_runtime_config *active)
{
	struct rtlmq_runtime_config *candidate;

	if (!tp || !active)
		return -EINVAL;

	candidate = rtlmq_candidate_runtime_config_owned_mut(tp);
	if (!candidate)
		return -EINVAL;

	/*
	 * Generic preparation leaves IRQ mode unallocated. Same-shape refresh
	 * and ring-size changes reuse IRQ/NAPI, so preserve the active mode
	 * while deriving and checking the other fields independently.
	 */
	candidate->irq_mode = active->irq_mode;

	return rtlmq_validate_candidate_runtime_profile(tp);
}

static bool rtlmq_runtime_shape_matches_for_active_policy_publish(
	const struct rtlmq_runtime_config *active_cfg,
	const struct rtlmq_runtime_config *candidate_cfg)
{
	if (!active_cfg || !candidate_cfg)
		return false;

	return active_cfg->num_rx_queues == candidate_cfg->num_rx_queues &&
	       active_cfg->num_tx_queues == candidate_cfg->num_tx_queues &&
	       active_cfg->rx_ring_count == candidate_cfg->rx_ring_count &&
	       active_cfg->tx_ring_count == candidate_cfg->tx_ring_count &&
	       active_cfg->num_vectors == candidate_cfg->num_vectors &&
	       active_cfg->rss_enabled == candidate_cfg->rss_enabled &&
	       active_cfg->routing_mode == candidate_cfg->routing_mode;
}

static bool rtlmq_runtime_layout_matches_for_active_same_shape_commit(
	const struct rtlmq_runtime_layout_state *active_layout,
	const struct rtlmq_runtime_layout_state *candidate_layout)
{
	if (!active_layout || !candidate_layout || !candidate_layout->valid)
		return false;

	/*
	 * active_profile.valid covers both configuration and layout. Compare
	 * layout contents only; layout.valid checks requested/candidate setup.
	 */
	return memcmp(active_layout, candidate_layout,
		      offsetof(struct rtlmq_runtime_layout_state, valid)) == 0;
}

int rtlmq_refresh_active_runtime_from_requested_same_shape(struct rtlmq_private *tp)
{
	const struct rtlmq_runtime_layout_state *candidate_layout;
	const struct rtlmq_runtime_config *candidate_cfg;
	const struct rtlmq_runtime_config *active_cfg;
	bool was_published;
	int ret;

	lockdep_assert_held(&tp->control_lock);

	if (!tp || READ_ONCE(tp->state) != RTLMQ_STATE_OPEN ||
	    !tp->active_profile.valid || !rtlmq_active_rss_state(tp))
		return -EINVAL;

	/*
	 * Even a same-shape request refreshes active policy through candidate
	 * validation and commit. Do not copy requested fields directly to active.
	 */
	ret = rtlmq_prepare_requested_runtime_profile(tp);
	if (ret)
		return ret;

	candidate_cfg = rtlmq_candidate_runtime_config(tp);
	candidate_layout = rtlmq_candidate_runtime_layout(tp);
	active_cfg = rtlmq_active_runtime_config(tp);
	if (!candidate_cfg || !candidate_layout || !active_cfg) {
		ret = -EINVAL;
		goto err_clear_candidate;
	}

	ret = rtlmq_inherit_active_irq_mode(tp, active_cfg);
	if (ret)
		goto err_clear_candidate;

	ret = rtlmq_validate_visible_queue_publication_policy(tp, candidate_cfg);
	if (ret)
		goto err_clear_candidate;

	if (!rtlmq_runtime_shape_matches_for_active_policy_publish(active_cfg, candidate_cfg) ||
	    !rtlmq_runtime_layout_matches_for_active_same_shape_commit(
		    rtlmq_active_runtime_layout(tp), candidate_layout)) {
		ret = -EINVAL;
		goto err_clear_candidate;
	}

	rtlmq_stage_candidate_rss_for_config(tp, candidate_cfg);
	was_published = READ_ONCE(tp->runtime_published);
	if (was_published) {
		/*
		 * The active profile is embedded in tp. Stop publishing the old
		 * generation before replacing it, then wait for lockless qstats
		 * readers to leave the old read-side snapshot.
		 */
		rtlmq_revoke_runtime_publication_locked(tp);
		synchronize_rcu();
	}
	ret = rtlmq_publish_runtime_profile_active(tp);
	if (ret) {
		if (was_published)
			rtlmq_publish_runtime_publication_locked(tp);
		goto err_clear_candidate;
	}
	if (was_published)
		rtlmq_publish_runtime_publication_locked(tp);

	return 0;

err_clear_candidate:
	rtlmq_clear_candidate_rss(tp);
	rtlmq_clear_candidate_profile(tp);
	return ret;
}

static int rtlmq_validate_candidate_ring_storage(struct rtlmq_private *tp,
						 const struct rtlmq_runtime_config *cfg,
						 const struct rtlmq_runtime_layout_state *layout)
{
	u64 expected_rx_mask;
	u64 expected_tx_mask;
	unsigned int j;
	unsigned int i;
	int ret;

	ret = rtlmq_validate_candidate_tx_storage(tp, cfg->num_tx_queues);
	if (ret)
		return ret;
	for (i = 0; i < cfg->num_rx_queues; i++) {
		const struct rtlmq_rx_ring *ring =
			tp->transaction.rx_rings[i];
		u16 vector_index;

		if (!ring || !ring->desc || !ring->buffers || !ring->cold ||
		    (!ring->page_pool && !ring->cold->xsk_pool))
			return -EINVAL;

		/*
		 * The candidate pool has a netdev/queue ID but no NAPI consumer.
		 * Commit assigns the validated vector; rollback need not stop
		 * the active consumer to discard this private pool.
		 */
		vector_index = layout->rx_to_vector[i];
		if (!tp->vectors || vector_index >= cfg->num_vectors)
			return -EINVAL;
		if (ring->page_pool &&
		    (ring->page_pool->slow.netdev != tp->ndev ||
		     ring->page_pool->slow.queue_idx != i ||
		     ring->page_pool->p.napi))
			return -EINVAL;
		if (!rtlmq_rx_ring_xdp_mem_model_matches(ring, tp->ndev, i))
			return -EINVAL;
		if (ring->cold->xsk_pool &&
		    !rtlmq_rx_ring_xsk_binding_matches(ring, NULL))
			return -EINVAL;
		if (ring->cold->xsk_pool != rtlmq_candidate_xsk_pool(tp, i))
			return -EINVAL;
		if (!ring->cold->xsk_pool)
			continue;
		for (j = 0; j < i; j++) {
			const struct rtlmq_rx_ring *prior =
				tp->transaction.rx_rings[j];

			if (prior && prior->cold &&
			    prior->cold->xsk_pool == ring->cold->xsk_pool)
				return -EINVAL;
		}
	}

	expected_rx_mask = GENMASK_ULL(cfg->num_rx_queues - 1, 0);
	expected_tx_mask = GENMASK_ULL(cfg->num_tx_queues - 1, 0);
	if (tp->transaction.rx_dma_ring_mask != expected_rx_mask ||
	    tp->transaction.tx_dma_ring_mask != expected_tx_mask)
		return -EINVAL;

	return 0;
}

static int rtlmq_assign_candidate_resource_generation(struct rtlmq_private *tp)
{
	u32 generation;

	if (tp->transaction.resource_generation)
		return 0;

	generation = tp->runtime_generation_seq + 1;
	if (!generation)
		generation = 1;
	tp->runtime_generation_seq = generation;
	tp->transaction.resource_generation = generation;

	return 0;
}

int rtlmq_validate_open_runtime_ownership(struct rtlmq_private *tp)
{
	const struct rtlmq_runtime_layout_state *layout;
	const struct rtlmq_runtime_config *cfg;
	u64 expected_napi_mask;
	u64 expected_irq_mask;
	int ret;

	ASSERT_RTNL();

	if (!tp || !tp->ndev)
		return -EINVAL;

	layout = rtlmq_candidate_runtime_layout(tp);
	if (!layout)
		return -EINVAL;
	cfg = rtlmq_candidate_runtime_config(tp);
	if (!cfg)
		return -EINVAL;

	/*
	 * Validate concrete candidate NAPI/IRQ resources without publishing
	 * queue->NAPI, ring->vector, or vector->ring mappings. Those mappings
	 * move from active layout only during the final open commit.
	 */
	ret = rtlmq_validate_candidate_runtime_profile(tp);
	if (ret)
		return ret;

	if (!rtlmq_candidate_resource_owner_valid(tp))
		return -EBUSY;

	ret = rtlmq_validate_runtime_queue_ownership(tp, cfg, layout);
	if (ret)
		return ret;
	ret = rtlmq_validate_napi_enable_state(tp, cfg, layout);
	if (ret)
		return ret;
	ret = rtlmq_validate_candidate_ring_storage(tp, cfg, layout);
	if (ret)
		return ret;

	expected_napi_mask = GENMASK_ULL(cfg->num_vectors - 1, 0);
	expected_irq_mask = GENMASK_ULL(cfg->num_vectors - 1, 0);
	if (tp->transaction.napi_registered_mask != expected_napi_mask ||
	    tp->transaction.irq_requested_mask != expected_irq_mask)
		return -EINVAL;

	return rtlmq_assign_candidate_resource_generation(tp);
}

static bool
rtlmq_live_ring_candidate_service_shape_matches(const struct rtlmq_runtime_config *active,
						const struct rtlmq_runtime_config *candidate)
{
	if (!active || !candidate)
		return false;

	/*
	 * Ringparam changes ring sizes only. Require identical queue counts and
	 * policy so the update can reuse active IRQ/NAPI/vector/RSS state.
	 */
	return active->num_rx_queues == candidate->num_rx_queues &&
	       active->num_tx_queues == candidate->num_tx_queues &&
	       active->requested_num_rx_queues ==
		       candidate->requested_num_rx_queues &&
	       active->requested_num_tx_queues ==
		       candidate->requested_num_tx_queues &&
	       active->num_vectors == candidate->num_vectors &&
	       active->irq_mode == candidate->irq_mode &&
	       active->rss_enabled == candidate->rss_enabled &&
	       active->routing_mode == candidate->routing_mode;
}

static bool
rtlmq_live_ring_candidate_bindings_match(const struct rtlmq_private *tp,
					 const struct rtlmq_runtime_config *cfg)
{
	unsigned int i;

	if (!tp || !cfg || READ_ONCE(tp->requested_xdp_prog) !=
			    READ_ONCE(tp->xdp_prog))
		return false;

	for (i = 0; i < cfg->num_rx_queues; i++) {
		const struct rtlmq_rx_ring *active;

		if (!tp->rx_rings || i >= rtlmq_owned_rx_ring_slots(tp))
			return false;
		active = &tp->rx_rings[i];
		if (!active->cold ||
		    rtlmq_candidate_xsk_pool(tp, i) != active->cold->xsk_pool)
			return false;
	}

	return true;
}

int rtlmq_build_live_ring_candidate_locked(struct rtlmq_private *tp)
{
	const struct rtlmq_runtime_layout_state *candidate_layout;
	const struct rtlmq_runtime_layout_state *active_layout;
	const struct rtlmq_runtime_config *candidate_cfg;
	const struct rtlmq_runtime_config *active_cfg;
	int ret;

	if (!tp || !tp->ndev)
		return -EINVAL;
	lockdep_assert_held(&tp->control_lock);
	ASSERT_RTNL();

	if (READ_ONCE(tp->state) != RTLMQ_STATE_OPEN ||
	    !netif_running(tp->ndev) ||
	    !READ_ONCE(tp->runtime_published) ||
	    !tp->active_resource_generation)
		return -EBUSY;
	/* A live XSK pool is not private candidate storage; quiesce it first. */
	if (rtlmq_active_xsk_queue_count(tp))
		return -EBUSY;
	if (tp->transaction.profile.valid ||
	    tp->transaction.resource_generation ||
	    tp->transaction.rx_dma_ring_mask ||
	    tp->transaction.tx_dma_ring_mask ||
	    tp->transaction.xdp_staged)
		return -EBUSY;

	active_cfg = rtlmq_active_runtime_config(tp);
	active_layout = rtlmq_active_runtime_layout(tp);
	if (!active_cfg || !active_layout)
		return -EINVAL;

	ret = rtlmq_prepare_requested_runtime_profile(tp);
	if (ret)
		return ret;
	ret = rtlmq_inherit_active_irq_mode(tp, active_cfg);
	if (ret)
		goto err_clear_candidate;

	candidate_cfg = rtlmq_candidate_runtime_config(tp);
	candidate_layout = rtlmq_candidate_runtime_layout(tp);
	if (!rtlmq_live_ring_candidate_service_shape_matches(active_cfg, candidate_cfg)) {
		netdev_err(tp->ndev,
			   "live ring candidate changed service shape active_irq=%u candidate_irq=%u\n",
			   active_cfg->irq_mode,
			   candidate_cfg ? candidate_cfg->irq_mode :
					   RTLMQ_IRQ_MODE_NONE);
		ret = -EINVAL;
		goto err_clear_candidate;
	}
	if (!candidate_layout ||
	    memcmp(active_layout, candidate_layout, sizeof(*active_layout))) {
		netdev_err(tp->ndev,
			   "live ring candidate changed active vector layout\n");
		ret = -EINVAL;
		goto err_clear_candidate;
	}
	if (!rtlmq_live_ring_candidate_bindings_match(tp, candidate_cfg)) {
		netdev_err(tp->ndev,
			   "live ring candidate changed XDP/XSK bindings\n");
		ret = -EINVAL;
		goto err_clear_candidate;
	}

	ret = rtlmq_init_rings(tp);
	if (ret) {
		netdev_err(tp->ndev,
			   "live ring candidate allocation failed: %d\n", ret);
		goto err_clear_candidate;
	}

	ret = rtlmq_validate_candidate_runtime_profile(tp);
	if (ret) {
		netdev_err(tp->ndev,
			   "live ring candidate profile validation failed: %d\n",
			   ret);
		goto err_clear_candidate;
	}
	if (!rtlmq_candidate_resource_owner_valid(tp)) {
		netdev_err(tp->ndev,
			   "live ring candidate resource owner is ambiguous\n");
		ret = -EBUSY;
		goto err_clear_candidate;
	}
	ret = rtlmq_validate_candidate_ring_storage(tp, candidate_cfg, candidate_layout);
	if (ret) {
		netdev_err(tp->ndev,
			   "live ring candidate storage validation failed: %d\n",
			   ret);
		goto err_clear_candidate;
	}
	if (tp->transaction.napi_registered_mask ||
	    tp->transaction.irq_requested_mask ||
	    tp->transaction.irq_vectors_allocated) {
		ret = -EINVAL;
		goto err_clear_candidate;
	}

	ret = rtlmq_assign_candidate_resource_generation(tp);
	if (ret)
		goto err_clear_candidate;

	return 0;

err_clear_candidate:
	rtlmq_clear_candidate_profile(tp);
	return ret;
}

static void rtlmq_refresh_live_ring_bindings_locked(struct rtlmq_private *tp)
{
	const struct rtlmq_runtime_layout_state *layout;
	const struct rtlmq_runtime_config *cfg;

	cfg = rtlmq_active_runtime_config(tp);
	layout = rtlmq_active_runtime_layout(tp);
	rtlmq_publish_active_xsk_bindings_locked(tp, cfg, layout);
}

static bool rtlmq_live_ring_change_blocked(const struct rtlmq_private *tp)
{
	return READ_ONCE(tp->runtime_published) ||
	       READ_ONCE(tp->active_resource_generation) ||
	       !rtlmq_retired_generation_release_allowed(tp);
}

int rtlmq_refresh_live_ring_bindings_unpublished_locked(struct rtlmq_private *tp)
{
	if (!tp)
		return -EINVAL;
	lockdep_assert_held(&tp->control_lock);
	if (rtlmq_live_ring_change_blocked(tp))
		return -EBUSY;

	rtlmq_refresh_live_ring_bindings_locked(tp);
	return 0;
}

int rtlmq_publish_live_ring_generation_locked(struct rtlmq_private *tp,
					      u32 resource_generation)
{
	if (!tp || !resource_generation)
		return -EINVAL;
	lockdep_assert_held(&tp->control_lock);
	if (rtlmq_live_ring_change_blocked(tp))
		return -EBUSY;

	tp->active_resource_generation = resource_generation;
	tp->retired_generation_quiesced = false;
	rtlmq_publish_runtime_publication_locked(tp);
	return 0;
}

int
rtlmq_stage_live_ring_profile_locked(struct rtlmq_private *tp,
				     const struct rtlmq_requested_runtime_snapshot *old_requested,
				     u32 old_resource_generation,
				     struct rtlmq_live_ring_profile_commit *commit)
{
	if (!tp || !old_requested || !old_resource_generation ||
	    !commit || commit->staged)
		return -EINVAL;
	lockdep_assert_held(&tp->control_lock);
	if (rtlmq_live_ring_change_blocked(tp) ||
	    !tp->transaction.profile.valid ||
	    !tp->transaction.resource_generation ||
	    tp->transaction.rx_dma_ring_mask ||
	    tp->transaction.tx_dma_ring_mask)
		return -EBUSY;

	commit->old_active_profile = tp->active_profile;
	commit->old_requested = *old_requested;
	commit->old_resource_generation = old_resource_generation;
	commit->new_resource_generation =
		tp->transaction.resource_generation;
	rtlmq_copy_candidate_profiles(tp);
	tp->transaction.resource_generation = 0;
	rtlmq_clear_candidate_profile(tp);
	commit->staged = true;
	rtlmq_refresh_live_ring_bindings_locked(tp);

	return 0;
}

void
rtlmq_restore_live_ring_profile_unpublished_locked(struct rtlmq_private *tp,
						   struct rtlmq_live_ring_profile_commit *commit)
{
	if (!tp || !commit || !commit->staged)
		return;
	lockdep_assert_held(&tp->control_lock);
	if (WARN_ON_ONCE(READ_ONCE(tp->runtime_published) ||
			 READ_ONCE(tp->active_resource_generation)))
		return;

	tp->active_profile = commit->old_active_profile;
	rtlmq_restore_requested_runtime(tp, &commit->old_requested);
	rtlmq_refresh_live_ring_bindings_locked(tp);
}

int rtlmq_publish_live_ring_profile_locked(struct rtlmq_private *tp,
					   struct rtlmq_live_ring_profile_commit *commit,
					   bool use_candidate)
{
	u32 generation;
	int ret;

	if (!tp || !commit || !commit->staged)
		return -EINVAL;
	lockdep_assert_held(&tp->control_lock);
	if (WARN_ON_ONCE(READ_ONCE(tp->runtime_published) ||
			 READ_ONCE(tp->active_resource_generation)))
		return -EBUSY;

	generation = use_candidate ? commit->new_resource_generation :
		     commit->old_resource_generation;
	if (WARN_ON_ONCE(!generation))
		return -EINVAL;
	ret = rtlmq_publish_live_ring_generation_locked(tp, generation);
	if (ret)
		return ret;
	commit->staged = false;
	return 0;
}

static int
rtlmq_validate_runtime_rss_state(const struct rtlmq_runtime_config *cfg,
				 const struct rtlmq_rss *rss)
{
	unsigned int i;

	if (!cfg || !rss)
		return -EINVAL;
	if (rss->programmed_queue_count != cfg->num_rx_queues)
		return -EINVAL;

	for (i = 0; i < RTLMQ_RSS_INDIR_SIZE; i++) {
		if (rss->indir[i] < cfg->num_rx_queues)
			continue;

		return -EINVAL;
	}

	return 0;
}

static int rtlmq_validate_open_xdp_xsk_state(const struct rtlmq_private *tp,
					     const struct rtlmq_runtime_config *cfg,
					     const struct rtlmq_runtime_layout_state *layout,
					     bool wants_xdp)
{
	struct rtlmq_rx_geometry geometry;
	unsigned int j;
	unsigned int i;
	u32 mtu;

	if (!tp || !cfg || !layout || !layout->valid || !tp->rx_rings ||
	    !tp->ndev)
		return -EINVAL;

	mtu = READ_ONCE(tp->ndev->mtu);
	if (rtlmq_rx_geometry_normalize(mtu, &geometry))
		return -EINVAL;

	for (i = 0; i < cfg->num_rx_queues; i++) {
		const struct rtlmq_rx_ring *ring = &tp->rx_rings[i];
		const struct rtlmq_rx_ring_cold *cold = ring->cold;
		const struct rtlmq_vector *vector;
		bool queue_has_xsk;
		u16 vector_index;

		if (!cold)
			return -EINVAL;
		if (!ring->page_pool && !cold->xsk_pool)
			return -EINVAL;
		vector_index = layout->rx_to_vector[i];
		if (!tp->vectors || vector_index >= cfg->num_vectors)
			return -EINVAL;
		vector = &tp->vectors[vector_index];
		if (!vector->napi_added ||
		    ring->vector_index != vector_index ||
		    ring->napi != &vector->napi)
			return -EINVAL;
		if (!xdp_rxq_info_is_reg((struct xdp_rxq_info *)&cold->xdp_rxq) ||
		    cold->xdp_rxq.dev != tp->ndev ||
		    cold->xdp_rxq.queue_index != i)
			return -EINVAL;
		if (!rtlmq_rx_ring_xdp_mem_model_matches(ring, tp->ndev, i))
			return -EINVAL;
		queue_has_xsk = !!cold->xsk_pool;
		if (queue_has_xsk &&
		    !rtlmq_rx_ring_xsk_binding_matches(ring,
						       READ_ONCE(tp->state) == RTLMQ_STATE_OPEN ?
						       &cold->xdp_rxq : NULL))
			return -EINVAL;
		if (queue_has_xsk) {
			if (!rtlmq_rx_xsk_frame_size_valid_pool_geometry(tp, cold->xsk_pool,
									 ring->buffer_len,
									 &geometry))
				return -EINVAL;
		} else if (wants_xdp &&
			   !rtlmq_rx_sg_frame_size_valid_geometry(ring->buffer_len,
								  RTLMQ_XDP_MAX_SEGS, &geometry)) {
			return -EINVAL;
		}
		if (!queue_has_xsk &&
		    (ring->page_pool->p.napi != &vector->napi ||
		     ring->page_pool->slow.netdev != tp->ndev ||
		     ring->page_pool->slow.queue_idx != i))
			return -EINVAL;
		if (!queue_has_xsk)
			continue;
		if (cold->xsk_pool != rtlmq_requested_xsk_pool(tp, i))
			return -EINVAL;
		for (j = 0; j < i; j++)
			if (tp->rx_rings[j].cold &&
			    tp->rx_rings[j].cold->xsk_pool == cold->xsk_pool)
				return -EINVAL;
	}

	if (wants_xdp) {
		if (geometry.xdp_headroom < XDP_PACKET_HEADROOM)
			return -EINVAL;
	}

	return 0;
}

static int
rtlmq_validate_open_ring_state(const struct rtlmq_private *tp,
			       const struct rtlmq_runtime_config *cfg,
			       const struct rtlmq_runtime_layout_state *layout)
{
	unsigned int i;

	if (!tp || !cfg || !layout || !layout->valid ||
	    !tp->rx_rings || !tp->tx_rings)
		return -EINVAL;
	if (cfg->num_tx_queues > rtlmq_owned_tx_ring_slots(tp) ||
	    cfg->num_rx_queues > rtlmq_owned_rx_ring_slots(tp))
		return -EINVAL;

	for (i = 0; i < cfg->num_rx_queues; i++) {
		const struct rtlmq_rx_ring *ring = &tp->rx_rings[i];

		if (!ring->desc || !ring->desc_alloc_size || !ring->cold)
			return -EINVAL;
		if (!ring->page_pool && !ring->cold->xsk_pool)
			return -EINVAL;
		if (ring->vector_index != layout->rx_to_vector[i])
			return -EINVAL;
	}

	for (i = 0; i < cfg->num_tx_queues; i++) {
		const struct rtlmq_tx_ring *ring = &tp->tx_rings[i];

		if (!ring->desc || !ring->infos || !ring->completions ||
		    !ring->ndev_queue)
			return -EINVAL;
		if (ring->vector_index != layout->tx_to_vector[i])
			return -EINVAL;
	}

	return 0;
}

static int rtlmq_validate_runtime_scaling_state(struct rtlmq_private *tp,
						const struct rtlmq_runtime_config *cfg,
						const struct rtlmq_runtime_layout_state *layout,
						const struct rtlmq_rss *rss, bool wants_xdp)
{
	int ret;

	if (!cfg || !layout || !layout->valid)
		return -EINVAL;

	ret = rtlmq_validate_runtime_queue_ownership(tp, cfg, layout);
	if (ret)
		return ret;

	ret = rtlmq_validate_open_ring_state(tp, cfg, layout);
	if (ret)
		return ret;

	ret = rtlmq_validate_open_xdp_xsk_state(tp, cfg, layout, wants_xdp);
	if (ret)
		return ret;

	return rtlmq_validate_runtime_rss_state(cfg, rss);
}

int rtlmq_validate_open_scaling_state(struct rtlmq_private *tp,
				      const struct rtlmq_runtime_config *cfg,
				      const struct rtlmq_runtime_layout_state *layout)
{
	return rtlmq_validate_runtime_scaling_state(tp, cfg, layout, rtlmq_active_rss_state(tp),
						    !!READ_ONCE(tp->xdp_prog));
}

int rtlmq_validate_published_real_queue_counts(const struct rtlmq_private *tp,
					       const struct rtlmq_runtime_config *cfg)
{
	if (!tp || !tp->ndev || !cfg)
		return -EINVAL;
	if (tp->ndev->real_num_rx_queues != cfg->num_rx_queues)
		return -EINVAL;
	if (tp->ndev->real_num_tx_queues != cfg->num_tx_queues)
		return -EINVAL;

	return 0;
}

void rtlmq_unwind_active_profile(struct rtlmq_private *tp)
{
	lockdep_assert_held(&tp->control_lock);

	rtlmq_revoke_runtime_publication_locked(tp);
	tp->active_resource_generation = 0;
	tp->active_profile.valid = false;
}

void rtlmq_publish_runtime_publication_locked(struct rtlmq_private *tp)
{
	lockdep_assert_held(&tp->control_lock);

	/* Publish the committed profile and queue bindings to acquire readers. */
	smp_store_release(&tp->runtime_published, true);
}

void rtlmq_revoke_runtime_publication_locked(struct rtlmq_private *tp)
{
	lockdep_assert_held(&tp->control_lock);

	/*
	 * Withdraw admission, not ownership. Callers must still retire host
	 * readers and establish DMA quiescence before releasing runtime storage.
	 */
	smp_store_release(&tp->runtime_published, false);
}

const char *rtlmq_state_name(enum rtlmq_lifecycle_state state)
{
	switch (state) {
	case RTLMQ_STATE_NEW:
		return "NEW";
	case RTLMQ_STATE_PROBED:
		return "PROBED";
	case RTLMQ_STATE_OPENING:
		return "OPENING";
	case RTLMQ_STATE_OPEN:
		return "OPEN";
	case RTLMQ_STATE_RESETTING:
		return "RESETTING";
	case RTLMQ_STATE_CLOSING:
		return "CLOSING";
	case RTLMQ_STATE_REMOVING:
		return "REMOVING";
	case RTLMQ_STATE_DEAD:
		return "DEAD";
	default:
		return "UNKNOWN";
	}
}

const char *rtlmq_irq_mode_name(enum rtlmq_irq_mode mode)
{
	switch (mode) {
	case RTLMQ_IRQ_MODE_NONE:
		return "NONE";
	case RTLMQ_IRQ_MODE_INTX:
		return "INTX";
	case RTLMQ_IRQ_MODE_MSI:
		return "MSI";
	case RTLMQ_IRQ_MODE_MSIX:
		return "MSIX";
	default:
		return "UNKNOWN";
	}
}

int rtlmq_publish_netdev_queue_counts_for_config(struct rtlmq_private *tp,
						 const struct rtlmq_runtime_config *cfg)
{
	u16 num_tx_queues;
	u16 num_rx_queues;
	u16 old_tx_queues;
	int mapping_ret;
	int ret;

	if (!cfg)
		return -EINVAL;

	ret = rtlmq_validate_visible_queue_publication_policy(tp, cfg);
	if (ret)
		return ret;

	ret = rtlmq_validate_real_queue_publication_scope(tp, cfg);
	if (ret)
		return ret;

	num_tx_queues = cfg->num_tx_queues;
	num_rx_queues = cfg->num_rx_queues;
	old_tx_queues = tp->ndev->real_num_tx_queues;

	/*
	 * Update real queue counts through this helper for both probe and channel
	 * changes. Validation above requires active counts while OPEN, preventing
	 * a requested or candidate configuration from exposing extra queues.
	 */
	if (num_tx_queues < old_tx_queues) {
		ret = rtlmq_tc_sync_queue_mapping(tp, num_tx_queues);
		if (ret)
			return ret;
	}
	ret = netif_set_real_num_queues(tp->ndev, num_tx_queues,
					num_rx_queues);
	/* Restore the previous map too if core rejected a withdrawal. */
	if ((!ret && num_tx_queues > old_tx_queues) ||
	    (ret && num_tx_queues < old_tx_queues)) {
		mapping_ret = rtlmq_tc_sync_queue_mapping(tp, tp->ndev->real_num_tx_queues);
		if (!ret)
			ret = mapping_ret;
	}
	if (ret)
		netdev_err(tp->ndev,
			   "failed to publish runtime queue counts rxq=%u txq=%u: %d\n",
			   num_rx_queues, num_tx_queues, ret);

	return ret;
}

int rtlmq_publish_netdev_queue_counts(struct rtlmq_private *tp)
{
	return rtlmq_publish_netdev_queue_counts_for_config(tp, rtlmq_active_runtime_config(tp));
}

int rtlmq_withdraw_netdev_queue_counts(struct rtlmq_private *tp)
{
	struct rtlmq_runtime_topology_plan plan;
	int ret;

	/*
	 * Netdev core does not support zero real queues. Close/probe/reset unwind
	 * uses bootstrap 1x1 until the next open. Build a full bootstrap topology
	 * through the normal planner, not a queue-count-only placeholder.
	 */
	ret = rtlmq_plan_runtime_topology(tp, RTLMQ_BOOT_RX_QUEUES,
					  RTLMQ_BOOT_TX_QUEUES, &plan);
	if (ret)
		return ret;

	return rtlmq_publish_netdev_queue_counts_for_config(tp, &plan.config);
}
