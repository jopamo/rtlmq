// SPDX-License-Identifier: GPL-2.0-only

#include <linux/math64.h>
#include <linux/phy.h>
#include <linux/rtnetlink.h>
#include <net/netlink.h>
#include <net/pkt_sched.h>

#include "rtlmq_chip.h"
#include "rtlmq.h"
#include "rtlmq_lifecycle.h"
#include "rtlmq_runtime_accessors.h"
#include "rtlmq_regs.h"
#include "rtlmq_runtime.h"
#include "rtlmq_tx.h"

/*
 * RTL8125B's credit scale follows link rate. At 2500 Mb/s this is the
 * calibrated RTASE-derived 32-tick conversion with denominator 10000000.
 * Canonical slopes describe the intended line rate even while PHY is down;
 * the existing link validator prevents activation at a different rate.
 * The other Linux CBS fields remain API state, not hardware credit bounds.
 */
#define RTLMQ_CBS_CLOCK_TICKS		32U
#define RTLMQ_CBS_LINE_RATE_SCALE	4U
#define RTLMQ_CBS_FRACTION_BITS		25U
#define RTLMQ_TC_ALL_QUEUES		GENMASK(RTLMQ_IMPL_MAX_TX_QUEUES - 1, 0)

static_assert(BITS_PER_TYPE(u16) > TC_BITMASK);

int rtlmq_cbs_encode_idleslope(s32 idleslope, s32 sendslope, u32 *value)
{
	u64 line_rate;
	u64 scaled;

	if (!value || idleslope < 0 || sendslope > 0 ||
	    (idleslope && !sendslope))
		return -EINVAL;
	if (!idleslope) {
		*value = 0;
		return 0;
	}

	line_rate = (s64)idleslope - sendslope;
	scaled = (u64)idleslope * RTLMQ_CBS_CLOCK_TICKS;
	scaled <<= RTLMQ_CBS_FRACTION_BITS;

	/* Positive idle and negative send keep the result strictly below 2^28. */
	scaled = div64_u64(scaled, line_rate * RTLMQ_CBS_LINE_RATE_SCALE);
	*value = scaled;
	return 0;
}

static int rtlmq_tc_validate_cbs_rate(const struct rtlmq_private *tp,
				      s32 idleslope, s32 sendslope)
{
	s32 line_rate;
	int speed;

	if (idleslope <= 0 || sendslope >= 0)
		return -EINVAL;
	if (!tp->phydev || !tp->phydev->link)
		return -ENOLINK;

	speed = READ_ONCE(tp->phydev->speed);
	if (speed <= 0 || speed > S32_MAX / 1000)
		return -ENOLINK;
	line_rate = speed * 1000;
	if (idleslope > line_rate || sendslope != idleslope - line_rate)
		return -EINVAL;

	return 0;
}

static u32 rtlmq_tc_credit_register(u16 queue)
{
	return RTLMQ_TX_QUEUE_CREDIT_BASE +
	       queue * RTLMQ_TX_QUEUE_CREDIT_STRIDE;
}

static int rtlmq_tc_validate_config_rate(const struct rtlmq_private *tp,
					 const struct rtlmq_tc_config *config)
{
	u16 queue;
	int ret;

	for (queue = 0; queue < RTLMQ_IMPL_MAX_TX_QUEUES; queue++) {
		const struct rtlmq_cbs_config *cbs = &config->cbs[queue];

		if (!cbs->enabled)
			continue;
		if (!rtlmq_chip_supports_cbs(tp))
			return -EOPNOTSUPP;
		ret = rtlmq_tc_validate_cbs_rate(tp, cbs->idleslope, cbs->sendslope);
		if (ret)
			return ret;
	}
	return 0;
}

static void rtlmq_tc_write_credit(struct rtlmq_private *tp, u16 queue,
				  u32 credit)
{
	writel(credit, tp->mmio_addr + rtlmq_tc_credit_register(queue));
}

static void rtlmq_tc_retire_failed_update_locked(struct rtlmq_private *tp)
{
	int ret;

	lockdep_assert_held(&tp->control_lock);

	rtlmq_tc_invalidate_active_locked(tp);
	if (tp->state != RTLMQ_STATE_OPEN)
		return;

	/* Unknown shaping cannot keep a published runtime claiming old policy. */
	ret = rtlmq_retire_failed_control_update_locked(tp);
	if (ret)
		netdev_err(tp->ndev,
			   "traffic-control rollback shutdown failed: %d\n", ret);
}

static int rtlmq_tc_config_credit(struct rtlmq_tc_config *config, u16 queue)
{
	if (!config->mqprio_enabled || !config->cbs[queue].enabled) {
		config->cbs[queue].idle_credit = 0;
		return 0;
	}

	return rtlmq_cbs_encode_idleslope(config->cbs[queue].idleslope,
					  config->cbs[queue].sendslope,
					  &config->cbs[queue].idle_credit);
}

static int rtlmq_tc_program_config_locked(struct rtlmq_private *tp,
					  struct rtlmq_tc_config *config,
					  u32 queue_mask)
{
	u32 old_credit[RTLMQ_IMPL_MAX_TX_QUEUES];
	bool restored = true;
	u16 queue;
	int ret;

	lockdep_assert_held(&tp->control_lock);

	if (!config || !rtlmq_mmio_ordinary_authorized(tp))
		return -ENODEV;
	/*
	 * Single-TX production does not support mqprio/CBS. Open/reset must still
	 * disable shaping on both hardware slots to clear inherited credit
	 * settings, without failing just because TC is unsupported.
	 */
	if (config->mqprio_enabled &&
	    (!rtlmq_chip_supports_tc(tp) ||
	     rtlmq_candidate_or_published_num_tx_queues(tp) !=
		     RTLMQ_IMPL_MAX_TX_QUEUES))
		return -EOPNOTSUPP;
	if (!rtlmq_chip_supports_cbs(tp))
		for (queue = 0; queue < RTLMQ_IMPL_MAX_TX_QUEUES; queue++)
			if (config->cbs[queue].enabled)
				return -EOPNOTSUPP;
	/*
	 * mqprio classification itself has no family register transaction.
	 * Only profiles that own the RTL8125 credit layout may clear or replay
	 * those registers; RTL8126 keeps CBS unavailable.
	 */
	if (!rtlmq_qualified_supports_cbs(tp))
		return 0;

	for (queue = 0; queue < RTLMQ_IMPL_MAX_TX_QUEUES; queue++) {
		if (!(queue_mask & BIT(queue)))
			continue;
		ret = rtlmq_tc_config_credit(config, queue);
		if (ret)
			return ret;
	}
	for (queue = 0; queue < RTLMQ_IMPL_MAX_TX_QUEUES; queue++) {
		if (!(queue_mask & BIT(queue)))
			continue;
		old_credit[queue] =
			readl(tp->mmio_addr + rtlmq_tc_credit_register(queue));
		/* All-ones readback cannot establish a restorable hardware state. */
		if (old_credit[queue] == U32_MAX) {
			rtlmq_tc_retire_failed_update_locked(tp);
			return -ENODEV;
		}
	}

	for (queue = 0; queue < RTLMQ_IMPL_MAX_TX_QUEUES; queue++) {
		u32 expected = config->cbs[queue].idle_credit;
		u32 observed;

		if (!(queue_mask & BIT(queue)))
			continue;
		rtlmq_tc_write_credit(tp, queue, expected);
		observed =
			readl(tp->mmio_addr + rtlmq_tc_credit_register(queue));
		if (observed == expected)
			continue;

		for (queue = 0; queue < RTLMQ_IMPL_MAX_TX_QUEUES; queue++)
			if (queue_mask & BIT(queue))
				rtlmq_tc_write_credit(tp, queue, old_credit[queue]);
		for (queue = 0; queue < RTLMQ_IMPL_MAX_TX_QUEUES; queue++)
			if ((queue_mask & BIT(queue)) &&
			    readl(tp->mmio_addr +
				  rtlmq_tc_credit_register(queue)) !=
			    old_credit[queue]) {
				restored = false;
				netdev_err(tp->ndev,
					   "failed to restore TX q%u credit after programming error\n",
					   queue);
			}
		if (!restored)
			rtlmq_tc_retire_failed_update_locked(tp);
		return observed == U32_MAX ? -ENODEV : -EIO;
	}

	return 0;
}

static int rtlmq_tc_publish_mqprio(struct net_device *ndev, bool enabled)
{
	int ret;

	if (!enabled) {
		netdev_reset_tc(ndev);
		return 0;
	}

	ret = netdev_set_num_tc(ndev, RTLMQ_IMPL_MAX_TX_QUEUES);
	if (ret)
		return ret;
	ret = netdev_set_tc_queue(ndev, 0, 1, 0);
	if (ret)
		goto rollback;
	ret = netdev_set_tc_queue(ndev, 1, 1, 1);
	if (!ret)
		return 0;

rollback:
	netdev_reset_tc(ndev);
	return ret;
}

int rtlmq_tc_sync_queue_mapping(struct rtlmq_private *tp, u16 num_tx_queues)
{
	const struct rtlmq_tc_config *config = &tp->tc.requested;
	unsigned int priority;
	int ret;

	if (!config->mqprio_enabled)
		return 0;

	ASSERT_RTNL();
	lockdep_assert_held(&tp->control_lock);
	/*
	 * Bootstrap 1x1 is publication withdrawal, not mqprio removal. Core
	 * otherwise rewrites priorities targeting TX1 and cannot restore them.
	 * Keep the user's map in requested state until two queues are published.
	 */
	ret = rtlmq_tc_publish_mqprio(tp->ndev, num_tx_queues == RTLMQ_IMPL_MAX_TX_QUEUES);
	if (ret || num_tx_queues != RTLMQ_IMPL_MAX_TX_QUEUES)
		return ret;

	for (priority = 0; priority <= TC_BITMASK; priority++)
		netdev_set_prio_tc_map(tp->ndev, priority,
				       !!(config->priority_tc1 & BIT(priority)));
	return 0;
}

bool rtlmq_tc_cbs_enabled(const struct rtlmq_tc_config *config)
{
	u16 queue;

	if (!config->mqprio_enabled)
		return false;
	for (queue = 0; queue < RTLMQ_IMPL_MAX_TX_QUEUES; queue++)
		if (config->cbs[queue].enabled)
			return true;
	return false;
}

static bool rtlmq_tc_revokes_config(const struct rtlmq_tc_config *old,
				    const struct rtlmq_tc_config *candidate)
{
	u16 queue;

	if (old->mqprio_enabled && !candidate->mqprio_enabled)
		return true;
	for (queue = 0; queue < RTLMQ_IMPL_MAX_TX_QUEUES; queue++)
		if (old->cbs[queue].enabled && !candidate->cbs[queue].enabled)
			return true;

	return false;
}

static int
rtlmq_tc_rebuild_service_locked(struct rtlmq_private *tp,
				const struct rtlmq_tc_config *candidate)
{
	struct rtlmq_tc_config old = tp->tc.requested;
	bool revokes = rtlmq_tc_revokes_config(&old, candidate);
	struct rtlmq_netpoll_guard guard = {};
	int rollback_ret;
	int ret;

	ASSERT_RTNL();
	lockdep_assert_held(&tp->control_lock);
	/*
	 * Cached publication grants and already-staged GSO packets belong to the
	 * old service profile. Use normal quiesced teardown, not a policy change
	 * underneath outstanding DMA. Reopen builds fresh grants and replays TC.
	 */
	rtlmq_netpoll_guard_acquire_locked(tp, &guard);
	ret = rtlmq_detach_and_close_locked(tp, false);
	if (ret)
		goto failed;

	tp->tc.requested = *candidate;
	ret = rtlmq_open_device(tp);
	if (!ret)
		goto out;

failed:
	/* Core cannot retain a destroyed qdisc; never reopen a revoked policy. */
	tp->tc.requested = revokes ? *candidate : old;
	if (!revokes && tp->state == RTLMQ_STATE_PROBED) {
		rollback_ret = rtlmq_open_device(tp);
		if (rollback_ret)
			netdev_err(tp->ndev,
				   "CBS service rollback reopen failed: %d\n",
				   rollback_ret);
	}
	if (tp->state != RTLMQ_STATE_OPEN) {
		rtlmq_tc_invalidate_active_locked(tp);
		netif_carrier_off(tp->ndev);
		netif_device_detach(tp->ndev);
	}
out:
	rtlmq_netpoll_guard_release_locked(tp, &guard);
	return ret;
}

static int rtlmq_tc_commit_candidate_locked(struct rtlmq_private *tp,
					    struct rtlmq_tc_config *candidate,
					    bool publish_mqprio, u32 queue_mask)
{
	struct rtlmq_tc_config old_requested = tp->tc.requested;
	bool live = tp->state == RTLMQ_STATE_OPEN &&
		    tp->platform_quiesce_owner == RTLMQ_PLATFORM_QUIESCE_NONE;
	int rollback_ret;
	int ret;

	lockdep_assert_held(&tp->control_lock);

	if (live &&
	    rtlmq_tc_cbs_enabled(&old_requested) != rtlmq_tc_cbs_enabled(candidate))
		return rtlmq_tc_rebuild_service_locked(tp, candidate);

	if (live) {
		ret = rtlmq_tc_program_config_locked(tp, candidate, queue_mask);
		if (ret) {
			/*
			 * Qdisc destruction ignores disable errors. Rollback may
			 * preserve a replacement, never a departing configuration.
			 */
			if (rtlmq_tc_revokes_config(&old_requested, candidate)) {
				tp->tc.requested = *candidate;
				if (publish_mqprio)
					rtlmq_tc_publish_mqprio(tp->ndev, false);
				rtlmq_tc_retire_failed_update_locked(tp);
			}
			return ret;
		}
	}

	if (publish_mqprio) {
		ret = rtlmq_tc_publish_mqprio(tp->ndev,
					      candidate->mqprio_enabled);
		if (ret) {
			if (live) {
				rollback_ret = rtlmq_tc_program_config_locked(tp, &old_requested,
									      queue_mask);
				if (rollback_ret) {
					netdev_err(tp->ndev,
						   "failed to restore traffic-control hardware after publication error: %d\n",
						   rollback_ret);
					rtlmq_tc_retire_failed_update_locked(tp);
				}
			}
			return ret;
		}
	}

	tp->tc.requested = *candidate;
	if (live) {
		tp->tc.active = *candidate;
		tp->tc.active_valid = true;
		tp->tc.registers_valid = true;
		/* Corrected CBS intent can restore service without another PHY event. */
		rtlmq_sync_link_state(tp);
	} else {
		tp->tc.active_valid = false;
		tp->tc.registers_valid = false;
	}

	return 0;
}

static int rtlmq_tc_validate_mqprio(struct rtlmq_private *tp,
				    struct tc_mqprio_qopt_offload *mqprio)
{
	int tc;

	if (!mqprio)
		return -EINVAL;
	if (!mqprio->qopt.num_tc)
		return 0;
	if (tp->ndev->real_num_tx_queues != RTLMQ_IMPL_MAX_TX_QUEUES) {
		NL_SET_ERR_MSG_MOD(mqprio->extack,
				   "RTLMQ mqprio requires the validated two-TX-queue runtime");
		return -EOPNOTSUPP;
	}
	if (mqprio->qopt.num_tc != RTLMQ_IMPL_MAX_TX_QUEUES) {
		NL_SET_ERR_MSG_MOD(mqprio->extack,
				   "RTLMQ supports exactly two traffic classes");
		return -EOPNOTSUPP;
	}
	for (tc = 0; tc < RTLMQ_IMPL_MAX_TX_QUEUES; tc++) {
		if (mqprio->qopt.count[tc] != 1 ||
		    mqprio->qopt.offset[tc] != tc) {
			NL_SET_ERR_MSG_MOD(mqprio->extack,
					   "RTLMQ requires one ordered TX queue per traffic class");
			return -EOPNOTSUPP;
		}
	}
	for (tc = 0; tc <= TC_BITMASK; tc++) {
		if (mqprio->qopt.prio_tc_map[tc] >= RTLMQ_IMPL_MAX_TX_QUEUES) {
			NL_SET_ERR_MSG_MOD(mqprio->extack,
					   "RTLMQ priority must map to TC0 or TC1");
			return -EINVAL;
		}
	}
	if ((mqprio->flags & TC_MQPRIO_F_MODE) &&
	    mqprio->mode != TC_MQPRIO_MODE_DCB) {
		NL_SET_ERR_MSG_MOD(mqprio->extack,
				   "RTLMQ supports only mqprio DCB mode");
		return -EOPNOTSUPP;
	}
	if ((mqprio->flags & TC_MQPRIO_F_SHAPER) &&
	    mqprio->shaper != TC_MQPRIO_SHAPER_DCB) {
		NL_SET_ERR_MSG_MOD(mqprio->extack,
				   "Use the CBS qdisc for RTLMQ hardware shaping");
		return -EOPNOTSUPP;
	}
	if (mqprio->flags & (TC_MQPRIO_F_MIN_RATE | TC_MQPRIO_F_MAX_RATE)) {
		NL_SET_ERR_MSG_MOD(mqprio->extack,
				   "RTLMQ mqprio rate attributes are unsupported; use CBS");
		return -EOPNOTSUPP;
	}
	if (mqprio->preemptible_tcs) {
		NL_SET_ERR_MSG_MOD(mqprio->extack,
				   "RTLMQ does not support frame preemption");
		return -EOPNOTSUPP;
	}

	return 0;
}

static int rtlmq_tc_setup_mqprio_locked(struct rtlmq_private *tp,
					struct tc_mqprio_qopt_offload *mqprio)
{
	struct rtlmq_tc_config candidate = tp->tc.requested;
	int ret;

	ret = rtlmq_tc_validate_mqprio(tp, mqprio);
	if (ret)
		return ret;

	candidate.mqprio_enabled = !!mqprio->qopt.num_tc;
	candidate.priority_tc1 = 0;
	if (candidate.mqprio_enabled) {
		unsigned int priority;

		for (priority = 0; priority <= TC_BITMASK; priority++)
			if (mqprio->qopt.prio_tc_map[priority])
				candidate.priority_tc1 |= BIT(priority);
	}
	if (!candidate.mqprio_enabled)
		memset(candidate.cbs, 0, sizeof(candidate.cbs));

	ret = rtlmq_tc_commit_candidate_locked(tp, &candidate, true,
					       RTLMQ_TC_ALL_QUEUES);
	if (!ret)
		mqprio->qopt.hw = candidate.mqprio_enabled ?
			TC_MQPRIO_HW_OFFLOAD_TCS :
			TC_MQPRIO_HW_OFFLOAD_NONE;

	return ret;
}

static int rtlmq_tc_validate_cbs_locked(struct rtlmq_private *tp,
					const struct tc_cbs_qopt_offload *cbs,
					struct rtlmq_cbs_config *normalized)
{
	int ret;

	if (!cbs || !normalized)
		return -EINVAL;
	if (cbs->queue < 0 || cbs->queue >= RTLMQ_IMPL_MAX_TX_QUEUES)
		return -EINVAL;
	if (!cbs->enable) {
		memset(normalized, 0, sizeof(*normalized));
		return 0;
	}
	if (!rtlmq_chip_supports_cbs(tp) ||
	    !tp->tc.requested.mqprio_enabled)
		return -EOPNOTSUPP;
	if (cbs->queue >= tp->ndev->real_num_tx_queues ||
	    netdev_get_num_tc(tp->ndev) != RTLMQ_IMPL_MAX_TX_QUEUES ||
	    tp->ndev->tc_to_txq[cbs->queue].count != 1 ||
	    tp->ndev->tc_to_txq[cbs->queue].offset != cbs->queue)
		return -EINVAL;
	if (cbs->hicredit < 0 || cbs->locredit > 0)
		return -EINVAL;

	ret = rtlmq_tc_validate_cbs_rate(tp, cbs->idleslope, cbs->sendslope);
	if (ret && ret != -ENOLINK)
		return ret;

	normalized->enabled = true;
	normalized->hicredit = cbs->hicredit;
	normalized->locredit = cbs->locredit;
	normalized->idleslope = cbs->idleslope;
	normalized->sendslope = cbs->sendslope;
	return rtlmq_cbs_encode_idleslope(cbs->idleslope, cbs->sendslope,
					  &normalized->idle_credit);
}

static int rtlmq_tc_setup_cbs_locked(struct rtlmq_private *tp,
				     const struct tc_cbs_qopt_offload *cbs)
{
	struct rtlmq_tc_config candidate = tp->tc.requested;
	struct rtlmq_cbs_config normalized = {};
	int ret;

	ret = rtlmq_tc_validate_cbs_locked(tp, cbs, &normalized);
	if (ret)
		return ret;

	candidate.cbs[cbs->queue] = normalized;
	return rtlmq_tc_commit_candidate_locked(tp, &candidate, false,
						 BIT(cbs->queue));
}

static int rtlmq_tc_query_caps(const struct rtlmq_private *tp,
			       struct tc_query_caps_base *base)
{
	if (!rtlmq_chip_supports_tc(tp))
		return -EOPNOTSUPP;
	if (!base || base->type != TC_SETUP_QDISC_MQPRIO || !base->caps)
		return -EOPNOTSUPP;

	((struct tc_mqprio_caps *)base->caps)->validate_queue_counts = true;
	return 0;
}

static bool rtlmq_tc_is_disable(enum tc_setup_type type, const void *data)
{
	if (!data)
		return false;

	switch (type) {
	case TC_SETUP_QDISC_MQPRIO:
		return !((const struct tc_mqprio_qopt_offload *)data)->qopt.num_tc;
	case TC_SETUP_QDISC_CBS:
		return !((const struct tc_cbs_qopt_offload *)data)->enable;
	default:
		return false;
	}
}

int rtlmq_setup_tc(struct net_device *ndev, enum tc_setup_type type,
		   void *type_data)
{
	struct rtlmq_private *tp = netdev_priv(ndev);
	struct rtlmq_tc_config old;
	int ret = 0;

	if (type == TC_QUERY_CAPS)
		return rtlmq_tc_query_caps(tp, type_data);
	if (type != TC_SETUP_QDISC_MQPRIO &&
	    type != TC_SETUP_QDISC_CBS)
		return -EOPNOTSUPP;

	ASSERT_RTNL();
	mutex_lock(&tp->control_lock);
	old = tp->tc.requested;
	/*
	 * Core does not retain a destroyed qdisc on error. Disable still goes
	 * through request validation, but cannot require new-change admission.
	 * A platform owner has already withdrawn publication before dropping
	 * these locks; it retains hardware teardown and replays current intent.
	 */
	if (!rtlmq_tc_is_disable(type, type_data)) {
		ret = rtlmq_admit_control_change_locked(tp, true);
		if (!ret && (!rtlmq_chip_supports_tc(tp) ||
			     rtlmq_requested_num_tx_queues(tp) !=
				     RTLMQ_IMPL_MAX_TX_QUEUES))
			ret = -EOPNOTSUPP;
	}
	if (!ret && type == TC_SETUP_QDISC_MQPRIO)
		ret = rtlmq_tc_setup_mqprio_locked(tp, type_data);
	else if (!ret)
		ret = rtlmq_tc_setup_cbs_locked(tp, type_data);
	if (!ret && tp->state == RTLMQ_STATE_PROBED) {
		ret = rtlmq_sync_features_locked(tp);
		/* A down-device enable still requires feature admission. */
		if (ret && !rtlmq_tc_revokes_config(&old, &tp->tc.requested))
			tp->tc.requested = old;
	}
	mutex_unlock(&tp->control_lock);

	return ret;
}

int rtlmq_tc_restore_locked(struct rtlmq_private *tp)
{
	struct rtlmq_tc_config candidate = tp->tc.requested;
	int rate_ret;
	int ret;

	lockdep_assert_held(&tp->control_lock);

	rtlmq_tc_invalidate_active_locked(tp);
	rate_ret = rtlmq_tc_validate_config_rate(tp, &candidate);
	if (rate_ret && rate_ret != -ENOLINK && rate_ret != -EINVAL)
		return rate_ret;
	ret = rtlmq_tc_program_config_locked(tp, &candidate, RTLMQ_TC_ALL_QUEUES);
	if (ret)
		return ret;

	tp->tc.requested = candidate;
	tp->tc.active = candidate;
	tp->tc.registers_valid = true;
	tp->tc.active_valid = !rate_ret;
	tp->tc.link_waiting = !!rate_ret;
	return 0;
}

void rtlmq_tc_invalidate_active_locked(struct rtlmq_private *tp)
{
	lockdep_assert_held(&tp->control_lock);

	tp->tc.active_valid = false;
	tp->tc.link_waiting = false;
	tp->tc.registers_valid = false;
	tp->tc.blocked_epoch = 0;
}

void rtlmq_tc_publish_runtime_locked(struct rtlmq_private *tp)
{
	lockdep_assert_held(&tp->control_lock);
	ASSERT_RTNL();

	if (tp->tc.link_waiting) {
		tp->tc.blocked_epoch = tp->reset_epoch;
		rtlmq_serial_tx_pause(tp);
		netif_device_detach(tp->ndev);
	} else {
		netif_device_attach(tp->ndev);
	}
}

static bool rtlmq_tc_link_service_allowed(const struct rtlmq_private *tp)
{
	return tp->state == RTLMQ_STATE_OPEN && netif_running(tp->ndev) &&
	       READ_ONCE(tp->runtime_published) &&
	       tp->platform_quiesce_owner == RTLMQ_PLATFORM_QUIESCE_NONE &&
	       rtlmq_mmio_ordinary_authorized(tp) &&
	       !READ_ONCE(tp->reset_pending_reasons);
}

bool rtlmq_tc_link_recovery_owned(const struct rtlmq_private *tp)
{
	lockdep_assert_held(&tp->control_lock);

	return rtlmq_tc_link_service_allowed(tp) && tp->tc.link_waiting &&
	       tp->tc.blocked_epoch == tp->reset_epoch;
}

void rtlmq_tc_sync_link_locked(struct rtlmq_private *tp)
{
	bool owned;
	int ret;

	lockdep_assert_held(&tp->control_lock);

	if (!tp->tc.requested.mqprio_enabled && !tp->tc.link_waiting)
		return;
	if (!rtlmq_tc_link_service_allowed(tp))
		return;
	owned = rtlmq_tc_link_recovery_owned(tp);
	/* Another lifecycle owner may have detached this same netdev. */
	if (!netif_device_present(tp->ndev) && !owned)
		return;
	ASSERT_RTNL();

	ret = rtlmq_tc_validate_config_rate(tp, &tp->tc.requested);
	if (ret) {
		tp->tc.active_valid = false;
		tp->tc.link_waiting = true;
		rtlmq_serial_tx_pause(tp);
		netif_device_detach(tp->ndev);
		netif_carrier_off(tp->ndev);
		tp->tc.blocked_epoch = tp->reset_epoch;
		return;
	}
	if (!tp->tc.registers_valid) {
		ret = rtlmq_tc_restore_locked(tp);
		if (ret) {
			netdev_err(tp->ndev,
				   "CBS link-ready programming failed: %d; withdrawing service\n",
				   ret);
			rtlmq_tc_retire_failed_update_locked(tp);
			return;
		}
	}
	tp->tc.active_valid = true;
	tp->tc.link_waiting = false;
	tp->tc.blocked_epoch = 0;
	if (owned) {
		rtlmq_serial_tx_resume(tp);
		netif_device_attach(tp->ndev);
	}
}

bool rtlmq_tc_tx_queue_count_allowed(const struct rtlmq_private *tp,
				     u16 num_tx_queues)
{
	return !tp || !tp->tc.requested.mqprio_enabled ||
	       num_tx_queues == RTLMQ_IMPL_MAX_TX_QUEUES;
}
