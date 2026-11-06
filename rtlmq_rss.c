// SPDX-License-Identifier: GPL-2.0-only

#include <linux/ethtool.h>
#include <linux/log2.h>
#include <linux/unaligned.h>

#include "rtlmq_regs.h"
#include "rtlmq_hw.h"
#include "rtlmq_runtime.h"
#include "rtlmq_lifecycle.h"
#include "rtlmq.h"
#include "rtlmq_runtime_accessors.h"
#include "rtlmq_xdp.h"

static u32 rtlmq_default_rss_hash_options(void)
{
	return RTLMQ_RSS_CTRL_PROTOCOL_DEFAULT;
}

static unsigned int rtlmq_rss_queue_count(const struct rtlmq_private *tp)
{
	return rtlmq_candidate_or_published_num_rx_queues(tp) ?
	       rtlmq_candidate_or_published_num_rx_queues(tp) : 1;
}

static unsigned int
rtlmq_rss_queue_count_for_config(const struct rtlmq_runtime_config *cfg)
{
	return cfg && cfg->num_rx_queues ? cfg->num_rx_queues : 1;
}

struct rtlmq_rss *rtlmq_requested_rss_state_owned_mut(struct rtlmq_private *tp)
{
	if (!tp)
		return NULL;

	return &tp->rss;
}

struct rtlmq_rss *rtlmq_candidate_rss_state_owned_mut(struct rtlmq_private *tp)
{
	if (!tp || !tp->transaction.rss_valid)
		return NULL;

	return &tp->transaction.rss;
}

static const struct rtlmq_rss *
rtlmq_candidate_or_published_rss_state(const struct rtlmq_private *tp)
{
	const struct rtlmq_rss *rss;

	rss = rtlmq_candidate_rss_state(tp);
	if (rss)
		return rss;

	return rtlmq_published_rss_state(tp);
}

static void rtlmq_reset_rss_indirection(struct rtlmq_rss *rss,
					unsigned int rx_queues)
{
	unsigned int i;

	if (!rss)
		return;

	for (i = 0; i < RTLMQ_RSS_INDIR_SIZE; i++)
		rss->indir[i] = ethtool_rxfh_indir_default(i, rx_queues);
}

static void
rtlmq_rebalance_rss_state_for_config(struct rtlmq_private *tp,
				     struct rtlmq_rss *rss,
				     const struct rtlmq_runtime_config *cfg)
{
	unsigned int rx_queues = rtlmq_rss_queue_count_for_config(cfg);
	unsigned int previous_rx_queues;

	if (!rss)
		return;

	previous_rx_queues = rss->programmed_queue_count;

	/*
	 * Initialize the table on first use, then reset it on RX count changes.
	 * Preserve user key, table, and hash fields across same-width reopen.
	 */
	rss->enabled = rtlmq_chip_supports_rss(tp) && cfg && cfg->rss_enabled;
	if (!previous_rx_queues || previous_rx_queues != rx_queues)
		rtlmq_reset_rss_indirection(rss, rx_queues);
	rss->programmed_queue_count = rx_queues;
}

void rtlmq_clear_candidate_rss(struct rtlmq_private *tp)
{
	if (!tp)
		return;

	tp->transaction.rss_valid = false;
	memset(&tp->transaction.rss, 0, sizeof(tp->transaction.rss));
}

static void rtlmq_stage_candidate_rss_from(struct rtlmq_private *tp,
					   const struct rtlmq_rss *base)
{
	if (!tp)
		return;

	if (base) {
		tp->transaction.rss = *base;
	} else {
		memset(&tp->transaction.rss, 0, sizeof(tp->transaction.rss));
		tp->transaction.rss.hash_options =
			rtlmq_default_rss_hash_options();
	}

	tp->transaction.rss_valid = true;
}

void rtlmq_stage_candidate_rss_from_requested(struct rtlmq_private *tp)
{
	rtlmq_stage_candidate_rss_from(tp, rtlmq_requested_rss_state(tp));
}

void rtlmq_stage_candidate_rss_for_config(struct rtlmq_private *tp,
					  const struct rtlmq_runtime_config *cfg)
{
	struct rtlmq_rss *rss;

	/*
	 * Seed candidate RSS from active state when present, otherwise from the
	 * committed request, not a rollback snapshot.
	 */
	rtlmq_stage_candidate_rss_from(tp,
				       rtlmq_active_or_requested_rss_state(tp));
	rss = rtlmq_candidate_rss_state_owned_mut(tp);
	rtlmq_rebalance_rss_state_for_config(tp, rss, cfg);
}

static void rtlmq_commit_candidate_rss(struct rtlmq_private *tp,
				       bool commit_requested, bool commit_active
)
{
	if (commit_requested)
		tp->rss = tp->transaction.rss;
	if (commit_active)
		tp->active_rss = tp->transaction.rss;
	rtlmq_clear_candidate_rss(tp);
}

int rtlmq_apply_candidate_rss_transaction(struct rtlmq_private *tp)
{
	bool publish_active;
	int ret;

	if (!tp || !tp->transaction.rss_valid)
		return -EINVAL;

	lockdep_assert_held(&tp->control_lock);

	switch (READ_ONCE(tp->state)) {
	case RTLMQ_STATE_OPEN:
		publish_active = true;
		break;
	case RTLMQ_STATE_PROBED:
		publish_active = false;
		break;
	default:
		return -EBUSY;
	}

	/*
	 * While OPEN, program hardware before copying the candidate to requested
	 * and active state. While PROBED, update only the request. A programming
	 * error clears the candidate without changing the saved software state.
	 */
	if (publish_active) {
		ret = rtlmq_program_rss(tp);
		if (ret)
			goto err_clear_candidate;
	}

	rtlmq_commit_candidate_rss(tp, true, publish_active);
	return 0;

err_clear_candidate:
	rtlmq_clear_candidate_rss(tp);
	return ret;
}

void rtlmq_publish_rss_active(struct rtlmq_private *tp)
{
	/*
	 * Commit requested and active RSS together so a channel change updates
	 * requested programmed_queue_count to match the new runtime.
	 */
	rtlmq_commit_candidate_rss(tp, true, true);
}

void rtlmq_unwind_active_rss(struct rtlmq_private *tp)
{
	if (!tp)
		return;

	/*
	 * active_profile.valid determines whether active RSS is usable.
	 * Clear storage without adding another validity flag.
	 */
	memset(&tp->active_rss, 0, sizeof(tp->active_rss));
}

void rtlmq_rebalance_rss_for_config(struct rtlmq_private *tp,
				    const struct rtlmq_runtime_config *cfg)
{
	struct rtlmq_rss *rss = rtlmq_requested_rss_state_owned_mut(tp);

	rtlmq_rebalance_rss_state_for_config(tp, rss, cfg);
}

void rtlmq_rebalance_rss(struct rtlmq_private *tp)
{
	rtlmq_rebalance_rss_for_config(tp, rtlmq_active_or_requested_runtime_config(tp));
}

void rtlmq_rebalance_requested_rss(struct rtlmq_private *tp)
{
	rtlmq_rebalance_rss_for_config(tp, rtlmq_requested_runtime_config(tp));
}

void rtlmq_init_rss(struct rtlmq_private *tp)
{
	struct rtlmq_rss *rss = rtlmq_requested_rss_state_owned_mut(tp);

	if (!rss)
		return;

	memset(rss, 0, sizeof(*rss));
	memset(&tp->transaction.rss, 0, sizeof(tp->transaction.rss));
	memset(&tp->active_rss, 0, sizeof(tp->active_rss));
	tp->transaction.rss_valid = false;
	netdev_rss_key_fill(rss->hash_key, sizeof(rss->hash_key));
	rss->hash_options = rtlmq_default_rss_hash_options();
	rtlmq_rebalance_rss(tp);
}

struct rtlmq_rxfh_request {
	bool has_indir;
	bool has_key;
	u32 indir[RTLMQ_RSS_INDIR_SIZE];
	u8 key[RTLMQ_RSS_KEY_SIZE];
};

static int
rtlmq_validate_rxfh_buffer_sizes(const struct ethtool_rxfh_param *rxfh,
				 bool set, struct netlink_ext_ack *extack)
{
	if (!rxfh)
		return -EINVAL;

	/*
	 * SET requests carry explicit device-sized buffers. Most GET callers do
	 * too, but ethtool_check_max_channel() intentionally allocates the
	 * device-sized indirection table and leaves indir_size zero. Accept that
	 * kernel-internal callback form while still rejecting an explicit,
	 * contradictory nonzero size. A SET with no pointer may carry only the
	 * core's no-change sentinels.
	 */
	if (rxfh->indir) {
		if (rxfh->indir_size != RTLMQ_RSS_INDIR_SIZE &&
		    (set || rxfh->indir_size != 0)) {
			NL_SET_ERR_MSG_MOD(extack,
					   "RSS indirection buffer size does not match the device");
			return -EINVAL;
		}
	} else if (set) {
		if (rxfh->indir_size != 0 &&
		    rxfh->indir_size != ETH_RXFH_INDIR_NO_CHANGE) {
			NL_SET_ERR_MSG_MOD(extack,
					   "RSS indirection size is set without an indirection buffer");
			return -EINVAL;
		}
	} else if (rxfh->indir_size != 0 &&
		   rxfh->indir_size != RTLMQ_RSS_INDIR_SIZE) {
		return -EINVAL;
	}

	if (rxfh->key) {
		if (rxfh->key_size != RTLMQ_RSS_KEY_SIZE &&
		    (set || rxfh->key_size != 0)) {
			NL_SET_ERR_MSG_MOD(extack,
					   "RSS hash-key buffer size does not match the device");
			return -EINVAL;
		}
	} else if (set && rxfh->key_size) {
		NL_SET_ERR_MSG_MOD(extack, "RSS hash-key size is set without a hash-key buffer");
		return -EINVAL;
	} else if (!set && rxfh->key_size != 0 &&
		   rxfh->key_size != RTLMQ_RSS_KEY_SIZE) {
		return -EINVAL;
	}

	return 0;
}

static int rtlmq_normalize_rxfh_request_locked(struct rtlmq_private *tp,
					       const struct ethtool_rxfh_param *rxfh,
					       struct rtlmq_rxfh_request *request,
					       struct netlink_ext_ack *extack)
{
	const struct rtlmq_runtime_config *cfg;
	u16 num_rx_queues;
	unsigned int i;
	int ret;

	lockdep_assert_held(&tp->control_lock);

	if (!rxfh || !request)
		return -EINVAL;
	ret = rtlmq_validate_rxfh_buffer_sizes(rxfh, true, extack);
	if (ret)
		return ret;
	if (!rtlmq_chip_supports_rss(tp)) {
		NL_SET_ERR_MSG_MOD(extack, "RSS is unsupported by this device");
		return -EOPNOTSUPP;
	}
	if (rxfh->rss_context) {
		NL_SET_ERR_MSG_MOD(extack, "RSS contexts are not supported");
		return -EOPNOTSUPP;
	}
	if (rxfh->rss_delete) {
		NL_SET_ERR_MSG_MOD(extack, "RSS context deletion is not supported");
		return -EOPNOTSUPP;
	}
	if (rxfh->input_xfrm != 0 &&
	    rxfh->input_xfrm != RXH_XFRM_NO_CHANGE &&
	    rxfh->input_xfrm != RXH_XFRM_SYM_XOR &&
	    rxfh->input_xfrm != RXH_XFRM_SYM_OR_XOR) {
		NL_SET_ERR_MSG_MOD(extack, "invalid RSS input transformation");
		return -EINVAL;
	}
	if (rxfh->input_xfrm != 0 &&
	    rxfh->input_xfrm != RXH_XFRM_NO_CHANGE) {
		NL_SET_ERR_MSG_MOD(extack,
				   "RSS input transformation is unsupported");
		return -EOPNOTSUPP;
	}
	if (rxfh->hfunc != ETH_RSS_HASH_NO_CHANGE &&
	    rxfh->hfunc != ETH_RSS_HASH_TOP) {
		NL_SET_ERR_MSG_MOD(extack,
				   "requested RSS hash function is unsupported");
		return -EOPNOTSUPP;
	}

	cfg = rtlmq_active_or_requested_runtime_config(tp);
	if (!cfg)
		return -EINVAL;
	num_rx_queues = cfg->num_rx_queues;

	memset(request, 0, sizeof(*request));
	if (rxfh->indir) {
		for (i = 0; i < RTLMQ_RSS_INDIR_SIZE; i++) {
			if (rxfh->indir[i] >= num_rx_queues) {
				NL_SET_ERR_MSG_MOD(extack,
						   "RSS indirection entry refers to an unavailable RX queue");
				return -EINVAL;
			}
			request->indir[i] = rxfh->indir[i];
		}
		request->has_indir = true;
	}
	if (rxfh->key) {
		memcpy(request->key, rxfh->key, RTLMQ_RSS_KEY_SIZE);
		request->has_key = true;
	}

	return 0;
}

static int rtlmq_validate_rss_indirection(struct rtlmq_private *tp)
{
	const struct rtlmq_rss *rss = rtlmq_candidate_or_published_rss_state(tp);
	unsigned int rx_queues = rtlmq_rss_queue_count(tp);
	unsigned int i;

	if (!rss)
		return -EINVAL;

	for (i = 0; i < RTLMQ_RSS_INDIR_SIZE; i++) {
		if (rss->indir[i] >= rx_queues)
			return -EINVAL;
	}

	return 0;
}

static void rtlmq_write_rss_key(struct rtlmq_private *tp)
{
	const struct rtlmq_rss *rss = rtlmq_candidate_or_published_rss_state(tp);
	unsigned int i;

	if (!rtlmq_mmio_ordinary_authorized(tp) || !rss)
		return;

	for (i = 0; i < RTLMQ_RSS_KEY_SIZE; i += sizeof(u32))
		writel(get_unaligned_le32(rss->hash_key + i),
		       tp->mmio_addr + RTLMQ_RSS_KEY + i);
}

static void rtlmq_write_rss_indirection(struct rtlmq_private *tp)
{
	const struct rtlmq_rss *rss = rtlmq_candidate_or_published_rss_state(tp);
	unsigned int i;
	u32 reta = 0;

	if (!rtlmq_mmio_ordinary_authorized(tp) || !rss)
		return;

	for (i = 0; i < RTLMQ_RSS_INDIR_SIZE; i++) {
		reta |= (rss->indir[i] & 0xff) << ((i & 0x3) * 8);
		if ((i & 0x3) == 0x3) {
			writel(reta, tp->mmio_addr + RTLMQ_RSS_INDIR_TBL +
			       ((i >> 2) * sizeof(u32)));
			reta = 0;
		}
	}
}

static u32 rtlmq_build_rss_ctrl(struct rtlmq_private *tp)
{
	const struct rtlmq_rss *rss = rtlmq_candidate_or_published_rss_state(tp);
	u32 rss_ctrl;

	if (!rss)
		return 0;

	rss_ctrl = rss->hash_options;
	rss_ctrl |= ilog2(RTLMQ_RSS_INDIR_SIZE) <<
		    RTLMQ_RSS_CTRL_HASH_MASK_SHIFT;
	rss_ctrl |= ilog2(rtlmq_candidate_or_published_num_rx_queues(tp)) <<
		    RTLMQ_RSS_CTRL_CPU_COUNT_SHIFT;

	return rss_ctrl;
}

int rtlmq_program_rss(struct rtlmq_private *tp)
{
	struct rtlmq_rss *rss = rtlmq_candidate_rss_state_owned_mut(tp);
	u16 num_rx_queues = rtlmq_candidate_or_published_num_rx_queues(tp);

	lockdep_assert_held(&tp->control_lock);

	if (!rtlmq_mmio_ordinary_authorized(tp))
		return -ENODEV;
	if (!rss)
		return -EINVAL;
	if (rtlmq_chip_supports_rss(tp) &&
	    rtlmq_hw_rss_version(tp) != RTLMQ_RSS_VERSION_5)
		return -EOPNOTSUPP;

	/*
	 * Program only staged candidate RSS. Requested or active settings must
	 * first be copied into the candidate; do not mutate them here.
	 */
	if (!rtlmq_chip_supports_rss(tp) || !rtlmq_candidate_or_published_rss_enabled(tp) ||
	    num_rx_queues <= RTLMQ_BOOT_RX_QUEUES) {
		writel(0, tp->mmio_addr + RTLMQ_RSS_CTRL);
		rss->enabled = false;
		rss->programmed_queue_count = num_rx_queues;
		return 0;
	}

	if (!is_power_of_2(num_rx_queues))
		return -EINVAL;

	if (rtlmq_validate_rss_indirection(tp))
		return -EINVAL;

	/*
	 * Disable RSS while replacing the key and indirection table so hardware
	 * cannot hash packets with a mixture of old and new settings.
	 */
	writel(0, tp->mmio_addr + RTLMQ_RSS_CTRL);
	rtlmq_write_rss_key(tp);
	rtlmq_write_rss_indirection(tp);
	writel(rtlmq_build_rss_ctrl(tp),
	       tp->mmio_addr + RTLMQ_RSS_CTRL);

	rss->enabled = true;
	rss->programmed_queue_count = num_rx_queues;

	return 0;
}

u32 rtlmq_get_rxfh_key_size(struct net_device *ndev)
{
	struct rtlmq_private *tp = netdev_priv(ndev);

	return rtlmq_chip_supports_rss(tp) ? RTLMQ_RSS_KEY_SIZE : 0;
}

u32 rtlmq_get_rxfh_indir_size(struct net_device *ndev)
{
	struct rtlmq_private *tp = netdev_priv(ndev);

	return rtlmq_chip_supports_rss(tp) ? RTLMQ_RSS_INDIR_SIZE : 0;
}

int rtlmq_get_rxfh(struct net_device *ndev, struct ethtool_rxfh_param *rxfh)
{
	struct rtlmq_private *tp = netdev_priv(ndev);
	const struct rtlmq_rss *rss;
	unsigned int i;
	int ret;

	ret = rtlmq_validate_rxfh_buffer_sizes(rxfh, false, NULL);
	if (ret)
		return ret;
	if (!rtlmq_chip_supports_rss(tp) || rxfh->rss_context)
		return -EOPNOTSUPP;

	mutex_lock(&tp->control_lock);
	rss = rtlmq_active_or_requested_rss_state(tp);

	if (rxfh->indir)
		for (i = 0; i < RTLMQ_RSS_INDIR_SIZE; i++)
			rxfh->indir[i] = rss->indir[i];

	if (rxfh->key)
		memcpy(rxfh->key, rss->hash_key, RTLMQ_RSS_KEY_SIZE);

	rxfh->hfunc = ETH_RSS_HASH_TOP;
	rxfh->input_xfrm = 0;

	mutex_unlock(&tp->control_lock);

	return 0;
}

int rtlmq_set_rxfh(struct net_device *ndev, struct ethtool_rxfh_param *rxfh,
		   struct netlink_ext_ack *extack)
{
	struct rtlmq_private *tp = netdev_priv(ndev);
	struct rtlmq_rxfh_request request;
	struct rtlmq_rss *rss;
	bool changed = false;
	unsigned int i;
	int ret;

	mutex_lock(&tp->control_lock);

	ret = rtlmq_admit_control_change_locked(tp, true);
	if (ret) {
		if (ret == -ENODEV)
			NL_SET_ERR_MSG_MOD(extack, "device is unavailable");
		else if (ret == -EBUSY)
			NL_SET_ERR_MSG_MOD(extack,
					   "RSS changes are blocked while the device is busy");
		goto out_unlock;
	}

	if (tp->state != RTLMQ_STATE_PROBED &&
	    tp->state != RTLMQ_STATE_OPEN) {
		NL_SET_ERR_MSG_MOD(extack, "RSS changes require a running or quiesced device");
		ret = -EBUSY;
		goto out_unlock;
	}

	ret = rtlmq_normalize_rxfh_request_locked(tp, rxfh, &request,
						  extack);
	if (ret)
		goto out_unlock;

	if (tp->state == RTLMQ_STATE_OPEN)
		rtlmq_stage_candidate_rss_from(tp, rtlmq_active_or_requested_rss_state(tp));
	else
		rtlmq_stage_candidate_rss_from_requested(tp);

	rss = rtlmq_candidate_rss_state_owned_mut(tp);
	if (!rss) {
		ret = -EINVAL;
		goto out_unlock;
	}

	if (request.has_indir)
		for (i = 0; i < RTLMQ_RSS_INDIR_SIZE; i++) {
			if (rss->indir[i] == request.indir[i])
				continue;
			rss->indir[i] = request.indir[i];
			changed = true;
		}

	if (request.has_key &&
	    memcmp(rss->hash_key, request.key, RTLMQ_RSS_KEY_SIZE)) {
		memcpy(rss->hash_key, request.key, RTLMQ_RSS_KEY_SIZE);
		changed = true;
	}

	if (!changed) {
		rtlmq_clear_candidate_rss(tp);
		goto out_unlock;
	}

	/*
	 * Use the same staged RSS update as set_rxnfc() hash-field changes.
	 * The shared helper programs hardware only while OPEN.
	 */
	ret = rtlmq_apply_candidate_rss_transaction(tp);

out_unlock:
	mutex_unlock(&tp->control_lock);

	return ret;
}
