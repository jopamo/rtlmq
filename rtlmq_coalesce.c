// SPDX-License-Identifier: GPL-2.0-only

#include <linux/errno.h>
#include <linux/ethtool.h>
#include <linux/bitops.h>
#include <linux/moduleparam.h>
#include <linux/netdevice.h>
#include <linux/string.h>

#include "rtlmq_diag.h"
#include "rtlmq_hw.h"
#include "rtlmq_runtime_accessors.h"
#include "rtlmq_irq.h"
#include "rtlmq_lifecycle.h"
#include "rtlmq_runtime.h"
#include "rtlmq.h"

#define RTLMQ_INT_MITI_COUNT_BITS	4
#define RTLMQ_INT_MITI_COUNT_MAX	15
#define RTLMQ_INT_MITI_FRAME_UNIT_16	3
#define RTLMQ_INT_MITI_FRAME_UNIT_16_MIN	64
#define RTLMQ_MODERATION_V5_UNSUPPORTED_MSG \
	"RTL8126 moderation-v5 supports only disabled coalescing; " \
	"nonzero timer/frame encoding is not source-established"

static u8 rtlmq_int_miti_encode_count_unit(u32 value)
{
	u8 count;
	u8 unit;
	u8 msb;

	if (!value)
		return 0;

	if (value <= RTLMQ_INT_MITI_COUNT_MAX)
		return value;

	msb = fls(value);
	unit = msb - RTLMQ_INT_MITI_COUNT_BITS;
	count = value >> unit;

	return count | (unit << RTLMQ_INT_MITI_COUNT_BITS);
}

static u16 rtlmq_int_miti_encode_frames(u32 frames)
{
	u8 count;
	u8 unit;
	u8 msb;

	if (!frames)
		return 0;

	if (frames > RTLMQ_MAX_IRQ_COALESCE_FRAMES)
		frames = RTLMQ_MAX_IRQ_COALESCE_FRAMES;

	if (frames >= RTLMQ_INT_MITI_FRAME_UNIT_16_MIN) {
		unit = RTLMQ_INT_MITI_FRAME_UNIT_16;
		count = frames / 16;
	} else if (frames <= RTLMQ_INT_MITI_COUNT_MAX) {
		unit = 0;
		count = frames;
	} else {
		msb = fls(frames);
		unit = msb - RTLMQ_INT_MITI_COUNT_BITS;
		count = frames >> unit;
	}

	return ((u16)count << 8) | ((u16)unit << 12);
}

u16 rtlmq_coalesce_encode_moderation(u32 usecs, u32 frames)
{
	if (usecs > RTLMQ_MAX_IRQ_COALESCE_USECS)
		usecs = RTLMQ_MAX_IRQ_COALESCE_USECS;

	return rtlmq_int_miti_encode_count_unit(usecs) |
	       rtlmq_int_miti_encode_frames(frames);
}

u16 rtlmq_coalesce_effective_tx_moderation(const struct rtlmq_private *tp,
					   u32 usecs, u32 frames)
{
	if (tp->tc.registers_valid && rtlmq_tc_cbs_enabled(&tp->tc.active))
		return 0;
	return rtlmq_coalesce_encode_moderation(usecs, frames);
}

static uint default_rx_coalesce_usecs = RTLMQ_DEFAULT_RX_IRQ_COALESCE_USECS;
module_param_named(rx_coalesce_usecs,
		   default_rx_coalesce_usecs, uint, 0400);
MODULE_PARM_DESC(rx_coalesce_usecs,
		 "boot default RX interrupt moderation usecs");

static uint default_rx_coalesce_frames = RTLMQ_DEFAULT_RX_IRQ_COALESCE_FRAMES;
module_param_named(rx_coalesce_frames,
		   default_rx_coalesce_frames, uint, 0400);
MODULE_PARM_DESC(rx_coalesce_frames,
		 "boot default RX interrupt moderation frames");

static uint default_tx_coalesce_usecs = RTLMQ_DEFAULT_TX_IRQ_COALESCE_USECS;
module_param_named(tx_coalesce_usecs,
		   default_tx_coalesce_usecs, uint, 0400);
MODULE_PARM_DESC(tx_coalesce_usecs,
		 "boot default TX interrupt moderation usecs");

static uint default_tx_coalesce_frames = RTLMQ_DEFAULT_TX_IRQ_COALESCE_FRAMES;
module_param_named(tx_coalesce_frames,
		   default_tx_coalesce_frames, uint, 0400);
MODULE_PARM_DESC(tx_coalesce_frames,
		 "boot default TX interrupt moderation frames");

static u32 rtlmq_coalesce_clamp_default_usecs(u32 usecs)
{
	if (WARN_ON_ONCE(usecs > RTLMQ_MAX_IRQ_COALESCE_USECS))
		return RTLMQ_MAX_IRQ_COALESCE_USECS;

	return usecs;
}

static u32 rtlmq_coalesce_clamp_default_frames(u32 frames)
{
	if (WARN_ON_ONCE(frames > RTLMQ_MAX_IRQ_COALESCE_FRAMES))
		return RTLMQ_MAX_IRQ_COALESCE_FRAMES;

	return frames;
}

u32 rtlmq_default_rx_irq_coalesce_usecs(void)
{
	return rtlmq_coalesce_clamp_default_usecs(READ_ONCE(default_rx_coalesce_usecs));
}

u32 rtlmq_default_rx_irq_coalesce_frames(void)
{
	return rtlmq_coalesce_clamp_default_frames(READ_ONCE(default_rx_coalesce_frames));
}

u32 rtlmq_default_tx_irq_coalesce_usecs(void)
{
	return rtlmq_coalesce_clamp_default_usecs(READ_ONCE(default_tx_coalesce_usecs));
}

u32 rtlmq_default_tx_irq_coalesce_frames(void)
{
	return rtlmq_coalesce_clamp_default_frames(READ_ONCE(default_tx_coalesce_frames));
}

static bool
rtlmq_coalesce_has_unsupported_fields(const struct ethtool_coalesce *ec)
{
	return ec->rx_coalesce_usecs_irq ||
	       ec->rx_max_coalesced_frames_irq ||
	       ec->tx_coalesce_usecs_irq ||
	       ec->tx_max_coalesced_frames_irq ||
	       ec->stats_block_coalesce_usecs ||
	       ec->use_adaptive_rx_coalesce ||
	       ec->use_adaptive_tx_coalesce ||
	       ec->pkt_rate_low ||
	       ec->rx_coalesce_usecs_low ||
	       ec->rx_max_coalesced_frames_low ||
	       ec->tx_coalesce_usecs_low ||
	       ec->tx_max_coalesced_frames_low ||
	       ec->pkt_rate_high ||
	       ec->rx_coalesce_usecs_high ||
	       ec->rx_max_coalesced_frames_high ||
	       ec->tx_coalesce_usecs_high ||
	       ec->tx_max_coalesced_frames_high ||
	       ec->rate_sample_interval;
}

static bool
rtlmq_coalesce_has_unsupported_kernel_fields(const struct kernel_ethtool_coalesce *kernel_coal)
{
	if (!kernel_coal)
		return false;

	/*
	 * kernel_ethtool_coalesce changes across kernel versions. Check only
	 * the private fields available in the headers being built. ethtool
	 * already filters userspace fields through supported_coalesce_params.
	 */
	return kernel_coal->use_cqe_mode_tx ||
	       kernel_coal->use_cqe_mode_rx ||
	       kernel_coal->tx_aggr_max_bytes ||
	       kernel_coal->tx_aggr_max_frames ||
	       kernel_coal->tx_aggr_time_usecs ||
	       kernel_coal->rx_cqe_frames ||
	       kernel_coal->rx_cqe_nsecs;
}

struct rtlmq_coalesce_request {
	u32 rx_usecs;
	u32 rx_frames;
	u32 tx_usecs;
	u32 tx_frames;
};

static int
rtlmq_normalize_coalesce_request(const struct ethtool_coalesce *ec,
				 const struct kernel_ethtool_coalesce *kernel_coal,
				 struct netlink_ext_ack *extack,
				 struct rtlmq_coalesce_request *request)
{
	if (!ec || !request)
		return -EINVAL;
	if (ec->rx_coalesce_usecs > RTLMQ_MAX_IRQ_COALESCE_USECS ||
	    ec->tx_coalesce_usecs > RTLMQ_MAX_IRQ_COALESCE_USECS) {
		NL_SET_ERR_MSG_MOD(extack,
				   "coalesce usecs exceeds driver maximum");
		return -EINVAL;
	}

	if (ec->rx_max_coalesced_frames > RTLMQ_MAX_IRQ_COALESCE_FRAMES ||
	    ec->tx_max_coalesced_frames > RTLMQ_MAX_IRQ_COALESCE_FRAMES) {
		NL_SET_ERR_MSG_MOD(extack,
				   "coalesce frames exceeds driver maximum");
		return -EINVAL;
	}

	if (rtlmq_coalesce_has_unsupported_fields(ec)) {
		NL_SET_ERR_MSG_MOD(extack,
				   "only RX/TX coalesce usecs and frames are supported; "
				   "the requested userspace field is unsupported");
		return -EOPNOTSUPP;
	}

	if (rtlmq_coalesce_has_unsupported_kernel_fields(kernel_coal)) {
		NL_SET_ERR_MSG_MOD(extack,
				   "only RX/TX coalesce usecs and frames are supported; "
				   "the requested kernel field is unsupported");
		return -EOPNOTSUPP;
	}

	request->rx_usecs = ec->rx_coalesce_usecs;
	request->rx_frames = ec->rx_max_coalesced_frames;
	request->tx_usecs = ec->tx_coalesce_usecs;
	request->tx_frames = ec->tx_max_coalesced_frames;
	return 0;
}

void rtlmq_set_requested_irq_coalesce_usecs(struct rtlmq_private *tp, u32 usecs)
{
	if (WARN_ON_ONCE(usecs > RTLMQ_MAX_IRQ_COALESCE_USECS))
		usecs = RTLMQ_MAX_IRQ_COALESCE_USECS;

	WRITE_ONCE(tp->requested_irq_coalesce_usecs, usecs);
}

void rtlmq_set_requested_irq_coalesce_frames(struct rtlmq_private *tp,
					     u32 frames)
{
	if (WARN_ON_ONCE(frames > RTLMQ_MAX_IRQ_COALESCE_FRAMES))
		frames = RTLMQ_MAX_IRQ_COALESCE_FRAMES;

	WRITE_ONCE(tp->requested_irq_coalesce_frames, frames);
}

void rtlmq_set_requested_tx_irq_coalesce_usecs(struct rtlmq_private *tp,
					       u32 usecs)
{
	if (WARN_ON_ONCE(usecs > RTLMQ_MAX_IRQ_COALESCE_USECS))
		usecs = RTLMQ_MAX_IRQ_COALESCE_USECS;

	WRITE_ONCE(tp->requested_tx_irq_coalesce_usecs, usecs);
}

void rtlmq_set_requested_tx_irq_coalesce_frames(struct rtlmq_private *tp,
						u32 frames)
{
	if (WARN_ON_ONCE(frames > RTLMQ_MAX_IRQ_COALESCE_FRAMES))
		frames = RTLMQ_MAX_IRQ_COALESCE_FRAMES;

	WRITE_ONCE(tp->requested_tx_irq_coalesce_frames, frames);
}

int rtlmq_ethtool_get_coalesce(struct net_device *ndev,
			       struct ethtool_coalesce *ec,
			       struct kernel_ethtool_coalesce *kernel_coal,
			       struct netlink_ext_ack *extack)
{
	struct rtlmq_private *tp = netdev_priv(ndev);

	(void)kernel_coal;
	(void)extack;

	memset(ec, 0, sizeof(*ec));
	mutex_lock(&tp->control_lock);
	ec->rx_coalesce_usecs = tp->requested_irq_coalesce_usecs;
	ec->tx_coalesce_usecs = tp->requested_tx_irq_coalesce_usecs;
	ec->rx_max_coalesced_frames = tp->requested_irq_coalesce_frames;
	ec->tx_max_coalesced_frames = tp->requested_tx_irq_coalesce_frames;
	mutex_unlock(&tp->control_lock);

	return 0;
}

int rtlmq_ethtool_set_coalesce(struct net_device *ndev,
			       struct ethtool_coalesce *ec,
			       struct kernel_ethtool_coalesce *kernel_coal,
			       struct netlink_ext_ack *extack)
{
	struct rtlmq_private *tp = netdev_priv(ndev);
	struct rtlmq_coalesce_request request;
	int ret;

	/*
	 * While OPEN, verify hardware before saving the new request. On write
	 * failure, attempt to restore all sources; retire service if restoration
	 * fails or MMIO is lost. While down, save the request for the next open.
	 *
	 * RX and TX coalescing use separate hardware moderation registers.
	 * Software IRQ deferral remains under the core NAPI settings.
	 */
	ret = rtlmq_normalize_coalesce_request(ec, kernel_coal, extack,
					       &request);
	if (ret)
		return ret;

	mutex_lock(&tp->control_lock);
	ret = rtlmq_admit_control_change_locked(tp, true);
	if (ret) {
		if (ret == -ENODEV)
			NL_SET_ERR_MSG_MOD(extack, "device is unavailable");
		else if (ret == -EBUSY)
			NL_SET_ERR_MSG_MOD(extack,
					   "coalesce changes are blocked while the device is busy");
		goto out_unlock;
	}
	if ((rtlmq_hw_irq_moderation_version(tp) ==
		     RTLMQ_IRQ_MODERATION_V5
	    ) &&
	    (request.rx_usecs || request.rx_frames ||
	     request.tx_usecs || request.tx_frames)) {
		/*
		 * Disabled banks have byte timer slots but no established
		 * ethtool time/frame conversion. Do not use the RTL8125 codec.
		 */
		NL_SET_ERR_MSG_MOD(extack, RTLMQ_MODERATION_V5_UNSUPPORTED_MSG);
		ret = -EOPNOTSUPP;
		goto out_unlock;
	}
	if (tp->state == RTLMQ_STATE_OPEN) {
		u16 rx_moderation, tx_moderation;

		rx_moderation = rtlmq_coalesce_encode_moderation(request.rx_usecs,
								 request.rx_frames);
		tx_moderation = rtlmq_coalesce_effective_tx_moderation(tp,
								       request.tx_usecs,
								       request.tx_frames);
		ret = rtlmq_program_irq_moderation(tp, rx_moderation, tx_moderation);
		if (ret) {
			NL_SET_ERR_MSG_MOD(extack,
					   "interrupt moderation hardware update failed");
			if (ret == -EUCLEAN || ret == -ENODEV) {
				int close_ret;

				close_ret = rtlmq_retire_failed_control_update_locked(tp);
				if (close_ret)
					netdev_err(ndev,
						   "moderation rollback shutdown failed: %d\n",
						   close_ret);
			}
			goto out_unlock;
		}
	}
	rtlmq_set_requested_irq_coalesce_usecs(tp, request.rx_usecs);
	rtlmq_set_requested_irq_coalesce_frames(tp, request.rx_frames);
	rtlmq_set_requested_tx_irq_coalesce_usecs(tp, request.tx_usecs);
	rtlmq_set_requested_tx_irq_coalesce_frames(tp, request.tx_frames);
	ret = 0;
out_unlock:
	mutex_unlock(&tp->control_lock);

	return ret;
}
