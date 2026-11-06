// SPDX-License-Identifier: GPL-2.0-only

#include <linux/bitops.h>
#include <linux/limits.h>
#include <linux/rtnetlink.h>
#include <linux/string.h>

#include "rtlmq.h"
#include "rtlmq_diag.h"
#include "rtlmq_lifecycle.h"
#include "rtlmq_tx.h"

static const char *rtlmq_reset_reason_name(enum rtlmq_reset_reason reason)
{
	switch (reason) {
	case RTLMQ_RESET_REASON_TX_TIMEOUT:
		return "tx_timeout";
	case RTLMQ_RESET_REASON_CHANNEL_RECONFIG:
		return "channel_reconfig";
	case RTLMQ_RESET_REASON_RUNTIME_RECONFIG:
		return "runtime_reconfig";
	case RTLMQ_RESET_REASON_LINK_WEDGE:
		return "link_wedge";
	case RTLMQ_RESET_REASON_PCI_RECOVERY:
		return "pci_recovery";
	case RTLMQ_RESET_REASON_USER:
		return "user";
	case RTLMQ_RESET_REASON_PM_RESUME:
		return "pm_resume";
	case RTLMQ_RESET_REASON_RX_FILTER:
		return "rx_filter";
	default:
		return "unknown";
	}
}

void rtlmq_format_reset_reasons(char *buf, size_t size, u32 reasons)
{
	size_t len = 0;
	unsigned int i;

	if (!size)
		return;

	if (!reasons) {
		strscpy(buf, "none", size);
		return;
	}

	for (i = 0; i < __RTLMQ_RESET_REASON_COUNT && len < size; i++) {
		if (!(reasons & BIT(i)))
			continue;

		len += scnprintf(buf + len, size - len, "%s%s",
				 len ? "," : "",
				 rtlmq_reset_reason_name(i));
	}
}

static void rtlmq_clear_pending_reset_locked(struct rtlmq_private *tp)
{
	tp->reset_pending_epoch = 0;
	tp->reset_pending_reasons = 0;
	tp->tx_timeout_pending_events = 0;
	tp->last_tx_timeout_valid = false;
}

void rtlmq_advance_reset_epoch_locked(struct rtlmq_private *tp)
{
	unsigned long flags;

	lockdep_assert_held(&tp->control_lock);

	spin_lock_irqsave(&tp->state_lock, flags);
	lockdep_assert_held(&tp->state_lock);
	tp->reset_epoch++;
	rtlmq_clear_pending_reset_locked(tp);
	spin_unlock_irqrestore(&tp->state_lock, flags);
}

static bool rtlmq_queue_reset(struct rtlmq_private *tp, u32 reset_reasons,
			      const struct rtlmq_tx_timeout_snapshot *snap,
			      u32 epoch)
{
	unsigned long flags;

	spin_lock_irqsave(&tp->state_lock, flags);
	lockdep_assert_held(&tp->state_lock);
	if (snap && epoch != tp->reset_epoch) {
		spin_unlock_irqrestore(&tp->state_lock, flags);
		return false;
	}
	if (!tp->reset_pending_reasons)
		tp->reset_pending_epoch = tp->reset_epoch;
	tp->reset_pending_reasons |= reset_reasons;
	if (snap) {
		rtlmq_store_tx_timeout_snapshot_locked(tp, snap);
		if (tp->tx_timeout_pending_events != UINT_MAX)
			tp->tx_timeout_pending_events++;
	}
	spin_unlock_irqrestore(&tp->state_lock, flags);
	return true;
}

static void rtlmq_request_reset_delayed_owned(struct rtlmq_private *tp,
					      u32 reset_reasons,
					      unsigned long delay,
					      const struct rtlmq_tx_timeout_snapshot *snap,
					      u32 epoch)
{
	if (!tp || !reset_reasons || !tp->ndev)
		return;

	if (READ_ONCE(tp->state) != RTLMQ_STATE_OPEN &&
	    READ_ONCE(tp->state) != RTLMQ_STATE_PROBED)
		return;

	if (!rtlmq_queue_reset(tp, reset_reasons, snap, epoch))
		return;

	/*
	 * Delay TX-timeout reset briefly, as the vendor driver does, to allow
	 * pending completion interrupts/NAPI to run first. A delayed request
	 * must not postpone an immediate reset; an immediate request may
	 * advance a delayed reset.
	 */
	if (delay)
		queue_delayed_work(system_dfl_wq, &tp->reset_work, delay);
	else
		mod_delayed_work(system_dfl_wq, &tp->reset_work, 0);
}

static void rtlmq_request_reset_delayed(struct rtlmq_private *tp,
					u32 reset_reasons,
					unsigned long delay)
{
	rtlmq_request_reset_delayed_owned(tp, reset_reasons, delay, NULL, 0);
}

void rtlmq_request_tx_timeout_validation(struct rtlmq_private *tp,
					 const struct rtlmq_tx_timeout_snapshot *snap,
					 u32 epoch,
					 unsigned long delay)
{
	if (!snap)
		return;
	rtlmq_request_reset_delayed_owned(tp,
					  BIT(RTLMQ_RESET_REASON_TX_TIMEOUT),
					  delay, snap, epoch);
}

void rtlmq_request_reset(struct rtlmq_private *tp,
			 u32 reset_reasons
)
{
	rtlmq_request_reset_delayed(tp, reset_reasons
				    , 0);
}

struct rtlmq_reset_batch {
	u32 reasons;
	unsigned int timeout_events;
	bool timeout_valid;
	struct rtlmq_tx_timeout_snapshot timeout;
};

static void rtlmq_consume_reset(struct rtlmq_private *tp,
				struct rtlmq_reset_batch *batch)
{
	unsigned long flags;

	spin_lock_irqsave(&tp->state_lock, flags);
	lockdep_assert_held(&tp->state_lock);
	if (tp->reset_pending_reasons &&
	    tp->reset_pending_epoch == tp->reset_epoch)
		batch->reasons = tp->reset_pending_reasons;
	if (batch->reasons & BIT(RTLMQ_RESET_REASON_TX_TIMEOUT)) {
		batch->timeout_events = tp->tx_timeout_pending_events;
		batch->timeout_valid = batch->timeout_events &&
			rtlmq_load_tx_timeout_snapshot_locked(tp, &batch->timeout);
	}
	rtlmq_clear_pending_reset_locked(tp);
	spin_unlock_irqrestore(&tp->state_lock, flags);
}

void rtlmq_reset_work(struct work_struct *work)
{
	const struct rtlmq_tx_timeout_snapshot *timeout;
	struct rtlmq_reset_batch batch = {};
	struct rtlmq_private *tp;
	char reason_buf[96];
	int ret;

	tp = container_of(to_delayed_work(work), struct rtlmq_private,
			  reset_work);

	rtnl_lock();
	mutex_lock(&tp->control_lock);

	rtlmq_consume_reset(tp, &batch);
	timeout = batch.timeout_valid ? &batch.timeout : NULL;
	if (!batch.reasons)
		goto out_unlock;

	if (!netif_running(tp->ndev))
		goto out_unlock;

	if (rtlmq_tx_timeout_maybe_suppress_reset(tp, batch.reasons, batch.timeout_events, timeout))
		goto out_unlock;

	ret = rtlmq_recover_queued_reset_locked(tp, batch.reasons);
	if (ret == -EBUSY)
		goto out_unlock;

	if (ret) {
		rtlmq_format_reset_reasons(reason_buf, sizeof(reason_buf),
					   batch.reasons);
		netdev_err(tp->ndev,
			   "reset unwind left device quiesced reasons=%s ret=%d\n",
			   reason_buf, ret);
	}

out_unlock:
	mutex_unlock(&tp->control_lock);
	rtnl_unlock();
}
