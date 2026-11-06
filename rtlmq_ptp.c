// SPDX-License-Identifier: GPL-2.0-only

#include <linux/atomic.h>
#include <linux/bitfield.h>
#include <linux/ethtool.h>
#include <linux/jiffies.h>
#include <linux/ktime.h>
#include <linux/module.h>
#include <linux/net_tstamp.h>
#include <linux/ptp_clock_kernel.h>
#include <linux/slab.h>
#include <linux/time64.h>
#include <linux/wait.h>
#include <linux/workqueue.h>
#include <net/netlink.h>

#include "rtlmq.h"
#include "rtlmq_hw.h"
#include "rtlmq_runtime_accessors.h"
#include "rtlmq_ptp.h"
#include "rtlmq_regs.h"
#include "rtlmq_runtime.h"

static void rtlmq_ptp_link_reconcile_work(struct work_struct *work);

bool rtlmq_ptp_defer_link_event(struct rtlmq_private *tp)
{
	unsigned long deadline;
	struct rtlmq_ptp *ptp;
	unsigned long delay;

	if (!tp)
		return false;

	ptp = READ_ONCE(tp->ptp);
	if (!ptp)
		return false;

	deadline = READ_ONCE(ptp->link_quarantine_until);
	if (!deadline || time_after_eq(jiffies, deadline))
		return false;

	delay = max_t(unsigned long, 1, deadline - jiffies);
	mod_delayed_work(system_dfl_wq, &ptp->link_reconcile_work, delay);
	return true;
}

static void rtlmq_ptp_link_reconcile_work(struct work_struct *work)
{
	struct rtlmq_ptp *ptp =
		container_of(to_delayed_work(work), struct rtlmq_ptp, link_reconcile_work);
	struct rtlmq_private *tp = ptp->tp;
	unsigned long deadline;
	unsigned long delay;

	mutex_lock(&tp->control_lock);
	if (READ_ONCE(tp->ptp) != ptp)
		goto out_unlock;

	deadline = READ_ONCE(ptp->link_quarantine_until);
	if (!deadline)
		goto out_unlock;
	if (time_before(jiffies, deadline)) {
		delay = max_t(unsigned long, 1, deadline - jiffies);
		mutex_unlock(&tp->control_lock);
		mod_delayed_work(system_dfl_wq, &ptp->link_reconcile_work, delay);
		return;
	}

	WRITE_ONCE(ptp->link_quarantine_until, 0);
	/*
	 * CBS may have withdrawn TX until phylib learns the new link rate.
	 * Suppressing the deferred status read for that owner deadlocks reopen:
	 * the link IRQ was already consumed by the PTP quarantine.
	 */
	if (READ_ONCE(tp->state) == RTLMQ_STATE_OPEN &&
	    netif_running(tp->ndev) &&
	    (netif_device_present(tp->ndev) ||
	     rtlmq_tc_link_recovery_owned(tp))) {
		if (tp->ndev->phydev)
			phy_mac_interrupt(tp->ndev->phydev);
		else
			rtlmq_request_link_state_sync(tp);
	}

out_unlock:
	mutex_unlock(&tp->control_lock);
}

static void rtlmq_ptp_config_to_kernel(const struct rtlmq_ptp_config *source,
				       struct kernel_hwtstamp_config *config)
{
	memset(config, 0, sizeof(*config));
	config->tx_type = source->tx_type;
	config->rx_filter = source->rx_filter;
	config->source = HWTSTAMP_SOURCE_NETDEV;
	config->qualifier = HWTSTAMP_PROVIDER_QUALIFIER_PRECISE;
}

static int rtlmq_ptp_normalize_config(const struct rtlmq_ptp *ptp,
				      struct kernel_hwtstamp_config *config,
				      struct rtlmq_ptp_config *normalized,
				      struct netlink_ext_ack *extack)
{
	if (config->source != HWTSTAMP_SOURCE_NETDEV) {
		NL_SET_ERR_MSG_MOD(extack,
				   "selected PTP backend uses the integrated PHY");
		return -EOPNOTSUPP;
	}
	if (config->flags)
		return -EINVAL;

	switch (config->tx_type) {
	case HWTSTAMP_TX_OFF:
	case HWTSTAMP_TX_ON:
		break;
	default:
		return -ERANGE;
	}

	switch (config->rx_filter) {
	case HWTSTAMP_FILTER_NONE:
		break;
	case HWTSTAMP_FILTER_PTP_V2_EVENT:
	case HWTSTAMP_FILTER_PTP_V2_L2_EVENT:
	case HWTSTAMP_FILTER_PTP_V2_L4_EVENT:
	case HWTSTAMP_FILTER_PTP_V2_SYNC:
	case HWTSTAMP_FILTER_PTP_V2_L2_SYNC:
	case HWTSTAMP_FILTER_PTP_V2_L4_SYNC:
	case HWTSTAMP_FILTER_PTP_V2_DELAY_REQ:
	case HWTSTAMP_FILTER_PTP_V2_L2_DELAY_REQ:
	case HWTSTAMP_FILTER_PTP_V2_L4_DELAY_REQ:
		if (!ptp || !ptp->backend->supports_rx) {
			NL_SET_ERR_MSG_MOD(extack,
					   "RTL8126 PTP RX is not qualified");
			return -EOPNOTSUPP;
		}
		config->rx_filter = HWTSTAMP_FILTER_PTP_V2_EVENT;
		break;
	default:
		return -ERANGE;
	}

	normalized->tx_type = config->tx_type;
	normalized->rx_filter = config->rx_filter;
	return 0;
}

static u8 rtlmq_ptp_config_modes(const struct rtlmq_ptp_config *config)
{
	u8 modes = 0;

	if (config->tx_type == HWTSTAMP_TX_ON)
		modes |= RTLMQ_PTP_MODE_TX;
	if (config->rx_filter != HWTSTAMP_FILTER_NONE)
		modes |= RTLMQ_PTP_MODE_RX;
	return modes;
}

static int rtlmq_ptp_apply_modes_locked(struct rtlmq_ptp *ptp, u8 modes)
{
	struct rtlmq_private *tp = ptp->tp;
	bool hardware_changed = false;
	bool added_rx_scan = false;
	int rollback_ret = 0;
	u8 old_modes;
	int ret;

	lockdep_assert_held(&tp->control_lock);
	if (!READ_ONCE(ptp->active))
		return 0;
	if (!rtlmq_mmio_ordinary_authorized(tp))
		return -ENODEV;

	old_modes = READ_ONCE(ptp->active_modes);
	if (old_modes == modes)
		return 0;

	if (!(old_modes & RTLMQ_PTP_MODE_RX) &&
	    modes & RTLMQ_PTP_MODE_RX) {
		rtlmq_ptp_require_rx_timestamp_scan(ptp);
		added_rx_scan = true;
	}

	if (old_modes & RTLMQ_PTP_MODE_TX &&
	    !(modes & RTLMQ_PTP_MODE_TX))
		rtlmq_ptp_tx_cancel_request(ptp, true);

	ret = rtlmq_ptp_hw_acquire(ptp, false);
	if (ret)
		goto err_restore;
	/*
	 * The PHY timestamp path is already selected. A640 would briefly block
	 * traffic without dropping carrier. Update detector registers only,
	 * as in StopReq pause/resume.
	 */
	ret = rtlmq_ptp_phy_program_modes_owned(ptp, old_modes, modes, true);
	hardware_changed = true;
	if (ret)
		rollback_ret = rtlmq_ptp_phy_program_modes_owned(ptp, modes, old_modes, true);
	rtlmq_ptp_hw_release(ptp);
	if (ret)
		goto err_restore;

	if (!(old_modes & RTLMQ_PTP_MODE_TX) &&
	    modes & RTLMQ_PTP_MODE_TX) {
		ret = rtlmq_ptp_tx_start(ptp, modes);
		if (ret) {
			if (!rtlmq_ptp_hw_acquire(ptp, false)) {
				rollback_ret = rtlmq_ptp_phy_program_modes_owned(ptp, modes,
										 old_modes, true);
				rtlmq_ptp_hw_release(ptp);
			} else {
				rollback_ret = -EBUSY;
			}
			goto err_restore;
		}
	}

	/* Publish detector programming and TX admission to acquire readers. */
	smp_store_release(&ptp->active_modes, modes);
	if (old_modes & RTLMQ_PTP_MODE_RX &&
	    !(modes & RTLMQ_PTP_MODE_RX))
		rtlmq_ptp_clear_rx_timestamp_scan(ptp);
	return 0;

err_restore:
	if (hardware_changed && rollback_ret) {
		/*
		 * Hardware is partially programmed. Clear active PTP state
		 * rather than report the old configuration. A later device
		 * restore can reapply the requested settings.
		 */
		smp_store_release(&ptp->active_modes, 0);
		smp_store_release(&ptp->active, false);
		rtlmq_ptp_tx_cancel_request(ptp, false);
		rtlmq_ptp_clear_rx_timestamp_scan(ptp);
		netdev_err(tp->ndev,
			   "PTP mode rollback failed transition=%#x->%#x primary=%d rollback=%d\n",
			   old_modes, modes, ret, rollback_ret);
		return ret;
	}
	if (added_rx_scan)
		rtlmq_ptp_clear_rx_timestamp_scan(ptp);
	if (old_modes & RTLMQ_PTP_MODE_TX) {
		rollback_ret = rtlmq_ptp_tx_start(ptp, old_modes);
		if (rollback_ret) {
			/* Failed TX admission cannot represent the old active modes. */
			smp_store_release(&ptp->active_modes, 0);
			smp_store_release(&ptp->active, false);
			rtlmq_ptp_clear_rx_timestamp_scan(ptp);
			netdev_err(tp->ndev,
				   "PTP TX admission rollback failed: %d\n",
				   rollback_ret);
		}
	}
	return ret;
}

int rtlmq_ptp_hwtstamp_get(struct net_device *ndev,
			   struct kernel_hwtstamp_config *config)
{
	struct rtlmq_private *tp = netdev_priv(ndev);
	struct rtlmq_ptp *ptp;
	int ret = 0;

	mutex_lock(&tp->control_lock);
	ptp = tp->ptp;
	if (!rtlmq_ptp_backend_matches(tp, ptp))
		ret = -EOPNOTSUPP;
	else
		rtlmq_ptp_config_to_kernel(&ptp->requested_config, config);
	mutex_unlock(&tp->control_lock);
	return ret;
}

int rtlmq_ptp_hwtstamp_set(struct net_device *ndev,
			   struct kernel_hwtstamp_config *config,
			   struct netlink_ext_ack *extack)
{
	struct rtlmq_private *tp = netdev_priv(ndev);
	struct rtlmq_ptp_config normalized;
	struct rtlmq_ptp *ptp;
	u8 modes;
	int ret;

	mutex_lock(&tp->control_lock);
	ptp = tp->ptp;
	if (!rtlmq_ptp_backend_matches(tp, ptp)) {
		ret = -EOPNOTSUPP;
		goto out_unlock;
	}
	if (READ_ONCE(tp->state) == RTLMQ_STATE_REMOVING ||
	    READ_ONCE(tp->state) == RTLMQ_STATE_DEAD) {
		ret = -ENODEV;
		goto out_unlock;
	}
	ret = rtlmq_ptp_normalize_config(ptp, config, &normalized, extack);
	if (ret)
		goto out_unlock;

	modes = rtlmq_ptp_config_modes(&normalized);
	ret = rtlmq_ptp_apply_modes_locked(ptp, modes);
	if (ret)
		goto out_unlock;

	/* Save the request after any required live programming succeeds. */
	ptp->requested_config = normalized;
	rtlmq_ptp_config_to_kernel(&normalized, config);
	ret = 0;

out_unlock:
	mutex_unlock(&tp->control_lock);
	return ret;
}

int rtlmq_ptp_get_ts_info(struct net_device *ndev,
			  struct kernel_ethtool_ts_info *info)
{
	struct rtlmq_private *tp = netdev_priv(ndev);
	struct rtlmq_ptp *ptp;

	ethtool_op_get_ts_info(ndev, info);

	/*
	 * Netdev timestamp discovery runs inside an RCU read-side section and
	 * therefore cannot take control_lock. The PHC is registered before the
	 * netdev and freed only after unregister_netdev() drains these queries,
	 * so the pointer and clock identity stay valid during this callback.
	 */
	ptp = READ_ONCE(tp->ptp);
	if (!rtlmq_ptp_backend_matches(tp, ptp))
		return 0;

	info->so_timestamping |= SOF_TIMESTAMPING_TX_HARDWARE |
				 SOF_TIMESTAMPING_RAW_HARDWARE;
	if (ptp->backend->supports_rx)
		info->so_timestamping |= SOF_TIMESTAMPING_RX_HARDWARE;
	info->phc_index = ptp->clock ? ptp_clock_index(ptp->clock) : -1;
	info->phc_source = HWTSTAMP_SOURCE_NETDEV;
	info->phc_qualifier = HWTSTAMP_PROVIDER_QUALIFIER_PRECISE;
	info->tx_types = BIT(HWTSTAMP_TX_OFF) | BIT(HWTSTAMP_TX_ON);
	info->rx_filters = BIT(HWTSTAMP_FILTER_NONE);
	if (ptp->backend->supports_rx)
		info->rx_filters |= BIT(HWTSTAMP_FILTER_PTP_V2_EVENT) |
				    BIT(HWTSTAMP_FILTER_PTP_V2_L2_EVENT) |
				    BIT(HWTSTAMP_FILTER_PTP_V2_L4_EVENT) |
				    BIT(HWTSTAMP_FILTER_PTP_V2_SYNC) |
				    BIT(HWTSTAMP_FILTER_PTP_V2_L2_SYNC) |
				    BIT(HWTSTAMP_FILTER_PTP_V2_L4_SYNC) |
				    BIT(HWTSTAMP_FILTER_PTP_V2_DELAY_REQ) |
				    BIT(HWTSTAMP_FILTER_PTP_V2_L2_DELAY_REQ) |
				    BIT(HWTSTAMP_FILTER_PTP_V2_L4_DELAY_REQ);
	return 0;
}

static struct rtlmq_ptp *
rtlmq_ptp_alloc(struct rtlmq_private *tp,
		const struct rtlmq_ptp_backend_contract *backend)
{
	struct rtlmq_ptp *ptp;

	if (!backend)
		return ERR_PTR(-EOPNOTSUPP);
	ptp = kzalloc_obj(*ptp, GFP_KERNEL);
	if (!ptp)
		return ERR_PTR(-ENOMEM);

	ptp->tp = tp;
	ptp->backend = backend;
	rtlmq_ptp_clock_init(ptp);
	atomic_set(&ptp->hw_busy, 0);
	init_waitqueue_head(&ptp->hw_wait);
	rtlmq_ptp_request_init(ptp);
	INIT_DELAYED_WORK(&ptp->link_reconcile_work, rtlmq_ptp_link_reconcile_work);
	snprintf(ptp->info.name, sizeof(ptp->info.name), "%s-%s",
		 KBUILD_MODNAME, pci_name(tp->pdev));
	return ptp;
}

int rtlmq_ptp_restore_locked(struct rtlmq_private *tp)
{
	struct timespec64 now;
	struct rtlmq_ptp *ptp;
	int ret;

	if (!tp)
		return -EINVAL;
	lockdep_assert_held(&tp->control_lock);

	ptp = tp->ptp;
	if (!ptp)
		return 0;
	if (!rtlmq_ptp_backend_matches(tp, ptp))
		return -EOPNOTSUPP;
	if (!rtlmq_mmio_ordinary_authorized(tp))
		return -ENODEV;

	/* Withdraw admission before cancelling old requests and reprogramming. */
	smp_store_release(&ptp->active, false);
	smp_store_release(&ptp->active_modes, 0);
	rtlmq_ptp_tx_cancel_request(ptp, false);
	rtlmq_ptp_clear_rx_timestamp_scan(ptp);
	if (ptp->backend->clear_mac_timestamp_source) {
		/*
		 * CFG_METHOD_4/5 timestamps come from the PHY. Clear legacy
		 * MAC-source selection left by firmware or a previous driver.
		 */
		ret = rtlmq_mac_ocp_modify_checked(tp, RTLMQ_PTP_SOURCE_OCP,
						   RTLMQ_PTP_SOURCE_MAC, 0);
		if (ret)
			return ret;
	}
	ret = rtlmq_ptp_hw_acquire(ptp, false);
	if (ret)
		return ret;
	ret = rtlmq_ptp_phy_config_modes_owned(ptp, 0, 0, true);
	rtlmq_ptp_hw_release(ptp);
	if (ret)
		return ret;
	/* Admit PHC rebasing against the idle PHY; packet modes remain zero. */
	smp_store_release(&ptp->active, true);

	/*
	 * A device reset can clear the PHY free-running counter. Rebase it
	 * before enabling packet timestamp modes so callers do not see a
	 * reset-zero PHC.
	 */
	ktime_get_real_ts64(&now);
	ret = rtlmq_ptp_write_time_locked(ptp, &now);
	if (!ret)
		ret = rtlmq_ptp_program_frequency_locked(ptp, ptp->frequency_ppb);
	if (!ret)
		ret = rtlmq_ptp_apply_modes_locked(ptp,
						   rtlmq_ptp_config_modes(&ptp->requested_config));
	if (ret) {
		if (!rtlmq_ptp_hw_acquire(ptp, false)) {
			(void)rtlmq_ptp_phy_config_modes_owned(ptp, READ_ONCE(ptp->active_modes), 0,
							       false);
			rtlmq_ptp_hw_release(ptp);
		}
		/* Withdraw the partial restore from packet timestamp admission. */
		smp_store_release(&ptp->active_modes, 0);
		smp_store_release(&ptp->active, false);
		rtlmq_ptp_clear_rx_timestamp_scan(ptp);
	}
	return ret;
}

static void rtlmq_ptp_withdraw_locked(struct rtlmq_private *tp,
				      bool quiesce_hardware)
{
	struct rtlmq_ptp *ptp;
	u8 old_modes;

	if (!tp)
		return;
	lockdep_assert_held(&tp->control_lock);

	ptp = tp->ptp;
	if (!ptp)
		return;

	old_modes = READ_ONCE(ptp->active_modes);
	/*
	 * Withdraw admission from acquire readers. These stores do not drain
	 * users: cancel TX work below and serialize PHY access with hw_busy.
	 */
	smp_store_release(&ptp->active_modes, 0);
	smp_store_release(&ptp->active, false);
	rtlmq_ptp_tx_cancel_request(ptp, quiesce_hardware);
	if (quiesce_hardware && rtlmq_mmio_ordinary_authorized(tp) &&
	    !rtlmq_ptp_hw_acquire(ptp, false)) {
		(void)rtlmq_ptp_phy_config_modes_owned(ptp, old_modes, 0, false);
		rtlmq_ptp_hw_release(ptp);
	}
	rtlmq_ptp_clear_rx_timestamp_scan(ptp);
}

void rtlmq_ptp_quiesce_locked(struct rtlmq_private *tp)
{
	rtlmq_ptp_withdraw_locked(tp, true);
}

void rtlmq_ptp_freeze_locked(struct rtlmq_private *tp)
{
	/*
	 * Withdraw PTP after a frozen channel or uncertain programming.
	 * Stop asynchronous work without issuing further MMIO.
	 */
	rtlmq_ptp_withdraw_locked(tp, false);
}

int rtlmq_ptp_register(struct rtlmq_private *tp)
{
	const struct rtlmq_ptp_backend_contract *backend;
	struct rtlmq_ptp *ptp;
	int err;

	if (!tp || !tp->pdev || !tp->ndev)
		return -EINVAL;
	if (rtlmq_qualified_ptp_backend(tp) == RTLMQ_PTP_BACKEND_NONE)
		return 0;
	backend = rtlmq_ptp_backend_contract(rtlmq_qualified_ptp_backend(tp));
	if (!backend)
		return -EOPNOTSUPP;
	if (tp->ptp)
		return -EALREADY;

	ptp = rtlmq_ptp_alloc(tp, backend);
	if (IS_ERR(ptp))
		return PTR_ERR(ptp);

	ptp->clock = rtlmq_ptp_register_clock(tp, ptp);
	if (IS_ERR(ptp->clock)) {
		err = PTR_ERR(ptp->clock);
		kfree(ptp);
		return err;
	}

	/* Probe admits netdev users only after this instance is complete. */
	tp->ptp = ptp;
	/* CONFIG_PTP_1588_CLOCK=n is represented by the framework as NULL. */
	if (ptp->clock)
		netdev_info(tp->ndev, "registered PHC %d\n",
			    ptp_clock_index(ptp->clock));
	return 0;
}

void rtlmq_ptp_unregister(struct rtlmq_private *tp)
{
	struct rtlmq_ptp *ptp;

	if (!tp)
		return;

	mutex_lock(&tp->control_lock);
	ptp = tp->ptp;
	if (ptp) {
		rtlmq_ptp_quiesce_locked(tp);
		tp->ptp = NULL;
	}
	mutex_unlock(&tp->control_lock);
	if (!ptp)
		return;

	cancel_delayed_work_sync(&ptp->link_reconcile_work);
	if (ptp->clock)
		ptp_clock_unregister(ptp->clock);
	kfree(ptp);
}
