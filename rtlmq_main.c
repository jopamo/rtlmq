// SPDX-License-Identifier: GPL-2.0-only

#include <linux/etherdevice.h>
#include <linux/filter.h>
#include <linux/if_vlan.h>
#include <linux/ip.h>
#include <linux/kernel.h>
#include <linux/ipv6.h>
#include <linux/phy.h>
#include <linux/rtnetlink.h>
#include <net/sch_generic.h>
#include <net/xdp_sock_drv.h>

#include "rtlmq_hw.h"
#include "rtlmq_irq.h"
#include "rtlmq_lifecycle.h"
#include "rtlmq.h"
#include "rtlmq_runtime_accessors.h"
#include "rtlmq_runtime.h"
#include "rtlmq_rx.h"
#include "rtlmq_ring.h"
#include "rtlmq_regs.h"
#include "rtlmq_diag.h"
#include "rtlmq_ptp.h"
#include "rtlmq_tx.h"
#include "rtlmq_xdp.h"

static int rtlmq_open(struct net_device *ndev)
{
	struct rtlmq_private *tp = netdev_priv(ndev);
	int ret;

	ret = rtlmq_runtime_pm_netdev_open(tp);
	if (ret)
		return ret;
	mutex_lock(&tp->control_lock);
	/*
	 * Core has already closed the old administrative open, even if its
	 * teardown failed. Finish a recoverable retained transition before
	 * starting another generation; never take over PM/AER or terminal state.
	 */
	if (tp->platform_quiesce_owner == RTLMQ_PLATFORM_QUIESCE_NONE &&
	    (tp->state == RTLMQ_STATE_CLOSING ||
	     tp->state == RTLMQ_STATE_OPENING ||
	     tp->state == RTLMQ_STATE_RESETTING))
		ret = rtlmq_close_device(tp);
	if (!ret)
		ret = rtlmq_admit_control_change_locked(tp, true);
	if (!ret)
		ret = rtlmq_open_device(tp);
	mutex_unlock(&tp->control_lock);
	if (ret)
		rtlmq_runtime_pm_netdev_close(tp);
	if (!ret)
		netif_info(tp, ifup, ndev, "interface opened\n");

	return ret;
}

static int rtlmq_stop(struct net_device *ndev)
{
	struct rtlmq_private *tp = netdev_priv(ndev);
	int ret;

	mutex_lock(&tp->control_lock);
	ret = rtlmq_close_device(tp);
	mutex_unlock(&tp->control_lock);
	/*
	 * ndo_stop errors do not undo core close. The state/quiescence checks
	 * reject autosuspend of retained storage; this reference owns only the
	 * administrative open, not a failed teardown.
	 */
	rtlmq_runtime_pm_netdev_close(tp);
	if (!ret)
		netif_info(tp, ifdown, ndev, "interface closed\n");

	return ret;
}

static int rtlmq_eth_ioctl(struct net_device *ndev, struct ifreq *ifr, int cmd)
{
	struct rtlmq_private *tp = netdev_priv(ndev);
	int ret;

	mutex_lock(&tp->control_lock);
	ret = rtlmq_admit_control_change_locked(tp, false);
	if (!ret && !ndev->phydev)
		ret = -ENODEV;
	if (!ret)
		ret = phy_do_ioctl_running(ndev, ifr, cmd);
	mutex_unlock(&tp->control_lock);

	return ret;
}

struct rtlmq_mac_request {
	struct sockaddr address;
};

static int rtlmq_normalize_mac_request(struct net_device *ndev, void *addr,
				       struct rtlmq_mac_request *request)
{
	int ret;

	if (!addr || !request)
		return -EINVAL;

	ret = eth_prepare_mac_addr_change(ndev, addr);
	if (ret)
		return ret;

	request->address = *(const struct sockaddr *)addr;
	return 0;
}

static int rtlmq_set_rx_mode_async(struct net_device *ndev,
				   struct netdev_hw_addr_list *uc,
				   struct netdev_hw_addr_list *mc)
{
	struct rtlmq_private *tp = netdev_priv(ndev);
	struct rtlmq_rx_filter_plan plan;
	int ret;

	/*
	 * Core keeps these address-list snapshots valid during the callback.
	 * Build all filter values before taking control_lock or writing MMIO.
	 */
	ret = rtlmq_build_rx_filter_plan(ndev, uc, mc, &plan);
	if (ret)
		return ret;

	mutex_lock(&tp->control_lock);
	ret = rtlmq_apply_rx_filter_plan_locked(tp, &plan);
	mutex_unlock(&tp->control_lock);

	return ret;
}

static int rtlmq_set_mac_address(struct net_device *ndev, void *addr)
{
	struct rtlmq_private *tp = netdev_priv(ndev);
	struct rtlmq_mac_request request;
	u8 old_addr[ETH_ALEN];
	int rollback_ret;
	int pm_token;
	int ret;

	pm_token = rtlmq_runtime_pm_access_get(tp);
	if (pm_token < 0)
		return pm_token;
	mutex_lock(&tp->control_lock);

	ret = rtlmq_admit_control_change_locked(tp, true);
	if (ret)
		goto out_unlock;

	ret = rtlmq_normalize_mac_request(ndev, addr, &request);
	if (ret)
		goto out_unlock;

	ether_addr_copy(old_addr, ndev->dev_addr);
	ret = rtlmq_program_mac_address(tp, request.address.sa_data);
	if (ret) {
		/*
		 * The software address is still old because commit is deferred
		 * until the hardware readback succeeds. If a write was partially
		 * accepted, make one best-effort hardware rollback before
		 * returning the failure to the netdev core.
		 */
		rollback_ret = rtlmq_program_mac_address(tp, old_addr);
		if (rollback_ret)
			netdev_err(ndev,
				   "failed to restore station address after MAC change error: %d\n",
				   rollback_ret);
		goto out_unlock;
	}
	eth_commit_mac_addr_change(ndev, &request.address);

out_unlock:
	mutex_unlock(&tp->control_lock);
	rtlmq_runtime_pm_access_put(tp, pm_token);

	return ret;
}

static netdev_features_t rtlmq_fix_features(struct net_device *ndev,
					    netdev_features_t features)
{
	struct rtlmq_private *tp = netdev_priv(ndev);
	unsigned int mtu = READ_ONCE(ndev->mtu);

	/*
	 * TSO requires SG and checksum offload for the matching IP version.
	 * ndo_fix_features() receives the resulting wanted set, not the original
	 * request mask. Clear unsupported combinations here; core preserves
	 * wanted_features separately.
	 *
	 * Match RTL8125 restrictions: jumbo MTUs retain checksum and SG but
	 * disable hardware TSO. The descriptor MSS field is also limited to
	 * 11 bits.
	 */
	if (mtu > ETH_DATA_LEN || mtu > RTLMQ_TX_DESC_MSS_MASK ||
	    tp->cbs_tso_constrained)
		features &= ~NETIF_F_ALL_TSO;

	if (!(features & NETIF_F_SG))
		features &= ~(NETIF_F_TSO | NETIF_F_TSO6);

	if (!(features & NETIF_F_IP_CSUM))
		features &= ~NETIF_F_TSO;

	if (!(features & NETIF_F_IPV6_CSUM))
		features &= ~NETIF_F_TSO6;

	if (!rtlmq_qualified_supports_vlan_ctag_tx(tp))
		features &= ~NETIF_F_HW_VLAN_CTAG_TX;
	if (!rtlmq_qualified_supports_vlan_ctag_rx(tp))
		features &= ~NETIF_F_HW_VLAN_CTAG_RX;
	if (!rtlmq_qualified_supports_rx_fcs(tp))
		features &= ~NETIF_F_RXFCS;
	if (!rtlmq_qualified_supports_rx_all(tp))
		features &= ~NETIF_F_RXALL;

	return features;
}

void rtlmq_sync_vlan_features(struct net_device *ndev,
			      netdev_features_t features)
{
	netdev_features_t vlan_features = NETIF_F_SG;

	/*
	 * With hardware CTAG insertion disabled, the VLAN upper pushes the tag
	 * after ndo_features_check. RTL8125B cannot checksum or segment through
	 * that inline tag in this validated configuration, so make the upper
	 * stack finish those operations in software before it pushes the tag.
	 */
	if (features & NETIF_F_HW_VLAN_CTAG_TX)
		vlan_features |= NETIF_F_IP_CSUM | NETIF_F_IPV6_CSUM |
				 NETIF_F_TSO | NETIF_F_TSO6;

	ndev->vlan_features = vlan_features;
}

static int rtlmq_set_features(struct net_device *ndev,
			      netdev_features_t features)
{
	struct rtlmq_private *tp = netdev_priv(ndev);
	int ret;

	features = rtlmq_fix_features(ndev, features);
	mutex_lock(&tp->control_lock);
	/*
	 * Internal feature replay runs while PROBED and unpublished, including
	 * a PM/PCI owner's reopen. Ordinary setters retain external admission.
	 */
	if (tp->feature_sync_error)
		ret = tp->state == RTLMQ_STATE_PROBED ? 0 : -EBUSY;
	else
		ret = rtlmq_admit_control_change_locked(tp, true);
	if (ret)
		goto out_unlock;
	ret = rtlmq_sync_rx_config_features(tp, features);
	if (ret)
		goto out_unlock;
	ret = rtlmq_sync_cplus_cmd_features(tp, features);
	if (ret) {
		int rx_ret, cp_ret;

		/*
		 * The core retains the old feature set on failure. CPlusCmd
		 * readback may fail after RxConfig was already changed; attempt
		 * to restore both register owners to the core's retained policy.
		 */
		rx_ret = rtlmq_sync_rx_config_features(tp, ndev->features);
		cp_ret = rtlmq_sync_cplus_cmd_features(tp, ndev->features);
		if (rx_ret || cp_ret)
			netdev_warn(ndev,
				    "feature rollback incomplete (RX: %d, CPlusCmd: %d)\n",
				    rx_ret, cp_ret);
		goto out_unlock;
	}
	/*
	 * RXFCS, RXALL, and CTAG-VLAN toggles change receiver filtering or
	 * descriptor interpretation only. They do not change RX allocations,
	 * which already reserve FCS space and skb_shared_info tailroom.
	 */
	rtlmq_sync_rx_fast_features(tp, features);
	rtlmq_sync_vlan_features(ndev, features);
out_unlock:
	if (ret && tp->feature_sync_error)
		*tp->feature_sync_error = ret;
	mutex_unlock(&tp->control_lock);

	return ret;
}

int rtlmq_sync_features_locked(struct rtlmq_private *tp)
{
	bool constrained = rtlmq_tc_cbs_enabled(&tp->tc.requested);
	bool old = tp->cbs_tso_constrained;
	int ret = 0;

	ASSERT_RTNL();
	lockdep_assert_held(&tp->control_lock);
	if (tp->state != RTLMQ_STATE_PROBED || tp->feature_sync_error ||
	    READ_ONCE(tp->runtime_published))
		return -EBUSY;
	if (old == constrained)
		return 0;
	/*
	 * Core retains wanted_features, including requests made while CBS hid
	 * TSO. Do not save/overwrite gso_max_segs or reimplement core filtering.
	 * RTNL excludes another feature setter while its callback takes our lock.
	 */
	tp->feature_sync_error = &ret;
	tp->cbs_tso_constrained = constrained;
	mutex_unlock(&tp->control_lock);
	netdev_update_features(tp->ndev);
	if (!ret && constrained) {
		/*
		 * qdisc gso_skb entries bypass feature validation on retry.
		 * Drain old validators, then discard queued TX before admitting
		 * the single-frame profile. Driver teardown alone drains only
		 * packets already handed to the driver.
		 */
		synchronize_net();
		qdisc_reset_all_tx_gt(tp->ndev, 0);
	}
	mutex_lock(&tp->control_lock);
	tp->feature_sync_error = NULL;
	if (ret)
		tp->cbs_tso_constrained = old;
	return ret;
}

static netdev_features_t rtlmq_features_check(struct sk_buff *skb,
					      struct net_device *ndev,
					      netdev_features_t features)
{
	(void)ndev;

	/*
	 * The core pushes unsupported metadata tags after this check.
	 * Neither checksum nor TSO descriptors support an inline VLAN header.
	 */
	if (eth_type_vlan(skb->protocol) ||
	    (skb_vlan_tag_present(skb) &&
	     !vlan_hw_offload_capable(features, skb->vlan_proto)))
		features &= ~(NETIF_F_CSUM_MASK | NETIF_F_GSO_MASK);

	if (skb_is_gso(skb)) {
		u32 gso_size = skb_shinfo(skb)->gso_size;
		u32 gso_type = skb_shinfo(skb)->gso_type;

		if (!(gso_type & (SKB_GSO_TCPV4 | SKB_GSO_TCPV6)))
			features &= ~NETIF_F_GSO_MASK;
		if (gso_size > RTLMQ_TX_DESC_MSS_MASK)
			features &= ~NETIF_F_GSO_MASK;
		if (skb_transport_offset(skb) >
		    RTLMQ_TX_DESC_GSO_TCPHO_MAX)
			features &= ~NETIF_F_GSO_MASK;
	}

	if (skb->ip_summed == CHECKSUM_PARTIAL) {
		__be16 proto = vlan_get_protocol(skb);

		/*
		 * RTL8125B can checksum the Ethernet padding as transport data.
		 * Let the stack finish short-frame checksums before TX pads.
		 */
		if (!skb_is_gso(skb) && skb->len < ETH_ZLEN)
			features &= ~NETIF_F_CSUM_MASK;

		if (proto == htons(ETH_P_IP)) {
			if (ip_hdr(skb)->protocol != IPPROTO_TCP &&
			    ip_hdr(skb)->protocol != IPPROTO_UDP)
				features &= ~NETIF_F_CSUM_MASK;
		} else if (proto == htons(ETH_P_IPV6)) {
			u8 nexthdr = ipv6_hdr(skb)->nexthdr;

			if (nexthdr != IPPROTO_TCP && nexthdr != IPPROTO_UDP)
				features &= ~NETIF_F_CSUM_MASK;
		} else {
			features &= ~NETIF_F_CSUM_MASK;
		}
		if (proto == htons(ETH_P_IPV6) &&
		    (!skb_transport_offset(skb) ||
		     skb_transport_offset(skb) >
			     RTLMQ_TX_DESC_TCPHO_MAX))
			features &= ~NETIF_F_CSUM_MASK;
	}

	return vlan_features_check(skb, features);
}

static void rtlmq_get_stats64(struct net_device *ndev,
			      struct rtnl_link_stats64 *stats)
{
	struct rtlmq_private *tp = netdev_priv(ndev);
	struct rtlmq_queue_stats queue_stats;

	memset(stats, 0, sizeof(*stats));

	rtlmq_get_queue_stats_totals(tp, &queue_stats);
	stats->rx_packets = queue_stats.rx_packets;
	stats->rx_bytes = queue_stats.rx_bytes;
	stats->rx_dropped = queue_stats.rx_drops;
	stats->rx_errors = queue_stats.rx_errors;
	stats->rx_crc_errors = queue_stats.rx_crc_errors;
	stats->rx_length_errors = queue_stats.rx_length_errors;
	stats->tx_packets = queue_stats.tx_packets;
	stats->tx_bytes = queue_stats.tx_bytes;
	stats->tx_dropped = queue_stats.tx_drops;

	/*
	 * ndo_get_stats64() may run under the netdev core's RCU read-side
	 * section. It must remain non-sleeping and must not issue the tally
	 * DMA/MMIO transaction. Locked ethtool callbacks read hardware tally
	 * counters; this callback uses the queue snapshots above.
	 */
}

const struct net_device_ops rtlmq_netdev_ops = {
	.ndo_open		= rtlmq_open,
	.ndo_stop		= rtlmq_stop,
	.ndo_change_mtu		= rtlmq_change_mtu,
	.ndo_bpf		= rtlmq_xdp_setup,
	.ndo_xdp_xmit		= rtlmq_xdp_xmit,
	.ndo_xsk_wakeup		= rtlmq_xsk_wakeup,
	.ndo_eth_ioctl		= rtlmq_eth_ioctl,
	.ndo_hwtstamp_get	= rtlmq_ptp_hwtstamp_get,
	.ndo_hwtstamp_set	= rtlmq_ptp_hwtstamp_set,
	.ndo_set_rx_mode_async	= rtlmq_set_rx_mode_async,
	.ndo_setup_tc		= rtlmq_setup_tc,
	.ndo_start_xmit		= rtlmq_start_xmit,
	.ndo_fix_features	= rtlmq_fix_features,
	.ndo_set_features	= rtlmq_set_features,
	.ndo_features_check	= rtlmq_features_check,
	.ndo_get_stats64	= rtlmq_get_stats64,
	.ndo_tx_timeout		= rtlmq_tx_timeout,
	.ndo_set_mac_address	= rtlmq_set_mac_address,
	.ndo_validate_addr	= eth_validate_addr,
#ifdef CONFIG_NET_POLL_CONTROLLER
	.ndo_poll_controller	= rtlmq_poll_controller,
#endif
};
