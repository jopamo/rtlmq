// SPDX-License-Identifier: GPL-2.0-only

#include <linux/bitfield.h>
#include <linux/ethtool.h>
#include <linux/if_ether.h>
#include <linux/if_vlan.h>
#include <linux/ip.h>
#include <linux/jiffies.h>
#include <linux/ktime.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/net_tstamp.h>
#include <linux/overflow.h>
#include <linux/ptp_classify.h>
#include <linux/skbuff.h>
#include <linux/udp.h>
#include <linux/unaligned.h>
#include <linux/wait.h>
#include <linux/workqueue.h>
#include <net/xdp_sock_drv.h>

#include "rtlmq.h"
#include "rtlmq_hw.h"
#include "rtlmq_ptp.h"
#include "rtlmq_regs.h"
#include "rtlmq_xdp.h"

#define RTLMQ_PTP_TX_TIMEOUT_MS		1000
#define RTLMQ_PTP_TX_RETRY_MS		1
#define RTLMQ_PTP_TX_DRAIN_MS		100
#define RTLMQ_PTP_IPV4_FRAGMENT_MASK	0x3fff
#define RTLMQ_PTP_PACKET_HEADER_LEN \
	(ETH_HLEN + 2 * VLAN_HLEN + 60 + sizeof(struct udphdr) + \
	 sizeof(struct ptp_header))

struct rtlmq_ptp_xsk_finish {
	struct xsk_buff_pool *pool;
	struct xsk_tx_metadata_compl meta;
	const struct xsk_tx_metadata_ops *ops;
	u64 timestamp;
	u32 descs;
	bool valid;
};

DEFINE_STATIC_KEY_FALSE(rtlmq_ptp_rx_scan_key);

static void rtlmq_ptp_tx_work(struct work_struct *work);

static bool rtlmq_ptp_active_acquire(const struct rtlmq_ptp *ptp)
{
	/* Pairs with active publication in rtlmq_ptp_restore_locked(). */
	return smp_load_acquire(&ptp->active);
}

static u8 rtlmq_ptp_active_modes_acquire(const struct rtlmq_ptp *ptp)
{
	/* Pairs with mode publication after detector programming. */
	return smp_load_acquire(&ptp->active_modes);
}

static bool rtlmq_ptp_rx_scan_acquire(const struct rtlmq_ptp *ptp)
{
	/* Pairs with scan publication before RX mode activation. */
	return smp_load_acquire(&ptp->rx_timestamp_scan_required);
}

static bool rtlmq_ptp_tx_pending_locked(const struct rtlmq_ptp *ptp)
{
	return ptp->tx_skb || ptp->tx_xsk_pool;
}

static void rtlmq_ptp_advance_tx_generation_locked(struct rtlmq_ptp *ptp)
{
	lockdep_assert_held(&ptp->tx_lock);
	/* Zero denotes an ordinary EOP without a timestamp reservation. */
	ptp->tx_generation++;
	if (!ptp->tx_generation)
		ptp->tx_generation++;
}

static bool rtlmq_ptp_tx_pending(struct rtlmq_ptp *ptp)
{
	unsigned long flags;
	bool pending;

	spin_lock_irqsave(&ptp->tx_lock, flags);
	pending = rtlmq_ptp_tx_pending_locked(ptp);
	spin_unlock_irqrestore(&ptp->tx_lock, flags);
	return pending;
}

static void rtlmq_ptp_clear_xsk_locked(struct rtlmq_ptp *ptp)
{
	ptp->tx_xsk_pool = NULL;
	memset(&ptp->tx_xsk_meta, 0, sizeof(ptp->tx_xsk_meta));
	ptp->tx_xsk_ops = NULL;
	ptp->tx_xsk_timestamp = 0;
	ptp->tx_xsk_descs = 0;
	ptp->tx_xsk_completed_descs = 0;
	ptp->tx_xsk_dma_complete = false;
	ptp->tx_xsk_timestamp_ready = false;
	ptp->tx_xsk_timestamp_valid = false;
}

static void rtlmq_ptp_take_xsk_locked(struct rtlmq_ptp *ptp,
				      struct rtlmq_ptp_xsk_finish *finish,
				      bool deliver_timestamp)
{
	finish->pool = ptp->tx_xsk_pool;
	finish->meta = ptp->tx_xsk_meta;
	finish->ops = ptp->tx_xsk_ops;
	finish->descs = ptp->tx_xsk_completed_descs;
	finish->timestamp = deliver_timestamp &&
			    ptp->tx_xsk_timestamp_valid ?
				    ptp->tx_xsk_timestamp : 0;
	finish->valid = true;
	if (deliver_timestamp && ptp->tx_xsk_timestamp_valid)
		ptp->tx_timestamps++;
	else
		ptp->tx_timestamp_missed++;
	rtlmq_ptp_clear_xsk_locked(ptp);
}

static void rtlmq_ptp_finish_xsk_locked(struct rtlmq_ptp *ptp,
					struct rtlmq_ptp_xsk_finish *finish)
{
	lockdep_assert_held(&ptp->tx_lock);
	if (!finish->valid)
		return;

	/* Keep later EOPs behind metadata publication and positional CQ release. */
	if (finish->ops)
		xsk_tx_metadata_complete(&finish->meta, finish->ops, &finish->timestamp);
	if (finish->descs)
		rtlmq_xsk_tx_completed(finish->pool, finish->descs);
}

static void rtlmq_ptp_complete_xsk_locked(struct rtlmq_ptp *ptp,
					  struct rtlmq_ptp_xsk_finish *finish)
{
	rtlmq_ptp_take_xsk_locked(ptp, finish, true);
	rtlmq_ptp_finish_xsk_locked(ptp, finish);
	rtlmq_ptp_advance_tx_generation_locked(ptp);
}

int rtlmq_ptp_tx_start(struct rtlmq_ptp *ptp, u8 modes)
{
	struct rtlmq_private *tp = ptp->tp;
	unsigned long flags;
	int ret;

	if (!(modes & RTLMQ_PTP_MODE_TX))
		return 0;
	ret = rtlmq_ptp_hw_acquire(ptp, false);
	if (ret)
		return ret;
	ret = rtlmq_ptp_clear_tx_status_owned(ptp);
	if (ret)
		goto out_release;
	spin_lock_irqsave(&ptp->tx_lock, flags);
	if (rtlmq_ptp_active_acquire(ptp) &&
	    !rtlmq_ptp_tx_pending_locked(ptp) &&
	    rtlmq_mmio_ordinary_authorized(tp))
		ptp->tx_accepting = true;
	spin_unlock_irqrestore(&ptp->tx_lock, flags);

out_release:
	rtlmq_ptp_hw_release(ptp);
	return ret;
}

void rtlmq_ptp_tx_cancel_request(struct rtlmq_ptp *ptp, bool clear_hardware)
{
	struct rtlmq_ptp_xsk_finish xsk_finish = {};
	struct rtlmq_private *tp = ptp->tp;
	struct sk_buff *skb;
	unsigned long flags;

	/*
	 * Block new requests and remove the saved reference before cancelling
	 * work. A running callback may finish its register read, but the
	 * generation check prevents it from reporting or requeuing stale work.
	 */
	spin_lock_irqsave(&ptp->tx_lock, flags);
	ptp->tx_accepting = false;
	skb = ptp->tx_skb;
	ptp->tx_skb = NULL;
	if (ptp->tx_xsk_pool) {
		if (ptp->tx_xsk_dma_complete) {
			rtlmq_ptp_take_xsk_locked(ptp, &xsk_finish, false);
			rtlmq_ptp_finish_xsk_locked(ptp, &xsk_finish);
		} else {
			ptp->tx_timestamp_missed++;
			rtlmq_ptp_clear_xsk_locked(ptp);
		}
	}
	rtlmq_ptp_advance_tx_generation_locked(ptp);
	if (skb)
		ptp->tx_timestamp_missed++;
	spin_unlock_irqrestore(&ptp->tx_lock, flags);

	wake_up_all(&ptp->tx_idle_wait);
	cancel_delayed_work_sync(&ptp->tx_work);
	if (clear_hardware && rtlmq_mmio_ordinary_authorized(tp) &&
	    !rtlmq_ptp_hw_acquire(ptp, false)) {
		(void)rtlmq_ptp_clear_tx_status_owned(ptp);
		rtlmq_ptp_hw_release(ptp);
	}
	dev_kfree_skb_any(skb);
}

void rtlmq_ptp_require_rx_timestamp_scan(struct rtlmq_ptp *ptp)
{
	if (ptp->rx_timestamp_scan_required)
		return;

	/*
	 * Enable generic RX parsing before enabling PHY ingress capture. The
	 * static key skips per-device scan checks while no device needs RX
	 * timestamps.
	 */
	static_branch_inc(&rtlmq_ptp_rx_scan_key);
	/* Publish scan admission only after the static key is enabled. */
	smp_store_release(&ptp->rx_timestamp_scan_required, true);
}

void rtlmq_ptp_clear_rx_timestamp_scan(struct rtlmq_ptp *ptp)
{
	if (!ptp->rx_timestamp_scan_required)
		return;

	/* Withdraw scan admission before disabling the static key. */
	smp_store_release(&ptp->rx_timestamp_scan_required, false);
	static_branch_dec(&rtlmq_ptp_rx_scan_key);
}

void rtlmq_ptp_get_ts_stats(struct net_device *ndev,
			    struct ethtool_ts_stats *stats)
{
	struct rtlmq_private *tp = netdev_priv(ndev);
	struct rtlmq_ptp *ptp = tp ? READ_ONCE(tp->ptp) : NULL;
	unsigned long flags;

	if (!ptp || !stats)
		return;

	spin_lock_irqsave(&ptp->tx_lock, flags);
	stats->pkts = ptp->tx_timestamps;
	stats->lost = ptp->tx_timestamp_missed;
	stats->err = ptp->tx_timestamp_skipped;
	spin_unlock_irqrestore(&ptp->tx_lock, flags);
}

static bool rtlmq_ptp_tx_enabled(const struct rtlmq_ptp *ptp)
{
	return ptp && rtlmq_ptp_active_acquire(ptp) &&
	       (rtlmq_ptp_active_modes_acquire(ptp) & RTLMQ_PTP_MODE_TX);
}

static void rtlmq_ptp_reopen_tx_admission(struct rtlmq_ptp *ptp)
{
	struct rtlmq_private *tp = ptp->tp;
	unsigned long flags;

	spin_lock_irqsave(&ptp->tx_lock, flags);
	/* A pending request still excludes reservations after admission reopens. */
	if (rtlmq_ptp_active_acquire(ptp) &&
	    (rtlmq_ptp_active_modes_acquire(ptp) & RTLMQ_PTP_MODE_TX) &&
	    rtlmq_mmio_ordinary_authorized(tp))
		ptp->tx_accepting = true;
	spin_unlock_irqrestore(&ptp->tx_lock, flags);
}

int rtlmq_ptp_pause_datapath_locked(struct rtlmq_private *tp, bool force)
{
	struct rtlmq_ptp *ptp;
	unsigned long flags;
	int rollback_ret;
	u8 modes;
	int ret;
	bool pending;

	if (!tp)
		return -EINVAL;
	lockdep_assert_held(&tp->control_lock);

	ptp = tp->ptp;
	if (!ptp)
		return 0;

	/*
	 * Live ring/XSK replacement asserts global MAC StopReq without resetting
	 * the PHC. Block new timestamp reservations, then drain the existing
	 * request while IRQ/NAPI service can still finish it.
	 *
	 * StopReq disrupts the RTL8125B packet detector even when its registers
	 * still read enabled. Disable detection before StopReq and restore it
	 * afterward, without changing the PHY path or issuing A640.
	 */
	spin_lock_irqsave(&ptp->tx_lock, flags);
	modes = rtlmq_ptp_active_modes_acquire(ptp);
	if (!rtlmq_ptp_active_acquire(ptp) || !modes) {
		spin_unlock_irqrestore(&ptp->tx_lock, flags);
		return 0;
	}
	if (!rtlmq_mmio_ordinary_authorized(tp)) {
		spin_unlock_irqrestore(&ptp->tx_lock, flags);
		return -ENODEV;
	}
	ptp->tx_accepting = false;
	pending = rtlmq_ptp_tx_pending_locked(ptp);
	spin_unlock_irqrestore(&ptp->tx_lock, flags);

	if (pending) {
		mod_delayed_work(system_highpri_wq, &ptp->tx_work, 0);
		if (!wait_event_timeout(ptp->tx_idle_wait, !rtlmq_ptp_tx_pending(ptp),
					msecs_to_jiffies(RTLMQ_PTP_TX_DRAIN_MS))) {
			if (force) {
				rtlmq_ptp_tx_cancel_request(ptp, true);
			} else {
				/*
				 * Attach has not committed. Preserve the pending
				 * request and allow reservations again.
				 */
				rtlmq_ptp_reopen_tx_admission(ptp);
				return -EBUSY;
			}
		}
	}

	/*
	 * tx_accepting is false. Wait for a callback that cleared the last
	 * request but is still delivering its result before stopping the MAC.
	 */
	cancel_delayed_work_sync(&ptp->tx_work);
	ret = rtlmq_ptp_hw_acquire(ptp, false);
	if (ret)
		goto err_reopen_admission;
	ret = rtlmq_ptp_phy_live_pause_owned(ptp, modes);
	if (ret) {
		rollback_ret = rtlmq_ptp_phy_live_resume_owned(ptp, modes);
		rtlmq_ptp_hw_release(ptp);
		if (rollback_ret)
			return rollback_ret;
		goto err_reopen_admission;
	}
	rtlmq_ptp_hw_release(ptp);
	return 0;

err_reopen_admission:
	if (!force)
		rtlmq_ptp_reopen_tx_admission(ptp);
	return ret;
}

int rtlmq_ptp_resume_datapath_locked(struct rtlmq_private *tp)
{
	struct rtlmq_ptp *ptp;
	u8 modes;
	int ret;

	if (!tp)
		return -EINVAL;
	lockdep_assert_held(&tp->control_lock);

	/*
	 * Reset/PM restore rebases the PHC; do not use it for a live pause.
	 * After StopReq clears, restore packet detection on the unchanged PHY
	 * path and allow TX timestamp requests again.
	 */
	ptp = tp->ptp;
	if (!ptp || !rtlmq_ptp_active_acquire(ptp))
		return 0;

	modes = rtlmq_ptp_active_modes_acquire(ptp);
	ret = rtlmq_ptp_hw_acquire(ptp, false);
	if (ret)
		return ret;
	ret = rtlmq_ptp_phy_live_resume_owned(ptp, modes);
	rtlmq_ptp_hw_release(ptp);
	if (ret)
		return ret;

	ret = rtlmq_ptp_tx_start(ptp, modes);
	return ret;
}

bool rtlmq_ptp_rx_timestamp_scan_required(const struct rtlmq_private *tp)
{
	struct rtlmq_ptp *ptp = tp ? READ_ONCE(tp->ptp) : NULL;

	return ptp && rtlmq_ptp_rx_scan_acquire(ptp);
}

static bool rtlmq_ptp_rx_enabled(const struct rtlmq_private *tp)
{
	struct rtlmq_ptp *ptp = tp ? READ_ONCE(tp->ptp) : NULL;

	if (!ptp || !ptp->backend->supports_rx)
		return false;
	/* Pairs with mode publication after detector programming. */
	return rtlmq_ptp_active_acquire(ptp) &&
	       (rtlmq_ptp_active_modes_acquire(ptp) & RTLMQ_PTP_MODE_RX);
}

static int rtlmq_ptp_message_type(const void *packet, size_t linear_len,
				  size_t len, u8 *message_type)
{
	static const u8 minimum_event_len[] = { 44, 44, 54, 54 };
	unsigned int offset = ETH_HLEN;
	unsigned int vlan_depth = 0;
	const u8 *data = packet;
	unsigned int ihl;
	u16 protocol;
	u16 fragment;
	u16 length;
	u8 type;

	linear_len = min(linear_len, len);
	if (!data || !message_type || linear_len < ETH_HLEN)
		return -EINVAL;

	protocol = get_unaligned_be16(data + 2 * ETH_ALEN);
	while (protocol == ETH_P_8021Q || protocol == ETH_P_8021AD) {
		if (++vlan_depth > 2 || linear_len < offset + VLAN_HLEN)
			return -EINVAL;
		protocol = get_unaligned_be16(data + offset + 2);
		offset += VLAN_HLEN;
	}

	if (protocol == ETH_P_1588) {
		/* PTP header follows the Ethernet/VLAN header directly. */
	} else if (protocol == ETH_P_IP) {
		if (linear_len < offset + sizeof(struct iphdr) ||
		    (data[offset] >> 4) != 4)
			return -EINVAL;
		ihl = (data[offset] & 0x0f) * 4;
		if (ihl < sizeof(struct iphdr) ||
		    linear_len < offset + ihl + sizeof(struct udphdr) ||
		    data[offset + offsetof(struct iphdr, protocol)] !=
			    IPPROTO_UDP)
			return -EINVAL;
		length = get_unaligned_be16(data + offset + offsetof(struct iphdr, tot_len));
		if (length < ihl + sizeof(struct udphdr) ||
		    length > len - offset)
			return -EINVAL;
		len = offset + length;
		fragment = get_unaligned_be16(data + offset + offsetof(struct iphdr, frag_off));
		if (fragment & RTLMQ_PTP_IPV4_FRAGMENT_MASK)
			return -ENODATA;
		offset += ihl;
		if (get_unaligned_be16(data + offset + offsetof(struct udphdr, dest)) !=
		    PTP_EV_PORT)
			return -ENODATA;
		length = get_unaligned_be16(data + offset + offsetof(struct udphdr, len));
		if (length < sizeof(struct udphdr) || length > len - offset)
			return -EINVAL;
		len = offset + length;
		offset += sizeof(struct udphdr);
	} else {
		return -ENODATA;
	}

	if (min(linear_len, len) < offset + sizeof(struct ptp_header))
		return -EINVAL;
	if (data[offset + 1] != 2)
		return -ENODATA;
	type = data[offset] & 0x0f;
	if (type >= ARRAY_SIZE(minimum_event_len))
		return -ENODATA;
	length = get_unaligned_be16(data + offset + offsetof(struct ptp_header, message_length));
	if (length < minimum_event_len[type] || length > len - offset)
		return -EINVAL;
	*message_type = type;
	return 0;
}

static int rtlmq_ptp_skb_message_type(const struct sk_buff *skb,
				      u8 *message_type)
{
	u8 header[RTLMQ_PTP_PACKET_HEADER_LEN];
	unsigned int copied = min_t(unsigned int, skb->len, sizeof(header));

	if (skb_is_gso(skb) || skb_copy_bits(skb, 0, header, copied))
		return -EINVAL;
	return rtlmq_ptp_message_type(header, copied, skb->len, message_type);
}

static int rtlmq_ptp_xsk_message_type(struct xsk_buff_pool *pool,
				      const struct xdp_desc *descs,
				      u16 num_descs, u8 *message_type)
{
	u8 header[RTLMQ_PTP_PACKET_HEADER_LEN];
	unsigned int copied = 0, len = 0;
	unsigned int i;

	if (!descs || !num_descs || num_descs > RTLMQ_XSK_MAX_SEGS)
		return -EINVAL;
	/* The publisher has validated every descriptor's UMEM address range. */
	for (i = 0; i < num_descs; i++) {
		unsigned int copy = min_t(unsigned int, descs[i].len,
					 sizeof(header) - copied);

		if (check_add_overflow(len, descs[i].len, &len))
			return -EINVAL;
		if (copy) {
			memcpy(header + copied,
			       xsk_buff_raw_get_data(pool, descs[i].addr), copy);
			copied += copy;
		}
	}
	return rtlmq_ptp_message_type(header, copied, len, message_type);
}

int rtlmq_ptp_rx_timestamp(struct rtlmq_private *tp, const void *packet,
			   size_t linear_len, size_t len, ktime_t *timestamp)
{
	struct rtlmq_ptp *ptp = tp ? READ_ONCE(tp->ptp) : NULL;
	u8 message_type;
	u16 command;
	int ret;

	if (!ptp || !timestamp || !rtlmq_ptp_rx_enabled(tp))
		return -ENODATA;
	ret = rtlmq_ptp_message_type(packet, linear_len, len, &message_type);
	if (ret)
		return ret;
	ret = rtlmq_ptp_hw_acquire(ptp, true);
	if (ret)
		return ret;

	command = RTLMQ_PHY_PTP_TRX_RX_SELECT |
		  (message_type << 2) | RTLMQ_PHY_PTP_TRX_READ;
	ret = rtlmq_ptp_phy_trx_command(ptp, command, true);
	if (!ret)
		ret = rtlmq_ptp_phy_read_timestamp(ptp, timestamp);
	rtlmq_ptp_hw_release(ptp);
	return ret;
}

bool rtlmq_ptp_tx_prepare(struct rtlmq_private *tp, struct sk_buff *skb,
			  struct rtlmq_tx_completion *completion)
{
	struct rtlmq_ptp *ptp = tp ? READ_ONCE(tp->ptp) : NULL;
	struct sk_buff *held = NULL;
	bool release_held = false;
	bool reserved = false;
	unsigned long flags;
	u32 generation = 0;
	u8 message_type;
	int ret;

	if (!skb)
		return false;

	if (ptp && completion &&
	    skb_shinfo(skb)->tx_flags & SKBTX_HW_TSTAMP &&
	    rtlmq_ptp_tx_enabled(ptp)) {
		ret = rtlmq_ptp_skb_message_type(skb, &message_type);
		if (!ret)
			ret = rtlmq_ptp_hw_acquire(ptp, true);
		if (ret) {
			spin_lock_irqsave(&ptp->tx_lock, flags);
			ptp->tx_timestamp_skipped++;
			spin_unlock_irqrestore(&ptp->tx_lock, flags);
			goto out_sw_timestamp;
		}
		spin_lock_irqsave(&ptp->tx_lock, flags);
		if (ptp->tx_accepting &&
		    !rtlmq_ptp_tx_pending_locked(ptp) &&
		    rtlmq_ptp_tx_enabled(ptp)) {
			held = skb_get(skb);
			ptp->tx_skb = held;
			ptp->tx_message_type = message_type;
			ptp->tx_deadline =
				jiffies +
				msecs_to_jiffies(RTLMQ_PTP_TX_TIMEOUT_MS);
			rtlmq_ptp_advance_tx_generation_locked(ptp);
			generation = ptp->tx_generation;
		} else {
			ptp->tx_timestamp_skipped++;
		}
		spin_unlock_irqrestore(&ptp->tx_lock, flags);

		ret = held ? rtlmq_ptp_clear_tx_status_owned(ptp) : 0;
		spin_lock_irqsave(&ptp->tx_lock, flags);
		if (held && !ret && ptp->tx_skb == held &&
		    ptp->tx_generation == generation &&
		    rtlmq_ptp_tx_enabled(ptp)) {
			completion->timestamp_generation = generation;
			completion->flags |= RTLMQ_TX_COMPLETIONF_HW_TSTAMP;
			skb_shinfo(skb)->tx_flags |= SKBTX_IN_PROGRESS;
			reserved = true;
		} else if (held && ptp->tx_skb == held &&
			   ptp->tx_generation == generation) {
			ptp->tx_skb = NULL;
			rtlmq_ptp_advance_tx_generation_locked(ptp);
			ptp->tx_timestamp_skipped++;
			release_held = true;
		}
		spin_unlock_irqrestore(&ptp->tx_lock, flags);
		rtlmq_ptp_hw_release(ptp);

		if (reserved)
			mod_delayed_work(system_highpri_wq, &ptp->tx_work,
					 msecs_to_jiffies(RTLMQ_PTP_TX_RETRY_MS));
		else if (release_held)
			dev_kfree_skb_any(held);
	}

out_sw_timestamp:
	/* Preserve the stack's independent software/PHY timestamp hook. */
	skb_tx_timestamp(skb);
	return reserved;
}

bool rtlmq_ptp_xsk_tx_prepare(struct rtlmq_private *tp,
			      struct xsk_buff_pool *pool,
			      const struct xdp_desc *descs, u16 num_descs,
			      struct xsk_tx_metadata *meta,
			      const struct xsk_tx_metadata_ops *ops,
			      u32 *generation)
{
	struct rtlmq_ptp *ptp = tp ? READ_ONCE(tp->ptp) : NULL;
	u32 reserved_generation = 0;
	bool reserved = false;
	unsigned long flags;
	u8 message_type;
	int ret;

	if (generation)
		*generation = 0;
	if (!ptp || !pool || !descs || !num_descs || !meta || !ops || !generation ||
	    !rtlmq_ptp_tx_enabled(ptp))
		return false;
	ret = rtlmq_ptp_xsk_message_type(pool, descs, num_descs, &message_type);
	if (!ret)
		ret = rtlmq_ptp_hw_acquire(ptp, true);
	if (ret) {
		spin_lock_irqsave(&ptp->tx_lock, flags);
		ptp->tx_timestamp_skipped++;
		spin_unlock_irqrestore(&ptp->tx_lock, flags);
		return false;
	}

	spin_lock_irqsave(&ptp->tx_lock, flags);
	if (ptp->tx_accepting && !rtlmq_ptp_tx_pending_locked(ptp) &&
	    rtlmq_ptp_tx_enabled(ptp)) {
		ptp->tx_xsk_pool = pool;
		ptp->tx_xsk_descs = num_descs;
		ptp->tx_message_type = message_type;
		xsk_tx_metadata_to_compl(meta, &ptp->tx_xsk_meta);
		ptp->tx_xsk_ops = ops;
		ptp->tx_deadline =
			jiffies + msecs_to_jiffies(RTLMQ_PTP_TX_TIMEOUT_MS);
		rtlmq_ptp_advance_tx_generation_locked(ptp);
		reserved_generation = ptp->tx_generation;
	} else {
		ptp->tx_timestamp_skipped++;
	}
	spin_unlock_irqrestore(&ptp->tx_lock, flags);

	ret = reserved_generation ?
		rtlmq_ptp_clear_tx_status_owned(ptp) : 0;
	spin_lock_irqsave(&ptp->tx_lock, flags);
	if (reserved_generation && !ret && ptp->tx_xsk_pool == pool &&
	    ptp->tx_generation == reserved_generation &&
	    rtlmq_ptp_tx_enabled(ptp)) {
		*generation = reserved_generation;
		reserved = true;
	} else if (reserved_generation && ptp->tx_xsk_pool == pool &&
		   ptp->tx_generation == reserved_generation) {
		rtlmq_ptp_clear_xsk_locked(ptp);
		rtlmq_ptp_advance_tx_generation_locked(ptp);
		ptp->tx_timestamp_skipped++;
	}
	spin_unlock_irqrestore(&ptp->tx_lock, flags);
	rtlmq_ptp_hw_release(ptp);

	if (reserved)
		mod_delayed_work(system_highpri_wq, &ptp->tx_work,
				 msecs_to_jiffies(RTLMQ_PTP_TX_RETRY_MS));
	return reserved;
}

static int rtlmq_ptp_read_tx_timestamp(struct rtlmq_ptp *ptp, u8 message_type,
				       struct skb_shared_hwtstamps *hwtstamp)
{
	ktime_t timestamp;
	u16 command;
	u16 status;
	u16 ready;
	u16 isr;
	int ret;

	if (!rtlmq_mmio_ordinary_authorized(ptp->tp))
		return -ENODEV;

	ret = rtlmq_ptp_hw_acquire(ptp, false);
	if (ret)
		return ret;
	ret = rtlmq_ptp_read_tx_status_owned(ptp, &isr);
	if (ret)
		goto out_release;
	if (!ptp->backend->tx_status_mac_ocp && isr == U16_MAX) {
		ret = -ENODEV;
		goto out_release;
	}
	if (!(isr & RTLMQ_PHY_PTP_ISR_TX)) {
		ret = -EAGAIN;
		goto out_release;
	}

	ret = rtlmq_ptp_phy_read(ptp, RTLMQ_PHY_PTP_TRX_STATUS, &status);
	if (ret)
		goto out_clear;
	ready = status & RTLMQ_PHY_PTP_TX_READY_MASK;
	if (hweight16(ready) != 1) {
		ret = ready ? -EIO : -EAGAIN;
		if (ready)
			goto out_clear;
		goto out_release;
	}
	switch (ready) {
	case RTLMQ_PHY_PTP_TX_SYNC_READY:
		command = 0;
		break;
	case RTLMQ_PHY_PTP_TX_DELAY_READY:
		command = 1U << 2;
		break;
	case RTLMQ_PHY_PTP_TX_PDELAY_REQ_READY:
		command = 2U << 2;
		break;
	case RTLMQ_PHY_PTP_TX_PDELAY_RESP_READY:
		command = 3U << 2;
		break;
	default:
		ret = -EIO;
		goto out_clear;
	}
	if (command != message_type << 2) {
		ret = -ENODATA;
		goto out_clear;
	}
	command |= RTLMQ_PHY_PTP_TRX_READ;
	ret = rtlmq_ptp_phy_trx_command(ptp, command, false);
	if (!ret)
		ret = rtlmq_ptp_phy_read_timestamp(ptp, &timestamp);
out_clear:
	(void)rtlmq_ptp_clear_tx_status_owned(ptp);
	if (ret)
		goto out_release;
	memset(hwtstamp, 0, sizeof(*hwtstamp));
	hwtstamp->hwtstamp = timestamp;
out_release:
	rtlmq_ptp_hw_release(ptp);
	return ret;
}

static void rtlmq_ptp_tx_work(struct work_struct *work)
{
	struct rtlmq_ptp_xsk_finish xsk_finish = {};
	struct skb_shared_hwtstamps hwtstamp;
	struct rtlmq_ptp *ptp =
		container_of(to_delayed_work(work), struct rtlmq_ptp, tx_work);
	struct rtlmq_private *tp = ptp->tp;
	struct sk_buff *skb = NULL;
	unsigned long deadline;
	unsigned long flags;
	u8 message_type;
	u32 generation;
	int ret;

	spin_lock_irqsave(&ptp->tx_lock, flags);
	if (!rtlmq_ptp_tx_pending_locked(ptp)) {
		spin_unlock_irqrestore(&ptp->tx_lock, flags);
		return;
	}
	generation = ptp->tx_generation;
	message_type = ptp->tx_message_type;
	deadline = ptp->tx_deadline;
	spin_unlock_irqrestore(&ptp->tx_lock, flags);

	if (!rtlmq_ptp_active_acquire(ptp) ||
	    !rtlmq_mmio_ordinary_authorized(tp))
		ret = -ENODEV;
	else
		ret = rtlmq_ptp_read_tx_timestamp(ptp, message_type, &hwtstamp);
	if (ret == -EBUSY)
		ret = -EAGAIN;

	if (ret == -EAGAIN && time_before(jiffies, deadline)) {
		spin_lock_irqsave(&ptp->tx_lock, flags);
		if (rtlmq_ptp_tx_pending_locked(ptp) &&
		    ptp->tx_generation == generation)
			mod_delayed_work(system_highpri_wq, &ptp->tx_work,
					 msecs_to_jiffies(RTLMQ_PTP_TX_RETRY_MS));
		spin_unlock_irqrestore(&ptp->tx_lock, flags);
		return;
	}

	/*
	 * Keep the request pending until timeout cleanup finishes. Publishing
	 * an empty slot or XSK timestamp readiness lets a new request reserve
	 * the detector, so no hardware clear may follow that publication.
	 */
	if (ret == -EAGAIN && rtlmq_mmio_ordinary_authorized(tp) &&
	    !rtlmq_ptp_hw_acquire(ptp, false)) {
		(void)rtlmq_ptp_clear_tx_status_owned(ptp);
		rtlmq_ptp_hw_release(ptp);
	}

	/*
	 * Cancellation withdraws the pointer and advances the generation before
	 * waiting for this callback. Never publish a result from an old request
	 * after settime, reset, suspend, remove, or a newer reservation.
	 */
	spin_lock_irqsave(&ptp->tx_lock, flags);
	if (ptp->tx_generation == generation) {
		if (ptp->tx_skb) {
			skb = ptp->tx_skb;
			ptp->tx_skb = NULL;
			rtlmq_ptp_advance_tx_generation_locked(ptp);
			if (!ret)
				ptp->tx_timestamps++;
			else
				ptp->tx_timestamp_missed++;
		} else if (ptp->tx_xsk_pool) {
			ptp->tx_xsk_timestamp_ready = true;
			ptp->tx_xsk_timestamp_valid = !ret;
			ptp->tx_xsk_timestamp =
				ret ? 0 : ktime_to_ns(hwtstamp.hwtstamp);
			if (ptp->tx_xsk_dma_complete)
				rtlmq_ptp_complete_xsk_locked(ptp, &xsk_finish);
		}
	}
	spin_unlock_irqrestore(&ptp->tx_lock, flags);

	if (skb || xsk_finish.valid)
		wake_up_all(&ptp->tx_idle_wait);
	if (!skb && !xsk_finish.valid)
		return;
	if (!ret && skb)
		skb_tstamp_tx(skb, &hwtstamp);
	if (skb)
		dev_kfree_skb_any(skb);
}

void rtlmq_ptp_tx_complete(struct rtlmq_private *tp, u32 generation)
{
	struct rtlmq_ptp *ptp = tp ? READ_ONCE(tp->ptp) : NULL;
	unsigned long flags;

	if (!ptp)
		return;

	/*
	 * Reclaim is only a readiness hint. It must not poll PTP registers or
	 * enter the indirect OCP mailbox from NAPI/softirq context.
	 */
	spin_lock_irqsave(&ptp->tx_lock, flags);
	if (ptp->tx_skb && ptp->tx_generation == generation)
		mod_delayed_work(system_highpri_wq, &ptp->tx_work, 0);
	spin_unlock_irqrestore(&ptp->tx_lock, flags);
}

u32 rtlmq_ptp_xsk_tx_peek_release_desc_batch(struct rtlmq_private *tp,
					     struct xsk_buff_pool *pool,
					     u32 max)
{
	struct rtlmq_ptp *ptp = tp ? READ_ONCE(tp->ptp) : NULL;
	unsigned long flags = 0;
	u32 count;

	/*
	 * Core reservation snapshots CQ producer and updates reclaim-only
	 * counters. Exclude asynchronous timestamp/cancellation publication
	 * for the entire transaction, including the core's internal returns.
	 * Queue lock -> PTP lock -> socket wakeup locks; none calls back here.
	 */
	if (ptp)
		spin_lock_irqsave(&ptp->tx_lock, flags);
	count = xsk_tx_peek_release_desc_batch(pool, max);
	if (ptp)
		spin_unlock_irqrestore(&ptp->tx_lock, flags);
	return count;
}

bool rtlmq_ptp_xsk_tx_complete(struct rtlmq_private *tp,
			       struct xsk_buff_pool *pool, u32 descs,
			       u32 generation)
{
	struct rtlmq_ptp *ptp = tp ? READ_ONCE(tp->ptp) : NULL;
	struct rtlmq_ptp_xsk_finish finish = {};
	unsigned long flags;
	bool owned = false;

	if (!ptp || !pool || !descs)
		return false;

	spin_lock_irqsave(&ptp->tx_lock, flags);
	if (ptp->tx_xsk_pool == pool) {
		if (generation && ptp->tx_generation == generation &&
		    ptp->tx_xsk_descs == descs) {
			ptp->tx_xsk_dma_complete = true;
			ptp->tx_xsk_completed_descs = descs;
			owned = true;
			if (ptp->tx_xsk_timestamp_ready)
				rtlmq_ptp_complete_xsk_locked(ptp, &finish);
			else
				mod_delayed_work(system_highpri_wq,
						 &ptp->tx_work, 0);
		} else if (!generation && ptp->tx_xsk_dma_complete) {
			/*
			 * This pool has one ordered TX reclaim owner. Once its
			 * timestamp EOP retires, all later EOPs belong to the same
			 * deferred CQ prefix until metadata is ready or cancelled.
			 * The pool's bounded CQ reservations bound this count.
			 */
			ptp->tx_xsk_completed_descs += descs;
			owned = true;
		}
	}
	spin_unlock_irqrestore(&ptp->tx_lock, flags);

	if (finish.valid)
		wake_up_all(&ptp->tx_idle_wait);
	return owned;
}

void rtlmq_ptp_tx_cancel(struct rtlmq_private *tp)
{
	struct rtlmq_ptp *ptp = tp ? READ_ONCE(tp->ptp) : NULL;

	if (!ptp)
		return;

	rtlmq_ptp_tx_cancel_request(ptp, true);
}

void rtlmq_ptp_request_init(struct rtlmq_ptp *ptp)
{
	spin_lock_init(&ptp->tx_lock);
	init_waitqueue_head(&ptp->tx_idle_wait);
	INIT_DELAYED_WORK(&ptp->tx_work, rtlmq_ptp_tx_work);
}
