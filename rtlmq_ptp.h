/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _RTLMQ_PTP_H
#define _RTLMQ_PTP_H

#include <linux/atomic.h>
#include <linux/jump_label.h>
#include <linux/ptp_clock_kernel.h>
#include <linux/spinlock.h>
#include <linux/wait.h>
#include <linux/workqueue.h>
#include <net/xdp_sock_drv.h>

#include "rtlmq_base.h"

struct rtlmq_private;
struct device;
struct ptp_clock;
struct ptp_clock_info;
struct ethtool_ts_stats;
struct kernel_ethtool_ts_info;
struct kernel_hwtstamp_config;
struct net_device;
struct netlink_ext_ack;
struct rtlmq_tx_completion;
struct sk_buff;
struct timespec64;
struct xsk_buff_pool;
struct xsk_tx_metadata;
struct xsk_tx_metadata_ops;

/*
 * Probe/remove registers and unregisters the PHC. Activate hardware under
 * control_lock; reset, PM, and AER use the same quiesce/restore helpers.
 */
int rtlmq_ptp_register(struct rtlmq_private *tp);
void rtlmq_ptp_unregister(struct rtlmq_private *tp);
int rtlmq_ptp_restore_locked(struct rtlmq_private *tp);
void rtlmq_ptp_quiesce_locked(struct rtlmq_private *tp);
void rtlmq_ptp_freeze_locked(struct rtlmq_private *tp);
/*
 * Live datapath pauses preserve PHC time/frequency and timestamp modes.
 * Attach may fail before commit if the pending TX timestamp does not drain.
 * Detach may cancel it so the old pool can be released safely.
 */
int rtlmq_ptp_pause_datapath_locked(struct rtlmq_private *tp, bool force);
int rtlmq_ptp_resume_datapath_locked(struct rtlmq_private *tp);
bool rtlmq_ptp_defer_link_event(struct rtlmq_private *tp);
int rtlmq_ptp_hwtstamp_get(struct net_device *ndev,
			   struct kernel_hwtstamp_config *config);
int rtlmq_ptp_hwtstamp_set(struct net_device *ndev,
			   struct kernel_hwtstamp_config *config,
			   struct netlink_ext_ack *extack);
int rtlmq_ptp_get_ts_info(struct net_device *ndev,
			  struct kernel_ethtool_ts_info *info);
void rtlmq_ptp_get_ts_stats(struct net_device *ndev,
			    struct ethtool_ts_stats *stats);
bool rtlmq_ptp_tx_prepare(struct rtlmq_private *tp, struct sk_buff *skb,
			  struct rtlmq_tx_completion *completion);
bool rtlmq_ptp_xsk_tx_prepare(struct rtlmq_private *tp,
			      struct xsk_buff_pool *pool,
			      const struct xdp_desc *descs, u16 num_descs,
			      struct xsk_tx_metadata *meta,
			      const struct xsk_tx_metadata_ops *ops,
			      u32 *generation);
u32 rtlmq_ptp_xsk_tx_peek_release_desc_batch(struct rtlmq_private *tp,
					     struct xsk_buff_pool *pool,
					     u32 max);
/* generation=0 denotes an ordinary EOP following the ordered PTP CQ owner. */
bool rtlmq_ptp_xsk_tx_complete(struct rtlmq_private *tp,
			       struct xsk_buff_pool *pool, u32 descs,
			       u32 generation);
void rtlmq_ptp_tx_complete(struct rtlmq_private *tp, u32 generation);
void rtlmq_ptp_tx_cancel(struct rtlmq_private *tp);
bool rtlmq_ptp_rx_timestamp_scan_required(const struct rtlmq_private *tp);
int rtlmq_ptp_rx_timestamp(struct rtlmq_private *tp, const void *packet,
			   size_t linear_len, size_t len, ktime_t *timestamp);

DECLARE_STATIC_KEY_FALSE(rtlmq_ptp_rx_scan_key);

static inline bool
rtlmq_ptp_rx_scan_maybe_required(const struct rtlmq_private *tp)
{
	return static_branch_unlikely(&rtlmq_ptp_rx_scan_key) &&
	       rtlmq_ptp_rx_timestamp_scan_required(tp);
}

#define RTLMQ_PTP_MODE_TX	BIT(0)
#define RTLMQ_PTP_MODE_RX	BIT(1)
#define RTLMQ_PTP_NSEC_MASK	GENMASK(29, 0)

/* Commands and status values passed across the hardware-owner boundary. */
#define RTLMQ_PHY_PTP_CLOCK_DIRECT_READ	4
#define RTLMQ_PHY_PTP_CLOCK_DIRECT_WRITE	6
#define RTLMQ_PHY_PTP_CLOCK_RATE_WRITE	14
#define RTLMQ_PHY_PTP_ISR_TX		BIT(3)
#define RTLMQ_PHY_PTP_TRX_RX_SELECT	BIT(1)
#define RTLMQ_PHY_PTP_TRX_READ		BIT(4)
#define RTLMQ_PHY_PTP_TX_READY_MASK	GENMASK(15, 12)
#define RTLMQ_PHY_PTP_TX_SYNC_READY	BIT(15)
#define RTLMQ_PHY_PTP_TX_DELAY_READY	BIT(14)
#define RTLMQ_PHY_PTP_TX_PDELAY_REQ_READY BIT(13)
#define RTLMQ_PHY_PTP_TX_PDELAY_RESP_READY BIT(12)

struct rtlmq_ptp_backend_contract {
	enum rtlmq_ptp_backend type;
	u16 idle_ctrl;
	u16 packet_ctrl;
	bool tx_status_mac_ocp;
	bool status_w1c;
	bool supports_rx;
	bool clear_mac_timestamp_source;
};

struct rtlmq_ptp_config {
	int tx_type;
	int rx_filter;
};

struct rtlmq_ptp {
	struct rtlmq_private *tp;
	const struct rtlmq_ptp_backend_contract *backend;
	struct ptp_clock_info info;
	struct ptp_clock *clock;
	struct rtlmq_ptp_config requested_config;
	u8 active_modes;
	bool rx_timestamp_scan_required;
	s32 frequency_ppb;
	/*
	 * One PHY timestamp register bank is shared by PHC, RX, and TX. Process
	 * context waits for exclusive access; atomic NAPI/TX callers retry for
	 * a bounded time and fail the timestamp rather than sleep.
	 */
	atomic_t hw_busy;
	wait_queue_head_t hw_wait;
	/*
	 * Allow one TX timestamp request at a time. Hold an SKB reference, or
	 * retain AF_XDP metadata/CQ state until both DMA completion and the
	 * timestamp worker finish. Reclaim does not read the OCP mailbox.
	 */
	struct delayed_work tx_work;
	/*
	 * Applying A640 raises a delayed MAC link event without dropping the
	 * peer link. Defer that event and ask phylib to read link status after
	 * hardware settles.
	 */
	struct delayed_work link_reconcile_work;
	unsigned long link_quarantine_until;
	wait_queue_head_t tx_idle_wait;
	struct sk_buff *tx_skb;
	struct xsk_buff_pool *tx_xsk_pool;
	struct xsk_tx_metadata_compl tx_xsk_meta;
	const struct xsk_tx_metadata_ops *tx_xsk_ops;
	u64 tx_xsk_timestamp;
	u16 tx_xsk_descs;
	u32 tx_xsk_completed_descs;
	bool tx_xsk_dma_complete;
	bool tx_xsk_timestamp_ready;
	bool tx_xsk_timestamp_valid;
	unsigned long tx_deadline;
	u32 tx_generation;
	u8 tx_message_type;
	bool tx_accepting;
	/* PREEMPT_RT: request/CQ serialization; no sleeping or hardware polling. */
	spinlock_t tx_lock;
	u64 tx_timestamps;
	u64 tx_timestamp_skipped;
	u64 tx_timestamp_missed;
	bool active;
};

const struct rtlmq_ptp_backend_contract *
rtlmq_ptp_backend_contract(enum rtlmq_ptp_backend type);
bool rtlmq_ptp_backend_matches(const struct rtlmq_private *tp,
			       const struct rtlmq_ptp *ptp);
void rtlmq_ptp_clock_init(struct rtlmq_ptp *ptp);
struct ptp_clock *rtlmq_ptp_register_clock(struct rtlmq_private *tp,
					   struct rtlmq_ptp *ptp);
int rtlmq_ptp_write_time_locked(struct rtlmq_ptp *ptp,
				const struct timespec64 *ts);
int rtlmq_ptp_program_frequency_locked(struct rtlmq_ptp *ptp, s32 ppb);
void rtlmq_ptp_request_init(struct rtlmq_ptp *ptp);
void rtlmq_ptp_require_rx_timestamp_scan(struct rtlmq_ptp *ptp);
void rtlmq_ptp_clear_rx_timestamp_scan(struct rtlmq_ptp *ptp);
int rtlmq_ptp_tx_start(struct rtlmq_ptp *ptp, u8 modes);
void rtlmq_ptp_tx_cancel_request(struct rtlmq_ptp *ptp, bool clear_hardware);
int rtlmq_ptp_hw_acquire(struct rtlmq_ptp *ptp, bool atomic_context);
void rtlmq_ptp_hw_release(struct rtlmq_ptp *ptp);
int rtlmq_ptp_phy_read(struct rtlmq_ptp *ptp, u16 reg, u16 *value);
int rtlmq_ptp_phy_write(struct rtlmq_ptp *ptp, u16 reg, u16 value);
int rtlmq_ptp_phy_clock_command(struct rtlmq_ptp *ptp, u16 command);
int rtlmq_ptp_phy_trx_command(struct rtlmq_ptp *ptp, u16 command,
			      bool atomic_context);
int rtlmq_ptp_phy_read_timestamp(struct rtlmq_ptp *ptp, ktime_t *timestamp);
int rtlmq_ptp_clear_tx_status_owned(struct rtlmq_ptp *ptp);
int rtlmq_ptp_read_tx_status_owned(struct rtlmq_ptp *ptp, u16 *status);
int rtlmq_ptp_phy_program_modes_owned(struct rtlmq_ptp *ptp, u8 old_modes,
				      u8 modes, bool phc_active);
int rtlmq_ptp_phy_config_modes_owned(struct rtlmq_ptp *ptp, u8 old_modes,
				     u8 new_modes, bool phc_active);
int rtlmq_ptp_phy_live_pause_owned(struct rtlmq_ptp *ptp, u8 old_modes);
int rtlmq_ptp_phy_live_resume_owned(struct rtlmq_ptp *ptp, u8 modes);

#endif /* _RTLMQ_PTP_H */
