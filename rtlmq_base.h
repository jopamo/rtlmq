/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef __RTLMQ_BASE_H__
#define __RTLMQ_BASE_H__

#include <linux/atomic.h>
#include <linux/build_bug.h>
#include <linux/kconfig.h>
#include <linux/types.h>

/* Common limits, enums, and definitions that do not depend on driver types. */
#define RTLMQ_DRIVER_VERSION		"0.1.2"
#define RTLMQ_BOOT_RX_QUEUES	1
#define RTLMQ_BOOT_TX_QUEUES	1
/*
 * These are implementation limits for storage and algorithms, not hardware
 * capabilities or supported queue counts. Raise them only when the driver
 * needs to support wider hardware.
 */
#define RTLMQ_IMPL_MAX_RX_QUEUES	4
#define RTLMQ_IMPL_MAX_TX_QUEUES	2
#define RTLMQ_IMPL_MAX_NAPI	(RTLMQ_IMPL_MAX_RX_QUEUES + 1)
#define RTLMQ_TX_POLL_KICK_MAX_QUEUES	16
#define RTLMQ_BOOT_VECTORS		1
/* Live vector/statistics capacity is retained for layout and ABI stability. */
#define RTLMQ_MAX_IRQ_SOURCES	6
/* Per-RX routing: one RX source, all TX sources, and one link event. */
#define RTLMQ_MAX_LAYOUT_IRQ_SOURCES (RTLMQ_IMPL_MAX_TX_QUEUES + 2)
#define RTLMQ_RSS_INDIR_SIZE	128
#define RTLMQ_RSS_KEY_SIZE		40

struct bpf_prog;
struct xdp_frame;

enum rtlmq_mac_version {
	RTLMQ_MAC_NONE = 0,
	RTLMQ_MAC_8125B,
	RTLMQ_MAC_8126A,
};

enum rtlmq_eee_tx_lpi_layout {
	RTLMQ_EEE_TX_LPI_NONE = 0,
	RTLMQ_EEE_TX_LPI_E040,
	RTLMQ_EEE_TX_LPI_E040_EB62,
};

/*
 * These names match the family vendor drivers. They come from the
 * TxConfig IC-version bits, not the PCI revision.
 */
enum rtlmq_config_method {
	RTLMQ_CFG_METHOD_UNKNOWN = 0,
	RTLMQ_CFG_METHOD_2 = 2,
	RTLMQ_CFG_METHOD_3 = 3,
	RTLMQ_CFG_METHOD_4 = 4,
	RTLMQ_CFG_METHOD_5 = 5,
};

/*
 * PTP register layouts and timestamp handling differ by backend. Select the
 * full backend from the chip profile instead of guessing from the MAC family.
 */
enum rtlmq_ptp_backend {
	RTLMQ_PTP_BACKEND_NONE = 0,
	RTLMQ_PTP_BACKEND_PHY_V2,
	RTLMQ_PTP_BACKEND_PHY_V3,
};

enum rtlmq_irq_mode {
	RTLMQ_IRQ_MODE_NONE = 0,
	RTLMQ_IRQ_MODE_INTX,
	RTLMQ_IRQ_MODE_MSI,
	RTLMQ_IRQ_MODE_MSIX,
};

/*
 * control_lock serializes device-wide lifecycle changes.
 *
 * Lock order, outer to inner:
 * - rtnl_lock() -> control_lock -> the short netdev instance lock used only
 *   when updating XDP features;
 * - control_lock -> state_lock, led_lock, mac_ocp_lock, phy_ocp_lock,
 *   PHY/MDIO core locks, PTP hardware access, netdev TX queue lock,
 *   napi_rearm_lock;
 * - netdev TX queue lock -> napi_rearm_lock.
 *
 * state_lock, led_lock, mac_ocp_lock, phy_ocp_lock, and the PTP TX-request lock
 * are leaf locks. PTP register access may take phy_ocp_lock.
 * napi_rearm_lock is a leaf lock owning TX IRQ intent, mask writes and the
 * IRQ/XSK schedule/rearm gate. NAPI completion never runs while it is held.
 * There is no private TX producer or reclaim lock. TX producers use the netdev
 * TX queue lock; reclaim is serialized by each ring's NAPI owner and
 * release/acquire cursor updates.
 *
 * NAPI_STATE_SCHED marks NAPI service ownership; it is not a driver lock.
 * Page-pool and XSK objects are managed by the core. The driver creates page
 * pools and acquires/releases XSK DMA mappings under control_lock. Private
 * candidate page pools may be built or discarded while the old generation
 * runs. Stop a pool's driver users before releasing its active bindings.
 * Page-pool datapath operations run from NAPI, and XSK TX runs under the
 * netdev TX queue lock. RCU and u64_stats_sync do not take driver locks.
 *
 * PREEMPT_RT: driver spin locks use spinlock_t, not raw_spinlock_t.
 * irqsave is required because state updates, IRQ/NAPI rearm, XSK wakeup, and
 * TX IRQ reconciliation can run from hard IRQ, NAPI, or process context.
 * These critical sections only update small state, do lockless reads, write
 * MMIO masks, perform bounded MMIO mailbox transactions, trace, schedule NAPI,
 * or invoke nonblocking callbacks. They must not sleep, allocate, drain
 * callbacks, disable or delete NAPI, or take a mutex.
 *
 * Production does not create a devlink instance or use per-netdev ops locking
 * because its phylib callbacks still use RTNL semantics. Do not take RTNL,
 * a netdev instance lock, or control_lock while holding a state, LED,
 * vector, or TX queue lock.
 *
 * Legal transitions:
 * NEW -> PROBED -> OPENING -> OPEN -> CLOSING -> PROBED
 * OPEN -> RESETTING -> OPEN
 * PROBED -> DEAD
 * OPENING -> PROBED on unwind
 * RESETTING -> PROBED on reset unwind
 * suspend/resume and PCI AER recovery both reuse
 * OPEN -> CLOSING -> PROBED and PROBED -> OPENING -> OPEN
 * through the same lifecycle helpers
 * PROBED/OPENING/OPEN/RESETTING/CLOSING -> REMOVING -> DEAD for final teardown
 * Only KUnit failed-stop cleanup permits REMOVING -> PROBED.
 */
enum rtlmq_lifecycle_state {
	RTLMQ_STATE_NEW = 0,
	RTLMQ_STATE_PROBED,
	RTLMQ_STATE_OPENING,
	RTLMQ_STATE_OPEN,
	RTLMQ_STATE_RESETTING,
	RTLMQ_STATE_CLOSING,
	RTLMQ_STATE_REMOVING,
	RTLMQ_STATE_DEAD,
};

/*
 * Quiescing active resources is a sequence of completed steps, not a delay.
 * These bits record when publication, async work, IRQ/NAPI, DMA, ring
 * registers, and readers have been shut down far enough to free the active
 * generation. Discarding an uncommitted candidate is separate and does not
 * set these bits.
 */
enum rtlmq_quiesce_phase {
	RTLMQ_QUIESCE_PUBLICATION_REVOKED	= 1U << 0,
	RTLMQ_QUIESCE_ASYNC_REACQUIRE_REVOKED	= 1U << 1,
	RTLMQ_QUIESCE_IRQ_SOURCE_REVOKED	= 1U << 2,
	RTLMQ_QUIESCE_NAPI_DISABLED		= 1U << 3,
	RTLMQ_QUIESCE_HARDWARE_STOPPED		= 1U << 4,
	/* PCI command/pending checks only; not a posted-DMA drain. */
	RTLMQ_QUIESCE_DMA_REVOKED		= 1U << 5,
	RTLMQ_QUIESCE_RING_REGISTERS_REVOKED	= 1U << 6,
	RTLMQ_QUIESCE_IRQ_HANDLERS_RELEASED	= 1U << 7,
	RTLMQ_QUIESCE_RCU_DRAINED		= 1U << 8,
	/* Irreversible core teardown after failed shutdown; not DMA evidence. */
	RTLMQ_QUIESCE_FINAL_RELEASE		= 1U << 9,
};

enum rtlmq_reset_reason {
	RTLMQ_RESET_REASON_TX_TIMEOUT = 0,
	RTLMQ_RESET_REASON_CHANNEL_RECONFIG,
	RTLMQ_RESET_REASON_RUNTIME_RECONFIG,
	RTLMQ_RESET_REASON_LINK_WEDGE,
	RTLMQ_RESET_REASON_PCI_RECOVERY,
	RTLMQ_RESET_REASON_USER,
	RTLMQ_RESET_REASON_PM_RESUME,
	RTLMQ_RESET_REASON_RX_FILTER,
	__RTLMQ_RESET_REASON_COUNT,
};

enum rtlmq_routing_mode {
	RTLMQ_ROUTING_SINGLE_VECTOR = 0,
	RTLMQ_ROUTING_RX_PER_VECTOR_TX_COUPLED,
};

struct rtlmq_private;

/*
 * Shared queue/ring declarations, transition reasons, and counters.
 * RX, TX, and vector struct definitions are in their respective headers.
 */
struct rtlmq_rx_ring;
struct rtlmq_tx_ring;
struct rtlmq_vector;
/* Disjoint terminal dispositions; error/refill events are not packet drops. */
enum rtlmq_rx_drop_reason {
	RTLMQ_RX_DROP_STATUS,
	RTLMQ_RX_DROP_FRAGMENT,
	RTLMQ_RX_DROP_LENGTH,
	RTLMQ_RX_DROP_SKB,
	RTLMQ_RX_DROP_RESOURCE,
	RTLMQ_RX_DROP_XDP,
	RTLMQ_RX_DROP_COUNT,
};

struct rtlmq_rx_ring_stats {
	u64 drops;
	u64 drop_reason[RTLMQ_RX_DROP_COUNT];
	u64 rx_errors;
	u64 rx_crc_errors;
	u64 rx_length_errors;
	u64 alloc_fail;
	u64 csum_ok;
	u64 csum_bad;
};

static_assert(sizeof(struct rtlmq_rx_ring_stats) == 104);
enum rtlmq_tx_buffer_type {
	RTLMQ_TX_BUFFER_NONE = 0,
	RTLMQ_TX_BUFFER_SKB,
	RTLMQ_TX_BUFFER_XDP_NDO,
	RTLMQ_TX_BUFFER_XDP_RX,
	RTLMQ_TX_BUFFER_XSK,
};

enum rtlmq_tx_publish_result {
	RTLMQ_TX_PUBLISHED = 0,
	RTLMQ_TX_BUSY,
	RTLMQ_TX_DROPPED,
};

struct rtlmq_ring_stats {
	u64 packets;
	u64 bytes;
	u64 drops;
	/* Transmit and completion paths both publish queue-state changes. */
	atomic64_t queue_stopped;
	atomic64_t queue_wake;
	u64 xmit_tso_pkts;
};

static_assert(sizeof(struct rtlmq_ring_stats) == 48);

#endif /* __RTLMQ_BASE_H__ */
