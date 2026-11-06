/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef __RTLMQ_H__
#define __RTLMQ_H__

#include <linux/atomic.h>
#include <linux/dma-mapping.h>
#include <linux/mii.h>
#include <linux/mutex.h>
#include <linux/netdevice.h>
#include <linux/pci.h>
#include <linux/phy.h>
#include <linux/spinlock.h>
#include <linux/u64_stats_sync.h>
#include <linux/workqueue.h>

#include "rtlmq_chip.h"
#include "rtlmq_hw.h"
#include "rtlmq_runtime.h"
#include "rtlmq_rx.h"
#include "rtlmq_tx.h"
#include "rtlmq_irq.h"

#define RTLMQ_DEFAULT_MSG_ENABLE	NETIF_MSG_LINK

struct completion;
struct rtlmq_chip_profile;
struct rtlmq_leds;
struct rtlmq_ptp;

enum rtlmq_platform_quiesce_owner {
	RTLMQ_PLATFORM_QUIESCE_NONE = 0,
	RTLMQ_PLATFORM_QUIESCE_SYSTEM_PM,
	RTLMQ_PLATFORM_QUIESCE_RUNTIME_PM,
	RTLMQ_PLATFORM_QUIESCE_PCI_ERROR,
};

enum rtlmq_mmio_authority {
	RTLMQ_MMIO_AUTHORITY_REVOKED = 0,
	RTLMQ_MMIO_AUTHORITY_ORDINARY,
	RTLMQ_MMIO_AUTHORITY_RECOVERY,
	RTLMQ_MMIO_AUTHORITY_TERMINAL,
};

/*
 * Queue statistics must survive ring replacement. Keep retired counters on
 * the device so channel changes and resets do not lose or double-count traffic.
 */
struct rtlmq_queue_stats {
	u64 rx_packets;
	u64 rx_bytes;
	u64 rx_drops;
	u64 rx_drop_reason[RTLMQ_RX_DROP_COUNT];
	u64 rx_errors;
	u64 rx_crc_errors;
	u64 rx_length_errors;
	u64 rx_alloc_fail;
	u64 rx_csum_unnecessary;
	u64 rx_csum_bad;
	u64 tx_packets;
	u64 tx_bytes;
	u64 tx_drops;
	u64 tx_hw_gso_packets;
	u64 tx_stop;
	u64 tx_wake;
};

static_assert(sizeof(struct rtlmq_queue_stats) == 168);

struct rtlmq_private {
	/*
	 * mmio_addr is the pcim-managed BAR mapping, but a valid mapping does not
	 * mean MMIO is allowed. mmio_authority controls access independently.
	 * Freeze and PM paths block normal access without unmapping the BAR.
	 * PCI recovery uses separate access checks. Terminal revocation is final.
	 */
	void __iomem *mmio_addr;
	enum rtlmq_mmio_authority mmio_authority;
	struct pci_dev *pdev;
	struct net_device *ndev;
	/* Optional 8126 serial EEPROM, detected after MAC reset under control_lock. */
	u16 eeprom_len;
	struct mii_bus *mii_bus;
	struct phy_device *phydev;
	u16 ocp_base;
	/* Bootstrap obligation retained across a partially completed UPS exit. */
	bool phy_ups_cleanup_pending;
	/*
	 * OCPDR is a shared indirect MAC mailbox. Hold the lock across each
	 * command/read pair so another CPU cannot change the selected OCP address
	 * in the middle of a transaction.
	 */
	/* PREEMPT_RT: short MMIO transaction; no sleeping or nested driver locks. */
	spinlock_t mac_ocp_lock;
	/* PREEMPT_RT: bounded atomic GPHY polling; no nested driver locks. */
	spinlock_t phy_ocp_lock;
	/* Probe determines the chip identity and selects the fixed chip profile. */
	u16 xid;
	u16 raw_xid;
	enum rtlmq_config_method config_method;
	const struct rtlmq_chip_profile *profile;
	dma_addr_t counters_phys_addr;
	struct rtlmq_counters *counters;
	/*
	 * counters is DMA scratch space written by the hardware. tally holds the
	 * cumulative software-visible counts and updates only after a completed
	 * dump. It survives runtime rebuilds; previous_valid is cleared on reset.
	 */
	struct rtlmq_tally_state tally;
	/*
	 * Writers hold control_lock. READ_ONCE samples are only for conservative
	 * checks or diagnostics. Load runtime_published with acquire semantics
	 * before reading the committed active configuration.
	 */
	enum rtlmq_lifecycle_state state;
	/*
	 * requested_profile stores persistent queue requests plus derived
	 * configuration and layout. Active commit also copies the selected IRQ
	 * outcome here. Replanning uses the requested counts and selects IRQ mode
	 * again; this copy does not describe live hardware.
	 *
	 * requested_profile.config.requested_num_* stores the current requested
	 * queue counts. The rollback snapshot only keeps a temporary copy so a
	 * failed transaction can restore the previous request.
	 */
	struct rtlmq_runtime_profile_state requested_profile;
	/*
	 * Candidate profile, resources and staged RSS/XDP bindings. Commit
	 * transfers ownership to active state before clearing the transaction.
	 */
	struct rtlmq_runtime_transaction transaction;
	struct rtlmq_runtime_profile_state active_profile;
	/*
	 * A generation token identifies the DMA-ring, NAPI, and IRQ resources
	 * validated for a rebuild. Commit makes that generation active; unwind
	 * clears it before freeing any resources. Ring DMA ownership comes from
	 * the ring descriptors, not a second device-wide mask.
	 */
	u32 runtime_generation_seq;
	u32 active_resource_generation;
	/*
	 * Release authority for DMA storage: never admitted, or normal hardware
	 * shutdown followed by successful PCI checks. PCI checks alone cannot
	 * establish this after admission. Not a terminal posted-DMA fence.
	 */
	bool dma_quiesced;
	u32 quiesce_phase;
	/*
	 * A rebuild may quiesce the old generation before the replacement becomes
	 * active. Keep that fact across candidate activation so commit only frees
	 * storage that is still known to be safe from old-generation DMA.
	 */
	bool retired_generation_quiesced;
	/*
	 * Release-published gate for userspace and datapath readers. A valid
	 * active_profile alone is not sufficient while OPENING or unwinding.
	 * Live ring replacement also withdraws and republishes this gate.
	 */
	bool runtime_published;
	/*
	 * These are the requested ethtool coalescing values. While the device is
	 * OPEN, hardware moderation is encoded from these values. The lifecycle
	 * state gates current-value reporting, so no second set of active values
	 * is needed.
	 */
	u32 requested_irq_coalesce_usecs;
	u32 requested_irq_coalesce_frames;
	u32 requested_tx_irq_coalesce_usecs;
	u32 requested_tx_irq_coalesce_frames;
	/*
	 * Cache CPlusCmd after reset. Feature changes update only the bits managed
	 * by the driver and preserve the interrupt-timer scale chosen by hardware.
	 */
	u16 cp_cmd;
	u32 wol_opts;
	/*
	 * Per-device ethtool message mask. Link messages stay enabled by default;
	 * userspace can change the mask later through ETHTOOL_SMSGLVL.
	 */
	u32 msg_enable;
	/*
	 * Retired queue counters live on the device so they survive ring teardown.
	 * This sequence protects both the history and the active-to-retired handoff
	 * from torn reads on narrow architectures. Ring-local sequences still
	 * protect live counters.
	 */
	struct rtlmq_queue_stats queue_stats_history;
	struct u64_stats_sync queue_stats_syncp;
	u32 queue_stats_retired_generation;
	/*
	 * RX descriptor format written into ring memory and confirmed by queue
	 * state readback before the datapath is enabled.
	 */
	enum rtlmq_rx_desc_format rx_desc_fmt;
	/*
	 * allocated_vectors sizes the persistent vector and NAPI objects. It is
	 * limited by the driver's queue model, not the raw MSI-X table size. The
	 * active MSI-X width is derived from the committed vector message IDs.
	 */
	u16 allocated_vectors;
	/*
	 * PCI IRQ-vector allocation transferred from
	 * transaction.irq_vectors_allocated at active commit.
	 */
	bool active_irq_vectors_allocated;
	/*
	 * control_lock protects the split between requested and active XDP state:
	 * - requested_xdp_prog is the admin request, retained while down.
	 * - xdp_prog is the active program used by RX polling and XDP_TX.
	 */
	struct bpf_prog *requested_xdp_prog;
	struct bpf_prog *xdp_prog;
	/*
	 * requested_xsk_pools stores the requested queue bindings. Active AF_XDP
	 * bindings live in ring->cold->xsk_pool after the ring set is committed.
	 * The AF_XDP core keeps each bound pool alive; these pointers are
	 * borrowed. DMA mapping lifetime is tracked separately;
	 * do not add another xp_get_pool()/xp_put_pool() reference here.
	 */
	struct xsk_buff_pool *requested_xsk_pools[RTLMQ_IMPL_MAX_RX_QUEUES];
	/*
	 * Track the pools that are actually DMA-mapped, one entry per unique pool.
	 * This is separate from requested queue bindings, so rollback can restore
	 * those pointers without pretending each queue owns a DMA mapping.
	 */
	struct rtlmq_xsk_dma_pool_state xsk_dma_pools[RTLMQ_IMPL_MAX_RX_QUEUES];
	/* Serializes lifecycle transitions and requested/candidate/active updates. */
	struct mutex control_lock;
	/* LED register RMW and saved modes; acquired under control_lock. */
	struct mutex led_lock;
	/* PREEMPT_RT: spinlock_t; held only for short reset-state updates. */
	spinlock_t state_lock;
	u32 reset_epoch;
	u32 reset_pending_epoch;
	u32 reset_pending_reasons;
	struct delayed_work reset_work;
	unsigned int tx_timeout_pending_events;
	struct work_struct link_work;
	u32 link_work_epoch;
	enum rtlmq_platform_quiesce_owner platform_quiesce_owner;
	bool platform_quiesce_was_running;
	/* Borrowed only during synchronous platform close/reopen under control_lock. */
	struct task_struct *platform_access_task;
	/*
	 * Runtime PM may suspend the hardware only while the netdev is down.
	 * When runtime PM is enabled, OPEN holds runtime_pm_netdev_ref. System
	 * sleep may still quiesce an OPEN device through platform recovery;
	 * system_sleep_transition marks that suspend interval.
	 * control_lock protects runtime_pm_suspended.
	 */
	bool runtime_pm_ready;
	bool runtime_pm_suspended;
	bool runtime_pm_netdev_ref;
	bool system_sleep_transition;
	/*
	 * Probe sets this only when PCIe link policy is under driver control.
	 * RTL8126 replays its internal ASPM/CLKREQ lock from this committed fact
	 * after each reset; inherited register state is not authoritative.
	 */
	bool aspm_manageable;
	struct rtlmq_requested_runtime_snapshot platform_recovery_committed;
	bool platform_recovery_committed_valid;
	struct rtlmq_rx_ring *rx_rings;
	struct rtlmq_rx_ring_cold *rx_ring_cold;
	u16 allocated_rx_ring_slots;
	struct rtlmq_tx_ring *tx_rings;
	struct rtlmq_tx_ring *txq2ring[RTLMQ_IMPL_MAX_TX_QUEUES];
	/*
	 * Number of TX ring slots allocated internally, not a user-visible queue
	 * count.
	 */
	u16 allocated_tx_ring_slots;
	/* Vector objects live from probe to remove. Active IRQ count is separate. */
	struct rtlmq_vector *vectors;
	struct rtlmq_rss rss;
	struct rtlmq_rss active_rss;
	/*
	 * Traffic-control state has separate requested and active forms. mqprio is
	 * exposed through the netdev core, while queue-local CBS registers must be
	 * restored after every MAC reset.
	 */
	struct rtlmq_tc_state tc;
	/* state_lock protects snapshot replacement and validity together. */
	struct rtlmq_tx_timeout_snapshot last_tx_timeout;
	bool last_tx_timeout_valid;
	struct rtlmq_fw *fw;
	struct rtlmq_leds *leds;
	struct rtlmq_ptp *ptp;
	/* Apply the boot-time EEE default once; phylib handles later EEE changes. */
	bool eee_default_applied;
	/* Failed live-ring rollback retains these sets until quiesce succeeds. */
	struct rtlmq_live_rx_ring_set *retained_live_rx;
	struct rtlmq_live_tx_ring_set *retained_live_tx;
	/* RTNL-scoped feedback from core's void feature-update API; cold only. */
	int *feature_sync_error;
	/* Applied control-path TSO filter; core still owns wanted_features/GSO. */
	bool cbs_tso_constrained;
};

int rtlmq_sync_features_locked(struct rtlmq_private *tp);
void rtlmq_sync_vlan_features(struct net_device *ndev,
			      netdev_features_t features);

static inline bool
rtlmq_mmio_ordinary_authorized(const struct rtlmq_private *tp)
{
	if (!tp)
		return false;

	/* Acquire pairs with stores that change mmio_authority. */
	if (smp_load_acquire(&tp->mmio_authority) !=
	    RTLMQ_MMIO_AUTHORITY_ORDINARY)
		return false;

	return READ_ONCE(tp->mmio_addr);
}

static inline void __iomem *
rtlmq_mmio_recovery_addr(const struct rtlmq_private *tp)
{
	if (!tp)
		return NULL;

	/* Acquire pairs with stores that change mmio_authority. */
	if (smp_load_acquire(&tp->mmio_authority) !=
	    RTLMQ_MMIO_AUTHORITY_RECOVERY)
		return NULL;

	return READ_ONCE(tp->mmio_addr);
}

#endif /* __RTLMQ_H__ */
