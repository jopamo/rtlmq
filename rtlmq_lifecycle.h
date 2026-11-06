/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef __RTLMQ_LIFECYCLE_H__
#define __RTLMQ_LIFECYCLE_H__

#include <linux/types.h>

#include "rtlmq_runtime_accessors.h"

struct rtlmq_tx_timeout_snapshot;
struct work_struct;

enum rtlmq_live_mac_pause_scope {
	RTLMQ_LIVE_MAC_PAUSE_RX_ONLY = 0,
	RTLMQ_LIVE_MAC_PAUSE_ALL_ENGINES,
};

struct rtlmq_live_service_pause {
	u64 napi_mask;
	enum rtlmq_live_mac_pause_scope scope;
	bool tx_disabled;
	bool ptp_paused;
	bool mac_paused;
};

enum rtlmq_live_change_outcome {
	RTLMQ_LIVE_CHANGE_NOT_COMMITTED,
	RTLMQ_LIVE_CHANGE_COMMITTED,
	RTLMQ_LIVE_CHANGE_RECOVERY_PENDING,
};

int rtlmq_complete_live_change_locked(struct rtlmq_private *tp,
				      enum rtlmq_live_change_outcome outcome,
				      int error, const char *where);

/*
 * Exclude netpoll during live changes that bypass netdev open/close.
 * Netpoll core already provides this exclusion for ndo_open()/ndo_stop().
 */
struct rtlmq_netpoll_guard {
	struct netpoll_info *info;
	bool armed;
};

void rtlmq_netpoll_guard_acquire_locked(struct rtlmq_private *tp,
					struct rtlmq_netpoll_guard *guard);
void rtlmq_netpoll_guard_release_locked(struct rtlmq_private *tp,
					struct rtlmq_netpoll_guard *guard);

/*
 * Lifecycle and rebuild API:
 * - device state transitions
 * - open / close / reset / queued-recovery entry points
 * - reset epoch and XDP feature publication tied to lifecycle commits
 */
int rtlmq_set_state_locked(struct rtlmq_private *tp,
			   enum rtlmq_lifecycle_state state);
int rtlmq_admit_control_change_locked(struct rtlmq_private *tp,
				      bool allow_probed);
bool rtlmq_can_apply_down_admin_change_locked(const struct rtlmq_private *tp);
bool rtlmq_can_apply_live_admin_rebuild_locked(const struct rtlmq_private *tp);
int rtlmq_grant_ordinary_mmio_locked(struct rtlmq_private *tp);
int rtlmq_grant_recovery_mmio_locked(struct rtlmq_private *tp);
void rtlmq_revoke_mmio_locked(struct rtlmq_private *tp);
void rtlmq_terminalize_mmio_locked(struct rtlmq_private *tp);
void rtlmq_sync_xdp_features_locked(struct rtlmq_private *tp);
void rtlmq_sync_netdev_tso_limits(struct rtlmq_private *tp);
int rtlmq_finalize_failed_runtime_locked(struct rtlmq_private *tp);
int rtlmq_pause_live_service_locked(struct rtlmq_private *tp,
				    struct rtlmq_live_service_pause *pause,
				    enum rtlmq_live_mac_pause_scope scope,
				    bool preserve_fence_on_failure);
int rtlmq_restore_live_service_locked(struct rtlmq_private *tp,
				      struct rtlmq_live_service_pause *pause);
void rtlmq_revoke_platform_datapath_locked(struct rtlmq_private *tp);
int rtlmq_capture_platform_recovery_committed_locked(struct rtlmq_private *tp);
int rtlmq_restore_platform_recovery_committed_locked(struct rtlmq_private *tp);
void rtlmq_clear_platform_recovery_committed_locked(struct rtlmq_private *tp);
int rtlmq_terminalize_frozen_pci_failure_locked(struct rtlmq_private *tp);
int rtlmq_open_device(struct rtlmq_private *tp);
int rtlmq_apply_live_reconfiguration_locked(
	struct rtlmq_private *tp,
	u32 reset_reason,
	const struct rtlmq_requested_runtime_snapshot *rollback_requested
);
int rtlmq_apply_live_ring_reconfiguration_locked(
	struct rtlmq_private *tp,
	const struct rtlmq_requested_runtime_snapshot *rollback_requested,
	const char *where);
/*
 * Reset/reopen recovery. Use the supplied pre-change snapshot, or capture the
 * current committed request if none is supplied. Reset OPEN devices; restore
 * the request for PROBED devices before reopening or reattaching while down.
 */
int rtlmq_apply_recovery_transaction_locked(
	struct rtlmq_private *tp,
	u32 reset_reasons,
	const struct rtlmq_requested_runtime_snapshot *rollback_requested,
	bool reopen_rollback
);
int rtlmq_recover_queued_reset_locked(struct rtlmq_private *tp,
				      u32 reset_reasons
);
int rtlmq_close_device(struct rtlmq_private *tp);
int rtlmq_begin_removal_locked(struct rtlmq_private *tp);
int rtlmq_finish_removal_locked(struct rtlmq_private *tp);
int rtlmq_detach_and_close_locked(struct rtlmq_private *tp, bool prepare_wol);
int rtlmq_retire_failed_control_update_locked(struct rtlmq_private *tp);

/*
 * Resource release and generation reuse have separate ownership checks.
 * Final software cleanup does not imply hardware DMA has stopped.
 */
void rtlmq_clear_quiesce_phase(struct rtlmq_private *tp);
void rtlmq_mark_quiesce_phase(struct rtlmq_private *tp,
			      enum rtlmq_quiesce_phase phase);
bool rtlmq_quiesce_contract_satisfied(const struct rtlmq_private *tp);
bool rtlmq_final_release_allowed(const struct rtlmq_private *tp);
bool rtlmq_candidate_release_allowed(const struct rtlmq_private *tp);
bool rtlmq_candidate_discard_allowed(const struct rtlmq_private *tp);
bool rtlmq_candidate_resource_owner_valid(const struct rtlmq_private *tp);
bool rtlmq_release_allowed(const struct rtlmq_private *tp);
bool rtlmq_generation_reuse_allowed(const struct rtlmq_private *tp);
bool rtlmq_irq_release_allowed(const struct rtlmq_private *tp);
bool rtlmq_vector_release_allowed(const struct rtlmq_private *tp);
bool rtlmq_vector_storage_release_allowed(const struct rtlmq_private *tp);
bool rtlmq_ring_storage_release_allowed(const struct rtlmq_private *tp);
bool rtlmq_retired_generation_release_allowed(const struct rtlmq_private *tp);

void rtlmq_format_reset_reasons(char *buf, size_t size, u32 reasons);
void rtlmq_advance_reset_epoch_locked(struct rtlmq_private *tp);
void rtlmq_request_reset(struct rtlmq_private *tp,
			 u32 reset_reasons
);
void rtlmq_request_tx_timeout_validation(struct rtlmq_private *tp,
					 const struct rtlmq_tx_timeout_snapshot *snap,
					 u32 epoch,
					 unsigned long delay);
void rtlmq_reset_work(struct work_struct *work);

#define RTLMQ_RUNTIME_AUTOSUSPEND_DELAY_MS	5000

int rtlmq_runtime_pm_netdev_open(struct rtlmq_private *tp);
void rtlmq_runtime_pm_netdev_close(struct rtlmq_private *tp);
int rtlmq_runtime_pm_access_get(struct rtlmq_private *tp);
void rtlmq_runtime_pm_access_put(struct rtlmq_private *tp, int token);

#endif /* __RTLMQ_LIFECYCLE_H__ */
