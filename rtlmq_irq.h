/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef __RTLMQ_IRQ_H__
#define __RTLMQ_IRQ_H__

#include <linux/bitops.h>
#include <linux/build_bug.h>
#include <linux/interrupt.h>

#include "rtlmq_base.h"
/*
 * Vector/NAPI state and IRQ allocation, routing, scheduling, and teardown.
 */
#include <linux/netdevice.h>
#include <linux/u64_stats_sync.h>

#include "rtlmq_runtime.h"
#include "rtlmq_tx.h"

struct completion;

enum rtlmq_napi_schedule_source {
	RTLMQ_NAPI_SCHED_SOURCE_UNKNOWN = 0,
	RTLMQ_NAPI_SCHED_IRQ_RX,
	RTLMQ_NAPI_SCHED_IRQ_TX,
	RTLMQ_NAPI_SCHED_RX_RESCUE,
	RTLMQ_NAPI_SCHED_TX_RESWEEP,
	RTLMQ_NAPI_SCHED_XSK_WAKEUP,
	RTLMQ_NAPI_SCHED_OTHER_DEFERRED,
	RTLMQ_NAPI_SCHED_SOURCE_COUNT,
};

struct rtlmq_vector {
	struct rtlmq_private *priv;
	int irq;
	u16 index;
	u32 irq_mask;
	bool napi_added;
	bool napi_enabled;
	bool irq_requested;
	u8 num_irq_sources;
	u32 msix_data_mask;
	u32 msix_event_mask;
	struct rtlmq_irq_source irq_sources[RTLMQ_MAX_IRQ_SOURCES];
	/* PREEMPT_RT: spinlock_t; no sleeping inside these short critical sections. */
	/* Owns TX IRQ intent, mask writes, scheduling and rearm withdrawal. */
	spinlock_t napi_rearm_lock;
	struct napi_struct napi;
	struct delayed_work rx_refill_work;
	u32 generation;
	bool napi_rearm_allowed;
	bool uses_msix_message_routing;
	/* Requested TX-completion IRQ state, not a hardware mask readback. */
	bool tx_irq_armed_wanted;
	/*
	 * Set when the core refuses a NAPI completion (its own timer, a missed
	 * schedule, or an external poll owner) and therefore owns the next
	 * poll. Cleared when that poll starts. Producers must not convert this
	 * guaranteed service path into a hard IRQ.
	 */
	bool napi_service_pending;
	u8 xdp_tx_pending_mask;
	u32 tx_irq_mask;
	/* Last software mask write, not hardware readback. */
	u32 data_irq_mask_current;
	/* Advisory cache; available capabilities and current TX intent win. */
	u32 data_irq_mask_wanted;
	/* Layout capabilities, refreshed before runtime publication. */
	u32 data_irq_mask_rx_only;
	u32 data_irq_mask_tx_only;
	u32 data_irq_mask_rx_tx;
	unsigned long tx_resweep_hints;
	u64 tx_poll_seq;
	u16 rx_poll_start;
	u16 xsk_tx_poll_start;
	unsigned long rx_service_irq_cause_pending;
	/*
	 * Queue bits authorized by observed RX-capable hardware sources for the
	 * next positive-budget poll. Software scheduling never sets this mask.
	 */
	unsigned long rx_own_recheck_pending_mask;
	/*
	 * This vector's NAPI instance handles RX/TX completion for these queues.
	 * Store indices into tp->rx_rings[]/tp->tx_rings[] rather than pointers
	 * so ring replacement does not require updating every map entry.
	 */
	u16 rx_ring_indices[RTLMQ_IMPL_MAX_RX_QUEUES];
	u16 tx_ring_indices[RTLMQ_IMPL_MAX_TX_QUEUES];
	/* MSI-X message for each entry in the corresponding ring-index array. */
	u8 rx_message_ids[RTLMQ_IMPL_MAX_RX_QUEUES];
	u8 tx_message_ids[RTLMQ_IMPL_MAX_TX_QUEUES];
	u16 num_rx_rings;
	u16 num_tx_rings;
};

struct rtlmq_private;
struct rtlmq_runtime_config;
struct rtlmq_runtime_layout_state;
struct rtlmq_tx_ring;
struct rtlmq_vector_layout;

struct rtlmq_napi_rearm_plan {
	struct rtlmq_private *tp;
	/* Legacy vector shadow and device-wide register snapshot are distinct. */
	u32 legacy_data_mask;
	/* MSI-X RX admission and TX capability; TX intent stays live. */
	u32 rx_mask;
	u32 tx_mask;
	u32 legacy_mask;
	u32 generation;
	bool uses_msix_message_routing;
	bool completion_succeeded;
	bool valid;
};

/*
 * MSI-X / IRQ-mode lifecycle: message-routing eligibility, vector storage,
 * PCI IRQ fallback, quiesce, release, and close.
 */
bool rtlmq_staged_wants_msix_message_routing(const struct rtlmq_private *tp);
bool rtlmq_candidate_uses_msix_message_routing(const struct rtlmq_private *tp);
bool rtlmq_uses_msix_message_routing(const struct rtlmq_private *tp);
u16 rtlmq_required_msix_vectors(const struct rtlmq_runtime_config *cfg,
				const struct rtlmq_runtime_layout_state *layout);
int rtlmq_alloc_vectors(struct rtlmq_private *tp);
void rtlmq_free_vectors(struct rtlmq_private *tp);
int rtlmq_prepare_candidate_irq_vectors(struct rtlmq_private *tp);
int rtlmq_open_irqs(struct rtlmq_private *tp);
bool rtlmq_prepare_open_irq_fallback(struct rtlmq_private *tp,
				     enum rtlmq_irq_mode failed_mode,
				     int alloc_ret,
				     enum rtlmq_irq_mode *next_mode);
int rtlmq_quiesce_irqs(struct rtlmq_private *tp);
int rtlmq_quiesce_irqs_frozen(struct rtlmq_private *tp);
int rtlmq_release_irqs(struct rtlmq_private *tp);

enum rtlmq_irq_source_role {
	RTLMQ_IRQ_SOURCE_DATA = 0,
	RTLMQ_IRQ_SOURCE_EVENT,
};

enum {
	RTLMQ_VECTOR_TX_RESWEEP_HINT_TX_IRQ = 0,
	RTLMQ_VECTOR_TX_RESWEEP_HINT_FORCE,
};

#define RTLMQ_IRQ_SOURCE_KIND_RX	BIT(0)
#define RTLMQ_IRQ_SOURCE_KIND_TX	BIT(1)
#define RTLMQ_IRQ_SOURCE_KIND_EVENT	BIT(2)

/*
 * RX/rearm values program RX moderation. TX values program only TX
 * moderation. GRO and software-NAPI defaults are set before registration.
 * Disable hardware RX batching for latency; software NAPI still batches
 * active traffic. Both RX controls must be zero to disable moderation.
 */
#define RTLMQ_MAX_IRQ_COALESCE_USECS		1024
#define RTLMQ_MAX_IRQ_COALESCE_FRAMES		240
#define RTLMQ_DEFAULT_RX_IRQ_COALESCE_USECS	0
#define RTLMQ_DEFAULT_RX_IRQ_COALESCE_FRAMES	0
#define RTLMQ_DEFAULT_TX_IRQ_COALESCE_USECS 8
#define RTLMQ_DEFAULT_TX_IRQ_COALESCE_FRAMES 16
#define RTLMQ_DEFAULT_GRO_FLUSH_TIMEOUT_NS	160000UL
#define RTLMQ_IRQ_SOURCE_NONE	0xff

/*
 * IRQ handling:
 * - interrupt masking / ack / vector source control
 * - vector queue/ring mappings and interrupt entry
 */
enum rtlmq_rx_service_irq_cause_bit {
	RTLMQ_RX_SERVICE_IRQ_CAUSE_RX_BIT = 0,
	RTLMQ_RX_SERVICE_IRQ_CAUSE_TX_BIT,
	RTLMQ_RX_SERVICE_IRQ_CAUSE_UNKNOWN_BIT,
	RTLMQ_RX_SERVICE_REFILL_BIT, /* software retry, not an observed IRQ */
};

#define RTLMQ_RX_SERVICE_IRQ_CAUSE_RX \
	BIT_ULL(RTLMQ_RX_SERVICE_IRQ_CAUSE_RX_BIT)
#define RTLMQ_RX_SERVICE_IRQ_CAUSE_TX \
	BIT_ULL(RTLMQ_RX_SERVICE_IRQ_CAUSE_TX_BIT)
#define RTLMQ_RX_SERVICE_IRQ_CAUSE_UNKNOWN \
	BIT_ULL(RTLMQ_RX_SERVICE_IRQ_CAUSE_UNKNOWN_BIT)

static inline bool rtlmq_irq_layout_has_publication(u8 num_irq_sources,
						    u16 num_rx_rings,
						    u16 num_tx_rings)
{
	return num_irq_sources || num_rx_rings || num_tx_rings;
}

bool rtlmq_vector_layout_published(const struct rtlmq_vector_layout *vector);
u32 rtlmq_vector_wake_data_mask(const struct rtlmq_vector *vector);
u64 rtlmq_rx_ready_queue_mask(const struct rtlmq_vector *qv);
bool rtlmq_rx_ready(const struct rtlmq_vector *qv);
bool rtlmq_vector_has_tx_pending_desc(const struct rtlmq_vector *vector);
bool rtlmq_vector_has_tx_reclaimable_desc(const struct rtlmq_vector *vector,
					  const struct rtlmq_tx_reclaim_wait *wait);
bool rtlmq_tx_ring_has_reclaimable_desc(const struct rtlmq_tx_ring *ring,
					const struct rtlmq_tx_reclaim_wait *wait);
bool rtlmq_vector_has_tx_status_pending(const struct rtlmq_vector *vector);
bool rtlmq_runtime_vector_uses_msix_message_routing(const struct rtlmq_vector *vector);
u32 rtlmq_vector_data_irq_wanted_for_unmask(const struct rtlmq_vector *vector);
bool rtlmq_prepare_napi_rearm(struct rtlmq_vector *vector,
			      struct rtlmq_napi_rearm_plan *plan);
void rtlmq_apply_napi_rearm(struct rtlmq_vector *vector,
			    const struct rtlmq_napi_rearm_plan *plan);
bool rtlmq_coupled_tx_irq_suppression_active(const struct rtlmq_private *tp);
bool rtlmq_vector_has_shared_data_sources(const struct rtlmq_vector *vector);
void rtlmq_vector_note_napi_service_pending(struct rtlmq_vector *vector,
					    bool pending);
void rtlmq_sync_tx_irq_arming(struct rtlmq_private *tp);
void rtlmq_sync_tx_ring_irq_arming(struct rtlmq_tx_ring *ring);
void rtlmq_enable_intrs(struct rtlmq_private *tp);
void rtlmq_apply_data_irq_affinity(struct rtlmq_private *tp);
void rtlmq_disable_intrs(struct rtlmq_private *tp);
void rtlmq_mask_and_ack_intrs(struct rtlmq_private *tp);
int rtlmq_mask_and_ack_intrs_recovery(struct rtlmq_private *tp);
irqreturn_t rtlmq_vector_interrupt(int irq, void *data);
#ifdef CONFIG_NET_POLL_CONTROLLER
void rtlmq_poll_controller(struct net_device *ndev);
#endif
void rtlmq_reset_vector_irq_sources(struct rtlmq_vector *vector);
void rtlmq_reset_vector_queue_map(struct rtlmq_vector *vector);
void rtlmq_clear_unpublished_vector_queue_maps(struct rtlmq_private *tp);
void rtlmq_force_vector_tx_resweep(struct rtlmq_private *tp, u16 vector_index);
int rtlmq_stage_candidate_irq_routing(struct rtlmq_private *tp,
				      const struct rtlmq_runtime_config *cfg,
				      const struct rtlmq_runtime_layout_state *layout);
int rtlmq_validate_runtime_queue_ownership(struct rtlmq_private *tp,
					   const struct rtlmq_runtime_config *cfg,
					   const struct rtlmq_runtime_layout_state *layout);
int rtlmq_publish_vector_queue_layout(struct rtlmq_private *tp,
				      const struct rtlmq_runtime_config *cfg,
				      const struct rtlmq_runtime_layout_state *layout);
void rtlmq_withdraw_runtime_queue_ownership(struct rtlmq_private *tp);
int rtlmq_publish_runtime_queue_ownership(struct rtlmq_private *tp,
					  const struct rtlmq_runtime_config *cfg,
					  const struct rtlmq_runtime_layout_state *layout);

struct rtlmq_napi_poll_result {
	int rx_done;
	unsigned int tx_done;
	bool rx_budget_exhausted;
	struct rtlmq_tx_reclaim_wait tx_wait[RTLMQ_IMPL_MAX_TX_QUEUES];
};

struct rtlmq_vector_tx_poll {
	unsigned int pkts_cleaned;
	struct rtlmq_tx_reclaim_wait wait[RTLMQ_IMPL_MAX_TX_QUEUES];
};

int rtlmq_poll_vector_rx_first(struct rtlmq_private *tp,
			       struct rtlmq_vector *vector, int budget,
			       u64 rx_service_entry_irq_cause,
			       unsigned long rx_own_recheck_mask);
int rtlmq_napi_complete_vector(struct napi_struct *napi,
			       struct rtlmq_vector *vector, int budget,
			       const struct rtlmq_napi_poll_result *result);

struct rtlmq_vector_tx_poll
rtlmq_poll_vector_tx_desc_fast(struct rtlmq_private *tp,
			       struct rtlmq_vector *vector, int budget);

/*
 * NAPI owns vector service, completion, registration, and queue bindings.
 * IRQ code owns scheduling and interrupt rearm after completion.
 */
/* Caller holds napi_rearm_lock with NAPI and its rearm gate enabled. */
bool rtlmq_schedule_vector_napi(struct rtlmq_vector *vector,
				enum rtlmq_napi_schedule_source source);
int rtlmq_enable_vector_napi(struct rtlmq_vector *vector);
int rtlmq_disable_vector_napi(struct rtlmq_vector *vector);
void rtlmq_withdraw_vector_napi_rearm(struct rtlmq_vector *vector);
void rtlmq_retry_rx_refill(struct rtlmq_vector *vector);
void rtlmq_restore_vector_rx_refill(struct rtlmq_vector *vector);
void rtlmq_delete_vector_napi(struct rtlmq_vector *vector);
int rtlmq_validate_napi_enable_state(const struct rtlmq_private *tp,
				     const struct rtlmq_runtime_config *cfg,
				     const struct rtlmq_runtime_layout_state *layout);
void rtlmq_enable_active_napi(struct rtlmq_private *tp,
			      const struct rtlmq_runtime_config *cfg,
			      const struct rtlmq_runtime_layout_state *layout);
void rtlmq_clear_candidate_napi(struct rtlmq_private *tp);
void rtlmq_withdraw_napi_bindings(struct rtlmq_private *tp);
int rtlmq_validate_napi_bindings_layout(const struct rtlmq_private *tp,
					const struct rtlmq_runtime_config *cfg,
					const struct rtlmq_runtime_layout_state *layout);
int rtlmq_publish_napi_bindings_locked(struct rtlmq_private *tp,
				       const struct rtlmq_runtime_config *cfg,
				       const struct rtlmq_runtime_layout_state *layout);
int rtlmq_register_candidate_napi(struct rtlmq_private *tp);

void rtlmq_enable_msix_mask(struct rtlmq_private *tp, u32 mask);
void rtlmq_disable_msix_mask(struct rtlmq_private *tp, u32 mask);

u32 rtlmq_vector_data_irq_mask_wanted_cached(const struct rtlmq_vector *vector,
					     bool tx_armed);
u32 rtlmq_vector_requested_source_mask(const struct rtlmq_vector *vector,
				       enum rtlmq_irq_source_role role);
u32 rtlmq_filter_empty_rx_irq_mask(const struct rtlmq_vector *vector, u32 mask);
u32 rtlmq_vector_rx_data_irq_wanted(const struct rtlmq_vector *vector);
u32 rtlmq_vector_resolve_data_irq_wanted(struct rtlmq_vector *vector);
u32 rtlmq_vector_data_irq_hw_enable_mask(const struct rtlmq_vector *vector,
					 u32 wanted_mask);
void rtlmq_vector_refresh_data_irq_masks(struct rtlmq_vector *vector);
u32 rtlmq_vector_tx_irq_mask(const struct rtlmq_vector *vector);

/*
 * Coalescing policy and register encoding live in rtlmq_coalesce.c.
 * rtlmq_hw.c writes the encoded values to hardware.
 */
u32 rtlmq_default_rx_irq_coalesce_usecs(void);
u32 rtlmq_default_rx_irq_coalesce_frames(void);
u32 rtlmq_default_tx_irq_coalesce_usecs(void);
u32 rtlmq_default_tx_irq_coalesce_frames(void);
void rtlmq_set_requested_irq_coalesce_usecs(struct rtlmq_private *tp,
					    u32 usecs);
void rtlmq_set_requested_irq_coalesce_frames(struct rtlmq_private *tp,
					     u32 frames);
void rtlmq_set_requested_tx_irq_coalesce_usecs(struct rtlmq_private *tp,
					       u32 usecs);
void rtlmq_set_requested_tx_irq_coalesce_frames(struct rtlmq_private *tp,
						u32 frames);
u16 rtlmq_coalesce_encode_moderation(u32 usecs, u32 frames);
u16 rtlmq_coalesce_effective_tx_moderation(const struct rtlmq_private *tp,
					   u32 usecs, u32 frames);

#endif /* __RTLMQ_IRQ_H__ */
