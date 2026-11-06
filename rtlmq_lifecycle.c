// SPDX-License-Identifier: GPL-2.0-only

#include <linux/delay.h>
#include <linux/bitops.h>
#include <linux/completion.h>
#include <linux/netpoll.h>
#include <linux/rcupdate.h>
#include <linux/rtnetlink.h>
#include <linux/string.h>
#include <net/netdev_lock.h>

#include "rtlmq_chip.h"
#include "rtlmq_diag.h"
#include "rtlmq_hw.h"
#include "rtlmq_irq.h"
#include "rtlmq_runtime.h"
#include "rtlmq_lifecycle.h"
#include "rtlmq.h"
#include "rtlmq_ptp.h"
#include "rtlmq_runtime_accessors.h"
#include "rtlmq_regs.h"
#include "rtlmq_ring.h"
#include "rtlmq_trace.h"
#include "rtlmq_rx.h"
#include "rtlmq_tx.h"
#include "rtlmq_xdp.h"

/*
 * Completed teardown requires a hardware stop as well as PCI revocation.
 * Host observer retirement and PCI command/status do not drain posted DMA.
 */
#define RTLMQ_QUIESCE_REQUIRED_PHASES \
	(RTLMQ_QUIESCE_PUBLICATION_REVOKED | \
	 RTLMQ_QUIESCE_ASYNC_REACQUIRE_REVOKED | \
	 RTLMQ_QUIESCE_IRQ_SOURCE_REVOKED | \
	 RTLMQ_QUIESCE_NAPI_DISABLED | \
	 RTLMQ_QUIESCE_HARDWARE_STOPPED | \
	 RTLMQ_QUIESCE_DMA_REVOKED | \
	 RTLMQ_QUIESCE_RING_REGISTERS_REVOKED | \
	 RTLMQ_QUIESCE_IRQ_HANDLERS_RELEASED | \
	 RTLMQ_QUIESCE_RCU_DRAINED)

#define RTLMQ_IRQ_RELEASE_REQUIRED_PHASES \
	(RTLMQ_QUIESCE_PUBLICATION_REVOKED | \
	 RTLMQ_QUIESCE_ASYNC_REACQUIRE_REVOKED | \
	 RTLMQ_QUIESCE_IRQ_SOURCE_REVOKED | \
	 RTLMQ_QUIESCE_NAPI_DISABLED | \
	 RTLMQ_QUIESCE_RCU_DRAINED)

void rtlmq_clear_quiesce_phase(struct rtlmq_private *tp)
{
	if (!tp)
		return;

	WRITE_ONCE(tp->quiesce_phase, 0);
}

void rtlmq_mark_quiesce_phase(struct rtlmq_private *tp,
			      enum rtlmq_quiesce_phase phase)
{
	u32 phase_bits;

	if (!tp || !phase)
		return;

	phase_bits = READ_ONCE(tp->quiesce_phase);
	WRITE_ONCE(tp->quiesce_phase, phase_bits | phase);
}

static bool rtlmq_vector_host_flags_clear(const struct rtlmq_private *tp)
{
	unsigned int i;

	if (!tp->vectors)
		return true;

	for (i = 0; i < tp->allocated_vectors; i++) {
		const struct rtlmq_vector *vector = &tp->vectors[i];

		if (READ_ONCE(vector->irq_requested) ||
		    READ_ONCE(vector->napi_added) ||
		    READ_ONCE(vector->napi_enabled) ||
		    READ_ONCE(vector->napi_rearm_allowed))
			return false;
	}

	return true;
}

static bool rtlmq_vector_napi_flags_clear(const struct rtlmq_private *tp)
{
	unsigned int i;

	if (!tp->vectors)
		return true;

	for (i = 0; i < tp->allocated_vectors; i++) {
		const struct rtlmq_vector *vector = &tp->vectors[i];

		if (READ_ONCE(vector->napi_enabled) ||
		    READ_ONCE(vector->napi_rearm_allowed))
			return false;
	}

	return true;
}

bool rtlmq_quiesce_contract_satisfied(const struct rtlmq_private *tp)
{
	u32 phase;

	if (!tp)
		return false;

	phase = READ_ONCE(tp->quiesce_phase);
	if ((phase & RTLMQ_QUIESCE_REQUIRED_PHASES) !=
	    RTLMQ_QUIESCE_REQUIRED_PHASES)
		return false;
	if (!READ_ONCE(tp->dma_quiesced) ||
	    READ_ONCE(tp->runtime_published) ||
	    rtlmq_published_runtime_config(tp) ||
	    rtlmq_published_runtime_layout(tp))
		return false;
	if (tp->active_irq_vectors_allocated ||
	    tp->transaction.irq_vectors_allocated ||
	    tp->transaction.napi_registered_mask ||
	    tp->transaction.irq_requested_mask)
		return false;

	return rtlmq_vector_host_flags_clear(tp);
}

static bool
rtlmq_active_runtime_ownership_clear(const struct rtlmq_private *tp)
{
	return !READ_ONCE(tp->runtime_published) &&
	       !rtlmq_published_runtime_config(tp) &&
	       !rtlmq_published_runtime_layout(tp) &&
	       !READ_ONCE(tp->active_resource_generation) &&
	       !rtlmq_derive_active_rx_dma_ring_mask(tp) &&
	       !rtlmq_derive_active_tx_dma_ring_mask(tp) &&
	       !READ_ONCE(tp->active_irq_vectors_allocated);
}

bool rtlmq_candidate_release_allowed(const struct rtlmq_private *tp)
{
	unsigned int i;

	/*
	 * Candidate cleanup requires DMA stopped, no active generation, and no
	 * candidate vector able to schedule NAPI. Completed active teardown
	 * also satisfies these requirements.
	 */
	if (!tp)
		return false;
	if (rtlmq_quiesce_contract_satisfied(tp))
		return true;
	if (!READ_ONCE(tp->dma_quiesced) ||
	    !rtlmq_active_runtime_ownership_clear(tp))
		return false;

	if (!tp->vectors)
		return true;

	for (i = 0; i < tp->allocated_vectors; i++) {
		const struct rtlmq_vector *vector = &tp->vectors[i];

		if (READ_ONCE(vector->irq_requested) ||
		    READ_ONCE(vector->napi_enabled) ||
		    READ_ONCE(vector->napi_rearm_allowed))
			return false;
	}

	return true;
}

static bool rtlmq_candidate_irq_release_allowed(const struct rtlmq_private *tp)
{
	/*
	 * Partial setup may have requested IRQs before general cleanup is safe.
	 * With DMA stopped, runtime unpublished, and candidate NAPI disabled,
	 * free_irq() is the next cleanup step.
	 */
	if (!tp || !READ_ONCE(tp->dma_quiesced) ||
	    !rtlmq_active_runtime_ownership_clear(tp))
		return false;

	return rtlmq_vector_napi_flags_clear(tp);
}

bool rtlmq_candidate_discard_allowed(const struct rtlmq_private *tp)
{
	unsigned int i;
	unsigned int j;

	/*
	 * Discard may run while the active runtime is live, but only for private
	 * transaction objects. Check that candidate ring storage does not alias
	 * active ring storage.
	 */
	if (!tp)
		return false;

	for (i = 0; i < RTLMQ_IMPL_MAX_RX_QUEUES; i++) {
		const struct rtlmq_rx_ring *candidate =
			tp->transaction.rx_rings[i];

		if (!candidate)
			continue;
		if (candidate->priv != tp)
			return false;

		if (rtlmq_rx_ring_storage_aliases_active(tp, candidate))
			return false;

		if (tp->rx_ring_cold) {
			for (j = 0; j < rtlmq_owned_rx_ring_slots(tp); j++)
				if (candidate->cold == &tp->rx_ring_cold[j])
					return false;
		}
	}

	for (i = 0; i < RTLMQ_IMPL_MAX_TX_QUEUES; i++) {
		struct rtlmq_tx_desc *candidate_desc =
			tp->transaction.tx_descs[i];
		struct rtlmq_tx_info *candidate_infos =
			tp->transaction.tx_infos[i];
		struct rtlmq_tx_completion *candidate_completions =
			tp->transaction.tx_completions[i];
		struct rtlmq_tx_dma_txn *candidate_dma_txns =
			tp->transaction.tx_dma_txns[i];

		if (tp->tx_rings) {
			for (j = 0; j < rtlmq_owned_tx_ring_slots(tp); j++) {
				const struct rtlmq_tx_ring *active =
					&tp->tx_rings[j];

				if ((candidate_desc &&
				     (candidate_desc == active->desc ||
				      (active->desc &&
				       tp->transaction.tx_desc_dmas[i] ==
					       active->desc_dma))) ||
				    (candidate_infos &&
				     candidate_infos == active->infos) ||
				    (candidate_completions &&
				     candidate_completions == active->completions) ||
				    (candidate_dma_txns &&
				     candidate_dma_txns == active->dma_txns))
					return false;
			}
		}
	}

	return true;
}

bool rtlmq_candidate_resource_owner_valid(const struct rtlmq_private *tp)
{
	unsigned int i;

	/*
	 * Keep candidate DMA handles in the transaction until commit, separate
	 * from persistent active ring/vector arrays. A candidate generation may
	 * be zero during setup, otherwise it must be new and differ from active.
	 * NAPI enabled state is checked separately from resource association.
	 */
	if (!tp || !rtlmq_candidate_discard_allowed(tp))
		return false;

	if (tp->transaction.resource_generation &&
	    (tp->transaction.resource_generation !=
		     READ_ONCE(tp->runtime_generation_seq) ||
	     tp->transaction.resource_generation ==
		     READ_ONCE(tp->active_resource_generation)))
		return false;

	for (i = 0; i < RTLMQ_IMPL_MAX_RX_QUEUES; i++) {
		const struct rtlmq_rx_ring *candidate =
			tp->transaction.rx_rings[i];

		if (!candidate) {
			if (tp->transaction.rx_dma_ring_mask & BIT_ULL(i))
				return false;
			continue;
		}

		if (candidate->priv != tp ||
		    ((tp->transaction.rx_dma_ring_mask & BIT_ULL(i)) &&
		     !candidate->desc) ||
		    (candidate->desc &&
		     !(tp->transaction.rx_dma_ring_mask & BIT_ULL(i))))
			return false;
	}

	for (i = 0; i < RTLMQ_IMPL_MAX_TX_QUEUES; i++) {
		struct rtlmq_tx_desc *candidate_desc =
			tp->transaction.tx_descs[i];
		struct rtlmq_tx_info *candidate_infos =
			tp->transaction.tx_infos[i];
		struct rtlmq_tx_completion *candidate_completions =
			tp->transaction.tx_completions[i];
		struct rtlmq_tx_dma_txn *candidate_dma_txns =
			tp->transaction.tx_dma_txns[i];
		bool has_candidate =
			candidate_desc || candidate_infos || candidate_completions ||
			candidate_dma_txns;

		if (!has_candidate) {
			if (tp->transaction.tx_dma_ring_mask & BIT_ULL(i))
				return false;
			continue;
		}

		if (((tp->transaction.tx_dma_ring_mask & BIT_ULL(i)) &&
		     (!candidate_desc || !candidate_infos ||
		      !candidate_completions || !candidate_dma_txns)) ||
		    (candidate_desc &&
		     !(tp->transaction.tx_dma_ring_mask & BIT_ULL(i))))
			return false;
	}

	if (!tp->vectors)
		return true;

	for (i = 0; i < tp->allocated_vectors; i++) {
		const struct rtlmq_vector *vector = &tp->vectors[i];
		u64 vector_mask = BIT_ULL(i);

		if (tp->transaction.napi_registered_mask & vector_mask) {
			if (!vector->napi_added)
				return false;
		}
		if (tp->transaction.irq_requested_mask & vector_mask) {
			if (!vector->irq_requested)
				return false;
		}
	}

	return true;
}

bool rtlmq_release_allowed(const struct rtlmq_private *tp)
{
	if (!tp)
		return false;
	if (rtlmq_quiesce_contract_satisfied(tp) ||
	    rtlmq_candidate_release_allowed(tp))
		return true;

	return false;
}

static bool
rtlmq_unpublished_candidate_generation_allowed(const struct rtlmq_private *tp)
{
	if (!tp || !tp->transaction.profile.valid ||
	    !READ_ONCE(tp->transaction.resource_generation))
		return false;

	/*
	 * Install candidate storage before setting the active generation, with
	 * runtime unpublished. Check that handles remaining in the transaction
	 * do not alias slots already transferred by the commit loop.
	 */
	if (READ_ONCE(tp->runtime_published) ||
	    READ_ONCE(tp->active_resource_generation) ||
	    rtlmq_published_runtime_config(tp) ||
	    rtlmq_published_runtime_layout(tp))
		return false;

	return rtlmq_candidate_discard_allowed(tp);
}

bool rtlmq_generation_reuse_allowed(const struct rtlmq_private *tp)
{
	/*
	 * Changing a generation permits storage reuse. Require prior shutdown,
	 * retired storage, or an unpublished candidate. Changing the
	 * generation does not stop DMA or callbacks.
	 */
	return rtlmq_release_allowed(tp) ||
	       rtlmq_retired_generation_release_allowed(tp) ||
	       rtlmq_unpublished_candidate_generation_allowed(tp);
}

static bool
rtlmq_host_retirement_predecessors_satisfied(const struct rtlmq_private *tp)
{
	u32 phase = READ_ONCE(tp->quiesce_phase);

	if (READ_ONCE(tp->runtime_published) ||
	    rtlmq_published_runtime_config(tp) ||
	    rtlmq_published_runtime_layout(tp) ||
	    (phase & RTLMQ_IRQ_RELEASE_REQUIRED_PHASES) !=
		    RTLMQ_IRQ_RELEASE_REQUIRED_PHASES)
		return false;
	/*
	 * A terminal device cannot regain MMIO or service. free_irq()/NAPI
	 * deletion must drain host users even if DMA shutdown failed. This
	 * permits no descriptor, packet, pool or generation release/reuse.
	 */
	if ((READ_ONCE(tp->state) == RTLMQ_STATE_REMOVING ||
	     READ_ONCE(tp->state) == RTLMQ_STATE_DEAD) &&
	    READ_ONCE(tp->mmio_authority) == RTLMQ_MMIO_AUTHORITY_TERMINAL)
		return true;

	/* Preserve normal ordering and the existing frozen-channel exception. */
	return (phase & RTLMQ_QUIESCE_DMA_REVOKED) &&
	       ((phase & RTLMQ_QUIESCE_RING_REGISTERS_REVOKED) ||
		tp->platform_quiesce_owner == RTLMQ_PLATFORM_QUIESCE_PCI_ERROR);
}

bool rtlmq_irq_release_allowed(const struct rtlmq_private *tp)
{
	if (!tp)
		return false;
	if (rtlmq_candidate_irq_release_allowed(tp))
		return true;

	if (!rtlmq_host_retirement_predecessors_satisfied(tp))
		return false;

	return rtlmq_vector_napi_flags_clear(tp);
}

bool rtlmq_vector_release_allowed(const struct rtlmq_private *tp)
{
	u32 phase;

	if (!tp)
		return false;
	phase = READ_ONCE(tp->quiesce_phase);
	if (!(phase & RTLMQ_QUIESCE_IRQ_HANDLERS_RELEASED))
		return false;
	if (rtlmq_candidate_release_allowed(tp))
		return true;

	if (!rtlmq_host_retirement_predecessors_satisfied(tp))
		return false;

	return rtlmq_vector_host_flags_clear(tp);
}

bool rtlmq_vector_storage_release_allowed(const struct rtlmq_private *tp)
{
	/*
	 * Free the vector array only after release_irqs() has drained handlers,
	 * deleted every NAPI instance, and freed PCI vectors. Checking for no
	 * requested IRQs alone is insufficient: NAPI callbacks still use tp.
	 */
	if (!tp || !rtlmq_vector_release_allowed(tp))
		return false;
	if (READ_ONCE(tp->transaction.napi_registered_mask) ||
	    READ_ONCE(tp->transaction.irq_requested_mask))
		return false;
	return rtlmq_vector_host_flags_clear(tp);
}

bool rtlmq_ring_storage_release_allowed(const struct rtlmq_private *tp)
{
	unsigned int i;

	/*
	 * Free ring arrays and cold slabs last. Unlike
	 * rtlmq_release_allowed(), this also requires no active/candidate
	 * generation, registered callback, PHY, XSK mapping, or timeout dump
	 * that could still reference the storage.
	 */
	if (!tp || !rtlmq_release_allowed(tp))
		return false;
	if (READ_ONCE(tp->runtime_published) ||
	    READ_ONCE(tp->active_resource_generation) ||
	    tp->retained_live_rx || tp->retained_live_tx ||
	    rtlmq_derive_active_rx_dma_ring_mask(tp) ||
	    rtlmq_derive_active_tx_dma_ring_mask(tp) ||
	    rtlmq_active_rx_cold_heap_present(tp) ||
	    READ_ONCE(tp->transaction.rx_dma_ring_mask) ||
	    READ_ONCE(tp->transaction.tx_dma_ring_mask) ||
	    READ_ONCE(tp->transaction.napi_registered_mask) ||
	    READ_ONCE(tp->transaction.irq_requested_mask) ||
	    READ_ONCE(tp->transaction.irq_vectors_allocated))
		return false;
	if (tp->vectors && !rtlmq_vector_storage_release_allowed(tp))
		return false;
	if (tp->phydev || tp->mii_bus)
		return false;

	for (i = 0; i < RTLMQ_IMPL_MAX_RX_QUEUES; i++) {
		if (tp->transaction.rx_rings[i])
			return false;
		if (tp->xsk_dma_pools[i].pool ||
		    READ_ONCE(tp->xsk_dma_pools[i].users))
			return false;
	}
	for (i = 0; i < RTLMQ_IMPL_MAX_TX_QUEUES; i++) {
		if (rtlmq_candidate_tx_storage_present(tp, i))
			return false;
	}
	if (tp->rx_rings) {
		for (i = 0; i < rtlmq_owned_rx_ring_slots(tp); i++) {
			const struct rtlmq_rx_ring *ring = &tp->rx_rings[i];

			if (READ_ONCE(ring->desc) ||
			    READ_ONCE(ring->desc_dma) ||
			    READ_ONCE(ring->buffers) ||
			    READ_ONCE(ring->page_pool))
				return false;
		}
	}
	if (tp->tx_rings) {
		for (i = 0; i < rtlmq_owned_tx_ring_slots(tp); i++) {
			const struct rtlmq_tx_ring *ring = &tp->tx_rings[i];

			if (READ_ONCE(ring->desc) ||
			    READ_ONCE(ring->desc_dma) ||
			    READ_ONCE(ring->infos) ||
			    READ_ONCE(ring->completions))
				return false;
		}
	}

	return true;
}

bool rtlmq_final_release_allowed(const struct rtlmq_private *tp)
{
	/*
	 * This permission records terminal host cleanup, not DMA retirement.
	 * FINAL_RELEASE is not evidence that the failed device can no longer
	 * access memory.
	 */
	return tp &&
	       (READ_ONCE(tp->quiesce_phase) & RTLMQ_QUIESCE_FINAL_RELEASE) &&
	       (READ_ONCE(tp->state) == RTLMQ_STATE_REMOVING ||
		READ_ONCE(tp->state) == RTLMQ_STATE_DEAD) &&
	       READ_ONCE(tp->mmio_authority) == RTLMQ_MMIO_AUTHORITY_TERMINAL &&
	       rtlmq_host_retirement_predecessors_satisfied(tp) &&
	       rtlmq_vector_storage_release_allowed(tp);
}

bool rtlmq_retired_generation_release_allowed(const struct rtlmq_private *tp)
{
	if (!tp)
		return false;

	/*
	 * The previous generation was fully quiesced before this flag was set.
	 * The replacement may already have enabled bus mastering, so its current
	 * dma_quiesced value does not determine whether old storage can be freed.
	 */
	return READ_ONCE(tp->retired_generation_quiesced) &&
	       !READ_ONCE(tp->runtime_published) &&
	       !READ_ONCE(tp->active_resource_generation);
}

static int rtlmq_deactivate_device(struct rtlmq_private *tp);
static void rtlmq_withdraw_open_queue_generation(struct rtlmq_private *tp);
static void
rtlmq_withdraw_open_runtime_visibility_locked(struct rtlmq_private *tp);

void rtlmq_netpoll_guard_acquire_locked(struct rtlmq_private *tp,
					struct rtlmq_netpoll_guard *guard)
{
	struct netpoll_info *info = NULL;

	if (!guard)
		return;

	guard->info = NULL;
	guard->armed = false;
	if (!tp || !tp->ndev)
		return;
	lockdep_assert_held(&tp->control_lock);
	ASSERT_RTNL();

#ifdef CONFIG_NETPOLL
	/*
	 * netpoll_poll_disable() is not exported to modules. Take the same
	 * semaphore as core open/close; RTNL keeps npinfo alive.
	 */
	info = rtnl_dereference(tp->ndev->npinfo);
	if (info)
		down(&info->dev_lock);
#endif
	guard->info = info;
	guard->armed = true;
}

void rtlmq_netpoll_guard_release_locked(struct rtlmq_private *tp,
					struct rtlmq_netpoll_guard *guard)
{
	if (!guard || !guard->armed)
		return;
	if (tp) {
		lockdep_assert_held(&tp->control_lock);
		ASSERT_RTNL();
	}

	if (guard->info)
		up(&guard->info->dev_lock);
	guard->info = NULL;
	guard->armed = false;
}

static void rtlmq_synchronize_live_irq_handlers(struct rtlmq_private *tp)
{
	unsigned int i;

	for (i = 0; tp->vectors && i < tp->allocated_vectors; i++) {
		struct rtlmq_vector *vector = &tp->vectors[i];
		bool source_synced = false;
		unsigned int j;

		if (!READ_ONCE(vector->irq_requested))
			continue;

		for (j = 0; j < READ_ONCE(vector->num_irq_sources); j++) {
			const struct rtlmq_irq_source *source =
				&vector->irq_sources[j];

			if (!READ_ONCE(source->requested) ||
			    READ_ONCE(source->irq) < 0)
				continue;
			synchronize_irq(READ_ONCE(source->irq));
			source_synced = true;
		}

		if (!source_synced && READ_ONCE(vector->irq) >= 0)
			synchronize_irq(READ_ONCE(vector->irq));
	}
}

static int rtlmq_resume_live_mac_locked(struct rtlmq_private *tp,
					struct rtlmq_live_service_pause *pause)
{
	const struct rtlmq_runtime_config *cfg;
	int ret;

	if (!tp || !pause)
		return -EINVAL;
	lockdep_assert_held(&tp->control_lock);

	if (!pause->mac_paused)
		return 0;

	cfg = rtlmq_candidate_or_published_runtime_config(tp);
	if (!cfg) {
		/*
		 * Ring replacement has committed the active profile but not
		 * runtime_published or the active generation. Neither candidate
		 * nor published accessors can return it. Use the active profile
		 * only for this all-engines-stopped resume case.
		 */
		if (pause->scope != RTLMQ_LIVE_MAC_PAUSE_ALL_ENGINES ||
		    READ_ONCE(tp->runtime_published) ||
		    READ_ONCE(tp->active_resource_generation) ||
		    !rtlmq_retired_generation_release_allowed(tp))
			return -EINVAL;
		cfg = rtlmq_active_runtime_config(tp);
	}
	if (!cfg)
		return -EINVAL;

	ret = rtlmq_resume_mac_datapath_locked(tp, cfg);
	if (ret)
		return ret;
	pause->mac_paused = false;
	return 0;
}

static int
rtlmq_restore_live_host_service_locked(struct rtlmq_private *tp,
				       struct rtlmq_live_service_pause *pause)
{
	u64 restored_napi_mask;
	int primary_ret = 0;
	unsigned int i;
	int ret;

	if (!tp || !pause)
		return -EINVAL;
	lockdep_assert_held(&tp->control_lock);
	if (pause->mac_paused)
		return -EBUSY;
	restored_napi_mask = pause->napi_mask;

	if (pause->ptp_paused) {
		ret = rtlmq_ptp_resume_datapath_locked(tp);
		if (ret && !primary_ret)
			primary_ret = ret;
		pause->ptp_paused = false;
	}

	for (i = 0; tp->vectors && i < tp->allocated_vectors; i++) {
		struct rtlmq_vector *vector = &tp->vectors[i];

		if (!(pause->napi_mask & BIT_ULL(i)))
			continue;
		ret = rtlmq_enable_vector_napi(vector);
		if (ret && ret != -EALREADY && !primary_ret)
			primary_ret = ret;
	}
	pause->napi_mask = 0;

	rtlmq_mask_and_ack_intrs(tp);
	rtlmq_enable_intrs(tp);
	if (pause->tx_disabled) {
		if (!tp->tc.link_waiting)
			rtlmq_serial_tx_resume(tp);
		rtlmq_sync_tx_subqueues(tp);
		pause->tx_disabled = false;
	}

	/* Pausing cancels retries even on empty queues that cannot raise an IRQ. */
	if (!primary_ret)
		for (i = 0; tp->vectors && i < tp->allocated_vectors; i++)
			if (restored_napi_mask & BIT_ULL(i))
				rtlmq_restore_vector_rx_refill(&tp->vectors[i]);

	return primary_ret;
}

int rtlmq_restore_live_service_locked(struct rtlmq_private *tp,
				      struct rtlmq_live_service_pause *pause)
{
	int ret;

	ret = rtlmq_resume_live_mac_locked(tp, pause);
	if (ret)
		return ret;

	return rtlmq_restore_live_host_service_locked(tp, pause);
}

int rtlmq_pause_live_service_locked(struct rtlmq_private *tp,
				    struct rtlmq_live_service_pause *pause,
				    enum rtlmq_live_mac_pause_scope scope,
				    bool preserve_fence_on_failure)
{
	unsigned int i;
	int ret;

	if (!tp || !pause)
		return -EINVAL;
	if (scope != RTLMQ_LIVE_MAC_PAUSE_RX_ONLY &&
	    scope != RTLMQ_LIVE_MAC_PAUSE_ALL_ENGINES)
		return -EINVAL;
	lockdep_assert_held(&tp->control_lock);
	memset(pause, 0, sizeof(*pause));
	pause->scope = scope;

	netif_tx_disable(tp->ndev);
	pause->tx_disabled = true;
	rtlmq_serial_tx_pause(tp);
	rtlmq_flush_pending_tx_doorbells(tp);

	/*
	 * StopReq is global. Drain the sole PTP TX request before IRQ/NAPI
	 * shutdown so the timestamp worker and completion path can still
	 * finish it. Keep the PHC running.
	 */
	{ ret = rtlmq_ptp_pause_datapath_locked(tp, preserve_fence_on_failure); }
	if (ret)
		goto err_restore;
	pause->ptp_paused = true;

	/*
	 * Block completion rearm before masking device interrupts. Keep IRQ/NAPI
	 * registrations and vector mappings for resume; only pause service.
	 */
	for (i = 0; tp->vectors && i < tp->allocated_vectors; i++) {
		struct rtlmq_vector *vector = &tp->vectors[i];

		if (!READ_ONCE(vector->napi_enabled))
			continue;
		pause->napi_mask |= BIT_ULL(i);
		rtlmq_withdraw_vector_napi_rearm(vector);
	}
	rtlmq_mask_and_ack_intrs(tp);

	for (i = 0; tp->vectors && i < tp->allocated_vectors; i++) {
		struct rtlmq_vector *vector = &tp->vectors[i];

		if (!(pause->napi_mask & BIT_ULL(i)))
			continue;
		ret = rtlmq_disable_vector_napi(vector);
		if (ret)
			goto err_restore;
	}
	rtlmq_synchronize_live_irq_handlers(tp);
	synchronize_net();
	/*
	 * RX NAPI may have queued XDP_TX after the first flush. Flush again
	 * after napi_disable() has stopped every producer, before StopReq.
	 */
	rtlmq_flush_pending_tx_doorbells(tp);

	switch (scope) {
	case RTLMQ_LIVE_MAC_PAUSE_RX_ONLY:
		ret = rtlmq_pause_mac_datapath_locked(tp);
		break;
	case RTLMQ_LIVE_MAC_PAUSE_ALL_ENGINES:
		ret = rtlmq_pause_all_mac_datapath_locked(tp);
		break;
	default:
		return -EINVAL;
	}
	if (ret) {
		/*
		 * StopReq may have been accepted before a strict drain timeout.
		 * Clear it through the normal datapath-enable helper before
		 * restoring host service.
		 */
		pause->mac_paused = true;
		goto err_restore;
	}
	pause->mac_paused = true;

	return 0;

err_restore:
	if (!preserve_fence_on_failure) {
		int restore_ret = rtlmq_restore_live_service_locked(tp, pause);

		if (restore_ret) {
			int close_ret;

			/* Failed rollback cannot leave the old runtime published. */
			netif_device_detach(tp->ndev);
			close_ret = rtlmq_close_device(tp);
			netdev_err(tp->ndev,
				   "live pause rollback failed pause=%d restore=%d close=%d\n",
				   ret, restore_ret, close_ret);
			if (tp->state == RTLMQ_STATE_PROBED &&
			    tp->platform_quiesce_owner == RTLMQ_PLATFORM_QUIESCE_NONE)
				rtlmq_request_reset(tp, BIT(RTLMQ_RESET_REASON_RUNTIME_RECONFIG)
				);
		}
	}
	return ret;
}

/*
 * Lifecycle transitions, serialized by control_lock:
 *
 * Open/reopen builds candidate rings, IRQs, and NAPI with DMA disabled.
 * Commit installs the active profile and queue mappings before enabling DMA
 * and setting runtime_published. Failure tears down the attempted runtime
 * and restores the requested configuration.
 *
 * Close clears runtime_published, stops service and DMA, releases resources,
 * and returns to PROBED. If quiescence fails, retain resources rather than
 * free storage still reachable by hardware or callbacks.
 *
 * Live channel and MTU changes validate the request before reset/reopen.
 * Rollback restores the saved request and attempts to reopen it. Eligible
 * ring-size changes retain the old rings, stop both MAC engines, and swap bases
 * without dropping carrier. Shared XSK pools or unsupported profiles require
 * a full rebuild. A failed restore requires DMA shutdown and recovery.
 *
 * Live XDP replacement drains old readers. It also rebuilds hardware if the
 * existing RX geometry or page-pool DMA direction cannot support the new program.
 * XSK memory-model replacement pauses RX while preserving TX enable and cursor.
 * Detach must remove all references to the old pool before returning, even if
 * restoring ordinary RX requires later recovery.
 *
 * Down-state changes save configuration for the next open. RSS-only changes
 * stage and commit RSS separately; feature/filter/moderation updates use their
 * serialized register-update paths.
 *
 * Watchdog and queued resets rebuild the runtime. Failed reopen leaves the
 * netdev detached in PROBED, or retains unpublished resources if shutdown fails.
 *
 * PM/AER saves the committed request before stopping service. Resume restores
 * MMIO access and reopens only if the interface was running. AER slot reset
 * performs recovery-only MMIO and teardown; reopening waits for PCI resume.
 * Permanent PCI failure disables hardware access irreversibly.
 *
 * Remove enters REMOVING before unregister, then DEAD after resource release.
 * Failed shutdown retains resources in REMOVING until final core teardown.
 * That teardown releases software ownership without proving hardware DMA stopped.
 */

int rtlmq_grant_ordinary_mmio_locked(struct rtlmq_private *tp)
{
	enum rtlmq_mmio_authority authority;
	void __iomem *mmio;

	if (!tp)
		return -EINVAL;
	lockdep_assert_held(&tp->control_lock);

	authority = READ_ONCE(tp->mmio_authority);
	mmio = READ_ONCE(tp->mmio_addr);
	if (!mmio || authority == RTLMQ_MMIO_AUTHORITY_TERMINAL ||
	    READ_ONCE(tp->state) == RTLMQ_STATE_DEAD)
		return -ENODEV;
	if (authority != RTLMQ_MMIO_AUTHORITY_REVOKED &&
	    authority != RTLMQ_MMIO_AUTHORITY_RECOVERY &&
	    authority != RTLMQ_MMIO_AUTHORITY_ORDINARY)
		return -EINVAL;

	/* Pairs with acquire loads before ordinary MMIO access. */
	smp_store_release(&tp->mmio_authority,
			  RTLMQ_MMIO_AUTHORITY_ORDINARY);
	return 0;
}

int rtlmq_grant_recovery_mmio_locked(struct rtlmq_private *tp)
{
	enum rtlmq_mmio_authority authority;
	void __iomem *mmio;

	if (!tp)
		return -EINVAL;
	lockdep_assert_held(&tp->control_lock);

	authority = READ_ONCE(tp->mmio_authority);
	mmio = READ_ONCE(tp->mmio_addr);
	if (!mmio || authority == RTLMQ_MMIO_AUTHORITY_TERMINAL ||
	    READ_ONCE(tp->state) == RTLMQ_STATE_DEAD)
		return -ENODEV;
	if (tp->platform_quiesce_owner !=
	    RTLMQ_PLATFORM_QUIESCE_PCI_ERROR ||
	    authority != RTLMQ_MMIO_AUTHORITY_REVOKED)
		return -EINVAL;

	/*
	 * Keep normal MMIO blocked. Only PCI recovery may obtain the retained
	 * mapping through rtlmq_mmio_recovery_addr().
	 */
	/* Pairs with acquire loads in rtlmq_mmio_recovery_addr(). */
	smp_store_release(&tp->mmio_authority,
			  RTLMQ_MMIO_AUTHORITY_RECOVERY);
	return 0;
}

void rtlmq_revoke_mmio_locked(struct rtlmq_private *tp)
{
	if (!tp)
		return;
	lockdep_assert_held(&tp->control_lock);

	if (READ_ONCE(tp->mmio_authority) ==
	    RTLMQ_MMIO_AUTHORITY_TERMINAL)
		return;

	/*
	 * Block new MMIO users, then let the caller drain IRQ/NAPI/TX/work
	 * callbacks before freeing storage. Keep mmio_addr unchanged so readers
	 * that passed the access check cannot dereference a cleared pointer.
	 */
	/* Pairs with acquire loads in the MMIO access checks. */
	smp_store_release(&tp->mmio_authority,
			  RTLMQ_MMIO_AUTHORITY_REVOKED);
}

void rtlmq_terminalize_mmio_locked(struct rtlmq_private *tp)
{
	if (!tp)
		return;
	lockdep_assert_held(&tp->control_lock);

	/* Pairs with acquire loads in the MMIO access checks. */
	smp_store_release(&tp->mmio_authority,
			  RTLMQ_MMIO_AUTHORITY_TERMINAL);
}

static bool
rtlmq_candidate_runtime_has_resources(const struct rtlmq_private *tp)
{
	unsigned int i;

	if (!tp)
		return false;
	if (tp->transaction.profile.valid ||
	    tp->transaction.profile.layout.valid ||
	    tp->transaction.resource_generation ||
	    tp->transaction.rx_dma_ring_mask ||
	    tp->transaction.tx_dma_ring_mask ||
	    tp->transaction.napi_registered_mask ||
	    tp->transaction.irq_requested_mask ||
	    tp->transaction.irq_vectors_allocated ||
	    tp->transaction.rss_valid ||
	    tp->transaction.xdp_staged)
		return true;

	for (i = 0; i < RTLMQ_IMPL_MAX_RX_QUEUES; i++)
		if (tp->transaction.rx_rings[i])
			return true;

	for (i = 0; i < RTLMQ_IMPL_MAX_TX_QUEUES; i++)
		if (rtlmq_candidate_tx_storage_present(tp, i))
			return true;

	return false;
}

/*
 * Abort the private candidate after stopping DMA and releasing candidate
 * IRQ/NAPI service. Otherwise a failed open leaves rings allocated and the
 * next init_rings() returns -EBUSY.
 */
static int rtlmq_abort_candidate_runtime_locked(struct rtlmq_private *tp,
						const char *where)
{
	unsigned int i;

	lockdep_assert_held(&tp->control_lock);

	if (!rtlmq_candidate_runtime_has_resources(tp))
		return 0;

	{
		bool owner_valid = rtlmq_candidate_resource_owner_valid(tp);

		if (!owner_valid) {
			netdev_err(tp->ndev,
				   "%s cannot abort candidate runtime owner_valid=%u dma_quiesced=%u state=%s\n",
				   where ? where : "candidate_abort",
				   owner_valid,
				   READ_ONCE(tp->dma_quiesced),
				   rtlmq_state_name(READ_ONCE(tp->state)));
			return -EBUSY;
		}
	}

	rtlmq_clear_candidate_profile(tp);

	if (rtlmq_candidate_runtime_has_resources(tp)) {
		netdev_err(tp->ndev,
			   "%s left candidate resources after abort rxmask=%#llx txmask=%#llx napi=%#llx irq=%#llx vectors=%u\n",
			   where ? where : "candidate_abort",
			   (unsigned long long)tp->transaction.rx_dma_ring_mask,
			   (unsigned long long)tp->transaction.tx_dma_ring_mask,
			   (unsigned long long)tp->transaction.napi_registered_mask,
			   (unsigned long long)tp->transaction.irq_requested_mask,
			   tp->transaction.irq_vectors_allocated);
		for (i = 0; i < RTLMQ_IMPL_MAX_RX_QUEUES; i++)
			WARN_ON_ONCE(tp->transaction.rx_rings[i]);
		for (i = 0; i < RTLMQ_IMPL_MAX_TX_QUEUES; i++)
			WARN_ON_ONCE(rtlmq_candidate_tx_storage_present(tp, i));
		return -EBUSY;
	}

	return 0;
}

static void rtlmq_mark_retired_generation_quiesced(struct rtlmq_private *tp)
{
	if (!tp)
		return;

	if (WARN_ON_ONCE(!rtlmq_quiesce_contract_satisfied(tp)))
		return;

	WRITE_ONCE(tp->retired_generation_quiesced, true);
}

static int
rtlmq_mark_live_retired_generation_quiesced(struct rtlmq_private *tp,
					    const struct rtlmq_live_service_pause *pause)
{
	if (!tp || !pause)
		return -EINVAL;
	lockdep_assert_held(&tp->control_lock);

	/*
	 * Live ring replacement requires stopped producers/consumers, both MAC
	 * engines disabled with empty-state readback, and runtime unpublished.
	 * The caller must have saved and cleared tp's active generation.
	 */
	if (!pause->mac_paused ||
	    pause->scope != RTLMQ_LIVE_MAC_PAUSE_ALL_ENGINES ||
	    READ_ONCE(tp->runtime_published) ||
	    READ_ONCE(tp->active_resource_generation))
		return -EBUSY;

	WRITE_ONCE(tp->retired_generation_quiesced, true);
	return 0;
}

static void rtlmq_withdraw_active_xdp_locked(struct rtlmq_private *tp)
{
	struct bpf_prog *old_prog;

	lockdep_assert_held(&tp->control_lock);

	old_prog = xchg(&tp->xdp_prog, NULL);
	if (old_prog)
		bpf_prog_put(old_prog);
}

static bool rtlmq_live_xdp_tx_ready(const struct rtlmq_private *tp)
{
	u16 published_txq;
	u64 expected_mask;
	u16 i;

	if (!tp->tx_rings)
		return false;

	published_txq = rtlmq_published_num_tx_queues(tp);
	if (!is_power_of_2(published_txq) ||
	    published_txq > RTLMQ_IMPL_MAX_TX_QUEUES)
		return false;

	expected_mask = GENMASK_ULL(published_txq - 1, 0);
	if (rtlmq_derive_active_tx_dma_ring_mask(tp) != expected_mask)
		return false;

	for (i = 0; i < published_txq; i++) {
		struct rtlmq_tx_ring *ring = &tp->tx_rings[i];

		if (READ_ONCE(tp->txq2ring[i]) != ring ||
		    !rtlmq_tx_ring_boundary_ready(ring))
			return false;
	}

	return true;
}

void rtlmq_sync_xdp_features_locked(struct rtlmq_private *tp)
{
	xdp_features_t feature_mask;
	bool capability_visible;
	bool runtime_committed;
	bool netdev_visible;
	bool tx_ready;

	lockdep_assert_held(&tp->control_lock);
	netdev_visible = tp->ndev->reg_state == NETREG_REGISTERED ||
			 tp->ndev->reg_state == NETREG_UNREGISTERING;
	if (netdev_visible)
		netdev_lock(tp->ndev);

	/*
	 * Advertise BASIC, REDIRECT, zero-copy, and metadata while down so
	 * userspace can query support and bind XSK pools before open.
	 * Do not expose RX/TX resources until commit. Remove clears capabilities
	 * before unregister completes.
	 */
	feature_mask = rtlmq_xdp_feature_mask(tp);
	capability_visible = feature_mask &&
			     READ_ONCE(tp->state) != RTLMQ_STATE_REMOVING &&
			     READ_ONCE(tp->state) != RTLMQ_STATE_DEAD;
	if (capability_visible) {
		WRITE_ONCE(tp->ndev->xdp_metadata_ops,
			   rtlmq_get_xdp_metadata_ops(tp));
		WRITE_ONCE(tp->ndev->xsk_tx_metadata_ops,
			   rtlmq_get_xsk_tx_metadata_ops(tp));
		WRITE_ONCE(tp->ndev->xdp_zc_max_segs,
			   rtlmq_xdp_zc_max_segs(tp));
		xdp_set_features_flag_locked(tp->ndev, feature_mask);
	} else {
		WRITE_ONCE(tp->ndev->xdp_metadata_ops, NULL);
		WRITE_ONCE(tp->ndev->xsk_tx_metadata_ops, NULL);
		WRITE_ONCE(tp->ndev->xdp_zc_max_segs, 0);
		xdp_set_features_flag_locked(tp->ndev, 0);
	}

	runtime_committed = READ_ONCE(tp->state) == RTLMQ_STATE_OPEN &&
			    READ_ONCE(tp->runtime_published) &&
			    rtlmq_published_runtime_config(tp);

	/*
	 * Redirect-target TX requires every TX queue in the running generation.
	 * Clear it before ring teardown and restore it at the final OPEN commit.
	 */
	tx_ready = runtime_committed &&
		   feature_mask &&
		   rtlmq_live_xdp_tx_ready(tp);
	if (tx_ready)
		xdp_features_set_redirect_target_locked(tp->ndev, true);
	else
		xdp_features_clear_redirect_target_locked(tp->ndev);

	if (netdev_visible)
		netdev_unlock(tp->ndev);
}

void rtlmq_sync_netdev_tso_limits(struct rtlmq_private *tp)
{
	struct net_device *ndev;
	u32 max_size;

	lockdep_assert_held(&tp->control_lock);

	if (!tp || !tp->ndev)
		return;

	ndev = tp->ndev;
	max_size = rtlmq_qualified_tso_size_limit(tp);
	if (!max_size)
		max_size = RTLMQ_TSO_MAX_SKB_BYTES;
	/*
	 * Reapply TSO/GSO limits when committing OPEN, not just at probe.
	 */
	netif_set_tso_max_size(ndev, max_size);
	netif_set_tso_max_segs(ndev, 64);
}

static bool rtlmq_valid_state_transition(enum rtlmq_lifecycle_state from,
					 enum rtlmq_lifecycle_state to)
{
	switch (from) {
	case RTLMQ_STATE_NEW:
		return to == RTLMQ_STATE_PROBED;
	case RTLMQ_STATE_PROBED:
		return to == RTLMQ_STATE_OPENING ||
		       to == RTLMQ_STATE_REMOVING ||
		       to == RTLMQ_STATE_DEAD;
	case RTLMQ_STATE_OPENING:
		return to == RTLMQ_STATE_OPEN ||
		       to == RTLMQ_STATE_PROBED ||
		       to == RTLMQ_STATE_REMOVING;
	case RTLMQ_STATE_OPEN:
		return to == RTLMQ_STATE_RESETTING ||
		       to == RTLMQ_STATE_CLOSING ||
		       to == RTLMQ_STATE_REMOVING;
	case RTLMQ_STATE_RESETTING:
		return to == RTLMQ_STATE_OPEN ||
		       to == RTLMQ_STATE_PROBED ||
		       to == RTLMQ_STATE_REMOVING;
	case RTLMQ_STATE_CLOSING:
		return to == RTLMQ_STATE_PROBED ||
		       to == RTLMQ_STATE_REMOVING;
	case RTLMQ_STATE_REMOVING:
		return to == RTLMQ_STATE_DEAD;
	case RTLMQ_STATE_DEAD:
	default:
		return false;
	}
}

int rtlmq_set_state_locked(struct rtlmq_private *tp,
			   enum rtlmq_lifecycle_state state)
{
	enum rtlmq_lifecycle_state old_state = tp->state;

	lockdep_assert_held(&tp->control_lock);

	if (!rtlmq_valid_state_transition(tp->state, state))
		return -EINVAL;

	/*
	 * Clear runtime_published before leaving OPEN. The final open commit
	 * sets it again after validation.
	 */
	if (state != RTLMQ_STATE_OPEN)
		rtlmq_revoke_runtime_publication_locked(tp);
	tp->state = state;
	if (trace_rtlmq_lifecycle_transition_enabled())
		trace_rtlmq_lifecycle_transition(tp->ndev->name, old_state, state,
						 tp->active_resource_generation);
	return 0;
}

int rtlmq_admit_control_change_locked(struct rtlmq_private *tp,
				      bool allow_probed)
{
	lockdep_assert_held(&tp->control_lock);

	if (tp->state == RTLMQ_STATE_REMOVING ||
	    tp->state == RTLMQ_STATE_DEAD)
		return -ENODEV;
	if (tp->platform_quiesce_owner != RTLMQ_PLATFORM_QUIESCE_NONE)
		return -EBUSY;

	switch (tp->state) {
	case RTLMQ_STATE_OPEN:
		return 0;
	case RTLMQ_STATE_PROBED:
		return allow_probed ? 0 : -EBUSY;
	case RTLMQ_STATE_REMOVING:
	case RTLMQ_STATE_DEAD:
		return -ENODEV;
	default:
		return -EBUSY;
	}
}

bool rtlmq_can_apply_down_admin_change_locked(const struct rtlmq_private *tp)
{
	if (!tp || !tp->ndev)
		return false;
	lockdep_assert_held(&tp->control_lock);

	return !netif_running(tp->ndev) &&
	       READ_ONCE(tp->state) == RTLMQ_STATE_PROBED &&
	       rtlmq_published_irq_mode(tp) == RTLMQ_IRQ_MODE_NONE;
}

bool rtlmq_can_apply_live_admin_rebuild_locked(const struct rtlmq_private *tp)
{
	if (!tp || !tp->ndev)
		return false;
	lockdep_assert_held(&tp->control_lock);

	return netif_running(tp->ndev) &&
	       READ_ONCE(tp->state) == RTLMQ_STATE_OPEN;
}

/*
 * A committed request has both a valid expanded configuration and a valid
 * layout. Check both before saving a recovery snapshot so reset and PM/AER
 * cannot restore partially prepared requested state.
 */
static bool rtlmq_requested_runtime_committed(const struct rtlmq_private *tp)
{
	return tp && tp->requested_profile.valid &&
	       tp->requested_profile.layout.valid;
}

static bool
rtlmq_requested_snapshot_committed(const struct rtlmq_requested_runtime_snapshot *snapshot)
{
	return snapshot && snapshot->profile.valid &&
	       snapshot->profile.layout.valid;
}

int rtlmq_capture_platform_recovery_committed_locked(struct rtlmq_private *tp)
{
	lockdep_assert_held(&tp->control_lock);

	if ((tp->state != RTLMQ_STATE_OPEN &&
	     tp->state != RTLMQ_STATE_PROBED) ||
	    !rtlmq_requested_runtime_committed(tp))
		return -EBUSY;

	/*
	 * Save the committed request, layout, and RSS before PM/AER shutdown.
	 * Recovery must not use an in-progress candidate or later partial request.
	 */
	rtlmq_capture_requested_runtime(tp, &tp->platform_recovery_committed);
	tp->platform_recovery_committed_valid = true;
	return 0;
}

int rtlmq_restore_platform_recovery_committed_locked(struct rtlmq_private *tp)
{
	lockdep_assert_held(&tp->control_lock);

	if (!tp->platform_recovery_committed_valid)
		return -EINVAL;

	rtlmq_restore_requested_runtime(tp, &tp->platform_recovery_committed);
	return 0;
}

void rtlmq_clear_platform_recovery_committed_locked(struct rtlmq_private *tp)
{
	lockdep_assert_held(&tp->control_lock);

	memset(&tp->platform_recovery_committed, 0,
	       sizeof(tp->platform_recovery_committed));
	tp->platform_recovery_committed_valid = false;
}

int rtlmq_terminalize_frozen_pci_failure_locked(struct rtlmq_private *tp)
{
	int ret;

	lockdep_assert_held(&tp->control_lock);
	ASSERT_RTNL();

	if (tp->platform_quiesce_owner !=
	    RTLMQ_PLATFORM_QUIESCE_PCI_ERROR)
		return -EINVAL;
	if (tp->state != RTLMQ_STATE_OPEN &&
	    tp->state != RTLMQ_STATE_PROBED &&
	    tp->state != RTLMQ_STATE_REMOVING)
		return tp->state == RTLMQ_STATE_DEAD ? 0 : -EBUSY;

	if (tp->state != RTLMQ_STATE_REMOVING) {
		ret = rtlmq_set_state_locked(tp, RTLMQ_STATE_REMOVING);
		if (ret)
			return ret;
	}

	/*
	 * Permanently disable MMIO before freeing host resources. Callbacks may
	 * retain tp until PCI removal, but must no longer access registers.
	 */
	rtlmq_terminalize_mmio_locked(tp);
	ret = rtlmq_quiesce_irqs_frozen(tp);
	if (ret)
		goto out_failure;
	/*
	 * Frozen-channel IRQ retirement is a host-side operation. Clearing BME
	 * alone did not drain posted writes into descriptors or packet storage.
	 */
	if (!(READ_ONCE(tp->quiesce_phase) & RTLMQ_QUIESCE_HARDWARE_STOPPED)) {
		WRITE_ONCE(tp->dma_quiesced, false);
		ret = -EBUSY;
		goto out_failure;
	}
	ret = rtlmq_withdraw_ring_registers(tp);
	if (ret)
		goto out_failure;
	rtlmq_stop_rings(tp);
	rtlmq_clear_unpublished_vector_queue_maps(tp);
	rtlmq_withdraw_active_xdp_locked(tp);
	/*
	 * Frozen-channel cleanup bypasses normal requested-state rollback.
	 * Discard the candidate explicitly; active ring cleanup does not do it.
	 */
	rtlmq_clear_candidate_profile(tp);
	rtlmq_unwind_active_profile(tp);
	rtlmq_unwind_active_rss(tp);
	rtlmq_release_all_xsk_pool_dma_locked(tp);
	rtlmq_clear_platform_recovery_committed_locked(tp);

	ret = rtlmq_finish_removal_locked(tp);
	if (ret)
		return ret;

	return 0;

out_failure:
	netdev_err(tp->ndev,
		   "terminal PCI failure retained resources after quiesce error: %d\n",
		   ret);
	return ret;
}

static int rtlmq_restore_requested_down_state(
	struct rtlmq_private *tp,
	const struct rtlmq_requested_runtime_snapshot *rollback_requested,
	const char *where)
{
	int primary_ret = 0;
	int requested_ret;
	int counts_ret;
	int state_ret;

	lockdep_assert_held(&tp->control_lock);

	if (rollback_requested)
		rtlmq_restore_requested_runtime(tp, rollback_requested);

	/*
	 * Enter PROBED before restoring requested configuration through the
	 * down-state transaction helper.
	 */
	state_ret = rtlmq_set_state_locked(tp, RTLMQ_STATE_PROBED);
	if (state_ret) {
		primary_ret = state_ret;
		netdev_err(tp->ndev,
			   "%s failed to enter probed state before requested-layout restore: %d\n",
			   where ? where : "lifecycle", state_ret);
	}

	/*
	 * Undo publish_open_runtime(): reset real queue counts before rebuilding
	 * the requested down-state configuration.
	 */
	counts_ret = rtlmq_withdraw_netdev_queue_counts(tp);
	if (counts_ret) {
		if (!primary_ret)
			primary_ret = counts_ret;
		netdev_err(tp->ndev,
			   "%s failed to withdraw active real queue counts: %d\n",
			   where ? where : "lifecycle", counts_ret);
	}

	rtlmq_clear_candidate_rss(tp);
	rtlmq_unwind_active_profile(tp);
	rtlmq_unwind_active_rss(tp);

	requested_ret = rtlmq_apply_requested_runtime_transaction(tp, rollback_requested, true);
	if (requested_ret) {
		if (!primary_ret)
			primary_ret = requested_ret;
		netdev_err(tp->ndev,
			   "%s failed to restore requested down-state state: %d\n",
			   where ? where : "lifecycle", requested_ret);
	}
	rtlmq_sync_xdp_features_locked(tp);
	rtlmq_advance_reset_epoch_locked(tp);

	if (primary_ret)
		netdev_err(tp->ndev,
			   "%s down-state restore failed primary=%d state=%d counts=%d requested=%d\n",
			   where ? where : "lifecycle", primary_ret, state_ret,
			   counts_ret, requested_ret);

	return primary_ret;
}

static int rtlmq_quiesce_datapath(struct rtlmq_private *tp)
{
	int ring_withdraw_ret = 0;
	bool hardware_accessible;
	int irq_release_ret = 0;
	int irq_quiesce_ret;
	int primary_ret = 0;
	int dma_revoke_ret;
	int mac_reset_ret;

	lockdep_assert_held(&tp->control_lock);

	/*
	 * deactivate_device() already hid the runtime, reset queue counts and
	 * mappings, and stopped TX/carrier. Disable IRQs and NAPI, stop hardware,
	 * clear ring registers, drain/free IRQs and NAPI, then free ring DMA.
	 */
	irq_quiesce_ret = rtlmq_quiesce_irqs(tp);
	if (irq_quiesce_ret)
		primary_ret = irq_quiesce_ret;
	rtlmq_ptp_quiesce_locked(tp);
	hardware_accessible = rtlmq_mmio_ordinary_authorized(tp);
	mac_reset_ret = rtlmq_hw_stop(tp);
	if (mac_reset_ret && mac_reset_ret != -ENODEV && !irq_quiesce_ret &&
	    rtlmq_mmio_ordinary_authorized(tp)) {
		int recovery_ret;

		/*
		 * hw_stop already performs the supported reset sequence. Allow
		 * one further attempt, with software users still stopped; never
		 * retry inaccessible MMIO or restart the departing generation.
		 */
		recovery_ret = rtlmq_hw_stop(tp);
		netdev_warn(tp->ndev,
			    "normal shutdown recovery initial=%d recovery=%d\n",
			    mac_reset_ret, recovery_ret);
		if (!recovery_ret)
			mac_reset_ret = 0;
	}
	if (mac_reset_ret && hardware_accessible) {
		int state_ret;

		/*
		 * REMOVING blocks service and new control requests while still
		 * owning resources. DEAD would incorrectly imply cleanup.
		 * Temporary PM/AER access revocation is owned by its caller and
		 * is not evidence that an attempted hardware reset failed.
		 */
		state_ret = rtlmq_begin_removal_locked(tp);
		rtlmq_terminalize_mmio_locked(tp);
		rtlmq_sync_xdp_features_locked(tp);
		netdev_crit(tp->ndev,
			    "normal hardware shutdown failed: %d state=%d; device unavailable, DMA safety not established\n",
			    mac_reset_ret, state_ret);
	}
	if (!primary_ret && mac_reset_ret)
		primary_ret = mac_reset_ret;
	/*
	 * Disable bus mastering regardless of reset success, then check pending
	 * non-posted PCIe requests. This does not replace a successful MAC stop.
	 */
	dma_revoke_ret = rtlmq_pci_disable_bus_master(tp, true);
	if (!primary_ret && dma_revoke_ret)
		primary_ret = dma_revoke_ret;
	/*
	 * A failed MAC stop may leave posted writes in flight even when PCI
	 * revocation succeeds. Retain DMA storage and its active ownership;
	 * terminal failure still retires host callbacks below.
	 */
	if (mac_reset_ret || dma_revoke_ret || irq_quiesce_ret)
		goto out_failure;

	ring_withdraw_ret = rtlmq_withdraw_ring_registers(tp);
	if (!primary_ret && ring_withdraw_ret)
		primary_ret = ring_withdraw_ret;
	if (ring_withdraw_ret)
		goto out_failure;
	/*
	 * Keep descriptors and queue allocations alive until free_irq() has
	 * waited for handlers using the old vector mappings to return.
	 */
	irq_release_ret = rtlmq_release_irqs(tp);
	if (!primary_ret && irq_release_ret)
		primary_ret = irq_release_ret;
	if (irq_release_ret)
		goto out_failure;
	rtlmq_mark_retired_generation_quiesced(tp);
	rtlmq_stop_rings(tp);
	rtlmq_clear_unpublished_vector_queue_maps(tp);

	if (!primary_ret)
		return 0;

out_failure:
	if (tp->state == RTLMQ_STATE_REMOVING &&
	    tp->mmio_authority == RTLMQ_MMIO_AUTHORITY_TERMINAL)
		irq_release_ret = rtlmq_quiesce_irqs_frozen(tp);
	netdev_err(tp->ndev,
		   "quiesce failed primary=%d irq=%d hw=%d dma=%d ring=%d irq_release=%d dma_quiesced=%u\n",
		   primary_ret, irq_quiesce_ret, mac_reset_ret,
		   dma_revoke_ret, ring_withdraw_ret, irq_release_ret,
		   READ_ONCE(tp->dma_quiesced));
	return primary_ret;
}

static int rtlmq_prepare_candidate_host_resources(struct rtlmq_private *tp)
{
	int ret;

	lockdep_assert_held(&tp->control_lock);
	ASSERT_RTNL();

	/*
	 * Leave the committed request and layout unchanged during open.
	 * Partial allocations stay in tp->transaction for lifecycle unwind.
	 */

	/*
	 * Finish IRQ fallback before allocating rings or programming queues.
	 * All later setup uses the resulting candidate configuration.
	 */
	{
		ret = rtlmq_prepare_candidate_irq_vectors(tp);
	}
	if (ret) {
		netdev_err(tp->ndev,
			   "open failed at candidate IRQ preparation: %d\n", ret);
		return ret;
	}

	ret = rtlmq_init_rings(tp);
	if (ret) {
		netdev_err(tp->ndev, "open failed at init_rings: %d\n", ret);
		return ret;
	}

	return 0;
}

static int rtlmq_runtime_build_hardware(struct rtlmq_private *tp)
{
	int ret;

	rtlmq_tc_invalidate_active_locked(tp);

	/*
	 * Keep PHY/firmware/config ahead of the final MAC common-start path.
	 * Host ring/NAPI construction is complete, but the 8125 MAC bootstrap
	 * waits until PHY initialization has settled for this epoch.
	 */
	ret = rtlmq_hw_init(tp);
	if (ret) {
		netdev_err(tp->ndev, "open failed at hw_init: %d\n", ret);
		return ret;
	}

	{
		ret = rtlmq_open_phy(tp);
	}
	if (ret) {
		netdev_err(tp->ndev, "open failed at open_phy: %d\n", ret);
		return ret;
	}

	ret = rtlmq_hw_start(tp);
	if (ret) {
		netdev_err(tp->ndev, "open failed at hw_start: %d\n", ret);
		return ret;
	}

	ret = rtlmq_tc_restore_locked(tp);
	if (ret) {
		netdev_err(tp->ndev,
			   "open failed at traffic-control restore: %d\n", ret);
		return ret;
	}

	ret = rtlmq_ptp_restore_locked(tp);
	if (ret) {
		netdev_err(tp->ndev, "open failed at ptp_restore: %d\n", ret);
		return ret;
	}

	return 0;
}

static int rtlmq_runtime_build(struct rtlmq_private *tp)
{
	const struct rtlmq_runtime_config *cfg;
	int ret;

	lockdep_assert_held(&tp->control_lock);
	ASSERT_RTNL();

	ret = rtlmq_prepare_requested_runtime_profile(tp);
	if (ret) {
		netdev_err(tp->ndev,
			   "open failed at prepare_requested_runtime_profile: %d\n",
			   ret);
		return ret;
	}
	cfg = rtlmq_candidate_runtime_config(tp);
	if (!cfg)
		return -EINVAL;
	ret = rtlmq_prepare_candidate_host_resources(tp);
	if (ret)
		return ret;

	ret = rtlmq_runtime_build_hardware(tp);
	if (ret)
		return ret;

	ret = rtlmq_program_candidate_runtime_state(tp, "runtime_build");
	if (ret) {
		netdev_err(tp->ndev,
			   "open failed at program_candidate_runtime_state: %d\n",
			   ret);
		return ret;
	}

	{
		ret = rtlmq_open_irqs(tp);
		if (ret) {
			netdev_err(tp->ndev,
				   "open failed at candidate IRQ activation: %d\n",
				   ret);
			return ret;
		}
	}

	return 0;
}

static int rtlmq_runtime_activate(struct rtlmq_private *tp)
{
	int ret;

	lockdep_assert_held(&tp->control_lock);
	ASSERT_RTNL();
	rtlmq_clear_quiesce_phase(tp);

	rtlmq_leds_restore(tp);
	ret = rtlmq_update_rx_mode(tp);
	if (ret)
		return ret;
	netif_carrier_off(tp->ndev);
	rtlmq_sync_tx_subqueues(tp);

	ret = rtlmq_validate_open_runtime_ownership(tp);
	if (ret) {
		netdev_err(tp->ndev,
			   "open failed at validate_open_runtime_ownership: %d\n",
			   ret);
		return ret;
	}

	return 0;
}

static int rtlmq_validate_published_tx_setup(struct rtlmq_private *tp)
{
	const struct rtlmq_runtime_layout_state *layout;
	const struct rtlmq_runtime_config *cfg;
	unsigned int i;

	cfg = rtlmq_active_runtime_config(tp);
	layout = rtlmq_active_runtime_layout(tp);
	if (!tp || !cfg || !layout || !layout->valid ||
	    !tp->tx_rings || !tp->vectors)
		return -EINVAL;
	if (cfg->num_tx_queues > rtlmq_owned_tx_ring_slots(tp) ||
	    cfg->num_vectors > tp->allocated_vectors)
		return -EINVAL;

	for (i = 0; i < cfg->num_tx_queues; i++) {
		struct rtlmq_tx_ring *ring = &tp->tx_rings[i];
		struct rtlmq_vector *vector;
		bool mapped = false;
		u16 vector_index;
		unsigned int j;

		vector_index = READ_ONCE(ring->vector_index);
		if (vector_index >= cfg->num_vectors)
			goto invalid;
		if (vector_index != layout->tx_to_vector[i] ||
		    ring->queue_index != i || ring->ndev != tp->ndev ||
		    ring->ndev_queue != netdev_get_tx_queue(tp->ndev, i) ||
		    !ring->desc ||
		    !ring->infos || !ring->completions ||
		    !ring->ndev_queue || !ring->doorbell ||
		    !ring->doorbell_value ||
		    READ_ONCE(tp->txq2ring[i]) != ring)
			goto invalid;

		vector = &tp->vectors[vector_index];
		if (!vector->napi_added || !vector->irq_requested ||
		    vector->irq < 0)
			goto invalid;
		if (vector->num_tx_rings > ARRAY_SIZE(vector->tx_ring_indices))
			goto invalid;
		for (j = 0; j < vector->num_tx_rings; j++) {
			if (vector->tx_ring_indices[j] != i)
				continue;
			mapped = true;
			break;
		}
		if (!mapped)
			goto invalid;
	}

	return 0;

invalid:
	netdev_err(tp->ndev,
		   "published TX setup invalid q%u desc=%p desc_dma=%#llx infos=%p completions=%p txq=%p doorbell=%p doorbell_value=%#x vector=%u txq2ring=%p\n",
		   i,
		   i < rtlmq_owned_tx_ring_slots(tp) ?
		   tp->tx_rings[i].desc : NULL,
		   i < rtlmq_owned_tx_ring_slots(tp) ?
		   (unsigned long long)tp->tx_rings[i].desc_dma : 0,
		   i < rtlmq_owned_tx_ring_slots(tp) ?
		   tp->tx_rings[i].infos : NULL,
		   i < rtlmq_owned_tx_ring_slots(tp) ?
		   tp->tx_rings[i].completions : NULL,
		   i < rtlmq_owned_tx_ring_slots(tp) ?
		   tp->tx_rings[i].ndev_queue : NULL,
		   i < rtlmq_owned_tx_ring_slots(tp) ?
		   tp->tx_rings[i].doorbell : NULL,
		   i < rtlmq_owned_tx_ring_slots(tp) ?
		   tp->tx_rings[i].doorbell_value : 0,
		   i < rtlmq_owned_tx_ring_slots(tp) ?
		   tp->tx_rings[i].vector_index : 0xffff,
		   i < rtlmq_owned_tx_ring_slots(tp) ?
		   tp->txq2ring[i] : NULL);
	return -EINVAL;
}

static int rtlmq_publish_open_queue_generation(struct rtlmq_private *tp,
					       const struct rtlmq_runtime_config *cfg,
					       const struct rtlmq_runtime_layout_state *layout)
{
	int ret;

	lockdep_assert_held(&tp->control_lock);
	ASSERT_RTNL();

	ret = rtlmq_publish_runtime_queue_ownership(tp, cfg, layout);
	if (ret)
		return ret;

	ret = rtlmq_validate_open_scaling_state(tp, cfg, layout);
	if (ret)
		goto err_withdraw_ownership;

	rtlmq_publish_active_xsk_bindings_locked(tp, cfg, layout);

	/*
	 * Update real queue counts immediately after queue/NAPI/ring mappings.
	 * Do not interleave affinity or other lifecycle changes.
	 */
	ret = rtlmq_publish_netdev_queue_counts(tp);
	if (ret)
		goto err_withdraw_counts;

	ret = rtlmq_validate_published_real_queue_counts(tp, cfg);
	if (ret)
		goto err_withdraw_counts;

	return 0;

err_withdraw_counts:
	rtlmq_withdraw_netdev_queue_counts(tp);
	rtlmq_withdraw_active_xsk_bindings_locked(tp);
err_withdraw_ownership:
	rtlmq_withdraw_runtime_queue_ownership(tp);
	return ret;
}

static void rtlmq_withdraw_open_queue_generation(struct rtlmq_private *tp)
{
	lockdep_assert_held(&tp->control_lock);
	ASSERT_RTNL();

	rtlmq_withdraw_netdev_queue_counts(tp);
	rtlmq_withdraw_active_xsk_bindings_locked(tp);
	rtlmq_withdraw_runtime_queue_ownership(tp);
}

static void
rtlmq_withdraw_open_runtime_visibility_locked(struct rtlmq_private *tp)
{
	lockdep_assert_held(&tp->control_lock);
	ASSERT_RTNL();

	/*
	 * Clear runtime_published before removing queue mappings, while their
	 * resources still exist. Update xdp_features after resetting queue
	 * counts so it cannot advertise TX resources being torn down.
	 */
	rtlmq_revoke_runtime_publication_locked(tp);
	rtlmq_withdraw_open_queue_generation(tp);
	rtlmq_sync_xdp_features_locked(tp);
	rtlmq_mark_quiesce_phase(tp, RTLMQ_QUIESCE_PUBLICATION_REVOKED);
}

void rtlmq_revoke_platform_datapath_locked(struct rtlmq_private *tp)
{
	lockdep_assert_held(&tp->control_lock);
	ASSERT_RTNL();

	/*
	 * Workers may take RTNL or instance/control locks. Stop new datapath
	 * work before the caller drops those locks to cancel workers, while
	 * retaining resources for callbacks already in progress.
	 */
	netif_tx_disable(tp->ndev);
	rtlmq_flush_pending_tx_doorbells(tp);
	rtlmq_withdraw_open_runtime_visibility_locked(tp);
	synchronize_rcu();
	rtlmq_advance_reset_epoch_locked(tp);
	rtlmq_mark_quiesce_phase(tp,
				 RTLMQ_QUIESCE_ASYNC_REACQUIRE_REVOKED | RTLMQ_QUIESCE_RCU_DRAINED);
	netif_device_detach(tp->ndev);
	netif_carrier_off(tp->ndev);
	rtlmq_sync_tx_subqueues(tp);
}

int rtlmq_complete_live_change_locked(struct rtlmq_private *tp,
				      enum rtlmq_live_change_outcome outcome,
				      int error, const char *where)
{
	int irq_ret;

	if (outcome == RTLMQ_LIVE_CHANGE_NOT_COMMITTED)
		return error;
	if (outcome == RTLMQ_LIVE_CHANGE_COMMITTED)
		return 0;

	lockdep_assert_held(&tp->control_lock);
	ASSERT_RTNL();

	/*
	 * Retain committed resources, but withdraw partially resumed service.
	 * This retires host admission only; it establishes no DMA release proof.
	 */
	rtlmq_serial_tx_pause(tp);
	rtlmq_revoke_platform_datapath_locked(tp);
	irq_ret = rtlmq_quiesce_irqs(tp);
	netdev_err(tp->ndev,
		   "%s committed but service recovery is pending: %d irq_quiesce=%d\n",
		   where, error, irq_ret);
	rtlmq_request_reset(tp, BIT(RTLMQ_RESET_REASON_RUNTIME_RECONFIG)
	);

	/* An ndo_bpf error would let the core revoke the committed pool. */
	return 0;
}

static int rtlmq_publish_open_runtime(struct rtlmq_private *tp)
{
	const struct rtlmq_runtime_layout_state *layout;
	bool queue_generation_published = false;
	const struct rtlmq_runtime_config *cfg;
	int ret;

	lockdep_assert_held(&tp->control_lock);
	ASSERT_RTNL();

	cfg = rtlmq_candidate_runtime_config(tp);
	if (!cfg)
		return -EINVAL;

	ret = rtlmq_validate_open_runtime_ownership(tp);
	if (ret)
		return ret;

	ret = rtlmq_publish_runtime_profile_active(tp);
	if (ret)
		goto err_withdraw_runtime_publication;

	cfg = rtlmq_active_runtime_config(tp);
	layout = rtlmq_active_runtime_layout(tp);

	/*
	 * Keep bootstrap 1x1 queue counts until runtime and XDP state commit,
	 * then install queue mappings and real counts together.
	 */
	ret = rtlmq_publish_open_queue_generation(tp, cfg, layout);
	if (ret)
		goto err_withdraw_runtime_publication;
	queue_generation_published = true;

	ret = rtlmq_validate_published_tx_setup(tp);
	if (ret)
		goto err_withdraw_runtime_publication;

	/*
	 * Keep bus mastering disabled through reset, PHY/MAC setup, and ring
	 * programming so reload cannot resume stale DMA. Enable it only after
	 * validating active queue mappings and TX doorbells.
	 */
	ret = rtlmq_pci_enable_bus_master(tp);
	if (ret) {
		netdev_err(tp->ndev,
			   "open failed at pci_enable_bus_master: %d\n",
			   ret);
		goto err_withdraw_runtime_publication;
	}

	ret = rtlmq_hw_enable_datapath(tp);
	if (ret) {
		netdev_err(tp->ndev,
			   "open failed at hw_enable_datapath: %d\n",
			   ret);
		goto err_withdraw_runtime_publication;
	}

	/* Counter dumps need RX enabled; rebase before IRQ/NAPI publication. */
	rtlmq_init_counter_offsets(tp);

	ret = rtlmq_program_active_irq_moderation(tp);
	if (ret)
		goto err_withdraw_runtime_publication;
	rtlmq_enable_active_napi(tp, cfg, layout);

	ret = rtlmq_set_state_locked(tp, RTLMQ_STATE_OPEN);
	if (ret)
		goto err_withdraw_runtime_publication;
	rtlmq_publish_runtime_publication_locked(tp);

	rtlmq_advance_reset_epoch_locked(tp);
	/* Autonomous recovery must belong to the newly initialized reset epoch. */
	if (!tp->tc.link_waiting)
		rtlmq_serial_tx_resume(tp);
	rtlmq_sync_netdev_tso_limits(tp);
	rtlmq_tc_publish_runtime_locked(tp);
	rtlmq_sync_xdp_features_locked(tp);
	rtlmq_xsk_start_tx_locked(tp);
	rtlmq_mask_and_ack_intrs(tp);
	rtlmq_enable_intrs(tp);
	rtlmq_apply_data_irq_affinity(tp);
	/* PCI recovery must release its access gate before PHY work can run. */
	if (tp->platform_quiesce_owner != RTLMQ_PLATFORM_QUIESCE_PCI_ERROR) {
		rtlmq_start_phy(tp);
		rtlmq_sync_link_state(tp);
	}

	return 0;

err_withdraw_runtime_publication:
	if (queue_generation_published)
		rtlmq_withdraw_open_queue_generation(tp);
	else
		/*
		 * The caller may have installed queue/NAPI mappings before entry.
		 * Remove them even if validation failed before updating real
		 * queue counts here.
		 */
		rtlmq_withdraw_runtime_queue_ownership(tp);
	rtlmq_withdraw_active_xdp_locked(tp);
	rtlmq_unwind_active_profile(tp);
	rtlmq_unwind_active_rss(tp);
	rtlmq_sync_xdp_features_locked(tp);
	return ret;
}

static int rtlmq_teardown_to_requested_locked(
	struct rtlmq_private *tp,
	const struct rtlmq_requested_runtime_snapshot *rollback_requested,
	const char *where)
{
	int candidate_ret;
	int destroy_ret;
	int restore_ret;

	lockdep_assert_held(&tp->control_lock);

	/*
	 * A failed open may already have rings, IRQs, NAPI, and XDP/XSK bindings.
	 * Tear them down before restoring requested down-state configuration.
	 */
	destroy_ret = rtlmq_deactivate_device(tp);
	if (!READ_ONCE(tp->dma_quiesced) ||
	    !rtlmq_quiesce_contract_satisfied(tp) ||
	    tp->retained_live_tx || tp->retained_live_rx) {
		rtlmq_restore_requested_settings(tp, rollback_requested);
		if (!destroy_ret)
			destroy_ret = -EBUSY;
		netdev_err(tp->ndev,
			   "%s retaining teardown ownership after incomplete resource retirement: %d\n",
			   where ? where : "lifecycle", destroy_ret);
		return destroy_ret;
	}

	candidate_ret = rtlmq_abort_candidate_runtime_locked(tp, where);
	if (candidate_ret) {
		netdev_err(tp->ndev,
			   "%s refusing requested-state restore with candidate resources still owned: %d\n",
			   where ? where : "lifecycle", candidate_ret);
		return destroy_ret ? destroy_ret : candidate_ret;
	}

	/*
	 * Final removal owns the remaining requests until core detaches them.
	 * Do not allocate a down-state candidate or reopen control admission.
	 */
	if (tp->state == RTLMQ_STATE_REMOVING)
		return destroy_ret;

	restore_ret = rtlmq_restore_requested_down_state(tp, rollback_requested, where);
	WARN_ON_ONCE(rtlmq_published_runtime_config(tp));
	WARN_ON_ONCE(tp->active_resource_generation);
	WARN_ON_ONCE(tp->transaction.resource_generation);

	if (destroy_ret && restore_ret)
		netdev_err(tp->ndev,
			   "%s cleanup failure %d did not replace teardown root cause %d\n",
			   where ? where : "lifecycle", restore_ret, destroy_ret);

	return destroy_ret ? destroy_ret : restore_ret;
}

static void rtlmq_publish_detached_rebuild_failure_locked(struct rtlmq_private *tp,
							  const char *where, int primary_ret,
							  int rollback_ret, bool reopen_attempted,
							  int reopen_ret)
{
	lockdep_assert_held(&tp->control_lock);

	if (!READ_ONCE(tp->dma_quiesced) ||
	    !rtlmq_quiesce_contract_satisfied(tp)) {
		netif_carrier_off(tp->ndev);
		netif_device_detach(tp->ndev);
		netdev_err(tp->ndev,
			   "%s retained unpublished runtime after incomplete quiesce primary=%d rollback=%d dma_quiesced=%u state=%s\n",
			   where ? where : "rebuild_to_open",
			   primary_ret, rollback_ret,
			   READ_ONCE(tp->dma_quiesced),
			 rtlmq_state_name(READ_ONCE(tp->state)));
		return;
	}

	/*
	 * The old datapath is already stopped. Failure must leave PROBED,
	 * bootstrap queue counts, no active runtime, and a detached netdev.
	 * Detach explicitly rather than rely on an earlier rollback step.
	 */
	WARN_ON_ONCE(READ_ONCE(tp->state) != RTLMQ_STATE_PROBED);
	WARN_ON_ONCE(rtlmq_published_runtime_config(tp));
	WARN_ON_ONCE(tp->ndev->real_num_rx_queues != RTLMQ_BOOT_RX_QUEUES);
	WARN_ON_ONCE(tp->ndev->real_num_tx_queues != RTLMQ_BOOT_TX_QUEUES);
	netif_carrier_off(tp->ndev);
	netif_device_detach(tp->ndev);
	netdev_err(tp->ndev,
		   "%s left device detached after destructive rebuild failure primary=%d rollback=%d reopen_attempted=%u reopen=%d state=%s\n",
		   where ? where : "rebuild_to_open",
		   primary_ret, rollback_ret, reopen_attempted, reopen_ret,
		   rtlmq_state_name(READ_ONCE(tp->state)));
}

static int rtlmq_rebuild_to_open(struct rtlmq_private *tp, const char *where,
				 const struct rtlmq_requested_runtime_snapshot *rollback_requested,
				 bool reopen_rollback, bool detach_after_failure)
{
	int rollback_ret = 0;
	int reopen_ret = 0;
	bool reopen_attempted = false;
	int ret;

	lockdep_assert_held(&tp->control_lock);

	rtlmq_unwind_active_profile(tp);
	rtlmq_unwind_active_rss(tp);
	ret = rtlmq_abort_candidate_runtime_locked(tp, where);
	if (ret)
		return ret;

	ret = rtlmq_runtime_build(tp);
	if (ret)
		goto err_rollback;

	ret = rtlmq_runtime_activate(tp);
	if (ret)
		goto err_rollback;

	ret = rtlmq_publish_open_runtime(tp);
	if (!ret)
		return 0;

err_rollback:
	rollback_ret = rtlmq_teardown_to_requested_locked(tp, rollback_requested, where);
	if (reopen_rollback && rollback_requested && !rollback_ret &&
	    READ_ONCE(tp->state) == RTLMQ_STATE_PROBED) {
		reopen_attempted = true;
		{ reopen_ret = rtlmq_open_device(tp); }
		if (reopen_ret)
			netdev_err(tp->ndev,
				   "%s rollback reopen failed: %d\n",
				   where ? where : "rebuild_to_open",
				   reopen_ret);
	}
	if (detach_after_failure &&
	    READ_ONCE(tp->state) != RTLMQ_STATE_OPEN)
		rtlmq_publish_detached_rebuild_failure_locked(tp, where, ret, rollback_ret,
							      reopen_attempted, reopen_ret);

	return ret;
}

static int rtlmq_release_runtime_owners_locked(struct rtlmq_private *tp)
{
	lockdep_assert_held(&tp->control_lock);

	rtlmq_tc_invalidate_active_locked(tp);
	rtlmq_withdraw_active_xdp_locked(tp);
	rtlmq_unwind_active_profile(tp);
	rtlmq_unwind_active_rss(tp);
	if (rtlmq_release_live_tx_ring_set(tp, tp->retained_live_tx))
		tp->retained_live_tx = NULL;
	if (rtlmq_release_live_rx_ring_set(tp, tp->retained_live_rx))
		tp->retained_live_rx = NULL;

	return tp->retained_live_tx || tp->retained_live_rx ? -EBUSY : 0;
}

static int rtlmq_runtime_destroy(struct rtlmq_private *tp)
{
	int release_ret;
	int ret;

	lockdep_assert_held(&tp->control_lock);

	rtlmq_close_phy(tp);
	ret = rtlmq_quiesce_datapath(tp);
	if (!rtlmq_quiesce_contract_satisfied(tp))
		return ret ? ret : -EBUSY;
	release_ret = rtlmq_release_runtime_owners_locked(tp);
	return ret ? ret : release_ret;
}

static int rtlmq_deactivate_device(struct rtlmq_private *tp)
{
	int ret;

	lockdep_assert_held(&tp->control_lock);

	netif_tx_disable(tp->ndev);
	rtlmq_serial_tx_pause(tp);
	rtlmq_flush_pending_tx_doorbells(tp);
	rtlmq_withdraw_open_runtime_visibility_locked(tp);
	synchronize_rcu();
	/*
	 * Keep control_lock through teardown so reset/link workers cannot use
	 * the old generation while its callbacks and resources are drained.
	 */
	rtlmq_mark_quiesce_phase(tp,
				 RTLMQ_QUIESCE_ASYNC_REACQUIRE_REVOKED | RTLMQ_QUIESCE_RCU_DRAINED);
	netif_carrier_off(tp->ndev);
	rtlmq_sync_tx_subqueues(tp);
	ret = rtlmq_runtime_destroy(tp);

	return ret;
}

int rtlmq_finalize_failed_runtime_locked(struct rtlmq_private *tp)
{
	int ret;

	lockdep_assert_held(&tp->control_lock);
	ASSERT_RTNL();

	if (!rtlmq_final_release_allowed(tp)) {
		if (tp->state != RTLMQ_STATE_DEAD) {
			ret = rtlmq_begin_removal_locked(tp);
			if (ret)
				return ret;
		}
		rtlmq_terminalize_mmio_locked(tp);
		/*
		 * No reset retry through revoked MMIO. Reuse publication, PHY,
		 * PTP and host observer retirement; retain the shutdown error.
		 */
		ret = rtlmq_deactivate_device(tp);
		if (!rtlmq_host_retirement_predecessors_satisfied(tp) ||
		    !rtlmq_vector_storage_release_allowed(tp))
			return ret ? ret : -EBUSY;
		rtlmq_mark_quiesce_phase(tp, RTLMQ_QUIESCE_FINAL_RELEASE);
	}

	if (!rtlmq_release_allowed(tp)) {
		netdev_crit(tp->ndev,
			    "terminal host cleanup complete; retaining DMA ownership without shutdown proof\n");
		return -EBUSY;
	}

	/* TX completion retirement precedes RX pool unmap, as in normal close. */
	rtlmq_stop_rings(tp);
	ret = rtlmq_release_runtime_owners_locked(tp);
	if (ret)
		return ret;
	ret = rtlmq_abort_candidate_runtime_locked(tp, "final_core_teardown");
	if (ret)
		return ret;
	rtlmq_release_all_xsk_pool_dma_locked(tp);
	rtlmq_clear_platform_recovery_committed_locked(tp);
	rtlmq_sync_xdp_features_locked(tp);
	return 0;
}

static int
rtlmq_preflight_rebuild_locked(struct rtlmq_private *tp,
			       const struct rtlmq_requested_runtime_snapshot *rollback_requested,
			       const char *where)
{
	int ret;

	lockdep_assert_held(&tp->control_lock);

	/*
	 * Validate hardware-independent candidate settings while the old OPEN
	 * datapath still runs. Discard this preflight candidate; ring/IRQ/NAPI
	 * replacement builds it again after stopping the old hardware runtime.
	 */
	ret = rtlmq_prepare_requested_runtime_profile(tp);
	if (ret && rollback_requested)
		rtlmq_restore_requested_runtime(tp, rollback_requested);

	rtlmq_clear_candidate_rss(tp);
	rtlmq_clear_candidate_profile(tp);
	if (ret)
		netdev_err(tp->ndev,
			   "%s rejected candidate before live datapath teardown: %d\n",
			   where ? where : "rebuild_preflight", ret);

	return ret;
}

static int rtlmq_reset_device(struct rtlmq_private *tp, u32 reset_reasons,
			      const struct rtlmq_requested_runtime_snapshot *rollback_requested,
			      bool reopen_rollback)
{
	char reason_buf[96];
	int ret;

	lockdep_assert_held(&tp->control_lock);
	if (!rtlmq_requested_snapshot_committed(rollback_requested))
		return -EINVAL;
	rtlmq_format_reset_reasons(reason_buf, sizeof(reason_buf),
				   reset_reasons);

	ret = rtlmq_preflight_rebuild_locked(tp, rollback_requested,
					     "reset_device");
	if (ret)
		return ret;

	ret = rtlmq_set_state_locked(tp, RTLMQ_STATE_RESETTING);
	if (ret)
		return ret;

	netif_device_detach(tp->ndev);
	ret = rtlmq_deactivate_device(tp);
	if (ret) {
		int unwind_ret;

		unwind_ret = rtlmq_teardown_to_requested_locked(tp, rollback_requested,
								"reset_deactivate_failed");
		netdev_err(tp->ndev,
			   "reset_device deactivate failed reasons=%s ret=%d unwind=%d\n",
			   reason_buf, ret, unwind_ret);
		rtlmq_publish_detached_rebuild_failure_locked(tp, "reset_device_deactivate_failed",
							      ret, unwind_ret, false, 0);
		return ret;
	}

	ret = rtlmq_rebuild_to_open(tp, "reset_device", rollback_requested,
				    reopen_rollback, true);
	if (!ret)
		return 0;

	netdev_err(tp->ndev, "reset_device failed reasons=%s ret=%d\n",
		   reason_buf, ret);
	return ret;
}

int rtlmq_apply_live_reconfiguration_locked(
	struct rtlmq_private *tp,
	u32 reset_reason,
	const struct rtlmq_requested_runtime_snapshot *rollback_requested
)
{
	if (!tp || !tp->ndev || !rollback_requested || !reset_reason)
		return -EINVAL;
	lockdep_assert_held(&tp->control_lock);
	if (!netif_running(tp->ndev) ||
	    READ_ONCE(tp->state) != RTLMQ_STATE_OPEN)
		return -EBUSY;

	return rtlmq_apply_recovery_transaction_locked(tp, reset_reason, rollback_requested, true);
}

static int rtlmq_restore_active_ring_runtime_locked(struct rtlmq_private *tp)
{
	int ret;

	ret = rtlmq_republish_active_tx_ring_bases_locked(tp);
	if (ret)
		return ret;

	ret = rtlmq_restart_active_rx_rings_from_zero_locked(tp);
	if (ret)
		return ret;

	return rtlmq_refresh_live_ring_bindings_unpublished_locked(tp);
}

static int rtlmq_republish_restored_live_ring_service_locked(
	struct rtlmq_private *tp, struct rtlmq_live_service_pause *pause,
	u32 resource_generation,
	struct rtlmq_live_ring_profile_commit *profile_commit)
{
	int ret;

	ret = rtlmq_resume_live_mac_locked(tp, pause);
	if (ret)
		return ret;

	if (profile_commit && profile_commit->staged)
		ret = rtlmq_publish_live_ring_profile_locked(tp, profile_commit, false);
	else
		ret = rtlmq_publish_live_ring_generation_locked(tp, resource_generation);
	if (ret)
		return ret;

	return rtlmq_restore_live_host_service_locked(tp, pause);
}

struct rtlmq_live_ring_change {
	struct rtlmq_live_ring_profile_commit profile_commit;
	struct rtlmq_live_service_pause pause;
	struct rtlmq_live_rx_ring_set *retired_rx;
	struct rtlmq_live_tx_ring_set *retired_tx;
	u32 old_generation;
	bool tx_swapped;
	bool rx_swapped;
};

static int rtlmq_close_failed_live_ring_change_locked(
	struct rtlmq_private *tp, struct rtlmq_live_ring_change *change,
	const struct rtlmq_requested_runtime_snapshot *rollback_requested,
	int ret, int restore_ret)
{
	lockdep_assert_held(&tp->control_lock);
	ASSERT_RTNL();

	/*
	 * Fencing or restoration failed. Transfer local storage to the full
	 * lifecycle owner before stopping the remaining host and device users.
	 * An ordinary reconfiguration failure cannot authorize final release.
	 */
	tp->retired_generation_quiesced = false;
	tp->retained_live_tx = change->retired_tx;
	tp->retained_live_rx = change->retired_rx;
	change->retired_tx = NULL;
	change->retired_rx = NULL;
	rtlmq_restore_requested_settings(tp, rollback_requested);
	rtlmq_close_device(tp);
	if (tp->state == RTLMQ_STATE_PROBED &&
	    tp->platform_quiesce_owner == RTLMQ_PLATFORM_QUIESCE_NONE)
		rtlmq_request_reset(tp, BIT(RTLMQ_RESET_REASON_RUNTIME_RECONFIG)
		);
	return ret;
}

static int rtlmq_rollback_live_ring_change_locked(
	struct rtlmq_private *tp, struct rtlmq_live_ring_change *change,
	const struct rtlmq_requested_runtime_snapshot *rollback_requested, int ret)
{
	int restore_ret;

	lockdep_assert_held(&tp->control_lock);
	ASSERT_RTNL();

	if (change->profile_commit.staged)
		rtlmq_restore_live_ring_profile_unpublished_locked(tp, &change->profile_commit);
	if (change->tx_swapped)
		rtlmq_rollback_live_tx_rings(tp, change->retired_tx);
	if (change->rx_swapped)
		rtlmq_rollback_live_rx_rings(tp, change->retired_rx);
	if (!change->profile_commit.staged)
		rtlmq_restore_requested_settings(tp, rollback_requested);
	restore_ret = rtlmq_restore_active_ring_runtime_locked(tp);
	if (restore_ret)
		return rtlmq_close_failed_live_ring_change_locked(tp, change, rollback_requested,
								  ret, restore_ret);
	/* A partial swap can leave programmed TX storage in the candidate. */
	rtlmq_clear_candidate_profile(tp);
	if (change->retired_tx) {
		rtlmq_release_live_tx_ring_set(tp, change->retired_tx);
		change->retired_tx = NULL;
	}
	if (change->retired_rx) {
		rtlmq_release_live_rx_ring_set(tp, change->retired_rx);
		change->retired_rx = NULL;
	}
	restore_ret = rtlmq_republish_restored_live_ring_service_locked(tp, &change->pause,
									change->old_generation,
									&change->profile_commit);
	if (restore_ret)
		return rtlmq_close_failed_live_ring_change_locked(tp, change, rollback_requested,
								  ret, restore_ret);
	return ret;
}

static int rtlmq_finish_live_ring_change_locked(struct rtlmq_private *tp,
						struct rtlmq_live_ring_change *change,
						enum rtlmq_live_change_outcome *outcome)
{
	int restore_ret;
	int ret;

	lockdep_assert_held(&tp->control_lock);
	ASSERT_RTNL();
	*outcome = RTLMQ_LIVE_CHANGE_RECOVERY_PENDING;

	/*
	 * MAC enable has switched hardware to the new rings. Free old DMA
	 * storage before setting runtime_published. A later PTP/NAPI resume
	 * failure requires recovery; it cannot restore the old ring sizes.
	 */
	rtlmq_release_live_tx_ring_set(tp, change->retired_tx);
	change->retired_tx = NULL;
	rtlmq_release_live_rx_ring_set(tp, change->retired_rx);
	change->retired_rx = NULL;
	ret = rtlmq_publish_live_ring_profile_locked(tp, &change->profile_commit, true);
	if (ret)
		return ret;
	restore_ret = rtlmq_restore_live_host_service_locked(tp, &change->pause);
	if (restore_ret)
		return restore_ret;
	*outcome = RTLMQ_LIVE_CHANGE_COMMITTED;
	return 0;
}

static int rtlmq_apply_live_ring_reconfiguration_inner_locked(
	struct rtlmq_private *tp,
	const struct rtlmq_requested_runtime_snapshot *rollback_requested,
	const char *where, enum rtlmq_live_change_outcome *outcome)
{
	struct rtlmq_live_ring_change change = {};
	int restore_ret;
	int ret;

	if (!tp || !tp->ndev || !rollback_requested)
		return -EINVAL;
	lockdep_assert_held(&tp->control_lock);
	ASSERT_RTNL();
	if (!netif_running(tp->ndev) ||
	    READ_ONCE(tp->state) != RTLMQ_STATE_OPEN)
		return -EBUSY;
	if (tp->retained_live_rx || tp->retained_live_tx)
		return -EBUSY;

	change.retired_rx = rtlmq_alloc_live_rx_ring_set();
	{ change.retired_tx = rtlmq_alloc_live_tx_ring_set(); }
	if (!change.retired_rx || !change.retired_tx) {
		ret = -ENOMEM;
		goto out_restore_requested;
	}

	ret = rtlmq_build_live_ring_candidate_locked(tp);
	if (ret) {
		netdev_err(tp->ndev,
			   "%s failed at live ring candidate build: %d\n",
			   where ? where : "live ring reconfiguration", ret);
		goto out_restore_requested;
	}

	ret = rtlmq_pause_live_service_locked(tp, &change.pause, RTLMQ_LIVE_MAC_PAUSE_ALL_ENGINES,
					      false);
	if (ret) {
		netdev_err(tp->ndev,
			   "%s failed at live service pause: %d\n",
			   where ? where : "live ring reconfiguration", ret);
		goto out_restore_requested;
	}

	change.old_generation = tp->active_resource_generation;
	rtlmq_revoke_runtime_publication_locked(tp);
	synchronize_rcu();
	tp->active_resource_generation = 0;
	ret = rtlmq_mark_live_retired_generation_quiesced(tp, &change.pause);
	if (ret) {
		netdev_err(tp->ndev,
			   "%s failed at live generation fence: %d\n",
			   where ? where : "live ring reconfiguration", ret);
		return rtlmq_close_failed_live_ring_change_locked(tp, &change, rollback_requested,
								  ret, ret);
	}

	ret = rtlmq_stop_active_tx_rings_for_live_swap(tp, &change.pause);
	if (ret) {
		netdev_err(tp->ndev,
			   "%s failed at live TX retirement: %d\n",
			   where ? where : "live ring reconfiguration", ret);
		goto rollback;
	}

	/*
	 * Program candidate bases with both engines stopped and old rings
	 * retained. Verify TX/RX base readback before swapping ownership. Failure
	 * before the swap restores old rings with cursors reset to zero.
	 */
	ret = rtlmq_program_ring_registers(tp);
	if (ret) {
		netdev_err(tp->ndev,
			   "%s failed at live candidate programming: %d\n",
			   where ? where : "live ring reconfiguration", ret);
		goto rollback;
	}

	ret = rtlmq_swap_live_candidate_rx_rings(tp, change.retired_rx);
	if (ret) {
		netdev_err(tp->ndev,
			   "%s failed at live RX ownership swap: %d\n",
			   where ? where : "live ring reconfiguration", ret);
		goto rollback;
	}
	change.rx_swapped = true;
	ret = rtlmq_swap_live_candidate_tx_rings(tp, change.retired_tx);
	if (ret) {
		netdev_err(tp->ndev,
			   "%s failed at live TX ownership swap: %d\n",
			   where ? where : "live ring reconfiguration", ret);
		goto rollback;
	}
	change.tx_swapped = true;

	ret = rtlmq_stage_live_ring_profile_locked(tp, rollback_requested, change.old_generation,
						   &change.profile_commit);
	if (ret) {
		netdev_err(tp->ndev,
			   "%s failed at live profile staging: %d\n",
			   where ? where : "live ring reconfiguration", ret);
		goto rollback;
	}

	restore_ret = rtlmq_resume_live_mac_locked(tp, &change.pause);
	if (restore_ret) {
		netdev_err(tp->ndev,
			   "%s failed at live MAC resume: %d\n",
			   where ? where : "live ring reconfiguration",
			   restore_ret);
		ret = restore_ret;
		goto rollback;
	}

	/* MAC resume commits the new rings; only recovery remains. */
	return rtlmq_finish_live_ring_change_locked(tp, &change, outcome);

rollback:
	return rtlmq_rollback_live_ring_change_locked(tp, &change, rollback_requested, ret);

out_restore_requested:
	rtlmq_restore_requested_runtime(tp, rollback_requested);
	rtlmq_release_live_tx_ring_set(tp, change.retired_tx);
	rtlmq_release_live_rx_ring_set(tp, change.retired_rx);
	return ret;
}

int rtlmq_apply_live_ring_reconfiguration_locked(
	struct rtlmq_private *tp,
	const struct rtlmq_requested_runtime_snapshot *rollback_requested,
	const char *where)
{
	enum rtlmq_live_change_outcome outcome = RTLMQ_LIVE_CHANGE_NOT_COMMITTED;
	struct rtlmq_netpoll_guard netpoll_guard;
	int ret;

	/*
	 * XSK candidates share the active pool's fill/free lists and RXQ
	 * pointers. Use the full quiesce/rebuild transaction, including request
	 * rollback, when pools are shared or the profile does not qualify
	 * in-place DMA-base replacement.
	 */
	if (rtlmq_active_xsk_queue_count(tp) ||
	    !rtlmq_qualified_supports_inplace_ring_resize(tp))
		return rtlmq_apply_live_reconfiguration_locked(
			tp, BIT(RTLMQ_RESET_REASON_RUNTIME_RECONFIG),
			rollback_requested
		);

	rtlmq_netpoll_guard_acquire_locked(tp, &netpoll_guard);
	ret = rtlmq_apply_live_ring_reconfiguration_inner_locked(tp, rollback_requested, where,
								 &outcome);
	ret = rtlmq_complete_live_change_locked(tp, outcome, ret, where);
	rtlmq_netpoll_guard_release_locked(tp, &netpoll_guard);

	return ret;
}

int rtlmq_open_device(struct rtlmq_private *tp)
{
	int ret;

	lockdep_assert_held(&tp->control_lock);

	ret = rtlmq_sync_features_locked(tp);
	if (ret)
		return ret;
	ret = rtlmq_set_state_locked(tp, RTLMQ_STATE_OPENING);
	if (ret)
		return ret;

	ret = rtlmq_rebuild_to_open(tp, "open_device", NULL, false, false);
	if (!ret)
		return 0;
	netdev_err(tp->ndev, "open_device failed: %d\n", ret);
	return ret;
}

int rtlmq_close_device(struct rtlmq_private *tp)
{
	int ret;

	lockdep_assert_held(&tp->control_lock);

	if (tp->state == RTLMQ_STATE_DEAD)
		return 0;
	/*
	 * XSK final detach can finish close while PM has dropped its locks to
	 * drain workers. The original owner must be able to finish that close
	 * without another hardware reset or an invalid PROBED -> CLOSING step.
	 */
	if ((tp->state == RTLMQ_STATE_PROBED ||
	     tp->state == RTLMQ_STATE_REMOVING) &&
	    (rtlmq_quiesce_contract_satisfied(tp) ||
	     rtlmq_final_release_allowed(tp)) &&
	    !tp->active_resource_generation &&
	    !rtlmq_candidate_runtime_has_resources(tp) &&
	    !rtlmq_derive_active_rx_dma_ring_mask(tp) &&
	    !rtlmq_derive_active_tx_dma_ring_mask(tp) &&
	    !tp->retained_live_rx && !tp->retained_live_tx)
		return 0;

	switch (tp->state) {
	case RTLMQ_STATE_REMOVING:
	case RTLMQ_STATE_CLOSING:
	case RTLMQ_STATE_OPENING:
	case RTLMQ_STATE_RESETTING:
		/* Finish the retained owner's teardown without inventing a phase. */
		break;
	case RTLMQ_STATE_OPEN:
		ret = rtlmq_set_state_locked(tp, RTLMQ_STATE_CLOSING);
		if (ret)
			return ret;
		break;
	default:
		return -EINVAL;
	}

	ret = rtlmq_teardown_to_requested_locked(tp, NULL, "close_device");
	/*
	 * Both reference rtl8169_down() and V1 rtl8126_close() clear the exit-L1
	 * mask on ordinary close. The PM owner separately decides whether to
	 * prepare wake RX or commit a sleep transition.
	 */
	if (!ret && tp->state == RTLMQ_STATE_PROBED &&
	    rtlmq_mmio_ordinary_authorized(tp)) {
		const struct rtlmq_hw_contract *hw =
			rtlmq_chip_profile_hw(tp->profile);

		if (hw && hw->mac_version == RTLMQ_MAC_8126A)
			ret = rtlmq_prepare_d3_8126(tp);
	}
	return ret;
}

int rtlmq_begin_removal_locked(struct rtlmq_private *tp)
{
	int ret;

	lockdep_assert_held(&tp->control_lock);

	/* Terminal failure may have already revoked ownership before PCI remove. */
	if (tp->state == RTLMQ_STATE_REMOVING)
		return 0;

	if (tp->state != RTLMQ_STATE_OPEN &&
	    tp->state != RTLMQ_STATE_PROBED &&
	    tp->state != RTLMQ_STATE_OPENING &&
	    tp->state != RTLMQ_STATE_RESETTING &&
	    tp->state != RTLMQ_STATE_CLOSING)
		return -EBUSY;

	ret = rtlmq_set_state_locked(tp, RTLMQ_STATE_REMOVING);
	if (ret)
		return ret;

	rtlmq_advance_reset_epoch_locked(tp);
	netif_device_detach(tp->ndev);
	return 0;
}

int rtlmq_finish_removal_locked(struct rtlmq_private *tp)
{
	int ret;

	lockdep_assert_held(&tp->control_lock);

	if (tp->state != RTLMQ_STATE_REMOVING &&
	    tp->state != RTLMQ_STATE_PROBED)
		return -EBUSY;

	ret = rtlmq_set_state_locked(tp, RTLMQ_STATE_DEAD);
	if (ret)
		return ret;

	rtlmq_clear_candidate_rss(tp);
	rtlmq_unwind_active_profile(tp);
	rtlmq_unwind_active_rss(tp);
	rtlmq_release_all_xsk_pool_dma_locked(tp);
	rtlmq_sync_xdp_features_locked(tp);
	return 0;
}

int rtlmq_detach_and_close_locked(struct rtlmq_private *tp, bool prepare_wol)
{
	int ret = 0;

	lockdep_assert_held(&tp->control_lock);

	if (tp->state == RTLMQ_STATE_DEAD)
		return 0;

	netif_device_detach(tp->ndev);
	if (netif_running(tp->ndev))
		ret = rtlmq_close_device(tp);
	if (!ret && prepare_wol)
		ret = rtlmq_prepare_wol_suspend(tp);
	return ret;
}

int rtlmq_retire_failed_control_update_locked(struct rtlmq_private *tp)
{
	struct rtlmq_netpoll_guard guard;
	int ret;

	rtlmq_netpoll_guard_acquire_locked(tp, &guard);
	ret = rtlmq_detach_and_close_locked(tp, false);
	rtlmq_netpoll_guard_release_locked(tp, &guard);

	return ret;
}

static int rtlmq_republish_probed_locked(struct rtlmq_private *tp)
{
	int ret;

	ASSERT_RTNL();
	lockdep_assert_held(&tp->control_lock);

	if (tp->state != RTLMQ_STATE_PROBED ||
	    !rtlmq_mmio_ordinary_authorized(tp))
		return -EBUSY;

	ret = rtlmq_apply_wol(tp);
	if (ret)
		return ret;
	rtlmq_sync_netdev_tso_limits(tp);
	rtlmq_sync_xdp_features_locked(tp);
	netif_device_attach(tp->ndev);
	rtlmq_sync_link_state(tp);
	return 0;
}

static int rtlmq_apply_recovery_transaction_inner_locked(
	struct rtlmq_private *tp,
	u32 reset_reasons,
	const struct rtlmq_requested_runtime_snapshot *rollback_requested,
	bool reopen_rollback
)
{
	struct rtlmq_requested_runtime_snapshot captured_requested;
	const struct rtlmq_requested_runtime_snapshot *committed_requested =
		rollback_requested;
	int ret;

	if (!tp || !tp->ndev)
		return -EINVAL;
	lockdep_assert_held(&tp->control_lock);

	if (tp->state == RTLMQ_STATE_DEAD)
		return -ENODEV;
	if (tp->state == RTLMQ_STATE_REMOVING)
		return -ENODEV;

	/*
	 * OPEN recovery resets and rebuilds hardware. PROBED recovery reuses
	 * ndo_open() setup if the interface was running. An administratively
	 * down interface only needs its saved request restored and reattachment.
	 */
	if (tp->state == RTLMQ_STATE_OPEN) {
		if (!reset_reasons)
			return -EINVAL;
		if (!committed_requested) {
			if (!rtlmq_requested_runtime_committed(tp))
				return -EBUSY;
			rtlmq_capture_requested_runtime(tp, &captured_requested);
			committed_requested = &captured_requested;
		}
		if (!rtlmq_requested_snapshot_committed(committed_requested))
			return -EINVAL;
		return rtlmq_reset_device(tp, reset_reasons,
					  committed_requested,
					  reopen_rollback);
	}
	if (tp->state != RTLMQ_STATE_PROBED)
		return -EBUSY;
	if (reopen_rollback)
		return -EINVAL;
	if (committed_requested) {
		if (!rtlmq_requested_snapshot_committed(committed_requested))
			return -EINVAL;
		rtlmq_restore_requested_runtime(tp, committed_requested);
	} else if (!rtlmq_requested_runtime_committed(tp)) {
		return -EBUSY;
	}

	if (netif_running(tp->ndev))
		ret = rtlmq_open_device(tp);
	else
		ret = rtlmq_republish_probed_locked(tp);

	return ret;
}

int rtlmq_recover_queued_reset_locked(struct rtlmq_private *tp,
				      u32 reset_reasons
)
{
	lockdep_assert_held(&tp->control_lock);

	if (tp->state == RTLMQ_STATE_REMOVING ||
	    tp->state == RTLMQ_STATE_DEAD)
		return -ENODEV;
	if (tp->platform_quiesce_owner != RTLMQ_PLATFORM_QUIESCE_NONE)
		return -EBUSY;

	if (tp->state == RTLMQ_STATE_OPEN) {
		return rtlmq_apply_recovery_transaction_locked(tp, reset_reasons,
						       NULL, false
		);
	}

	if (tp->state == RTLMQ_STATE_PROBED) {
		return rtlmq_apply_recovery_transaction_locked(tp, reset_reasons,
						       NULL, false
		);
	}

	return -EBUSY;
}

int rtlmq_apply_recovery_transaction_locked(
	struct rtlmq_private *tp,
	u32 reset_reasons,
	const struct rtlmq_requested_runtime_snapshot *rollback_requested,
	bool reopen_rollback
)
{
	struct rtlmq_netpoll_guard netpoll_guard;
	int ret;

	rtlmq_netpoll_guard_acquire_locked(tp, &netpoll_guard);
	ret = rtlmq_apply_recovery_transaction_inner_locked(tp, reset_reasons,
							    rollback_requested, reopen_rollback
	);
	rtlmq_netpoll_guard_release_locked(tp, &netpoll_guard);

	return ret;
}
