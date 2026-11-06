// SPDX-License-Identifier: GPL-2.0-only

#define CREATE_TRACE_POINTS
#include "rtlmq_trace.h"
#undef CREATE_TRACE_POINTS

#include "rtlmq_irq.h"
#include "rtlmq_hw.h"
#include "rtlmq_runtime_accessors.h"
#include "rtlmq_regs.h"

#include <linux/completion.h>
#include <linux/io.h>
#include <linux/smp.h>

#include "rtlmq_rx.h"
#include "rtlmq_ptp.h"
#include "rtlmq_tx_private.h"
#include "rtlmq_tx.h"

/*
 * Callers validate vector/device/ring associations before looping.
 * This helper checks only vector-array and allocated-ring bounds.
 */
static inline bool
rtlmq_rx_vector_ring_index_valid(const struct rtlmq_vector *qv,
				 unsigned int index, u16 ring_slots,
				 u16 *ring_index)
{
	if (unlikely(index >= ARRAY_SIZE(qv->rx_ring_indices)))
		return false;

	*ring_index = READ_ONCE(qv->rx_ring_indices[index]);
	return *ring_index < ring_slots;
}

static u32 rtlmq_irq_active_mask_snapshot(const struct rtlmq_private *tp);
static void rtlmq_unmask_vector_data_irq(struct rtlmq_vector *vector);
static void
rtlmq_note_vector_tx_desc_status_pending(struct rtlmq_vector *vector);

void rtlmq_enable_msix_mask(struct rtlmq_private *tp, u32 mask)
{
	if (mask && rtlmq_mmio_ordinary_authorized(tp))
		writel(mask, tp->mmio_addr + RTLMQ_IMR_V2_SET_REG);
}

void rtlmq_disable_msix_mask(struct rtlmq_private *tp, u32 mask)
{
	if (mask && rtlmq_mmio_ordinary_authorized(tp))
		writel(mask, tp->mmio_addr + RTLMQ_IMR_V2_CLEAR_REG);
}

static void rtlmq_ack_msix_mask(struct rtlmq_private *tp, u32 mask)
{
	if (mask && rtlmq_mmio_ordinary_authorized(tp))
		writel(mask, tp->mmio_addr + RTLMQ_ISR_V2);
}

static u32 rtlmq_read_intr_status(struct rtlmq_private *tp)
{
	if (!rtlmq_mmio_ordinary_authorized(tp))
		return ~0U;

	return readl(tp->mmio_addr + RTLMQ_INTR_STATUS);
}

static void rtlmq_ack_intr_status(struct rtlmq_private *tp, u32 status)
{
	if (!rtlmq_mmio_ordinary_authorized(tp))
		return;

	writel(status, tp->mmio_addr + RTLMQ_INTR_STATUS);
}

static void rtlmq_hw_write_intr_mask(struct rtlmq_private *tp, u32 mask)
{
	if (!rtlmq_mmio_ordinary_authorized(tp))
		return;

	writel(mask, tp->mmio_addr + RTLMQ_INTR_MASK);
}

static void rtlmq_irq_restore_mask(struct rtlmq_private *tp)
{
	if (!tp || !rtlmq_mmio_ordinary_authorized(tp))
		return;

	rtlmq_hw_write_intr_mask(tp, rtlmq_irq_active_mask_snapshot(tp));
}

static void rtlmq_irq_apply_legacy_mask_snapshot(struct rtlmq_private *tp,
						 u32 mask)
{
	if (!tp || !rtlmq_mmio_ordinary_authorized(tp))
		return;

	rtlmq_hw_write_intr_mask(tp, mask);
}

static void rtlmq_clear_vector_data_irq_state(struct rtlmq_private *tp)
{
	unsigned int i;

	if (!tp || !tp->vectors)
		return;

	for (i = 0; i < tp->allocated_vectors; i++)
		WRITE_ONCE(tp->vectors[i].data_irq_mask_current, 0);
}

static void rtlmq_disable_all_msix_messages(struct rtlmq_private *tp)
{
	if (!rtlmq_mmio_ordinary_authorized(tp))
		return;

	writel(~0U, tp->mmio_addr + RTLMQ_IMR_V2_CLEAR_REG);

	rtlmq_clear_vector_data_irq_state(tp);
}

static void rtlmq_disable_msix_message(struct rtlmq_private *tp, u8 message_id)
{
	rtlmq_disable_msix_mask(tp, BIT(message_id));
}

static void rtlmq_ack_msix_message(struct rtlmq_private *tp, u8 message_id)
{
	rtlmq_ack_msix_mask(tp, BIT(message_id));
}

static void rtlmq_enable_msix_message(struct rtlmq_private *tp, u8 message_id)
{
	rtlmq_enable_msix_mask(tp, BIT(message_id));
}

static bool rtlmq_vector_published(const struct rtlmq_vector *vector)
{
	return rtlmq_irq_layout_has_publication(vector->num_irq_sources,
						vector->num_rx_rings,
						vector->num_tx_rings);
}

bool rtlmq_vector_layout_published(const struct rtlmq_vector_layout *vector)
{
	return rtlmq_irq_layout_has_publication(vector->num_irq_sources,
						vector->num_rx_rings,
						vector->num_tx_rings);
}

static u32 rtlmq_irq_active_mask_snapshot(const struct rtlmq_private *tp)
{
	u32 mask = RTLMQ_INT_BOOT_MASK;
	unsigned int i;

	if (!tp->vectors)
		return mask;

	for (i = 0; i < tp->allocated_vectors; i++) {
		const struct rtlmq_vector *vector = &tp->vectors[i];

		if (!vector->irq_requested)
			continue;

		mask |= rtlmq_filter_empty_rx_irq_mask(vector, READ_ONCE(vector->irq_mask));
	}

	return mask;
}

static u32 rtlmq_vector_event_irq_mask(const struct rtlmq_vector *vector)
{
	u32 mask = 0;

	if (!vector || !READ_ONCE(vector->irq_requested))
		return 0;

	mask = READ_ONCE(vector->msix_event_mask);
	if (mask)
		return mask;

	return rtlmq_vector_requested_source_mask(vector, RTLMQ_IRQ_SOURCE_EVENT);
}

static void rtlmq_enable_vector_irq_sources(struct rtlmq_vector *vector,
					    enum rtlmq_irq_source_role role)
{
	struct rtlmq_private *tp = vector->priv;
	u32 mask;

	if (role == RTLMQ_IRQ_SOURCE_DATA) {
		mask = rtlmq_vector_resolve_data_irq_wanted(vector);
		u32 current_mask = READ_ONCE(vector->data_irq_mask_current);
		u32 enable_mask =
			rtlmq_vector_data_irq_hw_enable_mask(vector, mask);

		if (!enable_mask)
			return;

		WRITE_ONCE(vector->data_irq_mask_current,
			   current_mask | enable_mask);
		rtlmq_enable_msix_mask(tp, enable_mask);
		return;
	}

	mask = rtlmq_vector_event_irq_mask(vector);
	if (!mask)
		return;

	rtlmq_enable_msix_mask(tp, mask);
}

static void rtlmq_disable_vector_data_irq(struct rtlmq_vector *vector)
{
	struct rtlmq_private *tp = vector->priv;
	u32 current_mask = READ_ONCE(vector->data_irq_mask_current);

	if (!current_mask)
		return;

	WRITE_ONCE(vector->data_irq_mask_current, 0);
	rtlmq_disable_msix_mask(tp, current_mask);
}

static void rtlmq_ack_vector_data_irq(struct rtlmq_vector *vector)
{
	struct rtlmq_private *tp = vector->priv;
	u32 mask = READ_ONCE(vector->data_irq_mask_rx_tx);

	if (!mask)
		mask = rtlmq_vector_wake_data_mask(vector);
	if (!mask)
		return;

	rtlmq_ack_msix_mask(tp, mask);
}

void rtlmq_enable_intrs(struct rtlmq_private *tp)
{
	unsigned int i;

	if (!rtlmq_mmio_ordinary_authorized(tp))
		return;

	if (rtlmq_uses_msix_message_routing(tp)) {
		for (i = 0; i < tp->allocated_vectors; i++) {
			struct rtlmq_vector *vector = &tp->vectors[i];

			if (!rtlmq_vector_published(vector))
				continue;

			rtlmq_unmask_vector_data_irq(vector);
			rtlmq_enable_vector_irq_sources(vector,
							RTLMQ_IRQ_SOURCE_EVENT);
		}
		return;
	}

	if (tp->vectors) {
		for (i = 0; i < tp->allocated_vectors; i++) {
			struct rtlmq_vector *vector = &tp->vectors[i];
			u32 wanted_mask;

			if (!vector->irq_requested)
				continue;
			wanted_mask =
				rtlmq_vector_resolve_data_irq_wanted(vector);
			WRITE_ONCE(vector->data_irq_mask_current,
				   wanted_mask);
		}
	}
	rtlmq_irq_restore_mask(tp);
}

enum rtlmq_irq_handler_rearm_kind {
	RTLMQ_IRQ_HANDLER_REARM_MSIX_MESSAGE,
	RTLMQ_IRQ_HANDLER_REARM_VECTOR_DATA,
	RTLMQ_IRQ_HANDLER_REARM_LEGACY,
};

static void
rtlmq_irq_handler_rearm(struct rtlmq_vector *vector, enum rtlmq_irq_handler_rearm_kind kind,
			u16 message_id)
{
	unsigned long flags;

	spin_lock_irqsave(&vector->napi_rearm_lock, flags);
	lockdep_assert_held(&vector->napi_rearm_lock);
	if (!READ_ONCE(vector->napi_rearm_allowed))
		goto out_unlock;

	switch (kind) {
	case RTLMQ_IRQ_HANDLER_REARM_MSIX_MESSAGE:
		rtlmq_enable_msix_message(vector->priv, message_id);
		break;
	case RTLMQ_IRQ_HANDLER_REARM_VECTOR_DATA:
		rtlmq_enable_vector_irq_sources(vector, RTLMQ_IRQ_SOURCE_DATA);
		break;
	case RTLMQ_IRQ_HANDLER_REARM_LEGACY:
		rtlmq_enable_intrs(vector->priv);
		break;
	}

out_unlock:
	spin_unlock_irqrestore(&vector->napi_rearm_lock, flags);
}

void rtlmq_disable_intrs(struct rtlmq_private *tp)
{
	if (!rtlmq_mmio_ordinary_authorized(tp))
		return;

	if (rtlmq_staged_wants_msix_message_routing(tp)) {
		rtlmq_disable_all_msix_messages(tp);
		return;
	}

	rtlmq_clear_vector_data_irq_state(tp);
	rtlmq_hw_write_intr_mask(tp, RTLMQ_INT_BOOT_MASK);
}

void rtlmq_mask_and_ack_intrs(struct rtlmq_private *tp)
{
	u32 status;

	if (!rtlmq_mmio_ordinary_authorized(tp))
		return;

	if (rtlmq_staged_wants_msix_message_routing(tp)) {
		rtlmq_disable_all_msix_messages(tp);
		status = readl(tp->mmio_addr + RTLMQ_ISR_V2);
		if (status && status != ~0U)
			writel(status, tp->mmio_addr + RTLMQ_ISR_V2);
		return;
	}

	rtlmq_disable_intrs(tp);
	status = rtlmq_read_intr_status(tp);
	if (status && status != ~0U)
		rtlmq_ack_intr_status(tp, status);
}

int rtlmq_mask_and_ack_intrs_recovery(struct rtlmq_private *tp)
{
	void __iomem *mmio = rtlmq_mmio_recovery_addr(tp);
	u32 status;

	if (!mmio)
		return -ENODEV;

	/*
	 * PCI core has restored BAR access, but normal driver MMIO is still
	 * blocked. Use this mapping only to mask and acknowledge interrupts.
	 * Do not call normal IRQ helpers, update vectors, or schedule NAPI.
	 */
	if (rtlmq_staged_wants_msix_message_routing(tp)) {
		writel(~0U, mmio + RTLMQ_IMR_V2_CLEAR_REG);

		status = readl(mmio + RTLMQ_ISR_V2);
		if (status == ~0U)
			return -ENODEV;
		if (status)
			writel(status, mmio + RTLMQ_ISR_V2);

		return 0;
	}

	writel(RTLMQ_INT_BOOT_MASK, mmio + RTLMQ_INTR_MASK);
	status = readl(mmio + RTLMQ_INTR_STATUS);
	if (status == ~0U)
		return -ENODEV;
	if (status)
		writel(status, mmio + RTLMQ_INTR_STATUS);

	return 0;
}

static struct rtlmq_irq_source *
rtlmq_find_irq_source(struct rtlmq_vector *vector, int irq)
{
	unsigned int i;

	for (i = 0; i < vector->num_irq_sources; i++) {
		struct rtlmq_irq_source *source = &vector->irq_sources[i];

		if (source->requested && source->irq == irq)
			return source;
	}

	return NULL;
}

static void
rtlmq_arm_vector_tx_own_recheck(struct rtlmq_vector *vector,
				const struct rtlmq_irq_source *source)
{
	bool source_mapped = false;
	struct rtlmq_private *tp;
	u16 ring_slots;
	unsigned int i;

	if (!vector || !READ_ONCE(vector->num_tx_rings))
		return;
	if (source &&
	    (!(source->kind_flags & RTLMQ_IRQ_SOURCE_KIND_TX) ||
	     source->role != RTLMQ_IRQ_SOURCE_DATA))
		return;

	tp = vector->priv;
	if (!tp || !tp->tx_rings)
		return;

	ring_slots = rtlmq_owned_tx_ring_slots(tp);
	for (i = 0; i < READ_ONCE(vector->num_tx_rings); i++) {
		struct rtlmq_tx_ring *ring =
			rtlmq_vector_tx_ring_at(tp, vector, i, ring_slots);

		if (!ring)
			continue;
		if (source &&
		    READ_ONCE(vector->tx_message_ids[i]) !=
			    source->message_id)
			continue;

		source_mapped = true;
		rtlmq_tx_ring_arm_own_recheck(ring, RTLMQ_TX_OWN_RECHECK_IRQ_BIT);
	}

	if (!source || source_mapped)
		return;

	/*
	 * Recheck all relevant rings for unknown or combined interrupt sources.
	 * Keep a latch per ring so checking q0 does not clear q1's pending work.
	 */
	for (i = 0; i < READ_ONCE(vector->num_tx_rings); i++) {
		struct rtlmq_tx_ring *ring =
			rtlmq_vector_tx_ring_at(tp, vector, i, ring_slots);

		if (ring)
			rtlmq_tx_ring_arm_own_recheck(ring, RTLMQ_TX_OWN_RECHECK_IRQ_BIT);
	}
}

static void
rtlmq_arm_vector_rx_own_recheck(struct rtlmq_vector *vector,
				const struct rtlmq_irq_source *source)
{
	unsigned int i;

	if (!vector || !READ_ONCE(vector->num_rx_rings))
		return;
	if (source &&
	    (!(source->kind_flags & RTLMQ_IRQ_SOURCE_KIND_RX) ||
	     source->role != RTLMQ_IRQ_SOURCE_DATA))
		return;

	for (i = 0; i < READ_ONCE(vector->num_rx_rings); i++) {
		u16 ring_index;

		if (source &&
		    READ_ONCE(vector->rx_message_ids[i]) !=
			    source->message_id)
			continue;
		ring_index = READ_ONCE(vector->rx_ring_indices[i]);
		if (ring_index >= BITS_PER_LONG)
			continue;
		set_bit(ring_index, &vector->rx_own_recheck_pending_mask);
	}
}

static bool rtlmq_irq_runtime_service_allowed(const struct rtlmq_vector *vector)
{
	const struct rtlmq_private *tp;

	if (!vector)
		return false;

	tp = vector->priv;
	/* Acquire the committed profile before testing its validity. */
	return tp &&
	       READ_ONCE(tp->state) == RTLMQ_STATE_OPEN &&
	       smp_load_acquire(&tp->runtime_published) &&
	       READ_ONCE(tp->active_profile.valid) &&
	       READ_ONCE(vector->irq_requested);
}

/*
 * request_irq() can invoke the handler before the new runtime is committed.
 * Mask and acknowledge reset/link interrupts in that interval without
 * entering the normal legacy/MSI-X handler. Service starts at the OPEN commit.
 */
static irqreturn_t rtlmq_reject_unpublished_irq(struct rtlmq_vector *vector,
						int irq)
{
	struct rtlmq_irq_source *source;
	struct rtlmq_private *tp;

	if (!vector)
		return IRQ_HANDLED;
	tp = vector->priv;
	if (!tp)
		return IRQ_HANDLED;

	if (rtlmq_staged_wants_msix_message_routing(tp)) {
		source = rtlmq_find_irq_source(vector, irq);
		if (source) {
			rtlmq_disable_msix_message(tp, source->message_id);
			rtlmq_ack_msix_message(tp, source->message_id);
		} else {
			rtlmq_disable_all_msix_messages(tp);
		}
	} else {
		rtlmq_mask_and_ack_intrs(tp);
	}

	return IRQ_HANDLED;
}

static void rtlmq_handle_link_interrupt(struct rtlmq_private *tp)
{
	if (rtlmq_ptp_defer_link_event(tp))
		return;

	if (tp->ndev->phydev)
		phy_mac_interrupt(tp->ndev->phydev);
	else
		rtlmq_request_link_state_sync(tp);
}

static bool rtlmq_legacy_intr_status_has_data(u32 status)
{
	return status & (RTLMQ_INT_RX_OK | RTLMQ_INT_RX_ERR |
			 RTLMQ_INT_RX_DESC_UNAVAIL | RTLMQ_INT_FOVW |
			 RTLMQ_INT_TX_OK | RTLMQ_INT_TX_ERR);
}

static void rtlmq_note_vector_rx_service_irq_cause(struct rtlmq_vector *vector,
						   u64 cause)
{
	if (!vector || !cause)
		return;

	if (cause & RTLMQ_RX_SERVICE_IRQ_CAUSE_RX)
		set_bit(RTLMQ_RX_SERVICE_IRQ_CAUSE_RX_BIT,
			&vector->rx_service_irq_cause_pending);
	if (cause & RTLMQ_RX_SERVICE_IRQ_CAUSE_TX)
		set_bit(RTLMQ_RX_SERVICE_IRQ_CAUSE_TX_BIT,
			&vector->rx_service_irq_cause_pending);
	if (cause & RTLMQ_RX_SERVICE_IRQ_CAUSE_UNKNOWN)
		set_bit(RTLMQ_RX_SERVICE_IRQ_CAUSE_UNKNOWN_BIT,
			&vector->rx_service_irq_cause_pending);
}

static void
rtlmq_note_vector_rx_service_irq_source(struct rtlmq_vector *vector,
					const struct rtlmq_irq_source *source)
{
	u64 cause = 0;

	if (!source || source->role != RTLMQ_IRQ_SOURCE_DATA)
		return;

	if (source->kind_flags & RTLMQ_IRQ_SOURCE_KIND_RX)
		cause |= RTLMQ_RX_SERVICE_IRQ_CAUSE_RX;
	if (source->kind_flags & RTLMQ_IRQ_SOURCE_KIND_TX)
		cause |= RTLMQ_RX_SERVICE_IRQ_CAUSE_TX;
	if (!cause)
		cause = RTLMQ_RX_SERVICE_IRQ_CAUSE_UNKNOWN;

	rtlmq_note_vector_rx_service_irq_cause(vector, cause);
}

static void
rtlmq_note_vector_rx_service_legacy_irq_status(struct rtlmq_vector *vector,
					       u32 status)
{
	const u32 rx_bits = RTLMQ_INT_RX_OK | RTLMQ_INT_RX_ERR |
			    RTLMQ_INT_RX_DESC_UNAVAIL | RTLMQ_INT_FOVW;
	const u32 tx_bits = RTLMQ_INT_TX_OK | RTLMQ_INT_TX_ERR;
	u64 cause = 0;

	if (status & rx_bits)
		cause |= RTLMQ_RX_SERVICE_IRQ_CAUSE_RX;
	if (status & tx_bits)
		cause |= RTLMQ_RX_SERVICE_IRQ_CAUSE_TX;

	rtlmq_note_vector_rx_service_irq_cause(vector, cause);
}

static void
rtlmq_irq_schedule_napi_locked(struct rtlmq_vector *vector,
			       enum rtlmq_napi_schedule_source schedule_source)
{
	lockdep_assert_held(&vector->napi_rearm_lock);
	/* Record missed schedules even when NAPI already owns the vector. */
	(void)rtlmq_schedule_vector_napi(vector, schedule_source);
}

static irqreturn_t
rtlmq_msix_vector_interrupt(struct rtlmq_vector *vector, int irq)
{
	struct rtlmq_private *tp = vector->priv;
	enum rtlmq_napi_schedule_source schedule_source =
		RTLMQ_NAPI_SCHED_SOURCE_UNKNOWN;
	struct rtlmq_irq_source *source;

	source = rtlmq_find_irq_source(vector, irq);
	if (!source)
		return IRQ_NONE;

	if (source->role == RTLMQ_IRQ_SOURCE_EVENT) {
		rtlmq_disable_msix_message(tp, source->message_id);
		rtlmq_ack_msix_message(tp, source->message_id);
		rtlmq_handle_link_interrupt(tp);
		rtlmq_irq_handler_rearm(vector, RTLMQ_IRQ_HANDLER_REARM_MSIX_MESSAGE,
					source->message_id);
	} else {
		unsigned long flags;
		bool napi_enabled;
		bool tx_source =
			source->kind_flags & RTLMQ_IRQ_SOURCE_KIND_TX;

		/*
		 * Serialize mask/ACK and NAPI scheduling with completion
		 * rearm. Once teardown closes the gate, it owns source
		 * masking; do not touch the registers here.
		 */
		spin_lock_irqsave(&vector->napi_rearm_lock, flags);
		if (!READ_ONCE(vector->napi_rearm_allowed)) {
			spin_unlock_irqrestore(&vector->napi_rearm_lock, flags);
			return IRQ_HANDLED;
		}

		napi_enabled = READ_ONCE(vector->napi_enabled);
		if (napi_enabled) {
			if (source->kind_flags &
			    RTLMQ_IRQ_SOURCE_KIND_RX)
				rtlmq_arm_vector_rx_own_recheck(vector, source);
			if (tx_source) {
				rtlmq_note_vector_tx_desc_status_pending(vector);
				rtlmq_arm_vector_tx_own_recheck(vector, source);
				set_bit(RTLMQ_VECTOR_TX_RESWEEP_HINT_TX_IRQ,
					&vector->tx_resweep_hints);
			}
		}
		/*
		 * One NAPI instance may service several interrupt sources.
		 * Mask and acknowledge all data sources on this vector
		 * before scheduling it. Do not rely on hardware auto-mask.
		 */
		rtlmq_disable_vector_data_irq(vector);
		rtlmq_ack_vector_data_irq(vector);
		if (!napi_enabled) {
			spin_unlock_irqrestore(&vector->napi_rearm_lock, flags);
			rtlmq_irq_handler_rearm(vector, RTLMQ_IRQ_HANDLER_REARM_VECTOR_DATA, 0);
			return IRQ_HANDLED;
		}

		rtlmq_note_vector_rx_service_irq_source(vector, source);
		rtlmq_irq_schedule_napi_locked(vector, schedule_source);
		spin_unlock_irqrestore(&vector->napi_rearm_lock, flags);
	}

	return IRQ_HANDLED;
}

static irqreturn_t
rtlmq_legacy_vector_interrupt(struct rtlmq_vector *vector, int irq)
{
	struct rtlmq_private *tp = vector->priv;
	enum rtlmq_napi_schedule_source schedule_source =
		RTLMQ_NAPI_SCHED_SOURCE_UNKNOWN;
	bool data_cause;
	u32 status;

	status = rtlmq_read_intr_status(tp);
	if (!status || status == ~0U)
		return IRQ_NONE;

	data_cause = rtlmq_legacy_intr_status_has_data(status);
	if (!data_cause) {
		rtlmq_disable_intrs(tp);
		rtlmq_ack_intr_status(tp, status);
		if (status & RTLMQ_INT_LINK_CHG)
			rtlmq_handle_link_interrupt(tp);
		if (!vector->napi_enabled ||
		    !(READ_ONCE(vector->napi.state) & NAPIF_STATE_SCHED)) {
			rtlmq_irq_handler_rearm(vector, RTLMQ_IRQ_HANDLER_REARM_LEGACY, 0);
		}
		return IRQ_HANDLED;
	}

	{
		unsigned long flags;
		bool napi_enabled;

		/*
		 * Legacy INTx/MSI does not guarantee per-vector auto-mask.
		 * Serialize mask, ACK, and NAPI scheduling with completion rearm.
		 */
		spin_lock_irqsave(&vector->napi_rearm_lock, flags);
		if (!READ_ONCE(vector->napi_rearm_allowed)) {
			spin_unlock_irqrestore(&vector->napi_rearm_lock, flags);
			return IRQ_HANDLED;
		}

		napi_enabled = READ_ONCE(vector->napi_enabled);
		if (napi_enabled &&
		    (status & (RTLMQ_INT_RX_OK | RTLMQ_INT_RX_ERR |
			       RTLMQ_INT_RX_DESC_UNAVAIL | RTLMQ_INT_FOVW)))
			rtlmq_arm_vector_rx_own_recheck(vector, NULL);
		if (napi_enabled &&
		    (status & (RTLMQ_INT_TX_OK | RTLMQ_INT_TX_ERR))) {
			rtlmq_note_vector_tx_desc_status_pending(vector);
			rtlmq_arm_vector_tx_own_recheck(vector, NULL);
			set_bit(RTLMQ_VECTOR_TX_RESWEEP_HINT_TX_IRQ,
				&vector->tx_resweep_hints);
		}

		rtlmq_disable_intrs(tp);
		rtlmq_ack_intr_status(tp, status);

		if (!napi_enabled) {
			spin_unlock_irqrestore(&vector->napi_rearm_lock, flags);
			if (status & RTLMQ_INT_LINK_CHG)
				rtlmq_handle_link_interrupt(tp);
			rtlmq_irq_handler_rearm(vector, RTLMQ_IRQ_HANDLER_REARM_LEGACY, 0);
			return IRQ_HANDLED;
		}

		rtlmq_note_vector_rx_service_legacy_irq_status(vector,
							       status);
		rtlmq_irq_schedule_napi_locked(vector, schedule_source);
		spin_unlock_irqrestore(&vector->napi_rearm_lock, flags);
	}

	if (status & RTLMQ_INT_LINK_CHG)
		rtlmq_handle_link_interrupt(tp);

	return IRQ_HANDLED;
}

irqreturn_t rtlmq_vector_interrupt(int irq, void *data)
{
	struct rtlmq_vector *vector = data;

	if (!rtlmq_irq_runtime_service_allowed(vector))
		return rtlmq_reject_unpublished_irq(vector, irq);

	if (rtlmq_runtime_vector_uses_msix_message_routing(vector))
		return rtlmq_msix_vector_interrupt(vector, irq);

	return rtlmq_legacy_vector_interrupt(vector, irq);
}

#ifdef CONFIG_NET_POLL_CONTROLLER
void rtlmq_poll_controller(struct net_device *ndev)
{
	const struct rtlmq_runtime_config *cfg;
	struct rtlmq_private *tp;
	unsigned int i;

	if (!ndev || !netif_running(ndev) || !netif_device_present(ndev))
		return;

	tp = netdev_priv(ndev);
	cfg = rtlmq_published_runtime_config(tp);
	if (!cfg || !READ_ONCE(tp->active_resource_generation) || !tp->vectors ||
	    cfg->num_vectors > READ_ONCE(tp->allocated_vectors))
		return;

	/*
	 * netpoll invokes each registered NAPI instance with budget zero after
	 * this callback. Set only a TX hint; let normal NAPI reclaim update BQL.
	 * Calling the IRQ handler would mask MSI-X data sources that budget-zero
	 * polling cannot rearm, since it must not call NAPI completion.
	 * Netpoll's device semaphore excludes runtime replacement. Vector map
	 * epochs are independent of the device resource-generation counter.
	 */
	for (i = 0; i < cfg->num_vectors; i++) {
		struct rtlmq_vector *vector = &tp->vectors[i];

		if (READ_ONCE(vector->priv) != tp ||
		    !READ_ONCE(vector->irq_requested) ||
		    !READ_ONCE(vector->napi_enabled) ||
		    !READ_ONCE(vector->num_tx_rings))
			continue;

		rtlmq_note_vector_tx_desc_status_pending(vector);
		set_bit(RTLMQ_VECTOR_TX_RESWEEP_HINT_FORCE,
			&vector->tx_resweep_hints);
	}
}
#endif

/* Admit IRQ and software service under the same mask/rearm gate. */

bool rtlmq_schedule_vector_napi(struct rtlmq_vector *vector,
				enum rtlmq_napi_schedule_source source)
{
	bool prepared;

	lockdep_assert_held(&vector->napi_rearm_lock);

	/*
	 * Call __napi_schedule() only if napi_schedule_prep() succeeds.
	 * Otherwise NAPI is already scheduled or unavailable. Use the core
	 * state machine for busy-poll and threaded NAPI too.
	 */

	prepared = napi_schedule_prep(&vector->napi);
	if (prepared) {
		/*
		 * IRQ callers already masked and acknowledged their sources.
		 * Software admission must also mask before core can run the
		 * poll, without manufacturing an ACK or OWN-reread authority.
		 * The gate keeps an older completion's unmask on either side
		 * of this entire admission, never between its check and mask.
		 */
		if (READ_ONCE(vector->data_irq_mask_current)) {
			if (rtlmq_runtime_vector_uses_msix_message_routing(vector))
				rtlmq_disable_vector_data_irq(vector);
			else
				rtlmq_disable_intrs(vector->priv);
		}
		__napi_schedule(&vector->napi);
		if (trace_rtlmq_service_request_enabled())
			trace_rtlmq_service_request(vector->priv->ndev->name, vector->index,
						    READ_ONCE(vector->generation), source, true);
		return true;
	}

	if (trace_rtlmq_service_request_enabled())
		trace_rtlmq_service_request(vector->priv->ndev->name, vector->index,
					    READ_ONCE(vector->generation), source, false);
	return false;
}

static void
rtlmq_note_vector_tx_desc_status_pending(struct rtlmq_vector *vector)
{
	struct rtlmq_private *tp;
	u16 ring_slots;
	unsigned int i;

	if (!vector || !vector->num_tx_rings)
		return;

	tp = vector->priv;
	if (!tp || !tp->tx_rings)
		return;

	ring_slots = rtlmq_owned_tx_ring_slots(tp);
	for (i = 0; i < vector->num_tx_rings; i++) {
		struct rtlmq_tx_ring *ring;

		ring = rtlmq_vector_tx_ring_at(tp, vector, i, ring_slots);
		if (!ring)
			continue;

		rtlmq_tx_ring_note_desc_status_pending(ring);
	}
}

void rtlmq_force_vector_tx_resweep(struct rtlmq_private *tp, u16 vector_index)
{
	struct rtlmq_vector *vector;
	unsigned long flags;

	if (!tp || !tp->vectors || vector_index >= tp->allocated_vectors)
		return;

	vector = &tp->vectors[vector_index];
	spin_lock_irqsave(&vector->napi_rearm_lock, flags);
	if (!READ_ONCE(vector->napi_enabled) ||
	    !READ_ONCE(vector->napi_rearm_allowed))
		goto out_unlock;

	rtlmq_note_vector_tx_desc_status_pending(vector);
	set_bit(RTLMQ_VECTOR_TX_RESWEEP_HINT_FORCE, &vector->tx_resweep_hints);
	rtlmq_schedule_vector_napi(vector, RTLMQ_NAPI_SCHED_TX_RESWEEP);

out_unlock:
	spin_unlock_irqrestore(&vector->napi_rearm_lock, flags);
}

u64 rtlmq_rx_ready_queue_mask(const struct rtlmq_vector *qv)
{
	const struct rtlmq_private *tp;
	unsigned int num_rx_rings;
	u64 queue_mask = 0;
	u16 ring_slots;
	unsigned int i;

	if (!qv)
		return 0;

	num_rx_rings = READ_ONCE(qv->num_rx_rings);
	if (!num_rx_rings)
		return 0;

	tp = qv->priv;
	if (!tp || !tp->rx_rings)
		return 0;

	ring_slots = rtlmq_owned_rx_ring_slots(tp);
	for (i = 0; i < num_rx_rings; i++) {
		struct rtlmq_rx_ring *ring;
		u16 ring_index;

		if (!rtlmq_rx_vector_ring_index_valid(qv, i, ring_slots, &ring_index))
			continue;

		ring = &tp->rx_rings[ring_index];
		if (rtlmq_rx_ring_head_ready(ring))
			queue_mask |= BIT_ULL(ring_index);
	}

	return queue_mask;
}

bool rtlmq_rx_ready(const struct rtlmq_vector *qv)
{
	return !!rtlmq_rx_ready_queue_mask(qv);
}

bool rtlmq_vector_has_tx_pending_desc(const struct rtlmq_vector *vector)
{
	const struct rtlmq_private *tp;
	unsigned int num_tx_rings;
	u16 ring_slots;
	unsigned int i;

	if (!vector)
		return false;

	num_tx_rings = READ_ONCE(vector->num_tx_rings);
	if (!num_tx_rings)
		return false;

	tp = vector->priv;
	if (!tp || !tp->tx_rings)
		return false;

	ring_slots = rtlmq_owned_tx_ring_slots(tp);
	for (i = 0; i < num_tx_rings; i++) {
		struct rtlmq_tx_ring *ring;

		ring = rtlmq_vector_tx_ring_at(tp, vector, i, ring_slots);
		if (!ring)
			continue;

		if (READ_ONCE(ring->next_to_use_count) !=
		    READ_ONCE(ring->next_to_clean_count))
			return true;
	}

	return false;
}

bool rtlmq_tx_ring_has_reclaimable_desc(const struct rtlmq_tx_ring *ring,
					const struct rtlmq_tx_reclaim_wait *wait)
{
	const struct rtlmq_tx_ring_read_mostly *rm;
	unsigned int entry;
	u16 ring_count;
	u32 pending;
	u32 opts1;

	if (!ring)
		return false;

	/*
	 * Acquire the producer count before reading descriptor and completion
	 * metadata. Check the pending range before sampling the head descriptor.
	 */
	pending = rtlmq_tx_ring_desc_status_hint_window(ring);
	if (!pending)
		return false;

	rm = rtlmq_tx_ring_rm(ring);
	if (unlikely(!READ_ONCE(rm->desc)))
		return false;

	ring_count = READ_ONCE(rm->ring_count);
	if (unlikely(!ring_count))
		return false;

	entry = READ_ONCE(ring->next_to_clean);
	if (unlikely(entry >= ring_count))
		entry &= READ_ONCE(rm->ring_mask);

	/*
	 * NAPI has not released ring ownership or run another reclaimer since
	 * packet preflight. Refill may extend hw but cannot recycle this packet.
	 * Recheck its actual OWN-set blocker, not an already-returned head.
	 * Cursor/generation changes invalidate the location, never grant reclaim.
	 */
	if (wait && wait->valid &&
	    wait->generation == READ_ONCE(ring->generation) &&
	    wait->clean_count == READ_ONCE(ring->next_to_clean_count) &&
	    pending < ring_count &&
	    rtlmq_tx_ring_distance(wait->blocked_count, wait->clean_count) <
		    pending)
		entry = wait->blocked_count & READ_ONCE(rm->ring_mask);

	opts1 = rtlmq_tx_desc_status_read(
		rtlmq_tx_desc_at_base(rm->desc, rtlmq_tx_ring_desc_stride(ring),
				      entry));
	return !(opts1 & RTLMQ_DESC_OWN);
}

bool rtlmq_vector_has_tx_reclaimable_desc(const struct rtlmq_vector *vector,
					  const struct rtlmq_tx_reclaim_wait *wait)
{
	const struct rtlmq_private *tp;
	unsigned int num_tx_rings;
	u16 ring_slots;
	unsigned int i;

	if (!vector)
		return false;

	num_tx_rings = READ_ONCE(vector->num_tx_rings);
	if (!num_tx_rings)
		return false;

	tp = vector->priv;
	if (!tp || !tp->tx_rings)
		return false;

	ring_slots = rtlmq_owned_tx_ring_slots(tp);
	for (i = 0; i < num_tx_rings; i++) {
		struct rtlmq_tx_ring *ring;

		ring = rtlmq_vector_tx_ring_at(tp, vector, i, ring_slots);
		if (!ring)
			continue;

		if (rtlmq_tx_ring_has_reclaimable_desc(ring,
						       wait ? &wait[ring->queue_index] : NULL))
			return true;
	}

	return false;
}

bool rtlmq_vector_has_tx_status_pending(const struct rtlmq_vector *vector)
{
	if (!vector)
		return false;

	return READ_ONCE(vector->tx_resweep_hints) != 0;
}

static void rtlmq_unmask_vector_data_irq(struct rtlmq_vector *vector)
{
	struct rtlmq_private *tp;
	u32 current_mask;
	u32 wanted_mask;
	u32 enable_mask;

	if (!vector)
		return;

	tp = vector->priv;
	if (!tp || !rtlmq_mmio_ordinary_authorized(tp) || !vector->irq_requested ||
	    !vector->napi_enabled ||
	    !READ_ONCE(vector->napi_rearm_allowed))
		return;

	wanted_mask = rtlmq_vector_resolve_data_irq_wanted(vector);
	if (!wanted_mask)
		return;

	current_mask = READ_ONCE(vector->data_irq_mask_current);
	if (current_mask == wanted_mask &&
	    !READ_ONCE(vector->tx_irq_armed_wanted)) {
		return;
	}

	enable_mask = rtlmq_vector_data_irq_hw_enable_mask(vector,
							   wanted_mask);
	WRITE_ONCE(vector->data_irq_mask_current, wanted_mask);
	if (rtlmq_runtime_vector_uses_msix_message_routing(vector)) {
		rtlmq_enable_msix_mask(tp, enable_mask);
		return;
	}

	/*
	 * Restore the active IMR mask directly for legacy single-vector mode.
	 * rtlmq_enable_intrs() may already be iterating MSI-X vectors.
	 */
	rtlmq_irq_restore_mask(tp);
}

bool rtlmq_prepare_napi_rearm(struct rtlmq_vector *vector,
			      struct rtlmq_napi_rearm_plan *plan)
{
	struct rtlmq_private *tp;

	if (!vector || !plan)
		return false;

	*plan = (struct rtlmq_napi_rearm_plan) {};
	tp = vector->priv;
	if (!tp || !rtlmq_mmio_ordinary_authorized(tp) || !vector->irq_requested ||
	    !vector->napi_enabled)
		return false;

	/*
	 * Snapshot runtime state before NAPI completion lets teardown proceed
	 * on another CPU. Rearm may then use only this snapshot and the vector
	 * rearm gate, not live ring state. TX intent remains mutable under that
	 * gate: a producer may publish after this snapshot. A zero RX mask is
	 * valid: an empty XSK RX queue stays masked until wakeup supplies buffers.
	 */
	plan->tp = tp;
	plan->generation = READ_ONCE(vector->generation);
	plan->uses_msix_message_routing =
		rtlmq_runtime_vector_uses_msix_message_routing(vector);
	if (plan->uses_msix_message_routing) {
		plan->rx_mask = rtlmq_vector_rx_data_irq_wanted(vector);
		plan->tx_mask = READ_ONCE(vector->data_irq_mask_tx_only);
	} else {
		plan->legacy_data_mask =
			rtlmq_vector_resolve_data_irq_wanted(vector);
		plan->legacy_mask = rtlmq_irq_active_mask_snapshot(tp);
	}
	plan->valid = true;
	return true;
}

void rtlmq_apply_napi_rearm(struct rtlmq_vector *vector,
			    const struct rtlmq_napi_rearm_plan *plan)
{
	unsigned long flags;
	u32 disable_mask;
	u32 current_mask;
	u32 wanted_mask;

	if (!vector || !plan || !plan->valid ||
	    !plan->completion_succeeded || !plan->tp)
		return;

	/*
	 * Teardown closes this gate before disabling IRQ or MMIO access.
	 * NAPI completion has already run; do not read live runtime, ring,
	 * or layout state here.
	 */
	spin_lock_irqsave(&vector->napi_rearm_lock, flags);
	lockdep_assert_held(&vector->napi_rearm_lock);
	if (!READ_ONCE(vector->napi_rearm_allowed) ||
	    plan->generation != READ_ONCE(vector->generation) ||
	    (READ_ONCE(vector->napi.state) & NAPIF_STATE_SCHED))
		goto out_unlock;

	/*
	 * Preserve the snapshot's RX/XSK filtering, but not stale TX intent.
	 * The same lock owns producer arming, IRQ mask/schedule and teardown:
	 * a late producer either contributes here or rearms after us. A new
	 * NAPI owner or generation must never inherit this poll's unmask.
	 */
	if (plan->uses_msix_message_routing) {
		wanted_mask = plan->rx_mask;
		if (READ_ONCE(vector->tx_irq_armed_wanted))
			wanted_mask |= plan->tx_mask;
	} else {
		wanted_mask = plan->legacy_data_mask;
	}
	current_mask = READ_ONCE(vector->data_irq_mask_current);
	disable_mask = current_mask & ~wanted_mask;

	if (plan->uses_msix_message_routing) {
		if (disable_mask)
			rtlmq_disable_msix_mask(plan->tp, disable_mask);
		WRITE_ONCE(vector->data_irq_mask_current, wanted_mask);
		if (wanted_mask)
			rtlmq_enable_msix_mask(plan->tp, wanted_mask);
	} else {
		WRITE_ONCE(vector->data_irq_mask_current, wanted_mask);
		rtlmq_irq_apply_legacy_mask_snapshot(plan->tp,
						     plan->legacy_mask);
	}

out_unlock:
	spin_unlock_irqrestore(&vector->napi_rearm_lock, flags);
}
