// SPDX-License-Identifier: GPL-2.0-only

#include <linux/cpu.h>
#include <linux/interrupt.h>
#include <linux/sched/topology.h>
#include <linux/slab.h>
#include <linux/string.h>

#include "rtlmq_hw.h"
#include "rtlmq_runtime_accessors.h"
#include "rtlmq_irq.h"
#include "rtlmq_runtime.h"
#include "rtlmq_lifecycle.h"
#include "rtlmq.h"
#include "rtlmq_regs.h"
#include "rtlmq_ring.h"
#include "rtlmq_xdp.h"

struct rtlmq_irq_allocation_plan {
	u16 min_vectors;
	u16 max_vectors;
	unsigned int flags;
	enum rtlmq_irq_mode irq_mode;
};

static int rtlmq_program_candidate_irq_mode(struct rtlmq_private *tp);

static const struct rtlmq_runtime_config *
rtlmq_staged_irq_runtime_config(const struct rtlmq_private *tp)
{
	const struct rtlmq_runtime_config *cfg;

	if (!tp)
		return NULL;

	/*
	 * Setup may need the candidate configuration before commit; teardown
	 * may need active state after candidate discard. Keep this selection
	 * separate from normal IRQ/NAPI accessors.
	 */
	cfg = rtlmq_candidate_runtime_config(tp);
	if (cfg)
		return cfg;

	return rtlmq_active_runtime_config(tp);
}

bool rtlmq_staged_wants_msix_message_routing(const struct rtlmq_private *tp)
{
	const struct rtlmq_runtime_config *cfg =
		rtlmq_staged_irq_runtime_config(tp);

	return cfg && rtlmq_hw_has_msix_message_routing(tp) &&
	       cfg->routing_mode == RTLMQ_ROUTING_RX_PER_VECTOR_TX_COUPLED;
}

bool rtlmq_candidate_uses_msix_message_routing(const struct rtlmq_private *tp)
{
	const struct rtlmq_runtime_config *cfg =
		rtlmq_candidate_runtime_config(tp);

	return cfg && rtlmq_hw_has_msix_message_routing(tp) &&
	       cfg->routing_mode == RTLMQ_ROUTING_RX_PER_VECTOR_TX_COUPLED &&
	       cfg->irq_mode == RTLMQ_IRQ_MODE_MSIX;
}

/*
 * Snapshot state that IRQ fallback must not change: chip policy, active DMA
 * allocations, and requested/active XDP and XSK bindings. Only the private
 * candidate may be rebuilt for a narrower interrupt mode.
 */
struct rtlmq_irq_fallback_authority {
	const struct rtlmq_chip_profile *profile;
	struct bpf_prog *requested_xdp_prog;
	struct bpf_prog *active_xdp_prog;
	struct xsk_buff_pool *requested_xsk_pools[RTLMQ_IMPL_MAX_RX_QUEUES];
	u64 rx_dma_ring_mask;
	u64 tx_dma_ring_mask;
	u32 active_resource_generation;
	u16 requested_num_rx_queues;
	u16 requested_num_tx_queues;
	u16 num_rx_queues;
	u16 num_tx_queues;
	u16 num_vectors;
	enum rtlmq_routing_mode routing_mode;
	u32 real_num_rx_queues;
	u32 real_num_tx_queues;
	u32 xdp_features;
	bool active_profile_valid;
	bool runtime_published;
};

static int rtlmq_set_msix_interrupt_type(struct rtlmq_private *tp)
{
	u8 int_cfg0;

	if (!rtlmq_mmio_ordinary_authorized(tp))
		return -ENODEV;
	if (!rtlmq_hw_has_msix_message_routing(tp) ||
	    rtlmq_candidate_or_published_irq_mode(tp) != RTLMQ_IRQ_MODE_MSIX)
		return 0;

	int_cfg0 = readb(tp->mmio_addr + RTLMQ_INT_CFG0);
	if (int_cfg0 == U8_MAX)
		return -ENODEV;
	int_cfg0 &= ~rtlmq_hw_msix_int_cfg0_clear_mask(tp);

	/*
	 * Match upstream behavior: INT_CFG0 message-routing programming is a
	 * MSI-X concern. Leave fallback MSI/INTx paths on the bootstrap
	 * defaults so legacy interrupt delivery continues to use IMR/ISR.
	 */
	int_cfg0 |= RTLMQ_INT_CFG0_ENABLE;

	{
		writeb(int_cfg0, tp->mmio_addr + RTLMQ_INT_CFG0);
	}
	return 0;
}

static unsigned int rtlmq_irq_mode_flags(enum rtlmq_irq_mode irq_mode)
{
	switch (irq_mode) {
	case RTLMQ_IRQ_MODE_INTX:
		return PCI_IRQ_INTX;
	case RTLMQ_IRQ_MODE_MSI:
		return PCI_IRQ_MSI;
	case RTLMQ_IRQ_MODE_MSIX:
		return PCI_IRQ_MSIX;
	case RTLMQ_IRQ_MODE_NONE:
	default:
		return 0;
	}
}

u16 rtlmq_required_msix_vectors(const struct rtlmq_runtime_config *cfg,
				const struct rtlmq_runtime_layout_state *layout)
{
	u16 required_vectors = cfg->num_vectors;
	unsigned int i;

	if (!layout || !layout->valid)
		return required_vectors;

	/*
	 * Message routing uses sparse hardware message IDs (for example TX/link
	 * events above queue-local RX IDs) and request_irq() resolves those IDs
	 * through pci_irq_vector(). The allocated MSI-X width must therefore
	 * cover the highest published message ID + 1 rather than only the number
	 * of software vectors/NAPI instances.
	 */
	for (i = 0; i < cfg->num_vectors; i++) {
		const struct rtlmq_vector_layout *vector = &layout->vectors[i];
		unsigned int j;

		if (!rtlmq_vector_layout_published(vector))
			continue;

		for (j = 0; j < vector->num_irq_sources; j++) {
			const struct rtlmq_irq_source *source;
			u16 source_vectors;

			source = &vector->irq_sources[j];
			if (source->message_id == RTLMQ_IRQ_SOURCE_NONE)
				continue;

			source_vectors = (u16)source->message_id + 1;
			if (source_vectors > required_vectors)
				required_vectors = source_vectors;
		}
	}

	return required_vectors;
}

static int
rtlmq_build_irq_allocation_plan(const struct rtlmq_private *tp,
				const struct rtlmq_runtime_config *cfg,
				const struct rtlmq_runtime_layout_state *layout,
				struct rtlmq_irq_allocation_plan *plan)
{
	if (!cfg || !plan || !rtlmq_hw_contract_present(tp) ||
	    !rtlmq_qualified_policy_present(tp))
		return -EINVAL;
	if (!rtlmq_chip_routing_mode_admitted(tp, cfg->routing_mode))
		return -EOPNOTSUPP;

	switch (cfg->routing_mode) {
	case RTLMQ_ROUTING_SINGLE_VECTOR:
		plan->min_vectors = RTLMQ_BOOT_VECTORS;
		plan->max_vectors = RTLMQ_BOOT_VECTORS;
		/*
		 * The bootstrap profile has one software vector;
		 * request one concrete interrupt mode at a time so fallback
		 * between MSI and INTx is explicit instead of allocator-driven.
		 */
		if (rtlmq_chip_irq_mode_admitted(tp, RTLMQ_IRQ_MODE_MSI))
			plan->irq_mode = RTLMQ_IRQ_MODE_MSI;
		else if (rtlmq_chip_irq_mode_admitted(tp, RTLMQ_IRQ_MODE_INTX))
			plan->irq_mode = RTLMQ_IRQ_MODE_INTX;
		else
			return -EOPNOTSUPP;
		plan->flags = rtlmq_irq_mode_flags(plan->irq_mode);
		return 0;
	case RTLMQ_ROUTING_RX_PER_VECTOR_TX_COUPLED:
		plan->min_vectors = rtlmq_required_msix_vectors(cfg, layout);

		if (!rtlmq_chip_irq_mode_admitted(tp, RTLMQ_IRQ_MODE_MSIX))
			return -EOPNOTSUPP;
		if (!plan->min_vectors ||
		    plan->min_vectors > rtlmq_hw_max_irq_vectors(tp))
			return -EINVAL;
		plan->max_vectors = plan->min_vectors;
		plan->irq_mode = RTLMQ_IRQ_MODE_MSIX;
		plan->flags = rtlmq_irq_mode_flags(plan->irq_mode);
		return 0;
	default:
		return -EINVAL;
	}
}

static bool rtlmq_can_fallback_to_msi(const struct rtlmq_private *tp,
				      const struct rtlmq_runtime_config *cfg)
{
	return rtlmq_chip_irq_mode_admitted(tp, RTLMQ_IRQ_MODE_MSI) &&
	       cfg &&
	       cfg->routing_mode == RTLMQ_ROUTING_RX_PER_VECTOR_TX_COUPLED;
}

static enum rtlmq_irq_mode rtlmq_detect_irq_mode(const struct pci_dev *pdev)
{
	if (pdev->msix_enabled)
		return RTLMQ_IRQ_MODE_MSIX;
	if (pdev->msi_enabled)
		return RTLMQ_IRQ_MODE_MSI;

	return RTLMQ_IRQ_MODE_INTX;
}

static void
rtlmq_mark_candidate_vector_irq_requested(struct rtlmq_vector *vector)
{
	if (WARN_ON_ONCE(!vector || !vector->priv ||
			 vector->index >= RTLMQ_IMPL_MAX_NAPI))
		return;

	vector->priv->transaction.irq_requested_mask |=
		BIT_ULL(vector->index);
}

static void rtlmq_clear_vector_irq_request_owner(struct rtlmq_vector *vector)
{
	struct rtlmq_private *tp;
	u64 vector_mask;

	if (!vector || !vector->priv)
		return;
	if (WARN_ON_ONCE(vector->index >= RTLMQ_IMPL_MAX_NAPI))
		return;

	tp = vector->priv;
	vector_mask = BIT_ULL(vector->index);
	tp->transaction.irq_requested_mask &= ~vector_mask;
}

static int rtlmq_asymmetric_data_irq_cpu(void)
{
	unsigned long min_capacity = ULONG_MAX;
	unsigned long max_capacity = 0;
	int target = -1;
	int cpu;

	cpus_read_lock();
	for_each_online_cpu(cpu) {
		unsigned long capacity = arch_scale_cpu_capacity(cpu);

		if (capacity < min_capacity)
			min_capacity = capacity;
		if (capacity > max_capacity) {
			max_capacity = capacity;
			target = cpu;
		}
	}
	cpus_read_unlock();

	return max_capacity > min_capacity ? target : -1;
}

static bool
rtlmq_irq_source_prefers_fast_cpu(const struct rtlmq_vector *vector,
				  const struct rtlmq_irq_source *source)
{
	u8 kind = source->kind_flags &
		  (RTLMQ_IRQ_SOURCE_KIND_RX | RTLMQ_IRQ_SOURCE_KIND_TX);

	return kind == RTLMQ_IRQ_SOURCE_KIND_RX ||
	       (kind == RTLMQ_IRQ_SOURCE_KIND_TX &&
		!vector->num_rx_rings && vector->num_tx_rings);
}

static void rtlmq_set_data_irq_affinity(struct rtlmq_vector *vector,
					struct rtlmq_irq_source *source, int cpu)
{
	if (!rtlmq_irq_source_prefers_fast_cpu(vector, source))
		return;

	irq_set_affinity(source->irq, cpumask_of(cpu));
}

void rtlmq_apply_data_irq_affinity(struct rtlmq_private *tp)
{
	unsigned int i;
	int cpu;

	if (!tp || !tp->vectors)
		return;

	cpu = rtlmq_asymmetric_data_irq_cpu();
	if (cpu < 0)
		return;

	for (i = 0; i < tp->allocated_vectors; i++) {
		struct rtlmq_vector *vector = &tp->vectors[i];
		unsigned int j;

		for (j = 0; j < vector->num_irq_sources; j++) {
			struct rtlmq_irq_source *source = &vector->irq_sources[j];

			if (source->requested)
				rtlmq_set_data_irq_affinity(vector, source, cpu);
		}
	}
}

static int rtlmq_request_vector_irq(struct rtlmq_vector *vector)
{
	struct rtlmq_private *tp = vector->priv;
	unsigned long irqflags = pci_dev_msi_enabled(tp->pdev) ? 0 : IRQF_SHARED;
	unsigned int i;
	int irq;
	int ret;

	if (rtlmq_candidate_uses_msix_message_routing(tp)) {
		for (i = 0; i < vector->num_irq_sources; i++) {
			struct rtlmq_irq_source *source = &vector->irq_sources[i];

			irq = pci_irq_vector(tp->pdev, source->message_id);
			if (irq < 0) {
				ret = irq;
				goto err_free_requested;
			}

			ret = request_irq(irq, rtlmq_vector_interrupt, 0,
					  tp->ndev->name, vector);
			if (ret)
				goto err_free_requested;

			source->irq = irq;
			source->requested = true;
			if (!vector->irq_requested)
				vector->irq = irq;
			vector->irq_requested = true;
			rtlmq_mark_candidate_vector_irq_requested(vector);
		}

		if (!vector->irq_requested)
			return -EINVAL;

		return 0;
	}

	irq = pci_irq_vector(tp->pdev, vector->index);
	if (irq < 0)
		return irq;

	ret = request_irq(irq, rtlmq_vector_interrupt, irqflags,
			  tp->ndev->name, vector);
	if (ret)
		return ret;

	vector->irq = irq;
	vector->irq_requested = true;
	rtlmq_mark_candidate_vector_irq_requested(vector);

	return 0;

err_free_requested:
	if (WARN_ON_ONCE(!rtlmq_irq_release_allowed(vector->priv)))
		return ret;
	while (i--) {
		struct rtlmq_irq_source *source = &vector->irq_sources[i];

		if (!source->requested)
			continue;

		free_irq(source->irq, vector);
		source->requested = false;
		source->irq = -1;
	}
	vector->irq_requested = false;
	vector->irq = -1;
	rtlmq_clear_vector_irq_request_owner(vector);

	return ret;
}

static void rtlmq_free_vector_irq(struct rtlmq_vector *vector)
{
	int primary_irq = vector->irq;
	unsigned int i;

	if (!vector->irq_requested) {
		rtlmq_clear_vector_irq_request_owner(vector);
		return;
	}

	free_irq(vector->irq, vector);
	vector->irq_requested = false;
	vector->irq = -1;

	for (i = 0; i < vector->num_irq_sources; i++) {
		struct rtlmq_irq_source *source = &vector->irq_sources[i];

		if (!source->requested)
			continue;
		if (source->irq == primary_irq)
			continue;

		free_irq(source->irq, vector);
		source->requested = false;
		source->irq = -1;
	}

	for (i = 0; i < vector->num_irq_sources; i++) {
		vector->irq_sources[i].requested = false;
		vector->irq_sources[i].irq = -1;
	}
	rtlmq_clear_vector_irq_request_owner(vector);
}

static u16 rtlmq_vector_capacity(const struct rtlmq_private *tp)
{
	u16 max_vectors;

	if (!rtlmq_hw_contract_present(tp))
		return 0;

	/*
	 * Allocate enough vector/NAPI storage at probe for later queue changes,
	 * not just bootstrap's single vector. Use the supported software-vector
	 * limit, not the raw MSI-X table width.
	 */
	max_vectors = rtlmq_hw_max_runtime_vectors(tp);
	if (!max_vectors)
		return 0;

	return max_t(u16, max_vectors, RTLMQ_BOOT_VECTORS);
}

int rtlmq_alloc_vectors(struct rtlmq_private *tp)
{
	u16 max_vectors;
	unsigned int i;
	int ret;

	if (!tp->rx_rings || !tp->tx_rings)
		return -EINVAL;

	/*
	 * Allocate vector/NAPI storage for the hardware-supported runtime limit,
	 * independent of current active or requested queue counts.
	 */
	max_vectors = rtlmq_vector_capacity(tp);
	if (!max_vectors)
		return -EINVAL;

	tp->vectors = kcalloc(max_vectors, sizeof(*tp->vectors), GFP_KERNEL);
	if (!tp->vectors)
		return -ENOMEM;
	tp->allocated_vectors = max_vectors;

	for (i = 0; i < tp->allocated_vectors; i++) {
		struct rtlmq_vector *vector = &tp->vectors[i];

		vector->priv = tp;
		vector->irq = -1;
		vector->index = i;
		vector->irq_mask = RTLMQ_INT_BOOTSTRAP_MASK;
		spin_lock_init(&vector->napi_rearm_lock);
		rtlmq_reset_vector_queue_map(vector);
	}

	ret = rtlmq_rebuild_vector_layout(tp);
	if (ret)
		goto err_free_vectors;

	return 0;

err_free_vectors:
	rtlmq_free_vectors(tp);

	return ret;
}

void rtlmq_free_vectors(struct rtlmq_private *tp)
{
	unsigned int i;
	int ret;

	if (!tp || !rtlmq_release_allowed(tp))
		return;

	if (!tp->vectors) {
		if (WARN_ON_ONCE(!rtlmq_vector_storage_release_allowed(tp)))
			return;
		return;
	}

	ret = rtlmq_release_irqs(tp);
	if (ret)
		return;
	if (WARN_ON_ONCE(!rtlmq_vector_storage_release_allowed(tp)))
		return;

	for (i = 0; i < tp->allocated_vectors; i++)
		rtlmq_reset_vector_queue_map(&tp->vectors[i]);

	kfree(tp->vectors);
	tp->vectors = NULL;
	tp->allocated_vectors = 0;
	tp->active_irq_vectors_allocated = false;
	tp->transaction.irq_vectors_allocated = false;
	tp->transaction.napi_registered_mask = 0;
	tp->transaction.irq_requested_mask = 0;
}

static int rtlmq_capture_irq_fallback_authority(const struct rtlmq_private *tp,
						struct rtlmq_irq_fallback_authority *authority)
{
	const struct rtlmq_runtime_config *cfg;

	if (!tp || !tp->ndev || !authority)
		return -EINVAL;

	cfg = rtlmq_candidate_runtime_config(tp);
	if (!cfg)
		return -EINVAL;

	memset(authority, 0, sizeof(*authority));
	authority->profile = tp->profile;
	authority->requested_xdp_prog =
		READ_ONCE(tp->requested_xdp_prog);
	authority->active_xdp_prog = READ_ONCE(tp->xdp_prog);
	memcpy(authority->requested_xsk_pools, tp->requested_xsk_pools,
	       sizeof(authority->requested_xsk_pools));
	authority->rx_dma_ring_mask =
		rtlmq_derive_active_rx_dma_ring_mask(tp);
	authority->tx_dma_ring_mask =
		rtlmq_derive_active_tx_dma_ring_mask(tp);
	authority->active_resource_generation =
		READ_ONCE(tp->active_resource_generation);
	authority->requested_num_rx_queues = cfg->requested_num_rx_queues;
	authority->requested_num_tx_queues = cfg->requested_num_tx_queues;
	authority->num_rx_queues = cfg->num_rx_queues;
	authority->num_tx_queues = cfg->num_tx_queues;
	authority->num_vectors = cfg->num_vectors;
	authority->routing_mode = cfg->routing_mode;
	authority->real_num_rx_queues = tp->ndev->real_num_rx_queues;
	authority->real_num_tx_queues = tp->ndev->real_num_tx_queues;
	authority->xdp_features = tp->ndev->xdp_features;
	authority->active_profile_valid = tp->active_profile.valid;
	authority->runtime_published = READ_ONCE(tp->runtime_published);

	return 0;
}

static bool rtlmq_fallback_dma_build_not_started(const struct rtlmq_private *tp)
{
	unsigned int i;

	if (!tp || tp->transaction.rx_dma_ring_mask ||
	    tp->transaction.tx_dma_ring_mask)
		return false;
	for (i = 0; i < RTLMQ_IMPL_MAX_RX_QUEUES; i++)
		if (tp->transaction.rx_rings[i])
			return false;
	for (i = 0; i < RTLMQ_IMPL_MAX_TX_QUEUES; i++)
		if (rtlmq_candidate_tx_storage_present(tp, i))
			return false;
	return true;
}

static int
rtlmq_validate_irq_fallback_authority(struct rtlmq_private *tp,
				      const struct rtlmq_irq_fallback_authority *authority)
{
	struct rtlmq_runtime_config *candidate_cfg;
	u16 i;

	if (!tp || !tp->ndev || !authority)
		return -EINVAL;

	candidate_cfg = rtlmq_candidate_runtime_config_owned_mut(tp);
	if (!candidate_cfg)
		return -EINVAL;

	if (tp->profile != authority->profile)
		return -EPERM;
	if (READ_ONCE(tp->requested_xdp_prog) !=
		    authority->requested_xdp_prog ||
	    READ_ONCE(tp->xdp_prog) != authority->active_xdp_prog ||
	    tp->ndev->xdp_features != authority->xdp_features)
		return -EPERM;
	if (tp->ndev->real_num_rx_queues != authority->real_num_rx_queues ||
	    tp->ndev->real_num_tx_queues != authority->real_num_tx_queues)
		return -EPERM;
	if (rtlmq_derive_active_rx_dma_ring_mask(tp) !=
		    authority->rx_dma_ring_mask ||
	    rtlmq_derive_active_tx_dma_ring_mask(tp) !=
		    authority->tx_dma_ring_mask ||
	    READ_ONCE(tp->active_resource_generation) !=
		    authority->active_resource_generation ||
	    tp->active_profile.valid != authority->active_profile_valid ||
	    READ_ONCE(tp->runtime_published) != authority->runtime_published)
		return -EPERM;

	if (candidate_cfg->requested_num_rx_queues !=
		    authority->requested_num_rx_queues ||
	    candidate_cfg->requested_num_tx_queues !=
		    authority->requested_num_tx_queues ||
	    candidate_cfg->num_rx_queues > authority->num_rx_queues ||
	    candidate_cfg->num_tx_queues > authority->num_tx_queues ||
	    candidate_cfg->num_vectors > authority->num_vectors)
		return -EINVAL;

	if (authority->routing_mode == RTLMQ_ROUTING_SINGLE_VECTOR &&
	    candidate_cfg->routing_mode != RTLMQ_ROUTING_SINGLE_VECTOR)
		return -EINVAL;
	if (authority->routing_mode != RTLMQ_ROUTING_SINGLE_VECTOR &&
	    candidate_cfg->routing_mode != RTLMQ_ROUTING_SINGLE_VECTOR &&
	    candidate_cfg->routing_mode !=
		    RTLMQ_ROUTING_RX_PER_VECTOR_TX_COUPLED)
		return -EINVAL;

	/*
	 * Reject fallback if it removes a queue with a requested XSK binding.
	 * IRQ allocation must not silently detach userspace's pool.
	 */
	for (i = 0; i < RTLMQ_IMPL_MAX_RX_QUEUES; i++) {
		if (rtlmq_requested_xsk_pool(tp, i) !=
		    authority->requested_xsk_pools[i])
			return -EPERM;
		if (i >= candidate_cfg->num_rx_queues &&
		    authority->requested_xsk_pools[i])
			return -EBUSY;
	}

	return 0;
}

static int rtlmq_prepare_fallback_profile(struct rtlmq_private *tp,
					  u16 num_rx_queues,
					  u16 num_tx_queues
)
{
	struct rtlmq_irq_fallback_authority authority;
	const struct rtlmq_runtime_config *cfg;
	int ret;

	/*
	 * Change queue counts only before allocating candidate rings. Narrowing
	 * resource masks after allocation would leave untracked rings to leak.
	 */
	if (!rtlmq_fallback_dma_build_not_started(tp))
		return -EBUSY;

	ret = rtlmq_capture_irq_fallback_authority(tp, &authority);
	if (ret)
		return ret;

	ret = rtlmq_prepare_candidate_runtime_profile(tp, num_rx_queues,
						      num_tx_queues);
	if (ret)
		goto err_clear_unpublished;

	/*
	 * Check that fallback changed only the candidate before staging RSS/IRQs.
	 */
	ret = rtlmq_validate_irq_fallback_authority(tp, &authority);
	if (ret)
		goto err_clear_unpublished;

	/*
	 * IRQ allocation precedes bus mastering. Update only candidate software
	 * state here; normal runtime setup programs rings/RSS/queues after
	 * MAC/PHY initialization.
	 */
	cfg = rtlmq_candidate_runtime_config(tp);
	if (!cfg) {
		ret = -EINVAL;
		goto err_clear_unpublished;
	}
	rtlmq_stage_candidate_rss_for_config(tp, cfg);

	/*
	 * Verify RSS staging did not change state outside the candidate.
	 */
	ret = rtlmq_validate_irq_fallback_authority(tp, &authority);
	if (ret)
		goto err_clear_unpublished;

	return 0;

err_clear_unpublished:
	rtlmq_clear_unpublished_vector_queue_maps(tp);
	return ret;
}

static int
rtlmq_prepare_single_vector_fallback(struct rtlmq_private *tp,
				     struct rtlmq_irq_allocation_plan *plan,
				     enum rtlmq_irq_mode irq_mode)
{
	int ret;

	if (!plan || !rtlmq_chip_irq_mode_admitted(tp, irq_mode))
		return -EOPNOTSUPP;

	/*
	 * Rebuild the full bootstrap profile before retrying so queue and vector
	 * counts cannot be left over from different interrupt-mode attempts.
	 */
	ret = rtlmq_prepare_fallback_profile(tp, RTLMQ_BOOT_RX_QUEUES,
					     RTLMQ_BOOT_TX_QUEUES
	);
	if (ret)
		return ret;

	plan->min_vectors = RTLMQ_BOOT_VECTORS;
	plan->max_vectors = RTLMQ_BOOT_VECTORS;
	plan->irq_mode = irq_mode;
	plan->flags = rtlmq_irq_mode_flags(irq_mode);
	if (rtlmq_candidate_runtime_config_owned_mut(tp))
		rtlmq_candidate_runtime_config_owned_mut(tp)->irq_mode =
			irq_mode;

	return 0;
}

static int rtlmq_prepare_msi_fallback(struct rtlmq_private *tp,
				      struct rtlmq_irq_allocation_plan *plan)
{
	const struct rtlmq_runtime_config *cfg;

	cfg = rtlmq_candidate_runtime_config(tp);
	if (!rtlmq_can_fallback_to_msi(tp, cfg))
		return -EOPNOTSUPP;

	return rtlmq_prepare_single_vector_fallback(tp, plan,
						    RTLMQ_IRQ_MODE_MSI);
}

static int rtlmq_prepare_intx_fallback(struct rtlmq_private *tp,
				       struct rtlmq_irq_allocation_plan *plan)
{
	return rtlmq_prepare_single_vector_fallback(tp, plan,
						    RTLMQ_IRQ_MODE_INTX);
}

static int
rtlmq_prepare_degraded_msix_fallback(struct rtlmq_private *tp,
				     struct rtlmq_irq_allocation_plan *plan)
{
	const struct rtlmq_runtime_config *cfg;
	const struct rtlmq_topology *topology;
	u16 next_rx_queues;
	int ret;

	cfg = rtlmq_candidate_runtime_config(tp);
	if (!cfg)
		return -EINVAL;

	if (!plan ||
	    cfg->routing_mode != RTLMQ_ROUTING_RX_PER_VECTOR_TX_COUPLED ||
	    cfg->num_rx_queues <= RTLMQ_BOOT_RX_QUEUES)
		return -EOPNOTSUPP;

	/*
	 * Degrade RSS fanout one step at a time (4 -> 2 -> 1 in this tree).
	 * Keep TX at one queue and retry with a narrower
	 * multiqueue profile before collapsing all the way to single-vector
	 * MSI/INTx fallback.
	 */
	next_rx_queues = cfg->num_rx_queues >> 1;
	if (next_rx_queues <= RTLMQ_BOOT_RX_QUEUES)
		return -EOPNOTSUPP;
	/*
	 * An absent intermediate MSI-X shape must not clear the current
	 * candidate before the MSI/INTx fallback gets a chance to build.
	 */
	topology = rtlmq_find_qualified_topology(tp, next_rx_queues,
						 RTLMQ_BOOT_TX_QUEUES);
	if (!topology ||
	    topology->routing != RTLMQ_ROUTING_RX_PER_VECTOR_TX_COUPLED ||
	    !(topology->irq_modes & BIT(RTLMQ_IRQ_MODE_MSIX)))
		return -EOPNOTSUPP;

	ret = rtlmq_prepare_fallback_profile(tp, next_rx_queues,
					     RTLMQ_BOOT_TX_QUEUES
	);
	if (ret)
		return ret;

	cfg = rtlmq_candidate_runtime_config(tp);
	ret = rtlmq_build_irq_allocation_plan(tp, cfg,
					      rtlmq_candidate_runtime_layout(tp),
					      plan);
	if (ret)
		goto err_clear_unpublished;

	if (plan->irq_mode != RTLMQ_IRQ_MODE_MSIX)
		goto err_not_msix;

	if (rtlmq_candidate_runtime_config_owned_mut(tp))
		rtlmq_candidate_runtime_config_owned_mut(tp)->irq_mode =
			plan->irq_mode;

	return 0;

err_not_msix:
	ret = -EOPNOTSUPP;
err_clear_unpublished:
	rtlmq_clear_unpublished_vector_queue_maps(tp);
	return ret;
}

bool rtlmq_prepare_open_irq_fallback(struct rtlmq_private *tp,
				     enum rtlmq_irq_mode failed_mode,
				     int alloc_ret,
				     enum rtlmq_irq_mode *next_mode)
{
	struct rtlmq_irq_allocation_plan fallback_plan;
	int ret;

	if (next_mode)
		*next_mode = RTLMQ_IRQ_MODE_NONE;

	switch (failed_mode) {
	case RTLMQ_IRQ_MODE_MSIX:
		ret = rtlmq_prepare_degraded_msix_fallback(tp,
							   &fallback_plan);
		if (!ret) {
			if (next_mode)
				*next_mode = RTLMQ_IRQ_MODE_NONE;
			netdev_warn(tp->ndev,
				    "MSI-X allocation failed (%d), retrying degraded MSI-X profile rxq=%u txq=%u vec=%u\n",
				    alloc_ret, rtlmq_candidate_or_published_num_rx_queues(tp),
				    rtlmq_candidate_or_published_num_tx_queues(tp),
				    rtlmq_candidate_or_published_num_vectors(tp));
			return true;
		}

		ret = rtlmq_prepare_msi_fallback(tp, &fallback_plan);
		if (!ret) {
			if (next_mode)
				*next_mode = RTLMQ_IRQ_MODE_MSI;
			netdev_warn(tp->ndev,
				    "MSI-X allocation failed (%d), retrying MSI fallback profile rxq=%u txq=%u vec=%u\n",
				    alloc_ret, rtlmq_candidate_or_published_num_rx_queues(tp),
				    rtlmq_candidate_or_published_num_tx_queues(tp),
				    rtlmq_candidate_or_published_num_vectors(tp));
			return true;
		}

		ret = rtlmq_prepare_intx_fallback(tp, &fallback_plan);
		if (!ret) {
			if (next_mode)
				*next_mode = RTLMQ_IRQ_MODE_INTX;
			netdev_warn(tp->ndev,
				    "MSI-X allocation failed (%d), retrying INTx fallback profile rxq=%u txq=%u vec=%u\n",
				    alloc_ret, rtlmq_candidate_or_published_num_rx_queues(tp),
				    rtlmq_candidate_or_published_num_tx_queues(tp),
				    rtlmq_candidate_or_published_num_vectors(tp));
			return true;
		}
		break;
	case RTLMQ_IRQ_MODE_MSI:
		ret = rtlmq_prepare_intx_fallback(tp, &fallback_plan);
		if (!ret) {
			if (next_mode)
				*next_mode = RTLMQ_IRQ_MODE_INTX;
			netdev_warn(tp->ndev,
				    "MSI allocation failed (%d), retrying INTx fallback profile rxq=%u txq=%u vec=%u\n",
				    alloc_ret, rtlmq_candidate_or_published_num_rx_queues(tp),
				    rtlmq_candidate_or_published_num_tx_queues(tp),
				    rtlmq_candidate_or_published_num_vectors(tp));
			return true;
		}
		break;
	case RTLMQ_IRQ_MODE_INTX:
	case RTLMQ_IRQ_MODE_NONE:
	default:
		break;
	}

	return false;
}

int rtlmq_prepare_candidate_irq_vectors(struct rtlmq_private *tp)
{
	enum rtlmq_irq_mode forced_irq_mode = RTLMQ_IRQ_MODE_NONE;
	const struct rtlmq_runtime_layout_state *layout;
	const struct rtlmq_runtime_config *cfg;
	struct rtlmq_irq_allocation_plan plan;
	enum rtlmq_irq_mode irq_mode;
	unsigned int i;
	int ret;

retry:
	tp->transaction.irq_vectors_allocated = false;
	tp->transaction.irq_requested_mask = 0;
	cfg = rtlmq_candidate_runtime_config(tp);
	if (!cfg)
		return -EINVAL;

	ret = rtlmq_validate_candidate_runtime_profile(tp);
	if (ret)
		return ret;

	/*
	 * A fallback can narrow the candidate vector count. Synchronize cold
	 * NAPI registration before allocating the replacement PCI IRQ shape;
	 * no NAPI instance is enabled and no queue binding is published here.
	 */
	ret = rtlmq_register_candidate_napi(tp);
	if (ret)
		return ret;

	cfg = rtlmq_candidate_runtime_config(tp);
	layout = rtlmq_candidate_runtime_layout(tp);
	for (i = 0; i < cfg->num_vectors; i++) {
		const struct rtlmq_vector_layout *vector_layout;

		vector_layout = rtlmq_runtime_layout_vector(cfg, layout, i);
		if (!vector_layout ||
		    !rtlmq_vector_layout_published(vector_layout))
			continue;
		if (!tp->vectors[i].napi_added)
			return -EINVAL;
	}
	ret = rtlmq_build_irq_allocation_plan(tp, cfg, layout, &plan);
	if (ret)
		return ret;
	if (forced_irq_mode != RTLMQ_IRQ_MODE_NONE) {
		if (plan.min_vectors != RTLMQ_BOOT_VECTORS ||
		    plan.max_vectors != RTLMQ_BOOT_VECTORS ||
		    !rtlmq_chip_irq_mode_admitted(tp, forced_irq_mode)) {
			return -EINVAL;
		}

		plan.irq_mode = forced_irq_mode;
		plan.flags = rtlmq_irq_mode_flags(forced_irq_mode);
	}

	rtlmq_mask_and_ack_intrs(tp);

	ret = pci_alloc_irq_vectors(tp->pdev, plan.min_vectors, plan.max_vectors,
				    plan.flags);
	if (ret < 0) {
		enum rtlmq_irq_mode next_mode;

		if (rtlmq_prepare_open_irq_fallback(tp, plan.irq_mode, ret,
						    &next_mode)) {
			forced_irq_mode = next_mode;
			goto retry;
		}

		return ret;
	}
	tp->transaction.irq_vectors_allocated = true;

	if (ret != plan.max_vectors) {
		netdev_err(tp->ndev,
			   "open_irqs allocated unexpected vector count %d plan_max=%u\n",
			   ret, plan.max_vectors);
		return -EINVAL;
	}

	irq_mode = rtlmq_detect_irq_mode(tp->pdev);
	if (irq_mode != plan.irq_mode) {
		netdev_err(tp->ndev,
			   "open_irqs allocated unexpected irq mode %s plan_mode=%s\n",
			   rtlmq_irq_mode_name(irq_mode),
			   rtlmq_irq_mode_name(plan.irq_mode));
		return -EINVAL;
	}

	/*
	 * Record the allocated IRQ mode in the candidate only. Do not overwrite
	 * requested configuration with a PCI allocation result.
	 */
	if (rtlmq_candidate_runtime_config_owned_mut(tp))
		rtlmq_candidate_runtime_config_owned_mut(tp)->irq_mode = irq_mode;
	forced_irq_mode = RTLMQ_IRQ_MODE_NONE;

	return 0;
}

int rtlmq_open_irqs(struct rtlmq_private *tp)
{
	const struct rtlmq_runtime_layout_state *layout;
	const struct rtlmq_runtime_config *cfg;
	unsigned int i;
	int ret;

	cfg = rtlmq_candidate_runtime_config(tp);
	layout = rtlmq_candidate_runtime_layout(tp);
	if (!cfg || !layout || !layout->valid ||
	    !tp->transaction.irq_vectors_allocated)
		return -EINVAL;

	ret = rtlmq_program_candidate_irq_mode(tp);
	if (ret) {
		netdev_err(tp->ndev,
			   "open_irqs failed to program candidate irq mode: %d\n",
			   ret);
		return ret;
	}

	ret = rtlmq_stage_candidate_irq_routing(tp, cfg, layout);
	if (ret) {
		netdev_err(tp->ndev,
			   "open_irqs failed to stage fenced candidate vector layout: %d\n",
			   ret);
		return ret;
	}

	for (i = 0; i < cfg->num_vectors; i++) {
		const struct rtlmq_vector_layout *vector_layout;
		struct rtlmq_vector *vector = &tp->vectors[i];

		vector_layout = rtlmq_runtime_layout_vector(cfg, layout, i);
		if (!vector_layout ||
		    !rtlmq_vector_layout_published(vector_layout))
			continue;

		ret = rtlmq_request_vector_irq(vector);
		if (ret)
			return ret;
	}

	/*
	 * Prepare IRQ/NAPI resources only. Leave queue->NAPI and ring->vector
	 * mappings unset and service disabled. Final open installs mappings and
	 * the generation before enabling NAPI, then interrupts.
	 */
	rtlmq_mask_and_ack_intrs(tp);

	return 0;
}

static int rtlmq_program_candidate_irq_mode(struct rtlmq_private *tp)
{
	const struct rtlmq_runtime_config *cfg;
	int ret;

	if (!tp)
		return -EINVAL;

	cfg = rtlmq_candidate_runtime_config(tp);
	if (!cfg || cfg->irq_mode == RTLMQ_IRQ_MODE_NONE ||
	    !tp->transaction.irq_vectors_allocated)
		return -EINVAL;

	/*
	 * IRQ allocation and handlers are ready, but hw_init resets INT_CFG0.
	 * Program the selected mode after MAC setup, keeping device interrupts
	 * and NAPI disabled until final open.
	 */
	ret = rtlmq_set_msix_interrupt_type(tp);
	if (ret)
		return ret;

	rtlmq_mask_and_ack_intrs(tp);
	return 0;
}

static void rtlmq_withdraw_all_vector_napi_rearm(struct rtlmq_private *tp)
{
	unsigned int i;

	if (!tp || !tp->vectors)
		return;

	for (i = 0; i < tp->allocated_vectors; i++)
		rtlmq_withdraw_vector_napi_rearm(&tp->vectors[i]);
}

static bool rtlmq_disable_all_vector_napi(struct rtlmq_private *tp)
{
	bool disabled = true;
	unsigned int i;

	if (!tp || !tp->vectors)
		return true;

	for (i = 0; i < tp->allocated_vectors; i++) {
		struct rtlmq_vector *vector = &tp->vectors[i];
		int ret;

		if (!vector->napi_enabled)
			continue;
		ret = rtlmq_disable_vector_napi(vector);
		if (ret)
			disabled = false;
		WARN_ON_ONCE(ret);
	}

	return disabled;
}

int rtlmq_quiesce_irqs(struct rtlmq_private *tp)
{
	int ret = 0;

	if (!tp)
		return -EINVAL;
	if (!rtlmq_mmio_ordinary_authorized(tp) &&
	    READ_ONCE(tp->mmio_addr))
		return -EACCES;

	/*
	 * Block completion rearm before masking the device. A concurrent rearm
	 * either finishes before mask_and_ack or sees the closed gate and skips
	 * MMIO.
	 */
	rtlmq_withdraw_all_vector_napi_rearm(tp);
	if (rtlmq_mmio_ordinary_authorized(tp))
		rtlmq_mask_and_ack_intrs(tp);
	rtlmq_mark_quiesce_phase(tp, RTLMQ_QUIESCE_IRQ_SOURCE_REVOKED);

	if (rtlmq_disable_all_vector_napi(tp))
		rtlmq_mark_quiesce_phase(tp, RTLMQ_QUIESCE_NAPI_DISABLED);
	else
		ret = -EIO;
	return ret;
}

int rtlmq_quiesce_irqs_frozen(struct rtlmq_private *tp)
{
	int ret;

	if (!tp)
		return -EINVAL;

	/*
	 * Frozen PCI or terminal failure cannot use normal MMIO mask/ACK.
	 * Retire host callbacks without asserting that outstanding DMA has
	 * drained. Ring storage remains separately owned.
	 */
	rtlmq_withdraw_all_vector_napi_rearm(tp);
	rtlmq_mark_quiesce_phase(tp, RTLMQ_QUIESCE_IRQ_SOURCE_REVOKED);
	if (!rtlmq_disable_all_vector_napi(tp))
		return -EIO;
	rtlmq_mark_quiesce_phase(tp, RTLMQ_QUIESCE_NAPI_DISABLED);
	ret = rtlmq_release_irqs(tp);
	if (ret)
		return ret;
	rtlmq_clear_unpublished_vector_queue_maps(tp);
	return 0;
}

int rtlmq_release_irqs(struct rtlmq_private *tp)
{
	unsigned int i;

	if (!tp)
		return -EINVAL;
	if (!rtlmq_irq_release_allowed(tp))
		return -EBUSY;

	rtlmq_clear_pending_tx_doorbells(tp);

	if (tp->vectors) {
		for (i = 0; i < tp->allocated_vectors; i++) {
			struct rtlmq_vector *vector = &tp->vectors[i];

			if (!vector->irq_requested)
				continue;

			rtlmq_free_vector_irq(vector);
		}

		for (i = tp->allocated_vectors; i-- > 0;)
			rtlmq_delete_vector_napi(&tp->vectors[i]);
	}
	rtlmq_mark_quiesce_phase(tp,
				 RTLMQ_QUIESCE_IRQ_HANDLERS_RELEASED);

	if ((tp->active_irq_vectors_allocated ||
	     tp->transaction.irq_vectors_allocated) && tp->pdev) {
		if (WARN_ON_ONCE(!rtlmq_vector_release_allowed(tp)))
			return -EBUSY;
		pci_free_irq_vectors(tp->pdev);
	}
	tp->active_irq_vectors_allocated = false;
	tp->transaction.irq_vectors_allocated = false;
	tp->transaction.napi_registered_mask = 0;
	tp->transaction.irq_requested_mask = 0;

	if (rtlmq_candidate_runtime_config_owned_mut(tp))
		rtlmq_candidate_runtime_config_owned_mut(tp)->irq_mode =
			RTLMQ_IRQ_MODE_NONE;

	return 0;
}
