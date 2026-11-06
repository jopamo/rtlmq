// SPDX-License-Identifier: GPL-2.0-only

#include <linux/etherdevice.h>
#include <linux/err.h>
#include <linux/mm.h>
#include <linux/moduleparam.h>
#include <linux/overflow.h>
#include <linux/rcupdate.h>
#include <linux/skbuff.h>
#include <linux/slab.h>
#include <net/page_pool/helpers.h>
#include <net/page_pool/types.h>
#include <net/xdp_sock_drv.h>

#include "rtlmq_runtime.h"
#include "rtlmq_lifecycle.h"
#include "rtlmq_hw.h"
#include "rtlmq_runtime_accessors.h"
#include "rtlmq.h"
#include "rtlmq_ring.h"
#include "rtlmq_rx.h"
#include "rtlmq_xdp.h"

/* Frame validation and allocation policy run before ring publication. */

static bool rtlmq_rx_large_buffers;
module_param_named(rx_large_buffers, rtlmq_rx_large_buffers, bool, 0444);
MODULE_PARM_DESC(rx_large_buffers,
		 "[policy] ordinary-SKB large RX buffers, default off");

bool rtlmq_rx_large_buffers_for(const struct bpf_prog *prog,
				const struct xsk_buff_pool *pool)
{
	return rtlmq_rx_large_buffers && !prog && !pool;
}

static enum dma_data_direction
rtlmq_rx_pool_dma_dir(const struct bpf_prog *admitted_xdp_prog)
{
	/*
	 * An admitted XDP program can return XDP_TX or XDP_REDIRECT and send
	 * the received page back toward a device, so that pool keeps
	 * bidirectional DMA. Ordinary skb RX only consumes device-written data
	 * and uses DMA_FROM_DEVICE. This is a ring build input: the runtime
	 * rebuild owns changing it when the admitted program changes.
	 */
	return admitted_xdp_prog ? DMA_BIDIRECTIONAL : DMA_FROM_DEVICE;
}

bool rtlmq_rx_xdp_requires_rebuild(const struct rtlmq_private *tp,
				   const struct bpf_prog *prog)
{
	enum dma_data_direction wanted_dma_dir = rtlmq_rx_pool_dma_dir(prog);
	unsigned int i;

	lockdep_assert_held(&tp->control_lock);

	for (i = 0; i < rtlmq_published_num_rx_queues(tp); i++) {
		const struct rtlmq_rx_ring *ring = &tp->rx_rings[i];
		struct rtlmq_rx_geometry geometry;
		u32 span;

		/* UMEM geometry is owned by the queue-local XSK replacement. */
		if (rtlmq_rx_ring_uses_xsk_pool(ring))
			continue;

		if (ring->page_pool &&
		    ring->page_pool->p.dma_dir != wanted_dma_dir)
			return true;

		if (!rtlmq_rx_large_buffers)
			continue;

		if (rtlmq_rx_geometry_normalize_pages(tp->ndev->mtu,
						      rtlmq_rx_large_buffers_for(prog, NULL),
						      &geometry))
			return true;
		span = rtlmq_rx_geometry_page_span(&geometry);
		if (ring->alloc_size != geometry.allocation_size ||
		    ring->buffer_len != span)
			return true;
	}
	return false;
}

void rtlmq_snapshot_published_rx_geometry_locked(const struct rtlmq_private *tp, u16 queue_index,
						 struct rtlmq_rx_published_geometry *snapshot)
{
	const struct rtlmq_runtime_config *cfg;
	const struct rtlmq_rx_ring_cold *cold;
	const struct rtlmq_rx_ring *ring;
	bool valid;

	if (!snapshot)
		return;
	memset(snapshot, 0, sizeof(*snapshot));
	if (!tp)
		return;

	lockdep_assert_held(&tp->control_lock);
	cfg = rtlmq_published_runtime_config(tp);
	if (!cfg || queue_index >= cfg->num_rx_queues)
		return;

	snapshot->published = true;
	if (!tp->rx_rings)
		return;

	ring = &tp->rx_rings[queue_index];
	cold = ring->cold;
	snapshot->allocation_size = ring->alloc_size;
	snapshot->dma_span = ring->buffer_len;
	snapshot->ring_count = ring->ring_count;
	valid = cold && ring->priv == tp && ring->queue_index == queue_index &&
		ring->ring_count == cfg->rx_ring_count &&
		ring->alloc_size && ring->buffer_len &&
		ring->buffer_len <= RTLMQ_RX_DESC_LEN_MASK;

	if (ring->page_pool && (!cold || !cold->xsk_pool)) {
		const struct page_pool_params_fast *params = &ring->page_pool->p;

		snapshot->memory_model = RTLMQ_RX_MEMORY_MODEL_PAGE_POOL;
		snapshot->page_order = params->order;
		snapshot->page_pool_size = params->pool_size;
		snapshot->page_pool_dma_span = params->max_len;
		valid = valid && cold && !cold->xsk_pool &&
			params->order <= get_order(RTLMQ_RX_ALLOC_SIZE) &&
			is_power_of_2(ring->alloc_size) &&
			ring->alloc_size <= RTLMQ_RX_ALLOC_SIZE &&
			params->order == get_order(ring->alloc_size) &&
			ring->ring_count == params->pool_size &&
			ring->buffer_len == params->max_len &&
			params->offset == RTLMQ_RX_HEADROOM &&
			ring->page_pool->slow.netdev == tp->ndev &&
			ring->page_pool->slow.queue_idx == queue_index;
	} else if (!ring->page_pool && cold && cold->xsk_pool) {
		snapshot->memory_model = RTLMQ_RX_MEMORY_MODEL_XSK_POOL;
	} else {
		valid = false;
	}

	snapshot->valid = valid;
}

static u32 rtlmq_rx_geometry_max_mtu(void)
{
	return RTLMQ_RX_MAX_MTU;
}

int rtlmq_rx_geometry_normalize_pages(u32 mtu, bool large_buffers,
				      struct rtlmq_rx_geometry *geometry)
{
	u32 allocation_size;
	size_t skb_truesize;
	u32 skb_data_size;
	u32 packet_bytes;
	u32 required;
	u32 dma_len;
	u32 max_mtu;

	if (!geometry)
		return -EINVAL;

	memset(geometry, 0, sizeof(*geometry));
	max_mtu = rtlmq_rx_geometry_max_mtu();
	if (mtu < ETH_MIN_MTU || mtu > max_mtu)
		return -EINVAL;

	if (check_add_overflow(mtu, (u32)ETH_HLEN, &packet_bytes) ||
	    check_add_overflow(packet_bytes, (u32)VLAN_HLEN, &packet_bytes) ||
	    check_add_overflow(packet_bytes, (u32)ETH_FCS_LEN, &dma_len))
		return -EOVERFLOW;

	if (packet_bytes < ETH_ZLEN)
		return -EINVAL;

	if (!dma_len || dma_len > RTLMQ_RX_HW_FRAME_SIZE)
		return -EINVAL;

	/* Aggregate capacity and one descriptor's storage are separate limits. */
	required = rtlmq_rx_page_required_size(min_t(u32, dma_len, RTLMQ_RX_DMA_LEN));
	if (!required || required > RTLMQ_RX_ALLOC_SIZE)
		return -EINVAL;

	/* Default to order-0 fragments; ordinary SKB RX may opt into large pages. */
	allocation_size =
		min_t(u32, roundup_pow_of_two(required),
		      large_buffers ? RTLMQ_RX_ALLOC_SIZE : PAGE_SIZE);
	if (!allocation_size || allocation_size > RTLMQ_RX_ALLOC_SIZE)
		return -EINVAL;

	if (allocation_size < RTLMQ_RX_TAILROOM)
		return -EINVAL;
	skb_data_size = allocation_size - RTLMQ_RX_TAILROOM;
	if (check_add_overflow((size_t)skb_data_size,
			       (size_t)SKB_DATA_ALIGN(sizeof(struct sk_buff)),
			       &skb_truesize) ||
	    check_add_overflow(skb_truesize,
			       (size_t)SKB_DATA_ALIGN(sizeof(struct skb_shared_info)),
			       &skb_truesize) ||
	    skb_truesize > U32_MAX)
		return -EOVERFLOW;

	geometry->allocation_size = allocation_size;
	geometry->dma_writable_len = dma_len;
	geometry->xdp_headroom = RTLMQ_RX_HEADROOM;
	geometry->xsk_headroom = XDP_PACKET_HEADROOM;

	return 0;
}

int rtlmq_rx_geometry_normalize(u32 mtu, struct rtlmq_rx_geometry *geometry)
{
	return rtlmq_rx_geometry_normalize_pages(mtu, false, geometry);
}

u32 rtlmq_rx_dma_len_for_mtu(u32 mtu)
{
	struct rtlmq_rx_geometry geometry;

	return rtlmq_rx_geometry_normalize(mtu, &geometry) ?
		       0 :
		       geometry.dma_writable_len;
}

u32 rtlmq_max_supported_mtu(void)
{
	return rtlmq_rx_geometry_max_mtu();
}

static u32 rtlmq_rx_ring_flags(netdev_features_t features)
{
	u32 flags = rtlmq_rx_fast_flags_from_features(features);

	/* The load option is immutable for this module's lifetime. */
	if (rtlmq_rx_copy_recycle_enabled())
		flags |= RTLMQ_RX_FAST_COPY_RECYCLE;
	return flags;
}

void rtlmq_refresh_rx_ring_fast_state(struct rtlmq_rx_ring *ring)
{
	netdev_features_t features = ring && ring->ndev ?
				     READ_ONCE(ring->ndev->features) : 0;
	u32 opts1_base;

	if (!ring)
		return;

	opts1_base = RTLMQ_DESC_OWN | ring->buffer_len;
	WRITE_ONCE(ring->rx_fast_flags, rtlmq_rx_ring_flags(features));
	ring->rx_refill_opts1 = cpu_to_le32(opts1_base);
	ring->rx_refill_opts1_eor =
		cpu_to_le32(opts1_base | RTLMQ_DESC_RING_END);
	ring->rx_refill_opts2 = cpu_to_le32(0);
	ring->rx_refill_clear_size =
		min_t(u16, ring->profile->clear_size,
		      ring->profile->addr_offset);
}

void rtlmq_sync_rx_fast_features(struct rtlmq_private *tp,
				 netdev_features_t features)
{
	u32 flags = rtlmq_rx_ring_flags(features);
	unsigned int i;

	if (!tp || !tp->rx_rings)
		return;

	for (i = 0; i < rtlmq_owned_rx_ring_slots(tp); i++)
		WRITE_ONCE(tp->rx_rings[i].rx_fast_flags, flags);
}

static struct bpf_prog *
rtlmq_candidate_xdp_prog_for_runtime_build(const struct rtlmq_private *tp)
{
	/*
	 * Use the candidate's retained XDP reference for ring setup before OPEN,
	 * not the independently mutable requested program.
	 */
	return rtlmq_candidate_xdp_prog(tp);
}

enum rtlmq_rx_ring_resource_owner {
	RTLMQ_RX_RING_RESOURCE_ACTIVE,
	RTLMQ_RX_RING_RESOURCE_RUNTIME_CANDIDATE,
	RTLMQ_RX_RING_RESOURCE_XSK_PRIVATE,
	RTLMQ_RX_RING_RESOURCE_LIVE_RETIRED,
};

struct rtlmq_live_rx_ring_set {
	struct rtlmq_rx_ring rings[RTLMQ_IMPL_MAX_RX_QUEUES];
	u64 owned_mask;
};

struct rtlmq_rx_ring_replacement {
	struct rtlmq_rx_ring *candidate;
	struct rtlmq_rx_ring retired;
	u16 queue_index;
	bool aligned;
	bool committed;
};

static struct rtlmq_vector *
rtlmq_rx_ring_consumer_vector_for_layout(struct rtlmq_rx_ring *ring,
					 const struct rtlmq_runtime_config *cfg,
					 const struct rtlmq_runtime_layout_state *layout)
{
	struct rtlmq_private *tp = ring->priv;
	u16 vector_index;

	if (!cfg || !layout || !tp->vectors ||
	    ring->queue_index >= cfg->num_rx_queues)
		return ERR_PTR(-EINVAL);

	vector_index = rtlmq_runtime_layout_rx_vector_index(cfg, layout, ring->queue_index);
	if (vector_index >= cfg->num_vectors ||
	    !tp->vectors[vector_index].napi_added)
		return ERR_PTR(-EINVAL);

	return &tp->vectors[vector_index];
}

static struct rtlmq_vector *
rtlmq_candidate_rx_ring_consumer_vector(struct rtlmq_rx_ring *ring)
{
	const struct rtlmq_runtime_layout_state *layout;
	const struct rtlmq_runtime_config *cfg;
	struct rtlmq_private *tp = ring->priv;

	cfg = rtlmq_candidate_runtime_config(tp);
	layout = rtlmq_candidate_runtime_layout(tp);
	return rtlmq_rx_ring_consumer_vector_for_layout(ring, cfg, layout);
}

static struct rtlmq_vector *
rtlmq_active_rx_ring_consumer_vector(struct rtlmq_rx_ring *ring)
{
	const struct rtlmq_runtime_layout_state *layout;
	const struct rtlmq_runtime_config *cfg;
	struct rtlmq_private *tp = ring->priv;

	cfg = rtlmq_active_runtime_config(tp);
	layout = rtlmq_active_runtime_layout(tp);
	return rtlmq_rx_ring_consumer_vector_for_layout(ring, cfg, layout);
}

static int rtlmq_create_rx_page_pool(struct rtlmq_rx_ring *ring,
				     struct rtlmq_vector *vector,
				     const struct bpf_prog *xdp_prog)
{
	struct page_pool_params pp_params = { 0 };
	struct rtlmq_private *tp = ring->priv;
	struct device *dev = &tp->pdev->dev;
	struct page_pool *page_pool;

	/*
	 * Set netdev/queue identity at creation, but attach NAPI only at commit.
	 * A private candidate may be discarded while old NAPI still runs;
	 * page_pool_destroy() requires an attached NAPI instance to be stopped.
	 */
	if (!vector)
		return -EINVAL;

	pp_params.order = get_order(ring->alloc_size);
	pp_params.pool_size = ring->ring_count;
	pp_params.nid = rtlmq_ring_preferred_nid(tp);
	pp_params.dev = dev;
	pp_params.dma_dir = rtlmq_rx_pool_dma_dir(xdp_prog);
	pp_params.max_len = ring->buffer_len;
	pp_params.offset = RTLMQ_RX_HEADROOM;
	pp_params.netdev = tp->ndev;
	pp_params.queue_idx = ring->queue_index;
	pp_params.napi = NULL;
	/*
	 * Device synchronization is not delegated to page pool: the return
	 * path can run on a different CPU from the NAPI consumer. The refill that
	 * publishes the buffer to hardware owns the explicit sync instead.
	 */
	pp_params.flags = PP_FLAG_DMA_MAP;

	page_pool = page_pool_create(&pp_params);
	if (IS_ERR(page_pool))
		return PTR_ERR(page_pool);

	ring->page_pool = page_pool;
	return 0;
}

static void
rtlmq_commit_rx_page_pool(struct rtlmq_rx_ring *ring,
			  const struct rtlmq_runtime_layout_state *layout)
{
	struct rtlmq_private *tp = ring->priv;
	u16 vector_index = layout->rx_to_vector[ring->queue_index];

	if (ring->page_pool)
		page_pool_enable_direct_recycling(ring->page_pool, &tp->vectors[vector_index].napi);
}

static int rtlmq_register_rx_xdp(struct rtlmq_rx_ring *ring,
				 struct rtlmq_vector *vector)
{
	struct rtlmq_rx_ring_cold *cold = ring->cold;
	struct rtlmq_private *tp = ring->priv;
	enum xdp_mem_type mem_type;
	void *allocator;
	int ret;

	if (!cold)
		return -EINVAL;

	if (xdp_rxq_info_is_reg(&cold->xdp_rxq))
		return 0;

	if (!vector)
		return -EINVAL;

	ret = __xdp_rxq_info_reg(&cold->xdp_rxq, tp->ndev, ring->queue_index,
				 vector->napi.napi_id,
				 rtlmq_rx_ring_uses_xsk_pool(ring) ? 0 : ring->alloc_size);
	if (ret < 0)
		return ret;

	if (rtlmq_rx_ring_uses_xsk_pool(ring)) {
		mem_type = MEM_TYPE_XSK_BUFF_POOL;
		allocator = NULL;
	} else {
		mem_type = MEM_TYPE_PAGE_POOL;
		allocator = ring->page_pool;
	}

	ret = xdp_rxq_info_reg_mem_model(&cold->xdp_rxq, mem_type, allocator);
	if (ret < 0) {
		xdp_rxq_info_unreg(&cold->xdp_rxq);
		return ret;
	}

	return 0;
}

static void rtlmq_unregister_rx_xdp(struct rtlmq_rx_ring *ring)
{
	struct rtlmq_rx_ring_cold *cold = ring->cold;

	if (!cold || !xdp_rxq_info_is_reg(&cold->xdp_rxq))
		return;

	xdp_rxq_info_unreg_mem_model(&cold->xdp_rxq);
	xdp_rxq_info_unreg(&cold->xdp_rxq);
}

bool rtlmq_rx_ring_xdp_mem_model_matches(const struct rtlmq_rx_ring *ring,
					 const struct net_device *ndev,
					 u16 queue_index)
{
	const struct rtlmq_rx_ring_cold *cold;
	const struct xdp_mem_info *mem;

	if (!ring || !ndev || !ring->cold)
		return false;

	cold = ring->cold;
	if (!xdp_rxq_info_is_reg((struct xdp_rxq_info *)&cold->xdp_rxq) ||
	    cold->xdp_rxq.dev != ndev ||
	    cold->xdp_rxq.queue_index != queue_index)
		return false;

	mem = &cold->xdp_rxq.mem;
	if (cold->xsk_pool)
		return mem->type == MEM_TYPE_XSK_BUFF_POOL && !mem->id;
	if (!ring->page_pool)
		return false;

	return mem->type == MEM_TYPE_PAGE_POOL && mem->id &&
	       mem->id == ring->page_pool->xdp_mem_id;
}

static void rtlmq_release_rx_buffers(struct rtlmq_rx_ring *ring)
{
	struct rtlmq_rx_ring_cold *cold = ring->cold;
	unsigned int i;

	if ((!cold || (!ring->page_pool && !cold->xsk_pool)) || !ring->buffers)
		return;

	rtlmq_release_rx_multibuffer(ring);
	rtlmq_xsk_release_rx_ring_state(ring);
	for (i = 0; i < ring->ring_count; i++)
		rtlmq_release_rx_buffer(ring, &ring->buffers[i], false);
	ring->rx_posted = 0;
}

static int rtlmq_populate_rx_buffers(struct rtlmq_rx_ring *ring)
{
	unsigned int remaining;
	unsigned int i;
	int ret;

	if (rtlmq_rx_ring_uses_xsk_pool(ring)) {
		enum rtlmq_rx_refill_failure failure;
		unsigned int posted;

		/*
		 * An empty AF_XDP fill ring is normal at bind time. Build and
		 * commit the candidate ring with zero or partial RX
		 * descriptors, then let XDP_WAKEUP_RX/NAPI populate descriptors
		 * after userspace publishes fill entries. DMA or descriptor
		 * setup failures still fail candidate construction.
		 */

		posted = rtlmq_refill_rx_ring_generic(ring, ring->ring_count, &failure);
		if (posted == ring->ring_count ||
		    failure == RTLMQ_RX_REFILL_ALLOC_FAILED)
			return 0;

		rtlmq_release_rx_buffers(ring);
		if (ring->desc)
			memset(ring->desc, 0, ring->desc_alloc_size);
		return -EIO;
	}

	for (i = 0; i < ring->ring_count; i++) {
		struct rtlmq_rx_buffer *buffer = &ring->buffers[i];

		if (buffer->opaque)
			continue;

		ret = rtlmq_alloc_rx_buffer(ring, buffer, GFP_KERNEL);
		if (ret) {
			if (ring->cold)
				ring->cold->stats.alloc_fail++;
			goto err_release_rx_buffers;
		}
	}

	ring->next_to_alloc = 0;
	for (remaining = ring->ring_count; remaining; ) {
		enum rtlmq_rx_refill_failure failure;
		unsigned int batch = min_t(unsigned int, remaining,
					   ring->refill_batch);
		unsigned int posted;

		posted = rtlmq_refill_rx_ring_generic(ring, batch, &failure);
		if (posted != batch) {
			ret = failure == RTLMQ_RX_REFILL_ALLOC_FAILED ?
				      -ENOMEM : -EIO;
			goto err_release_rx_buffers;
		}
		remaining -= batch;
	}

	return 0;

err_release_rx_buffers:
	rtlmq_release_rx_buffers(ring);
	if (ring->desc)
		memset(ring->desc, 0, ring->desc_alloc_size);

	return ret;
}

static bool rtlmq_rx_ring_storage_aliases(const struct rtlmq_rx_ring *ring,
					  const struct rtlmq_rx_ring *active)
{
	return ring == active || ring->cold == active->cold ||
	       (ring->desc &&
		(ring->desc == active->desc ||
		 (active->desc && ring->desc_dma == active->desc_dma))) ||
	       (ring->buffers && ring->buffers == active->buffers) ||
	       (ring->page_pool && ring->page_pool == active->page_pool);
}

bool rtlmq_rx_ring_storage_aliases_active(const struct rtlmq_private *tp,
					  const struct rtlmq_rx_ring *ring)
{
	unsigned int i;

	for (i = 0; tp->rx_rings && i < rtlmq_owned_rx_ring_slots(tp); i++) {
		const struct rtlmq_rx_ring *active = &tp->rx_rings[i];

		if (rtlmq_rx_ring_storage_aliases(ring, active))
			return true;
	}

	return false;
}

static bool rtlmq_xsk_private_rx_ring(const struct rtlmq_rx_ring *ring)
{
	const struct rtlmq_private *tp;

	if (!ring || !ring->priv)
		return false;

	tp = ring->priv;
	lockdep_assert_held(&tp->control_lock);

	return !rtlmq_rx_ring_storage_aliases_active(tp, ring);
}

static bool
rtlmq_rx_ring_release_allowed(const struct rtlmq_rx_ring *ring,
			      enum rtlmq_rx_ring_resource_owner owner)
{
	if (!ring || !ring->priv)
		return false;

	switch (owner) {
	case RTLMQ_RX_RING_RESOURCE_ACTIVE:
		return rtlmq_release_allowed(ring->priv);
	case RTLMQ_RX_RING_RESOURCE_RUNTIME_CANDIDATE:
		return rtlmq_candidate_discard_allowed(ring->priv);
	case RTLMQ_RX_RING_RESOURCE_XSK_PRIVATE:
		return rtlmq_xsk_private_rx_ring(ring);
	case RTLMQ_RX_RING_RESOURCE_LIVE_RETIRED:
		return rtlmq_retired_generation_release_allowed(ring->priv);
	default:
		return false;
	}
}

static bool rtlmq_destroy_rx_page_pool(struct rtlmq_rx_ring *ring,
				       enum rtlmq_rx_ring_resource_owner owner)
{
	if (!rtlmq_rx_ring_release_allowed(ring, owner))
		return false;

	rtlmq_unregister_rx_xdp(ring);

	if (ring->page_pool) {
		/*
		 * TX teardown must have returned driver-owned XDP frames.
		 * Core defers final pool cleanup until outstanding SKB/XDP
		 * references return.
		 */
		page_pool_destroy(ring->page_pool);
		ring->page_pool = NULL;
	}

	return true;
}

static bool
rtlmq_release_rx_ring_aux_memory(struct rtlmq_rx_ring *ring,
				 enum rtlmq_rx_ring_resource_owner owner)
{
	struct rtlmq_rx_ring_cold *cold;

	if (!rtlmq_rx_ring_release_allowed(ring, owner))
		return false;
	cold = ring->cold;

	if (!rtlmq_destroy_rx_page_pool(ring, owner))
		return false;

	kfree(ring->buffers);
	ring->buffers = NULL;
	ring->desc_alloc_size = 0;
	ring->alloc_size = 0;
	ring->buffer_len = 0;
	ring->rx_fast_skb_refill = false;
	rtlmq_refresh_rx_ring_fast_state(ring);
	if (cold)
		cold->xsk_pool = NULL;

	return true;
}

static void rtlmq_reset_private_rx_ring_resources(struct rtlmq_rx_ring *ring)
{
	ring->page_pool = NULL;
	ring->desc = NULL;
	ring->desc_dma = 0;
	ring->desc_alloc_size = 0;
	ring->buffers = NULL;
	ring->alloc_size = 0;
	ring->buffer_len = 0;
	ring->next_to_clean = 0;
	ring->next_to_alloc = 0;
	ring->rx_posted = 0;
	ring->rx_fast_skb_refill = false;
}

/*
 * Snapshot a paused ring before replacement. The caller transfers ownership
 * by installing its replacement before either snapshot can be released.
 * Keep cold in place: it owns the memory model and synchronization object.
 */
static void rtlmq_capture_rx_ring(struct rtlmq_rx_ring *snapshot,
				  const struct rtlmq_rx_ring *ring)
{
	/* Borrowed queue identity and descriptor format. */
	snapshot->priv = ring->priv;
	snapshot->ndev = ring->ndev;
	snapshot->napi = ring->napi;
	snapshot->profile = ring->profile;
	snapshot->queue_index = ring->queue_index;
	snapshot->vector_index = ring->vector_index;

	/* Geometry and refill policy travel with the replacement storage. */
	snapshot->rx_fast_flags = ring->rx_fast_flags;
	snapshot->rx_refill_opts1 = ring->rx_refill_opts1;
	snapshot->rx_refill_opts1_eor = ring->rx_refill_opts1_eor;
	snapshot->rx_refill_opts2 = ring->rx_refill_opts2;
	snapshot->alloc_size = ring->alloc_size;
	snapshot->buffer_len = ring->buffer_len;
	snapshot->ring_count = ring->ring_count;
	snapshot->ring_mask = ring->ring_mask;
	snapshot->refill_batch = ring->refill_batch;
	snapshot->rx_refill_clear_size = ring->rx_refill_clear_size;
	snapshot->rx_fast_skb_refill = ring->rx_fast_skb_refill;

	/* Cursors and counters have already been staged by the caller. */
	snapshot->consumer = ring->consumer;

	snapshot->desc = ring->desc;
	snapshot->desc_dma = ring->desc_dma;
	snapshot->desc_alloc_size = ring->desc_alloc_size;
	snapshot->buffers = ring->buffers;
	snapshot->page_pool = ring->page_pool;
	snapshot->cold = ring->cold;
}

/* Destination storage is retired or absent; source is a private owner. */
static void rtlmq_take_rx_ring(struct rtlmq_rx_ring *destination,
			       struct rtlmq_rx_ring *source)
{
	rtlmq_capture_rx_ring(destination, source);
	rtlmq_reset_private_rx_ring_resources(source);
	source->cold = NULL;
}

/* The caller owns any displaced descriptor allocation until it is freed. */
static void rtlmq_take_rx_descriptors(struct rtlmq_rx_ring *destination,
				      struct rtlmq_rx_ring *source)
{
	destination->desc = source->desc;
	destination->desc_dma = source->desc_dma;
	destination->desc_alloc_size = source->desc_alloc_size;
	source->desc = NULL;
	source->desc_dma = 0;
	source->desc_alloc_size = 0;
}

struct rtlmq_rx_ring *rtlmq_prepare_candidate_rx_ring(struct rtlmq_private *tp,
						      u16 queue_index)
{
	struct rtlmq_rx_ring_cold *candidate_cold;
	struct rtlmq_rx_ring *candidate;
	struct rtlmq_rx_ring *active;

	if (!tp || !tp->rx_rings ||
	    queue_index >= rtlmq_owned_rx_ring_slots(tp))
		return ERR_PTR(-EINVAL);
	if (tp->transaction.rx_rings[queue_index])
		return ERR_PTR(-EBUSY);

	active = &tp->rx_rings[queue_index];

	candidate = kmemdup(active, sizeof(*candidate), GFP_KERNEL);
	if (!candidate)
		return ERR_PTR(-ENOMEM);

	candidate_cold = kzalloc_obj(*candidate_cold, GFP_KERNEL);
	if (!candidate_cold) {
		kfree(candidate);
		return ERR_PTR(-ENOMEM);
	}
	/*
	 * Start public counters at zero. Ring retirement moves them into device
	 * history; copying them here would count them twice. Retain private
	 * observations separately from the new memory model and fragment state.
	 */
	candidate->hw_packets = 0;
	candidate->hw_bytes = 0;
	u64_stats_init(&candidate_cold->syncp);
	candidate->cold = candidate_cold;
	/*
	 * Keep copied queue identity and configuration, but clear resource
	 * handles. The private clone must not alias active DMA, buffers, or pools.
	 */
	rtlmq_reset_private_rx_ring_resources(candidate);
	tp->transaction.rx_rings[queue_index] = candidate;
	return candidate;
}

const struct rtlmq_rx_ring *
rtlmq_candidate_or_active_rx_ring(const struct rtlmq_private *tp,
				  u16 queue_index)
{
	if (!tp || !tp->rx_rings ||
	    queue_index >= rtlmq_owned_rx_ring_slots(tp))
		return NULL;
	if (tp->transaction.rx_rings[queue_index])
		return tp->transaction.rx_rings[queue_index];

	return &tp->rx_rings[queue_index];
}

bool rtlmq_active_rx_cold_heap_present(const struct rtlmq_private *tp)
{
	unsigned int i;

	if (!tp || !tp->rx_rings || !tp->rx_ring_cold)
		return false;

	/*
	 * cold points either into probe's rx_ring_cold slab or to a separately
	 * allocated replacement. Inspect the pointer rather than cache this
	 * distinction in another device-wide bitmap.
	 */
	for (i = 0; i < rtlmq_owned_rx_ring_slots(tp); i++) {
		const struct rtlmq_rx_ring_cold *cold = tp->rx_rings[i].cold;

		if (cold && cold != &tp->rx_ring_cold[i])
			return true;
	}

	return false;
}

void rtlmq_commit_candidate_rx_rings(struct rtlmq_private *tp)
{
	u16 num_rx_queues;
	unsigned int i;

	if (!tp || !tp->rx_rings || !tp->transaction.resource_generation)
		return;
	if (WARN_ON_ONCE(rtlmq_active_rx_cold_heap_present(tp) &&
			 !rtlmq_retired_generation_release_allowed(tp)))
		return;

	num_rx_queues = tp->transaction.profile.config.num_rx_queues;
	for (i = 0; i < num_rx_queues; i++) {
		struct rtlmq_rx_ring *candidate = tp->transaction.rx_rings[i];
		struct rtlmq_rx_ring_cold *old_cold;

		if (!candidate)
			continue;

		rtlmq_commit_rx_page_pool(candidate,
					  &tp->transaction.profile.layout);
		old_cold = tp->rx_rings[i].cold;
		rtlmq_take_rx_ring(&tp->rx_rings[i], candidate);
		if (tp->rx_rings[i].cold != old_cold) {
			if (old_cold && old_cold != &tp->rx_ring_cold[i])
				kfree(old_cold);
		}
		tp->transaction.rx_rings[i] = NULL;
		kfree(candidate);
	}
	tp->retired_generation_quiesced = false;
}

static bool rtlmq_release_rx_ring_resources(struct rtlmq_rx_ring *ring,
					    enum rtlmq_rx_ring_resource_owner owner);

struct rtlmq_live_rx_ring_set *rtlmq_alloc_live_rx_ring_set(void)
{
	return kzalloc(sizeof(struct rtlmq_live_rx_ring_set), GFP_KERNEL);
}

static void rtlmq_carry_live_rx_ring_stats(struct rtlmq_rx_ring *candidate,
					   const struct rtlmq_rx_ring *active)
{
	candidate->hw_packets = active->hw_packets;
	candidate->hw_bytes = active->hw_bytes;
	candidate->cold->stats = active->cold->stats;
	u64_stats_init(&candidate->cold->syncp);
}

int rtlmq_swap_live_candidate_rx_rings(struct rtlmq_private *tp,
				       struct rtlmq_live_rx_ring_set *retired)
{
	const struct rtlmq_runtime_config *cfg;
	unsigned int i;

	if (!tp || !tp->rx_rings || !retired || retired->owned_mask)
		return -EINVAL;
	if (!rtlmq_retired_generation_release_allowed(tp))
		return -EBUSY;
	cfg = rtlmq_candidate_runtime_config(tp);
	if (!cfg)
		return -EINVAL;

	for (i = 0; i < cfg->num_rx_queues; i++) {
		struct rtlmq_rx_ring *candidate =
			tp->transaction.rx_rings[i];
		struct rtlmq_rx_ring *active = &tp->rx_rings[i];

		if (!candidate || !candidate->cold || !active->cold)
			return -EINVAL;
	}

	for (i = 0; i < cfg->num_rx_queues; i++) {
		struct rtlmq_rx_ring *candidate =
			tp->transaction.rx_rings[i];
		struct rtlmq_rx_ring *active = &tp->rx_rings[i];

		rtlmq_carry_live_rx_ring_stats(candidate, active);
		rtlmq_commit_rx_page_pool(candidate,
					  &tp->transaction.profile.layout);
		rtlmq_capture_rx_ring(&retired->rings[i], active);
		retired->owned_mask |= BIT_ULL(i);
		rtlmq_take_rx_ring(active, candidate);
		tp->transaction.rx_rings[i] = NULL;
		kfree(candidate);
	}
	tp->transaction.rx_dma_ring_mask = 0;

	return 0;
}

void rtlmq_rollback_live_rx_rings(struct rtlmq_private *tp,
				  struct rtlmq_live_rx_ring_set *retired)
{
	unsigned int i;

	if (!tp || !tp->rx_rings || !retired ||
	    !rtlmq_retired_generation_release_allowed(tp))
		return;

	for (i = 0; i < RTLMQ_IMPL_MAX_RX_QUEUES; i++) {
		struct rtlmq_rx_ring replacement;

		if (!(retired->owned_mask & BIT_ULL(i)))
			continue;
		rtlmq_capture_rx_ring(&replacement, &tp->rx_rings[i]);
		rtlmq_take_rx_ring(&tp->rx_rings[i], &retired->rings[i]);
		rtlmq_take_rx_ring(&retired->rings[i], &replacement);
	}
}

bool rtlmq_release_live_rx_ring_set(struct rtlmq_private *tp,
				    struct rtlmq_live_rx_ring_set *retired)
{
	unsigned int i;

	if (!retired)
		return true;
	if (!retired->owned_mask) {
		kfree(retired);
		return true;
	}
	if (!tp || !rtlmq_retired_generation_release_allowed(tp))
		return false;

	for (i = 0; i < RTLMQ_IMPL_MAX_RX_QUEUES; i++) {
		struct rtlmq_rx_ring *ring = &retired->rings[i];

		if (!(retired->owned_mask & BIT_ULL(i)))
			continue;
		if (!rtlmq_release_rx_ring_resources(ring, RTLMQ_RX_RING_RESOURCE_LIVE_RETIRED))
			continue;
		if (ring->cold && tp->rx_ring_cold &&
		    ring->cold != &tp->rx_ring_cold[i])
			kfree(ring->cold);
		retired->owned_mask &= ~BIT_ULL(i);
	}

	if (retired->owned_mask)
		return false;
	kfree(retired);
	return true;
}

void rtlmq_discard_candidate_rx_rings(struct rtlmq_private *tp)
{
	unsigned int i;

	if (!tp || !rtlmq_candidate_discard_allowed(tp))
		return;

	for (i = 0; i < RTLMQ_IMPL_MAX_RX_QUEUES; i++) {
		struct rtlmq_rx_ring *candidate = tp->transaction.rx_rings[i];

		if (!candidate)
			continue;

		if (!rtlmq_release_rx_ring_resources(candidate,
						     RTLMQ_RX_RING_RESOURCE_RUNTIME_CANDIDATE))
			continue;
		tp->transaction.rx_dma_ring_mask &= ~BIT_ULL(i);
		if (tp->rx_rings &&
		    candidate->cold != tp->rx_rings[i].cold)
			kfree(candidate->cold);
		tp->transaction.rx_rings[i] = NULL;
		kfree(candidate);
	}

	tp->transaction.rx_dma_ring_mask = 0;
}

void rtlmq_release_active_rx_cold(struct rtlmq_private *tp)
{
	unsigned int i;

	if (!tp || !tp->rx_rings || !tp->rx_ring_cold ||
	    !rtlmq_release_allowed(tp))
		return;

	for (i = 0; i < rtlmq_owned_rx_ring_slots(tp); i++) {
		if (!tp->rx_rings[i].cold ||
		    tp->rx_rings[i].cold == &tp->rx_ring_cold[i])
			continue;

		kfree(tp->rx_rings[i].cold);
		tp->rx_rings[i].cold = &tp->rx_ring_cold[i];
	}
}

static int rtlmq_alloc_rx_ring_owned(struct rtlmq_rx_ring *ring,
				     struct rtlmq_vector *vector,
				     struct bpf_prog *xdp_prog,
				     enum rtlmq_rx_ring_resource_owner owner)
{
	struct rtlmq_private *tp = ring->priv;
	struct device *dev = &tp->pdev->dev;
	struct rtlmq_rx_ring_cold *cold = ring->cold;
	struct rtlmq_rx_geometry geometry;
	int nid = rtlmq_ring_preferred_nid(tp);
	int ret;

	if (owner == RTLMQ_RX_RING_RESOURCE_RUNTIME_CANDIDATE) {
		if (!rtlmq_candidate_discard_allowed(tp))
			return -EBUSY;
	} else if (owner != RTLMQ_RX_RING_RESOURCE_XSK_PRIVATE ||
		   READ_ONCE(tp->state) != RTLMQ_STATE_OPEN ||
		   !rtlmq_xsk_private_rx_ring(ring)) {
		return -EBUSY;
	}

	if (!cold)
		return -EINVAL;

	if (ring->desc || ring->buffers || ring->page_pool)
		return -EBUSY;

	ret = rtlmq_rx_geometry_normalize_pages(
		tp->ndev->mtu,
		rtlmq_rx_large_buffers_for(xdp_prog, cold->xsk_pool),
		&geometry);
	if (ret)
		return ret;

	ring->buffer_len = rtlmq_rx_ring_uses_xsk_pool(ring) ?
			   xsk_pool_get_rx_frame_size(cold->xsk_pool) : 0;
	if (rtlmq_rx_ring_uses_xsk_pool(ring) &&
	    !rtlmq_rx_xsk_frame_size_valid_pool_geometry(tp, cold->xsk_pool,
							 ring->buffer_len, &geometry))
		return -EOPNOTSUPP;
	ring->alloc_size = ring->buffer_len;
	if (!rtlmq_rx_ring_uses_xsk_pool(ring)) {
		ring->alloc_size = geometry.allocation_size;
		ring->buffer_len = rtlmq_rx_geometry_page_span(&geometry);
	}
	if (!ring->buffer_len)
		return -EINVAL;

	ring->rx_fast_skb_refill = false;

	ring->buffers = kcalloc_node(ring->ring_count, sizeof(*ring->buffers),
				     GFP_KERNEL, nid);
	if (!ring->buffers)
		return -ENOMEM;

	if (!rtlmq_rx_ring_uses_xsk_pool(ring)) {
		ret = rtlmq_create_rx_page_pool(ring, vector, xdp_prog);
		if (ret) {
			kfree(ring->buffers);
			ring->buffers = NULL;
			return ret;
		}
	}

	ret = rtlmq_register_rx_xdp(ring, vector);
	if (ret)
		goto err_release_aux;

	rtlmq_init_rx_desc_profile(ring);
	ring->rx_fast_skb_refill = !rtlmq_rx_ring_uses_xsk_pool(ring) &&
				   !xdp_prog &&
				   rtlmq_rx_ring_uses_single_descriptor(ring);
	ret = rtlmq_rx_desc_ring_size(ring->ring_count,
				      ring->profile->fmt,
				      &ring->desc_alloc_size);
	if (ret)
		goto err_release_aux;

	ring->desc = dma_alloc_coherent(dev, ring->desc_alloc_size,
					&ring->desc_dma, GFP_KERNEL);
	if (!ring->desc) {
		ret = -ENOMEM;
		goto err_release_aux;
	}
	if (!rtlmq_dma_range_fits_coherent(dev, ring->desc_dma,
					   ring->desc_alloc_size)) {
		dma_free_coherent(dev, ring->desc_alloc_size,
				  ring->desc, ring->desc_dma);
		ring->desc = NULL;
		ring->desc_dma = 0;
		ret = -EIO;
		goto err_release_aux;
	}
	if (owner == RTLMQ_RX_RING_RESOURCE_RUNTIME_CANDIDATE)
		tp->transaction.rx_dma_ring_mask |= BIT_ULL(ring->queue_index);

	return 0;

err_release_aux:
	(void)rtlmq_release_rx_ring_aux_memory(ring, owner);
	return ret;
}

int rtlmq_alloc_rx_ring(struct rtlmq_rx_ring *ring)
{
	struct rtlmq_vector *vector;

	if (!ring)
		return -EINVAL;

	vector = rtlmq_candidate_rx_ring_consumer_vector(ring);
	if (IS_ERR(vector))
		return PTR_ERR(vector);

	return rtlmq_alloc_rx_ring_owned(ring, vector,
					 rtlmq_candidate_xdp_prog_for_runtime_build(ring->priv),
					 RTLMQ_RX_RING_RESOURCE_RUNTIME_CANDIDATE);
}

void rtlmq_reset_rx_ring_posting(struct rtlmq_rx_ring *ring)
{
	memset(ring->desc, 0, ring->desc_alloc_size);
	ring->next_to_alloc = 0;
	ring->next_to_clean = 0;
	ring->rx_posted = 0;
}

int rtlmq_init_rx_ring(struct rtlmq_rx_ring *ring)
{
	int ret;

	rtlmq_reset_rx_ring_posting(ring);

	ret = rtlmq_populate_rx_buffers(ring);
	if (ret)
		return ret;

	return 0;
}

static int rtlmq_validate_rx_ring_storage(const struct rtlmq_rx_ring *ring,
					  bool require_pool)
{
	size_t desc_bytes;

	if (!ring || !ring->priv || !ring->priv->pdev || !ring->cold ||
	    !ring->desc || !ring->desc_alloc_size ||
	    !ring->buffers || !ring->profile || !ring->ring_count ||
	    (require_pool && !ring->page_pool && !ring->cold->xsk_pool) ||
	    !is_power_of_2(ring->ring_count) ||
	    ring->ring_mask != ring->ring_count - 1 ||
	    !ring->buffer_len ||
	    ring->buffer_len > RTLMQ_RX_DESC_LEN_MASK)
		return -EINVAL;

	if (check_mul_overflow((size_t)ring->ring_count,
			       (size_t)ring->profile->stride, &desc_bytes) ||
	    desc_bytes > ring->desc_alloc_size)
		return -EINVAL;

	return 0;
}

static int rtlmq_validate_xsk_rx_candidate(const struct rtlmq_rx_ring *ring,
					   unsigned int *posted)
{
	bool found_empty = false;
	unsigned int i;
	int ret;

	ret = rtlmq_validate_rx_ring_storage(ring, false);
	if (ret)
		return ret;

	*posted = 0;
	for (i = 0; i < ring->ring_count; i++) {
		const struct rtlmq_rx_buffer *buffer = &ring->buffers[i];

		if (!buffer->opaque) {
			if (buffer->dma_addr || buffer->hard_start)
				return -EIO;
			found_empty = true;
			continue;
		}
		if (found_empty)
			return -EIO;

		if (!rtlmq_rx_buffer_ready_for_device(buffer) ||
		    !rtlmq_dma_range_fits_device(&ring->priv->pdev->dev,
						 buffer->dma_addr,
						 ring->buffer_len))
			return -EIO;
		(*posted)++;
	}

	return 0;
}

static int rtlmq_validate_rx_ring_restart(const struct rtlmq_rx_ring *ring)
{
	unsigned int i;
	int ret;

	ret = rtlmq_validate_rx_ring_storage(ring, true);
	if (ret)
		return ret;

	for (i = 0; i < ring->ring_count; i++) {
		const struct rtlmq_rx_buffer *buffer = &ring->buffers[i];

		if (!buffer->opaque) {
			if (buffer->dma_addr || buffer->hard_start)
				return -EIO;
			continue;
		}

		if (!rtlmq_rx_buffer_ready_for_device(buffer) ||
		    !rtlmq_dma_range_fits_device(&ring->priv->pdev->dev,
						 buffer->dma_addr,
						 ring->buffer_len))
			return -EIO;
	}

	return 0;
}

static unsigned int rtlmq_compact_rx_ring_buffers(struct rtlmq_rx_ring *ring)
{
	unsigned int destination = 0;
	unsigned int source;

	for (source = 0; source < ring->ring_count; source++) {
		if (!ring->buffers[source].opaque)
			continue;

		if (destination != source) {
			ring->buffers[destination] = ring->buffers[source];
			memset(&ring->buffers[source], 0,
			       sizeof(ring->buffers[source]));
		}
		destination++;
	}

	if (destination < ring->ring_count)
		memset(&ring->buffers[destination], 0,
		       array_size(ring->ring_count - destination,
				  sizeof(*ring->buffers)));

	return destination;
}

int rtlmq_validate_active_rx_rings_for_restart_locked(struct rtlmq_private *tp)
{
	const struct rtlmq_runtime_config *cfg;
	unsigned int queue;
	int ret;

	if (!tp || !tp->rx_rings)
		return -EINVAL;
	lockdep_assert_held(&tp->control_lock);

	cfg = rtlmq_active_runtime_config(tp);
	if (!cfg || !cfg->num_rx_queues ||
	    cfg->num_rx_queues > rtlmq_owned_rx_ring_slots(tp))
		return -EINVAL;

	/*
	 * IRQ/NAPI and RX DMA are stopped, with engine-off readback complete.
	 * Validate all active RX rings before committing the target replacement;
	 * an invalid unaffected queue must not cause a partial restart.
	 */
	for (queue = 0; queue < cfg->num_rx_queues; queue++) {
		ret = rtlmq_validate_rx_ring_restart(&tp->rx_rings[queue]);
		if (ret)
			return ret;
	}

	return 0;
}

int rtlmq_restart_active_rx_rings_from_zero_locked(struct rtlmq_private *tp)
{
	const struct rtlmq_runtime_config *cfg;
	unsigned int queue;
	int ret;

	ret = rtlmq_validate_active_rx_rings_for_restart_locked(tp);
	if (ret)
		return ret;

	cfg = rtlmq_active_runtime_config(tp);

	for (queue = 0; queue < cfg->num_rx_queues; queue++) {
		struct rtlmq_rx_ring *ring = &tp->rx_rings[queue];
		unsigned int posted;
		unsigned int entry;

		/*
		 * Discard partial packets tied to the old cursor. Retain complete
		 * buffers and restart hardware/software cursors at descriptor zero.
		 */
		rtlmq_release_rx_multibuffer(ring);
		posted = rtlmq_compact_rx_ring_buffers(ring);
		memset(ring->desc, 0, ring->desc_alloc_size);
		ring->next_to_clean = 0;
		ring->next_to_alloc = 0;
		ring->rx_posted = 0;

		for (entry = 0; entry < posted; entry++) {
			rtlmq_sync_rx_buffer_for_device(ring, &ring->buffers[entry],
							ring->buffer_len);
			if (!rtlmq_publish_rx_desc(ring, entry))
				return -EIO;
		}

		ring->next_to_alloc = posted & ring->ring_mask;
		ring->rx_posted = posted;
		rtlmq_xsk_update_rx_need_wakeup(ring, ring->ring_count, posted);
	}

	return rtlmq_republish_active_rx_ring_bases_locked(tp, cfg);
}

static unsigned int
rtlmq_xsk_stopped_rx_cursor(const struct rtlmq_rx_ring *active)
{
	unsigned int scanned;
	unsigned int cursor;

	if (WARN_ON_ONCE(!active || !active->desc || !active->buffers ||
			 !active->profile || !active->ring_count))
		return 0;

	cursor = active->next_to_clean & active->ring_mask;
	/*
	 * With RX stopped, scan completed buffered descriptors from next_to_clean
	 * to the first OWN-set or empty slot. Align the candidate to that cursor;
	 * the caller later restarts all active RX rings from descriptor zero.
	 */
	dma_rmb();
	for (scanned = 0; scanned < active->ring_count; scanned++) {
		const struct rtlmq_rx_buffer *buffer = &active->buffers[cursor];
		u32 status;

		if (!buffer->opaque)
			break;
		status = le32_to_cpu(rtlmq_rx_desc_opts1(active,
							 rtlmq_rx_desc_addr(active, cursor)));
		if (!rtlmq_rx_desc_status_cpu_owned(status))
			break;
		cursor = (cursor + 1) & active->ring_mask;
	}

	return cursor;
}

static void rtlmq_reverse_rx_buffers(struct rtlmq_rx_buffer *buffers,
				     unsigned int first, unsigned int last)
{
	while (first < last) {
		struct rtlmq_rx_buffer tmp;

		last--;
		if (first >= last)
			break;
		tmp = buffers[first];
		buffers[first] = buffers[last];
		buffers[last] = tmp;
		first++;
	}
}

static void rtlmq_rotate_rx_buffers_to_cursor(struct rtlmq_rx_ring *ring,
					      unsigned int cursor)
{
	if (!cursor)
		return;

	rtlmq_reverse_rx_buffers(ring->buffers, 0, ring->ring_count);
	rtlmq_reverse_rx_buffers(ring->buffers, 0, cursor);
	rtlmq_reverse_rx_buffers(ring->buffers, cursor, ring->ring_count);
}

int rtlmq_align_xsk_rx_ring_replacement_locked(struct rtlmq_private *tp,
					       struct rtlmq_rx_ring_replacement *replacement)
{
	struct rtlmq_rx_ring *candidate;
	struct rtlmq_rx_ring *active;
	unsigned int resume_cursor;
	unsigned int posted;
	unsigned int offset;
	int ret;

	if (!tp || !tp->rx_rings || !replacement ||
	    replacement->committed || replacement->aligned ||
	    !replacement->candidate)
		return -EINVAL;
	lockdep_assert_held(&tp->control_lock);

	active = &tp->rx_rings[replacement->queue_index];
	candidate = replacement->candidate;
	ret = rtlmq_validate_xsk_rx_candidate(candidate, &posted);
	if (ret)
		return ret;
	if (candidate->ring_count != active->ring_count ||
	    candidate->ring_mask != active->ring_mask)
		return -EINVAL;

	resume_cursor = rtlmq_xsk_stopped_rx_cursor(active);
	rtlmq_rotate_rx_buffers_to_cursor(candidate, resume_cursor);
	memset(candidate->desc, 0, candidate->desc_alloc_size);
	candidate->next_to_clean = resume_cursor;
	candidate->next_to_alloc = resume_cursor;
	candidate->rx_posted = 0;

	for (offset = 0; offset < posted; offset++) {
		unsigned int entry =
			(resume_cursor + offset) & candidate->ring_mask;

		if (!rtlmq_publish_rx_desc(candidate, entry))
			return -EIO;
	}
	candidate->next_to_alloc =
		(resume_cursor + posted) & candidate->ring_mask;
	candidate->rx_posted = posted;
	rtlmq_xsk_update_rx_need_wakeup(candidate, candidate->ring_count, posted);

	replacement->aligned = true;
	return 0;
}

static void rtlmq_stop_rx_ring_owned(struct rtlmq_rx_ring *ring,
				     enum rtlmq_rx_ring_resource_owner owner)
{
	if (!rtlmq_rx_ring_release_allowed(ring, owner))
		return;

	if (!ring->desc && !ring->buffers &&
	    (!ring->cold || !ring->page_pool))
		return;

	ring->next_to_alloc = 0;
	ring->next_to_clean = 0;
	ring->rx_posted = 0;
	rtlmq_release_rx_buffers(ring);

	if (ring->desc)
		memset(ring->desc, 0, ring->desc_alloc_size);
}

void rtlmq_stop_rx_ring(struct rtlmq_rx_ring *ring)
{
	rtlmq_stop_rx_ring_owned(ring, RTLMQ_RX_RING_RESOURCE_ACTIVE);
}

static bool
rtlmq_release_rx_ring_resources(struct rtlmq_rx_ring *ring,
				enum rtlmq_rx_ring_resource_owner owner)
{
	struct rtlmq_private *tp = ring ? ring->priv : NULL;
	struct device *dev = NULL;

	if (!rtlmq_rx_ring_release_allowed(ring, owner))
		return false;

	if (tp->pdev)
		dev = &tp->pdev->dev;

	rtlmq_stop_rx_ring_owned(ring, owner);
	/*
	 * Unregister the RX memory model before freeing coherent descriptors.
	 * Page-pool core handles outstanding stack/XDP references during deferred
	 * pool destruction.
	 */
	if (!rtlmq_destroy_rx_page_pool(ring, owner))
		return false;

	if (ring->desc) {
		if (dev)
			dma_free_coherent(dev, ring->desc_alloc_size, ring->desc,
					  ring->desc_dma);
		else
			kfree(ring->desc);
		ring->desc = NULL;
		ring->desc_dma = 0;
	}

	if (!rtlmq_release_rx_ring_aux_memory(ring, owner))
		return false;

	return true;
}

void rtlmq_free_rx_ring(struct rtlmq_rx_ring *ring)
{
	struct rtlmq_private *tp = ring ? ring->priv : NULL;

	if (!tp)
		return;

	if (!rtlmq_release_rx_ring_resources(ring, RTLMQ_RX_RING_RESOURCE_ACTIVE))
		return;
}

static void rtlmq_release_xsk_private_ring(struct rtlmq_private *tp,
					   struct rtlmq_rx_ring *ring)
{
	struct rtlmq_rx_ring_cold *cold;

	if (!ring)
		return;

	cold = ring->cold;
	if (!rtlmq_release_rx_ring_resources(ring, RTLMQ_RX_RING_RESOURCE_XSK_PRIVATE))
		WARN_ON_ONCE(1);
	if (cold && (!tp->rx_ring_cold ||
		     cold < tp->rx_ring_cold ||
		     cold >= tp->rx_ring_cold + rtlmq_owned_rx_ring_slots(tp)))
		kfree(cold);
}

struct rtlmq_rx_ring_replacement *
rtlmq_prepare_xsk_rx_ring_replacement(struct rtlmq_private *tp, u16 queue_index,
				      struct xsk_buff_pool *new_pool)
{
	struct rtlmq_rx_ring_replacement *replacement;
	struct rtlmq_rx_ring_cold *candidate_cold;
	struct rtlmq_rx_ring *candidate;
	struct rtlmq_rx_ring *active;
	struct rtlmq_vector *vector;
	int ret;

	if (!tp || !tp->rx_rings ||
	    queue_index >= rtlmq_published_num_rx_queues(tp) ||
	    queue_index >= rtlmq_owned_rx_ring_slots(tp))
		return ERR_PTR(-EINVAL);
	lockdep_assert_held(&tp->control_lock);

	active = &tp->rx_rings[queue_index];
	if (!active->cold || !active->desc ||
	    !active->desc_alloc_size || !active->buffers)
		return ERR_PTR(-EINVAL);

	vector = rtlmq_active_rx_ring_consumer_vector(active);
	if (IS_ERR(vector))
		return ERR_CAST(vector);

	replacement = kzalloc_obj(*replacement, GFP_KERNEL);
	if (!replacement)
		return ERR_PTR(-ENOMEM);

	candidate = kmemdup(active, sizeof(*candidate), GFP_KERNEL);
	if (!candidate) {
		ret = -ENOMEM;
		goto err_free_replacement;
	}
	candidate_cold = kzalloc_obj(*candidate_cold, GFP_KERNEL);
	if (!candidate_cold) {
		ret = -ENOMEM;
		goto err_free_candidate;
	}

	candidate->cold = candidate_cold;
	rtlmq_reset_private_rx_ring_resources(candidate);
	u64_stats_init(&candidate_cold->syncp);
	candidate_cold->xsk_pool = new_pool;
	replacement->candidate = candidate;
	replacement->queue_index = queue_index;

	ret = rtlmq_alloc_rx_ring_owned(candidate, vector, READ_ONCE(tp->xdp_prog),
					RTLMQ_RX_RING_RESOURCE_XSK_PRIVATE);
	if (ret)
		goto err_release_candidate;

	ret = rtlmq_init_rx_ring(candidate);
	if (ret)
		goto err_release_candidate;

	if (candidate->desc_alloc_size != active->desc_alloc_size) {
		ret = -EINVAL;
		goto err_release_candidate;
	}

	return replacement;

err_release_candidate:
	rtlmq_release_xsk_private_ring(tp, candidate);
	replacement->candidate = NULL;
	kfree(candidate);
	goto err_free_replacement;
err_free_candidate:
	kfree(candidate);
err_free_replacement:
	kfree(replacement);
	return ERR_PTR(ret);
}

void rtlmq_commit_xsk_rx_ring_replacement(struct rtlmq_private *tp,
					  struct rtlmq_rx_ring_replacement *replacement)
{
	struct rtlmq_rx_ring *candidate;
	struct xsk_buff_pool *old_pool;
	struct xsk_buff_pool *new_pool;
	dma_addr_t candidate_desc_dma;
	struct rtlmq_rx_ring *active;
	size_t candidate_desc_size;
	u8 *candidate_desc;

	if (WARN_ON_ONCE(!tp || !replacement || replacement->committed ||
			 !replacement->aligned ||
			 !replacement->candidate))
		return;
	lockdep_assert_held(&tp->control_lock);

	active = &tp->rx_rings[replacement->queue_index];
	candidate = replacement->candidate;
	if (WARN_ON_ONCE(candidate->desc_alloc_size != active->desc_alloc_size))
		return;

	rtlmq_capture_rx_ring(&replacement->retired, active);

	/*
	 * XSK replacement is not a statistics reset. With NAPI stopped, carry
	 * cumulative counters into the replacement before installing it.
	 */
	rtlmq_carry_live_rx_ring_stats(candidate, active);

	old_pool = active->cold->xsk_pool;
	new_pool = candidate->cold->xsk_pool;
	if (old_pool)
		xsk_pool_set_rxq_info(old_pool, NULL);

	/*
	 * Reuse the active coherent descriptor allocation and programmed base.
	 * Copy the aligned candidate descriptors into it before transferring
	 * buffers and memory-model resources.
	 */
	candidate_desc = candidate->desc;
	candidate_desc_dma = candidate->desc_dma;
	candidate_desc_size = candidate->desc_alloc_size;
	memcpy(replacement->retired.desc, candidate_desc, candidate_desc_size);
	rtlmq_take_rx_descriptors(candidate, &replacement->retired);
	rtlmq_commit_rx_page_pool(candidate, rtlmq_active_runtime_layout(tp));
	rtlmq_take_rx_ring(active, candidate);
	replacement->candidate = NULL;
	kfree(candidate);
	dma_free_coherent(&tp->pdev->dev, candidate_desc_size,
			  candidate_desc, candidate_desc_dma);

	if (new_pool)
		xsk_pool_set_rxq_info(new_pool, &active->cold->xdp_rxq);
	dma_wmb();
	replacement->committed = true;
}

void rtlmq_finish_xsk_rx_ring_replacement(struct rtlmq_private *tp,
					  struct rtlmq_rx_ring_replacement *replacement)
{
	if (!replacement)
		return;
	lockdep_assert_held(&tp->control_lock);

	if (replacement->committed) {
		/* Wakeup and stats readers can enter after the precommit drain. */
		synchronize_rcu();
		rtlmq_release_xsk_private_ring(tp, &replacement->retired);
	} else if (replacement->candidate) {
		rtlmq_release_xsk_private_ring(tp, replacement->candidate);
		kfree(replacement->candidate);
	}

	kfree(replacement);
}

void rtlmq_discard_xsk_rx_ring_replacement(struct rtlmq_private *tp,
					   struct rtlmq_rx_ring_replacement *replacement)
{
	if (WARN_ON_ONCE(replacement && replacement->committed))
		return;
	rtlmq_finish_xsk_rx_ring_replacement(tp, replacement);
}
