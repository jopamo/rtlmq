/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef __RTLMQ_RX_H__
#define __RTLMQ_RX_H__

#include <linux/overflow.h>

/* RX rings and packet metadata. Keep XDP metadata types in their own header. */
#include <linux/cache.h>
#include <linux/compiler.h>
#include <linux/dma-mapping.h>
#include <linux/netdevice.h>
#include <linux/sizes.h>
#include <linux/u64_stats_sync.h>
#include <net/xdp.h>

#include "rtlmq_desc.h"
#include "rtlmq_base.h"
#include "rtlmq_xdp.h"

#define RTLMQ_MIN_RX_DESC		256U
#define RTLMQ_MAX_RX_DESC		1024U
#define RTLMQ_NUM_RX_DESC		RTLMQ_MAX_RX_DESC
/*
 * RTL8126A methods 2/3 select the vendor's Jumbo_Frame_16k limit and program
 * RxMaxSize for the complete receive span. The reference RTL8126 path uses the same
 * 16K window. Derive the Linux MTU after reserving one VLAN tag and the FCS;
 * do not copy a netdev MTU from either reference.
 *
 * A complete frame may span multiple descriptors. Each descriptor still
 * carries only the DMA payload available behind driver-managed headroom and
 * skb_shared_info tailroom. Keep aggregate hardware capacity, maximum
 * single-buffer span, and open-time page allocation distinct.
 */
#define RTLMQ_RX_HW_FRAME_SIZE		SZ_16K
#define RTLMQ_RX_MAX_MTU		(RTLMQ_RX_HW_FRAME_SIZE - \
					 VLAN_ETH_HLEN - ETH_FCS_LEN)
#define RTLMQ_RX_BUF_SIZE		(RTLMQ_RX_HW_FRAME_SIZE - 1)
#define RTLMQ_RX_ALLOC_SIZE		SZ_16K
#define RTLMQ_RX_HEADROOM		XDP_PACKET_HEADROOM
#define RTLMQ_RX_COPYBREAK		256U
#define RTLMQ_RX_TAILROOM		SKB_DATA_ALIGN(sizeof(struct skb_shared_info))
#define RTLMQ_RX_DMA_LEN		(RTLMQ_RX_BUF_SIZE - RTLMQ_RX_HEADROOM - \
					 RTLMQ_RX_TAILROOM)
#define RTLMQ_XDP_MAX_SEGS		16U
static_assert(ETH_MIN_MTU + ETH_HLEN >= ETH_ZLEN);
static_assert(RTLMQ_RX_MAX_MTU == 16362);
static_assert(RTLMQ_RX_HW_FRAME_SIZE > RTLMQ_RX_DMA_LEN);
static_assert(RTLMQ_RX_ALLOC_SIZE >= RTLMQ_RX_HEADROOM);
static_assert(RTLMQ_RX_ALLOC_SIZE - RTLMQ_RX_HEADROOM >=
	      RTLMQ_RX_DMA_LEN);
static_assert(RTLMQ_RX_ALLOC_SIZE - RTLMQ_RX_HEADROOM -
	      RTLMQ_RX_DMA_LEN >= RTLMQ_RX_TAILROOM);
static_assert(RTLMQ_RX_DMA_LEN <= RTLMQ_RX_DESC_LEN_MASK);

struct page_pool;
struct xsk_buff_pool;

enum rtlmq_rx_clean_exit_reason {
	RTLMQ_RX_CLEAN_EXIT_BUDGET,
	RTLMQ_RX_CLEAN_EXIT_HW_OWNED,
	RTLMQ_RX_CLEAN_EXIT_NO_STATUS,
	RTLMQ_RX_CLEAN_EXIT_ALLOC_FAIL,
	RTLMQ_RX_CLEAN_EXIT_DMA_MAP_FAIL,
	RTLMQ_RX_CLEAN_EXIT_REFILL_BLOCKED,
	RTLMQ_RX_CLEAN_EXIT_STATUS_UNSUPPORTED,
	RTLMQ_RX_CLEAN_EXIT_UNKNOWN,
};

struct rtlmq_rx_buffer {
	union {
		void *opaque;
		struct page *page;
		struct xdp_buff *xdp;
	};
	dma_addr_t dma_addr;
	void *hard_start;
	/* Retained copy source: CPU-touched payload bytes; zero means full span. */
	u16 sync_len;
};

enum {
	RTLMQ_RX_FAST_RXCSUM	= BIT(0),
	RTLMQ_RX_FAST_RXHASH	= BIT(1),
	RTLMQ_RX_FAST_STRIP_FCS	= BIT(2),
	RTLMQ_RX_FAST_RXVLAN	= BIT(3),
	RTLMQ_RX_FAST_RXALL	= BIT(4),
	RTLMQ_RX_FAST_COPY_RECYCLE = BIT(5),
};

/*
 * Hot structures require separate cachelines, including non-SMP builds where
 * ____cacheline_aligned_in_smp provides no alignment.
 */
struct rtlmq_rx_ring_read_mostly {
	struct net_device *ndev;
	struct napi_struct *napi;
	struct page_pool *page_pool;
	u8 *desc;
	dma_addr_t desc_dma;
	size_t desc_alloc_size;
	const struct rtlmq_rx_desc_profile *profile;
	u32 rx_fast_flags;
	__le32 rx_refill_opts1;
	__le32 rx_refill_opts1_eor;
	__le32 rx_refill_opts2;
	u32 alloc_size;
	u16 buffer_len;
	u16 ring_count;
	u16 ring_mask;
	u16 queue_index;
	u16 refill_batch;
	u16 rx_refill_clear_size;
	bool rx_fast_skb_refill;
	struct rtlmq_rx_buffer *buffers;
} ____cacheline_aligned;

struct rtlmq_rx_ring_consumer {
	/* NAPI consumes descriptors from this cursor. */
	u16 next_to_clean;
	/* NAPI refills descriptors from this cursor. */
	u16 next_to_alloc;
	/*
	 * Posted descriptors, including completed but unconsumed slots. This
	 * distinguishes empty from full on wrap. Setup initializes it; only NAPI
	 * updates it while running. An empty XSK ring stays masked until
	 * ndo_xsk_wakeup() schedules refill from new fill entries.
	 */
	u16 rx_posted;
	/*
	 * Keep RX counters with the NAPI consumer that advances them.
	 * Standard queue statistics use hw_packets/hw_bytes; delivery and
	 * clean observations belong to private builds.
	 */
	u64 hw_packets;
	u64 hw_bytes;
} ____cacheline_aligned;

struct rtlmq_rx_ring_cold {
	struct xdp_rxq_info xdp_rxq;
	struct rtlmq_rx_rss_xdp_map xdp_rss_map;
	ktime_t mb_rx_timestamp;
	bool mb_rx_timestamp_valid;
	struct xsk_buff_pool *xsk_pool;
	struct xdp_buff *xsk_mb_head;
	u32 xsk_mb_len;
	u16 xsk_mb_segs;
	struct rtlmq_xdp_buff page_mb_xdp;
	struct rtlmq_rx_buffer page_mb_head;
	u32 page_mb_len;
	u16 page_mb_segs;
	/* The packet was already dropped; consume its suffix without recounting. */
	bool mb_discarding;
	u8 packet_errors;
	struct u64_stats_sync syncp;
	struct rtlmq_rx_ring_stats stats;
};

struct rtlmq_rx_ring {
	struct rtlmq_private *priv;
	/*
	 * Named blocks let helpers access read-mostly and consumer cachelines.
	 * Anonymous members provide ring->field access to the same storage.
	 * Keep both field lists identical; assertions below check the layout.
	 */
	union {
		struct rtlmq_rx_ring_read_mostly read_mostly;
		struct {
			struct net_device *ndev;
			struct napi_struct *napi;
			struct page_pool *page_pool;
			u8 *desc;
			dma_addr_t desc_dma;
			size_t desc_alloc_size;
			const struct rtlmq_rx_desc_profile *profile;
			u32 rx_fast_flags;
			__le32 rx_refill_opts1;
			__le32 rx_refill_opts1_eor;
			__le32 rx_refill_opts2;
			u32 alloc_size;
			u16 buffer_len;
			u16 ring_count;
			u16 ring_mask;
			u16 queue_index;
			u16 refill_batch;
			u16 rx_refill_clear_size;
			bool rx_fast_skb_refill;
			struct rtlmq_rx_buffer *buffers;
		};
	};
	union {
		struct rtlmq_rx_ring_consumer consumer;
		struct {
			u16 next_to_clean;
			u16 next_to_alloc;
			u16 rx_posted;
			u64 hw_packets;
			u64 hw_bytes;
		};
	};
	u16 vector_index;
	/*
	 * Each RX ring uses page-pool pages or its bound XSK pool. NAPI recycles
	 * consumed buffers and prepares replacements before setting descriptor
	 * OWN for hardware.
	 */
	struct rtlmq_rx_ring_cold *cold;
};

static_assert(__alignof__(struct rtlmq_rx_ring_read_mostly) >=
	      SMP_CACHE_BYTES);
static_assert(__alignof__(struct rtlmq_rx_ring_consumer) >=
	      SMP_CACHE_BYTES);
/*
 * Check maximum cacheline counts, not exact sizes, to allow smaller layouts
 * on other architectures. Shared descriptor-profile tables keep the read-mostly
 * block within budget even with 32-byte SMP cachelines.
 */
static_assert(sizeof(struct rtlmq_rx_ring_read_mostly) <=
	      3 * SMP_CACHE_BYTES);
static_assert(sizeof(struct rtlmq_rx_ring_consumer) <=
	      2 * SMP_CACHE_BYTES);
static_assert(!(offsetof(struct rtlmq_rx_ring, read_mostly) %
		SMP_CACHE_BYTES));
static_assert(!(offsetof(struct rtlmq_rx_ring, consumer) %
		SMP_CACHE_BYTES));
static_assert(offsetof(struct rtlmq_rx_ring, consumer) >=
	      offsetofend(struct rtlmq_rx_ring, read_mostly));
static_assert((offsetof(struct rtlmq_rx_ring, read_mostly) /
	       SMP_CACHE_BYTES) !=
	      (offsetof(struct rtlmq_rx_ring, consumer) /
	       SMP_CACHE_BYTES));

struct bpf_prog;
struct net_device;
struct rtlmq_private;
struct rtlmq_runtime_layout_state;
struct rtlmq_vector;
struct sk_buff;
struct xdp_buff;
struct xdp_rxq_info;

/*
 * RX geometry calculated on the control path. Keep allocation size, the
 * descriptor/DMA span, and the two XDP-family data offsets distinct even
 * though RTL8125B currently uses the same numeric headroom for both XDP
 * modes. XSK pool headroom supplied by userspace is validated separately by
 * AF_XDP setup against this core-required data offset.
 *
 * dma_writable_len includes the device-written FCS. Delivery strips it unless
 * NETIF_F_RXFCS is enabled.
 */
struct rtlmq_rx_geometry {
	u32 allocation_size;
	u32 dma_writable_len;
	u32 xdp_headroom;
	u32 xsk_headroom;
};

enum rtlmq_rx_memory_model {
	RTLMQ_RX_MEMORY_MODEL_NONE = 0,
	RTLMQ_RX_MEMORY_MODEL_PAGE_POOL = 1,
	RTLMQ_RX_MEMORY_MODEL_XSK_POOL = 2,
};

/*
 * Visibility for one committed RX ring. The ring and page-pool
 * fields remain separate so a mismatch is observable instead of normalized
 * away. An inactive queue is all zero; a published but inconsistent queue has
 * published=true and valid=false.
 */
struct rtlmq_rx_published_geometry {
	u32 allocation_size;
	u32 dma_span;
	u32 page_pool_dma_span;
	u32 ring_count;
	u32 page_pool_size;
	u32 memory_model;
	u32 page_order;
	bool published;
	bool valid;
};

void rtlmq_snapshot_published_rx_geometry_locked(const struct rtlmq_private *tp, u16 queue_index,
						 struct rtlmq_rx_published_geometry *snapshot);

/* Requires normalized page geometry; UMEM capacity is validated separately. */
static inline u32
rtlmq_rx_geometry_page_span(const struct rtlmq_rx_geometry *geometry)
{
	return min_t(u32, geometry->dma_writable_len,
		     geometry->allocation_size - RTLMQ_RX_HEADROOM -
			     RTLMQ_RX_TAILROOM);
}

int rtlmq_rx_geometry_normalize(u32 mtu, struct rtlmq_rx_geometry *geometry);
int rtlmq_rx_geometry_normalize_pages(u32 mtu, bool large_buffers,
				      struct rtlmq_rx_geometry *geometry);
bool rtlmq_rx_large_buffers_for(const struct bpf_prog *prog,
				const struct xsk_buff_pool *pool);
bool rtlmq_rx_xdp_requires_rebuild(const struct rtlmq_private *tp,
				   const struct bpf_prog *prog);
u32 rtlmq_rx_dma_len_for_mtu(u32 mtu);
u32 rtlmq_max_supported_mtu(void);

static inline u32 rtlmq_max_xsk_frame_len(void)
{
	return min_t(u32, RTLMQ_RX_DESC_LEN_MASK, RTLMQ_RX_DMA_LEN);
}

static inline u32 rtlmq_rx_page_required_size(u32 dma_len)
{
	u32 required;

	if (check_add_overflow((u32)RTLMQ_RX_HEADROOM, dma_len, &required) ||
	    check_add_overflow(required, (u32)RTLMQ_RX_TAILROOM, &required))
		return 0;

	return required;
}

static inline bool
rtlmq_rx_xsk_frame_covers_geometry(u32 frame_size,
				   const struct rtlmq_rx_geometry *geometry)
{
	return geometry && geometry->dma_writable_len &&
	       frame_size >= geometry->dma_writable_len;
}

static inline bool
rtlmq_rx_xsk_frame_size_valid_geometry(u32 frame_size,
				       const struct rtlmq_rx_geometry *geometry)
{
	return rtlmq_rx_xsk_frame_covers_geometry(frame_size, geometry) &&
	       frame_size <= rtlmq_max_xsk_frame_len();
}

static inline bool
rtlmq_rx_sg_frame_size_valid_geometry(u32 fragment_size, u32 max_segs,
				      const struct rtlmq_rx_geometry *geometry)
{
	u32 covered;

	return geometry && fragment_size && max_segs &&
	       !check_mul_overflow(fragment_size, max_segs, &covered) &&
	       covered >= geometry->dma_writable_len;
}

static inline bool rtlmq_rx_multibuffer_add_len(u32 accumulated, u32 fragment,
						u32 limit, u32 *total)
{
	return total && !check_add_overflow(accumulated, fragment, total) &&
	       *total <= limit;
}

static inline bool
rtlmq_rx_xsk_sg_frame_size_valid_geometry(u32 frame_size, u32 max_segs,
					  const struct rtlmq_rx_geometry *geometry)
{
	return frame_size <= rtlmq_max_xsk_frame_len() &&
	       rtlmq_rx_sg_frame_size_valid_geometry(frame_size, max_segs, geometry);
}

/* RX fast-path settings and per-vector polling. */
static inline bool rtlmq_rx_ring_uses_xsk_pool(const struct rtlmq_rx_ring *ring)
{
	return ring->cold && !!ring->cold->xsk_pool;
}

static inline bool
rtlmq_rx_ring_uses_single_descriptor(const struct rtlmq_rx_ring *ring)
{
	u32 dma_len;

	if (!ring || !ring->ndev)
		return false;
	dma_len = rtlmq_rx_dma_len_for_mtu(ring->ndev->mtu);
	return dma_len && dma_len <= ring->buffer_len;
}

bool rtlmq_rx_ring_xdp_mem_model_matches(const struct rtlmq_rx_ring *ring,
					 const struct net_device *ndev,
					 u16 queue_index);
void rtlmq_release_rx_multibuffer(struct rtlmq_rx_ring *ring);

static inline bool
rtlmq_rx_skb_fast_entry_ready(const struct rtlmq_rx_ring *ring)
{
	return READ_ONCE(ring->rx_fast_skb_refill) &&
	       likely(ring->cold);
}

static inline u32 rtlmq_rx_desc_dword_read_fast(const void *desc, u16 offset)
{
	return le32_to_cpu(READ_ONCE(*(const __le32 *)((const u8 *)desc +
						       offset)));
}

static inline bool rtlmq_rx_desc_status_cpu_owned(u32 status)
{
	/*
	 * Shared by readiness checks and RX clean. OWN set means hardware owns
	 * the descriptor; after OWN clears, callers must order subsequent reads.
	 */
	return !(status & RTLMQ_DESC_OWN);
}

static inline bool
rtlmq_rx_buffer_ready_for_device(const struct rtlmq_rx_buffer *buffer)
{
	/*
	 * Set opaque only after the DMA address and CPU mapping are ready.
	 * Do not set descriptor OWN for a partially initialized buffer.
	 */
	return buffer && buffer->opaque && buffer->hard_start;
}

static inline bool rtlmq_rx_desc_acquire(const struct rtlmq_rx_ring *ring,
					 const void *desc, u32 *status)
{
	*status = rtlmq_rx_desc_dword_read_fast(desc,
						ring->profile->opts1_offset);

	/*
	 * Do not read descriptor fields or packet data while OWN is set.
	 * After it clears, dma_rmb() orders subsequent reads before the caller
	 * accesses the completed buffer.
	 */
	if (!rtlmq_rx_desc_status_cpu_owned(*status))
		return false;

	dma_rmb();
	return true;
}

static inline bool rtlmq_rx_ring_head_ready(const struct rtlmq_rx_ring *ring)
{
	unsigned int entry;
	u16 ring_count;
	u16 stride;
	u32 status;
	void *desc;

	if (unlikely(!ring || !ring->desc || !ring->profile))
		return false;

	ring_count = READ_ONCE(ring->ring_count);
	stride = READ_ONCE(ring->profile->stride);
	if (unlikely(!ring_count || !stride))
		return false;

	if (!READ_ONCE(ring->rx_posted))
		return false;

	/*
	 * Sample the same published clean cursor that RX clean consumes.
	 * Descriptors behind next_to_clean are already consumed and cannot
	 * contribute readiness.
	 */
	entry = READ_ONCE(ring->next_to_clean);
	if (unlikely(entry >= ring_count))
		entry &= READ_ONCE(ring->ring_mask);

	desc = rtlmq_rx_desc_addr(ring, entry);
	/* Read the descriptor, not cached diagnostic status. */
	return rtlmq_rx_desc_acquire(ring, desc, &status);
}

void rtlmq_refresh_rx_ring_fast_state(struct rtlmq_rx_ring *ring);
void rtlmq_sync_rx_fast_features(struct rtlmq_private *tp,
				 netdev_features_t features);
int rtlmq_poll_rx_ring(struct rtlmq_vector *vector, struct rtlmq_rx_ring *ring,
		       int budget, bool skip_entry_own_mmio,
		       enum rtlmq_rx_clean_exit_reason *exit_reason_out);
int rtlmq_poll_rx_ring_skb_fast(struct rtlmq_vector *vector, struct rtlmq_rx_ring *ring, int budget,
				bool skip_entry_own_mmio,
				enum rtlmq_rx_clean_exit_reason *exit_reason_out);

static inline u32 rtlmq_rx_fast_flags_from_features(netdev_features_t features)
{
	u32 flags = 0;

	if (features & NETIF_F_RXCSUM)
		flags |= RTLMQ_RX_FAST_RXCSUM;
	if (features & NETIF_F_RXHASH)
		flags |= RTLMQ_RX_FAST_RXHASH;
	if (!(features & NETIF_F_RXFCS))
		flags |= RTLMQ_RX_FAST_STRIP_FCS;
	if (features & NETIF_F_HW_VLAN_CTAG_RX)
		flags |= RTLMQ_RX_FAST_RXVLAN;
	if (features & NETIF_F_RXALL)
		flags |= RTLMQ_RX_FAST_RXALL;

	return flags;
}

static inline void rtlmq_rx_stats_update_begin(struct rtlmq_rx_ring *ring)
{
	if (!ring || !ring->cold)
		return;

	u64_stats_update_begin(&ring->cold->syncp);
}

static inline void rtlmq_rx_stats_update_end(struct rtlmq_rx_ring *ring)
{
	if (!ring || !ring->cold)
		return;

	u64_stats_update_end(&ring->cold->syncp);
}

static inline unsigned int
rtlmq_rx_stats_fetch_begin(const struct rtlmq_rx_ring *ring)
{
	if (!ring || !ring->cold)
		return 0;

	return u64_stats_fetch_begin(&ring->cold->syncp);
}

static inline bool rtlmq_rx_stats_fetch_retry(const struct rtlmq_rx_ring *ring,
					      unsigned int start)
{
	if (!ring || !ring->cold)
		return false;

	return u64_stats_fetch_retry(&ring->cold->syncp, start);
}

bool rtlmq_rx_copy_recycle_enabled(void);

static inline bool
rtlmq_rx_copy_recycle_active(const struct rtlmq_rx_ring *ring)
{
	return READ_ONCE(ring->rx_fast_flags) & RTLMQ_RX_FAST_COPY_RECYCLE;
}

struct xdp_frame *rtlmq_rx_copy_xdp_frame(const struct xdp_buff *xdp);
void rtlmq_release_rx_page_packet(struct rtlmq_rx_ring *ring,
				  struct rtlmq_rx_buffer *buffer,
				  const struct xdp_buff *xdp, bool direct);

enum rtlmq_rx_refill_failure {
	RTLMQ_RX_REFILL_OK,
	RTLMQ_RX_REFILL_ALLOC_FAILED,
	RTLMQ_RX_REFILL_DMA_FAILED,
	RTLMQ_RX_REFILL_BLOCKED,
};

void *rtlmq_rx_buffer_hard_start(const struct rtlmq_rx_ring *ring,
				 const struct rtlmq_rx_buffer *buffer);
void rtlmq_sync_rx_page_pool_buffer_for_cpu(struct rtlmq_rx_ring *ring,
					    const struct rtlmq_rx_buffer *buffer,
					    unsigned int len);
void rtlmq_sync_rx_buffer_for_cpu(struct rtlmq_rx_ring *ring,
				  struct rtlmq_rx_buffer *buffer,
				  unsigned int len);
void rtlmq_sync_rx_buffer_for_device(struct rtlmq_rx_ring *ring,
				     struct rtlmq_rx_buffer *buffer,
				     unsigned int len);
int rtlmq_alloc_rx_buffer(struct rtlmq_rx_ring *ring,
			  struct rtlmq_rx_buffer *buffer, gfp_t gfp);
void rtlmq_release_rx_buffer(struct rtlmq_rx_ring *ring,
			     struct rtlmq_rx_buffer *buffer, bool direct);
void rtlmq_recycle_rx_buffer_napi(struct rtlmq_rx_ring *ring,
				  struct rtlmq_rx_buffer *buffer);
unsigned int
rtlmq_refill_rx_ring_generic(struct rtlmq_rx_ring *ring, unsigned int cleaned,
			     enum rtlmq_rx_refill_failure *failure);
unsigned int rtlmq_refill_rx_ring_fast(struct rtlmq_rx_ring *ring,
				       unsigned int cleaned,
				       enum rtlmq_rx_refill_failure *failure);

#endif /* __RTLMQ_RX_H__ */
