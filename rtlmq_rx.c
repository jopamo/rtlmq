// SPDX-License-Identifier: GPL-2.0-only

#include <linux/delay.h>
#include <linux/etherdevice.h>
#include <linux/err.h>
#include <linux/filter.h>
#include <linux/if_vlan.h>
#include <linux/mm.h>
#include <linux/moduleparam.h>
#include <linux/overflow.h>
#include <linux/prefetch.h>
#include <linux/skbuff.h>
#include <linux/slab.h>
#include <linux/topology.h>
#include <linux/bpf_trace.h>
#include <net/xdp_sock_drv.h>

#include "rtlmq_hw.h"
#include "rtlmq_runtime_accessors.h"
#include "rtlmq_irq.h"
#include "rtlmq_runtime.h"
#include "rtlmq.h"
#include "rtlmq_regs.h"
#include "rtlmq_rx.h"
#include "rtlmq_tx.h"
#include "rtlmq_xdp.h"
#include "rtlmq_ptp.h"
#include "rtlmq_trace.h"

/* Copy construction borrows RX storage; it never releases a source buffer. */

static bool rtlmq_rx_copy_recycle;
module_param_named(rx_copy_recycle, rtlmq_rx_copy_recycle, bool, 0444);
MODULE_PARM_DESC(rx_copy_recycle, "[policy] ordinary-SKB copy/recycle, default off");

bool rtlmq_rx_copy_recycle_enabled(void)
{
	return rtlmq_rx_copy_recycle;
}

static int rtlmq_rx_copy_linear_data(struct sk_buff *skb, const void *data,
				     unsigned int pkt_size)
{
	if (!skb || !data || !pkt_size)
		return -EINVAL;
	if (unlikely(skb_tailroom(skb) < pkt_size))
		return -EMSGSIZE;

	skb_put_data(skb, data, pkt_size);
	return 0;
}

static int rtlmq_rx_copy_xdp_data(void *target, const struct xdp_buff *xdp,
				  unsigned int pkt_size)
{
	const struct skb_shared_info *sinfo;
	unsigned int copied;
	unsigned int i;

	if (!target || !xdp || !xdp->data)
		return -EINVAL;

	copied = xdp->data_end - xdp->data;
	if (!copied || copied > pkt_size)
		return -EMSGSIZE;
	memcpy(target, xdp->data, copied);

	if (xdp_buff_has_frags(xdp)) {
		sinfo = xdp_get_shared_info_from_buff(xdp);
		if (sinfo->nr_frags > MAX_SKB_FRAGS)
			return -EMSGSIZE;
		for (i = 0; i < sinfo->nr_frags; i++) {
			const skb_frag_t *frag = &sinfo->frags[i];
			const void *frag_data = skb_frag_address_safe(frag);
			unsigned int frag_len = skb_frag_size(frag);
			unsigned int next_copied;

			if (!frag_data || !frag_len ||
			    check_add_overflow(copied, frag_len,
					       &next_copied) ||
			    next_copied > pkt_size)
				return -EMSGSIZE;
			memcpy(target + copied, frag_data, frag_len);
			copied = next_copied;
		}
	}

	return copied == pkt_size ? 0 : -EMSGSIZE;
}

static int rtlmq_rx_copy_xdp_skb_data(struct sk_buff *skb,
				      const struct xdp_buff *xdp,
				      unsigned int pkt_size)
{
	int ret;

	if (!skb || !xdp)
		return -EINVAL;
	if (skb_tailroom(skb) < pkt_size)
		return -EMSGSIZE;
	ret = rtlmq_rx_copy_xdp_data(skb_tail_pointer(skb), xdp, pkt_size);
	if (!ret)
		skb_put(skb, pkt_size);
	return ret;
}

struct xdp_frame *rtlmq_rx_copy_xdp_frame(const struct xdp_buff *xdp)
{
	unsigned int pkt_size, meta_len, size;
	struct xdp_frame *frame;
	struct page *page;

	if (!xdp || !xdp->data || !xdp->data_meta)
		return NULL;
	pkt_size = xdp_get_buff_len(xdp);
	meta_len = xdp_data_meta_unsupported(xdp) ? 0 :
		   xdp->data - xdp->data_meta;
	if (!pkt_size || pkt_size > RTLMQ_RX_BUF_SIZE ||
	    xdp_metalen_invalid(meta_len))
		return NULL;
	size = sizeof(*frame) + meta_len + pkt_size +
	       SKB_DATA_ALIGN(sizeof(struct skb_shared_info));
	page = dev_alloc_pages(get_order(size));
	if (!page)
		return NULL;

	/* dev_alloc_pages supplies a compound reference; XDP return uses put_page. */
	frame = page_address(page);
	memset(frame, 0, sizeof(*frame));
	frame->data = (void *)(frame + 1) + meta_len;
	frame->len = pkt_size;
	frame->headroom = meta_len;
	frame->metasize = meta_len;
	frame->frame_sz = PAGE_SIZE << get_order(size);
	frame->mem_type = MEM_TYPE_PAGE_ORDER0;
	if (meta_len)
		memcpy(frame->data - meta_len, xdp->data_meta, meta_len);
	if (rtlmq_rx_copy_xdp_data(frame->data, xdp, pkt_size)) {
		put_page(page);
		return NULL;
	}
	return frame;
}

static struct sk_buff *
rtlmq_rx_copy_skb(struct napi_struct *napi,
		  const struct xdp_buff *xdp, const void *data,
		  const void *data_meta, unsigned int pkt_size, unsigned int meta_len)
{
	unsigned int skb_len;
	struct sk_buff *skb;

	if (unlikely(!napi || !data || !pkt_size ||
		     check_add_overflow(pkt_size, meta_len, &skb_len)))
		return ERR_PTR(-EINVAL);

	skb = napi_alloc_skb(napi, skb_len);
	if (unlikely(!skb))
		return ERR_PTR(-ENOMEM);

	/* Preserve allocator truesize; no RX page is attached to this skb. */
	if (meta_len) {
		if (unlikely(!data_meta || meta_len > skb_tailroom(skb)))
			goto invalid;
		skb_reserve(skb, meta_len);
		memcpy(skb->data - meta_len, data_meta, meta_len);
		skb_metadata_set(skb, meta_len);
	}

	if (unlikely(xdp ? rtlmq_rx_copy_xdp_skb_data(skb, xdp, pkt_size) :
			   rtlmq_rx_copy_linear_data(skb, data, pkt_size)))
		goto invalid;

	return skb;

invalid:
	dev_kfree_skb_any(skb);
	return ERR_PTR(-EINVAL);
}

struct rtlmq_rx_poll_hot_stats {
	u64 drops;
	u64 drop_reason[RTLMQ_RX_DROP_COUNT];
	u64 alloc_fail;
	u64 csum_ok;
	u64 csum_bad;
};

struct rtlmq_rx_fast_batch {
	unsigned int rx_packets;
	unsigned int hw_packets;
	u32 hw_bytes;
	unsigned int rx_errors;
	unsigned int rx_crc_errors;
	unsigned int rx_length_errors;
};

static void rtlmq_rx_publish_fast_batch(struct rtlmq_rx_ring *ring,
					struct rtlmq_rx_ring_stats *stats,
					const struct rtlmq_rx_fast_batch *batch)
{
	ring->hw_packets += batch->hw_packets;
	ring->hw_bytes += batch->hw_bytes;
	if (!stats)
		return;

	if (batch->rx_errors)
		stats->rx_errors += batch->rx_errors;
	if (batch->rx_crc_errors)
		stats->rx_crc_errors += batch->rx_crc_errors;
	if (batch->rx_length_errors)
		stats->rx_length_errors += batch->rx_length_errors;
}

static enum rtlmq_rx_clean_exit_reason
rtlmq_rx_refill_failure_exit_reason(enum rtlmq_rx_refill_failure failure)
{
	switch (failure) {
	case RTLMQ_RX_REFILL_ALLOC_FAILED:
		return RTLMQ_RX_CLEAN_EXIT_ALLOC_FAIL;
	case RTLMQ_RX_REFILL_DMA_FAILED:
		return RTLMQ_RX_CLEAN_EXIT_DMA_MAP_FAIL;
	case RTLMQ_RX_REFILL_OK:
	case RTLMQ_RX_REFILL_BLOCKED:
	default:
		return RTLMQ_RX_CLEAN_EXIT_REFILL_BLOCKED;
	}
}

static enum rtlmq_rx_clean_exit_reason
rtlmq_rx_refill_clean_exit_reason(enum rtlmq_rx_clean_exit_reason reason,
				  unsigned int refill_pending,
				  unsigned int posted,
				  enum rtlmq_rx_refill_failure failure
)
{
	if (!refill_pending || posted == refill_pending)
		return reason;
	if (reason == RTLMQ_RX_CLEAN_EXIT_BUDGET)
		return reason;

	return rtlmq_rx_refill_failure_exit_reason(failure);
}

static enum rtlmq_rx_clean_exit_reason
rtlmq_rx_refill_fast_clean_exit(struct rtlmq_rx_ring *ring,
				unsigned int refill_pending,
				enum rtlmq_rx_clean_exit_reason exit_reason
)
{
	enum rtlmq_rx_refill_failure failure;
	unsigned int posted;

	posted = rtlmq_refill_rx_ring_fast(ring, refill_pending, &failure);
	return rtlmq_rx_refill_clean_exit_reason(exit_reason,
						refill_pending, posted, failure
	);
}

static void rtlmq_rx_flush_hot_stats(struct rtlmq_rx_ring_stats *stats,
				     const struct rtlmq_rx_poll_hot_stats *hot)
{
	unsigned int i;

	if (!stats || !hot)
		return;

	stats->drops += hot->drops;
	for (i = 0; i < RTLMQ_RX_DROP_COUNT; i++)
		stats->drop_reason[i] += hot->drop_reason[i];
	stats->alloc_fail += hot->alloc_fail;
	stats->csum_ok += hot->csum_ok;
	stats->csum_bad += hot->csum_bad;
}

static void rtlmq_rx_prefetch_next_slot(const u8 *desc_base, u16 desc_stride,
					const struct rtlmq_rx_buffer *buffers,
					unsigned int entry)
{
	prefetch(desc_base + entry * desc_stride);
	prefetch(&buffers[entry]);
}

static unsigned int rtlmq_rx_refill_pending(const struct rtlmq_rx_ring *ring)
{
	/* Posted slots form one interval starting at the clean cursor. */
	return ring->ring_count - READ_ONCE(ring->rx_posted);
}

static void rtlmq_rx_desc_quirk(struct rtlmq_private *tp)
{
	if (!rtlmq_mmio_ordinary_authorized(tp))
		return;

	/*
	 * RTL8125B can briefly report a stale RX DescOwn bit after interrupt
	 * delivery. Read IMR to flush the internal status path before rechecking.
	 */
	readl(tp->mmio_addr + RTLMQ_INTR_MASK);
}

static bool rtlmq_rx_desc_try_recover_own_fast(struct rtlmq_rx_ring *ring,
					       const void *desc, u32 *status)
{
	struct rtlmq_private *tp;

	if (!ring || !desc || !status)
		return false;

	tp = ring->priv;
	if (!tp || !rtlmq_mmio_ordinary_authorized(tp))
		return false;

	/*
	 * Match the vendor/upstream RX exit workaround: one MMIO flush and
	 * DescOwn recheck. The fast path needs it too, or stale OWN sends a
	 * ready descriptor through an unnecessary NAPI complete/rearm cycle.
	 */
	rtlmq_rx_desc_quirk(tp);
	if (!rtlmq_rx_desc_acquire(ring, desc, status))
		return false;

	return true;
}

static void rtlmq_rx_clear_buffer_fast(struct rtlmq_rx_buffer *buffer)
{
	WRITE_ONCE(buffer->opaque, NULL);
	WRITE_ONCE(buffer->dma_addr, 0);
	WRITE_ONCE(buffer->hard_start, NULL);
	buffer->sync_len = 0;
}

static void rtlmq_reuse_rx_buffer(struct rtlmq_rx_ring *ring,
				  struct rtlmq_rx_buffer *buffer)
{
	if (!buffer->opaque)
		return;

	/*
	 * An XSK ring can be only partially populated. Its refill cursor then
	 * points beyond the completed descriptor, so retaining the frame in
	 * that descriptor strands it behind next_to_alloc and eventually
	 * drains the fill ring. Return completed driver-owned frames to the
	 * pool's local free list instead. The common refill path allocates
	 * from that list first, resets the xdp_buff, performs the device DMA
	 * sync, and posts it at next_to_alloc.
	 */
	if (rtlmq_rx_ring_uses_xsk_pool(ring)) {
		rtlmq_recycle_rx_buffer_napi(ring, buffer);
		return;
	}

	/* Ordinary pages remain in place; refill owns their next device sync. */
}

static unsigned int rtlmq_rx_frame_len_checked(unsigned int pkt_size,
					       bool strip_fcs)
{
	if (strip_fcs) {
		if (pkt_size <= ETH_FCS_LEN)
			return 0;
		pkt_size -= ETH_FCS_LEN;
	}

	return pkt_size;
}

enum rtlmq_rx_length_error {
	RTLMQ_RX_LENGTH_OK = 0,
	RTLMQ_RX_LENGTH_ZERO,
	RTLMQ_RX_LENGTH_TOO_SMALL,
	RTLMQ_RX_LENGTH_TOO_LARGE,
};

/*
 * Normalize and validate one hardware-reported packet length before any
 * buffer sync, XDP access, copy, or delivery. The descriptor decoder has
 * already applied the format-specific length mask; this helper applies the
 * feature-dependent FCS policy and the active buffer capacity.
 */
static enum rtlmq_rx_length_error
rtlmq_rx_packet_len_validate(const struct rtlmq_rx_ring *ring,
			     const struct rtlmq_rx_buffer *buffer,
			     const struct rtlmq_rx_desc_decode *decoded,
			     bool strip_fcs, unsigned int *pkt_size)
{
	unsigned int normalized;

	if (!ring || !buffer || !decoded || !pkt_size)
		return RTLMQ_RX_LENGTH_TOO_LARGE;

	normalized = decoded->packet_len;
	if (!normalized)
		return RTLMQ_RX_LENGTH_ZERO;

	if (!buffer->opaque || !ring->buffer_len ||
	    normalized > ring->buffer_len)
		return RTLMQ_RX_LENGTH_TOO_LARGE;

	/*
	 * Hardware can put only the last one to four FCS bytes in a final
	 * descriptor when the frame crosses a DMA-span boundary. There is no
	 * packet data in that descriptor to normalize. Multibuffer assembly
	 * recycles it and removes any preceding FCS bytes from the assembled
	 * tail before XDP or the stack can observe the frame.
	 */
	if (strip_fcs && decoded->fragmented && decoded->last_frag &&
	    normalized <= ETH_FCS_LEN) {
		*pkt_size = 0;
		return RTLMQ_RX_LENGTH_OK;
	}

	normalized = rtlmq_rx_frame_len_checked(normalized, strip_fcs && decoded->last_frag);
	if (!normalized)
		return RTLMQ_RX_LENGTH_TOO_SMALL;

	*pkt_size = normalized;
	return RTLMQ_RX_LENGTH_OK;
}

/*
 * build_skb() consumes the allocation minus skb_shared_info tailroom. Keep
 * the check beside the packet consumers so an XDP-adjusted data offset cannot
 * turn a valid descriptor length into an out-of-bounds skb write.
 */
static bool rtlmq_rx_skb_data_bounds_valid(const struct rtlmq_rx_ring *ring,
					   unsigned int data_offset,
					   unsigned int pkt_size)
{
	unsigned int capacity;

	if (!ring || ring->alloc_size <= (u32)RTLMQ_RX_TAILROOM)
		return false;

	capacity = ring->alloc_size - (u32)RTLMQ_RX_TAILROOM;
	return data_offset <= capacity &&
	       pkt_size <= capacity - data_offset;
}

enum rtlmq_rx_packet_error {
	RTLMQ_RX_ERROR_ANY = BIT(0),
	RTLMQ_RX_ERROR_CRC = BIT(1),
	RTLMQ_RX_ERROR_LENGTH = BIT(2),
};

static u8 rtlmq_rx_packet_errors(struct rtlmq_rx_ring_cold *cold,
				 const struct rtlmq_rx_desc_decode *decoded)
{
	u8 seen = decoded->first_frag ? 0 : cold->packet_errors;
	u8 errors = 0;

	if (decoded->error) {
		errors = RTLMQ_RX_ERROR_ANY;
		if (decoded->error_crc)
			errors |= RTLMQ_RX_ERROR_CRC;
		if (decoded->error_length)
			errors |= RTLMQ_RX_ERROR_LENGTH;
	}
	/* Count each error class once, including a dropped packet's suffix. */
	if (decoded->fragmented || cold->packet_errors)
		cold->packet_errors = decoded->last_frag ? 0 : seen | errors;

	return errors & ~seen;
}

static void
rtlmq_rx_account_packet_errors(struct rtlmq_rx_ring_stats *stats, u8 errors)
{
	if (errors & RTLMQ_RX_ERROR_ANY)
		stats->rx_errors++;
	if (errors & RTLMQ_RX_ERROR_CRC)
		stats->rx_crc_errors++;
	if (errors & RTLMQ_RX_ERROR_LENGTH)
		stats->rx_length_errors++;
}

struct rtlmq_xdp_result {
	void *data;
	void *data_meta;
	unsigned int len;
	unsigned int meta_len;
	bool dma_cpu_owned;
	bool detached;
	struct xdp_buff *xdp;
	struct rtlmq_xdp_buff local_xdp;
	struct bpf_prog *prog;
};

static unsigned int rtlmq_rx_dma_data_limit(const struct rtlmq_rx_ring *ring)
{
	return ring->buffer_len;
}

static unsigned int rtlmq_rx_mapped_headroom(const struct rtlmq_rx_ring *ring)
{
	if (rtlmq_rx_ring_uses_xsk_pool(ring))
		return XDP_PACKET_HEADROOM;

	return RTLMQ_RX_HEADROOM;
}

static bool rtlmq_rx_xdp_bounds_valid(const struct rtlmq_rx_ring *ring,
				      const struct rtlmq_rx_buffer *buffer,
				      const struct rtlmq_xdp_result *xdp,
				      unsigned int data_limit)
{
	unsigned long hard_start;
	unsigned long data_meta;
	unsigned int seg_count;
	unsigned int span_len;
	unsigned int max_segs;
	unsigned long offset;
	unsigned int max_len;
	unsigned long data;

	if (!xdp->data)
		return false;

	if (!data_limit || !buffer->opaque || !ring->buffer_len)
		return false;

	if (ring->buffer_len > data_limit)
		return false;

	if (check_add_overflow(rtlmq_rx_mapped_headroom(ring),
			       ring->buffer_len, &span_len))
		return false;
	hard_start = (unsigned long)rtlmq_rx_buffer_hard_start(ring, buffer);
	if (!hard_start)
		return false;

	data = (unsigned long)xdp->data;
	if (data < hard_start)
		return false;

	data_meta = (unsigned long)xdp->data_meta;
	if (data_meta < hard_start || data_meta > data ||
	    xdp->meta_len != data - data_meta)
		return false;

	offset = data - hard_start;
	if (offset > span_len)
		return false;

	if (xdp->xdp && xdp_buff_has_frags(xdp->xdp)) {
		const struct skb_shared_info *sinfo =
			xdp_get_shared_info_from_buff(xdp->xdp);
		unsigned int linear_len =
			xdp->xdp->data_end - xdp->xdp->data;

		max_segs = rtlmq_rx_ring_uses_xsk_pool(ring) ?
			READ_ONCE(ring->ndev->xdp_zc_max_segs) :
			RTLMQ_XDP_MAX_SEGS;
		if (check_add_overflow((unsigned int)sinfo->nr_frags, 1U,
				       &seg_count) ||
		    check_mul_overflow(data_limit, max_segs, &max_len))
			return false;

		return linear_len <= span_len - offset &&
		       (rtlmq_rx_ring_uses_xsk_pool(ring) ||
			rtlmq_rx_skb_data_bounds_valid(ring, offset, linear_len)) &&
		       seg_count <= max_segs &&
		       xdp->len <= max_len;
	}

	if (!rtlmq_rx_ring_uses_xsk_pool(ring) &&
	    !rtlmq_rx_skb_data_bounds_valid(ring, offset, xdp->len))
		return false;

	return xdp->len <= span_len - offset;
}

static bool rtlmq_rx_csum_valid(const struct rtlmq_rx_desc_decode *decoded,
				struct rtlmq_rx_ring_stats *stats,
				struct rtlmq_rx_poll_hot_stats *hot)
{
	if (decoded->checksum_failed && hot)
		hot->csum_bad++;
	else if (decoded->checksum_failed && stats)
		stats->csum_bad++;
	if (decoded->checksum_valid) {
		if (hot)
			hot->csum_ok++;
		else if (stats)
			stats->csum_ok++;
	}

	return decoded->checksum_valid;
}

static void
rtlmq_rx_set_skb_checksum_fast(const struct rtlmq_rx_desc_decode *decoded,
			       struct rtlmq_rx_ring_stats *stats,
			       struct rtlmq_rx_poll_hot_stats *hot,
			       struct sk_buff *skb)
{
	skb->ip_summed = CHECKSUM_NONE;
	if (rtlmq_rx_csum_valid(decoded, stats, hot))
		skb->ip_summed = CHECKSUM_UNNECESSARY;
}

static void
rtlmq_rx_set_skb_checksum_status_fast(const struct rtlmq_rx_desc_decode *decoded,
				      struct sk_buff *skb)
{
	skb->ip_summed = CHECKSUM_NONE;
	if (decoded->checksum_valid) {
		skb->ip_summed = CHECKSUM_UNNECESSARY;
		return;
	}
}

static void rtlmq_rx_set_skb_checksum_checked(const struct rtlmq_rx_desc_decode *decoded,
					      bool rxcsum, struct rtlmq_rx_ring_stats *stats,
					      struct rtlmq_rx_poll_hot_stats *hot,
					      struct sk_buff *skb)
{
	if (!rxcsum) {
		skb->ip_summed = CHECKSUM_NONE;
		return;
	}

	rtlmq_rx_set_skb_checksum_fast(decoded, stats, hot, skb);
}

struct rtlmq_rx_rss_metadata {
	u32 hash;
	enum pkt_hash_types skb_type;
	enum xdp_rss_hash_type xdp_type;
	bool xdp_valid;
};

static bool rtlmq_rx_decode_rss(const struct rtlmq_rx_desc_decode *decoded,
				const struct rtlmq_rx_rss_xdp_map *xdp_map,
				struct rtlmq_rx_rss_metadata *metadata)
{
	u32 l3_info;
	u32 l4_info;

	if (!decoded || !metadata || !decoded->rss_valid)
		return false;

	l3_info = decoded->rss_l3_info;
	l4_info = decoded->rss_l4_info;
	metadata->hash = decoded->rss_hash;
	metadata->skb_type = l4_info ?
			     PKT_HASH_TYPE_L4 : PKT_HASH_TYPE_L3;

	/*
	 * Keep the skb path's broad L3/L4 classification. XDP metadata requires
	 * exactly one L3 class and, if present, one L4 protocol. Ordinary
	 * descriptor bits cannot identify IPv6 extension headers.
	 */
	if (!xdp_map)
		return true;

	if (l3_info == xdp_map->ipv4_mask) {
		if (!l4_info)
			metadata->xdp_type = XDP_RSS_TYPE_L3_IPV4;
		else if (l4_info == xdp_map->tcp_mask)
			metadata->xdp_type = XDP_RSS_TYPE_L4_IPV4_TCP;
		else if (l4_info == xdp_map->udp_mask)
			metadata->xdp_type = XDP_RSS_TYPE_L4_IPV4_UDP;
		else
			return true;
	} else if (l3_info == xdp_map->ipv6_mask) {
		if (!l4_info)
			metadata->xdp_type = XDP_RSS_TYPE_L3_IPV6;
		else if (l4_info == xdp_map->tcp_mask)
			metadata->xdp_type = XDP_RSS_TYPE_L4_IPV6_TCP;
		else if (l4_info == xdp_map->udp_mask)
			metadata->xdp_type = XDP_RSS_TYPE_L4_IPV6_UDP;
		else
			return true;
	} else {
		return true;
	}

	metadata->xdp_valid = true;
	return true;
}

static void
rtlmq_rx_set_skb_hash_enabled(const struct rtlmq_rx_desc_decode *decoded,
			      struct sk_buff *skb)
{
	struct rtlmq_rx_rss_metadata rss = {};

	if (!rtlmq_rx_decode_rss(decoded, NULL, &rss))
		return;

	skb_set_hash(skb, rss.hash, rss.skb_type);
}

static void rtlmq_rx_set_skb_hash(const struct net_device *ndev,
				  const struct rtlmq_rx_desc_decode *decoded,
				  struct sk_buff *skb)
{
	if (!(ndev->features & NETIF_F_RXHASH))
		return;

	rtlmq_rx_set_skb_hash_enabled(decoded, skb);
}

static bool
rtlmq_rx_set_skb_vlan_tag(struct sk_buff *skb,
			  const struct rtlmq_rx_desc_decode *decoded)
{
	if (!decoded->vlan_valid)
		return false;

	__vlan_hwaccel_put_tag(skb, htons(ETH_P_8021Q),
			       decoded->vlan_tci);
	return true;
}

static void
rtlmq_rx_decode_xdp_metadata(const struct rtlmq_rx_ring *ring,
			     const struct rtlmq_rx_desc_decode *decoded,
			     const ktime_t *rx_timestamp,
			     struct rtlmq_xdp_metadata_snapshot *snapshot)
{
	struct rtlmq_rx_rss_metadata rss = {};

	if (!ring || !ring->ndev || !decoded || !snapshot)
		return;

	if (rx_timestamp) {
		snapshot->rx_timestamp = ktime_to_ns(*rx_timestamp);
		snapshot->timestamp_valid = true;
	}

	if ((ring->ndev->features & NETIF_F_RXHASH) &&
	    rtlmq_rx_decode_rss(decoded,
				ring->cold ? &ring->cold->xdp_rss_map : NULL,
				&rss) &&
	    rss.xdp_valid) {
		snapshot->rss_hash = rss.hash;
		snapshot->rss_type = rss.xdp_type;
		snapshot->rss_valid = true;
	}

	if (!(ring->ndev->features & NETIF_F_HW_VLAN_CTAG_RX))
		return;

	if (decoded->vlan_valid) {
		snapshot->vlan_tci = decoded->vlan_tci;
		snapshot->vlan_valid = true;
	}
}

static int rtlmq_xdp_metadata_rx_timestamp(const struct xdp_md *ctx,
					   u64 *timestamp)
{
	const struct rtlmq_xdp_buff *xdp = (const void *)ctx;

	if (!timestamp)
		return -EINVAL;
	if (!xdp || !xdp->metadata.timestamp_valid)
		return -ENODATA;

	*timestamp = xdp->metadata.rx_timestamp;
	return 0;
}

static int rtlmq_xdp_metadata_rx_hash(const struct xdp_md *ctx, u32 *hash,
				      enum xdp_rss_hash_type *rss_type)
{
	const struct rtlmq_xdp_buff *xdp = (const void *)ctx;

	if (!hash || !rss_type)
		return -EINVAL;
	if (!xdp || !xdp->metadata.rss_valid)
		return -ENODATA;

	*hash = xdp->metadata.rss_hash;
	*rss_type = xdp->metadata.rss_type;
	return 0;
}

static int rtlmq_xdp_metadata_rx_vlan_tag(const struct xdp_md *ctx,
					  __be16 *vlan_proto, u16 *vlan_tci)
{
	const struct rtlmq_xdp_buff *xdp = (const void *)ctx;

	if (!vlan_proto || !vlan_tci)
		return -EINVAL;
	if (!xdp || !xdp->metadata.vlan_valid)
		return -ENODATA;

	*vlan_proto = htons(ETH_P_8021Q);
	*vlan_tci = xdp->metadata.vlan_tci;
	return 0;
}

static const struct xdp_metadata_ops rtlmq_xdp_metadata_policy[] = {
	[RTLMQ_XDP_METADATA_HASH | RTLMQ_XDP_METADATA_VLAN] = {
		.xmo_rx_hash = rtlmq_xdp_metadata_rx_hash,
		.xmo_rx_vlan_tag = rtlmq_xdp_metadata_rx_vlan_tag,
	},
	[RTLMQ_XDP_METADATA_TIMESTAMP | RTLMQ_XDP_METADATA_HASH |
	 RTLMQ_XDP_METADATA_VLAN] = {
		.xmo_rx_timestamp = rtlmq_xdp_metadata_rx_timestamp,
		.xmo_rx_hash = rtlmq_xdp_metadata_rx_hash,
		.xmo_rx_vlan_tag = rtlmq_xdp_metadata_rx_vlan_tag,
	},
};

const struct xdp_metadata_ops *
rtlmq_xdp_metadata_ops_for_capabilities(u8 capabilities)
{
	if (!capabilities || capabilities >= ARRAY_SIZE(rtlmq_xdp_metadata_policy))
		return NULL;

	return &rtlmq_xdp_metadata_policy[capabilities];
}

static void rtlmq_rx_count_drop(struct rtlmq_rx_ring_stats *stats,
				struct rtlmq_rx_poll_hot_stats *hot,
				enum rtlmq_rx_drop_reason reason)
{
	if (hot) {
		hot->drops++;
		hot->drop_reason[reason]++;
	} else if (stats) {
		stats->drops++;
		stats->drop_reason[reason]++;
	}
}

/* Fast success accounting stays batched; only a failure touches these totals. */
static void rtlmq_rx_fast_drop_reason(struct rtlmq_rx_ring *ring,
				      enum rtlmq_rx_drop_reason reason)
{
	struct rtlmq_rx_ring_stats *stats = &ring->cold->stats;

	rtlmq_rx_stats_update_begin(ring);
	stats->drops++;
	stats->drop_reason[reason]++;
	if (reason == RTLMQ_RX_DROP_SKB)
		stats->alloc_fail++;
	rtlmq_rx_stats_update_end(ring);
}

static struct sk_buff *rtlmq_rx_build_skb_from_buffer(struct rtlmq_rx_ring *ring,
						      struct rtlmq_rx_buffer *buffer,
						      void *hard_start, unsigned int data_offset,
						      unsigned int pkt_size,
						      struct rtlmq_rx_ring_stats *stats,
						      struct rtlmq_rx_poll_hot_stats *hot)
{
	struct sk_buff *skb;

	if (unlikely(!hard_start)) {
		rtlmq_rx_count_drop(stats, hot, RTLMQ_RX_DROP_LENGTH);
		rtlmq_recycle_rx_buffer_napi(ring, buffer);
		return NULL;
	}

	skb = napi_build_skb(hard_start, ring->alloc_size);
	if (unlikely(!skb)) {
		rtlmq_rx_count_drop(stats, hot, RTLMQ_RX_DROP_SKB);
		if (hot)
			hot->alloc_fail++;
		else if (stats)
			stats->alloc_fail++;
		rtlmq_recycle_rx_buffer_napi(ring, buffer);
		return NULL;
	}

	skb_reserve(skb, data_offset);
	__skb_put(skb, pkt_size);
	skb_mark_for_recycle(skb);

	return skb;
}

static int rtlmq_rx_replace_buffer_for_handoff(struct rtlmq_rx_ring *ring,
					       struct rtlmq_rx_buffer *slot,
					       struct rtlmq_rx_buffer *original)
{
	struct rtlmq_rx_buffer replacement = {};
	int ret;

	ret = rtlmq_alloc_rx_buffer(ring, &replacement,
				    GFP_ATOMIC | __GFP_NOWARN);
	if (ret)
		return ret;

	if (original)
		*original = *slot;
	*slot = replacement;
	return 0;
}

static struct sk_buff *
rtlmq_rx_build_skb_recycle(struct rtlmq_rx_ring *ring, struct rtlmq_rx_buffer *buffer,
			   const struct rtlmq_rx_desc_decode *decoded, const void *data,
			   const void *data_meta, unsigned int pkt_size, unsigned int meta_len,
			   bool dma_cpu_owned, bool rxcsum, struct rtlmq_rx_ring_stats *stats,
			   struct rtlmq_rx_poll_hot_stats *hot)
{
	struct rtlmq_rx_buffer original;
	unsigned int data_offset;
	struct sk_buff *skb;
	void *hard_start;

	if (rtlmq_rx_replace_buffer_for_handoff(ring, buffer, &original)) {
		rtlmq_rx_count_drop(stats, hot, RTLMQ_RX_DROP_RESOURCE);
		if (hot)
			hot->alloc_fail++;
		else if (stats)
			stats->alloc_fail++;
		rtlmq_reuse_rx_buffer(ring, buffer);
		return NULL;
	}

	hard_start = rtlmq_rx_buffer_hard_start(ring, &original);
	if (unlikely(!hard_start))
		goto drop_length;

	if (unlikely(!data || (const u8 *)data < (const u8 *)hard_start))
		goto drop_length;

	data_offset = (const u8 *)data - (const u8 *)hard_start;
	if (unlikely(!rtlmq_rx_skb_data_bounds_valid(ring, data_offset,
						     pkt_size)))
		goto drop_length;

	if (!dma_cpu_owned)
		rtlmq_sync_rx_buffer_for_cpu(ring, &original, pkt_size);

	skb = rtlmq_rx_build_skb_from_buffer(ring, &original, hard_start,
					     data_offset, pkt_size, stats, hot);
	if (!skb)
		return NULL;
	if (meta_len) {
		if (unlikely(!data_meta || meta_len > skb_headroom(skb) ||
			     (const u8 *)data_meta + meta_len != data)) {
			rtlmq_rx_count_drop(stats, hot, RTLMQ_RX_DROP_LENGTH);
			dev_kfree_skb_any(skb);
			return NULL;
		}
		skb_metadata_set(skb, meta_len);
	}
	rtlmq_rx_set_skb_checksum_checked(decoded, rxcsum, stats, hot, skb);

	return skb;

drop_length:
	rtlmq_rx_count_drop(stats, hot, RTLMQ_RX_DROP_LENGTH);
	rtlmq_recycle_rx_buffer_napi(ring, &original);
	return NULL;
}

static struct sk_buff *
rtlmq_rx_build_copy_skb(struct napi_struct *napi,
			const struct xdp_buff *xdp, const struct rtlmq_rx_desc_decode *decoded,
			const void *data, const void *data_meta, unsigned int pkt_size,
			unsigned int meta_len, bool rxcsum, struct rtlmq_rx_ring_stats *stats,
			struct rtlmq_rx_poll_hot_stats *hot)
{
	struct sk_buff *skb;

	skb = rtlmq_rx_copy_skb(napi, xdp, data, data_meta, pkt_size, meta_len);
	if (IS_ERR(skb)) {
		rtlmq_rx_count_drop(stats, hot, PTR_ERR(skb) == -ENOMEM ?
			RTLMQ_RX_DROP_SKB : RTLMQ_RX_DROP_LENGTH);
		if (PTR_ERR(skb) == -ENOMEM) {
			if (hot)
				hot->alloc_fail++;
			else if (stats)
				stats->alloc_fail++;
		}
		return NULL;
	}

	rtlmq_rx_set_skb_checksum_checked(decoded, rxcsum, stats, hot, skb);

	return skb;
}

static struct sk_buff *rtlmq_rx_build_skb_fast(struct rtlmq_rx_ring *ring,
					       struct rtlmq_rx_buffer *buffer,
					       unsigned int pkt_size)
{
	struct rtlmq_rx_buffer original = *buffer;
	struct sk_buff *skb;
	void *hard_start = original.hard_start;

	if (unlikely(!hard_start ||
		     !rtlmq_rx_skb_data_bounds_valid(ring, RTLMQ_RX_HEADROOM,
						    pkt_size))) {
		rtlmq_rx_fast_drop_reason(ring, RTLMQ_RX_DROP_LENGTH);
		rtlmq_recycle_rx_buffer_napi(ring, buffer);
		return NULL;
	}

	/*
	 * The normal skb fast path consumes completed RX buffers directly and
	 * leaves the ring slot empty. Batch refill repopulates and republishes
	 * those slots after the poll instead of paying replacement allocation
	 * inside the per-packet loop.
	 */
	rtlmq_rx_clear_buffer_fast(buffer);
	rtlmq_sync_rx_page_pool_buffer_for_cpu(ring, &original, pkt_size);

	skb = napi_build_skb(hard_start, ring->alloc_size);
	if (unlikely(!skb)) {
		rtlmq_rx_fast_drop_reason(ring, RTLMQ_RX_DROP_SKB);
		rtlmq_recycle_rx_buffer_napi(ring, &original);
		return NULL;
	}

	skb_reserve(skb, RTLMQ_RX_HEADROOM);
	__skb_put(skb, pkt_size);
	skb_mark_for_recycle(skb);

	return skb;
}

static bool rtlmq_xdp_metadata_bounds_valid(const struct xdp_buff *xdp)
{
	unsigned long data_meta;
	unsigned long frame_end;
	unsigned long meta_len;
	unsigned long data;

	if (!xdp || !xdp->data_hard_start || !xdp->data ||
	    !xdp->data_meta)
		return false;

	data = (unsigned long)xdp->data;
	data_meta = (unsigned long)xdp->data_meta;
	frame_end = (unsigned long)xdp->data_hard_start +
		    sizeof(struct xdp_frame);
	if (data < frame_end)
		return false;

	/*
	 * xdp_prepare_buff(..., false) represents "no metadata" with the
	 * core sentinel data_meta == data + 1. It is not an invalid BPF
	 * adjustment and is normalized to zero metadata below. Reject every
	 * other pointer beyond data as invalid.
	 */
	if (xdp_data_meta_unsupported(xdp))
		return data_meta == data + 1;

	if (data_meta < frame_end)
		return false;

	meta_len = data - data_meta;
	return !xdp_metalen_invalid(meta_len);
}

static u32 rtlmq_finalize_xdp_action(struct rtlmq_private *tp,
				     struct bpf_prog *xdp_prog,
				     struct xdp_buff *xdp,
				     struct rtlmq_xdp_result *result, u32 act)
{
	switch (act) {
	case XDP_PASS:
	case XDP_TX:
	case XDP_REDIRECT:
		if (unlikely(!rtlmq_xdp_metadata_bounds_valid(xdp))) {
			trace_xdp_exception(tp->ndev, xdp_prog, XDP_ABORTED);
			return XDP_ABORTED;
		}

		result->data = xdp->data;
		if (xdp_data_meta_unsupported(xdp)) {
			result->data_meta = xdp->data;
			result->meta_len = 0;
		} else {
			result->data_meta = xdp->data_meta;
			result->meta_len = xdp->data - xdp->data_meta;
		}
		result->len = xdp_get_buff_len(xdp);
		return act;
	case XDP_DROP:
	case XDP_ABORTED:
		if (act == XDP_ABORTED)
			trace_xdp_exception(tp->ndev, xdp_prog, act);
		return act;
	default:
		bpf_warn_invalid_xdp_action(tp->ndev, xdp_prog, act);
		trace_xdp_exception(tp->ndev, xdp_prog, act);
		return XDP_ABORTED;
	}
}

static u32 rtlmq_run_xdp(struct rtlmq_rx_ring *ring,
			 struct rtlmq_rx_buffer *buffer,
			 const struct rtlmq_rx_desc_decode *decoded,
			 const ktime_t *rx_timestamp, unsigned int pkt_size,
			 struct xdp_buff *prepared_xdp, bool dma_cpu_owned,
			 struct rtlmq_xdp_result *result)
{
	struct rtlmq_private *tp = ring->priv;
	struct bpf_prog *xdp_prog = READ_ONCE(tp->xdp_prog);
	struct rtlmq_xdp_buff *context;
	struct xdp_buff *xdp;
	bool uses_xsk = rtlmq_rx_ring_uses_xsk_pool(ring);
	unsigned char *hard_start;
	u32 act;

	result->dma_cpu_owned = dma_cpu_owned;
	result->detached = false;
	result->prog = xdp_prog;
	result->xdp = NULL;
	result->data_meta = NULL;
	result->meta_len = 0;

	if (uses_xsk) {
		XSK_CHECK_PRIV_TYPE(struct rtlmq_xdp_buff);
		xdp = buffer->xdp;
		if (!xdp)
			return XDP_ABORTED;
		context = container_of(xdp, struct rtlmq_xdp_buff, xdp);
		memset(&context->metadata, 0, sizeof(context->metadata));
		rtlmq_rx_decode_xdp_metadata(ring, decoded, rx_timestamp,
					     &context->metadata);
		if (!xdp_buff_has_frags(xdp)) {
			if (!result->dma_cpu_owned) {
				rtlmq_sync_rx_buffer_for_cpu(ring, buffer, pkt_size);
				result->dma_cpu_owned = true;
			}
			xsk_buff_set_size(xdp, pkt_size);
		}
		result->xdp = xdp;
		result->data = xdp->data;
		result->data_meta = xdp->data;
		result->len = pkt_size;

		/*
		 * Pool binding and XDP-program attachment are independent core
		 * operations. Before an XSKMAP program is attached (or after it
		 * is detached), preserve normal netdev delivery through the
		 * existing XSK-to-skb PASS path. AF_XDP TX remains usable too.
		 */
		if (!xdp_prog)
			return XDP_PASS;
	} else {
		context = prepared_xdp ?
			container_of(prepared_xdp, struct rtlmq_xdp_buff, xdp) :
			&result->local_xdp;
		xdp = &context->xdp;
		memset(&context->metadata, 0, sizeof(context->metadata));
		hard_start = rtlmq_rx_buffer_hard_start(ring, buffer);
		result->xdp = prepared_xdp;
		result->data = prepared_xdp ? xdp->data :
			hard_start + RTLMQ_RX_HEADROOM;
		result->data_meta = result->data;
		result->len = prepared_xdp ? xdp_get_buff_len(xdp) : pkt_size;

		if (!xdp_prog)
			return XDP_PASS;

		if (!prepared_xdp) {
			if (!result->dma_cpu_owned)
				rtlmq_sync_rx_buffer_for_cpu(ring, buffer, pkt_size);
			result->dma_cpu_owned = true;
			xdp_init_buff(xdp, ring->alloc_size,
				      &ring->cold->xdp_rxq);
			/* Empty metadata is supported; the sentinel forbids adjust_meta. */
			xdp_prepare_buff(xdp, hard_start, RTLMQ_RX_HEADROOM,
					 pkt_size, true);
		}
		result->xdp = xdp;
		rtlmq_rx_decode_xdp_metadata(ring, decoded, rx_timestamp,
					     &context->metadata);
	}

	if (unlikely(xdp_buff_has_frags(xdp) &&
		     !xdp_prog->aux->xdp_has_frags))
		act = XDP_DROP;
	else
		act = bpf_prog_run_xdp(xdp_prog, xdp);
	return rtlmq_finalize_xdp_action(tp, xdp_prog, xdp, result, act);
}

static u16 rtlmq_xdp_tx_queue_index(const struct rtlmq_private *tp,
				    const struct rtlmq_rx_ring *rx_ring)
{
	u16 published_txq;

	if (!tp || !rx_ring ||
	    rx_ring->queue_index >= rtlmq_published_num_rx_queues(tp))
		return U16_MAX;

	published_txq = rtlmq_published_num_tx_queues(tp);
	if (!is_power_of_2(published_txq) ||
	    published_txq > RTLMQ_IMPL_MAX_TX_QUEUES)
		return U16_MAX;

	return rx_ring->queue_index & (published_txq - 1);
}

static int rtlmq_xdp_tx_frame(struct rtlmq_rx_ring *rx_ring,
			      struct rtlmq_rx_buffer *buffer,
			      struct rtlmq_xdp_result *xdp)
{
	struct rtlmq_private *tp = rx_ring->priv;
	struct xdp_buff *xdp_buff;
	u16 queue_index = rtlmq_xdp_tx_queue_index(tp, rx_ring);
	struct rtlmq_vector *vector = NULL;
	enum rtlmq_tx_publish_result ret;
	struct rtlmq_tx_ring *tx_ring;
	struct netdev_queue *nq;
	struct xdp_frame *xdpf;
	u16 vector_index;
	bool dma_map;

	if (unlikely(!xdp || !xdp->xdp)) {
		rtlmq_recycle_rx_buffer_napi(rx_ring, buffer);
		return -EINVAL;
	}
	xdp_buff = xdp->xdp;

	if (unlikely(queue_index >= rtlmq_published_num_tx_queues(tp)))
		goto drop_buffer_enodev;
	tx_ring = &tp->tx_rings[queue_index];
	nq = tx_ring->ndev_queue;
	if (unlikely(!tx_ring->desc || !tx_ring->infos ||
		     !tx_ring->completions || !nq))
		goto drop_buffer_enodev;

	vector_index = READ_ONCE(rx_ring->vector_index);
	if (likely(tp->vectors &&
		   vector_index < rtlmq_published_num_vectors(tp)))
		vector = &tp->vectors[vector_index];

	if (rtlmq_rx_ring_uses_xsk_pool(rx_ring) &&
	    xdp_buff_has_frags(xdp_buff)) {
		/*
		 * The core ZC converter copies only the linear head, then frees
		 * the entire XSK chain. Copy the complete packet before releasing
		 * that chain; conversion failure leaves it owned by the drop path.
		 */
		xdpf = rtlmq_rx_copy_xdp_frame(xdp_buff);
		if (xdpf)
			xsk_buff_free(xdp_buff);
	} else {
		xdpf = xdp_convert_buff_to_frame(xdp_buff);
	}
	if (unlikely(!xdpf))
		goto drop_buffer_enomem;

	dma_map = rtlmq_rx_ring_uses_xsk_pool(rx_ring) ||
		  xdpf->mem_type != MEM_TYPE_PAGE_POOL;
	/*
	 * Use the BH variant even from NAPI: threaded callers may migrate
	 * before locking, so the lock helper must pin the CPU itself.
	 */
	__netif_tx_lock_bh(nq);
	txq_trans_cond_update(nq);
	ret = rtlmq_publish_tx_xdp_frame(tx_ring, xdpf,
					 RTLMQ_TX_BUFFER_XDP_RX,
					 dma_map, !vector);
	if (ret == RTLMQ_TX_PUBLISHED && vector)
		WRITE_ONCE(vector->xdp_tx_pending_mask,
			   READ_ONCE(vector->xdp_tx_pending_mask) |
			   BIT(queue_index));
	__netif_tx_unlock_bh(nq);

	if (ret == RTLMQ_TX_PUBLISHED)
		return 0;

	xdp_return_frame_rx_napi(xdpf);
	return ret == RTLMQ_TX_BUSY ? -ENOSPC : -EIO;

drop_buffer_enodev:
	if (xdp_buff_has_frags(xdp_buff))
		xdp_return_buff(xdp_buff);
	else
		rtlmq_recycle_rx_buffer_napi(rx_ring, buffer);
	return -ENODEV;

drop_buffer_enomem:
	if (xdp_buff_has_frags(xdp_buff))
		xdp_return_buff(xdp_buff);
	else
		rtlmq_recycle_rx_buffer_napi(rx_ring, buffer);
	return -ENOMEM;
}

static inline int rtlmq_xdp_redirect(struct rtlmq_rx_ring *ring,
				     struct xdp_buff *xdp,
				     struct bpf_prog *prog)
{
	bool uses_xsk = rtlmq_rx_ring_uses_xsk_pool(ring);
	struct xdp_frame *frame;
	int ret;

	/*
	 * Consume the source on every outcome. Non-XSKMAP targets need an
	 * independent frame: core ZC conversion truncates fragments and frees
	 * the source before enqueue can fail, hiding the clone from cleanup.
	 * XSKMAP keeps its existing zero-copy path and ownership contract.
	 */
	if (uses_xsk &&
	    bpf_net_ctx_get_ri()->map_type != BPF_MAP_TYPE_XSKMAP) {
		frame = rtlmq_rx_copy_xdp_frame(xdp);
		/* NULL must also reach core so it consumes this redirect request. */
		ret = xdp_do_redirect_frame(ring->ndev, xdp, frame, prog);
		if (ret && frame)
			xdp_return_frame_rx_napi(frame);
		xsk_buff_free(xdp);
		return ret;
	}

	ret = xdp_do_redirect(ring->ndev, xdp, prog);
	/*
	 * XSKMAP's xsk_rcv_zc() returns the source before reporting RX-ring
	 * exhaustion. Other errors leave it with this redirect owner.
	 */
	if (!ret || (uses_xsk && ret == -ENOBUFS))
		return ret;
	xdp_return_buff(xdp);
	return ret;
}

static int rtlmq_poll_rx_ring_generic(struct rtlmq_vector *vector, struct rtlmq_rx_ring *ring,
				      int budget, enum rtlmq_rx_clean_exit_reason *exit_reason_out);

enum rtlmq_xsk_mb_result {
	RTLMQ_XSK_MB_ERROR = -1,
	RTLMQ_XSK_MB_CONTINUE,
	RTLMQ_XSK_MB_COMPLETE,
};

enum rtlmq_page_mb_result {
	RTLMQ_PAGE_MB_ERROR = -1,
	RTLMQ_PAGE_MB_CONTINUE,
	RTLMQ_PAGE_MB_COMPLETE,
};

static void rtlmq_rx_mb_timestamp_reset(struct rtlmq_rx_ring_cold *cold)
{
	cold->mb_rx_timestamp = 0;
	cold->mb_rx_timestamp_valid = false;
}

static void rtlmq_rx_mb_timestamp_add(struct rtlmq_rx_ring_cold *cold,
				      const ktime_t *rx_timestamp)
{
	if (!rx_timestamp)
		return;

	cold->mb_rx_timestamp = *rx_timestamp;
	cold->mb_rx_timestamp_valid = true;
}

static bool rtlmq_rx_mb_timestamp_complete(struct rtlmq_rx_ring_cold *cold,
					   ktime_t *rx_timestamp)
{
	bool valid = cold->mb_rx_timestamp_valid;

	if (valid && rx_timestamp)
		*rx_timestamp = cold->mb_rx_timestamp;
	rtlmq_rx_mb_timestamp_reset(cold);
	return valid;
}

static void rtlmq_xsk_mb_discard(struct rtlmq_rx_ring *ring)
{
	struct rtlmq_rx_ring_cold *cold = ring->cold;

	if (cold->xsk_mb_head)
		xsk_buff_free(cold->xsk_mb_head);
	cold->xsk_mb_head = NULL;
	cold->xsk_mb_len = 0;
	cold->xsk_mb_segs = 0;
	rtlmq_rx_mb_timestamp_reset(cold);
}

static void rtlmq_page_mb_discard(struct rtlmq_rx_ring *ring)
{
	struct rtlmq_rx_ring_cold *cold = ring->cold;
	struct xdp_buff *xdp = &cold->page_mb_xdp.xdp;

	rtlmq_release_rx_page_packet(ring, &cold->page_mb_head, xdp, false);
	memset(&cold->page_mb_xdp, 0, sizeof(cold->page_mb_xdp));
	cold->page_mb_len = 0;
	cold->page_mb_segs = 0;
	rtlmq_rx_mb_timestamp_reset(cold);
}

void rtlmq_release_rx_multibuffer(struct rtlmq_rx_ring *ring)
{
	if (!ring || !ring->cold)
		return;
	if (ring->cold->xsk_mb_head)
		rtlmq_xsk_mb_discard(ring);
	if (ring->cold->page_mb_head.opaque)
		rtlmq_page_mb_discard(ring);
	ring->cold->mb_discarding = false;
	ring->cold->packet_errors = 0;
}

static bool rtlmq_rx_discard_suffix(struct rtlmq_rx_ring *ring,
				    const struct rtlmq_rx_desc_decode *decoded)
{
	if (!ring->cold->mb_discarding)
		return false;
	if (decoded->first_frag) {
		ring->cold->mb_discarding = false;
		return false;
	}
	ring->cold->mb_discarding = !decoded->last_frag;
	return true;
}

static void rtlmq_rx_drop_chain(struct rtlmq_rx_ring *ring,
				const struct rtlmq_rx_desc_decode *decoded)
{
	u8 packet_errors = ring->cold->packet_errors;

	rtlmq_release_rx_multibuffer(ring);
	ring->cold->mb_discarding = !decoded->last_frag;
	ring->cold->packet_errors = packet_errors;
}

/*
 * Trim bytes already attached to a multibuffer packet without touching page
 * ownership. This is only used for up to three FCS bytes preceding an
 * FCS-only final descriptor. A valid non-final descriptor is much larger;
 * reject malformed short state instead of manufacturing an empty segment.
 */
static bool rtlmq_rx_multibuffer_trim_tail(struct xdp_buff *xdp,
					   unsigned int trim)
{
	unsigned int linear_len;

	if (!trim)
		return true;
	if (!xdp)
		return false;

	if (xdp_buff_has_frags(xdp)) {
		struct skb_shared_info *sinfo =
			xdp_get_shared_info_from_buff(xdp);
		skb_frag_t *frag;

		if (!sinfo->nr_frags)
			return false;
		frag = &sinfo->frags[sinfo->nr_frags - 1];
		if (skb_frag_size(frag) <= trim ||
		    sinfo->xdp_frags_size < trim)
			return false;
		skb_frag_size_sub(frag, trim);
		sinfo->xdp_frags_size -= trim;
		return true;
	}

	linear_len = xdp->data_end - xdp->data;
	if (linear_len <= trim)
		return false;
	xdp->data_end -= trim;
	return true;
}

static enum rtlmq_page_mb_result
rtlmq_page_mb_take_fragment(struct rtlmq_rx_ring *ring, struct rtlmq_rx_buffer *buffer,
			    unsigned int pkt_size, const struct rtlmq_rx_desc_decode *decoded,
			    bool strip_fcs, const ktime_t *rx_timestamp,
			    struct rtlmq_rx_buffer *packet, struct xdp_buff **packet_xdp,
			    ktime_t *packet_timestamp, bool *packet_timestamp_valid)
{
	struct rtlmq_rx_ring_cold *cold;
	struct rtlmq_rx_buffer original;
	unsigned int total_len;

	if (!ring || !ring->cold || !ring->page_pool || !buffer ||
	    !buffer->page || !decoded || !packet || !packet_xdp)
		return RTLMQ_PAGE_MB_ERROR;
	cold = ring->cold;
	if ((decoded->first_frag && cold->page_mb_head.opaque) ||
	    (!decoded->first_frag && !cold->page_mb_head.opaque) ||
	    cold->page_mb_segs >= RTLMQ_XDP_MAX_SEGS)
		goto malformed;
	if (strip_fcs && decoded->last_frag && !pkt_size) {
		unsigned int trim;

		if (decoded->first_frag ||
		    decoded->packet_len > ETH_FCS_LEN)
			goto malformed;
		trim = ETH_FCS_LEN - decoded->packet_len;
		if (cold->page_mb_len <= trim ||
		    !rtlmq_rx_multibuffer_trim_tail(&cold->page_mb_xdp.xdp, trim))
			goto malformed;
		original = *buffer;
		rtlmq_rx_clear_buffer_fast(buffer);
		rtlmq_recycle_rx_buffer_napi(ring, &original);
		cold->page_mb_len -= trim;
		rtlmq_rx_mb_timestamp_add(cold, rx_timestamp);
		goto complete;
	}
	if (!rtlmq_rx_multibuffer_add_len(decoded->first_frag ? 0 : cold->page_mb_len,
					  pkt_size, RTLMQ_RX_HW_FRAME_SIZE, &total_len))
		goto malformed;
	if (decoded->first_frag)
		rtlmq_rx_mb_timestamp_reset(cold);
	rtlmq_rx_mb_timestamp_add(cold, rx_timestamp);

	original = *buffer;
	rtlmq_rx_clear_buffer_fast(buffer);
	rtlmq_sync_rx_buffer_for_cpu(ring, &original, pkt_size);

	if (decoded->first_frag) {
		struct xdp_buff *xdp = &cold->page_mb_xdp.xdp;

		memset(&cold->page_mb_xdp, 0, sizeof(cold->page_mb_xdp));
		xdp_init_buff(xdp, ring->alloc_size, &cold->xdp_rxq);
		xdp_prepare_buff(xdp, original.hard_start, RTLMQ_RX_HEADROOM,
				 pkt_size, true);
		cold->page_mb_head = original;
		cold->page_mb_len = total_len;
		cold->page_mb_segs = 1;
		return RTLMQ_PAGE_MB_CONTINUE;
	}

	if (!xdp_buff_add_frag(&cold->page_mb_xdp.xdp,
			       page_to_netmem(original.page),
			       RTLMQ_RX_HEADROOM, pkt_size,
			       ring->alloc_size))
		goto moved_malformed;
	cold->page_mb_len = total_len;
	cold->page_mb_segs++;
	if (!decoded->last_frag)
		return RTLMQ_PAGE_MB_CONTINUE;

complete:
	*packet = cold->page_mb_head;
	*packet_xdp = &cold->page_mb_xdp.xdp;
	memset(&cold->page_mb_head, 0, sizeof(cold->page_mb_head));
	cold->page_mb_len = 0;
	cold->page_mb_segs = 0;
	if (packet_timestamp_valid)
		*packet_timestamp_valid = rtlmq_rx_mb_timestamp_complete(cold, packet_timestamp);
	else
		rtlmq_rx_mb_timestamp_reset(cold);
	return RTLMQ_PAGE_MB_COMPLETE;

moved_malformed:
	rtlmq_recycle_rx_buffer_napi(ring, &original);
malformed:
	rtlmq_page_mb_discard(ring);
	return RTLMQ_PAGE_MB_ERROR;
}

static enum rtlmq_xsk_mb_result
rtlmq_xsk_mb_take_fragment(struct rtlmq_rx_ring *ring, struct rtlmq_rx_buffer *buffer,
			   unsigned int pkt_size, const struct rtlmq_rx_desc_decode *decoded,
			   bool strip_fcs, const ktime_t *rx_timestamp,
			   struct rtlmq_rx_buffer *packet,
			   ktime_t *packet_timestamp, bool *packet_timestamp_valid)
{
	struct rtlmq_rx_ring_cold *cold;
	struct rtlmq_rx_buffer original;
	unsigned int total_len;
	unsigned int max_segs;
	unsigned int max_len;
	bool first;
	bool last;

	if (!ring || !decoded)
		return RTLMQ_XSK_MB_ERROR;
	first = decoded->first_frag;
	last = decoded->last_frag;
	cold = ring->cold;
	if (!ring->ndev || !cold || !cold->xsk_pool ||
	    !cold->xsk_pool->umem ||
	    !(cold->xsk_pool->umem->flags & XDP_UMEM_SG_FLAG))
		return RTLMQ_XSK_MB_ERROR;
	max_segs = READ_ONCE(ring->ndev->xdp_zc_max_segs);
	if (!max_segs ||
	    check_mul_overflow(ring->buffer_len, max_segs, &max_len))
		goto malformed;
	max_len = min_t(unsigned int, max_len, RTLMQ_RX_HW_FRAME_SIZE);
	if ((first && cold->xsk_mb_head) ||
	    (!first && !cold->xsk_mb_head) ||
	    cold->xsk_mb_segs >= max_segs)
		goto malformed;
	if (strip_fcs && last && !pkt_size) {
		unsigned int trim;

		if (first || decoded->packet_len > ETH_FCS_LEN)
			goto malformed;
		trim = ETH_FCS_LEN - decoded->packet_len;
		if (cold->xsk_mb_len <= trim ||
		    !rtlmq_rx_multibuffer_trim_tail(cold->xsk_mb_head, trim))
			goto malformed;
		/* XSK redirect reads the pool tail, not the shared-info length. */
		if (xdp_buff_has_frags(cold->xsk_mb_head))
			xsk_buff_get_tail(cold->xsk_mb_head)->data_end -= trim;
		original = *buffer;
		rtlmq_rx_clear_buffer_fast(buffer);
		xsk_buff_free(original.xdp);
		cold->xsk_mb_len -= trim;
		rtlmq_rx_mb_timestamp_add(cold, rx_timestamp);
		goto complete;
	}
	if (!rtlmq_rx_multibuffer_add_len(first ? 0 : cold->xsk_mb_len,
					  pkt_size, max_len, &total_len))
		goto malformed;
	if (first)
		rtlmq_rx_mb_timestamp_reset(cold);
	rtlmq_rx_mb_timestamp_add(cold, rx_timestamp);

	original = *buffer;
	rtlmq_rx_clear_buffer_fast(buffer);
	rtlmq_sync_rx_buffer_for_cpu(ring, &original, pkt_size);
	xsk_buff_set_size(original.xdp, pkt_size);

	if (first) {
		cold->xsk_mb_head = original.xdp;
		cold->xsk_mb_len = total_len;
		cold->xsk_mb_segs = 1;
		return RTLMQ_XSK_MB_CONTINUE;
	}

	if (!xsk_buff_add_frag(cold->xsk_mb_head, original.xdp)) {
		xsk_buff_free(original.xdp);
		goto malformed;
	}
	cold->xsk_mb_len = total_len;
	cold->xsk_mb_segs++;
	if (!last)
		return RTLMQ_XSK_MB_CONTINUE;

complete:
	packet->xdp = cold->xsk_mb_head;
	packet->dma_addr = xsk_buff_xdp_get_dma(packet->xdp);
	packet->hard_start = packet->xdp->data_hard_start;
	cold->xsk_mb_head = NULL;
	cold->xsk_mb_len = 0;
	cold->xsk_mb_segs = 0;
	if (packet_timestamp_valid)
		*packet_timestamp_valid = rtlmq_rx_mb_timestamp_complete(cold, packet_timestamp);
	else
		rtlmq_rx_mb_timestamp_reset(cold);
	return RTLMQ_XSK_MB_COMPLETE;

malformed:
	rtlmq_xsk_mb_discard(ring);
	return RTLMQ_XSK_MB_ERROR;
}

static void rtlmq_rx_publish_clean_exit_reason(enum rtlmq_rx_clean_exit_reason *exit_reason_out,
					       enum rtlmq_rx_clean_exit_reason exit_reason)
{
	if (exit_reason_out)
		*exit_reason_out = exit_reason;
}

static int rtlmq_rx_finish_skb_fast_empty(struct rtlmq_rx_ring *ring,
					  enum rtlmq_rx_clean_exit_reason
						  *exit_reason_out)
{
	enum rtlmq_rx_clean_exit_reason exit_reason =
		RTLMQ_RX_CLEAN_EXIT_HW_OWNED;
	unsigned int refill_pending;

	refill_pending = rtlmq_rx_refill_pending(ring);
	if (unlikely(refill_pending)) {
		exit_reason = rtlmq_rx_refill_fast_clean_exit(ring, refill_pending,
							      exit_reason
		);
	}
	rtlmq_rx_publish_clean_exit_reason(exit_reason_out, exit_reason);
	return 0;
}

static bool rtlmq_rx_skb_fast_prepare_next_desc(struct rtlmq_rx_ring *ring, u8 *desc_base,
						u16 desc_stride, unsigned int next_to_clean,
						unsigned int *entry, void **desc, u32 *status,
						enum rtlmq_rx_clean_exit_reason *exit_reason)
{
	*entry = next_to_clean;
	*desc = desc_base + *entry * desc_stride;
	if (!ring->rx_posted ||
	    !rtlmq_rx_desc_acquire(ring, *desc, status)) {
		*exit_reason = RTLMQ_RX_CLEAN_EXIT_HW_OWNED;
		return false;
	}
	return true;
}

static enum rtlmq_rx_clean_exit_reason
rtlmq_rx_finish_skb_fast_poll(struct rtlmq_rx_ring *ring, struct rtlmq_rx_ring_stats *stats,
			      unsigned int next_to_clean, const struct rtlmq_rx_fast_batch *batch,
			      int work_done, enum rtlmq_rx_clean_exit_reason exit_reason)
{
	unsigned int refill_pending;

	refill_pending = rtlmq_rx_refill_pending(ring);
	/*
	 * Advance next_to_clean before refill/accounting lets RX-ready sample
	 * the ring again. It must not inspect already-consumed descriptors.
	 */
	ring->next_to_clean = next_to_clean;
	exit_reason = rtlmq_rx_refill_fast_clean_exit(ring, refill_pending,
						      exit_reason
	);

	rtlmq_rx_stats_update_begin(ring);
	rtlmq_rx_publish_fast_batch(ring, stats, batch);
	rtlmq_rx_stats_update_end(ring);
	if (trace_rtlmq_rx_result_enabled() &&
	    (batch->rx_packets || work_done != batch->rx_packets))
		trace_rtlmq_rx_result(ring->ndev->name, ring->queue_index,
				      READ_ONCE(ring->priv->active_resource_generation), true,
				      batch->rx_packets, work_done - batch->rx_packets);

	return exit_reason;
}

static int
rtlmq_poll_rx_ring_skb_fast_path(struct rtlmq_vector *vector,
				 struct rtlmq_rx_ring *ring,
				 int budget,
				 bool skip_entry_own_mmio,
				 enum rtlmq_rx_clean_exit_reason *exit_reason_out)
{
	struct rtlmq_rx_ring_stats *stats;
	struct rtlmq_rx_buffer *buffers;
	struct napi_struct *napi;
	struct net_device *ndev;
	const struct rtlmq_rx_desc_profile *profile = ring->profile;
	u8 *desc_base = ring->desc;
	u16 desc_stride = profile->stride;
	unsigned int desc_decode_flags = 0;
	u32 rx_fast_flags;
	u32 fcs_adjust;
	unsigned int next_to_clean = ring->next_to_clean;
	unsigned int ring_mask = ring->ring_mask;
	u16 queue_index;
	bool rxhash, rxcsum, rxvlan, rxall;
	bool copy_recycle = rtlmq_rx_copy_recycle_active(ring);
	unsigned int rx_length_errors = 0;
	unsigned int rx_crc_errors = 0;
	unsigned int rx_packets = 0;
	unsigned int rx_errors = 0;
	unsigned int hw_packets = 0;
	unsigned int entry;
	u32 hw_bytes = 0;
	enum rtlmq_rx_clean_exit_reason exit_reason =
		RTLMQ_RX_CLEAN_EXIT_UNKNOWN;
	int work_done = 0;
	void *desc;
	u32 status;

	stats = &ring->cold->stats;
	/*
	 * RX clean starts from the same published clean cursor sampled by
	 * rtlmq_rx_ring_head_ready(); only consumed descriptors advance it.
	 */
	entry = next_to_clean;
	desc = desc_base + entry * desc_stride;
	if (!ring->rx_posted ||
	    (!rtlmq_rx_desc_acquire(ring, desc, &status) &&
	    (skip_entry_own_mmio ||
	     !rtlmq_rx_desc_try_recover_own_fast(ring, desc, &status))))
		return rtlmq_rx_finish_skb_fast_empty(ring,
						      exit_reason_out);

	ndev = ring->ndev;
	napi = &vector->napi;
	buffers = ring->buffers;
	queue_index = ring->queue_index;
	rx_fast_flags = READ_ONCE(ring->rx_fast_flags);
	rxhash = (rx_fast_flags & RTLMQ_RX_FAST_RXHASH) &&
		 profile->rss_l3_mask;
	rxcsum = !!(rx_fast_flags & RTLMQ_RX_FAST_RXCSUM);
	rxvlan = !!(rx_fast_flags & RTLMQ_RX_FAST_RXVLAN);
	rxall = !!(rx_fast_flags & RTLMQ_RX_FAST_RXALL);
	if (rxcsum)
		desc_decode_flags |= RTLMQ_RX_DESC_DECODE_CSUM;
	if (rxhash)
		desc_decode_flags |= RTLMQ_RX_DESC_DECODE_RSS;
	if (rxvlan)
		desc_decode_flags |= RTLMQ_RX_DESC_DECODE_VLAN;
	fcs_adjust = (rx_fast_flags & RTLMQ_RX_FAST_STRIP_FCS) ?
		     ETH_FCS_LEN : 0;

	/*
	 * work_done counts packets, including terminal drops. A malformed
	 * packet's suffix only consumes descriptors. rx_posted bounds that
	 * scan to one ring even if no final-fragment marker ever arrives.
	 */
	for (;;) {
		struct rtlmq_rx_buffer *buffer = &buffers[entry];
		struct rtlmq_rx_desc_decode decoded;
		unsigned int pkt_size;
		struct sk_buff *skb;
		enum rtlmq_rx_length_error length_error =
			RTLMQ_RX_LENGTH_OK;
		bool fragment_bad;
		bool status_bad;
		u8 errors;

		/*
		 * Hwtstamp setup enables this static key before PHY capture.
		 * Leave this descriptor untouched for the generic parser.
		 */
		if (static_branch_unlikely(&rtlmq_ptp_rx_scan_key) &&
		    unlikely(rtlmq_ptp_rx_timestamp_scan_required(ring->priv))) {
			exit_reason = RTLMQ_RX_CLEAN_EXIT_STATUS_UNSUPPORTED;
			break;
		}

		/* Additional inline VLAN tags can cross the normal DMA span. */
		if (unlikely((status & profile->first_frag_mask) &&
			     !(status & profile->last_frag_mask))) {
			exit_reason = RTLMQ_RX_CLEAN_EXIT_STATUS_UNSUPPORTED;
			break;
		}

		next_to_clean = (entry + 1) & ring_mask;
		if (work_done + 1 < budget)
			rtlmq_rx_prefetch_next_slot(desc_base, desc_stride,
						    buffers,
						    next_to_clean);
		ring->rx_posted--;
		rtlmq_decode_rx_desc(profile, desc, status, desc_decode_flags,
				     &decoded);
		errors = rtlmq_rx_packet_errors(ring->cold, &decoded);
		if (errors) {
			if (errors & RTLMQ_RX_ERROR_ANY)
				rx_errors++;
			if (errors & RTLMQ_RX_ERROR_CRC)
				rx_crc_errors++;
			if (errors & RTLMQ_RX_ERROR_LENGTH)
				rx_length_errors++;
		}
		status_bad = !rtlmq_rx_desc_error_deliverable_decoded(&decoded,
								     rxall);
		fragment_bad = decoded.fragmented;
		if (unlikely(rtlmq_rx_discard_suffix(ring, &decoded))) {
			rtlmq_reuse_rx_buffer(ring, buffer);
			goto rx_desc_done;
		}

		if (unlikely(status_bad || fragment_bad)) {
			rtlmq_rx_drop_chain(ring, &decoded);
			rtlmq_rx_fast_drop_reason(ring, status_bad ?
				RTLMQ_RX_DROP_STATUS : RTLMQ_RX_DROP_FRAGMENT);
			work_done++;
			rtlmq_reuse_rx_buffer(ring, buffer);
			goto rx_desc_done;
		}

		length_error = rtlmq_rx_packet_len_validate(ring, buffer, &decoded, !!fcs_adjust,
							    &pkt_size);
		if (length_error == RTLMQ_RX_LENGTH_OK && pkt_size < ETH_HLEN)
			length_error = RTLMQ_RX_LENGTH_TOO_SMALL;
		if (unlikely(length_error != RTLMQ_RX_LENGTH_OK)) {
			rtlmq_rx_fast_drop_reason(ring, RTLMQ_RX_DROP_LENGTH);
			work_done++;
			rtlmq_reuse_rx_buffer(ring, buffer);
			goto rx_desc_done;
		}

		hw_packets++;
		hw_bytes += pkt_size;
		/* Invalid storage uses the native constructor's rejection path. */
		if ((copy_recycle || pkt_size <= RTLMQ_RX_COPYBREAK) &&
		    likely(buffer->hard_start &&
			   rtlmq_rx_skb_data_bounds_valid(ring, RTLMQ_RX_HEADROOM, pkt_size))) {
			rtlmq_sync_rx_page_pool_buffer_for_cpu(ring, buffer,
							       pkt_size);
			skb = rtlmq_rx_copy_skb(
				napi, NULL,
				(u8 *)buffer->hard_start + RTLMQ_RX_HEADROOM,
				NULL, pkt_size, 0);
			buffer->sync_len = pkt_size;
			/* Failure drops the packet; no fallback borrows recycled data. */
			rtlmq_reuse_rx_buffer(ring, buffer);
			if (IS_ERR(skb)) {
				rtlmq_rx_fast_drop_reason(
					ring, PTR_ERR(skb) == -ENOMEM ?
						RTLMQ_RX_DROP_SKB : RTLMQ_RX_DROP_LENGTH);
				skb = NULL;
			}
		} else {
			skb = rtlmq_rx_build_skb_fast(ring, buffer, pkt_size);
		}
		work_done++;
		if (!skb)
			goto rx_desc_done;

		if (unlikely(rxvlan))
			rtlmq_rx_set_skb_vlan_tag(skb, &decoded);

		if (likely(rxcsum))
			rtlmq_rx_set_skb_checksum_status_fast(&decoded, skb);
		else
			skb->ip_summed = CHECKSUM_NONE;

		if (unlikely(rxhash))
			rtlmq_rx_set_skb_hash_enabled(&decoded, skb);
		skb->protocol = eth_type_trans(skb, ndev);
		skb_record_rx_queue(skb, queue_index);
		napi_gro_receive(napi, skb);

		rx_packets++;

rx_desc_done:
		if (work_done >= budget) {
			exit_reason = RTLMQ_RX_CLEAN_EXIT_BUDGET;
			break;
		}

		if (!rtlmq_rx_skb_fast_prepare_next_desc(ring, desc_base, desc_stride,
							 next_to_clean, &entry, &desc, &status,
							 &exit_reason))
			break;
	}
	{
		const struct rtlmq_rx_fast_batch batch = {
			.rx_packets = rx_packets,
			.hw_packets = hw_packets,
			.hw_bytes = hw_bytes,
			.rx_errors = rx_errors,
			.rx_crc_errors = rx_crc_errors,
			.rx_length_errors = rx_length_errors,
		};

		exit_reason = rtlmq_rx_finish_skb_fast_poll(ring, stats, next_to_clean, &batch,
							    work_done, exit_reason);
	}
	rtlmq_rx_publish_clean_exit_reason(exit_reason_out, exit_reason);
	return work_done;
}

static int
rtlmq_poll_rx_ring_generic(struct rtlmq_vector *vector,
			   struct rtlmq_rx_ring *ring, int budget,
			   enum rtlmq_rx_clean_exit_reason *exit_reason_out);

int rtlmq_poll_rx_ring_skb_fast(struct rtlmq_vector *vector,
				struct rtlmq_rx_ring *ring, int budget,
				bool skip_entry_own_mmio,
				enum rtlmq_rx_clean_exit_reason *exit_reason_out)
{
	enum rtlmq_rx_clean_exit_reason exit_reason;
	int work;

	if (unlikely(ring->cold->page_mb_head.opaque))
		return rtlmq_poll_rx_ring_generic(vector, ring, budget,
						  exit_reason_out);

	{
		work = rtlmq_poll_rx_ring_skb_fast_path(vector, ring, budget,
							skip_entry_own_mmio,
							&exit_reason);
	}
	if (unlikely(exit_reason == RTLMQ_RX_CLEAN_EXIT_STATUS_UNSUPPORTED))
		work += rtlmq_poll_rx_ring_generic(vector, ring, budget - work,
						    &exit_reason);
	rtlmq_rx_publish_clean_exit_reason(exit_reason_out, exit_reason);
	return work;
}

static enum rtlmq_rx_clean_exit_reason
rtlmq_rx_finish_generic_poll(struct rtlmq_rx_ring *ring, struct rtlmq_rx_ring_stats *stats,
			     const struct rtlmq_rx_poll_hot_stats *hot, unsigned int next_to_clean,
			     bool xdp_flush_needed, int work_done, int budget, u64 hw_packets,
			     u64 hw_bytes, u64 rx_packets,
			     enum rtlmq_rx_clean_exit_reason exit_reason)
{
	unsigned int refill_pending;

	if (xdp_flush_needed)
		xdp_do_flush();
	if (work_done >= budget)
		exit_reason = RTLMQ_RX_CLEAN_EXIT_BUDGET;

	refill_pending = rtlmq_rx_refill_pending(ring);
	/*
	 * Advance next_to_clean before refill/accounting lets RX-ready sample
	 * the ring again. It must not inspect already-consumed descriptors.
	 */
	ring->next_to_clean = next_to_clean;
	{
		enum rtlmq_rx_refill_failure failure;
		unsigned int posted;

		posted = rtlmq_refill_rx_ring_generic(ring, refill_pending,
						      &failure);
		exit_reason = rtlmq_rx_refill_clean_exit_reason(exit_reason,
								refill_pending, posted, failure
		);
	}
	rtlmq_rx_flush_hot_stats(stats, hot);
	ring->hw_packets += hw_packets;
	ring->hw_bytes += hw_bytes;
	rtlmq_rx_stats_update_end(ring);
	if (trace_rtlmq_rx_result_enabled() &&
	    (rx_packets || hot->drops))
		trace_rtlmq_rx_result(ring->ndev->name, ring->queue_index,
				      READ_ONCE(ring->priv->active_resource_generation), false,
				      rx_packets, hot->drops);
	return exit_reason;
}

/* Rejection retires the old chain; the caller still owns the current slot. */
static inline bool
rtlmq_rx_validate_generic_packet(struct rtlmq_rx_ring *ring,
				 struct rtlmq_rx_buffer *buffer,
				 const struct rtlmq_rx_desc_decode *decoded,
				 bool rxall, bool strip_fcs, bool uses_xsk,
				 struct rtlmq_rx_poll_hot_stats *hot,
				 unsigned int *pkt_size)
{
	struct rtlmq_rx_ring_stats *stats = &ring->cold->stats;
	enum rtlmq_rx_length_error length_error;

	if (unlikely(!rtlmq_rx_desc_error_deliverable_decoded(decoded, rxall))) {
		rtlmq_rx_drop_chain(ring, decoded);
		rtlmq_rx_count_drop(stats, hot, RTLMQ_RX_DROP_STATUS);
		return false;
	}

	length_error = rtlmq_rx_packet_len_validate(ring, buffer, decoded,
						    strip_fcs, pkt_size);
	if (unlikely(length_error != RTLMQ_RX_LENGTH_OK)) {
		rtlmq_rx_drop_chain(ring, decoded);
		rtlmq_rx_count_drop(stats, hot, RTLMQ_RX_DROP_LENGTH);
		return false;
	}

	return true;
}

static inline bool
rtlmq_rx_prepare_timestamp(struct rtlmq_rx_ring *ring,
			   struct rtlmq_rx_buffer *packet_buffer, struct xdp_buff *prepared_xdp,
			   unsigned int pkt_size, bool packet_detached, bool uses_xsk,
			   ktime_t *timestamp, bool *dma_cpu_owned)
{
	const void *packet_data;
	size_t linear_len;

	/*
	 * PHY ingress storage is selected by the packet's event
	 * message type. Synchronize only while timestamping is
	 * active; fragmented packets were synchronized as each
	 * segment entered multibuffer assembly.
	 */
	if (!packet_detached) {
		rtlmq_sync_rx_buffer_for_cpu(ring, packet_buffer, pkt_size);
		*dma_cpu_owned = true;
	}
	if (prepared_xdp) {
		packet_data = prepared_xdp->data;
		linear_len = prepared_xdp->data_end -
			     prepared_xdp->data;
	} else if (uses_xsk) {
		packet_data = packet_buffer->xdp->data;
		linear_len = pkt_size;
	} else {
		packet_data = (u8 *)rtlmq_rx_buffer_hard_start(ring, packet_buffer) +
			RTLMQ_RX_HEADROOM;
		linear_len = pkt_size;
	}
	return !rtlmq_ptp_rx_timestamp(ring->priv, packet_data, linear_len,
				       pkt_size, timestamp);
}

/* True consumes the packet even on submission failure; false retains its slot. */
static inline bool
rtlmq_rx_take_xdp_tx(struct rtlmq_rx_ring *ring,
		     struct rtlmq_rx_buffer *packet_buffer, struct rtlmq_xdp_result *xdp,
		     bool uses_xsk, struct rtlmq_rx_poll_hot_stats *hot)
{
	struct rtlmq_rx_ring_stats *stats = &ring->cold->stats;
	struct rtlmq_rx_buffer original;
	int ret;

	if (xdp->detached) {
		original = *packet_buffer;
	} else if (uses_xsk) {
		/*
		 * XDP TX copies the packet and returns this UMEM
		 * frame during conversion, not TX completion.
		 * Leave the RX slot empty for common refill rather
		 * than require another fill entry before sending.
		 */
		original = *packet_buffer;
		rtlmq_rx_clear_buffer_fast(packet_buffer);
	} else {
		ret = rtlmq_rx_replace_buffer_for_handoff(ring, packet_buffer, &original);
		if (ret) {
			rtlmq_rx_count_drop(stats, hot, RTLMQ_RX_DROP_RESOURCE);
			hot->alloc_fail++;
			return false;
		}
	}

	if (rtlmq_xdp_tx_frame(ring, &original, xdp)) {
		rtlmq_rx_count_drop(stats, hot, RTLMQ_RX_DROP_XDP);
	}
	return true;
}

/* As for TX, false retains the slot and true consumes every handoff outcome. */
static inline bool
rtlmq_rx_take_xdp_redirect(struct rtlmq_rx_ring *ring,
			   struct rtlmq_rx_buffer *buffer, struct rtlmq_xdp_result *xdp,
			   bool uses_xsk, struct rtlmq_rx_poll_hot_stats *hot,
			   bool *xdp_flush_needed)
{
	struct rtlmq_rx_ring_stats *stats = &ring->cold->stats;
	int ret;

	/*
	 * Redirect consumes the current RX page on both the
	 * success path and the post-conversion failure path, so
	 * acquire the replacement buffer before handing page
	 * ownership to the XDP core.
	 */
	if (!xdp->detached && uses_xsk) {
		/*
		 * XSKMAP consumes the completed UMEM frame. Detach
		 * it from the hardware slot before redirect; the
		 * common refill path may repopulate the slot from
		 * a later userspace FQ entry. Requiring an immediate
		 * replacement deadlocks a normally full RX ring.
		 */
		rtlmq_rx_clear_buffer_fast(buffer);
	} else if (!xdp->detached) {
		ret = rtlmq_rx_replace_buffer_for_handoff(ring, buffer, NULL);
		if (ret) {
			rtlmq_rx_count_drop(stats, hot, RTLMQ_RX_DROP_RESOURCE);
			hot->alloc_fail++;
			return false;
		}
	}

	/* Fan-out can enqueue recipients before returning an error. */
	*xdp_flush_needed = true;
	ret = rtlmq_xdp_redirect(ring, xdp->xdp, xdp->prog);
	if (ret) {
		rtlmq_rx_count_drop(stats, hot, RTLMQ_RX_DROP_XDP);
	}
	return true;
}

/* All outcomes consume or recycle the packet; a returned skb owns its data. */
static inline struct sk_buff *
rtlmq_rx_build_pass_skb(struct rtlmq_rx_ring *ring,
			struct rtlmq_rx_buffer *packet_buffer,
			const struct rtlmq_rx_desc_decode *decoded, struct rtlmq_xdp_result *xdp,
			struct napi_struct *napi, bool copy_recycle, bool uses_xsk, bool rxcsum,
			struct rtlmq_rx_poll_hot_stats *hot, bool *skb_protocol_ready)
{
	struct rtlmq_rx_ring_stats *stats = &ring->cold->stats;
	unsigned int pkt_size = xdp->len;
	struct sk_buff *skb;

	/* The native XDP skb constructor parses the linear Ethernet header. */
	if ((!xdp->prog &&
	     (copy_recycle || (!xdp->detached && !uses_xsk &&
			       pkt_size <= RTLMQ_RX_COPYBREAK))) ||
	    (xdp->detached && !uses_xsk &&
	     xdp->xdp->data_end - xdp->xdp->data < ETH_HLEN)) {
		if (!xdp->detached && !xdp->dma_cpu_owned)
			rtlmq_sync_rx_buffer_for_cpu(ring, packet_buffer, pkt_size);
		skb = rtlmq_rx_build_copy_skb(napi, xdp->xdp, decoded, xdp->data, xdp->data_meta,
					      pkt_size, xdp->meta_len, rxcsum, stats, hot);
		if (!xdp->prog && !xdp->detached && !uses_xsk)
			packet_buffer->sync_len = pkt_size;
		if (xdp->detached)
			rtlmq_release_rx_page_packet(ring, packet_buffer, xdp->xdp, true);
		else
			rtlmq_reuse_rx_buffer(ring, packet_buffer);
	} else if (uses_xsk) {
		skb = rtlmq_rx_build_copy_skb(napi, xdp->xdp, decoded, xdp->data,
					      xdp->data_meta, pkt_size, xdp->meta_len,
					      rxcsum, stats, hot);
		/*
		 * The skb owns a copy. Keep the XSK frame queue-local
		 * and republish it to hardware regardless of whether
		 * skb allocation succeeded.
		 */
		if (xdp->detached)
			xdp_return_buff(xdp->xdp);
		else
			rtlmq_reuse_rx_buffer(ring, packet_buffer);
	} else if (xdp->detached) {
		skb = xdp_build_skb_from_buff(xdp->xdp);
		if (!skb) {
			rtlmq_rx_count_drop(stats, hot, RTLMQ_RX_DROP_SKB);
			hot->alloc_fail++;
			xdp_return_buff(xdp->xdp);
		} else {
			rtlmq_rx_set_skb_checksum_checked(decoded, rxcsum, stats, hot, skb);
			*skb_protocol_ready = true;
		}
	} else {
		skb = rtlmq_rx_build_skb_recycle(ring, packet_buffer, decoded,
						 xdp->data,
						 xdp->data_meta,
						 pkt_size,
						 xdp->meta_len,
						 xdp->dma_cpu_owned,
						 rxcsum, stats, hot);
	}
	return skb;
}

static int
rtlmq_poll_rx_ring_generic(struct rtlmq_vector *vector,
			   struct rtlmq_rx_ring *ring, int budget,
			   enum rtlmq_rx_clean_exit_reason *exit_reason_out)
{
	struct rtlmq_rx_ring_stats *stats;
	struct rtlmq_private *tp = ring->priv;
	struct net_device *ndev = ring->ndev;
	struct napi_struct *napi = ring->napi ? ring->napi : &vector->napi;
	const struct rtlmq_rx_desc_profile *profile = ring->profile;
	struct rtlmq_rx_buffer *buffers = ring->buffers;
	u8 *desc_base = ring->desc;
	u16 desc_stride = profile->stride;
	unsigned int desc_decode_flags = 0;
	bool uses_xsk = rtlmq_rx_ring_uses_xsk_pool(ring);
	bool copy_recycle = !uses_xsk && rtlmq_rx_copy_recycle_active(ring);
	unsigned int data_limit = rtlmq_rx_dma_data_limit(ring);
	unsigned int next_to_clean = ring->next_to_clean;
	unsigned int ring_mask = ring->ring_mask;
	bool xdp_flush_needed = false;
	bool rxhash = !!(ndev->features & NETIF_F_RXHASH);
	bool rxcsum = !!(ndev->features & NETIF_F_RXCSUM);
	bool rxvlan = !!(ndev->features & NETIF_F_HW_VLAN_CTAG_RX);
	bool rxall = !!(ndev->features & NETIF_F_RXALL);
	bool strip_fcs = !(ndev->features & NETIF_F_RXFCS);
	struct rtlmq_rx_poll_hot_stats hot = {};
	u64 rx_packets = 0;
	u64 hw_packets = 0;
	u64 hw_bytes = 0;
	enum rtlmq_rx_clean_exit_reason exit_reason =
		RTLMQ_RX_CLEAN_EXIT_UNKNOWN;
	int work_done = 0;

	if (unlikely(!ring->cold))
		return 0;
	stats = &ring->cold->stats;
	if (rxhash)
		desc_decode_flags |= RTLMQ_RX_DESC_DECODE_RSS;
	if (rxcsum)
		desc_decode_flags |= RTLMQ_RX_DESC_DECODE_CSUM;
	if (rxvlan)
		desc_decode_flags |= RTLMQ_RX_DESC_DECODE_VLAN;
	rtlmq_rx_stats_update_begin(ring);

	/*
	 * work_done counts packets, not fragments. The posted interval also
	 * bounds descriptor progress, including a corrupt unterminated suffix.
	 * Refill only occurs after this loop, so a poll cannot lap the ring.
	 */
	while (work_done < budget) {
		unsigned int entry = next_to_clean;
		void *desc = desc_base + entry * desc_stride;
		struct rtlmq_rx_buffer *buffer = &buffers[entry];
		struct rtlmq_rx_buffer multibuffer_packet = {};
		struct rtlmq_rx_buffer *packet_buffer = buffer;
		struct xdp_buff *prepared_xdp = NULL;
		struct rtlmq_rx_desc_decode decoded;
		unsigned int pkt_size;
		struct sk_buff *skb;
		ktime_t rx_timestamp = 0;
		bool skb_protocol_ready = false;
		bool rx_timestamp_valid = false;
		bool ptp_dma_cpu_owned = false;
		bool packet_detached = false;
		struct rtlmq_xdp_result xdp;
		u32 xdp_act;
		u32 status;
		u8 errors;

		/*
		 * Unposted descriptors can retain CPU-owned completion status.
		 * Check rx_posted to distinguish them from completed DMA.
		 * Poll exit retries fill-ring allocation.
		 */
		if (unlikely(!READ_ONCE(ring->rx_posted))) {
			exit_reason = RTLMQ_RX_CLEAN_EXIT_HW_OWNED;
			break;
		}

		if (!rtlmq_rx_desc_acquire(ring, desc, &status)) {
			exit_reason = RTLMQ_RX_CLEAN_EXIT_HW_OWNED;
			break;
		}
		/*
		 * Fragment status is per-descriptor; checksum, VLAN and RSS
		 * describe the delivered packet and are consumed only on its
		 * terminal descriptor. In V4 the RSS word overlays the DMA address.
		 */
		rtlmq_decode_rx_desc(profile, desc, status,
				     status & profile->last_frag_mask ?
				     desc_decode_flags : 0,
				     &decoded);
		if (unlikely(decoded.first_frag &&
			     (ring->cold->xsk_mb_head ||
			      ring->cold->page_mb_head.opaque))) {
			/* A new first marker terminates the old incomplete frame. */
			rtlmq_release_rx_multibuffer(ring);
			rtlmq_rx_count_drop(stats, &hot, RTLMQ_RX_DROP_FRAGMENT);
			if (++work_done == budget)
				break; /* Leave this new frame posted for the next poll. */
		}
		ring->rx_posted--;
		next_to_clean = (entry + 1) & ring_mask;
		if (work_done + 1 < budget)
			rtlmq_rx_prefetch_next_slot(desc_base, desc_stride,
						    buffers,
						    next_to_clean);

		errors = rtlmq_rx_packet_errors(ring->cold, &decoded);
		if (unlikely(errors))
			rtlmq_rx_account_packet_errors(stats, errors);
		if (unlikely(rtlmq_rx_discard_suffix(ring, &decoded))) {
			rtlmq_reuse_rx_buffer(ring, buffer);
			goto rx_desc_done;
		}

		if (unlikely(!rtlmq_rx_validate_generic_packet(ring, buffer,
							       &decoded, rxall,
							       strip_fcs, uses_xsk,
							       &hot, &pkt_size)))
			goto discard_slot;

		if (unlikely(decoded.fragmented)) {
			bool packet_timestamp_valid = false;
			int mb_result;

			if (uses_xsk)
				mb_result = rtlmq_xsk_mb_take_fragment(
					ring, buffer, pkt_size, &decoded,
					strip_fcs,
					rx_timestamp_valid ?
						&rx_timestamp : NULL,
					&multibuffer_packet, &rx_timestamp,
					&packet_timestamp_valid);
			else
				mb_result = rtlmq_page_mb_take_fragment(
					ring, buffer, pkt_size, &decoded,
					strip_fcs,
					rx_timestamp_valid ?
						&rx_timestamp : NULL,
					&multibuffer_packet, &prepared_xdp,
					&rx_timestamp,
					&packet_timestamp_valid);
			if (mb_result == RTLMQ_XSK_MB_CONTINUE)
				goto rx_desc_done;
			if (mb_result == RTLMQ_XSK_MB_ERROR) {
				rtlmq_rx_drop_chain(ring, &decoded);
				rtlmq_rx_count_drop(stats, &hot, RTLMQ_RX_DROP_FRAGMENT);
				goto discard_slot;
			}

			packet_buffer = &multibuffer_packet;
			packet_detached = true;
			rx_timestamp_valid = packet_timestamp_valid;
			if (uses_xsk)
				prepared_xdp = multibuffer_packet.xdp;
			pkt_size = xdp_get_buff_len(prepared_xdp);
		}
		if (rtlmq_ptp_rx_scan_maybe_required(tp) &&
		    !rx_timestamp_valid) {
			rx_timestamp_valid =
				rtlmq_rx_prepare_timestamp(ring, packet_buffer,
							   prepared_xdp, pkt_size, packet_detached,
							   uses_xsk, &rx_timestamp,
							   &ptp_dma_cpu_owned);
		}
		hw_packets++;
		hw_bytes += pkt_size;

		xdp_act = rtlmq_run_xdp(ring, packet_buffer, &decoded,
					rx_timestamp_valid ? &rx_timestamp : NULL, pkt_size,
					prepared_xdp, ptp_dma_cpu_owned, &xdp);
		xdp.detached = packet_detached;
		if (unlikely((xdp_act == XDP_PASS || xdp_act == XDP_TX ||
			      xdp_act == XDP_REDIRECT) &&
			     !rtlmq_rx_xdp_bounds_valid(ring, packet_buffer, &xdp,
							data_limit))) {
			rtlmq_rx_count_drop(stats, &hot, RTLMQ_RX_DROP_LENGTH);
			goto discard_xdp;
		}
		if (xdp_act == XDP_TX) {
			if (!(ndev->xdp_features & NETDEV_XDP_ACT_NDO_XMIT))
				goto drop_xdp;
			if (!rtlmq_rx_take_xdp_tx(ring, packet_buffer, &xdp,
						  uses_xsk, &hot))
				goto discard_slot;
			work_done++;
			goto rx_desc_done;
		}

		if (xdp_act == XDP_REDIRECT) {
			if (!(ndev->xdp_features & NETDEV_XDP_ACT_REDIRECT))
				goto drop_xdp;
			if (!rtlmq_rx_take_xdp_redirect(ring, buffer, &xdp,
							uses_xsk, &hot,
							&xdp_flush_needed))
				goto discard_slot;
			work_done++;
			goto rx_desc_done;
		}

		if (xdp_act != XDP_PASS)
			goto drop_xdp;

		pkt_size = xdp.len;
		if (unlikely(pkt_size < ETH_HLEN)) {
			rtlmq_rx_count_drop(stats, &hot, RTLMQ_RX_DROP_LENGTH);
			goto discard_xdp;
		}

		skb = rtlmq_rx_build_pass_skb(ring, packet_buffer, &decoded, &xdp,
					      napi, copy_recycle, uses_xsk, rxcsum,
					      &hot, &skb_protocol_ready);
		if (!skb) {
			work_done++;
			goto rx_desc_done;
		}

		if (rx_timestamp_valid)
			skb_hwtstamps(skb)->hwtstamp = rx_timestamp;

		if (unlikely(rxvlan))
			rtlmq_rx_set_skb_vlan_tag(skb, &decoded);

		if (rxhash)
			rtlmq_rx_set_skb_hash(ndev, &decoded, skb);
		if (!skb_protocol_ready)
			skb->protocol = eth_type_trans(skb, ndev);
		skb_record_rx_queue(skb, ring->queue_index);
		napi_gro_receive(napi, skb);

		rx_packets++;
		work_done++;
		goto rx_desc_done;

discard_slot:
		work_done++;
		rtlmq_reuse_rx_buffer(ring, buffer);
		goto rx_desc_done;

drop_xdp:
		rtlmq_rx_count_drop(stats, &hot, RTLMQ_RX_DROP_XDP);
discard_xdp:
		work_done++;
		if (xdp.detached)
			xdp_return_buff(xdp.xdp);
		else
			rtlmq_reuse_rx_buffer(ring, buffer);
rx_desc_done:
		continue;
	}

	exit_reason = rtlmq_rx_finish_generic_poll(ring, stats, &hot,
						   next_to_clean,
						   xdp_flush_needed,
						   work_done, budget,
						   hw_packets, hw_bytes,
						   rx_packets, exit_reason);
	rtlmq_rx_publish_clean_exit_reason(exit_reason_out, exit_reason);
	return work_done;
}

int rtlmq_poll_rx_ring(struct rtlmq_vector *vector, struct rtlmq_rx_ring *ring,
		       int budget, bool skip_entry_own_mmio,
		       enum rtlmq_rx_clean_exit_reason *exit_reason_out)
{
	if (unlikely(budget <= 0)) {
		if (exit_reason_out)
			*exit_reason_out = RTLMQ_RX_CLEAN_EXIT_NO_STATUS;
		return 0;
	}

	if (likely(!rtlmq_ptp_rx_scan_maybe_required(ring->priv) &&
		   rtlmq_rx_skb_fast_entry_ready(ring)))
		return rtlmq_poll_rx_ring_skb_fast(vector, ring, budget,
						   skip_entry_own_mmio,
						   exit_reason_out);

	return rtlmq_poll_rx_ring_generic(vector, ring, budget,
					  exit_reason_out);
}
