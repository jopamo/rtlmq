// SPDX-License-Identifier: GPL-2.0-only

#include <linux/etherdevice.h>
#include <linux/err.h>
#include <linux/mm.h>
#include <linux/overflow.h>
#include <linux/skbuff.h>
#include <linux/slab.h>
#include <net/page_pool/helpers.h>
#include <net/xdp_sock_drv.h>

#include "rtlmq.h"
#include "rtlmq_ring.h"
#include "rtlmq_rx.h"
#include "rtlmq_xdp.h"

/* RX buffer allocation, recycling, and descriptor refill. */

void *rtlmq_rx_buffer_hard_start(const struct rtlmq_rx_ring *ring,
				 const struct rtlmq_rx_buffer *buffer)
{
	if (buffer->hard_start)
		return buffer->hard_start;

	if (rtlmq_rx_ring_uses_xsk_pool(ring))
		return buffer->xdp->data_hard_start;

	return page_address(buffer->page);
}

void rtlmq_sync_rx_page_pool_buffer_for_cpu(struct rtlmq_rx_ring *ring,
					    const struct rtlmq_rx_buffer *buffer,
					    unsigned int len)
{
	unsigned int sync_len = min_t(unsigned int, len, ring->buffer_len);

	if (likely(sync_len))
		page_pool_dma_sync_for_cpu(ring->page_pool, buffer->page, 0,
					   sync_len);
}

void rtlmq_sync_rx_buffer_for_cpu(struct rtlmq_rx_ring *ring,
				  struct rtlmq_rx_buffer *buffer,
				  unsigned int len)
{
	if (!min_t(unsigned int, len, ring->buffer_len))
		return;

	if (rtlmq_rx_ring_uses_xsk_pool(ring)) {
		xsk_buff_dma_sync_for_cpu(buffer->xdp);
		return;
	}

	/*
	 * page_pool_dma_sync_for_cpu() adds pp_params.offset, which is already
	 * RTLMQ_RX_HEADROOM. Pass zero per-frame offset to avoid adding headroom
	 * twice and leaving packet headers stale on non-coherent systems.
	 */
	rtlmq_sync_rx_page_pool_buffer_for_cpu(ring, buffer, len);
}

void rtlmq_sync_rx_buffer_for_device(struct rtlmq_rx_ring *ring,
				     struct rtlmq_rx_buffer *buffer,
				     unsigned int len)
{
	struct rtlmq_private *tp = ring->priv;
	unsigned int sync_len = min_t(unsigned int, len, ring->buffer_len);

	if (!sync_len)
		return;

	if (rtlmq_rx_ring_uses_xsk_pool(ring)) {
		xsk_buff_raw_dma_sync_for_device(ring->cold->xsk_pool,
						 buffer->dma_addr,
						 sync_len);
		return;
	}

	/*
	 * Use this pool's DMA direction, not requested XDP settings that may
	 * already describe a later configuration.
	 *
	 * buffer->dma_addr is the device-facing address of the payload area
	 * (pool mapping plus headroom) and is the same value the refill
	 * publishes, so the synchronization cannot disagree with the
	 * descriptor and no page metadata has to be dereferenced.
	 */
	dma_sync_single_for_device(&tp->pdev->dev, buffer->dma_addr,
				   sync_len, ring->page_pool->p.dma_dir);
}

/*
 * Publish-time device synchronization for ordinary page-pool RX buffers.
 *
 * The pool is created without PP_FLAG_DMA_SYNC_DEV, so page pool never syncs a
 * returned page. Only an ordinary copy that retains its source in the ring
 * records a bounded CPU access. All other paths synchronize the full writable
 * span, including pages returned by the stack and XDP reuse. Consume that bound
 * before publishing OWN so the next completion cannot inherit it.
 * XSK pools keep their own fill-ring synchronization contract.
 */
static void rtlmq_refill_rx_sync_for_device(struct rtlmq_rx_ring *ring,
					    struct rtlmq_rx_buffer *buffer)
{
	if (rtlmq_rx_ring_uses_xsk_pool(ring) || !buffer->opaque)
		return;

	rtlmq_sync_rx_buffer_for_device(ring, buffer,
					buffer->sync_len ?: ring->buffer_len);
	buffer->sync_len = 0;
}

/* Consume the allocated page; leave buffer unchanged on failure. */
static inline int rtlmq_init_rx_page_buffer(struct rtlmq_rx_ring *ring,
					    struct rtlmq_rx_buffer *buffer,
					    struct page *page)
{
	struct rtlmq_rx_buffer candidate = {};

	if (unlikely(!rtlmq_dma_addr_add_checked(page_pool_get_dma_addr(page),
						 (u64)RTLMQ_RX_HEADROOM,
						 &candidate.dma_addr)))
		goto err_put_page;
	candidate.hard_start = page_address(page);
	candidate.page = page;
	if (unlikely(!rtlmq_rx_buffer_ready_for_device(&candidate) ||
		     !rtlmq_dma_range_fits_device(&ring->priv->pdev->dev,
						 candidate.dma_addr,
						 ring->buffer_len)))
		goto err_put_page;
	*buffer = candidate;
	return 0;

err_put_page:
	page_pool_put_full_page(ring->page_pool, page, false);
	return -EIO;
}

int rtlmq_alloc_rx_buffer(struct rtlmq_rx_ring *ring,
			  struct rtlmq_rx_buffer *buffer, gfp_t gfp)
{
	struct rtlmq_rx_buffer candidate = {};
	struct xdp_buff *xdp;
	struct page *page;
	int ret;

	if (rtlmq_rx_ring_uses_xsk_pool(ring)) {
		xdp = xsk_buff_alloc(ring->cold->xsk_pool);
		if (!xdp)
			return -ENOMEM;

		candidate.dma_addr = xsk_buff_xdp_get_dma(xdp);
		candidate.hard_start = xdp->data_hard_start;
		candidate.xdp = xdp;
		if (unlikely(!rtlmq_rx_buffer_ready_for_device(&candidate)))
			goto err_free_xsk;
		if (unlikely(!rtlmq_dma_range_fits_device(&ring->priv->pdev->dev,
							  candidate.dma_addr, ring->buffer_len)))
			goto err_free_xsk;
		*buffer = candidate;
		return 0;

err_free_xsk:
		xsk_buff_free(xdp);
		return -EIO;
	}

	page = page_pool_alloc_pages(ring->page_pool, gfp);
	if (!page)
		return -ENOMEM;

	ret = rtlmq_init_rx_page_buffer(ring, buffer, page);
	return ret;
}

void rtlmq_release_rx_buffer(struct rtlmq_rx_ring *ring,
			     struct rtlmq_rx_buffer *buffer, bool direct)
{
	if (!buffer->opaque)
		return;

	if (rtlmq_rx_ring_uses_xsk_pool(ring)) {
		xsk_buff_free(buffer->xdp);
		memset(buffer, 0, sizeof(*buffer));
		return;
	}

	page_pool_put_full_page(ring->page_pool, buffer->page, direct);
	memset(buffer, 0, sizeof(*buffer));
}

void rtlmq_recycle_rx_buffer_napi(struct rtlmq_rx_ring *ring,
				  struct rtlmq_rx_buffer *buffer)
{
	rtlmq_release_rx_buffer(ring, buffer, true);
}

void rtlmq_release_rx_page_packet(struct rtlmq_rx_ring *ring,
				  struct rtlmq_rx_buffer *buffer,
				  const struct xdp_buff *xdp, bool direct)
{
	if (!buffer->opaque)
		return;

	/*
	 * Source pages remain owned until copying or discard is complete.
	 * Return suffix pages before the head containing their shared_info.
	 * Refill, not pool return, owns the next device synchronization.
	 * Process-context teardown must pass direct=false.
	 */
	if (xdp_buff_has_frags(xdp)) {
		const struct skb_shared_info *sinfo =
			xdp_get_shared_info_from_buff(xdp);
		unsigned int i;

		for (i = 0; i < sinfo->nr_frags; i++)
			page_pool_put_full_netmem(ring->page_pool,
						  skb_frag_netmem(&sinfo->frags[i]), direct);
	}
	rtlmq_release_rx_buffer(ring, buffer, direct);
}

static void rtlmq_rx_desc_clear_refill_prefix(void *desc, u16 clear_size)
{
	switch (clear_size) {
	case 16:
		WRITE_ONCE(*(__le64 *)desc, 0);
		WRITE_ONCE(*(__le64 *)((u8 *)desc + 8), 0);
		break;
	case 8:
		WRITE_ONCE(*(__le64 *)desc, 0);
		break;
	case 4:
		WRITE_ONCE(*(__le32 *)desc, 0);
		break;
	case 0:
		break;
	default:
		memset(desc, 0, clear_size);
		break;
	}
}

static bool
rtlmq_publish_rx_desc_refill_fast(const struct rtlmq_rx_desc_profile *profile,
				  void *desc,
				  const struct rtlmq_rx_buffer *buffer,
				  u32 buffer_len,
				  u16 ring_count,
				  unsigned int entry)
{
	u32 opts1 = buffer_len;

	if (unlikely(!rtlmq_rx_buffer_ready_for_device(buffer)))
		return false;
	if (unlikely(!buffer_len))
		return false;
	if (unlikely(buffer_len > RTLMQ_RX_DESC_LEN_MASK))
		return false;
	if (entry == ring_count - 1) {
		opts1 |= RTLMQ_DESC_RING_END;
	}

	if (profile->clear_size)
		memset(desc, 0, profile->clear_size);

	rtlmq_rx_desc_qword_write(desc, profile->addr_offset,
				  cpu_to_le64(buffer->dma_addr));
	rtlmq_rx_desc_dword_write(desc, profile->opts2_offset,
				  cpu_to_le32(0));
	rtlmq_rx_desc_device_release(desc, profile->opts1_offset, opts1);

	return true;
}

static int rtlmq_refill_rx_slot_buffer(struct rtlmq_rx_ring *ring,
				       struct rtlmq_rx_buffer *buffer,
				       struct rtlmq_rx_ring_stats *stats)
{
	int ret;

	if (unlikely(buffer->opaque))
		return rtlmq_rx_buffer_ready_for_device(buffer) &&
		       rtlmq_dma_range_fits_device(&ring->priv->pdev->dev, buffer->dma_addr,
						   ring->buffer_len) ? 0 : -EINVAL;

	ret = rtlmq_alloc_rx_buffer(ring, buffer, GFP_ATOMIC | __GFP_NOWARN);
	if (!ret)
		return 0;

	if (stats)
		stats->alloc_fail++;

	return ret;
}

static __always_inline int
rtlmq_refill_rx_page_pool_buffer_fast(struct rtlmq_rx_ring *ring,
				      struct rtlmq_rx_buffer *buffer)
{
	struct page *page;

	if (unlikely(buffer->opaque))
		return rtlmq_rx_buffer_ready_for_device(buffer) &&
		       rtlmq_dma_range_fits_device(&ring->priv->pdev->dev, buffer->dma_addr,
						   ring->buffer_len) ? 0 : -EINVAL;

	page = page_pool_alloc_pages(ring->page_pool,
				     GFP_ATOMIC | __GFP_NOWARN);
	if (unlikely(!page))
		return -ENOMEM;

	return rtlmq_init_rx_page_buffer(ring, buffer, page);
}

static void rtlmq_refill_rx_fast_alloc_fail(struct rtlmq_rx_ring *ring)
{
	if (ring->cold)
		ring->cold->stats.alloc_fail++;
}

static unsigned int rtlmq_refill_rx_ring_fast_prepare_span_clear(struct rtlmq_rx_ring *ring,
								 unsigned int start,
								 unsigned int count, u8 *desc,
								 u16 desc_stride, u16 addr_offset,
								 u16 opts2_offset, __le32 opts2,
								 u16 clear_size, int *failure)
{
	struct rtlmq_rx_buffer *buffer = &ring->buffers[start];
	unsigned int prepared;

	for (prepared = 0; prepared < count; prepared++, buffer++,
	     desc += desc_stride) {
		*failure = rtlmq_refill_rx_page_pool_buffer_fast(ring, buffer);
		if (unlikely(*failure)) {
			rtlmq_refill_rx_fast_alloc_fail(ring);
			break;
		}

		rtlmq_refill_rx_sync_for_device(ring, buffer);
		rtlmq_rx_desc_clear_refill_prefix(desc, clear_size);
		rtlmq_rx_desc_qword_write(desc, addr_offset,
					  cpu_to_le64(buffer->dma_addr));
		rtlmq_rx_desc_dword_write(desc, opts2_offset, opts2);
	}

	return prepared;
}

static unsigned int rtlmq_refill_rx_ring_fast_prepare_span(struct rtlmq_rx_ring *ring,
							   unsigned int start, unsigned int count,
							   u8 *desc, u16 desc_stride,
							   u16 addr_offset, u16 opts2_offset,
							   __le32 opts2, int *failure)
{
	struct rtlmq_rx_buffer *buffer = &ring->buffers[start];
	unsigned int prepared;

	for (prepared = 0; prepared < count; prepared++, buffer++,
	     desc += desc_stride) {
		*failure = rtlmq_refill_rx_page_pool_buffer_fast(ring, buffer);
		if (unlikely(*failure)) {
			rtlmq_refill_rx_fast_alloc_fail(ring);
			break;
		}

		rtlmq_refill_rx_sync_for_device(ring, buffer);
		rtlmq_rx_desc_qword_write(desc, addr_offset,
					  cpu_to_le64(buffer->dma_addr));
		rtlmq_rx_desc_dword_write(desc, opts2_offset, opts2);
	}

	return prepared;
}

static void rtlmq_refill_rx_ring_fast_publish_span(u8 *desc, u16 desc_stride, u16 opts1_offset,
						   __le32 opts1, __le32 opts1_eor,
						   unsigned int count, bool set_eor)
{
	unsigned int normal = count;

	/*
	 * The caller issued dma_wmb() after preparing both descriptor spans.
	 * Write precomputed opts1/OWN words without another barrier per slot.
	 */
	if (set_eor && normal)
		normal--;

	while (normal--) {
		rtlmq_rx_desc_dword_write(desc, opts1_offset, opts1);
		desc += desc_stride;
	}

	if (set_eor)
		rtlmq_rx_desc_dword_write(desc, opts1_offset, opts1_eor);
}

static enum rtlmq_rx_refill_failure
rtlmq_rx_refill_failure_from_errno(int error)
{
	if (error == -ENOMEM)
		return RTLMQ_RX_REFILL_ALLOC_FAILED;
	if (error == -EIO)
		return RTLMQ_RX_REFILL_DMA_FAILED;

	return RTLMQ_RX_REFILL_BLOCKED;
}

static void rtlmq_rx_refill_update_nid(struct rtlmq_rx_ring *ring)
{
	if (ring->page_pool)
		page_pool_nid_changed(ring->page_pool, numa_mem_id());
}

unsigned int rtlmq_refill_rx_ring_generic(struct rtlmq_rx_ring *ring,
					  unsigned int cleaned,
					  enum rtlmq_rx_refill_failure *failure)
{
	const struct rtlmq_rx_desc_profile *profile = ring->profile;
	struct rtlmq_rx_ring_stats *stats = ring->cold ? &ring->cold->stats :
						       NULL;
	struct rtlmq_rx_buffer *buffers = ring->buffers;
	unsigned int next_to_alloc = ring->next_to_alloc;
	unsigned int ring_mask = ring->ring_mask;
	u32 buffer_len = ring->buffer_len;
	u16 ring_count = ring->ring_count;
	u8 *desc_base = ring->desc;
	u16 desc_stride = profile->stride;
	unsigned int requested = cleaned;
	unsigned int posted = 0;

	*failure = RTLMQ_RX_REFILL_OK;
	cleaned = min_t(unsigned int, cleaned, ring->ring_count - ring->rx_posted);
	if (!cleaned)
		return 0;

	rtlmq_rx_refill_update_nid(ring);

	while (cleaned--) {
		unsigned int entry = next_to_alloc;
		void *desc = desc_base + entry * desc_stride;
		int ret;

		ret = rtlmq_refill_rx_slot_buffer(ring, &buffers[entry], stats);
		if (ret) {
			*failure = rtlmq_rx_refill_failure_from_errno(ret);
			break;
		}

		rtlmq_refill_rx_sync_for_device(ring, &buffers[entry]);

		if (!rtlmq_publish_rx_desc_refill_fast(profile,
						       desc,
						       &buffers[entry],
						       buffer_len, ring_count,
						       entry)) {
			*failure = buffers[entry].opaque ?
				   RTLMQ_RX_REFILL_BLOCKED :
				   RTLMQ_RX_REFILL_DMA_FAILED;
			break;
		}

		posted++;
		next_to_alloc = (entry + 1) & ring_mask;
	}

	ring->next_to_alloc = next_to_alloc;
	ring->rx_posted += posted;
	rtlmq_xsk_update_rx_need_wakeup(ring, requested, posted);
	return posted;
}

unsigned int rtlmq_refill_rx_ring_fast(struct rtlmq_rx_ring *ring,
				       unsigned int cleaned,
				       enum rtlmq_rx_refill_failure *failure)
{
	const struct rtlmq_rx_desc_profile *profile = ring->profile;
	unsigned int second_prepared = 0;
	unsigned int second_count = 0;
	unsigned int first_prepared;
	unsigned int first_count;
	unsigned int prepared;
	unsigned int start;
	unsigned int entry;
	u16 ring_count = ring->ring_count;
	u8 *desc_base = ring->desc;
	u16 desc_stride = profile->stride;
	u16 addr_offset = profile->addr_offset;
	u16 opts1_offset = profile->opts1_offset;
	u16 opts2_offset = profile->opts2_offset;
	u16 clear_size = ring->rx_refill_clear_size;
	__le32 opts1 = ring->rx_refill_opts1;
	__le32 opts1_eor = ring->rx_refill_opts1_eor;
	__le32 opts2 = ring->rx_refill_opts2;
	int prepare_failure = 0;

	*failure = RTLMQ_RX_REFILL_OK;
	cleaned = min_t(unsigned int, cleaned, ring->ring_count - ring->rx_posted);
	if (!cleaned)
		return 0;

	rtlmq_rx_refill_update_nid(ring);

	start = ring->next_to_alloc;
	first_count = min_t(unsigned int, cleaned, ring_count - start);

	if (clear_size) {
		first_prepared = rtlmq_refill_rx_ring_fast_prepare_span_clear(
			ring, start, first_count,
			desc_base + start * desc_stride, desc_stride,
			addr_offset, opts2_offset, opts2, clear_size,
			&prepare_failure);
		if (first_prepared == first_count && cleaned > first_count) {
			second_count = cleaned - first_count;
			second_prepared =
				rtlmq_refill_rx_ring_fast_prepare_span_clear(
					ring, 0, second_count,
					desc_base, desc_stride, addr_offset,
					opts2_offset, opts2, clear_size,
					&prepare_failure);
		}
	} else {
		first_prepared = rtlmq_refill_rx_ring_fast_prepare_span(
			ring, start, first_count,
			desc_base + start * desc_stride, desc_stride,
			addr_offset, opts2_offset, opts2, &prepare_failure);
		if (first_prepared == first_count && cleaned > first_count) {
			second_count = cleaned - first_count;
			second_prepared =
				rtlmq_refill_rx_ring_fast_prepare_span(ring, 0, second_count,
								       desc_base, desc_stride,
								       addr_offset, opts2_offset,
								       opts2, &prepare_failure);
		}
	}

	prepared = first_prepared + second_prepared;
	if (prepare_failure)
		*failure =
			rtlmq_rx_refill_failure_from_errno(prepare_failure);
	if (!prepared)
		return 0;

	/*
	 * All refill descriptor address/opts2 writes must
	 * be visible before any opts1/OWN write returns a slot to hardware.
	 */
	dma_wmb();

	if (first_prepared)
		rtlmq_refill_rx_ring_fast_publish_span(desc_base + start * desc_stride, desc_stride,
						       opts1_offset, opts1, opts1_eor,
						       first_prepared,
						       start + first_prepared == ring_count);
	if (second_prepared)
		rtlmq_refill_rx_ring_fast_publish_span(desc_base, desc_stride, opts1_offset, opts1,
						       opts1_eor, second_prepared,
						       second_prepared == ring_count);

	entry = start + prepared;
	if (entry >= ring_count)
		entry -= ring_count;
	ring->next_to_alloc = entry;
	ring->rx_posted += prepared;
	return prepared;
}
