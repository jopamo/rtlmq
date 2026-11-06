// SPDX-License-Identifier: GPL-2.0-only

#include <linux/etherdevice.h>
#include <linux/hash.h>
#include <linux/if_vlan.h>
#include <linux/ip.h>
#include <linux/ipv6.h>
#include <linux/overflow.h>
#include <linux/rcupdate.h>
#include <linux/skbuff.h>
#include <linux/tcp.h>
#include <linux/udp.h>
#include <net/ip6_checksum.h>
#include <net/page_pool/helpers.h>
#include <net/xdp_sock_drv.h>

#include "rtlmq_irq.h"
#include "rtlmq.h"
#include "rtlmq_runtime_accessors.h"
#include "rtlmq_ring.h"
#include "rtlmq_tx.h"
#include "rtlmq_trace.h"
#include "rtlmq_tx_internal.h"
#include "rtlmq_tx_private.h"
#include "rtlmq_xdp.h"
#include "rtlmq_ptp.h"
#include "rtlmq_regs.h"

static bool
rtlmq_tx_kick_fast(struct rtlmq_tx_ring *ring);
static netdev_tx_t
rtlmq_xmit_frame_validated(struct sk_buff *skb, struct rtlmq_tx_ring *ring);

#define RTLMQ_TSO_MIN_FIRST_DESC	256U
#define RTLMQ_TSO_PULL_ADMISSION_SLACK	2U

struct rtlmq_xsk_checksum_request {
	u16 start;
	u16 offset;
	bool requested;
};

struct rtlmq_xsk_metadata_request {
	struct xsk_tx_metadata *meta;
	const struct xsk_tx_metadata_ops *ops;
	struct rtlmq_xsk_checksum_request checksum;
	bool timestamp_requested;
};

static void rtlmq_xsk_request_timestamp(void *priv)
{
	struct rtlmq_xsk_metadata_request *request = priv;

	request->timestamp_requested = true;
}

static u64 rtlmq_xsk_fill_timestamp(void *priv)
{
	return *(u64 *)priv;
}

static void rtlmq_xsk_request_checksum(u16 start, u16 offset, void *priv)
{
	struct rtlmq_xsk_metadata_request *request = priv;

	request->checksum.start = start;
	request->checksum.offset = offset;
	request->checksum.requested = true;
}

static const struct xsk_tx_metadata_ops rtlmq_xsk_metadata_policy[] = {
	[RTLMQ_XSK_METADATA_CHECKSUM | RTLMQ_XSK_METADATA_TIMESTAMP] = {
		.tmo_request_timestamp = rtlmq_xsk_request_timestamp,
		.tmo_fill_timestamp = rtlmq_xsk_fill_timestamp,
		.tmo_request_checksum = rtlmq_xsk_request_checksum,
	},
};

const struct xsk_tx_metadata_ops *
rtlmq_xsk_tx_metadata_ops_for_capabilities(u8 capabilities)
{
	if (!capabilities || capabilities >= ARRAY_SIZE(rtlmq_xsk_metadata_policy))
		return NULL;

	return &rtlmq_xsk_metadata_policy[capabilities];
}

static unsigned int rtlmq_tx_construction_tail(const struct rtlmq_tx_ring *ring)
{
	return READ_ONCE(ring->serial.stage) & ring->ring_mask;
}

static unsigned int rtlmq_tx_descs_for_len(unsigned int len,
					   unsigned int max_len)
{
	if (!len)
		return 0;
	if (!max_len)
		return UINT_MAX;

	/*
	 * Avoid DIV_ROUND_UP(len, max_len)'s len + max_len - 1 addition:
	 * skb lengths are externally derived and may already be near UINT_MAX.
	 */
	return (len - 1) / max_len + 1;
}

static unsigned int rtlmq_tx_tso_descs_needed(const struct sk_buff *skb)
{
	unsigned int frag_descs;
	unsigned int needed;
	int i;

	needed = rtlmq_tx_descs_for_len(skb_headlen(skb),
					RTLMQ_TX_TSO_MAX_SEG_LEN);

	for (i = 0; i < skb_shinfo(skb)->nr_frags; i++) {
		frag_descs = rtlmq_tx_descs_for_len(skb_frag_size(&skb_shinfo(skb)->frags[i]),
						    RTLMQ_TX_TSO_MAX_SEG_LEN);
		if (check_add_overflow(needed, frag_descs, &needed))
			return UINT_MAX;
	}

	return needed;
}

/*
 * RTL8125B TSO cannot start on a header-only descriptor. Pull enough payload
 * into the linear head before descriptor accounting so the final skb shape is
 * the one we count, map, split, and publish.
 */
static bool rtlmq_tso_pull_first_payload(struct sk_buff *skb)
{
	unsigned int payload_pull;
	unsigned int hdr_len;
	unsigned int thoff;
	unsigned int thlen;
	unsigned int want;

	if (!skb_is_gso(skb))
		return true;

	thoff = skb_transport_offset(skb);
	if (unlikely(check_add_overflow(thoff, (unsigned int)sizeof(struct tcphdr),
					&want) ||
		     !pskb_may_pull(skb, want)))
		goto fail;

	thlen = tcp_hdrlen(skb);
	if (unlikely(check_add_overflow(thoff, thlen, &hdr_len)))
		goto fail;

	if (skb_headlen(skb) > hdr_len)
		return true;

	payload_pull = min_t(unsigned int,
			     skb_shinfo(skb)->gso_size,
			     RTLMQ_TSO_MIN_FIRST_DESC);
	if (unlikely(check_add_overflow(hdr_len, payload_pull, &want)))
		goto fail;
	want = min_t(unsigned int, skb->len, want);

	if (skb_headlen(skb) >= want)
		return true;

	if (unlikely(!pskb_may_pull(skb, want)))
		goto fail;

	return true;

fail:
	return false;
}

static void
rtlmq_init_tx_info(struct rtlmq_tx_info *info,
		   u32 desc_len, dma_addr_t unmap_dma, u32 unmap_len,
		   u32 generation, u16 flags)
{
	info->unmap_dma = unmap_dma;
	info->desc_len = desc_len;
	info->unmap_len = unmap_len;
	info->generation = generation;
	info->completion_packets = 0;
	info->flags = flags;
	if (unmap_len)
		info->flags |= RTLMQ_TX_INFOF_UNMAP_OWNER;
}

static void rtlmq_set_tx_completion(struct rtlmq_tx_completion *completion,
				    enum rtlmq_tx_buffer_type type,
				    void *cookie, u32 completion_bytes)
{
	switch (type) {
	case RTLMQ_TX_BUFFER_SKB:
		completion->skb = cookie;
		break;
	case RTLMQ_TX_BUFFER_XDP_NDO:
	case RTLMQ_TX_BUFFER_XDP_RX:
		completion->xdpf = cookie;
		break;
	case RTLMQ_TX_BUFFER_XSK:
		completion->xsk_pool = cookie;
		break;
	case RTLMQ_TX_BUFFER_NONE:
	default:
		completion->xdpf = NULL;
		break;
	}

	completion->type = type;
	completion->completion_bytes = completion_bytes;
	completion->timestamp_generation = 0;
	completion->xsk_descs = 0;
	completion->flags = 0;
}

static void
rtlmq_set_tx_skb_completion_fast(struct rtlmq_tx_completion *completion,
				 struct sk_buff *skb, u32 completion_bytes)
{
	completion->skb = skb;
	completion->type = RTLMQ_TX_BUFFER_SKB;
	completion->completion_bytes = completion_bytes;
	completion->timestamp_generation = 0;
	completion->xsk_descs = 0;
	completion->flags = 0;
}

static void rtlmq_tx_bql_enqueue_prefetchw(struct netdev_queue *txq)
{
	netdev_txq_bql_enqueue_prefetchw(txq);
}

static bool rtlmq_tx_bql_sent(struct rtlmq_tx_ring *ring, u32 bytes,
			      bool xmit_more)
{
	struct netdev_queue *txq = rtlmq_tx_ring_rm(ring)->txq;
	bool doorbell;

	rtlmq_tx_bql_enqueue_prefetchw(txq);
	doorbell = __netdev_tx_sent_queue(txq, bytes, xmit_more);

	return doorbell;
}

static bool rtlmq_tx_publish_account_bql(struct rtlmq_tx_ring *ring,
					 u32 completion_bytes, bool xmit_more)
{
	bool doorbell;

	doorbell = rtlmq_tx_bql_sent(ring, completion_bytes, xmit_more);
	/* Charge one BQL object per accepted packet, including one TSO skb. */
	ring->bql_queued_packets++;
	ring->bql_queued_bytes += completion_bytes;

	return doorbell;
}

/*
 * Staged TX descriptor ordering:
 * 1. Prepare descriptors, completion metadata, and DMA mappings under the
 *    netdev queue's xmit lock.
 * 2. Charge BQL before staging or hardware publication.
 * 3. Use dma_wmb() before setting RTLMQ_DESC_OWN so descriptor data is
 *    device-visible before hardware owns the slot.
 * 4. The shared publisher stores the index before releasing the hardware
 *    frontier. Reclaim acquires descriptor stores, completion metadata and
 *    BQL accounting. Hardware may clear OWN before a new doorbell.
 * 5. Mark tx_doorbell_pending and flush only after producer tail/BQL
 *    publication is complete.
 * 6. Use wmb() before the TX_POLL MMIO producer write; never rely on
 *    DMA barriers for doorbell ordering and do not synchronously read back a
 *    posted doorbell when no following operation depends on its completion.
 *
 * Fragment and TSO chains keep the first descriptor unowned until all later
 * descriptors and completion metadata are ready; first-OWN is the final
 * hardware handoff.
 */
static void
rtlmq_tx_flush_doorbell_if_pending_hot(struct rtlmq_tx_ring *ring)
{
		if (ring->tx_doorbell_pending)
			rtlmq_serial_tx_notify(ring);
		return;
}

static void
rtlmq_tx_flush_doorbell_after_publish(struct rtlmq_tx_ring *ring, bool doorbell)
{
	if (doorbell)
		rtlmq_tx_flush_doorbell_if_pending_hot(ring);
}

bool rtlmq_tx_ring_boundary_ready(struct rtlmq_tx_ring *ring)
{
	const struct rtlmq_tx_ring_read_mostly *rm;
	struct rtlmq_private *tp;
	u16 published_txq;

	if (unlikely(!ring || !ring->priv))
		return false;

	rm = rtlmq_tx_ring_rm(ring);
	tp = ring->priv;
	if (unlikely(!rtlmq_mmio_ordinary_authorized(tp)))
		return false;
	if (unlikely(READ_ONCE(tp->state) != RTLMQ_STATE_OPEN))
		return false;

	if (unlikely(!rm->ndev || !netif_device_present(rm->ndev)))
		return false;

	if (unlikely(!rm->desc || !rm->infos || !rm->completions ||
		     !rm->dma_txns ||
		     !rm->txq || !rm->ndev_queue ||
		     !rm->doorbell))
		return false;

	published_txq = rtlmq_published_num_tx_queues(tp);
	if (unlikely(!published_txq || rm->queue_index >= published_txq))
		return false;

	if (unlikely(rm->queue_index >= RTLMQ_TX_POLL_KICK_MAX_QUEUES))
		return false;

	if (unlikely(READ_ONCE(tp->txq2ring[rm->queue_index]) != ring))
		return false;

	return true;
}

static bool
rtlmq_tx_descs_available_fast(struct rtlmq_tx_ring *ring,
			      const struct rtlmq_tx_ring_read_mostly *rm,
			      unsigned int needed, u32 *pending_before)
{
	u32 use_count;
	/* Pairs with rtlmq_tx_publish_clean_batch() after next_to_clean. */
	u32 clean_count = smp_load_acquire(&ring->next_to_clean_count);
	{
		use_count = READ_ONCE(ring->serial.stage);
		*pending_before =
			rtlmq_tx_ring_distance(use_count, clean_count);
		return *pending_before < rm->ring_count &&
		       needed < rm->ring_count - *pending_before &&
		       rtlmq_tx_ring_has_stage_space(ring, needed);
	}
}

static noinline bool
rtlmq_tx_stop_and_recheck_admission(struct rtlmq_tx_ring *ring,
				    const struct rtlmq_tx_ring_read_mostly *rm,
				    unsigned int needed, u32 *pending_before)
{
	int stop_result;
	bool available;

	/*
	 * A previous xmit_more publication may still be waiting for its batch
	 * boundary. Flush that published work before making the queue stop
	 * visible; otherwise the stopped queue can retain descriptors that
	 * hardware was never notified about.
	 */
	rtlmq_tx_flush_doorbell_if_pending_hot(ring);
	stop_result = rtlmq_tx_maybe_stop_subqueue(ring, needed);
	rtlmq_publish_tx_datapath_subqueue_state(ring,
						 stop_result ? RTLMQ_TX_SUBQUEUE_READY :
							       RTLMQ_TX_SUBQUEUE_BACKPRESSURED);

	/*
	 * netif_txq_maybe_stop() already publishes the stop and rechecks current
	 * descriptor availability with the core stop/wake barrier pairing.
	 * Re-evaluate the packet-specific reservation here because availability
	 * below the wake high-water mark can still satisfy this transmission.
	 */
	available = rtlmq_tx_descs_available_fast(ring, rm, needed, pending_before);
	if (available && stop_result)
		rtlmq_publish_tx_datapath_subqueue_state(ring, RTLMQ_TX_SUBQUEUE_READY);

	return available;
}

static __always_inline bool
rtlmq_tx_admit_descs(struct rtlmq_tx_ring *ring,
		     const struct rtlmq_tx_ring_read_mostly *rm,
		     unsigned int needed, u32 *pending_before)
{
	if (likely(rtlmq_tx_descs_available_fast(ring, rm, needed,
						 pending_before)))
		return true;

	return rtlmq_tx_stop_and_recheck_admission(ring, rm, needed,
						  pending_before);
}

static noinline void
rtlmq_tx_stop_and_recheck_after_publish(struct rtlmq_tx_ring *ring)
{
	int stop_result;

	stop_result = rtlmq_tx_maybe_stop_subqueue(ring, rtlmq_tx_stop_desc_low(ring));
	rtlmq_publish_tx_datapath_subqueue_state(ring,
						 stop_result ? RTLMQ_TX_SUBQUEUE_READY :
							       RTLMQ_TX_SUBQUEUE_BACKPRESSURED);
}

static noinline __noclone enum rtlmq_tx_publish_result
rtlmq_publish_tx_desc_tail_fast(struct rtlmq_tx_ring *ring,
				const struct rtlmq_tx_ring_read_mostly *rm,
				unsigned int needed, u32 completion_bytes,
				bool xmit_more, bool force_flush, bool single_desc,
				u32 pending_before)
{
	bool desc_backpressured;
	bool single_pressure;
	u32 pending_after;
	bool doorbell;

	/*
	 * Charge BQL before staged acceptance and hardware-frontier release.
	 * NAPI can complete this packet as soon as it sees that count, even
	 * before the doorbell. Native fixtures have already released OWN.
	 * The TX_POLL helper separately orders MMIO.
	 */
	pending_after = pending_before + needed;
	doorbell = rtlmq_tx_publish_account_bql(ring, completion_bytes,
						xmit_more);
	desc_backpressured = rm->ring_count - pending_after - 1 <
			     rm->tx_stop_desc_low;
	{
		bool direct = rtlmq_tx_can_publish_fast(ring);

		rtlmq_serial_tx_stage(ring, needed, completion_bytes);
		if (likely(direct)) {
			/* The admission check authorizes this complete packet. */
			rtlmq_tx_publish_fast(ring, needed);
			if (trace_rtlmq_tx_publish_range_enabled()) {
				u32 end = READ_ONCE(ring->next_to_use_count);

				trace_rtlmq_tx_publish_range(rm->ndev->name, ring->queue_index,
							     READ_ONCE(ring->generation),
							     end - needed, end, 1);
			}
			if (unlikely(!ring->serial_control.watching))
				rtlmq_serial_tx_watch(ring);
			goto notify;
		}
		rtlmq_serial_tx_stage_pending(ring);
		desc_backpressured = rtlmq_tx_desc_avail(ring) <
				     rm->tx_stop_desc_low;
		goto check_capacity;
	}

notify:
	if (unlikely(desc_backpressured))
		doorbell = true;
	else if (unlikely(force_flush))
		doorbell = true;
	single_pressure = single_desc &&
			  rtlmq_tx_ring_single_desc_pressure(ring);
	if (unlikely(single_pressure))
		doorbell = true;
	if (doorbell) {
		/*
		 * Pair with the disarm barrier in rtlmq_sync_vector_tx_irq_arming():
		 * the producer and the rescan must not both miss the other's store.
		 */
		smp_mb();
		if (unlikely(!rtlmq_tx_ring_tx_irq_wanted(ring)))
			rtlmq_sync_tx_ring_irq_arming(ring);
	}
	rtlmq_tx_flush_doorbell_after_publish(ring, doorbell);

check_capacity:
	if (unlikely(desc_backpressured))
		rtlmq_tx_stop_and_recheck_after_publish(ring);

	return RTLMQ_TX_PUBLISHED;
}

struct rtlmq_tso_emit_state {
	const struct rtlmq_tx_ring_read_mostly *rm;
	struct rtlmq_tx_ring *ring;
	struct sk_buff *skb;
	u32 desc_opts1;
	u32 desc_opts2;
	u32 generation;
	u16 info_flags;
	u16 completion_packets;
	u32 completion_bytes;
	unsigned int needed;
	unsigned int entry;
	unsigned int mapped;
};

struct rtlmq_tx_dma_sg_cursor {
	struct scatterlist *sg;
	unsigned int nents;
	unsigned int index;
	dma_addr_t dma_addr;
	u32 dma_len;
};

static unsigned int rtlmq_tx_desc_nth(const struct rtlmq_tx_ring *ring,
				      unsigned int entry, unsigned int n)
{
	return (entry + n) & ring->ring_mask;
}

static bool
rtlmq_tx_dma_sg_cursor_advance(struct rtlmq_tx_dma_sg_cursor *cursor)
{
	while (cursor->index < cursor->nents) {
		struct scatterlist *sg = &cursor->sg[cursor->index++];

		if (!sg_dma_len(sg))
			continue;
		cursor->dma_addr = sg_dma_address(sg);
		cursor->dma_len = sg_dma_len(sg);
		return true;
	}

	cursor->dma_addr = 0;
	cursor->dma_len = 0;
	return false;
}

static bool rtlmq_tx_dma_sg_cursor_take(struct rtlmq_tx_dma_sg_cursor *cursor,
					u32 len, dma_addr_t *dma_addr)
{
	bool have_expected = false;
	dma_addr_t expected = 0;
	u32 remaining = len;

	if (!cursor || !dma_addr || !len)
		return false;
	if (!cursor->dma_len && !rtlmq_tx_dma_sg_cursor_advance(cursor))
		return false;

	*dma_addr = cursor->dma_addr;
	while (remaining) {
		u32 chunk;

		if (!cursor->dma_len) {
			if (!rtlmq_tx_dma_sg_cursor_advance(cursor) ||
			    !have_expected || cursor->dma_addr != expected)
				return false;
		}

		chunk = min(remaining, cursor->dma_len);
		if (!rtlmq_dma_addr_add_checked(cursor->dma_addr, chunk,
						&expected))
			return false;
		cursor->dma_addr = expected;
		cursor->dma_len -= chunk;
		remaining -= chunk;
		have_expected = true;
	}

	return true;
}

static int rtlmq_tx_map_tso_sg(struct rtlmq_tx_ring *ring,
			       struct rtlmq_tx_dma_txn *txn,
			       struct sk_buff *skb)
{
	const struct skb_shared_info *shinfo = skb_shinfo(skb);
	unsigned int orig_nents;
	unsigned int i;
	int dma_nents;

	if (check_add_overflow(shinfo->nr_frags, 1U, &orig_nents))
		return 0;
	if (WARN_ON_ONCE(!txn || txn->mapped ||
			 !orig_nents || orig_nents > RTLMQ_TX_DMA_SG_NENTS))
		return 0;

	sg_init_table(txn->sg, orig_nents);
	sg_set_buf(&txn->sg[0], skb->data, skb_headlen(skb));
	for (i = 0; i < shinfo->nr_frags; i++) {
		const skb_frag_t *frag = &shinfo->frags[i];

		sg_set_page(&txn->sg[i + 1], skb_frag_page(frag),
			    skb_frag_size(frag), skb_frag_off(frag));
	}

	dma_nents = rtlmq_tx_dma_map_sg(ring->priv, txn->sg, orig_nents);
	if (dma_nents <= 0)
		return 0;

	WRITE_ONCE(txn->orig_nents, orig_nents);
	WRITE_ONCE(txn->mapped, true);
	return dma_nents;
}

static bool rtlmq_emit_tso_split_range(struct rtlmq_tso_emit_state *emit,
				       dma_addr_t dma_addr, u32 len)
{
	unsigned int offset;

	for (offset = 0; offset < len; ) {
		u32 chunk = min_t(u32, len - offset, RTLMQ_TX_TSO_MAX_SEG_LEN);
		struct rtlmq_tx_info *info = &emit->rm->infos[emit->entry];
		u32 opts1 = emit->desc_opts1;
		dma_addr_t desc_dma;
		bool final_desc;
		u16 info_flags;

		if (unlikely(!rtlmq_dma_addr_add_checked(dma_addr, offset,
							 &desc_dma)))
			return false;

		final_desc = emit->mapped + 1 == emit->needed;
		info_flags = emit->info_flags;
		if (final_desc)
			info_flags |= RTLMQ_TX_INFOF_UNMAP_OWNER;

		rtlmq_init_tx_info(info, chunk,
				   0, 0,
				   emit->generation, info_flags);
		if (!emit->mapped)
			opts1 |= RTLMQ_DESC_FIRST_FRAG;
		else
			opts1 |= RTLMQ_DESC_OWN;

		if (final_desc) {
			struct rtlmq_tx_completion *completion =
				&emit->rm->completions[emit->entry];

			opts1 |= RTLMQ_DESC_LAST_FRAG;
			info->flags |= RTLMQ_TX_INFOF_EOP;
			info->completion_packets = emit->completion_packets;
			rtlmq_set_tx_completion(completion,
						RTLMQ_TX_BUFFER_SKB,
						emit->skb,
						emit->completion_bytes);
		}

		rtlmq_write_tx_desc_staged(emit->ring, emit->entry, desc_dma,
					   chunk, opts1, emit->desc_opts2);
		emit->mapped++;
		offset += chunk;
		if (emit->mapped < emit->needed)
			emit->entry =
				rtlmq_tx_desc_advance(emit->ring, emit->entry);
	}

	return true;
}

static enum rtlmq_tx_publish_result
rtlmq_drop_tx_skb(struct rtlmq_tx_ring *ring, struct sk_buff *skb)
{
	u64_stats_update_begin(&ring->syncp);
	ring->stats.drops++;
	u64_stats_update_end(&ring->syncp);
	dev_kfree_skb_any(skb);

	return RTLMQ_TX_DROPPED;
}

static enum rtlmq_tx_publish_result
rtlmq_unwind_and_drop_tx_skb(struct rtlmq_tx_ring *ring, struct sk_buff *skb,
			     unsigned int first_entry, unsigned int mapped)
{
	rtlmq_unwind_tx_entries(ring, first_entry, mapped);
	return rtlmq_drop_tx_skb(ring, skb);
}

static enum rtlmq_tx_publish_result
rtlmq_commit_single_tx_desc(struct rtlmq_tx_ring *ring, dma_addr_t dma_addr, u32 len, u32 opts2,
			    enum rtlmq_tx_buffer_type type, void *cookie, u16 info_flags,
			    u16 completion_packets, u32 completion_bytes, bool flush_doorbell)
{
	const struct rtlmq_tx_ring_read_mostly *rm = rtlmq_tx_ring_rm(ring);
	/*
	 * Caller must hold the netdev queue's xmit lock to serialize writes to
	 * next_to_use and new descriptors. NAPI updates next_to_clean.
	 */
	struct rtlmq_tx_completion *completion;
	struct rtlmq_tx_info *info;
	unsigned int entry;
	unsigned int desc_avail_before;
	u32 pending_before;
	u32 generation;

	desc_avail_before = rtlmq_tx_desc_avail(ring);
	pending_before = rm->ring_count - desc_avail_before - 1;
	if (unlikely(!desc_avail_before) &&
	    !rtlmq_tx_stop_and_recheck_admission(ring, rm, 1, &pending_before))
		return RTLMQ_TX_BUSY;

	entry = rtlmq_tx_construction_tail(ring);
	/* Pairs with rtlmq_tx_ring_publish_generation() before slot reuse. */
	generation = smp_load_acquire(&ring->generation);
	info = &rm->infos[entry];
	completion = &rm->completions[entry];
	rtlmq_init_tx_info(info, len,
			   info_flags & RTLMQ_TX_INFOF_MAPPED ? dma_addr : 0,
			   info_flags & RTLMQ_TX_INFOF_MAPPED ? len : 0,
			   generation, info_flags | RTLMQ_TX_INFOF_EOP);
	info->completion_packets = completion_packets;
	rtlmq_set_tx_completion(completion, type, cookie, completion_bytes);
	if (type == RTLMQ_TX_BUFFER_SKB)
		rtlmq_ptp_tx_prepare(ring->priv, completion->skb, completion);

	rtlmq_write_tx_desc_staged(ring, entry, dma_addr, len,
				   RTLMQ_DESC_FIRST_FRAG |
				   RTLMQ_DESC_LAST_FRAG |
				   RTLMQ_DESC_OWN, opts2);
	return rtlmq_publish_tx_desc_tail_fast(ring, rm,
					       1, completion_bytes,
					       !flush_doorbell, flush_doorbell, false,
					       pending_before);
}

static unsigned int rtlmq_xdp_frame_tailroom(const struct xdp_frame *xdpf)
{
	u8 *hard_start;
	u8 *hard_end;
	u8 *data_end;

	hard_start = (u8 *)xdpf->data - xdpf->headroom - sizeof(*xdpf);
	hard_end = hard_start + xdpf->frame_sz -
		   SKB_DATA_ALIGN(sizeof(struct skb_shared_info));
	data_end = (u8 *)xdpf->data + xdpf->len;
	if (data_end > hard_end)
		return 0;

	return hard_end - data_end;
}

static int rtlmq_pad_xdp_frame(struct xdp_frame *xdpf)
{
	unsigned int pad_len;

	if (xdpf->len >= ETH_ZLEN)
		return 0;

	pad_len = ETH_ZLEN - xdpf->len;
	if (rtlmq_xdp_frame_tailroom(xdpf) < pad_len)
		return -ENOMEM;

	memset((u8 *)xdpf->data + xdpf->len, 0, pad_len);
	xdpf->len = ETH_ZLEN;

	return 0;
}

static bool rtlmq_xdp_external_frame_valid(const struct xdp_frame *xdpf)
{
	const struct skb_shared_info *sinfo;
	unsigned int data_offset;
	unsigned int seg_count;
	uintptr_t hard_start;
	uintptr_t data_addr;
	unsigned int usable;
	u32 frag_bytes = 0;
	unsigned int i;
	u32 total_len;

	/*
	 * Validate each frame before transmitting any of the batch so an error
	 * leaves every frame unchanged and the caller can return them.
	 */
	if (!xdpf || !xdpf->data || !xdpf->len ||
	    xdpf->len > RTLMQ_TX_DESC_LEN_MASK ||
	    xdpf->mem_type >= MEM_TYPE_MAX ||
	    xdpf->mem_type == MEM_TYPE_XSK_BUFF_POOL ||
	    xdpf->metasize > xdpf->headroom)
		return false;

	if (check_add_overflow((unsigned int)sizeof(*xdpf), xdpf->headroom,
			       &data_offset))
		return false;
	data_addr = (uintptr_t)xdpf->data;
	if (data_addr < data_offset ||
	    xdpf->frame_sz < SKB_DATA_ALIGN(sizeof(*sinfo)))
		return false;
	hard_start = data_addr - data_offset;
	if (xdpf->frame_sz > ULONG_MAX - hard_start)
		return false;
	usable = xdpf->frame_sz - SKB_DATA_ALIGN(sizeof(*sinfo));
	if (data_offset > usable || xdpf->len > usable - data_offset)
		return false;

	if (!xdp_frame_has_frags(xdpf))
		return xdpf->len >= ETH_HLEN;

	sinfo = xdp_get_shared_info_from_frame(xdpf);
	if (!sinfo->nr_frags ||
	    check_add_overflow((unsigned int)sinfo->nr_frags, 1U,
			       &seg_count) ||
	    seg_count > RTLMQ_XSK_MAX_SEGS)
		return false;
	for (i = 0; i < sinfo->nr_frags; i++) {
		u32 len = skb_frag_size(&sinfo->frags[i]);

		if (!len || len > RTLMQ_TX_DESC_LEN_MASK)
			return false;
		if (check_add_overflow(frag_bytes, len, &frag_bytes))
			return false;
	}

	if (sinfo->xdp_frags_size != frag_bytes ||
	    check_add_overflow(xdpf->len, frag_bytes, &total_len))
		return false;

	return total_len >= ETH_HLEN;
}

static int rtlmq_xdp_frame_dma_addr(const struct xdp_frame *xdpf,
				    dma_addr_t *dma_addr)
{
	struct page *page;
	ptrdiff_t offset;
	void *page_addr;

	if (xdpf->mem_type != MEM_TYPE_PAGE_POOL)
		return -EINVAL;

	page = virt_to_head_page(xdpf->data);
	page_addr = page_address(page);
	if (!page_addr)
		return -EINVAL;

	offset = (u8 *)xdpf->data - (u8 *)page_addr;
	if (offset < 0)
		return -EINVAL;
	if ((size_t)offset > xdpf->frame_sz ||
	    xdpf->len > xdpf->frame_sz - (size_t)offset)
		return -EINVAL;

	if (!rtlmq_dma_addr_add_checked(page_pool_get_dma_addr(page),
					(u64)(size_t)offset, dma_addr))
		return -EINVAL;

	return 0;
}

static enum rtlmq_tx_publish_result
rtlmq_publish_tx_xdp_frags(struct rtlmq_tx_ring *ring, struct xdp_frame *xdpf,
			   enum rtlmq_tx_buffer_type type, bool flush_doorbell)
{
	const struct rtlmq_tx_ring_read_mostly *rm = rtlmq_tx_ring_rm(ring);
	const struct skb_shared_info *sinfo =
		xdp_get_shared_info_from_frame(xdpf);
	struct rtlmq_private *tp = ring->priv;
	unsigned int first_entry, entry;
	unsigned int pending_before;
	unsigned int needed;
	unsigned int total_bytes;
	unsigned int pad_len = 0;
	unsigned int mapped = 0;
	unsigned int next_total;
	dma_addr_t first_dma;
	u32 generation;
	unsigned int i;

	if (check_add_overflow(xdpf->len, sinfo->xdp_frags_size, &total_bytes) ||
	    check_add_overflow((unsigned int)sinfo->nr_frags, 1U, &needed))
		return RTLMQ_TX_DROPPED;
	if (total_bytes < ETH_ZLEN) {
		pad_len = ETH_ZLEN - total_bytes;
		if (rtlmq_xdp_frame_tailroom(xdpf) < pad_len)
			return RTLMQ_TX_DROPPED;
		needed++;
	}

	if (!xdpf->len || xdpf->len > RTLMQ_TX_DESC_LEN_MASK ||
	    !sinfo->nr_frags || needed > RTLMQ_XSK_MAX_SEGS)
		return RTLMQ_TX_DROPPED;
	if (unlikely(!rtlmq_tx_admit_descs(ring, rm, needed,
					   &pending_before)))
		return RTLMQ_TX_BUSY;

	first_entry = rtlmq_tx_construction_tail(ring);
	entry = first_entry;
	/* Pairs with rtlmq_tx_ring_publish_generation() before slot reuse. */
	generation = smp_load_acquire(&ring->generation);
	/* Append head-tailroom zeros after the fragments, not between them. */
	if (pad_len)
		memset((u8 *)xdpf->data + xdpf->len, 0, pad_len);
	first_dma = rtlmq_tx_dma_map_single(tp, xdpf->data, xdpf->len);
	if (rtlmq_tx_dma_mapping_error(tp, first_dma))
		return RTLMQ_TX_DROPPED;

	rtlmq_init_tx_info(&rm->infos[entry], xdpf->len,
			   first_dma, xdpf->len,
			   generation,
			   RTLMQ_TX_INFOF_MAPPED);
	mapped = 1;
	total_bytes = xdpf->len;

	for (i = 0; i < sinfo->nr_frags; i++) {
		const skb_frag_t *frag = &sinfo->frags[i];
		dma_addr_t dma;
		u32 len = skb_frag_size(frag);
		u32 opts1 = RTLMQ_DESC_OWN;

		if (!len || len > RTLMQ_TX_DESC_LEN_MASK ||
		    check_add_overflow(total_bytes, len, &next_total))
			goto unwind;
		entry = rtlmq_tx_desc_advance(ring, entry);
		dma = rtlmq_tx_dma_map_frag(tp, frag, 0, len);
		if (rtlmq_tx_dma_mapping_error(tp, dma))
			goto unwind;
		rtlmq_init_tx_info(&rm->infos[entry], len,
				   dma, len,
				   generation, RTLMQ_TX_INFOF_MAPPED |
				   RTLMQ_TX_INFOF_PAGE);
		if (i + 1 == sinfo->nr_frags && !pad_len)
			opts1 |= RTLMQ_DESC_LAST_FRAG;
		rtlmq_write_tx_desc_staged(ring, entry, dma, len, opts1, 0);
		total_bytes = next_total;
		mapped++;
	}

	if (pad_len) {
		dma_addr_t dma;

		entry = rtlmq_tx_desc_advance(ring, entry);
		dma = rtlmq_tx_dma_map_single(tp, (u8 *)xdpf->data + xdpf->len,
					      pad_len);
		if (rtlmq_tx_dma_mapping_error(tp, dma))
			goto unwind;
		rtlmq_init_tx_info(&rm->infos[entry], pad_len, dma, pad_len,
				   generation, RTLMQ_TX_INFOF_MAPPED);
		rtlmq_write_tx_desc_staged(ring, entry, dma, pad_len,
					   RTLMQ_DESC_OWN |
					   RTLMQ_DESC_LAST_FRAG, 0);
		total_bytes += pad_len;
		mapped++;
	}

	rm->infos[entry].flags |= RTLMQ_TX_INFOF_EOP;
	rm->infos[entry].completion_packets = 1;
	rtlmq_set_tx_completion(&rm->completions[entry], type, xdpf,
				total_bytes);
	rtlmq_write_tx_desc_staged(ring, first_entry, first_dma, xdpf->len,
				   RTLMQ_DESC_FIRST_FRAG, 0);
	return rtlmq_publish_tx_desc_tail_fast(ring, rm,
					       needed, total_bytes,
					       !flush_doorbell, flush_doorbell, false,
					       pending_before);

unwind:
	rtlmq_unwind_tx_entries(ring, first_entry, mapped);
	return RTLMQ_TX_DROPPED;
}

static enum rtlmq_tx_publish_result
rtlmq_publish_tx_xdp_frame_validated(struct rtlmq_tx_ring *ring, struct xdp_frame *xdpf,
				     enum rtlmq_tx_buffer_type type, bool dma_map,
				     bool flush_doorbell)
{
	const struct rtlmq_tx_ring_read_mostly *rm = rtlmq_tx_ring_rm(ring);
	struct rtlmq_private *tp = ring->priv;
	enum rtlmq_tx_publish_result ret;
	u32 original_len = xdpf->len;
	dma_addr_t dma_addr;
	u16 info_flags = 0;
	u32 pending_before;
	int err;

	if (unlikely(!rtlmq_tx_admit_descs(ring, rm, 1,
					   &pending_before)))
		return RTLMQ_TX_BUSY;

	if (unlikely(xdp_frame_has_frags(xdpf)))
		return rtlmq_publish_tx_xdp_frags(ring, xdpf, type,
						 flush_doorbell);

	err = rtlmq_pad_xdp_frame(xdpf);
	if (err)
		return RTLMQ_TX_DROPPED;

	if (dma_map) {
		dma_addr = rtlmq_tx_dma_map_single(tp, xdpf->data, xdpf->len);
		if (rtlmq_tx_dma_mapping_error(tp, dma_addr))
			goto restore_len;
		info_flags |= RTLMQ_TX_INFOF_MAPPED;
	} else {
		err = rtlmq_xdp_frame_dma_addr(xdpf, &dma_addr);
		if (err)
			goto restore_len;
		if (!tp->pdev ||
		    !rtlmq_dma_range_fits_device(&tp->pdev->dev, dma_addr,
						 xdpf->len))
			goto restore_len;

		rtlmq_tx_dma_sync_for_device(tp, dma_addr, xdpf->len,
					     DMA_BIDIRECTIONAL);
	}

	ret = rtlmq_commit_single_tx_desc(ring, dma_addr, xdpf->len, 0,
					  type, xdpf, info_flags,
					  1, xdpf->len, flush_doorbell);
	if (ret != RTLMQ_TX_PUBLISHED &&
	    info_flags & RTLMQ_TX_INFOF_MAPPED)
		rtlmq_tx_dma_unmap_single_producer_owned(ring, dma_addr, xdpf->len);
	if (ret != RTLMQ_TX_PUBLISHED)
		xdpf->len = original_len;

	return ret;

restore_len:
	xdpf->len = original_len;
	return RTLMQ_TX_DROPPED;
}

enum rtlmq_tx_publish_result
rtlmq_publish_tx_xdp_frame(struct rtlmq_tx_ring *ring, struct xdp_frame *xdpf,
			   enum rtlmq_tx_buffer_type type, bool dma_map,
			   bool flush_doorbell)
{
	if (unlikely(!xdpf))
		return RTLMQ_TX_DROPPED;

	return rtlmq_publish_tx_xdp_frame_validated(ring, xdpf, type,
						    dma_map, flush_doorbell);
}

static int rtlmq_validate_xsk_tx_chain(const struct xdp_desc *descs,
				       unsigned int num_descs)
{
	unsigned int i;

	if (!descs || !num_descs || num_descs > RTLMQ_XSK_MAX_SEGS)
		return -EINVAL;

	for (i = 0; i < num_descs; i++) {
		u32 required = i + 1 < num_descs ? XDP_PKT_CONTD : 0;
		u32 allowed = required |
			(i ? 0 : XDP_TX_METADATA);

		if (!descs[i].len ||
		    descs[i].len > RTLMQ_TX_DESC_LEN_MASK ||
		    (descs[i].options & ~allowed) ||
		    (descs[i].options & XDP_PKT_CONTD) != required)
			return -EINVAL;
	}

	return 0;
}

/*
 * All non-GSO checksum consumers share this format-selected encoding.
 * Legacy descriptors carry TCPHO in opts2 for both IP versions. V3 carries
 * it in opts1 for IPv6 and lets the IPv4 selector derive the transport
 * header. Hardware checksum offload is not supported for inline VLAN frames.
 */
static inline bool
rtlmq_tx_checksum_desc_opts(enum rtlmq_tx_desc_format format,
			    __be16 protocol, u8 l4_proto,
			    u32 transport_offset, bool inline_vlan,
			    u32 *opts1, u32 *opts2)
{
	u32 encoded_opts1 = 0;
	u32 encoded_opts2;
	bool ipv4;

	if (unlikely(!opts1 || !opts2 || inline_vlan || !transport_offset))
		return false;

	switch (protocol) {
	case htons(ETH_P_IP):
		ipv4 = true;
		break;
	case htons(ETH_P_IPV6):
		ipv4 = false;
		break;
	default:
		return false;
	}

	if (l4_proto == IPPROTO_TCP)
		encoded_opts2 = RTLMQ_TX_DESC_CSUM_TCP;
	else if (l4_proto == IPPROTO_UDP)
		encoded_opts2 = RTLMQ_TX_DESC_CSUM_UDP;
	else
		return false;

	switch (format) {
	case RTLMQ_TX_DESC_FORMAT_LEGACY:
		if (transport_offset > RTLMQ_TX_DESC_TCPHO_MAX)
			return false;
		encoded_opts2 |= ipv4 ? RTLMQ_TX_DESC_CSUM_IP :
					RTLMQ_TX_DESC_CSUM_IPV6;
		encoded_opts2 |=
			transport_offset << RTLMQ_TX_DESC_TCPHO_SHIFT;
		break;
	case RTLMQ_TX_DESC_FORMAT_V3:
		if (ipv4) {
			encoded_opts2 |= RTLMQ_TX_DESC_CSUM_IP;
		} else {
			if (transport_offset > RTLMQ_TX_DESC_TCPHO_MAX)
				return false;
			encoded_opts1 =
				transport_offset << RTLMQ_TX_DESC_TCPHO_SHIFT;
		}
		break;
	default:
		return false;
	}

	*opts1 |= encoded_opts1;
	*opts2 |= encoded_opts2;
	return true;
}

static int rtlmq_xsk_metadata_opts(enum rtlmq_tx_desc_format format,
				   const struct xsk_tx_metadata_ops *ops,
				   struct xsk_buff_pool *pool, const struct xdp_desc *descs,
				   unsigned int num_descs,
				   struct rtlmq_xsk_metadata_request *request, u32 *opts1,
				   u32 *opts2)
{
	struct xsk_tx_metadata *meta;
	const struct ipv6hdr *ip6h;
	const struct ethhdr *eth;
	const struct iphdr *iph;
	unsigned int total = 0;
	__be16 protocol;
	unsigned int l3;
	unsigned int l4;
	const u8 *data;
	unsigned int i;
	u8 l4_proto;
	u64 flags;

	memset(request, 0, sizeof(*request));
	*opts1 = 0;
	*opts2 = 0;
	if (!(descs[0].options & XDP_TX_METADATA))
		return 0;

	meta = xsk_buff_get_metadata(pool, descs[0].addr, descs[0].options);
	if (!meta || !xsk_buff_valid_tx_metadata(pool, meta, &flags) ||
	    (flags & XDP_TXMD_FLAGS_LAUNCH_TIME))
		return 0;
	if ((flags & XDP_TXMD_FLAGS_CHECKSUM) &&
	    pool->tx_metadata_len <
		    offsetofend(struct xsk_tx_metadata, request.csum_offset))
		return 0;
	if ((flags & XDP_TXMD_FLAGS_TIMESTAMP) &&
	    pool->tx_metadata_len <
		    offsetofend(struct xsk_tx_metadata,
				completion.tx_timestamp))
		return 0;

	if (!ops)
		return 0;
	request->meta = meta;
	request->ops = ops;
	xsk_tx_metadata_request(pool, &request->meta, ops, request);
	if (!request->timestamp_requested)
		request->meta = NULL;
	if (!request->checksum.requested)
		return 0;

	for (i = 0; i < num_descs; i++)
		if (check_add_overflow(total, descs[i].len, &total))
			return -EINVAL;
	/* As on skb TX, hardware must not checksum Ethernet padding. */
	if (total < ETH_ZLEN)
		return -EOPNOTSUPP;
	if (descs[0].len < ETH_HLEN)
		return -EINVAL;
	data = xsk_buff_raw_get_data(pool, descs[0].addr);
	eth = (const struct ethhdr *)data;
	protocol = eth->h_proto;
	l3 = ETH_HLEN;
	for (i = 0; eth_type_vlan(protocol) && i < 2; i++) {
		const struct vlan_hdr *vh;

		if (descs[0].len < l3 + sizeof(*vh))
			return -EINVAL;
		vh = (const struct vlan_hdr *)(data + l3);
		protocol = vh->h_vlan_encapsulated_proto;
		l3 += sizeof(*vh);
	}

	switch (protocol) {
	case htons(ETH_P_IP):
		if (descs[0].len < l3 + sizeof(*iph))
			return -EINVAL;
		iph = (const struct iphdr *)(data + l3);
		if (iph->version != 4 || iph->ihl < 5)
			return -EINVAL;
		l4 = l3 + iph->ihl * 4U;
		l4_proto = iph->protocol;
		break;
	case htons(ETH_P_IPV6):
		if (descs[0].len < l3 + sizeof(*ip6h))
			return -EINVAL;
		ip6h = (const struct ipv6hdr *)(data + l3);
		if (ip6h->version != 6)
			return -EINVAL;
		l4 = l3 + sizeof(*ip6h);
		l4_proto = ip6h->nexthdr;
		break;
	default:
		return -EOPNOTSUPP;
	}

	if (request->checksum.start != l4 || l4 >= descs[0].len)
		return -EINVAL;
	switch (l4_proto) {
	case IPPROTO_TCP:
		if (request->checksum.offset != offsetof(struct tcphdr, check))
			return -EINVAL;
		break;
	case IPPROTO_UDP:
		if (request->checksum.offset != offsetof(struct udphdr, check))
			return -EINVAL;
		break;
	default:
		return -EOPNOTSUPP;
	}
	if (request->checksum.start + request->checksum.offset +
		    sizeof(__sum16) > total ||
	    request->checksum.start + request->checksum.offset +
		    sizeof(__sum16) > descs[0].len)
		return -EINVAL;
	if (!rtlmq_tx_checksum_desc_opts(format, protocol, l4_proto, l4,
					 i != 0, opts1, opts2))
		return -EINVAL;

	return 0;
}

/* Caller has checked a nonempty UMEM range. */
static bool rtlmq_xsk_tx_dma_span_valid(const struct xsk_buff_pool *pool,
					u64 addr, u64 end)
{
	u64 last_page = (end - 1) >> PAGE_SHIFT;
	u64 first_page = addr >> PAGE_SHIFT;

	if (first_page >= pool->dma_pages_cnt ||
	    last_page >= pool->dma_pages_cnt)
		return false;
	if (first_page != last_page &&
	    !(pool->dma_pages[first_page] & XSK_NEXT_PG_CONTIG_MASK))
		return false;

	return true;
}

/*
 * xsk_buff_raw_get_dma() deliberately assumes that the descriptor was
 * validated by the AF_XDP consumer.  This driver consumes a copied batch
 * after that core validation, so keep the raw helper behind an equivalent
 * local gate.  In particular, never let an untrusted address select an
 * entry in the pool's DMA-page array.
 */
static bool rtlmq_xsk_tx_desc_address_valid(const struct xsk_buff_pool *pool,
					    const struct xdp_desc *desc)
{
	u64 len_with_metadata;
	u64 chunk_offset;
	u64 metadata_len;
	u64 checked_end;
	u64 chunk_end;
	u64 raw_addr;
	u64 desc_len;
	u64 data_end;
	u64 addr;

	if (!pool || !desc || !pool->umem || !pool->dev ||
	    !pool->dma_pages || !pool->dma_pages_cnt || !pool->addrs_cnt ||
	    !pool->chunk_size || !desc->len ||
	    desc->len > RTLMQ_TX_DESC_LEN_MASK)
		return false;

	if (!pool->unaligned && !is_power_of_2(pool->chunk_size))
		return false;

	desc_len = desc->len;
	metadata_len = pool->tx_metadata_len;
	if (pool->unaligned) {
		u64 offset = desc->addr >> XSK_UNALIGNED_BUF_OFFSET_SHIFT;
		u64 base = desc->addr & XSK_UNALIGNED_BUF_ADDR_MASK;

		if (check_add_overflow(base, offset, &raw_addr))
			return false;
	} else {
		raw_addr = desc->addr;
	}

	/* Match the core's metadata-inclusive descriptor validation. */
	if (check_sub_overflow(raw_addr, metadata_len, &addr) ||
	    check_add_overflow(desc_len, metadata_len, &len_with_metadata) ||
	    addr >= pool->addrs_cnt)
		return false;

	if (pool->unaligned) {
		if (len_with_metadata > pool->chunk_size)
			return false;
	} else {
		chunk_offset = addr & (pool->chunk_size - 1);
		if (check_add_overflow(chunk_offset, len_with_metadata,
				       &chunk_end) ||
		    chunk_end > pool->chunk_size)
			return false;
	}

	/*
	 * The checked end is the raw data end after the metadata subtraction.
	 * Check it independently of the chunk test because the raw DMA helper
	 * indexes the UMEM's global page array, not a chunk-local array.
	 */
	if (check_add_overflow(addr, len_with_metadata, &checked_end) ||
	    checked_end > pool->addrs_cnt ||
	    check_add_overflow(raw_addr, desc_len, &data_end) ||
	    data_end > pool->addrs_cnt)
		return false;

	/*
	 * Mirror xp_desc_crosses_non_contig_pg() for the metadata-inclusive
	 * range, then repeat the page bound/continuity check for the actual
	 * bytes passed to the raw DMA lookup.
	 */
	if (!rtlmq_xsk_tx_dma_span_valid(pool, addr, checked_end))
		return false;

	if (!rtlmq_xsk_tx_dma_span_valid(pool, raw_addr, data_end))
		return false;

	return true;
}

enum rtlmq_tx_publish_result
rtlmq_publish_tx_xsk_batch(struct rtlmq_tx_ring *ring,
			   struct xsk_buff_pool *pool,
			   const struct xdp_desc *descs, unsigned int num_descs,
			   bool flush_doorbell)
{
	const struct rtlmq_runtime_config *active_cfg;
	const struct rtlmq_tx_ring_read_mostly *rm;
	dma_addr_t dma[RTLMQ_XSK_MAX_SEGS];
	unsigned int first_entry, entry;
	struct rtlmq_xsk_metadata_request metadata;
	enum rtlmq_tx_publish_result ret;
	unsigned int total_bytes = 0;
	u32 timestamp_generation = 0;
	unsigned int pending_before;
	bool hw_timestamp = false;
	u32 generation;
	unsigned int i;
	u32 desc_opts1;
	u32 opts2;

	if (unlikely(!ring || !pool))
		return RTLMQ_TX_DROPPED;
	if (unlikely(rtlmq_validate_xsk_tx_chain(descs, num_descs)))
		return RTLMQ_TX_DROPPED;
	if (unlikely(!pool->xdp_zc_max_segs ||
		     num_descs > min_t(unsigned int, RTLMQ_XSK_MAX_SEGS,
				       pool->xdp_zc_max_segs)))
		return RTLMQ_TX_DROPPED;
	for (i = 0; i < num_descs; i++) {
		if (!rtlmq_xsk_tx_desc_address_valid(pool, &descs[i]))
			return RTLMQ_TX_DROPPED;
	}
	rm = rtlmq_tx_ring_rm(ring);
	active_cfg = rtlmq_active_runtime_config(ring->priv);
	if (rtlmq_xsk_metadata_opts(rm->desc_format, active_cfg ?
				    active_cfg->xsk_tx_metadata_ops : NULL,
				    pool, descs, num_descs, &metadata,
				    &desc_opts1, &opts2))
		return RTLMQ_TX_DROPPED;

	if (unlikely(!rtlmq_tx_admit_descs(ring, rm, num_descs,
					   &pending_before)))
		return RTLMQ_TX_BUSY;

	for (i = 0; i < num_descs; i++) {
		dma[i] = xsk_buff_raw_get_dma(pool, descs[i].addr);
		if (!rtlmq_dma_range_fits_device(pool->dev, dma[i], descs[i].len))
			return RTLMQ_TX_DROPPED;
		if (check_add_overflow(total_bytes, descs[i].len,
				       &total_bytes))
			return RTLMQ_TX_DROPPED;
	}

	first_entry = rtlmq_tx_construction_tail(ring);
	entry = first_entry;
	/* Pairs with rtlmq_tx_ring_publish_generation() before slot reuse. */
	generation = smp_load_acquire(&ring->generation);
	if (metadata.timestamp_requested && metadata.meta)
		hw_timestamp = rtlmq_ptp_xsk_tx_prepare(ring->priv, pool, descs, num_descs,
							metadata.meta, metadata.ops,
							&timestamp_generation);
	for (i = 0; i < num_descs; i++) {
		struct rtlmq_tx_info *info = &rm->infos[entry];
		u32 opts1 = RTLMQ_DESC_OWN;

		xsk_buff_raw_dma_sync_for_device(pool, dma[i], descs[i].len);
		rtlmq_init_tx_info(info, descs[i].len,
				   0, 0,
				   generation, 0);

		if (i + 1 == num_descs)
			opts1 |= RTLMQ_DESC_LAST_FRAG;
		if (i)
			rtlmq_write_tx_desc_staged(ring, entry, dma[i],
						   descs[i].len, opts1, opts2);
		if (i + 1 < num_descs)
			entry = rtlmq_tx_desc_advance(ring, entry);
	}

	rm->infos[entry].flags |= RTLMQ_TX_INFOF_EOP;
	rm->infos[entry].completion_packets = 1;
	rm->completions[entry].xsk_pool = pool;
	rm->completions[entry].completion_bytes = total_bytes;
	rm->completions[entry].timestamp_generation =
		timestamp_generation;
	rm->completions[entry].xsk_descs = num_descs;
	rm->completions[entry].type = RTLMQ_TX_BUFFER_XSK;
	rm->completions[entry].flags =
		hw_timestamp ? RTLMQ_TX_COMPLETIONF_HW_TSTAMP : 0;

	rtlmq_write_tx_desc_staged(ring, first_entry, dma[0], descs[0].len,
				   desc_opts1 | RTLMQ_DESC_FIRST_FRAG |
				   (num_descs == 1 ? RTLMQ_DESC_LAST_FRAG : 0),
				   opts2);
	ret = rtlmq_publish_tx_desc_tail_fast(ring, rm,
					      num_descs, total_bytes,
					      !flush_doorbell, flush_doorbell,
					      false, pending_before);
	if (ret == RTLMQ_TX_PUBLISHED)
		rtlmq_sync_tx_ring_irq_arming(ring);

	return ret;
}

static u32 rtlmq_tx_vlan_tag(const struct sk_buff *skb, bool enabled)
{
	if (!enabled || !skb_vlan_tag_present(skb) ||
	    skb->vlan_proto != htons(ETH_P_8021Q))
		return 0;

	return rtlmq_tx_desc_vlan_tag_encode(skb_vlan_tag_get(skb));
}

static bool rtlmq_tx_encode_gso(enum rtlmq_tx_desc_format fmt, bool ipv4,
				u32 transport_offset, u32 mss,
				u32 *opts1, u32 *opts2)
{
	if (!opts1 || !opts2 || !mss ||
	    mss > RTLMQ_TX_DESC_MSS_MASK ||
	    transport_offset > RTLMQ_TX_DESC_GSO_TCPHO_MAX)
		return false;

	switch (fmt) {
	case RTLMQ_TX_DESC_FORMAT_LEGACY:
		*opts1 |= ipv4 ? RTLMQ_TX_DESC_GSO_V4 :
				 RTLMQ_TX_DESC_GSO_V6;
		break;
	case RTLMQ_TX_DESC_FORMAT_V3:
		*opts2 |= RTLMQ_TX_DESC_V3_GSO_ENABLE |
			  RTLMQ_TX_DESC_V3_GSO_TCP;
		if (ipv4)
			*opts2 |= RTLMQ_TX_DESC_V3_GSO_IPV4;
		break;
	default:
		return false;
	}

	*opts1 |= transport_offset << RTLMQ_TX_DESC_GSO_TCPHO_SHIFT;
	*opts2 |= mss << RTLMQ_TX_DESC_MSS_SHIFT;
	return true;
}

static enum rtlmq_tx_publish_result
rtlmq_prepare_tx_tso_offloads(struct rtlmq_tx_ring *ring,
			      struct sk_buff *skb,
			      netdev_features_t features,
			      u32 *desc_opts1, u32 *desc_opts2,
			      u16 *completion_packets)
{
	u32 trans_off;
	u32 gso_size;
	u32 gso_type;

	/* Preserve the core VLAN parser's malformed-metadata warning. */
	vlan_get_protocol(skb);

	*desc_opts1 = 0;
	*desc_opts2 = rtlmq_tx_vlan_tag(skb, features & NETIF_F_HW_VLAN_CTAG_TX);
	gso_size = skb_shinfo(skb)->gso_size;
	trans_off = skb_transport_offset(skb);
	gso_type = skb_shinfo(skb)->gso_type;

	/*
	 * Both formats carry TCP header offset in opts1 and MSS in
	 * opts2[28:18]. The format-selected giant-send selectors must
	 * not be mixed with non-GSO checksum selectors.
	 */
	if (!(gso_type & (SKB_GSO_TCPV4 | SKB_GSO_TCPV6)))
		return RTLMQ_TX_DROPPED;
	if (!rtlmq_tx_encode_gso(ring->desc_format,
				 gso_type & SKB_GSO_TCPV4,
				 trans_off, gso_size,
				 desc_opts1, desc_opts2))
		return RTLMQ_TX_DROPPED;

	*completion_packets = max_t(u16, skb_shinfo(skb)->gso_segs, 1);
	return RTLMQ_TX_PUBLISHED;
}

/*
 * RTL8125 giant-send v6 consumes the zero-length IPv6 pseudo-header seed,
 * not the full CHECKSUM_PARTIAL seed presented by the network stack. This
 * mutates protocol headers and may unshare the skb head, so callers must run
 * it only after descriptor admission has ruled out NETDEV_TX_BUSY.
 */
static bool rtlmq_prepare_tso6_checksum(struct sk_buff *skb)
{
	if (!(skb_shinfo(skb)->gso_type & SKB_GSO_TCPV6))
		return true;

	if (unlikely(skb_cow_head(skb, 0)))
		return false;

	tcp_v6_gso_csum_prep(skb);
	return true;
}

static bool rtlmq_tx_skb_single_fast_candidate(const struct sk_buff *skb)
{
	const struct skb_shared_info *shinfo = skb_shinfo(skb);

	return likely(!shinfo->gso_size && !shinfo->nr_frags &&
		      !shinfo->frag_list && skb->len >= ETH_ZLEN &&
		      skb_headlen(skb) == skb->len);
}

static bool rtlmq_tx_skb_normal_fast_candidate(const struct sk_buff *skb)
{
	const struct skb_shared_info *shinfo = skb_shinfo(skb);

	return likely(!shinfo->gso_size && !shinfo->frag_list &&
		      skb->len >= ETH_ZLEN && skb_headlen(skb));
}

static enum rtlmq_tx_publish_result
rtlmq_prepare_tx_nongso_skb_offloads_fast(enum rtlmq_tx_desc_format format, struct sk_buff *skb,
					  netdev_features_t features, u32 *opts1, u32 *opts2)
{
	__be16 protocol;

	*opts1 = 0;
	*opts2 = rtlmq_tx_vlan_tag(skb, features & NETIF_F_HW_VLAN_CTAG_TX);
	if (skb->ip_summed != CHECKSUM_PARTIAL)
		return RTLMQ_TX_PUBLISHED;

	protocol = vlan_get_protocol(skb);
	switch (protocol) {
	case htons(ETH_P_IP):
		if (!rtlmq_tx_checksum_desc_opts(format, protocol, ip_hdr(skb)->protocol,
						 skb_transport_offset(skb),
						 eth_type_vlan(eth_hdr(skb)->h_proto), opts1,
						 opts2))
			return RTLMQ_TX_DROPPED;
		break;
	case htons(ETH_P_IPV6):
		if (!rtlmq_tx_checksum_desc_opts(format, protocol, ipv6_hdr(skb)->nexthdr,
						 skb_transport_offset(skb),
						 eth_type_vlan(eth_hdr(skb)->h_proto), opts1,
						 opts2))
			return RTLMQ_TX_DROPPED;
		break;
	default:
		return RTLMQ_TX_DROPPED;
	}
	return RTLMQ_TX_PUBLISHED;
}

static noinline enum rtlmq_tx_publish_result
rtlmq_publish_tx_skb_single_fast(struct sk_buff *skb,
				 struct rtlmq_tx_ring *ring,
				 const struct rtlmq_tx_ring_read_mostly *rm)
{
	struct rtlmq_tx_completion *completion;
	struct rtlmq_private *tp = ring->priv;
	enum rtlmq_tx_publish_result ret;
	struct rtlmq_tx_info *info;
	dma_addr_t dma_addr;
	unsigned int entry;
	u32 completion_bytes = skb->len;
	u32 pending_before;
	u32 len = skb->len;
	bool small_copy;
	u32 generation;
	bool xmit_more;
	u32 opts1;
	u32 opts2;

	ret = rtlmq_prepare_tx_nongso_skb_offloads_fast(rm->desc_format, skb,
							READ_ONCE(ring->ndev->features), &opts1,
							&opts2);
	if (unlikely(ret != RTLMQ_TX_PUBLISHED))
		return rtlmq_drop_tx_skb(ring, skb);

	xmit_more = netdev_xmit_more();

	if (unlikely(!rtlmq_tx_admit_descs(ring, rm, 1,
					   &pending_before)))
		return RTLMQ_TX_BUSY;

	if (unlikely(!len || len > RTLMQ_TX_DESC_LEN_MASK))
		return rtlmq_drop_tx_skb(ring, skb);

	entry = rtlmq_tx_construction_tail(ring);
	small_copy = len <= RTLMQ_TX_DESC_PRESSURE_MAX_BYTES ||
		     (len >= RTLMQ_TX_LARGE_COPY_MIN_BYTES &&
		      len <= RTLMQ_TX_SMALL_COPY_BYTES);
	if (likely(small_copy)) {
		u32 offset = entry * RTLMQ_TX_SMALL_COPY_BYTES;
		u8 *slot;

		if (unlikely(!rm->small_copy))
			return rtlmq_drop_tx_skb(ring, skb);
		slot = rm->small_copy + offset;

		memcpy(slot, skb->data, len);
		dma_addr = rm->small_copy_dma + offset;
	} else {
		dma_addr = rtlmq_tx_dma_map_single(tp, skb->data, len);
		if (rtlmq_tx_dma_mapping_error(tp, dma_addr))
			return rtlmq_drop_tx_skb(ring, skb);
	}

	/* Pairs with rtlmq_tx_ring_publish_generation() before slot reuse. */
	generation = smp_load_acquire(&ring->generation);
	info = &rm->infos[entry];
	completion = &rm->completions[entry];

	rtlmq_init_tx_info(info, len,
			   small_copy ? 0 : dma_addr, small_copy ? 0 : len,
			   generation,
			   RTLMQ_TX_INFOF_EOP | (small_copy ? RTLMQ_TX_INFOF_SMALL_COPY :
							     RTLMQ_TX_INFOF_MAPPED));
	info->completion_packets = 1;
	rtlmq_set_tx_skb_completion_fast(completion, skb, completion_bytes);

	rtlmq_ptp_tx_prepare(tp, skb, completion);

	rtlmq_write_tx_desc_staged(ring, entry, dma_addr, len,
				   opts1 | RTLMQ_DESC_FIRST_FRAG |
				   RTLMQ_DESC_LAST_FRAG, opts2);
	return rtlmq_publish_tx_desc_tail_fast(ring, rm,
					       1,
					       completion_bytes, xmit_more, false,
					       true, pending_before);
}

static inline void rtlmq_tx_write_poll_fast(struct rtlmq_tx_ring *ring)
{
	const struct rtlmq_tx_ring_read_mostly *rm = rtlmq_tx_ring_rm(ring);
	u16 queue_command = rm->doorbell_value;

	/*
	 * TX_POLL is a one-shot, one-hot MMIO queue command. Its write ordering
	 * is separate from descriptor DMA ownership barriers: do not rely on
	 * dma_wmb()/dma_rmb() to order the doorbell. No caller requires a
	 * serializing PCIe readback.
	 */
	wmb();
	writew(queue_command, rm->doorbell);
}

static bool
rtlmq_tx_kick_fast(struct rtlmq_tx_ring *ring)
{
	if (unlikely(!rtlmq_mmio_ordinary_authorized(ring->priv)))
		return false;

	rtlmq_tx_write_poll_fast(ring);
	ring->doorbells++;
	return true;
}

bool rtlmq_serial_tx_notify(struct rtlmq_tx_ring *ring)
{
	const struct rtlmq_serial_tx_control *c = &ring->serial_control;

	lockdep_assert_held(&ring->txq->_xmit_lock);
	/* DRAIN may flush/re-kick old publication; revoked queues cannot kick. */
	if (c->permit == RTLMQ_SERIAL_NONE || c->paused ||
	    c->generation != READ_ONCE(ring->generation))
		return false;
	if (!rtlmq_tx_kick_fast(ring))
		return false;
	ring->tx_doorbell_pending = false;
	return true;
}

bool rtlmq_tx_desc_status_rekick_pending(struct rtlmq_tx_ring *ring,
					 u32 pending_after)
{
	if (!ring)
		return false;
	{
		bool kicked = false;

		__netif_tx_lock_bh(ring->txq);
		if (pending_after && rtlmq_tx_published_count(ring))
			kicked = rtlmq_serial_tx_notify(ring);
		__netif_tx_unlock_bh(ring->txq);
		return kicked;
	}
}

static noinline enum rtlmq_tx_publish_result
rtlmq_publish_tx_tso_desc_chain_validated(struct sk_buff *skb, struct rtlmq_tx_ring *ring,
					  const struct rtlmq_tx_ring_read_mostly *rm,
					  u32 desc_opts1, u32 desc_opts2,
					  u16 completion_packets, u32 completion_bytes,
					  unsigned int needed, bool xmit_more, u32 pending_before)
{
	const struct skb_shared_info *shinfo = skb_shinfo(skb);
	unsigned int nr_frags = shinfo->nr_frags;
	struct rtlmq_tx_dma_sg_cursor cursor;
	struct rtlmq_tx_dma_txn *txn = NULL;
	struct rtlmq_tso_emit_state emit;
	unsigned int first_entry;
	unsigned int owner_entry;
	unsigned int i;
	u32 generation;

	/*
	 * RTL8125B TSO invariants:
	 * - every DMA chunk stays <= RTLMQ_TX_TSO_MAX_SEG_LEN
	 * - the first descriptor must carry header + payload, never header-only
	 * - keep the first descriptor OWN bit clear until the whole chain is
	 *   built so a mapping failure can unwind without racing hardware
	 * - charge BQL once per skb, then complete it once at the skb EOP
	 * - publication releases the first descriptor OWN bit last; only native
	 *   fixtures may mark later descriptors OWN while building the chain
	 */
	first_entry = rtlmq_tx_construction_tail(ring);
	/* Pairs with rtlmq_tx_ring_publish_generation() before slot reuse. */
	generation = smp_load_acquire(&ring->generation);
	desc_opts1 &= ~RTLMQ_DESC_OWN;

	emit = (struct rtlmq_tso_emit_state) {
		.rm = rm,
		.ring = ring,
		.skb = skb,
		.desc_opts1 = desc_opts1,
		.desc_opts2 = desc_opts2,
		.generation = generation,
		.info_flags = RTLMQ_TX_INFOF_MAPPED |
			      RTLMQ_TX_INFOF_TSO |
			      RTLMQ_TX_INFOF_DMA_SG,
		.completion_packets = completion_packets,
		.completion_bytes = completion_bytes,
		.needed = needed,
		.entry = first_entry,
	};

	if (unlikely(!skb_headlen(skb)))
		goto drop;

	owner_entry = rtlmq_tx_desc_nth(ring, first_entry, needed - 1);
	txn = &rm->dma_txns[owner_entry];
	cursor = (struct rtlmq_tx_dma_sg_cursor) {
		.sg = txn->sg,
	};
	cursor.nents = rtlmq_tx_map_tso_sg(ring, txn, skb);
	if (!cursor.nents)
		goto err_unwind;

	{
		u32 linear_len = skb_headlen(skb);
		dma_addr_t linear_dma;

		if (unlikely(!rtlmq_tx_dma_sg_cursor_take(&cursor, linear_len, &linear_dma)) ||
		    unlikely(!rtlmq_emit_tso_split_range(&emit, linear_dma, linear_len)))
			goto err_unwind;
	}

	for (i = 0; i < nr_frags; i++) {
		const skb_frag_t *frag = &shinfo->frags[i];
		u32 frag_len = skb_frag_size(frag);
		dma_addr_t frag_dma;

		if (unlikely(!rtlmq_tx_dma_sg_cursor_take(&cursor, frag_len, &frag_dma)) ||
		    unlikely(!rtlmq_emit_tso_split_range(&emit, frag_dma, frag_len))) {
			goto err_unwind;
		}
	}

	if (unlikely(emit.mapped != needed))
		goto err_unwind;

	rtlmq_ptp_tx_prepare(ring->priv, skb, &rm->completions[owner_entry]);

	return rtlmq_publish_tx_desc_tail_fast(ring, rm,
					       needed,
					       completion_bytes, xmit_more,
					       false, false,
					       pending_before);

err_unwind:
	if (txn && txn->mapped)
		rtlmq_tx_unmap_sg_txn_owned(ring, owner_entry);
	return rtlmq_unwind_and_drop_tx_skb(ring, skb, first_entry,
					    emit.mapped);

drop:
	return rtlmq_drop_tx_skb(ring, skb);
}

static enum rtlmq_tx_publish_result
rtlmq_publish_tx_skb_frags_validated(struct sk_buff *skb,
				     struct rtlmq_tx_ring *ring)
{
	const struct rtlmq_tx_ring_read_mostly *rm = rtlmq_tx_ring_rm(ring);
	const struct skb_shared_info *shinfo = skb_shinfo(skb);
	struct rtlmq_private *tp = ring->priv;
	unsigned int first_entry;
	unsigned int mapped = 0;
	unsigned int needed;
	unsigned int entry;
	unsigned int i;
	struct rtlmq_tx_completion *completion;
	enum rtlmq_tx_publish_result ret;
	u32 completion_bytes = skb->len;
	struct rtlmq_tx_info *info;
	dma_addr_t first_dma_addr;
	dma_addr_t dma_addr;
	u32 pending_before;
	u32 first_opts1;
	u32 generation;
	u32 desc_opts1;
	bool xmit_more;
	u32 first_len;
	u32 opts2;
	u32 len;

	if (check_add_overflow((unsigned int)shinfo->nr_frags, 1U, &needed))
		goto drop;

	ret = rtlmq_prepare_tx_nongso_skb_offloads_fast(rm->desc_format, skb,
							READ_ONCE(ring->ndev->features),
							&desc_opts1, &opts2);
	if (unlikely(ret != RTLMQ_TX_PUBLISHED))
		goto drop;

	xmit_more = netdev_xmit_more();
	if (unlikely(!rtlmq_tx_admit_descs(ring, rm, needed,
					   &pending_before)))
		return RTLMQ_TX_BUSY;

	entry = rtlmq_tx_construction_tail(ring);
	first_entry = entry;
	/* Pairs with rtlmq_tx_ring_publish_generation() before slot reuse. */
	generation = smp_load_acquire(&ring->generation);
	len = skb_headlen(skb);
	if (unlikely(!len || len > RTLMQ_TX_DESC_LEN_MASK))
		goto drop;

	dma_addr = rtlmq_tx_dma_map_single(tp, skb->data, len);
	if (rtlmq_tx_dma_mapping_error(tp, dma_addr))
		goto drop;
	first_dma_addr = dma_addr;
	first_len = len;

	info = &rm->infos[first_entry];
	rtlmq_init_tx_info(info, len,
			   dma_addr, len,
			   generation, RTLMQ_TX_INFOF_MAPPED);
	mapped = 1;

	for (i = 0; i < shinfo->nr_frags; i++) {
		const skb_frag_t *frag = &shinfo->frags[i];
		u32 frag_opts1 = RTLMQ_DESC_OWN;
		u32 frag_len = skb_frag_size(frag);

		if (unlikely(!frag_len || frag_len > RTLMQ_TX_DESC_LEN_MASK))
			goto err_unwind;

		entry = rtlmq_tx_desc_advance(ring, entry);
		dma_addr = rtlmq_tx_dma_map_frag(tp, frag, 0, frag_len);
		if (rtlmq_tx_dma_mapping_error(tp, dma_addr))
			goto err_unwind;

		info = &rm->infos[entry];
		rtlmq_init_tx_info(info, frag_len,
				   dma_addr, frag_len,
				   generation,
				   RTLMQ_TX_INFOF_MAPPED |
				   RTLMQ_TX_INFOF_PAGE);
		if (i + 1 == shinfo->nr_frags)
			frag_opts1 |= RTLMQ_DESC_LAST_FRAG;

		rtlmq_write_tx_desc_staged(ring, entry, dma_addr, frag_len,
					   frag_opts1, opts2);
		mapped++;
	}

	completion = &rm->completions[entry];
	info = &rm->infos[entry];
	info->flags |= RTLMQ_TX_INFOF_EOP;
	info->completion_packets = 1;
	rtlmq_set_tx_skb_completion_fast(completion, skb, completion_bytes);

	rtlmq_ptp_tx_prepare(tp, skb, completion);

	first_opts1 = desc_opts1 | RTLMQ_DESC_FIRST_FRAG;
	if (unlikely(needed == 1))
		first_opts1 |= RTLMQ_DESC_LAST_FRAG;
	rtlmq_write_tx_desc_staged(ring, first_entry, first_dma_addr, first_len,
				   first_opts1, opts2);
	return rtlmq_publish_tx_desc_tail_fast(ring, rm,
					       needed, completion_bytes,
					       xmit_more, false, false,
					       pending_before);

err_unwind:
	return rtlmq_unwind_and_drop_tx_skb(ring, skb, first_entry, mapped);

drop:
	return rtlmq_drop_tx_skb(ring, skb);
}

static noinline enum rtlmq_tx_publish_result
rtlmq_publish_tx_skb_tso_validated(struct sk_buff *skb,
				   struct rtlmq_tx_ring *ring)
{
	const struct rtlmq_tx_ring_read_mostly *rm = rtlmq_tx_ring_rm(ring);
	enum rtlmq_tx_publish_result ret;
	u32 completion_bytes = skb->len;
	unsigned int admission_needed;
	u16 completion_packets = 1;
	unsigned int needed;
	u32 pending_before;
	u32 desc_opts1;
	u32 desc_opts2;
	bool xmit_more;

	if (unlikely(skb_has_frag_list(skb)))
		goto drop;

	ret = rtlmq_prepare_tx_tso_offloads(ring, skb, READ_ONCE(ring->ndev->features), &desc_opts1,
					    &desc_opts2, &completion_packets);
	if (ret != RTLMQ_TX_PUBLISHED)
		goto drop;

	/*
	 * Admission must precede pskb_may_pull(): NETDEV_TX_BUSY promises that
	 * the stack still owns an unmodified skb. Pulling at most one small
	 * payload window can add one head descriptor and can expose one final
	 * byte-patch descriptor, so reserve both before changing skb layout.
	 */
	needed = rtlmq_tx_tso_descs_needed(skb);
	if (needed > UINT_MAX - RTLMQ_TSO_PULL_ADMISSION_SLACK)
		goto drop;
	admission_needed = needed + RTLMQ_TSO_PULL_ADMISSION_SLACK;
	if (unlikely(!rtlmq_tx_admit_descs(ring, rm, admission_needed,
					   &pending_before)))
		return RTLMQ_TX_BUSY;

	if (unlikely(!rtlmq_prepare_tso6_checksum(skb)))
		goto drop;

	/*
	 * Finalize the TSO head layout before counting descriptors. The count
	 * must match the skb we actually map and split.
	 */
	if (unlikely(!rtlmq_tso_pull_first_payload(skb)))
		goto drop;

	xmit_more = netdev_xmit_more();
	needed = rtlmq_tx_tso_descs_needed(skb);
	if (needed == UINT_MAX)
		goto drop;

	if (WARN_ON_ONCE(needed > admission_needed))
		goto drop;

	return rtlmq_publish_tx_tso_desc_chain_validated(skb, ring, rm, desc_opts1, desc_opts2,
							 completion_packets, completion_bytes,
							 needed, xmit_more, pending_before);

drop:
	return rtlmq_drop_tx_skb(ring, skb);
}

static noinline enum rtlmq_tx_publish_result
rtlmq_publish_tx_skb_fallback_validated(struct sk_buff *skb,
					struct rtlmq_tx_ring *ring)
{
	const struct rtlmq_tx_ring_read_mostly *rm = rtlmq_tx_ring_rm(ring);
	unsigned int admission_needed;
	u32 pending_before;

	if (unlikely(skb_has_frag_list(skb)))
		goto drop;

	if (unlikely(skb->len < ETH_ZLEN)) {
		/*
		 * Padding mutates the skb, so reserve the descriptor shape that
		 * the nested single/frags/TSO publisher will need first. The TX
		 * queue lock excludes another producer, and reclaim can only
		 * increase availability between this check and the nested one.
		 */
		if (skb_is_gso(skb)) {
			unsigned int needed = rtlmq_tx_tso_descs_needed(skb);

			if (needed > UINT_MAX - RTLMQ_TSO_PULL_ADMISSION_SLACK)
				goto drop;
			admission_needed = needed +
				RTLMQ_TSO_PULL_ADMISSION_SLACK;
		} else if (check_add_overflow((unsigned int)skb_shinfo(skb)->nr_frags, 1U,
					      &admission_needed)) {
			goto drop;
		}

		if (unlikely(!rtlmq_tx_admit_descs(ring, rm, admission_needed,
						   &pending_before)))
			return RTLMQ_TX_BUSY;

		if (__skb_put_padto(skb, ETH_ZLEN, false))
			goto drop;
	}

	if (likely(rtlmq_tx_skb_single_fast_candidate(skb)))
		return rtlmq_publish_tx_skb_single_fast(skb, ring, rm);

	if (likely(rtlmq_tx_skb_normal_fast_candidate(skb)))
		return rtlmq_publish_tx_skb_frags_validated(skb, ring);

	if (unlikely(skb_is_gso(skb)))
		return rtlmq_publish_tx_skb_tso_validated(skb, ring);

drop:
	return rtlmq_drop_tx_skb(ring, skb);
}

static enum rtlmq_tx_publish_result
rtlmq_publish_tx_skb_validated(struct sk_buff *skb, struct rtlmq_tx_ring *ring)
{
	if (likely(rtlmq_tx_skb_single_fast_candidate(skb))) {
		const struct rtlmq_tx_ring_read_mostly *rm =
			rtlmq_tx_ring_rm(ring);

		return rtlmq_publish_tx_skb_single_fast(skb, ring, rm);
	}

	if (likely(rtlmq_tx_skb_normal_fast_candidate(skb)))
		return rtlmq_publish_tx_skb_frags_validated(skb, ring);

	if (unlikely(skb_is_gso(skb)))
		return rtlmq_publish_tx_skb_tso_validated(skb, ring);

	return rtlmq_publish_tx_skb_fallback_validated(skb, ring);
}

static struct rtlmq_tx_ring *rtlmq_xmit_queue_ring(struct rtlmq_private *tp,
						   const struct sk_buff *skb)
{
	u16 queue_index = skb_get_queue_mapping(skb);
	u16 published_txq = rtlmq_published_num_tx_queues(tp);

	if (unlikely(!published_txq))
		return NULL;

	/*
	 * Fall back to queue zero if the skb mapping exceeds the current queue
	 * count. The zero-count case was rejected above.
	 */
	if (unlikely(queue_index >= published_txq))
		queue_index = 0;

	return READ_ONCE(tp->txq2ring[queue_index]);
}

static netdev_tx_t rtlmq_xmit_skb(struct sk_buff *skb, struct rtlmq_private *tp)
{
	struct rtlmq_tx_ring *ring;
	netdev_tx_t ret;

	/*
	 * OPEN publication installs every real TX queue mapping before the
	 * stack can call ndo_start_xmit(), and close quiesces the stack before
	 * withdrawing it. The queue lock keeps this mapping stable for the
	 * duration of the call.
	 */
	ring = rtlmq_xmit_queue_ring(tp, skb);
	if (unlikely(!ring)) {
		dev_kfree_skb_any(skb);
		return NETDEV_TX_OK;
	}

	ret = rtlmq_xmit_frame_validated(skb, ring);

	return ret;
}

bool rtlmq_tx_flush_doorbell_if_pending(struct rtlmq_tx_ring *ring)
{
	if (!ring)
		return false;
	{
		return ring->tx_doorbell_pending &&
		       rtlmq_serial_tx_notify(ring);
	}
}

bool rtlmq_tx_flush_pending_ring(struct rtlmq_tx_ring *ring)
{
	struct netdev_queue *txq;
	bool flushed;

	if (!ring)
		return false;

	txq = READ_ONCE(rtlmq_tx_ring_rm(ring)->txq);
	if (!txq)
		return false;

	/*
	 * Producers publish the pending latch under this queue lock. Keep the
	 * check-and-kick under the same lock so a concurrent xmit_more producer
	 * cannot set the latch just after an unlocked check returns false.
	 */
	__netif_tx_lock_bh(txq);
	flushed = rtlmq_tx_flush_doorbell_if_pending(ring);
	__netif_tx_unlock_bh(txq);

	return flushed;
}

void rtlmq_flush_pending_tx_doorbells(struct rtlmq_private *tp)
{
	u16 num_tx_queues;
	unsigned int i;

	if (!tp || !tp->tx_rings ||
	    !rtlmq_mmio_ordinary_authorized(tp))
		return;

	/*
	 * Flush deferred xmit_more doorbells before clearing queue counts and
	 * ring mappings or disabling MMIO access.
	 */
	num_tx_queues = min_t(u16, rtlmq_published_num_tx_queues(tp),
			      rtlmq_owned_tx_ring_slots(tp));
	for (i = 0; i < num_tx_queues; i++)
		rtlmq_tx_flush_pending_ring(&tp->tx_rings[i]);
}

void rtlmq_flush_vector_pending_tx_doorbells(struct rtlmq_vector *vector)
{
	unsigned int num_tx_rings;
	struct rtlmq_private *tp;
	unsigned long ring_mask;
	u16 ring_slots;
	unsigned int i;

	if (!vector)
		return;

	num_tx_rings = min_t(unsigned int, READ_ONCE(vector->num_tx_rings),
			     ARRAY_SIZE(vector->tx_ring_indices));
	ring_mask = READ_ONCE(vector->xdp_tx_pending_mask);
	/*
	 * RX-only NAPI may publish XDP_TX to another vector's TX ring. Its
	 * NAPI-local mask must still be drained even without mapped TX rings.
	 */
	if (!num_tx_rings && !ring_mask)
		return;

	tp = vector->priv;
	if (!tp || !tp->tx_rings ||
	    !rtlmq_mmio_ordinary_authorized(tp))
		return;

	ring_slots = rtlmq_owned_tx_ring_slots(tp);
	WRITE_ONCE(vector->xdp_tx_pending_mask, 0);
	for (i = 0; i < num_tx_rings; i++) {
		u16 queue_index = READ_ONCE(vector->tx_ring_indices[i]);

		if (queue_index < ring_slots)
			ring_mask |= BIT(queue_index);
	}

	for (i = 0; i < ring_slots; i++)
		if (ring_mask & BIT(i))
			rtlmq_tx_flush_pending_ring(&tp->tx_rings[i]);
}

static netdev_tx_t rtlmq_xmit_frame_validated(struct sk_buff *skb,
					      struct rtlmq_tx_ring *ring)
{
	const struct rtlmq_tx_ring_read_mostly *rm = rtlmq_tx_ring_rm(ring);
	enum rtlmq_tx_publish_result result;
	bool tcp_gso = skb_is_gso(skb) &&
		(skb_shinfo(skb)->gso_type &
		 (SKB_GSO_TCPV4 | SKB_GSO_TCPV6));
	u32 pending_before;

	/*
	 * Core stop/wake permits a false wake while the descriptor ring is
	 * still full. Recheck the minimum one-descriptor admission at the
	 * ndo_start_xmit boundary before checksum parsing, padding, pull-up,
	 * mapping, or any other skb mutation. Shape-specific publishers
	 * retain their larger reservation checks.
	 */
	if (unlikely(!rtlmq_tx_admit_descs(ring, rm, 1,
					   &pending_before)))
		result = RTLMQ_TX_BUSY;
	else
		result = rtlmq_publish_tx_skb_validated(skb, ring);
	if (result == RTLMQ_TX_BUSY)
		return NETDEV_TX_BUSY;
	/* The publisher may already have released skb; use the saved shape. */
	if (result == RTLMQ_TX_PUBLISHED && tcp_gso) {
		u64_stats_update_begin(&ring->syncp);
		ring->stats.xmit_tso_pkts++;
		u64_stats_update_end(&ring->syncp);
	}

	return NETDEV_TX_OK;
}

netdev_tx_t rtlmq_start_xmit(struct sk_buff *skb, struct net_device *ndev)
{
	struct rtlmq_private *tp = netdev_priv(ndev);

	return rtlmq_xmit_skb(skb, tp);
}

static u16 rtlmq_xdp_xmit_queue_index(u16 published_txq,
				      const struct xdp_frame *frame)
{
	if (!frame || !frame->data || !is_power_of_2(published_txq) ||
	    published_txq > RTLMQ_IMPL_MAX_TX_QUEUES)
		return U16_MAX;
	if (published_txq == 1)
		return 0;

	return hash_ptr(frame->data, ilog2(published_txq));
}

int rtlmq_xdp_xmit(struct net_device *ndev, int n, struct xdp_frame **frames,
		   u32 flags)
{
	struct rtlmq_private *tp = netdev_priv(ndev);
	struct rtlmq_tx_ring *ring;
	struct netdev_queue *nq;
	u16 published_txq;
	u16 queue_index;
	int nxmit = 0;
	int i;

	if (unlikely(n < 0 || (n && !frames) ||
		     flags & ~XDP_XMIT_FLAGS_MASK))
		return -EINVAL;

	if (unlikely(!n))
		return 0;

	/*
	 * XDP redirect may race with teardown. Keep the ring alive through the
	 * TX queue critical section; teardown calls synchronize_rcu() before
	 * freeing ring storage.
	 */
	rcu_read_lock();

	if (!(ndev->xdp_features & NETDEV_XDP_ACT_NDO_XMIT))
		goto out_not_supported;

	if (unlikely(!netif_running(ndev) || !netif_device_present(ndev) ||
		     READ_ONCE(tp->state) != RTLMQ_STATE_OPEN))
		goto out_net_down;

	published_txq = rtlmq_published_num_tx_queues(tp);
	if (unlikely(!published_txq))
		goto out_no_queue;

	queue_index = rtlmq_xdp_xmit_queue_index(published_txq, frames[0]);
	if (unlikely(queue_index >= published_txq))
		goto out_no_queue;
	ring = READ_ONCE(tp->txq2ring[queue_index]);
	if (unlikely(!rtlmq_tx_ring_boundary_ready(ring)))
		goto out_no_queue;

	for (i = 0; i < n; i++) {
		if (unlikely(!rtlmq_xdp_external_frame_valid(frames[i])))
			goto out_invalid;
	}

	nq = rtlmq_tx_ring_rm(ring)->ndev_queue;
	/*
	 * NDO_XDP_XMIT can be reached through different net-core execution
	 * contexts. The BH variant prevents migration and sets xmit_lock_owner.
	 */
	__netif_tx_lock_bh(nq);
	txq_trans_cond_update(nq);
	for (i = 0; i < n; i++) {
		enum rtlmq_tx_publish_result ret;

		ret = rtlmq_publish_tx_xdp_frame_validated(ring, frames[i], RTLMQ_TX_BUFFER_XDP_NDO,
							   true, false);
		if (ret == RTLMQ_TX_PUBLISHED) {
			nxmit++;
			continue;
		}

		break;
	}
	if (nxmit)
		rtlmq_tx_flush_doorbell_if_pending_hot(ring);

	__netif_tx_unlock_bh(nq);

	rcu_read_unlock();
	return nxmit;

out_not_supported:
	rcu_read_unlock();
	return -EOPNOTSUPP;
out_net_down:
	rcu_read_unlock();
	return -ENETDOWN;
out_no_queue:
	rcu_read_unlock();
	return -ENXIO;
out_invalid:
	rcu_read_unlock();
	return -EINVAL;
}
