// SPDX-License-Identifier: GPL-2.0-only

#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/string.h>
#include <net/netdev_queues.h>
#include <net/xdp_sock_drv.h>

#include "rtlmq_hw.h"
#include "rtlmq_irq.h"
#include "rtlmq_lifecycle.h"
#include "rtlmq_regs.h"
#include "rtlmq.h"
#include "rtlmq_runtime_accessors.h"
#include "rtlmq_tx.h"
#include "rtlmq_tx_internal.h"
#include "rtlmq_tx_private.h"
#include "rtlmq_trace.h"
#include "rtlmq_xdp.h"
#include "rtlmq_ptp.h"

struct rtlmq_tx_poll_batch {
	unsigned int pkts_compl;
	unsigned int bytes_compl;
	unsigned int descs_cleaned;
	unsigned int completion_events;
	u32 pending_before;
	struct rtlmq_tx_reclaim_wait wait;
};

static __always_inline void
rtlmq_tx_clear_completion_owner(struct rtlmq_tx_completion *completion)
{
	if (!completion)
		return;

	completion->xdpf = NULL;
	completion->completion_bytes = 0;
	completion->timestamp_generation = 0;
	completion->xsk_descs = 0;
	completion->type = RTLMQ_TX_BUFFER_NONE;
	completion->flags = 0;
}

static __always_inline bool
rtlmq_tx_completion_uses_xdp_api(const struct rtlmq_tx_completion *completion)
{
	if (!completion)
		return false;

	return completion->type == RTLMQ_TX_BUFFER_XDP_NDO ||
	       completion->type == RTLMQ_TX_BUFFER_XDP_RX;
}

static bool rtlmq_tx_completion_budget_allows_xdp(const struct rtlmq_tx_completion *completion,
						  int budget)
{
	/*
	 * NAPI budget-zero polls may clean TX SKB completions, but must not
	 * call RX, XDP, or page-pool APIs. Leave XDP-frame completions pending
	 * until a nonzero-budget poll can return the frame.
	 */
	return budget > 0 || !rtlmq_tx_completion_uses_xdp_api(completion);
}

static __always_inline void rtlmq_return_xdp_frame(struct xdp_frame *xdpf)
{
	if (!xdpf)
		return;

	/*
	 * The frame's page pool may belong to a different RX NAPI. Use the
	 * serialized return path, not xdp_return_frame_rx_napi(), which allows
	 * direct recycling only from that pool's NAPI.
	 */
	xdp_return_frame(xdpf);
}

static void rtlmq_tx_account_completion_stats_fast(struct rtlmq_tx_ring *ring,
						   unsigned int packets,
						   unsigned int bytes)
{
	if (!packets && !bytes)
		return;

	u64_stats_update_begin(&ring->comp_syncp);
	ring->stats.packets += packets;
	ring->stats.bytes += bytes;
	u64_stats_update_end(&ring->comp_syncp);
}

static __always_inline void
rtlmq_tx_complete_skb_owned(struct rtlmq_tx_ring *ring,
			    struct rtlmq_tx_completion *completion, int budget)
{
	if (!completion->skb)
		return;

	if (completion->flags & RTLMQ_TX_COMPLETIONF_HW_TSTAMP) {
		rtlmq_ptp_tx_complete(ring->priv,
				      completion->timestamp_generation);
		completion->flags &= ~RTLMQ_TX_COMPLETIONF_HW_TSTAMP;
	}
	napi_consume_skb(completion->skb, budget);
}

static __always_inline void rtlmq_tx_complete_eop_owned(struct rtlmq_tx_ring *ring,
							const struct rtlmq_tx_info *info,
							struct rtlmq_tx_completion *completion,
							int budget, unsigned int *pkts,
							unsigned int *bytes, unsigned int *events)
{
	if (!info || !completion)
		return;

	*pkts += info->completion_packets;
	*bytes += completion->completion_bytes;
	(*events)++;

	if (likely(completion->type == RTLMQ_TX_BUFFER_SKB)) {
		rtlmq_tx_complete_skb_owned(ring, completion, budget);
		return;
	}

	switch (completion->type) {
	case RTLMQ_TX_BUFFER_XDP_NDO:
	case RTLMQ_TX_BUFFER_XDP_RX:
		rtlmq_return_xdp_frame(completion->xdpf);
		break;
	case RTLMQ_TX_BUFFER_XSK:
		if (completion->xsk_pool) {
			bool owned;

			owned = rtlmq_ptp_xsk_tx_complete(
				ring->priv, completion->xsk_pool,
				completion->xsk_descs,
				completion->flags & RTLMQ_TX_COMPLETIONF_HW_TSTAMP ?
					completion->timestamp_generation : 0);
			completion->flags &=
				~RTLMQ_TX_COMPLETIONF_HW_TSTAMP;
			if (owned)
				break;
		}
		if (completion->xsk_pool)
			rtlmq_xsk_tx_completed(completion->xsk_pool,
					       completion->xsk_descs);
		break;
	case RTLMQ_TX_BUFFER_NONE:
	default:
		break;
	}
}

void rtlmq_tx_finish_eop(struct rtlmq_tx_ring *ring, unsigned int entry,
			 int budget, unsigned int *pkts, unsigned int *bytes,
			 unsigned int *events)
{
	struct rtlmq_tx_completion *completion = &ring->completions[entry];
	struct rtlmq_tx_info *info = &ring->infos[entry];

	rtlmq_tx_complete_eop_owned(ring, info, completion, budget, pkts, bytes,
				    events);
}

static void
rtlmq_tx_finish_skb_eop_fast(struct rtlmq_tx_ring *ring,
			     unsigned int entry, int budget,
			     unsigned int *pkts, unsigned int *bytes,
			     unsigned int *events)
{
	struct rtlmq_tx_completion *completion = &ring->completions[entry];
	struct rtlmq_tx_info *info = &ring->infos[entry];

	*pkts += info->completion_packets;
	*bytes += completion->completion_bytes;
	(*events)++;

	rtlmq_tx_complete_skb_owned(ring, completion, budget);
}

static int rtlmq_tx_bql_completed_fast(struct rtlmq_tx_ring *ring,
				       unsigned int objects, unsigned int bytes)
{
	struct netdev_queue *txq = rtlmq_tx_ring_rm(ring)->txq;
	int wake_result = -1;

	ring->bql_completed_packets += objects;
	ring->bql_completed_bytes += bytes;

	if (unlikely(!txq || !rtlmq_netstack_tx_queue_live(ring)))
		return wake_result;

	netdev_txq_bql_complete_prefetchw(txq);
	if (likely(rtlmq_tx_completion_wake_allowed(ring)))
		wake_result = netif_txq_completed_wake(txq, objects, bytes,
						       rtlmq_tx_desc_avail(ring),
						       rtlmq_tx_wake_desc_high(ring));
	else
		wake_result = __netif_txq_completed_wake(txq, objects, bytes,
							 rtlmq_tx_desc_avail(ring),
							 rtlmq_tx_wake_desc_high(ring), true);
	/*
	 * qdisc/BQL-visible completion progress can arrive while the queue is
	 * briefly stack-XOFF. Refresh the per-queue watchdog timestamp from the
	 * same completion event so dev_watchdog() sees live TX progress instead
	 * of only private driver clean timestamps.
	 */
	txq_trans_cond_update(txq);
	return wake_result;
}

void rtlmq_tx_unmap_sg_txn_owned(struct rtlmq_tx_ring *ring, unsigned int entry)
{
	struct rtlmq_tx_dma_txn *txn;
	u16 orig_nents;

	if (WARN_ON_ONCE(!ring || !ring->dma_txns ||
			 entry >= ring->ring_count))
		return;

	txn = &ring->dma_txns[entry];
	if (!READ_ONCE(txn->mapped))
		return;

	orig_nents = READ_ONCE(txn->orig_nents);
	if (WARN_ON_ONCE(!orig_nents ||
			 orig_nents > RTLMQ_TX_DMA_SG_NENTS))
		return;

	rtlmq_tx_dma_unmap_sg(ring->priv, txn->sg, orig_nents);
	WRITE_ONCE(txn->mapped, false);
	WRITE_ONCE(txn->orig_nents, 0);
}

static __always_inline void
rtlmq_tx_unmap_mapping_owned(struct rtlmq_tx_ring *ring, unsigned int entry,
			     const struct rtlmq_tx_info *info, u16 flags)
{
	if ((flags & (RTLMQ_TX_INFOF_MAPPED | RTLMQ_TX_INFOF_UNMAP_OWNER)) !=
	    (RTLMQ_TX_INFOF_MAPPED | RTLMQ_TX_INFOF_UNMAP_OWNER))
		return;

	/*
	 * Caller owns completion, unpublished unwind, or lifecycle release
	 * permission. Only UNMAP_OWNER releases a shared mapping.
	 */
	if (flags & RTLMQ_TX_INFOF_DMA_SG)
		rtlmq_tx_unmap_sg_txn_owned(ring, entry);
	else if (flags & RTLMQ_TX_INFOF_PAGE)
		rtlmq_tx_dma_unmap_page_tracked(ring, info->unmap_dma,
						info->unmap_len);
	else
		rtlmq_tx_dma_unmap_single_tracked(ring, info->unmap_dma,
						  info->unmap_len);
}

static void rtlmq_tx_unmap_single_owner_nostat(struct rtlmq_tx_ring *ring,
					       const struct rtlmq_tx_info *info)
{
	rtlmq_tx_dma_unmap_single(ring->priv, info->unmap_dma,
				  info->unmap_len);
}

static void rtlmq_tx_recycle_entry_owned(struct rtlmq_tx_ring *ring,
					 unsigned int entry,
					 struct rtlmq_tx_info *info)
{
	/*
	 * After reclaim removes a slot from the pending window, clear its
	 * flags and completion pointer. Leave dma_addr,
	 * unmap_dma, desc_len, unmap_len, generation, and descriptor contents
	 * intentionally stale so the next publisher can overwrite them without
	 * a whole-slot memset in the steady state. Rollback also clears the
	 * descriptor because a partially built chain may have OWN set on
	 * non-head entries.
	 */
	WRITE_ONCE(info->flags, 0);
	WRITE_ONCE(info->completion_packets, 0);
	rtlmq_tx_clear_completion_owner(&ring->completions[entry]);
}

static void rtlmq_tx_retire_completed_range(struct rtlmq_tx_ring *ring,
					    unsigned int entry,
					    unsigned int count, int budget,
					    struct rtlmq_tx_poll_batch *batch)
{
	struct rtlmq_tx_completion *completions = ring->completions;
	struct rtlmq_tx_info *infos = ring->infos;
	unsigned int mask = ring->ring_mask;

	/*
	 * Reclaim has acquired OWN-clear evidence for every descriptor here,
	 * and admitted any EOP against the NAPI budget. Serialized ranges are
	 * complete packets; native fixtures may retire a partial packet.
	 * The caller publishes consumer progress only after all releases.
	 *
	 * Mapping ownership is descriptor-specific, not synonymous with EOP.
	 * Packet completion storage is written only at EOP by every producer,
	 * and cleared there before reuse. Middle slots need only info cleanup.
	 */
	while (count--) {
		struct rtlmq_tx_info *info = &infos[entry];
		u16 flags = READ_ONCE(info->flags);

		rtlmq_tx_unmap_mapping_owned(ring, entry, info, flags);
		WRITE_ONCE(info->flags, 0);
		if (flags & RTLMQ_TX_INFOF_EOP) {
			struct rtlmq_tx_completion *completion =
				&completions[entry];

			rtlmq_tx_complete_eop_owned(ring, info, completion,
						    budget, &batch->pkts_compl,
						    &batch->bytes_compl,
						    &batch->completion_events);
			rtlmq_tx_clear_completion_owner(completion);
		}
		WRITE_ONCE(info->completion_packets, 0);
		entry = (entry + 1) & mask;
	}
}

void rtlmq_unmap_tx_info(struct rtlmq_tx_ring *ring, unsigned int entry)
{
	struct rtlmq_tx_info *info = &ring->infos[entry];
	u16 flags = READ_ONCE(info->flags);

	if (!(flags & RTLMQ_TX_INFOF_MAPPED))
		return;

	rtlmq_tx_unmap_mapping_owned(ring, entry, info, flags);
	WRITE_ONCE(info->flags,
		   flags & ~(RTLMQ_TX_INFOF_MAPPED | RTLMQ_TX_INFOF_PAGE |
			     RTLMQ_TX_INFOF_UNMAP_OWNER | RTLMQ_TX_INFOF_TSO |
			     RTLMQ_TX_INFOF_DMA_SG));
}

static void
rtlmq_release_tx_completion_slow(struct rtlmq_tx_ring *ring,
				 struct rtlmq_tx_completion *completion)
{
	if (completion->flags & RTLMQ_TX_COMPLETIONF_HW_TSTAMP) {
		rtlmq_ptp_tx_cancel(ring->priv);
		completion->flags &= ~RTLMQ_TX_COMPLETIONF_HW_TSTAMP;
	}

	switch (completion->type) {
	case RTLMQ_TX_BUFFER_SKB:
		if (completion->skb)
			dev_kfree_skb_any(completion->skb);
		break;
	case RTLMQ_TX_BUFFER_XDP_NDO:
	case RTLMQ_TX_BUFFER_XDP_RX:
		rtlmq_return_xdp_frame(completion->xdpf);
		break;
	case RTLMQ_TX_BUFFER_XSK:
		if (completion->xsk_pool &&
		    !rtlmq_xsk_pool_detached(ring->priv, completion->xsk_pool))
			rtlmq_xsk_tx_completed(completion->xsk_pool,
					       completion->xsk_descs);
		break;
	case RTLMQ_TX_BUFFER_NONE:
	default:
		break;
	}
}

void rtlmq_free_tx_info(struct rtlmq_tx_ring *ring, unsigned int entry)
{
	struct rtlmq_tx_completion *completion = &ring->completions[entry];
	struct rtlmq_tx_info *info = &ring->infos[entry];

	rtlmq_unmap_tx_info(ring, entry);
	rtlmq_release_tx_completion_slow(ring, completion);

	memset(info, 0, sizeof(*info));
	memset(completion, 0, sizeof(*completion));
	rtlmq_clear_tx_desc(ring, entry);
}

static void rtlmq_recycle_tx_entry(struct rtlmq_tx_ring *ring,
				   unsigned int entry)
{
	struct rtlmq_tx_info *info = &ring->infos[entry];

	rtlmq_tx_recycle_entry_owned(ring, entry, info);
}

void rtlmq_unwind_tx_entries(struct rtlmq_tx_ring *ring, unsigned int start,
			     unsigned int mapped)
{
	unsigned int i;

	for (i = 0; i < mapped; i++) {
		unsigned int entry = (start + i) & ring->ring_mask;

		rtlmq_unmap_tx_info(ring, entry);
		rtlmq_recycle_tx_entry(ring, entry);
		/*
		 * These entries were never reachable through the published
		 * producer tail. Clear descriptor state during
		 * rollback so a prepared non-head OWN bit cannot survive a
		 * partial-map failure into the next publication.
		 */
		rtlmq_clear_tx_desc(ring, entry);
	}
}

/*
 * NAPI pins these CPU-owned fields until the poll releases ownership. This is
 * a round-local storage/generation snapshot, not cached device OWN evidence or
 * a snapshot of producer progress, queue state, or MMIO authority.
 */
struct rtlmq_tx_reclaim_context {
	struct rtlmq_tx_info *infos;
	struct rtlmq_tx_completion *completions;
	struct rtlmq_tx_desc *desc;
	u8 desc_stride;
	u32 generation;
	u16 ring_count;
	u16 ring_mask;
};

static inline struct rtlmq_tx_reclaim_context
rtlmq_tx_reclaim_context(const struct rtlmq_tx_ring *ring)
{
	const struct rtlmq_tx_ring_read_mostly *rm = rtlmq_tx_ring_rm(ring);
	u32 generation;

	/* Pairs with rtlmq_tx_ring_publish_generation() before slot reuse. */
	generation = smp_load_acquire(&ring->generation);

	return (struct rtlmq_tx_reclaim_context){
		.infos = rm->infos,
		.completions = rm->completions,
		.desc = rm->desc,
		.desc_stride = rtlmq_tx_ring_desc_stride(ring),
		.generation = generation,
		.ring_count = rm->ring_count,
		.ring_mask = rm->ring_mask,
	};
}

static __always_inline bool
rtlmq_tx_desc_status_prepare_reclaim_hint_with_override(struct rtlmq_tx_ring *ring)
{
	if (!ring)
		return false;

	rtlmq_tx_ring_note_desc_status_pending(ring);
	return READ_ONCE(ring->tx_reclaim_hint) != 0;
}

static bool rtlmq_tx_reclaim_close_own_desc_status(struct rtlmq_tx_ring *ring, int budget,
						   struct rtlmq_tx_poll_batch *batch,
						   u32 pending_before,
						   const struct rtlmq_tx_reclaim_context *context);
static void rtlmq_tx_desc_status_irq_sync(struct rtlmq_tx_ring *ring)
{
	if (rtlmq_tx_ring_tx_irq_wanted(ring))
		rtlmq_sync_tx_ring_irq_arming(ring);
}

static __always_inline void
rtlmq_tx_reclaim_close_own_commit(struct rtlmq_tx_poll_batch *batch,
				  unsigned int retired)
{
	if (!retired)
		return;

	batch->descs_cleaned += retired;
}

static __always_inline void
rtlmq_tx_publish_clean_batch(struct rtlmq_tx_ring *ring,
			     unsigned int next_to_clean,
			     unsigned int descs)
{
	u32 use_count;
	u32 pending;
	u32 count;

	if (!descs)
		return;

	WRITE_ONCE(ring->next_to_clean, next_to_clean);
	count = READ_ONCE(ring->next_to_clean_count);
	count += descs;
	/* Pairs with smp_load_acquire() readers of next_to_clean_count. */
	smp_store_release(&ring->next_to_clean_count, count);

	use_count = READ_ONCE(ring->next_to_use_count);
	pending = use_count - count;
	if (!pending)
		(void)xchg(&ring->own_recheck_reasons, 0UL);
}

static noinline __cold bool
rtlmq_tx_entry_generation_invalid(struct rtlmq_tx_ring *ring,
				  unsigned int entry, u32 slot_generation,
				  u32 ring_generation)
{
	WARN_ONCE(1,
		  "rtlmq: txq%u stale TX entry entry=%u ntc=%u ntu=%u slot_generation=%u ring_generation=%u\n",
		  ring->queue_index, entry,
		  READ_ONCE(ring->next_to_clean),
		  READ_ONCE(ring->next_to_use),
		  slot_generation, ring_generation);

	return false;
}

static __always_inline bool
rtlmq_tx_entry_generation_valid(struct rtlmq_tx_ring *ring,
				const struct rtlmq_tx_reclaim_context *ctx,
				unsigned int entry)
{
	const struct rtlmq_tx_info *info;
	u32 slot_generation;

	if (!ctx->infos || entry >= ctx->ring_count)
		return false;

	info = &ctx->infos[entry];
	slot_generation = READ_ONCE(info->generation);

	if (likely(rtlmq_tx_slot_generation_valid(slot_generation,
						  ctx->generation)))
		return true;

	return rtlmq_tx_entry_generation_invalid(ring, entry, slot_generation,
						 ctx->generation);
}

static noinline bool rtlmq_tx_desc_status_recheck(struct rtlmq_tx_ring *ring,
						  struct rtlmq_tx_desc *desc,
						  u32 *opts1)
{
	struct rtlmq_private *tp;
	unsigned long reasons;
	bool own_cleared;

	tp = ring->priv;
	if (!tp || !rtlmq_mmio_ordinary_authorized(tp))
		return false;

	reasons = xchg(&ring->own_recheck_reasons, 0UL);
	if (!reasons)
		return false;

	/*
	 * Match the vendor close-mode workaround exactly: a normal 8-bit read
	 * from primary IMR on this PCI function orders any earlier device DMA
	 * writes before the immediate descriptor-memory reread. A relaxed MMIO
	 * accessor or another coherent-memory load is not equivalent.
	 */
	(void)readb(tp->mmio_addr + RTLMQ_INTR_MASK);
	own_cleared = rtlmq_tx_desc_acquire(desc, opts1);

	return own_cleared;
}

static __always_inline bool
rtlmq_tx_desc_status_acquire_rechecked(struct rtlmq_tx_ring *ring,
				       struct rtlmq_tx_desc *desc, u32 *opts1)
{
	if (rtlmq_tx_desc_acquire(desc, opts1))
		return true;

	return rtlmq_tx_desc_status_recheck(ring, desc, opts1);
}

static __always_inline bool
rtlmq_tx_desc_status_reclaimable(struct rtlmq_tx_ring *ring,
				 const struct rtlmq_tx_reclaim_context *ctx, unsigned int entry,
				 u32 *opts1, bool *generation_invalid)
{
	*generation_invalid =
		!rtlmq_tx_entry_generation_valid(ring, ctx, entry);
	if (unlikely(*generation_invalid))
		return false;

	/*
	 * Reclaim only after hardware clears OWN and the DMA read barrier
	 * orders subsequent descriptor reads.
	 */
	return rtlmq_tx_desc_status_acquire_rechecked(
		ring, rtlmq_tx_desc_at_base(ctx->desc, ctx->desc_stride, entry),
		opts1);
}

static noinline __cold bool
rtlmq_tx_reclaim_desc_status_poisoned(struct rtlmq_tx_ring *ring, struct rtlmq_tx_poll_batch *batch,
				      unsigned int retired, unsigned int entry, const char *where,
				      bool generation_invalid)
{
	struct rtlmq_private *tp = ring->priv;
	struct rtlmq_tx_reclaim_context ctx = rtlmq_tx_reclaim_context(ring);
	const struct rtlmq_tx_info *info;
	u16 flags;

	if (batch)
		rtlmq_tx_reclaim_close_own_commit(batch, retired);

	if (generation_invalid ||
	    !rtlmq_tx_entry_generation_valid(ring, &ctx, entry))
		goto request_reset;

	info = &ctx.infos[entry];
	flags = READ_ONCE(info->flags);
	if (!(flags & RTLMQ_TX_INFOF_MAPPED))
		netdev_err(tp->ndev,
			   "tx q%u desc_status stale slot where=%s entry=%u ntc=%u ntu=%u flags=%#x info_gen=%u ring_gen=%u\n",
			   ring->queue_index, where ? where : "unknown", entry,
			   READ_ONCE(ring->next_to_clean),
			   READ_ONCE(ring->next_to_use), flags,
			   READ_ONCE(info->generation), ctx.generation);

request_reset:
	rtlmq_request_reset(tp, BIT(RTLMQ_RESET_REASON_TX_TIMEOUT)
	);
	return false;
}

static noinline __cold bool
rtlmq_tx_pending_window_invalid(struct rtlmq_tx_ring *ring, u32 pending,
				const char *where)
{
	struct rtlmq_private *tp = ring->priv;

	if (tp && tp->ndev)
		netdev_warn_once(tp->ndev,
				 "tx q%u ownership window invalid where=%s pending=%u ring=%u use=%u clean=%u; requesting reset\n",
				 ring->queue_index, where ? where : "unknown", pending,
				 READ_ONCE(ring->ring_count),
				 READ_ONCE(ring->next_to_use_count),
				 READ_ONCE(ring->next_to_clean_count));
	else
		WARN_ONCE(1,
			  "rtlmq: TX ownership window invalid where=%s pending=%u ring=%u\n",
			  where ? where : "unknown", pending,
			  READ_ONCE(ring->ring_count));

	if (tp)
		rtlmq_request_reset(tp, BIT(RTLMQ_RESET_REASON_TX_TIMEOUT)
		);
	return false;
}

static __always_inline bool
rtlmq_tx_pending_window_valid(struct rtlmq_tx_ring *ring, u32 pending,
			      const char *where)
{
	u16 ring_count = READ_ONCE(ring->ring_count);

	/* One descriptor is always reserved to distinguish full from empty. */
	if (likely(ring_count && pending < ring_count))
		return true;

	return rtlmq_tx_pending_window_invalid(ring, pending, where);
}

static __always_inline bool
rtlmq_tx_reclaim_desc_status_preflight_needed(const struct rtlmq_tx_ring *ring, u32 pending_before)
{
	return !READ_ONCE(ring->ring_count) ||
	       pending_before >= READ_ONCE(ring->ring_count);
}

static noinline __cold bool
rtlmq_tx_reclaim_desc_status_preflight_cold(struct rtlmq_tx_ring *ring,
					    u32 pending_before,
					    bool *force_subqueue_sync)
{
	if (unlikely(!READ_ONCE(ring->ring_count) ||
		     pending_before >= READ_ONCE(ring->ring_count))) {
		*force_subqueue_sync = true;
		(void)rtlmq_tx_pending_window_invalid(ring, pending_before,
						      "poll_enter_impossible");
		return false;
	}

	return true;
}

static noinline bool
rtlmq_tx_reclaim_close_own_desc_status(struct rtlmq_tx_ring *ring, int budget,
				       struct rtlmq_tx_poll_batch *batch, u32 pending_before,
				       const struct rtlmq_tx_reclaim_context *context)
{
	const struct rtlmq_tx_reclaim_context ctx = *context;
	struct rtlmq_tx_info *infos = ctx.infos;
	struct rtlmq_tx_completion *completions = ctx.completions;
	unsigned int mask = ctx.ring_mask;
	unsigned int entry;
	unsigned int pending = pending_before;
	unsigned int retired = 0;

	if (unlikely(!rtlmq_tx_pending_window_valid(ring, pending_before, "desc_status_reclaim")))
		return false;

	/*
	 * Hardware clears OWN on each completed descriptor. Reclaim the
	 * completed prefix and stop at the first descriptor with OWN set.
	 */
	entry = READ_ONCE(ring->next_to_clean);
	while (pending) {
		unsigned int extent;
		bool generation_invalid = false;
		unsigned int probe = entry;
		unsigned int n;
		u32 opts1 = 0;

		{
			/*
			 * Staged dirty is packet-aligned. Verify the complete
			 * published packet before retiring its prefix, including
			 * the budget-zero XDP return restriction.
			 */
			extent = infos[entry].packet.descs;
			if (!extent || extent > pending ||
			    !(infos[(entry + extent - 1) & mask].flags &
			      RTLMQ_TX_INFOF_EOP)) {
				rtlmq_tx_publish_clean_batch(ring, entry,
							     retired);
				return rtlmq_tx_reclaim_desc_status_poisoned(ring, batch,
									    retired, entry,
									    "serialized_packet_boundary",
									    false);
			}
			if (budget <= 0 &&
			    !rtlmq_tx_completion_budget_allows_xdp(
				    &completions[(entry + extent - 1) & mask],
				    budget))
				break;
		}
		for (n = 0; n < extent; n++) {
			probe = (entry + n) & mask;
			if (!rtlmq_tx_desc_status_reclaimable(ring, &ctx, probe, &opts1,
							      &generation_invalid))
				break;
		}
		if (generation_invalid) {
			rtlmq_tx_publish_clean_batch(ring, entry, retired);
			return rtlmq_tx_reclaim_desc_status_poisoned(ring, batch, retired, probe,
								     "reclaim_close_own_generation",
								     true);
		}
		if (n != extent) {
			{
				/*
				 * Keep dirty packet-aligned, but do not mistake its
				 * already-returned head for work on NAPI completion.
				 */
				batch->wait = (struct rtlmq_tx_reclaim_wait){
					.clean_count =
						ring->next_to_clean_count +
						retired,
					.blocked_count =
						ring->next_to_clean_count +
						retired + n,
					.generation = ctx.generation,
					.valid = true,
				};
			}
			break;
		}

		rtlmq_tx_retire_completed_range(ring, entry, extent, budget,
						batch);
		retired += extent;
		entry = (entry + extent) & mask;
		pending -= extent;
	}

	rtlmq_tx_publish_clean_batch(ring, entry, retired);
	rtlmq_tx_reclaim_close_own_commit(batch, retired);
	return true;
}

static bool rtlmq_tx_reclaim_close_own_fast(struct rtlmq_tx_ring *ring,
					    int budget,
					    struct rtlmq_tx_poll_batch *batch,
					    bool *force_subqueue_sync)
{
	const u16 fast_flags = RTLMQ_TX_INFOF_MAPPED |
			       RTLMQ_TX_INFOF_UNMAP_OWNER |
			       RTLMQ_TX_INFOF_EOP;
	const u16 small_copy_flags = RTLMQ_TX_INFOF_SMALL_COPY |
				     RTLMQ_TX_INFOF_EOP;
	unsigned int entry = READ_ONCE(ring->next_to_clean);
	struct rtlmq_tx_reclaim_context ctx;
	bool force_sync_local = false;
	unsigned int pending = 0;
	unsigned int retired = 0;
	u32 reclaim_hint;

	/*
	 * Consume the scan request before reading descriptors. Stop on OWN;
	 * never loop waiting for hardware. The NAPI owner can request another
	 * sweep while RX keeps it scheduled, without requiring a new TX IRQ.
	 */
	reclaim_hint = xchg(&ring->tx_reclaim_hint, 0);
	if (reclaim_hint) {
		pending = rtlmq_tx_ring_desc_status_hint_window(ring);
		if (!pending)
			(void)xchg(&ring->own_recheck_reasons, 0UL);
	}

	if (!force_subqueue_sync)
		force_subqueue_sync = &force_sync_local;
	batch->pending_before = pending;
	if (unlikely(rtlmq_tx_reclaim_desc_status_preflight_needed(ring, pending)) &&
	    !rtlmq_tx_reclaim_desc_status_preflight_cold(ring, pending, force_subqueue_sync))
		return true;

	/*
	 * NAPI retains the admitted ring storage through reclaim. Generation
	 * reuse requires NAPI/producer quiescence, so acquire it once for this
	 * round, not once per descriptor. Slot tags are still checked each time.
	 * The published producer frontier and final IRQ/rearm intent remain live.
	 */
	ctx = rtlmq_tx_reclaim_context(ring);
	while (pending) {
		struct rtlmq_tx_info *info;
		u32 opts1;
		u16 flags;

		if (unlikely(!rtlmq_tx_entry_generation_valid(ring, &ctx,
							      entry))) {
			rtlmq_tx_publish_clean_batch(ring, entry, retired);
			return rtlmq_tx_reclaim_desc_status_poisoned(ring, batch, retired, entry,
								     "reclaim_close_own_fast_generation",
								     true);
		}

		info = &ctx.infos[entry];
		/* Metadata is immutable within the acquired published interval. */
		flags = READ_ONCE(info->flags);
		if (unlikely(flags != fast_flags &&
			     flags != small_copy_flags)) {
			if (retired) {
				rtlmq_tx_publish_clean_batch(ring, entry,
							     retired);
				rtlmq_tx_reclaim_close_own_commit(batch, retired);
			}
			return rtlmq_tx_reclaim_close_own_desc_status(ring, budget, batch, pending,
								      &ctx);
		}

		if (unlikely(!rtlmq_tx_desc_status_acquire_rechecked(
			    ring, rtlmq_tx_desc_at_base(ctx.desc,
							ctx.desc_stride, entry),
			    &opts1)))
			break;

		if (budget <= 0 &&
		    unlikely(!rtlmq_tx_completion_budget_allows_xdp(&ctx.completions[entry],
								    budget)))
			break;

		retired++;
		if (flags == fast_flags)
			rtlmq_tx_unmap_single_owner_nostat(ring, info);
		if (likely(ctx.completions[entry].type == RTLMQ_TX_BUFFER_SKB))
			rtlmq_tx_finish_skb_eop_fast(ring, entry, budget, &batch->pkts_compl,
						     &batch->bytes_compl,
						     &batch->completion_events);
		else
			rtlmq_tx_finish_eop(ring, entry, budget,
					    &batch->pkts_compl,
					    &batch->bytes_compl,
					    &batch->completion_events);
		rtlmq_tx_recycle_entry_owned(ring, entry, info);
		entry = (entry + 1) & ctx.ring_mask;
		pending--;
	}

	rtlmq_tx_publish_clean_batch(ring, entry, retired);
	rtlmq_tx_reclaim_close_own_commit(batch, retired);
	return true;
}

static __always_inline bool
rtlmq_tx_clean_may_need_subqueue_sync(struct rtlmq_tx_ring *ring)
{
	struct netdev_queue *txq = ring->ndev_queue;

	if (READ_ONCE(ring->tx_subqueue_state) != RTLMQ_TX_SUBQUEUE_READY)
		return true;

	return txq && rtlmq_netstack_tx_queue_live(ring) &&
	       (netif_xmit_frozen_or_stopped(txq) ||
		(READ_ONCE(txq->state) & BIT(__QUEUE_STATE_STACK_XOFF)));
}

static __always_inline void
rtlmq_tx_complete_reclaim_transaction(struct rtlmq_tx_ring *ring,
				      const struct rtlmq_tx_poll_batch *batch, u32 pending_before,
				      u32 pending_after)
{
	int completed_wake_result = -1;

	/*
	 * Descriptor scanning has already unmapped DMA, released completed
	 * objects, recycled their slots, and release-published next_to_clean.
	 * Account the released objects, report BQL completion, wake the core
	 * queue, then update driver queue state.
	 */
	rtlmq_tx_account_completion_stats_fast(ring, batch->pkts_compl,
					       batch->bytes_compl);

	if (batch->completion_events) {
		/*
		 * BQL completion can schedule a stopped queue's qdisc. Report
		 * to the core only after netdev registration.
		 */
		completed_wake_result = rtlmq_tx_bql_completed_fast(ring, batch->completion_events,
								    batch->bytes_compl);
	}

	if (unlikely(batch->completion_events &&
		     (completed_wake_result == 0 ||
		      rtlmq_tx_clean_may_need_subqueue_sync(ring)))) {
		rtlmq_sync_tx_subqueue_after_clean(ring, batch->descs_cleaned,
						   pending_before,
						   pending_after);
	}
}

struct rtlmq_tx_poll_result
rtlmq_poll_tx_ring_with_hint(struct rtlmq_tx_ring *ring, int budget)
{
	struct rtlmq_tx_poll_result result = {};
	struct rtlmq_tx_poll_batch batch = {};
	bool force_subqueue_sync = false;
	u32 pending_before;
	u32 pending_after;

	/*
	 * budget is NAPI skb-consume context, not a TX reclaim limit. The
	 * desc_status pending hint and descriptor OWN bits decide whether TX
	 * completions can be reclaimed. Budget-zero polls may reclaim non-XDP
	 * completions, but leave XDP-frame returns pending for a nonzero-budget
	 * poll.
	 */
	if (likely(!rtlmq_tx_desc_status_prepare_reclaim_hint_with_override(ring)))
		return result;

	if (unlikely(!rtlmq_tx_reclaim_close_own_fast(ring, budget, &batch,
						      &force_subqueue_sync)))
		force_subqueue_sync = true;
	pending_before = batch.pending_before;
	result.wait = batch.wait;

	pending_after = READ_ONCE(ring->next_to_use_count) -
			READ_ONCE(ring->next_to_clean_count);

	if (unlikely(force_subqueue_sync))
		rtlmq_sync_tx_datapath_subqueue(ring);

	if (!batch.descs_cleaned && !batch.completion_events) {
		rtlmq_tx_desc_status_irq_sync(ring);
		return result;
	}

	result.pkts_cleaned = batch.pkts_compl;
	rtlmq_tx_complete_reclaim_transaction(ring, &batch, pending_before, pending_after);
	rtlmq_tx_desc_status_rekick_pending(ring, pending_after);
	rtlmq_tx_desc_status_irq_sync(ring);
	return result;
}
