/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef __RTLMQ_TX_INTERNAL_H__
#define __RTLMQ_TX_INTERNAL_H__

/* Internal TX DMA and slot-generation helpers. */
#include "rtlmq.h"
#include "rtlmq_tx.h"

static __always_inline bool rtlmq_tx_slot_generation_valid(u32 slot_generation,
							   u32 ring_generation)
{
	if (slot_generation == ring_generation)
		return true;

	return false;
}

static inline dma_addr_t rtlmq_tx_dma_map_single(struct rtlmq_private *tp,
						 void *data, size_t len)
{
	return dma_map_single(&tp->pdev->dev, data, len, DMA_TO_DEVICE);
}

static inline dma_addr_t rtlmq_tx_dma_map_frag(struct rtlmq_private *tp,
					       const skb_frag_t *frag,
					       size_t offset, size_t len)
{
	return skb_frag_dma_map(&tp->pdev->dev, frag, offset, len,
				DMA_TO_DEVICE);
}

static inline bool rtlmq_tx_dma_mapping_error(struct rtlmq_private *tp,
					      dma_addr_t dma_addr)
{
	return dma_mapping_error(&tp->pdev->dev, dma_addr) != 0;
}

/*
 * Descriptor-status reclaim reissues the ring's one-hot queue command after
 * it advances the consumer but leaves a device-owned suffix.
 */
bool rtlmq_tx_desc_status_rekick_pending(struct rtlmq_tx_ring *ring,
					 u32 pending_after);

static inline int rtlmq_tx_dma_map_sg(struct rtlmq_private *tp,
				      struct scatterlist *sg, int orig_nents)
{
	return dma_map_sg(&tp->pdev->dev, sg, orig_nents, DMA_TO_DEVICE);
}

static inline void rtlmq_tx_dma_unmap_sg(struct rtlmq_private *tp,
					 struct scatterlist *sg, int orig_nents)
{
	dma_unmap_sg(&tp->pdev->dev, sg, orig_nents, DMA_TO_DEVICE);
}

static inline void rtlmq_tx_dma_unmap_single(struct rtlmq_private *tp,
					     dma_addr_t dma_addr, size_t len)
{
	/*
	 * Caller owns completion, unpublished unwind, or lifecycle release
	 * permission. This wrapper does not stop DMA.
	 */

	dma_unmap_single(&tp->pdev->dev, dma_addr, len, DMA_TO_DEVICE);
}

static inline void rtlmq_tx_dma_unmap_page(struct rtlmq_private *tp,
					   dma_addr_t dma_addr, size_t len)
{
	dma_unmap_page(&tp->pdev->dev, dma_addr, len, DMA_TO_DEVICE);
}

static inline void rtlmq_tx_dma_unmap_single_tracked(struct rtlmq_tx_ring *ring,
						     dma_addr_t dma_addr,
						     size_t len)
{
	if (WARN_ON_ONCE(!ring))
		return;

	rtlmq_tx_dma_unmap_single(ring->priv, dma_addr, len);
}

static inline void
rtlmq_tx_dma_unmap_single_producer_owned(struct rtlmq_tx_ring *ring,
					 dma_addr_t dma_addr, size_t len)
{
	if (WARN_ON_ONCE(!ring || !ring->priv))
		return;

	/*
	 * Descriptor setup rejected this mapping before it reached hardware.
	 */
	rtlmq_tx_dma_unmap_single(ring->priv, dma_addr, len);
}

static inline void rtlmq_tx_dma_unmap_page_tracked(struct rtlmq_tx_ring *ring,
						   dma_addr_t dma_addr,
						   size_t len)
{
	if (WARN_ON_ONCE(!ring))
		return;

	rtlmq_tx_dma_unmap_page(ring->priv, dma_addr, len);
}

static inline void rtlmq_tx_dma_sync_for_device(struct rtlmq_private *tp,
						dma_addr_t dma_addr, size_t len,
						enum dma_data_direction dir)
{
	dma_sync_single_for_device(&tp->pdev->dev, dma_addr, len, dir);
}

#endif /* __RTLMQ_TX_INTERNAL_H__ */
