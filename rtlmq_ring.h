/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef __RTLMQ_RING_H__
#define __RTLMQ_RING_H__

#include <linux/dma-mapping.h>
#include <linux/overflow.h>

#include "rtlmq_base.h"

struct rtlmq_private;
struct rtlmq_live_service_pause;
struct rtlmq_live_rx_ring_set;
struct rtlmq_live_tx_ring_set;
struct rtlmq_rx_ring_replacement;
struct sk_buff;
struct xsk_buff_pool;

int rtlmq_ring_preferred_nid(const struct rtlmq_private *tp);
int rtlmq_validate_ring_counts(u32 rx_ring_count, u32 tx_ring_count);
size_t rtlmq_tx_desc_ring_bytes_for_stride(u16 ring_count, u8 stride);
size_t rtlmq_tx_ring_allocation_bytes_for_stride(u16 ring_count, u8 stride);
int rtlmq_alloc_rx_ring(struct rtlmq_rx_ring *ring);
/* Caller owns DMA-inactive, allocated descriptor storage. */
void rtlmq_reset_rx_ring_posting(struct rtlmq_rx_ring *ring);
int rtlmq_init_rx_ring(struct rtlmq_rx_ring *ring);
int rtlmq_validate_active_rx_rings_for_restart_locked(struct rtlmq_private *tp);
int rtlmq_restart_active_rx_rings_from_zero_locked(struct rtlmq_private *tp);
struct rtlmq_rx_ring *rtlmq_prepare_candidate_rx_ring(struct rtlmq_private *tp,
						      u16 queue_index);
const struct rtlmq_rx_ring *
rtlmq_candidate_or_active_rx_ring(const struct rtlmq_private *tp,
				  u16 queue_index);
bool rtlmq_active_rx_cold_heap_present(const struct rtlmq_private *tp);
bool rtlmq_rx_ring_storage_aliases_active(const struct rtlmq_private *tp,
					  const struct rtlmq_rx_ring *ring);
void rtlmq_commit_candidate_rx_rings(struct rtlmq_private *tp);
void rtlmq_discard_candidate_rx_rings(struct rtlmq_private *tp);
struct rtlmq_live_rx_ring_set *rtlmq_alloc_live_rx_ring_set(void);
int rtlmq_swap_live_candidate_rx_rings(struct rtlmq_private *tp,
				       struct rtlmq_live_rx_ring_set *retired);
void rtlmq_rollback_live_rx_rings(struct rtlmq_private *tp,
				  struct rtlmq_live_rx_ring_set *retired);
bool rtlmq_release_live_rx_ring_set(struct rtlmq_private *tp,
				    struct rtlmq_live_rx_ring_set *retired);
void rtlmq_release_active_rx_cold(struct rtlmq_private *tp);
void rtlmq_stop_rx_ring(struct rtlmq_rx_ring *ring);
void rtlmq_free_rx_ring(struct rtlmq_rx_ring *ring);
struct rtlmq_rx_ring_replacement *
rtlmq_prepare_xsk_rx_ring_replacement(struct rtlmq_private *tp, u16 queue_index,
				      struct xsk_buff_pool *new_pool);
int rtlmq_align_xsk_rx_ring_replacement_locked(struct rtlmq_private *tp,
					       struct rtlmq_rx_ring_replacement *replacement);
void rtlmq_commit_xsk_rx_ring_replacement(struct rtlmq_private *tp,
					  struct rtlmq_rx_ring_replacement *replacement);
void rtlmq_finish_xsk_rx_ring_replacement(struct rtlmq_private *tp,
					  struct rtlmq_rx_ring_replacement *replacement);
void rtlmq_discard_xsk_rx_ring_replacement(struct rtlmq_private *tp,
					   struct rtlmq_rx_ring_replacement *replacement);
int rtlmq_alloc_tx_ring(struct rtlmq_tx_ring *ring);
void rtlmq_refresh_tx_ring_slot(struct rtlmq_private *tp,
				struct rtlmq_tx_ring *ring,
				unsigned int queue_index);
int rtlmq_init_tx_ring(struct rtlmq_tx_ring *ring);
/* Caller owns candidate state and bounds queue_index to implementation slots. */
bool rtlmq_candidate_tx_storage_present(const struct rtlmq_private *tp,
					unsigned int queue_index);
int rtlmq_validate_candidate_tx_storage(const struct rtlmq_private *tp,
					u16 num_tx_queues);
void rtlmq_commit_candidate_tx_resources(struct rtlmq_private *tp);
void rtlmq_discard_candidate_tx_resources(struct rtlmq_private *tp);
struct rtlmq_live_tx_ring_set *rtlmq_alloc_live_tx_ring_set(void);
int rtlmq_stop_active_tx_rings_for_live_swap(struct rtlmq_private *tp,
					     const struct rtlmq_live_service_pause *pause);
int rtlmq_swap_live_candidate_tx_rings(struct rtlmq_private *tp,
				       struct rtlmq_live_tx_ring_set *retired);
void rtlmq_rollback_live_tx_rings(struct rtlmq_private *tp,
				  struct rtlmq_live_tx_ring_set *retired);
bool rtlmq_release_live_tx_ring_set(struct rtlmq_private *tp,
				    struct rtlmq_live_tx_ring_set *retired);
struct rtlmq_tx_desc *
rtlmq_candidate_or_active_tx_desc(const struct rtlmq_private *tp,
				  u16 queue_index);
dma_addr_t rtlmq_candidate_or_active_tx_desc_dma(const struct rtlmq_private *tp,
						 u16 queue_index);
void rtlmq_stop_tx_ring(struct rtlmq_tx_ring *ring);
void rtlmq_free_tx_ring(struct rtlmq_tx_ring *ring);
void rtlmq_clear_pending_tx_doorbells(struct rtlmq_private *tp);
int rtlmq_sync_rx_desc_format(struct rtlmq_private *tp);
int rtlmq_alloc_rings(struct rtlmq_private *tp);
int rtlmq_init_rings(struct rtlmq_private *tp);
u64 rtlmq_derive_active_rx_dma_ring_mask(const struct rtlmq_private *tp);
u64 rtlmq_derive_active_tx_dma_ring_mask(const struct rtlmq_private *tp);
void rtlmq_stop_rings(struct rtlmq_private *tp);
void rtlmq_free_rings(struct rtlmq_private *tp);

/*
 * RTL8125 descriptors and descriptor-base register pairs hold 64-bit addresses.
 * dma_addr_t is the DMA API's platform address type, so keep it intact when
 * building descriptors or programming base registers.
 */
static_assert(sizeof(dma_addr_t) <= sizeof(u64));
static_assert(sizeof(size_t) <= sizeof(u64));

/*
 * Check address offsets before using them for DMA. A wrapped page-pool or XDP
 * address must never reach the device-range check or a descriptor.
 */
static inline bool rtlmq_dma_addr_add_checked(dma_addr_t base, u64 offset,
					      dma_addr_t *result)
{
	if (!result || check_add_overflow(base, offset, result))
		return false;

	return true;
}

/*
 * dma_alloc_coherent(), dma_map_*, page-pool, and XSK mappings all use the
 * device DMA mask. Use this helper before writing a descriptor or base
 * register to make sure the whole range is addressable.
 */
static inline bool rtlmq_dma_range_fits_mask(u64 mask, dma_addr_t addr,
					     size_t len)
{
	u64 end;

	if (!len)
		return false;
	if (check_add_overflow((u64)addr, (u64)len - 1, &end))
		return false;

	return end <= mask;
}

static inline bool rtlmq_dma_range_fits_device(struct device *dev,
					       dma_addr_t addr, size_t len)
{
	return dev && rtlmq_dma_range_fits_mask(dma_get_mask(dev), addr, len);
}

static inline bool rtlmq_dma_range_fits_coherent(struct device *dev,
						 dma_addr_t addr, size_t len)
{
	u64 mask;

	if (!dev)
		return false;

	mask = dev->coherent_dma_mask ?: DMA_BIT_MASK(32);
	return rtlmq_dma_range_fits_mask(mask, addr, len);
}

#endif /* __RTLMQ_RING_H__ */
