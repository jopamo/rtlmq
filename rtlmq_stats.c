// SPDX-License-Identifier: GPL-2.0-only

#include <linux/ethtool.h>
#include <linux/dma-mapping.h>
#include <linux/iopoll.h>
#include <linux/netdevice.h>
#include <linux/pci.h>
#include <linux/rcupdate.h>
#include <net/netdev_queues.h>

#include "rtlmq_desc.h"
#include "rtlmq_diag.h"
#include "rtlmq_hw.h"
#include "rtlmq_runtime_accessors.h"
#include "rtlmq_irq.h"
#include "rtlmq_regs.h"
#include "rtlmq_runtime.h"
#include "rtlmq.h"
#include "rtlmq_ring.h"
#include "rtlmq_rx.h"
#include "rtlmq_tx.h"
#include "rtlmq_tx_private.h"

static enum rtlmq_tally_sample_state
rtlmq_get_tally_snapshot_locked(struct rtlmq_private *tp, u64 values[RTLMQ_TALLY_STAT_COUNT]);

static bool
rtlmq_get_committed_tally_snapshot(struct rtlmq_private *tp,
				   u64 values[RTLMQ_TALLY_STAT_COUNT])
{
	bool valid;

	mutex_lock(&tp->control_lock);
	rtlmq_get_tally_snapshot_locked(tp, values);
	valid = tp->tally.total_valid;
	mutex_unlock(&tp->control_lock);

	return valid;
}

static enum rtlmq_tally_sample_state
rtlmq_tally_sample_result(struct rtlmq_private *tp, int ret, bool valid)
{
	if (!ret)
		return RTLMQ_TALLY_SAMPLE_FRESH;
	if (ret == -EAGAIN ||
	    (ret == -ENODEV && !rtlmq_mmio_ordinary_authorized(tp)))
		return valid ? RTLMQ_TALLY_SAMPLE_RETAINED :
			       RTLMQ_TALLY_SAMPLE_UNAVAILABLE;
	return RTLMQ_TALLY_SAMPLE_READ_FAILED;
}

static void
rtlmq_copy_queue_stats_fields(struct rtlmq_queue_stats *destination,
			      const struct rtlmq_queue_stats *source)
{
	destination->rx_packets = READ_ONCE(source->rx_packets);
	destination->rx_bytes = READ_ONCE(source->rx_bytes);
	destination->rx_drops = READ_ONCE(source->rx_drops);
	for (unsigned int i = 0; i < RTLMQ_RX_DROP_COUNT; i++)
		destination->rx_drop_reason[i] =
			READ_ONCE(source->rx_drop_reason[i]);
	destination->rx_errors = READ_ONCE(source->rx_errors);
	destination->rx_crc_errors = READ_ONCE(source->rx_crc_errors);
	destination->rx_length_errors = READ_ONCE(source->rx_length_errors);
	destination->rx_alloc_fail = READ_ONCE(source->rx_alloc_fail);
	destination->rx_csum_unnecessary =
		READ_ONCE(source->rx_csum_unnecessary);
	destination->rx_csum_bad = READ_ONCE(source->rx_csum_bad);
	destination->tx_packets = READ_ONCE(source->tx_packets);
	destination->tx_bytes = READ_ONCE(source->tx_bytes);
	destination->tx_drops = READ_ONCE(source->tx_drops);
	destination->tx_hw_gso_packets =
		READ_ONCE(source->tx_hw_gso_packets);
	destination->tx_stop = READ_ONCE(source->tx_stop);
	destination->tx_wake = READ_ONCE(source->tx_wake);
}

static void rtlmq_copy_retired_queue_stats(const struct rtlmq_private *tp,
					   struct rtlmq_queue_stats *stats)
{
	unsigned int start;

	memset(stats, 0, sizeof(*stats));
	do {
		start = u64_stats_fetch_begin(&tp->queue_stats_syncp);
		rtlmq_copy_queue_stats_fields(stats, &tp->queue_stats_history);
	} while (u64_stats_fetch_retry(&tp->queue_stats_syncp, start));
}

struct rtlmq_rx_queue_stats_snapshot {
	u64 drop_reason[RTLMQ_RX_DROP_COUNT];
	u64 packets;
	u64 bytes;
	u64 drops;
	u64 errors;
	u64 crc_errors;
	u64 length_errors;
	u64 alloc_fail;
	u64 csum_unnecessary;
	u64 csum_bad;
};

struct rtlmq_tx_queue_stats_snapshot {
	u64 packets;
	u64 bytes;
	u64 hw_gso_packets;
	u64 stop;
	u64 wake;
};

static void
rtlmq_snapshot_rx_queue_stats(const struct rtlmq_rx_ring *ring,
			      const struct rtlmq_rx_ring_cold *cold,
			      struct rtlmq_rx_queue_stats_snapshot *snapshot)
{
	unsigned int start;

	if (!ring || !cold || !snapshot)
		return;

	do {
		start = rtlmq_rx_stats_fetch_begin(ring);
		snapshot->packets = READ_ONCE(ring->hw_packets);
		snapshot->bytes = READ_ONCE(ring->hw_bytes);
		snapshot->drops = READ_ONCE(cold->stats.drops);
		for (unsigned int i = 0; i < RTLMQ_RX_DROP_COUNT; i++)
			snapshot->drop_reason[i] =
				READ_ONCE(cold->stats.drop_reason[i]);
		snapshot->errors = READ_ONCE(cold->stats.rx_errors);
		snapshot->crc_errors = READ_ONCE(cold->stats.rx_crc_errors);
		snapshot->length_errors =
			READ_ONCE(cold->stats.rx_length_errors);
		snapshot->alloc_fail = READ_ONCE(cold->stats.alloc_fail);
		snapshot->csum_unnecessary = READ_ONCE(cold->stats.csum_ok);
		snapshot->csum_bad = READ_ONCE(cold->stats.csum_bad);
	} while (rtlmq_rx_stats_fetch_retry(ring, start));
}

/*
 * TX completions and producer-side counters have distinct writers and
 * sequence counters. Drops are not part of netdev_queue_stats_tx, so skip
 * that read for per-queue callers while retaining one producer snapshot.
 * Queue-state counters have both writers and are read independently.
 */
static void
rtlmq_snapshot_tx_queue_stats(const struct rtlmq_tx_ring *ring,
			      struct rtlmq_tx_queue_stats_snapshot *snapshot,
			      u64 *drops)
{
	unsigned int start;

	do {
		start = u64_stats_fetch_begin(&ring->comp_syncp);
		snapshot->packets = READ_ONCE(ring->stats.packets);
		snapshot->bytes = READ_ONCE(ring->stats.bytes);
	} while (u64_stats_fetch_retry(&ring->comp_syncp, start));

	do {
		start = u64_stats_fetch_begin(&ring->syncp);
		if (drops)
			*drops = READ_ONCE(ring->stats.drops);
		snapshot->hw_gso_packets =
			READ_ONCE(ring->stats.xmit_tso_pkts);
	} while (u64_stats_fetch_retry(&ring->syncp, start));

	snapshot->stop = atomic64_read(&ring->stats.queue_stopped);
	snapshot->wake = atomic64_read(&ring->stats.queue_wake);
}

/*
 * Add active queue counters to device history after hiding the runtime and
 * stopping NAPI/IRQs. The generation token prevents double counting across
 * repeated stop/free or rollback calls.
 */
void rtlmq_retire_queue_stats(struct rtlmq_private *tp)
{
	u32 generation;
	unsigned int i;
	u64 rx_mask;
	u64 tx_mask;

	if (!tp || !tp->rx_rings || !tp->tx_rings)
		return;

	generation = READ_ONCE(tp->active_resource_generation);
	rx_mask = rtlmq_derive_active_rx_dma_ring_mask(tp);
	tx_mask = rtlmq_derive_active_tx_dma_ring_mask(tp);
	if (!generation || (!rx_mask && !tx_mask) ||
	    READ_ONCE(tp->queue_stats_retired_generation) == generation)
		return;

	/*
	 * This is the only writer of queue_stats_history. Ring-local sequence
	 * writes below are safe here because lifecycle teardown has already
	 * stopped every producer and consumer of the retiring generation.
	 */
	u64_stats_update_begin(&tp->queue_stats_syncp);
	for (i = 0; i < RTLMQ_IMPL_MAX_RX_QUEUES; i++) {
		struct rtlmq_rx_queue_stats_snapshot snapshot;
		struct rtlmq_rx_ring_cold *cold;
		struct rtlmq_rx_ring *ring;

		if (!(rx_mask & BIT_ULL(i)))
			continue;

		ring = &tp->rx_rings[i];
		cold = READ_ONCE(ring->cold);
		if (!cold)
			continue;

		rtlmq_snapshot_rx_queue_stats(ring, cold, &snapshot);

		tp->queue_stats_history.rx_packets += snapshot.packets;
		tp->queue_stats_history.rx_bytes += snapshot.bytes;
		tp->queue_stats_history.rx_drops += snapshot.drops;
		for (unsigned int j = 0; j < RTLMQ_RX_DROP_COUNT; j++)
			tp->queue_stats_history.rx_drop_reason[j] +=
				snapshot.drop_reason[j];
		tp->queue_stats_history.rx_errors += snapshot.errors;
		tp->queue_stats_history.rx_crc_errors += snapshot.crc_errors;
		tp->queue_stats_history.rx_length_errors +=
			snapshot.length_errors;
		tp->queue_stats_history.rx_alloc_fail += snapshot.alloc_fail;
		tp->queue_stats_history.rx_csum_unnecessary +=
			snapshot.csum_unnecessary;
		tp->queue_stats_history.rx_csum_bad += snapshot.csum_bad;

		rtlmq_rx_stats_update_begin(ring);
		WRITE_ONCE(ring->hw_packets, 0);
		WRITE_ONCE(ring->hw_bytes, 0);
		WRITE_ONCE(cold->stats.drops, 0);
		for (unsigned int j = 0; j < RTLMQ_RX_DROP_COUNT; j++)
			WRITE_ONCE(cold->stats.drop_reason[j], 0);
		WRITE_ONCE(cold->stats.rx_errors, 0);
		WRITE_ONCE(cold->stats.rx_crc_errors, 0);
		WRITE_ONCE(cold->stats.rx_length_errors, 0);
		WRITE_ONCE(cold->stats.alloc_fail, 0);
		WRITE_ONCE(cold->stats.csum_ok, 0);
		WRITE_ONCE(cold->stats.csum_bad, 0);
		rtlmq_rx_stats_update_end(ring);
	}

	for (i = 0; i < RTLMQ_IMPL_MAX_TX_QUEUES; i++) {
		struct rtlmq_tx_queue_stats_snapshot snapshot;
		struct rtlmq_tx_ring *ring;
		u64 drops;

		if (!(tx_mask & BIT_ULL(i)))
			continue;

		ring = &tp->tx_rings[i];
		rtlmq_snapshot_tx_queue_stats(ring, &snapshot, &drops);

		tp->queue_stats_history.tx_packets += snapshot.packets;
		tp->queue_stats_history.tx_bytes += snapshot.bytes;
		tp->queue_stats_history.tx_drops += drops;
		tp->queue_stats_history.tx_hw_gso_packets +=
			snapshot.hw_gso_packets;
		tp->queue_stats_history.tx_stop += snapshot.stop;
		tp->queue_stats_history.tx_wake += snapshot.wake;

		u64_stats_update_begin(&ring->comp_syncp);
		WRITE_ONCE(ring->stats.packets, 0);
		WRITE_ONCE(ring->stats.bytes, 0);
		u64_stats_update_end(&ring->comp_syncp);
		u64_stats_update_begin(&ring->syncp);
		WRITE_ONCE(ring->stats.drops, 0);
		WRITE_ONCE(ring->stats.xmit_tso_pkts, 0);
		u64_stats_update_end(&ring->syncp);
		atomic64_set(&ring->stats.queue_stopped, 0);
		atomic64_set(&ring->stats.queue_wake, 0);
	}
	WRITE_ONCE(tp->queue_stats_retired_generation, generation);
	u64_stats_update_end(&tp->queue_stats_syncp);
}

void rtlmq_get_queue_stats_totals(const struct rtlmq_private *tp,
				  struct rtlmq_queue_stats *stats)
{
	const struct rtlmq_runtime_config *published_cfg;
	const struct rtlmq_rx_ring *rx_rings;
	const struct rtlmq_tx_ring *tx_rings;
	unsigned int global_start;
	unsigned int i;
	u16 rx_queues;
	u16 tx_queues;

	if (!stats)
		return;
	memset(stats, 0, sizeof(*stats));
	if (!tp)
		return;

	/*
	 * Pin one published generation for the whole read. Publication is
	 * withdrawn before ring teardown, and the teardown path waits for this
	 * read-side section before retiring ring-local storage.
	 */
	rcu_read_lock();
	published_cfg = rtlmq_published_runtime_config(tp);
	rx_rings = READ_ONCE(tp->rx_rings);
	tx_rings = READ_ONCE(tp->tx_rings);
	rx_queues = published_cfg ? published_cfg->num_rx_queues : 0;
	tx_queues = published_cfg ? published_cfg->num_tx_queues : 0;

	do {
		const struct rtlmq_queue_stats *history =
			&tp->queue_stats_history;

		global_start = u64_stats_fetch_begin(&tp->queue_stats_syncp);
		rtlmq_copy_queue_stats_fields(stats, history);

		if (rx_rings) {
			for (i = 0; i < rx_queues; i++) {
				const struct rtlmq_rx_ring *ring = &rx_rings[i];
				const struct rtlmq_rx_ring_cold *cold =
					READ_ONCE(ring->cold);
				struct rtlmq_rx_queue_stats_snapshot snapshot;

				if (!cold)
					continue;

				rtlmq_snapshot_rx_queue_stats(ring, cold, &snapshot);

				stats->rx_packets += snapshot.packets;
				stats->rx_bytes += snapshot.bytes;
				stats->rx_drops += snapshot.drops;
				for (unsigned int j = 0; j < RTLMQ_RX_DROP_COUNT; j++)
					stats->rx_drop_reason[j] += snapshot.drop_reason[j];
				stats->rx_errors += snapshot.errors;
				stats->rx_crc_errors += snapshot.crc_errors;
				stats->rx_length_errors += snapshot.length_errors;
				stats->rx_alloc_fail += snapshot.alloc_fail;
				stats->rx_csum_unnecessary +=
					snapshot.csum_unnecessary;
				stats->rx_csum_bad += snapshot.csum_bad;
			}
		}

		if (tx_rings) {
			for (i = 0; i < tx_queues; i++) {
				const struct rtlmq_tx_ring *ring = &tx_rings[i];
				struct rtlmq_tx_queue_stats_snapshot snapshot;
				u64 drops;

				rtlmq_snapshot_tx_queue_stats(ring, &snapshot,
							      &drops);

				stats->tx_packets += snapshot.packets;
				stats->tx_bytes += snapshot.bytes;
				stats->tx_drops += drops;
				stats->tx_hw_gso_packets +=
					snapshot.hw_gso_packets;
				stats->tx_stop += snapshot.stop;
				stats->tx_wake += snapshot.wake;
			}
		}
	} while (u64_stats_fetch_retry(&tp->queue_stats_syncp,
				      global_start));
	rcu_read_unlock();
}

static void rtlmq_get_queue_stats_rx(struct net_device *ndev, int index,
				     struct netdev_queue_stats_rx *stats)
{
	const struct rtlmq_runtime_config *published_cfg;
	struct rtlmq_private *tp = netdev_priv(ndev);
	const struct rtlmq_rx_ring_cold *cold;
	const struct rtlmq_rx_ring *rx_rings;
	const struct rtlmq_rx_ring *ring;
	unsigned int start;
	u64 alloc_fail;

	if (!tp)
		return;

	rcu_read_lock();
	published_cfg = rtlmq_published_runtime_config(tp);
	rx_rings = READ_ONCE(tp->rx_rings);
	if (!published_cfg || index < 0 ||
	    index >= published_cfg->num_rx_queues || !rx_rings)
		goto out_unlock;

	ring = &rx_rings[index];
	cold = READ_ONCE(ring->cold);
	if (!cold)
		goto out_unlock;

	do {
		start = rtlmq_rx_stats_fetch_begin(ring);
		stats->packets = READ_ONCE(ring->hw_packets);
		stats->bytes = READ_ONCE(ring->hw_bytes);
		alloc_fail = READ_ONCE(cold->stats.alloc_fail);
		stats->csum_unnecessary = READ_ONCE(cold->stats.csum_ok);
		stats->csum_bad = READ_ONCE(cold->stats.csum_bad);
	} while (rtlmq_rx_stats_fetch_retry(ring, start));

	stats->alloc_fail = alloc_fail;
out_unlock:
	rcu_read_unlock();
}

static void rtlmq_get_queue_stats_tx(struct net_device *ndev, int index,
				     struct netdev_queue_stats_tx *stats)
{
	const struct rtlmq_runtime_config *published_cfg;
	struct rtlmq_tx_queue_stats_snapshot snapshot;
	struct rtlmq_private *tp = netdev_priv(ndev);
	const struct rtlmq_tx_ring *tx_rings;
	const struct rtlmq_tx_ring *ring;

	if (!tp)
		return;

	rcu_read_lock();
	published_cfg = rtlmq_published_runtime_config(tp);
	tx_rings = READ_ONCE(tp->tx_rings);
	if (!published_cfg || index < 0 ||
	    index >= published_cfg->num_tx_queues || !tx_rings)
		goto out_unlock;

	ring = &tx_rings[index];
	rtlmq_snapshot_tx_queue_stats(ring, &snapshot, NULL);
	stats->packets = snapshot.packets;
	stats->bytes = snapshot.bytes;
	stats->hw_gso_packets = snapshot.hw_gso_packets;
	stats->stop = snapshot.stop;
	stats->wake = snapshot.wake;
out_unlock:
	rcu_read_unlock();
}

static void rtlmq_get_base_stats(struct net_device *ndev,
				 struct netdev_queue_stats_rx *rx,
				 struct netdev_queue_stats_tx *tx)
{
	struct rtlmq_private *tp = netdev_priv(ndev);
	struct rtlmq_queue_stats history;

	rtlmq_copy_retired_queue_stats(tp, &history);

	rx->packets = history.rx_packets;
	rx->bytes = history.rx_bytes;
	rx->alloc_fail = history.rx_alloc_fail;
	rx->csum_unnecessary = history.rx_csum_unnecessary;
	rx->csum_bad = history.rx_csum_bad;

	tx->packets = history.tx_packets;
	tx->bytes = history.tx_bytes;
	tx->hw_gso_packets = history.tx_hw_gso_packets;
	tx->stop = history.tx_stop;
	tx->wake = history.tx_wake;
}

const struct netdev_stat_ops rtlmq_stat_ops = {
	.get_queue_stats_rx	= rtlmq_get_queue_stats_rx,
	.get_queue_stats_tx	= rtlmq_get_queue_stats_tx,
	.get_base_stats		= rtlmq_get_base_stats,
};

/* Append in every build; preserve the existing ethtool statistic ordering. */
static const char rtlmq_rx_drop_names[RTLMQ_RX_DROP_COUNT][ETH_GSTRING_LEN] = {
	[RTLMQ_RX_DROP_STATUS] = "driver_rx_drop_status",
	[RTLMQ_RX_DROP_FRAGMENT] = "driver_rx_drop_fragment",
	[RTLMQ_RX_DROP_LENGTH] = "driver_rx_drop_length",
	[RTLMQ_RX_DROP_SKB] = "driver_rx_drop_skb",
	[RTLMQ_RX_DROP_RESOURCE] = "driver_rx_drop_resource",
	[RTLMQ_RX_DROP_XDP] = "driver_rx_drop_xdp",
};

static void rtlmq_ethtool_put_rx_drop_strings(u8 **data)
{
	unsigned int i;

	ethtool_puts(data, "driver_rx_drops");
	for (i = 0; i < RTLMQ_RX_DROP_COUNT; i++)
		ethtool_puts(data, rtlmq_rx_drop_names[i]);
}

static void rtlmq_ethtool_put_rx_drop_values(const struct rtlmq_private *tp,
					     u64 **data)
{
	struct rtlmq_queue_stats stats;
	unsigned int i;

	rtlmq_get_queue_stats_totals(tp, &stats);
	*(*data)++ = stats.rx_drops;
	for (i = 0; i < RTLMQ_RX_DROP_COUNT; i++)
		*(*data)++ = stats.rx_drop_reason[i];
}

/*
 * Append committed ring geometry at a fixed width so the ethtool string set
 * remains stable across open, close, and runtime rebuilds. Values come only
 * from published active rings under control_lock.
 */
#define RTLMQ_RX_GEOMETRY_STAT_LIST(X) \
	X(geometry_published, published) \
	X(geometry_valid, valid) \
	X(ring_count, ring_count) \
	X(alloc_size, allocation_size) \
	X(dma_span, dma_span) \
	X(memory_model, memory_model) \
	X(page_order, page_order) \
	X(page_pool_size, page_pool_size) \
	X(page_pool_dma_span, page_pool_dma_span)

static const char rtlmq_rx_geometry_stat_names[][ETH_GSTRING_LEN] = {
#define RTLMQ_RX_GEOMETRY_STAT_NAME(_name, _member) { #_name },
	RTLMQ_RX_GEOMETRY_STAT_LIST(RTLMQ_RX_GEOMETRY_STAT_NAME)
#undef RTLMQ_RX_GEOMETRY_STAT_NAME
};

#define RTLMQ_RX_GEOMETRY_TOTAL \
	(RTLMQ_IMPL_MAX_RX_QUEUES * ARRAY_SIZE(rtlmq_rx_geometry_stat_names))

static_assert(sizeof("rx_queue_0_geometry_published") <= ETH_GSTRING_LEN);
static_assert(sizeof("rx_queue_0_page_pool_dma_span") <= ETH_GSTRING_LEN);

static void rtlmq_ethtool_put_rx_geometry_strings(u8 **data)
{
	unsigned int i, j;

	for (i = 0; i < RTLMQ_IMPL_MAX_RX_QUEUES; i++)
		for (j = 0; j < ARRAY_SIZE(rtlmq_rx_geometry_stat_names); j++)
			ethtool_sprintf(data, "rx_queue_%u_%s", i,
					rtlmq_rx_geometry_stat_names[j]);
}

static void
rtlmq_ethtool_put_rx_geometry_values_locked(const struct rtlmq_private *tp,
					    u64 **data)
{
	unsigned int i;

	lockdep_assert_held(&tp->control_lock);
	for (i = 0; i < RTLMQ_IMPL_MAX_RX_QUEUES; i++) {
		struct rtlmq_rx_published_geometry geometry;

		rtlmq_snapshot_published_rx_geometry_locked(tp, i, &geometry);
#define RTLMQ_PUT_RX_GEOMETRY_VALUE(_name, _member) \
		*(*data)++ = geometry._member;
		RTLMQ_RX_GEOMETRY_STAT_LIST(RTLMQ_PUT_RX_GEOMETRY_VALUE);
#undef RTLMQ_PUT_RX_GEOMETRY_VALUE
	}
}

static unsigned int rtlmq_num_ethtool_stats(const struct rtlmq_private *tp)
{
	(void)tp;

	return RTLMQ_TALLY_STAT_COUNT +
	       RTLMQ_RX_GEOMETRY_TOTAL + RTLMQ_RX_DROP_COUNT + 3;
}

int rtlmq_ethtool_get_sset_count(struct net_device *ndev, int sset)
{
	struct rtlmq_private *tp = netdev_priv(ndev);
	int count;

	switch (sset) {
	case ETH_SS_STATS:
		mutex_lock(&tp->control_lock);
		count = rtlmq_num_ethtool_stats(tp);
		mutex_unlock(&tp->control_lock);
		return count;
	default:
		return -EOPNOTSUPP;
	}
}

u32 rtlmq_ethtool_get_rx_ring_count(struct net_device *ndev)
{
	struct rtlmq_private *tp = netdev_priv(ndev);
	const struct rtlmq_runtime_config *cfg;
	u32 count;

	mutex_lock(&tp->control_lock);
	cfg = rtlmq_active_or_requested_runtime_config(tp);
	count = cfg ? cfg->num_rx_queues : 0;
	mutex_unlock(&tp->control_lock);

	return count;
}

void rtlmq_ethtool_get_strings(struct net_device *ndev, u32 stringset, u8 *data)
{
	(void)ndev;

	if (stringset != ETH_SS_STATS)
		return;

#define RTLMQ_TALLY_PUT_STRING(_id, _name, _member, _bits) \
	ethtool_puts(&data, _name);
	RTLMQ_TALLY_STAT_LIST(RTLMQ_TALLY_PUT_STRING)
#undef RTLMQ_TALLY_PUT_STRING
	rtlmq_ethtool_put_rx_geometry_strings(&data);
	rtlmq_ethtool_put_rx_drop_strings(&data);
	ethtool_puts(&data, "hw_tally_sample_state");
	ethtool_puts(&data, "hw_tally_command_failures");
}

void rtlmq_ethtool_get_stats(struct net_device *ndev,
			     struct ethtool_stats *stats, u64 *data)
{
	struct rtlmq_private *tp = netdev_priv(ndev);
	enum rtlmq_tally_sample_state sample_state;
	u64 values[RTLMQ_TALLY_STAT_COUNT];

	(void)stats;

	mutex_lock(&tp->control_lock);
	sample_state = rtlmq_get_tally_snapshot_locked(tp, values);
	memcpy(data, values, sizeof(values));
	data += RTLMQ_TALLY_STAT_COUNT;
	rtlmq_ethtool_put_rx_geometry_values_locked(tp, &data);
	rtlmq_ethtool_put_rx_drop_values(tp, &data);
	*data++ = sample_state;
	*data = tp->tally.command_failures;
	mutex_unlock(&tp->control_lock);
}

void rtlmq_ethtool_get_pause_stats(struct net_device *ndev,
				   struct ethtool_pause_stats *pause_stats)
{
	struct rtlmq_private *tp = netdev_priv(ndev);
	u64 values[RTLMQ_TALLY_STAT_COUNT];

	if (!tp || !pause_stats)
		return;

	if (!rtlmq_get_committed_tally_snapshot(tp, values))
		return;

	pause_stats->tx_pause_frames = values[RTLMQ_TALLY_TX_PAUSE_ON];
	pause_stats->rx_pause_frames = values[RTLMQ_TALLY_RX_PAUSE_ON];
}

void rtlmq_ethtool_get_eth_mac_stats(struct net_device *ndev,
				     struct ethtool_eth_mac_stats *mac_stats)
{
	struct rtlmq_private *tp = netdev_priv(ndev);
	u64 values[RTLMQ_TALLY_STAT_COUNT];

	if (!tp || !mac_stats)
		return;

	if (!rtlmq_get_committed_tally_snapshot(tp, values))
		return;

	mac_stats->FramesTransmittedOK = values[RTLMQ_TALLY_TX_PACKETS];
	mac_stats->SingleCollisionFrames =
		values[RTLMQ_TALLY_TX_ONE_COLLISION];
	mac_stats->MultipleCollisionFrames =
		values[RTLMQ_TALLY_TX_MULTI_COLLISION];
	mac_stats->FramesReceivedOK = values[RTLMQ_TALLY_RX_PACKETS];
	mac_stats->AlignmentErrors = values[RTLMQ_TALLY_ALIGN_ERRORS];
	mac_stats->OctetsTransmittedOK = values[RTLMQ_TALLY_TX_OCTETS];
	mac_stats->FramesWithDeferredXmissions =
		values[RTLMQ_TALLY_TX_DEFERRED];
	mac_stats->LateCollisions = values[RTLMQ_TALLY_TX_LATE_COLLISION];
	mac_stats->FramesAbortedDueToXSColls =
		values[RTLMQ_TALLY_TX_ABORTED];
	mac_stats->FramesLostDueToIntMACXmitError =
		values[RTLMQ_TALLY_TX_ERRORS];
	mac_stats->OctetsReceivedOK = values[RTLMQ_TALLY_RX_OCTETS];
	mac_stats->FramesLostDueToIntMACRcvError =
		values[RTLMQ_TALLY_RX_MAC_ERROR];
	mac_stats->MulticastFramesXmittedOK =
		values[RTLMQ_TALLY_TX_MULTICAST];
	mac_stats->BroadcastFramesXmittedOK =
		values[RTLMQ_TALLY_TX_BROADCAST];
	mac_stats->MulticastFramesReceivedOK =
		values[RTLMQ_TALLY_RX_MULTICAST];
	mac_stats->BroadcastFramesReceivedOK =
		values[RTLMQ_TALLY_RX_BROADCAST];
	mac_stats->FrameTooLongErrors = values[RTLMQ_TALLY_RX_TOO_LONG];
}

void rtlmq_ethtool_get_eth_ctrl_stats(struct net_device *ndev,
				      struct ethtool_eth_ctrl_stats *ctrl_stats)
{
	struct rtlmq_private *tp = netdev_priv(ndev);
	u64 values[RTLMQ_TALLY_STAT_COUNT];

	if (!tp || !ctrl_stats)
		return;

	if (!rtlmq_get_committed_tally_snapshot(tp, values))
		return;

	ctrl_stats->UnsupportedOpcodesReceived =
		values[RTLMQ_TALLY_RX_UNKNOWN_OPCODE];
}

void rtlmq_ethtool_get_rmon_stats(struct net_device *ndev,
				  struct ethtool_rmon_stats *rmon_stats,
				  const struct ethtool_rmon_hist_range **ranges)
{
	struct rtlmq_private *tp = netdev_priv(ndev);
	u64 values[RTLMQ_TALLY_STAT_COUNT];

	if (!tp || !rmon_stats)
		return;

	if (ranges)
		*ranges = NULL;
	if (!rtlmq_get_committed_tally_snapshot(tp, values))
		return;

	rmon_stats->undersize_pkts = values[RTLMQ_TALLY_RX_RUNT];
	rmon_stats->oversize_pkts = values[RTLMQ_TALLY_RX_TOO_LONG];
}

/* Hardware tally allocation, sampling, and accumulation. */

int rtlmq_alloc_counter_memory(struct rtlmq_private *tp)
{
	if (!tp || !tp->pdev)
		return -EINVAL;

	/* No DMA scratch or tally commands for an unqualified register layout. */
	if (!rtlmq_qualified_supports_hw_tally(tp))
		return 0;

	if (tp->counters)
		return 0;

	tp->counters = dma_alloc_coherent(&tp->pdev->dev,
					  sizeof(*tp->counters),
					  &tp->counters_phys_addr,
					  GFP_KERNEL);
	if (!tp->counters)
		return -ENOMEM;

	return 0;
}

static int rtlmq_do_counters(struct rtlmq_private *tp, u32 counter_cmd)
{
	u32 counter_addr;
	u32 cmd;
	int ret;

	if (!tp || !rtlmq_mmio_ordinary_authorized(tp) || !tp->counters)
		return -ENODEV;
	if (!rtlmq_qualified_supports_hw_tally(tp))
		return -EOPNOTSUPP;

	/* A failed command can leave a partially updated DMA image. */
	tp->tally.raw_valid = false;
	cmd = lower_32_bits(tp->counters_phys_addr);

	writel(upper_32_bits(tp->counters_phys_addr),
	       tp->mmio_addr + RTLMQ_COUNTER_ADDR_HIGH);
	readl(tp->mmio_addr + RTLMQ_COUNTER_ADDR_HIGH);
	writel(cmd, tp->mmio_addr + RTLMQ_COUNTER_ADDR_LOW);
	writel(cmd | counter_cmd, tp->mmio_addr + RTLMQ_COUNTER_ADDR_LOW);

	ret = readl_poll_timeout_atomic(tp->mmio_addr + RTLMQ_COUNTER_ADDR_LOW, counter_addr,
					!(counter_addr &
					  (RTLMQ_COUNTER_RESET | RTLMQ_COUNTER_DUMP)),
					10, 1000);
	if (ret) {
		tp->tally.command_failures++;
		dev_warn_ratelimited(&tp->pdev->dev,
				     "%s: counter command %#x timed out; retaining last committed statistics\n",
				     tp->ndev->name, counter_cmd);
		return ret;
	}

	return 0;
}

static void rtlmq_capture_tally_raw(const struct rtlmq_counters *counters,
				    u64 values[RTLMQ_TALLY_STAT_COUNT])
{
#define RTLMQ_TALLY_CAPTURE_32(_member) \
	le32_to_cpu(counters->_member)
#define RTLMQ_TALLY_CAPTURE_64(_member) \
	le64_to_cpu(counters->_member)
#define RTLMQ_TALLY_CAPTURE(_id, _name, _member, _bits) \
	values[RTLMQ_TALLY_##_id] = RTLMQ_TALLY_CAPTURE_##_bits(_member);
	RTLMQ_TALLY_STAT_LIST(RTLMQ_TALLY_CAPTURE)
#undef RTLMQ_TALLY_CAPTURE
#undef RTLMQ_TALLY_CAPTURE_64
#undef RTLMQ_TALLY_CAPTURE_32
}

static u64 rtlmq_tally_delta(u64 raw_value, u64 previous, unsigned int width)
{
	if (width == 64)
		return raw_value - previous;

	return (raw_value - previous) & GENMASK_ULL(width - 1, 0);
}

static void rtlmq_accumulate_tally_snapshot(struct rtlmq_private *tp)
{
	u64 raw_values[RTLMQ_TALLY_STAT_COUNT];

	lockdep_assert_held(&tp->control_lock);
	rtlmq_capture_tally_raw(tp->counters, raw_values);
	tp->tally.raw_valid = true;
	tp->tally.total_valid = true;

	if (!tp->tally.previous_valid) {
		memcpy(tp->tally.previous, raw_values, sizeof(raw_values));
		tp->tally.previous_valid = true;
		return;
	}

#define RTLMQ_TALLY_ACCUMULATE(_id, _name, _member, _bits) \
	do { \
		const unsigned int index = RTLMQ_TALLY_##_id; \
		\
		tp->tally.total[index] += \
			rtlmq_tally_delta(raw_values[index], \
					  tp->tally.previous[index], _bits); \
		tp->tally.previous[index] = raw_values[index]; \
	} while (0);
	RTLMQ_TALLY_STAT_LIST(RTLMQ_TALLY_ACCUMULATE)
#undef RTLMQ_TALLY_ACCUMULATE
}

int rtlmq_update_counters(struct rtlmq_private *tp)
{
	u8 chipcmd;
	int ret;

	if (!tp)
		return -ENODEV;

	lockdep_assert_held(&tp->control_lock);
	if (!rtlmq_mmio_ordinary_authorized(tp) || !tp->counters) {
		ret = -ENODEV;
		goto out;
	}
	if (!tp->tally.initialized) {
		ret = -EAGAIN;
		goto out;
	}

	chipcmd = readb(tp->mmio_addr + RTLMQ_CHIP_CMD);
	if (chipcmd == U8_MAX) {
		ret = -ENODEV;
		goto out;
	}
	if (!(chipcmd & RTLMQ_CHIP_CMD_RX_ENABLE)) {
		ret = -EAGAIN;
		goto out;
	}

	ret = rtlmq_do_counters(tp, RTLMQ_COUNTER_DUMP);
	if (ret)
		goto out;

	rtlmq_accumulate_tally_snapshot(tp);
out:
	tp->tally.sample_state =
		rtlmq_tally_sample_result(tp, ret, tp->tally.total_valid);
	return ret;
}

void rtlmq_init_counter_offsets(struct rtlmq_private *tp)
{
	int ret;

	lockdep_assert_held(&tp->control_lock);

	if (!tp->counters)
		return;

	/*
	 * Reset extended tally counters only on first activation. Preserve
	 * cumulative software totals across later MAC resets and runtime
	 * rebuilds. Failed commands must not expose the DMA scratch buffer.
	 */
	if (!tp->tally.initialized) {
		ret = rtlmq_do_counters(tp, RTLMQ_COUNTER_RESET);
		if (ret) {
			tp->tally.sample_state = RTLMQ_TALLY_SAMPLE_READ_FAILED;
			return;
		}
		tp->tally.initialized = true;
		tp->tally.previous_valid = false;
	}

	/*
	 * The caller enables RX before taking this baseline and keeps IRQ/NAPI
	 * disabled until publication. If the preceding reset kept the hardware
	 * counters, the final pre-reset snapshot was already folded into total[].
	 */
	if (!tp->tally.previous_valid)
		(void)rtlmq_update_counters(tp);
}

void rtlmq_tally_reset_epoch(struct rtlmq_private *tp)
{
	if (!tp)
		return;

	lockdep_assert_held(&tp->control_lock);
	tp->tally.previous_valid = false;
	tp->tally.raw_valid = false;
	tp->tally.sample_state = tp->tally.total_valid ?
		RTLMQ_TALLY_SAMPLE_RETAINED : RTLMQ_TALLY_SAMPLE_UNAVAILABLE;
}

static enum rtlmq_tally_sample_state
rtlmq_get_tally_snapshot_locked(struct rtlmq_private *tp,
				u64 values[RTLMQ_TALLY_STAT_COUNT])
{
	lockdep_assert_held(&tp->control_lock);
	memset(values, 0, sizeof(tp->tally.total));
	if (!tp->tally.initialized)
		return tp->tally.sample_state;

	/*
	 * Keep the last cumulative snapshot while closed, suspended, inaccessible,
	 * or on timeout. update_counters() changes it only after a completed dump.
	 */
	(void)rtlmq_update_counters(tp);
	memcpy(values, tp->tally.total, sizeof(tp->tally.total));

	return tp->tally.sample_state;
}
