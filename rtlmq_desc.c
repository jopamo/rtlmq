// SPDX-License-Identifier: GPL-2.0-only

#include <linux/bitfield.h>
#include <linux/kernel.h>
#include <linux/moduleparam.h>
#include <linux/overflow.h>
#include <linux/string.h>

#include "rtlmq_hw.h"
#include "rtlmq.h"
#include "rtlmq_rx.h"
#include "rtlmq_ring.h"
#include "rtlmq_regs.h"

struct rtlmq_rx_desc_v3 {
	__le32 reserved0;
	__le32 reserved1;
	union {
		struct {
			__le32 rss_hash;
			__le16 header_buffer_len;
			__le16 rss_info;
		};
		struct {
			__le32 reserved2;
			__le32 reserved3;
		};
	};
	__le64 addr;
	__le32 opts2;
	__le32 opts1;
};

struct rtlmq_rx_desc_v4 {
	union {
		__le64 addr;
		struct {
			__le32 rss_info;
			__le32 rss_hash;
		};
	};
	struct {
		__le32 opts2;
		__le32 opts1;
	} opts;
};

static_assert(!(offsetof(struct rtlmq_rx_desc, opts1) & 3));
static_assert(!(offsetof(struct rtlmq_rx_desc, opts2) & 3));
static_assert(!(offsetof(struct rtlmq_rx_desc, addr) & 7));
static_assert(!(offsetof(struct rtlmq_rx_desc_v3, opts1) & 3));
static_assert(!(offsetof(struct rtlmq_rx_desc_v3, opts2) & 3));
static_assert(!(offsetof(struct rtlmq_rx_desc_v3, addr) & 7));
static_assert(offsetof(struct rtlmq_rx_desc_v3, addr) == 16);
static_assert(offsetof(struct rtlmq_rx_desc_v3, opts2) == 24);
static_assert(!(offsetof(struct rtlmq_rx_desc_v4, opts.opts1) & 3));
static_assert(!(offsetof(struct rtlmq_rx_desc_v4, opts.opts2) & 3));
static_assert(!(offsetof(struct rtlmq_rx_desc_v4, addr) & 7));
static_assert(sizeof(struct rtlmq_rx_desc_v4) == 16);
static_assert(offsetof(struct rtlmq_rx_desc_v4, addr) == 0);
static_assert(offsetof(struct rtlmq_rx_desc_v4, rss_info) == 0);
static_assert(offsetof(struct rtlmq_rx_desc_v4, rss_hash) == 4);
static_assert(offsetof(struct rtlmq_rx_desc_v4, opts.opts2) == 8);
static_assert(offsetof(struct rtlmq_rx_desc_v4, opts.opts1) == 12);

static __le32 rtlmq_rx_desc_dword_read(const void *desc, u16 offset)
{
	return READ_ONCE(*(const __le32 *)((const u8 *)desc + offset));
}

static size_t rtlmq_rx_desc_format_size(enum rtlmq_rx_desc_format fmt)
{
	switch (fmt) {
	case RX_DESC_FMT_V3:
		return sizeof(struct rtlmq_rx_desc_v3);
	case RX_DESC_FMT_V4:
	case RX_DESC_FMT_V1:
	default:
		return sizeof(struct rtlmq_rx_desc);
	}
}

int rtlmq_rx_desc_ring_size(unsigned int ring_count,
			    enum rtlmq_rx_desc_format fmt, size_t *size)
{
	if (!ring_count || !size || fmt < RX_DESC_FMT_V1 ||
	    fmt > RX_DESC_FMT_V4)
		return -EINVAL;

	if (check_mul_overflow((size_t)ring_count,
			       rtlmq_rx_desc_format_size(fmt), size))
		return -EOVERFLOW;

	return 0;
}

const char *rtlmq_rx_desc_fmt_name(enum rtlmq_rx_desc_format fmt)
{
	switch (fmt) {
	case RX_DESC_FMT_V1:
		return "v1";
	case RX_DESC_FMT_V3:
		return "v3";
	case RX_DESC_FMT_V4:
		return "v4";
	default:
		return "unknown";
	}
}

int rtlmq_decode_rx_desc_fmt(u32 rx_config, u8 rx_desc_mode,
			     enum rtlmq_rx_desc_format *fmt)
{
	bool v3 = !!(rx_config & RTLMQ_RX_CONFIG_ENABLE_RX_DESC_V3);
	bool v4 = !!(rx_desc_mode & RTLMQ_RX_DESC_MODE_V4);

	if (!fmt)
		return -EINVAL;

	if (v3 && v4)
		return -EINVAL;

	if (v3)
		*fmt = RX_DESC_FMT_V3;
	else if (v4)
		*fmt = RX_DESC_FMT_V4;
	else
		*fmt = RX_DESC_FMT_V1;

	return 0;
}

static const struct rtlmq_rx_desc_profile rtlmq_rx_desc_profiles[] = {
	[RX_DESC_FMT_V1] = {
		.fmt			= RX_DESC_FMT_V1,
		.stride			= sizeof(struct rtlmq_rx_desc),
		.opts1_offset		= offsetof(struct rtlmq_rx_desc, opts1),
		.opts2_offset		= offsetof(struct rtlmq_rx_desc, opts2),
		.addr_offset		= offsetof(struct rtlmq_rx_desc, addr),
		.first_frag_mask	= RTLMQ_DESC_FIRST_FRAG,
		.last_frag_mask		= RTLMQ_DESC_LAST_FRAG,
		.error_mask		= RTLMQ_RX_DESC_RES |
					  RTLMQ_RX_DESC_RWT |
					  RTLMQ_RX_DESC_RUNT |
					  RTLMQ_RX_DESC_CRC,
		.error_length_mask	= RTLMQ_RX_DESC_RWT |
					  RTLMQ_RX_DESC_RUNT,
		.error_crc_mask		= RTLMQ_RX_DESC_CRC,
		.rxall_drop_mask	= RTLMQ_RX_DESC_RWT,
		.rxall_allowed_mask	= RTLMQ_RX_DESC_RUNT |
					  RTLMQ_RX_DESC_CRC,
		.len_mask		= RTLMQ_RX_DESC_LEN_MASK,
		.csum_version_from_opts2 = true,
		.csum_tcp_mask		= RTLMQ_RX_DESC_TCPT,
		.csum_udp_mask		= RTLMQ_RX_DESC_UDPT,
		.csum_ip_fail_mask	= RTLMQ_RX_DESC_IP_FAIL,
		.csum_tcp_fail_mask	= RTLMQ_RX_DESC_TCP_FAIL,
		.csum_udp_fail_mask	= RTLMQ_RX_DESC_UDP_FAIL,
		.csum_v4_mask		= RTLMQ_RX_DESC_V4F,
		.csum_v6_mask		= RTLMQ_RX_DESC_V6F,
	},
	[RX_DESC_FMT_V3] = {
		.fmt			= RX_DESC_FMT_V3,
		.stride			= sizeof(struct rtlmq_rx_desc_v3),
		.opts1_offset		= offsetof(struct rtlmq_rx_desc_v3,
						   opts1),
		.opts2_offset		= offsetof(struct rtlmq_rx_desc_v3,
						   opts2),
		.addr_offset		= offsetof(struct rtlmq_rx_desc_v3, addr),
		.clear_size		= sizeof(struct rtlmq_rx_desc_v3),
		.first_frag_mask	= RTLMQ_RX_DESC_V3_FIRST_FRAG,
		.last_frag_mask		= RTLMQ_RX_DESC_V3_LAST_FRAG,
		.error_mask		= RTLMQ_RX_DESC_V3_RES |
					  RTLMQ_RX_DESC_V3_RWT |
					  RTLMQ_RX_DESC_V3_RUNT |
					  RTLMQ_RX_DESC_V3_CRC,
		.error_length_mask	= RTLMQ_RX_DESC_V3_RWT |
					  RTLMQ_RX_DESC_V3_RUNT,
		.error_crc_mask		= RTLMQ_RX_DESC_V3_CRC,
		.rxall_drop_mask	= RTLMQ_RX_DESC_V3_RWT,
		.rxall_allowed_mask	= RTLMQ_RX_DESC_V3_RUNT |
					  RTLMQ_RX_DESC_V3_CRC,
		.len_mask		= RTLMQ_RX_DESC_LEN_MASK,
		.csum_status_from_opts2 = true,
		.csum_version_from_opts2 = true,
		.csum_tcp_mask		= RTLMQ_RX_DESC_V3_TCPT,
		.csum_udp_mask		= RTLMQ_RX_DESC_V3_UDPT,
		.csum_ip_fail_mask	= RTLMQ_RX_DESC_V3_IP_FAIL,
		.csum_tcp_fail_mask	= RTLMQ_RX_DESC_V3_TCP_FAIL,
		.csum_udp_fail_mask	= RTLMQ_RX_DESC_V3_UDP_FAIL,
		.csum_v4_mask		= RTLMQ_RX_DESC_V4F,
		.csum_v6_mask		= RTLMQ_RX_DESC_V6F,
		.rss_l3_mask		= RTLMQ_RX_DESC_V3_RSS_L3_MASK,
		.rss_l4_mask		= RTLMQ_RX_DESC_V3_RSS_L4_MASK,
		.rss_info_offset	= offsetof(struct rtlmq_rx_desc_v3,
						   rss_info),
		.rss_hash_offset	= offsetof(struct rtlmq_rx_desc_v3,
						   rss_hash),
		.rss_info_size		= sizeof(__le16),
	},
	[RX_DESC_FMT_V4] = {
		.fmt			= RX_DESC_FMT_V4,
		.stride			= sizeof(struct rtlmq_rx_desc_v4),
		.opts1_offset		= offsetof(struct rtlmq_rx_desc_v4,
						   opts.opts1),
		.opts2_offset		= offsetof(struct rtlmq_rx_desc_v4,
						   opts.opts2),
		.addr_offset		= offsetof(struct rtlmq_rx_desc_v4, addr),
		.first_frag_mask	= RTLMQ_DESC_FIRST_FRAG,
		.last_frag_mask		= RTLMQ_DESC_LAST_FRAG,
		/* V4 error details require RES; orphan details are invalid. */
		.error_mask		= RTLMQ_RX_DESC_RES_RSS,
		.error_length_mask	= RTLMQ_RX_DESC_RUNT_RSS,
		.error_crc_mask		= RTLMQ_RX_DESC_CRC_RSS,
		.rxall_allowed_mask	= RTLMQ_RX_DESC_RUNT_RSS |
					  RTLMQ_RX_DESC_CRC_RSS,
		.len_mask		= RTLMQ_RX_DESC_LEN_MASK,
		.csum_tcp_mask		= RTLMQ_RX_DESC_V4_TCPT,
		.csum_udp_mask		= RTLMQ_RX_DESC_V4_UDPT,
		.csum_ip_fail_mask	= RTLMQ_RX_DESC_V4_IP_FAIL,
		.csum_tcp_fail_mask	= RTLMQ_RX_DESC_V4_TCP_FAIL,
		.csum_udp_fail_mask	= RTLMQ_RX_DESC_V4_UDP_FAIL,
		.csum_v4_mask		= RTLMQ_RX_DESC_V4F,
		.csum_v6_mask		= RTLMQ_RX_DESC_V6F,
		.rss_l3_mask		= RTLMQ_RX_DESC_V4_RSS_L3_MASK,
		.rss_l4_mask		= RTLMQ_RX_DESC_V4_RSS_L4_MASK,
		.rss_info_offset	= offsetof(struct rtlmq_rx_desc_v4,
						   rss_info),
		.rss_hash_offset	= offsetof(struct rtlmq_rx_desc_v4,
						   rss_hash),
		.rss_info_size		= sizeof(__le32),
	},
};

void rtlmq_init_rx_desc_profile(struct rtlmq_rx_ring *ring)
{
	enum rtlmq_rx_desc_format fmt = READ_ONCE(ring->priv->rx_desc_fmt);

	if (WARN_ON_ONCE(fmt < RX_DESC_FMT_V1 || fmt > RX_DESC_FMT_V4))
		fmt = RX_DESC_FMT_V1;

	ring->profile = &rtlmq_rx_desc_profiles[fmt];
	if (ring->cold) {
		struct rtlmq_rx_rss_xdp_map *map = &ring->cold->xdp_rss_map;

		memset(map, 0, sizeof(*map));
		if (fmt == RX_DESC_FMT_V3) {
			map->ipv4_mask = RTLMQ_RX_DESC_V3_RSS_IPV4;
			map->ipv6_mask = RTLMQ_RX_DESC_V3_RSS_IPV6;
			map->tcp_mask = RTLMQ_RX_DESC_V3_RSS_TCP;
			map->udp_mask = RTLMQ_RX_DESC_V3_RSS_UDP;
		} else if (fmt == RX_DESC_FMT_V4) {
			map->ipv4_mask = RTLMQ_RX_DESC_V4_RSS_IPV4;
			map->ipv6_mask = RTLMQ_RX_DESC_V4_RSS_IPV6;
			map->tcp_mask = RTLMQ_RX_DESC_V4_RSS_TCP;
			map->udp_mask = RTLMQ_RX_DESC_V4_RSS_UDP;
		}
	}

	rtlmq_refresh_rx_ring_fast_state(ring);
}

void *rtlmq_rx_desc_addr(const struct rtlmq_rx_ring *ring, unsigned int entry)
{
	return ring->desc + entry * ring->profile->stride;
}

__le32 rtlmq_rx_desc_opts1(const struct rtlmq_rx_ring *ring, const void *desc)
{
	return rtlmq_rx_desc_dword_read(desc, ring->profile->opts1_offset);
}

bool rtlmq_publish_rx_desc(struct rtlmq_rx_ring *ring, unsigned int entry)
{
	void *desc = rtlmq_rx_desc_addr(ring, entry);
	const struct rtlmq_rx_buffer *buffer = &ring->buffers[entry];
	u32 opts1 = ring->buffer_len;

	if (!rtlmq_rx_buffer_ready_for_device(buffer))
		return false;
	if (!ring->buffer_len)
		return false;
	if (ring->buffer_len > RTLMQ_RX_DESC_LEN_MASK ||
	    !rtlmq_dma_range_fits_device(&ring->priv->pdev->dev,
					 buffer->dma_addr,
					 ring->buffer_len)) {
		return false;
	}
	if (entry == ring->ring_count - 1)
		opts1 |= RTLMQ_DESC_RING_END;

	if (ring->profile->clear_size)
		memset(desc, 0, ring->profile->clear_size);

	rtlmq_rx_desc_qword_write(desc, ring->profile->addr_offset,
				  cpu_to_le64(buffer->dma_addr));
	rtlmq_rx_desc_dword_write(desc, ring->profile->opts2_offset,
				  cpu_to_le32(0));
	rtlmq_rx_desc_device_release(desc, ring->profile->opts1_offset, opts1);

	return true;
}

void rtlmq_clear_tx_desc(struct rtlmq_tx_ring *ring, unsigned int entry)
{
	if (!ring || !ring->desc || entry >= ring->ring_count)
		return;

	memset(rtlmq_tx_desc_at(ring, entry), 0,
	       rtlmq_tx_ring_desc_stride(ring));
	if (entry == ring->ring_count - 1)
		WRITE_ONCE(rtlmq_tx_desc_at(ring, entry)->opts1,
			   cpu_to_le32(RTLMQ_DESC_RING_END));
}
