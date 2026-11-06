/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef __RTLMQ_DESC_H__
#define __RTLMQ_DESC_H__

#include <linux/bits.h>
#include <linux/compiler.h>
#include <linux/dma-mapping.h>
#include <linux/ktime.h>
#include <linux/swab.h>
#include <linux/types.h>

#include "rtlmq_base.h"
#include "rtlmq_ring.h"

/*
 * Hardware descriptor layouts and encode/decode helpers. Runtime configuration
 * and device state belong in the corresponding runtime and device headers.
 */

#define RTLMQ_DESC_OWN			0x80000000U
#define RTLMQ_DESC_RING_END		0x40000000U
#define RTLMQ_DESC_FIRST_FRAG		0x20000000U
#define RTLMQ_DESC_LAST_FRAG		0x10000000U
#define RTLMQ_TX_DESC_GSO_V4		BIT(26)
#define RTLMQ_TX_DESC_GSO_V6		BIT(25)
#define RTLMQ_TX_DESC_GSO_TCPHO_SHIFT	18
/*
 * Vendor RTL8125 giant-send checks limit the TCP header offset to 0x70, not
 * the full 7-bit field. Reject larger offsets even though the descriptor could
 * encode them.
 */
#define RTLMQ_TX_DESC_GSO_TCPHO_MAX	0x70
#define RTLMQ_TX_DESC_MSS_SHIFT		18
#define RTLMQ_TX_DESC_MSS_MASK		0x7ff
#define RTLMQ_TX_DESC_V3_GSO_ENABLE	BIT(16)
#define RTLMQ_TX_DESC_V3_GSO_IPV4	BIT(29)
#define RTLMQ_TX_DESC_V3_GSO_TCP	BIT(30)
#define RTLMQ_TX_DESC_VLAN_TAG		BIT(17)
#define RTLMQ_TX_DESC_VLAN_TCI_MASK	0x0000ffffU
#define RTLMQ_TX_DESC_TCPHO_SHIFT	18
#define RTLMQ_TX_DESC_TCPHO_MAX		0x3ff
#define RTLMQ_TX_DESC_CSUM_UDP		BIT(31)
#define RTLMQ_TX_DESC_CSUM_TCP		BIT(30)
#define RTLMQ_TX_DESC_CSUM_IP		BIT(29)
#define RTLMQ_TX_DESC_CSUM_IPV6		BIT(28)
#define RTLMQ_TX_TSO_MAX_SEG_LEN	0x3fffU
#define RTLMQ_TX_DESC_LEN_MASK		0x0000ffffU
#define RTLMQ_RX_DESC_V3_FIRST_FRAG	BIT(25)
#define RTLMQ_RX_DESC_V3_LAST_FRAG	BIT(24)

#define RTLMQ_RX_DESC_RWT		0x00400000U
#define RTLMQ_RX_DESC_RES		0x00200000U
#define RTLMQ_RX_DESC_RUNT		0x00100000U
#define RTLMQ_RX_DESC_CRC		0x00080000U
#define RTLMQ_RX_DESC_V6F		BIT(31)
#define RTLMQ_RX_DESC_V4F		BIT(30)
#define RTLMQ_RX_DESC_UDPT		BIT(18)
#define RTLMQ_RX_DESC_TCPT		BIT(17)
#define RTLMQ_RX_DESC_IP_FAIL		BIT(16)
#define RTLMQ_RX_DESC_UDP_FAIL		BIT(15)
#define RTLMQ_RX_DESC_TCP_FAIL		BIT(14)
#define RTLMQ_RX_DESC_V3_RWT		BIT(18)
#define RTLMQ_RX_DESC_V3_RES		BIT(20)
#define RTLMQ_RX_DESC_V3_RUNT		BIT(19)
#define RTLMQ_RX_DESC_V3_CRC		BIT(17)
#define RTLMQ_RX_DESC_V3_UDPT		BIT(29)
#define RTLMQ_RX_DESC_V3_TCPT		BIT(28)
#define RTLMQ_RX_DESC_V3_IP_FAIL	BIT(26)
#define RTLMQ_RX_DESC_V3_UDP_FAIL	BIT(25)
#define RTLMQ_RX_DESC_V3_TCP_FAIL	BIT(24)
#define RTLMQ_RX_DESC_V3_RSS_UDP	BIT(9)
#define RTLMQ_RX_DESC_V3_RSS_IPV4	BIT(10)
#define RTLMQ_RX_DESC_V3_RSS_IPV6	BIT(12)
#define RTLMQ_RX_DESC_V3_RSS_TCP	BIT(13)
#define RTLMQ_RX_DESC_V3_RSS_L3_MASK	(RTLMQ_RX_DESC_V3_RSS_IPV4 | \
					 RTLMQ_RX_DESC_V3_RSS_IPV6)
#define RTLMQ_RX_DESC_V3_RSS_L4_MASK	(RTLMQ_RX_DESC_V3_RSS_TCP | \
					 RTLMQ_RX_DESC_V3_RSS_UDP)
#define RTLMQ_RX_DESC_RES_RSS		0x00400000U
#define RTLMQ_RX_DESC_RUNT_RSS		0x00200000U
#define RTLMQ_RX_DESC_CRC_RSS		0x00100000U
#define RTLMQ_RX_DESC_V4_UDPT		BIT(19)
#define RTLMQ_RX_DESC_V4_TCPT		BIT(18)
#define RTLMQ_RX_DESC_V4_IP_FAIL	BIT(17)
#define RTLMQ_RX_DESC_V4_UDP_FAIL	BIT(16)
#define RTLMQ_RX_DESC_V4_TCP_FAIL	BIT(15)
#define RTLMQ_RX_DESC_V4_RSS_UDP	BIT(27)
#define RTLMQ_RX_DESC_V4_RSS_IPV4	BIT(28)
#define RTLMQ_RX_DESC_V4_RSS_IPV6	BIT(29)
#define RTLMQ_RX_DESC_V4_RSS_TCP	BIT(30)
#define RTLMQ_RX_DESC_V4_RSS_L3_MASK	(RTLMQ_RX_DESC_V4_RSS_IPV4 | \
					 RTLMQ_RX_DESC_V4_RSS_IPV6)
#define RTLMQ_RX_DESC_V4_RSS_L4_MASK	(RTLMQ_RX_DESC_V4_RSS_TCP | \
					 RTLMQ_RX_DESC_V4_RSS_UDP)
#define RTLMQ_RX_DESC_VLAN_TAG		BIT(16)
#define RTLMQ_RX_DESC_VLAN_TCI_MASK	0x0000ffffU
#define RTLMQ_RX_DESC_LEN_MASK		0x00003fffU

static_assert(RTLMQ_TX_DESC_LEN_MASK <= U32_MAX);
static_assert(RTLMQ_RX_DESC_LEN_MASK <= U32_MAX);
static_assert(RTLMQ_TX_TSO_MAX_SEG_LEN <= RTLMQ_TX_DESC_LEN_MASK);

/*
 * RTL8125/RTL8126 store the 16-bit VLAN TCI in descriptor byte order. Keep the
 * conversion here so TX and XDP metadata use the same CPU-endian
 * interpretation.
 */
static inline u32 rtlmq_tx_desc_vlan_tag_encode(u16 tci)
{
	return RTLMQ_TX_DESC_VLAN_TAG |
	       (swab16(tci) & RTLMQ_TX_DESC_VLAN_TCI_MASK);
}

static inline bool rtlmq_rx_desc_vlan_tag_decode(u32 opts2, u16 *tci)
{
	if (!(opts2 & RTLMQ_RX_DESC_VLAN_TAG))
		return false;

	*tci = swab16((u16)(opts2 & RTLMQ_RX_DESC_VLAN_TCI_MASK));
	return true;
}

/*
 * CPU view of either TX wire format. The final 16 bytes exist only on
 * 32-byte rings; use rtlmq_tx_desc_at() for ring indexing.
 */
struct rtlmq_tx_desc {
	__le32 opts1;
	__le32 opts2;
	__le64 addr;
	__le32 reserved0;
	__le32 reserved1;
	__le32 reserved2;
	__le32 reserved3;
};

static_assert(sizeof(struct rtlmq_tx_desc) == 32);

/*
 * Publish one TX descriptor to hardware. Callers fill the address, options,
 * length, and completion state first. dma_wmb() orders those stores before OWN
 * is written. Chained descriptors may be prepared first, but the producer tail
 * must not move until the first descriptor is published.
 */
static inline void rtlmq_tx_desc_device_release(struct rtlmq_tx_desc *desc,
						u32 opts1)
{
	dma_wmb();
	WRITE_ONCE(desc->opts1,
		   cpu_to_le32(opts1 | RTLMQ_DESC_OWN));
}

struct rtlmq_rx_desc {
	__le32 opts1;
	__le32 opts2;
	__le64 addr;
};

/*
 * RX descriptor writes use little-endian wire types and one-copy stores.
 * Profile offsets are alignment-checked, so normal and batched refill paths
 * can use the same helpers.
 */
static inline void rtlmq_rx_desc_dword_write(void *desc, u16 offset,
					     __le32 value)
{
	WRITE_ONCE(*(__le32 *)((u8 *)desc + offset), value);
}

static inline void rtlmq_rx_desc_qword_write(void *desc, u16 offset,
					     __le64 value)
{
	WRITE_ONCE(*(__le64 *)((u8 *)desc + offset), value);
}

/*
 * Publish one refilled RX descriptor to hardware. The caller has already
 * validated the buffer and filled the address, options, and length. Set OWN
 * only after dma_wmb().
 */
static inline void rtlmq_rx_desc_device_release(void *desc, u16 opts1_offset,
						u32 opts1)
{
	dma_wmb();
	rtlmq_rx_desc_dword_write(desc, opts1_offset,
				  cpu_to_le32(opts1 | RTLMQ_DESC_OWN));
}

enum rtlmq_rx_desc_format {
	RX_DESC_FMT_V1 = 0,
	RX_DESC_FMT_V3,
	RX_DESC_FMT_V4,
};

struct rtlmq_rx_desc_profile {
	enum rtlmq_rx_desc_format fmt;
	u16 stride;
	u16 opts1_offset;
	u16 opts2_offset;
	u16 addr_offset;
	u16 clear_size;
	u32 first_frag_mask;
	u32 last_frag_mask;
	u32 error_mask;
	u32 error_length_mask;
	u32 error_crc_mask;
	u32 rxall_drop_mask;
	u32 rxall_allowed_mask;
	u32 len_mask;
	bool csum_status_from_opts2;
	bool csum_version_from_opts2;
	u32 csum_tcp_mask;
	u32 csum_udp_mask;
	u32 csum_ip_fail_mask;
	u32 csum_tcp_fail_mask;
	u32 csum_udp_fail_mask;
	u32 csum_v4_mask;
	u32 csum_v6_mask;
	u32 rss_l3_mask;
	u32 rss_l4_mask;
	u16 rss_info_offset;
	u16 rss_hash_offset;
	u8 rss_info_size;
};

enum rtlmq_rx_desc_decode_flags {
	RTLMQ_RX_DESC_DECODE_CSUM		= BIT(1),
	RTLMQ_RX_DESC_DECODE_RSS		= BIT(2),
	RTLMQ_RX_DESC_DECODE_VLAN		= BIT(3),
};

/*
 * Decoded view of one completed RX descriptor. Decode flags avoid reading
 * optional fields when the caller does not need them. Status fields are always
 * available.
 */
struct rtlmq_rx_desc_decode {
	u32 opts2;
	u32 packet_len;
	u32 rss_l3_info;
	u32 rss_l4_info;
	u32 rss_hash;
	u16 vlan_tci;
	bool first_frag;
	bool last_frag;
	bool fragmented;
	bool metadata_invalid;
	bool error;
	bool error_length;
	bool error_crc;
	bool rxall_drop;
	bool rxall_allowed;
	bool vlan_valid;
	bool rss_valid;
	bool checksum_valid;
	bool checksum_failed;
	bool checksum_v4;
	bool checksum_v6;
};

/*
 * Decode hardware fields once so skb, XDP, stats, and diagnostics all use the
 * same result. Keep this inline to avoid adding a function call in the RX
 * packet loops.
 */
static __always_inline void
rtlmq_decode_rx_desc(const struct rtlmq_rx_desc_profile *profile,
		     const void *desc, u32 status, unsigned int flags,
		     struct rtlmq_rx_desc_decode *decoded)
{
	u32 csum_version;
	u32 csum_status;
	u64 rss_metadata;
	u32 rss_info;
	u32 rss_hash;
	bool read_opts2;
	bool opts2_valid = false;
	bool checksum_tcp;
	bool checksum_udp;

	if (!decoded)
		return;

	*decoded = (struct rtlmq_rx_desc_decode) {
		.packet_len = status & profile->len_mask,
		.first_frag = !!(status & profile->first_frag_mask),
		.last_frag = !!(status & profile->last_frag_mask),
		.error = !!(status & profile->error_mask),
		.error_length = !!(status & profile->error_length_mask),
		.error_crc = !!(status & profile->error_crc_mask),
		.rxall_drop = !!(status & profile->rxall_drop_mask),
		.rxall_allowed = !!(status & profile->rxall_allowed_mask),
	};
	decoded->fragmented = !decoded->first_frag || !decoded->last_frag;
	decoded->metadata_invalid =
		(decoded->error_length || decoded->error_crc) &&
		!decoded->error;
	if (decoded->rxall_drop && decoded->rxall_allowed)
		decoded->metadata_invalid = true;

	read_opts2 = desc &&
		     ((flags & RTLMQ_RX_DESC_DECODE_VLAN) ||
		      ((flags & RTLMQ_RX_DESC_DECODE_CSUM) &&
		       (profile->csum_status_from_opts2 ||
			profile->csum_version_from_opts2)));
	if (read_opts2) {
		decoded->opts2 = le32_to_cpu(READ_ONCE(
			*(const __le32 *)((const u8 *)desc +
					  profile->opts2_offset)));
		opts2_valid = true;
	}
	if ((flags & RTLMQ_RX_DESC_DECODE_VLAN) && !opts2_valid)
		decoded->metadata_invalid = true;
	if ((flags & RTLMQ_RX_DESC_DECODE_CSUM) &&
	    (profile->csum_status_from_opts2 ||
	     profile->csum_version_from_opts2) && !opts2_valid)
		decoded->metadata_invalid = true;
	if (flags & RTLMQ_RX_DESC_DECODE_VLAN)
		decoded->vlan_valid =
			rtlmq_rx_desc_vlan_tag_decode(decoded->opts2,
						      &decoded->vlan_tci);

	if (flags & RTLMQ_RX_DESC_DECODE_CSUM) {
		csum_status = profile->csum_status_from_opts2 &&
			      opts2_valid ? decoded->opts2 : status;
		csum_version = profile->csum_version_from_opts2 &&
				opts2_valid ? decoded->opts2 : status;
		decoded->checksum_v4 = !!(csum_version & profile->csum_v4_mask);
		decoded->checksum_v6 = !!(csum_version & profile->csum_v6_mask);
		checksum_tcp = !!(csum_status & profile->csum_tcp_mask);
		checksum_udp = !!(csum_status & profile->csum_udp_mask);
		if (decoded->checksum_v4 && decoded->checksum_v6)
			decoded->metadata_invalid = true;
		/*
		 * IPv6 has no IP-header checksum. RTL8125 can set IPFail for
		 * valid IPv6 transport checksums, so only transport failure
		 * bits apply to IPv6.
		 */
		if (decoded->checksum_v6)
			csum_status &= ~profile->csum_ip_fail_mask;
		/*
		 * RTL8125 uses both protocol bits for a valid IP-only result.
		 * Neither that nor missing/contradictory metadata authorizes a
		 * TCP/UDP checksum result.
		 */
		if ((checksum_tcp && checksum_udp) || decoded->metadata_invalid) {
			decoded->checksum_valid = false;
		} else if (checksum_tcp) {
			decoded->checksum_failed =
				!!(csum_status & (profile->csum_ip_fail_mask |
						  profile->csum_tcp_fail_mask));
			decoded->checksum_valid = !decoded->checksum_failed;
		} else if (checksum_udp) {
			decoded->checksum_failed =
				!!(csum_status & (profile->csum_ip_fail_mask |
						  profile->csum_udp_fail_mask));
			decoded->checksum_valid = !decoded->checksum_failed;
		}
	}

	if (!(flags & RTLMQ_RX_DESC_DECODE_RSS) ||
	    !profile->rss_l3_mask)
		return;
	if (!desc) {
		decoded->metadata_invalid = true;
		return;
	}

	/*
	 * Both supported RSS descriptor formats keep hash and type in one
	 * aligned qword. Read it once: V3 stores hash low/type high, while V4
	 * stores type low/hash high. This keeps the pair consistent and avoids
	 * a second descriptor load.
	 */
	if (profile->rss_info_size == sizeof(__le16)) {
		rss_metadata = le64_to_cpu(READ_ONCE(
			*(const __le64 *)((const u8 *)desc +
					  profile->rss_hash_offset)));
		rss_info = rss_metadata >> 48;
		rss_hash = rss_metadata;
	} else {
		rss_metadata = le64_to_cpu(READ_ONCE(
			*(const __le64 *)((const u8 *)desc +
					  profile->rss_info_offset)));
		rss_info = rss_metadata;
		rss_hash = rss_metadata >> 32;
	}

	decoded->rss_l3_info = rss_info & profile->rss_l3_mask;
	decoded->rss_l4_info = rss_info & profile->rss_l4_mask;
	if ((decoded->rss_l3_info &&
	     (decoded->rss_l3_info & (decoded->rss_l3_info - 1))) ||
	    (decoded->rss_l4_info &&
	     (decoded->rss_l4_info & (decoded->rss_l4_info - 1))) ||
	    (!decoded->rss_l3_info && decoded->rss_l4_info)) {
		decoded->metadata_invalid = true;
		return;
	}
	decoded->rss_valid = !!decoded->rss_l3_info;
	if (decoded->rss_valid)
		decoded->rss_hash = rss_hash;
}

static __always_inline bool
rtlmq_rx_desc_error_deliverable_decoded(const struct rtlmq_rx_desc_decode *decoded, bool rxall)
{
	if (decoded->metadata_invalid)
		return false;
	if (!decoded->error)
		return true;
	if (!rxall || decoded->rxall_drop)
		return false;

	/*
	 * RXALL is intentionally narrow. Only runt and CRC errors may be
	 * delivered; summary-only and other fatal errors are always dropped.
	 */
	return decoded->rxall_allowed;
}

/*
 * XDP RSS type mapping is cold metadata. Keep it out of the read-mostly RX
 * ring profile so descriptor polling does not spend cache space on it.
 */
struct rtlmq_rx_rss_xdp_map {
	u32 ipv4_mask;
	u32 ipv6_mask;
	u32 tcp_mask;
	u32 udp_mask;
};

struct rtlmq_rx_buffer;
struct rtlmq_rx_ring;
struct rtlmq_tx_ring;

int rtlmq_rx_desc_ring_size(unsigned int ring_count,
			    enum rtlmq_rx_desc_format fmt, size_t *size);
const char *rtlmq_rx_desc_fmt_name(enum rtlmq_rx_desc_format fmt);
int rtlmq_decode_rx_desc_fmt(u32 rx_config, u8 rx_desc_mode,
			     enum rtlmq_rx_desc_format *fmt);
void rtlmq_init_rx_desc_profile(struct rtlmq_rx_ring *ring);
void *rtlmq_rx_desc_addr(const struct rtlmq_rx_ring *ring, unsigned int entry);
__le32 rtlmq_rx_desc_opts1(const struct rtlmq_rx_ring *ring, const void *desc);
bool rtlmq_publish_rx_desc(struct rtlmq_rx_ring *ring, unsigned int entry);
void rtlmq_clear_tx_desc(struct rtlmq_tx_ring *ring, unsigned int entry);

#endif /* __RTLMQ_DESC_H__ */
