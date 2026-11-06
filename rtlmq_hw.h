/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef __RTLMQ_HW_H__
#define __RTLMQ_HW_H__

#include <linux/errno.h>
#include <linux/ethtool.h>
#include <linux/limits.h>
#include <linux/netdev_features.h>

#include "rtlmq_desc.h"
#include "rtlmq_base.h"

struct rtlmq_runtime_config;
struct phy_device;

#define RTLMQ_WOL_SUPPORTED_OPTS	(WAKE_PHY | WAKE_MAGIC | \
					 WAKE_UCAST | WAKE_BCAST | \
					 WAKE_MCAST)

struct rtlmq_fw;
struct rtlmq_fw_profile;

/*
 * RX filter register values built from a netdev address-list snapshot.
 * Temporary state for one serialized hardware update, not a copy of the
 * requested address lists.
 */
struct rtlmq_rx_filter_plan {
	u32 rx_mode;
	u32 mc_filter[2];
};

/*
 * RTL8125B and RTL8126A share this 192-byte extended tally layout.
 * Coherent tally buffer allocated at probe and retained until remove.
 * Hardware writes a snapshot only after a counter command. Statistics readers
 * use accumulated software counts rather than issuing their own MMIO reads.
 */
struct rtlmq_counters {
	__le64	tx_packets;
	__le64	rx_packets;
	__le64	tx_errors;
	__le32	rx_errors;
	__le16	rx_missed;
	__le16	align_errors;
	__le32	tx_one_collision;
	__le32	tx_multi_collision;
	__le64	rx_unicast;
	__le64	rx_broadcast;
	__le32	rx_multicast;
	__le16	tx_aborted;
	__le16	tx_underrun;
	__le64	tx_octets;
	__le64	rx_octets;
	__le64	rx_multicast64;
	__le64	tx_unicast64;
	__le64	tx_broadcast64;
	__le64	tx_multicast64;
	__le32	tx_pause_on;
	__le32	tx_pause_off;
	__le32	tx_pause_all;
	__le32	tx_deferred;
	__le32	tx_late_collision;
	__le32	tx_all_collision;
	__le32	tx_aborted32;
	__le32	align_errors32;
	__le32	rx_frame_too_long;
	__le32	rx_runt;
	__le32	rx_pause_on;
	__le32	rx_pause_off;
	__le32	rx_pause_all;
	__le32	rx_unknown_opcode;
	__le32	rx_mac_error;
	__le32	tx_underrun32;
	__le32	rx_mac_missed;
	__le32	rx_tcam_dropped;
	__le32	tdu;
	__le32	rdu;
};

static_assert(sizeof(struct rtlmq_counters) == 192);
static_assert(offsetof(struct rtlmq_counters, tx_octets) == 64);
static_assert(offsetof(struct rtlmq_counters, rx_octets) == 72);
static_assert(offsetof(struct rtlmq_counters, rx_mac_missed) == 176);
static_assert(offsetof(struct rtlmq_counters, rdu) == 188);

/*
 * Export only extended RTL8125B/RTL8126A counters in production. Legacy 16-bit
 * counters can wrap more than once between samples, so production omits them.
 * Keep counter IDs, ethtool names, DMA members, and widths in one table for
 * sampling, accumulation, and reporting.
 */
#define RTLMQ_TALLY_STAT_LIST(X) \
	X(TX_UNICAST, "hw_tx_unicast", tx_unicast64, 64) \
	X(RX_UNICAST, "hw_rx_unicast", rx_unicast, 64) \
	X(TX_PACKETS, "hw_tx_packets", tx_packets, 64) \
	X(RX_PACKETS, "hw_rx_packets", rx_packets, 64) \
	X(TX_OCTETS, "hw_tx_octets", tx_octets, 64) \
	X(RX_OCTETS, "hw_rx_octets", rx_octets, 64) \
	X(TX_MULTICAST, "hw_tx_multicast", tx_multicast64, 64) \
	X(TX_BROADCAST, "hw_tx_broadcast", tx_broadcast64, 64) \
	X(RX_MULTICAST, "hw_rx_multicast", rx_multicast64, 64) \
	X(RX_BROADCAST, "hw_rx_broadcast", rx_broadcast, 64) \
	X(TX_ERRORS, "hw_tx_errors", tx_errors, 64) \
	X(RX_ERRORS, "hw_rx_errors", rx_errors, 32) \
	X(TX_ONE_COLLISION, "hw_tx_one_collision", tx_one_collision, 32) \
	X(TX_MULTI_COLLISION, "hw_tx_multi_collision", tx_multi_collision, 32) \
	X(TX_ALL_COLLISION, "hw_tx_all_collision", tx_all_collision, 32) \
	X(TX_DEFERRED, "hw_tx_deferred", tx_deferred, 32) \
	X(TX_LATE_COLLISION, "hw_tx_late_collision", tx_late_collision, 32) \
	X(TX_ABORTED, "hw_tx_aborted", tx_aborted32, 32) \
	X(TX_UNDERRUN, "hw_tx_underrun", tx_underrun32, 32) \
	X(ALIGN_ERRORS, "hw_align_errors", align_errors32, 32) \
	X(RX_TOO_LONG, "hw_rx_frame_too_long", rx_frame_too_long, 32) \
	X(RX_RUNT, "hw_rx_runt", rx_runt, 32) \
	X(TX_PAUSE_ON, "hw_tx_pause_on", tx_pause_on, 32) \
	X(TX_PAUSE_OFF, "hw_tx_pause_off", tx_pause_off, 32) \
	X(TX_PAUSE_ALL, "hw_tx_pause_all", tx_pause_all, 32) \
	X(RX_PAUSE_ON, "hw_rx_pause_on", rx_pause_on, 32) \
	X(RX_PAUSE_OFF, "hw_rx_pause_off", rx_pause_off, 32) \
	X(RX_PAUSE_ALL, "hw_rx_pause_all", rx_pause_all, 32) \
	X(RX_UNKNOWN_OPCODE, "hw_rx_unknown_opcode", rx_unknown_opcode, 32) \
	X(RX_MAC_ERROR, "hw_rx_mac_error", rx_mac_error, 32) \
	X(RX_MAC_MISSED, "hw_rx_mac_missed", rx_mac_missed, 32) \
	X(RX_TCAM_DROPPED, "hw_rx_tcam_dropped", rx_tcam_dropped, 32) \
	X(TDU, "hw_tdu", tdu, 32) \
	X(RDU, "hw_rdu", rdu, 32)

enum rtlmq_tally_stat_id {
#define RTLMQ_TALLY_STAT_ID(_id, _name, _member, _bits) \
	RTLMQ_TALLY_##_id,
	RTLMQ_TALLY_STAT_LIST(RTLMQ_TALLY_STAT_ID)
#undef RTLMQ_TALLY_STAT_ID
	RTLMQ_TALLY_STAT_COUNT,
};

enum rtlmq_tally_sample_state {
	RTLMQ_TALLY_SAMPLE_UNAVAILABLE = 0,
	RTLMQ_TALLY_SAMPLE_FRESH,
	RTLMQ_TALLY_SAMPLE_RETAINED,
	RTLMQ_TALLY_SAMPLE_READ_FAILED,
};

struct rtlmq_tally_state {
	u64 total[RTLMQ_TALLY_STAT_COUNT];
	u64 previous[RTLMQ_TALLY_STAT_COUNT];
	u64 command_failures;
	/* Result of the last dump attempt; reset withdrawal makes it retained. */
	enum rtlmq_tally_sample_state sample_state;
	bool initialized;
	bool previous_valid;
	bool total_valid;
	/* The DMA scratch buffer is readable only after a completed dump. */
	bool raw_valid;
};

struct ethtool_ops;
struct net_device;
struct netdev_hw_addr_list;
struct net_device_ops;
struct rtlmq_private;
struct rtlmq_rx_filter_plan;

/*
 * Runtime hardware programming: MAC/datapath bring-up and stop, queue/ring
 * register setup, RX mode, RSS, moderation, and TX engine programming.
 */
int rtlmq_hw_init(struct rtlmq_private *tp);
int rtlmq_hw_start(struct rtlmq_private *tp);
int rtlmq_hw_enable_datapath(struct rtlmq_private *tp);
int rtlmq_prepare_mac_reset(struct rtlmq_private *tp, const char *where);
int rtlmq_pause_mac_datapath_locked(struct rtlmq_private *tp);
int rtlmq_pause_all_mac_datapath_locked(struct rtlmq_private *tp);
int rtlmq_resume_mac_datapath_locked(struct rtlmq_private *tp,
				     const struct rtlmq_runtime_config *cfg);
int rtlmq_republish_active_rx_ring_bases_locked(struct rtlmq_private *tp,
						const struct rtlmq_runtime_config *active_cfg);
int rtlmq_republish_active_tx_ring_bases_locked(struct rtlmq_private *tp);
int rtlmq_hw_stop(struct rtlmq_private *tp);
/* control_lock serializes native/CSI extended-configuration RMW. */
int rtlmq_pci_config_modify(struct rtlmq_private *tp, u32 reg, u32 mask,
			    u32 set);
/*
 * Probe disables PCI bus mastering. Open enables it only after reset,
 * PHY/MAC setup, ring programming, queue mappings, and TX doorbells have
 * been validated. Teardown disables mastering before freeing ring DMA
 * storage. Do not expose the new runtime until activation completes.
 */
int rtlmq_pci_enable_bus_master(struct rtlmq_private *tp);
int rtlmq_pci_disable_bus_master(struct rtlmq_private *tp,
				 bool wait_for_pending);
/* Caller holds control_lock with PCI bus mastering disabled. */
int rtlmq_hw_prepare_8125(struct rtlmq_private *tp);
int rtlmq_8126_admit_board_mode(struct rtlmq_private *tp);
int rtlmq_hw_prepare_8126(struct rtlmq_private *tp);
int rtlmq_prepare_d3_8126(struct rtlmq_private *tp);
int rtlmq_hw_reset(struct rtlmq_private *tp, const char *where);
int rtlmq_hw_config_8125_common(struct rtlmq_private *tp);
int rtlmq_wait_mac_ocp_e00e_clear(struct rtlmq_private *tp);
int rtlmq_disable_rxdvgate(struct rtlmq_private *tp);
int rtlmq_configure_mac_8125b(struct rtlmq_private *tp);
int rtlmq_configure_mac_8126(struct rtlmq_private *tp);
int rtlmq_clear_rxdv_gate(struct rtlmq_private *tp);
int rtlmq_disable_double_vlan(struct rtlmq_private *tp);
int rtlmq_pcie_state_l2l3_disable(struct rtlmq_private *tp);
int rtlmq_configure_mac_common(struct rtlmq_private *tp);
int rtlmq_latch_tx_engine_8125(struct rtlmq_private *tp, const char *where);
int rtlmq_latch_tx_engine_8126(struct rtlmq_private *tp, const char *where);
/* Pulse the MCU only after descriptor bases, modes and queue counts are set. */
int rtlmq_prepare_tx_engine_fifo(struct rtlmq_private *tp, bool restart_mcu);
int rtlmq_finish_tx_engine_fifo(struct rtlmq_private *tp);
int rtlmq_pulse_tcam_and_clear_tx_engine(struct rtlmq_private *tp);
int rtlmq_validate_tx_engine_queue_count_latch(struct rtlmq_private *tp,
					       const char *where);
int rtlmq_program_tx_engine_runtime(struct rtlmq_private *tp,
				    const char *where);
int rtlmq_sync_cplus_cmd_features(struct rtlmq_private *tp,
				  netdev_features_t features);
int rtlmq_sync_rx_config_features(struct rtlmq_private *tp,
				  netdev_features_t features);
int rtlmq_read_rx_config(struct rtlmq_private *tp, u32 *value);
int rtlmq_clear_rx_acceptance(struct rtlmq_private *tp);
int rtlmq_program_irq_moderation(struct rtlmq_private *tp, u16 rx, u16 tx);
int rtlmq_program_active_irq_moderation(struct rtlmq_private *tp);
int rtlmq_program_rss(struct rtlmq_private *tp);
int rtlmq_set_bootstrap_rx_mode(struct rtlmq_private *tp);
int rtlmq_build_rx_filter_plan(struct net_device *ndev,
			       const struct netdev_hw_addr_list *uc,
			       const struct netdev_hw_addr_list *mc,
			       struct rtlmq_rx_filter_plan *plan);
int rtlmq_apply_rx_filter_plan_locked(struct rtlmq_private *tp,
				      const struct rtlmq_rx_filter_plan *plan);
int rtlmq_update_rx_mode(struct rtlmq_private *tp);
int rtlmq_program_ring_registers(struct rtlmq_private *tp);
int rtlmq_withdraw_ring_registers(struct rtlmq_private *tp);
int rtlmq_program_queue_state(struct rtlmq_private *tp,
			      const struct rtlmq_runtime_config *cfg);

int rtlmq_identify_chip(struct rtlmq_private *tp);
bool rtlmq_chip_needs_conservative_aspm(const struct rtlmq_private *tp);
const char *rtlmq_config_method_name(enum rtlmq_config_method method);

int rtlmq_read_mac_address(struct rtlmq_private *tp, u8 *addr);
int rtlmq_program_mac_address(struct rtlmq_private *tp, const u8 *addr);

int rtlmq_init_runtime_config(struct rtlmq_private *tp);
int rtlmq_change_mtu(struct net_device *ndev, int new_mtu);

enum rtlmq_rx_desc_format
rtlmq_wanted_rx_desc_fmt(const struct rtlmq_private *tp);

u32 rtlmq_get_supported_wol_opts(const struct rtlmq_private *tp);
int rtlmq_apply_wol(struct rtlmq_private *tp);
int rtlmq_prepare_wol_suspend(struct rtlmq_private *tp);
int rtlmq_prepare_no_wake_suspend(struct rtlmq_private *tp);
int rtlmq_program_eee_mac_locked(struct rtlmq_private *tp);
int rtlmq_set_eee_tx_lpi_locked(struct rtlmq_private *tp, bool enable);
unsigned int rtlmq_get_eee_tx_lpi_timer_us(const struct rtlmq_private *tp);
u16 rtlmq_eee_tx_lpi_timer_bytes(const struct net_device *ndev);

void rtlmq_sync_link_state(struct rtlmq_private *tp);
void rtlmq_request_link_state_sync(struct rtlmq_private *tp);

int rtlmq_request_firmware(struct rtlmq_private *tp);
int rtlmq_apply_firmware(struct rtlmq_private *tp);
void rtlmq_release_firmware(struct rtlmq_private *tp);
const char *rtlmq_firmware_name(const struct rtlmq_private *tp);
const char *rtlmq_firmware_version(const struct rtlmq_private *tp);

/* Page-zero aliases belong to the caller; byte-address bounds are shared. */
static inline int rtlmq_ocp_page_to_base(int page)
{
	if (page < 0 || page > U16_MAX >> 4)
		return -EINVAL;
	return page << 4;
}

u16 rtlmq_mac_ocp_read(struct rtlmq_private *tp, u16 reg);
int rtlmq_mac_ocp_read_checked(struct rtlmq_private *tp, u32 reg);
int rtlmq_mac_ocp_modify_checked(struct rtlmq_private *tp, u32 reg,
				 u16 clear_mask, u16 set_mask);
int rtlmq_mac_ocp_write_checked(struct rtlmq_private *tp, u32 reg, u16 value);

void rtlmq_unlock_config_regs(struct rtlmq_private *tp);
void rtlmq_lock_config_regs(struct rtlmq_private *tp);

extern const struct net_device_ops rtlmq_netdev_ops;
const struct ethtool_ops *rtlmq_get_ethtool_ops(const struct rtlmq_private *tp);

/*
 * rtlmq_phy.c implements MDIO/OCP transport, EEE configuration, and phylib/link
 * lifecycle. rtlmq_phy_config.c owns cold firmware/PHY setup and keeps each
 * family's register programming separate from the shared sequence.
 */
int rtlmq_phy_ocp_write(struct rtlmq_private *tp, u32 reg, u16 value);
int rtlmq_phy_ocp_read(struct rtlmq_private *tp, u32 reg);
int rtlmq_mdio_write(struct rtlmq_private *tp, int reg, int value);
int rtlmq_mdio_read(struct rtlmq_private *tp, int reg);
int rtlmq_apply_eee_policy(struct rtlmq_private *tp);
int rtlmq_phy_init_analog(struct rtlmq_private *tp);
int rtlmq_configure_phy_8125b(struct rtlmq_private *tp);
int rtlmq_configure_phy_8126(struct rtlmq_private *tp);
int rtlmq_prepare_phy(struct rtlmq_private *tp);
int rtlmq_open_phy(struct rtlmq_private *tp);
int rtlmq_resume_wol_phy(struct rtlmq_private *tp);
void rtlmq_start_phy(struct rtlmq_private *tp);
void rtlmq_close_phy(struct rtlmq_private *tp);
void rtlmq_cleanup_phy(struct rtlmq_private *tp);

#if RTLMQ_LEDS_ENABLED
int rtlmq_leds_register(struct rtlmq_private *tp);
void rtlmq_leds_unregister(struct rtlmq_private *tp);
void rtlmq_leds_restore(struct rtlmq_private *tp);
#else
static inline int rtlmq_leds_register(struct rtlmq_private *tp)
{
	(void)tp;
	return 0;
}

static inline void rtlmq_leds_unregister(struct rtlmq_private *tp)
{
	(void)tp;
}

static inline void rtlmq_leds_restore(struct rtlmq_private *tp)
{
	(void)tp;
}
#endif

#endif /* __RTLMQ_HW_H__ */
