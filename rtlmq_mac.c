// SPDX-License-Identifier: GPL-2.0-only

#include <linux/etherdevice.h>
#include <linux/delay.h>
#include <linux/rtnetlink.h>
#include <linux/unaligned.h>

#include "rtlmq_regs.h"
#include "rtlmq_hw.h"
#include "rtlmq.h"
#include "rtlmq_runtime_accessors.h"
#include "rtlmq_lifecycle.h"

static int __rtlmq_mac_ocp_read(struct rtlmq_private *tp, u16 reg)
{
	u32 data;

	writel((u32)reg << RTLMQ_OCP_REG_SHIFT,
	       tp->mmio_addr + RTLMQ_OCPDR);

	data = readl(tp->mmio_addr + RTLMQ_OCPDR);
	if (data == U32_MAX)
		return -EIO;
	return data & RTLMQ_OCP_DATA_MASK;
}

static void __rtlmq_mac_ocp_write(struct rtlmq_private *tp, u16 reg, u16 value)
{
	writel(RTLMQ_OCP_CMD_FLAG |
	       ((u32)reg << RTLMQ_OCP_REG_SHIFT) | value,
	       tp->mmio_addr + RTLMQ_OCPDR);
}

static bool rtlmq_mac_ocp_reg_valid(u32 reg)
{
	/* OCP byte addresses are even and fit the 15-bit word-address field. */
	return reg <= U16_MAX && !(reg & 1);
}

int rtlmq_mac_ocp_read_checked(struct rtlmq_private *tp, u32 reg)
{
	unsigned long flags;
	int value;

	if (!rtlmq_mac_ocp_reg_valid(reg))
		return -EINVAL;
	if (!tp)
		return -ENODEV;

	spin_lock_irqsave(&tp->mac_ocp_lock, flags);
	value = rtlmq_mmio_ordinary_authorized(tp) ?
		__rtlmq_mac_ocp_read(tp, reg) : -ENODEV;
	spin_unlock_irqrestore(&tp->mac_ocp_lock, flags);
	return value;
}

int rtlmq_mac_ocp_write_checked(struct rtlmq_private *tp, u32 reg, u16 value)
{
	unsigned long flags;
	int ret = 0;

	if (!rtlmq_mac_ocp_reg_valid(reg))
		return -EINVAL;
	if (!tp)
		return -ENODEV;

	spin_lock_irqsave(&tp->mac_ocp_lock, flags);
	/* This transport has no write acknowledgment or completion poll. */
	if (rtlmq_mmio_ordinary_authorized(tp))
		__rtlmq_mac_ocp_write(tp, reg, value);
	else
		ret = -ENODEV;
	spin_unlock_irqrestore(&tp->mac_ocp_lock, flags);
	return ret;
}

u16 rtlmq_mac_ocp_read(struct rtlmq_private *tp, u16 reg)
{
	int value = rtlmq_mac_ocp_read_checked(tp, reg);

	return value < 0 ? U16_MAX : value;
}

int rtlmq_mac_ocp_modify_checked(struct rtlmq_private *tp, u32 reg,
				 u16 clear_mask, u16 set_mask)
{
	unsigned long flags;
	int value;

	if (!rtlmq_mac_ocp_reg_valid(reg))
		return -EINVAL;
	if (!tp)
		return -ENODEV;

	spin_lock_irqsave(&tp->mac_ocp_lock, flags);
	if (!rtlmq_mmio_ordinary_authorized(tp)) {
		value = -ENODEV;
		goto out_unlock;
	}
	value = __rtlmq_mac_ocp_read(tp, reg);
	if (value < 0)
		goto out_unlock;
	__rtlmq_mac_ocp_write(tp, reg,
			      (value & ~clear_mask) | set_mask);
	value = 0;

out_unlock:
	spin_unlock_irqrestore(&tp->mac_ocp_lock, flags);
	return value;
}

void rtlmq_unlock_config_regs(struct rtlmq_private *tp)
{
	if (!rtlmq_mmio_ordinary_authorized(tp))
		return;

	writeb(RTLMQ_CFG9346_UNLOCK, tp->mmio_addr + RTLMQ_CFG9346);
}

void rtlmq_lock_config_regs(struct rtlmq_private *tp)
{
	if (!rtlmq_mmio_ordinary_authorized(tp))
		return;

	writeb(RTLMQ_CFG9346_LOCK, tp->mmio_addr + RTLMQ_CFG9346);
}

static int rtlmq_clear_mmio_bits8(struct rtlmq_private *tp, u32 reg, u8 mask)
{
	u8 value;

	if (!tp || !rtlmq_mmio_ordinary_authorized(tp))
		return -ENODEV;
	value = readb(tp->mmio_addr + reg);
	if (value == U8_MAX)
		return -ENODEV;
	writeb(value & ~mask, tp->mmio_addr + reg);
	value = readb(tp->mmio_addr + reg);
	if (value == U8_MAX)
		return -ENODEV;
	return value & mask ? -EIO : 0;
}

int rtlmq_clear_rxdv_gate(struct rtlmq_private *tp)
{
	return rtlmq_clear_mmio_bits8(tp, RTLMQ_PPSW, RTLMQ_PPSW_RXDV_GATED_EN);
}

int rtlmq_pcie_state_l2l3_disable(struct rtlmq_private *tp)
{
	return rtlmq_clear_mmio_bits8(tp, RTLMQ_CONFIG3, RTLMQ_CONFIG3_RDY_TO_L23);
}

int rtlmq_disable_double_vlan(struct rtlmq_private *tp)
{
	u16 value;

	if (!tp || !rtlmq_mmio_ordinary_authorized(tp))
		return -ENODEV;

	/*
	 * Realtek mode 0xf002 strips both VLAN tags, but the RX descriptor can
	 * report only one TCI. It also strips a single CTAG without reporting VLAN
	 * metadata. Reset this register to one-tag mode before enabling RX so
	 * software only sees states it can represent.
	 */
	writew(0, tp->mmio_addr + RTLMQ_DOUBLE_VLAN_CONFIG);
	value = readw(tp->mmio_addr + RTLMQ_DOUBLE_VLAN_CONFIG);
	if (value == U16_MAX)
		return -ENODEV;

	return value ? -EIO : 0;
}

/* Common V1/r8125 Config1 and C140/C142 setup; no queue or DMA authority. */
int rtlmq_configure_mac_common(struct rtlmq_private *tp)
{
	int ret;

	ret = rtlmq_clear_mmio_bits8(tp, RTLMQ_CONFIG1, RTLMQ_CONFIG1_SPEED_DOWN);
	if (ret)
		return ret;
	ret = rtlmq_mac_ocp_write_checked(tp, RTLMQ_MAC_OCP_VENDOR_C140, U16_MAX);
	if (ret)
		return ret;
	return rtlmq_mac_ocp_write_checked(tp, RTLMQ_MAC_OCP_VENDOR_C142, U16_MAX);
}

u16 rtlmq_eee_tx_lpi_timer_bytes(const struct net_device *ndev)
{
	/*
	 * Both families store this timer in transmitted bytes. Follow r8169 and
	 * allow one MTU-sized frame, the Ethernet header, and 32 bytes of
	 * controller headroom.
	 */
	return READ_ONCE(ndev->mtu) + ETH_HLEN + 0x20;
}

unsigned int rtlmq_get_eee_tx_lpi_timer_us(const struct rtlmq_private *tp)
{
	unsigned int timer;
	int speed;

	if (!tp || !tp->ndev || !tp->phydev ||
	    !rtlmq_qualified_supports_eee(tp))
		return 0;

	speed = READ_ONCE(tp->phydev->speed);
	if (speed <= 0 || speed == SPEED_UNKNOWN)
		return 0;

	timer = rtlmq_eee_tx_lpi_timer_bytes(tp->ndev);
	return DIV_ROUND_CLOSEST(timer * BITS_PER_BYTE, speed);
}

int rtlmq_prepare_tx_engine_fifo(struct rtlmq_private *tp, bool restart_mcu)
{
	int ret;

	/* Both vendor families preserve the queue count in E63E[11:10]. */
	ret = rtlmq_mac_ocp_modify_checked(tp, RTLMQ_TX_QUEUE_COUNT_OCP_REG,
					   0x0030, 0x0020);
	if (ret)
		return ret;
	if (restart_mcu) {
		ret = rtlmq_mac_ocp_modify_checked(tp, RTLMQ_MAC_OCP_VENDOR_MCU_CTRL, BIT(0), 0);
		if (ret)
			return ret;
		ret = rtlmq_mac_ocp_modify_checked(tp, RTLMQ_MAC_OCP_VENDOR_MCU_CTRL, 0, BIT(0));
		if (ret)
			return ret;
	}
	ret = rtlmq_mac_ocp_modify_checked(tp, RTLMQ_MAC_OCP_VENDOR_MCU_CTRL, 0, 0x000c);
	if (ret)
		return ret;
	ret = rtlmq_mac_ocp_modify_checked(tp, RTLMQ_MAC_OCP_VENDOR_EB6A, 0x00ff, 0x0033);
	if (ret)
		return ret;
	ret = rtlmq_mac_ocp_modify_checked(tp, RTLMQ_MAC_OCP_VENDOR_EB50, 0x03e0, 0x0040);
	if (ret)
		return ret;
	ret = rtlmq_mac_ocp_modify_checked(tp, RTLMQ_MAC_OCP_VENDOR_E056, 0x00f0, 0);
	if (ret)
		return ret;
	/* Both sources use a sixteen-descriptor fetch window. */
	writeb(RTLMQ_TX_DESC_FETCH_COUNT, tp->mmio_addr + RTLMQ_TX_DESC_FETCH_NUM);
	ret = rtlmq_mac_ocp_modify_checked(tp, RTLMQ_MAC_OCP_VENDOR_E040, 0x1000, 0);
	if (ret)
		return ret;
	return rtlmq_mac_ocp_modify_checked(tp, RTLMQ_MAC_OCP_VENDOR_EA1C, 0x0003, 0x0001);
}

int rtlmq_pulse_tcam_and_clear_tx_engine(struct rtlmq_private *tp)
{
	u16 control;
	int ret;

	ret = rtlmq_mac_ocp_modify_checked(tp, RTLMQ_MAC_OCP_VENDOR_EB54, 0, BIT(0));
	if (ret)
		return ret;
	udelay(1);
	ret = rtlmq_mac_ocp_modify_checked(tp, RTLMQ_MAC_OCP_VENDOR_EB54, BIT(0), 0);
	if (ret)
		return ret;
	control = readw(tp->mmio_addr + RTLMQ_VENDOR_TX_ENGINE_MMIO_1880);
	if (control == U16_MAX)
		return -ENODEV;
	writew(control & ~(BIT(4) | BIT(5)), tp->mmio_addr + RTLMQ_VENDOR_TX_ENGINE_MMIO_1880);

	return 0;
}

int rtlmq_finish_tx_engine_fifo(struct rtlmq_private *tp)
{
	int ret;

	ret = rtlmq_pulse_tcam_and_clear_tx_engine(tp);
	if (ret)
		return ret;
	ret = rtlmq_mac_ocp_modify_checked(tp, RTLMQ_MAC_OCP_VENDOR_C0AC, 0, GENMASK(12, 7));
	if (ret)
		return ret;
	return rtlmq_mac_ocp_write_checked(tp, RTLMQ_MAC_OCP_VENDOR_E098, 0xc302);
}

u32 rtlmq_get_supported_wol_opts(const struct rtlmq_private *tp)
{
	return rtlmq_qualified_wol_opts(tp);
}

int rtlmq_read_mac_address(struct rtlmq_private *tp, u8 *addr)
{
	unsigned int i;

	if (!rtlmq_mmio_ordinary_authorized(tp) || !addr)
		return -EINVAL;

	for (i = 0; i < ETH_ALEN; i++)
		addr[i] = readb(tp->mmio_addr + RTLMQ_MAC0 + i);

	return is_valid_ether_addr(addr) ? 0 : -EADDRNOTAVAIL;
}

static int rtlmq_pci_commit(struct rtlmq_private *tp)
{
	if (!rtlmq_mmio_ordinary_authorized(tp))
		return -ENODEV;

	return readb(tp->mmio_addr + RTLMQ_CHIP_CMD) == U8_MAX ?
	       -ENODEV : 0;
}

int rtlmq_program_mac_address(struct rtlmq_private *tp, const u8 *addr)
{
	u8 readback[ETH_ALEN];
	int ret;

	if (!tp || !rtlmq_mmio_ordinary_authorized(tp))
		return -ENODEV;
	if (!addr || !is_valid_ether_addr(addr))
		return -EINVAL;
	lockdep_assert_held(&tp->control_lock);

	/*
	 * Open/reset programs the station address with descriptors and RX
	 * filters. Write MAC4 before MAC0, matching upstream Realtek ordering.
	 */
	rtlmq_unlock_config_regs(tp);
	writel(get_unaligned_le16(addr + 4), tp->mmio_addr + RTLMQ_MAC4);
	ret = rtlmq_pci_commit(tp);
	if (ret)
		goto out_lock;
	writel(get_unaligned_le32(addr), tp->mmio_addr + RTLMQ_MAC0);
	ret = rtlmq_pci_commit(tp);
	if (ret)
		goto out_lock;
	rtlmq_lock_config_regs(tp);

	ret = rtlmq_read_mac_address(tp, readback);
	if (ret || !ether_addr_equal(readback, addr))
		ret = -EIO;

	return ret;

out_lock:
	if (ret != -ENODEV)
		rtlmq_lock_config_regs(tp);
	return ret;
}

int rtlmq_build_rx_filter_plan(struct net_device *ndev,
			       const struct netdev_hw_addr_list *uc,
			       const struct netdev_hw_addr_list *mc,
			       struct rtlmq_rx_filter_plan *plan)
{
	struct netdev_hw_addr *ha;
	unsigned int flags;

	if (!ndev || !uc || !mc || !plan)
		return -EINVAL;

	memset(plan, 0, sizeof(*plan));
	plan->rx_mode = RTLMQ_RX_ACCEPT_BROADCAST |
			RTLMQ_RX_ACCEPT_MY_PHYS;
	flags = READ_ONCE(ndev->flags);
	/*
	 * With no secondary filter table, use permissive unicast mode whenever
	 * the snapshot requests an additional address.
	 * IFF_UNICAST_FLT stays clear so net core tracks unicast promiscuity.
	 */
	if (!netdev_hw_addr_list_empty(uc))
		plan->rx_mode |= RTLMQ_RX_ACCEPT_ALL_PHYS;
	if (flags & IFF_PROMISC) {
		plan->rx_mode |= RTLMQ_RX_ACCEPT_ALL_PHYS |
				 RTLMQ_RX_ACCEPT_MULTICAST;
		plan->mc_filter[0] = ~0U;
		plan->mc_filter[1] = ~0U;
		return 0;
	}

	if (!(flags & IFF_MULTICAST))
		return 0;

	plan->rx_mode |= RTLMQ_RX_ACCEPT_MULTICAST;
	if (flags & IFF_ALLMULTI) {
		plan->mc_filter[0] = ~0U;
		plan->mc_filter[1] = ~0U;
		return 0;
	}

	if (netdev_hw_addr_list_empty(mc)) {
		plan->rx_mode &= ~RTLMQ_RX_ACCEPT_MULTICAST;
		return 0;
	}

	netdev_hw_addr_list_for_each(ha, mc) {
		u32 bit_nr = eth_hw_addr_crc(ha) >> 26;

		plan->mc_filter[bit_nr >> 5] |= BIT(bit_nr & 31);
	}
	{
		u32 tmp = plan->mc_filter[0];

		plan->mc_filter[0] = swab32(plan->mc_filter[1]);
		plan->mc_filter[1] = swab32(tmp);
	}

	return 0;
}

static int rtlmq_program_rx_filter_plan(struct rtlmq_private *tp,
					const struct rtlmq_rx_filter_plan *plan)
{
	u32 rx_config;
	u32 value;
	int ret;

	if (!tp || !rtlmq_mmio_ordinary_authorized(tp) || !plan)
		return -ENODEV;
	lockdep_assert_held(&tp->control_lock);

	/*
	 * The plan is complete before the first write. Preserve unrelated
	 * RxConfig state while replacing the complete station/multicast filter
	 * image and its low receive-mode bits.
	 */
	ret = rtlmq_read_rx_config(tp, &rx_config);
	if (ret)
		return ret;
	rx_config &= ~RTLMQ_RX_CONFIG_ACCEPT_OK_MASK;
	rx_config |= plan->rx_mode;
	writel(plan->mc_filter[0], tp->mmio_addr + RTLMQ_MAR0);
	{
		writel(plan->mc_filter[1],
		       tp->mmio_addr + RTLMQ_MAR0 + sizeof(u32));
	}
	writel(rx_config, tp->mmio_addr + RTLMQ_RX_CONFIG);
	ret = rtlmq_pci_commit(tp);
	if (ret)
		goto failed_after_write;

	value = readl(tp->mmio_addr + RTLMQ_MAR0);
	if (value != plan->mc_filter[0]) {
		ret = -EIO;
		goto failed_after_write;
	}
	value = readl(tp->mmio_addr + RTLMQ_MAR0 + sizeof(u32));
	if (value != plan->mc_filter[1]) {
		ret = -EIO;
		goto failed_after_write;
	}
	value = readl(tp->mmio_addr + RTLMQ_RX_CONFIG);
	/* Hardware owns the non-filter fields while the RX engine is live. */
	if ((value & RTLMQ_RX_CONFIG_ACCEPT_OK_MASK) != plan->rx_mode) {
		ret = -EIO;
		goto failed_after_write;
	}

	return 0;

failed_after_write:
	/*
	 * Writes may have reached the MAC even when the barrier/readback
	 * fails. OPEN has no rollback image: replay the current netdev lists
	 * through the lifecycle owner rather than claiming a filter commit.
	 * OPENING already aborts activation on this error.
	 */
	if (READ_ONCE(tp->state) == RTLMQ_STATE_OPEN)
		rtlmq_request_reset(tp, BIT(RTLMQ_RESET_REASON_RX_FILTER)
		);
	return ret;
}

int rtlmq_apply_rx_filter_plan_locked(struct rtlmq_private *tp,
				      const struct rtlmq_rx_filter_plan *plan)
{
	enum rtlmq_lifecycle_state state;
	int ret;

	if (!tp || !tp->ndev || !plan)
		return -EINVAL;
	lockdep_assert_held(&tp->control_lock);

	state = READ_ONCE(tp->state);
	if (state == RTLMQ_STATE_REMOVING || state == RTLMQ_STATE_DEAD)
		return -ENODEV;

	/*
	 * Do not program filters while down, resetting, or suspended/frozen.
	 * The netdev address lists retain the request; the next open rebuilds
	 * the register values from those lists.
	 */
	if (state == RTLMQ_STATE_PROBED ||
	    state == RTLMQ_STATE_CLOSING ||
	    state == RTLMQ_STATE_RESETTING ||
	    (state == RTLMQ_STATE_OPEN &&
	     !netif_device_present(tp->ndev)) ||
	    tp->platform_quiesce_owner != RTLMQ_PLATFORM_QUIESCE_NONE) {
		return 0;
	}

	if (state != RTLMQ_STATE_OPENING && state != RTLMQ_STATE_OPEN)
		return -EBUSY;

	ret = rtlmq_program_rx_filter_plan(tp, plan);
	if (ret)
		return ret;

	return 0;
}

static int rtlmq_build_current_rx_filter_plan(struct net_device *ndev,
					      struct rtlmq_rx_filter_plan *plan)
{
	int ret;

	netif_addr_lock_bh(ndev);
	ret = rtlmq_build_rx_filter_plan(ndev, &ndev->uc, &ndev->mc, plan);
	netif_addr_unlock_bh(ndev);

	return ret;
}

int rtlmq_update_rx_mode(struct rtlmq_private *tp)
{
	struct rtlmq_rx_filter_plan plan;
	int ret;

	if (!tp || !tp->ndev)
		return -EINVAL;
	ASSERT_RTNL();
	lockdep_assert_held(&tp->control_lock);

	ret = rtlmq_build_current_rx_filter_plan(tp->ndev, &plan);
	if (ret) {
		netdev_err(tp->ndev,
			   "failed to build receive filter plan: %d\n", ret);
		return ret;
	}

	ret = rtlmq_apply_rx_filter_plan_locked(tp, &plan);
	if (ret)
		netdev_err(tp->ndev,
			   "failed to apply receive filter plan: %d\n", ret);

	return ret;
}

int rtlmq_set_bootstrap_rx_mode(struct rtlmq_private *tp)
{
	u32 rx_config;

	if (!tp || !rtlmq_mmio_ordinary_authorized(tp))
		return -ENODEV;

	/*
	 * Bootstrap bring-up starts with a permissive receive filter:
	 * accept station, broadcast, and multicast traffic while preserving
	 * the current RX fetch/burst configuration.
	 */
	writel(~0U, tp->mmio_addr + RTLMQ_MAR0);
	writel(~0U, tp->mmio_addr + RTLMQ_MAR0 + sizeof(u32));

	if (rtlmq_read_rx_config(tp, &rx_config))
		return -ENODEV;
	rx_config &= ~RTLMQ_RX_CONFIG_ACCEPT_OK_MASK;
	rx_config |= RTLMQ_RX_ACCEPT_BROADCAST |
		     RTLMQ_RX_ACCEPT_MULTICAST |
		     RTLMQ_RX_ACCEPT_MY_PHYS;
	writel(rx_config, tp->mmio_addr + RTLMQ_RX_CONFIG);

	return rtlmq_pci_commit(tp);
}
