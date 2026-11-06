// SPDX-License-Identifier: GPL-2.0-only

#include <linux/delay.h>
#include <linux/etherdevice.h>
#include <linux/iopoll.h>
#include <linux/pci.h>
#include <linux/pci_ids.h>

#include "rtlmq_chip.h"
#include "rtlmq_diag.h"
#include "rtlmq_regs.h"
#include "rtlmq_hw.h"
#include "rtlmq_runtime_accessors.h"
#include "rtlmq_irq.h"

/* read_poll_timeout() sleeps for at least a quarter of this interval. */
#define RTLMQ_RESET_DRAIN_POLL_US		200
#define RTLMQ_RESET_DRAIN_TIMEOUT_US	150000

static int rtlmq_mod_reg8_cond(struct rtlmq_private *tp, u32 reg, u8 bits,
			       bool enable)
{
	u8 value;

	if (!rtlmq_mmio_ordinary_authorized(tp))
		return -ENODEV;

	value = readb(tp->mmio_addr + reg);
	if (value == U8_MAX)
		return -ENODEV;
	if (enable)
		value |= bits;
	else
		value &= ~bits;
	writeb(value, tp->mmio_addr + reg);
	return 0;
}

struct rtlmq_aspm_clkreq_state {
	u8 config2;
	u8 config5;
	bool valid;
};

static int rtlmq_hw_aspm_clkreq_enable(struct rtlmq_private *tp, bool enable,
				       struct rtlmq_aspm_clkreq_state *saved)
{
	u8 config2;
	u8 config5;

	if (!rtlmq_mmio_ordinary_authorized(tp))
		return -ENODEV;

	if (!enable) {
		config5 = readb(tp->mmio_addr + RTLMQ_CONFIG5);
		if (config5 == U8_MAX)
			return -ENODEV;
		config2 = readb(tp->mmio_addr + RTLMQ_CONFIG2);
		if (config2 == U8_MAX)
			return -ENODEV;
		if (saved) {
			saved->config2 = config2;
			saved->config5 = config5;
			saved->valid = true;
		}

		writeb(config5 & ~RTLMQ_CONFIG5_ASPM_ENABLE,
		       tp->mmio_addr + RTLMQ_CONFIG5);

		writeb(config2 & ~RTLMQ_CONFIG2_CLKREQ_ENABLE,
		       tp->mmio_addr + RTLMQ_CONFIG2);
	} else {
		if (!saved || !saved->valid)
			return -EINVAL;

		writeb(saved->config5, tp->mmio_addr + RTLMQ_CONFIG5);
		writeb(saved->config2, tp->mmio_addr + RTLMQ_CONFIG2);
	}
	config5 = readb(tp->mmio_addr + RTLMQ_CONFIG5);
	config2 = readb(tp->mmio_addr + RTLMQ_CONFIG2);
	if (config5 == U8_MAX || config2 == U8_MAX)
		return -ENODEV;
	if (!enable)
		return (config5 & RTLMQ_CONFIG5_ASPM_ENABLE ||
			config2 & RTLMQ_CONFIG2_CLKREQ_ENABLE) ? -EIO : 0;
	return ((config5 ^ saved->config5) & RTLMQ_CONFIG5_ASPM_ENABLE ||
		(config2 ^ saved->config2) & RTLMQ_CONFIG2_CLKREQ_ENABLE) ?
	       -EIO : 0;
}

static int rtlmq_wait_rxdv_gate_ready(struct rtlmq_private *tp);
static int rtlmq_wait_link_list_ready(struct rtlmq_private *tp);
static u8 rtlmq_live_all_engine_stop_chipcmd(u8 chipcmd);

static int rtlmq_enable_rxdvgate_strict(struct rtlmq_private *tp)
{
	u32 misc;

	if (!rtlmq_mmio_ordinary_authorized(tp))
		return -ENODEV;

	misc = readl(tp->mmio_addr + RTLMQ_MISC);
	if (misc == U32_MAX)
		return -ENODEV;
	writel(misc | RTLMQ_RXDV_GATED_EN,
	       tp->mmio_addr + RTLMQ_MISC);

	return rtlmq_wait_rxdv_gate_ready(tp);
}

static int rtlmq_enable_rxdvgate(struct rtlmq_private *tp)
{
	int ret = rtlmq_enable_rxdvgate_strict(tp);

	/*
	 * Keep full reset behavior compatible with the vendor driver. It treats
	 * the FIFO drain as best effort before CmdReset. XSK queue replacement
	 * uses the strict drain because old DMA mappings cannot be released until
	 * it completes.
	 */
	return ret == -ENODEV ? ret : 0;
}

int rtlmq_disable_rxdvgate(struct rtlmq_private *tp)
{
	u32 misc;
	int ret;

	ret = rtlmq_clear_rxdv_gate(tp);
	if (ret)
		return ret;

	/* PPSW[3] aliases MISC[19]; confirm the dword view before RX. */
	misc = readl(tp->mmio_addr + RTLMQ_MISC);
	if (misc == U32_MAX)
		return -ENODEV;
	writel(misc & ~RTLMQ_RXDV_GATED_EN,
	       tp->mmio_addr + RTLMQ_MISC);
	misc = readl(tp->mmio_addr + RTLMQ_MISC);
	if (misc == U32_MAX)
		return -ENODEV;
	return misc & RTLMQ_RXDV_GATED_EN ? -EIO : 0;
}

static u8 rtlmq_live_stop_request_chipcmd(u8 chipcmd)
{
	/*
	 * Set StopReq before disabling either engine. This stops new work while
	 * leaving descriptor handling intact long enough for the RTL8125 FIFOs to
	 * drain.
	 */
	return chipcmd | RTLMQ_CHIP_CMD_STOP_REQ;
}

static u8 rtlmq_reset_clear_stop_request_chipcmd(u8 chipcmd)
{
	return chipcmd & (RTLMQ_CHIP_CMD_TX_ENABLE |
			  RTLMQ_CHIP_CMD_RX_ENABLE);
}

int rtlmq_prepare_mac_reset(struct rtlmq_private *tp, const char *where)
{
	u16 intrmit = 0;
	u8 chipcmd;
	u8 mcu = 0;
	u8 ppsw;
	int ret;

	if (!tp || !rtlmq_mmio_ordinary_authorized(tp))
		return -ENODEV;

	/*
	 * Before CmdReset, block packet admission with PPSW RXDV and StopReq,
	 * then attempt FIFO and engine drain. Drain timeouts are logged but do
	 * not block reset. Live ring replacement instead uses MISC RXDV gating
	 * and strict engine-off checks before retiring old DMA mappings.
	 */
	ret = rtlmq_clear_rx_acceptance(tp);
	if (ret)
		return ret;

	ppsw = readb(tp->mmio_addr + RTLMQ_PPSW);
	if (ppsw == U8_MAX)
		return -ENODEV;
	writeb(ppsw | RTLMQ_PPSW_RXDV_GATED_EN,
	       tp->mmio_addr + RTLMQ_PPSW);

	chipcmd = readb(tp->mmio_addr + RTLMQ_CHIP_CMD);
	if (chipcmd == U8_MAX)
		return -ENODEV;
	writeb(rtlmq_live_stop_request_chipcmd(chipcmd),
	       tp->mmio_addr + RTLMQ_CHIP_CMD);
	fsleep(200);
	chipcmd = readb(tp->mmio_addr + RTLMQ_CHIP_CMD);
	if (chipcmd == U8_MAX)
		return -ENODEV;

	{
		ret = read_poll_timeout(readb, mcu,
					mcu == U8_MAX ||
					(mcu & RTLMQ_MCU_RXTX_EMPTY) == RTLMQ_MCU_RXTX_EMPTY,
					RTLMQ_RESET_DRAIN_POLL_US, RTLMQ_RESET_DRAIN_TIMEOUT_US,
					true, tp->mmio_addr + RTLMQ_MCU);
	}
	if (mcu == U8_MAX)
		return -ENODEV;
	if (ret) {
		netdev_err(tp->ndev,
			   "reset admission %s FIFO drain timeout chipcmd=%#02x mcu=%#02x intrmit=%#04x\n",
			   where ? where : "unknown", chipcmd, mcu,
			   readw(tp->mmio_addr + RTLMQ_INTR_MITIGATE));
	}

	{
		ret = read_poll_timeout(readw, intrmit,
					intrmit == U16_MAX || (intrmit & 0x0103) == 0x0103,
					RTLMQ_RESET_DRAIN_POLL_US, RTLMQ_RESET_DRAIN_TIMEOUT_US,
					true, tp->mmio_addr + RTLMQ_INTR_MITIGATE);
	}
	if (intrmit == U16_MAX)
		return -ENODEV;
	if (ret) {
		netdev_err(tp->ndev,
			   "reset admission %s engine idle timeout chipcmd=%#02x mcu=%#02x intrmit=%#04x\n",
			   where ? where : "unknown", chipcmd, mcu, intrmit);
	}

	chipcmd = readb(tp->mmio_addr + RTLMQ_CHIP_CMD);
	if (chipcmd == U8_MAX)
		return -ENODEV;
	writeb(rtlmq_reset_clear_stop_request_chipcmd(chipcmd),
	       tp->mmio_addr + RTLMQ_CHIP_CMD);
	chipcmd = readb(tp->mmio_addr + RTLMQ_CHIP_CMD);
	if (chipcmd == U8_MAX)
		return -ENODEV;
	if (chipcmd & RTLMQ_CHIP_CMD_STOP_REQ) {
		netdev_err(tp->ndev,
			   "reset admission %s failed to clear StopReq chipcmd=%#02x\n",
			   where ? where : "unknown", chipcmd);
	}

	return 0;
}

static int rtlmq_disable_mac_engines_after_drain(struct rtlmq_private *tp)
{
	u16 intrmit = 0;
	unsigned int i;
	u8 mcu = 0;
	u8 chipcmd;

	if (!rtlmq_mmio_ordinary_authorized(tp))
		return -ENODEV;

	/*
	 * RTL8125 ties the RX descriptor and link-list logic to both ChipCmd engine
	 * enables. After RXDV and StopReq drain, stop RX and TX, wait for the
	 * hardware to settle, then verify MCU and IntrMitigate report empty. PCIe
	 * Transactions Pending is not enough because memory writes can be posted.
	 */
	chipcmd = readb(tp->mmio_addr + RTLMQ_CHIP_CMD);
	if (chipcmd == U8_MAX)
		return -ENODEV;
	writeb(rtlmq_live_all_engine_stop_chipcmd(chipcmd),
	       tp->mmio_addr + RTLMQ_CHIP_CMD);
	if (readb_poll_timeout(tp->mmio_addr + RTLMQ_CHIP_CMD, chipcmd,
			       chipcmd == U8_MAX ||
			       !(chipcmd & (RTLMQ_CHIP_CMD_TX_ENABLE |
					    RTLMQ_CHIP_CMD_RX_ENABLE)),
			       20, 20000))
		return -ETIMEDOUT;
	if (chipcmd == U8_MAX)
		return -ENODEV;

	fsleep(1000);
	for (i = 0; i < 3000; i++) {
		mcu = readb(tp->mmio_addr + RTLMQ_MCU);
		intrmit = readw(tp->mmio_addr + RTLMQ_INTR_MITIGATE);
		chipcmd = readb(tp->mmio_addr + RTLMQ_CHIP_CMD);
		if (mcu == U8_MAX || intrmit == U16_MAX ||
		    chipcmd == U8_MAX)
			return -ENODEV;
		if (!(chipcmd & (RTLMQ_CHIP_CMD_TX_ENABLE |
				 RTLMQ_CHIP_CMD_RX_ENABLE)) &&
		    (mcu & RTLMQ_MCU_RXTX_EMPTY) ==
			    RTLMQ_MCU_RXTX_EMPTY &&
		    (intrmit & 0x0103) == 0x0103)
			return 0;
		fsleep(50);
	}

	netdev_warn(tp->ndev,
		    "RTL8125 engine-off quiescence timeout chipcmd=%#02x mcu=%#02x intrmit=%#04x misc=%#08x\n",
		    chipcmd, mcu, intrmit,
		    readl(tp->mmio_addr + RTLMQ_MISC));
	return -ETIMEDOUT;
}

static u8 rtlmq_live_all_engine_stop_chipcmd(u8 chipcmd)
{
	return chipcmd & ~(RTLMQ_CHIP_CMD_TX_ENABLE |
			   RTLMQ_CHIP_CMD_RX_ENABLE);
}

static u8 rtlmq_live_rx_engine_stop_chipcmd(u8 chipcmd)
{
	/*
	 * After the drain, an XSK memory-model change clears only RX enable.
	 * Leave TX enabled so its descriptors and hardware consumer position
	 * are not disturbed.
	 */
	return chipcmd & ~RTLMQ_CHIP_CMD_RX_ENABLE;
}

static int rtlmq_disable_rx_engine_after_drain(struct rtlmq_private *tp)
{
	u8 stopped_chipcmd;
	u16 intrmit = 0;
	unsigned int i;
	u8 mcu = 0;
	u8 chipcmd;

	if (!rtlmq_mmio_ordinary_authorized(tp))
		return -ENODEV;

	chipcmd = readb(tp->mmio_addr + RTLMQ_CHIP_CMD);
	if (chipcmd == U8_MAX)
		return -ENODEV;
	stopped_chipcmd = rtlmq_live_rx_engine_stop_chipcmd(chipcmd);
	writeb(stopped_chipcmd, tp->mmio_addr + RTLMQ_CHIP_CMD);
	if (readb_poll_timeout(tp->mmio_addr + RTLMQ_CHIP_CMD, chipcmd,
			       chipcmd == U8_MAX ||
			       (!(chipcmd & RTLMQ_CHIP_CMD_RX_ENABLE) &&
				(chipcmd & RTLMQ_CHIP_CMD_TX_ENABLE) ==
				(stopped_chipcmd &
				 RTLMQ_CHIP_CMD_TX_ENABLE)),
			       20, 20000))
		return -ETIMEDOUT;
	if (chipcmd == U8_MAX)
		return -ENODEV;

	fsleep(1000);
	for (i = 0; i < 3000; i++) {
		mcu = readb(tp->mmio_addr + RTLMQ_MCU);
		intrmit = readw(tp->mmio_addr + RTLMQ_INTR_MITIGATE);
		chipcmd = readb(tp->mmio_addr + RTLMQ_CHIP_CMD);
		if (mcu == U8_MAX || intrmit == U16_MAX ||
		    chipcmd == U8_MAX)
			return -ENODEV;
		if (!(chipcmd & RTLMQ_CHIP_CMD_RX_ENABLE) &&
		    (chipcmd & RTLMQ_CHIP_CMD_TX_ENABLE) ==
			    (stopped_chipcmd & RTLMQ_CHIP_CMD_TX_ENABLE) &&
		    (mcu & RTLMQ_MCU_RXTX_EMPTY) ==
			    RTLMQ_MCU_RXTX_EMPTY &&
		    (intrmit & 0x0103) == 0x0103)
			return 0;
		fsleep(50);
	}

	netdev_warn(tp->ndev,
		    "RTL8125 RX-engine-off quiescence timeout chipcmd=%#02x mcu=%#02x intrmit=%#04x misc=%#08x\n",
		    chipcmd, mcu, intrmit,
		    readl(tp->mmio_addr + RTLMQ_MISC));
	return -ETIMEDOUT;
}

static int rtlmq_reinitialize_link_list_after_drain(struct rtlmq_private *tp)
{
	int ret;
	u8 mcu;

	ret = rtlmq_disable_mac_engines_after_drain(tp);
	if (ret)
		return ret;

	mcu = readb(tp->mmio_addr + RTLMQ_MCU);
	if (mcu == U8_MAX)
		return -ENODEV;
	writeb(mcu & ~RTLMQ_MCU_NOW_IS_OOB,
	       tp->mmio_addr + RTLMQ_MCU);

	ret = rtlmq_mac_ocp_modify_checked(tp, RTLMQ_MAC_OCP_LINK_LIST_CTRL,
					   RTLMQ_MAC_OCP_LINK_LIST_OOB, 0);
	if (ret)
		return ret;
	ret = rtlmq_wait_link_list_ready(tp);
	if (ret)
		return ret;

	ret = rtlmq_mac_ocp_write_checked(tp, RTLMQ_MAC_OCP_FIFO_NE, 0x07d0);
	if (ret)
		return ret;
	ret = rtlmq_mac_ocp_write_checked(tp, RTLMQ_MAC_OCP_FIFO_NF, 0x0150);
	if (ret)
		return ret;
	ret = rtlmq_mac_ocp_write_checked(tp, RTLMQ_MAC_OCP_LINK_LIST_REINIT,
					  0x5555);
	if (ret)
		return ret;

	ret = rtlmq_wait_link_list_ready(tp);
	if (ret)
		return ret;

	return 0;
}

int rtlmq_pause_mac_datapath_locked(struct rtlmq_private *tp)
{
	int ret;

	if (!tp)
		return -EINVAL;
	lockdep_assert_held(&tp->control_lock);

	/*
	 * StopReq and RXDV gating prevent new work. Stop RX and wait for RTL8125 to
	 * report empty before releasing the old RX mappings, since hardware may
	 * have prefetched buffer addresses. Leave TX enabled and preserve PHY,
	 * carrier, and queue topology.
	 */
	ret = rtlmq_enable_rxdvgate_strict(tp);
	if (ret)
		return ret;

	return rtlmq_disable_rx_engine_after_drain(tp);
}

int rtlmq_pause_all_mac_datapath_locked(struct rtlmq_private *tp)
{
	int ret;

	if (!tp)
		return -EINVAL;
	lockdep_assert_held(&tp->control_lock);

	/*
	 * Replacing both RX and TX rings means neither hardware cursor can be kept.
	 * Stop new work with StopReq and RXDV, drain TX, then disable both engines
	 * and require RX/TX-empty readback.
	 * This is a live ring rebuild, so leave PHY, carrier, link-list setup, and
	 * queue configuration alone.
	 */
	ret = rtlmq_enable_rxdvgate_strict(tp);
	if (ret)
		return ret;

	return rtlmq_disable_mac_engines_after_drain(tp);
}

int rtlmq_resume_mac_datapath_locked(struct rtlmq_private *tp,
				     const struct rtlmq_runtime_config *cfg)
{
	int ret;

	if (!tp || !cfg)
		return -EINVAL;
	lockdep_assert_held(&tp->control_lock);

	/*
	 * Reassert queue counts and the selected descriptor format after stopping
	 * RX, before re-enabling it. An RX-only pause leaves TX enabled;
	 * this helper also handles the both-engines-stopped ring rebuild.
	 */
	ret = rtlmq_program_queue_state(tp, cfg);
	if (ret)
		return ret;

	ret = rtlmq_disable_rxdvgate(tp);
	if (ret)
		return ret;
	return rtlmq_hw_enable_datapath(tp);
}

static int rtlmq_wait_rxdv_gate_ready(struct rtlmq_private *tp)
{
	unsigned int i;
	u8 mcu = 0;
	u8 chipcmd;

	if (!rtlmq_mmio_ordinary_authorized(tp))
		return -ENODEV;

	chipcmd = readb(tp->mmio_addr + RTLMQ_CHIP_CMD);
	if (chipcmd == U8_MAX)
		return -ENODEV;
	writeb(rtlmq_live_stop_request_chipcmd(chipcmd),
	       tp->mmio_addr + RTLMQ_CHIP_CMD);

	fsleep(2000);

	/*
	 * StopReq must let TX drain so its hardware cursor stays valid. RX can still
	 * hold a packet when an XSK fill ring is empty and RDU is set; that packet
	 * cannot complete against the old buffers. The later engine-off check must
	 * therefore confirm RX and TX are empty before old DMA mappings are
	 * released. At this point RX-empty is only useful for debugging.
	 */
	for (i = 0; i < 3000; i++) {
		mcu = readb(tp->mmio_addr + RTLMQ_MCU);
		if (mcu == U8_MAX)
			return -ENODEV;
		if (mcu & RTLMQ_MCU_TX_EMPTY)
			break;
		fsleep(50);
	}

	if (i == 3000) {
		netdev_warn(tp->ndev,
			    "rxdv TX drain timeout chipcmd=%#02x mcu=%#02x misc=%#08x\n",
			    readb(tp->mmio_addr + RTLMQ_CHIP_CMD), mcu,
			    readl(tp->mmio_addr + RTLMQ_MISC));
		return -ETIMEDOUT;
	}

	return 0;
}

static int rtlmq_wait_link_list_ready(struct rtlmq_private *tp)
{
	unsigned int i;
	u8 mcu = 0;

	if (!tp || !rtlmq_mmio_ordinary_authorized(tp))
		return -ENODEV;

	for (i = 0; i < 100; i++) {
		mcu = readb(tp->mmio_addr + RTLMQ_MCU);
		if (mcu == U8_MAX)
			return -ENODEV;
		if (mcu & RTLMQ_MCU_LINK_LIST_RDY)
			return 0;
		fsleep(42);
	}

	netdev_err(tp->ndev,
		   "link_list_ready timeout mcu=%#02x chipcmd=%#02x misc=%#08x e8de=%#04x e00e=%#04x\n",
		   mcu,
		   readb(tp->mmio_addr + RTLMQ_CHIP_CMD),
		   readl(tp->mmio_addr + RTLMQ_MISC),
		   rtlmq_mac_ocp_read(tp, RTLMQ_MAC_OCP_LINK_LIST_CTRL),
		   rtlmq_mac_ocp_read(tp, 0xe00e));

	return -ETIMEDOUT;
}

int rtlmq_wait_mac_ocp_e00e_clear(struct rtlmq_private *tp)
{
	unsigned int i;
	u16 fifo = 0;

	if (!tp || !rtlmq_mmio_ordinary_authorized(tp))
		return -ENODEV;

	for (i = 0; i < 1000; i++) {
		fifo = rtlmq_mac_ocp_read(tp, 0xe00e);
		if (fifo == U16_MAX)
			return -ENODEV;
		if (!(fifo & BIT(13)))
			return 0;
		udelay(10);
	}

	netdev_err(tp->ndev,
		   "mac_ocp_e00e_clear timeout e00e=%#04x chipcmd=%#02x mcu=%#02x misc=%#08x\n",
		   fifo, readb(tp->mmio_addr + RTLMQ_CHIP_CMD),
		   readb(tp->mmio_addr + RTLMQ_MCU),
		   readl(tp->mmio_addr + RTLMQ_MISC));

	return -ETIMEDOUT;
}

enum rtlmq_8125_tx_engine_stage {
	RTLMQ_8125_TX_ENGINE_BOOTSTRAP,
	RTLMQ_8125_TX_ENGINE_RUNTIME_LATCH,
};

static int
rtlmq_program_8125_vendor_tx_engine(struct rtlmq_private *tp,
				    enum rtlmq_8125_tx_engine_stage stage)
{
	int ret;

	if (!rtlmq_mmio_ordinary_authorized(tp))
		return -ENODEV;
	if (!rtlmq_hw_requires_tx_engine_runtime_latch(tp))
		return 0;

	ret = rtlmq_prepare_tx_engine_fifo(tp, stage == RTLMQ_8125_TX_ENGINE_RUNTIME_LATCH);
	if (ret)
		return ret;
	ret = rtlmq_mac_ocp_modify_checked(tp, RTLMQ_MAC_OCP_VENDOR_EA1C,
					   0x0004, 0x0000);
	if (ret)
		return ret;
	/*
	 * Match r8125 CFG_METHOD_4/5: write the full 0x4400 value,
	 * including bit 10, rather than modifying an inherited value.
	 */
	ret = rtlmq_mac_ocp_write_checked(tp, RTLMQ_MAC_OCP_VENDOR_E0C0,
					  RTLMQ_MAC_OCP_VENDOR_E0C0_8125B);
	if (ret)
		return ret;
	/* r8125 sets bits 6 and 5, and clears bits 7 and 3. */
	ret = rtlmq_mac_ocp_modify_checked(tp, RTLMQ_MAC_OCP_VENDOR_TX_ENGINE_E052, 0x0088, 0x0060);
	if (ret)
		return ret;
	ret = rtlmq_mac_ocp_modify_checked(tp, RTLMQ_MAC_OCP_VENDOR_TX_ENGINE_D430, 0x0fff, 0x047f);
	if (ret)
		return ret;
	ret = rtlmq_mac_ocp_modify_checked(tp, RTLMQ_MAC_OCP_VENDOR_E080,
					   BIT(1), 0);
	if (ret)
		return ret;
	ret = rtlmq_mac_ocp_modify_checked(tp, RTLMQ_MAC_OCP_VENDOR_EA1C,
					   0x0004, 0x0000);
	if (ret)
		return ret;
	return rtlmq_finish_tx_engine_fifo(tp);
}

int rtlmq_apply_wol(struct rtlmq_private *tp)
{
	u32 supported = rtlmq_get_supported_wol_opts(tp);
	u32 wol_opts = tp->wol_opts;
	u8 pmch;
	int ret;

	lockdep_assert_held(&tp->control_lock);

	if (wol_opts & ~supported)
		return -EINVAL;

	/* Down-state requests can be staged while runtime PM owns the device. */
	if (!rtlmq_mmio_ordinary_authorized(tp) || !supported)
		goto out_wakeup;

	rtlmq_unlock_config_regs(tp);
	ret = rtlmq_mac_ocp_modify_checked(tp, RTLMQ_WOL_MAGIC_OCP_REG, RTLMQ_WOL_MAGIC_BIT,
					   wol_opts & WAKE_MAGIC ? RTLMQ_WOL_MAGIC_BIT : 0);
	if (ret)
		goto out_lock;
	ret = rtlmq_mod_reg8_cond(tp, RTLMQ_CONFIG3,
				  RTLMQ_CONFIG3_LINK_UP, wol_opts & WAKE_PHY);
	if (ret)
		goto out_lock;
	if (wol_opts & WAKE_PHY) {
		ret = rtlmq_mac_ocp_modify_checked(tp, RTLMQ_WOL_LINK_OCP_REG,
						   RTLMQ_WOL_LINK_MASK,
						   RTLMQ_WOL_LINK_BITS);
	} else if (rtlmq_qualified_wol_link_v3(tp)) {
		int value = rtlmq_mac_ocp_read_checked(tp, RTLMQ_WOL_LINK_OCP_REG);

		if (value < 0) {
			ret = value;
			goto out_lock;
		}
		if (!(value & BIT(0)))
			ret = rtlmq_mac_ocp_modify_checked(tp, RTLMQ_WOL_LINK_OCP_REG,
							   BIT(5) | BIT(3) | BIT(2) | BIT(1),
							   BIT(4));
	} else {
		ret = rtlmq_mac_ocp_modify_checked(tp, RTLMQ_WOL_LINK_OCP_REG,
						   RTLMQ_WOL_LINK_MASK, 0);
	}
	if (ret)
		goto out_lock;
	ret = rtlmq_mod_reg8_cond(tp, RTLMQ_CONFIG5, RTLMQ_CONFIG5_UWF,
				  wol_opts & WAKE_UCAST);
	if (ret)
		goto out_lock;
	ret = rtlmq_mod_reg8_cond(tp, RTLMQ_CONFIG5, RTLMQ_CONFIG5_BWF,
				  wol_opts & WAKE_BCAST);
	if (ret)
		goto out_lock;
	ret = rtlmq_mod_reg8_cond(tp, RTLMQ_CONFIG5, RTLMQ_CONFIG5_MWF,
				  wol_opts & WAKE_MCAST);
	if (ret)
		goto out_lock;
	ret = rtlmq_mod_reg8_cond(tp, RTLMQ_CONFIG5, RTLMQ_CONFIG5_LAN_WAKE,
				  wol_opts);
	if (ret)
		goto out_lock;
	ret = rtlmq_mod_reg8_cond(tp, RTLMQ_CONFIG2, RTLMQ_CONFIG2_PME_SIGNAL,
				  wol_opts);
	if (ret)
		goto out_lock;
	rtlmq_lock_config_regs(tp);

	/* K1 rtl_set_d3_pll_down(): retain D3hot clocks; D3cold follows WoL. */
	pmch = readb(tp->mmio_addr + RTLMQ_PMCH);
	if (pmch == U8_MAX)
		return -ENODEV;
	pmch |= RTLMQ_PMCH_D3HOT_NO_PLL_DOWN;
	if (wol_opts)
		pmch |= RTLMQ_PMCH_D3COLD_NO_PLL_DOWN;
	else
		pmch &= ~RTLMQ_PMCH_D3COLD_NO_PLL_DOWN;
	writeb(pmch, tp->mmio_addr + RTLMQ_PMCH);

	netif_info(tp, wol, tp->ndev, "applied wake-on-LAN options %#x\n",
		   wol_opts);

out_wakeup:
	if (tp->pdev) {
		struct device *dev = &tp->pdev->dev;
		bool was_enabled = device_may_wakeup(dev);

		/*
		 * pci_enable_wake() rejects enable before device_may_wakeup().
		 * Replays must reuse the wakeup source instead of allocating a
		 * duplicate, which device_wakeup_attach() rejects with EEXIST.
		 */
		if (was_enabled != !!wol_opts) {
			ret = device_set_wakeup_enable(dev, !!wol_opts);
			if (ret)
				return ret;
		}
		{
			ret = pci_wake_from_d3(tp->pdev, !!wol_opts);
		}
		if (ret && !was_enabled && wol_opts)
			device_set_wakeup_enable(dev, false);
		return ret;
	}
	return 0;

out_lock:
	if (ret != -ENODEV)
		rtlmq_lock_config_regs(tp);
	return ret;
}

int rtlmq_prepare_wol_suspend(struct rtlmq_private *tp)
{
	const struct rtlmq_hw_contract *hw = rtlmq_chip_profile_hw(tp->profile);
	u32 wol_opts = tp->wol_opts;
	u32 rx_config;
	int ret;

	lockdep_assert_held(&tp->control_lock);

	if (wol_opts & ~rtlmq_get_supported_wol_opts(tp))
		return -EINVAL;
	if (!wol_opts)
		return 0;
	if (!rtlmq_mmio_ordinary_authorized(tp))
		return -ENODEV;

	rtlmq_mask_and_ack_intrs(tp);
	ret = rtlmq_apply_wol(tp);
	if (ret)
		return ret;

	if (hw && hw->mac_version == RTLMQ_MAC_8126A) {
		ret = rtlmq_prepare_d3_8126(tp);
		if (ret)
			return ret;
	}

	ret = rtlmq_read_rx_config(tp, &rx_config);
	if (ret)
		return ret;
	rx_config &= ~RTLMQ_RX_CONFIG_ACCEPT_OK_MASK;
	rx_config |= RTLMQ_RX_ACCEPT_BROADCAST |
		     RTLMQ_RX_ACCEPT_MULTICAST |
		     RTLMQ_RX_ACCEPT_MY_PHYS;
	writel(rx_config, tp->mmio_addr + RTLMQ_RX_CONFIG);

	/*
	 * The wake matcher needs RX filtering and an open RX-DV gate, not the
	 * descriptor engine. Close has already withdrawn and freed its rings.
	 */
	ret = rtlmq_disable_rxdvgate(tp);
	if (ret)
		return ret;

	return 0;
}

int rtlmq_prepare_no_wake_suspend(struct rtlmq_private *tp)
{
	const struct rtlmq_hw_contract *hw = rtlmq_chip_profile_hw(tp->profile);
	int ret;

	lockdep_assert_held(&tp->control_lock);
	if (!hw || hw->mac_version != RTLMQ_MAC_8126A)
		return 0;
	if (tp->wol_opts)
		return -EINVAL;
	if (!rtlmq_mmio_ordinary_authorized(tp))
		return -ENODEV;

	/* Reassert the no-wake PLL policy before relinquishing register access. */
	ret = rtlmq_apply_wol(tp);
	if (ret)
		return ret;
	return rtlmq_prepare_d3_8126(tp);
}

int rtlmq_hw_prepare_8125(struct rtlmq_private *tp)
{
	int ret;

	if (!rtlmq_mmio_ordinary_authorized(tp))
		return -ENODEV;

	ret = rtlmq_enable_rxdvgate(tp);
	if (ret)
		return ret;

	ret = rtlmq_reinitialize_link_list_after_drain(tp);
	if (ret) {
		netdev_err(tp->ndev,
			   "hw_prepare_8125 link-list reinit failed chipcmd=%#02x mcu=%#02x misc=%#08x e8de=%#04x e00e=%#04x c0aa=%#04x c0a6=%#04x c01e=%#04x\n",
			   readb(tp->mmio_addr + RTLMQ_CHIP_CMD),
			   readb(tp->mmio_addr + RTLMQ_MCU),
			   readl(tp->mmio_addr + RTLMQ_MISC),
			   rtlmq_mac_ocp_read(tp, RTLMQ_MAC_OCP_LINK_LIST_CTRL),
			   rtlmq_mac_ocp_read(tp, 0xe00e),
			   rtlmq_mac_ocp_read(tp, RTLMQ_MAC_OCP_FIFO_NE),
			   rtlmq_mac_ocp_read(tp, RTLMQ_MAC_OCP_FIFO_NF),
			   rtlmq_mac_ocp_read(tp, RTLMQ_MAC_OCP_LINK_LIST_REINIT));
		return ret;
	}

	/* Preserve the RTL8125 link-list-before-reset ordering. */
	return rtlmq_hw_reset(tp, "hw_init");
}

int rtlmq_set_eee_tx_lpi_locked(struct rtlmq_private *tp, bool enable)
{
	const struct rtlmq_hw_contract *hw;
	bool aux_programmed = false;
	int restore_aux = 0;
	u16 expected_ctrl;
	u16 expected_aux;
	int restore_ctrl;
	bool uses_aux;
	u16 old_ctrl;
	u16 old_aux;
	int ctrl;
	int aux;
	int ret;

	lockdep_assert_held(&tp->control_lock);

	if (!rtlmq_qualified_supports_eee(tp))
		return 0;
	if (!rtlmq_mmio_ordinary_authorized(tp))
		return -ENODEV;

	hw = rtlmq_selected_hw_contract(tp);
	if (!hw)
		return -EOPNOTSUPP;
	switch (hw->eee_tx_lpi_layout) {
	case RTLMQ_EEE_TX_LPI_E040:
		uses_aux = false;
		break;
	case RTLMQ_EEE_TX_LPI_E040_EB62:
		uses_aux = true;
		break;
	default:
		return -EOPNOTSUPP;
	}

	ctrl = rtlmq_mac_ocp_read_checked(tp, RTLMQ_MAC_OCP_VENDOR_E040);
	if (ctrl < 0)
		return ctrl;
	old_ctrl = ctrl & RTLMQ_MAC_OCP_EEE_TX_LPI_MASK;
	old_aux = 0;
	if (uses_aux) {
		aux = rtlmq_mac_ocp_read_checked(tp,
						 RTLMQ_MAC_OCP_VENDOR_EB62);
		if (aux < 0)
			return aux;
		old_aux = aux & RTLMQ_MAC_OCP_EEE_TX_LPI_AUX_MASK;
	}

	expected_ctrl = enable ? RTLMQ_MAC_OCP_EEE_TX_LPI_MASK : 0;
	expected_aux = enable ? RTLMQ_MAC_OCP_EEE_TX_LPI_AUX_MASK : 0;
	ret = rtlmq_mac_ocp_modify_checked(tp, RTLMQ_MAC_OCP_VENDOR_E040,
					   RTLMQ_MAC_OCP_EEE_TX_LPI_MASK,
					   expected_ctrl);
	if (ret)
		return ret;
	if (uses_aux) {
		ret = rtlmq_mac_ocp_modify_checked(tp,
						   RTLMQ_MAC_OCP_VENDOR_EB62,
						   RTLMQ_MAC_OCP_EEE_TX_LPI_AUX_MASK,
						   expected_aux);
		if (ret)
			goto rollback;
		aux_programmed = true;
	}
	ctrl = rtlmq_mac_ocp_read_checked(tp, RTLMQ_MAC_OCP_VENDOR_E040);
	aux = uses_aux ?
		rtlmq_mac_ocp_read_checked(tp, RTLMQ_MAC_OCP_VENDOR_EB62) : 0;
	if (ctrl < 0) {
		ret = ctrl;
		goto rollback;
	}
	if (aux < 0) {
		ret = aux;
		goto rollback;
	}
	if ((ctrl & RTLMQ_MAC_OCP_EEE_TX_LPI_MASK) == expected_ctrl &&
	    (!uses_aux ||
	     (aux & RTLMQ_MAC_OCP_EEE_TX_LPI_AUX_MASK) == expected_aux))
		return 0;
	ret = -EIO;

rollback:
	if (aux_programmed)
		restore_aux = rtlmq_mac_ocp_modify_checked(tp,
							   RTLMQ_MAC_OCP_VENDOR_EB62,
							   RTLMQ_MAC_OCP_EEE_TX_LPI_AUX_MASK,
							   old_aux);
	restore_ctrl = rtlmq_mac_ocp_modify_checked(tp,
						    RTLMQ_MAC_OCP_VENDOR_E040,
						    RTLMQ_MAC_OCP_EEE_TX_LPI_MASK,
						    old_ctrl);
	if (restore_aux == -ENODEV || restore_ctrl == -ENODEV)
		return -ENODEV;
	if (restore_aux || restore_ctrl)
		return -EUCLEAN;

	ctrl = rtlmq_mac_ocp_read_checked(tp, RTLMQ_MAC_OCP_VENDOR_E040);
	aux = uses_aux ?
		rtlmq_mac_ocp_read_checked(tp, RTLMQ_MAC_OCP_VENDOR_EB62) : 0;
	if (ctrl == -ENODEV || aux == -ENODEV)
		return -ENODEV;
	if (ctrl < 0 || aux < 0 ||
	    (ctrl & RTLMQ_MAC_OCP_EEE_TX_LPI_MASK) != old_ctrl ||
	    (uses_aux &&
	     (aux & RTLMQ_MAC_OCP_EEE_TX_LPI_AUX_MASK) != old_aux))
		return -EUCLEAN;

	return ret;
}

int rtlmq_program_eee_mac_locked(struct rtlmq_private *tp)
{
	u16 timer;
	u16 value;

	lockdep_assert_held(&tp->control_lock);

	if (!rtlmq_qualified_supports_eee(tp))
		return 0;
	if (!tp->ndev || !rtlmq_mmio_ordinary_authorized(tp))
		return -ENODEV;

	timer = rtlmq_eee_tx_lpi_timer_bytes(tp->ndev);
	writew(timer, tp->mmio_addr + RTLMQ_EEE_TXIDLE_TIMER);
	value = readw(tp->mmio_addr + RTLMQ_EEE_TXIDLE_TIMER);
	if (value == U16_MAX)
		return -ENODEV;
	if (value != timer)
		return -EIO;

	/*
	 * A reset or previous driver may leave the MAC LPI bits out of sync with
	 * phylib. Start with MAC LPI disabled. The link callback enables it only
	 * when phylib reports EEE active and enable_tx_lpi is set.
	 */
	return rtlmq_set_eee_tx_lpi_locked(tp, false);
}

int rtlmq_hw_config_8125_common(struct rtlmq_private *tp)
{
	struct rtlmq_aspm_clkreq_state pm_state = {};
	int ret, restore_ret;

	if (!tp || !rtlmq_mmio_ordinary_authorized(tp))
		return -ENODEV;

	ret = rtlmq_pcie_state_l2l3_disable(tp);
	if (ret)
		return ret;
	ret = rtlmq_hw_aspm_clkreq_enable(tp, false, &pm_state);
	if (ret) {
		if (pm_state.valid) {
			restore_ret = rtlmq_hw_aspm_clkreq_enable(tp, true,
								  &pm_state);
			if (restore_ret)
				netdev_warn(tp->ndev,
					    "failed to restore ASPM/CLKREQ after setup error: %d\n",
					    restore_ret);
		}
		return ret;
	}

	writew(0x221b, tp->mmio_addr + RTLMQ_8125_MAC_CFG);
	ret = rtlmq_disable_double_vlan(tp);
	if (ret)
		goto out_restore_pm;
	writel(0, tp->mmio_addr + RTLMQ_RSS_CTRL);
	writew(0, tp->mmio_addr + RTLMQ_Q_NUM_CTRL);

	ret = rtlmq_mac_ocp_modify_checked(tp, 0xd40a, 0x0010, 0x0000);
	if (ret)
		goto out_restore_pm;

	ret = rtlmq_configure_mac_common(tp);
	if (ret)
		goto out_restore_pm;

	ret = rtlmq_mac_ocp_modify_checked(tp, 0xd3e2, 0x0fff, 0x03a9);
	if (ret)
		goto out_restore_pm;
	ret = rtlmq_mac_ocp_modify_checked(tp, 0xd3e4, 0x00ff, 0x0000);
	if (ret)
		goto out_restore_pm;
	ret = rtlmq_mac_ocp_modify_checked(tp, 0xe860, 0x0000, 0x0080);
	if (ret)
		goto out_restore_pm;

	ret = rtlmq_mac_ocp_modify_checked(tp, RTLMQ_TX_DESC_FORMAT_OCP_REG,
					   RTLMQ_TX_DESC_FORMAT_MASK,
					   RTLMQ_TX_DESC_FORMAT_MASK);
	if (ret)
		goto out_restore_pm;

	ret = rtlmq_mac_ocp_modify_checked(tp, 0xe614, 0x0700, 0x0200);
	if (ret)
		goto out_restore_pm;

	ret = rtlmq_program_8125_vendor_tx_engine(tp, RTLMQ_8125_TX_ENGINE_BOOTSTRAP);
	if (ret)
		goto out_restore_pm;
	ret = rtlmq_wait_mac_ocp_e00e_clear(tp);
	if (ret == -ENODEV)
		goto out_restore_pm;
	if (ret) {
		netdev_err(tp->ndev,
			   "hw_config_8125_common e00e clear wait failed: %d\n",
			   ret);
		goto out_restore_pm;
	}
	ret = rtlmq_program_eee_mac_locked(tp);
	if (ret)
		goto out_restore_pm;
	ret = rtlmq_hw_aspm_clkreq_enable(tp, true, &pm_state);
	if (ret)
		return ret;
	return rtlmq_disable_rxdvgate(tp);

out_restore_pm:
	restore_ret = rtlmq_hw_aspm_clkreq_enable(tp, true, &pm_state);
	if (restore_ret)
		netdev_warn(tp->ndev,
			    "failed to restore ASPM/CLKREQ after setup error: %d\n",
			    restore_ret);
	return ret;
}

int rtlmq_configure_mac_8125b(struct rtlmq_private *tp)
{
	int ret;

	/* K1 latency byte: L0 7 us, L1 16 us; preserve the other bytes. */
	ret = rtlmq_pci_config_modify(tp, RTLMQ_PCI_ASPM_LATENCY,
				      0xff000000, 0x27000000);
	if (ret)
		return ret;

	ret = rtlmq_phy_init_analog(tp);
	if (ret)
		return ret;

	return rtlmq_hw_config_8125_common(tp);
}

int rtlmq_latch_tx_engine_8125(struct rtlmq_private *tp, const char *where)
{
	int ret;

	rtlmq_unlock_config_regs(tp);
	ret = rtlmq_program_8125_vendor_tx_engine(tp, RTLMQ_8125_TX_ENGINE_RUNTIME_LATCH);
	if (ret == -ENODEV)
		return ret;
	if (ret)
		goto out_lock;
	ret = rtlmq_wait_mac_ocp_e00e_clear(tp);
	if (ret == -ENODEV)
		return ret;
out_lock:
	rtlmq_lock_config_regs(tp);
	if (ret) {
		netdev_err(tp->ndev,
			   "%s: tx_engine_latch wait failed: %d\n",
			   where ? where : "runtime_build", ret);
		return ret;
	}

	ret = rtlmq_validate_tx_engine_queue_count_latch(tp, where);
	if (ret)
		return ret;

	return 0;
}

/*
 * Only include RTL8125B registers that are safe to read directly through
 * MMIO. These are passive configuration and status registers already used by
 * the driver or verified on hardware. Leave out interrupt status and mask
 * aliases, counters, doorbells, descriptor DMA addresses, hardware cursors,
 * and indirect OCP, PHY, and EPHY command windows.
 */
static const struct rtlmq_regdump_register rtlmq_8125b_reg_dump[] = {
	{ RTLMQ_INT_CFG0, sizeof(u8) },
	{ RTLMQ_CHIP_CMD, sizeof(u8) },
	{ RTLMQ_TX_CONFIG, sizeof(u32) },
	{ RTLMQ_RX_CONFIG, sizeof(u32) },
	{ RTLMQ_CFG9346, sizeof(u8) },
	{ RTLMQ_CONFIG1, sizeof(u8) },
	{ RTLMQ_CONFIG2, sizeof(u8) },
	{ RTLMQ_CONFIG3, sizeof(u8) },
	{ RTLMQ_CONFIG5, sizeof(u8) },
	{ RTLMQ_PHY_STATUS, sizeof(u8) },
	{ RTLMQ_MCU, sizeof(u8) },
	{ RTLMQ_RX_DESC_MODE, sizeof(u8) },
	{ RTLMQ_RX_MAX_SIZE, sizeof(u16) },
	{ RTLMQ_CPLUS_CMD, sizeof(u16) },
	{ RTLMQ_INTR_MITIGATE, sizeof(u16) },
	{ RTLMQ_INT_MITI_V2_0_TX, sizeof(u16) },
	{ RTLMQ_INT_MITI_V2_1_TX, sizeof(u16) },
	{ RTLMQ_MISC, sizeof(u32) },
	{ RTLMQ_DOUBLE_VLAN_CONFIG, sizeof(u16) },
	{ RTLMQ_TX_QUEUE_CREDIT_BASE, sizeof(u32) },
	{ RTLMQ_TX_QUEUE_CREDIT_BASE + RTLMQ_TX_QUEUE_CREDIT_STRIDE,
	  sizeof(u32) },
	{ RTLMQ_RSS_CTRL, sizeof(u32) },
	{ RTLMQ_Q_NUM_CTRL, sizeof(u16) },
};

static const struct rtlmq_regdump_register rtlmq_8125b_moderation[] = {
	{ RTLMQ_INT_MITI_V2_0_RX, sizeof(u16) },
	{ RTLMQ_INT_MITI_V2_1_RX, sizeof(u16) },
	{ RTLMQ_INT_MITI_V2_0_RX + 2 * RTLMQ_INT_MITI_V2_STRIDE, sizeof(u16) },
	{ RTLMQ_INT_MITI_V2_0_RX + 3 * RTLMQ_INT_MITI_V2_STRIDE, sizeof(u16) },
	{ RTLMQ_INT_MITI_V2_0_TX, sizeof(u16) },
	{ RTLMQ_INT_MITI_V2_1_TX, sizeof(u16) },
};

static_assert(ARRAY_SIZE(rtlmq_8125b_moderation) ==
	      RTLMQ_IMPL_MAX_RX_QUEUES + RTLMQ_IMPL_MAX_TX_QUEUES);

#define RTLMQ_8125_REG_DUMP_ENTRY_COUNT \
	ARRAY_SIZE(rtlmq_8125b_reg_dump)

static_assert(sizeof(struct rtlmq_reg_dump_header) +
	      RTLMQ_8125_REG_DUMP_ENTRY_COUNT *
		      sizeof(struct rtlmq_reg_dump_entry) <= U16_MAX);
static_assert(RTLMQ_8125_REG_DUMP_ENTRY_COUNT <= U16_MAX);
static_assert(RTLMQ_Q_NUM_CTRL + sizeof(u16) <= RTLMQ_MMIO_REQUIRED_SIZE);

const struct rtlmq_regdump_schema rtlmq_8125b_regdump_schema = {
	.registers = rtlmq_8125b_reg_dump,
	.register_count = RTLMQ_8125_REG_DUMP_ENTRY_COUNT,
	.moderation = rtlmq_8125b_moderation,
	.moderation_count = ARRAY_SIZE(rtlmq_8125b_moderation),
};

static const struct rtlmq_chip_match rtlmq_8125b_match = {
	.pci_vendor = PCI_VENDOR_ID_REALTEK,
	.pci_device = 0x8125,
	.xid = 0x641,
	.xid_mask = RTLMQ_TX_CONFIG_XID_FAMILY_MASK,
	.accepted_config_methods = BIT(RTLMQ_CFG_METHOD_4) |
				   BIT(RTLMQ_CFG_METHOD_5),
};

static const struct rtlmq_hw_contract rtlmq_8125b_hw = {
	.mac_version = RTLMQ_MAC_8125B,
	.rss_rx_desc_fmt = RX_DESC_FMT_V3,
	.tx_desc_fmt = RTLMQ_TX_DESC_FORMAT_LEGACY,
	.rss_version = RTLMQ_RSS_VERSION_5,
	.irq_moderation_version = RTLMQ_IRQ_MODERATION_V2,
	.eee_tx_lpi_layout = RTLMQ_EEE_TX_LPI_E040_EB62,
	.max_rx_queues = 4,
	.max_tx_queues = 2,
	.max_irq_vectors = 32,
	.max_runtime_vectors = RTLMQ_IMPL_MAX_NAPI,
	.requires_tx_engine_runtime_latch = true,
	.has_q1_tx_desc_base_regs = true,
	.has_msi = true,
	.has_msix = true,
	.has_rss = true,
	.has_msix_message_routing = true,
	.requires_conservative_aspm = true,
	.msix_rx_message_ids = { 0, 1, 2, 3 },
	.msix_tx_message_ids = { 16, 18 },
	.msix_link_message_id = 21,
	.msix_int_cfg0_clear_mask = 0,
};

/*
 * Admitted shapes in table order: 1RX/1TX, 2RX/1TX, 4RX/1TX, 4RX/2TX,
 * 2RX/2TX. Each profile picks its own default index below; all five remain
 * selectable at runtime.
 */
static const struct rtlmq_topology rtlmq_8125b_topologies[] = {
	{
		.rx_queues = 1,
		.tx_queues = 1,
		.vectors = 1,
		.routing = RTLMQ_ROUTING_SINGLE_VECTOR,
		.irq_modes = BIT(RTLMQ_IRQ_MODE_INTX) |
			     BIT(RTLMQ_IRQ_MODE_MSI),
	},
	{
		.rx_queues = 2,
		.tx_queues = 1,
		.vectors = 2,
		.routing = RTLMQ_ROUTING_RX_PER_VECTOR_TX_COUPLED,
		.irq_modes = BIT(RTLMQ_IRQ_MODE_MSIX),
	},
	{
		.rx_queues = 4,
		.tx_queues = 1,
		.vectors = 4,
		.routing = RTLMQ_ROUTING_RX_PER_VECTOR_TX_COUPLED,
		.irq_modes = BIT(RTLMQ_IRQ_MODE_MSIX),
	},
	{
		.rx_queues = 4,
		.tx_queues = 2,
		.vectors = 5,
		.routing = RTLMQ_ROUTING_RX_PER_VECTOR_TX_COUPLED,
		.irq_modes = BIT(RTLMQ_IRQ_MODE_MSIX),
	},
	{
		.rx_queues = 2,
		.tx_queues = 2,
		.vectors = 2,
		.routing = RTLMQ_ROUTING_RX_PER_VECTOR_TX_COUPLED,
		.irq_modes = BIT(RTLMQ_IRQ_MODE_MSIX),
	},
};

/* Realtek 9.018 uses PHY PTP v3 for CFG_METHOD_4/5. */
#define RTLMQ_8125B_POLICY(_default)				\
	.base_netdev_features = NETIF_F_SG | NETIF_F_IP_CSUM |	\
		NETIF_F_IPV6_CSUM | NETIF_F_RXCSUM |		\
		NETIF_F_TSO | NETIF_F_TSO6,			\
	.max_tso_size = 49152,					\
	.ptp_backend = RTLMQ_PTP_BACKEND_PHY_V3,			\
	.topologies = rtlmq_8125b_topologies,			\
	.default_topology = &rtlmq_8125b_topologies[_default],	\
	.num_topologies = ARRAY_SIZE(rtlmq_8125b_topologies),	\
	.max_xsk_tx_queues = 2,					\
	.use_core_napi_defaults = true,				\
	.supports_rss = true,					\
	.supports_xdp = true,					\
	.supports_af_xdp = true,					\
	.supports_tc = true,					\
	.supports_cbs = true,					\
	.supports_vlan_ctag_tx = true,				\
	.supports_vlan_ctag_rx = true,				\
	.supports_rx_fcs = true,					\
	.supports_rx_all = true,					\
	.supports_highdma = true,				\
	.supports_hw_tally = true,				\
	.supports_live_mac_change = true,				\
	.supports_inplace_ring_resize = true,			\
	.supports_eee = true,					\
	.supports_leds = true,					\
	.wol_opts = RTLMQ_WOL_SUPPORTED_OPTS

static const struct rtlmq_qualified_policy rtlmq_8125b_qualified = {
	RTLMQ_8125B_POLICY(2),		/* 4RX/1TX */
};

#undef RTLMQ_8125B_POLICY

static const struct rtlmq_chip_ops rtlmq_8125b_ops = {
	.prepare_mac = rtlmq_hw_prepare_8125,
	.prepare_reset = rtlmq_prepare_mac_reset,
	.configure_mac = rtlmq_configure_mac_8125b,
	.configure_phy = rtlmq_configure_phy_8125b,
	.latch_tx_engine = rtlmq_latch_tx_engine_8125,
};

const struct rtlmq_chip_profile rtlmq_8125b_profile = {
	.name = "RTL8125B",
	.match = &rtlmq_8125b_match,
	.hw = &rtlmq_8125b_hw,
	.qualified = &rtlmq_8125b_qualified,
	.ops = &rtlmq_8125b_ops,
	.firmware = &rtlmq_8125b_firmware,
	.regdump = &rtlmq_8125b_regdump_schema,
};
